"""
S035 UTM Simulation
====================
A centralised UAS Traffic Management (UTM) service manages 12 delivery drones in a
1000 x 1000 x 120 m urban airspace with 4 directional altitude layers and 16 corridors
(4 parallel centre-lines per layer). Every drone takes off from one of 4 depots that
sit on corridor lines, flies an L-shaped route made only of corridor segments
(first leg east/west in L1/L3, second leg north/south in L2/L4) and lands.

Strategic resolution (ground holds before departure), compared on the same requests:
  FCFS      - intents registered in request order; each drone gets the smallest
              ground hold that makes it conflict-free with all registered intents
  Priority  - same registration, but in order of urgency score u_k (highest first)
  LP        - centralised: minimise sum of holds with a linear programme over
              departure offsets (precedence per conflicting pair, constraint generation)

Execution: drones fly their plans; the separation-assurance monitor looks 15 s ahead
and issues Resolution Advisories (RAs) that make a drone hover / hold until the
predicted encounter clears. Drones on the ground (before take-off or after landing)
are never counted as airborne traffic.

A drone pair is in conflict when horizontal separation < 30 m AND vertical < 10 m.

Usage:
    conda activate drones
    python src/02_logistics_delivery/s035_utm_simulation.py
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from scipy.optimize import linprog
from dataclasses import dataclass

# ── Parameters ─────────────────────────────────────────────────────────────────
N_DRONES        = 12
V_CRUISE        = 10.0     # m/s horizontal cruise speed
V_VERT          = 3.0      # m/s climb / descent / layer-change speed
D_SEP_H         = 30.0     # m horizontal separation minimum
D_SEP_V         = 10.0     # m vertical separation minimum
D_WARN_H        = 45.0     # m RA warning distance (1.5 x D_SEP_H)
T_LOOKAHEAD     = 15.0     # s separation-assurance look-ahead
T_BUFFER        = 2.0      # s extra RA hold buffer
V_CLOSE         = 10.0     # m/s closing speed used for the RA hold time
RHO_MAX         = 3.0      # drones/km corridor capacity (corridors are 1 km long)
DT              = 0.5      # s execution timestep = conflict-scan resolution
T_REQ_WINDOW    = 10.0     # s all 12 requests arrive within [0, T_REQ_WINDOW] (near-simultaneous)
MAX_HOLD        = 400.0    # s largest ground hold considered
SEED            = 7        # request set shown in the plots
N_SEEDS         = 10       # Monte-Carlo request sets (seeds 0 ... 9)
ARENA           = 1000.0   # m
OFFSETS         = [150.0, 350.0, 650.0, 850.0]   # corridor centre-line offsets (m)
# Altitude layers: (z_lo, z_hi, direction)
LAYERS = [(20.0, 40.0, 'E'), (40.0, 60.0, 'N'), (60.0, 80.0, 'W'), (80.0, 100.0, 'S')]
DEPOTS = np.array([[200.0, 150.0], [800.0, 350.0], [300.0, 650.0], [700.0, 850.0]])
# Priority score u_k = ALPHA_U / slack + BETA_U * emergency + GAMMA_U * payload
ALPHA_U, BETA_U, GAMMA_U = 100.0, 1.0, 0.3

STRATEGIES = ['FCFS', 'Priority', 'LP']
S_COLORS = {'None': '#9e9e9e', 'RA only': '#bdbdbd', 'FCFS': '#4FC3F7',
            'Priority': '#81C784', 'LP': '#FFB74D'}
LAYER_COLORS = ['#4FC3F7', '#81C784', '#FFB74D', '#CE93D8']

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '02_logistics_delivery', 's035_utm_simulation',
)


# ── Airspace & intents ─────────────────────────────────────────────────────────

def layer_z(direction):
    for z_lo, z_hi, d in LAYERS:
        if d == direction:
            return 0.5 * (z_lo + z_hi)


@dataclass
class Intent:
    id: int
    t_req: float              # requested departure time (s)
    waypoints: np.ndarray     # (m, 3)
    seg_corridor: list        # corridor id per segment (-1 = vertical transition)
    samples: np.ndarray       # positions every DT from take-off to landing (n, 3)
    sample_corr: np.ndarray   # corridor id per sample
    urgency: float
    deadline: float
    emergency: bool
    payload: float

    @property
    def duration(self):
        return (len(self.samples) - 1) * DT


def corridor_id(direction, offset):
    layer = 'ENWS'.index(direction)
    return layer * 4 + OFFSETS.index(offset)


def sample_route(wps, seg_corr):
    """Positions every DT along the route (vertical legs at V_VERT, horizontal at V_CRUISE)."""
    seg_t = []
    for a, b in zip(wps[:-1], wps[1:]):
        dh = np.linalg.norm(b[:2] - a[:2]); dz = abs(b[2] - a[2])
        seg_t.append(dh / V_CRUISE if dh > 0 else dz / V_VERT)
    T = np.cumsum([0.0] + seg_t)
    n = int(np.ceil(T[-1] / DT)) + 1
    ts = np.minimum(np.arange(n) * DT, T[-1])
    pos = np.zeros((n, 3)); corr = np.full(n, -1)
    for i, t in enumerate(ts):
        s = min(np.searchsorted(T, t, side='right') - 1, len(seg_t) - 1)
        f = (t - T[s]) / max(seg_t[s], 1e-9)
        pos[i] = wps[s] + f * (wps[s + 1] - wps[s])
        corr[i] = seg_corr[s]
    return pos, corr


def build_intents(seed=SEED):
    rng = np.random.default_rng(seed)
    intents = []
    t_req = np.round(rng.uniform(0, T_REQ_WINDOW, N_DRONES) / DT) * DT
    emergency_idx = int(rng.integers(N_DRONES))
    for k in range(N_DRONES):
        ox, oy = DEPOTS[k % 4]
        while True:
            xc = OFFSETS[int(rng.integers(4))]
            yd = rng.uniform(100, 900)
            if abs(xc - ox) >= 100 and abs(yd - oy) >= 100:
                break
        dx = 'E' if xc > ox else 'W'
        dy = 'N' if yd > oy else 'S'
        zx, zy = layer_z(dx), layer_z(dy)
        wps = np.array([[ox, oy, 0.0], [ox, oy, zx], [xc, oy, zx],
                        [xc, oy, zy], [xc, yd, zy], [xc, yd, 0.0]])
        seg_corr = [-1, corridor_id(dx, oy), -1, corridor_id(dy, xc), -1]
        samples, scorr = sample_route(wps, seg_corr)
        dur = (len(samples) - 1) * DT
        slack = rng.uniform(60, 300)
        payload = rng.uniform(0.5, 2.5)
        emergency = (k == emergency_idx)
        deadline = t_req[k] + dur + slack
        u = ALPHA_U / slack + BETA_U * emergency + GAMMA_U * payload
        intents.append(Intent(k, float(t_req[k]), wps, seg_corr, samples, scorr,
                              u, deadline, emergency, payload))
    return intents


# ── Pairwise conflict geometry ─────────────────────────────────────────────────

def forbidden_shifts(intents, d_h=D_SEP_H):
    """F[(k, j)] = set of integer s (units of DT) such that delaying j by s*DT relative
    to k (s = d_j - d_k) makes the two *airborne* trajectories conflict."""
    a = {it.id: int(round(it.t_req / DT)) for it in intents}
    F = {}
    for ik, k in enumerate(intents):
        for j in intents[ik + 1:]:
            Pk, Pj = k.samples, j.samples
            dh = np.linalg.norm(Pk[:, None, :2] - Pj[None, :, :2], axis=2)
            dv = np.abs(Pk[:, None, 2] - Pj[None, :, 2])
            ii, jj = np.nonzero((dh < d_h) & (dv < D_SEP_V))
            # k sample i at global index a_k + d_k + i equals j sample i' at a_j + d_j + i'
            # -> s = d_j - d_k = a_k - a_j + i - i'
            F[(k.id, j.id)] = set((a[k.id] - a[j.id] + ii - jj).tolist())
    return F


def in_conflict(F, d, k, j):
    """True if intents k < j conflict with integer holds d (units of DT)."""
    return (d[j] - d[k]) in F[(k, j)]


def conflict_pairs(F, d):
    return sorted(p for p in F if in_conflict(F, d, *p))


# ── Strategic resolution ───────────────────────────────────────────────────────

def register_sequential(F, order, intents):
    """Register intents one by one in `order`; each gets the smallest hold that is
    conflict-free with all intents registered before it AND keeps every corridor at
    <= RHO_MAX drones (corridors are 1 km long, so density = count per km)."""
    a = {it.id: int(round(it.t_req / DT)) for it in intents}
    horizon = int((T_REQ_WINDOW + MAX_HOLD) / DT) + max(len(it.samples) for it in intents) + 2
    occ = np.zeros((16, horizon), dtype=int)
    d, reg = {}, []
    for j in order:
        sc = intents[j].sample_corr
        on = np.flatnonzero(sc >= 0)
        for h in range(int(MAX_HOLD / DT) + 1):
            d[j] = h
            if not all((h - d[k]) not in F[(k, j)] if k < j else (d[k] - h) not in F[(j, k)]
                       for k in reg):
                continue
            if np.any(occ[sc[on], a[j] + h + on] + 1 > RHO_MAX):
                continue
            break
        occ[sc[on], a[j] + d[j] + on] += 1
        reg.append(j)
    return d


def resolve_fcfs(intents, F):
    order = [it.id for it in sorted(intents, key=lambda it: (it.t_req, it.id))]
    return register_sequential(F, order, intents), order


def resolve_priority(intents, F):
    order = [it.id for it in sorted(intents, key=lambda it: (-it.urgency, it.id))]
    return register_sequential(F, order, intents), order


def resolve_lp(intents, F, max_iter=60):
    """min sum d_k s.t. pairwise precedence constraints (constraint generation).
    For a conflicting pair the cheaper side is chosen: j after k (s >= max F + 1)
    or j before k (s <= min F - 1), measured from the current relative shift."""
    n = len(intents)
    cons = {}                     # (k, j) -> ('after', bound) or ('before', bound)
    d = {i: 0 for i in range(n)}
    lp_iters = 0
    for _ in range(max_iter):
        bad = conflict_pairs(F, d)
        if not bad:
            break
        for (k, j) in bad:
            s = d[j] - d[k]
            lo, hi = min(F[(k, j)]), max(F[(k, j)])
            cons[(k, j)] = ('after', hi + 1) if (hi + 1 - s) <= (s - (lo - 1)) else ('before', lo - 1)
        A, b = [], []
        for (k, j), (side, bound) in cons.items():
            row = np.zeros(n)
            if side == 'after':          # d_j - d_k >= bound  ->  d_k - d_j <= -bound
                row[k], row[j] = 1.0, -1.0; b.append(-bound)
            else:                        # d_j - d_k <= bound
                row[k], row[j] = -1.0, 1.0; b.append(bound)
            A.append(row)
        res = linprog(np.ones(n), A_ub=np.array(A), b_ub=np.array(b),
                      bounds=[(0, None)] * n, method='highs')
        lp_iters += 1
        d = {i: int(round(res.x[i])) for i in range(n)}
    order = [it.id for it in sorted(intents, key=lambda it: (it.t_req + d[it.id] * DT, it.id))]
    return d, order, lp_iters, len(cons)


# ── Execution with separation assurance ────────────────────────────────────────

def execute(intents, d, order, use_ra=True):
    """Fly all plans (ground holds d, in DT units). The monitor predicts each pair over the
    next T_LOOKAHEAD s; if predicted horizontal separation < D_WARN_H while vertical
    separation < D_SEP_V, an RA holds one drone (hover if airborne, stay on the ground if
    not) for t_RA = (D_WARN_H - d_pred)/V_CLOSE + T_BUFFER. The drone later in `order`
    is asked first; if holding it does not clear the encounter the other drone holds."""
    n = len(intents)
    rank = {k: r for r, k in enumerate(order)}
    a = np.array([int(round(it.t_req / DT)) + d[it.id] for it in intents])   # departure idx
    nsamp = np.array([len(it.samples) for it in intents])
    hold = np.zeros(n, dtype=int)            # accumulated RA hold (steps)
    hold_until = np.zeros(n, dtype=int)      # RA active until this step (exclusive)
    la = int(T_LOOKAHEAD / DT)
    hist, air_hist = [], []
    ra_events, ra_pairs_prev = [], set()
    los_pairs, los_events = set(), []
    min_sep_h = np.inf
    land_step = np.full(n, -1)
    step = 0

    def tau(k, st):
        return st - a[k] - hold[k]

    def pos_at(k, ta):
        return intents[k].samples[int(np.clip(ta, 0, nsamp[k] - 1))]

    def predicted_min(k, j, frozen=None, frozen_steps=0):
        """Min horizontal sep (with dv < D_SEP_V, both airborne) over the look-ahead."""
        best = np.inf
        for q in range(0, la + 1, 2):
            tk = tau(k, step) + q - (min(q, frozen_steps) if frozen == k else 0)
            tj = tau(j, step) + q - (min(q, frozen_steps) if frozen == j else 0)
            if not (0 <= tk < nsamp[k] and 0 <= tj < nsamp[j]):
                continue
            pk, pj = pos_at(k, tk), pos_at(j, tj)
            if abs(pk[2] - pj[2]) < D_SEP_V:
                best = min(best, float(np.linalg.norm(pk[:2] - pj[:2])))
        return best

    while True:
        taus = np.array([tau(k, step) for k in range(n)])
        landed = taus >= nsamp - 1
        for k in np.flatnonzero(landed & (land_step < 0)):
            land_step[k] = step
        if landed.all():
            break
        airborne = (taus >= 0) & ~landed
        P = np.array([pos_at(k, taus[k]) for k in range(n)])
        hist.append(P.copy()); air_hist.append(airborne.copy())

        # actual separation (airborne pairs only)
        idx = np.flatnonzero(airborne)
        for ii, k in enumerate(idx):
            for j in idx[ii + 1:]:
                if abs(P[k, 2] - P[j, 2]) < D_SEP_V:
                    dh = float(np.linalg.norm(P[k, :2] - P[j, :2]))
                    min_sep_h = min(min_sep_h, dh)
                    if dh < D_SEP_H and (k, j) not in los_pairs:
                        los_pairs.add((k, j)); los_events.append((step * DT, k, j, P[k].copy()))

        # separation assurance
        ra_now = set()
        if use_ra:
            active = np.flatnonzero(~landed)
            for ii, k in enumerate(active):
                for j in active[ii + 1:]:
                    dmin = predicted_min(k, j)
                    if dmin < D_WARN_H:
                        ra_now.add((k, j))
                        t_ra = int(np.ceil(((D_WARN_H - dmin) / V_CLOSE + T_BUFFER) / DT))
                        first, second = (j, k) if rank[j] > rank[k] else (k, j)

                        def hold_ok(w):
                            # holding w must clear this encounter and not create a new one
                            return all(predicted_min(min(w, o), max(w, o), frozen=w,
                                                     frozen_steps=t_ra) >= D_SEP_H
                                       for o in active if o != w)
                        who = first if hold_ok(first) else (second if hold_ok(second) else first)
                        hold_until[who] = max(hold_until[who], step + t_ra)
            for (k, j) in sorted(ra_now - ra_pairs_prev):
                ra_events.append((step * DT, k, j))
            ra_pairs_prev = ra_now
        # advance: a drone under an active RA does not progress along its plan
        for k in range(n):
            if not landed[k] and step < hold_until[k]:
                hold[k] += 1
        step += 1
        if step > 20000:
            break

    arrival = land_step * DT
    nominal = np.array([it.t_req + it.duration for it in intents])
    return {
        'hist': np.array(hist), 'air': np.array(air_hist), 'arrival': arrival,
        'delay': arrival - nominal, 'ground_hold': np.array([d[k] * DT for k in range(n)]),
        'ra_hold': hold * DT, 'ra_events': ra_events, 'los_events': los_events,
        'min_sep_h': min_sep_h, 'depart': a * DT,
    }


def corridor_occupancy(intents, res):
    """Number of drones on each corridor (centre-line segment, airborne) per step."""
    n_steps = len(res['hist'])
    occ = np.zeros((16, n_steps), dtype=int)
    for s in range(n_steps):
        for k, it in enumerate(intents):
            if not res['air'][s, k]:
                continue
            p = res['hist'][s, k]
            i = int(np.argmin(np.linalg.norm(it.samples - p, axis=1)))
            c = it.sample_corr[i]
            if c >= 0:
                occ[c, s] += 1
    return occ


# ── Simulation driver ──────────────────────────────────────────────────────────

def summarise(name, intents, res, planned_conflicts):
    n = len(intents)
    t_first = min(it.t_req for it in intents)
    t_mission = float(np.max(res['arrival'])) - t_first
    occ = corridor_occupancy(intents, res)
    return {
        'name': name, 'planned_conflicts': planned_conflicts,
        'los': len(res['los_events']), 'min_sep_h': res['min_sep_h'],
        'n_ra': len(res['ra_events']), 'ra_hold': float(np.sum(res['ra_hold'])),
        'ground_hold': float(np.sum(res['ground_hold'])),
        'total_delay': float(np.sum(res['delay'])), 'mean_delay': float(np.mean(res['delay'])),
        'max_delay': float(np.max(res['delay'])),
        'throughput': n / t_mission * 60.0, 't_mission': t_mission,
        'conflict_rate': planned_conflicts / (n * (n - 1) / 2 * t_mission / 60.0),
        'max_occ': int(occ.max()), 'occ': occ,
        'late': int(sum(res['arrival'][k] > intents[k].deadline for k in range(n))),
    }


def run_simulation(seed=SEED):
    intents = build_intents(seed)
    F = forbidden_shifts(intents, D_SEP_H)          # separation minima (conflict counting)
    Fp = forbidden_shifts(intents, D_WARN_H)        # planning buffer: strategic plans clear d_warn
    d0 = {k: 0 for k in range(N_DRONES)}
    raw = conflict_pairs(F, d0)
    req_order = [it.id for it in sorted(intents, key=lambda it: (it.t_req, it.id))]

    runs, summ, plans = {}, {}, {}
    runs['None'] = execute(intents, d0, req_order, use_ra=False)
    summ['None'] = summarise('None', intents, runs['None'], len(raw))
    runs['RA only'] = execute(intents, d0, req_order, use_ra=True)
    summ['RA only'] = summarise('RA only', intents, runs['RA only'], len(raw))
    for nm in ('None', 'RA only'):
        summ[nm]['planned_warnings'] = len(conflict_pairs(Fp, d0))

    d_f, o_f = resolve_fcfs(intents, Fp)
    d_p, o_p = resolve_priority(intents, Fp)
    d_l, o_l, lp_iters, n_cons = resolve_lp(intents, Fp)
    for name, d, o in [('FCFS', d_f, o_f), ('Priority', d_p, o_p), ('LP', d_l, o_l)]:
        plans[name] = (d, o)
        runs[name] = execute(intents, d, o, use_ra=True)
        summ[name] = summarise(name, intents, runs[name], len(conflict_pairs(F, d)))
        summ[name]["planned_warnings"] = len(conflict_pairs(Fp, d))
    return intents, F, raw, runs, summ, plans, (lp_iters, n_cons)


# ── Plots ──────────────────────────────────────────────────────────────────────

def _drone_colors():
    return plt.get_cmap('tab20')(np.linspace(0, 1, N_DRONES))


def plot_3d_airspace(intents, runs, raw, out_dir):
    """Corridors, flown paths (LP plan) and pre-resolution conflict points."""
    fig = plt.figure(figsize=(13, 9))
    ax = fig.add_subplot(111, projection='3d')
    for li, (z_lo, z_hi, dr) in enumerate(LAYERS):
        z = 0.5 * (z_lo + z_hi)
        for off in OFFSETS:
            if dr in 'EW':
                ax.plot([0, ARENA], [off, off], [z, z], color=LAYER_COLORS[li], alpha=0.3, lw=1.5)
            else:
                ax.plot([off, off], [0, ARENA], [z, z], color=LAYER_COLORS[li], alpha=0.3, lw=1.5)
    cols = _drone_colors()
    res = runs['LP']
    for k, it in enumerate(intents):
        m = res['air'][:, k]
        tr = res['hist'][m, k]
        ax.plot(tr[:, 0], tr[:, 1], tr[:, 2], color=cols[k], lw=1.4)
        ax.scatter(*it.waypoints[0], color=cols[k], marker='s', s=25)
        ax.scatter(*it.waypoints[-1], color=cols[k], marker='*', s=60)
    for (t, k, j, p) in runs['None']['los_events']:
        ax.scatter(*p, color='red', s=90, alpha=0.8)
    ax.set_xlim(0, ARENA); ax.set_ylim(0, ARENA); ax.set_zlim(0, 120)
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
    ax.set_title('S035 UTM - corridors, flown paths (LP plan) and conflict points '
                 'of the unresolved plan (red)', fontsize=11)
    handles = [mpatches.Patch(color='red', label=f'Unresolved conflicts ({len(raw)})')]
    handles += [mpatches.Patch(color=LAYER_COLORS[i], label=f'L{i+1} {lo:.0f}-{hi:.0f} m ({d})')
                for i, (lo, hi, d) in enumerate(LAYERS)]
    ax.legend(handles=handles, loc='upper left', fontsize=8)
    path = os.path.join(out_dir, '3d_airspace.png')
    plt.savefig(path, dpi=130, bbox_inches='tight'); plt.close()
    print(f'Saved: {path}')


def plot_conflict_timeline(intents, runs, out_dir):
    """Per-drone mission bars (request, ground hold, flight) for unresolved vs LP plans."""
    fig, axes = plt.subplots(1, 2, figsize=(16, 6), sharey=True)
    for ax, name in zip(axes, ['RA only', 'LP']):
        res = runs[name]
        for k, it in enumerate(intents):
            ax.barh(k, res['depart'][k] - it.t_req, left=it.t_req, color='#bdbdbd', height=0.5)
            ax.barh(k, res['arrival'][k] - res['depart'][k], left=res['depart'][k],
                    color='steelblue', height=0.5)
        for (t, k, j, p) in runs['None']['los_events'] if name == 'RA only' else []:
            ax.plot([t, t], [k, j], color='red', lw=1.5)
        for (t, k, j) in res['ra_events']:
            ax.scatter([t, t], [k, j], color='orange', marker='D', s=18, zorder=5)
        ax.set_xlabel('Time (s)'); ax.grid(axis='x', alpha=0.3)
        ax.set_title(f"{name}: grey = ground hold, blue = request-to-landing flight,\n"
                     f"orange = RA event, red = conflict of the unresolved plan", fontsize=10)
    axes[0].set_ylabel('Drone'); axes[0].set_yticks(range(N_DRONES))
    fig.tight_layout()
    path = os.path.join(out_dir, 'conflict_timeline.png')
    plt.savefig(path, dpi=130); plt.close()
    print(f'Saved: {path}')


def plot_separation_histogram(runs, out_dir):
    """Per-pair minimum horizontal separation (airborne, |dz| < 10 m) for each run."""
    names = ['None', 'RA only'] + STRATEGIES
    fig, axes = plt.subplots(1, len(names), figsize=(20, 4), sharey=True)
    for ax, name in zip(axes, names):
        res = runs[name]
        H, A = res['hist'], res['air']
        mins = []
        for k in range(N_DRONES):
            for j in range(k + 1, N_DRONES):
                m = A[:, k] & A[:, j] & (np.abs(H[:, k, 2] - H[:, j, 2]) < D_SEP_V)
                if m.any():
                    mins.append(float(np.min(np.linalg.norm(H[m, k, :2] - H[m, j, :2], axis=1))))
        ax.hist(mins, bins=np.arange(0, 400, 10), color=S_COLORS[name], edgecolor='white')
        ax.axvline(D_SEP_H, color='red', ls='--', label='d_sep_H 30 m')
        ax.axvline(D_WARN_H, color='orange', ls='--', label='d_warn_H 45 m')
        ax.set_title(f'{name} ({len(mins)} pairs share a layer)', fontsize=10)
        ax.set_xlabel('Min horizontal separation (m)')
    axes[0].set_ylabel('Drone pairs'); axes[0].legend(fontsize=7)
    fig.suptitle('Per-pair minimum horizontal separation while both airborne and |dz| < 10 m',
                 fontsize=12)
    fig.tight_layout()
    path = os.path.join(out_dir, 'separation_histogram.png')
    plt.savefig(path, dpi=130); plt.close()
    print(f'Saved: {path}')


def plot_delay_comparison(summ, out_dir):
    names = ['None', 'RA only'] + STRATEGIES
    x = np.arange(len(names))
    fig, axes = plt.subplots(1, 3, figsize=(17, 4.8))
    gh = [summ[n]['ground_hold'] for n in names]
    rh = [summ[n]['ra_hold'] for n in names]
    axes[0].bar(x, gh, color='#90A4AE', label='strategic ground holds')
    axes[0].bar(x, rh, bottom=gh, color='#FF8A65', label='RA holds')
    for xi, (g, r) in enumerate(zip(gh, rh)):
        axes[0].text(xi, g + r + 2, f'{g + r:.1f}', ha='center', fontsize=9)
    axes[0].set_title('Total fleet delay (s)'); axes[0].legend(fontsize=8)
    axes[0].set_ylim(0, max(g + r for g, r in zip(gh, rh)) * 1.15)
    for ax, key, title in [(axes[1], 'los', 'Losses of separation (pairs)'),
                           (axes[2], 'n_ra', 'RA events')]:
        v = [summ[n][key] for n in names]
        b = ax.bar(x, v, color=[S_COLORS[n] for n in names])
        ax.bar_label(b, fontsize=9); ax.set_title(title)
    for ax in axes:
        ax.set_xticks(x); ax.set_xticklabels(names); ax.grid(axis='y', alpha=0.3)
    fig.suptitle('S035 - delay, safety and RA workload per strategy', fontsize=12)
    fig.tight_layout()
    path = os.path.join(out_dir, 'delay_comparison.png')
    plt.savefig(path, dpi=130); plt.close()
    print(f'Saved: {path}')


def plot_corridor_utilisation(summ, out_dir):
    fig, axes = plt.subplots(1, 3, figsize=(18, 5), sharey=True)
    for ax, name in zip(axes, STRATEGIES):
        occ = summ[name]['occ']
        im = ax.imshow(occ, aspect='auto', origin='lower', cmap='YlOrRd', vmin=0, vmax=RHO_MAX,
                       extent=[0, occ.shape[1] * DT, -0.5, 15.5], interpolation='nearest')
        for i in range(1, 4):
            ax.axhline(i * 4 - 0.5, color='k', lw=0.8)
        ax.set_title(f"{name}: max {summ[name]['max_occ']} drones on one 1-km corridor "
                     f"(limit {RHO_MAX:.0f})", fontsize=10)
        ax.set_xlabel('Time (s)')
    axes[0].set_ylabel('Corridor id (0-3 L1 E, 4-7 L2 N, 8-11 L3 W, 12-15 L4 S)')
    fig.colorbar(im, ax=axes, label='drones/km')
    path = os.path.join(out_dir, 'corridor_utilisation.png')
    plt.savefig(path, dpi=130, bbox_inches='tight'); plt.close()
    print(f'Saved: {path}')


def save_animation(intents, runs, out_dir):
    """Top-down: unresolved (RA only) vs LP plan side by side, <= 150 frames."""
    import matplotlib.animation as animation
    names = ['None', 'LP']
    n = max(len(runs[nm]['hist']) for nm in names)
    frames = list(range(0, n, max(1, int(np.ceil(n / 150)))))
    cols = _drone_colors()
    fig, axes = plt.subplots(1, 2, figsize=(12, 6.3))
    dots, rings = {}, {}
    for ax, nm in zip(axes, names):
        ax.set_xlim(0, ARENA); ax.set_ylim(0, ARENA); ax.set_aspect('equal')
        ax.set_facecolor('#1a1a2e'); ax.set_xticks([]); ax.set_yticks([])
        for li, (_, _, dr) in enumerate(LAYERS):
            for off in OFFSETS:
                xy = ([0, ARENA], [off, off]) if dr in 'EW' else ([off, off], [0, ARENA])
                ax.plot(*xy, color=LAYER_COLORS[li], alpha=0.2, lw=1.2)
        ax.plot(DEPOTS[:, 0], DEPOTS[:, 1], 'ws', ms=6)
        dots[nm] = [ax.plot([], [], 'o', color=cols[k], ms=6)[0] for k in range(N_DRONES)]
        rings[nm] = ax.plot([], [], 'o', mfc='none', mec='red', ms=22, mew=2)[0]
        ax.set_title('Unresolved plan, no RA' if nm == 'None' else 'LP plan + RA monitor',
                     fontsize=10)
    ttl = fig.suptitle('')

    def update(fi):
        f = frames[fi]
        ttl.set_text(f'S035 UTM  t = {f * DT:.0f} s  (red ring = loss of separation)')
        out = [ttl]
        for nm in names:
            H, A = runs[nm]['hist'], runs[nm]['air']
            i = min(f, len(H) - 1)
            rx, ry = [], []
            for k in range(N_DRONES):
                if A[i, k]:
                    dots[nm][k].set_data([H[i, k, 0]], [H[i, k, 1]])
                else:
                    dots[nm][k].set_data([], [])
            idx = np.flatnonzero(A[i])
            for a_, k in enumerate(idx):
                for j in idx[a_ + 1:]:
                    if abs(H[i, k, 2] - H[i, j, 2]) < D_SEP_V and \
                            np.linalg.norm(H[i, k, :2] - H[i, j, :2]) < D_SEP_H:
                        rx.append(H[i, k, 0]); ry.append(H[i, k, 1])
            rings[nm].set_data(rx, ry)
            out += dots[nm] + [rings[nm]]
        return out

    ani = animation.FuncAnimation(fig, update, frames=len(frames), interval=60, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer='pillow', fps=20, dpi=70)
    plt.close()
    print(f'Saved: {path}  ({len(frames)} frames)')


# ── Entry point ────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    out_dir = os.path.normpath(OUTPUT_DIR)
    os.makedirs(out_dir, exist_ok=True)
    intents, F, raw, runs, summ, plans, (lp_iters, n_cons) = run_simulation()

    print('Intents (seed 7):')
    for it in intents:
        print(f'  D{it.id:2d}: req {it.t_req:5.1f} s  depot ({it.waypoints[0][0]:.0f},{it.waypoints[0][1]:.0f})'
              f' -> ({it.waypoints[-1][0]:.0f},{it.waypoints[-1][1]:.0f})  corridors {it.seg_corridor[1]},{it.seg_corridor[3]}'
              f'  flight {it.duration:5.1f} s  u={it.urgency:.2f}{"  EMERGENCY" if it.emergency else ""}')
    print(f'\nUnresolved plan: {len(raw)} conflicting pairs: ' + ', '.join(f'D{k}-D{j}' for k, j in raw))
    print(f'LP: {lp_iters} LP solves, {n_cons} pairwise precedence constraints')
    for name in STRATEGIES:
        d, o = plans[name]
        print(f'  {name:8s} ground holds (s): ' + ' '.join(f'{d[k] * DT:.1f}' for k in range(N_DRONES)))

    print('\n=== Results ===')
    for name in ['None', 'RA only'] + STRATEGIES:
        s = summ[name]
        print(f"  {name:8s} planned_conflicts={s['planned_conflicts']:2d}  planned_warnings={s['planned_warnings']:2d}  LoS={s['los']:2d}  "
              f"min_sep_H={s['min_sep_h']:6.1f} m  RA_events={s['n_ra']:2d}  "
              f"ground_hold={s['ground_hold']:6.1f} s  RA_hold={s['ra_hold']:6.1f} s  "
              f"total_delay={s['total_delay']:6.1f} s  mean_delay={s['mean_delay']:5.1f} s  "
              f"max_delay={s['max_delay']:5.1f} s  throughput={s['throughput']:.3f}/min  "
              f"conflict_rate={s['conflict_rate']:.4f}/pair/min  max_corridor_occ={s['max_occ']}  "
              f"late={s['late']}")

    print(f'\n=== Monte Carlo over {N_SEEDS} request sets (seeds 0..{N_SEEDS - 1}) ===')
    mc = {nm: [] for nm in ['None', 'RA only'] + STRATEGIES}
    for sd in range(N_SEEDS):
        _, _, raw_s, _, summ_s, _, _ = run_simulation(sd)
        for nm in mc:
            mc[nm].append(summ_s[nm])
    for nm, rows in mc.items():
        print(f"  {nm:8s} planned_conflicts={np.mean([r['planned_conflicts'] for r in rows]):5.1f}  "
              f"LoS={np.mean([r['los'] for r in rows]):5.1f}  RA_events={np.mean([r['n_ra'] for r in rows]):5.1f}  "
              f"total_delay={np.mean([r['total_delay'] for r in rows]):6.1f} s "
              f"(+/- {np.std([r['total_delay'] for r in rows]):5.1f})  "
              f"max_corridor_occ={max(r['max_occ'] for r in rows)}  "
              f"sets_over_capacity={sum(r['max_occ'] > RHO_MAX for r in rows)}/{N_SEEDS}")
    lp_better = sum(l['total_delay'] < f_['total_delay'] - 1e-9 for l, f_ in zip(mc['LP'], mc['FCFS']))
    lp_equal = sum(abs(l['total_delay'] - f_['total_delay']) < 1e-9 for l, f_ in zip(mc['LP'], mc['FCFS']))
    print(f'  LP total delay < FCFS in {lp_better}/{N_SEEDS} sets, equal in {lp_equal}/{N_SEEDS}')

    plot_3d_airspace(intents, runs, raw, out_dir)
    plot_conflict_timeline(intents, runs, out_dir)
    plot_separation_histogram(runs, out_dir)
    plot_delay_comparison(summ, out_dir)
    plot_corridor_utilisation(summ, out_dir)
    save_animation(intents, runs, out_dir)
