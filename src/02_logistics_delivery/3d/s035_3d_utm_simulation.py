"""
S035 3D Upgrade - UTM Simulation
=================================
A centralised UTM service manages 12 delivery drones in a 1000 x 1000 x 150 m
airspace with 6 altitude layers (card): L1 E 20-40 m, L2 N 40-60 m, L3 W 60-80 m,
L4 S 80-100 m, L5 "any heading" express 100-120 m, L6 emergency 120-150 m.
Corridor tubes (radius r_c = 25 m) run along 4 centre-lines per axis.  The
request sets (depots, destinations, request times) are the 2D S035 ones.

Each drone flies an L-shaped route: vertical take-off at its depot, an east/west
leg, an inclined layer-transition ramp centred on the corner and flown at exactly
gamma_max = 20 deg, a north/south leg and a vertical landing.  The default
layers are the heading layers (L1/L3 for the E/W leg, L2/L4 for the N/S leg);
each leg may instead use L5 (all drones) or L6 (emergency drone only).  Options
whose ramp would not fit on the legs at gamma <= gamma_max are rejected at
registration.

Separation is a single 3D sphere: ||p_k - p_j|| >= r_sep = 35 m (card), instead
of the 2D pair "horizontal < 30 m AND vertical < 10 m".

Strategic resolution (same requests, same trajectories, sphere conflict sets with
the r_warn = 52.5 m warning sphere as planning buffer):
  FCFS (2D rule)  - 2D S035 FCFS planned with the 2D cylinder rule (45 m / 10 m)
  FCFS time-only  - FCFS ground holds, default layers, sphere rule (card strategy 1)
  Altitude-first  - FCFS order; a conflicting drone first takes an alternate layer
                    option at zero hold (if its extra flight time <= T_THRESH),
                    otherwise the cheapest (hold + extra time) option (strategy 2)
  Joint LP        - layer options chosen by local search, ground holds for each
                    layer assignment by the 2D precedence LP (HiGHS) (strategy 3)

Execution: every 0.5 s the separation-assurance monitor predicts every pair along
the plans over T_la = 15 s.  A predicted 3D distance < r_warn raises an RA:
  hover mode    - hold one drone (2D S035 RA)
  vertical mode - first command a vertical offset dz_cmd (card formula), flown at
                  dz/dt = v tan(gamma_max); fall back to a hover hold if no offset
                  clears the encounter.

Outputs (outputs/02_logistics_delivery/3d/s035_3d_utm_simulation/):
  airspace_3d.png, altitude_timeseries.png, separation_histogram.png,
  delay_comparison.png, corridor_utilisation.png, animation.gif

Usage:
    conda activate drones
    python src/02_logistics_delivery/3d/s035_3d_utm_simulation.py
"""

import matplotlib
matplotlib.use('Agg')

import os
import sys
import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.lines import Line2D
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from scipy.optimize import linprog
from dataclasses import dataclass, field

# ── Parameters (card unless marked) ─────────────────────────────────────────────
N_DRONES     = 12
ARENA        = 1000.0      # m horizontal extent
Z_CEIL       = 150.0       # m airspace ceiling
Z_FLOOR_AIR  = 20.0        # m lowest altitude for a vertical-offset RA (bottom of L1)
V_CRUISE     = 10.0        # m/s horizontal cruise speed
V_VERT       = 3.0         # m/s vertical take-off / landing speed (2D S035)
GAMMA_MAX    = np.radians(20.0)   # max climb / descent angle
TAN_G        = np.tan(GAMMA_MAX)
VZ_TRANS     = V_CRUISE * TAN_G    # 3.64 m/s transition climb rate (card)
R_SEP        = 35.0        # m 3D separation sphere
R_WARN       = 1.5 * R_SEP # m 3D warning sphere (52.5 m)
R_TUBE       = 25.0        # m corridor tube radius
T_LA         = 15.0        # s look-ahead
DZ_MIN       = 5.0         # m minimum commanded altitude step
T_BUFFER     = 2.0         # s RA hold buffer
V_CLOSE      = 10.0        # m/s closing speed for the hover-RA hold time (2D S035)
RHO_MAX      = 3.0         # drones/km corridor capacity (tubes are 1 km long)
DT           = 0.5         # s execution step = conflict-scan resolution
T_REQ_WINDOW = 10.0        # s requests within [0, 10] s (2D S035)
MAX_HOLD     = 400.0       # s largest ground hold considered (2D S035)
T_THRESH     = 30.0        # s altitude-first: max extra flight time of an altitude option (added)
T_STUCK      = 60.0        # s watchdog: max continuous hover before release (added)
T_IMMUNE     = 10.0        # s released drone cannot be held again for this long (added)
RAMP_MARGIN  = 10.0        # m a ramp half must end >= 10 m before the leg end (added)
D_SEP_H_2D, D_SEP_V_2D, D_WARN_H_2D = 30.0, 10.0, 45.0   # 2D S035 cylinder rule
SEED         = 7           # request set in the plots (2D S035)
N_SEEDS      = 20          # Monte-Carlo request sets (seeds 0 .. 19)
LS_PASSES    = 3           # joint LP: local-search passes over drones (added)
ALPHA_U, BETA_U, GAMMA_U = 100.0, 1.0, 0.3   # urgency weights (2D S035, deadlines only)

OFFSETS = [150.0, 350.0, 650.0, 850.0]       # tube centre-line offsets (m)
# layer: (z_lo, z_hi, heading); heading '*' = any (express), '!' = emergency
LAYERS = [(20.0, 40.0, 'E'), (40.0, 60.0, 'N'), (60.0, 80.0, 'W'),
          (80.0, 100.0, 'S'), (100.0, 120.0, '*'), (120.0, 150.0, '!')]
ZMID = [0.5 * (lo + hi) for lo, hi, _ in LAYERS]
DEPOTS = np.array([[200.0, 150.0], [800.0, 350.0], [300.0, 650.0], [700.0, 850.0]])
LAYER_COLORS = ['#4FC3F7', '#81C784', '#FFB74D', '#CE93D8', '#E57373', '#A1887F']

# tubes: (layer, axis, offset); axis 'x' = runs along x at y = offset
TUBES = []
for _li, (_lo, _hi, _hd) in enumerate(LAYERS):
    _axes = ['x'] if _hd in 'EW' else (['y'] if _hd in 'NS' else ['x', 'y'])
    for _ax in _axes:
        for _off in OFFSETS:
            TUBES.append((_li, _ax, _off))
N_TUBES = len(TUBES)     # 4 x 4 + 8 + 8 = 32
TUBE_ID = {t: i for i, t in enumerate(TUBES)}

RUNS = ['None', 'RA hover', 'RA vertical', 'FCFS (2D rule)', 'FCFS time-only',
        'Altitude-first', 'Joint LP']
STRATS = ['FCFS (2D rule)', 'FCFS time-only', 'Altitude-first', 'Joint LP']
R_COLORS = {'None': '#9e9e9e', 'RA hover': '#bdbdbd', 'RA vertical': '#7986CB',
            'FCFS (2D rule)': '#B0BEC5', 'FCFS time-only': '#4FC3F7',
            'Altitude-first': '#81C784', 'Joint LP': '#FFB74D'}

OUTPUT_DIR = os.path.normpath(os.path.join(
    os.path.dirname(__file__), '..', '..', '..',
    'outputs', '02_logistics_delivery', '3d', 's035_3d_utm_simulation',
))

DENIED = []      # (seed, strategy, drone) registrations with no feasible hold


# ── Routes ──────────────────────────────────────────────────────────────────────

@dataclass
class Option:
    layers: tuple             # (leg-1 layer, leg-2 layer), 0-based
    wps: np.ndarray           # waypoints (m, 3)
    samples: np.ndarray       # positions every DT from take-off to landing (n, 3)
    tube_si: np.ndarray       # sample indices inside one of its two tubes (r <= r_c)
    tube_ci: np.ndarray       # matching tube ids (a sample can be in both)
    i_top: int                # first sample after the take-off climb
    i_land: int               # first sample of the landing descent
    vert_travel: float        # m total |dz| along the route

    @property
    def duration(self):
        return (len(self.samples) - 1) * DT


@dataclass
class Intent:
    id: int
    t_req: float
    depot: np.ndarray
    dest: np.ndarray
    opts: list = field(default_factory=list)    # feasible Option list, [0] = default
    n_rejected: int = 0                          # options rejected by the gamma check
    deadline: float = 0.0
    emergency: bool = False


def tube_dist(p, t):
    li, ax, off = TUBES[t]
    if ax == 'x':
        return np.hypot(p[..., 1] - off, p[..., 2] - ZMID[li])
    return np.hypot(p[..., 0] - off, p[..., 2] - ZMID[li])


def build_option(ox, oy, xc, yd, l1, l2):
    """Route with legs in layers l1 (E/W leg) and l2 (N/S leg), or None if the
    transition ramp at gamma_max does not fit on the legs (gamma check)."""
    z1, z2 = ZMID[l1], ZMID[l2]
    sx, sy = np.sign(xc - ox), np.sign(yd - oy)
    h = abs(z2 - z1) / TAN_G                      # horizontal length of the ramp
    if h / 2 > abs(xc - ox) - RAMP_MARGIN or h / 2 > abs(yd - oy) - RAMP_MARGIN:
        return None
    w = [(ox, oy, 0.0), (ox, oy, z1)]
    if h > 0:
        w += [(xc - sx * h / 2, oy, z1), (xc, oy, 0.5 * (z1 + z2)), (xc, oy + sy * h / 2, z2)]
    else:
        w += [(xc, oy, z1)]
    w += [(xc, yd, z2), (xc, yd, 0.0)]
    wps = np.array(w)
    seg_t = []
    for i, (a, b) in enumerate(zip(wps[:-1], wps[1:])):
        dh = np.linalg.norm(b[:2] - a[:2]); dz = abs(b[2] - a[2])
        if dh > 0:
            assert np.arctan2(dz, dh) <= GAMMA_MAX + 1e-9, 'gamma_max exceeded'
            seg_t.append(dh / V_CRUISE)
        else:
            assert i in (0, len(wps) - 2), 'vertical leg outside take-off / landing'
            seg_t.append(dz / V_VERT)
    T = np.cumsum([0.0] + seg_t)
    n = int(np.ceil(T[-1] / DT)) + 1
    ts = np.minimum(np.arange(n) * DT, T[-1])
    s = np.minimum(np.searchsorted(T, ts, side='right') - 1, len(seg_t) - 1)
    f = (ts - T[s]) / np.maximum(np.array(seg_t)[s], 1e-9)
    pos = wps[s] + f[:, None] * (wps[s + 1] - wps[s])
    assert pos[:, 2].min() >= -1e-9 and pos[:, 2].max() <= Z_CEIL + 1e-9
    t1, t2 = TUBE_ID[(l1, 'x', oy)], TUBE_ID[(l2, 'y', xc)]
    m1, m2 = np.flatnonzero(tube_dist(pos, t1) <= R_TUBE), np.flatnonzero(tube_dist(pos, t2) <= R_TUBE)
    tube_si = np.concatenate([m1, m2]); tube_ci = np.concatenate([np.full(len(m1), t1), np.full(len(m2), t2)])
    i_top = int(np.searchsorted(ts, T[1] - 1e-9))
    i_land = int(np.searchsorted(ts, T[-2] + 1e-9))
    return Option((l1, l2), wps, pos, tube_si, tube_ci, i_top, i_land,
                  float(np.sum(np.abs(np.diff(pos[:, 2])))))


def build_intents(seed=SEED):
    """Same random draws as the 2D S035 script (identical request sets)."""
    rng = np.random.default_rng(seed)
    t_req = np.round(rng.uniform(0, T_REQ_WINDOW, N_DRONES) / DT) * DT
    emergency_idx = int(rng.integers(N_DRONES))
    intents = []
    for k in range(N_DRONES):
        ox, oy = DEPOTS[k % 4]
        while True:
            xc = OFFSETS[int(rng.integers(4))]
            yd = rng.uniform(100, 900)
            if abs(xc - ox) >= 100 and abs(yd - oy) >= 100:
                break
        slack = rng.uniform(60, 300)
        rng.uniform(0.5, 2.5)                       # payload draw (keeps 2D sequence)
        l1 = 0 if xc > ox else 2                    # L1 E / L3 W
        l2 = 1 if yd > oy else 3                    # L2 N / L4 S
        alts = [4] + ([5] if k == emergency_idx else [])
        combos = [(l1, l2)] + [(a, b) for a in [l1] + alts for b in [l2] + alts
                               if (a, b) != (l1, l2)]
        it = Intent(k, float(t_req[k]), np.array([ox, oy]), np.array([xc, yd]),
                    emergency=(k == emergency_idx))
        for (a, b) in combos:
            o = build_option(ox, oy, xc, yd, a, b)
            if o is None:
                it.n_rejected += 1
            else:
                it.opts.append(o)
        assert it.opts[0].layers == (l1, l2), 'default option must be feasible'
        it.deadline = it.t_req + it.opts[0].duration + slack
        intents.append(it)
    return intents


# ── Pairwise conflict sets ──────────────────────────────────────────────────────

class ConflictSets:
    """F(k, j, a, b) = set of relative holds s = d_j - d_k (DT units) for which drone
    k on option a and drone j on option b come closer than the threshold."""

    def __init__(self, intents, rule, thr):
        self.it, self.rule, self.thr = intents, rule, thr
        self.a = [int(round(i.t_req / DT)) for i in intents]
        self.cache = {}

    def get(self, k, j, a, b):
        key = (k, j, a, b)
        if key not in self.cache:
            Pk, Pj = self.it[k].opts[a].samples, self.it[j].opts[b].samples
            if self.rule == 'sphere':
                d2 = np.sum((Pk[:, None, :] - Pj[None, :, :]) ** 2, axis=2)
                ii, jj = np.nonzero(d2 < self.thr ** 2)
            else:   # 2D cylinder rule: horizontal < thr AND vertical < 10 m
                dh = np.linalg.norm(Pk[:, None, :2] - Pj[None, :, :2], axis=2)
                dv = np.abs(Pk[:, None, 2] - Pj[None, :, 2])
                ii, jj = np.nonzero((dh < self.thr) & (dv < D_SEP_V_2D))
            self.cache[key] = frozenset((self.a[k] - self.a[j] + ii - jj).tolist())
        return self.cache[key]

    def fixed(self, opt):
        return {(k, j): self.get(k, j, opt[k], opt[j])
                for k in range(N_DRONES) for j in range(k + 1, N_DRONES)}


def conflict_pairs(F, d):
    return sorted(p for p in F if (d[p[1]] - d[p[0]]) in F[p])


# ── Strategic resolution ────────────────────────────────────────────────────────

class Registrar:
    """Sequential intent registration with conflict sets and tube capacity."""

    def __init__(self, intents, CS):
        self.it, self.CS = intents, CS
        self.a = CS.a
        horizon = int((T_REQ_WINDOW + MAX_HOLD) / DT) + max(
            len(o.samples) for i in intents for o in i.opts) + 2
        self.occ = np.zeros((N_TUBES, horizon), dtype=int)
        self.d, self.opt, self.reg = {}, {}, []

    def smallest_hold(self, j, o):
        bad = set()
        for k in self.reg:
            if k < j:
                bad |= {s + self.d[k] for s in self.CS.get(k, j, self.opt[k], o)}
            else:
                bad |= {self.d[k] - s for s in self.CS.get(j, k, o, self.opt[k])}
        op = self.it[j].opts[o]; si, ci = op.tube_si, op.tube_ci
        for h in range(int(MAX_HOLD / DT) + 1):
            if h in bad:
                continue
            if np.any(self.occ[ci, self.a[j] + h + si] + 1 > RHO_MAX):
                continue
            return h
        return None

    def commit(self, j, o, h):
        op = self.it[j].opts[o]; si, ci = op.tube_si, op.tube_ci
        np.add.at(self.occ, (ci, self.a[j] + h + si), 1)
        self.d[j], self.opt[j] = h, o
        self.reg.append(j)


def fcfs_order(intents):
    return [i.id for i in sorted(intents, key=lambda i: (i.t_req, i.id))]


def extra_time(it, o):
    return it.opts[o].duration - it.opts[0].duration


def resolve_time_only(intents, CS, opt=None, tag=''):
    """FCFS registration with fixed layer options (default: all default layers)."""
    opt = opt or [0] * N_DRONES
    R = Registrar(intents, CS)
    for j in fcfs_order(intents):
        h = R.smallest_hold(j, opt[j])
        if h is None:
            DENIED.append((tag, j)); h = int(MAX_HOLD / DT)
        R.commit(j, opt[j], h)
    return R.d, list(opt)


def resolve_altitude_first(intents, CS, tag=''):
    R = Registrar(intents, CS)
    n_alt = 0
    for j in fcfs_order(intents):
        it = intents[j]
        if R.smallest_hold(j, 0) == 0:
            R.commit(j, 0, 0); continue
        alts = sorted(range(1, len(it.opts)), key=lambda o: (extra_time(it, o), o))
        pick = None
        for o in alts:
            if extra_time(it, o) <= T_THRESH and R.smallest_hold(j, o) == 0:
                pick = (o, 0); break
        if pick is None:      # no cheap altitude option at zero hold: cheapest option + hold
            best = None
            for o in range(len(it.opts)):
                h = R.smallest_hold(j, o)
                if h is None:
                    continue
                c = h * DT + extra_time(it, o)
                if best is None or c < best[0] - 1e-9:
                    best = (c, o, h)
            if best is None:
                DENIED.append((tag, j)); best = (0, 0, int(MAX_HOLD / DT))
            pick = (best[1], best[2])
        n_alt += pick[0] != 0
        R.commit(j, *pick)
    return R.d, [R.opt[k] for k in range(N_DRONES)], n_alt


def resolve_lp(F, d_ref, max_iter=60):
    """Precedence LP (2D S035, generalised): min sum d_k subject to, for every pair that
    conflicts, s = d_j - d_k staying in the conflict-free interval of the reference plan
    d_ref (after all conflicting offsets, before them, or in the gap between two
    encounters).  d_ref satisfies every constraint, so the LP is feasible and its total
    never exceeds the reference; difference constraints with integer bounds give
    integral vertices.  Pairs that become conflicting are added (constraint generation)."""
    n = N_DRONES
    cons, d, iters = {}, {i: 0 for i in range(n)}, 0
    for _ in range(max_iter):
        bad = conflict_pairs(F, d)
        if not bad:
            break
        for (k, j) in bad:
            s_ref = d_ref[j] - d_ref[k]
            assert s_ref not in F[(k, j)], 'reference plan is not conflict-free'
            below = [f for f in F[(k, j)] if f < s_ref]
            above = [f for f in F[(k, j)] if f > s_ref]
            cons[(k, j)] = (max(below) + 1 if below else None, min(above) - 1 if above else None)
        A, b = [], []
        for (k, j), (lo, hi) in sorted(cons.items()):
            if lo is not None:              # d_j - d_k >= lo  ->  d_k - d_j <= -lo
                row = np.zeros(n); row[k], row[j] = 1.0, -1.0; A.append(row); b.append(-lo)
            if hi is not None:              # d_j - d_k <= hi
                row = np.zeros(n); row[k], row[j] = -1.0, 1.0; A.append(row); b.append(hi)
        res = linprog(np.ones(n), A_ub=np.array(A), b_ub=np.array(b),
                      bounds=[(0, None)] * n, method='highs')
        assert res.status == 0, 'LP infeasible'
        iters += 1
        d = {i: int(round(res.x[i])) for i in range(n)}
    assert not conflict_pairs(F, d), 'LP did not converge to a conflict-free plan'
    return d, iters


def plan_cost(intents, d, opt):
    return sum(d[k] * DT + extra_time(intents[k], opt[k]) for k in range(N_DRONES))


def resolve_joint(intents, CS, starts):
    """Layer local search; holds of each assignment by the precedence LP whose sides
    come from a feasible reference plan (FCFS registration on those layers, or a
    given start plan), so the LP total never exceeds the reference."""
    def evaluate(opt, d_ref=None):
        if d_ref is None:
            d_ref, _ = resolve_time_only(intents, CS, opt, tag='joint-ref')
        d, _ = resolve_lp(CS.fixed(opt), d_ref)
        return plan_cost(intents, d, opt), d
    best = None
    for (opt, d_ref) in starts:
        c, d = evaluate(opt, d_ref)
        if best is None or c < best[0] - 1e-9:
            best = (c, d, list(opt))
    n_eval = len(starts)
    for _ in range(LS_PASSES):
        improved = False
        for k in range(N_DRONES):
            for o in range(len(intents[k].opts)):
                if o == best[2][k]:
                    continue
                trial = list(best[2]); trial[k] = o
                c, d = evaluate(trial); n_eval += 1
                if c < best[0] - 1e-9:
                    best = (c, d, trial); improved = True
        if not improved:
            break
    return best[1], best[2], n_eval


# ── Execution with 3D separation assurance ─────────────────────────────────────

def execute(intents, opt, d, order, ra_mode):
    """Fly the plans. ra_mode: None, 'hover' or 'vertical'."""
    n = N_DRONES
    O = [intents[k].opts[opt[k]] for k in range(n)]
    S = [o.samples for o in O]
    ns = np.array([len(s) for s in S])
    rank = {k: r for r, k in enumerate(order)}
    a = np.array([int(round(intents[k].t_req / DT)) + d[k] for k in range(n)])
    hold = np.zeros(n, dtype=int); hold_until = np.zeros(n, dtype=int)
    held_cont = np.zeros(n, dtype=int); immune_until = np.zeros(n, dtype=int)
    n_watchdog = 0
    off = np.zeros(n); tgt = np.zeros(n); off_travel = np.zeros(n)
    vert_pair = [None] * n
    la = int(T_LA / DT)
    Q = np.arange(0, la + 1, 2)
    rate = VZ_TRANS * DT
    hist, air_hist = [], []
    ra_events, prev = [], set()
    vert_cmds = []                       # (t, drone, z_before, z_cmd)
    los_pairs, los_events, los_cyl = set(), [], set()
    min_sep = np.inf
    land_step = np.full(n, -1)
    step = 0

    def predict(k, frozen=None, target=None):
        fz = max(0, hold_until[k] - step) if frozen is None else frozen
        tg = tgt[k] if target is None else target
        tk = (step - a[k] - hold[k]) + Q - np.minimum(Q, fz)
        valid = (tk > 0) & (tk < ns[k] - 1)
        p = S[k][np.clip(tk, 0, ns[k] - 1)].copy()
        p[:, 2] += off[k] + np.clip(tg - off[k], -rate * Q, rate * Q)
        return p, valid

    def pair_min(pk, vk, pj, vj):
        m = vk & vj
        if not m.any():
            return np.inf
        return float(np.min(np.linalg.norm(pk[m] - pj[m], axis=1)))

    while True:
        taus = step - a - hold
        landed = taus >= ns - 1
        for k in np.flatnonzero(landed & (land_step < 0)):
            land_step[k] = step
        if landed.all():
            break
        air = (taus > 0) & ~landed
        P = np.array([S[k][int(np.clip(taus[k], 0, ns[k] - 1))] for k in range(n)])
        P[:, 2] += off
        assert P[:, 2].min() >= -1e-9 and P[:, 2].max() <= Z_CEIL + 1e-9, 'altitude out of bounds'
        hist.append(P.copy()); air_hist.append(air.copy())

        idx = np.flatnonzero(air)
        for ii, k in enumerate(idx):
            for j in idx[ii + 1:]:
                dd = float(np.linalg.norm(P[k] - P[j]))
                min_sep = min(min_sep, dd)
                if dd < R_SEP and (k, j) not in los_pairs:
                    los_pairs.add((k, j)); los_events.append((step * DT, k, j, 0.5 * (P[k] + P[j])))
                if (np.linalg.norm(P[k, :2] - P[j, :2]) < D_SEP_H_2D
                        and abs(P[k, 2] - P[j, 2]) < D_SEP_V_2D):
                    los_cyl.add((k, j))

        if ra_mode is not None:
            active = np.flatnonzero(~landed)
            pred = {k: predict(k) for k in active}
            # release vertical offsets that are no longer needed / before landing
            for w in active:
                if tgt[w] == 0.0:
                    continue
                margin = int(np.ceil(abs(off[w]) / rate)) + 2
                if taus[w] >= O[w].i_land - margin:
                    tgt[w] = 0.0; vert_pair[w] = None; pred[w] = predict(w); continue
                pw, vw = predict(w, target=0.0)
                if all(pair_min(pw, vw, *pred[o]) >= R_WARN for o in active if o != w):
                    tgt[w] = 0.0; vert_pair[w] = None; pred[w] = (pw, vw)
            ra_now, kinds = set(), {}
            for ii, k in enumerate(active):
                for j in active[ii + 1:]:
                    dmin = pair_min(*pred[k], *pred[j])
                    if dmin >= R_WARN:
                        continue
                    ra_now.add((k, j))
                    first, second = (j, k) if rank[j] > rank[k] else (k, j)
                    if ra_mode == 'vertical':
                        if (vert_pair[k] == (k, j) or vert_pair[j] == (k, j)) and dmin >= R_SEP:
                            kinds.setdefault((k, j), 'vertical'); continue
                        done = False
                        for w in (first, second):
                            if vert_pair[w] is not None:
                                continue
                            o_ = k if w == j else j
                            tw = taus[w]
                            if tw < O[w].i_top or tw + la + 4 * int(np.ceil(R_SEP / rate)) >= O[w].i_land:
                                continue
                            pw, vw = pred[w]; po, vo = pred[o_]
                            zw_la = pw[-1, 2] if vw[-1] else P[w, 2]
                            zo_la = po[-1, 2] if vo[-1] else P[o_, 2]
                            sgn = 1.0 if P[w, 2] >= P[o_, 2] else -1.0
                            dz_cmd = sgn * max(R_SEP - abs(zw_la - zo_la), DZ_MIN)
                            new_t = off[w] + dz_cmd
                            seg = S[w][O[w].i_top:O[w].i_land, 2]
                            if seg.min() + new_t < Z_FLOOR_AIR or seg.max() + new_t > Z_CEIL:
                                continue
                            pn, vn = predict(w, target=new_t)
                            if all(pair_min(pn, vn, *pred[oo]) >= R_SEP for oo in active if oo != w):
                                vert_cmds.append((step * DT, w, P[w, 2], P[w, 2] + dz_cmd))
                                tgt[w] = new_t; vert_pair[w] = (k, j); pred[w] = (pn, vn)
                                kinds.setdefault((k, j), 'vertical'); done = True
                                break
                        if done:
                            continue
                    # hover hold (2D S035 RA).  A drone still on the pad always waits
                    # there.  Otherwise the lower-precedence drone holds if that clears
                    # the encounter without creating a new one, else the other drone;
                    # drones released by the watchdog are immune for T_IMMUNE.
                    t_ra = int(np.ceil(((R_WARN - min(dmin, R_WARN)) / V_CLOSE + T_BUFFER) / DT))

                    def hold_ok(w):
                        pw, vw = predict(w, frozen=max(t_ra, hold_until[w] - step))
                        return all(pair_min(pw, vw, *pred[o]) >= R_SEP for o in active if o != w)
                    on_ground = [w for w in (first, second) if taus[w] <= 0]
                    if on_ground:
                        who = on_ground[0]
                    else:
                        cands = [w for w in (first, second) if step >= immune_until[w]]
                        oks = [w for w in cands if hold_ok(w)]
                        who = oks[0] if oks else (cands[0] if cands else None)
                    kinds.setdefault((k, j), 'hover')
                    if who is None:
                        continue
                    hold_until[who] = max(hold_until[who], step + t_ra)
                    pred[who] = predict(who)
            for p in sorted(ra_now - prev):
                ra_events.append((step * DT, p[0], p[1], kinds.get(p, 'hover'),
                                  bool(taus[p[0]] < O[p[0]].i_top or taus[p[1]] < O[p[1]].i_top)))
            prev = ra_now
        # advance
        for k in range(n):
            if landed[k]:
                continue
            frozen = step < hold_until[k]
            if off[k] != 0.0 and taus[k] + 1 >= O[k].i_land:
                frozen = True; tgt[k] = 0.0          # never start the landing with an offset
            if frozen:
                hold[k] += 1
                held_cont[k] += 1
                if taus[k] > 0 and held_cont[k] >= int(T_STUCK / DT) and step < hold_until[k]:
                    # watchdog: a drone hovering for T_STUCK is released (breaks hold cycles)
                    hold_until[k] = step + 1; immune_until[k] = step + int(T_IMMUNE / DT)
                    held_cont[k] = 0; n_watchdog += 1
            else:
                held_cont[k] = 0
            if taus[k] >= 0:
                dz = float(np.clip(tgt[k] - off[k], -rate, rate))
                off[k] += dz; off_travel[k] += abs(dz)
        step += 1
        assert step < 20000, (f'execution did not finish (mode {ra_mode}): taus {taus.tolist()} '
                              f'hold_until-step {(hold_until - step).tolist()} off {off.tolist()} '
                              f'events {ra_events[-6:]}')

    arrival = land_step * DT
    nominal = np.array([intents[k].t_req + intents[k].opts[0].duration for k in range(n)])
    gh = np.array([d[k] * DT for k in range(n)])
    ex = np.array([extra_time(intents[k], opt[k]) for k in range(n)])
    delay = arrival - nominal
    assert np.allclose(delay, gh + ex + hold * DT), 'delay decomposition mismatch'
    return {'hist': np.array(hist), 'air': np.array(air_hist), 'arrival': arrival,
            'delay': delay, 'ground_hold': gh, 'extra': ex, 'ra_hold': hold * DT,
            'ra_events': ra_events, 'vert_cmds': vert_cmds, 'los_events': los_events,
            'los_cyl': len(los_cyl), 'min_sep': min_sep, 'opt': list(opt),
            'c_alt': np.array([O[k].vert_travel for k in range(n)]) + off_travel,
            'route_vert': np.array([O[k].vert_travel for k in range(n)]),
            'off_travel': off_travel, 'n_watchdog': n_watchdog}


def corridor_occupancy(intents, res, d):
    n_steps = len(res['hist'])
    occ = np.zeros((N_TUBES, n_steps), dtype=int)
    for k in range(N_DRONES):
        o = intents[k].opts[res['opt'][k]]
        for s in np.flatnonzero(res['air'][:, k]):
            p = res['hist'][s, k]
            for t in sorted({int(c) for c in o.tube_ci}):
                if tube_dist(p, t) <= R_TUBE:
                    occ[t, s] += 1
    return occ


def summarise(intents, res, d, planned):
    n = N_DRONES
    t_mission = float(np.max(res['arrival'])) - min(i.t_req for i in intents)
    occ = corridor_occupancy(intents, res, d)
    return {
        'planned': planned, 'los': len(res['los_events']), 'los_cyl': res['los_cyl'],
        'min_sep': res['min_sep'], 'n_ra': len(res['ra_events']),
        'n_ra_vert': sum(e[3] == 'vertical' for e in res['ra_events']),
        'n_ra_takeoff': sum(e[4] for e in res['ra_events']),
        'n_vcmd': len(res['vert_cmds']),
        'n_watchdog': res['n_watchdog'],
        'ground_hold': float(res['ground_hold'].sum()), 'extra': float(res['extra'].sum()),
        'ra_hold': float(res['ra_hold'].sum()), 'total_delay': float(res['delay'].sum()),
        'mean_delay': float(res['delay'].mean()), 'max_delay': float(res['delay'].max()),
        'throughput': n / t_mission * 60.0, 't_mission': t_mission,
        'conflict_rate': planned / (n * (n - 1) / 2 * t_mission / 60.0),
        'c_alt': float(res['c_alt'].sum()), 'route_vert': float(res['route_vert'].sum()),
        'off_travel': float(res['off_travel'].sum()),
        'max_occ': int(occ.max()), 'occ': occ,
        'n_alt_layer': int(sum(o != 0 for o in res['opt'])),
        'late': int(sum(res['arrival'][k] > intents[k].deadline for k in range(n))),
    }


def run_simulation(seed=SEED, keep=True):
    intents = build_intents(seed)
    CS_sep = ConflictSets(intents, 'sphere', R_SEP)        # conflict counting
    CS_warn = ConflictSets(intents, 'sphere', R_WARN)      # strategic planning buffer
    CS_2d = ConflictSets(intents, 'cyl', D_SEP_H_2D)       # 2D rule counting
    CS_2d_plan = ConflictSets(intents, 'cyl', D_WARN_H_2D) # 2D rule planning
    zero = {k: 0 for k in range(N_DRONES)}
    opt0 = [0] * N_DRONES
    raw = conflict_pairs(CS_sep.fixed(opt0), zero)
    raw_2d = conflict_pairs(CS_2d.fixed(opt0), zero)
    order = fcfs_order(intents)
    tag = f'seed{seed}'

    plans = {'None': (zero, opt0, None), 'RA hover': (zero, opt0, 'hover'),
             'RA vertical': (zero, opt0, 'vertical')}
    d2, _ = resolve_time_only(intents, CS_2d_plan, tag=tag + '-2d')
    plans['FCFS (2D rule)'] = (d2, opt0, 'hover')
    df, of = resolve_time_only(intents, CS_warn, tag=tag + '-fcfs')
    plans['FCFS time-only'] = (df, of, 'hover')
    da, oa, n_alt = resolve_altitude_first(intents, CS_warn, tag=tag + '-alt')
    plans['Altitude-first'] = (da, oa, 'vertical')
    dj, oj, n_eval = resolve_joint(intents, CS_warn, [(of, df), (oa, da)])
    plans['Joint LP'] = (dj, oj, 'vertical')
    jc = plan_cost(intents, dj, oj)
    assert jc <= plan_cost(intents, df, of) + 1e-9 and jc <= plan_cost(intents, da, oa) + 1e-9

    runs, summ = {}, {}
    for name in RUNS:
        d, opt, mode = plans[name]
        exec_order = order if name in ('None', 'RA hover', 'RA vertical') else \
            [i for i in sorted(range(N_DRONES), key=lambda i: (intents[i].t_req + d[i] * DT, i))]
        res = execute(intents, opt, d, exec_order, mode)
        planned = len(conflict_pairs(CS_sep.fixed(opt), d))
        summ[name] = summarise(intents, res, d, planned)
        summ[name]['planned_warn'] = len(conflict_pairs(CS_warn.fixed(opt), d))
        if keep:
            runs[name] = res
    for nm in ('FCFS time-only', 'Altitude-first', 'Joint LP'):
        assert summ[nm]['planned_warn'] == 0 or DENIED, f'{nm} plan not conflict-free'
    info = {'raw': raw, 'raw_2d': raw_2d, 'n_alt': n_alt, 'n_eval': n_eval, 'plans': plans,
            'n_rejected': sum(i.n_rejected for i in intents),
            'n_options': sum(len(i.opts) for i in intents)}
    return intents, runs, summ, info


# ── Plots ───────────────────────────────────────────────────────────────────────

def _cols():
    return plt.get_cmap('tab20')(np.linspace(0, 1, N_DRONES))


def _draw_airspace(ax, alpha_lines=0.25, planes=True):
    for t, (li, axn, off) in enumerate(TUBES):
        z = ZMID[li]
        if axn == 'x':
            ax.plot([0, ARENA], [off, off], [z, z], color=LAYER_COLORS[li], alpha=alpha_lines, lw=1.0)
        else:
            ax.plot([off, off], [0, ARENA], [z, z], color=LAYER_COLORS[li], alpha=alpha_lines, lw=1.0)
    xx, yy = np.meshgrid([0, ARENA], [0, ARENA])
    for zb in (sorted({lo for lo, _, _ in LAYERS} | {Z_CEIL}) if planes else []):
        ax.plot_surface(xx, yy, np.full_like(xx, zb), color='grey', alpha=0.04, linewidth=0)
    ax.scatter(DEPOTS[:, 0], DEPOTS[:, 1], 0, color='k', marker='s', s=30)
    ax.set_xlim(0, ARENA); ax.set_ylim(0, ARENA); ax.set_zlim(0, Z_CEIL)
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')


def _sphere(ax, c, r, color='red'):
    u, v = np.meshgrid(np.linspace(0, 2 * np.pi, 14), np.linspace(0, np.pi, 8))
    ax.plot_wireframe(c[0] + r * np.cos(u) * np.sin(v), c[1] + r * np.sin(u) * np.sin(v),
                      c[2] + r * np.cos(v), color=color, alpha=0.35, lw=0.6)


def plot_airspace_3d(intents, runs, summ, out_dir):
    fig = plt.figure(figsize=(17, 8))
    cols = _cols()
    for p, name in enumerate(['None', 'Joint LP']):
        ax = fig.add_subplot(1, 2, p + 1, projection='3d')
        _draw_airspace(ax)
        res = runs[name]
        for k in range(N_DRONES):
            tr = res['hist'][res['air'][:, k], k]
            ax.plot(tr[:, 0], tr[:, 1], tr[:, 2], color=cols[k], lw=1.5)
            ax.scatter(*intents[k].dest, 0, color=cols[k], marker='*', s=50)
        if name == 'None':
            for (t, k, j, c) in res['los_events']:
                _sphere(ax, c, R_SEP / 2)
            ttl = (f"Unresolved plan (default layers, no RA): "
                   f"{summ[name]['los']} 3D losses of separation\n(red spheres of radius r_sep/2 at the midpoints; z axis stretched)")
        else:
            ttl = (f"Joint LP plan: {summ[name]['n_alt_layer']} drones use L5/L6, "
                   f"{summ[name]['los']} losses of separation")
        ax.set_title(ttl, fontsize=10)
        ax.view_init(elev=24, azim=-58)
    handles = [mpatches.Patch(color=LAYER_COLORS[i],
                              label=f'L{i+1} {lo:.0f}-{hi:.0f} m ({hd if hd in "ENWS" else ("any" if hd == "*" else "emergency")})')
               for i, (lo, hi, hd) in enumerate(LAYERS)]
    fig.legend(handles=handles, loc='lower center', ncol=6, fontsize=9)
    fig.suptitle('S035 3D UTM - corridor tube centre-lines, layer boundaries and flown 3D paths '
                 '(seed 7)', fontsize=12)
    path = os.path.join(out_dir, 'airspace_3d.png')
    plt.savefig(path, dpi=120, bbox_inches='tight'); plt.close()
    print(f'Saved: {path}')


def plot_altitude_timeseries(intents, runs, out_dir, ra_run, ra_seed):
    names = ['RA vertical', 'FCFS time-only', 'Altitude-first', 'Joint LP']
    fig, axes = plt.subplots(2, 2, figsize=(16, 9), sharex=True, sharey=True)
    cols = _cols()
    for ax, name in zip(axes.flat, names):
        res = ra_run if name == "RA vertical" else runs[name]
        t = np.arange(len(res['hist'])) * DT
        for k in range(N_DRONES):
            z = np.where(res['air'][:, k], res['hist'][:, k, 2], np.nan)
            ax.plot(t, z, color=cols[k], lw=1.2)
        for zb in sorted({lo for lo, _, _ in LAYERS} | {Z_CEIL}):
            ax.axhline(zb, color='k', ls='--', lw=0.6, alpha=0.5)
        for (tc, w, z0, z1) in res['vert_cmds']:
            ax.annotate('', xy=(tc, z1), xytext=(tc, z0),
                        arrowprops=dict(arrowstyle='->', color='red', lw=1.6))
        nv = len(res['vert_cmds']); nh = sum(e[3] == 'hover' for e in res['ra_events'])
        lab = f'{name} (seed {ra_seed})' if name == 'RA vertical' else f'{name} (seed {SEED})'
        ax.set_title(f'{lab}: {nv} vertical-offset RAs (red arrows), {nh} hover RA events',
                     fontsize=10)
        ax.set_ylim(0, Z_CEIL + 5); ax.grid(alpha=0.2)
    for ax in axes[1]:
        ax.set_xlabel('Time (s)')
    for ax in axes[:, 0]:
        ax.set_ylabel('Altitude z (m)')
    fig.suptitle('S035 3D - altitude of every drone (dashed = layer boundaries)', fontsize=12)
    fig.tight_layout()
    path = os.path.join(out_dir, 'altitude_timeseries.png')
    plt.savefig(path, dpi=120); plt.close()
    print(f'Saved: {path}')


def pair_min_seps(res):
    H, A = res['hist'], res['air']
    out = []
    for k in range(N_DRONES):
        for j in range(k + 1, N_DRONES):
            m = A[:, k] & A[:, j]
            if m.any():
                out.append(float(np.min(np.linalg.norm(H[m, k] - H[m, j], axis=1))))
    return np.array(out)


def plot_separation_histogram(runs, out_dir):
    fig, axes = plt.subplots(1, len(RUNS), figsize=(24, 4.6), sharey=True)
    for ax, name in zip(axes, RUNS):
        v = pair_min_seps(runs[name])
        ax.hist(v, bins=np.arange(0, 420, 15), color=R_COLORS[name], edgecolor='white')
        ax.axvline(R_SEP, color='red', ls='--', label='r_sep 35 m')
        ax.axvline(R_WARN, color='orange', ls='--', label='r_warn 52.5 m')
        ax.set_title(f'{name}\n{len(v)} pairs airborne together, {int(np.sum(v < R_SEP))} < 35 m',
                     fontsize=9)
        ax.set_xlabel('Min 3D separation (m)')
    axes[0].set_ylabel('Drone pairs'); axes[0].legend(fontsize=7)
    fig.suptitle('Per-pair minimum 3D distance while both airborne (seed 7)', fontsize=12)
    fig.tight_layout()
    path = os.path.join(out_dir, 'separation_histogram.png')
    plt.savefig(path, dpi=120); plt.close()
    print(f'Saved: {path}')


def plot_delay_comparison(summ, mc, out_dir):
    names = RUNS[1:]
    x = np.arange(len(names))
    fig, axes = plt.subplots(2, 2, figsize=(17, 10))
    ax = axes[0, 0]
    gh = np.array([summ[n]['ground_hold'] for n in names])
    ex = np.array([summ[n]['extra'] for n in names])
    rh = np.array([summ[n]['ra_hold'] for n in names])
    ax.bar(x, gh, color='#90A4AE', label='ground holds')
    ax.bar(x, ex, bottom=gh, color='#AED581', label='extra flight time (L5/L6)')
    ax.bar(x, rh, bottom=gh + ex, color='#FF8A65', label='RA hover holds')
    for xi in range(len(names)):
        tot = gh[xi] + ex[xi] + rh[xi]
        ax.text(xi, tot + 3, f'{tot:.1f}\n(mean {tot / N_DRONES:.1f})', ha='center', fontsize=8)
    ax.set_ylim(0, (gh + ex + rh).max() * 1.25)
    ax.set_title('Seed 7: total fleet delay (s)'); ax.legend(fontsize=8)
    ax = axes[0, 1]
    rv = np.array([summ[n]['route_vert'] for n in names])
    ot = np.array([summ[n]['off_travel'] for n in names])
    ax.bar(x, rv, color='#9575CD', label='planned climb / descent (incl. take-off, landing)')
    ax.bar(x, ot, bottom=rv, color='#E57373', label='vertical-offset RA travel')
    for xi in range(len(names)):
        ax.text(xi, rv[xi] + ot[xi] + 30, f'{rv[xi] + ot[xi]:.0f}', ha='center', fontsize=8)
    ax.set_ylim(0, (rv + ot).max() * 1.15)
    ax.set_title('Seed 7: altitude manoeuvre cost C_alt (m of vertical travel)'); ax.legend(fontsize=8)
    ax = axes[1, 0]
    data = [[r['total_delay'] for r in mc[n]] for n in names]
    ax.boxplot(data, positions=x, widths=0.5, showfliers=False)
    for xi, dd in enumerate(data):
        ax.scatter(np.full(len(dd), xi) + np.linspace(-0.15, 0.15, len(dd)), dd, s=10,
                   color=R_COLORS[names[xi]], edgecolor='k', lw=0.3, zorder=3)
    ax.set_title(f'Monte Carlo ({N_SEEDS} request sets): total fleet delay (s)')
    ax = axes[1, 1]
    c2 = [r['raw_2d'] for r in mc['_raw']]; c3 = [r['raw'] for r in mc['_raw']]
    xs = np.arange(N_SEEDS)
    ax.bar(xs - 0.2, c2, 0.4, color='#B0BEC5', label='2D rule (H < 30 m and V < 10 m)')
    ax.bar(xs + 0.2, c3, 0.4, color='#E57373', label='3D sphere (< 35 m)')
    ax.set_xticks(xs); ax.set_xlabel('Request-set seed')
    ax.set_title('Conflicting pairs of the unresolved plan, same trajectories')
    ax.legend(fontsize=8)
    for a_ in (axes[0, 0], axes[0, 1], axes[1, 0]):
        a_.set_xticks(x); a_.set_xticklabels(names, fontsize=8, rotation=15)
    for a_ in axes.flat:
        a_.grid(axis='y', alpha=0.3)
    fig.suptitle('S035 3D - delay vs altitude cost per strategy', fontsize=12)
    fig.tight_layout()
    path = os.path.join(out_dir, 'delay_comparison.png')
    plt.savefig(path, dpi=120); plt.close()
    print(f'Saved: {path}')


def plot_corridor_utilisation(summ, out_dir):
    names = ['FCFS time-only', 'Altitude-first', 'Joint LP']
    fig, axes = plt.subplots(1, 3, figsize=(19, 6), sharey=True)
    for ax, name in zip(axes, names):
        occ = summ[name]['occ']
        im = ax.imshow(occ, aspect='auto', origin='lower', cmap='YlOrRd', vmin=0, vmax=RHO_MAX + 1,
                       extent=[0, occ.shape[1] * DT, -0.5, N_TUBES - 0.5], interpolation='nearest')
        for b in (3.5, 7.5, 11.5, 15.5, 23.5):
            ax.axhline(b, color='k', lw=0.8)
        over = int(np.sum(occ > RHO_MAX))
        ax.set_title(f"{name}:\n"
                     f"max {summ[name]['max_occ']} drones in one 1-km tube "
                     f"(limit {RHO_MAX:.0f}; {over} tube-steps over)", fontsize=10)
        ax.set_xlabel('Time (s)')
    axes[0].set_yticks([1.5, 5.5, 9.5, 13.5, 19.5, 27.5])
    axes[0].set_yticklabels(['L1 E', 'L2 N', 'L3 W', 'L4 S', 'L5 x|y', 'L6 x|y'])
    axes[0].set_ylabel('Corridor tube (4 per heading layer, 8 in L5 / L6)')
    fig.colorbar(im, ax=axes, label='drones in tube (= drones/km)')
    path = os.path.join(out_dir, 'corridor_utilisation.png')
    plt.savefig(path, dpi=120, bbox_inches='tight'); plt.close()
    print(f'Saved: {path}')


def save_animation(runs, out_dir, seed, step=8):
    """Rotating 3D view of the unresolved plan of one request set under hover RAs
    (left) and vertical-first RAs (right)."""
    import matplotlib.animation as animation
    names = ['RA hover', 'RA vertical']
    n = max(len(runs[nm]['hist']) for nm in names)
    n_frames = int(np.ceil((n - 1) / step)) + 1
    cols = _cols()
    fig = plt.figure(figsize=(13, 6.3))
    axes, dots, rings = [], {}, {}
    for p, nm in enumerate(names):
        ax = fig.add_subplot(1, 2, p + 1, projection='3d')
        _draw_airspace(ax, alpha_lines=0.15, planes=False)
        ax.set_title(f'Seed {seed}, unresolved plan + hover RAs' if nm == 'RA hover'
                     else f'Seed {seed}, unresolved plan + vertical-first RAs', fontsize=10)
        dots[nm] = ax.scatter(np.zeros(N_DRONES), np.zeros(N_DRONES), np.zeros(N_DRONES),
                              c=cols, s=22, depthshade=False)
        rings[nm] = ax.scatter([], [], [], s=260, facecolors='none', edgecolors='red', lw=2)
        axes.append(ax)
    ttl = fig.suptitle('')
    trails = {nm: [axes[p].plot([], [], [], color=cols[k], lw=0.8, alpha=0.6)[0]
                   for k in range(N_DRONES)] for p, nm in enumerate(names)}

    def update(fi):
        f = min(fi * step, n - 1)
        ttl.set_text(f'S035 3D UTM  t = {f * DT:.0f} s   (red ring = 3D loss of separation < 35 m, '
                     f'red drone = vertical RA manoeuvre)')
        for p, nm in enumerate(names):
            H, A = runs[nm]['hist'], runs[nm]['air']
            i = min(f, len(H) - 1)
            m = A[i]
            pts = np.where(m[:, None], H[i], np.nan)
            dots[nm]._offsets3d = (pts[:, 0], pts[:, 1], pts[:, 2])
            vc = [c for c in runs[nm]['vert_cmds'] if 0 <= f * DT - c[0] < 10]
            fc = cols.copy()
            for c in vc:
                fc[c[1]] = (1, 0, 0, 1)
            dots[nm].set_color(fc)
            rx = []
            idx = np.flatnonzero(m)
            for a_, k in enumerate(idx):
                for j in idx[a_ + 1:]:
                    if np.linalg.norm(H[i, k] - H[i, j]) < R_SEP:
                        rx.append(0.5 * (H[i, k] + H[i, j]))
            rx = np.array(rx) if rx else np.full((0, 3), np.nan)
            rings[nm]._offsets3d = (rx[:, 0], rx[:, 1], rx[:, 2])
            for k in range(N_DRONES):
                lo = max(0, i - 40)
                seg = H[lo:i + 1, k][A[lo:i + 1, k]]
                trails[nm][k].set_data(seg[:, 0], seg[:, 1]); trails[nm][k].set_3d_properties(seg[:, 2])
            axes[p].view_init(elev=26, azim=-60 + 120.0 * fi / max(1, n_frames - 1))
        return []

    ani = animation.FuncAnimation(fig, update, frames=n_frames, interval=80, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer='pillow', fps=8, dpi=60)
    plt.close()
    print(f'Saved: {path}  ({n_frames} frames, every {step} steps of {DT} s)')


# ── Entry point ─────────────────────────────────────────────────────────────────

def _row(name, s):
    return (f"  {name:15s} planned={s['planned']:2d} LoS3D={s['los']:2d} LoS2Drule={s['los_cyl']:2d} "
            f"min_sep={s['min_sep']:6.1f} m  RA={s['n_ra']:2d} (take-off {s['n_ra_takeoff']:2d}, vert {s['n_ra_vert']:2d}, cmds {s['n_vcmd']:2d})  "
            f"ground={s['ground_hold']:6.1f} extra={s['extra']:6.1f} RA_hold={s['ra_hold']:6.1f}  "
            f"total={s['total_delay']:6.1f} s mean={s['mean_delay']:5.1f} max={s['max_delay']:5.1f}  "
            f"C_alt={s['c_alt']:7.1f} m (route {s['route_vert']:7.1f}, RA {s['off_travel']:5.1f})  "
            f"L5/L6 drones={s['n_alt_layer']:2d}  thr={s['throughput']:.3f}/min  "
            f"lambda3D={s['conflict_rate']:.4f}  max_occ={s['max_occ']}  late={s['late']}  watchdog={s['n_watchdog']}")


if __name__ == '__main__':
    t0 = time.time()
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    intents, runs, summ, info = run_simulation(SEED)
    print(f'Seed {SEED}: transition climb rate v tan(gamma_max) = {VZ_TRANS:.2f} m/s, '
          f'r_warn = {R_WARN:.1f} m')
    for it in intents:
        o = it.opts[0]
        print(f"  D{it.id:2d}: req {it.t_req:4.1f} s  ({it.depot[0]:.0f},{it.depot[1]:.0f}) -> "
              f"({it.dest[0]:.0f},{it.dest[1]:.0f})  default L{o.layers[0]+1}/L{o.layers[1]+1}  "
              f"flight {o.duration:5.1f} s  options " +
              ' '.join(f"L{p.layers[0]+1}/L{p.layers[1]+1}(+{p.duration - o.duration:.1f}s)"
                       for p in it.opts[1:]) +
              (f"  rejected {it.n_rejected}" if it.n_rejected else '') +
              ('  EMERGENCY' if it.emergency else ''))
    print(f"Options: {info['n_options']} feasible, {info['n_rejected']} rejected by the gamma check")
    print(f"Unresolved plan: {len(info['raw'])} conflicting pairs (3D sphere 35 m): "
          + ', '.join(f'D{k}-D{j}' for k, j in info['raw']))
    print(f"                 {len(info['raw_2d'])} conflicting pairs (2D rule 30 m / 10 m): "
          + ', '.join(f'D{k}-D{j}' for k, j in info['raw_2d']))
    for nm in STRATS:
        d, opt, _ = info['plans'][nm]
        print(f"  {nm:15s} holds (s): " + ' '.join(f'{d[k] * DT:.1f}' for k in range(N_DRONES))
              + '  layers: ' + ' '.join(
                  f"L{intents[k].opts[opt[k]].layers[0]+1}{intents[k].opts[opt[k]].layers[1]+1}"
                  for k in range(N_DRONES)))
    print(f"Altitude-first took an alternate layer for {info['n_alt']} drones; "
          f"joint LP evaluated {info['n_eval']} layer assignments")
    print('\n=== Results, seed 7 ===')
    for nm in RUNS:
        print(_row(nm, summ[nm]))

    print(f'\n=== Monte Carlo over {N_SEEDS} request sets (seeds 0..{N_SEEDS - 1}) ===')
    mc = {nm: [] for nm in RUNS}; mc['_raw'] = []
    for sd in range(N_SEEDS):
        _, _, s_sd, i_sd = run_simulation(sd, keep=False)
        for nm in RUNS:
            mc[nm].append(s_sd[nm])
        mc['_raw'].append({'raw': len(i_sd['raw']), 'raw_2d': len(i_sd['raw_2d'])})
    r2 = np.array([r['raw_2d'] for r in mc['_raw']]); r3 = np.array([r['raw'] for r in mc['_raw']])
    print(f'  unresolved conflicting pairs: 2D rule {r2.mean():.2f} +/- {r2.std():.2f}, '
          f'3D sphere {r3.mean():.2f} +/- {r3.std():.2f}; sphere >= 2D rule in '
          f'{int(np.sum(r3 >= r2))}/{N_SEEDS} sets')
    for nm in RUNS:
        rows = mc[nm]
        td = np.array([r['total_delay'] for r in rows])
        print(f"  {nm:15s} LoS3D={np.mean([r['los'] for r in rows]):5.2f} (sets with LoS "
              f"{sum(r['los'] > 0 for r in rows):2d})  RA={np.mean([r['n_ra'] for r in rows]):5.2f} (take-off {np.mean([r['n_ra_takeoff'] for r in rows]):5.2f})  "
              f"total_delay={td.mean():6.1f} +/- {td.std():5.1f} s [min {td.min():5.1f}, max {td.max():5.1f}]  "
              f"extra={np.mean([r['extra'] for r in rows]):5.1f}  "
              f"C_alt={np.mean([r['c_alt'] for r in rows]):7.1f} m  "
              f"L5/L6={np.mean([r['n_alt_layer'] for r in rows]):4.2f}  "
              f"max_occ={max(r['max_occ'] for r in rows)} (sets over cap "
              f"{sum(r['max_occ'] > RHO_MAX for r in rows)})  "
              f"thr={np.mean([r['throughput'] for r in rows]):.3f}  "
              f"vert_cmds={np.mean([r['n_vcmd'] for r in rows]):4.2f}  "
              f"watchdog={np.mean([r['n_watchdog'] for r in rows]):4.2f}  "
              f"late={np.mean([r['late'] for r in rows]):4.2f}")

    def wins(a, b):
        A = np.array([r['total_delay'] for r in mc[a]]); B = np.array([r['total_delay'] for r in mc[b]])
        return int(np.sum(A < B - 1e-9)), int(np.sum(np.abs(A - B) <= 1e-9)), int(np.sum(A > B + 1e-9))
    for a_, b_ in [('Altitude-first', 'FCFS time-only'), ('Joint LP', 'FCFS time-only'),
                   ('Joint LP', 'Altitude-first'), ('RA vertical', 'RA hover'),
                   ('FCFS time-only', 'FCFS (2D rule)')]:
        w, e, l = wins(a_, b_)
        print(f'  total delay {a_} vs {b_}: lower {w}, equal {e}, higher {l} (of {N_SEEDS})')
    print(f'  registrations denied (all runs): {len(DENIED)}')

    plot_airspace_3d(intents, runs, summ, OUTPUT_DIR)
    nv = [r['n_vcmd'] for r in mc['RA vertical']]
    seed_v = int(np.argmax(nv))
    _, runs_v, summ_v, _ = run_simulation(seed_v)
    print(f'Seed with the most vertical-offset RAs: {seed_v}')
    for nm in ('RA hover', 'RA vertical'):
        print(_row(nm, summ_v[nm]))
    plot_altitude_timeseries(intents, runs, OUTPUT_DIR, runs_v['RA vertical'], seed_v)
    plot_separation_histogram(runs, OUTPUT_DIR)
    plot_delay_comparison(summ, mc, OUTPUT_DIR)
    plot_corridor_utilisation(summ, OUTPUT_DIR)
    save_animation(runs_v, OUTPUT_DIR, seed_v)
    print(f'Runtime: {time.time() - t0:.0f} s', file=sys.stderr)
