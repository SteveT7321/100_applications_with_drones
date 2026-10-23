"""
S038 Disaster Relief Drop
=========================
A heterogeneous fleet of 5 drones flies relief supplies from a forward operating
base (FOB) to disaster sites in a 1000 x 1000 m area (2-D top-down model; the
vertical descent/ascent at each drop is folded into the drop time tau_drop).
Each site has a priority (1 critical / 2 urgent / 3 standard), a demand q_i and a
soft deadline d_i after which its value decays as exp(-lambda * lateness).
Drones fly multiple sorties (battery swap tau_turn at the FOB between sorties);
each sortie must respect the drone's payload Q_k and range R_k.

Objective (maximise):
    J = sum_served w_i q_i exp(-lambda max(0, t_i - d_i))
        - mu * sum_unserved w_i q_i,          w_i = (pi_max + 1 - pi_i) / pi_max

Three strategies share the same multi-sortie schedule representation and the
same evaluator:
  1. Greedy priority-first: the earliest-available drone takes the site with
     the best priority/flight-time score it can carry, packs nearest neighbours
     within payload/range, then 2-opt (distance) on the sortie.
  2. Regret insertion: repeatedly insert the unassigned site with the largest
     regret (2nd-best minus best insertion cost over drones, cost = loss of J)
     at its best position (inside an existing sortie or as a new sortie).
  3. Iterated local search (ILS): random feasible start, local search with
     Or-opt relocation (any site to any position / new sortie) and intra-sortie
     2-opt, both accepted on J; perturbation = random relocation of 3 sites.

The card's 8-site instance is solved first: every strategy reaches the
theoretical upper bound there, so it cannot rank them. The comparison is
therefore made on harder generated instances (20 sites, earlier deadlines,
heavier demand): one showcase instance (plotted) and a seeded study.

Usage:
    conda activate drones
    python src/02_logistics_delivery/s038_disaster_relief_drop.py
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.animation as animation

# ── Parameters ─────────────────────────────────────────────────────────────────
K_DRONES     = 5
T_MAX        = 600.0   # s — mission horizon (arrivals after T_MAX are unserved)
TAU_DROP     = 8.0     # s — descent + hover + release + ascent at each site
TAU_TURN     = 30.0    # s — battery swap turnaround at FOB
LAMBDA_DECAY = 0.01    # 1/s — value decay rate after soft deadline
MU_PENALTY   = 5.0     # unserved demand penalty multiplier
PI_MAX       = 3       # maximum priority level

FOB = np.array([500.0, 100.0])

# Heterogeneous fleet: [payload_kg, range_m, speed_m/s]
FLEET = np.array([
    [5.0, 2000.0, 12.0],   # drone 0 — heavy-lift, slow
    [5.0, 2000.0, 12.0],   # drone 1
    [2.5, 2800.0, 16.0],   # drone 2 — mid-range
    [2.5, 2800.0, 16.0],   # drone 3
    [1.0, 3500.0, 20.0],   # drone 4 — fast scout, light payload
])

# Card instance — relief sites: [x, y, demand_kg, priority, soft_deadline_s]
CARD_SITES = np.array([
    [150., 800., 3.0, 1, 120.],   # s0 critical
    [820., 750., 2.0, 1, 150.],   # s1 critical
    [300., 600., 1.5, 2, 200.],   # s2 urgent
    [700., 500., 1.0, 2, 240.],   # s3 urgent
    [200., 350., 2.5, 1, 100.],   # s4 critical
    [600., 300., 1.0, 3, 300.],   # s5 standard
    [850., 200., 0.8, 3, 350.],   # s6 standard
    [450., 700., 1.5, 2, 180.],   # s7 urgent
])

# Generated (harder) instances
M_GEN          = 20
GEN_XY_RANGE   = ((50.0, 950.0), (150.0, 950.0))
GEN_DEMAND     = (0.5, 4.0)                 # kg, uniform, rounded to 0.1
GEN_PRIO_P     = (0.3, 0.4, 0.3)           # P(critical / urgent / standard)
GEN_DEADLINE   = {1: (60.0, 150.0), 2: (120.0, 300.0), 3: (200.0, 450.0)}
SHOWCASE_SEED  = 0
STUDY_SEEDS    = list(range(10))           # includes the showcase seed

# ILS budget
ILS_RESTARTS   = 3
ILS_ITERS      = 25
ILS_KICK       = 3                          # sites relocated per perturbation
ILS_SEED       = 2026

GIF_FRAMES     = 150

DRONE_COLORS = ['tab:blue', 'tab:orange', 'tab:green', 'tab:red', 'tab:purple']
PRIORITY_COLORS = {1: 'red', 2: 'darkorange', 3: 'royalblue'}
STRATS = ['Greedy', 'Regret', 'ILS']
STRAT_COLORS = {'Greedy': '#1f77b4', 'Regret': '#ff7f0e', 'ILS': '#2ca02c'}

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '02_logistics_delivery', 's038_disaster_relief_drop',
)


# ── Instance ──────────────────────────────────────────────────────────────────
class Instance:
    def __init__(self, sites):
        self.sites = np.asarray(sites, dtype=float)
        self.M = len(self.sites)
        self.q = self.sites[:, 2]
        self.prio = self.sites[:, 3].astype(int)
        self.dl = self.sites[:, 4]
        self.w = (PI_MAX + 1 - self.prio) / PI_MAX
        pts = np.vstack([self.sites[:, :2], FOB])          # node M = FOB
        self.D = np.linalg.norm(pts[:, None, :] - pts[None, :, :], axis=2).tolist()
        self.F = self.M
        self.upper_bound = float(np.sum(self.w * self.q))   # every site on time

    def value(self, i, t):
        return self.w[i] * self.q[i] * np.exp(-LAMBDA_DECAY * max(0.0, t - self.dl[i]))

    def penalty(self, i):
        return MU_PENALTY * self.w[i] * self.q[i]


def make_instance(seed):
    rng = np.random.default_rng(seed)
    rows = []
    for _ in range(M_GEN):
        x = rng.uniform(*GEN_XY_RANGE[0])
        y = rng.uniform(*GEN_XY_RANGE[1])
        q = round(float(rng.uniform(*GEN_DEMAND)), 1)
        p = int(rng.choice([1, 2, 3], p=GEN_PRIO_P))
        d = float(rng.uniform(*GEN_DEADLINE[p]))
        rows.append([x, y, q, p, d])
    return Instance(rows)


# ── Schedule evaluation (schedule[k] = list of sorties, sortie = list of sites)
def sortie_len(inst, seq):
    D, F = inst.D, inst.F
    prev, L = F, 0.0
    for i in seq:
        L += D[prev][i]
        prev = i
    return L + D[prev][F]


def sortie_feasible(inst, k, seq):
    return (sum(inst.q[i] for i in seq) <= FLEET[k, 0] + 1e-9 and
            sortie_len(inst, seq) <= FLEET[k, 1] + 1e-9)


def drone_timeline(inst, k, sorties):
    """Per-sortie (depart, [(site, t_arrive)], return) for drone k."""
    D, F, v = inst.D, inst.F, FLEET[k, 2]
    out, t0 = [], 0.0
    for seq in sorties:
        t, prev, arr = t0, F, []
        for i in seq:
            t += D[prev][i] / v
            arr.append((i, t))
            t += TAU_DROP
            prev = i
        t_ret = t + D[prev][F] / v
        out.append((t0, arr, t_ret))
        t0 = t_ret + TAU_TURN
    return out


def drone_value(inst, k, sorties):
    """Value collected by drone k, plus the penalty of its sites arriving
    after T_MAX (they count as unserved)."""
    D, F, v = inst.D, inst.F, FLEET[k, 2]
    val, t0 = 0.0, 0.0
    for seq in sorties:
        t, prev = t0, F
        for i in seq:
            t += D[prev][i] / v
            val += inst.value(i, t) if t <= T_MAX else -inst.penalty(i)
            t += TAU_DROP
            prev = i
        t0 = t + D[prev][F] / v + TAU_TURN
    return val


def total_J(inst, schedule):
    assigned = {i for srt in schedule for seq in srt for i in seq}
    J = sum(drone_value(inst, k, schedule[k]) for k in range(K_DRONES))
    J -= sum(inst.penalty(i) for i in range(inst.M) if i not in assigned)
    return J


def schedule_stats(inst, schedule):
    arrivals = {}
    for k in range(K_DRONES):
        for _, arr, _ in drone_timeline(inst, k, schedule[k]):
            for i, t in arr:
                arrivals[i] = t
    served = {i for i, t in arrivals.items() if t <= T_MAX}
    late = [max(0.0, arrivals[i] - inst.dl[i]) for i in served]
    n_sorties = sum(len(s) for s in schedule)
    return dict(J=total_J(inst, schedule), served=served, arrivals=arrivals,
                late=late, n_sorties=n_sorties)


def clean(schedule):
    return [[list(seq) for seq in srt if seq] for srt in schedule]


def drop_late_sites(inst, schedule):
    """Remove sites that would be reached after T_MAX (they are unserved anyway;
    removing them can only make the remaining arrivals earlier)."""
    sched = clean(schedule)
    while True:
        late = [i for k in range(K_DRONES)
                for _, arr, _ in drone_timeline(inst, k, sched[k])
                for i, t in arr if t > T_MAX]
        if not late:
            return sched
        sched = clean([[[i for i in seq if i not in late] for seq in srt]
                       for srt in sched])


# ── Strategy 1: Greedy priority-first ─────────────────────────────────────────
def two_opt_distance(inst, seq):
    best = list(seq)
    improved = True
    while improved:
        improved = False
        for a in range(len(best) - 1):
            for b in range(a + 1, len(best)):
                cand = best[:a] + best[a:b + 1][::-1] + best[b + 1:]
                if sortie_len(inst, cand) < sortie_len(inst, best) - 1e-9:
                    best, improved = cand, True
    return best


def greedy_dispatch(inst):
    D, F = inst.D, inst.F
    t_avail = [0.0] * K_DRONES
    unserved = set(range(inst.M))
    schedule = [[] for _ in range(K_DRONES)]
    while unserved:
        k = int(np.argmin(t_avail))
        if t_avail[k] >= T_MAX:
            break
        v = FLEET[k, 2]
        # sites this drone can carry and reach before T_MAX
        cands = [i for i in unserved
                 if inst.q[i] <= FLEET[k, 0] and sortie_feasible(inst, k, [i])
                 and t_avail[k] + D[F][i] / v <= T_MAX]
        if not cands:
            t_avail[k] = np.inf            # nothing left for this drone
            continue
        seed = max(cands, key=lambda i: (inst.w[i] * PI_MAX / (D[F][i] / v), -i))
        seq, load, pos = [seed], inst.q[seed], seed
        rest = [i for i in cands if i != seed]
        while True:   # nearest-neighbour packing
            nxt = [i for i in sorted(rest, key=lambda i: (D[pos][i], i))
                   if load + inst.q[i] <= FLEET[k, 0] and
                   sortie_feasible(inst, k, seq + [i])]
            if not nxt:
                break
            seq.append(nxt[0])
            load += inst.q[nxt[0]]
            pos = nxt[0]
            rest.remove(nxt[0])
        seq = two_opt_distance(inst, seq)
        schedule[k].append(seq)
        unserved -= set(seq)
        t_avail[k] = drone_timeline(inst, k, schedule[k])[-1][2] + TAU_TURN
    return schedule


# ── Strategy 2: Regret insertion ──────────────────────────────────────────────
def best_insertion(inst, k, sorties, i, base_val):
    """Best (gain, new_sorties) for inserting site i into drone k's schedule."""
    best = (-np.inf, None)
    options = []
    for r, seq in enumerate(sorties):
        for p in range(len(seq) + 1):
            new_seq = seq[:p] + [i] + seq[p:]
            if sortie_feasible(inst, k, new_seq):
                options.append(sorties[:r] + [new_seq] + sorties[r + 1:])
    if sortie_feasible(inst, k, [i]):
        options.append(sorties + [[i]])
    for opt in options:
        gain = drone_value(inst, k, opt) - base_val
        if gain > best[0] + 1e-12:
            best = (gain, opt)
    return best


def regret_dispatch(inst):
    schedule = [[] for _ in range(K_DRONES)]
    base = [0.0] * K_DRONES
    unassigned = list(range(inst.M))
    while unassigned:
        pick = None   # (regret, -best_cost, -i, i, k, new_sorties)
        for i in unassigned:
            per_drone = []
            for k in range(K_DRONES):
                gain, opt = best_insertion(inst, k, schedule[k], i, base[k])
                # keep only insertions that beat leaving i unserved
                if opt is not None and gain > -inst.penalty(i) + 1e-9:
                    # cost = loss of J vs. serving i at full value on time
                    cost = inst.w[i] * inst.q[i] - gain
                    per_drone.append((cost, k, opt))
            if not per_drone:
                continue
            per_drone.sort(key=lambda c: (c[0], c[1]))
            c1 = per_drone[0][0]
            c2 = per_drone[1][0] if len(per_drone) > 1 else np.inf
            key = (c2 - c1, -c1, -i)
            if pick is None or key > pick[0]:
                pick = (key, i, per_drone[0][1], per_drone[0][2])
        if pick is None:
            break                    # nothing insertable: remaining sites unserved
        _, i, k, opt = pick
        schedule[k] = opt
        base[k] = drone_value(inst, k, opt)
        unassigned.remove(i)
    return schedule


# ── Strategy 3: Iterated local search ─────────────────────────────────────────
def random_start(inst, rng):
    schedule = [[] for _ in range(K_DRONES)]
    for i in rng.permutation(inst.M):
        i = int(i)
        for k in rng.permutation(K_DRONES):
            k = int(k)
            if schedule[k] and sortie_feasible(inst, k, schedule[k][-1] + [i]):
                schedule[k][-1].append(i)
                break
            if sortie_feasible(inst, k, [i]):
                schedule[k].append([i])
                break
    return schedule


def local_search(inst, schedule):
    """First-improvement Or-opt relocation + intra-sortie 2-opt, accepted on J."""
    schedule = clean(schedule)
    vals = [drone_value(inst, k, schedule[k]) for k in range(K_DRONES)]
    improved = True
    while improved:
        improved = False
        # 2-opt inside each sortie (J-based)
        for k in range(K_DRONES):
            for r, seq in enumerate(schedule[k]):
                for a in range(len(seq) - 1):
                    for b in range(a + 1, len(seq)):
                        cand = seq[:a] + seq[a:b + 1][::-1] + seq[b + 1:]
                        trial = schedule[k][:r] + [cand] + schedule[k][r + 1:]
                        tv = drone_value(inst, k, trial)
                        if tv > vals[k] + 1e-9:
                            schedule[k], vals[k], seq = trial, tv, cand
                            improved = True
        # Or-opt: relocate one site anywhere (incl. unassigned sites)
        assigned = {i for srt in schedule for s in srt for i in s}
        for i in range(inst.M):
            src = None
            for k in range(K_DRONES):
                for r, seq in enumerate(schedule[k]):
                    if i in seq:
                        src = (k, r)
            if src is not None:
                ks, rs = src
                removed = [list(s) for s in schedule[ks]]
                removed[rs].remove(i)
                removed = [s for s in removed if s]
                v_src_removed = drone_value(inst, ks, removed)
                old_total = vals[ks]
            else:
                ks, removed, v_src_removed = None, None, 0.0
                old_total = -inst.penalty(i)
            best = (0.0, None)
            for kd in range(K_DRONES):
                base_sched = removed if kd == ks else schedule[kd]
                base_val = v_src_removed if kd == ks else vals[kd]
                gain, opt = best_insertion(inst, kd, base_sched, i, base_val)
                if opt is None:
                    continue
                if kd == ks:
                    delta = (v_src_removed + gain) - old_total
                else:
                    delta = gain + (v_src_removed - old_total if ks is not None
                                    else inst.penalty(i))
                if delta > best[0] + 1e-9:
                    best = (delta, (kd, opt))
            if best[1] is not None:
                kd, opt = best[1]
                if ks is not None and kd != ks:
                    schedule[ks] = removed
                    vals[ks] = v_src_removed
                schedule[kd] = opt
                vals[kd] = drone_value(inst, kd, opt)
                improved = True
        assert assigned <= {i for srt in schedule for s in srt for i in s}
    return schedule


def perturb(inst, schedule, rng):
    sched = [[list(s) for s in srt] for srt in clean(schedule)]
    sites = [i for srt in sched for s in srt for i in s]
    if not sites:
        return sched
    for i in rng.choice(sites, size=min(ILS_KICK, len(sites)), replace=False):
        i = int(i)
        for srt in sched:
            for s in srt:
                if i in s:
                    s.remove(i)
        sched = [[s for s in srt if s] for srt in sched]
        for k in rng.permutation(K_DRONES):
            k = int(k)
            opts = []
            for r, s in enumerate(sched[k]):
                for p in range(len(s) + 1):
                    ns = s[:p] + [i] + s[p:]
                    if sortie_feasible(inst, k, ns):
                        opts.append((r, ns))
            if sortie_feasible(inst, k, [i]):
                opts.append((len(sched[k]), [i]))
            if opts:
                r, ns = opts[int(rng.integers(len(opts)))]
                if r == len(sched[k]):
                    sched[k].append(ns)
                else:
                    sched[k][r] = ns
                break
    return sched


def ils_dispatch(inst, seed):
    rng = np.random.default_rng(seed)
    best, best_J = None, -np.inf
    for _ in range(ILS_RESTARTS):
        cur = local_search(inst, random_start(inst, rng))
        cur_J = total_J(inst, cur)
        for _ in range(ILS_ITERS):
            cand = local_search(inst, perturb(inst, cur, rng))
            cand_J = total_J(inst, cand)
            if cand_J > cur_J + 1e-9:
                cur, cur_J = cand, cand_J
        if cur_J > best_J + 1e-9:
            best, best_J = cur, cur_J
    return best


# ── Simulation ────────────────────────────────────────────────────────────────
def solve_all(inst, ils_seed):
    out = {}
    for name, fn in (('Greedy', lambda: greedy_dispatch(inst)),
                     ('Regret', lambda: regret_dispatch(inst)),
                     ('ILS', lambda: ils_dispatch(inst, ils_seed))):
        sched = drop_late_sites(inst, fn())
        st = schedule_stats(inst, sched)
        st['schedule'] = sched
        out[name] = st
    return out


def run_simulation():
    card = Instance(CARD_SITES)
    card_res = solve_all(card, ILS_SEED)
    show = make_instance(SHOWCASE_SEED)
    show_res = solve_all(show, ILS_SEED + SHOWCASE_SEED)
    study = []
    for seed in STUDY_SEEDS:
        if seed == SHOWCASE_SEED:
            res = show_res
        else:
            res = solve_all(make_instance(seed), ILS_SEED + seed)
        study.append(dict(seed=seed, J={s: res[s]['J'] for s in STRATS},
                          served={s: len(res[s]['served']) for s in STRATS}))
    return dict(card=card, card_res=card_res, show=show, show_res=show_res,
                study=study)


# ── Plots ─────────────────────────────────────────────────────────────────────
def _setup_map(ax, inst, title):
    for i in range(inst.M):
        x, y = inst.sites[i, :2]
        ax.scatter(x, y, c=PRIORITY_COLORS[inst.prio[i]], s=40 + 30 * inst.q[i],
                   zorder=5, edgecolors='black', linewidths=0.5)
        ax.annotate(f'S{i}', (x, y), textcoords='offset points', xytext=(5, 3),
                    fontsize=6.5)
    ax.scatter(*FOB, marker='*', s=300, c='black', zorder=6)
    ax.set_xlim(-20, 1020)
    ax.set_ylim(-20, 1020)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.set_title(title, fontsize=10, fontweight='bold')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')


def plot_mission_map(data, out_dir):
    inst, res = data['show'], data['show_res']
    fig, axes = plt.subplots(1, 3, figsize=(18, 6.5))
    for ax, name in zip(axes, STRATS):
        r = res[name]
        _setup_map(ax, inst, f'{name}: J = {r["J"]:.2f}, served {len(r["served"])}/{inst.M}, '
                             f'{r["n_sorties"]} sorties')
        for k in range(K_DRONES):
            for r_idx, seq in enumerate(r['schedule'][k]):
                pts = [FOB] + [inst.sites[i, :2] for i in seq] + [FOB]
                ax.plot([p[0] for p in pts], [p[1] for p in pts],
                        color=DRONE_COLORS[k], lw=1.3, alpha=0.75,
                        ls='-' if r_idx == 0 else ('--' if r_idx == 1 else ':'))
        for i in range(inst.M):
            if i not in r['served']:
                ax.scatter(*inst.sites[i, :2], s=220, facecolors='none',
                           edgecolors='black', linewidths=1.5, zorder=7)
    handles = [mpatches.Patch(color=PRIORITY_COLORS[p], label=l) for p, l in
               ((1, 'Critical (P1)'), (2, 'Urgent (P2)'), (3, 'Standard (P3)'))]
    handles += [plt.Line2D([], [], color=DRONE_COLORS[k], lw=2, label=f'Drone {k}')
                for k in range(K_DRONES)]
    handles += [plt.Line2D([], [], color='gray', ls=ls, label=lab) for ls, lab in
                (('-', '1st sortie'), ('--', '2nd sortie'), (':', '3rd+ sortie'))]
    handles += [plt.Line2D([], [], marker='o', ls='', mfc='none', mec='black', ms=12,
                           label='Unserved')]
    fig.legend(handles=handles, loc='lower center', ncol=6, fontsize=8)
    fig.suptitle(f'S038 Disaster Relief Drop — Showcase Instance (seed {SHOWCASE_SEED}, '
                 f'{inst.M} sites; marker size ∝ demand)', fontsize=13, fontweight='bold')
    plt.tight_layout(rect=[0, 0.08, 1, 0.95])
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'mission_map.png')
    plt.savefig(path, dpi=130)
    plt.close()
    print(f'Saved: {path}')


def plot_gantt(data, out_dir):
    inst, res = data['show'], data['show_res']
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    for ax, name in zip(axes, STRATS):
        for k in range(K_DRONES):
            for t0, arr, t_ret in drone_timeline(inst, k, res[name]['schedule'][k]):
                ax.barh(k, t_ret - t0, left=t0, color=DRONE_COLORS[k], alpha=0.45,
                        edgecolor='gray', height=0.6)
                ax.barh(k, TAU_TURN, left=t_ret, color='gray', alpha=0.3,
                        height=0.6, hatch='//')
                for i, t in arr:
                    ax.scatter(t, k, color=PRIORITY_COLORS[inst.prio[i]], s=40,
                               zorder=5, marker='o' if t <= T_MAX else 'x')
        ax.axvline(T_MAX, color='black', ls='--', lw=1.2)
        ax.set_yticks(range(K_DRONES))
        ax.set_yticklabels([f'Drone {k}' for k in range(K_DRONES)])
        ax.set_xlim(0, T_MAX + 60)
        ax.set_title(f'{name} (J = {res[name]["J"]:.2f})', fontsize=10, fontweight='bold')
        ax.grid(True, axis='x', alpha=0.3)
    axes[-1].set_xlabel('Time (s)')
    axes[0].legend(handles=[
        mpatches.Patch(color='red', label='Critical drop'),
        mpatches.Patch(color='darkorange', label='Urgent drop'),
        mpatches.Patch(color='royalblue', label='Standard drop'),
        mpatches.Patch(facecolor='gray', alpha=0.4, hatch='//', label='Battery swap'),
        plt.Line2D([], [], color='black', ls='--', label='T_max'),
    ], loc='upper right', fontsize=8)
    fig.suptitle('S038 Disaster Relief Drop — Sortie Gantt Chart (showcase instance)',
                 fontsize=13, fontweight='bold')
    plt.tight_layout()
    path = os.path.join(out_dir, 'gantt_chart.png')
    plt.savefig(path, dpi=130)
    plt.close()
    print(f'Saved: {path}')


def plot_value_timeline(data, out_dir):
    inst, res = data['show'], data['show_res']
    fig, ax = plt.subplots(figsize=(10, 5))
    for name in STRATS:
        ev = sorted((t, inst.value(i, t)) for i, t in res[name]['arrivals'].items()
                    if t <= T_MAX)
        ts = [0.0] + [e[0] for e in ev] + [T_MAX]
        cum = np.concatenate([[0.0], np.cumsum([e[1] for e in ev]),
                              [sum(e[1] for e in ev)]])
        n_un = inst.M - len(res[name]['served'])
        ax.step(ts, cum, where='post', color=STRAT_COLORS[name], lw=2,
                label=f'{name}: collected {cum[-1]:.2f}, unserved {n_un} '
                      f'(J = {res[name]["J"]:.2f})')
    ax.axhline(inst.upper_bound, color='gray', ls='--', lw=1.2,
               label=f'Upper bound Σ w·q = {inst.upper_bound:.2f}')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Cumulative delivery value')
    ax.set_title('S038 — Cumulative Delivery Value vs Time (showcase instance)')
    ax.legend(fontsize=8.5, loc='lower right')
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'value_timeline.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def plot_coverage_bar(data, out_dir):
    inst, res = data['show'], data['show_res']
    fig, ax = plt.subplots(figsize=(9, 5))
    x = np.arange(len(STRATS))
    width = 0.25
    for p_idx, (lab, col) in enumerate((('Critical (P1)', 'red'), ('Urgent (P2)', 'darkorange'),
                                        ('Standard (P3)', 'royalblue'))):
        pri = p_idx + 1
        total = sum(inst.q[i] for i in range(inst.M) if inst.prio[i] == pri)
        fr = [sum(inst.q[i] for i in res[s]['served'] if inst.prio[i] == pri) / total
              for s in STRATS]
        bars = ax.bar(x + (p_idx - 1) * width, fr, width, color=col, alpha=0.75,
                      edgecolor='black', label=f'{lab}, {total:.1f} kg')
        for b, f in zip(bars, fr):
            ax.text(b.get_x() + b.get_width() / 2, f + 0.01, f'{f*100:.0f}%',
                    ha='center', fontsize=8)
    ax.set_xticks(x)
    ax.set_xticklabels(STRATS)
    ax.set_ylim(0, 1.2)
    ax.axhline(1.0, color='gray', ls='--', lw=1)
    ax.set_ylabel('Fraction of demand delivered before T_max')
    ax.set_title('S038 — Demand Coverage by Priority Tier (showcase instance)')
    ax.legend(fontsize=9, loc='upper center', ncol=3)
    ax.grid(True, axis='y', alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'coverage_bar.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def plot_lateness_histogram(data, out_dir):
    inst, res = data['show'], data['show_res']
    fig, axes = plt.subplots(1, 3, figsize=(14, 4), sharey=True)
    bins = np.arange(0, 451, 30)
    for ax, name in zip(axes, STRATS):
        late = res[name]['late']
        ax.hist(late, bins=bins, color=STRAT_COLORS[name], alpha=0.75, edgecolor='black')
        ax.set_title(f'{name}: {sum(1 for l in late if l > 0)}/{len(late)} served late, '
                     f'mean {np.mean(late):.0f} s', fontsize=10)
        ax.set_xlabel('Lateness max(0, t - d_soft) (s)')
        ax.grid(True, alpha=0.3)
    axes[0].set_ylabel('Served sites')
    fig.suptitle('S038 — Arrival Lateness of Served Sites (showcase instance)', fontsize=12)
    plt.tight_layout()
    path = os.path.join(out_dir, 'lateness_histogram.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def plot_instance_study(data, out_dir):
    study = data['study']
    fig, ax = plt.subplots(figsize=(11, 4.5))
    x = np.arange(len(study))
    for j, name in enumerate(STRATS):
        ax.bar(x + (j - 1) * 0.27, [s['J'][name] for s in study], 0.27,
               color=STRAT_COLORS[name], edgecolor='black', linewidth=0.4, label=name)
    ax.set_xticks(x)
    ax.set_xticklabels([f'seed {s["seed"]}' for s in study], fontsize=8)
    ax.set_ylabel('Total value J')
    ax.axhline(0, color='black', lw=0.8)
    ax.set_title(f'S038 — J per strategy on {len(study)} generated instances '
                 f'({M_GEN} sites each)')
    ax.legend(fontsize=9)
    ax.grid(True, axis='y', alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'instance_study.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def save_animation(data, out_dir):
    """All three strategies on the showcase instance, common clock."""
    inst, res = data['show'], data['show_res']
    frame_t = np.linspace(0.0, T_MAX, GIF_FRAMES)

    def knots(name, k):
        ts, ps = [0.0], [FOB]
        for t0, arr, t_ret in drone_timeline(inst, k, res[name]['schedule'][k]):
            ts.append(t0)
            ps.append(FOB)
            for i, t in arr:
                ts += [t, t + TAU_DROP]
                ps += [inst.sites[i, :2], inst.sites[i, :2]]
            ts.append(t_ret)
            ps.append(FOB)
        return np.array(ts), np.array(ps)

    kn = {(n, k): knots(n, k) for n in STRATS for k in range(K_DRONES)}
    fig, axes = plt.subplots(1, 3, figsize=(15, 5.4))
    site_sc, drone_sc = {}, {}
    for ax, name in zip(axes, STRATS):
        _setup_map(ax, inst, name)
        site_sc[name] = ax.scatter(inst.sites[:, 0], inst.sites[:, 1], s=40 + 30 * inst.q,
                                   facecolors='white',
                                   edgecolors=[PRIORITY_COLORS[p] for p in inst.prio],
                                   linewidths=2, zorder=8)
        drone_sc[name] = ax.scatter([FOB[0]] * K_DRONES, [FOB[1]] * K_DRONES,
                                    c=DRONE_COLORS, s=60, marker='D', zorder=9)
    clock = fig.suptitle('', fontsize=12, fontweight='bold')

    def update(f):
        t = frame_t[f]
        for name in STRATS:
            xy = []
            for k in range(K_DRONES):
                ts, ps = kn[(name, k)]
                xy.append([np.interp(t, ts, ps[:, 0]), np.interp(t, ts, ps[:, 1])])
            drone_sc[name].set_offsets(xy)
            fc = [PRIORITY_COLORS[inst.prio[i]]
                  if (i in res[name]['arrivals'] and res[name]['arrivals'][i] <= t)
                  else 'white' for i in range(inst.M)]
            site_sc[name].set_facecolors(fc)
        clock.set_text(f'S038 Disaster Relief Drop — t = {t:.0f} s '
                       f'(filled = delivered)')
        return list(drone_sc.values()) + list(site_sc.values()) + [clock]

    ani = animation.FuncAnimation(fig, update, frames=GIF_FRAMES, blit=False)
    plt.tight_layout()
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer='pillow', fps=15, dpi=72)
    plt.close()
    print(f'Saved: {path}')


# ── Main ──────────────────────────────────────────────────────────────────────
def print_results(title, inst, res):
    print(f'\n=== {title} ===')
    print(f'Sites {inst.M}, total demand {inst.q.sum():.1f} kg, '
          f'priorities P1/P2/P3 = {"/".join(str(int((inst.prio == p).sum())) for p in (1, 2, 3))}, '
          f'upper bound sum(w*q) = {inst.upper_bound:.3f}')
    print(f'{"Strategy":<8} {"J":>8} {"served":>7} {"crit":>6} {"late":>5} '
          f'{"meanLate":>9} {"sorties":>8}')
    n_crit = int((inst.prio == 1).sum())
    for s in STRATS:
        r = res[s]
        crit = sum(1 for i in r['served'] if inst.prio[i] == 1)
        n_late = sum(1 for l in r['late'] if l > 0)
        ml = np.mean(r['late']) if r['late'] else 0.0
        print(f'{s:<8} {r["J"]:>8.3f} {len(r["served"]):>3}/{inst.M:<3} '
              f'{crit:>2}/{n_crit:<3} {n_late:>5} {ml:>9.1f} {r["n_sorties"]:>8}')
    heavy = int((inst.q > FLEET[2, 0]).sum())
    print(f'  Sites heavier than {FLEET[2, 0]:.1f} kg (heavy-lift drones only): {heavy}/{inst.M}')
    for s in STRATS:
        un = sorted(set(range(inst.M)) - res[s]['served'])
        desc = ', '.join(f'S{i} ({inst.q[i]:.1f} kg, P{inst.prio[i]})' for i in un)
        print(f'  {s:<7} unserved sites: {desc if desc else "none"}')


if __name__ == '__main__':
    data = run_simulation()

    print_results('Card instance (8 sites)', data['card'], data['card_res'])
    print_results(f'Showcase generated instance (seed {SHOWCASE_SEED})',
                  data['show'], data['show_res'])

    study = data['study']
    print(f'\n=== Study: {len(study)} generated instances (seeds {STUDY_SEEDS[0]}..'
          f'{STUDY_SEEDS[-1]}) ===')
    print(f'{"seed":>4} ' + ' '.join(f'{s:>9}' for s in STRATS) + '   served G/R/I')
    for s in study:
        print(f'{s["seed"]:>4} ' + ' '.join(f'{s["J"][n]:>9.2f}' for n in STRATS) +
              f'   {s["served"]["Greedy"]}/{s["served"]["Regret"]}/{s["served"]["ILS"]}')
    for n in STRATS:
        Js = np.array([s['J'][n] for s in study])
        best = sum(1 for s in study if s['J'][n] >= max(s['J'].values()) - 1e-9)
        print(f'{n:<7} mean J = {Js.mean():.2f}, std = {Js.std():.2f}, '
              f'best (incl. ties) on {best}/{len(study)} instances, '
              f'mean served = {np.mean([s["served"][n] for s in study]):.1f}/{M_GEN}')
    ils_vs = {o: sum(1 for s in study if s['J']['ILS'] > s['J'][o] + 1e-9) for o in ('Greedy', 'Regret')}
    print(f'ILS beats Greedy on {ils_vs["Greedy"]}/{len(study)}, '
          f'beats Regret on {ils_vs["Regret"]}/{len(study)}; '
          f'Regret beats Greedy on {sum(1 for s in study if s["J"]["Regret"] > s["J"]["Greedy"] + 1e-9)}/{len(study)}')

    out_dir = os.path.normpath(OUTPUT_DIR)
    plot_mission_map(data, out_dir)
    plot_gantt(data, out_dir)
    plot_value_timeline(data, out_dir)
    plot_coverage_bar(data, out_dir)
    plot_lateness_histogram(data, out_dir)
    plot_instance_study(data, out_dir)
    save_animation(data, out_dir)
