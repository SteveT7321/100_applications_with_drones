"""
S038 3D Upgrade — Disaster Relief Drop
======================================
The S038 relief mission (5 heterogeneous drones, priority sites with soft
deadlines, multi-sortie schedules, value J) flown in a 3-D world:

  * The FOB is at z = 2 m, sites are on the ground (z = 0); drones fly straight
    3-D segments between waypoints at constant speed (card kinematics).
  * 6 debris AABBs (top <= 15 m) per instance.  Every segment is checked with
    the slab method; a blocked segment gets a vertical detour waypoint at its
    midpoint, raised to z_max + 5 m, recursively (card).
  * Payload range penalty R_eff = R_k - alpha_R * (sortie payload) (card).
  * Two drop protocols:
      hover          -- fly to the site overhead, descend / release / climb,
                        tau_drop = 8 s (the 2-D S038 protocol);
      release-and-go -- release in flight at z_r = 18 m from a wind-corrected
                        release point, tau_drop = 2 s (card).
    The release point comes from a numerical ballistic solver (3-D quadratic
    drag on the air-relative velocity, steady wind w).  The card's closed-form
    offset (v + w) sqrt(2 z_r / g) and an uncorrected overhead release are
    evaluated as alternatives.
  * Landing accuracy: Monte Carlo over the wind-estimation error
    (sigma_w = 0.5 m/s per horizontal axis, common random numbers for every
    method); a drop counts only within r_LZ = 3 m of the site centre.

The planners (Greedy, Regret, ILS) are the fixed 2-D S038 planners, re-run on
the 3-D distance matrix with the protocol's tau_drop and the payload range
penalty.  The 2-D baseline is the 2-D script itself (imported).  Release-and-go
schedules are re-evaluated exactly (true release points, detours, touchdown
times) after planning.

Usage:
    conda activate drones
    python src/02_logistics_delivery/3d/s038_3d_disaster_relief.py
"""

import matplotlib
matplotlib.use('Agg')

import os
import math
import importlib.util
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.lines import Line2D
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from matplotlib.animation import FuncAnimation, PillowWriter

HERE = os.path.dirname(os.path.abspath(__file__))
OUTPUT_DIR = os.path.normpath(os.path.join(
    HERE, '..', '..', '..', 'outputs', '02_logistics_delivery', '3d',
    's038_3d_disaster_relief'))
SRC_2D = os.path.normpath(os.path.join(HERE, '..', 's038_disaster_relief_drop.py'))

_spec = importlib.util.spec_from_file_location('s038_2d', SRC_2D)
m2d = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(m2d)

# ── Parameters (card values unless noted) ─────────────────────────────────────
K_DRONES     = m2d.K_DRONES          # 5
FLEET        = m2d.FLEET             # [payload kg, range m, speed m/s]
T_MAX        = m2d.T_MAX             # 600 s
TAU_TURN     = m2d.TAU_TURN          # 30 s
TAU_HOVER    = m2d.TAU_DROP          # 8 s  (2-D hover drop)
TAU_RG       = 2.0                   # s    release-and-go dwell (card)
LAMBDA_DECAY = m2d.LAMBDA_DECAY      # 0.01 1/s
MU_PENALTY   = m2d.MU_PENALTY        # 5.0
PI_MAX       = m2d.PI_MAX            # 3
ALPHA_R      = 50.0                  # m/kg payload range penalty
G            = 9.81
Z_FOB        = 2.0                   # m
FOB3         = np.array([m2d.FOB[0], m2d.FOB[1], Z_FOB])
Z_R          = 18.0                  # m release altitude (card)
Z_R_LOW      = 10.0                  # m added low-release variant
Z_HOVER_REL  = 2.0                   # m hover protocol releases at 2 m (2-D)
Z_BOUNDS     = (5.0, 40.0)           # m cruise altitude bounds
R_LZ         = 3.0                   # m landing zone radius
SIGMA_W      = 0.5                   # m/s wind-estimation error per axis
C_D          = 0.3                   # kg/m drag constant
WIND         = np.array([3.0, 2.0, 0.0])   # m/s
N_OBS        = 6
OBS_ZMAX     = (5.0, 15.0)           # m debris height range (added: U(5, 15))
OBS_SIDE     = (40.0, 160.0)         # m debris footprint side (added)
OBS_SITE_GAP = 30.0                  # m min gap site <-> debris footprint (added)
OBS_FOB_GAP  = 40.0                  # m min gap FOB <-> debris footprint (added)
DZ_CLEAR     = 5.0                   # m detour clearance margin
DETOUR_DEPTH = 20                    # max recursion depth (asserted)
CARD_DEBRIS_SEED = 38
GEN_DEBRIS_SEED  = 1000              # + instance seed
DT_BAL       = 0.01                  # s ballistic RK4 step
N_MC         = 1000                  # wind-error samples per drop
MC_SEED      = 3838
ZR_SWEEP     = np.array(sorted(set(np.arange(5.0, 40.01, 2.5).tolist()) | {18.0}))
SWEEP_CLASSES = [(1.0, 20.0, 'scout: 1.0 kg, 20 m/s'),
                 (2.5, 16.0, 'mid-range: 2.5 kg, 16 m/s'),
                 (4.0, 12.0, 'heavy-lift: 4.0 kg, 12 m/s')]
L_RUN        = 20.0                  # m straight level run-in before each release (added)
SHOWCASE_SEED = m2d.SHOWCASE_SEED
STUDY_SEEDS   = m2d.STUDY_SEEDS
ILS_SEED      = m2d.ILS_SEED
STRATS        = m2d.STRATS
GIF_DT        = 4.0                  # s per animation frame
GIF_FPS       = 12

DRONE_COLORS = m2d.DRONE_COLORS
PRIORITY_COLORS = m2d.PRIORITY_COLORS
STRAT_COLORS = m2d.STRAT_COLORS
WORLD_COLORS = {'2D': '#7f7f7f', 'hover': '#1f77b4', 'R&G': '#d62728', 'R&G low': '#9467bd'}

_MC_DW = np.random.default_rng(MC_SEED).normal(0.0, SIGMA_W, (N_MC, 2))


# ── Geometry: debris and detours ──────────────────────────────────────────────
def _rect_dist(px, py, b):
    dx = max(b[0] - px, 0.0, px - b[1])
    dy = max(b[2] - py, 0.0, py - b[3])
    return math.hypot(dx, dy)


def make_debris(seed, sites_xy):
    rng = np.random.default_rng(seed)
    boxes = []
    while len(boxes) < N_OBS:
        cx, cy = rng.uniform(0.0, 1000.0, 2)
        wx, wy = rng.uniform(*OBS_SIDE, 2)
        zt = rng.uniform(*OBS_ZMAX)
        b = (cx - wx / 2, cx + wx / 2, cy - wy / 2, cy + wy / 2, 0.0, zt)
        if _rect_dist(FOB3[0], FOB3[1], b) < OBS_FOB_GAP:
            continue
        if any(_rect_dist(x, y, b) < OBS_SITE_GAP for x, y in sites_xy):
            continue
        boxes.append(b)
    return boxes


def seg_hits_box(A, B, b):
    """Slab test: does segment A->B intersect the closed AABB b?"""
    t0, t1 = 0.0, 1.0
    for ax in range(3):
        lo, hi = b[2 * ax], b[2 * ax + 1]
        d = B[ax] - A[ax]
        if abs(d) < 1e-12:
            if A[ax] < lo or A[ax] > hi:
                return False
            continue
        ta, tb = (lo - A[ax]) / d, (hi - A[ax]) / d
        if ta > tb:
            ta, tb = tb, ta
        t0, t1 = max(t0, ta), min(t1, tb)
        if t0 > t1:
            return False
    return True


def detour_leg(A, B, boxes, depth=0):
    """Intermediate waypoints for A->B (card's recursive vertical detour; the
    detour altitude is SET to z_max + dz, see card correction)."""
    hits = [b for b in boxes if seg_hits_box(A, B, b)]
    if not hits:
        return []
    assert depth < DETOUR_DEPTH, 'detour recursion did not converge'
    M = 0.5 * (A + B)
    M[2] = max(M[2], max(b[5] for b in hits) + DZ_CLEAR)
    return detour_leg(A, M, boxes, depth + 1) + [M] + detour_leg(M, B, boxes, depth + 1)


def build_path(nodes, boxes):
    """Full waypoint list through the nodes with detours; also the index of
    each node in the list."""
    full, idx = [nodes[0]], [0]
    for A, B in zip(nodes[:-1], nodes[1:]):
        full += detour_leg(A, B, boxes)
        full.append(B)
        idx.append(len(full) - 1)
    full = np.array(full)
    for p in full[1:-1]:
        assert Z_BOUNDS[0] - 1e-9 <= p[2] <= Z_BOUNDS[1] + 1e-9, 'waypoint altitude out of bounds'
    for a, b in zip(full[:-1], full[1:]):
        assert not any(seg_hits_box(a, b, bx) for bx in boxes), 'segment hits debris'
    return full, idx


def path_len(full):
    return float(np.sum(np.linalg.norm(np.diff(full, axis=0), axis=1)))


# ── Ballistics ────────────────────────────────────────────────────────────────
def simulate_drops(z0, v0, m, wind, keep_traj=False):
    """Vectorised RK4 of n packages released at (0, 0, z0) with velocity v0
    (ground frame); drag -(C_D/m)|v - w|(v - w).  Returns horizontal landing
    offsets (n, 2), fall times (n,), and optionally trajectories."""
    n = len(z0)
    x = np.zeros((n, 3))
    x[:, 2] = z0
    v = np.array(v0, dtype=float)
    w = np.broadcast_to(wind, (n, 3)) if np.ndim(wind) == 1 else wind
    k = (C_D / np.asarray(m, dtype=float))[:, None]

    def acc(vv):
        rel = vv - w
        a = -k * np.linalg.norm(rel, axis=1, keepdims=True) * rel
        a[:, 2] -= G
        return a

    land = np.full((n, 2), np.nan)
    tf = np.full(n, np.nan)
    done = np.zeros(n, bool)
    traj = [x.copy()] if keep_traj else None
    t = 0.0
    for _ in range(20000):
        a1 = acc(v)
        a2 = acc(v + 0.5 * DT_BAL * a1)
        a3 = acc(v + 0.5 * DT_BAL * a2)
        a4 = acc(v + DT_BAL * a3)
        xn = x + DT_BAL * (v + DT_BAL / 6 * (a1 + a2 + a3))
        vn = v + DT_BAL / 6 * (a1 + 2 * a2 + 2 * a3 + a4)
        hit = (~done) & (xn[:, 2] <= 0.0)
        if hit.any():
            f = x[hit, 2] / (x[hit, 2] - xn[hit, 2])
            land[hit] = x[hit, :2] + f[:, None] * (xn[hit, :2] - x[hit, :2])
            tf[hit] = t + f * DT_BAL
            done |= hit
        x, v, t = xn, vn, t + DT_BAL
        if keep_traj:
            traj.append(np.where(done[:, None] & (x[:, 2] < 0)[:, None],
                                 np.column_stack([land, np.zeros(n)]), x))
        if done.all():
            break
    assert done.all(), 'package did not land'
    if keep_traj:
        return land, tf, np.array(traj)
    return land, tf


def tfall_analytic(z, m):
    vt = math.sqrt(m * G / C_D)
    return vt / G * math.acosh(math.exp(G * z / vt ** 2))


def card_offset(v0, z0):
    """Card's closed-form ballistic offset (v_xy + w_xy) sqrt(2 z / g)."""
    return (np.asarray(v0)[:, :2] + WIND[:2]) * np.sqrt(2 * np.asarray(z0) / G)[:, None]


def mc_miss(z0, v0, m):
    """Miss distances (n, N_MC) of the solver (A), card formula (B) and
    overhead (C) release points under wind-estimation error; common draws."""
    n = len(z0)
    off_nom, tf_nom = simulate_drops(z0, v0, m, WIND)
    zz = np.repeat(z0, N_MC)
    vv = np.repeat(np.asarray(v0), N_MC, axis=0)
    mm = np.repeat(m, N_MC)
    ww = np.zeros((n * N_MC, 3))
    ww[:, :2] = WIND[:2] + np.tile(_MC_DW, (n, 1))
    off_true, _ = simulate_drops(zz, vv, mm, ww)
    off_true = off_true.reshape(n, N_MC, 2)
    mA = np.linalg.norm(off_true - off_nom[:, None, :], axis=2)
    mB = np.linalg.norm(off_true - card_offset(v0, z0)[:, None, :], axis=2)
    mC = np.linalg.norm(off_true, axis=2)
    return dict(A=mA, B=mB, C=mC, off_nom=off_nom, tf=tf_nom,
                off_card=card_offset(v0, z0), off_true=off_true)


# ── 3-D planning world (planners copied from the fixed 2-D S038) ──────────────
class World:
    def __init__(self, sites, boxes, protocol, z_node):
        self.sites = np.asarray(sites, dtype=float)
        self.M = len(self.sites)
        self.q = self.sites[:, 2]
        self.prio = self.sites[:, 3].astype(int)
        self.dl = self.sites[:, 4]
        self.w = (PI_MAX + 1 - self.prio) / PI_MAX
        self.boxes = boxes
        self.protocol = protocol
        self.z_node = z_node
        self.tau = TAU_HOVER if protocol == 'hover' else TAU_RG
        self.nodes = np.vstack([np.column_stack([self.sites[:, :2],
                                                 np.full(self.M, z_node)]), FOB3])
        self.F = self.M
        n = self.M + 1
        D = np.zeros((n, n))
        self.n_blocked_pairs = 0
        for a in range(n):
            for b in range(a + 1, n):
                full, _ = build_path([self.nodes[a], self.nodes[b]], boxes)
                D[a, b] = D[b, a] = path_len(full)
                self.n_blocked_pairs += len(full) > 2
        self.D = D.tolist()
        self.upper_bound = float(np.sum(self.w * self.q))

    def value(self, i, t):
        return self.w[i] * self.q[i] * math.exp(-LAMBDA_DECAY * max(0.0, t - self.dl[i]))

    def penalty(self, i):
        return MU_PENALTY * self.w[i] * self.q[i]

    def r_eff(self, k, load):
        return FLEET[k, 1] - ALPHA_R * load


def sortie_len(W, seq):
    D, F = W.D, W.F
    prev, L = F, 0.0
    for i in seq:
        L += D[prev][i]
        prev = i
    return L + D[prev][F]


def sortie_feasible(W, k, seq):
    load = sum(W.q[i] for i in seq)
    return load <= FLEET[k, 0] + 1e-9 and sortie_len(W, seq) <= W.r_eff(k, load) + 1e-9


def drone_timeline(W, k, sorties):
    D, F, v = W.D, W.F, FLEET[k, 2]
    out, t0 = [], 0.0
    for seq in sorties:
        t, prev, arr = t0, F, []
        for i in seq:
            t += D[prev][i] / v
            arr.append((i, t))
            t += W.tau
            prev = i
        t_ret = t + D[prev][F] / v
        out.append((t0, arr, t_ret))
        t0 = t_ret + TAU_TURN
    return out


def drone_value(W, k, sorties):
    D, F, v = W.D, W.F, FLEET[k, 2]
    val, t0 = 0.0, 0.0
    for seq in sorties:
        t, prev = t0, F
        for i in seq:
            t += D[prev][i] / v
            val += W.value(i, t) if t <= T_MAX else -W.penalty(i)
            t += W.tau
            prev = i
        t0 = t + D[prev][F] / v + TAU_TURN
    return val


def total_J(W, schedule):
    assigned = {i for srt in schedule for seq in srt for i in seq}
    J = sum(drone_value(W, k, schedule[k]) for k in range(K_DRONES))
    J -= sum(W.penalty(i) for i in range(W.M) if i not in assigned)
    return J


def clean(schedule):
    return [[list(seq) for seq in srt if seq] for srt in schedule]


def drop_late_sites(W, schedule):
    sched = clean(schedule)
    while True:
        late = [i for k in range(K_DRONES)
                for _, arr, _ in drone_timeline(W, k, sched[k])
                for i, t in arr if t > T_MAX]
        if not late:
            return sched
        sched = clean([[[i for i in seq if i not in late] for seq in srt] for srt in sched])


def two_opt_distance(W, seq):
    best = list(seq)
    improved = True
    while improved:
        improved = False
        for a in range(len(best) - 1):
            for b in range(a + 1, len(best)):
                cand = best[:a] + best[a:b + 1][::-1] + best[b + 1:]
                if sortie_len(W, cand) < sortie_len(W, best) - 1e-9:
                    best, improved = cand, True
    return best


def greedy_dispatch(W):
    D, F = W.D, W.F
    t_avail = [0.0] * K_DRONES
    unserved = set(range(W.M))
    schedule = [[] for _ in range(K_DRONES)]
    while unserved:
        k = int(np.argmin(t_avail))
        if t_avail[k] >= T_MAX:
            break
        v = FLEET[k, 2]
        cands = [i for i in unserved
                 if W.q[i] <= FLEET[k, 0] and sortie_feasible(W, k, [i])
                 and t_avail[k] + D[F][i] / v <= T_MAX]
        if not cands:
            t_avail[k] = np.inf
            continue
        seed = max(cands, key=lambda i: (W.w[i] * PI_MAX / (D[F][i] / v), -i))
        seq, load, pos = [seed], W.q[seed], seed
        rest = [i for i in cands if i != seed]
        while True:
            nxt = [i for i in sorted(rest, key=lambda i: (D[pos][i], i))
                   if load + W.q[i] <= FLEET[k, 0] and sortie_feasible(W, k, seq + [i])]
            if not nxt:
                break
            seq.append(nxt[0])
            load += W.q[nxt[0]]
            pos = nxt[0]
            rest.remove(nxt[0])
        seq = two_opt_distance(W, seq)
        schedule[k].append(seq)
        unserved -= set(seq)
        t_avail[k] = drone_timeline(W, k, schedule[k])[-1][2] + TAU_TURN
    return schedule


def best_insertion(W, k, sorties, i, base_val):
    best = (-np.inf, None)
    options = []
    for r, seq in enumerate(sorties):
        for p in range(len(seq) + 1):
            new_seq = seq[:p] + [i] + seq[p:]
            if sortie_feasible(W, k, new_seq):
                options.append(sorties[:r] + [new_seq] + sorties[r + 1:])
    if sortie_feasible(W, k, [i]):
        options.append(sorties + [[i]])
    for opt in options:
        gain = drone_value(W, k, opt) - base_val
        if gain > best[0] + 1e-12:
            best = (gain, opt)
    return best


def regret_dispatch(W):
    schedule = [[] for _ in range(K_DRONES)]
    base = [0.0] * K_DRONES
    unassigned = list(range(W.M))
    while unassigned:
        pick = None
        for i in unassigned:
            per_drone = []
            for k in range(K_DRONES):
                gain, opt = best_insertion(W, k, schedule[k], i, base[k])
                if opt is not None and gain > -W.penalty(i) + 1e-9:
                    per_drone.append((W.w[i] * W.q[i] - gain, k, opt))
            if not per_drone:
                continue
            per_drone.sort(key=lambda c: (c[0], c[1]))
            c1 = per_drone[0][0]
            c2 = per_drone[1][0] if len(per_drone) > 1 else np.inf
            key = (c2 - c1, -c1, -i)
            if pick is None or key > pick[0]:
                pick = (key, i, per_drone[0][1], per_drone[0][2])
        if pick is None:
            break
        _, i, k, opt = pick
        schedule[k] = opt
        base[k] = drone_value(W, k, opt)
        unassigned.remove(i)
    return schedule


def random_start(W, rng):
    schedule = [[] for _ in range(K_DRONES)]
    for i in rng.permutation(W.M):
        i = int(i)
        for k in rng.permutation(K_DRONES):
            k = int(k)
            if schedule[k] and sortie_feasible(W, k, schedule[k][-1] + [i]):
                schedule[k][-1].append(i)
                break
            if sortie_feasible(W, k, [i]):
                schedule[k].append([i])
                break
    return schedule


def local_search(W, schedule):
    schedule = clean(schedule)
    vals = [drone_value(W, k, schedule[k]) for k in range(K_DRONES)]
    improved = True
    while improved:
        improved = False
        for k in range(K_DRONES):
            for r, seq in enumerate(schedule[k]):
                for a in range(len(seq) - 1):
                    for b in range(a + 1, len(seq)):
                        cand = seq[:a] + seq[a:b + 1][::-1] + seq[b + 1:]
                        trial = schedule[k][:r] + [cand] + schedule[k][r + 1:]
                        tv = drone_value(W, k, trial)
                        if tv > vals[k] + 1e-9:
                            schedule[k], vals[k], seq = trial, tv, cand
                            improved = True
        for i in range(W.M):
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
                v_src_removed = drone_value(W, ks, removed)
                old_total = vals[ks]
            else:
                ks, removed, v_src_removed = None, None, 0.0
                old_total = -W.penalty(i)
            best = (0.0, None)
            for kd in range(K_DRONES):
                base_sched = removed if kd == ks else schedule[kd]
                base_val = v_src_removed if kd == ks else vals[kd]
                gain, opt = best_insertion(W, kd, base_sched, i, base_val)
                if opt is None:
                    continue
                if kd == ks:
                    delta = (v_src_removed + gain) - old_total
                else:
                    delta = gain + (v_src_removed - old_total if ks is not None
                                    else W.penalty(i))
                if delta > best[0] + 1e-9:
                    best = (delta, (kd, opt))
            if best[1] is not None:
                kd, opt = best[1]
                if ks is not None and kd != ks:
                    schedule[ks] = removed
                    vals[ks] = v_src_removed
                schedule[kd] = opt
                vals[kd] = drone_value(W, kd, opt)
                improved = True
    return schedule


def perturb(W, schedule, rng):
    sched = [[list(s) for s in srt] for srt in clean(schedule)]
    sites = [i for srt in sched for s in srt for i in s]
    if not sites:
        return sched
    for i in rng.choice(sites, size=min(m2d.ILS_KICK, len(sites)), replace=False):
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
                    if sortie_feasible(W, k, ns):
                        opts.append((r, ns))
            if sortie_feasible(W, k, [i]):
                opts.append((len(sched[k]), [i]))
            if opts:
                r, ns = opts[int(rng.integers(len(opts)))]
                if r == len(sched[k]):
                    sched[k].append(ns)
                else:
                    sched[k][r] = ns
                break
    return sched


def ils_dispatch(W, seed):
    rng = np.random.default_rng(seed)
    best, best_J = None, -np.inf
    for _ in range(m2d.ILS_RESTARTS):
        cur = local_search(W, random_start(W, rng))
        cur_J = total_J(W, cur)
        for _ in range(m2d.ILS_ITERS):
            cand = local_search(W, perturb(W, cur, rng))
            cand_J = total_J(W, cand)
            if cand_J > cur_J + 1e-9:
                cur, cur_J = cand, cand_J
        if cur_J > best_J + 1e-9:
            best, best_J = cur, cur_J
    return best


# ── Exact 3-D evaluation (true release points, detours, touchdown) ────────────
def flown_geometry(W, schedule):
    """Flown waypoints for every sortie.  Hover: through the site overheads.
    Release-and-go: each drop gets a straight, level run-in of L_RUN metres at
    z_node along the planning-leg heading u (previous node -> site), so the
    release velocity v_k u is known before the release point is; the release
    point is then p_rel = s - offset(v_k u) from the ballistic solver."""
    entries = [(k, r, j, i) for k in range(K_DRONES)
               for r, seq in enumerate(schedule[k]) for j, i in enumerate(seq)]
    n = len(entries)
    pos = {(e[0], e[1], e[2]): idx for idx, e in enumerate(entries)}
    m = np.array([W.q[e[3]] for e in entries])
    rg = W.protocol != 'hover'
    vel = np.zeros((n, 3))
    p = np.array([W.nodes[e[3]] for e in entries]) if n else np.zeros((0, 3))
    tf = np.zeros(n)
    if rg and n:
        for e, (k, r, j, i) in enumerate(entries):
            prev = FOB3 if j == 0 else W.nodes[schedule[k][r][j - 1]]
            u = W.sites[i, :2] - prev[:2]
            u = u / np.linalg.norm(u)
            vel[e, :2] = FLEET[k, 2] * u
        off, tf = simulate_drops(p[:, 2], vel, m, WIND)
        p[:, :2] = np.array([W.sites[e[3], :2] for e in entries]) - off
    paths, sidx = {}, {}
    for k in range(K_DRONES):
        for r, seq in enumerate(schedule[k]):
            nodes, where = [FOB3], []
            for j in range(len(seq)):
                e = pos[(k, r, j)]
                if rg:
                    u3 = np.array([vel[e, 0], vel[e, 1], 0.0]) / FLEET[k, 2]
                    nodes.append(p[e] - L_RUN * u3)
                nodes.append(p[e])
                where.append(len(nodes) - 1)
            nodes.append(FOB3)
            full, idx = build_path(nodes, W.boxes)
            paths[(k, r)] = (full, idx)
            sidx[(k, r)] = [idx[w] for w in where]
            if rg:   # the run-in must be a single straight, level segment
                for w in where:
                    assert idx[w] - idx[w - 1] == 1, 'run-in blocked by debris'
                    assert abs(full[idx[w]][2] - full[idx[w - 1]][2]) < 1e-9
    if rg and n:
        off, _, traj = simulate_drops(p[:, 2], vel, m, WIND, keep_traj=True)
        land = p[:, :2] + off
        site = np.array([W.sites[e[3], :2] for e in entries])
        assert np.max(np.linalg.norm(land - site, axis=1)) < 1e-3, 'nominal drop misses site'
        pts = traj + np.column_stack([p[:, :2], np.zeros(n)])[None, :, :]
        for b in W.boxes:   # no package passes through debris
            inside = ((pts[..., 0] >= b[0]) & (pts[..., 0] <= b[1]) & (pts[..., 1] >= b[2]) &
                      (pts[..., 1] <= b[3]) & (pts[..., 2] <= b[5]))
            assert not inside.any(), 'package trajectory hits debris'
    return dict(entries=entries, pos=pos, p=p, vel=vel, paths=paths, sidx=sidx, tf=tf, m=m)


def evaluate(W, schedule):
    """Exact evaluation; removes sites that would break range (repair) or be
    delivered after T_MAX.  Returns stats incl. per-drop data."""
    sched = clean(schedule)
    n_repair = 0
    while True:
        geo = flown_geometry(W, sched)
        changed = False
        for k in range(K_DRONES):   # range (exact path) repair
            for r, seq in enumerate(sched[k]):
                load = sum(W.q[i] for i in seq)
                L = path_len(geo['paths'][(k, r)][0])
                if L > W.r_eff(k, load) + 1e-9:
                    sched[k][r] = seq[:-1]
                    n_repair += 1
                    changed = True
        if changed:
            sched = clean(sched)
            continue
        deliv, release_t, timeline = {}, {}, {}
        for k in range(K_DRONES):
            v, t0, tl = FLEET[k, 2], 0.0, []
            for r, seq in enumerate(sched[k]):
                full, idx = geo['paths'][(k, r)]
                seglen = np.linalg.norm(np.diff(full, axis=0), axis=1)
                ts, t = [t0], t0
                dwell_at = {si: j for j, si in enumerate(geo['sidx'][(k, r)])}
                kn_t, kn_p = [t0], [full[0]]
                for s_i in range(len(seglen)):
                    t += seglen[s_i] / v
                    kn_t.append(t)
                    kn_p.append(full[s_i + 1])
                    if s_i + 1 in dwell_at:
                        j = dwell_at[s_i + 1]
                        i = seq[j]
                        e = geo['pos'][(k, r, j)]
                        release_t[i] = t
                        deliv[i] = t + (geo['tf'][e] if W.protocol != 'hover' else 0.0)
                        t += W.tau
                        kn_t.append(t)
                        kn_p.append(full[s_i + 1])
                tl.append(dict(t0=t0, t_ret=t, kn_t=np.array(kn_t), kn_p=np.array(kn_p),
                               length=float(seglen.sum()),
                               load=sum(W.q[i] for i in seq)))
                t0 = t + TAU_TURN
            timeline[k] = tl
        late = [i for i, t in deliv.items() if t > T_MAX]
        if not late:
            break
        sched = clean([[[i for i in seq if i not in late] for seq in srt] for srt in sched])
    served = set(deliv)
    J = sum(W.value(i, deliv[i]) for i in served) - \
        sum(W.penalty(i) for i in range(W.M) if i not in served)
    lateness = [max(0.0, deliv[i] - W.dl[i]) for i in served]
    for k in range(K_DRONES):
        for s in timeline[k]:
            assert s['load'] <= FLEET[k, 0] + 1e-9
            assert s['length'] <= W.r_eff(k, s['load']) + 1e-9
            assert np.all(s['kn_p'][:, 2] >= Z_FOB - 1e-9)
    n_det = sum(len(full) - 2 - (len(idx) - 2) for full, idx in geo['paths'].values())
    straight = 0.0
    for (k, r), (full, idx) in geo['paths'].items():
        straight += path_len(full[idx])
    return dict(schedule=sched, geo=geo, J=J, served=served, deliv=deliv,
                release_t=release_t, timeline=timeline, late=lateness,
                n_sorties=sum(len(s) for s in sched), n_repair=n_repair,
                n_detour_wp=n_det,
                detour_extra=sum(path_len(f) for f, _ in geo['paths'].values()) - straight,
                total_len=sum(path_len(f) for f, _ in geo['paths'].values()))


def landing_mc(W, ev):
    """Per-drop Monte Carlo miss distances for the flown schedule."""
    geo = ev['geo']
    ent = geo['entries']
    if W.protocol == 'hover':
        z0 = np.full(len(ent), Z_HOVER_REL)
        v0 = np.zeros((len(ent), 3))
    else:
        z0 = geo['p'][:, 2]
        v0 = geo['vel']
    mc = mc_miss(z0, v0, geo['m'])
    mc['sites'] = [e[3] for e in ent]
    mc['p_hit'] = {meth: (mc[meth] <= R_LZ).mean(axis=1) for meth in 'ABC'}
    return mc


def expected_J(W, ev, mc):
    """J if a drop outside r_LZ counts as unserved (penalty), method A."""
    J = -sum(W.penalty(i) for i in range(W.M) if i not in ev['served'])
    for e, i in enumerate(mc['sites']):
        p = mc['p_hit']['A'][e]
        J += p * W.value(i, ev['deliv'][i]) - (1 - p) * W.penalty(i)
    return J


# ── Simulation ────────────────────────────────────────────────────────────────
def solve_3d(sites, boxes, ils_seed, protocol, z_node, strategies=STRATS):
    W = World(sites, boxes, protocol, z_node)
    out = {'world': W}
    for name in strategies:
        if name == 'Greedy':
            sch = greedy_dispatch(W)
        elif name == 'Regret':
            sch = regret_dispatch(W)
        else:
            sch = ils_dispatch(W, ils_seed)
        ev = evaluate(W, drop_late_sites(W, sch))
        ev['J_plan'] = total_J(W, drop_late_sites(W, sch))
        ev['mc'] = landing_mc(W, ev)
        ev['J_exp'] = expected_J(W, ev, ev['mc'])
        out[name] = ev
    return out


def solve_instance(sites, debris_seed, ils_seed):
    boxes = make_debris(debris_seed, np.asarray(sites)[:, :2])
    res = {'boxes': boxes}
    inst2d = m2d.Instance(sites)
    res['2D'] = m2d.solve_all(inst2d, ils_seed)
    res['2D_inst'] = inst2d
    res['hover'] = solve_3d(sites, boxes, ils_seed, 'hover', Z_R)
    res['R&G'] = solve_3d(sites, boxes, ils_seed, 'rg', Z_R)
    res['R&G low'] = solve_3d(sites, boxes, ils_seed, 'rg', Z_R_LOW, strategies=['ILS'])
    return res


def release_sweep():
    """P(hit) of the solver release point vs release altitude, level flight
    heading +x, per drone class (common wind-error draws)."""
    out = {}
    for m, v, lab in SWEEP_CLASSES:
        z0 = ZR_SWEEP.copy()
        v0 = np.tile([v, 0.0, 0.0], (len(z0), 1))
        mc = mc_miss(z0, v0, np.full(len(z0), m))
        out[lab] = dict(p_hit=(mc['A'] <= R_LZ).mean(axis=1), tf=mc['tf'],
                        r99=np.percentile(mc['A'], 99.7, axis=1), m=m, v=v,
                        tf_card=np.sqrt(2 * z0 / G),
                        z_crit=(m * G / C_D) / G * math.log(math.cosh(
                            G * (R_LZ / (3 * SIGMA_W)) / math.sqrt(m * G / C_D))))
    return out


def run_simulation():
    card_sites = m2d.CARD_SITES
    card = solve_instance(card_sites, CARD_DEBRIS_SEED, ILS_SEED)
    study = []
    show = None
    for seed in STUDY_SEEDS:
        inst = m2d.make_instance(seed)
        res = solve_instance(inst.sites, GEN_DEBRIS_SEED + seed, ILS_SEED + seed)
        res['seed'] = seed
        study.append(res)
        if seed == SHOWCASE_SEED:
            show = res
    return dict(card=card, show=show, study=study, sweep=release_sweep())


# ── Plots ─────────────────────────────────────────────────────────────────────
def _box_edges(b):
    x0, x1, y0, y1, z0, z1 = b
    c = np.array([[x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0],
                  [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1]])
    e = [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4),
         (0, 4), (1, 5), (2, 6), (3, 7)]
    return [(c[a], c[b]) for a, b in e]


def _draw_scene(ax, W, boxes, served=None):
    for b in boxes:
        for a, c in _box_edges(b):
            ax.plot(*zip(a, c), color='gray', lw=0.9, alpha=0.8)
    for i in range(W.M):
        fill = served is None or i in served
        ax.scatter(*W.sites[i, :2], 0.0, s=25 + 15 * W.q[i],
                   c=PRIORITY_COLORS[W.prio[i]] if fill else 'white',
                   edgecolors=PRIORITY_COLORS[W.prio[i]] if not fill else 'black',
                   linewidths=1.2 if not fill else 0.4, depthshade=False)
    ax.scatter(*FOB3, marker='*', s=250, c='black', depthshade=False)
    ax.quiver(FOB3[0], FOB3[1], 30.0, WIND[0] * 25, WIND[1] * 25, 0.0, color='teal',
              lw=2, arrow_length_ratio=0.25)
    ax.set_xlim(0, 1000)
    ax.set_ylim(0, 1000)
    ax.set_zlim(0, 40)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')


def plot_mission_map_3d(data, out_dir):
    res = data['show']
    fig = plt.figure(figsize=(16, 7))
    for col, key in enumerate(['hover', 'R&G']):
        W = res[key]['world']
        ev = res[key]['ILS']
        ax = fig.add_subplot(1, 2, col + 1, projection='3d')
        _draw_scene(ax, W, res['boxes'], ev['served'])
        for (k, r), (full, _) in ev['geo']['paths'].items():
            ax.plot(full[:, 0], full[:, 1], full[:, 2], color=DRONE_COLORS[k], lw=1.1,
                    ls='-' if r == 0 else ('--' if r == 1 else ':'))
        if key == 'R&G':
            p = ev['geo']['p']
            ax.scatter(p[:, 0], p[:, 1], p[:, 2], marker='v', c='k', s=12, depthshade=False)
        ax.view_init(elev=28, azim=-60)
        ax.set_title(f'ILS, {"hover drop (tau = 8 s)" if key == "hover" else "release-and-go (tau = 2 s)"}\n'
                     f'J = {ev["J"]:.2f}, served {len(ev["served"])}/{W.M}, '
                     f'{ev["n_sorties"]} sorties, {ev["n_detour_wp"]} detour waypoints',
                     fontsize=10)
    handles = [mpatches.Patch(color=PRIORITY_COLORS[p], label=l) for p, l in
               ((1, 'Critical'), (2, 'Urgent'), (3, 'Standard'))]
    handles += [Line2D([], [], color=DRONE_COLORS[k], label=f'Drone {k}') for k in range(K_DRONES)]
    handles += [Line2D([], [], color='gray', label='Debris AABB'),
                Line2D([], [], color='teal', lw=2, label='Wind (3, 2) m/s'),
                Line2D([], [], marker='*', color='black', ls='', ms=12, label='FOB'),
                Line2D([], [], marker='v', color='black', ls='', label='Release point'),
                Line2D([], [], marker='o', mfc='white', mec='red', ls='', label='Unserved')]
    fig.legend(handles=handles, loc='lower center', ncol=8, fontsize=8)
    fig.suptitle(f'S038 3D — Showcase instance (seed {SHOWCASE_SEED}), 3-D mission map',
                 fontsize=13, fontweight='bold')
    plt.tight_layout(rect=[0, 0.06, 1, 0.95])
    path = os.path.join(out_dir, 'mission_map_3d.png')
    plt.savefig(path, dpi=120)
    plt.close()
    print(f'Saved: {path}')


def plot_release_points(data, out_dir):
    res = data['card']
    W = res['R&G']['world']
    ev = res['R&G']['ILS']
    mc = ev['mc']
    geo = ev['geo']
    fig, axes = plt.subplots(2, 4, figsize=(17, 9.5))
    order = np.argsort(mc['sites'])
    for ax, e in zip(axes.ravel(), order):
        i = mc['sites'][e]
        s = W.sites[i, :2]
        pA = s - mc['off_nom'][e]
        pB = s - mc['off_card'][e]
        landA = pA + mc['off_true'][e]
        landB = pB + mc['off_true'][e]
        ax.scatter(landA[:, 0] - s[0], landA[:, 1] - s[1], s=2, c='tab:green', alpha=0.4)
        ax.scatter(landB[:, 0] - s[0], landB[:, 1] - s[1], s=2, c='tab:orange', alpha=0.4)
        ax.add_patch(plt.Circle((0, 0), R_LZ, fill=False, color='green', lw=1.5))
        ax.scatter(0, 0, marker='+', c='k', s=80, zorder=5)
        ax.scatter(*(pA - s), marker='v', c='tab:green', edgecolors='k', s=60, zorder=5)
        ax.scatter(*(pB - s), marker='v', c='tab:orange', edgecolors='k', s=60, zorder=5)
        vh = geo['vel'][e, :2]
        ax.annotate('', xy=(pA - s) + vh * 0.6, xytext=pA - s,
                    arrowprops=dict(arrowstyle='->', color='tab:red'))
        ax.annotate('', xy=(18, -30) + WIND[:2] * 3, xytext=(18, -30),
                    arrowprops=dict(arrowstyle='->', color='teal'))
        ax.set_xlim(-50, 30)
        ax.set_ylim(-40, 30)
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.set_title(f'S{i} ({W.q[i]:.1f} kg, drone {geo["entries"][e][0]}): '
                     f'P(hit) solver {mc["p_hit"]["A"][e]*100:.0f}%, '
                     f'card {mc["p_hit"]["B"][e]*100:.0f}%', fontsize=8.5)
        ax.set_xlabel('x - x_site (m)', fontsize=8)
        ax.set_ylabel('y - y_site (m)', fontsize=8)
    handles = [Line2D([], [], marker='+', c='k', ls='', ms=10, label='Site centre'),
               Line2D([], [], marker='v', c='tab:green', mec='k', ls='', label='Solver release point'),
               Line2D([], [], marker='v', c='tab:orange', mec='k', ls='', label='Card closed-form release point'),
               Line2D([], [], marker='.', c='tab:green', ls='', label='Landings (solver)'),
               Line2D([], [], marker='.', c='tab:orange', ls='', label='Landings (card formula)'),
               Line2D([], [], c='green', label=f'r_LZ = {R_LZ:.0f} m'),
               Line2D([], [], c='tab:red', label='Drone velocity at release (x0.6 s)'),
               Line2D([], [], c='teal', label='Wind (x3 s)')]
    fig.legend(handles=handles, loc='lower center', ncol=4, fontsize=9)
    fig.suptitle(f'S038 3D — Release points and landing scatter, card instance, ILS release-and-go '
                 f'(z_r = {Z_R:.0f} m, {N_MC} wind-error samples, sigma_w = {SIGMA_W} m/s)',
                 fontsize=12, fontweight='bold')
    plt.tight_layout(rect=[0, 0.08, 1, 0.95], h_pad=2.5)
    path = os.path.join(out_dir, 'release_points.png')
    plt.savefig(path, dpi=110)
    plt.close()
    print(f'Saved: {path}')


def plot_landing_scatter(data, out_dir):
    res = data['card']
    W = res['R&G']['world']
    mc = res['R&G']['ILS']['mc']
    mch = res['hover']['ILS']['mc']
    fig, axes = plt.subplots(1, 2, figsize=(13, 6.2))
    for ax, m_, lab in ((axes[0], mc, f'release-and-go from z_r = {Z_R:.0f} m (solver point)'),
                        (axes[1], mch, f'hover drop from {Z_HOVER_REL:.0f} m')):
        for e, i in enumerate(m_['sites']):
            d = m_['off_true'][e] - m_['off_nom'][e]
            ax.scatter(d[:, 0], d[:, 1], s=2, alpha=0.35, color=PRIORITY_COLORS[W.prio[i]])
        ax.add_patch(plt.Circle((0, 0), R_LZ, fill=False, color='green', lw=2))
        ph = m_['p_hit']['A']
        ax.set_title(f'{lab}\nP(hit) per drop {ph.min()*100:.1f}-{ph.max()*100:.1f}%, '
                     f'mean {ph.mean()*100:.1f}%', fontsize=10)
        ax.set_xlim(-7, 7)
        ax.set_ylim(-7, 7)
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.set_xlabel('touchdown x - site x (m)')
        ax.set_ylabel('touchdown y - site y (m)')
    axes[0].legend(handles=[mpatches.Patch(color=PRIORITY_COLORS[p], label=l) for p, l in
                            ((1, 'Critical site'), (2, 'Urgent site'), (3, 'Standard site'))]
                   + [Line2D([], [], color='green', lw=2, label='r_LZ = 3 m')],
                   fontsize=8, loc='upper left')
    fig.suptitle('S038 3D — Touchdown scatter under wind-estimation error (card instance, ILS)',
                 fontsize=12, fontweight='bold')
    plt.tight_layout()
    path = os.path.join(out_dir, 'landing_scatter.png')
    plt.savefig(path, dpi=120)
    plt.close()
    print(f'Saved: {path}')


def plot_release_altitude(data, out_dir):
    sw = data['sweep']
    fig, axes = plt.subplots(1, 2, figsize=(13, 5))
    cols = ['tab:purple', 'tab:green', 'tab:blue']
    for c, (lab, d) in zip(cols, sw.items()):
        axes[0].plot(ZR_SWEEP, d['p_hit'] * 100, 'o-', color=c, label=lab)
        axes[0].axvline(d['z_crit'], color=c, ls=':', lw=1.2)
        axes[1].plot(ZR_SWEEP, d['tf'], 'o-', color=c, label=f'{lab} (drag)')
    axes[1].plot(ZR_SWEEP, sw[SWEEP_CLASSES[0][2]]['tf_card'], 'k--',
                 label='sqrt(2 z / g) (card low-drag approx.)')
    axes[1].axhline(R_LZ / (3 * SIGMA_W), color='gray', ls=':',
                    label='card criterion 3 sigma_w T_fall = r_LZ')
    for ax in axes:
        ax.axvline(Z_R, color='red', ls='--', lw=1)
        ax.grid(True, alpha=0.3)
        ax.set_xlabel('Release altitude z_r (m)')
    axes[0].set_ylabel('P(touchdown within 3 m) (%)')
    axes[0].set_title('Solver release point: hit rate vs altitude\n'
                      '(dotted: max z_r from the card criterion 3 sigma_w T_fall <= r_LZ)',
                      fontsize=10)
    axes[0].legend(fontsize=8)
    axes[1].set_ylabel('Fall time T_fall (s)')
    axes[1].set_title('Fall time: drag model vs card approximation', fontsize=10)
    axes[1].legend(fontsize=8)
    fig.suptitle('S038 3D — Release altitude trade-off (level release heading +x, '
                 f'{N_MC} wind-error samples; red dashed = card z_r = {Z_R:.0f} m)',
                 fontsize=12, fontweight='bold')
    plt.tight_layout()
    path = os.path.join(out_dir, 'release_altitude.png')
    plt.savefig(path, dpi=120)
    plt.close()
    print(f'Saved: {path}')


def plot_altitude_profile(data, out_dir):
    res = data['show']
    fig, axes = plt.subplots(2, 1, figsize=(13, 7.5), sharex=True)
    for ax, key in zip(axes, ['R&G', 'R&G low']):
        ev = res[key]['ILS']
        Wk = res[key]['world']
        for k in range(K_DRONES):
            for r, s in enumerate(ev['timeline'][k]):
                ax.plot(s['kn_t'], s['kn_p'][:, 2], color=DRONE_COLORS[k], lw=1.2,
                        label=f'Drone {k}' if r == 0 else None)
        for i, t in ev['release_t'].items():
            ax.scatter(t, Wk.z_node, marker='v', s=25,
                       color=PRIORITY_COLORS[Wk.prio[i]], zorder=5)
        ax.axhline(max(b[5] for b in res['boxes']), color='gray', ls=':', lw=1,
                   label='Tallest debris top')
        ax.axhline(Z_BOUNDS[0], color='k', ls='--', lw=0.6)
        ax.axvline(T_MAX, color='k', ls='--', lw=1)
        ax.set_ylim(0, 25)
        ax.set_ylabel('z (m)')
        ax.grid(True, alpha=0.3)
        ax.set_title(f'ILS release-and-go, z_r = {Wk.z_node:.0f} m: '
                     f'{ev["n_detour_wp"]} detour waypoints, J = {ev["J"]:.2f}', fontsize=10)
    axes[0].legend(fontsize=8, ncol=7, loc='upper right')
    axes[1].set_xlabel('Time (s)')
    fig.suptitle('S038 3D — Altitude profiles, showcase instance (triangles = releases, '
                 'coloured by priority)', fontsize=12, fontweight='bold')
    plt.tight_layout()
    path = os.path.join(out_dir, 'altitude_profile.png')
    plt.savefig(path, dpi=120)
    plt.close()
    print(f'Saved: {path}')


def plot_debris_check(data, out_dir):
    res = data['show']
    ev = res['R&G']['ILS']
    boxes = res['boxes']
    fig, axes = plt.subplots(1, 2, figsize=(15, 6.5), gridspec_kw=dict(width_ratios=[1, 1.3]))
    ax = axes[0]
    for b in boxes:
        ax.add_patch(mpatches.Rectangle((b[0], b[2]), b[1] - b[0], b[3] - b[2],
                                        color='gray', alpha=0.45))
        ax.text(0.5 * (b[0] + b[1]), 0.5 * (b[2] + b[3]), f'{b[5]:.1f} m',
                ha='center', va='center', fontsize=7)
    blocked = []
    for (k, r), (full, idx) in sorted(ev['geo']['paths'].items()):
        for a, bidx in zip(idx[:-1], idx[1:]):
            A, B = full[a], full[bidx]
            hit = any(seg_hits_box(A, B, bx) for bx in boxes)
            ax.plot([A[0], B[0]], [A[1], B[1]], color='red' if hit else 'lightsteelblue',
                    lw=1.6 if hit else 0.8, zorder=3 if hit else 2)
            if hit:
                blocked.append((k, A, B, full[a:bidx + 1]))
    ax.scatter(*FOB3[:2], marker='*', s=250, c='black', zorder=6)
    ax.scatter(res['R&G']['world'].sites[:, 0], res['R&G']['world'].sites[:, 1], s=15,
               c='k', zorder=5)
    ax.set_xlim(0, 1000)
    ax.set_ylim(0, 1000)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.set_title(f'Top view: {len(blocked)} of the plan\'s straight legs hit debris (red)',
                 fontsize=10)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax = axes[1]
    leg_cols = plt.cm.tab10(np.arange(10))
    for n, (k, A, B, sub) in enumerate(blocked):
        u = (B[:2] - A[:2]) / np.linalg.norm(B[:2] - A[:2])
        s_str = np.array([0.0, np.linalg.norm(B[:2] - A[:2])])
        c = leg_cols[n % 10]
        ax.plot(s_str, [A[2], B[2]], '--', color=c, lw=0.9)
        s_det = (sub[:, :2] - A[:2]) @ u
        ax.plot(s_det, sub[:, 2], '-', color=c, lw=1.8,
                label=f'Leg {n + 1} (drone {k}, {"climb-out" if A[2] < B[2] else "return"})')
        for bx in boxes:
            if seg_hits_box(A, B, bx):
                # along-track span of the box on this leg
                ts = np.linspace(0, 1, 400)
                pts = A[None, :2] + ts[:, None] * (B[:2] - A[:2])[None, :]
                inside = ((pts[:, 0] >= bx[0]) & (pts[:, 0] <= bx[1]) &
                          (pts[:, 1] >= bx[2]) & (pts[:, 1] <= bx[3]))
                if inside.any():
                    s0 = ts[inside].min() * s_str[1]
                    s1 = ts[inside].max() * s_str[1]
                    ax.add_patch(mpatches.Rectangle((s0, 0), s1 - s0, bx[5], color='gray',
                                                    alpha=0.25))
    ax.set_xlabel('Along-track distance from leg start (m)')
    ax.set_ylabel('z (m)')
    ax.set_ylim(0, 25)
    if blocked:
        ax.legend(fontsize=8, loc='upper right')
    ax.grid(True, alpha=0.3)
    ax.set_title('Side view of the blocked legs: straight (dashed) vs detoured (solid), '
                 'grey = debris span on the leg', fontsize=10)
    fig.suptitle(f'S038 3D — Debris avoidance check (showcase, ILS release-and-go, '
                 f'z_r = {Z_R:.0f} m)', fontsize=12, fontweight='bold')
    plt.tight_layout()
    path = os.path.join(out_dir, 'debris_check.png')
    plt.savefig(path, dpi=120)
    plt.close()
    print(f'Saved: {path}')


def plot_value_timeline(data, out_dir):
    res = data['show']
    W = res['hover']['world']
    fig, ax = plt.subplots(figsize=(11, 5.5))
    curves = [('2D Greedy (2-D model)', res['2D']['Greedy'], WORLD_COLORS['2D'], ':'),
              ('2D ILS (2-D model)', res['2D']['ILS'], WORLD_COLORS['2D'], '-'),
              ('3D ILS, hover tau = 8 s', res['hover']['ILS'], WORLD_COLORS['hover'], '-'),
              (f'3D ILS, release-and-go z_r = {Z_R:.0f} m', res['R&G']['ILS'],
               WORLD_COLORS['R&G'], '-')]
    for lab, r, c, ls in curves:
        times = r['arrivals'] if 'arrivals' in r else r['deliv']
        ev = sorted((t, W.value(i, t)) for i, t in times.items()
                    if t <= T_MAX and i in r['served'])
        ts = [0.0] + [e[0] for e in ev] + [T_MAX]
        cum = np.concatenate([[0.0], np.cumsum([e[1] for e in ev]), [sum(e[1] for e in ev)]])
        ax.step(ts, cum, where='post', color=c, ls=ls, lw=2,
                label=f'{lab}: J = {r["J"]:.2f}, served {len(r["served"])}/{W.M}')
    ax.axhline(W.upper_bound, color='k', ls='--', lw=1, label=f'Upper bound {W.upper_bound:.2f}')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Cumulative delivery value (before unserved penalty)')
    ax.set_title(f'S038 3D — Cumulative delivery value, showcase instance (seed {SHOWCASE_SEED})')
    ax.legend(fontsize=8.5, loc='upper left')
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'value_timeline.png')
    plt.savefig(path, dpi=130)
    plt.close()
    print(f'Saved: {path}')


def plot_study(data, out_dir):
    study = data['study']
    fig, axes = plt.subplots(2, 1, figsize=(13, 8.5), sharex=True)
    x = np.arange(len(study))
    keys = [('2D', '2D model'), ('hover', '3D hover'), ('R&G', f'3D R&G z_r = {Z_R:.0f} m'),
            ('R&G low', f'3D R&G z_r = {Z_R_LOW:.0f} m')]
    wdt = 0.2
    for j, (key, lab) in enumerate(keys):
        Js = [s[key]['ILS']['J'] for s in study]
        axes[0].bar(x + (j - 1.5) * wdt, Js, wdt, color=WORLD_COLORS[key], label=lab,
                    edgecolor='k', lw=0.3)
        if key != '2D':
            Je = [s[key]['ILS']['J_exp'] for s in study]
            axes[1].bar(x + (j - 1.5) * wdt, Je, wdt, color=WORLD_COLORS[key], label=lab,
                        edgecolor='k', lw=0.3)
    axes[0].set_ylabel('J (nominal, every drop lands)')
    axes[1].set_ylabel('Expected J (drop outside r_LZ = unserved)')
    for ax in axes:
        ax.axhline(0, color='k', lw=0.8)
        ax.grid(True, axis='y', alpha=0.3)
        ax.legend(fontsize=8, ncol=4)
    axes[1].set_xticks(x)
    axes[1].set_xticklabels([f'seed {s["seed"]}' for s in study])
    fig.suptitle('S038 3D — ILS on 10 generated instances: 2-D model vs 3-D protocols',
                 fontsize=12, fontweight='bold')
    plt.tight_layout()
    path = os.path.join(out_dir, 'instance_study.png')
    plt.savefig(path, dpi=120)
    plt.close()
    print(f'Saved: {path}')


def save_animation(data, out_dir):
    res = data['show']
    W = res['R&G']['world']
    ev = res['R&G']['ILS']
    geo = ev['geo']
    # nominal package trajectories (release frame -> world)
    land, tf, traj = simulate_drops(geo['p'][:, 2], geo['vel'], geo['m'], WIND, keep_traj=True)
    rel_t = np.array([ev['release_t'][e[3]] for e in geo['entries']])
    t_end = max(s['t_ret'] for k in range(K_DRONES) for s in ev['timeline'][k])
    t_grid = np.arange(0.0, t_end + 1e-9, 1.0)
    if t_grid[-1] < t_end:
        t_grid = np.append(t_grid, t_end)
    n = len(t_grid)
    step = int(GIF_DT)
    n_frames = math.ceil((n - 1) / step) + 1

    def drone_pos(k, t):
        for s in ev['timeline'][k]:
            if s['kn_t'][0] <= t <= s['kn_t'][-1]:
                return np.array([np.interp(t, s['kn_t'], s['kn_p'][:, a]) for a in range(3)])
        return FOB3.copy()

    fig = plt.figure(figsize=(8.5, 7))
    ax = fig.add_subplot(111, projection='3d')
    _draw_scene(ax, W, res['boxes'], served=set())
    for (k, r), (full, _) in geo['paths'].items():
        ax.plot(full[:, 0], full[:, 1], full[:, 2], color=DRONE_COLORS[k], lw=0.5, alpha=0.35)
    ax.view_init(elev=30, azim=-60)
    dr = ax.scatter(*np.tile(FOB3, (K_DRONES, 1)).T, c=DRONE_COLORS, s=40, marker='D',
                    depthshade=False)
    pk = ax.scatter([], [], [], c='saddlebrown', s=25, depthshade=False)
    filled = ax.scatter([], [], [], c='limegreen', s=50, edgecolors='k', depthshade=False)
    arcs = []
    title = ax.set_title('')

    def update(f):
        t = t_grid[min(f * step, n - 1)]
        P = np.array([drone_pos(k, t) for k in range(K_DRONES)])
        dr._offsets3d = (P[:, 0], P[:, 1], P[:, 2])
        falling, done = [], []
        for e in range(len(rel_t)):
            if t >= rel_t[e]:
                tr = traj[:, e, :] + np.array([geo['p'][e, 0], geo['p'][e, 1], 0.0])
                kk = min(int(round((t - rel_t[e]) / DT_BAL)), len(tr) - 1)
                if t < rel_t[e] + tf[e]:
                    falling.append(tr[kk])
                else:
                    done.append(e)
                if len(arcs) <= e or arcs[e] is None:
                    while len(arcs) <= e:
                        arcs.append(None)
                    nn = int(math.ceil(tf[e] / DT_BAL)) + 1
                    arcs[e] = ax.plot(tr[:nn, 0], tr[:nn, 1], np.maximum(tr[:nn, 2], 0),
                                      color='saddlebrown', lw=1.0)[0]
        F = np.array(falling) if falling else np.zeros((0, 3))
        pk._offsets3d = (F[:, 0], F[:, 1], F[:, 2])
        S = np.array([[*W.sites[geo['entries'][e][3], :2], 0.0] for e in done]) \
            if done else np.zeros((0, 3))
        filled._offsets3d = (S[:, 0], S[:, 1], S[:, 2])
        title.set_text(f'S038 3D — ILS release-and-go, showcase, t = {t:.0f} s, '
                       f'delivered {len(done)}/{len(rel_t)}')
        return [dr, pk, filled, title]

    ani = FuncAnimation(fig, update, frames=n_frames, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer=PillowWriter(fps=GIF_FPS), dpi=72)
    plt.close()
    print(f'Saved: {path} ({n_frames} frames)')


# ── Report ────────────────────────────────────────────────────────────────────
def _row(lab, r, M, n_crit, prio):
    crit = sum(1 for i in r['served'] if prio[i] == 1)
    late = r['late']
    ml = float(np.mean(late)) if late else 0.0
    return (f'  {lab:<28} J {r["J"]:>8.3f}  served {len(r["served"]):>2}/{M}  '
            f'crit {crit}/{n_crit}  late {sum(1 for l in late if l > 0):>2}  '
            f'meanLate {ml:>6.1f} s  sorties {r["n_sorties"]:>2}')


def unreachable(W):
    return [i for i in range(W.M)
            if not any(W.q[i] <= FLEET[k, 0] and sortie_feasible(W, k, [i])
                       for k in range(K_DRONES))]


def print_instance(title, res):
    W = res['hover']['world']
    n_crit = int((W.prio == 1).sum())
    print(f'\n=== {title} ===')
    print(f'Sites {W.M}, demand {W.q.sum():.1f} kg, upper bound {W.upper_bound:.3f}')
    print('Debris AABBs [x0, x1, y0, y1, top]:')
    for b in res['boxes']:
        print(f'  [{b[0]:6.1f}, {b[1]:6.1f}, {b[2]:6.1f}, {b[3]:6.1f}, {b[5]:5.2f}]')
    for key in ('hover', 'R&G', 'R&G low'):
        Wk = res[key]['world']
        print(f'  {key}: node pairs needing a detour {Wk.n_blocked_pairs}/'
              f'{(Wk.M + 1) * Wk.M // 2}')
    un = unreachable(W)
    print('  Sites no drone can serve in 3-D (payload range penalty): ' +
          (', '.join(f'S{i} ({W.q[i]:.1f} kg, round trip {2 * W.D[W.F][i]:.0f} m)' for i in un)
           if un else 'none'))
    for name in ('ILS',):
        for key in ('2D', 'hover', 'R&G', 'R&G low'):
            r = res[key][name]
            print(f'  {name} {key:<8} unserved: ' + (', '.join(
                f'S{i}' for i in sorted(set(range(W.M)) - r['served'])) or 'none'))
    for name in STRATS:
        print(f' {name}:')
        print(_row('2D model (tau 8 s)', res['2D'][name], W.M, n_crit, W.prio))
        for key, lab in (('hover', '3D hover (tau 8 s)'), ('R&G', f'3D R&G z_r {Z_R:.0f} m'),
                         ('R&G low', f'3D R&G z_r {Z_R_LOW:.0f} m')):
            if name not in res[key]:
                continue
            r = res[key][name]
            print(_row(lab, r, W.M, n_crit, W.prio) +
                  f'  | J_plan {r["J_plan"]:.3f}, expected J {r["J_exp"]:.3f}, '
                  f'P(hit) mean {r["mc"]["p_hit"]["A"].mean()*100:.1f}% '
                  f'(min {r["mc"]["p_hit"]["A"].min()*100:.1f}%), '
                  f'detour wp {r["n_detour_wp"]}, detour +{r["detour_extra"]:.1f} m, '
                  f'flown {r["total_len"]:.0f} m, repairs {r["n_repair"]}')


if __name__ == '__main__':
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    # sanity: drag integrator vs analytic vertical fall
    print('Ballistic check (vertical release from rest, no wind):')
    for m in (1.0, 2.5, 4.0):
        _, tfn = simulate_drops(np.array([Z_R]), np.zeros((1, 3)), np.array([m]), np.zeros(3))
        print(f'  m = {m:.1f} kg: v_t = {math.sqrt(m * G / C_D):.2f} m/s, '
              f'T_fall RK4 {tfn[0]:.4f} s, analytic {tfall_analytic(Z_R, m):.4f} s, '
              f'low-drag sqrt(2z/g) {math.sqrt(2 * Z_R / G):.4f} s, '
              f'card 3 sigma_w T_fall = {3 * SIGMA_W * tfall_analytic(Z_R, m):.2f} m')

    data = run_simulation()
    card, show, study = data['card'], data['show'], data['study']

    print_instance('Card instance (8 sites, debris seed 38)', card)
    print_instance(f'Showcase generated instance (seed {SHOWCASE_SEED})', show)

    # landing accuracy of the three release-point methods, card instance
    for key in ('R&G', 'R&G low', 'hover'):
        ev = card[key]['ILS']
        mc = ev['mc']
        W = card[key]['world']
        print(f'\nLanding accuracy, card instance, ILS {key} '
              f'(release z = {W.z_node if key != "hover" else Z_HOVER_REL:.0f} m):')
        print('  site  mass  T_fall  |offset solver|  |offset card|  nominal miss card  '
              'P(hit) solver / card / overhead   99.7% miss solver')
        for e in np.argsort(mc['sites']):
            i = mc['sites'][e]
            print(f'  S{i:<3} {W.q[i]:4.1f}  {mc["tf"][e]:5.2f} s  '
                  f'{np.linalg.norm(mc["off_nom"][e]):10.2f} m  '
                  f'{np.linalg.norm(mc["off_card"][e]):10.2f} m  '
                  f'{np.linalg.norm(mc["off_card"][e] - mc["off_nom"][e]):12.2f} m  '
                  f'{mc["p_hit"]["A"][e]*100:6.1f}% / {mc["p_hit"]["B"][e]*100:5.1f}% / '
                  f'{mc["p_hit"]["C"][e]*100:5.1f}%   {np.percentile(mc["A"][e], 99.7):6.2f} m')

    print('\nRelease-altitude sweep (solver point, level flight heading +x):')
    for lab, d in data['sweep'].items():
        print(f'  {lab}: card-criterion max z_r = {d["z_crit"]:.1f} m')
        for z, ph, t in zip(ZR_SWEEP, d['p_hit'], d['tf']):
            if z in (5.0, 10.0, 15.0, 18.0, 20.0, 30.0, 40.0):
                print(f'    z_r {z:5.1f} m: T_fall {t:5.2f} s, P(hit) {ph*100:5.1f}%, '
                      f'99.7% miss {d["r99"][list(ZR_SWEEP).index(z)]:.2f} m')

    print(f'\n=== Study: {len(study)} generated instances, ILS ===')
    print(f'{"seed":>4} {"2D":>8} {"hover":>8} {"R&G18":>8} {"R&G10":>8}   '
          f'{"Eh":>8} {"E18":>8} {"E10":>8}   served 2D/h/18/10')
    for s in study:
        print(f'{s["seed"]:>4} ' + ' '.join(f'{s[k]["ILS"]["J"]:>8.2f}'
                                         for k in ('2D', 'hover', 'R&G', 'R&G low')) + '   ' +
              ' '.join(f'{s[k]["ILS"]["J_exp"]:>8.2f}' for k in ('hover', 'R&G', 'R&G low')) +
              '   ' + '/'.join(str(len(s[k]['ILS']['served']))
                               for k in ('2D', 'hover', 'R&G', 'R&G low')))
    for k in ('2D', 'hover', 'R&G', 'R&G low'):
        Js = np.array([s[k]['ILS']['J'] for s in study])
        line = f'  {k:<8} mean J {Js.mean():7.2f} (std {Js.std():5.2f})'
        if k != '2D':
            Je = np.array([s[k]['ILS']['J_exp'] for s in study])
            line += f', mean expected J {Je.mean():7.2f} (std {Je.std():5.2f})'
        print(line)

    def wins(a, b, field='J', strat='ILS'):
        return sum(1 for s in study if s[a][strat][field] > s[b][strat][field] + 1e-9)

    print(f'  R&G18 > hover (J): {wins("R&G", "hover")}/10; hover > 2D (J): {wins("hover", "2D")}/10; '
          f'R&G18 > 2D (J): {wins("R&G", "2D")}/10')
    print(f'  R&G18 > hover (expected J): {wins("R&G", "hover", "J_exp")}/10; '
          f'R&G10 > R&G18 (J): {wins("R&G low", "R&G")}/10; '
          f'R&G10 > R&G18 (expected J): {wins("R&G low", "R&G", "J_exp")}/10; '
          f'R&G10 > hover (expected J): {wins("R&G low", "hover", "J_exp")}/10')
    d = np.array([s['R&G low']['ILS']['J_exp'] - s['hover']['ILS']['J_exp'] for s in study])
    print(f'  expected J(R&G10) - J(hover): mean {d.mean():.2f}, min {d.min():.2f}, max {d.max():.2f}')
    d = np.array([s['R&G']['ILS']['J_exp'] - s['hover']['ILS']['J_exp'] for s in study])
    print(f'  expected J(R&G18) - J(hover): mean {d.mean():.2f}, min {d.min():.2f}, max {d.max():.2f}')
    ph = np.concatenate([s['R&G']['ILS']['mc']['p_hit']['A'] for s in study])
    pl = np.concatenate([s['R&G low']['ILS']['mc']['p_hit']['A'] for s in study])
    print(f'  per-drop P(hit) over the study (ILS): z_r 18 m mean {ph.mean()*100:.1f}% '
          f'(min {ph.min()*100:.1f}%, {int((ph < 0.9).sum())}/{len(ph)} drops below 90%); '
          f'z_r 10 m mean {pl.mean()*100:.1f}% (min {pl.min()*100:.1f}%)')
    d = np.array([s['R&G']['ILS']['J'] - s['hover']['ILS']['J'] for s in study])
    print(f'  J(R&G18) - J(hover): mean {d.mean():.2f}, min {d.min():.2f}, max {d.max():.2f}')
    d = np.array([s['hover']['ILS']['J'] - s['2D']['ILS']['J'] for s in study])
    print(f'  J(3D hover) - J(2D):  mean {d.mean():.2f}, min {d.min():.2f}, max {d.max():.2f}')

    print('\n  All planners, mean J over the study (2D / hover / R&G18):')
    for name in STRATS:
        print(f'    {name:<7} ' + ' / '.join(
            f'{np.mean([s[k][name]["J"] for s in study]):.2f}' for k in ('2D', 'hover', 'R&G')) +
            f'; R&G18 > hover on {wins("R&G", "hover", strat=name)}/10')
    det = [s['R&G']['ILS']['n_detour_wp'] for s in study]
    ext = [s['R&G']['ILS']['detour_extra'] for s in study]
    print(f'  ILS R&G18 detour waypoints per instance: min {min(det)}, max {max(det)}; '
          f'extra length {min(ext):.1f}-{max(ext):.1f} m')
    det = [s['R&G low']['ILS']['n_detour_wp'] for s in study]
    ext = [s['R&G low']['ILS']['detour_extra'] for s in study]
    print(f'  ILS R&G10 detour waypoints per instance: min {min(det)}, max {max(det)}; '
          f'extra length {min(ext):.1f}-{max(ext):.1f} m')
    for s in study:
        un = unreachable(s['hover']['world'])
        if un:
            Ws = s['hover']['world']
            print(f'  seed {s["seed"]}: unreachable in 3-D: ' + ', '.join(
                f'S{i} ({Ws.q[i]:.1f} kg, round trip {2 * Ws.D[Ws.F][i]:.0f} m > heavy-lift R_eff '
                f'{Ws.r_eff(0, Ws.q[i]):.0f} m)' for i in un))
    reps = sum(s[k][n]['n_repair'] for s in study for k in ('hover', 'R&G')
               for n in STRATS) + sum(s['R&G low']['ILS']['n_repair'] for s in study)
    print(f'  Exact-evaluation range repairs over all study schedules: {reps}')
    pen = [card['R&G']['world'].r_eff(k, FLEET[k, 0]) for k in range(K_DRONES)]
    print(f'  R_eff at full payload per drone: ' + ', '.join(f'{p:.0f} m' for p in pen))

    plot_mission_map_3d(data, OUTPUT_DIR)
    plot_release_points(data, OUTPUT_DIR)
    plot_landing_scatter(data, OUTPUT_DIR)
    plot_release_altitude(data, OUTPUT_DIR)
    plot_altitude_profile(data, OUTPUT_DIR)
    plot_debris_check(data, OUTPUT_DIR)
    plot_value_timeline(data, OUTPUT_DIR)
    plot_study(data, OUTPUT_DIR)
    save_animation(data, OUTPUT_DIR)
