"""
S043 3D Upgrade — Confined Space Exploration
============================================
A rescue drone explores a four-floor collapsed building (30 x 30 x 10 m, 60 x 60 x 20
voxels of 0.5 m) with no prior map.  Every floor uses the S043 floor plan (outer walls,
internal walls with doorways, debris columns, fallen beam), extruded over the floor's free
layers, plus floor-specific rubble.  Floor slabs (layers 5, 10, 15) are solid except for
stairwell openings; one F0 room has a partly collapsed ceiling (only 1.0 m headroom, so
the drone must fly low) and one corner has a 0.5 m crawl space (impassable).

Sensing: 3D lidar, 128 beams (92 Fibonacci directions over the full sphere + a
36-beam horizontal ring), 2 m range, 5 cm range noise, sampled ray-march (0.05 m steps),
log-odds voxel map (+0.85 / -0.40, clamp [-2, 3.5]; free < 0.35, occupied > 0.65).

Body / passability: the drone occupies one voxel plus the voxel above it (1.0 m >=
h_min = 0.6 m of headroom; a 0.5 m voxel is wider than g_min = 0.4 m), so a node (i, j, k)
is traversable when voxels k and k+1 are free.  Motion follows the planned voxel path at
0.5 m/s horizontally and v_z,max = 0.3 m/s vertically; every move is checked against the
map (p < 0.65) and against the ground truth (assert).

Strategies (same building, start (2, 2, 0.5) m, lidar noise stream):
  slice   2D-slice baseline: nodes and frontiers restricted to the start layer (z fixed)
  near3d  3D frontier-nearest: nearest frontier node by travel time, any floor
  vp3d    3D frontier + vertical-passage planner: finish the current floor first; when it
          has no frontier left, take the nearest frontier on another floor with a
          -2.0 s cost bonus for frontiers in detected stairwells
Planning: Dijkstra (scipy csgraph) over known-free traversable nodes, 6-connected, edge
cost = travel time (1.0 s per horizontal voxel, 1.67 s per vertical voxel).

Usage:
    conda activate drones
    python src/03_environmental_sar/3d/s043_3d_confined_space.py
"""

import os
import importlib.util
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401
from matplotlib.animation import FuncAnimation, PillowWriter
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import dijkstra
from scipy.ndimage import label
from concurrent.futures import ProcessPoolExecutor

_HERE = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location('s043_2d', os.path.join(_HERE, '..', 's043_confined_space.py'))
m2d = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(m2d)

OUTPUT_DIR = os.path.normpath(os.path.join(
    _HERE, '..', '..', '..', 'outputs', '03_environmental_sar', '3d', 's043_3d_confined_space'))

# ── Parameters ──────────────────────────────────────────────────────────────
RES          = 0.5                 # m voxel
NX, NY, NZ   = 60, 60, 20
SLABS        = (5, 10, 15)         # floor-slab layers (layer 0 = ground slab, 19 = roof)
FLOOR_OF     = np.minimum(np.arange(NZ) // 5, 3)
SENSOR_R     = 2.0                 # m
N_FIB        = 92
N_RING       = 36
SIGMA_R      = 0.05                # m
RAY_STEP     = 0.05                # m
L_OCC, L_FREE, L_MIN, L_MAX = m2d.L_OCC, m2d.L_FREE, m2d.L_MIN, m2d.L_MAX
P_FREE, P_OCC = m2d.P_FREE_THRESH, m2d.P_OCC_THRESH
V_XY         = 0.5                 # m/s
V_Z          = 0.3                 # m/s
DT           = 0.1                 # s
T_MAX        = 600.0               # s
T_EXT        = 3600.0              # s — supplementary horizon (the card horizon is too short to leave F0)
STAIR_SPAN   = 2.5                 # m — card: a stairwell cluster spans >= 2.5 m vertically
VP_BONUS     = 2.0                 # s — stairwell priority bonus (card: +2.0 BFS cost offset)
START        = np.array([2.0, 2.0, 0.5])
SEEDS        = list(range(10))     # lidar-noise seeds 0-9
N_WORKERS    = 8                   # parallel processes (results do not depend on it)
ANIM_EVERY   = 300                 # steps (30 s)
W_H, W_V     = RES / V_XY, RES / V_Z   # edge times (s)

STRATS = ['slice', 'near3d', 'vp3d']
LABELS = {'slice': '2D-slice baseline', 'near3d': '3D frontier-nearest',
          'vp3d': '3D frontier + vertical-passage planner'}
COLORS = {'slice': 'tab:red', 'near3d': 'tab:orange', 'vp3d': 'tab:blue'}
OPENINGS = [('S1 F0-F1', 5, (30, 34), (25, 29)),      # (name, slab layer, x-cells, y-cells)
            ('S2 F1-F2', 10, (50, 54), (42, 46)),
            ('S3 F2-F3', 15, (5, 9), (5, 9))]


# ── Building ────────────────────────────────────────────────────────────────

def build_building():
    """True = occupied voxel, indexed [x, y, z]."""
    plan = m2d.build_ground_truth().T              # S043 plan, [x, y]
    V = np.zeros((NX, NY, NZ), dtype=bool)
    V[:, :, 0] = True                              # ground slab
    V[:, :, NZ - 1] = True                         # roof
    for s in SLABS:
        V[:, :, s] = True
    rng = np.random.default_rng(43)
    for f in range(4):
        k0, k1 = 5 * f + 1, min(5 * f + 5, NZ - 1)
        V[:, :, k0:k1] |= plan[:, :, None]
        if f > 0:                                  # floor-specific rubble (2x2 columns)
            for _ in range(6):
                i, j = rng.integers(3, NX - 5), rng.integers(3, NY - 5)
                V[i:i + 2, j:j + 2, k0:k1] = True
    for name, s, (x0, x1), (y0, y1) in OPENINGS:  # stairwell openings (cleared over both floors)
        V[x0:x1, y0:y1, s - 4:min(s + 5, NZ - 1)] = False   # never opens the roof (layer NZ-1)
    assert V[:, :, NZ - 1].all() and V[:, :, 0].all(), 'roof / ground slab must stay closed'
    # partly collapsed ceiling in F0 (room x 14-24, y 2-9): only layers 1-2 free (1.0 m)
    V[28:48, 4:18, 3:5] = True
    # crawl space in F0 (x 1-5, y 26-29): only layer 1 free (0.5 m) -> impassable
    V[2:10, 52:58, 2:5] = True
    return V


def traversable(free):
    """Node (i, j, k) valid when voxels k and k+1 are free."""
    t = np.zeros_like(free)
    t[:, :, :-1] = free[:, :, :-1] & free[:, :, 1:]
    return t


def build_graph(nodes):
    """6-connected graph over node mask; edge weight = travel time."""
    idx = -np.ones(nodes.shape, dtype=np.int64)
    pts = np.argwhere(nodes)
    idx[tuple(pts.T)] = np.arange(len(pts))
    rows, cols, w = [], [], []
    for ax, wt in ((0, W_H), (1, W_H), (2, W_V)):
        sl_a = [slice(None)] * 3; sl_b = [slice(None)] * 3
        sl_a[ax] = slice(0, -1); sl_b[ax] = slice(1, None)
        both = nodes[tuple(sl_a)] & nodes[tuple(sl_b)]
        a = idx[tuple(sl_a)][both]; b = idx[tuple(sl_b)][both]
        rows += [a, b]; cols += [b, a]; w += [np.full(len(a), wt)] * 2
    n = len(pts)
    if rows:
        g = coo_matrix((np.concatenate(w), (np.concatenate(rows), np.concatenate(cols))), shape=(n, n)).tocsr()
    else:
        g = coo_matrix((n, n)).tocsr()
    return g, idx, pts


def reachable_nodes(V, start_node, layer_only=None):
    nodes = traversable(~V)
    if layer_only is not None:
        m = np.zeros_like(nodes); m[:, :, layer_only] = nodes[:, :, layer_only]; nodes = m
    g, idx, pts = build_graph(nodes)
    d = dijkstra(g, indices=idx[start_node])
    R = np.zeros_like(nodes)
    R[tuple(pts[np.isfinite(d)].T)] = True
    return R


def node_voxels(R):
    """Free voxels that belong to a reachable node (the node voxel and its headroom voxel)."""
    vox = R.copy()
    vox[:, :, 1:] |= R[:, :, :-1]
    return vox


# ── Lidar ───────────────────────────────────────────────────────────────────

def beam_dirs():
    i = np.arange(N_FIB) + 0.5
    th = np.arccos(1 - 2 * i / N_FIB)                  # full sphere
    ph = np.pi * (1 + 5 ** 0.5) * i
    fib = np.column_stack([np.sin(th) * np.cos(ph), np.sin(th) * np.sin(ph), np.cos(th)])
    a = np.linspace(0, 2 * np.pi, N_RING, endpoint=False)
    ring = np.column_stack([np.cos(a), np.sin(a), np.zeros(N_RING)])
    return np.vstack([fib, ring])


DIRS = beam_dirs()
RS = np.arange(RAY_STEP, SENSOR_R + 1e-9, RAY_STEP)


def scan(L, V, pos, rng):
    """Cast all beams, update the log-odds map in place."""
    P = pos[None, None, :] + DIRS[:, None, :] * RS[None, :, None]        # (B, S, 3)
    ijk = np.floor(P / RES).astype(int)
    ijk[..., 0] = np.clip(ijk[..., 0], 0, NX - 1)
    ijk[..., 1] = np.clip(ijk[..., 1], 0, NY - 1)
    ijk[..., 2] = np.clip(ijk[..., 2], 0, NZ - 1)
    occ = V[ijk[..., 0], ijk[..., 1], ijk[..., 2]]
    hit_any = occ.any(axis=1)
    first = np.where(hit_any, occ.argmax(axis=1), len(RS))
    r_true = np.where(hit_any, RS[np.minimum(first, len(RS) - 1)], SENSOR_R)
    r_meas = np.clip(r_true + rng.normal(0, SIGMA_R, len(DIRS)), 0.0, SENSOR_R)
    lin = (ijk[..., 0] * NY + ijk[..., 1]) * NZ + ijk[..., 2]
    hit_lin = lin[np.arange(len(DIRS)), np.minimum(first, len(RS) - 1)]
    free_m = RS[None, :] < (r_meas[:, None] - RAY_STEP / 2)
    free_m &= lin != hit_lin[:, None]
    # one free update per (beam, voxel): a straight ray visits each voxel in one consecutive run
    first_of_run = np.ones(lin.shape, dtype=bool)
    first_of_run[:, 1:] = lin[:, 1:] != lin[:, :-1]
    Lf = L.reshape(-1)
    np.add.at(Lf, lin[free_m & first_of_run], L_FREE)
    hits = hit_lin[hit_any & (r_meas < SENSOR_R)]
    np.add.at(Lf, hits, L_OCC)
    np.clip(Lf, L_MIN, L_MAX, out=Lf)


def prob(L):
    return 1.0 / (1.0 + np.exp(-L))


def frontier_mask(p):
    free = p < P_FREE
    unk = (p >= P_FREE) & (p <= P_OCC)
    nb = np.zeros_like(unk)
    nb[1:] |= unk[:-1]; nb[:-1] |= unk[1:]
    nb[:, 1:] |= unk[:, :-1]; nb[:, :-1] |= unk[:, 1:]
    nb[:, :, 1:] |= unk[:, :, :-1]; nb[:, :, :-1] |= unk[:, :, 1:]
    return free & nb


def detect_passages(p):
    """Known-free openings in the slab layers -> list of (slab, centroid xy, area m^2, z span m)."""
    out = []
    free = p < P_FREE
    for s in SLABS:
        lab, n = label(free[:, :, s], structure=np.ones((3, 3)))
        for c in range(1, n + 1):
            cells = np.argwhere(lab == c)
            if len(cells) < 4:                       # < g_min x g_min = 0.16 m^2 -> ignore single voxels
                continue
            ci, cj = np.round(cells.mean(axis=0)).astype(int)
            col = free[ci, cj, :]
            k0 = s
            while k0 - 1 >= 0 and col[k0 - 1]:
                k0 -= 1
            k1 = s
            while k1 + 1 < NZ and col[k1 + 1]:
                k1 += 1
            if (k1 - k0 + 1) * RES >= STAIR_SPAN:
                out.append((s, (cells.mean(axis=0) + 0.5) * RES, len(cells) * RES ** 2, (k1 - k0 + 1) * RES))
    return out


# ── Mission ─────────────────────────────────────────────────────────────────

def node_of(pos):
    return tuple(np.floor(pos / RES).astype(int))


def centre(n):
    return (np.array(n) + 0.5) * RES


def run(V, reach_vox, strategy, seed, t_max=T_MAX, record=False):
    rng = np.random.default_rng([seed, 0])
    L = np.zeros(V.shape)
    pos = centre(node_of(START))
    k_fixed = node_of(START)[2]
    n_steps = int(round(t_max / DT)) + 1
    traj = [pos.copy()]
    cov_t, cov, cov_f = [], [], []
    path, goal = [], None
    passages, pass_log = [], []
    floor_switch = []
    n_plans = 0
    snaps, frames = {}, []
    exhausted_t = None
    next_plan_t = 0.0
    floors_vox = [reach_vox & (FLOOR_OF[None, None, :] == f) for f in range(4)]
    n_reach = reach_vox.sum()
    snap_steps = {int(round(tt / DT)) for tt in (0, 600, 1800, 3600) if tt <= t_max}
    last_floor = FLOOR_OF[node_of(pos)[2]]
    for step in range(n_steps):
        t = step * DT
        scan(L, V, pos, rng)
        cur = node_of(pos)
        at_node = bool(np.all(np.abs(pos - centre(cur)) <= 1e-9))
        sample = step % 10 == 0 or step == n_steps - 1
        plan_now = at_node and t >= next_plan_t and step < n_steps - 1
        if sample or plan_now or step in snap_steps or (record and step % ANIM_EVERY == 0):
            p = prob(L)                            # only evaluated when it is used
            known_free = p < P_FREE
        if sample:
            cov_t.append(t)
            cov.append(100 * np.sum(known_free & reach_vox) / n_reach)
            cov_f.append([100 * np.sum(known_free & fv) / max(fv.sum(), 1) for fv in floors_vox])
            for s, c, a, span in detect_passages(p):
                if not any(s == q[0] and np.linalg.norm(c - q[1]) < 1.5 for q in passages):
                    passages.append((s, c, a, span, t))
        if step in snap_steps:
            snaps[int(round(t))] = (p.copy(), pos.copy())
        if record and step % ANIM_EVERY == 0:
            frames.append((t, p.copy(), pos.copy()))
        if step == n_steps - 1:
            break
        # replan when idle at a node and the goal is gone / reached / path blocked
        if plan_now:
            frv = frontier_mask(p)
            fr = frv.copy()                        # frontier NODE: its own voxel or its headroom voxel borders unknown
            fr[:, :, :-1] |= frv[:, :, 1:]
            nodes = traversable(known_free)
            nodes[cur] = True
            if strategy == 'slice':
                m = np.zeros_like(nodes); m[:, :, k_fixed] = nodes[:, :, k_fixed]; nodes = m
            if path:
                nxt = path[0]
                if not (nodes[nxt] and p[nxt] < P_OCC and p[nxt[0], nxt[1], nxt[2] + 1] < P_OCC):
                    path = []
            if not path or goal is None or not (fr[goal] and nodes[goal]):
                fnodes = fr & nodes
                fnodes[cur] = False
                g, idx, pts = build_graph(nodes)
                d, pred = dijkstra(g, indices=idx[cur], return_predecessors=True)
                fi = idx[fnodes]
                fi = fi[fi >= 0]
                fi = fi[np.isfinite(d[fi])]
                path, goal = [], None
                if len(fi):
                    cost = d[fi].copy()
                    if strategy == 'vp3d':
                        fl = FLOOR_OF[pts[fi, 2]]
                        cf = FLOOR_OF[cur[2]]
                        on_floor = fl == cf
                        if on_floor.any():
                            cost = np.where(on_floor, cost, np.inf)
                        else:
                            for s, c, a, span, tt in passages:
                                near = (np.hypot(pts[fi, 0] * RES + 0.25 - c[0], pts[fi, 1] * RES + 0.25 - c[1]) < 1.5)
                                cost = np.where(near, cost - VP_BONUS, cost)
                    j = fi[np.argmin(cost)]
                    goal = tuple(pts[j])
                    seq = [j]
                    while seq[-1] != idx[cur]:
                        seq.append(pred[seq[-1]])
                    path = [tuple(pts[q]) for q in seq[::-1][1:]]
                    n_plans += 1
                else:
                    next_plan_t = t + 5.0          # nothing to explore: re-check every 5 s
                if not path and exhausted_t is None:
                    exhausted_t = t
        # move toward the next node (axis-aligned step, speed limits)
        if path:
            tgt = centre(path[0])
            dvec = tgt - pos
            ax = int(np.argmax(np.abs(dvec)))
            vmax = V_Z if ax == 2 else V_XY
            stepl = min(abs(dvec[ax]), vmax * DT)
            pos = pos.copy()
            pos[ax] += np.sign(dvec[ax]) * stepl
            if np.allclose(pos, tgt, atol=1e-9):
                pos = tgt
                n = path.pop(0)
                assert not V[n] and not V[n[0], n[1], n[2] + 1], 'entered an occupied voxel'
                assert prob(L[n]) < P_OCC, 'entered a mapped-occupied voxel'
                f_now = FLOOR_OF[n[2]]
                if f_now != last_floor:
                    floor_switch.append((t, last_floor, f_now))
                    last_floor = f_now
        traj.append(pos.copy())
    traj = np.array(traj)
    path_len = float(np.sum(np.linalg.norm(np.diff(traj, axis=0), axis=1)))
    return dict(cov_t=np.array(cov_t), cov=np.array(cov), cov_f=np.array(cov_f), traj=traj,
                path_len=path_len, passages=passages, floor_switch=floor_switch, n_plans=n_plans,
                snaps=snaps, frames=frames, exhausted_t=exhausted_t, final_p=prob(L))


def exh(r):
    return 'never' if r['exhausted_t'] is None else f"{r['exhausted_t']:.1f} s"


def cov_at(r, tt):
    i = np.searchsorted(r['cov_t'], tt + 1e-9) - 1
    return r['cov'][i]


def cov_f_at(r, tt):
    i = np.searchsorted(r['cov_t'], tt + 1e-9) - 1
    return r['cov_f'][i]


def _job(args):
    """One mission in a worker process (the building is rebuilt deterministically)."""
    strategy, seed, t_max, record = args
    V = build_building()
    reach_vox = node_voxels(reachable_nodes(V, node_of(START)))
    return run(V, reach_vox, strategy, seed, t_max=t_max, record=record)


def t_reach(res, level):
    i = np.nonzero(res['cov'] >= level)[0]
    return res['cov_t'][i[0]] if len(i) else np.nan


def run_simulation():
    V = build_building()
    start_node = node_of(START)
    R = reachable_nodes(V, start_node)
    reach_vox = node_voxels(R)
    R_slice = reachable_nodes(V, start_node, layer_only=start_node[2])
    print(f'Voxel grid {NX} x {NY} x {NZ} ({NX * RES:.0f} x {NY * RES:.0f} x {NZ * RES:.0f} m), '
          f'occupied {V.mean() * 100:.1f} %')
    print(f'Reachable nodes {R.sum()}, reachable free voxels (C_reach) {reach_vox.sum()}; per floor '
          + ', '.join(f'F{f} {np.sum(reach_vox & (FLOOR_OF[None, None, :] == f))}' for f in range(4)))
    print(f'Reachable within the start layer only: {R_slice.sum()} nodes = '
          f'{100 * node_voxels(R_slice).sum() / reach_vox.sum():.1f} % of C_reach (upper bound needs vertical sensing)')
    free = ~V
    crawl = free[2:10, 52:58, 1] & ~traversable(free)[2:10, 52:58, 1]
    low = traversable(free)[28:48, 4:18, 1:3]
    print(f'Low-ceiling room: traversable at layer 1 only ({int(low[:, :, 0].sum())} nodes, layer 2: '
          f'{int(low[:, :, 1].sum())}); crawl space: {int(crawl.sum())} free voxels with 0.5 m headroom, '
          f'reachable F0 nodes there {int(R[2:10, 52:58, 1:5].sum())}')
    for name, s, (x0, x1), (y0, y1) in OPENINGS:
        print(f'Opening {name}: slab layer {s} (z {s * RES:.1f} m), x {x0 * RES:.1f}-{x1 * RES:.1f} m, '
              f'y {y0 * RES:.1f}-{y1 * RES:.1f} m, area {(x1 - x0) * (y1 - y0) * RES ** 2:.1f} m^2')
    print(f'Card horizon check: {T_MAX:.0f} s at {V_XY} m/s is at most {T_MAX * V_XY:.0f} m of flight')
    # Every 3D run is flown for T_EXT; its first T_MAX seconds are identical to a T_MAX run (same
    # noise stream), so the card-horizon numbers are read from it.  The slice baseline is flown to
    # T_EXT for seed 0 only (its frontiers run out long before).  Runs are independent and are
    # executed in parallel processes; results are collected in a fixed order.
    jobs = []
    for seed in SEEDS:
        for s in STRATS:
            long_run = s != 'slice' or seed == SEEDS[0]
            jobs.append((s, seed, T_EXT if long_run else T_MAX, seed == SEEDS[0]))
    with ProcessPoolExecutor(max_workers=min(N_WORKERS, len(jobs))) as ex:
        outs = list(ex.map(_job, jobs))
    res = {s: [] for s in STRATS}
    for (s, seed, tm, rec), r in zip(jobs, outs):
        res[s].append(r)
    ext = {s: res[s][0] for s in STRATS}
    print(f'=== Card horizon {T_MAX:.0f} s, lidar seeds {SEEDS} ===')
    for i, seed in enumerate(SEEDS):
        for s in STRATS:
            r = res[s][i]
            cf = cov_f_at(r, T_MAX)
            print(f'seed {seed} [{s:6s}] coverage at {T_MAX:.0f} s {cov_at(r, T_MAX):.1f} % (F0 {cf[0]:.1f} / F1 {cf[1]:.1f} / '
                  f'F2 {cf[2]:.1f} / F3 {cf[3]:.1f} %), at 200/400 s {cov_at(r, 200):.1f}/{cov_at(r, 400):.1f} %, '
                  f'floor changes by {T_MAX:.0f} s {sum(1 for x in r["floor_switch"] if x[0] <= T_MAX)}')
    print(f'=== Summary at {T_MAX:.0f} s over {len(SEEDS)} seeds (mean, min-max) ===')
    cov600 = {s: np.array([cov_at(r, T_MAX) for r in res[s]]) for s in STRATS}
    for s in STRATS:
        c = cov600[s]
        print(f'[{s:6s}] coverage {c.mean():.1f} % ({c.min():.1f}-{c.max():.1f})')
    for a, b in [('near3d', 'slice'), ('vp3d', 'slice'), ('vp3d', 'near3d')]:
        d = cov600[a] - cov600[b]
        print(f'{a} vs {b}: higher in {int(np.sum(d > 0))}/{len(SEEDS)} seeds, equal in {int(np.sum(d == 0))} '
              f'(mean diff {d.mean():+.1f} pts)')
    print(f'=== Extended horizon {T_EXT:.0f} s, 3D strategies, {len(SEEDS)} seeds ===')
    marks = (1200, 1800, 2400, 3600)
    for i, seed in enumerate(SEEDS):
        a, b = res['near3d'][i], res['vp3d'][i]
        fa = a['floor_switch'][0][0] if a['floor_switch'] else float('nan')
        fb = b['floor_switch'][0][0] if b['floor_switch'] else float('nan')
        print(f'seed {seed}: near3d ' + '/'.join(f'{cov_at(a, tt):.1f}' for tt in marks)
              + ' %, vp3d ' + '/'.join(f'{cov_at(b, tt):.1f}' for tt in marks)
              + f' % (at 1200/1800/2400/3600 s); time to 50 %: near3d {t_reach(a, 50):.1f} s, '
              f'vp3d {t_reach(b, 50):.1f} s; first floor change near3d {fa:.0f} s, vp3d {fb:.0f} s')
    for tt in marks:
        ca = np.array([cov_at(r, tt) for r in res['near3d']]); cb = np.array([cov_at(r, tt) for r in res['vp3d']])
        d = ca - cb
        print(f'at {tt} s: near3d {ca.mean():.1f} +- {ca.std():.1f} % [{ca.min():.1f}, {ca.max():.1f}], vp3d '
              f'{cb.mean():.1f} +- {cb.std():.1f} % [{cb.min():.1f}, {cb.max():.1f}]; near3d higher in '
              f'{int(np.sum(d > 0))}/{len(SEEDS)}, vp3d higher in {int(np.sum(d < 0))}/{len(SEEDS)}, '
              f'difference {d.mean():+.1f} pts [{d.min():+.1f}, {d.max():+.1f}]')
    ta = np.array([t_reach(r, 50) for r in res['near3d']]); tb = np.array([t_reach(r, 50) for r in res['vp3d']])
    print(f'time to 50 %: near3d {np.nanmean(ta):.1f} s [{np.nanmin(ta):.1f}, {np.nanmax(ta):.1f}], vp3d '
          f'{np.nanmean(tb):.1f} s [{np.nanmin(tb):.1f}, {np.nanmax(tb):.1f}]; near3d earlier in '
          f'{int(np.sum(ta < tb))}/{len(SEEDS)}')
    print(f'=== Seed {SEEDS[0]} details ({T_EXT:.0f} s) ===')
    for s in STRATS:
        r = ext[s]
        cf = r['cov_f'][-1]
        print(f'[{s:6s}] coverage 600/1200/1800/2400/3600 s ' + '/'.join(f'{cov_at(r, tt):.1f}' for tt in (600, 1200, 1800, 2400, 3600))
              + f' %; final F0-F3 {cf[0]:.1f}/{cf[1]:.1f}/{cf[2]:.1f}/{cf[3]:.1f} %; time to 25/50/75 % '
              f'{t_reach(r, 25):.1f}/{t_reach(r, 50):.1f}/{t_reach(r, 75):.1f} s; path {r["path_len"]:.1f} m; '
              f'plans {r["n_plans"]}; frontiers exhausted {exh(r)}')
        print(f'         floor changes: ' + (', '.join(f'{tt:.0f} s F{a}->F{b}' for tt, a, b in r['floor_switch']) or 'none'))
    print(f'=== Vertical passage detection log, seed {SEEDS[0]} (first detection, span >= {STAIR_SPAN} m) ===')
    for s in ('near3d', 'vp3d'):
        for sl, c, a, span, tt in ext[s]['passages']:
            print(f'  [{s}] slab z = {sl * RES:.1f} m: centroid ({c[0]:.2f}, {c[1]:.2f}) m, free cross-section '
                  f'{a:.2f} m^2, known vertical span {span:.1f} m, detected at t = {tt:.1f} s')
    return dict(V=V, reach=reach_vox, res=res, ext=ext)


# ── Plots ───────────────────────────────────────────────────────────────────
MAP_CMAP = ListedColormap(['white', '#bbbbbb', 'black'])
FLOOR_LAYER = [2, 7, 12, 17]          # displayed layer per floor


def _state(p):
    return np.where(p < P_FREE, 0, np.where(p > P_OCC, 2, 1))


def plot_map_snapshots(data, out_dir):
    r = data['res']['vp3d'][0]
    times = sorted(r['snaps'])
    fig, axes = plt.subplots(4, len(times), figsize=(4 * len(times), 15))
    for ci, tt in enumerate(times):
        p, pos = r['snaps'][tt]
        for f in range(4):
            ax = axes[f, ci]
            k = FLOOR_LAYER[f]
            ax.imshow(_state(p[:, :, k]).T, origin='lower', cmap=MAP_CMAP, vmin=0, vmax=2,
                      extent=[0, NX * RES, 0, NY * RES])
            tr = r['traj'][:int(round(tt / DT)) + 1]
            on = FLOOR_OF[np.floor(tr[:, 2] / RES).astype(int)] == f
            ax.plot(np.where(on, tr[:, 0], np.nan), np.where(on, tr[:, 1], np.nan), color='tab:blue', lw=0.8)
            if FLOOR_OF[int(pos[2] / RES)] == f:
                ax.plot(pos[0], pos[1], 'o', color='red', ms=6)
            ax.set_title(f'F{f} (z = {k * RES + 0.25:.2f} m), t = {tt} s', fontsize=9)
            ax.set_xticks([]); ax.set_yticks([])
    fig.suptitle('Voxel map, vertical-passage planner, seed 0 (white free, grey unknown, black occupied; '
                 'blue = path on that floor, red = drone)', fontsize=12)
    plt.tight_layout()
    path = os.path.join(out_dir, 'map_snapshots.png')
    plt.savefig(path, dpi=90); plt.close()
    print(f'Saved: {path}')


def plot_coverage(data, out_dir):
    fig, axes = plt.subplots(1, 5, figsize=(24, 4.5))
    for s in STRATS:
        for i, r in enumerate(data['res'][s]):
            axes[0].plot(r['cov_t'], r['cov'], color=COLORS[s], lw=1.2 if i == 0 else 0.6,
                         alpha=1 if i == 0 else 0.5, label=LABELS[s] if i == 0 else None)
            for f in range(4):
                axes[f + 1].plot(r['cov_t'], r['cov_f'][:, f], color=COLORS[s], lw=1.2 if i == 0 else 0.6,
                                 alpha=1 if i == 0 else 0.5, label=LABELS[s] if i == 0 else None)
    for tt, a, b in data['res']['vp3d'][0]['floor_switch']:
        for ax in axes[1:]:
            ax.axvline(tt, color='tab:blue', ls=':', lw=0.7)
    axes[0].axhline(100, color='k', ls='--', lw=0.8)
    for ax in axes:
        ax.axvline(T_MAX, color='k', ls='--', lw=0.8)
    axes[0].set_title(f'Total 3D coverage (thick = seed 0 to {T_EXT:.0f} s; dashed = card horizon)', fontsize=9)
    for f in range(4):
        axes[f + 1].set_title(f'Floor F{f} (dotted = vp3d floor changes, seed 0)')
    for ax in axes:
        ax.set_xlabel('time (s)'); ax.set_ylabel('coverage (%)'); ax.set_ylim(0, 102); ax.grid(True, alpha=0.3)
    axes[0].legend(fontsize=8)
    plt.tight_layout()
    path = os.path.join(out_dir, 'coverage_curves.png')
    plt.savefig(path, dpi=110); plt.close()
    print(f'Saved: {path}')


def plot_trajectory_3d(data, out_dir):
    fig = plt.figure(figsize=(17, 6.5))
    ax = fig.add_subplot(1, 2, 1, projection='3d')
    p = data['res']['vp3d'][0]['final_p']
    occ = np.argwhere((p > P_OCC) & (np.arange(NZ)[None, None, :] % 5 != 0))[::3]
    ax.scatter(occ[:, 0] * RES, occ[:, 1] * RES, occ[:, 2] * RES, s=0.3, color='grey', alpha=0.3)
    for s in ('near3d', 'vp3d'):
        tr = data['res'][s][0]['traj']
        ax.plot(tr[:, 0], tr[:, 1], tr[:, 2], color=COLORS[s], lw=0.9, label=LABELS[s])
    ax.scatter(*START, color='green', s=60, label='start')
    for name, s, (x0, x1), (y0, y1) in OPENINGS:
        ax.plot([x0 * RES, x1 * RES, x1 * RES, x0 * RES, x0 * RES], [y0 * RES, y0 * RES, y1 * RES, y1 * RES, y0 * RES],
                [s * RES + 0.25] * 5, color='orange', lw=2)
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)'); ax.set_zlabel('z (m)'); ax.set_zlim(0, 10)
    ax.legend(fontsize=8); ax.set_title('Seed-0 paths (orange = stairwell openings, grey = mapped obstacles)')
    ax2 = fig.add_subplot(1, 2, 2)
    for s in STRATS:
        tr = data['res'][s][0]['traj']
        ax2.plot(np.arange(len(tr)) * DT, tr[:, 2], color=COLORS[s], lw=1, label=LABELS[s])
    for s_ in SLABS:
        ax2.axhspan(s_ * RES, (s_ + 1) * RES, color='grey', alpha=0.3)
    ax2.set_xlabel('time (s)'); ax2.set_ylabel('z (m)'); ax2.set_title('Altitude (grey bands = floor slabs)')
    ax2.legend(fontsize=8); ax2.grid(True, alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'trajectory_3d.png')
    plt.savefig(path, dpi=110); plt.close()
    print(f'Saved: {path}')


def save_animation(data, out_dir):
    r = data['res']['vp3d'][0]
    frames = r['frames']
    fig, axes = plt.subplots(1, 4, figsize=(16, 4.6))
    ims = []
    for f, ax in enumerate(axes):
        ims.append(ax.imshow(np.ones((NY, NX)), origin='lower', cmap=MAP_CMAP, vmin=0, vmax=2,
                             extent=[0, NX * RES, 0, NY * RES]))
        ax.set_xticks([]); ax.set_yticks([])
    dots = [ax.plot([], [], 'o', color='red', ms=6)[0] for ax in axes]
    fdots = [ax.plot([], [], 's', color='cyan', ms=1.5, ls='')[0] for ax in axes]
    ttl = fig.suptitle('')

    def update(i):
        t, p, pos = frames[i]
        fr = frontier_mask(p)
        for f in range(4):
            k = FLOOR_LAYER[f]
            ims[f].set_data(_state(p[:, :, k]).T)
            fi = np.argwhere(fr[:, :, 5 * f + 1:5 * f + 5].any(axis=2))
            fdots[f].set_data(fi[:, 0] * RES + 0.25, fi[:, 1] * RES + 0.25)
            on = FLOOR_OF[int(pos[2] / RES)] == f
            dots[f].set_data([pos[0]] if on else [], [pos[1]] if on else [])
            axes[f].set_title(f'F{f}' + ('  <- drone' if on else ''), fontsize=10)
        ttl.set_text(f'Vertical-passage planner, t = {t:.0f} s, drone z = {pos[2]:.2f} m (cyan = frontier)')
        return []

    ani = FuncAnimation(fig, update, frames=len(frames), interval=100, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer=PillowWriter(fps=8), dpi=60)
    plt.close()
    print(f'Saved: {path}  ({len(frames)} frames)')


if __name__ == '__main__':
    print('Running S043 3D Confined Space Exploration simulation...')
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    data = run_simulation()
    plot_map_snapshots(data, OUTPUT_DIR)
    plot_coverage(data, OUTPUT_DIR)
    plot_trajectory_3d(data, OUTPUT_DIR)
    save_animation(data, OUTPUT_DIR)
    print('Done.')
