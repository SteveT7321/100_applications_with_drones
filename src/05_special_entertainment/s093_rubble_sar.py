"""
S093 Earthquake Rubble Search and Rescue
========================================
A micro-drone (body radius 0.15 m, 0.8 m/s) explores a 20 x 20 x 5 m rubble field (40 %
of the volume filled with concrete chunks) to find three buried victims by sound.
It builds a log-odds voxel map from a multi-ring LiDAR, inflates obstacles (and unknown
space) by r_inf = 0.25 m, picks frontier clusters with the card's score
distance / sqrt(cluster size) using geodesic distance, and flies shortest 26-connected
voxel paths. A victim is confirmed after K_conf = 3 positive acoustic detections within a
5 s window, where each 10 Hz observation is Bernoulli(exp(-d^2 / 2 sigma_a^2)).

The expected number of victims found is computed two ways along the flown trajectory:
the card's analytic formula (any single positive detection) and a Monte Carlo of the
actual K-of-window confirmation rule. A Monte Carlo over rubble fields gives mean +- std.

Usage:
    conda activate drones
    python src/05_special_entertainment/s093_rubble_sar.py
"""

import os
import math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401
from scipy.ndimage import distance_transform_edt, label, binary_dilation
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra

# ── Parameters ────────────────────────────────────────────────────────────────
ENV = np.array([20.0, 20.0, 5.0])          # m
VOX = 0.2                                   # m voxel side (card: 0.1)
SHAPE = tuple(int(round(e / VOX)) for e in ENV)   # (100, 100, 25)
FILL_RATE = 0.40
CHUNK_MIN, CHUNK_MAX = 5, 15                # voxels: rubble chunk sides 1.0-3.0 m
N_VICTIMS = 3
VICTIM_MIN_DIST_ENTRANCE = 6.0              # m
VICTIM_MIN_SEP = 4.0                        # m
VICTIM_MAX_FROM_PASSAGE = 1.0               # m from reachable flyable space

R_DRONE = 0.15                              # m body radius
R_INF = 0.25                                # m inflation radius
INFL = R_INF + VOX / 2                      # m centre-to-centre blocking distance (0.35)
INFL_UNK = 0.30                             # m blocking distance around unknown voxels
INFL_RECOVER = R_DRONE + VOX / 2            # m obstacle inflation of the recovery plan (0.25)
V_MAX = 0.8                                 # m/s
DT = 0.1                                    # s
T_MAX = 300.0                               # s
N_STEPS = int(round(T_MAX / DT))

N_BEAMS = 2400                              # spherical (Fibonacci) beam pattern, card: 120
SCAN_EVERY = 1                              # scan at 10 Hz
PATH_CHECK_EVERY = 10                       # re-check the current path every 1 s
R_LIDAR = 3.0                               # m
RAY_STEP = 0.1                              # m sampling along a beam
SIGMA_R = 0.03                              # m range noise

L_OCC = math.log(0.85 / 0.15)
L_FREE = math.log(0.40 / 0.60)
L_MIN, L_MAX = -10.0, 10.0
L_OCC_TH = math.log(0.7 / 0.3)              # P > 0.7
L_FREE_TH = math.log(0.3 / 0.7)             # P < 0.3

R_ACOUSTIC = 2.0                            # m
SIGMA_A = 0.8                               # m
K_CONF = 3
T_WIN = 5.0                                 # s
N_WIN = int(round(T_WIN / DT))              # 50 samples

VIEW_R = 1.0                                # m viewpoint radius around a frontier voxel
MIN_CLUSTER = 10                            # frontier clusters smaller than this are ignored
REPLAN_T = 5.0                              # s periodic replanning
DWELL_STEPS = 2                             # 0.2 s (2 more scans) hover at a reached viewpoint
N_EXPECT_MC = 2000                          # replicas for the K-of-window expectation

ENTRANCE = (slice(0, 7), slice(46, 54), slice(0, 7))   # x 0-1.4, y 9.2-10.8, z 0-1.4 m
START = np.array([0.7, 10.0, 0.7])

SEED_MAIN = 42
MC_SEEDS = [42, 43, 44]

OUTPUT_DIR = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', '..',
    'outputs', '05_special_entertainment', 's093_rubble_sar'))


def centres(idx):
    return (np.asarray(idx, dtype=float) + 0.5) * VOX


def to_idx(p):
    return tuple(int(np.clip(math.floor(p[k] / VOX), 0, SHAPE[k] - 1)) for k in range(3))


# ── Environment ───────────────────────────────────────────────────────────────

def blocked_mask(obstacle, infl=INFL):
    """Voxels whose centre is within infl of an obstacle voxel centre or of the outside
    of the field (walls, floor and ceiling count as obstacles)."""
    pad = np.pad(obstacle, 1, constant_values=True)
    d = distance_transform_edt(~pad, sampling=VOX)[1:-1, 1:-1, 1:-1]
    return d <= infl + 1e-9


def blocked_estimate(occ_e, unknown, infl=INFL):
    """Planner's no-fly set: within r_inf + vox/2 of a mapped obstacle, or within INFL_UNK
    of an unknown voxel (a voxel-centre drone then stays >= 0.17 m from any unknown box)."""
    d_unk = distance_transform_edt(~unknown, sampling=VOX)
    return blocked_mask(occ_e, infl) | (d_unk <= INFL_UNK + 1e-9)


def generate_rubble(seed):
    """Random box-shaped chunks until 40 % of the volume is filled (entrance kept clear);
    victims placed next to rubble, within 1 m of flyable space reachable from the entrance.
    Returns occ (bool grid), victims (3, 3), reach (bool grid), attempts."""
    rng = np.random.default_rng(seed)
    for attempt in range(1, 50):
        occ = np.zeros(SHAPE, dtype=bool)
        while True:
            size = rng.integers(CHUNK_MIN, CHUNK_MAX + 1, 3)
            lo = [int(rng.integers(0, SHAPE[k] - size[k] + 1)) for k in range(3)]
            occ[lo[0]:lo[0] + size[0], lo[1]:lo[1] + size[1], lo[2]:lo[2] + size[2]] = True
            occ[ENTRANCE] = False
            if occ.mean() >= FILL_RATE:
                break
        flyable = ~blocked_mask(occ)
        lab, _ = label(flyable, structure=np.ones((3, 3, 3)))
        s_lab = lab[to_idx(START)]
        if s_lab == 0:
            continue
        reach = lab == s_lab
        d_reach = distance_transform_edt(~reach, sampling=VOX)
        adj_rubble = binary_dilation(occ, structure=np.array(
            [[[0, 0, 0], [0, 1, 0], [0, 0, 0]], [[0, 1, 0], [1, 1, 1], [0, 1, 0]],
             [[0, 0, 0], [0, 1, 0], [0, 0, 0]]], dtype=bool)) & ~occ
        cand = np.argwhere(adj_rubble & (d_reach <= VICTIM_MAX_FROM_PASSAGE))
        cpos = centres(cand)
        cand = cpos[np.linalg.norm(cpos - START, axis=1) >= VICTIM_MIN_DIST_ENTRANCE]
        order = rng.permutation(len(cand))
        vic = []
        for i in order:
            if all(np.linalg.norm(cand[i] - v) >= VICTIM_MIN_SEP for v in vic):
                vic.append(cand[i])
            if len(vic) == N_VICTIMS:
                return occ, np.array(vic), reach, attempt
    raise RuntimeError('no valid rubble field')


# ── Sensors ───────────────────────────────────────────────────────────────────

def lidar_dirs(n=N_BEAMS):
    """Fibonacci sphere: n nearly uniform directions (mean spacing ~sqrt(4 pi / n) rad)."""
    k = np.arange(n) + 0.5
    z = 1.0 - 2.0 * k / n
    phi = np.pi * (1.0 + math.sqrt(5.0)) * k
    rxy = np.sqrt(1.0 - z ** 2)
    return np.column_stack([rxy * np.cos(phi), rxy * np.sin(phi), z])


DIRS = lidar_dirs()
R_SAMPLES = np.arange(1, int(round(R_LIDAR / RAY_STEP)) + 1) * RAY_STEP


def lidar_scan(pos, occ, log_odds, rng):
    """Cast all beams, update log-odds in place (one update per voxel per scan)."""
    pts = pos[None, None, :] + R_SAMPLES[None, :, None] * DIRS[:, None, :]      # (B, S, 3)
    idx = np.floor(pts / VOX).astype(int)
    inside = np.all((idx >= 0) & (idx < np.array(SHAPE)), axis=2)
    idc = np.clip(idx, 0, np.array(SHAPE) - 1)
    hit = occ[idc[..., 0], idc[..., 1], idc[..., 2]] & inside
    stop = hit | ~inside
    any_stop = stop.any(axis=1)
    first = np.where(any_stop, stop.argmax(axis=1), len(R_SAMPLES))
    k = np.arange(len(R_SAMPLES))[None, :]
    free_s = (k < first[:, None]) & inside
    fidx = idc[free_s]
    flat_free = np.unique(np.ravel_multi_index(fidx.T, SHAPE))
    hb = np.nonzero(any_stop & hit[np.arange(len(DIRS)), np.minimum(first, len(R_SAMPLES) - 1)])[0]
    r_hit = R_SAMPLES[first[hb]] + rng.normal(0.0, SIGMA_R, len(hb))
    r_hit = np.clip(r_hit, RAY_STEP, R_LIDAR)
    hp = pos[None, :] + r_hit[:, None] * DIRS[hb]
    hidx = np.floor(hp / VOX).astype(int)
    ok = np.all((hidx >= 0) & (hidx < np.array(SHAPE)), axis=1)
    flat_hit = np.unique(np.ravel_multi_index(hidx[ok].T, SHAPE))
    flat_free = np.setdiff1d(flat_free, flat_hit, assume_unique=True)
    lo = log_odds.reshape(-1)
    lo[flat_free] = np.clip(lo[flat_free] + L_FREE, L_MIN, L_MAX)
    lo[flat_hit] = np.clip(lo[flat_hit] + L_OCC, L_MIN, L_MAX)


def p_detect(d):
    return np.exp(-d ** 2 / (2 * SIGMA_A ** 2))


# ── Planning ──────────────────────────────────────────────────────────────────

_OFFS = [(dx, dy, dz) for dx in (-1, 0, 1) for dy in (-1, 0, 1) for dz in (-1, 0, 1)
         if (dx, dy, dz) > (0, 0, 0)]


def build_graph(trav):
    """Undirected 26-connected graph over traversable voxels (edge cost = centre distance)."""
    ids = -np.ones(SHAPE, dtype=np.int64)
    nodes = np.argwhere(trav)
    ids[tuple(nodes.T)] = np.arange(len(nodes))
    rows, cols, w = [], [], []
    for d in _OFFS:
        sl_a = tuple(slice(max(0, -d[k]), SHAPE[k] - max(0, d[k])) for k in range(3))
        sl_b = tuple(slice(max(0, d[k]), SHAPE[k] - max(0, -d[k])) for k in range(3))
        both = trav[sl_a] & trav[sl_b]
        a = ids[sl_a][both]
        b = ids[sl_b][both]
        rows.append(a); cols.append(b)
        w.append(np.full(len(a), VOX * math.sqrt(d[0] ** 2 + d[1] ** 2 + d[2] ** 2)))
    rows = np.concatenate(rows); cols = np.concatenate(cols); w = np.concatenate(w)
    g = csr_matrix((w, (rows, cols)), shape=(len(nodes), len(nodes)))
    return g, ids, nodes


def plan(log_odds, pos, banned, infl=INFL):
    """Returns (path as (n, 3) waypoint positions, goal viewpoint index, n_frontier_voxels,
    n_clusters) or (None, ...) if no reachable frontier."""
    occ_e = log_odds > L_OCC_TH
    free_e = log_odds < L_FREE_TH
    unknown = ~occ_e & ~free_e
    trav = ~blocked_estimate(occ_e, unknown, infl)
    start = to_idx(pos)
    trav[start] = True                       # the drone's own voxel
    frontier = unknown & binary_dilation(free_e, structure=np.ones((3, 3, 3))) & ~banned
    n_front = int(frontier.sum())
    if n_front == 0:
        return None, None, 0, 0
    lab, n_cl = label(frontier, structure=np.ones((3, 3, 3)))
    sizes = np.bincount(lab.ravel())
    small = sizes < MIN_CLUSTER
    small[0] = False
    frontier &= ~small[lab]
    lab[~frontier] = 0
    if not frontier.any():
        return None, None, n_front, n_cl
    g, ids, nodes = build_graph(trav)
    dist, pred = dijkstra(g, directed=False, indices=ids[start], return_predecessors=True)
    d_f, ind = distance_transform_edt(~frontier, sampling=VOX, return_indices=True)
    cand = trav & (d_f <= VIEW_R)
    cidx = np.argwhere(cand)
    if len(cidx) == 0:
        return None, None, n_front, n_cl
    nid = ids[tuple(cidx.T)]
    geo = dist[nid]
    near = ind[:, cidx[:, 0], cidx[:, 1], cidx[:, 2]]
    clus = lab[near[0], near[1], near[2]]
    ok = np.isfinite(geo) & (geo > 0)
    if not ok.any():
        return None, None, n_front, n_cl
    score = geo[ok] / np.sqrt(sizes[clus[ok]])
    best = int(np.argmin(score))
    goal_node = nid[ok][best]
    path = [goal_node]
    while path[-1] != ids[start]:
        path.append(pred[path[-1]])
    path = nodes[np.array(path[::-1])]
    return centres(path), tuple(nodes[goal_node]), n_front, n_cl


# ── Simulation ────────────────────────────────────────────────────────────────

def clearance_to_rubble(traj, occ):
    """Exact distance from each trajectory point to the nearest obstacle voxel box
    (field boundary included as walls/floor/ceiling)."""
    out = np.full(len(traj), np.inf)
    rad = 3
    for t, p in enumerate(traj):
        i0 = np.array(to_idx(p))
        lo = np.maximum(i0 - rad, 0)
        hi = np.minimum(i0 + rad + 1, np.array(SHAPE))
        sub = np.argwhere(occ[lo[0]:hi[0], lo[1]:hi[1], lo[2]:hi[2]]) + lo
        if len(sub):
            bmin = sub * VOX
            q = np.clip(p, bmin, bmin + VOX)
            out[t] = np.linalg.norm(q - p, axis=1).min()
        out[t] = min(out[t], p.min(), *(ENV - p))
    return out


def run_mission(seed, keep_frames=False, frame_every=20):
    occ, victims, reach, attempts = generate_rubble(seed)
    rng_lidar = np.random.default_rng([seed, 1])
    rng_ac = np.random.default_rng([seed, 2])
    log_odds = np.zeros(SHAPE)
    log_odds[ENTRANCE] = L_MIN               # the entrance pocket is seen from outside
    banned = np.zeros(SHAPE, dtype=bool)
    total_free = int((~occ).sum())
    reach_free = int(reach.sum())
    pos = START.copy()
    traj = [pos.copy()]
    det = np.zeros((N_STEPS + 1, N_VICTIMS), dtype=bool)
    found_t = np.full(N_VICTIMS, np.nan)
    cov_t, cov_xi, cov_reach = [], [], []
    frames = []
    path, goal, next_replan = None, None, 0.0
    dwell_until = None
    n_replans, n_banned_events, n_recover = 0, 0, 0
    end_reason = 'timeout'
    k_end = N_STEPS
    for k in range(N_STEPS + 1):
        t = k * DT
        if k % SCAN_EVERY == 0:
            lidar_scan(pos, occ, log_odds, rng_lidar)
        d_v = np.linalg.norm(victims - pos, axis=1)
        det[k] = rng_ac.random(N_VICTIMS) < p_detect(d_v)
        lo_k = max(0, k - N_WIN + 1)
        cnt = det[lo_k:k + 1].sum(axis=0)
        newly = np.isnan(found_t) & (cnt >= K_CONF)
        found_t[newly] = t
        if k % 10 == 0:
            free_e = log_odds < L_FREE_TH
            cov_t.append(t)
            cov_xi.append(100.0 * free_e.sum() / total_free)
            cov_reach.append(100.0 * (free_e & reach).sum() / reach_free)
        if keep_frames and k % frame_every == 0:
            frames.append((k, (log_odds < L_FREE_TH).any(axis=2), found_t.copy()))
        if not np.isnan(found_t).any():
            end_reason, k_end = 'all found', k
            break
        if k == N_STEPS:
            break
        arrived = path is not None and len(path) == 0 and goal is not None
        if arrived and dwell_until is None:
            dwell_until = k + DWELL_STEPS
        if arrived and k < dwell_until:
            traj.append(pos.copy())
            continue
        need = path is None or len(path) == 0 or t >= next_replan
        if path is not None and len(path) and k % PATH_CHECK_EVERY == 0:
            occ_e = log_odds > L_OCC_TH
            unk = ~occ_e & ~(log_odds < L_FREE_TH)
            pidx = np.floor(path / VOX).astype(int)
            blk = blocked_estimate(occ_e, unk)
            if blk[pidx[:, 0], pidx[:, 1], pidx[:, 2]][1:].any():
                need = True
        if need:
            dwell_until = None
            if arrived:
                # reached the viewpoint: ban frontier voxels near it that are still unknown
                gp = centres(goal)
                lo3 = np.maximum(np.array(goal) - 5, 0)
                hi3 = np.minimum(np.array(goal) + 6, np.array(SHAPE))
                sub = np.argwhere(np.ones(tuple(hi3 - lo3), dtype=bool)) + lo3
                sub = sub[np.linalg.norm(centres(sub) - gp, axis=1) <= VIEW_R + 1e-9]
                unk_now = np.abs(log_odds[tuple(sub.T)]) <= L_OCC_TH
                banned[tuple(sub[unk_now].T)] = True
                n_banned_events += 1
            path, goal, nf, ncl = plan(log_odds, pos, banned)
            n_replans += 1
            next_replan = t + REPLAN_T
            if path is None:
                # recovery: obstacle inflation reduced to body radius + half a voxel
                path, goal, nf, ncl = plan(log_odds, pos, banned, INFL_RECOVER)
                n_recover += 1
                next_replan = t + DT                 # return to normal planning next step
            if path is None:
                end_reason, k_end = 'no reachable frontier', k
                break
            path = path[1:]
        # move along the path
        step = V_MAX * DT
        while step > 1e-12 and len(path):
            d = path[0] - pos
            dn = np.linalg.norm(d)
            if dn <= step:
                pos = path[0].copy()
                path = path[1:]
                step -= dn
            else:
                pos = pos + d / dn * step
                step = 0.0
        traj.append(pos.copy())
    traj = np.array(traj)
    det = det[:len(traj)]
    clear = clearance_to_rubble(traj, occ)
    speeds = np.linalg.norm(np.diff(traj, axis=0), axis=1) / DT
    assert clear.min() >= R_DRONE, f'drone body hits rubble (clearance {clear.min():.3f} m)'
    assert speeds.max() <= V_MAX + 1e-9, 'speed limit violated'
    free_e = log_odds < L_FREE_TH
    occ_e = log_odds > L_OCC_TH
    res = {
        'seed': seed, 'attempts': attempts, 'occ': occ, 'victims': victims, 'traj': traj,
        'found_t': found_t, 'n_found': int(np.sum(~np.isnan(found_t))),
        't_mission': float(k_end * DT), 'end_reason': end_reason,
        'cov_t': np.array(cov_t), 'cov_xi': np.array(cov_xi), 'cov_reach': np.array(cov_reach),
        'xi': cov_xi[-1], 'xi_reach': cov_reach[-1],
        'fill': float(occ.mean()), 'reach_frac': reach_free / total_free,
        'path_len': float(np.linalg.norm(np.diff(traj, axis=0), axis=1).sum()),
        'min_clear': float(clear.min()), 'max_speed': float(speeds.max()),
        'n_replans': n_replans, 'n_banned_events': n_banned_events, 'n_recover': n_recover,
        'false_free': int((free_e & occ).sum()), 'false_occ': int((occ_e & ~occ).sum()),
        'n_occ_est': int(occ_e.sum()), 'det': det,
        'min_dist_victim': np.linalg.norm(traj[:, None] - victims[None], axis=2).min(axis=0),
        'frames': frames, 'log_odds': log_odds, 'banned': banned,
    }
    res.update(expected_curves(traj, victims, seed))
    return res


def expected_curves(traj, victims, seed):
    """E[N_found](t) along the flown trajectory: (a) card formula, any single detection
    while within r_acoustic; (b) Monte Carlo of the K-of-window confirmation rule."""
    d = np.linalg.norm(traj[:, None] - victims[None], axis=2)          # (T, V)
    p = p_detect(d)
    p_card = np.where(d <= R_ACOUSTIC, p, 0.0)
    e_card = np.sum(1.0 - np.exp(np.cumsum(np.log1p(-np.minimum(p_card, 1 - 1e-12)), axis=0)),
                    axis=1)
    rng = np.random.default_rng([seed, 3])
    T = len(traj)
    found_by = np.zeros((T, len(victims)))
    for v in range(len(victims)):
        z = rng.random((N_EXPECT_MC, T)) < p[None, :, v]
        c = np.cumsum(z, axis=1)
        win = c - np.concatenate([np.zeros((N_EXPECT_MC, N_WIN)), c[:, :-N_WIN]], axis=1)
        ok = win >= K_CONF
        first = np.where(ok.any(axis=1), ok.argmax(axis=1), T)
        found_by[:, v] = np.mean(first[None, :] <= np.arange(T)[:, None], axis=1)
    return {'e_card': e_card, 'e_rule': found_by.sum(axis=1), 'p_rule_final': found_by[-1]}


def run_simulation():
    main = run_mission(SEED_MAIN, keep_frames=True)
    mc = [main] + [run_mission(s) for s in MC_SEEDS if s != SEED_MAIN]
    return main, mc


# ── Plotting ──────────────────────────────────────────────────────────────────

def plot_rubble_3d(r, out_dir):
    fig = plt.figure(figsize=(11, 8))
    ax = fig.add_subplot(111, projection='3d')
    occ = r['occ']
    # draw only the rubble surface voxels that border free space, subsampled
    surf = occ & binary_dilation(~occ, structure=np.ones((3, 3, 3)))
    pts = centres(np.argwhere(surf))
    sel = np.random.default_rng(0).permutation(len(pts))[:6000]
    ax.scatter(*pts[sel].T, s=1, c='grey', alpha=0.12)
    tr = r['traj']
    ax.plot(*tr.T, color='red', lw=1.0, label='drone path')
    ax.scatter(*tr[0], color='red', marker='^', s=80, label='start')
    for i, v in enumerate(r['victims']):
        fnd = not np.isnan(r['found_t'][i])
        ax.scatter(*v, s=160, marker='*', color='green' if fnd else 'cyan', edgecolor='k',
                   label=f'victim {i + 1} ' + (f"found at {r['found_t'][i]:.1f} s" if fnd
                                               else 'not found'))
    ax.set_xlim(0, ENV[0]); ax.set_ylim(0, ENV[1]); ax.set_zlim(0, ENV[2])
    ax.set_box_aspect((4, 4, 1.4))
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)'); ax.set_zlabel('z (m)')
    ax.legend(fontsize=8, loc='upper left')
    ax.set_title(f"S093 Rubble field (seed {r['seed']}, fill {100 * r['fill']:.1f} %) and "
                 f"exploration path, {r['n_found']}/{N_VICTIMS} found in {r['t_mission']:.1f} s")
    path = os.path.join(out_dir, 'rubble_3d.png')
    plt.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def _top_view(ax, r, known=None):
    col = r['occ'].mean(axis=2).T
    ax.imshow(col, origin='lower', extent=[0, ENV[0], 0, ENV[1]], cmap='Greys', vmin=0,
              vmax=1, alpha=0.8)
    if known is not None:
        ax.imshow(np.ma.masked_where(~known.T, known.T), origin='lower',
                  extent=[0, ENV[0], 0, ENV[1]], cmap='Blues', vmin=0, vmax=1.6, alpha=0.45)


def plot_top_down(r, out_dir):
    fig, ax = plt.subplots(figsize=(8.5, 7.5))
    _top_view(ax, r)
    tr = r['traj']
    sc = ax.scatter(tr[:, 0], tr[:, 1], c=tr[:, 2], cmap='autumn', s=2, vmin=0, vmax=ENV[2])
    fig.colorbar(sc, ax=ax, label='drone altitude z (m)')
    ax.plot(*START[:2], '^', color='red', ms=10, label='start')
    th = np.linspace(0, 2 * np.pi, 80)
    for i, v in enumerate(r['victims']):
        fnd = not np.isnan(r['found_t'][i])
        ax.plot(v[0], v[1], '*', ms=16, mec='k', color='green' if fnd else 'cyan',
                label=f'victim {i + 1} (z = {v[2]:.1f} m)')
        ax.plot(v[0] + R_ACOUSTIC * np.cos(th), v[1] + R_ACOUSTIC * np.sin(th), ':',
                color='green', lw=1)
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')
    ax.set_title('S093 Top view: grey = fraction of the column filled with rubble,\n'
                 'path coloured by altitude, dotted = 2 m acoustic range')
    ax.legend(fontsize=8, loc='upper right')
    path = os.path.join(out_dir, 'top_down.png')
    plt.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_coverage(mc, out_dir):
    fig, ax = plt.subplots(figsize=(10, 5))
    for r in mc:
        lw = 2.2 if r['seed'] == SEED_MAIN else 1.0
        ax.plot(r['cov_t'], r['cov_xi'], color='tab:blue', lw=lw, alpha=0.9 if lw > 1 else 0.5)
        ax.plot(r['cov_t'], r['cov_reach'], color='tab:green', lw=lw,
                alpha=0.9 if lw > 1 else 0.5)
        for tf in r['found_t'][~np.isnan(r['found_t'])]:
            ax.plot(tf, np.interp(tf, r['cov_t'], r['cov_xi']), 'k*', ms=7)
    ax.plot([], [], color='tab:blue', label=r'$\xi$: known free / all free voxels (card)')
    ax.plot([], [], color='tab:green', label=r'$\xi_{reach}$: known free / free voxels '
            'connected to flyable space')
    ax.plot([], [], 'k*', label='victim confirmed')
    ax.axhline(75, color='orange', ls='--', lw=1, label='card range 75–90 %')
    ax.axhline(90, color='orange', ls='--', lw=1)
    ax.set_xlabel('time (s)')
    ax.set_ylabel('explored volume (%)')
    ax.set_title(f'S093 Explored volume vs time (thick = seed {SEED_MAIN}, thin = other seeds)')
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    path = os.path.join(out_dir, 'coverage.png')
    plt.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_detection(r, out_dir):
    fig, axes = plt.subplots(1, 2, figsize=(15, 5))
    ax = axes[0]
    t = np.arange(len(r['traj'])) * DT
    n_found = np.array([np.sum(r['found_t'] <= tt) for tt in t])
    ax.step(t, n_found, where='post', color='darkgreen', lw=2, label='actual $N_{found}$')
    ax.plot(t, r['e_card'], 'b--', lw=1.4,
            label=r'card formula $E[N]$ (any detection within $r_{acoustic}$)')
    ax.plot(t, r['e_rule'], color='tab:orange', lw=1.4,
            label=f'Monte Carlo $E[N]$ of the {K_CONF}-in-{T_WIN:.0f} s rule '
                  f'({N_EXPECT_MC} replicas)')
    ax.axhline(N_VICTIMS, color='grey', ls=':')
    ax.set_xlabel('time (s)'); ax.set_ylabel('victims')
    ax.set_yticks(range(N_VICTIMS + 1))
    ax.set_title(f"Victim detection along the seed-{r['seed']} trajectory")
    ax.legend(fontsize=8, loc='upper left')
    ax.grid(alpha=0.3)
    ax = axes[1]
    d = np.linspace(0, 4, 300)
    ax.plot(d, p_detect(d), 'b-', lw=1.8, label=r'$P_{detect}(d) = e^{-d^2/2\sigma_a^2}$')
    ax.axvline(R_ACOUSTIC, color='red', ls='--', label=f'$r_{{acoustic}}$ = {R_ACOUSTIC} m')
    ax.axhline(p_detect(R_ACOUSTIC), color='orange', ls=':',
               label=f'P at 2 m = {p_detect(R_ACOUSTIC):.3f}')
    # probability of confirming within one 5 s window while hovering at distance d
    pk = p_detect(d)
    from scipy.stats import binom
    ax.plot(d, binom.sf(K_CONF - 1, N_WIN, pk), color='tab:orange', lw=1.8,
            label=f'P(≥ {K_CONF} positives in {N_WIN} samples) while hovering')
    ax.set_xlabel('distance to victim d (m)'); ax.set_ylabel('probability')
    ax.set_title('Acoustic detection model')
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'detection.png')
    plt.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_monte_carlo(mc, out_dir):
    fig, axes = plt.subplots(1, 4, figsize=(18, 4.5))
    seeds = [r['seed'] for r in mc]
    vals = [('n_found', 'Victims found', None),
            ('t_mission', 'Mission time (s)', T_MAX),
            ('xi', r'Coverage $\xi$ (%)', None),
            ('path_len', 'Path length (m)', None)]
    for ax, (key, title, ref) in zip(axes, vals):
        v = [r[key] for r in mc]
        ax.bar(range(len(mc)), v, color='tab:blue', edgecolor='k', alpha=0.8)
        if ref:
            ax.axhline(ref, color='red', ls='--', lw=1, label='T_max')
            ax.legend(fontsize=8)
        ax.set_xticks(range(len(mc)))
        ax.set_xticklabels([str(s) for s in seeds])
        ax.set_xlabel('rubble seed')
        ax.set_title(f'{title}\nmean {np.mean(v):.1f} ± {np.std(v):.1f}', fontsize=10)
        ax.grid(alpha=0.3, axis='y')
    fig.suptitle(f'S093 Monte Carlo over {len(mc)} rubble fields')
    plt.tight_layout()
    path = os.path.join(out_dir, 'monte_carlo.png')
    plt.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def save_animation(r, out_dir, step=20):
    tr = r['traj']
    n = len(tr)
    n_frames = math.ceil((n - 1) / step) + 1
    frames = {k: (known, ft) for k, known, ft in r['frames']}
    fig, ax = plt.subplots(figsize=(7, 6.5))
    _top_view(ax, r)
    known_im = ax.imshow(np.zeros((SHAPE[1], SHAPE[0])), origin='lower',
                         extent=[0, ENV[0], 0, ENV[1]], cmap='Blues', vmin=0, vmax=1.6,
                         alpha=0.45)
    line, = ax.plot([], [], 'r-', lw=0.9)
    dot, = ax.plot([], [], 'ro', ms=7)
    vic = [ax.plot(v[0], v[1], '*', ms=15, mec='k', color='cyan')[0] for v in r['victims']]
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')
    title = ax.set_title('')
    keys = sorted(frames)

    def update(f):
        k = min(f * step, n - 1)
        kk = max(key for key in keys if key <= k)
        known, ft = frames[kk]
        known_im.set_data(np.ma.masked_where(~known.T, known.T.astype(float)))
        line.set_data(tr[:k + 1, 0], tr[:k + 1, 1])
        dot.set_data([tr[k, 0]], [tr[k, 1]])
        t = k * DT
        for i, h in enumerate(vic):
            h.set_color('green' if r['found_t'][i] <= t else 'cyan')
        nf = int(np.sum(r['found_t'] <= t))
        title.set_text(f'S093 t = {t:5.1f} s  victims {nf}/{N_VICTIMS}\n'
                       'blue = columns with mapped free space, grey = rubble')
        return [known_im, line, dot, title] + vic

    anim = animation.FuncAnimation(fig, update, frames=n_frames, interval=66, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    anim.save(path, writer='pillow', fps=15, dpi=72)
    plt.close(fig)
    print(f'Saved: {path} ({n_frames} frames)')


# ── Main ──────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    main, mc = run_simulation()
    r = main
    print('\n=== S093 Earthquake Rubble SAR — Results ===')
    print(f'  Grid {SHAPE} at {VOX} m; LiDAR {len(DIRS)} beams per scan, mean spacing '
          f'{math.degrees(math.sqrt(4 * math.pi / N_BEAMS)):.1f} deg '
          f'({R_LIDAR * math.sqrt(4 * math.pi / N_BEAMS):.2f} m at {R_LIDAR} m range)')
    print(f'  Card i.i.d. fill check: P(a 0.25 m-radius sphere of 0.1 m voxels is all free) '
          f'= 0.6^{int(np.sum(np.linalg.norm(np.argwhere(np.ones((7, 7, 7))) - 3, axis=1) <= 2.5))}'
          f' = {0.6 ** int(np.sum(np.linalg.norm(np.argwhere(np.ones((7, 7, 7))) - 3, axis=1) <= 2.5)):.1e}')
    print(f'  Inflation: a 0.4 m gap minus 2 x r_inf = {0.4 - 2 * R_INF:.2f} m free width')
    print(f'  P_detect at r_acoustic = {p_detect(R_ACOUSTIC):.4f}; P_detect >= 0.9 needs '
          f'd <= {SIGMA_A * math.sqrt(2 * math.log(1 / 0.9)):.3f} m')
    print(f"\n  Main seed {r['seed']} (field generated in {r['attempts']} attempt(s)):")
    print(f"    Fill rate {100 * r['fill']:.1f} %; free voxels connected to the reachable "
          f"flyable space: {100 * r['reach_frac']:.1f} % of all free voxels")
    for i, v in enumerate(r['victims']):
        print(f'    Victim {i + 1} at ({v[0]:.1f}, {v[1]:.1f}, {v[2]:.1f}) m: '
              f"closest approach {r['min_dist_victim'][i]:.2f} m, "
              + (f"confirmed at {r['found_t'][i]:.1f} s" if not np.isnan(r['found_t'][i])
                 else 'not confirmed')
              + f", P(confirmed by end | path) = {r['p_rule_final'][i]:.3f}")
    print(f"    Victims found     : {r['n_found']}/{N_VICTIMS} ({r['end_reason']})")
    print(f"    Mission time      : {r['t_mission']:.1f} s")
    print(f"    Coverage xi       : {r['xi']:.1f} % (reachable-component xi {r['xi_reach']:.1f} %)")
    print(f"    Path length       : {r['path_len']:.1f} m, replans {r['n_replans']}, "
          f"viewpoints exhausted {r['n_banned_events']}, recovery plans {r['n_recover']}")
    print(f"    Min clearance     : {r['min_clear']:.3f} m to rubble (body radius {R_DRONE} m); "
          f"max speed {r['max_speed']:.2f} m/s")
    print(f"    Map errors        : {r['false_free']} occupied voxels mapped free, "
          f"{r['false_occ']} free voxels mapped occupied (of {r['n_occ_est']} mapped occupied)")
    print(f"    E[N_found] at end : card formula {r['e_card'][-1]:.3f}, "
          f"K-of-window Monte Carlo {r['e_rule'][-1]:.3f}, actual {r['n_found']}")
    print(f'\n  Monte Carlo over {len(mc)} rubble fields (seeds {MC_SEEDS[0]}–{MC_SEEDS[-1]}):')
    for x in mc:
        print(f"    seed {x['seed']}: found {x['n_found']}/{N_VICTIMS}, T {x['t_mission']:6.1f} s "
              f"({x['end_reason']}), xi {x['xi']:5.1f} %, xi_reach {x['xi_reach']:5.1f} %, "
              f"path {x['path_len']:6.1f} m, min clearance {x['min_clear']:.3f} m, "
              f"E_card {x['e_card'][-1]:.2f}, E_rule {x['e_rule'][-1]:.2f}")
    nf = [x['n_found'] for x in mc]
    print(f'    Victims found  : {np.mean(nf):.2f} ± {np.std(nf):.2f} '
          f'(all three in {sum(v == N_VICTIMS for v in nf)}/{len(mc)} fields)')
    print(f"    Mission time   : {np.mean([x['t_mission'] for x in mc]):.1f} ± "
          f"{np.std([x['t_mission'] for x in mc]):.1f} s")
    print(f"    Coverage xi    : {np.mean([x['xi'] for x in mc]):.1f} ± "
          f"{np.std([x['xi'] for x in mc]):.1f} %")
    print(f"    E[N] at end    : card {np.mean([x['e_card'][-1] for x in mc]):.2f}, "
          f"rule {np.mean([x['e_rule'][-1] for x in mc]):.2f}, actual {np.mean(nf):.2f}")

    os.makedirs(OUTPUT_DIR, exist_ok=True)
    plot_rubble_3d(main, OUTPUT_DIR)
    plot_top_down(main, OUTPUT_DIR)
    plot_coverage(mc, OUTPUT_DIR)
    plot_detection(main, OUTPUT_DIR)
    plot_monte_carlo(mc, OUTPUT_DIR)
    save_animation(main, OUTPUT_DIR)
