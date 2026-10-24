"""
S074 Mine 3D Mapping
====================
A drone explores a GPS-denied mine (main corridor + three branches, 3 x 3 m section) with
a horizontal 360-degree LiDAR (180 beams, 10 m). It estimates its pose by scan-to-scan ICP
seeded with an odometry prior, builds a log-odds occupancy grid of the scan plane and
explores autonomously by flying to the nearest frontier cluster.

The closed-loop mission uses point-to-line ICP. Pose estimators are then compared on the
same flown trajectory and the same scans (replay with common random numbers):
dead reckoning (odometry + gyro only), the card's point-to-point ICP, and point-to-line ICP
with different iteration caps. Map coverage is also recomputed for three grid resolutions.

Usage:
    conda activate drones
    python src/04_industrial_agriculture/s074_mine_mapping.py
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401
from scipy.spatial import cKDTree
from scipy.ndimage import binary_dilation, label
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra

# ── Parameters (card) ─────────────────────────────────────────────────────────
TUNNEL_HALF_WIDTH = 1.5      # m
N_BEAMS    = 180
R_LIDAR    = 10.0            # m
SIGMA_R    = 0.05            # m
K_ICP      = 20
EPS_ICP    = 1e-4            # m, stop when the mean residual changes by less than this
SIGMA_ICP  = 0.02            # m per step, per axis (odometry / motion noise)
VOX_RES    = 0.2             # m
L_OCC      = np.log(0.9 / 0.1)
L_FREE     = np.log(0.4 / 0.6)
L_MIN, L_MAX = -10.0, 10.0
P_OCC_THRESH, P_FREE_THRESH = 0.7, 0.3
V_NAV      = 1.0             # m/s
V_MAX      = 1.5             # m/s (asserted)
DT         = 0.1             # s
Z_FLIGHT   = 1.5             # m, constant flight / scan-plane height
T_TIMEOUT  = 1200.0          # s
STOP_DIST  = 0.5             # m, halt this far before the frontier target
SEED       = 42

# Branches (card code): (start, end) of the axis in the scan plane
BRANCHES = [((0.0, 0.0), (80.0, 0.0)),     # main corridor
            ((30.0, 0.0), (30.0, 35.0)),   # branch A (+y)
            ((50.0, 0.0), (50.0, -35.0)),  # branch B (-y)
            ((70.0, 0.0), (70.0, 25.0))]   # branch C (+y)

# ── Added parameters (see README Deviations) ──────────────────────────────────
SIGMA_GYRO   = 1e-4          # rad per step, gyro yaw-increment noise (about 1 deg/sqrt(h))
D_REJECT     = 0.5           # m, ICP correspondence rejection distance
NORMAL_RADIUS = 0.6          # m, neighbourhood radius for normal estimation
SIGMA_PL     = np.sqrt(2) * SIGMA_R   # m, point-to-line residual std (noise on both scans)
P0_INV       = np.diag([1 / SIGMA_ICP ** 2, 1 / SIGMA_ICP ** 2, 1 / SIGMA_GYRO ** 2])
INFLATE      = 0.6           # m, planner prefers to stay this far from occupied cells
HARD_CLEAR   = 0.4           # m, planner never enters cells this close to occupied cells
SOFT_COST    = 5.0           # path-cost multiplier inside the INFLATE zone
MIN_FRONTIER = 8             # cells, smaller frontier clusters are ignored
REPLAN_EVERY = 20            # steps
CARROT       = 0.6           # m, look-ahead along the planned path
IGNORE_R     = 1.0           # m, frontier cells this close to a reached goal are ignored
SAFE_R       = 0.7           # m, reactive safety distance on raw LiDAR returns
K_REPEL      = 1.0           # 1/s, repulsion gain of the safety filter
R_DRONE      = 0.25          # m, asserted clearance to the true walls
REPLAY_SEEDS = [101, 102, 103, 104, 105, 106]
K_SWEEP      = [1, 5, 20]
RES_SWEEP    = [0.1, 0.2, 0.4]
SNAP_EVERY   = 25            # steps per animation snapshot
GRID_MIN     = np.array([-2.0, -38.0])
GRID_MAX     = np.array([84.0, 38.0])

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '04_industrial_agriculture', 's074_mine_mapping',
)

RECTS = []
for (a, b) in BRANCHES:
    a, b = np.array(a), np.array(b)
    lo = np.minimum(a, b) - np.where(a == b, TUNNEL_HALF_WIDTH, 0.0)
    hi = np.maximum(a, b) + np.where(a == b, TUNNEL_HALF_WIDTH, 0.0)
    RECTS.append((lo, hi))
RECTS = [(np.array(lo), np.array(hi)) for lo, hi in RECTS]
TOTAL_LENGTH = sum(np.linalg.norm(np.subtract(b, a)) for a, b in BRANCHES)
PHI = np.arange(N_BEAMS) * 2 * np.pi / N_BEAMS
DIRS = np.stack([np.cos(PHI), np.sin(PHI)], axis=1)


def rot(th):
    c, s = np.cos(th), np.sin(th)
    return np.array([[c, -s], [s, c]])


# ── Ground truth ──────────────────────────────────────────────────────────────
def free_mask(pts):
    pts = np.atleast_2d(pts)
    m = np.zeros(len(pts), dtype=bool)
    for lo, hi in RECTS:
        m |= np.all((pts >= lo) & (pts <= hi), axis=1)
    return m


def wall_clearance(p):
    """Distance from p (inside the tunnels) to the nearest wall, by exact ray casting."""
    r = cast_rays(p, DIRS)
    return r.min()


def cast_rays(p, dirs):
    """Exact exit distance of rays from the union of axis-aligned rectangles."""
    t = np.zeros(len(dirs))
    for _ in range(len(RECTS) + 1):
        q = p[None, :] + (t[:, None] + 1e-7) * dirs
        best = t.copy()
        for lo, hi in RECTS:
            inside = np.all((q >= lo) & (q <= hi), axis=1)
            with np.errstate(divide='ignore', invalid='ignore'):
                tx = np.where(dirs[:, 0] > 0, (hi[0] - p[0]) / dirs[:, 0],
                              np.where(dirs[:, 0] < 0, (lo[0] - p[0]) / dirs[:, 0], np.inf))
                ty = np.where(dirs[:, 1] > 0, (hi[1] - p[1]) / dirs[:, 1],
                              np.where(dirs[:, 1] < 0, (lo[1] - p[1]) / dirs[:, 1], np.inf))
            best = np.where(inside, np.maximum(best, np.minimum(tx, ty)), best)
        if np.all(best == t):
            break
        t = best
    return t


class Grid:
    def __init__(self, res):
        self.res = res
        self.shape = tuple(np.ceil((GRID_MAX - GRID_MIN) / res).astype(int))
        self.lo = np.zeros(self.shape, dtype=np.float32)
        cx = GRID_MIN[0] + (np.arange(self.shape[0]) + 0.5) * res
        cy = GRID_MIN[1] + (np.arange(self.shape[1]) + 0.5) * res
        self.cx, self.cy = cx, cy

    def idx(self, pts):
        ij = np.floor((pts - GRID_MIN) / self.res).astype(int)
        ok = (ij[:, 0] >= 0) & (ij[:, 0] < self.shape[0]) & (ij[:, 1] >= 0) & (ij[:, 1] < self.shape[1])
        return ij[ok]

    def update(self, origin, body_pts, ranges_ok, th):
        """Free along each ray (card stepping: r = res, 2 res, ... < range - res), occupied
        at returned hits; no-return beams are free up to R_LIDAR and add no hit."""
        R = rot(th)
        rng_all = np.linalg.norm(body_pts, axis=1)
        dirs_w = (body_pts / np.maximum(rng_all, 1e-9)[:, None]) @ R.T
        n = np.floor((rng_all - self.res) / self.res).astype(int)
        n = np.maximum(n, 0)
        rep = np.repeat(np.arange(len(n)), n)
        steps = (np.arange(n.sum()) - np.repeat(np.cumsum(n) - n, n) + 1) * self.res
        free_pts = origin[None, :] + steps[:, None] * dirs_w[rep]
        fi = self.idx(free_pts)
        np.add.at(self.lo, (fi[:, 0], fi[:, 1]), L_FREE)
        hits = origin[None, :] + body_pts[ranges_ok] @ R.T
        hi = self.idx(hits)
        np.add.at(self.lo, (hi[:, 0], hi[:, 1]), L_OCC)
        np.clip(self.lo, L_MIN, L_MAX, out=self.lo)

    def classes(self):
        p = 1.0 / (1.0 + np.exp(-self.lo.astype(np.float64)))
        return (p < P_FREE_THRESH), (p > P_OCC_THRESH)


def gt_masks(res):
    g = Grid(res)
    X, Y = np.meshgrid(g.cx, g.cy, indexing='ij')
    free = free_mask(np.stack([X.ravel(), Y.ravel()], 1)).reshape(g.shape)
    wall = binary_dilation(free, structure=np.ones((3, 3))) & ~free
    return free, wall


# ── ICP ───────────────────────────────────────────────────────────────────────
def icp_p2p(src, tgt, th0, t0, k_max=K_ICP):
    """Card: point-to-point ICP with SVD. Returns (dth, t) with tgt ~ R(dth) src + t."""
    tree = cKDTree(tgt)
    R = rot(th0)
    t = t0.copy()
    prev = np.inf
    it = 0
    for it in range(1, k_max + 1):
        s = src @ R.T + t
        d, idx = tree.query(s)
        m = d < D_REJECT
        if m.sum() < 10:
            break
        p, q = s[m], tgt[idx[m]]
        pb, qb = p.mean(0), q.mean(0)
        W = (q - qb).T @ (p - pb)
        U, _, Vt = np.linalg.svd(W)
        Ri = U @ Vt
        if np.linalg.det(Ri) < 0:
            U[:, -1] *= -1
            Ri = U @ Vt
        ti = qb - Ri @ pb
        R = Ri @ R
        t = Ri @ t + ti
        res = d[m].mean()
        if abs(prev - res) < EPS_ICP:
            break
        prev = res
    return np.arctan2(R[1, 0], R[0, 0]), t, it


def normals(pts):
    """Surface normals by PCA over the returns within NORMAL_RADIUS (at most 10). Points
    with fewer than 4 such neighbours (sparse far returns) get no normal and are not used
    by point-to-line ICP. The radius is well below the 3 m tunnel width, so the opposite
    wall never enters a fit."""
    tree = cKDTree(pts)
    d, nn = tree.query(pts, k=min(10, len(pts)), distance_upper_bound=NORMAL_RADIUS)
    ok = np.isfinite(d)
    nn = np.where(ok, nn, 0)
    w = ok.astype(float)
    cnt = w.sum(1)
    P = pts[nn]
    mu = (P * w[:, :, None]).sum(1) / np.maximum(cnt, 1)[:, None]
    Q = (P - mu[:, None, :]) * w[:, :, None]
    C = np.einsum('nki,nkj->nij', Q, Q)
    _, v = np.linalg.eigh(C)
    return v[:, :, 0], cnt >= 4, tree


def icp_p2l(src, tgt, th0, t0, k_max=K_ICP, tgt_cache=None):
    """Point-to-line ICP fused with the odometry prior: Gauss-Newton on (tx, ty, th)
    minimising sum(r_i^2) / sigma_pl^2 + (x - x0)' P0^-1 (x - x0). In directions the
    scan cannot observe (along a featureless corridor) the prior is kept."""
    nrm, nvalid, tree = tgt_cache if tgt_cache is not None else normals(tgt)
    x0 = np.array([t0[0], t0[1], th0])
    x = x0.copy()
    prev = np.inf
    it = 0
    for it in range(1, k_max + 1):
        R = rot(x[2])
        s = src @ R.T + x[:2]
        d, idx = tree.query(s)
        m = (d < D_REJECT) & nvalid[idx]
        if m.sum() < 10:
            break
        n = nrm[idx[m]]
        e = s[m] - tgt[idx[m]]
        r = np.einsum('ij,ij->i', n, e)
        ds = np.stack([-(s[m, 1] - x[1]), s[m, 0] - x[0]], 1)   # d(R s)/d th
        J = np.column_stack([n[:, 0], n[:, 1], np.einsum('ij,ij->i', n, ds)])
        H = J.T @ J / SIGMA_PL ** 2 + P0_INV
        g = J.T @ r / SIGMA_PL ** 2 + P0_INV @ (x - x0)
        x = x - np.linalg.solve(H, g)
        res = np.abs(r).mean()
        if abs(prev - res) < EPS_ICP:
            break
        prev = res
    return x[2], x[:2], it


# ── Exploration ───────────────────────────────────────────────────────────────
OFFS = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
        (1, 1, np.sqrt(2)), (1, -1, np.sqrt(2)), (-1, 1, np.sqrt(2)), (-1, -1, np.sqrt(2))]


def plan(grid, p_est, keep_goal=None, ignored=None):
    free, occ = grid.classes()
    def disk(radius):
        rr = int(np.ceil(radius / grid.res))
        yy, xx = np.mgrid[-rr:rr + 1, -rr:rr + 1]
        return (xx ** 2 + yy ** 2) <= rr ** 2
    rad = int(np.ceil(INFLATE / grid.res))
    hard = binary_dilation(occ, structure=disk(HARD_CLEAR))     # never enter
    soft = binary_dilation(occ, structure=disk(INFLATE))        # allowed, but costly
    trav = free & ~hard
    s_ij = grid.idx(p_est[None, :])[0]
    # let the drone leave the keep-out zone if drift has put it there
    near_start = ((grid.cx[:, None] - p_est[0]) ** 2 + (grid.cy[None, :] - p_est[1]) ** 2
                  <= (HARD_CLEAR + grid.res) ** 2)
    trav |= free & near_start
    trav[s_ij[0], s_ij[1]] = True
    unknown = ~free & ~occ
    front = unknown & binary_dilation(free, structure=np.ones((3, 3)))
    if ignored is not None:
        front &= ~ignored
    lab, n_lab = label(front, structure=np.ones((3, 3)))
    if n_lab == 0:
        return None, 0, None
    sizes = np.bincount(lab.ravel())
    big = np.where(sizes >= MIN_FRONTIER)[0]
    big = big[big > 0]
    if len(big) == 0:
        return None, 0, None
    nid = -np.ones(grid.shape, dtype=np.int64)
    ti = np.argwhere(trav)
    nid[ti[:, 0], ti[:, 1]] = np.arange(len(ti))
    rows, cols, wts = [], [], []
    for dx, dy, w in OFFS:
        a = ti
        b = ti + np.array([dx, dy])
        ok = (b[:, 0] >= 0) & (b[:, 0] < grid.shape[0]) & (b[:, 1] >= 0) & (b[:, 1] < grid.shape[1])
        a, b = a[ok], b[ok]
        ok2 = trav[b[:, 0], b[:, 1]]
        rows.append(nid[a[ok2, 0], a[ok2, 1]])
        cols.append(nid[b[ok2, 0], b[ok2, 1]])
        pen = np.where(soft[b[ok2, 0], b[ok2, 1]], SOFT_COST, 1.0)
        wts.append(w * grid.res * pen)
    G = csr_matrix((np.concatenate(wts), (np.concatenate(rows), np.concatenate(cols))),
                   shape=(len(ti), len(ti)))
    start = nid[s_ij[0], s_ij[1]]
    dist, pred = dijkstra(G, indices=start, return_predecessors=True)
    # candidate goals: traversable cells next to a big frontier cluster
    lab_big = np.isin(lab, big) & (lab > 0)
    near = binary_dilation(lab_big, structure=np.ones((3, 3)), iterations=rad + 1) & trav
    cand = np.argwhere(near)
    if len(cand) == 0:
        return None, len(big), None
    cd = dist[nid[cand[:, 0], cand[:, 1]]]
    if not np.isfinite(cd).any():
        return None, len(big), None
    g = cand[int(np.argmin(cd))]
    if keep_goal is not None:
        # commit to the current target while it is still a reachable frontier goal
        kg = np.all(cand == keep_goal, axis=1)
        if kg.any() and np.isfinite(cd[kg][0]):
            g = keep_goal
    node = nid[g[0], g[1]]
    path = []
    while node != start and node >= 0:
        path.append(ti[node])
        node = pred[node]
    path = np.array(path[::-1]) if path else np.array([s_ij])
    world = GRID_MIN + (path + 0.5) * grid.res
    return world, len(big), np.array(g)


def run_simulation():
    rng = np.random.default_rng(SEED)
    grid = Grid(VOX_RES)
    p_true = np.array([1.5, 0.0])
    p_est, th_est = p_true.copy(), 0.0
    traj_true, traj_est, th_log = [p_true.copy()], [p_est.copy()], [0.0]
    scans_true, cmds, motion_noise, gyro_noise = [], [], [], []
    icp_iters, cov_log, snaps = [], [], []
    gt_free, gt_wall = gt_masks(VOX_RES)
    path, n_front, k_plan, goal_cell = None, -1, 0, None
    ignored = np.zeros(grid.shape, dtype=bool)
    prev_pts, prev_cache = None, None
    v_cmd = np.zeros(2)
    t, k = 0.0, 0
    min_clear = np.inf
    status = 'timeout'
    while t < T_TIMEOUT:
        # ── LiDAR (true pose; true yaw is 0) ─────────────────────────────────
        r_true = cast_rays(p_true, DIRS)
        ok = r_true < R_LIDAR
        r_meas = np.clip(r_true + rng.normal(0, SIGMA_R, N_BEAMS), 0.05, R_LIDAR)
        r_meas[~ok] = R_LIDAR
        scans_true.append(r_true)
        body = r_meas[:, None] * DIRS
        pts = body[ok]
        # ── ICP with odometry prior ──────────────────────────────────────────
        if prev_pts is not None:
            prior_t = rot(-th_est_prev) @ v_cmd * DT                # commanded body displacement
            prior_th = gyro_noise[-1]
            th_d, t_d, it = icp_p2l(pts, prev_pts, prior_th, prior_t, tgt_cache=prev_cache)
            icp_iters.append(it)
            p_est = p_est + rot(th_est) @ t_d
            th_est = th_est + th_d
        prev_pts, prev_cache = pts, normals(pts)
        grid.update(p_est, body, ok, th_est)
        # ── frontier planning ────────────────────────────────────────────────
        reached = path is not None and np.linalg.norm(path[-1] - p_est) < STOP_DIST
        if reached:
            # frontier cells around a reached goal that are still unknown cannot be observed
            # from the tunnel (e.g. rock behind a junction corner): ignore them from now on
            gx, gy = path[-1]
            ignored |= ((grid.cx[:, None] - gx) ** 2 + (grid.cy[None, :] - gy) ** 2
                        <= IGNORE_R ** 2)
        if path is None or reached or k - k_plan >= REPLAN_EVERY:
            path, n_front, goal_cell = plan(grid, p_est, None if reached else goal_cell, ignored)
            k_plan = k
            if path is None:
                status = 'no frontier left' if n_front == 0 else 'frontiers unreachable'
                break
        # carrot on the path
        dpath = np.linalg.norm(path - p_est, axis=1)
        j = int(np.argmin(dpath))
        ahead = path[j:]
        far = np.where(np.linalg.norm(ahead - p_est, axis=1) >= CARROT)[0]
        goal = ahead[far[0]] if len(far) else path[-1]
        d = goal - p_est
        dist_goal = np.linalg.norm(path[-1] - p_est)
        if dist_goal < STOP_DIST or np.linalg.norm(d) < 1e-9:
            v_cmd = np.zeros(2)
        else:
            v_cmd = V_NAV * d / np.linalg.norm(d)
        # ── reactive safety filter in the body frame (raw LiDAR, independent of drift):
        # remove the velocity component toward returns closer than SAFE_R and push away
        v_body = rot(-th_est) @ v_cmd
        close = ok & (r_meas < SAFE_R)
        for u, r in zip(DIRS[close], r_meas[close]):
            toward = v_body @ u
            if toward > 0:
                v_body = v_body - toward * u
            v_body = v_body - K_REPEL * (SAFE_R - r) * u
        sp = np.linalg.norm(v_body)
        if sp > V_NAV:
            v_body *= V_NAV / sp
        v_cmd = rot(th_est) @ v_body
        # ── true motion: body command executed with true yaw 0, plus noise ───
        w = rng.normal(0, SIGMA_ICP, 2)
        g = rng.normal(0, SIGMA_GYRO)
        step = rot(-th_est) @ v_cmd * DT + w
        th_est_prev = th_est
        p_true = p_true + step
        motion_noise.append(w)
        gyro_noise.append(g)
        cmds.append(rot(-th_est_prev) @ v_cmd * DT)
        # invariants
        assert np.linalg.norm(rot(-th_est) @ v_cmd) <= V_MAX + 1e-9, 'speed limit'
        assert free_mask(p_true)[0], 'drone left the tunnel'
        clear = wall_clearance(p_true)
        min_clear = min(min_clear, clear)
        assert clear >= R_DRONE, f'wall clearance {clear:.2f} m below drone radius'
        t += DT
        k += 1
        traj_true.append(p_true.copy())
        traj_est.append(p_est.copy())
        th_log.append(th_est)
        if k % 50 == 0:
            fr, oc = grid.classes()
            cov_log.append((t, (fr & gt_free).sum() / gt_free.sum(),
                            ((fr | oc) & gt_free).sum() / gt_free.sum()))
        if k % SNAP_EVERY == 0:
            fr, oc = grid.classes()
            snaps.append((k, (fr.astype(np.int8) - oc.astype(np.int8))))
    # final scan bookkeeping
    fr, oc = grid.classes()
    cov_log.append((t, (fr & gt_free).sum() / gt_free.sum(), ((fr | oc) & gt_free).sum() / gt_free.sum()))
    snaps.append((k, (fr.astype(np.int8) - oc.astype(np.int8))))
    traj_true, traj_est = np.array(traj_true), np.array(traj_est)
    return dict(grid=grid, traj_true=traj_true, traj_est=traj_est, th=np.array(th_log),
                scans=np.array(scans_true), cmds=np.array(cmds), w=np.array(motion_noise),
                g=np.array(gyro_noise), icp_iters=np.array(icp_iters), cov_log=np.array(cov_log),
                snaps=snaps, t_exp=t, status=status, min_clear=min_clear, n_front=n_front,
                n_ignored=int(ignored.sum()),
                gt_free=gt_free, gt_wall=gt_wall)


# ── Replays on the flown trajectory (common random numbers) ───────────────────
def prepare_replay(mission, seed):
    """Fresh LiDAR / odometry / gyro noise for the flown trajectory (shared by all
    estimators of this seed: common random numbers)."""
    rng = np.random.default_rng(seed)
    tt = mission['traj_true']
    true_step = np.diff(tt, axis=0)          # true yaw is 0, so body frame = world frame
    pts, caches = [], []
    for r_true in mission['scans']:
        ok = r_true < R_LIDAR
        r = np.clip(r_true + rng.normal(0, SIGMA_R, N_BEAMS), 0.05, R_LIDAR)
        p = (r[:, None] * DIRS)[ok]
        pts.append(p)
        caches.append(normals(p))
    prior_t = true_step - rng.normal(0, SIGMA_ICP, true_step.shape)
    prior_th = rng.normal(0, SIGMA_GYRO, len(true_step))
    return dict(tt=tt, pts=pts, caches=caches, prior_t=prior_t, prior_th=prior_th)


def replay(prep, method, k_max=K_ICP):
    """Re-estimate the pose along the flown trajectory with one estimator."""
    tt = prep['tt']
    p, th = tt[0].copy(), 0.0
    est = [p.copy()]
    iters = []
    for k in range(1, len(prep['pts'])):
        t0, th0 = prep['prior_t'][k - 1], prep['prior_th'][k - 1]
        if method == 'dr':
            th_d, t_d = th0, t0
        elif method == 'p2p':
            th_d, t_d, it = icp_p2p(prep['pts'][k], prep['pts'][k - 1], th0, t0, k_max)
            iters.append(it)
        else:
            th_d, t_d, it = icp_p2l(prep['pts'][k], prep['pts'][k - 1], th0, t0, k_max,
                                    prep['caches'][k - 1])
            iters.append(it)
        p = p + rot(th) @ t_d
        th = th + th_d
        est.append(p.copy())
    est = np.array(est)
    err = np.linalg.norm(est - tt[:len(est)], axis=1)
    return dict(est=est, err=err, rmse=float(np.sqrt(np.mean(err ** 2))), final=float(err[-1]),
                iters=float(np.mean(iters)) if iters else 0.0)


def remap(mission, res, use_true=False):
    """Rebuild the map at another resolution from the stored scans (fixed noise seed)."""
    rng = np.random.default_rng(777)
    g = Grid(res)
    poses = mission['traj_true'] if use_true else mission['traj_est']
    ths = np.zeros(len(poses)) if use_true else mission['th']
    for k, r_true in enumerate(mission['scans']):
        ok = r_true < R_LIDAR
        r = np.clip(r_true + rng.normal(0, SIGMA_R, N_BEAMS), 0.05, R_LIDAR)
        r[~ok] = R_LIDAR
        g.update(poses[k], r[:, None] * DIRS, ok, ths[k])
    gt_free, gt_wall = gt_masks(res)
    fr, oc = g.classes()
    return dict(cov=(fr & gt_free).sum() / gt_free.sum(),
                classified=((fr | oc) & gt_free).sum() / gt_free.sum(),
                wall_hit=(oc & gt_wall).sum() / gt_wall.sum(),
                false_free=(fr & ~gt_free).sum() / max(fr.sum(), 1),
                cells=g.lo.size, mem_kb=g.lo.nbytes / 1024)


# ── Plots ─────────────────────────────────────────────────────────────────────
def _class_img(grid):
    fr, oc = grid.classes()
    img = np.full(grid.shape, 0.5)
    img[fr] = 0.0
    img[oc] = 1.0
    return img.T


def _extent(grid):
    return [GRID_MIN[0], GRID_MIN[0] + grid.shape[0] * grid.res,
            GRID_MIN[1], GRID_MIN[1] + grid.shape[1] * grid.res]


def plot_map_3d(m, out_dir):
    grid = m['grid']
    _, oc = grid.classes()
    ij = np.argwhere(oc)[::2]
    xy = GRID_MIN + (ij + 0.5) * grid.res
    fig = plt.figure(figsize=(13, 8))
    ax = fig.add_subplot(111, projection='3d')
    for z in (0.3, 1.5, 2.7):
        ax.scatter(xy[:, 0], xy[:, 1], np.full(len(xy), z), s=0.3, c='grey', alpha=0.25)
    tt, te = m['traj_true'], m['traj_est']
    ax.plot(tt[:, 0], tt[:, 1], np.full(len(tt), Z_FLIGHT), 'g-', lw=1.2, label='Ground truth')
    ax.plot(te[:, 0], te[:, 1], np.full(len(te), Z_FLIGHT), 'r--', lw=1.0, label='ICP estimate')
    ax.scatter([], [], [], c='grey', s=5, label='Occupied cells (scan plane, extruded 0-3 m)')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_zlim(0, 3)
    ax.set_box_aspect((86, 76, 20))
    ax.set_zticks([0, 1.5, 3])
    ax.view_init(elev=35, azim=-60)
    ax.set_title(f"S074 occupancy map and trajectories (T_exp = {m['t_exp']:.0f} s)")
    ax.legend(loc='upper left', fontsize=8)
    path = os.path.join(out_dir, 'map_3d.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_map_topdown(m, out_dir):
    grid = m['grid']
    fig, ax = plt.subplots(figsize=(11, 9.5))
    ax.imshow(_class_img(grid), origin='lower', extent=_extent(grid), cmap='RdYlGn_r',
              vmin=0, vmax=1, interpolation='nearest')
    for lo, hi in RECTS:
        ax.add_patch(plt.Rectangle(lo, *(hi - lo), fill=False, ec='k', lw=0.5, ls=':'))
    ax.plot(m['traj_true'][:, 0], m['traj_true'][:, 1], 'b-', lw=1, label='ground truth path')
    ax.plot(m['traj_est'][:, 0], m['traj_est'][:, 1], 'm--', lw=1, label='ICP estimate path')
    ax.plot(*m['traj_true'][0], 'k^', ms=9, label='entrance / start')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('Map at mission end (green = free, red = occupied, yellow = unknown); '
                 'dotted = true tunnel outline')
    ax.legend(loc='lower right', fontsize=8)
    path = os.path.join(out_dir, 'map_topdown.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_coverage(m, out_dir):
    c = m['cov_log']
    fig, ax = plt.subplots(figsize=(9, 4.5))
    ax.plot(c[:, 0], 100 * c[:, 1], 'b-', label='true free cells mapped as free')
    ax.plot(c[:, 0], 100 * c[:, 2], 'c--', label='true free cells classified (free or occupied)')
    ax.axhline(90, color='orange', ls='--', lw=1, label='90%')
    ax.axhline(95, color='red', ls='--', lw=1, label='95%')
    ax.set_xlabel('time (s)')
    ax.set_ylabel('coverage (%)')
    ax.set_ylim(0, 102)
    ax.set_title('Map coverage of the scan plane vs time')
    ax.legend(fontsize=8, loc='lower right')
    ax.grid(alpha=0.3)
    path = os.path.join(out_dir, 'coverage_vs_time.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_drift(m, rep, out_dir):
    t = np.arange(len(m['traj_true'])) * DT
    fig, ax = plt.subplots(figsize=(10, 5))
    err_cl = np.linalg.norm(m['traj_est'] - m['traj_true'], axis=1)
    ax.plot(t, err_cl, 'r-', lw=1.2, label='closed-loop mission (point-to-line ICP)')
    s0 = REPLAY_SEEDS[0]
    for key, c, lab in [('dr', 'tab:blue', 'dead reckoning (replay)'),
                        ('p2p', 'tab:orange', 'point-to-point ICP, card (replay)'),
                        (('p2l', K_ICP), 'tab:green', 'point-to-line ICP (replay)')]:
        e = rep[key][s0]['err']
        ax.plot(t[:len(e)], e, color=c, lw=0.9, label=lab)
    n = np.arange(len(t))
    ax.plot(t, SIGMA_ICP * np.sqrt(n), 'k--', lw=1, label=r'$\sigma_{ICP}\sqrt{n}$ (one axis)')
    ax.plot(t, SIGMA_ICP * np.sqrt(2 * n), 'k:', lw=1, label=r'$\sigma_{ICP}\sqrt{2n}$ (2D)')
    ax.set_yscale('log')
    ax.set_ylim(1e-3, None)
    ax.set_xlabel('time (s)')
    ax.set_ylabel('position error (m)')
    ax.set_title(f'Pose drift on the flown trajectory (replays: seed {s0})')
    ax.legend(fontsize=8, loc='lower right')
    ax.grid(alpha=0.3, which='both')
    path = os.path.join(out_dir, 'pose_drift.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_logodds(m, out_dir):
    lo = m['grid'].lo.ravel()
    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.hist(lo[lo != 0], bins=80, color='steelblue')
    ax.axvline(np.log(P_OCC_THRESH / (1 - P_OCC_THRESH)), color='red', ls='--',
               label='occupied threshold (P = 0.7)')
    ax.axvline(np.log(P_FREE_THRESH / (1 - P_FREE_THRESH)), color='green', ls='--',
               label='free threshold (P = 0.3)')
    ax.set_yscale('log')
    ax.set_xlabel(r'log-odds $\ell(v)$')
    ax.set_ylabel('cells (log scale)')
    ax.set_title('Log-odds of updated cells at mission end')
    ax.legend(fontsize=8)
    path = os.path.join(out_dir, 'logodds_hist.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_sensitivity(rep, res_tab, out_dir):
    fig, axes = plt.subplots(1, 2, figsize=(13, 4.8))
    labels = ['DR'] + ['p2p\nK=20'] + [f'p2l\nK={k}' for k in K_SWEEP]
    keys = ['dr', 'p2p'] + [('p2l', k) for k in K_SWEEP]
    vals = np.array([[rep[key][s]['rmse'] for s in REPLAY_SEEDS] for key in keys])
    cols = ['tab:blue', 'tab:orange'] + ['tab:green'] * len(K_SWEEP)
    axes[0].bar(range(len(keys)), vals.mean(1), yerr=vals.std(1), color=cols, capsize=4,
                edgecolor='k')
    for i, v in enumerate(vals.mean(1)):
        axes[0].text(i, v * 1.05 + 0.01, f'{v:.2f}', ha='center', fontsize=8)
    axes[0].set_xticks(range(len(keys)))
    axes[0].set_xticklabels(labels, fontsize=8)
    axes[0].set_ylabel('pose RMSE (m), mean ± std')
    axes[0].set_title(f'Pose estimators on the flown trajectory ({len(REPLAY_SEEDS)} noise seeds)')
    axes[0].set_yscale('log')
    x = np.arange(len(RES_SWEEP))
    axes[1].bar(x - 0.2, [100 * res_tab[r]['cov'] for r in RES_SWEEP], 0.4, color='tab:blue',
                label='coverage (est. poses)')
    axes[1].bar(x + 0.2, [100 * res_tab[('true', r)]['cov'] for r in RES_SWEEP], 0.4,
                color='tab:cyan', label='coverage (true poses)')
    for i, r in enumerate(RES_SWEEP):
        axes[1].text(i - 0.2, 100 * res_tab[r]['cov'] + 1, f"{100 * res_tab[r]['cov']:.1f}",
                     ha='center', fontsize=8)
        axes[1].text(i + 0.2, 100 * res_tab[('true', r)]['cov'] + 1,
                     f"{100 * res_tab[('true', r)]['cov']:.1f}", ha='center', fontsize=8)
    axes[1].set_xticks(x)
    axes[1].set_xticklabels([f'{r} m\n{res_tab[r]["mem_kb"]:.0f} kB' for r in RES_SWEEP])
    axes[1].set_ylabel('true free cells mapped as free (%)')
    axes[1].set_ylim(0, 110)
    axes[1].set_title('Map coverage vs grid resolution (same scans)')
    axes[1].legend(fontsize=8, loc='lower right')
    plt.tight_layout()
    path = os.path.join(out_dir, 'sensitivity.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def save_animation(m, out_dir):
    grid = m['grid']
    snaps = m['snaps']
    tt, te = m['traj_true'], m['traj_est']
    fig, ax = plt.subplots(figsize=(8, 7))
    img = ax.imshow(np.zeros(grid.shape).T, origin='lower', extent=_extent(grid),
                    cmap='RdYlGn_r', vmin=0, vmax=1, interpolation='nearest')
    lt, = ax.plot([], [], 'b-', lw=1, label='ground truth')
    le, = ax.plot([], [], 'm--', lw=1, label='ICP estimate')
    dt_, = ax.plot([], [], 'o', color='blue', ms=6)
    de, = ax.plot([], [], 's', color='m', ms=5)
    txt = ax.text(0.02, 0.97, '', transform=ax.transAxes, va='top', fontsize=9,
                  bbox=dict(fc='white', alpha=0.8))
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.legend(loc='lower right', fontsize=8)
    ax.set_title('S074 frontier exploration with ICP mapping')

    def update(f):
        k, cls = snaps[f]
        im = np.full(cls.shape, 0.5)
        im[cls == 1] = 0.0
        im[cls == -1] = 1.0
        img.set_data(im.T)
        lt.set_data(tt[:k + 1, 0], tt[:k + 1, 1])
        le.set_data(te[:k + 1, 0], te[:k + 1, 1])
        dt_.set_data([tt[k, 0]], [tt[k, 1]])
        de.set_data([te[k, 0]], [te[k, 1]])
        txt.set_text(f't = {k * DT:5.1f} s | error {np.linalg.norm(te[k] - tt[k]):.2f} m')
        return img, lt, le, dt_, de, txt

    anim = animation.FuncAnimation(fig, update, frames=len(snaps), interval=100, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    anim.save(path, writer='pillow', fps=10, dpi=70)
    plt.close(fig)
    print(f'Saved: {path} ({len(snaps)} frames)')


if __name__ == '__main__':
    m = run_simulation()
    err = np.linalg.norm(m['traj_est'] - m['traj_true'], axis=1)
    print('\n=== S074 Mine Mapping — Results ===')
    print(f'  Tunnel network: {len(BRANCHES)} segments, total centreline length {TOTAL_LENGTH:.0f} m')
    print(f"  Mission ended: {m['status']} at T_exp = {m['t_exp']:.1f} s "
          f"({len(m['traj_true']) - 1} steps); path flown "
          f"{np.sum(np.linalg.norm(np.diff(m['traj_true'], axis=0), axis=1)):.1f} m")
    c = m['cov_log'][-1]
    print(f'  Coverage (true free cells mapped free): {100 * c[1]:.1f}%; classified (free or '
          f'occupied): {100 * c[2]:.1f}%')
    for thr in (0.9, 0.95):
        hit = np.where(m['cov_log'][:, 1] >= thr)[0]
        print(f"    reached {int(100 * thr)}% at t = "
              + (f"{m['cov_log'][hit[0], 0]:.0f} s" if len(hit) else 'never'))
    print(f'  Closed-loop pose RMSE {np.sqrt(np.mean(err ** 2)):.3f} m, final error {err[-1]:.3f} m, '
          f'max {err.max():.3f} m; final heading error {np.degrees(m["th"][-1]):+.2f} deg')
    print(f"  Frontier clusters (>= {MIN_FRONTIER} cells) left at the end: {m['n_front']}; "
          f"cells ignored near reached goals: {m['n_ignored']}")
    print(f"  ICP iterations per scan: mean {m['icp_iters'].mean():.2f}, max {m['icp_iters'].max()}")
    lo = m['grid'].lo.ravel()
    upd = lo[lo != 0]
    print(f'  Log-odds of updated cells: {upd.size} cells, {100 * np.mean(upd <= L_MIN + 1e-6):.1f}% '
          f'at the -10 clamp, {100 * np.mean(upd >= L_MAX - 1e-6):.1f}% at the +10 clamp')
    print(f"  Minimum true wall clearance {m['min_clear']:.2f} m (asserted >= {R_DRONE} m)")
    n = len(err) - 1
    print(f'  Card drift model sigma_ICP sqrt(n) at n = {n}: {SIGMA_ICP * np.sqrt(n):.2f} m '
          f'(2D: {SIGMA_ICP * np.sqrt(2 * n):.2f} m)')
    print(f'  [Replays on the flown trajectory, seeds {REPLAY_SEEDS}]')
    rep = {}
    keys = ['dr', 'p2p'] + [('p2l', k) for k in K_SWEEP]
    preps = {s: prepare_replay(m, s) for s in REPLAY_SEEDS}
    for key in keys:
        rep[key] = {}
        for s in REPLAY_SEEDS:
            if key == 'dr':
                rep[key][s] = replay(preps[s], 'dr')
            elif key == 'p2p':
                rep[key][s] = replay(preps[s], 'p2p', K_ICP)
            else:
                rep[key][s] = replay(preps[s], 'p2l', key[1])
        rm = np.array([rep[key][s]['rmse'] for s in REPLAY_SEEDS])
        fe = np.array([rep[key][s]['final'] for s in REPLAY_SEEDS])
        it = np.array([rep[key][s]['iters'] for s in REPLAY_SEEDS])
        name = key if isinstance(key, str) else f'p2l K={key[1]}'
        print(f'    {name:<10} RMSE {rm.mean():7.3f} ± {rm.std():6.3f} m, final {fe.mean():7.3f} '
              f'± {fe.std():6.3f} m, mean iterations {it.mean():5.2f}')
    for a, b in [('dr', ('p2l', K_ICP)), ('p2p', ('p2l', K_ICP)), ('dr', 'p2p')]:
        ra = np.array([rep[a][s]['rmse'] for s in REPLAY_SEEDS])
        rb = np.array([rep[b][s]['rmse'] for s in REPLAY_SEEDS])
        na = a if isinstance(a, str) else f'p2l K={a[1]}'
        nb = b if isinstance(b, str) else f'p2l K={b[1]}'
        print(f'    paired RMSE {nb} < {na}: {int((rb < ra).sum())}/{len(REPLAY_SEEDS)}')
    print('  [Map rebuilt from the same scans at other resolutions]')
    res_tab = {}
    for r in RES_SWEEP:
        res_tab[r] = remap(m, r)
        res_tab[('true', r)] = remap(m, r, use_true=True)
        a, b = res_tab[r], res_tab[('true', r)]
        print(f"    res {r:.1f} m: cells {a['cells']:7d} ({a['mem_kb']:6.0f} kB); est poses: "
              f"coverage {100 * a['cov']:5.1f}%, walls found {100 * a['wall_hit']:5.1f}%, "
              f"free cells outside tunnel {100 * a['false_free']:4.1f}% | true poses: coverage "
              f"{100 * b['cov']:5.1f}%, walls {100 * b['wall_hit']:5.1f}%, outside "
              f"{100 * b['false_free']:4.1f}%")
    out = os.path.normpath(OUTPUT_DIR)
    os.makedirs(out, exist_ok=True)
    plot_map_3d(m, out)
    plot_map_topdown(m, out)
    plot_coverage(m, out)
    plot_drift(m, rep, out)
    plot_logodds(m, out)
    plot_sensitivity(rep, res_tab, out)
    save_animation(m, out)
