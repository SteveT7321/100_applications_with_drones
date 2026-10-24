"""
S074 3D Mine Mapping (multi-level mine, shaft traversal)
========================================================
The 2D S074 flies at a fixed z = 1.5 m with a horizontal 180-beam LiDAR and maps one
plane of a single-level mine. Here the mine has three galleries (centre heights 1.5, 7.0
and 11.0 m, 3 x 3 m section) joined by two vertical 2 x 2 m shafts, and the drone carries
a 16-layer LiDAR (360 x 16 beams, +-15 deg, 10 m) plus up/down rangefinders. It builds a
0.2 m log-odds voxel map and explores by flying to the nearest 3D frontier: lateral
frontiers at a gallery level, and vertical frontiers that open a candidate shaft. A
candidate is tested with the rangefinders from below (or above); a confirmed shaft is
traversed as the card describes: align under the opening centre, then climb (descend) to
the gallery beyond. Planning keeps traversable cells >= 0.4 m from every occupied voxel.
Altitude is therefore a planned state. Dust attenuates beams by Beer-Lambert.
Robustness (seeds, layouts, planner constants): s074_3d_mine_mapping_robustness.py.

Missions (seed 42), mapped and navigated with the true pose:
  3d        16-layer LiDAR + rangefinders, 3D frontiers, shaft traversal   (card)
  3d_fixed  16-layer LiDAR, altitude held at 1.5 m (no shaft traversal)    (ablation)
  2d        the 2D base: 180-beam planar LiDAR, altitude held at 1.5 m      (2D baseline)
Pose estimation (point-to-plane ICP on x, y, z, yaw with an odometry prior, degeneracy-
aware, plus a rangefinder altitude reference) is evaluated by replays on the flown 3D
trajectory (3 noise seeds, against dead reckoning), by rebuilding the map with the ICP
poses, and by a capped closed-loop run with the estimate. Dust: card values and stronger
extinction coefficients (map rebuilt from the Level 1 scans).

Usage:
    conda activate drones
    python src/04_industrial_agriculture/3d/s074_3d_mine_mapping.py
"""

import os
import sys
import time
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from scipy.spatial import cKDTree
from scipy.ndimage import binary_dilation, label, maximum_filter, find_objects
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, '..', '..', '..'))
OUTPUT_DIR = os.path.join(PROJECT_ROOT, 'outputs', '04_industrial_agriculture', '3d',
                          's074_3d_mine_mapping')

# ── Parameters (card) ─────────────────────────────────────────────────────────
Z_LEVELS = (1.5, 7.0, 11.0)      # gallery centre heights
HALF_W = 1.5                     # tunnel half-width (3 x 3 m section)
SHAFT_R = 1.0                    # shaft half-width (2 x 2 m)
N_AZ, N_EL = 360, 16
EL_MAX = np.deg2rad(15.0)
R_LIDAR = 10.0
SIGMA_R = 0.05
C0, H_DUST = 0.8, 4.0            # dust peak concentration / scale height
MU_EXT = 0.15                    # 1/(m ppm)
I_THRESH = 0.05
K_ICP = 20
EPS_ICP = 1e-4
DT_SCAN = 0.5                    # s between LiDAR scans
VOX = 0.2
L_OCC = np.log(0.9 / 0.1)
L_FREE = np.log(0.4 / 0.6)
L_MIN, L_MAX = -10.0, 10.0
LO_FREE_T, LO_OCC_T = np.log(0.3 / 0.7), np.log(0.7 / 0.3)
V_NAV, V_MAX, VZ_MAX = 1.0, 1.5, 0.8
Z_MIN, Z_MAX = 0.3, 12.5
DT = 0.1
SEED = 42
# Kept from the 2D base
SIGMA_ODO = 0.02                 # m per 0.1 s step per axis (base SIGMA_ICP motion/odometry noise)
SIGMA_GYRO = 1e-4                # rad per step
N_AZ_2D = 180                    # base planar LiDAR
D_REJECT = 0.5
NORMAL_RADIUS = 0.6
K_NORMAL = 10
PLANAR_RATIO = 0.1
LAM_TH = 3e4
MAX_VISITS = 3
SHAFT_ZPAD = 3.0                 # m, vertical reach of a shaft box beyond its frontier cells
REJECT_EXCL = 1.0                # m, vertical clusters this close to a rejected test point stay excluded
REJECT_R = 0.3                   # m, a candidate shaft is rejected only from near its centre
VISIT_GAP = 3 * int(round(DT_SCAN / DT))   # steps (3 scans) between counted visits
LAT_DZ = 0.6                     # m, lateral frontier: has cells this close to a cruise level
SHAFT_DZ = 1.8                   # m, vertical frontier: beyond the gallery half-height (1.5 m) + 0.3 m
SHAFT_PAD = 0.6                  # m, shaft box margin
CLR_VOX = 1                      # voxels of occupied-map dilation: traversable cell centres >= 0.4 m from walls
T_TRAV_MAX = 120.0               # s, a shaft traverse that has not arrived by then is given up
TRAV_TOL = 0.5                   # m, arrival tolerance at the gallery beyond a shaft
ALIGN_TOL = 0.15                 # m, horizontal alignment with the cell under the shaft centre before climbing
ALIGN_MAX = 0.6                  # m, that cell must be this close to the estimated shaft centre
TRAV_STOP = 0.1                  # m, stop distance while traversing a shaft
CLIMB_HTOL = 0.1                 # m, vertical motion only while this close to the shaft axis
CENTRE_WIN = 1.5                 # m, search window for the opening around the drone
GUARD_DZ = 0.2                   # m, wait this far inside the gallery until the shaft axis is re-centred
CLIMB_VXY = 0.3                  # m/s, horizontal speed limit of the axis-holding controller
R_RF = 12.0                      # m, up/down rangefinder range
RF_GATE = 0.3                    # m, rangefinder pair used only if consistent with the prior
STOP_DIST = 0.5
IGNORE_R = 1.0
SAFE_R = 0.7
K_REPEL = 1.0
R_DRONE = 0.25
T_TIMEOUT = 1500.0

# ── Added parameters (see README Deviations) ──────────────────────────────────
PLAN_RES = 0.4                   # planning grid (2 x 2 x 2 map voxels)
MIN_FRONTIER = 8                 # planning cells per frontier cluster
GOAL_CELLS = 2                   # goal = traversable cell within 2 planning cells of a frontier
SOFT_COST = 5.0
REPLAN_EVERY = 30                # steps
CARROT = 0.6
ICP_SUB = 5                      # use every 5th azimuth for ICP
REPLAY_SEEDS = [101, 102, 103]
LAM_SWEEP = [0.0, 3e4]  # degeneracy threshold sweep (first LAM_N scans, seed 101)
LAM_N = 400
T_PROBE = 200.0                  # s, cap of the closed-loop run with the estimated pose
MU_SWEEP = [1.0, 6.0, 10.0]  # multiples of MU_EXT for the dust remaps
SNAP_EVERY = 80                  # steps per animation frame
SCAN_STEPS = int(round(DT_SCAN / DT))

# Mine geometry: axis-aligned boxes (lo, hi)
L1, L2, L3 = Z_LEVELS
BOXES = {
    'L1 main':   ((0.0, -HALF_W, L1 - 1.5), (80.0, HALF_W, L1 + 1.5)),
    'L1 branch': ((50.0 - HALF_W, -20.0, L1 - 1.5), (50.0 + HALF_W, 0.0, L1 + 1.5)),
    'L2 main':   ((30.0 - HALF_W, -HALF_W, L2 - 1.5), (61.5, HALF_W, L2 + 1.5)),
    'L2 branch': ((45.0 - HALF_W, 0.0, L2 - 1.5), (45.0 + HALF_W, 20.0, L2 + 1.5)),
    'L3 drift':  ((60.0 - HALF_W, -HALF_W, L3 - 1.5), (60.0 + HALF_W, 20.0 + HALF_W, L3 + 1.5)),
    'L3 cross':  ((60.0, 20.0 - HALF_W, L3 - 1.5), (80.0, 20.0 + HALF_W, L3 + 1.5)),
    'Shaft 1':   ((30.0 - SHAFT_R, -SHAFT_R, L1), (30.0 + SHAFT_R, SHAFT_R, L2)),
    'Shaft 2':   ((60.0 - SHAFT_R, -SHAFT_R, L2), (60.0 + SHAFT_R, SHAFT_R, L3)),
}
BLO = np.array([b[0] for b in BOXES.values()])
BHI = np.array([b[1] for b in BOXES.values()])
LEVEL_OF_BOX = [1, 1, 2, 2, 3, 3, 0, 0]      # 0 = shaft
START = np.array([1.5, 0.0, L1])
GMIN = np.array([-1.0, -21.6, -0.4])
GSHAPE = (416, 222, 68)                      # 0.2 m voxels, even sizes for 2x pooling
PSHAPE = tuple(s // 2 for s in GSHAPE)

EL = np.linspace(-EL_MAX, EL_MAX, N_EL)


def lidar_dirs(n_az, els):
    az = np.arange(n_az) * 2 * np.pi / n_az
    A, E = np.meshgrid(az, els, indexing='ij')
    d = np.stack([np.cos(E) * np.cos(A), np.cos(E) * np.sin(A), np.sin(E)], -1).reshape(-1, 3)
    return d, A.ravel(), E.ravel()


DIRS3, AZ3, EL3 = lidar_dirs(N_AZ, EL)
DIRS2, AZ2, EL2 = lidar_dirs(N_AZ_2D, np.array([0.0]))
RF_DIRS = np.array([[0.0, 0.0, 1.0], [0.0, 0.0, -1.0]])
DIRS_ALL = np.vstack([DIRS3, RF_DIRS])
CLEAR_DIRS = np.array([(i, j, k) for i in (-1, 0, 1) for j in (-1, 0, 1) for k in (-1, 0, 1)
                       if (i, j, k) != (0, 0, 0)], float)
CLEAR_DIRS /= np.linalg.norm(CLEAR_DIRS, axis=1, keepdims=True)


def rotz(th):
    c, s = np.cos(th), np.sin(th)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


# ── Ground truth ──────────────────────────────────────────────────────────────
def inside(pts):
    pts = np.atleast_2d(pts)
    return np.any(np.all((pts[:, None, :] >= BLO[None]) & (pts[:, None, :] <= BHI[None]), axis=2), axis=1)


def cast3d(p, dirs):
    """Exact exit distance of rays from the union of axis-aligned boxes."""
    t = np.zeros(len(dirs))
    with np.errstate(divide='ignore', invalid='ignore'):
        inv = 1.0 / dirs
    for _ in range(len(BLO) + 2):
        q = p[None, :] + (t[:, None] + 1e-7) * dirs
        best = t.copy()
        for lo, hi in zip(BLO, BHI):
            ins = np.all((q >= lo) & (q <= hi), axis=1)
            if not ins.any():
                continue
            with np.errstate(invalid='ignore'):
                tb = np.where(dirs > 0, (hi - p) * inv, np.where(dirs < 0, (lo - p) * inv, np.inf))
            best = np.where(ins, np.maximum(best, tb.min(axis=1)), best)
        if np.array_equal(best, t):
            break
        t = best
    return t


def clearance(p):
    return float(cast3d(p, CLEAR_DIRS).min())


def dust_reach(z0, sin_el, mu):
    """Range at which the one-way transmission exp(-tau) falls to I_THRESH (inf if never)."""
    tau = np.log(1.0 / I_THRESH)
    k = mu * C0
    e0 = np.exp(-z0 / H_DUST)
    out = np.full(len(sin_el), np.inf)
    flat = np.abs(sin_el) < 1e-9
    out[flat] = tau / (k * e0)
    s = sin_el[~flat]
    rhs = e0 - tau * s / (k * H_DUST)
    ok = rhs > 0
    r = np.full(len(s), np.inf)
    r[ok] = (-H_DUST * np.log(rhs[ok]) - z0) / s[ok]
    out[~flat] = r
    return out


def optical_depth(z0, sin_el, r, mu):
    k = mu * C0
    e0 = np.exp(-z0 / H_DUST)
    flat = np.abs(sin_el) < 1e-9
    s = np.where(flat, 1.0, sin_el)
    tau = k * H_DUST / s * (e0 - np.exp(-(z0 + r * s) / H_DUST))
    return np.where(flat, k * e0 * r, tau)


class VoxelMap:
    def __init__(self):
        self.lo = np.zeros(int(np.prod(GSHAPE)), dtype=np.float32)
        self.first_occ = np.full(int(np.prod(GSHAPE)), -1, dtype=np.int32)
        self.bb_lo = np.array(PSHAPE)                # planning-cell box that rays may have touched
        self.bb_hi = -np.ones(3, int)

    @staticmethod
    def flat(pts):
        ijk = np.floor((pts - GMIN) / VOX).astype(np.int64)
        ok = np.all((ijk >= 0) & (ijk < np.array(GSHAPE)), axis=1)
        ijk = ijk[ok]
        return (ijk[:, 0] * GSHAPE[1] + ijk[:, 1]) * GSHAPE[2] + ijk[:, 2]

    def update(self, origin, dirs_w, L_free, hit_pts, k=0):
        c = pcell(origin)
        r = int(np.ceil(R_LIDAR / PLAN_RES)) + 1
        self.bb_lo = np.minimum(self.bb_lo, c - r)
        self.bb_hi = np.maximum(self.bb_hi, c + r)
        n = np.maximum(np.floor((L_free - VOX) / VOX).astype(np.int64), 0)
        rep = np.repeat(np.arange(len(n)), n)
        steps = (np.arange(n.sum()) - np.repeat(np.cumsum(n) - n, n) + 1) * VOX
        fi = self.flat(origin[None, :] + steps[:, None] * dirs_w[rep])
        hi = self.flat(hit_pts)
        np.add.at(self.lo, fi, L_FREE)
        np.add.at(self.lo, hi, L_OCC)
        touched = np.unique(np.concatenate([fi, hi]))
        self.lo[touched] = np.clip(self.lo[touched], L_MIN, L_MAX)
        new = touched[(self.lo[touched] > LO_OCC_T) & (self.first_occ[touched] < 0)]
        self.first_occ[new] = k

    def classes(self):
        return self.lo < LO_FREE_T, self.lo > LO_OCC_T


def voxel_centres():
    ax = [GMIN[i] + (np.arange(GSHAPE[i]) + 0.5) * VOX for i in range(3)]
    return ax


def ground_truth():
    ax = voxel_centres()
    free = np.zeros(GSHAPE, bool)
    level = np.zeros(GSHAPE, np.int8)
    for (lo, hi), lv in zip(zip(BLO, BHI), LEVEL_OF_BOX):
        ix = [(ax[i] >= lo[i]) & (ax[i] <= hi[i]) for i in range(3)]
        m = ix[0][:, None, None] & ix[1][None, :, None] & ix[2][None, None, :]
        free |= m
        if lv > 0:
            level[m] = lv
    level[free & (level == 0)] = 4                   # shaft-only voxels
    wall = binary_dilation(free, structure=np.ones((3, 3, 3))) & ~free
    z = ax[2][None, None, :] * np.ones(GSHAPE)
    dust = C0 * np.exp(-z / H_DUST) > 0.5
    return dict(free_idx=np.flatnonzero(free), wall_idx=np.flatnonzero(wall),
                free_level=level.ravel()[np.flatnonzero(free)],
                wall_z=np.broadcast_to(ax[2][None, None, :], GSHAPE).ravel()[np.flatnonzero(wall)],
                dust_idx=np.flatnonzero((free | wall) & dust),
                plane_idx=np.flatnonzero(free & (np.abs(z - L1) < VOX / 2 + 1e-9)
                                         & (level == 1)))


def coverage(vm, gt):
    fr, oc = vm.lo[gt['free_idx']] < LO_FREE_T, vm.lo[gt['wall_idx']] > LO_OCC_T
    lv = gt['free_level']
    out = dict(xi=(fr.sum() + oc.sum()) / (len(fr) + len(oc)), free=fr.mean(), wall=oc.mean())
    for k, name in ((1, 'L1'), (2, 'L2'), (3, 'L3'), (4, 'shafts')):
        out[name] = fr[lv == k].mean()
    lod = vm.lo[gt['dust_idx']]
    out['dust_unknown'] = float(((lod >= LO_FREE_T) & (lod <= LO_OCC_T)).mean())
    lp = vm.lo[gt['plane_idx']]
    out['plane'] = float((lp < LO_FREE_T).mean())
    return out


# ── ICP (point-to-plane, x y z yaw, odometry prior) ───────────────────────────
def normals(pts, planar):
    tree = cKDTree(pts)
    d, nn = tree.query(pts, k=min(K_NORMAL, len(pts)), distance_upper_bound=NORMAL_RADIUS)
    ok = np.isfinite(d)
    nn = np.where(ok, nn, 0)
    w = ok.astype(float)
    cnt = w.sum(1)
    dim = 2 if planar else 3
    P = pts[nn][:, :, :dim]
    mu = (P * w[:, :, None]).sum(1) / np.maximum(cnt, 1)[:, None]
    Q = (P - mu[:, None, :]) * w[:, :, None]
    C = np.einsum('nki,nkj->nij', Q, Q)
    ev, v = np.linalg.eigh(C)
    nrm = np.zeros((len(pts), 3))
    nrm[:, :dim] = v[:, :, 0]
    if planar:
        good = cnt >= 4
    else:
        # a neighbourhood that is a curve (one LiDAR ring on the floor) has no defined
        # surface normal: require a 2D spread (planarity test on the eigenvalues)
        good = (cnt >= 5) & (ev[:, 1] > PLANAR_RATIO * ev[:, 2]) & (ev[:, 0] < 0.3 * ev[:, 1])
    return nrm, good, tree


def icp_p2pl(src, tgt_cache, tgt, x0, info0, k_max=K_ICP, lam=None):
    lam = LAM_TH if lam is None else lam
    nrm, nvalid, tree = tgt_cache
    x = x0.copy()
    prev = np.inf
    it = 0
    for it in range(1, k_max + 1):
        R = rotz(x[3])
        s = src @ R.T + x[:3]
        d, idx = tree.query(s)
        m = (d < D_REJECT) & nvalid[idx]
        if m.sum() < 10:
            break
        n = nrm[idx[m]]
        r = np.einsum('ij,ij->i', n, s[m] - tgt[idx[m]])
        ds = np.stack([-(s[m, 1] - x[1]), s[m, 0] - x[0], np.zeros(m.sum())], 1)
        J = np.column_stack([n, np.einsum('ij,ij->i', n, ds)])
        sig2 = 2 * SIGMA_R ** 2
        Hi, gi = J.T @ J / sig2, J.T @ r / sig2
        if lam > 0:
            # degeneracy-aware update: directions the scan barely constrains keep the prior
            w, V = np.linalg.eigh(Hi)
            keep = w >= lam
            Hi = (V * (w * keep)) @ V.T
            gi = V @ (keep * (V.T @ gi))
        H = Hi + info0
        g = gi + info0 @ (x - x0)
        x = x - np.linalg.solve(H, g)
        res = np.abs(r).mean()
        if abs(prev - res) < EPS_ICP:
            break
        prev = res
    return x, it


def prior_info(planar):
    s2 = SIGMA_ODO ** 2 * SCAN_STEPS
    zinfo = 1e8 if planar else 1 / s2              # altitude held exactly in the fixed modes
    return np.diag([1 / s2, 1 / s2, zinfo, 1 / (SIGMA_GYRO ** 2 * SCAN_STEPS)])


def rf_altitude(z_pred, rm, ret, ref, n_anchor):
    """Altitude from the up/down rangefinders against the surface heights they saw before.
    The same surface (within RF_GATE) gives an absolute altitude; a new surface (opening,
    shaft, other level) is re-anchored at the predicted altitude, which is where drift enters."""
    zs = []
    for j, sgn in ((0, 1.0), (1, -1.0)):          # 0: up (ceiling = z + r), 1: down (floor = z - r)
        if not ret[j]:
            continue
        surf = z_pred + sgn * rm[j]
        if ref[j] is not None and abs(surf - ref[j]) < RF_GATE:
            zs.append(ref[j] - sgn * rm[j])
        else:
            ref[j] = surf
            n_anchor += 1
    return (float(np.mean(zs)) if zs else z_pred), n_anchor


# ── Planning (0.4 m grid pooled from the 0.2 m map) ───────────────────────────
OFFS26 = [(i, j, k) for i in (-1, 0, 1) for j in (-1, 0, 1) for k in (-1, 0, 1) if (i, j, k) != (0, 0, 0)]


def pool2(a):
    a = a[0::2] + a[1::2]
    a = a[:, 0::2] + a[:, 1::2]
    return a[:, :, 0::2] + a[:, :, 1::2]


def dil(a, size=3):
    return maximum_filter(a.view(np.uint8), size=size, mode='constant', cval=0) > 0


def pcell(p):
    return np.clip(np.floor((p - GMIN) / PLAN_RES).astype(int), 0, np.array(PSHAPE) - 1)


def pcentre(ijk):
    return GMIN + (np.asarray(ijk) + 0.5) * PLAN_RES


AX_IDX = [a * N_EL for a in (0, 180, 90, 270)]   # beams toward +x, -x, +y, -y (lowest layer)


def recentre(trav, p, r, ret, gal_half, levels):
    """Shaft axis from the steepest LiDAR layer: in each of +x/-x and +y/-y, if both beams hit a
    wall beyond the current gallery's ceiling (climbing) or floor (descending), the axis lies
    midway between the two hits."""
    up = trav['zt'] > p[2]
    k = N_EL - 1 if up else 0
    lvl = min(levels, key=lambda z: abs(z - p[2]))
    edge = lvl + (gal_half if up else -gal_half)
    s, c = np.sin(EL[k]), np.cos(EL[k])
    for ax, (i, j) in enumerate(((AX_IDX[0], AX_IDX[1]), (AX_IDX[2], AX_IDX[3]))):
        i, j = i + k, j + k
        if ret[i] and ret[j]:
            zi, zj = p[2] + r[i] * s, p[2] + r[j] * s
            beyond = (zi > edge and zj > edge) if up else (zi < edge and zj < edge)
            if beyond:
                trav['goal'][ax] = p[ax] + 0.5 * (r[i] - r[j]) * c
                trav['centred'][ax] = True


def climb_guard(trav, gal_half, levels, z):
    """Height the drone may not pass before the axis has been re-centred from shaft-wall
    returns in both directions: GUARD_DZ inside the current gallery's ceiling (floor)."""
    if all(trav['centred']):
        return None
    up = trav['zt'] > z
    lvl = min(levels, key=lambda q: abs(q - z))
    return lvl + gal_half - GUARD_DZ if up else lvl - gal_half + GUARD_DZ


def shaft_centre(vm, p, sgn, gal_half, levels):
    """Centre of a shaft opening: centroid of the voxels mapped free within CENTRE_WIN of the
    drone (which is under the opening), in the first 0.4 m beyond the current gallery's
    ceiling (sgn = +1) or floor (sgn = -1)."""
    ax = voxel_centres()
    lvl = min(levels, key=lambda q: abs(q - p[2]))
    z_edge = lvl + sgn * gal_half                 # ceiling (floor) of the current gallery
    zlo, zhi = (z_edge, z_edge + 2 * VOX) if sgn > 0 else (z_edge - 2 * VOX, z_edge)
    ix = np.flatnonzero(np.abs(ax[0] - p[0]) <= CENTRE_WIN)
    iy = np.flatnonzero(np.abs(ax[1] - p[1]) <= CENTRE_WIN)
    iz = np.flatnonzero((ax[2] >= zlo) & (ax[2] <= zhi))
    if not (len(ix) and len(iy) and len(iz)):
        return p[0], p[1]
    L = vm.lo.reshape(GSHAPE)[ix[0]:ix[-1] + 1, iy[0]:iy[-1] + 1, iz[0]:iz[-1] + 1]
    f = np.argwhere(L < LO_FREE_T)
    if len(f) == 0:
        return p[0], p[1]
    return float(ax[0][ix[0] + f[:, 0]].mean()), float(ax[1][iy[0] + f[:, 1]].mean())


def plan(vm, p_est, layer, ignored, keep_goal=None, levels=None, shaft_boxes=None, target=None):
    """Dijkstra on the 0.4 m planning grid. Without a target: path to the nearest frontier goal
    (lateral frontiers at a known gallery level, vertical frontiers that open a candidate shaft
    box). With a target (shaft traverse): path to the reachable cell closest to it."""
    pad = GOAL_CELLS + 3
    lo_c = np.maximum(vm.bb_lo - pad, 0)
    hi_c = np.minimum(vm.bb_hi + pad + 1, np.array(PSHAPE))
    shp = hi_c - lo_c
    L = vm.lo.reshape(GSHAPE)[2 * lo_c[0]:2 * hi_c[0], 2 * lo_c[1]:2 * hi_c[1], 2 * lo_c[2]:2 * hi_c[2]]
    occ_v = (L > LO_OCC_T).view(np.uint8)
    fr = pool2((L < LO_FREE_T).view(np.uint8))
    occ = pool2(occ_v) >= 1
    free = (fr >= 2) & ~occ
    # clearance on the 0.2 m map: no sub-voxel of a traversable cell may touch the
    # (CLR_VOX)-voxel dilation of an occupied voxel -> cell centre >= 0.2 (CLR_VOX + 1) m
    # from every occupied voxel face
    hard = pool2(maximum_filter(occ_v, size=2 * CLR_VOX + 1, mode='constant', cval=0)) > 0
    soft = dil(occ, 5)
    zc = GMIN[2] + (np.arange(lo_c[2], hi_c[2]) + 0.5) * PLAN_RES
    zok = (zc >= Z_MIN) & (zc <= Z_MAX)
    trav = free & ~hard & zok[None, None, :]
    s = pcell(p_est) - lo_c
    if layer is not None:
        lay = np.zeros(shp[2], bool)
        lay[layer - lo_c[2]] = True
        trav &= lay[None, None, :]
    trav[s[0], s[1], s[2]] = True
    ti = np.argwhere(trav)
    nid = -np.ones(tuple(shp), np.int64)
    nid[ti[:, 0], ti[:, 1], ti[:, 2]] = np.arange(len(ti))
    rows, cols, wts = [], [], []
    for o in OFFS26:
        b = ti + np.array(o)
        ok = np.all((b >= 0) & (b < shp), axis=1)
        a, b = ti[ok], b[ok]
        ok2 = trav[b[:, 0], b[:, 1], b[:, 2]]
        rows.append(nid[a[ok2, 0], a[ok2, 1], a[ok2, 2]])
        cols.append(nid[b[ok2, 0], b[ok2, 1], b[ok2, 2]])
        pen = np.where(soft[b[ok2, 0], b[ok2, 1], b[ok2, 2]], SOFT_COST, 1.0)
        wts.append(np.linalg.norm(o) * PLAN_RES * pen)
    G = csr_matrix((np.concatenate(wts), (np.concatenate(rows), np.concatenate(cols))),
                   shape=(len(ti), len(ti)))
    start = nid[s[0], s[1], s[2]]
    dist, pred = dijkstra(G, indices=start, return_predecessors=True)
    if target is not None:
        reach = np.isfinite(dist)
        cw = pcentre(ti[reach] + lo_c)
        g = ti[reach][int(np.argmin(np.linalg.norm(cw - target, axis=1)))]
        n_big = 1
    else:
        ign = ignored[lo_c[0]:hi_c[0], lo_c[1]:hi_c[1], lo_c[2]:hi_c[2]]
        front = ~free & ~occ & dil(free) & ~ign
        if layer is not None:
            near = np.zeros(shp[2], bool)
            near[max(layer - 1 - lo_c[2], 0):layer + 2 - lo_c[2]] = True
            front &= near[None, None, :]
        st = np.ones((3, 3, 3), bool)
        lab, nl = label(front, structure=st)
        if nl == 0:
            return None, 0, None
        sizes = np.bincount(lab.ravel())
        big = np.flatnonzero(sizes >= MIN_FRONTIER)
        big = big[big > 0]
        if len(big) == 0:
            return None, 0, None
        if layer is None:
            # card: lateral frontiers (at a known gallery level) and vertical frontiers that
            # may be a shaft; floor / ceiling patches of a gallery are not pursued. Shaft boxes:
            # [x0, x1, y0, y1, state, z0, z1]; state 0 candidate, 1 confirmed (being traversed),
            # 2 traversed, -1 rejected (ceiling or floor patch), -2 traverse failed
            objs = find_objects(lab)
            keep = []
            in_box = lambda x, y, z: [bx for bx in shaft_boxes
                                      if bx[0] <= x <= bx[1] and bx[2] <= y <= bx[3] and bx[5] <= z <= bx[6]]
            for b in big:
                sl = objs[b - 1]
                idx = np.argwhere(lab[sl] == b) + np.array([q.start for q in sl])
                cen = pcentre(idx.mean(0) + lo_c)
                zc_ = pcentre(idx + lo_c)[:, 2]
                dzl = np.min(np.abs(zc_[:, None] - np.array(levels)[None, :]), axis=1)
                if dzl.min() < LAT_DZ:
                    keep.append(b)                    # lateral: part of a known gallery
                    continue
                boxes = in_box(*cen)
                if any(bx[4] in (2, -2) for bx in boxes):
                    continue                          # already traversed, or given up
                if any(bx[4] == -1 and np.hypot(cen[0] - 0.5 * (bx[0] + bx[1]), cen[1] - 0.5 * (bx[2] + bx[3]))
                       < REJECT_EXCL for bx in boxes):
                    continue                          # the patch whose centre was tested and rejected
                if dzl.max() >= SHAFT_DZ:
                    keep.append(b)
                    if not any(bx[4] in (0, 1) for bx in boxes):
                        lo_xy = pcentre(idx.min(0) + lo_c)[:2] - SHAFT_PAD
                        hi_xy = pcentre(idx.max(0) + lo_c)[:2] + SHAFT_PAD
                        shaft_boxes.append([lo_xy[0], hi_xy[0], lo_xy[1], hi_xy[1], 0,
                                            zc_.min() - SHAFT_ZPAD, zc_.max() + SHAFT_ZPAD])
            big = np.array(keep, int)
            if len(big) == 0:
                return None, 0, None
        n_big = len(big)
        cand_m = dil(np.isin(lab, big), 2 * GOAL_CELLS + 1) & trav
        cand = np.argwhere(cand_m)
        if len(cand) == 0:
            return None, n_big, None
        cd = dist[nid[cand[:, 0], cand[:, 1], cand[:, 2]]]
        if not np.isfinite(cd).any():
            return None, n_big, None
        g = cand[int(np.argmin(cd))]
        if keep_goal is not None:
            kg = np.all(cand == keep_goal - lo_c, axis=1)
            if kg.any() and np.isfinite(cd[kg][0]):
                g = keep_goal - lo_c
    node = nid[g[0], g[1], g[2]]
    path = []
    while node != start and node >= 0:
        path.append(ti[node])
        node = pred[node]
    path = np.array(path[::-1]) if path else np.array([s])
    return pcentre(path + lo_c), n_big, np.array(g) + lo_c


# ── Closed-loop mission ───────────────────────────────────────────────────────
def mission(mode, seed=SEED, mu=MU_EXT, record=False, pose='true', t_max=T_TIMEOUT):
    """mode: '3d' (shafts allowed, up/down rangefinders), '3d_fixed' (16 layers, z held),
    '2d' (base sensor, z held). pose: 'true' maps and navigates with the true pose (the
    estimators are evaluated by replay); 'est' closes the loop with the ICP estimate."""
    t_wall = time.time()
    rng = np.random.default_rng(seed)
    planar = mode == '2d'
    dirs = DIRS2 if planar else DIRS3
    sin_el = dirs[:, 2]
    icp_mask = np.ones(len(dirs), bool) if planar else (np.arange(len(dirs)) // N_EL) % ICP_SUB == 0
    vertical = mode == '3d'
    info0 = prior_info(not vertical)
    layer = None if vertical else int(pcell(START)[2])
    vm = VoxelMap()
    gt = GT
    p_true = START.copy()
    p_est, th_est = START.copy(), 0.0
    scan_pose_est, scan_th = p_est.copy(), 0.0
    disp_cmd, gyro_acc = np.zeros(3), 0.0
    prev = None
    tt, te, modes, scan_k, scans, cov_log, snaps = [p_true.copy()], [p_est.copy()], [], [], [], [], []
    icp_it, tau_max = [], 0.0
    lost_dust = 0
    n_beams = 0
    path, goal, k_plan = None, None, 0
    ignored = np.zeros(PSHAPE, bool)
    visits, last_visit = {}, {}
    levels, shaft_boxes = [START[2]], []
    rf_ref, n_anchor = [None, None], 0
    gal_half = None
    traverse, n_traverse, n_trav_fail = None, 0, 0
    status, k, t = 'timeout', 0, 0.0
    min_clear, vz_max_seen = np.inf, 0.0
    r_meas, ok_ret = None, None
    while t < t_max - 1e-9:
        if k % SCAN_STEPS == 0:
            r_true = cast3d(p_true, dirs)
            reach = dust_reach(p_true[2], sin_el, mu)
            ret = (r_true < R_LIDAR) & (r_true < reach)
            tau_max = max(tau_max, float(optical_depth(p_true[2], sin_el, np.minimum(r_true, R_LIDAR), mu).max()))
            lost_dust += int(((r_true < R_LIDAR) & ~ret).sum())
            n_beams += len(dirs)
            r_meas = np.clip(r_true + rng.normal(0, SIGMA_R, len(dirs)), 0.05, R_LIDAR)
            L_free = np.where(ret, r_meas, np.minimum(R_LIDAR, reach))
            body = r_meas[:, None] * dirs
            pts = body[ret & icp_mask]
            if mode == '3d':                          # up/down single-beam rangefinders
                r_rf = cast3d(p_true, RF_DIRS)
                reach_rf = dust_reach(p_true[2], RF_DIRS[:, 2], mu)
                ret_rf = (r_rf < R_RF) & (r_rf < reach_rf)
                rm_rf = np.clip(r_rf + rng.normal(0, SIGMA_R, 2), 0.05, R_RF)
            if pose == 'est' and prev is not None:
                x0 = np.array([*(rotz(-scan_th) @ disp_cmd), gyro_acc])
                x, it = icp_p2pl(pts, prev[1], prev[0], x0, info0)
                icp_it.append(it)
                p_est = scan_pose_est + rotz(scan_th) @ x[:3]
                th_est = scan_th + x[3]
                if not vertical:
                    p_est[2] = START[2]
            if mode == '3d' and pose == 'est':
                p_est[2], n_anchor = rf_altitude(p_est[2], rm_rf, ret_rf, rf_ref, n_anchor)
            if pose == 'true':
                p_est, th_est = p_true.copy(), 0.0
            scan_pose_est, scan_th = p_est.copy(), th_est
            disp_cmd, gyro_acc = np.zeros(3), 0.0
            if pose == 'est':
                prev = (pts, normals(pts, planar))
            Rw = rotz(th_est)
            dw = dirs @ Rw.T
            hits = p_est[None, :] + body[ret] @ Rw.T
            if mode == '3d':
                if gal_half is None and ret_rf.all():
                    gal_half = 0.5 * (rm_rf[0] + rm_rf[1])   # gallery half-height, measured at start
                far = np.where(ret_rf, rm_rf, R_RF)
                for bx in shaft_boxes:                # test candidate shafts with the rangefinders
                    if (traverse is None and bx[4] == 0 and bx[0] + SHAFT_PAD <= p_est[0] <= bx[1] - SHAFT_PAD
                            and bx[2] + SHAFT_PAD <= p_est[1] <= bx[3] - SHAFT_PAD
                            and bx[5] <= p_est[2] <= bx[6]):
                        if far.max() > 2 * gal_half:   # free space beyond a full gallery height
                            j = int(np.argmax(far))
                            sgn = 1.0 if j == 0 else -1.0
                            zt = p_est[2] + sgn * (far[j] - gal_half)   # centre of the gallery beyond
                            if min(abs(zt - z) for z in levels) > SHAFT_DZ:
                                xc, yc = shaft_centre(vm, p_est, sgn, gal_half, levels)
                                bx[4] = 1
                                traverse = dict(box=bx, goal=np.array([xc, yc, p_est[2]]), t0=t, zt=zt,
                                                phase='align')
                                n_traverse += 1
                            else:
                                bx[4] = 2             # leads to a known level
                        elif (np.hypot(p_est[0] - 0.5 * (bx[0] + bx[1]), p_est[1] - 0.5 * (bx[2] + bx[3])) < REJECT_R
                              and ret_rf.all()):
                            bx[4] = -1                # ceiling and floor right at the box centre
                if traverse is not None and traverse['phase'] == 'climb':
                    recentre(traverse, p_est, r_meas, ret, gal_half, levels)
                dw = np.vstack([dw, RF_DIRS])
                L_free = np.concatenate([L_free, np.where(ret_rf, rm_rf, np.minimum(R_RF, reach_rf))])
                hits = np.vstack([hits, p_est[None, :] + rm_rf[ret_rf, None] * RF_DIRS[ret_rf]])
            vm.update(p_est, dw, L_free, hits, k)
            scan_k.append(k)
            if record and mode == '3d':
                scans.append(np.concatenate([r_true, r_rf]).astype(np.float32))
            ok_ret = ret
            if k == 0:
                snaps.append(0)
        # planning
        if traverse is not None:
            if (traverse['phase'] == 'align' and path is not None
                    and np.linalg.norm(path[-1][:2] - p_est[:2]) < ALIGN_TOL
                    and np.linalg.norm(path[-1][:2] - traverse['goal'][:2]) < ALIGN_MAX):
                traverse['phase'] = 'climb'           # centred under / over the opening: go vertical
                traverse['goal'] = np.array([path[-1][0], path[-1][1], traverse['zt']])
                traverse['centred'] = [False, False]
                path = None
            elif traverse['phase'] == 'climb' and abs(traverse['zt'] - p_est[2]) < TRAV_TOL:
                levels.append(float(traverse['zt']))  # arrived in the gallery beyond the shaft
                traverse['box'][4] = 2
                traverse, path = None, None
            elif t - traverse['t0'] > T_TRAV_MAX:
                traverse['box'][4] = -2               # give up this shaft
                traverse, path = None, None
                n_trav_fail += 1
        reached = path is not None and np.linalg.norm(path[-1] - p_est) < STOP_DIST
        if reached and traverse is None:
            # lateral frontier cells near a reached goal are given up (as in 2D); vertical
            # ones (in other planning layers) stay, because climbing reveals them,
            # unless the same goal cell has been reached MAX_VISITS times
            gc = pcell(path[-1])
            key = tuple(int(v) for v in gc)
            if k - last_visit.get(key, -10 ** 9) >= VISIT_GAP:   # one visit = a new reach after
                visits[key] = visits.get(key, 0) + 1          # at least VISIT_GAP steps of scans
                last_visit[key] = k
            r = int(np.ceil(IGNORE_R / PLAN_RES))
            rz = r if (visits[key] >= MAX_VISITS or not vertical) else 0
            sl = (slice(max(gc[0] - r, 0), gc[0] + r + 1), slice(max(gc[1] - r, 0), gc[1] + r + 1),
                  slice(max(gc[2] - rz, 0), gc[2] + rz + 1))
            ignored[sl] = True
        climbing = traverse is not None and traverse['phase'] == 'climb'
        if not climbing and (path is None or (reached and k - k_plan >= SCAN_STEPS) or k - k_plan >= REPLAN_EVERY):
            path, n_front, goal = plan(vm, p_est, layer, ignored, None if reached else goal,
                                       levels, shaft_boxes, None if traverse is None else traverse['goal'])
            k_plan = k
            if path is None:
                status = 'no frontier left' if n_front == 0 else 'frontiers unreachable'
                break
        if climbing:
            # card's shaft-traverse mode: hold the shaft axis (re-centred from the shaft-wall
            # returns), and move vertically only while within CLIMB_HTOL of it
            herr = traverse['goal'][:2] - p_est[:2]
            v = np.zeros(3)
            v[:2] = herr * min(1.0, CLIMB_VXY / max(np.linalg.norm(herr), 1e-9))
            if np.linalg.norm(herr) < CLIMB_HTOL:
                z_to = traverse['zt']
                zg = climb_guard(traverse, gal_half, levels, p_est[2])
                if zg is not None:                    # not centred yet: stop short of the rim
                    z_to = min(z_to, zg) if z_to > p_est[2] else max(z_to, zg)
                v[2] = float(np.clip(z_to - p_est[2], -VZ_MAX, VZ_MAX))
            modes.append('shaft')
            path = None
        else:
            v = None
        if v is None:
            dpath = np.linalg.norm(path - p_est, axis=1)
            j = int(np.argmin(dpath))
            ahead = path[j:]
            far = np.flatnonzero(np.linalg.norm(ahead - p_est, axis=1) >= CARROT)
            carrot = ahead[far[0]] if len(far) else path[-1]
            d = carrot - p_est
            if not vertical:
                d[2] = 0.0
            if np.linalg.norm(path[-1] - p_est) < (TRAV_STOP if traverse is not None else STOP_DIST) \
                    or np.linalg.norm(d) < 1e-9:
                v = np.zeros(3)
            else:
                v = V_NAV * d / np.linalg.norm(d)
                if abs(v[2]) > VZ_MAX:
                    v *= VZ_MAX / abs(v[2])
            vert = abs(d[2]) > np.linalg.norm(d[:2])
            modes.append('shaft' if vert else 'cruise')
        # reactive safety filter on raw returns (body frame) + vertical rangefinders
        vb = rotz(-th_est) @ v
        close = ok_ret & (r_meas < SAFE_R)
        us = list(zip(dirs[close], r_meas[close]))
        if vertical:
            for u in RF_DIRS:
                rv = float(cast3d(p_true, u[None, :])[0])
                if rv < SAFE_R:
                    us.append((u, rv))
        for u, r in us:
            tw = vb @ u
            if tw > 0:
                vb = vb - tw * u
            vb = vb - K_REPEL * (SAFE_R - r) * u
        sp = np.linalg.norm(vb[:2])
        if sp > V_NAV:
            vb[:2] *= V_NAV / sp
        if abs(vb[2]) > VZ_MAX:
            vb[2] = np.sign(vb[2]) * VZ_MAX
        if not vertical:
            vb[2] = 0.0
        w = rng.normal(0, SIGMA_ODO, 3)
        if not vertical:
            w[2] = 0.0                                # ideal altitude hold (as the 2D base)
        g = rng.normal(0, SIGMA_GYRO)
        step = vb * DT + w                            # true yaw is 0: body = world
        p_true = p_true + step
        disp_cmd = disp_cmd + rotz(th_est) @ vb * DT
        gyro_acc += g
        p_est = p_est + rotz(th_est) @ vb * DT        # dead reckoning until the next scan
        if pose == 'true':
            p_est = p_true.copy()
        assert np.linalg.norm(vb[:2]) <= V_MAX + 1e-9 and abs(vb[2]) <= VZ_MAX + 1e-9, 'speed limit'
        assert inside(p_true)[0], 'drone left the mine'
        c = clearance(p_true)
        min_clear = min(min_clear, c)
        if pose == 'est' and c < R_DRONE:             # drift-driven wall contact ends the probe
            status = f'wall contact (clearance {c:.2f} m)'
            t += DT
            k += 1
            tt.append(p_true.copy())
            te.append(p_est.copy())
            break
        assert c >= R_DRONE, f'clearance {c:.2f} m at {np.round(p_true, 2)}, t = {t:.1f} s, traverse {traverse is not None}'
        assert Z_MIN - 1e-9 <= p_true[2] <= Z_MAX + 1e-9, 'altitude bounds'
        vz_max_seen = max(vz_max_seen, abs(vb[2]))
        t += DT
        k += 1
        tt.append(p_true.copy())
        te.append(p_est.copy())
        if k % 50 == 0:
            cov_log.append((t, coverage(vm, gt)))
        if k % SNAP_EVERY == 0:
            snaps.append(k)
    cov_log.append((t, coverage(vm, gt)))
    if snaps[-1] != k:
        snaps.append(k)
    tt, te = np.array(tt), np.array(te)
    return dict(mode=mode, vm=vm, tt=tt, te=te, modes=np.array(modes), scan_k=np.array(scan_k),
                scans=scans, cov_log=cov_log, snaps=snaps, t=t, status=status,
                icp_it=np.array(icp_it), tau_max=tau_max, lost_dust=lost_dust, n_beams=n_beams,
                min_clear=min_clear, vz_max=vz_max_seen, wall=time.time() - t_wall,
                ignored=ignored, visits=visits, levels=levels, shaft_boxes=shaft_boxes, n_anchor=n_anchor,
                n_traverse=n_traverse, n_trav_fail=n_trav_fail)


# ── Replays on the flown 3D trajectory ────────────────────────────────────────
def in_shaft(p):
    p = np.atleast_2d(p)
    lv = np.zeros(len(p), bool)
    sh = np.zeros(len(p), bool)
    for (lo, hi), l in zip(zip(BLO, BHI), LEVEL_OF_BOX):
        m = np.all((p >= lo) & (p <= hi), axis=1)
        if l == 0:
            sh |= m
        else:
            lv |= m
    return sh & ~lv


def _rms(a):
    return float(np.sqrt(np.mean(np.square(a)))) if len(a) else float('nan')


def replay(m, seed, n_max=None, lam=None):
    """Fresh noise on the flown 3D trajectory; all estimators share it (CRN).
    dr: odometry + gyro only; icp: point-to-plane ICP with the odometry prior and the
    rangefinder altitude reference (the closed-loop estimator)."""
    rng = np.random.default_rng(seed)
    sk = m['scan_k'] if n_max is None else m['scan_k'][:n_max + 1]
    P = m['tt'][sk]
    true_d = np.diff(P, axis=0)
    prior = true_d + rng.normal(0, SIGMA_ODO * np.sqrt(SCAN_STEPS), true_d.shape)
    gyro = rng.normal(0, SIGMA_GYRO * np.sqrt(SCAN_STEPS), len(true_d))
    icp_mask = (np.arange(len(DIRS3)) // N_EL) % ICP_SUB == 0
    pts_all, rf_all = [], []
    for r_all in m['scans'][:len(P)]:
        r_true = r_all[:len(DIRS3)].astype(float)
        ret = r_true < R_LIDAR
        r = np.clip(r_true + rng.normal(0, SIGMA_R, len(r_true)), 0.05, R_LIDAR)
        pts_all.append((r[:, None] * DIRS3)[ret & icp_mask])
        r_rf = r_all[len(DIRS3):].astype(float)
        rf_all.append((np.clip(r_rf + rng.normal(0, SIGMA_R, 2), 0.05, R_RF), r_rf < R_RF))
    info0 = prior_info(False)
    est = {'dr': [P[0].copy()], 'icp': [P[0].copy()]}
    th = {'dr': 0.0, 'icp': 0.0}
    inc_err = {'dr': [], 'icp': []}
    icp_dz_err = []
    ref, n_anchor = [None, None], 0
    _, n_anchor = rf_altitude(P[0][2], rf_all[0][0], rf_all[0][1], ref, n_anchor)
    cache = normals(pts_all[0], False)
    for i in range(1, len(P)):
        x0 = np.array([*(rotz(-th['icp']) @ prior[i - 1]), gyro[i - 1]])
        x, _ = icp_p2pl(pts_all[i], cache, pts_all[i - 1], x0, info0, lam=lam)
        cache = normals(pts_all[i], False)
        icp_dz_err.append(x[2] - true_d[i - 1, 2])           # scan-only vertical increment error
        for key, (dx, dth) in (('dr', (rotz(-th['dr']) @ prior[i - 1], gyro[i - 1])),
                               ('icp', (x[:3], x[3]))):
            p_new = est[key][-1] + rotz(th[key]) @ dx
            if key == 'icp':
                p_new[2], n_anchor = rf_altitude(p_new[2], rf_all[i][0], rf_all[i][1], ref, n_anchor)
            inc_err[key].append(p_new - est[key][-1] - true_d[i - 1])
            est[key].append(p_new)
            th[key] += dth
    sh = in_shaft(0.5 * (P[1:] + P[:-1]))
    shp_ = in_shaft(P)
    out = {}
    for key in ('dr', 'icp'):
        e = np.array(est[key]) - P
        ie = np.array(inc_err[key])
        out[key] = dict(est=np.array(est[key]), rmse=float(np.sqrt(np.mean(np.sum(e ** 2, 1)))),
                        ez_shaft=_rms(e[shp_, 2]),
                        ez_gal=_rms(e[~shp_, 2]),
                        rmse_xy=float(np.sqrt(np.mean(np.sum(e[:, :2] ** 2, 1)))),
                        rmse_z=_rms(e[:, 2]), final=float(np.linalg.norm(e[-1])),
                        final_z=float(abs(e[-1, 2])),
                        inc_z_shaft=_rms(ie[sh, 2]),
                        inc_z_gal=_rms(ie[~sh, 2]),
                        inc_xy_shaft=_rms(np.linalg.norm(ie[sh, :2], axis=1)),
                        inc_xy_gal=_rms(np.linalg.norm(ie[~sh, :2], axis=1)))
    dz = np.array(icp_dz_err)
    out['icp_scan_dz_shaft'] = _rms(dz[sh])
    out['icp_scan_dz_gal'] = _rms(dz[~sh])
    out['n_anchor'] = n_anchor
    out['n_shaft'] = int(sh.sum())
    out['n'] = len(sh)
    return out


def remap(m, mu, poses=None, low_only=False):
    """Rebuild the 3D map from the stored scans with another dust level and/or other poses.
    low_only: only the scans taken below z = 4 m (all the dust-zone voxels lie in Level 1)."""
    rng = np.random.default_rng(777)
    vm = VoxelMap()
    sk = m['scan_k']
    lost, n = 0, 0
    rmax = np.r_[np.full(len(DIRS3), R_LIDAR), np.full(2, R_RF)]
    for i, r_all in enumerate(m['scans']):
        p_true = m['tt'][sk[i]]
        if low_only and p_true[2] > 4.0:
            continue
        p_map = p_true if poses is None else poses[i]
        r_true = r_all.astype(float)
        reach = dust_reach(p_true[2], DIRS_ALL[:, 2], mu)
        ret = (r_true < rmax) & (r_true < reach)
        lost += int(((r_true[:len(DIRS3)] < R_LIDAR) & ~ret[:len(DIRS3)]).sum())
        n += len(DIRS3)
        r = np.clip(r_true + rng.normal(0, SIGMA_R, len(r_true)), 0.05, rmax)
        L_free = np.where(ret, r, np.minimum(rmax, reach))
        vm.update(p_map, DIRS_ALL, L_free, p_map[None, :] + (r[:, None] * DIRS_ALL)[ret])
    c = coverage(vm, GT)
    c['lost'] = lost / n
    return c


GT = None


def run_simulation():
    global GT
    t0 = time.time()
    GT = ground_truth()
    res = dict(missions={}, gt=GT)
    for mode in ('3d', '3d_fixed', '2d'):
        res['missions'][mode] = mission(mode, record=(mode == '3d'))
        print(f"  [{mode}] mission done in {res['missions'][mode]['wall']:.0f} s", file=sys.stderr)
    res['probe_est'] = mission('3d', pose='est', t_max=T_PROBE)
    print(f"  closed-loop estimate probe done in {res['probe_est']['wall']:.0f} s", file=sys.stderr)
    m3 = res['missions']['3d']
    res['replays'] = {s: replay(m3, s) for s in REPLAY_SEEDS}
    print(f'  replays done at {time.time() - t0:.0f} s', file=sys.stderr)
    res['lam_sweep'] = {lam: replay(m3, REPLAY_SEEDS[0], n_max=LAM_N, lam=lam) for lam in LAM_SWEEP}
    res['map_est'] = remap(m3, MU_EXT, poses=res['replays'][REPLAY_SEEDS[0]]['icp']['est'])
    res['dust'] = {mu: remap(m3, mu * MU_EXT, low_only=True) for mu in MU_SWEEP}
    res['runtime'] = time.time() - t0
    return res


# ── Plots ─────────────────────────────────────────────────────────────────────
def _box_edges(ax, color='0.6', lw=0.4):
    for lo, hi in zip(BLO, BHI):
        x = [lo[0], hi[0]]
        y = [lo[1], hi[1]]
        z = [lo[2], hi[2]]
        for a in x:
            for b in y:
                ax.plot([a, a], [b, b], z, color=color, lw=lw)
        for a in x:
            for c in z:
                ax.plot([a, a], y, [c, c], color=color, lw=lw)
        for b in y:
            for c in z:
                ax.plot(x, [b, b], [c, c], color=color, lw=lw)


def _occ_points(vm, upto=None, step=5):
    fo = vm.first_occ
    idx = np.flatnonzero(fo >= 0) if upto is None else np.flatnonzero((fo >= 0) & (fo <= upto))
    idx = idx[::step]
    k = idx % GSHAPE[2]
    j = (idx // GSHAPE[2]) % GSHAPE[1]
    i = idx // (GSHAPE[1] * GSHAPE[2])
    return GMIN + (np.stack([i, j, k], 1) + 0.5) * VOX


def plot_occupancy_3d(res, out_dir):
    m = res['missions']['3d']
    fig = plt.figure(figsize=(13, 8))
    ax = fig.add_subplot(111, projection='3d')
    P = _occ_points(m['vm'], step=10)
    ax.scatter(P[:, 0], P[:, 1], P[:, 2], s=0.2, c='grey', alpha=0.25)
    tt = m['tt']
    te = res['replays'][REPLAY_SEEDS[0]]['icp']['est']
    sc = ax.scatter(tt[::5, 0], tt[::5, 1], tt[::5, 2], c=tt[::5, 2], cmap='viridis', s=2,
                    label='ground truth (colour = z)')
    ax.plot(te[:, 0], te[:, 1], te[:, 2], 'r--', lw=0.8, label=f'ICP estimate (replay seed {REPLAY_SEEDS[0]})')
    t2 = res['missions']['2d']['tt']
    ax.plot(t2[:, 0], t2[:, 1], t2[:, 2], color='tab:blue', lw=0.6, alpha=0.7, label='2D baseline path (z = 1.5 m)')
    ax.scatter([], [], [], c='grey', s=5, label='occupied voxels (3D mission)')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_zlabel('z (m)')
    ax.set_box_aspect((83, 44, 26))
    ax.view_init(elev=24, azim=-58)
    plt.colorbar(sc, ax=ax, shrink=0.5, pad=0.1, label='altitude of the true path (m)')
    ax.legend(loc='upper left', fontsize=8)
    ax.set_title(f"3D mission: occupancy map and trajectories (T_exp = {m['t']:.0f} s)")
    path = os.path.join(out_dir, 'occupancy_3d.png')
    plt.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_xz_section(res, out_dir):
    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    for ax, mode in zip(axes, ('3d', '3d_fixed', '2d')):
        m = res['missions'][mode]
        lo = m['vm'].lo.reshape(GSHAPE)
        fr = (lo < LO_FREE_T).any(axis=1)
        oc = (lo > LO_OCC_T).any(axis=1)
        img = np.full(fr.shape, 0.5)
        img[fr] = 0.0
        img[oc & ~fr] = 1.0
        ax.imshow(img.T, origin='lower', cmap='RdYlGn_r', vmin=0, vmax=1, aspect='auto',
                  extent=[GMIN[0], GMIN[0] + GSHAPE[0] * VOX, GMIN[2], GMIN[2] + GSHAPE[2] * VOX])
        for lo_, hi_ in zip(BLO, BHI):
            ax.add_patch(plt.Rectangle((lo_[0], lo_[2]), hi_[0] - lo_[0], hi_[2] - lo_[2], fill=False,
                                       ec='k', lw=0.5, ls=':'))
        ax.plot(m['tt'][:, 0], m['tt'][:, 2], color='tab:blue', lw=0.7)
        c = m['cov_log'][-1][1]
        ax.set_title(f"{mode}: projection on x-z (green = free in any y, red = occupied only, yellow = unknown); "
                     f"xi = {100 * c['xi']:.1f} %", fontsize=9)
        ax.set_ylabel('z (m)')
    axes[-1].set_xlabel('x (m)')
    path = os.path.join(out_dir, 'xz_section.png')
    plt.tight_layout()
    plt.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_altitude(res, out_dir):
    m = res['missions']['3d']
    rp = res['replays'][REPLAY_SEEDS[0]]
    t = np.arange(len(m['tt'])) * DT
    ts = m['scan_k'] * DT
    fig, axes = plt.subplots(2, 1, figsize=(12, 6.5), sharex=True)
    ax = axes[0]
    ax.plot(t, m['tt'][:, 2], 'k-', lw=1.2, label='z true')
    ax.plot(ts, rp['icp']['est'][:, 2], 'r--', lw=1, label=f'z estimated, ICP + RF (replay seed {REPLAY_SEEDS[0]})')
    sh = np.array([mm == 'shaft' for mm in m['modes']])
    ax.fill_between(t[1:], 0, 13, where=sh, color='gold', alpha=0.3, label='shaft-traverse mode')
    for z in Z_LEVELS:
        ax.axhline(z, color='grey', ls=':', lw=0.7)
    ax.set_ylabel('z (m)')
    ax.legend(fontsize=8, loc='upper left')
    ax.set_title('Altitude time series (3D mission)')
    ax = axes[1]
    for key, c in (('dr', 'tab:orange'), ('icp', 'tab:red')):
        ax.plot(ts, rp[key]['est'][:, 2] - m['tt'][m['scan_k'], 2], color=c,
                label=f'z error, {"dead reckoning" if key == "dr" else "ICP + prior + RF altitude"} (replay seed {REPLAY_SEEDS[0]})')
    shs = in_shaft(m['tt'][m['scan_k']])
    ax.fill_between(ts, -2, 2, where=shs, color='gold', alpha=0.3, label='inside a shaft')
    ax.set_ylim(-2.0, 2.0)
    ax.axhline(0, color='k', lw=0.5)
    ax.set_xlabel('t (s)')
    ax.set_ylabel('z error (m)')
    ax.legend(fontsize=7, loc='lower left')
    path = os.path.join(out_dir, 'altitude_timeseries.png')
    plt.tight_layout()
    plt.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_coverage(res, out_dir):
    fig, axes = plt.subplots(1, 2, figsize=(14, 4.8))
    m = res['missions']['3d']
    t = [c[0] for c in m['cov_log']]
    for key, c in (('L1', 'tab:blue'), ('L2', 'tab:green'), ('L3', 'tab:orange'), ('shafts', 'goldenrod'),
                   ('free', 'k')):
        axes[0].plot(t, [100 * x[1][key] for x in m['cov_log']], color=c,
                     lw=2 if key == 'free' else 1.2, label=('all free voxels' if key == 'free' else key))
    axes[0].set_xlabel('t (s)')
    axes[0].set_ylabel('free voxels mapped free (%)')
    axes[0].set_title('3D mission: coverage per level')
    axes[0].legend(fontsize=8)
    axes[0].grid(alpha=0.3)
    for mode, c in (('3d', 'tab:red'), ('3d_fixed', 'tab:purple'), ('2d', 'tab:blue')):
        mm = res['missions'][mode]
        axes[1].plot([x[0] for x in mm['cov_log']], [100 * x[1]['xi'] for x in mm['cov_log']], color=c,
                     label=mode)
    axes[1].set_xlabel('t (s)')
    axes[1].set_ylabel('volumetric coverage xi (%)')
    axes[1].set_title('Volumetric coverage, three missions')
    axes[1].legend(fontsize=8)
    axes[1].grid(alpha=0.3)
    path = os.path.join(out_dir, 'coverage_vs_time.png')
    plt.tight_layout()
    plt.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_dust(res, out_dir):
    mus = MU_SWEEP
    fig, ax = plt.subplots(figsize=(9, 4.8))
    x = np.arange(len(mus))
    du = [100 * res['dust'][u]['dust_unknown'] for u in mus]
    l1 = [100 * res['dust'][u]['L1'] for u in mus]
    lo = [100 * res['dust'][u]['lost'] for u in mus]
    ax.bar(x - 0.27, du, 0.27, color='tab:brown', label='dust-zone voxels unknown (deficit)')
    ax.bar(x, l1, 0.27, color='tab:blue', label='Level 1 free voxels mapped free')
    ax.bar(x + 0.27, lo, 0.27, color='0.5', label='beams lost to dust')
    for i in range(len(mus)):
        ax.text(x[i] - 0.27, du[i] + 1, f'{du[i]:.1f}', ha='center', fontsize=7)
        ax.text(x[i], l1[i] + 1, f'{l1[i]:.1f}', ha='center', fontsize=7)
        ax.text(x[i] + 0.27, lo[i] + 1, f'{lo[i]:.1f}', ha='center', fontsize=7)
    ax.set_xticks(x)
    ax.set_xticklabels([f'mu x{u:g}\n({u * MU_EXT:.2f})' for u in mus])
    ax.set_ylabel('%')
    ax.set_ylim(0, 125)
    ax.set_title('Dust: Level 1 scans of the 3D mission rebuilt with stronger extinction')
    ax.legend(fontsize=8, loc='upper center', ncol=3)
    path = os.path.join(out_dir, 'dust_deficit.png')
    plt.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def save_animation(res, out_dir):
    m = res['missions']['3d']
    tt = m['tt']
    snaps = m['snaps']
    fig = plt.figure(figsize=(8, 5.5))
    ax = fig.add_subplot(111, projection='3d')
    _box_edges(ax, color='0.75', lw=0.3)
    sc = ax.scatter([], [], [], s=0.3, c='grey', alpha=0.3)
    line, = ax.plot([], [], [], 'k-', lw=0.8)
    dot, = ax.plot([], [], [], 'o', ms=6, mec='k')
    ax.set_xlim(GMIN[0], 82)
    ax.set_ylim(-21, 22)
    ax.set_zlim(0, 13)
    ax.set_box_aspect((83, 43, 26))
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    fo = m['vm'].first_occ
    occ_idx = np.flatnonzero(fo >= 0)[::16]
    kk = occ_idx % GSHAPE[2]
    jj = (occ_idx // GSHAPE[2]) % GSHAPE[1]
    ii = occ_idx // (GSHAPE[1] * GSHAPE[2])
    P = GMIN + (np.stack([ii, jj, kk], 1) + 0.5) * VOX
    first = fo[occ_idx]

    def update(f):
        k = snaps[f]
        sel = first <= k
        sc._offsets3d = (P[sel, 0], P[sel, 1], P[sel, 2])
        line.set_data(tt[:k + 1, 0], tt[:k + 1, 1])
        line.set_3d_properties(tt[:k + 1, 2])
        mode = m['modes'][min(k, len(m['modes']) - 1)]
        dot.set_data([tt[k, 0]], [tt[k, 1]])
        dot.set_3d_properties([tt[k, 2]])
        dot.set_color('gold' if mode == 'shaft' else 'tab:blue')
        ax.view_init(elev=25, azim=-60 + 40 * f / max(len(snaps) - 1, 1))
        ax.set_title(f't = {k * DT:5.1f} s | z = {tt[k, 2]:.1f} m | mode {mode}', fontsize=9)
        return [sc, line, dot]

    anim = animation.FuncAnimation(fig, update, frames=len(snaps), interval=100, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    anim.save(path, writer='pillow', fps=10, dpi=65)
    plt.close(fig)
    print(f'Saved: {path} ({len(snaps)} frames)')


# ── Report ────────────────────────────────────────────────────────────────────
def report(res):
    gt = res['gt']
    print('\n=== S074 3D Mine Mapping — Results ===')
    vol = sum(np.prod(hi - lo) for lo, hi in zip(BLO, BHI))
    print(f'  Mine: {len(BOXES)} boxes (box volume sum {vol:.0f} m^3), ground truth {len(gt["free_idx"])} free '
          f'voxels, {len(gt["wall_idx"])} wall voxels; free voxels L1/L2/L3/shafts: '
          + '/'.join(str(int((gt['free_level'] == k).sum())) for k in (1, 2, 3, 4)))
    print(f'  LiDAR geometry: from the tunnel centre (1.5 m above the floor) the lowest beam (-15 deg) reaches the '
          f'floor {HALF_W / np.tan(EL_MAX):.2f} m away; on a shaft wall 1 m away the highest beam is '
          f'{SHAFT_R * np.tan(EL_MAX):.3f} m above the drone')
    print(f'  Dust: c > 0.5 below z = {H_DUST * np.log(C0 / 0.5):.3f} m ({len(gt["dust_idx"])} voxels); a beam is lost '
          f'at optical depth ln(1/{I_THRESH}) = {np.log(1 / I_THRESH):.3f}; the largest possible depth over 10 m '
          f'(at the floor, c0 = {C0}) is {MU_EXT * C0 * R_LIDAR:.3f}; card r_eff formula at z = 1.5 m: '
          f'{R_LIDAR * np.exp(-MU_EXT * C0 * np.exp(-L1 / H_DUST) * R_LIDAR):.2f} m')
    for mode in ('3d', '3d_fixed', '2d'):
        m = res['missions'][mode]
        c = m['cov_log'][-1][1]
        path_len = np.sum(np.linalg.norm(np.diff(m['tt'], axis=0), axis=1))
        n_sh = int((m['modes'] == 'shaft').sum())
        print(f'  [{mode} mission, true pose] ended: {m["status"]} at T_exp = {m["t"]:.1f} s, path {path_len:.1f} m, '
              f'shaft-traverse steps {n_sh} ({100 * n_sh / max(len(m["modes"]), 1):.1f}%), max altitude '
              f'{m["tt"][:, 2].max():.2f} m')
        print(f'    volumetric coverage xi {100 * c["xi"]:.2f}% (free voxels mapped free {100 * c["free"]:.2f}%, '
              f'wall voxels mapped occupied {100 * c["wall"]:.2f}%)')
        print(f'    free mapped free by level: L1 {100 * c["L1"]:.2f}%, L2 {100 * c["L2"]:.2f}%, '
              f'L3 {100 * c["L3"]:.2f}%, shafts {100 * c["shafts"]:.2f}%; L1 scan plane (z = 1.5 m layer) '
              f'{100 * c["plane"]:.2f}%')
        print(f'    dust-zone voxels unknown {100 * c["dust_unknown"]:.2f}%; beams lost to dust {m["lost_dust"]} of '
              f'{m["n_beams"]}; max one-way optical depth {m["tau_max"]:.3f}; min wall clearance '
              f'{m["min_clear"]:.2f} m (asserted >= {R_DRONE}); max |v_z| {m["vz_max"]:.2f} m/s')
        if mode == '3d':
            print(f'    galleries found at z = ' + ', '.join(f'{z:.2f}' for z in m['levels'])
                  + f' m; shaft traverses {m["n_traverse"]} (given up {m["n_trav_fail"]}); candidate shaft boxes: '
                  f'{sum(b[4] == 2 for b in m["shaft_boxes"])} traversed or leading to a known level, '
                  f'{sum(b[4] == -1 for b in m["shaft_boxes"])} rejected, {sum(b[4] == 0 for b in m["shaft_boxes"])} untested')
        for thr in (0.25, 0.5):
            hit = [x[0] for x in m['cov_log'] if x[1]['xi'] >= thr]
            print(f'    xi reached {int(100 * thr)}% at ' + (f'{hit[0]:.0f} s' if hit else 'never'))
    c3, cf, c2 = (res['missions'][k]['cov_log'][-1][1] for k in ('3d', '3d_fixed', '2d'))
    print(f'  Fair comparison (same 16-layer sensor): xi 3d_fixed {100 * cf["xi"]:.2f}% -> 3d {100 * c3["xi"]:.2f}% '
          f'(+{100 * (c3["xi"] - cf["xi"]):.2f} pts from altitude control / shaft traversal)')
    print(f'  2D baseline on its own plane (Level 1 layer at z = 1.5 m): {100 * c2["plane"]:.2f}% vs 3d '
          f'{100 * c3["plane"]:.2f}% (its volumetric xi {100 * c2["xi"]:.2f}% is limited by construction to one '
          f'{VOX} m layer)')
    t3, tf = res['missions']['3d']['t'], res['missions']['3d_fixed']['t']
    print(f'  T_exp: 3d {t3:.1f} s vs 3d_fixed {tf:.1f} s (+{t3 - tf:.1f} s); time above z = 4.25 m in the 3d mission '
          f'{100 * np.mean(res["missions"]["3d"]["tt"][:, 2] > 4.25):.1f}% of steps')
    pe = res['probe_est']
    cp = pe['cov_log'][-1][1]
    e = pe['te'] - pe['tt']
    print(f'  [Closed loop with the ICP estimate (cap {T_PROBE:.0f} s)] ended: {pe["status"]} at {pe["t"]:.1f} s, '
          f'max altitude {pe["tt"][:, 2].max():.2f} m; xi {100 * cp["xi"]:.2f}%, L1/L2/L3 free '
          f'{100 * cp["L1"]:.2f}/{100 * cp["L2"]:.2f}/{100 * cp["L3"]:.2f}%; pose RMSE {np.sqrt(np.mean(np.sum(e ** 2, 1))):.3f} m '
          f'(z {np.sqrt(np.mean(e[:, 2] ** 2)):.3f} m), final error {np.linalg.norm(e[-1]):.3f} m; '
          f'true-pose mission xi at the same time '
          f'{100 * [x[1]["xi"] for x in res["missions"]["3d"]["cov_log"] if x[0] <= pe["t"] + 1e-9][-1]:.2f}%')
    rp = res['replays']
    print(f'  [Pose replays on the flown 3D trajectory, seeds {REPLAY_SEEDS}, {rp[REPLAY_SEEDS[0]]["n"]} scan '
          f'intervals, {rp[REPLAY_SEEDS[0]]["n_shaft"]} inside shafts]')
    for key, name in (('dr', 'dead reckoning'), ('icp', 'ICP + prior + RF altitude')):
        rs = [rp[s][key] for s in REPLAY_SEEDS]
        f = lambda q: np.array([r[q] for r in rs])
        print(f'    {name:<26} RMSE 3D {f("rmse").mean():.3f} ± {f("rmse").std():.3f} m, horizontal '
              f'{f("rmse_xy").mean():.3f} ± {f("rmse_xy").std():.3f} m, z {f("rmse_z").mean():.3f} ± '
              f'{f("rmse_z").std():.3f} m; final {f("final").mean():.3f} ± {f("final").std():.3f} m '
              f'(per seed RMSE ' + ', '.join(f'{v:.3f}' for v in f('rmse')) + ')')
        print(f'      z error rms while in shafts {f("ez_shaft").mean():.3f} m, in galleries {f("ez_gal").mean():.3f} m')
        print(f'      per-scan increment error rms: z shafts {1000 * f("inc_z_shaft").mean():.1f} mm, z galleries '
              f'{1000 * f("inc_z_gal").mean():.1f} mm; xy shafts {1000 * f("inc_xy_shaft").mean():.1f} mm, xy galleries '
              f'{1000 * f("inc_xy_gal").mean():.1f} mm')
    a = np.array([rp[s]['icp']['rmse'] for s in REPLAY_SEEDS])
    b = np.array([rp[s]['dr']['rmse'] for s in REPLAY_SEEDS])
    zs = np.array([rp[s]['icp_scan_dz_shaft'] for s in REPLAY_SEEDS])
    zg = np.array([rp[s]['icp_scan_dz_gal'] for s in REPLAY_SEEDS])
    print(f'    paired: ICP RMSE < dead reckoning in {int((a < b).sum())}/{len(a)} seeds')
    print(f'    LiDAR-only (before the RF altitude) vertical increment error per scan: shafts {1000 * zs.mean():.1f} mm, '
          f'galleries {1000 * zg.mean():.1f} mm (larger in shafts in {int((zs > zg).sum())}/{len(zs)} seeds); prior alone '
          f'{1000 * SIGMA_ODO * np.sqrt(SCAN_STEPS):.1f} mm; RF re-anchors per run '
          + ', '.join(str(rp[s]['n_anchor']) for s in REPLAY_SEEDS))
    print(f'  [Degeneracy threshold sweep: seed {REPLAY_SEEDS[0]}, first {LAM_N} scan intervals]')
    for lam, r in res['lam_sweep'].items():
        print(f"    LAM_TH = {lam:8.0f}: ICP RMSE 3D {r['icp']['rmse']:.3f} m (horizontal {r['icp']['rmse_xy']:.3f}, "
              f"z {r['icp']['rmse_z']:.3f}); dead reckoning {r['dr']['rmse']:.3f} m; LiDAR-only dz error shafts "
              f"{1000 * r['icp_scan_dz_shaft']:.1f} mm, galleries {1000 * r['icp_scan_dz_gal']:.1f} mm")
    mt, me = res['missions']['3d']['cov_log'][-1][1], res['map_est']
    print(f'  Map rebuilt from the mission scans with the ICP poses of seed {REPLAY_SEEDS[0]}: xi {100 * me["xi"]:.2f}% '
          f'(walls {100 * me["wall"]:.2f}%) vs {100 * mt["xi"]:.2f}% (walls {100 * mt["wall"]:.2f}%) with true poses, '
          f'difference {100 * (me["xi"] - mt["xi"]):+.2f} pts')
    print('  [Dust sweep: Level 1 scans rebuilt with stronger extinction (true poses)]')
    for mu in MU_SWEEP:
        d = res['dust'][mu]
        reach = dust_reach(L1, np.array([0.0]), mu * MU_EXT)[0]
        print(f'    mu = {mu * MU_EXT:.2f} (x{mu:g}): horizontal reach at z = 1.5 m '
              + (f'{reach:.2f} m' if np.isfinite(reach) else 'unlimited')
              + f'; beams lost {100 * d["lost"]:.2f}%; dust-zone voxels unknown {100 * d["dust_unknown"]:.2f}%; '
              f'L1 free mapped {100 * d["L1"]:.2f}%')


if __name__ == '__main__':
    res = run_simulation()
    report(res)
    print(f"  Simulation time {res['runtime']:.0f} s", file=sys.stderr)
    out = os.path.normpath(OUTPUT_DIR)
    os.makedirs(out, exist_ok=True)
    plot_occupancy_3d(res, out)
    plot_xz_section(res, out)
    plot_altitude(res, out)
    plot_coverage(res, out)
    plot_dust(res, out)
    save_animation(res, out)
