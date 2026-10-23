"""
S050 3D Swarm SLAM
==================
Four drones explore a 200 x 200 x 40 m GPS-denied volume with 50 unknown 3D
landmarks (z in [0, 30] m).  Each drone runs a 3D EKF-SLAM filter with state
(x, y, z, psi) + 3 per landmark and a range-bearing-elevation sensor
(r_sense = 30 m).  The drones fly a TRUE trajectory that differs from the
commanded odometry by process noise; measurements come from the true pose.

Two flight logs are generated per seed (both navigate on each drone's own
isolated estimate, as in the 2D-folder S050):
  * 'band'  - altitude-separated 3D frontier policy: drone i prefers frontier
              voxels in its 10 m altitude band (out-of-band gain x 0.2)
  * 'flat'  - 2D-constrained baseline: all drones locked at z = 5 m, frontier
              chosen in the horizontal plane (same 3D sensor and filter)
Both logs of a seed draw their motion and measurement noise from the same
pre-generated tables (common random numbers, indexed by step, drone and landmark).
On each log two estimators are run on the SAME measurements:
  * isolated    - one 3D EKF-SLAM per drone, no communication
  * distributed - same, plus pairwise SE(3) map merging within r_comm = 50 m
                  (yaw-hypothesis consensus, full SVD refinement on >= 4
                  non-coplanar inliers, only new landmarks are transferred)
The EKF/merge logic is the 2D script's, lifted to 3D.

Usage:
    conda activate drones
    python src/03_environmental_sar/3d/s050_3d_slam.py
"""

import os
for _v in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ.setdefault(_v, '1')      # single-threaded BLAS in every worker (reproducible)
import multiprocessing as mp
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

SCRIPT_DIR   = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, '..', '..', '..'))
OUTPUT_DIR   = os.path.join(PROJECT_ROOT, 'outputs', '03_environmental_sar', '3d', 's050_3d_slam')

# -- Parameters --------------------------------------------------------------------
N_DRONES     = 4
AREA, HEIGHT = 200.0, 40.0
M_LANDMARKS  = 50
LM_Z         = (0.0, 30.0)
V_FWD        = 3.0        # m/s
VZ_MAX       = 1.5        # m/s
K_ZCMD       = 0.5        # 1/s  added: altitude command gain
DT           = 0.2
T_MAX        = 800.0
N_STEPS      = int(round(T_MAX / DT))
R_SENSE      = 30.0
R_COMM       = 50.0
SIG_RHO, SIG_PHI, SIG_EPS = 0.3, 0.03, 0.04
SIG_AX = SIG_AY = 0.05    # 2D values (per-step position noise = SIG * DT)
SIG_AZ       = 0.05       # card
SIG_OMEGA    = 0.02       # 2D value
OMEGA_MAX    = 1.0        # rad/s (2D)
P0_POSE      = 0.01
FILTER_INFLATE = 4.0      # filter uses 4x the true Q and R (2D)
GATE         = 7.815      # chi2(3, 0.95)
NEW_GATE     = 16.266     # chi2(3, 0.999)
MIN_LM_SEP   = 4.0        # m (2D)
MATCH_R      = 2.5        # m  card: common-landmark proximity threshold
MIN_COMMON   = 4          # card
NONCOPLANAR  = 1.0        # m  added: smallest RMS spread of the inliers about their best-fit plane
ALIGN_SEARCH_R = 20.0     # m (2D)
ALIGN_MAX_ROT  = 0.5      # rad (2D)
MERGE_PERIOD = 30.0       # s (2D)
MERGE_INFLATE = 10.0      # (2D)
SENSED_MARGIN = 3.0       # m (2D)
SUCCESS_R    = 1.5        # m card
Z_BOUNDS     = (0.5, 35.0)
BANDS        = [(0.0, 10.0), (10.0, 20.0), (20.0, 30.0), (30.0, 40.0)]
GAMMA_OUT    = 0.2
Z_FLAT       = 5.0        # 2D-constrained baseline altitude (card start altitude)
VOX          = 5.0        # m  frontier voxel (card: 2 m, see README)
RETARGET_DT  = 2.0        # s (2D)
LOOP_GAP     = 60.0       # s  a landmark not seen for this long counts as a revisit
LOOP_MIN     = 3          # mapped landmarks updated in one step for a revisit event (card: 5, see README)
SNAP_DT      = 10.0
N_MC         = 10         # seeds 0..9 (2D value)
N_WORKERS    = 10         # parallel processes
SHOW_SEED    = 0
LANDMARK_SEED = 42
GIF_FRAMES   = 100

STARTS = np.array([[20.0, 20.0, 5.0, 0.0], [25.0, 20.0, 5.0, np.pi / 6],
                   [20.0, 25.0, 5.0, np.pi / 3], [25.0, 25.0, 5.0, np.pi / 2]])
R_OBS  = FILTER_INFLATE * np.diag([SIG_RHO ** 2, SIG_PHI ** 2, SIG_EPS ** 2])
Q_POSE = FILTER_INFLATE * np.diag([(SIG_AX * DT) ** 2, (SIG_AY * DT) ** 2,
                                   (SIG_AZ * DT) ** 2, (SIG_OMEGA * DT) ** 2])
NX, NZ = int(AREA / VOX), int(HEIGHT / VOX)
VC_XY = (np.arange(NX) + 0.5) * VOX
VC_Z = (np.arange(NZ) + 0.5) * VOX
GZ, GY, GX = np.meshgrid(VC_Z, VC_XY, VC_XY, indexing='ij')   # grid[z, y, x]
DRONE_COLORS = ['#e74c3c', '#2980b9', '#27ae60', '#8e44ad']
STRATS = [('band', 'iso'), ('band', 'dist'), ('flat', 'iso'), ('flat', 'dist')]
LABELS = {('band', 'iso'): '3D bands, isolated', ('band', 'dist'): '3D bands, distributed',
          ('flat', 'iso'): 'flat 5 m, isolated', ('flat', 'dist'): 'flat 5 m, distributed'}
STYLE = {('band', 'iso'): ('#2980b9', '--'), ('band', 'dist'): ('#e74c3c', '-'),
         ('flat', 'iso'): ('#7f8c8d', ':'), ('flat', 'dist'): ('#f39c12', '-.')}


def wrap(a):
    return np.arctan2(np.sin(a), np.cos(a))


# -- 3D EKF-SLAM ----------------------------------------------------------------------

class Slam3D:
    def __init__(self, pose):
        self.X = np.array(pose, float)
        self.P = np.eye(4) * P0_POSE

    @property
    def n_lm(self):
        return (len(self.X) - 4) // 3

    @property
    def landmarks(self):
        return self.X[4:].reshape(-1, 3)

    def lm_cov(self, j):
        l = 4 + 3 * j
        return self.P[l:l + 3, l:l + 3]

    @property
    def pose(self):
        return self.X[:4]

    def predict(self, v, vz, omega):
        psi = self.X[3]
        F = np.eye(4)
        F[0, 3] = -v * DT * np.sin(psi); F[1, 3] = v * DT * np.cos(psi)
        self.X[0] += v * DT * np.cos(psi); self.X[1] += v * DT * np.sin(psi)
        self.X[2] += vz * DT; self.X[3] = wrap(psi + omega * DT)
        self.P[:4, :] = F @ self.P[:4, :]
        self.P[:, :4] = self.P[:, :4] @ F.T
        self.P[:4, :4] += Q_POSE

    def _obs(self, js):
        """Predicted (rho, phi, eps), Jacobians (n,3,7) w.r.t. (x,y,z,psi,lx,ly,lz), indices (n,7)."""
        x, y, z, psi = self.pose
        L = self.landmarks[js]
        d = L - np.array([x, y, z])
        rxy2 = np.maximum(d[:, 0] ** 2 + d[:, 1] ** 2, 1e-12)
        rxy = np.sqrt(rxy2)
        r2 = rxy2 + d[:, 2] ** 2
        r = np.sqrt(r2)
        rho, phi, eps = r, wrap(np.arctan2(d[:, 1], d[:, 0]) - psi), np.arcsin(np.clip(d[:, 2] / r, -1, 1))
        Hl = np.zeros((len(js), 3, 3))            # d h / d delta (= d h / d landmark)
        Hl[:, 0, :] = d / r[:, None]
        Hl[:, 1, 0] = -d[:, 1] / rxy2; Hl[:, 1, 1] = d[:, 0] / rxy2
        Hl[:, 2, 0] = -d[:, 0] * d[:, 2] / (rxy * r2)
        Hl[:, 2, 1] = -d[:, 1] * d[:, 2] / (rxy * r2)
        Hl[:, 2, 2] = rxy / r2                     # card's 1/rho_xy corrected
        H = np.zeros((len(js), 3, 7))
        H[:, :, :3] = -Hl; H[:, 1, 3] = -1.0; H[:, :, 4:] = Hl
        l = 4 + 3 * np.asarray(js)
        idx = np.stack([np.zeros_like(l), np.ones_like(l), np.full_like(l, 2), np.full_like(l, 3),
                        l, l + 1, l + 2], axis=1)
        return np.stack([rho, phi, eps], 1), H, idx

    def associate(self, zm):
        if self.n_lm == 0:
            return -1, np.inf
        js = np.flatnonzero(np.linalg.norm(self.landmarks - self.pose[:3], axis=1) < R_SENSE + 15)
        if len(js) == 0:
            return -1, np.inf
        h, H, idx = self._obs(js)
        Pb = self.P[idx[:, :, None], idx[:, None, :]]
        S = H @ Pb @ np.transpose(H, (0, 2, 1)) + R_OBS
        nu = zm[None, :] - h
        nu[:, 1] = wrap(nu[:, 1])
        dm = np.einsum('ni,ni->n', nu, np.linalg.solve(S, nu[:, :, None])[:, :, 0])
        b = int(np.argmin(dm))
        self._cache = (int(js[b]), nu[b].copy(), H[b], idx[b], S[b])   # reused by update()
        return int(js[b]), float(dm[b])

    def update(self, j, zm):
        cj, nu, H, idx, S = self._cache     # innovation / Jacobian / S from associate()
        assert cj == j
        K = self.P[:, idx] @ H.T @ np.linalg.inv(S)
        self.X += K @ nu
        self.X[3] = wrap(self.X[3])
        self.P -= K @ S @ K.T
        self.P = 0.5 * (self.P + self.P.T)

    def add_from_obs(self, zm):
        x, y, z, psi = self.pose
        rho, ph, ep = zm
        a = psi + ph
        ce, se, ca, sa = np.cos(ep), np.sin(ep), np.cos(a), np.sin(a)
        lm = np.array([x + rho * ce * ca, y + rho * ce * sa, z + rho * se])
        Jx = np.array([[1, 0, 0, -rho * ce * sa], [0, 1, 0, rho * ce * ca], [0, 0, 1, 0.0]])
        Jz = np.array([[ce * ca, -rho * ce * sa, -rho * se * ca],
                       [ce * sa, rho * ce * ca, -rho * se * sa],
                       [se, 0.0, rho * ce]])
        n = len(self.X)
        Pn = np.zeros((n + 3, n + 3))
        Pn[:n, :n] = self.P
        cross = Jx @ self.P[:4, :]
        Pn[n:, :n] = cross; Pn[:n, n:] = cross.T
        Pn[n:, n:] = Jx @ self.P[:4, :4] @ Jx.T + Jz @ R_OBS @ Jz.T
        self.X = np.concatenate([self.X, lm]); self.P = Pn

    def add_landmark(self, lm, cov):
        n = len(self.X)
        Pn = np.zeros((n + 3, n + 3)); Pn[:n, :n] = self.P; Pn[n:, n:] = cov
        self.X = np.concatenate([self.X, lm]); self.P = Pn

    def process(self, zm):
        """Returns ('upd', j) / ('new', j) / ('skip', -1)."""
        j, dm = self.associate(zm)
        if j >= 0 and dm <= GATE:
            self.update(j, zm)
            return 'upd', j
        if dm > NEW_GATE:
            x, y, z, psi = self.pose
            a = psi + zm[1]
            cand = np.array([x + zm[0] * np.cos(zm[2]) * np.cos(a),
                             y + zm[0] * np.cos(zm[2]) * np.sin(a), z + zm[0] * np.sin(zm[2])])
            if self.n_lm == 0 or np.min(np.linalg.norm(self.landmarks - cand, axis=1)) >= MIN_LM_SEP:
                self.add_from_obs(zm)
                return 'new', self.n_lm - 1
        return 'skip', -1


# -- SE(3) map merging -----------------------------------------------------------------------

def _one_to_one(A, B, radius):
    if len(A) == 0 or len(B) == 0:
        return []
    D = np.linalg.norm(A[:, None, :] - B[None, :, :], axis=2)
    pairs, ua, ub = [], set(), set()
    for flat in np.argsort(D, axis=None):
        i, j = np.unravel_index(flat, D.shape)
        if D[i, j] >= radius:
            break
        if i in ua or j in ub:
            continue
        pairs.append((int(i), int(j))); ua.add(i); ub.add(j)
    return pairs


def svd_align(Lh, Lv):
    """Card SE(3) alignment: R* = U diag(1, 1, det(U V^T)) V^T, t* = mean_h - R* mean_v."""
    mh, mv = Lh.mean(0), Lv.mean(0)
    U, _, Vt = np.linalg.svd((Lh - mh).T @ (Lv - mv))
    R = U @ np.diag([1.0, 1.0, np.sign(np.linalg.det(U @ Vt))]) @ Vt
    return R, mh - R @ mv


def _rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def robust_align(Lh, Lv):
    """
    Consensus alignment (2D method lifted to 3D): hypotheses from two
    correspondences with consistent 3D inter-landmark distances give a yaw and
    a 3D translation; the hypothesis with most one-to-one inliers (< MATCH_R)
    wins and is refined by the full SVD SE(3) solution on its inliers.
    Returns (R, t, pairs, planarity) with planarity = smallest RMS spread of the
    inliers about their best-fit plane (m).
    """
    Dc = np.linalg.norm(Lh[:, None, :] - Lv[None, :, :], axis=2)
    cand = np.argwhere(Dc < ALIGN_SEARCH_R)
    if len(cand) < 2:
        return None, None, [], 0.0
    Dh = np.linalg.norm(Lh[:, None] - Lh[None], axis=2)
    Dv = np.linalg.norm(Lv[:, None] - Lv[None], axis=2)
    best = (0, np.inf, None, None)
    for a, b in cand:
        ok = ((cand[:, 0] != a) & (cand[:, 1] != b)
              & (np.abs(Dh[a, cand[:, 0]] - Dv[b, cand[:, 1]]) < 1.0) & (Dh[a, cand[:, 0]] > 3.0))
        cd = cand[ok]
        if len(cd) == 0:
            continue
        vh = Lh[cd[:, 0], :2] - Lh[a, :2]; vv = Lv[cd[:, 1], :2] - Lv[b, :2]
        good = (np.linalg.norm(vh, axis=1) > 2.0) & (np.linalg.norm(vv, axis=1) > 2.0)
        ang = wrap(np.arctan2(vh[:, 1], vh[:, 0]) - np.arctan2(vv[:, 1], vv[:, 0]))
        good &= np.abs(ang) <= ALIGN_MAX_ROT
        gi = np.flatnonzero(good)
        if len(gi) == 0:
            continue
        c, s = np.cos(ang[gi]), np.sin(ang[gi])
        Rs = np.zeros((len(gi), 3, 3))
        Rs[:, 0, 0] = c; Rs[:, 0, 1] = -s; Rs[:, 1, 0] = s; Rs[:, 1, 1] = c; Rs[:, 2, 2] = 1.0
        ts = Lh[a][None] - np.einsum('gij,j->gi', Rs, Lv[b])
        Lt = np.einsum('gij,nj->gni', Rs, Lv) + ts[:, None, :]           # (G, nv, 3)
        dmin = np.sqrt(((Lt[:, None, :, :] - Lh[None, :, None, :]) ** 2).sum(-1)).min(1)  # (G, nv)
        inl = dmin < MATCH_R
        n_in = inl.sum(1)
        sq = np.where(inl, dmin ** 2, 0.0).sum(1)
        for g in range(len(gi)):          # same tie-break order as a sequential loop
            if n_in[g] > best[0] or (n_in[g] == best[0] and sq[g] < best[1]):
                best = (int(n_in[g]), float(sq[g]), Rs[g], ts[g])
    if best[0] < MIN_COMMON:
        return None, None, [], 0.0
    R, t = best[2], best[3]
    pairs = _one_to_one(Lh, Lv @ R.T + t, MATCH_R)
    if len(pairs) < MIN_COMMON:
        return None, None, pairs, 0.0
    P = Lh[[i for i, _ in pairs]]
    planarity = float(np.linalg.svd(P - P.mean(0), compute_uv=False)[-1] / np.sqrt(len(P)))
    if planarity < NONCOPLANAR:
        return R, t, pairs, planarity            # caller rejects (degenerate for SO(3))
    R, t = svd_align(P, Lv[[j for _, j in pairs]])
    pairs = _one_to_one(Lh, Lv @ R.T + t, MATCH_R)
    return R, t, pairs, planarity


def merge_maps(host, vis_L, vis_C, host_sensed):
    """2D merge rule in 3D. Returns dict(n_common, n_good, n_added, status, resid, rot_deg)."""
    out = dict(n_common=0, n_good=0, n_added=0, status='no_overlap', resid=np.nan, rot_deg=np.nan)
    if len(vis_L) == 0 or host.n_lm == 0:
        return out
    R, t, pairs, plan = robust_align(host.landmarks, vis_L)
    out['n_common'] = len(pairs)
    if R is None or len(pairs) < MIN_COMMON:
        out['status'] = 'too_few'
        return out
    if plan < NONCOPLANAR:
        out['status'] = 'coplanar'
        return out
    Lt = vis_L @ R.T + t
    Ct = np.einsum('ij,njk,lk->nil', R, vis_C, R)
    good = []
    for i, j in pairs:
        dl = Lt[j] - host.landmarks[i]
        if float(dl @ np.linalg.solve(host.lm_cov(i) + MERGE_INFLATE * Ct[j], dl)) <= NEW_GATE:
            good.append((i, j))
    out['n_good'] = len(good)
    out['resid'] = float(np.sqrt(np.mean([np.sum((Lt[j] - host.landmarks[i]) ** 2) for i, j in pairs])))
    out['rot_deg'] = float(np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))))
    if len(good) < MIN_COMMON or len(good) < 0.5 * len(pairs):
        out['status'] = 'inconsistent'
        return out
    matched = {j for _, j in pairs}
    for j in range(len(Lt)):
        if j in matched:
            continue
        diff = host.landmarks - Lt[j]
        if np.min(np.linalg.norm(diff, axis=1)) < MIN_LM_SEP:
            continue
        iz, iy, ix = _vox_index(Lt[j])
        if host_sensed[iz, iy, ix]:
            continue
        S = np.array([host.lm_cov(i) for i in range(host.n_lm)]) + MERGE_INFLATE * Ct[j]
        d2 = np.einsum('ni,ni->n', diff, np.linalg.solve(S, diff[:, :, None])[:, :, 0])
        if d2.min() <= NEW_GATE:
            continue
        host.add_landmark(Lt[j], MERGE_INFLATE * Ct[j])
        out['n_added'] += 1
    out['status'] = 'merged'
    return out


def _snapshot(f):
    return f.landmarks.copy(), np.array([f.lm_cov(a) for a in range(f.n_lm)]).reshape(-1, 3, 3)


# -- Exploration grid / frontier ----------------------------------------------------------

def _vox_index(p):
    return (int(np.clip(p[2] // VOX, 0, NZ - 1)), int(np.clip(p[1] // VOX, 0, NX - 1)),
            int(np.clip(p[0] // VOX, 0, NX - 1)))


def mark(grid, p, radius=R_SENSE):
    grid |= (GX - p[0]) ** 2 + (GY - p[1]) ** 2 + (GZ - p[2]) ** 2 <= radius ** 2


def _box_sum(a, r):
    """Sum of a over a (2r+1)^3 box around every voxel (3D integral image)."""
    I = np.pad(a.astype(np.int32).cumsum(0).cumsum(1).cumsum(2), ((1, 0), (1, 0), (1, 0)))
    def rng(n):
        i = np.arange(n); return np.clip(i - r, 0, n), np.clip(i + r + 1, 0, n)
    zl, zh = rng(a.shape[0]); yl, yh = rng(a.shape[1]); xl, xh = rng(a.shape[2])
    Z0, Z1 = zl[:, None, None], zh[:, None, None]
    Y0, Y1 = yl[None, :, None], yh[None, :, None]
    X0, X1 = xl[None, None, :], xh[None, None, :]
    return (I[Z1, Y1, X1] - I[Z0, Y1, X1] - I[Z1, Y0, X1] - I[Z1, Y1, X0]
            + I[Z0, Y0, X1] + I[Z0, Y1, X0] + I[Z1, Y0, X0] - I[Z0, Y0, X0]).astype(float)


R_VOX = int(R_SENSE // VOX)


def frontier_target(grid, pos, others, band, flat):
    unexp = ~grid
    if not unexp.any():
        return None
    IG = _box_sum(unexp, R_VOX)
    for ot in others:
        if ot is not None:
            IG[(GX - ot[0]) ** 2 + (GY - ot[1]) ** 2 + (GZ - ot[2]) ** 2 < R_SENSE ** 2] *= 0.1
    if flat:   # 2D-constrained: candidates in the layer of the flight altitude only
        k = int(Z_FLAT // VOX)
        mask = np.zeros_like(unexp); mask[k] = True
        cand = unexp.any(axis=0)[None].repeat(NZ, 0) & mask
        IG = np.where(mask, IG, 0.0)
    else:
        cand = unexp
        lo, hi = band
        IG = IG * np.where((GZ >= lo) & (GZ < hi), 1.0, GAMMA_OUT)
    dist = np.sqrt((GX - pos[0]) ** 2 + (GY - pos[1]) ** 2 + (GZ - pos[2]) ** 2)
    score = np.where(cand, IG / (dist + VOX), -1.0)
    iz, iy, ix = np.unravel_index(int(np.argmax(score)), score.shape)
    tz = Z_FLAT if flat else float(np.clip(VC_Z[iz], *Z_BOUNDS))
    return np.array([VC_XY[ix], VC_XY[iy], tz])


# -- Metrics ---------------------------------------------------------------------------------

def quality(L, true_lm):
    if len(L) == 0:
        return dict(found=np.zeros(len(true_lm), bool), rms=np.nan, vrms=np.nan, hrms=np.nan,
                    vrms_all=np.nan, hrms_all=np.nan, n_est=0, err=np.full((len(true_lm), 3), np.nan))
    D = np.linalg.norm(true_lm[:, None] - L[None], axis=2)
    jn = D.argmin(1); dmin = D.min(1)
    found = dmin < SUCCESS_R
    err = L[jn] - true_lm
    f = found
    return dict(found=found, n_est=len(L), err=np.where(found[:, None], err, np.nan),
                rms=float(np.sqrt(np.mean(dmin[f] ** 2))) if f.any() else np.nan,
                vrms=float(np.sqrt(np.mean(err[f, 2] ** 2))) if f.any() else np.nan,
                hrms=float(np.sqrt(np.mean(np.sum(err[f, :2] ** 2, 1)))) if f.any() else np.nan,
                # untruncated: nearest estimate of every true landmark, no 1.5 m cut
                vrms_all=float(np.sqrt(np.mean(err[:, 2] ** 2))),
                hrms_all=float(np.sqrt(np.mean(np.sum(err[:, :2] ** 2, 1)))))


# -- Simulation ------------------------------------------------------------------------------

def run_log(mode, seed, true_lm, record):
    """One flight log ('band' or 'flat'); isolated and distributed filters on the same data."""
    rng = np.random.default_rng(seed)
    # common random numbers: the same noise table drives both flight logs of a seed
    W_MOT = rng.standard_normal((N_STEPS, N_DRONES, 4))
    W_MEAS = rng.standard_normal((N_STEPS, N_DRONES, M_LANDMARKS, 3))
    flat = mode == 'flat'
    starts = STARTS.copy()
    truth = starts.copy()
    iso = [Slam3D(s) for s in starts]
    dist = [Slam3D(s) for s in starts]
    grids = [np.zeros((NZ, NX, NX), bool) for _ in range(N_DRONES)]
    sensed = [np.zeros((NZ, NX, NX), bool) for _ in range(N_DRONES)]
    targets = [None] * N_DRONES
    in_contact = np.zeros((N_DRONES, N_DRONES), bool)
    last_merge = np.full((N_DRONES, N_DRONES), -np.inf)
    merges, loops = [], []
    last_seen = [dict() for _ in range(N_DRONES)]       # distributed filter: landmark -> time
    snaps = {'t': [], 'iso': [], 'dist': []}
    hist = dict(true=[[] for _ in range(N_DRONES)], est=[[] for _ in range(N_DRONES)])
    frames = []
    frame_every = max(1, N_STEPS // GIF_FRAMES)
    min_eig = np.inf
    for step in range(N_STEPS):
        t = (step + 1) * DT
        for k in range(N_DRONES):
            nav = iso[k].pose
            mark(grids[k], nav[:3])
            mark(sensed[k], dist[k].pose[:3], R_SENSE - SENSED_MARGIN)
            if (targets[k] is None or step % int(RETARGET_DT / DT) == 0
                    or np.linalg.norm(targets[k] - nav[:3]) < VOX):
                others = [targets[j] for j in range(N_DRONES) if j != k]
                targets[k] = frontier_target(grids[k], nav[:3], others, BANDS[k], flat)
                if targets[k] is None:
                    grids[k][:] = False
                    targets[k] = frontier_target(grids[k], nav[:3], [], BANDS[k], flat)
            ang = np.arctan2(targets[k][1] - nav[1], targets[k][0] - nav[0])
            omega = float(np.clip(wrap(ang - nav[3]) / DT, -OMEGA_MAX, OMEGA_MAX))
            vz = float(np.clip(K_ZCMD * (targets[k][2] - nav[2]), -VZ_MAX, VZ_MAX))
            psi = truth[k, 3]
            truth[k] += [V_FWD * DT * np.cos(psi) + SIG_AX * DT * W_MOT[step, k, 0],
                         V_FWD * DT * np.sin(psi) + SIG_AY * DT * W_MOT[step, k, 1],
                         vz * DT + SIG_AZ * DT * W_MOT[step, k, 2],
                         omega * DT + SIG_OMEGA * DT * W_MOT[step, k, 3]]
            truth[k, 3] = wrap(truth[k, 3])
            assert truth[k, 2] > 0.0, 'drone below ground'
            iso[k].predict(V_FWD, vz, omega)
            dist[k].predict(V_FWD, vz, omega)
            pre = dist[k].pose[:3].copy()
            d = true_lm - truth[k, :3]
            r = np.linalg.norm(d, axis=1)
            n_revisit, n_upd = 0, 0
            for j in np.flatnonzero(r <= R_SENSE):
                zm = np.array([r[j] + SIG_RHO * W_MEAS[step, k, j, 0],
                               wrap(np.arctan2(d[j, 1], d[j, 0]) - truth[k, 3] + SIG_PHI * W_MEAS[step, k, j, 1]),
                               np.arcsin(d[j, 2] / r[j]) + SIG_EPS * W_MEAS[step, k, j, 2]])
                iso[k].process(zm)
                kind, jl = dist[k].process(zm)
                if kind == 'upd':
                    n_upd += 1
                    if t - last_seen[k].get(jl, t) >= LOOP_GAP:
                        n_revisit += 1
                    last_seen[k][jl] = t
                elif kind == 'new':
                    last_seen[k][jl] = t
            if n_revisit >= 1 and n_upd >= LOOP_MIN:
                post = dist[k].pose[:3]
                loops.append(dict(t=t, k=k, n=n_revisit, corr=float(np.linalg.norm(post - pre)),
                                  err_before=float(np.linalg.norm(pre - truth[k, :3])),
                                  err_after=float(np.linalg.norm(post - truth[k, :3]))))
        for i in range(N_DRONES):
            for j in range(i + 1, N_DRONES):
                close = np.linalg.norm(truth[i, :3] - truth[j, :3]) <= R_COMM
                if close and (not in_contact[i, j] or t - last_merge[i, j] >= MERGE_PERIOD - 1e-9):
                    si, sj = _snapshot(dist[i]), _snapshot(dist[j])
                    a = merge_maps(dist[i], *sj, sensed[i])
                    b = merge_maps(dist[j], *si, sensed[j])
                    last_merge[i, j] = t
                    merges.append(dict(t=t, i=i, j=j, a=a, b=b))
                in_contact[i, j] = close
        if step % 100 == 0:
            for f in iso + dist:
                min_eig = min(min_eig, float(np.linalg.eigvalsh(f.P).min()))
        if record:
            for k in range(N_DRONES):
                hist['true'][k].append(truth[k, :3].copy()); hist['est'][k].append(dist[k].pose[:3].copy())
            if step % frame_every == 0:
                frames.append((t, truth[:, :3].copy(), [g.copy() for g in grids]))
        if abs(t / SNAP_DT - round(t / SNAP_DT)) < 1e-9:
            snaps['t'].append(t)
            for name, fl in (('iso', iso), ('dist', dist)):
                q = [quality(f.landmarks, true_lm) for f in fl]
                pe = [np.linalg.norm(fl[k].pose[:3] - truth[k, :3]) for k in range(N_DRONES)]
                snaps[name].append(dict(cov=np.mean([x['found'].mean() for x in q]),
                                        rms=np.nanmean([x['rms'] for x in q]) if any(np.isfinite(x['rms']) for x in q) else np.nan,
                                        vrms=np.nanmean([x['vrms'] for x in q]) if any(np.isfinite(x['vrms']) for x in q) else np.nan,
                                        hrms=np.nanmean([x['hrms'] for x in q]) if any(np.isfinite(x['hrms']) for x in q) else np.nan,
                                        pose=float(np.sqrt(np.mean(np.square(pe))))))
    for f in iso + dist:
        min_eig = min(min_eig, float(np.linalg.eigvalsh(f.P).min()))
    final = {}
    for name, fl in (('iso', iso), ('dist', dist)):
        q = [quality(f.landmarks, true_lm) for f in fl]
        found_any = np.any([x['found'] for x in q], axis=0)
        final[name] = dict(q=q, cov=float(np.mean([x['found'].mean() for x in q])),
                           union=float(found_any.mean()),
                           rms=float(np.nanmean([x['rms'] for x in q])),
                           vrms=float(np.nanmean([x['vrms'] for x in q])),
                           hrms=float(np.nanmean([x['hrms'] for x in q])),
                           hrms_all=float(np.nanmean([x['hrms_all'] for x in q])),
                           vrms_all=float(np.nanmean([x['vrms_all'] for x in q])),
                           pose=float(np.sqrt(np.mean([np.sum((fl[k].pose[:3] - truth[k, :3]) ** 2)
                                                       for k in range(N_DRONES)]))),
                           n_est=[x['n_est'] for x in q],
                           found_by_z=[float(np.mean([x['found'][b].mean() for x in q]))
                                       for b in _z_bins(true_lm)])
    return dict(final=final, snaps=snaps, merges=merges, loops=loops, hist=hist, frames=frames,
                iso=iso, dist=dist, min_eig=min_eig, truth=truth.copy())


def _z_bins(true_lm):
    z = true_lm[:, 2]
    return [(z < 10), (z >= 10) & (z < 20), (z >= 20)]


def make_landmarks():
    rng = np.random.default_rng(LANDMARK_SEED)
    return np.c_[rng.uniform(10, 190, (M_LANDMARKS, 2)), rng.uniform(*LM_Z, M_LANDMARKS)]


def _task(args):
    mode, seed = args
    return run_log(mode, seed, make_landmarks(), record=(seed == SHOW_SEED))


def run_simulation():
    """All (log, seed) runs; independent, so they run in parallel processes.
    Each run seeds its own generator, so the result does not depend on scheduling."""
    true_lm = make_landmarks()
    tasks = [(mode, seed) for seed in range(N_MC) for mode in ('band', 'flat')]
    with mp.Pool(min(N_WORKERS, len(tasks))) as pool:
        outs = pool.map(_task, tasks)
    res = dict(zip(tasks, outs))
    for mode, seed in tasks:
        f = res[(mode, seed)]['final']
        print(f'  seed {seed} {mode}: coverage iso {100*f["iso"]["cov"]:.1f}% dist {100*f["dist"]["cov"]:.1f}%'
              f' | pose RMSE iso {f["iso"]["pose"]:.2f} m dist {f["dist"]["pose"]:.2f} m')
    return dict(true_lm=true_lm, res=res)


# -- Plots -----------------------------------------------------------------------------------

def _save(fig, name):
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    p = os.path.join(OUTPUT_DIR, name)
    fig.savefig(p, dpi=120, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {p}')


def plot_ground_truth(data):
    L = data['true_lm']
    fig = plt.figure(figsize=(8, 6.5))
    ax = fig.add_subplot(111, projection='3d')
    sc = ax.scatter(*L.T, c=L[:, 2], cmap='viridis', s=40, depthshade=False)
    fig.colorbar(sc, ax=ax, shrink=0.6, label='landmark height (m)')
    ax.scatter(STARTS[:, 0], STARTS[:, 1], STARTS[:, 2], c=DRONE_COLORS, marker='^', s=80)
    xx, yy = np.meshgrid([0, AREA], [0, AREA])
    for lo, _ in BANDS[1:]:
        ax.plot_surface(xx, yy, np.full_like(xx, lo), alpha=0.08, color='gray')
    ax.set_xlim(0, AREA); ax.set_ylim(0, AREA); ax.set_zlim(0, HEIGHT)
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)'); ax.set_zlabel('z (m)')
    ax.set_title('S050 3D - 50 true landmarks, drone starts (triangles), 10 m band boundaries',
                 fontsize=10)
    _save(fig, 'ground_truth_3d.png')


def plot_trajectories_map(data):
    r = data['res'][('band', SHOW_SEED)]; L = data['true_lm']
    fig = plt.figure(figsize=(14, 11))
    for k in range(N_DRONES):
        ax = fig.add_subplot(2, 2, k + 1, projection='3d')
        tr = np.array(r['hist']['true'][k]); es = np.array(r['hist']['est'][k])
        ax.plot(*tr.T, color=DRONE_COLORS[k], lw=1, label='true path')
        ax.plot(*es.T, color='k', lw=0.6, ls='--', label='distributed estimate')
        ax.scatter(*L.T, c='green', s=18, label='true landmarks')
        M = r['dist'][k].landmarks
        if len(M):
            ax.scatter(*M.T, c='red', marker='x', s=22, label='estimated')
            D = np.linalg.norm(L[:, None] - M[None], axis=2)
            for i in np.flatnonzero(D.min(1) < SUCCESS_R):
                ax.plot(*np.c_[L[i], M[D[i].argmin()]], color='orange', lw=1)
        q = r['final']['dist']['q'][k]
        ax.set_title(f'Drone {k+1} (band {BANDS[k][0]:.0f}-{BANDS[k][1]:.0f} m): found {q["found"].sum()}/50, '
                     f'RMS {q["rms"]:.2f} m', fontsize=9)
        ax.set_xlim(0, AREA); ax.set_ylim(0, AREA); ax.set_zlim(0, HEIGHT)
        ax.set_xlabel('x'); ax.set_ylabel('y'); ax.set_zlabel('z')
        if k == 0:
            ax.legend(fontsize=7, loc='upper left')
    fig.suptitle(f'S050 3D - Distributed 3D band run, seed {SHOW_SEED}: paths and maps', fontsize=12,
                 fontweight='bold')
    _save(fig, 'trajectories_map_3d.png')


def plot_altitude_profile(data):
    fig, axes = plt.subplots(1, 2, figsize=(13, 4.2), sharey=True)
    for ax, mode in zip(axes, ('band', 'flat')):
        r = data['res'][(mode, SHOW_SEED)]
        for k in range(N_DRONES):
            tr = np.array(r['hist']['true'][k])
            ax.plot(np.arange(1, len(tr) + 1) * DT, tr[:, 2], color=DRONE_COLORS[k], lw=1, label=f'drone {k+1}')
        for lo, hi in BANDS:
            ax.axhline(lo, color='gray', lw=0.5, ls=':')
        ax.set_xlabel('time (s)'); ax.set_title(f'{mode} flight log', fontsize=10); ax.grid(True, alpha=0.3)
    axes[0].set_ylabel('true altitude z (m)'); axes[0].legend(fontsize=8)
    fig.suptitle(f'S050 3D - Altitude profiles (seed {SHOW_SEED})', fontsize=12, fontweight='bold')
    fig.tight_layout()
    _save(fig, 'altitude_profile.png')


def _mc_curve(data, mode, name, key):
    arr = np.array([[s[key] for s in data['res'][(mode, sd)]['snaps'][name]] for sd in range(N_MC)])
    return np.nanmean(arr, 0), np.nanmin(arr, 0), np.nanmax(arr, 0)


def plot_coverage_rms(data):
    t = np.array(data['res'][('band', 0)]['snaps']['t'])
    fig, axes = plt.subplots(1, 4, figsize=(19, 4.3))
    for (mode, name) in STRATS:
        c, ls = STYLE[(mode, name)]
        for ax, key, sc in zip(axes, ('cov', 'rms', 'vrms', 'pose'), (100, 1, 1, 1)):
            m, lo, hi = _mc_curve(data, mode, name, key)
            ax.plot(t, sc * m, color=c, ls=ls, lw=1.5, label=LABELS[(mode, name)])
            ax.fill_between(t, sc * lo, sc * hi, color=c, alpha=0.08)
    titles = ['coverage (landmarks within 1.5 m), %', '3D RMS of found landmarks (m)',
              'vertical RMS of found landmarks (m)', 'pose RMSE (m)']
    for ax, tt in zip(axes, titles):
        ax.set_title(tt, fontsize=10); ax.set_xlabel('time (s)'); ax.grid(True, alpha=0.3)
    axes[0].legend(fontsize=8)
    fig.suptitle(f'S050 3D - Map quality vs time (mean of per-drone maps, {N_MC} seeds, band = min/max)',
                 fontsize=12, fontweight='bold')
    fig.tight_layout()
    _save(fig, 'coverage_rms_vs_time.png')


def plot_merge_events(data):
    fig, ax = plt.subplots(figsize=(11, 4))
    mk = {'merged': ('o', 'green'), 'inconsistent': ('x', 'red'), 'coplanar': ('s', 'purple'),
          'too_few': ('.', 'gray'), 'no_overlap': ('.', 'lightgray')}
    seen = set()
    for e in data['res'][('band', SHOW_SEED)]['merges']:
        for side, (h, v) in (('a', (e['i'], e['j'])), ('b', (e['j'], e['i']))):
            st = e[side]['status']; m, c = mk[st]
            ax.scatter(e['t'], e[side]['n_common'], marker=m, color=c, s=30,
                       label=st if st not in seen else None)
            seen.add(st)
    ax.axhline(MIN_COMMON, color='k', ls='--', lw=0.8, label='minimum common landmarks')
    ax.set_xlabel('time (s)'); ax.set_ylabel('common landmarks found')
    ax.set_title(f'S050 3D - Pairwise SE(3) merge attempts (distributed, band log, seed {SHOW_SEED})',
                 fontsize=11, fontweight='bold')
    ax.grid(True, alpha=0.3); ax.legend(fontsize=8)
    _save(fig, 'merge_events.png')


def plot_loop_closures(data):
    loops = data['res'][('band', SHOW_SEED)]['loops']
    fig, ax = plt.subplots(figsize=(10, 4))
    for k in range(N_DRONES):
        L = [l for l in loops if l['k'] == k]
        if L:
            ax.scatter([l['t'] for l in L], [l['corr'] for l in L], color=DRONE_COLORS[k], s=20,
                       label=f'drone {k+1}: correction')
            ax.scatter([l['t'] for l in L], [l['err_before'] - l['err_after'] for l in L],
                       color=DRONE_COLORS[k], marker='x', s=20)
    ax.axhline(0, color='k', lw=0.6)
    ax.set_xlabel('time (s)'); ax.set_ylabel('m')
    ax.set_title('S050 3D - Revisit (loop-closure) events: dots = EKF pose correction, '
                 'x = reduction of true pose error', fontsize=10)
    ax.grid(True, alpha=0.3); ax.legend(fontsize=8)
    _save(fig, 'loop_closures.png')


def plot_vertical_heatmap(data):
    r = data['res'][('band', SHOW_SEED)]; L = data['true_lm']
    fig, axes = plt.subplots(1, 2, figsize=(13, 5.5))
    for ax, mode in zip(axes, ('band', 'flat')):
        q = data['res'][(mode, SHOW_SEED)]['final']['dist']['q']
        E = np.array([np.abs(x['err'][:, 2]) for x in q])
        found = np.any(~np.isnan(E), axis=0)                  # found by at least one drone
        err = np.full(len(L), np.nan)
        err[found] = np.nanmean(E[:, found], axis=0)
        sc = ax.scatter(L[found, 0], L[found, 1], c=err[found], cmap='coolwarm', vmin=0, vmax=1.5,
                        s=60, edgecolors='k')
        ax.scatter(L[~found, 0], L[~found, 1], marker='x', c='k', s=30, label='not found by any drone')
        for i in range(len(L)):
            ax.annotate(f'{L[i, 2]:.0f}', L[i, :2], fontsize=6, xytext=(3, 3), textcoords='offset points')
        ax.set_xlim(0, AREA); ax.set_ylim(0, AREA); ax.set_aspect('equal')
        ax.set_title(f'{mode} log, distributed: mean |z error| over drones that found it', fontsize=10)
        ax.legend(fontsize=8, loc='lower right')
    fig.colorbar(sc, ax=axes, label='|l_z est - l_z true| (m)', shrink=0.8)
    fig.suptitle(f'S050 3D - Vertical accuracy per landmark (labels = true height, seed {SHOW_SEED})',
                 fontsize=12, fontweight='bold')
    _save(fig, 'vertical_error_heatmap.png')


def save_animation(data):
    r = data['res'][('band', SHOW_SEED)]
    frames = r['frames']
    fig, axes = plt.subplots(1, 3, figsize=(14, 4.8))
    zs = (5.0, 15.0, 25.0)
    ims = []
    for ax, z in zip(axes, zs):
        im = ax.imshow(np.zeros((NX, NX)), origin='lower', extent=[0, AREA, 0, AREA], cmap='Greys_r',
                       vmin=0, vmax=1)
        ims.append(im)
        ax.set_title(f'slice z = {z:.0f} m (white = explored by any drone)', fontsize=9)
    dots = [[ax.plot([], [], 'o', color=DRONE_COLORS[k], ms=6)[0] for k in range(N_DRONES)] for ax in axes]
    ttl = fig.suptitle('')

    def update(f):
        t, tr, grids = frames[f]
        anyg = np.any(grids, axis=0)
        for a, (ax, z) in enumerate(zip(axes, zs)):
            ims[a].set_data(anyg[int(z // VOX)].astype(float))
            for k in range(N_DRONES):
                dots[a][k].set_data([tr[k, 0]], [tr[k, 1]])
                dots[a][k].set_alpha(1.0 if abs(tr[k, 2] - z) < 5 else 0.25)
        ttl.set_text(f'S050 3D - band log, seed {SHOW_SEED}, t = {t:.0f} s (faded dot = drone > 5 m from slice)')
        return ims

    ani = animation.FuncAnimation(fig, update, frames=len(frames), blit=False)
    p = os.path.join(OUTPUT_DIR, 'animation.gif')
    ani.save(p, writer='pillow', fps=5, dpi=70)
    plt.close(fig)
    print(f'Saved: {p} ({len(frames)} frames)')


# -- Report -----------------------------------------------------------------------------------

def report(data):
    L = data['true_lm']; R = data['res']
    print(f'\nLandmarks: {M_LANDMARKS}, heights {L[:, 2].min():.1f}-{L[:, 2].max():.1f} m; '
          f'per 10 m bin: {[int(b.sum()) for b in _z_bins(L)]}')
    print(f'Planar-sensor bound (a 2D range-bearing filter cannot estimate z): landmarks within '
          f'{SUCCESS_R} m of the 5 m flight plane: {int(np.sum(np.abs(L[:, 2] - Z_FLAT) < SUCCESS_R))}/{M_LANDMARKS}')
    print(f'\n-- Final values at t = {T_MAX:.0f} s, {N_MC} seeds: mean [min, max] --')
    keys = [('cov', 'coverage (mean per-drone map) %', 100), ('union', 'team union %', 100),
            ('rms', '3D RMS found (m)', 1), ('hrms', 'horizontal RMS (m)', 1),
            ('vrms', 'vertical RMS (m)', 1), ('hrms_all', 'horizontal RMS all landmarks, untruncated (m)', 1),
            ('vrms_all', 'vertical RMS all landmarks, untruncated (m)', 1), ('pose', 'pose RMSE (m)', 1)]
    for (mode, name) in STRATS:
        parts = []
        for key, lab, sc in keys:
            v = sc * np.array([R[(mode, sd)]['final'][name][key] for sd in range(N_MC)])
            parts.append(f'{lab} {v.mean():.2f} [{v.min():.2f}, {v.max():.2f}]')
        fz = 100 * np.array([R[(mode, sd)]['final'][name]['found_by_z'] for sd in range(N_MC)]).mean(0)
        ne = np.array([R[(mode, sd)]['final'][name]['n_est'] for sd in range(N_MC)])
        print(f'  {LABELS[(mode, name)]:24s}: ' + ' | '.join(parts) +
              f' | coverage by height <10/10-20/>=20 m: {fz[0]:.1f}/{fz[1]:.1f}/{fz[2]:.1f}% | estimates per map max {ne.max()}')
    cov = {s: np.array([R[(s[0], sd)]['final'][s[1]]['cov'] for sd in range(N_MC)]) for s in STRATS}
    pose = {s: np.array([R[(s[0], sd)]['final'][s[1]]['pose'] for sd in range(N_MC)]) for s in STRATS}
    def h2h(a, b, d, better_low=False):
        x, y = d[a], d[b]
        w = np.sum(x < y - 1e-12) if better_low else np.sum(x > y + 1e-12)
        l = np.sum(x > y + 1e-12) if better_low else np.sum(x < y - 1e-12)
        return f'{w}/{l}/{N_MC - w - l}'
    print('  per-seed better/worse/tie (coverage): band-dist vs band-iso ' + h2h(STRATS[1], STRATS[0], cov) +
          ', band-iso vs flat-iso ' + h2h(STRATS[0], STRATS[2], cov) +
          ', band-dist vs flat-dist ' + h2h(STRATS[1], STRATS[3], cov))
    print('  per-seed better/worse/tie (pose RMSE): band-dist vs band-iso ' + h2h(STRATS[1], STRATS[0], pose, True) +
          ', band-iso vs flat-iso ' + h2h(STRATS[0], STRATS[2], pose, True))
    print('  per seed coverage % (band iso / band dist / flat iso / flat dist): ' + '; '.join(
        f's{sd}: ' + '/'.join(f'{100*cov[s][sd]:.0f}' for s in STRATS) for sd in range(N_MC)))
    for mode in ('band', 'flat'):
        st = {}
        for sd in range(N_MC):
            for e in R[(mode, sd)]['merges']:
                for side in ('a', 'b'):
                    st[e[side]['status']] = st.get(e[side]['status'], 0) + 1
        added = sum(e[s]['n_added'] for sd in range(N_MC) for e in R[(mode, sd)]['merges'] for s in 'ab')
        print(f'  merge attempts ({mode}, all seeds, both directions): {dict(sorted(st.items()))}; landmarks transferred {added}')
    ms = [e[s] for e in R[('band', SHOW_SEED)]['merges'] for s in 'ab' if e[s]['status'] == 'merged']
    if ms:
        print(f'  seed {SHOW_SEED} band executed merges: {len(ms)}, alignment residual RMS '
              f'{np.mean([m["resid"] for m in ms]):.2f} m (max {np.max([m["resid"] for m in ms]):.2f}), '
              f'rotation {np.mean([m["rot_deg"] for m in ms]):.2f} deg mean, common landmarks '
              f'{np.mean([m["n_common"] for m in ms]):.1f} mean')
    lp = [l for sd in range(N_MC) for l in R[('band', sd)]['loops']]
    if lp:
        red = np.array([l['err_before'] - l['err_after'] for l in lp])
        print(f'  revisit events (band, distributed, all seeds): {len(lp)}; EKF pose correction mean '
              f'{np.mean([l["corr"] for l in lp]):.2f} m (max {np.max([l["corr"] for l in lp]):.2f}); true pose error '
              f'reduced in {np.sum(red > 0)}/{len(lp)} events (mean change {red.mean():+.3f} m)')
    for mode in ('band', 'flat'):
        z = np.concatenate([np.array(R[(mode, SHOW_SEED)]['hist']['true'][k])[:, 2] for k in range(N_DRONES)])
        print(f'  seed {SHOW_SEED} {mode}: true altitude range {z.min():.2f}-{z.max():.2f} m')
    me = min(R[k]['min_eig'] for k in R)
    print(f'  smallest covariance eigenvalue over all filters and runs: {me:.3e}')


if __name__ == '__main__':
    print('Running S050 3D Swarm SLAM...')
    data = run_simulation()
    report(data)
    plot_ground_truth(data)
    plot_trajectories_map(data)
    plot_altitude_profile(data)
    plot_coverage_rms(data)
    plot_merge_events(data)
    plot_loop_closures(data)
    plot_vertical_heatmap(data)
    save_animation(data)
    print('\nDone. Outputs in:', OUTPUT_DIR)
