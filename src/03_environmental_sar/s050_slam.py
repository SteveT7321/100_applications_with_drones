"""
S050 Swarm Cooperative Mapping (Distributed EKF-SLAM)
======================================================
Three drones explore a 200x200 m GPS-denied environment with 30 unknown
landmarks.  Each drone flies a *true* trajectory that differs from its
commanded odometry by process noise; range-bearing measurements are generated
from the true pose.  Three estimators process the SAME flight log
(true trajectories, odometry commands and measurements):

  * isolated     - one EKF-SLAM per drone, no communication
  * distributed  - one EKF-SLAM per drone; when two drones are within
                   COMM_RANGE they merge maps (robust consensus + SVD
                   alignment on >= 3 common landmarks, consistency check,
                   only landmarks the host lacks are transferred, with
                   inflated covariance); at most once per contact or every
                   MERGE_PERIOD while in contact
  * centralised  - one joint EKF over all three poses and all landmarks fed
                   with every measurement (upper bound, needs perfect comms)

Two alternative merge policies are evaluated on the same flight log for the
report only (MERGE_VARIANTS): 'dist_certainty' (the drone with the larger
position covariance adopts the other's frame) and 'dist_agree' (merges whose
alignment would shift the visitor map by more than MAX_FRAME_SHIFT are rejected).

The drones navigate with an information-gain frontier policy that steers on
each drone's own (isolated) on-board estimate, so the flight log does not
depend on the estimator being evaluated.

Usage:
    conda activate drones
    python src/03_environmental_sar/s050_slam.py
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import matplotlib.animation as animation

# ── Parameters ─────────────────────────────────────────────────────────────────
N_DRONES     = 3
AREA_SIZE    = 200.0      # m, square side
M_LANDMARKS  = 30         # true landmarks
DRONE_SPEED  = 3.0        # m/s
DT           = 0.2        # s
T_MAX        = 600.0      # s
N_STEPS      = int(round(T_MAX / DT))

SENSOR_RANGE = 25.0       # m
COMM_RANGE   = 40.0       # m
SIGMA_RHO    = 0.3        # m
SIGMA_PHI    = 0.03       # rad
SIGMA_AX     = 0.05       # m/s^2-equivalent: per-step position noise std = SIGMA_AX*DT
SIGMA_AY     = 0.05
SIGMA_OMEGA  = 0.02       # rad/s: per-step heading noise std = SIGMA_OMEGA*DT
P0_POSE      = 0.01       # initial pose covariance (x I3)
OMEGA_MAX    = 1.0        # rad/s turn-rate limit

MAHA_GATE    = 5.991      # chi2(2, 0.95): associate if d_M <= gate
NEW_LM_GATE  = 13.816     # chi2(2, 0.999): new landmark only if min d_M > this
MATCH_RADIUS = 2.0        # m - map-merge landmark correspondence radius
MIN_LM_SEP   = 4.0        # m - no new landmark closer than this to an existing one
MIN_COMMON   = 3          # min common landmarks for SVD alignment
MAX_FRAME_SHIFT = 1.0     # m - variant 'dist_agree': reject merge if alignment moves the visitor map more (RMS)
MERGE_VARIANTS = ('dist_certainty', 'dist_agree')   # alternative merge policies, same flight log
ALIGN_SEARCH_R = 20.0     # m - max map-to-map offset searched by the robust alignment
ALIGN_MAX_ROT = 0.5       # rad - max relative rotation between two drones' maps
MERGE_PERIOD = 30.0       # s - repeat merge while staying in contact
MERGE_INFLATE = 10.0      # covariance inflation of transferred landmarks (alignment error)
SENSED_MARGIN = 3.0       # m - host 'already sensed' area = within SENSOR_RANGE - margin of its path
SUCCESS_R    = 1.0        # m - landmark localisation success radius
LOOSE_R      = 5.0        # m - secondary 'landmark is in the map' radius
SNAPSHOT_DT  = 10.0       # s - quality snapshot interval
GRID_RES     = 5.0        # m - exploration grid
RETARGET_DT  = 2.0        # s - frontier target refresh interval
N_MC         = 10         # Monte Carlo noise seeds (0..N_MC-1)
SHOW_SEED    = 0
ANIM_FRAMES  = 120
LANDMARK_SEED = 42

STARTS = np.array([[20.0, 20.0, 0.0], [100.0, 20.0, np.pi / 2], [180.0, 20.0, np.pi]])
FILTER_INFLATE = 4.0     # filter uses 4x the true Q and R (EKF-SLAM consistency)
# true noise (used to simulate truth and measurements) is SIGMA_*; the filters use
R_OBS = FILTER_INFLATE * np.diag([SIGMA_RHO ** 2, SIGMA_PHI ** 2])
Q_POSE = FILTER_INFLATE * np.diag([(SIGMA_AX * DT) ** 2, (SIGMA_AY * DT) ** 2,
                                   (SIGMA_OMEGA * DT) ** 2])
STRATS = ['isolated', 'distributed', 'centralised']
STRAT_STYLE = {'isolated': ('#2980b9', '--'), 'distributed': ('#e74c3c', '-'),
               'centralised': ('#27ae60', '-.')}
DRONE_COLORS = ['#e74c3c', '#2980b9', '#27ae60']

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), '..', '..',
                          'outputs', '03_environmental_sar', 's050_slam')


def wrap(a):
    return np.arctan2(np.sin(a), np.cos(a))


# ── EKF-SLAM filter (handles 1 pose = local filter or 3 poses = centralised) ─────
class SlamFilter:
    def __init__(self, poses):
        self.n_pose = len(poses)
        self.X = np.concatenate([p.copy() for p in poses]).astype(float)
        self.P = np.eye(3 * self.n_pose) * P0_POSE
        self.lm0 = 3 * self.n_pose            # index of first landmark

    @property
    def n_lm(self):
        return (len(self.X) - self.lm0) // 2

    @property
    def landmarks(self):
        return self.X[self.lm0:].reshape(-1, 2)

    def lm_cov(self, j):
        l = self.lm0 + 2 * j
        return self.P[l:l + 2, l:l + 2]

    def pose(self, k):
        return self.X[3 * k:3 * k + 3]

    def predict(self, k, v, omega):
        p = 3 * k
        th = self.X[p + 2]
        F = np.array([[1, 0, -v * DT * np.sin(th)], [0, 1, v * DT * np.cos(th)], [0, 0, 1.0]])
        self.X[p] += v * DT * np.cos(th)
        self.X[p + 1] += v * DT * np.sin(th)
        self.X[p + 2] = wrap(th + omega * DT)
        self.P[p:p + 3, :] = F @ self.P[p:p + 3, :]
        self.P[:, p:p + 3] = self.P[:, p:p + 3] @ F.T
        self.P[p:p + 3, p:p + 3] += Q_POSE

    def _obs_blocks(self, k, js):
        """Predicted (rho, phi), sparse Jacobians (n,2,5) and index sets (n,5)."""
        x, y, th = self.pose(k)
        L = self.landmarks[js]
        dx, dy = L[:, 0] - x, L[:, 1] - y
        q = np.maximum(dx ** 2 + dy ** 2, 1e-12)
        rho = np.sqrt(q)
        phi = wrap(np.arctan2(dy, dx) - th)
        H = np.zeros((len(js), 2, 5))
        H[:, 0, 0] = -dx / rho; H[:, 0, 1] = -dy / rho
        H[:, 1, 0] = dy / q;    H[:, 1, 1] = -dx / q;   H[:, 1, 2] = -1.0
        H[:, 0, 3] = dx / rho;  H[:, 0, 4] = dy / rho
        H[:, 1, 3] = -dy / q;   H[:, 1, 4] = dx / q
        p = 3 * k
        l = self.lm0 + 2 * np.asarray(js)
        idx = np.stack([np.full_like(l, p), np.full_like(l, p + 1), np.full_like(l, p + 2),
                        l, l + 1], axis=1)
        return rho, phi, H, idx

    def associate(self, k, z):
        """Return (best landmark index or -1, its Mahalanobis distance)."""
        if self.n_lm == 0:
            return -1, np.inf
        x, y, _ = self.pose(k)
        d_e = np.linalg.norm(self.landmarks - [x, y], axis=1)
        js = np.flatnonzero(d_e < SENSOR_RANGE + 15.0)
        if len(js) == 0:
            return -1, np.inf
        rho, phi, H, idx = self._obs_blocks(k, js)
        Pb = self.P[idx[:, :, None], idx[:, None, :]]
        S = H @ Pb @ np.transpose(H, (0, 2, 1)) + R_OBS
        nu = np.stack([z[0] - rho, wrap(z[1] - phi)], axis=1)
        d = np.einsum('ni,ni->n', nu, np.linalg.solve(S, nu[:, :, None])[:, :, 0])
        b = int(np.argmin(d))
        return int(js[b]), float(d[b])

    def update(self, k, j, z):
        rho, phi, H, idx = self._obs_blocks(k, [j])
        H, idx = H[0], idx[0]
        S = H @ self.P[np.ix_(idx, idx)] @ H.T + R_OBS
        K = self.P[:, idx] @ H.T @ np.linalg.inv(S)
        nu = np.array([z[0] - rho[0], wrap(z[1] - phi[0])])
        self.X += K @ nu
        self.X[3 * k + 2] = wrap(self.X[3 * k + 2])
        self.P -= K @ S @ K.T
        self.P = 0.5 * (self.P + self.P.T)

    def add_landmark_from_obs(self, k, z):
        """Initialise a landmark with full cross-covariance to the existing state."""
        p = 3 * k
        x, y, th = self.pose(k)
        a = th + z[1]
        lm = np.array([x + z[0] * np.cos(a), y + z[0] * np.sin(a)])
        Jx = np.array([[1, 0, -z[0] * np.sin(a)], [0, 1, z[0] * np.cos(a)]])
        Jz = np.array([[np.cos(a), -z[0] * np.sin(a)], [np.sin(a), z[0] * np.cos(a)]])
        n = len(self.X)
        Pn = np.zeros((n + 2, n + 2))
        Pn[:n, :n] = self.P
        cross = Jx @ self.P[p:p + 3, :]
        Pn[n:, :n] = cross
        Pn[:n, n:] = cross.T
        Pn[n:, n:] = Jx @ self.P[p:p + 3, p:p + 3] @ Jx.T + Jz @ R_OBS @ Jz.T
        self.X = np.concatenate([self.X, lm])
        self.P = Pn

    def add_landmark(self, lm, cov):
        n = len(self.X)
        Pn = np.zeros((n + 2, n + 2))
        Pn[:n, :n] = self.P
        Pn[n:, n:] = cov
        self.X = np.concatenate([self.X, lm])
        self.P = Pn

    def to_frame(self, R, t):
        """Re-express the whole state in another frame: p' = R^T (p - t),
        theta' = theta - angle(R); covariance J P J^T (rigid, no information added)."""
        alpha = np.arctan2(R[1, 0], R[0, 0])
        J = np.eye(len(self.X))
        for k in range(self.n_pose):
            p = 3 * k
            self.X[p:p + 2] = R.T @ (self.X[p:p + 2] - t)
            self.X[p + 2] = wrap(self.X[p + 2] - alpha)
            J[p:p + 2, p:p + 2] = R.T
        for a in range(self.n_lm):
            l = self.lm0 + 2 * a
            self.X[l:l + 2] = R.T @ (self.X[l:l + 2] - t)
            J[l:l + 2, l:l + 2] = R.T
        self.P = J @ self.P @ J.T
        self.P = 0.5 * (self.P + self.P.T)

    def pos_uncertainty(self, k=0):
        """Trace of the position covariance of pose k (m^2)."""
        return float(np.trace(self.P[3 * k:3 * k + 2, 3 * k:3 * k + 2]))

    def process(self, k, z):
        """Data association + update / initialise for one measurement."""
        j, d = self.associate(k, z)
        if j >= 0 and d <= MAHA_GATE:
            self.update(k, j, z)
        elif d > NEW_LM_GATE:
            x, y, th = self.pose(k)
            cand = np.array([x + z[0] * np.cos(th + z[1]), y + z[0] * np.sin(th + z[1])])
            if self.n_lm == 0 or np.min(np.linalg.norm(self.landmarks - cand, axis=1)) >= MIN_LM_SEP:
                self.add_landmark_from_obs(k, z)
        # otherwise ambiguous: measurement discarded


def _one_to_one(A, B, radius):
    """Greedy one-to-one nearest pairs (index in A, index in B) closer than radius."""
    if len(A) == 0 or len(B) == 0:
        return []
    D = np.linalg.norm(A[:, None, :] - B[None, :, :], axis=2)
    pairs, used_a, used_b = [], set(), set()
    for flat in np.argsort(D, axis=None):
        i, j = np.unravel_index(flat, D.shape)
        if D[i, j] >= radius:
            break
        if i in used_a or j in used_b:
            continue
        pairs.append((int(i), int(j))); used_a.add(i); used_b.add(j)
    return pairs


def _svd_align(Lh, Lv):
    """Least-squares rigid transform (R, t) with Lh ~ R Lv + t (card's SVD solution)."""
    mu_h, mu_v = Lh.mean(0), Lv.mean(0)
    U, _, Vt = np.linalg.svd((Lh - mu_h).T @ (Lv - mu_v))
    D = np.diag([1.0, np.sign(np.linalg.det(U @ Vt))])
    R = U @ D @ Vt
    return R, mu_h - R @ mu_v


def robust_align(Lh, Lv):
    """
    Deterministic consensus alignment of two drifted maps.  Hypotheses come
    from two correspondences (a<->b, c<->d) that lie within ALIGN_SEARCH_R of each
    other and have consistent inter-landmark distances (|Dh_ac - Dv_bd| < 1 m);
    rotation limited to ALIGN_MAX_ROT.  The hypothesis with most one-to-one
    inliers (< MATCH_RADIUS) wins and is refined by SVD on its inliers.
    Returns (R, t, inlier pairs).
    """
    Dc = np.linalg.norm(Lh[:, None, :] - Lv[None, :, :], axis=2)
    cand = np.argwhere(Dc < ALIGN_SEARCH_R)
    Dh = np.linalg.norm(Lh[:, None] - Lh[None], axis=2)
    Dv = np.linalg.norm(Lv[:, None] - Lv[None], axis=2)
    best = (0, np.inf, np.eye(2), np.zeros(2))
    for a, b in cand:
        ok = ((cand[:, 0] != a) & (cand[:, 1] != b)
              & (np.abs(Dh[a, cand[:, 0]] - Dv[b, cand[:, 1]]) < 1.0)
              & (Dh[a, cand[:, 0]] > 3.0))
        for c, d in cand[ok]:
            ang = (np.arctan2(*(Lh[c] - Lh[a])[::-1]) - np.arctan2(*(Lv[d] - Lv[b])[::-1]))
            ang = wrap(ang)
            if abs(ang) > ALIGN_MAX_ROT:
                continue
            R = np.array([[np.cos(ang), -np.sin(ang)], [np.sin(ang), np.cos(ang)]])
            t = Lh[a] - R @ Lv[b]
            Lt = Lv @ R.T + t
            dmin = np.linalg.norm(Lh[:, None] - Lt[None], axis=2).min(0)
            inl = dmin < MATCH_RADIUS
            score = (int(inl.sum()), float(np.sum(dmin[inl] ** 2)))
            if score[0] > best[0] or (score[0] == best[0] and score[1] < best[1]):
                best = (score[0], score[1], R, t)
    if best[0] < MIN_COMMON:
        return None, None, []
    R, t = best[2], best[3]
    pairs = _one_to_one(Lh, Lv @ R.T + t, MATCH_RADIUS)
    if len(pairs) >= MIN_COMMON:
        R, t = _svd_align(Lh[[i for i, _ in pairs]], Lv[[j for _, j in pairs]])
        pairs = _one_to_one(Lh, Lv @ R.T + t, MATCH_RADIUS)
    return R, t, pairs


def _snapshot(f):
    """(landmarks, 2x2 covariances) copy of a filter's map."""
    return (f.landmarks.copy(),
            np.array([f.lm_cov(a) for a in range(f.n_lm)]).reshape(-1, 2, 2))


def merge_maps(host, vis_L, vis_C, host_sensed=None, check_shift=False, adopt=False):
    """
    Merge a visitor map snapshot (landmarks vis_L, 2x2 covariances vis_C) into
    host, transferring only NEW information:
      1. robust alignment of the visitor map onto the host map;
      2. common landmarks are used only for the alignment and a consistency
         check (chi2(2, 0.999) on the combined covariance); if fewer than
         MIN_COMMON or fewer than half of them pass, the merge is rejected;
      3. visitor landmarks the host does not have are appended with covariance
         MERGE_INFLATE x (rotated visitor covariance) and no cross-correlation,
         unless they are within MIN_LM_SEP of a host landmark, statistically
         compatible with one, or inside the area the host has sensed itself.
    Variants (evaluated alongside, see MERGE_VARIANTS):
      check_shift - also reject the merge if the alignment moves the visitor map
                    by more than MAX_FRAME_SHIFT (RMS), i.e. the frames disagree;
      adopt       - the host (chosen by the caller as the drone with the larger
                    position covariance) first re-expresses its whole state in
                    the visitor's frame, then imports the new landmarks.
    Host landmark estimates and the host pose are not modified (fusing landmarks
    that both maps already contain double-counts information exchanged in
    earlier merges and made the filters overconfident).
    Returns (n_common, n_consistent, n_added); n_consistent < 0 marks a rejected merge.
    """
    if len(vis_L) == 0 or host.n_lm == 0:
        return 0, 0, 0
    R, t, pairs = robust_align(host.landmarks, vis_L)
    if len(pairs) < MIN_COMMON:
        return len(pairs), 0, 0
    Lt = vis_L @ R.T + t
    Ct = np.einsum('ij,njk,lk->nil', R, vis_C, R)
    # statistical consistency of every geometric match
    good = []
    for i, j in pairs:
        dlt = Lt[j] - host.landmarks[i]
        d2 = float(dlt @ np.linalg.solve(host.lm_cov(i) + MERGE_INFLATE * Ct[j], dlt))
        if d2 <= NEW_LM_GATE:
            good.append((i, j))
    if len(good) < MIN_COMMON or len(good) < 0.5 * len(pairs):
        return len(pairs), -len(good), 0      # maps disagree: merge rejected
    if check_shift:
        shift = float(np.sqrt(np.mean(np.sum((Lt - vis_L) ** 2, axis=1))))
        if shift > MAX_FRAME_SHIFT:
            return len(pairs), -len(good), 0  # frames disagree (drift): merge rejected
    L_old = Lt.copy()                         # visitor landmarks in the host's (old) frame
    if adopt:
        host.to_frame(R, t)
        Lt = vis_L.copy()
        Ct = vis_C.copy()
    matched_v = {j for _, j in pairs}
    n_added = 0
    for j in range(len(Lt)):
        if j in matched_v:
            continue
        diff = host.landmarks - Lt[j]
        if np.min(np.linalg.norm(diff, axis=1)) < MIN_LM_SEP:
            continue
        if host_sensed is not None:           # host_sensed is in the host's old frame
            ix = int(np.clip(L_old[j][0] // GRID_RES, 0, W_CELLS - 1))
            iy = int(np.clip(L_old[j][1] // GRID_RES, 0, W_CELLS - 1))
            if host_sensed[iy, ix]:
                continue      # host has sensed this area: landmark would be a duplicate
        # skip if statistically compatible with an existing host landmark
        S = np.array([host.lm_cov(i) for i in range(host.n_lm)]) + MERGE_INFLATE * Ct[j]
        d2 = np.einsum('ni,ni->n', diff, np.linalg.solve(S, diff[:, :, None])[:, :, 0])
        if d2.min() <= NEW_LM_GATE:
            continue
        host.add_landmark(Lt[j], MERGE_INFLATE * Ct[j])
        n_added += 1
    return len(pairs), len(good), n_added


# ── Frontier policy ────────────────────────────────────────────────────────────
W_CELLS = int(AREA_SIZE / GRID_RES)
CELL_C = (np.arange(W_CELLS) + 0.5) * GRID_RES
R_CELLS = int(SENSOR_RANGE / GRID_RES)


def mark_explored(grid, pos, radius=SENSOR_RANGE):
    d = np.hypot(CELL_C[None, :] - pos[0], CELL_C[:, None] - pos[1])
    grid |= d <= radius


def frontier_target(grid, pos, other_targets):
    """argmax IG / distance over unexplored cells; IG = unexplored cells in the
    (2r+1)^2 square around the candidate (integral image)."""
    unexp = ~grid
    if not unexp.any():
        return None
    I = np.pad(unexp.astype(np.int32).cumsum(0).cumsum(1), ((1, 0), (1, 0)))
    r = R_CELLS
    lo = np.clip(np.arange(W_CELLS) - r, 0, W_CELLS)
    hi = np.clip(np.arange(W_CELLS) + r + 1, 0, W_CELLS)
    IG = (I[hi[:, None], hi[None, :]] - I[lo[:, None], hi[None, :]]
          - I[hi[:, None], lo[None, :]] + I[lo[:, None], lo[None, :]]).astype(float)
    cx, cy = np.meshgrid(CELL_C, CELL_C)
    for ot in other_targets:
        if ot is not None:
            IG[np.hypot(cx - ot[0], cy - ot[1]) < SENSOR_RANGE] *= 0.1
    dist = np.hypot(cx - pos[0], cy - pos[1])
    score = np.where(unexp, IG / (dist + GRID_RES), -1.0)
    iy, ix = np.unravel_index(int(np.argmax(score)), score.shape)
    return np.array([CELL_C[ix], CELL_C[iy]])


# ── Metrics ────────────────────────────────────────────────────────────────────
def map_quality(L, true_lm):
    """Per true landmark: found if an estimate lies within SUCCESS_R."""
    if len(L) == 0:
        return dict(cov=0.0, cov_loose=0.0, rms=np.nan, n_found=0, n_est=0, n_dup=0)
    D = np.linalg.norm(true_lm[:, None, :] - L[None, :, :], axis=2)
    dmin = D.min(axis=1)
    found = dmin < SUCCESS_R
    nearest_true = D.argmin(axis=0)
    n_dup = len(L) - len(np.unique(nearest_true))
    return dict(cov=found.mean(), cov_loose=float(np.mean(dmin < LOOSE_R)), rms=float(np.sqrt(np.mean(dmin[found] ** 2))) if found.any()
                else np.nan, n_found=int(found.sum()), n_est=len(L), n_dup=int(n_dup))


def team_union(Ls, true_lm):
    found = np.zeros(M_LANDMARKS, dtype=bool)
    for L in Ls:
        if len(L):
            found |= np.linalg.norm(true_lm[:, None] - L[None], axis=2).min(1) < SUCCESS_R
    return found.mean()


def min_eig(P):
    return float(np.linalg.eigvalsh(0.5 * (P + P.T)).min())


# ── Simulation ─────────────────────────────────────────────────────────────────
def run_simulation(seed=SHOW_SEED, record=True):
    """Generate one flight log and run the three estimators on it."""
    true_lm = np.random.default_rng(LANDMARK_SEED).uniform(10.0, 190.0, size=(M_LANDMARKS, 2))
    rng = np.random.default_rng(seed)

    truth = STARTS.copy()
    dead_reck = STARTS.copy()
    iso = [SlamFilter([STARTS[k]]) for k in range(N_DRONES)]
    dist = [SlamFilter([STARTS[k]]) for k in range(N_DRONES)]
    var = {v: [SlamFilter([STARTS[k]]) for k in range(N_DRONES)] for v in MERGE_VARIANTS}
    var_sensed = {v: [np.zeros((W_CELLS, W_CELLS), dtype=bool) for _ in range(N_DRONES)]
                  for v in MERGE_VARIANTS}
    var_events = {v: [] for v in MERGE_VARIANTS}
    cen = SlamFilter([STARTS[k] for k in range(N_DRONES)])
    grids = [np.zeros((W_CELLS, W_CELLS), dtype=bool) for _ in range(N_DRONES)]
    sensed = [np.zeros((W_CELLS, W_CELLS), dtype=bool) for _ in range(N_DRONES)]
    targets = [None] * N_DRONES
    in_contact = np.zeros((N_DRONES, N_DRONES), dtype=bool)
    last_merge = np.full((N_DRONES, N_DRONES), -np.inf)
    comm_events = []
    min_eig_seen = np.inf

    snaps = {'t': [], **{s: {'cov': [], 'rms': [], 'pose': [], 'n_est': []} for s in STRATS}}
    true_hist = [[] for _ in range(N_DRONES)]
    est_hist = {s: [[] for _ in range(N_DRONES)] for s in STRATS}
    frames = []
    pose_err = {s: [] for s in STRATS + ['dead_reckoning']}
    frame_every = max(1, N_STEPS // ANIM_FRAMES)

    def est_pose(s, k):
        return {'isolated': iso[k].pose(0), 'distributed': dist[k].pose(0),
                'centralised': cen.pose(k)}[s]

    for step in range(N_STEPS):
        t = (step + 1) * DT
        for k in range(N_DRONES):
            # navigation on the drone's own on-board (isolated) estimate
            nav = iso[k].pose(0)
            mark_explored(grids[k], nav)
            mark_explored(sensed[k], dist[k].pose(0), SENSOR_RANGE - SENSED_MARGIN)
            for v in MERGE_VARIANTS:
                mark_explored(var_sensed[v][k], var[v][k].pose(0), SENSOR_RANGE - SENSED_MARGIN)
            if (targets[k] is None or step % int(RETARGET_DT / DT) == 0
                    or np.hypot(*(targets[k] - nav[:2])) < GRID_RES):
                targets[k] = frontier_target(grids[k], nav,
                                             [targets[j] for j in range(N_DRONES) if j != k])
                if targets[k] is None:            # own map fully explored: start again
                    grids[k][:] = False
                    targets[k] = frontier_target(grids[k], nav, [])
            ang = np.arctan2(targets[k][1] - nav[1], targets[k][0] - nav[0])
            omega = float(np.clip(wrap(ang - nav[2]) / DT, -OMEGA_MAX, OMEGA_MAX))

            # true motion = commanded odometry + process noise
            th = truth[k, 2]
            truth[k] += [DRONE_SPEED * DT * np.cos(th) + rng.normal(0, SIGMA_AX * DT),
                         DRONE_SPEED * DT * np.sin(th) + rng.normal(0, SIGMA_AY * DT),
                         omega * DT + rng.normal(0, SIGMA_OMEGA * DT)]
            truth[k, 2] = wrap(truth[k, 2])
            thd = dead_reck[k, 2]
            dead_reck[k] += [DRONE_SPEED * DT * np.cos(thd), DRONE_SPEED * DT * np.sin(thd),
                             omega * DT]
            iso[k].predict(0, DRONE_SPEED, omega)
            dist[k].predict(0, DRONE_SPEED, omega)
            for v in MERGE_VARIANTS:
                var[v][k].predict(0, DRONE_SPEED, omega)
            cen.predict(k, DRONE_SPEED, omega)

            # measurements from the TRUE pose
            d_true = np.linalg.norm(true_lm - truth[k, :2], axis=1)
            for j in np.flatnonzero(d_true <= SENSOR_RANGE):
                z = np.array([d_true[j] + rng.normal(0, SIGMA_RHO),
                              wrap(np.arctan2(true_lm[j, 1] - truth[k, 1],
                                              true_lm[j, 0] - truth[k, 0]) - truth[k, 2]
                                   + rng.normal(0, SIGMA_PHI))])
                iso[k].process(0, z)
                dist[k].process(0, z)
                for v in MERGE_VARIANTS:
                    var[v][k].process(0, z)
                cen.process(k, z)

        # pairwise map merging (distributed), true inter-drone distance
        for i in range(N_DRONES):
            for j in range(i + 1, N_DRONES):
                close = np.hypot(*(truth[i, :2] - truth[j, :2])) <= COMM_RANGE
                if close and (not in_contact[i, j] or t - last_merge[i, j] >= MERGE_PERIOD - 1e-9):
                    snap_i = (dist[i].landmarks.copy(),
                              np.array([dist[i].lm_cov(a) for a in range(dist[i].n_lm)]).reshape(-1, 2, 2))
                    snap_j = (dist[j].landmarks.copy(),
                              np.array([dist[j].lm_cov(a) for a in range(dist[j].n_lm)]).reshape(-1, 2, 2))
                    r_ij = merge_maps(dist[i], *snap_j, sensed[i])
                    r_ji = merge_maps(dist[j], *snap_i, sensed[j])
                    last_merge[i, j] = t
                    comm_events.append((t, i, j, max(r_ij[0], r_ji[0]), r_ij[2] + r_ji[2],
                                        r_ij[1] > 0 or r_ji[1] > 0,
                                        r_ij[1] < 0 or r_ji[1] < 0))
                    # variant: frame-agreement gate, both directions
                    F, S_ = var['dist_agree'], var_sensed['dist_agree']
                    si, sj = _snapshot(F[i]), _snapshot(F[j])
                    ra = merge_maps(F[i], *sj, S_[i], check_shift=True)
                    rb = merge_maps(F[j], *si, S_[j], check_shift=True)
                    var_events['dist_agree'].append((t, i, j, ra[1] > 0 or rb[1] > 0))
                    # variant: less certain drone adopts the other's frame (one direction)
                    F, S_ = var['dist_certainty'], var_sensed['dist_certainty']
                    rcv, src = (i, j) if F[i].pos_uncertainty() >= F[j].pos_uncertainty() else (j, i)
                    rc = merge_maps(F[rcv], *_snapshot(F[src]), S_[rcv], adopt=True)
                    var_events['dist_certainty'].append((t, i, j, rc[1] > 0))
                in_contact[i, j] = close

        if step % 50 == 0:
            for f in dist + iso + [cen] + var['dist_certainty'] + var['dist_agree']:
                min_eig_seen = min(min_eig_seen, min_eig(f.P))

        if record:
            for k in range(N_DRONES):
                for s in STRATS:
                    est_hist[s][k].append(est_pose(s, k)[:2].copy())
                true_hist[k].append(truth[k, :2].copy())

        if abs(t / SNAPSHOT_DT - round(t / SNAPSHOT_DT)) < 1e-9:
            snaps['t'].append(t)
            maps = {'isolated': [f.landmarks for f in iso],
                    'distributed': [f.landmarks for f in dist],
                    'centralised': [cen.landmarks] * N_DRONES}
            for s in STRATS:
                q = [map_quality(L, true_lm) for L in maps[s]]
                snaps[s]['cov'].append(np.mean([x['cov'] for x in q]))
                snaps[s]['rms'].append(np.nanmean([x['rms'] for x in q]) if any(
                    not np.isnan(x['rms']) for x in q) else np.nan)
                snaps[s]['n_est'].append(np.mean([x['n_est'] for x in q]))
                snaps[s]['pose'].append(np.sqrt(np.mean(
                    [np.sum((est_pose(s, k)[:2] - truth[k, :2]) ** 2) for k in range(N_DRONES)])))
            for s in STRATS:
                pose_err[s].append(snaps[s]['pose'][-1])
            pose_err['dead_reckoning'].append(np.sqrt(np.mean(
                np.sum((dead_reck[:, :2] - truth[:, :2]) ** 2, axis=1))))

        if record and step % frame_every == 0:
            frames.append((t, truth[:, :2].copy(), [f.landmarks.copy() for f in dist],
                           [[f.lm_cov(a).copy() for a in range(f.n_lm)] for f in dist],
                           [(e[1], e[2]) for e in comm_events if t - 2.0 < e[0] <= t]))

    for f in dist + iso + [cen] + var['dist_certainty'] + var['dist_agree']:
        min_eig_seen = min(min_eig_seen, min_eig(f.P))

    final = {}
    for v in MERGE_VARIANTS:
        q = [map_quality(f.landmarks, true_lm) for f in var[v]]
        final[v] = dict(
            cov=float(np.mean([x['cov'] for x in q])),
            cov_loose=float(np.mean([x['cov_loose'] for x in q])),
            rms=float(np.nanmean([x['rms'] for x in q])) if any(x['n_found'] for x in q) else np.nan,
            n_est=[x['n_est'] for x in q], n_found=[x['n_found'] for x in q],
            n_dup=[x['n_dup'] for x in q],
            union=team_union([f.landmarks for f in var[v]], true_lm),
            pose_final=float(np.sqrt(np.mean([np.sum((var[v][k].pose(0)[:2] - truth[k, :2]) ** 2)
                                              for k in range(N_DRONES)]))),
            merges=int(sum(e[3] for e in var_events[v])))
    maps = {'isolated': [f.landmarks for f in iso], 'distributed': [f.landmarks for f in dist],
            'centralised': [cen.landmarks]}
    for s in STRATS:
        q = [map_quality(L, true_lm) for L in maps[s]]
        final[s] = dict(
            cov=float(np.mean([x['cov'] for x in q])),
            cov_loose=float(np.mean([x['cov_loose'] for x in q])),
            rms=float(np.nanmean([x['rms'] for x in q])) if any(x['n_found'] for x in q) else np.nan,
            n_est=[x['n_est'] for x in q], n_found=[x['n_found'] for x in q],
            n_dup=[x['n_dup'] for x in q], union=team_union(maps[s], true_lm),
            pose_final=pose_err[s][-1], pose_mean=float(np.mean(pose_err[s])))
    final['dead_reckoning'] = dict(pose_final=pose_err['dead_reckoning'][-1],
                                   pose_mean=float(np.mean(pose_err['dead_reckoning'])))
    return dict(true_lm=true_lm, iso=iso, dist=dist, cen=cen, snaps=snaps, final=final,
                comm_events=comm_events, true_hist=true_hist, est_hist=est_hist,
                frames=frames, min_eig=min_eig_seen, seed=seed)


# ── Plots ───────────────────────────────────────────────────────────────────────
def _ellipse(ax, mean, cov, n_std, **kw):
    vals, vecs = np.linalg.eigh(0.5 * (cov + cov.T))
    vals = np.clip(vals, 0, None)
    ang = np.degrees(np.arctan2(vecs[1, 1], vecs[0, 1]))
    ax.add_patch(Ellipse(mean, 2 * n_std * np.sqrt(vals[1]), 2 * n_std * np.sqrt(vals[0]),
                         angle=ang, **kw))


def _save(fig, out_dir, name):
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, name)
    fig.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_trajectories_map(res, out_dir):
    """True (solid) vs estimated (dotted) paths and drone-1 / central map per strategy."""
    fig, axes = plt.subplots(1, 3, figsize=(20, 7))
    true_lm = res['true_lm']
    maps = {'isolated': res['iso'][0], 'distributed': res['dist'][0], 'centralised': res['cen']}
    titles = {'isolated': 'Isolated EKF-SLAM (map of drone 1)',
              'distributed': 'Distributed EKF-SLAM (map of drone 1)',
              'centralised': 'Centralised EKF-SLAM (shared map)'}
    for ax, s in zip(axes, STRATS):
        ax.set_aspect('equal'); ax.set_xlim(-5, AREA_SIZE + 5); ax.set_ylim(-5, AREA_SIZE + 5)
        ax.scatter(true_lm[:, 0], true_lm[:, 1], c='k', marker='o', s=25, label='True landmark')
        for k in range(N_DRONES):
            tr = np.array(res['true_hist'][k]); es = np.array(res['est_hist'][s][k])
            ax.plot(tr[:, 0], tr[:, 1], '-', color=DRONE_COLORS[k], lw=0.8, alpha=0.6,
                    label=f'D{k+1} true path')
            ax.plot(es[:, 0], es[:, 1], ':', color=DRONE_COLORS[k], lw=0.8, alpha=0.9)
        f = maps[s]
        L = f.landmarks
        ax.scatter(L[:, 0], L[:, 1], marker='+', c='purple', s=60, label='Estimate')
        for a in range(f.n_lm):
            _ellipse(ax, L[a], f.lm_cov(a), 3.0, edgecolor='purple', facecolor='none', lw=0.6)
        q = map_quality(L, true_lm)
        ax.set_title(f'{titles[s]}\n{q["n_found"]}/{M_LANDMARKS} found, {q["n_est"]} estimates, '
                     f'RMS {q["rms"]:.3f} m', fontsize=10)
        ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
    axes[0].legend(fontsize=7, loc='upper right')
    fig.suptitle(f'S050: true (solid) vs estimated (dotted) paths; 3σ landmark ellipses '
                 f'(seed {res["seed"]})', fontsize=12)
    _save(fig, out_dir, 'trajectories_map.png')


def plot_coverage_rms(res, mc, out_dir):
    """Coverage, landmark RMS and pose RMSE vs time; Monte Carlo min/max bands."""
    fig, axes = plt.subplots(3, 1, figsize=(10, 11), sharex=True)
    t = np.array(res['snaps']['t'])
    for s in STRATS:
        c, ls = STRAT_STYLE[s]
        for ax, key, scale in ((axes[0], 'cov', 100.0), (axes[1], 'rms', 1.0), (axes[2], 'pose', 1.0)):
            arr = np.array([m['snaps'][s][key] for m in mc], dtype=float) * scale
            ax.fill_between(t, np.nanmin(arr, 0), np.nanmax(arr, 0), color=c, alpha=0.15)
            ax.plot(t, np.nanmean(arr, 0), ls, color=c, lw=2, label=f'{s} (MC mean)')
    axes[0].set_ylabel('Coverage (%)\nmean per-drone map'); axes[0].set_ylim(0, 105)
    axes[1].set_ylabel('Landmark RMS (m)\nfound landmarks')
    axes[2].set_ylabel('Pose RMSE (m)')
    axes[2].set_xlabel('Time (s)')
    for ev in res['comm_events']:
        if ev[5]:
            axes[0].axvline(ev[0], color='gray', alpha=0.25, lw=0.8)  # executed merges
    axes[0].set_title(f'S050: Map quality vs time ({N_MC} Monte Carlo seeds, bands = min/max; '
                      f'grey lines = merges in seed {res["seed"]})', fontsize=11)
    for ax in axes:
        ax.grid(True, alpha=0.3); ax.legend(fontsize=8)
    _save(fig, out_dir, 'coverage_rms.png')


def plot_final_map_heatmap(res, out_dir):
    """Final distributed map of drone 1: landmark uncertainty sqrt(trace P_ll)."""
    f = res['dist'][0]
    true_lm = res['true_lm']
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_aspect('equal'); ax.set_xlim(0, AREA_SIZE); ax.set_ylim(0, AREA_SIZE)
    L = f.landmarks
    u = np.array([np.sqrt(np.trace(f.lm_cov(a))) for a in range(f.n_lm)])
    sc = ax.scatter(L[:, 0], L[:, 1], c=u, cmap='RdYlBu_r', s=160, marker='P',
                    edgecolors='k', linewidths=0.5, zorder=6)
    plt.colorbar(sc, ax=ax, shrink=0.7, label='sqrt(trace P_ll) (m)')
    D = np.linalg.norm(true_lm[:, None] - L[None], axis=2) if len(L) else np.full((M_LANDMARKS, 1), np.inf)
    missing = D.min(1) >= SUCCESS_R
    ax.scatter(true_lm[~missing, 0], true_lm[~missing, 1], facecolors='none', edgecolors='k',
               s=60, zorder=5, label='True landmark (found)')
    ax.scatter(true_lm[missing, 0], true_lm[missing, 1], c='gray', marker='x', s=60, zorder=5,
               label=f'True landmark not found ({int(missing.sum())})')
    ax.set_title('S050: Final distributed map of drone 1 — landmark uncertainty', fontsize=12)
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.legend(loc='upper right', fontsize=8)
    _save(fig, out_dir, 'final_map_heatmap.png')


def plot_communication_events(res, out_dir):
    """Merge events annotated with the number of common landmarks."""
    fig, ax = plt.subplots(figsize=(12, 3.5))
    cols = {(0, 1): '#e74c3c', (0, 2): '#27ae60', (1, 2): '#2980b9'}
    for (t, i, j, nc, nadd, ok, rej) in res['comm_events']:
        ax.vlines(t, 0, nc, color=cols[(i, j)], lw=2, alpha=0.9 if ok else 0.3,
                  linestyles='-' if not rej else ':')
        ax.text(t, nc + 0.5, f'{nc}/+{nadd}' if ok else ('rej' if rej else str(nc)),
                fontsize=7, ha='center')
    ax.axhline(MIN_COMMON, color='k', ls=':', lw=1)
    handles = [plt.Line2D([], [], color=c, lw=2, label=f'D{a+1}-D{b+1}') for (a, b), c in cols.items()]
    handles.append(plt.Line2D([], [], color='k', ls=':', lw=1, label=f'min common = {MIN_COMMON}'))
    ax.set_xlim(0, T_MAX); ax.set_xlabel('Time (s)'); ax.set_ylabel('Common landmarks')
    ax.set_title('S050: Map-merge attempts — label common/+added; faded = no merge '
                 '(< 3 common or rejected as inconsistent)', fontsize=10)
    ax.legend(handles=handles, fontsize=8, loc='upper left'); ax.grid(alpha=0.3)
    _save(fig, out_dir, 'comm_events.png')


def save_animation(res, out_dir):
    """Drones (true positions), distributed landmark estimates with 3-sigma ellipses
    as they evolve, communication links when a merge happens."""
    frames = res['frames']
    true_lm = res['true_lm']
    fig, ax = plt.subplots(figsize=(7, 7))

    def update(i):
        ax.clear()
        t, pos, Ls, Cs, links = frames[i]
        ax.set_aspect('equal'); ax.set_xlim(0, AREA_SIZE); ax.set_ylim(0, AREA_SIZE)
        ax.scatter(true_lm[:, 0], true_lm[:, 1], c='k', s=15, alpha=0.5)
        for k in range(N_DRONES):
            if len(Ls[k]):
                ax.scatter(Ls[k][:, 0], Ls[k][:, 1], marker='+', color=DRONE_COLORS[k], s=40)
                for a in range(len(Ls[k])):
                    _ellipse(ax, Ls[k][a], Cs[k][a], 3.0, edgecolor=DRONE_COLORS[k],
                             facecolor='none', lw=0.5)
            ax.plot(*pos[k], '^', color=DRONE_COLORS[k], ms=10, markeredgecolor='k')
            ax.add_patch(plt.Circle(pos[k], SENSOR_RANGE, fill=False, color=DRONE_COLORS[k],
                                    lw=0.5, ls=':'))
        for (a, b) in links:
            ax.plot([pos[a, 0], pos[b, 0]], [pos[a, 1], pos[b, 1]], '-', color='gold', lw=2.5)
        n = [len(L) for L in Ls]
        ax.set_title(f'S050 distributed EKF-SLAM  t = {t:.0f} s  landmarks per drone {n}',
                     fontsize=10)
        return []

    ani = animation.FuncAnimation(fig, update, frames=len(frames), blit=False)
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer='pillow', fps=5, dpi=80)
    plt.close(fig)
    print(f'Saved: {path} ({len(frames)} frames)')


# ── Main ───────────────────────────────────────────────────────────────────────
def _print_final(res, label):
    f = res['final']
    print(f'\n=== {label} ===')
    for s in STRATS:
        x = f[s]
        print(f'  {s:12s}: coverage (mean per-drone map) {x["cov"]*100:.1f}% '
              f'(within {LOOSE_R:.0f} m: {x["cov_loose"]*100:.1f}%) | '
              f'team union {x["union"]*100:.1f}% | RMS {x["rms"]:.3f} m | estimates/map '
              f'{x["n_est"]} | found {x["n_found"]} | duplicates {x["n_dup"]} | pose RMSE '
              f'final {x["pose_final"]:.3f} m, mean {x["pose_mean"]:.3f} m')
    for v in MERGE_VARIANTS:
        x = f[v]
        print(f'  {v:12s}: coverage (mean per-drone map) {x["cov"]*100:.1f}% '
              f'(within {LOOSE_R:.0f} m: {x["cov_loose"]*100:.1f}%) | estimates/map {x["n_est"]} | '
              f'found {x["n_found"]} | pose RMSE final {x["pose_final"]:.3f} m | '
              f'executed merges {x["merges"]}')
    print(f'  dead reckoning: pose RMSE final {f["dead_reckoning"]["pose_final"]:.3f} m, '
          f'mean {f["dead_reckoning"]["pose_mean"]:.3f} m')
    ev = res['comm_events']
    ok = [e for e in ev if e[5]]
    print(f'  merge attempts {len(ev)}, executed merges {len(ok)}, rejected (inconsistent / frames disagree) '
          f'{sum(1 for e in ev if e[6])}, landmarks transferred {sum(e[4] for e in ok)}')
    print(f'  smallest covariance eigenvalue seen (all filters): {res["min_eig"]:.3e}')


if __name__ == '__main__':
    out_dir = os.path.normpath(OUTPUT_DIR)
    print(f'Running S050 swarm EKF-SLAM (showcase seed {SHOW_SEED})...')
    res = run_simulation(SHOW_SEED, record=True)
    # seed-0 mechanism evidence: drone 1 pose error, isolated vs distributed
    th = np.array(res['true_hist'][0])
    ei = np.linalg.norm(np.array(res['est_hist']['isolated'][0]) - th, axis=1)
    ed = np.linalg.norm(np.array(res['est_hist']['distributed'][0]) - th, axis=1)
    e3 = np.linalg.norm(np.array(res['est_hist']['isolated'][2]) - np.array(res['true_hist'][2]), axis=1)
    print('  D1 position error (m), isolated / distributed:')
    for tt in (100.0, 160.0, 166.6, 170.0, 200.0, 300.0, 400.0, 500.0, 600.0):
        n = int(round(tt / DT)) - 1
        print(f'    t={tt:5.1f} s: {ei[n]:.2f} / {ed[n]:.2f}')
    print(f'  D3 isolated position error at t=166.6 s: {e3[int(round(166.6 / DT)) - 1]:.2f} m')
    for lab, f in (('isolated', res['iso'][0]), ('distributed', res['dist'][0])):
        L = f.landmarks
        nearest = res['true_lm'][np.linalg.norm(res['true_lm'][:, None] - L[None], axis=2).argmin(0)]
        off = np.mean(L - nearest, axis=0)
        print(f'  D1 {lab} final map: mean offset (estimate - nearest true) = '
              f'({off[0]:+.2f}, {off[1]:+.2f}) m')
    tl = res['true_lm']
    dmin = np.min(np.linalg.norm(tl[:, None] - tl[None], axis=2) + 1e9 * np.eye(M_LANDMARKS))
    print(f'  closest pair of true landmarks: {dmin:.2f} m (MIN_LM_SEP = {MIN_LM_SEP} m)')
    _print_final(res, f'Showcase run, seed {SHOW_SEED}')
    print('  merge events (t, pair, common, added):')
    for (t, i, j, nc, nadd, ok, rej) in res['comm_events']:
        st = ('merged' if ok else
              ('rejected (inconsistent or frames disagree)' if rej else 'skipped (<3 common)'))
        print(f'    t={t:6.1f} s  D{i+1}-D{j+1}  common {nc:2d}  added {nadd:2d}  {st}')

    print(f'\nRunning Monte Carlo over {N_MC} noise seeds...')
    mc = [res] + [run_simulation(s, record=False) for s in range(1, N_MC)]
    print(f'\n=== Monte Carlo ({N_MC} seeds), final values: mean [min, max] ===')
    for s in STRATS + list(MERGE_VARIANTS):
        for key, lab, sc in (('cov', 'coverage %', 100), ('cov_loose', 'cov<5 m %', 100),
                             ('rms', 'RMS m', 1),
                             ('union', 'team union %', 100), ('pose_final', 'pose RMSE m', 1)):
            v = np.array([m['final'][s][key] for m in mc], dtype=float) * sc
            print(f'  {s:14s} {lab:13s}: {np.nanmean(v):7.3f} [{np.nanmin(v):.3f}, {np.nanmax(v):.3f}]'
                  f' median {np.nanmedian(v):.3f}')
        nd = sum(sum(m['final'][s]['n_dup']) for m in mc)
        ne = max(max(m['final'][s]['n_est']) for m in mc)
        print(f'  {s:12s} duplicates over all runs/maps: {nd}, largest map: {ne} estimates')
    v = np.array([m['final']['dead_reckoning']['pose_final'] for m in mc])
    print(f'  dead reckoning pose RMSE m: {np.mean(v):7.3f} [{np.min(v):.3f}, {np.max(v):.3f}]')
    print(f'  smallest covariance eigenvalue over all runs: {min(m["min_eig"] for m in mc):.3e}')
    print(f'  merges per run: mean {np.mean([sum(e[5] for e in m["comm_events"]) for m in mc]):.1f}'
          + ''.join(f', {v} {np.mean([m["final"][v]["merges"] for m in mc]):.1f}'
                    for v in MERGE_VARIANTS))
    print('\n=== Per seed (final): 1 m coverage % | pose RMSE m ===')
    for key, lab, sc, fmt in (('cov', 'coverage %', 100, '{:6.1f}'), ('pose_final', 'pose RMSE m', 1, '{:6.2f}')):
        for st in STRATS + list(MERGE_VARIANTS):
            v = [m['final'][st][key] * sc for m in mc]
            print(f'  {lab:11s} {st:12s}: ' + ' '.join(fmt.format(x) for x in v)
                  + f' | median {np.median(v):.2f}')
    for key, lab, better in (('cov', '1 m coverage', np.greater), ('pose_final', 'pose RMSE', np.less)):
        for a_, b_ in (('distributed', 'isolated'), ('dist_certainty', 'isolated'),
                       ('dist_agree', 'isolated'), ('centralised', 'isolated'),
                       ('centralised', 'distributed')):
            va = np.array([m['final'][a_][key] for m in mc])
            vb = np.array([m['final'][b_][key] for m in mc])
            w = int(np.sum(better(va, vb) & ~np.isclose(va, vb)))
            tie = int(np.sum(np.isclose(va, vb)))
            print(f'  {lab:12s} {a_} vs {b_}: better in {w}/{N_MC}, worse in {N_MC - w - tie}/{N_MC}, '
                  f'tie {tie}/{N_MC}')

    print('\nGenerating plots...')
    plot_trajectories_map(res, out_dir)
    plot_coverage_rms(res, mc, out_dir)
    plot_final_map_heatmap(res, out_dir)
    plot_communication_events(res, out_dir)
    save_animation(res, out_dir)
    print('\nDone.')
