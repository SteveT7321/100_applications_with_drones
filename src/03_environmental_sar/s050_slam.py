"""
S050 Swarm Cooperative Mapping (Distributed EKF-SLAM)
======================================================
Three drones explore a 200x200 m GPS-denied environment, each running an
independent EKF-SLAM filter that estimates both the drone pose and the positions
of observed landmarks. When drones come within communication range they exchange
and merge local maps via SVD-based rigid alignment plus covariance-intersection
fusion. An information-gain frontier policy drives exploration. Results are
compared to an isolated (no communication) baseline.

Usage:
    conda activate drones
    python src/03_environmental_sar/s050_slam.py
"""

import sys
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import Ellipse
import matplotlib.animation as animation
from dataclasses import dataclass, field

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

# ── Parameters ─────────────────────────────────────────────────────────────────
N_DRONES     = 3
AREA_SIZE    = 200.0      # m, square side
M_LANDMARKS  = 30         # true landmarks in environment
DRONE_SPEED  = 3.0        # m/s
DT           = 0.2        # s — simulation timestep
T_MAX        = 600.0      # s — mission horizon
N_STEPS      = int(T_MAX / DT)

SENSOR_RANGE = 25.0       # m — landmark detection range
COMM_RANGE   = 40.0       # m — inter-drone communication range
SIGMA_RHO    = 0.3        # m — range noise std
SIGMA_PHI    = 0.03       # rad — bearing noise std (~1.7 deg)

SIGMA_AX     = 0.05       # m/s^2 acceleration noise
SIGMA_AY     = 0.05
SIGMA_OMEGA  = 0.02       # rad/s turn rate noise

MAHA_GATE    = 5.991      # chi^2(2, 0.95) data association gate
MATCH_RADIUS = 2.0        # m — common landmark proximity threshold
SUCCESS_R    = 1.0        # m — localisation success radius
SNAPSHOT_DT  = 10.0       # s — quality snapshot interval
MIN_COMMON   = 3          # min common landmarks to attempt merge

R_OBS = np.diag([SIGMA_RHO**2, SIGMA_PHI**2])

DRONE_COLORS = ['#e74c3c', '#2980b9', '#27ae60']  # red, blue, green

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '03_environmental_sar', 's050_slam',
)

RNG = np.random.default_rng(0)


# ── EKF-SLAM State ─────────────────────────────────────────────────────────────
@dataclass
class EKFSLAMState:
    """Local EKF-SLAM state for one drone."""
    drone_id: int
    pose: np.ndarray = field(default_factory=lambda: np.zeros(3))  # [x, y, theta]
    landmarks: np.ndarray = field(default_factory=lambda: np.empty((0, 2)))
    n_landmarks: int = 0
    P: np.ndarray = field(default_factory=lambda: np.eye(3) * 0.01)

    def state_dim(self):
        return 3 + 2 * self.n_landmarks

    def full_state(self):
        return np.concatenate([self.pose, self.landmarks.ravel()])


# ── EKF Helpers ────────────────────────────────────────────────────────────────
def predict_step(ekf, v, omega, dt):
    """EKF predict: propagate pose forward, landmarks unchanged."""
    x, y, th = ekf.pose
    n = ekf.n_landmarks
    dim = 3 + 2 * n

    ekf.pose = np.array([
        x + v * dt * np.cos(th),
        y + v * dt * np.sin(th),
        th + omega * dt,
    ])

    F = np.eye(dim)
    F[0, 2] = -v * dt * np.sin(th)
    F[1, 2] =  v * dt * np.cos(th)

    Q = np.zeros((dim, dim))
    Q[0, 0] = (SIGMA_AX * dt) ** 2
    Q[1, 1] = (SIGMA_AY * dt) ** 2
    Q[2, 2] = (SIGMA_OMEGA * dt) ** 2

    ekf.P = F @ ekf.P @ F.T + Q
    return ekf


def observation_jacobian(ekf, lm_idx):
    """Return H (2 x dim) and predicted range/bearing for landmark lm_idx."""
    x, y, th = ekf.pose
    lx, ly = ekf.landmarks[lm_idx]
    dx, dy = lx - x, ly - y
    rho = np.sqrt(dx ** 2 + dy ** 2) + 1e-9
    dim = ekf.state_dim()

    H = np.zeros((2, dim))
    H[0, 0] = -dx / rho;  H[0, 1] = -dy / rho;  H[0, 2] = 0.0
    H[1, 0] =  dy / rho**2; H[1, 1] = -dx / rho**2; H[1, 2] = -1.0
    li = 3 + 2 * lm_idx
    H[0, li] =  dx / rho;   H[0, li+1] =  dy / rho
    H[1, li] = -dy / rho**2; H[1, li+1] =  dx / rho**2

    pred_phi = np.arctan2(dy, dx) - th
    return H, rho, pred_phi


def ekf_update(ekf, lm_idx, z_rho, z_phi):
    """EKF update for one range-bearing observation."""
    H, pred_rho, pred_phi = observation_jacobian(ekf, lm_idx)
    nu = np.array([z_rho - pred_rho, z_phi - pred_phi])
    nu[1] = np.arctan2(np.sin(nu[1]), np.cos(nu[1]))

    S = H @ ekf.P @ H.T + R_OBS
    try:
        K = ekf.P @ H.T @ np.linalg.inv(S)
    except np.linalg.LinAlgError:
        return ekf

    full = ekf.full_state() + K @ nu
    ekf.pose = full[:3]
    if ekf.n_landmarks > 0:
        ekf.landmarks = full[3:].reshape(-1, 2)
    ekf.P = (np.eye(ekf.state_dim()) - K @ H) @ ekf.P
    return ekf


def data_association(ekf, z_rho, z_phi):
    """Nearest-neighbour with Mahalanobis gate. Returns landmark index or -1."""
    best_dist = np.inf
    best_idx = -1
    for j in range(ekf.n_landmarks):
        H, pred_rho, pred_phi = observation_jacobian(ekf, j)
        nu = np.array([z_rho - pred_rho, z_phi - pred_phi])
        nu[1] = np.arctan2(np.sin(nu[1]), np.cos(nu[1]))
        S = H @ ekf.P @ H.T + R_OBS
        try:
            d_maha = float(nu @ np.linalg.inv(S) @ nu)
        except np.linalg.LinAlgError:
            continue
        if d_maha < best_dist:
            best_dist = d_maha
            best_idx = j
    if best_dist <= MAHA_GATE:
        return best_idx
    return -1


def initialise_landmark(ekf, z_rho, z_phi):
    """Append a new landmark to the EKF state."""
    x, y, th = ekf.pose
    global_angle = th + z_phi
    lx = x + z_rho * np.cos(global_angle)
    ly = y + z_rho * np.sin(global_angle)

    Jinv = np.zeros((2, 5))
    Jinv[0, 0] = 1.0; Jinv[0, 2] = -z_rho * np.sin(global_angle)
    Jinv[0, 3] = np.cos(global_angle)
    Jinv[0, 4] = -z_rho * np.sin(global_angle)
    Jinv[1, 1] = 1.0; Jinv[1, 2] =  z_rho * np.cos(global_angle)
    Jinv[1, 3] = np.sin(global_angle)
    Jinv[1, 4] =  z_rho * np.cos(global_angle)

    src_cov = np.zeros((5, 5))
    src_cov[:3, :3] = ekf.P[:3, :3]
    src_cov[3:, 3:] = R_OBS
    P_ll = Jinv @ src_cov @ Jinv.T

    old_dim = ekf.state_dim()
    new_dim = old_dim + 2
    new_P = np.zeros((new_dim, new_dim))
    new_P[:old_dim, :old_dim] = ekf.P
    new_P[old_dim:, old_dim:] = P_ll

    if ekf.n_landmarks > 0:
        ekf.landmarks = np.vstack([ekf.landmarks, [lx, ly]])
    else:
        ekf.landmarks = np.array([[lx, ly]])
    ekf.n_landmarks += 1
    ekf.P = new_P
    return ekf


def merge_maps(host, visitor):
    """
    Merge visitor's map into host using SVD alignment + covariance intersection.
    Returns number of common landmarks found.
    """
    if visitor.n_landmarks == 0 or host.n_landmarks == 0:
        return 0

    # Find common landmarks
    common_host, common_vis = [], []
    for vi in range(visitor.n_landmarks):
        vl = visitor.landmarks[vi]
        dists = np.linalg.norm(host.landmarks - vl, axis=1)
        if dists.min() < MATCH_RADIUS:
            hi = int(np.argmin(dists))
            if hi not in common_host:
                common_host.append(hi)
                common_vis.append(vi)

    if len(common_host) < MIN_COMMON:
        return len(common_host)

    # SVD rigid alignment: visitor -> host frame
    Lh = host.landmarks[common_host]
    Lv = visitor.landmarks[common_vis]
    mu_h = Lh.mean(axis=0)
    mu_v = Lv.mean(axis=0)
    W = (Lh - mu_h).T @ (Lv - mu_v)
    try:
        U, _, Vt = np.linalg.svd(W)
    except np.linalg.LinAlgError:
        return len(common_host)

    R_align = U @ Vt
    if np.linalg.det(R_align) < 0:
        Vt[-1, :] *= -1
        R_align = U @ Vt
    t_align = mu_h - R_align @ mu_v

    # Fuse non-common visitor landmarks into host
    for vi in range(visitor.n_landmarks):
        if vi in common_vis:
            continue
        lv_t = R_align @ visitor.landmarks[vi] + t_align
        P_v = visitor.P[3 + 2*vi: 3 + 2*vi + 2, 3 + 2*vi: 3 + 2*vi + 2]

        if host.n_landmarks > 0:
            dists = np.linalg.norm(host.landmarks - lv_t, axis=1)
            if dists.min() < MATCH_RADIUS:
                hi = int(np.argmin(dists))
                P_h = host.P[3 + 2*hi: 3 + 2*hi + 2, 3 + 2*hi: 3 + 2*hi + 2]
                try:
                    K_m = P_h @ np.linalg.inv(P_h + P_v)
                except np.linalg.LinAlgError:
                    continue
                host.landmarks[hi] = host.landmarks[hi] + K_m @ (lv_t - host.landmarks[hi])
                host.P[3 + 2*hi: 3 + 2*hi + 2, 3 + 2*hi: 3 + 2*hi + 2] = (np.eye(2) - K_m) @ P_h
                continue

        # Add as new landmark to host
        old_dim = host.state_dim()
        new_dim = old_dim + 2
        new_P = np.zeros((new_dim, new_dim))
        new_P[:old_dim, :old_dim] = host.P
        new_P[old_dim:, old_dim:] = P_v
        if host.n_landmarks > 0:
            host.landmarks = np.vstack([host.landmarks, lv_t])
        else:
            host.landmarks = lv_t[np.newaxis]
        host.n_landmarks += 1
        host.P = new_P

    return len(common_host)


# ── Navigation policy ──────────────────────────────────────────────────────────
def frontier_policy(ekf, explored_grid, grid_res, rng, other_targets):
    """
    Information-gain frontier policy. Returns turn rate omega.
    explored_grid: boolean array (W, W) — cells within sensor range of past positions.
    """
    W = explored_grid.shape[0]
    x, y, th = ekf.pose
    cx, cy = int(x / grid_res), int(y / grid_res)

    # Find frontier cells (unexplored cells adjacent to explored ones)
    # Sample candidate waypoints on unexplored grid
    unexplored_mask = ~explored_grid
    if not unexplored_mask.any():
        # Fully explored: wander randomly
        return rng.uniform(-0.3, 0.3)

    # Candidate waypoints: unexplored cells
    ys, xs = np.where(unexplored_mask)
    if len(xs) == 0:
        return rng.uniform(-0.3, 0.3)

    # Compute information gain (number of unexplored cells within sensor range)
    wpts_x = xs * grid_res + grid_res / 2
    wpts_y = ys * grid_res + grid_res / 2
    dist_to_drone = np.sqrt((wpts_x - x)**2 + (wpts_y - y)**2)

    # Only consider reachable frontier cells within 2*SENSOR_RANGE
    close_mask = dist_to_drone < 2.0 * SENSOR_RANGE
    if not close_mask.any():
        close_mask = dist_to_drone < 4.0 * SENSOR_RANGE
    if not close_mask.any():
        return rng.uniform(-0.3, 0.3)

    wpts_x = wpts_x[close_mask]
    wpts_y = wpts_y[close_mask]
    dist_to_drone = dist_to_drone[close_mask]

    # IG = unexplored cells within sensor range of candidate
    ig_scores = np.zeros(len(wpts_x))
    for k, (wx, wy) in enumerate(zip(wpts_x, wpts_y)):
        ci_x = int(np.clip(wx / grid_res, 0, W - 1))
        ci_y = int(np.clip(wy / grid_res, 0, W - 1))
        # Count unexplored in a square neighbourhood
        r = max(1, int(SENSOR_RANGE / grid_res))
        x0, x1 = max(0, ci_x - r), min(W, ci_x + r + 1)
        y0, y1 = max(0, ci_y - r), min(W, ci_y + r + 1)
        ig_scores[k] = unexplored_mask[y0:y1, x0:x1].sum()

    # Penalise proximity to other drone targets
    for ot in other_targets:
        if ot is not None:
            d_other = np.sqrt((wpts_x - ot[0])**2 + (wpts_y - ot[1])**2)
            ig_scores[d_other < SENSOR_RANGE] *= 0.1

    ratio = ig_scores / (dist_to_drone + 1e-6)
    best = int(np.argmax(ratio))
    target = np.array([wpts_x[best], wpts_y[best]])

    # Turn toward target
    desired_angle = np.arctan2(target[1] - y, target[0] - x)
    angle_diff = np.arctan2(np.sin(desired_angle - th), np.cos(desired_angle - th))
    omega = np.clip(angle_diff / DT, -1.0, 1.0)  # max 1 rad/s turn
    return omega, target


# ── Simulation ─────────────────────────────────────────────────────────────────
def compute_map_quality(ekf, true_landmarks):
    """Return (rms, coverage, n_found) for the current map vs ground truth."""
    if ekf.n_landmarks == 0:
        return np.nan, 0.0, 0

    est = ekf.landmarks
    best_dist_per_est = np.array([
        np.min(np.linalg.norm(true_landmarks - lm, axis=1))
        for lm in est
    ])
    found_mask = best_dist_per_est < SUCCESS_R
    n_found = int(found_mask.sum())

    # RMS: over matched estimates only
    if n_found > 0:
        rms = float(np.sqrt(np.mean(best_dist_per_est[found_mask] ** 2)))
    else:
        rms = np.nan
    coverage = n_found / M_LANDMARKS
    return rms, coverage, n_found


def run_simulation():
    """Run distributed EKF-SLAM and isolated baseline. Return all data."""
    rng_lm = np.random.default_rng(42)
    true_landmarks = rng_lm.uniform(10.0, 190.0, size=(M_LANDMARKS, 2))

    results = {}
    for strategy in ('distributed', 'isolated'):
        rng = np.random.default_rng(0)

        # Grid for frontier exploration
        grid_res = 5.0
        W = int(AREA_SIZE / grid_res)
        explored = np.zeros((N_DRONES, W, W), dtype=bool)

        # Start positions
        starts = [
            np.array([20.0, 20.0, 0.0]),
            np.array([100.0, 20.0, np.pi / 2]),
            np.array([180.0, 20.0, np.pi]),
        ]
        ekfs = [
            EKFSLAMState(drone_id=k, pose=starts[k].copy())
            for k in range(N_DRONES)
        ]

        # Per-drone waypoints for frontier policy
        targets = [None] * N_DRONES

        # History
        pose_history = [[] for _ in range(N_DRONES)]  # list of [x, y, th]
        quality_history = []   # (t, rms_dist, coverage_dist, rms_iso, cov_iso)
        comm_events = []       # (t, i, j, n_common)

        snap_times = []
        rms_hist   = {'distributed': [], 'isolated_avg': []}
        cov_hist   = {'distributed': [], 'isolated_avg': []}

        # Separate isolated ekfs
        ekfs_iso = [
            EKFSLAMState(drone_id=k, pose=starts[k].copy())
            for k in range(N_DRONES)
        ]

        for step in range(N_STEPS):
            sim_time = (step + 1) * DT

            for i in range(N_DRONES):
                ekf = ekfs[i]
                ekf_iso = ekfs_iso[i]

                # Navigation: frontier policy
                other_t = [targets[j] for j in range(N_DRONES) if j != i]
                result = frontier_policy(ekf, explored[i], grid_res, rng, other_t)
                if isinstance(result, tuple):
                    omega_cmd, targets[i] = result
                else:
                    omega_cmd = result

                # Predict step — both distributed and isolated move identically
                ekf = predict_step(ekf, DRONE_SPEED, omega_cmd, DT)
                ekf_iso = predict_step(ekf_iso, DRONE_SPEED, omega_cmd, DT)

                # Boundary reflection
                for e in [ekf, ekf_iso]:
                    e.pose[0] = np.clip(e.pose[0], 1.0, AREA_SIZE - 1.0)
                    e.pose[1] = np.clip(e.pose[1], 1.0, AREA_SIZE - 1.0)

                # Mark explored grid cells
                gx = int(np.clip(ekf.pose[0] / grid_res, 0, W - 1))
                gy = int(np.clip(ekf.pose[1] / grid_res, 0, W - 1))
                r_cells = max(1, int(SENSOR_RANGE / grid_res))
                gx0, gx1 = max(0, gx - r_cells), min(W, gx + r_cells + 1)
                gy0, gy1 = max(0, gy - r_cells), min(W, gy + r_cells + 1)
                explored[i, gy0:gy1, gx0:gx1] = True

                # Sensor: observe landmarks within range
                dists_lm = np.linalg.norm(true_landmarks - ekf.pose[:2], axis=1)
                visible = np.where(dists_lm <= SENSOR_RANGE)[0]

                for lm_idx in visible:
                    true_rho = dists_lm[lm_idx]
                    true_phi = np.arctan2(
                        true_landmarks[lm_idx, 1] - ekf.pose[1],
                        true_landmarks[lm_idx, 0] - ekf.pose[0],
                    ) - ekf.pose[2]

                    z_rho = true_rho + rng.normal(0, SIGMA_RHO)
                    z_phi = true_phi + rng.normal(0, SIGMA_PHI)
                    z_phi = np.arctan2(np.sin(z_phi), np.cos(z_phi))

                    assoc = data_association(ekf, z_rho, z_phi)
                    if assoc >= 0:
                        ekf = ekf_update(ekf, assoc, z_rho, z_phi)
                    else:
                        ekf = initialise_landmark(ekf, z_rho, z_phi)

                    # Same for isolated
                    assoc_iso = data_association(ekf_iso, z_rho, z_phi)
                    if assoc_iso >= 0:
                        ekf_iso = ekf_update(ekf_iso, assoc_iso, z_rho, z_phi)
                    else:
                        ekf_iso = initialise_landmark(ekf_iso, z_rho, z_phi)

                ekfs[i] = ekf
                ekfs_iso[i] = ekf_iso
                pose_history[i].append(ekf.pose.copy())

            # Communication / map merging (distributed only)
            if strategy == 'distributed':
                for i in range(N_DRONES):
                    for j in range(i + 1, N_DRONES):
                        d_ij = np.linalg.norm(ekfs[i].pose[:2] - ekfs[j].pose[:2])
                        if d_ij <= COMM_RANGE:
                            n_c1 = merge_maps(ekfs[i], ekfs[j])
                            n_c2 = merge_maps(ekfs[j], ekfs[i])
                            if n_c1 >= MIN_COMMON or n_c2 >= MIN_COMMON:
                                comm_events.append((sim_time, i, j, max(n_c1, n_c2)))

            # Quality snapshots every SNAPSHOT_DT seconds
            if step > 0 and abs(sim_time % SNAPSHOT_DT) < DT * 0.5:
                snap_times.append(sim_time)

                # Distributed: best drone (most landmarks)
                best_ekf = max(ekfs, key=lambda e: e.n_landmarks)
                rms_d, cov_d, _ = compute_map_quality(best_ekf, true_landmarks)

                # Isolated: average across drones
                rms_list, cov_list = [], []
                for e in ekfs_iso:
                    r, c, _ = compute_map_quality(e, true_landmarks)
                    if not np.isnan(r):
                        rms_list.append(r)
                    cov_list.append(c)
                rms_iso = float(np.mean(rms_list)) if rms_list else np.nan
                cov_iso = float(np.mean(cov_list))

                rms_hist['distributed'].append(rms_d)
                rms_hist['isolated_avg'].append(rms_iso)
                cov_hist['distributed'].append(cov_d)
                cov_hist['isolated_avg'].append(cov_iso)

        # Final metrics
        best_ekf = max(ekfs, key=lambda e: e.n_landmarks)
        rms_final, cov_final, n_found_final = compute_map_quality(best_ekf, true_landmarks)

        iso_rms_list, iso_cov_list = [], []
        for e in ekfs_iso:
            r, c, _ = compute_map_quality(e, true_landmarks)
            if not np.isnan(r): iso_rms_list.append(r)
            iso_cov_list.append(c)

        results[strategy] = {
            'ekfs': ekfs,
            'ekfs_iso': ekfs_iso,
            'pose_history': pose_history,
            'snap_times': snap_times,
            'rms_hist': rms_hist,
            'cov_hist': cov_hist,
            'comm_events': comm_events,
            'rms_final': rms_final,
            'cov_final': cov_final,
            'n_found_final': n_found_final,
            'iso_rms_final': float(np.mean(iso_rms_list)) if iso_rms_list else np.nan,
            'iso_cov_final': float(np.mean(iso_cov_list)),
        }
        break  # Only run one strategy (distributed); isolated is embedded within

    return results, true_landmarks


# ── Plots ───────────────────────────────────────────────────────────────────────
def plot_covariance_ellipse(ax, mean, cov, n_std=1.0, **kwargs):
    """Draw a covariance ellipse on ax."""
    try:
        vals, vecs = np.linalg.eigh(cov)
        vals = np.abs(vals)
        angle = np.degrees(np.arctan2(vecs[1, 1], vecs[0, 1]))
        w, h = 2 * n_std * np.sqrt(vals)
        ell = Ellipse(xy=mean, width=w, height=h, angle=angle, **kwargs)
        ax.add_patch(ell)
    except Exception:
        pass


def plot_environment_and_trajectories(results, true_landmarks, out_dir):
    """Trajectory + final map for distributed vs isolated."""
    res = results['distributed']
    pose_hist = res['pose_history']
    ekfs = res['ekfs']
    ekfs_iso = res['ekfs_iso']

    fig, axes = plt.subplots(1, 2, figsize=(16, 7))

    titles = ['Distributed Swarm EKF-SLAM', 'Isolated EKF-SLAM (Drone 0)']
    ekf_sets = [ekfs, [ekfs_iso[0]]]

    for ax_idx, (ax, title, ekf_set) in enumerate(zip(axes, titles, ekf_sets)):
        ax.set_aspect('equal')
        ax.set_xlim(0, AREA_SIZE)
        ax.set_ylim(0, AREA_SIZE)
        ax.set_facecolor('#f8f8f8')
        ax.set_title(title, fontsize=12, fontweight='bold')
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')

        # True landmarks
        ax.scatter(true_landmarks[:, 0], true_landmarks[:, 1],
                   c='k', marker='*', s=80, zorder=5, label='True landmark', alpha=0.6)

        # Drone trajectories
        for i in range(N_DRONES if ax_idx == 0 else 1):
            traj = np.array(pose_hist[i])
            ax.plot(traj[:, 0], traj[:, 1], '-',
                    color=DRONE_COLORS[i], lw=0.8, alpha=0.5, label=f'Drone {i+1} path')
            ax.plot(traj[-1, 0], traj[-1, 1], 'o',
                    color=DRONE_COLORS[i], ms=8, zorder=6)

        # Estimated landmarks with covariance ellipses
        for ekf in ekf_set:
            for k in range(ekf.n_landmarks):
                lm = ekf.landmarks[k]
                P_ll = ekf.P[3 + 2*k: 3 + 2*k + 2, 3 + 2*k: 3 + 2*k + 2]
                ax.scatter(lm[0], lm[1], marker='+', c='purple', s=60, zorder=4)
                plot_covariance_ellipse(ax, lm, P_ll, n_std=1.0,
                                        edgecolor='purple', facecolor='none',
                                        alpha=0.4, lw=0.7)

        ax.legend(loc='upper right', fontsize=8)

    plt.tight_layout()
    path = os.path.join(out_dir, 'trajectories_map.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def plot_coverage_and_rms(results, out_dir):
    """Coverage and RMS error over time: distributed vs isolated."""
    res = results['distributed']
    times = np.array(res['snap_times'])
    cov_d = np.array(res['cov_hist']['distributed']) * 100.0
    cov_i = np.array(res['cov_hist']['isolated_avg']) * 100.0
    rms_d = np.array(res['rms_hist']['distributed'])
    rms_i = np.array(res['rms_hist']['isolated_avg'])

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

    ax1.plot(times, cov_d, '-', color='#e74c3c', lw=2, label='Distributed swarm')
    ax1.plot(times, cov_i, '--', color='#2980b9', lw=2, label='Isolated (avg drone)')
    ax1.set_ylabel('Coverage (%)', fontsize=11)
    ax1.set_title('Landmark Coverage vs Time', fontsize=12, fontweight='bold')
    ax1.set_ylim(0, 105)
    ax1.legend(fontsize=10)
    ax1.grid(True, alpha=0.3)

    # Add communication events as tick marks
    for (t, i, j, nc) in res['comm_events'][:50]:  # cap for readability
        ax1.axvline(t, color='gray', alpha=0.2, lw=0.8)

    # RMS
    valid_d = ~np.isnan(rms_d)
    valid_i = ~np.isnan(rms_i)
    if valid_d.any():
        ax2.plot(times[valid_d], rms_d[valid_d], '-', color='#e74c3c', lw=2,
                 label='Distributed swarm')
    if valid_i.any():
        ax2.plot(times[valid_i], rms_i[valid_i], '--', color='#2980b9', lw=2,
                 label='Isolated (avg drone)')
    ax2.set_ylabel('RMS Error (m)', fontsize=11)
    ax2.set_xlabel('Time (s)', fontsize=11)
    ax2.set_title('Landmark RMS Position Error vs Time', fontsize=12, fontweight='bold')
    ax2.legend(fontsize=10)
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    path = os.path.join(out_dir, 'coverage_rms.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def plot_final_map_heatmap(results, true_landmarks, out_dir):
    """Final merged map: landmark uncertainty heatmap."""
    res = results['distributed']
    ekfs = res['ekfs']
    best_ekf = max(ekfs, key=lambda e: e.n_landmarks)

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_aspect('equal')
    ax.set_xlim(0, AREA_SIZE)
    ax.set_ylim(0, AREA_SIZE)
    ax.set_facecolor('#1a1a2e')
    ax.set_title('Final Merged Map — Landmark Uncertainty', fontsize=13, fontweight='bold')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')

    # True landmarks
    ax.scatter(true_landmarks[:, 0], true_landmarks[:, 1],
               c='white', marker='*', s=100, zorder=5, alpha=0.9, label='True')

    # Estimated landmarks coloured by uncertainty
    if best_ekf.n_landmarks > 0:
        uncertainties = []
        for k in range(best_ekf.n_landmarks):
            P_ll = best_ekf.P[3 + 2*k: 3 + 2*k + 2, 3 + 2*k: 3 + 2*k + 2]
            uncertainties.append(np.sqrt(np.trace(P_ll)))
        uncertainties = np.array(uncertainties)

        sc = ax.scatter(best_ekf.landmarks[:, 0], best_ekf.landmarks[:, 1],
                        c=uncertainties, cmap='RdYlBu_r',
                        s=120, marker='+', linewidths=2,
                        vmin=0, vmax=uncertainties.max() + 0.1, zorder=6)
        cb = plt.colorbar(sc, ax=ax, shrink=0.7)
        cb.set_label('Position Uncertainty (m)', fontsize=10)

    # Draw matching lines
    if best_ekf.n_landmarks > 0:
        for lm in best_ekf.landmarks:
            dists = np.linalg.norm(true_landmarks - lm, axis=1)
            nearest_idx = np.argmin(dists)
            if dists[nearest_idx] < SUCCESS_R * 2:
                ax.plot([lm[0], true_landmarks[nearest_idx, 0]],
                        [lm[1], true_landmarks[nearest_idx, 1]],
                        '-', color='cyan', lw=0.5, alpha=0.4)

    ax.legend(loc='upper right', fontsize=9)
    plt.tight_layout()
    path = os.path.join(out_dir, 'final_map_heatmap.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def plot_communication_events(results, out_dir):
    """Communication event timeline."""
    res = results['distributed']
    events = res['comm_events']

    fig, ax = plt.subplots(figsize=(12, 3))
    colors_pair = {(0, 1): '#e74c3c', (0, 2): '#27ae60', (1, 2): '#2980b9'}
    labels_done = set()

    for (t, i, j, nc) in events:
        col = colors_pair.get((i, j), 'gray')
        lbl = f'D{i+1}↔D{j+1}' if (i, j) not in labels_done else '_'
        if (i, j) not in labels_done:
            labels_done.add((i, j))
        ax.axvline(t, color=col, alpha=0.6, lw=1.5, label=lbl)

    ax.set_xlim(0, T_MAX)
    ax.set_ylim(0, 1)
    ax.set_yticks([])
    ax.set_xlabel('Time (s)', fontsize=11)
    ax.set_title('Map Merge Events (Pairwise Communication)', fontsize=12, fontweight='bold')
    ax.legend(fontsize=10, loc='upper right')
    ax.grid(axis='x', alpha=0.3)

    plt.tight_layout()
    path = os.path.join(out_dir, 'comm_events.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def save_animation(results, true_landmarks, out_dir):
    """Animate drone positions, estimated landmarks, and communication links."""
    res = results['distributed']
    pose_hist = res['pose_history']
    ekfs_final = res['ekfs']

    # Decimate frames for animation
    total_steps = len(pose_hist[0])
    step_size = max(1, total_steps // 150)
    frame_indices = list(range(0, total_steps, step_size))

    fig, ax = plt.subplots(figsize=(7, 7))
    ax.set_aspect('equal')
    ax.set_xlim(0, AREA_SIZE)
    ax.set_ylim(0, AREA_SIZE)
    ax.set_facecolor('#f0f0f0')

    # True landmark markers (static)
    ax.scatter(true_landmarks[:, 0], true_landmarks[:, 1],
               c='k', marker='*', s=60, zorder=5, alpha=0.5)

    time_text = ax.text(5, 192, '', fontsize=10)
    lm_scatter = ax.scatter([], [], c='purple', marker='+', s=50, zorder=4, alpha=0.8)

    drone_dots = []
    drone_trails = []
    for i in range(N_DRONES):
        dot, = ax.plot([], [], 'o', color=DRONE_COLORS[i], ms=10, zorder=7)
        trail, = ax.plot([], [], '-', color=DRONE_COLORS[i], lw=0.8, alpha=0.5)
        drone_dots.append(dot)
        drone_trails.append(trail)

    comm_lines = []
    for _ in range(3):
        line, = ax.plot([], [], '-', color='gold', lw=2, alpha=0.0)
        comm_lines.append(line)

    comm_events = res['comm_events']
    comm_event_times = [e[0] for e in comm_events]

    def init():
        for dot in drone_dots:
            dot.set_data([], [])
        for trail in drone_trails:
            trail.set_data([], [])
        for cl in comm_lines:
            cl.set_alpha(0.0)
        lm_scatter.set_offsets(np.empty((0, 2)))
        time_text.set_text('')
        return drone_dots + drone_trails + comm_lines + [lm_scatter, time_text]

    def update(frame_idx):
        step = frame_indices[frame_idx]
        sim_time = (step + 1) * DT

        for i in range(N_DRONES):
            pos = pose_hist[i][step]
            drone_dots[i].set_data([pos[0]], [pos[1]])
            trail_start = max(0, step - 100)
            traj = np.array(pose_hist[i][trail_start:step + 1])
            drone_trails[i].set_data(traj[:, 0], traj[:, 1])

        # Estimated landmarks at this time (use final as proxy — simplification)
        all_lms = []
        for ekf in ekfs_final:
            if ekf.n_landmarks > 0:
                all_lms.append(ekf.landmarks)
        if all_lms:
            all_lms = np.vstack(all_lms)
            lm_scatter.set_offsets(all_lms)

        # Communication links: flash for 1 second after event
        pair_idx = 0
        for ci, (t_ev, i_ev, j_ev, _) in enumerate(comm_events):
            if pair_idx >= len(comm_lines):
                break
            if abs(sim_time - t_ev) < 2.0:
                p_i = pose_hist[i_ev][step]
                p_j = pose_hist[j_ev][step]
                comm_lines[pair_idx].set_data([p_i[0], p_j[0]], [p_i[1], p_j[1]])
                comm_lines[pair_idx].set_alpha(0.8)
                pair_idx += 1
        for k in range(pair_idx, len(comm_lines)):
            comm_lines[k].set_alpha(0.0)

        time_text.set_text(f't = {sim_time:.0f} s')
        return drone_dots + drone_trails + comm_lines + [lm_scatter, time_text]

    ani = animation.FuncAnimation(
        fig, update, frames=len(frame_indices),
        init_func=init, blit=True, interval=80,
    )

    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer='pillow', fps=12, dpi=100)
    plt.close()
    print(f'Saved: {path}')


# ── Main ───────────────────────────────────────────────────────────────────────
if __name__ == '__main__':
    out_dir = os.path.normpath(OUTPUT_DIR)
    os.makedirs(out_dir, exist_ok=True)

    print('Running Swarm EKF-SLAM simulation...')
    results, true_landmarks = run_simulation()

    res = results['distributed']
    print(f'\n=== Final Results ===')
    print(f'  Distributed swarm — Coverage: {res["cov_final"]*100:.1f}%  '
          f'RMS: {res["rms_final"]:.3f} m  '
          f'Landmarks found: {res["n_found_final"]}/{M_LANDMARKS}')
    print(f'  Isolated (avg)    — Coverage: {res["iso_cov_final"]*100:.1f}%  '
          f'RMS: {res["iso_rms_final"]:.3f} m')
    print(f'  Map merge events: {len(res["comm_events"])}')

    print('\nGenerating plots...')
    plot_environment_and_trajectories(results, true_landmarks, out_dir)
    plot_coverage_and_rms(results, out_dir)
    plot_final_map_heatmap(results, true_landmarks, out_dir)
    plot_communication_events(results, out_dir)
    save_animation(results, true_landmarks, out_dir)
    print('\nDone.')
