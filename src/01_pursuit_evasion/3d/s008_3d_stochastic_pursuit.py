"""
S008 3D Upgrade — Stochastic Pursuit (Kalman Filter Tracking)
=============================================================
Evader moves in true 3D with stochastic acceleration.
Pursuer tracks using a 6-state Kalman Filter and compares 4 modes:
  - kalman_3d        : full 3D KF (6-state)
  - kalman_2d_althold: 4-state KF on x-y; z held at last known altitude
  - naive            : aim at raw noisy measurement
  - oracle           : perfect 3D state knowledge

3 evader trajectories: random_walk, helix, z_hop

Usage:
    conda activate drones
    python src/01_pursuit_evasion/3d/s008_3d_stochastic_pursuit.py
"""

import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from matplotlib.animation import FuncAnimation, PillowWriter

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..'))
from src.base.drone_base import DroneBase

# ── Parameters ───────────────────────────────────────────────
DT          = 1 / 48
Q_XY        = 0.1     # process noise m²/s³
Q_Z_RATIO   = 0.5     # q_z = Q_XY * Q_Z_RATIO
MEAS_SIGMA  = 0.3     # measurement noise m
V_PURSUER   = 5.0     # m/s
V_EVADER    = 3.0     # m/s
CAPTURE_R   = 0.15    # m
MAX_TIME    = 30.0

INIT_PURSUER = np.array([-4.0, 0.0, 2.0])
INIT_EVADER  = np.array([ 4.0, 0.0, 3.0])

TRACKER_MODES   = ['kalman_3d', 'kalman_2d_althold', 'naive', 'oracle']
EVADER_TYPES    = ['random_walk', 'helix', 'z_hop']

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..', '..',
    'outputs', '01_pursuit_evasion', '3d', 's008_3d_stochastic_pursuit',
)
os.makedirs(OUTPUT_DIR, exist_ok=True)

RNG = np.random.default_rng(42)


# ── Kalman Filter (6-state 3D) ────────────────────────────────

class KalmanFilter6D:
    """6-state 3D Kalman filter: [x,y,z, vx,vy,vz]."""

    def __init__(self, dt=DT, q_xy=Q_XY, q_z_ratio=Q_Z_RATIO, sigma_meas=MEAS_SIGMA):
        I3 = np.eye(3)
        Z3 = np.zeros((3, 3))
        self.F = np.block([[I3, dt * I3], [Z3, I3]])
        self.H = np.hstack([I3, Z3])

        q_diag = q_xy * np.array([1, 1, q_z_ratio, 1, 1, q_z_ratio])
        self.Q = np.diag(q_diag)

        self.R = (sigma_meas ** 2) * I3
        self.x = np.zeros(6)
        self.P = np.eye(6) * 2.0

    def init_state(self, pos, vel=None):
        self.x[:3] = pos.copy()
        if vel is not None:
            self.x[3:] = vel.copy()

    def step(self, z):
        # Predict
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        # Update
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        innov = z - self.H @ self.x
        self.x = self.x + K @ innov
        self.P = (np.eye(6) - K @ self.H) @ self.P
        return self.x[:3].copy(), self.x[3:].copy(), self.P[:3, :3].copy()


class KalmanFilter4D:
    """4-state 2D Kalman filter for x-y only: [x,y, vx,vy]."""

    def __init__(self, dt=DT, q_xy=Q_XY, sigma_meas=MEAS_SIGMA):
        I2 = np.eye(2)
        Z2 = np.zeros((2, 2))
        self.F = np.block([[I2, dt * I2], [Z2, I2]])
        self.H = np.hstack([I2, Z2])
        q_diag = q_xy * np.array([1, 1, 1, 1])
        self.Q = np.diag(q_diag)
        self.R = (sigma_meas ** 2) * I2
        self.x = np.zeros(4)
        self.P = np.eye(4) * 2.0

    def init_state(self, pos_xy, vel_xy=None):
        self.x[:2] = pos_xy.copy()
        if vel_xy is not None:
            self.x[2:] = vel_xy.copy()

    def step(self, z_xy):
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        innov = z_xy - self.H @ self.x
        self.x = self.x + K @ innov
        self.P = (np.eye(4) - K @ self.H) @ self.P
        return self.x[:2].copy(), self.x[2:].copy()


# ── Evader trajectory generators ─────────────────────────────

def gen_random_walk(max_steps, rng):
    pos = INIT_EVADER.copy().astype(float)
    vel = np.array([V_EVADER * 0.5, 0.0, 0.0])
    traj = [pos.copy()]
    for _ in range(max_steps):
        acc = rng.normal(0, 0.5, 3) * DT
        vel += acc
        spd = np.linalg.norm(vel)
        if spd > V_EVADER:
            vel = vel / spd * V_EVADER
        vel[2] = np.clip(vel[2], -1.0, 1.0)
        pos = pos + vel * DT
        pos[2] = np.clip(pos[2], 0.3, 8.0)
        traj.append(pos.copy())
    return np.array(traj)


def gen_helix(max_steps):
    R = 3.0
    z0 = 3.0
    omega = V_EVADER / R
    traj = []
    for k in range(max_steps + 1):
        t = k * DT
        x = R * np.cos(omega * t)
        y = R * np.sin(omega * t)
        z = z0 + 0.3 * t
        z = min(z, 8.0)
        traj.append(np.array([x, y, z]))
    return np.array(traj)


def gen_z_hop(max_steps, rng):
    pos = INIT_EVADER.copy().astype(float)
    vel = np.array([V_EVADER * 0.5, 0.0, 0.0])
    traj = [pos.copy()]
    hop_interval = int(3.0 / DT)
    for k in range(max_steps):
        acc_xy = rng.normal(0, 0.5, 2) * DT
        vel[:2] += acc_xy
        spd_xy = np.linalg.norm(vel[:2])
        if spd_xy > V_EVADER * 0.9:
            vel[:2] = vel[:2] / spd_xy * V_EVADER * 0.9
        if k % hop_interval == 0 and k > 0:
            vel[2] = rng.choice([-1.5, 1.5])
        else:
            vel[2] *= 0.8
        pos = pos + vel * DT
        pos[2] = np.clip(pos[2], 0.3, 8.0)
        traj.append(pos.copy())
    return np.array(traj)


def get_evader_trajectory(evader_type, rng):
    max_steps = int(MAX_TIME / DT)
    if evader_type == 'random_walk':
        return gen_random_walk(max_steps, rng)
    elif evader_type == 'helix':
        return gen_helix(max_steps)
    elif evader_type == 'z_hop':
        return gen_z_hop(max_steps, rng)
    else:
        raise ValueError(f'Unknown evader type: {evader_type}')


# ── Predictive pursuit (lead target) ─────────────────────────

def predictive_pursuit_cmd(pos_p, pos_hat, vel_hat, v_p):
    """Iterative 2-pass lead prediction."""
    p_pred = pos_hat.copy()
    for _ in range(2):
        d = np.linalg.norm(p_pred - pos_p)
        t_int = d / (v_p + 1e-8)
        p_pred = pos_hat + vel_hat * t_int
    diff = p_pred - pos_p
    n = np.linalg.norm(diff)
    return v_p * diff / n if n > 1e-8 else np.zeros(3)


# ── Simulation ───────────────────────────────────────────────

def run_simulation(tracker_mode, evader_traj, rng):
    pursuer = DroneBase(INIT_PURSUER.copy(), max_speed=V_PURSUER, dt=DT)
    N = len(evader_traj)

    kf3d = KalmanFilter6D()
    kf3d.init_state(evader_traj[0])

    kf2d = KalmanFilter4D()
    kf2d.init_state(evader_traj[0][:2])
    last_known_z = evader_traj[0][2]

    p_traj    = [pursuer.pos.copy()]
    est_traj  = [evader_traj[0].copy()]
    meas_log  = [evader_traj[0].copy()]
    err_xyz   = [np.zeros(3)]
    cov_vol   = [0.0]
    times     = [0.0]

    captured = False
    cap_time = None

    for step in range(1, N):
        t = step * DT
        true_pos = evader_traj[step]
        meas = true_pos + rng.normal(0, MEAS_SIGMA, 3)
        last_known_z = meas[2]

        if tracker_mode == 'kalman_3d':
            pos_hat, vel_hat, P_pos = kf3d.step(meas)
            # Covariance ellipsoid volume
            eigvals = np.linalg.eigvalsh(P_pos)
            eigvals = np.maximum(eigvals, 0)
            vol = (4 * np.pi / 3) * np.prod(np.sqrt(eigvals))
            cov_vol.append(vol)
            cmd = predictive_pursuit_cmd(pursuer.pos, pos_hat, vel_hat, V_PURSUER)

        elif tracker_mode == 'kalman_2d_althold':
            pos_xy, vel_xy = kf2d.step(meas[:2])
            pos_hat = np.array([pos_xy[0], pos_xy[1], last_known_z])
            vel_hat = np.array([vel_xy[0], vel_xy[1], 0.0])
            cov_vol.append(0.0)
            cmd = predictive_pursuit_cmd(pursuer.pos, pos_hat, vel_hat, V_PURSUER)

        elif tracker_mode == 'naive':
            pos_hat = meas.copy()
            vel_hat = np.zeros(3)
            cov_vol.append(0.0)
            diff = pos_hat - pursuer.pos
            n = np.linalg.norm(diff)
            cmd = V_PURSUER * diff / n if n > 1e-8 else np.zeros(3)

        else:  # oracle
            pos_hat = true_pos.copy()
            vel_hat = np.zeros(3)
            if step > 1:
                vel_hat = (evader_traj[step] - evader_traj[step - 1]) / DT
            cov_vol.append(0.0)
            cmd = predictive_pursuit_cmd(pursuer.pos, pos_hat, vel_hat, V_PURSUER)

        pursuer.step(cmd)

        ex = true_pos[0] - pos_hat[0]
        ey = true_pos[1] - pos_hat[1]
        ez = true_pos[2] - pos_hat[2]

        p_traj.append(pursuer.pos.copy())
        est_traj.append(pos_hat.copy())
        meas_log.append(meas.copy())
        err_xyz.append(np.array([abs(ex), abs(ey), abs(ez)]))
        times.append(t)

        if np.linalg.norm(pursuer.pos - true_pos) < CAPTURE_R:
            captured = True
            cap_time = t
            break

    return {
        'p_traj':   np.array(p_traj),
        'est_traj': np.array(est_traj),
        'meas_log': np.array(meas_log),
        'err_xyz':  np.array(err_xyz),
        'cov_vol':  np.array(cov_vol),
        'times':    np.array(times),
        'captured': captured,
        'cap_time': cap_time,
    }


# ── Plots ────────────────────────────────────────────────────

TRACKER_COLORS = {
    'kalman_3d':         'firebrick',
    'kalman_2d_althold': 'darkorange',
    'naive':             'purple',
    'oracle':            'forestgreen',
}

TRACKER_LABELS = {
    'kalman_3d':         'Kalman 3D',
    'kalman_2d_althold': '2D KF + AltHold',
    'naive':             'Naive',
    'oracle':            'Oracle',
}


def plot_trajectories_3d(results_helix, evader_traj_helix, out_dir):
    """4 tracker modes × helix evader: 3D trajectories."""
    fig = plt.figure(figsize=(16, 12))
    for idx, mode in enumerate(TRACKER_MODES):
        res = results_helix[mode]
        ax = fig.add_subplot(2, 2, idx + 1, projection='3d')
        n = len(res['p_traj'])
        et = evader_traj_helix[:n]

        ax.plot(et[:, 0], et[:, 1], et[:, 2],
                color='royalblue', linewidth=1.5, label='Evader (true)', alpha=0.8)
        ax.plot(res['est_traj'][:, 0], res['est_traj'][:, 1], res['est_traj'][:, 2],
                color='cyan', linewidth=1.0, linestyle='--', alpha=0.6, label='KF estimate')
        ax.scatter(res['meas_log'][::10, 0], res['meas_log'][::10, 1],
                   res['meas_log'][::10, 2],
                   color='lightcoral', s=8, alpha=0.5, label='Measurements')
        ax.plot(res['p_traj'][:, 0], res['p_traj'][:, 1], res['p_traj'][:, 2],
                color='red', linewidth=1.8, label='Pursuer')

        # Start markers
        ax.scatter(*et[0], color='blue', s=50, zorder=5)
        ax.scatter(*res['p_traj'][0], color='red', s=50, zorder=5)

        status = f"Captured {res['cap_time']:.2f}s" if res['captured'] else 'Timeout'
        ax.set_title(f"{TRACKER_LABELS[mode]}\n{status}", fontsize=9)
        ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
        ax.legend(fontsize=6, loc='upper left')

    fig.suptitle('S008 3D Stochastic Pursuit — Helix Evader: 4 Tracker Modes', fontsize=11)
    plt.tight_layout()
    path = os.path.join(out_dir, 'trajectories_3d.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_per_axis_error(results_helix, out_dir):
    """Per-axis error ex, ey, ez vs time for kalman_3d and kalman_2d_althold."""
    fig, axes = plt.subplots(2, 3, figsize=(14, 8))
    modes_to_plot = ['kalman_3d', 'kalman_2d_althold']
    axis_labels = ['ex (m)', 'ey (m)', 'ez (m)']
    axis_colors = ['steelblue', 'mediumseagreen', 'darkorange']

    for row, mode in enumerate(modes_to_plot):
        res = results_helix[mode]
        for col, (axis_label, color) in enumerate(zip(axis_labels, axis_colors)):
            ax = axes[row][col]
            ax.plot(res['times'], res['err_xyz'][:, col],
                    color=color, linewidth=1.5)
            ax.set_xlabel('Time (s)')
            ax.set_ylabel(axis_label)
            ax.set_title(f'{TRACKER_LABELS[mode]} — {axis_label}', fontsize=9)
            ax.grid(True, alpha=0.3)
            if res['captured']:
                ax.axvline(res['cap_time'], color='red', linestyle='--',
                           linewidth=1.0, alpha=0.7, label='Capture')
                ax.legend(fontsize=7)

    fig.suptitle('S008 3D — Per-Axis Estimation Error (Helix Evader)', fontsize=11)
    plt.tight_layout()
    path = os.path.join(out_dir, 'per_axis_error.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_covariance_volume(results_helix, out_dir):
    """Covariance ellipsoid volume vs time for kalman_3d."""
    res = results_helix['kalman_3d']
    fig, ax = plt.subplots(figsize=(10, 5))
    ax.semilogy(res['times'], np.maximum(res['cov_vol'], 1e-10),
                color='firebrick', linewidth=2.0)
    if res['captured']:
        ax.axvline(res['cap_time'], color='black', linestyle='--',
                   linewidth=1.2, label=f"Capture {res['cap_time']:.2f}s")
        ax.legend(fontsize=9)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Covariance Ellipsoid Volume (m³, log scale)')
    ax.set_title('S008 3D — Kalman 3D: Covariance Ellipsoid Volume vs Time\n'
                 '(4π/3 × ∏√λᵢ(P_pos))', fontsize=10)
    ax.grid(True, which='both', alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'covariance_volume.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def plot_capture_table(all_results, out_dir):
    """Table: 4 modes × 3 evader types, capture time or 'missed'."""
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.axis('off')

    col_labels = [TRACKER_LABELS[m] for m in TRACKER_MODES]
    row_labels = EVADER_TYPES

    table_data = []
    cell_colors = []
    for et in EVADER_TYPES:
        row = []
        row_c = []
        for mode in TRACKER_MODES:
            res = all_results[et][mode]
            if res['captured']:
                row.append(f"{res['cap_time']:.2f}s")
                row_c.append('#d4efdf')
            else:
                row.append('missed')
                row_c.append('#f9ebea')
        table_data.append(row)
        cell_colors.append(row_c)

    table = ax.table(
        cellText=table_data,
        rowLabels=row_labels,
        colLabels=col_labels,
        cellLoc='center',
        loc='center',
        cellColours=cell_colors,
    )
    table.auto_set_font_size(False)
    table.set_fontsize(10)
    table.scale(1.2, 2.0)

    ax.set_title('S008 3D Stochastic Pursuit — Capture Time Table\n'
                 '(green=captured, red=missed, max time=30s)',
                 fontsize=10, pad=20)
    plt.tight_layout()
    path = os.path.join(out_dir, 'capture_table.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def save_animation(results_helix, evader_traj_helix, out_dir):
    """Animate kalman_3d tracking helix evader: estimated vs true."""
    res = results_helix['kalman_3d']
    n = len(res['p_traj'])
    et = evader_traj_helix[:n]
    step = 4

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    # Pre-compute limits
    all_pts = np.vstack([res['p_traj'], et, res['est_traj']])
    margin = 1.0
    ax.set_xlim(all_pts[:, 0].min() - margin, all_pts[:, 0].max() + margin)
    ax.set_ylim(all_pts[:, 1].min() - margin, all_pts[:, 1].max() + margin)
    ax.set_zlim(all_pts[:, 2].min() - margin, all_pts[:, 2].max() + margin)
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')

    line_true,  = ax.plot([], [], [], color='royalblue', linewidth=1.5,
                          label='Evader (true)', alpha=0.8)
    line_est,   = ax.plot([], [], [], color='cyan', linewidth=1.2,
                          linestyle='--', label='KF estimate', alpha=0.7)
    line_pursuer, = ax.plot([], [], [], color='red', linewidth=1.8,
                            label='Pursuer')
    dot_true,   = ax.plot([], [], [], 'bs', markersize=8, zorder=6)
    dot_est,    = ax.plot([], [], [], 'c^', markersize=7, zorder=6)
    dot_pursuer,= ax.plot([], [], [], 'r^', markersize=9, zorder=6)

    ax.legend(fontsize=8, loc='upper left')
    title = ax.set_title('S008 3D — Kalman Tracking: Helix Evader', fontsize=10)

    def update(i):
        si = min(i * step, n - 1)
        line_true.set_data(et[:si+1, 0], et[:si+1, 1])
        line_true.set_3d_properties(et[:si+1, 2])
        line_est.set_data(res['est_traj'][:si+1, 0], res['est_traj'][:si+1, 1])
        line_est.set_3d_properties(res['est_traj'][:si+1, 2])
        line_pursuer.set_data(res['p_traj'][:si+1, 0], res['p_traj'][:si+1, 1])
        line_pursuer.set_3d_properties(res['p_traj'][:si+1, 2])
        dot_true.set_data([et[si, 0]], [et[si, 1]])
        dot_true.set_3d_properties([et[si, 2]])
        dot_est.set_data([res['est_traj'][si, 0]], [res['est_traj'][si, 1]])
        dot_est.set_3d_properties([res['est_traj'][si, 2]])
        dot_pursuer.set_data([res['p_traj'][si, 0]], [res['p_traj'][si, 1]])
        dot_pursuer.set_3d_properties([res['p_traj'][si, 2]])
        t_now = res['times'][si]
        title.set_text(f'S008 3D — Kalman Tracking: Helix Evader  t={t_now:.2f}s')
        return [line_true, line_est, line_pursuer, dot_true, dot_est, dot_pursuer, title]

    n_frames = n // step
    ani = FuncAnimation(fig, update, frames=n_frames, interval=60, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer=PillowWriter(fps=16), dpi=100)
    plt.close()
    print(f'Saved: {path}')


# ── Main ─────────────────────────────────────────────────────

if __name__ == '__main__':
    rng = np.random.default_rng(42)
    out_dir = os.path.normpath(OUTPUT_DIR)

    # Run all combinations
    all_results = {}
    for evader_type in EVADER_TYPES:
        rng_traj = np.random.default_rng(99)
        evader_traj = get_evader_trajectory(evader_type, rng_traj)
        all_results[evader_type] = {}
        for mode in TRACKER_MODES:
            rng_sim = np.random.default_rng(7 + TRACKER_MODES.index(mode))
            res = run_simulation(mode, evader_traj, rng_sim)
            status = f"captured @ {res['cap_time']:.2f}s" if res['captured'] else 'timeout'
            print(f'[{evader_type:<12}][{mode:<20}]  {status}')
            all_results[evader_type][mode] = res

    # Helix results for detailed plots
    rng_helix = np.random.default_rng(99)
    helix_traj = get_evader_trajectory('helix', rng_helix)
    results_helix = all_results['helix']

    print('\nGenerating plots...')
    plot_trajectories_3d(results_helix, helix_traj, out_dir)
    plot_per_axis_error(results_helix, out_dir)
    plot_covariance_volume(results_helix, out_dir)
    plot_capture_table(all_results, out_dir)
    save_animation(results_helix, helix_traj, out_dir)
    print('Done.')
