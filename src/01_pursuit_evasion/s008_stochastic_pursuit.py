"""
S008 Stochastic Pursuit — Kalman Filter Tracking
==================================================
Evader moves under a constant-velocity model with Gaussian acceleration noise.
Pursuer receives noisy position measurements (sigma=0.3 m) and compares:

  - Kalman filter : smooth position + velocity estimate
  - Naive         : aim at raw noisy measurement
  - Oracle        : perfect state knowledge (upper bound)

Usage:
    conda activate drones
    python src/01_pursuit_evasion/s008_stochastic_pursuit.py
"""

import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from src.base.drone_base import DroneBase

# ── Parameters ───────────────────────────────────────────────
Q_PROCESS   = 0.1    # m²/s³ — evader process noise intensity
SIGMA_MEAS  = 0.3    # m     — measurement noise std (per axis)
ACCEL_STD   = 0.5    # m/s²  — evader random acceleration std
V_PURSUER   = 5.0    # m/s
V_EVADER    = 3.0    # m/s (mean)
R0          = 8.0    # m
CAPTURE_R   = 0.15   # m
DT          = 1 / 48
MAX_TIME    = 25.0

INIT_PURSUER = np.array([0.0, 0.0, 2.0])
INIT_EVADER  = np.array([R0,  0.0, 2.0])
EVADER_VELO  = np.array([V_EVADER, 0.0, 0.0])

STRATEGIES = ['kalman', 'naive', 'oracle']

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '01_pursuit_evasion', 's008_stochastic_pursuit',
)

RNG = np.random.default_rng(7)


# ── Kalman Filter ────────────────────────────────────────────

class KalmanFilter3D:
    """Linear Kalman filter for 3-D position+velocity tracking."""

    def __init__(self, dt, q, sigma_meas):
        I3 = np.eye(3)
        Z3 = np.zeros((3, 3))
        self.F = np.block([[I3, dt * I3], [Z3, I3]])
        self.H = np.hstack([I3, Z3])
        self.Q = q * np.block([
            [dt ** 3 / 3 * I3, dt ** 2 / 2 * I3],
            [dt ** 2 / 2 * I3, dt * I3],
        ])
        self.R = sigma_meas ** 2 * I3
        self.x = np.zeros(6)   # [px,py,pz, vx,vy,vz]
        self.P = np.eye(6) * 1.0

    def init_state(self, pos, vel=None):
        self.x[:3] = pos
        if vel is not None:
            self.x[3:] = vel

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
        return self.x[:3].copy(), self.P[:3, :3].copy()


# ── Simulation ───────────────────────────────────────────────

def run_simulation(strategy, evader_traj_fixed):
    """
    Re-uses the same evader trajectory for fair comparison.

    Returns:
        p_traj      (N,3)
        est_pos     (N,3)  — pursuer's estimate of evader position
        meas_log    (N,3)  — raw noisy measurements
        est_err     (N,)   — |true - estimated|
        cov_log     (N,3,3)— KF position covariance (zeros for non-KF)
        times       (N,)
        captured    bool
        cap_time    float or None
    """
    pursuer = DroneBase(INIT_PURSUER.copy(), max_speed=V_PURSUER, dt=DT)

    kf = KalmanFilter3D(DT, Q_PROCESS, SIGMA_MEAS)
    kf.init_state(evader_traj_fixed[0], EVADER_VELO.copy())

    p_traj_list = [pursuer.pos.copy()]
    est_list    = [evader_traj_fixed[0].copy()]
    meas_list   = [evader_traj_fixed[0].copy()]
    err_list    = [0.0]
    cov_list    = [np.zeros((3, 3))]
    time_list   = [0.0]

    captured = False
    cap_time = None
    N = len(evader_traj_fixed)

    for step in range(1, N):
        t = step * DT
        true_pos_e = evader_traj_fixed[step]

        # Noisy measurement
        meas = true_pos_e + RNG.normal(0, SIGMA_MEAS, 3)

        # Estimate evader position
        if strategy == 'kalman':
            est_pos_e, cov = kf.step(meas)
            cov_list.append(cov.copy())
        elif strategy == 'naive':
            est_pos_e = meas.copy()
            cov_list.append(np.zeros((3, 3)))
        else:  # oracle
            est_pos_e = true_pos_e.copy()
            cov_list.append(np.zeros((3, 3)))

        # Pursuer command — aim at estimate
        diff = est_pos_e - pursuer.pos
        n    = np.linalg.norm(diff)
        cmd  = V_PURSUER * diff / n if n > 1e-8 else np.zeros(3)
        pursuer.step(cmd)

        err = np.linalg.norm(true_pos_e - est_pos_e)

        p_traj_list.append(pursuer.pos.copy())
        est_list.append(est_pos_e.copy())
        meas_list.append(meas.copy())
        err_list.append(err)
        time_list.append(t)

        if np.linalg.norm(pursuer.pos - true_pos_e) < CAPTURE_R:
            captured = True
            cap_time = t
            break

    return (
        np.array(p_traj_list),
        np.array(est_list),
        np.array(meas_list),
        np.array(err_list),
        np.array(cov_list),
        np.array(time_list),
        captured,
        cap_time,
    )


def generate_evader_trajectory():
    """Simulate evader once with stochastic acceleration."""
    pos  = INIT_EVADER.copy().astype(float)
    vel  = EVADER_VELO.copy().astype(float)
    traj = [pos.copy()]
    max_steps = int(MAX_TIME / DT)
    for _ in range(max_steps):
        acc  = RNG.normal(0, ACCEL_STD, 3)
        acc[2] = 0.0   # keep altitude fixed
        vel  = vel + acc * DT
        # clamp speed
        spd = np.linalg.norm(vel)
        if spd > V_EVADER * 2:
            vel = vel / spd * V_EVADER * 2
        pos  = pos + vel * DT
        traj.append(pos.copy())
    return np.array(traj)


# ── Plots ────────────────────────────────────────────────────

def plot_trajectories(results, evader_traj, out_dir):
    colors = ['steelblue', 'darkorange', 'mediumseagreen']
    labels = ['Kalman Filter', 'Naive (noisy)', 'Oracle']

    fig = plt.figure(figsize=(16, 5))
    for idx, (res, color, label) in enumerate(zip(results, colors, labels)):
        p_traj, est_pos, meas_log, est_err, cov_log, times, captured, cap_t = res

        ax = fig.add_subplot(1, 3, idx + 1)
        n = len(p_traj)
        et = evader_traj[:n]

        # Raw measurements (sparse)
        ax.scatter(meas_log[::8, 0], meas_log[::8, 1],
                   color='lightcoral', s=10, zorder=2, label='Meas (noisy)', alpha=0.6)
        ax.plot(et[:, 0], et[:, 1], color='royalblue',
                linewidth=1.8, zorder=3, label='Evader (true)')
        ax.plot(est_pos[:, 0], est_pos[:, 1], color=color,
                linewidth=1.2, linestyle='--', zorder=4, alpha=0.8, label='Estimate')
        ax.plot(p_traj[:, 0], p_traj[:, 1], color='red',
                linewidth=1.8, zorder=5, label='Pursuer')

        ax.scatter(*p_traj[0, :2],  color='red',  s=60, zorder=6)
        ax.scatter(*et[0, :2],      color='blue', s=60, zorder=6)
        if captured:
            ax.scatter(*p_traj[-1, :2], color='black', s=100, marker='X', zorder=7,
                       label=f'Captured {cap_t:.2f}s')
            status = f'Captured {cap_t:.2f} s'
        else:
            status = 'Timeout'

        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
        ax.set_title(f'{label}\n{status}', fontsize=9)
        ax.legend(fontsize=7, loc='upper left')

    fig.suptitle('S008 Stochastic Pursuit — XY Trajectories', fontsize=11)
    plt.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'trajectories_xy.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_estimation_error(results, out_dir):
    colors = ['steelblue', 'darkorange', 'mediumseagreen']
    labels = ['Kalman Filter', 'Naive (noisy)', 'Oracle']

    fig, ax = plt.subplots(figsize=(12, 5))
    for res, color, label in zip(results, colors, labels):
        _, _, _, est_err, _, times, captured, cap_t = res
        ax.plot(times, est_err, color=color, linewidth=1.8, label=label)
        if captured:
            ax.axvline(cap_t, color=color, linestyle='--', linewidth=1.0, alpha=0.7)

    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Estimation Error (m)')
    ax.set_title('S008 Stochastic Pursuit — Evader Position Estimation Error vs Time',
                 fontsize=10)
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'estimation_error.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def plot_covariance_ellipses(results, evader_traj, out_dir):
    """Plot KF uncertainty ellipses sampled every 0.5 s."""
    res = results[0]   # KF only
    p_traj, est_pos, meas_log, est_err, cov_log, times, captured, cap_t = res
    n = len(p_traj)
    et = evader_traj[:n]

    fig, ax = plt.subplots(figsize=(10, 7))
    ax.plot(et[:, 0], et[:, 1], color='royalblue', linewidth=1.8, label='Evader (true)')
    ax.plot(est_pos[:, 0], est_pos[:, 1], color='steelblue',
            linewidth=1.4, linestyle='--', alpha=0.7, label='KF estimate')
    ax.plot(p_traj[:, 0], p_traj[:, 1], color='red',
            linewidth=1.8, label='Pursuer')

    sample_dt = 0.5   # s
    sample_step = max(1, int(sample_dt / DT))
    theta = np.linspace(0, 2 * np.pi, 60)

    for k in range(0, n, sample_step):
        cov2d = cov_log[k][:2, :2]
        try:
            vals, vecs = np.linalg.eigh(cov2d)
            vals = np.maximum(vals, 0)
            w, h = 2 * np.sqrt(vals)
            ang  = np.degrees(np.arctan2(*vecs[:, -1][::-1]))
            from matplotlib.patches import Ellipse
            ell = Ellipse(xy=est_pos[k, :2], width=w, height=h,
                          angle=ang, edgecolor='steelblue', facecolor='none',
                          linewidth=0.8, alpha=0.5)
            ax.add_patch(ell)
        except Exception:
            pass

    if captured:
        ax.scatter(*p_traj[-1, :2], color='black', s=100, marker='X',
                   label=f'Captured {cap_t:.2f}s')

    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
    ax.set_title('S008 Stochastic Pursuit — Kalman Filter\n'
                 'Uncertainty ellipses sampled every 0.5 s', fontsize=10)
    ax.legend(fontsize=9)
    plt.tight_layout()
    path = os.path.join(out_dir, 'covariance_ellipses.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def plot_capture_bar(results, out_dir):
    labels     = ['Kalman\nFilter', 'Naive\n(noisy)', 'Oracle']
    colors     = ['steelblue', 'darkorange', 'mediumseagreen']
    cap_times  = []
    bar_colors = []
    anns       = []

    for res, color in zip(results, colors):
        _, _, _, _, _, times, captured, cap_t = res
        if captured:
            cap_times.append(cap_t)
            bar_colors.append(color)
            anns.append(f'{cap_t:.2f} s')
        else:
            cap_times.append(MAX_TIME)
            bar_colors.append('tomato')
            anns.append('Timeout')

    fig, ax = plt.subplots(figsize=(8, 5))
    bars = ax.bar(labels, cap_times, color=bar_colors, edgecolor='black', width=0.4)
    for bar, ann in zip(bars, anns):
        ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.3,
                ann, ha='center', va='bottom', fontsize=10, fontweight='bold')
    ax.set_ylabel('Capture Time (s)')
    ax.set_title('S008 Stochastic Pursuit — Capture Time by Strategy', fontsize=10)
    ax.set_ylim(0, MAX_TIME * 1.2)
    ax.grid(axis='y', alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'capture_bar.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def save_animation(results, evader_traj, out_dir):
    import matplotlib.animation as animation

    colors = ['steelblue', 'darkorange', 'mediumseagreen']
    labels = ['Kalman Filter', 'Naive', 'Oracle']

    max_len = max(len(res[0]) for res in results)
    step = 3

    fig, axes = plt.subplots(1, 3, figsize=(15, 5))

    lines_p, lines_e, lines_est = [], [], []
    dots_p,  dots_e,  dots_est  = [], [], []
    titles = []

    for ax, color, label, res in zip(axes, colors, labels, results):
        p_traj, est_pos, meas_log = res[0], res[1], res[2]
        n = len(p_traj)
        et = evader_traj[:n]
        all_pts = np.vstack([p_traj, et, est_pos])
        lo = all_pts.min(axis=0)[:2] - 1.5
        hi = all_pts.max(axis=0)[:2] + 1.5

        ax.set_xlim(lo[0], hi[0]); ax.set_ylim(lo[1], hi[1])
        ax.set_aspect('equal'); ax.grid(True, alpha=0.3)
        ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
        ax.scatter(*p_traj[0, :2], color='red',  s=60, zorder=5)
        ax.scatter(*et[0, :2],     color='blue', s=60, zorder=5)

        lp,  = ax.plot([], [], color='red',       linewidth=1.8, alpha=0.8)
        le,  = ax.plot([], [], color='royalblue', linewidth=1.8, alpha=0.8)
        lest,= ax.plot([], [], color=color, linewidth=1.2,
                       linestyle='--', alpha=0.7)
        dp,  = ax.plot([], [], 'r^', markersize=9, zorder=6)
        de,  = ax.plot([], [], 'bs', markersize=9, zorder=6)
        dest,= ax.plot([], [], 'x', color=color, markersize=8,
                       zorder=7, markeredgewidth=2)
        ti   = ax.set_title(label, fontsize=9)

        lines_p.append(lp); lines_e.append(le); lines_est.append(lest)
        dots_p.append(dp); dots_e.append(de); dots_est.append(dest)
        titles.append(ti)

    n_frames = max_len // step

    def update(i):
        artists = []
        for idx, res in enumerate(results):
            p_traj, est_pos, meas_log, est_err, cov_log, times, captured, cap_t = res
            si = min(i * step, len(p_traj) - 1)
            done = (i * step >= len(p_traj) - 1)
            n = len(p_traj)
            et = evader_traj[:n]

            px = p_traj[:si+1, 0]; py = p_traj[:si+1, 1]
            ex = et[:si+1, 0];     ey = et[:si+1, 1]
            epx = est_pos[:si+1, 0]; epy = est_pos[:si+1, 1]
            lines_p[idx].set_data(px, py)
            lines_e[idx].set_data(ex, ey)
            lines_est[idx].set_data(epx, epy)
            dots_p[idx].set_data([float(px[-1])], [float(py[-1])])
            dots_e[idx].set_data([float(ex[-1])], [float(ey[-1])])
            dots_est[idx].set_data([float(epx[-1])], [float(epy[-1])])

            t = times[si]; err = est_err[si]
            if done:
                st = '✓ Captured [DONE]' if captured else 'Timeout [DONE]'
                titles[idx].set_text(f'{labels[idx]}  t={t:.1f}s\n{st}')
            else:
                titles[idx].set_text(
                    f'{labels[idx]}  t={t:.1f}s\nest_err={err:.2f}m')
            artists += [lines_p[idx], lines_e[idx], lines_est[idx],
                        dots_p[idx], dots_e[idx], dots_est[idx], titles[idx]]
        return artists

    ani = animation.FuncAnimation(fig, update, frames=n_frames,
                                  interval=60, blit=True)
    fig.suptitle('S008 Stochastic Pursuit — Kalman vs Naive vs Oracle\n'
                 '(red=pursuer, blue=evader true, dashed=estimate)',
                 fontsize=10)
    plt.tight_layout()

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer='pillow', fps=16, dpi=100)
    plt.close()
    print(f'Saved: {path}')


# ── Main ─────────────────────────────────────────────────────

if __name__ == '__main__':
    evader_traj = generate_evader_trajectory()
    print(f'Evader trajectory: {len(evader_traj)} steps ({len(evader_traj)*DT:.1f} s)')

    results = []
    for strategy in STRATEGIES:
        res = run_simulation(strategy, evader_traj)
        _, _, _, est_err, _, times, captured, cap_t = res
        status = f'captured @ {cap_t:.2f} s' if captured else f'timeout @ {times[-1]:.2f} s'
        mean_err = np.mean(est_err)
        print(f'[{strategy:<10}]  {status}  |  mean est_err={mean_err:.3f} m')
        results.append(res)

    out_dir = os.path.normpath(OUTPUT_DIR)
    plot_trajectories(results, evader_traj, out_dir)
    plot_estimation_error(results, out_dir)
    plot_covariance_ellipses(results, evader_traj, out_dir)
    plot_capture_bar(results, out_dir)
    save_animation(results, evader_traj, out_dir)
