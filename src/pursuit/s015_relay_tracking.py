"""
S015 Relay Tracking
====================
Three stationary sensor drones measure bearing (angle-only) to a moving
target.  Zone-based handoff selects the two highest-SNR sensors; two-line
triangulation estimates the target position.  Estimation error spikes at
handoff transitions.

Usage:
    conda activate drones
    python src/pursuit/s015_relay_tracking.py
"""

import sys, os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

# ── Parameters ───────────────────────────────────────────────
SENSOR_POS    = np.array([[ 0.0,  4.0, 3.0],   # S1 — top
                           [-3.46, -2.0, 3.0],  # S2 — bottom-left
                           [ 3.46, -2.0, 3.0]]) # S3 — bottom-right
R_ZONE        = 8.0    # m — coverage radius (wider for orbital path)
SIGMA_THETA   = 0.05   # rad — bearing noise (~3°)
TARGET_SPEED  = 2.0    # m/s
HANDOFF_THRESH = 0.4   # SNR threshold
DT            = 0.05   # s
T_MAX         = 20.0   # s

SENSOR_COLORS = ['red', 'darkorange', 'mediumseagreen']

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '01_pursuit_evasion', 's015_relay_tracking',
)

RNG = np.random.default_rng(0)


# ── Helpers ───────────────────────────────────────────────────

def target_path(t):
    """Circular orbit at radius 3 m through equilateral sensor triangle."""
    r     = 3.0
    omega = TARGET_SPEED / r      # ≈ 0.667 rad/s, period ≈ 9.4 s
    x = r * np.cos(omega * t)
    y = r * np.sin(omega * t)
    return np.array([x, y])


def snr(sensor_pos, target_pos, r_zone=R_ZONE):
    dist = np.linalg.norm(target_pos - sensor_pos[:2])
    return max(0.0, 1.0 - dist / r_zone)


def measure_bearing(sensor_pos, target_pos, sigma=SIGMA_THETA):
    dx = target_pos[0] - sensor_pos[0]
    dy = target_pos[1] - sensor_pos[1]
    return float(np.arctan2(dy, dx) + RNG.normal(0, sigma))


def triangulate(p1, theta1, p2, theta2):
    """Intersect two bearing lines; return None if nearly parallel."""
    t1 = np.tan(theta1); t2 = np.tan(theta2)
    A = np.array([[t1, -1.0], [t2, -1.0]])
    b = np.array([t1 * p1[0] - p1[1], t2 * p2[0] - p2[1]])
    if abs(np.linalg.det(A)) < 1e-4:
        return None
    return np.linalg.solve(A, b)


# ── Simulation ────────────────────────────────────────────────

def run_simulation():
    n_steps = int(T_MAX / DT)
    times, true_pos, est_pos = [], [], []
    errors, active_counts, active_sensors = [], [], []
    handoff_times = []

    current_primary = -1

    for step in range(n_steps):
        t    = step * DT
        p_T  = target_path(t)

        snrs = [snr(SENSOR_POS[i], p_T) for i in range(3)]
        active_count = sum(s > HANDOFF_THRESH for s in snrs)

        # Handoff: highest-SNR sensor becomes primary
        new_primary = int(np.argmax(snrs))
        if new_primary != current_primary:
            if current_primary >= 0:
                handoff_times.append(t)
            current_primary = new_primary

        # Triangulate using two highest-SNR sensors
        ranked = list(np.argsort(snrs)[::-1])
        i, j  = ranked[0], ranked[1]
        theta_i = measure_bearing(SENSOR_POS[i], p_T)
        theta_j = measure_bearing(SENSOR_POS[j], p_T)
        p_est = triangulate(SENSOR_POS[i, :2], theta_i,
                            SENSOR_POS[j, :2], theta_j)

        err = float(np.linalg.norm(p_est - p_T)) if p_est is not None else np.nan

        times.append(t); true_pos.append(p_T.copy())
        est_pos.append(p_est.copy() if p_est is not None else np.array([np.nan, np.nan]))
        errors.append(err); active_counts.append(active_count)
        active_sensors.append(current_primary)

    return (
        np.array(times), np.array(true_pos), np.array(est_pos),
        np.array(errors), np.array(active_counts), np.array(active_sensors),
        handoff_times,
    )


# ── Plots ─────────────────────────────────────────────────────

def plot_tracking_overview(times, true_pos, est_pos, active_sensors, handoff_times, out_dir):
    fig, ax = plt.subplots(figsize=(11, 8))

    # Coverage zones
    for i, (sp, color) in enumerate(zip(SENSOR_POS, SENSOR_COLORS)):
        circ = Circle(sp[:2], R_ZONE, fill=False, edgecolor=color,
                      linestyle='--', linewidth=1.2, alpha=0.6)
        ax.add_patch(circ)
        ax.scatter(*sp[:2], color=color, s=150, marker='*', zorder=6,
                   label=f'Sensor {i+1}')
        ax.text(sp[0]+0.2, sp[1]+0.2, f'S{i+1}', fontsize=9, color=color)

    # True path
    ax.plot(true_pos[:, 0], true_pos[:, 1], color='royalblue',
            linewidth=2.0, label='True path', zorder=3)

    # Estimated positions (colour by active sensor)
    for i, color in enumerate(SENSOR_COLORS):
        mask = active_sensors == i
        ep = est_pos[mask]
        valid = ~np.isnan(ep[:, 0])
        if valid.any():
            ax.scatter(ep[valid, 0], ep[valid, 1], color=color, s=10,
                       alpha=0.5, zorder=4)

    # Handoff markers
    for ht in handoff_times:
        idx = np.argmin(np.abs(times - ht))
        ax.axvline(x=true_pos[idx, 0], color='black', linestyle=':', linewidth=0.8, alpha=0.5)
        ax.scatter(*true_pos[idx], color='black', s=60, marker='|', zorder=7)

    ax.set_xlim(-7, 7); ax.set_ylim(-7, 7)
    ax.set_aspect('equal'); ax.grid(True, alpha=0.3)
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
    ax.set_title('S015 Relay Tracking — Top-Down View\n'
                 '(coloured dots = triangulation estimates per active sensor)', fontsize=10)
    ax.legend(fontsize=8, loc='upper right')
    plt.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'tracking_overview.png')
    plt.savefig(path, dpi=150); plt.close(); print(f'Saved: {path}')


def plot_error_and_coverage(times, errors, active_counts, handoff_times, out_dir):
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 7), sharex=True)

    # Shade handoff transition windows
    for ht in handoff_times:
        ax1.axvline(ht, color='orange', linestyle=':', linewidth=1.2, alpha=0.7)
        ax2.axvline(ht, color='orange', linestyle=':', linewidth=1.2, alpha=0.7)
    if handoff_times:
        ax1.axvline(handoff_times[0], color='orange', linestyle=':',
                    linewidth=1.2, alpha=0.7, label='Handoff event')

    ax1.plot(times, errors, color='steelblue', linewidth=1.5, label='Triangulation error')
    ax1.set_ylabel('Position Error (m)')
    ax1.set_title('S015 Relay Tracking — Estimation Error & Active Sensor Count vs Time')
    ax1.legend(fontsize=8); ax1.grid(True, alpha=0.3)
    mean_err = np.nanmean(errors)
    ax1.axhline(mean_err, color='red', linestyle='--', linewidth=1.0,
                label=f'Mean = {mean_err:.3f}m')
    ax1.legend(fontsize=8)

    ax2.step(times, active_counts, color='mediumseagreen', linewidth=1.8,
             where='mid', label='# active sensors')
    ax2.set_xlabel('Time (s)'); ax2.set_ylabel('Active sensor count')
    ax2.set_ylim(0, 4); ax2.legend(fontsize=8); ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    path = os.path.join(out_dir, 'error_and_coverage.png')
    plt.savefig(path, dpi=150); plt.close(); print(f'Saved: {path}')


def plot_snr_over_time(times, true_pos, handoff_times, out_dir):
    snr_logs = [[], [], []]
    for p_T in true_pos:
        for i in range(3):
            snr_logs[i].append(snr(SENSOR_POS[i], p_T))

    fig, ax = plt.subplots(figsize=(12, 5))
    for i, (snr_data, color) in enumerate(zip(snr_logs, SENSOR_COLORS)):
        ax.plot(times, snr_data, color=color, linewidth=1.8, label=f'Sensor {i+1}')
    ax.axhline(HANDOFF_THRESH, color='grey', linestyle='--', linewidth=1.0,
               label=f'Handoff threshold ({HANDOFF_THRESH})')
    for ht in handoff_times:
        ax.axvline(ht, color='orange', linestyle=':', linewidth=1.2, alpha=0.7)
    if handoff_times:
        ax.axvline(handoff_times[0], color='orange', linestyle=':',
                   linewidth=1.2, alpha=0.7, label='Handoff event')
    ax.set_xlabel('Time (s)'); ax.set_ylabel('SNR (proxy)')
    ax.set_title('S015 Relay Tracking — Sensor SNR vs Time')
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'snr_over_time.png')
    plt.savefig(path, dpi=150); plt.close(); print(f'Saved: {path}')


def save_animation(times, true_pos, est_pos, active_sensors, handoff_times, out_dir):
    import matplotlib.animation as animation

    step = 2
    n_frames = len(times) // step
    fig, ax = plt.subplots(figsize=(9, 8))

    # Static: coverage zones + sensor markers
    for i, (sp, color) in enumerate(zip(SENSOR_POS, SENSOR_COLORS)):
        circ = Circle(sp[:2], R_ZONE, fill=False, edgecolor=color,
                      linestyle='--', linewidth=1.2, alpha=0.5)
        ax.add_patch(circ)
        ax.scatter(*sp[:2], color=color, s=150, marker='*', zorder=6)
        ax.text(sp[0]+0.2, sp[1]+0.2, f'S{i+1}', fontsize=9, color=color)

    ax.set_xlim(-7, 7); ax.set_ylim(-7, 7)
    ax.set_aspect('equal'); ax.grid(True, alpha=0.3)
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')

    true_line, = ax.plot([], [], color='royalblue', linewidth=2.0, label='True path')
    true_dot,  = ax.plot([], [], 'bo', markersize=10, zorder=7)
    est_dot,   = ax.plot([], [], 'rx', markersize=10, zorder=8, markeredgewidth=2.0,
                         label='Estimated position')
    # Bearing lines (one per sensor, updated each frame)
    bear_lines = [ax.plot([], [], color=c, linewidth=0.8, alpha=0.5)[0]
                  for c in SENSOR_COLORS]
    title = ax.set_title('')
    ax.legend(fontsize=8, loc='upper right')

    def update(i):
        si = min(i * step, len(times)-1)
        t  = times[si]
        p_T = true_pos[si]
        p_E = est_pos[si]
        ai  = active_sensors[si]

        true_line.set_data(true_pos[:si+1, 0], true_pos[:si+1, 1])
        true_dot.set_data([float(p_T[0])], [float(p_T[1])])
        if not np.isnan(p_E[0]):
            est_dot.set_data([float(p_E[0])], [float(p_E[1])])

        # Update bearing lines from active sensors
        snrs_cur = [snr(SENSOR_POS[j], p_T) for j in range(3)]
        ranked   = list(np.argsort(snrs_cur)[::-1])
        for j, bl in enumerate(bear_lines):
            if j in ranked[:2]:
                sp = SENSOR_POS[j, :2]
                theta = measure_bearing(SENSOR_POS[j], p_T, sigma=0)
                length = R_ZONE
                bl.set_data([sp[0], sp[0]+length*np.cos(theta)],
                            [sp[1], sp[1]+length*np.sin(theta)])
            else:
                bl.set_data([], [])

        err = float(np.linalg.norm(p_E - p_T)) if not np.isnan(p_E[0]) else float('nan')
        title.set_text(f'S015 Relay Tracking  t={t:.2f}s  Active:S{ai+1}\n'
                       f'Triangulation err={err:.3f}m')
        return [true_line, true_dot, est_dot, title] + bear_lines

    ani = animation.FuncAnimation(fig, update, frames=n_frames, interval=50, blit=True)
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer='pillow', fps=20, dpi=100); plt.close(); print(f'Saved: {path}')


if __name__ == '__main__':
    times, true_pos, est_pos, errors, active_counts, active_sensors, handoff_times = run_simulation()
    print(f'Handoff events: {len(handoff_times)} at t={[f"{h:.2f}" for h in handoff_times]}')
    print(f'Mean triangulation error: {np.nanmean(errors):.4f} m')
    print(f'Max triangulation error:  {np.nanmax(errors):.4f} m')

    out_dir = os.path.normpath(OUTPUT_DIR)
    plot_tracking_overview(times, true_pos, est_pos, active_sensors, handoff_times, out_dir)
    plot_error_and_coverage(times, errors, active_counts, handoff_times, out_dir)
    plot_snr_over_time(times, true_pos, handoff_times, out_dir)
    save_animation(times, true_pos, est_pos, active_sensors, handoff_times, out_dir)
