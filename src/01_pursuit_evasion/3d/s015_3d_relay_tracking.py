"""
S015 3D Upgrade — Relay Tracking (Communication Relay)
========================================================
1 target drone following a helical trajectory + 3 observer/relay drones at
altitude tiers z=1.5, 3.5, 5.0 m.

Observers measure azimuth+elevation bearing with Gaussian noise.
When >=2 observers detect target: 3D bearing triangulation via closest-
approach midpoint.  Altitude-weighted SNR selects best pair.

Usage:
    conda activate drones
    python src/01_pursuit_evasion/3d/s015_3d_relay_tracking.py
"""
import sys, os, numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from matplotlib.animation import FuncAnimation, PillowWriter

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..'))

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..', '..', 'outputs',
    '01_pursuit_evasion', '3d', 's015_3d_relay_tracking',
)
os.makedirs(OUTPUT_DIR, exist_ok=True)

# ── Parameters ────────────────────────────────────────────────
SENSOR_POS = np.array([
    [ 0.0,  4.0, 1.5],    # Observer 0 — low tier
    [-3.46, -2.0, 3.5],   # Observer 1 — mid tier
    [ 3.46, -2.0, 5.0],   # Observer 2 — high tier
], dtype=float)

R_ZONE         = 8.0    # m — spherical coverage radius
SIGMA_ALPHA    = 0.05   # rad — azimuth noise
SIGMA_BETA     = 0.07   # rad — elevation noise
TARGET_SPEED   = 2.0    # m/s
HANDOFF_THRESH = 0.4
SIGMA_Z_ALT    = 2.0    # altitude-weighting bandwidth
DT             = 0.05   # s
T_MAX          = 25.0   # s
Z_MIN          = 0.5
Z_MAX          = 6.5

SENSOR_COLORS = ['tomato', 'peru', 'mediumseagreen']

RNG = np.random.default_rng(0)


# ── Target trajectory ─────────────────────────────────────────

def target_path_3d(t: float) -> np.ndarray:
    """Helical: circular orbit + altitude oscillation."""
    r, omega = 3.0, TARGET_SPEED / 3.0
    return np.array([
        r * np.cos(omega * t),
        r * np.sin(omega * t),
        2.0 + 1.5 * np.sin(0.4 * t),
    ])


# ── Sensor model ──────────────────────────────────────────────

def snr_3d(sensor_pos: np.ndarray, target_pos: np.ndarray, z_t: float) -> float:
    dist = np.linalg.norm(target_pos - sensor_pos)
    base_snr = max(0.0, 1.0 - dist / R_ZONE)
    alt_weight = float(np.exp(-0.5 * ((z_t - sensor_pos[2]) / SIGMA_Z_ALT) ** 2))
    return base_snr * alt_weight


def measure_bearing_3d(sensor_pos: np.ndarray, target_pos: np.ndarray,
                        sigma_a: float = SIGMA_ALPHA, sigma_b: float = SIGMA_BETA):
    """Returns (azimuth, elevation) in radians with Gaussian noise."""
    dx = target_pos[0] - sensor_pos[0]
    dy = target_pos[1] - sensor_pos[1]
    dz = target_pos[2] - sensor_pos[2]
    r_xy = np.sqrt(dx ** 2 + dy ** 2)
    alpha = np.arctan2(dy, dx)  + RNG.normal(0, sigma_a)
    beta  = np.arctan2(dz, r_xy + 1e-8) + RNG.normal(0, sigma_b)
    return alpha, beta


def bearing_to_direction(alpha: float, beta: float) -> np.ndarray:
    return np.array([
        np.cos(beta) * np.cos(alpha),
        np.cos(beta) * np.sin(alpha),
        np.sin(beta),
    ])


def triangulate_3d(p1: np.ndarray, d1: np.ndarray,
                    p2: np.ndarray, d2: np.ndarray):
    """Closest-approach midpoint between two bearing rays."""
    w = p1 - p2
    b = float(np.dot(d1, d2))
    denom = 1.0 - b ** 2
    if abs(denom) < 1e-8:
        return None   # parallel rays
    d_val = float(np.dot(d1, w))
    e_val = float(np.dot(d2, w))
    s1 = (b * e_val - d_val) / denom
    s2 = (e_val - b * d_val) / denom
    closest1 = p1 + s1 * d1
    closest2 = p2 + s2 * d2
    return 0.5 * (closest1 + closest2)


def compute_dop_3d(active_sensors: list, target_pos: np.ndarray) -> float:
    """3D DOP from bearing direction vectors."""
    dirs = []
    for sp in active_sensors:
        dv = target_pos - sp
        n = np.linalg.norm(dv)
        if n > 1e-8:
            dirs.append(dv / n)
    if len(dirs) < 2:
        return np.inf
    H = np.array(dirs)
    HtH = H.T @ H
    try:
        return float(np.sqrt(np.trace(np.linalg.inv(HtH))))
    except np.linalg.LinAlgError:
        return np.inf


# ── Simulation ────────────────────────────────────────────────

def run_simulation():
    n_steps = int(T_MAX / DT)
    times, true_pos, est_pos = [], [], []
    errors_xyz, errors_z = [], []
    active_sensor_log, snr_logs = [], [[], [], []]
    coverage_log     = [[], [], []]   # binary per observer
    handoff_times    = []
    dop_log          = []

    active_zone = 0
    last_known  = target_path_3d(0.0)

    for step in range(n_steps):
        t   = step * DT
        p_T = target_path_3d(t)
        z_T = p_T[2]

        # Compute SNR for each sensor
        snrs = [snr_3d(SENSOR_POS[i], p_T, z_T) for i in range(3)]
        for i in range(3):
            snr_logs[i].append(snrs[i])
            coverage_log[i].append(1 if snrs[i] > HANDOFF_THRESH else 0)

        # Zone handoff
        new_zone = int(np.argmax(snrs))
        if new_zone != active_zone:
            handoff_times.append(t)
            active_zone = new_zone

        # Pick best two sensors for triangulation
        ranked = list(np.argsort(snrs)[::-1])
        i_best, j_best = ranked[0], ranked[1]

        n_visible = sum(s > HANDOFF_THRESH for s in snrs)

        if n_visible >= 2:
            a_i, b_i = measure_bearing_3d(SENSOR_POS[i_best], p_T)
            a_j, b_j = measure_bearing_3d(SENSOR_POS[j_best], p_T)
            d_i = bearing_to_direction(a_i, b_i)
            d_j = bearing_to_direction(a_j, b_j)
            p_est = triangulate_3d(SENSOR_POS[i_best], d_i, SENSOR_POS[j_best], d_j)
            if p_est is None:
                p_est = last_known
            last_known = p_est.copy()
        elif n_visible == 1:
            # Single sensor: add measurement noise
            a_i, b_i = measure_bearing_3d(SENSOR_POS[i_best], p_T,
                                           sigma_a=SIGMA_ALPHA * 2,
                                           sigma_b=SIGMA_BETA * 2)
            d_i   = bearing_to_direction(a_i, b_i)
            dist_est = np.linalg.norm(p_T - SENSOR_POS[i_best])
            p_est = SENSOR_POS[i_best] + dist_est * d_i
            last_known = p_est.copy()
        else:
            # Dead reckoning
            p_est = last_known

        err_xyz = float(np.linalg.norm(p_est - p_T))
        err_z   = float(abs(p_est[2] - p_T[2]))

        # DOP
        active_sp = [SENSOR_POS[k] for k in range(3) if snrs[k] > HANDOFF_THRESH]
        dop       = compute_dop_3d(active_sp, p_T)

        times.append(t)
        true_pos.append(p_T.copy())
        est_pos.append(p_est.copy())
        errors_xyz.append(err_xyz)
        errors_z.append(err_z)
        active_sensor_log.append(active_zone)
        dop_log.append(dop)

    return (
        np.array(times), np.array(true_pos), np.array(est_pos),
        np.array(errors_xyz), np.array(errors_z),
        np.array(active_sensor_log),
        [np.array(s) for s in snr_logs],
        [np.array(c) for c in coverage_log],
        np.array(dop_log), handoff_times,
    )


# ── Plots ─────────────────────────────────────────────────────

def plot_trajectories_3d(times, true_pos, out_dir):
    fig = plt.figure(figsize=(12, 9))
    ax  = fig.add_subplot(111, projection='3d')

    # Target helix
    ax.plot(true_pos[:, 0], true_pos[:, 1], true_pos[:, 2],
            color='royalblue', linewidth=2.2, label='Target (helix)', zorder=5)
    ax.scatter(*true_pos[0], color='blue', s=80, marker='o', zorder=7, label='Target start')

    # Observer positions + range spheres
    u_sph = np.linspace(0, 2 * np.pi, 30)
    v_sph = np.linspace(0, np.pi, 20)
    for i, (sp, color) in enumerate(zip(SENSOR_POS, SENSOR_COLORS)):
        ax.scatter(*sp, color=color, s=200, marker='*', zorder=8, label=f'Observer {i} z={sp[2]}m')
        # Transparent sphere
        xs = sp[0] + R_ZONE * 0.5 * np.outer(np.cos(u_sph), np.sin(v_sph))
        ys = sp[1] + R_ZONE * 0.5 * np.outer(np.sin(u_sph), np.sin(v_sph))
        zs = sp[2] + R_ZONE * 0.5 * np.outer(np.ones_like(u_sph), np.cos(v_sph))
        ax.plot_surface(xs, ys, zs, color=color, alpha=0.05)

    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
    ax.set_title('S015 3D Relay Tracking — Target Helix + Observer Coverage')
    ax.legend(fontsize=8, loc='upper left')
    plt.tight_layout()
    path = os.path.join(out_dir, 'trajectories_3d.png')
    plt.savefig(path, dpi=150, bbox_inches='tight'); plt.close()
    print(f'Saved: {path}')


def plot_tracking_error(times, errors_xyz, errors_z, dop_log, handoff_times, out_dir):
    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

    # Panel 1: 3D error
    ax = axes[0]
    ax.plot(times, errors_xyz, color='steelblue', linewidth=1.4, label='3D error')
    mean_err = np.nanmean(errors_xyz)
    ax.axhline(mean_err, color='red', linestyle='--', linewidth=0.9,
               label=f'Mean = {mean_err:.3f}m')
    for ht in handoff_times:
        ax.axvline(ht, color='orange', linestyle=':', linewidth=1.0, alpha=0.7)
    if handoff_times:
        ax.axvline(handoff_times[0], color='orange', linestyle=':',
                   linewidth=1.0, alpha=0.7, label='Handoff event')
    ax.set_ylabel('3D Error (m)'); ax.legend(fontsize=8); ax.grid(True, alpha=0.3)
    ax.set_title('S015 3D Relay Tracking — Estimation Error vs Time')

    # Panel 2: z error
    ax = axes[1]
    ax.plot(times, errors_z, color='tomato', linewidth=1.4, label='z error')
    mean_z = np.nanmean(errors_z)
    ax.axhline(mean_z, color='darkred', linestyle='--', linewidth=0.9,
               label=f'Mean z = {mean_z:.3f}m')
    for ht in handoff_times:
        ax.axvline(ht, color='orange', linestyle=':', linewidth=1.0, alpha=0.7)
    ax.set_ylabel('z Error (m)'); ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

    # Panel 3: DOP
    ax = axes[2]
    dop_plot = np.where(np.isinf(dop_log), np.nan, dop_log)
    ax.plot(times, dop_plot, color='purple', linewidth=1.4, label='3D DOP')
    ax.set_ylabel('DOP'); ax.set_xlabel('Time (s)')
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

    plt.tight_layout()
    path = os.path.join(out_dir, 'tracking_error.png')
    plt.savefig(path, dpi=150); plt.close()
    print(f'Saved: {path}')


def plot_coverage_status(times, coverage_log, handoff_times, out_dir):
    fig, axes = plt.subplots(3, 1, figsize=(12, 6), sharex=True)
    for i, (ax, color) in enumerate(zip(axes, SENSOR_COLORS)):
        ax.fill_between(times, 0, coverage_log[i], step='mid',
                        color=color, alpha=0.7)
        ax.set_ylabel(f'Obs {i}\nz={SENSOR_POS[i,2]}m', fontsize=8)
        ax.set_ylim(-0.05, 1.3); ax.set_yticks([0, 1])
        ax.grid(True, alpha=0.3)
        for ht in handoff_times:
            ax.axvline(ht, color='orange', linestyle=':', linewidth=0.9, alpha=0.7)
    axes[-1].set_xlabel('Time (s)')
    axes[0].set_title('S015 3D Relay Tracking — Observer Coverage Status vs Time')
    plt.tight_layout()
    path = os.path.join(out_dir, 'coverage_status.png')
    plt.savefig(path, dpi=150); plt.close()
    print(f'Saved: {path}')


def plot_altitude_time(times, true_pos, out_dir):
    fig, ax = plt.subplots(figsize=(12, 5))
    ax.plot(times, true_pos[:, 2], color='royalblue', linewidth=2.0, label='Target z (helix)')
    for i, (sp, color) in enumerate(zip(SENSOR_POS, SENSOR_COLORS)):
        ax.axhline(sp[2], color=color, linestyle='--', linewidth=1.4,
                   label=f'Observer {i} z={sp[2]}m')
    ax.axhline(Z_MIN, color='gray', linestyle=':', alpha=0.5)
    ax.axhline(Z_MAX, color='gray', linestyle=':', alpha=0.5, label='z bounds')
    ax.set_xlabel('Time (s)'); ax.set_ylabel('Altitude z (m)')
    ax.set_title('S015 3D Relay Tracking — Altitude vs Time')
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'altitude_time.png')
    plt.savefig(path, dpi=150); plt.close()
    print(f'Saved: {path}')


def save_animation(times, true_pos, est_pos, active_sensor_log, handoff_times, out_dir):
    step = 4
    n_frames = max(1, len(times) // step)

    fig = plt.figure(figsize=(11, 9))
    ax  = fig.add_subplot(111, projection='3d')

    # Observer markers
    for i, (sp, color) in enumerate(zip(SENSOR_POS, SENSOR_COLORS)):
        ax.scatter(*sp, color=color, s=200, marker='*', zorder=8)
        ax.text(sp[0] + 0.2, sp[1] + 0.2, sp[2] + 0.1,
                f'O{i}', fontsize=9, color=color)

    ax.set_xlim(-5, 5); ax.set_ylim(-5, 5); ax.set_zlim(Z_MIN, Z_MAX)
    ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')

    true_line, = ax.plot([], [], [], color='royalblue', linewidth=2.0, label='Target')
    true_dot,  = ax.plot([], [], [], 'bo', markersize=10, zorder=9)
    est_dot,   = ax.plot([], [], [], 'rx', markersize=12, markeredgewidth=2.5,
                          zorder=10, label='Estimated')
    # Bearing lines to best two observers
    bear_lines = [ax.plot([], [], [], color=SENSOR_COLORS[k], linewidth=0.8, alpha=0.5)[0]
                  for k in range(3)]
    title = ax.set_title('')
    ax.legend(fontsize=8, loc='upper left')

    def update(fi):
        si = min(fi * step, len(times) - 1)
        t   = times[si]
        p_T = true_pos[si]
        p_E = est_pos[si]

        true_line.set_data(true_pos[:si+1, 0], true_pos[:si+1, 1])
        true_line.set_3d_properties(true_pos[:si+1, 2])
        true_dot.set_data([p_T[0]], [p_T[1]])
        true_dot.set_3d_properties([p_T[2]])
        est_dot.set_data([p_E[0]], [p_E[1]])
        est_dot.set_3d_properties([p_E[2]])

        snrs_now = [snr_3d(SENSOR_POS[k], p_T, p_T[2]) for k in range(3)]
        ranked   = list(np.argsort(snrs_now)[::-1])
        for k, bl in enumerate(bear_lines):
            if k in ranked[:2] and snrs_now[k] > HANDOFF_THRESH:
                sp = SENSOR_POS[k]
                a, b = measure_bearing_3d(sp, p_T, 0.0, 0.0)
                d = bearing_to_direction(a, b)
                ep = sp + R_ZONE * d
                bl.set_data([sp[0], ep[0]], [sp[1], ep[1]])
                bl.set_3d_properties([sp[2], ep[2]])
            else:
                bl.set_data([], [])
                bl.set_3d_properties([])

        err = float(np.linalg.norm(p_E - p_T))
        title.set_text(f'S015 3D Relay  t={t:.1f}s  Err={err:.2f}m')
        return [true_line, true_dot, est_dot, title] + bear_lines

    ani = FuncAnimation(fig, update, frames=n_frames, interval=50, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer=PillowWriter(fps=20), dpi=90)
    plt.close()
    print(f'Saved: {path}')


# ── Main ──────────────────────────────────────────────────────
if __name__ == '__main__':
    (times, true_pos, est_pos,
     errors_xyz, errors_z,
     active_sensor_log, snr_logs, coverage_log,
     dop_log, handoff_times) = run_simulation()

    print(f'Handoff events: {len(handoff_times)} at t={[f"{h:.2f}" for h in handoff_times]}')
    print(f'Mean 3D error: {np.nanmean(errors_xyz):.4f}m   '
          f'Mean z error: {np.nanmean(errors_z):.4f}m')
    print(f'Max 3D error: {np.nanmax(errors_xyz):.4f}m')

    out_dir = os.path.normpath(OUTPUT_DIR)
    plot_trajectories_3d(times, true_pos, out_dir)
    plot_tracking_error(times, errors_xyz, errors_z, dop_log, handoff_times, out_dir)
    plot_coverage_status(times, coverage_log, handoff_times, out_dir)
    plot_altitude_time(times, true_pos, out_dir)
    save_animation(times, true_pos, est_pos, active_sensor_log, handoff_times, out_dir)
    print('S015 3D complete.')
