"""
S002 3D Upgrade — Evasive Maneuver
====================================
Pursuer (Pure Pursuit, full 3D) vs Evader with 4 different 3D evasion strategies:
  1. horizontal_perp  — perpendicular in x-y only (2D baseline)
  2. perp_3d          — Gram-Schmidt 3D perpendicular to LOS
  3. helix            — constant angular rate + climbing
  4. dive_and_run     — drop to 0.5 m then straight escape

Usage:
    conda activate drones
    python src/01_pursuit_evasion/3d/s002_3d_evasive_maneuver.py
"""
import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from matplotlib.animation import FuncAnimation, PillowWriter

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..'))
from src.base.drone_base import DroneBase

# ── Scenario parameters ─────────────────────────────────────
PURSUER_SPEED = 5.0
EVADER_SPEED  = 3.5
DT            = 1 / 48
MAX_TIME      = 30.0
CAPTURE_R     = 0.15

INIT_PURSUER = np.array([-2.0, 0.0, 2.0])
INIT_EVADER  = np.array([ 2.0, 0.0, 2.0])

Z_MIN = 0.3
Z_MAX = 8.0
ARENA = 8.0

# Helix parameters
HELIX_ALPHA   = np.radians(30.0)   # pitch angle
R_HELIX       = 1.5                # helix radius (m)
OMEGA_HELIX   = EVADER_SPEED * np.cos(HELIX_ALPHA) / R_HELIX

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..', '...',
    'outputs', '01_pursuit_evasion', '3d', 's002_3d_evasive_maneuver',
)
OUTPUT_DIR = os.path.normpath(os.path.join(
    os.path.dirname(__file__), '..', '..', '..',
    'outputs', '01_pursuit_evasion', '3d', 's002_3d_evasive_maneuver',
))
os.makedirs(OUTPUT_DIR, exist_ok=True)


# ── Evader strategy functions ────────────────────────────────

def strategy_horizontal_perp(pos_e, pos_p, v_max, t):
    """2D baseline: perpendicular in x-y plane, z constant."""
    r_xy = pos_e[:2] - pos_p[:2]
    norm_xy = np.linalg.norm(r_xy) + 1e-8
    r_hat = r_xy / norm_xy
    perp = np.array([-r_hat[1], r_hat[0], 0.0])
    return v_max * perp


def strategy_perp_3d(pos_e, pos_p, v_max, t):
    """Gram-Schmidt: perpendicular to 3D LOS in the plane containing z-hat."""
    r3 = pos_e - pos_p
    norm_r = np.linalg.norm(r3) + 1e-8
    r_hat = r3 / norm_r
    z_hat = np.array([0.0, 0.0, 1.0])
    e_perp = z_hat - np.dot(z_hat, r_hat) * r_hat
    if np.linalg.norm(e_perp) < 1e-4:
        # fallback: near-singular — use x_hat
        x_hat = np.array([1.0, 0.0, 0.0])
        e_perp = x_hat - np.dot(x_hat, r_hat) * r_hat
    e_perp = e_perp / (np.linalg.norm(e_perp) + 1e-8)
    return v_max * e_perp


def strategy_helix(pos_e, pos_p, v_max, t):
    """Helix: angular rotation in x-y + vertical climb."""
    vz = v_max * np.sin(HELIX_ALPHA)
    # horizontal speed magnitude
    vh = v_max * np.cos(HELIX_ALPHA)
    vx = -vh * np.sin(OMEGA_HELIX * t)
    vy =  vh * np.cos(OMEGA_HELIX * t)
    return np.array([vx, vy, vz])


def strategy_dive_and_run(pos_e, pos_p, v_max, t):
    """Dive to 0.5 m in first 0.5 s, then straight escape from pursuer."""
    if t < 0.5:
        # Dive phase: move downward aggressively
        target_z = 0.5
        dz = target_z - pos_e[2]
        # horizontal: escape in x-y too
        r_xy = pos_e[:2] - pos_p[:2]
        norm_xy = np.linalg.norm(r_xy) + 1e-8
        r_hat_xy = r_xy / norm_xy
        # blend: mostly dive, partially escape
        v = np.array([r_hat_xy[0] * 0.3, r_hat_xy[1] * 0.3, np.sign(dz) * 0.95])
        v_norm = np.linalg.norm(v) + 1e-8
        return v_max * v / v_norm
    else:
        # Run phase: straight escape in 3D from pursuer
        r3 = pos_e - pos_p
        return v_max * r3 / (np.linalg.norm(r3) + 1e-8)


# ── Simulation ───────────────────────────────────────────────

def clamp_altitude(pos):
    """Keep drone within altitude bounds."""
    pos[2] = np.clip(pos[2], Z_MIN, Z_MAX)
    return pos


def run_simulation(evader_fn):
    pursuer = DroneBase(INIT_PURSUER, max_speed=PURSUER_SPEED, dt=DT)
    evader  = DroneBase(INIT_EVADER,  max_speed=EVADER_SPEED,  dt=DT)

    max_steps = int(MAX_TIME / DT)
    captured = False
    capture_time = None
    los_angles = []   # angle between evader velocity and LOS

    for step in range(max_steps):
        t = step * DT

        dist = np.linalg.norm(pursuer.pos - evader.pos)
        if dist < CAPTURE_R:
            captured = True
            capture_time = t
            break

        # Pursuer: pure pursuit in full 3D
        r_pe = evader.pos - pursuer.pos
        v_pursuer = PURSUER_SPEED * r_pe / (np.linalg.norm(r_pe) + 1e-8)
        pursuer.step(v_pursuer)
        pursuer.pos = clamp_altitude(pursuer.pos)
        pursuer.trajectory[-1] = pursuer.pos.copy()

        # Evader: apply strategy
        v_evader = evader_fn(evader.pos, pursuer.pos, EVADER_SPEED, t)
        evader.step(v_evader)
        evader.pos = clamp_altitude(evader.pos)
        evader.trajectory[-1] = evader.pos.copy()

        # Log LOS angle: angle between evader velocity and LOS vector
        los = evader.pos - pursuer.pos
        los_norm = np.linalg.norm(los) + 1e-8
        v_e_norm = np.linalg.norm(v_evader) + 1e-8
        cos_theta = np.dot(v_evader, los) / (v_e_norm * los_norm)
        cos_theta = np.clip(cos_theta, -1.0, 1.0)
        los_angles.append(np.degrees(np.arccos(cos_theta)))

    return (
        pursuer.get_trajectory(),
        evader.get_trajectory(),
        captured,
        capture_time,
        np.array(los_angles),
    )


# ── Plotting ─────────────────────────────────────────────────

STRATEGY_NAMES = ['horizontal_perp', 'perp_3d', 'helix', 'dive_and_run']
STRATEGY_LABELS = ['Horizontal Perp\n(2D baseline)', '3D Perp\n(Gram-Schmidt)', 'Helix\n(climb)', 'Dive-and-Run']


def plot_trajectories_3d(results):
    fig = plt.figure(figsize=(20, 5))
    for i, (name, label, p_traj, e_traj, captured, cap_time, _) in enumerate(results):
        ax = fig.add_subplot(1, 4, i + 1, projection='3d')
        ax.plot(p_traj[:, 0], p_traj[:, 1], p_traj[:, 2],
                color='red', linewidth=1.5, label='Pursuer')
        ax.plot(e_traj[:, 0], e_traj[:, 1], e_traj[:, 2],
                color='blue', linewidth=1.5, label='Evader')
        ax.scatter(*p_traj[0], color='red',  s=50, marker='o')
        ax.scatter(*e_traj[0], color='blue', s=50, marker='o')
        if captured:
            ax.scatter(*p_traj[-1], color='black', s=120, marker='X',
                       label=f'Cap {cap_time:.1f}s')
        ax.set_xlabel('X (m)', fontsize=7)
        ax.set_ylabel('Y (m)', fontsize=7)
        ax.set_zlabel('Z (m)', fontsize=7)
        ax.set_zlim(0, Z_MAX)
        cap_str = f'Capture: {cap_time:.2f}s' if captured else 'Not captured'
        ax.set_title(f'{label}\n{cap_str}', fontsize=9)
        ax.legend(loc='upper left', fontsize=6)
    fig.suptitle('S002 3D — Evasive Maneuver: Trajectories', fontsize=13)
    plt.tight_layout()
    path = os.path.join(OUTPUT_DIR, 'trajectories_3d.png')
    plt.savefig(path, dpi=120, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_altitude_time(results):
    fig, axes = plt.subplots(2, 2, figsize=(12, 8), sharex=True)
    axes = axes.flatten()
    for i, (name, label, p_traj, e_traj, captured, cap_time, _) in enumerate(results):
        ax = axes[i]
        n = min(len(p_traj), len(e_traj))
        t_arr = np.arange(n) * DT
        ax.plot(t_arr, p_traj[:n, 2], color='red',  linewidth=1.5, label='Pursuer z')
        ax.plot(t_arr, e_traj[:n, 2], color='blue', linewidth=1.5, label='Evader z')
        ax.axhline(Z_MIN, color='gray', linestyle=':', linewidth=0.8, alpha=0.6)
        ax.axhline(Z_MAX, color='gray', linestyle=':', linewidth=0.8, alpha=0.6)
        if captured:
            ax.axvline(cap_time, color='black', linestyle='--', linewidth=1.0, alpha=0.7)
        ax.set_ylabel('Altitude z (m)')
        ax.set_title(label, fontsize=9)
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)
        ax.set_ylim(-0.2, Z_MAX + 0.5)
    axes[-2].set_xlabel('Time (s)')
    axes[-1].set_xlabel('Time (s)')
    fig.suptitle('S002 3D — Altitude vs Time', fontsize=13)
    plt.tight_layout()
    path = os.path.join(OUTPUT_DIR, 'altitude_time.png')
    plt.savefig(path, dpi=120, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_capture_times(results):
    labels = [label.replace('\n', ' ') for _, label, *_ in results]
    times  = []
    colors = []
    for _, _, _, _, captured, cap_time, _ in results:
        times.append(cap_time if captured else MAX_TIME)
        colors.append('steelblue' if captured else 'lightcoral')

    fig, ax = plt.subplots(figsize=(9, 5))
    bars = ax.bar(labels, times, color=colors, edgecolor='black', linewidth=0.8)
    for bar, (_, _, _, _, captured, cap_time, _) in zip(bars, results):
        lbl = f'{cap_time:.2f}s' if captured else 'Timeout'
        ax.text(bar.get_x() + bar.get_width() / 2,
                bar.get_height() + 0.3, lbl,
                ha='center', va='bottom', fontsize=9)
    ax.axhline(MAX_TIME, color='red', linestyle='--', linewidth=1.0, alpha=0.6,
               label=f'Max time ({MAX_TIME:.0f}s)')
    ax.set_ylabel('Capture Time (s)')
    ax.set_title(f'S002 3D — Capture Time by Evasion Strategy\n'
                 f'Pursuer {PURSUER_SPEED} m/s vs Evader {EVADER_SPEED} m/s')
    ax.set_ylim(0, MAX_TIME * 1.2)
    ax.legend()
    ax.grid(True, axis='y', alpha=0.3)
    plt.tight_layout()
    path = os.path.join(OUTPUT_DIR, 'capture_times.png')
    plt.savefig(path, dpi=120, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_los_angle(results):
    fig, axes = plt.subplots(2, 2, figsize=(12, 8), sharex=True)
    axes = axes.flatten()
    for i, (name, label, p_traj, e_traj, captured, cap_time, los_angles) in enumerate(results):
        ax = axes[i]
        n = len(los_angles)
        t_arr = np.arange(n) * DT
        ax.plot(t_arr, los_angles, color='darkorange', linewidth=1.5)
        ax.axhline(90.0, color='green', linestyle='--', linewidth=1.0, alpha=0.7,
                   label='90° (optimal)')
        if captured:
            ax.axvline(cap_time, color='black', linestyle='--', linewidth=1.0, alpha=0.7,
                       label=f'Captured {cap_time:.1f}s')
        ax.set_ylabel('LOS Angle (°)')
        ax.set_title(label, fontsize=9)
        ax.set_ylim(0, 200)
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)
    axes[-2].set_xlabel('Time (s)')
    axes[-1].set_xlabel('Time (s)')
    fig.suptitle('S002 3D — Evader Velocity vs LOS Angle', fontsize=13)
    plt.tight_layout()
    path = os.path.join(OUTPUT_DIR, 'los_angle.png')
    plt.savefig(path, dpi=120, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def save_animation(results):
    """Animate the helix strategy (index 2)."""
    _, _, p_traj, e_traj, captured, cap_time, _ = results[2]   # helix

    step_skip = 4
    p_frames = p_traj[::step_skip]
    e_frames = e_traj[::step_skip]
    n_frames = min(len(p_frames), len(e_frames))

    all_pts = np.vstack([p_traj, e_traj])
    margin = 0.5
    lo = all_pts.min(axis=0) - margin
    hi = all_pts.max(axis=0) + margin

    fig = plt.figure(figsize=(8, 7))
    ax = fig.add_subplot(111, projection='3d')

    ax.set_xlim(lo[0], hi[0])
    ax.set_ylim(lo[1], hi[1])
    ax.set_zlim(max(0, lo[2]), max(hi[2], Z_MAX))
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')

    ax.scatter(*p_traj[0], color='red',  s=50, marker='o', alpha=0.4, label='P start')
    ax.scatter(*e_traj[0], color='blue', s=50, marker='o', alpha=0.4, label='E start')
    ax.legend(loc='upper left', fontsize=8)

    p_trail, = ax.plot([], [], [], color='red',  linewidth=1.2, alpha=0.6)
    e_trail, = ax.plot([], [], [], color='blue', linewidth=1.2, alpha=0.6)
    p_dot = ax.scatter([], [], [], color='red',  s=80, marker='^', zorder=6)
    e_dot = ax.scatter([], [], [], color='blue', s=80, marker='s', zorder=6)

    def update(i):
        px = p_frames[:i+1, 0]; py = p_frames[:i+1, 1]; pz = p_frames[:i+1, 2]
        ex = e_frames[:i+1, 0]; ey = e_frames[:i+1, 1]; ez = e_frames[:i+1, 2]
        p_trail.set_data(px, py); p_trail.set_3d_properties(pz)
        e_trail.set_data(ex, ey); e_trail.set_3d_properties(ez)
        p_dot._offsets3d = ([float(px[-1])], [float(py[-1])], [float(pz[-1])])
        e_dot._offsets3d = ([float(ex[-1])], [float(ey[-1])], [float(ez[-1])])
        t = i * step_skip * DT
        ax.set_title(f'S002 3D — Helix Evasion  t={t:.2f}s', fontsize=10)
        return p_trail, e_trail, p_dot, e_dot

    ani = FuncAnimation(fig, update, frames=n_frames, interval=50, blit=False)
    path = os.path.join(OUTPUT_DIR, 'animation.gif')
    ani.save(path, writer=PillowWriter(fps=20))
    plt.close()
    print(f'Saved: {path}')


# ── Main ─────────────────────────────────────────────────────

def main():
    strategies = [
        (STRATEGY_NAMES[0], STRATEGY_LABELS[0], strategy_horizontal_perp),
        (STRATEGY_NAMES[1], STRATEGY_LABELS[1], strategy_perp_3d),
        (STRATEGY_NAMES[2], STRATEGY_LABELS[2], strategy_helix),
        (STRATEGY_NAMES[3], STRATEGY_LABELS[3], strategy_dive_and_run),
    ]

    results = []
    for name, label, fn in strategies:
        p_traj, e_traj, captured, cap_time, los_angles = run_simulation(fn)
        status = f'captured at {cap_time:.2f}s' if captured else 'not captured (timeout)'
        print(f'[{name:>18}]  {status}')
        results.append((name, label, p_traj, e_traj, captured, cap_time, los_angles))

    print('\nGenerating plots...')
    plot_trajectories_3d(results)
    plot_altitude_time(results)
    plot_capture_times(results)
    plot_los_angle(results)
    print('Generating animation (helix strategy)...')
    save_animation(results)
    print('\nDone. All outputs saved to:', OUTPUT_DIR)


if __name__ == '__main__':
    main()
