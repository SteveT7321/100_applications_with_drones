"""
S002 Evasive Maneuver
====================
Pursuer (Pure Pursuit) vs Evader using 4 different strategies.
Compares capture time across: straight, perpendicular, random, spiral escape.

Usage:
    conda activate drones
    python src/pursuit/s002_evasive_maneuver.py
"""

import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from src.base.drone_base import DroneBase

# ── Scenario parameters ───────────────────────────────────
PURSUER_SPEED = 5.0     # m/s
EVADER_SPEED  = 3.5     # m/s (slower — capture guaranteed)
DT            = 1 / 48  # 48 Hz control
MAX_TIME      = 30.0    # seconds
CAPTURE_R     = 0.15    # capture radius (m)

INIT_PURSUER = np.array([-2.0, 0.0, 2.0])
INIT_EVADER  = np.array([ 2.0, 0.0, 2.0])

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), '..', '..', 'outputs', '01_pursuit_evasion', 's002_evasive_maneuver')


# ── Evader strategies ─────────────────────────────────────

def straight_escape(pos_e, pos_p, v_max, t):
    """Fly directly away from pursuer."""
    r = pos_e - pos_p
    return v_max * r / (np.linalg.norm(r) + 1e-8)


def perpendicular_escape(pos_e, pos_p, v_max, t):
    """Optimal: fly perpendicular to line-of-sight in the xy plane."""
    r_xy = pos_e[:2] - pos_p[:2]
    r_hat = r_xy / (np.linalg.norm(r_xy) + 1e-8)
    perp = np.array([-r_hat[1], r_hat[0], 0.0])
    return v_max * perp


def make_random_escape(seed=42):
    """Returns a stateless random strategy with fixed seed for reproducibility."""
    rng = np.random.default_rng(seed)

    def _strategy(pos_e, pos_p, v_max, t):
        d = rng.standard_normal(3)
        d[2] = 0.0
        return v_max * d / (np.linalg.norm(d) + 1e-8)

    return _strategy


def spiral_escape(pos_e, pos_p, v_max, t):
    """Mix perpendicular (70%) + outward (30%) to spiral away."""
    r_xy = pos_e[:2] - pos_p[:2]
    r_hat = r_xy / (np.linalg.norm(r_xy) + 1e-8)
    perp    = np.array([-r_hat[1],  r_hat[0], 0.0])
    outward = np.array([ r_hat[0],  r_hat[1], 0.0])
    v = 0.7 * perp + 0.3 * outward
    return v_max * v / (np.linalg.norm(v) + 1e-8)


# ── Simulation ─────────────────────────────────────────────

def run_simulation(evader_fn):
    pursuer = DroneBase(INIT_PURSUER, max_speed=PURSUER_SPEED, dt=DT)
    evader  = DroneBase(INIT_EVADER,  max_speed=EVADER_SPEED,  dt=DT)

    max_steps = int(MAX_TIME / DT)
    captured = False
    capture_time = None

    for step in range(max_steps):
        dist = np.linalg.norm(pursuer.pos - evader.pos)
        if dist < CAPTURE_R:
            captured = True
            capture_time = step * DT
            break

        # Pursuer: pure pursuit — always aim at current evader position
        r_pe = evader.pos - pursuer.pos
        v_pursuer = PURSUER_SPEED * r_pe / (np.linalg.norm(r_pe) + 1e-8)
        pursuer.step(v_pursuer)

        # Evader: apply strategy
        v_evader = evader_fn(evader.pos, pursuer.pos, EVADER_SPEED, step * DT)
        evader.step(v_evader)

    return pursuer.get_trajectory(), evader.get_trajectory(), captured, capture_time


# ── Plotting ───────────────────────────────────────────────

STRATEGY_COLORS = {
    'Straight':      'orange',
    'Perpendicular': 'blue',
    'Random':        'green',
    'Spiral':        'purple',
}


def plot_trajectories(results, out_dir):
    """3D trajectory subplots — one per strategy."""
    n = len(results)
    fig = plt.figure(figsize=(5 * n, 5))

    for i, (name, p_traj, e_traj, captured, cap_time) in enumerate(results):
        ax = fig.add_subplot(1, n, i + 1, projection='3d')

        ax.plot(p_traj[:, 0], p_traj[:, 1], p_traj[:, 2],
                color='red',  linewidth=1.5, label='Pursuer')
        ax.plot(e_traj[:, 0], e_traj[:, 1], e_traj[:, 2],
                color='blue', linewidth=1.5, label='Evader')

        ax.scatter(*p_traj[0], color='red',  s=50, marker='o')
        ax.scatter(*e_traj[0], color='blue', s=50, marker='o')

        if captured:
            ax.scatter(*p_traj[-1], color='black', s=100, marker='X',
                       label=f'Captured {cap_time:.1f}s')

        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        cap_str = f'Capture: {cap_time:.2f}s' if captured else 'Not captured'
        ax.set_title(f'{name} escape\n{cap_str}', fontsize=9)
        ax.legend(loc='upper left', fontsize=7)

    fig.suptitle('S002 Evasive Maneuver — 3D Trajectories', fontsize=12, y=1.01)
    plt.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'trajectories_3d.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_capture_times(results, out_dir):
    """Bar chart comparing capture time for each strategy."""
    names  = [name.replace(' ', '\n') for name, *_ in results]
    times  = []
    colors = []

    for _, _, _, captured, cap_time in results:
        times.append(cap_time if captured else MAX_TIME)
        colors.append('steelblue' if captured else 'lightgray')

    fig, ax = plt.subplots(figsize=(8, 5))
    bars = ax.bar(names, times, color=colors, edgecolor='black', linewidth=0.8)

    for bar, (_, _, _, captured, cap_time) in zip(bars, results):
        label = f'{cap_time:.2f}s' if captured else 'Escaped'
        ax.text(bar.get_x() + bar.get_width() / 2,
                bar.get_height() + 0.3,
                label, ha='center', va='bottom', fontsize=9)

    ax.axhline(MAX_TIME, color='red', linestyle='--', linewidth=1,
               alpha=0.6, label=f'Max time ({MAX_TIME:.0f}s)')
    ax.set_ylabel('Capture Time (s)')
    ax.set_title('S002 Evasive Maneuver — Capture Time by Strategy\n'
                 f'(Pursuer {PURSUER_SPEED} m/s  vs  Evader {EVADER_SPEED} m/s)')
    ax.set_ylim(0, MAX_TIME * 1.2)
    ax.legend()
    ax.grid(True, axis='y', alpha=0.3)

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'capture_times.png')
    plt.tight_layout()
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def plot_distance_time(results, out_dir):
    """Pursuer–evader distance over time for all strategies."""
    fig, ax = plt.subplots(figsize=(8, 5))

    for name, p_traj, e_traj, captured, cap_time in results:
        n = min(len(p_traj), len(e_traj))
        dists = np.linalg.norm(p_traj[:n] - e_traj[:n], axis=1)
        times = np.arange(n) * DT
        color = STRATEGY_COLORS.get(name, 'gray')
        label = f'{name}' + (f' (cap {cap_time:.2f}s)' if captured else ' (escaped)')
        ax.plot(times, dists, color=color, linewidth=1.5, label=label)

    ax.axhline(CAPTURE_R, color='red', linestyle='--', linewidth=1,
               label=f'Capture radius {CAPTURE_R} m')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Pursuer–Evader Distance (m)')
    ax.set_title('S002 Evasive Maneuver — Distance vs Time')
    ax.legend()
    ax.grid(True, alpha=0.3)

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'distance_time.png')
    plt.tight_layout()
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def save_animation(results, out_dir):
    """2×2 subplot animation — all 4 strategies simultaneously, ~12 fps."""
    step = 4
    n_frames = max(
        min(len(p[::step]), len(e[::step]))
        for _, p, e, _, _ in results
    )

    # Shared axis limits across all trajectories
    all_pts = np.vstack([
        np.vstack([p, e]) for _, p, e, _, _ in results
    ])
    margin = 0.5
    lo = all_pts.min(axis=0) - margin
    hi = all_pts.max(axis=0) + margin

    fig = plt.figure(figsize=(20, 5))
    axes, trails_p, trails_e, dots_p, dots_e = [], [], [], [], []
    frame_data = []

    for idx, (name, p_traj, e_traj, captured, cap_time) in enumerate(results):
        ax = fig.add_subplot(1, 4, idx + 1, projection='3d')
        ax.set_xlim(lo[0], hi[0])
        ax.set_ylim(lo[1], hi[1])
        ax.set_zlim(lo[2], hi[2])
        ax.set_xlabel('X (m)', fontsize=7)
        ax.set_ylabel('Y (m)', fontsize=7)
        ax.set_zlabel('Z (m)', fontsize=7)
        cap_str = f'cap {cap_time:.2f}s' if captured else 'escaped'
        ax.set_title(f'{name} ({cap_str})', fontsize=9)
        ax.scatter(*p_traj[0], color='red',  s=30, marker='o', alpha=0.4)
        ax.scatter(*e_traj[0], color='blue', s=30, marker='o', alpha=0.4)

        tp, = ax.plot([], [], [], color='red',  linewidth=1.0, alpha=0.6)
        te, = ax.plot([], [], [], color='blue', linewidth=1.0, alpha=0.6)
        dp = ax.scatter([], [], [], color='red',  s=60, marker='^', zorder=6)
        de = ax.scatter([], [], [], color='blue', s=60, marker='s', zorder=6)

        axes.append(ax)
        trails_p.append(tp); trails_e.append(te)
        dots_p.append(dp);   dots_e.append(de)
        frame_data.append((p_traj[::step], e_traj[::step]))

    fig.suptitle('S002 Evasive Maneuver — All Strategies', fontsize=12)

    def update(i):
        artists = []
        for k, (pf, ef) in enumerate(frame_data):
            ni = min(i + 1, len(pf), len(ef))
            px, py, pz = pf[:ni, 0], pf[:ni, 1], pf[:ni, 2]
            ex, ey, ez = ef[:ni, 0], ef[:ni, 1], ef[:ni, 2]
            trails_p[k].set_data(px, py); trails_p[k].set_3d_properties(pz)
            trails_e[k].set_data(ex, ey); trails_e[k].set_3d_properties(ez)
            dots_p[k]._offsets3d = ([float(px[-1])], [float(py[-1])], [float(pz[-1])])
            dots_e[k]._offsets3d = ([float(ex[-1])], [float(ey[-1])], [float(ez[-1])])
            artists += [trails_p[k], trails_e[k], dots_p[k], dots_e[k]]
        t = i * step * DT
        fig.suptitle(f'S002 Evasive Maneuver — All Strategies  t={t:.2f}s', fontsize=12)
        return artists

    ani = animation.FuncAnimation(fig, update, frames=n_frames, interval=83, blit=False)

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'animation_v2.gif')
    ani.save(path, writer='pillow', fps=12, dpi=80)
    plt.close()
    print(f'Saved: {path}')


# ── Main ───────────────────────────────────────────────────

if __name__ == '__main__':
    import matplotlib.animation as animation

    strategies = [
        ('Straight',      straight_escape),
        ('Perpendicular', perpendicular_escape),
        ('Random',        make_random_escape(seed=42)),
        ('Spiral',        spiral_escape),
    ]

    results = []
    for name, fn in strategies:
        p_traj, e_traj, captured, cap_time = run_simulation(fn)
        status = f'captured at {cap_time:.2f}s' if captured else 'escaped (not captured)'
        print(f'[{name:>14}] {status}')
        results.append((name, p_traj, e_traj, captured, cap_time))

    out_dir = os.path.normpath(OUTPUT_DIR)
    plot_trajectories(results, out_dir)
    plot_capture_times(results, out_dir)
    plot_distance_time(results, out_dir)
    save_animation(results, out_dir)
