"""
S004 3D Upgrade — Obstacle-Course Chase
=========================================
Pursuer (3D APF) chases an evader through 4 spheres + 1 vertical cylinder.
Compares:
  1. fixed_z  — APF with z=2.0m fixed (2D baseline)
  2. full_3d  — full 3D APF with over-fly heuristic

Usage:
    conda activate drones
    python src/01_pursuit_evasion/3d/s004_3d_obstacle_chase.py
"""
import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.animation import FuncAnimation, PillowWriter

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..'))
from src.base.drone_base import DroneBase

# ── Scenario parameters ─────────────────────────────────────
PURSUER_SPEED = 5.0
EVADER_SPEED  = 3.5
DT            = 1 / 48
MAX_TIME      = 30.0
CAPTURE_R     = 0.15
COLLISION_R   = 0.05

INIT_PURSUER = np.array([-4.0, 0.0, 2.0])
INIT_EVADER  = np.array([ 4.0, 0.0, 2.0])

Z_MIN = 0.3
Z_MAX = 10.0

# APF parameters
K_REP = 4.0
RHO0  = 1.5

# ── Obstacle layout ─────────────────────────────────────────
SPHERES = [
    {'c': np.array([-2.0,  0.0, 2.0]), 'r': 0.60},
    {'c': np.array([-0.5,  0.5, 3.5]), 'r': 0.55},   # elevated
    {'c': np.array([ 1.0, -0.4, 1.5]), 'r': 0.50},   # low
    {'c': np.array([ 2.5,  0.3, 2.5]), 'r': 0.45},
]
CYLINDER = {'xy': np.array([0.5, -1.0]), 'r': 0.4, 'z_bot': 0.0, 'z_top': 5.0}

OUTPUT_DIR = os.path.normpath(os.path.join(
    os.path.dirname(__file__), '..', '..', '..',
    'outputs', '01_pursuit_evasion', '3d', 's004_3d_obstacle_chase',
))
os.makedirs(OUTPUT_DIR, exist_ok=True)


# ── APF helpers ──────────────────────────────────────────────

def sphere_repulsion(pos, spheres):
    """3D repulsion from all spheres."""
    v_rep = np.zeros(3)
    for sp in spheres:
        diff = pos - sp['c']
        dist = np.linalg.norm(diff) + 1e-8
        rho  = dist - sp['r']
        if 0 < rho < RHO0:
            n_hat  = diff / dist
            v_rep += K_REP * (1.0 / rho - 1.0 / RHO0) * (1.0 / rho**2) * n_hat
    return v_rep


def cylinder_repulsion(pos, cyl):
    """Horizontal repulsion from cylinder, active only when z_bot <= z <= z_top."""
    if not (cyl['z_bot'] <= pos[2] <= cyl['z_top']):
        return np.zeros(3)
    dx = pos[0] - cyl['xy'][0]
    dy = pos[1] - cyl['xy'][1]
    dist_xy = np.sqrt(dx**2 + dy**2) + 1e-8
    rho = dist_xy - cyl['r']
    if 0 < rho < RHO0:
        n_hat_xy = np.array([dx, dy, 0.0]) / dist_xy
        return K_REP * (1.0 / rho - 1.0 / RHO0) * (1.0 / rho**2) * n_hat_xy
    return np.zeros(3)


def total_repulsion(pos, spheres, cyl):
    return sphere_repulsion(pos, spheres) + cylinder_repulsion(pos, cyl)


def inside_any_obstacle(pos, spheres, cyl):
    """True if pos is inside any obstacle (plus margin)."""
    for sp in spheres:
        if np.linalg.norm(pos - sp['c']) < sp['r'] + COLLISION_R:
            return True
    dx = pos[0] - cyl['xy'][0]
    dy = pos[1] - cyl['xy'][1]
    dist_xy = np.sqrt(dx**2 + dy**2)
    if (dist_xy < cyl['r'] + COLLISION_R
            and cyl['z_bot'] <= pos[2] <= cyl['z_top']):
        return True
    return False


def over_fly_heuristic(pos_p, pos_e, spheres):
    """
    For each sphere in the influence zone, compare horizontal-detour distance
    vs vertical-detour distance. Return a suggested z offset if over-fly is better.
    """
    for sp in spheres:
        diff = pos_p - sp['c']
        dist = np.linalg.norm(diff)
        rho  = dist - sp['r']
        if rho < RHO0 * 1.5:   # within expanded influence
            # Vertical detour: climb to top of sphere + 0.5 m safety
            target_z = sp['c'][2] + sp['r'] + 0.5
            # Only suggest over-fly if we're roughly in the sphere's x-y vicinity
            dx = pos_p[0] - sp['c'][0]
            dy = pos_p[1] - sp['c'][1]
            h_dist = np.sqrt(dx**2 + dy**2)
            if h_dist < sp['r'] + RHO0:
                # vertical detour cost
                v_cost = abs(target_z - pos_p[2]) + abs(target_z - pos_e[2])
                # horizontal detour cost: approx circumference quarter
                h_cost = np.pi / 2 * (sp['r'] + 0.5)
                if v_cost < h_cost:
                    return target_z
    return None


# ── Simulation ───────────────────────────────────────────────

def clamp_altitude(pos):
    pos[2] = np.clip(pos[2], Z_MIN, Z_MAX)
    return pos


def run_simulation(strategy='full_3d'):
    """
    strategy: 'fixed_z' or 'full_3d'
    Both pursuer and evader use APF obstacle avoidance.
    """
    pursuer = DroneBase(INIT_PURSUER, max_speed=PURSUER_SPEED, dt=DT)
    evader  = DroneBase(INIT_EVADER,  max_speed=EVADER_SPEED,  dt=DT)

    max_steps  = int(MAX_TIME / DT)
    captured   = False
    cap_time   = None
    p_crashed  = False
    p_crash_t  = None
    e_crashed  = False
    e_crash_t  = None

    fixed_z_val = 2.0  # used when strategy == 'fixed_z'

    for step in range(max_steps):
        t = step * DT

        # Crash check
        if inside_any_obstacle(pursuer.pos, SPHERES, CYLINDER):
            p_crashed = True; p_crash_t = t; break
        if inside_any_obstacle(evader.pos, SPHERES, CYLINDER):
            e_crashed = True; e_crash_t = t; break

        # Capture check
        if np.linalg.norm(pursuer.pos - evader.pos) < CAPTURE_R:
            captured = True; cap_time = t; break

        # ── Pursuer velocity ──
        r_pe  = evader.pos - pursuer.pos
        v_att = PURSUER_SPEED * r_pe / (np.linalg.norm(r_pe) + 1e-8)
        v_rep = total_repulsion(pursuer.pos, SPHERES, CYLINDER)

        if strategy == 'fixed_z':
            # Keep altitude fixed: zero out z components of velocities
            v_att[2] = 0.0
            v_rep[2] = 0.0
        else:
            # Over-fly heuristic: add upward velocity component if beneficial
            fly_z = over_fly_heuristic(pursuer.pos, evader.pos, SPHERES)
            if fly_z is not None:
                dz = fly_z - pursuer.pos[2]
                v_att[2] += np.sign(dz) * min(abs(dz) * 2.0, PURSUER_SPEED * 0.5)

        v_p = v_att + v_rep
        pursuer.step(v_p)

        if strategy == 'fixed_z':
            pursuer.pos[2] = fixed_z_val
        else:
            clamp_altitude(pursuer.pos)
        pursuer.trajectory[-1] = pursuer.pos.copy()

        # ── Evader velocity (straight escape + APF) ──
        r_ep    = evader.pos - pursuer.pos
        v_esc   = EVADER_SPEED * r_ep / (np.linalg.norm(r_ep) + 1e-8)
        v_rep_e = total_repulsion(evader.pos, SPHERES, CYLINDER)

        if strategy == 'fixed_z':
            v_esc[2]   = 0.0
            v_rep_e[2] = 0.0

        evader.step(v_esc + v_rep_e)

        if strategy == 'fixed_z':
            evader.pos[2] = fixed_z_val
        else:
            clamp_altitude(evader.pos)
        evader.trajectory[-1] = evader.pos.copy()

    return (
        pursuer.get_trajectory(),
        evader.get_trajectory(),
        captured, cap_time,
        p_crashed, p_crash_t,
        e_crashed, e_crash_t,
    )


# ── Drawing helpers ──────────────────────────────────────────

def draw_spheres(ax, spheres, color='green', alpha=0.20):
    u = np.linspace(0, 2 * np.pi, 20)
    v = np.linspace(0, np.pi, 12)
    for sp in spheres:
        cx, cy, cz, r = sp['c'][0], sp['c'][1], sp['c'][2], sp['r']
        xs = cx + r * np.outer(np.cos(u), np.sin(v))
        ys = cy + r * np.outer(np.sin(u), np.sin(v))
        zs = cz + r * np.outer(np.ones_like(u), np.cos(v))
        ax.plot_surface(xs, ys, zs, color=color, alpha=alpha, linewidth=0)


def draw_cylinder(ax, cyl, color='green', alpha=0.20, n_theta=30):
    theta = np.linspace(0, 2 * np.pi, n_theta)
    z_vals = np.array([cyl['z_bot'], cyl['z_top']])
    theta_grid, z_grid = np.meshgrid(theta, z_vals)
    x_grid = cyl['xy'][0] + cyl['r'] * np.cos(theta_grid)
    y_grid = cyl['xy'][1] + cyl['r'] * np.sin(theta_grid)
    ax.plot_surface(x_grid, y_grid, z_grid, color=color, alpha=alpha, linewidth=0)


# ── Plots ─────────────────────────────────────────────────────

def plot_trajectories_3d(results):
    labels = ['Fixed-Z APF\n(2D baseline)', 'Full 3D APF\n(+ over-fly)']
    all_pts = np.vstack([np.vstack([r[0], r[1]]) for r in results])
    margin = 1.0
    lo = all_pts.min(axis=0) - margin
    hi = all_pts.max(axis=0) + margin

    fig = plt.figure(figsize=(14, 6))
    for i, (label, res) in enumerate(zip(labels, results)):
        p_traj, e_traj, captured, cap_t, p_crashed, p_crash_t, e_crashed, e_crash_t = res
        ax = fig.add_subplot(1, 2, i + 1, projection='3d')

        draw_spheres(ax, SPHERES)
        draw_cylinder(ax, CYLINDER)

        ax.plot(p_traj[:, 0], p_traj[:, 1], p_traj[:, 2],
                color='red', linewidth=1.8, label='Pursuer', zorder=5)
        ax.plot(e_traj[:, 0], e_traj[:, 1], e_traj[:, 2],
                color='blue', linewidth=1.8, label='Evader', zorder=5)
        ax.scatter(*p_traj[0], color='red',  s=50, marker='o', zorder=6)
        ax.scatter(*e_traj[0], color='blue', s=50, marker='o', zorder=6)

        if captured:
            ax.scatter(*p_traj[-1], color='black', s=120, marker='X', zorder=7,
                       label=f'Cap {cap_t:.2f}s')
        if p_crashed:
            ax.scatter(*p_traj[-1], color='orange', s=120, marker='X', zorder=7,
                       label=f'P-Crash {p_crash_t:.2f}s')

        ax.set_xlabel('X (m)', fontsize=8)
        ax.set_ylabel('Y (m)', fontsize=8)
        ax.set_zlabel('Z (m)', fontsize=8)
        ax.set_xlim(lo[0], hi[0])
        ax.set_ylim(lo[1], hi[1])
        ax.set_zlim(0, hi[2])

        status = (f'Captured {cap_t:.2f}s' if captured
                  else f'P-Crash {p_crash_t:.2f}s' if p_crashed
                  else 'Timeout')
        ax.set_title(f'{label}\n{status}', fontsize=9)
        ax.legend(loc='upper left', fontsize=7)

    fig.suptitle('S004 3D — Obstacle-Course Chase: Trajectories', fontsize=13, y=1.01)
    plt.tight_layout()
    path = os.path.join(OUTPUT_DIR, 'trajectories_3d.png')
    plt.savefig(path, dpi=120, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_altitude_profile(results):
    labels = ['Fixed-Z APF (2D baseline)', 'Full 3D APF (+ over-fly)']
    colors_p = ['red', 'darkred']
    colors_e = ['blue', 'navy']

    fig, axes = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    for i, (label, res) in enumerate(zip(labels, results)):
        p_traj, e_traj, captured, cap_t, *_ = res
        ax = axes[i]
        n = min(len(p_traj), len(e_traj))
        t_arr = np.arange(n) * DT
        ax.plot(t_arr, p_traj[:n, 2], color='red',  linewidth=1.5, label='Pursuer z')
        ax.plot(t_arr, e_traj[:n, 2], color='blue', linewidth=1.5, label='Evader z')
        ax.axhline(Z_MIN, color='gray', linestyle=':', linewidth=0.8, alpha=0.6)
        ax.axhline(Z_MAX, color='gray', linestyle=':', linewidth=0.8, alpha=0.6)
        # Draw sphere altitude ranges
        for sp in SPHERES:
            ax.axhspan(sp['c'][2] - sp['r'], sp['c'][2] + sp['r'],
                       color='green', alpha=0.07)
        # Draw cylinder altitude range
        ax.axhspan(CYLINDER['z_bot'], CYLINDER['z_top'], color='olive', alpha=0.05)
        if captured:
            ax.axvline(cap_t, color='black', linestyle='--', linewidth=1.0, alpha=0.7,
                       label=f'Captured {cap_t:.2f}s')
        ax.set_ylabel('Altitude z (m)')
        ax.set_title(label, fontsize=9)
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.set_ylim(-0.2, Z_MAX + 0.5)
    axes[-1].set_xlabel('Time (s)')
    fig.suptitle('S004 3D — Altitude Profile (green bands = obstacle altitude ranges)',
                 fontsize=12)
    plt.tight_layout()
    path = os.path.join(OUTPUT_DIR, 'altitude_profile.png')
    plt.savefig(path, dpi=120, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def _path_length(traj):
    diffs = np.diff(traj, axis=0)
    return np.sum(np.linalg.norm(diffs, axis=1))


def plot_path_length_comparison(results):
    labels = ['Fixed-Z APF\n(2D baseline)', 'Full 3D APF\n(+ over-fly)']
    p_lengths = [_path_length(r[0]) for r in results]
    e_lengths = [_path_length(r[1]) for r in results]
    cap_times  = [r[3] if r[2] else None for r in results]

    x = np.arange(len(labels))
    width = 0.35

    fig, ax = plt.subplots(figsize=(9, 5))
    bars_p = ax.bar(x - width/2, p_lengths, width, label='Pursuer', color='red',   alpha=0.75, edgecolor='black')
    bars_e = ax.bar(x + width/2, e_lengths, width, label='Evader',  color='blue',  alpha=0.75, edgecolor='black')

    for bar, val in zip(bars_p, p_lengths):
        ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.3,
                f'{val:.1f}m', ha='center', va='bottom', fontsize=9)
    for bar, val in zip(bars_e, e_lengths):
        ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.3,
                f'{val:.1f}m', ha='center', va='bottom', fontsize=9)

    # Annotate capture time
    for xi, ct in zip(x, cap_times):
        if ct is not None:
            ax.text(xi, max(p_lengths + e_lengths) * 1.05,
                    f'Cap: {ct:.1f}s', ha='center', va='bottom', fontsize=8,
                    color='black', style='italic')

    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.set_ylabel('Path Length (m)')
    ax.set_title('S004 3D — Path Length Comparison\n(Fixed-Z vs Full-3D APF)')
    ax.legend()
    ax.grid(True, axis='y', alpha=0.3)
    plt.tight_layout()
    path = os.path.join(OUTPUT_DIR, 'path_length_comparison.png')
    plt.savefig(path, dpi=120, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def save_animation(results):
    """Animate the full_3d strategy (index 1)."""
    p_traj, e_traj, captured, cap_t, p_crashed, p_crash_t, *_ = results[1]

    step_skip = 4
    p_frames = p_traj[::step_skip]
    e_frames = e_traj[::step_skip]
    n_frames = min(len(p_frames), len(e_frames))

    all_pts = np.vstack([p_traj, e_traj])
    margin = 1.0
    lo = all_pts.min(axis=0) - margin
    hi = all_pts.max(axis=0) + margin

    fig = plt.figure(figsize=(9, 8))
    ax = fig.add_subplot(111, projection='3d')

    draw_spheres(ax, SPHERES)
    draw_cylinder(ax, CYLINDER)

    ax.set_xlim(lo[0], hi[0])
    ax.set_ylim(lo[1], hi[1])
    ax.set_zlim(0, max(hi[2], Z_MAX))
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.scatter(*p_traj[0], color='red',  s=50, marker='o', alpha=0.4, label='P start')
    ax.scatter(*e_traj[0], color='blue', s=50, marker='o', alpha=0.4, label='E start')
    ax.legend(loc='upper left', fontsize=8)

    p_trail, = ax.plot([], [], [], color='red',  linewidth=1.3, alpha=0.6)
    e_trail, = ax.plot([], [], [], color='blue', linewidth=1.3, alpha=0.6)
    p_dot = ax.scatter([], [], [], color='red',  s=90, marker='^', zorder=6)
    e_dot = ax.scatter([], [], [], color='blue', s=90, marker='s', zorder=6)

    def update(i):
        px = p_frames[:i+1, 0]; py = p_frames[:i+1, 1]; pz = p_frames[:i+1, 2]
        ex = e_frames[:i+1, 0]; ey = e_frames[:i+1, 1]; ez = e_frames[:i+1, 2]
        p_trail.set_data(px, py); p_trail.set_3d_properties(pz)
        e_trail.set_data(ex, ey); e_trail.set_3d_properties(ez)
        p_dot._offsets3d = ([float(px[-1])], [float(py[-1])], [float(pz[-1])])
        e_dot._offsets3d = ([float(ex[-1])], [float(ey[-1])], [float(ez[-1])])
        t = i * step_skip * DT
        ax.set_title(f'S004 3D — Obstacle Chase (Full 3D APF)  t={t:.2f}s', fontsize=10)
        return p_trail, e_trail, p_dot, e_dot

    ani = FuncAnimation(fig, update, frames=n_frames, interval=50, blit=False)
    path = os.path.join(OUTPUT_DIR, 'animation.gif')
    ani.save(path, writer=PillowWriter(fps=20))
    plt.close()
    print(f'Saved: {path}')


# ── Main ─────────────────────────────────────────────────────

def main():
    cases = [
        ('fixed_z',  'Fixed-Z APF (2D baseline)'),
        ('full_3d',  'Full 3D APF (+ over-fly)'),
    ]

    results = []
    for strategy, label in cases:
        res = run_simulation(strategy)
        captured, cap_t, p_crashed, p_crash_t = res[2], res[3], res[4], res[5]
        if captured:
            status = f'captured at {cap_t:.2f}s'
        elif p_crashed:
            status = f'PURSUER CRASHED at {p_crash_t:.2f}s'
        else:
            status = 'timeout'
        print(f'[{label:>32}]  {status}')
        results.append(res)

    print('\nGenerating plots...')
    plot_trajectories_3d(results)
    plot_altitude_profile(results)
    plot_path_length_comparison(results)
    print('Generating animation (full_3d strategy)...')
    save_animation(results)
    print('\nDone. All outputs saved to:', OUTPUT_DIR)


if __name__ == '__main__':
    main()
