"""
S004 Obstacle-Course Chase
===========================
Pursuer (Attractive-Repulsive Potential Field) chases an evader through a 3-D
obstacle course of spherical obstacles.

Comparison:
  - With obstacle avoidance: pursuer steers around obstacles using APF
  - Without obstacle avoidance: pure pursuit — crashes on first obstacle

Both agents have the same APF obstacle repulsion; only the pursuer comparison
case is stripped of it.

Usage:
    conda activate drones
    python src/pursuit/s004_obstacle_chase.py
"""

import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from src.base.drone_base import DroneBase

# ── Scenario parameters ────────────────────────────────────
PURSUER_SPEED = 5.0
EVADER_SPEED  = 3.5
DT            = 1 / 48
MAX_TIME      = 15.0
CAPTURE_R     = 0.15
COLLISION_R   = 0.05   # tolerance: drone centre inside obstacle + this → crash

INIT_PURSUER = np.array([-4.0,  0.0, 2.0])
INIT_EVADER  = np.array([ 3.5,  0.0, 2.0])

# ── Obstacle layout ────────────────────────────────────────
# Each row: (cx, cy, cz, radius)
# Obstacles are placed along / near the pursuer–evader line of approach
OBSTACLES = np.array([
    [-2.0,  0.0, 2.0, 0.60],   # directly on straight-line path
    [-0.5,  0.5, 2.0, 0.55],   # centre, slightly above
    [ 1.0, -0.4, 2.0, 0.50],   # right of centre
    [ 2.5,  0.3, 2.0, 0.45],   # near evader start
], dtype=float)

# APF parameters
K_REP = 4.0    # repulsion gain
RHO0  = 1.2    # influence range from obstacle surface (m)

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '01_pursuit_evasion', 's004_obstacle_chase',
)


# ── Potential field helpers ────────────────────────────────

def repulsion_velocity(pos, obstacles):
    """
    Sum of repulsive velocity contributions from all obstacles.

    v_rep = Σ K_rep * (1/ρ - 1/ρ0) * (1/ρ²) * n̂    for ρ < ρ0
    where ρ = ||pos - centre|| - radius  (distance to surface)
    """
    v_rep = np.zeros(3)
    for cx, cy, cz, r in obstacles:
        centre = np.array([cx, cy, cz])
        diff   = pos - centre
        dist   = np.linalg.norm(diff) + 1e-8
        rho    = dist - r
        if 0 < rho < RHO0:
            n_hat  = diff / dist
            v_rep += K_REP * (1.0 / rho - 1.0 / RHO0) * (1.0 / rho**2) * n_hat
    return v_rep


def inside_obstacle(pos, obstacles):
    """True if pos is within any obstacle (plus COLLISION_R margin)."""
    for cx, cy, cz, r in obstacles:
        if np.linalg.norm(pos - np.array([cx, cy, cz])) < r + COLLISION_R:
            return True
    return False


# ── Simulation ─────────────────────────────────────────────

def run_simulation(pursuer_uses_apf):
    """
    Both evader always uses APF obstacle avoidance + straight escape.
    Pursuer uses APF only when pursuer_uses_apf=True; else pure pursuit.
    """
    pursuer = DroneBase(INIT_PURSUER, max_speed=PURSUER_SPEED, dt=DT)
    evader  = DroneBase(INIT_EVADER,  max_speed=EVADER_SPEED,  dt=DT)

    max_steps    = int(MAX_TIME / DT)
    captured     = False;  capture_time = None
    p_crashed    = False;  p_crash_time = None
    e_crashed    = False;  e_crash_time = None
    rep_mag_log  = []      # |v_rep| at pursuer position each step

    for step in range(max_steps):
        t = step * DT

        # ── Pursuer crash / capture ──
        if inside_obstacle(pursuer.pos, OBSTACLES):
            p_crashed    = True
            p_crash_time = t
            break
        if np.linalg.norm(pursuer.pos - evader.pos) < CAPTURE_R:
            captured     = True
            capture_time = t
            break

        # ── Evader crash ──
        if inside_obstacle(evader.pos, OBSTACLES):
            e_crashed    = True
            e_crash_time = t
            break

        # ── Pursuer velocity ──
        r_pe  = evader.pos - pursuer.pos
        v_att = PURSUER_SPEED * r_pe / (np.linalg.norm(r_pe) + 1e-8)
        if pursuer_uses_apf:
            v_rep = repulsion_velocity(pursuer.pos, OBSTACLES)
            rep_mag_log.append(np.linalg.norm(v_rep))
            v_p = v_att + v_rep
        else:
            rep_mag_log.append(0.0)
            v_p = v_att
        pursuer.step(v_p)

        # ── Evader velocity (straight escape + APF) ──
        r_ep  = evader.pos - pursuer.pos
        v_esc = EVADER_SPEED * r_ep / (np.linalg.norm(r_ep) + 1e-8)
        v_rep_e = repulsion_velocity(evader.pos, OBSTACLES)
        evader.step(v_esc + v_rep_e)

    return (
        pursuer.get_trajectory(), evader.get_trajectory(),
        captured, capture_time,
        p_crashed, p_crash_time,
        e_crashed, e_crash_time,
        np.array(rep_mag_log),
    )


# ── Plotting helpers ───────────────────────────────────────

def _draw_spheres(ax, obstacles, color='steelblue', alpha=0.25):
    u = np.linspace(0, 2 * np.pi, 20)
    v = np.linspace(0,     np.pi, 12)
    cos_u, sin_u = np.cos(u), np.sin(u)
    cos_v, sin_v = np.cos(v), np.sin(v)
    for cx, cy, cz, r in obstacles:
        xs = cx + r * np.outer(cos_u, sin_v)
        ys = cy + r * np.outer(sin_u, sin_v)
        zs = cz + r * np.outer(np.ones_like(u), cos_v)
        ax.plot_surface(xs, ys, zs, color=color, alpha=alpha, linewidth=0)


# ── Plots ──────────────────────────────────────────────────

def _axis_limits(results, margin=1.0):
    """Compute shared axis limits from all trajectories."""
    all_pts = np.vstack([
        np.vstack([res[0], res[1]]) for res in results
    ])
    lo = all_pts.min(axis=0) - margin
    hi = all_pts.max(axis=0) + margin
    return lo, hi


def plot_trajectories_3d(results, out_dir):
    labels = ['With APF obstacle avoidance', 'Without obstacle avoidance']
    lo, hi = _axis_limits(results)
    fig = plt.figure(figsize=(14, 6))

    for i, (label, res) in enumerate(zip(labels, results)):
        (p_traj, e_traj, captured, cap_t,
         p_crashed, p_crash_t, e_crashed, e_crash_t, _) = res

        ax = fig.add_subplot(1, 2, i + 1, projection='3d')
        _draw_spheres(ax, OBSTACLES)

        ax.plot(p_traj[:, 0], p_traj[:, 1], p_traj[:, 2],
                color='red',  linewidth=1.8, label='Pursuer', zorder=5)
        ax.plot(e_traj[:, 0], e_traj[:, 1], e_traj[:, 2],
                color='blue', linewidth=1.8, label='Evader',  zorder=5)
        ax.scatter(*p_traj[0], color='red',  s=50, marker='o', zorder=6)
        ax.scatter(*e_traj[0], color='blue', s=50, marker='o', zorder=6)

        if captured:
            ax.scatter(*p_traj[-1], color='black', s=100, marker='X', zorder=7,
                       label=f'Captured {cap_t:.2f} s')
        if p_crashed:
            ax.scatter(*p_traj[-1], color='orange', s=120, marker='X', zorder=7,
                       label=f'P-CRASH {p_crash_t:.2f} s')
        if e_crashed:
            ax.scatter(*e_traj[-1], color='purple', s=120, marker='X', zorder=7,
                       label=f'E-CRASH {e_crash_t:.2f} s')

        ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
        ax.set_xlim(lo[0], hi[0]); ax.set_ylim(lo[1], hi[1]); ax.set_zlim(max(0, lo[2]), hi[2])
        status = (f'Captured {cap_t:.2f} s' if captured
                  else f'P-CRASH {p_crash_t:.2f} s' if p_crashed
                  else f'E-CRASH {e_crash_t:.2f} s' if e_crashed
                  else 'Timeout')
        ax.set_title(f'{label}\n{status}', fontsize=9)
        ax.legend(loc='upper left', fontsize=7)

    fig.suptitle('S004 Obstacle-Course Chase — 3D Trajectories', fontsize=12, y=1.01)
    plt.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'trajectories_3d.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_distance_time(results, out_dir):
    labels = ['With APF avoidance', 'Without avoidance']
    colors = ['steelblue', 'darkorange']

    fig, ax = plt.subplots(figsize=(9, 5))
    for label, color, res in zip(labels, colors, results):
        p_traj, e_traj = res[0], res[1]
        captured, cap_t = res[2], res[3]
        p_crashed, p_crash_t = res[4], res[5]

        n     = min(len(p_traj), len(e_traj))
        dists = np.linalg.norm(p_traj[:n] - e_traj[:n], axis=1)
        times = np.arange(n) * DT
        ax.plot(times, dists, color=color, linewidth=1.8, label=label)

        if captured:
            ax.axvline(cap_t, color=color, linestyle='--', linewidth=1, alpha=0.7)
        if p_crashed:
            ax.axvline(p_crash_t, color=color, linestyle=':', linewidth=1.5, alpha=0.9)

    ax.axhline(CAPTURE_R, color='red', linestyle='--', linewidth=1,
               label=f'Capture radius {CAPTURE_R} m')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Pursuer–Evader Distance (m)')
    ax.set_title('S004 Obstacle-Course Chase — Distance vs Time')
    ax.legend()
    ax.grid(True, alpha=0.3)

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'distance_time.png')
    plt.tight_layout()
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def plot_repulsion(results, out_dir):
    """Repulsion velocity magnitude over time (APF case only)."""
    res = results[0]  # with-APF case
    rep_log = res[8]
    times   = np.arange(len(rep_log)) * DT

    fig, ax = plt.subplots(figsize=(9, 4))
    ax.plot(times, rep_log, color='steelblue', linewidth=1.5,
            label='$|\\mathbf{v}_{rep}|$ (pursuer)')
    ax.fill_between(times, 0, rep_log, color='steelblue', alpha=0.15)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Repulsion speed (m/s)')
    ax.set_title('S004 Obstacle-Course Chase — APF Repulsion Magnitude\n'
                 f'($K_{{rep}}={K_REP}$, influence range $\\rho_0={RHO0}$ m)')
    ax.legend()
    ax.grid(True, alpha=0.3)

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'repulsion_magnitude.png')
    plt.tight_layout()
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def save_animation(results, out_dir):
    """Animate the APF case: pursuer (red) chases evader (blue) through obstacles, ~12 fps."""
    import matplotlib.animation as animation

    (p_traj, e_traj, captured, cap_t,
     p_crashed, p_crash_t, e_crashed, e_crash_t, _) = results[0]  # APF case

    lo, hi = _axis_limits(results)

    step = 4
    p_frames = p_traj[::step]
    e_frames = e_traj[::step]
    n_frames = min(len(p_frames), len(e_frames))

    fig = plt.figure(figsize=(8, 7))
    ax = fig.add_subplot(111, projection='3d')
    _draw_spheres(ax, OBSTACLES)

    ax.set_xlim(lo[0], hi[0])
    ax.set_ylim(lo[1], hi[1])
    ax.set_zlim(max(0, lo[2]), hi[2])
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')

    ax.scatter(*p_traj[0], color='red',  s=50, marker='o', alpha=0.4, label='P start')
    ax.scatter(*e_traj[0], color='blue', s=50, marker='o', alpha=0.4, label='E start')
    ax.legend(loc='upper left')

    p_trail, = ax.plot([], [], [], color='red',  linewidth=1.2, alpha=0.6)
    e_trail, = ax.plot([], [], [], color='blue', linewidth=1.2, alpha=0.6)
    p_dot = ax.scatter([], [], [], color='red',  s=80, marker='^', zorder=6)
    e_dot = ax.scatter([], [], [], color='blue', s=80, marker='s', zorder=6)

    def update(i):
        px, py, pz = p_frames[:i+1, 0], p_frames[:i+1, 1], p_frames[:i+1, 2]
        ex, ey, ez = e_frames[:i+1, 0], e_frames[:i+1, 1], e_frames[:i+1, 2]
        p_trail.set_data(px, py); p_trail.set_3d_properties(pz)
        e_trail.set_data(ex, ey); e_trail.set_3d_properties(ez)
        p_dot._offsets3d = ([float(px[-1])], [float(py[-1])], [float(pz[-1])])
        e_dot._offsets3d = ([float(ex[-1])], [float(ey[-1])], [float(ez[-1])])
        t = i * step * DT
        ax.set_title(f'S004 Obstacle-Course Chase (APF)  t={t:.2f}s')
        return p_trail, e_trail, p_dot, e_dot

    ani = animation.FuncAnimation(fig, update, frames=n_frames, interval=83, blit=False)

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer='pillow', fps=12)
    plt.close()
    print(f'Saved: {path}')


# ── Main ───────────────────────────────────────────────────

if __name__ == '__main__':
    cases = [
        ('With APF obstacle avoidance',    True),
        ('Without obstacle avoidance',     False),
    ]

    results = []
    for label, use_apf in cases:
        res = run_simulation(use_apf)
        captured, cap_t, p_crashed, p_crash_t = res[2], res[3], res[4], res[5]

        if captured:
            status = f'captured at {cap_t:.2f} s'
        elif p_crashed:
            status = f'PURSUER CRASHED at {p_crash_t:.2f} s'
        else:
            status = 'timeout'

        print(f'[{label:>30}]  {status}')
        results.append(res)

    out_dir = os.path.normpath(OUTPUT_DIR)
    plot_trajectories_3d(results, out_dir)
    plot_distance_time(results, out_dir)
    plot_repulsion(results, out_dir)
    save_animation(results, out_dir)
