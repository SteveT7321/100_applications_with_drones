"""
S004 Obstacle-Course Chase
===========================
Pursuer (Attractive-Repulsive Potential Field) chases an evader through a 3-D
obstacle course of spherical obstacles.

Comparison:
  - With obstacle avoidance: pursuer steers around obstacles using APF
  - Without obstacle avoidance: pure pursuit — crashes on first obstacle

Both agents have the same APF obstacle repulsion; only the pursuer comparison
case is stripped of it. Both agents are confined to the same walled arena
x in [-5, 5], y in [-3, 3] (hard walls, agents slide along them), so the
evader cannot leave the obstacle course. An obstacle-free run in the same
arena is used as the reference for the detour cost.

Usage:
    conda activate drones
    python src/01_pursuit_evasion/s004_obstacle_chase.py
"""

import sys
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
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

# Bounded arena (hard walls for BOTH agents): the obstacle course is enclosed,
# so the evader cannot run out into open space and the pursuer has to get
# through the obstacles to reach it.
ARENA_LO = np.array([-5.0, -3.0, 0.0])
ARENA_HI = np.array([ 5.0,  3.0, 4.0])

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

def bounded_velocity(pos, v_cmd, max_speed):
    """
    Clamp speed to max_speed, then shorten the velocity so that the next
    position stays inside the arena box (wall = hard constraint, the agent
    slides along it). Applied identically to pursuer and evader.
    """
    speed = np.linalg.norm(v_cmd)
    if speed > max_speed:
        v_cmd = v_cmd / speed * max_speed
    next_pos = np.clip(pos + v_cmd * DT, ARENA_LO, ARENA_HI)
    return (next_pos - pos) / DT


def run_simulation(pursuer_uses_apf, obstacles=OBSTACLES):
    """
    The evader always uses straight escape + APF obstacle repulsion.
    Pursuer uses APF only when pursuer_uses_apf=True; else pure pursuit.
    Both agents are confined to the arena box [ARENA_LO, ARENA_HI].
    `obstacles` can be an empty (0, 4) array for the obstacle-free reference.
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
        if inside_obstacle(pursuer.pos, obstacles):
            p_crashed    = True
            p_crash_time = t
            break
        if np.linalg.norm(pursuer.pos - evader.pos) < CAPTURE_R:
            captured     = True
            capture_time = t
            break

        # ── Evader crash ──
        if inside_obstacle(evader.pos, obstacles):
            e_crashed    = True
            e_crash_time = t
            break

        # ── Pursuer velocity ──
        r_pe  = evader.pos - pursuer.pos
        v_att = PURSUER_SPEED * r_pe / (np.linalg.norm(r_pe) + 1e-8)
        if pursuer_uses_apf:
            v_rep = repulsion_velocity(pursuer.pos, obstacles)
            rep_mag_log.append(np.linalg.norm(v_rep))
            v_p = v_att + v_rep
        else:
            rep_mag_log.append(0.0)
            v_p = v_att
        pursuer.step(bounded_velocity(pursuer.pos, v_p, PURSUER_SPEED))

        # ── Evader velocity (straight escape + APF) ──
        r_ep  = evader.pos - pursuer.pos
        v_esc = EVADER_SPEED * r_ep / (np.linalg.norm(r_ep) + 1e-8)
        v_rep_e = repulsion_velocity(evader.pos, obstacles)
        evader.step(bounded_velocity(evader.pos, v_esc + v_rep_e, EVADER_SPEED))

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
    """Axis limits: the arena in x/y, trajectories +/- margin in z."""
    all_pts = np.vstack([
        np.vstack([res[0], res[1]]) for res in results
    ])
    lo = ARENA_LO.copy()
    hi = ARENA_HI.copy()
    lo[2] = all_pts[:, 2].min() - margin
    hi[2] = all_pts[:, 2].max() + margin
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


def plot_distance_time(results, out_dir, ref=None):
    labels = ['With APF avoidance', 'Without avoidance']
    colors = ['steelblue', 'darkorange']
    if ref is not None:
        results = list(results) + [ref]
        labels  = labels + ['Obstacle-free reference']
        colors  = colors + ['gray']

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
    ani.save(path, writer='pillow', fps=12, dpi=100)
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

    # Obstacle-free reference: same arena, same agents, no obstacles
    ref = run_simulation(True, obstacles=np.zeros((0, 4)))
    ref_t = ref[3]
    print(f'[{"Obstacle-free reference":>30}]  '
          + (f'captured at {ref_t:.2f} s' if ref[2] else 'timeout'))

    # Diagnostics for the APF case
    p_traj, e_traj, captured, cap_t = results[0][:4]
    rep_log = results[0][8]
    print(f'APF case: evader final position ({e_traj[-1, 0]:.2f}, '
          f'{e_traj[-1, 1]:.2f}, {e_traj[-1, 2]:.2f}) m')
    if captured and ref[2]:
        print(f'APF case: detour cost vs obstacle-free reference: '
              f'{cap_t - ref_t:.2f} s')
    for i, (cx, cy, cz, r) in enumerate(OBSTACLES, start=1):
        d_p = np.min(np.linalg.norm(p_traj - [cx, cy, cz], axis=1) - r)
        d_e = np.min(np.linalg.norm(e_traj - [cx, cy, cz], axis=1) - r)
        flag = 'inside rho0' if d_p < RHO0 else 'outside rho0'
        print(f'  Obstacle {i}: closest surface distance pursuer {d_p:.2f} m '
              f'({flag}), evader {d_e:.2f} m')
    active = rep_log > 0
    print(f'APF case: repulsion active {active.sum()} / {len(rep_log)} steps '
          f'({100 * active.mean():.0f}% of the chase), '
          f'peak |v_rep| {rep_log.max():.2f} m/s')
    p_speed = np.linalg.norm(np.diff(p_traj, axis=0), axis=1) / DT
    stall = p_speed < 0.5
    if stall.any():
        print(f'APF case: pursuer slower than 0.5 m/s for {stall.sum() * DT:.2f} s '
              f'(local minimum), mean stall position '
              f'({p_traj[1:][stall, 0].mean():.2f}, {p_traj[1:][stall, 1].mean():.2f}) m')
    at_wall = np.nonzero(e_traj[:, 0] >= ARENA_HI[0] - 1e-9)[0]
    if len(at_wall):
        print(f'APF case: evader reaches the x = {ARENA_HI[0]:.0f} m wall at '
              f't = {at_wall[0] * DT:.2f} s at y = {e_traj[at_wall[0], 1]:.2f} m')
    p_len = np.sum(np.linalg.norm(np.diff(p_traj, axis=0), axis=1))
    print(f'APF case: pursuer path length {p_len:.2f} m, '
          f'mean speed {p_len / cap_t:.2f} m/s' if captured else '')

    out_dir = os.path.normpath(OUTPUT_DIR)
    plot_trajectories_3d(results, out_dir)
    plot_distance_time(results, out_dir, ref)
    plot_repulsion(results, out_dir)
    save_animation(results, out_dir)
