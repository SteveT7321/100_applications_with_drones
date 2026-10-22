"""
S003 Low-Altitude Tracking
===========================
Pursuer intercepts a stationary target at low altitude while navigating over a Gaussian hill.
Compares two cases: with vs. without terrain-aware altitude control.

Key features:
  - Gaussian hill terrain model
  - Altitude safety constraint: z >= h_terrain + h_safe (climb has priority
    over horizontal speed when the 5 m/s limit is reached)
  - Ground effect factor k_GE (CF2X propeller) evaluated along the flown path
    as a DIAGNOSTIC only: the kinematic point-mass model has no thrust, so
    k_GE does not enter the dynamics
  - Case comparison: terrain avoidance ON vs OFF (crash)

Usage:
    conda activate drones
    python src/01_pursuit_evasion/s003_low_altitude_tracking.py
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
PURSUER_SPEED = 5.0      # m/s
DT            = 1 / 48   # 48 Hz
MAX_TIME      = 10.0     # s
CAPTURE_R     = 0.15     # m
H_SAFE        = 0.3      # m  — safety margin above terrain

INIT_PURSUER = np.array([-4.0, 0.0, 0.5])
TARGET_POS   = np.array([ 4.0, 0.0, 0.5])

# Terrain — Gaussian hill directly on the pursuer-to-target path
HILL_X, HILL_Y = 0.0, 0.0
HILL_H         = 0.8      # peak height (m)
HILL_SIGMA2    = 1.5      # Gaussian width (m²)

# Ground effect (CF2X propeller)
PROP_RADIUS = 0.05  # m

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '01_pursuit_evasion', 's003_low_altitude_tracking',
)


# ── Terrain & ground effect ────────────────────────────────

def terrain_height(x, y):
    """Gaussian hill: h(x,y) = H * exp(-((x-x0)²+(y-y0)²)/σ²)"""
    return HILL_H * np.exp(-((x - HILL_X)**2 + (y - HILL_Y)**2) / HILL_SIGMA2)


def ground_effect_factor(h_above_terrain):
    """
    Thrust correction for ground effect.
    k_GE(h) = 1 + 0.16*(R/h)² / (1 + (R/h)²)
    Returns factor >= 1. Diagnostic only — logged along the trajectory,
    not applied to the (velocity-commanded) point-mass dynamics.
    """
    ratio = PROP_RADIUS / max(h_above_terrain, 1e-4)
    return 1.0 + 0.16 * ratio**2 / (1.0 + ratio**2)


# ── Simulation ─────────────────────────────────────────────

def run_simulation(use_terrain_avoidance):
    """
    Pure pursuit toward TARGET_POS from INIT_PURSUER.
    If use_terrain_avoidance: next position is lifted to h_terrain + H_SAFE;
    if the lifted step would exceed PURSUER_SPEED, the vertical component is
    kept and the horizontal component is shortened so that |v| = PURSUER_SPEED.
    Otherwise: drone only avoids z < 0 (flat ground), can hit the hill.
    """
    pursuer = DroneBase(INIT_PURSUER, max_speed=PURSUER_SPEED, dt=DT)

    max_steps  = int(MAX_TIME / DT)
    captured   = False
    capture_time = None
    crashed    = False
    crash_time = None
    k_ge_log   = []
    terrain_log = []

    for step in range(max_steps):
        x, y, z = pursuer.pos
        h_cur = terrain_height(x, y)
        terrain_log.append(h_cur)

        # Crash: drone centre below terrain surface
        if z < h_cur - 0.02:
            crashed    = True
            crash_time = step * DT
            break

        # Capture
        if np.linalg.norm(pursuer.pos - TARGET_POS) < CAPTURE_R:
            captured     = True
            capture_time = step * DT
            break

        # Log ground effect at current height (diagnostic only)
        h_above = max(z - h_cur, 1e-4)
        k_ge_log.append(ground_effect_factor(h_above))

        # Pure pursuit: project one step toward target
        r     = TARGET_POS - pursuer.pos
        r_hat = r / (np.linalg.norm(r) + 1e-8)
        next_pos = pursuer.pos + r_hat * PURSUER_SPEED * DT

        if use_terrain_avoidance:
            xn, yn = next_pos[0], next_pos[1]
            z_safe = terrain_height(xn, yn) + H_SAFE
            next_pos[2] = max(next_pos[2], z_safe)

        # Hard floor: never below flat ground
        next_pos[2] = max(next_pos[2], 0.0)

        v_cmd = (next_pos - pursuer.pos) / DT
        speed = np.linalg.norm(v_cmd)
        if speed > PURSUER_SPEED:
            # Climb has priority: keep vz, shrink horizontal speed
            vz = np.clip(v_cmd[2], -PURSUER_SPEED, PURSUER_SPEED)
            v_xy = v_cmd[:2]
            v_xy_max = np.sqrt(max(PURSUER_SPEED**2 - vz**2, 0.0))
            v_xy = v_xy * v_xy_max / (np.linalg.norm(v_xy) + 1e-12)
            v_cmd = np.array([v_xy[0], v_xy[1], vz])
        pursuer.step(v_cmd)

    return (
        pursuer.get_trajectory(),
        captured, capture_time,
        crashed,  crash_time,
        np.array(k_ge_log),
        np.array(terrain_log),
    )


# ── Plotting helpers ───────────────────────────────────────

def _add_terrain_surface(ax, x_range=(-5, 5), y_range=(-3, 3), n=50, alpha=0.25):
    xs = np.linspace(*x_range, n)
    ys = np.linspace(*y_range, n)
    X, Y = np.meshgrid(xs, ys)
    Z    = terrain_height(X, Y)
    ax.plot_surface(X, Y, Z, color='saddlebrown', alpha=alpha, linewidth=0, zorder=1)


# ── Plots ──────────────────────────────────────────────────

def plot_trajectories_3d(results, out_dir):
    """Side-by-side 3D trajectories with terrain surface."""
    labels = ['With terrain avoidance', 'Without terrain avoidance']
    fig = plt.figure(figsize=(14, 6))

    for i, (label, (traj, captured, cap_t, crashed, crash_t, k_ge, h_log)) in enumerate(
            zip(labels, results)):
        ax = fig.add_subplot(1, 2, i + 1, projection='3d')
        _add_terrain_surface(ax)

        ax.plot(traj[:, 0], traj[:, 1], traj[:, 2],
                color='red', linewidth=1.8, label='Pursuer', zorder=5)
        ax.scatter(*traj[0], color='red',   s=50,  marker='o', zorder=6)
        ax.scatter(*TARGET_POS, color='green', s=120, marker='*', zorder=6,
                   label='Target')

        if captured:
            ax.scatter(*traj[-1], color='black', s=100, marker='X', zorder=7,
                       label=f'Captured {cap_t:.2f} s')
        if crashed:
            ax.scatter(*traj[-1], color='orange', s=120, marker='X', zorder=7,
                       label=f'CRASHED {crash_t:.2f} s')

        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        status = (f'Captured {cap_t:.2f} s' if captured
                  else f'CRASHED {crash_t:.2f} s' if crashed
                  else 'Not captured')
        ax.set_title(f'{label}\n{status}', fontsize=9)
        ax.legend(loc='upper left', fontsize=7)
        ax.set_zlim(0, 2)

    fig.suptitle('S003 Low-Altitude Tracking — 3D Trajectories', fontsize=12, y=1.01)
    plt.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'trajectories_3d.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_altitude_time(results, out_dir):
    """Altitude vs time — pursuer z and terrain height at pursuer's (x,y)."""
    labels = ['With avoidance', 'Without avoidance']
    colors = ['red', 'darkorange']

    fig, ax = plt.subplots(figsize=(10, 5))

    for label, color, (traj, captured, cap_t, crashed, crash_t, k_ge, h_log) in \
            zip(labels, colors, results):
        n = len(h_log)
        times = np.arange(n) * DT
        z_drone   = traj[:n, 2]
        h_terrain = h_log

        ax.plot(times, z_drone, color=color, linewidth=1.8, label=f'Drone z ({label})')
        ax.fill_between(times, 0, h_terrain, color=color, alpha=0.15,
                        label=f'Terrain under drone ({label})')
        if label == 'With avoidance':
            # Safety floor evaluated at the drone's actual (x, y) at each time
            ax.plot(times, h_terrain + H_SAFE, 'k--', linewidth=1, alpha=0.6,
                    label=f'Safety floor under drone (terrain + {H_SAFE} m)')

    ax.axhline(0.5, color='gray', linestyle=':', linewidth=1, label='Start/Target altitude (0.5 m)')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Altitude (m)')
    ax.set_title('S003 Low-Altitude Tracking — Drone Altitude vs Time')
    ax.set_ylim(0, 1.6)
    ax.set_xlim(0, None)
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'altitude_time.png')
    plt.tight_layout()
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def plot_ground_effect(results, out_dir):
    """Ground effect factor k_GE along the terrain-avoidance trajectory (diagnostic)."""
    traj, captured, cap_t, crashed, crash_t, k_ge, h_log = results[0]  # avoidance case
    n = len(k_ge)
    times = np.arange(n) * DT

    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(times, k_ge, color='steelblue', linewidth=1.5, label='$k_{GE}$ (with avoidance)')
    ax.axhline(1.0, color='gray', linestyle='--', linewidth=1, label='No ground effect')
    ax.fill_between(times, 1.0, k_ge, where=(k_ge > 1.0), color='steelblue',
                    alpha=0.2, label='Ground effect zone')

    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Ground Effect Factor $k_{GE}$')
    ax.set_title('S003 Low-Altitude Tracking — Ground Effect Factor (diagnostic, not applied)\n'
                 f'(propeller radius R = {PROP_RADIUS} m, CF2X model)')
    ax.set_xlim(0, times[-1])
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'ground_effect.png')
    plt.tight_layout()
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def save_animation(results, out_dir):
    """Animate the terrain-avoidance case with terrain surface, ~12 fps."""
    import matplotlib.animation as animation

    traj, captured, cap_t, crashed, crash_t, k_ge, h_log = results[0]  # avoidance case

    step = 4
    frames = traj[::step]
    n_frames = len(frames)

    fig = plt.figure(figsize=(7, 6))
    ax = fig.add_subplot(111, projection='3d')
    _add_terrain_surface(ax)

    ax.set_xlim(-5, 5)
    ax.set_ylim(-3, 3)
    ax.set_zlim(0, 2)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')

    ax.scatter(*TARGET_POS, color='green', s=120, marker='*', zorder=5, label='Target')
    ax.scatter(*traj[0], color='red', s=50, marker='o', alpha=0.4, label='Start')
    ax.legend(loc='upper left')

    trail, = ax.plot([], [], [], color='red', linewidth=1.2, alpha=0.6)
    dot = ax.scatter([], [], [], color='red', s=80, marker='^', zorder=6)

    def update(i):
        x, y, z = frames[:i+1, 0], frames[:i+1, 1], frames[:i+1, 2]
        trail.set_data(x, y)
        trail.set_3d_properties(z)
        dot._offsets3d = ([float(x[-1])], [float(y[-1])], [float(z[-1])])
        t = i * step * DT
        ax.set_title(f'S003 Low-Altitude Tracking (avoidance ON)  t={t:.2f}s')
        return trail, dot

    ani = animation.FuncAnimation(fig, update, frames=n_frames, interval=83, blit=False)

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer='pillow', fps=12)
    plt.close()
    print(f'Saved: {path}')


# ── Main ───────────────────────────────────────────────────

if __name__ == '__main__':
    cases = [
        ('With terrain avoidance',    True),
        ('Without terrain avoidance', False),
    ]

    results = []
    for label, use_avoidance in cases:
        (traj, captured, cap_t,
         crashed, crash_t, k_ge, h_log) = run_simulation(use_avoidance)

        if captured:
            status = f'captured at {cap_t:.2f} s'
        elif crashed:
            status = f'CRASHED at {crash_t:.2f} s'
        else:
            status = 'not captured (timeout)'

        print(f'[{label:>28}]  {status}')
        clr = traj[:, 2] - terrain_height(traj[:, 0], traj[:, 1])
        path_len = np.sum(np.linalg.norm(np.diff(traj, axis=0), axis=1))
        print(f'{"":>32}path length {path_len:.3f} m, peak z {traj[:, 2].max():.3f} m, '
              f'min clearance above terrain {clr.min():.4f} m, '
              f'max k_GE {k_ge.max():.4f}')
        if use_avoidance:
            above = clr - H_SAFE                      # height above the safety floor
            dz = np.diff(traj[:, 2])
            climb = np.nonzero(dz > 1e-9)[0] + 1      # positions reached while climbing
            on_floor = np.nonzero(np.abs(above) < 5e-4)[0]
            off = [k for k in climb if abs(above[k]) >= 5e-4]
            print(f'{"":>32}climb: {len(climb)} steps; off-floor climb steps are '
                  f'{1000 * min(above[k] for k in off):.1f}-'
                  f'{1000 * max(above[k] for k in off):.1f} mm above the safety floor')
            print(f'{"":>32}on the floor (|gap| < 0.5 mm) at t = '
                  + ', '.join(f'{k * DT:.3f}' for k in on_floor) + ' s')
        results.append((traj, captured, cap_t, crashed, crash_t, k_ge, h_log))

    d_straight = np.linalg.norm(TARGET_POS - INIT_PURSUER) - CAPTURE_R
    print(f'Straight-line path to capture: {d_straight:.3f} m '
          f'({d_straight / PURSUER_SPEED:.3f} s at {PURSUER_SPEED} m/s)')
    x_act = np.sqrt(HILL_SIGMA2 * np.log(HILL_H / (INIT_PURSUER[2] - H_SAFE)))
    print(f'Safety floor rises above start altitude for |x| < {x_act:.3f} m')

    out_dir = os.path.normpath(OUTPUT_DIR)
    plot_trajectories_3d(results, out_dir)
    plot_altitude_time(results, out_dir)
    plot_ground_effect(results, out_dir)
    save_animation(results, out_dir)
