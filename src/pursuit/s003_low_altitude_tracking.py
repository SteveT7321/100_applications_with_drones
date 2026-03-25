"""
S003 Low-Altitude Tracking
===========================
Pursuer intercepts a stationary target at low altitude while navigating over a Gaussian hill.
Compares two cases: with vs. without terrain-aware altitude control.

Key features:
  - Gaussian hill terrain model
  - Ground effect thrust correction (CF2X propeller)
  - Altitude safety constraint: z >= h_terrain + h_safe
  - Case comparison: terrain avoidance ON vs OFF (crash)

Usage:
    conda activate drones
    python src/pursuit/s003_low_altitude_tracking.py
"""

import sys
import os
import numpy as np
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
    Returns factor >= 1; effective max speed multiplied by k_GE.
    """
    ratio = PROP_RADIUS / max(h_above_terrain, 1e-4)
    return 1.0 + 0.16 * ratio**2 / (1.0 + ratio**2)


# ── Simulation ─────────────────────────────────────────────

def run_simulation(use_terrain_avoidance):
    """
    Pure pursuit toward TARGET_POS from INIT_PURSUER.
    If use_terrain_avoidance: next position is clamped to h_terrain + H_SAFE.
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

        # Log ground effect at current height
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
        ax.fill_between(times, 0, h_terrain, color=color, alpha=0.12)

    # Show terrain height on a reference line-of-flight (x from -4 to 4, y=0)
    x_sample = np.linspace(-4, 4, 300)
    t_sample  = (x_sample - INIT_PURSUER[0]) / PURSUER_SPEED
    h_sample  = terrain_height(x_sample, np.zeros_like(x_sample))
    ax.fill_between(t_sample, 0, h_sample, color='saddlebrown', alpha=0.20,
                    label='Terrain profile (y = 0)')
    ax.plot(t_sample, h_sample + H_SAFE, 'k--', linewidth=1,
            alpha=0.5, label=f'Safety floor (terrain + {H_SAFE} m)')

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
    """Ground effect factor k_GE along the terrain-avoidance trajectory."""
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
    ax.set_title('S003 Low-Altitude Tracking — Ground Effect Factor\n'
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
        results.append((traj, captured, cap_t, crashed, crash_t, k_ge, h_log))

    out_dir = os.path.normpath(OUTPUT_DIR)
    plot_trajectories_3d(results, out_dir)
    plot_altitude_time(results, out_dir)
    plot_ground_effect(results, out_dir)
