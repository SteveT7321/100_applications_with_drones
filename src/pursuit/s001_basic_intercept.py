"""
S001 Basic Intercept
====================
Pursuer starts at (-2, -2, 1) and intercepts a stationary target at (3, 2, 2)
using a PD controller (simplified Proportional Navigation Guidance).

Usage:
    conda activate drones
    python src/pursuit/s001_basic_intercept.py
"""

import sys
import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

# Allow Python to find src/base
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from src.base.drone_base import DroneBase

# ── Scenario parameters ───────────────────────────────────
INIT_POS    = np.array([-2.0, -2.0, 1.0])
TARGET_POS  = np.array([ 3.0,  2.0, 2.0])
MAX_SPEED   = 5.0       # m/s
DT          = 1 / 48    # control frequency 48 Hz
MAX_TIME    = 30.0      # seconds
CAPTURE_R   = 0.15      # capture radius (m)

# PD gains
Kp = 2.0
Kd = 0.5

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), '..', '..', 'outputs', '01_pursuit_evasion', 's001_basic_intercept')


def run_simulation():
    drone = DroneBase(INIT_POS, max_speed=MAX_SPEED, dt=DT)
    max_steps = int(MAX_TIME / DT)
    captured = False
    capture_time = None

    for step in range(max_steps):
        error = TARGET_POS - drone.pos
        dist = np.linalg.norm(error)

        if dist < CAPTURE_R:
            captured = True
            capture_time = step * DT
            break

        v_cmd = Kp * error + Kd * (-drone.vel)
        drone.step(v_cmd)

    return drone.get_trajectory(), captured, capture_time


def plot_trajectory_3d(traj, captured, capture_time, out_dir):
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection='3d')

    ax.plot(traj[:, 0], traj[:, 1], traj[:, 2],
            color='red', linewidth=1.5, label='Pursuer trajectory')
    ax.scatter(float(traj[0, 0]), float(traj[0, 1]), float(traj[0, 2]),
               color='red', s=60, marker='o', zorder=5, label='Start')
    ax.scatter(float(TARGET_POS[0]), float(TARGET_POS[1]), float(TARGET_POS[2]),
               color='green', s=120, marker='*', zorder=5, label='Target')

    if captured:
        ax.scatter(float(traj[-1, 0]), float(traj[-1, 1]), float(traj[-1, 2]),
                   color='black', s=100, marker='X', zorder=5,
                   label=f'Captured ({capture_time:.2f}s)')

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('S001 Basic Intercept — 3D Trajectory')
    ax.legend(loc='upper left')

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'trajectory_3d.png')
    plt.tight_layout()
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def plot_distance_time(traj, out_dir):
    dists = np.linalg.norm(traj - TARGET_POS, axis=1)
    times = np.arange(len(dists)) * DT

    _, ax = plt.subplots(figsize=(8, 4))
    ax.plot(times, dists, color='steelblue', linewidth=1.5)
    ax.axhline(CAPTURE_R, color='red', linestyle='--', linewidth=1, label=f'Capture radius {CAPTURE_R} m')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Distance (m)')
    ax.set_title('S001 Basic Intercept — Distance vs Time')
    ax.legend()
    ax.grid(True, alpha=0.3)

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'distance_time.png')
    plt.tight_layout()
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def save_animation(traj, out_dir):
    """Sample every 4 steps (~12 fps) and export as GIF."""
    step = 4
    frames = traj[::step]

    fig = plt.figure(figsize=(7, 6))
    ax = fig.add_subplot(111, projection='3d')

    # Fix axis limits
    margin = 0.5
    lo = traj.min(axis=0) - margin
    hi = traj.max(axis=0) + margin
    ax.set_xlim(lo[0], hi[0])
    ax.set_ylim(lo[1], hi[1])
    ax.set_zlim(lo[2], hi[2])
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('S001 Basic Intercept — Animation')

    # Static elements
    ax.scatter(float(TARGET_POS[0]), float(TARGET_POS[1]), float(TARGET_POS[2]),
               color='green', s=150, marker='*', zorder=5, label='Target')
    ax.scatter(float(traj[0, 0]), float(traj[0, 1]), float(traj[0, 2]),
               color='red', s=60, marker='o', alpha=0.4, label='Start')
    ax.legend(loc='upper left')

    trail, = ax.plot([], [], [], color='red', linewidth=1.2, alpha=0.6)
    dot = ax.scatter([], [], [], color='red', s=80, marker='^', zorder=6)

    def update(i):
        x, y, z = frames[:i+1, 0], frames[:i+1, 1], frames[:i+1, 2]
        trail.set_data(x, y)
        trail.set_3d_properties(z)
        dot._offsets3d = ([float(x[-1])], [float(y[-1])], [float(z[-1])])
        t = i * step * DT
        ax.set_title(f'S001 Basic Intercept  t={t:.2f}s')
        return trail, dot

    ani = animation.FuncAnimation(fig, update, frames=len(frames),
                                  interval=83, blit=False)

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer='pillow', fps=12)
    plt.close()
    print(f'Saved: {path}')


if __name__ == '__main__':
    traj, captured, capture_time = run_simulation()

    if captured:
        final_dist = np.linalg.norm(traj[-1] - TARGET_POS)
        print(f'Captured! Time: {capture_time:.2f}s, Distance: {final_dist:.3f}m')
    else:
        print(f'Not captured (exceeded {MAX_TIME}s)')

    out_dir = os.path.normpath(OUTPUT_DIR)
    plot_trajectory_3d(traj, captured, capture_time, out_dir)
    plot_distance_time(traj, out_dir)
    save_animation(traj, out_dir)
