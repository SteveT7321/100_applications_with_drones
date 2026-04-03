"""
S021 Point Delivery
===================
Domain: Logistics & Delivery
Difficulty: ⭐

A single delivery drone departs from a depot, cruises to a point above the
target at maximum speed, then descends vertically to deliver a payload on a
ground-level landing pad — minimising total mission time and energy.

Outputs (saved to outputs/02_logistics_delivery/s021_point_delivery/):
  trajectory_3d.png   — 3D flight path with all mission phases labelled
  telemetry.png       — Altitude / Speed / Power vs time subplots
  animation.gif       — Animated 3D drone flight through all phases
"""

import os
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

# ── Output directory ──────────────────────────────────────────────────────────
OUTPUT_DIR = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..", "..", "outputs", "02_logistics_delivery", "s021_point_delivery"
)
os.makedirs(OUTPUT_DIR, exist_ok=True)

# ── Scenario constants ────────────────────────────────────────────────────────
V_MAX       = 5.0          # m/s  maximum horizontal cruise speed
V_Z_MAX     = 1.5          # m/s  maximum vertical descent rate
Z_CRUISE    = 2.0          # m    cruise altitude
R_LAND      = 0.20         # m    landing capture radius
EPSILON_ALN = 0.10         # m    horizontal alignment threshold (cruise→descent)
DT          = 1.0 / 48.0  # s    control timestep (48 Hz)
P_HOVER     = 30.0         # W    hover power
K_DRAG      = 0.6          # W·s²/m²  drag power coefficient
T_MAX       = 60.0         # s    maximum mission time

DEPOT = np.array([-4.0,  0.0, Z_CRUISE])
GOAL  = np.array([ 3.5,  2.5, 0.0])


# ── Velocity command FSM ──────────────────────────────────────────────────────

def get_velocity_cmd(pos: np.ndarray, goal: np.ndarray, phase: str):
    """Return (velocity_command_3d, next_phase)."""
    if phase == "CRUISE":
        diff = goal[:2] - pos[:2]
        dist = np.linalg.norm(diff)
        if dist < EPSILON_ALN:
            return np.zeros(3), "DESCENT"
        return np.append(diff / dist * V_MAX, 0.0), "CRUISE"
    elif phase == "DESCENT":
        if pos[2] <= goal[2] + 0.05:
            return np.zeros(3), "LANDED"
        return np.array([0.0, 0.0, -V_Z_MAX]), "DESCENT"
    return np.zeros(3), "LANDED"


# ── Simulation ────────────────────────────────────────────────────────────────

def run_simulation():
    """Run the time-optimal point-delivery mission and return trajectory data."""
    pos   = DEPOT.copy()
    phase = "CRUISE"

    # Analytical predictions
    d_xy    = np.linalg.norm(GOAL[:2] - DEPOT[:2])
    delta_z = Z_CRUISE - GOAL[2]
    T1_star = d_xy   / V_MAX
    T2_star = delta_z / V_Z_MAX
    T_pred  = T1_star + T2_star

    print(f"[S021] Analytical predictions:")
    print(f"  d_xy   = {d_xy:.3f} m")
    print(f"  T1*    = {T1_star:.3f} s  (cruise)")
    print(f"  T2*    = {T2_star:.3f} s  (descent)")
    print(f"  T_pred = {T_pred:.3f} s")
    print()

    # Storage
    times, positions, velocities, powers, phases_log = [], [], [], [], []

    t = 0.0
    while phase != "LANDED" and t < T_MAX:
        v_cmd, phase = get_velocity_cmd(pos, GOAL, phase)

        speed  = np.linalg.norm(v_cmd)
        power  = P_HOVER + K_DRAG * speed ** 2

        times.append(t)
        positions.append(pos.copy())
        velocities.append(speed)
        powers.append(power)
        phases_log.append(phase)

        pos = pos + v_cmd * DT
        # clamp altitude to ground
        pos[2] = max(pos[2], GOAL[2])
        t += DT

    # Final snapshot
    times.append(t)
    positions.append(pos.copy())
    velocities.append(0.0)
    powers.append(P_HOVER)
    phases_log.append("LANDED")

    times      = np.array(times)
    positions  = np.array(positions)
    velocities = np.array(velocities)
    powers     = np.array(powers)

    # Mission metrics
    T_mission  = times[-1]
    landing_err = np.linalg.norm(positions[-1] - GOAL)
    energy = np.trapz(powers, times)

    print(f"[S021] Simulation results:")
    print(f"  Mission time   = {T_mission:.3f} s  (predicted {T_pred:.3f} s)")
    print(f"  Landing error  = {landing_err:.4f} m  (limit {R_LAND} m)")
    print(f"  Energy used    = {energy:.2f} J")
    print(f"  Landing OK?    = {landing_err < R_LAND}")

    return {
        "times": times,
        "positions": positions,
        "velocities": velocities,
        "powers": powers,
        "phases_log": phases_log,
        "T_mission": T_mission,
        "T1_star": T1_star,
        "T2_star": T2_star,
        "T_pred": T_pred,
        "landing_err": landing_err,
        "energy": energy,
    }


# ── Plot 1: 3D Trajectory ─────────────────────────────────────────────────────

def plot_trajectory_3d(data: dict):
    pos = data["positions"]

    fig = plt.figure(figsize=(9, 7))
    ax  = fig.add_subplot(111, projection="3d")

    # Split phases by altitude change
    cruise_mask  = pos[:, 2] >= Z_CRUISE - 0.05
    descent_mask = ~cruise_mask

    # Cruise segment
    ax.plot(pos[cruise_mask, 0], pos[cruise_mask, 1], pos[cruise_mask, 2],
            color="crimson", linewidth=2.5, label="Cruise phase")

    # Descent segment
    ax.plot(pos[descent_mask, 0], pos[descent_mask, 1], pos[descent_mask, 2],
            color="crimson", linewidth=2.5, linestyle="--", label="Descent phase")

    # Depot marker
    ax.scatter(*DEPOT, color="royalblue", s=120, marker="s",
               zorder=5, label="Depot (start)")

    # Goal / landing pad
    ax.scatter(*GOAL, color="limegreen", s=200, marker="*",
               zorder=5, label="Landing pad (goal)")

    # Landing circle (ground level visualisation)
    theta  = np.linspace(0, 2 * np.pi, 64)
    cx, cy = GOAL[0] + R_LAND * np.cos(theta), GOAL[1] + R_LAND * np.sin(theta)
    ax.plot(cx, cy, np.zeros_like(cx) + GOAL[2],
            color="limegreen", linewidth=1.0, linestyle=":", alpha=0.7)

    # Cruise altitude plane annotation
    ax.set_zlim(0, Z_CRUISE + 0.5)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title(
        f"S021 Point Delivery — 3D Trajectory\n"
        f"T_mission = {data['T_mission']:.2f} s, "
        f"Energy = {data['energy']:.1f} J, "
        f"Landing err = {data['landing_err']*100:.1f} cm",
        fontsize=10
    )
    ax.legend(loc="upper left", fontsize=8)

    out = os.path.join(OUTPUT_DIR, "trajectory_3d.png")
    fig.savefig(out, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"[S021] Saved {out}")


# ── Plot 2: Telemetry subplots ────────────────────────────────────────────────

def plot_telemetry(data: dict):
    t   = data["times"]
    pos = data["positions"]
    vel = data["velocities"]
    pwr = data["powers"]

    T1 = data["T1_star"]  # approximate cruise end time

    fig, axes = plt.subplots(3, 1, figsize=(9, 9), sharex=True)
    fig.suptitle("S021 Point Delivery — Telemetry", fontsize=13, fontweight="bold")

    # --- Altitude ---
    ax = axes[0]
    ax.plot(t, pos[:, 2], color="steelblue", linewidth=2)
    ax.axhline(Z_CRUISE, color="gray", linestyle=":", linewidth=1, label=f"Cruise alt {Z_CRUISE} m")
    ax.axhline(GOAL[2],  color="limegreen", linestyle="--", linewidth=1, label="Ground (0 m)")
    ax.axvline(T1, color="orange", linestyle="--", linewidth=1, label=f"Cruise→Descent @ {T1:.2f}s")
    ax.scatter(t[-1], pos[-1, 2], color="crimson", s=80, zorder=5, label="Touchdown")
    ax.set_ylabel("Altitude (m)")
    ax.set_ylim(-0.1, Z_CRUISE + 0.5)
    ax.legend(fontsize=7, loc="upper right")
    ax.grid(True, alpha=0.3)

    # --- Speed ---
    ax = axes[1]
    ax.plot(t, vel, color="tomato", linewidth=2)
    ax.axhline(V_MAX,   color="gray", linestyle=":", linewidth=1, label=f"v_max = {V_MAX} m/s")
    ax.axhline(V_Z_MAX, color="salmon", linestyle=":", linewidth=1, label=f"v_z_max = {V_Z_MAX} m/s")
    ax.axvline(T1, color="orange", linestyle="--", linewidth=1)
    ax.set_ylabel("Speed (m/s)")
    ax.set_ylim(-0.1, V_MAX + 0.5)
    ax.legend(fontsize=7, loc="upper right")
    ax.grid(True, alpha=0.3)

    # --- Power ---
    ax = axes[2]
    ax.plot(t, pwr, color="darkorchid", linewidth=2)
    ax.axhline(P_HOVER, color="gray", linestyle=":", linewidth=1, label=f"P_hover = {P_HOVER} W")
    ax.axvline(T1, color="orange", linestyle="--", linewidth=1)
    ax.fill_between(t, pwr, P_HOVER, alpha=0.15, color="darkorchid", label="Extra drag power")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Power (W)")
    ax.legend(fontsize=7, loc="upper right")
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    out = os.path.join(OUTPUT_DIR, "telemetry.png")
    fig.savefig(out, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"[S021] Saved {out}")


# ── Animation ─────────────────────────────────────────────────────────────────

def save_animation(data: dict):
    pos = data["positions"]
    N   = len(pos)

    # Subsample to keep GIF manageable (~15 fps target, source is 48 Hz)
    step    = max(1, N // 120)
    indices = list(range(0, N, step))
    if indices[-1] != N - 1:
        indices.append(N - 1)
    frames = [pos[i] for i in indices]

    fig = plt.figure(figsize=(7, 6))
    ax  = fig.add_subplot(111, projection="3d")

    ax.set_xlim(min(DEPOT[0], GOAL[0]) - 0.5, max(DEPOT[0], GOAL[0]) + 0.5)
    ax.set_ylim(min(DEPOT[1], GOAL[1]) - 0.5, max(DEPOT[1], GOAL[1]) + 0.5)
    ax.set_zlim(GOAL[2] - 0.1, Z_CRUISE + 0.4)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")

    # Static markers
    ax.scatter(*DEPOT, color="royalblue", s=100, marker="s", zorder=5)
    ax.scatter(*GOAL,  color="limegreen", s=160, marker="*", zorder=5)

    # Landing circle
    theta = np.linspace(0, 2 * np.pi, 64)
    ax.plot(GOAL[0] + R_LAND * np.cos(theta),
            GOAL[1] + R_LAND * np.sin(theta),
            np.zeros(64) + GOAL[2],
            color="limegreen", linewidth=0.8, linestyle=":", alpha=0.6)

    trail_line, = ax.plot([], [], [], color="crimson", linewidth=1.5, alpha=0.6)
    drone_dot,  = ax.plot([], [], [], "o", color="crimson", markersize=8)

    title_obj = ax.set_title("S021 Point Delivery — t = 0.00 s")

    trail_x, trail_y, trail_z = [], [], []

    def init():
        trail_line.set_data([], [])
        trail_line.set_3d_properties([])
        drone_dot.set_data([], [])
        drone_dot.set_3d_properties([])
        return trail_line, drone_dot, title_obj

    def update(frame_idx):
        p = frames[frame_idx]
        trail_x.append(p[0])
        trail_y.append(p[1])
        trail_z.append(p[2])
        trail_line.set_data(trail_x, trail_y)
        trail_line.set_3d_properties(trail_z)
        drone_dot.set_data([p[0]], [p[1]])
        drone_dot.set_3d_properties([p[2]])
        sim_t = indices[frame_idx] * DT
        title_obj.set_text(f"S021 Point Delivery — t = {sim_t:.2f} s")
        return trail_line, drone_dot, title_obj

    ani = animation.FuncAnimation(
        fig, update, frames=len(frames),
        init_func=init, interval=67, blit=False
    )

    out = os.path.join(OUTPUT_DIR, "animation.gif")
    ani.save(out, writer="pillow", fps=15)
    plt.close(fig)
    print(f"[S021] Saved {out}")


# ── Entry point ───────────────────────────────────────────────────────────────

if __name__ == "__main__":
    print("=" * 60)
    print("S021 Point Delivery — Time-Optimal Drone Delivery")
    print("=" * 60)

    sim_data = run_simulation()
    plot_trajectory_3d(sim_data)
    plot_telemetry(sim_data)
    save_animation(sim_data)

    print()
    print("=" * 60)
    print("S021 complete. Outputs written to:")
    print(f"  {OUTPUT_DIR}")
    print("=" * 60)
