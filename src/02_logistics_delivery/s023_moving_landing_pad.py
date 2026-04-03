"""
S023 Moving Landing Pad
=======================
A delivery drone intercepts and lands on a moving truck (simulated as a ground vehicle).

Two-phase guidance:
  Phase 1 — Proportional Navigation to predicted intercept point
  Phase 2 — PD descent + velocity matching + tanh flare

Outputs (saved to outputs/02_logistics_delivery/s023_moving_landing_pad/):
  - trajectory_3d.png       : 3-D flight path with truck road and pad
  - metrics_time_series.png : altitude, lateral miss, velocity mismatch vs time
  - animation.gif           : top-down + side-view animated trajectory
"""

import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401
from matplotlib.animation import FuncAnimation, PillowWriter

# ── Output directory ──────────────────────────────────────────────────────────
OUT_DIR = os.path.join(
    os.path.dirname(__file__),
    "..", "..",
    "outputs", "02_logistics_delivery", "s023_moving_landing_pad"
)
OUT_DIR = os.path.normpath(OUT_DIR)
os.makedirs(OUT_DIR, exist_ok=True)

# ── Simulation constants ──────────────────────────────────────────────────────
DT          = 0.02          # time step (s) — 50 Hz
SIM_TIME    = 30.0          # total simulation budget (s)
N_STEPS     = int(SIM_TIME / DT)

DRONE_VMAX  = 8.0           # m/s  — max speed
DRONE_AMAX  = 5.0           # m/s² — max acceleration
TRUCK_SPEED = 4.0           # m/s  — constant forward speed (x-axis)

N_PNG       = 3             # PNG navigation constant
Z0          = 15.0          # initial drone altitude (m)
Z_SWITCH    = 5.0           # altitude threshold for Phase 2 (m)
R_SWITCH    = 3.0           # lateral radius threshold for Phase 2 (m)

R_PAD       = 0.5           # landing pad radius (m)
V_LAND_MAX  = 1.0           # max acceptable touchdown |vz| (m/s)
DV_MAX      = 0.5           # max acceptable horizontal velocity mismatch (m/s)
Z_FLARE     = 2.0           # flare altitude constant (m)
Z_TOUCHDOWN = 0.05          # touchdown detection threshold (m)

# Phase-2 PD gains
KP      = 1.2
KV      = 1.8
KZ_RATE = 2.0               # descent rate controller gain


# ── Guidance functions ────────────────────────────────────────────────────────

def predict_intercept(p_drone, p_truck, v_truck_vec, v_drone_max, n_iter=5):
    """Iterative fixed-point intercept prediction (horizontal plane only)."""
    # Work in 2-D (x,y) — the intercept target is at truck ground level
    p_drone_2d  = p_drone[:2]
    p_truck_2d  = p_truck[:2]
    v_truck_2d  = v_truck_vec[:2]

    dt_fly = np.linalg.norm(p_truck_2d - p_drone_2d) / v_drone_max
    for _ in range(n_iter):
        p_int_2d = p_truck_2d + v_truck_2d * dt_fly
        # Distance in 3-D: horizontal to intercept + drone is at altitude Z0
        dist_3d  = np.sqrt(np.linalg.norm(p_int_2d - p_drone_2d)**2 + p_drone[2]**2)
        dt_fly   = dist_3d / v_drone_max

    # Return full 3-D intercept waypoint at Z_SWITCH altitude (not ground)
    p_intercept_3d = np.array([p_int_2d[0], p_int_2d[1], Z_SWITCH])
    return p_intercept_3d


def png_acceleration(p_drone, v_drone, p_target, v_target, N, a_max):
    """3-D Proportional Navigation guidance law."""
    r = p_target - p_drone
    r_norm = np.linalg.norm(r) + 1e-9
    los = r / r_norm

    v_rel = v_target - v_drone
    vc = -np.dot(v_rel, los)                        # closing speed (positive = closing)
    v_perp = v_rel - np.dot(v_rel, los) * los        # velocity perpendicular to LOS
    los_rate = v_perp / (r_norm + 1e-9)              # approximate LOS rate

    a = N * vc * los_rate
    a_norm = np.linalg.norm(a)
    if a_norm > a_max:
        a = a * (a_max / a_norm)
    return a


def phase1_acceleration(p_drone, v_drone, p_intercept, v_truck, a_max):
    """Phase 1: navigate to intercept waypoint (lateral + altitude), hold altitude near Z0."""
    r    = p_intercept - p_drone
    dist = np.linalg.norm(r) + 1e-9

    # Simple proportional pursuit toward intercept waypoint
    desired_v = (r / dist) * DRONE_VMAX
    a = 3.0 * (desired_v - v_drone)          # proportional velocity tracking

    # Altitude hold: if drone is above intercept Z, brake z descent
    if p_drone[2] > Z_SWITCH + 1.0:
        # Keep altitude until we're close laterally
        lat_err = np.linalg.norm(r[:2])
        if lat_err > R_SWITCH * 3:
            a[2] = 2.0 * (Z0 - p_drone[2]) - 1.0 * v_drone[2]  # hold altitude

    a_norm = np.linalg.norm(a)
    if a_norm > a_max:
        a = a * (a_max / a_norm)
    return a


def descent_acceleration(p_drone, v_drone, p_truck, v_truck):
    """Phase 2: lateral PD + tanh flare descent."""
    # Lateral: position + velocity match
    dp_xy = p_truck[:2] - p_drone[:2]
    dv_xy = v_truck[:2] - v_drone[:2]
    a_xy  = KP * dp_xy + KV * dv_xy

    # Vertical: tanh flare
    z = p_drone[2]
    z_dot_cmd = -V_LAND_MAX * np.tanh(z / Z_FLARE)
    a_z = KZ_RATE * (z_dot_cmd - v_drone[2])

    return np.array([a_xy[0], a_xy[1], a_z])


# ── Core simulation ───────────────────────────────────────────────────────────

def run_simulation():
    """Run the moving-landing-pad simulation and return history dict."""
    # Initial conditions
    p_drone = np.array([-20.0, 8.0, Z0], dtype=float)
    v_drone = np.zeros(3)
    v_truck = np.array([TRUCK_SPEED, 0.0, 0.0])

    history = {
        "time":       [],
        "drone":      [],
        "truck":      [],
        "phase":      [],
        "lat_miss":   [],
        "vz":         [],
        "dv_horiz":   [],
    }
    landed = False
    touchdown_info = None

    for step in range(N_STEPS):
        t = step * DT

        # Truck position (straight road along x, y=z=0)
        p_truck = np.array([TRUCK_SPEED * t, 0.0, 0.0])

        # Phase logic
        lat_dist = np.linalg.norm(p_drone[:2] - p_truck[:2])
        phase = 2 if (p_drone[2] <= Z_SWITCH and lat_dist <= R_SWITCH) else 1

        # Control
        if phase == 1:
            p_intercept = predict_intercept(p_drone, p_truck, v_truck, DRONE_VMAX)
            a_cmd = phase1_acceleration(p_drone, v_drone, p_intercept, v_truck, DRONE_AMAX)
        else:
            a_cmd = descent_acceleration(p_drone, v_drone, p_truck, v_truck)
            a_norm = np.linalg.norm(a_cmd)
            if a_norm > DRONE_AMAX:
                a_cmd = a_cmd * (DRONE_AMAX / a_norm)

        # Integrate kinematics
        v_drone = v_drone + a_cmd * DT
        speed = np.linalg.norm(v_drone)
        if speed > DRONE_VMAX:
            v_drone = v_drone * (DRONE_VMAX / speed)
        p_drone = p_drone + v_drone * DT
        p_drone[2] = max(p_drone[2], 0.0)          # ground clamp

        # Record
        history["time"].append(t)
        history["drone"].append(p_drone.copy())
        history["truck"].append(p_truck.copy())
        history["phase"].append(phase)
        history["lat_miss"].append(np.linalg.norm(p_drone[:2] - p_truck[:2]))
        history["vz"].append(abs(v_drone[2]))
        history["dv_horiz"].append(np.linalg.norm(v_drone[:2] - v_truck[:2]))

        # Touchdown check
        if p_drone[2] <= Z_TOUCHDOWN:
            miss = np.linalg.norm(p_drone[:2] - p_truck[:2])
            vz   = abs(v_drone[2])
            dv   = np.linalg.norm(v_drone[:2] - v_truck[:2])
            success = (miss <= R_PAD) and (vz <= V_LAND_MAX) and (dv <= DV_MAX)
            touchdown_info = dict(t=t, miss=miss, vz=vz, dv=dv, success=success)
            landed = True
            break

    # Convert lists to arrays
    for key in ("time", "drone", "truck", "phase", "lat_miss", "vz", "dv_horiz"):
        history[key] = np.array(history[key])

    return history, landed, touchdown_info


# ── Plot 1: 3-D Trajectory ────────────────────────────────────────────────────

def plot_trajectory_3d(history, touchdown_info, out_dir):
    drone = history["drone"]
    truck = history["truck"]
    phase = history["phase"]

    fig = plt.figure(figsize=(12, 8))
    ax  = fig.add_subplot(111, projection="3d")

    # Truck road (orange dashed)
    ax.plot(truck[:, 0], truck[:, 1], truck[:, 2],
            color="orange", lw=2, linestyle="--", label="Truck road")

    # Drone Phase 1 (red) and Phase 2 (magenta)
    idx1 = np.where(phase == 1)[0]
    idx2 = np.where(phase == 2)[0]
    if len(idx1):
        ax.plot(drone[idx1, 0], drone[idx1, 1], drone[idx1, 2],
                color="red", lw=2, label="Drone – Phase 1 (PNG)")
    if len(idx2):
        ax.plot(drone[idx2, 0], drone[idx2, 1], drone[idx2, 2],
                color="darkred", lw=2, label="Drone – Phase 2 (Descent)")

    # Phase-switch point
    if len(idx2):
        sw = idx2[0]
        ax.scatter(*drone[sw], color="yellow", s=80, zorder=5, label="Phase switch")

    # Drone start / end
    ax.scatter(*drone[0], color="red", s=100, marker="^", zorder=5, label="Drone start")
    ax.scatter(*drone[-1], color="darkred", s=100, marker="v", zorder=5, label="Touchdown")

    # Truck start
    ax.scatter(*truck[0], color="orange", s=100, marker="s", zorder=5, label="Truck start")

    # Landing pad circle (green) at touchdown truck position
    pad_cx, pad_cy = truck[-1, 0], truck[-1, 1]
    theta = np.linspace(0, 2 * np.pi, 60)
    ax.plot(pad_cx + R_PAD * np.cos(theta),
            pad_cy + R_PAD * np.sin(theta),
            np.zeros(60),
            color="green", lw=2, label=f"Pad r={R_PAD}m")

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")

    title = "S023 — Moving Landing Pad: 3-D Trajectory"
    if touchdown_info:
        status = "SUCCESS" if touchdown_info["success"] else "FAIL"
        title += f"\nTouchdown t={touchdown_info['t']:.1f}s  miss={touchdown_info['miss']:.3f}m  [{status}]"
    ax.set_title(title)
    ax.legend(loc="upper left", fontsize=8)

    path = os.path.join(out_dir, "trajectory_3d.png")
    fig.savefig(path, dpi=120, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved: {path}")


# ── Plot 2: Metrics time series ───────────────────────────────────────────────

def plot_metrics(history, touchdown_info, out_dir):
    t       = history["time"]
    drone   = history["drone"]
    lat     = history["lat_miss"]
    vz      = history["vz"]
    dv      = history["dv_horiz"]
    phase   = history["phase"]

    fig, axes = plt.subplots(4, 1, figsize=(12, 12), sharex=True)
    fig.suptitle("S023 — Moving Landing Pad: Flight Metrics", fontsize=14)

    # ── Altitude ──
    ax = axes[0]
    ax.plot(t, drone[:, 2], color="steelblue", lw=2)
    ax.axhline(Z_SWITCH, color="orange", lw=1, linestyle="--", label=f"z_switch={Z_SWITCH}m")
    ax.axhline(Z_TOUCHDOWN, color="green", lw=1, linestyle=":", label=f"touchdown={Z_TOUCHDOWN}m")
    ax.set_ylabel("Altitude (m)")
    ax.legend(fontsize=8)
    ax.set_title("Drone Altitude")
    ax.grid(True, alpha=0.3)

    # ── Lateral miss distance ──
    ax = axes[1]
    ax.plot(t, lat, color="tomato", lw=2)
    ax.axhline(R_PAD, color="green", lw=1, linestyle="--", label=f"r_pad={R_PAD}m")
    ax.axhline(R_SWITCH, color="orange", lw=1, linestyle="--", label=f"r_switch={R_SWITCH}m")
    ax.set_ylabel("Lateral miss (m)")
    ax.legend(fontsize=8)
    ax.set_title("Lateral Miss Distance")
    ax.grid(True, alpha=0.3)

    # ── Vertical speed ──
    ax = axes[2]
    ax.plot(t, vz, color="purple", lw=2)
    ax.axhline(V_LAND_MAX, color="red", lw=1, linestyle="--", label=f"v_land_max={V_LAND_MAX}m/s")
    ax.set_ylabel("|vz| (m/s)")
    ax.legend(fontsize=8)
    ax.set_title("Vertical Speed (touchdown limit)")
    ax.grid(True, alpha=0.3)

    # ── Horizontal velocity mismatch ──
    ax = axes[3]
    ax.plot(t, dv, color="teal", lw=2)
    ax.axhline(DV_MAX, color="red", lw=1, linestyle="--", label=f"dv_max={DV_MAX}m/s")
    ax.set_ylabel("Δv_xy (m/s)")
    ax.set_xlabel("Time (s)")
    ax.legend(fontsize=8)
    ax.set_title("Horizontal Velocity Mismatch vs Truck")
    ax.grid(True, alpha=0.3)

    # Phase shading on all axes
    phase1_mask = phase == 1
    for ax in axes:
        ymin, ymax = ax.get_ylim()
        ax.fill_between(t, ymin, ymax, where=phase1_mask,
                        color="lightblue", alpha=0.25, label="_Phase1")
        ax.fill_between(t, ymin, ymax, where=~phase1_mask,
                        color="lightyellow", alpha=0.35, label="_Phase2")
        ax.set_ylim(ymin, ymax)

    # Phase legend patch
    p1_patch = mpatches.Patch(color="lightblue",  alpha=0.5, label="Phase 1 (PNG)")
    p2_patch = mpatches.Patch(color="lightyellow", alpha=0.7, label="Phase 2 (Descent)")
    axes[0].legend(handles=axes[0].get_legend_handles_labels()[0] + [p1_patch, p2_patch],
                   labels=axes[0].get_legend_handles_labels()[1] + ["Phase 1 (PNG)", "Phase 2 (Descent)"],
                   fontsize=7, loc="upper right")

    # Touchdown annotation
    if touchdown_info:
        status = "SUCCESS" if touchdown_info["success"] else "FAIL"
        fig.text(0.5, 0.01,
                 f"Touchdown  t={touchdown_info['t']:.2f}s  |  miss={touchdown_info['miss']:.3f}m  |  "
                 f"|vz|={touchdown_info['vz']:.3f}m/s  |  Δv={touchdown_info['dv']:.3f}m/s  |  [{status}]",
                 ha="center", fontsize=10,
                 color="green" if touchdown_info["success"] else "red",
                 bbox=dict(boxstyle="round", facecolor="white", alpha=0.8))

    plt.tight_layout(rect=[0, 0.04, 1, 1])
    path = os.path.join(out_dir, "metrics_time_series.png")
    fig.savefig(path, dpi=120, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved: {path}")


# ── Animation (GIF) ───────────────────────────────────────────────────────────

def save_animation(history, out_dir):
    drone = history["drone"]
    truck = history["truck"]
    phase = history["phase"]
    t     = history["time"]

    # Subsample to keep GIF manageable (~200 frames)
    n = len(t)
    step = max(1, n // 200)
    idx  = np.arange(0, n, step)

    fig, (ax_top, ax_side) = plt.subplots(1, 2, figsize=(14, 6))
    fig.suptitle("S023 — Moving Landing Pad Animation", fontsize=13)

    # ── Top-down view (XY plane) ──
    ax_top.set_xlim(min(drone[:, 0].min(), truck[:, 0].min()) - 5,
                    max(drone[:, 0].max(), truck[:, 0].max()) + 5)
    ax_top.set_ylim(drone[:, 1].min() - 5, drone[:, 1].max() + 5)
    ax_top.set_aspect("equal")
    ax_top.set_xlabel("X (m)")
    ax_top.set_ylabel("Y (m)")
    ax_top.set_title("Top View (XY)")
    ax_top.grid(True, alpha=0.3)

    trail_drone_top, = ax_top.plot([], [], color="red",    lw=1, alpha=0.5)
    trail_truck_top, = ax_top.plot([], [], color="orange", lw=1, alpha=0.5, linestyle="--")
    dot_drone_top,   = ax_top.plot([], [], "ro", ms=8)
    dot_truck_top,   = ax_top.plot([], [], "s",  ms=10, color="orange")
    pad_circle_top   = plt.Circle((0, 0), R_PAD, color="green", fill=False, lw=2)
    ax_top.add_patch(pad_circle_top)
    time_text_top    = ax_top.text(0.02, 0.95, "", transform=ax_top.transAxes, fontsize=9)
    phase_text_top   = ax_top.text(0.02, 0.88, "", transform=ax_top.transAxes, fontsize=9)

    # ── Side view (XZ plane) ──
    ax_side.set_xlim(min(drone[:, 0].min(), truck[:, 0].min()) - 5,
                     max(drone[:, 0].max(), truck[:, 0].max()) + 5)
    ax_side.set_ylim(-1, Z0 + 3)
    ax_side.set_xlabel("X (m)")
    ax_side.set_ylabel("Z (m)")
    ax_side.set_title("Side View (XZ)")
    ax_side.grid(True, alpha=0.3)
    ax_side.axhline(0, color="brown", lw=1, linestyle=":")    # ground
    ax_side.axhline(Z_SWITCH, color="orange", lw=1, linestyle="--", alpha=0.6)

    trail_drone_side, = ax_side.plot([], [], color="red",    lw=1, alpha=0.5)
    trail_truck_side, = ax_side.plot([], [], color="orange", lw=1, alpha=0.5, linestyle="--")
    dot_drone_side,   = ax_side.plot([], [], "ro", ms=8)
    dot_truck_side,   = ax_side.plot([], [], "s",  ms=10, color="orange")

    def init():
        for artist in (trail_drone_top, trail_truck_top, dot_drone_top, dot_truck_top,
                       trail_drone_side, trail_truck_side, dot_drone_side, dot_truck_side):
            artist.set_data([], [])
        time_text_top.set_text("")
        phase_text_top.set_text("")
        pad_circle_top.center = (0, 0)
        return (trail_drone_top, trail_truck_top, dot_drone_top, dot_truck_top,
                trail_drone_side, trail_truck_side, dot_drone_side, dot_truck_side,
                time_text_top, phase_text_top, pad_circle_top)

    def update(frame_i):
        i   = idx[frame_i]
        d   = drone[:i+1]
        tr  = truck[:i+1]
        ph  = phase[i]

        # Top view
        trail_drone_top.set_data(d[:, 0],  d[:, 1])
        trail_truck_top.set_data(tr[:, 0], tr[:, 1])
        dot_drone_top.set_data([d[-1, 0]], [d[-1, 1]])
        dot_truck_top.set_data([tr[-1, 0]], [tr[-1, 1]])
        pad_circle_top.center = (tr[-1, 0], tr[-1, 1])
        time_text_top.set_text(f"t = {t[i]:.1f}s")
        phase_text_top.set_text(f"Phase {ph}")

        # Side view
        trail_drone_side.set_data(d[:, 0],  d[:, 2])
        trail_truck_side.set_data(tr[:, 0], tr[:, 2])
        dot_drone_side.set_data([d[-1, 0]], [d[-1, 2]])
        dot_truck_side.set_data([tr[-1, 0]], [tr[-1, 2]])

        return (trail_drone_top, trail_truck_top, dot_drone_top, dot_truck_top,
                trail_drone_side, trail_truck_side, dot_drone_side, dot_truck_side,
                time_text_top, phase_text_top, pad_circle_top)

    anim = FuncAnimation(fig, update, frames=len(idx),
                         init_func=init, interval=50, blit=True)

    path = os.path.join(out_dir, "animation.gif")
    writer = PillowWriter(fps=20)
    anim.save(path, writer=writer)
    plt.close(fig)
    print(f"Saved: {path}")


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    print("=" * 60)
    print("S023 Moving Landing Pad — Simulation")
    print("=" * 60)

    history, landed, touchdown_info = run_simulation()

    if touchdown_info:
        td = touchdown_info
        status = "SUCCESS" if td["success"] else "FAIL"
        print(f"\nTouchdown at t={td['t']:.2f}s")
        print(f"  Lateral miss : {td['miss']:.4f} m  (limit {R_PAD} m)")
        print(f"  |vz|         : {td['vz']:.4f} m/s (limit {V_LAND_MAX} m/s)")
        print(f"  Δv_horiz     : {td['dv']:.4f} m/s (limit {DV_MAX} m/s)")
        print(f"  Result       : {status}")
    else:
        print("\nNo touchdown within simulation time!")

    print("\nGenerating plots...")
    plot_trajectory_3d(history, touchdown_info, OUT_DIR)
    plot_metrics(history, touchdown_info, OUT_DIR)
    print("Generating animation...")
    save_animation(history, OUT_DIR)

    print("\nAll outputs saved to:", OUT_DIR)
    print("Done.")


if __name__ == "__main__":
    main()
