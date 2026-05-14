"""
S024 3D Wind Compensation
=========================
Extends the 2D wind compensation scenario into full 3D, where the crosswind field
has both horizontal (y) and vertical (z-turbulence) components. A delivery drone
must fly 40 m from origin to a rooftop waypoint at altitude 15 m under a persistent
crosswind. Three guidance strategies are compared in 3D:

  1. No Compensation  — fly the nominal bearing in 3D, ignore wind
  2. Crab-Angle FF    — analytically rotate the horizontal heading to null mean
                        horizontal crosswind drift; vertical altitude hold
  3. PID Cross-Track  — closed-loop 3D error feedback (lateral + vertical)

Outputs (saved to outputs/02_logistics_delivery/s024_3d_wind_compensation/):
  - trajectory_3d.png         — 3D trajectory comparison
  - crosstrack_error.png      — lateral and vertical cross-track error vs time
  - lateral_bar.png           — bar chart of RMS errors and final miss distances
  - animation.gif             — animated top-down and 3D view

Usage:
    conda activate drones
    python src/02_logistics_delivery/3d/s024_3d_wind_compensation.py
"""

import matplotlib
matplotlib.use("Agg")

import os
import sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401
from matplotlib.animation import FuncAnimation, PillowWriter

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..'))

# ── Output directory ──────────────────────────────────────────────────────────
SCRIPT_DIR   = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, '..', '..', '..'))
OUT_DIR      = os.path.join(PROJECT_ROOT, 'outputs', '02_logistics_delivery',
                             's024_3d_wind_compensation')
os.makedirs(OUT_DIR, exist_ok=True)

# ── Key constants ─────────────────────────────────────────────────────────────
V_MAX        = 5.0      # m/s, maximum airspeed
W_MEAN_Y     = 2.5      # m/s, mean crosswind in +y direction
W_MEAN_Z     = 0.4      # m/s, mean updraft / downdraft in +z direction
SIGMA_GUST   = 0.8      # m/s, gust standard deviation (horizontal)
SIGMA_GUST_Z = 0.3      # m/s, gust standard deviation (vertical)
TAU_GUST     = 1.5      # s,   gust correlation time constant
DT           = 1 / 48   # s,   48 Hz control rate
R_LAND       = 0.3      # m,   arrival radius
T_MAX        = 90.0     # s,   safety timeout

# Start at origin; goal is a rooftop 40 m ahead and 15 m up
P0     = np.array([0.0,  0.0,  2.5])   # take-off altitude 2.5 m
P_GOAL = np.array([40.0, 0.0, 15.0])   # rooftop waypoint

# PID gains
KP_LAT  = 1.2   # lateral cross-track
KI_LAT  = 0.05
KD_LAT  = 0.3
KP_VERT = 1.5   # vertical track
KI_VERT = 0.04
KD_VERT = 0.2

STRATEGIES = ["none", "crab", "pid"]
LABELS     = ["No Compensation", "Crab-Angle FF", "3D PID"]
COLORS     = ["#888888", "#E87722", "#1F77B4"]

RNG = np.random.default_rng(0)


# ── Wind model ────────────────────────────────────────────────────────────────
def wind_step(w_gust, dt, tau_g, sigma_h, sigma_v):
    """
    First-order Gauss-Markov gust for horizontal (x,y) and vertical (z) components.
    Returns (w_gust_new [3], w_total [3]).
    """
    alpha = np.exp(-dt / tau_g)
    sq    = np.sqrt(1.0 - alpha ** 2)

    noise_h = sq * sigma_h * RNG.standard_normal(2)
    noise_v = sq * sigma_v * RNG.standard_normal(1)

    w_gust_new = np.zeros(3)
    w_gust_new[:2] = alpha * w_gust[:2] + noise_h
    w_gust_new[2]  = alpha * w_gust[2]  + noise_v[0]

    w_total = np.array([0.0, W_MEAN_Y, W_MEAN_Z]) + w_gust_new
    return w_gust_new, w_total


# ── Crab-angle feed-forward (horizontal only) ─────────────────────────────────
def crab_heading_3d(bearing, v_max, w_mean_y):
    """Compute analytically corrected horizontal heading to cancel mean y-crosswind."""
    sin_alpha = np.clip(-w_mean_y / v_max, -1.0, 1.0)
    return bearing + np.arcsin(sin_alpha)


# ── PID controller (generic SISO) ─────────────────────────────────────────────
class PID:
    def __init__(self, kp, ki, kd, dt, integral_limit=10.0):
        self.kp = kp;  self.ki = ki;  self.kd = kd
        self.dt = dt
        self.integral_limit = integral_limit
        self.integral  = 0.0
        self.prev_err  = 0.0

    def step(self, err):
        self.integral = np.clip(
            self.integral + err * self.dt,
            -self.integral_limit, self.integral_limit
        )
        deriv         = (err - self.prev_err) / self.dt
        self.prev_err = err
        return self.kp * err + self.ki * self.integral + self.kd * deriv

    def reset(self):
        self.integral = 0.0
        self.prev_err = 0.0


# ── Signed 3D cross-track errors ─────────────────────────────────────────────
def cross_track_errors_3d(pos, p0, d_hat):
    """
    Returns (e_lateral, e_vertical):
      e_lateral = signed distance off the horizontal plane through the nominal path
      e_vertical = signed altitude error relative to the nominal straight-line height
    """
    rel      = pos - p0                           # vector from start to current
    along    = np.dot(rel, d_hat)                 # projection along nominal path
    perp_vec = rel - along * d_hat                # perpendicular component (3D)

    # Lateral: y-component of perpendicular
    # Vertical: z-component of perpendicular
    e_lat  = perp_vec[1]    # positive = drifted in +y (downwind)
    e_vert = perp_vec[2]    # positive = above nominal path
    return e_lat, e_vert


# ── Single-strategy simulation ────────────────────────────────────────────────
def run_strategy(strategy, seed=0):
    """Simulate one guidance strategy. Returns dict with traj, ect, etc."""
    rng_local = np.random.default_rng(seed)
    # Reset global RNG to same seed for fair comparison
    global RNG
    RNG = np.random.default_rng(seed)

    pos    = P0.copy().astype(float)
    w_gust = np.zeros(3)

    pid_lat  = PID(KP_LAT,  KI_LAT,  KD_LAT,  DT)
    pid_vert = PID(KP_VERT, KI_VERT, KD_VERT, DT)

    # Nominal path direction unit vector (3D)
    d_hat0 = P_GOAL - P0
    d_hat0 = d_hat0 / np.linalg.norm(d_hat0)

    # Horizontal nominal direction
    d_horiz = (P_GOAL - P0)[:2]
    d_horiz = d_horiz / np.linalg.norm(d_horiz)

    # Left-of-track normal (horizontal plane)
    n_lat = np.array([-d_horiz[1], d_horiz[0], 0.0])
    # Up-of-track normal (perpendicular to d_hat0, in the vertical plane)
    n_vert = np.array([0.0, 0.0, 1.0])

    traj      = [pos.copy()]
    wind_log  = []
    ect_lat   = []
    ect_vert  = []
    speed_log = []
    t         = 0.0

    while np.linalg.norm(pos - P_GOAL) > R_LAND and t < T_MAX:
        w_gust, w_total = wind_step(w_gust, DT, TAU_GUST, SIGMA_GUST, SIGMA_GUST_Z)
        wind_log.append(w_total.copy())

        d    = P_GOAL - pos
        dist = np.linalg.norm(d)
        d_hat_now = d / dist if dist > 1e-6 else d_hat0

        # Horizontal bearing for crab angle
        d_horiz_now  = d[:2]
        dh_norm      = np.linalg.norm(d_horiz_now)
        bearing      = np.arctan2(d_horiz_now[1], d_horiz_now[0]) if dh_norm > 1e-6 else 0.0

        # Nominal elevation angle
        elev = np.arctan2(d[2], dh_norm) if dh_norm > 1e-6 else 0.0

        if strategy == "none":
            # Point directly at goal in 3D, ignore wind
            v_air = V_MAX * d_hat_now

        elif strategy == "crab":
            # Correct horizontal heading; let altitude hold handle Z
            corrected_bearing = crab_heading_3d(bearing, V_MAX, W_MEAN_Y)
            # Also counter mean vertical wind
            # Vertical correction: tilt the airspeed vector to oppose W_MEAN_Z
            sin_beta = np.clip(-W_MEAN_Z / V_MAX, -1.0, 1.0)
            beta     = np.arcsin(sin_beta)    # small pitch-down adjustment

            # Build 3D airspeed: corrected horizontal direction + pitch correction
            v_horiz_mag = V_MAX * np.cos(elev + beta)
            v_air = np.array([
                v_horiz_mag * np.cos(corrected_bearing),
                v_horiz_mag * np.sin(corrected_bearing),
                V_MAX       * np.sin(elev + beta),
            ])
            # Rescale to V_MAX
            v_norm = np.linalg.norm(v_air)
            if v_norm > 1e-6:
                v_air = v_air * (V_MAX / v_norm)

        else:  # PID 3D
            # Compute 3D cross-track errors
            e_lat, e_vert = cross_track_errors_3d(pos, P0, d_hat0)

            # PID outputs: how much lateral / vertical velocity to add to push
            # drone back toward the nominal path.
            # e_lat > 0  means drone is in +y side  → need -y correction → subtract n_lat
            # e_vert > 0 means drone is above path  → need -z correction → subtract n_vert
            v_lat_corr  = pid_lat.step(e_lat)    # positive → push in -n_lat direction
            v_vert_corr = pid_vert.step(e_vert)  # positive → push in -n_vert direction

            # Base: aim at goal in 3D
            v_air = V_MAX * d_hat_now \
                  - v_lat_corr  * n_lat  \
                  - v_vert_corr * n_vert

            v_norm = np.linalg.norm(v_air)
            if v_norm > V_MAX:
                v_air = v_air * (V_MAX / v_norm)

        v_ground = v_air + w_total
        pos     += v_ground * DT

        e_lat_now, e_vert_now = cross_track_errors_3d(pos, P0, d_hat0)
        ect_lat.append(e_lat_now)
        ect_vert.append(e_vert_now)
        speed_log.append(np.linalg.norm(v_ground))
        traj.append(pos.copy())
        t += DT

    return {
        "traj"    : np.array(traj),
        "wind"    : np.array(wind_log),
        "ect_lat" : np.array(ect_lat),
        "ect_vert": np.array(ect_vert),
        "speed"   : np.array(speed_log),
        "time"    : t,
    }


# ── Run all three strategies ──────────────────────────────────────────────────
def run_simulation(seed=0):
    results = {}
    for strat, label in zip(STRATEGIES, LABELS):
        results[strat] = run_strategy(strat, seed=seed)
        r     = results[strat]
        miss  = np.linalg.norm(r["traj"][-1] - P_GOAL)
        rms_l = float(np.sqrt(np.mean(r["ect_lat"]  ** 2))) if len(r["ect_lat"])  else 0.0
        rms_v = float(np.sqrt(np.mean(r["ect_vert"] ** 2))) if len(r["ect_vert"]) else 0.0
        print(f"[{label:20s}]  steps={len(r['traj'])-1:4d}  "
              f"flight_time={r['time']:5.2f}s  final_miss={miss:.3f}m  "
              f"rms_lat={rms_l:.3f}m  rms_vert={rms_v:.3f}m")
    return results


# ── Plot 1 — 3D trajectories ──────────────────────────────────────────────────
def plot_trajectories_3d(results):
    fig = plt.figure(figsize=(13, 8))
    ax  = fig.add_subplot(111, projection="3d")

    for strat, col, label in zip(STRATEGIES, COLORS, LABELS):
        traj = results[strat]["traj"]
        ax.plot(traj[:, 0], traj[:, 1], traj[:, 2],
                color=col, lw=1.8, label=label, alpha=0.85)
        ax.scatter(traj[-1, 0], traj[-1, 1], traj[-1, 2],
                   color=col, s=60, zorder=5)

    # Nominal straight path (P0 -> P_GOAL)
    ax.plot([P0[0], P_GOAL[0]], [P0[1], P_GOAL[1]], [P0[2], P_GOAL[2]],
            "k--", lw=1.0, label="Nominal path", alpha=0.5)

    # Start / goal markers
    ax.scatter(*P0,    color="green", s=120, marker="^", zorder=6, label="Start")
    ax.scatter(*P_GOAL, color="red",  s=150, marker="*", zorder=6, label="Goal")

    # Wind arrows (crosswind +y) on the z=2 plane
    for xi in np.linspace(4, 36, 6):
        ax.quiver(xi, -1.0, 2.0, 0.0, 2.0, 0.0,
                  color="#bbbbbb", arrow_length_ratio=0.25, linewidth=0.9)

    # Data-driven axis limits
    all_traj = np.vstack([results[s]["traj"] for s in STRATEGIES])
    x_lo, x_hi = all_traj[:, 0].min() - 1, all_traj[:, 0].max() + 1
    y_lo, y_hi = all_traj[:, 1].min() - 1, all_traj[:, 1].max() + 1
    z_lo, z_hi = all_traj[:, 2].min() - 1, all_traj[:, 2].max() + 1
    ax.set_xlim(x_lo, x_hi)
    ax.set_ylim(y_lo, y_hi)
    ax.set_zlim(z_lo, z_hi)

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)  [crosswind]")
    ax.set_zlabel("Z (m)  [altitude]")
    ax.set_title("S024 3D Wind Compensation — Trajectory Comparison", fontsize=13)
    ax.legend(loc="upper left", fontsize=8)
    ax.view_init(elev=22, azim=-55)

    out = os.path.join(OUT_DIR, "trajectory_3d.png")
    fig.savefig(out, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved: {out}")


# ── Plot 2 — cross-track errors ───────────────────────────────────────────────
def plot_crosstrack_error(results):
    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=False)

    # ─ Lateral cross-track error ─
    ax = axes[0]
    for strat, col, label in zip(STRATEGIES, COLORS, LABELS):
        ect  = results[strat]["ect_lat"]
        t_ax = np.arange(len(ect)) * DT
        ax.plot(t_ax, ect, color=col, lw=1.5, label=label, alpha=0.85)
    ax.axhline(0, color="k", lw=0.8, ls="--")
    ax.set_ylabel("Lateral error (m)")
    ax.set_title("Lateral Cross-Track Error vs Time")
    ax.legend(fontsize=9);  ax.grid(True, alpha=0.3)

    # ─ Vertical cross-track error ─
    ax = axes[1]
    for strat, col, label in zip(STRATEGIES, COLORS, LABELS):
        ect  = results[strat]["ect_vert"]
        t_ax = np.arange(len(ect)) * DT
        ax.plot(t_ax, ect, color=col, lw=1.5, label=label, alpha=0.85)
    ax.axhline(0, color="k", lw=0.8, ls="--")
    ax.set_ylabel("Vertical error (m)")
    ax.set_title("Vertical Cross-Track Error vs Time")
    ax.legend(fontsize=9);  ax.grid(True, alpha=0.3)

    # ─ Ground speed ─
    ax = axes[2]
    for strat, col, label in zip(STRATEGIES, COLORS, LABELS):
        spd  = results[strat]["speed"]
        t_ax = np.arange(len(spd)) * DT
        ax.plot(t_ax, spd, color=col, lw=1.5, label=label, alpha=0.85)
    ax.axhline(V_MAX, color="k", lw=0.8, ls="--", label=f"v_max={V_MAX} m/s")
    ax.set_ylabel("Ground speed (m/s)");  ax.set_xlabel("Time (s)")
    ax.set_title("Ground Speed vs Time")
    ax.legend(fontsize=9);  ax.grid(True, alpha=0.3)

    fig.suptitle("S024 3D Wind Compensation — Performance Metrics", fontsize=13)
    fig.tight_layout()

    out = os.path.join(OUT_DIR, "crosstrack_error.png")
    fig.savefig(out, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved: {out}")


# ── Plot 3 — lateral / vertical bar chart ─────────────────────────────────────
def plot_lateral_bar(results):
    fig, ax = plt.subplots(figsize=(9, 5))

    rms_lat  = []
    rms_vert = []
    miss     = []
    for strat in STRATEGIES:
        r = results[strat]
        rms_lat.append(float(np.sqrt(np.mean(r["ect_lat"]  ** 2))) if len(r["ect_lat"])  else 0.0)
        rms_vert.append(float(np.sqrt(np.mean(r["ect_vert"] ** 2))) if len(r["ect_vert"]) else 0.0)
        miss.append(float(np.linalg.norm(r["traj"][-1] - P_GOAL)))

    x = np.arange(len(STRATEGIES))
    w = 0.25
    b1 = ax.bar(x - w,   rms_lat,  w, label="RMS lateral (m)",   color=COLORS, alpha=0.80)
    b2 = ax.bar(x,       rms_vert, w, label="RMS vertical (m)",  color=COLORS, alpha=0.55,
                hatch="//", edgecolor="white")
    b3 = ax.bar(x + w,   miss,     w, label="Final miss 3D (m)", color=COLORS, alpha=0.40,
                edgecolor="black", linewidth=0.8)

    ax.set_xticks(x)
    ax.set_xticklabels(LABELS, fontsize=9)
    ax.set_ylabel("Distance (m)")
    ax.set_title("S024 3D Wind Compensation — Error Summary")
    ax.legend(fontsize=9);  ax.grid(axis="y", alpha=0.3)

    for bars in (b1, b2, b3):
        for bar in bars:
            ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.02,
                    f"{bar.get_height():.3f}", ha="center", va="bottom", fontsize=7)

    fig.tight_layout()
    out = os.path.join(OUT_DIR, "lateral_bar.png")
    fig.savefig(out, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved: {out}")


# ── Animation ─────────────────────────────────────────────────────────────────
def save_animation(results):
    """Two-panel animation: top-down XY and side-view XZ."""
    fig, (ax_xy, ax_xz) = plt.subplots(1, 2, figsize=(14, 6))
    fig.suptitle("S024 3D Wind Compensation — Animation", fontsize=12)

    trajs = [results[s]["traj"] for s in STRATEGIES]
    n_max = max(len(t) for t in trajs)

    # Data-driven limits
    all_traj = np.vstack(trajs)
    x_lo = all_traj[:, 0].min() - 1;  x_hi = all_traj[:, 0].max() + 1
    y_lo = all_traj[:, 1].min() - 1;  y_hi = all_traj[:, 1].max() + 1
    z_lo = all_traj[:, 2].min() - 1;  z_hi = all_traj[:, 2].max() + 1

    for ax in (ax_xy, ax_xz):
        ax.plot([P0[0], P_GOAL[0]], [P0[1] if ax is ax_xy else P0[2],
                                      P_GOAL[1] if ax is ax_xy else P_GOAL[2]],
                "k--", lw=0.8, alpha=0.5, label="Nominal")

    ax_xy.set_xlim(x_lo, x_hi);  ax_xy.set_ylim(y_lo, y_hi)
    ax_xz.set_xlim(x_lo, x_hi);  ax_xz.set_ylim(z_lo, z_hi)
    ax_xy.set_xlabel("X (m)");  ax_xy.set_ylabel("Y (m) [crosswind]")
    ax_xz.set_xlabel("X (m)");  ax_xz.set_ylabel("Z (m) [altitude]")
    ax_xy.set_title("Top-Down View (XY)");  ax_xz.set_title("Side View (XZ)")

    # Start / goal markers
    ax_xy.scatter(*P0[:2],    color="green", s=80, marker="^", zorder=5)
    ax_xy.scatter(*P_GOAL[:2], color="red",  s=100, marker="*", zorder=5)
    ax_xz.scatter(P0[0],    P0[2],    color="green", s=80, marker="^", zorder=5)
    ax_xz.scatter(P_GOAL[0], P_GOAL[2], color="red",  s=100, marker="*", zorder=5)

    # Wind arrows (static) in XY
    for xi in np.linspace(4, 36, 6):
        ax_xy.annotate("", xy=(xi, 1.5), xytext=(xi, 0.0),
                       arrowprops=dict(arrowstyle="->", color="#cccccc", lw=1.0))

    lines_xy = [];  dots_xy = []
    lines_xz = [];  dots_xz = []
    for col, label in zip(COLORS, LABELS):
        lxy, = ax_xy.plot([], [], color=col, lw=1.5, label=label)
        dxy, = ax_xy.plot([], [], "o", color=col, ms=6)
        lxz, = ax_xz.plot([], [], color=col, lw=1.5, label=label)
        dxz, = ax_xz.plot([], [], "o", color=col, ms=6)
        lines_xy.append(lxy);  dots_xy.append(dxy)
        lines_xz.append(lxz);  dots_xz.append(dxz)

    time_txt = ax_xy.text(0.02, 0.95, "", transform=ax_xy.transAxes, fontsize=8,
                          verticalalignment="top")
    ax_xy.legend(loc="lower right", fontsize=7)
    ax_xz.legend(loc="lower right", fontsize=7)

    skip   = max(1, n_max // 120)
    frames = list(range(0, n_max, skip)) + [n_max - 1]

    def init():
        for ln, dt in zip(lines_xy, dots_xy):
            ln.set_data([], []);  dt.set_data([], [])
        for ln, dt in zip(lines_xz, dots_xz):
            ln.set_data([], []);  dt.set_data([], [])
        time_txt.set_text("")
        return lines_xy + dots_xy + lines_xz + dots_xz + [time_txt]

    def animate(frame_idx):
        fi = frames[frame_idx]
        for i, traj in enumerate(trajs):
            k = min(fi, len(traj) - 1)
            lines_xy[i].set_data(traj[:k+1, 0], traj[:k+1, 1])
            dots_xy[i].set_data([traj[k, 0]], [traj[k, 1]])
            lines_xz[i].set_data(traj[:k+1, 0], traj[:k+1, 2])
            dots_xz[i].set_data([traj[k, 0]], [traj[k, 2]])
        time_txt.set_text(f"t = {fi * DT:.1f} s")
        return lines_xy + dots_xy + lines_xz + dots_xz + [time_txt]

    anim = FuncAnimation(fig, animate, init_func=init,
                         frames=len(frames), interval=50, blit=True)

    out = os.path.join(OUT_DIR, "animation.gif")
    anim.save(out, writer=PillowWriter(fps=20), dpi=100)
    plt.close(fig)
    print(f"Saved: {out}")


# ── Main ──────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    print("=" * 65)
    print("S024 3D Wind Compensation — Simulation")
    print("=" * 65)

    results = run_simulation(seed=0)

    print("\nGenerating plots ...")
    plot_trajectories_3d(results)
    plot_crosstrack_error(results)
    plot_lateral_bar(results)

    print("\nGenerating animation ...")
    save_animation(results)

    print("\n── Summary ───────────────────────────────────────────────────────")
    for strat, label in zip(STRATEGIES, LABELS):
        r     = results[strat]
        miss  = np.linalg.norm(r["traj"][-1] - P_GOAL)
        rms_l = float(np.sqrt(np.mean(r["ect_lat"]  ** 2))) if len(r["ect_lat"])  else 0.0
        rms_v = float(np.sqrt(np.mean(r["ect_vert"] ** 2))) if len(r["ect_vert"]) else 0.0
        print(f"  {label:20s}  time={r['time']:6.2f}s  "
              f"miss={miss:.3f}m  rms_lat={rms_l:.4f}m  rms_vert={rms_v:.4f}m")

    print(f"\nAll outputs saved to: {OUT_DIR}")
    print("Done.")
