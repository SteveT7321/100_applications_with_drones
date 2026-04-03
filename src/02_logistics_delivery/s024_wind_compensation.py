"""
S024 Wind Compensation
======================
Delivery drone flying from origin to a 20 m rooftop waypoint under a persistent
crosswind. Three guidance strategies are compared:
  1. No compensation  — point directly at goal, ignore wind
  2. Crab-angle FF    — analytically rotate heading to null mean drift
  3. PID cross-track  — closed-loop lateral error feedback

Outputs (saved to outputs/02_logistics_delivery/s024_wind_compensation/):
  - trajectory_3d.png
  - crosstrack_and_speed.png
  - animation.gif
"""

import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401
from matplotlib.animation import FuncAnimation, PillowWriter
import matplotlib.patches as mpatches

# ── Output directory ─────────────────────────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "..", ".."))
OUT_DIR = os.path.join(PROJECT_ROOT, "outputs", "02_logistics_delivery",
                       "s024_wind_compensation")
os.makedirs(OUT_DIR, exist_ok=True)

# ── Key constants (from scenario card) ───────────────────────────────────────
V_MAX      = 5.0      # m/s, maximum airspeed
W_MEAN     = 2.5      # m/s, mean crosswind magnitude (+y)
SIGMA_GUST = 0.8      # m/s, gust standard deviation
TAU_GUST   = 1.5      # s,   gust correlation time constant
DT         = 1 / 48   # s,   simulation timestep (48 Hz)
Z_REF      = 2.5      # m,   cruise altitude
R_LAND     = 0.3      # m,   arrival radius
P0         = np.array([0.0, 0.0, Z_REF])
P_GOAL     = np.array([20.0, 0.0, Z_REF])   # 20 m straight ahead (+x)
T_MAX      = 60.0     # s,   safety timeout

STRATEGIES = ["none", "crab", "pid"]
LABELS     = ["No Compensation", "Crab-Angle FF", "PID Cross-Track"]
COLORS     = ["#888888", "#E87722", "#1F77B4"]


# ── Wind model ────────────────────────────────────────────────────────────────
def wind_step(w_gust, dt, tau_g, sigma_g, w_mean_vec):
    """First-order Gauss-Markov gust + mean wind."""
    alpha     = np.exp(-dt / tau_g)
    noise     = np.sqrt(1.0 - alpha ** 2) * sigma_g * np.random.randn(2)
    w_gust_new = alpha * w_gust + noise
    return w_gust_new, w_mean_vec + w_gust_new


# ── Crab-angle feed-forward ───────────────────────────────────────────────────
def crab_heading(v_max, w_mean_y, bearing):
    """Compute analytically corrected heading to cancel mean crosswind."""
    sin_alpha = np.clip(-w_mean_y / v_max, -1.0, 1.0)
    return bearing + np.arcsin(sin_alpha)


# ── PID cross-track controller ────────────────────────────────────────────────
class CrossTrackPID:
    def __init__(self, kp=1.2, ki=0.05, kd=0.3, dt=DT,
                 integral_limit=10.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.dt = dt
        self.integral_limit = integral_limit
        self.integral  = 0.0
        self.prev_err  = 0.0

    def step(self, e_ct):
        self.integral += e_ct * self.dt
        # Anti-windup clamp
        self.integral = np.clip(self.integral,
                                -self.integral_limit, self.integral_limit)
        deriv          = (e_ct - self.prev_err) / self.dt
        self.prev_err  = e_ct
        return self.kp * e_ct + self.ki * self.integral + self.kd * deriv


def cross_track_error(pos, p0, d_hat):
    """Signed perpendicular distance to nominal straight-line path (2D)."""
    rel = pos[:2] - p0[:2]
    return rel[0] * d_hat[1] - rel[1] * d_hat[0]


# ── Single-strategy simulation ────────────────────────────────────────────────
def run_strategy(strategy, seed=42):
    np.random.seed(seed)
    pos    = P0.copy().astype(float)
    w_gust = np.zeros(2)
    pid    = CrossTrackPID()

    d_hat0 = (P_GOAL - P0)[:2]
    d_hat0 = d_hat0 / np.linalg.norm(d_hat0)
    n_hat0 = np.array([-d_hat0[1], d_hat0[0]])   # left-of-track normal

    w_mean_vec = np.array([0.0, W_MEAN])

    traj       = [pos.copy()]
    wind_log   = []
    ect_log    = []
    speed_log  = []
    t          = 0.0

    while np.linalg.norm(pos[:2] - P_GOAL[:2]) > R_LAND and t < T_MAX:
        w_gust, w_total = wind_step(w_gust, DT, TAU_GUST, SIGMA_GUST, w_mean_vec)
        wind_log.append(w_total.copy())

        d    = P_GOAL[:2] - pos[:2]
        dist = np.linalg.norm(d)
        bearing = np.arctan2(d[1], d[0])

        if strategy == "none":
            heading  = bearing
            v_air_2d = V_MAX * np.array([np.cos(heading), np.sin(heading)])

        elif strategy == "crab":
            heading  = crab_heading(V_MAX, W_MEAN, bearing)
            v_air_2d = V_MAX * np.array([np.cos(heading), np.sin(heading)])

        else:  # PID
            # Cross-track error: signed distance from nominal straight line
            e_ct    = cross_track_error(pos, P0, d_hat0)
            v_perp  = pid.step(e_ct)
            # Use current bearing to goal as along-track direction (scenario card eq.)
            d_hat_now = np.array([np.cos(bearing), np.sin(bearing)])
            v_air_2d  = V_MAX * d_hat_now + v_perp * n_hat0
            norm = np.linalg.norm(v_air_2d)
            if norm > V_MAX:
                v_air_2d *= V_MAX / norm

        v_ground = v_air_2d + w_total
        pos[:2] += v_ground * DT
        pos[2]  += 2.0 * (Z_REF - pos[2]) * DT        # altitude hold kz=2.0

        e_ct_now = cross_track_error(pos, P0, d_hat0)
        ect_log.append(e_ct_now)
        speed_log.append(np.linalg.norm(v_ground))

        traj.append(pos.copy())
        t += DT

    return {
        "traj"   : np.array(traj),
        "wind"   : np.array(wind_log),
        "ect"    : np.array(ect_log),
        "speed"  : np.array(speed_log),
        "time"   : t,
    }


# ── Run all three strategies ──────────────────────────────────────────────────
def run_simulation(seed=42):
    results = {}
    for strat in STRATEGIES:
        results[strat] = run_strategy(strat, seed=seed)
        traj  = results[strat]["traj"]
        ect   = results[strat]["ect"]
        label = LABELS[STRATEGIES.index(strat)]
        final_miss = np.linalg.norm(traj[-1, :2] - P_GOAL[:2])
        rms_ect    = np.sqrt(np.mean(ect ** 2)) if len(ect) else 0.0
        print(f"[{label:22s}]  steps={len(traj)-1:4d}  "
              f"flight_time={results[strat]['time']:5.2f}s  "
              f"final_miss={final_miss:.3f}m  rms_ect={rms_ect:.3f}m")
    return results


# ── Plot 1 — 3D trajectories ──────────────────────────────────────────────────
def plot_trajectories_3d(results):
    fig = plt.figure(figsize=(12, 7))
    ax  = fig.add_subplot(111, projection="3d")

    for strat, col, label in zip(STRATEGIES, COLORS, LABELS):
        traj = results[strat]["traj"]
        ax.plot(traj[:, 0], traj[:, 1], traj[:, 2],
                color=col, lw=1.8, label=label, alpha=0.85)
        ax.scatter(*traj[-1], color=col, s=60, zorder=5)

    # Nominal straight path
    ax.plot([P0[0], P_GOAL[0]], [P0[1], P_GOAL[1]], [P0[2], P_GOAL[2]],
            "k--", lw=1.0, label="Nominal path", alpha=0.5)

    # Start / goal markers
    ax.scatter(*P0,    color="green", s=100, marker="^", zorder=6, label="Start")
    ax.scatter(*P_GOAL, color="red",  s=120, marker="*", zorder=6, label="Goal")

    # Wind arrows on z=0 plane (decorative)
    for xi in np.linspace(2, 18, 5):
        ax.quiver(xi, -0.5, 0.0, 0.0, 1.4, 0.0,
                  color="#cccccc", arrow_length_ratio=0.3, linewidth=0.8)

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)  ← crosswind")
    ax.set_zlabel("Z (m)")
    ax.set_title("S024 Wind Compensation — 3D Trajectories", fontsize=13)
    ax.legend(loc="upper left", fontsize=8)
    ax.view_init(elev=25, azim=-50)

    out = os.path.join(OUT_DIR, "trajectory_3d.png")
    fig.savefig(out, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved: {out}")


# ── Plot 2 — cross-track error + ground speed ─────────────────────────────────
def plot_crosstrack_and_speed(results):
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=False)

    # ─ Cross-track error ─
    ax = axes[0]
    for strat, col, label in zip(STRATEGIES, COLORS, LABELS):
        ect  = results[strat]["ect"]
        time = np.arange(len(ect)) * DT
        ax.plot(time, ect, color=col, lw=1.5, label=label, alpha=0.85)

    ax.axhline(0, color="k", lw=0.8, ls="--")
    ax.set_ylabel("Cross-Track Error (m)")
    ax.set_xlabel("Time (s)")
    ax.set_title("Cross-Track Error vs Time")
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)

    # ─ Ground speed ─
    ax = axes[1]
    for strat, col, label in zip(STRATEGIES, COLORS, LABELS):
        spd  = results[strat]["speed"]
        time = np.arange(len(spd)) * DT
        ax.plot(time, spd, color=col, lw=1.5, label=label, alpha=0.85)

    ax.axhline(V_MAX, color="k", lw=0.8, ls="--", label=f"v_max = {V_MAX} m/s")
    ax.set_ylabel("Ground Speed (m/s)")
    ax.set_xlabel("Time (s)")
    ax.set_title("Ground Speed vs Time")
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)

    fig.suptitle("S024 Wind Compensation — Performance Metrics", fontsize=13, y=1.01)
    fig.tight_layout()

    out = os.path.join(OUT_DIR, "crosstrack_and_speed.png")
    fig.savefig(out, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved: {out}")


# ── Plot 3 — lateral displacement bar chart ───────────────────────────────────
def plot_lateral_bar(results):
    fig, ax = plt.subplots(figsize=(7, 5))
    rms_vals  = []
    miss_vals = []
    for strat in STRATEGIES:
        ect  = results[strat]["ect"]
        traj = results[strat]["traj"]
        rms_vals.append(np.sqrt(np.mean(ect ** 2)) if len(ect) else 0.0)
        miss_vals.append(np.linalg.norm(traj[-1, :2] - P_GOAL[:2]))

    x   = np.arange(len(STRATEGIES))
    w   = 0.35
    b1  = ax.bar(x - w/2, rms_vals,  w, label="RMS cross-track (m)", color=COLORS, alpha=0.75)
    b2  = ax.bar(x + w/2, miss_vals, w, label="Final miss dist (m)",  color=COLORS, alpha=0.45,
                 edgecolor="black", linewidth=0.8)

    ax.set_xticks(x)
    ax.set_xticklabels(LABELS, fontsize=9)
    ax.set_ylabel("Distance (m)")
    ax.set_title("S024 Wind Compensation — Lateral Performance Summary")
    ax.legend(fontsize=9)
    ax.grid(axis="y", alpha=0.3)

    # Annotate bars
    for bar in b1:
        ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.02,
                f"{bar.get_height():.3f}", ha="center", va="bottom", fontsize=8)
    for bar in b2:
        ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.02,
                f"{bar.get_height():.3f}", ha="center", va="bottom", fontsize=8)

    fig.tight_layout()
    out = os.path.join(OUT_DIR, "lateral_bar.png")
    fig.savefig(out, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved: {out}")


# ── Animation — top-down XY view ──────────────────────────────────────────────
def save_animation(results):
    fig, ax = plt.subplots(figsize=(10, 6))
    ax.set_xlim(-1, 22)

    # Y limits: accommodate worst drift (no-compensation)
    traj_none = results["none"]["traj"]
    y_lo = min(traj_none[:, 1].min() - 0.5, -1.0)
    y_hi = max(traj_none[:, 1].max() + 0.5,  1.5)
    ax.set_ylim(y_lo, y_hi)

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)  ← crosswind drift")
    ax.set_title("S024 Wind Compensation — Top-Down View (XY)")

    # Nominal path
    ax.plot([P0[0], P_GOAL[0]], [P0[1], P_GOAL[1]], "k--", lw=0.8, alpha=0.5)

    # Wind arrows (static background)
    for xi in np.linspace(1, 19, 7):
        ax.annotate("", xy=(xi, 1.2), xytext=(xi, 0.0),
                    arrowprops=dict(arrowstyle="->", color="#cccccc", lw=1.2))

    # Start / goal
    ax.scatter(*P0[:2],    color="green", s=100, marker="^", zorder=5)
    ax.scatter(*P_GOAL[:2], color="red",  s=120, marker="*", zorder=5)

    # Trajectory lines and drone markers (animated)
    lines  = []
    dots   = []
    trajs  = [results[s]["traj"] for s in STRATEGIES]
    n_max  = max(len(t) for t in trajs)

    for col, label in zip(COLORS, LABELS):
        ln, = ax.plot([], [], color=col, lw=1.6, label=label)
        dt, = ax.plot([], [], "o", color=col, ms=7)
        lines.append(ln)
        dots.append(dt)

    # Wind text
    wind_txt = ax.text(0.02, 0.95, "", transform=ax.transAxes, fontsize=8,
                       verticalalignment="top")
    ax.legend(loc="lower right", fontsize=8)

    # Skip frames for a 5 s GIF at ~20 fps = 100 frames
    skip   = max(1, n_max // 100)
    frames = list(range(0, n_max, skip)) + [n_max - 1]

    def init():
        for ln, dt in zip(lines, dots):
            ln.set_data([], [])
            dt.set_data([], [])
        wind_txt.set_text("")
        return lines + dots + [wind_txt]

    def animate(frame_idx):
        fi = frames[frame_idx]
        for i, (ln, dt, traj) in enumerate(zip(lines, dots, trajs)):
            k = min(fi, len(traj) - 1)
            ln.set_data(traj[:k+1, 0], traj[:k+1, 1])
            dt.set_data([traj[k, 0]], [traj[k, 1]])

        # Display current wind from "none" strategy wind log
        wind_data = results["none"]["wind"]
        wi = min(fi, len(wind_data) - 1) if len(wind_data) else 0
        if len(wind_data):
            wv = wind_data[wi]
            wind_txt.set_text(f"Wind  wx={wv[0]:.2f}  wy={wv[1]:.2f} m/s")
        return lines + dots + [wind_txt]

    anim = FuncAnimation(fig, animate, init_func=init,
                         frames=len(frames), interval=50, blit=True)

    out = os.path.join(OUT_DIR, "animation.gif")
    anim.save(out, writer=PillowWriter(fps=20))
    plt.close(fig)
    print(f"Saved: {out}")


# ── Main ──────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    print("=" * 60)
    print("S024 Wind Compensation — Simulation")
    print("=" * 60)

    results = run_simulation(seed=42)

    print("\nGenerating plots ...")
    plot_trajectories_3d(results)
    plot_crosstrack_and_speed(results)
    plot_lateral_bar(results)

    print("\nGenerating animation ...")
    save_animation(results)

    # Print summary statistics
    print("\n── Summary ─────────────────────────────────────────────")
    for strat, label in zip(STRATEGIES, LABELS):
        traj = results[strat]["traj"]
        ect  = results[strat]["ect"]
        miss = np.linalg.norm(traj[-1, :2] - P_GOAL[:2])
        rms  = np.sqrt(np.mean(ect ** 2)) if len(ect) else 0.0
        t    = results[strat]["time"]
        print(f"  {label:22s}  time={t:5.2f}s  miss={miss:.3f}m  rms_ect={rms:.4f}m")

    print("\nAll outputs saved to:", OUT_DIR)
    print("Done.")
