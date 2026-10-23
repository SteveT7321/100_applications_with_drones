"""
S024 3D Wind Compensation
=========================
Extends the 2D wind compensation scenario into full 3D, where the crosswind field
has both horizontal (y) and vertical (z-turbulence) components. A delivery drone
must fly 40 m from origin to a rooftop waypoint at altitude 15 m under a persistent
crosswind. Three guidance strategies are compared in 3D:

  1. No Compensation  -- aim at the goal in 3D every step (pure pursuit), ignore wind
  2. Crab-Angle FF    -- analytically rotate the horizontal heading and the pitch to
                        null the mean crosswind and the mean updraft
  3. PID Cross-Track  -- closed-loop feedback on lateral + vertical cross-track error

Strategies are compared on cross-track RMS / peak error and flight time, for a
showcase gust seed and a Monte-Carlo over N_MC gust seeds (same gusts for every
strategy).  Final distance is only an arrival check: every run stops at its first
step inside R_LAND, so it cannot rank the strategies.

Outputs (saved to outputs/02_logistics_delivery/3d/s024_3d_wind_compensation/):
  - trajectory_3d.png         -- 3D trajectory comparison (showcase seed)
  - crosstrack_error.png      -- lateral / vertical cross-track error and ground speed
  - lateral_bar.png           -- Monte-Carlo RMS / peak errors and flight time
  - animation.gif             -- animated top-down and side view

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
OUT_DIR      = os.path.join(PROJECT_ROOT, 'outputs', '02_logistics_delivery', '3d',
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
SEED         = 0        # showcase gust seed
N_MC         = 50       # Monte-Carlo gust seeds SEED .. SEED+N_MC-1

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



# ── Wind model ────────────────────────────────────────────────────────────────
def wind_step(w_gust, dt, tau_g, sigma_h, sigma_v, rng):
    """
    First-order Gauss-Markov gust for horizontal (x,y) and vertical (z) components.
    Returns (w_gust_new [3], w_total [3]).
    """
    alpha = np.exp(-dt / tau_g)
    sq    = np.sqrt(1.0 - alpha ** 2)

    noise_h = sq * sigma_h * rng.standard_normal(2)
    noise_v = sq * sigma_v * rng.standard_normal(1)

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
def run_strategy(strategy, seed=SEED):
    """Simulate one guidance strategy. Returns dict with traj, ect, etc."""
    # Same seed for every strategy -> identical gust sequence (fair comparison)
    rng = np.random.default_rng(seed)

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
        w_gust, w_total = wind_step(w_gust, DT, TAU_GUST, SIGMA_GUST, SIGMA_GUST_Z, rng)
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

    ect_lat = np.array(ect_lat)
    ect_vert = np.array(ect_vert)
    traj = np.array(traj)
    final_dist = float(np.linalg.norm(traj[-1] - P_GOAL))
    return {
        "traj"    : traj,
        "wind"    : np.array(wind_log),
        "ect_lat" : ect_lat,
        "ect_vert": ect_vert,
        "speed"   : np.array(speed_log),
        "time"    : t,
        "rms_lat" : float(np.sqrt(np.mean(ect_lat ** 2))),
        "rms_vert": float(np.sqrt(np.mean(ect_vert ** 2))),
        "max_lat" : float(np.max(np.abs(ect_lat))),
        "max_vert": float(np.max(np.abs(ect_vert))),
        "mean_lat": float(np.mean(ect_lat)),
        "path_len": float(np.sum(np.linalg.norm(np.diff(traj, axis=0), axis=1))),
        "final_dist": final_dist,
        "arrived" : final_dist <= R_LAND,
    }


MC_KEYS = ("time", "rms_lat", "rms_vert", "max_lat", "max_vert", "mean_lat",
           "path_len", "final_dist", "arrived")


# ── Run all three strategies ──────────────────────────────────────────────────
def run_simulation(seed=SEED, n_mc=N_MC):
    print(f"Showcase run (gust seed {seed}):")
    results = {}
    for strat, label in zip(STRATEGIES, LABELS):
        r = run_strategy(strat, seed=seed)
        results[strat] = r
        print(f"  [{label:16s}] time={r['time']:5.2f}s  path={r['path_len']:5.1f}m  "
              f"rms_lat={r['rms_lat']:.3f}m  max|lat|={r['max_lat']:.3f}m  "
              f"mean_lat={r['mean_lat']:+.3f}m  rms_vert={r['rms_vert']:.3f}m  "
              f"max|vert|={r['max_vert']:.3f}m  final_dist={r['final_dist']:.3f}m"
              f"{'' if r['arrived'] else '  (TIMEOUT)'}")
    w_bar = results["crab"]["wind"].mean(axis=0)
    print(f"  Mean wind over the crab-FF flight: ({w_bar[0]:+.3f}, {w_bar[1]:+.3f}, {w_bar[2]:+.3f}) m/s "
          f"(nominal mean (0, {W_MEAN_Y:+.3f}, {W_MEAN_Z:+.3f}) m/s)")

    mc = {}
    for strat in STRATEGIES:
        runs = [run_strategy(strat, seed=seed + k) for k in range(n_mc)]
        mc[strat] = {key: np.array([r[key] for r in runs], dtype=float) for key in MC_KEYS}
    print(f"\nMonte-Carlo, {n_mc} gust seeds ({seed}..{seed + n_mc - 1}), mean ± std:")
    for strat, label in zip(STRATEGIES, LABELS):
        m = mc[strat]
        print(f"  [{label:16s}] time={m['time'].mean():5.2f}±{m['time'].std():.2f}s  "
              f"rms_lat={m['rms_lat'].mean():.3f}±{m['rms_lat'].std():.3f}m  "
              f"max|lat|={m['max_lat'].mean():.3f}±{m['max_lat'].std():.3f}m  "
              f"mean_lat={m['mean_lat'].mean():+.3f}m  "
              f"rms_vert={m['rms_vert'].mean():.3f}±{m['rms_vert'].std():.3f}m  "
              f"final_dist={m['final_dist'].min():.3f}-{m['final_dist'].max():.3f}m  "
              f"arrived={int(m['arrived'].sum())}/{n_mc}")
    p = mc["pid"]
    print(f"\nCard expectation PID rms_lat < 0.2 m: met in "
          f"{int((p['rms_lat'] < 0.2).sum())}/{n_mc} seeds "
          f"(min {p['rms_lat'].min():.3f} m); "
          f"PID rms_vert < 0.3 m: met in {int((p['rms_vert'] < 0.3).sum())}/{n_mc} seeds")
    wins = int((mc["crab"]["rms_lat"] < mc["pid"]["rms_lat"]).sum())
    print(f"Crab FF has lower rms_lat than PID in {wins}/{n_mc} seeds; "
          f"PID has lower rms_vert than crab FF in "
          f"{int((mc['pid']['rms_vert'] < mc['crab']['rms_vert']).sum())}/{n_mc} seeds")
    results["_mc"] = mc
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
    ax.set_zlabel("Z (m)  [altitude]", labelpad=10)
    ax.set_title(f"S024 3D Wind Compensation — Trajectories (gust seed {SEED})", fontsize=13)
    ax.legend(loc="upper left", fontsize=8)
    ax.view_init(elev=22, azim=-55)

    out = os.path.join(OUT_DIR, "trajectory_3d.png")
    fig.savefig(out, dpi=150, bbox_inches="tight", pad_inches=0.5)
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
    ax.set_title(f"Lateral Cross-Track Error vs Time (gust seed {SEED}; + = downwind, +y)")
    ax.legend(fontsize=9);  ax.grid(True, alpha=0.3)

    # ─ Vertical cross-track error ─
    ax = axes[1]
    for strat, col, label in zip(STRATEGIES, COLORS, LABELS):
        ect  = results[strat]["ect_vert"]
        t_ax = np.arange(len(ect)) * DT
        ax.plot(t_ax, ect, color=col, lw=1.5, label=label, alpha=0.85)
    ax.axhline(0, color="k", lw=0.8, ls="--")
    ax.set_ylabel("Vertical error (m)")
    ax.set_title(f"Vertical Cross-Track Error vs Time (gust seed {SEED}; + = above the nominal climb line)")
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

    fig.suptitle(f"S024 3D Wind Compensation — showcase run, gust seed {SEED}", fontsize=13)
    fig.tight_layout()

    out = os.path.join(OUT_DIR, "crosstrack_error.png")
    fig.savefig(out, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved: {out}")


# ── Plot 3 — Monte-Carlo error / time bar chart ──────────────────────────────
def plot_lateral_bar(results):
    mc = results["_mc"]
    n_mc = len(mc["pid"]["time"])
    fig, (ax, ax_t) = plt.subplots(1, 2, figsize=(13, 5),
                                   gridspec_kw={"width_ratios": [2.2, 1]})
    metrics = [("rms_lat", "RMS lateral"), ("max_lat", "Peak |lateral|"),
               ("rms_vert", "RMS vertical"), ("max_vert", "Peak |vertical|")]
    x = np.arange(len(metrics))
    w = 0.26
    for i, (strat, col, label) in enumerate(zip(STRATEGIES, COLORS, LABELS)):
        means = [mc[strat][k].mean() for k, _ in metrics]
        stds = [mc[strat][k].std() for k, _ in metrics]
        bars = ax.bar(x + (i - 1) * w, means, w, yerr=stds, capsize=3,
                      color=col, alpha=0.85, label=label)
        for b, mval, sval in zip(bars, means, stds):
            ax.text(b.get_x() + b.get_width() / 2, mval + sval + 0.12, f"{mval:.2f}",
                    ha="center", va="bottom", fontsize=7)
    ax.axhline(0.2, color="red", ls="--", lw=1, label="Card target: PID RMS lateral < 0.2 m")
    ax.set_xticks(x)
    ax.set_xticklabels([lab for _, lab in metrics])
    ax.set_ylabel("Cross-track error (m)")
    ax.set_title(f"Cross-track error, mean ± std over {n_mc} gust seeds")
    ax.legend(fontsize=8, loc="upper right")
    ax.grid(axis="y", alpha=0.3)

    tm = [mc[s]["time"].mean() for s in STRATEGIES]
    ts = [mc[s]["time"].std() for s in STRATEGIES]
    bars = ax_t.bar(np.arange(3), tm, 0.6, yerr=ts, capsize=4, color=COLORS, alpha=0.85)
    for b, mval in zip(bars, tm):
        ax_t.text(b.get_x() + b.get_width() / 2, 0.3, f"{mval:.2f} s",
                  ha="center", va="bottom", fontsize=8, color="white")
    ax_t.set_xticks(np.arange(3))
    ax_t.set_xticklabels(LABELS, fontsize=8)
    ax_t.set_ylabel("Flight time to the 0.3 m arrival sphere (s)")
    ax_t.set_title("Time to arrive")
    ax_t.grid(axis="y", alpha=0.3)

    fig.suptitle("S024 3D Wind Compensation — Monte-Carlo summary", fontsize=12)
    fig.tight_layout()
    out = os.path.join(OUT_DIR, "lateral_bar.png")
    fig.savefig(out, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved: {out}")


# ── Animation ─────────────────────────────────────────────────────────────────
def save_animation(results):
    """Two-panel animation: top-down XY and side-view XZ."""
    fig, (ax_xy, ax_xz) = plt.subplots(1, 2, figsize=(14, 6))
    fig.suptitle(f"S024 3D Wind Compensation — gust seed {SEED}", fontsize=12)

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

    results = run_simulation(seed=SEED)

    print("\nGenerating plots ...")
    plot_trajectories_3d(results)
    plot_crosstrack_error(results)
    plot_lateral_bar(results)

    print("\nGenerating animation ...")
    save_animation(results)

    print(f"\nAll outputs saved to: {OUT_DIR}")
    print("Done.")
