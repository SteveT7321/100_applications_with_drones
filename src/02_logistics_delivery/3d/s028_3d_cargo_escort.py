"""
S028 3D Cargo Escort Formation
================================
A cargo drone (Carrier) flies a varying-altitude waypoint route while three escort
drones maintain a 3-D diamond/triangle formation around it. This is Extension 6 of
S028: escort altitudes are staggered (+/-1 m vertical offset) and vertical slot
tracking is added so escorts can hold their 3-D positions precisely even as the
carrier climbs and descends through its waypoints.

Formation: Lead escort flies ahead (+2.0 m long, +1 m alt), Left escort flies to
port (-1.5 m long, +2.0 m lat, -1 m alt), Right escort to starboard
(-1.5 m long, -2.0 m lat, +1 m alt) — in the carrier body frame.

Outputs:
    trajectory_3d.png        — 3-D trajectory of all drones
    topdown_xy.png           — XY top-down view with formation snapshots
    altitude_profile.png     — Z(t) for all drones showing formation tracking
    formation_metrics.png    — slot error & min separation vs time
    animation.gif            — animated 3-D view of the escort mission

Usage:
    conda activate drones
    "C:\\Users\\user\\anaconda3\\envs\\drones\\python.exe" -u src/02_logistics_delivery/3d/s028_3d_cargo_escort.py
"""

import matplotlib
matplotlib.use('Agg')

import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
import matplotlib.animation as animation
from matplotlib.lines import Line2D

sys.path.insert(0, os.path.normpath(os.path.join(os.path.dirname(__file__), '..', '..', '..')))

# ── Output directory ─────────────────────────────────────────────────────────
OUTPUT_DIR = os.path.normpath(os.path.join(
    os.path.dirname(__file__), '..', '..', '..',
    'outputs', '02_logistics_delivery', '3d', 's028_3d_cargo_escort',
))

# ── Parameters ────────────────────────────────────────────────────────────────
N_ESCORTS   = 3
V_CARRIER   = 3.0       # m/s — carrier cruising speed
V_ESCORT    = 7.0       # m/s — escort max speed (higher than carrier to catch up in 3D)
K_PC        = 1.2       # carrier position gain
K_DC        = 0.3       # carrier damping gain
K_PE        = 3.0       # escort slot-tracking gain
K_REP       = 3.0       # inter-drone repulsion gain
D_SAFE      = 1.0       # m — minimum safe separation
DT          = 1 / 48    # s — simulation timestep (~48 Hz)
T_MAX       = 45.0      # s — longer mission to accommodate altitude changes
ARRIVAL_TOL = 0.3       # m

# 3D Waypoints for the Carrier — varying altitude to test formation altitude tracking
WAYPOINTS = np.array([
    [-8.0,  0.0,  2.0],   # WP0: start (low)
    [-3.0,  2.0,  4.0],   # WP1: climb + turn right
    [ 0.0, -2.0,  5.5],   # WP2: peak altitude + turn left
    [ 4.0,  1.0,  3.0],   # WP3: descend + slight right
    [ 8.0,  0.0,  2.0],   # WP4: goal (back to low altitude)
])
WP_ACCEPT   = 0.5       # m — waypoint acceptance radius

# Formation offsets in carrier body frame [long, lat, alt]  — 3-D staggered
SLOT_OFFSETS = np.array([
    [ 2.0,  0.0,  1.0],   # Escort-1 Lead  — high
    [-1.5,  2.0, -1.0],   # Escort-2 Left  — low
    [-1.5, -2.0,  1.0],   # Escort-3 Right — high
])

ESCORT_COLORS = ['red', 'blue', 'orange']
ESCORT_LABELS = ['Escort-1 (Lead, +1m)', 'Escort-2 (Left, -1m)', 'Escort-3 (Right, +1m)']

RNG = np.random.default_rng(0)


# ── Helper functions ──────────────────────────────────────────────────────────

def rotation_z(psi: float) -> np.ndarray:
    """3x3 rotation matrix about Z-axis by angle psi."""
    c, s = np.cos(psi), np.sin(psi)
    return np.array([[c, -s, 0.0],
                     [s,  c, 0.0],
                     [0.0, 0.0, 1.0]])


def slot_positions(pos_c: np.ndarray, vel_c: np.ndarray) -> np.ndarray:
    """Return desired 3-D slot positions for all escorts in world frame.
    Heading (yaw) is derived from horizontal velocity components only.
    Vertical offsets are applied directly in world Z (not rotated).
    """
    speed_h = np.linalg.norm(vel_c[:2])
    psi = np.arctan2(vel_c[1], vel_c[0]) if speed_h > 1e-6 else 0.0
    R = rotation_z(psi)
    slots = []
    for f in SLOT_OFFSETS:
        # Rotate horizontal offsets, keep vertical offset in world Z
        f_rotated = R @ np.array([f[0], f[1], 0.0])
        slot = pos_c + f_rotated + np.array([0.0, 0.0, f[2]])
        slots.append(slot)
    return np.array(slots)


def escort_velocity(pos_e: np.ndarray,
                    slot:  np.ndarray,
                    vel_c: np.ndarray) -> np.ndarray:
    """Virtual-structure tracking law with carrier feed-forward."""
    delta = slot - pos_e
    v_cmd = K_PE * delta + vel_c
    speed = np.linalg.norm(v_cmd)
    if speed > V_ESCORT:
        v_cmd = v_cmd / speed * V_ESCORT
    return v_cmd


def repulsion(pos_ei: np.ndarray, pos_ej: np.ndarray) -> np.ndarray:
    """Pairwise repulsion velocity impulse (escort-escort only)."""
    r_vec = pos_ei - pos_ej
    d = np.linalg.norm(r_vec) + 1e-8
    if d < D_SAFE:
        return K_REP * (D_SAFE - d) / d * (r_vec / d)
    return np.zeros(3)


# ── Simulation ────────────────────────────────────────────────────────────────

def run_simulation():
    """Run the 3-D cargo escort formation simulation.

    Carrier follows a multi-waypoint route with altitude changes.
    Escorts maintain 3-D staggered diamond formation throughout.
    """
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    steps = int(T_MAX / DT) + 1
    t_arr = np.linspace(0.0, T_MAX, steps)

    # Carrier initial state
    pos_c = WAYPOINTS[0].copy().astype(float)
    vel_c = np.zeros(3)
    wp_idx = 1  # next waypoint to fly to

    # Escorts start at their initial slot positions (carrier heading +X initially)
    slots0 = slot_positions(pos_c, np.array([1.0, 0.0, 0.0]))
    pos_e = slots0.copy()

    # Trajectory storage
    traj_c     = np.zeros((steps, 3))
    traj_e     = np.zeros((steps, N_ESCORTS, 3))
    slot_traj  = np.zeros((steps, N_ESCORTS, 3))
    vel_c_traj = np.zeros((steps, 3))
    form_err   = np.zeros((steps, N_ESCORTS))
    sep_mat    = np.zeros((steps, N_ESCORTS, N_ESCORTS))
    wp_log     = np.zeros(steps, dtype=int)  # current waypoint index

    arrived = False
    arrive_step = steps - 1

    for k in range(steps):
        t = t_arr[k]

        # ── Record state ───────────────────────────────────────────────────
        traj_c[k]     = pos_c.copy()
        traj_e[k]     = pos_e.copy()
        vel_c_traj[k] = vel_c.copy()
        wp_log[k]     = wp_idx

        heading = vel_c if np.linalg.norm(vel_c) > 1e-6 else np.array([1.0, 0.0, 0.0])
        slots = slot_positions(pos_c, heading)
        slot_traj[k] = slots

        for i in range(N_ESCORTS):
            form_err[k, i] = np.linalg.norm(pos_e[i] - slots[i])

        for i in range(N_ESCORTS):
            for j in range(N_ESCORTS):
                sep_mat[k, i, j] = np.linalg.norm(pos_e[i] - pos_e[j])

        # ── Carrier waypoint navigation ────────────────────────────────────
        if arrived:
            vel_c = np.zeros(3)
        else:
            goal_wp = WAYPOINTS[wp_idx]
            err_c = goal_wp - pos_c
            dist_to_wp = np.linalg.norm(err_c)

            # Advance waypoint when close enough
            if dist_to_wp < WP_ACCEPT:
                if wp_idx < len(WAYPOINTS) - 1:
                    wp_idx += 1
                    goal_wp = WAYPOINTS[wp_idx]
                    err_c = goal_wp - pos_c
                    dist_to_wp = np.linalg.norm(err_c)
                else:
                    # Final waypoint reached
                    arrived = True
                    arrive_step = k
                    vel_c = np.zeros(3)
                    print(f"[t={t:.2f}s]  Carrier arrived at final waypoint.")

            if not arrived:
                v_cmd_c = K_PC * err_c - K_DC * vel_c
                speed_c = np.linalg.norm(v_cmd_c)
                if speed_c > V_CARRIER:
                    v_cmd_c = v_cmd_c / speed_c * V_CARRIER
                vel_c = v_cmd_c

        # ── Escort velocity commands ───────────────────────────────────────
        heading = vel_c if np.linalg.norm(vel_c) > 1e-6 else np.array([1.0, 0.0, 0.0])
        slots_next = slot_positions(pos_c, heading)

        vel_e_cmds = np.zeros((N_ESCORTS, 3))
        for i in range(N_ESCORTS):
            vel_e_cmds[i] = escort_velocity(pos_e[i], slots_next[i], vel_c)

        # Add pairwise repulsion and re-clamp
        for i in range(N_ESCORTS):
            for j in range(N_ESCORTS):
                if i != j:
                    vel_e_cmds[i] += repulsion(pos_e[i], pos_e[j])
            sp = np.linalg.norm(vel_e_cmds[i])
            if sp > V_ESCORT:
                vel_e_cmds[i] = vel_e_cmds[i] / sp * V_ESCORT

        # ── Integrate ──────────────────────────────────────────────────────
        pos_c += vel_c * DT
        pos_e += vel_e_cmds * DT

    # ── Metrics ───────────────────────────────────────────────────────────────
    rms_total = float(np.sqrt(np.mean(form_err ** 2)))
    rms_per_escort = [float(np.sqrt(np.mean(form_err[:, i] ** 2)))
                      for i in range(N_ESCORTS)]

    mask = ~np.eye(N_ESCORTS, dtype=bool)
    min_sep_ts = np.array([sep_mat[k][mask].min() for k in range(steps)])
    min_sep_overall = float(min_sep_ts.min())
    final_error = float(np.linalg.norm(traj_c[-1] - WAYPOINTS[-1]))

    # Vertical tracking quality: altitude deviation for each escort
    alt_err = np.zeros((steps, N_ESCORTS))
    for i in range(N_ESCORTS):
        alt_err[:, i] = np.abs(traj_e[:, i, 2] - slot_traj[:, i, 2])
    rms_alt_err = float(np.sqrt(np.mean(alt_err ** 2)))

    return dict(
        t=t_arr, steps=steps,
        traj_c=traj_c, traj_e=traj_e,
        slot_traj=slot_traj, vel_c_traj=vel_c_traj,
        form_err=form_err, sep_mat=sep_mat,
        min_sep_ts=min_sep_ts, alt_err=alt_err,
        rms_total=rms_total,
        rms_per_escort=rms_per_escort,
        min_sep_overall=min_sep_overall,
        final_error=final_error,
        rms_alt_err=rms_alt_err,
        arrive_step=arrive_step,
        arrived=arrived,
    )


# ── Plot functions ────────────────────────────────────────────────────────────

def plot_trajectory_3d(res, out_dir):
    """Full 3-D trajectories of carrier and all escorts."""
    traj_c = res['traj_c']
    traj_e = res['traj_e']
    steps  = res['steps']
    t      = res['t']

    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')

    # Carrier
    ax.plot(traj_c[:, 0], traj_c[:, 1], traj_c[:, 2],
            color='green', lw=2.0, label='Carrier', zorder=4)
    ax.scatter(*traj_c[0],       color='green', s=80, marker='o', zorder=5)
    ax.scatter(*WAYPOINTS[-1],   color='green', s=120, marker='*', zorder=6,
               label='Goal')
    # Plot waypoints
    for wp in WAYPOINTS[1:]:
        ax.scatter(*wp, color='lime', s=60, marker='D', zorder=5)

    # Escorts
    for i in range(N_ESCORTS):
        ax.plot(traj_e[:, i, 0], traj_e[:, i, 1], traj_e[:, i, 2],
                color=ESCORT_COLORS[i], lw=1.5, label=ESCORT_LABELS[i], alpha=0.9)
        ax.scatter(*traj_e[0, i], color=ESCORT_COLORS[i], s=60, marker='^')

    # Formation snapshots every ~5 s
    dt = t[1] - t[0]
    snap_interval = max(1, int(5.0 / dt))
    for k in range(0, steps, snap_interval):
        pts = np.vstack([traj_e[k, :, :],
                         traj_e[k, 0, :].reshape(1, 3)])
        ax.plot(pts[:, 0], pts[:, 1], pts[:, 2],
                color='gray', lw=0.5, alpha=0.3, linestyle='--')

    # Compute data-driven axis limits
    all_x = np.concatenate([traj_c[:, 0], traj_e[:, :, 0].ravel()])
    all_y = np.concatenate([traj_c[:, 1], traj_e[:, :, 1].ravel()])
    all_z = np.concatenate([traj_c[:, 2], traj_e[:, :, 2].ravel()])
    pad = 1.5
    ax.set_xlim(all_x.min() - pad, all_x.max() + pad)
    ax.set_ylim(all_y.min() - pad, all_y.max() + pad)
    ax.set_zlim(max(0, all_z.min() - pad), all_z.max() + pad)

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('S028 3D Cargo Escort — 3-D Trajectories\n(varying-altitude route with staggered escort altitudes)')
    ax.legend(loc='upper left', fontsize=8)

    plt.tight_layout()
    path = os.path.join(out_dir, 'trajectory_3d.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def plot_topdown_xy(res, out_dir):
    """Top-down XY view showing formation shape during transit."""
    traj_c = res['traj_c']
    traj_e = res['traj_e']
    steps  = res['steps']
    t      = res['t']

    fig, ax = plt.subplots(figsize=(12, 6))

    ax.plot(traj_c[:, 0], traj_c[:, 1], color='green', lw=2, label='Carrier')
    ax.scatter(*traj_c[0, :2],       color='green', s=80, marker='o')
    ax.scatter(*WAYPOINTS[-1, :2],   color='green', s=120, marker='*',
               label='Goal', zorder=5)
    for wp in WAYPOINTS[1:]:
        ax.scatter(*wp[:2], color='lime', s=50, marker='D', zorder=4)

    for i in range(N_ESCORTS):
        ax.plot(traj_e[:, i, 0], traj_e[:, i, 1],
                color=ESCORT_COLORS[i], lw=1.5, label=ESCORT_LABELS[i], alpha=0.9)
        ax.scatter(*traj_e[0, i, :2], color=ESCORT_COLORS[i], s=60, marker='^')

    # Formation snapshots
    dt = t[1] - t[0]
    snap_interval = max(1, int(5.0 / dt))
    for k in range(0, steps, snap_interval):
        xs = list(traj_e[k, :, 0]) + [traj_e[k, 0, 0]]
        ys = list(traj_e[k, :, 1]) + [traj_e[k, 0, 1]]
        ax.plot(xs, ys, color='gray', lw=0.6, alpha=0.4, linestyle='--')

    # Data-driven axis limits
    all_x = np.concatenate([traj_c[:, 0], traj_e[:, :, 0].ravel()])
    all_y = np.concatenate([traj_c[:, 1], traj_e[:, :, 1].ravel()])
    pad = 1.5
    ax.set_xlim(all_x.min() - pad, all_x.max() + pad)
    ax.set_ylim(all_y.min() - pad, all_y.max() + pad)

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('S028 3D — Top-Down (XY) Formation View')
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)

    plt.tight_layout()
    path = os.path.join(out_dir, 'topdown_xy.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def plot_altitude_profile(res, out_dir):
    """Z(t) for carrier and all escorts showing altitude tracking."""
    t      = res['t']
    traj_c = res['traj_c']
    traj_e = res['traj_e']

    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

    # Top: absolute altitudes
    ax = axes[0]
    ax.plot(t, traj_c[:, 2], color='green', lw=2.0, label='Carrier', zorder=4)
    for i in range(N_ESCORTS):
        ax.plot(t, traj_e[:, i, 2],
                color=ESCORT_COLORS[i], lw=1.5, label=ESCORT_LABELS[i], alpha=0.9)
    # Mark carrier waypoint altitudes
    ax.scatter([0, 8, 13, 22, 30], WAYPOINTS[:, 2], color='lime', s=60,
               marker='D', zorder=5, label='WP altitude')
    ax.set_ylabel('Altitude Z (m)')
    ax.set_title('S028 3D — Altitude Profile: Carrier + Staggered Escorts')
    ax.legend(fontsize=8, ncol=2)
    ax.grid(True, alpha=0.3)
    ax.set_ylim(bottom=0)

    # Bottom: vertical slot error per escort
    ax = axes[1]
    alt_err = res['alt_err']
    for i in range(N_ESCORTS):
        ax.plot(t, alt_err[:, i],
                color=ESCORT_COLORS[i], lw=1.5, label=ESCORT_LABELS[i])
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Vertical Slot Error (m)')
    ax.set_title('Vertical Formation Error (|z_escort - z_slot|)')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.set_ylim(bottom=0)

    plt.tight_layout()
    path = os.path.join(out_dir, 'altitude_profile.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def plot_formation_metrics(res, out_dir):
    """4-panel: 3D slot error, min separation, altitude deviation, carrier-to-goal."""
    t         = res['t']
    form_err  = res['form_err']
    min_sep   = res['min_sep_ts']
    traj_c    = res['traj_c']
    alt_err   = res['alt_err']

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    # Panel 1: 3-D slot error per escort
    ax = axes[0, 0]
    for i in range(N_ESCORTS):
        ax.plot(t, form_err[:, i],
                color=ESCORT_COLORS[i], lw=1.5, label=ESCORT_LABELS[i])
    ax.axhline(0.20, color='black', lw=1, linestyle=':', label='Target 0.20 m')
    ax.set_ylabel('3-D Formation Error (m)')
    ax.set_title('3-D Slot Error vs Time')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)
    ax.set_ylim(bottom=0)

    # Panel 2: Minimum inter-drone separation
    ax = axes[0, 1]
    ax.plot(t, min_sep, color='purple', lw=1.5, label='Min pair separation')
    ax.axhline(D_SAFE, color='red', lw=1.2, linestyle='--',
               label=f'd_safe = {D_SAFE} m')
    ax.set_ylabel('Separation (m)')
    ax.set_title('Min Inter-Drone Separation')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.set_ylim(bottom=0)

    # Panel 3: Vertical slot tracking error
    ax = axes[1, 0]
    for i in range(N_ESCORTS):
        ax.plot(t, alt_err[:, i],
                color=ESCORT_COLORS[i], lw=1.5, label=ESCORT_LABELS[i])
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Vertical Error (m)')
    ax.set_title('Vertical Slot Tracking Error')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.set_ylim(bottom=0)

    # Panel 4: Carrier distance to current/final goal
    dist_to_final = np.linalg.norm(traj_c - WAYPOINTS[-1], axis=1)
    ax = axes[1, 1]
    ax.plot(t, dist_to_final, color='green', lw=1.5, label='Distance to goal')
    ax.axhline(ARRIVAL_TOL, color='orange', lw=1.2, linestyle='--',
               label=f'Arrival tol. {ARRIVAL_TOL} m')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Distance (m)')
    ax.set_title('Carrier Distance to Final Goal')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.set_ylim(bottom=0)

    plt.suptitle('S028 3D — Cargo Escort Formation: Summary Metrics', fontsize=13)
    plt.tight_layout(rect=[0, 0, 1, 0.97])
    path = os.path.join(out_dir, 'formation_metrics.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def save_animation(res, out_dir):
    """Animated 3-D view of the 3D cargo escort mission."""
    t      = res['t']
    traj_c = res['traj_c']
    traj_e = res['traj_e']
    steps  = res['steps']

    # Downsample: target ~20 fps from ~48 Hz
    step_frame = max(1, int(round(1.0 / (20.0 * (t[1] - t[0])))))
    indices = np.arange(0, steps, step_frame)

    t_an  = t[indices]
    pC    = traj_c[indices]
    pE    = traj_e[indices]   # shape (frames, 3, 3)

    trail_len = 30  # frames of trail to show

    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')

    # Compute data-driven axis limits
    all_x = np.concatenate([pC[:, 0], pE[:, :, 0].ravel()])
    all_y = np.concatenate([pC[:, 1], pE[:, :, 1].ravel()])
    all_z = np.concatenate([pC[:, 2], pE[:, :, 2].ravel()])
    pad = 1.5
    ax.set_xlim(all_x.min() - pad, all_x.max() + pad)
    ax.set_ylim(all_y.min() - pad, all_y.max() + pad)
    ax.set_zlim(max(0, all_z.min() - pad), all_z.max() + pad)

    ax.set_xlabel('X (m)', fontsize=8)
    ax.set_ylabel('Y (m)', fontsize=8)
    ax.set_zlabel('Z (m)', fontsize=8)

    # Static waypoints
    for wp in WAYPOINTS:
        ax.scatter(*wp, color='lime', s=50, marker='D', zorder=5, alpha=0.7)
    ax.scatter(*WAYPOINTS[-1], color='green', s=120, marker='*', zorder=6)

    # Drone markers and trails
    dot_c,  = ax.plot([], [], [], 'o', color='green',  ms=10, zorder=10)
    trail_c, = ax.plot([], [], [], '-', color='green',  lw=1.5, alpha=0.5)

    dots_e   = [ax.plot([], [], [], '^', color=ESCORT_COLORS[i], ms=8, zorder=10)[0]
                for i in range(N_ESCORTS)]
    trails_e = [ax.plot([], [], [], '-', color=ESCORT_COLORS[i], lw=1.2, alpha=0.5)[0]
                for i in range(N_ESCORTS)]
    form_lines = [ax.plot([], [], [], '--', color='gray', lw=0.6, alpha=0.4)[0]
                  for _ in range(N_ESCORTS)]  # formation outline segments

    title_text = ax.set_title('', fontsize=10)

    # Legend
    legend_elements = [
        Line2D([0], [0], color='green', lw=2, label='Carrier'),
        Line2D([0], [0], color='red',   lw=2, label='Escort-1 (Lead, +1m)'),
        Line2D([0], [0], color='blue',  lw=2, label='Escort-2 (Left, -1m)'),
        Line2D([0], [0], color='orange', lw=2, label='Escort-3 (Right, +1m)'),
        Line2D([0], [0], marker='D', color='lime', ls='None', ms=7, label='Waypoints'),
    ]
    ax.legend(handles=legend_elements, fontsize=7, loc='upper left')

    def update(frame):
        i = frame
        i0 = max(0, i - trail_len)

        # Carrier
        dot_c.set_data([pC[i, 0]], [pC[i, 1]])
        dot_c.set_3d_properties([pC[i, 2]])
        trail_c.set_data(pC[i0:i+1, 0], pC[i0:i+1, 1])
        trail_c.set_3d_properties(pC[i0:i+1, 2])

        # Escorts
        for j in range(N_ESCORTS):
            dots_e[j].set_data([pE[i, j, 0]], [pE[i, j, 1]])
            dots_e[j].set_3d_properties([pE[i, j, 2]])
            trails_e[j].set_data(pE[i0:i+1, j, 0], pE[i0:i+1, j, 1])
            trails_e[j].set_3d_properties(pE[i0:i+1, j, 2])

        # Formation outline (triangle in 3D — project to lines)
        for j in range(N_ESCORTS):
            j_next = (j + 1) % N_ESCORTS
            xl = [pE[i, j, 0], pE[i, j_next, 0]]
            yl = [pE[i, j, 1], pE[i, j_next, 1]]
            zl = [pE[i, j, 2], pE[i, j_next, 2]]
            form_lines[j].set_data(xl, yl)
            form_lines[j].set_3d_properties(zl)

        title_text.set_text(f'S028 3D Cargo Escort  |  t = {t_an[i]:.1f} s')
        return ([dot_c, trail_c, title_text]
                + dots_e + trails_e + form_lines)

    ani = animation.FuncAnimation(
        fig, update, frames=len(indices),
        interval=int(1000 / 20), blit=False
    )

    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer='pillow', fps=20, dpi=100)
    plt.close()
    print(f'Saved: {path}')


# ── Main ──────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    print('=' * 65)
    print('S028 3D Cargo Escort Formation Simulation')
    print('=' * 65)

    res = run_simulation()

    print('\n--- Key Metrics ---')
    print(f'  RMS 3-D formation error (all):     {res["rms_total"]:.4f} m')
    for i in range(N_ESCORTS):
        print(f'  {ESCORT_LABELS[i]} RMS:  {res["rms_per_escort"][i]:.4f} m')
    print(f'  RMS vertical slot error:           {res["rms_alt_err"]:.4f} m')
    print(f'  Min inter-drone separation:        {res["min_sep_overall"]:.4f} m  '
          f'(d_safe = {D_SAFE} m)')
    print(f'  Carrier final error to goal:       {res["final_error"]:.4f} m')
    print(f'  Carrier arrived:                   {res["arrived"]}')

    pass_rms = res['rms_total'] < 0.20
    pass_sep = res['min_sep_overall'] >= D_SAFE
    pass_arr = res['final_error'] < ARRIVAL_TOL
    print(f'\n  [{"PASS" if pass_rms else "FAIL"}] RMS formation error < 0.20 m')
    print(f'  [{"PASS" if pass_sep else "FAIL"}] Min separation >= d_safe')
    print(f'  [{"PASS" if pass_arr else "FAIL"}] Carrier arrived within tolerance')

    print('\n--- Generating plots ---')
    out_dir = OUTPUT_DIR
    os.makedirs(out_dir, exist_ok=True)

    plot_trajectory_3d(res, out_dir)
    plot_topdown_xy(res, out_dir)
    plot_altitude_profile(res, out_dir)
    plot_formation_metrics(res, out_dir)

    print('\n--- Generating animation ---')
    save_animation(res, out_dir)

    print(f'\nAll outputs saved to: {os.path.abspath(out_dir)}')
    print('=' * 65)
