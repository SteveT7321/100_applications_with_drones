"""
S028 Cargo Escort Formation
===========================
A cargo drone (Carrier) flies from (-8, 0, 2) to (8, 0, 2) while three
escort drones maintain a protective triangle formation around it.

Formation control uses virtual-structure (leader-follower) with feed-forward
velocity matching and pairwise repulsion for collision avoidance.
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.lines import Line2D
import matplotlib.animation as animation

# ── Output directory ────────────────────────────────────────────────────────
OUT_DIR = os.path.join(
    os.path.dirname(__file__),
    "../../outputs/02_logistics_delivery/s028_cargo_escort_formation"
)
os.makedirs(OUT_DIR, exist_ok=True)

# ── Simulation constants ─────────────────────────────────────────────────────
N_ESCORTS   = 3
V_CARRIER   = 3.0     # m/s — carrier cruising speed
V_ESCORT    = 6.0     # m/s — escort max speed
K_PC        = 1.5     # carrier position gain
K_DC        = 0.4     # carrier damping gain
K_PE        = 3.0     # escort slot-tracking gain
K_REP       = 3.0     # inter-drone repulsion gain
D_SAFE      = 1.0     # m — minimum safe separation
DT          = 1 / 48  # s — timestep (~48 Hz)
T_MAX       = 30.0    # s

CARRIER_START = np.array([-8.0, 0.0, 2.0])
CARRIER_GOAL  = np.array([ 8.0, 0.0, 2.0])
ARRIVAL_TOL   = 0.2   # m

# Formation offsets in carrier body frame [long, lat, alt]
SLOT_OFFSETS = np.array([
    [ 2.0,  0.0,  0.0],   # Escort-1 Lead
    [-1.5,  2.0,  0.0],   # Escort-2 Left
    [-1.5, -2.0,  0.0],   # Escort-3 Right
])

ESCORT_COLORS  = ['red', 'blue', 'orange']
ESCORT_LABELS  = ['Escort-1 (Lead)', 'Escort-2 (Left)', 'Escort-3 (Right)']

# ── Helper functions ─────────────────────────────────────────────────────────

def rotation_z(psi: float) -> np.ndarray:
    """Rotation matrix about Z-axis by angle psi."""
    c, s = np.cos(psi), np.sin(psi)
    return np.array([[c, -s, 0.0],
                     [s,  c, 0.0],
                     [0.0, 0.0, 1.0]])


def slot_positions(pos_c: np.ndarray, vel_c: np.ndarray) -> np.ndarray:
    """
    Return desired slot positions for all escorts in world frame.
    If the carrier is nearly stationary, heading defaults to +X.
    """
    speed = np.linalg.norm(vel_c[:2])
    psi = np.arctan2(vel_c[1], vel_c[0]) if speed > 1e-6 else 0.0
    R = rotation_z(psi)
    return np.array([pos_c + R @ f for f in SLOT_OFFSETS])


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


# ── Main simulation ──────────────────────────────────────────────────────────

def run_simulation():
    steps = int(T_MAX / DT) + 1
    t_arr = np.linspace(0.0, T_MAX, steps)

    # State arrays
    pos_c = CARRIER_START.copy()
    vel_c = np.zeros(3)

    # Escorts start at their initial slot positions
    slots0 = slot_positions(pos_c, np.array([1.0, 0.0, 0.0]))  # initial heading +X
    pos_e  = slots0.copy()

    # Trajectory storage
    traj_c = np.zeros((steps, 3))
    traj_e = np.zeros((steps, N_ESCORTS, 3))
    slot_traj = np.zeros((steps, N_ESCORTS, 3))
    vel_c_traj = np.zeros((steps, 3))
    form_err = np.zeros((steps, N_ESCORTS))
    sep_mat  = np.zeros((steps, N_ESCORTS, N_ESCORTS))

    arrived = False
    arrive_step = steps - 1

    for k, t in enumerate(t_arr):
        # ── Record state ────────────────────────────────────────────────
        traj_c[k]      = pos_c.copy()
        traj_e[k]      = pos_e.copy()
        vel_c_traj[k]  = vel_c.copy()

        slots = slot_positions(pos_c, vel_c if np.linalg.norm(vel_c) > 1e-6
                               else np.array([1.0, 0.0, 0.0]))
        slot_traj[k] = slots

        for i in range(N_ESCORTS):
            form_err[k, i] = np.linalg.norm(pos_e[i] - slots[i])

        for i in range(N_ESCORTS):
            for j in range(N_ESCORTS):
                sep_mat[k, i, j] = np.linalg.norm(pos_e[i] - pos_e[j])

        # ── Check carrier arrival ────────────────────────────────────────
        if not arrived and np.linalg.norm(pos_c - CARRIER_GOAL) < ARRIVAL_TOL:
            arrived = True
            arrive_step = k
            print(f"[t={t:.2f}s]  Carrier arrived at goal. Steps remaining: {steps-k}")

        if arrived:
            vel_c = np.zeros(3)
        else:
            # Carrier PD controller
            err_c = CARRIER_GOAL - pos_c
            v_cmd_c = K_PC * err_c - K_DC * vel_c
            speed_c = np.linalg.norm(v_cmd_c)
            if speed_c > V_CARRIER:
                v_cmd_c = v_cmd_c / speed_c * V_CARRIER
            vel_c = v_cmd_c

        # ── Escort velocities ────────────────────────────────────────────
        heading = vel_c if np.linalg.norm(vel_c) > 1e-6 else np.array([1.0, 0.0, 0.0])
        slots_next = slot_positions(pos_c, heading)

        vel_e_cmds = np.zeros((N_ESCORTS, 3))
        for i in range(N_ESCORTS):
            vel_e_cmds[i] = escort_velocity(pos_e[i], slots_next[i], vel_c)

        # Add pairwise repulsion
        for i in range(N_ESCORTS):
            for j in range(N_ESCORTS):
                if i != j:
                    vel_e_cmds[i] += repulsion(pos_e[i], pos_e[j])
            # Re-clamp after repulsion
            sp = np.linalg.norm(vel_e_cmds[i])
            if sp > V_ESCORT:
                vel_e_cmds[i] = vel_e_cmds[i] / sp * V_ESCORT

        # ── Integrate positions ──────────────────────────────────────────
        pos_c += vel_c * DT
        pos_e += vel_e_cmds * DT

    # ── Metrics ─────────────────────────────────────────────────────────────
    rms_total = float(np.sqrt(np.mean(form_err ** 2)))
    rms_per_escort = [float(np.sqrt(np.mean(form_err[:, i] ** 2)))
                      for i in range(N_ESCORTS)]

    # Min inter-drone separation (off-diagonal only)
    mask  = ~np.eye(N_ESCORTS, dtype=bool)
    min_sep_ts = np.array([sep_mat[k][mask].min() for k in range(steps)])
    min_sep_overall = float(min_sep_ts.min())
    final_carrier_error = float(np.linalg.norm(traj_c[-1] - CARRIER_GOAL))

    return dict(
        t=t_arr, steps=steps,
        traj_c=traj_c, traj_e=traj_e,
        slot_traj=slot_traj, vel_c_traj=vel_c_traj,
        form_err=form_err, sep_mat=sep_mat,
        min_sep_ts=min_sep_ts,
        rms_total=rms_total,
        rms_per_escort=rms_per_escort,
        min_sep_overall=min_sep_overall,
        final_carrier_error=final_carrier_error,
        arrive_step=arrive_step,
        arrived=arrived,
    )


# ── Plotting ─────────────────────────────────────────────────────────────────

def plot_trajectory_3d(res):
    fig = plt.figure(figsize=(12, 8))
    ax  = fig.add_subplot(111, projection='3d')

    traj_c = res['traj_c']
    traj_e = res['traj_e']
    steps  = res['steps']

    # Carrier
    ax.plot(traj_c[:, 0], traj_c[:, 1], traj_c[:, 2],
            color='green', lw=2.0, label='Carrier')
    ax.scatter(*traj_c[0],   color='green', s=80, marker='o', zorder=5)
    ax.scatter(*CARRIER_GOAL, color='green', s=120, marker='*', zorder=5,
               label='Goal')

    # Escorts
    for i in range(N_ESCORTS):
        ax.plot(traj_e[:, i, 0], traj_e[:, i, 1], traj_e[:, i, 2],
                color=ESCORT_COLORS[i], lw=1.5, label=ESCORT_LABELS[i], alpha=0.9)
        ax.scatter(*traj_e[0, i], color=ESCORT_COLORS[i], s=60, marker='^')

    # Formation snapshots every ~3 s
    snap_interval = int(3.0 / (res['t'][1] - res['t'][0]))
    for k in range(0, steps, snap_interval):
        pts = np.vstack([traj_e[k, :, :],
                         traj_e[k, 0, :].reshape(1, 3)])  # close triangle
        ax.plot(pts[:, 0], pts[:, 1], pts[:, 2],
                color='gray', lw=0.6, alpha=0.35, linestyle='--')

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('S028 — Cargo Escort Formation: 3D Trajectories')
    ax.legend(loc='upper left', fontsize=8)

    plt.tight_layout()
    path = os.path.join(OUT_DIR, 'trajectory_3d.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f"Saved: {path}")


def plot_topdown_xy(res):
    fig, ax = plt.subplots(figsize=(10, 6))

    traj_c = res['traj_c']
    traj_e = res['traj_e']
    steps  = res['steps']

    ax.plot(traj_c[:, 0], traj_c[:, 1], color='green', lw=2, label='Carrier')
    ax.scatter(*traj_c[0, :2],   color='green', s=80,  marker='o')
    ax.scatter(*CARRIER_GOAL[:2], color='green', s=120, marker='*',
               label='Goal', zorder=5)

    for i in range(N_ESCORTS):
        ax.plot(traj_e[:, i, 0], traj_e[:, i, 1],
                color=ESCORT_COLORS[i], lw=1.5, label=ESCORT_LABELS[i], alpha=0.9)
        ax.scatter(*traj_e[0, i, :2], color=ESCORT_COLORS[i], s=60, marker='^')

    # Formation snapshots
    snap_interval = int(3.0 / (res['t'][1] - res['t'][0]))
    for k in range(0, steps, snap_interval):
        xs = list(traj_e[k, :, 0]) + [traj_e[k, 0, 0]]
        ys = list(traj_e[k, :, 1]) + [traj_e[k, 0, 1]]
        ax.plot(xs, ys, color='gray', lw=0.6, alpha=0.4, linestyle='--')

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('S028 — Top-Down (XY) Formation View')
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)

    plt.tight_layout()
    path = os.path.join(OUT_DIR, 'topdown_xy.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f"Saved: {path}")


def plot_formation_error(res):
    fig, axes = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

    t         = res['t']
    form_err  = res['form_err']
    min_sep   = res['min_sep_ts']

    ax = axes[0]
    for i in range(N_ESCORTS):
        ax.plot(t, form_err[:, i],
                color=ESCORT_COLORS[i], lw=1.5, label=ESCORT_LABELS[i])
    ax.axhline(0.15, color='black', lw=1, linestyle=':', label='Target (0.15 m)')
    ax.set_ylabel('Formation Error (m)')
    ax.set_title('S028 — Formation Error & Inter-Drone Separation')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.set_ylim(bottom=0)

    ax = axes[1]
    ax.plot(t, min_sep, color='purple', lw=1.5, label='Min pair separation')
    ax.axhline(D_SAFE, color='red', lw=1, linestyle='--',
               label=f'd_safe = {D_SAFE} m')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Separation (m)')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.set_ylim(bottom=0)

    plt.tight_layout()
    path = os.path.join(OUT_DIR, 'formation_metrics.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f"Saved: {path}")


def plot_combined_metrics(res):
    """4-panel summary figure."""
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    t         = res['t']
    traj_c    = res['traj_c']
    traj_e    = res['traj_e']
    form_err  = res['form_err']
    min_sep   = res['min_sep_ts']
    steps     = res['steps']

    # Panel 1: Top-down
    ax = axes[0, 0]
    ax.plot(traj_c[:, 0], traj_c[:, 1], color='green', lw=2, label='Carrier')
    ax.scatter(*CARRIER_GOAL[:2], color='green', s=100, marker='*', zorder=5)
    for i in range(N_ESCORTS):
        ax.plot(traj_e[:, i, 0], traj_e[:, i, 1],
                color=ESCORT_COLORS[i], lw=1.2, label=ESCORT_LABELS[i])
    snap_interval = int(3.0 / (t[1] - t[0]))
    for k in range(0, steps, snap_interval):
        xs = list(traj_e[k, :, 0]) + [traj_e[k, 0, 0]]
        ys = list(traj_e[k, :, 1]) + [traj_e[k, 0, 1]]
        ax.plot(xs, ys, 'gray', lw=0.5, alpha=0.4, linestyle='--')
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
    ax.set_title('Top-Down View (XY)')
    ax.set_aspect('equal'); ax.grid(True, alpha=0.3)
    ax.legend(fontsize=7)

    # Panel 2: Formation error per escort
    ax = axes[0, 1]
    for i in range(N_ESCORTS):
        ax.plot(t, form_err[:, i],
                color=ESCORT_COLORS[i], lw=1.5, label=ESCORT_LABELS[i])
    ax.axhline(0.15, color='k', lw=1, linestyle=':', label='Target 0.15 m')
    ax.set_xlabel('Time (s)'); ax.set_ylabel('Error (m)')
    ax.set_title('Formation Slot Error vs Time')
    ax.legend(fontsize=7); ax.grid(True, alpha=0.3); ax.set_ylim(bottom=0)

    # Panel 3: Minimum inter-drone separation
    ax = axes[1, 0]
    ax.plot(t, min_sep, color='purple', lw=1.5)
    ax.axhline(D_SAFE, color='red', lw=1.2, linestyle='--',
               label=f'd_safe={D_SAFE} m')
    ax.set_xlabel('Time (s)'); ax.set_ylabel('Separation (m)')
    ax.set_title('Min Inter-Drone Separation')
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3); ax.set_ylim(bottom=0)

    # Panel 4: Carrier distance-to-goal
    dist_to_goal = np.linalg.norm(traj_c - CARRIER_GOAL, axis=1)
    ax = axes[1, 1]
    ax.plot(t, dist_to_goal, color='green', lw=1.5)
    ax.axhline(ARRIVAL_TOL, color='orange', lw=1.2, linestyle='--',
               label=f'Arrival tol. {ARRIVAL_TOL} m')
    ax.set_xlabel('Time (s)'); ax.set_ylabel('Distance (m)')
    ax.set_title('Carrier Distance to Goal')
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3); ax.set_ylim(bottom=0)

    plt.suptitle('S028 — Cargo Escort Formation: Summary Metrics', fontsize=13)
    plt.tight_layout(rect=[0, 0, 1, 0.97])
    path = os.path.join(OUT_DIR, 'combined_metrics.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f"Saved: {path}")


def save_animation(res):
    """Save an animated GIF of the formation transit (top-down view)."""
    traj_c = res['traj_c']
    traj_e = res['traj_e']
    t      = res['t']
    steps  = res['steps']

    # Subsample to ~15 fps for manageable GIF size
    target_fps = 15
    dt_orig    = t[1] - t[0]
    skip       = max(1, int(round(1.0 / (target_fps * dt_orig))))
    indices    = list(range(0, steps, skip))

    fig, ax = plt.subplots(figsize=(8, 5))
    ax.set_xlim(-10, 10); ax.set_ylim(-5, 5)
    ax.set_aspect('equal')
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
    ax.set_title('S028 — Cargo Escort Formation (animated)')
    ax.scatter(*CARRIER_GOAL[:2], color='green', s=120, marker='*', zorder=6,
               label='Goal')
    ax.grid(True, alpha=0.3)

    carrier_dot,  = ax.plot([], [], 'o', color='green',  ms=10, label='Carrier')
    escort_dots   = [ax.plot([], [], '^', color=ESCORT_COLORS[i], ms=8,
                             label=ESCORT_LABELS[i])[0] for i in range(N_ESCORTS)]
    formation_line, = ax.plot([], [], '--', color='gray', lw=0.8, alpha=0.5)
    time_text = ax.text(0.02, 0.95, '', transform=ax.transAxes, fontsize=9)

    # Faded trail
    trail_len = 30
    trail_c_line, = ax.plot([], [], '-', color='green', lw=1, alpha=0.4)
    trail_e_lines = [ax.plot([], [], '-', color=ESCORT_COLORS[i], lw=1,
                             alpha=0.4)[0] for i in range(N_ESCORTS)]

    ax.legend(fontsize=7, loc='upper left')

    def init():
        carrier_dot.set_data([], [])
        for ed in escort_dots:
            ed.set_data([], [])
        formation_line.set_data([], [])
        trail_c_line.set_data([], [])
        for tl in trail_e_lines:
            tl.set_data([], [])
        time_text.set_text('')
        return ([carrier_dot, formation_line, trail_c_line, time_text]
                + escort_dots + trail_e_lines)

    def update(frame_idx):
        k = indices[frame_idx]
        carrier_dot.set_data([traj_c[k, 0]], [traj_c[k, 1]])
        for i, ed in enumerate(escort_dots):
            ed.set_data([traj_e[k, i, 0]], [traj_e[k, i, 1]])

        # Formation outline
        xs = list(traj_e[k, :, 0]) + [traj_e[k, 0, 0]]
        ys = list(traj_e[k, :, 1]) + [traj_e[k, 0, 1]]
        formation_line.set_data(xs, ys)

        # Trails
        t0 = max(0, k - trail_len * skip)
        sl = slice(t0, k + 1, skip)
        trail_c_line.set_data(traj_c[sl, 0], traj_c[sl, 1])
        for i, tl in enumerate(trail_e_lines):
            tl.set_data(traj_e[sl, i, 0], traj_e[sl, i, 1])

        time_text.set_text(f't = {t[k]:.1f} s')
        return ([carrier_dot, formation_line, trail_c_line, time_text]
                + escort_dots + trail_e_lines)

    anim = animation.FuncAnimation(
        fig, update, frames=len(indices),
        init_func=init, blit=True, interval=int(1000 / target_fps)
    )

    path = os.path.join(OUT_DIR, 'formation_animation.gif')
    writer = animation.PillowWriter(fps=target_fps)
    anim.save(path, writer=writer)
    plt.close()
    print(f"Saved: {path}")


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    print("=" * 60)
    print("S028 — Cargo Escort Formation Simulation")
    print("=" * 60)

    res = run_simulation()

    print("\n--- Key Metrics ---")
    print(f"  RMS formation error (all escorts): {res['rms_total']:.4f} m  "
          f"(target < 0.15 m)")
    for i in range(N_ESCORTS):
        print(f"  {ESCORT_LABELS[i]} RMS error: {res['rms_per_escort'][i]:.4f} m")
    print(f"  Min inter-drone separation:        {res['min_sep_overall']:.4f} m  "
          f"(d_safe = {D_SAFE} m)")
    print(f"  Carrier final error to goal:       {res['final_carrier_error']:.4f} m")
    print(f"  Carrier arrived:                   {res['arrived']}")
    print(f"  Simulation steps:                  {res['steps']}")
    print()

    # Status flags
    pass_rms = res['rms_total'] < 0.15
    pass_sep = res['min_sep_overall'] >= D_SAFE
    pass_arr = res['final_carrier_error'] < ARRIVAL_TOL
    print(f"  [{'PASS' if pass_rms else 'FAIL'}] RMS formation error < 0.15 m")
    print(f"  [{'PASS' if pass_sep else 'FAIL'}] Min separation >= d_safe")
    print(f"  [{'PASS' if pass_arr else 'FAIL'}] Carrier arrived within tolerance")

    print("\n--- Generating plots ---")
    plot_trajectory_3d(res)
    plot_topdown_xy(res)
    plot_formation_error(res)
    plot_combined_metrics(res)
    print("\n--- Generating animation ---")
    save_animation(res)

    print("\nAll outputs saved to:", os.path.abspath(OUT_DIR))
    print("=" * 60)


if __name__ == "__main__":
    main()
