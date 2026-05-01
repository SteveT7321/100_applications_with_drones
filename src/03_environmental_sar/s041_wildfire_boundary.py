"""
S041 Wildfire Boundary Scan
============================
A fleet of 3 IR-equipped drones tracks the expanding boundary of an active
wildfire in a 200x200 m arena. The fire spreads via the elliptical Rothermel
model driven by a 45-degree NE wind. Each drone uses a gradient-following
guidance law to patrol the perimeter, updating a shared occupancy grid from
noisy IR observations and triggering replanning when contact is lost or
coverage gaps exceed 30 s. Key metrics are the mean Hausdorff distance between
the true and estimated fire boundary, and the coverage freshness fraction.

Usage:
    conda activate drones
    python src/03_environmental_sar/s041_wildfire_boundary.py
"""

import sys
import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.colors import LinearSegmentedColormap
from scipy.spatial.distance import cdist

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

# ── Parameters ─────────────────────────────────────────────────────────────
N_DRONES       = 3
ARENA_SIZE     = 200.0     # m — square arena side length
GRID_RES       = 0.5       # m — occupancy grid cell size
SCAN_HEIGHT    = 10.0      # m — drone altitude (fixed, informational)
SENSOR_RADIUS  = 2.0       # m — IR footprint radius on ground
R_SENSOR_VAR   = 25.0      # K^2 — IR sensor noise variance
T_FIRE         = 800.0     # K — temperature of burning cells
T_AMBIENT      = 300.0     # K — background temperature
T_THRESH       = 550.0     # K — burn/no-burn classification threshold
V_CRUISE       = 2.5       # m/s — drone cruise speed
K_PHI          = 0.8       # s^-1 — stand-off correction gain
D_REPLAN       = 3.0       # m — lost-contact replanning distance
DT_GAP         = 30.0      # s — max allowed scan gap per boundary cell
R0_SPREAD      = 0.04      # m/s — head-fire rate of spread
ECC            = 0.6       # ellipse eccentricity parameter e_w
WIND_DIR_DEG   = 45.0      # degrees — prevailing wind direction (from N)
SIGMA_SPOT     = 0.05      # m — spotting noise std per timestep
IGN_CENTRE     = np.array([100.0, 100.0])  # m — ignition point
R_INIT         = 20.0      # m — initial fire radius
DT             = 0.5       # s — simulation timestep
T_SIM          = 300.0     # s — total simulation duration

N_CELLS = int(ARENA_SIZE / GRID_RES)  # 400

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '03_environmental_sar', 's041_wildfire_boundary',
)

RNG = np.random.default_rng(0)

# ── Helpers ────────────────────────────────────────────────────────────────

def init_grid():
    """Initialise binary burn grid: circle of radius R_INIT centred at IGN_CENTRE."""
    xs = np.arange(N_CELLS) * GRID_RES + GRID_RES / 2.0
    ys = np.arange(N_CELLS) * GRID_RES + GRID_RES / 2.0
    X, Y = np.meshgrid(xs, ys, indexing='ij')
    dist = np.hypot(X - IGN_CENTRE[0], Y - IGN_CENTRE[1])
    return (dist <= R_INIT).astype(np.float32)


def spread_fire(F, dt, wind_dir_rad, r0, ecc, sigma_spot):
    """Expand fire by one timestep using elliptical Huygens model (vectorised)."""
    # Find boundary cells: burning cells adjacent to an unburned cell
    padded = np.pad(F, 1, constant_values=0)
    boundary_mask = np.zeros_like(F, dtype=bool)
    for di, dj in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
        neighbour = padded[1 + di:N_CELLS + 1 + di, 1 + dj:N_CELLS + 1 + dj]
        boundary_mask |= ((F == 1) & (neighbour == 0))

    cells_i, cells_j = np.where(boundary_mask)
    if len(cells_i) == 0:
        return F

    # Approximate centroid of burning region for outward normal computation
    bi, bj = np.where(F == 1)
    cx = np.mean(bi) * GRID_RES + GRID_RES / 2.0
    cy = np.mean(bj) * GRID_RES + GRID_RES / 2.0

    cell_x = cells_i * GRID_RES + GRID_RES / 2.0
    cell_y = cells_j * GRID_RES + GRID_RES / 2.0

    nx = cell_x - cx
    ny = cell_y - cy
    norm = np.hypot(nx, ny) + 1e-8
    nx /= norm
    ny /= norm

    # Angle between outward normal and wind direction
    theta = np.arctan2(ny, nx) - wind_dir_rad

    # Elliptical Rothermel rate of spread
    R_theta = r0 * (1.0 + ecc) ** 2 / (1.0 + ecc * np.cos(theta))

    # Stochastic advance
    noise = RNG.normal(0.0, sigma_spot, size=len(cells_i))
    advance = np.maximum(0.0, R_theta * dt + noise)

    # Advance fire along outward normal
    new_i = np.round(cells_i + advance * nx / GRID_RES).astype(int)
    new_j = np.round(cells_j + advance * ny / GRID_RES).astype(int)

    valid = (new_i >= 0) & (new_i < N_CELLS) & (new_j >= 0) & (new_j < N_CELLS)
    F[new_i[valid], new_j[valid]] = 1.0
    return F


def get_boundary_mask(F):
    """Return boolean mask of boundary cells (burning, adjacent to unburned)."""
    padded = np.pad(F, 1, constant_values=0)
    mask = np.zeros_like(F, dtype=bool)
    for di, dj in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
        neighbour = padded[1 + di:N_CELLS + 1 + di, 1 + dj:N_CELLS + 1 + dj]
        mask |= ((F == 1) & (neighbour == 0))
    return mask


def get_boundary_cells(F):
    """Return (i, j) index arrays for boundary cells."""
    mask = get_boundary_mask(F)
    return np.where(mask)


def ir_observation(F, drone_pos):
    """Return footprint mask and noisy binary observation."""
    xs = np.arange(N_CELLS) * GRID_RES + GRID_RES / 2.0
    ys = np.arange(N_CELLS) * GRID_RES + GRID_RES / 2.0
    X, Y = np.meshgrid(xs, ys, indexing='ij')
    dist_to_drone = np.hypot(X - drone_pos[0], Y - drone_pos[1])
    footprint = dist_to_drone <= SENSOR_RADIUS

    T_true = np.where(F == 1, T_FIRE, T_AMBIENT)
    noise = RNG.normal(0.0, np.sqrt(R_SENSOR_VAR), F.shape)
    z = T_true + noise
    obs = (z >= T_THRESH).astype(np.float32)
    return footprint, obs


def boundary_gradient_step(pos, F_est):
    """Compute commanded velocity for one drone using gradient-following law."""
    bi, bj = get_boundary_cells(F_est)
    if len(bi) == 0:
        return np.zeros(2), False

    bx = bi * GRID_RES + GRID_RES / 2.0
    by = bj * GRID_RES + GRID_RES / 2.0
    dists = np.hypot(bx - pos[0], by - pos[1])
    nearest_idx = np.argmin(dists)
    d_fire = dists[nearest_idx]

    nearest_pt = np.array([bx[nearest_idx], by[nearest_idx]])
    n_hat = (pos - nearest_pt) / (d_fire + 1e-8)   # inward normal (drone to fire)

    # Boundary tangent (clockwise): rotate n_hat by -90 degrees
    t_hat = np.array([n_hat[1], -n_hat[0]])

    phi = d_fire - SENSOR_RADIUS
    vel = V_CRUISE * t_hat - K_PHI * phi * n_hat

    speed = np.linalg.norm(vel)
    if speed > V_CRUISE:
        vel = vel * V_CRUISE / speed

    replan = (d_fire > SENSOR_RADIUS + D_REPLAN)
    return vel, replan


def select_replan_target(F_est, last_scan, t_now):
    """Find boundary cell with the oldest scan timestamp."""
    bi, bj = get_boundary_cells(F_est)
    if len(bi) == 0:
        return None
    ages = t_now - last_scan[bi, bj]
    oldest_idx = np.argmax(ages)
    return np.array([
        bi[oldest_idx] * GRID_RES + GRID_RES / 2.0,
        bj[oldest_idx] * GRID_RES + GRID_RES / 2.0,
    ])


def hausdorff_distance(bi_true, bj_true, bi_est, bj_est):
    """Symmetric mean Hausdorff distance between two boundary cell sets."""
    if len(bi_true) == 0 or len(bi_est) == 0:
        return 0.0
    tb = np.stack([bi_true, bj_true], axis=1).astype(float) * GRID_RES
    eb = np.stack([bi_est,  bj_est],  axis=1).astype(float) * GRID_RES
    D = cdist(tb, eb)
    d1 = np.mean(np.min(D, axis=1))
    d2 = np.mean(np.min(D, axis=0))
    return 0.5 * (d1 + d2)


# ── Simulation ──────────────────────────────────────────────────────────────

def run_simulation():
    """Run the main wildfire boundary scan simulation. Return all data for plotting."""
    wind_rad = np.deg2rad(WIND_DIR_DEG)
    F_true = init_grid()
    F_est  = F_true.copy()

    # Per-cell last-scan timestamp (initialised to -inf)
    last_scan = np.full((N_CELLS, N_CELLS), -np.inf)

    # Initialise drones evenly spaced around initial fire perimeter
    angles = np.linspace(0, 2.0 * np.pi, N_DRONES, endpoint=False)
    drone_pos = np.array([
        IGN_CENTRE + (R_INIT + SENSOR_RADIUS) * np.array([np.cos(a), np.sin(a)])
        for a in angles
    ])
    replan_targets = [None] * N_DRONES

    t_steps = int(T_SIM / DT)
    hausdorff_history  = np.zeros(t_steps)
    freshness_history  = np.zeros(t_steps)
    drone_trajectories = [[] for _ in range(N_DRONES)]
    replan_events      = [[] for _ in range(N_DRONES)]

    # Snapshot times for the evolution map plot
    snap_times  = {0: None, 100: None, 200: None, int(T_SIM) - 1: None}
    snap_frames = {}

    for step in range(t_steps):
        t = step * DT

        # 1. Spread fire
        F_true = spread_fire(F_true, DT, wind_rad, R0_SPREAD, ECC, SIGMA_SPOT)

        # 2. Each drone observes and updates shared estimate
        for k in range(N_DRONES):
            footprint, obs = ir_observation(F_true, drone_pos[k])
            F_est[footprint] = obs[footprint]
            ci_arr, cj_arr = np.where(footprint)
            last_scan[ci_arr, cj_arr] = t

        # 3. Move each drone
        for k in range(N_DRONES):
            if replan_targets[k] is not None:
                diff = replan_targets[k] - drone_pos[k]
                dist = np.linalg.norm(diff)
                if dist < V_CRUISE * DT or dist < 1e-6:
                    drone_pos[k] = replan_targets[k].copy()
                    replan_targets[k] = None
                else:
                    drone_pos[k] += V_CRUISE * (diff / dist) * DT
            else:
                vel, need_replan = boundary_gradient_step(drone_pos[k], F_est)
                drone_pos[k] += vel * DT
                if need_replan:
                    replan_events[k].append(drone_pos[k].copy())
                    replan_targets[k] = select_replan_target(F_est, last_scan, t)

            # Gap coverage trigger
            bi_b, bj_b = get_boundary_cells(F_est)
            if len(bi_b) > 0 and replan_targets[k] is None:
                ages = t - last_scan[bi_b, bj_b]
                if np.any(ages > DT_GAP):
                    replan_events[k].append(drone_pos[k].copy())
                    replan_targets[k] = select_replan_target(F_est, last_scan, t)

            drone_pos[k] = np.clip(drone_pos[k], 0.0, ARENA_SIZE)
            drone_trajectories[k].append(drone_pos[k].copy())

        # 4. Compute metrics
        bi_t, bj_t = get_boundary_cells(F_true)
        bi_e, bj_e = get_boundary_cells(F_est)
        hausdorff_history[step] = hausdorff_distance(bi_t, bj_t, bi_e, bj_e)

        if len(bi_e) > 0:
            fresh_ages = t - last_scan[bi_e, bj_e]
            freshness_history[step] = np.mean(fresh_ages <= DT_GAP)
        else:
            freshness_history[step] = 1.0

        # Save snapshots
        t_int = int(round(t))
        if t_int in snap_times and snap_times[t_int] is None:
            snap_times[t_int] = step
            snap_frames[t_int] = {
                'F_true': F_true.copy(),
                'F_est':  F_est.copy(),
                'drone_pos': drone_pos.copy(),
                'last_scan': last_scan.copy(),
                't': t,
            }

    t_axis = np.arange(t_steps) * DT
    drone_trajectories = [np.array(tr) for tr in drone_trajectories]

    # Print key metrics
    mean_hd  = np.mean(hausdorff_history)
    final_hd = hausdorff_history[-1]
    mean_fr  = np.mean(freshness_history) * 100.0
    print(f"Mean Hausdorff distance : {mean_hd:.2f} m")
    print(f"Final Hausdorff distance: {final_hd:.2f} m")
    print(f"Mean coverage freshness : {mean_fr:.1f} %")
    total_replan = sum(len(e) for e in replan_events)
    print(f"Total replanning events : {total_replan}")
    bi_f, bj_f = get_boundary_cells(F_true)
    # Approx fire area
    fire_area = np.sum(F_true) * GRID_RES ** 2
    print(f"Final fire area         : {fire_area:.0f} m^2")

    return {
        'F_true_final':      F_true,
        'F_est_final':       F_est,
        'hausdorff_history': hausdorff_history,
        'freshness_history': freshness_history,
        'drone_trajectories': drone_trajectories,
        'replan_events':     replan_events,
        't_axis':            t_axis,
        'snap_frames':       snap_frames,
        'last_scan':         last_scan,
        'mean_hd':           mean_hd,
        'final_hd':          final_hd,
        'mean_fr':           mean_fr,
        'total_replan':      total_replan,
    }


# ── Colours ─────────────────────────────────────────────────────────────────
DRONE_COLORS = ['#e63946', '#2a9d8f', '#f4a261']


# ── Plots ───────────────────────────────────────────────────────────────────

def plot_fire_evolution(snap_frames, out_dir):
    """4-panel fire evolution map at t=0, 100, 200, 300 s."""
    snap_keys = sorted(snap_frames.keys())
    fig, axes = plt.subplots(1, 4, figsize=(20, 5))

    xs = np.arange(N_CELLS) * GRID_RES
    ys = np.arange(N_CELLS) * GRID_RES

    for ax, t_key in zip(axes, snap_keys):
        frame = snap_frames[t_key]
        F_t = frame['F_true']
        F_e = frame['F_est']
        dpos = frame['drone_pos']
        t_val = frame['t']

        # True burn region (orange)
        ax.contourf(xs, ys, F_t.T, levels=[0.5, 1.5], colors=['#ff6b35'],
                    alpha=0.55)
        # Estimated boundary (red contour)
        bnd_e = get_boundary_mask(F_e)
        if bnd_e.any():
            ax.contour(xs, ys, bnd_e.T.astype(float), levels=[0.5],
                       colors=['#c1121f'], linewidths=1.2)
        # True boundary (dark orange)
        bnd_t = get_boundary_mask(F_t)
        if bnd_t.any():
            ax.contour(xs, ys, bnd_t.T.astype(float), levels=[0.5],
                       colors=['#8b2500'], linewidths=0.8, linestyles='--')

        # Drones and sensor footprints
        for k in range(N_DRONES):
            px, py = dpos[k]
            circle = plt.Circle((px, py), SENSOR_RADIUS,
                                 color=DRONE_COLORS[k], fill=False,
                                 linewidth=1.2, alpha=0.8)
            ax.add_patch(circle)
            ax.plot(px, py, 'o', color=DRONE_COLORS[k], ms=6, zorder=5)

        # Wind arrow
        wind_rad = np.deg2rad(WIND_DIR_DEG)
        ax.annotate('', xy=(15 + 12 * np.cos(wind_rad),
                             15 + 12 * np.sin(wind_rad)),
                    xytext=(15, 15),
                    arrowprops=dict(arrowstyle='->', color='navy', lw=1.5))
        ax.text(15, 8, 'Wind', fontsize=7, color='navy')

        ax.set_xlim(60, 160)
        ax.set_ylim(60, 160)
        ax.set_aspect('equal')
        ax.set_title(f't = {t_val:.0f} s', fontsize=11)
        ax.set_xlabel('x (m)')
        if ax == axes[0]:
            ax.set_ylabel('y (m)')

    # Legend
    patches = [
        mpatches.Patch(color='#ff6b35', alpha=0.55, label='True burn region'),
        mpatches.Patch(color='#c1121f', label='Estimated boundary'),
        mpatches.Patch(color='#8b2500', label='True boundary'),
    ]
    for k in range(N_DRONES):
        patches.append(mpatches.Patch(color=DRONE_COLORS[k],
                                      label=f'Drone {k+1} + footprint'))
    fig.legend(handles=patches, loc='lower center', ncol=6,
               bbox_to_anchor=(0.5, -0.05), fontsize=8)
    fig.suptitle('S041 Wildfire Boundary Scan — Fire Evolution', fontsize=13,
                 fontweight='bold')
    plt.tight_layout(rect=[0, 0.05, 1, 0.97])
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'fire_evolution.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_hausdorff(hausdorff_history, t_axis, replan_events,
                   drone_trajectories, out_dir):
    """Hausdorff distance over time with replanning event markers."""
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(t_axis, hausdorff_history, color='#264653', lw=1.5,
            label='Hausdorff distance $d_H(t)$')
    ax.axhline(np.mean(hausdorff_history), color='#e9c46a', lw=1.2,
               ls='--', label=f'Mean = {np.mean(hausdorff_history):.2f} m')

    # Mark replanning events for each drone
    for k, events in enumerate(replan_events):
        if events:
            ev_arr = np.array(events)
            # Map events to time by nearest trajectory point
            traj = drone_trajectories[k]
            ev_times = []
            for ev_pos in ev_arr:
                diffs = traj - ev_pos[np.newaxis, :]
                idx = np.argmin(np.linalg.norm(diffs, axis=1))
                ev_times.append(t_axis[min(idx, len(t_axis) - 1)])
            ev_times = np.array(ev_times)
            hd_at_ev = np.interp(ev_times, t_axis, hausdorff_history)
            ax.scatter(ev_times, hd_at_ev, marker='^', s=50,
                       color=DRONE_COLORS[k], zorder=5,
                       label=f'Drone {k+1} replan ({len(ev_times)})')

    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Hausdorff distance (m)')
    ax.set_title('Boundary Tracking Error — Mean Hausdorff Distance')
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'hausdorff_distance.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def plot_coverage_freshness(freshness_history, t_axis, out_dir):
    """Coverage freshness fraction over time."""
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(t_axis, freshness_history * 100.0, color='#2a9d8f', lw=1.5,
            label='Coverage freshness $\\eta(t)$')
    ax.axhline(90.0, color='#e76f51', lw=1.2, ls='--',
               label='90% target threshold')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Fresh boundary fraction (%)')
    ax.set_ylim(-5, 105)
    ax.set_title('Coverage Freshness — Fraction of Boundary Scanned within 30 s')
    ax.legend(fontsize=10)
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'coverage_freshness.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def plot_trajectory_overlay(drone_trajectories, F_true_final, replan_events,
                             out_dir):
    """Full 2D drone paths colour-coded by time, superimposed on final burn footprint."""
    fig, ax = plt.subplots(figsize=(8, 8))

    xs = np.arange(N_CELLS) * GRID_RES
    ys = np.arange(N_CELLS) * GRID_RES
    ax.contourf(xs, ys, F_true_final.T, levels=[0.5, 1.5],
                colors=['#ff6b35'], alpha=0.35)
    ax.contour(xs, ys, F_true_final.T, levels=[0.5],
               colors=['#8b2500'], linewidths=0.8)

    cmap_list = [
        LinearSegmentedColormap.from_list('r_t', ['#2196F3', '#F44336']),
        LinearSegmentedColormap.from_list('g_t', ['#4CAF50', '#FF9800']),
        LinearSegmentedColormap.from_list('b_t', ['#9C27B0', '#FFEB3B']),
    ]

    for k, traj in enumerate(drone_trajectories):
        n = len(traj)
        for i in range(n - 1):
            frac = i / max(n - 1, 1)
            col = cmap_list[k](frac)
            ax.plot(traj[i:i+2, 0], traj[i:i+2, 1], '-', color=col,
                    lw=0.8, alpha=0.7)
        ax.plot(traj[0, 0], traj[0, 1], 's', color=DRONE_COLORS[k], ms=7,
                label=f'Drone {k+1} start')
        ax.plot(traj[-1, 0], traj[-1, 1], '*', color=DRONE_COLORS[k], ms=10)

        # Replan events
        if replan_events[k]:
            ev_arr = np.array(replan_events[k])
            ax.scatter(ev_arr[:, 0], ev_arr[:, 1], marker='^', s=50,
                       color=DRONE_COLORS[k], zorder=5,
                       edgecolors='k', linewidths=0.5)

    # Wind arrow
    wind_rad = np.deg2rad(WIND_DIR_DEG)
    ax.annotate('', xy=(170 + 15 * np.cos(wind_rad),
                         170 + 15 * np.sin(wind_rad)),
                xytext=(170, 170),
                arrowprops=dict(arrowstyle='->', color='navy', lw=2))
    ax.text(175, 163, 'Wind NE', fontsize=9, color='navy')

    ax.set_xlim(50, 180)
    ax.set_ylim(50, 180)
    ax.set_aspect('equal')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title('Drone Trajectory Overlay (colour = time, blue→red)\n'
                 'Triangles = replanning events')
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.2)
    plt.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'trajectory_overlay.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def save_animation(snap_frames, drone_trajectories, t_axis, out_dir):
    """Animate fire expansion with drones orbiting the boundary."""
    import matplotlib.animation as animation

    # Use every 4th step for performance (0.5 s * 4 = 2 s per frame, 150 frames)
    t_steps = len(t_axis)
    step_size = 4
    frame_indices = list(range(0, t_steps, step_size))

    # Pre-build a list of (F_true, drone_pos) snapshots by re-running
    # We use the drone_trajectories already stored
    # Rebuild F_true snapshots by using snap_frames at known times and
    # linearly picking the closest. For animation we need per-step F_true.
    # Because storing every grid is memory-intensive, we re-simulate compactly.

    # --- Lightweight re-simulation storing only what animation needs ---
    wind_rad = np.deg2rad(WIND_DIR_DEG)
    F_anim = init_grid()
    anim_frames = []

    for step in range(t_steps):
        F_anim = spread_fire(F_anim, DT, wind_rad, R0_SPREAD, ECC, SIGMA_SPOT)
        if step in frame_indices:
            dposes = [drone_trajectories[k][min(step, len(drone_trajectories[k])-1)]
                      for k in range(N_DRONES)]
            anim_frames.append((F_anim.copy(), list(dposes), step * DT))

    xs = np.arange(N_CELLS) * GRID_RES
    ys = np.arange(N_CELLS) * GRID_RES

    fig, ax = plt.subplots(figsize=(6, 6))
    ax.set_xlim(50, 170)
    ax.set_ylim(50, 170)
    ax.set_aspect('equal')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')

    # Initial artists
    contf = [None]
    contb = [None]
    drone_markers = [ax.plot([], [], 'o', color=DRONE_COLORS[k],
                             ms=8, zorder=5)[0]
                     for k in range(N_DRONES)]
    footprint_circles = [plt.Circle((0, 0), SENSOR_RADIUS,
                                     color=DRONE_COLORS[k], fill=False,
                                     lw=1.2, alpha=0.7)
                          for k in range(N_DRONES)]
    for c in footprint_circles:
        ax.add_patch(c)
    time_text = ax.text(0.02, 0.96, '', transform=ax.transAxes, fontsize=10,
                        va='top', color='black',
                        bbox=dict(boxstyle='round', facecolor='white', alpha=0.7))
    title_text = ax.set_title('S041 Wildfire Boundary Scan', fontsize=11)

    def update(frame_idx):
        F, dposes, t_val = anim_frames[frame_idx]
        # Clear previous contours
        for coll in ax.collections[:]:
            coll.remove()
        for c in footprint_circles:
            ax.add_patch(c)

        # Fire region
        ax.contourf(xs, ys, F.T, levels=[0.5, 1.5],
                    colors=['#ff6b35'], alpha=0.55)
        ax.contour(xs, ys, F.T, levels=[0.5],
                   colors=['#8b2500'], linewidths=0.8)

        for k in range(N_DRONES):
            px, py = dposes[k]
            drone_markers[k].set_data([px], [py])
            footprint_circles[k].center = (px, py)

        time_text.set_text(f't = {t_val:.0f} s')
        return drone_markers + footprint_circles + [time_text]

    ani = animation.FuncAnimation(
        fig, update, frames=len(anim_frames),
        interval=80, blit=False
    )

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer='pillow', fps=12, dpi=100)
    plt.close()
    print(f'Saved: {path}')


# ── Main ────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    print('Running S041 Wildfire Boundary Scan simulation...')
    data = run_simulation()

    out_dir = os.path.normpath(OUTPUT_DIR)
    plot_fire_evolution(data['snap_frames'], out_dir)
    plot_hausdorff(data['hausdorff_history'], data['t_axis'],
                   data['replan_events'], data['drone_trajectories'], out_dir)
    plot_coverage_freshness(data['freshness_history'], data['t_axis'], out_dir)
    plot_trajectory_overlay(data['drone_trajectories'], data['F_true_final'],
                             data['replan_events'], out_dir)
    save_animation(data['snap_frames'], data['drone_trajectories'],
                   data['t_axis'], out_dir)
    print('Done.')
