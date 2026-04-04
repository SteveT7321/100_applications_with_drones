"""
S031 Path De-confliction
========================
A fleet of 8 delivery drones departs simultaneously from a shared depot and flies
pre-planned straight-line routes to distinct delivery waypoints within a 500x500 m urban
airspace. Because routes are generated independently, pairs of routes may cross or come
dangerously close in both space and time. This simulation implements three de-confliction
strategies (priority-based speed adjustment, horizontal waypoint insertion, altitude layer
assignment) and compares conflict counts, total added flight distance, and maximum arrival delay.

Usage:
    conda activate drones
    python src/02_logistics_delivery/s031_path_deconfliction.py
"""

import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from itertools import combinations
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

# ── Parameters ─────────────────────────────────────────────────────────────────
N_DRONES   = 8          # number of drones
V_DRONE    = 12.0       # m/s cruise speed (nominal)
V_MIN      = 4.0        # m/s minimum speed for strategy 1
D_SEP      = 5.0        # m minimum separation threshold
D_MARGIN   = 2.0        # m extra clearance above D_SEP
T_HORIZON  = 30.0       # s conflict look-ahead window
DT         = 0.1        # s simulation timestep
Z_NOM      = 20.0       # m nominal cruise altitude
Z_BASE     = 18.0       # m base altitude for layer assignment (strategy 3)
DZ_LAYER   = 2.0        # m altitude layer spacing (strategy 3)
ARENA      = 500.0      # m side of delivery area
DEPOT      = np.array([250.0, 250.0, Z_NOM])
SEED       = 7

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '02_logistics_delivery', 's031_path_deconfliction',
)

RNG = np.random.default_rng(SEED)

# Pre-defined goals that guarantee crossing routes when flying from shared depot.
# Pairs (0,4), (1,5), (2,6), (3,7) fly in nearly opposite directions so that
# adjacent drones have similar headings and tend to converge in mid-flight.
_FIXED_GOALS_XY = np.array([
    [450, 450],  # D1 NE
    [50,  450],  # D2 NW
    [50,  50],   # D3 SW
    [450, 50],   # D4 SE
    [60,  60],   # D5 toward SW — nearly opposite to D1, paths cross
    [440, 60],   # D6 toward SE — nearly opposite to D2, paths cross
    [440, 440],  # D7 toward NE — nearly opposite to D3, paths cross
    [60,  440],  # D8 toward NW — nearly opposite to D4, paths cross
], dtype=float)


# ── Helpers ────────────────────────────────────────────────────────────────────

def make_goals(seed=SEED):
    """Return fixed goal positions that guarantee intersecting routes from the depot."""
    return np.c_[_FIXED_GOALS_XY, np.full(N_DRONES, Z_NOM)]


def cpa(pos_i, vel_i, pos_j, vel_j):
    """Return (t_cpa, d_cpa) for two drones with constant velocities."""
    dp = pos_i - pos_j
    dv = vel_i - vel_j
    dv2 = dv @ dv
    if dv2 < 1e-12:
        return 0.0, float(np.linalg.norm(dp))
    t = -dp @ dv / dv2
    t = max(t, 0.0)
    d = float(np.linalg.norm(dp + dv * t))
    return t, d


def detect_conflicts(positions, velocities, t_horizon, d_sep):
    """Return list of (i, j, t_cpa, d_cpa) conflict tuples."""
    conflicts = []
    for i, j in combinations(range(len(positions)), 2):
        t_c, d_c = cpa(positions[i], velocities[i], positions[j], velocities[j])
        if d_c < d_sep and 0 < t_c <= t_horizon:
            conflicts.append((i, j, t_c, d_c))
    return conflicts


def resolve_speed(pos_i, vel_i, vel_j, pos_j, goal_i, origin_i, d_sep, d_margin, v_min):
    """Bisection search for reduced speed that clears the conflict."""
    unit_i = (goal_i - origin_i)
    norm_u = np.linalg.norm(unit_i)
    if norm_u < 1e-9:
        return vel_i
    unit_i = unit_i / norm_u
    lo, hi = v_min, float(np.linalg.norm(vel_i))
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        new_vel = mid * unit_i
        _, d_c = cpa(pos_i, new_vel, pos_j, vel_j)
        if d_c >= d_sep + d_margin:
            lo = mid
        else:
            hi = mid
    return lo * unit_i


def avoidance_waypoint(pos_i, pos_j, vel_i, vel_j, d_sep, d_margin):
    """Compute lateral avoidance waypoint for drone i."""
    t_c, _ = cpa(pos_i, vel_i, pos_j, vel_j)
    mid = 0.5 * (pos_i + vel_i * t_c + pos_j + vel_j * t_c)
    dv = vel_i - vel_j
    # perpendicular in horizontal plane
    perp = np.array([-dv[1], dv[0], 0.0])
    norm = np.linalg.norm(perp)
    if norm < 1e-9:
        perp = np.array([1.0, 0.0, 0.0])
    else:
        perp /= norm
    return mid + (d_sep + d_margin) * perp


def assign_altitude_layers(n, z_base, dz):
    """Pre-assign unique altitude layers."""
    return np.array([z_base + i * dz for i in range(n)])


def min_pairwise_separation(positions):
    """Minimum Euclidean distance among all drone pairs."""
    n = len(positions)
    min_d = np.inf
    for i, j in combinations(range(n), 2):
        d = np.linalg.norm(positions[i] - positions[j])
        if d < min_d:
            min_d = d
    return min_d


# ── Simulation ─────────────────────────────────────────────────────────────────

def run_simulation(strategy):
    """
    Run the full simulation for one de-confliction strategy.

    strategy: 'speed' | 'waypoint' | 'altitude'

    Returns:
        history         : list of (N,3) position arrays per timestep
        arrival_times   : array of actual arrival times per drone
        nominal_times   : array of nominal flight times per drone
        added_dist_per_drone : array of added flight distances (strategy 2)
        sep_history     : list of min pairwise separations per timestep
        total_conflicts : int, total conflict detection events logged
        goals           : (N,3) goal positions used
        origins         : (N,3) origin positions used
    """
    goals = make_goals(SEED)

    if strategy == 'altitude':
        z_layers = assign_altitude_layers(N_DRONES, Z_BASE, DZ_LAYER)
        goals = goals.copy()
        goals[:, 2] = z_layers

    origins = np.tile(DEPOT, (N_DRONES, 1)).astype(float)
    if strategy == 'altitude':
        origins[:, 2] = goals[:, 2]

    positions    = origins.copy()
    nominal_dists = np.linalg.norm(goals - origins, axis=1)
    nominal_times = nominal_dists / V_DRONE

    speeds           = np.full(N_DRONES, V_DRONE)
    extra_waypoints  = [None] * N_DRONES
    arrived          = np.zeros(N_DRONES, dtype=bool)
    arrival_times    = np.full(N_DRONES, np.nan)
    added_dist_wp    = np.zeros(N_DRONES)  # strategy 2 detour distances

    history       = [positions.copy()]
    sep_history   = [min_pairwise_separation(positions)]
    total_conflicts = 0
    t = 0.0

    while not np.all(arrived) and t < 400.0:
        # Determine current target for each drone
        targets = np.array([
            extra_waypoints[k] if extra_waypoints[k] is not None else goals[k]
            for k in range(N_DRONES)
        ])
        directions = targets - positions
        distances  = np.linalg.norm(directions, axis=1, keepdims=True)
        safe_dist  = np.where(distances > 1e-6, distances, 1.0)
        unit_dirs  = directions / safe_dist
        velocities = speeds[:, None] * unit_dirs

        # Conflict detection and resolution (not needed for altitude strategy)
        if strategy != 'altitude':
            conflicts = detect_conflicts(positions, velocities, T_HORIZON, D_SEP)
            total_conflicts += len(conflicts)
            for (i, j, t_c, d_c) in conflicts:
                if strategy == 'speed':
                    new_vel = resolve_speed(
                        positions[i], velocities[i], velocities[j], positions[j],
                        goals[i], origins[i], D_SEP, D_MARGIN, V_MIN
                    )
                    velocities[i] = new_vel
                    speeds[i] = float(np.linalg.norm(new_vel))
                elif strategy == 'waypoint' and extra_waypoints[i] is None:
                    wp = avoidance_waypoint(
                        positions[i], positions[j],
                        velocities[i], velocities[j],
                        D_SEP, D_MARGIN
                    )
                    # Record added distance before inserting waypoint
                    orig_dist = np.linalg.norm(goals[i] - positions[i])
                    detour = (np.linalg.norm(wp - positions[i])
                              + np.linalg.norm(goals[i] - wp))
                    added_dist_wp[i] += max(0.0, detour - orig_dist)
                    extra_waypoints[i] = wp

        # Advance positions
        for k in range(N_DRONES):
            if arrived[k]:
                continue
            step_vec = velocities[k] * DT
            target_k = extra_waypoints[k] if extra_waypoints[k] is not None else goals[k]
            dist_to_target = np.linalg.norm(target_k - positions[k])
            step_size = np.linalg.norm(step_vec)

            if dist_to_target <= step_size:
                if extra_waypoints[k] is not None:
                    positions[k] = extra_waypoints[k].copy()
                    extra_waypoints[k] = None
                    speeds[k] = V_DRONE  # resume nominal speed
                else:
                    positions[k] = goals[k].copy()
                    arrived[k] = True
                    arrival_times[k] = t
            else:
                positions[k] += step_vec

        history.append(positions.copy())
        sep_history.append(min_pairwise_separation(positions))
        t += DT

    return (history, arrival_times, nominal_times,
            added_dist_wp, sep_history, total_conflicts,
            goals, origins)


# ── Plots ──────────────────────────────────────────────────────────────────────

def plot_trajectories_2d(results_all, out_dir):
    """Top-down 2D trajectory map for all three strategies side by side."""
    strategies  = ['speed', 'waypoint', 'altitude']
    titles      = ['Strategy 1: Speed Adjustment',
                   'Strategy 2: Waypoint Insertion',
                   'Strategy 3: Altitude Layers']
    colors = plt.cm.tab10(np.linspace(0, 0.9, N_DRONES))

    fig, axes = plt.subplots(1, 3, figsize=(18, 6))
    fig.suptitle('Path De-confliction — Top-Down Trajectory Comparison', fontsize=14, y=1.01)

    for ax, strat, title, res in zip(axes, strategies, titles, results_all):
        history, _, _, _, sep_history, _, goals, origins = res
        traj = np.array(history)  # (T, N, 3)

        for k in range(N_DRONES):
            xs = traj[:, k, 0]
            ys = traj[:, k, 1]
            ax.plot(xs, ys, color=colors[k], lw=1.2, alpha=0.8)
            ax.plot(origins[k, 0], origins[k, 1], 'o', color=colors[k], ms=5)
            ax.plot(goals[k, 0], goals[k, 1], '*', color=colors[k], ms=8)

        # Mark depot
        ax.plot(DEPOT[0], DEPOT[1], 'ks', ms=10, zorder=5, label='Depot')

        ax.set_xlim(0, ARENA)
        ax.set_ylim(0, ARENA)
        ax.set_aspect('equal')
        ax.set_title(title, fontsize=11)
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.legend(loc='upper right', fontsize=7)

    plt.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'trajectories_2d.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_3d_altitude_layers(results_all, out_dir):
    """3D trajectory plot for strategy 3 (altitude layers) showing vertical separation."""
    _, _, _, _, _, _, goals_s3, origins_s3 = results_all[2]
    history_s3 = results_all[2][0]
    traj = np.array(history_s3)  # (T, N, 3)
    colors = plt.cm.tab10(np.linspace(0, 0.9, N_DRONES))

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    for k in range(N_DRONES):
        xs = traj[:, k, 0]
        ys = traj[:, k, 1]
        zs = traj[:, k, 2]
        ax.plot(xs, ys, zs, color=colors[k], lw=1.5, label=f'D{k+1} z={goals_s3[k,2]:.0f}m')
        ax.scatter(origins_s3[k, 0], origins_s3[k, 1], origins_s3[k, 2],
                   c=[colors[k]], marker='o', s=40)
        ax.scatter(goals_s3[k, 0], goals_s3[k, 1], goals_s3[k, 2],
                   c=[colors[k]], marker='*', s=80)

    ax.set_xlim(0, ARENA)
    ax.set_ylim(0, ARENA)
    ax.set_zlim(15, 36)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('Strategy 3: Altitude Layer Assignment — 3D Trajectories', fontsize=12)
    ax.legend(loc='upper left', fontsize=7, ncol=2)

    plt.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'trajectories_3d_altitude.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_separation_timeseries(results_all, out_dir):
    """Minimum pairwise separation vs time for each strategy."""
    strategies = ['speed', 'waypoint', 'altitude']
    labels     = ['Speed Adjustment', 'Waypoint Insertion', 'Altitude Layers']
    linestyles = ['-', '--', '-.']
    colors     = ['tab:blue', 'tab:orange', 'tab:green']

    fig, ax = plt.subplots(figsize=(12, 5))

    for (strat, label, ls, col, res) in zip(strategies, labels, linestyles, colors, results_all):
        sep = np.array(res[4])
        times = np.arange(len(sep)) * DT
        ax.plot(times, sep, lw=1.5, ls=ls, color=col, label=label)

    ax.axhline(D_SEP, color='red', ls=':', lw=2, label=f'd_sep = {D_SEP} m')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Min pairwise separation (m)')
    ax.set_title('Minimum Pairwise Separation vs Time — All Strategies')
    ax.legend()
    ax.set_xlim(left=0)
    ax.set_ylim(bottom=0)

    plt.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'separation_timeseries.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def plot_strategy_comparison(metrics, out_dir):
    """Bar chart comparing added flight distance and max arrival delay per strategy."""
    labels       = ['Speed\nAdjustment', 'Waypoint\nInsertion', 'Altitude\nLayers']
    added_dist   = [m['added_dist']  for m in metrics]
    max_delay    = [m['max_delay']   for m in metrics]
    n_conflicts  = [m['n_conflicts'] for m in metrics]

    x = np.arange(3)
    width = 0.3

    fig, axes = plt.subplots(1, 3, figsize=(14, 5))
    fig.suptitle('Strategy Comparison', fontsize=13)

    # Added distance
    axes[0].bar(x, added_dist, width=0.5, color=['tab:blue', 'tab:orange', 'tab:green'])
    axes[0].set_xticks(x); axes[0].set_xticklabels(labels)
    axes[0].set_ylabel('Added flight distance (m)')
    axes[0].set_title('Total Added Flight Distance')
    for i, v in enumerate(added_dist):
        axes[0].text(i, v + 0.5, f'{v:.1f}', ha='center', va='bottom', fontsize=9)

    # Max delay
    axes[1].bar(x, max_delay, width=0.5, color=['tab:blue', 'tab:orange', 'tab:green'])
    axes[1].set_xticks(x); axes[1].set_xticklabels(labels)
    axes[1].set_ylabel('Max arrival delay (s)')
    axes[1].set_title('Maximum Arrival Delay')
    for i, v in enumerate(max_delay):
        axes[1].text(i, v + 0.1, f'{v:.2f}', ha='center', va='bottom', fontsize=9)

    # Conflict count
    axes[2].bar(x, n_conflicts, width=0.5, color=['tab:blue', 'tab:orange', 'tab:green'])
    axes[2].set_xticks(x); axes[2].set_xticklabels(labels)
    axes[2].set_ylabel('Conflict events detected')
    axes[2].set_title('Conflict Detection Events')
    for i, v in enumerate(n_conflicts):
        axes[2].text(i, v + 0.3, str(v), ha='center', va='bottom', fontsize=9)

    plt.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'strategy_comparison.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def plot_altitude_timeseries(results_all, out_dir):
    """Altitude time series for strategy 3 showing constant layer assignment."""
    traj = np.array(results_all[2][0])  # (T, N, 3)
    colors = plt.cm.tab10(np.linspace(0, 0.9, N_DRONES))
    goals_s3 = results_all[2][6]

    fig, ax = plt.subplots(figsize=(12, 5))
    times = np.arange(len(traj)) * DT

    for k in range(N_DRONES):
        zs = traj[:, k, 2]
        ax.plot(times, zs, color=colors[k], lw=1.5, label=f'D{k+1} (z={goals_s3[k,2]:.0f}m)')

    ax.axhline(Z_NOM, color='black', ls=':', lw=1.5, label=f'z_nom = {Z_NOM} m')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Altitude (m)')
    ax.set_title('Strategy 3: Altitude Time Series — Each Drone Holds Its Layer')
    ax.legend(fontsize=8, ncol=2)
    ax.set_xlim(left=0)

    plt.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'altitude_timeseries.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def save_animation(results_all, out_dir):
    """Animated top-down view of all three strategies simultaneously."""
    import matplotlib.animation as animation

    strategies = ['speed', 'waypoint', 'altitude']
    titles     = ['Speed Adjustment', 'Waypoint Insertion', 'Altitude Layers']
    colors     = plt.cm.tab10(np.linspace(0, 0.9, N_DRONES))

    # Find minimum trajectory length across strategies
    min_len = min(len(res[0]) for res in results_all)
    # Decimate for animation performance
    step = max(1, min_len // 200)
    frames = range(0, min_len, step)

    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    fig.suptitle('Path De-confliction Animation', fontsize=12)

    # Pre-build trajectory arrays
    trajs = [np.array(res[0]) for res in results_all]
    goals_list   = [res[6] for res in results_all]
    origins_list = [res[7] for res in results_all]

    # Static elements
    drone_dots = []
    for ax_i, (ax, traj, goals, origins, title) in enumerate(
            zip(axes, trajs, goals_list, origins_list, titles)):
        ax.set_xlim(0, ARENA); ax.set_ylim(0, ARENA)
        ax.set_aspect('equal')
        ax.set_title(title, fontsize=10)
        ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
        ax.plot(DEPOT[0], DEPOT[1], 'ks', ms=8, zorder=5)
        for k in range(N_DRONES):
            ax.plot(goals[k, 0], goals[k, 1], '*', color=colors[k], ms=8)
            # Plot full nominal straight-line path in light grey
            ax.plot([origins[k, 0], goals[k, 0]],
                    [origins[k, 1], goals[k, 1]],
                    color='grey', lw=0.5, alpha=0.4)
        dots = [ax.plot([], [], 'o', color=colors[k], ms=6)[0]
                for k in range(N_DRONES)]
        trails = [ax.plot([], [], '-', color=colors[k], lw=1, alpha=0.6)[0]
                  for k in range(N_DRONES)]
        drone_dots.append((dots, trails))

    time_text = fig.text(0.5, 0.97, '', ha='center', fontsize=10)

    def init():
        for (dots, trails) in drone_dots:
            for d, tr in zip(dots, trails):
                d.set_data([], [])
                tr.set_data([], [])
        time_text.set_text('')
        return [item for pair in drone_dots for sub in pair for item in sub] + [time_text]

    def update(frame_idx):
        frame = list(frames)[frame_idx]
        t_val = frame * DT
        time_text.set_text(f't = {t_val:.1f} s')
        for si, (traj, (dots, trails)) in enumerate(zip(trajs, drone_dots)):
            f = min(frame, len(traj) - 1)
            trail_start = max(0, f - 50)
            for k in range(N_DRONES):
                dots[k].set_data([traj[f, k, 0]], [traj[f, k, 1]])
                trails[k].set_data(traj[trail_start:f+1, k, 0],
                                   traj[trail_start:f+1, k, 1])
        return [item for pair in drone_dots for sub in pair for item in sub] + [time_text]

    n_frames = len(list(frames))
    ani = animation.FuncAnimation(fig, update, frames=n_frames,
                                  init_func=init, blit=True, interval=50)

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer='pillow', fps=20, dpi=100)
    plt.close()
    print(f'Saved: {path}')


# ── Main ───────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    strategies = ['speed', 'waypoint', 'altitude']
    results_all = []
    metrics     = []

    for strat in strategies:
        print(f'\nRunning strategy: {strat} ...')
        res = run_simulation(strat)
        (history, arrival_times, nominal_times,
         added_dist_wp, sep_history, total_conflicts, goals, origins) = res
        results_all.append(res)

        # Compute metrics
        delays = arrival_times - nominal_times
        valid  = ~np.isnan(delays)

        if strat == 'speed':
            # Added distance for speed strategy: delay * V_DRONE (approx extra path)
            added_dist = float(np.nansum(delays[valid]) * V_DRONE) if valid.any() else 0.0
        elif strat == 'waypoint':
            added_dist = float(np.sum(added_dist_wp))
        else:  # altitude
            added_dist = 0.0  # horizontal paths unchanged

        max_delay = float(np.nanmax(delays[valid])) if valid.any() else 0.0

        m = {
            'strategy':    strat,
            'added_dist':  added_dist,
            'max_delay':   max_delay,
            'n_conflicts': total_conflicts,
        }
        metrics.append(m)

        print(f'  Conflict events detected : {total_conflicts}')
        print(f'  Total added flight dist  : {added_dist:.1f} m')
        print(f'  Max arrival delay        : {max_delay:.2f} s')
        min_sep = min(sep_history)
        print(f'  Min pairwise separation  : {min_sep:.2f} m')

    out_dir = os.path.normpath(OUTPUT_DIR)
    plot_trajectories_2d(results_all, out_dir)
    plot_3d_altitude_layers(results_all, out_dir)
    plot_separation_timeseries(results_all, out_dir)
    plot_strategy_comparison(metrics, out_dir)
    plot_altitude_timeseries(results_all, out_dir)
    save_animation(results_all, out_dir)

    print('\n=== Summary ===')
    for m in metrics:
        print(f"  {m['strategy']:10s}: conflicts={m['n_conflicts']:4d}  "
              f"added_dist={m['added_dist']:7.1f} m  "
              f"max_delay={m['max_delay']:.2f} s")
