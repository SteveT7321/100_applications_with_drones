"""
S049 Dynamic Zone Search
========================
Four drones search a 400x400m disaster area using Voronoi zone partitioning.
A prior probability map (Gaussian mixture) guides sweep order. Three strategies
are compared: (1) Static Voronoi — fixed initial partition, (2) Dynamic Equal-weight
Voronoi — reallocation on survivor-find or low battery using equal weights,
(3) Dynamic Battery-weighted Voronoi — reallocation using SoC-proportional weights
so higher-battery drones absorb more uncovered area.

Usage:
    conda activate drones
    python src/03_environmental_sar/s049_dynamic_zone.py
"""

import sys
import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.colors as mcolors
from matplotlib.gridspec import GridSpec
from scipy.spatial import KDTree

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

# ── Parameters ─────────────────────────────────────────────────────────────
K_DRONES    = 4          # number of drones
AREA_SIZE   = 400.0      # m — square search area side length
GRID_RES    = 5.0        # m — cell resolution (delta)
SPEED       = 6.0        # m/s — drone cruise speed
SENSOR_R    = 8.0        # m — sensor footprint radius
D_MAX       = 2000.0     # m — maximum range on full battery
SOC_THRESH  = 0.30       # reallocation trigger SoC threshold
N_SURVIVORS = 5          # number of survivors
DT          = 0.5        # s — simulation timestep
T_MAX       = 3600.0     # s — safety cutoff

Nx = int(AREA_SIZE / GRID_RES)   # 80
Ny = int(AREA_SIZE / GRID_RES)   # 80

DRONE_COLORS = ['#E63946', '#2196F3', '#4CAF50', '#FF9800']  # red, blue, green, orange

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '03_environmental_sar', 's049_dynamic_zone',
)

RNG = np.random.default_rng(0)  # fixed seed for reproducibility


# ── Helpers ────────────────────────────────────────────────────────────────

def build_prior_map():
    """Build a 3-component Gaussian mixture prior probability map (Nx x Ny)."""
    xi = np.arange(Nx) * GRID_RES + GRID_RES / 2.0
    yi = np.arange(Ny) * GRID_RES + GRID_RES / 2.0
    xv, yv = np.meshgrid(xi, yi, indexing='ij')
    coords = np.stack([xv, yv], axis=-1)   # (Nx, Ny, 2)

    components = [
        (0.5, np.array([100., 300.]), 60.),
        (0.3, np.array([280., 150.]), 50.),
        (0.2, np.array([340., 320.]), 40.),
    ]
    P = np.zeros((Nx, Ny))
    for alpha, mu, sigma in components:
        diff = coords - mu
        P += alpha * np.exp(-0.5 * np.sum(diff ** 2, axis=-1) / sigma ** 2)
    P /= P.sum()
    return P


def build_cell_centres():
    """Return (Nx*Ny, 2) array of cell centre coordinates."""
    xi = np.arange(Nx) * GRID_RES + GRID_RES / 2.0
    yi = np.arange(Ny) * GRID_RES + GRID_RES / 2.0
    xv, yv = np.meshgrid(xi, yi, indexing='ij')
    return np.stack([xv.ravel(), yv.ravel()], axis=1)


def weighted_voronoi_assignment(positions, weights, candidate_centres):
    """
    Assign each candidate cell to drone with minimum battery-weighted distance.

    weighted dist = ||x - p_k|| / w_k

    positions:         (K, 2)
    weights:           (K,)  — SoC values in (0, 1]
    candidate_centres: (N, 2)

    Returns assignment array (N,) with drone index per cell.
    """
    K = len(positions)
    N = len(candidate_centres)
    scaled_dist = np.zeros((N, K))
    for k in range(K):
        diff = candidate_centres - positions[k]
        dist = np.linalg.norm(diff, axis=1)
        scaled_dist[:, k] = dist / (weights[k] + 1e-9)
    return np.argmin(scaled_dist, axis=1)


def jain_fairness(areas):
    """Compute Jain fairness index for array of per-drone remaining areas."""
    areas = np.array(areas, dtype=float)
    if len(areas) == 0 or areas.sum() == 0:
        return 1.0
    return areas.sum() ** 2 / (len(areas) * (areas ** 2).sum())


def place_survivors(cell_centres, P_flat, n=N_SURVIVORS, rng=None):
    """Sample survivor positions from prior map distribution."""
    if rng is None:
        rng = np.random.default_rng(42)
    idx = rng.choice(len(cell_centres), size=n, replace=False, p=P_flat)
    return cell_centres[idx].copy()


# ── Drone Agent ────────────────────────────────────────────────────────────

class DroneAgent:
    def __init__(self, idx, start_pos):
        self.idx = idx
        self.pos = start_pos.copy().astype(float)
        self.soc = 1.0
        self.dist_flown = 0.0
        self.waypoints = []        # list of (2,) arrays
        self.active = True
        self.found_survivor = False
        self.history = [start_pos.copy().astype(float)]

    def step(self):
        """Advance one timestep. Returns distance moved."""
        if not self.active or len(self.waypoints) == 0:
            return 0.0
        target = self.waypoints[0]
        diff = target - self.pos
        dist_to_target = np.linalg.norm(diff)
        move = SPEED * DT
        if dist_to_target <= move:
            actual_move = dist_to_target
            self.pos = target.copy()
            self.waypoints.pop(0)
        else:
            actual_move = move
            self.pos += (diff / dist_to_target) * move
        self.dist_flown += actual_move
        self.soc = max(0.0, 1.0 - self.dist_flown / D_MAX)
        self.history.append(self.pos.copy())
        return actual_move


# ── Simulation ─────────────────────────────────────────────────────────────

def assign_waypoints(active_drones, scanned_mask, cell_centres, P_flat, strategy):
    """
    Compute battery-weighted (or equal) Voronoi zones and assign priority-sorted
    waypoints to each active drone.
    """
    if len(active_drones) == 0:
        return

    positions = np.array([d.pos for d in active_drones])
    if strategy == 'weighted':
        weights = np.array([max(d.soc, 1e-3) for d in active_drones])
    else:
        weights = np.ones(len(active_drones))

    unscanned_idx = np.where(~scanned_mask)[0]
    if len(unscanned_idx) == 0:
        for d in active_drones:
            d.waypoints = []
        return

    unscanned_centres = cell_centres[unscanned_idx]
    assignment = weighted_voronoi_assignment(positions, weights, unscanned_centres)

    for i, d in enumerate(active_drones):
        zone_local = np.where(assignment == i)[0]
        zone_global = unscanned_idx[zone_local]
        if len(zone_global) == 0:
            d.waypoints = []
            continue
        # Sort by descending prior probability
        probs = P_flat[zone_global]
        order = np.argsort(-probs)
        sorted_centres = cell_centres[zone_global[order]]
        d.waypoints = list(sorted_centres)


def run_simulation(strategy='weighted'):
    """
    Run the dynamic zone search simulation.

    strategy: 'static' | 'equal' | 'weighted'

    Returns dict with all simulation results.
    """
    P = build_prior_map()
    P_flat = P.ravel()
    cell_centres = build_cell_centres()
    cell_tree = KDTree(cell_centres)

    # Use fixed RNG for reproducible survivor placement
    sim_rng = np.random.default_rng(42)
    survivors = place_survivors(cell_centres, P_flat, rng=sim_rng)
    survivor_found = [False] * N_SURVIVORS
    detect_times = [None] * N_SURVIVORS

    # Drones start at corners
    starts = np.array([
        [20.0,  20.0],
        [380.0, 20.0],
        [20.0,  380.0],
        [380.0, 380.0],
    ])
    drones = [DroneAgent(k, starts[k]) for k in range(K_DRONES)]

    scanned = np.zeros(len(cell_centres), dtype=bool)

    # Reallocation snapshot storage
    zone_snapshots = []   # list of (t, assignment_full_grid)
    realloc_events = []   # list of (t, drone_idx, reason)

    def get_full_zone_map(active_drones):
        """Compute zone assignment for all cells (for visualisation)."""
        if len(active_drones) == 0:
            return np.full(len(cell_centres), -1)
        positions = np.array([d.pos for d in active_drones])
        if strategy == 'weighted':
            weights = np.array([max(d.soc, 1e-3) for d in active_drones])
        else:
            weights = np.ones(len(active_drones))
        assignment_local = weighted_voronoi_assignment(positions, weights, cell_centres)
        # Map local index back to actual drone idx
        result = np.array([active_drones[a].idx for a in assignment_local])
        return result

    # Initial assignment
    active = [d for d in drones if d.active]
    if strategy == 'static':
        assign_waypoints(active, scanned, cell_centres, P_flat, 'equal')
    else:
        assign_waypoints(active, scanned, cell_centres, P_flat, strategy)

    # Record initial zone map
    zone_snapshots.append((0.0, get_full_zone_map(active)))

    t = 0.0
    coverage_log = []
    fairness_log = []
    t_log = []

    while True:
        active = [d for d in drones if d.active]
        if not active:
            break
        if all(survivor_found):
            break

        # Step all active drones
        for d in active:
            d.step()

        # Mark newly scanned cells
        for d in active:
            nearby = cell_tree.query_ball_point(d.pos, SENSOR_R)
            for c in nearby:
                scanned[c] = True

        # Check survivor detections
        for i, s in enumerate(survivors):
            if survivor_found[i]:
                continue
            for d in active:
                if np.linalg.norm(d.pos - s) <= SENSOR_R:
                    survivor_found[i] = True
                    detect_times[i] = t
                    if strategy != 'static':
                        d.active = False
                        d.found_survivor = True
                        realloc_events.append((t, d.idx, 'survivor'))
                    break

        # Check SoC triggers (non-static)
        if strategy != 'static':
            for d in list(active):
                if d.active and d.soc < SOC_THRESH:
                    d.active = False
                    realloc_events.append((t, d.idx, 'battery'))

        # Reallocation when any drone just deactivated
        active_now = [d for d in drones if d.active]
        if len(active_now) < len(active) and len(active_now) > 0:
            assign_waypoints(active_now, scanned, cell_centres, P_flat, strategy)
            zone_snapshots.append((t, get_full_zone_map(active_now)))

        # Log metrics
        coverage = scanned.sum() / len(scanned)
        coverage_log.append(coverage)
        rem_areas = [len(d.waypoints) * GRID_RES ** 2
                     for d in active_now if d.active]
        fairness_log.append(jain_fairness(rem_areas) if rem_areas else 1.0)
        t_log.append(t)

        t += DT
        if t > T_MAX:
            break

    # Compute final time-to-find-all
    valid_times = [dt for dt in detect_times if dt is not None]
    time_all_found = max(valid_times) if valid_times else T_MAX
    n_found = sum(1 for dt in detect_times if dt is not None)

    return {
        'strategy': strategy,
        'drones': drones,
        'survivors': survivors,
        'detect_times': detect_times,
        'coverage_log': np.array(coverage_log),
        'fairness_log': np.array(fairness_log),
        't_log': np.array(t_log),
        'realloc_events': realloc_events,
        'zone_snapshots': zone_snapshots,
        'scanned_final': scanned.copy(),
        'cell_centres': cell_centres,
        'P': P,
        'P_flat': P_flat,
        'time_all_found': time_all_found,
        'n_found': n_found,
        'mean_detect_time': float(np.mean(valid_times)) if valid_times else T_MAX,
        'final_coverage': scanned.sum() / len(scanned),
        'n_realloc': len(realloc_events),
        'mean_fairness': float(np.mean(fairness_log)) if len(fairness_log) > 0 else 0.0,
    }


# ── Plots ──────────────────────────────────────────────────────────────────

def plot_search_area_map(result, out_dir):
    """Prior probability heatmap with initial Voronoi zones, survivors, and drone starts."""
    P = result['P']
    survivors = result['survivors']
    zone_map = result['zone_snapshots'][0][1]   # initial assignment
    cell_centres = result['cell_centres']

    fig, ax = plt.subplots(figsize=(8, 8))

    # Prior heatmap
    P_img = P.T   # (Ny, Nx) for imshow with origin='lower'
    im = ax.imshow(
        P_img, origin='lower', extent=[0, AREA_SIZE, 0, AREA_SIZE],
        cmap='YlOrRd', alpha=0.55, aspect='equal',
    )
    plt.colorbar(im, ax=ax, label='Prior probability (normalised)')

    # Zone colour overlay
    zone_img = np.full((Nx, Ny), -1)
    for ci, (cx, cy) in enumerate(cell_centres):
        ix = int(cx / GRID_RES)
        iy = int(cy / GRID_RES)
        if 0 <= ix < Nx and 0 <= iy < Ny:
            zone_img[ix, iy] = zone_map[ci]

    cmap_zones = mcolors.ListedColormap(DRONE_COLORS[:K_DRONES])
    zone_rgba = np.zeros((Ny, Nx, 4))
    for k in range(K_DRONES):
        mask = zone_img.T == k
        c = mcolors.to_rgba(DRONE_COLORS[k], alpha=0.18)
        zone_rgba[mask] = c
    ax.imshow(zone_rgba, origin='lower', extent=[0, AREA_SIZE, 0, AREA_SIZE], aspect='equal')

    # Drone start positions
    starts = np.array([[20, 20], [380, 20], [20, 380], [380, 380]])
    for k in range(K_DRONES):
        ax.plot(starts[k, 0], starts[k, 1], '^', color=DRONE_COLORS[k],
                markersize=12, zorder=5, label=f'Drone {k+1} start')

    # Survivor positions
    ax.scatter(survivors[:, 0], survivors[:, 1], s=200, marker='*',
               color='gold', edgecolors='black', zorder=6, linewidths=1.0,
               label='Survivors')

    ax.set_xlim(0, AREA_SIZE)
    ax.set_ylim(0, AREA_SIZE)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('S049: Search Area — Prior Map & Initial Voronoi Zones')
    ax.legend(loc='upper right', fontsize=9)

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'search_area_map.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_reallocation_snapshots(result, out_dir):
    """Three side-by-side panels: t=0, first realloc, second realloc (weighted strategy)."""
    if result['strategy'] != 'weighted':
        return

    cell_centres = result['cell_centres']
    survivors = result['survivors']
    snapshots = result['zone_snapshots']
    scanned_final = result['scanned_final']

    n_panels = min(3, len(snapshots))
    fig, axes = plt.subplots(1, n_panels, figsize=(5 * n_panels, 5), constrained_layout=True)
    if n_panels == 1:
        axes = [axes]

    titles = ['t = 0 s (initial)', 'After 1st reallocation', 'After 2nd reallocation']

    for panel_i, (t_snap, zone_map) in enumerate(snapshots[:n_panels]):
        ax = axes[panel_i]

        # Build zone colour image
        zone_rgba = np.zeros((Ny, Nx, 4))
        for ci, (cx, cy) in enumerate(cell_centres):
            ix = int(cx / GRID_RES)
            iy = int(cy / GRID_RES)
            if 0 <= ix < Nx and 0 <= iy < Ny:
                k = zone_map[ci]
                if 0 <= k < K_DRONES:
                    c = mcolors.to_rgba(DRONE_COLORS[k], alpha=0.55)
                    zone_rgba[iy, ix] = c

        ax.imshow(zone_rgba, origin='lower', extent=[0, AREA_SIZE, 0, AREA_SIZE], aspect='equal')

        # Grey out scanned cells (after simulation end; approximate for t=0 as empty)
        if panel_i > 0:
            scanned_rgba = np.zeros((Ny, Nx, 4))
            for ci, (cx, cy) in enumerate(cell_centres):
                ix = int(cx / GRID_RES)
                iy = int(cy / GRID_RES)
                if 0 <= ix < Nx and 0 <= iy < Ny:
                    if scanned_final[ci]:
                        scanned_rgba[iy, ix] = (0.7, 0.7, 0.7, 0.6)
            ax.imshow(scanned_rgba, origin='lower', extent=[0, AREA_SIZE, 0, AREA_SIZE], aspect='equal')

        # Survivors
        ax.scatter(survivors[:, 0], survivors[:, 1], s=150, marker='*',
                   color='gold', edgecolors='black', zorder=5, linewidths=0.8)

        ax.set_xlim(0, AREA_SIZE)
        ax.set_ylim(0, AREA_SIZE)
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)' if panel_i == 0 else '')
        snap_title = titles[panel_i] if panel_i < len(titles) else f't={t_snap:.0f}s'
        ax.set_title(snap_title)

    fig.suptitle('S049: Dynamic Voronoi Zone Reallocation Snapshots', fontsize=13)
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'reallocation_snapshots.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_trajectories(result, out_dir):
    """Final trajectories of all drones with detection events marked."""
    drones = result['drones']
    survivors = result['survivors']
    detect_times = result['detect_times']
    realloc_events = result['realloc_events']
    P = result['P']

    fig, ax = plt.subplots(figsize=(8, 8))

    # Background: prior map
    ax.imshow(P.T, origin='lower', extent=[0, AREA_SIZE, 0, AREA_SIZE],
              cmap='Blues', alpha=0.3, aspect='equal')

    for d in drones:
        if len(d.history) < 2:
            continue
        hist = np.array(d.history)
        ax.plot(hist[:, 0], hist[:, 1], '-', color=DRONE_COLORS[d.idx],
                linewidth=0.8, alpha=0.8, label=f'Drone {d.idx+1}')
        ax.plot(hist[0, 0], hist[0, 1], '^', color=DRONE_COLORS[d.idx], markersize=10, zorder=5)
        ax.plot(hist[-1, 0], hist[-1, 1], 's', color=DRONE_COLORS[d.idx], markersize=8, zorder=5)

    # Survivors: mark found with circle
    for i, s in enumerate(survivors):
        if detect_times[i] is not None:
            ax.scatter(s[0], s[1], s=200, marker='*', color='gold',
                       edgecolors='black', zorder=7, linewidths=1.0)
            ax.add_patch(plt.Circle((s[0], s[1]), SENSOR_R * 1.5,
                                    fill=False, edgecolor='gold', linewidth=1.5, zorder=6))
        else:
            ax.scatter(s[0], s[1], s=200, marker='*', color='gray',
                       edgecolors='black', zorder=7, linewidths=1.0)

    ax.set_xlim(0, AREA_SIZE)
    ax.set_ylim(0, AREA_SIZE)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title(f'S049: Drone Trajectories — {result["strategy"].capitalize()} Strategy')
    ax.legend(loc='upper right', fontsize=9)

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'trajectories.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_coverage_comparison(results_dict, out_dir):
    """Coverage vs time for all three strategies."""
    fig, ax = plt.subplots(figsize=(10, 5))

    style = {
        'static':   ('--', '#888888', 'Static Voronoi'),
        'equal':    ('-.', '#2196F3', 'Equal-weight Voronoi'),
        'weighted': ('-',  '#E63946', 'Battery-weighted Voronoi'),
    }

    for strat, res in results_dict.items():
        ls, color, label = style[strat]
        ax.plot(res['t_log'], res['coverage_log'] * 100.0,
                linestyle=ls, color=color, linewidth=1.8, label=label)
        # Realloc event markers
        if strat != 'static':
            for (t_ev, didx, reason) in res['realloc_events']:
                ax.axvline(t_ev, color=color, alpha=0.3, linewidth=0.8, linestyle=':')

    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Coverage (%)')
    ax.set_title('S049: Area Coverage vs Time — Strategy Comparison')
    ax.legend(loc='lower right', fontsize=10)
    ax.grid(True, alpha=0.3)
    ax.set_xlim(left=0)
    ax.set_ylim(0, 102)

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'coverage_comparison.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_survivor_detection_timeline(results_dict, out_dir):
    """Grouped bar chart of detection times per survivor per strategy."""
    strategies = list(results_dict.keys())
    n_strat = len(strategies)
    n_surv = N_SURVIVORS

    fig, ax = plt.subplots(figsize=(10, 5))
    x = np.arange(n_surv)
    width = 0.25
    colors_strat = ['#888888', '#2196F3', '#E63946']

    for si, strat in enumerate(strategies):
        times = [t if t is not None else T_MAX for t in results_dict[strat]['detect_times']]
        offset = (si - 1) * width
        bars = ax.bar(x + offset, times, width, label=strat.capitalize(),
                      color=colors_strat[si], alpha=0.8, edgecolor='black', linewidth=0.5)

    ax.set_xlabel('Survivor Index')
    ax.set_ylabel('Detection Time (s)')
    ax.set_title('S049: Survivor Detection Timeline by Strategy')
    ax.set_xticks(x)
    ax.set_xticklabels([f'S{i+1}' for i in range(n_surv)])
    ax.legend(fontsize=10)
    ax.grid(True, axis='y', alpha=0.3)

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'detection_timeline.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_jain_fairness(results_dict, out_dir):
    """Jain fairness index vs time for all strategies."""
    fig, ax = plt.subplots(figsize=(10, 5))

    style = {
        'static':   ('--', '#888888', 'Static Voronoi'),
        'equal':    ('-.', '#2196F3', 'Equal-weight Voronoi'),
        'weighted': ('-',  '#E63946', 'Battery-weighted Voronoi'),
    }

    for strat, res in results_dict.items():
        ls, color, label = style[strat]
        ax.plot(res['t_log'], res['fairness_log'],
                linestyle=ls, color=color, linewidth=1.8, label=label)
        if strat != 'static':
            for (t_ev, didx, reason) in res['realloc_events']:
                ax.axvline(t_ev, color=color, alpha=0.3, linewidth=0.8, linestyle=':')

    ax.axhline(1.0, color='black', linestyle=':', alpha=0.4, label='Perfect fairness')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Jain Fairness Index J(t)')
    ax.set_title('S049: Load Balance (Jain Fairness) vs Time — Strategy Comparison')
    ax.legend(loc='lower left', fontsize=10)
    ax.grid(True, alpha=0.3)
    ax.set_xlim(left=0)
    ax.set_ylim(0, 1.05)

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'jain_fairness.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_summary_table(results_dict, out_dir):
    """Summary statistics table as a figure."""
    strategies = list(results_dict.keys())
    col_labels = ['Strategy', 'Time (all found) s', 'Mean detect time s',
                  'Final coverage %', '# realloc', 'Mean fairness']
    rows = []
    for strat in strategies:
        res = results_dict[strat]
        rows.append([
            strat.capitalize(),
            f"{res['time_all_found']:.1f}",
            f"{res['mean_detect_time']:.1f}",
            f"{res['final_coverage']*100:.1f}",
            str(res['n_realloc']),
            f"{res['mean_fairness']:.3f}",
        ])

    fig, ax = plt.subplots(figsize=(11, 2.5))
    ax.axis('off')
    tbl = ax.table(cellText=rows, colLabels=col_labels, cellLoc='center', loc='center')
    tbl.auto_set_font_size(False)
    tbl.set_fontsize(11)
    tbl.scale(1.2, 2.0)
    ax.set_title('S049: Strategy Comparison Summary', fontsize=13, pad=20)

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'summary_table.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def save_animation(result, out_dir):
    """Top-down animation: drones move, cells fill, Voronoi updates, survivors detected."""
    import matplotlib.animation as animation

    drones = result['drones']
    survivors = result['survivors']
    detect_times = result['detect_times']
    cell_centres = result['cell_centres']
    P = result['P']
    zone_snapshots = result['zone_snapshots']  # list of (t, zone_map)
    t_log = result['t_log']

    # Subsample for animation speed
    step = max(1, len(t_log) // 300)
    frames = list(range(0, len(t_log), step))
    if not frames:
        return

    # Pre-compute scanned state at each frame
    # Replay drone histories to build scanned state per frame
    cell_tree = KDTree(cell_centres)
    n_cells = len(cell_centres)
    scanned_frames = []
    scanned_now = np.zeros(n_cells, dtype=bool)
    frame_hist_idx = [0] * K_DRONES  # how many history steps used per drone

    for fi, frame_idx in enumerate(frames):
        t_frame = t_log[frame_idx]
        # Update scanned: how many steps has each drone taken?
        for d in drones:
            steps_needed = min(frame_idx + 1, len(d.history))
            while frame_hist_idx[d.idx] < steps_needed:
                pos = d.history[frame_hist_idx[d.idx]]
                nearby = cell_tree.query_ball_point(pos, SENSOR_R)
                for c in nearby:
                    scanned_now[c] = True
                frame_hist_idx[d.idx] += 1
        scanned_frames.append(scanned_now.copy())

    # Determine which zone snapshot applies at each frame
    snap_times = [s[0] for s in zone_snapshots]

    def get_snap_idx(t):
        idx = 0
        for si, st in enumerate(snap_times):
            if st <= t:
                idx = si
        return idx

    fig, ax = plt.subplots(figsize=(7, 7))
    ax.set_xlim(0, AREA_SIZE)
    ax.set_ylim(0, AREA_SIZE)
    ax.set_aspect('equal')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    title_obj = ax.set_title('S049: Dynamic Zone Search (t=0.0s)')

    # Background prior
    ax.imshow(P.T, origin='lower', extent=[0, AREA_SIZE, 0, AREA_SIZE],
              cmap='YlOrRd', alpha=0.25, aspect='equal')

    # Zone overlay (imshow, updated per frame)
    zone_rgba_init = np.zeros((Ny, Nx, 4))
    zone_im = ax.imshow(zone_rgba_init, origin='lower',
                        extent=[0, AREA_SIZE, 0, AREA_SIZE], aspect='equal')

    # Scanned overlay
    scanned_rgba_init = np.zeros((Ny, Nx, 4))
    scanned_im = ax.imshow(scanned_rgba_init, origin='lower',
                           extent=[0, AREA_SIZE, 0, AREA_SIZE], aspect='equal')

    # Drone markers
    drone_markers = []
    for d in drones:
        m, = ax.plot([], [], 'o', color=DRONE_COLORS[d.idx], markersize=9, zorder=10)
        drone_markers.append(m)

    # Drone trails
    drone_trails = []
    for d in drones:
        tr, = ax.plot([], [], '-', color=DRONE_COLORS[d.idx], linewidth=0.7, alpha=0.6, zorder=9)
        drone_trails.append(tr)

    # Survivor markers
    surv_markers = []
    for s in survivors:
        sm, = ax.plot(s[0], s[1], '*', color='gold', markersize=14,
                      markeredgecolor='black', markeredgewidth=0.8, zorder=11)
        surv_markers.append(sm)

    time_text = ax.text(0.02, 0.97, '', transform=ax.transAxes,
                        fontsize=10, va='top', color='black',
                        bbox=dict(boxstyle='round', facecolor='white', alpha=0.7))

    def init():
        zone_im.set_data(np.zeros((Ny, Nx, 4)))
        scanned_im.set_data(np.zeros((Ny, Nx, 4)))
        for m in drone_markers:
            m.set_data([], [])
        for tr in drone_trails:
            tr.set_data([], [])
        return [zone_im, scanned_im] + drone_markers + drone_trails + surv_markers

    def update(fi):
        frame_idx = frames[fi]
        t_frame = t_log[frame_idx]

        # Zone overlay
        snap_idx = get_snap_idx(t_frame)
        zone_map = zone_snapshots[snap_idx][1]
        zone_rgba = np.zeros((Ny, Nx, 4))
        for ci, (cx, cy) in enumerate(cell_centres):
            ix = int(cx / GRID_RES)
            iy = int(cy / GRID_RES)
            if 0 <= ix < Nx and 0 <= iy < Ny:
                k = zone_map[ci]
                if 0 <= k < K_DRONES:
                    zone_rgba[iy, ix] = mcolors.to_rgba(DRONE_COLORS[k], alpha=0.18)
        zone_im.set_data(zone_rgba)

        # Scanned overlay
        scanned_mask = scanned_frames[fi]
        scanned_rgba = np.zeros((Ny, Nx, 4))
        for ci, (cx, cy) in enumerate(cell_centres):
            ix = int(cx / GRID_RES)
            iy = int(cy / GRID_RES)
            if 0 <= ix < Nx and 0 <= iy < Ny and scanned_mask[ci]:
                scanned_rgba[iy, ix] = (0.6, 0.6, 0.6, 0.55)
        scanned_im.set_data(scanned_rgba)

        # Drone positions and trails
        for d in drones:
            step_now = min(frame_idx + 1, len(d.history))
            if step_now > 0:
                pos = d.history[step_now - 1]
                drone_markers[d.idx].set_data([pos[0]], [pos[1]])
                trail_start = max(0, step_now - 60)
                trail = np.array(d.history[trail_start:step_now])
                drone_trails[d.idx].set_data(trail[:, 0], trail[:, 1])
            else:
                drone_markers[d.idx].set_data([], [])

        # Survivor detected marker update
        for i, sm in enumerate(surv_markers):
            if detect_times[i] is not None and t_frame >= detect_times[i]:
                sm.set_color('limegreen')
                sm.set_markeredgecolor('darkgreen')

        time_text.set_text(f't = {t_frame:.1f} s')
        title_obj.set_text(f'S049: Dynamic Zone Search — {result["strategy"].capitalize()} (t={t_frame:.0f}s)')
        return [zone_im, scanned_im] + drone_markers + drone_trails + surv_markers + [time_text]

    ani = animation.FuncAnimation(fig, update, frames=len(frames),
                                  init_func=init, blit=False, interval=50)

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer='pillow', fps=20, dpi=100)
    plt.close()
    print(f'Saved: {path}')


# ── Main ───────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    out_dir = os.path.normpath(OUTPUT_DIR)

    print('Running Static Voronoi...')
    res_static = run_simulation('static')

    print('Running Equal-weight Voronoi...')
    res_equal = run_simulation('equal')

    print('Running Battery-weighted Voronoi...')
    res_weighted = run_simulation('weighted')

    results_dict = {
        'static': res_static,
        'equal':  res_equal,
        'weighted': res_weighted,
    }

    # Print key metrics
    print('\n=== Strategy Comparison ===')
    for strat, res in results_dict.items():
        print(f'\n[{strat.upper()}]')
        print(f'  Survivors found:       {res["n_found"]}/{N_SURVIVORS}')
        print(f'  Time to find all:      {res["time_all_found"]:.1f} s')
        print(f'  Mean detection time:   {res["mean_detect_time"]:.1f} s')
        print(f'  Final coverage:        {res["final_coverage"]*100:.1f}%')
        print(f'  Reallocation events:   {res["n_realloc"]}')
        print(f'  Mean Jain fairness:    {res["mean_fairness"]:.4f}')

    print('\nGenerating plots...')
    # Use weighted result for area map & snapshot (most informative)
    plot_search_area_map(res_weighted, out_dir)
    plot_reallocation_snapshots(res_weighted, out_dir)
    plot_trajectories(res_weighted, out_dir)
    plot_coverage_comparison(results_dict, out_dir)
    plot_survivor_detection_timeline(results_dict, out_dir)
    plot_jain_fairness(results_dict, out_dir)
    plot_summary_table(results_dict, out_dir)
    save_animation(res_weighted, out_dir)

    print('\nAll outputs saved to:', out_dir)
