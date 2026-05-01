"""
S043 Confined Space Exploration
================================
A rescue drone is deployed inside a collapsed building (30x30 m, 60x60 occupancy grid).
The drone starts with no map and builds one using a 360-degree lidar sensor while navigating
with three strategies: Random Walk (baseline), Frontier-Nearest (greedy BFS), and
Frontier + Bug2 (frontier selection with Bug2 local obstacle avoidance). Coverage
fraction of reachable free space is tracked over a 300-second mission horizon.

Usage:
    conda activate drones
    python src/03_environmental_sar/s043_confined_space.py
"""

import sys
import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.colors as mcolors
from collections import deque

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

# ── Parameters ─────────────────────────────────
GRID_RES      = 0.5          # m — cell side length
GRID_W        = 60           # cells (30 m wide)
GRID_H        = 60           # cells (30 m tall)
SENSOR_RADIUS = 2.0          # m
N_BEAMS       = 36           # lidar rays
SIGMA_RANGE   = 0.05         # m — range noise std dev
L_OCC         = 0.85         # log-odds increment for hit cell
L_FREE        = -0.40        # log-odds decrement for free cells
L_MIN         = -2.0         # log-odds clamp minimum
L_MAX         = 3.5          # log-odds clamp maximum
P_OCC_THRESH  = 0.65         # occupied probability threshold
P_FREE_THRESH = 0.35         # free probability threshold
G_MIN         = 0.35         # m — minimum passable gap
DRONE_SPEED   = 0.5          # m/s
D_TURN        = 0.4          # m — Bug2 obstacle detection distance
D_GOAL        = 0.3          # m — goal-reached tolerance
T_MAX         = 300.0        # s — mission horizon
DT            = 0.1          # s — simulation timestep

START_M = np.array([2.0, 2.0])   # m

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '03_environmental_sar', 's043_confined_space',
)

RNG = np.random.default_rng(0)   # fixed seed for reproducibility

# ── Helpers ────────────────────────────────────

def log_odds_to_prob(l):
    """Convert log-odds value(s) to probability."""
    return np.exp(l) / (1.0 + np.exp(l))


def world_to_cell(pos_m):
    """Convert world coordinates (m) to grid cell (row, col)."""
    col = int(pos_m[0] / GRID_RES)
    row = int(pos_m[1] / GRID_RES)
    return (int(np.clip(row, 0, GRID_H - 1)),
            int(np.clip(col, 0, GRID_W - 1)))


def cell_to_world(row, col):
    """Convert grid cell centre to world coordinates (m)."""
    return np.array([col * GRID_RES + GRID_RES / 2.0,
                     row * GRID_RES + GRID_RES / 2.0])


def build_ground_truth():
    """
    Create a 60x60 boolean occupancy grid representing a collapsed building.
    True = occupied (wall / debris).
    Layout: outer walls + inner rooms + narrow corridors + debris columns.
    """
    gt = np.zeros((GRID_H, GRID_W), dtype=bool)

    # Outer perimeter walls (1-cell thick)
    gt[0, :] = True
    gt[-1, :] = True
    gt[:, 0] = True
    gt[:, -1] = True

    # --- Horizontal internal walls with gaps ---
    # Wall at row 20 (y=10 m), gap at col 25-27
    gt[20, 5:24] = True
    gt[20, 28:55] = True

    # Wall at row 38 (y=19 m), gap at col 30-33
    gt[38, 10:29] = True
    gt[38, 34:50] = True

    # Wall at row 50 (y=25 m), gap at col 15-18
    gt[50, 2:14] = True
    gt[50, 19:45] = True

    # --- Vertical internal walls with gaps ---
    # Wall at col 25 (x=12.5 m), gap at row 10-13
    gt[2:9, 25] = True
    gt[14:20, 25] = True
    gt[21:38, 25] = True

    # Wall at col 40 (x=20 m), gap at row 28-31
    gt[2:27, 40] = True
    gt[32:50, 40] = True

    # Wall at col 12 (x=6 m), gap at row 40-43
    gt[21:39, 12] = True
    gt[44:58, 12] = True

    # --- Debris columns (2x2 blocks) ---
    debris_positions = [
        (8, 8), (8, 32), (8, 48),
        (28, 16), (28, 44),
        (42, 8), (42, 32), (42, 48),
        (54, 20), (54, 44),
        (15, 50), (30, 50),
    ]
    for r, c in debris_positions:
        gt[r:r+2, c:c+2] = True

    # --- Fallen beam: diagonal-ish debris ---
    for k in range(8):
        r, c = 6 + k, 16 + k
        if 0 <= r < GRID_H and 0 <= c < GRID_W:
            gt[r, c] = True
            if c + 1 < GRID_W:
                gt[r, c + 1] = True

    return gt


def cast_ray(ground_truth, pos_m, angle, r_max):
    """
    Ray-cast on ground truth grid.
    Returns (true_range, hit_cell_or_None).
    """
    dx = np.cos(angle)
    dy = np.sin(angle)
    step = GRID_RES / 2.0
    r = step
    while r <= r_max:
        pt = pos_m + r * np.array([dx, dy])
        row, col = world_to_cell(pt)
        if ground_truth[row, col]:
            return r, (row, col)
        r += step
    return r_max, None


def update_map(log_odds, ground_truth, pos_m):
    """
    Cast N_BEAMS rays, add Gaussian range noise, update log-odds map.
    Implements the inverse sensor model (binary Bayes).
    """
    angles = np.linspace(0, 2.0 * np.pi, N_BEAMS, endpoint=False)
    for angle in angles:
        true_range, hit_cell = cast_ray(ground_truth, pos_m, angle, SENSOR_RADIUS)
        noise = RNG.normal(0, SIGMA_RANGE)
        noisy_range = float(np.clip(true_range + noise, 0.0, SENSOR_RADIUS))

        dx = np.cos(angle)
        dy = np.sin(angle)
        step = GRID_RES / 2.0
        r = step
        # Mark free cells along beam up to (noisy_range - half_step)
        while r < noisy_range - step / 2.0:
            pt = pos_m + r * np.array([dx, dy])
            row, col = world_to_cell(pt)
            log_odds[row, col] = float(
                np.clip(log_odds[row, col] + L_FREE, L_MIN, L_MAX)
            )
            r += step

        # Mark hit cell as occupied
        if noisy_range < SENSOR_RADIUS and hit_cell is not None:
            ro, co = hit_cell
            log_odds[ro, co] = float(
                np.clip(log_odds[ro, co] + L_OCC, L_MIN, L_MAX)
            )


def extract_frontiers(prob_map):
    """
    Return list of (row, col) frontier cells.
    Frontier: free cell adjacent to at least one unknown cell.
    """
    free = prob_map < P_FREE_THRESH
    unkn = (prob_map >= P_FREE_THRESH) & (prob_map <= P_OCC_THRESH)
    frontiers = []
    for r in range(1, GRID_H - 1):
        for c in range(1, GRID_W - 1):
            if free[r, c]:
                if (unkn[r - 1, c] or unkn[r + 1, c] or
                        unkn[r, c - 1] or unkn[r, c + 1]):
                    frontiers.append((r, c))
    return frontiers


def bfs_nearest_frontier(prob_map, start_cell, frontiers):
    """
    BFS shortest path through free / unknown cells to the nearest frontier cell.
    Returns the frontier cell (row, col) or None if unreachable.
    """
    if not frontiers:
        return None
    frontier_set = set(frontiers)
    visited = set()
    queue = deque([start_cell])
    visited.add(start_cell)
    while queue:
        cell = queue.popleft()
        if cell in frontier_set:
            return cell
        r, c = cell
        for nr, nc in [(r - 1, c), (r + 1, c), (r, c - 1), (r, c + 1)]:
            if 0 <= nr < GRID_H and 0 <= nc < GRID_W and (nr, nc) not in visited:
                if prob_map[nr, nc] < P_OCC_THRESH:
                    visited.add((nr, nc))
                    queue.append((nr, nc))
    return None


def compute_reachable(ground_truth):
    """
    BFS from start position on ground-truth free cells.
    Returns set of reachable (row, col) cells.
    """
    start_cell = world_to_cell(START_M)
    gt_free = ~ground_truth
    reachable = set()
    visited = set()
    q = deque([start_cell])
    while q:
        cell = q.popleft()
        if cell in visited:
            continue
        visited.add(cell)
        r, c = cell
        if gt_free[r, c]:
            reachable.add(cell)
            for nr, nc in [(r - 1, c), (r + 1, c), (r, c - 1), (r, c + 1)]:
                if 0 <= nr < GRID_H and 0 <= nc < GRID_W:
                    q.append((nr, nc))
    return reachable


def compute_coverage(prob_map, reachable):
    """Fraction of reachable cells classified as free (in percent)."""
    count = 0
    for (r, c) in reachable:
        if prob_map[r, c] < P_FREE_THRESH:
            count += 1
    return count / max(len(reachable), 1) * 100.0


# ── Navigation helpers ─────────────────────────

def random_walk_step(pos_m, log_odds):
    """
    Random walk: pick a random angle; if it leads to a free cell, move there.
    Tries up to 36 random directions before giving up.
    """
    prob = log_odds_to_prob(log_odds)
    for _ in range(36):
        angle = RNG.uniform(0, 2 * np.pi)
        heading = np.array([np.cos(angle), np.sin(angle)])
        probe = pos_m + D_TURN * heading
        row, col = world_to_cell(probe)
        if prob[row, col] < P_OCC_THRESH:
            new_pos = pos_m + DRONE_SPEED * DT * heading
            return new_pos
    return pos_m  # stuck


def bug2_step(pos_m, goal_m, log_odds, mode, wall_dir):
    """
    Single Bug2 step.
    Returns (new_pos_m, new_mode, new_wall_dir).
    mode: 'MOTION_TO_GOAL' | 'BOUNDARY_FOLLOW' | 'REACHED'
    """
    prob = log_odds_to_prob(log_odds)
    to_goal = goal_m - pos_m
    dist_to_goal = np.linalg.norm(to_goal)
    if dist_to_goal < D_GOAL:
        return pos_m, 'REACHED', wall_dir

    if mode == 'MOTION_TO_GOAL':
        heading = to_goal / (dist_to_goal + 1e-9)
        probe = pos_m + D_TURN * heading
        pr, pc = world_to_cell(probe)
        if prob[pr, pc] > P_OCC_THRESH:
            # Hit obstacle — switch to boundary follow (right-hand rule)
            wall = np.array([-heading[1], heading[0]])
            return pos_m, 'BOUNDARY_FOLLOW', wall
        new_pos = pos_m + DRONE_SPEED * DT * heading
        return new_pos, 'MOTION_TO_GOAL', wall_dir

    else:  # BOUNDARY_FOLLOW
        heading = wall_dir / (np.linalg.norm(wall_dir) + 1e-9)
        probe = pos_m + D_TURN * heading
        pr, pc = world_to_cell(probe)
        if prob[pr, pc] > P_OCC_THRESH:
            # Turn right away from wall
            heading = np.array([heading[1], -heading[0]])
        # Check if back on M-line closer to goal
        new_pos = pos_m + DRONE_SPEED * DT * heading
        new_to_goal = np.linalg.norm(goal_m - new_pos)
        if new_to_goal < dist_to_goal - 0.1:
            return new_pos, 'MOTION_TO_GOAL', heading
        return new_pos, 'BOUNDARY_FOLLOW', heading


# ── Simulation ─────────────────────────────────

def run_strategy(ground_truth, reachable, strategy):
    """
    Run one exploration strategy.

    strategy: 'random' | 'frontier' | 'bug2'

    Returns:
        trajectory   : (N, 2) float array
        log_odds_hist: list of (row, col) snapshots at t=0,100,200,300
        coverage_curve: list of (t, coverage%) tuples
        mode_counts  : dict {'MOTION_TO_GOAL': int, 'BOUNDARY_FOLLOW': int}
    """
    log_odds = np.zeros((GRID_H, GRID_W), dtype=float)
    pos = START_M.copy()
    trajectory = [pos.copy()]
    coverage_curve = []
    mode_counts = {'MOTION_TO_GOAL': 0, 'BOUNDARY_FOLLOW': 0}

    goal = None
    mode = 'MOTION_TO_GOAL'
    wall_dir = np.array([1.0, 0.0])

    snapshot_times = {0, 100, 200, 300}
    snapshots = {}    # t_int -> log_odds copy

    steps = int(T_MAX / DT) + 1

    for step in range(steps):
        t = step * DT

        # Sensor update
        update_map(log_odds, ground_truth, pos)
        prob = log_odds_to_prob(log_odds)

        # Save snapshots
        t_int = int(round(t))
        if t_int in snapshot_times and t_int not in snapshots:
            snapshots[t_int] = (log_odds.copy(), pos.copy())

        # Coverage
        cov = compute_coverage(prob, reachable)
        coverage_curve.append((t, cov))

        if cov >= 99.5:
            # Fill remaining snapshots
            for ts in snapshot_times:
                if ts not in snapshots:
                    snapshots[ts] = (log_odds.copy(), pos.copy())
            break

        # --- Navigation ---
        if strategy == 'random':
            pos = random_walk_step(pos, log_odds)

        elif strategy == 'frontier':
            # Nearest frontier via BFS (no bug2)
            if goal is None or np.linalg.norm(pos - goal) < D_GOAL:
                frontiers = extract_frontiers(prob)
                sc = world_to_cell(pos)
                nearest = bfs_nearest_frontier(prob, sc, frontiers)
                if nearest is None:
                    break
                goal = cell_to_world(*nearest)
            # Move directly toward goal; if blocked choose new goal
            to_g = goal - pos
            dist = np.linalg.norm(to_g)
            heading = to_g / (dist + 1e-9)
            probe = pos + D_TURN * heading
            pr, pc = world_to_cell(probe)
            if prob[pr, pc] > P_OCC_THRESH:
                goal = None   # blocked — replan next step
            else:
                pos = pos + DRONE_SPEED * DT * heading
            mode_counts['MOTION_TO_GOAL'] += 1

        else:  # bug2
            if goal is None or np.linalg.norm(pos - goal) < D_GOAL or mode == 'REACHED':
                frontiers = extract_frontiers(prob)
                sc = world_to_cell(pos)
                nearest = bfs_nearest_frontier(prob, sc, frontiers)
                if nearest is None:
                    break
                goal = cell_to_world(*nearest)
                mode = 'MOTION_TO_GOAL'

            pos, mode, wall_dir = bug2_step(pos, goal, log_odds, mode, wall_dir)
            if mode in mode_counts:
                mode_counts[mode] += 1
            else:
                mode_counts['MOTION_TO_GOAL'] += 1

        trajectory.append(pos.copy())

    # Ensure all snapshots are filled
    for ts in snapshot_times:
        if ts not in snapshots:
            snapshots[ts] = (log_odds.copy(), pos.copy())

    return (np.array(trajectory), snapshots, coverage_curve, mode_counts)


def run_simulation():
    """Run all three strategies and return combined results."""
    ground_truth = build_ground_truth()
    reachable = compute_reachable(ground_truth)

    print(f"Ground-truth reachable free cells: {len(reachable)}")
    print(f"Running Random Walk strategy...")
    traj_rw, snap_rw, cov_rw, mode_rw = run_strategy(ground_truth, reachable, 'random')
    print(f"  Final coverage: {cov_rw[-1][1]:.1f}%  Steps: {len(traj_rw)}")

    print(f"Running Frontier-Nearest strategy...")
    traj_fn, snap_fn, cov_fn, mode_fn = run_strategy(ground_truth, reachable, 'frontier')
    print(f"  Final coverage: {cov_fn[-1][1]:.1f}%  Steps: {len(traj_fn)}")

    print(f"Running Frontier + Bug2 strategy...")
    traj_b2, snap_b2, cov_b2, mode_b2 = run_strategy(ground_truth, reachable, 'bug2')
    print(f"  Final coverage: {cov_b2[-1][1]:.1f}%  Steps: {len(traj_b2)}")

    return {
        'ground_truth': ground_truth,
        'reachable': reachable,
        'random':   {'traj': traj_rw, 'snap': snap_rw, 'cov': cov_rw, 'mode': mode_rw},
        'frontier': {'traj': traj_fn, 'snap': snap_fn, 'cov': cov_fn, 'mode': mode_fn},
        'bug2':     {'traj': traj_b2, 'snap': snap_b2, 'cov': cov_b2, 'mode': mode_b2},
    }


# ── Plots ──────────────────────────────────────

def _map_image(log_odds):
    """Convert log-odds grid to RGB image (free=white, unknown=grey, occupied=black)."""
    prob = log_odds_to_prob(log_odds)
    img = np.ones((GRID_H, GRID_W, 3))
    # unknown = grey (0.6)
    mask_unk = (prob >= P_FREE_THRESH) & (prob <= P_OCC_THRESH)
    img[mask_unk] = [0.6, 0.6, 0.6]
    # occupied = black
    mask_occ = prob > P_OCC_THRESH
    img[mask_occ] = [0.0, 0.0, 0.0]
    # free = white (already set)
    return img


def plot_map_evolution(data, out_dir):
    """Four map snapshots at t=0,100,200,300 for Bug2 strategy."""
    snap = data['bug2']['snap']
    times = [0, 100, 200, 300]
    fig, axes = plt.subplots(1, 4, figsize=(18, 5))
    fig.suptitle('S043 Confined Space Exploration — Map Evolution (Frontier + Bug2)',
                 fontsize=13, fontweight='bold')

    for ax, t in zip(axes, times):
        lo, pos = snap[t]
        prob = log_odds_to_prob(lo)
        img = _map_image(lo)

        # Highlight frontier cells in cyan
        frontiers = extract_frontiers(prob)
        for (r, c) in frontiers:
            img[r, c] = [0.0, 0.9, 0.9]

        ax.imshow(img, origin='lower', extent=[0, 30, 0, 30], aspect='equal')
        # Drone position
        ax.plot(pos[0], pos[1], 'ro', ms=6, zorder=5, label='Drone')
        ax.set_title(f't = {t} s')
        ax.set_xlabel('x (m)')
        ax.set_ylabel('y (m)')
        ax.set_xlim(0, 30)
        ax.set_ylim(0, 30)

    # Legend patches
    handles = [
        mpatches.Patch(color='white', ec='grey', label='Free'),
        mpatches.Patch(color='grey', label='Unknown'),
        mpatches.Patch(color='black', label='Occupied'),
        mpatches.Patch(color='cyan', label='Frontier'),
        plt.Line2D([0], [0], marker='o', color='r', ls='', ms=6, label='Drone'),
    ]
    fig.legend(handles=handles, loc='lower center', ncol=5,
               bbox_to_anchor=(0.5, -0.02))
    plt.tight_layout(rect=[0, 0.05, 1, 1])
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'map_evolution.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_trajectory_overlay(data, out_dir):
    """Final trajectory of Bug2 strategy overlaid on completed map."""
    snap = data['bug2']['snap']
    lo_final, _ = snap[300]
    traj = data['bug2']['traj']
    gt = data['ground_truth']

    img = _map_image(lo_final)

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.imshow(img, origin='lower', extent=[0, 30, 0, 30], aspect='equal')

    # Trajectory
    xs = traj[:, 0]
    ys = traj[:, 1]
    ax.plot(xs, ys, '-', color='royalblue', lw=1.0, alpha=0.7, label='Trajectory')
    ax.plot(xs[0], ys[0], 'go', ms=10, zorder=5, label='Start')
    ax.plot(xs[-1], ys[-1], 'rx', ms=10, mew=2, zorder=5, label='End')

    # Ground-truth occupied overlay (thin lines)
    for r in range(GRID_H):
        for c in range(GRID_W):
            if gt[r, c]:
                x0 = c * GRID_RES
                y0 = r * GRID_RES
                rect = plt.Rectangle((x0, y0), GRID_RES, GRID_RES,
                                     fc='none', ec='black', lw=0.3, alpha=0.3)
                ax.add_patch(rect)

    ax.set_xlim(0, 30)
    ax.set_ylim(0, 30)
    ax.set_title('S043 Final Trajectory — Frontier + Bug2', fontsize=13, fontweight='bold')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.legend(loc='upper right')

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'trajectory_overlay.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_coverage_curve(data, out_dir):
    """Coverage vs time for all three strategies."""
    fig, ax = plt.subplots(figsize=(10, 5))

    styles = {
        'random':   ('Random Walk',       'tab:red',    '--'),
        'frontier': ('Frontier-Nearest',  'tab:orange', '-.'),
        'bug2':     ('Frontier + Bug2',   'tab:blue',   '-'),
    }

    for key, (label, color, ls) in styles.items():
        cov = data[key]['cov']
        ts = [c[0] for c in cov]
        cs = [c[1] for c in cov]
        ax.plot(ts, cs, ls=ls, color=color, lw=2, label=label)

    ax.axhline(100, color='black', ls=':', lw=1.2, label='100% reachable')
    ax.set_xlim(0, T_MAX)
    ax.set_ylim(0, 105)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Coverage of reachable free space (%)')
    ax.set_title('S043 Coverage vs Time — Strategy Comparison', fontsize=13, fontweight='bold')
    ax.legend()
    ax.grid(True, alpha=0.3)

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'coverage_curve.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_mode_timeline(data, out_dir):
    """Stacked horizontal bar: time fraction in each Bug2 mode per strategy."""
    strategies = ['random', 'frontier', 'bug2']
    labels = ['Random Walk', 'Frontier-Nearest', 'Frontier + Bug2']

    fig, ax = plt.subplots(figsize=(9, 3))

    for i, (key, label) in enumerate(zip(strategies, labels)):
        mc = data[key]['mode']
        total = max(mc['MOTION_TO_GOAL'] + mc['BOUNDARY_FOLLOW'], 1)
        frac_mtg = mc['MOTION_TO_GOAL'] / total
        frac_bf  = mc['BOUNDARY_FOLLOW'] / total

        ax.barh(i, frac_mtg, color='steelblue', label='MOTION_TO_GOAL' if i == 2 else '')
        ax.barh(i, frac_bf, left=frac_mtg, color='goldenrod',
                label='BOUNDARY_FOLLOW' if i == 2 else '')
        ax.text(frac_mtg / 2, i, f'{frac_mtg*100:.0f}%',
                ha='center', va='center', color='white', fontsize=9, fontweight='bold')
        if frac_bf > 0.02:
            ax.text(frac_mtg + frac_bf / 2, i, f'{frac_bf*100:.0f}%',
                    ha='center', va='center', color='black', fontsize=9, fontweight='bold')

    ax.set_yticks(range(len(labels)))
    ax.set_yticklabels(labels)
    ax.set_xlim(0, 1)
    ax.set_xlabel('Fraction of steps')
    ax.set_title('S043 Navigation Mode Timeline', fontsize=13, fontweight='bold')
    ax.legend(loc='lower right')

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'mode_timeline.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def save_animation(data, out_dir):
    """
    Animate the Bug2 strategy: occupancy map + drone icon + frontier centroids.
    """
    import matplotlib.animation as animation

    snap = data['bug2']['snap']
    traj = data['bug2']['traj']
    gt   = data['ground_truth']
    reachable = data['reachable']

    # We need per-step log_odds — re-run a lightweight replay keeping every step
    log_odds = np.zeros((GRID_H, GRID_W), dtype=float)
    pos = START_M.copy()
    goal = None
    mode = 'MOTION_TO_GOAL'
    wall_dir = np.array([1.0, 0.0])

    # Decimate: record every 10th step for animation (0.1s * 10 = 1s per frame)
    FRAME_STEP = 10
    frames_lo = []
    frames_pos = []
    frames_front = []
    frames_traj = []

    MAX_STEPS = int(T_MAX / DT) + 1

    for step in range(MAX_STEPS):
        update_map(log_odds, gt, pos)
        prob = log_odds_to_prob(log_odds)

        if step % FRAME_STEP == 0:
            frames_lo.append(log_odds.copy())
            frames_pos.append(pos.copy())
            fronts = extract_frontiers(prob)
            frames_front.append(fronts[:50])   # limit for speed
            frames_traj.append(pos.copy())

        cov = compute_coverage(prob, reachable)
        if cov >= 99.5:
            break

        if goal is None or np.linalg.norm(pos - goal) < D_GOAL or mode == 'REACHED':
            frontiers = extract_frontiers(prob)
            sc = world_to_cell(pos)
            nearest = bfs_nearest_frontier(prob, sc, frontiers)
            if nearest is None:
                break
            goal = cell_to_world(*nearest)
            mode = 'MOTION_TO_GOAL'

        pos, mode, wall_dir = bug2_step(pos, goal, log_odds, mode, wall_dir)

    N_FRAMES = len(frames_lo)
    print(f'  Animation frames: {N_FRAMES}')

    # Build figure
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.set_xlim(0, 30)
    ax.set_ylim(0, 30)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title('S043 Confined Space Exploration (Bug2)', fontsize=11)

    img_obj = ax.imshow(
        np.ones((GRID_H, GRID_W, 3)),
        origin='lower', extent=[0, 30, 0, 30], aspect='equal', animated=True
    )
    drone_dot, = ax.plot([], [], 'ro', ms=7, zorder=6)
    path_line, = ax.plot([], [], '-', color='royalblue', lw=0.8, alpha=0.6, zorder=4)
    front_scat = ax.scatter([], [], c='cyan', s=8, zorder=5, alpha=0.7)

    time_text = ax.text(0.5, 28.5, '', ha='center', fontsize=9,
                        color='white', fontweight='bold',
                        bbox=dict(boxstyle='round', fc='black', alpha=0.5))

    traj_xs = [frames_pos[0][0]]
    traj_ys = [frames_pos[0][1]]

    def update(frame_idx):
        lo = frames_lo[frame_idx]
        pos_f = frames_pos[frame_idx]
        fronts = frames_front[frame_idx]

        img = _map_image(lo)
        img_obj.set_data(img)

        drone_dot.set_data([pos_f[0]], [pos_f[1]])

        traj_xs.append(pos_f[0])
        traj_ys.append(pos_f[1])
        path_line.set_data(traj_xs, traj_ys)

        if fronts:
            fxy = np.array([cell_to_world(r, c) for r, c in fronts])
            front_scat.set_offsets(fxy)
        else:
            front_scat.set_offsets(np.empty((0, 2)))

        t_sec = frame_idx * FRAME_STEP * DT
        time_text.set_text(f't = {t_sec:.0f} s')

        return img_obj, drone_dot, path_line, front_scat, time_text

    ani = animation.FuncAnimation(
        fig, update, frames=N_FRAMES, interval=80, blit=True
    )

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer='pillow', fps=12, dpi=90)
    plt.close()
    print(f'Saved: {path}')


if __name__ == '__main__':
    data = run_simulation()

    # Print key metrics
    print('\n--- Key Metrics ---')
    for key, label in [('random', 'Random Walk'), ('frontier', 'Frontier-Nearest'),
                        ('bug2', 'Frontier + Bug2')]:
        cov = data[key]['cov']
        final_cov = cov[-1][1]
        # Time to reach 50%
        t50 = next((c[0] for c in cov if c[1] >= 50.0), float('nan'))
        t80 = next((c[0] for c in cov if c[1] >= 80.0), float('nan'))
        mc = data[key]['mode']
        total_steps = max(mc['MOTION_TO_GOAL'] + mc['BOUNDARY_FOLLOW'], 1)
        bf_pct = mc['BOUNDARY_FOLLOW'] / total_steps * 100
        traj_len = len(data[key]['traj'])
        print(f'  [{label}]')
        print(f'    Final coverage:   {final_cov:.1f}%')
        print(f'    Time to 50% cov:  {t50:.1f} s')
        print(f'    Time to 80% cov:  {t80:.1f} s')
        print(f'    Boundary-follow:  {bf_pct:.1f}% of steps')
        print(f'    Total steps:      {traj_len}')

    reachable = data['reachable']
    print(f'\n  Reachable free cells: {len(reachable)}')

    out_dir = os.path.normpath(OUTPUT_DIR)
    plot_map_evolution(data, out_dir)
    plot_trajectory_overlay(data, out_dir)
    plot_coverage_curve(data, out_dir)
    plot_mode_timeline(data, out_dir)
    print('Saving animation (this may take a minute)...')
    save_animation(data, out_dir)
    print('\nAll outputs saved to:', out_dir)
