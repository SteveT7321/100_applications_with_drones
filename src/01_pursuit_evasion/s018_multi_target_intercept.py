"""
S018 Multi-Target Intercept
============================
A single pursuer must intercept N=4 stationary targets in minimum total time.
This is the Travelling Salesman Problem (TSP) with an initial position.
Three strategies are compared: brute-force optimal (all 4!=24 orderings),
nearest-neighbour heuristic, and random ordering (baseline).

Usage:
    conda activate drones
    python src/01_pursuit_evasion/s018_multi_target_intercept.py
"""

import sys, os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from itertools import permutations

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

# ── Parameters ─────────────────────────────────
V_PURSUER   = 5.0          # m/s  pursuer speed
CAPTURE_R   = 0.15         # m    capture radius per target
DT          = 0.05         # s    simulation timestep

START = np.array([-5., 0., 2.])   # pursuer start position (m)

TARGETS = np.array([
    [ 2.,  3., 2.],   # Target 0
    [ 4., -2., 2.],   # Target 1
    [-1., -3., 2.],   # Target 2
    [ 3.,  1., 2.],   # Target 3
])

N_TARGETS = len(TARGETS)

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '01_pursuit_evasion', 's018_multi_target_intercept',
)

RNG = np.random.default_rng(0)   # fixed seed for reproducibility


# ── Helpers ────────────────────────────────────

def tour_time(order, start, targets, v):
    """Compute total mission time for a given target visit order."""
    pts = [start] + [targets[i] for i in order]
    total_dist = sum(np.linalg.norm(pts[k+1] - pts[k]) for k in range(len(pts)-1))
    return total_dist / v


def nearest_neighbour(start, targets):
    """Nearest-neighbour heuristic: always fly to closest remaining target."""
    remaining = list(range(len(targets)))
    pos = start.copy()
    order = []
    while remaining:
        nearest = min(remaining, key=lambda i: np.linalg.norm(targets[i] - pos))
        order.append(nearest)
        pos = targets[nearest]
        remaining.remove(nearest)
    return tuple(order)


def simulate_trajectory(order, start, targets, v, dt):
    """Simulate drone flying through targets in given order. Returns trajectory array."""
    trajectory = [start.copy()]
    pos = start.copy()
    for idx in order:
        goal = targets[idx].copy()
        while True:
            diff = goal - pos
            dist = np.linalg.norm(diff)
            if dist <= CAPTURE_R:
                trajectory.append(goal.copy())
                pos = goal.copy()
                break
            step = min(v * dt, dist)
            pos = pos + (diff / dist) * step
            trajectory.append(pos.copy())
    return np.array(trajectory)


# ── Simulation ─────────────────────────────────

def run_simulation():
    """Run brute-force, nearest-neighbour, and random ordering. Return all data."""

    # ── Brute-force optimal: try all N! orderings
    all_orders = list(permutations(range(N_TARGETS)))
    times = [(tour_time(order, START, TARGETS, V_PURSUER), order) for order in all_orders]
    times.sort(key=lambda x: x[0])

    best_time, best_order = times[0]
    worst_time, worst_order = times[-1]
    all_times = [t for t, _ in times]
    all_orders_sorted = [o for _, o in times]

    # ── Nearest-neighbour heuristic
    nn_order = nearest_neighbour(START, TARGETS)
    nn_time = tour_time(nn_order, START, TARGETS, V_PURSUER)

    # ── Random ordering (fixed seed baseline)
    rand_indices = list(range(N_TARGETS))
    RNG.shuffle(rand_indices)
    rand_order = tuple(rand_indices)
    rand_time = tour_time(rand_order, START, TARGETS, V_PURSUER)

    # ── Optimality gap
    gap_nn   = (nn_time   - best_time) / best_time * 100.0
    gap_rand = (rand_time - best_time) / best_time * 100.0

    # ── Simulate trajectories for key orderings
    traj_best  = simulate_trajectory(best_order,  START, TARGETS, V_PURSUER, DT)
    traj_nn    = simulate_trajectory(nn_order,    START, TARGETS, V_PURSUER, DT)
    traj_worst = simulate_trajectory(worst_order, START, TARGETS, V_PURSUER, DT)
    traj_rand  = simulate_trajectory(rand_order,  START, TARGETS, V_PURSUER, DT)

    return {
        'best_time':   best_time,
        'best_order':  best_order,
        'worst_time':  worst_time,
        'worst_order': worst_order,
        'nn_time':     nn_time,
        'nn_order':    nn_order,
        'rand_time':   rand_time,
        'rand_order':  rand_order,
        'gap_nn':      gap_nn,
        'gap_rand':    gap_rand,
        'all_times':   all_times,
        'all_orders':  all_orders_sorted,
        'traj_best':   traj_best,
        'traj_nn':     traj_nn,
        'traj_worst':  traj_worst,
        'traj_rand':   traj_rand,
    }


# ── Plots ──────────────────────────────────────

def plot_trajectories(data, out_dir):
    """3D trajectory comparison: optimal, NN, worst-case orderings."""
    fig = plt.figure(figsize=(14, 5))

    configs = [
        ('traj_best',  'best_order',  'Brute-Force Optimal', 'steelblue',  'best_time'),
        ('traj_nn',    'nn_order',    'Nearest-Neighbour',    'darkorange', 'nn_time'),
        ('traj_worst', 'worst_order', 'Worst Case',           'crimson',    'worst_time'),
    ]

    for col, (tkey, okey, title, color, ttime) in enumerate(configs):
        ax = fig.add_subplot(1, 3, col+1, projection='3d')
        traj  = data[tkey]
        order = data[okey]
        t     = data[ttime]

        ax.plot(traj[:, 0], traj[:, 1], traj[:, 2],
                color=color, lw=1.5, alpha=0.85, label='Path')

        # Start
        ax.scatter(*START, color='green', s=80, marker='D', zorder=5, label='Start')

        # Targets with visit order labels
        for k, idx in enumerate(order):
            pos = TARGETS[idx]
            ax.scatter(*pos, color='red', s=60, marker='*', zorder=5)
            ax.text(pos[0]+0.1, pos[1]+0.1, pos[2]+0.1,
                    f'T{idx}(#{k+1})', fontsize=7)

        ax.set_title(f'{title}\nT = {t:.2f} s', fontsize=9)
        ax.set_xlabel('X (m)', fontsize=7)
        ax.set_ylabel('Y (m)', fontsize=7)
        ax.set_zlabel('Z (m)', fontsize=7)

        margin = 1.5
        ax.set_xlim(TARGETS[:, 0].min() - margin, TARGETS[:, 0].max() + margin)
        ax.set_ylim(TARGETS[:, 1].min() - margin, TARGETS[:, 1].max() + margin)
        ax.set_zlim(1.0, 3.0)
        ax.tick_params(labelsize=6)

    fig.suptitle('S018 Multi-Target Intercept — Trajectory Comparison', fontsize=11, y=1.01)
    plt.tight_layout()

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'trajectories.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_ordering_times(data, out_dir):
    """Bar chart of all 24 orderings with highlights for optimal, NN, and random."""
    all_times  = data['all_times']
    all_orders = data['all_orders']
    best_time  = data['best_time']
    best_order = data['best_order']
    nn_time    = data['nn_time']
    nn_order   = data['nn_order']
    rand_time  = data['rand_time']
    rand_order = data['rand_order']

    fig, ax = plt.subplots(figsize=(13, 5))

    colors = []
    for o, t in zip(all_orders, all_times):
        if o == best_order or o == tuple(best_order):
            colors.append('steelblue')
        elif o == nn_order:
            colors.append('darkorange')
        elif o == rand_order:
            colors.append('purple')
        else:
            colors.append('lightgray')

    x = np.arange(len(all_times))
    bars = ax.bar(x, all_times, color=colors, edgecolor='none', width=0.8)

    # x-tick labels: order string
    labels = [str(list(o)) for o in all_orders]
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=90, fontsize=6)

    # Reference lines
    ax.axhline(best_time, color='steelblue', linestyle='--', lw=1.2, label=f'Optimal {best_time:.2f} s')
    ax.axhline(nn_time,   color='darkorange', linestyle='--', lw=1.2, label=f'NN {nn_time:.2f} s')
    ax.axhline(rand_time, color='purple',     linestyle='--', lw=1.2, label=f'Random {rand_time:.2f} s')

    ax.set_xlabel('Visit Order (target indices)', fontsize=10)
    ax.set_ylabel('Total Mission Time (s)',        fontsize=10)
    ax.set_title('S018 — All 24 Orderings: Mission Time Comparison', fontsize=11)
    ax.legend(fontsize=9)
    ax.set_ylim(0, max(all_times) * 1.12)

    plt.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'ordering_times.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def plot_visit_table(data, out_dir):
    """Summary table: visit order, segment distances, total time for key strategies."""
    strategies = [
        ('Brute-Force Optimal', data['best_order'],  data['best_time']),
        ('Nearest-Neighbour',   data['nn_order'],    data['nn_time']),
        ('Random Baseline',     data['rand_order'],  data['rand_time']),
        ('Worst Case',          data['worst_order'], data['worst_time']),
    ]

    fig, ax = plt.subplots(figsize=(10, 4))
    ax.axis('off')

    col_labels = ['Strategy', 'Visit Order', 'Seg. Distances (m)', 'Total Time (s)']
    rows = []
    for name, order, t in strategies:
        pts = [START] + [TARGETS[i] for i in order]
        segs = [f'{np.linalg.norm(pts[k+1]-pts[k]):.2f}' for k in range(len(pts)-1)]
        rows.append([name, str(list(order)), ' → '.join(segs), f'{t:.3f}'])

    table = ax.table(cellText=rows, colLabels=col_labels,
                     cellLoc='center', loc='center')
    table.auto_set_font_size(False)
    table.set_fontsize(9)
    table.scale(1.1, 1.8)

    # Colour header
    for j in range(len(col_labels)):
        table[0, j].set_facecolor('#2d6a9f')
        table[0, j].set_text_props(color='white', fontweight='bold')
    # Colour optimal row
    for j in range(len(col_labels)):
        table[1, j].set_facecolor('#d6eaf8')

    ax.set_title('S018 — Visit Order Comparison Table', fontsize=11, pad=12)
    plt.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'visit_table.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def save_animation(data, out_dir):
    """Animate the pursuer following each of the four strategy trajectories."""
    import matplotlib.animation as animation

    trajs = [
        (data['traj_best'],  data['best_order'],  'Optimal',        'steelblue'),
        (data['traj_nn'],    data['nn_order'],    'Nearest-Neighbour', 'darkorange'),
        (data['traj_worst'], data['worst_order'], 'Worst Case',     'crimson'),
        (data['traj_rand'],  data['rand_order'],  'Random',         'purple'),
    ]

    # Use longest trajectory length, decimate for speed
    max_len = max(len(t[0]) for t in trajs)
    step = max(1, max_len // 150)

    fig = plt.figure(figsize=(13, 4))
    axes = []
    lines = []
    dots  = []

    for col, (traj, order, title, color) in enumerate(trajs):
        ax = fig.add_subplot(1, 4, col+1, projection='3d')
        axes.append(ax)

        line, = ax.plot([], [], [], color=color, lw=1.5)
        dot,  = ax.plot([], [], [], 'o', color=color, ms=6)
        lines.append(line)
        dots.append(dot)

        ax.scatter(*START, color='green', s=60, marker='D', zorder=5)
        for k, idx in enumerate(order):
            pos = TARGETS[idx]
            ax.scatter(*pos, color='red', s=50, marker='*', zorder=5)
            ax.text(pos[0]+0.1, pos[1]+0.1, pos[2]+0.1, f'T{idx}', fontsize=6)

        ax.set_title(title, fontsize=8)
        ax.set_xlabel('X', fontsize=6); ax.set_ylabel('Y', fontsize=6); ax.set_zlabel('Z', fontsize=6)
        margin = 1.5
        ax.set_xlim(TARGETS[:, 0].min()-margin, TARGETS[:, 0].max()+margin)
        ax.set_ylim(TARGETS[:, 1].min()-margin, TARGETS[:, 1].max()+margin)
        ax.set_zlim(1.0, 3.0)
        ax.tick_params(labelsize=5)

    # Pre-index for each trajectory (padded to same frame count)
    n_frames = max(len(t[0][::step]) for t in trajs)

    def init():
        for line, dot in zip(lines, dots):
            line.set_data([], [])
            line.set_3d_properties([])
            dot.set_data([], [])
            dot.set_3d_properties([])
        return lines + dots

    def update(frame):
        for i, (traj, _, _, _) in enumerate(trajs):
            sub = traj[::step]
            f = min(frame, len(sub)-1)
            lines[i].set_data(sub[:f+1, 0], sub[:f+1, 1])
            lines[i].set_3d_properties(sub[:f+1, 2])
            dots[i].set_data([sub[f, 0]], [sub[f, 1]])
            dots[i].set_3d_properties([sub[f, 2]])
        return lines + dots

    ani = animation.FuncAnimation(
        fig, update, frames=n_frames, init_func=init, blit=False, interval=50)

    fig.suptitle('S018 Multi-Target Intercept — Strategy Animation', fontsize=10)
    plt.tight_layout()

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer='pillow', fps=20, dpi=100)
    plt.close()
    print(f'Saved: {path}')


if __name__ == '__main__':
    data = run_simulation()

    # ── Print key metrics
    print('=' * 55)
    print('S018 Multi-Target Intercept — Results')
    print('=' * 55)
    print(f"Brute-Force Optimal  order: {list(data['best_order'])}  T = {data['best_time']:.3f} s")
    print(f"Nearest-Neighbour    order: {list(data['nn_order'])}  T = {data['nn_time']:.3f} s  (gap {data['gap_nn']:.1f}%)")
    print(f"Random Baseline      order: {list(data['rand_order'])}  T = {data['rand_time']:.3f} s  (gap {data['gap_rand']:.1f}%)")
    print(f"Worst Case           order: {list(data['worst_order'])}  T = {data['worst_time']:.3f} s")
    print(f"Total orderings evaluated: {len(data['all_times'])} (4! = 24)")
    print(f"Optimal total distance: {data['best_time'] * 5.0:.3f} m")
    print('=' * 55)

    out_dir = os.path.normpath(OUTPUT_DIR)
    plot_trajectories(data, out_dir)
    plot_ordering_times(data, out_dir)
    plot_visit_table(data, out_dir)
    save_animation(data, out_dir)
