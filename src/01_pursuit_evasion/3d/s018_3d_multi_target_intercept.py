"""
S018 3D Upgrade — Multi-Target Intercept
Usage:
    conda activate drones
    python src/01_pursuit_evasion/3d/s018_3d_multi_target_intercept.py
"""
import sys, os, numpy as np, matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa
from matplotlib.animation import FuncAnimation, PillowWriter
from itertools import permutations
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..'))
from src.base.drone_base import DroneBase
OUTPUT_DIR = os.path.join(os.path.dirname(__file__), '..', '..', '..', 'outputs', '01_pursuit_evasion', '3d', 's018_3d_multi_target_intercept')
os.makedirs(OUTPUT_DIR, exist_ok=True)

# ── Parameters ────────────────────────────────────────────────────────────────
V_XY    = 5.0
V_Z_MAX = 2.0
W_Z     = V_XY / V_Z_MAX   # = 2.5 altitude penalty weight
EPS_XY  = 0.2
EPS_Z   = 0.1
DT      = 0.05
T_MAX   = 30.0

START = np.array([-5., 0., 2.])

TARGETS_3D = np.array([
    [ 2.,  3., 4.0],   # T0 high
    [ 4., -2., 1.0],   # T1 low
    [-1., -3., 3.0],   # T2 mid
    [ 3.,  1., 5.0],   # T3 very high
])

TARGET_LABELS  = ['T0 (high)', 'T1 (low)', 'T2 (mid)', 'T3 (very high)']
TARGET_COLORS  = ['#e63946', '#2a9d8f', '#f4a261', '#8338ec']


# ── Travel time metrics ───────────────────────────────────────────────────────

def travel_time_3d(p1, p2, v_xy=V_XY, v_z=V_Z_MAX):
    """Altitude-aware travel time: bottleneck of horizontal vs vertical."""
    dxy = np.linalg.norm(p2[:2] - p1[:2])
    dz  = abs(p2[2] - p1[2])
    return max(dxy / v_xy, dz / v_z)


def travel_time_flat(p1, p2, v_xy=V_XY):
    """Flat (2D) travel time — ignores altitude."""
    dxy = np.linalg.norm(p2[:2] - p1[:2])
    return dxy / v_xy


def dist_alt_aware(a, b, w_z=W_Z):
    dxy = np.linalg.norm(b[:2] - a[:2])
    dz  = abs(b[2] - a[2])
    return dxy + w_z * dz


def tour_time(order, start=START, targets=TARGETS_3D, fn=travel_time_3d):
    pts = [start] + [targets[i] for i in order]
    return sum(fn(pts[k], pts[k + 1]) for k in range(len(pts) - 1))


# ── Ordering algorithms ───────────────────────────────────────────────────────

def brute_force_optimal():
    """Compare all 24 orderings with altitude-aware travel time."""
    all_perms = list(permutations(range(len(TARGETS_3D))))
    times = [(tour_time(p), p) for p in all_perms]
    times.sort(key=lambda x: x[0])
    return times  # sorted list of (time, order)


def nearest_neighbor_3d(start, targets):
    """Standard 3D Euclidean nearest neighbor."""
    remaining = list(range(len(targets)))
    pos = start.copy()
    order = []
    while remaining:
        nearest = min(remaining, key=lambda i: np.linalg.norm(targets[i] - pos))
        order.append(nearest)
        pos = targets[nearest]
        remaining.remove(nearest)
    return tuple(order)


def nearest_neighbor_alt_aware(start, targets):
    """Altitude-aware nearest neighbor."""
    remaining = list(range(len(targets)))
    pos = start.copy()
    order = []
    while remaining:
        nearest = min(remaining, key=lambda i: dist_alt_aware(pos, targets[i]))
        order.append(nearest)
        pos = targets[nearest]
        remaining.remove(nearest)
    return tuple(order)


# ── Simulate tour ─────────────────────────────────────────────────────────────

def simulate_tour_3d(order, start=START, targets=TARGETS_3D, v_xy=V_XY, v_z=V_Z_MAX, dt=DT):
    """Simulate pursuer following a 3D tour with decoupled XY/Z motion."""
    pos = start.copy().astype(float)
    traj = [pos.copy()]
    visit_times = []
    t = 0.0

    for target_idx in order:
        target = targets[target_idx].copy()
        max_steps = int(T_MAX / dt)
        for step in range(max_steps):
            dxy_vec = target[:2] - pos[:2]
            dxy = np.linalg.norm(dxy_vec)
            dz  = target[2] - pos[2]

            # XY motion
            if dxy > EPS_XY:
                pos[:2] = pos[:2] + v_xy * dt * dxy_vec / (dxy + 1e-8)

            # Z motion (limited rate)
            dz_step = np.clip(dz, -v_z * dt, v_z * dt)
            pos[2] = pos[2] + dz_step

            t += dt
            traj.append(pos.copy())

            if dxy < EPS_XY and abs(dz) < EPS_Z:
                visit_times.append(t)
                break

    return np.array(traj), visit_times


# ── Run all orderings ─────────────────────────────────────────────────────────

def run_simulation():
    print('S018 3D Multi-Target Intercept — Simulation Results')
    print('=' * 60)

    sorted_times = brute_force_optimal()
    best_time,  best_order  = sorted_times[0]
    worst_time, worst_order = sorted_times[-1]

    nn_order    = nearest_neighbor_3d(START, TARGETS_3D)
    nn_alt_order= nearest_neighbor_alt_aware(START, TARGETS_3D)
    nn_time     = tour_time(nn_order)
    nn_alt_time = tour_time(nn_alt_order)

    print(f'  Best order:    {best_order}  time={best_time:.2f}s')
    print(f'  Worst order:   {worst_order}  time={worst_time:.2f}s')
    print(f'  NN standard:   {nn_order}  time={nn_time:.2f}s  gap={100*(nn_time-best_time)/best_time:.1f}%')
    print(f'  NN alt-aware:  {nn_alt_order}  time={nn_alt_time:.2f}s  gap={100*(nn_alt_time-best_time)/best_time:.1f}%')

    # Flat vs 3D comparison for all orderings
    all_perms = [p for _, p in sorted_times]
    times_3d   = [tour_time(p, fn=travel_time_3d)  for p in all_perms]
    times_flat = [tour_time(p, fn=travel_time_flat) for p in all_perms]

    # Simulate trajectories for key orderings
    traj_best,  vt_best  = simulate_tour_3d(best_order)
    traj_nn,    vt_nn    = simulate_tour_3d(nn_order)
    traj_worst, vt_worst = simulate_tour_3d(worst_order)

    return {
        'sorted_times': sorted_times,
        'best':  {'order': best_order,  'time': best_time,  'traj': traj_best,  'visit_times': vt_best},
        'nn':    {'order': nn_order,    'time': nn_time,    'traj': traj_nn,    'visit_times': vt_nn},
        'worst': {'order': worst_order, 'time': worst_time, 'traj': traj_worst, 'visit_times': vt_worst},
        'all_perms': all_perms,
        'times_3d':   times_3d,
        'times_flat': times_flat,
        'nn_alt_order': nn_alt_order,
        'nn_alt_time':  nn_alt_time,
    }


# ── Plot 1: Trajectories 3D — best, nearest-neighbor, worst ──────────────────

def plot_trajectories_3d(data, out_dir):
    cases = [
        ('Best Order',       data['best'],  '#2a9d8f'),
        ('Nearest Neighbor', data['nn'],    '#f4a261'),
        ('Worst Order',      data['worst'], '#e63946'),
    ]

    fig = plt.figure(figsize=(18, 6))

    for idx, (case_label, res, traj_color) in enumerate(cases):
        ax = fig.add_subplot(1, 3, idx + 1, projection='3d')
        traj = res['traj']

        # Plot trajectory
        ax.plot(traj[:, 0], traj[:, 1], traj[:, 2], color=traj_color,
                linewidth=2.0, alpha=0.85, label='Pursuer path')

        # Start point
        ax.scatter(*START, color='black', s=100, marker='D', zorder=10, label='Start')

        # Target points
        for i, (t, lbl, tc) in enumerate(zip(TARGETS_3D, TARGET_LABELS, TARGET_COLORS)):
            ax.scatter(*t, color=tc, s=120, marker='*', zorder=10, label=lbl)
            ax.text(t[0] + 0.15, t[1] + 0.15, t[2] + 0.15,
                    f'{lbl}\nz={t[2]:.1f}m', fontsize=6, color=tc)

        # Draw order arrows
        order = res['order']
        pts = [START] + [TARGETS_3D[i] for i in order]
        for k in range(len(pts) - 1):
            a, b = pts[k], pts[k + 1]
            mid = (a + b) / 2
            ax.quiver(a[0], a[1], a[2], b[0]-a[0], b[1]-a[1], b[2]-a[2],
                      length=0.6, normalize=True, color='black', alpha=0.5, arrow_length_ratio=0.3)

        ax.set_xlim(-7, 6); ax.set_ylim(-5, 5); ax.set_zlim(0, 7)
        ax.set_xlabel('X', fontsize=7); ax.set_ylabel('Y', fontsize=7); ax.set_zlabel('Z', fontsize=7)
        ax.set_title(f'{case_label}\nOrder: {list(res["order"])}  t={res["time"]:.2f}s',
                     fontsize=8, fontweight='bold')
        ax.legend(fontsize=6, loc='upper left')
        ax.tick_params(labelsize=6)

    fig.suptitle('S018 3D Multi-Target Intercept — Trajectory Comparison\n'
                 'Best vs Nearest-Neighbor vs Worst ordering (altitude-aware travel time)',
                 fontsize=11, fontweight='bold')
    plt.tight_layout()
    path = os.path.join(out_dir, 'trajectories_3d.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


# ── Plot 2: All 24 orderings bar chart ───────────────────────────────────────

def plot_all_orderings(data, out_dir):
    sorted_times = data['sorted_times']
    times_3d = data['times_3d']
    best_time = sorted_times[0][0]
    worst_time = sorted_times[-1][0]
    nn_time = data['nn']['time']
    nn_alt_time = data['nn_alt_time']

    fig, ax = plt.subplots(figsize=(16, 6))
    n = len(times_3d)
    x = np.arange(n)
    colors = ['#2a9d8f' if t == best_time else ('#e63946' if t == worst_time else '#adb5bd')
              for t in times_3d]

    ax.bar(x, times_3d, color=colors, alpha=0.85, edgecolor='none')

    # Mark NN and NN alt-aware
    nn_idx = next((i for i, (_, p) in enumerate(sorted_times) if tuple(p) == data['nn']['order']), None)
    nn_alt_idx = next((i for i, (_, p) in enumerate(sorted_times) if tuple(p) == data['nn_alt_order']), None)
    if nn_idx is not None:
        ax.bar(nn_idx, times_3d[nn_idx], color='#f4a261', alpha=0.9, label=f'NN standard ({nn_time:.2f}s)')
    if nn_alt_idx is not None:
        ax.bar(nn_alt_idx, times_3d[nn_alt_idx], color='#8338ec', alpha=0.9, label=f'NN alt-aware ({nn_alt_time:.2f}s)')

    ax.axhline(best_time,  color='#2a9d8f', linestyle='--', linewidth=1.5, label=f'Best ({best_time:.2f}s)')
    ax.axhline(worst_time, color='#e63946', linestyle='--', linewidth=1.5, label=f'Worst ({worst_time:.2f}s)')

    # Label order indices on x axis for a few
    order_labels = ['→'.join(str(i) for i in p) for _, p in sorted_times]
    tick_step = max(1, n // 8)
    ax.set_xticks(x[::tick_step])
    ax.set_xticklabels(order_labels[::tick_step], fontsize=7, rotation=45, ha='right')

    ax.set_xlabel('Visit Ordering (sorted by altitude-aware time)', fontsize=10)
    ax.set_ylabel('Total Mission Time (s)', fontsize=10)
    ax.set_title('S018 3D Multi-Target Intercept — All 24 Orderings (Altitude-Aware)\n'
                 'Green = best  |  Red = worst  |  Orange = NN standard  |  Purple = NN alt-aware',
                 fontsize=10, fontweight='bold')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3, axis='y')
    plt.tight_layout()
    path = os.path.join(out_dir, 'all_orderings.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


# ── Plot 3: Flat vs 3D travel time comparison ─────────────────────────────────

def plot_flat_vs_3d(data, out_dir):
    times_3d   = data['times_3d']
    times_flat = data['times_flat']
    n = len(times_3d)
    x = np.arange(n)

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 6))

    # Scatter: flat vs 3D for all orderings
    ax1.scatter(times_flat, times_3d, alpha=0.6, color='#457b9d', s=50, edgecolors='black', linewidths=0.5)
    min_t = min(min(times_flat), min(times_3d)) - 0.1
    max_t = max(max(times_flat), max(times_3d)) + 0.1
    ax1.plot([min_t, max_t], [min_t, max_t], 'k--', linewidth=1.2, alpha=0.5, label='3D = Flat (reference)')
    ax1.set_xlabel('Flat (2D) Estimated Time (s)', fontsize=10)
    ax1.set_ylabel('Altitude-Aware 3D Time (s)', fontsize=10)
    ax1.set_title('Flat vs 3D Travel Time\n(each point = one of 24 orderings)', fontsize=10, fontweight='bold')
    ax1.legend(fontsize=8)
    ax1.grid(True, alpha=0.3)

    # Bar comparison: flat vs 3D sorted by 3D time
    ax2.bar(x - 0.2, times_flat, 0.4, label='Flat (2D) estimate', color='#e9c46a', alpha=0.8)
    ax2.bar(x + 0.2, times_3d,   0.4, label='3D altitude-aware',   color='#457b9d', alpha=0.8)
    ax2.set_xlabel('Ordering index (sorted by 3D time)', fontsize=10)
    ax2.set_ylabel('Mission Time (s)', fontsize=10)
    ax2.set_title('Flat vs 3D Comparison — All 24 Orderings\n(sorted by 3D time)', fontsize=10, fontweight='bold')
    ax2.legend(fontsize=8)
    ax2.grid(True, alpha=0.3, axis='y')

    fig.suptitle('S018 3D Multi-Target Intercept — Flat vs Altitude-Aware Travel Time',
                 fontsize=11, fontweight='bold')
    plt.tight_layout()
    path = os.path.join(out_dir, 'flat_vs_3d.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


# ── Plot 4: Animation — best ordering ────────────────────────────────────────

def save_animation(data, out_dir):
    traj = data['best']['traj']
    order = data['best']['order']
    max_len = len(traj)
    step_size = 3

    fig = plt.figure(figsize=(9, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim(-7, 6); ax.set_ylim(-5, 5); ax.set_zlim(0, 7)
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')

    # Static: targets and start
    ax.scatter(*START, color='black', s=120, marker='D', zorder=10, label='Start')
    for i, (t, lbl, tc) in enumerate(zip(TARGETS_3D, TARGET_LABELS, TARGET_COLORS)):
        ax.scatter(*t, color=tc, s=150, marker='*', zorder=10, label=lbl)
        ax.text(t[0] + 0.1, t[1] + 0.1, t[2] + 0.1, f'T{i}', fontsize=8, color=tc, fontweight='bold')

    # Visit order annotation
    pts = [START] + [TARGETS_3D[i] for i in order]
    for k in range(len(pts) - 1):
        a, b = pts[k], pts[k + 1]
        ax.plot([a[0], b[0]], [a[1], b[1]], [a[2], b[2]], 'k--', lw=0.8, alpha=0.3)

    path_line, = ax.plot([], [], [], color='#2a9d8f', lw=2.0, alpha=0.9, label='Pursuer')
    dot,       = ax.plot([], [], [], color='#2a9d8f', marker='o', ms=10, ls='',
                         zorder=10, markeredgecolor='black', markeredgewidth=0.8)
    title_txt = ax.set_title('')
    ax.legend(fontsize=7, loc='upper left')

    n_frames = max_len // step_size

    def update(frame):
        si = min(frame * step_size, max_len - 1)
        path_line.set_data_3d(traj[:si+1, 0], traj[:si+1, 1], traj[:si+1, 2])
        dot.set_data_3d([traj[si, 0]], [traj[si, 1]], [traj[si, 2]])
        title_txt.set_text(f'S018 3D — Best Order {list(order)}   t={si * DT:.2f}s')
        return path_line, dot, title_txt

    ani = FuncAnimation(fig, update, frames=n_frames, interval=50, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer=PillowWriter(fps=20), dpi=100)
    plt.close()
    print(f'Saved: {path}')


# ── Main ──────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    data = run_simulation()
    plot_trajectories_3d(data, OUTPUT_DIR)
    plot_all_orderings(data, OUTPUT_DIR)
    plot_flat_vs_3d(data, OUTPUT_DIR)
    save_animation(data, OUTPUT_DIR)
    print('Done.')
