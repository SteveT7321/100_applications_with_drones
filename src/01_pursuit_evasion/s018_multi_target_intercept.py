"""
S018 Multi-Target Intercept
============================
A single pursuer must intercept N=4 stationary targets in minimum total time.
This is the Travelling Salesman Problem (TSP) with an initial position (open
path, no return).  Three strategies are compared: brute-force optimal (all
4! = 24 orderings), nearest-neighbour heuristic, and random ordering.  The
random baseline is a distribution: N_RANDOM seeded random permutations.

Targets are numbered T1-T4 as on the scenario card.

Usage:
    conda activate drones
    python src/01_pursuit_evasion/s018_multi_target_intercept.py
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  (registers 3D projection)
from matplotlib.patches import Patch
from itertools import permutations

# ── Parameters ─────────────────────────────────
V_PURSUER   = 5.0          # m/s  pursuer speed
CAPTURE_R   = 0.15         # m    capture radius per target
DT          = 0.05         # s    animation / trajectory sampling step
N_RANDOM    = 1000         # number of random orderings drawn
RANDOM_SEED = 0            # seed for the random orderings

START = np.array([-5., 0., 2.])   # pursuer start position (m)

TARGETS = np.array([
    [ 2.,  3., 2.],   # T1
    [ 4., -2., 2.],   # T2
    [-1., -3., 2.],   # T3
    [ 3.,  1., 2.],   # T4
])
N_TARGETS = len(TARGETS)

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '01_pursuit_evasion', 's018_multi_target_intercept',
)


# ── Helpers ────────────────────────────────────

def name(order):
    """Card numbering: (2, 1, 3, 0) -> 'T3-T2-T4-T1'."""
    return '-'.join(f'T{i + 1}' for i in order)


def segments(order):
    pts = [START] + [TARGETS[i] for i in order]
    return [float(np.linalg.norm(pts[k + 1] - pts[k])) for k in range(len(pts) - 1)]


def tour_time(order):
    """Total mission time: path length (start -> targets in order) / v_P."""
    return sum(segments(order)) / V_PURSUER


def nearest_neighbour():
    """Always fly to the closest remaining target."""
    remaining, pos, order = list(range(N_TARGETS)), START.copy(), []
    while remaining:
        nearest = min(remaining, key=lambda i: np.linalg.norm(TARGETS[i] - pos))
        order.append(nearest)
        pos = TARGETS[nearest]
        remaining.remove(nearest)
    return tuple(order)


def trajectory(order):
    """
    Positions sampled every DT along the straight-line path at constant speed,
    plus the time each target comes within CAPTURE_R.
    """
    pts = np.array([START] + [TARGETS[i] for i in order])
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    cum = np.concatenate([[0.0], np.cumsum(seg)])
    t_end = cum[-1] / V_PURSUER
    ts = np.append(np.arange(0.0, t_end, DT), t_end)
    s = ts * V_PURSUER
    k = np.clip(np.searchsorted(cum, s, side='right') - 1, 0, len(seg) - 1)
    frac = (s - cum[k]) / seg[k]
    pos = pts[k] + frac[:, None] * (pts[k + 1] - pts[k])
    capture_t = (cum[1:] - CAPTURE_R) / V_PURSUER
    return ts, pos, capture_t


# ── Simulation ─────────────────────────────────

def run_simulation():
    all_orders = list(permutations(range(N_TARGETS)))
    times = {o: tour_time(o) for o in all_orders}
    ranked = sorted(all_orders, key=lambda o: times[o])
    best, worst = ranked[0], ranked[-1]
    nn = nearest_neighbour()

    rng = np.random.default_rng(RANDOM_SEED)
    rand_orders = [tuple(int(x) for x in rng.permutation(N_TARGETS)) for _ in range(N_RANDOM)]
    rand_times = np.array([times[o] for o in rand_orders])
    counts = {o: 0 for o in all_orders}
    for o in rand_orders:
        counts[o] += 1

    t_best = times[best]
    gap = lambda t: (t - t_best) / t_best * 100.0

    print('=' * 64)
    print('S018 Multi-Target Intercept — Results')
    print('=' * 64)
    for label, o in [('Brute-force optimal', best), ('Nearest-neighbour', nn),
                     ('Worst case', worst)]:
        segs = ' + '.join(f'{d:.3f}' for d in segments(o))
        print(f'{label:20s} {name(o):12s} T = {times[o]:.3f} s  gap {gap(times[o]):5.1f}%  '
              f'dist = {segs} = {sum(segments(o)):.3f} m')
    print(f'Rank of NN ordering among 24: {ranked.index(nn) + 1}')
    all_t = np.array([times[o] for o in all_orders])
    print(f'\nAll 24 orderings: mean {all_t.mean():.3f} s (gap {gap(all_t.mean()):.1f}%), '
          f'median {np.median(all_t):.3f} s, min {all_t.min():.3f} s, max {all_t.max():.3f} s')
    print(f'Random baseline: {N_RANDOM} seeded permutations (seed {RANDOM_SEED})')
    print(f'  mean {rand_times.mean():.3f} s (gap {gap(rand_times.mean()):.1f}%), '
          f'std {rand_times.std():.3f} s, median {np.median(rand_times):.3f} s')
    p10, p90 = np.percentile(rand_times, [10, 90])
    print(f'  10th-90th percentile {p10:.3f}-{p90:.3f} s '
          f'(gap {gap(p10):.1f}-{gap(p90):.1f}%)')
    print(f'  draws that hit the optimal ordering: {counts[best]}/{N_RANDOM} '
          f'({counts[best] / N_RANDOM * 100:.1f}%; 1/24 = {100 / 24:.1f}%)')
    print(f'  draws with gap > 20%: {int(np.sum(rand_times > 1.2 * t_best))}/{N_RANDOM}')
    print(f'  first draw: {name(rand_orders[0])}  T = {rand_times[0]:.3f} s')

    trajs = {}
    for key, o in [('best', best), ('nn', nn), ('worst', worst), ('rand', rand_orders[0])]:
        ts, pos, cap = trajectory(o)
        trajs[key] = (o, ts, pos, cap)
    _, ts, _, cap = trajs['best']
    print(f'\nOptimal path sampled: arrival at last target {ts[-1]:.3f} s; '
          f'targets within capture radius at ' + ', '.join(f'{c:.3f}' for c in cap) + ' s')
    print('=' * 64)
    return {'times': times, 'ranked': ranked, 'best': best, 'worst': worst, 'nn': nn,
            'rand_times': rand_times, 'counts': counts, 'trajs': trajs,
            'all_mean': all_t.mean()}


# ── Plots ──────────────────────────────────────

STYLE = {'best': ('Brute-force optimal', 'steelblue'), 'nn': ('Nearest-neighbour', 'darkorange'),
         'worst': ('Worst case', 'crimson'), 'rand': ('Random (draw 1)', 'purple')}


def draw_scene(ax, order):
    ax.scatter(*START, color='red', s=70, marker='D', label='Pursuer start')
    for k, idx in enumerate(order):
        p = TARGETS[idx]
        ax.scatter(*p, color='green', s=90, marker='*')
        ax.text(p[0] + 0.15, p[1] + 0.15, p[2] + 0.05, f'T{idx + 1} (#{k + 1})', fontsize=7)
    ax.set_xlim(-6, 5.5); ax.set_ylim(-4.5, 4.5); ax.set_zlim(1.0, 3.0)
    ax.set_xlabel('X (m)', fontsize=7); ax.set_ylabel('Y (m)', fontsize=7)
    ax.set_zlabel('Z (m)', fontsize=7); ax.tick_params(labelsize=6)


def plot_trajectories(data, out_dir):
    fig = plt.figure(figsize=(15, 5))
    for col, key in enumerate(['best', 'nn', 'worst']):
        ax = fig.add_subplot(1, 3, col + 1, projection='3d')
        o, ts, pos, _ = data['trajs'][key]
        title, color = STYLE[key]
        ax.plot(pos[:, 0], pos[:, 1], pos[:, 2], color=color, lw=1.8)
        draw_scene(ax, o)
        ax.set_title(f'{title}: {name(o)}\nT = {data["times"][o]:.3f} s', fontsize=9)
    fig.suptitle('S018 Multi-Target Intercept — trajectories (#k = visit order)', fontsize=11, y=1.04)
    plt.tight_layout()
    path = os.path.join(out_dir, 'trajectories.png')
    plt.savefig(path, dpi=150, bbox_inches='tight'); plt.close(); print(f'Saved: {path}')


def plot_ordering_times(data, out_dir):
    ranked, times, counts = data['ranked'], data['times'], data['counts']
    colors = ['steelblue' if o == data['best'] else 'crimson' if o == data['worst'] else 'lightgray'
              for o in ranked]
    fig, ax = plt.subplots(figsize=(14, 5.5))
    x = np.arange(len(ranked))
    ax.bar(x, [times[o] for o in ranked], color=colors, edgecolor='black', lw=0.3)
    nn_i = ranked.index(data['nn'])
    ax.bar(nn_i, times[data['nn']], color='none', edgecolor='darkorange', lw=2.5, hatch='//')
    for i, o in enumerate(ranked):
        ax.text(i, times[o] + 0.05, f'{counts[o]}', ha='center', fontsize=7, color='purple')
    ax.set_xticks(x); ax.set_xticklabels([name(o) for o in ranked], rotation=90, fontsize=7)
    rt = data['rand_times']
    ax.axhline(times[data['best']], color='steelblue', ls='--', lw=1.2,
               label=f'Optimal = NN {times[data["best"]]:.3f} s')
    ax.axhline(rt.mean(), color='purple', ls='--', lw=1.2,
               label=f'Random mean {rt.mean():.3f} s ({N_RANDOM} draws)')
    ax.axhspan(*np.percentile(rt, [10, 90]), color='purple', alpha=0.08,
               label='Random 10th–90th percentile')
    ax.set_xlabel('Visit order'); ax.set_ylabel('Total mission time (s)')
    ax.set_title('S018 — all 24 orderings (purple number = times drawn by the random baseline)',
                 fontsize=11)
    ax.set_ylim(0, max(times.values()) * 1.15)
    handles, _ = ax.get_legend_handles_labels()
    handles += [Patch(facecolor='crimson', label=f'Worst {times[data["worst"]]:.3f} s'),
                Patch(facecolor='steelblue', edgecolor='darkorange', hatch='//',
                      label='Optimal (blue) = NN ordering (orange hatch)')]
    ax.legend(handles=handles, fontsize=8, loc='upper left')
    plt.tight_layout()
    path = os.path.join(out_dir, 'ordering_times.png')
    plt.savefig(path, dpi=150); plt.close(); print(f'Saved: {path}')


def plot_visit_table(data, out_dir):
    t_best = data['times'][data['best']]
    rt = data['rand_times']
    rows = []
    for label, o in [('Brute-force optimal', data['best']), ('Nearest-neighbour', data['nn']),
                     ('Worst case', data['worst'])]:
        segs = segments(o)
        rows.append([label, name(o), ' + '.join(f'{d:.2f}' for d in segs) + f' = {sum(segs):.2f}',
                     f'{data["times"][o]:.3f}', f'{(data["times"][o] - t_best) / t_best * 100:.1f}%'])
    rows.append([f'Random ({N_RANDOM} draws)', 'mean / median', '—',
                 f'{rt.mean():.3f} / {np.median(rt):.3f}',
                 f'{(rt.mean() - t_best) / t_best * 100:.1f}%'])
    fig, ax = plt.subplots(figsize=(12, 3))
    ax.axis('off')
    table = ax.table(cellText=rows, colLabels=['Strategy', 'Visit order', 'Segment distances (m)',
                                               'Total time (s)', 'Gap'],
                     cellLoc='center', loc='center')
    table.auto_set_font_size(False); table.set_fontsize(9); table.scale(1.1, 1.8)
    for j in range(5):
        table[0, j].set_facecolor('#2d6a9f'); table[0, j].set_text_props(color='white', fontweight='bold')
        table[1, j].set_facecolor('#d6eaf8')
    ax.set_title('S018 — visit order comparison', fontsize=11, pad=12)
    plt.tight_layout()
    path = os.path.join(out_dir, 'visit_table.png')
    plt.savefig(path, dpi=150, bbox_inches='tight'); plt.close(); print(f'Saved: {path}')


def save_animation(data, out_dir):
    keys = ['best', 'nn', 'worst', 'rand']
    fig = plt.figure(figsize=(15, 4.5))
    arts = []
    for col, key in enumerate(keys):
        ax = fig.add_subplot(1, 4, col + 1, projection='3d')
        o, ts, pos, _ = data['trajs'][key]
        title, color = STYLE[key]
        draw_scene(ax, o)
        line, = ax.plot([], [], [], color=color, lw=1.5)
        dot, = ax.plot([], [], [], 'o', color=color, ms=6)
        ax.set_title(f'{title}\n{name(o)}', fontsize=8)
        arts.append((ax, ts, pos, line, dot, title))
    n_frames = max(len(a[1]) for a in arts)
    step = max(1, int(np.ceil(n_frames / 150)))
    frames = list(range(0, n_frames, step)) + [n_frames - 1]

    def update(f):
        out = []
        for ax, ts, pos, line, dot, title in arts:
            i = min(f, len(ts) - 1)
            line.set_data(pos[:i + 1, 0], pos[:i + 1, 1]); line.set_3d_properties(pos[:i + 1, 2])
            dot.set_data([pos[i, 0]], [pos[i, 1]]); dot.set_3d_properties([pos[i, 2]])
            out += [line, dot]
        fig.suptitle(f'S018 Multi-Target Intercept   t = {min(f * DT, max(a[1][-1] for a in arts)):.2f} s',
                     fontsize=10)
        return out

    ani = animation.FuncAnimation(fig, update, frames=frames, blit=False, interval=50)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer='pillow', fps=20, dpi=80)
    plt.close(); print(f'Saved: {path}  ({len(frames)} frames)')


if __name__ == '__main__':
    out_dir = os.path.normpath(OUTPUT_DIR)
    os.makedirs(out_dir, exist_ok=True)
    data = run_simulation()
    plot_trajectories(data, out_dir)
    plot_ordering_times(data, out_dir)
    plot_visit_table(data, out_dir)
    save_animation(data, out_dir)
