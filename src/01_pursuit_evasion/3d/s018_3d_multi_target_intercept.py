"""
S018 3D Upgrade — Multi-Target Intercept
========================================
A single pursuer must visit 4 stationary targets at different altitudes.  The
pursuer has independent horizontal (v_xy) and vertical (v_z) speed limits, so the
time of a leg is max(d_xy / v_xy, |dz| / v_z).  Compared orderings:
  * brute-force optimum over all 4! = 24 orderings,
  * standard nearest-neighbour (3D Euclidean distance),
  * altitude-aware nearest-neighbour (d_xy + w_z |dz|, w_z = v_xy / v_z),
  * a random ordering, as a seeded distribution of 1000 draws (S018 2D fix).

Every tour is also flown step by step with the card's decoupled x-y / z motion
to check the analytic leg times.  The card's v_z sweep shows how the heuristics'
optimality gaps depend on the vertical speed limit, and 500 seeded random
target layouts check whether the card layout is representative.

Usage:
    conda activate drones
    python src/01_pursuit_evasion/3d/s018_3d_multi_target_intercept.py
"""
import os
from itertools import permutations
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Patch

OUTPUT_DIR = os.path.normpath(os.path.join(
    os.path.dirname(__file__), '..', '..', '..',
    'outputs', '01_pursuit_evasion', '3d', 's018_3d_multi_target_intercept'))

# ── Parameters (card) ─────────────────────────────────────────────────────────
V_XY    = 5.0
V_Z_MAX = 2.5
W_Z     = V_XY / V_Z_MAX   # 2.0
EPS_XY  = 0.2
EPS_Z   = 0.1
DT      = 0.05
T_LEG_MAX = 20.0
Z_MIN, Z_MAX = 0.5, 5.5

START = np.array([-5.0, 0.0, 2.5])
TARGETS = np.array([
    [2.0,  3.0, 1.0],    # T1 low
    [4.0, -2.0, 4.5],    # T2 high
    [-1.0, -3.0, 0.8],   # T3 very low
    [3.0,  1.0, 3.5],    # T4 mid-high
])
N_T = len(TARGETS)
LABELS = ['T1', 'T2', 'T3', 'T4']
TARGET_COLORS = ['#2a9d8f', '#e76f51', '#8ab17d', '#e9c46a']

N_RANDOM    = 1000     # random-ordering baseline draws
RANDOM_SEED = 0
VZ_SWEEP    = [0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 4.0, 5.0, 7.5, 10.0, 20.0, 1e9]
N_LAYOUTS   = 500      # random target layouts
LAYOUT_SEED = 18
LAYOUT_BOX  = ((-2.0, 5.0), (-4.0, 4.0), (Z_MIN, Z_MAX))


def name(order):
    return '-'.join(LABELS[i] for i in order)


# ── Travel-time model ─────────────────────────────────────────────────────────

def leg_time(a, b, v_z=V_Z_MAX):
    return max(np.linalg.norm(b[:2] - a[:2]) / V_XY, abs(b[2] - a[2]) / v_z)


def tour_time(order, start=START, targets=TARGETS, v_z=V_Z_MAX):
    pts = [start] + [targets[i] for i in order]
    return sum(leg_time(pts[k], pts[k + 1], v_z) for k in range(len(order)))


def nn_order(start, targets, metric):
    rem, pos, order = list(range(len(targets))), start, []
    while rem:
        j = min(rem, key=lambda i: metric(pos, targets[i]))
        order.append(j); pos = targets[j]; rem.remove(j)
    return tuple(order)


def nn_standard(start=START, targets=TARGETS):
    return nn_order(start, targets, lambda a, b: np.linalg.norm(b - a))


def nn_altitude_aware(start=START, targets=TARGETS, w_z=W_Z):
    return nn_order(start, targets,
                    lambda a, b: np.linalg.norm(b[:2] - a[:2]) + w_z * abs(b[2] - a[2]))


def brute_force(start=START, targets=TARGETS, v_z=V_Z_MAX):
    return sorted((tour_time(p, start, targets, v_z), p)
                  for p in permutations(range(len(targets))))


# ── Step-by-step flight (card's decoupled x-y / z motion) ────────────────────

def fly_tour(order):
    pos = START.copy()
    traj, visit_t, t = [pos.copy()], [], 0.0
    for idx in order:
        tgt = TARGETS[idx]
        for _ in range(int(T_LEG_MAX / DT)):
            dxy_vec = tgt[:2] - pos[:2]
            dxy = np.linalg.norm(dxy_vec)
            if dxy > 1e-12:
                pos[:2] = pos[:2] + dxy_vec / dxy * min(V_XY * DT, dxy)
            pos[2] = pos[2] + np.clip(tgt[2] - pos[2], -V_Z_MAX * DT, V_Z_MAX * DT)
            t += DT
            traj.append(pos.copy())
            if np.linalg.norm(tgt[:2] - pos[:2]) < EPS_XY and abs(tgt[2] - pos[2]) < EPS_Z:
                visit_t.append(t)
                break
    return np.array(traj), visit_t


# ── Experiments ───────────────────────────────────────────────────────────────

def leg_table(order):
    pts = [START] + [TARGETS[i] for i in order]
    rows = []
    for k in range(len(order)):
        a, b = pts[k], pts[k + 1]
        txy = np.linalg.norm(b[:2] - a[:2]) / V_XY
        tz = abs(b[2] - a[2]) / V_Z_MAX
        rows.append({'to': LABELS[order[k]], 'dxy': np.linalg.norm(b[:2] - a[:2]), 'dz': b[2] - a[2],
                     'txy': txy, 'tz': tz, 't': max(txy, tz), 'limit': 'z' if tz > txy else 'xy'})
    return rows


def run_card_layout():
    ranked = brute_force()
    t_best, best = ranked[0]
    t_worst, worst = ranked[-1]
    nn, nna = nn_standard(), nn_altitude_aware()
    rank = {p: k + 1 for k, (_, p) in enumerate(ranked)}
    strategies = {'Brute-force optimal': best, 'NN standard': nn, 'NN altitude-aware': nna,
                  'Worst case': worst}
    res = {}
    print('Card layout (v_xy 5 m/s, v_z 2.5 m/s):')
    for lbl, o in strategies.items():
        T = tour_time(o)
        traj, vt = fly_tour(o)
        zs = [START[2]] + [TARGETS[i][2] for i in o]
        res[lbl] = {'order': o, 'time': T, 'traj': traj, 'visit_t': vt, 'legs': leg_table(o)}
        print(f'  {lbl:20s} | {name(o):11s} | T = {T:.3f} s | gap {100 * (T - t_best) / t_best:5.1f}% | '
              f'rank {rank[o]:2d}/24 | z path {" -> ".join(f"{z:.1f}" for z in zs)} | '
              f'flown {vt[-1]:.2f} s')
        for leg in res[lbl]['legs']:
            print(f'      -> {leg["to"]}: d_xy {leg["dxy"]:.3f} m ({leg["txy"]:.3f} s), dz {leg["dz"]:+.1f} m '
                  f'({leg["tz"]:.3f} s) -> {leg["t"]:.3f} s, {leg["limit"]}-limited')
    rng = np.random.default_rng(RANDOM_SEED)
    rand = [tuple(int(x) for x in rng.permutation(N_T)) for _ in range(N_RANDOM)]
    rt = np.array([tour_time(o) for o in rand])
    all_t = np.array([t for t, _ in ranked])
    counts = {p: 0 for _, p in ranked}
    for o in rand:
        counts[o] += 1
    print(f'  Random baseline ({N_RANDOM} draws, seed {RANDOM_SEED}): mean {rt.mean():.3f} s '
          f'(gap {100 * (rt.mean() - t_best) / t_best:.1f}%), std {rt.std():.3f}, median {np.median(rt):.3f}, '
          f'10-90% {np.percentile(rt, 10):.3f}-{np.percentile(rt, 90):.3f} s; '
          f'exact mean over 24 orderings {all_t.mean():.3f} s; optimum drawn {counts[best]}/{N_RANDOM}; '
          f'gap > 20% in {int(np.sum(rt > 1.2 * t_best))}/{N_RANDOM}')
    flat = brute_force(v_z=1e9)
    print(f'  Ignoring altitude (v_z -> inf): optimum {name(flat[0][1])} {flat[0][0]:.3f} s; '
          f'that ordering with v_z 2.5: {tour_time(flat[0][1]):.3f} s')
    return {'ranked': ranked, 'res': res, 'rand_times': rt, 'counts': counts, 'rank': rank}


def run_vz_sweep():
    """Optimality gap of both NN variants vs v_z: card layout and random layouts."""
    rng = np.random.default_rng(LAYOUT_SEED)
    layouts = [np.column_stack([rng.uniform(*LAYOUT_BOX[d], N_T) for d in range(3)])
               for _ in range(N_LAYOUTS)]
    out = {'card_std': [], 'card_alt': [], 'mc_std': [], 'mc_alt': [], 'mc_std_opt': [],
           'mc_alt_opt': [], 'mc_alt_better': [], 'mc_std_better': []}
    print(f'\nv_z sweep (w_z = v_xy / v_z). Card layout, and {N_LAYOUTS} random layouts '
          f'(seed {LAYOUT_SEED}; targets x{LAYOUT_BOX[0]} y{LAYOUT_BOX[1]} z{LAYOUT_BOX[2]}):')
    print('    v_z  | card: opt  NN-std gap  NN-alt gap | random: NN-std mean gap (opt found) | '
          'NN-alt mean gap (opt found) | alt better / std better')
    for vz in VZ_SWEEP:
        wz = V_XY / vz
        tb = brute_force(v_z=vz)[0][0]
        gs = 100 * (tour_time(nn_standard(), v_z=vz) - tb) / tb
        ga = 100 * (tour_time(nn_altitude_aware(w_z=wz), v_z=vz) - tb) / tb
        s_g, a_g = [], []
        for L in layouts:
            b = brute_force(START, L, vz)[0][0]
            s_g.append(100 * (tour_time(nn_standard(START, L), START, L, vz) - b) / b)
            a_g.append(100 * (tour_time(nn_altitude_aware(START, L, wz), START, L, vz) - b) / b)
        s_g, a_g = np.array(s_g), np.array(a_g)
        for k, v in [('card_std', gs), ('card_alt', ga), ('mc_std', s_g.mean()), ('mc_alt', a_g.mean()),
                     ('mc_std_opt', np.mean(s_g < 1e-9)), ('mc_alt_opt', np.mean(a_g < 1e-9)),
                     ('mc_alt_better', int(np.sum(a_g < s_g - 1e-9))),
                     ('mc_std_better', int(np.sum(s_g < a_g - 1e-9)))]:
            out[k].append(v)
        vz_s = 'inf' if vz > 1e6 else f'{vz:.1f}'
        print(f'  {vz_s:>5s}  | {tb:6.3f}s  {gs:6.1f}%     {ga:6.1f}%    | {s_g.mean():6.2f}% '
              f'({100 * np.mean(s_g < 1e-9):4.1f}%)           | {a_g.mean():6.2f}% ({100 * np.mean(a_g < 1e-9):4.1f}%)'
              f'          | {int(np.sum(a_g < s_g - 1e-9))} / {int(np.sum(s_g < a_g - 1e-9))}')
    return out


def run_simulation():
    print('S018 3D Multi-Target Intercept — Simulation Results')
    print('=' * 70)
    return {'card': run_card_layout(), 'sweep': run_vz_sweep()}


# ── Plots ─────────────────────────────────────────────────────────────────────

PLOT_KEYS = ['Brute-force optimal', 'NN standard', 'NN altitude-aware', 'Worst case']
PLOT_COLORS = {'Brute-force optimal': '#2a9d8f', 'NN standard': '#f4a261',
               'NN altitude-aware': '#8338ec', 'Worst case': '#e63946'}


def draw_scene(ax):
    ax.scatter(*START, color='red', s=80, marker='D', label='Pursuer start')
    for i, t in enumerate(TARGETS):
        ax.scatter(*t, color='green', s=120, marker='*', edgecolors='black')
        ax.text(t[0] + 0.2, t[1] + 0.2, t[2] + 0.2, f'{LABELS[i]} z={t[2]:.1f}', fontsize=7)
    ax.set_xlim(-6, 5); ax.set_ylim(-4, 4); ax.set_zlim(0, 6)
    ax.set_xlabel('X', fontsize=7); ax.set_ylabel('Y', fontsize=7); ax.set_zlabel('Z', fontsize=7)
    ax.tick_params(labelsize=6)


def plot_trajectories_3d(card, out_dir):
    fig = plt.figure(figsize=(18, 13))
    for k, key in enumerate(PLOT_KEYS):
        r = card['res'][key]
        ax = fig.add_subplot(2, 2, k + 1, projection='3d')
        draw_scene(ax)
        tr = r['traj']
        ax.plot(tr[:, 0], tr[:, 1], tr[:, 2], color=PLOT_COLORS[key], lw=2, label='flown path')
        ax.plot(tr[:, 0], tr[:, 1], np.zeros(len(tr)), color='grey', lw=0.8, ls=':', label='ground track')
        ax.set_title(f'{key}: {name(r["order"])}\nT = {r["time"]:.3f} s (analytic), '
                     f'{r["visit_t"][-1]:.2f} s (flown)', fontsize=9)
        ax.legend(fontsize=7, loc='upper left')
    fig.suptitle('S018 3D — tours with decoupled horizontal (5 m/s) and vertical (2.5 m/s) motion',
                 fontsize=11)
    plt.tight_layout()
    path = os.path.join(out_dir, 'trajectories_3d.png')
    plt.savefig(path, dpi=120); plt.close(); print(f'Saved: {path}')


def plot_altitude_profile(card, out_dir):
    fig, ax = plt.subplots(figsize=(12, 5.5))
    for key in PLOT_KEYS:
        r = card['res'][key]
        t = np.arange(len(r['traj'])) * DT
        opt = key == 'Brute-force optimal'
        ax.plot(t, r['traj'][:, 2], color=PLOT_COLORS[key], lw=5 if opt else 2, alpha=0.5 if opt else 1,
                ls='-' if opt or key != 'NN altitude-aware' else '--',
                label=f'{key} {name(r["order"])} ({r["visit_t"][-1]:.2f} s)')
        for vt in r['visit_t']:
            ax.axvline(vt, color=PLOT_COLORS[key], lw=0.5, ls=':')
    for i, t in enumerate(TARGETS):
        ax.axhline(t[2], color='grey', lw=0.6, ls='--')
        ax.text(0.02, t[2] + 0.05, f'{LABELS[i]} z={t[2]}', fontsize=7, color='grey')
    ax.set_xlabel('Time (s)'); ax.set_ylabel('Pursuer altitude z (m)'); ax.set_ylim(0, 6)
    ax.set_title('S018 3D — pursuer altitude vs time (dotted = target visit)', fontsize=10)
    ax.legend(fontsize=8, loc='upper right'); ax.grid(True, alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'altitude_profile.png')
    plt.savefig(path, dpi=130); plt.close(); print(f'Saved: {path}')


def plot_all_orderings(card, out_dir):
    ranked = card['ranked']
    times = [t for t, _ in ranked]
    fig, ax1 = plt.subplots(figsize=(15, 6))
    cols = []
    for _, p in ranked:
        c = '#adb5bd'
        for key in ['Worst case', 'NN altitude-aware', 'NN standard', 'Brute-force optimal']:
            if card['res'][key]['order'] == p:
                c = PLOT_COLORS[key]
        cols.append(c)
    x = np.arange(len(ranked))
    ax1.bar(x, times, color=cols, edgecolor='black')
    rt = card['rand_times']
    ax1.axhline(rt.mean(), color='black', ls='--', lw=1.2,
                label=f'Random ordering mean {rt.mean():.3f} s ({N_RANDOM} draws)')
    ax1.set_xticks(x); ax1.set_xticklabels([name(p) for _, p in ranked], rotation=60, fontsize=7, ha='right')
    ax1.set_ylabel('Tour time (s)'); ax1.set_ylim(0, max(times) * 1.15)
    ax2 = ax1.twinx()
    ax2.plot(x, [card['counts'][p] for _, p in ranked], 'ko-', ms=4, lw=0.8, label='random draws per ordering')
    ax2.set_ylabel('Random draws (of 1000)'); ax2.set_ylim(0, 120)
    h1, l1 = ax1.get_legend_handles_labels(); h2, l2 = ax2.get_legend_handles_labels()
    best = card['res']['Brute-force optimal']['order']
    for key in PLOT_KEYS:
        same = key != 'Brute-force optimal' and card['res'][key]['order'] == best
        h1.append(Patch(facecolor=PLOT_COLORS[key], edgecolor='black'))
        l1.append(key + (' (= optimal, bar shown in optimal colour)' if same else ''))
    ax1.legend(h1 + h2, l1 + l2, fontsize=8, loc='upper left')
    ax1.set_title('S018 3D — all 24 orderings ranked by altitude-aware tour time', fontsize=10)
    ax1.grid(True, alpha=0.3, axis='y')
    plt.tight_layout()
    path = os.path.join(out_dir, 'all_orderings.png')
    plt.savefig(path, dpi=130); plt.close(); print(f'Saved: {path}')


def plot_visit_table(card, out_dir):
    rows, colors = [], []
    for key in PLOT_KEYS:
        r = card['res'][key]
        for k, leg in enumerate(r['legs']):
            rows.append([key if k == 0 else '', f'{k + 1}: {leg["to"]}', f'{leg["dxy"]:.2f}',
                         f'{leg["dz"]:+.1f}', f'{leg["txy"]:.3f}', f'{leg["tz"]:.3f}',
                         f'{leg["t"]:.3f}', leg['limit']])
            colors.append(['white'] * 7 + ['#f4a3a8' if leg['limit'] == 'z' else '#b7e4c7'])
        rows.append(['', 'total', '', '', '', '', f'{r["time"]:.3f}', ''])
        colors.append(['#e9ecef'] * 8)
    fig, ax = plt.subplots(figsize=(13, 8))
    tb = ax.table(cellText=rows, colLabels=['Strategy', 'Leg', 'd_xy (m)', 'dz (m)', 'd_xy/v_xy (s)',
                                            '|dz|/v_z (s)', 'leg time (s)', 'limit'],
                  cellColours=colors, loc='center', cellLoc='center')
    tb.auto_set_font_size(False); tb.set_fontsize(8); tb.scale(1.0, 1.25)
    ax.axis('off')
    ax.set_title('S018 3D — legs per strategy: leg time = max(horizontal, vertical); '
                 'red = vertical-limited leg', fontsize=10)
    plt.tight_layout()
    path = os.path.join(out_dir, 'visit_table.png')
    plt.savefig(path, dpi=130); plt.close(); print(f'Saved: {path}')


def plot_vz_sweep(sweep, out_dir):
    vz = np.array([v if v < 1e6 else 40.0 for v in VZ_SWEEP])
    fig, axes = plt.subplots(1, 2, figsize=(15, 5.5))
    ax = axes[0]
    ax.plot(vz, sweep['card_std'], 'o-', color='#f4a261', label='NN standard')
    ax.plot(vz, sweep['card_alt'], 's-', color='#8338ec', label='NN altitude-aware')
    ax.set_title('Card layout', fontsize=10)
    ax = axes[1]
    ax.plot(vz, sweep['mc_std'], 'o-', color='#f4a261', label='NN standard (mean)')
    ax.plot(vz, sweep['mc_alt'], 's-', color='#8338ec', label='NN altitude-aware (mean)')
    ax.set_title(f'{N_LAYOUTS} random layouts (mean gap)', fontsize=10)
    for ax in axes:
        ax.set_xscale('log'); ax.axvline(V_Z_MAX, color='grey', ls='--', lw=1, label='card v_z = 2.5 m/s')
        ax.set_xticks(vz); ax.set_xticklabels([f'{v:g}' if v < 39 else 'inf' for v in vz], fontsize=7)
        ax.set_xlabel('Vertical speed limit v_z (m/s)'); ax.set_ylabel('Optimality gap (%)')
        ax.grid(True, alpha=0.3); ax.legend(fontsize=8)
    fig.suptitle('S018 3D — heuristic optimality gap vs vertical speed limit (w_z = v_xy / v_z)', fontsize=11)
    plt.tight_layout()
    path = os.path.join(out_dir, 'vz_sweep.png')
    plt.savefig(path, dpi=130); plt.close(); print(f'Saved: {path}')


def save_animation(card, out_dir):
    """Optimal, NN standard and NN altitude-aware tours flown side by side."""
    keys = PLOT_KEYS[:3]
    runs = [card['res'][k] for k in keys]
    max_len = max(len(r['traj']) for r in runs)
    step = 2
    frames = int(np.ceil((max_len - 1) / step)) + 1   # last frame = final sample, no duplicate
    fig = plt.figure(figsize=(16, 6))
    arts = []
    for k, (key, r) in enumerate(zip(keys, runs)):
        ax = fig.add_subplot(1, 3, k + 1, projection='3d')
        draw_scene(ax)
        ln, = ax.plot([], [], [], color=PLOT_COLORS[key], lw=2)
        dot, = ax.plot([], [], [], 'o', color='red', mec='black', ms=7)
        arts.append((ax, key, r, ln, dot))

    def update(f):
        si = min(f * step, max_len - 1)
        out = []
        for ax, key, r, ln, dot in arts:
            n = min(si + 1, len(r['traj']))
            tr = r['traj'][:n]
            ln.set_data_3d(tr[:, 0], tr[:, 1], tr[:, 2])
            dot.set_data_3d([tr[-1, 0]], [tr[-1, 1]], [tr[-1, 2]])
            done = sum(vt <= (n - 1) * DT + 1e-9 for vt in r['visit_t'])
            ax.set_title(f'{key} {name(r["order"])}\nt = {(n - 1) * DT:.2f} s, visited {done}/4', fontsize=9)
            out += [ln, dot]
        return out

    ani = FuncAnimation(fig, update, frames=frames, interval=60, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer=PillowWriter(fps=16), dpi=70)
    plt.close(); print(f'Saved: {path}  ({frames} frames)')


if __name__ == '__main__':
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    data = run_simulation()
    plot_trajectories_3d(data['card'], OUTPUT_DIR)
    plot_altitude_profile(data['card'], OUTPUT_DIR)
    plot_all_orderings(data['card'], OUTPUT_DIR)
    plot_visit_table(data['card'], OUTPUT_DIR)
    plot_vz_sweep(data['sweep'], OUTPUT_DIR)
    save_animation(data['card'], OUTPUT_DIR)
    print('Done.')
