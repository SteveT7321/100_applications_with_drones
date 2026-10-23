"""
S017 Swarm vs Swarm
===================
M pursuers compete against N evaders in an open 2D arena: 3v3 (symmetric) and
5v3 (pursuer advantage).  Pursuers fly pure pursuit at their assigned evader;
evaders scatter away from the centroid of all pursuers.  When an evader is
captured, every pursuer that was chasing it switches to the nearest remaining
evader.

Three initial assignment methods are compared:
  greedy          — pursuers in index order each take the nearest unclaimed
                    evader; once every evader is claimed, the surplus pursuers
                    take the nearest evader (shared).
  hungarian_dummy — the card's method: Hungarian on the distance matrix padded
                    with dummy columns; pursuers left on a dummy column have no
                    assignment and chase the nearest evader.
  hungarian_aug   — Hungarian on an augmented matrix in which every evader
                    column is repeated ceil(M/N) times, so the surplus pursuers
                    are placed by the same sum-of-distances optimisation.
For M = N the two Hungarian variants are identical.

The card geometry is symmetric, so the methods are also compared on seeded
random start positions (Monte Carlo).

Usage:
    conda activate drones
    python src/01_pursuit_evasion/s017_swarm_vs_swarm.py
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Patch
from scipy.optimize import linear_sum_assignment

# ── Parameters ──────────────────────────────────────────────────────────────
V_PURSUER = 5.0    # m/s — pursuer speed
V_EVADER  = 3.5    # m/s — evader speed
CAPTURE_R = 0.15   # m   — capture radius
DT        = 0.02   # s   — timestep
T_MAX     = 30.0   # s   — max simulation time

# Card geometry — Case A: 3 pursuers vs 3 evaders
PURSUERS_3 = np.array([[-5.0, -2.0], [-5.0,  0.0], [-5.0,  2.0]], dtype=float)
EVADERS_3  = np.array([[ 0.0, -2.0], [ 0.0,  0.0], [ 0.0,  2.0]], dtype=float)
# Case B: 5 pursuers vs the same 3 evaders
PURSUERS_5 = np.array([[-5.0, -3.0], [-5.0, -1.5], [-5.0,  0.0],
                       [-5.0,  1.5], [-5.0,  3.0]], dtype=float)

# Monte Carlo: random starts (pursuers left, evaders right)
MC_TRIALS   = 200
MC_SEED     = 17
MC_P_BOX    = ((-7.0, -3.0), (-4.0, 4.0))   # x-range, y-range for pursuers
MC_E_BOX    = ((-1.0,  1.0), (-3.0, 3.0))   # x-range, y-range for evaders

METHODS = ['greedy', 'hungarian_dummy', 'hungarian_aug']
METHOD_LABEL = {'greedy': 'Greedy', 'hungarian_dummy': 'Hungarian + dummy (card)',
                'hungarian_aug': 'Hungarian augmented'}
SHORT_LABEL = {'greedy': 'Greedy', 'hungarian_dummy': 'Hungarian\n+ dummy (card)',
               'hungarian_aug': 'Hungarian\naugmented'}
CASES = {'3v3': PURSUERS_3, '5v3': PURSUERS_5}

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '01_pursuit_evasion', 's017_swarm_vs_swarm',
)

PURSUER_COLORS = ['#e63946', '#f77f00', '#6a040f', '#ffba08', '#ff4d6d']
EVADER_COLORS  = ['#023e8a', '#0077b6', '#00b4d8']


# ── Assignment ──────────────────────────────────────────────────────────────

def distance_matrix(pursuers, evaders):
    return np.linalg.norm(pursuers[:, None, :] - evaders[None, :, :], axis=2)


def greedy_assignment(pursuers, evaders):
    """Pursuers in index order take the nearest unclaimed evader; surplus share."""
    C = distance_matrix(pursuers, evaders)
    assignment, taken = {}, set()
    for i in range(len(pursuers)):
        free = [j for j in range(len(evaders)) if j not in taken]
        cand = free if free else list(range(len(evaders)))
        j = cand[int(np.argmin(C[i, cand]))]
        assignment[i] = j
        taken.add(j)
    return assignment


def hungarian_dummy_assignment(pursuers, evaders):
    """Card: pad with zero-cost dummy columns; dummy rows get no evader."""
    C = distance_matrix(pursuers, evaders)
    M, N = C.shape
    C_pad = np.hstack([C, np.zeros((M, max(M - N, 0)))])
    rows, cols = linear_sum_assignment(C_pad)
    return {int(r): int(c) for r, c in zip(rows, cols) if c < N}


def hungarian_aug_assignment(pursuers, evaders):
    """Each evader column repeated ceil(M/N) times; every pursuer gets an evader."""
    C = distance_matrix(pursuers, evaders)
    M, N = C.shape
    reps = int(np.ceil(M / N))
    rows, cols = linear_sum_assignment(np.tile(C, (1, reps)))
    return {int(r): int(c) % N for r, c in zip(rows, cols)}


ASSIGNERS = {'greedy': greedy_assignment,
             'hungarian_dummy': hungarian_dummy_assignment,
             'hungarian_aug': hungarian_aug_assignment}


def nearest_remaining(p, evaders, captured):
    rem = [j for j in range(len(evaders)) if not captured[j]]
    if not rem:
        return None
    return rem[int(np.argmin([np.linalg.norm(p - evaders[j]) for j in rem]))]


# ── Simulation ───────────────────────────────────────────────────────────────

def simulate(pursuers_init, evaders_init, method, record=True):
    """
    One engagement.  Returns dict with capture_times, capturer, mission_time,
    initial assignment, trajectories and capture steps.
    """
    pursuers = pursuers_init.astype(float).copy()
    evaders = evaders_init.astype(float).copy()
    M, N = len(pursuers), len(evaders)
    captured = [False] * N
    capture_times, capture_steps, capturer = [None] * N, [None] * N, [None] * N

    assignment = ASSIGNERS[method](pursuers, evaders)
    initial = dict(assignment)
    traj_p = [[p.copy()] for p in pursuers]
    traj_e = [[e.copy()] for e in evaders]

    for step in range(int(round(T_MAX / DT))):
        t = (step + 1) * DT
        centroid = pursuers.mean(axis=0)
        for j in range(N):
            if not captured[j]:
                d = evaders[j] - centroid
                n = np.linalg.norm(d)
                d = d / n if n > 1e-9 else np.array([1.0, 0.0])
                evaders[j] = evaders[j] + d * V_EVADER * DT

        for i in range(M):
            tgt = assignment.get(i)
            if tgt is None or captured[tgt]:
                tgt = nearest_remaining(pursuers[i], evaders, captured)
                if tgt is None:
                    break
                assignment[i] = tgt
            d = evaders[tgt] - pursuers[i]
            n = np.linalg.norm(d)
            if n > 1e-9:
                pursuers[i] = pursuers[i] + d / n * V_PURSUER * DT
            if np.linalg.norm(pursuers[i] - evaders[tgt]) < CAPTURE_R and not captured[tgt]:
                captured[tgt] = True
                capture_times[tgt], capture_steps[tgt], capturer[tgt] = t, step, i

        if record:
            for i in range(M):
                traj_p[i].append(pursuers[i].copy())
            for j in range(N):
                traj_e[j].append(evaders[j].copy())
        if all(captured):
            break

    valid = [x for x in capture_times if x is not None]
    return {'capture_times': capture_times, 'capture_steps': capture_steps,
            'capturer': capturer, 'initial': initial,
            'mission_time': max(valid) if len(valid) == N else np.nan,
            'n_captured': len(valid),
            'traj_p': [np.array(x) for x in traj_p], 'traj_e': [np.array(x) for x in traj_e],
            'n_pursuers': M}


def sum_cost(pursuers, evaders, assignment):
    C = distance_matrix(pursuers, evaders)
    return sum(C[i, j] for i, j in assignment.items())


def run_card_cases():
    results = {}
    print('Card geometry:')
    for case, P in CASES.items():
        for m in METHODS:
            res = simulate(P, EVADERS_3, m)
            results[(case, m)] = res
            init = ', '.join(f'P{i+1}->E{j+1}' for i, j in sorted(res['initial'].items()))
            caps = ', '.join(f'E{j+1}:{res["capture_times"][j]:.2f}s(P{res["capturer"][j]+1})'
                             for j in range(len(EVADERS_3)))
            print(f'  {case} {m:16s} | T_mission={res["mission_time"]:.2f}s | '
                  f'sum dist={sum_cost(P, EVADERS_3, res["initial"]):.2f} m | init: {init} | {caps}')
    return results


def run_monte_carlo():
    """Seeded random start positions, same positions for every method."""
    rng = np.random.default_rng(MC_SEED)
    out = {(c, m): [] for c in CASES for m in METHODS}
    for _ in range(MC_TRIALS):
        E = np.column_stack([rng.uniform(*MC_E_BOX[0], 3), rng.uniform(*MC_E_BOX[1], 3)])
        for case, P0 in CASES.items():
            M = len(P0)
            P = np.column_stack([rng.uniform(*MC_P_BOX[0], M), rng.uniform(*MC_P_BOX[1], M)])
            for m in METHODS:
                out[(case, m)].append(simulate(P, E, m, record=False)['mission_time'])
    out = {k: np.array(v) for k, v in out.items()}

    print(f'\nMonte Carlo: {MC_TRIALS} random start sets (seed {MC_SEED}); pursuers in '
          f'x{MC_P_BOX[0]} y{MC_P_BOX[1]}, evaders in x{MC_E_BOX[0]} y{MC_E_BOX[1]}')
    for case in CASES:
        for m in METHODS:
            v = out[(case, m)]
            print(f'  {case} {m:16s} | all captured {np.sum(~np.isnan(v))}/{len(v)} | '
                  f'T_mission mean {np.nanmean(v):.3f} s, median {np.nanmedian(v):.3f} s, '
                  f'min {np.nanmin(v):.2f}, max {np.nanmax(v):.2f}')
        g, h, d = out[(case, 'greedy')], out[(case, 'hungarian_aug')], out[(case, 'hungarian_dummy')]
        tol = 1e-9
        print(f'  {case} Hungarian-aug vs greedy: faster {np.sum(h < g - tol)}, '
              f'tie {np.sum(np.abs(h - g) <= tol)}, slower {np.sum(h > g + tol)}; '
              f'mean diff (greedy - aug) {np.mean(g - h):+.3f} s')
        if case == '5v3':
            print(f'  {case} Hungarian-aug vs Hungarian-dummy: faster {np.sum(h < d - tol)}, '
                  f'tie {np.sum(np.abs(h - d) <= tol)}, slower {np.sum(h > d + tol)}; '
                  f'mean diff (dummy - aug) {np.mean(d - h):+.3f} s')
    m3, m5 = out[('3v3', 'hungarian_aug')], out[('5v3', 'hungarian_aug')]
    print(f'  5v3 vs 3v3 (Hungarian-aug): mean T_mission {np.mean(m5):.3f} s vs {np.mean(m3):.3f} s')
    return out


def run_simulation():
    print('S017 Swarm vs Swarm — Simulation Results')
    print('=' * 60)
    return {'card': run_card_cases(), 'mc': run_monte_carlo()}


# ── Plots ────────────────────────────────────────────────────────────────────

def data_limits(runs, margin=1.0):
    pts = np.concatenate([np.concatenate(r['traj_p'] + r['traj_e']) for r in runs])
    return ((pts[:, 0].min() - margin, pts[:, 0].max() + margin),
            (pts[:, 1].min() - margin, pts[:, 1].max() + margin))


def plot_trajectories(card, out_dir):
    lims = data_limits(list(card.values()))
    fig, axes = plt.subplots(2, 3, figsize=(18, 13))
    for r, case in enumerate(CASES):
        for c, m in enumerate(METHODS):
            ax, res = axes[r, c], card[(case, m)]
            for i, tp in enumerate(res['traj_p']):
                col = PURSUER_COLORS[i]
                ax.plot(tp[:, 0], tp[:, 1], color=col, lw=1.4, label=f'P{i+1}')
                ax.scatter(*tp[0], color=col, s=70, edgecolors='black', zorder=6)
                ax.annotate(f'P{i+1}', tp[0], xytext=(-18, 4), textcoords='offset points',
                            fontsize=7, color=col)
            for j, te in enumerate(res['traj_e']):
                col, cs = EVADER_COLORS[j], res['capture_steps'][j]
                end = cs + 2 if cs is not None else len(te)
                ax.plot(te[:end, 0], te[:end, 1], color=col, lw=1.4, ls='--', label=f'E{j+1}')
                ax.scatter(*te[0], color=col, s=70, marker='^', edgecolors='black', zorder=6)
                if cs is not None:
                    ax.scatter(*te[cs + 1], color=col, s=140, marker='x', linewidths=2.5, zorder=7)
            init = ', '.join(f'P{i+1}→E{j+1}' for i, j in sorted(res['initial'].items()))
            ax.set_title(f'{case} — {METHOD_LABEL[m]}: T = {res["mission_time"]:.2f} s\n'
                         f'initial: {init}', fontsize=8)
            ax.set_aspect('equal'); ax.grid(True, alpha=0.25)
            ax.set_xlim(*lims[0]); ax.set_ylim(*lims[1])
            ax.set_xlabel('X (m)', fontsize=8); ax.set_ylabel('Y (m)', fontsize=8)
            ax.legend(fontsize=6, loc='lower left', ncol=2)
    fig.suptitle('S017 Swarm vs Swarm — card geometry (circle = pursuer start, '
                 'triangle = evader start, x = capture)', fontsize=11, fontweight='bold')
    plt.tight_layout()
    path = os.path.join(out_dir, 'trajectories.png')
    plt.savefig(path, dpi=130); plt.close(); print(f'Saved: {path}')


def plot_gantt(card, out_dir):
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    h = 0.25
    for ax, case in zip(axes, CASES):
        for k, m in enumerate(METHODS):
            res = card[(case, m)]
            for j in range(3):
                t = res['capture_times'][j]
                y = j + (k - 1) * (h + 0.03)
                ax.barh(y, t, height=h, color=EVADER_COLORS[j], alpha=[0.95, 0.6, 0.35][k],
                        edgecolor='black', hatch=['', '//', '..'][k])
                ax.text(t + 0.05, y, f'{t:.2f}s (P{res["capturer"][j]+1})', va='center', fontsize=7)
        handles = [Patch(facecolor='grey', alpha=[0.95, 0.6, 0.35][k], hatch=['', '//', '..'][k],
                         edgecolor='black',
                         label=f'{METHOD_LABEL[m]} (T={card[(case, m)]["mission_time"]:.2f}s)')
                   for k, m in enumerate(METHODS)][::-1]
        ax.set_yticks(range(3)); ax.set_yticklabels(['E1', 'E2', 'E3'])
        ax.set_xlim(0, 4.6); ax.set_xlabel('Capture time (s)')
        ax.set_title(f'{case} — capture time per evader (capturing pursuer in brackets)', fontsize=9)
        ax.legend(handles=handles, fontsize=7, loc='lower right', title='bars top to bottom',
                  title_fontsize=7)
        ax.grid(True, alpha=0.25, axis='x')
    plt.tight_layout()
    path = os.path.join(out_dir, 'gantt_chart.png')
    plt.savefig(path, dpi=150); plt.close(); print(f'Saved: {path}')


def plot_mission_time_comparison(card, mc, out_dir):
    fig, axes = plt.subplots(1, 2, figsize=(15, 6))
    labels, vals, cols = [], [], []
    colors = {'greedy': '#f4a261', 'hungarian_dummy': '#adb5bd', 'hungarian_aug': '#2a9d8f'}
    for case in CASES:
        for m in METHODS:
            labels.append(f'{case}\n{SHORT_LABEL[m]}')
            vals.append(card[(case, m)]['mission_time']); cols.append(colors[m])
    bars = axes[0].bar(range(len(vals)), vals, color=cols, edgecolor='black')
    for b, v in zip(bars, vals):
        axes[0].text(b.get_x() + b.get_width() / 2, v + 0.03, f'{v:.2f}s', ha='center', fontsize=9)
    axes[0].set_xticks(range(len(vals))); axes[0].set_xticklabels(labels, fontsize=7)
    axes[0].set_ylabel('T_mission (s)'); axes[0].set_ylim(0, max(vals) * 1.2)
    axes[0].set_title('Card geometry (symmetric)', fontsize=10, fontweight='bold')
    axes[0].grid(True, alpha=0.3, axis='y')

    data = [mc[(case, m)] for case in CASES for m in METHODS]
    bp = axes[1].boxplot(data, patch_artist=True, showmeans=True,
                         meanprops=dict(marker='D', markerfacecolor='white', markeredgecolor='black'))
    for patch, (case, m) in zip(bp['boxes'], [(c, m) for c in CASES for m in METHODS]):
        patch.set_facecolor(colors[m])
    for k, v in enumerate(data):
        axes[1].text(k + 1, np.mean(v), f' {np.mean(v):.2f}', fontsize=8, va='bottom')
    axes[1].set_xticks(range(1, len(data) + 1)); axes[1].set_xticklabels(labels, fontsize=7)
    axes[1].set_ylabel('T_mission (s)')
    axes[1].set_title(f'Monte Carlo, {MC_TRIALS} random start sets (diamond = mean)',
                      fontsize=10, fontweight='bold')
    axes[1].grid(True, alpha=0.3, axis='y')
    fig.suptitle('S017 — Mission time (time of last capture)', fontsize=11)
    plt.tight_layout()
    path = os.path.join(out_dir, 'mission_time_comparison.png')
    plt.savefig(path, dpi=150); plt.close(); print(f'Saved: {path}')


def plot_assignment_bipartite(card, out_dir):
    fig, axes = plt.subplots(2, 3, figsize=(16, 10))
    for r, case in enumerate(CASES):
        P = CASES[case]
        for c, m in enumerate(METHODS):
            ax, res = axes[r, c], card[(case, m)]
            n_p = len(P)
            p_y = np.linspace(1, -1, n_p); e_y = np.linspace(0.8, -0.8, 3)
            for i in range(n_p):
                j = res['initial'].get(i)
                if j is None:
                    ax.text(-0.6, p_y[i], 'no assignment\n(chases nearest)', fontsize=7,
                            va='center', color='grey')
                    continue
                ax.plot([-1, 1], [p_y[i], e_y[j]], color=PURSUER_COLORS[i], lw=2)
                ax.text(-0.35, p_y[i] + 0.65 * (e_y[j] - p_y[i]) * 0.5 + 0.04,
                        f'{np.linalg.norm(P[i] - EVADERS_3[j]):.2f} m', fontsize=7,
                        color=PURSUER_COLORS[i])
            for i in range(n_p):
                ax.scatter(-1, p_y[i], s=220, color=PURSUER_COLORS[i], edgecolors='black', zorder=5)
                ax.text(-1.2, p_y[i], f'P{i+1}', ha='right', va='center', fontweight='bold')
            for j in range(3):
                ax.scatter(1, e_y[j], s=220, color=EVADER_COLORS[j], marker='^',
                           edgecolors='black', zorder=5)
                ax.text(1.2, e_y[j], f'E{j+1}', va='center', fontweight='bold')
            cost = sum_cost(P, EVADERS_3, res['initial'])
            ax.set_xlim(-1.8, 1.8); ax.set_ylim(-1.3, 1.3); ax.axis('off')
            ax.set_title(f'{case} — {METHOD_LABEL[m]}\nsum of assigned distances {cost:.2f} m, '
                         f'T = {res["mission_time"]:.2f} s', fontsize=9)
    fig.suptitle('S017 — Initial assignment at t = 0 (edge label = initial distance)',
                 fontsize=11, fontweight='bold')
    plt.tight_layout()
    path = os.path.join(out_dir, 'assignment_bipartite.png')
    plt.savefig(path, dpi=130); plt.close(); print(f'Saved: {path}')


def save_animation(card, out_dir):
    """5v3, the three assignment methods side by side."""
    runs = [card[('5v3', m)] for m in METHODS]
    max_len = max(len(r['traj_p'][0]) for r in runs)
    step = 2
    frames = (max_len + step - 1) // step
    lims = data_limits(runs)
    fig, axes = plt.subplots(1, 3, figsize=(16, 7))
    arts = []
    for ax, m, res in zip(axes, METHODS, runs):
        ax.set_xlim(*lims[0]); ax.set_ylim(*lims[1]); ax.set_aspect('equal'); ax.grid(True, alpha=0.2)
        pl = [ax.plot([], [], color=PURSUER_COLORS[i], lw=1.2)[0] for i in range(5)]
        pd = [ax.plot([], [], 'o', color=PURSUER_COLORS[i], mec='black', ms=8)[0] for i in range(5)]
        el = [ax.plot([], [], '--', color=EVADER_COLORS[j], lw=1.2)[0] for j in range(3)]
        ed = [ax.plot([], [], '^', color=EVADER_COLORS[j], mec='black', ms=9)[0] for j in range(3)]
        arts.append((ax, m, res, pl, pd, el, ed))

    def update(f):
        si = min(f * step, max_len - 1)
        out = []
        for ax, m, res, pl, pd, el, ed in arts:
            for i, tp in enumerate(res['traj_p']):
                n = min(si + 1, len(tp))
                pl[i].set_data(tp[:n, 0], tp[:n, 1]); pd[i].set_data([tp[n-1, 0]], [tp[n-1, 1]])
            for j, te in enumerate(res['traj_e']):
                cs = res['capture_steps'][j]
                n = min(si + 1, len(te)) if cs is None else min(si + 1, cs + 2)
                el[j].set_data(te[:n, 0], te[:n, 1]); ed[j].set_data([te[n-1, 0]], [te[n-1, 1]])
                ed[j].set_marker('x' if cs is not None and si >= cs + 1 else '^')
            ax.set_title(f'5v3 {METHOD_LABEL[m]}\nt = {min(si * DT, res["mission_time"]):.2f} s',
                         fontsize=9)
            out += pl + pd + el + ed
        return out

    ani = animation.FuncAnimation(fig, update, frames=frames, interval=50, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer='pillow', fps=20, dpi=80)
    plt.close(); print(f'Saved: {path}  ({frames} frames)')


if __name__ == '__main__':
    out_dir = os.path.normpath(OUTPUT_DIR)
    os.makedirs(out_dir, exist_ok=True)
    data = run_simulation()
    plot_trajectories(data['card'], out_dir)
    plot_gantt(data['card'], out_dir)
    plot_mission_time_comparison(data['card'], data['mc'], out_dir)
    plot_assignment_bipartite(data['card'], out_dir)
    save_animation(data['card'], out_dir)
