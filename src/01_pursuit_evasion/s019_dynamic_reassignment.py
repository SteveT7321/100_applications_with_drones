"""
S019 Dynamic Target Reassignment
==================================
3 pursuers vs 3 evaders with mixed strategies (straight, spiral, random), all at
the same altitude (z = 2 m, simulated in the horizontal plane).

Three assignment strategies:
  1. Static   — Hungarian at t = 0, never re-optimised.
  2. Dynamic  — Hungarian at t = 0, re-optimised every T_CHECK seconds; the new
                assignment is adopted only if it lowers the summed time-to-go
                by more than DC_SWITCH.
  3. Greedy   — every step each pursuer chases its nearest uncaptured evader.

Captured evaders are never assigned.  When a pursuer's evader is captured, the
freed pursuer joins the nearest remaining evader (static and dynamic alike), so
the only difference between static and dynamic is the periodic re-optimisation.
With fewer evaders than pursuers, the Hungarian step uses an augmented cost
matrix (each remaining evader column repeated ceil(M / N_remaining) times) so
every pursuer keeps a live target.  Any pursuer within CAPTURE_R captures.

Evader behaviours (speed V_EVADER):
  straight — fixed heading (given, or away from the pursuers' centroid at t = 0)
  spiral   — heading SPIRAL_ANGLE (sign = circling direction) away from the current
             flee direction (away from the pursuers' centroid): it circles round
             the pursuers while opening the range (logarithmic spiral)
  random   — uniformly random heading, redrawn every RANDOM_PERIOD seconds
             from a seeded generator

Scenarios:
  card      — card start positions; straight evader flees.
  crossing  — wide layout in which the straight evader runs across the other
              lanes and the spiral evader circles up through them, so the
              initial assignment goes stale.
  Monte Carlo on the crossing layout: jittered starts, random strategy-to-evader
  mapping, random straight heading, random spiral direction, random-evader seed.

Usage:
    conda activate drones
    python src/01_pursuit_evasion/s019_dynamic_reassignment.py
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.collections import LineCollection
from scipy.optimize import linear_sum_assignment

# ── Parameters ─────────────────────────────────
M           = 3            # number of pursuers
N           = 3            # number of evaders
V_PURSUER   = 5.0          # m/s  pursuer speed
V_EVADER    = 3.5          # m/s  evader speed
CAPTURE_R   = 0.30         # m    capture radius
DT          = 0.05         # s    simulation timestep
T_MAX       = 120.0        # s    max simulation time
T_CHECK     = 1.0          # s    reassignment check period
DC_SWITCH   = 0.5          # s    switching cost (summed time-to-go)
DC_SWITCH_SWEEP = [0.0, 0.1, 0.25, 0.5]   # s  sensitivity sweep (Monte Carlo)
SPIRAL_ANGLE = np.radians(60.0)   # spiral heading offset from the flee direction
RANDOM_PERIOD = 1.0        # s    random evader heading hold time
RANDOM_SEED = 42           # seed of the random evader (fixed scenarios)

SCENARIOS = {
    'card': {
        'P': np.array([[-5.0, -2.0], [-5.0, 0.0], [-5.0, 2.0]]),
        'E': np.array([[ 2.0, -2.0], [ 2.0, 0.0], [ 2.0, 2.0]]),
        'spec': [('straight', None), ('spiral', +1), ('random', None)],
    },
    'crossing': {
        'P': np.array([[0.0, 12.0], [0.0, 0.0], [0.0, -12.0]]),
        'E': np.array([[20.0, 12.0], [20.0, 0.0], [20.0, -12.0]]),
        'spec': [('straight', -np.pi / 2), ('random', None), ('spiral', +1)],
    },
}

# Monte Carlo (crossing layout)
MC_TRIALS = 200
MC_SEED   = 7
MC_JITTER = 2.0            # m, uniform jitter of every start coordinate

STRATEGIES = ['static', 'dynamic', 'greedy']

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '01_pursuit_evasion', 's019_dynamic_reassignment',
)


# ── Assignment helpers ─────────────────────────

def cost_matrix(pursuers, evaders):
    """C[i, j] = |p_Pi - p_Ej| / v_P (time-to-go estimate)."""
    return np.linalg.norm(pursuers[:, None] - evaders[None], axis=2) / V_PURSUER


def hungarian_live(pursuers, evaders, captured):
    """
    Optimal assignment over uncaptured evaders only.  Each live evader column is
    repeated ceil(M / n_live) times so that every pursuer gets a live evader.
    """
    live = [j for j in range(N) if not captured[j]]
    C = cost_matrix(pursuers, evaders[live])
    reps = int(np.ceil(M / len(live)))
    rows, cols = linear_sum_assignment(np.tile(C, (1, reps)))
    return {int(r): live[int(c) % len(live)] for r, c in zip(rows, cols)}


def nearest_live(p, evaders, captured):
    live = [j for j in range(N) if not captured[j]]
    return live[int(np.argmin([np.linalg.norm(p - evaders[j]) for j in live]))]


def assignment_cost(pursuers, evaders, assignment):
    C = cost_matrix(pursuers, evaders)
    return sum(C[i, j] for i, j in assignment.items())


# ── Evader behaviours ──────────────────────────

def flee_dir(e, pursuers):
    d = e - pursuers.mean(axis=0)
    n = np.linalg.norm(d)
    return d / n if n > 1e-9 else np.array([1.0, 0.0])


def rot(v, a):
    c, s = np.cos(a), np.sin(a)
    return np.array([c * v[0] - s * v[1], s * v[0] + c * v[1]])


def spec_label(spec):
    mode, prm = spec
    if mode == 'straight':
        return 'straight, flee' if prm is None else f'straight {np.degrees(prm):.0f}°'
    if mode == 'spiral':
        return 'spiral ' + ('CCW' if prm > 0 else 'CW')
    return 'random'


# ── Simulation ─────────────────────────────────

def run_one(strategy, p0, e0, spec, rng_seed, record=True, dc_switch=DC_SWITCH):
    """Simulate one strategy.  Returns capture times, events and trajectories."""
    rng = np.random.default_rng(rng_seed)
    pursuers, evaders = p0.astype(float).copy(), e0.astype(float).copy()
    captured, capture_times, capturer = [False] * N, [None] * N, [None] * N
    straight_dir = {}
    for j, (mode, prm) in enumerate(spec):
        if mode == 'straight':
            straight_dir[j] = (flee_dir(evaders[j], pursuers) if prm is None
                               else np.array([np.cos(prm), np.sin(prm)]))
    rand_heading = {j: rng.uniform(0, 2 * np.pi) for j in range(N)}
    check_every = int(round(T_CHECK / DT))
    rand_every = int(round(RANDOM_PERIOD / DT))

    if strategy == 'greedy':
        assignment = {i: nearest_live(pursuers[i], evaders, captured) for i in range(M)}
    else:
        assignment = hungarian_live(pursuers, evaders, captured)
    initial = dict(assignment)
    events = []       # (t, kind, old, new, improvement)
    best_imp = (0.0, 0.0)   # largest improvement seen at a dynamic check (value, t)
    p_traj, e_traj, a_hist = [pursuers.copy()], [evaders.copy()], [dict(assignment)]

    for step in range(int(round(T_MAX / DT))):
        t = (step + 1) * DT
        # ── Evaders move
        for j, (mode, prm) in enumerate(spec):
            if captured[j]:
                continue
            if mode == 'straight':
                d = straight_dir[j]
            elif mode == 'spiral':
                d = rot(flee_dir(evaders[j], pursuers), prm * SPIRAL_ANGLE)
            else:
                if step > 0 and step % rand_every == 0:
                    rand_heading[j] = rng.uniform(0, 2 * np.pi)
                d = np.array([np.cos(rand_heading[j]), np.sin(rand_heading[j])])
            evaders[j] = evaders[j] + d * V_EVADER * DT

        # ── Assignment update
        if strategy == 'greedy':
            assignment = {i: nearest_live(pursuers[i], evaders, captured) for i in range(M)}
        elif strategy == 'dynamic' and step > 0 and step % check_every == 0:
            new = hungarian_live(pursuers, evaders, captured)
            imp = assignment_cost(pursuers, evaders, assignment) - \
                assignment_cost(pursuers, evaders, new)
            if imp > best_imp[0]:
                best_imp = (imp, step * DT)
            if imp > dc_switch and new != assignment:
                events.append((step * DT, 'reoptimise', dict(assignment), dict(new), imp))
                assignment = new

        # ── Pursuers move (pure pursuit, no overshoot)
        for i in range(M):
            d = evaders[assignment[i]] - pursuers[i]
            n = np.linalg.norm(d)
            if n > 1e-9:
                pursuers[i] = pursuers[i] + d / n * min(V_PURSUER * DT, n)

        # ── Capture: any pursuer within CAPTURE_R of a live evader
        for j in range(N):
            if captured[j]:
                continue
            dists = np.linalg.norm(pursuers - evaders[j], axis=1)
            if dists.min() <= CAPTURE_R:
                captured[j], capture_times[j], capturer[j] = True, t, int(np.argmin(dists))
        if not all(captured) and strategy != 'greedy':
            # freed pursuers join the nearest live evader (static and dynamic)
            for i in range(M):
                if captured[assignment[i]]:
                    old = dict(assignment)
                    assignment[i] = nearest_live(pursuers[i], evaders, captured)
                    events.append((t, 'freed', old, dict(assignment), None))
        if record:
            p_traj.append(pursuers.copy()); e_traj.append(evaders.copy())
            a_hist.append(dict(assignment))
        if all(captured):
            break

    done = all(captured)
    return {'capture_times': capture_times, 'capturer': capturer, 'initial': initial,
            'total_time': max(capture_times) if done else np.nan, 'events': events,
            'n_reopt': sum(1 for e in events if e[1] == 'reoptimise'), 'best_imp': best_imp,
            'p_traj': np.array(p_traj), 'e_traj': np.array(e_traj), 'a_hist': a_hist,
            'spec': spec}


def fmt_assign(a):
    return ', '.join(f'P{i+1}→E{j+1}' for i, j in sorted(a.items()))


def run_fixed(name):
    sc = SCENARIOS[name]
    pos = lambda A: ', '.join(f'({x:g}, {y:g})' for x, y in A)
    print(f'\nScenario "{name}": P1-P3 at {pos(sc["P"])}; E1-E3 at {pos(sc["E"])}')
    print('  evaders: ' + ', '.join(f'E{j+1} {spec_label(s)}' for j, s in enumerate(sc['spec'])))
    res = {}
    for s in STRATEGIES:
        r = run_one(s, sc['P'], sc['E'], sc['spec'], RANDOM_SEED)
        res[s] = r
        caps = ', '.join(f'E{j+1} {r["capture_times"][j]:.2f}s (P{r["capturer"][j]+1})'
                         for j in range(N))
        print(f'  {s:8s} T_mission = {r["total_time"]:.2f} s | {caps} | '
              f're-optimisations {r["n_reopt"]}')
        print(f'           initial: {fmt_assign(r["initial"])}')
        if s == 'dynamic':
            print(f'           largest improvement at a check: {r["best_imp"][0]:.3f} s '
                  f'(t = {r["best_imp"][1]:.2f} s; threshold {DC_SWITCH} s)')
        for t, kind, old, new, imp in r['events']:
            extra = f' (improvement {imp:.3f} s)' if imp is not None else ''
            print(f'           t={t:5.2f}s {kind:10s}: {fmt_assign(old)}  ->  {fmt_assign(new)}{extra}')
    return res


def run_monte_carlo():
    rng = np.random.default_rng(MC_SEED)
    base = SCENARIOS['crossing']
    out = {s: [] for s in STRATEGIES}
    n_reopt = []
    sweep = {dc: ([], []) for dc in DC_SWITCH_SWEEP}
    for _ in range(MC_TRIALS):
        p0 = base['P'] + rng.uniform(-MC_JITTER, MC_JITTER, base['P'].shape)
        e0 = base['E'] + rng.uniform(-MC_JITTER, MC_JITTER, base['E'].shape)
        modes = [['straight', 'spiral', 'random'][k] for k in rng.permutation(N)]
        spec = []
        for mode in modes:
            if mode == 'straight':
                spec.append((mode, float(rng.uniform(0, 2 * np.pi))))
            elif mode == 'spiral':
                spec.append((mode, float(rng.choice([-1.0, 1.0]))))
            else:
                spec.append((mode, None))
        seed = int(rng.integers(1_000_000))
        for s in STRATEGIES:
            r = run_one(s, p0, e0, spec, seed, record=False)
            out[s].append(r['total_time'])
            if s == 'dynamic':
                n_reopt.append(r['n_reopt'])
        for dc in DC_SWITCH_SWEEP:
            r = run_one('dynamic', p0, e0, spec, seed, record=False, dc_switch=dc)
            sweep[dc][0].append(r['total_time']); sweep[dc][1].append(r['n_reopt'])
    out = {s: np.array(v) for s, v in out.items()}
    n_reopt = np.array(n_reopt)
    print(f'\nMonte Carlo on the crossing layout: {MC_TRIALS} trials (seed {MC_SEED}); starts '
          f'jittered ±{MC_JITTER} m, random mode mapping, straight heading U[0, 360°), '
          'spiral direction ±, random-evader seed per trial')
    for s in STRATEGIES:
        v = out[s]
        print(f'  {s:8s} all captured within {T_MAX:.0f} s: {np.sum(~np.isnan(v))}/{MC_TRIALS} | '
              f'mean {np.nanmean(v):.3f} s, median {np.nanmedian(v):.3f} s, '
              f'min {np.nanmin(v):.2f}, max {np.nanmax(v):.2f}')
    d = out['dynamic']
    tol = 1e-9
    for other in ['static', 'greedy']:
        o = out[other]
        ok = ~np.isnan(o) & ~np.isnan(d)
        print(f'  dynamic vs {other:6s}: faster {np.sum(d[ok] < o[ok] - tol)}, '
              f'tie {np.sum(np.abs(d[ok] - o[ok]) <= tol)}, slower {np.sum(d[ok] > o[ok] + tol)} '
              f'(of {ok.sum()} trials both finished); mean ({other} - dynamic) '
              f'{np.mean(o[ok] - d[ok]):+.3f} s')
    w = n_reopt > 0
    print(f'  dynamic re-optimisations per trial: mean {n_reopt.mean():.2f}; '
          f'trials with at least one: {w.sum()}/{MC_TRIALS}')
    if w.any():
        st = out['static'][w] - d[w]
        print(f'  in those {w.sum()} trials: dynamic faster {np.sum(st > tol)}, '
              f'slower {np.sum(st < -tol)}, tie {np.sum(np.abs(st) <= tol)}; '
              f'mean (static - dynamic) {st.mean():+.3f} s')
    print('  switching-cost sweep (dynamic vs static, same trials):')
    sw = {}
    for dc in DC_SWITCH_SWEEP:
        t, n = np.array(sweep[dc][0]), np.array(sweep[dc][1])
        st = out['static'] - t
        sw[dc] = {'mean_gain': float(np.mean(st)), 'mean_reopt': float(n.mean()),
                  'faster': int(np.sum(st > tol)), 'slower': int(np.sum(st < -tol)),
                  'mean_t': float(np.mean(t))}
        print(f'    DC_SWITCH = {dc:4.2f} s: mean T {np.mean(t):.3f} s, re-optimisations/trial '
              f'{n.mean():.2f}, faster {sw[dc]["faster"]}, slower {sw[dc]["slower"]}, '
              f'mean (static - dynamic) {np.mean(st):+.3f} s')
    return {'times': out, 'n_reopt': n_reopt, 'sweep': sw}


def run_simulation():
    print('=' * 70)
    print('S019 Dynamic Target Reassignment — Results')
    print('=' * 70)
    return {'fixed': {k: run_fixed(k) for k in SCENARIOS}, 'mc': run_monte_carlo()}


# ── Plots ──────────────────────────────────────

PURSUER_COLORS = ['#e63946', '#f77f00', '#9d0208']
EVADER_COLORS  = ['#1f77b4', '#17becf', '#6a3d9a']
STRATEGY_COLORS = {'static': 'darkorange', 'dynamic': 'seagreen', 'greedy': 'mediumpurple'}
TITLES = {'static': 'Static (Hungarian at t=0)', 'dynamic': 'Dynamic (re-optimise every 1 s)',
          'greedy': 'Greedy (nearest, every step)'}


def limits(runs):
    pts = np.concatenate([np.concatenate([r['p_traj'].reshape(-1, 2), r['e_traj'].reshape(-1, 2)])
                          for r in runs])
    return (pts[:, 0].min() - 1, pts[:, 0].max() + 1), (pts[:, 1].min() - 1, pts[:, 1].max() + 1)


def draw_paths(ax, r, xl, yl, title):
    P, E, A = r['p_traj'], r['e_traj'], r['a_hist']
    for j in range(N):
        ct = r['capture_times'][j]
        k = int(round(ct / DT)) if ct is not None else len(E) - 1
        ax.plot(E[:k + 1, j, 0], E[:k + 1, j, 1], '--', color=EVADER_COLORS[j], lw=1.5,
                label=f'E{j+1} ({spec_label(r["spec"][j])})')
        ax.scatter(*E[0, j], marker='^', s=80, color=EVADER_COLORS[j], edgecolors='black', zorder=5)
        if ct is not None:
            ax.scatter(*E[k, j], marker='X', s=140, color=EVADER_COLORS[j], edgecolors='black', zorder=6)
    for i in range(M):
        segs = np.stack([P[:-1, i], P[1:, i]], axis=1)
        cols = [EVADER_COLORS[A[k + 1][i]] for k in range(len(segs))]
        ax.add_collection(LineCollection(segs, colors=cols, linewidths=2.5))
        ax.scatter(*P[0, i], s=80, color=PURSUER_COLORS[i], edgecolors='black', zorder=5)
        ax.annotate(f'P{i+1}', P[0, i], xytext=(-16, 6), textcoords='offset points', fontsize=8)
    for t, kind, old, new, imp in r['events']:
        k = int(round(t / DT))
        for i in range(M):
            if old[i] != new[i]:
                ax.scatter(*P[k, i], marker='o', s=110, facecolors='none',
                           edgecolors='black' if kind == 'reoptimise' else 'grey', lw=1.8, zorder=7)
    ax.set_xlim(*xl); ax.set_ylim(*yl); ax.set_aspect('equal'); ax.grid(True, alpha=0.3)
    ax.set_title(title, fontsize=9)
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
    ax.legend(fontsize=7, loc='best')


def plot_trajectories(fixed, out_dir):
    """Top-down paths; each pursuer segment is coloured by the evader it is chasing."""
    fig, axes = plt.subplots(2, 3, figsize=(18, 13))
    for row, name in enumerate(SCENARIOS):
        xl, yl = limits(fixed[name].values())
        for ax, s in zip(axes[row], STRATEGIES):
            r = fixed[name][s]
            draw_paths(ax, r, xl, yl, f'{name}: {TITLES[s]}\nT_mission = {r["total_time"]:.2f} s, '
                                      f're-optimisations: {r["n_reopt"]}')
    fig.suptitle('S019 — top-down paths (z = 2 m). Pursuer path colour = evader it is chasing; X = capture;\n'
                 'black ring = switch by re-optimisation, grey ring = freed pursuer joins nearest evader',
                 fontsize=10)
    plt.tight_layout()
    path = os.path.join(out_dir, 'trajectories.png')
    plt.savefig(path, dpi=130, bbox_inches='tight'); plt.close(); print(f'Saved: {path}')


def plot_assignment_heatmap(fixed, out_dir):
    fig, axes = plt.subplots(3, 2, figsize=(15, 7.5))
    cmap = matplotlib.colors.ListedColormap(EVADER_COLORS)
    for col, name in enumerate(SCENARIOS):
        for ax, s in zip(axes[:, col], STRATEGIES):
            r = fixed[name][s]
            A = np.array([[a[i] for a in r['a_hist']] for i in range(M)])
            n = A.shape[1]
            ax.imshow(A, aspect='auto', cmap=cmap, vmin=-0.5, vmax=N - 0.5, interpolation='nearest',
                      extent=[-DT / 2, (n - 0.5) * DT, M - 0.5, -0.5])
            for j, ct in enumerate(r['capture_times']):
                if ct is not None:
                    ax.axvline(ct, color='black', lw=1.2)
                    ax.text(ct, -0.65, f'E{j+1}', fontsize=7, ha='center')
            for t, kind, old, new, imp in r['events']:
                if kind == 'reoptimise':
                    ax.axvline(t, color='red', ls='--', lw=1.5)
            ax.set_yticks(range(M)); ax.set_yticklabels([f'P{i+1}' for i in range(M)])
            ax.set_title(f'{name}: {TITLES[s]}', fontsize=9, pad=12)
        axes[-1, col].set_xlabel('Time (s)')
    handles = [plt.Rectangle((0, 0), 1, 1, color=EVADER_COLORS[j]) for j in range(N)]
    fig.legend(handles, [f'assigned to E{j+1}' for j in range(N)], loc='lower center', ncol=3, fontsize=8)
    fig.suptitle('S019 — assignment over time (black line = capture, red dashed = re-optimisation switch)',
                 fontsize=10)
    plt.tight_layout(rect=(0, 0.04, 1, 1))
    path = os.path.join(out_dir, 'assignment_heatmap.png')
    plt.savefig(path, dpi=150, bbox_inches='tight'); plt.close(); print(f'Saved: {path}')


def plot_capture_comparison(fixed, mc, out_dir):
    fig, axes = plt.subplots(1, 3, figsize=(20, 5.5))
    ax = axes[0]
    w = 0.25
    names = list(SCENARIOS)
    for k, s in enumerate(STRATEGIES):
        vals = [fixed[n][s]['total_time'] for n in names]
        bars = ax.bar(np.arange(len(names)) + (k - 1) * w, vals, w, color=STRATEGY_COLORS[s],
                      edgecolor='black', label=s.capitalize())
        for b, v in zip(bars, vals):
            ax.text(b.get_x() + b.get_width() / 2, v + 0.1, f'{v:.2f}', ha='center', fontsize=8)
    ax.set_xticks(range(len(names))); ax.set_xticklabels([f'{n} scenario' for n in names])
    ax.set_ylabel('T_mission (s)'); ax.legend(fontsize=8)
    ax.set_title('Fixed scenarios', fontsize=10, fontweight='bold'); ax.grid(True, alpha=0.3, axis='y')

    ax = axes[1]
    data = [mc['times'][s][~np.isnan(mc['times'][s])] for s in STRATEGIES]
    bp = ax.boxplot(data, patch_artist=True, showmeans=True,
                    meanprops=dict(marker='D', markerfacecolor='white', markeredgecolor='black'))
    for patch, s in zip(bp['boxes'], STRATEGIES):
        patch.set_facecolor(STRATEGY_COLORS[s])
    for k, v in enumerate(data):
        ax.text(k + 1.25, np.mean(v), f'{np.mean(v):.2f}', fontsize=8, va='center')
    ax.set_xticks(range(1, 4))
    ax.set_xticklabels([f'{s.capitalize()}\n({len(v)} finished)' for s, v in zip(STRATEGIES, data)])
    ax.set_ylabel('T_mission (s)'); ax.grid(True, alpha=0.3, axis='y')
    ax.set_title(f'Monte Carlo, crossing layout, {MC_TRIALS} trials (diamond = mean)',
                 fontsize=10, fontweight='bold')

    ax = axes[2]
    dcs = list(mc['sweep'])
    gain = [mc['sweep'][d]['mean_gain'] for d in dcs]
    bars = ax.bar(range(len(dcs)), gain, color=['seagreen' if g > 0 else 'indianred' for g in gain],
                  edgecolor='black')
    for b, d in zip(bars, dcs):
        sw = mc['sweep'][d]
        ax.text(b.get_x() + b.get_width() / 2, b.get_height(),
                f'{b.get_height():+.3f} s\n{sw["mean_reopt"]:.2f} switches/trial\n'
                f'{sw["faster"]} faster / {sw["slower"]} slower',
                ha='center', va='bottom' if b.get_height() >= 0 else 'top', fontsize=7)
    ax.axhline(0, color='black', lw=0.8)
    ax.set_xticks(range(len(dcs)))
    ax.set_xticklabels([f'{d:g} s' + (' (card)' if d == DC_SWITCH else '') for d in dcs])
    ax.set_xlabel('Switching cost DC_SWITCH'); ax.set_ylabel('Mean (static − dynamic) T_mission (s)')
    lo, hi = min(gain + [0]), max(gain + [0])
    ax.set_ylim(lo - 0.6 * (hi - lo + 0.05), hi + 0.6 * (hi - lo + 0.05))
    ax.set_title('Dynamic gain over static vs switching cost', fontsize=10, fontweight='bold')
    ax.grid(True, alpha=0.3, axis='y')
    plt.tight_layout()
    path = os.path.join(out_dir, 'capture_comparison.png')
    plt.savefig(path, dpi=150, bbox_inches='tight'); plt.close(); print(f'Saved: {path}')


def save_animation(fixed, out_dir):
    """Crossing scenario, the three strategies side by side."""
    runs = fixed['crossing']
    xl, yl = limits(runs.values())
    max_len = max(len(runs[s]['p_traj']) for s in STRATEGIES)
    step = max(1, int(np.ceil(max_len / 140)))
    frames = list(range(0, max_len, step)) + [max_len - 1]
    fig, axes = plt.subplots(1, 3, figsize=(16, 6))
    arts = []
    for ax, s in zip(axes, STRATEGIES):
        r = runs[s]
        ax.set_xlim(*xl); ax.set_ylim(*yl); ax.set_aspect('equal'); ax.grid(True, alpha=0.3)
        pl = [ax.plot([], [], color=PURSUER_COLORS[i], lw=1.3)[0] for i in range(M)]
        pd = [ax.plot([], [], 'o', color=PURSUER_COLORS[i], mec='black', ms=7)[0] for i in range(M)]
        el = [ax.plot([], [], '--', color=EVADER_COLORS[j], lw=1.2)[0] for j in range(N)]
        ed = [ax.plot([], [], '^', color=EVADER_COLORS[j], mec='black', ms=8)[0] for j in range(N)]
        ln = [ax.plot([], [], ':', color='grey', lw=1)[0] for i in range(M)]
        arts.append((ax, s, r, pl, pd, el, ed, ln))

    def update(f):
        out = []
        for ax, s, r, pl, pd, el, ed, ln in arts:
            k = min(f, len(r['p_traj']) - 1)
            P, E, a = r['p_traj'], r['e_traj'], r['a_hist'][k]
            for i in range(M):
                pl[i].set_data(P[:k + 1, i, 0], P[:k + 1, i, 1]); pd[i].set_data([P[k, i, 0]], [P[k, i, 1]])
                j = a[i]
                ln[i].set_data([P[k, i, 0], E[k, j, 0]], [P[k, i, 1], E[k, j, 1]])
            for j in range(N):
                ct = r['capture_times'][j]
                kk = min(k, int(round(ct / DT))) if ct is not None else k
                el[j].set_data(E[:kk + 1, j, 0], E[:kk + 1, j, 1]); ed[j].set_data([E[kk, j, 0]], [E[kk, j, 1]])
                ed[j].set_marker('X' if ct is not None and k >= int(round(ct / DT)) else '^')
            ax.set_title(f'crossing — {s.capitalize()}\nt = {k * DT:.2f} s (dotted = current assignment)',
                         fontsize=9)
            out += pl + pd + el + ed + ln
        return out

    ani = animation.FuncAnimation(fig, update, frames=frames, interval=50, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer='pillow', fps=20, dpi=80)
    plt.close(); print(f'Saved: {path}  ({len(frames)} frames)')


if __name__ == '__main__':
    out_dir = os.path.normpath(OUTPUT_DIR)
    os.makedirs(out_dir, exist_ok=True)
    data = run_simulation()
    plot_trajectories(data['fixed'], out_dir)
    plot_assignment_heatmap(data['fixed'], out_dir)
    plot_capture_comparison(data['fixed'], data['mc'], out_dir)
    save_animation(data['fixed'], out_dir)
