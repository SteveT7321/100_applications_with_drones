"""
S085 Drone Light Matrix Positioning
===================================
25 drones take off from random positions in a 20 x 20 x 10 m staging volume and form a
5 x 5 grid (2 m spacing) at z = 50 m. Each drone is assigned a grid slot once, before
take-off, and flies there with an independent 3-axis PID velocity command (|u| <= 5 m/s).
A pairwise repulsion (ORCA-lite) acts inside r_safe = 0.8 m.

Configurations (same initial positions for every configuration in each seed):
  (1) Hungarian + repulsion            (card)
  (2) Greedy nearest-pair + repulsion   (added: simple non-optimal assignment)
  (3) Random permutation + repulsion    (card's assignment baseline, here also flown)
  (4) Hungarian, no repulsion           (added: shows what the repulsion contributes)

Results: one showcase seed (figures, animation) and a Monte Carlo over N_MC staging layouts.

Usage:
    conda activate drones
    python src/05_special_entertainment/s085_light_matrix.py
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401
from scipy.optimize import linear_sum_assignment

# ── Parameters (card) ─────────────────────────────────────────────────────────
N_DRONES = 25
GRID_N = 5
LED_SHOW_HEIGHT = 50.0      # m
FORMATION_DIST = 2.0        # m
COLLISION_RADIUS = 0.5      # m
R_SAFE = 0.8                # m
R0 = R_SAFE
K_REP = 1.5
V_MAX = 5.0                 # m/s
CONV_THRESH = 0.1           # m
DT = 0.05                   # s
T_MAX = 60.0                # s
KP, KI, KD = 1.2, 0.05, 0.4
STAGE_LO = np.array([-10.0, -10.0, 1.0])
STAGE_HI = np.array([10.0, 10.0, 10.0])
N_RAND_TRIALS = 200         # random permutations for the assignment-cost histogram
SHOW_SEED = 42

# ── Parameters (added) ────────────────────────────────────────────────────────
N_MC = 30                   # staging layouts (seeds 0..N_MC-1)
ANIM_STEP = 4               # sim steps per GIF frame (0.2 s)
ANIM_TAIL = 2.0             # s shown after convergence

N_STEPS = int(round(T_MAX / DT))
CONFIGS = ['hung', 'greedy', 'random', 'hung_norep', 'hung_card']
LABELS = {'hung': 'Hungarian + repulsion', 'greedy': 'Greedy + repulsion',
          'random': 'Random + repulsion', 'hung_norep': 'Hungarian, no repulsion',
          'hung_card': 'Hungarian, card PID (windup)'}
SHORT = {'hung': 'Hungarian', 'greedy': 'Greedy', 'random': 'Random',
         'hung_norep': 'Hung.\nno rep.', 'hung_card': 'Hung.\ncard PID'}
COLORS = {'hung': 'tab:red', 'greedy': 'tab:orange', 'random': 'tab:blue',
          'hung_norep': 'tab:grey', 'hung_card': 'tab:purple'}

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), '..', '..',
                          'outputs', '05_special_entertainment', 's085_light_matrix')


# ── Targets and assignment ────────────────────────────────────────────────────

def make_grid_targets(n=GRID_N, d=FORMATION_DIST, z=LED_SHOW_HEIGHT):
    ii, jj = np.meshgrid(np.arange(n), np.arange(n), indexing='ij')
    t = np.stack([ii.ravel() * d, jj.ravel() * d, np.full(n * n, z)], axis=1).astype(float)
    t[:, :2] -= (n - 1) * d / 2
    return t


def cost_matrix(start, targets):
    return np.linalg.norm(start[:, None, :] - targets[None, :, :], axis=2)


def hungarian_assign(start, targets):
    r, c = linear_sum_assignment(cost_matrix(start, targets))
    a = np.empty(len(start), dtype=int)
    a[r] = c
    return a


def greedy_assign(start, targets):
    """Repeatedly pair the closest unassigned drone and free slot."""
    C = cost_matrix(start, targets)
    a = -np.ones(len(start), dtype=int)
    for _ in range(len(start)):
        k, m = np.unravel_index(np.argmin(C), C.shape)
        a[k] = m
        C[k, :] = np.inf
        C[:, m] = np.inf
    return a


def assign(config, start, targets, rng):
    if config in ('hung', 'hung_norep', 'hung_card'):
        return hungarian_assign(start, targets)
    if config == 'greedy':
        return greedy_assign(start, targets)
    return rng.permutation(len(start))


# ── Simulation ────────────────────────────────────────────────────────────────

def simulate(start, goals, repulsion=True, anti_windup=True, log=False):
    """Fly all drones to their assigned goals. Returns metrics (and history if log).

    anti_windup=False is the card's PID: the integral keeps accumulating while the velocity
    command is clipped at V_MAX. anti_windup=True freezes the integral of a drone whose PID
    command would exceed V_MAX (conditional integration).
    """
    pos = start.copy()
    integral = np.zeros_like(pos)
    prev_err = goals - pos                       # no derivative kick at t = 0
    iu = np.triu_indices(N_DRONES, 1)
    ever_close = np.zeros(len(iu[0]), dtype=bool)
    step_events = 0
    min_sep = np.inf
    t_conv = None
    max_err_log = np.zeros(N_STEPS)
    mean_err_log = np.zeros(N_STEPS)
    hist = np.zeros((N_STEPS + 1, N_DRONES, 3)) if log else None
    if log:
        hist[0] = pos
    path_len = np.zeros(N_DRONES)
    n_rep_steps = 0
    for k in range(N_STEPS):
        err = goals - pos
        integral_new = integral + err * DT
        u = KP * err + KI * integral_new + KD * (err - prev_err) / DT
        if anti_windup:
            sat = np.linalg.norm(u, axis=1) > V_MAX
            integral = np.where(sat[:, None], integral, integral_new)
            u = KP * err + KI * integral + KD * (err - prev_err) / DT
        else:
            integral = integral_new
        prev_err = err

        diff = pos[:, None, :] - pos[None, :, :]
        d = np.linalg.norm(diff, axis=2)
        dp = d[iu]
        min_sep = min(min_sep, float(dp.min()))
        close = dp < COLLISION_RADIUS
        ever_close |= close
        step_events += int(close.sum())
        if repulsion:
            np.fill_diagonal(d, np.inf)
            act = d < R0
            if act.any():
                n_rep_steps += 1
                mag = np.where(act, K_REP * (1.0 / d - 1.0 / R0) / d ** 2, 0.0)
                u = u + np.sum((mag / d)[:, :, None] * diff, axis=1)

        s = np.linalg.norm(u, axis=1, keepdims=True)
        u = np.where(s > V_MAX, u / np.maximum(s, 1e-12) * V_MAX, u)
        assert np.all(np.linalg.norm(u, axis=1) <= V_MAX + 1e-9)
        pos = pos + u * DT
        path_len += np.linalg.norm(u, axis=1) * DT
        assert np.all(pos[:, 2] > 0.0)
        if log:
            hist[k + 1] = pos

        e = np.linalg.norm(pos - goals, axis=1)
        max_err_log[k], mean_err_log[k] = e.max(), e.mean()
        if t_conv is None and e.max() <= CONV_THRESH:
            t_conv = (k + 1) * DT
    final_err = np.linalg.norm(pos - goals, axis=1)
    res = {
        't_conv': t_conv, 'pairs_close': int(ever_close.sum()), 'step_events': step_events,
        'min_sep': min_sep, 'final_max': float(final_err.max()),
        'final_mean': float(final_err.mean()), 'path_len': float(path_len.sum()),
        'max_err_log': max_err_log, 'mean_err_log': mean_err_log,
        'rep_steps': n_rep_steps,
        'overshoot_z': float(0.0),
    }
    if log:
        res['hist'] = hist
        res['overshoot_z'] = float((hist[:, :, 2] - LED_SHOW_HEIGHT).max())
    return res


def staging(seed):
    rng = np.random.default_rng(seed)
    return rng.uniform(STAGE_LO, STAGE_HI, size=(N_DRONES, 3)), rng


def run_simulation():
    targets = make_grid_targets()

    # Showcase seed
    start, rng = staging(SHOW_SEED)
    show = {}
    for c in CONFIGS:
        a = assign(c, start, targets, np.random.default_rng(SHOW_SEED + 1000))
        r = simulate(start, targets[a], repulsion=(c != 'hung_norep'),
                     anti_windup=(c != 'hung_card'), log=True)
        r['assignment'] = a
        r['cost'] = float(np.linalg.norm(start - targets[a], axis=1).sum())
        r['horiz'] = float(np.linalg.norm(start[:, :2] - targets[a][:, :2], axis=1).sum())
        r['max_dist'] = float(np.linalg.norm(start - targets[a], axis=1).max())
        show[c] = r
    prng = np.random.default_rng(0)
    rand_costs = np.array([np.linalg.norm(start - targets[prng.permutation(N_DRONES)], axis=1).sum()
                           for _ in range(N_RAND_TRIALS)])

    # Monte Carlo over staging layouts (same layout for every configuration)
    mc = {c: {k: [] for k in ('cost', 'horiz', 'max_dist', 't_conv', 'pairs_close', 'min_sep',
                              'final_max')}
          for c in CONFIGS}
    for seed in range(N_MC):
        st, _ = staging(seed)
        for c in CONFIGS:
            a = assign(c, st, targets, np.random.default_rng(seed + 1000))
            r = simulate(st, targets[a], repulsion=(c != 'hung_norep'),
                         anti_windup=(c != 'hung_card'))
            dist = np.linalg.norm(st - targets[a], axis=1)
            mc[c]['cost'].append(dist.sum())
            mc[c]['horiz'].append(np.linalg.norm(st[:, :2] - targets[a][:, :2], axis=1).sum())
            mc[c]['final_max'].append(r['final_max'])
            mc[c]['max_dist'].append(dist.max())
            mc[c]['t_conv'].append(np.nan if r['t_conv'] is None else r['t_conv'])
            mc[c]['pairs_close'].append(r['pairs_close'])
            mc[c]['min_sep'].append(r['min_sep'])
    mc = {c: {k: np.array(v) for k, v in d.items()} for c, d in mc.items()}
    return targets, start, show, rand_costs, mc


# ── Plotting ──────────────────────────────────────────────────────────────────

def plot_trajectories(show, targets, out_dir):
    r = show['hung']
    hist, goals = r['hist'], targets[r['assignment']]
    fig = plt.figure(figsize=(13, 6))
    ax1 = fig.add_subplot(121, projection='3d')
    cmap = plt.cm.plasma
    for k in range(N_DRONES):
        col = cmap(k / N_DRONES)
        ax1.plot(hist[:, k, 0], hist[:, k, 1], hist[:, k, 2], lw=0.7, alpha=0.7, color=col)
        ax1.scatter(*hist[0, k], s=15, color=col, marker='o')
    ax1.scatter(targets[:, 0], targets[:, 1], targets[:, 2], s=50, color='green', marker='*',
                label='Grid targets')
    ax1.set_xlabel('x (m)')
    ax1.set_ylabel('y (m)')
    ax1.set_zlabel('z (m)')
    ax1.set_title('Hungarian + repulsion: 3D paths\n(circles = staging positions)')
    ax1.legend(fontsize=8)

    ax2 = fig.add_subplot(122)
    final = hist[-1]
    ax2.scatter(targets[:, 0], targets[:, 1], s=220, color='green', marker='*', label='Grid target')
    ax2.scatter(final[:, 0], final[:, 1], s=40, color='red', zorder=3, label='Drone at t = 60 s')
    st = hist[0]
    for k in range(N_DRONES):
        ax2.plot([st[k, 0], goals[k, 0]], [st[k, 1], goals[k, 1]], '-', color='grey', lw=0.6,
                 alpha=0.6)
    ax2.scatter(st[:, 0], st[:, 1], s=12, color='grey', label='Staging (x, y)')
    ax2.set_aspect('equal')
    ax2.set_xlabel('x (m)')
    ax2.set_ylabel('y (m)')
    ax2.set_title(f'Top-down: staging → assigned slot (grey lines)\n'
                  f'mean final error {1000 * r["final_mean"]:.2f} mm')
    ax2.legend(fontsize=8, loc='upper right')
    ax2.grid(alpha=0.3)
    fig.suptitle(f'S085 Light Matrix — seed {SHOW_SEED}', fontsize=12)
    plt.tight_layout()
    path = os.path.join(out_dir, 'trajectories.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_convergence(show, out_dir):
    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(14, 5))
    t = (np.arange(N_STEPS) + 1) * DT
    for c in CONFIGS:
        r = show[c]
        ax.plot(t, r['max_err_log'], color=COLORS[c], lw=1.5,
                label=f"{LABELS[c]} (T_conv = {r['t_conv']:.2f} s)" if r['t_conv'] else LABELS[c])
    ax.plot(t, show['hung']['mean_err_log'], color='tab:red', ls=':', lw=1.2,
            label='Hungarian: mean error')
    ax.axhline(CONV_THRESH, color='k', ls='--', lw=1, label=f'ε_conv = {CONV_THRESH} m')
    ax.set_yscale('log')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Max position error over drones (m)')
    ax.set_title(f'Convergence (seed {SHOW_SEED})')
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3, which='both')

    hist = show['hung']['hist']
    z = hist[:, :, 2]
    tt = np.arange(N_STEPS + 1) * DT
    for k in range(N_DRONES):
        ax2.plot(tt, z[:, k], color=plt.cm.plasma(k / N_DRONES), lw=0.8)
    ax2.axhline(LED_SHOW_HEIGHT, color='green', ls='--', lw=1, label='z_show = 50 m')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Altitude z (m)')
    ax2.set_title(f'Hungarian: altitude of every drone (max overshoot '
                  f'{show["hung"]["overshoot_z"]:.2f} m)')
    ax2.legend(fontsize=8)
    ax2.grid(alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'convergence.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_assignment_comparison(show, rand_costs, out_dir):
    fig, ax = plt.subplots(figsize=(9, 5))
    ax.hist(rand_costs, bins=30, color='steelblue', edgecolor='white', alpha=0.85,
            label=f'Random permutations ({N_RAND_TRIALS})')
    ax.axvline(rand_costs.mean(), color='navy', lw=1.8, label=f'Random mean {rand_costs.mean():.1f} m')
    ax.axvline(show['greedy']['cost'], color='orange', lw=2.2, ls='-.',
               label=f"Greedy {show['greedy']['cost']:.1f} m")
    ax.axvline(show['hung']['cost'], color='red', lw=2.5, ls='--',
               label=f"Hungarian {show['hung']['cost']:.1f} m")
    eta = 1.0 - show['hung']['cost'] / rand_costs.mean()
    ax.set_xlabel('Total assignment distance $D_{assign}$ (m)')
    ax.set_ylabel('Count')
    ax.set_title(f'S085 Assignment cost, seed {SHOW_SEED}: '
                 f'η_assign = {100 * eta:.2f} % vs random mean')
    ax.legend(fontsize=9)
    ax.grid(alpha=0.3)
    path = os.path.join(out_dir, 'assignment_comparison.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_monte_carlo(mc, out_dir):
    fig, axes = plt.subplots(1, 5, figsize=(22, 4.8))
    items = [('cost', 'Total assignment distance (m)'), ('horiz', 'Horizontal part of it (m)'),
             ('t_conv', 'T_conv (s)'), ('pairs_close', 'Pairs closer than 0.5 m'),
             ('final_max', 'Max error at t = 60 s (m)')]
    for ax, (key, lab) in zip(axes, items):
        pos, data, cfg = [], [], []
        for i, c in enumerate(CONFIGS):
            d = mc[c][key][~np.isnan(mc[c][key])]
            if len(d):
                pos.append(i + 1)
                data.append(d)
                cfg.append(c)
            else:
                ax.text(i + 1, 0.5, 'never\nconverged', ha='center', va='center', fontsize=8,
                        transform=ax.get_xaxis_transform(), color=COLORS[c])
        bp = ax.boxplot(data, positions=pos, showmeans=True, patch_artist=True)
        for p, c in zip(bp['boxes'], cfg):
            p.set_facecolor(COLORS[c])
            p.set_alpha(0.4)
        ax.set_xticks(range(1, len(CONFIGS) + 1), [SHORT[c] for c in CONFIGS])
        ax.set_xlim(0.5, len(CONFIGS) + 0.5)
        ax.set_ylabel(lab)
        ax.grid(alpha=0.3)
        if key == 'final_max':
            ax.set_yscale('log')
    fig.suptitle(f'S085 Monte Carlo over {N_MC} staging layouts (same layout for every '
                 'configuration; triangle = mean)')
    plt.tight_layout()
    path = os.path.join(out_dir, 'monte_carlo.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def save_animation(show, targets, out_dir):
    r = show['hung']
    hist = r['hist']
    t_end = T_MAX if r['t_conv'] is None else r['t_conv'] + ANIM_TAIL
    n_used = min(N_STEPS + 1, int(round(t_end / DT)) + 1)
    n_frames = int(np.ceil((n_used - 1) / ANIM_STEP)) + 1
    idx = [min(i * ANIM_STEP, n_used - 1) for i in range(n_frames)]
    goals = targets[r['assignment']]
    fig = plt.figure(figsize=(7, 6.5))
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(targets[:, 0], targets[:, 1], targets[:, 2], s=40, color='green', marker='*')
    cols = [plt.cm.plasma(k / N_DRONES) for k in range(N_DRONES)]
    sc = ax.scatter(hist[0, :, 0], hist[0, :, 1], hist[0, :, 2], s=25, c=cols, depthshade=False)
    txt = ax.text2D(0.02, 0.95, '', transform=ax.transAxes, fontsize=9)
    ax.set_xlim(-10, 10)
    ax.set_ylim(-10, 10)
    ax.set_zlim(0, 55)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_zlabel('z (m)')
    ax.set_title('S085 Light Matrix — Hungarian + repulsion')

    def update(i):
        k = idx[i]
        p = hist[k]
        sc._offsets3d = (p[:, 0], p[:, 1], p[:, 2])
        e = np.linalg.norm(p - goals, axis=1).max()
        txt.set_text(f't = {k * DT:5.2f} s   max err = {e:.3f} m')
        return sc, txt

    ani = animation.FuncAnimation(fig, update, frames=len(idx), blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer='pillow', fps=25, dpi=72)
    plt.close(fig)
    print(f'Saved: {path} ({len(idx)} frames)')


# ── Main ──────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    targets, start, show, rand_costs, mc = run_simulation()
    print('\n=== S085 Drone Light Matrix Positioning — Results ===')
    print(f'  Showcase seed {SHOW_SEED}:')
    print(f"  {'Configuration':<26}{'D_assign (m)':>13}{'longest (m)':>12}{'T_conv (s)':>11}"
          f"{'pairs<0.5m':>11}{'step events':>12}{'min sep (m)':>12}{'final mean err (mm)':>21}")
    for c in CONFIGS:
        r = show[c]
        tc = f"{r['t_conv']:.2f}" if r['t_conv'] is not None else 'never'
        print(f"  {LABELS[c]:<26}{r['cost']:>13.1f}{r['max_dist']:>12.1f}{tc:>11}"
              f"{r['pairs_close']:>11}{r['step_events']:>12}{r['min_sep']:>12.3f}"
              f"{1000 * r['final_mean']:>21.3f}")
    eta = 1 - show['hung']['cost'] / rand_costs.mean()
    print(f'  Random permutations ({N_RAND_TRIALS}): mean {rand_costs.mean():.1f} m, '
          f'min {rand_costs.min():.1f} m; eta_assign = {100 * eta:.2f}%')
    print(f"  Greedy vs Hungarian cost: +{100 * (show['greedy']['cost'] / show['hung']['cost'] - 1):.2f}%")
    print(f"  Horizontal part of D_assign: Hungarian {show['hung']['horiz']:.1f} m, greedy "
          f"{show['greedy']['horiz']:.1f} m, random {show['random']['horiz']:.1f} m")
    climb = float(np.sum(LED_SHOW_HEIGHT - start[:, 2]))
    print(f'  Pure vertical climb (sum over drones): {climb:.1f} m; excess of D_assign over it: '
          f"Hungarian {show['hung']['cost'] - climb:.1f} m, random {show['random']['cost'] - climb:.1f} m")
    for c in ['hung', 'hung_card']:
        print(f"  {LABELS[c]}: max altitude overshoot {show[c]['overshoot_z']:.2f} m above "
              f"{LED_SHOW_HEIGHT:.0f} m; final max / mean error {show[c]['final_max']:.3f} / "
              f"{show[c]['final_mean']:.3f} m; repulsion active in {show[c]['rep_steps']} steps")
    st0 = np.linalg.norm(start[:, None] - start[None], axis=2)[np.triu_indices(N_DRONES, 1)]
    print(f'  Staging layout: closest pair {st0.min():.3f} m apart at t = 0')
    print(f'  Monte Carlo over {N_MC} staging layouts (mean ± std):')
    print(f"  {'Configuration':<30}{'D_assign (m)':>16}{'horiz. (m)':>16}{'longest (m)':>14}"
          f"{'T_conv (s)':>16}{'pairs<0.5m':>14}{'min sep (m)':>16}{'final max err (m)':>20}"
          f"{'converged':>11}")
    for c in CONFIGS:
        m = mc[c]
        ok = ~np.isnan(m['t_conv'])
        tc = (f"{m['t_conv'][ok].mean():>9.2f} ± {m['t_conv'][ok].std():4.2f}" if ok.any()
              else f"{'never':>16}")
        print(f"  {LABELS[c]:<30}{m['cost'].mean():>8.1f} ± {m['cost'].std():5.1f}"
              f"{m['horiz'].mean():>8.1f} ± {m['horiz'].std():5.1f}"
              f"{m['max_dist'].mean():>7.1f} ± {m['max_dist'].std():4.1f}"
              f"{tc}"
              f"{m['pairs_close'].mean():>8.2f} ± {m['pairs_close'].std():4.2f}"
              f"{m['min_sep'].mean():>9.3f} ± {m['min_sep'].std():5.3f}"
              f"{m['final_max'].mean():>12.3f} ± {m['final_max'].std():5.3f}"
              f"{int(ok.sum()):>7}/{N_MC}")
    h = mc['hung']
    for c in ['greedy', 'random', 'hung_norep']:
        m = mc[c]
        print(f"  Hungarian vs {LABELS[c]}: lower D_assign in {int(np.sum(h['cost'] < m['cost'] - 1e-9))}/{N_MC}, "
              f"faster T_conv in {int(np.sum(h['t_conv'] < m['t_conv']))}/{N_MC} "
              f"(ties {int(np.sum(h['t_conv'] == m['t_conv']))}), "
              f"fewer close pairs in {int(np.sum(h['pairs_close'] < m['pairs_close']))}/{N_MC} "
              f"(ties {int(np.sum(h['pairs_close'] == m['pairs_close']))})")
    for c in CONFIGS:
        print(f"  {LABELS[c]}: layouts with zero pairs < 0.5 m: "
              f"{int(np.sum(mc[c]['pairs_close'] == 0))}/{N_MC}")
    corr = np.corrcoef(h['max_dist'], h['t_conv'])[0, 1]
    print(f'  Hungarian: correlation(longest path, T_conv) over layouts = {corr:.3f}')
    poles = np.roots([1.0 + KD, KP, KI])
    print('  Unsaturated closed loop per axis ((1+Kd) s^2 + Kp s + Ki = 0): poles '
          + ', '.join(f'{p:.4f}' for p in poles)
          + f' 1/s; slow time constant {1.0 / np.min(np.abs(poles)):.1f} s')

    out = os.path.normpath(OUTPUT_DIR)
    os.makedirs(out, exist_ok=True)
    plot_trajectories(show, targets, out)
    plot_convergence(show, out)
    plot_assignment_comparison(show, rand_costs, out)
    plot_monte_carlo(mc, out)
    save_animation(show, targets, out)
