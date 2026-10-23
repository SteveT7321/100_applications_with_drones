"""
S019 3D Upgrade — Dynamic Target Reassignment
=============================================
3 pursuers vs 3 evaders in 3D (the card's scenario).  Evader behaviours (3.5 m/s):
  straight — fixed 3D heading, away from the pursuers' centroid at t = 0
  spiral   — the card's helix: horizontal circle (omega = 0.5 rad/s) plus a
             vertical component A_z sin(2 omega t), normalised to v_E
  random   — the card's 3D random walk: direction += N(0, 0.3^2) per step, renormalised
Evaders stay in the card's altitude band z in [0.8, 5.0] m: a step that would
leave it has its vertical velocity mirrored, and the evader keeps the mirrored
vertical sense (a bounce, not a slide along the limit).  Evaders do not react to
the pursuers, so every method faces exactly the same evader paths (common random
numbers).

Pursuers use the card's decoupled guidance: horizontal pure pursuit at 5 m/s
plus vertical tracking at up to 2.5 m/s.  Three methods (card):
  static  — 3D Hungarian at t = 0, never re-optimised.
  dynamic — 3D Hungarian re-optimised every T_CHECK = 1 s; the new assignment is
            adopted if the improvement in summed time-to-go exceeds the card's
            altitude-aware switching cost dC_base + w_z |z_E,new - z_P| / v_z.
  greedy  — every step each pursuer chases its 3D-nearest live evader.
Captured evaders are never assigned; a pursuer whose evader is captured joins
the nearest live evader (static and dynamic alike, so they differ only by the
periodic re-optimisation).  With fewer live evaders than pursuers the Hungarian
matrix repeats each live column.  Any pursuer within CAPTURE_R captures; the
test uses the closest approach inside each step.

Experiments: the card scenario and a wide "crossing" scenario (the S019 2D fix's
layout lifted to 3D: the straight evader runs across the other lanes while
climbing), each with a 200-trial seeded Monte Carlo (jittered starts, random
strategy-to-evader mapping, random-walk seeds, spiral phases; random straight
headings on the crossing layout) and a switching-cost sweep.

Usage:
    conda activate drones
    python src/01_pursuit_evasion/3d/s019_3d_dynamic_reassignment.py
"""
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.colors import ListedColormap
from scipy.optimize import linear_sum_assignment

OUTPUT_DIR = os.path.normpath(os.path.join(
    os.path.dirname(__file__), '..', '..', '..',
    'outputs', '01_pursuit_evasion', '3d', 's019_3d_dynamic_reassignment'))

# ── Parameters (card) ─────────────────────────────────────────────────────────
M, N       = 3, 3
V_XY       = 5.0
V_Z_MAX    = 2.5
V_EVADER   = 3.5
CAPTURE_R  = 0.15
DT         = 0.02
T_MAX      = 40.0
T_CHECK    = 1.0
CHECK_EVERY = int(round(T_CHECK / DT))
DC_BASE    = 0.5
W_Z        = 0.5
Z_MIN, Z_MAX = 0.8, 5.0
SPIRAL_OMEGA = 0.5
SPIRAL_AZ    = 0.3
RANDOM_SIGMA = 0.3
CARD_SEED    = 42      # random-walk seed in the card scenario

def _unit(v):
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


# heading None = straight evader flees the pursuers' centroid at t = 0
SCENARIOS = {
    'card': {
        'P': np.array([[-5.0, -2.0, 1.0], [-5.0, 0.0, 2.5], [-5.0, 2.0, 4.0]]),
        'E': np.array([[2.0, -2.0, 1.5], [2.0, 0.0, 3.0], [2.0, 2.0, 4.5]]),
        'strats': ['straight', 'spiral', 'random'], 'phi0': [0.0, 0.5, 0.0],
        'heading': None, 'jitter': (1.0, 0.5),
    },
    # Wide layout (S019 2D fix, lifted to 3D): the straight evader E1 runs down
    # across the other lanes while climbing from the low to the high layer.
    'crossing': {
        'P': np.array([[0.0, 12.0, 1.0], [0.0, 0.0, 2.5], [0.0, -12.0, 4.0]]),
        'E': np.array([[20.0, 12.0, 1.5], [20.0, 0.0, 3.0], [20.0, -12.0, 4.5]]),
        'strats': ['straight', 'random', 'spiral'], 'phi0': [0.0, 0.0, 0.5],
        'heading': _unit([0.0, -1.0, 0.2]), 'jitter': (2.0, 0.5),
    },
}
SCEN_NAMES = list(SCENARIOS)

MC_TRIALS = 200
MC_SEED   = 19
MC_ELEV_MAX = np.radians(20.0)   # crossing MC: straight heading elevation in +-20 deg
SWEEP = [(0.0, W_Z), (0.1, W_Z), (0.25, W_Z), (0.5, W_Z), (0.5, 0.0)]   # (dC_base, w_z)

METHODS = ['static', 'dynamic', 'greedy']
P_COLORS = ['#e63946', '#f77f00', '#9d0208']
E_COLORS = ['#023e8a', '#0096c7', '#48cae4']


# ── Assignment helpers ───────────────────────────────────────────────────────

def cost_matrix(P, E):
    """Card: C[i, j] = |p_Pi - p_Ej|_3D / v_P."""
    return np.linalg.norm(P[:, None] - E[None], axis=2) / V_XY


def hungarian_live(P, E, captured):
    live = [j for j in range(N) if not captured[j]]
    C = cost_matrix(P, E[live])
    rows, cols = linear_sum_assignment(np.tile(C, (1, int(np.ceil(M / len(live))))))
    return {int(r): live[int(c) % len(live)] for r, c in zip(rows, cols)}


def nearest_live(p, E, captured):
    live = [j for j in range(N) if not captured[j]]
    return live[int(np.argmin([np.linalg.norm(p - E[j]) for j in live]))]


def switching_cost(p, e_new, dc_base, w_z):
    return dc_base + w_z * abs(e_new[2] - p[2]) / V_Z_MAX


# ── Agents ───────────────────────────────────────────────────────────────────

class Evader:
    def __init__(self, pos, strategy, centroid, seed, phi0=0.0, heading=None):
        self.pos = np.asarray(pos, float).copy()
        self.strategy, self.phi0, self.t = strategy, phi0, 0.0
        self.rng = np.random.default_rng(seed)
        self.dir = _unit(self.pos - centroid) if heading is None else _unit(heading)
        self.bounces = 0
        self.zsign = 1.0      # spiral: vertical sense, flipped at each band bounce

    def step(self):
        if self.strategy == 'spiral':
            w = SPIRAL_OMEGA
            d = np.array([np.cos(w * self.t + self.phi0), np.sin(w * self.t + self.phi0),
                          self.zsign * SPIRAL_AZ * np.sin(2 * w * self.t)])
            d /= np.linalg.norm(d)
        elif self.strategy == 'random':
            d = self.dir + self.rng.normal(0.0, RANDOM_SIGMA, 3)
            d /= np.linalg.norm(d)
            self.dir = d
        else:
            d = self.dir
        v = V_EVADER * d
        z_new = self.pos[2] + v[2] * DT
        if z_new < Z_MIN or z_new > Z_MAX:       # mirror the vertical velocity
            v[2] = -v[2]
            self.bounces += 1
            if self.strategy == "spiral":
                self.zsign = -self.zsign
            else:
                self.dir = self.dir * np.array([1.0, 1.0, -1.0])
        self.pos = self.pos + v * DT
        self.t += DT


def pursuer_velocity(p, e):
    """Card: horizontal pure pursuit at v_xy plus vertical tracking at <= v_z."""
    v = np.zeros(3)
    dxy = e[:2] - p[:2]
    n = np.linalg.norm(dxy)
    if n > 1e-9:
        v[:2] = dxy / n * min(V_XY, n / DT)
    v[2] = np.clip((e[2] - p[2]) / DT, -V_Z_MAX, V_Z_MAX)
    return v


def min_dist_in_step(p0, p1, e0, e1):
    r0, d = e0 - p0, (e1 - p1) - (e0 - p0)
    a = np.dot(d, d)
    s = 0.0 if a < 1e-15 else float(np.clip(-np.dot(r0, d) / a, 0.0, 1.0))
    return np.linalg.norm(r0 + s * d)


# ── Simulation ───────────────────────────────────────────────────────────────

def simulate(P0, E0, strats, seeds, phi0s, heading, method, dc_base=DC_BASE, w_z=W_Z, record=True):
    P = np.asarray(P0, float).copy()
    centroid = P.mean(axis=0)
    evs = [Evader(E0[j], strats[j], centroid, seeds[j], phi0s[j], heading) for j in range(N)]
    captured = [False] * N
    cap_t, capturer = [None] * N, [None] * N
    E = np.array([e.pos for e in evs])
    assignment = hungarian_live(P, E, captured)
    initial = dict(assignment)
    switches, best_impr = [], -np.inf
    tp, te, ah = [P.copy()], [E.copy()], [[assignment[i] for i in range(M)]]
    n_steps = int(round(T_MAX / DT))
    t_end = T_MAX
    for step in range(n_steps):
        E = np.array([e.pos for e in evs])
        live = [j for j in range(N) if not captured[j]]
        if method == 'dynamic' and step > 0 and step % CHECK_EVERY == 0:
            new = hungarian_live(P, E, captured)
            C = cost_matrix(P, E)
            impr = sum(C[i, assignment[i]] for i in range(M)) - sum(C[i, new[i]] for i in range(M))
            best_impr = max(best_impr, impr)
            changed = [i for i in range(M) if new[i] != assignment[i]]
            if changed:
                thr = min(switching_cost(P[i], E[new[i]], dc_base, w_z) for i in changed)
                if impr > thr:
                    switches.append((step * DT, dict(assignment), dict(new), impr, thr))
                    assignment = new
        if method == 'greedy':
            assignment = {i: nearest_live(P[i], E, captured) for i in range(M)}
        P_old, E_old = P.copy(), E.copy()
        for i in range(M):
            P[i] = P[i] + pursuer_velocity(P[i], E[assignment[i]]) * DT
        for j in live:
            evs[j].step()
        E = np.array([e.pos for e in evs])
        t = (step + 1) * DT
        for j in live:
            for i in range(M):
                if min_dist_in_step(P_old[i], P[i], E_old[j], E[j]) < CAPTURE_R:
                    captured[j], cap_t[j], capturer[j] = True, t, i
                    break
        if all(captured):
            t_end = t
        elif any(captured[assignment[i]] for i in range(M)):
            for i in range(M):     # freed pursuers join the nearest live evader
                if captured[assignment[i]]:
                    assignment[i] = nearest_live(P[i], E, captured)
        if record:
            tp.append(P.copy()); te.append(E.copy())
            ah.append([assignment[i] if not all(captured) else -1 for i in range(M)])
        if all(captured):
            break
    done = all(captured)
    return {'capture_times': cap_t, 'capturer': capturer, 'initial': initial,
            'mission_time': t_end if done else np.nan, 'switches': switches,
            'best_impr': best_impr, 'traj_p': np.array(tp), 'traj_e': np.array(te),
            'assign': np.array(ah), 'bounces': [e.bounces for e in evs]}


def scenario_args(name):
    s = SCENARIOS[name]
    return s['P'], s['E'], s['strats'], [CARD_SEED + j for j in range(N)], s['phi0'], s['heading']


def run_fixed():
    res = {}
    for name in SCEN_NAMES:
        s = SCENARIOS[name]
        print(f'Scenario {name}: ' + ', '.join(f'E{j+1} {s["strats"][j]}' for j in range(N)) +
              f' (random-walk seeds {CARD_SEED}-{CARD_SEED + N - 1})')
        for m in METHODS:
            r = simulate(*scenario_args(name), m)
            res[(name, m)] = r
            caps = ', '.join(f'E{j+1} {r["capture_times"][j]:.2f}s (P{r["capturer"][j]+1})'
                             for j in np.argsort(r['capture_times']))
            init = ', '.join(f'P{i+1}->E{j+1}' for i, j in sorted(r['initial'].items()))
            ez = r['traj_e'][:, :, 2]
            bi = '' if m != 'dynamic' else f' | best check improvement {r["best_impr"]:.3f} s'
            print(f'  {m:8s} | T_mission {r["mission_time"]:.2f} s | init {init} | {caps} | '
                  f'switches {len(r["switches"])}{bi} | evader z {ez.min():.2f}-{ez.max():.2f} m, '
                  f'z-band bounces {r["bounces"]}')
            for (ts, old, new, impr, thr) in r['switches']:
                o = ', '.join(f'P{i+1}->E{j+1}' for i, j in sorted(old.items()))
                n = ', '.join(f'P{i+1}->E{j+1}' for i, j in sorted(new.items()))
                print(f'      switch at {ts:.2f} s: [{o}] -> [{n}], improvement {impr:.3f} s > cost {thr:.3f} s')
    return res


def run_monte_carlo(name):
    s = SCENARIOS[name]
    rng = np.random.default_rng(MC_SEED + SCEN_NAMES.index(name))
    configs = [('static', None), ('greedy', None)] + [('dynamic', c) for c in SWEEP]
    out = {c: {'T': [], 'sw': []} for c in configs}
    bounce_runs, bounces = 0, []
    jxy, jz = s['jitter']
    jit = np.array([jxy, jxy, jz])
    for _ in range(MC_TRIALS):
        P = s['P'] + rng.uniform(-1, 1, (M, 3)) * jit
        E = s['E'] + rng.uniform(-1, 1, (N, 3)) * jit
        P[:, 2] = np.clip(P[:, 2], Z_MIN, Z_MAX); E[:, 2] = np.clip(E[:, 2], Z_MIN, Z_MAX)
        strats = [s['strats'][k] for k in rng.permutation(N)]
        seeds = [int(x) for x in rng.integers(0, 2 ** 31, N)]
        phi0s = list(rng.uniform(0, 2 * np.pi, N))
        if s['heading'] is None:
            heading = None
        else:
            az, el = rng.uniform(0, 2 * np.pi), rng.uniform(-MC_ELEV_MAX, MC_ELEV_MAX)
            heading = np.array([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])
        for c in configs:
            m, sw = c
            kw = {} if sw is None else {'dc_base': sw[0], 'w_z': sw[1]}
            r = simulate(P, E, strats, seeds, phi0s, heading, m, record=False, **kw)
            out[c]['T'].append(r['mission_time']); out[c]['sw'].append(len(r['switches']))
            if c == ('static', None):
                bounces.append(sum(r['bounces'])); bounce_runs += sum(r['bounces']) > 0
    for c in out:
        out[c] = {k: np.array(v, float) for k, v in out[c].items()}
    tol = 1e-9
    st = out[('static', None)]['T']
    head = 'flee heading' if s['heading'] is None else 'random straight heading (elevation +-20 deg)'
    print(f'\nMonte Carlo, {name} layout: {MC_TRIALS} trials (seed {MC_SEED + SCEN_NAMES.index(name)}); '
          f'start jitter +-{jxy} m (x, y), +-{jz} m (z); random strategy mapping, random-walk seeds, '
          f'spiral phases, {head}')
    print(f'  static runs with a z-band bounce: {bounce_runs}/{MC_TRIALS}, '
          f'mean {np.mean(bounces):.2f} bounces per run')
    for c in configs:
        T, sw = out[c]['T'], out[c]['sw']
        lab = c[0] if c[1] is None else f'dynamic dC={c[1][0]:.2f} w_z={c[1][1]:.1f}'
        line = (f'  {lab:28s} | all captured {np.sum(~np.isnan(T))}/{MC_TRIALS} | mean {np.nanmean(T):.3f} s '
                f'median {np.nanmedian(T):.3f} min {np.nanmin(T):.2f} max {np.nanmax(T):.2f}')
        if c[0] != 'static':
            both = ~np.isnan(T) & ~np.isnan(st)
            line += (f' | vs static faster {np.sum(T < st - tol)}, tie {np.sum(np.abs(T - st) <= tol)}, '
                     f'slower {np.sum(T > st + tol)}, mean (static - this) {np.mean(st[both] - T[both]):+.3f} s')
        if c[0] == 'dynamic':
            line += f' | switches/trial {sw.mean():.2f}, trials with a switch {int(np.sum(sw > 0))}'
        print(line)
    return out


def run_simulation():
    print('S019 3D Dynamic Target Reassignment — Simulation Results')
    print('=' * 70)
    fixed = run_fixed()
    return {'fixed': fixed, 'mc': {name: run_monte_carlo(name) for name in SCEN_NAMES}}


# ── Plots ────────────────────────────────────────────────────────────────────

LIMS = {'card': ((-6, 10), (-8, 8)), 'crossing': ((-2, 24), (-16, 16))}
M_COLORS = {'static': '#adb5bd', 'dynamic': '#2a9d8f', 'greedy': '#f4a261'}


def plot_trajectories_3d(fixed, out_dir):
    fig = plt.figure(figsize=(19, 13))
    for row, name in enumerate(SCEN_NAMES):
        strats = SCENARIOS[name]['strats']
        for k, m in enumerate(METHODS):
            r = fixed[(name, m)]
            ax = fig.add_subplot(2, 3, row * 3 + k + 1, projection='3d')
            tp, te, ah = r['traj_p'], r['traj_e'], r['assign']
            for j in range(N):
                ct = r['capture_times'][j]
                end = int(round(ct / DT)) + 1 if ct is not None else len(te)
                ax.plot(te[:end, j, 0], te[:end, j, 1], te[:end, j, 2], color=E_COLORS[j], ls='--',
                        lw=1.4, label=f'E{j+1} ({strats[j]})')
                ax.scatter(*te[0, j], color=E_COLORS[j], marker='^', s=50, edgecolors='black')
                ax.scatter(*te[end - 1, j], color=E_COLORS[j], marker='x', s=80, linewidths=2)
            for i in range(M):
                cuts = [0] + [s for s in range(1, len(ah)) if ah[s][i] != ah[s - 1][i]] + [len(tp) - 1]
                for a, b in zip(cuts[:-1], cuts[1:]):   # path coloured by the assigned evader
                    j = ah[a][i]
                    ax.plot(tp[a:b + 1, i, 0], tp[a:b + 1, i, 1], tp[a:b + 1, i, 2],
                            color=E_COLORS[j] if j >= 0 else 'grey', lw=2.2, alpha=0.8)
                ax.scatter(*tp[0, i], color=P_COLORS[i], s=60, marker='o', edgecolors='black')
                ax.text(*tp[0, i], f' P{i+1}', fontsize=7)
            for (ts, *_rest) in r['switches']:
                s = int(round(ts / DT))
                ax.scatter(tp[s, :, 0], tp[s, :, 1], tp[s, :, 2], color='black', marker='s', s=25)
            (x0, x1), (y0, y1) = LIMS[name]
            ax.set_xlim(x0, x1); ax.set_ylim(y0, y1); ax.set_zlim(0, 6)
            ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
            ax.set_title(f'{name} — {m}: T = {r["mission_time"]:.2f} s, {len(r["switches"])} switches',
                         fontsize=10)
            ax.legend(fontsize=7, loc='upper left')
    fig.suptitle('S019 3D — pursuer path coloured by the evader it is chasing (circle = pursuer start, '
                 'triangle = evader start, x = capture, black square = re-optimisation switch)', fontsize=11)
    plt.tight_layout(rect=(0, 0, 1, 0.96))
    path = os.path.join(out_dir, 'trajectories_3d.png')
    plt.savefig(path, dpi=110); plt.close(); print(f'Saved: {path}')


def plot_altitude_time(fixed, out_dir):
    fig, axes = plt.subplots(2, 3, figsize=(18, 9), sharey=True)
    for row, name in enumerate(SCEN_NAMES):
        strats = SCENARIOS[name]['strats']
        for ax, m in zip(axes[row], METHODS):
            r = fixed[(name, m)]
            t = np.arange(len(r['traj_p'])) * DT
            for i in range(M):
                ax.plot(t, r['traj_p'][:, i, 2], color=P_COLORS[i], lw=1.8, label=f'P{i+1}')
            for j in range(N):
                ct = r['capture_times'][j]
                end = int(round(ct / DT)) + 1 if ct is not None else len(t)
                ax.plot(t[:end], r['traj_e'][:end, j, 2], color=E_COLORS[j], ls='--', lw=1.8,
                        label=f'E{j+1} ({strats[j]})')
                if ct is not None:
                    ax.axvline(ct, color=E_COLORS[j], ls=':', lw=1)
            ax.axhspan(0, Z_MIN, color='grey', alpha=0.15); ax.axhspan(Z_MAX, 6, color='grey', alpha=0.15)
            ax.set_title(f'{name} — {m} (dotted = capture)', fontsize=10)
            ax.grid(True, alpha=0.3); ax.set_ylim(0, 6)
            if row == 1:
                ax.set_xlabel('Time (s)')
        axes[row, 0].set_ylabel('z (m)'); axes[row, 0].legend(fontsize=7, ncol=2)
    fig.suptitle('S019 3D — altitude vs time (grey = outside the evader band 0.8–5.0 m)', fontsize=11)
    plt.tight_layout()
    path = os.path.join(out_dir, 'altitude_time.png')
    plt.savefig(path, dpi=120); plt.close(); print(f'Saved: {path}')


def plot_assignment_heatmap(fixed, out_dir):
    cmap = ListedColormap(['#dee2e6'] + E_COLORS)
    fig, axes = plt.subplots(6, 1, figsize=(13, 10))
    for row, name in enumerate(SCEN_NAMES):
        t_last = max(len(fixed[(name, m)]['assign']) for m in METHODS) * DT
        for k, m in enumerate(METHODS):
            ax = axes[row * 3 + k]
            a = fixed[(name, m)]['assign'].T + 1
            ax.imshow(a, aspect='auto', cmap=cmap, vmin=0, vmax=3, interpolation='nearest',
                      extent=[0, len(a[0]) * DT, M - 0.5, -0.5])
            for (ts, *_r) in fixed[(name, m)]['switches']:
                ax.axvline(ts, color='black', lw=1.5)
            ax.set_yticks(range(M)); ax.set_yticklabels([f'P{i+1}' for i in range(M)], fontsize=7)
            ax.set_xlim(0, t_last)
            ax.set_title(f'{name} — {m}: T = {fixed[(name, m)]["mission_time"]:.2f} s', fontsize=8)
    axes[-1].set_xlabel('Time (s)')
    handles = [plt.Rectangle((0, 0), 1, 1, color=E_COLORS[j]) for j in range(N)]
    axes[0].legend(handles, [f'E{j+1}' for j in range(N)], fontsize=7, loc='upper right', ncol=3)
    fig.suptitle('S019 3D — assigned evader over time (colour); black line = re-optimisation switch',
                 fontsize=10)
    plt.tight_layout()
    path = os.path.join(out_dir, 'assignment_heatmap.png')
    plt.savefig(path, dpi=120); plt.close(); print(f'Saved: {path}')


def plot_mission_comparison(fixed, mc, out_dir):
    fig, axes = plt.subplots(2, 3, figsize=(19, 10))
    for row, name in enumerate(SCEN_NAMES):
        ax = axes[row, 0]
        vals = [fixed[(name, m)]['mission_time'] for m in METHODS]
        bars = ax.bar(METHODS, vals, color=[M_COLORS[m] for m in METHODS], edgecolor='black')
        for b, v in zip(bars, vals):
            ax.text(b.get_x() + b.get_width() / 2, v + 0.05, f'{v:.2f} s', ha='center')
        ax.set_ylabel('T_mission (s)'); ax.grid(True, alpha=0.3, axis='y')
        ax.set_title(f'{name} scenario', fontsize=10)
        ax = axes[row, 1]
        keys = [('static', None), ('dynamic', (DC_BASE, W_Z)), ('greedy', None)]
        data = [mc[name][k]['T'][~np.isnan(mc[name][k]['T'])] for k in keys]
        bp = ax.boxplot(data, patch_artist=True, showmeans=True,
                        meanprops=dict(marker='D', markerfacecolor='white', markeredgecolor='black'))
        for p, m in zip(bp['boxes'], METHODS):
            p.set_facecolor(M_COLORS[m])
        for k, v in enumerate(data):
            ax.text(k + 1.1, np.mean(v), f'{np.mean(v):.2f}', fontsize=8)
        ax.set_xticks([1, 2, 3]); ax.set_xticklabels(['static', 'dynamic (card cost)', 'greedy'])
        ax.set_ylabel('T_mission (s)'); ax.grid(True, alpha=0.3, axis='y')
        ax.set_title(f'{name} Monte Carlo, {MC_TRIALS} trials (diamond = mean)', fontsize=10)
        ax = axes[row, 2]
        st = mc[name][('static', None)]['T']
        labels, gain, sw = [], [], []
        for s in SWEEP:
            T = mc[name][('dynamic', s)]['T']
            both = ~np.isnan(T) & ~np.isnan(st)
            labels.append(f'dC={s[0]:g}\nw_z={s[1]:g}')
            gain.append(np.mean(st[both] - T[both])); sw.append(mc[name][('dynamic', s)]['sw'].mean())
        x = np.arange(len(SWEEP))
        ax.bar(x, gain, color='#2a9d8f', edgecolor='black', label='mean (static - dynamic) T_mission')
        ax.axhline(0, color='black', lw=0.8)
        ax.set_xticks(x); ax.set_xticklabels(labels, fontsize=8)
        ax.set_ylabel('Mean time saved vs static (s)')
        ax2 = ax.twinx()
        ax2.plot(x, sw, 'ko-', label='switches per trial'); ax2.set_ylabel('Switches per trial')
        h1, l1 = ax.get_legend_handles_labels(); h2, l2 = ax2.get_legend_handles_labels()
        ax.legend(h1 + h2, l1 + l2, fontsize=8, loc='lower left')
        ax.set_title(f'{name}: switching-cost sweep (same trials)', fontsize=10)
    fig.suptitle('S019 3D — mission time (last capture)', fontsize=11)
    plt.tight_layout()
    path = os.path.join(out_dir, 'mission_comparison.png')
    plt.savefig(path, dpi=120); plt.close(); print(f'Saved: {path}')


ANIM_SCEN = 'crossing'


def save_animation(fixed, out_dir):
    runs = [fixed[(ANIM_SCEN, m)] for m in METHODS]
    max_len = max(len(r['traj_p']) for r in runs)
    step = 8
    frames = (max_len - 1) // step + 2
    fig = plt.figure(figsize=(16, 6))
    arts = []
    (x0, x1), (y0, y1) = LIMS[ANIM_SCEN]
    for k, (m, r) in enumerate(zip(METHODS, runs)):
        ax = fig.add_subplot(1, 3, k + 1, projection='3d')
        ax.set_xlim(x0, x1); ax.set_ylim(y0, y1); ax.set_zlim(0, 6)
        ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
        pl = [ax.plot([], [], [], color=P_COLORS[i], lw=1.2)[0] for i in range(M)]
        pd = [ax.plot([], [], [], 'o', color=P_COLORS[i], mec='black', ms=6)[0] for i in range(M)]
        el = [ax.plot([], [], [], '--', color=E_COLORS[j], lw=1.2)[0] for j in range(N)]
        ed = [ax.plot([], [], [], '^', color=E_COLORS[j], mec='black', ms=7)[0] for j in range(N)]
        al = [ax.plot([], [], [], ':', color='black', lw=0.8)[0] for i in range(M)]
        arts.append((ax, m, r, pl, pd, el, ed, al))

    def update(f):
        si = f * step
        out = []
        for ax, m, r, pl, pd, el, ed, al in arts:
            n = min(si + 1, len(r['traj_p']))
            tp, te, ah = r['traj_p'][:n], r['traj_e'][:n], r['assign'][n - 1]
            for i in range(M):
                pl[i].set_data_3d(tp[:, i, 0], tp[:, i, 1], tp[:, i, 2])
                pd[i].set_data_3d([tp[-1, i, 0]], [tp[-1, i, 1]], [tp[-1, i, 2]])
                j = ah[i]
                if j >= 0:
                    al[i].set_data_3d([tp[-1, i, 0], te[-1, j, 0]], [tp[-1, i, 1], te[-1, j, 1]],
                                      [tp[-1, i, 2], te[-1, j, 2]])
                else:
                    al[i].set_data_3d([], [], [])
            for j in range(N):
                ct = r['capture_times'][j]
                nj = n if ct is None else min(n, int(round(ct / DT)) + 1)
                el[j].set_data_3d(te[:nj, j, 0], te[:nj, j, 1], te[:nj, j, 2])
                ed[j].set_data_3d([te[nj - 1, j, 0]], [te[nj - 1, j, 1]], [te[nj - 1, j, 2]])
                ed[j].set_marker('x' if ct is not None and (n - 1) * DT >= ct - 1e-9 else '^')
            ax.set_title(f'{ANIM_SCEN} — {m}   t = {(n - 1) * DT:.2f} s', fontsize=9)
            out += pl + pd + el + ed + al
        return out

    ani = FuncAnimation(fig, update, frames=frames, interval=60, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer=PillowWriter(fps=16), dpi=70)
    plt.close(); print(f'Saved: {path}  ({frames} frames)')


if __name__ == '__main__':
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    data = run_simulation()
    plot_trajectories_3d(data['fixed'], OUTPUT_DIR)
    plot_altitude_time(data['fixed'], OUTPUT_DIR)
    plot_assignment_heatmap(data['fixed'], OUTPUT_DIR)
    plot_mission_comparison(data['fixed'], data['mc'], OUTPUT_DIR)
    save_animation(data['fixed'], OUTPUT_DIR)
    print('Done.')
