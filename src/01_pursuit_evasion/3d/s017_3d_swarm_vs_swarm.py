"""
S017 3D Upgrade — Swarm vs Swarm
================================
M pursuers vs N = 3 evaders in a 3D volume with three altitude layers
(z = 1.0, 2.5, 4.0 m).  Pursuers fly 3D pure pursuit at their assigned evader,
with the card's dive boost (up to 1.3 v_P when diving onto a lower evader).
Evaders flee the 3D centroid of all pursuers and bias their altitude toward the
layer farthest from every pursuer (k_z = 0.5), renormalised to v_E.  When an
evader is captured, every pursuer that was chasing it switches to the nearest
remaining evader (3D distance).

Three initial assignment methods on the 3D Euclidean cost matrix (the S017 2D fix):
  greedy          — pursuers in index order each take the nearest unclaimed
                    evader; surplus pursuers take the nearest evader (shared).
  hungarian_dummy — the card's method: Hungarian with dummy columns for M > N;
                    pursuers left on a dummy column chase the nearest evader.
  hungarian_aug   — Hungarian on an augmented matrix with every evader column
                    repeated ceil(M/N) times, so surplus pursuers are placed by
                    the same sum-of-distances objective.

Cases (card): A = 3v3, pursuers and evaders on the three layers (evader layers
reversed with respect to the pursuers); B = 5v3, pursuers spanning all layers,
evaders at mid altitude.  Each case also runs "flat" (every agent at z = 2.5 m)
to measure what altitude diversity changes.  A 200-trial seeded Monte Carlo
with common random numbers (the same random starts for every method, and the
same x-y starts for layered and flat) compares the methods beyond the
symmetric card geometry.

Usage:
    conda activate drones
    python src/01_pursuit_evasion/3d/s017_3d_swarm_vs_swarm.py
"""
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Patch
from scipy.optimize import linear_sum_assignment

OUTPUT_DIR = os.path.normpath(os.path.join(
    os.path.dirname(__file__), '..', '..', '..',
    'outputs', '01_pursuit_evasion', '3d', 's017_3d_swarm_vs_swarm'))

# ── Parameters ────────────────────────────────────────────────────────────────
V_PURSUER   = 5.0
V_EVADER    = 3.5
CAPTURE_R   = 0.15
DT          = 0.02
T_MAX       = 30.0
ETA_DIVE    = 0.15      # dive efficiency
DIVE_DZ_MIN = 0.5       # m, dive boost only when z_P > z_E + 0.5
DIVE_CAP    = 1.3       # x v_P
K_Z_ESCAPE  = 0.5
Z_LAYERS    = [1.0, 2.5, 4.0]
Z_FLAT      = 2.5
Z_MIN, Z_MAX = 0.5, 5.5

L0, L1, L2 = Z_LAYERS
PURSUERS_3 = np.array([[-5., -2., L0], [-5., 0., L1], [-5., 2., L2]])
EVADERS_3  = np.array([[0., -2., L2], [0., 0., L1], [0., 2., L0]])
PURSUERS_5 = np.array([[-5., -3., L0], [-5., -1., L1], [-5., 1., L2],
                       [-5., 3., L1], [-5., 0., L0]])
EVADERS_5  = np.array([[0., -2., L1], [0., 0., L1], [0., 2., L1]])
CASES = {'3v3': (PURSUERS_3, EVADERS_3), '5v3': (PURSUERS_5, EVADERS_5)}

METHODS = ['greedy', 'hungarian_dummy', 'hungarian_aug']
METHOD_LABEL = {'greedy': 'Greedy', 'hungarian_dummy': 'Hungarian + dummy (card)',
                'hungarian_aug': 'Hungarian augmented'}
SHORT = {'greedy': 'Greedy', 'hungarian_dummy': 'Hung.\ndummy', 'hungarian_aug': 'Hung.\naug'}
LAYOUTS = ['layered', 'flat']

MC_TRIALS = 200
MC_SEED   = 17
MC_P_BOX  = ((-7.0, -3.0), (-4.0, 4.0))   # pursuer x, y
MC_E_BOX  = ((-1.0, 1.0), (-3.0, 3.0))    # evader x, y
MC_Z_BOX  = (Z_LAYERS[0], Z_LAYERS[-1])   # layered starts: z uniform in [1, 4]

PURSUER_COLORS = ['#e63946', '#f77f00', '#6a040f', '#ffba08', '#ff4d6d']
EVADER_COLORS  = ['#023e8a', '#0077b6', '#00b4d8']


def flatten(P):
    Q = P.copy(); Q[:, 2] = Z_FLAT
    return Q


# ── Assignment ────────────────────────────────────────────────────────────────

def cost_matrix(P, E):
    """Card: full 3D Euclidean distance."""
    return np.linalg.norm(P[:, None, :] - E[None, :, :], axis=2)


def greedy_assignment(P, E):
    C = cost_matrix(P, E)
    assignment, taken = {}, set()
    for i in range(len(P)):
        free = [j for j in range(len(E)) if j not in taken]
        cand = free if free else list(range(len(E)))
        j = cand[int(np.argmin(C[i, cand]))]
        assignment[i] = j; taken.add(j)
    return assignment


def hungarian_dummy_assignment(P, E):
    """Card: pad with dummy columns of large (equal) cost; dummy rows get no evader."""
    C = cost_matrix(P, E)
    M, N = C.shape
    C_pad = np.hstack([C, np.full((M, max(M - N, 0)), 1e3)])
    rows, cols = linear_sum_assignment(C_pad)
    return {int(r): int(c) for r, c in zip(rows, cols) if c < N}


def hungarian_aug_assignment(P, E):
    C = cost_matrix(P, E)
    M, N = C.shape
    rows, cols = linear_sum_assignment(np.tile(C, (1, int(np.ceil(M / N)))))
    return {int(r): int(c) % N for r, c in zip(rows, cols)}


ASSIGNERS = {'greedy': greedy_assignment, 'hungarian_dummy': hungarian_dummy_assignment,
             'hungarian_aug': hungarian_aug_assignment}


# ── Motion ────────────────────────────────────────────────────────────────────

def dive_speed(p, e):
    dz = p[2] - e[2]
    if dz > DIVE_DZ_MIN:
        dist = np.linalg.norm(e - p) + 1e-9
        return min(V_PURSUER * (1.0 + ETA_DIVE * dz / dist), DIVE_CAP * V_PURSUER)
    return V_PURSUER


def evader_velocity(e, P):
    """Away from the pursuer centroid + bias to the layer farthest from every pursuer."""
    away = e - P.mean(axis=0)
    n = np.linalg.norm(away)
    base = away / n * V_EVADER if n > 1e-9 else np.array([V_EVADER, 0.0, 0.0])
    z_esc = Z_LAYERS[int(np.argmax([np.min(np.abs(P[:, 2] - z)) for z in Z_LAYERS]))]
    v = base + np.array([0.0, 0.0, K_Z_ESCAPE * (z_esc - e[2])])
    return v / np.linalg.norm(v) * V_EVADER


def nearest_remaining(p, E, captured):
    rem = [j for j in range(len(E)) if not captured[j]]
    if not rem:
        return None
    return rem[int(np.argmin([np.linalg.norm(p - E[j]) for j in rem]))]


def simulate(P0, E0, method, record=True):
    P = P0.astype(float).copy(); E = E0.astype(float).copy()
    M, N = len(P), len(E)
    captured = [False] * N
    cap_t, cap_step, capturer = [None] * N, [None] * N, [None] * N
    assignment = ASSIGNERS[method](P, E)
    initial = dict(assignment)
    traj_p, traj_e = [P.copy()], [E.copy()]
    dive_steps = 0
    for step in range(int(round(T_MAX / DT))):
        t = (step + 1) * DT
        for j in range(N):
            if not captured[j]:
                E[j] = E[j] + evader_velocity(E[j], P) * DT
                E[j, 2] = np.clip(E[j, 2], Z_MIN, Z_MAX)
        for i in range(M):
            tgt = assignment.get(i)
            if tgt is None or captured[tgt]:
                tgt = nearest_remaining(P[i], E, captured)
                if tgt is None:
                    break
                assignment[i] = tgt
            d = E[tgt] - P[i]
            n = np.linalg.norm(d)
            v = dive_speed(P[i], E[tgt])
            dive_steps += v > V_PURSUER
            if n > 1e-9:
                P[i] = P[i] + d / n * min(v * DT, n)
            P[i, 2] = np.clip(P[i, 2], Z_MIN, Z_MAX)
            if np.linalg.norm(P[i] - E[tgt]) < CAPTURE_R and not captured[tgt]:
                captured[tgt] = True
                cap_t[tgt], cap_step[tgt], capturer[tgt] = t, step, i
        if record:
            traj_p.append(P.copy()); traj_e.append(E.copy())
        if all(captured):
            break
    valid = [x for x in cap_t if x is not None]
    return {'capture_times': cap_t, 'capture_steps': cap_step, 'capturer': capturer,
            'initial': initial, 'mission_time': max(valid) if len(valid) == N else np.nan,
            'traj_p': np.array(traj_p), 'traj_e': np.array(traj_e),
            'dive_frac': dive_steps / max(1, M * (step + 1))}


def sum_cost(P, E, assignment):
    C = cost_matrix(P, E)
    return sum(C[i, j] for i, j in assignment.items())


# ── Experiments ───────────────────────────────────────────────────────────────

def run_card_cases():
    res = {}
    print('Card geometry (layered = card starts, flat = every agent at z = 2.5 m):')
    for case, (P, E) in CASES.items():
        for lay in LAYOUTS:
            P0, E0 = (P, E) if lay == 'layered' else (flatten(P), flatten(E))
            for m in METHODS:
                r = simulate(P0, E0, m)
                res[(case, lay, m)] = r
                init = ', '.join(f'P{i+1}->E{j+1}' for i, j in sorted(r['initial'].items()))
                caps = ', '.join(f'E{j+1}:{r["capture_times"][j]:.2f}s(P{r["capturer"][j]+1})'
                                 for j in range(3))
                zr = r['traj_e'][:, :, 2]
                print(f'  {case} {lay:7s} {m:16s} | T={r["mission_time"]:.2f}s | '
                      f'sum dist {sum_cost(P0, E0, r["initial"]):.2f} m | init {init} | {caps} | '
                      f'evader z {zr.min():.2f}-{zr.max():.2f} m | dive {100 * r["dive_frac"]:.0f}% of steps')
    return res


def run_monte_carlo():
    rng = np.random.default_rng(MC_SEED)
    out = {(c, lay, m): [] for c in CASES for lay in LAYOUTS for m in METHODS}
    for _ in range(MC_TRIALS):
        E = np.column_stack([rng.uniform(*MC_E_BOX[0], 3), rng.uniform(*MC_E_BOX[1], 3),
                             rng.uniform(*MC_Z_BOX, 3)])
        for case, (Pc, _) in CASES.items():
            M = len(Pc)
            P = np.column_stack([rng.uniform(*MC_P_BOX[0], M), rng.uniform(*MC_P_BOX[1], M),
                                 rng.uniform(*MC_Z_BOX, M)])
            for lay in LAYOUTS:
                P0, E0 = (P, E) if lay == 'layered' else (flatten(P), flatten(E))
                for m in METHODS:
                    out[(case, lay, m)].append(simulate(P0, E0, m, record=False)['mission_time'])
    out = {k: np.array(v) for k, v in out.items()}
    tol = 1e-9
    print(f'\nMonte Carlo: {MC_TRIALS} random start sets (seed {MC_SEED}); pursuers x{MC_P_BOX[0]} '
          f'y{MC_P_BOX[1]}, evaders x{MC_E_BOX[0]} y{MC_E_BOX[1]}, layered z in {MC_Z_BOX}; '
          'flat = same x-y at z = 2.5')
    for case in CASES:
        for lay in LAYOUTS:
            for m in METHODS:
                v = out[(case, lay, m)]
                print(f'  {case} {lay:7s} {m:16s} | all captured {np.sum(~np.isnan(v))}/{len(v)} | '
                      f'mean {np.nanmean(v):.3f} s  std {np.nanstd(v):.3f}  median {np.nanmedian(v):.3f}  '
                      f'min {np.nanmin(v):.2f}  max {np.nanmax(v):.2f}')
            g, h, d = (out[(case, lay, 'greedy')], out[(case, lay, 'hungarian_aug')],
                       out[(case, lay, 'hungarian_dummy')])
            print(f'  {case} {lay:7s} Hungarian-aug vs greedy: faster {np.sum(h < g - tol)}, '
                  f'tie {np.sum(np.abs(h - g) <= tol)}, slower {np.sum(h > g + tol)}; '
                  f'mean diff (greedy - aug) {np.mean(g - h):+.3f} s')
            if case == '5v3':
                print(f'  {case} {lay:7s} Hungarian-aug vs dummy: faster {np.sum(h < d - tol)}, '
                      f'tie {np.sum(np.abs(h - d) <= tol)}, slower {np.sum(h > d + tol)}; '
                      f'mean diff (dummy - aug) {np.mean(d - h):+.3f} s')
        for m in METHODS:
            a, b = out[(case, 'layered', m)], out[(case, 'flat', m)]
            print(f'  {case} {m:16s} layered vs flat: mean {np.mean(a):.3f} vs {np.mean(b):.3f} s, '
                  f'layered faster in {np.sum(a < b - tol)}, slower in {np.sum(a > b + tol)}')
    return out


def run_simulation():
    print('S017 3D Swarm vs Swarm — Simulation Results')
    print('=' * 70)
    return {'card': run_card_cases(), 'mc': run_monte_carlo()}


# ── Plots ─────────────────────────────────────────────────────────────────────

def plot_trajectories_3d(card, out_dir):
    fig = plt.figure(figsize=(19, 12))
    for r, case in enumerate(CASES):
        for c, m in enumerate(METHODS):
            res = card[(case, 'layered', m)]
            ax = fig.add_subplot(2, 3, r * 3 + c + 1, projection='3d')
            tp, te = res['traj_p'], res['traj_e']
            for i in range(tp.shape[1]):
                col = PURSUER_COLORS[i]
                ax.plot(tp[:, i, 0], tp[:, i, 1], tp[:, i, 2], color=col, lw=1.4, label=f'P{i+1}')
                ax.scatter(*tp[0, i], color=col, s=40, marker='o', edgecolors='black')
            for j in range(3):
                col, cs = EVADER_COLORS[j], res['capture_steps'][j]
                end = cs + 2 if cs is not None else len(te)
                ax.plot(te[:end, j, 0], te[:end, j, 1], te[:end, j, 2], color=col, lw=1.4, ls='--',
                        label=f'E{j+1}')
                ax.scatter(*te[0, j], color=col, s=50, marker='^', edgecolors='black')
                if cs is not None:
                    ax.scatter(*te[cs + 1, j], color=col, s=80, marker='x', linewidths=2)
            init = ', '.join(f'P{i+1}>E{j+1}' for i, j in sorted(res['initial'].items()))
            ax.set_title(f'{case} — {METHOD_LABEL[m]}: T = {res["mission_time"]:.2f} s\n{init}',
                         fontsize=8)
            ax.set_xlim(-7, 7); ax.set_ylim(-6, 6); ax.set_zlim(0, 6)
            ax.set_xlabel('X', fontsize=7); ax.set_ylabel('Y', fontsize=7); ax.set_zlabel('Z', fontsize=7)
            ax.tick_params(labelsize=6); ax.legend(fontsize=6, loc='upper left', ncol=2)
    fig.suptitle('S017 3D — card geometry, layered (circle = pursuer start, triangle = evader start, '
                 'x = capture)', fontsize=11)
    plt.tight_layout()
    path = os.path.join(out_dir, 'trajectories_3d.png')
    plt.savefig(path, dpi=120); plt.close(); print(f'Saved: {path}')


def plot_altitude_layers(card, out_dir):
    """Altitude time series and pursuers per layer (nearest layer) — card, Hungarian augmented."""
    fig, axes = plt.subplots(2, 2, figsize=(15, 9), sharex='col')
    for c, case in enumerate(CASES):
        res = card[(case, 'layered', 'hungarian_aug')]
        tp, te = res['traj_p'], res['traj_e']
        t = np.arange(len(tp)) * DT
        ax = axes[0, c]
        for i in range(tp.shape[1]):
            ax.plot(t, tp[:, i, 2], color=PURSUER_COLORS[i], lw=1.8, label=f'P{i+1}')
        for j in range(3):
            cs = res['capture_steps'][j]
            end = cs + 2 if cs is not None else len(t)
            ax.plot(t[:end], te[:end, j, 2], color=EVADER_COLORS[j], lw=1.8, ls='--', label=f'E{j+1}')
            if cs is not None:
                ax.axvline(res['capture_times'][j], color=EVADER_COLORS[j], ls=':', lw=1)
        for z in Z_LAYERS:
            ax.axhline(z, color='grey', lw=0.7, ls='-.')
        ax.set_ylim(0, 6); ax.set_ylabel('z (m)'); ax.grid(True, alpha=0.3)
        ax.legend(fontsize=7, ncol=2, loc='upper left')
        ax.set_title(f'{case} Hungarian augmented — altitude vs time (dotted = capture)', fontsize=9)
        ax = axes[1, c]
        layer_idx = np.argmin(np.abs(tp[:, :, 2, None] - np.array(Z_LAYERS)[None, None, :]), axis=2)
        for k, z in enumerate(Z_LAYERS):
            ax.plot(t, (layer_idx == k).sum(axis=1), lw=1.8, label=f'layer z={z} m')
        ax.set_xlabel('Time (s)'); ax.set_ylabel('Pursuers nearest the layer')
        ax.set_ylim(-0.2, tp.shape[1] + 0.2); ax.grid(True, alpha=0.3); ax.legend(fontsize=7)
        ax.set_title(f'{case} — layer coverage', fontsize=9)
    fig.suptitle('S017 3D — altitude use, card geometry', fontsize=11)
    plt.tight_layout()
    path = os.path.join(out_dir, 'altitude_layers.png')
    plt.savefig(path, dpi=130); plt.close(); print(f'Saved: {path}')


def plot_gantt(card, out_dir):
    fig, axes = plt.subplots(1, 2, figsize=(15, 5))
    h = 0.25
    alphas, hatches = [0.95, 0.6, 0.35], ['', '//', '..']
    for ax, case in zip(axes, CASES):
        for k, m in enumerate(METHODS):
            res = card[(case, 'layered', m)]
            for j in range(3):
                t = res['capture_times'][j]
                y = j + (k - 1) * (h + 0.03)
                ax.barh(y, t, height=h, color=EVADER_COLORS[j], alpha=alphas[k], hatch=hatches[k],
                        edgecolor='black')
                ax.text(t + 0.05, y, f'{t:.2f}s (P{res["capturer"][j]+1})', va='center', fontsize=7)
        handles = [Patch(facecolor='grey', alpha=alphas[k], hatch=hatches[k], edgecolor='black',
                         label=f'{METHOD_LABEL[m]} (T={card[(case, "layered", m)]["mission_time"]:.2f}s)')
                   for k, m in enumerate(METHODS)][::-1]
        ax.set_yticks(range(3)); ax.set_yticklabels(['E1', 'E2', 'E3'])
        ax.set_xlim(0, 6.0); ax.set_xlabel("Capture time (s)")
        ax.set_title(f'{case} layered — capture time per evader (capturing pursuer)', fontsize=9)
        ax.legend(handles=handles, fontsize=7, loc='lower right', title='bars top to bottom',
                  title_fontsize=7)
        ax.grid(True, alpha=0.25, axis='x')
    plt.tight_layout()
    path = os.path.join(out_dir, 'gantt_chart.png')
    plt.savefig(path, dpi=140); plt.close(); print(f'Saved: {path}')


def plot_mission_time_comparison(card, mc, out_dir):
    fig, axes = plt.subplots(1, 2, figsize=(17, 6.5))
    colors = {'greedy': '#f4a261', 'hungarian_dummy': '#adb5bd', 'hungarian_aug': '#2a9d8f'}
    keys = [(c, lay, m) for c in CASES for lay in LAYOUTS for m in METHODS]
    labels = [f'{c} {lay}\n{SHORT[m]}' for c, lay, m in keys]
    vals = [card[k]['mission_time'] for k in keys]
    bars = axes[0].bar(range(len(vals)), vals, color=[colors[k[2]] for k in keys], edgecolor='black',
                       hatch=None)
    for b, k in zip(bars, keys):
        if k[1] == 'flat':
            b.set_hatch('//')
    for b, v in zip(bars, vals):
        axes[0].text(b.get_x() + b.get_width() / 2, v + 0.03, f'{v:.2f}', ha='center', fontsize=7)
    axes[0].set_xticks(range(len(vals))); axes[0].set_xticklabels(labels, fontsize=6)
    axes[0].set_ylabel('T_mission (s)'); axes[0].set_ylim(0, max(vals) * 1.2)
    axes[0].set_title('Card geometry (hatched = flat, all agents at z = 2.5 m)', fontsize=10)
    axes[0].grid(True, alpha=0.3, axis='y')
    data = [mc[k] for k in keys]
    bp = axes[1].boxplot(data, patch_artist=True, showmeans=True,
                         meanprops=dict(marker='D', markerfacecolor='white', markeredgecolor='black'))
    for patch, k in zip(bp['boxes'], keys):
        patch.set_facecolor(colors[k[2]])
        if k[1] == 'flat':
            patch.set_hatch('//')
    for i, v in enumerate(data):
        axes[1].text(i + 1, np.mean(v), f' {np.mean(v):.2f}', fontsize=7, va='bottom')
    axes[1].set_xticks(range(1, len(data) + 1)); axes[1].set_xticklabels(labels, fontsize=6)
    axes[1].set_ylabel('T_mission (s)')
    axes[1].set_title(f'Monte Carlo, {MC_TRIALS} random start sets (diamond = mean)', fontsize=10)
    axes[1].grid(True, alpha=0.3, axis='y')
    fig.suptitle('S017 3D — mission time (last capture): assignment method, layered vs flat', fontsize=11)
    plt.tight_layout()
    path = os.path.join(out_dir, 'mission_time_comparison.png')
    plt.savefig(path, dpi=130); plt.close(); print(f'Saved: {path}')


def save_animation(card, out_dir):
    """5v3 layered, the three assignment methods side by side."""
    runs = [card[('5v3', 'layered', m)] for m in METHODS]
    max_len = max(len(r['traj_p']) for r in runs)
    step = 3
    frames = (max_len - 1) // step + 2     # last frame shows the final capture
    fig = plt.figure(figsize=(16, 6))
    arts = []
    for k, (m, res) in enumerate(zip(METHODS, runs)):
        ax = fig.add_subplot(1, 3, k + 1, projection='3d')
        ax.set_xlim(-7, 7); ax.set_ylim(-6, 6); ax.set_zlim(0, 6)
        ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
        M = res['traj_p'].shape[1]
        pl = [ax.plot([], [], [], color=PURSUER_COLORS[i], lw=1.2)[0] for i in range(M)]
        pd = [ax.plot([], [], [], 'o', color=PURSUER_COLORS[i], mec='black', ms=6)[0] for i in range(M)]
        el = [ax.plot([], [], [], '--', color=EVADER_COLORS[j], lw=1.2)[0] for j in range(3)]
        ed = [ax.plot([], [], [], '^', color=EVADER_COLORS[j], mec='black', ms=7)[0] for j in range(3)]
        arts.append((ax, m, res, pl, pd, el, ed))

    def update(f):
        si = min(f * step, max_len - 1)
        out = []
        for ax, m, res, pl, pd, el, ed in arts:
            tp, te = res['traj_p'], res['traj_e']
            n = min(si + 1, len(tp))
            for i in range(tp.shape[1]):
                pl[i].set_data_3d(tp[:n, i, 0], tp[:n, i, 1], tp[:n, i, 2])
                pd[i].set_data_3d([tp[n-1, i, 0]], [tp[n-1, i, 1]], [tp[n-1, i, 2]])
            for j in range(3):
                cs = res['capture_steps'][j]
                nj = n if cs is None else min(n, cs + 2)
                el[j].set_data_3d(te[:nj, j, 0], te[:nj, j, 1], te[:nj, j, 2])
                ed[j].set_data_3d([te[nj-1, j, 0]], [te[nj-1, j, 1]], [te[nj-1, j, 2]])
                ed[j].set_marker('x' if cs is not None and si >= cs + 1 else '^')
            ax.set_title(f'5v3 {METHOD_LABEL[m]}\nt = {min(si * DT, res["mission_time"]):.2f} s',
                         fontsize=9)
            out += pl + pd + el + ed
        return out

    ani = FuncAnimation(fig, update, frames=frames, interval=60, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer=PillowWriter(fps=16), dpi=75)
    plt.close(); print(f'Saved: {path}  ({frames} frames)')


if __name__ == '__main__':
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    data = run_simulation()
    plot_trajectories_3d(data['card'], OUTPUT_DIR)
    plot_altitude_layers(data['card'], OUTPUT_DIR)
    plot_gantt(data['card'], OUTPUT_DIR)
    plot_mission_time_comparison(data['card'], data['mc'], OUTPUT_DIR)
    save_animation(data['card'], OUTPUT_DIR)
    print('Done.')
