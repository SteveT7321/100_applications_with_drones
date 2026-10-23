"""
S014 3D Upgrade — Decoy & Lure
===============================
One pursuer must decide which of two tracks is the real evader.

  Evader : flees horizontally away from the pursuer at 3 m/s; once the decoy
           starts climbing (t = T_LURE) it also dives toward z = 0.8 m (card).
  Decoy  : 2 m/s toward the pursuer in XY (lure, t < 3 s), then toward the
           pursuer while climbing to z = 5 m (altitude trap, 3–6 s), then
           stops (card).  Its displayed position carries N(0, σ_decoy²) noise
           in x, y (card ε_xy); the evader is observed with σ_obs = 0.05 m.
  Flankers (allied with the evader): hold until the pursuer climbs above
           3.5 m, then fly to p_P + [0, 0, 1.5] and p_P + [0, 2, 0] (card).

Classifiers (card), updated every 0.5 s on the two observed tracks A, B:
  2D (jerk only)      Λ += λ_s·(s_B − s_A)
  3D (jerk + altitude) Λ += λ_s·(s_B − s_A) + λ_z·(z_B − z_A)
s = mean |Δv|/Δt over the last 10 steps (card 3D jerk proxy).  The real
evader is on track A or B at random per seed and the pursuer flies toward
the midpoint of the tracks until the first update (neutral start, 2D fix).
Bounds: misidentification (always chases the decoy) and oracle.

Usage:
    conda activate drones
    python src/01_pursuit_evasion/3d/s014_3d_decoy_lure.py
"""
import sys, os, numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from matplotlib.animation import FuncAnimation, PillowWriter

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..'))

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..', '..', 'outputs',
    '01_pursuit_evasion', '3d', 's014_3d_decoy_lure',
)
os.makedirs(OUTPUT_DIR, exist_ok=True)

# ── Parameters (card) ─────────────────────────────────────────
SIGMA_DECOY  = 0.4     # m  extra decoy displayed-position noise in x, y (card ε_xy); 0 = equal-noise case
SIGMA_OBS    = 0.05    # m  sensor noise on both tracks, all axes (not in card)
T_LURE       = 3.0     # s
T_TRAP       = 6.0     # s
Z_TRAP       = 5.0     # m
Z_ESCAPE     = 0.8     # m
Z_FLANK_ACT  = 3.5     # m
LAMBDA_S     = 0.5     # jerk weight
LAMBDA_Z     = 0.3     # altitude weight
T_UPDATE     = 0.5     # s
JERK_WINDOW  = 10      # steps (card)
V_PURSUER    = 5.0
V_EVADER     = 3.0
V_DECOY      = 2.0
V_FLANKER    = 4.0     # m/s (not in card)
DECOY_CLIMB  = 1.5     # m/s before renormalisation (card)
CAPTURE_R    = 0.15
DT           = 0.05
MAX_TIME     = 25.0
Z_MIN, Z_MAX = 0.5, 6.0

INIT_P  = np.array([0.0, -8.0, 2.0])
INIT_E  = np.array([0.0,  0.0, 2.0])
INIT_D  = INIT_E + np.array([0.5, 0.3, 0.0])
INIT_F  = np.array([[-3.0, 0.0, 2.0], [3.0, 0.0, 2.0]])   # flanker starts (not in card)
FLANK_OFFSETS = np.array([[0.0, 0.0, 1.5], [0.0, 2.0, 0.0]])

T_COMMIT = 4.0   # s  'hold' variants hover until the altitude cue exists (Δz > 2 m at 3.60 s)
STRATEGIES = ['jerk_2d', 'jerk_alt_3d', 'jerk_2d_hold', 'jerk_alt_3d_hold', 'alt_only_hold', 'misid', 'oracle']
STRAT_LABEL = {'jerk_2d': '2D classifier (jerk only)', 'jerk_alt_3d': '3D classifier (jerk + altitude)',
               'jerk_2d_hold': '2D classifier, commit at 4 s', 'jerk_alt_3d_hold': '3D classifier, commit at 4 s',
               'alt_only_hold': 'Altitude term only (λ_s = 0), commit at 4 s',
               'misid': 'Misidentification (chases decoy)', 'oracle': 'Oracle (perfect ID)'}
STRAT_COLOR = {'jerk_2d': 'darkorange', 'jerk_alt_3d': 'red', 'jerk_2d_hold': 'peru',
               'jerk_alt_3d_hold': 'firebrick', 'alt_only_hold': 'purple', 'misid': 'grey', 'oracle': 'black'}
N_SEEDS = 30
MAIN_SEED = 0


# ── Agent motion (card) ───────────────────────────────────────

def decoy_velocity(t, pos_d, pos_p):
    to_p = pos_p[:2] - pos_d[:2]
    to_p = to_p / (np.linalg.norm(to_p) + 1e-9)
    if t < T_LURE:
        vel = np.array([to_p[0], to_p[1], 0.0])
    elif t < T_TRAP:
        climb = DECOY_CLIMB if pos_d[2] < Z_TRAP else 0.0
        vel = np.array([to_p[0], to_p[1], climb])
    else:
        return np.zeros(3)
    return V_DECOY * vel / np.linalg.norm(vel)


def evader_velocity(t, pos_e, pos_p):
    away = pos_e[:2] - pos_p[:2]
    away = away / np.linalg.norm(away) if np.linalg.norm(away) > 1e-9 else np.array([1.0, 0.0])
    dz = Z_ESCAPE - pos_e[2] if t >= T_LURE else 0.0
    vel = np.array([away[0], away[1], dz])
    return V_EVADER * vel / np.linalg.norm(vel)


def flanker_velocity(pos_f, pos_p, k, active):
    if not active:
        return np.zeros(3)
    d = pos_p + FLANK_OFFSETS[k] - pos_f
    n = np.linalg.norm(d)
    return d / n * min(V_FLANKER, n / DT) if n > 1e-9 else np.zeros(3)


def jerk_proxy(vel_hist):
    """Card: mean |v(t) − v(t − Δt)| / Δt over the last JERK_WINDOW steps."""
    v = np.array(vel_hist[-JERK_WINDOW:])
    if len(v) < 2:
        return 0.0, 0.0
    a = np.diff(v, axis=0) / DT
    return float(np.linalg.norm(a, axis=1).mean()), float(np.abs(a[:, 2]).mean())


# ── Simulation ────────────────────────────────────────────────

def run_simulation(strategy='jerk_alt_3d', seed=MAIN_SEED, sigma_decoy=SIGMA_DECOY,
                   flankers=True, log=True):
    rng = np.random.default_rng(seed)
    real_is_a = bool(rng.random() < 0.5)     # random track labels
    pos_p, pos_e, pos_d = INIT_P.copy(), INIT_E.copy(), INIT_D.copy()
    pos_f = INIT_F.copy()
    flank_active = False
    obs = {'A': None, 'B': None}
    vel_hist = {'A': [], 'B': []}
    lam, last_upd = 0.0, 0.0
    L = dict(t=[0.0], p=[pos_p.copy()], e=[pos_e.copy()], d=[pos_d.copy()], f=[pos_f.copy()],
             p_real=[0.5], lam=[0.0])
    outcome, end_t = 'timeout', MAX_TIME
    min_flank = np.inf
    flank_t = None

    for step in range(1, int(round(MAX_TIME / DT)) + 1):
        t = step * DT
        # true motion (all agents react to the current pursuer position)
        v_e = evader_velocity(t, pos_e, pos_p)
        v_d = decoy_velocity(t, pos_d, pos_p)
        pos_e = pos_e + v_e * DT; pos_e[2] = np.clip(pos_e[2], Z_MIN, Z_MAX)
        pos_d = pos_d + v_d * DT; pos_d[2] = np.clip(pos_d[2], Z_MIN, Z_MAX)

        # observations (noise drawn in a fixed order -> common random numbers)
        n_e = rng.normal(0, SIGMA_OBS, 3)
        n_d = rng.normal(0, SIGMA_OBS, 3) + np.array([*rng.normal(0, sigma_decoy, 2), 0.0])
        o_e, o_d = pos_e + n_e, pos_d + n_d
        new = {'A': o_e if real_is_a else o_d, 'B': o_d if real_is_a else o_e}
        for k in 'AB':
            if obs[k] is not None:
                vel_hist[k].append((new[k] - obs[k]) / DT)
            obs[k] = new[k]

        if t - last_upd >= T_UPDATE - 1e-9:
            last_upd = t
            sA, _ = jerk_proxy(vel_hist['A']); sB, _ = jerk_proxy(vel_hist['B'])
            if not strategy.startswith('alt_only'):
                lam += LAMBDA_S * (sB - sA)
            if strategy.startswith('jerk_alt_3d') or strategy.startswith('alt_only'):
                lam += LAMBDA_Z * (obs['B'][2] - obs['A'][2])
        p_a = 0.5 * (1 + np.tanh(0.5 * lam))
        p_real = p_a if real_is_a else 1 - p_a

        if strategy.endswith('_hold') and t < T_COMMIT:
            target = pos_p                     # hover while the classifier gathers evidence
        elif strategy == 'oracle':
            target = pos_e
        elif strategy == 'misid':
            target = o_d
        elif lam == 0.0:
            target = 0.5 * (obs['A'] + obs['B'])
        else:
            target = obs['A'] if lam > 0 else obs['B']
        d = target - pos_p
        n = np.linalg.norm(d)
        pos_p = pos_p + (V_PURSUER * d / n if n > 1e-9 else np.zeros(3)) * DT
        pos_p[2] = np.clip(pos_p[2], Z_MIN, Z_MAX)

        if flankers:
            if not flank_active and pos_p[2] > Z_FLANK_ACT:
                flank_active, flank_t = True, t
            for k in range(2):
                pos_f[k] = pos_f[k] + flanker_velocity(pos_f[k], pos_p, k, flank_active) * DT
            min_flank = min(min_flank, float(np.linalg.norm(pos_f - pos_p, axis=1).min()))

        if log:
            L['t'].append(t); L['p'].append(pos_p.copy()); L['e'].append(pos_e.copy())
            L['d'].append(pos_d.copy()); L['f'].append(pos_f.copy())
            L['p_real'].append(p_real); L['lam'].append(lam)
        if np.linalg.norm(pos_p - pos_e) < CAPTURE_R:
            outcome, end_t = 'evader', t
            break
        if np.linalg.norm(pos_p - pos_d) < CAPTURE_R:
            outcome, end_t = 'decoy', t
            break

    out = dict(outcome=outcome, end_t=end_t, real_is_a=real_is_a, max_pz=None, lam_final=lam,
               min_flank=min_flank, flank_t=flank_t, final_p_real=p_real)
    if log:
        out.update({k: np.array(v) for k, v in L.items()})
        out['max_pz'] = float(out['p'][:, 2].max())
    return out


def run_monte_carlo(sigma_decoy, flankers=True):
    res = {s: [run_simulation(s, seed, sigma_decoy, flankers, log=False) for seed in range(N_SEEDS)]
           for s in STRATEGIES}
    return res


def summarize(rs):
    ev = [r['end_t'] for r in rs if r['outcome'] == 'evader']
    return dict(ev=len(ev), dc=sum(r['outcome'] == 'decoy' for r in rs),
                to=sum(r['outcome'] == 'timeout' for r in rs),
                t_ev=float(np.mean(ev)) if ev else None,
                t_dc_max=max([r['end_t'] for r in rs if r['outcome'] == 'decoy'], default=None),
                flank=sum(r['flank_t'] is not None for r in rs),
                min_flank=float(min(r['min_flank'] for r in rs)))


def jerk_breakdown(sigma_decoy, seed=MAIN_SEED):
    """Card jerk proxy (3D and z-only) per phase, with the pursuer held at its
    start: motion only / noise only / observed."""
    rng = np.random.default_rng(seed)
    n = int(round(T_TRAP / DT))
    pe, pd = [INIT_E.copy()], [INIT_D.copy()]
    for step in range(1, n + 1):
        t = step * DT
        pe.append(np.clip(pe[-1] + evader_velocity(t, pe[-1], INIT_P) * DT, [-1e9, -1e9, Z_MIN], [1e9, 1e9, Z_MAX]))
        pd.append(np.clip(pd[-1] + decoy_velocity(t, pd[-1], INIT_P) * DT, [-1e9, -1e9, Z_MIN], [1e9, 1e9, Z_MAX]))
    pe, pd = np.array(pe), np.array(pd)
    ne = rng.normal(0, SIGMA_OBS, pe.shape)
    nd = rng.normal(0, SIGMA_OBS, pd.shape)
    nd[:, :2] += rng.normal(0, sigma_decoy, (len(pd), 2))

    def proxy(pos, lo, hi):
        a = np.diff(pos, 2, axis=0)[lo:hi] / DT ** 2
        return float(np.linalg.norm(a, axis=1).mean()), float(np.abs(a[:, 2]).mean())

    k = int(round(T_LURE / DT))
    out = {}
    for name, pos, nz in (('evader', pe, ne), ('decoy', pd, nd)):
        for ph, (lo, hi) in (('lure 0–3 s', (0, k - 1)), ('trap 3–6 s', (k - 1, n - 1))):
            out[(name, ph)] = (proxy(pos, lo, hi), proxy(nz, lo, hi), proxy(pos + nz, lo, hi))
    return out, pe, pd


# ── Plots ────────────────────────────────────────────────────

def _status(r):
    return {'evader': f"caught EVADER @ {r['end_t']:.2f} s", 'decoy': f"caught DECOY @ {r['end_t']:.2f} s",
            'timeout': 'timeout'}[r['outcome']]


def plot_trajectories_3d(panels, out_dir):
    fig = plt.figure(figsize=(15, 12))
    for j, (r, lab) in enumerate(panels):
        ax = fig.add_subplot(2, 2, j + 1, projection='3d'); ax.view_init(elev=18, azim=-50)
        ax.plot(*r['p'].T, color='red', lw=1.8, label='Pursuer')
        ax.plot(*r['e'].T, color='royalblue', lw=1.8, label='Evader (real)')
        ax.plot(*r['d'].T, color='green', lw=1.5, ls='--', label='Decoy')
        for k in range(2):
            ax.plot(*r['f'][:, k].T, color='purple', lw=1.0, ls=':', label='Flankers' if k == 0 else None)
        for arr, c in ((r['p'], 'red'), (r['e'], 'blue'), (r['d'], 'green')):
            ax.scatter(*arr[0], color=c, s=40)
        ax.scatter(*r['p'][-1], color='black', marker='X', s=100)
        ax.set_zlim(Z_MIN - 0.2, Z_MAX + 0.2)
        ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
        ax.set_title(f"{lab}\n{_status(r)}", fontsize=9)
        ax.legend(fontsize=7, loc='upper left')
    fig.suptitle('S014 3D Decoy & Lure — example runs', fontsize=11)
    plt.tight_layout()
    path = os.path.join(out_dir, 'trajectories_3d.png')
    plt.savefig(path, dpi=110, bbox_inches='tight'); plt.close('all'); print(f'Saved: {path}')


def plot_altitude_time(pe, pd, out_dir):
    t = np.arange(len(pe)) * DT
    fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    axes[0].plot(t, pe[:, 2], color='royalblue', lw=2, label='Evader z')
    axes[0].plot(t, pd[:, 2], color='green', lw=2, ls='--', label='Decoy z')
    axes[0].axhline(Z_TRAP, color='green', ls=':', lw=0.8); axes[0].axhline(Z_ESCAPE, color='blue', ls=':', lw=0.8)
    axes[0].axvline(T_LURE, color='orange', ls=':', lw=1.2, label='decoy starts climbing')
    axes[0].set_ylabel('z (m)'); axes[0].legend(fontsize=8); axes[0].grid(True, alpha=0.3)
    axes[0].set_title('S014 3D — card motions with the pursuer held at its start (V-shape)', fontsize=10)
    axes[1].plot(t, pd[:, 2] - pe[:, 2], color='purple', lw=2, label='Δz = z_decoy − z_evader')
    axes[1].axhline(2.0, color='grey', ls='--', lw=1, label='Δz = 2 m (card cue)')
    axes[1].axvline(T_LURE, color='orange', ls=':', lw=1.2)
    axes[1].set_xlabel('Time (s)'); axes[1].set_ylabel('Δz (m)'); axes[1].legend(fontsize=8)
    axes[1].grid(True, alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'altitude_time.png')
    plt.savefig(path, dpi=120, bbox_inches='tight'); plt.close('all'); print(f'Saved: {path}')


def plot_classification(runs_nom, runs_eq, out_dir, ex_seed):
    fig, axes = plt.subplots(1, 2, figsize=(15, 5.5), sharey=True)
    for ax, runs, title in ((axes[0], runs_nom, f'nominal (decoy extra σ = {SIGMA_DECOY} m), seed {MAIN_SEED}'),
                            (axes[1], runs_eq, f'equal noise (σ = {SIGMA_OBS} m on both), seed {ex_seed}')):
        for s, ls in (('jerk_2d', '-'), ('jerk_alt_3d', '--'), ('jerk_2d_hold', '-.'), ('jerk_alt_3d_hold', ':'),
                      ('alt_only_hold', '-')):
            r = runs[s]
            ax.plot(r['t'], r['p_real'], color=STRAT_COLOR[s], lw=1.8, ls=ls,
                    label=f"{STRAT_LABEL[s]}: {_status(r)}")
        ax.axvline(T_COMMIT, color='purple', ls=':', lw=1.0, label='hold variants commit')
        ax.axhline(0.5, color='grey', ls='--', lw=1); ax.axvline(T_LURE, color='orange', ls=':', lw=1.2,
                                                                   label='decoy starts climbing')
        ax.set_xlabel('Time (s)'); ax.set_title(title, fontsize=9); ax.grid(True, alpha=0.3)
        ax.legend(fontsize=6.5, loc='lower right'); ax.set_ylim(-0.03, 1.03)
    axes[0].set_ylabel('P(true evader track = real)')
    fig.suptitle('S014 3D — Classifier Belief', fontsize=11)
    plt.tight_layout()
    path = os.path.join(out_dir, 'classification_prob.png')
    plt.savefig(path, dpi=120, bbox_inches='tight'); plt.close('all'); print(f'Saved: {path}')


def plot_monte_carlo(summ, out_dir):
    fig, axes = plt.subplots(1, 2, figsize=(17, 6))
    x = np.arange(len(STRATEGIES)); w = 0.38
    for ax, key, ttl in ((axes[0], 'nominal', f'nominal (decoy extra σ = {SIGMA_DECOY} m)'),
                         (axes[1], 'equal', f'equal noise (decoy = evader, σ = {SIGMA_OBS} m)')):
        ev = [summ[key][s]['ev'] for s in STRATEGIES]
        dc = [summ[key][s]['dc'] for s in STRATEGIES]
        b1 = ax.bar(x - w / 2, ev, w, color='royalblue', label='caught evader')
        b2 = ax.bar(x + w / 2, dc, w, color='green', label='caught decoy')
        for bars in (b1, b2):
            for b in bars:
                ax.text(b.get_x() + b.get_width() / 2, b.get_height() + 0.3, f'{int(b.get_height())}',
                        ha='center', fontsize=8)
        ax.set_xticks(x); ax.set_xticklabels([STRAT_LABEL[s] for s in STRATEGIES], rotation=12,
                                             ha='right', fontsize=8)
        ax.set_ylim(0, N_SEEDS + 3); ax.set_ylabel(f'runs out of {N_SEEDS}')
        ax.set_title(ttl, fontsize=9); ax.legend(fontsize=8); ax.grid(True, axis='y', alpha=0.3)
    fig.suptitle(f'S014 3D — Monte Carlo, {N_SEEDS} seeds (random track labels, common random numbers)',
                 fontsize=11)
    plt.tight_layout()
    path = os.path.join(out_dir, 'monte_carlo.png')
    plt.savefig(path, dpi=110, bbox_inches='tight'); plt.close('all'); print(f'Saved: {path}')


def save_animation(r, out_dir, seed):
    t = r['t']
    frames = list(range(0, len(t), 2))
    if frames[-1] != len(t) - 1:
        frames.append(len(t) - 1)
    fig = plt.figure(figsize=(9, 7))
    ax = fig.add_subplot(111, projection='3d'); ax.view_init(elev=18, azim=-50)
    allp = np.vstack([r['p'], r['e'], r['d'], r['f'].reshape(-1, 3)])
    lo, hi = allp.min(0) - 0.5, allp.max(0) + 0.5
    ax.set_xlim(lo[0], hi[0]); ax.set_ylim(lo[1], hi[1]); ax.set_zlim(Z_MIN - 0.2, Z_MAX + 0.2)
    ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
    lines = {k: ax.plot([], [], [], color=c, lw=1.6, ls=ls)[0]
             for k, c, ls in (('p', 'red', '-'), ('e', 'royalblue', '-'), ('d', 'green', '--'))}
    fl = [ax.plot([], [], [], 'o', color='purple', ms=6)[0] for _ in range(2)]
    dots = {k: ax.plot([], [], [], m, color=c, ms=8)[0]
            for k, c, m in (('p', 'red', '^'), ('e', 'royalblue', 's'), ('d', 'green', 'D'))}
    title = ax.set_title('')

    def update(fi):
        si = frames[fi]
        for k in 'ped':
            arr = r[k]
            lines[k].set_data(arr[:si+1, 0], arr[:si+1, 1]); lines[k].set_3d_properties(arr[:si+1, 2])
            dots[k].set_data([arr[si, 0]], [arr[si, 1]]); dots[k].set_3d_properties([arr[si, 2]])
        for k in range(2):
            fl[k].set_data([r['f'][si, k, 0]], [r['f'][si, k, 1]]); fl[k].set_3d_properties([r['f'][si, k, 2]])
        title.set_text(f"S014 3D, equal noise, seed {seed}, 3D classifier committing at 4 s  t={t[si]:.2f}s\n"
                       f"P(true evader track)={r['p_real'][si]:.2f}"
                       + (f"  {_status(r)}" if fi == len(frames) - 1 else ''))
        return list(lines.values()) + list(dots.values()) + fl + [title]

    ani = FuncAnimation(fig, update, frames=len(frames), interval=70, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer=PillowWriter(fps=12), dpi=90); plt.close('all')
    print(f'Saved: {path}  ({len(frames)} frames)')


# ── Main ─────────────────────────────────────────────────────

if __name__ == '__main__':
    print(f'S014 3D Decoy & Lure — main runs, seed {MAIN_SEED}')
    runs_nom = {s: run_simulation(s, MAIN_SEED) for s in STRATEGIES}
    runs_eq = {s: run_simulation(s, MAIN_SEED, 0.0) for s in STRATEGIES}
    print(f"  real evader on track {'A' if runs_nom['oracle']['real_is_a'] else 'B'}")
    for lab, runs in (('nominal', runs_nom), ('equal noise', runs_eq)):
        for s in STRATEGIES:
            r = runs[s]
            print(f"  [{lab:<11}] {STRAT_LABEL[s]:<34} {_status(r):<26} max pursuer z {r['max_pz']:.2f} m; "
                  f"flankers {'activated @ %.2f s' % r['flank_t'] if r['flank_t'] else 'never activated'}; "
                  f"min flanker distance {r['min_flank']:.2f} m")

    bd, pe, pd = jerk_breakdown(SIGMA_DECOY)
    bd_eq, _, _ = jerk_breakdown(0.0)
    print(f'\nCard motions with the pursuer held at its start: decoy peak z {pd[:, 2].max():.2f} m, '
          f'evader min z {pe[:, 2].min():.2f} m, Δz at 6 s {pd[-1, 2] - pe[-1, 2]:.2f} m, '
          f'Δz first > 2 m at {np.argmax(pd[:, 2] - pe[:, 2] > 2.0) * DT:.2f} s')
    print('Jerk proxy (m/s^2), 3D | z-only: motion only / noise only / observed')
    for lab, b in ((f'decoy extra sigma {SIGMA_DECOY} m', bd), ('decoy at evader noise', bd_eq)):
        for (name, ph), (m, nz, ob) in b.items():
            if lab == 'decoy at evader noise' and name == 'evader':
                continue
            print(f'  {name:<6} {ph:<11} [{lab}]  3D {m[0]:8.1f} / {nz[0]:8.1f} / {ob[0]:8.1f}   '
                  f'z {m[1]:7.1f} / {nz[1]:7.1f} / {ob[1]:7.1f}')

    print(f'\nMonte Carlo: {N_SEEDS} seeds, random track labels, common random numbers '
          f'(nominal: decoy extra xy noise {SIGMA_DECOY} m; equal: none, both tracks sigma {SIGMA_OBS} m)')
    summ, mcs = {}, {}
    for key, sig in (('nominal', SIGMA_DECOY), ('equal', 0.0)):
        mc = mcs[key] = run_monte_carlo(sig)
        summ[key] = {s: summarize(mc[s]) for s in STRATEGIES}
        for s in STRATEGIES:
            m = summ[key][s]
            te = f"{m['t_ev']:.2f} s" if m['t_ev'] is not None else '-'
            print(f"  [{key:<7} σ_decoy={sig}] {STRAT_LABEL[s]:<34} evader {m['ev']:2d}/{N_SEEDS}  "
                  f"decoy {m['dc']:2d}/{N_SEEDS}  timeout {m['to']:2d}  mean evader capture {te}; "
                  f"latest decoy capture {'%.2f s' % m['t_dc_max'] if m['t_dc_max'] is not None else '-'}; "
                  f"flankers activated in {m['flank']} runs, min flanker distance {m['min_flank']:.2f} m")
    nf = run_monte_carlo(0.0, flankers=False)
    eqf = mcs['equal']
    same = all(a['outcome'] == b['outcome'] and a['end_t'] == b['end_t']
               for s in STRATEGIES for a, b in zip(nf[s], eqf[s]))
    print(f'  flankers removed (equal noise): outcomes and times identical in all runs: {same}')

    ex_seed = next(i for i, r in enumerate(mcs['equal']['jerk_alt_3d_hold']) if r['outcome'] == 'decoy')
    runs_ex = {s: run_simulation(s, ex_seed, 0.0) for s in STRATEGIES}
    print(f'\nExample equal-noise seed {ex_seed} (first seed where the 3D classifier committing at 4 s '
          f'catches the decoy):')
    for s in STRATEGIES:
        r = runs_ex[s]
        print(f"  {STRAT_LABEL[s]:<44} {_status(r):<26} max pursuer z {r['max_pz']:.2f} m; flankers "
              f"{'activated @ %.2f s' % r['flank_t'] if r['flank_t'] else 'never activated'}; "
              f"min flanker distance {r['min_flank']:.2f} m")

    plot_trajectories_3d([(runs_nom['jerk_alt_3d'], f'nominal noise, seed {MAIN_SEED}: ' + STRAT_LABEL['jerk_alt_3d']),
                          (runs_nom['misid'], f'nominal noise, seed {MAIN_SEED}: ' + STRAT_LABEL['misid']),
                          (runs_ex['jerk_alt_3d_hold'], f'equal noise, seed {ex_seed}: ' + STRAT_LABEL['jerk_alt_3d_hold']),
                          (runs_ex['alt_only_hold'], f'equal noise, seed {ex_seed}: ' + STRAT_LABEL['alt_only_hold'])],
                         OUTPUT_DIR)
    plot_altitude_time(pe, pd, OUTPUT_DIR)
    plot_classification(runs_nom, runs_ex, OUTPUT_DIR, ex_seed)
    plot_monte_carlo(summ, OUTPUT_DIR)
    save_animation(runs_ex['jerk_alt_3d_hold'], OUTPUT_DIR, ex_seed)
    print('All outputs saved to:', OUTPUT_DIR)
