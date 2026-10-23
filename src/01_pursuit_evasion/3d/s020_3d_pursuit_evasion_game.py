"""
S020 3D Upgrade — Pursuit-Evasion Game (RL Capstone), hand-crafted benchmark
==========================================================================
The card's 3D game: a 5 m/s pursuer against a 3.5 m/s evader with 3D unit-sphere
velocity commands, capture radius 0.15 m, 30 s episodes and observation noise
sigma = 0.1 on every observed position and velocity component.  No PPO training is
run here (see the README); the card's benchmark grid is run with hand-crafted
policies on both sides:

Pursuers (all fly at exactly v_P):
  PP        pure pursuit at the observed evader position
  PN        3D proportional navigation, N = 3, constant speed, launched along
            the initial line of sight (the S016 / S020 2D fix)
  APF       attraction to the evader + repulsion from every arena face closer
            than d0 (a genuine potential field, not pure pursuit renamed)
  Lead      pure pursuit at the predicted position e + v_e * 0.3 s
  AltAdv    pure pursuit with a climb bias until 1 m above the evader
Evaders (all fly at exactly v_E):
  straight       fixed random 3D heading (elevation within +-30 deg)
  perpendicular  horizontal, perpendicular to the line of sight (fixed side)
  spiral         helix: horizontal heading rotating at 0.8 rad/s plus a
                 vertical component 0.5 sin(0.8 t + phase)
  alt_escape     the card's altitude escape: away from the pursuer horizontally
                 while climbing / diving to the altitude limit on the far side
                 of the pursuer

Two arenas, same spawns and noise (common random numbers):
  card  x, y in [-5, 5] m, z in [0.5, 5.5] m (card)
  open  no x-y walls, same z limits (to separate wall cornering from pursuit)
Every cell (pursuer x evader x arena) runs N_EPISODES seeded episodes, vectorised.
Wall contact of the evader before capture is recorded for every episode.

Usage:
    conda activate drones
    python src/01_pursuit_evasion/3d/s020_3d_pursuit_evasion_game.py
"""
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from matplotlib.animation import FuncAnimation, PillowWriter

OUTPUT_DIR = os.path.normpath(os.path.join(
    os.path.dirname(__file__), '..', '..', '..',
    'outputs', '01_pursuit_evasion', '3d', 's020_3d_pursuit_evasion_game'))

# ── Parameters ────────────────────────────────────────────────────────────────
V_PURSUER   = 5.0
V_EVADER    = 3.5
CAPTURE_R   = 0.15
DT          = 0.02
T_MAX       = 30.0
MAX_STEPS   = int(round(T_MAX / DT))
XY_HALF     = 5.0            # card arena: x, y in [-5, 5]
Z_MIN, Z_MAX = 0.5, 5.5      # card z range
NOISE_SIGMA = 0.1
PN_GAIN     = 3.0
APF_D0      = 1.0            # m, wall influence distance
APF_ETA     = 1.0 / 48.0     # repulsion = attraction at 0.25 m from a face
T_LEAD      = 0.3
ALT_ADV_DZ  = 1.0
STRAIGHT_EL_MAX = np.radians(30.0)
SPIRAL_OMEGA = 0.8
SPIRAL_AZ    = 0.5
SPAWN_MIN_SEP = 5.0
N_EPISODES  = 500
SEED        = 2020
CONTACT_TOL = 1e-6
N_RECORD    = 2

PURSUERS = ['PP', 'PN', 'APF', 'Lead', 'AltAdv']
EVADERS  = ['straight', 'perpendicular', 'spiral', 'alt_escape']
ARENAS   = ['card', 'open']
P_COLORS = {'PP': '#e63946', 'PN': '#9d0208', 'APF': '#f77f00', 'Lead': '#ff70a6', 'AltAdv': '#6a040f'}


def unit(v):
    n = np.linalg.norm(v, axis=-1, keepdims=True)
    return v / np.maximum(n, 1e-9)


def clip_arena(x, arena):
    x = x.copy()
    if arena == 'card':
        x[:, :2] = np.clip(x[:, :2], -XY_HALF, XY_HALF)
    x[:, 2] = np.clip(x[:, 2], Z_MIN, Z_MAX)
    return x


# ── Pursuers ─────────────────────────────────────────────────────────────────

class Pursuer:
    def __init__(self, kind, arena):
        self.kind, self.arena = kind, arena

    def reset(self, Po, Eo):
        self.v = unit(Eo - Po) * V_PURSUER          # PN launch: along the initial LOS

    def act(self, Po, Vpo, Eo, Veo):
        k = self.kind
        if k == 'PP':
            return unit(Eo - Po) * V_PURSUER
        if k == 'Lead':
            return unit(Eo + Veo * T_LEAD - Po) * V_PURSUER
        if k == 'AltAdv':
            v = unit(Eo - Po) * V_PURSUER
            dalt = Po[:, 2] - Eo[:, 2]
            v[:, 2] += np.where(dalt < ALT_ADV_DZ, np.minimum(2.0, (ALT_ADV_DZ - dalt) * 1.5), 0.0)
            return unit(v) * V_PURSUER
        if k == 'APF':
            f = unit(Eo - Po)
            faces = [(2, Z_MIN, +1.0), (2, Z_MAX, -1.0)]
            if self.arena == 'card':
                faces += [(0, -XY_HALF, +1.0), (0, XY_HALF, -1.0), (1, -XY_HALF, +1.0), (1, XY_HALF, -1.0)]
            for ax, pos, normal in faces:
                d = np.maximum(np.abs(Po[:, ax] - pos), 0.01)
                rep = np.where(d < APF_D0, APF_ETA * (1.0 / d - 1.0 / APF_D0) / d ** 2, 0.0)
                f[:, ax] += normal * rep
            return unit(f) * V_PURSUER
        # PN: a = N v_c lambda_dot, velocity rescaled to v_P
        los = Eo - Po
        r = np.maximum(np.linalg.norm(los, axis=1, keepdims=True), 1e-9)
        lh = los / r
        vrel = Veo - self.v
        vr = np.sum(vrel * lh, axis=1, keepdims=True)
        ldot = (vrel - vr * lh) / r
        vc = -vr
        self.v = unit(self.v + PN_GAIN * vc * ldot * DT) * V_PURSUER
        return self.v


# ── Evaders ──────────────────────────────────────────────────────────────────

class Evader:
    def __init__(self, kind, rng, n):
        self.kind = kind
        az = rng.uniform(0, 2 * np.pi, n)
        el = rng.uniform(-STRAIGHT_EL_MAX, STRAIGHT_EL_MAX, n)
        self.heading = np.column_stack([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])
        self.side = np.where(rng.random(n) < 0.5, -1.0, 1.0)
        self.phase = rng.uniform(0, 2 * np.pi, n)
        self.phase_z = rng.uniform(0, 2 * np.pi, n)

    def act(self, Po, Eo, t):
        k, n = self.kind, len(Po)
        if k == 'straight':
            return self.heading * V_EVADER
        if k == 'perpendicular':
            los = Eo - Po
            perp = np.column_stack([-los[:, 1], los[:, 0], np.zeros(n)]) * self.side[:, None]
            small = np.linalg.norm(perp, axis=1) < 1e-6
            perp[small] = [1.0, 0.0, 0.0]
            return unit(perp) * V_EVADER
        if k == 'spiral':
            a = SPIRAL_OMEGA * t + self.phase
            v = np.column_stack([np.cos(a), np.sin(a),
                                 SPIRAL_AZ * np.sin(SPIRAL_OMEGA * t + self.phase_z)])
            return unit(v) * V_EVADER
        # alt_escape: away horizontally, vertical toward the limit on the far side of the pursuer
        away = Eo[:, :2] - Po[:, :2]
        hn = np.linalg.norm(away, axis=1, keepdims=True)
        h = np.where(hn > 1e-6, away / np.maximum(hn, 1e-9), np.array([[1.0, 0.0]]))
        vz = np.where(Po[:, 2] <= Eo[:, 2], 1.0, -1.0)
        return unit(np.column_stack([h, vz])) * V_EVADER


# ── Episodes (vectorised) ────────────────────────────────────────────────────

def spawns(n, seed):
    rng = np.random.default_rng(seed)
    P = np.zeros((n, 3)); E = np.zeros((n, 3))
    for k in range(n):
        while True:
            p = np.array([rng.uniform(-4.5, 4.5), rng.uniform(-4.5, 4.5), rng.uniform(1.0, 5.0)])
            e = np.array([rng.uniform(-4.5, 4.5), rng.uniform(-4.5, 4.5), rng.uniform(1.0, 5.0)])
            if np.linalg.norm(p - e) >= SPAWN_MIN_SEP:
                break
        P[k], E[k] = p, e
    return P, E


def simulate(pk, ek, arena, n=N_EPISODES, seed=SEED, n_record=0, sigma=NOISE_SIGMA):
    P, E = spawns(n, seed)
    rng = np.random.default_rng(seed + 1)          # evader parameters (same for every cell)
    ev = Evader(ek, rng, n)
    noise = np.random.default_rng(seed + 2)        # observation noise (same stream for every cell)
    pu = Pursuer(pk, arena)
    VP, VE = np.zeros((n, 3)), np.zeros((n, 3))
    active = np.ones(n, bool)
    t_cap = np.full(n, np.nan)
    wall_xy = np.zeros(n, bool); wall_z = np.zeros(n, bool)
    min_d = np.linalg.norm(P - E, axis=1)
    zp_hist, ze_hist = np.zeros(10), np.zeros(10)
    z_edges = np.linspace(Z_MIN, Z_MAX, 11)
    rec_p, rec_e = [P[:n_record].copy()], [E[:n_record].copy()]
    pu.reset(P + noise.normal(0, sigma, (n, 3)), E + noise.normal(0, sigma, (n, 3)))
    for step in range(MAX_STEPS):
        Po = P + noise.normal(0, sigma, (n, 3)); Eo = E + noise.normal(0, sigma, (n, 3))
        Vpo = VP + noise.normal(0, sigma, (n, 3)); Veo = VE + noise.normal(0, sigma, (n, 3))
        vp = pu.act(Po, Vpo, Eo, Veo)
        ve = ev.act(Po, Eo, step * DT)
        Pn = clip_arena(P + vp * DT, arena); En = clip_arena(E + ve * DT, arena)
        r0, d = E - P, (En - Pn) - (E - P)
        a = np.sum(d * d, axis=1)
        s = np.clip(-np.sum(r0 * d, axis=1) / np.maximum(a, 1e-15), 0.0, 1.0)
        dist = np.linalg.norm(r0 + s[:, None] * d, axis=1)
        act = active.copy()
        VP[act], VE[act] = ((Pn - P) / DT)[act], ((En - E) / DT)[act]   # actual (wall-clipped) velocity
        P[act], E[act] = Pn[act], En[act]
        min_d[act] = np.minimum(min_d[act], dist[act])
        if arena == 'card':
            wall_xy[act] |= np.any(np.abs(E[act, :2]) >= XY_HALF - CONTACT_TOL, axis=1)
        wall_z[act] |= (E[act, 2] <= Z_MIN + CONTACT_TOL) | (E[act, 2] >= Z_MAX - CONTACT_TOL)
        zp_hist += np.histogram(P[act, 2], z_edges)[0]; ze_hist += np.histogram(E[act, 2], z_edges)[0]
        cap = act & (dist < CAPTURE_R)
        t_cap[cap] = (step + 1) * DT
        active &= ~cap
        if n_record:
            rec_p.append(P[:n_record].copy()); rec_e.append(E[:n_record].copy())
        if not active.any():
            break
    return {'t_cap': t_cap, 'wall_xy': wall_xy, 'wall_z': wall_z, 'min_d': min_d,
            'zp_hist': zp_hist, 'ze_hist': ze_hist, 'rec_p': np.array(rec_p), 'rec_e': np.array(rec_e)}


def summarise(r):
    t = r['t_cap']
    cap = ~np.isnan(t)
    out = {'rate': cap.mean(), 'mean': np.nanmean(t) if cap.any() else np.nan,
           'median': np.median(np.where(cap, t, np.inf)), 'n': len(t)}
    for key, mask in [('wall_xy', r['wall_xy']), ('wall_z', r['wall_z'])]:
        out[key + '_frac'] = mask[cap].mean() if cap.any() else np.nan
        out[key + '_t_with'] = np.nanmean(t[cap & mask]) if (cap & mask).any() else np.nan
    anyw = r['wall_xy'] | r['wall_z']
    out['no_wall_t'] = np.nanmean(t[cap & ~anyw]) if (cap & ~anyw).any() else np.nan
    out['no_wall_n'] = int((cap & ~anyw).sum())
    out['esc_min_d'] = np.median(r['min_d'][~cap]) if (~cap).any() else np.nan
    return out


def run_simulation():
    print('S020 3D Pursuit-Evasion Game — hand-crafted benchmark')
    print('=' * 70)
    print(f'{N_EPISODES} seeded episodes per cell (seed {SEED}), same spawns, evader parameters and '
          f'noise in every cell; spawn separation >= {SPAWN_MIN_SEP} m')
    res, summ = {}, {}
    for arena in ARENAS:
        print(f'\nArena "{arena}"' + (' (x, y in [-5, 5], z in [0.5, 5.5])' if arena == 'card'
                                     else ' (no x-y walls, z in [0.5, 5.5])'))
        print('  pursuer | evader        | capture | mean t | median | captures with evader on x-y wall / '
              'z limit | mean t no contact (n) | escapes: median closest approach')
        for pk in PURSUERS:
            for ek in EVADERS:
                r = simulate(pk, ek, arena, n_record=N_RECORD)
                s = summarise(r)
                res[(arena, pk, ek)], summ[(arena, pk, ek)] = r, s
                med = 'inf' if np.isinf(s['median']) else f'{s["median"]:.2f}'
                print(f'  {pk:7s} | {ek:13s} | {100 * s["rate"]:5.1f}% | {s["mean"]:5.2f}s | {med:>6s} | '
                      f'{100 * s["wall_xy_frac"]:5.1f}% / {100 * s["wall_z_frac"]:5.1f}%'
                      f'{"":21s} | {s["no_wall_t"]:5.2f}s ({s["no_wall_n"]:3d})'
                      f'{"":11s} | {s["esc_min_d"]:.2f} m')
    for arena in ARENAS:
        print(f'\nMean capture time by pursuer, arena {arena} (all captured episodes, all evaders):')
        for pk in PURSUERS:
            ts = np.concatenate([res[(arena, pk, ek)]['t_cap'] for ek in EVADERS])
            print(f'  {pk:7s}: capture {100 * np.mean(~np.isnan(ts)):5.1f}%, mean {np.nanmean(ts):.2f} s')
    print('\nWithout observation noise (sigma = 0, same spawns), capture rate in the card arena / open arena:')
    print('  pursuer | ' + ' | '.join(f'{e:>13s}' for e in EVADERS))
    for pk in PURSUERS:
        row = []
        for ek in EVADERS:
            rc = summarise(simulate(pk, ek, 'card', sigma=0.0))['rate']
            ro = summarise(simulate(pk, ek, 'open', sigma=0.0))['rate']
            row.append(f'{100 * rc:5.1f} /{100 * ro:5.1f}')
        print(f'  {pk:7s} | ' + ' | '.join(f'{x:>13s}' for x in row))
    return {'res': res, 'summ': summ}


# ── Plots ─────────────────────────────────────────────────────────────────────

def plot_capture_matrix(summ, out_dir):
    fig, axes = plt.subplots(2, 2, figsize=(15, 10))
    for row, arena in enumerate(ARENAS):
        rate = np.array([[summ[(arena, p, e)]['rate'] * 100 for e in EVADERS] for p in PURSUERS])
        mt = np.array([[summ[(arena, p, e)]['mean'] for e in EVADERS] for p in PURSUERS])
        for col, (M, title, cmap, fmt) in enumerate([(rate, 'capture rate (%)', 'Greens', '{:.1f}%'),
                                                     (mt, 'mean capture time of captured episodes (s)',
                                                      'Reds', '{:.2f} s')]):
            ax = axes[row, col]
            im = ax.imshow(M, cmap=cmap, aspect='auto')
            for i in range(len(PURSUERS)):
                for j in range(len(EVADERS)):
                    ax.text(j, i, fmt.format(M[i, j]), ha='center', va='center', fontsize=9)
            ax.set_xticks(range(len(EVADERS))); ax.set_xticklabels(EVADERS)
            ax.set_yticks(range(len(PURSUERS))); ax.set_yticklabels(PURSUERS)
            ax.set_title(f'{arena} arena — {title}', fontsize=10)
            plt.colorbar(im, ax=ax, fraction=0.04)
    fig.suptitle(f'S020 3D — pursuer x evader benchmark, {N_EPISODES} episodes per cell', fontsize=11)
    plt.tight_layout()
    path = os.path.join(out_dir, 'capture_matrix.png')
    plt.savefig(path, dpi=120); plt.close(); print(f'Saved: {path}')


def plot_wall_contact(summ, out_dir):
    fig, axes = plt.subplots(1, 2, figsize=(17, 5.5))
    x = np.arange(len(EVADERS)); w = 0.16
    for k, pk in enumerate(PURSUERS):
        f = [100 * summ[('card', pk, e)]['wall_xy_frac'] for e in EVADERS]
        axes[0].bar(x + (k - 2) * w, f, w, color=P_COLORS[pk], label=pk, edgecolor='black')
        tc = [summ[('card', pk, e)]['mean'] for e in EVADERS]
        to = [summ[('open', pk, e)]['mean'] for e in EVADERS]
        axes[1].bar(x + (k - 2) * w, tc, w, color=P_COLORS[pk], edgecolor='black', label=pk)
        axes[1].scatter(x + (k - 2) * w, to, color='white', edgecolors='black', zorder=5, s=30,
                        label='open arena (same spawns)' if k == 0 else None)
    axes[0].set_xticks(x); axes[0].set_xticklabels(EVADERS)
    axes[0].set_ylabel('% of captures with the evader on an x-y wall before capture')
    axes[0].set_title('Card arena: wall contact before capture', fontsize=10)
    axes[0].legend(fontsize=8); axes[0].grid(True, alpha=0.3, axis='y')
    axes[1].set_xticks(x); axes[1].set_xticklabels(EVADERS)
    axes[1].set_ylabel('Mean capture time of captured episodes (s)')
    axes[1].set_title('Bars: card arena (colours as left); dots: open arena', fontsize=10)
    axes[1].legend(fontsize=8); axes[1].grid(True, alpha=0.3, axis='y')
    fig.suptitle('S020 3D — how much do the walls of the card arena decide the result?', fontsize=11)
    plt.tight_layout()
    path = os.path.join(out_dir, 'wall_contact.png')
    plt.savefig(path, dpi=120); plt.close(); print(f'Saved: {path}')


def plot_altitude_usage(res, out_dir):
    fig, axes = plt.subplots(1, len(EVADERS), figsize=(18, 4.5), sharey=True)
    edges = np.linspace(Z_MIN, Z_MAX, 11)
    centers = 0.5 * (edges[1:] + edges[:-1])
    for ax, ek in zip(axes, EVADERS):
        r = res[('card', 'PP', ek)]
        ax.barh(centers, r['ze_hist'] / r['ze_hist'].sum(), height=0.45, color='#0077b6', alpha=0.7,
                label='evader')
        ax.plot(r['zp_hist'] / r['zp_hist'].sum(), centers, 'o-', color='#e63946', label='pursuer (PP)')
        ax.set_title(ek, fontsize=10); ax.set_xlabel('Fraction of agent-steps'); ax.grid(True, alpha=0.3)
    axes[0].set_ylabel('z (m)'); axes[0].legend(fontsize=8)
    fig.suptitle(f'S020 3D — altitude usage over {N_EPISODES} episodes, card arena, PP pursuer', fontsize=11)
    plt.tight_layout()
    path = os.path.join(out_dir, 'altitude_usage.png')
    plt.savefig(path, dpi=120); plt.close(); print(f'Saved: {path}')


def plot_capture_cdf(res, out_dir):
    fig, axes = plt.subplots(1, len(EVADERS), figsize=(18, 4.8), sharey=True)
    tt = np.linspace(0, T_MAX, 301)
    for ax, ek in zip(axes, EVADERS):
        for pk in PURSUERS:
            for arena, ls in [('card', '-'), ('open', '--')]:
                t = res[(arena, pk, ek)]['t_cap']
                ax.plot(tt, [np.mean(t <= x) for x in tt], ls=ls, color=P_COLORS[pk],
                        label=f'{pk} ({arena})')
        ax.set_xscale('log'); ax.set_xlim(0.3, T_MAX); ax.set_title(ek, fontsize=10)
        ax.set_xlabel('Time (s)'); ax.grid(True, alpha=0.3, which='both')
    axes[0].set_ylabel('Fraction captured'); axes[-1].legend(fontsize=7, loc='lower right', ncol=2)
    fig.suptitle('S020 3D — capture-time CDF (solid = card arena, dashed = open arena)', fontsize=11)
    plt.tight_layout()
    path = os.path.join(out_dir, 'capture_cdf.png')
    plt.savefig(path, dpi=120); plt.close(); print(f'Saved: {path}')


TRAJ_EVADER = 'alt_escape'


def draw_box(ax, arena):
    if arena == 'card':
        ax.set_xlim(-5, 5); ax.set_ylim(-5, 5)
    ax.set_zlim(0, 6)


def plot_trajectories_3d(res, out_dir):
    fig = plt.figure(figsize=(22, 9))
    for row, arena in enumerate(ARENAS):
        for k, pk in enumerate(PURSUERS):
            r = res[(arena, pk, TRAJ_EVADER)]
            ax = fig.add_subplot(2, 5, row * 5 + k + 1, projection='3d')
            tc = r['t_cap'][0]
            n = int(round(tc / DT)) + 1 if not np.isnan(tc) else len(r['rec_p'])
            p, e = r['rec_p'][:n, 0], r['rec_e'][:n, 0]
            ax.plot(p[:, 0], p[:, 1], p[:, 2], color='red', lw=1.6, label=f'pursuer {pk}')
            ax.plot(e[:, 0], e[:, 1], e[:, 2], color='blue', lw=1.6, ls='--', label=f'evader {TRAJ_EVADER}')
            ax.scatter(*p[0], color='red', marker='D', s=30); ax.scatter(*e[0], color='blue', marker='^', s=30)
            if not np.isnan(tc):
                ax.scatter(*e[-1], color='gold', marker='*', s=120, edgecolors='black')
            draw_box(ax, arena)
            ax.set_title(f'{arena}: {pk} — ' + ('timeout' if np.isnan(tc) else f'capture {tc:.2f} s'),
                         fontsize=9)
            ax.tick_params(labelsize=6); ax.legend(fontsize=6, loc='upper left')
    fig.suptitle(f'S020 3D — episode 0, every pursuer vs the altitude-escape evader '
                 '(top: card arena, bottom: open arena; star = capture)', fontsize=11)
    plt.tight_layout(rect=(0, 0, 1, 0.96))
    path = os.path.join(out_dir, 'trajectories_3d.png')
    plt.savefig(path, dpi=110); plt.close(); print(f'Saved: {path}')


ANIM_PURSUERS = ['PP', 'PN', 'APF', 'AltAdv']
ANIM_EVADER = 'spiral'


def save_animation(res, out_dir):
    runs = []
    for pk in ANIM_PURSUERS:
        r = res[('card', pk, ANIM_EVADER)]
        tc = r['t_cap'][0]
        n = int(round(tc / DT)) + 1 if not np.isnan(tc) else len(r['rec_p'])
        runs.append((pk, r['rec_p'][:n, 0], r['rec_e'][:n, 0], tc))
    max_len = max(len(p) for _, p, _, _ in runs)
    step = 4
    frames = (max_len - 1) // step + 2
    fig = plt.figure(figsize=(12, 10))
    arts = []
    for k, (pk, p, e, tc) in enumerate(runs):
        ax = fig.add_subplot(2, 2, k + 1, projection='3d')
        draw_box(ax, 'card')
        pl, = ax.plot([], [], [], color='red', lw=1.4)
        el, = ax.plot([], [], [], color='blue', lw=1.4, ls='--')
        pd, = ax.plot([], [], [], 'o', color='red', mec='black', ms=6)
        ed, = ax.plot([], [], [], '^', color='blue', mec='black', ms=7)
        arts.append((ax, pk, p, e, tc, pl, el, pd, ed))

    def update(f):
        out = []
        for ax, pk, p, e, tc, pl, el, pd, ed in arts:
            n = min(f * step + 1, len(p))
            pl.set_data_3d(p[:n, 0], p[:n, 1], p[:n, 2]); el.set_data_3d(e[:n, 0], e[:n, 1], e[:n, 2])
            pd.set_data_3d([p[n - 1, 0]], [p[n - 1, 1]], [p[n - 1, 2]])
            ed.set_data_3d([e[n - 1, 0]], [e[n - 1, 1]], [e[n - 1, 2]])
            done = n == len(p) and not np.isnan(tc)
            ax.set_title(f'{pk} vs {ANIM_EVADER}, card arena   t = {(n - 1) * DT:.2f} s'
                         + ('  CAPTURED' if done else ''), fontsize=9)
            out += [pl, el, pd, ed]
        return out

    ani = FuncAnimation(fig, update, frames=frames, interval=60, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer=PillowWriter(fps=16), dpi=60)
    plt.close(); print(f'Saved: {path}  ({frames} frames)')


if __name__ == '__main__':
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    data = run_simulation()
    plot_capture_matrix(data['summ'], OUTPUT_DIR)
    plot_wall_contact(data['summ'], OUTPUT_DIR)
    plot_altitude_usage(data['res'], OUTPUT_DIR)
    plot_capture_cdf(data['res'], OUTPUT_DIR)
    plot_trajectories_3d(data['res'], OUTPUT_DIR)
    save_animation(data['res'], OUTPUT_DIR)
    print('Done.')
