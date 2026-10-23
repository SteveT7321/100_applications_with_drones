"""
S010 3D Upgrade — Asymmetric Speed Bounded Arena Trapping
=========================================================
Evader (v_E = 4.5 m/s) is faster than the pursuer (v_P = 3.0 m/s).
Arena: x, y ∈ [-5, 5] m, z ∈ [0.5, 5.5] m (card: 10 × 10 m floor, z range
0.5–5.5 m).  Faces, corners and wall points all use these real bounds.

Arena rule (identical for every player, ported from the fixed 2D S010): at
a face the velocity component pointing out of the arena is removed and the
rest is renormalised to the player's speed (it slides along the face).  A
command pointing straight into a face, edge or corner leaves nothing, and
the player stops there.

Pursuer strategies:
  1. direct     — pure 3D pursuit
  2. wall_herd  — α·(unit vector to evader) + (1-α)·(unit vector to the
                  evader's projection onto its nearest face); on a tie
                  (evader on an edge/corner diagonal) the projection onto all
                  tied faces (corner tie-break)
  3. corner     — aim at the arena corner nearest to the evader
  4. dive_boost — climb to Z_CRUISE while closing horizontally; when the
                  pursuer is > DIVE_DZ above the evader (and above 2 m) it
                  dives at γ toward the evader with gravity-assisted speed
                  min(√(v_P² + 2gηΔz), 4.2 m/s); recovers when it reaches the
                  evader's altitude or the floor

Evaders:
  - flee  (card) — away from the pursuer plus β·[0, 0, sign(z_E - z_P)]
  - aware        — card flee direction plus repulsion from faces closer than
                   D_SAFE; if fully blocked it turns sideways along a free
                   face (as the 2D wall-aware evader)

Runs: the card start, plus N_MC random starts (same starts for every
strategy and evader — common random numbers), plus a dive-angle sweep.

Usage:
    conda activate drones
    python src/01_pursuit_evasion/3d/s010_3d_asymmetric_speed.py
"""

import sys
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from matplotlib.animation import FuncAnimation, PillowWriter

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..'))

# ── Parameters ───────────────────────────────────────────────
XY_HALF    = 5.0
Z_MIN, Z_MAX = 0.5, 5.5
LO = np.array([-XY_HALF, -XY_HALF, Z_MIN])
HI = np.array([XY_HALF, XY_HALF, Z_MAX])
V_PURSUER  = 3.0
V_EVADER   = 4.5
V_DIVE_MAX = 4.2          # m/s — card's dive-boost cap
G          = 9.81
ETA        = 0.3          # motor efficiency factor in the dive boost
ALPHA      = 0.5
BETA_VERT  = 1.5
DIVE_ANGLE_DEG = 30.0
DIVE_DZ    = 1.0          # m — altitude advantage needed to start a dive
Z_CRUISE   = 5.0          # m — dive-boost climb target
K_CLIMB    = 1.0          # 1/s
V_Z_MAX    = 1.5          # m/s — climb-rate limit while cruising
D_SAFE     = 1.5          # m — wall-aware evader repulsion range
K_REPEL    = 2.0
CAPTURE_R  = 0.15
DT         = 1 / 48
MAX_TIME   = 60.0
N_MC       = 50
MC_SEED    = 2026
MIN_SEP    = 3.0          # m — minimum start separation in the MC
DIVE_ANGLES = [10.0, 20.0, 30.0, 40.0, 50.0]
SPEED_RATIO = V_PURSUER / V_EVADER

INIT_PURSUER = np.array([0.0, 0.0, 2.0])
INIT_EVADER  = np.array([4.0, 0.0, 2.0])

CORNERS_3D = np.array([[x, y, z] for x in (-XY_HALF, XY_HALF) for y in (-XY_HALF, XY_HALF)
                       for z in (Z_MIN, Z_MAX)], dtype=float)
# outward normals of faces x=-5, x=+5, y=-5, y=+5, z=0.5, z=5.5
FACE_NORMALS = np.array([[-1, 0, 0], [1, 0, 0], [0, -1, 0], [0, 1, 0], [0, 0, -1], [0, 0, 1]],
                        dtype=float)

STRATEGIES = ['direct', 'wall_herd', 'corner', 'dive_boost']
EVADERS = ['flee', 'aware']
STRATEGY_COLORS = {'direct': 'darkorange', 'wall_herd': 'steelblue',
                   'corner': 'mediumseagreen', 'dive_boost': 'firebrick'}
STRATEGY_LABELS = {'direct': 'Direct 3D', 'wall_herd': 'Wall herd 3D',
                   'corner': 'Corner target 3D', 'dive_boost': 'Dive boost 3D'}
EV_LABELS = {'flee': 'flee (card)', 'aware': 'wall-aware'}

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), '..', '..', '..',
                          'outputs', '01_pursuit_evasion', '3d', 's010_3d_asymmetric_speed')


# ── Helpers ──────────────────────────────────────────────────

def unit(v):
    n = np.linalg.norm(v)
    return v / n if n > 1e-12 else np.zeros_like(v)


def remove_outward(pos, d):
    d = d.copy()
    for k in range(3):
        if (pos[k] >= HI[k] - 1e-9 and d[k] > 0) or (pos[k] <= LO[k] + 1e-9 and d[k] < 0):
            d[k] = 0.0
    return d


def wall_velocity(pos, d, speed):
    """Arena rule for every player: drop outward components at a face and
    renormalise the rest to full speed (zero if nothing is left)."""
    return speed * unit(remove_outward(pos, d))


def face_distances(e):
    return np.array([e[0] - LO[0], HI[0] - e[0], e[1] - LO[1], HI[1] - e[1],
                     e[2] - LO[2], HI[2] - e[2]])


def wall_point(e):
    """Projection of the evader onto its nearest face: p_E + δ·n̂_out.  On a
    tie (equal distances) the evader is moved onto all tied faces."""
    d = face_distances(e)
    tied = np.isclose(d, d.min(), atol=1e-9)
    return e + (d[tied][:, None] * FACE_NORMALS[tied]).sum(axis=0)


def apollonius_sphere(p, e, v_p=V_PURSUER, v_e=V_EVADER):
    """Equal-time sphere |X-P|/v_P = |X-E|/v_E: radius r·d/(1-r²), centre on
    the line E→P at distance d/(1-r²) from E (beyond P)."""
    r = v_p / v_e
    d = np.linalg.norm(e - p)
    if d < 1e-9:
        return e.copy(), 0.0
    return e + (p - e) / d * d / (1 - r ** 2), r * d / (1 - r ** 2)


# ── Pursuer strategies ────────────────────────────────────────

def dir_direct(p, e, st):
    return unit(e - p), V_PURSUER


def dir_wall_herd(p, e, st):
    return unit(ALPHA * unit(e - p) + (1 - ALPHA) * unit(wall_point(e) - p)), V_PURSUER


def dir_corner(p, e, st):
    c = CORNERS_3D[np.argmin(np.linalg.norm(CORNERS_3D - e, axis=1))]
    return unit(c - p), V_PURSUER


def dir_dive(p, e, st):
    gamma = np.radians(st['dive_angle'])
    dz = p[2] - e[2]
    if not st['diving'] and dz > DIVE_DZ and p[2] > 2.0:
        st['diving'] = True
        st['dives'] += 1
    if st['diving'] and (dz <= 0.2 or p[2] <= Z_MIN + 0.05):
        st['diving'] = False
    if st['diving']:
        st['dive_steps'] += 1
        h = unit((e - p)[:2])
        d = np.array([h[0] * np.cos(gamma), h[1] * np.cos(gamma), -np.sin(gamma)])
        v = min(np.sqrt(V_PURSUER ** 2 + 2 * G * ETA * max(dz, 0.0)), V_DIVE_MAX)
        st['max_speed'] = max(st['max_speed'], v)
        return d, v
    r = e - p
    if np.linalg.norm(r) < 1.0:                        # close range: direct
        return unit(r), V_PURSUER
    vz = np.clip(K_CLIMB * (Z_CRUISE - p[2]), -V_Z_MAX, V_Z_MAX)
    h = unit(r[:2]) * np.sqrt(V_PURSUER ** 2 - vz ** 2)
    return unit(np.array([h[0], h[1], vz])), V_PURSUER


PURSUERS = {'direct': dir_direct, 'wall_herd': dir_wall_herd, 'corner': dir_corner,
            'dive_boost': dir_dive}


# ── Evaders ──────────────────────────────────────────────────

def wall_repulsion(e):
    w = np.clip((D_SAFE - face_distances(e)) / D_SAFE, 0.0, 1.0)
    return -(w[:, None] * FACE_NORMALS).sum(axis=0)


def evader_velocity(p, e, kind):
    away = e - p
    sz = np.sign(e[2] - p[2])
    away = away + BETA_VERT * np.array([0.0, 0.0, sz if sz != 0 else 1.0])
    d = unit(away)
    if kind == 'aware':
        d2 = unit(d + K_REPEL * wall_repulsion(e))
        d = d2 if np.linalg.norm(d2) > 1e-12 else d
        if np.linalg.norm(remove_outward(e, d)) < 1e-9:
            for axis in (np.array([0, 0, 1.0]), np.array([1.0, 0, 0]), np.array([0, 1.0, 0])):
                c = remove_outward(e, np.cross(axis, d))
                if np.linalg.norm(c) > 1e-9:
                    d = c
                    break
    return wall_velocity(e, d, V_EVADER)


# ── Simulation ────────────────────────────────────────────────

def run_simulation(strategy, evader='flee', p0=INIT_PURSUER, e0=INIT_EVADER,
                   dive_angle=DIVE_ANGLE_DEG):
    p, e = np.array(p0, float), np.array(e0, float)
    st = dict(diving=False, dives=0, dive_steps=0, dive_angle=dive_angle, max_speed=0.0)
    P, E, A, dive_flag = [p.copy()], [e.copy()], [apollonius_sphere(p, e)[1]], [0]
    captured, cap_time = False, None
    for k in range(int(round(MAX_TIME / DT))):
        d, sp = PURSUERS[strategy](p, e, st)
        vp = wall_velocity(p, d, sp)
        ve = evader_velocity(p, e, evader)
        p = np.clip(p + vp * DT, LO, HI)
        e = np.clip(e + ve * DT, LO, HI)
        P.append(p.copy()); E.append(e.copy()); A.append(apollonius_sphere(p, e)[1])
        dive_flag.append(1 if st['diving'] else 0)
        if np.linalg.norm(p - e) < CAPTURE_R:
            captured, cap_time = True, (k + 1) * DT
            break
    P, E = np.array(P), np.array(E)
    return dict(p=P, e=E, t=np.arange(len(P)) * DT, d=np.linalg.norm(P - E, axis=1),
                apo=np.array(A), dive=np.array(dive_flag), captured=captured,
                cap_time=cap_time, dives=st['dives'], dive_steps=st['dive_steps'],
                max_dive_speed=st['max_speed'], strategy=strategy, evader=evader)


WHERE = {0: 'interior', 1: 'face', 2: 'edge', 3: 'corner'}


def end_state(r):
    """(number of faces the evader touches at the end, its mean speed over the
    last 0.5 s)."""
    e = r['e']
    on = int(np.sum((np.abs(e[-1] - LO) < 1e-6) | (np.abs(e[-1] - HI) < 1e-6)))
    k = int(round(0.5 / DT))
    sp = np.linalg.norm(np.diff(e[-k - 1:], axis=0), axis=1) / DT
    net = np.linalg.norm(e[-1] - e[-k - 1]) / (k * DT) if len(e) > k else np.nan
    return on, sp.mean(), net


def describe(r):
    e = r['e']
    on = int(np.sum((np.abs(e[-1] - LO) < 1e-6) | (np.abs(e[-1] - HI) < 1e-6)))
    sp = np.linalg.norm(np.diff(e, axis=0), axis=1) / DT
    k = int(round(0.5 / DT))
    stopped = 0
    for s_ in sp[::-1]:
        if s_ > 1.0:
            break
        stopped += 1
    path = np.linalg.norm(np.diff(r['p'], axis=0), axis=1).sum()
    if r['captured']:
        txt = (f'captured @ {r["cap_time"]:.3f} s, evader on {WHERE[on]} '
               f'{np.round(e[-1], 2).tolist()}, evader speed last 0.5 s {sp[-k:].mean():.2f} m/s '
               f'(net {np.linalg.norm(e[-1] - e[-k - 1]) / (k * DT):.2f} m/s), '
               f'stopped (<1 m/s) for the last {stopped * DT:.2f} s')
    else:
        last = r['d'][r['t'] >= r['t'][-1] - 10]
        txt = (f'timeout | last 10 s distance {last.min():.2f}–{last.max():.2f} m, evader '
               f'speed last 10 s {sp[-int(10 / DT):].mean():.2f} m/s')
    return (f'{txt} | pursuer path {path:.2f} m | dives {r["dives"]} '
            f'({r["dive_steps"] * DT:.2f} s, max speed {r["max_dive_speed"]:.2f} m/s) | '
            f'z_P {r["p"][:, 2].min():.2f}–{r["p"][:, 2].max():.2f}, '
            f'z_E {e[:, 2].min():.2f}–{e[:, 2].max():.2f}')


def random_starts():
    rng = np.random.default_rng(MC_SEED)
    out = []
    while len(out) < N_MC:
        p = rng.uniform(LO + 0.5, HI - 0.5); e = rng.uniform(LO + 0.5, HI - 0.5)
        if np.linalg.norm(p - e) >= MIN_SEP:
            out.append((p, e))
    return out


def run_monte_carlo(starts, dive_angle=DIVE_ANGLE_DEG, strategies=STRATEGIES):
    res = {}
    for s in strategies:
        for ev in EVADERS:
            caps, dives, trajs, where, spd, net = [], [], [], [], [], []
            for p0, e0 in starts:
                r = run_simulation(s, ev, p0, e0, dive_angle)
                caps.append(r['cap_time'] if r['captured'] else np.nan)
                dives.append(r['dives'])
                trajs.append(r['p'])
                w_, v_, n_ = end_state(r)
                where.append(w_); spd.append(v_); net.append(n_)
            res[(s, ev)] = dict(cap=np.array(caps), dives=np.array(dives), p=trajs,
                                where=np.array(where), speed=np.array(spd), net=np.array(net))
    return res


# ── Plots ────────────────────────────────────────────────────

def draw_arena(ax):
    for a in (LO[0], HI[0]):
        for b in (LO[1], HI[1]):
            ax.plot([a, a], [b, b], [LO[2], HI[2]], color='grey', lw=0.5, alpha=0.5)
    for a in (LO[0], HI[0]):
        for c in (LO[2], HI[2]):
            ax.plot([a, a], [LO[1], HI[1]], [c, c], color='grey', lw=0.5, alpha=0.5)
    for b in (LO[1], HI[1]):
        for c in (LO[2], HI[2]):
            ax.plot([LO[0], HI[0]], [b, b], [c, c], color='grey', lw=0.5, alpha=0.5)


def plot_trajectories_3d(main, out_dir):
    fig = plt.figure(figsize=(22, 10))
    for row, ev in enumerate(EVADERS):
        for col, s in enumerate(STRATEGIES):
            r = main[(s, ev)]
            ax = fig.add_subplot(2, 4, row * 4 + col + 1, projection='3d')
            draw_arena(ax)
            ax.plot(r['p'][:, 0], r['p'][:, 1], r['p'][:, 2], color='red', lw=1.4, label='Pursuer')
            ax.plot(r['e'][:, 0], r['e'][:, 1], r['e'][:, 2], color='royalblue', lw=1.2,
                    alpha=0.8, label='Evader')
            if r['dives']:
                pd = r['p'].copy(); pd[r['dive'] == 0] = np.nan
                ax.plot(pd[:, 0], pd[:, 1], pd[:, 2], color='black', lw=2.5, label='Diving')
            ax.scatter(*r['p'][0], color='red', s=40); ax.scatter(*r['e'][0], color='blue', s=40)
            if r['captured']:
                ax.scatter(*r['p'][-1], color='black', s=80, marker='X')
                stt = f'captured {r["cap_time"]:.2f} s'
            else:
                stt = f'no capture in {MAX_TIME:.0f} s'
            ax.set_xlim(LO[0], HI[0]); ax.set_ylim(LO[1], HI[1]); ax.set_zlim(0, 6)
            ax.set_title(f'{STRATEGY_LABELS[s]} vs {EV_LABELS[ev]}\n{stt}', fontsize=9)
            ax.legend(fontsize=6, loc='upper left')
    fig.suptitle(f'S010 3D — card start P{INIT_PURSUER.tolist()} E{INIT_EVADER.tolist()} '
                 f'(v_P={V_PURSUER}, v_E={V_EVADER} m/s)', fontsize=11)
    plt.tight_layout()
    path = os.path.join(out_dir, 'trajectories_3d.png')
    plt.savefig(path, dpi=120, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_altitude_time(main, out_dir):
    fig, axes = plt.subplots(2, 4, figsize=(20, 7), sharey=True)
    for row, ev in enumerate(EVADERS):
        for col, s in enumerate(STRATEGIES):
            ax = axes[row, col]
            r = main[(s, ev)]
            t_end = min(r['t'][-1], 20.0)
            m = r['t'] <= t_end
            ax.plot(r['t'][m], r['p'][m, 2], color='red', label='z_P')
            ax.plot(r['t'][m], r['e'][m, 2], color='royalblue', label='z_E')
            for k in np.where(np.diff(r['dive'][m].astype(int)) == 1)[0]:
                ax.axvline(r['t'][k + 1], color='black', ls=':', lw=1)
            ax.axhline(Z_MIN, color='grey', lw=0.8); ax.axhline(Z_MAX, color='grey', lw=0.8)
            ax.set_title(f'{STRATEGY_LABELS[s]} vs {EV_LABELS[ev]} (dives: {r["dives"]})',
                         fontsize=9)
            ax.set_xlabel('Time (s)'); ax.grid(True, alpha=0.3)
            if col == 0:
                ax.set_ylabel('Altitude (m)')
            ax.legend(fontsize=7)
    fig.suptitle('S010 3D — altitude vs time, card start (first 20 s; dotted = dive start)',
                 fontsize=11)
    plt.tight_layout()
    path = os.path.join(out_dir, 'altitude_time.png')
    plt.savefig(path, dpi=130)
    plt.close()
    print(f'Saved: {path}')


def plot_apollonius_radius(main, out_dir):
    fig, axes = plt.subplots(1, 2, figsize=(14, 4.8), sharey=True)
    for ax, ev in zip(axes, EVADERS):
        for s in STRATEGIES:
            r = main[(s, ev)]
            m = r['t'] <= 20.0
            ax.plot(r['t'][m], r['apo'][m], color=STRATEGY_COLORS[s], label=STRATEGY_LABELS[s])
        ax.set_xlabel('Time (s)'); ax.set_title(f'Evader: {EV_LABELS[ev]}', fontsize=10)
        ax.grid(True, alpha=0.3); ax.legend(fontsize=8)
    axes[0].set_ylabel('Apollonius sphere radius r·d/(1−r²) (m)')
    fig.suptitle('S010 3D — Apollonius (equal-time) sphere radius, card start (first 20 s)',
                 fontsize=11)
    plt.tight_layout()
    path = os.path.join(out_dir, 'apollonius_radius.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def plot_capture_comparison(mc, out_dir):
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(14, 5))
    xs = np.arange(len(STRATEGIES)); w = 0.38
    for k, ev in enumerate(EVADERS):
        rate = [np.mean(~np.isnan(mc[(s, ev)]['cap'])) * 100 for s in STRATEGIES]
        med = [np.nanmedian(mc[(s, ev)]['cap']) if np.any(~np.isnan(mc[(s, ev)]['cap']))
               else np.nan for s in STRATEGIES]
        hatch = '' if ev == 'flee' else '//'
        b1 = a1.bar(xs + (k - 0.5) * w, rate, w, color=[STRATEGY_COLORS[s] for s in STRATEGIES],
                    hatch=hatch, edgecolor='black', label=f'evader {EV_LABELS[ev]}')
        a2.bar(xs + (k - 0.5) * w, med, w, color=[STRATEGY_COLORS[s] for s in STRATEGIES],
               hatch=hatch, edgecolor='black', label=f'evader {EV_LABELS[ev]}')
        for x, v in zip(xs + (k - 0.5) * w, rate):
            a1.text(x, v + 1, f'{v:.0f}%', ha='center', fontsize=8)
    for a in (a1, a2):
        a.set_xticks(xs); a.set_xticklabels([STRATEGY_LABELS[s] for s in STRATEGIES])
        a.grid(axis='y', alpha=0.3); a.legend(fontsize=8, loc='lower left')
    a1.set_ylabel(f'Capture rate (%) over {N_MC} random starts'); a1.set_ylim(0, 110)
    a2.set_ylabel('Median capture time of captured runs (s)')
    a1.set_title('Capture rate (plain = flee, hatched = wall-aware)', fontsize=10)
    a2.set_title('Median capture time', fontsize=10)
    fig.suptitle(f'S010 3D — {N_MC} random starts (seed {MC_SEED}), same starts for every '
                 'strategy', fontsize=11)
    plt.tight_layout()
    path = os.path.join(out_dir, 'capture_comparison.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def plot_dive_angle_sweep(sweep, mc, out_dir):
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(14, 5))
    for ev, mk in zip(EVADERS, ['o-', 's--']):
        caps = [sweep[g][('dive_boost', ev)]['cap'] for g in DIVE_ANGLES]
        rate = [np.mean(~np.isnan(c)) * 100 for c in caps]
        med = [np.nanmedian(c) if np.any(~np.isnan(c)) else np.nan for c in caps]
        a1.plot(DIVE_ANGLES, rate, mk, color='firebrick', label=f'Dive boost vs {EV_LABELS[ev]}')
        a2.plot(DIVE_ANGLES, med, mk, color='firebrick', label=f'Dive boost vs {EV_LABELS[ev]}')
        a2.axhline(np.nanmedian(mc[('direct', ev)]['cap']), color='darkorange',
                   ls='-' if ev == 'flee' else '--', label=f'Direct 3D vs {EV_LABELS[ev]}')
    a1.set_ylim(-5, 105); a1.set_ylabel(f'Capture rate (%), {N_MC} starts')
    a2.set_ylabel('Median capture time (s)')
    for a in (a1, a2):
        a.set_xlabel('Dive angle γ (deg)'); a.grid(True, alpha=0.3); a.legend(fontsize=8)
    a1.set_title('Capture rate', fontsize=10); a2.set_title('Median capture time', fontsize=10)
    fig.suptitle('S010 3D — dive-boost sensitivity to the dive angle (same starts)', fontsize=11)
    plt.tight_layout()
    path = os.path.join(out_dir, 'dive_angle_sweep.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def save_animation(cases, out_dir):
    T_ANIM = 15.0
    n_max = int(round(T_ANIM / DT)) + 1
    step = 6
    n_frames = (n_max - 1) // step + 2
    fig = plt.figure(figsize=(15, 6))
    arts = []
    for k, (lab, r) in enumerate(cases):
        ax = fig.add_subplot(1, len(cases), k + 1, projection='3d')
        draw_arena(ax)
        ax.set_xlim(LO[0], HI[0]); ax.set_ylim(LO[1], HI[1]); ax.set_zlim(0, 6)
        lp, = ax.plot([], [], [], 'r-', lw=1.3)
        le, = ax.plot([], [], [], color='royalblue', lw=1.2)
        dp, = ax.plot([], [], [], 'r^', ms=8)
        de, = ax.plot([], [], [], 'bs', ms=7)
        ti = ax.set_title(lab, fontsize=9)
        arts.append((lab, r, lp, le, dp, de, ti))

    def update(i):
        out = []
        for lab, r, lp, le, dp, de, ti in arts:
            n = len(r['p']); si = min(i * step, n - 1, n_max - 1)
            lo_ = max(0, si - int(3 / DT))
            p, e = r['p'], r['e']
            lp.set_data(p[lo_:si + 1, 0], p[lo_:si + 1, 1]); lp.set_3d_properties(p[lo_:si + 1, 2])
            le.set_data(e[lo_:si + 1, 0], e[lo_:si + 1, 1]); le.set_3d_properties(e[lo_:si + 1, 2])
            dp.set_data([p[si, 0]], [p[si, 1]]); dp.set_3d_properties([p[si, 2]])
            de.set_data([e[si, 0]], [e[si, 1]]); de.set_3d_properties([e[si, 2]])
            if r['captured'] and si == n - 1:
                ti.set_text(f'{lab}\ncaptured {r["cap_time"]:.2f} s')
            else:
                dv = ' DIVE' if r['dive'][si] else ''
                ti.set_text(f'{lab}\nt={r["t"][si]:.2f} s d={r["d"][si]:.2f} m{dv}')
            out += [lp, le, dp, de, ti]
        return out

    fig.suptitle(f'S010 3D — first {T_ANIM:.0f} s (3 s trails; red ▲ pursuer, '
                 'blue ■ evader)', fontsize=10)
    ani = FuncAnimation(fig, update, frames=n_frames, interval=60, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer=PillowWriter(fps=16), dpi=75)
    plt.close()
    print(f'Saved: {path}  ({n_frames} frames, every {step} steps)')


# ── Main ─────────────────────────────────────────────────────

if __name__ == '__main__':
    out_dir = os.path.normpath(OUTPUT_DIR)
    os.makedirs(out_dir, exist_ok=True)

    pe = np.array([4.0, 0.0, 2.0])
    print(f'Wall point check: evader {pe.tolist()} -> {wall_point(pe).tolist()} (nearest face x = +5)')
    pe = np.array([0.0, 0.0, 4.8])
    print(f'Wall point check: evader {pe.tolist()} -> {wall_point(pe).tolist()} (ceiling z = {Z_MAX})')
    pe = np.array([4.0, 4.0, 4.5])
    print(f'Wall point check (tie): evader {pe.tolist()} -> {wall_point(pe).tolist()}')
    c, R = apollonius_sphere(INIT_PURSUER, INIT_EVADER)
    worst = 0.0
    for v in np.vstack([np.eye(3), -np.eye(3)]):
        x = c + R * v
        worst = max(worst, abs(np.linalg.norm(x - INIT_PURSUER) / V_PURSUER
                               - np.linalg.norm(x - INIT_EVADER) / V_EVADER))
    print(f'Apollonius sphere at t=0: centre {np.round(c, 3).tolist()}, R = {R:.3f} m, '
          f'max |t_P - t_E| on 6 points {worst:.1e} s, contains pursuer '
          f'{np.linalg.norm(INIT_PURSUER - c) < R}, contains evader {np.linalg.norm(INIT_EVADER - c) < R}')

    print('\nCard start:')
    main = {}
    for ev in EVADERS:
        for s in STRATEGIES:
            r = run_simulation(s, ev)
            main[(s, ev)] = r
            print(f'  [{s:<10} vs {ev:<5}] {describe(r)}')
    for ev in EVADERS:
        same = [(a, b) for i, a in enumerate(STRATEGIES) for b in STRATEGIES[i + 1:]
                if len(main[(a, ev)]['p']) == len(main[(b, ev)]['p'])
                and np.allclose(main[(a, ev)]['p'], main[(b, ev)]['p'])]
        print(f'  evader {ev}: strategy pairs with identical pursuer paths: {same}')

    starts = random_starts()
    mc = run_monte_carlo(starts)
    print(f'\nMonte Carlo: {N_MC} random starts (seed {MC_SEED}, separation >= {MIN_SEP} m), '
          'same starts for every strategy:')
    for ev in EVADERS:
        for s in STRATEGIES:
            c_ = mc[(s, ev)]['cap']
            nc = int(np.sum(~np.isnan(c_)))
            med = f'median {np.nanmedian(c_):.3f} s, max {np.nanmax(c_):.3f} s' if nc else '—'
            dv = mc[(s, ev)]['dives']
            capm = ~np.isnan(c_)
            wh = mc[(s, ev)]['where'][capm]
            locs = ', '.join(f'{WHERE[k]} {int(np.sum(wh == k))}' for k in range(4)
                             if np.sum(wh == k))
            stp = int(np.sum(mc[(s, ev)]['speed'][capm] < 1.0))
            nets = mc[(s, ev)]['net'][capm]
            nettxt = f'{np.median(nets):.2f}' if nc else '—'
            print(f'  [{s:<10} vs {ev:<5}] captured {nc:2d}/{N_MC} | {med} | runs with a dive '
                  f'{int(np.sum(dv > 0))}/{N_MC}, dives total {int(dv.sum())} | captures: '
                  f'evader stopped (<1 m/s over last 0.5 s) in {stp}/{nc}, median net speed '
                  f'(displacement over last 0.5 s / 0.5 s) {nettxt} m/s; location: {locs}')
        for i, a in enumerate(STRATEGIES):
            for b in STRATEGIES[i + 1:]:
                same = sum(len(x) == len(y) and np.allclose(x, y)
                           for x, y in zip(mc[(a, ev)]['p'], mc[(b, ev)]['p']))
                if same:
                    print(f'    evader {ev}: {a} and {b} give identical pursuer paths in '
                          f'{same}/{N_MC} starts')

    sweep = {}
    print('\nDive-angle sweep (dive boost, same starts):')
    for g in DIVE_ANGLES:
        sweep[g] = run_monte_carlo(starts, g, ['dive_boost'])
        row = []
        for ev in EVADERS:
            c_ = sweep[g][('dive_boost', ev)]['cap']
            nc = int(np.sum(~np.isnan(c_)))
            row.append(f'{ev}: {nc}/{N_MC}' + (f' (median {np.nanmedian(c_):.3f} s)' if nc else ''))
        print(f'  γ = {g:4.0f}°: ' + ' | '.join(row))

    plot_trajectories_3d(main, out_dir)
    plot_altitude_time(main, out_dir)
    plot_apollonius_radius(main, out_dir)
    plot_capture_comparison(mc, out_dir)
    plot_dive_angle_sweep(sweep, mc, out_dir)
    k_dive = int(np.argmax(mc[('dive_boost', 'aware')]['dives'] > 0))
    ex = run_simulation('dive_boost', 'aware', *starts[k_dive])
    first = np.where(np.diff(ex['dive']) == 1)[0]
    print(f'\nDive example: MC start {k_dive} P{np.round(starts[k_dive][0], 2).tolist()} '
          f'E{np.round(starts[k_dive][1], 2).tolist()}: {describe(ex)}; first dive at '
          f't = {ex["t"][first[0] + 1]:.3f} s')
    save_animation([('Direct vs wall-aware (card start)', main[('direct', 'aware')]),
                    ('Corner target vs wall-aware (card start)', main[('corner', 'aware')]),
                    (f'Dive boost vs wall-aware (MC start {k_dive})', ex)], out_dir)
    print('Done.')
