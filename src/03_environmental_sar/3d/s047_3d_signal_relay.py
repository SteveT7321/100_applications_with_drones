"""
S047 3D Signal Relay Enhancement
================================
Three relay drones form a 4-hop chain between a ground base station (BS) and a
rescue team across a corridor with three Gaussian hills ("ridges").  Compared
with the 2D-folder S047 (shadow rectangles tested at the hop midpoint), every
relay now has an altitude z_i, hop lengths are 3D, and a hop is shadowed only
if its straight line of sight dips below the terrain at any of 50 samples.

The objective (card) is the bottleneck SNR minus an altitude penalty
lambda * sum max(0, z_i - z_clear_i)^2, where z_clear_i is the lowest altitude
at which both adjacent hops of relay i are terrain-free.

Strategies (card):
  1. 2D uniform baseline   - equally spaced (x, y), fixed hover height 20 m AGL
  2. Altitude-only         - (x, y) fixed at the uniform spacing, z_i optimised
  3. Full 3D               - joint (x_i, y_i, z_i) gradient ascent
Both optimisers use the 2D script's method (soft-min objective, central
differences eps = 1 m, step eta x gradient) with the card's 3000 iterations,
and are run from several initial hover heights.

Usage:
    conda activate drones
    python src/03_environmental_sar/3d/s047_3d_signal_relay.py
"""

import os
import importlib.util
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from scipy.optimize import minimize

SCRIPT_DIR   = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, '..', '..', '..'))
OUTPUT_DIR   = os.path.join(PROJECT_ROOT, 'outputs', '03_environmental_sar', '3d',
                            's047_3d_signal_relay')

_spec = importlib.util.spec_from_file_location(
    's047_2d', os.path.join(SCRIPT_DIR, '..', 's047_signal_relay.py'))
s2d = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(s2d)

# -- Parameters (card) -----------------------------------------------------------
N_RELAYS     = 3
N_LOSS       = s2d.N_LOSS        # 2.8
P_TX         = s2d.P_TX          # 30 dBm
N_FLOOR      = s2d.N_FLOOR       # -95 dBm
SNR_MIN      = s2d.SNR_MIN       # 10 dB
L0           = s2d.L0            # FSPL at 1 m, 2.4 GHz
D_MAX        = 700.0             # m  (3D, card)
DELTA_SHADOW = s2d.DELTA_SHADOW  # 15 dB
ALPHA_SOFT   = s2d.ALPHA_SOFT    # 0.5
ETA          = s2d.ETA           # 5 (x gradient in dB/m)
N_ITER       = 3000              # card (2D: 2000)
LAMBDA_ALT   = 0.05              # dB / m^2
N_LOS        = 50                # LOS samples per hop
AGL_MIN, AGL_MAX = 5.0, 120.0    # m  card's z bounds, read as height above terrain
AGL_BASE     = 20.0              # m  baseline hover height (card "20 m"), read as AGL
AREA         = 1000.0
FD_EPS       = 1.0               # m  central-difference step (2D value)
RECORD_EVERY = 25                # animation sampling of the 3D ascent
START_AGLS   = (5.0, 20.0, 60.0, 120.0)   # added: initial hover heights for the optimisers
SHOW_START   = 20.0              # the run shown in the convergence plots / animation
NM_MAXFEV    = 6000              # added: Nelder-Mead reference, max objective evaluations

BS   = np.array([100.0, 500.0, 1.5])
TEAM = np.array([900.0, 500.0, 1.5])

# Ridges: centres taken from the 2D S047 shadow-zone centres (card gives none),
# heights and widths from the card
RIDGES = [  # (cx, cy, H, sigma_x, sigma_y)
    (350.0, 480.0, 40.0, 80.0, 80.0),
    (500.0, 520.0, 55.0, 80.0, 80.0),
    (650.0, 460.0, 45.0, 80.0, 80.0),
]
T_LOS = np.linspace(0.0, 1.0, N_LOS)


# -- Terrain / link model -----------------------------------------------------------

def terrain(x, y):
    x = np.asarray(x, float); y = np.asarray(y, float)
    h = np.zeros(np.broadcast(x, y).shape)
    for cx, cy, H, sx, sy in RIDGES:
        h = h + H * np.exp(-(x - cx) ** 2 / (2 * sx ** 2) - (y - cy) ** 2 / (2 * sy ** 2))
    return h


def los_margin(a, b):
    """z_los(t) - h_terrain(x(t), y(t)) at the 50 LOS samples."""
    p = (1 - T_LOS)[:, None] * a + T_LOS[:, None] * b
    return p[:, 2] - terrain(p[:, 0], p[:, 1])


def link(a, b):
    """(SNR dB, shadowed?, 3D length) of one hop."""
    d = max(float(np.linalg.norm(b - a)), 1e-3)
    sh = bool(los_margin(a, b).min() < 0.0)
    snr = P_TX - (L0 + 10 * N_LOSS * np.log10(d) + DELTA_SHADOW * sh) - N_FLOOR
    return snr, sh, d


def chain_eval(relays):
    nodes = np.vstack([BS, relays, TEAM])
    out = [link(nodes[k], nodes[k + 1]) for k in range(N_RELAYS + 1)]
    return (np.array([o[0] for o in out]), np.array([o[1] for o in out]),
            np.array([o[2] for o in out]))


def z_required(fixed, xy):
    """
    Lowest altitude of a relay at horizontal position xy such that the LOS from
    the fixed node to it clears the terrain at every sample t > 0:
    z >= max_t (h(t) - (1 - t) z_fixed) / t   (the horizontal track does not depend on z).
    """
    t = T_LOS[1:]
    px = (1 - t) * fixed[0] + t * xy[0]
    py = (1 - t) * fixed[1] + t * xy[1]
    return float(np.max((terrain(px, py) - (1 - t) * fixed[2]) / t))


def z_clear(relays):
    """Per-relay clearance altitude (absolute), neighbours at their current positions."""
    nodes = np.vstack([BS, relays, TEAM])
    zc = np.zeros(N_RELAYS)
    for i in range(1, N_RELAYS + 1):
        xy = nodes[i, :2]
        zc[i - 1] = max(z_required(nodes[i - 1], xy), z_required(nodes[i + 1], xy),
                        float(terrain(*xy)) + AGL_MIN)
    return zc


def objective(relays, soft=True):
    snrs = chain_eval(relays)[0]
    core = s2d.soft_min(snrs, ALPHA_SOFT) if soft else float(np.min(snrs))
    excess = np.maximum(0.0, relays[:, 2] - z_clear(relays))
    return core - LAMBDA_ALT * float(np.sum(excess ** 2))


def project(relays):
    """Area box, AGL band [5, 120] m and the 3D range limit (forward / backward pass)."""
    r = relays.copy()
    r[:, :2] = np.clip(r[:, :2], 0.0, AREA)
    h = terrain(r[:, 0], r[:, 1])
    r[:, 2] = np.clip(r[:, 2], h + AGL_MIN, h + AGL_MAX)
    nodes = np.vstack([BS, r, TEAM])
    for i in range(1, N_RELAYS + 1):
        v = nodes[i] - nodes[i - 1]; d = np.linalg.norm(v)
        if d > D_MAX:
            nodes[i] = nodes[i - 1] + v / d * D_MAX
    for i in range(N_RELAYS, 0, -1):
        v = nodes[i] - nodes[i + 1]; d = np.linalg.norm(v)
        if d > D_MAX:
            nodes[i] = nodes[i + 1] + v / d * D_MAX
    return nodes[1:-1]


def check_invariants(relays):
    h = terrain(relays[:, 0], relays[:, 1])
    assert np.all(relays[:, 2] >= h + AGL_MIN - 1e-6), 'relay below the minimum terrain clearance'
    assert np.all(relays[:, 2] <= h + AGL_MAX + 1e-6), 'relay above the maximum height'
    assert np.all(chain_eval(relays)[2] <= D_MAX + 1e-6), 'hop longer than d_max'


# -- Strategies -------------------------------------------------------------------------

def uniform_xy():
    return np.array([BS[:2] + (i / (N_RELAYS + 1)) * (TEAM[:2] - BS[:2])
                     for i in range(1, N_RELAYS + 1)])


def at_agl(xy, agl):
    return np.c_[xy, terrain(xy[:, 0], xy[:, 1]) + agl]


def gradient_ascent(start, dims):
    """Gradient ascent on the soft-min objective over the coordinates in dims."""
    relays = project(start)
    hist_bn, hist_obj, hist_agl, traj = [], [], [], [relays.copy()]
    for it in range(N_ITER):
        snrs = chain_eval(relays)[0]
        hist_bn.append(float(snrs.min())); hist_obj.append(objective(relays, soft=False))
        hist_agl.append(relays[:, 2] - terrain(relays[:, 0], relays[:, 1]))
        grad = np.zeros_like(relays)
        for i in range(N_RELAYS):
            for j in dims:
                rf = relays.copy(); rf[i, j] += FD_EPS
                rb = relays.copy(); rb[i, j] -= FD_EPS
                grad[i, j] = (objective(rf) - objective(rb)) / (2 * FD_EPS)
        relays = project(relays + ETA * grad)
        check_invariants(relays)
        if (it + 1) % RECORD_EVERY == 0:
            traj.append(relays.copy())
    snrs = chain_eval(relays)[0]
    hist_bn.append(float(snrs.min())); hist_obj.append(objective(relays, soft=False))
    hist_agl.append(relays[:, 2] - terrain(relays[:, 0], relays[:, 1]))
    return dict(relays=relays, bn=np.array(hist_bn), obj=np.array(hist_obj),
                agl=np.array(hist_agl), traj=np.array(traj))


def nelder_mead_full(start):
    """Added reference: derivative-free Nelder-Mead on the card objective J (hard min)."""
    hist = []

    def f(v):
        val = -objective(project(v.reshape(N_RELAYS, 3)), soft=False)
        hist.append(-val)
        return val

    r = minimize(f, start.ravel(), method='Nelder-Mead',
                 options=dict(maxfev=NM_MAXFEV, xatol=1e-3, fatol=1e-6))
    relays = project(r.x.reshape(N_RELAYS, 3))
    check_invariants(relays)
    return relays, int(r.nfev), np.maximum.accumulate(np.array(hist))


def summarise(relays):
    snrs, sh, d = chain_eval(relays)
    agl = relays[:, 2] - terrain(relays[:, 0], relays[:, 1])
    return dict(relays=relays, snrs=snrs, shadow=sh, hops=d, agl=agl,
                bottleneck=float(snrs.min()), J=objective(relays, soft=False),
                zclear=z_clear(relays))


def run_simulation():
    xy0 = uniform_xy()
    res = {'baseline': summarise(at_agl(xy0, AGL_BASE))}
    runs = {'alt': {}, 'full': {}}
    for agl in START_AGLS:
        start = at_agl(xy0, agl)
        runs['alt'][agl] = gradient_ascent(start, dims=(2,))
        runs['full'][agl] = gradient_ascent(start, dims=(0, 1, 2))
    res['alt'] = summarise(runs['alt'][SHOW_START]['relays'])
    res['full'] = summarise(runs['full'][SHOW_START]['relays'])
    nm_relays, nm_nfev, nm_hist = nelder_mead_full(at_agl(xy0, AGL_BASE))
    res['nm'] = summarise(nm_relays)
    res['nm']['nfev'] = nm_nfev; res['nm']['hist'] = nm_hist
    # fixed-height sweep of the uniform layout (what a single hover height gives)
    aglsweep = np.arange(AGL_MIN, AGL_MAX + 0.01, 0.5)
    fixed = [summarise(at_agl(xy0, a)) for a in aglsweep]
    # direct BS -> team link for reference
    direct = link(BS, TEAM)
    return dict(res=res, runs=runs, aglsweep=aglsweep, fixed=fixed, direct=direct)


# -- Plots ------------------------------------------------------------------------------

COLORS = {'baseline': '#2196F3', 'alt': '#FF9800', 'full': '#4CAF50', 'nm': '#9C27B0'}
LABELS = {'baseline': '2D uniform, 20 m AGL', 'alt': 'Altitude-only (GA)', 'full': 'Full 3D (GA)',
          'nm': 'Full 3D (Nelder-Mead, added)'}
KEYS = ('baseline', 'alt', 'full', 'nm')


def _save(fig, name):
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    path = os.path.join(OUTPUT_DIR, name)
    fig.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def _terrain_grid(n=90):
    gx, gy = np.meshgrid(np.linspace(0, AREA, n), np.linspace(150, 850, n))
    return gx, gy, terrain(gx, gy)


def plot_terrain_3d(data):
    gx, gy, gh = _terrain_grid()
    fig = plt.figure(figsize=(13, 10))
    for col, key in enumerate(KEYS):
        ax = fig.add_subplot(2, 2, col + 1, projection='3d')
        ax.plot_surface(gx, gy, gh, cmap='Greys', alpha=0.45, lw=0, rstride=2, cstride=2)
        r = data['res'][key]
        nodes = np.vstack([BS, r['relays'], TEAM])
        for k in range(N_RELAYS + 1):
            c = 'red' if r['shadow'][k] else 'limegreen'
            ax.plot(*nodes[k:k + 2].T, color=c, lw=2.2)
        ax.scatter(*r['relays'].T, s=60, color=COLORS[key], edgecolors='k', depthshade=False)
        ax.scatter(*BS, s=70, color='black', marker='s'); ax.scatter(*TEAM, s=80, color='green', marker='^')
        ax.set_xlim(0, AREA); ax.set_ylim(150, 850); ax.set_zlim(0, 120)
        ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
        ax.view_init(elev=22, azim=-70)
        ax.set_title(f'{LABELS[key]}\nbottleneck {r["bottleneck"]:.2f} dB, '
                     f'AGL {", ".join(f"{a:.1f}" for a in r["agl"])} m', fontsize=9)
    fig.suptitle('S047 3D - Terrain, relays and LOS links (green clear, red shadowed)',
                 fontsize=12, fontweight='bold')
    _save(fig, 'terrain_relay_3d.png')


def plot_topdown(data):
    gx, gy, gh = _terrain_grid(200)
    fig, ax = plt.subplots(figsize=(10, 6.2))
    cf = ax.contourf(gx, gy, gh, levels=np.arange(0, gh.max() + 5, 5), cmap='Greys', alpha=0.8)
    fig.colorbar(cf, ax=ax, label='terrain height (m)')
    for key, mk in (('baseline', 'o'), ('alt', 's'), ('full', 'D'), ('nm', 'P')):
        r = data['res'][key]
        nodes = np.vstack([BS, r['relays'], TEAM])
        ax.plot(nodes[:, 0], nodes[:, 1], '-', color=COLORS[key], lw=1.5, alpha=0.8)
        ax.scatter(r['relays'][:, 0], r['relays'][:, 1], color=COLORS[key], marker=mk, s=70,
                   edgecolors='k', zorder=5, label=LABELS[key])
    ax.plot(*BS[:2], 's', color='black', ms=10, label='Base station')
    ax.plot(*TEAM[:2], '^', color='green', ms=10, label='Rescue team')
    ax.set_xlim(0, AREA); ax.set_ylim(150, 850); ax.set_aspect('equal')
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.legend(fontsize=8, loc='upper right')
    ax.set_title('S047 3D - Top-down view on the terrain height map', fontsize=11, fontweight='bold')
    _save(fig, 'terrain_topdown.png')


def plot_altitude_convergence(data):
    fig, axes = plt.subplots(1, 2, figsize=(13, 4.5), sharey=True)
    for ax, key in zip(axes, ('alt', 'full')):
        for s in START_AGLS:
            agl = data['runs'][key][s]['agl']
            for i in range(N_RELAYS):
                ax.plot(agl[:, i], color=f'C{i}', lw=1.4 if s == SHOW_START else 0.8,
                        ls='-' if s == SHOW_START else ':',
                        label=f'R{i+1}' if s == SHOW_START else None)
        ax.set_xscale('symlog', linthresh=10); ax.set_xlim(0, N_ITER + 1)
        ax.set_xlabel('iteration (symlog)'); ax.set_ylabel('height above terrain (m)')
        ax.set_title(f'{LABELS[key]}: solid = start {SHOW_START:.0f} m AGL, dotted = starts '
                     f'{", ".join(f"{s:.0f}" for s in START_AGLS if s != SHOW_START)} m', fontsize=9)
        ax.grid(True, alpha=0.3); ax.legend(fontsize=8)
    fig.suptitle('S047 3D - Relay hover height during gradient ascent', fontsize=11,
                 fontweight='bold')
    fig.tight_layout()
    _save(fig, 'altitude_convergence.png')


def plot_snr_chain_bars(data):
    fig, ax = plt.subplots(figsize=(11, 5))
    w = 0.2
    for j, key in enumerate(KEYS):
        r = data['res'][key]
        x = np.arange(4) + (j - 1.5) * w
        bars = ax.bar(x, r['snrs'], w, color=COLORS[key], label=LABELS[key], edgecolor='k', lw=0.5)
        kb = int(np.argmin(r['snrs']))
        bars[kb].set_edgecolor('red'); bars[kb].set_linewidth(2.2)
        for xi, v, sh in zip(x, r['snrs'], r['shadow']):
            ax.text(xi, v + 0.2, f'{v:.2f}' + (' S' if sh else ''), ha='center', fontsize=7)
    ax.axhline(SNR_MIN, color='k', ls='--', lw=1, label='SNR_min 10 dB')
    ax.set_xticks(range(4)); ax.set_xticklabels(['BS-R1', 'R1-R2', 'R2-R3', 'R3-Team'])
    ax.set_ylabel('SNR (dB)'); ax.set_ylim(0, 26)
    ax.set_title('S047 3D - Per-link SNR (bottleneck outlined red, S = shadowed)', fontsize=11,
                 fontweight='bold')
    ax.legend(fontsize=8, loc='lower right'); ax.grid(True, axis='y', alpha=0.3)
    _save(fig, 'snr_chain_bars.png')


def plot_bottleneck_vs_height(data):
    fig, ax = plt.subplots(figsize=(10, 4.8))
    bn = [f['bottleneck'] for f in data['fixed']]
    ax.plot(data['aglsweep'], bn, 'k-', lw=1.5, label='uniform (x, y), all relays at one fixed AGL')
    for key, mk in (('baseline', 'o'), ('alt', 's'), ('full', 'D'), ('nm', 'P')):
        r = data['res'][key]
        ax.scatter(np.mean(r['agl']), r['bottleneck'], color=COLORS[key], marker=mk, s=80,
                   edgecolors='k', zorder=5, label=f'{LABELS[key]}: {r["bottleneck"]:.2f} dB '
                                                    f'(mean AGL {np.mean(r["agl"]):.1f} m)')
    ax.axhline(SNR_MIN, color='gray', ls='--', lw=1)
    ax.set_xlabel('hover height above terrain (m)'); ax.set_ylabel('bottleneck SNR (dB)')
    ax.set_title('S047 3D - Bottleneck SNR vs hover height (uniform spacing)', fontsize=11,
                 fontweight='bold')
    ax.grid(True, alpha=0.3); ax.legend(fontsize=8)
    _save(fig, 'bottleneck_vs_height.png')


def plot_convergence(data):
    fig, ax = plt.subplots(figsize=(10, 4.6))
    for key in ('alt', 'full'):
        for s in START_AGLS:
            bn = data['runs'][key][s]['bn']
            ax.plot(bn, color=COLORS[key], lw=1.6 if s == SHOW_START else 0.9,
                    ls='-' if s == SHOW_START else ':',
                    label=f'{LABELS[key]} (start {s:.0f} m AGL)')
    ax.axhline(14.21, color='purple', ls='--', lw=1,
               label='2D S047 gradient ascent final, 14.21 dB (rectangle shadow model)')
    ax.axhline(data['res']['baseline']['bottleneck'], color=COLORS['baseline'], ls='--', lw=1,
               label=f'2D uniform 20 m AGL baseline {data["res"]["baseline"]["bottleneck"]:.2f} dB')
    ax.set_xscale('symlog', linthresh=10); ax.set_xlim(0, N_ITER + 1)
    ax.set_xlabel('iteration (symlog)'); ax.set_ylabel('bottleneck SNR (dB)')
    ax.set_title('S047 3D - Bottleneck SNR during gradient ascent', fontsize=11, fontweight='bold')
    ax.grid(True, alpha=0.3); ax.legend(fontsize=7, ncol=2)
    _save(fig, 'convergence.png')


def plot_los_margin(data):
    fig, ax = plt.subplots(figsize=(10, 4.4))
    r = data['res']['full']
    nodes = np.vstack([BS, r['relays'], TEAM])
    names = ['BS-R1', 'R1-R2', 'R2-R3', 'R3-Team']
    for k in range(4):
        ax.plot(T_LOS, los_margin(nodes[k], nodes[k + 1]), '-o', ms=2.5, label=names[k])
    ax.axhline(0, color='red', lw=1)
    ax.set_xlabel('position along link t'); ax.set_ylabel('LOS height above terrain (m)')
    ax.set_title('S047 3D - LOS clearance margin along each link (full 3D solution)',
                 fontsize=11, fontweight='bold')
    ax.grid(True, alpha=0.3); ax.legend(fontsize=8)
    _save(fig, 'los_margin.png')


def save_animation(data):
    run = data['runs']['full'][SHOW_START]
    traj = run['traj']
    n = len(traj)
    gx, gy, gh = _terrain_grid(60)
    fig = plt.figure(figsize=(9, 6))
    ax = fig.add_subplot(111, projection='3d')
    ax.plot_surface(gx, gy, gh, cmap='Greys', alpha=0.4, lw=0)
    ax.scatter(*BS, s=70, color='black', marker='s'); ax.scatter(*TEAM, s=80, color='green', marker='^')
    lines = [ax.plot([], [], [], lw=2)[0] for _ in range(4)]
    pts = ax.plot([], [], [], 'o', color=COLORS['full'], mec='k', ms=7)[0]
    ax.set_xlim(0, AREA); ax.set_ylim(150, 850); ax.set_zlim(0, 120)
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
    ax.view_init(elev=20, azim=-70)
    ttl = ax.set_title('')

    def update(f):
        rel = traj[f]
        nodes = np.vstack([BS, rel, TEAM])
        snrs, sh, _ = chain_eval(rel)
        for k in range(4):
            lines[k].set_data(nodes[k:k + 2, 0], nodes[k:k + 2, 1])
            lines[k].set_3d_properties(nodes[k:k + 2, 2])
            lines[k].set_color('red' if sh[k] else 'limegreen')
        pts.set_data(rel[:, 0], rel[:, 1]); pts.set_3d_properties(rel[:, 2])
        ttl.set_text(f'S047 3D - full 3D ascent, iteration {min(f * RECORD_EVERY, N_ITER)}: '
                     f'bottleneck {snrs.min():.2f} dB')
        return lines + [pts, ttl]

    ani = animation.FuncAnimation(fig, update, frames=n, blit=False)
    path = os.path.join(OUTPUT_DIR, 'animation.gif')
    ani.save(path, writer='pillow', fps=10, dpi=80)
    plt.close(fig)
    print(f'Saved: {path} ({n} frames)')


# -- Report -----------------------------------------------------------------------------

def report(data):
    print(f'\nDirect BS -> team link: {data["direct"][2]:.1f} m, shadowed {data["direct"][1]}, '
          f'SNR {data["direct"][0]:.2f} dB')
    xy0 = uniform_xy()
    print('Terrain under the uniform relay positions: ' +
          ', '.join(f'{terrain(*p):.1f} m' for p in xy0))
    print('\n-- Strategies (shown run: optimisers started at 20 m AGL) --')
    print(f'  Nelder-Mead: {data["res"]["nm"]["nfev"]} objective evaluations')
    st = at_agl(xy0, AGL_BASE)
    g_soft, g_pen = np.zeros_like(st), np.zeros_like(st)
    for i in range(N_RELAYS):
        for j in range(3):
            rf = st.copy(); rf[i, j] += FD_EPS
            rb = st.copy(); rb[i, j] -= FD_EPS
            g_soft[i, j] = (s2d.soft_min(chain_eval(rf)[0], ALPHA_SOFT)
                            - s2d.soft_min(chain_eval(rb)[0], ALPHA_SOFT)) / (2 * FD_EPS)
            g_pen[i, j] = (objective(rf) - objective(rb)) / (2 * FD_EPS) - g_soft[i, j]
    for i in range(N_RELAYS):
        print(f'  gradient at the 20 m AGL start, R{i+1}: soft-min part '
              f'({g_soft[i, 0]:+.3f}, {g_soft[i, 1]:+.3f}, {g_soft[i, 2]:+.3f}) dB/m, '
              f'altitude-penalty part ({g_pen[i, 0]:+.3f}, {g_pen[i, 1]:+.3f}, {g_pen[i, 2]:+.3f}) dB/m')
    for key in KEYS:
        r = data['res'][key]
        pos = '; '.join(f'({p[0]:.1f}, {p[1]:.1f}, {p[2]:.1f})' for p in r['relays'])
        print(f'  {LABELS[key]:22s}: relays {pos}')
        print(f'      AGL {", ".join(f"{a:.2f}" for a in r["agl"])} m | z - z_clear '
              f'{", ".join(f"{a:+.2f}" for a in r["relays"][:, 2] - r["zclear"])} m')
        print(f'      SNR {", ".join(f"{s:.2f}" for s in r["snrs"])} dB | shadowed {r["shadow"].astype(int).tolist()}'
              f' | hops {", ".join(f"{h:.1f}" for h in r["hops"])} m | bottleneck {r["bottleneck"]:.3f} dB'
              f' | J {r["J"]:.3f} dB')
    nodes = np.vstack([BS, data['res']['full']['relays'], TEAM])
    print('  Full 3D (GA) minimum LOS clearance per link: ' + ', '.join(
        f'{los_margin(nodes[k], nodes[k + 1]).min():.2f} m' for k in range(N_RELAYS + 1)))
    print('\n-- All starts (final values after 3000 iterations) --')
    print('  strategy      start AGL | bottleneck | J (card objective) | shadowed links | final AGL (m) | best-iterate bottleneck')
    for key in ('alt', 'full'):
        for s in START_AGLS:
            run = data['runs'][key][s]
            r = summarise(run['relays'])
            print(f'  {LABELS[key]:13s} {s:6.0f}    | {r["bottleneck"]:10.3f} | {r["J"]:18.3f} | '
                  f'{int(r["shadow"].sum()):14d} | {", ".join(f"{a:.1f}" for a in r["agl"]):13s} | '
                  f'{run["bn"].max():.3f} (iter {int(np.argmax(run["bn"]))})')
    bn = np.array([f['bottleneck'] for f in data['fixed']])
    sh = np.array([f['shadow'].any() for f in data['fixed']])
    clear_from = data['aglsweep'][np.argmax(~sh)]
    print(f'\n-- Fixed-height sweep (uniform x, y) --')
    print(f'  shadowed below {clear_from:.1f} m AGL; best fixed height {data["aglsweep"][np.argmax(bn)]:.1f} m AGL '
          f'-> {bn.max():.3f} dB; at 5 m {bn[0]:.3f} dB; at 20 m '
          f'{bn[np.argmin(np.abs(data["aglsweep"] - 20))]:.3f} dB; at 120 m {bn[-1]:.3f} dB')
    print(f'  shadow-free upper bound for 4 equal 200 m flat hops: '
          f'{P_TX - (L0 + 10 * N_LOSS * np.log10(200.0)) - N_FLOOR:.3f} dB')


if __name__ == '__main__':
    print('Running S047 3D Signal Relay Enhancement...')
    data = run_simulation()
    report(data)
    plot_terrain_3d(data)
    plot_topdown(data)
    plot_altitude_convergence(data)
    plot_snr_chain_bars(data)
    plot_bottleneck_vs_height(data)
    plot_convergence(data)
    plot_los_margin(data)
    save_animation(data)
    print('\nDone. Outputs in:', OUTPUT_DIR)
