"""
S067 3D Upgrade — Spray Overlap Optimization on Undulating Terrain
==================================================================
The 2D S067 base sprays a flat 50 x 50 m field in parallel strips at a fixed 2 m height
(Gaussian cross-track dose, sigma_s = 0.5 m, GPS strip offsets N(0, 0.1^2)) and shows that
CV(d) has no interior minimum; the useful design is the widest spacing with CV <= 10 %
(1.365 m noise-free, 0.662 m at the P95 under GPS error).

The 3D variant puts the same mission over the card's bi-sinusoidal hill
h(x, y) = 1.5 sin(2 pi x / 30) cos(2 pi y / 30) and makes the drone altitude a controlled
state along every strip:
  * fixed inertial altitude (2.0 m = same mean height as the 2D plan; 3.5 m = never below
    2 m above ground),
  * terrain following with a first-order lag  z' = omega (h(x_k, y) + 2 - z),
  * terrain following + the card's adaptive strip spacing d_k = d * AGL(x_k, y_mid) / 2.
The spray is a cone of droplet directions (tan of the angle ~ N(0, (tan(theta_c)/2)^2)); every
droplet travels to the actual ground below it, so the footprint width follows the local
drop height and the slope.  Dose is mass per unit terrain-surface area, and the CV is
area-weighted over the field interior (x in [1, 49] m).

Common random numbers: the same GPS draws (default_rng(42)) for every configuration.

Usage:
    conda activate drones
    python src/04_industrial_agriculture/3d/s067_3d_spray_overlap.py
"""

import os
import math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, '..', '..', '..'))
OUTPUT_DIR = os.path.join(PROJECT_ROOT, 'outputs', '04_industrial_agriculture', '3d',
                          's067_3d_spray_overlap')

# ── Parameters ────────────────────────────────────────────────────────────────
FIELD_W = FIELD_L = 50.0
A_HILL = 1.5                       # card
LAMBDA = 30.0                      # card (both directions)
Z_AGL = 2.0                        # card
TAN_C = 0.5                        # card: tan(26.6 deg)
S_U = TAN_C / 2.0                  # std of tan(droplet angle); sigma_s = AGL * S_U = 0.5 m at 2 m
U_CUT = 6 * S_U                    # spray truncated at 6 sigma of the droplet angle (as the base)
GPS = 0.1                          # card
V = 3.0                            # card
N_MC = 100                         # card
N_CURVE = 30                       # added: trials per point of the CV curves / heat map
SEED = 42                          # base
X_RES, Y_RES = 0.05, 0.5           # cross-track 0.05 m (base); along-track 0.5 m (added; checked)
MARGIN = 1.0                       # base interior margin
AGL_FLOOR = 0.5                    # added: asserted minimum height above ground
Z_FIX_LO, Z_FIX_HI = 2.0, 3.5      # added fixed-altitude plans
OMEGA_NOM = 2.0                    # added nominal terrain-following bandwidth (card range 0.5-5)
OMEGAS = [0.5, 1.0, 2.0, 3.0, 5.0, np.inf]
D_DESIGN = 1.365                   # base: widest flat-field spacing with noise-free CV <= 10 %
D_POINTS = [0.662, 1.365, 1.8]     # base GPS-robust design, base noise-free design, card nominal
D_CURVE = np.round(np.arange(0.4, 2.6001, 0.2), 4)
CV_TARGET = 0.10
REDUND_K, UNDER_K = 1.5, 0.5
Y_MID = 25.0                       # card: representative along-track position for adaptive d
SUB = 10                           # altitude-integration sub-steps per along-track cell
ANIM_D = D_DESIGN

X = np.arange(X_RES / 2, FIELD_W, X_RES)
Y = np.arange(Y_RES / 2, FIELD_L, Y_RES)
INT = (X >= MARGIN) & (X <= FIELD_W - MARGIN)
N_STRIPS_MAX = 200

CONFIGS = {  # key: (label, terrain?, altitude mode, value, adaptive?)
    'flat': ('Flat field, z = 2.0 m (2D base)', False, 'fixed', 2.0, False),
    'fixed2': ('Fixed altitude z = 2.0 m', True, 'fixed', Z_FIX_LO, False),
    'fixed35': ('Fixed altitude z = 3.5 m', True, 'fixed', Z_FIX_HI, False),
    'tf': (f'Terrain following ω = {OMEGA_NOM:g} s⁻¹', True, 'tf', OMEGA_NOM, False),
    'tf_adapt': (f'TF ω = {OMEGA_NOM:g} + adaptive spacing (card)', True, 'tf', OMEGA_NOM, True),
    'fixed2_adapt': ('Fixed z = 2.0 m + adaptive spacing', True, 'fixed', Z_FIX_LO, True),
}
CURVE_KEYS = ['flat', 'fixed2', 'fixed35', 'tf', 'tf_adapt']
COLORS = {'flat': 'k', 'fixed2': '#D62728', 'fixed35': '#FF7F0E', 'tf': '#1F77B4',
          'tf_adapt': '#2CA02C', 'fixed2_adapt': '#9467BD'}


# ── Terrain ───────────────────────────────────────────────────────────────────

def h_fun(x, y, terrain=True):
    if not terrain:
        return np.zeros(np.broadcast(x, y).shape)
    k = 2 * np.pi / LAMBDA
    return A_HILL * np.sin(k * x) * np.cos(k * y)


def hx_fun(x, y, terrain=True):
    if not terrain:
        return np.zeros(np.broadcast(x, y).shape)
    k = 2 * np.pi / LAMBDA
    return A_HILL * k * np.cos(k * x) * np.cos(k * y)


def hy_fun(x, y, terrain=True):
    if not terrain:
        return np.zeros(np.broadcast(x, y).shape)
    k = 2 * np.pi / LAMBDA
    return -A_HILL * k * np.sin(k * x) * np.sin(k * y)


def cos_phi(terrain=True):
    XX, YY = np.meshgrid(X, Y)
    return 1.0 / np.sqrt(1 + hx_fun(XX, YY, terrain) ** 2 + hy_fun(XX, YY, terrain) ** 2)


# ── Flight ────────────────────────────────────────────────────────────────────

def altitude_profile(xk, k, terrain, mode, val):
    """Drone altitude at every along-track cell of strip k (odd strips fly -y)."""
    if mode == 'tf35':        # added ablation: same lag, command h + 3.5 m (linear: offset)
        return altitude_profile(xk, k, terrain, 'tf', val) + (Z_FIX_HI - Z_AGL)
    if mode == 'fixed':
        return np.full(Y.size, float(val))
    zc_cells = h_fun(xk, Y, terrain) + Z_AGL
    if not np.isfinite(val):
        return zc_cells
    ys = np.arange(0.0, FIELD_L + 1e-9, Y_RES / SUB)
    if k % 2 == 1:
        ys = ys[::-1]
    zc = h_fun(xk, ys, terrain) + Z_AGL
    a = 1 - math.exp(-val * (Y_RES / SUB) / V)
    z = np.empty_like(ys)
    z[0] = zc[0]                                  # settled during the turn
    for i in range(1, ys.size):
        z[i] = z[i - 1] + (zc[i - 1] - z[i - 1]) * a
    if k % 2 == 1:
        ys, z = ys[::-1], z[::-1]
    return np.interp(Y, ys, z)


def plan(d, cfg):
    """Strip positions and altitude profiles for spacing d."""
    _, terrain, mode, val, adapt = CONFIGS[cfg] if isinstance(cfg, str) else cfg
    xs, zs = [], []
    if not adapt:                                  # base: x = k d, ceil(W/d) + 1 strips
        n = int(np.ceil(FIELD_W / d - 1e-9)) + 1
        for k in range(n):
            xs.append(k * d)
            zs.append(altitude_profile(k * d, k, terrain, mode, val))
    else:                                          # card: d_k = d * AGL(x_k, y_mid) / 2
        xk, k = 0.0, 0
        while True:
            zp = altitude_profile(xk, k, terrain, mode, val)
            xs.append(xk)
            zs.append(zp)
            if xk >= FIELD_W - 1e-9:
                break
            agl_mid = float(np.interp(Y_MID, Y, zp) - h_fun(xk, Y_MID, terrain))
            xk = min(xk + d * agl_mid / Z_AGL, FIELD_W)
            k += 1
    assert len(xs) <= N_STRIPS_MAX
    return np.array(xs), np.array(zs), terrain


def dose_maps(xs, zs, terrain, eps, model='jacobian'):
    """Surface dose (M, Ny, Nx) for GPS offsets eps (M, >= n strips)."""
    M = eps.shape[0]
    dose = np.zeros((M, Y.size, X.size))
    cphi = cos_phi(terrain)
    for k, (xk, zp) in enumerate(zip(xs, zs)):
        agl_n = zp - h_fun(xk, Y, terrain)
        assert agl_n.min() >= AGL_FLOOR, f'drone below the {AGL_FLOOR} m floor: {agl_n.min():.3f}'
        half = U_CUT * (agl_n.max() + 2 * A_HILL) + 0.5     # covers |u| <= U_CUT everywhere
        i0, i1 = np.searchsorted(X, [xk - half, xk + half])
        xw = X[i0:i1]
        H = (zp[:, None] - h_fun(xw[None, :], Y[:, None], terrain)).astype(np.float32)  # (Ny, Nw)
        hx = hx_fun(xw[None, :], Y[:, None], terrain).astype(np.float32)
        e = eps[:, k]
        assert np.all(np.abs(e) < 0.5), 'GPS offset outside the kernel margin'
        dx = (xw[None, None, :] - xk - e[:, None, None]).astype(np.float32)  # (M, 1, Nw)
        if model == 'jacobian':
            H = np.where(H > 1e-6, H, np.float32(1e-6))  # ground above the drone: outside the cone
            u = dx / H[None]
            jac = (H[None] + dx * hx[None]) / H[None] ** 2
            inside = np.abs(u) <= U_CUT                  # droplets beyond 6 sigma ignored
            assert jac[inside].min() > 0, 'spray cone shadowed by the slope inside 6 sigma'
            q = np.where(inside, np.exp(np.float32(-0.5 / S_U ** 2) * u * u)
                         * np.float32(1 / (math.sqrt(2 * math.pi) * S_U)) * jac, np.float32(0))
            q = q * cphi[None, :, i0:i1]                                    # per surface area
        else:                                    # card: Q0 / cos(phi) * exp(-dx^2 / 2 sigma_nadir^2)
            sig = agl_n * S_U
            q = np.exp(-0.5 * (dx / sig[None, :, None]) ** 2) / cphi[None, :, i0:i1]
        dose[:, :, i0:i1] += q / V
    return dose


def stats(dose, terrain):
    w = 1.0 / cos_phi(terrain)[:, INT]                                      # surface area weights
    D = dose[:, :, INT]
    mu = (D * w).sum(axis=(1, 2)) / w.sum()
    var = (w * (D - mu[:, None, None]) ** 2).sum(axis=(1, 2)) / w.sum()
    cv = np.sqrt(var) / mu
    red = (w * (D > REDUND_K * mu[:, None, None])).sum(axis=(1, 2)) / w.sum()
    under = (w * (D < UNDER_K * mu[:, None, None])).sum(axis=(1, 2)) / w.sum()
    return cv, red, under, mu


def path_metrics(xs, zs, terrain):
    L = 0.0
    for zp in zs:
        L += np.sum(np.hypot(np.diff(zp), Y_RES)) + Y_RES        # along-track incl. z
    L += np.sum(np.abs(np.diff(xs)))
    agl = np.array([zp - h_fun(xk, Y, terrain) for xk, zp in zip(xs, zs)])
    return L, agl


def evaluate(cfg, d, eps, model='jacobian'):
    xs, zs, terrain = plan(d, cfg)
    dose = dose_maps(xs, zs, terrain, eps, model)
    cv, red, under, mu = stats(dose, terrain)
    Dh = dose[:, :, INT] / cos_phi(terrain)[None][:, :, INT]      # mass per horizontal area
    cv_h = Dh.std(axis=(1, 2)) / Dh.mean(axis=(1, 2))
    L, agl = path_metrics(xs, zs, terrain)
    return dict(cv=cv, red=red, under=under, mu=mu, n=len(xs), L=L, agl=agl, xs=xs,
                spacing=np.diff(xs), cv_h=cv_h)


def widest_spacing(d_vals, cv_vals, target):
    above = np.where(cv_vals > target)[0]
    if len(above) == 0:
        return float(d_vals[-1])
    i = above[0]
    if i == 0:
        return np.nan
    d0, d1, c0, c1 = d_vals[i - 1], d_vals[i], cv_vals[i - 1], cv_vals[i]
    return float(d0 + (target - c0) * (d1 - d0) / (c1 - c0))


# ── Simulation ────────────────────────────────────────────────────────────────

def _job(args):
    cfg, d, eps, model = args
    return evaluate(cfg, d, eps, model)


def run_simulation(workers=6, chunk=25):
    """Independent deterministic evaluations in a process pool; trials are split into
    chunks (statistics are per trial) and re-assembled in order."""
    from concurrent.futures import ProcessPoolExecutor
    zdraw = np.random.default_rng(SEED).standard_normal((N_MC, N_STRIPS_MAX))
    eps = GPS * zdraw
    zero = np.zeros((1, N_STRIPS_MAX))
    jobs, keys = [], []

    def add(key, cfg, d, e, model='jacobian'):
        for c0 in range(0, e.shape[0], chunk):
            jobs.append((cfg, d, e[c0:c0 + chunk], model))
            keys.append(key)

    for cfg in list(CONFIGS) + ['tf35']:
        for d in D_POINTS:
            if cfg == 'tf35' and d != D_DESIGN:
                continue
            add(('main', cfg, d), _cfg(cfg), d, eps)
            add(('nf', cfg, d), _cfg(cfg), d, zero)
    add(('nf', 'tf35inf', D_DESIGN), ('', True, 'tf35', np.inf, False), D_DESIGN, zero)
    for cfg in CURVE_KEYS:
        for d in D_CURVE:
            add(('cmean', cfg, d), _cfg(cfg), d, eps[:N_CURVE])
            add(('cnf', cfg, d), _cfg(cfg), d, zero)
    for om in OMEGAS:
        if om == OMEGA_NOM:
            continue                                  # identical to the 'tf' curve
        cfg = ('', True, 'tf', om, False)
        for d in D_CURVE:
            add(('hmean', om, d), cfg, d, eps[:N_CURVE])
            add(('hnf', om, d), cfg, d, zero)
    for cfg in ('flat', 'fixed2', 'fixed35', 'tf'):
        add(('mnf', cfg, D_DESIGN), _cfg(cfg), D_DESIGN, zero, 'card')
        add(('mmc', cfg, D_DESIGN), _cfg(cfg), D_DESIGN, eps[:N_CURVE], 'card')
    with ProcessPoolExecutor(max_workers=workers) as ex:
        res = list(ex.map(_job, jobs))
    merged = {}
    for key, r in zip(keys, res):
        if key not in merged:
            merged[key] = dict(r)
        else:
            for f in ('cv', 'red', 'under', 'mu', 'cv_h'):
                merged[key][f] = np.concatenate([merged[key][f], r[f]])
    out = {'eps': eps, 'main': {}, 'nf': {}, 'curve': {}, 'heat': {}, 'model': {}}
    for (kind, a, d), r in merged.items():
        if kind in ('main', 'nf'):
            out[kind][(a, d)] = r
    for cfg in CURVE_KEYS:
        out['curve'][cfg] = {'mean': np.array([merged[('cmean', cfg, d)]['cv'].mean() for d in D_CURVE]),
                             'nf': np.array([merged[('cnf', cfg, d)]['cv'][0] for d in D_CURVE])}
    i_des = int(np.argmin(np.abs(D_CURVE - D_DESIGN)))
    for om in OMEGAS:
        src = out['curve']['tf'] if om == OMEGA_NOM else {
            'mean': np.array([merged[('hmean', om, d)]['cv'].mean() for d in D_CURVE]),
            'nf': np.array([merged[('hnf', om, d)]['cv'][0] for d in D_CURVE])}
        out['heat'][om] = dict(src)
        k = ('cnf', 'tf', D_CURVE[i_des]) if om == OMEGA_NOM else ('hnf', om, D_CURVE[i_des])
        out['heat'][om]['agl'] = merged[k]['agl']
        out['heat'][om]['d_agl'] = float(D_CURVE[i_des])
    for cfg in ('flat', 'fixed2', 'fixed35', 'tf'):
        out['model'][cfg] = (merged[('mnf', cfg, D_DESIGN)]['cv'][0],
                             merged[('mmc', cfg, D_DESIGN)]['cv'].mean())
    return out


def _cfg(key):
    if key == 'tf35':
        return ('TF ω = 2 s⁻¹ at 3.5 m AGL', True, 'tf35', OMEGA_NOM, False)
    return CONFIGS[key]


# ── Plotting ──────────────────────────────────────────────────────────────────

def plot_dose_surfaces(out, out_dir):
    keys = ['flat', 'fixed2', 'fixed35', 'tf', 'tf_adapt', 'fixed2_adapt']
    eps1 = out['eps'][:1]
    fig = plt.figure(figsize=(18, 10.5))
    sub = (slice(None, None, 2), slice(None, None, 10))
    XX, YY = np.meshgrid(X, Y)
    for i, key in enumerate(keys):
        xs, zs, terrain = plan(D_DESIGN, _cfg(key))
        dose = dose_maps(xs, zs, terrain, eps1)[0]
        cv, *_ , mu = stats(dose[None], terrain)
        hh = h_fun(XX, YY, terrain)
        rel = dose / mu[0]
        ax = fig.add_subplot(2, 3, i + 1, projection='3d')
        ax.plot_surface(XX[sub], YY[sub], hh[sub], facecolors=cm.viridis(np.clip(rel[sub] / 2, 0, 1)),
                        rstride=1, cstride=1, linewidth=0, antialiased=False, shade=False)
        ax.set_zlim(-3, 6)
        ax.set_xlabel('x (m)')
        ax.set_ylabel('y (m)')
        ax.set_zlabel('z (m)')
        ax.view_init(elev=35, azim=-60)
        ax.set_title(f'{CONFIGS[key][0]}\nN = {len(xs)} strips, CV = {100 * cv[0]:.1f} % (one GPS draw)',
                     fontsize=9)
    m = cm.ScalarMappable(cmap='viridis')
    m.set_clim(0, 2)
    fig.colorbar(m, ax=fig.axes, fraction=0.015, pad=0.02, label='dose / interior mean')
    fig.suptitle(f'S067 3D — surface dose on the terrain, d = {D_DESIGN} m (seed {SEED}, trial 0)')
    path = os.path.join(out_dir, 'dose_surfaces.png')
    plt.savefig(path, dpi=120, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_cv_heatmap(out, out_dir):
    fig, axes = plt.subplots(1, 2, figsize=(15, 5.5))
    labels = [('∞' if not np.isfinite(o) else f'{o:g}') for o in OMEGAS]
    for ax, key, ttl in ((axes[0], 'mean', f'MC mean CV ({N_CURVE} GPS trials)'),
                         (axes[1], 'nf', 'noise-free CV')):
        M = np.array([100 * out['heat'][o][key] for o in OMEGAS])
        im = ax.imshow(M, aspect='auto', origin='lower', cmap='magma_r', vmin=0, vmax=60,
                       extent=[D_CURVE[0] - 0.05, D_CURVE[-1] + 0.05, -0.5, len(OMEGAS) - 0.5])
        cs = ax.contour(D_CURVE, np.arange(len(OMEGAS)), M, levels=[10], colors='cyan')
        ax.clabel(cs, fmt='CV = %g %%', fontsize=8)
        ax.set_yticks(range(len(OMEGAS)))
        ax.set_yticklabels(labels)
        ax.set_xlabel('strip spacing d (m)')
        ax.set_ylabel('terrain-following bandwidth ω (s⁻¹)')
        ax.set_title(ttl)
        fig.colorbar(im, ax=ax, label='CV (%)')
    fig.suptitle('S067 3D — CV over (d, ω): it falls monotonically in both directions; '
                 'cyan = 10 % contour')
    plt.tight_layout()
    path = os.path.join(out_dir, 'cv_heatmap.png')
    plt.savefig(path, dpi=140, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_altitude_strip(out, out_dir):
    xk = 7.5                                       # crest line sin(2 pi x / 30) = 1
    fig, axes = plt.subplots(2, 1, figsize=(13, 7), sharex=True)
    hh = h_fun(xk, Y)
    axes[0].fill_between(Y, -2, hh, color='#a0805a', alpha=0.5, label='terrain h(x_k, y)')
    axes[0].plot(Y, hh + Z_AGL, 'k--', lw=1.2, label='command h + 2 m')
    for om, c in ((0.5, '#D62728'), (2.0, '#1F77B4'), (5.0, '#2CA02C')):
        zp = altitude_profile(xk, 0, True, 'tf', om)
        axes[0].plot(Y, zp, color=c, lw=1.3, label=f'drone, ω = {om:g} s⁻¹')
        axes[1].plot(Y, zp - hh - Z_AGL, color=c, lw=1.3, label=f'ω = {om:g}')
        axes[1].fill_between(Y, 0, zp - hh - Z_AGL, color=c, alpha=0.12)
    axes[0].axhline(Z_FIX_LO, color='grey', lw=1, ls=':', label='fixed z = 2.0 m')
    axes[0].axhline(Z_FIX_HI, color='orange', lw=1, ls=':', label='fixed z = 3.5 m')
    axes[0].set_ylabel('z (m)')
    axes[0].set_ylim(-2, 6)
    axes[0].legend(fontsize=8, ncol=3, loc='upper right')
    axes[0].set_title(f'Strip at x = {xk} m (hill crest line), flown in +y at {V} m/s')
    axes[1].set_ylabel('AGL error ε (m)')
    axes[1].set_xlabel('along-track y (m)')
    axes[1].axhline(0, color='k', lw=0.6)
    axes[1].legend(fontsize=8)
    for a in axes:
        a.grid(alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'altitude_strip.png')
    plt.savefig(path, dpi=140, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_profiles_and_curves(out, out_dir):
    fig, axes = plt.subplots(1, 2, figsize=(16, 5.5))
    ax = axes[0]
    j = int(np.argmin(np.abs(Y - Y_MID)))
    eps1 = out['eps'][:1]
    for key in ('fixed2', 'fixed35', 'tf', 'tf_adapt', 'flat'):
        xs, zs, terrain = plan(D_DESIGN, _cfg(key))
        dose = dose_maps(xs, zs, terrain, eps1)
        mu = stats(dose, terrain)[3][0]
        ax.plot(X, dose[0, j] / mu, color=COLORS[key], lw=1.0 if key != 'flat' else 1.4,
                ls='--' if key == 'flat' else '-', label=CONFIGS[key][0])
    ax.axhline(1, color='grey', lw=0.6)
    ax.set_xlim(0, 30)
    ax.set_xlabel('cross-track x (m)')
    ax.set_ylabel(f'dose / interior mean at y = {Y[j]:.2f} m')
    ax.set_title(f'Cross-track profiles, d = {D_DESIGN} m, GPS trial 0 (first 30 m)')
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)
    ax = axes[1]
    for key in CURVE_KEYS:
        ax.plot(D_CURVE, 100 * out['curve'][key]['mean'], color=COLORS[key], lw=1.5,
                label=f'{CONFIGS[key][0]} (MC mean)')
        ax.plot(D_CURVE, 100 * out['curve'][key]['nf'], color=COLORS[key], lw=0.8, ls=':')
    ax.axhline(100 * CV_TARGET, color='grey', ls='--', lw=1)
    ax.set_xlabel('strip spacing d (m)')
    ax.set_ylabel('area-weighted CV (%)')
    ax.set_ylim(0, 80)
    ax.set_title(f'CV vs spacing (solid: MC mean of {N_CURVE} GPS trials; dotted: noise-free)')
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'profiles_cv_curves.png')
    plt.savefig(path, dpi=140, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_redundancy(out, out_dir):
    fig, axes = plt.subplots(1, 3, figsize=(18, 5.5))
    XX, YY = np.meshgrid(X, Y)
    eps1 = out['eps'][:1]
    for ax, key in zip(axes, ('fixed2', 'fixed35', 'tf')):
        xs, zs, terrain = plan(D_DESIGN, _cfg(key))
        dose = dose_maps(xs, zs, terrain, eps1)
        cv, red, under, mu = stats(dose, terrain)
        rel = dose[0] / mu[0]
        cls = np.where(rel > REDUND_K, 2, np.where(rel < UNDER_K, 0, 1))
        ax.imshow(cls, origin='lower', extent=[0, FIELD_W, 0, FIELD_L], cmap='coolwarm',
                  vmin=0, vmax=2, aspect='equal', interpolation='nearest')
        c = ax.contour(XX, YY, h_fun(XX, YY), levels=np.arange(-1.5, 1.51, 0.5), colors='k',
                       linewidths=0.6)
        ax.clabel(c, fontsize=6, fmt='%.1f')
        ax.set_title(f'{CONFIGS[key][0]}\nred > 1.5×mean {100 * red[0]:.1f} %, '
                     f'blue < 0.5×mean {100 * under[0]:.1f} % (trial 0)', fontsize=9)
        ax.set_xlabel('x (m)')
        ax.set_ylabel('y (m)')
    fig.suptitle(f'S067 3D — over-dosed (red) and under-dosed (blue) cells, d = {D_DESIGN} m; '
                 'contours = terrain height (m)')
    plt.tight_layout()
    path = os.path.join(out_dir, 'redundancy_maps.png')
    plt.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def save_animation(out, out_dir):
    xs, zs, terrain = plan(ANIM_D, CONFIGS['tf'])
    eps1 = out['eps'][:1]
    sx, sy = 10, 2                                     # display grid 0.5 m x 1 m
    XX, YY = np.meshgrid(X, Y)
    hh = h_fun(XX, YY)
    per = []
    acc = np.zeros((Y.size, X.size))
    for k in range(len(xs)):
        acc = acc + dose_maps(xs[k:k + 1], zs[k:k + 1], terrain, eps1[:, k:k + 1])[0]
        per.append(acc.copy())
    final_mu = stats(per[-1][None], terrain)[3][0]
    n_frames = len(xs) + 1
    fig = plt.figure(figsize=(9, 7))
    ax = fig.add_subplot(111, projection='3d')

    def update(f):
        ax.cla()
        dose = per[f - 1] if f > 0 else np.zeros_like(acc)
        rel = np.clip(dose / final_mu / 2, 0, 1)
        ax.plot_surface(XX[::sy, ::sx], YY[::sy, ::sx], hh[::sy, ::sx],
                        facecolors=cm.viridis(rel[::sy, ::sx]), rstride=1, cstride=1,
                        linewidth=0, antialiased=False, shade=False)
        if f > 0:
            k = f - 1
            ax.plot(np.full(Y.size, xs[k]), Y, zs[k], color='red', lw=1.5)
            yd = Y[-1] if k % 2 == 0 else Y[0]
            ax.scatter([xs[k]], [yd], [float(np.interp(yd, Y, zs[k]))], color='red', s=40)
            cv = stats(dose[None], terrain)[0][0]
            ttl = f'strip {f}/{len(xs)}: interior CV so far = {100 * cv:.1f} %'
        else:
            ttl = 'empty field'
        ax.set_xlim(0, FIELD_W)
        ax.set_ylim(0, FIELD_L)
        ax.set_zlim(-3, 6)
        ax.set_xlabel('x (m)')
        ax.set_ylabel('y (m)')
        ax.set_zlabel('z (m)')
        ax.view_init(elev=35, azim=-60)
        ax.set_title(f'S067 3D terrain-following spray, ω = {OMEGA_NOM:g} s⁻¹, d = {ANIM_D} m\n{ttl}')
        return []

    anim = animation.FuncAnimation(fig, update, frames=n_frames, interval=250, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    anim.save(path, writer='pillow', fps=4, dpi=70)
    plt.close(fig)
    print(f'Saved: {path}  ({n_frames} frames)')


# ── Report ────────────────────────────────────────────────────────────────────

def _resolution_check():
    """Noise-free CV at d = D_DESIGN with a 0.25 m along-track grid instead of 0.5 m."""
    global Y
    y_saved = Y
    Y = np.arange(0.125, FIELD_L, 0.25)
    try:
        chk = evaluate(CONFIGS['tf'], D_DESIGN, np.zeros((1, N_STRIPS_MAX)))['cv'][0]
        chk2 = evaluate(CONFIGS['fixed2'], D_DESIGN, np.zeros((1, N_STRIPS_MAX)))['cv'][0]
    finally:
        Y = y_saved
    return chk, chk2


def report(out):
    print('\n=== S067 3D Spray Overlap on Undulating Terrain — Results ===')
    g = np.hypot(hx_fun(*np.meshgrid(X, Y)), hy_fun(*np.meshgrid(X, Y)))
    print(f'  Terrain: h in [{-A_HILL}, {A_HILL}] m, max slope {np.degrees(np.arctan(g.max())):.1f} deg, '
          f'max |grad h| {g.max():.3f}; nominal sigma_s = {Z_AGL * S_U:.2f} m at {Z_AGL} m AGL')
    cp = cos_phi(True)[:, INT]
    w = 1 / cp
    mu_c = (cp * w).sum() / w.sum()
    print(f'  Slope floor: a perfectly uniform deposit per horizontal area gives a surface dose '
          f'proportional to cos(phi), whose area-weighted CV is '
          f'{100 * np.sqrt((w * (cp - mu_c) ** 2).sum() / w.sum()) / mu_c:.2f} %')
    print('  Noise-free CV per horizontal area (no cos(phi) factor) at d = '
          f'{D_DESIGN}: ' + ', '.join(f"{k} {100 * out['nf'][(k, D_DESIGN)]['cv_h'][0]:.2f} %"
                                      for k in ('flat', 'fixed2', 'fixed35', 'tf', 'tf35')))
    lag = [A_HILL * (2 * np.pi * V / LAMBDA) / math.hypot(o, 2 * np.pi * V / LAMBDA) for o in OMEGAS[:-1]]
    print('  First-order lag, along-track terrain frequency 2 pi v / lambda = '
          f'{2 * np.pi * V / LAMBDA:.3f} rad/s -> analytic AGL error amplitude on the crest line: ' +
          ', '.join(f'omega {o:g}: {a:.3f} m' for o, a in zip(OMEGAS[:-1], lag)))
    print(f'  Card altitude bound z >= 1.0 m vs terrain-following command min(h) + 2 = '
          f'{-A_HILL + Z_AGL:.1f} m (bound conflicts; an AGL floor of {AGL_FLOOR} m is asserted instead)')
    nf_flat = {d: out['nf'][('flat', d)]['cv'][0] for d in D_POINTS}
    print('  2D-base check (flat, noise-free CV): ' +
          ', '.join(f'd = {d}: {100 * v:.2f} %' for d, v in nf_flat.items()) +
          ' (2D S067: 0.01 / 10.00 / 30.72 %)')
    for d in D_POINTS:
        print(f'  --- d = {d} m ({N_MC} GPS trials, common random numbers) ---')
        print('    config                                  N   path 3D   time    CV nf    CV mean ± std'
              '     P95    CV<=10%  >1.5mu  <0.5mu   AGL min-max   mean|eps_AGL|')
        for cfg in list(CONFIGS) + ['tf35']:
            if (cfg, d) not in out['main']:
                continue
            r, rn = out['main'][(cfg, d)], out['nf'][(cfg, d)]
            eps_agl = np.abs(r['agl'] - Z_AGL).mean()
            print(f"    {_cfg(cfg)[0][:38]:38s} {r['n']:3d} {r['L']:8.1f} m {r['L'] / V:6.1f} s "
                  f"{100 * rn['cv'][0]:6.2f} % {100 * r['cv'].mean():6.2f} ± {100 * r['cv'].std():5.2f} % "
                  f"{100 * np.percentile(r['cv'], 95):6.2f} % {100 * np.mean(r['cv'] <= CV_TARGET):5.1f} % "
                  f"{100 * r['red'].mean():6.2f} % {100 * r['under'].mean():6.2f} % "
                  f"{r['agl'].min():5.2f}-{r['agl'].max():5.2f} m {eps_agl:6.3f} m")
        cf, c2, c35, ct = (out['main'][(k, d)]['cv'] for k in ('flat', 'fixed2', 'fixed35', 'tf'))
        ca = out['main'][('tf_adapt', d)]['cv']
        print(f'    per trial: TF < fixed 2.0 in {int(np.sum(ct < c2))}/{N_MC}; TF < fixed 3.5 in '
              f'{int(np.sum(ct < c35))}/{N_MC}; TF+adaptive < TF in {int(np.sum(ca < ct))}/{N_MC}; '
              f'TF - flat CV = {100 * (ct - cf).mean():+.2f} ± {100 * (ct - cf).std():.2f} pp')
        sp = out['main'][('tf_adapt', d)]['spacing']
        sp2 = out['main'][('fixed2_adapt', d)]['spacing']
        print(f'    adaptive spacing range: TF {sp[:-1].min():.3f}-{sp[:-1].max():.3f} m, '
              f'fixed 2.0 m {sp2[:-1].min():.3f}-{sp2[:-1].max():.3f} m (last strip, clipped at '
              f'the field edge, excluded)')
    r35 = out['main'][('tf35', D_DESIGN)]
    print(f"  Ablation at d = {D_DESIGN}: terrain following at 3.5 m AGL: CV mean "
          f"{100 * r35['cv'].mean():.2f} % (P95 {100 * np.percentile(r35['cv'], 95):.2f} %), "
          f"lower than fixed z = 3.5 m in "
          f"{int(np.sum(r35['cv'] < out['main'][('fixed35', D_DESIGN)]['cv']))}/{N_MC}; noise-free "
          f"{100 * out['nf'][('tf35', D_DESIGN)]['cv'][0]:.2f} % with the lag vs "
          f"{100 * out['nf'][('tf35inf', D_DESIGN)]['cv'][0]:.2f} % with perfect following at 3.5 m AGL")
    flat_path = out['main'][('flat', D_DESIGN)]['L']
    tf_path = out['main'][('tf', D_DESIGN)]['L']
    terr = np.mean([np.sum(np.hypot(np.diff(h_fun(xk, Y)), Y_RES)) + Y_RES
                    for xk in out['main'][('tf', D_DESIGN)]['xs']])
    print(f'  Path length at d = {D_DESIGN}: flat {flat_path:.1f} m, terrain following {tf_path:.1f} m '
          f'(+{100 * (tf_path / flat_path - 1):.2f} %); card terrain-profile strip length '
          f'{terr:.2f} m per 50 m strip (+{100 * (terr / FIELD_L - 1):.2f} %)')
    print(f'  Widest spacing with CV <= {100 * CV_TARGET:.0f} % (MC mean of {N_CURVE} trials / noise-free):')
    for key in CURVE_KEYS:
        c = out['curve'][key]
        print(f"    {CONFIGS[key][0][:44]:44s} {widest_spacing(D_CURVE, c['mean'], CV_TARGET):.3f} m / "
              f"{widest_spacing(D_CURVE, c['nf'], CV_TARGET):.3f} m")
    idx = [int(np.argmin(np.abs(D_CURVE - dd))) for dd in (0.6, 1.0, 1.4, 1.8)]
    print(f'  Terrain-following bandwidth sweep (MC mean CV of {N_CURVE} trials); flat field for '
          'reference: ' + ', '.join(f"d = {D_CURVE[i]:.1f}: {100 * out['curve']['flat']['mean'][i]:.2f} %"
                                    for i in idx))
    for om in OMEGAS:
        hdat = out['heat'][om]
        idx = [int(np.argmin(np.abs(D_CURVE - dd))) for dd in (0.6, 1.0, 1.4, 1.8)]
        eps_agl = hdat['agl'] - Z_AGL
        print(f"    omega {('inf' if not np.isfinite(om) else f'{om:g}'):>4s}: widest d (mean / nf) "
              f"{widest_spacing(D_CURVE, hdat['mean'], CV_TARGET):.3f} / "
              f"{widest_spacing(D_CURVE, hdat['nf'], CV_TARGET):.3f} m; CV at d = " +
              ', '.join(f'{D_CURVE[i]:.1f}: {100 * hdat["mean"][i]:.2f} %' for i in idx) +
              f"; AGL error at d = {hdat['d_agl']:.1f}: max |eps| {np.abs(eps_agl).max():.3f} m, "
              f'min AGL {hdat["agl"].min():.3f} m')
    mono_d = all(np.all(np.diff(out['heat'][om]['nf']) > -1e-4) for om in OMEGAS)
    mono_w = all(np.all(np.diff([out['heat'][om]['mean'][i] for om in OMEGAS]) < 1e-4)
                 for i in range(len(D_CURVE)))
    print(f'  Noise-free CV non-decreasing in d for every omega: {mono_d}; '
          f'MC mean CV non-increasing in omega at every d: {mono_w}')
    print(f'  Dose-model ablation at d = {D_DESIGN} (card formula Q0/cos(phi) exp(-dx^2/2sigma^2), '
          f'unnormalised): noise-free / MC mean CV')
    for cfg, (nfv, mcv) in out['model'].items():
        rj = out['main'][(cfg, D_DESIGN)]['cv'][:N_CURVE].mean()
        print(f'    {CONFIGS[cfg][0][:40]:40s} card model {100 * nfv:6.2f} / {100 * mcv:6.2f} %; '
              f'this model {100 * out["nf"][(cfg, D_DESIGN)]["cv"][0]:6.2f} / {100 * rj:6.2f} %')
    chk, chk2 = _resolution_check()
    print(f"  Along-track resolution check (noise-free CV at d = {D_DESIGN}): TF {100 * out['nf'][('tf', D_DESIGN)]['cv'][0]:.3f} % "
          f"(0.5 m) vs {100 * chk:.3f} % (0.25 m); fixed 2.0 m "
          f"{100 * out['nf'][('fixed2', D_DESIGN)]['cv'][0]:.3f} % vs {100 * chk2:.3f} %")


if __name__ == '__main__':
    res = run_simulation()
    report(res)
    od = os.path.normpath(OUTPUT_DIR)
    os.makedirs(od, exist_ok=True)
    plot_dose_surfaces(res, od)
    plot_cv_heatmap(res, od)
    plot_altitude_strip(res, od)
    plot_profiles_and_curves(res, od)
    plot_redundancy(res, od)
    save_animation(res, od)
