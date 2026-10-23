"""
S048 3D Lawnmower Coverage on Terrain
=====================================
A single drone flies a boustrophedon (lawnmower) scan of a 200 x 200 m area
over undulating terrain (sum of 5 Gaussian hills).  A proportional AGL
controller (K_z = 1.5 1/s, |z_dot| <= 3 m/s) tracks a constant 8 m clearance
while the drone moves horizontally at 10 m/s, exactly as the 2D-folder S048
moves it.  The nadir camera footprint radius scales with the actual height
above ground: r = r_s * h / h_AGL (r_s = 4 m at 8 m AGL).

Configurations (terrain-following unless stated):
  card   under-lap      d = 12 m
  card   optimal        d_i = 2 r_s cos(phi_ct,i)   (card maths section)
  card   over-lap       d = 7 m
  added  fixed 2 r_s    d = 8 m                      (no slope correction)
  added  card alt.      d_i = 2 r_s / cos(phi_ct,i)  (card comparison list)
  added  flat altitude  d = 8 m, constant altitude max(terrain) + 8 m (the 2D plan)

Coverage is evaluated on 0.25 m cells by sweeping the footprint between steps
(as in 2D).  A showcase terrain (seed 0) is plotted; all metrics are also
reported over N_TERRAIN random terrains (same terrains for every configuration).

Usage:
    conda activate drones
    python src/03_environmental_sar/3d/s048_3d_lawnmower.py
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

SCRIPT_DIR   = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, '..', '..', '..'))
OUTPUT_DIR   = os.path.join(PROJECT_ROOT, 'outputs', '03_environmental_sar', '3d',
                            's048_3d_lawnmower')

# -- Parameters (card) ------------------------------------------------------------
AREA      = 200.0     # m  (x = along-track, y = cross-track)
N_HILLS   = 5
A_RANGE   = (5.0, 15.0)   # m  hill amplitude
S_RANGE   = (20.0, 50.0)  # m  hill sigma
H_AGL     = 8.0       # m  target clearance
TAN_HFOV  = 0.5       # tan(26.57 deg)
R_S       = H_AGL * TAN_HFOV   # 4 m nominal footprint radius
V_CRUISE  = 10.0      # m/s
ZDOT_MAX  = 3.0       # m/s
K_Z       = 1.5       # 1/s
P_CRUISE  = 150.0     # W
P_HOVER   = 200.0     # W
P_CLIMB   = 250.0     # W
T_TURN    = 4.0       # s
DT        = 0.5       # s
AGL_FLOOR, AGL_CEIL = 2.0, 20.0   # m
D_UNDER, D_OVER = 12.0, 7.0
D_FIXED   = 2 * R_S   # added: 8 m
SEED_SHOW = 0         # showcase terrain
N_TERRAIN = 20        # added: terrains for the Monte Carlo (seeds 0..19)
GIF_STEP  = 12        # trajectory samples per GIF frame

CONFIGS = [  # key, label, strip rule, terrain-following?
    ('under',   'Under-lap d=12 m',             'under', True),
    ('optimal', 'Optimal d=2r_s·cosφ (card)',    'cos',   True),
    ('over',    'Over-lap d=7 m',               'over',  True),
    ('fixed8',  'Fixed d=2r_s=8 m',             'fixed', True),
    ('divcos',  'd=2r_s/cosφ (card list)',      'divcos', True),
    ('flat',    'Flat altitude, d=8 m (2D plan)', 'fixed', False),
]
CARD_KEYS = ('under', 'optimal', 'over')
COLORS = {'under': '#e17055', 'optimal': '#00b894', 'over': '#0984e3',
          'fixed8': '#6c5ce7', 'divcos': '#fdcb6e', 'flat': '#636e72'}

XC = np.arange(0.5, AREA, 1.0)          # 1 m cell centres
DX, DY = np.meshgrid(XC, XC)            # 1 m DEM grid; DX: x (along-track), DY: y (cross-track)
COV_RES = 0.25                          # added: coverage cell size (m), see README
GC = np.arange(COV_RES / 2, AREA, COV_RES)
GX, GY = np.meshgrid(GC, GC)            # coverage grid


# -- Terrain --------------------------------------------------------------------------

def make_terrain(seed):
    rng = np.random.default_rng(seed)
    hills = np.c_[rng.uniform(0, AREA, N_HILLS), rng.uniform(0, AREA, N_HILLS),
                  rng.uniform(*A_RANGE, N_HILLS), rng.uniform(*S_RANGE, N_HILLS)]

    def h(x, y):
        x = np.asarray(x, float); y = np.asarray(y, float)
        out = np.zeros(np.broadcast(x, y).shape)
        for xk, yk, ak, sk in hills:
            out = out + ak * np.exp(-((x - xk) ** 2 + (y - yk) ** 2) / (2 * sk ** 2))
        return out

    dem = h(DX, DY)                           # 1 m DEM at the cell centres
    dzdy, dzdx = np.gradient(dem, 1.0)        # central differences
    dzdy_cov = np.gradient(h(GX, GY), COV_RES)[0]   # cross-track slope on the coverage grid
    return dict(h=h, dem=dem, dzdx=dzdx, dzdy=dzdy, dzdy_cov=dzdy_cov, hills=hills, seed=seed)


def strip_slope(ter, y):
    """Mean |cross-track slope angle| along the strip at cross-track position y."""
    iy = int(np.clip(np.floor(y), 0, AREA - 1))
    return float(np.mean(np.abs(np.arctan(ter['dzdy'][iy, :]))))


def strip_centres(ter, rule):
    if rule in ('under', 'over', 'fixed'):
        d = {'under': D_UNDER, 'over': D_OVER, 'fixed': D_FIXED}[rule]
        n = int(np.ceil(AREA / d))
        return np.array([(i + 0.5) * d for i in range(n)]), np.full(n, d)

    def width(y):
        c = np.cos(strip_slope(ter, y))
        return 2 * R_S * c if rule == 'cos' else 2 * R_S / c

    ys, ds = [], []
    d0 = width(0.0)
    y = d0 / 2
    for _ in range(3):                         # fixed point: d evaluated at its own centre
        d0 = width(y); y = d0 / 2
    ys.append(y); ds.append(d0)
    while ys[-1] + ds[-1] / 2 < AREA:
        y_new = ys[-1] + ds[-1]
        for _ in range(3):                     # y_i = y_{i-1} + (d_{i-1} + d_i) / 2
            d_new = width(min(y_new, AREA - 0.5))
            y_new = ys[-1] + 0.5 * (ds[-1] + d_new)
        ys.append(y_new); ds.append(d_new)
    return np.array(ys), np.array(ds)


def waypoints_xy(ys):
    wp = []
    for i, y in enumerate(ys):
        xa, xb = (0.0, AREA) if i % 2 == 0 else (AREA, 0.0)
        wp += [(xa, min(y, AREA)), (xb, min(y, AREA))]
    return np.array(wp)


# -- Flight simulation -------------------------------------------------------------------

def _capsule(p0, p1, r):
    """Cells within r of segment p0-p1 (horizontal), restricted to a bounding box."""
    n = len(GC)
    lo = np.maximum(np.floor((np.minimum(p0, p1) - r) / COV_RES).astype(int) - 1, 0)
    hi = np.minimum(np.ceil((np.maximum(p0, p1) + r) / COV_RES).astype(int) + 1, n)
    sx, sy = slice(lo[1], hi[1]), slice(lo[0], hi[0])   # rows = y, cols = x
    cx, cy = GX[sx, sy], GY[sx, sy]
    seg = p1 - p0; L2 = float(seg @ seg)
    t = np.zeros_like(cx) if L2 < 1e-12 else np.clip(
        ((cx - p0[0]) * seg[0] + (cy - p0[1]) * seg[1]) / L2, 0, 1)
    m = (cx - p0[0] - t * seg[0]) ** 2 + (cy - p0[1] - t * seg[1]) ** 2 <= r * r
    return (sx, sy), m


SUB = np.linspace(0, 1, 6)[1:]   # sub-samples of a step used for the terrain checks


def _vertical(h, z, new):
    z_cmd = float(h(*new)) + H_AGL
    return z_cmd, z + float(np.clip(K_Z * (z_cmd - z), -ZDOT_MAX, ZDOT_MAX)) * DT


def _min_clearance(h, pos, z, new, z_new):
    q = pos[None, :] + (new - pos)[None, :] * SUB[:, None]
    return float(np.min(z + (z_new - z) * SUB - h(q[:, 0], q[:, 1])))


def fly(ter, rule, follow, speed_limit=True):
    """
    speed_limit=True (used for all results): the horizontal step is shortened
    (bisection on the step fraction) whenever the card's climb-limited AGL
    controller would otherwise let the drone sink below the 2 m safety floor
    on any of 5 sub-samples of the step; the drone may hover in place.
    speed_limit=False is the card's constant 10 m/s; it is only used as a
    diagnostic (records the lowest clearance, no assertion).
    """
    h = ter['h']
    ys, ds = strip_centres(ter, rule)
    wp = waypoints_xy(ys)
    n_strips = len(ys)
    z_flat = float(ter['dem'].max()) + H_AGL
    pos = wp[0].copy()
    z = z_flat if not follow else float(h(*pos)) + H_AGL
    traj = [np.r_[pos, z]]; zcmd_log = [z]
    passes = np.zeros(GX.shape, np.int16)
    covered = np.zeros(GX.shape, bool)
    cov_log, dist3d = [0.0], [0.0]
    n_slowed, min_clear_diag = 0, np.inf
    for a, b in zip(wp[:-1], wp[1:]):
        leg = float(np.linalg.norm(b - a))
        if leg < 1e-9:
            continue
        step_nom = leg / np.ceil(leg / (V_CRUISE * DT))   # equal steps per leg (as in 2D)
        this = np.zeros(GX.shape, bool)
        while float(np.linalg.norm(b - pos)) > 1e-9:
            rem = b - pos; drem = float(np.linalg.norm(rem))
            new = b.copy() if drem <= step_nom + 1e-9 else pos + rem * (step_nom / drem)
            if follow:
                z_cmd, z_new = _vertical(h, z, new)
                clear = _min_clearance(h, pos, z, new, z_new)
                if not speed_limit:
                    min_clear_diag = min(min_clear_diag, clear)
                elif clear < AGL_FLOOR:
                    n_slowed += 1
                    lo, hi = 0.0, 1.0
                    for _ in range(20):
                        mid = 0.5 * (lo + hi)
                        nm = pos + (new - pos) * mid
                        zc, zn = _vertical(h, z, nm)
                        if _min_clearance(h, pos, z, nm, zn) >= AGL_FLOOR:
                            lo = mid
                        else:
                            hi = mid
                    new = pos + (new - pos) * lo
                    z_cmd, z_new = _vertical(h, z, new)
            else:
                z_cmd = z_flat; z_new = z_flat
            if speed_limit:   # invariant: never below the 2 m floor (hence never underground)
                assert _min_clearance(h, pos, z, new, z_new) >= AGL_FLOOR - 1e-6, 'below floor'
            h0 = z - float(h(*pos)); h1 = z_new - float(h(*new))
            r = R_S * min(h0, h1) / H_AGL          # conservative swept radius
            sl, m = _capsule(pos, new, r)
            this[sl] |= m; covered[sl] |= m
            dist3d.append(dist3d[-1] + float(np.linalg.norm(np.r_[new - pos, z_new - z])))
            pos, z = new, z_new
            traj.append(np.r_[pos, z]); zcmd_log.append(z_cmd)
            cov_log.append(100.0 * covered.mean())
        passes += this
    traj = np.array(traj)
    ter_under = h(traj[:, 0], traj[:, 1])
    agl = traj[:, 2] - ter_under
    dxy = np.linalg.norm(np.diff(traj[:, :2], axis=0), axis=1)
    dzs = np.diff(traj[:, 2])
    L_h, L_3d = float(dxy.sum()), float(dist3d[-1])
    D_up, D_down = float(dzs[dzs > 0].sum()), float(-dzs[dzs < 0].sum())
    t_vert = float(np.sum(np.abs(dzs) > 1e-9) * DT)
    t_fly = (len(traj) - 1) * DT
    E_cruise = P_CRUISE * t_fly          # time-based: slowed steps cost time at cruise power
    E_turn = (P_HOVER - P_CRUISE) * T_TURN * (n_strips - 1)
    E_climb = (P_CLIMB - P_CRUISE) / ZDOT_MAX * D_up          # implemented (see README)
    vz_avg = (D_up + D_down) / t_vert if t_vert > 0 else 0.0
    E_climb_card = P_CLIMB * (D_up + D_down) / vz_avg if vz_avg > 0 else 0.0
    sc = passes[passes > 0]
    return dict(rule=rule, follow=follow, ys=ys, ds=ds, wp=wp, n_strips=n_strips, traj=traj,
                zcmd=np.array(zcmd_log), ter_under=ter_under, agl=agl, passes=passes,
                cov_log=np.array(cov_log), dist3d=np.array(dist3d),
                coverage=100.0 * np.mean(passes > 0), L_h=L_h, L_3d=L_3d, D_up=D_up,
                D_down=D_down, E_cruise=E_cruise, E_turn=E_turn, E_climb=E_climb,
                E=E_cruise + E_turn + E_climb, E_card=E_cruise + E_turn + E_climb_card,
                E_climb_card=E_climb_card,
                agl_err=agl - H_AGL, mean_pass=float(sc.mean()),
                multi=100.0 * np.mean(passes >= 2), cv_pass=float(sc.std() / sc.mean()),
                floor_viol=int(np.sum(agl < AGL_FLOOR)), ceil_viol=int(np.sum(agl > AGL_CEIL)),
                n_steps=len(traj), t_fly=t_fly, t_mission=t_fly + T_TURN * (n_strips - 1),
                n_slowed=n_slowed, min_clear_diag=min_clear_diag)


def gap_slope_stats(ter, res):
    """Mean |cross-track slope| of uncovered cells vs all cells (where do gaps occur?)."""
    slope = np.degrees(np.abs(np.arctan(ter['dzdy_cov'])))
    unc = res['passes'] == 0
    return float(slope.mean()), (float(slope[unc].mean()) if unc.any() else float('nan'))


def run_simulation():
    shows = {}
    ter0 = make_terrain(SEED_SHOW)
    for key, label, rule, follow in CONFIGS:
        shows[key] = fly(ter0, rule, follow)
    mc = {key: [] for key, *_ in CONFIGS}
    diag = []   # card's constant-speed controller: lowest clearance above terrain
    for seed in range(N_TERRAIN):
        ter = ter0 if seed == SEED_SHOW else make_terrain(seed)
        diag.append(fly(ter, 'cos', True, speed_limit=False)['min_clear_diag'])
        for key, label, rule, follow in CONFIGS:
            r = shows[key] if seed == SEED_SHOW else fly(ter, rule, follow)
            mc[key].append(dict(coverage=r['coverage'], L_3d=r['L_3d'], E=r['E'],
                                t_mission=r['t_mission'], n_slowed=r['n_slowed'],
                                E_climb=r['E_climb'], n=r['n_strips'],
                                within1=100 * np.mean(np.abs(r['agl_err']) <= 1.0),
                                p95=float(np.percentile(np.abs(r['agl_err']), 95)),
                                agl_min=float(r['agl'].min()), agl_max=float(r['agl'].max()),
                                floor=r['floor_viol'], ceil=r['ceil_viol'],
                                relief=float(ter['dem'].max() - ter['dem'].min()),
                                maxslope=float(np.degrees(np.arctan(np.hypot(ter['dzdx'], ter['dzdy']).max())))))
    return dict(ter=ter0, shows=shows, mc=mc, diag=np.array(diag))


# -- Plots ---------------------------------------------------------------------------------

LABELS = {k: lab for k, lab, *_ in CONFIGS}


def _save(fig, name):
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    path = os.path.join(OUTPUT_DIR, name)
    fig.savefig(path, dpi=120, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_trajectory_3d(data):
    ter = data['ter']
    fig = plt.figure(figsize=(17, 5.5))
    for i, key in enumerate(CARD_KEYS):
        r = data['shows'][key]
        ax = fig.add_subplot(1, 3, i + 1, projection='3d')
        ax.plot_surface(DX[::4, ::4], DY[::4, ::4], ter['dem'][::4, ::4], cmap='terrain',
                        alpha=0.55, lw=0)
        ax.plot(*r['traj'].T, color=COLORS[key], lw=0.8)
        ax.set_xlabel('x along-track (m)'); ax.set_ylabel('y cross-track (m)'); ax.set_zlabel('z (m)')
        ax.set_zlim(0, ter['dem'].max() + 15)
        ax.view_init(elev=30, azim=-60)
        ax.set_title(f'{LABELS[key]}: {r["n_strips"]} strips\ncoverage {r["coverage"]:.2f}%, '
                     f'L3D {r["L_3d"]:.0f} m', fontsize=9)
    fig.suptitle(f'S048 3D - Terrain-following boustrophedon paths (terrain seed {SEED_SHOW})',
                 fontsize=12, fontweight='bold')
    _save(fig, 'trajectory_3d.png')


def plot_altitude_timeseries(data):
    r = data['shows']['optimal']
    t = np.arange(r['n_steps']) * DT
    fig, axes = plt.subplots(2, 2, figsize=(15, 7), sharex='col',
                             gridspec_kw={'height_ratios': [2, 1]})
    k_end = np.searchsorted(t, 120.0)
    for col, sl in enumerate((slice(None), slice(0, k_end))):
        ax = axes[0, col]
        ax.plot(t[sl], r['ter_under'][sl], color='saddlebrown', lw=1, label='terrain under drone')
        ax.plot(t[sl], r['zcmd'][sl], 'k--', lw=0.8, label='z_cmd = terrain + 8 m')
        ax.plot(t[sl], r['traj'][sl, 2], color=COLORS['optimal'], lw=1.1, label='drone z_P')
        ax.set_ylabel('altitude (m)'); ax.grid(True, alpha=0.3); ax.legend(fontsize=8)
        ax.set_title('whole mission' if col == 0 else 'first 120 s', fontsize=10)
        ax2 = axes[1, col]
        ax2.plot(t[sl], r['agl'][sl], color=COLORS['optimal'], lw=1)
        ax2.axhline(H_AGL, color='k', ls='--', lw=0.8)
        ax2.axhspan(H_AGL - 1, H_AGL + 1, color='green', alpha=0.12, label='±1 m band')
        ax2.axhline(AGL_FLOOR, color='red', ls=':', lw=1, label='2 m safety floor')
        ax2.set_xlabel('time (s)'); ax2.set_ylabel('AGL h(t) (m)')
        ax2.grid(True, alpha=0.3); ax2.legend(fontsize=8)
    fig.suptitle('S048 3D - Altitude tracking, optimal (slope-adaptive) configuration',
                 fontsize=12, fontweight='bold')
    fig.tight_layout()
    _save(fig, 'altitude_timeseries.png')


def plot_agl_histogram(data):
    fig, ax = plt.subplots(figsize=(9, 4.6))
    bins = np.linspace(-6, 6, 61)
    for key in CARD_KEYS + ('fixed8',):
        e = data['shows'][key]['agl_err']
        ax.hist(e, bins=bins, histtype='step', lw=1.5, color=COLORS[key],
                label=f'{LABELS[key]}: {100*np.mean(np.abs(e) <= 1):.1f}% within ±1 m')
    ax.axvspan(-1, 1, color='green', alpha=0.1)
    ax.set_xlabel('AGL error ε = h - 8 m (m)'); ax.set_ylabel('samples (dt = 0.5 s)')
    ax.set_title('S048 3D - AGL tracking error distribution (terrain seed 0)', fontsize=11,
                 fontweight='bold')
    ax.grid(True, alpha=0.3); ax.legend(fontsize=8)
    _save(fig, 'agl_histogram.png')


def plot_coverage_maps(data):
    ter = data['ter']
    keys = ('under', 'optimal', 'over', 'fixed8', 'divcos', 'flat')
    cmap = mcolors.ListedColormap(['white', '#74b9ff', '#0984e3', '#2d3436'])
    fig, axes = plt.subplots(2, 3, figsize=(16, 10.5))
    for ax, key in zip(axes.ravel(), keys):
        r = data['shows'][key]
        im = ax.imshow(np.clip(r['passes'], 0, 3), origin='lower', extent=[0, AREA, 0, AREA],
                       cmap=cmap, vmin=-0.5, vmax=3.5, interpolation='nearest')
        ax.contour(DX, DY, ter['dem'], levels=8, colors='saddlebrown', linewidths=0.6, alpha=0.7)
        ax.set_title(f'{LABELS[key]}\ncoverage {r["coverage"]:.2f}%, '
                     f'uncovered {np.sum(r["passes"] == 0) * COV_RES ** 2:.1f} m²', fontsize=9)
        ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')
    cb = fig.colorbar(im, ax=axes, ticks=[0, 1, 2, 3], shrink=0.6)
    cb.ax.set_yticklabels(['0', '1', '2', '3+']); cb.set_label('distinct passes per 0.25 m cell')
    fig.suptitle('S048 3D - Coverage maps on the DEM (brown: terrain contours)', fontsize=12,
                 fontweight='bold')
    _save(fig, 'coverage_maps.png')


def plot_slope_strips(data):
    ter = data['ter']
    fig, axes = plt.subplots(1, 2, figsize=(14, 5.2))
    phi = np.degrees(np.arctan(ter['dzdy']))
    im = axes[0].imshow(phi, origin='lower', extent=[0, AREA, 0, AREA], cmap='coolwarm',
                        vmin=-np.abs(phi).max(), vmax=np.abs(phi).max())
    fig.colorbar(im, ax=axes[0], label='cross-track slope φ_ct (deg)')
    for y in data['shows']['optimal']['ys']:
        axes[0].axhline(y, color='k', lw=0.3, alpha=0.6)
    axes[0].set_title('Cross-track slope field with the optimal strip centres', fontsize=10)
    axes[0].set_xlabel('x (m)'); axes[0].set_ylabel('y (m)')
    for key in ('optimal', 'divcos', 'fixed8'):
        r = data['shows'][key]
        axes[1].step(r['ys'], r['ds'], where='mid', color=COLORS[key], lw=1.5,
                     label=f'{LABELS[key]} ({r["n_strips"]} strips)')
    slope_by_row = np.degrees([strip_slope(ter, y) for y in XC])
    ax2 = axes[1].twinx()
    ax2.plot(XC, slope_by_row, color='gray', lw=0.8, ls=':', label='mean |φ_ct| per row')
    ax2.set_ylabel('mean |φ_ct| along strip (deg)', color='gray')
    axes[1].set_xlabel('strip centre y (m)'); axes[1].set_ylabel('strip spacing d (m)')
    axes[1].set_title('Per-strip spacing', fontsize=10); axes[1].grid(True, alpha=0.3)
    axes[1].legend(fontsize=8, loc='upper left')
    fig.suptitle('S048 3D - Slope map and slope-adaptive strip widths', fontsize=12,
                 fontweight='bold')
    fig.tight_layout()
    _save(fig, 'slope_strips.png')


def plot_comparison_bars(data):
    keys = [k for k, *_ in CONFIGS]
    fig, axes = plt.subplots(1, 3, figsize=(17, 5))
    x = np.arange(len(keys))
    mc = data['mc']
    cov = np.array([[m['coverage'] for m in mc[k]] for k in keys])
    L = np.array([[m['L_3d'] for m in mc[k]] for k in keys])
    axes[0].bar(x, cov.mean(1), yerr=cov.std(1), color=[COLORS[k] for k in keys], capsize=4)
    axes[0].set_ylim(cov.min() - 3, 100.5); axes[0].set_ylabel('coverage (%)')
    axes[0].set_title(f'Coverage, mean ± std over {N_TERRAIN} terrains', fontsize=10)
    axes[1].bar(x, L.mean(1), yerr=L.std(1), color=[COLORS[k] for k in keys], capsize=4)
    axes[1].set_ylabel('3D path length (m)'); axes[1].set_title('3D path length', fontsize=10)
    s = data['shows']
    ec = np.array([s[k]['E_cruise'] for k in keys]) / 1e3
    et = np.array([s[k]['E_turn'] for k in keys]) / 1e3
    el = np.array([s[k]['E_climb'] for k in keys]) / 1e3
    axes[2].bar(x, ec, color='#b2bec3', label='cruise')
    axes[2].bar(x, et, bottom=ec, color='#636e72', label='turns')
    axes[2].bar(x, el, bottom=ec + et, color='#d63031', label='terrain-following climb')
    axes[2].set_ylabel('energy (kJ)'); axes[2].set_title('Energy, terrain seed 0', fontsize=10)
    axes[2].legend(fontsize=8)
    for ax in axes:
        ax.set_xticks(x); ax.set_xticklabels([LABELS[k] for k in keys], rotation=30, ha='right',
                                             fontsize=8)
        ax.grid(True, axis='y', alpha=0.3)
    fig.suptitle('S048 3D - Configuration comparison', fontsize=12, fontweight='bold')
    fig.tight_layout()
    _save(fig, 'comparison_bars.png')


def plot_coverage_vs_distance(data):
    fig, ax = plt.subplots(figsize=(10, 5))
    for k, *_ in CONFIGS:
        r = data['shows'][k]
        ax.plot(r['dist3d'], r['cov_log'], color=COLORS[k], lw=1.5,
                label=f'{LABELS[k]}: {r["coverage"]:.2f}%')
    ax.set_xlabel('3D distance flown (m)'); ax.set_ylabel('cumulative coverage (%)')
    ax.set_ylim(0, 101); ax.grid(True, alpha=0.3); ax.legend(fontsize=8, loc='lower right')
    ax.set_title('S048 3D - Coverage vs 3D distance (terrain seed 0)', fontsize=11,
                 fontweight='bold')
    _save(fig, 'coverage_vs_distance.png')


def save_animation(data):
    r = data['shows']['optimal']; ter = data['ter']
    traj = r['traj']; n = len(traj)
    n_frames = int(np.ceil((n - 1) / GIF_STEP)) + 1
    idx = [min(f * GIF_STEP, n - 1) for f in range(n_frames)]
    fig = plt.figure(figsize=(12, 5.2))
    ax = fig.add_subplot(1, 2, 1, projection='3d')
    ax.plot_surface(DX[::6, ::6], DY[::6, ::6], ter['dem'][::6, ::6], cmap='terrain', alpha=0.5, lw=0)
    ax.set_zlim(0, ter['dem'].max() + 15); ax.view_init(elev=32, azim=-60)
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)'); ax.set_zlabel('z (m)')
    (path,) = ax.plot([], [], [], color=COLORS['optimal'], lw=0.8)
    (dot,) = ax.plot([], [], [], 'o', color='red', ms=5)
    ax2 = fig.add_subplot(1, 2, 2)
    t = np.arange(n) * DT
    ax2.set_xlim(0, t[-1]); ax2.set_ylim(0, 14)
    ax2.axhspan(H_AGL - 1, H_AGL + 1, color='green', alpha=0.12)
    ax2.axhline(AGL_FLOOR, color='red', ls=':')
    ax2.set_xlabel('time (s)'); ax2.set_ylabel('AGL (m)'); ax2.grid(True, alpha=0.3)
    (agl_line,) = ax2.plot([], [], color=COLORS['optimal'], lw=0.8)
    ttl = fig.suptitle('')

    def update(f):
        k = idx[f]
        path.set_data(traj[:k + 1, 0], traj[:k + 1, 1]); path.set_3d_properties(traj[:k + 1, 2])
        dot.set_data([traj[k, 0]], [traj[k, 1]]); dot.set_3d_properties([traj[k, 2]])
        agl_line.set_data(t[:k + 1], r['agl'][:k + 1])
        ttl.set_text(f'S048 3D - optimal config, t = {t[k]:.0f} s, coverage {r["cov_log"][k]:.1f}%')
        return path, dot, agl_line, ttl

    ani = animation.FuncAnimation(fig, update, frames=n_frames, blit=False)
    p = os.path.join(OUTPUT_DIR, 'animation.gif')
    ani.save(p, writer='pillow', fps=10, dpi=70)
    plt.close(fig)
    print(f'Saved: {p} ({n_frames} frames)')


# -- Report ---------------------------------------------------------------------------------

def report(data):
    ter = data['ter']
    print(f'\nTerrain seed {SEED_SHOW}: relief {ter["dem"].min():.2f}-{ter["dem"].max():.2f} m, '
          f'max slope {np.degrees(np.arctan(np.hypot(ter["dzdx"], ter["dzdy"]).max())):.1f} deg, '
          f'max |cross-track slope| {np.degrees(np.arctan(np.abs(ter["dzdy"]).max())):.1f} deg')
    print('  hills (x, y, A, sigma): ' + '; '.join(
        f'({x:.0f}, {y:.0f}, {a:.1f}, {s:.1f})' for x, y, a, s in ter['hills']))
    print('\n-- Showcase terrain --')
    print(' config                         | strips | d range (m)  | coverage | uncovered | L_horiz | L_3D   | climb/desc (m) '
          '| E cruise/turn/climb = total (kJ) | E card formula | |eps|<=1 m | p95 |eps| | AGL min/max | <2 m | >20 m | mean passes | CV passes')
    for k, *_ in CONFIGS:
        r = data['shows'][k]
        e = np.abs(r['agl_err'])
        print(f' {LABELS[k]:30s} | {r["n_strips"]:6d} | {r["ds"].min():5.2f}-{r["ds"].max():5.2f} | '
              f'{r["coverage"]:7.2f}% | {np.sum(r["passes"] == 0) * COV_RES ** 2:7.1f} m2 | {r["L_h"]:7.0f} | {r["L_3d"]:6.0f} | '
              f'{r["D_up"]:6.0f}/{r["D_down"]:6.0f} | {r["E_cruise"]/1e3:5.2f}/{r["E_turn"]/1e3:4.2f}/'
              f'{r["E_climb"]/1e3:5.2f} = {r["E"]/1e3:6.2f} | {r["E_card"]/1e3:6.2f} | '
              f'{100*np.mean(e <= 1):6.1f}% | {np.percentile(e, 95):5.2f} | '
              f'{r["agl"].min():5.2f}/{r["agl"].max():5.2f} | {r["floor_viol"]:4d} | {r["ceil_viol"]:5d} | '
              f'{r["mean_pass"]:.3f} | {r["cv_pass"]:.3f} | t_fly {r["t_fly"]:.1f} s, mission '
              f'{r["t_mission"]:.1f} s, slowed steps {r["n_slowed"]}')
    for k in ('under', 'optimal', 'fixed8', 'divcos'):
        allm, unc = gap_slope_stats(ter, data['shows'][k])
        print(f'  {LABELS[k]}: mean |cross-track slope| all cells {allm:.2f} deg, uncovered cells {unc:.2f} deg')
    print(f'\n-- {N_TERRAIN} terrains (seeds 0..{N_TERRAIN-1}), mean ± std [min, max] --')
    mc = data['mc']
    rel = np.array([m['relief'] for m in mc['under']]); ms = np.array([m['maxslope'] for m in mc['under']])
    print(f'  terrain relief {rel.mean():.1f} ± {rel.std():.1f} m [{rel.min():.1f}, {rel.max():.1f}], '
          f'max slope {ms.mean():.1f} ± {ms.std():.1f} deg [{ms.min():.1f}, {ms.max():.1f}]')
    for k, *_ in CONFIGS:
        a = {f: np.array([m[f] for m in mc[k]]) for f in mc[k][0]}
        print(f'  {LABELS[k]:30s}: coverage {a["coverage"].mean():6.2f} ± {a["coverage"].std():4.2f}% '
              f'[{a["coverage"].min():6.2f}, {a["coverage"].max():6.2f}] | 100% in {np.sum(a["coverage"] >= 99.9999)}/{N_TERRAIN}'
              f' | strips {a["n"].mean():5.1f} | L3D {a["L_3d"].mean():6.0f} ± {a["L_3d"].std():4.0f} m'
              f' | E {a["E"].mean()/1e3:6.2f} ± {a["E"].std()/1e3:4.2f} kJ (climb {a["E_climb"].mean()/1e3:4.2f})'
              f' | within ±1 m {a["within1"].mean():5.1f}% [{a["within1"].min():5.1f}, {a["within1"].max():5.1f}]'
              f' | AGL min {a["agl_min"].min():5.2f} max {a["agl_max"].max():5.2f} | floor viol. {int(a["floor"].sum())}'
              f' | ceil viol. {int(a["ceil"].sum())} | mission {a["t_mission"].mean():6.1f} ± {a["t_mission"].std():4.1f} s'
              f' | slowed steps {a["n_slowed"].mean():5.1f} [{a["n_slowed"].min()}, {a["n_slowed"].max()}]')
    w1 = np.array([m['within1'] for m in mc['optimal']])
    print(f'  optimal config: AGL error within ±1 m for >= 95% of samples on {np.sum(w1 >= 95)}/{N_TERRAIN} terrains')
    dg = data['diag']
    print(f'  card constant-speed controller (optimal config, no slow-down): lowest clearance above terrain '
          f'{dg.min():.2f} m; below the 2 m floor on {np.sum(dg < AGL_FLOOR)}/{N_TERRAIN} terrains, '
          f'below ground on {np.sum(dg < 0)}/{N_TERRAIN}; per terrain: ' + ', '.join(f'{v:.1f}' for v in dg))
    cov = {k: np.array([m['coverage'] for m in mc[k]]) for k, *_ in CONFIGS}
    E = {k: np.array([m['E'] for m in mc[k]]) for k, *_ in CONFIGS}
    print(f'  head-to-head coverage: optimal > fixed8 in {np.sum(cov["optimal"] > cov["fixed8"] + 1e-9)}/{N_TERRAIN}, '
          f'equal in {np.sum(np.abs(cov["optimal"] - cov["fixed8"]) <= 1e-9)}; '
          f'optimal > divcos in {np.sum(cov["optimal"] > cov["divcos"] + 1e-9)}/{N_TERRAIN}; '
          f'optimal energy > fixed8 in {np.sum(E["optimal"] > E["fixed8"])}/{N_TERRAIN} '
          f'(mean +{(E["optimal"] - E["fixed8"]).mean()/1e3:.2f} kJ)')


if __name__ == '__main__':
    print('Running S048 3D Lawnmower Coverage on Terrain...')
    data = run_simulation()
    report(data)
    plot_trajectory_3d(data)
    plot_altitude_timeseries(data)
    plot_agl_histogram(data)
    plot_coverage_maps(data)
    plot_slope_strips(data)
    plot_comparison_bars(data)
    plot_coverage_vs_distance(data)
    save_animation(data)
    print('\nDone. Outputs in:', OUTPUT_DIR)
