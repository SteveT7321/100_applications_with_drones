"""
S073 Solar Panel Thermal Imaging Inspection
===========================================
A drone flies a boustrophedon at 5 m and 1 m/s over an 80 x 60 m solar farm (10 x 12
panels) with a nadir thermal camera (45 deg FOV -> 4.14 m footprint). Eight panels carry
Gaussian hot spots 10-20 degC above the 35 degC nominal. A panel is flagged when the
reading exceeds nominal + 8 degC and the motion blur is below 2.5 px.

The card's nominal case is reported first. Because detection is certain there (every
metric is fixed by construction), two studies make the card's questions discriminating:

  1. Plan comparison (50 seeds, common random numbers): the card's 19-strip lawnmower,
     an edge-aligned 18-strip lawnmower and a 10-strip plan centred on the panel rows,
     with hot spots at the panel centre (card) or anywhere on the panel.
  2. Speed sweep with wind-induced speed fluctuations (sigma_v = 0.1 m/s): detection rate
     and false-negative probability around the blur limit v_max = 1.52 m/s.

Usage:
    conda activate drones
    python src/04_industrial_agriculture/s073_solar_thermal.py
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation
from scipy.stats import norm
from scipy.signal import lfilter

# ── Parameters (card) ─────────────────────────────────────────────────────────
FARM_L, FARM_W   = 80.0, 60.0     # m, along-track (x) / cross-track (y)
N_ROWS, N_COLS   = 10, 12
M_DEFECTIVE      = 8
ALTITUDE         = 5.0            # m
FOV_DEG          = 45.0
OVERLAP_RATIO    = 0.20
V_NOMINAL        = 1.0            # m/s
GPS_ERROR        = 0.1            # m
DT               = 0.1            # s, one thermal frame per timestep
T_NOMINAL        = 35.0           # degC
DT_HOT_MIN, DT_HOT_MAX = 10.0, 20.0
DT_THRESH        = 8.0            # degC above nominal
SIGMA_HOT        = 0.4            # m
T_NOISE_STD      = 0.5            # degC
T_EXPOSURE       = 0.033          # s
GSD              = 0.02           # m/pixel
BLUR_MAX         = 2.5            # pixels
SIGMA_V          = 0.1            # m/s, speed fluctuation std (card P_FN model)
DEFECT_SEED      = 42             # card: assign_defects seed
NOISE_SEED       = 0              # card: run_simulation seed

# ── Added parameters (see README Deviations) ──────────────────────────────────
TAU_V            = 2.0            # s, correlation time of the speed fluctuation
N_MC             = 50
MC_SEED0         = 3000
SPEEDS           = [0.5, 0.75, 1.0, 1.25, 1.4, 1.5, 1.6, 1.75, 2.0, 2.5]
ANIM_STEP        = 120            # timesteps per GIF frame
TAU_V_SWEEP      = [0.2, 2.0, 20.0, 200.0, np.inf]   # s; inf = one random speed per mission
V_TAU_PROBE      = 1.5            # m/s mean speed of the correlation-time sweep

# ── Derived ───────────────────────────────────────────────────────────────────
W_FOOT   = 2.0 * ALTITUDE * np.tan(np.deg2rad(FOV_DEG) / 2.0)   # 4.142 m
STRIP_D  = W_FOOT * (1.0 - OVERLAP_RATIO)                         # 3.314 m
PANEL_L  = FARM_L / N_COLS                                        # 6.667 m
PANEL_W  = FARM_W / N_ROWS                                        # 6.0 m
V_MAX    = BLUR_MAX * GSD / T_EXPOSURE                            # 1.515 m/s
ATTEN    = np.exp(-GPS_ERROR ** 2 / (2 * SIGMA_HOT ** 2))         # card pointing loss

PLANS = ['card', 'edge', 'row']
PLAN_LABELS = {'card': 'Card lawnmower', 'edge': 'Edge-aligned lawnmower',
               'row': 'Row-centred'}
PLAN_COLORS = {'card': 'tab:red', 'edge': 'tab:blue', 'row': 'tab:olive'}
MODELS = ['centre', 'anywhere']

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '04_industrial_agriculture', 's073_solar_thermal',
)


# ── Geometry ──────────────────────────────────────────────────────────────────
def build_panel_grid():
    cx = (np.arange(N_COLS) + 0.5) * PANEL_L
    cy = (np.arange(N_ROWS) + 0.5) * PANEL_W
    X, Y = np.meshgrid(cx, cy)
    return np.stack([X.ravel(), Y.ravel()], axis=1)       # row-major: row, col


def strip_centres(plan):
    if plan == 'card':
        n = int(np.ceil(FARM_W / STRIP_D))
        return (np.arange(n) + 0.5) * STRIP_D
    if plan == 'edge':
        n = int(np.ceil((FARM_W - W_FOOT) / STRIP_D - 1e-9)) + 1
        return np.minimum(W_FOOT / 2 + np.arange(n) * STRIP_D, FARM_W - W_FOOT / 2)
    return (np.arange(N_ROWS) + 0.5) * PANEL_W            # 'row'


def waypoints(plan):
    wps = []
    for i, y in enumerate(strip_centres(plan)):
        wps += [(0.0, y), (FARM_L, y)] if i % 2 == 0 else [(FARM_L, y), (0.0, y)]
    return np.array(wps)


def path_length(wps):
    return float(np.sum(np.linalg.norm(np.diff(wps, axis=0), axis=1)))


def fly(wps, speeds):
    """Advance along the polyline with per-frame speed speeds[k]; returns positions,
    speed per frame and strip index per frame."""
    seg = np.diff(wps, axis=0)
    seg_len = np.linalg.norm(seg, axis=1)
    cum = np.concatenate([[0.0], np.cumsum(seg_len)])
    s = np.concatenate([[0.0], np.cumsum(speeds * DT)])
    n = int(np.searchsorted(s, cum[-1])) + 1
    s = np.minimum(s[:n], cum[-1])
    idx = np.clip(np.searchsorted(cum, s, side='right') - 1, 0, len(seg) - 1)
    xy = wps[idx] + ((s - cum[idx]) / seg_len[idx])[:, None] * seg[idx]
    v = np.concatenate([speeds[:n - 1], [speeds[n - 2]]])
    return xy, v, idx // 2


# ── Randomness (one stream per seed -> common random numbers) ─────────────────
def draw_case(seed, panels, model, defect_seed=None):
    rng = np.random.default_rng(seed if defect_seed is None else defect_seed)
    defect_idx = rng.choice(len(panels), size=M_DEFECTIVE, replace=False)
    delta_t = rng.uniform(DT_HOT_MIN, DT_HOT_MAX, size=M_DEFECTIVE)
    off = rng.uniform(-1, 1, (M_DEFECTIVE, 2)) * \
        np.array([PANEL_L / 2 - SIGMA_HOT, PANEL_W / 2 - SIGMA_HOT])
    hot = panels[defect_idx] + (off if model == 'anywhere' else 0.0)
    return defect_idx, delta_t, hot


def speed_series(seed, v0, n, fluctuate=True, tau=TAU_V):
    rng = np.random.default_rng(seed + 7919)
    if not fluctuate:
        return np.full(n, v0)
    if np.isinf(tau):                       # one random speed for the whole mission
        return np.full(n, max(v0 + SIGMA_V * rng.normal(0.0, 1.0), 0.05))
    phi = np.exp(-DT / tau)
    xi = rng.normal(0.0, 1.0, n)
    q = SIGMA_V * np.sqrt(1 - phi ** 2)
    # dv[0] = sigma xi[0]; dv[k] = phi dv[k-1] + q xi[k]  (stationary Gauss-Markov)
    dv = lfilter([q], [1.0, -phi], xi)
    dv += phi ** np.arange(n) * (SIGMA_V - q) * xi[0]
    return np.maximum(v0 + dv, 0.05)


def blur_radius(speed):
    return speed * T_EXPOSURE / GSD


# ── One inspection ────────────────────────────────────────────────────────────
def inspect(plan, panels, defect_idx, delta_t, hot, seed, v0=V_NOMINAL, fluctuate=False,
            check_fa=True, tau=TAU_V):
    wps = waypoints(plan)
    v_floor = max(v0 - 8 * SIGMA_V, 0.05) if fluctuate else v0
    n_max = int(np.ceil(path_length(wps) / (v_floor * DT))) + 10
    speeds = speed_series(seed, v0, n_max, fluctuate, tau)
    xy, v, strip = fly(wps, speeds)
    n = len(xy)
    rng = np.random.default_rng(seed + 104729)
    blur_ok = blur_radius(v) < BLUR_MAX
    # hot spots: in footprint if inside the square w_foot x w_foot image
    in_fp = ((np.abs(xy[:, None, 0] - hot[None, :, 0]) <= W_FOOT / 2) &
             (np.abs(xy[:, None, 1] - hot[None, :, 1]) <= W_FOOT / 2))
    t_meas = T_NOMINAL + delta_t[None, :] * ATTEN + rng.normal(0, T_NOISE_STD, in_fp.shape)
    hit = in_fp & blur_ok[:, None] & (t_meas > T_NOMINAL + DT_THRESH)
    detected = hit.any(axis=0)
    t_det = np.where(detected, hit.argmax(axis=0), -1)
    frames_in = in_fp.sum(axis=0)
    blurred_in = (in_fp & ~blur_ok[:, None]).sum(axis=0)
    # healthy panels: flagged if a frame with the panel centre in view reads above threshold
    fa_flags = np.zeros(len(panels), dtype=bool)
    max_healthy = -np.inf
    if check_fa:
        healthy = np.setdiff1d(np.arange(len(panels)), defect_idx)
        in_h = ((np.abs(xy[:, None, 0] - panels[None, healthy, 0]) <= W_FOOT / 2) &
                (np.abs(xy[:, None, 1] - panels[None, healthy, 1]) <= W_FOOT / 2))
        noise_h = rng.normal(0, T_NOISE_STD, in_h.shape)
        t_h = np.where(in_h & blur_ok[:, None], T_NOMINAL + noise_h, -np.inf)
        max_healthy = t_h.max()
        fa_flags[healthy] = (t_h > T_NOMINAL + DT_THRESH).any(axis=0)
    # physical invariants
    step = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    assert np.all(step <= v[:-1] * DT + 1e-9), 'moved faster than commanded'
    assert np.all((xy >= -1e-9).all(axis=1) & (xy[:, 0] <= FARM_L + 1e-9)), 'left the farm'
    return dict(plan=plan, xy=xy, v=v, strip=strip, detected=detected, t_det=t_det,
                frames_in=frames_in, blurred_in=blurred_in, fa=fa_flags,
                max_healthy=max_healthy, t_mission=(n - 1) * DT, path=path_length(wps),
                det_rate=detected.mean(), blur_frac=1 - blur_ok.mean(),
                n_strips=len(strip_centres(plan)))


def area_coverage(plan):
    """Exact imaged fraction of the farm: length of the union of the strips'
    cross-track intervals [y - w/2, y + w/2] clipped to [0, W], divided by W."""
    iv = sorted((max(y - W_FOOT / 2, 0.0), min(y + W_FOOT / 2, FARM_W))
                for y in strip_centres(plan))
    total, cur_lo, cur_hi = 0.0, iv[0][0], iv[0][1]
    for lo, hi in iv[1:]:
        if lo > cur_hi:
            total += cur_hi - cur_lo
            cur_lo, cur_hi = lo, hi
        else:
            cur_hi = max(cur_hi, hi)
    total += cur_hi - cur_lo
    return total / FARM_W


def build_thermal_map(panels, defect_idx, delta_t, hot, nx=800, ny=600):
    xs = np.linspace(0, FARM_L, nx)
    ys = np.linspace(0, FARM_W, ny)
    XX, YY = np.meshgrid(xs, ys)
    T = np.full((ny, nx), T_NOMINAL)
    for k in range(len(defect_idx)):
        T += delta_t[k] * np.exp(-((XX - hot[k, 0]) ** 2 + (YY - hot[k, 1]) ** 2)
                                 / (2 * SIGMA_HOT ** 2))
    return T, xs, ys


def run_simulation():
    panels = build_panel_grid()
    # Nominal (card seeds, constant 1 m/s, hot spots at panel centres)
    d_idx, d_t, hot = draw_case(None, panels, 'centre', defect_seed=DEFECT_SEED)
    nominal = inspect('card', panels, d_idx, d_t, hot, NOISE_SEED)
    T_map, xs, ys = build_thermal_map(panels, d_idx, d_t, hot)
    # Plan comparison (constant nominal speed), CRN per seed
    plan_mc = {m: {p: [] for p in PLANS} for m in MODELS}
    for m in MODELS:
        for k in range(N_MC):
            seed = MC_SEED0 + k
            di, dt_, ho = draw_case(seed, panels, m)
            for p in PLANS:
                r = inspect(p, panels, di, dt_, ho, seed)
                plan_mc[m][p].append(dict(det=r['det_rate'], fa=int(r['fa'].sum()),
                                          t=r['t_mission'], maxh=r['max_healthy']))
    # Speed sweep with speed fluctuations (card plan, centre model)
    sweep = {}
    for v0 in SPEEDS:
        res = []
        for k in range(N_MC):
            seed = MC_SEED0 + k
            di, dt_, ho = draw_case(seed, panels, 'centre')
            r = inspect('card', panels, di, dt_, ho, seed, v0=v0, fluctuate=True,
                        check_fa=False)
            res.append(dict(det=r['det_rate'], blur=r['blur_frac'], t=r['t_mission'],
                            n_missed=int((~r['detected']).sum()),
                            n_all_blur=int(((r['blurred_in'] == r['frames_in']) &
                                            (r['frames_in'] > 0)).sum())))
        sweep[v0] = res
    # Correlation time of the speed fluctuation at v0 = 1.5 m/s (card plan, CRN per seed)
    tau_sweep = {}
    for tau in TAU_V_SWEEP:
        res = []
        for k in range(N_MC):
            seed = MC_SEED0 + k
            di, dt_, ho = draw_case(seed, panels, 'centre')
            r = inspect('card', panels, di, dt_, ho, seed, v0=V_TAU_PROBE, fluctuate=True,
                        check_fa=False, tau=tau)
            res.append(dict(det=r['det_rate'], blur=r['blur_frac'],
                            n_missed=int((~r['detected']).sum())))
        tau_sweep[tau] = res
    return dict(panels=panels, defect_idx=d_idx, delta_t=d_t, hot=hot, nominal=nominal,
                T_map=T_map, xs=xs, ys=ys, plan_mc=plan_mc, sweep=sweep, tau_sweep=tau_sweep)


# ── Plots ─────────────────────────────────────────────────────────────────────
def _panel_grid(ax, color='grey', lw=0.5):
    for c in range(N_COLS + 1):
        ax.axvline(c * PANEL_L, color=color, lw=lw)
    for r in range(N_ROWS + 1):
        ax.axhline(r * PANEL_W, color=color, lw=lw)


def plot_thermal_map(res, out_dir):
    nom = res['nominal']
    fig, ax = plt.subplots(figsize=(11, 8.5))
    ax.set_facecolor('dimgray')
    im = ax.imshow(res['T_map'], origin='lower', extent=[0, FARM_L, 0, FARM_W], cmap='hot',
                   vmin=T_NOMINAL - 2, vmax=T_NOMINAL + 20)
    _panel_grid(ax)
    for i, y in enumerate(strip_centres('card')):
        for yb in (y - W_FOOT / 2, y + W_FOOT / 2):
            ax.plot([0, FARM_L], [yb, yb], color='yellow', lw=0.6, alpha=0.6,
                    label='footprint edges' if (i == 0 and yb < y) else None)
    ax.plot(nom['xy'][:, 0], nom['xy'][:, 1], color='white', lw=0.7, label='flight path')
    h = res['hot']
    ax.plot(h[:, 0], h[:, 1], 'x', color='magenta', ms=12, mew=2, label='defective panel')
    plt.colorbar(im, ax=ax, label='Temperature (°C)', shrink=0.8)
    ax.set_xlim(-1, FARM_L + 1)
    ax.set_ylim(-1, 63)
    ax.axhline(FARM_W, color='cyan', ls='--', lw=1, label='farm edge (y = 60 m)')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title(f'Thermal map and card lawnmower ({nom["n_strips"]} strips, '
                 f'spacing {STRIP_D:.2f} m, footprint {W_FOOT:.2f} m; grey = outside the farm)')
    ax.legend(loc='upper left', bbox_to_anchor=(0, -0.08), ncol=4, fontsize=8)
    path = os.path.join(out_dir, 'thermal_map.png')
    plt.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_detection_overlay(res, out_dir):
    nom = res['nominal']
    panels = res['panels']
    fig, ax = plt.subplots(figsize=(11, 8))
    d_set = set(res['defect_idx'].tolist())
    tp = fn = 0
    for i, (px, py) in enumerate(panels):
        if i in d_set:
            k = int(np.where(res['defect_idx'] == i)[0][0])
            if nom['detected'][k]:
                c, tp = 'red', tp + 1
            else:
                c, fn = 'orange', fn + 1
        else:
            c = 'yellow' if nom['fa'][i] else 'lightgrey'
        ax.add_patch(patches.Rectangle((px - PANEL_L / 2 + 0.2, py - PANEL_W / 2 + 0.2),
                                       PANEL_L - 0.4, PANEL_W - 0.4, fc=c, ec='grey'))
    ax.plot(nom['xy'][:, 0], nom['xy'][:, 1], 'k-', lw=0.6)
    handles = [patches.Patch(fc='red', label=f'detected hot panel (TP = {tp})'),
               patches.Patch(fc='orange', label=f'missed hot panel (FN = {fn})'),
               patches.Patch(fc='yellow', ec='grey', label=f"false alarm (FA = {int(nom['fa'].sum())})"),
               patches.Patch(fc='lightgrey', ec='grey', label='healthy')]
    ax.legend(handles=handles, loc='upper left', bbox_to_anchor=(1.01, 1), fontsize=8)
    ax.set_xlim(-1, FARM_L + 1)
    ax.set_ylim(-1, 63)
    ax.set_aspect('equal')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title('Detection result of the nominal mission (card seeds)')
    path = os.path.join(out_dir, 'detection_overlay.png')
    plt.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_blur_curve(out_dir):
    v = np.linspace(0.3, 2.5, 200)
    b = blur_radius(v)
    fig, ax = plt.subplots(figsize=(8, 5))
    ax.plot(v, b, 'k-', lw=2, label=r'$b = v\,t_{exp}/GSD$')
    ax.axhline(BLUR_MAX, color='red', ls='--', label=f'b_max = {BLUR_MAX} px')
    ax.axvline(V_NOMINAL, color='tab:blue', ls='--', label='v = 1.0 m/s (nominal)')
    ax.axvline(V_MAX, color='red', ls=':', label=f'v_max = {V_MAX:.3f} m/s')
    ax.fill_between(v, BLUR_MAX, b.max() * 1.05, where=b >= BLUR_MAX, color='red', alpha=0.1)
    ax.text(1.9, BLUR_MAX + 0.6, 'Detection suppressed', color='red')
    ax.plot(V_NOMINAL, blur_radius(V_NOMINAL), 'o', color='tab:blue', ms=9)
    ax.set_xlabel('drone speed v (m/s)')
    ax.set_ylabel('blur radius b (px)')
    ax.set_title('Motion blur vs speed')
    ax.legend(fontsize=8, loc='upper left')
    ax.grid(alpha=0.3)
    path = os.path.join(out_dir, 'blur_vs_speed.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_plan_comparison(res, out_dir):
    pm = res['plan_mc']
    fig, axes = plt.subplots(1, 2, figsize=(13, 4.8))
    x = np.arange(len(PLANS))
    w = 0.35
    for j, m in enumerate(MODELS):
        means = [100 * np.mean([r['det'] for r in pm[m][p]]) for p in PLANS]
        sds = [100 * np.std([r['det'] for r in pm[m][p]]) for p in PLANS]
        bars = axes[0].bar(x + (j - 0.5) * w, means, w, yerr=sds, capsize=4,
                           color=[PLAN_COLORS[p] for p in PLANS], alpha=0.85 if j == 0 else 0.45,
                           edgecolor='k', label=f'hot spot at panel {"centre (card)" if m == "centre" else "anywhere"}')
        for b_, mv in zip(bars, means):
            axes[0].text(b_.get_x() + b_.get_width() / 2, mv + 2, f'{mv:.1f}', ha='center',
                         fontsize=8)
    axes[0].set_xticks(x)
    axes[0].set_xticklabels([PLAN_LABELS[p] for p in PLANS])
    axes[0].set_ylabel('detection rate (%), mean ± std')
    axes[0].set_ylim(0, 115)
    axes[0].set_title(f'Detection rate, {N_MC} seeds (CRN)')
    axes[0].legend(fontsize=8, loc='lower left')
    t = [pm['centre'][p][0]['t'] for p in PLANS]
    axes[1].bar(x, t, color=[PLAN_COLORS[p] for p in PLANS], edgecolor='k')
    for i, tv in enumerate(t):
        axes[1].text(i, tv + 20, f'{tv:.0f} s', ha='center')
    axes[1].set_xticks(x)
    axes[1].set_xticklabels([PLAN_LABELS[p] for p in PLANS])
    axes[1].set_ylabel('coverage time (s)')
    axes[1].set_ylim(0, max(t) * 1.15)
    axes[1].set_title('Coverage time at 1.0 m/s')
    plt.tight_layout()
    path = os.path.join(out_dir, 'plan_comparison.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_speed_sweep(res, out_dir):
    sw = res['sweep']
    v = np.array(SPEEDS)
    det = np.array([np.mean([r['det'] for r in sw[s]]) for s in SPEEDS])
    det_sd = np.array([np.std([r['det'] for r in sw[s]]) for s in SPEEDS])
    blur = np.array([np.mean([r['blur'] for r in sw[s]]) for s in SPEEDS])
    t = np.array([np.mean([r['t'] for r in sw[s]]) for s in SPEEDS])
    vv = np.linspace(0.4, 2.6, 200)
    fig, axes = plt.subplots(1, 2, figsize=(13, 4.8))
    ax = axes[0]
    ax.errorbar(v, 100 * det, yerr=100 * det_sd, fmt='o-', color='tab:green', capsize=3,
                label='detection rate (MC)')
    ax.plot(v, 100 * blur, 's', color='tab:red', label='blurred frames (MC)')
    ax.plot(vv, 100 * norm.sf((V_MAX - vv) / SIGMA_V), 'r-', lw=1,
            label=r'card $P_{FN} = 1-\Phi((v_{max}-v_0)/\sigma_v)$')
    ax.axvline(V_MAX, color='red', ls=':', lw=1)
    ax.set_xlabel('mean speed v0 (m/s)')
    ax.set_ylabel('%')
    ax.set_title(f'Detection vs speed, sigma_v = {SIGMA_V} m/s, tau = {TAU_V:.0f} s '
                 f'({N_MC} seeds)')
    ax.legend(fontsize=8, loc='center left')
    ax.grid(alpha=0.3)
    ax = axes[1]
    ax.plot(v, t, 'o-', color='k')
    ax.set_xlabel('mean speed v0 (m/s)')
    ax.set_ylabel('coverage time (s)')
    ax.set_title('Coverage time (card plan)')
    ax.axvline(V_MAX, color='red', ls=':', lw=1)
    ax.grid(alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'speed_sweep.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_tau_sweep(res, out_dir):
    ts = res['tau_sweep']
    labels = [('constant\nper mission' if np.isinf(t) else f'{t:g} s') for t in TAU_V_SWEEP]
    miss = [100 * sum(r['n_missed'] for r in ts[t]) / (N_MC * M_DEFECTIVE) for t in TAU_V_SWEEP]
    blur = [100 * np.mean([r['blur'] for r in ts[t]]) for t in TAU_V_SWEEP]
    x = np.arange(len(TAU_V_SWEEP))
    fig, ax = plt.subplots(figsize=(9, 4.8))
    ax.bar(x - 0.2, miss, 0.4, color='tab:orange', edgecolor='k', label='hot spots missed (MC)')
    ax.bar(x + 0.2, blur, 0.4, color='tab:red', alpha=0.6, edgecolor='k',
           label='blurred frames (MC)')
    pfn = 100 * norm.sf((V_MAX - V_TAU_PROBE) / SIGMA_V)
    ax.axhline(pfn, color='k', ls='--', lw=1, label=f'card P_FN = {pfn:.1f}%')
    for i, (mv, bv) in enumerate(zip(miss, blur)):
        ax.text(i - 0.2, mv + 1, f'{mv:.1f}', ha='center', fontsize=8)
        ax.text(i + 0.2, bv + 1, f'{bv:.1f}', ha='center', fontsize=8)
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.set_xlabel('correlation time of the speed fluctuation')
    ax.set_ylabel('%')
    ax.set_ylim(0, 60)
    ax.set_title(f'Panel misses vs speed-fluctuation correlation time '
                 f'(v0 = {V_TAU_PROBE} m/s, sigma_v = {SIGMA_V} m/s, {N_MC} seeds)')
    ax.legend(fontsize=8, loc='upper left')
    path = os.path.join(out_dir, 'tau_v_sensitivity.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def save_animation(res, out_dir):
    nom = res['nominal']
    xy = nom['xy']
    n = len(xy)
    n_frames = int(np.ceil((n - 1) / ANIM_STEP)) + 1
    fig, ax = plt.subplots(figsize=(8, 6.3))
    ax.imshow(res['T_map'][::4, ::4], origin='lower', extent=[0, FARM_L, 0, FARM_W],
              cmap='hot', vmin=T_NOMINAL - 2, vmax=T_NOMINAL + 20)
    _panel_grid(ax, lw=0.3)
    trail, = ax.plot([], [], color='white', lw=0.6)
    dot, = ax.plot([], [], 'o', color='white', mec='k', ms=7)
    fp = patches.Rectangle((0, 0), W_FOOT, W_FOOT, fill=False, ec='cyan', lw=1.2)
    ax.add_patch(fp)
    hot = res['hot']
    marks = [ax.add_patch(patches.Rectangle((res['panels'][i][0] - PANEL_L / 2,
                                             res['panels'][i][1] - PANEL_W / 2),
                                            PANEL_L, PANEL_W, fill=False, ec='red', lw=2.5,
                                            visible=False))
             for i in res['defect_idx']]
    ax.set_xlim(-1, FARM_L + 1)
    ax.set_ylim(-1, 63)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')

    def update(f):
        k = min(f * ANIM_STEP, n - 1)
        trail.set_data(xy[:k + 1, 0], xy[:k + 1, 1])
        dot.set_data([xy[k, 0]], [xy[k, 1]])
        fp.set_xy((xy[k, 0] - W_FOOT / 2, xy[k, 1] - W_FOOT / 2))
        nd = 0
        for j, mk in enumerate(marks):
            on = 0 <= nom['t_det'][j] <= k
            mk.set_visible(on)
            nd += int(on)
        ax.set_title(f't = {k * DT:6.1f} s | strip {nom["strip"][k] + 1}/{nom["n_strips"]} | '
                     f'hot panels detected {nd}/{M_DEFECTIVE}', fontsize=10)
        return [trail, dot, fp] + marks

    anim = animation.FuncAnimation(fig, update, frames=n_frames, interval=100, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    anim.save(path, writer='pillow', fps=10, dpi=70)
    plt.close(fig)
    print(f'Saved: {path} ({n_frames} frames)')


if __name__ == '__main__':
    res = run_simulation()
    nom = res['nominal']
    print('\n=== S073 Solar Panel Thermal Inspection — Results ===')
    print(f'  Footprint w_foot = {W_FOOT:.3f} m, strip spacing d = {STRIP_D:.3f} m, '
          f'panel {PANEL_L:.3f} x {PANEL_W:.3f} m')
    print(f'  v_max = b_max GSD / t_exp = {V_MAX:.3f} m/s; blur at 1.0 m/s = '
          f'{blur_radius(V_NOMINAL):.3f} px (limit {BLUR_MAX} px)')
    print(f'  Card P_FN at v0 = 1.0, sigma_v = {SIGMA_V}: 1 - Phi({(V_MAX - V_NOMINAL) / SIGMA_V:.2f})'
          f' = {norm.sf((V_MAX - V_NOMINAL) / SIGMA_V):.2e}')
    print(f'  Pointing attenuation exp(-GPS^2/2 sigma_hot^2) = {ATTEN:.4f}; weakest possible '
          f'reading margin = ({DT_HOT_MIN} x {ATTEN:.4f} - {DT_THRESH}) / {T_NOISE_STD} = '
          f'{(DT_HOT_MIN * ATTEN - DT_THRESH) / T_NOISE_STD:.2f} sigma; healthy margin '
          f'{DT_THRESH / T_NOISE_STD:.0f} sigma')
    for p in PLANS:
        yc = strip_centres(p)
        cov = area_coverage(p)
        print(f'  Plan {PLAN_LABELS[p]:<24} {len(yc):2d} strips, y = {yc[0]:.2f} .. {yc[-1]:.2f} m, '
              f'path {path_length(waypoints(p)):7.1f} m, area covered {100 * cov:.1f}%')
    print('  [Nominal mission: card plan, card seeds, constant 1.0 m/s, hot spots at panel centres]')
    print(f"    Defective panels: {sorted(res['defect_idx'].tolist())}, dT = "
          + ', '.join(f'{x:.1f}' for x in res['delta_t']))
    print(f"    Detected {int(nom['detected'].sum())}/{M_DEFECTIVE} "
          f"(rate {100 * nom['det_rate']:.1f}%), false alarms {int(nom['fa'].sum())}, "
          f"false negatives {M_DEFECTIVE - int(nom['detected'].sum())}")
    print(f"    Frames with each hot spot in view: min {nom['frames_in'].min()}, "
          f"max {nom['frames_in'].max()}; hottest healthy reading {nom['max_healthy']:.2f} degC")
    print(f"    Coverage time {nom['t_mission']:.1f} s ({nom['t_mission'] / 60:.2f} min), "
          f"path {nom['path']:.1f} m, blurred frames {100 * nom['blur_frac']:.1f}%")
    print(f'  [Plan comparison: {N_MC} seeds, CRN, constant 1.0 m/s]')
    pm = res['plan_mc']
    for m in MODELS:
        for p in PLANS:
            d = np.array([r['det'] for r in pm[m][p]])
            fa = sum(r['fa'] for r in pm[m][p])
            print(f"    hot spot {m:<8} {PLAN_LABELS[p]:<24} detection {100 * d.mean():5.1f} ± "
                  f"{100 * d.std():4.1f}%, seeds with 8/8: {int((d == 1).sum()):2d}/{N_MC}, "
                  f"false alarms {fa}, time {pm[m][p][0]['t']:.1f} s")
        dr = np.array([r['det'] for r in pm[m]['row']])
        dc = np.array([r['det'] for r in pm[m]['card']])
        print(f'    hot spot {m}: row-centred vs card, paired: row better {int((dr > dc).sum())}, '
              f'card better {int((dc > dr).sum())}, ties {int((dr == dc).sum())}')
    tc, te, tr = (pm['centre'][p][0]['t'] for p in PLANS)
    print(f'  Time saved vs card plan: edge-aligned {tc - te:.1f} s ({100 * (1 - te / tc):.1f}%), '
          f'row-centred {tc - tr:.1f} s ({100 * (1 - tr / tc):.1f}%)')
    print(f'  [Speed sweep: card plan, hot spots at centres, speed fluctuation sigma_v = '
          f'{SIGMA_V} m/s, tau = {TAU_V} s, {N_MC} seeds]')
    print('     v0   P_FN card   blurred frames   detection mean ± std   seeds 8/8   '
          'missed (all frames blurred)   time')
    for v0 in SPEEDS:
        sw = res['sweep'][v0]
        d = np.array([r['det'] for r in sw])
        print(f"    {v0:4.2f}   {norm.sf((V_MAX - v0) / SIGMA_V):8.2e}   "
              f"{100 * np.mean([r['blur'] for r in sw]):6.2f}%        "
              f"{100 * d.mean():5.1f} ± {100 * d.std():4.1f}%        {int((d == 1).sum()):2d}/{N_MC}"
              f"        {sum(r['n_missed'] for r in sw):3d} ({sum(r['n_all_blur'] for r in sw):3d}) "
              f"= {100 * sum(r['n_missed'] for r in sw) / (N_MC * M_DEFECTIVE):5.2f}%"
              f"            {np.mean([r['t'] for r in sw]):7.1f} s")

    print(f'  [Speed-fluctuation correlation time: card plan, v0 = {V_TAU_PROBE} m/s, '
          f'sigma_v = {SIGMA_V} m/s, {N_MC} seeds, card P_FN = '
          f'{100 * norm.sf((V_MAX - V_TAU_PROBE) / SIGMA_V):.1f}%]')
    print('    tau_v          blurred frames   hot spots missed   detection mean ± std')
    for tau in TAU_V_SWEEP:
        ts = res['tau_sweep'][tau]
        d = np.array([r['det'] for r in ts])
        nm = sum(r['n_missed'] for r in ts)
        lab = 'per mission' if np.isinf(tau) else f'{tau:g} s'
        print(f"    {lab:<13}  {100 * np.mean([r['blur'] for r in ts]):6.2f}%          "
              f"{nm:3d} = {100 * nm / (N_MC * M_DEFECTIVE):5.2f}%      "
              f"{100 * d.mean():5.1f} ± {100 * d.std():4.1f}%")
    out = os.path.normpath(OUTPUT_DIR)
    os.makedirs(out, exist_ok=True)
    plot_thermal_map(res, out)
    plot_detection_overlay(res, out)
    plot_blur_curve(out)
    plot_plan_comparison(res, out)
    plot_speed_sweep(res, out)
    plot_tau_sweep(res, out)
    save_animation(res, out)
