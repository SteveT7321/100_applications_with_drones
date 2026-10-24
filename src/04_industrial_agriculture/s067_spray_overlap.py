"""
S067 Spray Coverage Overlap Optimization
========================================
A drone sprays a 50 x 50 m field in parallel strips with a Gaussian cross-track dose
profile (spray width 2 m, sigma_s = 0.5 m). Each strip is displaced laterally by GPS
error N(0, 0.1^2). The script

  1. computes the noise-free coefficient of variation CV(d) of the field-interior dose
     versus strip spacing d and checks whether CV has an interior minimum (it does not:
     CV falls monotonically as the strips get closer);
  2. turns the question into a well-posed design problem: the widest spacing (shortest
     flight path) whose CV stays below a uniformity target, noise-free and under GPS
     error (Monte Carlo, 200 trials, common random numbers across all d and sigma_gps);
  3. evaluates the card's nominal 10 % overlap spacing (1.8 m), the redundant (> 1.5 x
     mean) and under-dosed (< 0.5 x mean) area fractions, path length and mission time;
  4. sweeps the GPS error from RTK (0.02 m) to degraded (0.3 m).

Usage:
    conda activate drones
    python src/04_industrial_agriculture/s067_spray_overlap.py
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from scipy.optimize import minimize_scalar

# ── Parameters (card) ─────────────────────────────────────────────────────────
FIELD_W       = 50.0      # m cross-track width
FIELD_L       = 50.0      # m along-track length
SPRAY_WIDTH   = 2.0       # m effective boom spray width
SPRAY_OVERLAP = 0.10      # nominal overlap ratio
SIGMA_S       = SPRAY_WIDTH / 4.0          # 0.5 m
D_NOMINAL     = SPRAY_WIDTH * (1.0 - SPRAY_OVERLAP)   # 1.8 m
GPS_ERROR     = 0.1       # m 1-sigma lateral strip offset
GRID_RES      = 0.05      # m
V_CRUISE      = 3.0       # m/s
N_MC          = 200
REDUND_K      = 1.5       # redundant if D > 1.5 mean
SEED          = 42
SEED_OOS      = 4242      # independent draws for the out-of-sample check of d* (added)
N_OOS         = 1000
MARGIN        = SPRAY_WIDTH / 2.0          # m interior margin at each field edge

# ── Parameters (added) ───────────────────────────────────────────────────────
D_WIDE        = 2.5       # m example of a too-wide spacing (heatmaps/profiles)
D_SEARCH      = (0.25, 4.0)                # m union of the card text (0.5..4 sigma) and code (0.5..4 m)
D_GRID        = np.round(np.arange(0.25, 4.0001, 0.02), 4)   # MC spacing grid
CV_TARGET     = 0.10      # uniformity target (CV <= 10 %)
CV_TARGETS    = [0.05, 0.10, 0.15]
UNDER_K       = 0.5       # under-dosed if D < 0.5 mean
GPS_SWEEP     = [0.0, 0.02, 0.05, 0.10, 0.20, 0.30]
D_GRID_SWEEP  = np.round(np.arange(0.25, 3.0001, 0.02), 4)
KERNEL_R      = 6 * SIGMA_S + 2.0   # m dose kernel truncated 6 sigma_s from the shifted centre

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), '..', '..', 'outputs',
                          '04_industrial_agriculture', 's067_spray_overlap')

X_GRID = np.arange(GRID_RES / 2.0, FIELD_W, GRID_RES)
INTERIOR = (X_GRID >= MARGIN) & (X_GRID <= FIELD_W - MARGIN)
N_STRIPS_MAX = int(np.ceil(FIELD_W / (0.7 * D_GRID[0]))) + 1   # covers the 0.70 d* heatmap


# ── Model ─────────────────────────────────────────────────────────────────────

def n_strips(d):
    """Strips at x = 0, d, 2d, ... until one lies at or beyond the far edge."""
    return int(np.ceil(FIELD_W / d - 1e-9)) + 1


def dose_profiles(d, eps):
    """Cross-track dose for every row of eps (shape (M, >= n_strips)); returns (M, Nx)."""
    eps = np.atleast_2d(eps)
    assert np.all(np.abs(eps) < KERNEL_R - 6 * SIGMA_S), 'GPS offset exceeds kernel window'
    dose = np.zeros((eps.shape[0], X_GRID.size))
    for k in range(n_strips(d)):
        i0, i1 = np.searchsorted(X_GRID, [k * d - KERNEL_R, k * d + KERNEL_R])
        c = k * d + eps[:, k]
        dose[:, i0:i1] += np.exp(-0.5 * ((X_GRID[None, i0:i1] - c[:, None]) / SIGMA_S) ** 2)
    return dose


def stats(dose):
    """CV, redundant and under-dosed fractions over the field interior (per row)."""
    di = dose[:, INTERIOR]
    mu = di.mean(axis=1)
    cv = di.std(axis=1) / mu
    red = np.mean(di > REDUND_K * mu[:, None], axis=1)
    under = np.mean(di < UNDER_K * mu[:, None], axis=1)
    return cv, red, under


def cv_noise_free(d):
    return float(stats(dose_profiles(d, np.zeros((1, n_strips(d)))))[0][0])


def cv_fourier(d):
    """First-harmonic approximation of the noise-free comb: CV ~ sqrt(2) exp(-2 pi^2 s^2 / d^2)."""
    return np.sqrt(2.0) * np.exp(-2.0 * np.pi ** 2 * SIGMA_S ** 2 / d ** 2)


def path_length(d):
    n = n_strips(d)
    return n * FIELD_L + (n - 1) * d


def widest_spacing(d_vals, cv_vals, target):
    """Largest d such that cv <= target for every spacing up to d (linear interpolation)."""
    above = np.where(cv_vals > target)[0]
    if len(above) == 0:
        return float(d_vals[-1])
    i = above[0]
    if i == 0:
        return np.nan
    d0, d1, c0, c1 = d_vals[i - 1], d_vals[i], cv_vals[i - 1], cv_vals[i]
    return float(d0 + (target - c0) * (d1 - d0) / (c1 - c0))


def widest_spacing_nf(target):
    lo, hi = D_SEARCH[0], D_SEARCH[1]
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        if cv_noise_free(mid) <= target:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def mc_sweep(d_vals, z, sigma_gps):
    """CV / redundant / under fractions for every d and trial; z are shared N(0,1) draws."""
    cv = np.zeros((len(d_vals), z.shape[0]))
    red = np.zeros_like(cv)
    under = np.zeros_like(cv)
    for i, d in enumerate(d_vals):
        cv[i], red[i], under[i] = stats(dose_profiles(d, sigma_gps * z))
    return cv, red, under


# ── Simulation ───────────────────────────────────────────────────────────────

def run_simulation():
    rng = np.random.default_rng(SEED)
    z = rng.standard_normal((N_MC, N_STRIPS_MAX))     # common random numbers

    # 1. Noise-free CV curve and the card's argmin problem
    d_fine = np.linspace(D_SEARCH[0], D_SEARCH[1], 376)
    cv_nf = np.array([cv_noise_free(d) for d in d_fine])
    res = minimize_scalar(cv_noise_free, bounds=D_SEARCH, method='bounded')
    nf = {'d': d_fine, 'cv': cv_nf, 'argmin_grid': float(d_fine[np.argmin(cv_nf)]),
          'argmin_bounded': float(res.x), 'cv_at_argmin': float(res.fun),
          'monotone_above': bool(np.all(np.diff(cv_nf[np.argmin(cv_nf):]) > 0)),
          'max_cv_below': float(cv_nf[:np.argmin(cv_nf) + 1].max()),
          'd_star': {t: widest_spacing_nf(t) for t in CV_TARGETS}}
    st_nf = [stats(dose_profiles(d, np.zeros((1, n_strips(d))))) for d in d_fine]
    nf['red'] = np.array([s[1][0] for s in st_nf])
    nf['under'] = np.array([s[2][0] for s in st_nf])

    # 2. Monte Carlo at the card's GPS error
    cv, red, under = mc_sweep(D_GRID, z, GPS_ERROR)
    mc = {'d': D_GRID, 'cv_mean': cv.mean(axis=1), 'cv_std': cv.std(axis=1),
          'cv_p95': np.percentile(cv, 95, axis=1), 'red_mean': red.mean(axis=1),
          'under_mean': under.mean(axis=1), 'cv': cv}
    mc['argmin_mean'] = float(D_GRID[np.argmin(mc['cv_mean'])])
    mc['d_star_mean'] = {t: widest_spacing(D_GRID, mc['cv_mean'], t) for t in CV_TARGETS}
    mc['d_star_p95'] = {t: widest_spacing(D_GRID, mc['cv_p95'], t) for t in CV_TARGETS}

    # 3. GPS sweep (same z for every sigma)
    sweep = []
    for s in GPS_SWEEP:
        cvs, _, _ = mc_sweep(D_GRID_SWEEP, z, s)
        cv_nom = stats(dose_profiles(D_NOMINAL, s * z))[0]
        sweep.append({'sigma': s,
                      'd_mean': widest_spacing(D_GRID_SWEEP, cvs.mean(axis=1), CV_TARGET),
                      'd_p95': widest_spacing(D_GRID_SWEEP,
                                              np.percentile(cvs, 95, axis=1), CV_TARGET),
                      'cv_nom_mean': float(cv_nom.mean()),
                      'cv_nom_p95': float(np.percentile(cv_nom, 95))})

    # 4. Design points: card nominal and the 10 % CV design under GPS error (P95)
    d_design = mc['d_star_p95'][CV_TARGET]
    points = {}
    for name, d in [('nominal', D_NOMINAL), ('nf10', nf['d_star'][CV_TARGET]),
                    ('p95_10', d_design)]:
        c_nf, r_nf, u_nf = stats(dose_profiles(d, np.zeros((1, n_strips(d)))))
        c, r, u = stats(dose_profiles(d, GPS_ERROR * z))
        points[name] = dict(d=d, n=n_strips(d), path=path_length(d),
                            time=path_length(d) / V_CRUISE, cv_nf=float(c_nf[0]),
                            red_nf=float(r_nf[0]), under_nf=float(u_nf[0]),
                            cv_mean=float(c.mean()), cv_std=float(c.std()),
                            cv_p95=float(np.percentile(c, 95)), red_mean=float(r.mean()),
                            under_mean=float(u.mean()),
                            pass_rate=float(np.mean(c <= CV_TARGET)))
    # Out-of-sample check of d*: d* was chosen on the draws z, so its in-sample pass rate is
    # 95 % by construction; re-test it on independent draws
    z_oos = np.random.default_rng(SEED_OOS).standard_normal((N_OOS, N_STRIPS_MAX))
    c_oos = stats(dose_profiles(d_design, GPS_ERROR * z_oos))[0]
    points['p95_10']['oos'] = dict(pass_rate=float(np.mean(c_oos <= CV_TARGET)),
                                   cv_mean=float(c_oos.mean()), cv_std=float(c_oos.std()),
                                   cv_p95=float(np.percentile(c_oos, 95)))
    return z, nf, mc, sweep, points


# ── Plotting ──────────────────────────────────────────────────────────────────

def plot_cv_curve(nf, mc, points, out_dir):
    fig, ax = plt.subplots(figsize=(10, 6))
    ax.plot(nf['d'], 100 * nf['cv'], 'k-', lw=1.8, label='Noise-free CV')
    ax.plot(nf['d'], 100 * cv_fourier(nf['d']), 'k:', lw=1.2,
            label=r'First-harmonic approx. $\sqrt{2}\,e^{-2\pi^2\sigma_s^2/d^2}$')
    ax.fill_between(mc['d'], 100 * (mc['cv_mean'] - mc['cv_std']),
                    100 * (mc['cv_mean'] + mc['cv_std']), color='steelblue', alpha=0.3,
                    label=rf'MC mean $\pm$ 1$\sigma$ ($\sigma_{{gps}}$ = {GPS_ERROR} m)')
    ax.plot(mc['d'], 100 * mc['cv_mean'], 'b--', lw=1.4, label='MC mean CV')
    ax.plot(mc['d'], 100 * mc['cv_p95'], color='navy', lw=1.0, label='MC 95th percentile CV')
    ax.axhline(100 * CV_TARGET, color='gray', ls='--', lw=1, label='Target CV = 10 %')
    ax.axvline(nf['d_star'][CV_TARGET], color='red', lw=1.5,
               label=f"Widest d, noise-free = {nf['d_star'][CV_TARGET]:.3f} m")
    ax.axvline(points['p95_10']['d'], color='orange', lw=1.5, ls='--',
               label=f"Widest d, MC P95 = {points['p95_10']['d']:.3f} m")
    ax.axvline(D_NOMINAL, color='green', lw=1.2, ls=':',
               label=f'Card nominal (10 % overlap) = {D_NOMINAL:.2f} m')
    ax.set_xlim(D_SEARCH)
    ax.set_ylim(0, 80)
    ax.set_xlabel('Strip spacing d (m)')
    ax.set_ylabel(r'CV = $\sigma_D/\mu_D$ (%) over the field interior')
    ax.set_title('S067 Coverage uniformity vs strip spacing (CV rises monotonically with d)')
    ax.legend(fontsize=8, loc='upper left')
    ax.grid(alpha=0.3)
    path = os.path.join(out_dir, 'cv_vs_spacing.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def design_spacings(points):
    return [(points['p95_10']['d'], 'GPS-robust design d* (P95 CV <= 10 %)', '#1F77B4'),
            (points['nf10']['d'], 'Noise-free 10 % design', '#2CA02C'),
            (D_NOMINAL, 'Card nominal (10 % overlap)', '#9467BD'),
            (D_WIDE, 'Wide', '#D62728')]


def plot_heatmaps(z, points, out_dir):
    sp = design_spacings(points)
    ds = [d for d, _, _ in sp]
    names = [n for _, n, _ in sp]
    fig, axes = plt.subplots(1, 4, figsize=(20, 6))
    y = np.arange(GRID_RES / 2.0, FIELD_L, GRID_RES)
    for ax, d, nm in zip(axes, ds, names):
        dose = dose_profiles(d, GPS_ERROR * z[:1])[0]
        mu = dose[INTERIOR].mean()
        img = np.tile(dose / mu, (len(y), 1))
        im = ax.imshow(img, origin='lower', extent=[0, FIELD_W, 0, FIELD_L], cmap='YlOrRd',
                       vmin=0, vmax=2.0, aspect='equal')
        for k in range(n_strips(d)):
            ax.axvline(k * d + GPS_ERROR * z[0, k], color='cyan', lw=0.6, ls='--', alpha=0.8)
        cv, red, under = stats(dose[None, :])
        ax.set_title(f'{nm}\nd = {d:.2f} m: CV {100 * cv[0]:.1f}%, >1.5x mean '
                     f'{100 * red[0]:.1f}%, <0.5x mean {100 * under[0]:.1f}%', fontsize=9)
        ax.set_xlim(0, 15)
        ax.set_xlabel('Cross-track x (m)')
        ax.set_ylabel('Along-track y (m)')
        fig.colorbar(im, ax=ax, label='Dose / interior mean', fraction=0.046)
    fig.suptitle(f'S067 Dose maps, one GPS realisation (sigma_gps = {GPS_ERROR} m), first 15 m '
                 'of the field; dashed cyan = strip centrelines; stats over the whole interior',
                 fontsize=12)
    plt.tight_layout()
    path = os.path.join(out_dir, 'dose_heatmaps.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_profiles(points, out_dir):
    fig, ax = plt.subplots(figsize=(11, 5))
    for d, lab, col in design_spacings(points):
        dose = dose_profiles(d, np.zeros((1, n_strips(d))))[0]
        ax.plot(X_GRID, dose / dose[INTERIOR].mean(), color=col, lw=1.3,
                label=f'{lab} (d = {d:.2f} m)')
    ax.axhline(1.0, color='k', ls='--', lw=0.8, label='Mean')
    ax.axhline(REDUND_K, color='gray', ls=':', lw=1, label='Redundancy threshold 1.5')
    ax.axhline(UNDER_K, color='gray', ls='-.', lw=1, label='Under-dose threshold 0.5')
    for xm in (MARGIN, FIELD_W - MARGIN):
        ax.axvline(xm, color='0.5', lw=0.8)
    ax.set_xlim(0, 12)
    ax.set_xlabel('Cross-track position x (m) — first 12 m shown; grey line = interior margin')
    ax.set_ylabel(r'$D(x)/\mu_D$')
    ax.set_title('S067 Noise-free cross-track dose profiles')
    ax.legend(fontsize=8, loc='upper right', ncol=2)
    ax.grid(alpha=0.3)
    path = os.path.join(out_dir, 'dose_profiles.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_redundancy(nf, mc, points, out_dir):
    fig, ax = plt.subplots(figsize=(10, 5.5))
    ax.plot(nf['d'], 100 * nf['red'], 'r-', lw=1.5, label='> 1.5x mean, noise-free')
    ax.plot(mc['d'], 100 * mc['red_mean'], 'r--', lw=1.2, label='> 1.5x mean, MC mean')
    ax.plot(nf['d'], 100 * nf['under'], color='purple', lw=1.5, label='< 0.5x mean, noise-free')
    ax.plot(mc['d'], 100 * mc['under_mean'], color='purple', ls='--', lw=1.2,
            label='< 0.5x mean, MC mean')
    ax.set_xlabel('Strip spacing d (m)')
    ax.set_ylabel('Interior area fraction (%)')
    ax.set_xlim(D_SEARCH)
    ax2 = ax.twinx()
    dd = nf['d']
    ax2.plot(dd, [path_length(d) for d in dd], 'b-.', lw=1.2, label='Path length (m)')
    ax2.set_ylabel('Total path length (m)', color='b')
    ax2.set_yscale('log')
    ax.axvline(points['p95_10']['d'], color='orange', ls='--', lw=1.2,
               label=f"Design d* = {points['p95_10']['d']:.3f} m")
    ax.axvline(D_NOMINAL, color='green', ls=':', lw=1.2, label='Card nominal 1.8 m')
    h1, l1 = ax.get_legend_handles_labels()
    h2, l2 = ax2.get_legend_handles_labels()
    ax.legend(h1 + h2, l1 + l2, fontsize=8, loc='upper left')
    ax.grid(alpha=0.3)
    ax.set_title('S067 Redundant / under-dosed area and path length vs d')
    path = os.path.join(out_dir, 'redundancy_pathlength.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_gps_sweep(sweep, out_dir):
    s = np.array([r['sigma'] for r in sweep])
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(13, 4.8))
    a1.plot(s, [r['d_mean'] for r in sweep], 'o-', label='mean CV <= 10 %')
    a1.plot(s, [r['d_p95'] for r in sweep], 's-', label='95th-percentile CV <= 10 %')
    a1.set_xlabel(r'GPS error $\sigma_{gps}$ (m)')
    a1.set_ylabel('Widest strip spacing (m)')
    a1.set_title('Widest spacing meeting CV <= 10 %')
    a2.plot(s, [100 * r['cv_nom_mean'] for r in sweep], 'o-', label='mean')
    a2.plot(s, [100 * r['cv_nom_p95'] for r in sweep], 's-', label='95th percentile')
    a2.set_xlabel(r'GPS error $\sigma_{gps}$ (m)')
    a2.set_ylabel('CV (%)')
    a2.set_title(f'CV at the card nominal spacing d = {D_NOMINAL} m')
    for a in (a1, a2):
        a.grid(alpha=0.3)
        a.legend()
    fig.suptitle(f'S067 GPS error sweep ({N_MC} trials per point, same random draws)')
    plt.tight_layout()
    path = os.path.join(out_dir, 'gps_sweep.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def save_animation(z, d, out_dir):
    n = n_strips(d)
    eps = GPS_ERROR * z[0]
    y = np.arange(GRID_RES / 2.0, FIELD_L, GRID_RES)
    single = [np.exp(-0.5 * ((X_GRID - (k * d + eps[k])) / SIGMA_S) ** 2) for k in range(n)]
    full = np.sum(single, axis=0)
    vmax = 2.0 * full[INTERIOR].mean()
    fig, ax = plt.subplots(figsize=(6.5, 6))
    im = ax.imshow(np.zeros((len(y), len(X_GRID))), origin='lower',
                   extent=[0, FIELD_W, 0, FIELD_L], cmap='YlOrRd', vmin=0, vmax=vmax)
    fig.colorbar(im, ax=ax, label='Accumulated dose (peak strip dose = 1)', fraction=0.046)
    line, = ax.plot([], [], 'c--', lw=1.2)
    drone, = ax.plot([], [], 'ko', ms=6)
    ax.set_xlim(0, FIELD_W)
    ax.set_ylim(0, FIELD_L)
    ax.set_xlabel('Cross-track x (m)')
    ax.set_ylabel('Along-track y (m)')
    n_frames = n + 1

    def update(f):
        acc = np.sum(single[:f], axis=0) if f > 0 else np.zeros_like(X_GRID)
        im.set_data(np.tile(acc, (len(y), 1)))
        if f > 0:
            c = (f - 1) * d + eps[f - 1]
            line.set_data([c, c], [0, FIELD_L])
            drone.set_data([c], [FIELD_L if (f - 1) % 2 == 0 else 0.0])
            cv = acc[INTERIOR].std() / acc[INTERIOR].mean()
            ax.set_title(f'S067 strip {f}/{n}, d = {d:.3f} m\ninterior CV so far {100 * cv:.1f}%')
        else:
            line.set_data([], [])
            drone.set_data([], [])
            ax.set_title(f'S067 strip 0/{n}, d = {d:.3f} m')
        return [im, line, drone]

    anim = animation.FuncAnimation(fig, update, frames=n_frames, interval=200, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    anim.save(path, writer='pillow', fps=5)
    plt.close(fig)
    print(f'Saved: {path} ({n_frames} frames)')


# ── Main ─────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    z, nf, mc, sweep, points = run_simulation()
    print('=== S067 Spray Coverage Overlap ===')
    print(f'Interior: x in [{MARGIN:.1f}, {FIELD_W - MARGIN:.1f}] m, {INTERIOR.sum()} grid cells')
    print(f'Noise-free CV strictly increasing for d >= grid argmin up to {D_SEARCH[1]} m: '
          f'{nf["monotone_above"]}; largest CV for d <= argmin: '
          f'{100 * nf["max_cv_below"]:.3f} %')
    print(f'Card argmin: grid argmin d = {nf["argmin_grid"]:.3f} m, bounded minimize_scalar '
          f'd = {nf["argmin_bounded"]:.4f} m (CV = {100 * nf["cv_at_argmin"]:.2e} %)')
    print(f'MC mean CV argmin over d grid [{D_GRID[0]}, {D_GRID[-1]}]: {mc["argmin_mean"]:.2f} m')
    for d in [1.0, 1.5, 1.8, 2.0, 3.0]:
        print(f'  CV noise-free at d = {d:.1f} m: {100 * cv_noise_free(d):.2f} % '
              f'(first-harmonic approx {100 * cv_fourier(d):.2f} %)')
    for t in CV_TARGETS:
        print(f'Widest d with CV <= {100 * t:.0f} %: noise-free {nf["d_star"][t]:.3f} m, '
              f'MC mean {mc["d_star_mean"][t]:.3f} m, MC P95 {mc["d_star_p95"][t]:.3f} m')
    for name, lab in [('nominal', 'Card nominal 1.8 m'), ('nf10', 'Noise-free 10 % design'),
                      ('p95_10', 'MC-P95 10 % design (d*)')]:
        p = points[name]
        print(f'[{lab}] d = {p["d"]:.3f} m, overlap {100 * (1 - p["d"] / SPRAY_WIDTH):.1f} %, '
              f'N = {p["n"]}, path {p["path"]:.1f} m, time {p["time"]:.1f} s '
              f'({p["time"] / 60:.2f} min)')
        print(f'    noise-free: CV {100 * p["cv_nf"]:.2f} %, >1.5x {100 * p["red_nf"]:.2f} %, '
              f'<0.5x {100 * p["under_nf"]:.2f} %')
        print(f'    GPS MC:     CV {100 * p["cv_mean"]:.2f} ± {100 * p["cv_std"]:.2f} % '
              f'(P95 {100 * p["cv_p95"]:.2f} %), >1.5x {100 * p["red_mean"]:.2f} %, '
              f'<0.5x {100 * p["under_mean"]:.2f} %, trials with CV <= 10 %: '
              f'{100 * p["pass_rate"]:.1f} %')
    pn, pd = points['nominal'], points['p95_10']
    o = points['p95_10']['oos']
    print(f'd* = {points["p95_10"]["d"]:.3f} m: in-sample pass rate (CV <= 10 %) '
          f'{100 * points["p95_10"]["pass_rate"]:.1f} % is 95 % by construction; out-of-sample '
          f'({N_OOS} independent trials, seed {SEED_OOS}): pass rate {100 * o["pass_rate"]:.1f} %, '
          f'CV {100 * o["cv_mean"]:.2f} ± {100 * o["cv_std"]:.2f} % (P95 {100 * o["cv_p95"]:.2f} %)')
    i_red = int(np.argmax(nf['red'] > 0))
    print(f'Noise-free redundant area (> 1.5x mean) is 0 for d < {nf["d"][i_red]:.2f} m '
          f'(first non-zero value {100 * nf["red"][i_red]:.2f} % at d = {nf["d"][i_red]:.2f} m)')
    print(f'Design d* vs nominal: path +{100 * (pd["path"] / pn["path"] - 1):.1f} %, '
          f'time +{pd["time"] - pn["time"]:.1f} s')
    print('GPS sweep (sigma -> widest d mean / P95, CV at 1.8 m mean / P95):')
    for r in sweep:
        print(f'    {r["sigma"]:.2f} m -> {r["d_mean"]:.3f} / {r["d_p95"]:.3f} m, '
              f'{100 * r["cv_nom_mean"]:.2f} / {100 * r["cv_nom_p95"]:.2f} %')
    print(f'    (nan = target not met even at the smallest spacing {D_GRID_SWEEP[0]} m)')
    print('MC CV vs d at sigma_gps = 0.1 m (mean / P95, noise-free):')
    for d in [0.25, 0.35, 0.5, 0.65, 1.0, 1.37, 1.81, D_WIDE]:
        i1 = int(np.argmin(np.abs(D_GRID - d)))
        print(f'    d = {D_GRID[i1]:.2f} m: {100 * mc["cv_mean"][i1]:.2f} / '
              f'{100 * mc["cv_p95"][i1]:.2f} %, noise-free {100 * cv_noise_free(D_GRID[i1]):.3f} %')

    out = os.path.normpath(OUTPUT_DIR)
    os.makedirs(out, exist_ok=True)
    d_star = points['p95_10']['d']
    plot_cv_curve(nf, mc, points, out)
    plot_heatmaps(z, points, out)
    plot_profiles(points, out)
    plot_redundancy(nf, mc, points, out)
    plot_gps_sweep(sweep, out)
    save_animation(z, d_star, out)
