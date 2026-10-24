"""
S059 Underwater Target Sonar Marking (Surface Cooperative Localization)
========================================================================
Four drones drop passive sonar buoys on the sea surface (z = 0). A silent underwater
target pings every 5 s; each buoy time-stamps the ping with its own clock noise
(sigma_t = 0.1 ms), and the three TDOAs relative to buoy 1 are inverted with Gauss-Newton
to a 3D position. Buoy layouts compared:

  1. Square grid (card, half-side 100 m)
  2. Tetrahedral projection (card, R = 120 m, three on a circle + one at the centre)
  3. CRLB-optimal at the true target (card; needs the answer before the first ping)
  4. CRLB-optimal for a prior search volume (added; deployable before the first ping)
  5./6. Square and tetrahedral layouts scaled to the 500 m spread limit (added, fairness)

The TDOA noise is modelled per buoy, so the three TDOAs share buoy 1's clock error and
their covariance is sigma_t^2 c^2 (I + 1 1^T), not the card's 2 sigma_t^2 c^2 I.
All layouts see the same clock-noise draws (common random numbers).

Usage:
    conda activate drones
    python src/03_environmental_sar/s059_sonar_relay.py
"""

import os
for _v in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[_v] = '1'          # single-threaded BLAS: reproducible and fast on a shared machine
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.colors import LogNorm
from mpl_toolkits.mplot3d import Axes3D              # noqa: F401
from scipy.optimize import minimize

# ── Parameters (card) ─────────────────────────────────────────────────────────
N_BUOYS     = 4
C_SOUND     = 1500.0                 # m/s
SIGMA_T     = 1e-4                   # s — per-buoy clock noise
SIGMA_D     = C_SOUND * np.sqrt(2) * SIGMA_T   # m — std of one TDOA in range units (0.212 m)
T_PING      = 5.0                    # s
EPSILON_TOL = 1e-3                   # m — GN step tolerance
K_MAX       = 200                    # GN iteration cap
D_MAX       = 500.0                  # m — max pairwise buoy spread
BOX         = 250.0                  # m — operating box +/- BOX in x and y
N_TRIALS    = 1000                   # Monte Carlo trials at the true target
N_RESTARTS  = 20                     # optimiser restarts
REG         = 1e-6                   # FIM regularisation
P_TARGET    = np.array([40.0, -30.0, -120.0])
Z_RANGE     = (-200.0, -10.0)        # target depth range
SQUARE_HALF = 100.0
TETRA_R     = 120.0

# ── Parameters (added) ────────────────────────────────────────────────────────
PRIOR_XY    = 100.0                  # m — search area x, y in [-100, 100] (prior on the target)
N_PRIOR_MC  = 300                    # random prior targets for the layout comparison
P_INIT      = np.array([0.0, 0.0, -105.0])   # GN start: centre of the search volume
RES_TOL     = 1e-2                   # m — residual of a valid fix (the system is square)
PLAUSIBLE   = 1000.0                 # m — fixes beyond 1 km horizontally or deep are rejected
N_PRIOR_OPT = 400                    # random prior targets used by the prior-layout optimiser
PRIOR_OPT_SEED = 11
OPT_SEED    = 0
NOISE_SEED  = 42
PRIOR_SEED  = 7
COLLINEAR   = np.array([[-150.0, 0, 0], [-50.0, 0, 0], [50.0, 0, 0], [150.0, 0, 0]])

LAYOUT_ORDER = ['square', 'tetra', 'crlb_oracle', 'crlb_prior', 'square_big', 'tetra_big']
LABELS = {'square': 'Square grid (card)', 'tetra': 'Tetrahedral (card)',
          'crlb_oracle': 'CRLB-optimal at true target', 'crlb_prior': 'CRLB-optimal for prior',
          'square_big': 'Square, 500 m spread', 'tetra_big': 'Tetrahedral, box-limited'}
COLORS = {'square': '#7F7F7F', 'tetra': '#FF7F0E', 'crlb_oracle': '#D62728',
          'crlb_prior': '#1F77B4', 'square_big': '#BCBD22', 'tetra_big': '#9467BD'}

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '03_environmental_sar', 's059_sonar_relay',
)

# TDOA covariance in range units: d_i1 = c (eps_i - eps_1)
SIGMA_TDOA = (C_SOUND * SIGMA_T) ** 2 * (np.eye(N_BUOYS - 1) + np.ones((N_BUOYS - 1, N_BUOYS - 1)))
SIGMA_CARD = SIGMA_D ** 2 * np.eye(N_BUOYS - 1)


# ── Layouts ───────────────────────────────────────────────────────────────────

def layout_square(half):
    s = half
    return np.array([[s, s, 0.0], [-s, s, 0.0], [-s, -s, 0.0], [s, -s, 0.0]])


def layout_tetrahedral(radius):
    """Card order: three on the circle first (buoy 1 = reference at 0 deg), then the centre."""
    ang = np.deg2rad([0.0, 120.0, 240.0])
    ring = np.stack([radius * np.cos(ang), radius * np.sin(ang), np.zeros(3)], axis=1)
    return np.vstack([ring, [[0.0, 0.0, 0.0]]])


def check_layout(b):
    assert np.allclose(b[:, 2], 0.0), 'buoys must float at z = 0'
    assert np.all(np.abs(b[:, :2]) <= BOX + 1e-6), 'buoy outside the operating box'
    d = np.linalg.norm(b[:, None, :2] - b[None, :, :2], axis=-1)
    assert d.max() <= D_MAX + 1e-3, f'buoy spread {d.max():.1f} m > D_max'
    return d.max()


# ── TDOA model ────────────────────────────────────────────────────────────────

def jacobian(p, buoys):
    """H_i = u_1 - u_i (unit vectors from p toward the buoys); works for p (..., 3)."""
    diff = buoys - p[..., None, :]
    u = diff / np.linalg.norm(diff, axis=-1, keepdims=True)
    return u[..., :1, :] - u[..., 1:, :]


def crlb(buoys, p, sigma=SIGMA_TDOA):
    """CRLB covariance (H^T Sigma^-1 H + reg)^-1 for target(s) p (..., 3)."""
    H = jacobian(p, buoys)
    Si = np.linalg.inv(sigma)
    F = np.swapaxes(H, -1, -2) @ Si @ H
    return np.linalg.inv(F + REG * np.eye(3))


def crlb_rms(buoys, p, sigma=SIGMA_TDOA):
    return np.sqrt(np.trace(crlb(buoys, p, sigma), axis1=-2, axis2=-1))


def gdop(buoys, p):
    H = jacobian(p, buoys)
    return np.sqrt(np.trace(np.linalg.inv(np.swapaxes(H, -1, -2) @ H + REG * np.eye(3)),
                            axis1=-2, axis2=-1))


def range_diffs(buoys, p):
    r = np.linalg.norm(buoys - p, axis=1)
    return r[1:] - r[0]


def gauss_newton(buoys, dd_meas, p_init=P_INIT):
    """Card GN on f(p) = dd_meas - (r_i - r_1); least-squares step for robustness.

    Returns the estimate, iteration count, a validity flag and the per-iteration
    (step size, position) trace. A fix is valid if the step fell below EPSILON_TOL, the
    residual is below RES_TOL and the estimate lies inside the plausible volume.
    """
    p = p_init.copy()
    steps, path = [], [p.copy()]
    converged = False
    for k in range(K_MAX):
        f = dd_meas - range_diffs(buoys, p)
        H = jacobian(p, buoys)
        # H = u_1 - u_i = d(r_i - r_1)/dp = -df/dp, so the GN step is +(H^T H)^-1 H^T f
        delta = np.linalg.lstsq(H, f, rcond=None)[0]
        p = p + delta
        steps.append(np.linalg.norm(delta))
        path.append(p.copy())
        if steps[-1] < EPSILON_TOL:
            converged = True
            break
    if p[2] > 0:          # surface-mirror solution: identical TDOAs, target must be underwater
        p = p * np.array([1, 1, -1])
    res = np.linalg.norm(dd_meas - range_diffs(buoys, p))
    valid = (converged and res < RES_TOL and np.linalg.norm(p[:2]) <= PLAUSIBLE
             and p[2] >= -PLAUSIBLE)
    return p, k + 1, bool(valid), (steps, path)


def measure(buoys, p, eps):
    """Range differences from per-buoy clock errors eps (N_BUOYS,) in seconds."""
    return range_diffs(buoys, p) + C_SOUND * (eps[1:] - eps[0])


# ── Layout optimisation ───────────────────────────────────────────────────────

def prior_sample(n, rng):
    """Targets uniform in the search volume [-100, 100]^2 x [-200, -10] m."""
    return np.column_stack([rng.uniform(-PRIOR_XY, PRIOR_XY, n),
                            rng.uniform(-PRIOR_XY, PRIOR_XY, n),
                            rng.uniform(*Z_RANGE, n)])


def optimise_layout(targets, label):
    """min mean sqrt(tr CRLB) over targets; SLSQP with pairwise-spread and box constraints."""
    def unpack(v):
        b = np.zeros((N_BUOYS, 3))
        b[:, :2] = v.reshape(N_BUOYS, 2)
        return b

    def obj(v):
        b = unpack(v)
        if len(targets) == 1:
            return float(np.trace(crlb(b, targets[0])))           # card: tr(F^-1)
        return float(np.mean(crlb_rms(b, targets)))

    pairs = [(i, j) for i in range(N_BUOYS) for j in range(i + 1, N_BUOYS)]
    cons = [{'type': 'ineq',
             'fun': (lambda v, i=i, j=j: D_MAX ** 2 - np.sum((v.reshape(-1, 2)[i]
                                                              - v.reshape(-1, 2)[j]) ** 2))}
            for i, j in pairs]
    rng = np.random.default_rng(OPT_SEED)
    best = None
    vals = []
    for _ in range(N_RESTARTS):
        x0 = rng.uniform(-D_MAX / 2 / np.sqrt(2), D_MAX / 2 / np.sqrt(2), size=2 * N_BUOYS)
        res = minimize(obj, x0, method='SLSQP', bounds=[(-BOX, BOX)] * (2 * N_BUOYS),
                       constraints=cons, options={'maxiter': 400, 'ftol': 1e-12})
        b = unpack(res.x)
        feas = np.all(np.linalg.norm(b[:, None, :2] - b[None, :, :2], axis=-1) <= D_MAX + 1e-3)
        vals.append(res.fun if feas else np.inf)
        if feas and (best is None or res.fun < best[1]):
            best = (b, res.fun)
    vals = np.array(vals)
    print(f'  [{label}] {N_RESTARTS} restarts: best {best[1]:.5g}, median {np.median(vals):.5g}, '
          f'{int(np.sum(vals <= best[1] * 1.001))} restarts within 0.1 % of best')
    return best[0]


# ── Simulation ────────────────────────────────────────────────────────────────

def monte_carlo(buoys, targets, eps_all):
    """One GN fix per (target, clock-noise draw); targets (n, 3) or a single (3,) target."""
    targets = np.broadcast_to(targets, (len(eps_all), 3))
    errs = np.zeros((len(eps_all), 3))
    iters = np.zeros(len(eps_all), int)
    valid = np.zeros(len(eps_all), bool)
    for k, eps in enumerate(eps_all):
        p, it, ok, _ = gauss_newton(buoys, measure(buoys, targets[k], eps))
        errs[k] = p - targets[k]
        iters[k] = it
        valid[k] = ok
    return errs, iters, valid


def run_simulation():
    print('Optimising layouts ...')
    opt_targets = prior_sample(N_PRIOR_OPT, np.random.default_rng(PRIOR_OPT_SEED))
    layouts = {
        'square': layout_square(SQUARE_HALF),
        'tetra': layout_tetrahedral(TETRA_R),
        'crlb_oracle': optimise_layout(P_TARGET[None, :], 'CRLB at true target'),
        'crlb_prior': optimise_layout(opt_targets, 'CRLB for prior'),
        'square_big': layout_square(D_MAX / 2 / np.sqrt(2)),
        'tetra_big': layout_tetrahedral(BOX),
    }
    spreads = {k: check_layout(b) for k, b in layouts.items()}

    rng = np.random.default_rng(NOISE_SEED)
    eps_true = rng.normal(0.0, SIGMA_T, size=(N_TRIALS, N_BUOYS))   # CRN: shared by all layouts
    rng_p = np.random.default_rng(PRIOR_SEED)
    prior_targets = prior_sample(N_PRIOR_MC, rng_p)                # independent test targets
    eps_prior = rng_p.normal(0.0, SIGMA_T, size=(N_PRIOR_MC, N_BUOYS))
    assert np.all(prior_targets[:, 2] < 0) and P_TARGET[2] < 0

    results = {}
    for name in LAYOUT_ORDER:
        b = layouts[name]
        errs, iters, valid = monte_carlo(b, P_TARGET, eps_true)
        e3 = np.linalg.norm(errs, axis=1)
        C = crlb(b, P_TARGET)
        C_card = crlb(b, P_TARGET, SIGMA_CARD)
        perr, _, pvalid = monte_carlo(b, prior_targets, eps_prior)
        pe = np.linalg.norm(perr, axis=1)
        ev = e3[valid]
        results[name] = {
            'buoys': b, 'spread': spreads[name], 'errs': errs, 'iters': iters, 'valid': valid,
            'crlb': C, 'crlb_rms': np.sqrt(np.trace(C)), 'crlb_card_rms': np.sqrt(np.trace(C_card)),
            'gdop': float(gdop(b, P_TARGET)),
            'mc_rmse': np.sqrt(np.mean(ev ** 2)), 'mc_p95': np.percentile(ev, 95),
            'mc_median': np.median(e3),
            'mc_cov': np.cov(errs[valid].T), 'mc_bias': errs[valid].mean(axis=0),
            'prior_err': pe, 'prior_valid': pvalid,
            'prior_crlb': crlb_rms(b, prior_targets),
        }
    # GN traces: CRLB-oracle layout vs collinear layout, 10 noise draws
    traces = {'good': [], 'collinear': []}
    for k in range(10):
        for key, b in (('good', layouts['crlb_oracle']), ('collinear', COLLINEAR)):
            _, _, ok, (steps, path) = gauss_newton(b, measure(b, P_TARGET, eps_true[k]))
            errs = [np.linalg.norm(q * np.array([1, 1, -1 if q[2] > 0 else 1]) - P_TARGET)
                    for q in path]
            traces[key].append((steps, errs, ok))
    return layouts, results, prior_targets, traces


# ── Plots ─────────────────────────────────────────────────────────────────────

def _save(fig, out_dir, name):
    path = os.path.join(out_dir, name)
    fig.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_geometry_3d(results, out_dir):
    names = ['square', 'tetra', 'crlb_oracle', 'crlb_prior']
    fig = plt.figure(figsize=(18, 5.2))
    xs = np.linspace(-260, 260, 261)
    X, Y = np.meshgrid(xs, xs)
    P = np.stack([X, Y, np.full_like(X, P_TARGET[2])], axis=-1)
    for i, n in enumerate(names):
        ax = fig.add_subplot(1, 4, i + 1, projection='3d')
        b = results[n]['buoys']
        dd = range_diffs(b, P_TARGET)
        r = np.linalg.norm(P[..., None, :] - b, axis=-1)
        for j in range(1, N_BUOYS):
            F = (r[..., j] - r[..., 0]) - dd[j - 1]
            cs = plt.figure().add_subplot().contour(X, Y, F, levels=[0])
            plt.close(cs.axes.figure)
            for seg in cs.allsegs[0]:
                ax.plot(seg[:, 0], seg[:, 1], np.full(len(seg), P_TARGET[2]), color='#888888',
                        lw=0.9)
        ax.scatter(b[:, 0], b[:, 1], b[:, 2], color='cyan', s=60, edgecolors='k', depthshade=False)
        for k, bb in enumerate(b):
            ax.plot(*zip(bb, P_TARGET), 'k--', lw=0.7)
            ax.text(bb[0], bb[1], bb[2] + 12, f'B{k + 1}', fontsize=7)
        ax.scatter(*P_TARGET, color='green', marker='*', s=220, edgecolors='k', depthshade=False)
        ax.set_xlim(-260, 260); ax.set_ylim(-260, 260); ax.set_zlim(-200, 0)
        ax.set_xlabel('x (m)', fontsize=8); ax.set_ylabel('y (m)', fontsize=8)
        ax.set_zlabel('z (m)', fontsize=8)
        ax.set_title(f"{LABELS[n]}\nCRLB RMS {100 * results[n]['crlb_rms']:.1f} cm", fontsize=9)
        ax.view_init(elev=20, azim=-60)
    fig.suptitle('S059 buoy layouts (cyan, z = 0) and target (green star). Grey: the three '
                 'TDOA hyperboloids cut at z = -120 m; they meet at the target.')
    _save(fig, out_dir, 'geometry_3d.png')


def plot_crlb_heatmaps(results, out_dir):
    names = ['square', 'tetra', 'crlb_oracle', 'crlb_prior']
    xs = np.linspace(-250, 250, 201)
    X, Y = np.meshgrid(xs, xs)
    P = np.stack([X, Y, np.full_like(X, P_TARGET[2])], axis=-1)
    fig, axs = plt.subplots(1, 4, figsize=(19, 4.6))
    for ax, n in zip(axs, names):
        b = results[n]['buoys']
        Z = 100 * crlb_rms(b, P)
        im = ax.pcolormesh(X, Y, Z, cmap='Blues_r', shading='auto', norm=LogNorm(vmin=20, vmax=5000))
        ax.contour(X, Y, Z, levels=[25, 50, 100, 1000], colors='k', linewidths=0.6)
        ax.plot(b[:, 0], b[:, 1], 'o', color='cyan', mec='k', ms=7)
        ax.plot(P_TARGET[0], P_TARGET[1], 'x', color='green', ms=10, mew=2.5)
        ax.add_patch(plt.Rectangle((-PRIOR_XY, -PRIOR_XY), 2 * PRIOR_XY, 2 * PRIOR_XY,
                                   fill=False, ls=':', ec='k'))
        ax.set_aspect('equal')
        ax.set_title(LABELS[n], fontsize=10)
        ax.set_xlabel('target x (m)')
    axs[0].set_ylabel('target y (m)')
    fig.colorbar(im, ax=axs, label=r'$\sqrt{\mathrm{tr}\,F^{-1}}$ (cm, log scale)')
    fig.suptitle('S059 CRLB RMS error for targets at z = -120 m (contours 25 / 50 / 100 / 1000 cm; '
                 'dotted: prior search area; x: true target)')
    _save(fig, out_dir, 'crlb_heatmaps.png')


def plot_gn_convergence(traces, out_dir):
    fig, axs = plt.subplots(1, 2, figsize=(13, 4.8))
    for key, col, lab in (('good', '#D62728', 'CRLB-optimal layout (at true target)'),
                          ('collinear', '#7F7F7F', 'collinear buoys (x = -150 ... 150 m)')):
        for i, (s, e, _) in enumerate(traces[key]):
            axs[0].semilogy(np.arange(1, len(s) + 1), np.maximum(s, 1e-12), '-o', color=col,
                            ms=3, lw=0.8, alpha=0.8, label=lab if i == 0 else None)
            axs[1].semilogy(np.arange(len(e)), e, '-o', color=col, ms=3, lw=0.8, alpha=0.8,
                            label=lab if i == 0 else None)
    axs[0].axhline(EPSILON_TOL, color='k', ls='--', lw=1, label=r'$\epsilon_{tol}$ = 1 mm')
    axs[0].set_ylabel(r'step $\|\Delta p^{(k)}\|$ (m)')
    axs[1].set_ylabel(r'position error $\|p^{(k)} - p_T\|$ (m)')
    for ax in axs:
        ax.set_xlabel('Gauss-Newton iteration k')
        ax.legend(fontsize=8)
        ax.grid(alpha=0.3, which='both')
    fig.suptitle('S059 Gauss-Newton convergence, 10 noise realisations per layout '
                 f'(start {tuple(float(v) for v in P_INIT)} m)')
    _save(fig, out_dir, 'gn_convergence.png')


def plot_error_ellipsoids(results, out_dir):
    names = ['square', 'tetra', 'crlb_oracle', 'crlb_prior']
    fig = plt.figure(figsize=(18, 5))
    u, v = np.meshgrid(np.linspace(0, 2 * np.pi, 40), np.linspace(0, np.pi, 20))
    sphere = np.stack([np.cos(u) * np.sin(v), np.sin(u) * np.sin(v), np.cos(v)], axis=-1)
    for i, n in enumerate(names):
        ax = fig.add_subplot(1, 4, i + 1, projection='3d')
        C = results[n]['crlb'] * 1e4                       # cm^2
        w, V = np.linalg.eigh(C)
        E = sphere @ (V * np.sqrt(w)).T
        ax.plot_surface(E[..., 0], E[..., 1], E[..., 2], color=COLORS[n], alpha=0.35, lw=0)
        e = 100 * results[n]['errs'][:300]
        lim = 3.0 * 100 * np.sqrt(np.max(np.linalg.eigvalsh(results[n]['crlb'])))
        e = e[np.all(np.abs(e) < lim, axis=1)]
        ax.scatter(e[:, 0], e[:, 1], e[:, 2], s=2, color='k', alpha=0.4)
        sd = np.sqrt(np.diag(C))
        ax.set_xlim(-lim, lim); ax.set_ylim(-lim, lim); ax.set_zlim(-lim, lim)
        ax.set_xlabel('dx (cm)', fontsize=8); ax.set_ylabel('dy (cm)', fontsize=8)
        ax.set_zlabel('dz (cm)', fontsize=8)
        ax.set_title(f'{LABELS[n]}\n1-sigma: x {sd[0]:.1f}, y {sd[1]:.1f}, z {sd[2]:.1f} cm',
                     fontsize=9)
    fig.suptitle('S059 CRLB 1-sigma error ellipsoids at the true target (dots: 300 Gauss-Newton '
                 'errors within the axes; each panel scaled to 3 x its largest 1-sigma)')
    _save(fig, out_dir, 'error_ellipsoids.png')


def plot_rmse_bars(results, out_dir):
    fig, axs = plt.subplots(1, 2, figsize=(15, 5))
    x = np.arange(len(LAYOUT_ORDER))
    ax = axs[0]
    rm = [100 * results[n]['mc_rmse'] for n in LAYOUT_ORDER]
    p95 = [100 * results[n]['mc_p95'] for n in LAYOUT_ORDER]
    ax.bar(x - 0.2, rm, 0.4, color=[COLORS[n] for n in LAYOUT_ORDER], label='MC 3D RMSE')
    ax.bar(x + 0.2, p95, 0.4, color=[COLORS[n] for n in LAYOUT_ORDER], alpha=0.45,
           hatch='//', label='MC 95th percentile')
    for i, n in enumerate(LAYOUT_ORDER):
        ax.plot([i - 0.38, i - 0.02], [100 * results[n]['crlb_rms']] * 2, 'k--', lw=1.5,
                label='CRLB RMS (correlated TDOA noise)' if i == 0 else None)
        ax.plot([i - 0.38, i - 0.02], [100 * results[n]['crlb_card_rms']] * 2, 'r:', lw=1.5,
                label=r"card's CRLB ($\Sigma = 2\sigma_t^2 c^2 I$)" if i == 0 else None)
    ax.set_xticks(x, [LABELS[n].replace(' (card)', '\n(card)').replace(' at ', '\nat ')
                      .replace(' for ', '\nfor ').replace(', ', ',\n') for n in LAYOUT_ORDER],
                  fontsize=7)
    ax.set_ylabel('3D position error (cm, log)')
    ax.set_yscale('log')
    ax.set_title(f'True target, {N_TRIALS} trials (same clock noise for every layout)')
    ax.legend(fontsize=8)
    ax = axs[1]
    data = [100 * results[n]['prior_err'] for n in LAYOUT_ORDER]
    bp = ax.boxplot(data, patch_artist=True, widths=0.5, showfliers=False)
    for patch, n in zip(bp['boxes'], LAYOUT_ORDER):
        patch.set_facecolor(COLORS[n]); patch.set_alpha(0.6)
    ax.set_xticks(x + 1, [LABELS[n].replace(' (card)', '\n(card)').replace(' at ', '\nat ')
                          .replace(' for ', '\nfor ').replace(', ', ',\n') for n in LAYOUT_ORDER],
                  fontsize=7)
    ax.set_yscale('log')
    ax.set_ylabel('3D position error (cm, log)')
    ax.set_title(f'{N_PRIOR_MC} random targets in the prior volume (one ping each; '
                 'whiskers 1.5 IQR, all fixes)')
    fig.suptitle('S059 localisation accuracy by buoy layout')
    _save(fig, out_dir, 'rmse_bars.png')


def plot_error_histogram(results, out_dir):
    r = results['crlb_oracle']
    e = 100 * r['errs'][r['valid']]
    C = r['crlb'] * 1e4
    fig, axs = plt.subplots(1, 2, figsize=(12, 4.5))
    h = np.linalg.norm(e[:, :2], axis=1)
    rng = np.random.default_rng(1)
    pred = rng.multivariate_normal(np.zeros(2), C[:2, :2], size=200_000)
    hp = np.linalg.norm(pred, axis=1)
    bins = np.linspace(0, np.percentile(h, 99.5), 40)
    axs[0].hist(h, bins=bins, density=True, color='#D62728', alpha=0.6, label=f'GN, {len(e)} valid fixes')
    dens, edges = np.histogram(hp, bins=bins, density=True)
    axs[0].plot(0.5 * (edges[1:] + edges[:-1]), dens, 'k-', lw=1.5,
                label='predicted by CRLB (Gaussian, 2-D norm)')
    axs[0].set_xlabel(r'horizontal error $\sqrt{\Delta x^2 + \Delta y^2}$ (cm)')
    axs[0].set_ylabel('density')
    axs[0].legend(fontsize=8)
    dz = e[:, 2]
    bins = np.linspace(dz.min(), dz.max(), 40)
    axs[1].hist(dz, bins=bins, density=True, color='#D62728', alpha=0.6, label='GN')
    zz = np.linspace(dz.min(), dz.max(), 300)
    mu, sd = dz.mean(), dz.std()
    axs[1].plot(zz, np.exp(-(zz - mu) ** 2 / (2 * sd ** 2)) / (sd * np.sqrt(2 * np.pi)), 'b-',
                label=f'Gaussian fit (mean {mu:.2f}, std {sd:.2f} cm)')
    sc = np.sqrt(C[2, 2])
    axs[1].plot(zz, np.exp(-zz ** 2 / (2 * sc ** 2)) / (sc * np.sqrt(2 * np.pi)), 'k--',
                label=f'CRLB N(0, {sc:.2f} cm)')
    axs[1].set_xlabel(r'depth error $\Delta z$ (cm)')
    axs[1].legend(fontsize=8)
    fig.suptitle('S059 error distribution, CRLB-optimal layout at the true target')
    _save(fig, out_dir, 'error_histogram.png')


def plot_crlb_vs_depth(results, out_dir):
    zs = np.linspace(*Z_RANGE, 191)
    P = np.column_stack([np.full_like(zs, P_TARGET[0]), np.full_like(zs, P_TARGET[1]), zs])
    fig, ax = plt.subplots(figsize=(8, 5))
    for n in LAYOUT_ORDER:
        ax.semilogy(-zs, 100 * crlb_rms(results[n]['buoys'], P), color=COLORS[n], lw=1.8,
                    label=LABELS[n])
    ax.axvline(-P_TARGET[2], color='green', ls=':', label='true target depth')
    ax.set_xlabel('target depth (m)')
    ax.set_ylabel(r'CRLB RMS $\sqrt{\mathrm{tr}\,F^{-1}}$ (cm, log)')
    ax.set_title(f'S059 CRLB vs depth at (x, y) = ({P_TARGET[0]:.0f}, {P_TARGET[1]:.0f}) m')
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3, which='both')
    _save(fig, out_dir, 'crlb_vs_depth.png')


def save_animation(results, out_dir):
    """Deployment and first pings: four drones fly to the CRLB-prior drop points, then estimates
    from successive pings appear (each ping = one Gauss-Newton fix)."""
    b = results['crlb_prior']['buoys']
    errs = results['crlb_prior']['errs']
    launch = np.array([0.0, -400.0, 30.0])
    v_drone = 15.0
    d = np.linalg.norm(b[:, :2] - launch[:2], axis=1)
    t_fly = d.max() / v_drone
    dt = 1.0
    n_fly = int(np.ceil(t_fly / dt))
    n_ping = 24                                  # pings shown, every T_PING s
    n = n_fly + int(n_ping * T_PING / dt) + 1
    step = 3
    n_frames = int(np.ceil((n - 1) / step)) + 1
    idx = [min(f * step, n - 1) for f in range(n_frames)]
    assert len(set(idx)) == n_frames

    fig = plt.figure(figsize=(7, 6))
    ax = fig.add_subplot(111, projection='3d')
    xx, yy = np.meshgrid([-260, 260], [-420, 260])
    ax.plot_surface(xx, yy, np.zeros_like(xx), color='#9ecae1', alpha=0.15)
    ax.scatter(*P_TARGET, color='green', marker='*', s=200, edgecolors='k', depthshade=False)
    drones = ax.scatter([], [], [], color='red', s=40, depthshade=False)
    buoy_sc = ax.scatter([], [], [], color='cyan', s=50, edgecolors='k', depthshade=False)
    est_sc = ax.scatter([], [], [], color='#D62728', s=10, depthshade=False)
    ax.set_xlim(-260, 260); ax.set_ylim(-420, 260); ax.set_zlim(-200, 40)
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)'); ax.set_zlabel('z (m)')
    ax.view_init(elev=18, azim=-60)
    title = ax.set_title('')

    def update(f):
        k = idx[f]
        t = k * dt
        frac = np.minimum(t * v_drone / np.maximum(d, 1e-9), 1.0)
        pos = launch + (np.column_stack([b[:, :2], np.full(N_BUOYS, 30.0)]) - launch) * frac[:, None]
        drones._offsets3d = (pos[:, 0], pos[:, 1], pos[:, 2])
        dropped = frac >= 1.0
        buoy_sc._offsets3d = (b[dropped, 0], b[dropped, 1], b[dropped, 2])
        n_p = 0 if t < t_fly else min(n_ping, int((t - t_fly) // T_PING) + 1)
        est = P_TARGET + 40.0 * errs[:n_p]             # errors x40 so they are visible
        est_sc._offsets3d = (est[:, 0], est[:, 1], est[:, 2])
        state = 'drones flying to drop points' if t < t_fly else f'{n_p} ping(s) localised'
        title.set_text(f'S059 t = {t:5.0f} s: {state}\n(red dots: GN fixes, error magnified x40)')
        return [drones, buoy_sc, est_sc, title]

    anim = animation.FuncAnimation(fig, update, frames=n_frames, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    anim.save(path, writer='pillow', fps=10, dpi=70)
    plt.close(fig)
    print(f'Saved: {path} ({n_frames} frames, flight {t_fly:.1f} s at {v_drone:.0f} m/s)')
    return t_fly


# ── Main ──────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    out = os.path.normpath(OUTPUT_DIR)
    os.makedirs(out, exist_ok=True)
    layouts, results, prior_targets, traces = run_simulation()

    print('\n=== S059 Underwater Target Sonar Marking — Results ===')
    print(f'  sigma_t = {SIGMA_T * 1e3:.1f} ms -> one TDOA in range units sigma_d = {SIGMA_D:.4f} m; '
          f'TDOA correlation = {SIGMA_TDOA[0, 1] / SIGMA_TDOA[0, 0]:.2f}')
    print(f'  True target {tuple(float(v) for v in P_TARGET)} m; GN start '
          f'{tuple(float(v) for v in P_INIT)} m')
    sq = layout_square(SQUARE_HALF)
    sq_p = sq.copy()
    sq_p[2, :2] *= 0.7
    print(f'  Cocyclic check: square CRLB RMS {100 * crlb_rms(sq, P_TARGET):.1f} cm; same square '
          f'with buoy 3 moved 30 % toward the centre (no longer on one circle) '
          f'{100 * crlb_rms(sq_p, P_TARGET):.1f} cm')
    for n in ('square', 'tetra', 'crlb_prior', 'crlb_oracle'):
        b = results[n]['buoys']
        print(f'  Prior-sample objective (mean CRLB RMS over the {N_PRIOR_OPT} optimiser targets) '
              f'[{LABELS[n]}]: '
              f'{100 * np.mean(crlb_rms(b, prior_sample(N_PRIOR_OPT, np.random.default_rng(PRIOR_OPT_SEED)))):.2f} cm')
    for n in LAYOUT_ORDER:
        r = results[n]
        b = r['buoys']
        print(f'  [{LABELS[n]}] buoys (x, y): ' + ', '.join(f'({p[0]:.1f}, {p[1]:.1f})' for p in b)
              + f'; max spread {r["spread"]:.1f} m')
        sd = 100 * np.sqrt(np.diag(r['crlb']))
        msd = 100 * np.sqrt(np.diag(r['mc_cov']))
        print(f'    GDOP {r["gdop"]:.3f}; CRLB RMS {100 * r["crlb_rms"]:.2f} cm '
              f'(card Sigma: {100 * r["crlb_card_rms"]:.2f} cm); CRLB 1-sigma x/y/z '
              f'{sd[0]:.2f} / {sd[1]:.2f} / {sd[2]:.2f} cm')
        print(f'    MC (valid fixes): RMSE {100 * r["mc_rmse"]:.2f} cm, p95 {100 * r["mc_p95"]:.2f} cm, '
              f'std x/y/z {msd[0]:.2f} / {msd[1]:.2f} / {msd[2]:.2f} cm, '
              f'bias |mean| {100 * np.linalg.norm(r["mc_bias"]):.3f} cm; '
              f'RMSE / CRLB = {r["mc_rmse"] / r["crlb_rms"]:.3f}; median (all) '
              f'{100 * r["mc_median"]:.2f} cm')
        print(f'    GN: valid fixes {int(r["valid"].sum())}/{N_TRIALS}, iterations mean '
              f'{r["iters"].mean():.2f} (max {r["iters"].max()})')
        pv = r['prior_valid']
        print(f'    prior ({N_PRIOR_MC} test targets): median error (all) '
              f'{100 * np.median(r["prior_err"]):.2f} cm, RMSE (valid) '
              f'{100 * np.sqrt(np.mean(r["prior_err"][pv] ** 2)):.2f} cm, p95 (valid) '
              f'{100 * np.percentile(r["prior_err"][pv], 95):.2f} cm, mean CRLB RMS '
              f'{100 * np.mean(r["prior_crlb"]):.2f} cm, valid fixes {int(pv.sum())}/{N_PRIOR_MC}')
    pe = np.array([results[n]['prior_err'] for n in LAYOUT_ORDER])
    win = np.argmin(pe, axis=0)
    print('  Prior targets, wins (smallest error per target, all 6 layouts): ' +
          ', '.join(f'{LABELS[n]} {int(np.sum(win == i))}' for i, n in enumerate(LAYOUT_ORDER)))
    main4 = [LAYOUT_ORDER.index(n) for n in ('square', 'tetra', 'crlb_oracle', 'crlb_prior')]
    win4 = np.array(main4)[np.argmin(pe[main4], axis=0)]
    print('  Prior targets, wins among the 4 main layouts: ' +
          ', '.join(f'{LABELS[LAYOUT_ORDER[i]]} {int(np.sum(win4 == i))}' for i in main4))
    pc = np.array([results[n]['prior_crlb'] for n in LAYOUT_ORDER])
    print('  Prior targets, layout with the lowest CRLB RMS: ' +
          ', '.join(f'{LABELS[n]} {int(np.sum(np.argmin(pc, axis=0) == i))}'
                    for i, n in enumerate(LAYOUT_ORDER)))
    for key in ('good', 'collinear'):
        tr = traces[key]
        print(f'  GN traces [{key}]: iterations {min(len(s) for s, _, _ in tr)}-'
              f'{max(len(s) for s, _, _ in tr)}, final error '
              f'{min(e[-1] for _, e, _ in tr):.3g}-{max(e[-1] for _, e, _ in tr):.3g} m, '
              f'valid {sum(ok for _, _, ok in tr)}/10')
    zs = np.array([-10.0, -50.0, -120.0, -200.0])
    for n in ('square', 'tetra', 'crlb_oracle', 'crlb_prior'):
        P = np.column_stack([np.full(4, P_TARGET[0]), np.full(4, P_TARGET[1]), zs])
        v = 100 * crlb_rms(results[n]['buoys'], P)
        print(f'  CRLB RMS vs depth [{LABELS[n]}]: ' +
              ', '.join(f'{-z:.0f} m {x:.1f} cm' for z, x in zip(zs, v)))
    print()

    plot_geometry_3d(results, out)
    plot_crlb_heatmaps(results, out)
    plot_gn_convergence(traces, out)
    plot_error_ellipsoids(results, out)
    plot_rmse_bars(results, out)
    plot_error_histogram(results, out)
    plot_crlb_vs_depth(results, out)
    save_animation(results, out)
