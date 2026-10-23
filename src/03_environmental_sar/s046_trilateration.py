"""
S046 Multi-Drone 3D Trilateration
===================================
Four drones hover at known 3D anchor positions and each measures the slant
range to an unknown beacon (survivor / emergency transmitter) with additive
Gaussian noise.  Two solvers are compared:

  1. Linear Least Squares (LLS) via range-differencing - closed form.
     For the coplanar formation the differenced system has no z information
     (rank 2), so the height is recovered from the sphere equations with the
     prior that the beacon lies below the drones.
  2. Gauss-Newton nonlinear least squares (GN) - iterative, initialised from
     the LLS solution (a realistic initial guess; the truth is never used).

The scenario also evaluates the Geometric Dilution of Precision (GDOP) and the
Cramer-Rao lower bound (CRLB) for a well-spread 3-low + 1-overhead formation
vs. a coplanar flat formation, and produces a GDOP heatmap over the search area.

Usage:
    conda activate drones
    python src/03_environmental_sar/s046_trilateration.py
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

# -- Output directory ---------------------------------------------------------
OUTPUT_DIR = os.path.normpath(os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '03_environmental_sar', 's046_trilateration',
))

# -- Parameters ---------------------------------------------------------------
N_DRONES    = 4
SIGMA_R     = 0.5       # m  - baseline range noise std dev
N_TRIALS    = 500       # Monte Carlo trials per configuration and noise level
EPSILON_TOL = 1e-4      # m  - Gauss-Newton convergence threshold
K_MAX       = 100       # max Gauss-Newton iterations
R_XY        = 80.0      # m  - horizontal spread radius
Z_LOW       = 5.0       # m  - low-altitude drone tier
Z_HIGH      = 25.0      # m  - high-altitude overhead drone
X_TRUE      = np.array([30.0, -20.0, 2.0])   # true beacon position (m)
SIGMA_LEVELS = np.concatenate([[0.1], np.arange(0.25, 3.01, 0.25)])  # noise sweep (m), contains 0.5
DIVERGE_M    = 20.0     # m  - a trial with 3-D error above this counts as a gross error
INIT_GOOD    = 5.0      # m  - per-axis std of the "good" GN start (convergence study)
INIT_POOR    = 50.0     # m  - per-axis std of the "poor" GN start (convergence study)
SEED_MC      = 42       # Monte Carlo seed (same noise stream for every config/level)
SEED_CONV    = 3        # convergence-trace seed
SEED_ANIM    = 99       # animation seed

# Well-spread: 3 drones at 120 deg azimuth + 1 overhead
ANCHORS_GOOD = np.array([
    [ R_XY,                      0.0,                       Z_LOW],
    [-R_XY / 2,  R_XY * np.sqrt(3) / 2,                   Z_LOW],
    [-R_XY / 2, -R_XY * np.sqrt(3) / 2,                   Z_LOW],
    [ 0.0,                       0.0,                      Z_HIGH],
])

# Poor (coplanar): all 4 drones at the same altitude
ANCHORS_FLAT = np.array([
    [ R_XY,   0.0,   Z_LOW],
    [-R_XY,   0.0,   Z_LOW],
    [ 0.0,    R_XY,  Z_LOW],
    [ 0.0,   -R_XY,  Z_LOW],
])


# -- Helpers ------------------------------------------------------------------

def measure_ranges(anchors, x_true, sigma_r, rng):
    """Noisy slant-range measurements from each anchor to the beacon."""
    true_ranges = np.linalg.norm(anchors - x_true, axis=1)
    return true_ranges + rng.normal(0.0, sigma_r, size=len(anchors))


def solve_lls(anchors, ranges):
    """
    Linear least squares via range-differencing (last anchor = reference).

    From |x - p_i|^2 = r_i^2 minus the reference row:
        2 (p_i - p_N)^T x = |p_i|^2 - |p_N|^2 - r_i^2 + r_N^2

    If the anchors are coplanar (A has rank 2) the system carries no
    information along the plane normal; lstsq then returns the minimum-norm
    solution, and the out-of-plane coordinate is recovered from the sphere
    equations, choosing the root below the drone plane (beacon on the ground).
    """
    ref   = anchors[-1]
    r_ref = ranges[-1]
    A = 2.0 * (anchors[:-1] - ref)                                  # (N-1, 3)
    b = (np.sum(anchors[:-1] ** 2, axis=1) - np.dot(ref, ref)
         - ranges[:-1] ** 2 + r_ref ** 2)                           # (N-1,)
    x_hat, _, rank, _ = np.linalg.lstsq(A, b, rcond=None)
    if rank < 3:
        # coplanar anchors (all at the same z): solve z from the spheres
        z_plane = anchors[0, 2]
        d_h2 = np.sum((anchors[:, :2] - x_hat[:2]) ** 2, axis=1)
        dz2 = np.mean(ranges ** 2 - d_h2)
        x_hat[2] = z_plane - np.sqrt(max(dz2, 0.0))
    return x_hat


def compute_jacobian(anchors, x):
    """
    Jacobian of the range model h_i(x) = |x - p_i|:
    H[i,:] = (x - p_i) / |x - p_i|  (unit vector from drone i to x). Shape (N, 3).
    """
    diffs = x - anchors
    norms = np.linalg.norm(diffs, axis=1, keepdims=True)
    return diffs / np.maximum(norms, 1e-12)


def solve_gauss_newton(anchors, ranges, x_init):
    """
    Gauss-Newton for min sum (r_i - h_i(x))^2:
        dx = (H^T H)^{-1} H^T (r - h(x)),  x <- x + dx.
    Returns x_hat (3,) and the list of step norms (convergence trace).
    """
    x      = x_init.copy()
    trace  = []
    for _ in range(K_MAX):
        pred      = np.linalg.norm(x - anchors, axis=1)
        residuals = ranges - pred
        H         = compute_jacobian(anchors, x)
        HtH       = H.T @ H
        if abs(np.linalg.det(HtH)) < 1e-10:
            break
        delta = np.linalg.solve(HtH, H.T @ residuals)
        x     = x + delta
        step  = float(np.linalg.norm(delta))
        trace.append(step)
        if step < EPSILON_TOL:
            break
    return x, trace


def dop(anchors, x):
    """Return (GDOP, HDOP, VDOP) at position x; inf when degenerate."""
    H   = compute_jacobian(anchors, x)
    HtH = H.T @ H
    if abs(np.linalg.det(HtH)) < 1e-10:
        return np.inf, np.inf, np.inf
    cov = np.linalg.inv(HtH)
    return (float(np.sqrt(np.trace(cov))),
            float(np.sqrt(cov[0, 0] + cov[1, 1])),
            float(np.sqrt(cov[2, 2])))


def run_monte_carlo(anchors, sigma_r, n_trials, x_true):
    """
    Monte Carlo comparison of LLS vs Gauss-Newton (GN initialised from LLS).
    Returns error-vector arrays (n_trials, 3) for LLS and GN and the GN
    iteration counts.
    """
    rng = np.random.default_rng(SEED_MC)
    err_lls, err_gn, iters = [], [], []
    for _ in range(n_trials):
        ranges   = measure_ranges(anchors, x_true, sigma_r, rng)
        x_lls    = solve_lls(anchors, ranges)
        x_gn, tr = solve_gauss_newton(anchors, ranges, x_lls)
        err_lls.append(x_lls - x_true)
        err_gn.append(x_gn - x_true)
        iters.append(len(tr))
    return np.array(err_lls), np.array(err_gn), np.array(iters)


def rmse(err):
    """3-D RMSE of an (n, 3) error array."""
    return float(np.sqrt(np.mean(np.sum(err ** 2, axis=1))))


# -- Simulation ---------------------------------------------------------------

def run_simulation():
    """Run all analyses; return a results dict."""
    dop_good = dop(ANCHORS_GOOD, X_TRUE)
    dop_flat = dop(ANCHORS_FLAT, X_TRUE)

    # --- RMSE vs noise level ---
    rm = {k: [] for k in ('lls_good', 'gn_good', 'lls_flat', 'gn_flat',
                          'div_gn_good', 'div_gn_flat')}
    for sigma in SIGMA_LEVELS:
        e_l, e_g, _ = run_monte_carlo(ANCHORS_GOOD, sigma, N_TRIALS, X_TRUE)
        rm['lls_good'].append(rmse(e_l)); rm['gn_good'].append(rmse(e_g))
        rm['div_gn_good'].append(int(np.sum(np.linalg.norm(e_g, axis=1) > DIVERGE_M)))
        e_l, e_g, _ = run_monte_carlo(ANCHORS_FLAT, sigma, N_TRIALS, X_TRUE)
        rm['lls_flat'].append(rmse(e_l)); rm['gn_flat'].append(rmse(e_g))
        rm['div_gn_flat'].append(int(np.sum(np.linalg.norm(e_g, axis=1) > DIVERGE_M)))

    # --- Monte Carlo at exactly the baseline sigma_r = 0.5 m ---
    el_good, eg_good, it_good = run_monte_carlo(ANCHORS_GOOD, SIGMA_R, N_TRIALS, X_TRUE)
    el_flat, eg_flat, it_flat = run_monte_carlo(ANCHORS_FLAT, SIGMA_R, N_TRIALS, X_TRUE)

    # --- GN convergence traces (good vs poor initial guess) ---
    rng3 = np.random.default_rng(SEED_CONV)
    traces_good_init, traces_poor_init = [], []
    final_err_good, final_err_poor = [], []
    for _ in range(10):
        r = measure_ranges(ANCHORS_GOOD, X_TRUE, SIGMA_R, rng3)
        xg, tr_good = solve_gauss_newton(
            ANCHORS_GOOD, r, X_TRUE + rng3.normal(0, INIT_GOOD, 3))
        xp, tr_poor = solve_gauss_newton(
            ANCHORS_GOOD, r, X_TRUE + rng3.normal(0, INIT_POOR, 3))
        traces_good_init.append(tr_good); traces_poor_init.append(tr_poor)
        final_err_good.append(np.linalg.norm(xg - X_TRUE))
        final_err_poor.append(np.linalg.norm(xp - X_TRUE))

    # --- GDOP heatmap (z = 2 m slice) ---
    hmap_x = np.linspace(-120, 120, 80)
    hmap_y = np.linspace(-120, 120, 80)
    gdop_map = np.zeros((len(hmap_y), len(hmap_x)))
    for iy, yy in enumerate(hmap_y):
        for ix, xx in enumerate(hmap_x):
            gdop_map[iy, ix] = dop(ANCHORS_GOOD, np.array([xx, yy, X_TRUE[2]]))[0]

    return dict(
        dop_good=dop_good, dop_flat=dop_flat,
        sigma_levels=SIGMA_LEVELS,
        rmse_lls_good=np.array(rm['lls_good']), rmse_gn_good=np.array(rm['gn_good']),
        rmse_lls_flat=np.array(rm['lls_flat']), rmse_gn_flat=np.array(rm['gn_flat']),
        div_gn_good=np.array(rm['div_gn_good']), div_gn_flat=np.array(rm['div_gn_flat']),
        el_good=el_good, eg_good=eg_good, el_flat=el_flat, eg_flat=eg_flat,
        it_good=it_good, it_flat=it_flat,
        traces_good_init=traces_good_init, traces_poor_init=traces_poor_init,
        final_err_good=np.array(final_err_good), final_err_poor=np.array(final_err_poor),
        hmap_x=hmap_x, hmap_y=hmap_y, gdop_map=gdop_map,
    )


# -- Plot functions -----------------------------------------------------------

def _save(fig, out_dir, name):
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, name)
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f'Saved: {path}')


def plot_anchor_geometry(data, out_dir):
    """3D view of well-spread vs coplanar anchor configurations."""
    fig = plt.figure(figsize=(13, 5))
    titles   = [f"Well-spread (GDOP = {data['dop_good'][0]:.2f})",
                f"Coplanar flat (GDOP = {data['dop_flat'][0]:.2f})"]
    anchors_ = [ANCHORS_GOOD, ANCHORS_FLAT]

    for col, (title, anch) in enumerate(zip(titles, anchors_)):
        ax = fig.add_subplot(1, 2, col + 1, projection='3d')
        ax.scatter(anch[:, 0], anch[:, 1], anch[:, 2],
                   s=120, c='red', marker='^', label='Drone')
        ax.scatter(*X_TRUE, s=200, c='green', marker='*', label='Beacon')
        for i in range(N_DRONES):
            ax.plot([anch[i, 0], X_TRUE[0]], [anch[i, 1], X_TRUE[1]],
                    [anch[i, 2], X_TRUE[2]], 'k--', lw=0.8, alpha=0.5)
        for i, p in enumerate(anch):
            ax.text(p[0], p[1], p[2] + 1.5, f'D{i+1}', fontsize=8,
                    ha='center', color='darkred')
        ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
        ax.set_title(title, fontsize=10)
        ax.legend(fontsize=8, loc='upper left')
        ax.set_xlim(-100, 100); ax.set_ylim(-100, 100); ax.set_zlim(0, 30)

    fig.suptitle('S046 - 3D Anchor Geometry', fontsize=12, fontweight='bold')
    fig.tight_layout()
    _save(fig, out_dir, 'anchor_geometry.png')


def plot_rmse_vs_noise(data, out_dir):
    """RMSE vs range-noise level for LLS and GN in both configurations."""
    fig, ax = plt.subplots(figsize=(9, 5))
    sl = data['sigma_levels']
    crlb_good = sl * data['dop_good'][0]
    crlb_flat = sl * data['dop_flat'][0]

    ax.fill_between(sl, 0, crlb_good, alpha=0.15, color='royalblue',
                    label='below CRLB, well-spread (σ_r·GDOP)')
    ax.fill_between(sl, 0, crlb_flat, alpha=0.10, color='orange',
                    label='below CRLB, coplanar (σ_r·GDOP)')
    ax.plot(sl, crlb_good, color='royalblue', lw=1)
    ax.plot(sl, crlb_flat, color='orange', lw=1)

    ax.plot(sl, data['rmse_lls_good'], 'b-o',  ms=4, label='LLS  well-spread')
    ax.plot(sl, data['rmse_gn_good'],  'b--s', ms=4, label='GN   well-spread')
    ax.plot(sl, data['rmse_lls_flat'], 'r-o',  ms=4, label='LLS  coplanar')
    ax.plot(sl, data['rmse_gn_flat'],  'r--s', ms=4, label='GN   coplanar')

    ax.set_xlabel('Range noise std dev  σ_r (m)', fontsize=11)
    ax.set_yscale('log'); ax.set_ylim(0.1, None)
    ax.set_ylabel('3-D RMSE (m, log scale)', fontsize=11)
    ax.set_title('S046 - RMSE vs Noise Level\n(GN coplanar: RMSE dominated by diverged trials)',
                 fontsize=12, fontweight='bold')
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)
    fig.tight_layout()
    _save(fig, out_dir, 'rmse_vs_noise.png')


def plot_gdop_heatmap(data, out_dir):
    """2-D GDOP heatmap at z = 2 m for the well-spread configuration."""
    fig, ax = plt.subplots(figsize=(7, 6))
    im = ax.pcolormesh(data['hmap_x'], data['hmap_y'], data['gdop_map'],
                       cmap='RdYlGn_r', shading='auto', vmin=1.5, vmax=8.0)
    cb = fig.colorbar(im, ax=ax, label='GDOP (colour capped at 8)')
    cb.ax.tick_params(labelsize=9)
    ax.scatter(ANCHORS_GOOD[:, 0], ANCHORS_GOOD[:, 1],
               c='red', s=100, marker='^', zorder=5, label='Drone (projected)')
    ax.scatter(X_TRUE[0], X_TRUE[1], c='green', edgecolors='k', s=200,
               marker='*', zorder=6, label='Beacon')
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
    ax.set_title('S046 - GDOP Heatmap at z = 2 m\n(well-spread configuration)',
                 fontsize=11, fontweight='bold')
    ax.legend(fontsize=9); ax.set_aspect('equal')
    fig.tight_layout()
    _save(fig, out_dir, 'gdop_heatmap.png')


def plot_convergence(data, out_dir):
    """GN convergence traces for good vs poor initial guesses."""
    fig, axes = plt.subplots(1, 2, figsize=(12, 4), sharey=True)
    datasets  = [data['traces_good_init'], data['traces_poor_init']]
    titles    = [f'Good initial guess (truth + N(0, {INIT_GOOD:.0f}² m²) per axis)',
                 f'Poor initial guess (truth + N(0, {INIT_POOR:.0f}² m²) per axis)']
    colors    = plt.cm.tab10(np.linspace(0, 1, 10))
    for ax, traces, title in zip(axes, datasets, titles):
        for i, tr in enumerate(traces):
            ax.semilogy(np.arange(1, len(tr) + 1), tr, '-o', ms=3,
                        color=colors[i], alpha=0.75, lw=1.2)
        ax.axhline(EPSILON_TOL, color='k', ls='--', lw=1, label=f'ε_tol={EPSILON_TOL}')
        ax.set_xlabel('GN iteration'); ax.set_ylabel('‖Δx‖ (m)')
        ax.set_title(title, fontsize=10); ax.legend(fontsize=8); ax.grid(True, alpha=0.3)
    fig.suptitle('S046 - Gauss-Newton Convergence Traces (well-spread, σ_r = 0.5 m)',
                 fontsize=12, fontweight='bold')
    fig.tight_layout()
    _save(fig, out_dir, 'gn_convergence.png')


def _hist_pair(ax, e_lls, e_gn, ref, ref_label, xlabel, title):
    """Overlay LLS / GN histograms of errors <= DIVERGE_M; gross errors are counted, not drawn."""
    n_out_l = int(np.sum(e_lls > DIVERGE_M)); n_out_g = int(np.sum(e_gn > DIVERGE_M))
    hi = max(e_lls[e_lls <= DIVERGE_M].max(), e_gn[e_gn <= DIVERGE_M].max()) * 1.05
    bins = np.linspace(0, hi, 35)
    ax.hist(e_lls[e_lls <= DIVERGE_M], bins=bins, alpha=0.6, color='steelblue', label='LLS')
    ax.hist(e_gn[e_gn <= DIVERGE_M], bins=bins, alpha=0.6, color='darkorange', label='GN')
    ax.axvline(np.median(e_lls), color='blue', ls='--', lw=1.5,
               label=f'LLS median={np.median(e_lls):.2f} m')
    ax.axvline(np.median(e_gn), color='orange', ls='--', lw=1.5,
               label=f'GN  median={np.median(e_gn):.2f} m')
    ax.axvline(ref, color='k', ls=':', lw=1.5, label=f'{ref_label}={ref:.2f} m')
    ax.set_xlabel(xlabel); ax.set_ylabel('Count')
    ax.set_title(f'{title}\n(not shown: errors > {DIVERGE_M:.0f} m - LLS {n_out_l}, GN {n_out_g})',
                 fontsize=9)
    ax.legend(fontsize=7); ax.grid(True, alpha=0.3)


def plot_error_histogram(data, out_dir):
    """3-D error histograms for LLS vs GN at baseline sigma_r = 0.5 m."""
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.5))
    pairs = [
        (data['el_good'], data['eg_good'], 'Well-spread', data['dop_good'][0]),
        (data['el_flat'], data['eg_flat'], 'Coplanar flat', data['dop_flat'][0]),
    ]
    for ax, (el, eg, label, g) in zip(axes, pairs):
        _hist_pair(ax, np.linalg.norm(el, axis=1), np.linalg.norm(eg, axis=1),
                   SIGMA_R * g, 'CRLB 3-D RMS', '3-D position error (m)',
                   f'3-D error - {label} (σ_r = {SIGMA_R} m, {N_TRIALS} trials)')
    fig.suptitle('S046 - Error Histograms: LLS vs Gauss-Newton', fontsize=12, fontweight='bold')
    fig.tight_layout()
    _save(fig, out_dir, 'error_histogram.png')


def plot_vertical_breakdown(data, out_dir):
    """Horizontal vs vertical error breakdown (LLS and GN): good geometry vs coplanar."""
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    datasets = [
        (data['el_good'], data['eg_good'], 'Well-spread', data['dop_good']),
        (data['el_flat'], data['eg_flat'], 'Coplanar flat', data['dop_flat']),
    ]
    for col, (el, eg, label, d) in enumerate(datasets):
        _hist_pair(axes[0, col], np.linalg.norm(el[:, :2], axis=1),
                   np.linalg.norm(eg[:, :2], axis=1), SIGMA_R * d[1], 'σ_r·HDOP',
                   '√(Δx²+Δy²) (m)', f'Horizontal error - {label}')
        _hist_pair(axes[1, col], np.abs(el[:, 2]), np.abs(eg[:, 2]), SIGMA_R * d[2],
                   'σ_r·VDOP', '|Δz| (m)', f'Vertical error - {label}')
    fig.suptitle(f'S046 - Horizontal vs Vertical Error Breakdown (σ_r = {SIGMA_R} m)',
                 fontsize=12, fontweight='bold')
    fig.tight_layout()
    _save(fig, out_dir, 'vertical_breakdown.png')


def save_animation(out_dir):
    """
    Animate the Gauss-Newton iteration for one noisy measurement set started
    from a deliberately poor guess (truth + N(0, 40^2) m per axis), showing the
    estimate converging to the beacon.  Each iterate is held for 2 frames.
    """
    rng_anim = np.random.default_rng(SEED_ANIM)
    ranges   = measure_ranges(ANCHORS_GOOD, X_TRUE, SIGMA_R, rng_anim)
    x        = X_TRUE + rng_anim.normal(0, 40, 3)

    def cost(xx):
        return float(np.sum((ranges - np.linalg.norm(xx - ANCHORS_GOOD, axis=1)) ** 2))

    path, costs = [x.copy()], [cost(x)]
    for _ in range(K_MAX):
        H   = compute_jacobian(ANCHORS_GOOD, x)
        HtH = H.T @ H
        if abs(np.linalg.det(HtH)) < 1e-10:
            break
        delta = np.linalg.solve(HtH, H.T @ (ranges - np.linalg.norm(x - ANCHORS_GOOD, axis=1)))
        x = x + delta
        path.append(x.copy()); costs.append(cost(x))
        if np.linalg.norm(delta) < EPSILON_TOL:
            break
    path = np.array(path)
    n_steps = len(path)
    print(f'Animation: GN from poor guess converged in {n_steps - 1} iterations, '
          f'final error {np.linalg.norm(path[-1] - X_TRUE):.3f} m')

    fig = plt.figure(figsize=(12, 5))
    ax3 = fig.add_subplot(1, 2, 1, projection='3d')
    ax2 = fig.add_subplot(1, 2, 2)
    ax3.scatter(ANCHORS_GOOD[:, 0], ANCHORS_GOOD[:, 1], ANCHORS_GOOD[:, 2],
                s=80, c='red', marker='^')
    ax3.scatter(*X_TRUE, s=200, c='green', marker='*')
    for i, p in enumerate(ANCHORS_GOOD):
        ax3.text(p[0], p[1], p[2] + 1.5, f'D{i+1}', fontsize=8, color='darkred')
    ax3.set_xlabel('X (m)'); ax3.set_ylabel('Y (m)'); ax3.set_zlabel('Z (m)')
    ax3.set_title('GN iterates (blue) -> beacon (green star)', fontsize=10)
    (traj_line,) = ax3.plot([], [], [], 'b-o', ms=4, lw=1.2)
    (est_dot,)   = ax3.plot([], [], [], 'bo', ms=8)
    all_pts = np.vstack([ANCHORS_GOOD, path])
    margin = 15
    ax3.set_xlim(all_pts[:, 0].min() - margin, all_pts[:, 0].max() + margin)
    ax3.set_ylim(all_pts[:, 1].min() - margin, all_pts[:, 1].max() + margin)
    ax3.set_zlim(min(0, all_pts[:, 2].min() - margin), all_pts[:, 2].max() + margin)

    (cost_line,) = ax2.semilogy([], [], 'b-o', ms=4, lw=1.2)
    ax2.set_xlim(-0.5, n_steps - 0.5)
    ax2.set_ylim(min(costs) * 0.5, max(costs) * 2)
    ax2.set_xlabel('Iteration'); ax2.set_ylabel('Cost  J(x) (m²)')
    ax2.set_title('Residual cost vs iteration', fontsize=10)
    ax2.grid(True, alpha=0.3)
    title_txt = fig.suptitle('', fontsize=11, fontweight='bold')

    def update(k):
        traj_line.set_data(path[:k + 1, 0], path[:k + 1, 1])
        traj_line.set_3d_properties(path[:k + 1, 2])
        est_dot.set_data([path[k, 0]], [path[k, 1]])
        est_dot.set_3d_properties([path[k, 2]])
        cost_line.set_data(np.arange(k + 1), costs[:k + 1])
        err = np.linalg.norm(path[k] - X_TRUE)
        title_txt.set_text(f'S046 - Gauss-Newton  iter={k}  |error|={err:.3f} m')
        return traj_line, est_dot, cost_line, title_txt

    frames = [k for k in range(n_steps) for _ in range(2)] + [n_steps - 1] * 4
    ani = animation.FuncAnimation(fig, update, frames=frames, blit=False)
    os.makedirs(out_dir, exist_ok=True)
    path_gif = os.path.join(out_dir, 'animation.gif')
    ani.save(path_gif, writer='pillow', fps=2, dpi=100)
    plt.close(fig)
    print(f'Saved: {path_gif} ({len(frames)} frames)')


# -- Main ---------------------------------------------------------------------

def _report(data):
    dg, df = data['dop_good'], data['dop_flat']
    print('\n-- Geometry --------------------------------------------')
    print(f"Well-spread: GDOP {dg[0]:.3f}  HDOP {dg[1]:.3f}  VDOP {dg[2]:.3f}")
    print(f"Coplanar   : GDOP {df[0]:.3f}  HDOP {df[1]:.3f}  VDOP {df[2]:.3f}")
    print(f"CRLB 3D RMS (sigma_r={SIGMA_R} m): well-spread {SIGMA_R*dg[0]:.3f} m, "
          f"coplanar {SIGMA_R*df[0]:.3f} m")
    print(f"CRLB horiz / vert (well-spread): {SIGMA_R*dg[1]:.3f} / {SIGMA_R*dg[2]:.3f} m")
    print(f"CRLB horiz / vert (coplanar)   : {SIGMA_R*df[1]:.3f} / {SIGMA_R*df[2]:.3f} m")

    for name, el, eg, it in [('well-spread', data['el_good'], data['eg_good'], data['it_good']),
                             ('coplanar', data['el_flat'], data['eg_flat'], data['it_flat'])]:
        print(f'\n-- Baseline sigma_r = {SIGMA_R} m, {name} ({N_TRIALS} trials) --')
        for lab, e in (('LLS', el), ('GN ', eg)):
            h = np.linalg.norm(e[:, :2], axis=1); v = np.abs(e[:, 2])
            print(f'  {lab}: 3D RMSE {rmse(e):.3f} m | mean 3D err '
                  f'{np.linalg.norm(e, axis=1).mean():.3f} m | horiz RMS '
                  f'{np.sqrt(np.mean(h**2)):.3f} m | vert RMS {np.sqrt(np.mean(v**2)):.3f} m '
                  f'| mean |dz| {v.mean():.3f} m | bias (x,y,z) '
                  f'({e[:,0].mean():+.3f}, {e[:,1].mean():+.3f}, {e[:,2].mean():+.3f}) m')
        print(f'  GN iterations: mean {it.mean():.2f}, max {it.max()}')
        for lab, e in (('LLS', el), ('GN ', eg)):
            n3 = np.linalg.norm(e, axis=1); ok = n3 <= DIVERGE_M
            z_hat = e[:, 2] + X_TRUE[2]
            above = np.sum(z_hat > Z_LOW + 1e-9)
            on_plane = np.sum(np.abs(z_hat - Z_LOW) < 1e-6)
            print(f'  {lab}: median 3D err {np.median(n3):.3f} m | max {n3.max():.1f} m '
                  f'| gross errors (>{DIVERGE_M:.0f} m) {np.sum(~ok)}/{len(n3)} '
                  f'| RMSE without gross errors {rmse(e[ok]):.3f} m')
            print(f'       z_hat > Z_LOW: {above} | z_hat == Z_LOW (stuck on drone plane): {on_plane}')

    print('\n-- RMSE sweep (m) --------------------------------------')
    print(' sigma_r | LLS good | GN good | CRLB good | LLS flat |  GN flat | CRLB flat | GN >20 m good/flat')
    for i, s in enumerate(data['sigma_levels']):
        print(f'  {s:5.2f}  | {data["rmse_lls_good"][i]:8.3f} | {data["rmse_gn_good"][i]:7.3f} | '
              f'{s*dg[0]:9.3f} | {data["rmse_lls_flat"][i]:8.3f} | {data["rmse_gn_flat"][i]:8.1f} | '
              f'{s*df[0]:9.3f} | {data["div_gn_good"][i]}/{data["div_gn_flat"][i]}')

    print('\n-- GN convergence study (10 runs, well-spread) ---------')
    for lab, trs, fe in [('good init', data['traces_good_init'], data['final_err_good']),
                         ('poor init', data['traces_poor_init'], data['final_err_poor'])]:
        n = [len(t) for t in trs]
        print(f'  {lab}: iterations min {min(n)} / max {max(n)}, '
              f'final 3D error max {fe.max():.3f} m')
    g = data['gdop_map']
    print(f"\nGDOP map (z=2 m, 240x240 m): min {g.min():.3f}, median {np.median(g):.3f}, "
          f"max {g.max():.3f}; share of cells with GDOP < 3: {np.mean(g < 3)*100:.1f}%")


if __name__ == '__main__':
    print('Running S046 Multi-Drone 3D Trilateration...')
    data = run_simulation()
    _report(data)
    out_dir = OUTPUT_DIR
    plot_anchor_geometry(data, out_dir)
    plot_rmse_vs_noise(data, out_dir)
    plot_gdop_heatmap(data, out_dir)
    plot_convergence(data, out_dir)
    plot_error_histogram(data, out_dir)
    plot_vertical_breakdown(data, out_dir)
    save_animation(out_dir)
    print('\nDone. All outputs saved to:', out_dir)
