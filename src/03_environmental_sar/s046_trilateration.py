"""
S046 Multi-Drone 3D Trilateration
===================================
Four drones hover at known 3D anchor positions and each measures the slant
range to an unknown beacon (survivor / emergency transmitter) with additive
Gaussian noise.  Two solvers are compared:

  1. Linear Least Squares (LLS) via range-differencing — closed-form, fast,
     slightly biased at high noise.
  2. Gauss-Newton nonlinear least squares (GN) — iterative, unbiased, slower.

The scenario also evaluates Geometric Dilution of Precision (GDOP) for a
well-spread 3-low + 1-overhead drone formation vs. a coplanar flat formation,
and produces a GDOP heatmap over the 2-D search area.

Usage:
    conda activate drones
    python src/03_environmental_sar/s046_trilateration.py
"""

import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

# ── Output directory ────────────────────────────────────────────────────────
OUTPUT_DIR = os.path.normpath(os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '03_environmental_sar', 's046_trilateration',
))

# ── Parameters ──────────────────────────────────────────────────────────────
N_DRONES    = 4
SIGMA_R     = 0.5       # m  — baseline range noise std dev
N_TRIALS    = 500       # Monte Carlo trials per configuration
EPSILON_TOL = 1e-4      # m  — Gauss-Newton convergence threshold
K_MAX       = 100       # max Gauss-Newton iterations
R_XY        = 80.0      # m  — horizontal spread radius
Z_LOW       = 5.0       # m  — low-altitude drone tier
Z_HIGH      = 25.0      # m  — high-altitude overhead drone
X_TRUE      = np.array([30.0, -20.0, 2.0])   # true beacon position (m)

# Well-spread: 3 drones at 120° azimuth + 1 overhead
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

RNG = np.random.default_rng(0)   # global fixed seed


# ── Helpers ─────────────────────────────────────────────────────────────────

def measure_ranges(anchors, x_true, sigma_r, rng):
    """Noisy slant-range measurements from each anchor to the beacon."""
    true_ranges = np.linalg.norm(anchors - x_true, axis=1)
    return true_ranges + rng.normal(0.0, sigma_r, size=len(anchors))


def solve_lls(anchors, ranges):
    """
    Linear least-squares via range-differencing.
    The last anchor is used as the reference.
    Returns x_hat (3,).
    """
    ref   = anchors[-1]
    r_ref = ranges[-1]
    A = 2.0 * (anchors[:-1] - ref)                            # (N-1, 3)
    b = (ranges[:-1] ** 2 - r_ref ** 2
         - np.sum(anchors[:-1] ** 2, axis=1)
         + np.dot(ref, ref))                                   # (N-1,)
    x_hat, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
    return x_hat


def compute_jacobian(anchors, x):
    """
    Jacobian H[i,:] = unit vector from current estimate x towards anchor i.
    Shape: (N, 3).
    """
    diffs = anchors - x
    norms = np.linalg.norm(diffs, axis=1, keepdims=True)
    return diffs / np.maximum(norms, 1e-12)


def solve_gauss_newton(anchors, ranges, x_init):
    """
    Gauss-Newton nonlinear least squares for range measurements.
    Returns x_hat (3,) and a list of step-norms (convergence trace).
    """
    x      = x_init.copy()
    trace  = []
    for _ in range(K_MAX):
        pred      = np.linalg.norm(anchors - x, axis=1)
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


def compute_gdop(anchors, x):
    """
    GDOP = sqrt(trace((H^T H)^{-1})) evaluated at position x.
    Returns inf when the geometry is degenerate.
    """
    H   = compute_jacobian(anchors, x)
    HtH = H.T @ H
    if abs(np.linalg.det(HtH)) < 1e-10:
        return np.inf
    cov = np.linalg.inv(HtH)
    return float(np.sqrt(np.trace(cov)))


def run_monte_carlo(anchors, sigma_r, n_trials, x_true, x_init_offset=5.0):
    """
    Monte Carlo comparison of LLS vs Gauss-Newton over many noise realisations.
    Returns error arrays (n_trials,) for LLS and GN.
    """
    rng  = np.random.default_rng(42)
    x_init = x_true + rng.normal(0.0, x_init_offset, size=3)
    errors_lls, errors_gn = [], []
    for _ in range(n_trials):
        ranges      = measure_ranges(anchors, x_true, sigma_r, rng)
        x_lls       = solve_lls(anchors, ranges)
        x_gn, _     = solve_gauss_newton(anchors, ranges, x_init)
        errors_lls.append(np.linalg.norm(x_lls - x_true))
        errors_gn.append(np.linalg.norm(x_gn  - x_true))
    return np.array(errors_lls), np.array(errors_gn)


# ── Simulation ───────────────────────────────────────────────────────────────

def run_simulation():
    """Run all analyses; return a results dict."""
    rng = np.random.default_rng(0)

    # --- GDOP for both configurations ---
    gdop_good = compute_gdop(ANCHORS_GOOD, X_TRUE)
    gdop_flat = compute_gdop(ANCHORS_FLAT, X_TRUE)

    # --- RMSE vs noise level ---
    sigma_levels = np.linspace(0.1, 3.0, 15)
    rmse_lls_good, rmse_gn_good = [], []
    rmse_lls_flat, rmse_gn_flat = [], []

    for sigma in sigma_levels:
        e_lls, e_gn = run_monte_carlo(ANCHORS_GOOD, sigma, N_TRIALS, X_TRUE)
        rmse_lls_good.append(float(np.sqrt(np.mean(e_lls ** 2))))
        rmse_gn_good.append(float(np.sqrt(np.mean(e_gn  ** 2))))

        e_lls, e_gn = run_monte_carlo(ANCHORS_FLAT, sigma, N_TRIALS, X_TRUE)
        rmse_lls_flat.append(float(np.sqrt(np.mean(e_lls ** 2))))
        rmse_gn_flat.append(float(np.sqrt(np.mean(e_gn  ** 2))))

    # --- Monte Carlo at baseline sigma_r=0.5 for histograms ---
    errors_lls_good, errors_gn_good = run_monte_carlo(
        ANCHORS_GOOD, SIGMA_R, N_TRIALS, X_TRUE)
    errors_lls_flat, errors_gn_flat = run_monte_carlo(
        ANCHORS_FLAT, SIGMA_R, N_TRIALS, X_TRUE)

    # --- Horizontal / vertical error breakdown (coplanar vs good) ---
    rng2 = np.random.default_rng(7)
    x_init_good = X_TRUE + rng2.normal(0, 5, 3)
    herr_good, verr_good = [], []
    herr_flat, verr_flat = [], []
    for _ in range(N_TRIALS):
        for anchors, herr, verr in [
            (ANCHORS_GOOD, herr_good, verr_good),
            (ANCHORS_FLAT, herr_flat, verr_flat),
        ]:
            r  = measure_ranges(anchors, X_TRUE, SIGMA_R, rng2)
            xh, _ = solve_gauss_newton(anchors, r, x_init_good)
            herr.append(float(np.linalg.norm(xh[:2] - X_TRUE[:2])))
            verr.append(float(abs(xh[2] - X_TRUE[2])))

    # --- GN convergence traces (good vs poor initial guess) ---
    rng3 = np.random.default_rng(3)
    traces_good_init = []
    traces_poor_init = []
    for _ in range(10):
        r = measure_ranges(ANCHORS_GOOD, X_TRUE, SIGMA_R, rng3)
        _, tr_good = solve_gauss_newton(
            ANCHORS_GOOD, r, X_TRUE + rng3.normal(0, 5, 3))
        _, tr_poor = solve_gauss_newton(
            ANCHORS_GOOD, r, X_TRUE + rng3.normal(0, 50, 3))
        traces_good_init.append(tr_good)
        traces_poor_init.append(tr_poor)

    # --- GDOP heatmap (z=2 m slice) ---
    hmap_x = np.linspace(-120, 120, 80)
    hmap_y = np.linspace(-120, 120, 80)
    hmap_z = 2.0
    gdop_map = np.zeros((len(hmap_y), len(hmap_x)))
    for iy, yy in enumerate(hmap_y):
        for ix, xx in enumerate(hmap_x):
            pt = np.array([xx, yy, hmap_z])
            gdop_map[iy, ix] = compute_gdop(ANCHORS_GOOD, pt)
    gdop_map = np.clip(gdop_map, 0, 15)   # cap for display

    return dict(
        # geometry
        gdop_good=gdop_good, gdop_flat=gdop_flat,
        # RMSE curves
        sigma_levels=sigma_levels,
        rmse_lls_good=np.array(rmse_lls_good),
        rmse_gn_good=np.array(rmse_gn_good),
        rmse_lls_flat=np.array(rmse_lls_flat),
        rmse_gn_flat=np.array(rmse_gn_flat),
        # histograms at baseline
        errors_lls_good=errors_lls_good,
        errors_gn_good=errors_gn_good,
        errors_lls_flat=errors_lls_flat,
        errors_gn_flat=errors_gn_flat,
        # horizontal/vertical breakdown
        herr_good=np.array(herr_good), verr_good=np.array(verr_good),
        herr_flat=np.array(herr_flat), verr_flat=np.array(verr_flat),
        # convergence traces
        traces_good_init=traces_good_init,
        traces_poor_init=traces_poor_init,
        # heatmap
        hmap_x=hmap_x, hmap_y=hmap_y, gdop_map=gdop_map,
    )


# ── Plot functions ────────────────────────────────────────────────────────────

def plot_anchor_geometry(out_dir):
    """3D view of well-spread vs coplanar anchor configurations."""
    fig = plt.figure(figsize=(13, 5))
    titles   = ['Well-spread (GDOP ≈ 1.4)', 'Coplanar flat (GDOP ≈ 4.2)']
    anchors_ = [ANCHORS_GOOD, ANCHORS_FLAT]

    for col, (title, anch) in enumerate(zip(titles, anchors_)):
        ax = fig.add_subplot(1, 2, col + 1, projection='3d')
        # drones
        ax.scatter(anch[:, 0], anch[:, 1], anch[:, 2],
                   s=120, c='red', marker='^', zorder=5, label='Drone')
        # beacon
        ax.scatter(*X_TRUE, s=200, c='lime', marker='*', zorder=6, label='Beacon')
        # range lines
        for i in range(N_DRONES):
            ax.plot([anch[i, 0], X_TRUE[0]],
                    [anch[i, 1], X_TRUE[1]],
                    [anch[i, 2], X_TRUE[2]],
                    'k--', lw=0.8, alpha=0.5)
        # labels
        for i, p in enumerate(anch):
            ax.text(p[0], p[1], p[2] + 1.5, f'D{i+1}', fontsize=8,
                    ha='center', color='darkred')
        ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
        ax.set_title(title, fontsize=10)
        ax.legend(fontsize=8, loc='upper left')
        ax.set_xlim(-100, 100); ax.set_ylim(-100, 100); ax.set_zlim(0, 30)

    fig.suptitle('S046 · 3D Anchor Geometry', fontsize=12, fontweight='bold')
    plt.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'anchor_geometry.png')
    plt.savefig(path, dpi=150); plt.close()
    print(f'Saved: {path}')


def plot_rmse_vs_noise(data, out_dir):
    """RMSE vs range-noise level for LLS and GN in both configurations."""
    fig, ax = plt.subplots(figsize=(9, 5))
    sl = data['sigma_levels']

    # CRLB envelope for well-spread config: sigma_pos = sigma_r * GDOP
    crlb_good = sl * data['gdop_good']
    crlb_flat = sl * data['gdop_flat']

    ax.fill_between(sl, crlb_good * 0.9, crlb_good * 1.1,
                    alpha=0.15, color='royalblue', label='CRLB band (good geom)')
    ax.fill_between(sl, crlb_flat * 0.9, crlb_flat * 1.1,
                    alpha=0.15, color='orange', label='CRLB band (flat geom)')

    ax.plot(sl, data['rmse_lls_good'], 'b-o',  ms=4, label='LLS  well-spread')
    ax.plot(sl, data['rmse_gn_good'],  'b--s', ms=4, label='GN   well-spread')
    ax.plot(sl, data['rmse_lls_flat'], 'r-o',  ms=4, label='LLS  coplanar')
    ax.plot(sl, data['rmse_gn_flat'],  'r--s', ms=4, label='GN   coplanar')

    ax.set_xlabel('Range noise std dev  σ_r (m)', fontsize=11)
    ax.set_ylabel('3-D RMSE (m)', fontsize=11)
    ax.set_title('S046 · RMSE vs Noise Level', fontsize=12, fontweight='bold')
    ax.legend(fontsize=9); ax.grid(True, alpha=0.3)
    plt.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'rmse_vs_noise.png')
    plt.savefig(path, dpi=150); plt.close()
    print(f'Saved: {path}')


def plot_gdop_heatmap(data, out_dir):
    """2-D GDOP heatmap at z=2 m for the well-spread configuration."""
    fig, ax = plt.subplots(figsize=(7, 6))
    gmap = data['gdop_map']
    hx   = data['hmap_x']
    hy   = data['hmap_y']

    im = ax.pcolormesh(hx, hy, gmap, cmap='RdYlGn_r', shading='auto',
                       vmin=1.0, vmax=8.0)
    cb = fig.colorbar(im, ax=ax, label='GDOP')
    cb.ax.tick_params(labelsize=9)

    # overlay anchors (projected to z=2 m)
    ax.scatter(ANCHORS_GOOD[:, 0], ANCHORS_GOOD[:, 1],
               c='red', s=100, marker='^', zorder=5, label='Drone (projected)')
    ax.scatter(X_TRUE[0], X_TRUE[1],
               c='lime', s=200, marker='*', zorder=6, label='Beacon')

    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
    ax.set_title('S046 · GDOP Heatmap at z = 2 m\n(well-spread configuration)',
                 fontsize=11, fontweight='bold')
    ax.legend(fontsize=9); ax.set_aspect('equal')
    plt.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'gdop_heatmap.png')
    plt.savefig(path, dpi=150); plt.close()
    print(f'Saved: {path}')


def plot_convergence(data, out_dir):
    """GN convergence traces for good vs poor initial guesses."""
    fig, axes = plt.subplots(1, 2, figsize=(12, 4), sharey=True)
    datasets  = [data['traces_good_init'], data['traces_poor_init']]
    titles    = ['Good initial guess (±5 m)', 'Poor initial guess (±50 m)']
    colors    = plt.cm.tab10(np.linspace(0, 1, 10))

    for ax, traces, title in zip(axes, datasets, titles):
        for i, tr in enumerate(traces):
            ax.semilogy(tr, color=colors[i], alpha=0.75, lw=1.2)
        ax.axhline(EPSILON_TOL, color='k', ls='--', lw=1, label=f'ε_tol={EPSILON_TOL}')
        ax.set_xlabel('GN Iteration'); ax.set_ylabel('‖Δx‖ (m)')
        ax.set_title(title, fontsize=10); ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

    fig.suptitle('S046 · Gauss-Newton Convergence Traces', fontsize=12, fontweight='bold')
    plt.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'gn_convergence.png')
    plt.savefig(path, dpi=150); plt.close()
    print(f'Saved: {path}')


def plot_error_histogram(data, out_dir):
    """Error histograms for LLS vs GN at baseline σ_r = 0.5 m (well-spread)."""
    fig, axes = plt.subplots(1, 2, figsize=(12, 4))
    pairs = [
        (data['errors_lls_good'], data['errors_gn_good'], 'Well-spread'),
        (data['errors_lls_flat'], data['errors_gn_flat'], 'Coplanar flat'),
    ]
    colors = [('steelblue', 'darkorange')]

    for ax, (e_lls, e_gn, label) in zip(axes, pairs):
        bins = np.linspace(0, max(e_lls.max(), e_gn.max()) * 1.05, 35)
        ax.hist(e_lls, bins=bins, alpha=0.6, color='steelblue', label='LLS')
        ax.hist(e_gn,  bins=bins, alpha=0.6, color='darkorange',  label='GN')
        ax.axvline(e_lls.mean(), color='blue',   ls='--', lw=1.5,
                   label=f'LLS mean={e_lls.mean():.2f} m')
        ax.axvline(e_gn.mean(),  color='orange', ls='--', lw=1.5,
                   label=f'GN  mean={e_gn.mean():.2f} m')
        ax.set_xlabel('3-D position error (m)'); ax.set_ylabel('Count')
        ax.set_title(f'Error distribution — {label}\n(σ_r = {SIGMA_R} m, {N_TRIALS} trials)',
                     fontsize=10)
        ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

    fig.suptitle('S046 · Error Histograms: LLS vs Gauss-Newton', fontsize=12, fontweight='bold')
    plt.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'error_histogram.png')
    plt.savefig(path, dpi=150); plt.close()
    print(f'Saved: {path}')


def plot_vertical_breakdown(data, out_dir):
    """Horizontal vs vertical error breakdown: good geometry vs coplanar."""
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))

    datasets = [
        (data['herr_good'], data['verr_good'], 'Well-spread'),
        (data['herr_flat'], data['verr_flat'], 'Coplanar flat'),
    ]

    for col, (herr, verr, label) in enumerate(datasets):
        max_h = herr.max() * 1.05
        max_v = verr.max() * 1.05

        axes[0, col].hist(herr, bins=30, color='steelblue', alpha=0.75)
        axes[0, col].axvline(herr.mean(), color='navy', ls='--', lw=1.5,
                              label=f'mean={herr.mean():.2f} m')
        axes[0, col].set_title(f'Horizontal error — {label}', fontsize=10)
        axes[0, col].set_xlabel('√(Δx²+Δy²) (m)')
        axes[0, col].set_ylabel('Count')
        axes[0, col].legend(fontsize=8); axes[0, col].grid(True, alpha=0.3)

        axes[1, col].hist(verr, bins=30, color='coral', alpha=0.75)
        axes[1, col].axvline(verr.mean(), color='darkred', ls='--', lw=1.5,
                              label=f'mean={verr.mean():.2f} m')
        axes[1, col].set_title(f'Vertical error — {label}', fontsize=10)
        axes[1, col].set_xlabel('|Δz| (m)')
        axes[1, col].set_ylabel('Count')
        axes[1, col].legend(fontsize=8); axes[1, col].grid(True, alpha=0.3)

    fig.suptitle('S046 · Horizontal vs Vertical Error Breakdown (GN estimator)',
                 fontsize=12, fontweight='bold')
    plt.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'vertical_breakdown.png')
    plt.savefig(path, dpi=150); plt.close()
    print(f'Saved: {path}')


def save_animation(data, out_dir):
    """
    Animate the Gauss-Newton iteration for a single noisy measurement set,
    showing the estimate converging to the true beacon position.
    """
    import matplotlib.animation as animation

    rng_anim = np.random.default_rng(99)
    ranges   = measure_ranges(ANCHORS_GOOD, X_TRUE, SIGMA_R, rng_anim)
    x_init   = X_TRUE + rng_anim.normal(0, 40, 3)   # deliberately poor guess

    # Collect full path of iterates
    x      = x_init.copy()
    path   = [x.copy()]
    costs  = []
    for _ in range(K_MAX):
        pred      = np.linalg.norm(ANCHORS_GOOD - x, axis=1)
        residuals = ranges - pred
        H         = compute_jacobian(ANCHORS_GOOD, x)
        HtH       = H.T @ H
        if abs(np.linalg.det(HtH)) < 1e-10:
            break
        delta = np.linalg.solve(HtH, H.T @ residuals)
        x     = x + delta
        path.append(x.copy())
        costs.append(float(np.sum((ranges - np.linalg.norm(ANCHORS_GOOD - x, axis=1)) ** 2)))
        if np.linalg.norm(delta) < EPSILON_TOL:
            break

    path    = np.array(path)
    n_steps = len(path)

    fig = plt.figure(figsize=(12, 5))
    ax3 = fig.add_subplot(1, 2, 1, projection='3d')
    ax2 = fig.add_subplot(1, 2, 2)

    # static elements
    ax3.scatter(ANCHORS_GOOD[:, 0], ANCHORS_GOOD[:, 1], ANCHORS_GOOD[:, 2],
                s=80, c='red', marker='^', zorder=5)
    ax3.scatter(*X_TRUE, s=200, c='lime', marker='*', zorder=6)
    for i, p in enumerate(ANCHORS_GOOD):
        ax3.text(p[0], p[1], p[2] + 1.5, f'D{i+1}', fontsize=8, color='darkred')

    ax3.set_xlabel('X (m)'); ax3.set_ylabel('Y (m)'); ax3.set_zlabel('Z (m)')
    ax3.set_title('GN Convergence (3-D)', fontsize=10)

    # dynamic elements
    (traj_line,)  = ax3.plot([], [], [], 'b-o', ms=4, lw=1.2, label='GN path')
    (est_dot,)    = ax3.plot([], [], [], 'bo',  ms=8, zorder=7)

    (cost_line,)  = ax2.semilogy([], [], 'b-o', ms=4, lw=1.2)
    ax2.set_xlim(0, n_steps - 1); ax2.set_ylim(1e-4, max(costs) * 2 if costs else 1)
    ax2.set_xlabel('Iteration'); ax2.set_ylabel('Cost  J(x)')
    ax2.set_title('Residual Cost vs Iteration', fontsize=10)
    ax2.grid(True, alpha=0.3)

    title_txt = fig.suptitle('', fontsize=11, fontweight='bold')

    # axis limits driven by the GN path + anchors
    all_pts = np.vstack([ANCHORS_GOOD, path])
    margin  = 15
    ax3.set_xlim(all_pts[:, 0].min() - margin, all_pts[:, 0].max() + margin)
    ax3.set_ylim(all_pts[:, 1].min() - margin, all_pts[:, 1].max() + margin)
    ax3.set_zlim(0, all_pts[:, 2].max() + margin)

    def init():
        traj_line.set_data([], []); traj_line.set_3d_properties([])
        est_dot.set_data([], []);   est_dot.set_3d_properties([])
        cost_line.set_data([], [])
        return traj_line, est_dot, cost_line

    def update(frame):
        xs = path[:frame + 1, 0]
        ys = path[:frame + 1, 1]
        zs = path[:frame + 1, 2]
        traj_line.set_data(xs, ys); traj_line.set_3d_properties(zs)
        est_dot.set_data([xs[-1]], [ys[-1]]); est_dot.set_3d_properties([zs[-1]])

        if frame > 0 and frame - 1 < len(costs):
            iters = list(range(1, frame + 1))
            cost_line.set_data(iters, costs[:frame])

        err = np.linalg.norm(path[frame] - X_TRUE)
        title_txt.set_text(f'S046 · Gauss-Newton  iter={frame}  |error|={err:.3f} m')
        return traj_line, est_dot, cost_line, title_txt

    n_frames = min(n_steps, 40)
    step     = max(1, n_steps // n_frames)
    frames   = list(range(0, n_steps, step))
    if n_steps - 1 not in frames:
        frames.append(n_steps - 1)

    ani = animation.FuncAnimation(
        fig, update, frames=frames, init_func=init,
        blit=False, interval=200,
    )
    os.makedirs(out_dir, exist_ok=True)
    path_gif = os.path.join(out_dir, 'animation.gif')
    ani.save(path_gif, writer='pillow', fps=8, dpi=100)
    plt.close()
    print(f'Saved: {path_gif}')


# ── Main ─────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    print('Running S046 Multi-Drone 3D Trilateration...')
    data = run_simulation()

    # Key metrics
    print(f"\n── Key Metrics ──────────────────────────────────────")
    print(f"GDOP  well-spread config  : {data['gdop_good']:.3f}")
    print(f"GDOP  coplanar flat config: {data['gdop_flat']:.3f}")
    print(f"CRLB 3D RMS (good, σ=0.5 m) : {SIGMA_R * data['gdop_good']:.3f} m")
    print(f"CRLB 3D RMS (flat, σ=0.5 m) : {SIGMA_R * data['gdop_flat']:.3f} m")

    idx_baseline = np.argmin(np.abs(data['sigma_levels'] - SIGMA_R))
    print(f"\nAt σ_r = 0.5 m (well-spread):")
    print(f"  LLS RMSE : {data['rmse_lls_good'][idx_baseline]:.3f} m")
    print(f"  GN  RMSE : {data['rmse_gn_good'][idx_baseline]:.3f} m")
    print(f"\nAt σ_r = 0.5 m (coplanar):")
    print(f"  LLS RMSE : {data['rmse_lls_flat'][idx_baseline]:.3f} m")
    print(f"  GN  RMSE : {data['rmse_gn_flat'][idx_baseline]:.3f} m")

    print(f"\nHorizontal error (GN, well-spread) : {data['herr_good'].mean():.3f} m mean")
    print(f"Vertical   error (GN, well-spread) : {data['verr_good'].mean():.3f} m mean")
    print(f"Horizontal error (GN, coplanar)    : {data['herr_flat'].mean():.3f} m mean")
    print(f"Vertical   error (GN, coplanar)    : {data['verr_flat'].mean():.3f} m mean")
    print(f"────────────────────────────────────────────────────\n")

    out_dir = OUTPUT_DIR
    plot_anchor_geometry(out_dir)
    plot_rmse_vs_noise(data, out_dir)
    plot_gdop_heatmap(data, out_dir)
    plot_convergence(data, out_dir)
    plot_error_histogram(data, out_dir)
    plot_vertical_breakdown(data, out_dir)
    save_animation(data, out_dir)

    print('\nDone. All outputs saved to:', out_dir)
