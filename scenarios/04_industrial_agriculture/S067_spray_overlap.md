# S067 Spray Coverage Overlap Optimization

**Domain**: Industrial & Agriculture | **Difficulty**: ⭐⭐ | **Status**: `[ ]` Not Started
**Algorithm**: Gaussian Spray Model + CV-Minimisation Optimisation | **Dimension**: 2D

---

## Problem Definition

**Setup**: A single agricultural drone is tasked with spraying a $50 \times 50$ m rectangular
field using a boom sprayer. The sprayer delivers chemical at a Gaussian cross-track dose profile
with effective spray width $w_{spray} = 2$ m ($\sigma_s = w_{spray}/4 = 0.5$ m). The drone flies
parallel east-west strips separated by a centre-to-centre spacing $d$. GPS positioning error
$\sigma_{gps} = 0.1$ m randomly displaces each strip laterally.

**Strip spacing** $d$ is the single design parameter that governs the trade-off between two
failure modes:

- $d$ **too large**: dose drops to near-zero in the gaps between strips → under-coverage.
- $d$ **too small**: strips overlap heavily → wasted chemical (redundant dosing), higher cost,
  potential crop burn from over-application.

**Roles**:
- **Drone**: single UAV flying pre-planned parallel strips; boom sprayer always active.
- **Field**: static $50 \times 50$ m rectangle discretised into a $0.05 \times 0.05$ m grid for
  dose accumulation.

**Objective**: Find the optimal strip spacing $d^*$ that minimises the Coefficient of Variation
(CV) of the accumulated dose across the field, thereby maximising spray uniformity. Compare the
analytical noise-free optimum with Monte Carlo simulation results that include GPS error.

**Key question**: How much does GPS positioning error $\sigma_{gps}$ widen the optimal $d$ and
degrade achievable CV? What fraction of the field area receives redundant dose exceeding
$1.5 \times$ the mean?

---

## Mathematical Model

### Gaussian Spray Dose Profile

The boom sprayer delivers a Gaussian cross-track dose at lateral offset $y$ from the strip
centreline:

$$q(y) = Q_0 \exp\!\left(-\frac{y^2}{2\sigma_s^2}\right)$$

where $Q_0$ is the peak dose rate (normalised to 1 for uniformity analysis) and
$\sigma_s = w_{spray}/4$ is chosen so that the dose falls to $e^{-2} \approx 13.5\%$ at the
nominal half-spray-width. At $w_{spray} = 2$ m:

$$\sigma_s = \frac{2.0}{4} = 0.5 \text{ m}$$

### Cumulative Field Dose

Strip $k$ ($k = 0, 1, \ldots, N-1$) is centred at cross-track position $y_k = k \cdot d$ with a
GPS displacement $\varepsilon_k \sim \mathcal{N}(0,\, \sigma_{gps}^2)$. The accumulated dose at
field position $x$ (cross-track) is the sum of contributions from all strips:

$$D(x) = \sum_{k=0}^{N-1} Q_0 \exp\!\left(-\frac{(x - k\,d + \varepsilon_k)^2}{2\sigma_s^2}\right)$$

In the noise-free case ($\varepsilon_k = 0$), $D(x)$ is a periodic Gaussian comb. The dose
profile is symmetric about the field centre so only the cross-track dimension matters for CV.

### Coverage Uniformity (Coefficient of Variation)

The coverage quality metric is the CV of the dose distribution over the field interior (excluding
a half-strip-width margin at each edge):

$$\text{CV}(d) = \frac{\sigma_D}{\mu_D}$$

where $\mu_D = \mathbb{E}[D(x)]$ and $\sigma_D = \sqrt{\mathbb{E}[(D(x)-\mu_D)^2]}$ are
computed over all field positions $x$. Perfect uniformity gives $\text{CV} = 0$; larger values
indicate under- or over-dosing heterogeneity.

### Optimal Strip Spacing

The noise-free optimal spacing $d^*$ is found by minimising CV over a search range
$d \in [0.5 \sigma_s,\; 4 \sigma_s]$:

$$d^* = \underset{d}{\arg\min}\; \text{CV}(d)$$

A practical closed-form approximation based on the Gaussian periodicity condition is:

$$d^*_{approx} = w_{spray} \cdot (1 - \rho_{opt}), \qquad \rho_{opt} = \text{SPRAY\_OVERLAP} = 0.10$$

giving $d^*_{approx} = 2.0 \times 0.90 = 1.80$ m. The numerical minimisation refines this value.

### Strip Count and Mission Path Length

For a field of cross-track width $W = 50$ m, the number of strips is:

$$N = \left\lceil \frac{W}{d} \right\rceil$$

and the total path length (along-track strips of length $L = 50$ m plus lateral turn legs) is:

$$L_{total} = N \cdot L + (N-1) \cdot d$$

### Redundant Area Fraction

A field location $x$ is considered **redundantly dosed** if it receives more than $1.5 \times$
the mean dose:

$$A_{redund} = \frac{\bigl|\{x : D(x) > 1.5\,\mu_D\}\bigr|}{W} \times 100\%$$

This metric penalises excessive overlap that wastes chemical and risks crop phytotoxicity.

### Monte Carlo GPS Error Model

For each Monte Carlo trial $m = 1, \ldots, M_{MC}$, the GPS displacement of strip $k$ is sampled
independently:

$$\varepsilon_k^{(m)} \sim \mathcal{N}(0,\, \sigma_{gps}^2), \qquad \sigma_{gps} = 0.1 \text{ m}$$

The ensemble mean and standard deviation of $\text{CV}(d)$ over $M_{MC}$ trials gives the
expected performance and uncertainty band. The optimal spacing shifts from the noise-free
$d^*$ to $d^*_{MC}$ where the mean CV is minimised.

---

## Implementation

```python
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from scipy.optimize import minimize_scalar

# ── Key constants ──────────────────────────────────────────────────────────────
FIELD_W        = 50.0     # m — cross-track field width
FIELD_L        = 50.0     # m — along-track field length
SPRAY_WIDTH    = 2.0      # m — effective boom spray width
SPRAY_OVERLAP  = 0.10     # — nominal overlap ratio (10%)
GPS_ERROR      = 0.1      # m — 1-sigma GPS positioning error
GRID_RES       = 0.05     # m — dose accumulation grid resolution
V_CRUISE       = 3.0      # m/s — drone cruise speed
N_MC           = 200      # — number of Monte Carlo trials
D_SEARCH_MIN   = 0.5      # m — minimum strip spacing to search
D_SEARCH_MAX   = 4.0      # m — maximum strip spacing to search
D_STEPS        = 200      # — number of d values in CV(d) sweep
SEED           = 42       # — RNG seed for reproducibility

# ── Derived geometry ───────────────────────────────────────────────────────────
SIGMA_S        = SPRAY_WIDTH / 4.0        # Gaussian spray std dev (m)
D_NOMINAL      = SPRAY_WIDTH * (1.0 - SPRAY_OVERLAP)   # nominal spacing (m)


def compute_dose_profile(d, field_w, sigma_s, grid_res, eps=None):
    """
    Compute 1-D cross-track dose profile D(x) for strip spacing d.

    Parameters
    ----------
    d        : float — strip centre-to-centre spacing (m)
    field_w  : float — field cross-track width (m)
    sigma_s  : float — Gaussian spray sigma (m)
    grid_res : float — grid resolution (m)
    eps      : array-like or None — per-strip GPS offsets (m); None → noise-free

    Returns
    -------
    x_grid : ndarray, shape (Nx,) — cross-track positions of grid cell centres
    dose   : ndarray, shape (Nx,) — accumulated dose at each position
    """
    n_strips = int(np.ceil(field_w / d))
    x_grid   = np.arange(grid_res / 2.0, field_w, grid_res)
    dose     = np.zeros_like(x_grid)
    for k in range(n_strips):
        centre = k * d + (eps[k] if eps is not None else 0.0)
        dose  += np.exp(-0.5 * ((x_grid - centre) / sigma_s) ** 2)
    return x_grid, dose


def compute_cv(d, field_w, sigma_s, grid_res, eps=None):
    """Return CV = sigma_D / mu_D for a given strip spacing d."""
    _, dose = compute_dose_profile(d, field_w, sigma_s, grid_res, eps)
    mu  = dose.mean()
    std = dose.std()
    if mu < 1e-9:
        return np.inf
    return std / mu


def find_optimal_spacing(field_w, sigma_s, grid_res, d_min, d_max, n_steps):
    """Sweep d over [d_min, d_max] and return (d_values, cv_values, d_star)."""
    d_vals  = np.linspace(d_min, d_max, n_steps)
    cv_vals = np.array([compute_cv(d, field_w, sigma_s, grid_res) for d in d_vals])
    idx     = np.argmin(cv_vals)
    # Refine with scalar minimisation
    res = minimize_scalar(
        lambda d: compute_cv(d, field_w, sigma_s, grid_res),
        bounds=(d_min, d_max), method='bounded'
    )
    return d_vals, cv_vals, res.x, res.fun


def monte_carlo_cv(d_vals, field_w, sigma_s, grid_res, sigma_gps, n_mc, seed):
    """
    For each d in d_vals, run n_mc Monte Carlo trials with GPS noise.
    Returns mean and std of CV across trials, shape (len(d_vals),).
    """
    rng = np.random.default_rng(seed)
    cv_mean = np.zeros(len(d_vals))
    cv_std  = np.zeros(len(d_vals))
    for i, d in enumerate(d_vals):
        n_strips = int(np.ceil(field_w / d))
        cv_trials = []
        for _ in range(n_mc):
            eps = rng.normal(0.0, sigma_gps, size=n_strips)
            cv_trials.append(compute_cv(d, field_w, sigma_s, grid_res, eps=eps))
        cv_trials  = np.array(cv_trials)
        cv_mean[i] = cv_trials.mean()
        cv_std[i]  = cv_trials.std()
    return cv_mean, cv_std


def compute_dose_heatmap(d, field_w, field_l, sigma_s, grid_res, sigma_gps, seed):
    """
    Generate a 2-D dose heatmap for strip spacing d with one Monte Carlo realisation.
    Returns (X, Y, dose_2d) where X and Y are meshgrid arrays.
    """
    rng      = np.random.default_rng(seed)
    n_strips = int(np.ceil(field_w / d))
    eps      = rng.normal(0.0, sigma_gps, size=n_strips)
    x_grid, dose_1d = compute_dose_profile(d, field_w, sigma_s, grid_res, eps=eps)
    # Along-track dimension: dose is uniform along L (boom sprays full length)
    y_grid   = np.arange(grid_res / 2.0, field_l, grid_res)
    dose_2d  = np.tile(dose_1d, (len(y_grid), 1))   # shape (Ny, Nx)
    X, Y     = np.meshgrid(x_grid, y_grid)
    return X, Y, dose_2d


def compute_metrics(d, field_w, field_l, sigma_s, grid_res, sigma_gps, n_mc, seed):
    """Compute and print key mission metrics for strip spacing d."""
    n_strips   = int(np.ceil(field_w / d))
    path_len   = n_strips * field_l + (n_strips - 1) * d
    mission_t  = path_len / V_CRUISE

    _, dose_nf = compute_dose_profile(d, field_w, sigma_s, grid_res, eps=None)
    cv_nf      = dose_nf.std() / dose_nf.mean()
    redund_pct = 100.0 * np.mean(dose_nf > 1.5 * dose_nf.mean())

    rng = np.random.default_rng(seed)
    cv_mc_list = []
    for _ in range(n_mc):
        eps = rng.normal(0.0, sigma_gps, size=n_strips)
        cv_mc_list.append(compute_cv(d, field_w, sigma_s, grid_res, eps=eps))
    cv_mc_mean = np.mean(cv_mc_list)

    print(f"Strip spacing d          : {d:.3f} m")
    print(f"Number of strips N       : {n_strips}")
    print(f"Total path length        : {path_len:.1f} m")
    print(f"Mission time             : {mission_t:.1f} s  ({mission_t/60:.2f} min)")
    print(f"CV (noise-free)          : {cv_nf:.4f}")
    print(f"CV (MC mean, GPS noise)  : {cv_mc_mean:.4f}")
    print(f"Redundant area fraction  : {redund_pct:.1f}%")
    return {
        "n_strips"   : n_strips,
        "path_len"   : path_len,
        "mission_t"  : mission_t,
        "cv_nf"      : cv_nf,
        "cv_mc_mean" : cv_mc_mean,
        "redund_pct" : redund_pct,
    }


def run_simulation():
    # ── 1. Find optimal spacing (noise-free) ──────────────────────────────────
    d_vals, cv_nf, d_star, cv_star = find_optimal_spacing(
        FIELD_W, SIGMA_S, GRID_RES, D_SEARCH_MIN, D_SEARCH_MAX, D_STEPS
    )
    print(f"Optimal spacing (noise-free): d* = {d_star:.4f} m  (CV = {cv_star:.4f})")

    # ── 2. Monte Carlo CV curve ────────────────────────────────────────────────
    # Use coarser d grid for MC to keep runtime reasonable
    d_mc    = np.linspace(D_SEARCH_MIN, D_SEARCH_MAX, 80)
    cv_mc_mean, cv_mc_std = monte_carlo_cv(
        d_mc, FIELD_W, SIGMA_S, GRID_RES, GPS_ERROR, N_MC, SEED
    )
    d_star_mc = d_mc[np.argmin(cv_mc_mean)]
    print(f"Optimal spacing (MC mean):   d*_MC = {d_star_mc:.4f} m")

    # ── 3. Dose heatmaps for three d values ───────────────────────────────────
    d_compare = [d_star * 0.70, d_star, d_star * 1.35]  # under, optimal, over
    labels    = ["Under-spaced", "Optimal d*", "Over-spaced"]

    # ── 4. Full metrics at optimal d* ─────────────────────────────────────────
    metrics = compute_metrics(
        d_star, FIELD_W, FIELD_L, SIGMA_S, GRID_RES, GPS_ERROR, N_MC, SEED
    )

    # ── 5. Build figure ────────────────────────────────────────────────────────
    fig, axes = plt.subplots(2, 3, figsize=(15, 9))
    fig.suptitle("S067 Spray Coverage Overlap Optimization", fontsize=14, fontweight="bold")

    # Row 0 — dose heatmaps for under, optimal, over spacing
    for col, (d_c, lbl) in enumerate(zip(d_compare, labels)):
        ax   = axes[0, col]
        X, Y, dose2d = compute_dose_heatmap(
            d_c, FIELD_W, FIELD_L, SIGMA_S, GRID_RES, GPS_ERROR, SEED
        )
        im = ax.pcolormesh(X, Y, dose2d, cmap="YlOrRd", shading="auto")
        fig.colorbar(im, ax=ax, label="Dose (norm.)")
        # Mark strip centrelines
        n_s = int(np.ceil(FIELD_W / d_c))
        for k in range(n_s):
            ax.axvline(k * d_c, color="cyan", linewidth=0.6, alpha=0.6, linestyle="--")
        ax.set_title(f"{lbl}\nd = {d_c:.2f} m", fontsize=10)
        ax.set_xlabel("Cross-track x (m)")
        ax.set_ylabel("Along-track y (m)")
        ax.set_aspect("equal")

    # Row 1, col 0 — CV vs d curve (noise-free + MC band)
    ax = axes[1, 0]
    ax.plot(d_vals, cv_nf, "k-", linewidth=1.8, label="Noise-free CV")
    ax.fill_between(
        d_mc,
        cv_mc_mean - cv_mc_std,
        cv_mc_mean + cv_mc_std,
        alpha=0.3, color="steelblue", label=r"MC mean $\pm$ 1$\sigma$"
    )
    ax.plot(d_mc, cv_mc_mean, "b--", linewidth=1.4, label="MC mean CV")
    ax.axvline(d_star,    color="red",    linewidth=1.5, linestyle="-",
               label=f"$d^*$ (noise-free) = {d_star:.3f} m")
    ax.axvline(d_star_mc, color="orange", linewidth=1.5, linestyle="--",
               label=f"$d^*$ (MC) = {d_star_mc:.3f} m")
    ax.axvline(D_NOMINAL, color="green",  linewidth=1.2, linestyle=":",
               label=f"Nominal = {D_NOMINAL:.2f} m")
    ax.set_xlabel("Strip spacing $d$ (m)")
    ax.set_ylabel("CV = $\\sigma_D / \\mu_D$")
    ax.set_title("Coverage Uniformity vs Strip Spacing")
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    # Row 1, col 1 — cross-track dose profiles for three spacings
    ax = axes[1, 1]
    x_1d = np.arange(GRID_RES / 2.0, FIELD_W, GRID_RES)
    colors_c = ["#e74c3c", "#2ecc71", "#3498db"]
    for d_c, lbl, col_c in zip(d_compare, labels, colors_c):
        _, dose_1d = compute_dose_profile(d_c, FIELD_W, SIGMA_S, GRID_RES, eps=None)
        dose_norm  = dose_1d / dose_1d.mean()
        ax.plot(x_1d, dose_norm, color=col_c, linewidth=1.4, label=f"{lbl} d={d_c:.2f} m")
    ax.axhline(1.0, color="black", linewidth=0.8, linestyle="--", label="Mean = 1")
    ax.axhline(1.5, color="gray",  linewidth=0.8, linestyle=":",  label="Redundancy threshold")
    ax.set_xlabel("Cross-track position x (m)")
    ax.set_ylabel("Normalised dose $D(x)/\\mu_D$")
    ax.set_title("Cross-track Dose Profiles")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # Row 1, col 2 — redundant area % and path length vs d
    ax_left  = axes[1, 2]
    ax_right = ax_left.twinx()
    d_sweep     = np.linspace(0.6, 3.5, 120)
    redund_arr  = []
    path_arr    = []
    for d_s in d_sweep:
        _, dose_s = compute_dose_profile(d_s, FIELD_W, SIGMA_S, GRID_RES)
        redund_arr.append(100.0 * np.mean(dose_s > 1.5 * dose_s.mean()))
        n_s = int(np.ceil(FIELD_W / d_s))
        path_arr.append(n_s * FIELD_L + (n_s - 1) * d_s)
    ax_left.plot(d_sweep, redund_arr, "r-",  linewidth=1.6, label="Redundant area %")
    ax_right.plot(d_sweep, path_arr,  "b--", linewidth=1.4, label="Path length (m)")
    ax_left.axvline(d_star, color="k", linewidth=1.2, linestyle="--", label=f"d* = {d_star:.3f} m")
    ax_left.set_xlabel("Strip spacing $d$ (m)")
    ax_left.set_ylabel("Redundant area (%)", color="red")
    ax_right.set_ylabel("Total path length (m)", color="blue")
    ax_left.set_title("Redundant Coverage & Path Length vs $d$")
    ax_left.tick_params(axis="y", labelcolor="red")
    ax_right.tick_params(axis="y", labelcolor="blue")
    lines_l, labels_l = ax_left.get_legend_handles_labels()
    lines_r, labels_r = ax_right.get_legend_handles_labels()
    ax_left.legend(lines_l + lines_r, labels_l + labels_r, fontsize=8)
    ax_left.grid(True, alpha=0.3)

    plt.tight_layout()
    import os
    os.makedirs("outputs/04_industrial_agriculture/s067_spray_overlap", exist_ok=True)
    fig.savefig(
        "outputs/04_industrial_agriculture/s067_spray_overlap/spray_overview.png",
        dpi=150, bbox_inches="tight"
    )
    plt.show()

    # ── 6. Animation: dose heatmap building strip by strip ────────────────────
    n_strips = int(np.ceil(FIELD_W / d_star))
    rng_anim = np.random.default_rng(SEED + 1)
    eps_anim = rng_anim.normal(0.0, GPS_ERROR, size=n_strips)
    x_grid   = np.arange(GRID_RES / 2.0, FIELD_W, GRID_RES)
    y_grid   = np.arange(GRID_RES / 2.0, FIELD_L, GRID_RES)
    X_a, Y_a = np.meshgrid(x_grid, y_grid)

    fig_a, ax_a = plt.subplots(figsize=(6, 6))
    dose_accum  = np.zeros((len(y_grid), len(x_grid)))
    im_a = ax_a.pcolormesh(X_a, Y_a, dose_accum, cmap="YlOrRd",
                           shading="auto", vmin=0, vmax=n_strips * 0.9)
    fig_a.colorbar(im_a, ax=ax_a, label="Accumulated dose (norm.)")
    strip_line,  = ax_a.plot([], [], "c--", linewidth=1.2, alpha=0.8)
    drone_point, = ax_a.plot([], [], "ko", markersize=6)
    ax_a.set_xlabel("Cross-track x (m)")
    ax_a.set_ylabel("Along-track y (m)")
    ax_a.set_xlim(0, FIELD_W)
    ax_a.set_ylim(0, FIELD_L)
    title_a = ax_a.set_title("")

    def init_anim():
        dose_accum[:] = 0.0
        im_a.set_array(dose_accum.ravel())
        strip_line.set_data([], [])
        drone_point.set_data([], [])
        return im_a, strip_line, drone_point

    def update_anim(frame):
        k = frame
        centre = k * d_star + eps_anim[k]
        strip_dose_1d = np.exp(-0.5 * ((x_grid - centre) / SIGMA_S) ** 2)
        dose_accum += np.tile(strip_dose_1d, (len(y_grid), 1))
        im_a.set_array(dose_accum.ravel())
        # Draw current strip centreline
        strip_line.set_data([centre, centre], [0, FIELD_L])
        # Show drone at strip midpoint (along-track)
        drone_y = FIELD_L / 2.0
        drone_point.set_data([centre], [drone_y])
        cv_now = dose_accum.std() / max(dose_accum.mean(), 1e-9)
        title_a.set_text(
            f"Strip {k+1}/{n_strips}  |  d* = {d_star:.3f} m  |  CV = {cv_now:.4f}"
        )
        return im_a, strip_line, drone_point, title_a

    anim = animation.FuncAnimation(
        fig_a, update_anim, frames=n_strips,
        init_func=init_anim, blit=False, interval=400, repeat=False
    )
    anim.save(
        "outputs/04_industrial_agriculture/s067_spray_overlap/spray_animation.gif",
        writer="pillow", fps=2
    )
    plt.show()

    return {
        "d_star"      : d_star,
        "cv_star"     : cv_star,
        "d_star_mc"   : d_star_mc,
        "metrics"     : metrics,
        "d_vals"      : d_vals,
        "cv_nf"       : cv_nf,
        "cv_mc_mean"  : cv_mc_mean,
        "cv_mc_std"   : cv_mc_std,
    }


if __name__ == "__main__":
    results = run_simulation()
```

---

## Key Parameters

| Parameter | Symbol | Value |
|-----------|--------|-------|
| Field size | $W \times L$ | $50 \times 50$ m |
| Spray width | $w_{spray}$ | 2.0 m |
| Gaussian spray sigma | $\sigma_s = w_{spray}/4$ | 0.5 m |
| Nominal overlap ratio | $\rho_{opt}$ | 10% |
| Nominal strip spacing | $d_{nom} = w_{spray}(1-\rho_{opt})$ | 1.80 m |
| GPS positioning error | $\sigma_{gps}$ | 0.1 m |
| Dose grid resolution | — | 0.05 × 0.05 m |
| Drone cruise speed | $v$ | 3.0 m/s |
| Monte Carlo trials | $M_{MC}$ | 200 |
| Strip spacing search range | $[d_{min}, d_{max}]$ | [0.5, 4.0] m |
| Redundancy threshold | — | $1.5 \times \mu_D$ |
| RNG seed | — | 42 |

---

## Expected Output

- **Dose heatmaps** (3 panels): top-down 2-D dose maps for under-spaced, optimal, and over-spaced
  configurations ($d = 0.70\,d^*$, $d^*$, $1.35\,d^*$); strip centrelines drawn as dashed cyan
  lines; colour scale in normalised dose units; visual gap formation visible for under-spacing and
  hot-spot banding for over-spacing.
- **CV vs strip spacing curve**: noise-free CV curve (black solid) with Monte Carlo mean ± 1σ band
  (blue shading); vertical markers for noise-free $d^*$ (red), Monte Carlo $d^*_{MC}$ (orange),
  and nominal design spacing (green dashed); minimum CV clearly visible as a well-defined trough.
- **Cross-track dose profiles**: normalised $D(x)/\mu_D$ for all three spacing configurations on a
  single axis; horizontal reference lines at $D = 1$ (mean) and $D = 1.5$ (redundancy threshold);
  the optimal profile is nearly flat while under-spacing shows deep gaps and over-spacing shows
  pronounced peaks.
- **Redundant area and path length vs d**: dual-axis plot with redundant area fraction (red, left
  axis) and total path length (blue dashed, right axis) as functions of $d$; illustrates the
  cost-quality trade-off and marks the optimal spacing.
- **Animation (GIF)**: real-time top-down view of the dose field accumulating strip by strip at
  $d = d^*$; strip centreline appears in cyan; drone position marked; title updates with current CV
  after each strip is added; reveals how uniformity converges as coverage builds.
- **Printed metrics** (console): optimal spacing $d^*$, strip count $N$, path length, mission
  time, noise-free CV, Monte Carlo mean CV, and redundant area fraction.

---

## Extensions

1. **Non-uniform boom profile**: replace the Gaussian model with a measured flat-top or bimodal
   spray profile from a real nozzle characterisation; re-solve for $d^*$ numerically and assess
   how profile shape shifts the optimal spacing and achievable CV floor.
2. **Wind drift correction**: add a constant crosswind $v_w$ that laterally displaces the spray
   plume by $\Delta y = v_w \cdot (h_{boom} / v_{descent})$; fold this bias into the effective
   dose model and find the corrected strip spacing $d^*_{wind}$ as a function of wind speed.
3. **Variable-rate application zones**: divide the field into prescription zones with different
   target dose levels $Q^*_{zone}$; solve for zone-specific strip spacings and compute the
   minimum number of passes that satisfies all zone constraints simultaneously.
4. **GPS RTK vs standard**: compare CV achievable with $\sigma_{gps} = 0.01$ m (RTK) vs
   $0.1$ m (standard) vs $0.5$ m (degraded); quantify the CV improvement and identify the
   break-even spacing below which RTK precision yields diminishing returns.
5. **Multi-pass headland turns**: model the turning manoeuvre at each strip end; account for the
   spray being active during the turn arc and compute the additional dose deposited on headland
   cells; assess whether headland exclusion zones are necessary.

---

## Related Scenarios

- Prerequisites: [S048 Full-Area Coverage Scan](../../03_environmental_sar/S048_lawnmower.md) (boustrophedon strip planning fundamentals), [S063 Crop Row Following](S063_crop_row_following.md) (agricultural strip flight basics)
- Follow-ups: [S068 Multi-Drone Field Coverage](S068_multi_drone_field_coverage.md) (parallel strip assignment across a fleet), [S073 Precision Fertiliser Dosing](S073_precision_fertiliser_dosing.md) (variable-rate application with prescription maps)
- Algorithmic cross-reference: [S052 Glacier Melt Area Measurement](../../03_environmental_sar/S052_glacier.md) (same strip-spacing overlap trade-off in photogrammetric surveys)
