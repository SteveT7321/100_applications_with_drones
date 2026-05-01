# S046 Multi-Drone 3D Trilateration

**Domain**: Environmental & SAR | **Difficulty**: ⭐⭐ | **Status**: `[x]` Complete

---

## Problem Definition

**Setup**: A survivor or emergency beacon is located at an unknown 3D position
$\mathbf{x}^* = (x^*, y^*, z^*)$ inside a search area of $200 \times 200 \times 30$ m. Four
drones hover at known, fixed positions at different altitudes and each measures the slant range to
the beacon using an on-board RF or acoustic ranging unit. All measurements are corrupted by
zero-mean Gaussian noise. The mission is to fuse the four noisy range measurements into the best
possible estimate $\hat{\mathbf{x}}$ of the beacon's 3D position and to characterise the
localisation uncertainty via the Geometric Dilution of Precision (GDOP).

**Roles**:
- **Drones** ($N = 4$): stationary observers hovering at known anchor positions
  $\mathbf{p}_1, \ldots, \mathbf{p}_4$; each broadcasts one range measurement per epoch.
- **Beacon / Survivor**: passive transmitter at unknown position $\mathbf{x}^*$; does not
  participate in the estimation process.
- **Ground station**: receives all four range measurements and runs the least-squares solver.

**Objective**: Estimate $\hat{\mathbf{x}}$ from $N = 4$ noisy range measurements using two
methods:

1. **Linear least squares (LLS)** — algebraically linearise the range equations by differencing
   pairs, reducing the problem to a single linear system $\mathbf{A}\hat{\mathbf{x}} = \mathbf{b}$
   solved in closed form.
2. **Nonlinear least squares via Gauss-Newton (GN)** — iterate from an initial guess, refining
   $\hat{\mathbf{x}}$ by first-order Taylor expansion until convergence.

Compare the two methods across noise levels, initial-guess quality, and drone geometry (well-spread
vs. coplanar configurations).

---

## Mathematical Model

### Range Measurement Model

The true slant range from drone $i$ at anchor position $\mathbf{p}_i \in \mathbb{R}^3$ to the
beacon at $\mathbf{x}^* \in \mathbb{R}^3$ is:

$$\rho_i^* = \|\mathbf{p}_i - \mathbf{x}^*\|_2 = \sqrt{(p_{ix} - x^*)^2 + (p_{iy} - y^*)^2 + (p_{iz} - z^*)^2}$$

The measured range includes additive Gaussian noise:

$$r_i = \rho_i^* + \epsilon_i, \qquad \epsilon_i \sim \mathcal{N}(0,\, \sigma_r^2)$$

The nonlinear least-squares cost to minimise is:

$$J(\mathbf{x}) = \sum_{i=1}^{N} \bigl(r_i - \|\mathbf{p}_i - \mathbf{x}\|\bigr)^2$$

### Linear Least Squares (Range-Differencing)

Subtract the equation for drone $N$ from each drone $i \in \{1, \ldots, N-1\}$:

$$r_i^2 - r_N^2 = \|\mathbf{p}_N\|^2 - \|\mathbf{p}_i\|^2 + 2(\mathbf{p}_i - \mathbf{p}_N)^T \mathbf{x}$$

This eliminates the quadratic $\|\mathbf{x}\|^2$ term. Collecting all $N-1$ differences into a
matrix system:

$$\mathbf{A} \mathbf{x} = \mathbf{b}$$

where each row $i$ of $\mathbf{A} \in \mathbb{R}^{(N-1) \times 3}$ and $\mathbf{b} \in \mathbb{R}^{N-1}$ are:

$$\mathbf{A}_i = 2(\mathbf{p}_i - \mathbf{p}_N)^T$$

$$b_i = r_i^2 - r_N^2 - \|\mathbf{p}_i\|^2 + \|\mathbf{p}_N\|^2$$

The closed-form least-squares solution (normal equations):

$$\hat{\mathbf{x}}_{LLS} = (\mathbf{A}^T \mathbf{A})^{-1} \mathbf{A}^T \mathbf{b}$$

Note: this linearisation is exact only when range measurements are noise-free; noise in $r_i$ and
$r_N$ couples into both $\mathbf{A}$ and $\mathbf{b}$, introducing bias at high noise levels.

### Gauss-Newton Nonlinear Least Squares

Define the residual vector $\mathbf{f}(\mathbf{x}) \in \mathbb{R}^N$ with components:

$$f_i(\mathbf{x}) = r_i - \|\mathbf{p}_i - \mathbf{x}\|$$

The Jacobian $\mathbf{H} \in \mathbb{R}^{N \times 3}$ is the matrix of partial derivatives
$H_{ij} = \partial f_i / \partial x_j$:

$$\mathbf{H}_i = \frac{\partial f_i}{\partial \mathbf{x}} = \frac{\mathbf{p}_i - \mathbf{x}}{\|\mathbf{p}_i - \mathbf{x}\|}$$

(unit vector pointing from beacon estimate to drone $i$). At each iteration $k$, the Gauss-Newton
update is:

$$\Delta \mathbf{x}_k = (\mathbf{H}_k^T \mathbf{H}_k)^{-1} \mathbf{H}_k^T \mathbf{f}(\mathbf{x}_k)$$

$$\mathbf{x}_{k+1} = \mathbf{x}_k + \Delta \mathbf{x}_k$$

Convergence is declared when $\|\Delta \mathbf{x}_k\|_2 < \epsilon_{tol} = 10^{-4}$ m or after a
maximum of $K_{max} = 100$ iterations.

### Geometric Dilution of Precision (GDOP)

The GDOP quantifies how the geometric arrangement of the four drones amplifies range measurement
noise into position error. Given the Jacobian $\mathbf{H}$ evaluated at the true position:

$$\text{GDOP} = \sqrt{\text{tr}\!\left[(\mathbf{H}^T \mathbf{H})^{-1}\right]}$$

The 3D position error covariance (for isotropic range noise $\sigma_r$) is:

$$\mathbf{\Sigma}_{\hat{\mathbf{x}}} = \sigma_r^2 \, (\mathbf{H}^T \mathbf{H})^{-1}$$

The 3D RMS position error bound is therefore:

$$\sigma_{pos} = \sigma_r \cdot \text{GDOP}$$

A low GDOP (approaching 1.0) is achieved when the drones subtend a large solid angle as seen
from the beacon. A coplanar arrangement (all drones at the same altitude) degrades vertical
GDOP severely while leaving horizontal GDOP acceptable.

### Optimal Drone Placement to Minimise GDOP

For a fixed set of hover radii $\{R_{xy}, z_{max}\}$, GDOP is minimised by maximising
$\det(\mathbf{H}^T \mathbf{H})$. An effective heuristic: place three drones at azimuth offsets of
$120^\circ$ on a circle of radius $R_{xy}$ at altitude $z_{low}$, and one drone directly overhead
at altitude $z_{high}$:

$$\mathbf{p}_i = \begin{cases}
\bigl(R_{xy}\cos(2\pi(i-1)/3),\; R_{xy}\sin(2\pi(i-1)/3),\; z_{low}\bigr) & i = 1,2,3 \\
\bigl(0,\; 0,\; z_{high}\bigr) & i = 4
\end{cases}$$

This configuration provides good coverage of all three axes and is compared against a flat
(coplanar, constant-altitude) arrangement in simulation.

### Cramer-Rao Lower Bound

The Fisher information matrix for the unbiased estimator is:

$$\mathbf{I}(\mathbf{x}^*) = \frac{1}{\sigma_r^2} \mathbf{H}^T \mathbf{H}$$

The CRLB on each position component satisfies:

$$\text{Var}(\hat{x}_j) \geq \left[(\mathbf{H}^T \mathbf{H})^{-1}\right]_{jj} \cdot \sigma_r^2$$

---

## Implementation

```python
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Key constants
N_DRONES    = 4
SIGMA_R     = 0.5      # m — range measurement noise std dev
N_TRIALS    = 500      # Monte Carlo trials per configuration
EPSILON_TOL = 1e-4     # m — Gauss-Newton convergence threshold
K_MAX       = 100      # max Gauss-Newton iterations
R_XY        = 80.0     # m — horizontal spread radius
Z_LOW       = 5.0      # m — low-altitude drone tier
Z_HIGH      = 25.0     # m — high-altitude drone (overhead)

# True beacon position
X_TRUE = np.array([30.0, -20.0, 2.0])  # x, y, z in metres

# Well-spread configuration: 3 drones at 120° azimuth + 1 overhead
ANCHORS_GOOD = np.array([
    [ R_XY,              0.0,              Z_LOW],
    [-R_XY / 2,  R_XY * np.sqrt(3) / 2,  Z_LOW],
    [-R_XY / 2, -R_XY * np.sqrt(3) / 2,  Z_LOW],
    [ 0.0,               0.0,             Z_HIGH],
])

# Poor configuration: all 4 drones coplanar at same altitude
ANCHORS_FLAT = np.array([
    [ R_XY,       0.0,  Z_LOW],
    [-R_XY,       0.0,  Z_LOW],
    [  0.0,  R_XY,      Z_LOW],
    [  0.0, -R_XY,      Z_LOW],
])

def measure_ranges(anchors, x_true, sigma_r, rng):
    """Simulate noisy range measurements from N drones to the beacon."""
    diffs = anchors - x_true          # (N, 3)
    true_ranges = np.linalg.norm(diffs, axis=1)
    return true_ranges + rng.normal(0, sigma_r, size=N_DRONES)

def solve_lls(anchors, ranges):
    """Linear least squares via range-differencing (anchor N as reference)."""
    ref = anchors[-1]
    r_ref = ranges[-1]
    A = 2 * (anchors[:-1] - ref)
    b = (ranges[:-1]**2 - r_ref**2
         - np.sum(anchors[:-1]**2, axis=1)
         + np.dot(ref, ref))
    # Normal equations: (A^T A) x = A^T b
    x_hat, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
    return x_hat

def compute_jacobian(anchors, x):
    """Jacobian H[i,:] = unit vector from x towards anchor i."""
    diffs = anchors - x               # (N, 3)
    norms = np.linalg.norm(diffs, axis=1, keepdims=True)
    return diffs / (norms + 1e-12)    # (N, 3)

def solve_gauss_newton(anchors, ranges, x_init):
    """Gauss-Newton iteration for nonlinear range least squares."""
    x = x_init.copy()
    for _ in range(K_MAX):
        pred_ranges = np.linalg.norm(anchors - x, axis=1)
        residuals   = ranges - pred_ranges               # f(x)
        H           = compute_jacobian(anchors, x)       # (N, 3)
        HtH         = H.T @ H
        delta       = np.linalg.solve(HtH, H.T @ residuals)
        x           = x + delta
        if np.linalg.norm(delta) < EPSILON_TOL:
            break
    return x

def compute_gdop(anchors, x):
    """GDOP = sqrt(trace((H^T H)^{-1})) at position x."""
    H   = compute_jacobian(anchors, x)
    HtH = H.T @ H
    try:
        cov = np.linalg.inv(HtH)
        return np.sqrt(np.trace(cov))
    except np.linalg.LinAlgError:
        return np.inf

def run_monte_carlo(anchors, sigma_r, n_trials, x_true):
    """Run Monte Carlo comparison of LLS vs Gauss-Newton over many noise realisations."""
    rng = np.random.default_rng(42)
    errors_lls, errors_gn = [], []
    x_init = x_true + rng.normal(0, 5.0, size=3)   # perturbed initial guess

    for _ in range(n_trials):
        ranges       = measure_ranges(anchors, x_true, sigma_r, rng)
        x_lls        = solve_lls(anchors, ranges)
        x_gn         = solve_gauss_newton(anchors, ranges, x_init)
        errors_lls.append(np.linalg.norm(x_lls - x_true))
        errors_gn.append(np.linalg.norm(x_gn  - x_true))

    return np.array(errors_lls), np.array(errors_gn)

def run_simulation():
    """Main entry point: run all comparisons and produce figures."""
    rng = np.random.default_rng(0)

    gdop_good = compute_gdop(ANCHORS_GOOD, X_TRUE)
    gdop_flat = compute_gdop(ANCHORS_FLAT, X_TRUE)

    sigma_levels = np.linspace(0.1, 3.0, 15)
    rmse_lls_good, rmse_gn_good = [], []
    rmse_lls_flat, rmse_gn_flat = [], []

    for sigma in sigma_levels:
        e_lls, e_gn = run_monte_carlo(ANCHORS_GOOD, sigma, N_TRIALS, X_TRUE)
        rmse_lls_good.append(np.sqrt(np.mean(e_lls**2)))
        rmse_gn_good.append(np.sqrt(np.mean(e_gn**2)))

        e_lls, e_gn = run_monte_carlo(ANCHORS_FLAT, sigma, N_TRIALS, X_TRUE)
        rmse_lls_flat.append(np.sqrt(np.mean(e_lls**2)))
        rmse_gn_flat.append(np.sqrt(np.mean(e_gn**2)))

    return {
        "sigma_levels": sigma_levels,
        "rmse_lls_good": rmse_lls_good,
        "rmse_gn_good":  rmse_gn_good,
        "rmse_lls_flat": rmse_lls_flat,
        "rmse_gn_flat":  rmse_gn_flat,
        "gdop_good": gdop_good,
        "gdop_flat": gdop_flat,
    }
```

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Number of drones $N$ | 4 |
| Range noise std dev $\sigma_r$ | 0.5 m (baseline) |
| Horizontal spread radius $R_{xy}$ | 80 m |
| Low-altitude tier $z_{low}$ | 5 m |
| High-altitude (overhead) $z_{high}$ | 25 m |
| True beacon position $\mathbf{x}^*$ | (30, -20, 2) m |
| Gauss-Newton tolerance $\epsilon_{tol}$ | $10^{-4}$ m |
| Max GN iterations $K_{max}$ | 100 |
| Monte Carlo trials per config | 500 |
| Noise sweep range | 0.1 – 3.0 m |
| GDOP (well-spread config) | $\approx 1.4$ |
| GDOP (coplanar config) | $\approx 4.2$ |
| CRLB 3D RMS (well-spread, $\sigma_r=0.5$ m) | $\approx 0.7$ m |

---

## Expected Output

- **3D anchor geometry plot**: two side-by-side 3D views showing the well-spread configuration
  (three low-altitude drones at 120° + one overhead) vs. the coplanar configuration; beacon
  marked as a green star; lines drawn from each drone to the beacon.
- **RMSE vs. noise level curves**: four curves on one plot (LLS good-geometry, GN good-geometry,
  LLS flat-geometry, GN flat-geometry) against $\sigma_r \in [0.1, 3.0]$ m; CRLB $\cdot$ GDOP
  envelope overlaid as a shaded region.
- **GDOP heatmap**: 2D horizontal slice ($z = 2$ m) showing GDOP as a function of beacon $(x, y)$
  position for the well-spread configuration; low-GDOP zone highlighted in green.
- **Gauss-Newton convergence trace**: $\|\Delta \mathbf{x}_k\|$ vs. iteration number for 10
  representative noise realisations; comparison between a good initial guess (within 5 m of truth)
  and a poor one (50 m offset).
- **Error histogram**: distribution of 3D position errors across 500 Monte Carlo trials for both
  LLS and GN at $\sigma_r = 0.5$ m; Gaussian fit and mean/std annotations.
- **Vertical error breakdown**: separate histograms for horizontal error $\sqrt{\Delta x^2 + \Delta y^2}$
  and vertical error $|\Delta z|$ illustrating how coplanar geometry degrades altitude estimation
  while leaving horizontal estimation relatively unaffected.

---

## Extensions

1. **Weighted least squares**: assign measurement weights $w_i = 1 / \sigma_{r,i}^2$ when drones
   have heterogeneous sensor quality or different distances to the beacon; derive the weighted
   normal equations $\hat{\mathbf{x}} = (\mathbf{A}^T \mathbf{W} \mathbf{A})^{-1} \mathbf{A}^T
   \mathbf{W} \mathbf{b}$ and show variance reduction.
2. **Sequential (online) update**: process range measurements one drone at a time using a
   recursive least-squares (RLS) or extended Kalman filter (EKF) update to support real-time
   beacon tracking as the beacon moves slowly.
3. **Optimal hover placement via GDOP minimisation**: formulate drone placement as a continuous
   optimisation problem, minimise GDOP subject to altitude and collision-avoidance constraints,
   and solve with scipy.optimize; compare the optimal pattern to the heuristic 120° layout.
4. **Time-difference of arrival (TDOA)**: replace one-way ranging with TDOA measurements
   $\tau_{ij} = (r_i - r_j)/c$; re-derive the hyperbolic least-squares system and compare
   localisation accuracy to pure range measurements under the same noise budget.
5. **Moving beacon tracking with Gauss-Newton EKF**: augment the state vector with beacon velocity
   $[\mathbf{x}, \dot{\mathbf{x}}]$ and run an iterated EKF; evaluate tracking lag and RMSE
   against a survivor walking at 1 m/s.

---

## Related Scenarios

- Prerequisites: [S041 Wildfire Boundary Scan](S041_wildfire_boundary_scan.md), [S042 Missing Person Search](S042_missing_person_search.md)
- Follow-ups: [S047 Signal Relay Enhancement](S047_signal_relay_enhancement.md), [S050 Swarm SLAM](S050_swarm_slam.md)
- Algorithmic cross-reference: [S007 Blind Pursuit under Jamming](../01_pursuit_evasion/S007_blind_pursuit_jamming.md) (noisy state estimation), [S008 Stochastic Pursuit](../01_pursuit_evasion/S008_stochastic_pursuit.md) (Kalman filtering)
