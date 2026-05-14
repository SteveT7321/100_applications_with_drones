# S059 Underwater Target Sonar Marking (Surface Cooperative Localization)

**Domain**: Environmental Monitoring & SAR | **Difficulty**: ⭐⭐⭐ | **Status**: `[ ]` Not Started

---

## Problem Definition

**Setup**: An underwater object — a sunken wreck or a silent submarine — periodically emits a
broadband acoustic ping at interval $T_{ping} = 5$ s. Four drones hover over the ocean surface at
$z = 0$ m and each drops a sonar buoy. The buoys are passive hydrophone receivers: each records
the arrival time of the same ping, but the ping reaches each buoy at a slightly different instant
because the buoys are at different horizontal positions. These arrival-time differences (TDOA —
Time Difference Of Arrival) define a set of hyperboloids in 3D space; the intersection of three
independent hyperboloids yields the target's 3D position $(x_T, y_T, z_T)$ with $z_T < 0$.

The target position is estimated by fitting a Gauss-Newton nonlinear least-squares solver to the
four noisy TDOA measurements. The additional mission objective is to find the buoy deployment
layout that minimises the TDOA-CRLB (Cramer-Rao Lower Bound on position variance), so that the
drones drop their buoys in positions that are maximally informative before the first ping arrives.

**Roles**:
- **Drones** ($N = 4$): each drone hovers at surface position $\mathbf{p}_i = (x_i, y_i, 0)$,
  $i = 1, \ldots, 4$, and deploys one sonar buoy directly below its hover point.
- **Sonar buoys** ($N = 4$): passive hydrophone receivers that float at $z = 0$ m; each buoy
  records the absolute reception time $t_i^{rec}$ of each ping with clock noise
  $\epsilon_i \sim \mathcal{N}(0, \sigma_t^2)$.
- **Underwater target**: stationary acoustic source at unknown position
  $\mathbf{p}_T = (x_T, y_T, z_T)$ with $z_T \in [-200, -10]$ m; emits a recognisable ping
  every $T_{ping}$ seconds.
- **Ground station**: receives all four reception timestamps via the drone data-relay link and
  runs the TDOA localisation algorithm.

**Objective**: Jointly (1) optimise the four buoy deployment positions to minimise the CRLB on
the 3D target-position estimate, and (2) localise the target from the resulting TDOA measurements
using Gauss-Newton least squares. Compare three buoy layout strategies:

1. **Square grid** — four buoys at the corners of a $200 \times 200$ m square centred on the
   search area.
2. **Tetrahedral projection** — three buoys at $120^\circ$ azimuth spacing on a circle of radius
   $R$, plus one buoy at the circle centre.
3. **CRLB-optimal** — positions found by minimising $\text{tr}(\mathbf{F}^{-1})$ via
   `scipy.optimize.minimize` subject to a maximum inter-buoy-spread constraint.

---

## Mathematical Model

### TDOA Measurement Model

Let buoy $i$ be located at $\mathbf{p}_i \in \mathbb{R}^3$ (with $p_{iz} = 0$). The one-way
acoustic travel time from target $\mathbf{p}_T$ to buoy $i$ is:

$$\tau_i = \frac{r_i}{c_s}, \qquad r_i = \|\mathbf{p}_i - \mathbf{p}_T\|_2$$

where $c_s = 1500$ m/s is the speed of sound in seawater. Choosing buoy 1 as the reference, the
TDOA between buoys $i$ and $1$ is:

$$\Delta t_{i1} = \tau_i - \tau_1 = \frac{r_i - r_1}{c_s}, \qquad i = 2, 3, 4$$

The measured TDOA is corrupted by independent Gaussian timing noise:

$$\Delta \hat{t}_{i1} = \Delta t_{i1} + \nu_{i1}, \qquad
\nu_{i1} = \epsilon_i - \epsilon_1 \sim \mathcal{N}(0,\, 2\sigma_t^2)$$

where $\sigma_t$ is the per-buoy clock noise standard deviation. Equivalently, in terms of
range differences:

$$\Delta \hat{d}_{i1} = c_s \cdot \Delta \hat{t}_{i1} = (r_i - r_1) + c_s \nu_{i1}$$

The noise standard deviation in range-difference units is therefore $\sigma_d = c_s \sqrt{2} \, \sigma_t$.

### TDOA Hyperbola (Hyperboloid) Equation

For a fixed pair $(i, 1)$, the locus of all target positions $\mathbf{p}$ satisfying
$r_i - r_1 = c_s \cdot \Delta \hat{t}_{i1}$ is a two-sheet hyperboloid with foci at
$\mathbf{p}_i$ and $\mathbf{p}_1$:

$$\|\mathbf{p} - \mathbf{p}_i\| - \|\mathbf{p} - \mathbf{p}_1\| = c_s \cdot \Delta \hat{t}_{i1}$$

Three such hyperboloids (pairs $(2,1)$, $(3,1)$, $(4,1)$) have a unique intersection that
yields $\hat{\mathbf{p}}_T$ when the buoy geometry is non-degenerate.

### Residual Vector and Jacobian

Define the TDOA residual vector $\mathbf{f}(\mathbf{p}) \in \mathbb{R}^{N-1}$ with components:

$$f_i(\mathbf{p}) = \Delta \hat{d}_{i1} - \bigl(\|\mathbf{p}_i - \mathbf{p}\| - \|\mathbf{p}_1 - \mathbf{p}\|\bigr), \qquad i = 2, 3, 4$$

The Jacobian $\mathbf{H} \in \mathbb{R}^{(N-1) \times 3}$ is:

$$H_{ij} = \frac{\partial f_i}{\partial p_j}
  = \frac{(p_{1j} - p_j)}{\|\mathbf{p}_1 - \mathbf{p}\|}
  - \frac{(p_{ij} - p_j)}{\|\mathbf{p}_i - \mathbf{p}\|}$$

That is, $H_i = \hat{\mathbf{u}}_{1 \leftarrow T} - \hat{\mathbf{u}}_{i \leftarrow T}$, where
$\hat{\mathbf{u}}_{k \leftarrow T}$ is the unit vector pointing from the target estimate toward
buoy $k$.

### Gauss-Newton Localisation

Starting from an initial guess $\mathbf{p}^{(0)}$, iterate:

$$\Delta \mathbf{p}^{(k)} = \bigl(\mathbf{H}_k^T \mathbf{H}_k\bigr)^{-1} \mathbf{H}_k^T \mathbf{f}(\mathbf{p}^{(k)})$$

$$\mathbf{p}^{(k+1)} = \mathbf{p}^{(k)} + \Delta \mathbf{p}^{(k)}$$

Convergence criterion: $\|\Delta \mathbf{p}^{(k)}\|_2 < \epsilon_{tol} = 10^{-3}$ m, or
$k = K_{max} = 200$ iterations. The final estimate is $\hat{\mathbf{p}}_T = \mathbf{p}^{(k+1)}$.

### Fisher Information Matrix and CRLB

The measurement covariance matrix for the $(N-1) = 3$ independent TDOA pairs is:

$$\boldsymbol{\Sigma} = 2\sigma_t^2 c_s^2 \, \mathbf{I}_{3 \times 3} = \sigma_d^2 \, \mathbf{I}$$

The Fisher information matrix (FIM) for the target position $\mathbf{p}_T$ is:

$$\mathbf{F}(\mathbf{p}_T) = \mathbf{H}^T \boldsymbol{\Sigma}^{-1} \mathbf{H}
  = \frac{1}{\sigma_d^2} \mathbf{H}^T \mathbf{H}$$

The CRLB states that any unbiased estimator $\hat{\mathbf{p}}_T$ satisfies:

$$\text{Cov}(\hat{\mathbf{p}}_T) \succeq \mathbf{F}^{-1}$$

The scalar CRLB objective to minimise (A-optimality criterion) over buoy positions is:

$$\mathcal{C}\!\left(\{\mathbf{p}_i\}\right) = \text{tr}\!\left(\mathbf{F}^{-1}\right)
  = \sigma_d^2 \, \text{tr}\!\left[(\mathbf{H}^T \mathbf{H})^{-1}\right]$$

The predicted 3D RMS localisation error is bounded below by:

$$\sigma_{3D} \geq \sqrt{\text{tr}(\mathbf{F}^{-1})} = \sigma_d \cdot \sqrt{\text{tr}\!\left[(\mathbf{H}^T \mathbf{H})^{-1}\right]}$$

### CRLB-Optimal Buoy Placement

Parameterise buoy positions as $\mathbf{p}_i = (x_i, y_i, 0)$ for $i = 1, \ldots, 4$. The
optimisation problem is:

$$\min_{\{(x_i, y_i)\}} \; \text{tr}\!\left[(\mathbf{H}^T \mathbf{H})^{-1}\right]$$

$$\text{subject to:} \quad \|(x_i, y_i) - (x_j, y_j)\| \leq D_{max} \quad \forall\, i \neq j$$

$$\quad (x_i, y_i) \in \mathcal{B} \quad \text{(operating bounding box)}$$

This non-convex continuous optimisation is solved with `scipy.optimize.minimize` using the
L-BFGS-B method from a set of multiple random restarts to avoid local minima. The gradient is
computed by automatic finite differences. The regulariser $\delta = 10^{-6} \mathbf{I}$ is added
to $\mathbf{H}^T \mathbf{H}$ before inversion to handle near-singular geometries.

### Horizontal GDOP Analogue

By analogy with GPS GDOP, define the TDOA-GDOP as:

$$\text{GDOP}_{TDOA} = \frac{\sqrt{\text{tr}(\mathbf{F}^{-1})}}{\sigma_d}
  = \sqrt{\text{tr}\!\left[(\mathbf{H}^T \mathbf{H})^{-1}\right]}$$

A low GDOP indicates a buoy geometry that amplifies timing noise minimally into position error.
Degenerate geometries (all buoys collinear, or symmetric configurations where the target lies
on a symmetry plane) yield infinite or very large GDOP.

---

## Implementation

```python
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.optimize import minimize

# Key constants
N_BUOYS      = 4
C_SOUND      = 1500.0       # m/s — speed of sound in seawater
SIGMA_T      = 1e-4         # s  — per-buoy clock noise std dev
SIGMA_D      = C_SOUND * np.sqrt(2) * SIGMA_T   # m — TDOA noise in range units
T_PING       = 5.0          # s  — ping interval
EPSILON_TOL  = 1e-3         # m  — Gauss-Newton convergence threshold
K_MAX        = 200          # max Gauss-Newton iterations
D_MAX        = 500.0        # m  — max buoy spread (operating constraint)
N_TRIALS     = 1000         # Monte Carlo trials
N_RESTARTS   = 20           # CRLB optimisation random restarts
REG          = 1e-6         # regularisation for FIM inversion

# True target position (unknown to estimator)
P_TARGET = np.array([40.0, -30.0, -120.0])   # x, y, z (z < 0 = underwater)

# --- Buoy layout strategies ---

def layout_square(half_side=100.0):
    """Four buoys at corners of a square centred at origin."""
    s = half_side
    return np.array([[ s,  s, 0.0],
                     [-s,  s, 0.0],
                     [-s, -s, 0.0],
                     [ s, -s, 0.0]])

def layout_tetrahedral(radius=120.0):
    """Three buoys at 120° spacing + one at centre."""
    angles = np.deg2rad([0.0, 120.0, 240.0])
    buoys = np.array([[radius * np.cos(a), radius * np.sin(a), 0.0]
                      for a in angles])
    centre = np.array([[0.0, 0.0, 0.0]])
    return np.vstack([buoys, centre])

# --- TDOA physics ---

def tdoa_measurements(buoys, p_target, sigma_t, rng):
    """
    Simulate noisy TDOA measurements (in seconds) relative to buoy 0.
    Returns array of shape (N-1,): Delta_t_{i0} for i = 1, 2, 3.
    """
    ranges = np.linalg.norm(buoys - p_target, axis=1)      # (N,)
    true_tdoa = (ranges[1:] - ranges[0]) / C_SOUND         # (N-1,)
    noise = rng.normal(0.0, np.sqrt(2) * sigma_t, size=N_BUOYS - 1)
    return true_tdoa + noise

def tdoa_residuals(p, buoys, delta_d_meas):
    """
    Residual f_i = delta_d_meas_i - (r_i - r_0) for i = 1, 2, 3.
    p: current target position estimate (3,)
    delta_d_meas: measured range-difference (N-1,)
    """
    r = np.linalg.norm(buoys - p, axis=1)                  # (N,)
    pred = r[1:] - r[0]                                     # (N-1,)
    return delta_d_meas - pred

def tdoa_jacobian(p, buoys):
    """
    Jacobian H[i, j] = d f_i / d p_j.
    H_i = u_{0<-T} - u_{i<-T}, unit vectors from estimate to buoys.
    """
    diffs = buoys - p                                       # (N, 3)
    norms = np.linalg.norm(diffs, axis=1, keepdims=True)   # (N, 1)
    unit  = diffs / (norms + 1e-12)                        # (N, 3)
    H = unit[0:1, :] - unit[1:, :]                        # (N-1, 3)
    return H

# --- Gauss-Newton localisation ---

def gauss_newton_tdoa(buoys, delta_d_meas, p_init):
    """
    Gauss-Newton solver for TDOA-based 3D localisation.
    delta_d_meas: range-difference measurements (N-1,) = c_s * TDOA (s)
    """
    p = p_init.copy()
    for _ in range(K_MAX):
        f = tdoa_residuals(p, buoys, delta_d_meas)
        H = tdoa_jacobian(p, buoys)
        HtH = H.T @ H
        try:
            delta = np.linalg.solve(HtH, H.T @ f)
        except np.linalg.LinAlgError:
            break
        p = p + delta
        if np.linalg.norm(delta) < EPSILON_TOL:
            break
    return p

# --- Fisher Information and CRLB ---

def fisher_information(buoys, p_target, sigma_d=SIGMA_D):
    """FIM = (1/sigma_d^2) * H^T H evaluated at the true target position."""
    H = tdoa_jacobian(p_target, buoys)
    return (H.T @ H) / sigma_d**2

def crlb_trace(buoys, p_target, sigma_d=SIGMA_D):
    """
    Scalar CRLB: tr(F^{-1}) = sigma_d^2 * tr((H^T H)^{-1}).
    Returns inf if H^T H is singular.
    """
    H = tdoa_jacobian(p_target, buoys)
    HtH = H.T @ H + REG * np.eye(3)
    try:
        return sigma_d**2 * np.trace(np.linalg.inv(HtH))
    except np.linalg.LinAlgError:
        return np.inf

# --- CRLB-optimal placement ---

def optimise_buoy_layout(p_target, d_max=D_MAX, n_restarts=N_RESTARTS):
    """
    Minimise tr(F^{-1}) over buoy (x, y) positions via L-BFGS-B.
    Buoy z = 0 is fixed (surface constraint).
    """
    def objective(xy_flat):
        buoys = np.zeros((N_BUOYS, 3))
        buoys[:, :2] = xy_flat.reshape(N_BUOYS, 2)
        return crlb_trace(buoys, p_target)

    best_val = np.inf
    best_x   = None
    rng = np.random.default_rng(0)

    for _ in range(n_restarts):
        x0 = rng.uniform(-d_max / 2, d_max / 2, size=(N_BUOYS, 2))
        bounds = [(-d_max / 2, d_max / 2)] * (N_BUOYS * 2)
        res = minimize(objective, x0.ravel(), method="L-BFGS-B", bounds=bounds,
                       options={"maxiter": 500, "ftol": 1e-12})
        if res.fun < best_val:
            best_val = res.fun
            best_x   = res.x

    buoys_opt = np.zeros((N_BUOYS, 3))
    buoys_opt[:, :2] = best_x.reshape(N_BUOYS, 2)
    return buoys_opt, best_val

# --- Monte Carlo evaluation ---

def monte_carlo_localisation(buoys, p_target, sigma_t, n_trials):
    """
    Run n_trials TDOA localisations with independent noise realisations.
    Returns array of 3D position errors (n_trials,).
    """
    rng = np.random.default_rng(42)
    errors = []
    p_init = p_target + rng.normal(0, 15.0, size=3)   # perturbed initial guess

    for _ in range(n_trials):
        tdoa_s     = tdoa_measurements(buoys, p_target, sigma_t, rng)
        delta_d    = C_SOUND * tdoa_s                  # convert to metres
        p_est      = gauss_newton_tdoa(buoys, delta_d, p_init)
        errors.append(np.linalg.norm(p_est - p_target))

    return np.array(errors)

def run_simulation():
    """Main entry point: compare three buoy layouts and report CRLB and MC RMSE."""
    layouts = {
        "Square grid":           layout_square(half_side=100.0),
        "Tetrahedral projection": layout_tetrahedral(radius=120.0),
        "CRLB-optimal":          optimise_buoy_layout(P_TARGET)[0],
    }

    results = {}
    for name, buoys in layouts.items():
        crlb = crlb_trace(buoys, P_TARGET)
        errors = monte_carlo_localisation(buoys, P_TARGET, SIGMA_T, N_TRIALS)
        results[name] = {
            "buoys":   buoys,
            "crlb":    crlb,
            "crlb_rms": np.sqrt(crlb),
            "mc_rmse": np.sqrt(np.mean(errors**2)),
            "mc_p95":  np.percentile(errors, 95),
        }
    return results
```

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Number of buoys $N$ | 4 |
| Speed of sound $c_s$ | 1500 m/s |
| Per-buoy clock noise $\sigma_t$ | $10^{-4}$ s (0.1 ms) |
| TDOA range-difference noise $\sigma_d$ | $c_s \sqrt{2}\, \sigma_t \approx 0.212$ m |
| Ping interval $T_{ping}$ | 5 s |
| Target depth range | $-200$ to $-10$ m |
| True target position $\mathbf{p}_T$ | $(40, -30, -120)$ m |
| Gauss-Newton tolerance $\epsilon_{tol}$ | $10^{-3}$ m |
| Max GN iterations $K_{max}$ | 200 |
| Square-grid half-side | 100 m |
| Tetrahedral layout radius | 120 m |
| Max buoy spread $D_{max}$ | 500 m |
| CRLB optimisation restarts | 20 |
| Monte Carlo trials | 1000 |
| FIM regularisation $\delta$ | $10^{-6}$ |
| Operating bounding box | $\pm 250$ m in $x$ and $y$ |

---

## Expected Output

- **3D buoy-and-target geometry plot**: three side-by-side 3D axes (one per layout strategy)
  showing buoy positions at $z = 0$ (cyan markers), the true target (red star at depth), and
  dashed lines from each buoy down to the target; the three TDOA hyperboloid intersection curves
  sketched on the $z = -120$ m plane as a visual aid.
- **CRLB surface (horizontal slice)**: 2D heatmap of $\text{tr}(\mathbf{F}^{-1})$ as a function
  of target $(x, y)$ at fixed $z = -120$ m for each layout; low-CRLB region highlighted in blue;
  the true target position marked with a cross.
- **Gauss-Newton convergence trace**: $\|\Delta \mathbf{p}^{(k)}\|_2$ vs iteration index for
  10 representative noise realisations; good-geometry vs poor-geometry (collinear) buoy
  configurations compared; convergence in $< 20$ iterations for well-spread layouts.
- **3D error ellipsoids**: plot the $1\sigma$ CRLB error ellipsoid
  $\mathbf{F}^{-1}$ centred on the true target for each layout strategy; illustrates anisotropy
  — typically elongated in depth ($z$) due to the surface-only buoy constraint.
- **Monte Carlo RMSE bar chart**: 3D RMS position error and 95th-percentile error for each layout
  strategy; CRLB-predicted RMS overlaid as a dashed line to verify estimator efficiency.
- **Error histogram**: distribution of 1000 3D position errors for the CRLB-optimal layout;
  Gaussian fit; separate panels for horizontal $\sqrt{\Delta x^2 + \Delta y^2}$ and depth
  $|\Delta z|$ components.
- **CRLB vs target depth curve**: $\sqrt{\text{tr}(\mathbf{F}^{-1})}$ plotted against target
  depth $z_T \in [-200, -10]$ m for all three buoy layouts; shows how localisation degrades
  as the target moves deeper relative to the surface buoy spread.

---

## Extensions

1. **Moving target tracking**: the underwater object moves at $v_T = 1$ m/s on a straight-line
   course; implement an extended Kalman filter (EKF) that fuses successive TDOA batches (one per
   ping) to jointly estimate position and velocity $[\mathbf{p}_T, \dot{\mathbf{p}}_T]$; compare
   track latency and RMSE against the static Gauss-Newton estimator.
2. **Multipath acoustic propagation**: add a specular seabed-reflection path to each buoy;
   the effective measured arrival includes direct-path and surface-bounce echoes with known
   geometry; derive a modified TDOA model that accounts for the two-path structure and evaluate
   the estimation bias when the multipath is ignored.
3. **Adaptive buoy redeployment**: after the first position estimate $\hat{\mathbf{p}}_T$ is
   obtained, compute the CRLB for a refined 4-buoy layout centred on $\hat{\mathbf{p}}_T$;
   command drones to fly to the new hover positions and drop a second buoy set; measure CRLB
   improvement across successive redeployment rounds.
4. **Heterogeneous clock quality**: assign each buoy a different timing noise
   $\sigma_{t,i}$ (e.g., one high-quality GPS-disciplined oscillator vs three lower-grade units);
   derive the weighted FIM $\mathbf{F} = \mathbf{H}^T \boldsymbol{\Sigma}^{-1} \mathbf{H}$ with
   diagonal $\boldsymbol{\Sigma}$ and re-optimise buoy layout under this heterogeneous noise model.
5. **N > 4 buoys (overdetermined TDOA)**: scale to $N = 6$ or $N = 8$ buoys to obtain an
   overdetermined TDOA system; implement the full weighted least-squares solution and evaluate
   RMSE improvement vs the cost of deploying additional buoy drones.

---

## Related Scenarios

- Prerequisites: [S046 Multi-Drone 3D Trilateration](S046_trilateration.md), [S047 Base Station Signal Relay](S047_signal_relay.md)
- Follow-ups: [S060](S060_glacier_melt_monitor.md) (environmental monitoring with sensor networks)
- Algorithmic cross-reference: [S046 Trilateration](S046_trilateration.md) (range-based vs TDOA-based localisation), [S047 Signal Relay](S047_signal_relay.md) (drone relay chain for data exfiltration), [S008 Stochastic Pursuit](../01_pursuit_evasion/S008_stochastic_pursuit.md) (noisy state estimation with Kalman filtering)
