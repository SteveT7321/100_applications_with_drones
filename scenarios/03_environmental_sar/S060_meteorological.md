# S060 Multi-Drone Cooperative Meteorological Measurement

**Domain**: Environmental Monitoring & SAR | **Difficulty**: ⭐⭐⭐ | **Status**: `[ ]` Not Started

---

## Problem Definition

**Setup**: Three research drones are deployed in a $600 \times 600$ m horizontal domain to
reconstruct a continuous 3D atmospheric profile of temperature, relative humidity, and wind speed
magnitude. The measurement volume spans altitudes from $z_{min} = 20$ m to $z_{max} = 500$ m.
Each drone is assigned a non-overlapping altitude band and follows a helical or lawnmower
waypoint path through its band, collecting sensor readings at discrete sample locations. After
all drones complete their sampling passes, the ground station fuses all observations via
Ordinary Kriging to produce a continuous volumetric field estimate on a dense 3D grid. A
held-out set of validation points (not visited by any drone) is used to compute reconstruction
RMSE against a synthetic ground-truth atmospheric field.

**Roles**:
- **Drone 1** ($z$-band: 20 – 180 m): samples the atmospheric boundary layer; temperature
  gradients and humidity are most pronounced here; carries temperature and humidity sensors.
- **Drone 2** ($z$-band: 180 – 340 m): samples the lower free atmosphere transition zone;
  carries temperature and wind-speed sensors.
- **Drone 3** ($z$-band: 340 – 500 m): samples the free atmosphere; temperature lapse rate
  dominates; carries temperature and wind-speed sensors.
- **Ground station**: receives all sample data, runs the Hungarian altitude-band assignment
  optimisation at mission start, and executes the Kriging interpolation and validation pipeline.

**Objective**: Assign the three altitude bands to drones via the Hungarian algorithm to minimise
total flight path length, then plan waypoints inside each band that maximise spatial coverage.
After sampling, reconstruct the 3D atmospheric fields via Ordinary Kriging and evaluate
reconstruction quality as RMSE over held-out validation points.

---

## Mathematical Model

### Synthetic Atmospheric Field

The true atmospheric fields are generated analytically as smooth functions of 3D position
$\mathbf{x} = (x, y, z)$ to serve as ground truth. For temperature:

$$T(x, y, z) = T_0 - \Gamma_d \cdot z + A_T \sin\!\left(\frac{2\pi x}{L_x}\right)\cos\!\left(\frac{2\pi y}{L_y}\right)e^{-z/H_T}$$

where $T_0 = 288$ K is the surface reference temperature, $\Gamma_d = 6.5 \times 10^{-3}$ K/m
is the dry adiabatic lapse rate, $A_T = 2$ K is the amplitude of horizontal thermal structure,
and $H_T = 300$ m is the decay scale height. Sensor noise is additive Gaussian:

$$\tilde{f}(\mathbf{x}_i) = f(\mathbf{x}_i) + \epsilon_i, \qquad \epsilon_i \sim \mathcal{N}(0,\, \sigma_s^2)$$

### Variogram Model

The spatial correlation structure of the atmospheric field is characterised by the
experimental semivariogram computed from the $N$ sample pairs:

$$\hat{\gamma}(h) = \frac{1}{2\,|N(h)|} \sum_{(\mathbf{x}_i,\, \mathbf{x}_j) \in N(h)} \bigl[\tilde{f}(\mathbf{x}_i) - \tilde{f}(\mathbf{x}_j)\bigr]^2$$

where $N(h)$ is the set of all sample pairs separated by lag distance $h =
\|\mathbf{x}_i - \mathbf{x}_j\|$. The fitted theoretical model is an exponential variogram:

$$\gamma(h) = C_0 + C_1\!\left(1 - e^{-h/a}\right)$$

with nugget $C_0 \geq 0$ (sensor noise floor), sill $C_0 + C_1$ (total field variance), and
range $a$ (correlation length beyond which samples are effectively independent). Parameters
$(C_0, C_1, a)$ are estimated by non-linear least squares fit to the experimental variogram.

### Ordinary Kriging Estimator

Given $N$ observations $\{f(\mathbf{x}_i)\}_{i=1}^N$ with the fitted variogram, the Kriging
estimate at an unsampled prediction point $\mathbf{x}_0$ is the minimum-variance linear
unbiased estimator (BLUE):

$$\hat{f}(\mathbf{x}_0) = \sum_{i=1}^{N} \lambda_i(\mathbf{x}_0)\, \tilde{f}(\mathbf{x}_i)$$

The Kriging weights $\boldsymbol{\lambda}$ are obtained by solving the Kriging system, which
enforces unbiasedness via a Lagrange multiplier $\mu$:

$$\begin{bmatrix} \mathbf{\Gamma} & \mathbf{1} \\ \mathbf{1}^T & 0 \end{bmatrix}
\begin{bmatrix} \boldsymbol{\lambda} \\ \mu \end{bmatrix}
= \begin{bmatrix} \boldsymbol{\gamma}_0 \\ 1 \end{bmatrix}$$

where $\Gamma_{ij} = \gamma(\|\mathbf{x}_i - \mathbf{x}_j\|)$ is the $N \times N$ semivariogram
matrix evaluated at all pairs of sample locations, $\boldsymbol{\gamma}_0$ is the $N$-vector
of semivariogram values between all sample locations and the prediction point $\mathbf{x}_0$:

$$(\boldsymbol{\gamma}_0)_i = \gamma(\|\mathbf{x}_i - \mathbf{x}_0\|)$$

Solving gives the weights and the Kriging estimation variance (interpolation uncertainty):

$$\boldsymbol{\lambda} = \mathbf{\Gamma}^{-1}(\boldsymbol{\gamma}_0 - \mu\,\mathbf{1})$$

$$\sigma^2_K(\mathbf{x}_0) = \sum_{i=1}^{N} \lambda_i\,\gamma(\|\mathbf{x}_i - \mathbf{x}_0\|) + \mu$$

### Optimal Altitude Band Assignment via Hungarian Algorithm

Let $D = 3$ be the number of drones and $B = 3$ be the number of altitude bands. Define the
cost matrix $\mathbf{C} \in \mathbb{R}^{D \times B}$ where entry $C_{db}$ is the total
waypoint path length that drone $d$ would travel if assigned band $b$, computed from its
current position to the first waypoint of that band:

$$C_{db} = \|\mathbf{p}_d^{(0)} - \mathbf{w}_b^{(1)}\| + L_b^{path}$$

where $\mathbf{p}_d^{(0)}$ is the initial position of drone $d$, $\mathbf{w}_b^{(1)}$ is the
entry waypoint of band $b$, and $L_b^{path}$ is the fixed intra-band path length. The
Hungarian algorithm finds the bijective assignment $\sigma: \{1,2,3\} \to \{1,2,3\}$ that
minimises total cost:

$$\sigma^* = \arg\min_{\sigma} \sum_{d=1}^{D} C_{d,\,\sigma(d)}$$

### Waypoint Path Design Within Each Altitude Band

Each altitude band $[z_{lo}, z_{hi}]$ is covered by a helical lawnmower pattern: the drone
sweeps horizontal rows spaced $\Delta_y$ apart at $N_z$ discrete altitude levels uniformly
spaced within the band, reversing direction at each row end (boustrophedon). The $k$-th
waypoint altitude within a band of thickness $\Delta z = z_{hi} - z_{lo}$ is:

$$z_k = z_{lo} + \frac{(k-1)}{N_z - 1}\,\Delta z, \qquad k = 1, \ldots, N_z$$

Total sample count per drone: $N_{samples} = N_z \times N_{rows}$.

### Reconstruction Quality Metric

After Kriging, reconstruction quality is evaluated against $N_{val}$ held-out validation
points drawn from a uniform grid inside the measurement volume:

$$\text{RMSE} = \sqrt{\frac{1}{N_{val}} \sum_{j=1}^{N_{val}} \left[\hat{f}(\mathbf{x}_j^{val}) - f(\mathbf{x}_j^{val})\right]^2}$$

The mean Kriging variance over all prediction grid points serves as a model-internal uncertainty
estimate:

$$\bar{\sigma}^2_K = \frac{1}{N_{grid}} \sum_{j=1}^{N_{grid}} \sigma^2_K(\mathbf{x}_j^{grid})$$

---

## Implementation

```python
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.optimize import minimize, linear_sum_assignment
from scipy.spatial.distance import cdist

# Key constants
N_DRONES       = 3
X_RANGE        = 600.0       # m — horizontal domain width
Y_RANGE        = 600.0       # m — horizontal domain depth
Z_MIN          = 20.0        # m — lowest sampling altitude
Z_MAX          = 500.0       # m — highest sampling altitude
T0             = 288.0       # K — surface reference temperature
LAPSE_RATE     = 6.5e-3      # K/m — dry adiabatic lapse rate
A_TEMP         = 2.0         # K — horizontal thermal perturbation amplitude
H_DECAY        = 300.0       # m — thermal structure scale height
SIGMA_SENSOR   = 0.3         # K — sensor noise standard deviation
N_Z_LEVELS     = 6           # altitude levels per band
N_ROWS         = 8           # horizontal sweep rows per altitude level
N_VAL          = 200         # held-out validation points

# Altitude band boundaries for 3 drones
BAND_EDGES = np.array([20.0, 180.0, 340.0, 500.0])   # m

def true_temperature(x, y, z):
    """Synthetic 3D temperature field (ground truth)."""
    base    = T0 - LAPSE_RATE * z
    perturb = A_TEMP * np.sin(2*np.pi*x / X_RANGE) * np.cos(2*np.pi*y / Y_RANGE)
    return base + perturb * np.exp(-z / H_DECAY)

def plan_band_waypoints(z_lo, z_hi, n_z, n_rows, x_range, y_range):
    """Generate boustrophedon waypoints covering one altitude band."""
    waypoints = []
    z_levels  = np.linspace(z_lo, z_hi, n_z)
    y_lines   = np.linspace(0.0, y_range, n_rows)
    for iz, z in enumerate(z_levels):
        for ir, y in enumerate(y_lines):
            if ir % 2 == 0:
                xs = [0.0, x_range]
            else:
                xs = [x_range, 0.0]
            for x in xs:
                waypoints.append([x, y, z])
    return np.array(waypoints)

def build_cost_matrix(drone_starts, band_entry_points, band_path_lengths):
    """C[d, b] = transit cost + intra-band path length for drone d in band b."""
    transit = cdist(drone_starts, band_entry_points)    # (D, B)
    return transit + band_path_lengths[np.newaxis, :]   # broadcast over drones

def assign_bands(drone_starts, band_entry_points, band_path_lengths):
    """Hungarian assignment: returns (row_ind, col_ind) optimal assignment."""
    C = build_cost_matrix(drone_starts, band_entry_points, band_path_lengths)
    row_ind, col_ind = linear_sum_assignment(C)
    return row_ind, col_ind, C

def exponential_variogram(h, C0, C1, a):
    """Theoretical exponential variogram model."""
    return C0 + C1 * (1.0 - np.exp(-h / a))

def fit_variogram(sample_pts, sample_vals, n_lags=15):
    """Estimate and fit exponential variogram from sample data."""
    dists   = cdist(sample_pts, sample_pts)
    n       = len(sample_vals)
    d_max   = dists.max()
    lag_edges = np.linspace(0, d_max * 0.6, n_lags + 1)
    lags, gammas = [], []
    for k in range(n_lags):
        mask = (dists > lag_edges[k]) & (dists <= lag_edges[k+1])
        pairs = [(i, j) for i in range(n) for j in range(i+1, n) if mask[i, j]]
        if len(pairs) < 2:
            continue
        sv  = np.mean([(sample_vals[i] - sample_vals[j])**2 for i, j in pairs]) / 2.0
        lags.append((lag_edges[k] + lag_edges[k+1]) / 2.0)
        gammas.append(sv)
    lags, gammas = np.array(lags), np.array(gammas)
    # Non-linear least squares fit of (C0, C1, a)
    def residuals(params):
        C0, C1, a = np.abs(params)
        return exponential_variogram(lags, C0, C1, a) - gammas
    from scipy.optimize import least_squares
    result = least_squares(residuals, x0=[0.1, np.var(sample_vals), d_max/3],
                           bounds=(0, np.inf))
    C0, C1, a = np.abs(result.x)
    return C0, C1, a, lags, gammas

def kriging_predict(sample_pts, sample_vals, pred_pts, C0, C1, a):
    """Ordinary Kriging prediction at pred_pts; returns estimates and variances."""
    N   = len(sample_vals)
    M   = len(pred_pts)
    # Build (N+1)x(N+1) Kriging system matrix
    Gamma_ss = exponential_variogram(cdist(sample_pts, sample_pts), C0, C1, a)
    A        = np.block([[Gamma_ss,            np.ones((N, 1))],
                         [np.ones((1, N)),     np.zeros((1, 1))]])
    z_hat    = np.zeros(M)
    sigma2   = np.zeros(M)
    for j in range(M):
        gamma_s0 = exponential_variogram(
            np.linalg.norm(sample_pts - pred_pts[j], axis=1), C0, C1, a)
        rhs      = np.append(gamma_s0, 1.0)
        weights  = np.linalg.solve(A, rhs)
        lam, mu  = weights[:-1], weights[-1]
        z_hat[j] = lam @ sample_vals
        sigma2[j] = float(lam @ gamma_s0 + mu)
    return z_hat, sigma2

def run_simulation():
    """Main simulation: assign bands, sample, Kriging reconstruct, evaluate RMSE."""
    rng = np.random.default_rng(42)

    # Drone starting positions at ground level corners
    drone_starts = np.array([[0.0, 0.0, Z_MIN],
                              [X_RANGE, 0.0, Z_MIN],
                              [X_RANGE/2, Y_RANGE, Z_MIN]])

    # Plan waypoints for each altitude band
    all_band_wps = []
    for b in range(N_DRONES):
        z_lo, z_hi = BAND_EDGES[b], BAND_EDGES[b+1]
        wps = plan_band_waypoints(z_lo, z_hi, N_Z_LEVELS, N_ROWS,
                                  X_RANGE, Y_RANGE)
        all_band_wps.append(wps)

    # Band entry points and path lengths for cost matrix
    band_entries = np.array([wps[0] for wps in all_band_wps])
    band_lengths = np.array([
        np.sum(np.linalg.norm(np.diff(wps, axis=0), axis=1))
        for wps in all_band_wps
    ])

    # Hungarian assignment
    row_ind, col_ind, cost_matrix = assign_bands(
        drone_starts, band_entries, band_lengths)

    # Sample temperature field along assigned waypoints (with sensor noise)
    all_sample_pts  = []
    all_sample_vals = []
    for d, b in zip(row_ind, col_ind):
        wps  = all_band_wps[b]
        vals = true_temperature(wps[:,0], wps[:,1], wps[:,2])
        vals = vals + rng.normal(0, SIGMA_SENSOR, size=len(vals))
        all_sample_pts.append(wps)
        all_sample_vals.append(vals)

    sample_pts  = np.vstack(all_sample_pts)
    sample_vals = np.concatenate(all_sample_vals)

    # Fit variogram and reconstruct via Kriging
    C0, C1, a, exp_lags, exp_gammas = fit_variogram(sample_pts, sample_vals)

    # Validation points (held-out uniform grid)
    val_x = rng.uniform(0, X_RANGE, N_VAL)
    val_y = rng.uniform(0, Y_RANGE, N_VAL)
    val_z = rng.uniform(Z_MIN, Z_MAX, N_VAL)
    val_pts  = np.column_stack([val_x, val_y, val_z])
    val_true = true_temperature(val_x, val_y, val_z)

    z_hat_val, sigma2_val = kriging_predict(
        sample_pts, sample_vals, val_pts, C0, C1, a)
    rmse = np.sqrt(np.mean((z_hat_val - val_true)**2))

    return {
        "sample_pts":     sample_pts,
        "sample_vals":    sample_vals,
        "val_pts":        val_pts,
        "val_true":       val_true,
        "z_hat_val":      z_hat_val,
        "sigma2_val":     sigma2_val,
        "rmse":           rmse,
        "variogram_params": (C0, C1, a),
        "exp_lags":       exp_lags,
        "exp_gammas":     exp_gammas,
        "assignment":     list(zip(row_ind.tolist(), col_ind.tolist())),
        "cost_matrix":    cost_matrix,
        "all_band_wps":   all_band_wps,
        "drone_starts":   drone_starts,
    }
```

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Number of drones $N$ | 3 |
| Horizontal domain $X \times Y$ | $600 \times 600$ m |
| Altitude range $[z_{min}, z_{max}]$ | 20 – 500 m |
| Altitude band boundaries | 20, 180, 340, 500 m |
| Surface reference temperature $T_0$ | 288 K |
| Dry adiabatic lapse rate $\Gamma_d$ | 6.5 $\times$ 10$^{-3}$ K/m |
| Thermal perturbation amplitude $A_T$ | 2 K |
| Thermal decay scale height $H_T$ | 300 m |
| Sensor noise std dev $\sigma_s$ | 0.3 K |
| Altitude levels per band $N_z$ | 6 |
| Horizontal sweep rows per level $N_{rows}$ | 8 |
| Samples per drone | 96 (6 levels $\times$ 8 rows $\times$ 2 endpoints) |
| Total sample count | 288 |
| Held-out validation points $N_{val}$ | 200 |
| Variogram model | Exponential: $C_0 + C_1(1 - e^{-h/a})$ |
| Expected reconstruction RMSE | $< 0.5$ K |

---

## Expected Output

- **3D flight path plot**: three drone trajectories in the measurement volume, coloured by
  altitude band (low: green, mid: orange, high: purple); sample point locations marked as
  small dots; domain bounding box shown as a wire frame; drone start positions marked with
  filled circles.
- **Hungarian assignment cost matrix heatmap**: $3 \times 3$ grid showing $C_{db}$ values
  with the optimal assignment diagonal highlighted; total cost of optimal vs. unoptimised
  (identity) assignment annotated.
- **Experimental variogram with fitted model**: scatter of $\hat{\gamma}(h)$ values versus
  lag distance $h$; fitted exponential curve overlaid; nugget $C_0$, sill $C_0 + C_1$, and
  range $a$ annotated with dashed lines.
- **3D reconstructed temperature field**: volumetric slice visualisation (three orthogonal
  cross-sections at $x = 300$ m, $y = 300$ m, $z = 200$ m) showing Kriging-estimated
  temperature with a diverging colormap; sample locations projected onto slices.
- **3D Kriging variance map**: same three cross-sections showing $\sigma^2_K(\mathbf{x})$;
  high-uncertainty zones (far from sample paths) highlighted; demonstrates how the helical
  boustrophedon path reduces variance within each band while band boundaries show
  elevated uncertainty.
- **Reconstruction RMSE vs. samples-per-drone**: curve sweeping $N_z \in \{3, 4, 6, 8, 10\}$
  levels, showing how RMSE and mean Kriging variance decrease as sampling density increases;
  operating point ($N_z = 6$) marked.
- **Validation scatter plot**: predicted $\hat{T}$ vs. true $T$ at the 200 held-out points;
  diagonal reference line; RMSE and R$^2$ annotated.

---

## Extensions

1. **Adaptive sampling with Kriging variance feedback**: after each drone completes a pass,
   recompute $\sigma^2_K$ over the remaining unvisited volume and redirect drones to the
   highest-uncertainty subregion; compare final RMSE against fixed-path sampling.
2. **Multi-variable co-Kriging**: jointly interpolate temperature, humidity, and wind speed
   using cross-variograms that capture physical correlations (e.g. higher humidity correlated
   with lower temperature inversions); derive the co-Kriging system and show RMSE reduction
   over independent single-variable Kriging.
3. **Online variogram estimation with streaming updates**: fit variogram parameters
   incrementally as samples arrive using recursive exponential smoothing of the experimental
   semivariogram; evaluate fit convergence speed and effect on early-mission reconstruction
   quality.
4. **Time-varying atmospheric field**: introduce a slow drift in the temperature field
   $T(x,y,z,t)$ driven by a wind-advection term; propagate the Kriging estimate forward in
   time using a Kalman-Kriging (space-time Kriging) framework and quantify prediction accuracy
   for a 10-minute forecast horizon.
5. **N-drone scaling study**: vary the number of drones from 1 to 6 and re-run the Hungarian
   assignment and Kriging pipeline; plot reconstruction RMSE vs. drone count; identify the
   diminishing-returns inflection point where adding more drones yields less than 5% RMSE
   improvement.

---

## Related Scenarios

- Prerequisites: [S048 Lawnmower Coverage](S048_lawnmower.md), [S045 Chemical Plume Tracing](S045_plume_tracing.md)
- Follow-ups: [S058 Typhoon Eye Probing](S058_typhoon.md)
- Algorithmic cross-reference: [S046 Multi-Drone 3D Trilateration](S046_trilateration.md) (3D spatial measurement fusion), [S052 Glacier Surface Mapping](S052_glacier.md) (spatial interpolation over complex terrain)
