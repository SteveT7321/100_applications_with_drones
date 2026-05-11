# S060 Multi-Drone Cooperative Meteorological Measurement

**Domain**: Environmental Monitoring & SAR | **Difficulty**: ⭐⭐⭐ | **Status**: `[ ]` Not Started
**Algorithm**: Altitude-Stratified Sampling + Kriging 3D Interpolation | **Dimension**: 3D

---

## Problem Definition

**Setup**: A $200 \times 200 \times 100$ m atmospheric volume must be characterised for temperature,
humidity, and wind speed. The true atmospheric field varies smoothly with position — primarily
governed by an altitude-dependent lapse rate modulated by horizontal sinusoidal perturbations that
represent mesoscale boundary-layer structures. No pre-existing sensor network exists; all data must
be gathered in situ by airborne drones within a fixed flight-time budget.

A coordinator dispatches $N = 5$ lightweight UAVs, each carrying a miniature meteorological payload
(thermistor, capacitive humidity sensor, ultrasonic anemometer). To maximise vertical coverage and
avoid redundant sampling, the volume is partitioned into $N$ equal-depth altitude layers and each
drone is assigned exactly one layer. Within its layer, the drone executes a **lawnmower
(boustrophedon) path** — a systematic back-and-forth sweep — to maximise horizontal coverage.
All collected samples are relayed to a ground station, which merges them and fits a **Kriging
spatial interpolator** to reconstruct the full 3D atmospheric field on a regular grid.

**Roles**:
- **Drones** ($N = 5$): homogeneous platforms; each assigned one altitude layer; fly independently
  within their layer; broadcast sensor readings to the ground station upon completion.
- **Ground station**: receives all sample points, fits the Kriging model, evaluates RMSE against
  the analytic ground-truth field, and produces 3D field reconstructions.

**Objective**: Reconstruct the 3D temperature field $\hat{T}(\mathbf{x})$ with minimum RMSE
relative to the analytic truth $T(\mathbf{x})$. Two strategies are compared under an equal total
flight-time budget:

1. **Single-drone full-volume sweep** — one drone covers all altitudes sequentially; same total
   path length as the cooperative approach.
2. **Multi-drone stratified sweep** (proposed) — five drones cover their assigned layers in
   parallel; each drone has one-fifth the path length but the ensemble provides uniform vertical
   stratification.

---

## Mathematical Model

### True Atmospheric Field

The analytic temperature field combines a linear lapse rate with horizontal sinusoidal perturbations
representing atmospheric wave structures:

$$T(x, y, z) = T_0 - \Gamma z + A \sin\!\left(\frac{2\pi x}{\lambda_x}\right) \cos\!\left(\frac{2\pi y}{\lambda_y}\right)$$

where:
- $T_0 = 25\,^\circ\mathrm{C}$ — surface reference temperature
- $\Gamma = 0.0065\,^\circ\mathrm{C}\,\mathrm{m}^{-1}$ — environmental lapse rate (dry adiabatic $\approx 9.8$, moist $\approx 6.5$)
- $A = 2.0\,^\circ\mathrm{C}$ — horizontal perturbation amplitude
- $\lambda_x = 100\,\mathrm{m}$, $\lambda_y = 100\,\mathrm{m}$ — horizontal wavelengths

Sensor noise is additive Gaussian:

$$\tilde{T}(\mathbf{p}) = T(\mathbf{p}) + \varepsilon, \qquad \varepsilon \sim \mathcal{N}(0,\, \sigma_T^2)$$

### Altitude Layer Assignment

The vertical extent $[0, z_{max}]$ is partitioned into $N$ equal-depth layers. Drone $i$
($i = 1, \ldots, N$) is assigned the layer:

$$h_i \in \left[\frac{(i-1)\, z_{max}}{N},\; \frac{i\, z_{max}}{N}\right]$$

Each drone flies at the mid-altitude of its assigned layer:

$$z_i^{fly} = \frac{(2i - 1)\, z_{max}}{2N}$$

### Lawnmower Sampling Path

Within layer $i$, the drone executes a boustrophedon sweep over the $200 \times 200$ m horizontal
footprint. Given $n_{strips}$ equally spaced east–west passes separated by $\delta y = x_{max} / (n_{strips} - 1)$,
the waypoint sequence for strip $j$ is:

$$\mathbf{w}_{j,k} = \begin{cases}
(k \cdot \delta x,\; (j-1) \cdot \delta y,\; z_i^{fly}) & j \text{ odd} \\
(x_{max} - k \cdot \delta x,\; (j-1) \cdot \delta y,\; z_i^{fly}) & j \text{ even}
\end{cases}$$

where $\delta x$ is the inter-sample spacing along each strip. Samples are collected at every
waypoint $\mathbf{w}_{j,k}$.

### Kriging 3D Interpolation

Kriging (Gaussian process regression) estimates the field value at an unvisited location
$\mathbf{p}_0$ as a weighted linear combination of the $n_s$ observed values
$\{Z(\mathbf{p}_i)\}_{i=1}^{n_s}$:

$$\hat{Z}(\mathbf{p}_0) = \sum_{i=1}^{n_s} w_i \, Z(\mathbf{p}_i)$$

The weights are determined by the variogram structure. The **empirical variogram** is estimated
from data:

$$\hat{\gamma}(h) = \frac{1}{2 |N(h)|} \sum_{(i,j) \in N(h)} \bigl(Z(\mathbf{p}_i) - Z(\mathbf{p}_j)\bigr)^2$$

where $N(h)$ is the set of sample pairs separated by lag distance $h$. A theoretical model
(spherical or Gaussian) is then fitted to $\hat{\gamma}(h)$.

The **ordinary Kriging system** enforces unbiasedness ($\sum_i w_i = 1$) via a Lagrange multiplier
$\mu$:

$$\begin{bmatrix} \boldsymbol{\Gamma} + \lambda \mathbf{I} & \mathbf{1} \\ \mathbf{1}^\top & 0 \end{bmatrix}
\begin{bmatrix} \mathbf{w} \\ \mu \end{bmatrix}
=
\begin{bmatrix} \boldsymbol{\gamma}_0 \\ 1 \end{bmatrix}$$

where:
- $\Gamma_{ij} = \hat{\gamma}(\|\mathbf{p}_i - \mathbf{p}_j\|)$ — variogram matrix between sample pairs
- $\gamma_{0,i} = \hat{\gamma}(\|\mathbf{p}_i - \mathbf{p}_0\|)$ — variogram between each sample and the prediction point
- $\lambda \geq 0$ — nugget regularisation term (handles noise and numerical conditioning)

Solving the $(n_s + 1) \times (n_s + 1)$ linear system yields the Kriging weights $\mathbf{w}$.

### Variogram Models

Two standard parametric variogram models are compared:

**Gaussian variogram**:

$$\gamma(h) = c_0 + c_1 \left[1 - \exp\!\left(-\frac{h^2}{a^2}\right)\right]$$

**Spherical variogram**:

$$\gamma(h) = \begin{cases}
c_0 + c_1 \left[\dfrac{3h}{2a} - \dfrac{h^3}{2a^3}\right] & h \leq a \\
c_0 + c_1 & h > a
\end{cases}$$

where $c_0$ — nugget, $c_1$ — partial sill, $a$ — range parameter.

### Quality Metric: RMSE

After Kriging reconstruction on a regular $G_x \times G_y \times G_z$ evaluation grid, the
root-mean-square error against the analytic truth is:

$$\mathrm{RMSE} = \sqrt{\frac{1}{N_{grid}} \sum_{j=1}^{N_{grid}} \bigl(\hat{T}(\mathbf{p}_j) - T(\mathbf{p}_j)\bigr)^2}$$

where $N_{grid} = G_x \times G_y \times G_z$ is the total number of evaluation points.

### Coverage per Layer

The fraction of the horizontal extent sampled in layer $i$ is:

$$C_i = \frac{n_{samples,i} \cdot r_{sample}^2}{\,x_{max} \cdot y_{max}\,}$$

where $r_{sample}$ is the effective sensor footprint radius. Uniform stratification ensures
$C_i \approx C_j$ for all pairs $i, j$.

---

## Implementation

```python
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
from scipy.linalg import solve

# ── Simulation constants ──────────────────────────────────────────────────────
N_DRONES    = 5
X_MAX       = 200.0      # m — volume east-west extent
Y_MAX       = 200.0      # m — volume north-south extent
Z_MAX       = 100.0      # m — volume vertical extent
DRONE_SPEED = 5.0        # m/s — cruise speed

# Atmospheric field parameters
T0      = 25.0           # °C — surface temperature
GAMMA   = 0.0065         # °C/m — lapse rate
A_PERTURB = 2.0          # °C — horizontal perturbation amplitude
LAMBDA_X  = 100.0        # m  — east-west wavelength
LAMBDA_Y  = 100.0        # m  — north-south wavelength
SIGMA_T   = 0.2          # °C — sensor noise std dev

# Sampling path
N_STRIPS  = 10           # lawnmower strips per layer
N_SAMPLES_PER_STRIP = 20 # waypoints per strip

# Evaluation grid (for RMSE computation)
GX, GY, GZ = 20, 20, 20

# Kriging nugget regularisation
KRIGING_NUGGET = 1e-3


def true_temperature(x, y, z):
    """Analytic 3D temperature field."""
    return (T0
            - GAMMA * z
            + A_PERTURB * np.sin(2 * np.pi * x / LAMBDA_X)
                        * np.cos(2 * np.pi * y / LAMBDA_Y))


def assign_altitude_layers(n_drones, z_max):
    """Return (z_low, z_mid, z_high) for each drone layer."""
    layers = []
    for i in range(n_drones):
        z_low  = i * z_max / n_drones
        z_high = (i + 1) * z_max / n_drones
        z_mid  = (z_low + z_high) / 2.0
        layers.append((z_low, z_mid, z_high))
    return layers


def lawnmower_path(z_fly, n_strips, n_per_strip, x_max, y_max):
    """
    Generate lawnmower waypoints for a single altitude layer.
    Returns (n_strips * n_per_strip, 3) array of (x, y, z) positions.
    """
    waypoints = []
    y_positions = np.linspace(0, y_max, n_strips)
    x_positions = np.linspace(0, x_max, n_per_strip)

    for j, y in enumerate(y_positions):
        xs = x_positions if j % 2 == 0 else x_positions[::-1]
        for x in xs:
            waypoints.append([x, y, z_fly])

    return np.array(waypoints)


def collect_samples(waypoints, rng):
    """
    Simulate meteorological measurements at each waypoint.
    Returns array of (x, y, z, T_measured).
    """
    samples = []
    for wp in waypoints:
        T_true = true_temperature(*wp)
        T_obs  = T_true + rng.normal(0, SIGMA_T)
        samples.append([wp[0], wp[1], wp[2], T_obs])
    return np.array(samples)   # (n_pts, 4)


def compute_empirical_variogram(samples, n_lags=15, max_lag=None):
    """
    Estimate empirical variogram from scattered 3D samples.
    Returns (lag_centres, gamma_values).
    """
    coords = samples[:, :3]    # (n, 3)
    values = samples[:, 3]     # (n,)

    # All pairwise distances and squared differences
    n = len(coords)
    lags, sq_diffs = [], []
    for i in range(n):
        for j in range(i + 1, n):
            h = np.linalg.norm(coords[i] - coords[j])
            sd = (values[i] - values[j]) ** 2
            lags.append(h)
            sq_diffs.append(sd)

    lags     = np.array(lags)
    sq_diffs = np.array(sq_diffs)

    if max_lag is None:
        max_lag = lags.max() / 2.0

    bins = np.linspace(0, max_lag, n_lags + 1)
    centres = (bins[:-1] + bins[1:]) / 2.0
    gamma   = np.zeros(n_lags)

    for k in range(n_lags):
        mask = (lags >= bins[k]) & (lags < bins[k + 1])
        if mask.sum() > 0:
            gamma[k] = 0.5 * np.mean(sq_diffs[mask])

    return centres, gamma


def gaussian_variogram(h, nugget, sill, range_param):
    """γ(h) = nugget + (sill - nugget) * (1 - exp(-h² / range²))"""
    return nugget + (sill - nugget) * (1 - np.exp(-(h ** 2) / (range_param ** 2)))


def fit_variogram(lag_centres, gamma_vals):
    """
    Least-squares fit of a Gaussian variogram model.
    Returns (nugget, sill, range_param).
    """
    from scipy.optimize import curve_fit
    valid = gamma_vals > 0
    if valid.sum() < 3:
        return 0.0, gamma_vals.max(), lag_centres.max() / 2.0
    try:
        popt, _ = curve_fit(
            gaussian_variogram,
            lag_centres[valid],
            gamma_vals[valid],
            p0=[0.01, gamma_vals[valid].max(), lag_centres[valid].mean()],
            bounds=([0, 0, 1], [np.inf, np.inf, np.inf]),
            maxfev=2000
        )
        return popt
    except RuntimeError:
        return 0.0, gamma_vals.max(), lag_centres.max() / 2.0


def kriging_interpolate(samples, query_pts, nugget, sill, range_param):
    """
    Ordinary Kriging on 3D scattered data.
    samples   : (n_s, 4) — [x, y, z, value]
    query_pts : (n_q, 3) — [x, y, z] prediction locations
    Returns   : (n_q,) array of Kriging estimates.
    """
    coords  = samples[:, :3]
    values  = samples[:, 3]
    n_s     = len(coords)

    # Build variogram matrix Γ + nugget on diagonal
    dists   = np.linalg.norm(
        coords[:, np.newaxis, :] - coords[np.newaxis, :, :], axis=2
    )  # (n_s, n_s)
    Gamma   = gaussian_variogram(dists, nugget, sill, range_param)
    Gamma  += KRIGING_NUGGET * np.eye(n_s)

    # Augment with unbiasedness constraint
    A = np.zeros((n_s + 1, n_s + 1))
    A[:n_s, :n_s] = Gamma
    A[:n_s,  n_s] = 1.0
    A[n_s,  :n_s] = 1.0

    # Predict each query point
    estimates = np.zeros(len(query_pts))
    for q_idx, q in enumerate(query_pts):
        d0   = np.linalg.norm(coords - q, axis=1)   # (n_s,)
        g0   = gaussian_variogram(d0, nugget, sill, range_param)
        rhs  = np.append(g0, 1.0)
        try:
            sol  = solve(A, rhs)
        except np.linalg.LinAlgError:
            sol  = np.linalg.lstsq(A, rhs, rcond=None)[0]
        weights    = sol[:n_s]
        estimates[q_idx] = weights @ values

    return estimates


def compute_rmse(samples, eval_grid_pts, nugget, sill, range_param):
    """Kriging RMSE on a regular 3D grid vs analytic truth."""
    T_krig  = kriging_interpolate(samples, eval_grid_pts, nugget, sill, range_param)
    T_true  = true_temperature(
        eval_grid_pts[:, 0], eval_grid_pts[:, 1], eval_grid_pts[:, 2]
    )
    return float(np.sqrt(np.mean((T_krig - T_true) ** 2)))


def build_eval_grid(gx=GX, gy=GY, gz=GZ):
    """Build a regular 3D evaluation grid; return (N_grid, 3) array."""
    xs = np.linspace(0, X_MAX, gx)
    ys = np.linspace(0, Y_MAX, gy)
    zs = np.linspace(0, Z_MAX, gz)
    xx, yy, zz = np.meshgrid(xs, ys, zs, indexing='ij')
    return np.column_stack([xx.ravel(), yy.ravel(), zz.ravel()])


def run_simulation():
    rng = np.random.default_rng(42)
    eval_grid = build_eval_grid()

    # ── Multi-drone stratified sampling ──────────────────────────────────────
    layers    = assign_altitude_layers(N_DRONES, Z_MAX)
    all_paths = []
    all_samples_multi = []

    for drone_id, (z_low, z_mid, z_high) in enumerate(layers):
        path    = lawnmower_path(z_mid, N_STRIPS, N_SAMPLES_PER_STRIP, X_MAX, Y_MAX)
        samples = collect_samples(path, rng)
        all_paths.append(path)
        all_samples_multi.append(samples)

    samples_multi = np.vstack(all_samples_multi)

    # ── Single-drone full-volume sweep (same total path length) ───────────────
    z_levels      = np.linspace(0, Z_MAX, N_DRONES * N_STRIPS)
    all_samples_single = []
    n_per_strip_single = N_SAMPLES_PER_STRIP

    for j, z in enumerate(z_levels):
        path_s  = lawnmower_path(z, 1, n_per_strip_single, X_MAX, Y_MAX)
        samp_s  = collect_samples(path_s, rng)
        all_samples_single.append(samp_s)

    samples_single = np.vstack(all_samples_single)

    # ── Variogram fitting and Kriging for each strategy ───────────────────────
    results = {}
    for label, samples in [("multi", samples_multi), ("single", samples_single)]:
        lag_c, gamma_v = compute_empirical_variogram(samples)
        nugget, sill, r_param = fit_variogram(lag_c, gamma_v)
        rmse = compute_rmse(samples, eval_grid, nugget, sill, r_param)
        results[label] = {
            "samples":    samples,
            "lag_c":      lag_c,
            "gamma_v":    gamma_v,
            "nugget":     nugget,
            "sill":       sill,
            "range":      r_param,
            "rmse":       rmse,
        }
        print(f"[{label:>6s}]  n_samples={len(samples):4d}  "
              f"RMSE={rmse:.4f} °C  "
              f"nugget={nugget:.4f}  sill={sill:.4f}  range={r_param:.2f} m")

    # ── RMSE vs number of samples curve ──────────────────────────────────────
    sample_counts = np.linspace(50, len(samples_multi), 12, dtype=int)
    rmse_vs_n_multi,  rmse_vs_n_single = [], []

    for n in sample_counts:
        idx_m = rng.choice(len(samples_multi),  size=min(n, len(samples_multi)),  replace=False)
        idx_s = rng.choice(len(samples_single), size=min(n, len(samples_single)), replace=False)

        for idx, rmse_list, label in [
            (idx_m, rmse_vs_n_multi,  "multi"),
            (idx_s, rmse_vs_n_single, "single"),
        ]:
            sub = (samples_multi if label == "multi" else samples_single)[idx]
            lag_c2, gamma_v2 = compute_empirical_variogram(sub, n_lags=10)
            ng2, si2, rp2 = fit_variogram(lag_c2, gamma_v2)
            rmse_list.append(compute_rmse(sub, eval_grid, ng2, si2, rp2))

    return results, all_paths, layers, sample_counts, rmse_vs_n_multi, rmse_vs_n_single
```

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Volume extent $(x_{max}, y_{max}, z_{max})$ | $200 \times 200 \times 100$ m |
| Number of drones $N$ | 5 |
| Drone cruise speed $v$ | 5.0 m/s |
| Surface temperature $T_0$ | 25.0 °C |
| Lapse rate $\Gamma$ | 0.0065 °C m$^{-1}$ |
| Horizontal perturbation amplitude $A$ | 2.0 °C |
| Horizontal wavelengths $\lambda_x, \lambda_y$ | 100 m |
| Sensor noise std dev $\sigma_T$ | 0.2 °C |
| Lawnmower strips per layer | 10 |
| Samples per strip | 20 |
| Total samples (multi-drone) | 1 000 |
| Total samples (single-drone, equal budget) | 1 000 |
| Evaluation grid $G_x \times G_y \times G_z$ | $20 \times 20 \times 20 = 8\,000$ points |
| Kriging nugget regularisation $\lambda$ | $10^{-3}$ |
| Variogram model | Gaussian |
| Variogram lags | 15 |
| Random seed | 42 |

---

## Expected Output

- **3D drone path visualisation**: one 3D axes showing all five lawnmower paths simultaneously,
  each drawn in a distinct colour with altitude-coded transparency; layer boundaries shown as
  horizontal semi-transparent planes; axes labelled in metres.
- **RMSE vs number of samples**: two curves (multi-drone stratified vs. single-drone full-volume)
  as a function of total sample count from 50 to 1 000; multi-drone curve expected to lie
  consistently below single-drone curve due to uniform vertical coverage; y-axis in °C.
- **Vertical profile comparison**: 1D plot of horizontally averaged reconstructed temperature
  $\langle \hat{T} \rangle_{xy}(z)$ vs analytic mean profile $\langle T \rangle_{xy}(z)$ for both
  strategies; multi-drone reconstruction expected to track the lapse rate more faithfully.
- **Animation** (`s060_sampling.gif`): top-down and side-view dual-panel animation showing drone
  positions advancing along their lawnmower paths at each time step; sampled points accumulating
  as coloured dots colour-coded by measured temperature; frame rate 10 fps, 60 frames total.
- **Metrics reported** (console output):
  - RMSE (°C) for multi-drone vs single-drone Kriging reconstruction
  - Coverage fraction per altitude layer (should be $\approx 1.0$ for all five layers)
  - Total flight distance per drone (m) and mission time (s)
  - Fitted variogram parameters: nugget, sill, range (m)

---

## Extensions

1. **Humidity and wind-speed reconstruction**: extend the sensor model to output three scalar
   fields $(T, \mathrm{RH}, \|\mathbf{u}\|)$ simultaneously; fit independent Kriging models for
   each and visualise the joint 3D reconstruction as a multi-variable volumetric rendering.
2. **Adaptive re-sampling**: after an initial coarse sweep, identify high-gradient regions
   (large Kriging variance $\sigma^2_K(\mathbf{p}_0)$) and task one drone to perform a targeted
   densification flight; measure RMSE reduction per additional sample (information gain).
3. **Hilbert-curve path**: replace the lawnmower with a 2D Hilbert-curve traversal within each
   layer; compare spatial coverage uniformity using a discrepancy metric and resulting RMSE at
   equal sample counts.
4. **Kriging vs RBF interpolation**: compare ordinary Kriging against radial basis function (RBF)
   interpolation (`scipy.interpolate.RBFInterpolator`) and thin-plate splines; plot RMSE vs.
   sample count for all three; analyse computational cost scaling with $n_s$.
5. **Wind-drift compensation**: introduce a constant horizontal wind field $\mathbf{u}_{wind}$
   that displaces the drone from its nominal waypoint; re-plan paths in real time to compensate
   for drift and evaluate the impact on layer coverage completeness.

---

## Related Scenarios

- Prerequisites: [S046 Multi-Drone 3D Trilateration](S046_trilateration.md), [S047 Signal Relay](S047_signal_relay_enhancement.md), [S059 Swarm Formation Morphing](S059_swarm_formation_morphing.md)
- Follow-ups: S061 Industrial Gas Leak Detection (domain 4 opener)
- Algorithmic cross-reference: [S050 Swarm Cooperative Mapping (EKF-SLAM)](S050_slam.md) (multi-agent data fusion), [S042 Missing Person Search](S042_missing_person_search.md) (Bayesian spatial inference), [S007 Blind Pursuit under Jamming](../01_pursuit_evasion/S007_blind_pursuit_jamming.md) (noisy field estimation)
