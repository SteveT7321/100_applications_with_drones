# S056 Nuclear Plant Leak Detection — Radiation Hotspot Localization

**Domain**: Environmental Monitoring & SAR | **Difficulty**: ⭐⭐⭐ | **Status**: `[ ]` Not Started
**Algorithm**: Gaussian Radiation Model + IDW Mapping + Least-Squares Source Fitting | **Dimension**: 2D

---

## Problem Definition

**Setup**: A radiation leak of unknown origin has occurred somewhere inside a $100 \times 100$ m
nuclear plant perimeter. The radiation field follows an inverse-square-law attenuation combined with
exponential absorption (Gaussian dispersion) from the source. A single drone carries an onboard
Geiger counter that measures local dose rate as a scalar value corrupted by additive noise. The
drone does not know the source position, intensity, or absorption coefficient in advance.

**Roles**:
- **Drone**: single agent deployed at a fixed starting position on the perimeter; takes sequential
  dose-rate measurements at each visited location, builds an interpolated radiation map, and
  navigates toward regions of increasing dose rate.
- **Radiation source**: fixed point $(x_s, y_s)$ at an unknown location inside the perimeter,
  emitting radiation at intensity $S$ (Bq); position and strength unknown to the drone.
- **Geiger counter**: scalar sensor measuring local dose rate with additive Gaussian noise; all
  measurements are stored for the final least-squares source fit.

**Objective**: (1) Build a radiation dose-rate map of the plant perimeter via IDW interpolation of
noisy measurements; (2) locate the hotspot (maximum dose-rate point) using gradient-ascent
navigation; (3) estimate the true source position $(x_s, y_s)$, intensity $S$, and absorption
coefficient $\lambda$ by fitting the forward radiation model to all collected measurements via
nonlinear least squares.

**Comparison strategies**:
1. **Random walk sampling** — drone follows a random path, building the map without gradient
   guidance; source fitting accuracy serves as the benchmark reference.
2. **Lawnmower grid scan** — systematic boustrophedon coverage; dense uniform map but ignores
   gradient information; may miss fine-scale hotspot structure.
3. **Gradient-ascent (IDW-guided)** — drone moves in the direction of the interpolated dose-rate
   gradient; concentrates measurements near the hotspot; fastest source localization.

---

## Mathematical Model

### Radiation Forward Model

The dose rate at position $\mathbf{p} = (x, y)$ from a point source at $\mathbf{p}_s = (x_s, y_s)$
with source intensity $S$ and linear attenuation coefficient $\lambda$ is:

$$I(\mathbf{p};\; x_s, y_s, S, \lambda)
  = \frac{S \, \exp\!\bigl(-\lambda \,\|\mathbf{p} - \mathbf{p}_s\|\bigr)}
         {4\pi \,\|\mathbf{p} - \mathbf{p}_s\|^2}$$

The numerator $S \exp(-\lambda r)$ models exponential absorption through air and structural
material; the denominator $4\pi r^2$ captures geometric spreading. At $r \to 0$ the model
diverges — in simulation we clamp $r \geq r_{min} = 0.5$ m to avoid singularities.

For compact notation let $r_i = \|\mathbf{p}_i - \mathbf{p}_s\|$ denote the distance between
measurement location $i$ and the source.

### Sensor Model

The dose-rate measurement at drone position $\mathbf{p}$ is:

$$z = I(\mathbf{p}) + \eta, \qquad \eta \sim \mathcal{N}(0,\, \sigma_{sensor}^2)$$

where $\sigma_{sensor}$ is the Geiger counter noise standard deviation. All measurements
$(z_i, \mathbf{p}_i)$ are accumulated for offline map interpolation and source fitting.

### IDW Radiation Map

Given $N$ measurements $\{(z_i, \mathbf{p}_i)\}_{i=1}^{N}$, the interpolated dose rate at any
query point $\mathbf{p}$ is estimated by inverse-distance weighting:

$$\hat{I}(\mathbf{p}) = \frac{\displaystyle\sum_{i=1}^{N} w_i(\mathbf{p})\, z_i}
                              {\displaystyle\sum_{i=1}^{N} w_i(\mathbf{p})},
\qquad
w_i(\mathbf{p}) = \frac{1}{\|\mathbf{p} - \mathbf{p}_i\|^k + \varepsilon}$$

with power parameter $k = 2$ and a small regulariser $\varepsilon = 10^{-6}$ to avoid division by
zero. Higher $k$ gives sharper localisation near measurement points; lower $k$ gives smoother
interpolation.

The numerical gradient of the IDW map at the drone's current position is approximated by
central differences with probe offset $\delta$:

$$\hat{\nabla}_x \hat{I}
  \approx \frac{\hat{I}(x + \delta, y) - \hat{I}(x - \delta, y)}{2\delta}$$

$$\hat{\nabla}_y \hat{I}
  \approx \frac{\hat{I}(x, y + \delta) - \hat{I}(x, y - \delta)}{2\delta}$$

### Gradient-Ascent Navigation

After accumulating at least $N_{warmup}$ measurements (sufficient for a stable IDW estimate),
the drone moves in the direction of steepest dose-rate increase:

$$\mathbf{p}_{t+1} = \mathbf{p}_t + \alpha \, \frac{\hat{\nabla}\hat{I}(\mathbf{p}_t)}
                                                    {\|\hat{\nabla}\hat{I}(\mathbf{p}_t)\| + \varepsilon}$$

where $\alpha$ is the step size (m per timestep). The drone is declared to have reached the
hotspot when:

$$\|\mathbf{p}_t - \hat{\mathbf{p}}_{hotspot}\| \leq r_{capture}$$

where $\hat{\mathbf{p}}_{hotspot} = \arg\max_{\mathbf{p} \in \mathcal{G}} \hat{I}(\mathbf{p})$ is
the estimated hotspot from the current IDW map.

### Nonlinear Least-Squares Source Fitting

Given all $N$ dose-rate measurements $\{(z_i, \mathbf{p}_i)\}$ collected during the mission, the
source parameters $\boldsymbol{\theta} = (x_s, y_s, S, \lambda)$ are recovered by minimising the
sum of squared residuals:

$$\hat{\boldsymbol{\theta}} = \arg\min_{\boldsymbol{\theta}} \;
  \sum_{i=1}^{N} \Bigl(z_i - I(\mathbf{p}_i;\, \boldsymbol{\theta})\Bigr)^2$$

This is a nonlinear least-squares problem solved with `scipy.optimize.curve_fit` (Levenberg–
Marquardt algorithm). The residual Jacobian is evaluated numerically. Initial parameter guesses
are seeded from the IDW hotspot position and the peak measured dose rate.

### Localization Error Metric

The primary performance metric is the Euclidean distance between the estimated and true source
positions:

$$\varepsilon_{loc} = \|\hat{\mathbf{p}}_s - \mathbf{p}_{s,true}\|$$

Secondary metrics:
- **Relative intensity error**: $|(\hat{S} - S_{true})| / S_{true}$
- **Relative absorption error**: $|(\hat{\lambda} - \lambda_{true})| / \lambda_{true}$
- **Peak dose-rate estimate**: $\hat{I}(\hat{\mathbf{p}}_s)$ vs ground-truth $I(\mathbf{p}_{s,true})$

---

## Implementation

```python
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.optimize import curve_fit

# ── Domain constants ─────────────────────────────────────────────────────────
AREA_SIZE     = 100.0     # m, square plant perimeter side
R_MIN         = 0.5       # m, singularity clamp
SIGMA_SENSOR  = 0.02      # dose-rate noise std (normalised units)
IDW_K         = 2         # IDW power parameter
IDW_EPS       = 1e-6      # IDW regulariser
GRAD_DELTA    = 1.0       # m, gradient finite-difference offset

# ── True source (unknown to drone) ───────────────────────────────────────────
TRUE_SOURCE   = np.array([62.0, 55.0])   # m
TRUE_S        = 5.0                       # source intensity (normalised)
TRUE_LAMBDA   = 0.03                      # absorption coefficient (1/m)

# ── Navigation ────────────────────────────────────────────────────────────────
STEP_SIZE     = 1.5       # m, gradient-ascent step
N_WARMUP      = 20        # measurements before gradient ascent starts
R_CAPTURE     = 3.0       # m, hotspot declared found
V_MAX         = 2.0       # m/s, drone speed cap
DT            = 0.5       # s, timestep
T_MAX         = 500.0     # s, mission timeout

# ── Visualisation grid ────────────────────────────────────────────────────────
GRID_N        = 100       # cells per side (1 m resolution)
xs = np.linspace(0, AREA_SIZE, GRID_N)
ys = np.linspace(0, AREA_SIZE, GRID_N)
XX, YY = np.meshgrid(xs, ys)
GRID_PTS = np.stack([XX.ravel(), YY.ravel()], axis=-1)   # (10000, 2)


def radiation_model(pos, x_s, y_s, S, lam):
    """Vectorised forward model: dose rate at positions `pos` (N×2 array)."""
    dx = pos[:, 0] - x_s
    dy = pos[:, 1] - y_s
    r = np.maximum(np.sqrt(dx**2 + dy**2), R_MIN)
    return S * np.exp(-lam * r) / (4.0 * np.pi * r**2)


def dose_rate_scalar(p, source=TRUE_SOURCE, S=TRUE_S, lam=TRUE_LAMBDA):
    """Scalar dose rate at a single position p (length-2 array)."""
    r = max(np.linalg.norm(p - source), R_MIN)
    return S * np.exp(-lam * r) / (4.0 * np.pi * r**2)


def measure(p, rng):
    """Noisy Geiger-counter reading at position p."""
    return dose_rate_scalar(p) + rng.normal(0.0, SIGMA_SENSOR)


def idw_interpolate(query_pts, meas_pts, meas_vals, k=IDW_K):
    """
    IDW interpolation at query_pts given measurements (meas_pts, meas_vals).
    query_pts : (M, 2)
    meas_pts  : (N, 2)
    meas_vals : (N,)
    Returns   : (M,) interpolated values
    """
    # pairwise distances  (M, N)
    diff = query_pts[:, None, :] - meas_pts[None, :, :]   # (M, N, 2)
    dists = np.linalg.norm(diff, axis=-1)                  # (M, N)
    weights = 1.0 / (dists**k + IDW_EPS)                  # (M, N)
    return (weights * meas_vals[None, :]).sum(axis=1) / weights.sum(axis=1)


def idw_gradient(p, meas_pts, meas_vals, delta=GRAD_DELTA):
    """Central-difference gradient of the IDW map at position p."""
    def idw_at(q):
        q2 = q[None, :]
        diff = q2[:, None, :] - meas_pts[None, :, :]
        dists = np.linalg.norm(diff, axis=-1)
        w = 1.0 / (dists**IDW_K + IDW_EPS)
        return float((w * meas_vals[None, :]).sum() / w.sum())

    ex = np.array([delta, 0.0])
    ey = np.array([0.0, delta])
    gx = (idw_at(p + ex) - idw_at(p - ex)) / (2 * delta)
    gy = (idw_at(p + ey) - idw_at(p - ey)) / (2 * delta)
    return np.array([gx, gy])


def fit_source(meas_pts, meas_vals):
    """
    Fit radiation model to measurements using scipy curve_fit.
    Returns estimated (x_s, y_s, S, lambda) and covariance.
    """
    def model_flat(pts_flat, x_s, y_s, S, lam):
        pts = pts_flat.reshape(-1, 2)
        return radiation_model(pts, x_s, y_s, S, lam)

    # Initial guess: hotspot position, crude S/lambda
    hotspot_idx = np.argmax(meas_vals)
    p0_xy = meas_pts[hotspot_idx]
    p0 = [p0_xy[0], p0_xy[1], meas_vals.max() * 10.0, 0.05]

    pts_flat = meas_pts.ravel()
    try:
        popt, pcov = curve_fit(
            model_flat, pts_flat, meas_vals,
            p0=p0,
            bounds=([0, 0, 0, 0], [AREA_SIZE, AREA_SIZE, 1e4, 1.0]),
            maxfev=10000,
        )
    except RuntimeError:
        popt = np.array(p0)
        pcov = np.full((4, 4), np.nan)
    return popt, pcov


def run_simulation(strategy='gradient', seed=42):
    rng = np.random.default_rng(seed)

    pos = np.array([5.0, 5.0])   # start at corner
    meas_pts  = []
    meas_vals = []

    trajectory = [pos.copy()]
    t = 0.0
    found = False
    phase = 'explore'            # explore → ascent

    while t < T_MAX:
        z = measure(pos, rng)
        meas_pts.append(pos.copy())
        meas_vals.append(z)

        n = len(meas_vals)

        if strategy == 'gradient' and n >= N_WARMUP:
            phase = 'ascent'
            pts_arr = np.array(meas_pts)
            vals_arr = np.array(meas_vals)
            g = idw_gradient(pos, pts_arr, vals_arr)
            gnorm = np.linalg.norm(g)
            if gnorm > 1e-9:
                step = STEP_SIZE * g / gnorm
            else:
                step = rng.uniform(-1, 1, size=2)
            pos = np.clip(pos + step, 0, AREA_SIZE)

        elif strategy == 'lawnmower':
            # Boustrophedon: advance column by column
            col = int(t / (AREA_SIZE / V_MAX)) % GRID_N
            row_dir = 1 if (col % 2 == 0) else -1
            pos[0] = col * (AREA_SIZE / GRID_N)
            pos[1] = np.clip(pos[1] + row_dir * V_MAX * DT, 0, AREA_SIZE)

        else:  # random walk (explore-only baseline)
            angle = rng.uniform(0, 2 * np.pi)
            pos = np.clip(pos + STEP_SIZE * np.array([np.cos(angle),
                                                       np.sin(angle)]),
                          0, AREA_SIZE)

        trajectory.append(pos.copy())
        t += DT

        # Check hotspot capture (against IDW map hotspot)
        if n >= N_WARMUP and phase == 'ascent':
            pts_arr = np.array(meas_pts)
            vals_arr = np.array(meas_vals)
            idw_field = idw_interpolate(GRID_PTS, pts_arr, vals_arr)
            hotspot_est = GRID_PTS[np.argmax(idw_field)]
            if np.linalg.norm(pos - hotspot_est) <= R_CAPTURE:
                found = True
                break

    pts_arr  = np.array(meas_pts)
    vals_arr = np.array(meas_vals)
    popt, pcov = fit_source(pts_arr, vals_arr)
    loc_error = np.linalg.norm(popt[:2] - TRUE_SOURCE)

    return {
        'trajectory':  np.array(trajectory),
        'meas_pts':    pts_arr,
        'meas_vals':   vals_arr,
        'popt':        popt,        # (x_s, y_s, S, lambda)
        'pcov':        pcov,
        'loc_error':   loc_error,
        'found':       found,
        'time':        t,
        'phase_hist':  phase,
    }
```

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Plant perimeter area | 100 × 100 m |
| True source position $(x_s, y_s)$ | (62, 55) m |
| Source intensity $S$ | 5.0 (normalised) |
| Absorption coefficient $\lambda$ | 0.03 m⁻¹ |
| Singularity clamp $r_{min}$ | 0.5 m |
| Sensor noise $\sigma_{sensor}$ | 0.02 (normalised) |
| IDW power $k$ | 2 |
| IDW regulariser $\varepsilon$ | $10^{-6}$ |
| Gradient probe offset $\delta$ | 1.0 m |
| Warm-up measurements $N_{warmup}$ | 20 |
| Gradient-ascent step size $\alpha$ | 1.5 m/step |
| Hotspot capture radius $r_{capture}$ | 3.0 m |
| Drone maximum speed $v_{max}$ | 2.0 m/s |
| Simulation timestep $\Delta t$ | 0.5 s |
| Mission timeout $T_{max}$ | 500 s |
| Source fitting algorithm | Levenberg–Marquardt (`scipy.optimize.curve_fit`) |
| Visualisation grid resolution | 100 × 100 cells (1 m/cell) |

---

## Expected Output

- **Radiation field heatmap**: ground-truth 2D dose-rate map $I(x, y)$ over the 100 × 100 m area
  on a logarithmic colour scale; true source marked with a red star; iso-dose contours at 10 %, 
  25 %, 50 %, and 75 % of peak dose rate overlaid.
- **Drone trajectory + measurement scatter**: drone path overlaid on the IDW interpolated map
  (colour-coded by mission phase: exploration in grey, gradient ascent in red); scatter points
  coloured by measured dose rate; estimated source position from least-squares fit shown as a
  white diamond; true source as red star.
- **Dose-rate profile along drone path**: $z_i$ vs measurement index $i$, showing the rising
  trend as the drone approaches the hotspot; fitted model value $\hat{I}(\mathbf{p}_i)$ overlaid
  as a smooth curve; $\pm 2\sigma_{sensor}$ confidence band shown.
- **IDW map snapshots**: four-panel figure showing the interpolated radiation map at
  $N =$ 5, 20, 50, and final measurement counts, illustrating how map fidelity improves as
  measurements accumulate.
- **Source fitting convergence**: scatter plot of estimated $(x_s, y_s)$ across 30 Monte Carlo
  trials (varying noise seed) for gradient-ascent vs random-walk strategies; 1-$\sigma$ ellipses
  from covariance $\hat{\Sigma}_{xy}$; true source marked with a red cross.
- **Metrics summary bar chart**: localization error $\varepsilon_{loc}$, relative intensity error,
  and relative absorption error for all three strategies across $N_{MC} = 30$ trials; median and
  IQR as box plots.
- **Animation (GIF)**: drone moving across the plant perimeter; IDW radiation map rebuilding in
  real time each frame; gradient arrow overlaid when in ascent phase; estimated source position
  updating each frame; frame rate 10 fps.

---

## Extensions

1. **Wind-driven dispersion layer**: superimpose a Gaussian plume atop the inverse-square-law
   field to model airborne contamination; the drone must separate the two source contributions
   in the least-squares fit (joint chemical + radiation source estimation).
2. **Multiple radiation sources**: extend the forward model to a sum of $K$ point sources;
   apply $K$-means clustering to the measurement set as a warm-start for multi-source curve
   fitting and compare with expectation-maximisation (EM) source decomposition.
3. **Shielding obstacles**: introduce rectangular shielding walls that partially block radiation
   (Beer–Lambert attenuation along the line of sight); use a ray-casting model to evaluate
   $I(\mathbf{p})$ and demonstrate how the drone's IDW map reveals the shadow boundary.
4. **Real-time Bayesian source estimation**: replace the offline least-squares fit with an online
   Extended Kalman Filter (EKF) over the state $\boldsymbol{\theta} = (x_s, y_s, S, \lambda)$;
   evaluate estimation error vs measurement count to determine minimum sampling requirements.
5. **Multi-drone cooperative mapping**: deploy $N = 3$ drones with shared IDW map and
   information-gain waypoint allocation; compare coverage speed and source-fit accuracy against
   single-drone gradient ascent (see S049 Dynamic Zone Assignment for coordination policy).
6. **3D volumetric search**: extend to a $100 \times 100 \times 20$ m volume (e.g. searching
   inside a reactor building at varying altitudes); inverse-square-law in 3D; drone optimises
   altitude dynamically to disambiguate source height.

---

## Related Scenarios

- Prerequisites: [S041 Wildfire Boundary Scan](S041_wildfire_boundary_scan.md), [S045 Chemical Plume Tracing](S045_plume_tracing.md)
- Follow-ups: [S057 Underground Pipeline Leak](S057_pipeline_leak.md) (similar scalar-field source inversion), [S058 Air Quality Mapping](S058_air_quality_mapping.md) (dense sensor coverage)
- Algorithmic cross-reference: [S045 Chemical Plume Tracing](S045_plume_tracing.md) (gradient-ascent source finding), [S042 Missing Person Localization](S042_missing_person.md) (Bayesian map-based search), [S055 Oil Spill Tracking](S055_oil_spill_tracking.md) (environmental scalar field mapping)
