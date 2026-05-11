# S055 Coastline Oil Spill Tracking

**Domain**: Environmental Monitoring & SAR | **Difficulty**: ⭐⭐⭐ | **Status**: `[ ]` Not Started
**Algorithm**: Advection-Diffusion Simulation + Contour-Following + Kalman Prediction | **Dimension**: 2D

---

## Problem Definition

**Setup**: An oil tanker accident has released crude oil into a $200 \times 100$ m coastal section.
At $t = 0$ the spill is initialised as a Gaussian blob of radius $r_0 = 15$ m centred at the
release point $\mathbf{c}_{spill} = (60, 50)$ m. The oil concentration field $C(x, y, t)$
evolves under a simplified advection-diffusion model driven by a constant ocean current velocity
$\mathbf{v}_{current}$ and isotropic diffusion coefficient $D$. A single drone flies at a fixed
altitude of $z = 8$ m carrying a downward optical sensor that detects the oil/clean-water boundary
via reflectance contrast (oil appears darker than clean water).

**Roles**:
- **Oil spill** (environment): a spreading, advecting scalar concentration field whose
  $C_{thresh}$-isosurface is the boundary to be tracked.
- **Drone**: single agent using a contour-following zigzag strategy to repeatedly sweep the
  oil/water boundary, relay the extracted polygon to a coast-guard vessel, and predict where
  the boundary centroid will be $T_{pred} = 60$ s in the future so that chemical dispersant
  drones can be pre-positioned.

**Objective**: Continuously track the evolving oil boundary with low positional error, estimate
the spill area via the shoelace formula on the traced contour polygon, and deliver a Kalman
filter prediction of the boundary centroid $T_{pred} = 60$ s ahead at each lap completion.

**Key question**: How does boundary-tracking lag grow as current speed increases? At what
advection velocity does the single-drone contour-follower fail to close the contour polygon
before the spill boundary has moved by more than one sensor footprint width?

---

## Mathematical Model

### Advection-Diffusion Spill Dynamics

The oil concentration field $C(\mathbf{x}, t) \in [0, 1]$ evolves according to the
advection-diffusion PDE:

$$\frac{\partial C}{\partial t} = -\mathbf{v} \cdot \nabla C + D \nabla^2 C$$

where $\mathbf{v} = (v_x, v_y)^T$ is the depth-averaged ocean current velocity (m/s) and
$D$ is the turbulent diffusion coefficient (m$^2$/s). The domain $[0, 200] \times [0, 100]$ m
is discretised onto a grid with cell size $\delta = 0.5$ m. At each timestep $\Delta t$ the
concentration is updated with an explicit finite-difference scheme:

$$C_{i,j}^{t+1} = C_{i,j}^t
  - \Delta t \left( v_x \frac{C_{i+1,j}^t - C_{i-1,j}^t}{2\delta}
                  + v_y \frac{C_{i,j+1}^t - C_{i,j-1}^t}{2\delta} \right)
  + D\Delta t \frac{C_{i+1,j}^t + C_{i-1,j}^t + C_{i,j+1}^t + C_{i,j-1}^t - 4C_{i,j}^t}{\delta^2}$$

Boundary conditions: Neumann (zero-flux) on all four domain edges. Stability condition
(CFL + diffusion):

$$\Delta t \leq \min\!\left(\frac{\delta}{2\|\mathbf{v}\|_\infty},\; \frac{\delta^2}{4D}\right)$$

### Oil / Clean-Water Boundary

The spill boundary $\partial\Omega(t)$ is the $C_{thresh}$-level set of the concentration field:

$$\partial\Omega(t) = \bigl\{(x, y) : C(x, y, t) = C_{thresh}\bigr\}$$

The boundary polygon is extracted from the grid using marching-squares on the concentration field,
yielding an ordered sequence of $M$ vertices
$\{(x_1, y_1), \ldots, (x_M, y_M)\}$.

### Optical Sensor Model

The drone at position $\mathbf{p}_d = (p_x, p_y)$ observes a disc of radius $r_s = 1.5$ m on the
water surface. The raw reflectance reading is:

$$\rho(\mathbf{p}_d) = \rho_{oil} \cdot \bar{C}_{footprint} + \rho_{water} \cdot (1 - \bar{C}_{footprint}) + \eta$$

where $\bar{C}_{footprint}$ is the area-averaged concentration within the sensor footprint,
$\rho_{oil} = 0.05$, $\rho_{water} = 0.40$ are the reflectance values (0–1 scale), and
$\eta \sim \mathcal{N}(0, \sigma_{opt}^2)$ is optical noise with $\sigma_{opt} = 0.01$.
The oil detection flag is:

$$\hat{O}(\mathbf{p}_d) = \mathbf{1}\!\left[\rho(\mathbf{p}_d) \leq \rho_{thresh}\right], \qquad \rho_{thresh} = 0.20$$

### Contour-Following Guidance Law

The drone executes a boundary-hugging zigzag: it steers so that its sensor footprint straddles
the oil/water boundary, alternating between two sub-modes depending on the current detection flag:

| Detection flag | Action |
|----------------|--------|
| $\hat{O} = 1$ (oil detected) | Turn toward clean water: apply yaw rate $+\omega_{turn}$ |
| $\hat{O} = 0$ (clean water detected) | Turn toward oil: apply yaw rate $-\omega_{turn}$ |

The heading $\psi$ and position update in polar form:

$$\psi_{t+1} = \psi_t + s(\hat{O}_t) \cdot \omega_{turn} \cdot \Delta t, \qquad
s(\hat{O}) = \begin{cases} +1 & \hat{O} = 0 \\ -1 & \hat{O} = 1 \end{cases}$$

$$\mathbf{p}_{t+1} = \mathbf{p}_t + v_{cruise}\,(\cos\psi_t,\,\sin\psi_t)^T\,\Delta t$$

This produces a boundary-tracing sinusoid whose wavelength $\lambda \approx \pi v_{cruise} / \omega_{turn}$
controls tracking fidelity. The drone crosses the boundary roughly every
$\Delta s = v_{cruise} / \omega_{turn}$ metres of lateral motion.

### Contour Polygon Lap Detection

A lap is completed when the drone returns within $r_{lap} = 5$ m of its lap-start position after
having traversed at least $L_{min} = 50$ m. At each lap completion:

1. The contour polygon is extracted from the estimated concentration grid using marching squares.
2. The polygon area and centroid are computed (see below).
3. The Kalman filter is updated with the new centroid measurement.
4. A T = 60 s centroid prediction is issued.

### Spill Area Estimation (Shoelace Formula)

Given the ordered boundary polygon vertices
$\{(x_1, y_1), \ldots, (x_M, y_M)\}$ extracted from the concentration grid:

$$A(t) = \frac{1}{2} \left|\sum_{i=1}^{M} \bigl(x_i \, y_{i+1} - x_{i+1} \, y_i\bigr)\right|$$

with indices taken modulo $M$. The polygon centroid:

$$x_c = \frac{1}{6A} \sum_{i=1}^{M} (x_i + x_{i+1})(x_i y_{i+1} - x_{i+1} y_i)$$

$$y_c = \frac{1}{6A} \sum_{i=1}^{M} (y_i + y_{i+1})(x_i y_{i+1} - x_{i+1} y_i)$$

### Kalman Filter Boundary Centroid Prediction

The spill centroid $\mathbf{x}_c = (x_c, y_c)^T$ is modelled as a constant-velocity linear system:

$$\mathbf{s}_t = \begin{pmatrix} x_c \\ y_c \\ \dot{x}_c \\ \dot{y}_c \end{pmatrix}, \qquad
\mathbf{s}_{t+1} = A\,\mathbf{s}_t + \mathbf{q}_t$$

$$A = \begin{pmatrix} I_2 & \Delta\tau \cdot I_2 \\ 0 & I_2 \end{pmatrix}, \qquad
\mathbf{q}_t \sim \mathcal{N}(\mathbf{0}, Q)$$

where $\Delta\tau$ is the inter-lap interval (s), $Q = \sigma_q^2 \operatorname{diag}(1,1,1,1)$
is the process noise covariance ($\sigma_q = 0.1$ m or m/s per lap), and the measurement model
is $\mathbf{z}_t = H\,\mathbf{s}_t + \mathbf{r}_t$ with $H = [I_2 \mid 0]$ and
$\mathbf{r}_t \sim \mathcal{N}(\mathbf{0}, R)$, $R = \sigma_r^2 I_2$, $\sigma_r = 1.0$ m.

**Prediction** $T_{pred} = 60$ s ahead at lap $k$:

$$\hat{\mathbf{s}}_{k+T_{pred}/\Delta\tau \mid k} = A^{\lfloor T_{pred}/\Delta\tau \rfloor}\,\hat{\mathbf{s}}_{k\mid k}$$

The predicted centroid $(\hat{x}_c, \hat{y}_c)$ is the upper two components of the predicted state.

### Performance Metrics

**Boundary tracking lag** — mean displacement between the true spill centroid and the most recent
contour-polygon centroid:

$$\varepsilon_{lag}(t) = \|\mathbf{c}_{true}(t) - \mathbf{c}_{polygon}(t_{lap})\|$$

where $t_{lap}$ is the time of the most recently completed lap.

**Prediction error** at lap $k$:

$$\varepsilon_{pred}^{(k)} = \|\hat{\mathbf{c}}_{k+T_{pred}/\Delta\tau} - \mathbf{c}_{true}(t_k + T_{pred})\|$$

**Area estimation error**:

$$\varepsilon_A(t) = \frac{|A_{polygon}(t) - A_{true}(t)|}{A_{true}(t)} \times 100\%$$

where $A_{true}(t)$ is the true area of the $C > C_{thresh}$ region computed directly from the grid.

---

## Implementation

```python
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.contour import QuadContourSet

# ── Domain ──────────────────────────────────────────────────────────────────
LX, LY       = 200.0, 100.0      # m — domain width and height
GRID_DX      = 0.5               # m — grid cell size (same in x and y)
NX           = int(LX / GRID_DX)
NY           = int(LY / GRID_DX)

# ── Spill physics ────────────────────────────────────────────────────────────
V_CURRENT    = np.array([0.15, 0.05])  # m/s — ocean current (x, y)
D_DIFF       = 0.3                     # m^2/s — turbulent diffusion coefficient
C_THRESH     = 0.10                    # concentration detection threshold
C_INIT_PEAK  = 1.0                     # peak concentration at t=0
R_INIT       = 15.0                    # m — initial spill radius (Gaussian sigma)
SPILL_CENTRE = np.array([60.0, 50.0]) # m — initial spill centre

# ── Optical sensor ───────────────────────────────────────────────────────────
RHO_OIL      = 0.05    # reflectance over pure oil
RHO_WATER    = 0.40    # reflectance over clean water
RHO_THRESH   = 0.20    # oil/water classification threshold
SIGMA_OPT    = 0.01    # optical noise std
SENSOR_R     = 1.5     # m — sensor footprint radius

# ── Drone dynamics ───────────────────────────────────────────────────────────
V_CRUISE     = 2.0     # m/s — cruise speed
OMEGA_TURN   = 0.5     # rad/s — contour-following yaw rate
Z_FLY        = 8.0     # m — fixed altitude

# ── Lap detection ────────────────────────────────────────────────────────────
R_LAP        = 5.0     # m — lap-closure radius
L_MIN_LAP    = 50.0    # m — minimum path length to count as a lap

# ── Kalman filter ────────────────────────────────────────────────────────────
SIGMA_Q      = 0.1     # process noise std (m or m/s per lap)
SIGMA_R_KF   = 1.0     # centroid measurement noise std (m)
T_PRED       = 60.0    # s — prediction horizon

# ── Simulation ───────────────────────────────────────────────────────────────
DT           = 0.2     # s — simulation timestep
T_SIM        = 300.0   # s — total simulation duration


def init_concentration():
    """Gaussian blob initial condition centred at SPILL_CENTRE."""
    xs = (np.arange(NX) + 0.5) * GRID_DX
    ys = (np.arange(NY) + 0.5) * GRID_DX
    X, Y = np.meshgrid(xs, ys, indexing='ij')
    dist2 = (X - SPILL_CENTRE[0])**2 + (Y - SPILL_CENTRE[1])**2
    return C_INIT_PEAK * np.exp(-dist2 / (2.0 * R_INIT**2))


def advect_diffuse(C, dt):
    """One explicit finite-difference step of advection-diffusion."""
    vx, vy = V_CURRENT

    # Central-difference advection (interior only; boundary uses zero-flux)
    dCdx = np.zeros_like(C)
    dCdy = np.zeros_like(C)
    dCdx[1:-1, :] = (C[2:, :] - C[:-2, :]) / (2.0 * GRID_DX)
    dCdy[:, 1:-1] = (C[:, 2:] - C[:, :-2]) / (2.0 * GRID_DX)

    # Five-point Laplacian
    lap = np.zeros_like(C)
    lap[1:-1, 1:-1] = (
        C[2:,  1:-1] + C[:-2, 1:-1] +
        C[1:-1, 2:] + C[1:-1, :-2] -
        4.0 * C[1:-1, 1:-1]
    ) / GRID_DX**2

    C_new = C - dt * (vx * dCdx + vy * dCdy) + D_DIFF * dt * lap
    return np.clip(C_new, 0.0, 1.0)


def optical_reading(C_field, drone_pos, rng):
    """Return reflectance measurement and oil detection flag."""
    xs = (np.arange(NX) + 0.5) * GRID_DX
    ys = (np.arange(NY) + 0.5) * GRID_DX
    X, Y = np.meshgrid(xs, ys, indexing='ij')
    dist = np.hypot(X - drone_pos[0], Y - drone_pos[1])
    footprint = dist <= SENSOR_R

    C_footprint_mean = C_field[footprint].mean() if footprint.any() else 0.0
    rho = (RHO_OIL * C_footprint_mean
           + RHO_WATER * (1.0 - C_footprint_mean)
           + rng.normal(0.0, SIGMA_OPT))
    oil_detected = rho <= RHO_THRESH
    return rho, oil_detected


def shoelace_area_centroid(vertices):
    """Compute area and centroid of a closed polygon (Nx2 array)."""
    x = vertices[:, 0]
    y = vertices[:, 1]
    cross = x[:-1] * y[1:] - x[1:] * y[:-1]
    A = 0.5 * abs(cross.sum())
    if A < 1e-6:
        return A, np.mean(vertices, axis=0)
    cx = np.sum((x[:-1] + x[1:]) * cross) / (6.0 * A)
    cy = np.sum((y[:-1] + y[1:]) * cross) / (6.0 * A)
    return A, np.array([cx, cy])


def extract_contour_polygon(C_field):
    """Extract the C_THRESH level-set polygon using matplotlib's contour finder."""
    xs = (np.arange(NX) + 0.5) * GRID_DX
    ys = (np.arange(NY) + 0.5) * GRID_DX
    fig_tmp, ax_tmp = plt.subplots()
    cs = ax_tmp.contour(xs, ys, C_field.T, levels=[C_THRESH])
    plt.close(fig_tmp)
    paths = cs.collections[0].get_paths() if cs.collections else []
    if not paths:
        return None
    # Return the longest path (main spill boundary)
    longest = max(paths, key=lambda p: len(p.vertices))
    return longest.vertices   # shape (M, 2)


def true_spill_area(C_field):
    """True area of C > C_THRESH region from the grid."""
    return np.sum(C_field > C_THRESH) * GRID_DX**2


def true_spill_centroid(C_field):
    """Centroid of C > C_THRESH region."""
    xs = (np.arange(NX) + 0.5) * GRID_DX
    ys = (np.arange(NY) + 0.5) * GRID_DX
    X, Y = np.meshgrid(xs, ys, indexing='ij')
    mask = C_field > C_THRESH
    if not mask.any():
        return np.array([LX / 2, LY / 2])
    return np.array([X[mask].mean(), Y[mask].mean()])


class BoundaryCentroidKF:
    """Linear Kalman filter on 4D state [x_c, y_c, vx_c, vy_c]."""

    def __init__(self, init_centroid, dt_lap):
        self.dt = dt_lap
        A_sub = np.array([[1, 0, dt_lap, 0],
                          [0, 1, 0, dt_lap],
                          [0, 0, 1,      0],
                          [0, 0, 0,      1]])
        self.A = A_sub
        self.H = np.array([[1, 0, 0, 0],
                           [0, 1, 0, 0]])
        self.Q = SIGMA_Q**2 * np.eye(4)
        self.R = SIGMA_R_KF**2 * np.eye(2)
        self.x = np.array([init_centroid[0], init_centroid[1], 0.0, 0.0])
        self.P = np.eye(4) * 10.0

    def predict(self):
        self.x = self.A @ self.x
        self.P = self.A @ self.P @ self.A.T + self.Q

    def update(self, z):
        y  = z - self.H @ self.x
        S  = self.H @ self.P @ self.H.T + self.R
        K  = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ self.H) @ self.P

    def predict_ahead(self, n_steps):
        """Predict centroid n_steps laps ahead."""
        An = np.linalg.matrix_power(self.A, n_steps)
        x_pred = An @ self.x
        return x_pred[:2]


def run_simulation(seed=0):
    rng = np.random.default_rng(seed)

    C = init_concentration()

    # Drone initial state: place on known boundary, heading east
    pos = np.array([SPILL_CENTRE[0] + R_INIT, SPILL_CENTRE[1]])
    heading = 0.0  # radians (east)
    lap_start = pos.copy()
    path_len_since_lap = 0.0

    # KF initialised at first centroid
    kf = BoundaryCentroidKF(true_spill_centroid(C), dt_lap=30.0)

    # Logs
    traj = [pos.copy()]
    t_axis = []
    true_areas = []
    poly_areas = []
    true_centroids = []
    poly_centroids = []
    kf_predictions = []     # (t_issued, predicted_centroid, t_target)
    pred_errors = []
    lap_times = []

    n_steps = int(T_SIM / DT)

    for step in range(n_steps):
        t = step * DT

        # 1. Advance spill
        C = advect_diffuse(C, DT)

        # 2. Sense
        _, oil_flag = optical_reading(C, pos, rng)

        # 3. Contour-following yaw update
        s = -1.0 if oil_flag else +1.0
        heading += s * OMEGA_TURN * DT

        # 4. Move
        vel = V_CRUISE * np.array([np.cos(heading), np.sin(heading)])
        new_pos = pos + vel * DT
        new_pos = np.clip(new_pos, [0, 0], [LX, LY])
        step_len = np.linalg.norm(new_pos - pos)
        pos = new_pos
        path_len_since_lap += step_len
        traj.append(pos.copy())

        # 5. Lap detection
        dist_to_start = np.linalg.norm(pos - lap_start)
        if dist_to_start < R_LAP and path_len_since_lap > L_MIN_LAP:
            # Lap completed — extract contour polygon
            verts = extract_contour_polygon(C)
            if verts is not None and len(verts) >= 3:
                poly_A, poly_c = shoelace_area_centroid(verts)
            else:
                poly_A = np.nan
                poly_c = np.array([np.nan, np.nan])

            # KF update
            if not np.isnan(poly_c[0]):
                kf.update(poly_c)
            kf.predict()

            # Predict T_PRED seconds ahead
            n_ahead = max(1, int(T_PRED / 30.0))
            c_pred = kf.predict_ahead(n_ahead)
            t_target = t + T_PRED
            kf_predictions.append((t, c_pred.copy(), t_target))

            lap_times.append(t)
            poly_areas.append(poly_A)
            poly_centroids.append(poly_c.copy())

            # Reset lap
            lap_start = pos.copy()
            path_len_since_lap = 0.0

        # 6. Log ground-truth metrics
        t_axis.append(t)
        true_areas.append(true_spill_area(C))
        true_centroids.append(true_spill_centroid(C))

    # Compute prediction errors retrospectively
    t_axis_arr = np.array(t_axis)
    true_centroids_arr = np.array(true_centroids)
    for t_issued, c_pred, t_target in kf_predictions:
        idx = np.argmin(np.abs(t_axis_arr - t_target))
        err = np.linalg.norm(c_pred - true_centroids_arr[idx])
        pred_errors.append(err)

    return {
        "C_final"          : C,
        "trajectory"       : np.array(traj),
        "t_axis"           : t_axis_arr,
        "true_areas"       : np.array(true_areas),
        "poly_areas"       : np.array(poly_areas),
        "true_centroids"   : true_centroids_arr,
        "poly_centroids"   : np.array(poly_centroids) if poly_centroids else np.empty((0, 2)),
        "kf_predictions"   : kf_predictions,
        "pred_errors"      : np.array(pred_errors),
        "lap_times"        : np.array(lap_times),
    }
```

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Domain size | $200 \times 100$ m |
| Grid cell size $\delta$ | 0.5 m |
| Ocean current $\mathbf{v}_{current}$ | $(0.15,\;0.05)$ m/s |
| Diffusion coefficient $D$ | 0.3 m$^2$/s |
| Detection threshold $C_{thresh}$ | 0.10 |
| Initial spill radius $r_0$ | 15 m |
| Initial spill centre | $(60, 50)$ m |
| Drone cruise speed $v_{cruise}$ | 2.0 m/s |
| Drone altitude $z$ | 8.0 m |
| Contour-following yaw rate $\omega_{turn}$ | 0.5 rad/s |
| Sensor footprint radius $r_s$ | 1.5 m |
| Optical noise $\sigma_{opt}$ | 0.01 |
| Reflectance threshold $\rho_{thresh}$ | 0.20 |
| Lap-closure radius $r_{lap}$ | 5.0 m |
| Min lap path length $L_{min}$ | 50.0 m |
| KF process noise $\sigma_q$ | 0.1 m (or m/s) per lap |
| KF measurement noise $\sigma_r$ | 1.0 m |
| Prediction horizon $T_{pred}$ | 60 s |
| Simulation timestep $\Delta t$ | 0.2 s |
| Simulation duration $T_{sim}$ | 300 s |

---

## Expected Output

- **Spill evolution snapshots**: four 2D top-down colour maps of $C(x, y)$ at $t = 0, 100, 200, 300$ s,
  with the $C_{thresh}$ contour overlaid in red, the drone position marked in orange, and the ocean
  current arrow in the upper corner. Colourbar from 0 to 1 (concentration).
- **Drone trajectory plot**: full drone path colour-coded from blue (early) to red (late), superimposed
  on the final concentration field; lap-start markers shown as circles; KF prediction arrows drawn
  from each lap's centroid toward the predicted 60 s position.
- **Spill area vs time**: dual plot showing true area $A_{true}(t)$ (solid line) and per-lap polygon
  area estimate $A_{polygon}$ (scatter markers) over time; relative area error $\varepsilon_A$ on a
  secondary axis.
- **Boundary centroid tracking plot**: $x_c(t)$ and $y_c(t)$ for the true centroid (solid), the
  Kalman-filtered estimate at each lap (dashed), and the $T_{pred} = 60$ s prediction (dotted arrow
  tips); centroid tracking lag $\varepsilon_{lag}$ annotated.
- **Kalman prediction error bar chart**: per-lap prediction error $\varepsilon_{pred}^{(k)}$ (m)
  shown as a bar chart over lap index; mean error annotated with a dashed horizontal line.
- **Animation (GIF)**: top-down spreading spill (colour fill evolving), drone icon tracing the
  boundary in real time with sensor footprint circle, $C_{thresh}$ contour updating each frame,
  and current KF centroid prediction arrow updating at each lap.

---

## Extensions

1. **Wind-shifted current**: introduce a sudden 45-degree current direction change at $t = 150$ s
   (e.g. tidal reversal); measure how quickly the Kalman filter detects the velocity shift and how
   the contour-follower loses and re-acquires the boundary under the accelerated lateral drift.
2. **Multi-drone relay**: deploy $N = 2$ drones on opposite sides of the spill boundary so that
   each covers half the perimeter; compare per-lap completion time and tracking lag against the
   single-drone baseline; analyse how much the lap interval shortens as a function of $N$.
3. **Non-uniform diffusion**: introduce a coastal shallows region (left half of the domain) with
   $D_{shallow} = 0.05$ m$^2$/s and open water (right half) with $D_{open} = 0.6$ m$^2$/s;
   observe asymmetric spill shape and evaluate shoelace area accuracy on a non-convex polygon.
4. **Dispersant pre-positioning**: use the KF 60 s centroid prediction to autonomously command a
   second (dispersant) drone to arrive at the predicted boundary leading edge before the spill does;
   quantify reaction time saved vs a reactive strategy that waits for the boundary to be confirmed.
5. **EKF nonlinear state**: replace the constant-velocity KF with an EKF whose state includes
   centroid position, velocity, and spill radius; incorporate the shoelace area measurement as an
   additional observation to improve area-growth rate estimation.

---

## Related Scenarios

- Prerequisites: [S041 Wildfire Boundary Scan](S041_wildfire_boundary.md) (boundary-following logic
  and coverage freshness metrics), [S045 Chemical Plume Tracing](S045_plume_tracing.md) (scalar
  advection field and contour detection)
- Follow-ups: [S056 Radiation Hotspot Detection](S056_radiation_hotspot.md) (similar gradient-field
  tracking, point-source variant)
- Algorithmic cross-reference: [S041 Wildfire Boundary](S041_wildfire_boundary.md) (gradient-following
  guidance law), [S045 Plume Tracing](S045_plume_tracing.md) (advection-diffusion PDE and sensor
  model structure)
