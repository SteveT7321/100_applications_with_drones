# S059 Underwater Target Sonar Marking — Surface Cooperative Localization

**Domain**: Environmental Monitoring & SAR | **Difficulty**: ⭐⭐⭐ | **Status**: `[ ]` Not Started
**Algorithm**: TDOA Trilateration + WLS + D-Optimal Drone Placement | **Dimension**: 2D

---

## Problem Definition

**Setup**: An underwater sonar buoy (emergency beacon, submerged wreckage transponder, or marine
animal tag) at an unknown 3D position $\mathbf{x}_t = (x_t, y_t, z_t)$ periodically emits a
broadband acoustic ping. Four surface drones, each equipped with a hydrophone, float or hover
at known $(x, y)$ positions on the water surface (fixed depth $z = 0$). Each drone records the
arrival time of the same ping; from these four time-of-arrival (TOA) measurements the mission
control system forms three time-difference-of-arrival (TDOA) observations and solves iteratively
for the full 3D target position. Because surface drones can reposition between ping epochs, the
drone spreading policy is adapted on-line to minimise the Geometric Dilution of Precision (GDOP)
of the next estimate.

**Roles**:
- **Surface Drones** ($N = 4$): mobile hydrophone platforms at known 2D positions
  $\mathbf{p}_i = (x_i, y_i, 0)$; each records the arrival time $t_i$ of every sonar ping.
- **Sonar Buoy / Underwater Target**: passive acoustic emitter at unknown depth $z_t < 0$ and
  unknown horizontal position $(x_t, y_t)$; emits one ping per epoch.
- **Mission Control**: fuses TDOA measurements from all four drones; runs the WLS solver and
  issues new waypoints to the drones before the next epoch.

**Objective**: Estimate the 3D buoy position $\hat{\mathbf{x}}_t$ from TDOA measurements using
iterative weighted least squares (WLS), and simultaneously optimise drone surface positions to
minimise GDOP (D-optimal design), repeating over multiple ping epochs until the position error
falls below a target threshold.

---

## Mathematical Model

### Range and TOA Model

The true slant range from drone $i$ at surface position $\mathbf{p}_i = (x_i, y_i, 0)$ to the
underwater target at $\mathbf{x}_t = (x_t, y_t, z_t)$ is:

$$d_i = \|\mathbf{p}_i - \mathbf{x}_t\|_2 = \sqrt{(x_i - x_t)^2 + (y_i - y_t)^2 + z_t^2}$$

The time of arrival at drone $i$ for a ping emitted at time $t_0$ is:

$$t_i = t_0 + \frac{d_i}{c_s} + \eta_i, \qquad \eta_i \sim \mathcal{N}(0,\, \sigma_\tau^2)$$

where $c_s = 1500$ m/s is the speed of sound in water and $\sigma_\tau$ is the TOA measurement
noise standard deviation.

### TDOA Measurement Model

Using drone 1 as the reference receiver, the TDOA between drone $i$ and drone 1 is:

$$\Delta\tau_{i1} = t_i - t_1 = \frac{d_i - d_1}{c_s} + \varepsilon_{i1}$$

where the differential noise $\varepsilon_{i1} \sim \mathcal{N}(0,\, 2\sigma_\tau^2)$ (since TOA
noise at each receiver is independent). For $N = 4$ drones this yields $N - 1 = 3$ independent
TDOA observations:

$$\boldsymbol{\Delta\tau} = \begin{bmatrix} \Delta\tau_{21} \\ \Delta\tau_{31} \\ \Delta\tau_{41} \end{bmatrix}
\in \mathbb{R}^{N-1}$$

Multiplying through by $c_s$, define the range-difference observations $\rho_{i1} = c_s \Delta\tau_{i1}$:

$$\rho_{i1} = d_i - d_1 + \xi_{i1}, \qquad \xi_{i1} \sim \mathcal{N}(0,\, 2c_s^2\sigma_\tau^2)$$

### Linearisation via Taylor Expansion (Iterative WLS)

Given a current estimate $\hat{\mathbf{x}}^{(k)} = (\hat{x}_t, \hat{y}_t, \hat{z}_t)^{(k)}$,
define the predicted range-differences:

$$\hat{\rho}_{i1}^{(k)} = \hat{d}_i^{(k)} - \hat{d}_1^{(k)}, \qquad
\hat{d}_i^{(k)} = \|\mathbf{p}_i - \hat{\mathbf{x}}^{(k)}\|$$

The residual vector $\mathbf{b} \in \mathbb{R}^{N-1}$ is:

$$b_i = \rho_{i1} - \hat{\rho}_{i1}^{(k)}, \quad i = 2, 3, 4$$

The geometry (Jacobian) matrix $\mathbf{H} \in \mathbb{R}^{(N-1) \times 3}$ is obtained by
linearising $d_i - d_1$ around $\hat{\mathbf{x}}^{(k)}$:

$$H_{i,j} = \frac{\partial(d_i - d_1)}{\partial x_{t,j}}\bigg|_{\hat{\mathbf{x}}^{(k)}}
= -\frac{(p_{i,j} - \hat{x}_{t,j}^{(k)})}{\hat{d}_i^{(k)}}
  + \frac{(p_{1,j} - \hat{x}_{t,j}^{(k)})}{\hat{d}_1^{(k)}}$$

where $j \in \{x, y, z\}$. The linearised system is:

$$\mathbf{H} \,\Delta\mathbf{x} = \mathbf{b}$$

with $\Delta\mathbf{x} = \mathbf{x}_t - \hat{\mathbf{x}}^{(k)}$ the position correction.

### Weighted Least Squares Solution

Assuming independent TDOA noise with equal variance $\sigma_\xi^2 = 2c_s^2\sigma_\tau^2$, the
weight matrix simplifies to $\mathbf{W} = \sigma_\xi^{-2}\mathbf{I}$. The WLS update is:

$$\Delta\hat{\mathbf{x}} = (\mathbf{H}^T \mathbf{W} \mathbf{H})^{-1} \mathbf{H}^T \mathbf{W} \mathbf{b}$$

$$\hat{\mathbf{x}}^{(k+1)} = \hat{\mathbf{x}}^{(k)} + \Delta\hat{\mathbf{x}}$$

Convergence is declared when $\|\Delta\hat{\mathbf{x}}\|_2 < \epsilon_{tol}$ or after $K_{max}$
iterations. This procedure is repeated each ping epoch using the previous epoch's converged
estimate as the warm start.

### Geometric Dilution of Precision (GDOP)

The GDOP characterises how the surface geometry of the four drones amplifies TDOA noise into
3D position error. Using the geometry matrix $\mathbf{H}$ defined above:

$$\text{GDOP} = \sqrt{\text{tr}\!\left[(\mathbf{H}^T \mathbf{H})^{-1}\right]}$$

The 3D position error covariance is:

$$\boldsymbol{\Sigma}_{\hat{\mathbf{x}}} = \sigma_\xi^2\,(\mathbf{H}^T \mathbf{H})^{-1}$$

and the 3D RMS position error bound is:

$$\sigma_{pos} = \sigma_\xi \cdot \text{GDOP}$$

A GDOP map is computed by evaluating $\text{GDOP}(\mathbf{x})$ on a 2D horizontal grid at the
estimated depth $\hat{z}_t$, revealing the zone of minimum uncertainty centred on the drone
formation.

### D-Optimal Drone Spreading Policy

Between ping epochs the drones reposition to minimise GDOP at the current best estimate
$\hat{\mathbf{x}}^{(k)}$. This is equivalent to maximising the determinant of the information
matrix (D-optimal experimental design):

$$\mathbf{p}_1^*, \ldots, \mathbf{p}_4^* = \arg\max_{\{\mathbf{p}_i\}} \det\!\left(\mathbf{H}^T \mathbf{W} \mathbf{H}\right)$$

subject to:
- Each drone stays within the operating area $\mathcal{A} = [-R_{max}, R_{max}]^2$.
- Minimum inter-drone separation $d_{sep}$ to avoid collisions.

The optimal heuristic is to spread the four drones uniformly around a circle of radius
$R_{spread}$ centred on the horizontal projection of $\hat{\mathbf{x}}^{(k)}$:

$$\mathbf{p}_i = \hat{\mathbf{x}}_{xy}^{(k)} + R_{spread}\begin{pmatrix}\cos\!\bigl(\tfrac{2\pi(i-1)}{4}\bigr) \\ \sin\!\bigl(\tfrac{2\pi(i-1)}{4}\bigr)\end{pmatrix}, \quad i = 1,2,3,4$$

This four-way symmetric layout minimises $\text{tr}[(\mathbf{H}^T\mathbf{H})^{-1}]$ for a target
at the geometric centre and is compared against a random initial placement and a compact cluster
(poor geometry) in simulation.

---

## Implementation

```python
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation

# ── Physical constants ──────────────────────────────────────────────────────
C_SOUND     = 1500.0    # m/s — speed of sound in water
SIGMA_TAU   = 1e-4      # s  — per-hydrophone TOA noise std dev
SIGMA_XI    = np.sqrt(2) * C_SOUND * SIGMA_TAU   # m — range-diff noise std dev

# ── Scenario parameters ─────────────────────────────────────────────────────
N_DRONES    = 4
N_EPOCHS    = 20        # ping epochs (drone repositioning iterations)
K_MAX       = 50        # max WLS iterations per epoch
EPSILON_TOL = 1e-3      # m — WLS convergence threshold
R_SPREAD    = 80.0      # m — drone spreading radius
R_MAX       = 150.0     # m — operating area half-width
D_SEP       = 10.0      # m — minimum inter-drone separation
N_MC        = 200       # Monte Carlo trials for GDOP validation

# ── True target position ────────────────────────────────────────────────────
X_TRUE = np.array([20.0, -15.0, -30.0])   # (x, y, z) m; z < 0 = underwater

def compute_ranges(drone_positions, x_target):
    """True slant ranges from each surface drone to the underwater target."""
    return np.linalg.norm(drone_positions - x_target, axis=1)   # (N,)

def simulate_tdoa(drone_positions, x_target, sigma_tau, rng):
    """Simulate noisy TDOA observations (range-differences) using drone 1 as reference."""
    true_ranges = compute_ranges(drone_positions, x_target)
    # Noisy TOA for each drone
    toa_noise = rng.normal(0, sigma_tau, size=N_DRONES)
    toa = true_ranges / C_SOUND + toa_noise
    # TDOA w.r.t. drone 1 → multiply by c_s → range-differences
    rdiff = C_SOUND * (toa[1:] - toa[0])   # shape (N-1,)
    return rdiff

def build_geometry_matrix(drone_positions, x_hat):
    """
    Build Jacobian H ∈ R^{(N-1) x 3} of range-differences w.r.t. target position,
    linearised around current estimate x_hat.
    """
    diffs = drone_positions - x_hat          # (N, 3)
    ranges = np.linalg.norm(diffs, axis=1)   # (N,)
    # Unit vectors from target to each drone (gradient of d_i w.r.t. x_t is negative)
    unit = -diffs / (ranges[:, None] + 1e-12)  # (N, 3)
    # H[i-1, :] = grad(d_i - d_1) = unit[i] - unit[0],  i = 1..N-1
    H = unit[1:] - unit[0]                   # (N-1, 3)
    return H

def wls_solve(H, b, W=None):
    """Weighted least squares: Δx = (H^T W H)^{-1} H^T W b."""
    if W is None:
        W = np.eye(len(b))
    HtW  = H.T @ W
    HtWH = HtW @ H
    HtWb = HtW @ b
    delta, _, _, _ = np.linalg.lstsq(HtWH, HtWb, rcond=None)
    return delta

def iterative_wls(drone_positions, rdiff_obs, x_init,
                  sigma_xi=SIGMA_XI, k_max=K_MAX, tol=EPSILON_TOL):
    """Iterative WLS solver for TDOA-based 3D localisation."""
    x_hat = x_init.copy()
    W = (1.0 / sigma_xi**2) * np.eye(N_DRONES - 1)

    for _ in range(k_max):
        # Predicted range-differences
        ranges_hat = np.linalg.norm(drone_positions - x_hat, axis=1)
        rdiff_hat  = ranges_hat[1:] - ranges_hat[0]
        b          = rdiff_obs - rdiff_hat

        H     = build_geometry_matrix(drone_positions, x_hat)
        delta = wls_solve(H, b, W)
        x_hat = x_hat + delta

        if np.linalg.norm(delta) < tol:
            break

    return x_hat

def compute_gdop(drone_positions, x_target):
    """GDOP = sqrt(trace((H^T H)^{-1})) evaluated at x_target."""
    H   = build_geometry_matrix(drone_positions, x_target)
    HtH = H.T @ H
    try:
        cov  = np.linalg.inv(HtH)
        return np.sqrt(np.trace(cov))
    except np.linalg.LinAlgError:
        return np.inf

def d_optimal_positions(x_hat_xy, r_spread=R_SPREAD, r_max=R_MAX):
    """Place 4 drones uniformly on a circle around the estimated horizontal position."""
    angles = np.linspace(0, 2 * np.pi, N_DRONES, endpoint=False)
    positions = np.zeros((N_DRONES, 3))
    for i, a in enumerate(angles):
        px = x_hat_xy[0] + r_spread * np.cos(a)
        py = x_hat_xy[1] + r_spread * np.sin(a)
        # Clip to operating area
        px = np.clip(px, -r_max, r_max)
        py = np.clip(py, -r_max, r_max)
        positions[i] = [px, py, 0.0]
    return positions

def gdop_map(drone_positions, z_depth, grid_half=100.0, n_pts=40):
    """Compute GDOP on a 2D horizontal grid at given depth for visualisation."""
    xs = np.linspace(-grid_half, grid_half, n_pts)
    ys = np.linspace(-grid_half, grid_half, n_pts)
    XX, YY = np.meshgrid(xs, ys)
    GG = np.zeros_like(XX)
    for i in range(n_pts):
        for j in range(n_pts):
            pt = np.array([XX[i, j], YY[i, j], z_depth])
            GG[i, j] = compute_gdop(drone_positions, pt)
    return XX, YY, GG

def run_simulation():
    """Main loop: iterate over ping epochs, refine position, reposition drones."""
    rng = np.random.default_rng(42)

    # Initial drone placement: random compact cluster (poor geometry)
    init_positions = np.column_stack([
        rng.uniform(-20, 20, N_DRONES),
        rng.uniform(-20, 20, N_DRONES),
        np.zeros(N_DRONES),
    ])

    # Initial estimate: surface-level, zero horizontal offset
    x_init = np.array([0.0, 0.0, -10.0])

    drone_positions = init_positions.copy()
    x_hat = x_init.copy()
    errors, gdops, drone_history = [], [], [drone_positions.copy()]

    for epoch in range(N_EPOCHS):
        # 1. Simulate TDOA ping measurement
        rdiff_obs = simulate_tdoa(drone_positions, X_TRUE, SIGMA_TAU, rng)

        # 2. Iterative WLS to update position estimate
        x_hat = iterative_wls(drone_positions, rdiff_obs, x_hat)

        # 3. Record metrics
        error = np.linalg.norm(x_hat - X_TRUE)
        gdop  = compute_gdop(drone_positions, x_hat)
        errors.append(error)
        gdops.append(gdop)

        # 4. D-optimal repositioning for next epoch
        drone_positions = d_optimal_positions(x_hat[:2])
        drone_history.append(drone_positions.copy())

    return {
        "errors":        np.array(errors),
        "gdops":         np.array(gdops),
        "drone_history": drone_history,
        "x_hat_final":   x_hat,
        "x_true":        X_TRUE,
    }
```

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Number of surface drones $N$ | 4 |
| Speed of sound $c_s$ | 1500 m/s |
| TOA noise std dev $\sigma_\tau$ | $10^{-4}$ s ($\approx$ 0.15 m range-equivalent) |
| Range-difference noise $\sigma_\xi = \sqrt{2}\,c_s\sigma_\tau$ | $\approx 0.212$ m |
| True target position $\mathbf{x}_t$ | $(20,\,-15,\,-30)$ m |
| Initial depth guess | $-10$ m |
| Drone spreading radius $R_{spread}$ | 80 m |
| Operating area half-width $R_{max}$ | 150 m |
| Minimum inter-drone separation $d_{sep}$ | 10 m |
| WLS convergence threshold $\epsilon_{tol}$ | $10^{-3}$ m |
| Max WLS iterations per epoch $K_{max}$ | 50 |
| Number of ping epochs | 20 |
| Monte Carlo trials (GDOP validation) | 200 |
| Expected final position error | $< 2$ m |
| Expected GDOP (D-optimal layout) | $\approx 2.0$ – $3.5$ |
| Expected GDOP (compact/clustered layout) | $> 10$ |

---

## Expected Output

- **Drone geometry + GDOP contour map**: top-down 2D plot of the operating area showing the
  initial (compact) and final (D-optimal circle) drone positions; GDOP contours computed on a
  40 x 40 horizontal grid at depth $\hat{z}_t$; horizontal target position marked as a green
  star; contour colour scale clipped at GDOP = 10 to highlight the low-GDOP zone.
- **Localisation error vs epoch**: line plot of 3D position error $\|\hat{\mathbf{x}}^{(k)} - \mathbf{x}_t\|$
  vs ping epoch number; shows rapid initial drop after D-optimal repositioning takes effect;
  CRLB envelope $\sigma_\xi \cdot \text{GDOP}$ overlaid as a dashed line.
- **GDOP vs epoch**: secondary line plot tracking GDOP at the current estimate across all epochs;
  illustrates the one-epoch lag between repositioning and the resulting GDOP improvement.
- **Animation**: 2D top-down animation of the operating area across 20 epochs; drone markers
  (red circles) migrate to the circle layout; the estimated target position (blue cross)
  converges toward the true position (green star); GDOP contour background updated each frame.
- **Metrics summary**: printed table or annotation box with final values — 3D position error,
  horizontal error $\sqrt{\Delta x^2 + \Delta y^2}$, depth error $|\Delta z|$, final GDOP,
  number of WLS iterations at convergence.

---

## Extensions

1. **Heterogeneous noise model**: assign each hydrophone drone an individual noise level
   $\sigma_{\tau,i}$ (e.g., proportional to depth or sea-state); derive the full non-diagonal
   weight matrix $\mathbf{W} = \text{diag}(2c_s^2\sigma_{\tau,i}^2 + 2c_s^2\sigma_{\tau,1}^2)^{-1}$
   and show the improvement over the uniform-weight WLS.
2. **Moving target tracking**: allow the buoy to drift slowly at 0.1 m/s; incorporate a
   constant-velocity kinematic model and replace the WLS solver with an extended Kalman filter
   (EKF) that propagates position between pings.
3. **Depth ambiguity analysis**: investigate the condition number of $\mathbf{H}^T\mathbf{H}$
   as a function of depth $z_t$ and horizontal drone spread $R_{spread}$; identify the
   minimum spread required to achieve sub-metre depth accuracy.
4. **Clock synchronisation errors**: introduce a common clock offset $\delta t$ shared by all
   drones (hardware drift); augment the state vector to $(\mathbf{x}_t, \delta t)$ and re-derive
   the extended Jacobian; show the additional drone needed to resolve the extra unknown.
5. **Continuous optimisation of drone paths**: formulate D-optimal repositioning as a
   continuous trajectory-planning problem with velocity and turning-rate constraints; solve
   with a model-predictive controller (MPC) and compare arrival time vs GDOP improvement
   against the instantaneous-reposition heuristic used here.

---

## Related Scenarios

- Prerequisites: [S046 Multi-Drone 3D Trilateration](S046_trilateration.md), [S047 Base Station Signal Relay](S047_signal_relay.md)
- Follow-ups: [S060 Seismic Event Mapping](S060_seismic_mapping.md)
- Algorithmic cross-reference: [S008 Stochastic Pursuit](../01_pursuit_evasion/S008_stochastic_pursuit.md) (Kalman filtering under noisy observations), [S046 3D Trilateration](S046_trilateration.md) (range-based WLS; GDOP formulation)
