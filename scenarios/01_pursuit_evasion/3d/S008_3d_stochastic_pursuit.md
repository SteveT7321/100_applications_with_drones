# S008 3D Upgrade — Stochastic Pursuit (Kalman Filter Tracking)

**Domain**: Pursuit & Evasion | **Difficulty**: ⭐⭐⭐⭐ | **Status**: `[x]` Complete
**Based on**: [S008 original](../S008_stochastic_pursuit.md)

---

## What Changes in 3D

The original S008 already defines a `KalmanFilter3D` class with a 6D state $[\mathbf{p};\mathbf{v}] \in \mathbb{R}^6$, but the simulation fixes both drones at z = 2 m — so the z-channel of the filter is never exercised. The stochastic acceleration noise and measurement noise are applied uniformly across all three axes in the math, but only the x-y components drive any interesting dynamics. In true 3D, the evader moves with altitude-varying manoeuvres, the Kalman covariance ellipsoid becomes a 3D ellipsoid (not a flat pancake), and the pursuer must lead the evader in 3D based on the filter's velocity estimate.

---

## Problem Definition

**Setup**: The evader moves with stochastic 3D acceleration (constant-velocity model + independent Gaussian noise on all three axes, including z). The pursuer receives noisy 3D position measurements ($\sigma = 0.3$ m isotropic) and uses a 6-state Kalman Filter to estimate the evader's full 3D state.

**3D-specific evader manoeuvres**:
- Constant-velocity in x-y with random z-hops (altitude uncertainty stress test)
- Helical trajectory (coupled $v_x, v_y, v_z$) — tests KF velocity tracking in all axes
- Random-walk in all three axes (original S008 model, extended to z)

**Comparison**:
- **Kalman filter (3D)**: estimates full 6D state including $v_z$
- **2D KF + altitude hold**: only filters x-y state, fixes z estimate at last known altitude
- **Naive tracking**: aim at raw noisy 3D measurement
- **Oracle**: perfect 3D state knowledge

---

## Mathematical Model

### 3D Evader Process Model

State $\mathbf{x}_E = [\mathbf{p}_E;\,\mathbf{v}_E] \in \mathbb{R}^6$. Constant-velocity model with 3D process noise:

$$\mathbf{x}_E(k+1) = \mathbf{F}\,\mathbf{x}_E(k) + \mathbf{w}_k, \quad \mathbf{w}_k \sim \mathcal{N}(\mathbf{0}, \mathbf{Q})$$

$$\mathbf{F} = \begin{bmatrix}\mathbf{I}_3 & \Delta t\,\mathbf{I}_3 \\ \mathbf{0}_3 & \mathbf{I}_3\end{bmatrix}, \quad \mathbf{Q} = \begin{bmatrix} q_{xy} \mathbf{D} & \tfrac{\Delta t}{2} q_{xy} \mathbf{D} \\ \tfrac{\Delta t}{2} q_{xy} \mathbf{D} & q_{xy} \mathbf{D} \end{bmatrix}$$

where $\mathbf{D} = \text{diag}(1, 1, q_z/q_{xy})$ allows anisotropic noise (evader manoeuvres more in z than x-y or vice versa).

### 3D Measurement Model

$$\mathbf{z}_k = \mathbf{H}\,\mathbf{x}_E(k) + \mathbf{n}_k, \quad \mathbf{n}_k \sim \mathcal{N}(\mathbf{0}, \sigma^2 \mathbf{I}_3)$$

$\mathbf{H} = [\mathbf{I}_3\;\mathbf{0}_3]$ (position-only observation, same as S008).

### 3D Kalman Recursion

**Predict:**

$$\hat{\mathbf{x}}^- = \mathbf{F}\hat{\mathbf{x}}, \quad \mathbf{P}^- = \mathbf{F}\mathbf{P}\mathbf{F}^T + \mathbf{Q}$$

**Update:**

$$\mathbf{K} = \mathbf{P}^-\mathbf{H}^T(\mathbf{H}\mathbf{P}^-\mathbf{H}^T + \mathbf{R})^{-1}$$
$$\hat{\mathbf{x}} = \hat{\mathbf{x}}^- + \mathbf{K}(\mathbf{z}_k - \mathbf{H}\hat{\mathbf{x}}^-)$$
$$\mathbf{P} = (\mathbf{I} - \mathbf{K}\mathbf{H})\mathbf{P}^-$$

### Predictive Pursuit with 3D KF

The pursuer uses the Kalman velocity estimate to lead the evader. Predicted evader position at intercept time $T_{int}$:

$$\hat{\mathbf{p}}_{E,pred} = \hat{\mathbf{p}}_E + \hat{\mathbf{v}}_E \cdot T_{int}$$

$$T_{int} = \frac{\|\hat{\mathbf{p}}_{E,pred} - \mathbf{p}_P\|}{v_P}$$

Solve iteratively (two steps converges). Pursuer commands velocity toward $\hat{\mathbf{p}}_{E,pred}$.

### 3D Covariance Ellipsoid Volume

$$V_{ellipsoid}(t) = \frac{4\pi}{3} \prod_{k=1}^{3} \sqrt{\lambda_k(\mathbf{P}_{pos})}$$

where $\lambda_k(\mathbf{P}_{pos})$ are the eigenvalues of the 3×3 position submatrix of $\mathbf{P}$.
Tracking quality degrades as $V_{ellipsoid}$ grows.

---

## Key 3D Additions

- Anisotropic 3D process noise $\mathbf{Q}$: z-manoeuvre noise tunable independently
- Full 3D Kalman velocity estimate used for predictive intercept in 3D
- 3D covariance ellipsoid volume as tracking quality metric
- 2D-KF fallback comparison: shows what happens when z-filter is missing
- Helical evader trajectory as stress test for z-axis filter convergence

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Process noise $q_{xy}$ | 0.1 m²/s³ |
| Process noise $q_z / q_{xy}$ ratio | 0.5 (evader less agile vertically) |
| Measurement noise $\sigma$ | 0.3 m (isotropic) |
| Evader acceleration std (per axis) | 0.5 m/s² |
| Pursuer speed | 5 m/s |
| Evader mean speed | 3 m/s |
| Initial pursuer position | (-4, 0, 2) m |
| Initial evader position | (4, 0, 3) m |
| Control frequency | 48 Hz |

---

## Expected Output

- **3D trajectory plot**: true evader path (helical), Kalman estimate, raw noisy measurements, pursuer
- **Per-axis estimation error vs time**: $e_x, e_y, e_z$ separately — shows z-axis filter quality
- **3D covariance ellipsoid volume vs time**: convergence rate of the filter
- **Capture time table**: 3D KF, 2D-KF+althold, naive, oracle

---

## Extensions

1. Extended Kalman Filter (EKF) for a coordinated-turn model: better tracks helical evader
2. Particle filter with 500 particles: handles multi-modal z uncertainty (e.g., evader could be at two candidate altitudes)
3. Sensor fusion: combine radar range-only measurements with bearing-only optical sensor — test whether 3D triangulation converges faster than single-sensor KF

---

## Related Scenarios

- Original: [S008 2D version](../S008_stochastic_pursuit.md)
- Truly 3D reference: [S001](../S001_basic_intercept.md), [S003](../S003_low_altitude_tracking.md)
