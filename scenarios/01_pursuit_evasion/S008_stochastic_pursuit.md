# S008 Stochastic Pursuit — Kalman Filter Tracking

**Domain**: Pursuit & Evasion | **Difficulty**: ⭐⭐⭐ | **Status**: `[x]` Completed

---

## Problem Definition

**Setup**: The evader moves with stochastic acceleration (constant-velocity model + Gaussian noise). The pursuer receives noisy position measurements ($\sigma = 0.3$ m) and uses a Kalman Filter to estimate the evader's state (position + velocity). The filtered estimate drives the pursuit law.

**Comparison**:
- **Kalman filter**: smooth position + velocity estimate; leads the evader
- **Naive tracking**: aim at raw noisy measurement directly
- **Oracle**: perfect state knowledge (performance upper bound)

---

## Mathematical Model

### Evader Process Model

$$\mathbf{x}_E(k+1) = \mathbf{F}\,\mathbf{x}_E(k) + \mathbf{w}_k, \quad \mathbf{w}_k \sim \mathcal{N}(\mathbf{0}, \mathbf{Q})$$

State $\mathbf{x}_E = [\mathbf{p}_E;\,\mathbf{v}_E] \in \mathbb{R}^6$. Constant-velocity transition:

$$\mathbf{F} = \begin{bmatrix}\mathbf{I} & \Delta t\,\mathbf{I} \\ \mathbf{0} & \mathbf{I}\end{bmatrix}, \quad \mathbf{Q} = q\begin{bmatrix}\tfrac{\Delta t^3}{3}\mathbf{I} & \tfrac{\Delta t^2}{2}\mathbf{I} \\ \tfrac{\Delta t^2}{2}\mathbf{I} & \Delta t\,\mathbf{I}\end{bmatrix}$$

### Measurement Model

$$\mathbf{z}_k = \mathbf{H}\,\mathbf{x}_E(k) + \mathbf{n}_k, \quad \mathbf{n}_k \sim \mathcal{N}(\mathbf{0}, \sigma^2\mathbf{I})$$

where $\mathbf{H} = [\mathbf{I}\;\mathbf{0}]$ (position-only).

### Kalman Recursion

**Predict:** $\hat{\mathbf{x}}^- = \mathbf{F}\hat{\mathbf{x}}$, $\mathbf{P}^- = \mathbf{F}\mathbf{P}\mathbf{F}^T + \mathbf{Q}$

**Update:** $\mathbf{K} = \mathbf{P}^-\mathbf{H}^T(\mathbf{H}\mathbf{P}^-\mathbf{H}^T + \mathbf{R})^{-1}$, $\hat{\mathbf{x}} = \hat{\mathbf{x}}^- + \mathbf{K}(\mathbf{z}_k - \mathbf{H}\hat{\mathbf{x}}^-)$

---

## Implementation

```python
class KalmanFilter3D:
    def __init__(self, dt, q, sigma_meas):
        I3 = np.eye(3)
        self.F = np.block([[I3, dt*I3],[np.zeros((3,3)), I3]])
        self.H = np.hstack([I3, np.zeros((3,3))])
        self.Q = q * np.block([[dt**3/3*I3, dt**2/2*I3],
                                [dt**2/2*I3, dt*I3]])
        self.R = sigma_meas**2 * I3
        self.x = np.zeros(6)
        self.P = np.eye(6)

    def step(self, z):
        # Predict
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        # Update
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x += K @ (z - self.H @ self.x)
        self.P  = (np.eye(6) - K @ self.H) @ self.P
        return self.x[:3]   # estimated position
```

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Process noise q | 0.1 m²/s³ |
| Measurement noise sigma | 0.3 m |
| Evader acceleration std | 0.5 m/s² |
| Pursuer speed | 5 m/s |
| Evader mean speed | 3 m/s |
| Initial distance | 8 m |
| Control frequency | 48 Hz |

---

## Expected Output

- 3D trajectories: true evader, Kalman estimate, raw noisy measurement, pursuer
- Position estimation error vs time: Kalman vs naive
- Capture time comparison: Kalman, naive, oracle
- Covariance ellipse evolution (XY projection, sampled every 0.5 s)

---

## Extensions

1. Extended Kalman Filter (EKF) for nonlinear evader dynamics
2. Particle filter for non-Gaussian or multi-modal uncertainty
3. IMM filter: multiple models handle evaders that switch strategies mid-flight

---

## Related Scenarios

- Prerequisites: [S001](S001_basic_intercept.md), [S002](S002_evasive_maneuver.md)
- Next: [S009](S009_differential_game.md), [S019](S019_dynamic_reassignment.md)

## References

- Welch, G. & Bishop, G. (2006). *An Introduction to the Kalman Filter*. UNC-Chapel Hill.
