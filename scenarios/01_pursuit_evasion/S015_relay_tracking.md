# S015 Relay Tracking

**Domain**: Pursuit & Evasion | **Difficulty**: ⭐⭐ | **Status**: `[x]` Complete

---

## Problem Definition

**Setup**: A target moves across a large arena (10×10 m) along a sinusoidal path. Three stationary sensor drones are placed at fixed positions, each covering an overlapping circular zone. As the target travels, tracking responsibility is handed off between sensor drones.

**Roles**:
- **Sensor drones** (3): stationary; each measures only bearing (angle) to the target — no range information
- **Target**: moves at constant speed along a sinusoidal trajectory across the arena

**Objective**: demonstrate zone-based relay handoff and triangulation-based position estimation; measure how estimation error spikes at handoff moments.

---

## Mathematical Model

### Coverage Zones

Zone i is a circular region of radius R centered on sensor drone i:

$$\text{Zone}_i = \{ \mathbf{p} \in \mathbb{R}^2 \mid \|\mathbf{p} - \mathbf{p}_i\| \leq R_{zone} \}$$

### Bearing Measurement

Sensor drone i measures the bearing to the target with additive Gaussian noise:

$$\theta_i = \text{atan2}(y_T - y_i,\ x_T - x_i) + \epsilon_i, \quad \epsilon_i \sim \mathcal{N}(0,\ \sigma_\theta^2)$$

### Two-Drone Triangulation

Given bearings from drone 1 at position (x1, y1) and drone 2 at position (x2, y2), form two lines and solve for their intersection:

$$y - y_1 = \tan(\theta_1)(x - x_1)$$

$$y - y_2 = \tan(\theta_2)(x - x_2)$$

Solving the 2×2 linear system gives the estimated target position.

### Handoff Condition

A distance-based SNR proxy is defined for drone i:

$$\text{SNR}_i = 1 - \frac{\|\mathbf{p}_T - \mathbf{p}_i\|}{R_{zone}}$$

Handoff from zone i to zone j is triggered when SNR_i drops below the threshold and SNR_j rises above it:

$$\text{SNR}_i < \tau_{handoff} \quad \text{AND} \quad \text{SNR}_j > \tau_{handoff}$$

### Estimation Error

$$e(t) = \|\hat{\mathbf{p}}_T(t) - \mathbf{p}_T(t)\|$$

---

## Implementation

```python
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

# --- Parameters ---
SENSOR_POSITIONS = np.array([[-4, 0, 3], [0, 4, 3], [4, 0, 3]], dtype=float)
R_ZONE = 6.0          # m, coverage radius
SIGMA_THETA = 0.05    # rad, bearing noise (~3 deg)
TARGET_SPEED = 2.0    # m/s
HANDOFF_THRESH = 0.5  # SNR threshold
DT = 0.05             # s
T_MAX = 20.0          # s

def target_path(t):
    """Sinusoidal trajectory across arena."""
    x = -5.0 + TARGET_SPEED * t
    y = 3.0 * np.sin(0.5 * x)
    return np.array([x, y])

def measure_bearing(sensor_pos, target_pos, sigma):
    """Noisy bearing measurement."""
    dx = target_pos[0] - sensor_pos[0]
    dy = target_pos[1] - sensor_pos[1]
    return np.arctan2(dy, dx) + np.random.normal(0, sigma)

def triangulate(p1, theta1, p2, theta2):
    """Solve for intersection of two bearing lines."""
    # tan(theta1)*(x - x1) - y = -y1  =>  [tan(t1), -1] * [x, y] = tan(t1)*x1 - y1
    A = np.array([[np.tan(theta1), -1],
                  [np.tan(theta2), -1]])
    b = np.array([np.tan(theta1) * p1[0] - p1[1],
                  np.tan(theta2) * p2[0] - p2[1]])
    if abs(np.linalg.det(A)) < 1e-6:
        return None
    return np.linalg.solve(A, b)

def snr(sensor_pos, target_pos, r_zone):
    dist = np.linalg.norm(target_pos - sensor_pos[:2])
    return max(0.0, 1.0 - dist / r_zone)

# --- Simulation loop ---
times, errors, active_counts, handoff_times = [], [], [], []
active_zone = 0

for step in range(int(T_MAX / DT)):
    t = step * DT
    p_T = target_path(t)

    snrs = [snr(SENSOR_POSITIONS[i], p_T, R_ZONE) for i in range(3)]
    active_count = sum(s > HANDOFF_THRESH for s in snrs)

    # Handoff: choose highest-SNR sensor as primary
    new_zone = int(np.argmax(snrs))
    if new_zone != active_zone:
        handoff_times.append(t)
        active_zone = new_zone

    # Triangulate using two active sensors with highest SNR
    ranked = np.argsort(snrs)[::-1]
    i, j = ranked[0], ranked[1]
    theta_i = measure_bearing(SENSOR_POSITIONS[i], p_T, SIGMA_THETA)
    theta_j = measure_bearing(SENSOR_POSITIONS[j], p_T, SIGMA_THETA)
    p_est = triangulate(SENSOR_POSITIONS[i, :2], theta_i,
                        SENSOR_POSITIONS[j, :2], theta_j)

    err = np.linalg.norm(p_est - p_T) if p_est is not None else np.nan
    times.append(t)
    errors.append(err)
    active_counts.append(active_count)
```

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Sensor drone positions | (-4, 0, 3), (0, 4, 3), (4, 0, 3) m |
| Zone coverage radius | 6 m |
| Bearing noise sigma | 0.05 rad (~3 deg) |
| Target speed | 2.0 m/s |
| Target trajectory | sinusoidal, x from -5 to +5 m |
| Handoff SNR threshold | 0.5 |
| Simulation timestep | 0.05 s |

---

## Expected Output

- **2D top-down view**: target path, sensor zone circles, estimated positions, handoff event markers
- **Estimation error vs time**: spikes visible at handoff transitions
- **Active sensor count vs time**: shows overlap periods where 2–3 sensors are simultaneously active

---

## Extensions

1. Add a mobile relay drone that repositions to optimize triangulation geometry (minimize DOP)
2. DOP (Dilution of Precision) analysis — visualize how sensor geometry affects estimation accuracy
3. 3D triangulation using both azimuth and elevation bearing measurements

---

## Related Scenarios

- Prerequisites: [S001](S001_basic_intercept.md), [S012](S012_relay_pursuit.md)
- Next: [S019](S019_dynamic_reassignment.md) (dynamic reassignment)
- See [domains/01_pursuit_evasion.md](../../domains/01_pursuit_evasion.md)
