# S015 3D Upgrade — Relay Tracking

**Domain**: Pursuit & Evasion | **Difficulty**: ⭐⭐⭐⭐ | **Status**: `[ ]` Not started
**Based on**: [S015 original](../S015_relay_tracking.md)

---

## What Changes in 3D

The original is **purely 2D**: sensor drones report bearing angles only in the XY plane, `np.array([x, y])` positions throughout, and 2D triangulation. In 3D, the target moves along a helical or sinusoidal trajectory with altitude variation. Each sensor drone reports **both azimuth and elevation bearing** angles. Triangulation becomes a full 3D bearing-intersection problem. Sensor drones are positioned at **altitude tiers** (z = 1.5, 3.5, 5.0 m) rather than all at the same height, which improves 3D Dilution of Precision (DOP) for targets at varying altitudes.

---

## Problem Definition

**Setup**: A target follows a helical trajectory (circular orbit + z oscillation). Three sensor drones at different altitudes each measure azimuth and elevation bearing to the target with Gaussian noise. A relay handoff is triggered by 3D SNR (inverse distance within a spherical zone). Two-drone 3D bearing triangulation estimates the target's full $(x, y, z)$ position.

**Objective**: demonstrate that altitude-stratified sensors reduce estimation error compared to all-same-height deployment; show how estimation error spikes at handoff are worse in 3D due to the added z uncertainty.

---

## Mathematical Model

### 3D Coverage Zones

Zone $i$ is now a sphere of radius $R_{zone}$:

$$\text{Zone}_i = \{ \mathbf{p} \in \mathbb{R}^3 \mid \|\mathbf{p} - \mathbf{p}_i\| \leq R_{zone} \}$$

### 3D Bearing Measurements

Each sensor drone $i$ at position $\mathbf{p}_i = (x_i, y_i, z_i)$ measures azimuth $\alpha_i$ and elevation $\beta_i$ to the target at $\mathbf{p}_T = (x_T, y_T, z_T)$:

$$\alpha_i = \text{atan2}(y_T - y_i,\; x_T - x_i) + \epsilon_\alpha, \quad \epsilon_\alpha \sim \mathcal{N}(0, \sigma_\alpha^2)$$

$$\beta_i = \arctan\!\left(\frac{z_T - z_i}{\sqrt{(x_T-x_i)^2 + (y_T-y_i)^2}}\right) + \epsilon_\beta, \quad \epsilon_\beta \sim \mathcal{N}(0, \sigma_\beta^2)$$

### 3D Bearing Ray

The bearing pair $(\alpha_i, \beta_i)$ defines a unit direction vector:

$$\hat{\mathbf{d}}_i = \begin{bmatrix}
\cos\beta_i \cos\alpha_i \\
\cos\beta_i \sin\alpha_i \\
\sin\beta_i
\end{bmatrix}$$

The bearing ray from sensor $i$ is:

$$\mathbf{r}_i(s) = \mathbf{p}_i + s \cdot \hat{\mathbf{d}}_i, \quad s \geq 0$$

### 3D Two-Bearing Triangulation

Given two bearing rays $(\mathbf{p}_1, \hat{\mathbf{d}}_1)$ and $(\mathbf{p}_2, \hat{\mathbf{d}}_2)$, the closest-approach midpoint estimates the target position. The system:

$$\min_{s_1, s_2} \|\mathbf{r}_1(s_1) - \mathbf{r}_2(s_2)\|^2$$

has closed-form solution. Define $\mathbf{w} = \mathbf{p}_1 - \mathbf{p}_2$ and:

$$a = \hat{\mathbf{d}}_1 \cdot \hat{\mathbf{d}}_1 = 1, \quad b = \hat{\mathbf{d}}_1 \cdot \hat{\mathbf{d}}_2, \quad c = \hat{\mathbf{d}}_2 \cdot \hat{\mathbf{d}}_2 = 1$$
$$d = \hat{\mathbf{d}}_1 \cdot \mathbf{w}, \quad e = \hat{\mathbf{d}}_2 \cdot \mathbf{w}$$
$$\text{denom} = ac - b^2 = 1 - b^2$$

$$s_1^* = \frac{be - cd}{\text{denom}}, \quad s_2^* = \frac{ae - bd}{\text{denom}}$$

Estimated position (midpoint of closest approach):

$$\hat{\mathbf{p}}_T = \frac{1}{2}\!\left[(\mathbf{p}_1 + s_1^* \hat{\mathbf{d}}_1) + (\mathbf{p}_2 + s_2^* \hat{\mathbf{d}}_2)\right]$$

### 3D SNR and Handoff

Spherical SNR proxy:

$$\text{SNR}_i = \max\!\left(0,\; 1 - \frac{\|\mathbf{p}_T - \mathbf{p}_i\|}{R_{zone}}\right)$$

### Altitude Strategy

Sensor drones at three altitude tiers $z \in \{1.5, 3.5, 5.0\}$ m. For a target at altitude $z_T$, the sensor at the tier closest to $z_T$ has the smallest elevation angle uncertainty (better $\beta$ estimation). The active sensor SNR is weighted by the elevation angle proximity:

$$\text{SNR}_i^{3D} = \text{SNR}_i \cdot \exp\!\left(-\frac{(z_T - z_i)^2}{2 \sigma_z^2}\right)$$

where $\sigma_z = 2.0$ m is the altitude-weighting bandwidth.

### 3D DOP (Dilution of Precision)

The geometric DOP for 3D bearing triangulation with $K$ sensors is:

$$\mathbf{H} = \begin{bmatrix}
\hat{\mathbf{d}}_1^T \\
\hat{\mathbf{d}}_2^T \\
\vdots \\
\hat{\mathbf{d}}_K^T
\end{bmatrix}, \quad
\text{DOP}_{3D} = \sqrt{\text{tr}[(\mathbf{H}^T \mathbf{H})^{-1}]}$$

A planar sensor arrangement gives poor z-DOP; altitude-stratified sensors improve the $(\mathbf{H}^T \mathbf{H})^{-1}$ conditioning by providing non-coplanar bearing vectors.

---

## Key 3D Additions

- **Altitude strategy**: sensors at z = 1.5, 3.5, 5.0 m provide non-coplanar bearing vectors; altitude-weighted SNR selects the best sensor tier for target at each altitude
- **3D guidance law**: full azimuth-elevation bearing pair per sensor; 3D closest-approach triangulation replaces 2D line intersection
- **Vertical evasion / geometry**: target follows helical orbit with ±1.5 m altitude variation; z-component of estimation error measured separately; 3D DOP analysis

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Sensor drone positions | (0, 4, 1.5), (−3.46, −2, 3.5), (3.46, −2, 5.0) m |
| Zone coverage radius | 8 m (spherical) |
| Azimuth noise sigma_α | 0.05 rad |
| Elevation noise sigma_β | 0.07 rad |
| Target speed | 2.0 m/s |
| Target trajectory | helical: r=3 m orbit + z = 2 + 1.5 sin(ωt) |
| Handoff SNR threshold | 0.4 |
| Altitude-weighting bandwidth σ_z | 2.0 m |
| Simulation timestep | 0.05 s |
| z range | 0.5 – 6.5 m |

---

## Implementation

```python
import numpy as np
from numpy.linalg import norm, det

SENSOR_POSITIONS = np.array([
    [ 0.0,  4.0, 1.5],
    [-3.46, -2.0, 3.5],
    [ 3.46, -2.0, 5.0],
], dtype=float)
R_ZONE       = 8.0
SIGMA_ALPHA  = 0.05
SIGMA_BETA   = 0.07
TARGET_SPEED = 2.0
HANDOFF_THRESH = 0.4
SIGMA_Z_ALT  = 2.0
DT = 0.05
T_MAX = 25.0

def target_path_3d(t):
    r, omega = 3.0, TARGET_SPEED / 3.0
    return np.array([
        r * np.cos(omega * t),
        r * np.sin(omega * t),
        2.0 + 1.5 * np.sin(0.4 * t)
    ])

def measure_bearing_3d(sensor_pos, target_pos, sigma_a, sigma_b):
    dx = target_pos[0] - sensor_pos[0]
    dy = target_pos[1] - sensor_pos[1]
    dz = target_pos[2] - sensor_pos[2]
    r_xy = np.sqrt(dx**2 + dy**2)
    alpha = np.arctan2(dy, dx) + np.random.normal(0, sigma_a)
    beta  = np.arctan2(dz, r_xy + 1e-8) + np.random.normal(0, sigma_b)
    return alpha, beta

def bearing_to_direction(alpha, beta):
    return np.array([
        np.cos(beta) * np.cos(alpha),
        np.cos(beta) * np.sin(alpha),
        np.sin(beta)
    ])

def triangulate_3d(p1, d1, p2, d2):
    """Closest approach midpoint between two rays."""
    w = p1 - p2
    b = np.dot(d1, d2)
    denom = 1.0 - b**2
    if abs(denom) < 1e-8:
        return None  # parallel rays
    d_val = np.dot(d1, w)
    e_val = np.dot(d2, w)
    s1 = (b * e_val - d_val) / denom
    s2 = (e_val - b * d_val) / denom
    closest1 = p1 + s1 * d1
    closest2 = p2 + s2 * d2
    return 0.5 * (closest1 + closest2)

def snr_3d(sensor_pos, target_pos, r_zone, z_t, sigma_z=SIGMA_Z_ALT):
    dist = norm(target_pos - sensor_pos)
    base_snr = max(0.0, 1.0 - dist / r_zone)
    alt_weight = np.exp(-0.5 * ((z_t - sensor_pos[2]) / sigma_z)**2)
    return base_snr * alt_weight

def compute_dop_3d(sensor_positions, target_pos):
    """Compute 3D DOP from sensor geometry."""
    dirs = []
    for sp in sensor_positions:
        dv = target_pos - sp
        n = norm(dv)
        if n > 1e-8:
            dirs.append(dv / n)
    if len(dirs) < 3:
        return np.inf
    H = np.array(dirs)   # (K, 3)
    HtH = H.T @ H
    try:
        return float(np.sqrt(np.trace(np.linalg.inv(HtH))))
    except np.linalg.LinAlgError:
        return np.inf

# Simulation loop
times, errors_xyz, errors_z, active_sensor, handoff_times = [], [], [], [], []
active_zone = 0

for step in range(int(T_MAX / DT)):
    t = step * DT
    p_T = target_path_3d(t)
    z_T = p_T[2]

    snrs = [snr_3d(SENSOR_POSITIONS[i], p_T, R_ZONE, z_T) for i in range(3)]
    new_zone = int(np.argmax(snrs))
    if new_zone != active_zone:
        handoff_times.append(t)
        active_zone = new_zone

    ranked = np.argsort(snrs)[::-1]
    i, j = ranked[0], ranked[1]

    a_i, b_i = measure_bearing_3d(SENSOR_POSITIONS[i], p_T, SIGMA_ALPHA, SIGMA_BETA)
    a_j, b_j = measure_bearing_3d(SENSOR_POSITIONS[j], p_T, SIGMA_ALPHA, SIGMA_BETA)
    d_i = bearing_to_direction(a_i, b_i)
    d_j = bearing_to_direction(a_j, b_j)

    p_est = triangulate_3d(SENSOR_POSITIONS[i], d_i, SENSOR_POSITIONS[j], d_j)
    if p_est is not None:
        err_xyz = float(norm(p_est - p_T))
        err_z   = float(abs(p_est[2] - p_T[2]))
    else:
        err_xyz = err_z = np.nan

    times.append(t)
    errors_xyz.append(err_xyz)
    errors_z.append(err_z)
    active_sensor.append(active_zone)
```

---

## Expected Output

- 3D trajectory visualisation: helical target path, sensor zone spheres at three altitude tiers
- Full 3D estimation error $\|\hat{\mathbf{p}}_T - \mathbf{p}_T\|$ vs time with handoff event markers
- Separate z-estimation error $|\hat{z}_T - z_T|$ vs time — shows altitude-stratified sensors improve z accuracy
- DOP time series: 3D DOP vs time (lower when sensor geometry is non-coplanar)
- Comparison: all sensors at z = 3 m (planar) vs altitude-stratified — mean z-estimation error

---

## Extensions

1. Mobile relay drone that repositions to minimise 3D DOP in real time
2. Four sensors: optimise altitude tier values to minimise worst-case 3D DOP over entire target trajectory
3. Add range-rate (Doppler) measurement alongside bearing to create full 3D tracking fusion

---

## Related Scenarios

- Original 2D version: [S015](../S015_relay_tracking.md)
- Truly 3D references: [S001](../S001_basic_intercept.md), [S003](../S003_low_altitude_tracking.md)
