# S014 3D Upgrade — Decoy & Lure

**Domain**: Pursuit & Evasion | **Difficulty**: ⭐⭐⭐⭐ | **Status**: `[ ]` Not started
**Based on**: [S014 original](../S014_decoy_lure.md)

---

## What Changes in 3D

The original places the pursuer, evader, and decoy all at $z = 2.0$ m fixed. The decoy's lure trajectory is a flat 2D approach toward the pursuer. In 3D the decoy is programmed to **lead the pursuer upward into a high-altitude lure zone** while the real evader escapes at low altitude. Two flanking drones (cooperating with the evader) approach the pursuer from above and below during the lure phase. The Bayesian classifier must now account for 3D jerk, including the vertical axis, and the pursuer's target identification becomes more difficult under the added altitude dimension.

---

## Problem Definition

**Setup**: The evader deploys a decoy that first flies horizontally toward the pursuer (classical lure) then climbs steeply — drawing the pursuer into a high-altitude position. While the pursuer is elevated and decelerating from the climb, the real evader escapes at low altitude $z = 0.8$ m. Two flanking drones (allied with the evader) use the pursuer's elevated position to approach from the sides.

**Comparison**:
- **Correct 3D ID**: pursuer resists the altitude trap, identifies real evader by low-z signature
- **Misidentification**: pursuer climbs to chase decoy; loses altitude advantage and evader escapes

---

## Mathematical Model

### 3D Decoy Motion

During lure phase ($t \in [0, T_{lure}]$): decoy flies toward pursuer in XY while holding same altitude:

$$\mathbf{p}_{decoy}(t) = \mathbf{p}_E(t) + \boldsymbol{\varepsilon}_{xy}(t)$$

$$\boldsymbol{\varepsilon}_{xy}(t) \sim \mathcal{N}(\mathbf{0}, \sigma_{decoy}^2 \mathbf{I}_{2 \times 2})$$

During altitude-trap phase ($t \in [T_{lure}, T_{trap}]$): decoy climbs to $z_{trap} = 5.0$ m at rate $\dot{z}_{decoy} = 1.5$ m/s while continuing toward pursuer:

$$\mathbf{v}_{decoy} = v_{decoy} \cdot \frac{\mathbf{p}_P - \mathbf{p}_{decoy}}{\|\mathbf{p}_P - \mathbf{p}_{decoy}\|} + [0, 0, \dot{z}_{decoy}]^T$$

(renormalised to $v_{decoy}$ after adding z-component)

### 3D Motion Smoothness Score

Per-track jerk using full 3D acceleration:

$$s_j^{3D}(k) = \frac{1}{T} \sum_{t} \frac{\|\mathbf{v}_j(t) - \mathbf{v}_j(t - \Delta t)\|}{\Delta t}$$

The z-component contributes substantially to the decoy's smoothness score because the climb is an unnatural high-acceleration maneuver:

$$s_j^z(k) = \frac{1}{T} \sum_t \frac{|v_{z,j}(t) - v_{z,j}(t-\Delta t)|}{\Delta t}$$

### Extended Bayesian Classifier with Altitude Prior

An altitude signature prior is added: the real evader is known to fly at lower altitude:

$$P(\text{real} \mid \text{track}_k) \propto P(\text{smooth} \mid \text{track}_k) \cdot P(z_k < z_{threshold})$$

where $P(z_k < z_{threshold}) = \sigma\!\left(-\lambda_z (z_k - z_{threshold})\right)$ with $z_{threshold} = 3.0$ m, $\lambda_z = 2.0$.

Full Bayesian log-ratio update:

$$\Lambda \mathrel{+}= \lambda_s \cdot (s_2^{3D} - s_1^{3D}) + \lambda_z \cdot (z_2 - z_1)$$

where $\lambda_s = 0.5$ (jerk weight) and $\lambda_z = 0.3$ (altitude-bias weight).

### Altitude Strategy

The real evader exploits the altitude contrast: it dives to $z_{escape} = 0.8$ m as soon as the decoy begins its climb. The altitude difference $\Delta z = z_{decoy} - z_{evader}$ provides a discriminating feature:

$$\Delta z(t) = z_{decoy}(t) - z_{evader}(t)$$

A large $\Delta z > 2$ m with the decoy high and evader low is a strong indicator for classification, but the pursuer must observe this and update its classifier before committing to a climb.

### Flanking Drone Geometry

Two flanking drones (F1 above, F2 at same altitude) approach the pursuer when it climbs above $z_{flank} = 3.5$ m:

$$\mathbf{p}_{F1}^{target} = \mathbf{p}_P + [0, 0, 1.5]^T$$
$$\mathbf{p}_{F2}^{target} = \mathbf{p}_P + [0, 2.0, 0]^T$$

The flankers do not need to capture — they simply force the pursuer to choose between tracking the decoy and avoiding the flankers.

---

## Key 3D Additions

- **Altitude strategy**: decoy climbs to z = 5.0 m (altitude trap); real evader dives to z = 0.8 m (altitude contrast)
- **3D guidance law**: Bayesian classifier extended with z-component jerk and altitude prior; full 3D trajectory smoothness
- **Vertical evasion / geometry**: altitude contrast $\Delta z$ as discriminating feature; flanking drones exploit pursuer's elevated position

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Decoy noise sigma xy | 0.4 m |
| Decoy climb rate | 1.5 m/s |
| Altitude trap height | 5.0 m |
| Real evader escape altitude | 0.8 m |
| Lure phase duration T_lure | 3.0 s |
| Altitude trap phase T_trap | 6.0 s |
| Bayesian jerk weight λ_s | 0.5 |
| Bayesian altitude weight λ_z | 0.3 |
| Classification update period | 0.5 s |
| Altitude threshold for classifier | 3.0 m |
| Pursuer speed | 5 m/s |
| Evader speed | 3 m/s |
| Decoy speed | 2 m/s |
| Flanker activation altitude | 3.5 m |
| z range | 0.5 – 6.0 m |

---

## Implementation

```python
import numpy as np
from numpy.linalg import norm

SIGMA_DECOY  = 0.4
T_LURE       = 3.0
T_TRAP       = 6.0
Z_TRAP       = 5.0
Z_ESCAPE     = 0.8
Z_FLANK_ACT  = 3.5
Z_THRESH_CLS = 3.0
LAMBDA_S     = 0.5
LAMBDA_Z     = 0.3
V_PURSUER    = 5.0
V_EVADER     = 3.0
V_DECOY      = 2.0
DECOY_CLIMB  = 1.5
DT = 0.05
Z_MIN, Z_MAX = 0.5, 6.0

def decoy_velocity_3d(t, pos_decoy, pos_pursuer, v_max):
    toward_pursuer = pos_pursuer[:2] - pos_decoy[:2]
    xy_norm = norm(toward_pursuer) + 1e-8
    if t < T_LURE:
        # Horizontal lure
        vel = np.array([toward_pursuer[0]/xy_norm, toward_pursuer[1]/xy_norm, 0.0])
    elif t < T_TRAP:
        # Altitude trap: fly toward pursuer AND climb
        vel_xy = np.array([toward_pursuer[0]/xy_norm, toward_pursuer[1]/xy_norm, 0.0])
        vel_z  = np.array([0., 0., DECOY_CLIMB])
        vel    = vel_xy + vel_z
    else:
        # Decoy stops after trap phase
        return np.zeros(3)
    return v_max * vel / (norm(vel) + 1e-8)

def evader_escape_3d(pos_e, pos_pursuer, t, v_max):
    """Evader flees horizontally and dives to low altitude."""
    away_xy = pos_e[:2] - pos_pursuer[:2]
    if norm(away_xy) < 1e-8:
        away_xy = np.array([1.0, 0.0])
    away_xy /= norm(away_xy)
    # Dive component: go toward Z_ESCAPE
    dz = Z_ESCAPE - pos_e[2]
    vel = np.array([away_xy[0], away_xy[1], dz])
    return v_max * vel / (norm(vel) + 1e-8)

def jerk_3d(vel_history, dt):
    if len(vel_history) < 2:
        return 0.0
    vels = np.array(vel_history[-10:])  # last 10 steps
    jerks = norm(np.diff(vels, axis=0), axis=1) / dt
    return float(jerks.mean())

def bayesian_update(log_ratio, jerk_track1, jerk_track2, z1, z2,
                    lambda_s=LAMBDA_S, lambda_z=LAMBDA_Z):
    log_ratio += lambda_s * (jerk_track2 - jerk_track1)
    log_ratio += lambda_z * (z2 - z1)  # high z → more likely decoy
    return log_ratio

def simulate_decoy_3d():
    pos_p = np.array([0.0, -8.0, 2.0])
    pos_e = np.array([0.0,  0.0, 2.0])
    pos_d = pos_e.copy() + np.array([0.5, 0.3, 0.0])  # decoy near evader

    vel_p = np.zeros(3)
    vel_e_hist, vel_d_hist = [], []
    log_ratio = 0.0  # log P(real|track_evader) / P(real|track_decoy)
    T_UPDATE = 0.5
    last_update = 0.0

    traj_p, traj_e, traj_d = [pos_p.copy()], [pos_e.copy()], [pos_d.copy()]
    log_ratios = [0.0]

    for step in range(int(20 / DT)):
        t = step * DT

        # Move decoy
        v_d = decoy_velocity_3d(t, pos_d, pos_p, V_DECOY)
        pos_d += v_d * DT + np.append(np.random.randn(2)*SIGMA_DECOY*DT, 0)
        pos_d[2] = np.clip(pos_d[2], Z_MIN, Z_MAX)
        vel_d_hist.append(v_d.copy())

        # Move evader
        v_e = evader_escape_3d(pos_e, pos_p, t, V_EVADER)
        pos_e += v_e * DT
        pos_e[2] = np.clip(pos_e[2], Z_MIN, Z_MAX)
        vel_e_hist.append(v_e.copy())

        # Bayesian classification update
        if t - last_update >= T_UPDATE:
            j_e = jerk_3d(vel_e_hist, DT)
            j_d = jerk_3d(vel_d_hist, DT)
            log_ratio = bayesian_update(log_ratio, j_e, j_d, pos_e[2], pos_d[2])
            last_update = t

        # Pursuer targets track with higher P(real)
        # log_ratio > 0 => track_evader is real
        target = pos_e if log_ratio > 0 else pos_d
        d_to_target = target - pos_p
        vel_p = V_PURSUER * d_to_target / (norm(d_to_target) + 1e-8)
        pos_p += vel_p * DT
        pos_p[2] = np.clip(pos_p[2], Z_MIN, Z_MAX)

        traj_p.append(pos_p.copy()); traj_e.append(pos_e.copy())
        traj_d.append(pos_d.copy()); log_ratios.append(log_ratio)

        if norm(pos_p - pos_e) < 0.15:
            print(f"Captured real evader at t={t:.2f}s")
            break
        if norm(pos_p - pos_d) < 0.15:
            print(f"Captured DECOY at t={t:.2f}s — evader ESCAPED")
            break

    return (np.array(traj_p), np.array(traj_e),
            np.array(traj_d), np.array(log_ratios))
```

---

## Expected Output

- 3D trajectories: pursuer (red), real evader (blue, low altitude), decoy (dashed green, climbing)
- Altitude time series: clear V-shape — decoy climbs while evader dives
- Classification log-ratio $\Lambda(t)$: should show altitude-bias kicking in to correct misclassification
- Comparison: 2D classifier (jerk only) vs 3D classifier (jerk + altitude prior) — misidentification rate over 30 random seeds
- Altitude contrast $\Delta z(t)$ used as discriminating feature

---

## Extensions

1. Adaptive decoy: decoy mirrors evader altitude to defeat altitude-based classifier
2. Three decoys at different altitudes — pursuer must solve 3D multi-track data association
3. Active sensing: pursuer commands altitude change to improve track discrimination via parallax

---

## Related Scenarios

- Original 2D version: [S014](../S014_decoy_lure.md)
- Truly 3D references: [S001](../S001_basic_intercept.md), [S003](../S003_low_altitude_tracking.md)
