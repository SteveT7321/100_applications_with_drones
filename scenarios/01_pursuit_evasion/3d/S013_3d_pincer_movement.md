# S013 3D Upgrade — Pincer Movement

**Domain**: Pursuit & Evasion | **Difficulty**: ⭐⭐⭐⭐ | **Status**: `[x]` Completed
**Based on**: [S013 original](../S013_pincer_movement.md)

---

## What Changes in 3D

The original places two pursuers in a flat 2D plane at $z = 2.0$ m fixed, forming a horizontal pincer with the evader's escape cone lying entirely in the XY plane. In 3D the pincer becomes a **trident**: three drones approach from above, side-left, and below (offset by 120° in a tilted plane), creating a 3D blocking geometry. The evader's escape cone is now a **solid angle** in $\mathbb{R}^3$, and analysis of cone closure replaces the 2D angle measure.

---

## Problem Definition

**Setup**: Three pursuers approach the evader from three distinct directions in 3D — top-right, lateral, and bottom-left — approximating 120° angular separation on a great circle of the encirclement sphere. The evader uses an optimal 3D escape strategy that maximises the solid-angle escape cone. The three pursuers coordinate by maintaining their angular offsets while shrinking the offset radius.

**Objective**: measure how the 3D pincer solid-angle closing rate affects capture time; compare 3D trident vs 2D two-pursuer pincer.

---

## Mathematical Model

### 3D Pincer Target Positions

Each pursuer $i \in \{0, 1, 2\}$ targets an offset position on a sphere of radius $R(t)$:

$$\mathbf{p}_{P_i}^{target} = \mathbf{p}_E + R(t) \cdot \hat{\mathbf{d}}_i$$

The three approach directions are chosen as vertices of an equilateral triangle inscribed in a great circle tilted 45° from horizontal:

$$\hat{\mathbf{d}}_i = \mathbf{R}_{tilt} \cdot \begin{bmatrix}\cos(2\pi i / 3) \\ \sin(2\pi i / 3) \\ 0\end{bmatrix}, \quad i = 0, 1, 2$$

where $\mathbf{R}_{tilt}$ is a rotation by $\psi_{tilt} = 45°$ about the x-axis:

$$\mathbf{R}_{tilt} = \begin{bmatrix}1 & 0 & 0 \\ 0 & \cos\psi_{tilt} & -\sin\psi_{tilt} \\ 0 & \sin\psi_{tilt} & \cos\psi_{tilt}\end{bmatrix}$$

This places one drone above-right, one drone lateral-left, and one drone below-right.

### Shrinking Offset Radius

$$R(t) = \max\!\left(R_0 - v_{shrink} \cdot t,\; R_{min}\right)$$

### 3D Pincer Solid Angle

The solid angle $\Omega$ of the evader's escape cone is the area on the unit sphere not blocked by the three pursuers. It is approximated as:

$$\Omega_{blocked} \approx \sum_{i=1}^{3} \frac{\pi r_{cap}^2}{R(t)^2}$$

where $r_{cap}$ is the pursuer capture radius. The unblocked solid angle is:

$$\Omega_{escape} = 4\pi - \Omega_{blocked}$$

Capture becomes geometrically certain when $\Omega_{escape} \to 0$, i.e., when $R(t) \to r_{cap}\sqrt{3\pi}$.

### 3D Blocking Plane

The three pursuer positions define a blocking plane:

$$\hat{\mathbf{n}}_{block} = \frac{(\mathbf{p}_{P_1} - \mathbf{p}_{P_0}) \times (\mathbf{p}_{P_2} - \mathbf{p}_{P_0})}{\|(\mathbf{p}_{P_1} - \mathbf{p}_{P_0}) \times (\mathbf{p}_{P_2} - \mathbf{p}_{P_0})\|}$$

The signed distance of the evader from this plane:

$$d_{plane} = (\mathbf{p}_E - \mathbf{p}_{P_0}) \cdot \hat{\mathbf{n}}_{block}$$

Negative $d_{plane}$ means the evader is on the "closed" side of the pincer.

### Altitude Strategy

The tilt angle $\psi_{tilt}$ is dynamically updated based on the evader's vertical position relative to the pursuer centroid:

$$\psi_{tilt}^{cmd} = \arctan2\!\left(z_E - \bar{z}_P,\; \sqrt{(x_E - \bar{x}_P)^2 + (y_E - \bar{y}_P)^2}\right)$$

$$\dot{\psi}_{tilt} = k_{tilt}\!\left(\psi_{tilt}^{cmd} - \psi_{tilt}\right), \quad k_{tilt} = 0.5 \text{ rad/s}$$

This causes the triangular formation to **tilt toward the evader's altitude** — if the evader climbs, the top drone rises and the formation tilts upward to maintain blocking coverage.

### Evader Escape Direction

The evader maximises the angular distance to the nearest pursuer:

$$\mathbf{v}_E = v_E \cdot \arg\max_{\hat{\mathbf{u}} \in S^2} \min_{i} \arccos\!\left(\hat{\mathbf{u}} \cdot \frac{\mathbf{p}_E - \mathbf{p}_{P_i}}{\|\mathbf{p}_E - \mathbf{p}_{P_i}\|}\right)$$

---

## Key 3D Additions

- **Altitude strategy**: dynamic tilt angle $\psi_{tilt}$ tracks evader altitude; formation rotates to maintain 3D coverage
- **3D guidance law**: three approach directions on tilted equilateral triangle inscribed on encirclement sphere
- **Vertical evasion / geometry**: 3D blocking plane normal and solid-angle escape cone replace 2D pincer angle

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Pursuer 1 start | (−4, −2, 4) m (above-right approach) |
| Pursuer 2 start | (−4, 3, 2) m (lateral approach) |
| Pursuer 3 start | (−4, −2, 0.5) m (below approach) |
| Evader start | (2, 0, 2) m |
| Pursuer speed (each) | 5 m/s |
| Evader speed | 3.5 m/s |
| Initial offset radius R₀ | 3.0 m |
| Shrink rate | 0.3 m/s |
| Formation tilt angle ψ₀ | 45° |
| Tilt rate gain | 0.5 rad/s |
| z range | 0.5 – 6.0 m |
| Capture radius | 0.15 m |

---

## Implementation

```python
import numpy as np
from numpy.linalg import norm

N_PINCER = 3
R0       = 3.0
R_MIN    = 0.3
V_SHRINK = 0.3
V_PURSUER = 5.0
V_EVADER  = 3.5
PSI_INIT  = np.radians(45)
K_TILT    = 0.5
Z_MIN, Z_MAX = 0.5, 6.0
DT = 0.05

def rotation_x(psi):
    c, s = np.cos(psi), np.sin(psi)
    return np.array([[1,0,0],[0,c,-s],[0,s,c]])

def pincer_targets_3d(pos_e, R, psi_tilt):
    R_tilt = rotation_x(psi_tilt)
    targets = []
    for i in range(N_PINCER):
        angle = 2 * np.pi * i / N_PINCER
        d_local = np.array([np.cos(angle), np.sin(angle), 0.0])
        d_world = R_tilt @ d_local
        targets.append(pos_e + R * d_world)
    return np.array(targets)

def blocking_plane_normal(p0, p1, p2):
    v1 = p1 - p0
    v2 = p2 - p0
    n = np.cross(v1, v2)
    n_norm = norm(n)
    return n / n_norm if n_norm > 1e-8 else np.array([0., 0., 1.])

def evader_escape_3d(pos_e, pos_pursuers, v_max, K=200):
    dirs_to_p = pos_pursuers - pos_e[np.newaxis, :]
    norms_ = norm(dirs_to_p, axis=1, keepdims=True) + 1e-8
    dirs_to_p /= norms_
    theta = np.arccos(np.clip(1 - 2 * np.random.rand(K), -1, 1))
    phi   = 2 * np.pi * np.random.rand(K)
    cands = np.column_stack([
        np.sin(theta)*np.cos(phi),
        np.sin(theta)*np.sin(phi),
        np.cos(theta)])
    dots  = np.clip(cands @ dirs_to_p.T, -1, 1)
    gaps  = np.arccos(dots).min(axis=1)
    return v_max * cands[int(np.argmax(gaps))]

def simulate_pincer_3d():
    pos_p = np.array([
        [-4.0, -2.0, 4.0],
        [-4.0,  3.0, 2.0],
        [-4.0, -2.0, 0.8],
    ], dtype=float)
    pos_e = np.array([2.0, 0.0, 2.0])
    psi_tilt = PSI_INIT
    traj_e = [pos_e.copy()]
    traj_p = [pos_p.copy()]
    solid_angles = []

    for step in range(int(30 / DT)):
        t = step * DT
        R = max(R0 - V_SHRINK * t, R_MIN)

        # Update tilt toward evader's altitude
        centroid = pos_p.mean(axis=0)
        dz_e = pos_e[2] - centroid[2]
        dxy_e = norm(pos_e[:2] - centroid[:2])
        psi_cmd = np.arctan2(dz_e, dxy_e + 1e-8)
        psi_tilt += K_TILT * (psi_cmd - psi_tilt) * DT
        psi_tilt = np.clip(psi_tilt, -np.pi/3, np.pi/3)

        targets = pincer_targets_3d(pos_e, R, psi_tilt)
        targets[:, 2] = np.clip(targets[:, 2], Z_MIN, Z_MAX)

        for i in range(N_PINCER):
            d = targets[i] - pos_p[i]
            pos_p[i] += V_PURSUER * DT * d / (norm(d) + 1e-8)
            pos_p[i, 2] = np.clip(pos_p[i, 2], Z_MIN, Z_MAX)

        # Evader escape
        v_e = evader_escape_3d(pos_e, pos_p, V_EVADER)
        pos_e = pos_e + v_e * DT
        pos_e[2] = np.clip(pos_e[2], Z_MIN, Z_MAX)

        # Blocking plane metrics
        n_block = blocking_plane_normal(pos_p[0], pos_p[1], pos_p[2])
        d_plane = float(np.dot(pos_e - pos_p[0], n_block))
        omega_esc = max(4*np.pi - 3 * np.pi * (0.15/R)**2, 0)
        solid_angles.append(omega_esc)

        traj_e.append(pos_e.copy())
        traj_p.append(pos_p.copy())

        dists = norm(pos_p - pos_e[np.newaxis, :], axis=1)
        if dists.min() < 0.15:
            print(f"Captured at t={t:.2f}s, tilt={np.degrees(psi_tilt):.1f}°")
            break

    return np.array(traj_e), np.array(traj_p), np.array(solid_angles)
```

---

## Expected Output

- 3D trajectory showing trident formation approaching from different altitudes
- Altitude time series: all three pursuers tracking evader altitude via tilt adaptation
- Blocking plane normal vector vs time (should track toward evader)
- Solid-angle escape cone vs time (decreasing to near zero at capture)
- Comparison: 3D trident vs 2D two-pursuer pincer — capture time and success rate

---

## Extensions

1. Four-drone pincer with two drones in vertical pair: analyse tetrahedron vs flat-triangle blocking
2. Evader uses ballistic dive (sudden altitude drop) to escape the triangular plane
3. Dynamic reassignment of which pursuer is "top" vs "lateral" vs "bottom" based on current geometry

---

## Related Scenarios

- Original 2D version: [S013](../S013_pincer_movement.md)
- Truly 3D references: [S001](../S001_basic_intercept.md), [S003](../S003_low_altitude_tracking.md)
