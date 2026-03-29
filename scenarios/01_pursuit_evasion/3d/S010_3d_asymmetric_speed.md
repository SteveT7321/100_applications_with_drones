# S010 3D Upgrade — Asymmetric Speed Bounded Arena Trapping

**Domain**: Pursuit & Evasion | **Difficulty**: ⭐⭐⭐⭐ | **Status**: `[x]` Complete
**Based on**: [S010 original](../S010_asymmetric_speed.md)

---

## What Changes in 3D

The original confines both agents to a flat 10×10 m square arena at z = 2.0 m fixed. The pursuer's strategies (direct pursuit, wall-herding, corner-targeting) exploit only 2D geometry. In 3D the arena becomes a bounded cubic volume, the Apollonius circle becomes an **Apollonius sphere**, and the pursuer gains a critical new tactic: **dive-and-recover** — trading altitude for a momentary speed boost through gravity-assisted acceleration. The evader simultaneously exploits vertical space to escape the horizontal wall-herding trap.

---

## Problem Definition

**Setup**: The evader ($v_E = 4.5$ m/s) is faster than the pursuer ($v_P = 3.0$ m/s, speed ratio $r = 0.67$). The arena is now a bounded cubic volume $[-5, 5]^3$ m. The pursuer tests four strategies:

1. **Direct 3D pursuit** — pure pursuit in full 3D
2. **Wall-and-ceiling herding** — blend toward nearest face of the cube (6 faces)
3. **Corner-targeting** — aim at the 3D corner closest to the evader (8 corners)
4. **Dive-boost** — dive steeply to gain speed, then pull up to intercept

**Roles**:
- **Pursuer**: 3 m/s cruise, up to 4.2 m/s in dive (z-velocity component adds to ground speed projection)
- **Evader**: 4.5 m/s, straight escape in 3D away from pursuer

---

## Mathematical Model

### 3D Apollonius Sphere

The Apollonius sphere defines the set of points equidistant in travel time from pursuer and evader:

$$R_{apo} = \frac{r \cdot d}{1 - r^2}, \qquad \text{centre offset from pursuer} = \frac{d}{1 - r^2}$$

where $d = \|\mathbf{p}_E - \mathbf{p}_P\|_2$ (full 3D distance) and $r = v_P / v_E$.

The capture guarantee region is the interior of this sphere centred along the pursuer-evader axis.

### 3D Wall-Herding Velocity

Six faces of the cube define the wall-attraction targets. For each face $f$ with outward normal $\hat{\mathbf{n}}_f$:

$$d_f(\mathbf{p}_E) = \left(5 - |\mathbf{p}_E \cdot \hat{\mathbf{n}}_f|\right)$$

The nearest face $f^* = \arg\min_f d_f(\mathbf{p}_E)$ gives the wall-point:

$$\mathbf{p}_{wall} = \mathbf{p}_E - d_{f^*} \cdot \hat{\mathbf{n}}_{f^*} \cdot \text{sign}(\mathbf{p}_E \cdot \hat{\mathbf{n}}_{f^*})$$

$$\hat{\mathbf{r}}_{wall} = \frac{\mathbf{p}_{wall} - \mathbf{p}_P}{\|\mathbf{p}_{wall} - \mathbf{p}_P\|}$$

$$\mathbf{v}_{herd} = v_P \cdot \frac{\alpha \hat{\mathbf{r}}_{PE} + (1-\alpha)\hat{\mathbf{r}}_{wall}}{\|\alpha \hat{\mathbf{r}}_{PE} + (1-\alpha)\hat{\mathbf{r}}_{wall}\|}$$

### 3D Corner-Targeting

Eight corners of $[-5,5]^3$ are:

$$\mathcal{C} = \{-5, +5\}^3$$

$$\mathbf{p}_{target} = \arg\min_{\mathbf{c} \in \mathcal{C}} \|\mathbf{c} - \mathbf{p}_E\|$$

$$\mathbf{v}_{cmd} = v_P \cdot \frac{\mathbf{p}_{target} - \mathbf{p}_P}{\|\mathbf{p}_{target} - \mathbf{p}_P\|}$$

### Altitude Strategy

#### Pursuer Dive-Boost

When the pursuer is at altitude $z_P > 3$ m and the evader is below, the pursuer dives at angle $\gamma_{dive}$:

$$v_{effective} = \sqrt{v_P^2 + 2 g \Delta z \eta_{eff}}$$

where $\Delta z = z_P - z_E > 0$ is the altitude advantage and $\eta_{eff} \approx 0.3$ is the motor efficiency factor. The dive command:

$$\mathbf{v}_{dive} = v_P \cdot \begin{bmatrix}\cos\phi\cos\theta \\ \sin\phi\cos\theta \\ -\sin\theta\end{bmatrix}, \quad \theta = \gamma_{dive} = 30°$$

After reaching $z_{min} = 0.5$ m the pursuer recovers altitude using:

$$\dot{z}_{recover} = \min\left(k_{climb}(z_{target} - z_P),\; v_{z,max}\right)$$

#### Evader Altitude Escape

The evader appends a vertical component to its escape vector:

$$\mathbf{v}_E = v_E \cdot \frac{\mathbf{p}_E - \mathbf{p}_P + \beta [0, 0, \text{sign}(z_E - z_P)]}{\|\mathbf{p}_E - \mathbf{p}_P + \beta [0, 0, \text{sign}(z_E - z_P)]\|}$$

where $\beta = 1.5$ weights vertical escape.

---

## Key 3D Additions

- **Altitude strategy**: Pursuer dive-boost exploits gravity to temporarily exceed $v_P$; evader uses vertical escape vector
- **3D guidance law**: Wall-herding extends to 6 cube faces; corner-targeting uses 8 corners of $[-5,5]^3$
- **Vertical evasion / geometry**: Apollonius sphere replaces Apollonius circle; 3D corner proximity calculation

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Arena | 10 × 10 × 10 m cube, z ∈ [0.5, 5.5] m |
| Pursuer cruise speed | 3.0 m/s |
| Pursuer dive boost max | 4.2 m/s effective |
| Evader speed | 4.5 m/s |
| Speed ratio r | 0.67 |
| Blend factor alpha | 0.5 |
| Vertical escape weight beta | 1.5 |
| Dive angle gamma | 30° |
| Capture radius | 0.15 m |
| Max simulation time | 60 s |
| z range | 0.5 – 5.5 m |

---

## Implementation

```python
import numpy as np
from numpy.linalg import norm

# Arena and speed parameters
ARENA     = 5.0          # ±5 m in x, y, z
V_PURSUER = 3.0
V_EVADER  = 4.5
ALPHA     = 0.5
BETA_VERT = 1.5
DIVE_ANGLE = np.radians(30)
Z_MIN, Z_MAX = 0.5, 5.5
DT = 0.05

# 8 corners of the cube
CORNERS_3D = np.array([[sx, sy, sz]
                        for sx in [-5, 5]
                        for sy in [-5, 5]
                        for sz in [-5, 5]], dtype=float)

# 6 face normals
FACE_NORMALS = np.array([
    [1,0,0],[-1,0,0],[0,1,0],[0,-1,0],[0,0,1],[0,0,-1]], dtype=float)

def wall_herd_3d(pos_p, pos_e, v_max):
    r_pe = pos_e - pos_p
    r_pe /= norm(r_pe) + 1e-8
    # Find nearest face to evader
    face_dists = [5.0 - abs(pos_e @ n) for n in FACE_NORMALS]
    nearest_idx = int(np.argmin(face_dists))
    n_face = FACE_NORMALS[nearest_idx]
    p_wall = pos_e.copy()
    p_wall -= face_dists[nearest_idx] * n_face * np.sign(pos_e @ n_face)
    r_wall = p_wall - pos_p
    r_wall /= norm(r_wall) + 1e-8
    v_blend = ALPHA * r_pe + (1 - ALPHA) * r_wall
    return v_max * v_blend / (norm(v_blend) + 1e-8)

def corner_target_3d(pos_p, pos_e, v_max):
    nearest_corner = CORNERS_3D[np.argmin([norm(pos_e - c) for c in CORNERS_3D])]
    r = nearest_corner - pos_p
    return v_max * r / (norm(r) + 1e-8)

def dive_boost_3d(pos_p, pos_e, v_max):
    """Dive toward evader if pursuer has altitude advantage."""
    dz = pos_p[2] - pos_e[2]
    if dz > 1.0 and pos_p[2] > 2.0:
        # Dive at DIVE_ANGLE pitch-down
        horiz = pos_e[:2] - pos_p[:2]
        phi = np.arctan2(horiz[1], horiz[0])
        v_dive = v_max * np.array([
            np.cos(phi) * np.cos(DIVE_ANGLE),
            np.sin(phi) * np.cos(DIVE_ANGLE),
            -np.sin(DIVE_ANGLE)
        ])
        return v_dive
    else:
        # Normal pursuit
        r = pos_e - pos_p
        return v_max * r / (norm(r) + 1e-8)

def evader_escape_3d(pos_e, pos_p, v_max):
    """Escape with vertical bias."""
    away = pos_e - pos_p
    away[2] += BETA_VERT * np.sign(pos_e[2] - pos_p[2])
    return v_max * away / (norm(away) + 1e-8)

def apollonius_sphere_radius(pos_p, pos_e, r_speed):
    d = norm(pos_e - pos_p)
    if abs(1 - r_speed**2) < 1e-9:
        return np.inf
    return r_speed * d / (1 - r_speed**2)

def simulate_3d(strategy="wall_herd"):
    pos_p = np.array([0.0, 0.0, 2.0])
    pos_e = np.array([4.0, 0.0, 2.0])
    r = V_PURSUER / V_EVADER
    trajectories = {"pursuer": [pos_p.copy()], "evader": [pos_e.copy()]}
    apo_radii = []

    for step in range(int(60 / DT)):
        # Pursuer strategy
        if strategy == "wall_herd":
            v_p = wall_herd_3d(pos_p, pos_e, V_PURSUER)
        elif strategy == "corner":
            v_p = corner_target_3d(pos_p, pos_e, V_PURSUER)
        elif strategy == "dive":
            v_p = dive_boost_3d(pos_p, pos_e, V_PURSUER)
        else:
            d = pos_e - pos_p
            v_p = V_PURSUER * d / (norm(d) + 1e-8)

        v_e = evader_escape_3d(pos_e, pos_p, V_EVADER)
        pos_p = np.clip(pos_p + v_p * DT, -ARENA, ARENA)
        pos_p[2] = np.clip(pos_p[2], Z_MIN, Z_MAX)
        pos_e = np.clip(pos_e + v_e * DT, -ARENA, ARENA)
        pos_e[2] = np.clip(pos_e[2], Z_MIN, Z_MAX)

        apo_radii.append(apollonius_sphere_radius(pos_p, pos_e, r))
        trajectories["pursuer"].append(pos_p.copy())
        trajectories["evader"].append(pos_e.copy())

        if norm(pos_p - pos_e) < 0.15:
            print(f"Capture at t={step * DT:.2f}s  strategy={strategy}")
            break

    return trajectories, apo_radii
```

---

## Expected Output

- 3D trajectory plots with real z variation (all 4 strategies overlaid)
- Altitude time series: $z_P(t)$ and $z_E(t)$ showing dive events
- Apollonius sphere radius vs time for each strategy
- Capture time comparison table: 2D strategies vs 3D-enhanced dive strategy
- Sensitivity plot: capture rate vs dive angle $\gamma \in [10°, 50°]$

---

## Extensions

1. Optimal dive schedule: solve for the altitude profile that minimises time-to-capture given an energy budget
2. Two-pursuer 3D herding: one pursuer herds horizontally, one blocks vertical escape
3. Circular 3D arena (cylinder) — different Apollonius geometry in cylindrical coordinates

---

## Related Scenarios

- Original 2D version: [S010](../S010_asymmetric_speed.md)
- Truly 3D references: [S001](../S001_basic_intercept.md), [S003](../S003_low_altitude_tracking.md)
