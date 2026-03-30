# S016 3D Upgrade — Airspace Defense

**Domain**: Pursuit & Evasion | **Difficulty**: ⭐⭐⭐⭐ | **Status**: `[x]` Complete
**Based on**: [S016 original](../S016_airspace_defense.md)

---

## What Changes in 3D

The original already uses a 3D sphere for the protected zone, but all positions are restricted to $z = 2.0$ m fixed — intruders approach only from horizontal directions and the Apollonius sphere condition is never exercised in the vertical dimension. In the full 3D upgrade, intruders approach from all angles including steep dives from directly above and low-angle oblique entry below the zone. The defender uses altitude actively to cut off diagonal approach vectors, and the Apollonius sphere condition is evaluated in genuine 3D geometry.

---

## Problem Definition

**Setup**: A protected zone (sphere of radius 2 m at origin) is threatened by $N = 6$ intruders approaching from all directions in 3D — including top-down dive, oblique high, horizontal, oblique low, and steep climb from below. A single defender starts near the zone centre and must intercept each intruder before it crosses the zone boundary.

**Objective**: determine the 3D capture guarantee region; compare Pure Pursuit vs 3D Proportional Navigation; analyse how defender altitude positioning affects coverage of diagonal attack vectors.

---

## Mathematical Model

### Protected Zone (unchanged)

$$\mathcal{Z} = \{ \mathbf{p} \in \mathbb{R}^3 \mid \|\mathbf{p}\| \leq R_{zone} \}$$

### 3D Apollonius Intercept Condition

The defender can intercept the intruder before it reaches the zone boundary if and only if:

$$\frac{\|\mathbf{p}_D - \mathbf{p}_I\|}{v_D} \leq \frac{\|\mathbf{p}_I\| - R_{zone}}{v_I}$$

The Apollonius intercept surface in 3D is a sphere in $\mathbb{R}^3$ relative space. Intruder approach direction $\hat{\mathbf{v}}_I$ sets the time-to-zone:

$$t_{zone} = \frac{\|\mathbf{p}_I\| - R_{zone}}{v_I}$$

For diagonal approaches, $\|\mathbf{p}_I\| - R_{zone}$ is the same at equal radial distance regardless of elevation angle — the 3D geometry is spherically symmetric.

### 3D Proportional Navigation

3D LOS unit vector:

$$\hat{\boldsymbol{\lambda}} = \frac{\mathbf{p}_I - \mathbf{p}_D}{\|\mathbf{p}_I - \mathbf{p}_D\|}$$

LOS rate vector (perpendicular component of relative velocity):

$$\dot{\hat{\boldsymbol{\lambda}}} = \frac{\mathbf{v}_{rel} - (\mathbf{v}_{rel} \cdot \hat{\boldsymbol{\lambda}})\hat{\boldsymbol{\lambda}}}{\|\mathbf{p}_I - \mathbf{p}_D\|}$$

Closing speed:

$$v_c = -(\mathbf{v}_I - \mathbf{v}_D) \cdot \hat{\boldsymbol{\lambda}}$$

3D PN acceleration command:

$$\mathbf{a}_{cmd} = N \cdot v_c \cdot \dot{\hat{\boldsymbol{\lambda}}}$$

Velocity integration (with saturation at $v_D$):

$$\mathbf{v}_D^{new} = \mathbf{v}_D + \mathbf{a}_{cmd} \cdot \Delta t, \quad \mathbf{v}_D^{new} \leftarrow v_D \cdot \frac{\mathbf{v}_D^{new}}{\|\mathbf{v}_D^{new}\|}$$

### Altitude Strategy

The defender pre-positions at an altitude that minimises worst-case intercept time over all $N$ intruder directions. For $K$ candidate defender altitudes $z_D \in \{0.5, 1.0, 2.0, 3.5, 5.0\}$ m, the optimal starting altitude minimises the maximum intercept time:

$$z_D^* = \arg\min_{z_D} \max_{j=1}^{N} \frac{\|\mathbf{p}_{D}(z_D) - \mathbf{p}_{I_j}(0)\| / v_D - t_{zone,j}}{1}$$

In practice, $z_D^* \approx 0$ (zone centre altitude) for a symmetric intruder distribution, but for asymmetric attack patterns (more intruders from above) the defender should pre-position lower.

### Diagonal Approach Analysis

For an intruder approaching at elevation angle $\phi_{el}$ from range $R_{init}$, the time-to-zone is:

$$t_{zone} = \frac{R_{init} - R_{zone}}{v_I}$$

The defender must travel:

$$d_D = \|\mathbf{p}_D - \mathbf{p}_I(t=0)\| = \sqrt{R_{init}^2 + \|\mathbf{p}_D\|^2 - 2 R_{init} \|\mathbf{p}_D\| \cos\theta}$$

where $\theta$ is the angle between $\mathbf{p}_D$ and $\mathbf{p}_I$ directions from the origin. The capture guarantee requires $d_D / v_D \leq t_{zone}$.

---

## Key 3D Additions

- **Altitude strategy**: defender pre-positions at altitude minimising worst-case intercept time over all 3D attack directions; altitude selection is a 1D grid search
- **3D guidance law**: full 3D PN with LOS rate computed as perpendicular component of 3D relative velocity; velocity integration in 3D
- **Vertical evasion / geometry**: intruders approach from 6 directions including top-down ($\phi_{el} = -90°$), oblique ($\phi_{el} = ±45°$), horizontal ($\phi_{el} = 0°$), and upward ($\phi_{el} = +60°$)

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Protected zone radius | 2 m |
| Defender start position | (0, 0, 0) m (zone centre) |
| Defender speed | 6 m/s |
| Intruder speed | 4 m/s |
| Speed ratio | 1.5 |
| PN gain N | 3 |
| Capture radius | 0.15 m |
| Number of intruders | 6 (all 3D angles) |
| Intruder start positions | range 8 m, from 6 directions |
| z range for defender | 0.5 – 6.0 m |
| dt | 0.02 s |

---

## Implementation

```python
import numpy as np
from numpy.linalg import norm

R_ZONE = 2.0
V_DEFENDER = 6.0
V_INTRUDER = 4.0
PN_GAIN = 3.0
CAPTURE_R = 0.15
DT = 0.02
T_MAX = 10.0
R_INIT = 8.0  # intruder starting range from origin

# 6 intruder approach directions in 3D
# (azimuth_deg, elevation_deg) pairs
APPROACH_ANGLES = [
    (0,   0),    # horizontal east
    (180, 0),    # horizontal west
    (90,  0),    # horizontal north
    (45,  45),   # oblique high NE
    (270,-45),   # oblique low SW
    (0,  -80),   # near-vertical dive from above
]

def angles_to_pos(az_deg, el_deg, r=R_INIT):
    az = np.radians(az_deg)
    el = np.radians(el_deg)
    return r * np.array([
        np.cos(el) * np.cos(az),
        np.cos(el) * np.sin(az),
        np.sin(el)
    ])

INTRUDER_STARTS = [angles_to_pos(az, el) for az, el in APPROACH_ANGLES]

def time_to_zone(pos_i, r_zone=R_ZONE, v_i=V_INTRUDER):
    """Time for intruder to reach zone boundary from current position."""
    r = norm(pos_i)
    return max(r - r_zone, 0) / v_i

def pure_pursuit_3d(p_def, p_int, speed, dt):
    d = p_int - p_def
    d /= norm(d) + 1e-8
    return p_def + d * speed * dt

def pn_step_3d(p_def, p_int, v_def, v_int, N, speed, dt):
    los = p_int - p_def
    los_dist = norm(los) + 1e-8
    los_hat = los / los_dist
    v_rel = v_int - v_def
    # Perpendicular LOS rate
    los_dot = (v_rel - np.dot(v_rel, los_hat) * los_hat) / los_dist
    v_c = -np.dot(v_rel, los_hat)
    a_cmd = N * v_c * los_dot
    v_new = v_def + a_cmd * dt
    v_new = v_new / (norm(v_new) + 1e-8) * speed
    return p_def + v_new * dt, v_new

def optimal_defender_altitude(intruder_starts, z_candidates, v_d=V_DEFENDER):
    """Grid search for defender altitude minimising worst-case intercept time gap."""
    best_z, best_score = 2.0, -np.inf
    for z in z_candidates:
        p_def = np.array([0., 0., z])
        # Compute intercept feasibility for each intruder
        scores = []
        for p_i in intruder_starts:
            d_to_intruder = norm(p_def - p_i)
            t_intercept = d_to_intruder / v_d
            t_zone_val  = time_to_zone(p_i)
            scores.append(t_zone_val - t_intercept)  # positive = feasible
        worst = min(scores)
        if worst > best_score:
            best_score = worst
            best_z = z
    return best_z, best_score

# Find optimal altitude
Z_CANDIDATES = [0.5, 1.0, 2.0, 3.0, 4.0]
z_opt, margin = optimal_defender_altitude(INTRUDER_STARTS, Z_CANDIDATES)
print(f"Optimal defender altitude: z={z_opt:.1f}m, min margin={margin:.2f}s")
DEFENDER_START = np.array([0.0, 0.0, z_opt])

def simulate_3d(p_int_0, strategy="pn"):
    p_def = DEFENDER_START.copy()
    p_int = p_int_0.copy()
    v_def = np.zeros(3)
    # Intruder flies straight toward zone centre
    v_int = -p_int_0 / norm(p_int_0) * V_INTRUDER
    traj_d, traj_i = [p_def.copy()], [p_int.copy()]

    for _ in range(int(T_MAX / DT)):
        p_int = p_int + v_int * DT
        # Check zone penetration
        if norm(p_int) <= R_ZONE:
            return traj_d, traj_i, "intruder_wins"
        if strategy == "pure_pursuit":
            p_def = pure_pursuit_3d(p_def, p_int, V_DEFENDER, DT)
        else:  # PN
            p_def, v_def = pn_step_3d(
                p_def, p_int, v_def, v_int, PN_GAIN, V_DEFENDER, DT)
        traj_d.append(p_def.copy())
        traj_i.append(p_int.copy())
        if norm(p_def - p_int) < CAPTURE_R:
            return traj_d, traj_i, "defender_wins"

    return traj_d, traj_i, "timeout"

# Run all 6 approach angles × 2 strategies
for i, (az, el) in enumerate(APPROACH_ANGLES):
    p_start = INTRUDER_STARTS[i]
    for strat in ["pure_pursuit", "pn"]:
        traj_d, traj_i, outcome = simulate_3d(p_start, strat)
        print(f"Az={az:4d}° El={el:4d}° | {strat:14s}: {outcome}")
```

---

## Expected Output

- 3D plot: protected zone sphere, 6 intruder approach trajectories from all 3D angles, defender paths (colour-coded by strategy)
- Success/fail table: 6 × 2 grid (approach angle × strategy)
- Optimal defender altitude plot: worst-case intercept margin vs defender z
- Miss distance vs elevation angle $\phi_{el}$: shows PN outperforms pure pursuit for steep diagonal attacks
- Apollonius sphere visualisation overlaid on 3D scene

---

## Extensions

1. Multiple simultaneous intruders from all 6 directions — defender must prioritise using time-to-zone ranking
2. Intruder executes a last-second altitude jink during final approach to defeat PN
3. Two cooperating defenders at different altitude tiers — analyse expanded 3D guarantee region

---

## Related Scenarios

- Original 2D version: [S016](../S016_airspace_defense.md)
- Truly 3D references: [S001](../S001_basic_intercept.md), [S003](../S003_low_altitude_tracking.md)
