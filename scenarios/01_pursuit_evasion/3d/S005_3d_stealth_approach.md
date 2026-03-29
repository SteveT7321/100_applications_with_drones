# S005 3D Upgrade — Stealth Approach

**Domain**: Pursuit & Evasion | **Difficulty**: ⭐⭐⭐⭐ | **Status**: `[ ]` Not started
**Based on**: [S005 original](../S005_stealth_approach.md)

---

## What Changes in 3D

The original S005 keeps both drones at z = 2 m and computes the detection cone check in the horizontal plane only. The evader's detection cone is treated as a 2D wedge. In true 3D, the detection sensor is a **frustum cone** in 3D space pointing along the evader's velocity vector. This changes the stealth problem substantially: the pursuer can now approach from directly above or below to stay outside the forward cone, and the "rear blind-spot" is a cone rather than a wedge, adding a vertical component to the flanking manoeuvre.

---

## Problem Definition

**Setup**: Pursuer approaches an evader while staying outside the evader's 3D forward detection cone.

**Roles**:
- **Evader**: flies at constant heading in 3D (e.g., level flight); has a 3D frustum detection cone (half-angle 60°, range 5 m) pointing along its 3D velocity direction. If the pursuer enters the cone it becomes alerted and executes maximum 3D escape.
- **Pursuer**: must manoeuvre in 3D to approach from the evader's rear hemisphere. Can fly high, low, or lateral to stay outside the cone and exploit vertical blind spots.

**3D Strategies compared**:
1. **Horizontal flank** (2D baseline): encircle at fixed altitude (z = 2 m)
2. **Vertical high approach**: climb above the evader, approach from directly above its tail
3. **Vertical dive approach**: drop below the evader's altitude, approach from below-rear
4. **3D optimal approach**: solve shortest 3D path that stays outside the detection cone volume

**Objective**: show that vertical approaches reduce total approach distance and achieve a more reliable rear-hemisphere entry compared to horizontal flanking alone.

---

## Mathematical Model

### 3D Detection Cone Condition

Evader velocity unit vector: $\hat{\mathbf{v}}_E = \mathbf{v}_E / \|\mathbf{v}_E\|$

Displacement from evader to pursuer: $\mathbf{d} = \mathbf{p}_P - \mathbf{p}_E$

3D cone angle:

$$\cos\alpha_{3D} = \frac{\mathbf{d} \cdot \hat{\mathbf{v}}_E}{\|\mathbf{d}\|}$$

**Detected** if:

$$\alpha_{3D} < \theta_{cone} = 60° \quad \text{AND} \quad \|\mathbf{d}\| < R_{detect} = 5 \text{ m}$$

Note: unlike the 2D case, the pursuer can be directly above the evader with $\mathbf{d} = (0, 0, +h)$.
If $\hat{\mathbf{v}}_E$ is horizontal, then $\cos\alpha_{3D} = 0$ and $\alpha_{3D} = 90° > 60°$ — the pursuer is safely outside the cone at any altitude offset.

### 3D Blind-Spot Intermediate Target

The rear blind-spot in 3D is the hemisphere behind the evader:

$$\mathbf{p}_{rear} = \mathbf{p}_E - R_{offset} \cdot \hat{\mathbf{v}}_E$$

**Vertical high approach** intermediate waypoint:

$$\mathbf{p}_{mid} = \mathbf{p}_E + \begin{bmatrix}0 \\ 0 \\ h_{climb}\end{bmatrix} - \frac{R_{offset}}{2} \hat{\mathbf{v}}_E$$

where $h_{climb} = 2.5$ m above the evader's altitude; the pursuer climbs first, then descends toward the rear.

### Pursuer Speed Bonus (Undetected)

$$v_P = \begin{cases} v_{stealth} = 5.5 \text{ m/s} & \text{if } \alpha_{3D} \geq \theta_{cone} \\ v_{detected} = 4.5 \text{ m/s} & \text{if alert triggered} \end{cases}$$

### Alert Evasion (3D)

Once detected, evader flies directly away from pursuer in 3D:

$$\mathbf{v}_E = v_{E,alert} \cdot \frac{\mathbf{p}_E - \mathbf{p}_P}{\|\mathbf{p}_E - \mathbf{p}_P\|}$$

---

## Key 3D Additions

- Detection condition uses full 3D dot product (not just x-y projection)
- Vertical approach routes: climb-and-drop, belly-approach from below
- 3D rear hemisphere waypoint computation
- Detection cone visualised as a 3D cone mesh attached to the evader
- Altitude constraints: pursuer z ∈ [0.3, 10] m

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Detection cone half-angle | 60° |
| Detection range | 5.0 m |
| Pursuer speed (undetected) | 5.5 m/s |
| Pursuer speed (detected) | 4.5 m/s |
| Evader speed (normal) | 3.0 m/s |
| Evader speed (alerted) | 3.0 m/s (direction changes) |
| Evader heading | +x direction, level flight |
| Initial pursuer position | (-4, 2, 2) m |
| Initial evader position | (0, 0, 2) m |
| Vertical climb offset | 2.5 m |
| Rear offset $R_{offset}$ | 2.0 m |
| Capture radius | 0.15 m |
| Control frequency | 48 Hz |

---

## Expected Output

- **3D trajectory plot**: pursuer paths for all four strategies with the 3D detection cone (semi-transparent mesh) attached to the moving evader
- **Detection status vs time**: binary signal for each strategy
- **Approach angle $\alpha_{3D}$ vs time**: how close each strategy gets to the cone boundary
- **Capture time comparison**: bar chart for horizontal-flank, high-approach, low-approach, 3D-optimal

---

## Extensions

1. Elevation-dependent detection: cone's half-angle widens vertically (birds-eye camera has wider FOV up/down)
2. Pursuer maintains constant altitude band relative to evader to avoid acoustic detection from below
3. 3D stealth path planner (RRT with cone-avoidance constraint) replacing the hand-crafted approach waypoints

---

## Related Scenarios

- Original: [S005 2D version](../S005_stealth_approach.md)
- Truly 3D reference: [S001](../S001_basic_intercept.md), [S003](../S003_low_altitude_tracking.md)
