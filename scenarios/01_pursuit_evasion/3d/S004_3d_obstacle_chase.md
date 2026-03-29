# S004 3D Upgrade — Obstacle-Course Chase

**Domain**: Pursuit & Evasion | **Difficulty**: ⭐⭐⭐⭐ | **Status**: `[ ]` Not started
**Based on**: [S004 original](../S004_obstacle_chase.md)

---

## What Changes in 3D

The original S004 places all four obstacles at z = 2 m and fixes both drones at z = 2 m, effectively making the APF repulsion act only in the horizontal plane. In a genuine 3D scenario, obstacles have full spherical (or cylindrical) extent, and the pursuer can fly over or under an obstacle rather than only going around it horizontally. The evader gains the option to exploit vertical obstacle coverage as a shield.

---

## Problem Definition

**Setup**: Pursuer chases an evader through a field of 5 volumetric obstacles (spheres and one vertical cylinder representing a building). Both drones have unconstrained altitude.

**Roles**:
- **Pursuer**: 3D APF — attraction toward evader + repulsion from all obstacle surfaces in 3D. May choose to fly over an obstacle when vertical detour is shorter.
- **Evader**: 3D straight escape + 3D APF repulsion. Can use obstacles as altitude shields — e.g., fly low behind a tall cylinder.

**Objective**: compare 2D-confined APF (fixed z) against full 3D APF. Show that 3D routing allows the pursuer to over-fly obstacles, reducing path length and capture time.

---

## Mathematical Model

### 3D APF Total Velocity

$$\mathbf{v}_{cmd} = \mathbf{v}_{att} + \mathbf{v}_{rep}, \quad \text{clipped to } v_{max}$$

**Attraction** (3D):

$$\mathbf{v}_{att} = v_{max} \cdot \frac{\mathbf{p}_E - \mathbf{p}_P}{\|\mathbf{p}_E - \mathbf{p}_P\|}$$

**Repulsion from sphere obstacle $i$** (3D surface distance):

$$\rho_i = \|\mathbf{p} - \mathbf{c}_i\| - r_i$$

$$\mathbf{v}_{rep,i} = K_{rep} \left(\frac{1}{\rho_i} - \frac{1}{\rho_0}\right) \frac{1}{\rho_i^2} \cdot \frac{\mathbf{p} - \mathbf{c}_i}{\|\mathbf{p} - \mathbf{c}_i\|} \quad \text{for } \rho_i < \rho_0$$

**Repulsion from vertical cylinder obstacle** (centre $(x_c, y_c)$, height $[z_{bot}, z_{top}]$, radius $r_c$):

$$\rho_{cyl} = \sqrt{(x - x_c)^2 + (y - y_c)^2} - r_c$$

Active only when $z_{bot} \le z \le z_{top}$; repulsion direction is horizontal outward.
When $z > z_{top}$, the drone has cleared the obstacle — no repulsion.

### Over-Fly Decision Heuristic

At each step, compare the estimated path lengths:
- **Horizontal detour**: circumference fraction around the sphere, at current altitude
- **Vertical detour**: climb $\Delta z = r_i + 0.5$ m above obstacle centre, then descend

Choose the option with smaller Euclidean distance to the evader after the manoeuvre.

### Altitude Bounds

$$z \in [0.3,\; 10] \text{ m}$$

---

## Key 3D Additions

- Full 3D obstacle surface normals for repulsion (not just the x-y component)
- Vertical cylinder obstacle type with altitude-conditioned repulsion
- Over-fly vs go-around decision logic based on 3D path length estimate
- Evader altitude shield tactic: hide in the cylinder's shadow (fly at cylinder's altitude)
- 3D collision check: pursuer crash = any component of position inside obstacle volume

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Pursuer start | (-4, 0, 2) m |
| Evader start | (4, 0, 2) m |
| Pursuer speed | 5.0 m/s |
| Evader speed | 3.5 m/s |
| $K_{rep}$ | 4.0 |
| $\rho_0$ | 1.5 m |
| Capture radius | 0.15 m |
| Control frequency | 48 Hz |
| Altitude bounds | [0.3, 10] m |

**Obstacle layout (3D)**:

| Obstacle | Centre (x, y, z) | Radius / Height |
|----------|-----------------|-----------------|
| Sphere 1 | (-2.0, 0.0, 2.0) | r = 0.60 m |
| Sphere 2 | (-0.5, 0.5, 3.5) | r = 0.55 m (elevated) |
| Sphere 3 | ( 1.0, -0.4, 1.5) | r = 0.50 m (low) |
| Sphere 4 | ( 2.5, 0.3, 2.5) | r = 0.45 m |
| Cylinder | ( 0.5, -1.0, —) | r = 0.4 m, z ∈ [0, 5] m |

---

## Expected Output

- **3D trajectory plot** showing pursuer over-flying sphere 2 and going around the cylinder
- **Altitude profile vs time** for both pursuer and evader
- **Path length comparison**: 2D-fixed-z APF vs full 3D APF (bar chart)
- **Capture time comparison**: 2D vs 3D routing

---

## Extensions

1. Replace spheres with axis-aligned bounding boxes (buildings) for an urban canyon scenario
2. Dynamic obstacles: sphere moving laterally — pursuer must predict obstacle path
3. 3D RRT* planner replacing APF to escape local minima caused by the tall cylinder

---

## Related Scenarios

- Original: [S004 2D version](../S004_obstacle_chase.md)
- Truly 3D reference: [S001](../S001_basic_intercept.md), [S003](../S003_low_altitude_tracking.md)
