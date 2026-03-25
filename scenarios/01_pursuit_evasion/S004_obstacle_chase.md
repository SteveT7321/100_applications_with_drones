# S004 Obstacle-Course Chase

**Domain**: Pursuit & Evasion | **Difficulty**: ⭐⭐⭐ | **Status**: `[x]` Done

---

## Problem Definition

**Setup**: Pursuer chases an evader through a field of 4 spherical obstacles.

**Roles**:
- **Pursuer**: uses Attractive-Repulsive Potential Field (APF) to navigate toward the evader while avoiding obstacles
- **Evader**: straight escape + APF obstacle repulsion

**Objective**: show that APF enables successful capture through a cluttered environment; without obstacle avoidance the pursuer crashes immediately.

---

## Mathematical Model

### Attractive-Repulsive Potential Field (APF)

Total pursuer velocity command:

$$\mathbf{v}_{cmd} = \mathbf{v}_{att} + \mathbf{v}_{rep}$$

**Attraction** — pure pursuit toward evader:

$$\mathbf{v}_{att} = v_{max} \cdot \frac{\mathbf{p}_E - \mathbf{p}_P}{\|\mathbf{p}_E - \mathbf{p}_P\|}$$

**Repulsion** — sum over all obstacles:

$$\mathbf{v}_{rep} = \sum_{i} K_{rep} \left(\frac{1}{\rho_i} - \frac{1}{\rho_0}\right) \frac{1}{\rho_i^2} \, \hat{\mathbf{n}}_i \quad \text{for } \rho_i < \rho_0$$

where $\rho_i = \|\mathbf{p} - \mathbf{c}_i\| - r_i$ is the distance to obstacle $i$'s surface,
$\hat{\mathbf{n}}_i$ is the outward unit normal, and $\rho_0$ is the influence range.

The combined velocity is clamped to $v_{max}$.

### Capture and Collision Conditions

$$\|\mathbf{p}_P - \mathbf{p}_E\| < r_{capture} = 0.15 \text{ m}$$

$$\|\mathbf{p}_P - \mathbf{c}_i\| < r_i + 0.05 \text{ m} \Rightarrow \text{crash}$$

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Pursuer start | (-4, 0, 2) m |
| Evader start | (3.5, 0, 2) m |
| Initial distance | 7.5 m |
| Pursuer speed | 5.0 m/s |
| Evader speed | 3.5 m/s |
| $K_{rep}$ | 4.0 |
| $\rho_0$ | 1.2 m |
| Capture radius | 0.15 m |
| Control frequency | 48 Hz |
| Max simulation time | 15 s |

**Obstacle layout**:

| Obstacle | Centre (x, y, z) | Radius |
|----------|-----------------|--------|
| 1 | (-2.0, 0.0, 2.0) | 0.60 m |
| 2 | (-0.5, 0.5, 2.0) | 0.55 m |
| 3 | ( 1.0, -0.4, 2.0) | 0.50 m |
| 4 | ( 2.5, 0.3, 2.0) | 0.45 m |

---

## Implementation

```
src/base/drone_base.py               # Point-mass drone base class
src/pursuit/s004_obstacle_chase.py   # Main simulation script
```

```bash
conda activate drones
python src/pursuit/s004_obstacle_chase.py
```

---

## Expected Output

- 3D trajectory plot with sphere obstacles
- Distance vs time (both cases)
- APF repulsion magnitude over time

---

## Extensions

1. Replace spheres with cylinders (buildings) for urban canyon pursuit
2. Dynamic obstacles (moving obstacles)
3. Global path planner (A* or RRT) instead of local APF — avoids local minima
4. Multi-pursuer cooperative APF → [S005](S005_multi_pursuer.md)

---

## Related Scenarios

- Prerequisites: [S001](S001_basic_intercept.md), [S003](S003_low_altitude_tracking.md)
- Next: [S005](S005_multi_pursuer.md)
- See [domains/01_pursuit_evasion.md](../../domains/01_pursuit_evasion.md)
