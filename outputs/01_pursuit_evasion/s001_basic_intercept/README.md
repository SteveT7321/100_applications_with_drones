# S001 Basic Intercept

**Domain**: Pursuit & Evasion | **Difficulty**: ⭐ | **Status**: ✅ Completed

---

## Problem Definition

**Setup**: A pursuer drone starts at a given position and intercepts a stationary target.

**Objective**:
- Pursuer: minimize time to reach the target
- Target: stationary (no evasion)

**Constraints**:
- Pursuer max speed: $v_{max} = 5$ m/s
- Battery limit: 30 seconds max flight time (`MAX_TIME`)
- Arena: nominally $[-5, 5]^3$ m; not enforced in code (the trajectory never leaves it)

---

## Mathematical Model

### Proportional Navigation Guidance (PNG) — Simplified as PD Control

Line-of-sight vector:

$$\mathbf{r} = \mathbf{p}_{target} - \mathbf{p}_{pursuer}$$

Velocity command (PD control):

$$\mathbf{v}_{cmd} = K_p \cdot \mathbf{r} + K_d \cdot (-\mathbf{v}_{pursuer})$$

where $K_p = 2.0$, $K_d = 0.5$ and $\mathbf{v}_{pursuer}$ is the velocity applied in the previous step (point-mass model, `DroneBase`). The command is clamped to $v_{max} = 5$ m/s and integrated with forward Euler.

Once the command drops below $v_{max}$, the velocity settles to $\mathbf{v} \approx \frac{K_p}{1+K_d}\mathbf{r} = 1.33\,\mathbf{r}$, i.e. the distance decays exponentially with time constant $\tau = (1+K_d)/K_p = 0.75$ s.

This is a position-error simplification of the full PNG law:

$$\mathbf{a}_{cmd} = N \cdot |\mathbf{v}_{closing}| \cdot \dot{\lambda} \cdot \hat{\mathbf{n}}, \quad N=3$$

### Capture Condition

$$\|\mathbf{p}_{pursuer} - \mathbf{p}_{target}\| < r_{capture} = 0.15 \text{ m}$$

---

## Key Parameters

| Parameter | Value | Notes |
|-----------|-------|-------|
| Pursuer start | (-2, -2, 1) m | |
| Target position | (3, 2, 2) m | Stationary |
| Max speed | 5 m/s | |
| Capture radius | 0.15 m | |
| Control frequency | 48 Hz | dt = 1/48 s |
| Max simulation time | 30 s | |
| $K_p$ | 2.0 | PD proportional gain |
| $K_d$ | 0.5 | PD derivative gain |

---

## Implementation

```
src/base/drone_base.py               # Point-mass drone base class
src/01_pursuit_evasion/s001_basic_intercept.py  # Main simulation script
```

```bash
conda activate drones
python src/01_pursuit_evasion/s001_basic_intercept.py
```

---

## Results

| Quantity | Value |
|----------|-------|
| Initial distance | 6.481 m |
| Straight-line lower bound at $v_{max}$, $(d_0 - r_{capture})/v_{max}$ | 1.27 s |
| Time flown saturated at $v_{max}$ | 0.56 s |
| **Capture time** | **2.96 s** |
| Distance at capture | 0.148 m |

The capture time is about 2.3× the straight-line lower bound. The pursuer flies at full speed for only the first 0.56 s (covering ~2.8 m); after that the PD command falls below $v_{max}$ and the remaining ~3.7 m closes exponentially with $\tau = 0.75$ s ($0.75 \cdot \ln(3.7/0.15) \approx 2.4$ s), which is where most of the time goes. A higher $K_p$ (or a pure-pursuit command at constant $v_{max}$) would approach the 1.27 s bound.

**3D Trajectory** — pursuer (red) flies along a straight line to the stationary target (green star); the black X marks the capture point:

![3D Trajectory](trajectory_3d.png)

**Distance vs Time** — linear decrease while saturated at 5 m/s (0–0.56 s), then an exponential tail down to the 0.15 m capture radius:

![Distance vs Time](distance_time.png)

**Animation**:

![Animation](animation.gif)

### Deviations from the scenario card

None. The scenario card was previously an empty stub; it has been filled in from this implementation, so all parameters match. Note that the method is described as "simplified PNG": the code uses a PD position controller, not the full proportional-navigation law (listed as an extension).

---

## Extensions

1. **S002**: Let the target start evading → evasive maneuver
2. Implement full PNG algorithm and compare trajectories
3. Monte Carlo simulation over random initial positions
4. Add battery constraint analysis (capture success rate vs. initial distance)

---

## Related Scenarios

- Prerequisites: none (entry-level scenario)
- Next: [S002 Evasive Maneuver](../../../scenarios/01_pursuit_evasion/S002_evasive_maneuver.md), [S009 Differential Game 1v1 (Differential Game — Lion & Man)](../../../scenarios/01_pursuit_evasion/S009_differential_game.md)

## References

- Shneydor, N.A. (1998). *Missile Guidance and Pursuit*. Horwood.
- [MATH_FOUNDATIONS.md §4.1](../../../MATH_FOUNDATIONS.md)
