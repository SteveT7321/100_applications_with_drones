# S001 Basic Intercept

**Domain**: Pursuit & Evasion | **Difficulty**: ⭐ | **Status**: `[x]` Completed

→ **[Full details, results, and visualizations](../../outputs/01_pursuit_evasion/s001_basic_intercept/)**

---

## Problem Definition

**Setup**: A single pursuer drone starts at $(-2, -2, 1)$ m and must reach a stationary target at $(3, 2, 2)$ m. The start and target differ in all three axes, so the intercept is a genuine 3D flight.

**Roles**:
- **Pursuer**: closes on the target with a PD position controller (a simplified form of Proportional Navigation Guidance)
- **Target**: stationary, no evasion

**Objective**: minimise the time until the pursuer is within the capture radius; verify that the controller converges without overshoot.

**Constraints**:
- Pursuer speed limited to $v_{max} = 5$ m/s
- Flight time limited to 30 s (battery)
- Nominal arena $[-5, 5]^3$ m (not enforced — the straight-line path stays inside it)

---

## Mathematical Model

### Point-Mass Kinematics

The drone is a point mass whose velocity equals the (clamped) command, integrated with forward Euler at $\Delta t = 1/48$ s:

$$\mathbf{v}_{k} = \text{clip}_{v_{max}}(\mathbf{v}_{cmd,k}), \qquad \mathbf{p}_{k+1} = \mathbf{p}_k + \mathbf{v}_k \, \Delta t$$

where $\text{clip}_{v_{max}}(\mathbf{u}) = \mathbf{u} \cdot \min(1, v_{max}/\|\mathbf{u}\|)$.

### PD Guidance (simplified PNG)

Line-of-sight (error) vector:

$$\mathbf{r}_k = \mathbf{p}_{target} - \mathbf{p}_k$$

Velocity command, with damping on the previously applied velocity:

$$\mathbf{v}_{cmd,k} = K_p \, \mathbf{r}_k - K_d \, \mathbf{v}_{k-1}, \qquad K_p = 2.0,\; K_d = 0.5$$

Behaviour:
- **Far from the target** the command exceeds $v_{max}$, so the pursuer flies straight at 5 m/s.
- **Near the target** the command is unsaturated and settles to $\mathbf{v} \approx \frac{K_p}{1+K_d}\mathbf{r}$, so the distance decays exponentially with time constant $\tau = (1+K_d)/K_p = 0.75$ s.

The full PNG law, $\mathbf{a}_{cmd} = N \, |\mathbf{v}_{closing}| \, \dot{\lambda} \, \hat{\mathbf{n}}$ with $N = 3$, is not used here (it is only meaningful against a moving target; see S002).

### Capture Condition

$$\|\mathbf{p}_k - \mathbf{p}_{target}\| < r_{capture} = 0.15 \text{ m}$$

### Lower Bound

A pursuer flying straight at $v_{max}$ the whole way would need

$$T_{min} = \frac{\|\mathbf{p}_{target} - \mathbf{p}_0\| - r_{capture}}{v_{max}} = \frac{6.481 - 0.15}{5} = 1.27 \text{ s}$$

The PD controller is slower than this bound because of its exponential tail.

---

## Implementation

```python
from src.base.drone_base import DroneBase   # point-mass, velocity clamp, Euler step

drone = DroneBase(INIT_POS, max_speed=MAX_SPEED, dt=DT)
for step in range(int(MAX_TIME / DT)):
    error = TARGET_POS - drone.pos
    if np.linalg.norm(error) < CAPTURE_R:
        capture_time = step * DT
        break
    v_cmd = Kp * error + Kd * (-drone.vel)
    drone.step(v_cmd)
```

Script: `src/01_pursuit_evasion/s001_basic_intercept.py` — functions `run_simulation()`, `plot_trajectory_3d()`, `plot_distance_time()`, `save_animation()`.

---

## Key Parameters

| Parameter | Value | Notes |
|-----------|-------|-------|
| Pursuer start | (-2, -2, 1) m | |
| Target position | (3, 2, 2) m | Stationary |
| Max speed $v_{max}$ | 5 m/s | |
| Capture radius | 0.15 m | |
| Control frequency | 48 Hz | $\Delta t = 1/48$ s |
| Max simulation time | 30 s | |
| $K_p$ | 2.0 | proportional gain |
| $K_d$ | 0.5 | damping gain (on previous velocity) |

---

## Expected Output

- **3D trajectory plot** (`trajectory_3d.png`): straight red path from start to the green target star, capture point marked
- **Distance vs time** (`distance_time.png`): linear decrease while saturated at $v_{max}$, then exponential decay to the capture radius
- **Animation** (`animation.gif`)
- Capture well inside the 30 s limit, between the 1.27 s lower bound and a few seconds (the exponential tail with $\tau = 0.75$ s dominates)

---

## Extensions

1. Implement the full PNG law and compare against the PD controller (needs a moving target — see S002)
2. Tune $K_p$/$K_d$ and plot capture time vs gain; compare with constant-speed pure pursuit (approaches $T_{min}$)
3. Monte Carlo over random start positions inside the arena
4. Battery constraint analysis: capture success rate vs initial distance

---

## Related Scenarios

- Prerequisites: none (entry-level scenario)
- Follow-ups: [S002 Evasive Maneuver](S002_evasive_maneuver.md), [S009 Differential Game 1v1 (Differential Game — Lion & Man)](S009_differential_game.md)

## References

- Shneydor, N.A. (1998). *Missile Guidance and Pursuit*. Horwood.
- [MATH_FOUNDATIONS.md §4.1](../../MATH_FOUNDATIONS.md)
