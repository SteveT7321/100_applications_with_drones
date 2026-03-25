# S002 Evasive Maneuver

**Domain**: Pursuit & Evasion | **Difficulty**: ⭐⭐ | **Status**: `[x]` Done

---

## Problem Definition

**Setup**: Pursuer vs Evader 1v1 engagement.

**Roles**:
- **Pursuer**: uses PNG (Proportional Navigation Guidance) or Pure Pursuit strategy
- **Evader**: executes optimal evasion — always flies perpendicular to the line-of-sight direction

**Objective**: compare the effectiveness of different evasion strategies; analyze capture time vs evasion strategy.

---

## Mathematical Model

### Pure Pursuit Strategy (Pursuer)

Velocity direction always points toward the target:

$$\mathbf{v}_{P} = v_{max} \cdot \frac{\mathbf{p}_E - \mathbf{p}_P}{\|\mathbf{p}_E - \mathbf{p}_P\|}$$

### Optimal Evasion Strategy (Evader)

The evader's optimal strategy: keep its velocity perpendicular to the pursuer's line-of-sight direction (most effective against a pure pursuit pursuer).

Line-of-sight direction: $\hat{\mathbf{r}} = \frac{\mathbf{p}_E - \mathbf{p}_P}{\|\mathbf{p}_E - \mathbf{p}_P\|}$

Evasion velocity (perpendicular to line-of-sight) in the $xOy$ plane:

$$\mathbf{v}_E = v_{E,max} \cdot \mathbf{R}_{90°} \cdot \hat{\mathbf{r}}_{xy}$$

### Capture Analysis (Analytical Solution)

For pure pursuit in 2D, with $v_P / v_E = k$:

- $k > 1$: capture is guaranteed; capture time $T = \frac{r_0}{v_P - v_E \cos\theta_0}$ (approximation)
- $k \leq 1$: capture is impossible

---

## Implementation

```python
# Two-drone setup
N = 2   # pursuer index 0, evader index 1

PURSUER_SPEED = 5.0  # m/s
EVADER_SPEED  = 3.5  # m/s (slower than pursuer)

env = CtrlAviary(
    num_drones=2,
    initial_xyzs=np.array([[-2., 0., 2.], [2., 0., 2.]]),
    physics=Physics.PYB,
    ctrl_freq=48,
    gui=False,
)

def evader_strategy(pos_e, pos_p, v_max):
    """Optimal evasion: fly perpendicular to line-of-sight"""
    r = pos_e[:2] - pos_p[:2]
    r_hat = r / (np.linalg.norm(r) + 1e-8)
    perp = np.array([-r_hat[1], r_hat[0], 0.])  # rotate 90°
    return pos_e + perp * v_max / 48   # next target position

def pursuer_strategy(pos_p, pos_e, v_max):
    """Pure pursuit"""
    return pos_e.copy()
```

### Comparative Experiment Design

```python
strategies = {
    "Straight escape":     lambda pe, pp: pe + (pe - pp) / np.linalg.norm(pe - pp + 1e-8) * 0.1,
    "Perpendicular escape": evader_strategy,
    "Random escape":       lambda pe, pp: pe + np.random.randn(3) * 0.1,
    "Spiral escape":       spiral_strategy,   # implement separately
}
```

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Pursuer speed | 5.0 m/s |
| Evader speed | 3.5 m/s |
| Initial distance | 4.0 m |
| Capture radius | 0.15 m |

---

## Expected Output

- **3D trajectory plot**: pursuer follows a curved path, evader performs lateral maneuvers
- **Capture time comparison bar chart**: capture time for each evasion strategy
- Perpendicular escape extends capture time by approximately 20~50% compared to straight escape

---

## Extensions

1. Case where evader speed equals pursuer speed (capture impossible)
2. Train evasion strategy with RL (prerequisite for [S020](S020_pursuit_evasion_game.md))
3. 3D evasion (not limited to horizontal plane)

---

## Related Scenarios

- Prerequisites: [S001](S001_basic_intercept.md)
- Follow-ups: [S003](S003_low_altitude_tracking.md), [S009](S009_differential_game.md)
