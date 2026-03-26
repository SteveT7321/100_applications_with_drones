# S007 Jamming & Blind Pursuit

**Domain**: Pursuit & Evasion | **Difficulty**: ⭐⭐⭐ | **Status**: `[ ]` Not started

---

## Problem Definition

**Setup**: The pursuer's GPS is intermittently jammed. During jam intervals it must use dead reckoning (integrating the last known velocity) to estimate its own position. Estimation error accumulates while blind.

**Comparison**:
- **Dead reckoning**: continue at last known velocity during jam
- **Freeze on jam**: stop moving during jam window
- **Baseline**: perfect GPS (no jamming)

**Roles**:
- **Pursuer**: pure pursuit with periodic GPS dropout
- **Evader**: straight escape (no GPS effect)

---

## Mathematical Model

### GPS-Available Phase

Pursuer knows exact position and pursues normally:

$$\mathbf{v}_{cmd} = v_P \cdot \frac{\mathbf{p}_E - \mathbf{p}_P}{\|\mathbf{p}_E - \mathbf{p}_P\|}$$

### Dead-Reckoning Phase

During jam window $t \in [t_j,\; t_j + T_{jam}]$, estimated position:

$$\hat{\mathbf{p}}_P(t) = \mathbf{p}_P(t_j) + \mathbf{v}_{last} \cdot (t - t_j) + \boldsymbol{\varepsilon}(t)$$

where $\boldsymbol{\varepsilon}(t) \sim \mathcal{N}\!\left(\mathbf{0},\; \sigma_{drift}^2 (t - t_j) \mathbf{I}\right)$ models IMU drift.

### Accumulated Position Error

$$e(t) = \|\hat{\mathbf{p}}_P(t) - \mathbf{p}_P(t)\| = \mathcal{O}\!\left(\sigma_{drift} \sqrt{t - t_j}\right)$$

### Jam Schedule

Jammed during $[n T_{period},\; n T_{period} + T_{jam}]$ for integer $n \geq 0$.

---

## Implementation

```python
JAM_PERIOD   = 3.0   # s
JAM_DURATION = 1.0   # s  (33% duty cycle)
SIGMA_DRIFT  = 0.05  # m/s — IMU drift noise std

def is_jammed(t):
    return (t % JAM_PERIOD) < JAM_DURATION

pos_estimate = pursuer.pos.copy()
v_last = np.zeros(3)

for step in range(max_steps):
    t = step * DT
    if not is_jammed(t):
        pos_estimate = pursuer.pos.copy()   # GPS fix
        v_last       = pursuer.vel.copy()
    else:
        if strategy == 'dead_reckoning':
            pos_estimate += v_last * DT + rng.normal(0, SIGMA_DRIFT * DT, 3)
        # else: freeze — pos_estimate unchanged, pursuer gets v_cmd=0

    v_cmd = V_PURSUER * (evader.pos - pos_estimate) / (norm(...) + 1e-8)
    pursuer.step(v_cmd)
```

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Jam period | 3 s |
| Jam duration | 1 s (33% duty cycle) |
| IMU drift sigma | 0.05 m/s |
| Pursuer speed | 5 m/s |
| Evader speed | 3 m/s |
| Initial distance | 6 m |

---

## Expected Output

- 3D trajectories: zigzag (dead reckoning) vs frozen gaps vs smooth baseline
- Position estimation error vs time (spikes at jam onset)
- Capture time: dead reckoning vs freeze vs no-jam baseline

---

## Extensions

1. Kalman filter with IMU: reduced dead-reckoning drift
2. Adversarial jammer: activates only when pursuer is within 1 m of capture
3. Pursuer reduces speed during jam to limit positional divergence

---

## Related Scenarios

- Prerequisites: [S001](S001_basic_intercept.md), [S002](S002_evasive_maneuver.md)
- Next: [S008](S008_stochastic_pursuit.md)
