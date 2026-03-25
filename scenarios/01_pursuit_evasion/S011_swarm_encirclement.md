# S011 Swarm Encirclement

**Domain**: Pursuit & Evasion | **Difficulty**: ⭐⭐ | **Status**: `[ ]` Not started

---

## Problem Definition

**Setup**: 3 pursuer drones (Swarm) simultaneously encircle 1 evader from different directions.

**Strategy**:
- Each pursuer is evenly distributed on a circle centered on the evader
- Synchronously shrink the encirclement radius
- The evader attempts to break through the encirclement

---

## Mathematical Model

### Encirclement Design

$N$ pursuers distributed evenly; the target position for pursuer $i$:

$$\mathbf{p}_i^{target} = \mathbf{p}_E + R(t) \cdot \begin{bmatrix} \cos(2\pi i/N + \psi_E) \\ \sin(2\pi i/N + \psi_E) \\ 0 \end{bmatrix}$$

where $R(t)$ is the shrinking radius: $R(t) = R_0 - v_{shrink} \cdot t$, and $\psi_E$ is the heading angle.

### Shrink Rate Optimization

To guarantee capture (encirclement radius reduced to $r_{capture}$) within $T$ seconds:

$$v_{shrink} = \frac{R_0 - r_{capture}}{T}$$

### Evader Breakout Condition

If the evader speed $v_E$ is greater than the pursuer's lateral movement speed:

$$v_E > v_P \sin\left(\frac{\pi}{N}\right)$$

then a breakout through the gap is theoretically possible.

---

## Implementation

```python
N_PURSUERS = 3
R_INIT     = 4.0   # initial encirclement radius
T_CONVERGE = 15.0  # desired shrink time

def compute_encirclement_targets(pos_evader, t, n=N_PURSUERS):
    R = max(R_INIT - (R_INIT - 0.3) * t / T_CONVERGE, 0.3)
    targets = []
    for i in range(n):
        angle = 2 * np.pi * i / n
        dx = R * np.cos(angle)
        dy = R * np.sin(angle)
        targets.append(pos_evader + np.array([dx, dy, 0]))
    return np.array(targets)

# Initial positions: pursuers evenly distributed on the encirclement, evader at center
init_xyzs = []
for i in range(N_PURSUERS):
    angle = 2 * np.pi * i / N_PURSUERS
    init_xyzs.append([R_INIT * np.cos(angle), R_INIT * np.sin(angle), 2.0])
init_xyzs.append([0., 0., 2.0])   # evader (index N_PURSUERS)
```

---

## Expected Output

- Top-down view (XY plane): encirclement shrink animation screenshot
- Encirclement radius vs time
- Each pursuer's distance to evader vs time

---

## Extensions

1. Increase number of pursuers (3 → 6) and observe encirclement efficiency
2. Evader uses optimal breakout strategy (S013)
3. 3D encirclement (spherical encirclement)

---

## Related Scenarios

- Prerequisites: [S001](S001_basic_intercept.md)
- Follow-ups: [S013](S013_pincer_movement.md), [S018](S018_multi_target_intercept.md)
