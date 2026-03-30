# S018 3D Upgrade — Multi-Target Intercept

**Domain**: Pursuit & Evasion | **Difficulty**: ⭐⭐⭐⭐ | **Status**: `[x]` Complete
**Based on**: [S018 original](../S018_multi_target_intercept.md)

---

## What Changes in 3D

The original places all four targets and the pursuer at $z = 2.0$ m fixed, making the tour a purely 2D TSP problem. In 3D, targets are at different altitudes and the pursuer's travel time between two points depends on the **3D Euclidean distance**, which includes a vertical component. Climbing costs extra time due to limited vertical speed, so the altitude-aware travel time differs from the straight-line Euclidean time. The optimal ordering must now account for altitude sequencing — visiting targets in an altitude-monotone order (either all ascending or all descending) may be optimal to minimise total climb cost.

---

## Problem Definition

**Setup**: A single pursuer must intercept $N = 4$ stationary targets distributed at different altitudes. The pursuer has limited vertical speed $v_{z,max} < v_{xy,max}$, making the 3D travel time non-Euclidean. The goal is to find the visit ordering that minimises total flight time including climb/descent cost.

**Objectives**: compare nearest-neighbour heuristic vs brute-force optimal vs altitude-aware nearest-neighbour (which prefers targets reachable with minimal altitude change).

---

## Mathematical Model

### 3D Travel Time (Altitude-Aware)

Travel time from position $\mathbf{p}_a$ to $\mathbf{p}_b$ with asymmetric horizontal/vertical speeds:

$$t_{travel}(\mathbf{p}_a, \mathbf{p}_b) = \max\!\left(\frac{\Delta_{xy}}{v_{xy}},\; \frac{|\Delta z|}{v_{z,max}}\right)$$

where $\Delta_{xy} = \sqrt{(x_b - x_a)^2 + (y_b - y_a)^2}$ and $\Delta z = z_b - z_a$.

This decoupled model reflects that horizontal and vertical motion are produced by independent control channels. If $|\Delta z| / v_{z,max} > \Delta_{xy} / v_{xy}$, the vertical leg is the bottleneck.

### Total Mission Time

For visit ordering $\sigma = (\sigma_1, \sigma_2, \sigma_3, \sigma_4)$:

$$T_{total}(\sigma) = \sum_{k=1}^{N} t_{travel}(\mathbf{p}_{\sigma_{k-1}}, \mathbf{p}_{\sigma_k})$$

where $\mathbf{p}_{\sigma_0} = \mathbf{p}_P(0)$ is the pursuer start.

### Brute-Force Optimal (3D)

$$\sigma^* = \arg\min_{\sigma \in S_N} T_{total}(\sigma)$$

For $N = 4$: all $4! = 24$ orderings evaluated.

### Standard Nearest-Neighbour

$$\sigma_{k+1} = \arg\min_{j \notin \{\sigma_1,\ldots,\sigma_k\}} \|\mathbf{p}_j - \mathbf{p}_{\sigma_k}\|_2$$

### Altitude-Aware Nearest-Neighbour

Weighted distance considering altitude penalty:

$$d_{alt}(\mathbf{p}_a, \mathbf{p}_b) = \Delta_{xy} + w_{z} \cdot |\Delta z|$$

$$\sigma_{k+1} = \arg\min_{j \notin \{\sigma_1,\ldots,\sigma_k\}} d_{alt}(\mathbf{p}_{\sigma_k}, \mathbf{p}_j)$$

where $w_z = v_{xy} / v_{z,max}$ normalises the altitude penalty to equivalent horizontal distance.

### Altitude Strategy

The pursuer maintains horizontal speed $v_{xy}$ and adjusts z at rate $\dot{z}$:

$$\dot{\mathbf{p}}_P = v_{xy} \cdot \frac{(\mathbf{p}_{target} - \mathbf{p}_P)_{xy}}{\|(\mathbf{p}_{target} - \mathbf{p}_P)_{xy}\|} + \begin{bmatrix}0 \\ 0 \\ \text{clip}(z_{target} - z_P, -v_{z,max}\Delta t, v_{z,max}\Delta t) / \Delta t\end{bmatrix}$$

When the pursuer reaches the target's XY position, it hovers and waits for altitude to converge:

$$\text{arrival condition:} \quad \Delta_{xy} < \epsilon_{xy} \quad \text{AND} \quad |\Delta z| < \epsilon_z$$

---

## Key 3D Additions

- **Altitude strategy**: pursuer uses independent horizontal/vertical speed limits; altitude-aware travel time metric replaces Euclidean distance
- **3D guidance law**: pursuer decouples XY and Z motion; altitude converges to target z during approach
- **Vertical evasion / geometry**: targets at four different altitudes; altitude-monotone orderings identified; altitude-aware NN heuristic added as third algorithm

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Pursuer start | (−5, 0, 2.5) m |
| Target 1 | (2, 3, 1.0) m (low) |
| Target 2 | (4, −2, 4.5) m (high) |
| Target 3 | (−1, −3, 0.8) m (very low) |
| Target 4 | (3, 1, 3.5) m (mid-high) |
| Horizontal speed v_xy | 5 m/s |
| Vertical speed v_z,max | 2.5 m/s |
| Altitude weight w_z | v_xy / v_z = 2.0 |
| XY arrival threshold ε_xy | 0.2 m |
| Z arrival threshold ε_z | 0.1 m |
| z range | 0.5 – 5.5 m |

---

## Implementation

```python
import numpy as np
from itertools import permutations
from numpy.linalg import norm

V_XY    = 5.0
V_Z_MAX = 2.5
W_Z     = V_XY / V_Z_MAX   # altitude penalty weight
EPS_XY  = 0.2
EPS_Z   = 0.1
DT      = 0.05

TARGETS = np.array([
    [2.0,  3.0, 1.0],
    [4.0, -2.0, 4.5],
    [-1.0,-3.0, 0.8],
    [3.0,  1.0, 3.5],
], dtype=float)
START = np.array([-5.0, 0.0, 2.5])

def travel_time_3d(a, b, vxy=V_XY, vz=V_Z_MAX):
    """Altitude-aware travel time between two 3D points."""
    dxy = norm(b[:2] - a[:2])
    dz  = abs(b[2] - a[2])
    return max(dxy / vxy, dz / vz)

def tour_time_3d(order, start, targets):
    pts = [start] + [targets[i] for i in order]
    return sum(travel_time_3d(pts[k], pts[k+1]) for k in range(len(pts)-1))

def dist_alt_aware(a, b, w_z=W_Z):
    dxy = norm(b[:2] - a[:2])
    dz  = abs(b[2] - a[2])
    return dxy + w_z * dz

# Brute-force optimal (3D travel time)
best_time, best_order = min(
    (tour_time_3d(perm, START, TARGETS), perm)
    for perm in permutations(range(len(TARGETS)))
)

# Standard nearest-neighbour (3D Euclidean)
def nn_3d_standard(start, targets):
    remaining = list(range(len(targets)))
    pos = start.copy()
    order = []
    while remaining:
        nearest = min(remaining, key=lambda i: norm(targets[i] - pos))
        order.append(nearest)
        pos = targets[nearest]
        remaining.remove(nearest)
    return order

# Altitude-aware nearest-neighbour
def nn_3d_altitude_aware(start, targets):
    remaining = list(range(len(targets)))
    pos = start.copy()
    order = []
    while remaining:
        nearest = min(remaining,
                      key=lambda i: dist_alt_aware(pos, targets[i]))
        order.append(nearest)
        pos = targets[nearest]
        remaining.remove(nearest)
    return order

nn_std_order = nn_3d_standard(START, TARGETS)
nn_alt_order = nn_3d_altitude_aware(START, TARGETS)

print(f"Optimal order: {best_order}, time: {best_time:.2f}s")
print(f"NN standard:   {nn_std_order}, time: {tour_time_3d(nn_std_order, START, TARGETS):.2f}s")
print(f"NN alt-aware:  {nn_alt_order}, time: {tour_time_3d(nn_alt_order, START, TARGETS):.2f}s")

# Optimality gap
t_nn_std = tour_time_3d(nn_std_order, START, TARGETS)
t_nn_alt = tour_time_3d(nn_alt_order, START, TARGETS)
gap_std = (t_nn_std - best_time) / best_time * 100
gap_alt = (t_nn_alt - best_time) / best_time * 100
print(f"NN standard gap: {gap_std:.1f}%")
print(f"NN alt-aware gap: {gap_alt:.1f}%")

def simulate_tour_3d(order, start, targets, vxy=V_XY, vz=V_Z_MAX, dt=DT):
    """Simulate pursuer following a 3D tour."""
    pos = start.copy()
    traj = [pos.copy()]
    z_history = [pos[2]]
    visit_times = []

    for target_idx in order:
        target = targets[target_idx].copy()
        for step in range(int(20 / dt)):
            dxy_vec = target[:2] - pos[:2]
            dxy = norm(dxy_vec)
            dz  = target[2] - pos[2]

            # XY motion
            if dxy > EPS_XY:
                pos[:2] += vxy * dt * dxy_vec / (dxy + 1e-8)
            # Z motion (limited rate)
            dz_step = np.clip(dz, -vz * dt, vz * dt)
            pos[2] += dz_step

            traj.append(pos.copy())
            z_history.append(pos[2])

            if dxy < EPS_XY and abs(dz) < EPS_Z:
                visit_times.append(len(traj) * dt)
                break

    return np.array(traj), np.array(z_history), visit_times
```

---

## Expected Output

- 3D trajectory for optimal, NN standard, and NN altitude-aware orderings
- Altitude time series $z_P(t)$ for each ordering — shows climb/descent profile
- Bar chart: total tour time for all 24 orderings (highlight optimal, two NN variants)
- Optimality gap vs $v_{z,max}$ sweep: how much does altitude cost matter as a function of vertical speed limit?
- Visit order comparison table with 3D travel times and z-change per leg

---

## Extensions

1. Moving targets at different altitudes — altitude-aware greedy NN still applicable
2. 2-opt 3D local search: swap two visits and check if altitude-aware tour time improves
3. Heterogeneous vertical speeds: climbing slower than descending (gravity-assisted)

---

## Related Scenarios

- Original 2D version: [S018](../S018_multi_target_intercept.md)
- Truly 3D references: [S001](../S001_basic_intercept.md), [S003](../S003_low_altitude_tracking.md)
