# S018 Multi-Target Intercept

**Domain**: Pursuit & Evasion | **Difficulty**: ⭐⭐⭐ | **Status**: `[x]` Complete

---

## Problem Definition

**Setup**: A single pursuer must intercept $N = 4$ stationary targets in minimum total time. The visit order matters — this is the Travelling Salesman Problem (TSP) with an initial position.

**Objective**: find the visit ordering that minimises total flight time, and compare:
1. **Nearest-neighbour heuristic** — always fly to the closest remaining target
2. **Brute-force optimal** — try all $4! = 24$ orderings ($N!$ search)
3. **Random ordering** — baseline

---

## Mathematical Model

### Total Mission Time

For visit ordering $\sigma = (\sigma_1, \sigma_2, \sigma_3, \sigma_4)$:

$$T_{total}(\sigma) = \sum_{k=1}^{N} \frac{\|\mathbf{p}_{\sigma_k} - \mathbf{p}_{\sigma_{k-1}}\|}{v_P}$$

where $\mathbf{p}_{\sigma_0} = \mathbf{p}_P(0)$ (pursuer start) and $\mathbf{p}_{\sigma_k}$ are target positions.

### Optimal Ordering

$$\sigma^* = \arg\min_{\sigma \in S_N} T_{total}(\sigma)$$

### Nearest-Neighbour Heuristic

$$\sigma_{k+1} = \arg\min_{j \notin \{\sigma_1,\ldots,\sigma_k\}} \|\mathbf{p}_j - \mathbf{p}_{\sigma_k}\|$$

### Optimality Gap

$$\text{gap} = \frac{T_{NN} - T^*}{T^*} \times 100\%$$

---

## Implementation

```python
from itertools import permutations

TARGETS = np.array([
    [2., 3., 2.], [4., -2., 2.],
    [-1., -3., 2.], [3., 1., 2.]
])
START = np.array([-5., 0., 2.])

def tour_time(order, start, targets, v):
    pts = [start] + [targets[i] for i in order]
    return sum(np.linalg.norm(pts[k+1]-pts[k]) for k in range(len(pts)-1)) / v

# Brute force
best_t, best_order = min(
    (tour_time(perm, START, TARGETS, V_PURSUER), perm)
    for perm in permutations(range(len(TARGETS)))
)

# Nearest-neighbour
remaining, pos, nn_order = list(range(len(TARGETS))), START.copy(), []
while remaining:
    nearest = min(remaining, key=lambda i: np.linalg.norm(TARGETS[i]-pos))
    nn_order.append(nearest); pos = TARGETS[nearest]; remaining.remove(nearest)
```

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Pursuer start | (-5, 0, 2) m |
| Target 1 | (2, 3, 2) m |
| Target 2 | (4, -2, 2) m |
| Target 3 | (-1, -3, 2) m |
| Target 4 | (3, 1, 2) m |
| Pursuer speed | 5 m/s |
| Capture radius | 0.15 m per target |

---

## Expected Output

- 3D trajectory for optimal, nearest-neighbour, and worst-case orderings
- Bar chart: total mission time for all 24 orderings (highlight optimal, NN, random)
- Visit order comparison table with total distances

---

## Extensions

1. Moving targets — greedy NN still applicable; optimal becomes intractable
2. $N = 10$ targets: NN vs 2-opt local search vs optimal (timing comparison)
3. Return-to-base variant: pursuer must end at start position (classical TSP)

---

## Related Scenarios

- Prerequisites: [S001](S001_basic_intercept.md), [S011](S011_swarm_encirclement.md)
- Next: [S019](S019_dynamic_reassignment.md)
