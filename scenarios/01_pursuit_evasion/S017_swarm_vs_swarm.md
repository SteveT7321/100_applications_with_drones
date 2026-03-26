# S017 Swarm vs Swarm

**Domain**: Pursuit & Evasion | **Difficulty**: ⭐⭐⭐⭐ | **Status**: `[ ]` Not started

---

## Problem Definition

**Setup**: M pursuers compete against N evaders in an open arena. Two cases are studied: 3v3 (symmetric) and 5v3 (pursuer advantage). Each pursuer is assigned one evader via the Hungarian algorithm. When an evader is captured, its pursuer dynamically reassigns to the nearest remaining evader.

**Roles**:
- **Pursuers**: use pure pursuit toward their assigned evader; reassign on capture
- **Evaders**: scatter away from the centroid of all pursuers

**Objective**: compare 3v3 vs 5v3 total capture times; compare greedy (nearest-neighbor) assignment vs Hungarian algorithm optimal assignment.

---

## Mathematical Model

### Assignment Problem (Hungarian Algorithm)

Build a cost matrix where each entry is the Euclidean distance from pursuer i to evader j:

$$C_{ij} = \|\mathbf{p}_{P_i} - \mathbf{p}_{E_j}\|$$

Find the optimal assignment that minimizes total cost:

$$\sigma^* = \arg\min_{\sigma} \sum_{i=1}^{M} C_{i,\sigma(i)}$$

For M > N, solve a balanced assignment by padding the cost matrix with dummy columns.

### Pursuer Strategy (Pure Pursuit)

Each pursuer flies toward its assigned evader:

$$\mathbf{v}_{P_i} = v_P \cdot \frac{\mathbf{p}_{E_{\sigma(i)}} - \mathbf{p}_{P_i}}{\|\mathbf{p}_{E_{\sigma(i)}} - \mathbf{p}_{P_i}\|}$$

### Evader Scatter Strategy

Each evader flies directly away from the centroid of all pursuers:

$$\bar{\mathbf{p}}_P = \frac{1}{M} \sum_{i=1}^{M} \mathbf{p}_{P_i}$$

$$\mathbf{v}_{E_j} = v_E \cdot \frac{\mathbf{p}_{E_j} - \bar{\mathbf{p}}_P}{\|\mathbf{p}_{E_j} - \bar{\mathbf{p}}_P\|}$$

### Capture and Reassignment

Evader j is captured when:

$$\|\mathbf{p}_{P_i} - \mathbf{p}_{E_j}\| < r_{capture}$$

After capture, pursuer i reassigns to the nearest uncaptured evader:

$$j^* = \arg\min_{j \in \mathcal{E}_{remaining}} \|\mathbf{p}_{P_i} - \mathbf{p}_{E_j}\|$$

### Total Mission Time

$$T_{mission} = \max_{j} t_{capture,j}$$

---

## Implementation

```python
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import linear_sum_assignment

# --- Parameters ---
V_PURSUER = 5.0    # m/s
V_EVADER = 3.5     # m/s
CAPTURE_R = 0.15   # m
DT = 0.02          # s
T_MAX = 30.0

# Case A: 3v3
PURSUERS_3 = np.array([[-5, -2, 2], [-5, 0, 2], [-5, 2, 2]], dtype=float)
EVADERS_3  = np.array([[ 0, -2, 2], [ 0, 0, 2], [ 0, 2, 2]], dtype=float)

# Case B: 5v3
PURSUERS_5 = np.array([[-5, -3, 2], [-5, -1, 2], [-5, 1, 2],
                        [-5, 3, 2], [-5, 0, 2]], dtype=float)

def build_cost_matrix(pursuers, evaders):
    M, N = len(pursuers), len(evaders)
    C = np.zeros((M, N))
    for i in range(M):
        for j in range(N):
            C[i, j] = np.linalg.norm(pursuers[i] - evaders[j])
    return C

def hungarian_assignment(pursuers, evaders):
    C = build_cost_matrix(pursuers, evaders)
    row_ind, col_ind = linear_sum_assignment(C)
    # Map pursuer index -> evader index (handle M > N via modulo)
    assignment = {}
    for r, c in zip(row_ind, col_ind):
        assignment[r] = c
    return assignment

def evader_scatter(evader_pos, pursuer_positions):
    centroid = pursuer_positions.mean(axis=0)
    direction = evader_pos - centroid
    norm = np.linalg.norm(direction) + 1e-8
    return evader_pos + (direction / norm) * V_EVADER * DT

def simulate_swarm(pursuers_init, evaders_init, assignment_method="hungarian"):
    pursuers = pursuers_init.copy()
    evaders  = evaders_init.copy()
    M, N = len(pursuers), len(evaders)
    captured = [False] * N
    capture_times = [None] * N

    if assignment_method == "hungarian":
        assignment = hungarian_assignment(pursuers, evaders)
    else:
        # Greedy: each pursuer takes nearest evader
        assignment = {}
        taken = set()
        for i in range(min(M, N)):
            dists = [np.linalg.norm(pursuers[i] - evaders[j])
                     if j not in taken else np.inf for j in range(N)]
            j_best = int(np.argmin(dists))
            assignment[i] = j_best
            taken.add(j_best)
        for i in range(N, M):
            assignment[i] = list(taken)[0]  # overflow pursuers

    for step in range(int(T_MAX / DT)):
        t = step * DT
        active_pursuers = pursuers[~np.array([captured[assignment.get(i, 0)]
                                               for i in range(M)])]

        # Move evaders
        for j in range(N):
            if not captured[j]:
                evaders[j] = evader_scatter(evaders[j], pursuers)

        # Move pursuers and check capture
        for i in range(M):
            target_j = assignment.get(i)
            if target_j is None or captured[target_j]:
                # Reassign to nearest uncaptured
                remaining = [j for j in range(N) if not captured[j]]
                if not remaining:
                    break
                dists = [np.linalg.norm(pursuers[i] - evaders[j])
                         for j in remaining]
                target_j = remaining[int(np.argmin(dists))]
                assignment[i] = target_j

            direction = evaders[target_j] - pursuers[i]
            pursuers[i] += direction / (np.linalg.norm(direction) + 1e-8) * V_PURSUER * DT

            if np.linalg.norm(pursuers[i] - evaders[target_j]) < CAPTURE_R:
                captured[target_j] = True
                capture_times[target_j] = t

        if all(captured):
            break

    return capture_times, max(t for t in capture_times if t is not None)

# Run both cases and both assignment methods
for n_pursuers, p_init in [(3, PURSUERS_3), (5, PURSUERS_5)]:
    for method in ["hungarian", "greedy"]:
        times, total = simulate_swarm(p_init, EVADERS_3.copy(), method)
        print(f"{n_pursuers}v3 | {method}: capture times={times}, total={total:.2f}s")
```

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Case A | 3 pursuers vs 3 evaders |
| Case B | 5 pursuers vs 3 evaders |
| Pursuer speed | 5 m/s |
| Evader speed | 3.5 m/s |
| Pursuer initial positions | line at x = -5 m |
| Evader initial positions | cluster at x = 0 m |
| Capture radius | 0.15 m |
| Simulation timestep | 0.02 s |

---

## Expected Output

- **2D top-down trajectories**: color-coded pursuer and evader paths for both cases
- **Gantt chart**: capture time for each evader (horizontal bars per evader)
- **Total mission time comparison**: bar chart with 4 bars (3v3 greedy, 3v3 Hungarian, 5v3 greedy, 5v3 Hungarian)
- **Assignment matrix visualization**: initial assignment bipartite graph at t = 0

---

## Extensions

1. Cooperative pursuit — 2 pursuers per evader using flanking maneuvers
2. Evaders also use APF to avoid each other while scattering
3. Heterogeneous speeds — some evaders are faster than the pursuers

---

## Related Scenarios

- Prerequisites: [S011](S011_swarm_encirclement.md), [S013](S013_pincer_movement.md)
- Next: [S018](S018_multi_target_intercept.md), [S019](S019_dynamic_reassignment.md)
- See [domains/01_pursuit_evasion.md](../../domains/01_pursuit_evasion.md)
