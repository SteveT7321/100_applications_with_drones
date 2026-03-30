# S017 3D Upgrade — Swarm vs Swarm

**Domain**: Pursuit & Evasion | **Difficulty**: ⭐⭐⭐⭐ | **Status**: `[x]` Complete
**Based on**: [S017 original](../S017_swarm_vs_swarm.md)

---

## What Changes in 3D

The original is **purely 2D**: all `[x, y]` position arrays, 2D top-down plots, and a 2D Euclidean cost matrix. In 3D, drones operate at **different altitude layers** — pursuers and evaders are initialised at distinct z-tiers, the Hungarian cost matrix uses full 3D Euclidean distance, and pursuit strategies account for the z-axis. After a capture, reassigned pursuers must account for altitude cost when switching targets. Formation splitting across altitudes creates new tactical options: a pursuer at high altitude can exploit a dive-speed advantage against a low-altitude evader.

---

## Problem Definition

**Setup**: M pursuers vs N evaders in a 10 × 10 × 5 m volume. Drones are assigned to three altitude layers: low (z ≈ 1 m), mid (z ≈ 2.5 m), high (z ≈ 4 m). Two cases:
- **Case A (3v3)**: pursuers at three altitude layers; evaders at three altitude layers
- **Case B (5v3)**: five pursuers spanning all altitude tiers; three evaders at mid altitude

**Objective**: compare 3D Hungarian assignment vs 3D greedy assignment; measure how altitude diversity affects total mission time vs flat (all-same-z) deployment.

---

## Mathematical Model

### 3D Cost Matrix

Build cost matrix with full Euclidean distance:

$$C_{ij} = \|\mathbf{p}_{P_i} - \mathbf{p}_{E_j}\|_2 = \sqrt{(x_{P_i}-x_{E_j})^2 + (y_{P_i}-y_{E_j})^2 + (z_{P_i}-z_{E_j})^2}$$

For M > N, pad the cost matrix with dummy columns of large cost.

### Hungarian Assignment (unchanged structure)

$$\sigma^* = \arg\min_{\sigma \in S_{\min(M,N)}} \sum_{i=1}^{\min(M,N)} C_{i,\sigma(i)}$$

### 3D Pursuer Strategy (Pure Pursuit)

$$\mathbf{v}_{P_i} = v_P \cdot \frac{\mathbf{p}_{E_{\sigma(i)}} - \mathbf{p}_{P_i}}{\|\mathbf{p}_{E_{\sigma(i)}} - \mathbf{p}_{P_i}\|}$$

Full 3D vector — the z-component drives altitude change.

### Dive-Advantage Boost

When a pursuer at altitude $z_P > z_E + \Delta z_{boost}$ is assigned to an evader below, it gains a speed boost through a shallow dive:

$$v_{boost} = v_P \cdot \left(1 + \eta \cdot \frac{z_P - z_E}{\|\mathbf{p}_E - \mathbf{p}_P\|}\right)$$

where $\eta = 0.15$ is the dive efficiency factor. This is capped at $1.3 v_P$.

### 3D Evader Scatter Strategy

Each evader flies away from the 3D centroid of all pursuers, with an additional random vertical component:

$$\bar{\mathbf{p}}_P = \frac{1}{M} \sum_{i=1}^{M} \mathbf{p}_{P_i}$$

$$\mathbf{v}_{E_j}^{base} = v_E \cdot \frac{\mathbf{p}_{E_j} - \bar{\mathbf{p}}_P}{\|\mathbf{p}_{E_j} - \bar{\mathbf{p}}_P\|}$$

Vertical escape layer: evader $j$ also biases toward the altitude layer least populated by pursuers:

$$z_j^{escape} = \arg\max_{z \in \{z_{low}, z_{mid}, z_{high}\}} \min_{i} |z_{P_i} - z|$$

$$\mathbf{v}_{E_j} = \mathbf{v}_{E_j}^{base} + [0, 0, k_z(z_j^{escape} - z_{E_j})]^T$$

(renormalised to $v_E$ after adding z-component)

### Altitude Layer Formation Splitting

Pursuers are assigned to altitude layers to maintain vertical coverage. After each capture, the reassignment considers not only Euclidean distance but also altitude-layer balance:

$$C_{ij}^{balanced} = C_{ij} + \lambda_{layer} \cdot \mathbb{1}[\text{layer}(P_i) = \text{layer}(P_k) \text{ for some other active pursuer } k]$$

where $\lambda_{layer} = 2.0$ m penalises two pursuers occupying the same altitude layer.

### Capture and Reassignment (3D)

Evader $j$ captured when:

$$\|\mathbf{p}_{P_i} - \mathbf{p}_{E_j}\| < r_{capture}$$

After capture, pursuer $i$ reassigns to nearest uncaptured evader using 3D distance:

$$j^* = \arg\min_{j \in \mathcal{E}_{remaining}} \|\mathbf{p}_{P_i} - \mathbf{p}_{E_j}\|_2$$

---

## Key 3D Additions

- **Altitude strategy**: three altitude layers (z = 1.0, 2.5, 4.0 m); layer-balance penalty in reassignment cost; dive-advantage speed boost
- **3D guidance law**: 3D Euclidean cost matrix for Hungarian algorithm; full 3D pure pursuit velocity vector
- **Vertical evasion / geometry**: evaders scatter toward least-covered altitude layer; vertical escape bias $k_z$

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Case A | 3v3 |
| Case B | 5v3 |
| Altitude layers | z = 1.0, 2.5, 4.0 m |
| Pursuer speed | 5 m/s |
| Dive boost max | 1.3 × v_P |
| Evader speed | 3.5 m/s |
| Vertical escape gain k_z | 0.5 |
| Layer balance penalty λ_layer | 2.0 m |
| Capture radius | 0.15 m |
| Dive efficiency η | 0.15 |
| z range | 0.5 – 5.5 m |
| dt | 0.02 s |

---

## Implementation

```python
import numpy as np
from numpy.linalg import norm
from scipy.optimize import linear_sum_assignment

V_PURSUER = 5.0
V_EVADER  = 3.5
CAPTURE_R = 0.15
DT = 0.02
T_MAX = 30.0
ETA_DIVE = 0.15
K_Z_ESCAPE = 0.5
LAMBDA_LAYER = 2.0
Z_LAYERS = [1.0, 2.5, 4.0]
Z_MIN, Z_MAX = 0.5, 5.5

# Case A: 3v3, one pursuer per altitude layer
PURSUERS_3 = np.array([
    [-5, -2, Z_LAYERS[0]],
    [-5,  0, Z_LAYERS[1]],
    [-5,  2, Z_LAYERS[2]],
], dtype=float)
EVADERS_3 = np.array([
    [0, -2, Z_LAYERS[2]],  # evader at high layer
    [0,  0, Z_LAYERS[1]],  # evader at mid layer
    [0,  2, Z_LAYERS[0]],  # evader at low layer
], dtype=float)

# Case B: 5v3
PURSUERS_5 = np.array([
    [-5, -3, Z_LAYERS[0]],
    [-5, -1, Z_LAYERS[1]],
    [-5,  1, Z_LAYERS[2]],
    [-5,  3, Z_LAYERS[1]],
    [-5,  0, Z_LAYERS[0]],
], dtype=float)

def dive_speed(pos_p, pos_e, v_max, eta=ETA_DIVE):
    dz = pos_p[2] - pos_e[2]
    dist = norm(pos_e - pos_p) + 1e-8
    if dz > 0.5:
        boost = 1 + eta * dz / dist
        return min(v_max * boost, 1.3 * v_max)
    return v_max

def build_cost_matrix_3d(pursuers, evaders, assignment, layer_balance=True):
    M, N = len(pursuers), len(evaders)
    C = np.zeros((M, N))
    for i in range(M):
        for j in range(N):
            C[i, j] = norm(pursuers[i] - evaders[j])  # 3D Euclidean
    if layer_balance:
        # Penalise assigning two pursuers to same altitude layer
        for i in range(M):
            for j in range(N):
                for k in range(M):
                    if k != i and assignment.get(k) == j:
                        continue
                    # Check if another pursuer is already at same layer
                    for k2 in range(M):
                        if k2 != i and abs(pursuers[i, 2] - pursuers[k2, 2]) < 0.5:
                            C[i, j] += LAMBDA_LAYER
                            break
    return C

def evader_escape_3d(pos_e, pos_pursuers, v_max):
    centroid = pos_pursuers.mean(axis=0)
    away = pos_e - centroid
    # Find least-covered altitude layer
    layer_coverage = [sum(abs(p[2] - z) < 1.0 for p in pos_pursuers)
                      for z in Z_LAYERS]
    target_layer = Z_LAYERS[int(np.argmin(layer_coverage))]
    dz = target_layer - pos_e[2]
    vel = away + np.array([0, 0, K_Z_ESCAPE * dz])
    return v_max * vel / (norm(vel) + 1e-8)

def simulate_swarm_3d(pursuers_init, evaders_init, method="hungarian"):
    pursuers = pursuers_init.copy()
    evaders  = evaders_init.copy()
    M, N = len(pursuers), len(evaders)
    captured = [False] * N
    capture_times = [None] * N
    assignment = {}

    # Initial assignment
    C = np.array([[norm(pursuers[i] - evaders[j]) for j in range(N)]
                   for i in range(M)])
    if method == "hungarian":
        rows, cols = linear_sum_assignment(C[:min(M,N), :])
        assignment = dict(zip(rows.tolist(), cols.tolist()))
        # Extra pursuers: nearest available
        for i in range(N, M):
            remaining = [j for j in range(N) if j not in assignment.values()]
            if not remaining:
                remaining = list(range(N))
            j_best = min(remaining, key=lambda j: C[i, j])
            assignment[i] = j_best
    else:
        taken = set()
        for i in range(min(M, N)):
            dists = {j: C[i,j] for j in range(N) if j not in taken}
            j_best = min(dists, key=dists.get)
            assignment[i] = j_best; taken.add(j_best)
        for i in range(N, M):
            assignment[i] = list(taken)[0]

    traj_p = [pursuers.copy()]
    traj_e = [evaders.copy()]

    for step in range(int(T_MAX / DT)):
        t = step * DT
        active_pos = pursuers[~np.array([False]*M)]

        for j in range(N):
            if not captured[j]:
                evaders[j] = evaders[j] + evader_escape_3d(evaders[j], pursuers, V_EVADER) * DT
                evaders[j, 2] = np.clip(evaders[j, 2], Z_MIN, Z_MAX)

        for i in range(M):
            target_j = assignment.get(i)
            if target_j is None or captured[target_j]:
                remaining = [j for j in range(N) if not captured[j]]
                if not remaining:
                    break
                target_j = min(remaining,
                               key=lambda j: norm(pursuers[i] - evaders[j]))
                assignment[i] = target_j

            v_cmd = dive_speed(pursuers[i], evaders[target_j], V_PURSUER)
            direction = evaders[target_j] - pursuers[i]
            pursuers[i] += v_cmd * DT * direction / (norm(direction) + 1e-8)
            pursuers[i, 2] = np.clip(pursuers[i, 2], Z_MIN, Z_MAX)

            if norm(pursuers[i] - evaders[target_j]) < CAPTURE_R:
                captured[target_j] = True
                capture_times[target_j] = t

        traj_p.append(pursuers.copy())
        traj_e.append(evaders.copy())

        if all(captured):
            break

    total_time = max(t for t in capture_times if t is not None)
    return (np.array(traj_p), np.array(traj_e),
            capture_times, total_time)
```

---

## Expected Output

- 3D trajectory plots with altitude variation: pursuers and evaders colour-coded, z clearly visible
- Altitude time series: all drones' z(t) showing layer-switching and dive events
- Gantt chart of capture times per evader (Case A and B)
- Total mission time bar chart: 3v3 greedy, 3v3 Hungarian, 5v3 greedy, 5v3 Hungarian (3D vs 2D versions)
- Layer coverage analysis: number of pursuers per altitude layer over time

---

## Extensions

1. Cooperative altitude stacking: each pursuer is locked to its altitude layer, forcing vertical coordination
2. Heterogeneous altitude speeds: climbing is slower than level flight (different z vs xy max speeds)
3. MARL (MAPPO) end-to-end training for 3D swarm vs swarm — compare to rule-based 3D Hungarian

---

## Related Scenarios

- Original 2D version: [S017](../S017_swarm_vs_swarm.md)
- Truly 3D references: [S001](../S001_basic_intercept.md), [S003](../S003_low_altitude_tracking.md)
