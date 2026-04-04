# S034 Weather Rerouting

**Domain**: Logistics & Delivery | **Difficulty**: ⭐⭐⭐ | **Status**: `[x]` Complete

---

## Problem Definition

**Setup**: A single delivery drone flies a planned route through a 3D urban airspace at 20 m altitude. During flight, a weather sensor network detects wind-hazard cells that appear and dissipate stochastically. When a hazard cell intersects the planned path, the drone must re-plan in real time using D* Lite, finding a new minimum-cost 3D route that avoids active cells and still reaches the delivery waypoint.

**Roles**:
- **Drone**: navigates from depot $\mathbf{p}_{start}$ to delivery point $\mathbf{p}_{goal}$ while tracking an edge-cost map that changes as hazard cells activate or expire
- **Weather model**: generates spatially localised wind-hazard volumes. Each cell $c_k$ has a severity score $w_k \in [0, 1]$, an activation time $t_{on,k}$, and a lifetime $\tau_k \sim \text{Uniform}(30, 120)$ s

**Objective**: minimise total flight distance subject to the constraint that the traversal cost through any active hazard cell exceeds the avoidance cost, producing a path that dynamically circumnavigates storms while minimising detour length.

---

## Mathematical Model

### 3D Grid Graph

The airspace is discretised into a 3D voxel grid with resolution $\Delta = 2$ m. Each voxel is a node $v_{i,j,k}$ at position $(i\Delta,\; j\Delta,\; k\Delta)$. Edges connect 26-adjacent neighbours (face, edge, and corner adjacency in 3D). The base edge cost between adjacent voxels $u$ and $v$ is:

$$c_{base}(u,v) = \|\mathbf{p}_u - \mathbf{p}_v\|_2$$

When one or more hazard cells are active, the traversal cost is inflated:

$$c(u,v,t) = c_{base}(u,v) \cdot \left(1 + \alpha \sum_{k:\; v \in \mathcal{H}_k(t)} w_k\right)$$

where $\mathcal{H}_k(t)$ is the set of voxels occupied by hazard cell $k$ at time $t$ and $\alpha = 10$ is the hazard inflation factor. When the inflation is large enough, the planner routes around the cell entirely.

### Hazard Cell Model

Each hazard cell is a sphere of radius $r_k \sim \text{Uniform}(4, 12)$ m centred at $\mathbf{c}_k$:

$$\mathcal{H}_k(t) = \left\{ v \;:\; \|\mathbf{p}_v - \mathbf{c}_k\| \leq r_k \right\}, \quad t_{on,k} \leq t < t_{on,k} + \tau_k$$

The active set at time $t$ is $\mathcal{A}(t) = \{k : t_{on,k} \leq t < t_{on,k} + \tau_k\}$. Edge costs change only for voxels adjacent to or inside a hazard volume; all other edges are unaffected.

### D* Lite Incremental Replanning

D* Lite maintains two estimates for each node $s$:

$$g(s) = \text{current best cost-to-come from } s_{start}$$
$$rhs(s) = \min_{s' \in \text{Pred}(s)} \left[ c(s', s) + g(s') \right] \quad \text{(one-step lookahead)}$$

A node is **locally consistent** when $g(s) = rhs(s)$; inconsistent nodes are inserted into a priority queue $U$ with key:

$$\mathbf{k}(s) = \left[\min(g(s),\; rhs(s)) + h(s_{start}, s) + k_m;\;\; \min(g(s),\; rhs(s))\right]$$

where $h$ is the Euclidean heuristic and $k_m$ accumulates heuristic offset as the start moves. When hazard cells change, only the edges whose costs changed are updated, and only the nodes made locally inconsistent by those changes are re-expanded — making replanning far cheaper than full A*.

**Cost update procedure** (called on each weather event):

$$\text{for each changed edge } (u,v): \quad \text{UpdateVertex}(v)$$

$$\text{UpdateVertex}(s):\; rhs(s) = \min_{s' \in \text{Pred}(s)} [c(s',s) + g(s')]; \;\; \text{reinsert } s \text{ in } U \text{ if inconsistent}$$

### Drone Kinematics

Point-mass model with first-order speed control:

$$\dot{\mathbf{p}} = \mathbf{v}, \quad \mathbf{v}_{cmd} = v_{max} \cdot \frac{\mathbf{p}_{next} - \mathbf{p}}{\|\mathbf{p}_{next} - \mathbf{p}\|}$$

$$v_{max} = 5 \text{ m/s}, \quad \Delta t = 0.05 \text{ s}$$

The drone follows waypoints on the current D* Lite path. When a re-plan event fires, the new path replaces the queue of remaining waypoints from the drone's current position onward.

### Replanning Trigger Conditions

A replanning event is triggered when:

1. A newly activated hazard cell intersects the current planned path:
$$\exists\; k \in \mathcal{A}(t) : \mathcal{H}_k(t) \cap \Pi_{remaining}(t) \neq \emptyset$$

2. A previously active cell expires and may open a shorter route:
$$\exists\; k \notin \mathcal{A}(t) : \mathcal{H}_k(t^-) \cap \Pi_{remaining}(t^-) \neq \emptyset$$

where $\Pi_{remaining}(t)$ denotes the set of voxels on the planned path not yet traversed.

### Path Cost Metrics

Total mission cost decomposed into:

$$J_{total} = J_{base} + J_{detour} + J_{hazard}$$

$$J_{base} = \|\mathbf{p}_{goal} - \mathbf{p}_{start}\|_2 \quad \text{(straight-line lower bound)}$$

$$J_{detour} = L_{actual} - J_{base} \quad \text{(extra distance from rerouting)}$$

$$J_{hazard} = \int_0^T \mathbf{1}[\mathbf{p}(t) \in \bigcup_k \mathcal{H}_k(t)]\, dt \quad \text{(time spent inside hazard, target = 0)}$$

---

## Implementation

```python
# Key constants
GRID_RES      = 2.0     # m — voxel side length
GRID_SIZE     = (50, 50, 15)  # voxels (100 m × 100 m × 30 m airspace)
V_MAX         = 5.0     # m/s
DT            = 0.05    # s
ALPHA         = 10.0    # hazard inflation factor
N_HAZARDS     = 8       # total hazard cells over mission lifetime
DELIVERY_ALT  = 20.0    # m nominal cruise altitude

# Hazard cell dataclass
from dataclasses import dataclass
import numpy as np

@dataclass
class HazardCell:
    center: np.ndarray   # shape (3,) in metres
    radius: float        # metres
    severity: float      # w_k in [0, 1]
    t_on:  float         # activation time (s)
    t_off: float         # expiry time (s)

def is_active(cell: HazardCell, t: float) -> bool:
    return cell.t_on <= t < cell.t_off

# D* Lite core structures
import heapq

class DStarLite:
    def __init__(self, grid_shape, start_vox, goal_vox):
        self.g   = {}       # cost-to-come; default inf
        self.rhs = {}       # one-step lookahead; default inf
        self.U   = []       # priority queue (min-heap)
        self.km  = 0.0      # accumulated heuristic offset
        self.start = start_vox
        self.goal  = goal_vox
        self.rhs[goal_vox] = 0.0
        heapq.heappush(self.U, (self._key(goal_vox), goal_vox))

    def _h(self, a, b):
        # Euclidean heuristic in voxel coordinates
        return np.linalg.norm(np.array(a) - np.array(b)) * GRID_RES

    def _key(self, s):
        g_s   = self.g.get(s, np.inf)
        rhs_s = self.rhs.get(s, np.inf)
        return (min(g_s, rhs_s) + self._h(self.start, s) + self.km,
                min(g_s, rhs_s))

    def update_vertex(self, s, cost_map):
        if s != self.goal:
            self.rhs[s] = min(
                cost_map.get((s, nb), np.inf) + self.g.get(nb, np.inf)
                for nb in neighbours26(s)
            )
        # Remove stale entry; re-insert if inconsistent
        if self.g.get(s, np.inf) != self.rhs.get(s, np.inf):
            heapq.heappush(self.U, (self._key(s), s))

    def compute_shortest_path(self, cost_map):
        while self.U and (
            self.U[0][0] < self._key(self.start) or
            self.rhs.get(self.start, np.inf) != self.g.get(self.start, np.inf)
        ):
            _, u = heapq.heappop(self.U)
            if self.g.get(u, np.inf) > self.rhs.get(u, np.inf):
                self.g[u] = self.rhs[u]
            else:
                self.g[u] = np.inf
                self.update_vertex(u, cost_map)
            for nb in neighbours26(u):
                self.update_vertex(nb, cost_map)

# Main simulation loop (pseudocode)
def run_simulation():
    planner = DStarLite(GRID_SIZE, world_to_vox(p_start), world_to_vox(p_goal))
    cost_map = build_base_cost_map()          # static edge costs
    planner.compute_shortest_path(cost_map)
    path = extract_path(planner)

    t = 0.0
    drone_pos = p_start.copy()
    replan_count = 0

    while np.linalg.norm(drone_pos - p_goal) > LANDING_THRESHOLD:
        active = [c for c in hazard_cells if is_active(c, t)]
        changed = update_cost_map(cost_map, active, prev_active)  # returns changed edges
        if changed or path_intersects_hazard(path, active):
            planner.km += planner._h(planner.start, world_to_vox(drone_pos))
            planner.start = world_to_vox(drone_pos)
            for (u, v) in changed:
                planner.update_vertex(v, cost_map)
            planner.compute_shortest_path(cost_map)
            path = extract_path(planner)
            replan_count += 1

        drone_pos = step_drone(drone_pos, path, V_MAX, DT)
        t += DT

    return trajectory, replan_count, J_detour, J_hazard
```

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Airspace | 100 m × 100 m × 30 m |
| Voxel resolution | 2 m |
| Grid size | 50 × 50 × 15 voxels |
| Cruise altitude | 20 m (z-index 10) |
| Drone max speed | 5 m/s |
| Time step | 0.05 s |
| Number of hazard cells | 8 |
| Hazard radius | 4 – 12 m (uniform) |
| Hazard lifetime | 30 – 120 s (uniform) |
| Hazard severity $w_k$ | 0.3 – 1.0 (uniform) |
| Inflation factor $\alpha$ | 10 |
| 26-connectivity | face + edge + corner adjacency |
| Heuristic | Euclidean distance |

---

## Expected Output

- 3D trajectory plot showing the drone path with and without weather rerouting, overlaid on active hazard cell volumes at selected time snapshots
- Timeline plot: active hazard count and replanning events vs simulation time
- Altitude time series: z(t) showing vertical detour manoeuvres to pass above/below storm cells
- Cost breakdown bar chart: $J_{base}$, $J_{detour}$, $J_{hazard}$ for nominal vs rerouted flight
- Replanning statistics: number of replanning events, mean replanning latency (ms), mean detour distance per event
- Animation (GIF): top-down and isometric view, hazard cells fading in/out in orange, drone path in blue, active replanned segments in red

---

## Extensions

1. **Predictive rerouting**: use a weather forecast model (Ornstein-Uhlenbeck process for cell motion) to pre-emptively re-plan before a cell reaches the path
2. **Multi-drone coordination**: two drones share a common cost map; D* Lite replanning for one drone updates a shared hazard layer that the second drone also reads (S031 conflict deconfliction applied to weather cells)
3. **Energy-aware cost function**: replace Euclidean edge cost with energy consumption model $c(u,v) = P_{cruise} \cdot \|u-v\| / v_{max} + P_{wind} \cdot w_{cell}$ to trade detour length against wind energy penalty
4. **Anisotropic hazard cells**: model wind as a vector field $\mathbf{w}(\mathbf{x},t)$ and compute edge costs as $c(u,v) = \|u-v\| / (v_{max} - \hat{\mathbf{e}}_{uv} \cdot \mathbf{w})$ for headwind penalty
5. **Lifelong D* Lite over repeated missions**: accumulate weather statistics across multiple deliveries to build a probabilistic hazard map that seeds the initial cost function

---

## Related Scenarios

- Prerequisites: [S021 Point Delivery](S021_point_delivery.md), [S022 Obstacle Avoidance](S022_obstacle_avoidance.md), [S024 Wind Compensation](S024_wind_compensation.md)
- Follow-ups: [S031 Path Deconfliction](S031_path_deconfliction.md), [S038 Disaster Relief](S038_disaster_relief.md)
- Algorithm reference: see [domains/02_logistics_delivery/README.md](../../domains/02_logistics_delivery/README.md) §1.1 (D* Lite)
