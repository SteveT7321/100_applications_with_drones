# S022 Obstacle Avoidance Delivery

**Domain**: Logistics & Delivery | **Difficulty**: ⭐⭐ | **Status**: `[x]` Completed

---

## Problem Definition

**Setup**: A single delivery drone must fly from a depot (start position) to a drop zone (goal position) through a 3D urban-like environment containing cylindrical building obstacles. Two planners are compared head-to-head: a naive straight-line path (which collides) versus an RRT* path that routes around all obstacles.

**Roles**:
- **Delivery drone**: executes the planned path at constant cruise speed, tracking waypoints via a PID position controller
- **Obstacles**: static cylinders of varying radii and heights, arranged to force non-trivial detours

**Objective**: demonstrate that RRT* finds a collision-free, asymptotically-optimal path; quantify the detour overhead (path length ratio vs. straight-line distance) and track the waypoints with sub-0.1 m accuracy.

---

## Mathematical Model

### RRT* Algorithm

The configuration space is $\mathcal{C} = \mathbb{R}^3$. A node is collision-free if it lies outside all obstacle volumes. The algorithm maintains a tree $\mathcal{T}$ rooted at $\mathbf{q}_{start}$.

**Nearest neighbor** (Euclidean distance):

$$\mathbf{q}_{nearest} = \arg\min_{\mathbf{q} \in \mathcal{T}} \|\mathbf{q}_{rand} - \mathbf{q}\|_2$$

**Steering** (step size $\eta$):

$$\mathbf{q}_{new} = \mathbf{q}_{nearest} + \eta \cdot \frac{\mathbf{q}_{rand} - \mathbf{q}_{nearest}}{\|\mathbf{q}_{rand} - \mathbf{q}_{nearest}\|_2}$$

**Near-node search radius** (shrinks as tree grows, ensuring asymptotic optimality):

$$r_n = \min\!\left(\gamma_{RRT^*} \left(\frac{\log n}{n}\right)^{1/d},\; \eta\right)$$

where $n$ is the current tree size, $d = 3$ is the dimensionality, and $\gamma_{RRT^*} > \gamma^*$ is chosen as:

$$\gamma^* = 2\left(1 + \frac{1}{d}\right)^{1/d}\left(\frac{\mu(\mathcal{C}_{free})}{\zeta_d}\right)^{1/d}$$

with $\mu(\mathcal{C}_{free})$ the free-space volume and $\zeta_d = \frac{4}{3}\pi$ the unit ball volume in 3D.

**Cost and rewiring**: The cost of a node is the accumulated path length from the root:

$$\text{cost}(\mathbf{q}_{new}) = \text{cost}(\mathbf{q}_{min}) + \|\mathbf{q}_{new} - \mathbf{q}_{min}\|_2$$

After inserting $\mathbf{q}_{new}$, rewire all near nodes $\mathbf{q}_{near}$ if a path through $\mathbf{q}_{new}$ is cheaper:

$$\text{if } \text{cost}(\mathbf{q}_{new}) + \|\mathbf{q}_{new} - \mathbf{q}_{near}\|_2 < \text{cost}(\mathbf{q}_{near}) \Rightarrow \text{parent}(\mathbf{q}_{near}) \leftarrow \mathbf{q}_{new}$$

### Collision Checking

Each obstacle $i$ is a vertical cylinder with centre $(c_{x,i},\, c_{y,i})$, radius $r_i$, and height $h_i$. A segment $\mathbf{q}_a \to \mathbf{q}_b$ is collision-free iff for every obstacle $i$:

$$d_{\perp,i}(\mathbf{q}_a, \mathbf{q}_b) > r_i + r_{drone} \quad \text{OR} \quad \max(q_{a,z}, q_{b,z}) < z_{ground} \quad \text{OR} \quad \min(q_{a,z}, q_{b,z}) > h_i$$

where $d_{\perp,i}$ is the minimum 2D Euclidean distance from the cylinder axis $(c_{x,i}, c_{y,i})$ to the line segment projected onto the $xy$-plane, and $r_{drone} = 0.15$ m is the drone safety radius.

### Path Shortcutting (Post-Processing)

After RRT* terminates, a greedy shortcut pass removes unnecessary waypoints:

$$\text{For each triple } (\mathbf{w}_{k-1},\, \mathbf{w}_k,\, \mathbf{w}_{k+1}): \text{ if segment } \mathbf{w}_{k-1} \to \mathbf{w}_{k+1} \text{ is collision-free, remove } \mathbf{w}_k$$

### Waypoint Tracking Controller

The drone follows the pruned waypoint sequence using a proportional velocity command:

$$\mathbf{v}_{cmd} = v_{cruise} \cdot \frac{\mathbf{w}_{target} - \mathbf{p}}{\|\mathbf{w}_{target} - \mathbf{p}\|_2 + \epsilon}, \qquad \|\mathbf{v}_{cmd}\| \le v_{max}$$

Waypoint transition threshold:

$$\|\mathbf{p} - \mathbf{w}_{target}\|_2 < r_{wp} = 0.3 \text{ m} \Rightarrow \text{advance to next waypoint}$$

### Path Quality Metrics

| Metric | Definition |
|--------|-----------|
| Path length | $L = \sum_{k=1}^{K-1} \|\mathbf{w}_{k+1} - \mathbf{w}_k\|_2$ |
| Detour ratio | $\rho = L / \|\mathbf{q}_{goal} - \mathbf{q}_{start}\|_2$ |
| Minimum clearance | $d_{min} = \min_{i,\,\mathbf{p}(t)} \left(\|\mathbf{p}_{xy}(t) - \mathbf{c}_i\|_2 - r_i\right)$ |
| Tracking RMSE | $\epsilon_{track} = \sqrt{\frac{1}{T}\sum_t \|\mathbf{p}(t) - \mathbf{p}_{ref}(t)\|^2}$ |

---

## Implementation

```
src/base/drone_base.py
src/02_logistics_delivery/s022_obstacle_avoidance_delivery.py
```

```python
# Key constants
CRUISE_SPEED   = 3.0    # m/s — cruise velocity along waypoints
MAX_ITER       = 2000   # RRT* maximum iterations
STEP_SIZE      = 1.0    # m  — eta: tree expansion step
GAMMA_RRT      = 12.0   # gamma_RRT* coefficient (tuned for scene volume)
DRONE_RADIUS   = 0.15   # m  — collision safety radius
WP_THRESHOLD   = 0.3    # m  — waypoint acceptance radius
CTRL_FREQ      = 48     # Hz

# Core RRT* skeleton
class RRTStar:
    def __init__(self, start, goal, obstacles, bounds, step=STEP_SIZE, gamma=GAMMA_RRT):
        ...

    def plan(self, max_iter=MAX_ITER):
        for _ in range(max_iter):
            q_rand   = self._sample()
            q_near   = self._nearest(q_rand)
            q_new    = self._steer(q_near, q_rand)
            if not self._collision_free(q_near, q_new):
                continue
            near_nodes = self._near(q_new)
            q_min, c_min = self._choose_parent(q_new, near_nodes)
            self._insert(q_new, q_min, c_min)
            self._rewire(q_new, near_nodes)
        return self._extract_path()

    def _near(self, q_new):
        n = len(self.nodes)
        r = min(GAMMA_RRT * (np.log(n) / n) ** (1/3), STEP_SIZE)
        return [v for v in self.nodes if np.linalg.norm(v.pos - q_new) < r]

    def _rewire(self, q_new, near_nodes):
        for q_near in near_nodes:
            new_cost = self.cost[q_new] + np.linalg.norm(q_new.pos - q_near.pos)
            if new_cost < self.cost[q_near] and self._collision_free(q_new, q_near):
                q_near.parent = q_new
                self.cost[q_near] = new_cost

def shortcut(path, collision_free_fn):
    """Greedy shortcut: remove waypoints whose removal keeps path collision-free."""
    i = 0
    while i < len(path) - 2:
        if collision_free_fn(path[i], path[i + 2]):
            path.pop(i + 1)
        else:
            i += 1
    return path
```

```bash
conda activate drones
python src/02_logistics_delivery/s022_obstacle_avoidance_delivery.py
```

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Start position | (0, 0, 2) m |
| Goal position | (18, 12, 2) m |
| Straight-line distance | ~21.5 m |
| Cruise speed | 3.0 m/s |
| RRT* max iterations | 2 000 |
| Step size $\eta$ | 1.0 m |
| $\gamma_{RRT^*}$ | 12.0 |
| Drone safety radius | 0.15 m |
| Waypoint threshold | 0.3 m |
| Control frequency | 48 Hz |
| Flight altitude range | 1.0 – 5.0 m |

**Obstacle layout** (vertical cylinders):

| ID | Centre $(x, y)$ | Radius | Height |
|----|-----------------|--------|--------|
| 1 | (4.0, 2.0) | 1.2 m | 4.0 m |
| 2 | (7.0, 6.0) | 1.0 m | 5.0 m |
| 3 | (9.0, 1.5) | 0.8 m | 3.5 m |
| 4 | (12.0, 8.0) | 1.3 m | 4.5 m |
| 5 | (14.5, 4.0) | 0.9 m | 3.0 m |
| 6 | (16.0, 10.0) | 1.1 m | 4.0 m |

---

## Expected Output

- 3D trajectory plot: RRT* path in green, straight-line collision path in dashed red, obstacles as semi-transparent grey cylinders, drone trajectory as blue line
- RRT* tree visualization (scatter of explored nodes in light grey) overlaid on the scene
- Path length convergence plot: best path length vs. iteration number, showing asymptotic improvement
- Distance-to-nearest-obstacle over time: confirms clearance never drops below $r_{drone}$
- Waypoint tracking error over time: RMSE should remain below 0.1 m at cruise speed
- Summary metrics printed to console: path length, detour ratio $\rho$, minimum clearance, tracking RMSE, planning time

---

## Extensions

1. 3D obstacles (arbitrary-height buildings, spheres, no-fly zones at specific altitudes) — force true altitude variation
2. Informed RRT*: after finding the first solution, restrict sampling to a prolate spheroid to accelerate convergence
3. Kinodynamic RRT*: incorporate velocity and acceleration limits so waypoints are directly executable without a separate tracker
4. Dynamic obstacles (moving delivery truck, other drones): switch to MP-RRT or RRTX for online replanning
5. Compare with A* on a 3D voxel grid: measure planning time, path length, and memory usage

---

## Related Scenarios

- Prerequisites: [S021](S021_point_delivery.md) — basic A* point delivery (simpler obstacle-free baseline)
- Follow-ups: [S023](S023_moving_landing_pad.md) — dynamic goal (moving truck landing pad)
- Follow-ups: [S034](S034_weather_rerouting.md) — D* Lite dynamic replanning under changing wind zones
- See [domains/02_logistics_delivery/README.md](../../domains/02_logistics_delivery/README.md)
- Algorithm reference: [MATH_FOUNDATIONS.md §3.1](../../MATH_FOUNDATIONS.md) — RRT/RRT* foundations
