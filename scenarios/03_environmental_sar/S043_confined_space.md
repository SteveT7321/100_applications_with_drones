# S043 Confined Space Exploration

**Domain**: Environmental Monitoring & SAR | **Difficulty**: ⭐⭐⭐ | **Status**: `[x]` Complete

---

## Problem Definition

**Setup**: A small rescue drone is deployed inside a collapsed building whose interior is unknown.
The floor plan (30 × 30 m) contains load-bearing walls, fallen debris columns, and narrow corridors
formed by partial collapses. No prior map is available; the drone must build one from scratch while
navigating safely. The environment is discretised into a $60 \times 60$ occupancy grid at
$\Delta = 0.5$ m resolution. The drone carries a 360° lidar-range sensor with radius
$r_s = 2.0$ m and adds Gaussian range noise $w \sim \mathcal{N}(0, \sigma_r^2)$ to every beam.

The drone body has an effective radius of $r_d = 0.15$ m (width 0.3 m); it must not attempt to
traverse any gap narrower than $g_{min} = 0.35$ m (body diameter plus a 0.05 m safety margin).
Gap detection runs on the current map to identify passable openings before the Bug2 navigator
commits to a new heading.

**Roles**:
- **Drone**: single explorer; starts at entry point $\mathbf{p}_0 = (2.0, 2.0)$ m (grid cell
  $(4, 4)$); no return-to-base constraint within the mission horizon.
- **Environment**: static obstacles (walls, debris) encoded as a ground-truth occupancy matrix
  unknown to the drone at $t = 0$.

**Objective**: Maximise the fraction of **reachable free space** mapped within a mission horizon
of $T_{max} = 300$ s, while never entering a cell whose occupancy probability exceeds
$p_{occ} = 0.65$ and never squeezing through a gap narrower than $g_{min}$.

**Exploration strategy** (three compared):
1. **Random walk** — choose a random free direction at each step; baseline.
2. **Frontier-based (nearest)** — always navigate to the nearest unexplored frontier cell.
3. **Frontier-based + Bug2** — frontier selection as above; Bug2 local planner handles obstacle
   circumnavigation en route to the frontier goal.

---

## Mathematical Model

### Occupancy Grid and Log-Odds Update

The map is a grid $\mathcal{M}$ of $N_x \times N_y$ cells. Each cell $c$ stores a log-odds belief:

$$l(c) = \log \frac{p(c \mid z_{1:t})}{1 - p(c \mid z_{1:t})}$$

Sensor model (inverse sensor model, binary Bayes):

$$l_t(c) = l_{t-1}(c) + \begin{cases}
l_{occ} & \text{if } c \text{ is the hit cell of a beam} \\
l_{free} & \text{if } c \text{ is a free cell along the beam} \\
0 & \text{otherwise}
\end{cases}$$

with update increments $l_{occ} = +0.85$ and $l_{free} = -0.40$, and clamping bounds
$l_{min} = -2.0$, $l_{max} = 3.5$ to prevent log-odds saturation. The recovered probability:

$$p(c) = \frac{e^{l(c)}}{1 + e^{l(c)}}$$

A cell is classified **occupied** if $p(c) > 0.65$, **free** if $p(c) < 0.35$, and **unknown**
otherwise. Initial belief: $l(c) = 0$ for all cells (uniform prior, $p = 0.5$).

### Gaussian Range Sensor Model

Following MATH_FOUNDATIONS.md §6, the range measurement from the drone at position
$\mathbf{p} = (x, y)$ along beam angle $\theta_b$ is:

$$z_b = h_b(\mathbf{p}) + w_b, \qquad w_b \sim \mathcal{N}(0,\, \sigma_r^2)$$

where $h_b(\mathbf{p})$ is the true range to the nearest obstacle along $\theta_b$ (ray-casting
on the ground-truth grid) and $\sigma_r = 0.05$ m. Beams are cast at $N_b = 36$ uniformly spaced
angles $\theta_b = 2\pi b / N_b$, $b = 0, \ldots, N_b - 1$, each truncated at $r_s = 2.0$ m.

### Gap Detection

After each map update, passable gaps are identified along the drone's intended heading
$\hat{\mathbf{v}}$ to goal $\mathbf{g}$. For each candidate passage direction $\phi$, the gap
width is approximated by scanning perpendicular to $\phi$ at range $d_{scan} = 0.5$ m:

$$w_{gap}(\phi) = \Delta \cdot \sum_{k=-K}^{K}
  \mathbf{1}\!\left[p\!\left(c\!\left(\mathbf{p} + d_{scan}\hat{\phi} + k\Delta\hat{\phi}^{\perp}\right)\right) < 0.65\right]$$

where $\hat{\phi}^{\perp}$ is the unit vector perpendicular to $\hat{\phi}$, $K = 3$ (7-cell scan
window = 3.5 m), and $\mathbf{1}[\cdot]$ is the indicator function. The passage is declared
**passable** if $w_{gap}(\phi) \geq g_{min} = 0.35$ m (i.e., at least one free cell in the
7-cell window yields a 0.5 m clearance, but the drone requires the continuous run of free cells
spanning $\lceil g_{min}/\Delta \rceil = 1$ cell plus safety).

Formally, the gap is passable when the **maximum contiguous run** of free cells in the scan
window satisfies:

$$\max_{\text{contiguous run}} \left(\text{run length} \times \Delta\right) \geq g_{min}$$

### Bug2 Algorithm

Bug2 follows the **M-line** (straight line from start $\mathbf{p}_0$ to current goal $\mathbf{g}$)
as long as no obstacle is within turning distance $d_{turn} = 0.4$ m. When an obstacle is
detected, it switches to **wall-following mode** and circumnavigates the obstacle boundary
(right-hand rule) until the M-line is re-encountered closer to $\mathbf{g}$ than the hit point.

State machine:

$$\text{mode} = \begin{cases}
\text{MOTION\_TO\_GOAL} & \|\mathbf{p} - \mathbf{g}\| > d_{goal} \text{ and no obstacle within } d_{turn} \\
\text{BOUNDARY\_FOLLOW} & \text{obstacle detected within } d_{turn}
\end{cases}$$

Heading in MOTION\_TO\_GOAL:

$$\dot{\mathbf{p}} = v_d \cdot \frac{\mathbf{g} - \mathbf{p}}{\|\mathbf{g} - \mathbf{p}\|}$$

Heading in BOUNDARY\_FOLLOW (right-hand rule, wall on left):

$$\dot{\mathbf{p}} = v_d \cdot \hat{\mathbf{t}}_{wall}$$

where $\hat{\mathbf{t}}_{wall}$ is the unit tangent to the detected wall boundary, estimated from
the two nearest occupied cells. Exit condition: the drone's projection onto the M-line is closer
to $\mathbf{g}$ than the boundary entry point $\mathbf{p}_{hit}$ by at least $d_{progress} = 0.1$ m.

### Frontier Extraction

A **frontier cell** is a free cell adjacent to at least one unknown cell:

$$\mathcal{F} = \left\{ c \in \mathcal{M} \;\middle|\; p(c) < 0.35 \;\text{ and }\;
  \exists\, c' \in \mathcal{N}_4(c):\; 0.35 \leq p(c') \leq 0.65 \right\}$$

where $\mathcal{N}_4(c)$ is the 4-connected neighbourhood of $c$. Frontier cells are clustered
into contiguous frontier segments by connected-component labelling; the centroid of each cluster
becomes a candidate goal. The nearest-frontier selector picks:

$$\mathbf{g}^* = \underset{\mathbf{g}_f \in \text{clusters}}{\arg\min}\;
  d_{nav}(\mathbf{p},\, \mathbf{g}_f)$$

where $d_{nav}$ is the shortest known-free-cell path length (BFS on the current map) rather than
Euclidean distance, to avoid routing through walls.

### Coverage Metric

Let $\mathcal{C}_{reach}$ denote the set of ground-truth reachable free cells (computed offline
by BFS from $\mathbf{p}_0$ respecting $g_{min}$). The exploration coverage at time $t$ is:

$$\text{Coverage}(t) = \frac{\left|\left\{c \in \mathcal{C}_{reach} \;:\; p_t(c) < 0.35\right\}\right|}{|\mathcal{C}_{reach}|} \times 100\%$$

---

## Implementation

```python
import numpy as np
import matplotlib.pyplot as plt
from collections import deque

# Key constants
GRID_RES      = 0.5          # m — cell side length
GRID_W        = 60           # cells (30 m)
GRID_H        = 60           # cells (30 m)
SENSOR_RADIUS = 2.0          # m
N_BEAMS       = 36           # lidar rays
SIGMA_RANGE   = 0.05         # m — range noise std dev
L_OCC         = 0.85         # log-odds increment for hit cell
L_FREE        = -0.40        # log-odds decrement for free cells
L_MIN, L_MAX  = -2.0, 3.5   # clamping bounds
P_OCC_THRESH  = 0.65         # occupied probability threshold
P_FREE_THRESH = 0.35         # free probability threshold
G_MIN         = 0.35         # m — minimum passable gap
DRONE_SPEED   = 0.5          # m/s
D_TURN        = 0.4          # m — Bug2 obstacle detection distance
D_GOAL        = 0.3          # m — goal-reached tolerance
T_MAX         = 300.0        # s — mission horizon
DT            = 0.1          # s — simulation timestep

START_M       = np.array([2.0, 2.0])   # m

def log_odds_to_prob(l):
    return np.exp(l) / (1.0 + np.exp(l))

def world_to_cell(pos_m):
    """Convert world coordinates (m) to grid cell indices (row, col)."""
    col = int(pos_m[0] / GRID_RES)
    row = int(pos_m[1] / GRID_RES)
    return np.clip(row, 0, GRID_H - 1), np.clip(col, 0, GRID_W - 1)

def cell_to_world(row, col):
    return np.array([col * GRID_RES + GRID_RES / 2,
                     row * GRID_RES + GRID_RES / 2])

def cast_ray(ground_truth, pos_m, angle, r_max):
    """Ray-cast on ground truth; return true range and hit cell."""
    dx, dy = np.cos(angle), np.sin(angle)
    for r in np.arange(GRID_RES, r_max + GRID_RES, GRID_RES / 2):
        pt = pos_m + r * np.array([dx, dy])
        row, col = world_to_cell(pt)
        if ground_truth[row, col]:    # occupied
            return r, (row, col)
    return r_max, None

def update_map(log_odds, ground_truth, pos_m):
    """Cast N_BEAMS rays, add noise, update log-odds map."""
    angles = np.linspace(0, 2 * np.pi, N_BEAMS, endpoint=False)
    for angle in angles:
        true_range, hit_cell = cast_ray(ground_truth, pos_m, angle, SENSOR_RADIUS)
        noisy_range = true_range + np.random.normal(0, SIGMA_RANGE)
        noisy_range = np.clip(noisy_range, 0, SENSOR_RADIUS)
        # Mark free cells along beam
        for r in np.arange(GRID_RES, noisy_range - GRID_RES / 2, GRID_RES / 2):
            pt = pos_m + r * np.array([np.cos(angle), np.sin(angle)])
            row, col = world_to_cell(pt)
            log_odds[row, col] = np.clip(log_odds[row, col] + L_FREE, L_MIN, L_MAX)
        # Mark hit cell as occupied
        if noisy_range < SENSOR_RADIUS and hit_cell is not None:
            log_odds[hit_cell] = np.clip(log_odds[hit_cell] + L_OCC, L_MIN, L_MAX)

def extract_frontiers(prob_map):
    """Return list of (row, col) frontier cells."""
    free  = prob_map < P_FREE_THRESH
    unkn  = (prob_map >= P_FREE_THRESH) & (prob_map <= P_OCC_THRESH)
    frontiers = []
    for r in range(1, GRID_H - 1):
        for c in range(1, GRID_W - 1):
            if free[r, c]:
                nbrs = [(r-1,c),(r+1,c),(r,c-1),(r,c+1)]
                if any(unkn[nr, nc] for nr, nc in nbrs):
                    frontiers.append((r, c))
    return frontiers

def bfs_nearest_frontier(prob_map, start_cell, frontiers):
    """BFS shortest path (in free cells) to nearest frontier cluster centroid."""
    if not frontiers:
        return None
    frontier_set = set(frontiers)
    visited = set()
    queue = deque([(start_cell, [start_cell])])
    while queue:
        cell, path = queue.popleft()
        if cell in frontier_set:
            return cell
        if cell in visited:
            continue
        visited.add(cell)
        r, c = cell
        for nr, nc in [(r-1,c),(r+1,c),(r,c-1),(r,c+1)]:
            if 0 <= nr < GRID_H and 0 <= nc < GRID_W:
                if (nr, nc) not in visited:
                    p = log_odds_to_prob(prob_map[nr, nc]) if False else prob_map[nr, nc]
                    if prob_map[nr, nc] < P_OCC_THRESH:
                        queue.append(((nr, nc), path + [(nr, nc)]))
    return None

def bug2_step(pos_m, goal_m, log_odds, mode, wall_follow_dir):
    """Single Bug2 step; returns new_pos_m, new_mode, new_wall_follow_dir."""
    prob = log_odds_to_prob(log_odds)
    to_goal = goal_m - pos_m
    dist_to_goal = np.linalg.norm(to_goal)
    if dist_to_goal < D_GOAL:
        return pos_m, 'REACHED', wall_follow_dir

    if mode == 'MOTION_TO_GOAL':
        heading = to_goal / dist_to_goal
        probe = pos_m + D_TURN * heading
        pr, pc = world_to_cell(probe)
        if prob[pr, pc] > P_OCC_THRESH:
            # Hit obstacle — switch to boundary follow
            wall_dir = np.array([-heading[1], heading[0]])  # right-hand: rotate +90
            return pos_m, 'BOUNDARY_FOLLOW', wall_dir
        new_pos = pos_m + DRONE_SPEED * DT * heading
        return new_pos, 'MOTION_TO_GOAL', wall_follow_dir

    else:  # BOUNDARY_FOLLOW
        heading = wall_follow_dir / (np.linalg.norm(wall_follow_dir) + 1e-9)
        probe = pos_m + D_TURN * heading
        pr, pc = world_to_cell(probe)
        if prob[pr, pc] > P_OCC_THRESH:
            # Turn away from wall (rotate -90 around z)
            heading = np.array([heading[1], -heading[0]])
        # Check if back on M-line and closer to goal
        to_goal = goal_m - pos_m
        if np.linalg.norm(to_goal) < dist_to_goal - 0.1:
            return pos_m, 'MOTION_TO_GOAL', wall_follow_dir
        new_pos = pos_m + DRONE_SPEED * DT * heading
        return new_pos, 'BOUNDARY_FOLLOW', heading

def run_simulation(ground_truth):
    """
    ground_truth : (GRID_H, GRID_W) bool array — True = occupied.
    Returns trajectory, log_odds history, coverage_curve.
    """
    log_odds = np.zeros((GRID_H, GRID_W), dtype=float)
    pos = START_M.copy()
    trajectory = [pos.copy()]
    coverage_curve = []

    # Precompute reachable ground-truth free cells via BFS
    start_cell = world_to_cell(START_M)
    reachable = set()
    q = deque([start_cell])
    gt_free = ~ground_truth
    visited_reach = set()
    while q:
        cell = q.popleft()
        if cell in visited_reach:
            continue
        visited_reach.add(cell)
        reachable.add(cell)
        r, c = cell
        for nr, nc in [(r-1,c),(r+1,c),(r,c-1),(r,c+1)]:
            if 0 <= nr < GRID_H and 0 <= nc < GRID_W and gt_free[nr, nc]:
                q.append((nr, nc))

    mode = 'MOTION_TO_GOAL'
    goal = None
    wall_dir = np.array([1.0, 0.0])
    t = 0.0

    while t < T_MAX:
        update_map(log_odds, ground_truth, pos)
        prob = log_odds_to_prob(log_odds)

        # Frontier-based goal selection
        if goal is None or np.linalg.norm(pos - goal) < D_GOAL or mode == 'REACHED':
            frontiers = extract_frontiers(prob)
            start_cell = world_to_cell(pos)
            nearest = bfs_nearest_frontier(prob, start_cell, frontiers)
            if nearest is None:
                break   # fully explored
            goal = cell_to_world(*nearest)
            mode = 'MOTION_TO_GOAL'

        pos, mode, wall_dir = bug2_step(pos, goal, log_odds, mode, wall_dir)
        trajectory.append(pos.copy())

        # Coverage
        mapped_free = {world_to_cell(cell_to_world(r, c))
                       for r in range(GRID_H) for c in range(GRID_W)
                       if prob[r, c] < P_FREE_THRESH}
        cov = len(reachable & mapped_free) / max(len(reachable), 1) * 100.0
        coverage_curve.append((t, cov))

        t += DT

    return np.array(trajectory), log_odds, coverage_curve
```

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Environment size | 30 × 30 m |
| Grid resolution $\Delta$ | 0.5 m/cell |
| Grid dimensions | 60 × 60 cells |
| Sensor radius $r_s$ | 2.0 m |
| Number of lidar beams $N_b$ | 36 |
| Range noise std dev $\sigma_r$ | 0.05 m |
| Log-odds hit increment $l_{occ}$ | +0.85 |
| Log-odds free decrement $l_{free}$ | −0.40 |
| Log-odds clamp $[l_{min}, l_{max}]$ | [−2.0, 3.5] |
| Occupied probability threshold $p_{occ}$ | 0.65 |
| Free probability threshold $p_{free}$ | 0.35 |
| Drone body width | 0.3 m |
| Minimum passable gap $g_{min}$ | 0.35 m |
| Bug2 obstacle detection distance $d_{turn}$ | 0.4 m |
| Drone cruise speed $v_d$ | 0.5 m/s |
| Mission horizon $T_{max}$ | 300 s |
| Simulation timestep $\Delta t$ | 0.1 s |
| Start position $\mathbf{p}_0$ | (2.0, 2.0) m |

---

## Expected Output

- **Map evolution panel**: four snapshots of the log-odds occupancy map at $t = 0, 100, 200, 300$ s;
  colour-coded free (white) / unknown (grey) / occupied (black); drone position marked as a red dot;
  frontier cells highlighted in cyan.
- **Trajectory overlay**: final trajectory drawn as a blue polyline on the completed map; start
  marked with a green circle, end with a red cross; narrow gaps shown as orange shading.
- **Coverage vs. time curve**: three lines (Random Walk / Frontier-Nearest / Frontier + Bug2) showing
  $\text{Coverage}(t)$ from 0 to $T_{max}$; horizontal dashed line at 100% of reachable space.
- **Gap passage log**: table printed to stdout listing each gap traversal event: timestamp, gap
  width estimate, pre/post position, mode at entry.
- **Mode timeline**: stacked horizontal bar per strategy showing time fraction spent in
  MOTION\_TO\_GOAL vs. BOUNDARY\_FOLLOW.
- **Animation (GIF)**: occupancy map updating in real time, drone icon moving through corridors,
  frontier centroids shown as pulsing cyan circles, Bug2 wall-follow arcs drawn in yellow.

---

## Extensions

1. **Multi-drone coordinated exploration**: deploy $N = 3$ drones entering from different access
   points; partition the frontier queue so that each drone targets the frontier most distant from
   all others (max-dispersion allocation), then merge their individual maps via a shared log-odds
   accumulation server.
2. **Victim detection layer**: attach a heat sensor; model detections as $z_{heat} = T_{body} + w$
   with $w \sim \mathcal{N}(0, \sigma_T^2)$; integrate a Bayesian occupancy-augmented victim
   probability map that triggers a hover-and-alert when $p_{victim}(c) > 0.8$.
3. **Structural hazard avoidance**: label cells with a hazard potential $V_h(c)$ proportional to
   proximity to heavily-loaded occupied cells (unstable debris model); weight the BFS path cost as
   $c_{path} = d_{nav} + \alpha V_h$ to bias routes away from fragile structures.
4. **Dynamic gap closure**: simulate debris settling by randomly flipping free cells to occupied
   during the mission; require the drone to re-check known gaps on each frontier planning cycle and
   reroute if a previously passable gap has closed.
5. **3D confined-space extension**: extend the grid to a volumetric voxel map
   ($60 \times 60 \times 20$ cells at 0.5 m); model floor-level rubble piles and ceiling height
   constraints; add altitude control to find the vertical slice with maximum clearance through each
   horizontal gap.

---

## Related Scenarios

- Prerequisites: [S041 Wildfire Boundary Scan](S041_wildfire_boundary_scan.md), [S042 Missing Person Search](S042_missing_person_search.md)
- Follow-ups: [S044 Wall Crack Inspection](S044_wall_crack_inspection.md) (post-mapping structural survey), [S048 Lawnmower Coverage](S048_lawnmower_coverage.md) (systematic coverage baseline)
- Algorithmic cross-reference: [S004 Obstacle Chase](../01_pursuit_evasion/S004_obstacle_chase.md) (Bug algorithm in pursuit context), [S022 Obstacle Avoidance Delivery](../02_logistics_delivery/S022_obstacle_avoidance_delivery.md) (potential field avoidance)
