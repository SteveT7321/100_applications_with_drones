# S069 Automated Warehouse Inventory

**Domain**: Industrial & Agriculture | **Difficulty**: ⭐⭐ | **Status**: `[ ]` Not started

**Algorithm**: 3D A* Path Planning + Dwell-Based Scan Scheduling | **Dimension**: 3D

---

## Problem Definition

**Setup**: A single drone operates autonomously inside a 30 × 20 × 6 m GPS-denied warehouse.
Inventory stock is stored on metal shelving arranged in 20 columns at three vertical tiers
(z = 1, 3, 5 m), giving 60 shelf-face scan targets in total. The warehouse is discretised into a
3D occupancy grid at Δ = 0.5 m resolution (60 × 40 × 12 cells); shelf structures and the
building envelope are pre-loaded as known obstacles. The drone uses A* on this grid to plan
collision-free paths between successive scan waypoints, then hovers at each target for a dwell
period T_dwell = 1 s to simulate a barcode / RFID read. Position noise is modelled as
GPS_ERROR = 0.1 m standard deviation on each axis (indoor localisation only).

**Objective**: Visit all 60 shelf-face waypoints, achieve a scan-success rate ≥ 95 %, and
minimise total mission time T, subject to the constraint that the drone never enters an occupied
voxel.

---

## Mathematical Model

### 3D Occupancy Grid

The warehouse volume is represented as a voxel grid:

$$\mathcal{G} \subset \mathbb{Z}^3, \quad \Delta = 0.5\,\text{m}$$

Each voxel v is either free (occ(v) = 0) or occupied (occ(v) = 1). Shelf structures occupy the
voxels within 0.2 m of each shelf face; aisle corridors between columns are 1.5 m wide and
remain free.

### A* Path Planning

A* searches for the lowest-cost path from the current drone position to the next waypoint on the
discrete voxel graph. The cost function for node n is:

$$f(n) = g(n) + h(n)$$

where g(n) is the exact cost (sum of step lengths) from the start, and h(n) is the Euclidean
heuristic to the goal:

$$h(n) = \left\| \mathbf{p}(n) - \mathbf{p}_{goal} \right\|_2$$

The 26-connected neighbourhood (face + edge + corner voxels) is used to allow diagonal 3D
movement. Any voxel with occ = 1 is excluded from the search. Path cost is measured in metres.

### Waypoint Ordering (Nearest-Neighbour TSP Heuristic)

The 60 shelf-face waypoints are ordered by a greedy nearest-neighbour heuristic to reduce total
transit distance. Starting from the depot (home position), the drone always travels next to the
unvisited waypoint whose A* path cost is smallest:

$$w_{k+1} = \underset{w \in \mathcal{W} \setminus \{w_1,\ldots,w_k\}}{\arg\min}\; d_{A^*}(w_k,\, w)$$

where d_A*(u, v) is the A* path length from u to v.

### Scan Success Probability

When the drone arrives at a shelf-face waypoint it hovers at stand-off distance d_scan = 0.5 m.
Residual velocity noise (due to indoor positioning error and controller settling) is modelled as:

$$v_{noise} \sim \mathcal{N}(0,\, \sigma_v^2), \quad \sigma_v = \frac{\texttt{GPS\_ERROR}}{\Delta t_{ctrl}}$$

The probability that a single dwell attempt produces a successful scan is:

$$P_{scan}(v) = 1 - \exp\!\left(-k_{scan} \cdot \frac{v_{thresh} - v}{v_{thresh}}\right), \quad v \leq v_{thresh}$$

with sensitivity constant k_scan = 3.0 and velocity threshold v_thresh = 0.2 m/s. For v >
v_thresh the drone is moving too fast and P_scan = 0.

### Expected Scan Count

Over N = 60 targets, the expected number of successful scans is:

$$\mathbb{E}[N_{scanned}] = \sum_{i=1}^{N} P_{scan}(v_i)$$

The scan-success rate is:

$$\eta_{scan} = \frac{\mathbb{E}[N_{scanned}]}{N} \times 100\%$$

The mission requirement is η_scan ≥ 95 %.

### Mission Time Budget

Total mission time decomposes into transit and dwell components:

$$T_{mission} = \sum_{i=1}^{N} \left( t_{transit,i} + T_{dwell} \right)$$

where t_transit,i = d_A*(w_{i-1}, w_i) / v_cruise is the travel time along the i-th A* path at
cruise speed v_cruise = 1.0 m/s, and T_dwell = 1.0 s is the fixed hover period per shelf face.

### Speed–Accuracy Trade-off

The optimal cruise speed balances scan quality against transit time. Defining the per-waypoint
reward as P_scan(v) / t_transit(v):

$$v^* = \underset{v \leq v_{thresh}}{\arg\max}\; \frac{P_{scan}(v)}{d_i / v}$$

Differentiating and setting to zero yields v* = v_thresh / (1 + k_scan) for a fixed distance d_i,
showing that slower hover speeds pay diminishing returns once P_scan saturates.

---

## Implementation

```python
import numpy as np
import heapq
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

# --- Constants ---
GRID_RES     = 0.5          # m per voxel
WH_X, WH_Y, WH_Z = 60, 40, 12   # voxels (30 × 20 × 6 m)
INSPECT_SPEED = 1.0          # m/s cruise speed
INSPECT_DIST  = 0.5          # m stand-off distance from shelf face
GPS_ERROR     = 0.1          # m position noise std dev
T_DWELL       = 1.0          # s hover time per scan
V_THRESH      = 0.2          # m/s max velocity for successful scan
K_SCAN        = 3.0          # scan sensitivity constant
N_COLS        = 20           # shelf columns
TIER_Z        = [1.0, 3.0, 5.0]  # shelf tier heights (m)

def build_occupancy_grid():
    """Return (WH_Z, WH_Y, WH_X) bool array; True = occupied."""
    grid = np.zeros((WH_Z, WH_Y, WH_X), dtype=bool)
    # Boundary walls
    grid[:, 0, :] = grid[:, -1, :] = True
    grid[:, :, 0] = grid[:, :, -1] = True
    grid[0, :, :] = True
    # Shelf structures: 20 columns, each 0.4 m deep in y, full height tiers
    for col in range(N_COLS):
        cx = int((col + 0.5) * (WH_X / N_COLS))
        for iy in range(2, 6):               # shelf depth 2 voxels in y
            for iz in range(WH_Z):
                grid[iz, iy, cx] = True
    return grid

def astar_3d(grid, start, goal):
    """A* on 26-connected voxel graph; returns path as list of (iz,iy,ix)."""
    def h(a, b):
        return np.sqrt(sum((a[i]-b[i])**2 for i in range(3)))
    open_heap = [(h(start, goal), 0.0, start, [start])]
    visited = set()
    while open_heap:
        f, g, node, path = heapq.heappop(open_heap)
        if node == goal:
            return path
        if node in visited:
            continue
        visited.add(node)
        iz, iy, ix = node
        for dz in (-1,0,1):
            for dy in (-1,0,1):
                for dx in (-1,0,1):
                    if dz == dy == dx == 0:
                        continue
                    nz, ny, nx = iz+dz, iy+dy, ix+dx
                    if 0 <= nz < WH_Z and 0 <= ny < WH_Y and 0 <= nx < WH_X:
                        if not grid[nz, ny, nx] and (nz,ny,nx) not in visited:
                            step = np.sqrt(dz**2+dy**2+dx**2) * GRID_RES
                            g2 = g + step
                            heapq.heappush(open_heap,
                                (g2 + h((nz,ny,nx), goal), g2, (nz,ny,nx),
                                 path + [(nz,ny,nx)]))
    return []   # no path found

def p_scan(v):
    """Scan success probability as a function of hover velocity magnitude."""
    if v > V_THRESH:
        return 0.0
    return 1.0 - np.exp(-K_SCAN * (V_THRESH - v) / V_THRESH)

def nearest_neighbour_tour(waypoints, cost_matrix):
    """Greedy NN ordering; returns index list."""
    n = len(waypoints)
    unvisited = list(range(n))
    tour = [unvisited.pop(0)]
    while unvisited:
        last = tour[-1]
        nxt = min(unvisited, key=lambda j: cost_matrix[last][j])
        tour.append(nxt)
        unvisited.remove(nxt)
    return tour

def simulate_mission(grid, waypoints):
    """
    Run full inventory mission.
    Returns: total_time, scan_results, full_path_m, per_leg_costs
    """
    n = len(waypoints)
    # Build pairwise A* cost matrix
    cost_matrix = np.full((n, n), np.inf)
    paths = {}
    for i in range(n):
        for j in range(n):
            if i != j:
                p = astar_3d(grid, waypoints[i], waypoints[j])
                if p:
                    d = (len(p)-1) * GRID_RES
                    cost_matrix[i][j] = d
                    paths[(i,j)] = p
    tour = nearest_neighbour_tour(waypoints, cost_matrix)
    total_time = 0.0
    scan_successes = []
    full_path_m = []
    for k in range(len(tour)-1):
        i, j = tour[k], tour[k+1]
        seg = paths.get((i,j), [])
        d = cost_matrix[i][j]
        t_transit = d / INSPECT_SPEED
        # Simulate residual hover velocity
        v_hover = abs(np.random.normal(0, GPS_ERROR / 0.1))
        success = np.random.rand() < p_scan(v_hover)
        scan_successes.append(success)
        total_time += t_transit + T_DWELL
        full_path_m.extend([(ix*GRID_RES, iy*GRID_RES, iz*GRID_RES)
                            for iz, iy, ix in seg])
    scan_rate = np.mean(scan_successes) * 100.0
    return total_time, scan_successes, full_path_m, scan_rate
```

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Warehouse dimensions | 30 × 20 × 6 m |
| Grid resolution | 0.5 m/voxel |
| Grid size | 60 × 40 × 12 voxels |
| Number of shelf columns | 20 |
| Shelf tier heights | z = 1, 3, 5 m |
| Total scan targets | 60 |
| Stand-off scan distance d_scan | 0.5 m |
| Dwell time per target T_dwell | 1.0 s |
| Cruise speed v_cruise | 1.0 m/s |
| Velocity threshold for scan v_thresh | 0.2 m/s |
| Scan sensitivity constant k_scan | 3.0 |
| Indoor position noise GPS_ERROR | 0.1 m |
| A* connectivity | 26-connected |
| Scan success requirement | 95 % |

---

## Expected Output

- **3D flight path plot**: Matplotlib 3D axes showing the warehouse shell (grey wire-frame),
  shelf structures (grey boxes), the A* route as a red polyline, scan waypoints as green spheres,
  and the home depot as a blue marker; viewed from two angles (isometric + top-down).
- **Scan success rate bar chart**: per-shelf-tier success counts (z = 1, 3, 5 m) as grouped bars;
  a dashed horizontal line marks the 95 % requirement; colour indicates pass (green) / fail (red).
- **Speed–accuracy trade-off curve**: P_scan(v) vs. cruise speed v from 0 to v_thresh; a vertical
  dashed line at v* marks the optimal speed; a shaded band shows the feasible region
  (η_scan ≥ 95 %).
- **Mission time breakdown**: stacked bar showing total transit time vs. total dwell time; annotated
  with path length in metres and number of A* re-plans.
- **Animation (GIF)**: 3D animated view of the drone icon traversing the warehouse aisle-by-aisle,
  with scan waypoints changing colour from white → green (success) or red (fail) as the drone
  dwells; a live counter shows elapsed time and scan count.
- **Console metrics table**:

  | Metric | Value |
  |--------|-------|
  | Total mission time (s) | … |
  | Total path length (m) | … |
  | Scan success rate (%) | … |
  | Missed shelves | … |
  | Mean transit time per leg (s) | … |

---

## Extensions

1. **Re-scan on failure**: if a scan fails (P_scan < threshold draw), the drone immediately
   re-attempts from a slightly adjusted position; model the additional hover cost and study how
   retry budget affects total mission time.
2. **Dynamic obstacle avoidance**: introduce a forklift moving along a known aisle; the drone
   replans its A* path in real time when the forklift voxels enter its planned route within a
   look-ahead horizon of 3 m.
3. **Multi-drone parallelism**: deploy two drones simultaneously, partitioning the 60 waypoints
   into two clusters (k-means on 3D positions); each drone plans its own A* tour; compare total
   wall-clock time with the single-drone baseline.
4. **Battery-constrained sub-tours**: add a battery model (energy ∝ path length + hover time);
   force the drone to return to the charging depot when remaining energy falls below a safety
   margin; count the number of charging interruptions and their impact on inventory completion
   time.
5. **Probabilistic occupancy map**: replace the known static grid with a SLAM-derived occupancy
   map updated from a 2D lidar scan at each timestep; study how map uncertainty degrades A* path
   quality relative to the ground-truth grid baseline.

---

## Related Scenarios

- Prerequisites: [S065 Building 3D Modeling Sampling](S065_3d_scan_path.md), [S068 Large-Scale Farmland Cooperative Spraying](S068_large_field_spray.md)
- Next: [S074 Mine 3D Mapping](S074_mine_mapping.md) (GNSS-denied volumetric mapping), [S075 Port Container Yard Inventory](S075_container_yard.md) (high-density outdoor inventory)
- Algorithmic cross-reference: [S043 Confined Space Exploration](../03_environmental_sar/S043_confined_space.md) (A* + frontier in GPS-denied interior)
