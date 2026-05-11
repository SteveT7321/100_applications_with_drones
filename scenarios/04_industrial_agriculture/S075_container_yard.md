# S075 Port Container Yard Inventory

**Domain**: Industrial & Agriculture | **Difficulty**: ⭐⭐ | **Status**: `[ ]` Not Started
**Algorithm**: RRT* Path Planning + Scan Dwell Scheduling | **Dimension**: 3D

---

## Problem Definition

**Setup**: A single drone performs a full inventory audit of a $100 \times 80\ \text{m}$ container
yard. Fifty ISO-standard shipping containers (each $6 \times 2.5 \times 2.5\ \text{m}$) are
arranged in six rows; individual stacks reach 1, 2, or 3 containers high (maximum stack height
$7.5\ \text{m}$). Containers are separated by $3\ \text{m}$-wide aisles in the $y$-direction and
$4\ \text{m}$ gaps in the $x$-direction. The drone must visit one barcode scan position per
container face, for a total of $N_{scan} = 50$ scan waypoints, each centred $0.8\ \text{m}$ in
front of the target face at the midpoint of the container's long axis and at the height of the
container's barcode panel.

Navigation is performed entirely at $z = 5.0\ \text{m}$ unless a container stack is taller than
that, in which case the drone climbs to $z_{stack} + 1.0\ \text{m}$ before traversing. RRT* plans
a collision-free path through the 3D obstacle field formed by the container blocks. At each scan
waypoint the drone hovers for a dwell time of $T_{dwell} = 1.5\ \text{s}$ to capture the barcode.
GPS position error with $\sigma_{GPS} = 0.1\ \text{m}$ may displace the drone from the nominal
waypoint; the drone stabilises to within $d_{stab} = 0.05\ \text{m}$ of the waypoint before the
dwell timer starts, adding a stabilisation overhead of up to $T_{stab} = 0.8\ \text{s}$ per stop.

**Roles**:
- **Drone**: single UAV executing an RRT*-planned trajectory at $v_{scan} = 1.0\ \text{m/s}$
  with a double-integrator dynamics model and GPS noise.
- **Containers**: 50 static box obstacles, each an axis-aligned rectangular prism in the
  workspace $\mathcal{W} = [0, 100] \times [0, 80] \times [0, 7.5]\ \text{m}$.
- **Scan positions**: 50 waypoints (one per container face), derived offline from the known yard
  layout and ordered by a nearest-neighbour scan schedule.

**Objective**: Complete all 50 barcode scans with zero collisions while minimising total mission
time $T_{mission}$. Compare **RRT*** and **A*** (on a 3D grid) as alternative planners on path
smoothness and computation time.

---

## Mathematical Model

### Container Obstacle Model

Each container $k$ occupies an axis-aligned bounding box:

$$\mathcal{B}_k = [x_k^{lo},\; x_k^{hi}] \times [y_k^{lo},\; y_k^{hi}] \times [0,\; z_k^{hi}]$$

where $z_k^{hi} = n_k \times 2.5\ \text{m}$ with stack height $n_k \in \{1, 2, 3\}$.

A configuration $\mathbf{q} = (x, y, z)$ is **collision-free** if the drone's spherical body
(radius $r_d = 0.25\ \text{m}$) does not intersect any $\mathcal{B}_k$ expanded by $r_d$:

$$\text{free}(\mathbf{q}) = \bigwedge_{k=1}^{50}
  \left[ d_\infty\!\left(\mathbf{q},\, \mathcal{B}_k\right) > r_d \right]$$

where $d_\infty(\mathbf{q}, \mathcal{B}_k)$ is the $\ell^\infty$ clearance from $\mathbf{q}$ to
the surface of $\mathcal{B}_k$.

### RRT* Path Planning

RRT* incrementally builds a tree $\mathcal{T} = (V, E)$ rooted at $\mathbf{q}_{start}$
with optimal-cost rewiring. At each iteration:

1. **Sample**: draw $\mathbf{q}_{rand} \sim \mathcal{U}(\mathcal{W})$ (or the goal with
   probability $p_{goal} = 0.05$).

2. **Nearest**: find the closest tree node by Euclidean distance:

$$x_{near} = \underset{v \in V}{\arg\min}\;\|\mathbf{q}_{rand} - v\|$$

3. **Steer**: extend from $x_{near}$ toward $\mathbf{q}_{rand}$ by step $\eta$:

$$x_{new} = x_{near} + \eta\,\frac{\mathbf{q}_{rand} - x_{near}}{\|\mathbf{q}_{rand} - x_{near}\|}$$

4. **Collision check**: accept $x_{new}$ only if the segment
   $\overline{x_{near}\, x_{new}}$ lies entirely in free space (sampled at $\delta = r_d/2$).

5. **Near set**: collect all nodes within rewire radius:

$$\mathcal{X}_{near} = \left\{ v \in V \;\middle|\; \|v - x_{new}\| \leq r \right\}, \quad
r = \min\!\left(\gamma_{RRT^*}\!\left(\frac{\ln|V|}{|V|}\right)^{1/3},\; \eta\right)$$

with $\gamma_{RRT^*} = (2(1+1/3))^{1/3}(|\mathcal{W}|/\zeta_3)^{1/3}$ where $|\mathcal{W}|$
is the free-space volume and $\zeta_3 = 4\pi/3$ is the unit-ball volume.

6. **Connect**: choose the parent minimising cost-to-root:

$$x_{parent} = \underset{x \in \mathcal{X}_{near}\,\cup\,\{x_{near}\}}{\arg\min}\;
  \left[\text{cost}(x) + \|x - x_{new}\|\right]$$

7. **Rewire**: for each $x \in \mathcal{X}_{near}$, if

$$\text{cost}(x_{new}) + \|x_{new} - x\| < \text{cost}(x)$$

and the segment $\overline{x_{new}\,x}$ is collision-free, re-parent $x$ to $x_{new}$.

The cost of a node is the accumulated Euclidean path length from root:

$$\text{cost}(x_{new}) = \text{cost}(x_{parent}) + \|x_{parent} - x_{new}\|$$

### Path Smoothing (Shortcut + Bézier)

After RRT* terminates, a two-stage smoother reduces path length:

**Stage 1 — Shortcut pruning**: for $N_{short} = 200$ iterations, pick two random waypoints
$\mathbf{w}_i, \mathbf{w}_j$ ($i < j$) on the raw path; if the segment
$\overline{\mathbf{w}_i\,\mathbf{w}_j}$ is collision-free, delete intermediate nodes.

**Stage 2 — Cubic Bézier interpolation**: for each consecutive triplet
$(\mathbf{w}_{i-1}, \mathbf{w}_i, \mathbf{w}_{i+1})$ with an interior angle
$\theta < \theta_{max} = 150°$, replace the sharp corner with a cubic Bézier curve:

$$\mathbf{B}(t) = (1-t)^3\mathbf{P}_0 + 3(1-t)^2 t\,\mathbf{P}_1
  + 3(1-t)t^2\,\mathbf{P}_2 + t^3\mathbf{P}_3, \quad t \in [0,1]$$

where control points $\mathbf{P}_1, \mathbf{P}_2$ are placed at a blend fraction
$\alpha = 0.3$ along the incoming and outgoing edge directions.

### A* on 3D Grid (Comparison Baseline)

The workspace is discretised into a $200 \times 160 \times 15$ grid at $\Delta_{grid} = 0.5\ \text{m}$
resolution. A* searches a 26-connected graph with:

$$f(n) = g(n) + h(n)$$

where $g(n)$ is the exact cost from start (Euclidean edge weight), and $h(n)$ is the
admissible 3D Euclidean heuristic:

$$h(n) = \left\|(n_x, n_y, n_z) - \mathbf{q}_{goal}\right\|$$

Obstacle cells are inflated by $\lceil r_d / \Delta_{grid} \rceil = 1$ cell in all directions
before search. After A*, the same shortcut + Bézier smoother is applied to the grid path.

### Scan Dwell and Stabilisation Model

At each scan waypoint $\mathbf{w}_j$, the drone must stabilise before the barcode read begins.
The stabilisation time depends on the GPS-induced position error at arrival:

$$\epsilon_j = \|\hat{\mathbf{p}}_{arrive} - \mathbf{w}_j\|, \quad
  \hat{\mathbf{p}}_{arrive} = \mathbf{w}_j + \boldsymbol{\eta},\quad
  \boldsymbol{\eta} \sim \mathcal{N}(\mathbf{0},\,\sigma_{GPS}^2\mathbf{I})$$

A proportional hover controller drives the drone to the waypoint; the time to reach $d_{stab}$ is:

$$T_{stab,j} = \frac{\epsilon_j}{K_{stab} \cdot v_{scan}}, \quad K_{stab} = 0.5$$

Total time at scan waypoint $j$:

$$T_{stop,j} = T_{stab,j} + T_{dwell}$$

### Mission Time and Path Length

Total mission time:

$$T_{mission} = \frac{L_{path}}{v_{scan}} + \sum_{j=1}^{N_{scan}} T_{stop,j}$$

where $L_{path}$ is the total RRT* path length (sum of Euclidean segment lengths after smoothing).

Path smoothness is measured by the **cumulative turning angle** (CTA):

$$\text{CTA} = \sum_{i=1}^{N_{wp}-2}
  \arccos\!\left(\frac{(\mathbf{w}_{i+1}-\mathbf{w}_i)\cdot(\mathbf{w}_{i+2}-\mathbf{w}_{i+1})}
  {\|\mathbf{w}_{i+1}-\mathbf{w}_i\|\,\|\mathbf{w}_{i+2}-\mathbf{w}_{i+1}\|}\right)$$

A lower CTA indicates a smoother, more flyable path.

---

## Implementation

```python
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib.animation as animation
import heapq
import time

# ── Yard layout ───────────────────────────────────────────────────────────────
YARD_X, YARD_Y, YARD_Z = 100.0, 80.0, 10.0    # m — workspace bounds
CONTAINER_L = 6.0    # m — container length (x)
CONTAINER_W = 2.5    # m — container width (y)
CONTAINER_H = 2.5    # m — container height per tier

# Row layout: 6 rows along y, containers packed along x with 4 m gaps
ROW_Y_STARTS = [5.0, 15.0, 27.0, 39.0, 51.0, 63.0]   # y origin of each row
ROW_N_CONT   = [9, 9, 8, 8, 8, 8]                      # containers per row
X_SPACING    = CONTAINER_L + 4.0    # 10 m pitch in x
X_ORIGIN     = 3.0                  # x start of first container

RNG_SEED = 42
rng = np.random.default_rng(RNG_SEED)

def build_containers():
    """Return list of dicts {x_lo, x_hi, y_lo, y_hi, z_hi, cx, cy, cz}."""
    containers = []
    for row_idx, (y0, n) in enumerate(zip(ROW_Y_STARTS, ROW_N_CONT)):
        for col in range(n):
            stack = rng.integers(1, 4)   # 1, 2, or 3 tiers
            x_lo = X_ORIGIN + col * X_SPACING
            x_hi = x_lo + CONTAINER_L
            y_lo = y0
            y_hi = y0 + CONTAINER_W
            z_hi = stack * CONTAINER_H
            cx   = (x_lo + x_hi) / 2.0
            cy   = (y_lo + y_hi) / 2.0
            cz   = z_hi / 2.0
            containers.append(dict(x_lo=x_lo, x_hi=x_hi,
                                   y_lo=y_lo, y_hi=y_hi,
                                   z_lo=0.0,  z_hi=z_hi,
                                   cx=cx, cy=cy, cz=cz,
                                   stack=stack))
    return containers

# ── Drone and planning constants ──────────────────────────────────────────────
R_DRONE      = 0.25     # m — drone body radius (safety inflation)
V_SCAN       = 1.0      # m/s — cruise speed
T_DWELL      = 1.5      # s — barcode read dwell time
D_STAB       = 0.05     # m — stabilisation threshold
K_STAB       = 0.5      # proportional stabilisation gain factor
GPS_SIGMA    = 0.1      # m — GPS position noise std dev
Z_CRUISE     = 5.0      # m — default cruise altitude
Z_MARGIN     = 1.0      # m — clearance above tallest container in path

# RRT* parameters
ETA_RRT      = 2.0      # m — RRT step length
N_ITER       = 5000     # max RRT* iterations
P_GOAL_BIAS  = 0.05     # goal sampling probability
GAMMA_RRT    = 12.0     # rewire radius constant

# A* grid parameters
GRID_RES     = 0.5      # m — voxel side length
GRID_NX      = int(YARD_X / GRID_RES)    # 200
GRID_NY      = int(YARD_Y / GRID_RES)    # 160
GRID_NZ      = int(YARD_Z / GRID_RES)    # 20

# Smoothing
N_SHORTCUT   = 200      # shortcut iterations
BEZIER_ALPHA = 0.3      # Bézier blend fraction
THETA_MAX    = np.radians(150.0)   # max corner angle for Bézier insertion

# ── Obstacle helpers ──────────────────────────────────────────────────────────
def aabb_clearance(p, box, r):
    """Return True if sphere(p, r) does not intersect box (with inflation r)."""
    dx = max(box["x_lo"] - r - p[0], 0.0, p[0] - box["x_hi"] - r)
    dy = max(box["y_lo"] - r - p[1], 0.0, p[1] - box["y_hi"] - r)
    dz = max(box["z_lo"] - r - p[2], 0.0, p[2] - box["z_hi"] - r)
    return (dx**2 + dy**2 + dz**2) > 0.0

def is_free(p, containers):
    """Check a single configuration for collision."""
    if not (0 <= p[0] <= YARD_X and 0 <= p[1] <= YARD_Y and 0 <= p[2] <= YARD_Z):
        return False
    return all(aabb_clearance(p, c, R_DRONE) for c in containers)

def segment_free(a, b, containers, n_checks=20):
    """Discretised collision check along segment a→b."""
    for t in np.linspace(0, 1, n_checks):
        if not is_free(a + t * (b - a), containers):
            return False
    return True

# ── RRT* ─────────────────────────────────────────────────────────────────────
class RRTStar:
    def __init__(self, start, goal, containers):
        self.start = np.array(start, dtype=float)
        self.goal  = np.array(goal,  dtype=float)
        self.C     = containers
        self.nodes = [self.start.copy()]
        self.parent= [-1]
        self.cost  = [0.0]

    def _nearest(self, q):
        dists = [np.linalg.norm(v - q) for v in self.nodes]
        return int(np.argmin(dists))

    def _near(self, q):
        n = len(self.nodes)
        if n < 2:
            return [0]
        r = min(GAMMA_RRT * (np.log(n) / n) ** (1.0 / 3.0), ETA_RRT)
        return [i for i, v in enumerate(self.nodes)
                if np.linalg.norm(v - q) <= r]

    def plan(self):
        for _ in range(N_ITER):
            # Sample
            if rng.random() < P_GOAL_BIAS:
                q_rand = self.goal.copy()
            else:
                q_rand = rng.uniform([0, 0, 0], [YARD_X, YARD_Y, YARD_Z])

            # Nearest
            i_near = self._nearest(q_rand)
            x_near = self.nodes[i_near]
            diff   = q_rand - x_near
            d      = np.linalg.norm(diff)
            if d < 1e-6:
                continue
            x_new = x_near + min(ETA_RRT, d) * diff / d

            if not is_free(x_new, self.C):
                continue
            if not segment_free(x_near, x_new, self.C):
                continue

            # Near set and best parent
            near_ids = self._near(x_new)
            best_parent = i_near
            best_cost   = self.cost[i_near] + np.linalg.norm(x_new - x_near)
            for i in near_ids:
                c = self.cost[i] + np.linalg.norm(x_new - self.nodes[i])
                if c < best_cost and segment_free(self.nodes[i], x_new, self.C):
                    best_cost   = c
                    best_parent = i

            self.nodes.append(x_new)
            self.parent.append(best_parent)
            self.cost.append(best_cost)
            idx_new = len(self.nodes) - 1

            # Rewire
            for i in near_ids:
                c = best_cost + np.linalg.norm(x_new - self.nodes[i])
                if c < self.cost[i] and segment_free(x_new, self.nodes[i], self.C):
                    self.parent[i] = idx_new
                    self.cost[i]   = c

            # Goal check
            if np.linalg.norm(x_new - self.goal) < ETA_RRT:
                if segment_free(x_new, self.goal, self.C):
                    self.nodes.append(self.goal.copy())
                    self.parent.append(idx_new)
                    self.cost.append(best_cost + np.linalg.norm(self.goal - x_new))
                    return self._extract_path(len(self.nodes) - 1)

        # Return best partial path if goal not reached
        dists = [np.linalg.norm(v - self.goal) for v in self.nodes]
        return self._extract_path(int(np.argmin(dists)))

    def _extract_path(self, idx):
        path = []
        while idx != -1:
            path.append(self.nodes[idx].copy())
            idx = self.parent[idx]
        return path[::-1]

# ── Path smoothing ────────────────────────────────────────────────────────────
def shortcut_smooth(path, containers):
    path = [p.copy() for p in path]
    for _ in range(N_SHORTCUT):
        if len(path) < 3:
            break
        i = rng.integers(0, len(path) - 2)
        j = rng.integers(i + 1, len(path))
        if segment_free(path[i], path[j], containers):
            path = path[:i+1] + path[j:]
    return path

def bezier_smooth(path):
    if len(path) < 3:
        return path
    smooth = [path[0]]
    for i in range(1, len(path) - 1):
        v_in  = path[i]   - path[i-1]
        v_out = path[i+1] - path[i]
        d_in  = np.linalg.norm(v_in)
        d_out = np.linalg.norm(v_out)
        if d_in < 1e-6 or d_out < 1e-6:
            smooth.append(path[i])
            continue
        cos_a = np.dot(v_in, v_out) / (d_in * d_out)
        angle = np.arccos(np.clip(cos_a, -1.0, 1.0))
        if angle < THETA_MAX:
            p0 = path[i-1]
            p3 = path[i+1]
            p1 = path[i-1] + BEZIER_ALPHA * v_in
            p2 = path[i+1] - BEZIER_ALPHA * v_out
            ts = np.linspace(0, 1, 8)
            for t in ts:
                b = ((1-t)**3 * p0 + 3*(1-t)**2*t * p1
                     + 3*(1-t)*t**2 * p2 + t**3 * p3)
                smooth.append(b)
        else:
            smooth.append(path[i])
    smooth.append(path[-1])
    return smooth

# ── A* on 3D grid ─────────────────────────────────────────────────────────────
def build_occupancy_grid(containers):
    occ = np.zeros((GRID_NX, GRID_NY, GRID_NZ), dtype=bool)
    inflate = int(np.ceil(R_DRONE / GRID_RES))
    for c in containers:
        xi_lo = max(0, int(c["x_lo"] / GRID_RES) - inflate)
        xi_hi = min(GRID_NX, int(c["x_hi"] / GRID_RES) + inflate + 1)
        yi_lo = max(0, int(c["y_lo"] / GRID_RES) - inflate)
        yi_hi = min(GRID_NY, int(c["y_hi"] / GRID_RES) + inflate + 1)
        zi_lo = 0
        zi_hi = min(GRID_NZ, int(c["z_hi"] / GRID_RES) + inflate + 1)
        occ[xi_lo:xi_hi, yi_lo:yi_hi, zi_lo:zi_hi] = True
    return occ

def astar_3d(start, goal, occ):
    """26-connected A* on occupancy grid. Returns list of world-frame waypoints."""
    def to_cell(p):
        return (int(np.clip(p[0] / GRID_RES, 0, GRID_NX-1)),
                int(np.clip(p[1] / GRID_RES, 0, GRID_NY-1)),
                int(np.clip(p[2] / GRID_RES, 0, GRID_NZ-1)))
    def to_world(cell):
        return np.array([cell[0]*GRID_RES + GRID_RES/2,
                         cell[1]*GRID_RES + GRID_RES/2,
                         cell[2]*GRID_RES + GRID_RES/2])

    s = to_cell(start)
    g = to_cell(goal)
    h = lambda c: np.sqrt((c[0]-g[0])**2 + (c[1]-g[1])**2 + (c[2]-g[2])**2) * GRID_RES

    open_heap = [(h(s), 0.0, s, None)]
    came_from = {}
    g_score   = {s: 0.0}

    deltas = [(dx,dy,dz) for dx in (-1,0,1)
                          for dy in (-1,0,1)
                          for dz in (-1,0,1)
                          if not (dx==0 and dy==0 and dz==0)]
    while open_heap:
        f, cost, cur, prev = heapq.heappop(open_heap)
        if cur in came_from:
            continue
        came_from[cur] = prev
        if cur == g:
            break
        for d in deltas:
            nb = (cur[0]+d[0], cur[1]+d[1], cur[2]+d[2])
            if not (0 <= nb[0] < GRID_NX and 0 <= nb[1] < GRID_NY
                    and 0 <= nb[2] < GRID_NZ):
                continue
            if occ[nb[0], nb[1], nb[2]]:
                continue
            step = np.linalg.norm(d) * GRID_RES
            new_g = cost + step
            if nb not in g_score or new_g < g_score[nb]:
                g_score[nb] = new_g
                heapq.heappush(open_heap, (new_g + h(nb), new_g, nb, cur))

    # Reconstruct
    if g not in came_from:
        return [start, goal]
    path_cells = []
    cur = g
    while cur is not None:
        path_cells.append(cur)
        cur = came_from[cur]
    return [to_world(c) for c in reversed(path_cells)]

# ── Scan scheduling ───────────────────────────────────────────────────────────
def build_scan_waypoints(containers):
    """Generate one scan waypoint per container (front face, mid-height)."""
    waypoints = []
    for c in containers:
        # Scan the front face (low-y side), 0.8 m offset in -y
        wx = c["cx"]
        wy = c["y_lo"] - 0.8
        wz = min(c["z_hi"] / 2.0, Z_CRUISE)
        waypoints.append(np.array([wx, wy, wz]))
    return waypoints

def nearest_neighbour_order(start, waypoints):
    """Greedy nearest-neighbour tour from start."""
    remaining = list(range(len(waypoints)))
    order = []
    current = np.array(start)
    while remaining:
        dists = [np.linalg.norm(waypoints[i] - current) for i in remaining]
        best  = remaining[int(np.argmin(dists))]
        order.append(best)
        current = waypoints[best]
        remaining.remove(best)
    return order

# ── Metrics ───────────────────────────────────────────────────────────────────
def path_length(path):
    return sum(np.linalg.norm(path[i+1]-path[i]) for i in range(len(path)-1))

def cumulative_turning_angle(path):
    cta = 0.0
    for i in range(len(path)-2):
        v1 = path[i+1] - path[i]
        v2 = path[i+2] - path[i+1]
        d1, d2 = np.linalg.norm(v1), np.linalg.norm(v2)
        if d1 < 1e-9 or d2 < 1e-9:
            continue
        cos_a = np.dot(v1, v2) / (d1 * d2)
        cta  += np.degrees(np.arccos(np.clip(cos_a, -1.0, 1.0)))
    return cta

def simulate_mission(full_path, n_scan):
    """Compute mission time including GPS stabilisation at each waypoint."""
    L = path_length(full_path)
    t_travel = L / V_SCAN
    t_scan_total = 0.0
    for _ in range(n_scan):
        noise = rng.normal(0, GPS_SIGMA, 3)
        eps   = np.linalg.norm(noise)
        t_stab = eps / (K_STAB * V_SCAN)
        t_scan_total += t_stab + T_DWELL
    return t_travel, t_scan_total, t_travel + t_scan_total

def run_simulation():
    containers = build_containers()
    waypoints  = build_scan_waypoints(containers)
    start_pos  = np.array([0.0, 0.0, Z_CRUISE])
    order      = nearest_neighbour_order(start_pos, waypoints)

    full_waypoints = [start_pos] + [waypoints[i] for i in order]

    # ── RRT* planning ─────────────────────────────────────────────────────────
    print("Planning with RRT*...")
    t0 = time.perf_counter()
    rrt_path = [start_pos]
    for wp in full_waypoints[1:]:
        planner = RRTStar(rrt_path[-1], wp, containers)
        seg     = planner.plan()
        if len(seg) > 1:
            rrt_path.extend(seg[1:])
    rrt_path  = shortcut_smooth(rrt_path, containers)
    rrt_path  = bezier_smooth(rrt_path)
    rrt_path  = [np.array(p) for p in rrt_path]
    t_rrt     = time.perf_counter() - t0

    # ── A* planning ───────────────────────────────────────────────────────────
    print("Planning with A*...")
    t0  = time.perf_counter()
    occ = build_occupancy_grid(containers)
    astar_path = [start_pos]
    for wp in full_waypoints[1:]:
        seg = astar_3d(astar_path[-1], wp, occ)
        astar_path.extend(seg[1:])
    astar_path  = shortcut_smooth(astar_path, containers)
    astar_path  = bezier_smooth(astar_path)
    astar_path  = [np.array(p) for p in astar_path]
    t_astar     = time.perf_counter() - t0

    # ── Metrics ───────────────────────────────────────────────────────────────
    n = len(waypoints)
    rrt_L,  rrt_cta              = path_length(rrt_path),   cumulative_turning_angle(rrt_path)
    astar_L, astar_cta           = path_length(astar_path), cumulative_turning_angle(astar_path)
    t_tr_r, t_sc_r, t_mis_r     = simulate_mission(rrt_path,   n)
    t_tr_a, t_sc_a, t_mis_a     = simulate_mission(astar_path, n)

    print(f"\n{'Metric':<35} {'RRT*':>12} {'A*':>12}")
    print("-" * 60)
    print(f"{'Containers scanned':<35} {n:>12d} {n:>12d}")
    print(f"{'Path length (m)':<35} {rrt_L:>12.1f} {astar_L:>12.1f}")
    print(f"{'Cumulative turning angle (deg)':<35} {rrt_cta:>12.1f} {astar_cta:>12.1f}")
    print(f"{'Planning time (s)':<35} {t_rrt:>12.3f} {t_astar:>12.3f}")
    print(f"{'Travel time (s)':<35} {t_tr_r:>12.1f} {t_tr_a:>12.1f}")
    print(f"{'Total scan dwell time (s)':<35} {t_sc_r:>12.1f} {t_sc_a:>12.1f}")
    print(f"{'Total mission time (s)':<35} {t_mis_r:>12.1f} {t_mis_a:>12.1f}")

    return containers, waypoints, rrt_path, astar_path, order
```

---

## Key Parameters

| Parameter | Symbol | Value |
|-----------|--------|-------|
| Yard dimensions | $L_x \times L_y$ | 100 × 80 m |
| Container size | $l \times w \times h_{tier}$ | 6 × 2.5 × 2.5 m |
| Number of containers | $N_{cont}$ | 50 |
| Maximum stack height | $z_{max}$ | 7.5 m (3 tiers) |
| Drone body radius | $r_d$ | 0.25 m |
| Number of scan waypoints | $N_{scan}$ | 50 |
| Scan standoff distance | $d_{scan}$ | 0.8 m |
| Cruise altitude | $Z_{cruise}$ | 5.0 m |
| Altitude margin above stacks | $Z_{margin}$ | 1.0 m |
| Inspection speed | $v_{scan}$ | 1.0 m/s |
| Barcode dwell time | $T_{dwell}$ | 1.5 s |
| Stabilisation threshold | $d_{stab}$ | 0.05 m |
| GPS noise std dev | $\sigma_{GPS}$ | 0.1 m |
| RRT* step length | $\eta$ | 2.0 m |
| RRT* max iterations | $N_{iter}$ | 5 000 |
| RRT* rewire constant | $\gamma_{RRT^*}$ | 12.0 |
| Goal sampling probability | $p_{goal}$ | 0.05 |
| A* grid resolution | $\Delta_{grid}$ | 0.5 m |
| A* grid dimensions | $N_x \times N_y \times N_z$ | 200 × 160 × 20 |
| Shortcut smoothing iterations | $N_{short}$ | 200 |
| Bézier blend fraction | $\alpha$ | 0.3 |

---

## Expected Output

- **3D yard overview** (`mpl_toolkits.mplot3d`): container boxes rendered as grey rectangular
  prisms with height proportional to stack tier (1–3 tiers); RRT* path drawn as a red polyline,
  A* path as a blue polyline; scan waypoints shown as green crosses; drone start position as a
  black circle; yard boundary drawn as a black wireframe rectangle.
- **Path comparison plot**: side-by-side top-down ($xy$) and front ($xz$) projections of the
  smoothed RRT* and A* paths; corridor walls visible as grey column slices; path colour-coded
  by altitude ($z$); legend showing total path length for each planner.
- **Planning metrics bar chart**: two grouped bar charts — left group: path length (m); right
  group: cumulative turning angle (degrees) — one bar per planner (RRT* in red, A* in blue);
  planning time annotated as text above each bar.
- **Animation** (`FuncAnimation`): top-down 2D view of the drone traversing the RRT*-planned
  path; drone shown as a filled red circle with a fading trail (last 30 positions); green dot
  flashes at each scan waypoint during dwell; containers drawn as grey rectangles; frame rate
  10 fps, saved as
  `outputs/04_industrial_agriculture/s075_container_yard/s075_container_yard.gif`.

Terminal metrics printed at completion:

```
Metric                              RRT*           A*
------------------------------------------------------------
Containers scanned                    50           50
Path length (m)                    312.4        358.7
Cumulative turning angle (deg)     820.3       1540.6
Planning time (s)                    4.82         0.31
Travel time (s)                    312.4        358.7
Total scan dwell time (s)          108.2        108.5
Total mission time (s)             420.6        467.2
```

---

## Extensions

1. **Multi-drone parallel inventory**: deploy 2–3 drones entering from different yard gates;
   partition the 50 scan waypoints by $k$-means clustering so that each drone covers the
   cluster nearest its entry; use a shared collision-avoidance layer (velocity obstacles) to
   prevent mid-air conflicts in shared aisles.
2. **Dynamic container placement**: re-run the planner as containers are loaded or unloaded
   by a straddle carrier; invalidate only the RRT* sub-trees that intersect the changed
   obstacle and re-grow from the affected nodes, measuring re-planning latency vs. full
   replanning.
3. **RFID instead of barcode**: replace the fixed $T_{dwell} = 1.5\ \text{s}$ scan with an
   RFID probabilistic detection model $P_{det}(d) = 1 - e^{-\lambda / d^2}$; optimise the
   standoff distance $d_{scan}$ to maximise $P_{det}$ per pass while minimising path length.
4. **Battery-constrained mission**: impose a battery capacity of $E_{max} = 900\ \text{Wh}$
   with consumption $c_{fly} = 150\ \text{W}$ and $c_{hover} = 120\ \text{W}$; insert
   charging stops at a depot $((0, 0, 0))$ whenever remaining energy falls below a threshold,
   and measure the total number of charging interruptions and mission time increase.
5. **Adverse weather (wind)**: inject a uniform horizontal wind $\mathbf{v}_{wind} = (3, 0, 0)\ \text{m/s}$
   into the dynamics; compare how path-following accuracy (RMSE from planned path) degrades
   for RRT* vs. A* paths and assess which geometric profile is more wind-resilient.

---

## Related Scenarios

- Prerequisites: [S043 Confined Space Exploration](../03_environmental_sar/S043_confined_space.md), [S074 Automated Warehouse Picking](S074_automated_warehouse_picking.md)
- Follow-ups: [S076 Port Crane Inspection](S076_port_crane_inspection.md) (vertical structure survey), [S077 Ship Hull Survey](S077_ship_hull_survey.md) (curved surface coverage)
- Algorithmic cross-reference: [S004 Obstacle Chase](../01_pursuit_evasion/S004_obstacle_chase.md) (RRT in dynamic environments), [S069 Warehouse Inventory](S069_warehouse_inventory.md) (scan-scheduling fundamentals)
