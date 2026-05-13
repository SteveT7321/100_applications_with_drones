# S093 Earthquake Rubble Search and Rescue

**Domain**: Special Ops & Entertainment | **Difficulty**: ⭐⭐⭐⭐ | **Status**: `[ ]` Not Started
**Algorithm**: 3D Voxel Mapping + Frontier Exploration + Acoustic Detection | **Dimension**: 3D

---

## Problem Definition

**Setup**: A post-earthquake rubble field measuring $20 \times 20 \times 5$ m is populated with
random voxel obstacles at a **40 % fill rate** — chunks of concrete, rebar, and debris that block
line-of-sight and physical passage. No GPS signal penetrates the rubble; the drone must navigate
entirely from onboard sensors. The environment contains **3 buried victims**, each broadcasting a
low-power acoustic beacon that can be detected within a nominal range of $r_{acoustic} = 2$ m.
A micro-drone equipped with a short-range **LiDAR** (range $r_{lidar} = 3$ m, 360° horizontal,
$N_{beams} = 120$ beams) and a directional **acoustic sensor** ($\sigma_a = 0.8$ m) enters the
rubble pile from a safe perimeter point and explores autonomously.

Structural clearance is the central challenge: randomly collapsed debris leaves narrow passages
with a minimum clearance of $0.4$ m. The drone body radius is $0.15$ m, requiring the path planner
to operate on obstacle-inflated voxels with an inflation radius of $r_{inf} = 0.25$ m. Exploration
follows a **3D frontier policy** — any voxel classified as unknown that borders at least one free
voxel is a frontier candidate — with 3D A* used to route the drone through tight passages toward
the selected frontier.

**Roles**:

- **Drone** (1): micro-UAV, body radius 0.15 m, maximum speed $v_{max} = 0.8$ m/s; carries LiDAR
  and acoustic sensor; runs occupancy update, frontier detection, and A* on-board.
- **Victims** (3): stationary; each emitting an acoustic beacon; positions unknown at mission start;
  independently detectable based on Euclidean distance.
- **Rubble obstacles**: static voxelised debris; generated once per mission with a fixed random
  seed; passages of width $\geq 0.4$ m guaranteed to exist between the entrance and all victim
  locations (connectivity enforced during environment generation).

**Objective**: Locate all 3 victims within the mission time $T_{max} = 300$ s. Report:

1. **Victims found** $N_{found}$ — count of victims detected with $P_{detect} \geq 0.9$.
2. **Volume coverage** $\xi = V_{explored} / V_{total\_free} \times 100\%$ — fraction of
   free voxels visited.
3. **Mission time** $T_{mission}$ (s) — time elapsed when the last victim is confirmed, or
   $T_{max}$ if the mission times out.
4. **Expected cumulative detections** $E[N_{found}](t)$ — analytic prediction vs simulation.

---

## Mathematical Model

### 3D Voxel Occupancy Grid

The environment is discretised into a grid of voxels with side length $\Delta_{vox} = 0.1$ m,
giving a grid of $200 \times 200 \times 50$ cells. Each voxel $v$ stores a log-odds value
$\ell(v)$, initialised to $\ell_0 = 0$ (uniform prior, $P = 0.5$).

For each LiDAR beam $j$ with measured range $r_j$:

- **Occupied update** at the terminal hit voxel $v_{hit}$:

$$\ell(v_{hit}) \mathrel{+}= \log\frac{P_{occ}}{1 - P_{occ}}, \qquad P_{occ} = 0.85$$

- **Free update** along every voxel $v_{free}$ traversed before the hit:

$$\ell(v_{free}) \mathrel{+}= \log\frac{P_{free}}{1 - P_{free}}, \qquad P_{free} = 0.4$$

Both updates are clamped to prevent saturation:

$$\ell(v) \leftarrow \mathrm{clip}\!\bigl(\ell(v),\; \ell_{min},\; \ell_{max}\bigr), \qquad
  \ell_{min} = -10,\quad \ell_{max} = 10$$

Posterior occupancy probability and voxel classification:

$$P(v) = \frac{e^{\ell(v)}}{1 + e^{\ell(v)}}$$

$$\text{state}(v) = \begin{cases}
  \text{occupied} & P(v) > 0.7 \\
  \text{free}     & P(v) < 0.3 \\
  \text{unknown}  & \text{otherwise}
\end{cases}$$

### Obstacle Inflation

Before path planning, occupied voxels are dilated by $r_{inf} = 0.25$ m in all three axes using a
spherical structuring element of radius $r_{inf}$. A voxel $v$ is **inflated-occupied** if any
occupied voxel lies within Euclidean distance $r_{inf}$:

$$v \in \mathcal{V}_{inflated} \iff \exists\, v' \in \mathcal{V}_{occ} : \|\mathbf{x}_v - \mathbf{x}_{v'}\| \leq r_{inf}$$

The A* planner operates exclusively on $\mathcal{V}_{free} \setminus \mathcal{V}_{inflated}$ to
guarantee safe passage through the minimum-clearance gaps.

### 3D A* Path Planning

The planner searches over the 26-connected voxel graph (face, edge, and corner neighbours). Let
$\mathbf{s}$ be the drone's current voxel and $\mathbf{g}$ the target frontier voxel. A* minimises
the total cost:

$$f(\mathbf{n}) = g(\mathbf{n}) + h(\mathbf{n})$$

$$g(\mathbf{n}) = \text{(accumulated path cost from } \mathbf{s} \text{ to } \mathbf{n)}$$

$$h(\mathbf{n}) = \|\mathbf{x}_{\mathbf{n}} - \mathbf{x}_{\mathbf{g}}\|_2 \quad \text{(Euclidean heuristic)}$$

Edge cost between neighbouring voxels $\mathbf{n}$ and $\mathbf{n}'$:

$$c(\mathbf{n}, \mathbf{n}') = \|\mathbf{x}_{\mathbf{n}} - \mathbf{x}_{\mathbf{n}'}\|_2 = \Delta_{vox} \cdot \sqrt{\delta_x^2 + \delta_y^2 + \delta_z^2}$$

where $\delta_x, \delta_y, \delta_z \in \{0, 1\}$ are the axis-wise index differences. Diagonal
moves thus cost $\Delta_{vox}\sqrt{2}$ (face-diagonal) or $\Delta_{vox}\sqrt{3}$ (body-diagonal).
Voxels in $\mathcal{V}_{inflated}$ are treated as impassable (infinite cost).

### 3D Frontier Detection

A **frontier voxel** satisfies:

$$v_f \in \mathcal{V}_{unknown} \quad \text{and} \quad \exists\, v' \in \mathcal{N}_{26}(v_f) : v' \in \mathcal{V}_{free}$$

where $\mathcal{N}_{26}(v_f)$ is the 26-connected neighbourhood of $v_f$. Frontier voxels are
grouped by connected-components analysis on the 26-connectivity graph. The centroid of cluster $c$:

$$\mathbf{g}_c = \frac{1}{|\mathcal{F}_c|} \sum_{v \in \mathcal{F}_c} \mathbf{x}_v$$

Frontier selection balances information gain and travel cost:

$$c^* = \arg\min_{c} \frac{\|\mathbf{g}_c - \mathbf{p}_{drone}\|_2}{|\mathcal{F}_c|^{1/2}}$$

(nearest large frontier cluster preferred). The drone then executes the A* path toward $\mathbf{g}_{c^*}$.

### Acoustic Victim Detection Model

Each victim $i$ at position $\mathbf{v}_i \in \mathbb{R}^3$ emits a continuous beacon. The drone
at position $\mathbf{p}$ measures a noisy signal; the probabilistic detection model is:

$$P_{detect,i}(\mathbf{p}) = \exp\!\left(-\frac{d_i^2}{2\,\sigma_a^2}\right), \qquad
  d_i = \|\mathbf{p} - \mathbf{v}_i\|_2, \quad \sigma_a = 0.8 \text{ m}$$

At each timestep the acoustic sensor produces a binary observation:

$$z_i \sim \mathrm{Bernoulli}\!\bigl(P_{detect,i}(\mathbf{p})\bigr)$$

Victim $i$ is **confirmed found** when the cumulative detection count exceeds threshold
$K_{conf} = 3$ positive observations within a sliding window of $T_{win} = 5$ s. Equivalently,
the simplified instantaneous model used for expected-value analysis is:

$$z_i = \mathbf{1}\!\left[U_i < P_{detect,i}\right], \quad U_i \sim \mathcal{U}(0,1)$$

### Expected Victims Found

Let $\mathcal{T}_i = \{t : d_i(t) \leq r_{acoustic}\}$ be the set of timesteps at which the drone
is within acoustic range of victim $i$. The probability that victim $i$ is detected by time $t$ is:

$$\Pr(\text{victim } i \text{ found by } t) = 1 - \prod_{\tau \leq t,\, \tau \in \mathcal{T}_i}
  \bigl(1 - P_{detect,i}(\mathbf{p}(\tau))\bigr)$$

The expected total number of victims found by time $t$:

$$E[N_{found}](t) = \sum_{i=1}^{3} \Pr(\text{victim } i \text{ found by } t)$$

### Volume Coverage Metric

At each timestep, the explored volume fraction is:

$$\xi(t) = \frac{|\mathcal{V}_{free}(t)|}{|\mathcal{V}_{total\_free}|} \times 100\%$$

where $|\mathcal{V}_{total\_free}|$ is the true count of non-obstacle voxels (known only to the
simulator). Mission coverage at termination quantifies how thoroughly the drone searched the
accessible volume relative to the constraint of finding victims as quickly as possible.

---

## Implementation

```python
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
from scipy.ndimage import binary_dilation, label
import heapq
from collections import defaultdict

# ── Environment constants ─────────────────────────────────────────────────────
ENV_X, ENV_Y, ENV_Z = 20.0, 20.0, 5.0     # m — rubble field dimensions
FILL_RATE   = 0.40                          # fraction of voxels that are obstacle
N_VICTIMS   = 3
MIN_CLEAR   = 0.40                          # m — minimum passage clearance

# ── Sensor parameters ─────────────────────────────────────────────────────────
N_BEAMS     = 120                           # LiDAR horizontal beams
R_LIDAR     = 3.0                           # m — LiDAR max range
SIGMA_R     = 0.03                          # m — range noise std
R_ACOUSTIC  = 2.0                           # m — nominal acoustic range
SIGMA_A     = 0.8                           # m — acoustic detection std

# ── Occupancy grid parameters ─────────────────────────────────────────────────
VOX_RES     = 0.1                           # m — voxel side length
L_OCC       = np.log(0.85 / 0.15)          # log-odds occupied increment
L_FREE      = np.log(0.40 / 0.60)          # log-odds free increment
L_MIN, L_MAX = -10.0, 10.0
P_OCC_THRESH  = 0.70
P_FREE_THRESH = 0.30

# ── Path planning & drone motion ──────────────────────────────────────────────
R_INF       = 0.25                          # m — obstacle inflation radius
V_MAX       = 0.80                          # m/s — drone max speed
DT          = 0.10                          # s — simulation timestep
T_MAX       = 300.0                         # s — mission timeout
K_CONF      = 3                             # detection confirmations required
T_WIN       = 5.0                           # s — confirmation window


# ── Environment generation ────────────────────────────────────────────────────
def generate_rubble(seed=42):
    """
    Build a 3D boolean obstacle grid with FILL_RATE density.
    Enforces connectivity: a flood-fill from the entrance guarantees
    that all victim voxels are reachable through non-obstacle voxels.
    Returns obstacle_grid (NX, NY, NZ) bool array.
    """
    rng = np.random.default_rng(seed)
    NX = int(ENV_X / VOX_RES)
    NY = int(ENV_Y / VOX_RES)
    NZ = int(ENV_Z / VOX_RES)

    while True:
        grid = rng.random((NX, NY, NZ)) < FILL_RATE
        # Enforce entrance column free
        grid[0, NY // 2 - 2 : NY // 2 + 2, :4] = False
        # Place victims at random free voxels and verify connectivity
        victim_idxs = []
        free_idxs = np.argwhere(~grid)
        if len(free_idxs) < N_VICTIMS + 1:
            continue
        chosen = rng.choice(len(free_idxs), size=N_VICTIMS, replace=False)
        victim_idxs = [tuple(free_idxs[c]) for c in chosen]
        # Simple connectivity check via BFS from entrance voxel
        start = (0, NY // 2, 1)
        if _bfs_connected(grid, start, victim_idxs, (NX, NY, NZ)):
            victim_pos = np.array([
                [vi * VOX_RES + VOX_RES / 2 for vi in v] for v in victim_idxs
            ])
            return grid, victim_pos, (NX, NY, NZ)


def _bfs_connected(grid, start, targets, shape):
    """BFS check: can start reach all targets through free voxels?"""
    from collections import deque
    visited = set()
    queue = deque([start])
    visited.add(start)
    targets_set = set(targets)
    found = set()
    NX, NY, NZ = shape
    dirs = [(dx, dy, dz)
            for dx in (-1, 0, 1) for dy in (-1, 0, 1) for dz in (-1, 0, 1)
            if (dx, dy, dz) != (0, 0, 0)]
    while queue:
        cx, cy, cz = queue.popleft()
        if (cx, cy, cz) in targets_set:
            found.add((cx, cy, cz))
        if found == targets_set:
            return True
        for dx, dy, dz in dirs:
            nx, ny, nz = cx + dx, cy + dy, cz + dz
            if (0 <= nx < NX and 0 <= ny < NY and 0 <= nz < NZ
                    and not grid[nx, ny, nz]
                    and (nx, ny, nz) not in visited):
                visited.add((nx, ny, nz))
                queue.append((nx, ny, nz))
    return False


# ── Occupancy grid class ──────────────────────────────────────────────────────
class OccupancyGrid3D:
    def __init__(self, shape):
        NX, NY, NZ = shape
        self.log_odds = np.zeros((NX, NY, NZ), dtype=np.float32)
        self.shape = shape

    def world_to_idx(self, p):
        NX, NY, NZ = self.shape
        ix = int(np.clip(p[0] / VOX_RES, 0, NX - 1))
        iy = int(np.clip(p[1] / VOX_RES, 0, NY - 1))
        iz = int(np.clip(p[2] / VOX_RES, 0, NZ - 1))
        return ix, iy, iz

    def update_ray(self, origin, hit_pt, is_hit=True):
        """Bresenham-3D ray update along origin → hit_pt."""
        direction = hit_pt - origin
        r_total = np.linalg.norm(direction)
        if r_total < 1e-9:
            return
        d_hat = direction / r_total
        r = VOX_RES
        while r < r_total - VOX_RES * 0.5:
            pt = origin + r * d_hat
            ix, iy, iz = self.world_to_idx(pt)
            self.log_odds[ix, iy, iz] = np.clip(
                self.log_odds[ix, iy, iz] + L_FREE, L_MIN, L_MAX)
            r += VOX_RES
        if is_hit:
            ix, iy, iz = self.world_to_idx(hit_pt)
            self.log_odds[ix, iy, iz] = np.clip(
                self.log_odds[ix, iy, iz] + L_OCC, L_MIN, L_MAX)

    def probability(self):
        return 1.0 / (1.0 + np.exp(-self.log_odds))

    def classify(self):
        p = self.probability()
        occ = p > P_OCC_THRESH
        free = p < P_FREE_THRESH
        return occ, free

    def inflated_occupied(self):
        occ, _ = self.classify()
        n_inf = int(np.ceil(R_INF / VOX_RES))
        struct = np.zeros((2*n_inf+1,)*3, dtype=bool)
        cx = cy = cz = n_inf
        for dx in range(-n_inf, n_inf+1):
            for dy in range(-n_inf, n_inf+1):
                for dz in range(-n_inf, n_inf+1):
                    if dx**2 + dy**2 + dz**2 <= n_inf**2:
                        struct[cx+dx, cy+dy, cz+dz] = True
        return binary_dilation(occ, structure=struct)

    def frontiers(self):
        """Return world-space centroids of frontier clusters."""
        occ, free = self.classify()
        unknown = ~occ & ~free
        free_dilated = binary_dilation(free, structure=np.ones((3, 3, 3), dtype=bool))
        frontier_mask = unknown & free_dilated
        if not np.any(frontier_mask):
            return []
        labeled, n_clusters = label(frontier_mask)
        centroids = []
        for c in range(1, n_clusters + 1):
            voxels = np.argwhere(labeled == c)
            centroid_idx = voxels.mean(axis=0)
            centroid_w = centroid_idx * VOX_RES + VOX_RES / 2
            size = len(voxels)
            centroids.append((centroid_w, size))
        return centroids


# ── 3D A* planner ─────────────────────────────────────────────────────────────
def astar_3d(start_idx, goal_idx, blocked):
    """
    26-connected 3D A* on the inflated occupancy grid.
    blocked: boolean array (NX, NY, NZ), True = impassable.
    Returns list of (ix, iy, iz) index tuples or [] if no path found.
    """
    NX, NY, NZ = blocked.shape
    DIRS = [(dx, dy, dz)
            for dx in (-1, 0, 1) for dy in (-1, 0, 1) for dz in (-1, 0, 1)
            if (dx, dy, dz) != (0, 0, 0)]

    def h(a, b):
        return np.sqrt(sum((a[i] - b[i])**2 for i in range(3))) * VOX_RES

    open_heap = []
    heapq.heappush(open_heap, (h(start_idx, goal_idx), 0.0, start_idx, None))
    came_from = {}
    g_score = defaultdict(lambda: np.inf)
    g_score[start_idx] = 0.0

    while open_heap:
        f, g, current, parent = heapq.heappop(open_heap)
        if current in came_from:
            continue
        came_from[current] = parent
        if current == goal_idx:
            # Reconstruct path
            path = []
            node = current
            while node is not None:
                path.append(node)
                node = came_from[node]
            return path[::-1]
        cx, cy, cz = current
        for dx, dy, dz in DIRS:
            nx, ny, nz = cx + dx, cy + dy, cz + dz
            if not (0 <= nx < NX and 0 <= ny < NY and 0 <= nz < NZ):
                continue
            if blocked[nx, ny, nz]:
                continue
            step_cost = np.sqrt(dx**2 + dy**2 + dz**2) * VOX_RES
            ng = g + step_cost
            neighbour = (nx, ny, nz)
            if ng < g_score[neighbour]:
                g_score[neighbour] = ng
                heapq.heappush(open_heap,
                               (ng + h(neighbour, goal_idx), ng, neighbour, current))
    return []  # no path found


# ── LiDAR sensor simulation ────────────────────────────────────────────────────
def simulate_lidar(pos, obstacle_grid, rng):
    """
    Cast N_BEAMS horizontal rays from pos. Returns (N_BEAMS, 3) hit points.
    """
    NX, NY, NZ = obstacle_grid.shape
    azimuths = np.linspace(0, 2 * np.pi, N_BEAMS, endpoint=False)
    hits = []
    for phi in azimuths:
        d = np.array([np.cos(phi), np.sin(phi), 0.0])
        r = VOX_RES
        hit_pt = pos + R_LIDAR * d
        while r <= R_LIDAR:
            pt = pos + r * d
            ix = int(np.clip(pt[0] / VOX_RES, 0, NX - 1))
            iy = int(np.clip(pt[1] / VOX_RES, 0, NY - 1))
            iz = int(np.clip(pt[2] / VOX_RES, 0, NZ - 1))
            if obstacle_grid[ix, iy, iz]:
                r_noisy = r + rng.normal(0, SIGMA_R)
                r_noisy = np.clip(r_noisy, VOX_RES, R_LIDAR)
                hit_pt = pos + r_noisy * d
                break
            r += VOX_RES
        hits.append(hit_pt)
    return np.array(hits)


# ── Acoustic detection ────────────────────────────────────────────────────────
def acoustic_detect(pos, victim_positions, rng):
    """
    Sample acoustic observations for each victim. Returns list of bools.
    """
    detections = []
    for vp in victim_positions:
        d = np.linalg.norm(pos - vp)
        p_det = np.exp(-d**2 / (2 * SIGMA_A**2))
        detections.append(bool(rng.random() < p_det))
    return detections


# ── Main simulation ────────────────────────────────────────────────────────────
def run_simulation(seed=42):
    rng = np.random.default_rng(seed)

    # Build environment
    obstacle_grid, victim_pos, shape = generate_rubble(seed=seed)
    NX, NY, NZ = shape
    grid = OccupancyGrid3D(shape)

    # Drone starts at entrance
    pos = np.array([0.5, ENV_Y / 2, 0.3])
    traj = [pos.copy()]

    # Victim tracking
    confirm_counts = [0] * N_VICTIMS
    victim_found   = [False] * N_VICTIMS
    detect_history = [[] for _ in range(N_VICTIMS)]  # (t, bool) per victim

    coverage_log = []   # (t, xi)
    victim_log   = []   # (t, N_found)
    expected_log = []   # (t, E[N_found])

    # Frontier planner state
    current_path = []
    path_idx = 0
    t = 0.0
    n_found_cumulative = 0

    total_free = float(np.sum(~obstacle_grid))

    while t < T_MAX:
        # ── LiDAR update ──────────────────────────────────────────────────────
        hits = simulate_lidar(pos, obstacle_grid, rng)
        for h in hits:
            grid.update_ray(pos, h)

        # ── Acoustic update ───────────────────────────────────────────────────
        detections = acoustic_detect(pos, victim_pos, rng)
        for i, det in enumerate(detections):
            if not victim_found[i]:
                detect_history[i].append((t, det))
                # Count positives in sliding window
                recent = [d for (ts, d) in detect_history[i] if t - ts <= T_WIN]
                if sum(recent) >= K_CONF:
                    victim_found[i] = True
                    n_found_cumulative += 1
                    print(f"[t={t:.1f}s] Victim {i+1} FOUND at {victim_pos[i]}")

        # ── Path planning: re-plan if needed ─────────────────────────────────
        if not current_path or path_idx >= len(current_path):
            inflated = grid.inflated_occupied()
            frontiers = grid.frontiers()
            if not frontiers:
                print(f"[t={t:.1f}s] No frontiers. Mission complete.")
                break
            # Select nearest large frontier
            start_idx = tuple(
                int(np.clip(pos[k] / VOX_RES, 0, shape[k] - 1)) for k in range(3)
            )
            best = min(
                frontiers,
                key=lambda fw: np.linalg.norm(fw[0] - pos) / np.sqrt(fw[1])
            )
            goal_w = best[0]
            goal_idx = tuple(
                int(np.clip(goal_w[k] / VOX_RES, 0, shape[k] - 1)) for k in range(3)
            )
            path = astar_3d(start_idx, goal_idx, inflated)
            if path:
                current_path = [
                    np.array([idx[k] * VOX_RES + VOX_RES / 2 for k in range(3)])
                    for idx in path
                ]
                path_idx = 1  # skip start voxel

        # ── Drone motion ──────────────────────────────────────────────────────
        if current_path and path_idx < len(current_path):
            waypoint = current_path[path_idx]
            direction = waypoint - pos
            dist = np.linalg.norm(direction)
            step = min(V_MAX * DT, dist)
            if dist > 1e-9:
                pos = pos + step * direction / dist
            if dist < VOX_RES * 0.6:
                path_idx += 1

        t += DT
        traj.append(pos.copy())

        # ── Logging ───────────────────────────────────────────────────────────
        if int(t / 2.0) != int((t - DT) / 2.0):
            occ, free_vox = grid.classify()
            xi = np.sum(free_vox) / total_free * 100.0
            coverage_log.append((t, xi))
            victim_log.append((t, n_found_cumulative))

            # Expected detections (analytic)
            e_found = sum(
                1.0 - np.prod([
                    1.0 - np.exp(-np.linalg.norm(
                        np.array(traj[step_k]) - victim_pos[i])**2 / (2*SIGMA_A**2))
                    for step_k in range(len(traj))
                    if np.linalg.norm(np.array(traj[step_k]) - victim_pos[i]) <= R_ACOUSTIC
                ]) if any(
                    np.linalg.norm(np.array(traj[step_k]) - victim_pos[i]) <= R_ACOUSTIC
                    for step_k in range(len(traj))
                ) else 0.0
                for i in range(N_VICTIMS)
            )
            expected_log.append((t, e_found))

        # Early exit if all found
        if all(victim_found):
            print(f"[t={t:.1f}s] All {N_VICTIMS} victims found. Mission success!")
            break

    traj = np.array(traj)
    T_mission = t
    final_coverage = coverage_log[-1][1] if coverage_log else 0.0
    print(f"Victims found    : {n_found_cumulative} / {N_VICTIMS}")
    print(f"Volume coverage  : {final_coverage:.1f} %")
    print(f"Mission time     : {T_mission:.1f} s")

    return (traj, obstacle_grid, victim_pos, victim_found,
            coverage_log, victim_log, expected_log, shape, T_mission)


# ── Visualisation ─────────────────────────────────────────────────────────────
def plot_results(traj, obstacle_grid, victim_pos, victim_found,
                 coverage_log, victim_log, expected_log, shape, T_mission):
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

    fig = plt.figure(figsize=(18, 11))

    # ── Plot 1: 3D rubble field + trajectory + victims ─────────────────────
    ax1 = fig.add_subplot(2, 3, 1, projection='3d')
    occ_idx = np.argwhere(obstacle_grid)
    # Subsample for speed
    step_occ = max(1, len(occ_idx) // 3000)
    occ_w = occ_idx[::step_occ] * VOX_RES
    ax1.scatter(occ_w[:, 0], occ_w[:, 1], occ_w[:, 2],
                s=1, c='grey', alpha=0.15, label='Rubble')
    ax1.plot(traj[:, 0], traj[:, 1], traj[:, 2],
             'r-', lw=1.0, alpha=0.8, label='Drone path')
    for i, vp in enumerate(victim_pos):
        color = 'lime' if victim_found[i] else 'cyan'
        marker = '*' if victim_found[i] else 'o'
        ax1.scatter(*vp, s=120, c=color, marker=marker, edgecolors='k',
                    linewidths=0.8, label=f'Victim {i+1}' + (' ✓' if victim_found[i] else ''))
    ax1.scatter(*traj[0], s=80, c='red', marker='^', label='Start')
    ax1.set_xlabel('X (m)'); ax1.set_ylabel('Y (m)'); ax1.set_zlabel('Z (m)')
    ax1.set_title('3D Rubble Field + Exploration Path')
    ax1.legend(fontsize=6, loc='upper right')

    # ── Plot 2: Top-down XY slice ─────────────────────────────────────────
    ax2 = fig.add_subplot(2, 3, 2)
    NX, NY, NZ = shape
    z_slice = int(np.clip(0.5 / VOX_RES, 0, NZ - 1))
    occ_slice = obstacle_grid[:, :, z_slice].T
    ax2.imshow(occ_slice, origin='lower', cmap='Greys', alpha=0.6,
               extent=[0, ENV_X, 0, ENV_Y], aspect='equal')
    ax2.plot(traj[:, 0], traj[:, 1], 'r-', lw=0.8, alpha=0.7)
    for i, vp in enumerate(victim_pos):
        color = 'lime' if victim_found[i] else 'cyan'
        ax2.scatter(vp[0], vp[1], s=120, c=color, edgecolors='k',
                    zorder=5, label=f'Victim {i+1}')
    ax2.scatter(traj[0, 0], traj[0, 1], s=80, c='red', marker='^', label='Start')
    ax2.set_xlabel('X (m)'); ax2.set_ylabel('Y (m)')
    ax2.set_title('Top-Down View (z ≈ 0.5 m slice)')
    ax2.legend(fontsize=6); ax2.grid(True, alpha=0.3)

    # ── Plot 3: Volume coverage vs time ──────────────────────────────────
    ax3 = fig.add_subplot(2, 3, 3)
    if coverage_log:
        t_cov = [v[0] for v in coverage_log]
        xi_cov = [v[1] for v in coverage_log]
        ax3.plot(t_cov, xi_cov, 'b-', lw=1.5)
        ax3.fill_between(t_cov, xi_cov, alpha=0.15, color='blue')
        ax3.axhline(80, color='orange', ls='--', lw=1.0, label='80 % threshold')
        ax3.axhline(90, color='red', ls='--', lw=1.0, label='90 % threshold')
    ax3.set_xlabel('Time (s)'); ax3.set_ylabel('Coverage (%)')
    ax3.set_title('Explored Volume vs Time')
    ax3.legend(fontsize=7); ax3.grid(True)

    # ── Plot 4: Victims found (actual vs expected) ─────────────────────
    ax4 = fig.add_subplot(2, 3, 4)
    if victim_log:
        t_vic = [v[0] for v in victim_log]
        n_vic = [v[1] for v in victim_log]
        t_exp = [v[0] for v in expected_log]
        e_vic = [v[1] for v in expected_log]
        ax4.step(t_vic, n_vic, where='post', color='darkgreen', lw=1.5,
                 label='Actual $N_{found}$')
        ax4.plot(t_exp, e_vic, 'b--', lw=1.2, label=r'$E[N_{found}]$ analytic')
        ax4.axhline(N_VICTIMS, color='grey', ls=':', lw=1.0, label='All found')
    ax4.set_xlabel('Time (s)'); ax4.set_ylabel('Victims found')
    ax4.set_yticks([0, 1, 2, 3])
    ax4.set_title('Victim Detection: Actual vs Expected')
    ax4.legend(fontsize=7); ax4.grid(True)

    # ── Plot 5: Detection probability surface ─────────────────────────
    ax5 = fig.add_subplot(2, 3, 5)
    d_vals = np.linspace(0, 4.0, 200)
    p_det  = np.exp(-d_vals**2 / (2 * SIGMA_A**2))
    ax5.plot(d_vals, p_det, 'b-', lw=1.8, label=r'$P_{detect}(d)$')
    ax5.axvline(R_ACOUSTIC, color='red', ls='--', lw=1.2, label=f'$r_{{acoustic}}={R_ACOUSTIC}$ m')
    ax5.axhline(np.exp(-R_ACOUSTIC**2 / (2 * SIGMA_A**2)), color='orange',
                ls=':', lw=1.0, label=f'$P$ at $r_{{acoustic}}$')
    ax5.fill_between(d_vals, p_det, alpha=0.12, color='blue')
    ax5.set_xlabel('Distance $d$ (m)'); ax5.set_ylabel('$P_{detect}$')
    ax5.set_title('Acoustic Detection Probability vs Distance')
    ax5.legend(fontsize=7); ax5.grid(True)

    # ── Plot 6: Metrics summary bar chart ─────────────────────────────
    ax6 = fig.add_subplot(2, 3, 6)
    metrics = {
        'Victims\nfound': sum(victim_found) / N_VICTIMS * 100,
        'Volume\ncoverage': coverage_log[-1][1] if coverage_log else 0.0,
        'Time\nutilisation': min(T_mission / T_MAX * 100, 100),
    }
    colors = ['#2ecc71', '#3498db', '#e67e22']
    bars = ax6.bar(list(metrics.keys()), list(metrics.values()),
                   color=colors, edgecolor='grey', width=0.5)
    for bar, val in zip(bars, metrics.values()):
        ax6.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 1,
                 f'{val:.1f}%', ha='center', va='bottom', fontsize=9)
    ax6.set_ylabel('Percentage (%)')
    ax6.set_ylim(0, 115)
    ax6.set_title('Mission Metrics Summary')
    ax6.grid(True, axis='y', alpha=0.5)

    plt.suptitle(
        f'S093 Earthquake Rubble SAR  |  T_mission = {T_mission:.0f} s  |  '
        f'Victims = {sum(victim_found)}/{N_VICTIMS}', fontsize=12
    )
    plt.tight_layout()
    plt.savefig('outputs/05_special_entertainment/s093_rubble_sar/s093_analysis.png',
                dpi=150, bbox_inches='tight')
    plt.show()


def animate_exploration(traj, obstacle_grid, victim_pos, victim_found, shape, out_path):
    """
    Top-down 2D animation of drone exploring rubble field.
    Drone marker turns green as each victim is confirmed found.
    """
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

    NX, NY, NZ = shape
    z_slice = int(np.clip(0.5 / VOX_RES, 0, NZ - 1))
    occ_slice = obstacle_grid[:, :, z_slice].T

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.imshow(occ_slice, origin='lower', cmap='Greys', alpha=0.55,
              extent=[0, ENV_X, 0, ENV_Y], aspect='equal')
    for i, vp in enumerate(victim_pos):
        color = 'lime' if victim_found[i] else 'cyan'
        ax.scatter(vp[0], vp[1], s=150, c=color, edgecolors='k',
                   marker='*', zorder=6, label=f'Victim {i+1}')

    line, = ax.plot([], [], 'r-', lw=0.9, alpha=0.7, label='Drone path')
    drone_dot, = ax.plot([], [], 'ro', ms=7, zorder=7)
    time_txt = ax.text(0.02, 0.97, '', transform=ax.transAxes, fontsize=9,
                       va='top', color='black')

    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
    ax.set_title('S093 Rubble SAR — Frontier Exploration')
    ax.legend(fontsize=7, loc='lower right')

    step = max(1, len(traj) // 250)

    def init():
        line.set_data([], [])
        drone_dot.set_data([], [])
        time_txt.set_text('')
        return line, drone_dot, time_txt

    def update(frame):
        k = min(frame * step, len(traj) - 1)
        line.set_data(traj[:k, 0], traj[:k, 1])
        drone_dot.set_data([traj[k, 0]], [traj[k, 1]])
        time_txt.set_text(f't = {k * DT:.1f} s')
        return line, drone_dot, time_txt

    n_frames = len(traj) // step
    ani = animation.FuncAnimation(fig, update, frames=n_frames,
                                  init_func=init, blit=True, interval=60)
    ani.save(out_path, writer='pillow', fps=15)
    plt.close(fig)
    print(f"Animation saved to {out_path}")


if __name__ == '__main__':
    import os
    out_dir = 'outputs/05_special_entertainment/s093_rubble_sar'
    os.makedirs(out_dir, exist_ok=True)

    (traj, obstacle_grid, victim_pos, victim_found,
     coverage_log, victim_log, expected_log, shape, T_mission) = run_simulation(seed=42)

    plot_results(traj, obstacle_grid, victim_pos, victim_found,
                 coverage_log, victim_log, expected_log, shape, T_mission)

    animate_exploration(traj, obstacle_grid, victim_pos, victim_found, shape,
                        out_path=f'{out_dir}/s093_exploration.gif')
```

---

## Key Parameters

| Parameter | Symbol | Value |
|-----------|--------|-------|
| Rubble field dimensions | — | $20 \times 20 \times 5$ m |
| Voxel resolution | $\Delta_{vox}$ | 0.1 m |
| Obstacle fill rate | — | 40 % |
| Minimum passage clearance | $d_{clear}$ | 0.4 m |
| Number of victims | $N_v$ | 3 |
| Drone body radius | $r_{drone}$ | 0.15 m |
| Obstacle inflation radius | $r_{inf}$ | 0.25 m |
| LiDAR beams | $N_{beams}$ | 120 |
| LiDAR max range | $r_{lidar}$ | 3.0 m |
| LiDAR range noise std | $\sigma_r$ | 0.03 m |
| Acoustic nominal range | $r_{acoustic}$ | 2.0 m |
| Acoustic detection std | $\sigma_a$ | 0.8 m |
| Occupied log-odds increment | $\ell_{occ}$ | $\log(0.85/0.15) \approx 1.73$ |
| Free log-odds increment | $\ell_{free}$ | $\log(0.40/0.60) \approx -0.41$ |
| Log-odds clamp | $[\ell_{min}, \ell_{max}]$ | $[-10, 10]$ |
| Occupied threshold | $P_{occ}$ | 0.70 |
| Free threshold | $P_{free}$ | 0.30 |
| Detection confirmations required | $K_{conf}$ | 3 |
| Confirmation window | $T_{win}$ | 5.0 s |
| Drone max speed | $v_{max}$ | 0.8 m/s |
| Simulation timestep | $\Delta t$ | 0.1 s |
| Mission timeout | $T_{max}$ | 300 s |

---

## Expected Output

- **3D rubble field + trajectory** (`s093_analysis.png`, upper-left panel): sparse scatter of grey
  obstacle voxels (subsampled for rendering); drone path as a red 3D line; victim markers as
  stars (lime green = confirmed found, cyan = not yet found); mission start marked with a red
  triangle.
- **Top-down XY slice** (upper-centre panel): 2D binary rubble map at $z \approx 0.5$ m;
  drone trajectory overlaid; victim positions shown; reveals how the frontier planner routes
  through narrow passages rather than straight-line paths.
- **Volume coverage vs time** (upper-right panel): monotonically increasing $\xi(t)$ curve with
  80 % and 90 % threshold dashed lines; curve shape depends on how cluttered the rubble field is
  — steeply rising near the entrance where passage density is higher, plateauing in dead-end pockets.
- **Victim detection: actual vs expected** (lower-left panel): step function for confirmed victim
  count $N_{found}(t)$ (dark green); smooth analytic curve for $E[N_{found}](t)$ (blue dashed);
  the analytic curve rises whenever the drone enters acoustic range and asymptotes toward the true
  count.
- **Acoustic detection probability curve** (lower-centre panel): $P_{detect}(d) = \exp(-d^2 /
  2\sigma_a^2)$ vs distance; vertical red dashed line at $r_{acoustic} = 2$ m; horizontal orange
  dotted line at the detection probability at that range; clearly shows the soft detection zone.
- **Mission metrics summary bar chart** (lower-right panel): three bars showing victims found (%
  of 3), volume coverage (%), and time utilisation ($T_{mission}/T_{max} \times 100\%$); colour-coded
  green / blue / orange.
- **Exploration animation** (`s093_exploration.gif`): 15 fps top-down view; drone position as a
  red circle moving through the rubble slice; path tail drawn incrementally; victim markers
  pre-plotted; time counter in the upper-left corner.

**Typical metric values** (seed = 42):

| Metric | Expected value |
|--------|----------------|
| Victims found | 3 / 3 |
| Volume coverage $\xi$ | 75–90 % |
| Mission time $T_{mission}$ | 180–270 s |

---

## Extensions

1. **Multi-drone cooperative SAR**: deploy 3 micro-drones entering from different perimeter points;
   share occupancy grids via mesh radio (merge voxel log-odds by summation when drones are within
   $r_{comm} = 5$ m); compare single-drone vs 3-drone victim-find time and coverage.
2. **Victim prioritisation with signal strength gradient**: instead of binary confirmation, estimate
   the victim's 3D position by gradient ascent on the acoustic signal field; plan a direct intercept
   path once the gradient is sufficiently reliable; compare localisation error vs confirmation-count
   threshold.
3. **Structural collapse propagation**: simulate secondary collapses at random intervals
   ($\lambda = 0.005$ events/s); re-classify newly blocked voxels and replan A* on the updated
   inflated grid; assess how collapse frequency degrades coverage and victim-find rate.
4. **3D LiDAR (vertical fan)**: replace horizontal-only scanning with a 16-beam vertical fan
   ($\pm 30°$ elevation); update the occupancy grid in full 3D volume rather than a single scan
   plane; compare coverage speed and map accuracy against the horizontal-scan baseline.
5. **Energy-aware planning**: model battery capacity as $E_{max} = 500$ Wh; assign per-step energy
   cost proportional to speed and altitude change; add a return-to-base constraint; study the
   trade-off between aggressive exploration and ensuring sufficient energy to return.

---

## Related Scenarios

- Prerequisites: [S043 Post-Disaster Mapping](../03_environmental_sar/S043_post_disaster.md), [S074 Mine 3D Mapping](../04_industrial_agriculture/S074_mine_mapping.md)
- Follow-ups: [S094 Rubble Multi-Agent SAR](S094_rubble_multiagent.md)
- Algorithmic cross-reference: [S050 Swarm Cooperative Mapping (EKF-SLAM)](../03_environmental_sar/S050_slam.md) (frontier exploration), [S013 Particle Filter Intercept](../01_pursuit_evasion/S013_particle_filter_intercept.md) (probabilistic detection models)
