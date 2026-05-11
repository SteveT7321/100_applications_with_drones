# S054 Minefield Detection

**Domain**: Environmental Monitoring & SAR | **Difficulty**: ⭐⭐⭐ | **Status**: `[ ]` Not Started
**Algorithm**: Bayesian Occupancy Grid + Risk-Aware A* | **Dimension**: 2D

---

## Problem Definition

**Setup**: A 50 × 50 m area is suspected to contain $M = 15$ anti-personnel mines at unknown
positions. Before any personnel can safely cross, a single drone equipped with a downward-facing
metal-detector sensor (circular footprint radius $r_s = 1.5$ m) must scan the entire area, build
a probabilistic mine map, and then compute a human-safe extraction path from an entry point at the
southern edge to an exit point at the northern edge.

The area is discretised into a $100 \times 100$ grid of $0.5$ m cells. The drone first executes an
**initial lawnmower sweep** (boustrophedon pattern with row spacing $2\,r_s$) that provides
near-complete sensor coverage. At each waypoint the sensor reports a binary detection: **hit**
(mine signal detected) or **miss** (no signal). These observations are fused into a shared
**Bayesian occupancy grid** — a per-cell posterior probability $P_{mine}(x, y)$ — via Bayes'
theorem. After the sweep the posterior risk map is used by a **risk-aware A\*** path planner to
route a person through the area while minimising the expected number of mines encountered along
the path.

**Roles**:
- **Drone**: single agent; flies the lawnmower sweep at constant altitude $h = 3$ m and cruise
  speed $v_d = 2.0$ m/s; reports a binary observation after each sensor dwell; feeds all
  observations into the shared occupancy grid.
- **Occupancy grid**: $100 \times 100$ array of mine-presence posteriors, initialised with a
  uniform prior $P_0 = M / N_c$ ($N_c = 10\,000$ cells); updated by Bayes' theorem after every
  drone observation.
- **Risk-aware A\***: graph-search planner that finds the path of minimum cumulative mine risk
  from entry to exit, comparing the result against the straight-line (shortest) path.

**Comparison strategies**:
1. **Direct path** — straight line from entry to exit, ignoring the risk map; baseline.
2. **Risk-minimised A\*** — A\* with edge costs equal to the posterior mine probability of each
   traversed cell; finds the globally optimal low-risk corridor.
3. **Threshold-cleared path** — A\* restricted to cells with posterior $P_{mine} < 0.10$;
   equivalent to treating any cell above the threshold as an impassable obstacle.

**Objective**: Minimise the **expected number of mines hit** along the extraction path,
$\mathbb{E}[\text{hits}] = \sum_{\text{cells on path}} P_{mine}(\text{cell})$, while keeping
path length reasonably short (penalise detours exceeding $3\times$ the direct distance).

---

## Mathematical Model

### Mine Prior

Let $\mathcal{G}$ be the set of all $N_c = 100 \times 100 = 10\,000$ grid cells, each with centre
$\mathbf{x} = (x, y)$. The prior probability that cell $\mathbf{x}$ contains a mine is:

$$P_0(\mathbf{x}) = \frac{M}{N_c} = \frac{15}{10\,000} = 0.0015$$

The prior is uniform (no terrain information); all cells are treated as equally likely mine hosts.

### Sensor (Likelihood) Model

The metal-detector sensor at drone position $\mathbf{p}_d$ produces observation $z \in \{0, 1\}$
(miss / hit) with footprint radius $r_s$. Detection and false-alarm probabilities:

$$P(z = 1 \mid \text{mine at } \mathbf{x}) =
\begin{cases}
P_d   & \text{if } \|\mathbf{x} - \mathbf{p}_d\| \leq r_s \\
P_{fa} & \text{otherwise}
\end{cases}$$

with $P_d = 0.85$ (15 % miss rate) and $P_{fa} = 0.05$ (5 % false-alarm rate within the scan
strip). The full likelihood for observation $z$ given cell hypothesis $H_{\mathbf{x}}$ (mine at
$\mathbf{x}$) and $\bar{H}_{\mathbf{x}}$ (no mine):

$$P(z \mid H_{\mathbf{x}}) =
\begin{cases}
P_d \cdot \mathbf{1}[\|\mathbf{x}-\mathbf{p}_d\|\leq r_s] + P_{fa} \cdot \mathbf{1}[\|\mathbf{x}-\mathbf{p}_d\|>r_s]
  & z = 1 \\[4pt]
(1-P_d) \cdot \mathbf{1}[\|\mathbf{x}-\mathbf{p}_d\|\leq r_s] + (1-P_{fa}) \cdot \mathbf{1}[\|\mathbf{x}-\mathbf{p}_d\|>r_s]
  & z = 0
\end{cases}$$

### Bayesian Occupancy Grid Update

Each cell maintains an independent marginal mine probability. Let $P_t(\mathbf{x})$ be the
posterior at time $t$. After the drone reports observation $z$ at position $\mathbf{p}_d$:

$$P_{t+1}(\mathbf{x}) =
\frac{P(z \mid H_{\mathbf{x}}) \cdot P_t(\mathbf{x})}
     {P(z \mid H_{\mathbf{x}}) \cdot P_t(\mathbf{x})
      + P(z \mid \bar{H}_{\mathbf{x}}) \cdot [1 - P_t(\mathbf{x})]}$$

Because cells are treated as independent, each cell's posterior is updated in a single scalar
operation. The sensor footprint determines which likelihood branch applies:

$$P(z = 1 \mid \bar{H}_{\mathbf{x}}) =
\begin{cases}
P_{fa}   & \|\mathbf{x} - \mathbf{p}_d\| \leq r_s \\
0        & \text{otherwise}
\end{cases}$$

For a cell entirely outside the sensor footprint and with a miss observation ($z = 0$), the
likelihood ratio is 1 and the posterior is unchanged — only footprint cells carry information.

### Log-Odds Representation

For numerical stability, the update is implemented in log-odds form. Define:

$$l_t(\mathbf{x}) = \log \frac{P_t(\mathbf{x})}{1 - P_t(\mathbf{x})}$$

Then the Bayesian update becomes an additive correction:

$$l_{t+1}(\mathbf{x}) = l_t(\mathbf{x}) + \log \frac{P(z \mid H_{\mathbf{x}})}{P(z \mid \bar{H}_{\mathbf{x}})}$$

The posterior probability is recovered via:

$$P_{t+1}(\mathbf{x}) = \frac{1}{1 + e^{-l_{t+1}(\mathbf{x})}}$$

Log-odds bounds are clamped to $[-10,\, 10]$ to prevent numerical saturation.

### Lawnmower Sweep Coverage

The drone scans in a boustrophedon (row-by-row) pattern with row spacing $d_{row} = 2\,r_s = 3$ m,
ensuring every point in the area is within $r_s$ of at least one waypoint. The number of scan rows:

$$N_{rows} = \left\lceil \frac{L_{area}}{d_{row}} \right\rceil = \left\lceil \frac{50}{3} \right\rceil = 17$$

Total sweep path length:

$$L_{sweep} = N_{rows} \cdot L_{area} + (N_{rows} - 1) \cdot d_{row}
            = 17 \times 50 + 16 \times 3 = 898 \text{ m}$$

Coverage completeness (fraction of cells within $r_s$ of at least one waypoint):

$$C_{sweep} = 1 - \left(1 - \frac{\pi r_s^2}{d_{row}^2}\right)^{N_{scans}} \approx 0.97$$

### Risk-Aware A* Path Planning

After the sweep, the posterior $P_{mine}(\mathbf{x})$ forms a **risk map**. A\* finds the
minimum-risk path on the 2D cell grid (8-connected neighbourhood).

**Node cost** $g(n)$: accumulated mine risk along the path from the entry cell to node $n$:

$$g(n) = \sum_{\mathbf{x} \in \text{path}(entry \to n)} P_{mine}(\mathbf{x}) \cdot \Delta l(\mathbf{x})$$

where $\Delta l(\mathbf{x})$ is the step length (1 cell for cardinal moves, $\sqrt{2}$ cells for
diagonal moves), normalised so that the integral approximates a path-length-weighted risk.

**Heuristic** $h(n)$: admissible lower bound using straight-line distance to the goal, scaled by
the minimum per-cell risk (which is $\approx 0$ for cleared cells):

$$h(n) = \|\mathbf{x}_n - \mathbf{x}_{goal}\|_2 \cdot P_{min}$$

Because $P_{min} \leq P_{mine}(\mathbf{x})$ for all unvisited cells, $h(n)$ is admissible and
A\* returns the globally optimal risk path. In practice $P_{min} = 0$ (some cells have negligible
risk), so the heuristic degenerates to zero and A\* behaves like Dijkstra; a weighted variant
$h(n) = w \cdot \|\mathbf{x}_n - \mathbf{x}_{goal}\|_2$ with $w = \bar{P}_{mine}$ (mean posterior)
is used to speed convergence.

**Priority queue** ordering on $f(n) = g(n) + h(n)$; ties broken by path length to prefer shorter
paths of equal risk.

### Expected Mines Hit Along a Path

For a discrete path $\pi = [\mathbf{x}_1, \mathbf{x}_2, \ldots, \mathbf{x}_K]$ (cell centres),
treating mine presence at each cell as an independent Bernoulli random variable:

$$\mathbb{E}[\text{hits}(\pi)] = \sum_{k=1}^{K} P_{mine}(\mathbf{x}_k)$$

The **path risk integral** (continuous approximation) integrating risk per unit length:

$$R(\pi) = \int_{\pi} P_{mine}(\mathbf{x}(l))\, dl \approx \sum_{k=1}^{K} P_{mine}(\mathbf{x}_k) \cdot \Delta l_k$$

where $\Delta l_k$ is the arc-length of step $k$.

### Path Risk Reduction

The risk reduction achieved by the A\* planner relative to the direct path:

$$\Delta R = R(\pi_{direct}) - R(\pi_{A^*})$$

$$\text{Risk reduction ratio} = \frac{\Delta R}{R(\pi_{direct})} \times 100\%$$

A higher ratio indicates greater value of the sweep-and-plan approach vs. blind crossing.

---

## Implementation

```python
import numpy as np
import matplotlib.pyplot as plt
import heapq
from matplotlib.patches import Circle

# ── Key constants ──────────────────────────────────────────────────
AREA_SIZE     = 50.0      # m, square area side
GRID_RES      = 0.5       # m per cell
GRID_N        = int(AREA_SIZE / GRID_RES)   # 100 cells per side
N_MINES       = 15        # total mines in area
SENSOR_RADIUS = 1.5       # m, metal-detector footprint radius
DRONE_SPEED   = 2.0       # m/s
SCAN_HEIGHT   = 3.0       # m (informational)

P_D           = 0.85      # P(hit | mine in footprint)
P_FA          = 0.05      # P(hit | no mine in footprint)
P_PRIOR       = N_MINES / (GRID_N ** 2)     # uniform mine prior ≈ 0.0015

LOG_ODDS_MIN  = -10.0     # clamp bounds for log-odds
LOG_ODDS_MAX  =  10.0
ROW_SPACING   = 2.0 * SENSOR_RADIUS         # lawnmower row separation (m)

# Entry / exit (cell indices)
ENTRY_CELL    = (0,      GRID_N // 2)       # south edge, centre column
EXIT_CELL     = (GRID_N - 1, GRID_N // 2)  # north edge, centre column

# ── Grid cell centres ───────────────────────────────────────────────
xs = np.arange(0.5 * GRID_RES, AREA_SIZE, GRID_RES)   # shape (100,)
ys = np.arange(0.5 * GRID_RES, AREA_SIZE, GRID_RES)
XX, YY = np.meshgrid(xs, ys)                            # shape (100, 100)

def cell_to_pos(r, c):
    """Convert (row, col) cell index to world-space (x, y) coordinates."""
    return np.array([xs[c], ys[r]])

# ── Mine placement ──────────────────────────────────────────────────
def place_mines(seed=42):
    rng = np.random.default_rng(seed)
    idxs = rng.choice(GRID_N ** 2, size=N_MINES, replace=False)
    mine_grid = np.zeros((GRID_N, GRID_N), dtype=bool)
    rows, cols = np.unravel_index(idxs, (GRID_N, GRID_N))
    mine_grid[rows, cols] = True
    return mine_grid

# ── Log-odds initialisation ─────────────────────────────────────────
def init_log_odds():
    """Uniform prior in log-odds form."""
    p0 = P_PRIOR
    return np.full((GRID_N, GRID_N), np.log(p0 / (1.0 - p0)))

def log_odds_to_prob(log_odds):
    return 1.0 / (1.0 + np.exp(-log_odds))

# ── Bayesian update ─────────────────────────────────────────────────
def bayes_update(log_odds, drone_pos, hit, mine_grid=None):
    """
    Update the log-odds grid after one binary observation.
    drone_pos: (x, y) world coordinates of drone.
    hit: True = z=1 (detection), False = z=0 (miss).
    mine_grid: (optional) true mine locations, used only for ground-truth
               sensor simulation — not used inside the filter update.
    """
    dist = np.sqrt((XX - drone_pos[0])**2 + (YY - drone_pos[1])**2)
    in_fp = dist <= SENSOR_RADIUS     # cells inside sensor footprint

    if hit:
        # P(z=1 | H) for footprint cells = P_D; outside = P_FA
        lik_mine    = np.where(in_fp, P_D,       P_FA)
        lik_no_mine = np.where(in_fp, P_FA,      0.0)
    else:
        # P(z=0 | H) for footprint cells = 1-P_D; outside = 1-P_FA
        lik_mine    = np.where(in_fp, 1.0 - P_D, 1.0 - P_FA)
        lik_no_mine = np.where(in_fp, 1.0 - P_FA, 1.0)

    # Log-likelihood ratio update (only non-zero where lik_no_mine > 0)
    with np.errstate(divide='ignore'):
        llr = np.where(
            (lik_mine > 0) & (lik_no_mine > 0),
            np.log(lik_mine / np.maximum(lik_no_mine, 1e-300)),
            0.0
        )
    log_odds = np.clip(log_odds + llr, LOG_ODDS_MIN, LOG_ODDS_MAX)
    return log_odds

def simulate_observation(drone_pos, mine_grid, rng):
    """
    Simulate a noisy binary sensor reading given the true mine layout.
    Returns True (hit) or False (miss).
    """
    dist = np.sqrt((XX - drone_pos[0])**2 + (YY - drone_pos[1])**2)
    in_fp = dist <= SENSOR_RADIUS
    mine_in_fp = np.any(mine_grid & in_fp)

    if mine_in_fp:
        return rng.random() < P_D
    else:
        return rng.random() < P_FA

# ── Lawnmower sweep waypoints ────────────────────────────────────────
def lawnmower_waypoints():
    """
    Generate boustrophedon scan waypoints for the 50x50 m area.
    Row spacing = 2 * r_s to guarantee full coverage.
    Returns list of (x, y) world-space waypoints.
    """
    waypoints = []
    y = SENSOR_RADIUS
    row = 0
    while y <= AREA_SIZE + SENSOR_RADIUS:
        if row % 2 == 0:
            waypoints.append((0.0, y))
            waypoints.append((AREA_SIZE, y))
        else:
            waypoints.append((AREA_SIZE, y))
            waypoints.append((0.0, y))
        y += ROW_SPACING
        row += 1
    return waypoints

# ── Sweep simulation ────────────────────────────────────────────────
def run_sweep(mine_grid, seed=0):
    """
    Fly the lawnmower sweep, collect binary observations, and update the
    Bayesian occupancy grid.  Returns the final posterior probability map.
    """
    rng  = np.random.default_rng(seed)
    lo   = init_log_odds()
    wpts = lawnmower_waypoints()

    scan_positions = []
    hits           = []

    for wp in wpts:
        drone_pos = np.array(wp)
        hit       = simulate_observation(drone_pos, mine_grid, rng)
        lo        = bayes_update(lo, drone_pos, hit)
        scan_positions.append(drone_pos.copy())
        hits.append(hit)

    p_mine = log_odds_to_prob(lo)
    return p_mine, scan_positions, hits

# ── Risk-aware A* ────────────────────────────────────────────────────
def risk_astar(p_mine, start_rc, goal_rc, w_heuristic=1.0):
    """
    A* path planner on the 8-connected occupancy grid.
    Edge cost = P_mine(cell) * step_length.
    Heuristic = w_heuristic * mean(P_mine) * Euclidean_distance_in_cells.
    Returns list of (row, col) cell indices from start to goal.
    """
    mean_risk = float(np.mean(p_mine))
    DIRS      = [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]
    STEP_LEN  = [1.0,   1.0,  1.0,   1.0,  np.sqrt(2)]*2   # cardinal / diagonal

    def h(r, c):
        dr = goal_rc[0] - r
        dc = goal_rc[1] - c
        return w_heuristic * mean_risk * np.sqrt(dr*dr + dc*dc)

    # Priority queue: (f, g, row, col, path)
    open_heap = []
    g0 = p_mine[start_rc] * 1.0
    heapq.heappush(open_heap, (g0 + h(*start_rc), g0, start_rc[0], start_rc[1], [start_rc]))
    visited   = {}

    while open_heap:
        f, g, r, c, path = heapq.heappop(open_heap)
        if (r, c) in visited:
            continue
        visited[(r, c)] = g
        if (r, c) == goal_rc:
            return path

        for k, (dr, dc) in enumerate(DIRS):
            nr, nc = r + dr, c + dc
            if not (0 <= nr < GRID_N and 0 <= nc < GRID_N):
                continue
            if (nr, nc) in visited:
                continue
            step   = STEP_LEN[k % len(STEP_LEN)]
            ng     = g + p_mine[nr, nc] * step
            nf     = ng + h(nr, nc)
            heapq.heappush(open_heap, (nf, ng, nr, nc, path + [(nr, nc)]))

    return []   # no path found

def path_risk(p_mine, path):
    """Compute total risk integral for a given cell-index path."""
    total = 0.0
    for k in range(len(path) - 1):
        r1, c1 = path[k]
        r2, c2 = path[k+1]
        step   = np.sqrt((r2-r1)**2 + (c2-c1)**2)
        total += p_mine[r2, c2] * step
    return total

def direct_path(start_rc, goal_rc):
    """Bresenham-like straight-line cell path from start to goal."""
    r0, c0 = start_rc
    r1, c1 = goal_rc
    path   = []
    steps  = max(abs(r1 - r0), abs(c1 - c0))
    for i in range(steps + 1):
        t   = i / max(steps, 1)
        r   = int(round(r0 + t * (r1 - r0)))
        c   = int(round(c0 + t * (c1 - c0)))
        if not path or path[-1] != (r, c):
            path.append((r, c))
    return path

# ── Visualisation ────────────────────────────────────────────────────
def plot_risk_map(p_mine, mine_grid, title="Posterior Risk Map"):
    fig, ax = plt.subplots(figsize=(7, 7))
    im = ax.imshow(p_mine, origin="lower",
                   extent=[0, AREA_SIZE, 0, AREA_SIZE],
                   cmap="YlOrRd", vmin=0.0, vmax=min(1.0, p_mine.max() * 2))
    plt.colorbar(im, ax=ax, label="P(mine | observations)")
    # True mine locations (revealed for evaluation)
    mine_ys, mine_xs = np.where(mine_grid)
    ax.scatter(xs[mine_xs], ys[mine_ys], marker="x", s=40,
               color="black", linewidths=1.5, label="True mines")
    ax.set_xlim(0, AREA_SIZE)
    ax.set_ylim(0, AREA_SIZE)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title(title)
    ax.legend(loc="upper right", fontsize=8)
    plt.tight_layout()
    return fig

def plot_paths(p_mine, mine_grid, path_astar, path_direct,
               scan_positions=None):
    fig, ax = plt.subplots(figsize=(7, 7))
    im = ax.imshow(p_mine, origin="lower",
                   extent=[0, AREA_SIZE, 0, AREA_SIZE],
                   cmap="YlOrRd", vmin=0.0, vmax=min(1.0, p_mine.max() * 2),
                   alpha=0.85)
    plt.colorbar(im, ax=ax, label="P(mine | observations)")

    # True mine positions
    mine_ys, mine_xs = np.where(mine_grid)
    ax.scatter(xs[mine_xs], ys[mine_ys], marker="x", s=40,
               color="black", linewidths=1.5, label="True mines", zorder=5)

    # Direct path (red dashed)
    dp_x = [cell_to_pos(r, c)[0] for r, c in path_direct]
    dp_y = [cell_to_pos(r, c)[1] for r, c in path_direct]
    ax.plot(dp_x, dp_y, "r--", lw=2.0, label="Direct path", zorder=6)

    # A* path (blue solid)
    ap_x = [cell_to_pos(r, c)[0] for r, c in path_astar]
    ap_y = [cell_to_pos(r, c)[1] for r, c in path_astar]
    ax.plot(ap_x, ap_y, "b-",  lw=2.5, label="Risk-aware A*", zorder=7)

    # Entry / exit markers
    ep = cell_to_pos(*ENTRY_CELL)
    gp = cell_to_pos(*EXIT_CELL)
    ax.plot(*ep, "gs", ms=10, label="Entry", zorder=8)
    ax.plot(*gp, "g^", ms=10, label="Exit",  zorder=8)

    ax.set_xlim(0, AREA_SIZE)
    ax.set_ylim(0, AREA_SIZE)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("Extraction Path Comparison")
    ax.legend(loc="upper right", fontsize=8)
    plt.tight_layout()
    return fig

def run_simulation(seed=42):
    mine_grid = place_mines(seed=seed)
    p_mine, scan_pos, hits = run_sweep(mine_grid, seed=seed)

    path_a = risk_astar(p_mine, ENTRY_CELL, EXIT_CELL, w_heuristic=1.0)
    path_d = direct_path(ENTRY_CELL, EXIT_CELL)

    r_astar  = path_risk(p_mine, path_a)
    r_direct = path_risk(p_mine, path_d)
    e_astar  = sum(p_mine[r, c] for r, c in path_a)
    e_direct = sum(p_mine[r, c] for r, c in path_d)

    print(f"Direct path  — risk integral: {r_direct:.4f}  |  E[hits]: {e_direct:.4f}")
    print(f"A* path      — risk integral: {r_astar:.4f}  |  E[hits]: {e_astar:.4f}")
    print(f"Risk reduction: {(r_direct - r_astar) / max(r_direct, 1e-9) * 100:.1f} %")
    print(f"Path length ratio (A*/direct): {len(path_a)/max(len(path_d),1):.2f}")
    print(f"Sensor hits during sweep: {sum(hits)} detections")

    return p_mine, mine_grid, scan_pos, path_a, path_d
```

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Area | 50 × 50 m |
| Grid resolution | 0.5 m/cell (100 × 100 = 10 000 cells) |
| Number of mines $M$ | 15 |
| Mine prior $P_0$ | $M / N_c = 0.0015$ |
| Sensor footprint radius $r_s$ | 1.5 m |
| Scan altitude $h$ | 3.0 m |
| Drone cruise speed $v_d$ | 2.0 m/s |
| Probability of detection $P_d$ | 0.85 |
| False-alarm rate $P_{fa}$ | 0.05 |
| Log-odds clamp | $[-10,\; 10]$ |
| Lawnmower row spacing $d_{row}$ | $2\,r_s = 3.0$ m |
| Number of sweep rows $N_{rows}$ | 17 |
| Total sweep path length $L_{sweep}$ | $\approx$ 898 m |
| Entry cell | (row 0, col 50) — south edge centre |
| Exit cell | (row 99, col 50) — north edge centre |
| A* heuristic weight $w$ | 1.0 (admissible; increases to speed convergence) |
| 8-connected neighbourhood | Cardinal step $= 1$ cell; diagonal $= \sqrt{2}$ cells |

---

## Expected Output

- **Posterior risk map**: 2D top-down heatmap ($100 \times 100$) of $P_{mine}(x, y)$ after the
  full lawnmower sweep; colour scale from white (low risk) to red (high risk); true mine positions
  overlaid as black crosses; entry/exit points marked as green square and triangle; colour bar
  labelled "P(mine | observations)".
- **Path comparison plot**: risk map (semi-transparent) with the direct straight-line path (red
  dashed) and the risk-aware A\* path (blue solid) both overlaid; annotated with each path's risk
  integral $R(\pi)$ and expected hits $\mathbb{E}[\text{hits}]$ in the figure legend.
- **Sweep trajectory animation (GIF)**: drone flying the lawnmower sweep frame-by-frame; the
  posterior risk map updating after each waypoint observation; hit detections marked with a red
  circle and misses with a grey dot; current posterior entropy displayed in the title.
- **Risk reduction bar chart**: side-by-side bars for direct path and A\* path showing risk
  integral and expected mines hit; annotated with percentage reduction and path length ratio.
- **Metrics summary** (printed to console): risk integral and expected hits for each strategy,
  risk reduction ratio (%), path length ratio (A\*/direct), and total hit detections during sweep.

---

## Extensions

1. **Clustered mine layout**: replace the uniform random prior with a spatially clustered mine
   placement (Gaussian mixture or Poisson cluster process); test whether the lawnmower sweep's
   posterior correctly identifies high-density zones and whether A\* routes around them.
2. **Adaptive re-sweep**: after computing the initial posterior, dispatch the drone on a second
   targeted pass to re-scan all cells with $P_{mine} > 0.3$; quantify the reduction in posterior
   entropy and improvement in path safety versus the additional flight time.
3. **Multi-drone parallel sweep**: deploy $N = 3$ drones to sweep non-overlapping column strips
   simultaneously; share a single log-odds grid via broadcast; evaluate mission time reduction
   and coverage quality vs. single-drone baseline (see S048 Lawnmower Coverage).
4. **Probabilistic path execution**: model a human walking the extracted path as a stochastic
   agent with positional error $\sigma_{walk} = 0.5$ m; convolve the path with a Gaussian kernel
   and compute the resulting effective risk integral; optimise path width (risk corridor).
5. **Terrain-adjusted sensor model**: add a soil-type layer (clay / sand / gravel) that modifies
   $P_d$ and $P_{fa}$ spatially; update the log-odds sensor model to be position-dependent;
   evaluate impact on posterior accuracy.
6. **Threshold-policy sweep**: instead of always flying the full lawnmower pattern, stop the sweep
   early once the path risk drops below a safety threshold $R_{safe} = 0.01$; compare time-to-safe
   versus always-complete-sweep.

---

## Related Scenarios

- Prerequisites: [S041 Area Coverage Sweep](S041_area_coverage_sweep.md), [S042 Missing Person Localization](S042_missing_person.md)
- Follow-ups: [S055 Oil Spill Tracking](S055_oil_spill_tracking.md) (occupancy map for advecting hazard), [S056 Radiation Hotspot Detection](S056_radiation_hotspot_detection.md) (similar Bayesian sensor fusion structure)
- Algorithmic cross-reference: [S045 Chemical Plume Tracing](S045_plume_tracing.md) (Bayesian belief update + risk-driven navigation), [S048 Lawnmower Coverage](S048_lawnmower_coverage.md) (boustrophedon sweep baseline)
