# S054 Minefield Detection

**Domain**: Environmental & SAR | **Difficulty**: ⭐⭐⭐ | **Status**: `[ ]` Not Started

---

## Problem Definition

**Setup**: A $200 \times 200$ m suspected minefield must be surveyed before ground personnel can
enter. A single drone equipped with a downward-facing **ground-penetrating radar (GPR)** sensor
flies at a constant altitude of $z = 3$ m and builds a probabilistic risk map of the area in
real time. The GPR footprint has an effective detection radius $r_{gpr} = 4$ m. An unknown
number of mines $N_{mine}$ (drawn from $\mathcal{U}\{8, 20\}$) are scattered across the field;
each mine is a point target with a fixed position unknown to the drone at mission start.

The drone begins and ends at a **safe zone** entry point on the field boundary. If a mine is
detected above a confirmation threshold, the risk map is immediately updated and an **escape
path** back to the safe zone is replanned via A* on the risk-weighted grid. Full mission
success requires both (a) scanning the entire field to the required detection completeness and
(b) maintaining a viable low-risk escape corridor at all times.

**Roles**:
- **Drone**: single GPR-equipped survey platform; executes a boustrophedon (lawnmower) base
  path with dynamic detour capability; speed $v = 3$ m/s.
- **Mines**: $N_{mine}$ fixed point targets; position unknown; detectable within GPR range.
- **Safe Zone**: $10 \times 200$ m strip along the southern edge ($y \leq 10$ m);
  entry/exit waypoint at $(100, 0)$ m.

**Objective**: Achieve **coverage completeness** $C \geq 0.95$ (fraction of $1 \times 1$ m grid
cells scanned with cumulative $P_d > 0.9$) while keeping the risk of the maintained escape
path below $R_{path} < 0.1$. Minimise total mission time subject to these two safety constraints.

**Comparison strategies**:
1. **Fixed boustrophedon** — fixed parallel-strip lawnmower, no replanning; ignores detected
   mines when choosing next waypoint.
2. **Risk-aware boustrophedon** — same base pattern but detours around high-risk cells
   (replans locally when $R(cell) > R_{threshold}$).
3. **Adaptive frontier coverage** — frontier-based planner that prioritises unscanned cells
   reachable via low-risk corridors; global A* replanning after every mine detection.

---

## Mathematical Model

### GPR Detection Probability

For a mine at position $\mathbf{m}$ and drone at position $\mathbf{p}$, the instantaneous
detection probability over one timestep $\Delta t$ is:

$$P_d(r) = 1 - \exp\!\left(-\frac{r_{gpr}^2}{2\, r^2}\right), \qquad r = \|\mathbf{p} - \mathbf{m}\|$$

where $r_{gpr} = 4$ m is the GPR effective radius. At range $r = r_{gpr}$ this gives
$P_d \approx 0.394$; at $r = r_{gpr}/2$ it gives $P_d \approx 0.865$.

The cumulative detection probability after the drone has visited positions
$\mathbf{p}_1, \mathbf{p}_2, \ldots, \mathbf{p}_T$ is:

$$P_d^{cum}(\mathbf{m}) = 1 - \prod_{t=1}^{T} \bigl(1 - P_d(\|\mathbf{p}_t - \mathbf{m}\|)\bigr)$$

A mine is **confirmed** when $P_d^{cum}(\mathbf{m}) > \theta_{confirm} = 0.95$. It is
**detected as alarm** (unconfirmed) at the first timestep where $P_d(r) > \theta_{alarm} = 0.5$.

A fixed **false alarm probability** $P_{fa} = 0.05$ is applied independently per cell per scan
pass: with probability $P_{fa}$ a mine-free cell triggers a spurious alarm that must be
reclassified by a second overpass.

### Coverage Completeness

The field is discretised into a $200 \times 200$ grid of $1 \times 1$ m cells. Cell $(i, j)$
is considered **scanned** when the drone has accumulated sufficient cumulative detection
probability over any hypothetical mine at its centre:

$$\text{scanned}(i,j) = \mathbf{1}\!\left[P_d^{cum}(c_{ij}) > 0.9\right]$$

Coverage completeness:

$$C = \frac{1}{N_{cells}} \sum_{i,j} \text{scanned}(i,j), \qquad N_{cells} = 200 \times 200$$

### Risk Map

The risk value of cell $(i,j)$ is the probability that it contains a confirmed or alarmed mine
within influence radius $r_{influence} = 6$ m:

$$R(i,j) = \sum_{k \in \mathcal{M}_{active}} P_d^{cum}(\mathbf{m}_k) \cdot \exp\!\left(
  -\frac{\|c_{ij} - \mathbf{m}_k\|^2}{2\, r_{influence}^2}
\right)$$

where $\mathcal{M}_{active}$ is the set of all alarmed or confirmed mine positions. The risk map
is clipped to $[0, 1]$. A cell is classified as:

- **Safe**: $R(i,j) < R_{safe} = 0.1$
- **Caution**: $0.1 \leq R(i,j) < R_{caution} = 0.4$
- **Danger**: $R(i,j) \geq R_{caution} = 0.4$

### A* Replanning on Risk-Weighted Grid

The escape path is replanned as a least-cost path from the drone's current position to the
safe zone entry point. The traversal cost of moving from cell $(i,j)$ to an adjacent cell
$(i',j')$ is:

$$w(i',j') = 1 + \alpha \cdot R(i',j')^2$$

with $\alpha = 50$ (risk penalty weight). Standard 8-connected A* is applied on the
$200 \times 200$ grid with heuristic $h = \|c_{i'j'} - \mathbf{p}_{safe}\|_2$.

Replanning is triggered whenever:
1. A new mine alarm is raised ($P_d(r) > \theta_{alarm}$ for a previously undetected target).
2. A false-alarm cell is reclassified after a second overpass.
3. The current escape path passes through a cell whose risk exceeds $R_{caution}$.

The maintained **escape path cost**:

$$J_{escape} = \sum_{(i,j) \in \mathcal{P}} w(i,j)$$

Mission proceeds only while $J_{escape} < J_{max}$; if $J_{escape} \geq J_{max}$ the drone
aborts and immediately follows the escape path to the safe zone.

### Boustrophedon Base Pattern

Strip width equals the GPR scan diameter $2 r_{gpr} = 8$ m. For a $W \times W$ field with
$N_{strips} = \lceil W / (2 r_{gpr}) \rceil$ parallel strips, the base waypoints are:

$$\mathbf{w}_n^{start} = \begin{cases}
(4 + 8(n-1),\; 0) & n \text{ odd (south to north)} \\
(4 + 8(n-1),\; W) & n \text{ even (north to south)}
\end{cases}$$

$$\mathbf{w}_n^{end} = \begin{cases}
(4 + 8(n-1),\; W) & n \text{ odd} \\
(4 + 8(n-1),\; 0) & n \text{ even}
\end{cases}$$

for $n = 1, \ldots, N_{strips}$, with $W = 200$ m giving $N_{strips} = 25$ strips and a
theoretical path length of $25 \times 200 + 24 \times 8 = 5192$ m.

### Drone Kinematics

Point-mass constant-speed model:

$$\dot{\mathbf{p}} = v \cdot \hat{\mathbf{u}}, \qquad \hat{\mathbf{u}} =
\frac{\mathbf{w}_{next} - \mathbf{p}}{\|\mathbf{w}_{next} - \mathbf{p}\|}$$

with $v = 3$ m/s and $z$ fixed at $3$ m. Waypoint acceptance radius $r_{wp} = 0.5$ m.

---

## Implementation

```python
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
from collections import defaultdict
import heapq

# Key constants
FIELD_SIZE      = 200        # m — square field side length
CELL_SIZE       = 1.0        # m — risk map resolution
N_CELLS         = FIELD_SIZE  # cells per axis
R_GPR           = 4.0        # m — GPR effective radius
R_INFLUENCE     = 6.0        # m — risk spread radius
DRONE_SPEED     = 3.0        # m/s
DT              = 0.2        # s — simulation timestep
THETA_ALARM     = 0.50       # single-step detection threshold for alarm
THETA_CONFIRM   = 0.95       # cumulative threshold for confirmed mine
P_FA            = 0.05       # false alarm probability per cell per strip pass
R_SAFE          = 0.10       # safe corridor risk threshold
R_CAUTION       = 0.40       # danger zone risk threshold
ALPHA_RISK      = 50.0       # A* risk penalty weight
STRIP_WIDTH     = 2 * R_GPR  # m — boustrophedon strip spacing
SAFE_ZONE_Y     = 10.0       # m — safe zone boundary (y <= this value)
ENTRY_POINT     = np.array([100.0, 0.0])  # safe zone entry/exit

def gpr_detect_prob(r):
    """Single-timestep GPR detection probability at range r."""
    if r < 1e-3:
        return 1.0
    return 1.0 - np.exp(-R_GPR**2 / (2.0 * r**2))

def build_boustrophedon_waypoints(field_size, strip_width):
    """Generate parallel-strip lawnmower waypoints."""
    waypoints = [ENTRY_POINT.copy()]
    n_strips = int(np.ceil(field_size / strip_width))
    for n in range(n_strips):
        x = strip_width / 2 + n * strip_width
        x = min(x, field_size - strip_width / 2)
        if n % 2 == 0:  # south to north
            waypoints.append(np.array([x, 0.0]))
            waypoints.append(np.array([x, field_size]))
        else:            # north to south
            waypoints.append(np.array([x, field_size]))
            waypoints.append(np.array([x, 0.0]))
    waypoints.append(ENTRY_POINT.copy())
    return waypoints

def astar_risk_path(risk_map, start_cell, goal_cell, alpha=ALPHA_RISK):
    """A* on risk-weighted grid; returns list of (row, col) cells."""
    rows, cols = risk_map.shape
    def h(a, b):
        return np.hypot(a[0] - b[0], a[1] - b[1])
    def cost(cell):
        r, c = cell
        return 1.0 + alpha * risk_map[r, c]**2

    open_heap = [(h(start_cell, goal_cell), 0.0, start_cell, [start_cell])]
    visited = {}
    while open_heap:
        f, g, current, path = heapq.heappop(open_heap)
        if current in visited:
            continue
        visited[current] = g
        if current == goal_cell:
            return path
        r, c = current
        for dr in [-1, 0, 1]:
            for dc in [-1, 0, 1]:
                if dr == 0 and dc == 0:
                    continue
                nr, nc = r + dr, c + dc
                if 0 <= nr < rows and 0 <= nc < cols:
                    neighbor = (nr, nc)
                    if neighbor not in visited:
                        step = np.hypot(dr, dc) * cost(neighbor)
                        g_new = g + step
                        f_new = g_new + h(neighbor, goal_cell)
                        heapq.heappush(open_heap, (f_new, g_new, neighbor, path + [neighbor]))
    return []  # no path found

class MineFieldSimulation:
    def __init__(self, n_mines=15, seed=42):
        rng = np.random.default_rng(seed)
        # Place mines away from safe zone (y > 15 m)
        self.mine_positions = np.column_stack([
            rng.uniform(5, FIELD_SIZE - 5, n_mines),
            rng.uniform(15, FIELD_SIZE - 5, n_mines),
        ])
        self.n_mines = n_mines
        # Cumulative detection per mine
        self.cum_pd = np.zeros(n_mines)
        # Risk map
        self.risk_map = np.zeros((N_CELLS, N_CELLS))
        # Coverage map (cumulative detection for hypothetical mine at each cell)
        self.coverage = np.zeros((N_CELLS, N_CELLS))
        self.alarmed_mines = set()    # indices of alarmed mines
        self.confirmed_mines = set()  # indices of confirmed mines
        self.false_alarms = []        # list of (x,y) false alarm positions

    def pos_to_cell(self, pos):
        c = int(np.clip(pos[0] / CELL_SIZE, 0, N_CELLS - 1))
        r = int(np.clip(pos[1] / CELL_SIZE, 0, N_CELLS - 1))
        return (r, c)

    def update(self, drone_pos):
        """Update coverage map, mine detection, and risk map for current drone position."""
        # Coverage: update all cells within GPR range
        px, py = drone_pos
        i0 = max(0, int((px - R_GPR * 2) / CELL_SIZE))
        i1 = min(N_CELLS, int((px + R_GPR * 2) / CELL_SIZE) + 1)
        j0 = max(0, int((py - R_GPR * 2) / CELL_SIZE))
        j1 = min(N_CELLS, int((py + R_GPR * 2) / CELL_SIZE) + 1)
        for ci in range(i0, i1):
            for cj in range(j0, j1):
                cx = (ci + 0.5) * CELL_SIZE
                cy = (cj + 0.5) * CELL_SIZE
                r = np.hypot(px - cx, py - cy)
                pd = gpr_detect_prob(r)
                self.coverage[cj, ci] = 1 - (1 - self.coverage[cj, ci]) * (1 - pd)

        # Mine detection
        new_alarm = False
        for k, mpos in enumerate(self.mine_positions):
            r = np.linalg.norm(drone_pos - mpos)
            pd = gpr_detect_prob(r)
            self.cum_pd[k] = 1 - (1 - self.cum_pd[k]) * (1 - pd)
            if k not in self.alarmed_mines and pd > THETA_ALARM:
                self.alarmed_mines.add(k)
                new_alarm = True
            if k not in self.confirmed_mines and self.cum_pd[k] > THETA_CONFIRM:
                self.confirmed_mines.add(k)
                new_alarm = True

        # Update risk map after any new detection
        if new_alarm:
            self._rebuild_risk_map()
        return new_alarm

    def _rebuild_risk_map(self):
        self.risk_map[:] = 0.0
        for k in self.alarmed_mines | self.confirmed_mines:
            mpos = self.mine_positions[k]
            mx, my = int(mpos[0]), int(mpos[1])
            r_inf = int(R_INFLUENCE * 3)
            for ci in range(max(0, mx - r_inf), min(N_CELLS, mx + r_inf)):
                for cj in range(max(0, my - r_inf), min(N_CELLS, my + r_inf)):
                    cx = (ci + 0.5) * CELL_SIZE
                    cy = (cj + 0.5) * CELL_SIZE
                    d2 = (cx - mpos[0])**2 + (cy - mpos[1])**2
                    contrib = self.cum_pd[k] * np.exp(-d2 / (2 * R_INFLUENCE**2))
                    self.risk_map[cj, ci] = min(1.0, self.risk_map[cj, ci] + contrib)

    def coverage_completeness(self):
        return np.mean(self.coverage > 0.9)

    def escape_path_risk(self, drone_pos, path):
        if not path:
            return float('inf')
        return sum(self.risk_map[r, c] for r, c in path) / max(1, len(path))

def run_simulation(strategy='risk_aware', n_mines=15, seed=42):
    sim = MineFieldSimulation(n_mines=n_mines, seed=seed)
    waypoints = build_boustrophedon_waypoints(FIELD_SIZE, STRIP_WIDTH)

    drone_pos = ENTRY_POINT.copy()
    wp_idx = 1
    escape_path = []
    trajectory = [drone_pos.copy()]
    replan_events = []
    t = 0.0

    while wp_idx < len(waypoints) and sim.coverage_completeness() < 0.95:
        target = waypoints[wp_idx]
        direction = target - drone_pos
        dist = np.linalg.norm(direction)

        if dist < 0.5:  # waypoint reached
            wp_idx += 1
            continue

        drone_pos = drone_pos + (direction / dist) * DRONE_SPEED * DT
        t += DT

        new_alarm = sim.update(drone_pos)

        if new_alarm or strategy == 'risk_aware':
            start_cell = sim.pos_to_cell(drone_pos)
            goal_cell  = sim.pos_to_cell(ENTRY_POINT)
            escape_path = astar_risk_path(sim.risk_map, start_cell, goal_cell)
            if new_alarm:
                replan_events.append((t, drone_pos.copy()))

        trajectory.append(drone_pos.copy())

    return {
        'trajectory': np.array(trajectory),
        'risk_map': sim.risk_map,
        'coverage': sim.coverage,
        'mine_positions': sim.mine_positions,
        'confirmed_mines': sim.confirmed_mines,
        'alarmed_mines': sim.alarmed_mines,
        'coverage_completeness': sim.coverage_completeness(),
        'n_replan': len(replan_events),
        'replan_events': replan_events,
        'escape_path': escape_path,
        'mission_time': t,
    }
```

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Field size | 200 × 200 m |
| Grid resolution | 1 × 1 m ($200 \times 200$ cells) |
| Number of mines $N_{mine}$ | $\mathcal{U}\{8, 20\}$, default 15 |
| Drone speed $v$ | 3 m/s |
| Survey altitude $z$ | 3 m |
| GPR effective radius $r_{gpr}$ | 4 m |
| Risk influence radius $r_{influence}$ | 6 m |
| Strip width $2 r_{gpr}$ | 8 m |
| Number of strips | 25 |
| Theoretical path length | 5192 m |
| Estimated survey time (no replanning) | $\approx 1730$ s |
| Alarm threshold $\theta_{alarm}$ | 0.50 |
| Confirmation threshold $\theta_{confirm}$ | 0.95 |
| False alarm probability $P_{fa}$ | 0.05 |
| Risk safe threshold $R_{safe}$ | 0.10 |
| Risk caution threshold $R_{caution}$ | 0.40 |
| A* risk penalty weight $\alpha$ | 50 |
| Coverage completeness target $C$ | 0.95 |
| Simulation timestep $\Delta t$ | 0.2 s |
| Safe zone entry point | (100, 0) m |
| Safe zone extent | $y \leq 10$ m |

---

## Expected Output

- **Risk map heatmap**: 2D colour plot of $R(i,j)$ at mission end; confirmed mines marked with
  red circles, alarmed (unconfirmed) mines with orange triangles; false-alarm cells with yellow
  crosses; drone trajectory overlaid in blue; safe zone boundary as a dashed green line.
- **Coverage completeness map**: 2D plot where cell colour encodes $P_d^{cum}$ from 0 (white) to
  1 (dark green); cells meeting the 0.9 threshold shown as fully saturated; uncovered cells
  highlighted in red.
- **Escape path overlay**: A* least-risk path from drone's final abort position to safe zone,
  drawn on the risk map; path coloured by per-cell risk value to show corridor safety.
- **Coverage completeness vs time**: $C(t)$ curve for all three strategies on the same axes;
  horizontal dashed line at $C = 0.95$ target; vertical tick marks at each replan event for the
  adaptive strategy.
- **Risk along escape path vs time**: time series of mean escape path risk $\bar{R}_{path}(t)$
  for each strategy; dashed red line at $R_{safe} = 0.10$ and $R_{caution} = 0.40$.
- **Detection summary bar chart**: per-mine bar of $P_d^{cum}$ at mission end for confirmed,
  alarmed, and undetected mines; false-alarm count annotated.
- **Strategy comparison table**: mission time, final coverage completeness, mines confirmed,
  mines missed, mean escape path risk, and total replanning events for all three strategies.
- **Animation (GIF)**: top-down view with the drone sweeping the field; risk map background
  updating in real time as mines are detected; escape path redrawn in white after each replan
  event; confirmed mines appearing as red circles.

---

## Extensions

1. **Multi-drone cooperative sweep**: deploy $K = 3$ drones with non-overlapping strip
   assignments; implement Hungarian assignment to balance strip counts and rebalance strips
   dynamically when one drone must abort due to high escape-path risk.
2. **Uncertain mine positions**: replace point mine model with a Gaussian belief
   $\mathcal{N}(\hat{\mathbf{m}}_k, \Sigma_k)$; update belief via Bayes filter on GPR returns;
   replanning uses the expected risk $\mathbb{E}[R(i,j)]$ under the belief.
3. **Mine density prior map**: incorporate an external intelligence prior (e.g., historical
   minefield pattern) as a Bayesian prior on $R(i,j)$; compare survey efficiency with and
   without the prior.
4. **3D terrain following**: extend to undulating terrain using a digital elevation model (DEM);
   maintain constant AGL altitude $z_{AGL} = 3$ m with terrain-following controller; analyse
   GPR performance degradation on sloped ground.
5. **Sensor fusion — magnetometer + GPR**: add a second sensor (magnetometer) sensitive to
   metal-cased mines; combine detection likelihoods via log-odds fusion to reduce the
   false alarm rate $P_{fa}$ and the required number of strip passes.

---

## Related Scenarios

- Prerequisites: [S048 Lawnmower Coverage](S048_lawnmower_coverage.md), [S049 Dynamic Zone Assignment](S049_dynamic_zone_assignment.md)
- Follow-ups: [S056 Radiation Hotspot Detection](S056_radiation_hotspot.md) (probabilistic risk map with source localisation), [S074 Mine 3D Mapping](../../04_industrial_agriculture/S074_mine_3d_mapping.md) (full 3D volumetric scan of underground workings)
- Algorithmic cross-reference: [S034 Weather Rerouting](../../02_logistics_delivery/S034_weather_rerouting.md) (A* replanning under dynamic hazards), [S022 Obstacle Avoidance Delivery](../../02_logistics_delivery/S022_obstacle_avoidance_delivery.md) (RRT* path planning with forbidden zones)
