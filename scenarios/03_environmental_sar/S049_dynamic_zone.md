# S049 Dynamic Zone Search

**Domain**: Environmental Monitoring & SAR | **Difficulty**: ⭐⭐⭐ | **Status**: `[x]` Complete

---

## Problem Definition

**Setup**: A $400 \times 400$ m search area contains an unknown number of survivors following a
disaster. A prior probability map $P(\mathbf{x})$ encodes the likelihood of finding a survivor at
each grid cell, based on last-known positions and terrain features. Four drones depart from a base
station at the edge of the area. The area is initially partitioned into four Voronoi cells — one
per drone — so that each drone has exclusive responsibility for its own zone. Drones execute a
boustrophedon (lawnmower) sweep of their zone, scanning for survivors with a circular footprint
sensor. When a drone either finds a survivor or drops below $30\%$ state of charge (SoC), its
remaining uncovered zone is reallocated among the other active drones by recomputing a
battery-weighted Voronoi partition centred on their current positions.

**Roles**:
- **Drones** ($K = 4$): homogeneous quadrotors, each with a sensor footprint radius $r_s = 8$ m,
  cruise speed $v = 6$ m/s, and full battery capacity $E_{max}$ normalised to 1.0. Each drone
  maintains its own set of scanned cells and estimated remaining coverage area.
- **Survivors**: $N_s = 5$ survivors placed at fixed positions sampled from the prior map
  $P(\mathbf{x})$; a survivor is detected when a drone's sensor footprint overlaps its position.
- **Prior map** $P(\mathbf{x})$: a 2D Gaussian mixture density over the search grid, representing
  the pre-mission belief about survivor locations.

**Objective**: Minimise **total time to find all survivors** while maintaining balanced load
distribution across drones. Secondary objective: maximise coverage efficiency (fraction of
high-probability cells scanned early). Reallocation must keep the load imbalance metric below a
fairness threshold throughout the mission.

**Comparison strategies**:
1. **Static Voronoi** — initial partition fixed; no reallocation even after a survivor is found or
   battery drops.
2. **Dynamic Voronoi (equal weight)** — reallocation triggered on events; new partition uses equal
   weights (standard Voronoi on current positions).
3. **Dynamic Weighted Voronoi (battery weight)** — reallocation uses battery-proportional weights
   so higher-SoC drones absorb more of the vacated zone.

---

## Mathematical Model

### Search Grid and Prior Map

Discretise the $400 \times 400$ m area into a grid of $N_x \times N_y = 80 \times 80$ cells
(resolution $\delta = 5$ m). Cell centres are $\mathbf{x}_{ij} = (i\delta, j\delta)$. The prior
probability map is a normalised Gaussian mixture:

$$P(\mathbf{x}) = \frac{1}{Z} \sum_{m=1}^{M} \alpha_m \,
  \mathcal{N}\!\left(\mathbf{x};\, \boldsymbol{\mu}_m,\, \sigma_m^2 \mathbf{I}\right), \qquad
  Z = \sum_{\mathbf{x}_{ij}} P(\mathbf{x}_{ij}) \cdot \delta^2$$

with $M = 3$ Gaussian components, mixing weights $\alpha_m$, centres $\boldsymbol{\mu}_m$, and
spread $\sigma_m \in [40, 80]$ m.

### Standard Voronoi Partition

Given generator positions $\mathbf{p}_k \in \mathbb{R}^2$ for drones $k = 1, \ldots, K$, the
standard Voronoi cell for drone $k$ is:

$$V_k = \bigl\{\mathbf{x} \in \mathcal{A} : \|\mathbf{x} - \mathbf{p}_k\| \leq \|\mathbf{x} - \mathbf{p}_j\| \;\forall\, j \neq k\bigr\}$$

where $\mathcal{A}$ is the search area. This minimises total distance from each cell to its
assigned drone generator.

### Battery-Weighted Voronoi Partition

To redistribute load proportionally to remaining battery, replace Euclidean distance with a
battery-scaled distance. Define the weight $w_k = \mathrm{SoC}_k \in (0, 1]$ (fraction of battery
remaining). The weighted Voronoi cell is:

$$V_k^w = \left\{\mathbf{x} \in \mathcal{A} : \frac{\|\mathbf{x} - \mathbf{p}_k\|}{w_k} \leq
  \frac{\|\mathbf{x} - \mathbf{p}_j\|}{w_j} \;\forall\, j \neq k\right\}$$

A drone with higher $w_k$ (more battery) receives a larger cell because the effective distance to
any point is scaled down by $w_k$. When a drone $k^*$ exits the mission (survivor found or SoC
threshold crossed), it is removed from the generator set and the weighted Voronoi is recomputed
over the remaining $K' = K - 1$ drones using only the uncovered cells $\mathcal{A}_{uncov}$.

### Coverage Priority Ordering

Within each Voronoi zone $V_k$, drones visit cells in descending order of prior probability:

$$\pi_k = \operatorname{argsort}\!\Bigl(-P(\mathbf{x}_{ij})\Bigr), \quad \mathbf{x}_{ij} \in V_k$$

The boustrophedon sweep is then reordered to pass through high-probability cells first while
maintaining path continuity (nearest-unvisited greedy on the sorted priority list).

### Battery Consumption Model

Each drone consumes battery at a constant rate proportional to distance flown:

$$\mathrm{SoC}_k(t) = 1 - \frac{d_k(t)}{D_{max}}$$

where $d_k(t)$ is the total distance flown by drone $k$ up to time $t$ and $D_{max}$ is the
maximum mission distance on a full charge. Reallocation is triggered for drone $k$ when:

$$\mathrm{SoC}_k(t) < \mathrm{SoC}_{thresh} = 0.30$$

or when drone $k$ detects a survivor (it hovers in place for confirmation, then is considered
out-of-zone for further search).

### Sensor Detection Model

Drone $k$ at position $\mathbf{p}_k(t)$ detects survivor $i$ at position $\mathbf{s}_i$ if:

$$\|\mathbf{p}_k(t) - \mathbf{s}_i\| \leq r_s$$

The corresponding grid cell $\mathbf{x}_{ij}$ is marked as scanned when the drone centre passes
within $r_s$ of the cell centre. The fraction of the area scanned at time $t$ is:

$$C(t) = \frac{|\mathcal{A}_{scanned}(t)|}{|\mathcal{A}|} = \frac{\delta^2 \cdot
  |\{(i,j) : \text{cell } (i,j) \text{ scanned}\}|}{400 \times 400}$$

### Load Imbalance Metric

Define the uncovered area assigned to each active drone as $A_k^{rem}(t)$ (number of unscanned
cells in $V_k$ at time $t$ multiplied by $\delta^2$). The Jain fairness index on remaining load is:

$$\mathcal{J}(t) = \frac{\left(\sum_{k=1}^{K'} A_k^{rem}\right)^2}{K' \cdot \sum_{k=1}^{K'} \left(A_k^{rem}\right)^2} \in \left[\frac{1}{K'},\, 1\right]$$

$\mathcal{J} = 1$ means perfectly equal load; $\mathcal{J} = 1/K'$ means one drone carries all
remaining work. The weighted Voronoi reallocation is designed to maintain $\mathcal{J}$ close to 1
at each reallocation event.

### Reallocation Protocol

When trigger condition fires for drone $k^*$:

1. Remove $\mathbf{p}_{k^*}$ from the generator set; active set becomes $\mathcal{K}' = \mathcal{K} \setminus \{k^*\}$.
2. Collect uncovered cells: $\mathcal{A}_{uncov} = \mathcal{A}_{V_{k^*}} \setminus \mathcal{A}_{scanned}$.
3. Recompute battery-weighted Voronoi $\{V_k^w\}_{k \in \mathcal{K}'}$ over the full remaining
   search area $\mathcal{A} \setminus \mathcal{A}_{scanned}$ using current positions and SoC values.
4. Update each active drone $k$'s waypoint queue to reflect its new zone $V_k^w$, retaining
   priority ordering from the prior map.
5. Log $\mathcal{J}$ before and after reallocation to measure load balance improvement.

---

## Implementation

```python
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import Voronoi, KDTree

# Key constants
K_DRONES    = 4
AREA_SIZE   = 400.0     # m — square search area side length
GRID_RES    = 5.0       # m — cell resolution
SPEED       = 6.0       # m/s — drone cruise speed
SENSOR_R    = 8.0       # m — sensor footprint radius
D_MAX       = 2000.0    # m — maximum range on full battery
SOC_THRESH  = 0.30      # reallocation trigger threshold
N_SURVIVORS = 5
DT          = 0.5       # s — simulation timestep

Nx = int(AREA_SIZE / GRID_RES)   # 80
Ny = int(AREA_SIZE / GRID_RES)   # 80

def build_prior_map(Nx, Ny, res):
    """Build a 3-component Gaussian mixture prior probability map."""
    xv, yv = np.meshgrid(
        np.arange(Nx) * res + res / 2,
        np.arange(Ny) * res + res / 2,
        indexing='ij'
    )
    coords = np.stack([xv, yv], axis=-1)  # (Nx, Ny, 2)

    components = [
        (0.5, np.array([100., 300.]), 60.),
        (0.3, np.array([280., 150.]), 50.),
        (0.2, np.array([340., 320.]), 40.),
    ]
    P = np.zeros((Nx, Ny))
    for alpha, mu, sigma in components:
        diff = coords - mu  # (Nx, Ny, 2)
        P += alpha * np.exp(-0.5 * np.sum(diff**2, axis=-1) / sigma**2)
    P /= P.sum()  # normalise to sum = 1 over all cells
    return P

def weighted_voronoi_assignment(positions, weights, grid_centres):
    """
    Assign each grid cell to the drone with minimum weighted distance.
    positions: (K, 2)  weights: (K,)  grid_centres: (N, 2)
    Returns: assignment array of shape (N,) with drone index per cell.
    """
    K = len(positions)
    N = len(grid_centres)
    scaled_dist = np.zeros((N, K))
    for k in range(K):
        diff = grid_centres - positions[k]
        dist = np.linalg.norm(diff, axis=1)
        scaled_dist[:, k] = dist / (weights[k] + 1e-9)
    return np.argmin(scaled_dist, axis=1)

def jain_fairness(areas):
    """Compute Jain fairness index for a list of per-drone remaining areas."""
    areas = np.array(areas, dtype=float)
    if areas.sum() == 0 or len(areas) == 0:
        return 1.0
    return areas.sum()**2 / (len(areas) * (areas**2).sum())

def priority_sorted_waypoints(cell_indices, prior_flat, grid_centres):
    """Sort cells in descending prior probability order."""
    probs = prior_flat[cell_indices]
    order = np.argsort(-probs)
    return grid_centres[cell_indices[order]]

class DroneAgent:
    def __init__(self, idx, start_pos):
        self.idx      = idx
        self.pos      = start_pos.copy()
        self.soc      = 1.0
        self.dist     = 0.0
        self.waypoints = []
        self.active   = True
        self.found_survivor = False
        self.history  = [start_pos.copy()]

    def step(self, dt, speed):
        if not self.active or len(self.waypoints) == 0:
            return
        target = self.waypoints[0]
        diff   = target - self.pos
        dist   = np.linalg.norm(diff)
        move   = speed * dt
        if dist <= move:
            self.pos = target.copy()
            self.waypoints.pop(0)
        else:
            self.pos += (diff / dist) * move
            move = dist  # actual distance moved this step
        self.dist += move
        self.soc   = max(0.0, 1.0 - self.dist / D_MAX)
        self.history.append(self.pos.copy())

def run_simulation(strategy='weighted'):
    """
    strategy: 'static' | 'equal' | 'weighted'
    Returns: detection times for each survivor, coverage over time, fairness over time.
    """
    rng  = np.random.default_rng(42)
    P    = build_prior_map(Nx, Ny, GRID_RES)
    P_flat = P.ravel()

    # Grid cell centres (Nx*Ny, 2)
    xi = (np.arange(Nx) * GRID_RES + GRID_RES / 2)
    yi = (np.arange(Ny) * GRID_RES + GRID_RES / 2)
    xv, yv = np.meshgrid(xi, yi, indexing='ij')
    cell_centres = np.stack([xv.ravel(), yv.ravel()], axis=1)  # (6400, 2)

    # Place survivors by sampling from prior
    survivor_cells = rng.choice(len(cell_centres), size=N_SURVIVORS,
                                replace=False, p=P_flat)
    survivors = cell_centres[survivor_cells].copy()
    survivor_found = [False] * N_SURVIVORS
    detect_times   = [None]  * N_SURVIVORS

    # Initialise drones at corners / edge midpoints
    starts = np.array([
        [20., 20.], [380., 20.], [20., 380.], [380., 380.]
    ])
    drones = [DroneAgent(k, starts[k]) for k in range(K_DRONES)]

    scanned = np.zeros(len(cell_centres), dtype=bool)
    cell_tree = KDTree(cell_centres)

    def assign_waypoints(active_drones, scanned_mask):
        positions = np.array([d.pos for d in active_drones])
        if strategy == 'static' or strategy == 'equal':
            weights = np.ones(len(active_drones))
        else:  # weighted
            weights = np.array([d.soc for d in active_drones])

        unscanned_idx = np.where(~scanned_mask)[0]
        if len(unscanned_idx) == 0:
            return

        assignment = weighted_voronoi_assignment(
            positions, weights, cell_centres[unscanned_idx]
        )
        for i, d in enumerate(active_drones):
            zone_cells = unscanned_idx[assignment == i]
            if len(zone_cells) == 0:
                d.waypoints = []
                continue
            wpts = priority_sorted_waypoints(zone_cells, P_flat, cell_centres)
            d.waypoints = list(wpts)

    active = [d for d in drones if d.active]
    assign_waypoints(active, scanned)

    t = 0.0
    fairness_log = []
    coverage_log = []
    t_log        = []
    realloc_events = []

    while True:
        active = [d for d in drones if d.active]
        if not active:
            break
        if all(f for f in survivor_found):
            break

        # Step all active drones
        for d in active:
            d.step(DT, SPEED)

        # Mark newly scanned cells
        for d in active:
            nearby = cell_tree.query_ball_point(d.pos, SENSOR_R)
            for c in nearby:
                scanned[c] = True

        # Check survivor detections
        for i, s in enumerate(survivors):
            if survivor_found[i]:
                continue
            for d in active:
                if np.linalg.norm(d.pos - s) <= SENSOR_R:
                    survivor_found[i] = True
                    detect_times[i]   = t
                    if strategy != 'static':
                        d.active = False
                        d.found_survivor = True
                        realloc_events.append((t, d.idx, 'survivor'))
                    break

        # Check SoC triggers (non-static strategies only)
        if strategy != 'static':
            for d in active:
                if d.soc < SOC_THRESH and d.active:
                    d.active = False
                    realloc_events.append((t, d.idx, 'battery'))

        # Reallocation when any drone just deactivated
        active_now = [d for d in drones if d.active]
        if len(active_now) < len(active) and len(active_now) > 0:
            assign_waypoints(active_now, scanned)

        # Log metrics
        coverage = scanned.sum() / len(scanned)
        coverage_log.append(coverage)
        rem_areas = [len([wpt for wpt in d.waypoints]) * GRID_RES**2
                     for d in active_now if d.active]
        fairness_log.append(jain_fairness(rem_areas) if rem_areas else 1.0)
        t_log.append(t)
        t += DT

        if t > 3600.0:  # safety cutoff
            break

    return drones, detect_times, coverage_log, fairness_log, t_log, realloc_events, survivors, P
```

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Search area | 400 × 400 m |
| Grid resolution $\delta$ | 5 m (80 × 80 cells) |
| Number of drones $K$ | 4 |
| Drone cruise speed $v$ | 6 m/s |
| Sensor footprint radius $r_s$ | 8 m |
| Full-charge range $D_{max}$ | 2000 m |
| Reallocation SoC threshold | 30% |
| Number of survivors $N_s$ | 5 |
| Prior map components $M$ | 3 Gaussian blobs |
| Prior blob spreads $\sigma_m$ | 40 – 80 m |
| Simulation timestep $\Delta t$ | 0.5 s |
| Safety cutoff | 3600 s |
| Strategies compared | Static Voronoi, Equal-weight Voronoi, Battery-weighted Voronoi |

---

## Expected Output

- **Search area map**: 2D top-down plot of the $400 \times 400$ m grid with the prior probability
  heatmap overlaid; initial Voronoi cells drawn with coloured boundaries (one colour per drone);
  survivor positions marked as yellow stars; drone start positions marked as triangles.
- **Dynamic reallocation snapshots**: three side-by-side panels showing Voronoi cell boundaries at
  $t = 0$, at the first reallocation event, and at the second reallocation event; scanned cells
  shown in grey; remaining uncovered cells coloured by zone assignment.
- **Drone trajectories**: final paths of all four drones overlaid on the grid, coloured per drone;
  survivor detection events marked with a circle; reallocation moments marked with a vertical dashed
  line on the trajectory.
- **Coverage vs time**: plot of $C(t)$ for all three strategies on the same axes; vertical dashed
  lines marking reallocation events for the dynamic strategies.
- **Survivor detection timeline**: horizontal bar chart showing the detection time of each survivor
  for each strategy; grouped bars per survivor index.
- **Jain fairness index vs time**: $\mathcal{J}(t)$ plotted for all three strategies; drops at
  reallocation events and recovery are clearly visible for the dynamic strategies.
- **Summary statistics table**: per-strategy metrics: time to find all survivors, mean detection
  time, final coverage fraction, number of reallocation events, mean Jain fairness.
- **Animation (GIF)**: top-down view showing drones moving in real time, scanned cells filling in
  as grey, Voronoi boundaries updating at each reallocation, survivor icons turning green on
  detection.

---

## Extensions

1. **Heterogeneous sensor ranges**: assign different $r_s$ values per drone (e.g., a scout drone
   with wide-area thermal camera vs close-range optical drones); update weighted Voronoi to account
   for sensor coverage rate $\dot{C}_k = v_k \cdot 2 r_{s,k}$ rather than pure battery weight.
2. **Adaptive prior update (Bayesian)**: after each negative scan of a cell, reduce its posterior
   probability; drones re-sort their waypoint queues dynamically as the posterior evolves, converging
   to a fully informative coverage policy.
3. **Communication dropout**: drones cannot broadcast their positions or SoC during GPS/radio
   blackout windows $[t_{black}, t_{black} + \Delta t_{drop}]$; each drone must decide on local
   reallocation heuristics using last-known peer state.
4. **3D zone search**: extend to a $400 \times 400 \times 60$ m volume (e.g., building rubble
   or forested terrain); generators become 3D points, Voronoi cells are 3D, and drones sweep
   horizontal altitude layers within their zone.
5. **RL reallocation policy**: replace the geometric weighted Voronoi rule with a PPO agent that
   observes all drones' SoC, positions, and the current coverage map, and outputs zone boundaries
   directly; compare against weighted Voronoi on randomised survivor distributions.
6. **Mixed battery-and-urgency weighting**: survivors have known urgency levels (e.g., estimated
   survival time); incorporate urgency into the zone weight:
   $w_k = \mathrm{SoC}_k \cdot \bar{U}_k$ where $\bar{U}_k$ is the mean urgency of survivors
   estimated within $V_k$.

---

## Related Scenarios

- Prerequisites: [S041 Wildfire Boundary Scan](S041_wildfire_boundary_scan.md), [S042 Missing Person Search](S042_missing_person_search.md), [S048 Lawnmower Coverage](S048_lawnmower_coverage.md)
- Follow-ups: [S050 Swarm SLAM](S050_swarm_slam.md) (simultaneous mapping during search), [S054 Minefield Detection](S054_minefield_detection.md) (zone-based detection with risk weighting)
- Algorithmic cross-reference: [S018 Multi-Target Interception](../01_pursuit_evasion/S018_multi_target_interception.md) (Hungarian assignment), [S019 Dynamic Reassignment](../01_pursuit_evasion/S019_dynamic_reassignment.md) (real-time zone reallocation), [S040 Fleet Load Balancing](../02_logistics_delivery/S040_fleet_load_balancing.md) (Jain fairness, load redistribution)
