# S048 Full-Area Coverage Scan (Lawnmower)

**Domain**: Environmental Monitoring & SAR | **Difficulty**: ⭐ | **Status**: `[x]` Complete

---

## Problem Definition

**Setup**: A single drone must survey a $200 \times 200$ m rectangular search area from a fixed
altitude. The drone carries a nadir-pointing sensor with a circular footprint of radius
$r_{sensor} = 5$ m. Starting from a corner of the area, the drone flies a **boustrophedon
(lawnmower)** pattern — parallel east-west strips traversed alternately left-to-right and
right-to-left, connected by 180° heading reversals at each end. The goal is to guarantee that every
point in the area falls within at least one sensor footprint before the battery is exhausted.

The scenario compares **three strip-width configurations** produced by varying the lateral overlap
between adjacent strips. Under-lapping leaves blind gaps; optimal spacing achieves exactly 100%
coverage with minimum flight distance; over-lapping wastes energy on redundant re-scanning.

**Roles**:
- **Drone**: single UAV executing the pre-planned boustrophedon path; no in-flight replanning.
- **Search area**: static $200 \times 200$ m rectangle, discretised into $1 \times 1$ m grid cells
  for coverage accounting.

**Objective**: Quantify, for each strip-width configuration, (1) the final **coverage rate**
(percentage of grid cells scanned), (2) the **total path length**, and (3) the **energy consumed**,
and identify which configuration achieves 100% coverage at minimum energy.

**Comparison configurations**:
1. **Under-lap** — strip width $d = 12$ m (overlap ratio $\rho = 0\%$, gap between strips 2 m).
2. **Optimal** — strip width $d^* = 10$ m (overlap ratio $\rho = 0\%$, tangent footprints, exact
   full coverage).
3. **Over-lap** — strip width $d = 7$ m (overlap ratio $\rho = 30\%$, guaranteed coverage with
   energy penalty).

---

## Mathematical Model

### Sensor Footprint and Strip Geometry

The sensor sweeps a circle of radius $r_s = 5$ m centred directly below the drone. The effective
lateral coverage width per strip (the full footprint diameter) is:

$$w = 2 \cdot r_s = 10 \text{ m}$$

Given a desired lateral overlap ratio $\rho \in [0, 1)$, the strip spacing (centre-to-centre
distance between adjacent parallel strips) is:

$$d = w \cdot (1 - \rho) = 2 r_s (1 - \rho)$$

At $\rho = 0$ this gives the **optimal strip width** $d^* = 2 r_s$, which tiles the area with
tangent footprint circles and guarantees 100% coverage with no redundant overlap.

### Strip Count and Waypoint Generation

For a search area of width $W$ (cross-track dimension) with strips running along the $H$ (along-track)
dimension, the number of strips required is:

$$N = \left\lceil \frac{W}{d} \right\rceil$$

Strip $i$ ($i = 0, 1, \ldots, N-1$) is centred at cross-track position:

$$y_i = y_{start} + i \cdot d + \frac{d}{2}$$

The along-track start and end of strip $i$ alternate direction (boustrophedon):

$$\mathbf{w}_{i,start} = \begin{cases} (x_{min},\; y_i) & i \text{ even} \\ (x_{max},\; y_i) & i \text{ odd} \end{cases}, \qquad \mathbf{w}_{i,end} = \begin{cases} (x_{max},\; y_i) & i \text{ even} \\ (x_{min},\; y_i) & i \text{ odd} \end{cases}$$

The full waypoint sequence is:

$$\mathbf{w}_0^{start},\; \mathbf{w}_0^{end},\; \mathbf{w}_1^{start},\; \mathbf{w}_1^{end},\; \ldots,\; \mathbf{w}_{N-1}^{start},\; \mathbf{w}_{N-1}^{end}$$

where the transition segment $\mathbf{w}_{i}^{end} \to \mathbf{w}_{i+1}^{start}$ is a lateral
step of length $d$ (the turn leg).

### Total Path Length

The total path length decomposes into $N$ scan strips and $N-1$ turn legs:

$$L = \underbrace{N \cdot H}_{\text{scan strips}} + \underbrace{(N-1) \cdot d}_{\text{turn legs}}$$

where $H = 200$ m is the along-track strip length and $d$ is the strip spacing.

### Coverage Rate

The search area is discretised into a grid of $W \times H$ unit cells. A cell at position
$(x_c, y_c)$ is marked **scanned** when the drone passes within $r_s$ of its centre:

$$\text{cell}(x_c, y_c) \text{ scanned} \iff \min_{t}\; \|\mathbf{p}(t) - (x_c, y_c)\| \leq r_s$$

The coverage rate is:

$$C = \frac{\bigl|\{(x_c, y_c) : \text{scanned}\}\bigr|}{W \cdot H} \times 100\%$$

### Energy Model

Flight energy separates into cruise energy along scan strips and hover/turn energy at the $N-1$
heading reversals:

$$E = P_{cruise} \cdot \frac{L_{scan}}{v} + P_{hover} \cdot t_{turn} \cdot (N - 1)$$

where:

- $L_{scan} = N \cdot H$ — total scan-strip distance (m)
- $v = 10$ m/s — cruise speed
- $P_{cruise} = 150$ W — motor power during level cruise
- $P_{hover} = 200$ W — elevated motor power during the deceleration/turn/acceleration at each
  strip end
- $t_{turn} = 4$ s — time to execute one 180° heading reversal at strip speed

The turn-leg translation ($d$ metres at $v$ m/s) contributes additional cruise energy; absorbing it
into $L$ gives the unified expression:

$$E = P_{cruise} \cdot \frac{L}{v} + (P_{hover} - P_{cruise}) \cdot t_{turn} \cdot (N - 1)$$

### Redundant Coverage (Overlap Metric)

The mean number of times each scanned cell is overflown measures redundancy:

$$\bar{n}_{overlap} = \frac{\sum_{(x_c,y_c)} n(x_c, y_c)}{W \cdot H}$$

where $n(x_c, y_c)$ is the scan count of cell $(x_c, y_c)$. For the optimal configuration
$\bar{n}_{overlap} \approx 1.0$; for the over-lapping configuration
$\bar{n}_{overlap} > 1$.

---

## Implementation

```python
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

# Key constants
AREA_W      = 200.0    # m — cross-track width of search area
AREA_H      = 200.0    # m — along-track height of search area
R_SENSOR    = 5.0      # m — sensor footprint radius
V_CRUISE    = 10.0     # m/s — cruise speed
P_CRUISE    = 150.0    # W — cruise power
P_HOVER     = 200.0    # W — turn/hover power
T_TURN      = 4.0      # s — time per 180-degree heading reversal
DT          = 0.5      # s — simulation timestep

# Three comparison configurations: (label, overlap_ratio)
CONFIGS = [
    ("Under-lap (d=12m, rho=0%)",  0.0,  12.0),
    ("Optimal  (d=10m, rho=0%)",   0.0,  10.0),
    ("Over-lap (d=7m,  rho=30%)",  0.30,  7.0),
]

def generate_lawnmower_waypoints(area_w, area_h, strip_d, x0=0.0, y0=0.0):
    """Return ordered list of (x, y) waypoints for a boustrophedon scan."""
    n_strips = int(np.ceil(area_w / strip_d))
    waypoints = []
    for i in range(n_strips):
        y_centre = y0 + (i + 0.5) * strip_d
        if i % 2 == 0:   # left to right
            waypoints.append((x0,          y_centre))
            waypoints.append((x0 + area_h, y_centre))
        else:             # right to left
            waypoints.append((x0 + area_h, y_centre))
            waypoints.append((x0,          y_centre))
    return waypoints, n_strips

def simulate_coverage(waypoints, area_w, area_h, r_sensor, dt, v_cruise, x0=0.0, y0=0.0):
    """Simulate drone flight along waypoints; return coverage grid and trajectory."""
    # Discretise area into 1x1 m cells
    xs = np.arange(x0 + 0.5, x0 + area_h + 0.5, 1.0)
    ys = np.arange(y0 + 0.5, y0 + area_w + 0.5, 1.0)
    grid = np.zeros((len(ys), len(xs)), dtype=np.int32)   # scan-count grid
    cx, cy = np.meshgrid(xs, ys)  # cell centre coordinates

    trajectory = []
    pos = np.array(waypoints[0], dtype=float)
    trajectory.append(pos.copy())

    for wp in waypoints[1:]:
        target = np.array(wp, dtype=float)
        direction = target - pos
        dist = np.linalg.norm(direction)
        if dist < 1e-6:
            continue
        unit = direction / dist
        steps = int(np.ceil(dist / (v_cruise * dt)))
        step_dist = dist / steps
        for _ in range(steps):
            pos = pos + unit * step_dist
            trajectory.append(pos.copy())
            # Mark cells within sensor radius
            dist_to_cells = np.sqrt((cx - pos[0])**2 + (cy - pos[1])**2)
            grid[dist_to_cells <= r_sensor] += 1

    return np.array(trajectory), grid

def compute_metrics(n_strips, strip_d, area_h, grid):
    """Compute path length, energy, coverage rate, mean overlap."""
    L = n_strips * area_h + (n_strips - 1) * strip_d
    E = P_CRUISE * (L / V_CRUISE) + (P_HOVER - P_CRUISE) * T_TURN * (n_strips - 1)
    total_cells = grid.size
    covered = np.sum(grid > 0)
    coverage_pct = 100.0 * covered / total_cells
    mean_overlap = np.sum(grid) / total_cells
    return L, E, coverage_pct, mean_overlap

def run_simulation():
    results = []
    for label, rho, strip_d in CONFIGS:
        waypoints, n_strips = generate_lawnmower_waypoints(AREA_W, AREA_H, strip_d)
        traj, grid = simulate_coverage(waypoints, AREA_W, AREA_H, R_SENSOR, DT, V_CRUISE)
        L, E, cov, overlap = compute_metrics(n_strips, strip_d, AREA_H, grid)
        results.append({
            "label": label, "strip_d": strip_d, "n_strips": n_strips,
            "waypoints": waypoints, "trajectory": traj, "grid": grid,
            "path_length_m": L, "energy_J": E,
            "coverage_pct": cov, "mean_overlap": overlap,
        })
        print(f"{label}")
        print(f"  Strips: {n_strips}  |  Path: {L:.0f} m  |  Energy: {E/1000:.2f} kJ")
        print(f"  Coverage: {cov:.1f}%  |  Mean overlap: {overlap:.2f}x")
    return results
```

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Search area | 200 × 200 m |
| Sensor footprint radius $r_s$ | 5 m |
| Effective strip width (footprint diameter) | 10 m |
| Drone cruise speed $v$ | 10 m/s |
| Cruise power $P_{cruise}$ | 150 W |
| Hover/turn power $P_{hover}$ | 200 W |
| Turn time per reversal $t_{turn}$ | 4 s |
| Grid cell size | 1 × 1 m |
| Simulation timestep $\Delta t$ | 0.5 s |
| Under-lap strip spacing $d$ | 12 m (0% overlap, 2 m gap) |
| Optimal strip spacing $d^*$ | 10 m (0% overlap, tangent) |
| Over-lap strip spacing $d$ | 7 m (30% overlap) |
| Under-lap strip count $N$ | 17 strips |
| Optimal strip count $N$ | 20 strips |
| Over-lap strip count $N$ | 29 strips |

---

## Expected Output

- **Coverage map grid**: 2D top-down heatmap for each configuration showing scan-count per cell
  (white = 0 scans, light blue = 1 scan, dark blue = 2+ scans); flight trajectory overlaid as a
  thin orange line; sensor footprint circles drawn at representative waypoints.
- **Flight path diagram**: top-down view of the full boustrophedon waypoint sequence for each
  configuration, colour-coded by strip index, with turn legs shown as dashed segments.
- **Comparison bar chart**: three grouped bars (one group per configuration) showing (1) coverage
  rate %, (2) total path length m, and (3) total energy kJ, enabling direct trade-off assessment.
- **Coverage vs distance curve**: for each configuration, cumulative coverage rate (y-axis) plotted
  against distance flown (x-axis); the optimal and over-lap lines reach 100% while the under-lap
  line plateaus below 100%.
- **Mean overlap bar chart**: mean scan count per cell for the three configurations, illustrating
  the redundancy penalty of over-lapping.
- **Summary table** (printed to console): strip count, path length, energy, coverage rate, and mean
  overlap for each configuration.

---

## Extensions

1. **Non-rectangular areas**: replace the rectangular boundary with an arbitrary convex or
   concave polygon; compute the boustrophedon decomposition using the canonical rotational sweep
   approach and compare with a frontier-based coverage planner.
2. **Variable altitude and GSD**: model the sensor footprint as a function of flight altitude
   ($r_s = \tan(\theta_{FOV}/2) \cdot z$); optimise altitude jointly with strip spacing to minimise
   flight time subject to a minimum ground-sampling-distance (GSD) constraint.
3. **Wind drift compensation**: add a constant crosswind $\mathbf{v}_{wind}$ that displaces the
   drone laterally during straight segments; adjust strip spacing to maintain full coverage
   accounting for worst-case lateral displacement at each strip midpoint.
4. **Battery-constrained multi-pass**: if the area is larger than a single battery charge can cover,
   partition the area into sub-rectangles, plan a separate lawnmower pass per sub-rectangle,
   and minimise the number of battery swaps (see S032).
5. **Adaptive strip width**: use online coverage feedback from the scanned grid to dynamically
   tighten strip spacing in regions where terrain variation or sensor noise reduces the effective
   footprint radius below $r_s$.
6. **Multi-drone parallel coverage**: assign non-overlapping column bands to $K$ drones; use the
   Hungarian algorithm to match drones to bands minimising total completion time; compare with
   the single-drone baseline (see S049).

---

## Related Scenarios

- Prerequisites: [S041 Wildfire Boundary Scan](S041_wildfire_boundary_scan.md), [S042 Missing Person Search](S042_missing_person_search.md)
- Follow-ups: [S049 Dynamic Zone Assignment](S049_dynamic_zone_assignment.md) (multi-drone parallel coverage), [S050 Swarm SLAM](S050_swarm_slam.md) (simultaneous mapping)
- Algorithmic cross-reference: [S032 Charging Queue](../../02_logistics_delivery/S032_charging_queue.md) (battery-constrained multi-pass), [S067 Spray Overlap Optimization](../../04_industrial_agriculture/S067_spray_overlap_optimization.md) (same strip-width trade-off in agriculture)
