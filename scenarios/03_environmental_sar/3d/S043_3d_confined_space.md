# S043 3D Upgrade — Confined Space Exploration

**Domain**: Environmental & SAR | **Difficulty**: ⭐⭐⭐⭐ | **Status**: `[ ]` Not started
**Based on**: [S043 original](../S043_confined_space.md)

---

## What Changes in 3D

The original S043 locks the drone at a single altitude ($z = \text{const}$) and represents the environment as a flat $60 \times 60$ occupancy grid. All lidar beams are cast in the horizontal plane only, the Bug2 planner operates purely in $(x, y)$, and gap detection is a 1-D scan along a single horizontal row. This collapses every multi-storey real-world confined space — a collapsed apartment block, a parking structure, a mine shaft — into one floor.

This 3D variant extends the environment to a **multi-floor voxel map** ($60 \times 60 \times 20$ cells, $\Delta = 0.5$ m, spanning $30 \times 30 \times 10$ m). The building is modelled as a stack of rooms connected by stairwells and vertical shafts. The drone now reasons about:

- **Ceiling height constraints** that limit horizontal traversal at a given altitude slice.
- **Vertical passages** (stairwells, elevator shafts, rubble ramps) that must be detected and classified before the drone commits to a floor transition.
- **3D occupancy-grid SLAM**: lidar beams are cast in a 3D hemisphere, updating a volumetric log-odds voxel map rather than a 2D grid.
- **Altitude control**: the drone selects the vertical clearance slice that maximises headroom at every horizontal bottleneck.

---

## Problem Definition

**Setup**: A rescue drone is deployed inside a multi-floor collapsed building. The interior occupies a
$30 \times 30 \times 10$ m bounding box, discretised into a $60 \times 60 \times 20$ voxel grid
at $\Delta = 0.5$ m resolution. Each floor is approximately 2.5 m high (5 voxel layers), yielding
four nominal floors (F0–F3), connected by stairwells of width $\geq g_{min} = 0.4$ m and height
clearance $\geq h_{min} = 0.6$ m. Rubble piles block portions of each floor and may partially
obstruct vertical passages. The drone carries a 3D lidar with hemispherical sensing radius
$r_s = 2.0$ m, casting $N_b = 128$ beams distributed uniformly over the upper hemisphere plus
a horizontal ring, each with Gaussian range noise $w \sim \mathcal{N}(0, \sigma_r^2)$.

The drone body is modelled as a cylinder of radius $r_d = 0.15$ m and height $h_d = 0.20$ m.
A gap is passable only if there is a contiguous 3D tunnel of free voxels spanning at least
$g_{min}$ in both horizontal directions and $h_{min}$ in the vertical direction.

**Roles**:
- **Drone**: single explorer; starts at $\mathbf{p}_0 = (2.0,\, 2.0,\, 0.5)$ m (ground-floor entry).
  No return-to-base constraint within the mission horizon.
- **Environment**: static 3D obstacle field (walls, floor slabs, rubble columns, ceiling debris)
  encoded in a ground-truth voxel occupancy tensor unknown to the drone at $t = 0$.

**Objective**: Maximise the fraction of **3D reachable free voxels** mapped within mission horizon
$T_{max} = 600$ s, while never entering a voxel whose occupancy probability exceeds $p_{occ} = 0.65$
and never squeezing through a passage narrower than $g_{min}$ or lower than $h_{min}$.

**Exploration strategies** (three compared):
1. **2D-slice baseline** — frontier search restricted to the current floor slice; no floor
   transitions; z held fixed per floor.
2. **3D frontier-nearest** — volumetric frontier extraction; BFS on the 3D voxel map;
   nearest reachable frontier voxel selected globally across all floors.
3. **3D frontier + vertical-passage planner** — as above, but adds a dedicated
   vertical-passage detector that identifies stairwell voxel clusters and inserts them as
   high-priority frontier waypoints whenever the current floor is fully mapped.

---

## Mathematical Model

### 3D Voxel Map and Log-Odds Update

The map is a voxel tensor $\mathcal{M}$ of $N_x \times N_y \times N_z$ cells. Each voxel $v$
stores a log-odds belief:

$$l(v) = \log \frac{p(v \mid z_{1:t})}{1 - p(v \mid z_{1:t})}$$

The 3D inverse sensor model updates each voxel along the ray from the drone position
$\mathbf{p} = (x, y, z)$ in direction $\hat{\mathbf{d}}$:

$$l_t(v) = l_{t-1}(v) + \begin{cases}
l_{occ} & \text{if } v \text{ is the hit voxel of the ray} \\
l_{free} & \text{if } v \text{ lies along the ray before the hit} \\
0 & \text{otherwise}
\end{cases}$$

with $l_{occ} = +0.85$, $l_{free} = -0.40$, clamped to $[l_{min}, l_{max}] = [-2.0,\, 3.5]$.
The recovered probability:

$$p(v) = \frac{e^{l(v)}}{1 + e^{l(v)}}$$

A voxel is **occupied** if $p(v) > 0.65$, **free** if $p(v) < 0.35$, **unknown** otherwise.

### 3D Lidar Beam Directions

Beams are distributed over a hemisphere ($\theta \in [0°, 180°]$, $\phi \in [0°, 360°)$) using
the Fibonacci lattice method for near-uniform angular spacing, plus a dense horizontal ring
($\theta = 90°$, $N_{ring} = 36$ beams). For beam $b$ with direction
$\hat{\mathbf{d}}_b = (\sin\theta_b \cos\phi_b,\; \sin\theta_b \sin\phi_b,\; \cos\theta_b)$,
the noisy range measurement is:

$$z_b = h_b(\mathbf{p}) + w_b, \qquad w_b \sim \mathcal{N}(0,\, \sigma_r^2), \quad \sigma_r = 0.05 \text{ m}$$

where $h_b(\mathbf{p})$ is the true range to the nearest occupied voxel along $\hat{\mathbf{d}}_b$
(3D DDA ray-march on the ground-truth voxel grid), truncated at $r_s = 2.0$ m.

### 3D Frontier Extraction

A **3D frontier voxel** is a free voxel adjacent to at least one unknown voxel in the 6-connected
neighbourhood:

$$\mathcal{F}_{3D} = \left\{ v \in \mathcal{M} \;\middle|\; p(v) < 0.35 \;\text{ and }\;
  \exists\, v' \in \mathcal{N}_6(v):\; 0.35 \leq p(v') \leq 0.65 \right\}$$

Frontier voxels are clustered by 3D connected-component labelling; the centroid of each cluster
becomes a candidate waypoint. The 3D BFS navigator selects:

$$\mathbf{g}^* = \underset{\mathbf{g}_f \in \text{clusters}}{\arg\min}\; d_{nav}^{3D}(\mathbf{p},\, \mathbf{g}_f)$$

where $d_{nav}^{3D}$ is the 3D BFS shortest path length through free voxels, respecting the
cylindrical body constraint: each candidate voxel must be part of a free tunnel satisfying both
$g_{min}$ horizontal clearance and $h_{min}$ vertical clearance.

### Vertical Passage Detection

A **stairwell cluster** is a connected set of frontier voxels $S$ spanning at least two floor
levels ($\Delta z \geq 2.5$ m) with a continuous free-voxel channel of cross-section
$\geq g_{min} \times g_{min}$ and height $\geq h_{min}$:

$$S = \left\{ v \in \mathcal{F}_{3D} \;\middle|\; v_z^{max} - v_z^{min} \geq 2.5\,\text{m}
  \;\text{ and }\; A_{cross}(S) \geq g_{min}^2 \right\}$$

where $A_{cross}(S)$ is the minimum free cross-sectional area of $S$ over all horizontal slices
within the cluster's altitude range. Stairwell clusters are assigned a priority bonus
$\Delta p_{stair} = +2.0$ added to the BFS path cost metric (lower effective cost), making them
preferentially selected when the current floor coverage exceeds $\eta_{floor} = 0.80$.

### 3D Gap Clearance Check

Before committing to a motion step $\mathbf{p} \to \mathbf{p} + \delta \mathbf{p}$, a cylindrical
swept-volume check verifies that all voxels within the drone's bounding cylinder centred on the
path segment are free:

$$\text{passable} = \bigwedge_{k=0}^{K} \bigwedge_{\|\boldsymbol{\rho}\|_\infty \leq r_d}
  \left[ p\!\left(\mathbf{p} + \frac{k}{K}\delta\mathbf{p} + \boldsymbol{\rho}\right) < 0.65 \right]$$

where $\boldsymbol{\rho}$ ranges over the cylinder cross-section and $K = 5$ steps subdivide
the path segment. Vertical clearance is separately checked over $[z - h_d/2,\; z + h_d/2 + h_{margin}]$
with $h_{margin} = 0.1$ m.

### Altitude Selection at Horizontal Bottlenecks

When the horizontal BFS path passes through a corridor voxel column $(x_c, y_c)$, the drone
selects the altitude $z^*$ that maximises vertical headroom:

$$z^*(x_c, y_c) = \underset{z \in [z_{floor}, z_{ceil}]}{\arg\max}\;
  \min\!\left(\Delta z_{above}(x_c, y_c, z),\; \Delta z_{below}(x_c, y_c, z)\right)$$

where $\Delta z_{above}$ and $\Delta z_{below}$ are the distances to the nearest occupied voxel
above and below $(x_c, y_c, z)$, estimated from the current voxel map. The altitude command is
then smoothed:

$$z_{cmd}(t+1) = z(t) + \text{clip}\!\left(z^* - z(t),\; -v_{z,max}\Delta t,\; +v_{z,max}\Delta t\right)$$

with $v_{z,max} = 0.3$ m/s.

### 3D Coverage Metric

Let $\mathcal{C}_{reach}^{3D}$ be the set of ground-truth reachable free voxels (3D BFS from
$\mathbf{p}_0$ respecting both $g_{min}$ and $h_{min}$). The volumetric exploration coverage:

$$\text{Coverage}_{3D}(t) = \frac{\left|\left\{v \in \mathcal{C}_{reach}^{3D} \;:\; p_t(v) < 0.35\right\}\right|}{|\mathcal{C}_{reach}^{3D}|} \times 100\%$$

A per-floor breakdown is also computed:

$$\text{Coverage}_{F_k}(t) = \frac{\left|\left\{v \in \mathcal{C}_{reach}^{3D} \;:\; p_t(v) < 0.35,\; k\,h_{floor} \leq v_z < (k{+}1)\,h_{floor}\right\}\right|}{\left|\left\{v \in \mathcal{C}_{reach}^{3D} \;:\; k\,h_{floor} \leq v_z < (k{+}1)\,h_{floor}\right\}\right|} \times 100\%$$

---

## Key 3D Additions

- **Volumetric voxel map**: $60 \times 60 \times 20$ cells replacing the 2D $60 \times 60$ grid.
- **3D lidar beam distribution**: Fibonacci hemisphere lattice + horizontal ring, $N_b = 128$ beams total.
- **3D DDA ray-marching**: voxel-traversal algorithm (Amanatides & Woo) replaces 2D ray-cast loop.
- **Vertical passage detection**: stairwell cluster identification by cross-sectional area and altitude span.
- **Altitude optimisation**: per-corridor headroom maximisation with velocity-limited altitude command.
- **Cylindrical swept-volume check**: replaces 1D gap-width scan for 3D body clearance validation.
- **Per-floor coverage breakdown**: four floor-level $\text{Coverage}_{F_k}(t)$ curves plus total volumetric coverage.

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Environment size | 30 × 30 × 10 m |
| Grid resolution $\Delta$ | 0.5 m/voxel |
| Voxel grid dimensions | 60 × 60 × 20 |
| Number of floors | 4 (F0–F3, each ~2.5 m) |
| Sensor radius $r_s$ | 2.0 m |
| Number of lidar beams $N_b$ | 128 (92 hemisphere + 36 horizontal ring) |
| Range noise std dev $\sigma_r$ | 0.05 m |
| Log-odds hit increment $l_{occ}$ | +0.85 |
| Log-odds free decrement $l_{free}$ | −0.40 |
| Log-odds clamp $[l_{min}, l_{max}]$ | [−2.0, 3.5] |
| Occupied probability threshold $p_{occ}$ | 0.65 |
| Free probability threshold $p_{free}$ | 0.35 |
| Drone body radius $r_d$ | 0.15 m |
| Drone body height $h_d$ | 0.20 m |
| Min. horizontal gap $g_{min}$ | 0.40 m |
| Min. vertical clearance $h_{min}$ | 0.60 m |
| Max. vertical speed $v_{z,max}$ | 0.3 m/s |
| Drone cruise speed $v_d$ | 0.5 m/s |
| Mission horizon $T_{max}$ | 600 s |
| Simulation timestep $\Delta t$ | 0.1 s |
| Start position $\mathbf{p}_0$ | (2.0, 2.0, 0.5) m |
| Stairwell priority bonus $\Delta p_{stair}$ | +2.0 (BFS cost offset) |
| Floor-transition trigger $\eta_{floor}$ | 80% current-floor coverage |
| z range | 0.3 – 9.5 m |

---

## Expected Output

- **3D voxel map snapshots**: four isometric views of the voxel occupancy map at $t = 0, 200, 400, 600$ s; free voxels white, unknown grey, occupied black; floor slabs rendered as translucent horizontal planes; drone position shown as a red sphere.
- **Per-floor coverage curves**: four subplots (F0–F3) showing $\text{Coverage}_{F_k}(t)$ for each strategy; vertical dashed lines indicate floor-transition events for the vertical-passage planner.
- **Total 3D coverage vs. time**: three lines (2D-slice baseline / 3D frontier-nearest / 3D frontier + vertical-passage planner) from $t = 0$ to $T_{max}$; horizontal dashed line at 100%.
- **Altitude time series**: $z(t)$ for the 3D strategies, annotated with stairwell traversal events and altitude-optimisation adjustments.
- **Vertical passage detection log**: table of detected stairwell clusters — cluster centroid, altitude span, minimum cross-section area, detection timestamp.
- **3D trajectory plot**: full $(x, y, z)$ polyline coloured by floor level; stairwell traversal segments highlighted in orange; start marked with a green sphere, end with a red cross.
- **Animation (GIF)**: horizontal slice view stepping through floors, voxel map updating in real time, drone icon moving and climbing between floors, frontier voxels pulsing in cyan.

---

## Extensions

1. **Multi-drone floor partitioning**: deploy $N = 4$ drones, one per floor, entering through separate ground-floor openings; drones share the volumetric map via a log-odds merge server and coordinate floor assignments using a max-dispersion auction.
2. **Victim heat signature in 3D**: extend the 2D heat-sensor Bayesian model to 3D; model thermal plume rise as a buoyant Gaussian ($T(z) \propto e^{-\lambda z}$); fuse heat detections into a 3D victim probability voxel map.
3. **Dynamic floor collapse simulation**: randomly flip free voxels to occupied during the mission (debris settling model); trigger re-planning when a previously confirmed free passage is blocked; quantify replanning frequency vs. collapse rate.
4. **Energy-aware altitude routing**: model battery consumption as $P = P_{hover} + k_v v^2 + k_z v_z^2$; incorporate energy cost into the 3D BFS path metric to balance exploration coverage against remaining battery.
5. **Structural stability map overlay**: assign each voxel a load-bearing risk score $V_h(v)$ proportional to the number of occupied voxels directly above it; weight BFS path cost as $c_{path} = d_{nav}^{3D} + \alpha V_h$ to route the drone away from structurally unstable zones.

---

## Related Scenarios

- Original 2D version: [S043](../S043_confined_space.md)
- Volumetric SLAM reference: [S050 Autonomous SLAM Mapping](../S050_slam.md)
- 3D frontier exploration background: [S001 3D Basic Intercept](../../01_pursuit_evasion/S001_basic_intercept.md), [S002 3D Evasive Maneuver](../../01_pursuit_evasion/3d/S002_3d_evasive_maneuver.md)
- Post-mapping structural survey: [S044 Wall Crack Inspection](../S044_wall_crack.md)
- Multi-drone coordination pattern: [S042 Missing Person Search](../S042_missing_person.md)
