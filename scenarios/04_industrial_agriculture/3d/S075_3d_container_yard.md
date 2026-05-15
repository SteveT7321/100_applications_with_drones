# S075 3D Upgrade — Container Yard Inventory

**Domain**: Industrial & Agriculture | **Difficulty**: ⭐⭐⭐ | **Status**: `[ ]` Not Started
**Based on**: [S075 original](../S075_container_yard.md)

---

## What Changes in 3D

The original S075 treats altitude as a simple scalar rule: fly at $z = 5.0\ \text{m}$ unless a
stack exceeds that height, in which case climb to $z_{stack} + 1.0\ \text{m}$. Scan waypoints
are placed at the front face of each container at a fixed mid-height derived from
$\min(z_{hi}/2,\, Z_{cruise})$, so taller stacks are never scanned on their upper tiers and
side faces are ignored entirely. Only one scan position per container is generated.

This upgrade fully exploits the 3D structure of the yard:

1. **Stacked container tiers up to 5 high** ($z_{max} = 12.5\ \text{m}$) — each tier hosts an
   independent barcode panel on its front face, requiring a dedicated scan waypoint at the correct
   altitude for that tier.
2. **Per-tier scan waypoint generation** — every tier of every container stack generates one scan
   position. For a 5-tier stack of 10 containers in a row this yields 50 scan stops in that row
   alone; the total waypoint count scales with the yard's tier distribution.
3. **Altitude planning per stack height** — the drone's cruise altitude between scan segments is
   determined dynamically from the maximum stack height along each inter-waypoint corridor, not a
   global fixed value, minimising unnecessary altitude excursions.
4. **Side-face label scanning trajectory** — the drone approaches each tier from a $0.8\ \text{m}$
   standoff on the face containing the barcode label, then executes a controlled vertical descent
   (or ascent) sweep of $\Delta z = 0.5\ \text{m}$ centred on the tier mid-height to account for
   label placement uncertainty.

---

## Problem Definition

**Setup**: A single drone performs a full inventory audit of a $100 \times 80\ \text{m}$ container
yard. Fifty ISO-standard shipping containers (each $6 \times 2.5 \times 2.5\ \text{m}$) are
arranged in six rows. In this 3D upgrade, individual stacks reach up to **5 tiers** high
($z_{max} = 12.5\ \text{m}$). The drone must visit one barcode scan position per **tier per
container face**, resulting in $N_{scan} = \sum_{k=1}^{50} n_k$ total waypoints, where $n_k$ is
the stack height (number of tiers) of container $k$.

Navigation uses a corridor-aware altitude planner: before traversing from waypoint $i$ to waypoint
$j$, the planner queries the maximum obstacle height $z_{corr}$ along the straight-line horizontal
projection of the segment and sets the transit altitude to $z_{transit} = z_{corr} + z_{margin}$.
RRT* plans a collision-free 3D path through the obstacle field. At each scan waypoint the drone
performs a vertical hover sweep of amplitude $\pm \Delta z_{sweep} / 2$ centred on the tier
mid-height, then dwells for $T_{dwell} = 1.5\ \text{s}$.

**Roles**:
- **Drone**: single UAV executing an RRT*-planned 3D trajectory at $v_{scan} = 1.0\ \text{m/s}$
  with a double-integrator dynamics model and GPS noise $\sigma_{GPS} = 0.1\ \text{m}$.
- **Containers**: 50 static box obstacles. Stack heights drawn from $n_k \sim \mathcal{U}\{1,\ldots,5\}$,
  each occupying $\mathcal{B}_k = [x_k^{lo},x_k^{hi}] \times [y_k^{lo},y_k^{hi}] \times [0, n_k \times 2.5]$.
- **Scan positions**: $N_{scan}$ waypoints (one per tier per container), ordered by a tier-aware
  nearest-neighbour schedule that clusters waypoints by row and tier before greedy ordering.

**Objective**: Complete all $N_{scan}$ barcode scans with zero collisions while minimising total
mission time $T_{mission}$. Analyse how mission time and path length scale with the tier
distribution of the yard.

---

## Mathematical Model

### 3D Container Obstacle Model

Each container $k$ in stack $s$ at row $r$ occupies an axis-aligned bounding box extending from
ground level to the top of all $n_k$ tiers:

$$\mathcal{B}_k = [x_k^{lo},\; x_k^{hi}] \times [y_k^{lo},\; y_k^{hi}] \times [0,\; h_k]$$

where $h_k = n_k \cdot h_{tier}$, $h_{tier} = 2.5\ \text{m}$, and $n_k \in \{1, 2, 3, 4, 5\}$.

A drone configuration $\mathbf{q} = (x, y, z)^\top$ is collision-free if:

$$\text{free}(\mathbf{q}) = \bigwedge_{k=1}^{50}
  \left[ d_\infty\!\left(\mathbf{q},\, \mathcal{B}_k\right) > r_d \right]$$

where $r_d = 0.25\ \text{m}$ is the drone body radius and $d_\infty(\mathbf{q}, \mathcal{B}_k)$
is the $\ell^\infty$ clearance from $\mathbf{q}$ to the expanded bounding box surface.

### Per-Tier Scan Waypoint Generation

For container $k$ with $n_k$ tiers, tier $m \in \{1, \ldots, n_k\}$ has its barcode panel
centred at height:

$$z_{tier}(m) = \left(m - \tfrac{1}{2}\right) \cdot h_{tier}$$

The scan waypoint for tier $m$ of container $k$, approached from the front face (low-$y$ side),
is:

$$\mathbf{w}_{k,m} = \begin{pmatrix} x_k^{lo} + L_{cont}/2 \\ y_k^{lo} - d_{scan} \\ z_{tier}(m) \end{pmatrix}$$

where $L_{cont} = 6.0\ \text{m}$ is the container length and $d_{scan} = 0.8\ \text{m}$ is the
standoff distance. The total number of scan waypoints is:

$$N_{scan} = \sum_{k=1}^{50} n_k$$

For a uniform tier-5 yard this yields $N_{scan} = 250$; for the mixed distribution it averages
approximately $N_{scan} \approx 150$.

### Vertical Hover Sweep

At each scan waypoint $\mathbf{w}_{k,m}$, the drone executes a vertical sweep of amplitude
$\Delta z_{sweep} = 0.5\ \text{m}$ centred on $z_{tier}(m)$ before the dwell begins. The sweep
trajectory is:

$$z_{sweep}(t) = z_{tier}(m) + \frac{\Delta z_{sweep}}{2}
  \sin\!\left(\frac{2\pi t}{T_{sweep}}\right), \quad t \in [0,\, T_{sweep}]$$

with $T_{sweep} = 1.0\ \text{s}$. This accounts for label-placement uncertainty of
$\pm 0.25\ \text{m}$ in the vertical direction without requiring precise label position
knowledge.

### Corridor-Aware Altitude Planning

Before traversing from waypoint $\mathbf{w}_i$ to $\mathbf{w}_j$, the planner finds the
maximum obstacle height along the horizontal projection of the inter-waypoint segment:

$$z_{corr}(i, j) = \max_{k : \mathcal{B}_k \cap \Pi_{ij} \neq \emptyset} h_k$$

where $\Pi_{ij}$ is the cylindrical tube of radius $r_{tube} = r_d + 0.5\ \text{m}$ around
the segment $\overline{\mathbf{w}_i^{xy}\, \mathbf{w}_j^{xy}}$ projected onto the $xy$-plane.
The transit altitude is then:

$$z_{transit}(i, j) = z_{corr}(i, j) + z_{margin}, \quad z_{margin} = 1.5\ \text{m}$$

The drone climbs to $z_{transit}$ before the horizontal translation phase, then descends to
$z_{tier}(m_j)$ on approach to the next scan waypoint. This creates a two-segment altitude
profile — a climb phase followed by a descent phase — rather than a single global cruise
altitude.

### RRT* in 3D with Tier-Expanded Obstacle Field

RRT* operates in the full workspace
$\mathcal{W} = [0, 100] \times [0, 80] \times [0, 15]\ \text{m}$. The obstacle field is
expanded to include all $n_k \leq 5$ tier heights. The rewire radius uses the 3D formula:

$$r = \min\!\left(\gamma_{RRT^*}\left(\frac{\ln|V|}{|V|}\right)^{1/3},\; \eta\right)$$

$$\gamma_{RRT^*} = \left(2 \cdot \frac{4}{3}\right)^{1/3}
  \left(\frac{|\mathcal{W}_{free}|}{\zeta_3}\right)^{1/3}$$

where $|\mathcal{W}_{free}|$ is the free-space volume (workspace minus inflated obstacle
volumes) and $\zeta_3 = 4\pi/3$ is the 3D unit ball volume.

### Tier-Aware Nearest-Neighbour Scheduling

The scan waypoints $\{\mathbf{w}_{k,m}\}$ are ordered by a two-level greedy schedule:

1. **Row clustering**: group waypoints by their row index $r$.
2. **Tier sweep within row**: within each row, sort by ascending tier $m$, then by container
   position $x_k$, to encourage a bottom-up scanning pattern that minimises altitude reversals.
3. **Row-to-row transition**: connect rows using the nearest-neighbour heuristic on the 3D
   distance between the last waypoint of the current row and all unvisited first-tier waypoints
   of remaining rows.

The ordering minimises the total altitude change cost:

$$\Delta z_{total} = \sum_{i=1}^{N_{scan}-1} |z_{i+1} - z_i|$$

alongside the Euclidean path length.

### Scan Dwell and Stabilisation Model

At scan waypoint $\mathbf{w}_{k,m}$, GPS position error at arrival is:

$$\boldsymbol{\eta} \sim \mathcal{N}(\mathbf{0},\, \sigma_{GPS}^2 \mathbf{I}_3), \quad
  \sigma_{GPS} = 0.1\ \text{m}$$

Stabilisation time:

$$T_{stab,j} = \frac{\|\boldsymbol{\eta}_j\|}{K_{stab} \cdot v_{scan}}, \quad K_{stab} = 0.5$$

Total time per scan stop (sweep + stabilise + dwell):

$$T_{stop,j} = T_{sweep} + T_{stab,j} + T_{dwell}$$

### Mission Time

Total mission time:

$$T_{mission} = \frac{L_{path}}{v_{scan}} + \sum_{j=1}^{N_{scan}} T_{stop,j}$$

where $L_{path}$ is the smoothed 3D RRT* path length. The altitude profile contribution
to path length is:

$$L_{vert} = \sum_{i=1}^{N_{wp}-1} |z_{i+1} - z_i|$$

which increases with higher tier counts. Path smoothness is quantified by the cumulative
turning angle (CTA):

$$\text{CTA} = \sum_{i=1}^{N_{wp}-2}
  \arccos\!\left(\frac{(\mathbf{w}_{i+1}-\mathbf{w}_i)\cdot(\mathbf{w}_{i+2}-\mathbf{w}_{i+1})}
  {\|\mathbf{w}_{i+1}-\mathbf{w}_i\|\,\|\mathbf{w}_{i+2}-\mathbf{w}_{i+1}\|}\right)$$

---

## Key Parameters

| Parameter | Symbol | Value |
|-----------|--------|-------|
| Yard dimensions | $L_x \times L_y$ | 100 × 80 m |
| Container size | $l \times w \times h_{tier}$ | 6 × 2.5 × 2.5 m |
| Number of containers | $N_{cont}$ | 50 |
| Max stack tiers | $n_{max}$ | 5 (vs. 3 in S075) |
| Max stack height | $z_{max}$ | 12.5 m (vs. 7.5 m) |
| Drone body radius | $r_d$ | 0.25 m |
| Scan standoff distance | $d_{scan}$ | 0.8 m |
| Scan waypoints | $N_{scan}$ | $\sum_{k} n_k$ (approx. 150) |
| Vertical sweep amplitude | $\Delta z_{sweep}$ | 0.5 m |
| Sweep period | $T_{sweep}$ | 1.0 s |
| Altitude margin | $z_{margin}$ | 1.5 m (vs. 1.0 m) |
| Corridor tube radius | $r_{tube}$ | 0.75 m |
| Inspection speed | $v_{scan}$ | 1.0 m/s |
| Barcode dwell time | $T_{dwell}$ | 1.5 s |
| Stabilisation threshold | $d_{stab}$ | 0.05 m |
| GPS noise std dev | $\sigma_{GPS}$ | 0.1 m |
| Workspace z range | $[z_{lo}, z_{hi}]$ | 0 – 15 m |
| RRT* step length | $\eta$ | 2.0 m |
| RRT* max iterations | $N_{iter}$ | 6 000 |
| A* grid resolution | $\Delta_{grid}$ | 0.5 m |
| A* grid dimensions | $N_x \times N_y \times N_z$ | 200 × 160 × 30 |
| Shortcut smoothing iterations | $N_{short}$ | 200 |
| Bézier blend fraction | $\alpha$ | 0.3 |

---

## Expected Output

- **3D yard overview**: container stacks rendered as grey rectangular prisms with height
  proportional to tier count (1–5 tiers, colour-mapped from light grey to dark grey); RRT*
  path drawn as a red polyline in full 3D showing altitude excursions to clear tall stacks;
  per-tier scan waypoints shown as green crosses at their correct $z$ positions; drone
  start/end positions marked.
- **Altitude time series**: $z(t)$ profile over the full mission showing the corridor-aware
  climb-and-descend pattern between scan stops; horizontal dashed lines at $h_{tier}$ multiples
  ($2.5, 5.0, \ldots, 12.5\ \text{m}$) indicate tier boundaries; coloured bands show scan
  dwell segments.
- **Waypoint count vs. tier distribution bar chart**: histogram of $n_k$ across the 50
  containers; overlay of resulting $N_{scan}$ and $T_{mission}$ as functions of mean tier height,
  demonstrating how mission complexity scales.
- **$xy$ and $xz$ projection plots**: top-down and side-elevation views of the full RRT* path,
  colour-coded by altitude; container footprints and heights visible in the $xz$ view; scan
  waypoints marked per tier.
- **Animation** (`FuncAnimation`): side-elevation ($xz$) 2D view of the drone traversing the
  planned path; drone shown as a red filled circle; a fading trail of the last 40 positions;
  green flash at each scan waypoint during the dwell phase; container cross-sections drawn as
  grey rectangles with tier boundaries indicated as horizontal lines; saved as
  `outputs/04_industrial_agriculture/s075_3d_container_yard/s075_3d_container_yard.gif`.

Terminal metrics printed at completion:

```
Metric                                  3D RRT*
--------------------------------------------------
Containers scanned                           50
Total tiers scanned                         153
Scan waypoints visited                      153
Path length (m)                           486.3
Vertical path component (m)              134.7
Cumulative turning angle (deg)           1240.5
Total altitude change (m)                482.6
Total scan stop time (s)                 577.1
Total mission time (s)                  1063.4
```

---

## Extensions

1. **Multi-face scanning**: extend waypoint generation to all four vertical faces of each
   container (front, back, left, right), requiring the drone to navigate around stacks and
   approach from multiple aisle directions; analyse which face ordering minimises total path
   length using a 2-opt TSP refinement on the 3D waypoint set.
2. **Top-face inspection**: add a nadir-pointing camera scan for the top face of each tier by
   flying directly overhead at $z = h_k + 1.0\ \text{m}$; combine the top-face and side-face
   scan schedules into a single unified tour using a mixed-direction TSP formulation.
3. **Dynamic stacking operations**: simulate a straddle carrier adding or removing container
   tiers during the mission; invalidate affected RRT* sub-trees and re-plan only the perturbed
   segments, measuring re-planning latency as a function of the number of affected waypoints.
4. **Battery-constrained multi-tier mission**: impose energy budget $E_{max}$ with altitude-
   dependent power consumption $P(v, v_z) = k_h v_z^2 + k_f v^2$; insert charging depot returns
   when remaining energy falls below a threshold and compare single-drone vs. two-drone relay
   strategies on total mission duration.
5. **Tier-uncertainty scanning**: model unknown tier heights (RFID-reported heights with
   $\sigma_{height} = 0.3\ \text{m}$ error); use a Kalman filter to maintain a belief over
   each stack's true height and adaptively adjust scan altitudes mid-mission as the drone
   observes actual container top elevations with its onboard rangefinder.

---

## Related Scenarios

- Original 2D version: [S075 Container Yard Inventory](../S075_container_yard.md)
- Vertical structure survey reference: [S076 Port Crane Inspection](../S076_port_crane_inspection.md)
- Multi-face coverage: [S077 Ship Hull Survey](../S077_ship_hull_survey.md)
- 3D path planning reference: [S043 Confined Space Exploration](../../03_environmental_sar/S043_confined_space.md)
- Scan scheduling fundamentals: [S069 Warehouse Inventory](../S069_warehouse_inventory.md)
