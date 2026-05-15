# S049 3D Upgrade — Dynamic Zone Assignment

**Domain**: Environmental & SAR | **Difficulty**: ⭐⭐⭐⭐ | **Status**: `[ ]` Not Started
**Based on**: [S049 original](../S049_dynamic_zone.md)

---

## What Changes in 3D

The original S049 operates entirely in the horizontal plane: all drone positions have a fixed
altitude (z = const), Voronoi partitioning is computed in $\mathbb{R}^2$, and the coverage metric
counts 2D grid cells only. This means the model cannot represent volumetric search tasks — building
rubble, dense forest canopy, multi-storey rubble piles — where survivors may be at varying heights
and drones must sweep discrete altitude layers.

In this 3D upgrade:

- The search volume becomes $400 \times 400 \times 60$ m, discretised into a 3D voxel grid.
- Zone partitioning uses **3D power-diagram (weighted) Voronoi** with generator points
  $\mathbf{p}_k \in \mathbb{R}^3$.
- Each drone sweeps its zone in horizontal altitude layers (boustrophedon per layer, then ascend
  to the next layer), implementing **altitude-layer scheduling**.
- **Vertical zone hand-off** occurs when a drone exhausts its battery: its unscanned voxels across
  all altitude layers are redistributed to active drones using 3D battery-weighted Voronoi.
- The coverage metric becomes a **volumetric fraction** of scanned voxels.
- A new **Jain fairness index** operates over per-drone remaining voxel counts rather than cell
  counts.

---

## Problem Definition

**Setup**: A $400 \times 400 \times 60$ m disaster rubble volume is divided into $N_z = 12$
horizontal altitude layers of thickness $\Delta z = 5$ m, spanning $z \in [2.5, 57.5]$ m (layer
centres). The horizontal grid is $80 \times 80$ cells at $\delta_{xy} = 5$ m resolution, giving a
total voxel count of $80 \times 80 \times 12 = 76{,}800$ voxels. A 3D prior probability density
$P(\mathbf{x})$ encodes the likelihood of a survivor being present at each voxel, modelled as a
sum of 3D Gaussian blobs centred at rubble hotspots.

Four drones depart from a ground-level staging area at the edge of the volume. The volume is
initially partitioned into four 3D Voronoi cells using each drone's 3D starting position as a
generator. Each drone is assigned a stack of altitude layers within its Voronoi cell and sweeps
them bottom-up (lowest layer first, since survivor probability is highest near ground level in
collapse scenarios). When a drone's SoC drops below 30% or it detects a survivor, its remaining
unscanned voxels — distributed across multiple altitude layers — are reallocated among the
surviving drones via a new 3D battery-weighted Voronoi recomputation, producing a **vertical zone
hand-off**.

**Roles**:
- **Drones** ($K = 4$): homogeneous quadrotors, sensor footprint radius $r_s = 8$ m (horizontal),
  vertical sensing half-height $r_z = 2.5$ m (one layer half-thickness), cruise speed $v = 6$ m/s,
  climb/descent rate $v_z = 2$ m/s, full battery range $D_{max} = 3000$ m (extended for vertical
  travel).
- **Survivors**: $N_s = 5$ survivors placed in 3D voxel positions sampled from $P(\mathbf{x})$.
- **Prior map** $P(\mathbf{x})$: 3D Gaussian mixture density over the voxel grid, with hotspots
  concentrated in the lower altitude layers.

**Objective**: Minimise total time to detect all survivors in the 3D volume while maintaining
balanced volumetric load distribution. The **volumetric coverage fraction** $C_{vol}(t)$ must
reach at least 90% within the mission time budget.

**Comparison strategies**:
1. **Static 3D Voronoi** — initial 3D partition fixed; no reallocation.
2. **Dynamic 3D Voronoi (equal weight)** — reallocation uses equal weights on current 3D positions.
3. **Dynamic 3D Weighted Voronoi (battery weight)** — reallocation uses SoC-proportional weights
   in full 3D.

---

## Mathematical Model

### 3D Search Volume and Prior Map

Discretise the $400 \times 400 \times 60$ m volume into voxels with centres:

$$\mathbf{x}_{ijk} = \bigl(i\,\delta_{xy},\; j\,\delta_{xy},\; z_0 + k\,\Delta z\bigr), \quad
  i,j \in \{0,\ldots,79\},\; k \in \{0,\ldots,11\}$$

where $\delta_{xy} = 5$ m, $\Delta z = 5$ m, and $z_0 = 2.5$ m (first layer centre).

The 3D prior is a normalised Gaussian mixture over voxels:

$$P(\mathbf{x}) = \frac{1}{Z} \sum_{m=1}^{M} \alpha_m\,
  \mathcal{N}\!\left(\mathbf{x};\,\boldsymbol{\mu}_m^{3D},\,
  \mathrm{diag}(\sigma_{xy,m}^2, \sigma_{xy,m}^2, \sigma_{z,m}^2)\right)$$

with $M = 4$ components, horizontal spreads $\sigma_{xy,m} \in [30, 70]$ m, and vertical spreads
$\sigma_{z,m} \in [5, 15]$ m to concentrate probability in lower layers.

### 3D Standard Voronoi Partition

Given 3D generator positions $\mathbf{p}_k \in \mathbb{R}^3$ for drones $k = 1,\ldots,K$, the
3D Voronoi cell for drone $k$ is:

$$V_k = \bigl\{\mathbf{x} \in \mathcal{V} : \|\mathbf{x} - \mathbf{p}_k\| \leq
  \|\mathbf{x} - \mathbf{p}_j\| \;\forall\, j \neq k\bigr\}$$

where $\mathcal{V} \subset \mathbb{R}^3$ is the search volume and $\|\cdot\|$ is the Euclidean
3D norm. Each voxel $\mathbf{x}_{ijk}$ is assigned to the drone whose 3D generator is closest.

### 3D Battery-Weighted Voronoi Partition

Replace 3D Euclidean distance with SoC-scaled distance:

$$V_k^w = \left\{\mathbf{x} \in \mathcal{V} :
  \frac{\|\mathbf{x} - \mathbf{p}_k\|_3}{w_k} \leq
  \frac{\|\mathbf{x} - \mathbf{p}_j\|_3}{w_j} \;\forall\, j \neq k\right\}$$

where $w_k = \mathrm{SoC}_k \in (0, 1]$ and $\|\cdot\|_3$ is the full 3D Euclidean norm. A drone
with more remaining battery absorbs a larger volumetric zone because its effective 3D distance to
any voxel is scaled down proportionally.

### Altitude-Layer Scheduling

Within a drone's 3D Voronoi zone $V_k^w$, voxels are grouped by altitude layer index
$k_z \in \{0, \ldots, 11\}$. The drone sweeps layers bottom-up:

$$\mathcal{L}_k^{(k_z)} = \bigl\{\mathbf{x}_{ijk_z} \in V_k^w\bigr\}, \quad k_z = 0, 1, \ldots, 11$$

Within each horizontal layer $\mathcal{L}_k^{(k_z)}$, voxels are visited in descending prior
probability order (priority-greedy boustrophedon). After completing layer $k_z$, the drone climbs
to $z = z_0 + (k_z + 1)\Delta z$ at rate $v_z$.

Estimated time to complete one layer sweep:

$$T_{layer} \approx \frac{A_k^{(k_z)}}{2 r_s \cdot v}$$

where $A_k^{(k_z)}$ is the horizontal area of the layer footprint within $V_k^w$ and $2r_s$ is
the effective swath width.

### 3D Battery Consumption Model

Battery consumption accounts for both horizontal travel and vertical climb:

$$\mathrm{SoC}_k(t) = 1 - \frac{d_k^{xy}(t) + \gamma\, d_k^z(t)}{D_{max}}$$

where $d_k^{xy}(t)$ is cumulative horizontal distance flown, $d_k^z(t)$ is cumulative vertical
distance climbed or descended, and $\gamma \geq 1$ is the vertical energy penalty factor (climbing
costs more power per metre than horizontal cruise). Reallocation triggers when:

$$\mathrm{SoC}_k(t) < \mathrm{SoC}_{thresh} = 0.30$$

### 3D Sensor Detection Model

Drone $k$ at 3D position $\mathbf{p}_k(t) = (p_x, p_y, p_z)$ detects survivor $i$ at
$\mathbf{s}_i = (s_x, s_y, s_z)$ if:

$$\sqrt{(p_x - s_x)^2 + (p_y - s_y)^2} \leq r_s \quad \text{and} \quad |p_z - s_z| \leq r_z$$

The corresponding voxel is marked scanned when the drone centre satisfies both horizontal and
vertical proximity conditions simultaneously.

### Volumetric Coverage Metric

$$C_{vol}(t) = \frac{|\mathcal{V}_{scanned}(t)|}{|\mathcal{V}|} =
  \frac{\text{number of scanned voxels}}{76800}$$

### 3D Load Imbalance (Volumetric Jain Index)

Define the unscanned voxel count for active drone $k$ as $N_k^{rem}(t)$. The volumetric Jain
fairness index is:

$$\mathcal{J}_{vol}(t) = \frac{\left(\sum_{k=1}^{K'} N_k^{rem}\right)^2}{K' \cdot
  \sum_{k=1}^{K'} \left(N_k^{rem}\right)^2} \in \left[\frac{1}{K'},\, 1\right]$$

After each vertical zone hand-off, $\mathcal{J}_{vol}$ is logged pre- and post-reallocation to
measure the 3D load balance improvement.

### Vertical Zone Hand-Off Protocol

When reallocation trigger fires for drone $k^*$:

1. Remove $\mathbf{p}_{k^*}^{3D}$ from the 3D generator set; active set $\mathcal{K}' = \mathcal{K} \setminus \{k^*\}$.
2. Collect all unscanned voxels across every altitude layer in $V_{k^*}^w$:
   $\mathcal{V}_{uncov} = V_{k^*}^w \setminus \mathcal{V}_{scanned}$.
3. Recompute 3D battery-weighted Voronoi $\{V_k^w\}_{k \in \mathcal{K}'}$ over the full remaining
   unscanned volume $\mathcal{V} \setminus \mathcal{V}_{scanned}$ using current 3D positions and
   SoC values.
4. Rebuild each active drone's layer schedule: re-sort received voxels by altitude layer, then by
   descending prior probability within each layer; merge into the existing bottom-up sweep plan.
5. Log $\mathcal{J}_{vol}$ before and after reallocation.

---

## Key 3D Additions

- **Altitude-layer zone partitioning**: each 3D Voronoi zone is decomposed into $N_z = 12$
  horizontal layers; drones sweep layers bottom-up, exploiting the gravity-biased prior.
- **3D Voronoi territory**: generator positions are 3D points; voxel-to-drone assignment uses the
  full 3D Euclidean (or battery-weighted) distance, so vertical adjacency naturally shapes zone
  boundaries into 3D polyhedra rather than 2D polygons.
- **Vertical zone hand-off**: reallocation distributes unscanned voxels from all altitude layers
  of the vacated zone, not just a flat 2D remainder; active drones must integrate received layers
  into an ongoing 3D sweep plan without interrupting current layer progress.
- **Volumetric coverage metric**: $C_{vol}(t)$ counts scanned voxels across the full 3D volume
  rather than 2D cells; high-altitude layers contribute equally to coverage but are lower priority
  under the bottom-up schedule.
- **Vertical energy penalty**: climbing costs $\gamma > 1$ times more battery per metre than
  horizontal cruise, so the weighted Voronoi implicitly discourages assigning tall zones to
  low-SoC drones.

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Search volume | 400 × 400 × 60 m |
| Horizontal resolution $\delta_{xy}$ | 5 m (80 × 80 cells per layer) |
| Vertical layer thickness $\Delta z$ | 5 m |
| Number of altitude layers $N_z$ | 12 (z = 2.5 to 57.5 m) |
| Total voxel count | 76,800 |
| Number of drones $K$ | 4 |
| Cruise speed $v$ | 6 m/s |
| Climb/descent rate $v_z$ | 2 m/s |
| Horizontal sensor radius $r_s$ | 8 m |
| Vertical sensor half-height $r_z$ | 2.5 m |
| Full-charge range $D_{max}$ | 3000 m |
| Vertical energy penalty $\gamma$ | 2.0 |
| Reallocation SoC threshold | 30% |
| Number of survivors $N_s$ | 5 |
| Prior map components $M$ | 4 Gaussian blobs (3D) |
| Prior horizontal spread $\sigma_{xy,m}$ | 30 – 70 m |
| Prior vertical spread $\sigma_{z,m}$ | 5 – 15 m |
| Simulation timestep $\Delta t$ | 0.5 s |
| Safety cutoff | 3600 s |
| Strategies compared | Static 3D Voronoi, Equal-weight 3D, Battery-weighted 3D |

---

## Expected Output

- **3D volume slice views**: isometric view of the search volume showing initial 3D Voronoi zone
  boundaries as wireframe polyhedra, one colour per drone; survivor positions as 3D scatter points;
  drone starting positions as triangles at ground level.
- **Altitude-layer sweep snapshots**: three sets of 12 horizontal layer panels (one set per
  strategy) showing scanned voxels (grey) and remaining voxels (coloured by zone) at
  $t = 0$, first reallocation, and mission end.
- **3D drone trajectories**: per-drone path rendered in 3D axes, showing horizontal sweeps within
  each layer connected by vertical climbs; survivor detection events marked with spheres; vertical
  zone hand-off moments marked with horizontal dashed planes at the reallocation altitude.
- **Volumetric coverage vs time**: $C_{vol}(t)$ for all three strategies on shared axes; vertical
  dashed lines for reallocation events; annotation of each strategy's time to 90% coverage.
- **Layer completion timeline**: stacked Gantt chart showing which altitude layer each drone is
  sweeping at each time step, with hand-off events highlighted; one row per drone.
- **Survivor detection timeline**: horizontal bar chart of detection times per survivor per
  strategy, grouped by survivor index.
- **Volumetric Jain fairness vs time**: $\mathcal{J}_{vol}(t)$ for all three strategies; drops at
  vertical hand-off events with recovery to near-unity for the weighted strategy.
- **Summary statistics table**: per-strategy metrics: time to find all survivors, mean detection
  time, time to 90% volumetric coverage, number of reallocation events, mean $\mathcal{J}_{vol}$,
  total altitude change (m) per drone.
- **Animation (GIF)**: rotating 3D view showing drones ascending through layers, scanned voxels
  filling in layer-by-layer, 3D Voronoi boundaries updating at each hand-off, survivor icons
  turning green on detection.

---

## Extensions

1. **Non-uniform layer priority**: assign higher sweep priority to layers with elevated prior
   probability mass rather than always sweeping bottom-up; derive the optimal layer ordering as
   a function of the 3D prior marginals $P(z) = \sum_{i,j} P(\mathbf{x}_{ijk}) \cdot \delta_{xy}^2$.
2. **Anisotropic 3D weighted Voronoi**: replace scalar SoC weight with a diagonal weight matrix
   $\mathbf{W}_k = \mathrm{diag}(w_k, w_k, w_k / \gamma)$ that discounts vertical distance by the
   energy penalty, so zones are naturally flatter for low-SoC drones.
3. **Terrain-following altitude floor**: introduce a digital elevation model (DEM) so the minimum
   flight altitude is $z_{min}(x,y) = h_{terrain}(x,y) + h_{clearance}$; layer schedule must
   adapt to variable terrain height within each 3D zone.
4. **Communication dropout in 3D**: drones lose peer-state telemetry during GPS/radio blackout
   above a threshold altitude; each drone must decide on local 3D reallocation using last-known
   3D positions and SoC values.
5. **RL 3D reallocation policy**: train a PPO agent that observes all drones' 3D positions, SoC
   values, and the current 3D coverage voxel map, outputting 3D zone boundaries directly; compare
   against 3D battery-weighted Voronoi on randomised volumetric survivor distributions.
6. **Mixed urgency-and-battery 3D weighting**: incorporate per-voxel survivor urgency (estimated
   survival time decreasing with rubble depth) into the 3D zone weight:
   $w_k = \mathrm{SoC}_k \cdot \bar{U}_k^{3D}$ where $\bar{U}_k^{3D}$ is the mean urgency of
   voxels in $V_k^w$.

---

## Related Scenarios

- Original 2D version: [S049 Dynamic Zone Assignment](../S049_dynamic_zone.md)
- 3D search volume references: [S043 Confined Space](../S043_confined_space.md) (indoor 3D navigation), [S050 Swarm SLAM](../S050_slam.md) (simultaneous 3D mapping)
- Algorithmic cross-reference: [S019 Dynamic Reassignment](../../01_pursuit_evasion/S019_dynamic_reassignment.md) (real-time zone reallocation), [S040 Fleet Load Balancing](../../02_logistics_delivery/S040_fleet_load_balancing.md) (Jain fairness, load redistribution)
- Follow-ups: [S054 Minefield Detection](../S054_minefield.md) (3D risk-weighted zone assignment), [S056 Radiation Mapping](../S056_radiation.md) (volumetric hazard mapping)
