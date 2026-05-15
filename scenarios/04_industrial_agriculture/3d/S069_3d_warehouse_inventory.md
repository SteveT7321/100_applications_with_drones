# S069 3D Upgrade — Automated Warehouse Inventory

**Domain**: Industrial & Agriculture | **Difficulty**: ⭐⭐⭐ | **Status**: `[ ]` Not Started
**Based on**: [S069 original](../S069_warehouse_inventory.md)

---

## What Changes in 3D

The original S069 treats the warehouse as a 3D grid but assigns only a single drone and uses a flat
nearest-neighbour TSP ordering that ignores the vertical structure of the shelf layout. Scan
geometry is reduced to a single stand-off distance scalar; there is no model of barcode-reader
viewing angle or detection cone in 3D. Altitude transitions between tiers (z = 1, 3, 5 m) are
implicit in the A* path cost without any explicit altitude-layer planning strategy.

This upgrade introduces:
- **Full shelf-rack 3D structure** — shelves are modelled as oriented planar faces with a surface
  normal vector; the drone must approach along a valid reading cone, not merely reach the centroid.
- **Altitude-layer scanning strategy** — each floor is serviced as an independent sweep layer;
  inter-layer transitions follow a designated vertical shaft to avoid aisle congestion.
- **3D barcode detection angle constraint** — a scan is only valid when the drone's relative bearing
  to the shelf face falls within a half-angle $\phi_{max}$ of the face normal.
- **Multi-drone floor assignment** — two or more drones are statically assigned to disjoint floor
  bands (bottom / middle / top tier) and plan independent A* tours; a synchronised home-base
  rendezvous terminates the mission.

---

## Problem Definition

**Setup**: A warehouse of 30 × 20 × 6 m holds three shelf tiers at z = 1, 3, and 5 m, each tier
containing 20 shelf-face scan targets (60 faces total). Each shelf face is described by its centroid
$\mathbf{c}_i \in \mathbb{R}^3$ and outward unit normal $\hat{\mathbf{n}}_i$. A fleet of three
drones $\mathcal{D} = \{D_1, D_2, D_3\}$ is deployed, one drone assigned to each altitude tier. All
drones share the same pre-built 3D occupancy grid at $\Delta = 0.5$ m resolution. Vertical transit
shafts at x = 1 m and x = 29 m are reserved as free corridors for altitude changes.

**Roles**:
- **$D_k$ (Tier drone)**: executes A* routing within its assigned altitude band
  $z \in [z_k - 1,\; z_k + 1]$ m, dwells at each shelf face for $T_{dwell}$ seconds, and
  validates each scan against the 3D detection-angle constraint.
- **Ground control**: assigns tiers, receives scan confirmations, logs the global inventory map.

**Objective**: All 60 shelf faces are scanned with detection angle $\theta_{face} \leq \phi_{max}$
and residual velocity $v_{hover} \leq v_{thresh}$. Minimise the wall-clock completion time
$T_{wall}$ (time from mission start until all three drones return to their home positions), subject
to zero inter-drone collision and zero occupied-voxel violations.

---

## Mathematical Model

### Shelf-Face Geometry

Each shelf face $i$ is characterised by:

$$\mathbf{c}_i \in \mathbb{R}^3, \quad \hat{\mathbf{n}}_i \in \mathbb{R}^3,\; \|\hat{\mathbf{n}}_i\| = 1$$

The drone approaches to a stand-off position offset from the face centroid along the face normal:

$$\mathbf{w}_i = \mathbf{c}_i + d_{scan} \cdot \hat{\mathbf{n}}_i$$

where $d_{scan} = 0.5$ m is the stand-off distance. This is the A* goal waypoint for shelf $i$.

### 3D Barcode Detection Angle

At the moment of dwell the drone is at position $\mathbf{p}$ with residual velocity $\mathbf{v}$.
The line-of-sight from drone to shelf face centroid is:

$$\hat{\mathbf{l}}_i = \frac{\mathbf{c}_i - \mathbf{p}}{\|\mathbf{c}_i - \mathbf{p}\|}$$

The 3D detection angle between the reading axis and the face normal is:

$$\theta_{face,i} = \arccos\!\left( \hat{\mathbf{l}}_i \cdot (-\hat{\mathbf{n}}_i) \right)$$

A scan attempt at shelf $i$ is geometrically valid only when:

$$\theta_{face,i} \leq \phi_{max} = 30°$$

For $\theta_{face,i} > \phi_{max}$ the barcode reader returns no data regardless of velocity;
the drone must reposition along an arc on the cone surface $\theta = \phi_{max}$ until the
constraint is satisfied.

### Scan Success Probability (Extended)

The combined scan success probability accounts for both velocity noise and detection angle:

$$P_{scan}(\mathbf{p}, \mathbf{v}) =
  \underbrace{\left[1 - \exp\!\left(-k_{scan}\,\frac{v_{thresh} - \|\mathbf{v}\|}{v_{thresh}}\right)\right]}_{\text{velocity factor}}
  \cdot
  \underbrace{\cos^2\!\left(\frac{\theta_{face}}{\phi_{max}} \cdot \frac{\pi}{2}\right)}_{\text{angle factor}},
  \quad \|\mathbf{v}\| \leq v_{thresh}$$

The angle factor equals 1 for perfect face-normal approach ($\theta_{face} = 0$) and drops to 0
at the cone boundary ($\theta_{face} = \phi_{max}$).

### Altitude-Layer A* with Vertical Shaft Transitions

Each drone $D_k$ is constrained to its assigned altitude band. Inter-tier vertical moves are
permitted only inside the reserved shaft voxels at $x \in \{1, 29\}$ m. The modified cost
function penalises out-of-band altitude steps:

$$g_{step}(\mathbf{u}, \mathbf{v}) =
\begin{cases}
  \Delta z \cdot w_{vert} & \text{if } v_z \notin [z_k - 1,\; z_k + 1] \text{ and not in shaft} \\
  \|\mathbf{u} - \mathbf{v}\|_2 & \text{otherwise}
\end{cases}$$

with $w_{vert} = 10$ (large penalty discourages band violation while preserving path feasibility).

### Multi-Drone Floor Assignment

The 60 shelf-face waypoints are partitioned into three disjoint sets by tier height:

$$\mathcal{W}_k = \{ i \mid z_k - 1 < c_{i,z} \leq z_k + 1 \}, \quad k \in \{1,2,3\}$$

with $z_1 = 1$ m, $z_2 = 3$ m, $z_3 = 5$ m. Each drone plans an independent nearest-neighbour
TSP tour over its assigned set $\mathcal{W}_k$:

$$w_{k,\ell+1} = \underset{w \in \mathcal{W}_k \setminus \{w_{k,1},\ldots,w_{k,\ell}\}}{\arg\min}\; d_{A^*}(w_{k,\ell},\, w)$$

### Wall-Clock Mission Time

Drones operate in parallel. The wall-clock completion time is determined by the slowest tier:

$$T_{wall} = \max_{k \in \{1,2,3\}} T_{mission,k}$$

where $T_{mission,k} = \sum_{\ell} \left( t_{transit,k,\ell} + T_{dwell} \right)$.

The load-balancing efficiency is:

$$\eta_{balance} = \frac{\frac{1}{3}\sum_k T_{mission,k}}{\max_k T_{mission,k}}$$

Perfect balance yields $\eta_{balance} = 1$; imbalance in tier waypoint density degrades this metric.

### 3D Collision Avoidance Between Drones

Drones remain in disjoint altitude bands during scanning. During vertical shaft transitions,
a time-based slot allocation prevents simultaneous shaft occupancy:

$$\Delta t_{slot} = \frac{h_{shaft}}{v_{vert}} + t_{buffer}$$

where $h_{shaft} = 6$ m, $v_{vert} = 1.0$ m/s, and $t_{buffer} = 2$ s. Drone $D_k$ may enter
the shaft only after $D_{k-1}$ has cleared its time slot.

---

## Key 3D Additions

- **Shelf-face normal vectors**: each of 60 targets has an outward unit normal $\hat{\mathbf{n}}_i$;
  the A* goal is the approach waypoint $\mathbf{w}_i = \mathbf{c}_i + d_{scan}\hat{\mathbf{n}}_i$.
- **3D detection-angle constraint**: scan validity gated by
  $\theta_{face} \leq \phi_{max} = 30°$; repositioning arc computed on the reading cone.
- **Altitude-layer strategy**: three drones each confined to one altitude band; vertical moves
  allowed only inside reserved shaft corridors at $x \in \{1, 29\}$ m.
- **Modified A* cost**: out-of-band altitude steps penalised by $w_{vert} = 10$; shaft voxels
  use standard Euclidean step cost.
- **Parallel mission timing**: wall-clock time equals the slowest tier; load-balancing efficiency
  $\eta_{balance}$ quantifies tier workload equality.
- **Shaft slot allocation**: time-slotted vertical transit prevents inter-drone collisions inside
  shared shafts.

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Warehouse dimensions | 30 × 20 × 6 m |
| Grid resolution | 0.5 m/voxel |
| Grid size | 60 × 40 × 12 voxels |
| Tier heights $z_k$ | 1, 3, 5 m |
| Altitude band half-width | ±1 m per tier |
| Number of shelf columns | 20 |
| Total scan targets | 60 (20 per tier) |
| Shelf face normal orientation | $\pm\hat{\mathbf{y}}$ (aisle-facing) |
| Stand-off distance $d_{scan}$ | 0.5 m |
| Detection half-angle $\phi_{max}$ | 30° |
| Dwell time $T_{dwell}$ | 1.0 s |
| Cruise speed $v_{cruise}$ | 1.0 m/s |
| Vertical transit speed $v_{vert}$ | 1.0 m/s |
| Velocity threshold $v_{thresh}$ | 0.2 m/s |
| Scan sensitivity $k_{scan}$ | 3.0 |
| Indoor position noise | 0.1 m std dev |
| Out-of-band altitude penalty $w_{vert}$ | 10 |
| Shaft x-positions | 1 m, 29 m |
| Shaft time-slot buffer $t_{buffer}$ | 2.0 s |
| Number of drones | 3 |
| Scan success requirement | 95 % |

---

## Expected Output

- **3D warehouse layout plot**: full shelf-rack geometry with face normals shown as short arrows;
  each tier coloured by assigned drone (red / blue / green); A* routes for all three drones
  overlaid as coloured polylines; stand-off waypoints marked as translucent spheres.
- **Altitude time series**: z(t) for each drone on a shared time axis; horizontal dashed lines at
  $z = 1, 3, 5$ m; shaft transit segments highlighted; demonstrates tier confinement and shaft
  transitions.
- **3D detection angle map**: heatmap of $\theta_{face,i}$ for all 60 shelf faces; faces violating
  $\phi_{max}$ shown in red with repositioning arc illustrated on a zoomed inset.
- **Per-tier scan success rate bar chart**: grouped bars by tier (z = 1, 3, 5 m) for velocity
  factor, angle factor, and combined $P_{scan}$; dashed 95 % threshold line.
- **Load-balancing comparison**: bar chart of $T_{mission,k}$ for each drone; horizontal line at
  $T_{wall}$; annotation showing $\eta_{balance}$.
- **Animation (GIF)**: 3D animated view showing all three drones moving simultaneously through
  their respective tiers; shelf faces change colour (white → green success, red fail) on dwell;
  a live legend shows elapsed time, scans completed per drone, and current scan rate.
- **Console metrics table**:

  | Metric | Value |
  |--------|-------|
  | Wall-clock mission time (s) | ... |
  | Total path length per drone (m) | ... |
  | Scan success rate — tier 1 (%) | ... |
  | Scan success rate — tier 2 (%) | ... |
  | Scan success rate — tier 3 (%) | ... |
  | Angle-rejected scans | ... |
  | Load-balancing efficiency $\eta_{balance}$ | ... |

---

## Extensions

1. **Adaptive repositioning**: when $\theta_{face} > \phi_{max}$, compute the nearest point on the
   reading cone and add it as an intermediate waypoint; measure the extra distance cost versus the
   re-scan time saved.
2. **Dynamic tier rebalancing**: if one drone finishes early, reassign remaining waypoints from the
   slowest tier to the idle drone; model the handoff overhead (inter-tier transit time) against the
   wall-clock savings.
3. **Variable stand-off distance**: model scanner focal depth — $P_{scan}$ peaks at
   $d_{opt} = 0.5$ m but falls for $d < 0.2$ m (too close) or $d > 1.0$ m (too far); let the
   drone optimise $d$ per face given the approach trajectory.
4. **Battery-constrained sub-tours**: each drone carries a battery supporting 80 s of flight;
   add a per-tier charging station; count charging interruptions and their effect on $T_{wall}$.
5. **Probabilistic shelf face orientation**: introduce ±5° angular noise on shelf installation;
   the drone must estimate the actual face normal from a short hover scan before committing to the
   full dwell read; model the cost of the normal-estimation sweep.

---

## Related Scenarios

- Original 2D/flat version: [S069](../S069_warehouse_inventory.md)
- 3D volumetric mapping reference: [S065 Building 3D Modeling Sampling](../S065_3d_scan_path.md)
- GNSS-denied interior planning: [S043 Confined Space Exploration](../../03_environmental_sar/S043_confined_space.md)
- Outdoor high-density inventory: [S075 Port Container Yard Inventory](../S075_container_yard.md)
- Multi-drone coordination pattern: [S068 Large-Scale Farmland Cooperative Spraying](../S068_large_field_spray.md)
