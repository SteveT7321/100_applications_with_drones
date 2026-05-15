# S030 3D Upgrade — Multi-Depot Delivery

**Domain**: Logistics & Delivery | **Difficulty**: ⭐⭐⭐⭐ | **Status**: `[ ]` Not Started
**Based on**: [S030 original](../S030_multi_depot_delivery.md)

---

## What Changes in 3D

The original S030 assigns all depots and customers at a fixed `z = 0` m — the entire cost matrix is built from 2D Euclidean distances, altitude change contributes zero energy cost, and the Clarke-Wright savings formula uses only horizontal separation. In a realistic urban deployment, depots occupy distinct altitude tiers (ground warehouses, elevated helipads, rooftop stations), and drones from different tiers must climb or descend on every inter-depot or depot-to-customer leg. This variant introduces:

- **Altitude tiers** for each depot: ground (z = 0 m), mid-level (z = 15 m), rooftop (z = 30 m)
- **3D arc cost** that adds gravitational potential energy for altitude change, not just horizontal distance
- **Altitude-zone routing**: drones assigned to high-tier depots cruise at their depot altitude; altitude transitions occur at the start and end of each leg
- **Vertical separation between fleets**: drones originating from different altitude tiers fly at distinct cruise layers to prevent mid-air conflicts
- **3D open-route return**: the best-return depot selection is computed on 3D distance, penalising returns to a tier far from the drone's current altitude

---

## Problem Definition

**Setup**: A drone delivery network of $D = 4$ depots operates in a $1200 \times 1200 \times 35$ m
urban volume. Depots are positioned at the corners of the horizontal grid but are stratified into three
altitude tiers. A total of $N = 20$ customers are located at ground level ($z \approx 0$–2 m).
Each drone carries out an open-route sortie: it departs its home depot, serves a sequence of customers,
and returns to the nearest depot (in 3D) with available docking capacity.

**Roles**:
- **Depots** $\mathcal{D} = \{d_1, d_2, d_3, d_4\}$: at positions
  $(x_m, y_m, z_m^{depot})$ where $z_m^{depot} \in \{0, 15, 30\}$ m (tier assignment fixed at
  scenario start); each depot retains its fleet size $F_m$ and inventory $I_m$ from S030.
- **Drones** $\mathcal{K}$ (10 total): each cruises at the altitude of its originating depot
  $z_k^{cruise} = z_{\delta(k)}^{depot}$; vertical transitions to/from customers are modelled
  as instantaneous altitude changes at waypoints (energy-penalised, not time-penalised).
- **Customers** $\mathcal{C} = \{c_1, \ldots, c_{20}\}$: all at $z = 1$ m (landing/delivery
  altitude); demands $q_c \in [0.3, 1.8]$ kg as in S030.

**Objective**: Minimise the total 3D energy-weighted flight cost across all sorties, subject to
payload, range, and vertical separation constraints, and compare the three partition strategies
(Nearest-Depot, K-Means, AO) now evaluated on 3D arc costs.

**Altitude tier assignment** (fixed):

| Depot | $(x, y)$ m | Altitude $z^{depot}$ m | Tier |
|-------|-----------|------------------------|------|
| D1 SW | (150, 150) | 0 | Ground |
| D2 NW | (150, 1050) | 30 | Rooftop |
| D3 SE | (1050, 150) | 15 | Mid-level |
| D4 NE | (1050, 1050) | 0 | Ground |

---

## Mathematical Model

### 3D Arc Cost

For a leg from node $i$ at position $\mathbf{p}_i = (x_i, y_i, z_i)$ to node $j$ at
$\mathbf{p}_j = (x_j, y_j, z_j)$, the 3D Euclidean distance is:

$$d_{ij}^{3D} = \|\mathbf{p}_i - \mathbf{p}_j\|_2
  = \sqrt{(x_j - x_i)^2 + (y_j - y_i)^2 + (z_j - z_i)^2}$$

The altitude-change energy penalty (mass $m_{drone}$ kg, gravitational acceleration $g$):

$$E_{alt}(i \to j) = m_{drone} \cdot g \cdot \max(z_j - z_i,\; 0)$$

The combined arc cost used in the VRP formulation is:

$$c_{ij} = d_{ij}^{3D} + \alpha_E \cdot E_{alt}(i \to j)$$

where $\alpha_E$ (m/J) converts energy to an equivalent distance penalty. Descents are free
(recovered by gliding); only climbs are penalised.

### Altitude-Zone Routing Model

Each drone $k$ originating at depot $d_m$ is assigned a cruise altitude
$z_k^{cruise} = z_m^{depot}$. The drone's 3D position during a horizontal cruise leg from
customer $i$ (after climb/descent to cruise altitude) to customer $j$ is:

$$\mathbf{p}_k(t) = \mathbf{p}_i^{cruise} + v \cdot (t - t_i^{(k)}) \cdot \hat{\mathbf{r}}_{ij}^{horiz}$$

where:

$$\mathbf{p}_i^{cruise} = (x_i,\; y_i,\; z_k^{cruise}), \qquad
  \hat{\mathbf{r}}_{ij}^{horiz}
  = \frac{(x_j - x_i,\; y_j - y_i,\; 0)}{\|(x_j - x_i,\; y_j - y_i)\|_2}$$

Each customer stop consists of a vertical descent from $z_k^{cruise}$ to $z = 1$ m for delivery,
followed by a vertical re-climb to $z_k^{cruise}$. The energy cost of one stop-and-return climb:

$$E_{stop} = 2 \cdot m_{drone} \cdot g \cdot (z_k^{cruise} - 1)$$

### 3D Multi-Depot VRP Formulation

Decision variables $x_{ijk}$ and $y_{km}$ are unchanged from S030. The objective is extended to
the 3D arc cost and the per-stop altitude energy:

$$\min \; \sum_{k \in \mathcal{K}} \left[
  \sum_{i,j \in \mathcal{N}} c_{ij} \cdot x_{ijk}
  \;+\; \alpha_E \cdot E_{stop} \cdot |\text{route}_k|
\right]
  \;+\; \lambda \cdot \sum_{m \in \mathcal{D}} (n_m - \bar{n})^2$$

**3D range constraint** (drone $k$ must complete its full sortie including vertical legs):

$$\sum_{i,j} d_{ij}^{3D} \cdot x_{ijk}
  + 2 \cdot (z_k^{cruise} - 1) \cdot |\text{route}_k|
  + \min_{m \in \mathcal{D}} d_{last_k, m}^{3D} \cdot y_{km}
  \leq R_{max} \quad \forall k$$

**Capacity and flow constraints**: identical to S030.

### Clarke-Wright Savings in 3D

For each depot $d_m$ at position $\mathbf{p}_{d_m} = (x_m, y_m, z_m^{depot})$ and two customers
$i$, $j$ (both at $z = 1$ m), the 3D savings value is:

$$S_{ij}^{(m)} = c_{d_m, i} + c_{d_m, j} - c_{ij}$$

where each arc cost $c$ now uses $d^{3D}$ plus the altitude penalty. The savings-based merge is
executed exactly as in S030 (decreasing order, subject to capacity and range).

### 3D Open-Route Return Selection

After drone $k$ completes its last delivery at customer position $(x_{last}, y_{last}, 1)$,
it climbs back to cruise altitude and then flies to the nearest depot in 3D:

$$m^*_k = \arg\min_{m \in \mathcal{D}}
  \left[ d_{last_k, m}^{3D} + \alpha_E \cdot m_{drone} \cdot g \cdot \max(z_m^{depot} - 1, 0) \right]
  \quad \text{s.t. } A_m > 0$$

Return distance saving over forced home-base return (3D):

$$\Delta R_k^{3D} = c_{last_k,\, \delta(k)} - c_{last_k,\, m^*_k}$$

### Vertical Separation Constraint

To prevent mid-air conflicts between fleets originating at different altitude tiers, a minimum
vertical separation $\Delta z_{sep}$ is enforced:

$$|z_k^{cruise} - z_{k'}^{cruise}| \geq \Delta z_{sep}
  \quad \forall k, k' \in \mathcal{K}, \; \delta(k) \neq \delta(k')$$

With the tier assignment above ($z \in \{0, 15, 30\}$ m) and $\Delta z_{sep} = 5$ m, this is
automatically satisfied. If two depots share the same tier, their drones are offset by
$\pm 2.5$ m within that tier.

### Alternating Optimisation with 3D Costs

The AO loop from S030 is unchanged in structure; the only modification is that all distance and
cost evaluations use $c_{ij}$ in place of $d_{ij}$. The marginal reassignment criterion becomes:

$$\Delta_{m \to m'}^{3D} = \text{cost}^{3D}(\Pi_{m'} \cup \{c\}) + \text{cost}^{3D}(\Pi_m \setminus \{c\})
  - \text{cost}^{3D}(\Pi_m) - \text{cost}^{3D}(\Pi_{m'})$$

Accept the reassignment when $\Delta_{m \to m'}^{3D} < 0$ and feasibility is preserved.

---

## Key 3D Additions

- **Altitude tiers**: depots at $z \in \{0, 15, 30\}$ m; customers all at $z = 1$ m
- **3D arc cost**: $c_{ij} = d_{ij}^{3D} + \alpha_E \cdot m \cdot g \cdot \max(\Delta z, 0)$, climbs penalised, descents free
- **Per-stop energy**: vertical descent + re-climb at each customer waypoint contributes to range budget
- **Altitude-zone cruising**: each drone cruises at its depot's altitude tier, descends only at delivery waypoints
- **3D open-route return**: best-return depot selected on 3D cost including return-climb penalty
- **Vertical separation**: tiers at 15 m spacing guarantee $\geq 5$ m fleet separation without collision logic
- **3D trajectory visualisation**: full `(x, y, z)` paths with altitude time-series per drone

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Operational volume | 1200 × 1200 × 35 m |
| Depot altitudes | D1: 0 m, D2: 30 m, D3: 15 m, D4: 0 m |
| Customer altitude | 1 m (ground delivery) |
| Drone cruise altitude | equals originating depot's $z^{depot}$ |
| Drone mass $m_{drone}$ | 1.5 kg |
| Gravitational acceleration $g$ | 9.81 m/s² |
| Energy-to-distance factor $\alpha_E$ | 0.05 m/J |
| Vertical separation $\Delta z_{sep}$ | 5 m minimum |
| Fleet distribution | [3, 2, 3, 2] drones per depot = 10 total |
| Depot inventory | [25, 18, 25, 20] kg |
| Customers | 20, random in [100, 1100] m, $z = 1$ m, seed 7 |
| Drone payload $Q$ | 3.0 kg |
| Max range $R_{max}$ | 3500 m per sortie (3D path length) |
| Cruising speed $v$ | 14 m/s |
| Balance weight $\lambda$ | 50 m² / delivery² |
| AO convergence $\varepsilon_{AO}$ | 1.0 m |

---

## Expected Output

- 3D trajectory plot: all 10 drone paths with distinct altitude layers; depot markers at their
  true $(x, y, z)$ positions; customers shown at $z = 1$ m; open-return arcs as dashed lines
- Altitude time-series: $z(t)$ per drone showing cruise level, descent dips to $z = 1$ m at each
  customer stop, and final climb/descent to return depot altitude
- Strategy comparison bar chart: total 3D energy-weighted cost for Nearest-Depot vs K-Means vs AO
- 2D vs 3D cost comparison: side-by-side bars showing how altitude tiers change partition decisions
  relative to the flat S030 solution
- Load balance chart: deliveries per depot for each strategy (unchanged structure from S030)
- AO convergence curve: 3D objective value vs iteration number
- Per-drone summary table: origin depot, return depot, 3D route length, altitude tier, energy used
- Scalar: total open-route saving $\Delta R_{total}^{3D}$ vs forced home-base return (3D costs)
- Animated GIF: all 10 drones flying at their respective altitude tiers, coloured by originating depot

---

## Extensions

1. Heterogeneous altitude tiers — relax fixed tier assignment and let the AO loop also optimise
   which altitude each depot broadcasts as its cruise layer, trading off climb costs against
   airspace deconfliction
2. Dynamic tier conflict — add a fifth drone fleet operating a crossing corridor at $z = 15$ m;
   trigger a temporary tier-shift for mid-level drones and measure re-routing latency
3. Energy-optimal descent profile — replace instantaneous altitude change at waypoints with a
   continuous glide slope; add the horizontal distance consumed by the glide to the range budget
4. Wind-layer model — assign a horizontal wind vector $\mathbf{w}(z)$ that varies with altitude;
   ground-level drones face higher drag while rooftop-origin drones benefit from tailwind
5. Multi-sortie inventory replenishment — model a rooftop depot being resupplied by a heavier
   lift UAV from the ground depot; schedule replenishment legs jointly with delivery routes

---

## Related Scenarios

- Original 2D version: [S030](../S030_multi_depot_delivery.md)
- Other 3D logistics upgrades: [S029 3D](S029_3d_urban_logistics.md), [S027 3D](S027_3d_aerial_refueling.md)
- Truly 3D references: [S022 3D](S022_3d_obstacle_avoidance.md), [S028 3D](S028_3d_cargo_escort.md)
- Algorithmic cross-reference: [S019](../../01_pursuit_evasion/S019_dynamic_reassignment.md) (Hungarian assignment), [S018](../../01_pursuit_evasion/S018_multi_target_intercept.md) (TSP tour building)
