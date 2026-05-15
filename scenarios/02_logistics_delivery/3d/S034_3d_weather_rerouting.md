# S034 3D Upgrade — Weather Rerouting

**Domain**: Logistics & Delivery | **Difficulty**: ⭐⭐⭐⭐ | **Status**: `[ ]` Not Started
**Based on**: [S034 original](../S034_weather_rerouting.md)

---

## What Changes in 3D

The original S034 fixes the drone at a single nominal cruise altitude of z = 20 m. While the grid is technically stored as a 3D voxel structure (50 × 50 × 15), the D* Lite planner is seeded with a cost map whose hazard cells are spheres and the drone is only ever steered horizontally — vertical detours never materialise because the start and goal share the same z-layer and no altitude-band wind shear is modelled. In full 3D, the drone gains a vertical degree of freedom that it actively exploits: it can climb above or dive below weather volumes, traverse altitude-band wind shear layers at different energy costs, and execute altitude-hopping manoeuvres when a horizontal detour would be longer than a vertical bypass. Wind shear introduces altitude-dependent edge costs not present in the flat-cruise original, and the replanning trigger must now also monitor the vertical component of the remaining path.

---

## Problem Definition

**Setup**: A delivery drone departs from $\mathbf{p}_{start} = (0, 0, z_0)$ and must reach $\mathbf{p}_{goal} = (90, 90, z_f)$ through a 100 m × 100 m × 60 m airspace. Altitude is no longer fixed: the drone is free to cruise anywhere in $z \in [5, 55]$ m. The airspace contains:

1. **Weather volumes** — ellipsoidal convective cells that appear and dissipate stochastically, extending vertically up to 30 m.
2. **Altitude-band wind shear** — three horizontal layers each carrying a different mean horizontal wind speed, making flight at some altitudes faster but more energy-costly.
3. **Altitude floor / ceiling** — a minimum safe altitude of 5 m (obstacle clearance) and a maximum of 55 m (controlled airspace boundary).

**Roles**:
- **Drone**: navigates the 3D voxel graph using D* Lite with a composite edge-cost that includes Euclidean distance, wind-shear energy penalty, and weather inflation. Chooses altitude freely.
- **Weather model**: generates 3D ellipsoidal hazard volumes. Cell $k$ has semi-axes $(a_k, b_k, c_k)$ (horizontal × horizontal × vertical), severity $w_k \in [0,1]$, activation time $t_{on,k}$, and lifetime $\tau_k \sim \text{Uniform}(30, 120)$ s.

**Objective**: minimise total energy-weighted flight cost subject to zero traversal time inside active hazard volumes, by choosing the optimal 3D trajectory that trades horizontal detour distance against vertical detour distance and altitude-band wind penalties.

---

## Mathematical Model

### 3D Voxel Grid

The airspace is discretised into voxels with resolution $\Delta_{xy} = 2$ m horizontally and $\Delta_z = 2$ m vertically, giving a grid of size $N_x \times N_y \times N_z = 50 \times 50 \times 30$ voxels. Each node $v_{i,j,k}$ represents world position:

$$\mathbf{p}_{i,j,k} = (i \,\Delta_{xy},\; j\, \Delta_{xy},\; z_{min} + k\, \Delta_z)$$

where $z_{min} = 5$ m. Edges connect 26-adjacent neighbours (face, edge, and corner adjacency).

### Altitude-Band Wind Shear Model

The airspace is divided into three horizontal layers with altitude-dependent mean wind speed $\bar{W}(z)$:

$$\bar{W}(z) = \begin{cases} W_{low} & z \in [5, 20) \text{ m} \\ W_{mid} & z \in [20, 40) \text{ m} \\ W_{high} & z \in [40, 55] \text{ m} \end{cases}$$

with typical values $W_{low} = 2$ m/s, $W_{mid} = 6$ m/s, $W_{high} = 3$ m/s. The wind direction is fixed as $\hat{\mathbf{w}} = (\cos\psi_w, \sin\psi_w, 0)$.

The effective drone ground speed along edge $(u, v)$ when flying through altitude layer $\ell$ is:

$$v_{eff}(u,v) = v_{max} - \hat{\mathbf{e}}_{uv} \cdot \bar{W}(z_u)\,\hat{\mathbf{w}}$$

where $\hat{\mathbf{e}}_{uv} = (\mathbf{p}_v - \mathbf{p}_u) / \|\mathbf{p}_v - \mathbf{p}_u\|$ is the edge unit vector. The shear-weighted edge traversal time is:

$$t_{edge}(u,v) = \frac{\|\mathbf{p}_v - \mathbf{p}_u\|}{v_{eff}(u,v)}$$

### 3D Ellipsoidal Hazard Cell Model

Each hazard cell $k$ is an axis-aligned ellipsoid centred at $\mathbf{c}_k = (c_{x,k}, c_{y,k}, c_{z,k})$ with semi-axes $a_k, b_k \sim \text{Uniform}(8, 20)$ m (horizontal) and $c_k \sim \text{Uniform}(10, 30)$ m (vertical):

$$\mathcal{H}_k(t) = \left\{ v \;:\; \left(\frac{p_{x,v} - c_{x,k}}{a_k}\right)^2 + \left(\frac{p_{y,v} - c_{y,k}}{b_k}\right)^2 + \left(\frac{p_{z,v} - c_{z,k}}{c_k}\right)^2 \leq 1 \right\}$$

for $t_{on,k} \leq t < t_{on,k} + \tau_k$, and $\mathcal{H}_k(t) = \emptyset$ otherwise. The ellipsoidal geometry creates a vertical extent that forces the planner to decide whether it is cheaper to fly around horizontally or to transit above or below the cell.

### Composite Edge Cost

The total traversal cost of edge $(u, v)$ at time $t$ combines three terms:

$$c(u,v,t) = c_{dist}(u,v) + c_{shear}(u,v) + c_{hazard}(u,v,t)$$

**Distance cost** (Euclidean baseline):

$$c_{dist}(u,v) = \|\mathbf{p}_v - \mathbf{p}_u\|_2$$

**Wind-shear energy penalty** (headwind increases energy, tailwind decreases it):

$$c_{shear}(u,v) = \beta \cdot \bar{W}(z_u) \cdot \max\!\left(\hat{\mathbf{e}}_{uv} \cdot \hat{\mathbf{w}},\; 0\right) \cdot \|\mathbf{p}_v - \mathbf{p}_u\|_2$$

where $\beta = 0.5$ is the wind penalty weight. Crosswind components do not appear in the energy cost (drag increase is secondary).

**Weather hazard inflation**:

$$c_{hazard}(u,v,t) = c_{dist}(u,v) \cdot \alpha \sum_{k \in \mathcal{A}(t)} w_k \cdot \mathbf{1}[v \in \mathcal{H}_k(t)]$$

with $\alpha = 10$ and $\mathcal{A}(t)$ the set of active cells at time $t$.

### Altitude-Change Vertical Cost

Altitude transitions carry an additional climb/descent energy cost proportional to the elevation change:

$$c_{vert}(u,v) = \gamma \cdot |z_v - z_u|, \quad \gamma = 1.5$$

This penalises excessive altitude hopping and ensures the planner uses vertical detours only when they are clearly cheaper than horizontal ones. The full edge cost becomes:

$$c(u,v,t) = c_{dist}(u,v) + c_{shear}(u,v) + c_{hazard}(u,v,t) + c_{vert}(u,v)$$

### D* Lite Incremental Replanning (3D)

D* Lite operates identically to S034 but over the full 3D graph. The key and consistency definitions are unchanged:

$$g(s) = \text{current best cost-to-come}$$

$$rhs(s) = \min_{s' \in \text{Pred}(s)} \left[ c(s', s, t) + g(s') \right]$$

$$\mathbf{k}(s) = \left[\min(g(s),\; rhs(s)) + h(\mathbf{p}_s, \mathbf{p}_{start}) + k_m;\;\; \min(g(s),\; rhs(s))\right]$$

The heuristic is the 3D Euclidean distance scaled by a lower bound on edge cost per metre:

$$h(\mathbf{p}_s, \mathbf{p}_{start}) = \|\mathbf{p}_s - \mathbf{p}_{start}\|_2 \cdot c_{min}$$

where $c_{min} = 1 + \gamma \cdot \mathbf{1}[\text{altitude change required}]$ adjusts for the vertical penalty.

### Vertical Bypass Decision Heuristic

Before each D* Lite replanning call, a fast altitude-bypass check is performed. If a newly active cell $k$ intersects the remaining path, the planner computes:

$$\Delta z_{above} = c_{z,k} + c_k - z_{drone}, \quad \Delta z_{below} = z_{drone} - (c_{z,k} - c_k)$$

$$z_{bypass} = \begin{cases} z_{drone} + \Delta z_{above} + z_{margin} & \text{if } \Delta z_{above} \leq \Delta z_{below} \text{ and } z_{drone} + \Delta z_{above} \leq z_{max} \\ z_{drone} - \Delta z_{below} - z_{margin} & \text{otherwise} \end{cases}$$

with $z_{margin} = 3$ m. This heuristic seeds the D* Lite warm start by temporarily removing nodes at the bypass altitude from the closed set, accelerating convergence when a simple altitude change suffices.

### 3D Replanning Trigger

A replanning event fires when:

1. A newly activated cell intersects the 3D remaining path (horizontal **or** vertical component):

$$\exists\; k \in \mathcal{A}(t) : \mathcal{H}_k(t) \cap \Pi_{remaining}(t) \neq \emptyset$$

2. An expiring cell may re-open a shorter path in any spatial direction:

$$\exists\; k \notin \mathcal{A}(t) : \mathcal{H}_k(t^-) \cap \Pi_{remaining}(t^-) \neq \emptyset$$

3. The drone crosses an altitude-layer boundary, causing wind-shear edge costs to change for all edges in the new layer.

### Path Cost Metrics

$$J_{total} = J_{base} + J_{detour}^{xy} + J_{detour}^{z} + J_{shear} + J_{hazard}$$

$$J_{base} = \|\mathbf{p}_{goal} - \mathbf{p}_{start}\|_2$$

$$J_{detour}^{xy} = L_{xy,actual} - L_{xy,straight} \quad \text{(horizontal detour)}$$

$$J_{detour}^{z} = \sum_t |z(t+\Delta t) - z(t)| - |z_{goal} - z_{start}| \quad \text{(excess altitude change)}$$

$$J_{shear} = \int_0^T \beta \cdot \bar{W}(z(t)) \cdot \max(\dot{\mathbf{p}}(t) \cdot \hat{\mathbf{w}}, 0)\, dt$$

$$J_{hazard} = \int_0^T \mathbf{1}\!\left[\mathbf{p}(t) \in \bigcup_k \mathcal{H}_k(t)\right] dt \quad \text{(target: 0)}$$

### Drone Kinematics

Point-mass model with waypoint following in full 3D:

$$\dot{\mathbf{p}} = \mathbf{v}, \quad \mathbf{v}_{cmd} = v_{max} \cdot \frac{\mathbf{p}_{next} - \mathbf{p}}{\|\mathbf{p}_{next} - \mathbf{p}\|}$$

The vertical speed is naturally bounded by $v_{max}$ projected onto the z-axis; no separate climb-rate limiter is needed at this abstraction level.

---

## Key 3D Additions

- **Ellipsoidal weather volumes**: replaces spheres with axis-aligned ellipsoids parameterised by $(a_k, b_k, c_k)$, giving realistic tall convective cells with narrow horizontal footprints.
- **Altitude-band wind shear**: three horizontal layers with distinct mean wind speeds; the planner trades altitude for favourable wind when the shear gain outweighs the climb-energy penalty.
- **Vertical bypass heuristic**: fast pre-check computes the minimum altitude shift to clear a blocking cell above or below, seeding D* Lite for faster convergence.
- **Vertical cost term $c_{vert}$**: penalises unnecessary altitude oscillation, preventing the planner from ping-ponging between layers to exploit marginal wind differences.
- **Layer-crossing replanning trigger**: replanning fires not only on weather events but also when the drone enters a new wind-shear band, ensuring the cost map stays consistent with the drone's current altitude.
- **3D cost breakdown**: $J_{detour}$ is split into horizontal and vertical components to quantify how much of the detour penalty is due to altitude manoeuvring vs. horizontal circumnavigation.

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Airspace | 100 m × 100 m × 60 m |
| Voxel resolution | 2 m × 2 m × 2 m |
| Grid size | 50 × 50 × 30 voxels |
| Altitude range | 5 – 55 m |
| Number of hazard cells | 8 |
| Hazard horizontal semi-axes $a_k, b_k$ | 8 – 20 m (uniform) |
| Hazard vertical semi-axis $c_k$ | 10 – 30 m (uniform) |
| Hazard lifetime $\tau_k$ | 30 – 120 s (uniform) |
| Hazard severity $w_k$ | 0.3 – 1.0 (uniform) |
| Hazard inflation factor $\alpha$ | 10 |
| Wind shear layers | Low: 2 m/s, Mid: 6 m/s, High: 3 m/s |
| Wind shear penalty weight $\beta$ | 0.5 |
| Vertical climb penalty $\gamma$ | 1.5 |
| Vertical bypass margin $z_{margin}$ | 3 m |
| Drone max speed $v_{max}$ | 5 m/s |
| Time step $\Delta t$ | 0.05 s |
| Heuristic | 3D Euclidean × $c_{min}$ |

---

## Expected Output

- 3D trajectory plot (isometric and top-down projections) showing the drone path coloured by altitude, with ellipsoidal weather volumes rendered as transparent meshes fading in and out over time
- Altitude time series $z(t)$ highlighting bypass events: upward spikes when climbing above a cell, downward dips when diving below
- Wind-shear layer diagram: horizontal bands coloured by wind speed with drone altitude trajectory overlaid
- Replanning event timeline: vertical markers for weather events and layer-crossing triggers, with replanning latency annotated
- Cost breakdown bar chart comparing S034 (flat 2D cruise) vs S034-3D: $J_{base}$, $J_{detour}^{xy}$, $J_{detour}^{z}$, $J_{shear}$, $J_{hazard}$
- Vertical bypass vs horizontal detour decision log: table showing for each replanning event whether the planner chose to go around horizontally or vertically, and the cost difference
- Animation (GIF): isometric view, ellipsoidal cells pulsing orange when active, drone path in blue, replanned segments in red, altitude colour-coded on trajectory ribbon

---

## Extensions

1. **Forecast-driven pre-climb**: use an Ornstein-Uhlenbeck model for cell vertical growth to pre-emptively adjust altitude before a rising cell intersects the path
2. **Energy-optimal altitude cruising**: replace the fixed penalty $\gamma$ with a full aerodynamic climb model $P_{climb}(v_z) = P_0 + m g v_z / \eta$ and find the energy-Pareto frontier between altitude and horizontal detour
3. **Anisotropic wind field**: replace the layer-mean wind with a 3D vector field $\mathbf{W}(\mathbf{x})$ sampled from a turbulence spectrum; edge costs become direction-dependent in all three axes
4. **Multi-drone altitude deconfliction**: two drones assigned different nominal altitude bands; replanning must respect vertical separation minima (e.g., $\geq 5$ m) as a hard constraint alongside weather avoidance
5. **Stochastic D* Lite**: treat hazard lifetime $\tau_k$ as a random variable with known distribution; compute expected edge costs to bias the planner toward altitudes where the probability of encountering active cells is lowest

---

## Related Scenarios

- Original 2D version: [S034](../S034_weather_rerouting.md)
- 3D path planning reference: [S022 3D Obstacle Avoidance](S022_3d_obstacle_avoidance.md), [S029 3D Urban Logistics](S029_3d_urban_logistics.md)
- Wind compensation foundation: [S024 Wind Compensation](../S024_wind_compensation.md), [S024 3D Wind Compensation](S024_3d_wind_compensation.md)
- Follow-ups: [S031 Path Deconfliction](../S031_path_deconfliction.md), [S038 Disaster Relief](../S038_disaster_relief.md)
