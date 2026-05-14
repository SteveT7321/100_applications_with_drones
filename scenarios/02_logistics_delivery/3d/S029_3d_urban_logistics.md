# S029 3D Upgrade — Urban Logistics Scheduling

**Domain**: Logistics & Delivery | **Difficulty**: ⭐⭐⭐⭐ | **Status**: `[ ]` Not started
**Based on**: [S029 original](../S029_urban_logistics_scheduling.md)

---

## What Changes in 3D

The original S029 schedules drone routes in a flat 2D plane where every customer and depot
sits at altitude z = 2 m.  In practice, urban delivery zones enforce distinct operating
altitudes:

| Zone type | Required altitude |
|-----------|-------------------|
| Residential | 30 m |
| Commercial  | 60 m |
| Rooftop     | 100 m |

Adding altitude means:
- Each delivery customer is assigned to one of the three zone types (and thus one required altitude).
- Drones must **climb or descend** between consecutive stops at different altitudes.
- The **energy cost** of a leg is the 3D Euclidean distance plus a weighted altitude-change penalty:

$$E_{ij} = \|\mathbf{p}_j - \mathbf{p}_i\|_3 + \gamma \cdot |z_j - z_i|$$

- Clarke-Wright savings are recomputed using this 3D energy metric.
- Route feasibility checks use the 3D energy budget instead of planar distance.
- Visualization uses `mpl_toolkits.mplot3d` and altitude-profile subplots per drone.

---

## Problem Definition

**Setup**: $D = 3$ depots and $C = 12$ customer delivery points on a $1000 \times 1000$ m
ground grid.  Depot altitude is fixed at 30 m.  Each customer is randomly assigned to a
zone type (residential / commercial / rooftop), fixing its operating altitude.  A fleet of
$K = 6$ drones (2 per depot) must serve all customers subject to payload capacity, 3D energy
budget, and time-window constraints.

**Objective**: Minimise total weighted tardiness plus total 3D energy cost, re-solved with
Clarke-Wright savings using the 3D energy metric.

**Comparison**: 2D Clarke-Wright (flat z = 30 m) vs 3D altitude-aware Clarke-Wright.

---

## Mathematical Model

### 3D Node Positions

Depot altitude: $z_d = 30$ m.

Customer altitude based on zone type:

$$z_c = \begin{cases} 30 & \text{residential} \\ 60 & \text{commercial} \\ 100 & \text{rooftop} \end{cases}$$

### 3D Energy Cost

$$E_{ij} = \|\mathbf{p}_j - \mathbf{p}_i\|_3 + \gamma \cdot |z_j - z_i|$$

where $\gamma = 1.5$ is the altitude-penalty factor (climbing costs more than horizontal flight).

### Clarke-Wright Savings (3D)

Savings from merging route ending at $i$ with route starting at $j$ (same depot $d$):

$$S_{ij}^{3D} = E_{d,i} + E_{d,j} - E_{i,j}$$

Merge greedily in decreasing order of $S_{ij}^{3D}$, subject to:
- Payload: $\sum_{c \in \text{route}} q_c \leq Q$
- Energy budget: $\sum_{\text{legs}} E_{ij} \leq E_{max}$

### Time-Window Penalty

$$\text{TWT} = \sum_{c \in \mathcal{C}} w_c \cdot \max\!\bigl(0,\; s_c - l_c\bigr)$$

### Route Execution Kinematics (3D)

Drone flies straight-line 3D segments at constant speed $v$:

$$\mathbf{p}_k(t) = \mathbf{p}_i + v \cdot (t - t_i) \cdot \frac{\mathbf{p}_j - \mathbf{p}_i}{\|\mathbf{p}_j - \mathbf{p}_i\|_3}$$

Leg travel time: $t_{ij} = \|\mathbf{p}_j - \mathbf{p}_i\|_3 / v$

Remaining energy after leg $(i,j)$: $E_{rem} \leftarrow E_{rem} - E_{ij}$

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Depots | 3, at (100,100,30), (500,900,30), (900,200,30) m |
| Customers | 12, random x,y in [50,950] m |
| Zone altitudes | Residential 30 m, Commercial 60 m, Rooftop 100 m |
| Drones | 6 total (2 per depot) |
| Cruising speed | 15 m/s (3D) |
| Payload capacity | 3.0 kg per drone |
| Energy budget | 5000 m-equivalent per sortie |
| Altitude penalty $\gamma$ | 1.5 |
| Service time per stop | 10 s |
| Time-window width | 60 – 180 s |
| Tardiness weight $\alpha$ | 0.7 |
| Demand per customer | 0.3 – 1.5 kg |
| Simulation timestep | 0.5 s |
| Random seed | 42 |

---

## Expected Output

- **3D route map**: depot markers (black squares), customer markers by zone type (residential = cyan, commercial = orange, rooftop = magenta), drone routes colour-coded by drone index
- **Altitude profiles**: per-drone altitude vs time showing climb/descent to zone altitudes
- **Energy comparison bar chart**: 2D (flat) vs 3D (altitude-aware) Clarke-Wright — total energy cost per strategy
- **Tardiness comparison**: total weighted tardiness for 2D vs 3D routing
- **Animation (GIF)**: 3D flight of all drones simultaneously

---

## Extensions

1. Altitude speed coupling — commercial drones fly faster at higher altitudes; re-solve with zone-specific speeds
2. Air traffic separation — add minimum vertical separation (10 m) between drones in the same x-y proximity; re-route to avoid conflicts
3. Wind profile — add altitude-dependent horizontal wind; update 3D energy model to account for headwind/tailwind at each zone level
4. Battery thermal model — energy consumption increases at higher altitudes due to air density drop; integrate altitude-dependent power curve
5. Pareto front — sweep altitude-penalty $\gamma \in [0, 3]$ and plot (energy cost, tardiness) Pareto frontier

---

## Related Scenarios

- Original: [S029 2D version](../S029_urban_logistics_scheduling.md)
- Prerequisites: [S021](../S021_point_delivery.md), [S022](../S022_obstacle_avoidance_delivery.md)
- Follow-ups: [S030](../S030_multi_depot_delivery.md), [S031](../S031_path_deconfliction.md)
- Algorithmic cross-reference: [S018](../../01_pursuit_evasion/S018_multi_target_intercept.md) (TSP formulation)
