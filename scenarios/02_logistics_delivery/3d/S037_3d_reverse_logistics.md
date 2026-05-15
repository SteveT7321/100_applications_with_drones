# S037 3D Upgrade — Reverse Logistics

**Domain**: Logistics & Delivery | **Difficulty**: ⭐⭐⭐⭐ | **Status**: `[ ]` Not Started
**Based on**: [S037 original](../S037_reverse_logistics.md)

---

## What Changes in 3D

The original S037 fixes all drone motion at a constant altitude ($z = 2$ m throughout). Customer positions are 2D points `pos[0:2]`, route lengths are Euclidean 2D norms, and the entire CVRPTW optimisation operates in the flat $x$-$y$ plane. Three important real-world factors are therefore absent:

1. **Terrain elevation**: pick-up locations may sit at different ground heights (rooftops, hillsides, loading docks), requiring drones to climb or descend when transitioning between stops.
2. **Load-attachment hover**: a drone collecting a returned parcel must descend to a precise hover altitude above the customer, stabilise, attach or winch the load, then ascend — adding a vertical manoeuvre cost to every pick-up event.
3. **3D obstacle avoidance**: urban columns, building edges, and no-fly altitude bands partition the 3D flight volume; a feasible route must clear these obstacles, which can lengthen the spatial path and therefore the energy budget.

This variant lifts $z$ to a free variable, adds terrain-height offsets $h_i$ at each customer, models the descent-hover-ascend attachment sequence, penalises flight through obstacle volumes, and recalculates route costs using full 3D Euclidean distances.

---

## Problem Definition

**Setup**: A fleet of $N = 5$ drones operates from a depot at $\mathbf{d} = (250, 250, 2)$ m. The $M = 12$ return-parcel customers are distributed across a $500 \times 500$ m arena; each customer $i$ sits at ground elevation $h_i$ m (sampled from $[0, 8]$ m to model multi-storey rooftop pick-ups). Drones cruise at a fixed transit altitude $z_{cruise} = 12$ m between stops, then descend to $z_{hover,i} = h_i + 1.2$ m to attach the parcel, then ascend back to $z_{cruise}$ before proceeding. Three cylindrical building obstacles occupy the arena; drones must route around them horizontally while maintaining altitude above $z_{cruise} - 1$ m near the obstacle radii.

**Roles**:
- **Fleet drones** ($N$ units): start and end at the depot; each carries at most $Q_{max} = 3.0$ kg; must perform a descent-hover-ascend sequence of total vertical travel $2 \cdot (z_{cruise} - z_{hover,i})$ at each stop.
- **Return customers** ($M$ locations): each has a 3D position $(x_i, y_i, h_i + 1.2)$ m at hover altitude, a parcel weight $q_i$, and a time window $[e_i, l_i]$ (pick-up must occur within this window).
- **Obstacles** ($K = 3$ cylinders): each defined by centre $(x_c^k, y_c^k)$, radius $r_c^k$, and height $H_c^k$; drones must stay outside the cylinder footprint whenever flying below $H_c^k$.
- **Depot**: ground level hub at $z = 2$ m; drones ascend to $z_{cruise}$ immediately after departure and descend before landing.

**Objective**: Assign customers to drones and sequence each drone's 3D pick-up route to

$$\min \sum_{k=1}^{N} \left( C_k^{dist} + \lambda \cdot V_k + \mu \cdot E_k^{range} + \nu \cdot W_k^{obs} \right)$$

where:
- $C_k^{dist}$ is the total 3D flight path cost including transit legs and vertical attachment excursions,
- $V_k$ is the total time-window lateness for drone $k$,
- $E_k^{range}$ is the excess route length beyond battery range $R_{max}$,
- $W_k^{obs}$ is the total obstacle penetration volume (zero for feasible routes),
- $\lambda, \mu, \nu$ are penalty weights.

Subject to the same capacity, time-window, and range constraints as S037, plus:

4. **Obstacle avoidance**: the horizontal waypoint path for each drone must not intersect any cylinder footprint at transit altitude.
5. **Vertical clearance**: $z_{cruise} \geq H_c^k$ whenever the drone is within $2 r_c^k$ of obstacle $k$ horizontally.

---

## Mathematical Model

### 3D Position and Transit Altitude

Customer $i$ has a 3D hover position:

$$\mathbf{p}_i = \begin{bmatrix} x_i \\ y_i \\ h_i + z_{offset} \end{bmatrix}, \quad z_{offset} = 1.2 \text{ m (winch clearance)}$$

All transit legs fly at $z_{cruise} = 12$ m. The 3D waypoint sequence for drone $k$ at stop $j$ is therefore three sub-legs:

$$\text{transit leg:} \quad \mathbf{w}^k_{j-1,top} \to \mathbf{w}^k_{j,top}, \quad z = z_{cruise}$$

$$\text{descent:} \quad \mathbf{w}^k_{j,top} \to \mathbf{p}_{i_j}, \quad \Delta z_j = z_{cruise} - (h_{i_j} + z_{offset})$$

$$\text{ascent:} \quad \mathbf{p}_{i_j} \to \mathbf{w}^k_{j,top}, \quad \Delta z_j \text{ (same)}$$

where $\mathbf{w}^k_{j,top} = (x_{i_j}, y_{i_j}, z_{cruise})$ is the overhead waypoint directly above stop $j$.

### 3D Route Cost

Total 3D path length for drone $k$:

$$C_k^{dist} = \underbrace{\sum_{j=0}^{n_k} \| \mathbf{w}^k_{j+1,top} - \mathbf{w}^k_{j,top} \|_{xy}}_{\text{horizontal transit}} + \underbrace{2 \sum_{j=1}^{n_k} \Delta z_j}_{\text{vertical attachment excursions}} + \underbrace{\Delta z_{depot,dep} + \Delta z_{depot,ret}}_{\text{depot climbs}}$$

with $\|\cdot\|_{xy}$ denoting the horizontal (2D) Euclidean norm and

$$\Delta z_{depot} = z_{cruise} - z_{depot} = 12 - 2 = 10 \text{ m}$$

### 3D Arrival Time Propagation

The travel time from stop $j-1$ to stop $j$ in 3D includes the horizontal transit at $v_c$ plus the descent and ascent at $v_z$:

$$\tau_{transit,j} = \frac{\|\mathbf{w}^k_{j,top} - \mathbf{w}^k_{j-1,top}\|_{xy}}{v_c} + \frac{2\,\Delta z_j}{v_z}$$

Arrival at customer $i_j$ (after descent):

$$t^k_j = \max\!\left(e_{i_j},\; t^k_{j-1} + \tau_{transit,j}\right)$$

Time-window violation:

$$v^k_j = \max\!\left(0,\; t^k_j - l_{i_j}\right), \qquad V_k = \sum_{j=1}^{n_k} v^k_j$$

### Obstacle Avoidance — Horizontal Detour

For each transit leg between horizontal waypoints $\mathbf{w}_A = (x_A, y_A)$ and $\mathbf{w}_B = (x_B, y_B)$, and for each cylinder obstacle $k$ with centre $\mathbf{c}_k$ and radius $r_c^k$, compute the minimum distance from the obstacle centre to the line segment $[\mathbf{w}_A, \mathbf{w}_B]$:

$$d_{seg}^k = \min_{s \in [0,1]} \|\mathbf{w}_A + s(\mathbf{w}_B - \mathbf{w}_A) - \mathbf{c}_k\|$$

If $d_{seg}^k < r_c^k + r_{safety}$ (with safety buffer $r_{safety} = 5$ m), the leg must be rerouted. The obstacle is bypassed by inserting a single tangent waypoint at:

$$\mathbf{q}_k^{\pm} = \mathbf{c}_k + (r_c^k + r_{safety})\,\hat{\mathbf{n}}_k^{\pm}$$

where $\hat{\mathbf{n}}_k^{\pm}$ is the unit vector perpendicular to the leg direction, choosing the sign that gives a shorter total detour:

$$\Delta L_{detour}^k = \|\mathbf{q}_k - \mathbf{w}_A\| + \|\mathbf{w}_B - \mathbf{q}_k\| - \|\mathbf{w}_B - \mathbf{w}_A\|$$

The obstacle penetration penalty applied during optimisation (before the tangent detour is confirmed) is:

$$W_k^{obs} = \sum_{\text{legs}} \max\!\left(0,\; r_c + r_{safety} - d_{seg}\right)$$

### Clarke-Wright Savings with 3D Cost

The savings matrix is recomputed using the 3D route cost function to account for vertical excursions:

$$s_{ij}^{3D} = C^{dist}(\{i\}, \mathbf{d}) + C^{dist}(\{j\}, \mathbf{d}) - C^{dist}(\{i, j\}, \mathbf{d})$$

where $C^{dist}(\mathcal{R}, \mathbf{d})$ is the full 3D path cost for serving route $\mathcal{R}$ including attachment excursions. Customers with high $\Delta z_i$ (tall buildings) are less attractive merge candidates because each visit contributes a fixed vertical overhead.

### 2-opt Improvement in 3D

The 2-opt reversal cost is evaluated using the 3D cost function (horizontal transit + attachment excursions + time-window penalty):

$$\Delta_{j,k}^{3D} = \left(C'^{dist}_k + \lambda V_k'\right) - \left(C_k^{dist} + \lambda V_k\right) < 0$$

The reversal of sub-sequence $[j, k]$ does not change attachment excursion costs (those are fixed per customer), so only the horizontal transit distances change:

$$\Delta L_{transit} = \|\mathbf{w}_{j-1} - \mathbf{w}_k\|_{xy} + \|\mathbf{w}_j - \mathbf{w}_{k+1}\|_{xy} - \|\mathbf{w}_{j-1} - \mathbf{w}_j\|_{xy} - \|\mathbf{w}_k - \mathbf{w}_{k+1}\|_{xy}$$

### Load-Attachment Hover Model

At each stop, the drone must maintain a stable hover at $z_{hover,i}$ for a dwell time $\tau_{attach}$ while the parcel is winched aboard:

$$\tau_{attach} = \tau_0 + \kappa \cdot q_i$$

where $\tau_0 = 8$ s is the base attachment time and $\kappa = 3$ s/kg scales with parcel weight. This dwell time is added to $\tau_{transit,j}$ in the arrival propagation:

$$t^k_j = \max\!\left(e_{i_j},\; t^k_{j-1} + \tau_{transit,j}\right) + \tau_{attach,j}$$

Note that the time window $[e_i, l_i]$ refers to the moment the drone arrives at $z_{hover,i}$, before the attachment dwell begins.

### Payload and Energy Budget

Cumulative payload after stop $j$ of drone $k$:

$$Q^k_j = \sum_{m=1}^{j} q_{i_m} \leq Q_{max}$$

3D energy proxy (proportional to path length with a vertical penalty factor $\gamma > 1$ for climb cost):

$$E_k = \underbrace{\sum_{j=0}^{n_k} \|\mathbf{w}^k_{j+1,top} - \mathbf{w}^k_{j,top}\|_{xy}}_{\text{horizontal}} + \underbrace{\gamma \cdot 2\sum_{j=1}^{n_k} \Delta z_j}_{\text{vertical climb overhead}} + \gamma \cdot 2\,\Delta z_{depot}$$

with $\gamma = 1.4$ (climbing consumes approximately 40% more energy per metre than horizontal cruise). Range constraint: $E_k \leq R_{max}$.

---

## Key 3D Additions

- **Transit altitude**: all inter-stop legs flown at $z_{cruise} = 12$ m; vertical excursions at each stop add fixed per-stop cost $2\,\Delta z_i$.
- **Terrain elevation offsets**: each customer has a ground height $h_i \in [0, 8]$ m; hover altitude $z_{hover,i} = h_i + 1.2$ m; tall-building customers incur larger vertical overhead.
- **Load-attachment dwell**: descent + hover + winch + ascent sequence adds $\tau_{attach,i} = \tau_0 + \kappa q_i$ to the time budget at each stop.
- **Cylindrical obstacle avoidance**: horizontal legs are checked against $K = 3$ building cylinders; penetrating legs receive a tangent detour waypoint; obstacle penalty drives the optimiser away from infeasible paths.
- **Vertical energy penalty**: climbing costs $\gamma = 1.4 \times$ horizontal energy per metre; the effective range budget $R_{max}$ is tighter for routes serving high-elevation customers.
- **3D savings matrix**: Clarke-Wright savings computed from full 3D route costs, making high-elevation stops less attractive to merge with distant low-elevation stops.

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Fleet size $N$ | 5 drones |
| Pick-up locations $M$ | 12 customers |
| Payload capacity $Q_{max}$ | 3.0 kg |
| Drone max energy range $R_{max}$ | 2 500 m (energy-equivalent) |
| Horizontal cruise speed $v_c$ | 8.0 m/s |
| Vertical speed $v_z$ | 2.0 m/s |
| Transit altitude $z_{cruise}$ | 12 m |
| Depot altitude $z_{depot}$ | 2 m |
| Customer ground elevation $h_i$ | 0 – 8 m (uniform) |
| Hover clearance $z_{offset}$ | 1.2 m |
| Attachment base time $\tau_0$ | 8 s |
| Attachment weight factor $\kappa$ | 3 s/kg |
| Parcel weight $q_i$ | 0.3 – 1.2 kg (uniform) |
| Earliest pick-up $e_i$ | 0 – 60 s (uniform) |
| Latest pick-up $l_i$ | 90 – 180 s (uniform) |
| Arena | $[50, 450]^2$ m |
| Depot position | $(250, 250, 2)$ m |
| Obstacle cylinders $K$ | 3 |
| Obstacle radius $r_c$ | 15 – 25 m (uniform) |
| Safety buffer $r_{safety}$ | 5 m |
| Vertical energy penalty $\gamma$ | 1.4 |
| Time-window penalty $\lambda$ | 50 s/m |
| Range penalty $\mu$ | 1 000 |
| Obstacle penalty $\nu$ | 500 |

---

## Expected Output

- **3D route plot**: full 3D trajectory for each drone including transit legs at $z_{cruise}$, descent/ascent spikes at each customer, and depot climbs/descents; cylindrical obstacles rendered as translucent grey columns; drone colours match S037 convention.
- **Top-down route map**: horizontal projections of all five routes with obstacle footprints; detour waypoints annotated; customer markers sized by parcel weight and coloured by elevation $h_i$.
- **Altitude profile per drone**: $z(t)$ time series for each drone showing the transit plateau, descent spikes to each hover altitude, and return to depot; horizontal dashed line at $z_{cruise}$.
- **Attachment excursion cost breakdown**: stacked bar chart per drone showing horizontal transit distance vs total vertical excursion distance vs dwell time overhead.
- **Route cost comparison (three optimisation stages)**: Clarke-Wright 3D, after 2-opt 3D, after Or-opt 3D — same three-stage structure as S037 but using 3D cost function.
- **Arrival-time vs time-window plot**: per customer, showing $[e_i, l_i]$ bar and actual arrival marker; colour-coded green/red for on-time/late; note increased lateness for high-elevation customers due to longer attachment time.
- **Animation (GIF)**: 3D animated trajectories; drones descend visibly at each stop and ascend before moving to next customer; obstacle cylinders shown as static grey volumes; payload counter displayed per drone.

---

## Extensions

1. **Variable transit altitude**: instead of a fixed $z_{cruise}$, allow each leg to choose its transit altitude within $[z_{cruise,min}, z_{cruise,max}]$ to minimise combined horizontal + vertical path length subject to obstacle clearance; formulate as a mixed-integer linear program on altitude bands.
2. **Wind-field integration**: couple with S024's wind compensation model — at high altitude ($z > 10$ m) a persistent crosswind adds to transit time; drones can choose a lower transit altitude to reduce wind exposure at the cost of greater obstacle proximity.
3. **Winch length optimisation**: if the winch cable has adjustable length $\ell \in [1, 5]$ m, the drone need not descend all the way to $h_i + 1.2$ m; optimise hover altitude $z_{hover,i} = h_i + (z_{offset,max} - \ell)$ jointly with route sequencing to minimise total vertical excursion.
4. **Multi-rooftop depot**: add a secondary elevated depot at $z = 20$ m (building rooftop); solve the 3D location-routing problem to assign each drone's home base and route jointly, exploiting the height advantage for customers in tall-building clusters.
5. **Battery state-of-charge model**: replace the linear range proxy $E_k$ with a nonlinear battery discharge model that accounts for hover efficiency losses during load attachment; re-optimise using energy-aware Or-opt with dynamic range limits per stop.

---

## Related Scenarios

- Original 2D version: [S037](../S037_reverse_logistics.md)
- 3D logistics references: [S021 3D](S021_3d_point_delivery.md), [S029 3D](S029_3d_urban_logistics.md), [S030 3D](S030_3d_multi_depot.md)
- Obstacle avoidance in 3D: [S022 3D](S022_3d_obstacle_avoidance.md)
- Related logistics domain: [S036](../S036_last_mile_relay.md), [S038](../S038_disaster_relief_drop.md)
