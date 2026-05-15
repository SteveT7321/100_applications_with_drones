# S031 3D Upgrade — Path De-confliction

**Domain**: Logistics & Delivery | **Difficulty**: ⭐⭐⭐ | **Status**: `[ ]` Not Started
**Based on**: [S031 original](../S031_path_deconfliction.md)

---

## What Changes in 3D

The original S031 fixes every drone at `z = 20 m` throughout the simulation. The altitude-layer
strategy (Strategy 3) pre-assigns each drone a static altitude offset from that single plane, but
the CPA conflict detector and both reactive resolution strategies (speed adjustment and lateral
waypoint insertion) operate exclusively in the x-y plane — all position and velocity vectors have
`acc[2] = 0` and the perpendicular normal $\hat{\mathbf{n}}_{ij}$ is always horizontal.

In the 3D upgrade, the full airspace column $z \in [15, 60]$ m is available. Conflict detection
becomes a proper 4D check $(x, y, z, t)$ using 3D CPA geometry and a cylindrical proximity model
(horizontal radius $r_{h}$ plus vertical half-height $h_v$) that matches real UTM separation
standards. Resolution strategies gain a third degree of freedom: drones may climb or descend to
avoid a conflict without any horizontal detour. A new Strategy 4 — **reactive vertical climb-to-
avoid** — is added alongside the original three, and all four are compared in the full 3D space.

---

## Problem Definition

**Setup**: A fleet of $N = 8$ delivery drones departs simultaneously from a shared depot at
$(250, 250, 20)$ m and flies pre-planned straight-line routes to distinct delivery waypoints in a
$500 \times 500 \times 45$ m column of urban airspace ($z \in [15, 60]$ m). Drones cruise at a
nominal speed of $v = 12$ m/s. Routes are generated in 3D: goal altitudes are drawn uniformly from
$[15, 50]$ m, making genuine 3D crossing conflicts possible even without horizontal path overlap.

The simulation compares four de-confliction strategies on three metrics: residual conflict count,
total added flight distance, and maximum arrival delay.

**Roles**:
- **Drones** $D_1, \ldots, D_N$: each assigned a 3D origin–destination pair
  $(\mathbf{o}_i \in \mathbb{R}^3, \mathbf{g}_i \in \mathbb{R}^3)$; nominal paths are
  straight-line segments in 3D
- **Conflict detector**: monitors all $\binom{N}{2}$ pairs using a 4D bounding-cylinder test;
  flags a conflict when the predicted CPA violates the cylindrical separation envelope
- **De-confliction planner**: applies one of four resolution strategies to every flagged pair

**Objective**:
1. Ensure no two drones ever penetrate the cylindrical separation envelope (safety constraint)
2. Minimise total added 3D flight distance summed over the fleet
3. Minimise maximum arrival delay over all drones

**Strategies**:
1. **Priority-based speed adjustment** — lower-priority drone decelerates; now solved in full 3D
2. **Horizontal waypoint insertion** — lateral detour in the horizontal plane (3D baseline for comparison)
3. **Altitude layer pre-assignment** — each drone assigned a unique altitude corridor before departure
4. **Reactive vertical climb-to-avoid** — lower-priority drone inserts a vertical avoidance waypoint,
   climbing or descending by $\Delta z_{avoid}$ to clear the conflict, then resumes original altitude

---

## Mathematical Model

### 3D Nominal Trajectory

Drone $i$ departs at $t = 0$ from $\mathbf{o}_i \in \mathbb{R}^3$ toward $\mathbf{g}_i \in \mathbb{R}^3$
at speed $v$:

$$\mathbf{p}_i(t) = \mathbf{o}_i + v \cdot t \cdot \hat{\mathbf{u}}_i,
  \qquad \hat{\mathbf{u}}_i = \frac{\mathbf{g}_i - \mathbf{o}_i}{\|\mathbf{g}_i - \mathbf{o}_i\|}$$

The unit direction vector $\hat{\mathbf{u}}_i$ now has a non-zero $z$-component wherever the goal
altitude differs from the departure altitude, so the nominal path is a true 3D line segment.

### 4D Conflict Detection — Bounding Cylinder CPA

For drone pair $(i, j)$ with 3D positions and velocities, define the relative quantities:

$$\Delta\mathbf{p}_{ij}(t) = \mathbf{p}_i(t) - \mathbf{p}_j(t), \qquad
  \Delta\mathbf{v}_{ij} = \mathbf{v}_i - \mathbf{v}_j$$

Time of closest point of approach in 3D:

$$t_{CPA} = \max\!\left(0,\; -\frac{\Delta\mathbf{p}_{ij}(0) \cdot \Delta\mathbf{v}_{ij}}
                                    {\|\Delta\mathbf{v}_{ij}\|^2}\right)$$

At $t_{CPA}$ the 3D separation vector is $\boldsymbol{\delta} = \Delta\mathbf{p}_{ij}(t_{CPA})$.
Decompose into horizontal and vertical components:

$$d_{h} = \sqrt{\delta_x^2 + \delta_y^2}, \qquad d_{v} = |\delta_z|$$

A **conflict** is declared when:

$$d_{h} < r_{sep} \quad \text{AND} \quad d_{v} < h_{sep} \quad \text{AND} \quad t_{CPA} \in (0,\, T_{horizon}]$$

with horizontal separation radius $r_{sep} = 5$ m and vertical separation half-height $h_{sep} = 3$ m.
This cylindrical envelope replaces the scalar sphere used in S031, matching ICAO UTM standards for
low-altitude UAS.

### Strategy 1 — Priority-Based Speed Adjustment (3D)

Unchanged in logic but now operates in full 3D. When pair $(i,j)$ with lower priority $\pi_i$ conflicts,
bisect on the speed scalar $v' \in [v_{min},\, v]$ so that the modified velocity
$\mathbf{v}'_i = v' \hat{\mathbf{u}}_i$ satisfies the cylindrical CPA condition with margin:

$$d'_h \geq r_{sep} + \delta_{margin} \quad \text{or} \quad d'_v \geq h_{sep} + \delta_{margin}$$

The arrival-time penalty is identical to the 2D case:

$$\Delta T_i^{speed} = \frac{\|\mathbf{g}_i - \mathbf{o}_i\|}{v'} - \frac{\|\mathbf{g}_i - \mathbf{o}_i\|}{v}$$

### Strategy 2 — Horizontal Waypoint Insertion (3D Baseline)

A lateral avoidance waypoint is inserted at the predicted CPA midpoint displaced in the horizontal
plane by $\rho = r_{sep} + \delta_{margin}$:

$$\mathbf{w}_{avoid} = \mathbf{p}_{mid} + \rho \cdot \hat{\mathbf{n}}_{ij}^{xy}$$

where $\mathbf{p}_{mid} = \frac{1}{2}\!\left(\mathbf{p}_i(t_{CPA}) + \mathbf{p}_j(t_{CPA})\right)$
retains the full 3D midpoint, and the horizontal normal:

$$\hat{\mathbf{n}}_{ij}^{xy} = \frac{[-\Delta v_y,\; \Delta v_x,\; 0]^\top}
                                    {\sqrt{\Delta v_x^2 + \Delta v_y^2}}$$

The $z$-coordinate of $\mathbf{w}_{avoid}$ is held at $\mathbf{p}_{mid,z}$ so the detour stays
in the horizontal plane of the predicted conflict. Added 3D flight distance:

$$\Delta L_i^{(2)} = \|\mathbf{o}_i - \mathbf{w}_{avoid}\|_3
                   + \|\mathbf{w}_{avoid} - \mathbf{g}_i\|_3
                   - \|\mathbf{g}_i - \mathbf{o}_i\|_3$$

### Strategy 3 — Altitude Layer Pre-Assignment (3D)

Each drone is assigned a unique altitude corridor:

$$z_i = z_{base} + (i - 1) \cdot \Delta z, \qquad i = 1, \ldots, N$$

with $z_{base} = 20$ m and $\Delta z = 5$ m, spanning $[20, 55]$ m for $N = 8$. Because
$\Delta z = 5 > h_{sep} = 3$ m, vertical separation is guaranteed at all times irrespective of
horizontal proximity. Drones climb or descend from the depot to their assigned altitude at the
start of the mission; this transition adds a 3D path cost:

$$\Delta L_i^{(3)} = \|\mathbf{o}_i - \mathbf{o}_i^{layer}\|_3
                   + \|\mathbf{g}_i^{layer} - \mathbf{g}_i\|_3$$

where $\mathbf{o}_i^{layer}$ and $\mathbf{g}_i^{layer}$ are the depot and goal positions adjusted
to altitude $z_i$.

### Strategy 4 — Reactive Vertical Climb-to-Avoid (New in 3D)

When a conflict is detected for pair $(i, j)$ with lower priority $\pi_i$, drone $i$ inserts a
vertical avoidance waypoint directly above (or below) its current position by $\Delta z_{avoid}$,
chosen so that the new vertical separation satisfies $d'_v \geq h_{sep} + \delta_{margin}$:

$$\Delta z_{avoid} = \text{sign}(\delta_{z,ij}) \cdot (h_{sep} + \delta_{margin} - |\delta_{z,ij}|)$$

The avoidance waypoint in 3D:

$$\mathbf{w}_{vert} = \mathbf{p}_i(t_{CPA})
                    + \begin{bmatrix} 0 \\ 0 \\ \Delta z_{avoid} \end{bmatrix}$$

After passing $\mathbf{w}_{vert}$, the drone routes directly to its goal $\mathbf{g}_i$. The
vertical detour incurs no added horizontal distance; the 3D path cost is:

$$\Delta L_i^{(4)} = \|\mathbf{p}_i(t_{ins}) - \mathbf{w}_{vert}\|_3
                   + \|\mathbf{w}_{vert} - \mathbf{g}_i\|_3
                   - \|\mathbf{g}_i - \mathbf{p}_i(t_{ins})\|_3$$

where $t_{ins}$ is the wall-clock time at which the manoeuvre is triggered.

### Fleet-Level Metrics (3D)

Residual conflict count at time $t$, using the cylindrical test:

$$C_s(t) = \sum_{i < j} \mathbf{1}\!\left[d_{h,ij}(t) < r_{sep}
           \;\wedge\; d_{v,ij}(t) < h_{sep}\right]$$

Total added 3D flight distance:

$$\Delta L_{total}^{(s)} = \sum_{i=1}^{N} \Delta L_i^{(s)}$$

Maximum arrival delay:

$$\Delta T_{max}^{(s)} = \max_{i} \left(T_i^{actual} - T_i^{nom}\right), \qquad
  T_i^{nom} = \frac{\|\mathbf{g}_i - \mathbf{o}_i\|_3}{v}$$

---

## Key 3D Additions

- **Full 3D airspace column**: $z \in [15, 60]$ m; goal altitudes randomised, creating genuine 3D crossing conflicts
- **Cylindrical CPA envelope**: horizontal radius $r_{sep} = 5$ m plus vertical half-height $h_{sep} = 3$ m replaces scalar sphere
- **4D conflict schedule**: conflicts are detected in $(x, y, z, t)$ space; look-ahead uses the full 3D relative velocity
- **Strategy 4 — vertical climb-to-avoid**: inserts a waypoint in $z$ only, incurring no horizontal detour; often the cheapest resolution in 3D
- **Altitude time series**: per-drone $z(t)$ plots expose which strategy keeps drones at their nominal altitude vs. forces climbs/descents
- **3D trajectory visualisation**: all four strategies rendered on separate 3D axes with altitude colour-coding

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Fleet size $N$ | 8 drones |
| Cruise speed $v$ | 12.0 m/s |
| Minimum speed (strategy 1) $v_{min}$ | 4.0 m/s |
| Horizontal separation radius $r_{sep}$ | 5.0 m |
| Vertical separation half-height $h_{sep}$ | 3.0 m |
| Clearance margin $\delta_{margin}$ | 2.0 m |
| Look-ahead window $T_{horizon}$ | 30 s |
| Altitude bounds $z$ | 15 – 60 m |
| Nominal depot altitude | 20.0 m |
| Goal altitudes | uniform random $\in [15, 50]$ m |
| Altitude layer spacing $\Delta z$ (strategy 3) | 5.0 m |
| Altitude layer range (strategy 3) | 20 – 55 m |
| Vertical avoid offset $\Delta z_{avoid}$ (strategy 4) | computed per conflict |
| Airspace arena | $500 \times 500 \times 45$ m |
| Simulation timestep $\Delta t$ | 0.1 s |

---

## Expected Output

- **3D trajectory plots**: one subplot per strategy showing all 8 drone paths in 3D; altitude encoded in colour gradient
- **Top-down 2D projection**: horizontal footprints of all four strategies for direct comparison with S031 2D results
- **Altitude time series**: $z_i(t)$ for all drones under each strategy; strategy 3 shows flat horizontal bands, strategy 4 shows transient spikes
- **Horizontal and vertical separation time series**: $d_{h,\min}(t)$ and $d_{v,\min}(t)$ vs time; dashed reference lines at $r_{sep}$ and $h_{sep}$
- **Strategy comparison bar chart**: total added 3D flight distance and maximum arrival delay for all four strategies
- **Conflict event count** printed per strategy (strategy 3 should be zero; strategy 4 typically lowest added distance)

---

## Extensions

1. **Mixed-altitude fleet with overtaking**: drones cruise at different altitudes and speeds; test whether cylindrical separation is maintained during en-route altitude transitions
2. **Dynamic no-fly columns**: vertical cylinders (building rooftops, restricted airspace pillars) intersect the flight corridor; extend strategy 4 to route around static 3D obstacles using RRT*
3. **Cascading conflict resolution**: after any reactive manoeuvre, re-run 4D conflict detection to catch secondary conflicts introduced by the altitude change
4. **Velocity Obstacle in 3D**: replace all four strategies with a unified 3D VO/ORCA reactive planner operating in full $(v_x, v_y, v_z)$ space
5. **Energy-optimal vertical avoidance**: model climb power as $P_{climb} = m g v_z / \eta$; find the altitude offset that resolves the conflict with minimum energy expenditure rather than minimum added distance

---

## Related Scenarios

- Original 2D version: [S031](../S031_path_deconfliction.md)
- 3D logistics references: [S022 3D](S022_3d_obstacle_avoidance.md), [S029 3D](S029_3d_urban_logistics.md)
- Follow-ups: [S032](../S032_charging_queue.md), [S033](../S033_online_order_insertion.md)
- Algorithmic cross-reference: [S028 3D](S028_3d_cargo_escort.md) (3D formation separation), [S022](../S022_obstacle_avoidance_delivery.md) (obstacle avoidance)
