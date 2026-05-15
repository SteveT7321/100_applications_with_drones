# S033 3D Upgrade — Online Order Insertion

**Domain**: Logistics & Delivery | **Difficulty**: ⭐⭐⭐⭐ | **Status**: `[ ]` Not Started
**Based on**: [S033 original](../S033_online_order_insertion.md)

---

## What Changes in 3D

The original S033 treats all waypoints as 2D ground-projected positions with a fixed altitude of `z = 2.0 m` hardcoded for every waypoint in the fleet. The insertion cost $\Delta_{k,i}$ uses only the Euclidean norm of the x-y difference, and there is no concept of altitude corridor assignment or vertical separation between drones sharing the same horizontal airspace.

In the 3D upgrade, each drone is assigned a distinct altitude corridor (its nominal cruise altitude $z_k$), new orders carry a requested delivery altitude $z_{new}$, and the full 3D Euclidean distance drives every cost and feasibility calculation. Inserting an order now requires the dispatcher to evaluate whether the drone's altitude corridor is compatible with the destination altitude, and to account for the vertical climb or descent leg as part of the detour cost. Real-time route replanning therefore operates on 3D waypoint sequences, and a new altitude-reassignment step may shift a drone to a temporary altitude band to execute the delivery before returning to its nominal corridor.

---

## Problem Definition

**Setup**: A fleet of $K = 4$ drones executes a pre-planned 3D delivery schedule. Each drone $D_k$ cruises at its assigned altitude corridor $z_k \in \{3, 5, 7, 9\}$ m and holds an ordered 3D waypoint queue. At time $t_{arrive}$, a new order arrives at 3D location $\mathbf{q}_{new} = (x_{new}, y_{new}, z_{new})$ with urgency weight $w$ and deadline $t_{deadline}$. The dispatcher must immediately assign the order to one drone and splice the 3D delivery point into that drone's remaining route — without halting any in-flight drone.

**Roles**:
- **Drones** $D_1, \ldots, D_K$: each carries a 3D partial route $R_k = [\mathbf{w}^k_0, \mathbf{w}^k_1, \ldots, \mathbf{w}^k_{n_k}]$ where $\mathbf{w}^k_i \in \mathbb{R}^3$ and $\mathbf{w}^k_0$ is the current 3D position; each drone has a nominal corridor altitude $z_k$
- **Dispatcher**: greedy online algorithm that evaluates every feasible 3D insertion position across all drones, including altitude corridor compatibility, and picks the global minimum-cost insertion
- **New order**: 3D point $\mathbf{q}_{new}$ with delivery altitude $z_{new}$, deadline $t_{deadline}$, and urgency weight $w$

**Objective**: Minimise the total additional 3D flight distance (detour cost) introduced by the insertion, subject to:
1. Battery feasibility: the drone's updated 3D route must not exceed remaining range $B_k^{rem}$
2. Deadline feasibility: the drone can reach $\mathbf{q}_{new}$ before $t_{deadline}$
3. Altitude corridor separation: no two drones occupy the same altitude band simultaneously within a horizontal conflict radius $r_{sep}$

---

## Mathematical Model

### 3D Route Representation

Drone $k$ at insertion time has remaining 3D waypoints:

$$R_k = [\mathbf{w}^k_0,\; \mathbf{w}^k_1,\; \ldots,\; \mathbf{w}^k_{n_k}], \quad \mathbf{w}^k_i \in \mathbb{R}^3$$

The remaining 3D route length is:

$$L_k = \sum_{i=0}^{n_k - 1} \|\mathbf{w}^k_{i+1} - \mathbf{w}^k_i\|_2$$

where $\|\cdot\|_2$ denotes the full Euclidean norm in $\mathbb{R}^3$.

### 3D Cheapest-Insertion Cost

For drone $k$, inserting the 3D delivery point $\mathbf{q}_{new}$ between consecutive waypoints $\mathbf{w}^k_i$ and $\mathbf{w}^k_{i+1}$:

$$\Delta_{k,i}^{3D} = \|\mathbf{w}^k_i - \mathbf{q}_{new}\|_2 + \|\mathbf{q}_{new} - \mathbf{w}^k_{i+1}\|_2 - \|\mathbf{w}^k_i - \mathbf{w}^k_{i+1}\|_2$$

The vertical detour component is explicitly:

$$\Delta_{k,i}^{z} = |z^k_i - z_{new}| + |z_{new} - z^k_{i+1}| - |z^k_i - z^k_{i+1}|$$

where $z^k_i$ denotes the altitude of $\mathbf{w}^k_i$. Note $\Delta_{k,i}^{z} \ge 0$ and the full $\Delta_{k,i}^{3D}$ subsumes it.

The best 3D insertion position for drone $k$:

$$i^*_k = \arg\min_{0 \le i < n_k} \Delta_{k,i}^{3D}$$

The cheapest-insertion cost for drone $k$:

$$\delta_k = \Delta_{k,\, i^*_k}^{3D}$$

### Altitude Corridor Compatibility Penalty

Each drone $k$ has a nominal corridor altitude $z_k$. When $|z_{new} - z_k| > \Delta z_{tol}$, a corridor-switch overhead is added to represent the additional climb/descent leg back to the nominal altitude after delivery:

$$\phi_k = \max\!\left(0,\; |z_{new} - z_k| - \Delta z_{tol}\right) \cdot \gamma$$

where $\Delta z_{tol} = 1.5$ m is the tolerance within which no corridor switch is charged, and $\gamma \ge 1$ is a penalty multiplier (default $\gamma = 1.2$). The adjusted assignment cost becomes:

$$\tilde{\delta}_k = \delta_k + \phi_k$$

### Battery Feasibility Constraint

Remaining battery expressed as equivalent flight distance:

$$B_k^{rem} = B_{full} \cdot \frac{E_k^{rem}}{E_{full}}$$

Feasibility requires:

$$L_k + \delta_k \le B_k^{rem}$$

### Deadline Feasibility Constraint

Time for drone $k$ to reach $\mathbf{q}_{new}$ via the cheapest 3D insertion point, flying at speed $v_k$:

$$t_{arrive,k} = t_{now} + \frac{\displaystyle\sum_{j=0}^{i^*_k - 1} \|\mathbf{w}^k_{j+1} - \mathbf{w}^k_j\|_2 + \|\mathbf{w}^k_{i^*_k} - \mathbf{q}_{new}\|_2}{v_k}$$

Feasibility requires:

$$t_{arrive,k} \le t_{deadline}$$

### Altitude Corridor Separation Constraint

At any time $t$, two drones $k$ and $k'$ must not share the same altitude band within a horizontal conflict radius:

$$\text{conflict}(k, k', t) = \mathbf{1}\!\left[\,|z_k(t) - z_{k'}(t)| < h_{sep} \;\wedge\; \|(x_k(t), y_k(t)) - (x_{k'}(t), y_{k'}(t))\|_2 < r_{sep}\,\right] = 0$$

where $h_{sep} = 1.5$ m and $r_{sep} = 20$ m. Before finalising an insertion the dispatcher checks the projected altitude profile of $D_{k^*}$ against all other drones over the time interval $[t_{now},\; t_{arrive,k^*} + 30]$ s.

### Dispatcher Decision Rule

Among all feasible $(k, i^*_k)$ pairs, select:

$$k^* = \arg\min_{k \in \mathcal{F}} \tilde{\delta}_k$$

where the feasible set is:

$$\mathcal{F} = \{k : \text{battery feasible} \cap \text{deadline feasible} \cap \text{no corridor conflict}\}$$

If $\mathcal{F} = \emptyset$, the order is queued for the next available drone (earliest 3D route completion time).

### 3D Route Update

After assignment, drone $k^*$ route becomes:

$$R_{k^*} \leftarrow [\mathbf{w}^{k^*}_0,\; \ldots,\; \mathbf{w}^{k^*}_{i^*},\; \mathbf{q}_{new},\; \mathbf{w}^{k^*}_{i^*+1},\; \ldots,\; \mathbf{w}^{k^*}_{n_{k^*}}]$$

If $|z_{new} - z_{k^*}| > \Delta z_{tol}$, a corridor-return waypoint $\mathbf{w}^{ret}_{k^*} = (x_{new}, y_{new}, z_{k^*})$ is appended immediately after $\mathbf{q}_{new}$ to restore the drone to its nominal altitude band before it continues to $\mathbf{w}^{k^*}_{i^*+1}$:

$$R_{k^*} \leftarrow [\ldots,\; \mathbf{w}^{k^*}_{i^*},\; \mathbf{q}_{new},\; \mathbf{w}^{ret}_{k^*},\; \mathbf{w}^{k^*}_{i^*+1},\; \ldots]$$

### Comparison Strategies in 3D

| Strategy | Assignment rule |
|----------|----------------|
| **3D Cheapest Insertion** (proposed) | Global $\min \tilde{\delta}_k$ across all feasible drones including altitude penalty |
| **Nearest Drone 3D** | Drone with smallest $\|\mathbf{w}^k_0 - \mathbf{q}_{new}\|_2$ (3D current position) |
| **Corridor-Preferred** | Among feasible drones, prefer drone whose $z_k$ is closest to $z_{new}$ |
| **Least Loaded** | Drone with smallest remaining $n_k$ regardless of 3D geometry |

---

## Key 3D Additions

- **3D waypoints**: all positions are $\mathbb{R}^3$ vectors; $z$ is no longer hardcoded to 2.0 m
- **Altitude corridors**: each drone $D_k$ is assigned a distinct nominal cruise altitude $z_k \in \{3, 5, 7, 9\}$ m to maintain vertical separation
- **3D insertion cost**: $\Delta_{k,i}^{3D}$ uses full Euclidean distance, explicitly capturing vertical detour
- **Corridor compatibility penalty**: $\phi_k$ penalises assignments that force a drone far outside its altitude band
- **Corridor-return waypoint**: after a cross-altitude delivery, a synthetic waypoint restores the drone to $z_k$
- **Altitude conflict check**: dispatcher pre-screens for $h_{sep}$ / $r_{sep}$ violations before confirming an assignment
- **3D trajectory visualisation**: routes plotted on 3D axes with altitude-coded colouring per drone corridor

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Fleet size $K$ | 4 drones |
| Cruise speed $v$ | 8.0 m/s |
| Full battery range $B_{full}$ | 10 000 m |
| Arena (x, y) | $[-500, 500]^2$ m |
| Altitude corridors $z_k$ | 3, 5, 7, 9 m |
| Delivery altitude range $z_{new}$ | 1.5 – 10 m |
| Corridor tolerance $\Delta z_{tol}$ | 1.5 m |
| Corridor penalty multiplier $\gamma$ | 1.2 |
| Vertical separation $h_{sep}$ | 1.5 m |
| Horizontal conflict radius $r_{sep}$ | 20 m |
| Pre-planned waypoints per drone | 12 |
| Online orders $N_{ins}$ | 8 |
| Order arrival window | $t \in [10, 80]$ s |
| Deadline slack | $+30$ s beyond nearest-drone 3D ETA |
| Simulation timestep $\Delta t$ | 0.1 s |
| Depot | $(0, 0, 5)$ m |

---

## Expected Output

- 3D trajectory plot: original planned routes (dashed) vs updated routes after all insertions (solid), altitude-coded per corridor, new order locations marked as stars at their full $z_{new}$ coordinates
- Altitude vs time subplot: all four drone altitude profiles showing corridor excursions during cross-altitude deliveries and corridor-return legs
- Detour cost comparison bar chart: 3D Cheapest Insertion vs Nearest Drone 3D vs Corridor-Preferred vs Least Loaded — total additional 3D distance
- Vertical detour breakdown: stacked bar showing horizontal vs vertical components of $\Delta_{k,i}^{3D}$ per insertion event
- Feasibility rate table: fraction of orders served on-time per strategy, including corridor-conflict rejections
- Per-drone load balance histogram: number of inserted orders per drone under each strategy

---

## Extensions

1. Altitude-aware re-optimisation: after each 3D insertion apply 2-opt local search using the full 3D distance metric, comparing routes that share a corridor rearrangement against those that keep each drone in its band
2. Dynamic corridor reassignment: allow the dispatcher to permanently shift a drone's nominal $z_k$ when repeated deliveries cluster at a non-native altitude, minimising accumulated corridor-return overhead
3. Wind-layered altitudes: assign corridors to exploit altitude-dependent wind profiles — lower altitudes may have headwinds while higher altitudes provide tailwinds — and incorporate an altitude-dependent speed model $v_k(z)$
4. Conflict-aware scheduling: when $|\mathcal{F}| = 0$ due to corridor conflicts, implement a brief timed hold-and-retry loop (up to 5 s) before queuing the order
5. Multi-depot 3D: drones return to the nearest of $M$ charging depots at potentially different altitudes; insertion cost must include the 3D leg to the recharge depot

---

## Related Scenarios

- Original 2D version: [S033](../S033_online_order_insertion.md)
- 3D logistics references: S034 Weather Rerouting, S035 UTM Simulation
- Truly 3D domain references: [S001](../../01_pursuit_evasion/S001_basic_intercept.md), [S003](../../01_pursuit_evasion/S003_low_altitude_tracking.md)
