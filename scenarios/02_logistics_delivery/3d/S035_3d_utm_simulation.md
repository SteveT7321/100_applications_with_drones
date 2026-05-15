# S035 3D Upgrade — UTM Simulation

**Domain**: Logistics & Delivery | **Difficulty**: ⭐⭐⭐ | **Status**: `[ ]` Not started
**Based on**: [S035 original](../S035_utm_simulation.md)

---

## What Changes in 3D

The original S035 organises the airspace into four fixed altitude bands (L1–L4 at 20–40, 40–60, 60–80, 80–100 m), but drones travel horizontally within each band at a single nominal altitude — vertical position is essentially constant per corridor segment. Conflict detection decomposes separation into a horizontal component $\|\cdot\|_H$ and a scalar vertical gap $|z_k - z_j|$, which are checked independently; the 3D separation vector is never treated as a single geometric quantity. Resolution is purely time-based (holding delays) with no notion of vertical manoeuvre.

This 3D variant replaces those simplifications with:

1. **Continuous altitude-layer corridor allocation**: each drone is assigned a full 3D corridor tube — a cylinder with a true 3D centreline that climbs or descends through transition segments between layers — rather than snapping to a fixed $z$ band.
2. **Unified 3D conflict zones**: separation is enforced as a single 3D safety sphere of radius $r_{sep}$ around each drone rather than two independent thresholds, enabling more compact packing without sacrificing safety.
3. **Vertical separation standards**: intra-layer vertical buffers are augmented by explicit climb/descent angle limits $\gamma_{max}$ and inter-layer transition corridors, preventing simultaneous use of the same vertical slice by crossing drones.
4. **4D trajectory negotiation with vertical manoeuvre options**: conflict resolution may now command an altitude offset manoeuvre — a vertical detour of $\Delta z_{res}$ m — as a first choice before resorting to time-based holding, reducing total fleet delay on dense crossing routes.

---

## Problem Definition

**Setup**: An urban airspace of $1000 \times 1000 \times 150$ m is partitioned into $L = 6$ altitude layers, each hosting directed corridor tubes. $N = 12$ delivery drones submit 4D trajectory intents; the UTM service registers intents, detects 3D conflicts, and resolves them through (i) altitude layer reallocation, (ii) vertical offset manoeuvres, or (iii) time-based departure sequencing — applied in that order of preference.

**Roles**:
- **UTM Service**: maintains the 3D airspace state volume, processes intent registrations, detects 3D conflicts via sphere-based separation, and issues 3D resolution advisories (altitude change commands or holding delays).
- **Drones** ($N = 12$): execute assigned 4D trajectories including climb/descent segments; respond to RAs by adjusting altitude or speed; report full 3D state to the UTM at each telemetry interval.
- **Corridor Tubes**: directed 3D cylinders defined by a 3D centreline polyline, a tube radius $r_c$, a maximum drone density $\rho_{max}$, and a maximum climb/descent angle $\gamma_{max}$.

**Objective**: All $N$ drones complete their deliveries while:

1. Maintaining 3D separation $\|\mathbf{p}_k(t) - \mathbf{p}_j(t)\| \geq r_{sep} = 35$ m at all times.
2. Satisfying per-layer altitude bounds and inter-layer transition angle constraints $|\gamma_k| \leq \gamma_{max} = 20°$.
3. Minimising total fleet delay; preferring altitude adjustments over time delays when the altitude option costs less than a configurable delay threshold $T_{thresh}$.

**Comparison strategies**:
1. **FCFS with time-only resolution**: conflicts resolved solely by holding delays in submission order (2D baseline behaviour lifted to 3D geometry).
2. **Altitude-first resolution**: UTM first attempts to assign an alternate altitude layer or issue a vertical offset command; falls back to time delay only if no feasible altitude option exists.
3. **Joint 4D LP optimisation**: jointly optimise both departure time offsets $\delta_k$ and discrete altitude layer assignments $\ell_k \in \{1,\ldots,L\}$ via MILP relaxation to minimise total delay.

---

## Mathematical Model

### 3D Airspace Structure

The airspace is partitioned into $L = 6$ altitude layers:

| Layer | Altitude band | Nominal heading | Role |
|-------|--------------|-----------------|------|
| L1 | 20 – 40 m | Eastbound | Outbound low |
| L2 | 40 – 60 m | Northbound | Outbound low |
| L3 | 60 – 80 m | Westbound | Return low |
| L4 | 80 – 100 m | Southbound | Return low |
| L5 | 100 – 120 m | Any | Express / priority |
| L6 | 120 – 150 m | Any | Emergency / override |

Each layer hosts $M_l$ directed 3D corridor tubes. A corridor $c$ in layer $l$ is defined by its 3D centreline polyline $\{\mathbf{q}_{c,0}, \mathbf{q}_{c,1}, \ldots\}$ where $\mathbf{q} \in \mathbb{R}^3$, its tube radius $r_c$, and its climb angle limit:

$$c = \bigl(\{\mathbf{q}_{c,i}\},\; r_c,\; z_{lo,l},\; z_{hi,l},\; \gamma_{max}\bigr)$$

A drone position $\mathbf{p}$ is inside corridor $c$ if the minimum distance from $\mathbf{p}$ to the centreline polyline satisfies:

$$d(\mathbf{p},\, c) = \min_i \, d\bigl(\mathbf{p},\, \text{seg}(\mathbf{q}_{c,i}, \mathbf{q}_{c,i+1})\bigr) \leq r_c$$

### 4D Trajectory Intent with Altitude Legs

Drone $k$ submits a piecewise-linear 4D trajectory in full 3D:

$$\Pi_k = \bigl\{(\mathbf{p}_{k,0}, t_{k,0}),\; (\mathbf{p}_{k,1}, t_{k,1}),\; \ldots,\; (\mathbf{p}_{k,n_k}, t_{k,n_k})\bigr\}, \quad \mathbf{p}_{k,i} \in \mathbb{R}^3$$

Position at time $t$ during leg $[t_{k,i},\, t_{k,i+1}]$:

$$\mathbf{p}_k(t) = \mathbf{p}_{k,i} + \frac{t - t_{k,i}}{t_{k,i+1} - t_{k,i}} \bigl(\mathbf{p}_{k,i+1} - \mathbf{p}_{k,i}\bigr)$$

The climb angle of leg $i$ is:

$$\gamma_{k,i} = \arctan\!\left(\frac{z_{k,i+1} - z_{k,i}}{\|\mathbf{p}_{k,i+1}^{xy} - \mathbf{p}_{k,i}^{xy}\|}\right)$$

Intent registration rejects any leg with $|\gamma_{k,i}| > \gamma_{max}$.

### 3D Conflict Detection (Sphere-Based)

A **conflict** between drones $k$ and $j$ exists if there is any time $t \in [t_{start}, t_{end}]$ such that:

$$\|\mathbf{p}_k(t) - \mathbf{p}_j(t)\| < r_{sep}$$

For piecewise-linear trajectories, the squared inter-drone distance on a leg pair reduces to a 1D quadratic in relative time $\tau$:

$$\delta(\tau) = \|\Delta\mathbf{p}_0 + \tau \Delta\mathbf{v}\|^2, \quad \tau \in [0,\, \Delta t_{leg}]$$

where $\Delta\mathbf{p}_0 = \mathbf{p}_k(t_{leg}) - \mathbf{p}_j(t_{leg})$ is the 3D relative position at the start of the leg overlap window, and $\Delta\mathbf{v} = \mathbf{v}_k - \mathbf{v}_j$ is the 3D relative velocity. The minimum separation time is:

$$\tau^* = \text{clip}\!\left(-\frac{\Delta\mathbf{p}_0 \cdot \Delta\mathbf{v}}{\|\Delta\mathbf{v}\|^2},\; 0,\; \Delta t_{leg}\right)$$

The minimum 3D separation during the leg is $\sqrt{\delta(\tau^*)}$.

### Altitude Layer Allocation

Given $L$ layers, the feasible altitude assignments for drone $k$ form the set:

$$\mathcal{L}_k = \bigl\{\ell \in \{1,\ldots,L\} : z_{lo,\ell} \leq z_k^{nominal} \leq z_{hi,\ell}\bigr\}$$

After conflict detection, the UTM attempts to reassign the lower-priority conflicting drone to an alternate layer $\ell' \in \mathcal{L}_k \setminus \{\ell_k\}$. The reassigned trajectory shifts $z$ by the layer offset:

$$z_k'(t) = z_k(t) + \bigl(z_{mid,\ell'} - z_{mid,\ell_k}\bigr)$$

$$z_{mid,\ell} = \frac{z_{lo,\ell} + z_{hi,\ell}}{2}$$

The vertical transition segment uses a constant climb/descent rate:

$$\dot{z}_{transition} = v_{cruise} \cdot \tan\gamma_{max}$$

### Vertical Offset Manoeuvre (3D RA)

When a predicted 3D separation violation is detected by the separation assurance monitor, the UTM issues a **vertical offset RA** to drone $j$, commanding an altitude adjustment:

$$\Delta z_{cmd} = \text{sign}(z_j - z_k) \cdot \max\!\left(r_{sep} - |z_k(t + T_{la}) - z_j(t + T_{la})|,\; \Delta z_{min}\right)$$

where $T_{la} = 15$ s is the lookahead horizon and $\Delta z_{min} = 5$ m is the minimum commanded altitude step. The resulting commanded altitude is:

$$z_j^{cmd} = z_j(t) + \Delta z_{cmd}$$

subject to the layer altitude bound $z_j^{cmd} \in [z_{lo,\ell_j},\, z_{hi,\ell_j}]$; if the bound is violated, the drone transitions to the adjacent layer.

### Unified 3D Separation Assurance

The UTM monitors the predicted 3D position at lookahead $T_{la}$:

$$\hat{\mathbf{p}}_k(t + T_{la}) = \mathbf{p}_k(t) + \mathbf{v}_k(t) \cdot T_{la}$$

A **3D resolution advisory** is issued when the predicted 3D separation falls below the warning sphere:

$$\|\hat{\mathbf{p}}_k(t + T_{la}) - \hat{\mathbf{p}}_j(t + T_{la})\| < r_{warn} = 1.5 \cdot r_{sep}$$

The resolution preference order is:

$$\text{1. Altitude offset} \rightarrow \text{2. Speed reduction} \rightarrow \text{3. Hover hold}$$

### Joint 4D LP Optimisation (Continuous Relaxation)

Let $\delta_k \geq 0$ be the departure delay and $\ell_k \in \{1,\ldots,L\}$ the altitude layer for drone $k$. The combined delay-minimisation problem with precedence from FCFS order:

$$\min_{\delta_k} \sum_{k=1}^{N} \delta_k$$

$$\text{subject to: } \delta_k - \delta_j \geq \Delta t_{sep} \quad \forall\, (k,j) \in \mathcal{C}_{3D}$$

$$\delta_k \geq 0 \quad \forall\, k$$

where the 3D conflict set $\mathcal{C}_{3D}$ is computed using the sphere-based conflict detector, and the temporal separation requirement accounts for 3D closing speed:

$$\Delta t_{sep} = \frac{r_{sep}}{v_{close,3D}} + t_{buffer}, \quad v_{close,3D} = \|\mathbf{v}_k - \mathbf{v}_j\|$$

### Corridor Density Constraint (3D Tube)

The instantaneous density in corridor tube $c$ at time $t$:

$$\rho_c(t) = \frac{\#\{k : d(\mathbf{p}_k(t),\, c) \leq r_c\}}{L_c / 1000} \quad \text{[drones/km]}$$

$$\rho_c(t) \leq \rho_{max} \quad \forall\, c,\, t$$

### Performance Metrics

Total fleet delay and throughput carry over from S035 with the 3D conflict set substituted:

$$T_{delay}^{total} = \sum_{k=1}^{N} \bigl(t_k^{actual} - t_k^{planned}\bigr)$$

$$\Theta = \frac{N}{T_{mission}} \times 60 \quad \text{[deliveries/min]}$$

**Altitude manoeuvre cost** (additional metric specific to 3D resolution):

$$C_{alt} = \sum_{k=1}^{N} \int_0^{T_{mission}} |\dot{z}_k(t)|\, dt \quad \text{[m of vertical travel]}$$

**3D conflict rate**:

$$\lambda_{3D} = \frac{|\mathcal{C}_{3D}|}{\binom{N}{2} \cdot T_{mission} / 60}$$

---

## Key 3D Additions

- **6-layer altitude structure**: two additional express/emergency layers (L5, L6) above the original four, enabling priority escalation without horizontal rerouting.
- **3D corridor tubes**: cylindrical tubes with 3D centreline polylines replace flat 2D corridor bands; membership test uses point-to-segment distance in $\mathbb{R}^3$.
- **Sphere-based 3D conflict detection**: unified $r_{sep}$ sphere replaces the decoupled horizontal/vertical threshold pair, using the 1D quadratic minimum-distance closed form on each leg pair.
- **Vertical offset RA**: UTM first attempts to resolve conflicts by commanding a signed altitude step $\Delta z_{cmd}$, preserving drone throughput before resorting to hover holds.
- **Climb/descent angle enforcement**: intent registration rejects legs with $|\gamma| > \gamma_{max} = 20°$; transition segments between layers are planned at exactly $\gamma_{max}$.
- **Altitude manoeuvre cost metric $C_{alt}$**: tracks total vertical travel per strategy to quantify the energy penalty of altitude-first vs time-first resolution.

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Airspace volume | 1000 m × 1000 m × 150 m |
| Number of drones | 12 |
| Number of altitude layers | 6 |
| Altitude layer bands | 20–40, 40–60, 60–80, 80–100, 100–120, 120–150 m |
| Corridor tube radius $r_c$ | 25 m |
| 3D separation sphere $r_{sep}$ | 35 m |
| 3D warning sphere $r_{warn}$ | 52.5 m ($1.5 \times r_{sep}$) |
| Separation assurance lookahead $T_{la}$ | 15 s |
| Maximum climb/descent angle $\gamma_{max}$ | 20° |
| Minimum altitude step $\Delta z_{min}$ | 5 m |
| Cruise speed $v_{cruise}$ | 10 m/s |
| Maximum corridor density $\rho_{max}$ | 3 drones/km |
| Temporal buffer $t_{buffer}$ | 2 s |
| LP solver | `scipy.optimize.linprog` (HiGHS) |
| Simulation timestep $\Delta t$ | 0.1 s |
| Conflict scan resolution | 0.5 s |

---

## Expected Output

- **3D airspace visualisation**: all 12 drone trajectories plotted in 3D with altitude colour-coding; corridor tube centrelines shown as translucent cylinders; 3D conflict spheres at detected violation points; altitude layer boundaries drawn as semi-transparent horizontal planes.
- **Altitude time series**: $z_k(t)$ for all drones under each strategy; altitude offset RA events marked as vertical arrows; layer boundaries drawn as dashed horizontal lines.
- **3D separation histogram**: distribution of minimum pairwise 3D distances $\|\mathbf{p}_k - \mathbf{p}_j\|_{min}$ under each resolution strategy; dashed vertical line at $r_{sep} = 35$ m.
- **Delay comparison bar chart**: total fleet delay $T_{delay}^{total}$ and mean per-drone delay $\bar{T}_{delay}$ for FCFS time-only vs altitude-first vs joint LP optimisation.
- **Altitude manoeuvre cost $C_{alt}$**: total vertical travel per strategy as a stacked bar chart alongside delay, illustrating the delay–altitude cost trade-off.
- **Corridor tube utilisation heat-map**: drone density $\rho_c(t)$ for each 3D corridor tube vs time; dashed line at $\rho_{max}$.
- **Throughput and 3D conflict rate table**: $\Theta$ and $\lambda_{3D}$ for each strategy.
- **Animation (GIF)**: rotating 3D view of all drones moving through the layered airspace; altitude layer bands shaded by colour; conflict sphere violations pulsing red; RA events shown as brief altitude-arrow flashes.

---

## Extensions

1. **Wind shear by altitude layer**: model a layered wind field where each altitude band has a distinct horizontal wind vector; drones must account for wind drift when planning 3D legs, and the UTM re-evaluates separation under worst-case wind uncertainty.
2. **Dynamic layer capacity**: allow layers L5 and L6 to be reserved for emergency drones by default; release capacity to standard flights only when utilisation in L1–L4 exceeds a threshold, modelling dynamic airspace class boundaries.
3. **Terrain-aware 3D corridors**: import a digital elevation model (DEM); corridor tube centrelines are raised above terrain to maintain a minimum obstacle clearance $h_{min} = 30$ m, producing curved 3D centrelines over hilly urban environments.
4. **Decentralised 3D UTM (U-space)**: replace the centralised UTM with peer-to-peer 3D conflict negotiation; each drone pair independently negotiates an altitude offset or time delay via a TCAS-II-inspired 3D resolution advisory protocol.
5. **RL-based 3D corridor and layer assignment**: train a PPO agent to assign altitude layer, corridor tube, and departure time to each incoming intent; state encodes full 3D airspace occupancy; reward = negative total delay; penalty = 3D separation violation or $\gamma_{max}$ exceedance.

---

## Related Scenarios

- Original 2D version: [S035 UTM Simulation](../S035_utm_simulation.md)
- 3D logistics references: [S022 3D Obstacle Avoidance Delivery](S022_3d_obstacle_avoidance_delivery.md), [S031 3D Path Deconfliction](S031_3d_path_deconfliction.md)
- Algorithm reference: LP conflict resolution relates to [S019 Dynamic Reassignment](../../01_pursuit_evasion/S019_dynamic_reassignment.md)
- Follow-up: [S036 Last-Mile Relay](../S036_last_mile_relay.md)
