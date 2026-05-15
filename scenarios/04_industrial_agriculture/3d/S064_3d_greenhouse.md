# S064 3D Upgrade — Greenhouse Interior Navigation

**Domain**: Industrial & Agriculture | **Difficulty**: ⭐⭐⭐⭐ | **Status**: `[ ]` Not started
**Based on**: [S064 original](../S064_greenhouse.md)

---

## What Changes in 3D

The original S064 locks cruise altitude at the single constant `Z_CRUISE = 1.0 m` throughout the
entire mission. The altitude controller is a trivial proportional hold (`vz = KP_ALT * (1.0 - z)`)
and the plant-row obstacles are modelled as flat 2D walls extending from the floor to
`ROW_H = 1.5 m` — meaning the drone simply flies beneath the canopy top with no altitude
planning at all. Collision avoidance operates only in the $Y$ direction; no vertical repulsion is
applied.

In the full 3D upgrade, altitude becomes a first-class control dimension:

- **Trellis wire obstacles** run horizontally along each row at fixed heights (0.6 m and 1.2 m),
  requiring the drone to plan a passage height that clears them.
- **Canopy height is non-uniform**: individual plant canopies are modelled as cylinders of random
  height $h_k \sim \mathcal{U}(1.0, 1.6)$ m, so the maximum safe flight altitude varies along the
  row length.
- **Altitude transition at row turns**: when the drone turns from one aisle to the next, it must
  climb over the row's highest trellis wire before translating laterally, then descend back to
  inspection altitude — a three-phase manoeuvre (climb, translate, descend).
- **3D collision avoidance**: the repulsive field is extended to all three axes, preventing contact
  with trellis wires above, the floor below, and plant canopies on both sides.
- **Downward-facing ToF sensor**: a time-of-flight sensor measures clearance to the tallest
  canopy beneath the drone, providing a vertical analogue of the lateral ultrasonic readings and
  enabling altitude fusion alongside the existing lateral complementary filter.

---

## Problem Definition

**Setup**: A single drone navigates a commercial greenhouse (40 × 10 × 3 m) executing a
boustrophedon inspection sweep across four aisles (between five plant rows). The greenhouse now
contains a fully 3D obstacle field:

- **Plant-row walls** at $y \in \{2, 4, 6, 8, 10\}$ m, each 0.2 m thick, extending from
  $z = 0$ to a per-row canopy height $h_k \sim \mathcal{U}(1.0, 1.6)$ m.
- **Trellis wires** at two fixed heights: $z_{t1} = 0.6$ m and $z_{t2} = 1.2$ m, running
  continuously along each row from $x = 0$ to $x = 40$ m.
- **Row-end crossing zone**: the region $0 \le x \le 2$ m and $38 \le x \le 40$ m where the
  drone transitions between aisles; trellis wires are absent here ($x < 1$ m and $x > 39$ m) to
  allow lateral row-crossing manoeuvres.

No GPS is available. The drone fuses:
1. A dead-reckoning estimate from IMU integration (cumulative drift in all three axes).
2. Lateral ultrasonic readings (left/right wall distances, as in S064).
3. A downward ToF reading (clearance to the highest canopy directly below).

**Roles**:
- **Drone**: starts at $\mathbf{p}_0 = (0.0, 1.0, 0.4)$ m facing $+X$; dimensions
  0.3 × 0.3 × 0.15 m; carries IMU, two lateral ultrasonic sensors, and one downward ToF sensor.
- **Plant rows**: five static obstacle walls at $y = 2, 4, 6, 8, 10$ m; each row has an
  independently sampled canopy height $h_k$ and two trellis wires at $z = 0.6$ m and $z = 1.2$ m.
- **Greenhouse walls and floor/ceiling**: hard boundaries at $x \in \{0, 40\}$ m,
  $y \in \{0, 10\}$ m, $z \in \{0, 3\}$ m.

**Objective**: Complete the full boustrophedon sweep within $T_{max} = 300$ s without collision
(minimum 3D clearance $d_{safe} = 0.25$ m to any obstacle surface), minimising the combined
lateral and vertical position error accumulated by the sensor-fusion estimator.

---

## Mathematical Model

### 3D Dead-Reckoning Position Update

IMU integration now accumulates drift in all three axes:

$$\mathbf{p}_{t+1} = \mathbf{p}_t + \mathbf{v}^{cmd}\,\Delta t + \boldsymbol{\eta}_t,
\qquad \boldsymbol{\eta}_t \sim \mathcal{N}\!\left(\mathbf{0},\;
\text{diag}(\sigma_x^2, \sigma_y^2, \sigma_z^2)\,\Delta t\right)$$

Horizontal drift $\sigma_x = \sigma_y = 0.02$ m/s (identical to S064). Vertical drift is lower
because barometric pressure provides a coarse altitude reference:
$\sigma_z = 0.008$ m/s.

### Trellis Wire Geometry

Each row $k$ contains two horizontal cylindrical wire obstacles at heights $z_{t1} = 0.6$ m
and $z_{t2} = 1.2$ m. A drone at position $(x, y, z)$ is in collision with a trellis wire of
row $k$ if:

$$|y - y_{row,k}| \le \frac{w_{row}}{2} + r_{drone}
\quad \text{AND} \quad |z - z_{ti}| \le r_{wire} + r_{drone}$$

where $r_{wire} = 0.01$ m (wire radius), $r_{drone} = 0.18$ m (drone body half-diagonal), and
$w_{row} = 0.1$ m is the half-thickness of the row wall. For collision avoidance purposes the
combined clearance radius is $\rho = r_{wire} + r_{drone} = 0.19$ m.

### Altitude Selection Within an Aisle

The safe inspection altitude in aisle $k$ (between rows $k$ and $k+1$) must clear both
trellis wire levels and remain below the canopy tops. Given canopy heights $h_k$ and $h_{k+1}$,
the upper bound on flight altitude is:

$$z_{max}^{aisle} = \min(h_k, h_{k+1}) - d_{safe}$$

The drone targets the midpoint between the lower trellis wire top and the canopy floor:

$$z_{cruise}^{aisle} = \frac{z_{t2} + \rho + z_{max}^{aisle}}{2}
= \frac{(1.2 + 0.19) + z_{max}^{aisle}}{2}$$

This guarantees at least $d_{safe}$ clearance above $z_{t2}$ and below the canopy.

### Row-Turn Altitude Transition (Three-Phase Manoeuvre)

When the drone approaches a row-end turning zone ($x < x_{turn} = 2$ m or $x > 38$ m), it
executes a three-phase vertical manoeuvre to cross from aisle $k$ to aisle $k+1$:

**Phase 1 — Climb**: ascend from $z_{cruise}^k$ to $z_{cross} = h_{max} + 0.3$ m where
$h_{max} = \max_k h_k = 1.6$ m, giving $z_{cross} = 1.9$ m:

$$\dot{z} = K_{alt} \cdot (z_{cross} - z), \qquad K_{alt} = 1.5 \;\text{s}^{-1}$$

**Phase 2 — Lateral translate**: once $z > z_{cross} - 0.05$ m, move laterally from
$y_k^{centre}$ to $y_{k+1}^{centre}$ at $v_{lat,turn} = 0.5$ m/s; maintain altitude hold at
$z_{cross}$.

**Phase 3 — Descend**: after reaching $y_{k+1}^{centre} \pm 0.1$ m, descend to
$z_{cruise}^{k+1}$:

$$\dot{z} = K_{alt} \cdot (z_{cruise}^{k+1} - z)$$

### 3D Sensor Fusion

The lateral complementary filter is unchanged from S064. An independent altitude channel is
added using the downward ToF sensor:

$$z_{tof} = z_{canopy,k}(x) + d_{tof} + w_z, \qquad w_z \sim \mathcal{N}(0, \sigma_{tof}^2)$$

where $z_{canopy,k}(x)$ is the canopy height of the tallest plant directly below the drone,
$d_{tof}$ is the true clearance distance, and $\sigma_{tof} = 0.03$ m. The altitude estimate:

$$\hat{z}_{t+1} = \beta\,\hat{z}_{DR} + (1 - \beta)\,\hat{z}_{tof}$$

with $\hat{z}_{DR} = \hat{z}_t + v_z \Delta t + \eta_z$ (barometric-aided dead reckoning) and
filter gain $\beta = 0.75$ (higher trust in ToF than in the lateral ultrasonic, because the
vertical channel has lower DR drift).

### 3D Repulsive Collision Avoidance

The 3D repulsive velocity field combines lateral and vertical components. For each obstacle
surface $s$ with signed clearance $d_s$ (positive = safe side):

$$v_s^{rep} = \begin{cases}
K_{rep} \cdot \dfrac{d_{safe} - d_s}{d_{safe}} \cdot \hat{\mathbf{n}}_s & d_s < d_{safe} \\
\mathbf{0} & \text{otherwise}
\end{cases}$$

where $\hat{\mathbf{n}}_s$ is the unit normal pointing away from obstacle $s$ and
$K_{rep} = 1.5$ m/s. For trellis wires the normal is $\hat{\mathbf{n}}_s = \pm\hat{\mathbf{z}}$;
for plant walls it is $\pm\hat{\mathbf{y}}$; for the floor/ceiling it is $\pm\hat{\mathbf{z}}$.
The total commanded velocity is:

$$\mathbf{v}^{total} = \mathbf{v}^{cmd} + \sum_s \mathbf{v}_s^{rep}$$

### Drift Error Metrics (3D)

The 3D position error norm between filter estimate and ground truth:

$$E_{3D}(t) = \|\hat{\mathbf{p}}_t - \mathbf{p}_t^{true}\|_2$$

Separate lateral and vertical components:

$$E_{lat}(t) = |\hat{y}_t - y_t^{true}|, \qquad E_{alt}(t) = |\hat{z}_t - z_t^{true}|$$

Cumulative mean errors over the mission:

$$\bar{E}_{lat} = \frac{1}{N_t}\sum_t E_{lat}(t), \qquad
\bar{E}_{alt} = \frac{1}{N_t}\sum_t E_{alt}(t)$$

---

## Key 3D Additions

- **Non-uniform canopy heights**: per-row $h_k \sim \mathcal{U}(1.0, 1.6)$ m; cruise altitude
  computed per-aisle to stay above trellis wires and below canopy tops.
- **Trellis wire obstacles**: cylindrical obstacles at $z = 0.6$ m and $z = 1.2$ m along each
  row; 3D repulsion applied in the $\pm Z$ direction when within $d_{safe}$.
- **Three-phase row-turn manoeuvre**: climb-translate-descend sequence at each end of the
  greenhouse; altitude during crossing is $z_{cross} = 1.9$ m, above all canopies.
- **Downward ToF altitude sensor**: fusion gain $\beta = 0.75$; $\sigma_{tof} = 0.03$ m;
  corrects barometric/IMU vertical drift alongside the lateral ultrasonic correction.
- **Full 3D repulsive field**: obstacles on all three axes; repulsion gain identical to S064
  ($K_{rep} = 1.5$ m/s) but now vectorised in 3D.

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Greenhouse dimensions | 40 × 10 × 3 m |
| Number of plant rows | 5 |
| Row Y positions | 2, 4, 6, 8, 10 m |
| Row wall thickness | 0.2 m |
| Canopy height range $h_k$ | $\mathcal{U}(1.0, 1.6)$ m |
| Trellis wire heights | 0.6 m, 1.2 m |
| Wire radius $r_{wire}$ | 0.01 m |
| Aisle cruise altitude $z_{cruise}^{aisle}$ | per-aisle, ~1.4 – 1.7 m |
| Row-crossing altitude $z_{cross}$ | 1.9 m |
| Lateral cruise speed $v_x$ | 1.0 m/s |
| Turn lateral speed $v_{lat,turn}$ | 0.5 m/s |
| IMU horizontal drift $\sigma_{x,y}$ | 0.02 m/s |
| IMU vertical drift $\sigma_z$ | 0.008 m/s |
| Lateral ultrasonic noise $\sigma_{range}$ | 0.05 m |
| Downward ToF noise $\sigma_{tof}$ | 0.03 m |
| Lateral filter gain $\alpha$ | 0.85 |
| Altitude filter gain $\beta$ | 0.75 |
| Lateral proportional gain $K_p$ | 0.8 s⁻¹ |
| Altitude controller gain $K_{alt}$ | 1.5 s⁻¹ |
| Repulsion gain $K_{rep}$ | 1.5 m/s |
| Safety clearance $d_{safe}$ | 0.25 m |
| Waypoint capture radius $d_{wp}$ | 0.5 m |
| Mission horizon $T_{max}$ | 300 s |
| Simulation timestep $\Delta t$ | 0.05 s |
| Start position $\mathbf{p}_0$ | (0.0, 1.0, 0.4) m |
| Z flight range | 0.3 – 2.8 m |

---

## Expected Output

- **3D trajectory plot** (`s064_3d_gh_trajectory.png`): isometric view of the greenhouse; grey
  slabs show plant-row walls with individually sampled canopy heights; thin horizontal lines mark
  trellis wire levels; blue polyline is the true 3D flight path, red dashed polyline is the filter
  estimate; visible altitude changes at row turns (climb-translate-descend arcs at $x = 0$ and
  $x = 40$ m); green marker at start, red cross at end.
- **Altitude time series** (`s064_3d_gh_altitude.png`): $z_{true}(t)$ vs $\hat{z}(t)$ overlaid;
  horizontal bands show trellis wire clearance zones ($z < 0.6 + \rho$ and $z < 1.2 + \rho$
  shaded red); step-up and step-down events at each row turn clearly visible; ToF correction events
  annotated.
- **Six-panel analysis figure** (`s064_3d_gh_analysis.png`):
  - **(a)** Lateral position: true $Y(t)$ vs $\hat{Y}(t)$, row wall positions marked.
  - **(b)** Altitude position: true $Z(t)$ vs $\hat{Z}(t)$, trellis wire heights marked.
  - **(c)** Lateral complementary filter components (DR, wall-derived, fused).
  - **(d)** Altitude fusion components (DR+baro, ToF-derived, fused).
  - **(e)** 3D wall / trellis clearance over time; $d_{safe}$ threshold line; no crossings.
  - **(f)** Lateral and vertical error: $E_{lat}(t)$ (blue), $E_{alt}(t)$ (green), $E_{3D}(t)$ (purple).
- **Top-down animation** (`s064_3d_gh_animation.gif`): X–Y plane; grey row rectangles; blue/red
  trails for true and estimated path; drone icon colour-coded by current altitude (cool = low,
  warm = high); 25 fps.
- **Stdout metrics**: aisles completed, mean/peak lateral error, mean/peak altitude error,
  minimum 3D obstacle clearance, per-waypoint arrival log.

**Typical expected metrics** (seed 42):
- Aisles completed: 4 / 4
- Mean lateral error: ~0.02 – 0.05 m
- Mean altitude error: ~0.01 – 0.02 m (lower drift due to ToF)
- Minimum obstacle clearance: $> d_{safe} = 0.25$ m (collision-free)

---

## Extensions

1. **Random trellis heights**: sample wire heights per row from $\mathcal{U}(0.5, 1.4)$ m with
   varying numbers of wires (1 – 3); the altitude planner must query the wire map and compute
   a collision-free cruise band dynamically rather than from fixed $z_{t1}, z_{t2}$ values.
2. **EKF altitude estimator**: replace the scalar complementary filter with a 2-state EKF
   $(z, v_z)$ using barometric pressure as the prediction model and ToF as the measurement update;
   compare altitude RMSE against the complementary filter over 100 Monte Carlo trials.
3. **Plant protrusion + vertical dodge**: extend the canopy obstacle model so individual plants
   protrude 0.1 – 0.3 m beyond the nominal row boundary into the aisle; the forward-facing distance
   sensor triggers a vertical dodge when protrusion clearance falls below 0.8 m; evaluate dodge
   success rate and altitude deviation distribution.
4. **Multi-drone 3D coordination**: two drones starting from opposite greenhouse ends; each drone
   must negotiate trellis clearance at its own row turns while maintaining a 3D separation buffer
   of 1.5 m; add a central arbiter that transmits altitude hold commands when predicted 3D
   proximity is below the buffer.
5. **Energy-optimal altitude profile**: instead of targeting the midpoint of the safe altitude
   band, formulate a 1D trajectory optimisation that minimises motor thrust energy
   $\int (v_z^2 + k_{grav}) \,dt$ subject to per-aisle clearance constraints; compare energy
   consumption against the fixed midpoint heuristic across ten random canopy configurations.

---

## Related Scenarios

- Original 2D version: [S064](../S064_greenhouse.md)
- 3D indoor dead reckoning: [S069 Automated Warehouse Inventory](../S069_warehouse_inventory.md)
- 3D trellis/structure avoidance reference: [S061 Power Line Inspection](../S061_power_line.md)
- GPS-denied localisation: [S050 Distributed EKF-SLAM](../../03_environmental_sar/S050_slam.md), [S046 Trilateration](../../03_environmental_sar/S046_trilateration.md)
- Vertical obstacle planning: [S043 Confined Space Exploration](../../03_environmental_sar/S043_confined_space.md)
