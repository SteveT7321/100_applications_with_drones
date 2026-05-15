# S066 3D Upgrade — Cooperative Crane Simulation

**Domain**: Industrial & Agriculture | **Difficulty**: ⭐⭐⭐⭐⭐ | **Status**: `[ ]` Not Started
**Based on**: [S066 original](../S066_cooperative_crane.md)

---

## What Changes in 3D

The original S066 already integrates a 6-DOF rigid-body beam model with quaternion kinematics, but
two critical simplifications hide genuine 3D complexity:

1. **Fixed-altitude transport** — the mission is a flat horizontal traverse at $z = 2$ m. The
   reference trajectory has no vertical component ($\dot{z}_{ref} = 0$ always), so altitude
   control is never stressed.
2. **Vertical-only cable geometry** — drones hover exactly $l_c = 1$ m above their anchor in the
   $\hat{z}$ direction. The cable unit vectors $\hat{n}_i$ always point straight up, making the
   grasp matrix $\mathbf{G}$ trivially decoupled in the $z$-axis and unable to produce lateral
   torques through cable geometry variation.

This variant introduces:
- A **3D waypoint trajectory** $A \to P_1 \to P_2 \to B$ that includes a hoisting phase
  (ascent), a cross-route segment with simultaneous yaw rotation of the beam, and a final
  descent — stressing all six wrench DOFs.
- **Full 3D cable geometry**: drones may offset laterally from their anchors, so $\hat{n}_i$
  carries horizontal components. The grasp matrix $\mathbf{G}$ then couples translational and
  rotational control, and the QP tension allocation must handle non-trivial wrench decomposition.
- **Static obstacle avoidance during hoisting**: two cylindrical obstacles (building columns)
  are placed in the work volume. Each drone must plan a collision-free path around them while
  remaining attached to the beam via taut cables and maintaining beam orientation.
- **3D beam orientation control**: the beam must rotate $90°$ in yaw midway through the route
  (e.g., to pass through a narrow gap), requiring coordinated tension differentials that generate
  a pure yaw torque while suppressing roll and pitch.

---

## Problem Definition

**Setup**: A rigid steel beam ($m_L = 2$ kg, $L = 2$ m) is transported by three quadrotors
along a 3D piecewise-linear path with four waypoints:

$$A = (0, 0, 1) \;\to\; P_1 = (0, 0, 4) \;\to\; P_2 = (10, 5, 4) \;\to\; B = (15, 0, 2) \quad \text{(m)}$$

The segment $A \to P_1$ is a pure vertical hoist. The segment $P_1 \to P_2$ includes lateral
displacement and a $90°$ yaw rotation of the beam about its own centre of mass. The segment
$P_2 \to B$ is a coordinated descent. Two static cylindrical obstacles occupy the workspace:

$$\mathcal{O}_1: \text{centre}\,(4, 0),\; r_{obs} = 1.0 \text{ m} \qquad
  \mathcal{O}_2: \text{centre}\,(8, 4),\; r_{obs} = 0.8 \text{ m}$$

(infinite height; safe clearance $\delta_{obs} = 0.4$ m required for every drone.)

Cables remain taut ($T_i \geq T_{min} = 0.5$ N); drones are free to offset laterally from their
anchors up to a maximum cable angle $\phi_{max} = 30°$ from vertical.

**Roles**:
- **Drones** ($N = 3$): quadrotors with first-order attitude dynamics ($\tau = 0.05$ s).
  Each drone commands a full 3D thrust vector (not just a scalar magnitude), enabling lateral
  cable forces that contribute torque to the beam.
- **Rigid beam**: 6-DOF rigid body. Its yaw angle $\psi_L$ must track a time-varying reference
  $\psi_{ref}(t)$ that ramps $90°$ over the $P_1 \to P_2$ segment.
- **Obstacle-avoidance planner**: a per-drone artificial potential field (APF) layer that adds
  repulsive corrections to the formation setpoint, subject to the cable-angle constraint.

**Objective**: Complete the full $A \to B$ mission while satisfying:

1. **Attitude constraint**: $|\theta_{pitch}|, |\theta_{roll}| \leq 5°$ at all times.
2. **Yaw tracking**: $|\psi_L - \psi_{ref}| \leq 10°$ during the rotation segment.
3. **Obstacle clearance**: each drone maintains $\geq 0.4$ m clearance from both cylinder surfaces.
4. **Cable angle**: $\phi_i \leq 30°$ from vertical for all $i$ (taut-cable assumption validity).
5. **Inter-drone separation**: $\|p_i - p_j\| \geq d_{min} = 0.8$ m.
6. **Trajectory tracking**: beam CoM position error $< 0.15$ m along each segment.

---

## Mathematical Model

### Beam Rigid Body State (unchanged from S066)

$$\mathbf{I}_L = \begin{pmatrix}
\tfrac{1}{12} m_L L^2 & 0 & 0 \\
0 & \epsilon & 0 \\
0 & 0 & \tfrac{1}{12} m_L L^2
\end{pmatrix}, \qquad I_{yy} = I_{zz} = \tfrac{2}{3} \text{ kg m}^2$$

Quaternion kinematics:

$$\dot{\mathbf{q}}_L = \frac{1}{2} \mathbf{q}_L \otimes \begin{pmatrix} 0 \\ \boldsymbol{\omega}_L \end{pmatrix}$$

### 3D Cable Geometry with Lateral Offset

In the original S066 the desired drone position is fixed at $l_c \hat{z}$ above the anchor.
In this variant the drone's desired position is:

$$\mathbf{p}_i^{des} = \mathbf{a}_i + l_c \, \hat{\mathbf{d}}_i^{des}$$

where $\hat{\mathbf{d}}_i^{des}$ is a commanded cable direction (unit vector), not necessarily
vertical. The cable direction is constrained to lie within a cone of half-angle $\phi_{max}$
around $\hat{z}$:

$$\hat{\mathbf{d}}_i^{des} = \cos\phi_i \, \hat{z} + \sin\phi_i \, \hat{\mathbf{u}}_i$$

with $\phi_i \in [0, \phi_{max}]$ and $\hat{\mathbf{u}}_i$ a unit vector in the horizontal plane.
The cable unit vector pointing from anchor to drone is:

$$\hat{n}_i = \frac{\mathbf{p}_i - \mathbf{a}_i}{\|\mathbf{p}_i - \mathbf{a}_i\|}$$

The tension force on the beam at anchor $i$ is $\mathbf{f}_i = T_i \hat{n}_i$ (unchanged), but
now $\hat{n}_i$ carries horizontal components that contribute lateral torques to the beam.

### 3D Grasp Matrix and Wrench Decomposition

The 6-DOF grasp matrix with 3D cable directions:

$$\mathbf{G} = \begin{pmatrix} \hat{n}_1 & \hat{n}_2 & \hat{n}_3 \\
\mathbf{r}_1 \times \hat{n}_1 & \mathbf{r}_2 \times \hat{n}_2 & \mathbf{r}_3 \times \hat{n}_3
\end{pmatrix} \in \mathbb{R}^{6 \times 3}$$

where $\mathbf{r}_i = \mathbf{a}_i - \mathbf{p}_L$ is the moment arm from beam CoM to anchor $i$
in world frame. With 3D cable directions the torque columns $\mathbf{r}_i \times \hat{n}_i$
become non-zero in all three components (including yaw about $\hat{z}$), enabling yaw control
through differential cable tilting.

The desired wrench is extended to include yaw:

$$\mathbf{W}_{des} = \begin{pmatrix}
m_L \ddot{\mathbf{p}}_L^{des} - m_L \mathbf{g} \\
\mathbf{I}_L \dot{\boldsymbol{\omega}}_L^{des} + \boldsymbol{\omega}_L \times (\mathbf{I}_L \boldsymbol{\omega}_L)
\end{pmatrix} \in \mathbb{R}^6$$

Tension allocation remains a QP:

$$\min_{\mathbf{T}} \quad \mathbf{T}^\top \mathbf{T}
\qquad \text{s.t.} \quad \mathbf{G} \mathbf{T} = \mathbf{W}_{des}, \;\; T_i \geq T_{min}$$

with the pseudo-inverse solution:

$$\mathbf{T}^* = \mathbf{G}^+ \mathbf{W}_{des} + (\mathbf{I} - \mathbf{G}^+ \mathbf{G}) \boldsymbol{\lambda}$$

### Yaw Reference Trajectory

The beam yaw reference $\psi_{ref}(t)$ is a smooth step from $0°$ to $90°$ over the
$P_1 \to P_2$ segment using a raised-cosine profile:

$$\psi_{ref}(s) = \frac{\pi}{4} \left(1 - \cos\!\left(\pi \frac{s - s_1}{s_2 - s_1}\right)\right), \quad s \in [s_1, s_2]$$

where $s$ is arc length along the path and $s_1, s_2$ are the arc-length coordinates of
$P_1$ and $P_2$ respectively. The yaw rate command is:

$$\dot{\psi}_{ref}(s) = \frac{\pi^2}{2(s_2 - s_1)} \sin\!\left(\pi \frac{s - s_1}{s_2 - s_1}\right) \cdot \|\dot{\mathbf{p}}_L^{ref}\|$$

The full yaw PD command feeds into the beam angular velocity error term:

$$\dot{\omega}_{z,des} = K_{p,\psi} (\psi_{ref} - \psi_L) + K_{d,\psi} (\dot{\psi}_{ref} - \omega_{L,z})$$

### Altitude Dynamics and Hoisting Control

During the vertical hoist segment ($A \to P_1$) the desired beam acceleration is:

$$\ddot{z}_L^{des} = K_{p,z} (z_{ref} - z_L) + K_{d,z} (\dot{z}_{ref} - \dot{z}_L)$$

All three cables must collectively overcome gravity and provide upward acceleration:

$$\sum_{i=1}^3 T_i (\hat{n}_i \cdot \hat{z}) = m_L (\ddot{z}_L^{des} + g)$$

During hoisting, the cable directions are kept nearly vertical ($\phi_i \approx 0$) to maximise
the vertical force component. The yaw-torque demand is zero during this phase, so the QP
allocates tensions symmetrically.

### Obstacle Avoidance via Artificial Potential Fields

For cylindrical obstacle $k$ with centre $(x_k, y_k)$ and radius $r_k$, the 2D distance from
drone $i$ (projected onto the horizontal plane) is:

$$d_{ik} = \sqrt{(p_{i,x} - x_k)^2 + (p_{i,y} - y_k)^2} - r_k$$

The APF repulsion added to drone $i$'s desired velocity in the horizontal plane:

$$\mathbf{v}_{rep,ik} = \begin{cases}
k_{obs} \left(\dfrac{1}{d_{ik}} - \dfrac{1}{d_{inf}}\right) \dfrac{1}{d_{ik}^2}
\cdot \hat{\mathbf{e}}_{ik} & \text{if } d_{ik} < d_{inf} \\
\mathbf{0} & \text{otherwise}
\end{cases}$$

where $\hat{\mathbf{e}}_{ik}$ is the unit horizontal vector from the obstacle centre to the
drone, $d_{inf} = 2.5$ m is the influence radius, and $k_{obs}$ is the repulsion gain. The
cable-angle constraint $\phi_i \leq \phi_{max}$ clips the resulting lateral deviation:

$$\|\mathbf{p}_i^{des} - \mathbf{a}_i\|_{xy} \leq l_c \tan\phi_{max}$$

### 3D Inter-Drone Collision Avoidance

Soft repulsion is extended to full 3D (not only horizontal):

$$\mathbf{v}_{rep,ij} = \begin{cases}
k_{rep} \dfrac{d_{safe} - \|\mathbf{p}_i - \mathbf{p}_j\|}{d_{safe}}
\cdot \dfrac{\mathbf{p}_i - \mathbf{p}_j}{\|\mathbf{p}_i - \mathbf{p}_j\|} & \text{if } \|\mathbf{p}_i - \mathbf{p}_j\| < d_{safe} \\
\mathbf{0} & \text{otherwise}
\end{cases}$$

### Performance Metrics

**Attitude RMS** (over full mission):

$$\epsilon_{att} = \sqrt{\frac{1}{T_{total}} \int_0^{T_{total}} \left(\theta_{pitch}^2 + \theta_{roll}^2\right) dt}$$

**Yaw tracking error RMS** (over rotation segment only):

$$\epsilon_{yaw} = \sqrt{\frac{1}{t_2 - t_1} \int_{t_1}^{t_2} (\psi_L - \psi_{ref})^2 \, dt}$$

**Cable angle utilisation** (mean across drones and time):

$$\bar{\phi} = \frac{1}{N \, T_{total}} \sum_{i=1}^N \int_0^{T_{total}} \phi_i(t) \, dt$$

**Obstacle clearance margin** (minimum over mission):

$$\delta_{min} = \min_{i,k,t} \; d_{ik}(t)$$

---

## Key 3D Additions

- **3D waypoint trajectory with hoisting and descent**: altitude varies from 1 m to 4 m and back
  to 2 m, stressing vertical wrench allocation.
- **Lateral cable tilting for yaw control**: drones offset horizontally from their anchors to
  generate yaw torque via the 3D grasp matrix; cable angle capped at $\phi_{max} = 30°$.
- **Beam yaw rotation maneuver**: raised-cosine yaw profile rotates the beam $90°$ in the
  $P_1 \to P_2$ segment; yaw PD controller feeds into the 6-DOF wrench demand.
- **Static cylinder obstacle avoidance**: per-drone APF repulsion in the horizontal plane, clipped
  by the cable-angle feasibility cone.
- **Full 6-DOF QP tension allocation**: 3D cable geometry makes all six wrench components
  accessible (subject to the 3-cable rank limit); pseudo-inverse with null-space correction
  enforces $T_{min}$.

---

## Key Parameters

| Parameter | Symbol | Value |
|-----------|--------|-------|
| Number of drones | $N$ | 3 |
| Beam mass | $m_L$ | 2 kg |
| Beam length | $L$ | 2 m |
| Beam inertia (pitch/yaw) | $I_{yy} = I_{zz}$ | 2/3 kg m² |
| Drone mass | $m_D$ | 0.5 kg |
| Cable length | $l_c$ | 1 m |
| Max cable angle | $\phi_{max}$ | 30° |
| Waypoints | $A, P_1, P_2, B$ | (0,0,1), (0,0,4), (10,5,4), (15,0,2) m |
| Cruise speed (horizontal) | $v_{xy}$ | 0.5 m/s |
| Hoist speed (vertical) | $v_z$ | 0.3 m/s |
| Yaw rotation total | $\Delta\psi$ | 90° |
| Simulation timestep | $\Delta t$ | 0.01 s |
| Minimum cable tension | $T_{min}$ | 0.5 N |
| Altitude range | $z$ | 1.0 – 4.0 m |
| Beam attitude constraint | $\|\theta_{pitch}\|, \|\theta_{roll}\|$ | ≤ 5° |
| Yaw tracking constraint | $\|\psi_L - \psi_{ref}\|$ | ≤ 10° |
| Hard collision threshold | $d_{min}$ | 0.8 m |
| Soft repulsion onset | $d_{safe}$ | 1.2 m |
| Obstacle 1 centre / radius | $(x_1, y_1), r_1$ | (4, 0) m, 1.0 m |
| Obstacle 2 centre / radius | $(x_2, y_2), r_2$ | (8, 4) m, 0.8 m |
| Obstacle influence radius | $d_{inf}$ | 2.5 m |
| Obstacle clearance margin | $\delta_{obs}$ | 0.4 m |
| Beam trajectory PD gains | $K_{p,xyz}, K_{d,xyz}$ | 4.0, 3.0 |
| Yaw PD gains | $K_{p,\psi}, K_{d,\psi}$ | 6.0, 3.0 |
| Attitude PD gains | $K_{p,\omega}, K_{d,\omega}$ | 8.0, 4.0 |
| Drone formation PD gains | $K_{p,D}, K_{d,D}$ | 12.0, 5.0 |
| APF repulsion gain | $k_{obs}$ | 1.5 |

---

## Expected Output

- **3D waypoint trajectory plot**: world-frame 3D axes showing the piecewise path
  $A \to P_1 \to P_2 \to B$ as a dashed reference; beam CoM trajectory as a solid black line;
  three drone trajectories in distinct colours; cylindrical obstacles rendered as semi-transparent
  grey tubes; start and goal markers; cable lines drawn at five key instants.
- **Altitude and yaw time series**: two-panel subplot showing $z_L(t)$ (beam altitude) vs
  reference, and $\psi_L(t)$ vs $\psi_{ref}(t)$ with the $\pm 10°$ tolerance band shaded.
- **Beam attitude panel**: pitch and roll over the full mission with $\pm 5°$ constraint lines;
  shaded regions indicating the hoisting, lateral translation, and descent phases.
- **Cable angle time series**: per-drone $\phi_i(t)$ with the $30°$ limit marked; demonstrates
  lateral tilting during the yaw rotation maneuver.
- **Obstacle clearance plot**: $d_{ik}(t)$ for each drone-obstacle pair; horizontal red line at
  $\delta_{obs} = 0.4$ m; confirms no constraint violation.
- **Tension allocation panel**: $T_i(t)$ for each drone across all mission phases; tension spikes
  visible during hoisting and during yaw rotation when the QP demands asymmetric loads.
- **3D transport animation (GIF)**: 3D view at 20 fps; beam drawn as a thick black rod that
  rotates visibly during the $P_1 \to P_2$ segment; drones as coloured spheres with dashed
  cables; obstacles as grey cylinders; beam CoM trail as a dotted black line; altitude indicator
  in the figure title.

---

## Extensions

1. **N = 4 drones with asymmetric anchor placement**: add a fourth drone at a non-symmetric
   offset to make the grasp matrix overdetermined; reformulate the QP with inequality bounds
   on $T_{max}$ and compare tension distribution fairness against the 3-drone case.
2. **Dynamic obstacles (moving crane arm)**: model a second crane arm sweeping through the
   workspace at constant angular velocity; extend the APF to time-varying obstacle positions
   and evaluate the minimum-time replanning latency.
3. **Cable elasticity model**: replace rigid-cable assumption with a spring-damper cable
   ($k_c = 500$ N/m, $b_c = 5$ Ns/m); analyse oscillatory modes excited during the hoist
   phase and design a damping feedforward based on the cable natural frequency
   $\omega_n = \sqrt{k_c / m_L}$.
4. **Narrow-gap yaw passage**: reduce the gap width to $L + 0.2$ m and require the beam to
   rotate exactly $90°$ while passing through; add a kinematic constraint on beam endpoint
   positions relative to the gap walls and solve for the feasible tension trajectory via
   trajectory optimisation.
5. **Wind gust rejection during hoisting**: inject a horizontal Dryden-turbulence wind field
   during the $A \to P_1$ segment; compare pure-feedback QP allocation against a
   disturbance-observer feedforward that estimates wind force from IMU acceleration residuals.
6. **On-line load identification in 3D**: treat beam mass and inertia tensor as unknown;
   estimate them from the six-component wrench residual using a recursive least-squares
   identifier; evaluate how quickly the estimator converges and its effect on yaw-tracking
   performance.

---

## Related Scenarios

- Original 2D/flat version: [S066](../S066_cooperative_crane.md)
- 3D rigid-body transport references: [S001](../../01_pursuit_evasion/S001_basic_intercept.md), [S003](../../01_pursuit_evasion/S003_low_altitude_tracking.md)
- Multi-drone formation control: [S005 Formation Keeping](../../01_pursuit_evasion/S005_formation_keeping.md)
- Cooperative interception (distributed control): [S020 Cooperative Interception](../../01_pursuit_evasion/S020_cooperative_interception.md)
- Aerial assembly follow-up: [S068 Aerial Assembly Line](../S068_aerial_assembly.md)
- Swarm construction follow-up: [S070 Swarm Construction](../S070_swarm_construction.md)
- Tethered operations (cable dynamics): [S079 Tethered Drone Operations](../S079_offshore_wind.md)
