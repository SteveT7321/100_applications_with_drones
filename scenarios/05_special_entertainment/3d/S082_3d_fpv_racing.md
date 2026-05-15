# S082 3D Upgrade — FPV Racing

**Domain**: Special Ops & Entertainment | **Difficulty**: ⭐⭐⭐ | **Status**: `[ ]` Not Started
**Based on**: [S082 original](../S082_fpv_racing.md)

---

## What Changes in 3D

The original S082 already operates in three spatial dimensions for the drone's translational
position, but it treats the drone as a 3D **point mass** with no attitude state: roll, pitch, and
yaw are absent, and the control input is a Cartesian thrust-acceleration vector with no rotor
model. The gate course uses random-altitude waypoints, yet all gates share a similar,
roughly-horizontal orientation and there are no purely vertical gates, split-level chicanes, or
dive/climb sequences that demand deliberate altitude management.

This 3D upgrade introduces three principal additions:

1. **Full 6-DOF rigid-body dynamics** — replaces the point-mass integrator with a quadrotor model
   driven by collective thrust $T$ and body torques $(\tau_\phi, \tau_\theta, \tau_\psi)$; attitude
   is tracked in ZYX Euler angles $(\phi, \theta, \psi)$ and feeds back into the thrust direction.
2. **Structured 3D gate course** — the 8-gate layout is replaced by a 12-gate circuit that
   deliberately mixes horizontal gates, vertical gates (normal vector in the XY plane), and
   diagonal dive-through gates; minimum lap time now requires planned altitude transitions between
   gate types.
3. **Differential-flatness feedforward** — the minimum-snap reference trajectory is now
   differentiated to a fourth-order flat output $(\mathbf{p}, \psi)$ from which the full reference
   attitude and rotor speeds are recovered analytically, removing the approximate MPC gain tuning
   present in the original.

---

## Problem Definition

**Setup**: A racing quadrotor must complete a circuit of $N_{gates} = 12$ gates in a
$30 \times 30 \times 12$ m arena. The gate set contains three structural types:

- **Horizontal gates** (4 gates): gate plane is horizontal ($\hat{n}_k \approx \hat{z}$); the
  drone must level out to pass through, punishing excessive bank angle at entry.
- **Vertical gates** (4 gates): gate plane is vertical ($\hat{n}_k \cdot \hat{z} \approx 0$);
  approached at full speed in the horizontal plane with altitude held steady.
- **Diagonal dive gates** (4 gates): gate normal has a $z$-component between $-0.6$ and $-0.9$;
  the drone must commit to a steep dive or climb to satisfy the approach angle constraint.

Each gate has centre $\mathbf{g}_k \in \mathbb{R}^3$, normal $\hat{n}_k$, and radius
$r_{gate} = 0.6$ m; passage tolerance $r_{tol} = 0.25$ m measured in the gate plane.

**Roles**: single drone, two strategies compared:

- **Flat-output time-optimal**: minimum-snap polynomial differentiated via differential flatness to
  feed forward full attitude; TOPP time-scaling applied; attitude tracking PD inner loop.
- **Point-mass MPC (S082 baseline)**: carried over from the original for direct comparison.

**Objective**: minimise lap time $T_{lap}$ while passing all 12 gates within tolerance and
respecting motor thrust limits $T_{min} \leq T \leq T_{max}$ and tilt angle $|\phi|, |\theta| \leq
\phi_{max}$.

---

## Mathematical Model

### 6-DOF Quadrotor Dynamics

**Translational dynamics** (world frame):

$$\dot{\mathbf{p}} = \mathbf{v}$$

$$m\dot{\mathbf{v}} = T \mathbf{R}(\phi,\theta,\psi)\hat{z} - m g \hat{z} - c_D \mathbf{v}$$

where $\mathbf{R}(\phi,\theta,\psi) \in SO(3)$ is the ZYX rotation matrix, $T$ is the collective
thrust, and $\hat{z} = (0, 0, 1)^\top$ is the body thrust axis.

**Rotational dynamics** (ZYX Euler, small-angle approximation acceptable for
$|\phi|, |\theta| \leq 40°$):

$$I_{xx}\dot{p}_b = \tau_\phi - (I_{zz} - I_{yy}) q_b r_b$$

$$I_{yy}\dot{q}_b = \tau_\theta - (I_{xx} - I_{zz}) p_b r_b$$

$$I_{zz}\dot{r}_b = \tau_\psi - (I_{yy} - I_{xx}) p_b q_b$$

$$\dot{\phi} = p_b + (q_b \sin\phi + r_b \cos\phi)\tan\theta$$

$$\dot{\theta} = q_b \cos\phi - r_b \sin\phi$$

$$\dot{\psi} = (q_b \sin\phi + r_b \cos\phi)/\cos\theta$$

State vector: $\mathbf{x} = (\mathbf{p}, \mathbf{v}, \phi, \theta, \psi, p_b, q_b, r_b)
\in \mathbb{R}^{12}$.

### Differential Flatness Feedforward

A quadrotor is differentially flat in outputs $\sigma = (\mathbf{p}, \psi)$. Given the
minimum-snap position trajectory $\mathbf{p}^{ref}(t)$ and its time derivatives up to fourth order,
the reference thrust direction (body $z$-axis in world frame) is recovered as:

$$\hat{z}_B^{ref} = \frac{\mathbf{a}^{ref} - g\hat{z}}{\|\mathbf{a}^{ref} - g\hat{z}\|}$$

where $\mathbf{a}^{ref} = \ddot{\mathbf{p}}^{ref}$ is the reference acceleration including drag
feedforward: $\mathbf{a}^{ref}_{ff} = \ddot{\mathbf{p}}^{ref} + (c_D/m)\dot{\mathbf{p}}^{ref}$.

Reference collective thrust magnitude:

$$T^{ref} = m \| \mathbf{a}^{ref}_{ff} - g\hat{z} \|$$

Reference roll and pitch angles from the desired body $z$-axis (given reference yaw $\psi^{ref}$):

$$\hat{x}_C = (\cos\psi^{ref},\; \sin\psi^{ref},\; 0)^\top$$

$$\hat{y}_B^{ref} = \frac{\hat{z}_B^{ref} \times \hat{x}_C}{\|\hat{z}_B^{ref} \times \hat{x}_C\|}$$

$$\hat{x}_B^{ref} = \hat{y}_B^{ref} \times \hat{z}_B^{ref}$$

$$\mathbf{R}^{ref} = [\hat{x}_B^{ref}\;|\;\hat{y}_B^{ref}\;|\;\hat{z}_B^{ref}]$$

The reference body rates $\boldsymbol{\omega}^{ref}$ are recovered from the third derivative of
$\mathbf{p}^{ref}$ (jerk), and the reference angular accelerations from the fourth derivative
(snap):

$$\boldsymbol{\omega}^{ref} = \mathbf{R}^{ref\top}\dot{\mathbf{R}}^{ref}$$

### Attitude Inner Loop (PD)

The attitude tracking error in $SO(3)$:

$$\mathbf{e}_R = \frac{1}{2}\left(\mathbf{R}^{ref\top}\mathbf{R} - \mathbf{R}^\top\mathbf{R}^{ref}\right)^\vee$$

Body-rate error:

$$\mathbf{e}_\omega = \boldsymbol{\omega} - \mathbf{R}^\top\mathbf{R}^{ref}\boldsymbol{\omega}^{ref}$$

Control torques:

$$\boldsymbol{\tau} = -K_R \mathbf{e}_R - K_\omega \mathbf{e}_\omega
  + \boldsymbol{\omega} \times \mathbf{J}\boldsymbol{\omega}$$

where $\mathbf{J} = \mathrm{diag}(I_{xx}, I_{yy}, I_{zz})$, and $K_R$, $K_\omega$ are diagonal
gain matrices tuned for critical damping.

### 3D Gate Passage with Approach-Angle Constraint

Gate $k$ is considered **passed** when the drone's projection onto the gate plane satisfies:

$$\|\mathbf{p}(t_k) - \mathbf{g}_k - [(\mathbf{p}(t_k) - \mathbf{g}_k)\cdot\hat{n}_k]\hat{n}_k\|
  \leq r_{tol}$$

and the approach angle constraint:

$$\hat{v}(t_k) \cdot \hat{n}_k \geq \cos\theta_{max}, \qquad \theta_{max} = 45°$$

where $\hat{v}(t_k) = \mathbf{v}(t_k)/\|\mathbf{v}(t_k)\|$ is the unit velocity direction at
crossing time. Diagonal dive gates enforce a tighter $\theta_{max} = 30°$, penalising any
trajectory that glances through obliquely.

### Minimum-Snap Spline (extended from S082)

Identical to S082 but with $N_{gates} = 12$ segments and a gate-normal-aligned approach vector
constraint added at each waypoint: the polynomial velocity $\dot{\mathbf{p}}^{ref}(t_k)$ is
constrained to satisfy:

$$\frac{\dot{\mathbf{p}}^{ref}(t_k)}{\|\dot{\mathbf{p}}^{ref}(t_k)\|} \cdot \hat{n}_k \geq
  \cos\theta_{max}$$

This is enforced as a soft penalty term added to the minimum-snap cost:

$$J_{total} = J_{snap} + w_{dir} \sum_{k=1}^{N_{gates}}
  \max\!\left(0,\; \cos\theta_{max} - \frac{\dot{\mathbf{p}}^{ref}(t_k)}{\|\dot{\mathbf{p}}^{ref}(t_k)\|}
  \cdot \hat{n}_k\right)^2$$

### TOPP with Attitude Rate Constraint

The time-optimal path parameterisation (TOPP) from S082 is augmented with a rotor-speed-derived
tilt constraint. The maximum feasible body $z$-axis tilt from vertical is $\phi_{max} = 40°$,
which limits the horizontal acceleration component:

$$\|\mathbf{a}_{xy}(t)\| \leq g \tan\phi_{max}$$

This adds a third constraint family to the TOPP phase-plane speed profile:

$$\dot{s}_{tilt}(s) = \sqrt{\frac{g\tan\phi_{max}}{\|\mathbf{a}_{xy}^{(s)}(s)\| + \varepsilon}}$$

where $\mathbf{a}_{xy}^{(s)}$ is the lateral component of $d^2\mathbf{p}/ds^2$. The updated
optimal speed profile:

$$\dot{s}^*(s) = \min\bigl(\dot{s}_{fwd}(s),\; \dot{s}_{bwd}(s),\; \dot{s}_{vel}(s),\;
  \dot{s}_{tilt}(s)\bigr)$$

### Performance Metrics

**Tilt utilisation** $\eta_\phi$: fraction of lap time with tilt $> 0.8\phi_{max}$:

$$\eta_\phi = \frac{1}{T_{lap}}\int_0^{T_{lap}}
  \mathbf{1}\!\left[\max(|\phi(t)|, |\theta(t)|) > 0.8\,\phi_{max}\right]dt$$

**Approach angle compliance** $\eta_{ang}$: fraction of gate passages where the approach angle
constraint is satisfied.

**Altitude range** $\Delta z = \max_t z(t) - \min_t z(t)$: measures effective exploitation of the
vertical axis (flat trajectories score low).

---

## Key 3D Additions

- **Attitude dynamics**: full 12-state rigid-body model replaces 6-state point mass; thrust direction
  is a function of current roll and pitch, not a free 3D vector.
- **Differential-flatness feedforward**: reference roll, pitch, and body rates computed analytically
  from minimum-snap position derivatives; eliminates approximate gain scheduling.
- **Tilt constraint in TOPP**: lateral acceleration at high speed is bounded by $g\tan\phi_{max}$,
  tightening the time-optimal speed profile through horizontal turns.
- **Gate approach-angle enforcement**: diagonal dive gates require the velocity vector to align with
  $\hat{n}_k$ within 30°; the minimum-snap spline adds a soft directional penalty at each waypoint.
- **3D gate type variety**: horizontal, vertical, and diagonal gates enforce qualitatively different
  flight phases — level passes, banked turns, and dive/climb sequences — producing measurable
  altitude excursions $\Delta z > 6$ m for the optimal strategy.

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Number of gates | 12 (4 horizontal + 4 vertical + 4 diagonal) |
| Gate radius | 0.6 m |
| Gate passage tolerance | 0.25 m |
| Approach angle limit (vertical/horizontal gates) | 45° |
| Approach angle limit (diagonal gates) | 30° |
| Maximum speed | 12 m/s |
| Maximum acceleration | 18 m/s² |
| Maximum tilt angle $\phi_{max}$ | 40° |
| Drone mass | 0.7 kg |
| Inertia $(I_{xx}, I_{yy}, I_{zz})$ | $(4.9, 4.9, 8.8) \times 10^{-3}$ kg m² |
| Linear drag coefficient | 0.06 kg/s |
| Attitude PD gains $K_R$ | $\mathrm{diag}(8.0, 8.0, 3.0)$ |
| Attitude PD gains $K_\omega$ | $\mathrm{diag}(2.5, 2.5, 1.5)$ |
| Polynomial degree per segment | 7 |
| Simulation timestep | 0.01 s |
| TOPP arc grid points | 800 |
| Initial reference speed (segment durations) | 5 m/s |
| Arena dimensions | 30 × 30 × 12 m |
| z range | 1.0 – 11.0 m |

---

## Expected Output

- **3D circuit plot** (`s082_3d_circuit.png`): full arena 3D view with 12 gate circles coloured by
  type (green = horizontal, orange = vertical, red = diagonal dive); two overlaid trajectories —
  blue for point-mass MPC baseline and red for flat-output time-optimal; altitude excursions clearly
  visible for the time-optimal path through dive gates.
- **Altitude time series** (`s082_altitude.png`): $z(t)$ for both strategies; time-optimal curve
  shows dips below 3 m at dive-gate entries and peaks above 9 m at climb exits; MPC baseline stays
  closer to mid-altitude throughout.
- **Attitude time series** (`s082_attitude.png`): roll $\phi(t)$ and pitch $\theta(t)$; time-optimal
  strategy shows tilt saturation ($\phi \approx 40°$) through banked turns and negative pitch
  ($\theta < -25°$) during dive-gate descents.
- **Speed and control panel** (`s082_metrics_panel.png`): speed profile, control thrust magnitude,
  and per-gate approach-angle compliance bar chart; diagonal gates show the largest approach-angle
  differences between strategies.
- **Race animation** (`s082_animation.gif`): 3D view at 25 fps showing drone body frame (three
  coloured axes indicating roll/pitch/yaw attitude); gate circles colour-coded by type; red trail;
  current time, speed, and maximum tilt angle displayed in title.
- **Terminal summary**: lap time, gates passed, approach-angle compliance fraction $\eta_{ang}$,
  altitude range $\Delta z$, and tilt utilisation $\eta_\phi$ for both strategies.

**Typical expected results** (illustrative):

| Metric | Point-Mass MPC (S082) | Flat-Output Time-Optimal |
|--------|----------------------|--------------------------|
| Lap time $T_{lap}$ | ~16–20 s | ~11–14 s |
| Gates passed / 12 | 11–12 / 12 | 10–12 / 12 |
| Approach-angle compliance $\eta_{ang}$ | 60–75% | 85–95% |
| Altitude range $\Delta z$ | 3–5 m | 7–10 m |
| Tilt utilisation $\eta_\phi$ | 10–20% | 40–60% |

---

## Extensions

1. **Full rotor dynamics**: model each of the four rotors with first-order speed dynamics
   $\dot{\Omega}_i = (\Omega_i^{cmd} - \Omega_i) / \tau_{motor}$; derive thrust and torque from
   $T_i = k_T \Omega_i^2$ and $Q_i = k_Q \Omega_i^2$; add rotor speed saturation
   $\Omega_{min} \leq \Omega_i \leq \Omega_{max}$.
2. **Yaw-free minimum-snap**: allow yaw to be a free output optimised jointly with position; compare
   lap times against the fixed-heading ($\psi = const$) baseline to quantify the heading-alignment
   benefit through vertical and diagonal gates.
3. **Reinforcement learning attitude control**: replace the PD inner loop with a PPO agent trained
   on randomised gate layouts; the flat-output feedforward provides the reference signal (RL
   corrects residuals); evaluate sim-to-real transfer on a physical gate.
4. **Wind turbulence in 3D**: inject Dryden turbulence with separate horizontal and vertical
   spectral shapes; vertical gusts are especially disruptive to dive-gate entries — measure how
   gust intensity degrades approach-angle compliance for diagonal gates.
5. **Gate sequencing with mixed types**: formulate the gate visiting order as a TSP variant where
   transition cost includes the attitude adjustment time between consecutive gate types (horizontal
   to diagonal is expensive); solve with nearest-neighbour + 2-opt and compare against random order.
6. **Aerobatic manoeuvre injection**: insert mandatory loop or roll sequences between selected gates
   (a typical FPV freestyle element); replan the minimum-snap spline to include a $360°$ roll or a
   $270°$ split-S; measure the lap time penalty of each manoeuvre type.

---

## Related Scenarios

- Original 2D/point-mass version: [S082](../S082_fpv_racing.md)
- 3D pursuit reference: [S001 Basic Intercept](../../01_pursuit_evasion/S001_basic_intercept.md),
  [S003 Low Altitude Tracking](../../01_pursuit_evasion/S003_low_altitude_tracking.md)
- Other 3D upgrades in domain: [S002 3D Evasive Maneuver](../../01_pursuit_evasion/3d/S002_3d_evasive_maneuver.md)
- Related racing: [S090 Racing Optimal](../S090_racing_optimal.md),
  [S096 Relay Race](../S096_relay_race.md)
- MPC / trajectory reference: [S066 Cooperative Crane](../../04_industrial_agriculture/S066_cooperative_crane.md)
