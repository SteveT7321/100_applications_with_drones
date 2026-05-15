# S025 3D Upgrade — Payload CoG Offset

**Domain**: Logistics & Delivery | **Difficulty**: ⭐⭐⭐ | **Status**: `[ ]` Not Started
**Based on**: [S025 original](../S025_payload_cog_offset.md)

---

## What Changes in 3D

The original S025 constrains the CoG offset to the horizontal body plane:
`COG_DX = +0.12 m, COG_DY = +0.08 m, COG_DZ = 0`. This produces disturbance
torques in roll and pitch only, and the attitude controller is a planar PID
that treats roll and pitch as two independent single-axis loops. Yaw is left
uncontrolled (set to zero reference throughout), and the drone flies at a fixed
altitude of `z = 2.0 m` on the horizontal legs before a straight descent at the
final waypoint.

In full 3D the offset becomes a 3-vector
$\Delta\mathbf{r} = [\Delta r_x,\, \Delta r_y,\, \Delta r_z]^T$, where
$\Delta r_z \ne 0$ occurs whenever the payload is suspended below or above the
motor plane (e.g. a dense battery slung under the belly, or a tall mast mounted
above the frame). A non-zero $\Delta r_z$ couples into all three rotational
axes simultaneously: it shifts the effective thrust line away from the body
z-axis, producing thrust-induced roll, pitch, and — once the drone tilts — a
yaw moment that grows with tilt angle. The 2D PID cannot decouple these cross
terms and diverges for any meaningful $\Delta r_z$.

This upgrade adds:

- **Full 3D CoG offset vector** $\Delta\mathbf{r} \in \mathbb{R}^3$ with a
  non-zero vertical component $\Delta r_z$
- **6-DOF attitude disturbance**: disturbance torque tensor derived from the
  full cross-product $\boldsymbol{\tau}_{dist} = \Delta\mathbf{r} \times
  (m_{payload}\,\mathbf{g})$, coupling roll, pitch, and yaw
- **3D attitude error and compensation**: the PID works on the full rotation
  error using a rotation matrix formulation, not scalar Euler differences
- **Altitude-variable waypoint profile**: the drone climbs and descends between
  waypoints at different altitudes, so attitude disturbances act in a changing
  gravity-projection frame throughout the mission
- **Yaw drift analysis**: quantify yaw error accumulation (absent in S025) and
  show that a yaw-integrator term is required when $\Delta r_z \ne 0$

---

## Problem Definition

**Setup**: A single quadrotor of total mass $m = 2.30$ kg ($m_{airframe} = 1.50$ kg,
$m_{payload} = 0.80$ kg) carries a payload whose CoG is displaced from the drone
geometric centre by the full 3D vector
$\Delta\mathbf{r} = (+0.12,\, +0.08,\, -0.06)$ m — i.e. the payload is also
hung 6 cm below the motor plane, as happens with an under-slung camera or
battery pod. The mission profile includes altitude changes so the drone
traverses a 3D waypoint sequence at mixed heights.

Three attitude controllers are compared in identical simulation runs:

- **No compensation**: standard PD on roll and pitch only; yaw reference fixed
  at 0; no knowledge of $\Delta\mathbf{r}$
- **Feedforward (static trim)**: pre-computed constant offset added to all three
  attitude references to null the steady-state disturbance torque; no online
  adaptation
- **Full 3D PID**: closed-loop attitude controller with independent integrators
  on roll, pitch, and yaw; anti-windup on all three axes

**Roles**:
- **Drone**: single quadrotor executing the mission; attitude disturbance is
  injected every timestep as an external torque vector $\boldsymbol{\tau}_{dist}$

**Objective**: Reach all 5 waypoints with position error $< 0.15$ m, maintaining
attitude error $\|\boldsymbol{\eta}_{err}\| < 3°$ (full 3D PID only), while
quantifying how much the vertical CoG component $\Delta r_z$ amplifies yaw
coupling compared to the 2D (horizontal-offset-only) baseline.

---

## Mathematical Model

### 3D CoG Offset Disturbance Torque

Let $\Delta\mathbf{r} = [\Delta r_x,\, \Delta r_y,\, \Delta r_z]^T$ be the
payload CoG offset expressed in the drone body frame. The gravitational force on
the payload in the body frame (to first order in small tilt angles
$\phi, \theta$) is:

$$\mathbf{f}_{pay}^{body} = m_{pay}\,R^T(\phi,\theta,\psi)\,\begin{bmatrix}0\\0\\-g\end{bmatrix}$$

where $R$ is the ZYX rotation matrix from body to world. The disturbance torque
vector is the moment of this force about the body centre:

$$\boldsymbol{\tau}_{dist} = \Delta\mathbf{r} \times \mathbf{f}_{pay}^{body}$$

Expanding to first order in the Euler angles (small-angle regime near hover,
where $R^T \approx I - [\boldsymbol{\phi}\times]$):

$$\boldsymbol{\tau}_{dist} \approx \Delta\mathbf{r} \times \begin{bmatrix}m_{pay}g\theta \\ -m_{pay}g\phi \\ -m_{pay}g\end{bmatrix}$$

At hover ($\phi = \theta = 0$) this reduces to the three axis-wise components:

$$\tau_{dist,x} = +m_{pay}\,g\,\Delta r_y \quad\text{(roll)}$$
$$\tau_{dist,y} = -m_{pay}\,g\,\Delta r_x \quad\text{(pitch)}$$
$$\tau_{dist,z} = +m_{pay}\,g\,(\Delta r_x\,\phi - \Delta r_y\,\theta) \quad\text{(yaw — zero at exact hover)}$$

For the chosen parameters:

$$\tau_{dist,x} = 0.80 \times 9.81 \times 0.08 = +0.628 \text{ N·m}$$
$$\tau_{dist,y} = 0.80 \times 9.81 \times 0.12 = -0.942 \text{ N·m}$$

The $\Delta r_z$ component ($-0.06$ m) does not add a hover torque but modifies
the state-dependent yaw term: as roll or pitch deviates, a yaw disturbance
proportional to $m_{pay}\,g\,\Delta r_z \cdot \sin(\phi)\,\text{or}\,\sin(\theta)$
appears, pulling yaw away from the reference.

### 6-DOF Rigid Body Dynamics

**Translational equations of motion** (world frame, small-angle thrust projection):

$$m\,\ddot{\mathbf{p}} = T\,\begin{bmatrix}-\sin\theta \\ \sin\phi\cos\theta \\ \cos\phi\cos\theta\end{bmatrix} - mg\,\hat{\mathbf{z}}$$

**Rotational equations of motion** (body frame, Euler's equation):

$$\mathbf{I}\,\dot{\boldsymbol{\omega}} = \boldsymbol{\tau}_{cmd} + \boldsymbol{\tau}_{dist}(\phi,\theta,\psi) - \boldsymbol{\omega} \times (\mathbf{I}\,\boldsymbol{\omega})$$

where $\mathbf{I} = \mathrm{diag}(I_{xx}, I_{yy}, I_{zz})$ and $\boldsymbol{\omega} = [p,q,r]^T$ are body angular rates.

**Euler-angle kinematics** (ZYX convention):

$$\begin{bmatrix}\dot\phi\\\dot\theta\\\dot\psi\end{bmatrix} = \begin{bmatrix}1 & \sin\phi\tan\theta & \cos\phi\tan\theta \\ 0 & \cos\phi & -\sin\phi \\ 0 & \sin\phi/\cos\theta & \cos\phi/\cos\theta\end{bmatrix}\begin{bmatrix}p\\q\\r\end{bmatrix}$$

### Full 3D PID Attitude Controller

Define the 3D attitude error in all three axes:

$$\boldsymbol{\eta}_{err} = \begin{bmatrix}\phi_{ref} - \phi \\ \theta_{ref} - \theta \\ \psi_{ref} - \psi\end{bmatrix}$$

Integrator state with anti-windup:

$$\dot{\mathbf{s}} = \boldsymbol{\eta}_{err}, \qquad \mathbf{s} \leftarrow \mathrm{clip}(\mathbf{s},\,-s_{max},\,s_{max})$$

Controller output torque vector:

$$\boldsymbol{\tau}_{cmd} = K_p\,\boldsymbol{\eta}_{err} + K_d\,(-\boldsymbol{\omega}) + K_i\,\mathbf{s}$$

where $K_p$, $K_d$, $K_i$ are scalar gains applied identically on all axes
(diagonal gain matrix $= k \cdot \mathbf{I}_{3\times 3}$).

### Static Feedforward Trim Angles

From the linearised hover equilibrium with PD only
($K_p\,(\boldsymbol{\eta}_{ref} - \boldsymbol{\eta}_{ss}) + \boldsymbol{\tau}_{dist}^{hover} = \mathbf{0}$):

$$\begin{bmatrix}\phi_{trim}\\\theta_{trim}\\\psi_{trim}\end{bmatrix} = \frac{1}{K_p}\begin{bmatrix}-\tau_{dist,x}/I_{xx} \\ -\tau_{dist,y}/I_{yy} \\ 0\end{bmatrix}$$

The yaw trim is zero at hover (no hover yaw moment), but the feedforward mode
cannot compensate the state-dependent yaw coupling that grows with tilt angle,
which is the key weakness exposed in this scenario.

### 3D Attitude Error Magnitude

To compare all three controllers on a single scalar:

$$\Theta_{err}(t) = \sqrt{\phi_{err}^2(t) + \theta_{err}^2(t) + \psi_{err}^2(t)}$$

Mission success criterion: $\Theta_{err}(t) < 3°$ for $t > 2$ s (settling time)
under the full 3D PID controller.

### Thrust Required Under Tilt

When the drone carries a non-zero $\Delta r_z$ the effective gravity vector
shifts the hover thrust point off the body z-axis. The required total thrust to
maintain vertical acceleration command $a_z^{cmd}$ is:

$$T = \frac{m(g + a_z^{cmd})}{\cos\phi\cos\theta}$$

For the maximum tilt limit $\phi_{max} = \theta_{max} = 25°$:

$$T_{max} = \frac{m\,g}{\cos^2(25°)} \approx 1.21\,m g$$

Exceeding this increases rotor loading; the simulation clips thrust at $1.3\,mg$
and records saturation events.

### Position Controller (Inner Loop)

A PD position controller generates desired roll, pitch, and thrust commands:

$$\mathbf{a}_{cmd}^{xy} = K_{p,pos}(\mathbf{p}_{ref}^{xy} - \mathbf{p}^{xy}) + K_{d,pos}(-\mathbf{v}^{xy})$$

$$\theta_{ref} = -\frac{m\,a_{cmd,x}}{T}, \qquad \phi_{ref} = +\frac{m\,a_{cmd,y}}{T}$$

$$\psi_{ref} = 0 \quad \text{(fixed heading)}$$

These are handed to the attitude PID. The 3D PID integrator on yaw prevents the
state-dependent yaw moment from accumulating into a heading drift.

---

## Key 3D Additions

- **3D CoG offset vector** $\Delta\mathbf{r} = (+0.12, +0.08, -0.06)$ m including
  vertical component; 2D original uses $\Delta r_z = 0$
- **State-dependent yaw disturbance**: $\tau_{dist,z}$ is proportional to
  current tilt angle, coupling roll/pitch errors into yaw — not present in S025
- **Full Euler-angle kinematics matrix**: replaces the S025 small-angle
  approximation $\dot{\boldsymbol{\eta}} \approx \boldsymbol{\omega}$; required
  for correct yaw propagation off-axis
- **Yaw integrator**: third integral channel in the PID; anti-windup limit $s_{max}$
  tuned separately from roll/pitch
- **3D waypoint profile at mixed altitudes**: mission legs include climb and
  descent segments so tilt disturbance projects differently onto the world
  horizontal plane at each leg
- **Thrust saturation tracking**: records timesteps where $T > T_{sat}$ to
  quantify rotor over-demand caused by the $\Delta r_z$ coupling

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Total mass $m$ | 2.30 kg |
| Payload mass $m_{pay}$ | 0.80 kg |
| CoG offset $\Delta\mathbf{r}$ | (+0.12, +0.08, −0.06) m |
| Hover disturbance torque (roll) | +0.628 N·m |
| Hover disturbance torque (pitch) | −0.942 N·m |
| State-dep. yaw disturbance gain | $m_{pay}\,g\,\Delta r_z = 0.471$ N·m/rad |
| Inertia $(I_{xx}, I_{yy}, I_{zz})$ | (0.015, 0.015, 0.025) kg·m² |
| PD position gains $(K_{p,pos}, K_{d,pos})$ | 1.5, 1.8 |
| PID attitude gains $(K_p, K_d, K_i)$ | 8.0, 3.0, 2.0 |
| Anti-windup limit $s_{max}$ (roll/pitch) | 0.5 rad·s |
| Anti-windup limit $s_{max}$ (yaw) | 0.3 rad·s |
| Max tilt $\phi_{max},\,\theta_{max}$ | 25° |
| Thrust saturation $T_{sat}$ | $1.3\,mg$ = 29.4 N |
| Control frequency | 200 Hz |
| Mission waypoints | 5 (mixed altitudes, z ∈ [0.3, 4.0] m) |
| Waypoint reach radius | 0.15 m |
| Maximum simulation time | 30 s |
| Altitude bounds | [0.2, 6.0] m |

---

## Expected Output

- **3D trajectory plot**: three controller paths (no-comp: red, feedforward:
  orange, PID: green) with 3D waypoints shown as gold stars; reveals lateral and
  vertical drift under no-comp and feedforward
- **Attitude time series** ($\phi$, $\theta$, $\psi$ vs time): three subplots,
  one per Euler angle; yaw panel shows yaw drift growing proportionally to tilt
  magnitude — the key observable absent in S025
- **3D attitude error magnitude** $\Theta_{err}(t)$: scalar time series for all
  three controllers; dashed 3° success threshold line
- **Yaw error comparison bar chart**: final yaw deviation for each controller;
  feedforward has nonzero yaw error (cannot compensate state-dependent term),
  full PID drives it to near zero
- **Thrust saturation events**: stem plot showing timesteps where $T > T_{sat}$;
  expected to appear during the climb leg when tilt disturbance peaks
- **Waypoints reached table**: printout showing which controller completes all 5
  waypoints and the time to completion

---

## Extensions

1. Sweep $\Delta r_z$ from 0 to −0.15 m in 5 steps; plot the resulting
   steady-state yaw drift angle vs $\Delta r_z$ to find the threshold below which
   the yaw integrator is no longer necessary
2. Model payload mass uncertainty (±20% of $m_{pay}$) and evaluate robustness:
   the feedforward trim angles are computed from the nominal $m_{pay}$, so
   uncertainty degrades it — quantify residual error vs PID integrator rejection
3. Add a mid-flight payload shift (sudden $\Delta r_z$ step from 0 to −0.06 m
   at $t = 10$ s) and measure PID recovery time vs feedforward response;
   feedforward cannot adapt online
4. Replace the scalar PID gain $K_p\,\mathbf{I}$ with a full $3\times 3$ gain
   matrix designed via LQR on the linearised rotational dynamics; compare
   cross-axis coupling rejection with the scalar design
5. Extend to the cooperative case: two drones sharing the same asymmetric
   payload via rigid cables — each drone's attitude disturbance depends on its
   attachment point offset, making the combined system a 12-DOF problem
   (follow-on to S026 3D)

---

## Related Scenarios

- Original 2D version: [S025](../S025_payload_cog_offset.md)
- Cooperative payload (3D catenary cables): [S026 3D](S026_3d_cooperative_heavy_lift.md)
- Wind disturbance rejection reference: [S024 3D](S024_3d_wind_compensation.md)
- Obstacle avoidance in 3D delivery: [S022 3D](S022_3d_obstacle_avoidance.md)
