# S025 3D Upgrade — Payload CoG Offset

**Domain**: Logistics & Delivery | **Difficulty**: ⭐⭐⭐ | **Status**: `[x]` Completed
**Based on**: [S025 original](../S025_payload_cog_offset.md)
**Implementation**: `src/02_logistics_delivery/3d/s025_3d_payload_cog.py`. Results are in [outputs/02_logistics_delivery/3d/s025_3d_payload_cog/](../../../outputs/02_logistics_delivery/3d/s025_3d_payload_cog/README.md). The implementation found that the expected $\Delta r_z$ → yaw amplification does **not** occur (see the corrections below and the README).

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
above the frame). This card originally expected a non-zero $\Delta r_z$ to
couple into all three rotational axes, adding a yaw moment that grows with tilt
angle and making the 2D PID diverge. **The 3D implementation found neither
effect** (corrected 2026-10-24). $\Delta r_z$ does not appear in the yaw
component of the offset torque and reaches yaw only through the small products
of inertia. Peak yaw changes by a factor of 1.00–1.01 between $\Delta r_z = 0$
and $-0.06$ m in free flight, and no controller diverges down to
$\Delta r_z = -0.15$ m. In the card's payload-weight model, $\Delta r_z$ acts on
roll and pitch as a pendulum stiffness.

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
  test whether a yaw-integrator term is required when $\Delta r_z \ne 0$
  (result: it is not; the yaw integrator raises peak yaw and leaves the final
  yaw within 0.01° of the run without it)

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

$$\tau_{dist,x} = -m_{pay}\,g\,\Delta r_y + m_{pay}\,g\,\Delta r_z\,\phi \quad\text{(roll)}$$
$$\tau_{dist,y} = +m_{pay}\,g\,\Delta r_x + m_{pay}\,g\,\Delta r_z\,\theta \quad\text{(pitch)}$$
$$\tau_{dist,z} = -m_{pay}\,g\,(\Delta r_x\,\phi + \Delta r_y\,\theta) \quad\text{(yaw — zero at exact hover)}$$

For the chosen parameters (right-handed body frame, x forward, y left, z up):

$$\tau_{dist,x} = -0.80 \times 9.81 \times 0.08 = -0.628 \text{ N·m}$$
$$\tau_{dist,y} = +0.80 \times 9.81 \times 0.12 = +0.942 \text{ N·m}$$

> **Correction (2026-10-24, from the 3D implementation).** An earlier version of this
> card printed $\tau_{x} = +m_{pay}g\Delta r_y$, $\tau_{y} = -m_{pay}g\Delta r_x$ (the 2D
> model's sign convention) and $\tau_{z} = +m_{pay}g(\Delta r_x\phi - \Delta r_y\theta)$, and
> claimed that $\Delta r_z$ produces a state-dependent **yaw** torque with gain
> $m_{pay}g\Delta r_z = 0.471$ N·m/rad. Evaluating the cross product above shows that
> $\Delta r_z$ does **not** appear in $\tau_z$. It appears in $\tau_x$ and $\tau_y$ as a
> pendulum stiffness of 0.471 N·m/rad, which is restoring for $\Delta r_z < 0$. The physical
> direction of the hover torques is unchanged: the payload side dips. Note also that
> this formula is the payload-weight moment about the geometric centre. In free flight,
> gravity has no moment about the system CoM, and the offset torque is instead the thrust
> moment $-\mathbf{r}_c \times T\hat{\mathbf{e}}_z$ with $\mathbf{r}_c = (m_{pay}/m)\Delta\mathbf{r}$.
> That moment is independent of $\Delta r_z$ and has no yaw component. $\Delta r_z$ then reaches
> yaw only through the products of inertia $I_{xz}, I_{yz}$. The implementation runs both
> models; see the outputs README.

### 6-DOF Rigid Body Dynamics

**Translational equations of motion** (world frame, thrust along the body z-axis $R\,\hat{\mathbf{e}}_z$, shown at $\psi = 0$):

$$m\,\ddot{\mathbf{p}} = T\,\begin{bmatrix}\cos\phi\sin\theta \\ -\sin\phi \\ \cos\phi\cos\theta\end{bmatrix} - mg\,\hat{\mathbf{z}}$$

(Corrected 2026-10-24: the vector was printed as $(-\sin\theta,\ \sin\phi\cos\theta,\ \cos\phi\cos\theta)$.
That is $R^T\hat{\mathbf{e}}_z$, the world z-axis seen in the body frame, not the thrust
direction in the world frame.)

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

$$\begin{bmatrix}\phi_{trim}\\\theta_{trim}\\\psi_{trim}\end{bmatrix} = \frac{1}{K_p}\begin{bmatrix}-\tau_{dist,x} \\ -\tau_{dist,y} \\ 0\end{bmatrix}$$

(Corrected 2026-10-24: an earlier version also divided by $I_{xx}$ and $I_{yy}$. $K_p$ maps an
angle to a **torque**, so the trim is $-\tau/K_p$ in rad. This is the same unit fix as in the
2D S025.)

The yaw trim is zero at hover (no hover yaw moment). The card originally expected
that the feedforward mode could not compensate a tilt-dependent yaw coupling. The 3D
implementation found no such effect: the static trim ends with zero yaw error. Its real
weakness is that, in free flight, the offset torque scales with thrust, so the hover trim
is wrong on climb and descent legs.

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

$$T_{max} = \frac{m\,g}{\cos^2(25°)} \approx 1.22\,m g$$

($1/\cos^2 25° = 1.217$; corrected from 1.21.)

Exceeding this increases rotor loading; the simulation clips thrust at $1.3\,mg$
and records saturation events.

### Position Controller (Inner Loop)

A PD position controller generates desired roll, pitch, and thrust commands:

$$\mathbf{a}_{cmd}^{xy} = K_{p,pos}(\mathbf{p}_{ref}^{xy} - \mathbf{p}^{xy}) + K_{d,pos}(-\mathbf{v}^{xy})$$

$$\theta_{ref} = +\frac{m\,a_{cmd,x}}{T}, \qquad \phi_{ref} = -\frac{m\,a_{cmd,y}}{T}$$

(Corrected 2026-10-24: the signs were printed as $-$ / $+$, consistent with the earlier
wrong thrust vector. With $R\,\hat{\mathbf{e}}_z$ at $\psi = 0$, positive pitch accelerates
the drone toward $+x$ and positive roll toward $-y$. The implementation uses the exact form
$\theta_{ref} = \operatorname{atan2}(b_x, b_z)$, $\phi_{ref} = \arcsin(-b_y/\lVert\mathbf{b}\rVert)$ with
$\mathbf{b} = \mathbf{a}_{cmd} + g\hat{\mathbf{z}}$.)

$$\psi_{ref} = 0 \quad \text{(fixed heading)}$$

These are handed to the attitude PID. The card expected the yaw integrator to
stop a state-dependent yaw moment from building up into heading drift. The
implementation found no such $\Delta r_z$-driven moment (see the correction
above), and the yaw integrator raises peak yaw instead of lowering it.

---

## Key 3D Additions

- **3D CoG offset vector** $\Delta\mathbf{r} = (+0.12, +0.08, -0.06)$ m including
  vertical component; 2D original uses $\Delta r_z = 0$
- **State-dependent yaw disturbance**: $\tau_{dist,z}$ is proportional to
  current tilt angle (corrected: it does not depend on $\Delta r_z$, whose only
  path into yaw is through the products of inertia; the effect is negligible)
- **Full Euler-angle kinematics matrix**: replaces the S025 small-angle
  approximation $\dot{\boldsymbol{\eta}} \approx \boldsymbol{\omega}$; required
  for correct yaw propagation off-axis
- **Yaw integrator**: third integral channel in the PID; anti-windup limit $s_{max}$
  tuned separately from roll/pitch
- **3D waypoint profile at mixed altitudes**: mission legs include climb and
  descent segments so tilt disturbance projects differently onto the world
  horizontal plane at each leg
- **Thrust saturation tracking**: records timesteps where $T > T_{sat}$ to
  quantify rotor over-demand (correction: the simulation shows the saturation
  comes from the climb command, not from the $\Delta r_z$ coupling; see the
  results README)

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Total mass $m$ | 2.30 kg |
| Payload mass $m_{pay}$ | 0.80 kg |
| CoG offset $\Delta\mathbf{r}$ | (+0.12, +0.08, −0.06) m |
| Hover disturbance torque (roll) | −0.628 N·m (x-fwd/y-left/z-up; earlier printed +0.628 in the 2D sign convention) |
| Hover disturbance torque (pitch) | +0.942 N·m (earlier printed −0.942 in the 2D sign convention) |
| $\Delta r_z$ pendulum stiffness (acts on roll/pitch, not yaw; corrected) | $m_{pay}\,g\,\lvert\Delta r_z\rvert = 0.471$ N·m/rad |
| Inertia $(I_{xx}, I_{yy}, I_{zz})$ | (0.015, 0.015, 0.025) kg·m² |
| PD position gains $(K_{p,pos}, K_{d,pos})$ | 1.5, 1.8 |
| PID attitude gains $(K_p, K_d, K_i)$ | 8.0, 3.0, 2.0 |
| Anti-windup limit $s_{max}$ (roll/pitch) | 0.5 rad·s |
| Anti-windup limit $s_{max}$ (yaw) | 0.3 rad·s |
| Max tilt $\phi_{max},\,\theta_{max}$ | 25° |
| Thrust saturation $T_{sat}$ | $1.3\,mg$ = 29.33 N (corrected from 29.4) |
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
  one per Euler angle; the yaw panel was expected to show yaw drift growing with
  tilt (result: peak yaw is at most 2.11° in every run, and in free flight it
  changes by a factor of only 1.00–1.01 between $\Delta r_z = 0$ and $-0.06$ m)
- **3D attitude error magnitude** $\Theta_{err}(t)$: scalar time series for all
  three controllers; dashed 3° success threshold line
- **Yaw error comparison bar chart**: peak and final yaw deviation for each
  controller (result: contrary to the original expectation, feedforward ends
  with zero yaw error, and the PID's yaw integrator does not lower it)
- **Thrust saturation events**: stem plot showing timesteps where $T > T_{sat}$;
  expected during the climb leg (result: only in the first 0.44 s of the climb
  to WP1, caused by the vertical position command and identical with and
  without $\Delta r_z$ in the free-flight model)
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

- Original 2D version: [S025 Payload CoG Offset](../S025_payload_cog_offset.md)
- Cooperative payload (3D catenary cables): [S026 3D Upgrade — Cooperative Heavy Lift](S026_3d_cooperative_heavy_lift.md)
- Wind disturbance rejection reference: [S024 3D Upgrade — Wind Compensation](S024_3d_wind_compensation.md)
- Obstacle avoidance in 3D delivery: [S022 3D Upgrade — Obstacle Avoidance Delivery](S022_3d_obstacle_avoidance.md)
