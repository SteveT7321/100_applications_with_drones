# S097 3D Upgrade — Aerial Puzzle Assembly

**Domain**: Special Ops & Entertainment | **Difficulty**: ⭐⭐⭐⭐ | **Status**: `[ ]` Not Started
**Based on**: [S097 original](../S097_aerial_puzzle.md)

---

## What Changes in 3D

The original S097 treats every drone's altitude as fixed: all staging positions share
$z_{stage} = 8$ m and all target slots share $z_{asm} = 10$ m, so the z-axis is merely
a single discrete step. The assembly geometry lives entirely in the horizontal plane —
there is no attitude modelling beyond the yaw angle $\psi_k$, and the impedance
controller acts only on position errors; piece orientation in roll and pitch is never
considered.

This 3D upgrade introduces three key additions:

1. **Full 6-DOF piece transport**: each puzzle piece has a body-fixed frame; roll ($\phi$),
   pitch ($\theta$), and yaw ($\psi$) are all independently controlled and must converge to
   the slot's required attitude before insertion is attempted.
2. **Layered assembly sequence**: the hexagon is not assembled in a flat plane but in a
   shallow funnel geometry — alternating pieces arrive from above and below the nominal
   assembly plane ($z_{asm} \pm \Delta z_{approach}$), eliminating simultaneous-collision
   risk at slot boundaries and providing a defined insertion axis for each piece.
3. **Attitude-matching insertion phase**: the final impedance fitting step explicitly aligns
   each piece's body frame with the slot frame before applying the compliant inward force,
   using a 6-DOF impedance law that couples translational and rotational compliance.

---

## Problem Definition

**Setup**: Six quadrotors ($N = 6$) each carry one triangular puzzle piece (mass
$m_p = 0.05$ kg, half-side $a = 0.35$ m). The target assembly centre is
$\mathbf{p}_{centre} = (0,\, 0,\, 10)$ m. Drones start on a staging circle of radius
$r_{stage} = 5$ m at $z_{stage} = 8$ m with random heading offsets.

Each slot $k$ is assigned:
- A 3D target position $\mathbf{p}_k^* \in \mathbb{R}^3$ offset in both the horizontal plane
  and vertically by an approach layer $\Delta z_k$.
- A target orientation quaternion $\mathbf{q}_k^* \in \mathbb{S}^3$ encoding the required
  roll, pitch, and yaw for piece insertion.

**Roles**:
- **Drones** ($N = 6$): identical quadrotors; each runs a three-phase controller
  (consensus positioning, attitude alignment, 6-DOF impedance fitting).
- **Puzzle pieces**: rigid equilateral-triangle panels rigidly attached to each drone;
  their world-frame orientation tracks the drone's full Euler attitude $(\phi_k, \theta_k, \psi_k)$.
- **Slot frames**: six virtual frames fixed in the world; each defines a 6-DOF assembly
  target $(\mathbf{p}_k^*, \mathbf{q}_k^*)$.

**Mission phases**:

1. **Ascent + layered approach** ($t = 0$ to $t_{asc}$): drones climb to their
   individual approach altitudes $z_k^{app} = z_{asm} + \Delta z_k$, where
   $\Delta z_k = \delta_z \cos(2\pi k / N)$ staggers even- and odd-indexed drones.
2. **Consensus positioning** ($t_{asc}$ to $t_{conv}$): ring-graph consensus contracts
   horizontal position errors while altitude holds at $z_k^{app}$.
3. **Attitude alignment** ($t_{conv}$ to $t_{att}$): each drone independently rotates to
   match $\mathbf{q}_k^*$; translation is frozen until attitude error
   $\|\mathbf{q}_{err,k}\|_{axis-angle} \leq \delta_{att} = 2°$.
4. **6-DOF impedance fitting** ($t_{att}$ to $t_{fit}$): translational impedance drives
   drones to $\mathbf{p}_k^*$ while rotational impedance maintains attitude alignment;
   pieces converge along each slot's insertion axis.
5. **Hold**: final configuration is logged; assembly error $\varepsilon_{6DOF}$ reported.

**Objective**: achieve $\varepsilon_{assembly} = \max_k \|\mathbf{p}_k - \mathbf{p}_k^*\| \leq 5$ mm
and $\varepsilon_{att} = \max_k \angle(\mathbf{q}_k, \mathbf{q}_k^*) \leq 1°$ simultaneously,
while maintaining minimum inter-drone separation $d_{min} = 0.3$ m throughout.

---

## Mathematical Model

### Layered Assembly Target Positions

Slots are arranged on a regular hexagon with circumradius $r_{hex} = 1.0$ m. To avoid
simultaneous mid-air collisions, even-indexed slots arrive from above and odd-indexed
slots from below the nominal assembly plane:

$$\Delta z_k = \delta_z \cos\!\left(\frac{2\pi k}{N}\right), \qquad \delta_z = 0.25 \text{ m}$$

The approach position for drone $k$ is:

$$\mathbf{p}_k^{app} = \mathbf{p}_{centre}
  + \begin{pmatrix} r_{hex}\cos\!\left(\tfrac{2\pi k}{N}\right) \\
                    r_{hex}\sin\!\left(\tfrac{2\pi k}{N}\right) \\
                    \Delta z_k \end{pmatrix}$$

The final slot position lies at the nominal plane ($\Delta z = 0$):

$$\mathbf{p}_k^* = \mathbf{p}_{centre}
  + \begin{pmatrix} r_{hex}\cos\!\left(\tfrac{2\pi k}{N}\right) \\
                    r_{hex}\sin\!\left(\tfrac{2\pi k}{N}\right) \\
                    0 \end{pmatrix}$$

### Target Slot Orientation (Quaternion)

Each piece must arrive radially oriented in the horizontal plane (yaw $\psi_k^* = 2\pi k/N$)
and with a small inward tilt $\theta_{tilt} = 5°$ about the tangential axis to lock piece
edges during contact. Define the desired Euler angles:

$$\phi_k^* = 0, \qquad
  \theta_k^* = \theta_{tilt}\sin\!\left(\frac{2\pi k}{N}\right), \qquad
  \psi_k^* = \frac{2\pi k}{N}$$

The corresponding rotation matrix is $\mathbf{R}_k^* = \mathbf{R}_z(\psi_k^*)\,\mathbf{R}_y(\theta_k^*)$,
and the target quaternion is extracted as:

$$\mathbf{q}_k^* = \text{mat2quat}(\mathbf{R}_k^*) \in \mathbb{S}^3$$

### Attitude Error and Alignment Controller

The quaternion error between current attitude $\mathbf{q}_k$ and target $\mathbf{q}_k^*$ is:

$$\mathbf{q}_{err,k} = (\mathbf{q}_k^*)^{-1} \otimes \mathbf{q}_k$$

Expressed as an axis-angle pair $(\hat{\mathbf{n}}_k,\, \theta_{err,k})$, the attitude error
angle is $\theta_{err,k} = 2\arccos(|q_{err,k,0}|)$. The angular velocity command during
the alignment phase is:

$$\boldsymbol{\omega}_{cmd,k} = -K_\omega\,\theta_{err,k}\,\hat{\mathbf{n}}_k, \qquad K_\omega = 2.0 \text{ rad/s per rad}$$

Euler angles are integrated with:

$$\dot{\boldsymbol{\eta}}_k = \mathbf{T}^{-1}(\boldsymbol{\eta}_k)\,\boldsymbol{\omega}_{cmd,k}$$

where $\mathbf{T}^{-1}$ is the standard Euler-rate transformation matrix.

### Consensus Position Update (Extended to 3D)

During the consensus phase the stacked 3D position update retains the original ring-graph
Laplacian but now operates on all three spatial components, with separate gain terms for
horizontal and vertical channels to respect approach-layer constraints:

$$\mathbf{P}[t+1] = \mathbf{P}[t]
  - \varepsilon\,\mathbf{L}\,\mathbf{P}[t]
  + \alpha_{xy}\,\bigl(\mathbf{P}^{app}_{xy} - \mathbf{P}_{xy}[t]\bigr)
  + \alpha_z\,\bigl(\mathbf{z}^{app} - \mathbf{z}[t]\bigr)\hat{\mathbf{e}}_z$$

where $\mathbf{P}_{xy}$ collects the x-y columns, $\mathbf{z}^{app}$ is the vector of
approach altitudes, $\alpha_{xy} = 0.10$, and $\alpha_z = 0.15$ (faster vertical settling).

### 6-DOF Impedance Fitting Law

Once attitude alignment is satisfied, the controller switches to a 6-DOF impedance law.
Translational and rotational channels are decoupled at this scale, so the wrench command is:

$$\mathbf{F}_k^{imp} = K_t\!\left(\mathbf{p}_k^* - \mathbf{p}_k\right) - B_t\,\dot{\mathbf{p}}_k$$

$$\boldsymbol{\tau}_k^{imp} = K_r\,\theta_{err,k}\,\hat{\mathbf{n}}_k - B_r\,\boldsymbol{\omega}_k$$

with translational stiffness $K_t = 8.0$ N/m, translational damping $B_t = 4.0$ N s/m,
rotational stiffness $K_r = 3.0$ N m/rad, and rotational damping $B_r = 1.5$ N m s/rad.

The closed-loop translational dynamics for error $\mathbf{e}_k = \mathbf{p}_k^* - \mathbf{p}_k$:

$$m_D\,\ddot{\mathbf{e}}_k + B_t\,\dot{\mathbf{e}}_k + K_t\,\mathbf{e}_k = \mathbf{F}_{contact,k}$$

$$\omega_{n,t} = \sqrt{\frac{K_t}{m_D}} = 4.0 \text{ rad/s}, \qquad \zeta_t = \frac{B_t}{2\sqrt{K_t m_D}} = 1.0$$

The rotational channel for attitude error $\theta_{err,k}$:

$$I_{eff}\,\ddot{\theta}_{err,k} + B_r\,\dot{\theta}_{err,k} + K_r\,\theta_{err,k} = \tau_{contact,k}$$

$$\omega_{n,r} = \sqrt{\frac{K_r}{I_{eff}}} = 3.87 \text{ rad/s}, \qquad
  \zeta_r = \frac{B_r}{2\sqrt{K_r I_{eff}}} = 1.0 \quad \text{(}I_{eff} = 0.2 \text{ kg m}^2\text{)}$$

Both channels are critically damped to prevent oscillation at the moment of edge contact.

### 6-DOF Assembly Error Metric

$$\varepsilon_{6DOF}(t) = \max_{k}\sqrt{\|\mathbf{p}_k - \mathbf{p}_k^*\|^2
  + \left(\frac{K_t}{K_r}\right)\theta_{err,k}^2}$$

This scalar combines translational and rotational residuals weighted by the ratio of
stiffness gains, giving a single convergence measure in SI-compatible units (metres).

---

## Key Parameters

| Parameter | Symbol | Value |
|-----------|--------|-------|
| Number of drones / pieces | $N$ | 6 |
| Drone mass | $m_D$ | 0.5 kg |
| Piece mass | $m_p$ | 0.05 kg |
| Piece half-side | $a$ | 0.35 m |
| Assembly centre | $\mathbf{p}_{centre}$ | $(0,\, 0,\, 10)$ m |
| Hexagon circumradius | $r_{hex}$ | 1.0 m |
| Staging circle radius | $r_{stage}$ | 5.0 m |
| Staging altitude | $z_{stage}$ | 8.0 m |
| Assembly altitude | $z_{asm}$ | 10.0 m |
| Vertical approach stagger | $\delta_z$ | 0.25 m |
| Inward tilt angle | $\theta_{tilt}$ | 5° |
| Simulation timestep | $\Delta t$ | 0.02 s |
| Consensus coupling gain | $\varepsilon$ | 0.05 |
| Horizontal attraction gain | $\alpha_{xy}$ | 0.10 |
| Vertical attraction gain | $\alpha_z$ | 0.15 |
| Attitude controller gain | $K_\omega$ | 2.0 rad/s per rad |
| Attitude switch threshold | $\delta_{att}$ | 2° |
| Translational stiffness | $K_t$ | 8.0 N/m |
| Translational damping | $B_t$ | 4.0 N s/m |
| Translational natural frequency | $\omega_{n,t}$ | 4.0 rad/s |
| Rotational stiffness | $K_r$ | 3.0 N m/rad |
| Rotational damping | $B_r$ | 1.5 N m s/rad |
| Effective rotational inertia | $I_{eff}$ | 0.2 kg m² |
| Rotational natural frequency | $\omega_{n,r}$ | 3.87 rad/s |
| Both channels damping ratio | $\zeta$ | 1.0 (critical) |
| Phase-switch threshold | $\delta_{switch}$ | 50 mm |
| Assembly success threshold | $\delta_{fit}$ | 5 mm |
| Attitude success threshold | $\delta_{att,final}$ | 1° |
| Minimum inter-drone separation | $d_{min}$ | 0.30 m |

---

## Expected Output

- **3D trajectory plot** with real z variation: drone paths show the layered approach
  altitudes ($z_{asm} \pm \Delta z_k$) before converging on the final slot plane; even
  and odd drones visibly arrive from above and below.
- **Altitude time series**: all six $z_k(t)$ curves showing the approach stagger and
  the final descent to $z_{asm} = 10$ m during the impedance phase.
- **Attitude error panel**: per-drone axis-angle error $\theta_{err,k}(t)$ showing
  convergence through the alignment phase and maintenance during impedance fitting.
- **6-DOF assembly error** $\varepsilon_{6DOF}(t)$ on a log scale with threshold lines
  for $\delta_{switch}$, $\delta_{fit}$, and $\delta_{att,final}$.
- **3D animation**: drones shown as coloured spheres with oriented triangular piece
  outlines that visibly rotate as attitude alignment proceeds; approach layer separation
  is evident in the opening frames.
- **Terminal summary**: consensus convergence time, attitude alignment time, impedance
  fitting time, final translational error (mm), final attitude error (degrees).

---

## Extensions

1. **Piece geometry mismatch with roll correction**: introduce a ±3° manufacturing
   tilt in each piece's roll angle; verify that the rotational impedance channel
   corrects this within the 5 mm / 1° success criteria without exceeding a contact
   force limit of 2 N.
2. **Altitude-band collision avoidance**: during the approach phase, enforce
   altitude separation $|z_k^{app} - z_j^{app}| \geq 0.15$ m for all neighbours;
   implement a repulsive altitude potential and measure how it delays consensus
   convergence.
3. **Variable inward tilt sweep**: vary $\theta_{tilt} \in [0°, 15°]$ and measure
   the trade-off between mechanical self-locking robustness (higher tilt preferred)
   and attitude-control energy consumption.
4. **Partial failure and 5-piece recovery**: if one drone fails during the attitude
   alignment phase, the remaining five drones reconfigure to a regular pentagon using
   the Hungarian algorithm on 3D position costs, including updated target quaternions.
5. **Wind-induced attitude disturbance**: model a horizontal gust as a body-frame
   torque $\boldsymbol{\tau}_{wind} \sim \mathcal{N}(0, 0.05)$ N m during the
   impedance fitting phase; add a disturbance-observer feedforward term and compare
   final attitude error with and without compensation.
6. **Scalability to 3D stacked layers**: extend the puzzle to two hexagonal layers
   (12 drones, $z_{asm,1} = 10$ m and $z_{asm,2} = 10.5$ m) assembled in sequence;
   analyse inter-layer clearance requirements and the increase in total assembly time.

---

## Related Scenarios

- Original 2D version: [S097](../S097_aerial_puzzle.md)
- 3D attitude control reference: [S001](../../01_pursuit_evasion/S001_basic_intercept.md)
- Formation 3D references: [S083 3D Light Show](S083_3d_light_show.md), [S085 3D Light Matrix](S085_3d_light_matrix.md)
- Multi-agent coordination: [S020 Cooperative Interception](../../01_pursuit_evasion/S020_cooperative_interception.md)
- Compliant manipulation: [S066 Cooperative Crane](../../04_industrial_agriculture/S066_cooperative_crane.md)
