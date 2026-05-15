# S026 3D Upgrade — Cooperative Heavy Lift

**Domain**: Logistics & Delivery | **Difficulty**: ⭐⭐⭐⭐ | **Status**: `[ ]` Not Started
**Based on**: [S026 original](../S026_cooperative_heavy_lift.md)

---

## What Changes in 3D

The original S026 already uses 3D position vectors for drone and load kinematics, but it makes two simplifying assumptions that this upgrade removes:

1. **Cable model is taut-and-straight**: S026 assumes each cable is a rigid, inextensible rod whose unit vector is always $(\mathbf{p}_i - \mathbf{q} - \mathbf{b}_i)/\|\cdot\|$. In true 3D with a non-negligible cable mass this becomes a catenary curve; the cable direction at the attachment point differs from the straight-line direction, introducing a horizontal tension component even under symmetric loading.

2. **Load is treated as a point mass with no rotational dynamics**: S026 enforces moment balance algebraically but does not propagate load attitude ($\phi_L, \theta_L, \psi_L$). A real suspended load swings as a 3D pendulum coupled to the formation. Under wind disturbance or acceleration the load tilts in both the x-z and y-z planes simultaneously, and the swing must be actively damped via coordinated drone motion.

This upgrade adds:
- **Catenary cable geometry** in 3D: cable sag modelled as a parabolic arc; actual attachment-point force direction computed from the arc slope
- **6-DOF load dynamics**: full rigid-body rotational equations of motion for the load body, not just translational
- **3D pendulum swing damping**: a feedback term that drives load angular velocity to zero
- **True 3D obstacle avoidance** during horizontal transit: drones re-plan vertical separation when a cylindrical obstacle intersects the flat formation plane
- **Altitude-variable formation geometry**: the four drones may fly at different heights during obstacle negotiation, requiring the QP to re-solve with asymmetric cable angles

---

## Problem Definition

**Setup**: $N = 4$ quadrotor drones carry a rigid rectangular load ($m_L = 0.8$ kg, half-lengths $a = 0.15$ m, $b = 0.15$ m, $c = 0.05$ m) via catenary cables of rest length $l_0 = 0.5$ m in a full 3D workspace of $[-5, 10] \times [-5, 10] \times [0, 8]$ m. Two cylindrical obstacles of radius $r_{obs} = 0.4$ m stand on the ground along the transit corridor.

**Roles**:
- **Drones 1–4**: each contributes a cable force whose direction at the attachment point is given by the catenary tangent; individual altitude may vary during obstacle negotiation
- **Load**: 6-DOF rigid body; its centroid position $\mathbf{q} \in \mathbb{R}^3$ and attitude quaternion $\mathbf{q}_{att} \in \mathbb{H}$ are both propagated

**Objective**: Lift the load from $\mathbf{q}_0 = [0, 0, 0.1]^T$ to delivery altitude $z_d = 4.0$ m, translate to $\mathbf{q}_{goal} = [5.0, 3.0, 4.0]^T$ while avoiding obstacles, and descend — with load swing amplitude below $5°$ throughout and no cable slack.

---

## Mathematical Model

### Catenary Cable Geometry

For a cable of linear density $\mu$ (kg/m), rest length $l_0$, with endpoints at drone $\mathbf{p}_i$ and attachment point $\mathbf{a}_i = \mathbf{q} + R_L \mathbf{b}_i$ (where $R_L$ is the load rotation matrix), define the chord vector:

$$\boldsymbol{\delta}_i = \mathbf{p}_i - \mathbf{a}_i, \quad d_i = \|\boldsymbol{\delta}_i\|$$

The catenary parameter $\lambda_i$ satisfies:

$$l_0 = \lambda_i \sinh\!\left(\frac{d_{i,h}}{2\lambda_i}\right) \cdot \frac{2}{\cosh(d_{i,v}/(2\lambda_i) )}$$

where $d_{i,h}$ and $d_{i,v}$ are the horizontal and vertical chord components. For the lightweight cables used here ($\mu \ll f_i / g$), the parabolic approximation is accurate:

$$\text{sag}_i = \frac{\mu g d_i^2}{8 f_i}$$

The effective force direction at the load attachment point (bottom of cable) deviates from the straight chord by the sag angle:

$$\hat{\mathbf{c}}_i^{bot} = \frac{\boldsymbol{\delta}_i}{d_i} - \frac{\mu g l_0}{2 f_i}\,\hat{\mathbf{z}}$$

normalised to unit length. For $\mu \to 0$ this recovers the original taut-cable unit vector.

### 6-DOF Load Dynamics

The load body has inertia tensor $\mathbf{I}_L = \text{diag}(I_{xx}, I_{yy}, I_{zz})$ computed from a uniform cuboid. Let $\boldsymbol{\omega}_L$ be the angular velocity in the body frame.

**Translational equations of motion:**

$$m_L \ddot{\mathbf{q}} = \sum_{i=1}^{N} f_i \hat{\mathbf{c}}_i^{bot} - m_L g\,\hat{\mathbf{z}} + \mathbf{F}_{wind}$$

**Rotational equations of motion (Euler):**

$$\mathbf{I}_L \dot{\boldsymbol{\omega}}_L + \boldsymbol{\omega}_L \times (\mathbf{I}_L \boldsymbol{\omega}_L) = \sum_{i=1}^{N} \mathbf{b}_i \times (f_i R_L^T \hat{\mathbf{c}}_i^{bot})$$

**Quaternion kinematics:**

$$\dot{\mathbf{q}}_{att} = \frac{1}{2}\,\mathbf{q}_{att} \otimes \begin{bmatrix}0\\\boldsymbol{\omega}_L\end{bmatrix}$$

### 3D Static Equilibrium Matrix (Asymmetric Formation)

When drones fly at different altitudes $h_i$ (obstacle avoidance mode), the cable unit vectors $\hat{\mathbf{c}}_i^{bot}$ are no longer symmetric. The equilibrium system $\mathbf{A}\mathbf{f} = \mathbf{b}_{rhs}$ is rebuilt each timestep:

$$\mathbf{A} = \begin{bmatrix}
\hat{\mathbf{c}}_1^{bot} & \hat{\mathbf{c}}_2^{bot} & \hat{\mathbf{c}}_3^{bot} & \hat{\mathbf{c}}_4^{bot} \\
\mathbf{b}_1 \times \hat{\mathbf{c}}_1^{bot} & \mathbf{b}_2 \times \hat{\mathbf{c}}_2^{bot} & \mathbf{b}_3 \times \hat{\mathbf{c}}_3^{bot} & \mathbf{b}_4 \times \hat{\mathbf{c}}_4^{bot}
\end{bmatrix}$$

$$\mathbf{b}_{rhs} = \begin{bmatrix}0 & 0 & m_L g & 0 & 0 & 0\end{bmatrix}^T$$

The anti-slack QP from S026 is retained with $f_{min} = 0.05$ N; the 3D asymmetry means the optimal $\mathbf{f}^*$ now distributes load unevenly between the upper and lower pair of drones.

### 3D Pendulum Swing Damping

Represent the load tilt as a small-angle swing vector:

$$\boldsymbol{\eta} = \begin{bmatrix}\phi_L \\ \theta_L\end{bmatrix}$$

Add a swing-damping feedforward to each drone's desired position:

$$\Delta \mathbf{p}_i^{damp} = -K_\eta\,\boldsymbol{\eta} \times \hat{\mathbf{z}} \cdot s_i$$

where $s_i$ is the signed lateral offset of drone $i$ in the relevant plane, and $K_\eta = 0.3$ m/rad is the damping gain. This couples load attitude error back into drone position commands, driving $\dot{\boldsymbol{\omega}}_L \to \mathbf{0}$.

### Obstacle Avoidance — Vertical Separation

For a cylindrical obstacle centred at $(x_o, y_o)$ with radius $r_{obs}$, if the straight-line transit at altitude $z_d$ brings any drone within clearance $d_{clear} = 0.5$ m:

$$\|\mathbf{p}_i^{xy} - \mathbf{o}^{xy}\| < r_{obs} + d_{clear}$$

the affected drone $i$ climbs to:

$$z_i^{avoid} = z_d + \Delta z_{step}, \quad \Delta z_{step} = 0.8 \text{ m}$$

The remaining drones compensate by adjusting their cable tension via the QP, preventing load tilt. Once the obstacle is cleared the drone returns to $z_d$ at rate $0.3$ m/s.

### Formation Geometry in Obstacle Mode

In nominal cruise the four drones form a symmetric cross at altitude $z_d + h_{cable}$:

$$\mathbf{p}_i^{des} = \mathbf{q}^{des} + \mathbf{d}_i, \quad \mathbf{d}_i \in \{[\pm s, 0, h_c]^T,\,[0, \pm s, h_c]^T\}$$

During obstacle avoidance the vertical offsets become drone-specific:

$$\mathbf{d}_i^{avoid} = \begin{bmatrix}d_{i,x} \\ d_{i,y} \\ h_c + \Delta z_i\end{bmatrix}$$

where $\Delta z_i \in \{0, \Delta z_{step}\}$ depending on whether drone $i$ is avoiding the obstacle. The cable attachment geometry is re-evaluated every timestep.

### PD Controller with Tension Feedforward (3D)

Each drone's acceleration command is identical in structure to S026 but now the cable reaction $\mathbf{F}_i^{cable}$ uses the 3D catenary direction:

$$\mathbf{a}_i^{cmd} = K_p(\mathbf{p}_i^{des} - \mathbf{p}_i) + K_d(\dot{\mathbf{p}}_i^{des} - \dot{\mathbf{p}}_i) + \Delta \mathbf{a}_i^{damp}$$

$$\mathbf{F}_i^{total} = m_d(\mathbf{a}_i^{cmd} + g\hat{\mathbf{z}}) + f_i\,\hat{\mathbf{c}}_i^{bot}$$

The vertical thrust required from drone $i$ at cable angle $\alpha_i$ (angle from vertical) is:

$$T_i^{cmd} = \frac{f_i}{\cos\alpha_i} + m_d g$$

In obstacle-avoidance mode $\alpha_i$ increases for the climbing drone, raising its thrust demand and reducing the vertical force contribution to load support, which the QP compensates automatically.

### Load Swing Amplitude Metric

The instantaneous tilt magnitude:

$$\Theta_L(t) = \arccos\!\left(R_L(t)\,\hat{\mathbf{z}} \cdot \hat{\mathbf{z}}\right) = \sqrt{\phi_L^2 + \theta_L^2}$$

Mission success criterion: $\Theta_L(t) < 5°$ for all $t > t_{lift}$.

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Number of drones | 4 |
| Load mass $m_L$ | 0.8 kg |
| Load half-lengths $(a, b, c)$ | (0.15, 0.15, 0.05) m |
| Load inertia $(I_{xx}, I_{yy}, I_{zz})$ | (0.0006, 0.0006, 0.0012) kg·m² |
| Drone body mass $m_d$ | 0.027 kg |
| Cable rest length $l_0$ | 0.5 m |
| Cable linear density $\mu$ | 0.005 kg/m |
| Formation radius $s$ | 0.6 m |
| Nominal cable height offset $h_c$ | 0.40 m |
| Minimum cable tension $f_{min}$ | 0.05 N |
| Swing damping gain $K_\eta$ | 0.3 m/rad |
| Obstacle radius $r_{obs}$ | 0.4 m |
| Obstacle clearance $d_{clear}$ | 0.5 m |
| Altitude step for avoidance $\Delta z_{step}$ | 0.8 m |
| Delivery altitude $z_d$ | 4.0 m |
| Target position $\mathbf{q}_{goal}$ | (5.0, 3.0, 4.0) m |
| PD gains $(K_p, K_d)$ | 4.0, 2.0 |
| Simulation timestep $\Delta t$ | 0.01 s |
| Total mission time | 25 s |
| Max load tilt allowed | 5° |

---

## Expected Output

- 3D trajectory plot: four drone paths (individual colours) and load centroid path (blue), with obstacle cylinders rendered in grey and cable lines drawn at selected timesteps to show sag
- Load tilt $\Theta_L(t)$: time series with 5° threshold line; highlight the obstacle-avoidance phase where asymmetric cable angles excite swing, and show swing damping recovery
- Cable tension time series $f_1(t) \ldots f_4(t)$: shows deliberate asymmetry when one drone climbs over obstacle; verify $f_i \geq f_{min}$ throughout
- Altitude profiles $z_i(t)$ for all four drones: clear step up/down during obstacle negotiation
- Cable sag comparison at two timesteps: nominal symmetric hover vs obstacle-avoidance asymmetric configuration; plot actual catenary arc vs straight-line chord
- Formation top-down snapshot: shows cross pattern deforming and recovering around each obstacle

---

## Extensions

1. Increase cable mass ($\mu = 0.02$ kg/m) and compare catenary sag angle vs straight-line approximation across different payload heights; determine the $\mu$ threshold below which the taut-cable assumption is acceptable
2. Add simultaneous wind gust (lateral $3$ m/s for $2$ s) during obstacle avoidance to stress-test both the QP tension solver and the swing damping controller
3. Extend to $N = 6$ drones in hexagonal formation: when one drone climbs for obstacle avoidance, the five remaining provide better moment balance and the load tilt excursion should be smaller — quantify the improvement
4. Model one-drone failure during obstacle avoidance: the three remaining drones must simultaneously avoid the obstacle and maintain load stability; explore whether the QP can produce a feasible solution
5. Replace PD swing damping with an LQR controller designed from the linearised 3D pendulum model; compare transient swing amplitude and recovery time

---

## Related Scenarios

- Original 2D/flat version: [S026](../S026_cooperative_heavy_lift.md)
- Obstacle avoidance reference: [S022 3D](S022_3d_obstacle_avoidance.md)
- Cargo escort 3D: [S028 3D](S028_3d_cargo_escort.md)
- Payload CoG offset (asymmetric load baseline): [S025](../S025_payload_cog_offset.md)
