# S058 3D Upgrade — Typhoon Eye Penetration

**Domain**: Environmental & SAR | **Difficulty**: ⭐⭐⭐⭐⭐ | **Status**: `[ ]` Not Started
**Based on**: [S058 original](../S058_typhoon.md)

---

## What Changes in 3D

The original S058 models the Rankine vortex as a 2D horizontal wind field; the drone is confined to a fixed horizontal plane with no vertical dynamics. The state vector is $\mathbf{x} = (x, y, \dot{x}, \dot{y})^\top$, z is implicitly constant, and the spiral reference is purely planar. Real typhoon reconnaissance flights operate over an altitude range of 50 – 3000 m and must contend with three phenomena absent from the 2D model:

1. **Altitude-dependent vortex intensity**: tangential wind speed decays with height according to a power-law vertical profile; the eyewall is most intense at boundary-layer altitudes.
2. **Vertical secondary circulation**: the real typhoon has an organised updraft in the eyewall and a subsidence column inside the eye; these create a $w$-wind component that couples horizontal and vertical motion.
3. **Vertical sampling mission**: the scientific value lies in collecting a full meteorological sounding — pressure, humidity, and temperature as a function of altitude — so the reference trajectory must prescribe altitude as well as horizontal position.

This upgrade extends the state space to $\mathbb{R}^6$, lifts the Rankine vortex into a 3D axisymmetric field with an altitude envelope, adds a vertical reference descent profile, and requires the MPC and H-infinity controllers to track a helical descent path through the eyewall.

---

## Problem Definition

**Setup**: A single research drone enters the typhoon domain at $(r_{start}, 0, z_{top}) = (190\text{ m}, 0°, 500\text{ m})$ and must spiral inward while descending to the boundary layer at $z_{bot} = 50$ m, ultimately reaching the calm eye region ($r < r_{eye} = 15$ m) at low altitude. The 3D domain is a cylinder of radius 200 m and height 600 m centred on the eye axis. The 3D Rankine vortex wind field varies with both horizontal radius $r$ and altitude $z$.

**Roles**:
- **Drone**: mass $m = 1.5$ kg, maximum total thrust $F_{max} = 30$ N, attitude limit $\pm 45°$ in roll and pitch, MPC horizon $N_p = 20$ steps. Carries a three-axis IMU and a barometric altimeter.
- **3D typhoon wind field**: a 3D axisymmetric Rankine vortex modulated by a vertical decay envelope, plus a vertical updraft/subsidence component and a stochastic turbulence overlay; treated as a measured-disturbance input to the MPC.

**Objective**: Track the 3D helical descent reference from entry altitude to boundary layer while minimising 3D trajectory tracking error, satisfying thrust and attitude constraints, and completing the altitude sounding. Compare PD, MPC with 3D disturbance feed-forward, and H-infinity controllers on 3D tracking error and mission success rate.

**Comparison strategies**:
1. **PD baseline** — independent PD loops on $x$, $y$, $z$ with no wind model; wind acts as an uncompensated 3D external force.
2. **3D MPC with disturbance feed-forward** — full 3D state-space MPC using the known 3D vortex model as a measured disturbance input, optimising 3D thrust over the receding horizon.
3. **H-infinity robust controller** — 3D $\mathcal{H}_\infty$ synthesis bounding the worst-case gain from the 3D wind disturbance to 3D tracking error; provides a certified $\gamma$ bound across all altitudes.

---

## Mathematical Model

### 3D Rankine Vortex Wind Field

Let $(r, \theta, z)$ denote cylindrical coordinates relative to the eye axis, where $r = \sqrt{x^2 + y^2}$. The horizontal tangential and radial components follow the Rankine profile scaled by a vertical decay envelope $\Phi(z)$:

$$v_{tan}(r, z) = \Phi(z) \cdot \begin{cases}
  v_{max} \cdot \dfrac{r}{r_{max}} & r \leq r_{max} \\[6pt]
  v_{max} \cdot \dfrac{r_{max}}{r} & r > r_{max}
\end{cases}$$

$$v_{rad}(r, z) = -\Phi(z) \cdot v_{inflow} \cdot \exp\!\left(-\frac{(r - r_{max})^2}{2\sigma_{rad}^2}\right)$$

The vertical decay envelope models the reduction of vortex intensity with altitude above the boundary layer top $z_{bl} = 200$ m:

$$\Phi(z) = \begin{cases}
  1 & z \leq z_{bl} \\[4pt]
  \exp\!\left(-\dfrac{(z - z_{bl})^2}{2\sigma_z^2}\right) & z > z_{bl}
\end{cases}$$

with $\sigma_z = 300$ m so that wind intensity at 500 m is approximately 70% of the boundary-layer peak.

The vertical wind component $w(r, z)$ represents the organised secondary circulation — upward in the eyewall and downward inside the eye:

$$w(r, z) = w_{max} \cdot \exp\!\left(-\frac{(r - r_{max})^2}{2\sigma_{rad}^2}\right) \cdot \sin\!\left(\frac{\pi z}{z_{top}}\right)$$

where $w_{max} = 5$ m/s is the peak vertical velocity and $z_{top} = 600$ m is the domain top. Inside the eye ($r < r_{eye}$) the sign is reversed (subsidence):

$$w_{eye}(r, z) = -w_{sub} \cdot \left(1 - \frac{r}{r_{eye}}\right) \cdot \sin\!\left(\frac{\pi z}{z_{top}}\right), \qquad w_{sub} = 2\text{ m/s}$$

The full 3D wind vector at Cartesian position $\mathbf{p} = (x, y, z)^\top$ is:

$$\mathbf{f}_{wind}(\mathbf{p}) = v_{tan}(r,z)\,\hat{\mathbf{e}}_\theta + v_{rad}(r,z)\,\hat{\mathbf{e}}_r + w(r,z)\,\hat{\mathbf{e}}_z$$

where the horizontal unit vectors are identical to the 2D case and $\hat{\mathbf{e}}_z = (0, 0, 1)^\top$:

$$\hat{\mathbf{e}}_r = \frac{1}{r}\begin{pmatrix} x \\ y \\ 0 \end{pmatrix}, \qquad
  \hat{\mathbf{e}}_\theta = \frac{1}{r}\begin{pmatrix} -y \\ x \\ 0 \end{pmatrix}$$

The stochastic turbulence overlay is now three-dimensional:

$$\mathbf{f}_{turb}(t) \sim \mathcal{N}\!\left(\mathbf{0},\; \sigma_w^2(r,z)\,\mathbf{I}_3\right), \qquad
  \sigma_w(r,z) = 0.1 \cdot \Phi(z) \cdot v_{tan}(r, z_{bl})$$

### 3D Drone Dynamics

The drone is modelled as a 3D point mass with thrust control $\mathbf{u} = (u_x, u_y, u_z)^\top$ (body-frame force components in N) and gravity acting along $-\hat{\mathbf{e}}_z$:

$$m\ddot{\mathbf{p}} = \mathbf{u} - mg\,\hat{\mathbf{e}}_z + m\,\mathbf{f}_{wind}(\mathbf{p}) + \mathbf{f}_{turb}(t) - b\,\dot{\mathbf{p}}$$

The state vector is $\mathbf{x} = (\mathbf{p}, \dot{\mathbf{p}})^\top \in \mathbb{R}^6$. The linearised continuous-time state-space model about hover is:

$$\dot{\mathbf{x}} = A\mathbf{x} + B\mathbf{u} + E\mathbf{w} + \mathbf{g}$$

$$A = \begin{pmatrix} \mathbf{0}_3 & \mathbf{I}_3 \\ \mathbf{0}_3 & -\dfrac{b}{m}\mathbf{I}_3 \end{pmatrix}, \quad
  B = \begin{pmatrix} \mathbf{0}_3 \\ \dfrac{1}{m}\mathbf{I}_3 \end{pmatrix}, \quad
  E = \begin{pmatrix} \mathbf{0}_3 \\ \mathbf{I}_3 \end{pmatrix}, \quad
  \mathbf{g} = \begin{pmatrix} \mathbf{0}_3 \\ -g\,\hat{\mathbf{e}}_z \end{pmatrix}$$

where $\mathbf{w} = \mathbf{f}_{wind} + \mathbf{f}_{turb}/m \in \mathbb{R}^3$ is the total disturbance acceleration vector.

The combined thrust constraint couples horizontal and vertical actuation. Defining the gravity-compensating baseline $u_{z,0} = mg$, the attitude limit translates to:

$$\|\mathbf{u}_{xy}\| \leq u_{z,0} \tan(45°) = mg, \qquad u_{z,min} \leq u_z \leq u_{z,max}$$

$$u_{z,min} = mg - \Delta u_z, \quad u_{z,max} = mg + \Delta u_z, \quad \Delta u_z = 0.3\,mg$$

The total thrust magnitude constraint: $\|\mathbf{u}\| \leq F_{max} = 30$ N.

### 3D Helical Descent Reference Trajectory

The reference combines the 2D inbound spiral with a linear altitude descent:

$$r_{ref}(t) = r_{start} - v_{inbound}\,t, \qquad \theta_{ref}(t) = \theta_0 + \omega_{spiral}\,t$$

$$z_{ref}(t) = z_{top} - v_{descent}\,t$$

where $v_{descent} = (z_{top} - z_{bot}) / T_{mission}$ is the constant descent rate, $z_{top} = 500$ m, $z_{bot} = 50$ m, and $T_{mission} = (r_{start} - r_{eye}) / v_{inbound} \approx 219$ s gives $v_{descent} \approx 2.05$ m/s.

The full 3D Cartesian reference:

$$\mathbf{p}_{ref}(t) = \begin{pmatrix} r_{ref}(t)\cos\theta_{ref}(t) \\ r_{ref}(t)\sin\theta_{ref}(t) \\ z_{ref}(t) \end{pmatrix}$$

The reference velocity:

$$\dot{\mathbf{p}}_{ref}(t) = \begin{pmatrix}
  -v_{inbound}\cos\theta_{ref} - r_{ref}\omega_{spiral}\sin\theta_{ref} \\
  -v_{inbound}\sin\theta_{ref} + r_{ref}\omega_{spiral}\cos\theta_{ref} \\
  -v_{descent}
\end{pmatrix}$$

Altitude bands divide the mission into three sampling phases:

| Altitude band | Range | Primary hazard |
|---|---|---|
| Upper troposphere | 500 – 300 m | Moderate wind shear |
| Mid-level eyewall | 300 – 150 m | Peak $\Phi(z)$ transition |
| Boundary layer | 150 – 50 m | Maximum $v_{tan}$, vertical gusts |

### 3D MPC Formulation

Discretise the 3D dynamics with timestep $\Delta t = 0.1$ s via ZOH. The state weighting matrix is extended to six dimensions:

$$Q = \mathrm{diag}(10, 10, 10, 1, 1, 1)$$

penalising horizontal and vertical position errors equally. The control weight is:

$$R = \mathrm{diag}(0.1, 0.1, 0.05)$$

where the smaller $z$-axis weight reflects the tighter altitude tracking requirement. At each control step $k$, solve the finite-horizon QP over prediction horizon $N_p = 20$:

$$\min_{\{u_k,\ldots,u_{k+N_p-1}\}} \sum_{j=0}^{N_p-1} \Bigl(
  \|\mathbf{x}_{k+j} - \mathbf{x}_{ref,k+j}\|^2_Q + \|\mathbf{u}_{k+j}\|^2_R
\Bigr) + \|\mathbf{x}_{k+N_p} - \mathbf{x}_{ref,k+N_p}\|^2_P$$

subject to:

$$\|\mathbf{u}_{xy,k+j}\| \leq mg, \quad u_{z,min} \leq u_{z,k+j} \leq u_{z,max}, \quad j = 0, \ldots, N_p - 1$$

The 3D wind disturbance $\mathbf{w}_k = \mathbf{f}_{wind}(\mathbf{p}_k)$ is provided as a feed-forward term using the known 3D Rankine-vertical model evaluated at predicted positions and altitudes.

### 3D H-Infinity Disturbance Rejection

The 3D tracking error output selects all three position states:

$$\mathbf{z} = C_z(\mathbf{x} - \mathbf{x}_{ref}), \qquad C_z = \mathrm{diag}(\sqrt{Q_{pos}},\sqrt{Q_{pos}},\sqrt{Q_{pos}}, 0, 0, 0)$$

The closed-loop transfer from 3D disturbance $\mathbf{w} \in \mathbb{R}^3$ to 3D error $\mathbf{z} \in \mathbb{R}^3$ is $T_{zw}(s) \in \mathbb{R}^{3\times3}$. The $\mathcal{H}_\infty$ control problem seeks $K_\infty \in \mathbb{R}^{3\times6}$ such that:

$$\|T_{zw}\|_\infty = \sup_{\omega \in \mathbb{R}} \bar{\sigma}\!\left[T_{zw}(j\omega)\right] < \gamma$$

The worst-case disturbance direction now includes the vertical updraft channel; the optimal $\gamma^*$ is expected to increase relative to the 2D case because the vertical dynamics introduce additional resonant modes.

### 3D Tracking Error Metrics

Mean 3D position tracking error:

$$\bar{e}_{3D} = \frac{1}{T_{steps}} \sum_{k=1}^{T_{steps}} \|\mathbf{p}_k - \mathbf{p}_{ref,k}\|$$

Horizontal and vertical components separated:

$$\bar{e}_{xy} = \frac{1}{T_{steps}} \sum_{k} \sqrt{(x_k - x_{ref,k})^2 + (y_k - y_{ref,k})^2}, \qquad
  \bar{e}_z = \frac{1}{T_{steps}} \sum_{k} |z_k - z_{ref,k}|$$

Altitude sampling completeness: the fraction of prescribed altitude waypoints (every 10 m from $z_{top}$ to $z_{bot}$) for which $|z_k - z_{wpt}| < 5$ m when the drone is within $r < 20$ m of the waypoint radius.

---

## Key 3D Additions

- **3D Rankine vortex**: altitude-dependent tangential and radial wind via vertical decay envelope $\Phi(z)$; $z$-axis secondary circulation with eyewall updraft and eye subsidence.
- **Vertical reference profile**: helical descent from $z_{top} = 500$ m to $z_{bot} = 50$ m at $v_{descent} \approx 2.05$ m/s, giving three distinct altitude sampling bands.
- **6D state-space**: state $\mathbf{x} \in \mathbb{R}^6$, control $\mathbf{u} \in \mathbb{R}^3$, disturbance $\mathbf{w} \in \mathbb{R}^3$; gravity compensation explicit in the vertical thrust channel.
- **Decoupled thrust constraints**: horizontal thrust cone ($\|\mathbf{u}_{xy}\| \leq mg$) and vertical thrust band ($u_z \in [0.7\,mg, 1.3\,mg]$) applied separately to reflect rotor geometry.
- **Altitude-band hazard classification**: boundary-layer band carries maximum wind load; upper troposphere band imposes wind shear across the prediction horizon.
- **3D H-infinity synthesis**: worst-case disturbance now spans all three spatial directions; $\gamma^*$ certifies robustness across the full altitude descent.

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Entry position $(r_{start}, z_{top})$ | (190 m, 500 m) |
| Target eye radius $r_{eye}$ | 15 m |
| Target floor altitude $z_{bot}$ | 50 m |
| Max-wind radius $r_{max}$ | 150 m |
| Peak tangential wind $v_{max}$ | 50 m/s |
| Boundary-layer top $z_{bl}$ | 200 m |
| Vertical decay width $\sigma_z$ | 300 m |
| Peak vertical (eyewall updraft) $w_{max}$ | 5 m/s |
| Peak eye subsidence $w_{sub}$ | 2 m/s |
| Peak radial inflow $v_{inflow}$ | 8 m/s |
| Inflow layer width $\sigma_{rad}$ | 40 m |
| Turbulence intensity | 10% of local $v_{tan}(r, z_{bl})$ |
| Drone mass $m$ | 1.5 kg |
| Max total thrust $F_{max}$ | 30 N |
| Horizontal thrust limit | $mg \approx 14.7$ N (45 deg) |
| Vertical thrust band $\Delta u_z$ | $0.3\,mg \approx 4.4$ N |
| Aerodynamic drag $b$ | 0.4 N·s/m |
| Radial closure rate $v_{inbound}$ | 0.8 m/s |
| Descent rate $v_{descent}$ | ~2.05 m/s |
| Spiral angular rate $\omega_{spiral}$ | 0.12 rad/s |
| Mission duration $T_{mission}$ | ~219 s |
| Simulation timestep $\Delta t$ | 0.1 s |
| MPC prediction horizon $N_p$ | 20 steps (2 s) |
| MPC position weights $Q_{pos}$ | 10 (all axes) |
| MPC vertical control weight $R_z$ | 0.05 |
| Altitude waypoint spacing | 10 m |
| Altitude acceptance threshold | 5 m |

---

## Expected Output

- **3D helix trajectory plot**: full 3D axes showing the prescribed helical descent reference (dashed) and actual drone trajectories for all three controllers; eye boundary shown as a vertical cylinder ($r = r_{eye}$); eyewall ring visible at $r_{max}$ at multiple altitudes; colour-coded by altitude to show descent progress.
- **Horizontal trajectory (top-down)**: x-y plane projection identical in spirit to S058 2D plot, allowing direct comparison with the original; wind speed colour-mapped in the background at boundary-layer altitude.
- **Altitude time series**: $z(t)$ for all three controllers vs the reference $z_{ref}(t)$; shaded bands marking the three altitude zones (upper troposphere, mid-level eyewall, boundary layer).
- **3D tracking error decomposition**: $e_{xy}(t)$ and $e_z(t)$ on separate subplots, showing where horizontal vs vertical errors peak (expected: $e_{xy}$ peaks at eyewall crossing; $e_z$ peaks in updraft zone).
- **3D wind field slice plots**: quiver plots of $\mathbf{f}_{wind}$ at three representative altitudes ($z = 50, 200, 500$ m) to illustrate the effect of $\Phi(z)$; vertical cross-section ($r$-$z$ plane) showing the updraft column.
- **Wind speed vs altitude profile**: $v_{tan}(r_{max}, z)$ and $|w(r_{max}, z)|$ vs altitude, demonstrating the boundary-layer intensification and the $\Phi(z)$ decay.
- **Controller comparison table**: $\bar{e}_{3D}$, $\bar{e}_{xy}$, $\bar{e}_z$, $e_{peak}$, $J_{effort}$, sampling completeness (%), and mission success for PD, 3D MPC, and 3D H-infinity.
- **Animation (GIF)**: 3D rotating-view animation of the drone descending the helical path; wind vectors displayed at the drone's current $(r, z)$ position; altitude gauge on the side; eye region illuminates green on success.

---

## Extensions

1. **Moving typhoon track in 3D**: allow the eye centre to translate horizontally at $v_{eye} = 5$ m/s while the vertical structure propagates intact; the reference helix must be continuously recomputed in the moving eye-fixed frame, testing the 3D MPC look-ahead horizon.
2. **Wind shear layer crossing**: insert a sharp horizontal wind-shear layer at $z = 200$ m where $v_{tan}$ changes by $\pm 15$ m/s over a 20 m vertical interval; assess how the altitude-band aware MPC handles the shear crossing relative to the PD baseline.
3. **Multi-drone sounding array**: deploy $N = 3$ drones at entry angles 120° apart and at different starting altitudes (500, 350, 200 m) to sample the 3D vortex simultaneously; coordinate descent rates so all three reach the eye at the same time for a joint three-level pressure sounding.
4. **Online 3D wind estimation**: treat the 3D vortex as unknown and use an Unscented Kalman Filter (UKF) to estimate $v_{max}$, $r_{max}$, and $\Phi(z)$ online from IMU and barometer residuals; feed the estimates into the 3D MPC and quantify convergence speed vs altitude descent progress.
5. **RL policy for 3D penetration**: train a PPO agent on randomised 3D vortex parameters ($v_{max}$, $r_{max}$, $\sigma_z$, $w_{max}$) and variable entry altitudes; test zero-shot generalisation to a Category-5 vertical profile with $v_{max} = 70$ m/s and $w_{max} = 12$ m/s not seen during training.

---

## Related Scenarios

- Original 2D version: [S058 Typhoon Eye Probing](../S058_typhoon.md)
- Vertical profile reference: [S060 Meteorological Profiling](../S060_meteorological.md)
- 3D wind disturbance in pursuit context: [S004 Disturbance Rejection Chase](../../01_pursuit_evasion/S004_disturbance_rejection.md)
- Extreme environment monitoring: [S055 Oil Spill Tracking](../S055_oil_spill.md), [S056 Radiation Mapping](../S056_radiation.md)
- Ocean hazard analogue: [S059 Sonar Buoy Relay](../S059_sonar_relay.md)
- 3D evasion format reference: [S002 3D Upgrade](../../01_pursuit_evasion/3d/S002_3d_evasive_maneuver.md)
