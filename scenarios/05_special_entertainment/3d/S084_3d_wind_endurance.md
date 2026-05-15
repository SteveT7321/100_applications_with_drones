# S084 3D Upgrade — High-Wind Endurance Test

**Domain**: Special Ops & Entertainment | **Difficulty**: ⭐⭐⭐⭐ | **Status**: `[ ]` Not Started
**Based on**: [S084 original](../S084_wind_endurance.md)

---

## What Changes in 3D

The original S084 uses a Dryden shaping filter only for the lateral ($y$) and vertical ($z$) wind components while the mean wind vector is fixed in the $y$-direction throughout the mission. The disturbance spectrum is spectrally flat across altitude, and the RLS estimator tracks a single scalar mean-wind bias per axis.

This 3D upgrade introduces three qualitative extensions:

1. **Von Kármán turbulence spectrum** replaces the Dryden model. The von Kármán power-spectral density captures the $-5/3$ inertial-subrange roll-off more accurately for outdoor gusty environments; the shaping filter becomes 5th-order per axis, generating correlated $(W_x, W_y, W_z)$ turbulence.
2. **Altitude-varying gust profile** imposes a wind shear layer: mean wind speed and turbulence intensity both vary with height $z$ according to a logarithmic wind profile, so climbing or descending changes the disturbance the drone experiences.
3. **Full 3D position hold with attitude regulation** extends the mission from a fixed-altitude straight-line traverse to a 3D waypoint sequence $(x, y, z)$ where altitude is also a degree of freedom. The LQR state vector grows to include full 6-DOF position and velocity plus roll, pitch, and yaw, and the RLS estimator jointly adapts all three wind-acceleration components in real time.

---

## Problem Definition

**Setup**: A validation drone must fly a 3D waypoint route consisting of four legs at varying
altitudes (5 m, 10 m, 15 m, 8 m) under a von Kármán turbulent wind field with altitude-dependent
mean speed. The mean wind rotates slowly in the horizontal plane (full 360° over 200 s) while gust
intensity increases with altitude above the logarithmic roughness height $z_0 = 0.03$ m. The drone
runs an onboard RLS estimator that jointly adapts estimates of the 3D wind-acceleration vector
$\hat{\mathbf{w}} \in \mathbb{R}^3$. An LQR controller with full-state feedback handles the residual
dynamics. Roll and pitch are clamped to $\theta_{lim} = 30°$; the mission fails if any attitude
angle exceeds $\theta_{fail} = 40°$ or if 3D position error exceeds $e_{3D,fail} = 6$ m.

**Roles**:
- **Drone**: single quadrotor navigating a four-waypoint 3D route; subject to altitude-varying
  von Kármán gusts and a slowly rotating mean wind vector; controlled by LQR with optional 3D
  RLS feedforward.
- **Wind field**: von Kármán turbulence shaping filter (5th-order per axis) combined with a
  logarithmic wind-shear profile; mean horizontal wind rotates at $\dot{\psi}_w = 1.8°$/s; gust
  intensity $\sigma_w(z) = \sigma_{w,ref}\,(z / z_{ref})^{1/7}$ with $z_{ref} = 10$ m and
  $\sigma_{w,ref} = 4$ m/s.
- **RLS estimator**: online adaptive estimator that tracks the 3D wind-acceleration bias vector;
  uses a diagonal forgetting matrix $\boldsymbol{\Gamma} = \mathrm{diag}(\gamma_x, \gamma_y, \gamma_z)$
  with axis-specific gains to account for the different signal bandwidths of horizontal and vertical
  gusts.

**Objective**: Complete the four-waypoint 3D route while satisfying:
1. 3D position error $\|\mathbf{p}(t) - \mathbf{p}_{ref}(t)\| \leq 6$ m at all times.
2. Attitude angles $|\phi(t)|, |\theta(t)| \leq 40°$ at all times.
3. Minimise RMS 3D tracking error and total energy $\int_0^T \mathbf{u}^\top R\,\mathbf{u}\,dt$ over
   the full route.

**Comparison strategies**:
1. **PID only** — decoupled PD position controller per axis; no wind estimation.
2. **LQR (no feedforward)** — full-state LQR; no wind knowledge.
3. **LQR + 3D RLS feedforward** (proposed) — LQR plus a diagonal forgetting RLS estimator adapted
   to the altitude-dependent disturbance bandwidth.

---

## Mathematical Model

### Von Kármán Turbulence Spectrum

The one-sided PSD of the longitudinal velocity component $W_u$ in the von Kármán model is:

$$\Phi_{uu}(\Omega) = \sigma_u^2 \frac{L_u}{\pi V}
  \frac{1}{\left(1 + \left(\frac{1.339\,L_u\,\Omega}{V}\right)^2\right)^{5/6}}$$

where $\Omega$ (rad/m) is the spatial frequency, $L_u$ is the turbulence scale length (m), $V$ is
the nominal airspeed (m/s), and $\sigma_u$ is the turbulence intensity (m/s). The corresponding
shaping filter transfer function (Pade approximation to order $n = 5$) is:

$$W_u(s) = \sigma_u \sqrt{\frac{2L_u}{\pi V}} \cdot
  \frac{b_4 s^4 + b_3 s^3 + b_2 s^2 + b_1 s + b_0}
       {s^5 + a_4 s^4 + a_3 s^3 + a_2 s^2 + a_1 s + a_0} \cdot N(s)$$

The lateral and vertical components use the same spectral form with scale lengths
$L_v = L_w = L_u / 2$ (MIL-HDBK-1797B recommendation). All three filters are driven by
independent white noise processes; zero-order-hold discretisation at $\Delta t = 0.02$ s yields the
discrete state-space realisations $(A_{vk}, B_{vk}, C_{vk})$ per axis.

### Altitude-Varying Gust Profile (Logarithmic Wind Shear)

Mean horizontal wind speed at height $z$:

$$\bar{U}(z) = \bar{U}_{ref} \cdot \frac{\ln(z / z_0)}{\ln(z_{ref} / z_0)}$$

where $z_0 = 0.03$ m (open terrain roughness length), $z_{ref} = 10$ m, and
$\bar{U}_{ref} = 12$ m/s. The mean wind direction rotates in the $xy$-plane:

$$\bar{\mathbf{W}}(z,t) =
  \bar{U}(z) \begin{pmatrix} \cos(\psi_w(t)) \\ \sin(\psi_w(t)) \\ 0 \end{pmatrix},
  \qquad \dot{\psi}_w = \frac{\pi}{100} \text{ rad/s}$$

Altitude-scaled turbulence intensity:

$$\sigma_w(z) = \sigma_{w,ref} \left(\frac{z}{z_{ref}}\right)^{1/7}, \qquad
  z \in [1.0,\; 20.0] \text{ m}$$

The von Kármán filter input standard deviation is updated at each timestep to $\sigma_w(z_k)$,
creating a height-dependent gust envelope without reinitialising filter states.

### Total Wind Velocity Vector

$$\mathbf{V}_w(t) = \bar{\mathbf{W}}(z,t)
  + \begin{pmatrix} W_x^{turb}(t) \\ W_y^{turb}(t) \\ W_z^{turb}(t) \end{pmatrix}$$

where $W_x^{turb}$, $W_y^{turb}$, $W_z^{turb}$ are the von Kármán filter outputs.

### Aerodynamic Disturbance Force (3D)

$$\mathbf{F}_w = \tfrac{1}{2}\,\rho_{air}\,C_D\,A_{ref}\;
  \left(\mathbf{V}_w - \mathbf{v}_d\right) \odot \left|\mathbf{V}_w - \mathbf{v}_d\right|$$

where $\odot$ denotes element-wise multiplication and $\mathbf{v}_d \in \mathbb{R}^3$ is the drone
velocity vector. This is the same quadratic drag model as S084 but applied to the full 3D relative
wind including the altitude-corrected mean component.

### Full 6-DOF Linearised Quadrotor Dynamics

The state vector is extended to include yaw and yaw rate:

$$\mathbf{x} = \begin{pmatrix}
  p_x & p_y & p_z & v_x & v_y & v_z &
  \phi & \theta & \psi & \dot{\phi} & \dot{\theta} & \dot{\psi}
\end{pmatrix}^\top \in \mathbb{R}^{12}$$

The control input is $\mathbf{u} = (T,\; \tau_\phi,\; \tau_\theta,\; \tau_\psi)^\top$.
The continuous-time linearised plant about hover is:

$$\dot{\mathbf{x}} = A\mathbf{x} + B\mathbf{u} + B_w\mathbf{F}_w$$

The wind disturbance input matrix:

$$B_w = \mathrm{blkdiag}\!\left(\mathbf{0}_{3 \times 3},\;
  \tfrac{1}{m}\mathbf{I}_3,\; \mathbf{0}_{6 \times 3}\right)$$

Key linearised coupling terms (hover trim):

$$\dot{v}_x = g\theta, \quad \dot{v}_y = -g\phi, \quad \dot{v}_z = T/m - g$$

$$\dot{\phi} = \dot{\phi}, \quad \dot{\theta} = \dot{\theta}, \quad \dot{\psi} = \dot{\psi}$$

$$\ddot{\phi} = \tau_\phi / I_{xx}, \quad
  \ddot{\theta} = \tau_\theta / I_{yy}, \quad
  \ddot{\psi} = \tau_\psi / I_{zz}$$

### LQR Optimal Gain (12-State)

$$A^\top P + PA - PBR^{-1}B^\top P + Q = 0, \qquad K = R^{-1}B^\top P$$

State weighting matrix:

$$Q = \mathrm{diag}\!\bigl(
  1,\; 20,\; 20,\;\;
  0.5,\; 4,\; 4,\;\;
  8,\; 8,\; 4,\;\;
  1,\; 1,\; 0.5
\bigr)$$

Input weighting matrix:

$$R = \mathrm{diag}(0.1,\; 1.0,\; 1.0,\; 2.0)$$

### 3D RLS Wind Estimator with Diagonal Forgetting Matrix

The estimator adapts the 3D wind-acceleration bias $\hat{\mathbf{w}}_k \in \mathbb{R}^3$:

$$\hat{\mathbf{w}}_{k+1} = \hat{\mathbf{w}}_k
  + \boldsymbol{\Gamma}\,\bigl(\mathbf{a}_{meas,k}
    - \mathbf{a}_{model,k} - \hat{\mathbf{w}}_k\bigr)$$

where the diagonal forgetting matrix $\boldsymbol{\Gamma}$ has axis-specific gains:

$$\boldsymbol{\Gamma} = \mathrm{diag}(\gamma_x,\; \gamma_y,\; \gamma_z),
  \qquad \gamma_x = \gamma_y = 0.05,\; \gamma_z = 0.08$$

The higher vertical gain $\gamma_z$ accounts for the lower bandwidth of vertical gusts. The 3D
feedforward cancellation command is:

$$\mathbf{u}_{ff} = \begin{pmatrix}
  -m\hat{w}_z \\[4pt]
  \displaystyle -m\hat{w}_y / (g) \cdot k_\phi \\[4pt]
  \displaystyle\phantom{-}m\hat{w}_x / (g) \cdot k_\theta \\[4pt]
  0
\end{pmatrix}$$

where $k_\phi, k_\theta$ are lateral-to-attitude mapping gains (tuned to 5.0 rad·s²).

The combined LQR + 3D RLS control law:

$$\mathbf{u}(t) = -K\,\bigl(\mathbf{x}(t) - \mathbf{x}_{ref}(t)\bigr) + \mathbf{u}_{ff}(t)$$

### 3D Waypoint Reference Trajectory

Four waypoints $\mathbf{w}_i \in \mathbb{R}^3$ with inter-waypoint linear interpolation at
$v_{ref} = 1.0$ m/s:

| Leg | From | To | Altitude (m) |
|-----|------|----|-------------|
| 0 → 1 | $(0, 0, 5)$ | $(30, 0, 10)$ | climb 5 → 10 |
| 1 → 2 | $(30, 0, 10)$ | $(60, 20, 15)$ | climb 10 → 15 |
| 2 → 3 | $(60, 20, 15)$ | $(100, 0, 8)$ | descend 15 → 8 |

The reference state at time $t$ is:

$$\mathbf{p}_{ref}(t) = \mathbf{w}_{i} + \frac{t - t_i}{t_{i+1} - t_i}\,(\mathbf{w}_{i+1} - \mathbf{w}_i),
  \qquad t \in [t_i, t_{i+1}]$$

$$\mathbf{v}_{ref}(t) = v_{ref} \cdot \frac{\mathbf{w}_{i+1} - \mathbf{w}_i}{\|\mathbf{w}_{i+1} - \mathbf{w}_i\|}$$

### Performance Metrics

| Metric | Definition |
|--------|------------|
| RMS 3D tracking error | $\sqrt{\frac{1}{N}\sum_k \|\mathbf{p}_k - \mathbf{p}_{ref,k}\|^2}$ |
| Peak 3D position error | $\max_k \|\mathbf{p}_k - \mathbf{p}_{ref,k}\|$ |
| Peak attitude angle | $\max_k \max(\|\phi_k\|, \|\theta_k\|)$ |
| Altitude RMS error | $\sqrt{\frac{1}{N}\sum_k (z_k - z_{ref,k})^2}$ |
| RLS convergence time | First $t$ s.t. $\|\hat{\mathbf{w}} - \mathbf{w}_{true,mean}\| < 0.5$ m/s² |
| Energy consumption | $\sum_k \mathbf{u}_k^\top R\,\mathbf{u}_k \cdot \Delta t$ |
| Mission success | 1 if all waypoints reached without failure |

---

## Key 3D Additions

- **Von Kármán spectrum**: 5th-order Pade shaping filter per axis replacing the 2nd-order Dryden filter; captures the inertial-subrange $-5/3$ roll-off relevant to outdoor high-wind conditions.
- **Altitude-varying gust profile**: logarithmic wind-shear law scales mean wind speed and turbulence standard deviation $\sigma_w(z)$ continuously with the drone's current altitude $z_k$; climbing to leg 2 at 15 m increases disturbance by $\approx 10\%$ relative to the 10 m baseline.
- **Rotating mean wind direction**: $\dot{\psi}_w = \pi/100$ rad/s causes the mean wind vector to complete a full rotation over the mission, exercising the RLS estimator's ability to track non-stationary wind direction changes rather than a fixed lateral bias.
- **12-state LQR with yaw regulation**: yaw added to the state vector so that wind-induced yaw moments (propeller drag asymmetry under lateral gust) are actively rejected.
- **Axis-specific RLS forgetting gains**: $\gamma_z > \gamma_x = \gamma_y$ to match the higher bandwidth of vertical gusts relative to horizontal gusts in the von Kármán model.
- **3D waypoint route**: altitude changes across legs couple vertical and horizontal disturbance rejection, exposing the controller to wind shear layer transitions.

---

## Key Parameters

| Parameter | Symbol | Value |
|-----------|--------|-------|
| Mean wind speed at $z_{ref}$ | $\bar{U}_{ref}$ | 12 m/s |
| Wind direction rotation rate | $\dot{\psi}_w$ | $\pi/100$ rad/s |
| Von Kármán turbulence intensity | $\sigma_{w,ref}$ | 4 m/s |
| Turbulence scale length | $L_u$ | 200 m |
| Lateral/vertical scale length | $L_v = L_w$ | 100 m |
| Roughness height | $z_0$ | 0.03 m |
| Reference altitude | $z_{ref}$ | 10 m |
| Altitude range | $z$ | 5 – 15 m |
| Forward reference speed | $v_{ref}$ | 1.0 m/s |
| 3D position fail threshold | $e_{3D,fail}$ | 6 m |
| Attitude soft clamp | $\theta_{lim}$ | 30° |
| Attitude fail threshold | $\theta_{fail}$ | 40° |
| Horizontal RLS forgetting gain | $\gamma_x = \gamma_y$ | 0.05 |
| Vertical RLS forgetting gain | $\gamma_z$ | 0.08 |
| Drone mass | $m$ | 1.0 kg |
| Air density | $\rho_{air}$ | 1.225 kg/m³ |
| Drag coefficient | $C_D$ | 0.6 |
| Effective frontal area | $A_{ref}$ | 0.04 m² |
| Roll / pitch inertia | $I_{xx}, I_{yy}$ | 0.01 kg·m² |
| Yaw inertia | $I_{zz}$ | 0.02 kg·m² |
| LQR lateral position weight | $Q_{py}$ | 20 |
| LQR altitude weight | $Q_{pz}$ | 20 |
| LQR attitude weights | $Q_{\phi\theta\psi}$ | diag(8, 8, 4) |
| Simulation timestep | $\Delta t$ | 0.02 s |
| Mission timeout | $T_{max}$ | 200 s |

---

## Expected Output

- **3D waypoint trajectory plot**: `mpl_toolkits.mplot3d` axes showing the three-segment route with reference legs in green dashed and drone paths for all three controllers coloured red (PID), orange (LQR), crimson (LQR+RLS); altitude transitions clearly visible on the $z$-axis.
- **Altitude time series**: $z(t)$ for all three controllers overlaid on the reference altitude profile; shows how wind-shear transitions at leg boundaries affect altitude hold performance.
- **3D position error vs time**: $\|\mathbf{p}(t) - \mathbf{p}_{ref}(t)\|$ for all controllers; $\pm6$ m fail limit marked; LQR+RLS expected to remain below 1 m throughout.
- **Rotating wind direction**: polar plot of $\psi_w(t)$ and $\bar{U}(z_k, t)$ showing the mean wind vector rotating over the mission; overlaid with $\hat{w}_y$ vs $\hat{w}_x$ RLS trajectory in the wind estimation space.
- **Von Kármán gust realisation**: time-series of $W_x^{turb}$, $W_y^{turb}$, $W_z^{turb}$ for the LQR+RLS run; illustrates the correlated $-5/3$ spectrum structure versus the Dryden coloured noise of S084.
- **Altitude-dependent gust envelope**: $\sigma_w(z)$ profile plotted against altitude $z \in [1, 20]$ m; annotated at the four waypoint altitudes with the corresponding turbulence intensity used during that leg.
- **RLS convergence on all three axes**: time-series of $\hat{w}_x$, $\hat{w}_y$, $\hat{w}_z$ overlaid with the slowly-varying true mean wind components; convergence tracks the rotating mean wind vector.
- **Performance metrics bar chart**: grouped bars for RMS 3D error, peak 3D error, and energy consumption across PID, LQR, and LQR+RLS; SUCCESS/FAIL annotation per controller.
- **Flight animation (GIF)**: 3D animated view with the drone icon traversing the waypoint route; wind vector arrow drawn at the drone position updating in real time; altitude colour-coding on the trail.

**Expected metric targets**:

| Metric | PID only | LQR | LQR + 3D RLS |
|--------|----------|-----|--------------|
| Mission success | FAIL | marginal | SUCCESS |
| RMS 3D error | > 6 m | 1.5 – 4 m | < 0.8 m |
| Peak 3D error | > 6 m | 3 – 5 m | < 1.5 m |
| Altitude RMS error | > 3 m | 0.5 – 2 m | < 0.3 m |
| Peak attitude angle | > 40° | 22 – 35° | < 25° |
| RLS convergence time | — | — | < 20 s per leg |

---

## Extensions

1. **Extended Kalman Filter wind estimator**: replace the scalar-per-axis RLS with a 6-state EKF that jointly estimates the 3D mean wind vector and its angular rate of change $\dot{\psi}_w$; compare prediction accuracy during rapid direction transitions against the diagonal RLS.
2. **Optimal altitude selection**: formulate the altitude as an additional degree of freedom in the route optimisation — find the altitude profile $z^*(x)$ that minimises total disturbance energy $\int \sigma_w^2(z)\,ds$ along the route while satisfying waypoint constraints.
3. **Monte Carlo over wind seeds**: run 500 von Kármán realisations for each controller at each wind speed from 6 to 18 m/s; produce a violin plot of RMS 3D error vs mean wind speed, identifying the LQR+RLS break-even wind speed where it stops succeeding.
4. **Adaptive LQR gain scheduling**: schedule the LQR $Q$ matrix as a function of current altitude $z_k$ (increasing lateral penalty at higher, gustier altitudes) and compare with the fixed-gain design.
5. **Hardware-in-the-loop validation**: feed the von Kármán filter output into a gym-pybullet-drones environment via a custom wind-force plugin; use the same LQR+RLS gains from this simulation as initial conditions for fine-tuning with RL-PPO.

---

## Related Scenarios

- Original 2D version: [S084](../S084_wind_endurance.md)
- Wind environment cross-reference: [S058 Typhoon Eye Probing](../../03_environmental_sar/S058_typhoon.md) (Rankine vortex + LQR feedforward)
- 3D references: [S002 3D Evasive Maneuver](../../01_pursuit_evasion/3d/S002_3d_evasive_maneuver.md) (full 3D guidance law structure)
- Follow-ups: [S091 Acrobatics](../S091_acrobatics.md) (aggressive 3D maneuvers under disturbance), [S100 Grand Challenge](../S100_grand_challenge.md) (combined wind + multi-agent endurance)
