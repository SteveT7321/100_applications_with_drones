# S086 3D Upgrade — Multi-Angle Cinematography

**Domain**: Special Ops & Entertainment | **Difficulty**: ⭐⭐⭐ | **Status**: `[ ]` Not Started
**Based on**: [S086 original](../S086_multi_angle_cinema.md)

---

## What Changes in 3D

The original S086 locks all three camera drones at a single fixed altitude (`z_fly = 4.0 m`
hardcoded, and the desired position setter never modifies the z component). The orbit is
therefore a flat horizontal circle — all three drones share the same elevation and the
subject is always viewed from a single vertical angle (roughly 34° depression at $r = 6$ m,
$z = 4$ m). This means the formation provides angular diversity only in azimuth, not in
elevation, and the three camera feeds are cinematically equivalent in terms of vertical
perspective.

This 3D upgrade lifts that constraint in three coupled ways:

1. **Hemisphere orbit choreography**: each drone is assigned an independent elevation angle
   $\theta_k \in [15°, 75°]$ on the sphere of radius $r$ centred on the subject, producing a
   true hemispherical coverage pattern with simultaneous ground-level, eye-level, and
   high-angle perspectives.
2. **Gimbal pitch + yaw servo**: rather than assuming the camera passively points at the
   subject, each drone carries a 2-DOF gimbal whose commanded pitch and yaw angles are solved
   analytically from the 3D drone-to-subject vector, keeping the subject centred in the image
   plane regardless of the drone's altitude.
3. **3D visual occlusion avoidance**: when two drones' line-of-sight rays to the subject come
   within a threshold solid angle $\psi_{min}$ of one another (as seen from the subject), a
   repulsive elevation adjustment is applied to restore angular separation, preventing two
   cameras from shooting nearly identical frames.

---

## Problem Definition

**Setup**: Three camera drones simultaneously orbit a walking subject on a figure-8 path.
Each drone is assigned a fixed azimuthal phase offset $\varphi_k^{az} = 2\pi k/3$ as in
S086, but now also maintains a distinct elevation angle $\theta_k$ above the subject's
horizontal plane. The subject is tracked by a shared 3D Kalman Filter that estimates position
and velocity in all three axes. Each drone steers a 2-DOF gimbal to keep the subject centred.
Mutual occlusion — two drones filming from nearly the same angle — is actively penalised and
corrected.

**Roles**:
- **Subject**: walks at $v = 1.5$ m/s on the figure-8 path; altitude is 0 m (ground level).
- **Camera drones** ($N = 3$): drone $k$ orbits at spherical radius $r = 6$ m, azimuthal phase
  $\varphi_k^{az}$, and elevation angle $\theta_k$ relative to the subject. Default elevation
  assignment: $\theta_0 = 20°$ (low angle), $\theta_1 = 45°$ (mid), $\theta_2 = 70°$ (high).
- **Shared 3D KF** (centralised): estimates $(x, y, z, \dot{x}, \dot{y}, \dot{z})^\top$ of the
  subject; broadcasts to all drones at $\Delta t_{obs} = 0.2$ s.
- **Gimbal servo**: per-drone 2-DOF unit computing commanded pitch $\alpha_{cmd}$ and yaw
  $\beta_{cmd}$ from the 3D drone-to-subject vector.

**Objective**: Provide simultaneous three-angle hemisphere coverage of the subject with:

1. **Hemisphere coverage**: elevation angles are maintained within $\pm 5°$ of their targets
   $\theta_k$ throughout the mission.
2. **Gimbal framing**: the subject remains within $\pm 15\%$ of the image-plane centre for each
   camera; the gimbal must resolve pitch and yaw independently.
3. **Occlusion avoidance**: the angular separation between any two drones as seen from the
   subject must satisfy $\psi_{ij} \geq \psi_{min} = 30°$ at all times.
4. **Collision avoidance**: all pairwise 3D inter-drone distances $d_{ij} \geq d_{min} = 3$ m.

---

## Mathematical Model

### Subject Trajectory (Figure-8, ground plane)

$$\mathbf{q}(t) = \bigl(a \sin(\omega t),\; b \sin(2\omega t),\; 0\bigr)^\top$$

with $a = 10$ m, $b = 6$ m, $\omega = 0.18$ rad/s. Subject altitude is fixed at $z_s = 0$ m.

### 3D Kalman Filter for Subject Tracking

The 3D KF extends the 2D constant-velocity model of S086 to include z-axis position and
velocity. State vector:

$$\mathbf{s} = (x,\; y,\; z,\; \dot{x},\; \dot{y},\; \dot{z})^\top \in \mathbb{R}^6$$

State-transition matrix (constant-velocity model, timestep $\Delta t$):

$$\mathbf{F} = \begin{pmatrix} \mathbf{I}_3 & \Delta t\,\mathbf{I}_3 \\ \mathbf{0}_3 & \mathbf{I}_3 \end{pmatrix} \in \mathbb{R}^{6 \times 6}$$

Observation matrix (GPS position only):

$$\mathbf{H} = \begin{pmatrix} \mathbf{I}_3 & \mathbf{0}_3 \end{pmatrix} \in \mathbb{R}^{3 \times 6}$$

Process noise covariance (discrete Wiener acceleration model):

$$\mathbf{Q} = \sigma_a^2 \begin{pmatrix} \tfrac{\Delta t^4}{4}\mathbf{I}_3 & \tfrac{\Delta t^3}{2}\mathbf{I}_3 \\ \tfrac{\Delta t^3}{2}\mathbf{I}_3 & \Delta t^2\mathbf{I}_3 \end{pmatrix}$$

Observation noise covariance: $\mathbf{R} = \sigma_{obs}^2\,\mathbf{I}_3$.

Prediction and update steps follow the standard Kalman equations as in S086, applied to the
6-dimensional state.

### Hemisphere Orbit Setpoint

Let $\hat{\mathbf{q}}(t) = (\hat{x}, \hat{y}, \hat{z})^\top$ be the KF position estimate.
The base azimuthal phase from subject heading:

$$\varphi_{base}(t) = \operatorname{atan2}\!\bigl(\dot{\hat{y}},\; \dot{\hat{x}}\bigr)$$

The desired azimuthal phase for drone $k$:

$$\varphi_k^{az}(t) = \varphi_{base}(t) + \frac{2\pi k}{3}$$

The 3D orbit setpoint for drone $k$ in spherical coordinates centred on the subject:

$$\mathbf{p}_k^{des}(t) = \hat{\mathbf{q}}(t) + r \begin{pmatrix} \cos\theta_k \cos\!\bigl(\varphi_k^{az}(t)\bigr) \\ \cos\theta_k \sin\!\bigl(\varphi_k^{az}(t)\bigr) \\ \sin\theta_k \end{pmatrix}$$

where $\theta_k$ is the elevation angle above the subject's horizontal plane (positive up).
The altitude of drone $k$ is therefore:

$$z_k^{des}(t) = \hat{z}(t) + r \sin\theta_k$$

For $\theta_0 = 20°$, $\theta_1 = 45°$, $\theta_2 = 70°$ and $r = 6$ m:

$$z_0^{des} \approx 2.05 \text{ m}, \quad z_1^{des} \approx 4.24 \text{ m}, \quad z_2^{des} \approx 5.64 \text{ m}$$

### Gimbal Pitch and Yaw Command

The 3D vector from drone $k$ to the subject (in the drone's world frame):

$$\boldsymbol{\delta}_k(t) = \hat{\mathbf{q}}(t) - \mathbf{p}_k(t)$$

The commanded gimbal **yaw** (pan angle around the drone's vertical axis) and **pitch**
(tilt angle below the drone's horizon):

$$\beta_{k,cmd}(t) = \operatorname{atan2}\!\bigl(\delta_{k,y},\; \delta_{k,x}\bigr)$$

$$\alpha_{k,cmd}(t) = \operatorname{atan2}\!\Bigl(-\delta_{k,z},\; \sqrt{\delta_{k,x}^2 + \delta_{k,y}^2}\Bigr)$$

The gimbal is modelled as a first-order servo with time constant $\tau_g = 0.1$ s:

$$\dot{\alpha}_k = \frac{\alpha_{k,cmd} - \alpha_k}{\tau_g}, \qquad \dot{\beta}_k = \frac{\beta_{k,cmd} - \beta_k}{\tau_g}$$

The normalised image-plane offset used for the framing score is derived from the residual
pointing error angle $\varepsilon_k$:

$$\varepsilon_k(t) = \arccos\!\Bigl(\hat{\mathbf{c}}_k(t) \cdot \frac{\boldsymbol{\delta}_k(t)}{\|\boldsymbol{\delta}_k(t)\|}\Bigr)$$

where $\hat{\mathbf{c}}_k$ is the unit vector along the actual (servo-lagged) gimbal boresight.
The normalised image offset is $\|\delta_k^{img}\| = \tan(\varepsilon_k) / \tan(\text{FOV}/2)$.
Framing is considered good when $\|\delta_k^{img}\| \leq 0.15$.

### 3D Visual Occlusion Metric

The angular separation between drones $i$ and $j$ as seen from the subject is the angle
between their 3D line-of-sight vectors:

$$\psi_{ij}(t) = \arccos\!\left(\frac{(\mathbf{p}_i - \hat{\mathbf{q}}) \cdot (\mathbf{p}_j - \hat{\mathbf{q}})}{\|\mathbf{p}_i - \hat{\mathbf{q}}\|\;\|\mathbf{p}_j - \hat{\mathbf{q}}\|}\right)$$

The target minimum angular separation is $\psi_{min} = 30°$. When $\psi_{ij} < \psi_{min}$,
an elevation repulsion correction is applied to drone $j$ (the higher-indexed one):

$$\Delta\theta_j^{rep} = K_{rep}\,\bigl(\psi_{min} - \psi_{ij}\bigr) \cdot \operatorname{sign}\!\bigl(\theta_j - \theta_i\bigr)$$

The corrected elevation for drone $j$ is clamped to $\theta_j + \Delta\theta_j^{rep} \in [10°, 80°]$.

### Phase Synchronisation Error

Identical to S086, computed in the azimuthal plane:

$$\Delta\varphi_k(t) = \bigl|\operatorname{atan2}(p_{k,y} - \hat{y},\; p_{k,x} - \hat{x}) - \varphi_k^{az}(t)\bigr|_{\text{wrap}}$$

### 3D Pairwise Collision Constraint

$$d_{ij}(t) = \bigl\|\mathbf{p}_i(t) - \mathbf{p}_j(t)\bigr\|_2 \geq d_{min} = 3 \text{ m}$$

For the ideal hemisphere formation, the 3D pairwise distances between drones at different
elevations $\theta_i \neq \theta_j$ and azimuthal offsets $\Delta\varphi = 2\pi/3$:

$$d_{ij}^{ideal} = r\sqrt{\cos^2\!\theta_i + \cos^2\!\theta_j - 2\cos\theta_i\cos\theta_j\cos\!\tfrac{2\pi}{3} + (\sin\theta_i - \sin\theta_j)^2}$$

### Drone Position Controller

The position controller follows the same first-order proportional law as S086, extended to
full 3D:

$$\mathbf{u}_k(t) = K_p\,\bigl(\mathbf{p}_k^{des}(t) - \mathbf{p}_k(t)\bigr), \qquad \|\mathbf{u}_k\|_2 \leq v_{max}$$

$$\dot{\mathbf{p}}_k = \mathbf{u}_k$$

Altitude is bounded: $z_k \in [z_{min}, z_{max}] = [0.5, 8.0]$ m.

---

## Key Parameters

| Parameter | Symbol | Value |
|-----------|--------|-------|
| Number of camera drones | $N$ | 3 |
| Orbit radius | $r$ | 6.0 m |
| Elevation angles (drones 0, 1, 2) | $\theta_0, \theta_1, \theta_2$ | 20°, 45°, 70° |
| Subject walking speed (RMS) | $v_{sub}$ | 1.5 m/s |
| Figure-8 semi-axis x | $a$ | 10.0 m |
| Figure-8 semi-axis y | $b$ | 6.0 m |
| Altitude range | $z_k$ | 0.5 – 8.0 m |
| Minimum angular separation (occlusion) | $\psi_{min}$ | 30° |
| Occlusion repulsion gain | $K_{rep}$ | 0.5 rad/rad |
| Elevation clamp range | $\theta_k$ | [10°, 80°] |
| Minimum 3D inter-drone distance | $d_{min}$ | 3.0 m |
| Phase error tolerance | $\|\Delta\varphi_k\|$ | $\leq 5°$ |
| Framing threshold | — | $\pm 15\%$ of frame centre |
| Target fleet framing score | $\bar{F}$ | $\geq 0.90$ |
| Gimbal servo time constant | $\tau_g$ | 0.1 s |
| Gimbal FOV half-angle | — | 30° |
| KF process noise std dev | $\sigma_a$ | 0.8 m/s² |
| KF observation noise std dev | $\sigma_{obs}$ | 0.3 m |
| KF observation interval | $\Delta t_{obs}$ | 0.2 s |
| Controller proportional gain | $K_p$ | 2.0 |
| Drone max speed | $v_{max}$ | 4.0 m/s |
| Simulation time step | $\Delta t$ | 0.05 s |
| Total simulation duration | $T$ | 60.0 s |

---

## Expected Output

- **3D hemisphere formation plot** (`s086_3d_hemisphere.png`): single 3D axes showing the
  figure-8 subject path at $z = 0$ and three drone helical orbits at clearly different
  altitudes (roughly 2, 4, and 6 m); dashed arcs on the bounding sphere illustrate the
  hemisphere coverage; final positions annotated with elevation angles.
- **Altitude and elevation angle time series** (`s086_altitude_elevation.png`): upper panel
  shows $z_k(t)$ for all three drones (three distinct altitude bands); lower panel shows
  the instantaneous elevation angle $\theta_k(t)$ with dashed horizontal target lines and
  a shaded corridor for the $\pm 5°$ tolerance.
- **Gimbal angles time series** (`s086_gimbal_angles.png`): per-drone pitch $\alpha_k(t)$
  and yaw $\beta_{k}(t)$ versus time; the pitch of the high-elevation drone is steeper
  ($\approx -70°$) than the low-elevation drone ($\approx -20°$), confirming correct 3D
  pointing.
- **Occlusion separation plot** (`s086_occlusion.png`): all three pairwise angular
  separations $\psi_{ij}(t)$ versus time, with a red dashed line at $\psi_{min} = 30°$;
  the repulsion controller should keep all curves above the threshold after an initial
  transient.
- **Phase sync and 3D collision distance** (`s086_sync_separation.png`): azimuthal phase
  errors per drone (upper) and pairwise 3D distances (lower), both with threshold lines.
- **3D animation** (`s086_animation.gif`): rotating azimuth view showing all three drones
  at different altitudes orbiting the walking subject; gimbal boresight rays drawn as
  coloured dotted lines from each drone to the subject; altitude labels updated per frame;
  playback at 20 fps over the full 60-second mission.

**Representative metric values** (expected at successful convergence):

| Metric | Expected value |
|--------|---------------|
| Mean elevation error (all drones) | $\leq 3°$ |
| Minimum pairwise angular separation | $\geq 28°$ (near target) |
| Minimum 3D inter-drone distance | $\geq 5$ m |
| Mean gimbal pointing error | $\leq 2°$ |
| Fleet framing score $\bar{F}$ | $\geq 0.90$ |

---

## Extensions

1. **Continuous elevation sweep**: instead of fixed $\theta_k$ targets, choreograph a slow
   elevation oscillation $\theta_k(t) = \bar{\theta}_k + A\sin(\Omega t + \phi_k)$ to
   produce a dynamic sweep of cinematic angles; maintain $\psi_{ij} \geq \psi_{min}$ at all
   phases of the sweep.
2. **Optimal hemisphere placement**: formulate elevation angle assignment as a non-linear
   programme maximising the minimum pairwise $\psi_{ij}$ subject to drone speed and altitude
   constraints; compare the optimised assignment against the fixed [20°, 45°, 70°] baseline.
3. **Subject height estimation**: extend the 3D KF with a z-velocity state to track a subject
   who climbs stairs or a ramp; evaluate how quickly the gimbal and orbit setpoints adapt to
   the changing ground altitude.
4. **N > 3 hemisphere tessellation**: generalise to $N = 6$ drones using a Fibonacci sphere
   point distribution to maximise angular coverage uniformity; compute the spherical coverage
   fraction and compare coverage entropy across N.
5. **Wind disturbance in 3D**: add a Dryden turbulence model affecting each drone's altitude
   and lateral position independently; evaluate framing degradation and gimbal saturation
   frequency under increasing wind intensity.

---

## Related Scenarios

- Original 2D version: [S086](../S086_multi_angle_cinema.md)
- Truly 3D references: [S001](../../01_pursuit_evasion/S001_basic_intercept.md),
  [S003](../../01_pursuit_evasion/S003_low_altitude_tracking.md)
- Domain 3D variants: [S081 3D Selfie Follow](S081_3d_selfie_follow.md),
  [S085 3D Light Matrix](S085_3d_light_matrix.md)
- Related algorithmic patterns: [S046 Multi-Drone 3D Trilateration](../../03_environmental_sar/S046_trilateration.md)
  (shared 3D KF), [S088 Aerial Acrobatics Synchronisation](../S088_aerial_acrobatics_sync.md)
  (multi-drone timing)
