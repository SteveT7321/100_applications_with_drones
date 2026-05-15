# S081 3D Upgrade — Selfie Follow Mode

**Domain**: Special Ops & Entertainment | **Difficulty**: ⭐⭐⭐ | **Status**: `[ ]` Not Started
**Based on**: [S081 original](../S081_selfie_follow.md)

---

## What Changes in 3D

The original S081 fixes the target at $z_{tgt} = 0$ (flat ground) and the drone at a constant
cruise altitude of $h = 4\ \text{m}$ throughout the entire mission. The Kalman Filter tracks
only horizontal position and velocity (`state = [x, y, vx, vy]`), the desired drone position
always has `z_d = CRUISE_ALT` hardcoded, and the orbit PD controller applies the same gains to
all three axes but the $z$-error is always zero. The camera framing model only evaluates the
horizontal pixel offset $u$; the vertical offset $v$ is never computed.

In full 3D the subject follows a **height-varying trajectory** — running up a staircase, jumping
obstacles, moving across hilly terrain — so target altitude $z_{tgt}(t)$ changes continuously.
The drone must:

1. **Track target altitude** with a dedicated $z$-channel controller so it maintains a fixed
   vertical clearance above the subject.
2. **Control gimbal pitch** to keep the subject centred vertically in the frame even between
   altitude corrections.
3. **Plan 3D orbit positions** using the full spherical-offset formula so the standoff sphere
   stays at radius $r$ in 3D, not just in the horizontal plane.
4. **Evaluate 2-axis framing** (horizontal $u$ and vertical $v$) and report a joint framing
   score $F_{2D}$.

---

## Problem Definition

**Setup**: A single drone operates in selfie follow mode tracking a moving person across a
terrain with vertical relief. The person follows a **figure-8 path** of horizontal radius
$R = 8\ \text{m}$ at ground speed $v_{tgt} = 1.5\ \text{m/s}$, with altitude varying as a
sinusoidal height profile:

$$z_{tgt}(t) = h_{base} + \Delta h \sin\!\left(\frac{2\pi t}{T_{elev}}\right)$$

where $h_{base} = 0.5\ \text{m}$, $\Delta h = 1.5\ \text{m}$, and $T_{elev} = 20\ \text{s}$
(simulating stairs / gentle hill cycles). The subject occasionally jumps, adding a short
triangular altitude pulse of amplitude $\Delta h_{jump} = 0.6\ \text{m}$ lasting $0.8\ \text{s}$.

GPS measurements of the subject's 3D position are corrupted by zero-mean Gaussian noise:
$\sigma_{GPS,xy} = 0.3\ \text{m}$ horizontally and $\sigma_{GPS,z} = 0.5\ \text{m}$ vertically
(barometric altitude error is larger than horizontal GPS error).

The drone operates within altitude bounds $z \in [1.5,\, 8.0]\ \text{m}$ and maintains:

- **3D standoff radius**: $r_{3D} = 5\ \text{m}$ measured in the full 3D sense.
- **Vertical clearance**: $\Delta z_{clear} = 2.5\ \text{m}$ above the subject by default.
- **Bearing angle**: $\varphi = 135°$ from the target's horizontal heading (rear-left).
- **Gimbal pitch**: commanded to keep the subject on the vertical image centre-line.

**Roles**:

- **Subject (person)**: non-adversarial; follows the height-varying figure-8 with occasional
  jumps; 3D position is reported with independent $xy$ and $z$ noise.
- **Drone**: single UAV running a 6-state Kalman Filter (`[x, y, z, vx, vy, vz]`) and a
  decoupled 3D orbit PD controller; a 2-DOF gimbal (pan + tilt) is controlled separately from
  the body attitude.

**Comparison strategies**:

1. **3D KF + Spherical Orbit PD + Gimbal Tilt** — full 3D estimation and control (nominal).
2. **3D KF + Horizontal Orbit PD (fixed altitude offset)** — altitude is computed as
   $z_d = \hat{z}_{tgt} + \Delta z_{clear}$ without 3D standoff correction; gimbal tilt is
   still active. Captures the impact of ignoring spherical vs cylindrical offset geometry.
3. **Raw 3D GPS + Spherical Orbit PD** — no KF; direct noisy measurements drive the
   controller. Captures the GPS-noise baseline from S081 extended to 3D.

**Objective**: Maximise the joint 2-axis framing score $F_{2D}$ and minimise the 3D orbit
distance RMS error $\bar{e}_{3D}$, comparing all three strategies.

---

## Mathematical Model

### 3D Target Trajectory

Horizontal figure-8 as in S081 (parameterised by angular frequency $\omega$):

$$x_{tgt}(t) = R \sin(2\pi \omega t), \qquad y_{tgt}(t) = \tfrac{R}{2} \sin(4\pi \omega t)$$

Height profile with jump events at times $\{t_{j,i}\}$:

$$z_{tgt}(t) = h_{base} + \Delta h \sin\!\left(\tfrac{2\pi t}{T_{elev}}\right)
+ \sum_i \Delta h_{jump}\,\Lambda\!\left(\tfrac{t - t_{j,i}}{T_{jump}/2}\right)$$

where $\Lambda(\cdot)$ is the unit triangular pulse ($\Lambda(s) = \max(0,\, 1 - |s|)$).

Target 3D heading (yaw only; pitch not commanded):

$$\theta_{tgt}(t) = \operatorname{atan2}(\dot{y}_{tgt},\, \dot{x}_{tgt})$$

### 6-State Kalman Filter — 3D Target Estimation

State vector: $\mathbf{x} = [x,\; y,\; z,\; v_x,\; v_y,\; v_z]^\top \in \mathbb{R}^6$.

Constant-velocity prediction matrix:

$$F = \begin{bmatrix} I_3 & \Delta t\, I_3 \\ \mathbf{0}_3 & I_3 \end{bmatrix} \in \mathbb{R}^{6 \times 6}$$

**Predict step**:

$$\mathbf{x}_{k|k-1} = F\, \mathbf{x}_{k-1}$$

$$P_{k|k-1} = F\, P_{k-1}\, F^\top + Q_{3D}$$

Process noise covariance ($q_{\sigma,xy} = 0.5\ \text{m/s}^2$, $q_{\sigma,z} = 0.8\ \text{m/s}^2$
to account for faster vertical dynamics during jumps):

$$Q_{3D} = \operatorname{diag}\!\Bigl[
  q_{\sigma,xy}^2 \tfrac{\Delta t^3}{3},\;
  q_{\sigma,xy}^2 \tfrac{\Delta t^3}{3},\;
  q_{\sigma,z}^2 \tfrac{\Delta t^3}{3},\;
  q_{\sigma,xy}^2 \Delta t,\;
  q_{\sigma,xy}^2 \Delta t,\;
  q_{\sigma,z}^2 \Delta t
\Bigr]$$

**Measurement model** — GPS observes all three positions:

$$H = \begin{bmatrix} I_3 & \mathbf{0}_3 \end{bmatrix} \in \mathbb{R}^{3 \times 6}, \qquad
R_{gps} = \operatorname{diag}[\sigma_{xy}^2,\; \sigma_{xy}^2,\; \sigma_z^2]$$

**Update step**:

$$K_k = P_{k|k-1}\, H^\top \bigl(H\, P_{k|k-1}\, H^\top + R_{gps}\bigr)^{-1}$$

$$\mathbf{x}_k = \mathbf{x}_{k|k-1} + K_k\bigl(\mathbf{z}_k - H\, \mathbf{x}_{k|k-1}\bigr)$$

$$P_k = (I - K_k H)\, P_{k|k-1}$$

KF output: $\hat{\mathbf{p}}_{tgt} = [\hat{x},\, \hat{y},\, \hat{z}]^\top$,
$\hat{\mathbf{v}}_{tgt} = [\hat{v}_x,\, \hat{v}_y,\, \hat{v}_z]^\top$.

### 3D Orbit Controller — Spherical Standoff

Estimated target yaw from horizontal velocity:

$$\hat{\theta}_{tgt} = \operatorname{atan2}(\hat{v}_y,\, \hat{v}_x)$$

Desired drone position in 3D (spherical offset with fixed vertical clearance):

$$\mathbf{p}_d = \hat{\mathbf{p}}_{tgt} +
\begin{bmatrix}
  r_{xy} \cos(\hat{\theta}_{tgt} + \varphi) \\
  r_{xy} \sin(\hat{\theta}_{tgt} + \varphi) \\
  \Delta z_{clear}
\end{bmatrix}$$

where the horizontal standoff radius is derived from the 3D standoff sphere:

$$r_{xy} = \sqrt{r_{3D}^2 - \Delta z_{clear}^2}$$

So the true 3D distance from drone to subject satisfies $\|\mathbf{p}_d - \hat{\mathbf{p}}_{tgt}\| = r_{3D}$.

Altitude clipping to flight envelope:

$$z_d = \operatorname{clip}\!\bigl(\hat{z}_{tgt} + \Delta z_{clear},\; z_{min},\; z_{max}\bigr)$$

### Orbit PD Control — Full 3D

Position error and its derivative in $\mathbb{R}^3$:

$$\mathbf{e}(t) = \mathbf{p}_d(t) - \mathbf{p}_{drone}(t)$$

$$\dot{\mathbf{e}}(t) = \dot{\mathbf{p}}_d(t) - \mathbf{v}_{drone}(t)$$

Commanded acceleration with decoupled $z$ gain (vertical channel is slower):

$$\mathbf{a}_{cmd} = \begin{bmatrix} K_p\, e_x + K_d\, \dot{e}_x \\
                                      K_p\, e_y + K_d\, \dot{e}_y \\
                                      K_{p,z}\, e_z + K_{d,z}\, \dot{e}_z \end{bmatrix}$$

Drone dynamics (double integrator with drag, applied in 3D):

$$\dot{\mathbf{v}}_{drone} = \mathbf{a}_{cmd} - c_{drag}\, \mathbf{v}_{drone} - g\, \hat{\mathbf{z}} + g\, \hat{\mathbf{z}}$$

(gravity and lift cancel in hover; net effect is drag only, as in S081.)

Drone yaw commanded toward subject in horizontal plane:

$$\psi_{drone} = \operatorname{atan2}\!\bigl(\hat{y}_{tgt} - y_{drone},\; \hat{x}_{tgt} - x_{drone}\bigr)$$

### Gimbal Pitch Control

The gimbal pitch angle required to centre the subject vertically in frame:

$$\phi_{gimbal} = -\arctan\!\left(\frac{z_{drone} - \hat{z}_{tgt}}{d_{horiz}}\right)$$

where $d_{horiz} = \sqrt{(\hat{x}_{tgt} - x_{drone})^2 + (\hat{y}_{tgt} - y_{drone})^2}$.

Gimbal dynamics are modelled as a first-order lag with time constant $\tau_{gimbal} = 0.1\ \text{s}$:

$$\dot{\phi}_{gimbal} = \frac{\phi_{cmd} - \phi_{gimbal}}{\tau_{gimbal}}$$

### 2-Axis Camera Framing Model

The 3D relative vector from drone to subject:

$$\boldsymbol{\delta} = \hat{\mathbf{p}}_{tgt} - \mathbf{p}_{drone}$$

Camera look direction (unit vector toward subject):

$$\hat{\mathbf{l}} = \boldsymbol{\delta} / \|\boldsymbol{\delta}\|$$

Horizontal right-vector of the camera (perpendicular to look direction in horizontal plane):

$$\hat{\mathbf{n}}_{right} = \frac{[-\hat{l}_y,\; \hat{l}_x,\; 0]^\top}
{\|[-\hat{l}_y,\; \hat{l}_x,\; 0]^\top\|}$$

Camera up-vector (orthogonal to both look and right):

$$\hat{\mathbf{n}}_{up} = \hat{\mathbf{l}} \times \hat{\mathbf{n}}_{right}$$

Normalised horizontal and vertical pixel offsets:

$$u = \frac{\arctan\!\left(\dfrac{\boldsymbol{\delta} \cdot \hat{\mathbf{n}}_{right}}
{\|\boldsymbol{\delta}\|}\right)}{\theta_{FOV,h}/2}, \qquad
v = \frac{\arctan\!\left(\dfrac{\boldsymbol{\delta} \cdot \hat{\mathbf{n}}_{up}}
{\|\boldsymbol{\delta}\|}\right)}{\theta_{FOV,v}/2}$$

where $\theta_{FOV,h} = 60°$ and $\theta_{FOV,v} = 45°$ (4:3 sensor aspect ratio).

**Joint 2-axis framing score**:

$$F_{2D} = \frac{1}{T} \int_0^T \mathbf{1}\!\bigl[|u(t)| \leq 0.10 \;\wedge\; |v(t)| \leq 0.10\bigr]\, dt$$

### 3D Orbit Distance Error

$$e_{3D}(t) = \|\mathbf{p}_{drone}(t) - \hat{\mathbf{p}}_{tgt}(t)\| - r_{3D}$$

$$\bar{e}_{3D} = \sqrt{\frac{1}{T}\int_0^T e_{3D}^2(t)\, dt}$$

---

## Key 3D Additions

- **Height-varying subject trajectory**: sinusoidal elevation profile plus triangular jump pulses
  drive the drone's altitude tracking continuously, unlike the flat $z_{tgt} = 0$ of S081.
- **6-state Kalman Filter**: extends the 4-state horizontal KF to full 3D with anisotropic
  process and measurement noise; higher $q_{\sigma,z}$ and $\sigma_z$ reflect barometric
  altitude uncertainty.
- **Spherical standoff geometry**: $r_{xy} = \sqrt{r_{3D}^2 - \Delta z_{clear}^2}$ ensures the
  true 3D distance stays at $r_{3D}$ rather than the horizontal plane only.
- **Decoupled vertical PD gains**: $K_{p,z} < K_p$ accounts for the slower vertical actuator
  bandwidth of a multirotor.
- **Gimbal pitch control with first-order lag**: separate from body yaw; absorbs residual
  altitude tracking error to maintain vertical framing even during jumps.
- **Vertical FOV and 2-axis framing score $F_{2D}$**: replaces the 1-axis $u$-only score of
  S081; requires both $|u| \leq 0.10$ and $|v| \leq 0.10$ simultaneously.
- **Altitude bounds enforcement**: $z_{drone} \in [1.5,\, 8.0]\ \text{m}$ with saturation logic
  so the controller does not attempt to track the subject underground.

---

## Key Parameters

| Parameter | Symbol | Value |
|-----------|--------|-------|
| Figure-8 lobe radius | $R$ | 8.0 m |
| Target walking speed | $v_{tgt}$ | 1.5 m/s |
| Subject base altitude | $h_{base}$ | 0.5 m |
| Elevation amplitude | $\Delta h$ | 1.5 m |
| Elevation period | $T_{elev}$ | 20.0 s |
| Jump amplitude | $\Delta h_{jump}$ | 0.6 m |
| Jump duration | $T_{jump}$ | 0.8 s |
| GPS horizontal noise | $\sigma_{GPS,xy}$ | 0.3 m |
| GPS vertical noise | $\sigma_{GPS,z}$ | 0.5 m |
| GPS update rate | $f_{GPS}$ | 10 Hz |
| 3D standoff radius | $r_{3D}$ | 5.0 m |
| Vertical clearance | $\Delta z_{clear}$ | 2.5 m |
| Derived horizontal standoff | $r_{xy}$ | $\sqrt{5^2 - 2.5^2} \approx 4.33$ m |
| Bearing angle (rear-left) | $\varphi$ | 135° |
| Drone altitude bounds | $z$ range | 1.5 – 8.0 m |
| Camera horizontal FOV | $\theta_{FOV,h}$ | 60° |
| Camera vertical FOV | $\theta_{FOV,v}$ | 45° |
| Framing tolerance (both axes) | — | ±10% of half-FOV |
| KF horizontal process noise | $q_{\sigma,xy}$ | 0.5 m/s² |
| KF vertical process noise | $q_{\sigma,z}$ | 0.8 m/s² |
| Horizontal orbit PD $K_p$ | $K_p$ | 1.5 |
| Horizontal orbit PD $K_d$ | $K_d$ | 0.8 |
| Vertical orbit PD $K_{p,z}$ | $K_{p,z}$ | 1.0 |
| Vertical orbit PD $K_{d,z}$ | $K_{d,z}$ | 0.5 |
| Aerodynamic drag coefficient | $c_{drag}$ | 0.3 s⁻¹ |
| Gimbal lag time constant | $\tau_{gimbal}$ | 0.1 s |
| Simulation timestep | $\Delta t$ | 0.05 s |
| Total mission time | $T$ | 60.0 s |

---

## Expected Output

- **3D trajectory plot**: `mpl_toolkits.mplot3d` scene showing the height-varying subject path
  in blue (with visible altitude changes), the nominal drone path in red, and the cylindrical-
  offset baseline in orange dashed; standoff sphere radius annotated; jump events marked with
  grey vertical bands.

- **Altitude time series**: two-panel figure; top panel shows $z_{tgt}(t)$ (blue), $z_{drone}(t)$
  for all three strategies, and the jump event markers; bottom panel shows the vertical tracking
  error $z_{drone}(t) - z_{tgt}(t) - \Delta z_{clear}$ for each strategy.

- **Gimbal pitch time series**: $\phi_{gimbal}(t)$ showing pitch corrections during altitude
  changes and jumps; overlay with the ideal pitch $\phi_{cmd}(t)$ to show lag.

- **2-axis framing panel**: side-by-side time series of $u(t)$ and $v(t)$ for the nominal
  strategy; $\pm 10\%$ tolerance bands in green; jump event markers.

- **Joint framing score bar chart**: $F_{2D}$ for all three strategies; comparison against the
  S081 1-axis $F$ score to quantify the cost of adding the vertical requirement.

- **3D orbit error time series**: $e_{3D}(t)$ for all three strategies; RMS values in the
  legend.

- **Animation (GIF)**: 3D animated view showing the subject (blue sphere) moving through its
  height profile, the drone (red triangle) tracking in 3D, and a line from the drone to the
  subject representing the gimbal look direction; altitude of both agents shown in the title bar.

Terminal output at mission end:

```
=== S081 3D Selfie Follow ===
Strategy                  F_2D (%)  RMS e_3D (m)  Max e_3D (m)
3D KF + Spherical PD       71.2%       0.38 m        1.45 m
3D KF + Cylinder PD        58.6%       0.52 m        2.11 m
Raw 3D GPS + Spherical PD  39.4%       1.04 m        3.67 m
```

---

## Extensions

1. **Adaptive vertical clearance**: increase $\Delta z_{clear}$ automatically when the subject
   is detected as jumping (vertical velocity $\hat{v}_z > 0.5\ \text{m/s}$) to avoid framing
   the subject's feet during the jump arc; measure the improvement in $v(t)$ distribution.
2. **Terrain-following altitude floor**: replace the fixed $z_{min} = 1.5\ \text{m}$ with a
   Digital Elevation Model (DEM) lookup so the drone avoids obstacles on hilly terrain; combine
   with the spherical offset controller and measure the increase in orbit error near terrain
   features.
3. **Predictive gimbal control**: use the KF-estimated $\hat{v}_z$ to feed-forward the gimbal
   pitch command ($\phi_{ff} = -\arctan(\hat{v}_z \cdot T_{pred} / d_{horiz})$) and reduce
   the vertical framing lag during fast jumps.
4. **3D orbit shot planning**: instead of a fixed rear-left bearing, plan a slow orbit around
   the subject (bearing $\varphi(t)$ increasing at $\dot{\varphi} = 5°/\text{s}$) on the
   standoff sphere surface; combine with gimbal pan to keep the subject centred during the
   cinematic orbit.
5. **Multi-subject tracking**: extend the 6-state KF to a multi-target KF (one filter per
   person); switch the active target to whichever subject has the highest motion energy
   $\|\hat{\mathbf{v}}_{tgt}\|$; measure framing continuity during subject transitions.

---

## Related Scenarios

- Original 2D version: [S081 Selfie Follow Mode](../S081_selfie_follow.md)
- 3D pursuit reference: [S001 Basic Intercept](../../01_pursuit_evasion/S001_basic_intercept.md), [S003 Low-Altitude Tracking](../../01_pursuit_evasion/S003_low_altitude_tracking.md)
- Kalman Filter tracking: [S013 Particle Filter Intercept](../../01_pursuit_evasion/S013_particle_filter_intercept.md), [S042 Missing Person Localization](../../03_environmental_sar/S042_missing_person.md)
- Cinematic control patterns: [S086 Multi-Angle Cinema](../S086_multi_angle_cinema.md), [S092 Movie Chase](../S092_movie_chase.md), [S094 Counter Drone](../S094_counter_drone.md)
