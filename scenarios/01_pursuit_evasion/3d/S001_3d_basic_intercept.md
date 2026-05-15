# S001 3D Upgrade — Basic Intercept

**Domain**: Pursuit & Evasion | **Difficulty**: ⭐⭐ | **Status**: `[ ]` Not started
**Based on**: [S001 original](../S001_basic_intercept.md)

---

## What Changes in 3D

The original S001 uses a point-mass PD velocity controller: `v_cmd = Kp * error + Kd * (-vel)`. Although the arena is nominally 3D, the controller has no explicit orientation state — yaw and pitch are implicit in whatever direction the velocity vector happens to point. Because the pursuer starts at z = 1 m and the target sits at z = 2 m, the trajectory contains a brief vertical climb component, but this is incidental to the PD error term rather than a deliberate 6-DOF maneuver. There is also no coupling between altitude and sensor performance.

This upgrade replaces the implicit velocity command with a full 6-DOF guidance model:

- **Yaw** ($\psi$) and **pitch** ($\gamma$) angles are tracked as explicit state variables updated each control step.
- The acceleration command is decomposed into a heading-rotation demand and a speed demand, mimicking how a real quadrotor receives roll/pitch/thrust commands.
- **Vertical intercept geometry** is modeled through the elevation angle $\varepsilon$ to the target, allowing the guidance law to generate dedicated vertical acceleration when the target is above or below the current flight altitude.
- **Altitude-dependent sensor range** $R_{sense}(z)$ models a downward-facing sensor cone: the effective detection radius shrinks when the pursuer climbs too high above the target's altitude plane.

---

## Problem Definition

One pursuer drone must intercept one stationary target in unconstrained 3D space
(arena $[-8, 8]^3$ m, altitude floor 0.3 m).

**Pursuer**: 6-DOF point-mass with explicit yaw $\psi$ and pitch $\gamma$ states; PNG acceleration law.

**Target**: stationary at a position that differs from the pursuer in all three axes.

**Objective**: minimize time-to-intercept while operating within:
- maximum speed $v_{max}$ = 5.0 m/s
- maximum lateral (yaw-rate) acceleration $a_{\perp,max}$ = 6.0 m/s²
- maximum vertical acceleration $a_{z,max}$ = 4.0 m/s²
- sensor cone half-angle $\phi_{FOV}$ = 45° (altitude-dependent effective range)

**Guidance variants compared**:
1. **PD (baseline)**: original S001 implicit command — control group
2. **PNG-3D**: full 3D Proportional Navigation with $N = 3$ across all axes
3. **PNG-3D + vertical boost**: extra vertical acceleration when elevation angle $|\varepsilon| > 20°$
4. **PNG-3D + sensor-aware**: pursuer adjusts altitude to stay inside sensor detection cone

---

## Mathematical Model

### 6-DOF State

$$\mathbf{x} = [\mathbf{p}, \mathbf{v}, \psi, \gamma]^T, \quad \mathbf{p}, \mathbf{v} \in \mathbb{R}^3, \quad \psi \in (-\pi, \pi], \quad \gamma \in [-\pi/2, \pi/2]$$

Position update (Euler integration at frequency $f_c = 48$ Hz):

$$\mathbf{p}(t+dt) = \mathbf{p}(t) + \mathbf{v}(t) \cdot dt$$

Velocity update from body-frame acceleration decomposed into heading and climb components:

$$\mathbf{v}(t+dt) = \mathbf{v}(t) + \mathbf{a}_{cmd}(t) \cdot dt, \quad \|\mathbf{v}\| \leq v_{max}$$

### 3D Line-of-Sight Geometry

Range vector from pursuer to target:

$$\mathbf{r} = \mathbf{p}_T - \mathbf{p}_P, \qquad r = \|\mathbf{r}\|$$

Horizontal range and azimuth bearing to target:

$$r_{xy} = \sqrt{r_x^2 + r_y^2}, \qquad \psi_{los} = \text{atan2}(r_y,\, r_x)$$

Elevation angle (positive = target above pursuer):

$$\varepsilon = \text{atan2}(r_z,\, r_{xy})$$

LOS unit vector in 3D:

$$\hat{\mathbf{r}} = \frac{\mathbf{r}}{r}$$

### Proportional Navigation Guidance in 3D (PNG-3D)

LOS angular rate vector (3D cross product formulation):

$$\dot{\boldsymbol{\lambda}} = \frac{\mathbf{r} \times \dot{\mathbf{r}}}{r^2}, \quad \dot{\mathbf{r}} = \mathbf{v}_T - \mathbf{v}_P = -\mathbf{v}_P \text{ (target stationary)}$$

Closing speed:

$$V_c = -\dot{r} = -\frac{\mathbf{r} \cdot \dot{\mathbf{r}}}{r} = \frac{\mathbf{r} \cdot \mathbf{v}_P}{r}$$

PNG acceleration command (perpendicular to current velocity):

$$\mathbf{a}_{cmd} = N \cdot V_c \cdot \dot{\boldsymbol{\lambda}}, \quad N = 3$$

Decomposed into lateral (yaw-plane) and vertical (pitch-plane) components:

$$a_\perp = N \cdot V_c \cdot \dot{\lambda}_{xy}, \quad a_z = N \cdot V_c \cdot \dot{\lambda}_z$$

Clamped to physical limits:

$$a_\perp \leftarrow \text{clip}(a_\perp,\, -a_{\perp,max},\, a_{\perp,max})$$
$$a_z \leftarrow \text{clip}(a_z,\, -a_{z,max},\, a_{z,max})$$

### Explicit Yaw and Pitch Angle Update

Commanded yaw rate derived from lateral acceleration:

$$\dot{\psi}_{cmd} = a_\perp / \max(v,\, 0.1)$$

Commanded pitch rate from vertical acceleration:

$$\dot{\gamma}_{cmd} = a_z / \max(v,\, 0.1)$$

Angle update (first-order):

$$\psi(t+dt) = \psi(t) + \dot{\psi}_{cmd} \cdot dt, \qquad \gamma(t+dt) = \gamma(t) + \dot{\gamma}_{cmd} \cdot dt$$

### Vertical Intercept Geometry Boost

When the elevation angle to the target exceeds a threshold, an additional vertical acceleration command is added to reduce altitude error faster than PNG alone:

$$a_{z,boost} = K_{boost} \cdot r_z, \quad \text{if } |\varepsilon| > \varepsilon_{thresh} = 20°$$

Combined vertical command:

$$a_{z,total} = \text{clip}(a_z + a_{z,boost},\, -a_{z,max},\, a_{z,max})$$

### Altitude-Dependent Sensor Range

The pursuer carries a downward-facing sensor cone with half-angle $\phi_{FOV} = 45°$. Effective detection radius at altitude difference $\Delta z = z_P - z_T$:

$$R_{sense}(\Delta z) = |\Delta z| \cdot \tan(\phi_{FOV})$$

The sensor-aware guidance variant adds a vertical correction when the pursuer has climbed so high that $r_{xy} > R_{sense}$:

$$\Delta z_{corrective} = r_{xy} / \tan(\phi_{FOV}) - |\Delta z|$$

$$a_{z,sense} = K_{sense} \cdot \Delta z_{corrective}$$

This pulls the pursuer back into the sensor cone before continuing the intercept.

### Capture Condition

$$\|\mathbf{p}_P - \mathbf{p}_T\| < r_{capture} = 0.15 \text{ m}$$

---

## Key 3D Additions

- **Explicit yaw/pitch state**: $\psi$ and $\gamma$ updated each step from PNG acceleration commands — replaces the implicit velocity direction of S001
- **3D PNG formulation**: LOS rate computed via cross product in full 3D; vertical and lateral channels independently controlled
- **Elevation angle $\varepsilon$**: drives the vertical intercept geometry and the boost logic
- **Altitude-dependent sensor cone**: $R_{sense}(z) = |\Delta z| \tan(\phi_{FOV})$ — pursuer penalized for climbing above optimal sensing altitude
- **Guidance variant comparison**: four strategies benchmarked on the same initial conditions

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Pursuer start | (-2, -2, 1) m |
| Target position | (3, 2, 4) m |
| Max speed | 5.0 m/s |
| PNG navigation constant $N$ | 3 |
| Max lateral acceleration $a_{\perp,max}$ | 6.0 m/s² |
| Max vertical acceleration $a_{z,max}$ | 4.0 m/s² |
| Vertical boost gain $K_{boost}$ | 1.5 |
| Elevation threshold $\varepsilon_{thresh}$ | 20° |
| Sensor cone half-angle $\phi_{FOV}$ | 45° |
| Sensor correction gain $K_{sense}$ | 2.0 |
| Capture radius | 0.15 m |
| Altitude bounds | [0.3, 8.0] m |
| Control frequency | 48 Hz |
| Arena | $[-8, 8]^3$ m |

---

## Expected Output

- **3D trajectory plot**: all four guidance variants on shared axes; pursuer red, target green star; yaw/pitch arrows sampled every 0.5 s
- **Elevation angle vs time**: $\varepsilon(t)$ for each variant — shows how quickly each law drives the pursuer into vertical alignment
- **Altitude vs time**: $z_P(t)$ with the sensor-cone boundary $z_P = z_T + r_{xy}/\tan(\phi_{FOV})$ overlaid
- **Capture time bar chart**: PD baseline vs PNG-3D vs PNG-3D+boost vs PNG-3D+sensor-aware
- **Yaw/pitch angle time series**: $\psi(t)$ and $\gamma(t)$ illustrating the 6-DOF orientation history

---

## Extensions

1. Sweep navigation constant $N \in [2, 6]$ and report capture time sensitivity — in 3D the optimal $N$ may differ from the classical 2D value due to the vertical acceleration budget
2. Add a moving target with constant-velocity horizontal drift and test whether vertical intercept geometry degrades PNG performance compared to the 2D case
3. Model battery discharge as a function of vertical thrust: $\dot{E} = k_h \cdot a_z^2 + k \cdot v^2$; find the altitude profile that minimizes energy-to-intercept

---

## Related Scenarios

- Original 2D version: [S001](../S001_basic_intercept.md)
- Evasion in 3D: [S002 3D](S002_3d_evasive_maneuver.md)
- Low-altitude tracking (terrain + vertical): [S003](../S003_low_altitude_tracking.md)
