# S092 3D Upgrade — Movie Chase Scene

**Domain**: Special Ops & Entertainment | **Difficulty**: ⭐⭐⭐⭐ | **Status**: `[ ]` Not Started
**Based on**: [S092 original](../S092_movie_chase.md)

---

## What Changes in 3D

The original S092 already carries a fixed z-offset (`D_OFFSET[2] = 5.0 m`) that is added
directly as a world-frame constant — the Kalman filter and MPC state vectors are effectively
2D (x, y) with an appended altitude that never changes. The obstacle constraints are also
purely horizontal (`p_i^{xy}`), so the drone has no freedom — and no requirement — to use
vertical space for avoidance.

In a real urban-canyon film shoot, buildings, bridges, and signal gantries create a full 3D
obstacle volume. The car may also travel over elevated sections (ramps, flyovers) that change
its z-position over time. This upgrade extends:

1. **Kalman filter state** from 4-D (x, y, vx, vy) to 6-D (x, y, z, vx, vy, vz), enabling
   altitude prediction when the car climbs a ramp.
2. **Desired offset** from a fixed body-frame vector to a **3D cinematic offset** that keeps
   the camera-to-subject line-of-sight within the gimbal's pitch envelope at all times.
3. **MPC obstacle constraints** from horizontal half-planes to full 3D sphere-exclusion zones
   (cylindrical pillars become capped cylinders; bridge decks become box obstacles).
4. **Gimbal stabilisation model** that enforces a smooth camera pitch angle, preventing
   abrupt tilt changes that would ruin footage.
5. **Vertical evasion corridor**: when the horizontal path is blocked by a wall of pillars,
   the MPC is permitted to climb above them, subject to a maximum altitude ceiling.

---

## Problem Definition

**Setup**: A film production deploys a single camera drone to track a stunt car along a 300 m
urban-canyon route at 8 m/s. The route includes 5 concrete pillars arranged along the road
**and** one overpass bridge deck (a box obstacle, 3 m tall, 20 m wide) that the car passes
beneath. The drone must maintain a **3D cinematic offset** — dynamically adjusted to keep the
subject framed within the camera gimbal's safe pitch range of [-30°, +30°] — while navigating
the full 3D obstacle field. A 6-state Kalman Filter predicts the car's position and altitude
2 s ahead, feeding a 3D MPC that plans a 10-step receding-horizon trajectory.

**Roles**:
- **Car (target)**: follows a pre-scripted sinusoidal road that includes one altitude ramp
  (rises 2 m over 20 m, holds for 20 m, descends 2 m over 20 m); speed v = 8 m/s.
- **Camera drone**: executes KF-predicted 3D MPC trajectory to maintain the 3D desired offset
  and avoid all obstacles in 3D; carries a 2-axis gimbal (pitch + yaw) whose angular rate is
  jointly penalised in the MPC cost.

**Objective**: Maximise the fraction of mission time during which (a) the drone is within a
tolerance ball of the desired 3D offset position, (b) the car occupies 20–40 % of the camera
frame height, (c) the gimbal pitch rate stays below $\dot{\phi}_{max}$ = 30 deg/s, and
(d) the drone maintains minimum clearance $r_{safe}$ = 1.5 m from every obstacle surface.

---

## Mathematical Model

### Car Kinematics in 3D

The car's ground-plane sinusoidal trajectory is unchanged from S092:

$$x_{car}(t) = v\,t, \qquad y_{car}(t) = A\sin\!\left(\frac{2\pi v\,t}{L_{wave}}\right)$$

The altitude ramp adds a smooth vertical profile. Defining the arc-length parameter $s = v\,t$:

$$z_{car}(s) = h_{ramp} \cdot \sigma\!\left(\frac{s - s_1}{w_{ramp}}\right) - h_{ramp} \cdot \sigma\!\left(\frac{s - s_2}{w_{ramp}}\right)$$

where $\sigma(u) = 1/(1+e^{-u})$ is the logistic function, $h_{ramp} = 2.0$ m is the ramp
height, $s_1, s_2$ are the ramp start and end arc-lengths, and $w_{ramp} = 3.0$ m controls
the slope sharpness.

The full 3D heading frame uses the Frenet–Serret tangent:

$$\hat{\mathbf{t}}(t) = \frac{\dot{\mathbf{p}}_{car}}{\|\dot{\mathbf{p}}_{car}\|}, \qquad
R_{car}(t) = \begin{pmatrix} \hat{\mathbf{t}} & \hat{\mathbf{n}} & \hat{\mathbf{b}} \end{pmatrix}$$

where $\hat{\mathbf{n}}$ is the principal normal (pointing toward the road centre of curvature)
and $\hat{\mathbf{b}} = \hat{\mathbf{t}} \times \hat{\mathbf{n}}$ is the binormal.

### 6-State Kalman Filter

The 3D KF state vector adds vertical position and velocity:

$$\mathbf{x}_{KF} = \begin{pmatrix} x \\ y \\ z \\ v_x \\ v_y \\ v_z \end{pmatrix} \in \mathbb{R}^6$$

The constant-velocity process model generalises directly:

$$F = \begin{pmatrix} I_{3\times3} & \Delta t\, I_{3\times3} \\ 0_{3\times3} & I_{3\times3} \end{pmatrix} \in \mathbb{R}^{6\times6}$$

Process noise covariance with spectral density $q = 1.0$ m$^2$/s$^3$:

$$Q = q \begin{pmatrix} \tfrac{\Delta t^3}{3}I_3 & \tfrac{\Delta t^2}{2}I_3 \\ \tfrac{\Delta t^2}{2}I_3 & \Delta t\, I_3 \end{pmatrix}$$

The 3D measurement model uses GPS + barometric altitude fused observation:

$$H = \begin{pmatrix} I_{3\times3} & 0_{3\times3} \end{pmatrix}, \qquad
R_{meas} = \mathrm{diag}(\sigma_{xy}^2,\, \sigma_{xy}^2,\, \sigma_z^2)$$

with $\sigma_{xy} = 0.3$ m (GPS) and $\sigma_z = 0.1$ m (barometer).

Predicted car position $T_{pred} = 2$ s ahead:

$$\hat{\mathbf{p}}_{k+n}^{3D} = \left(F^n\, \hat{\mathbf{x}}_{k|k}\right)_{1:3}, \qquad n = \lfloor T_{pred}/\Delta t \rfloor$$

### 3D Desired Drone Position

The desired drone position maintains a body-frame offset that is additionally constrained to
keep the camera gimbal within its safe pitch envelope. Let $\mathbf{d}_{offset}$ be the
nominal offset in the car's Frenet frame:

$$\mathbf{p}_{d}(t) = \mathbf{p}_{car}(t) + R_{car}(t)\, \mathbf{d}_{offset}, \qquad
\mathbf{d}_{offset} = \begin{pmatrix} 0 \\ -8 \\ 5 \end{pmatrix}\ \text{m}$$

The **gimbal pitch angle** from drone to car is:

$$\phi_{gimbal}(t) = \arctan\!\left(\frac{p_{car,z}(t) - p_{drone,z}(t)}{\|\mathbf{p}_{car}^{xy}(t) - \mathbf{p}_{drone}^{xy}(t)\|}\right)$$

A soft altitude adjustment is applied when $|\phi_{gimbal}| > \phi_{max} = 30°$:

$$z_{d,\text{adj}}(t) = p_{car,z}(t) - \|\mathbf{p}_{car}^{xy} - \mathbf{p}_{drone}^{xy}\| \cdot \tan\phi_{max} \cdot \mathrm{sign}(\phi_{gimbal})$$

The final desired altitude is clamped:

$$z_d(t) = \mathrm{clip}\!\left(z_{d,\text{adj}}(t),\; z_{floor},\; z_{ceil}\right), \qquad z_{floor} = 1.0\ \text{m},\quad z_{ceil} = 12.0\ \text{m}$$

### Gimbal Pitch Rate Penalty

To produce cinematically smooth footage, the MPC penalises the time-derivative of gimbal
pitch angle. Defining $\phi_i$ at MPC step $i$:

$$\phi_i = \arctan\!\left(\frac{p_{car,z}(t_i) - p_{drone,z,i}}{\|\mathbf{p}_{car,i}^{xy} - \mathbf{p}_{drone,i}^{xy}\|}\right)$$

The rate penalty is approximated in the cost as:

$$J_{gimbal} = w_{\phi} \sum_{i=0}^{N-2} \left(\frac{\phi_{i+1} - \phi_i}{\Delta t_{mpc}}\right)^2$$

with weight $w_{\phi} = 2.0$.

### 3D MPC Formulation

The MPC horizon, timestep, and state/input structure are unchanged from S092 (N = 10,
$\Delta t_{mpc}$ = 0.2 s, double-integrator model). The cost function gains two additions:

$$J = \sum_{i=0}^{N-1} \Bigl( \|\mathbf{p}_i - \mathbf{p}_{d,i}\|^2_{Q_{mpc}} + \|\mathbf{u}_i\|^2_{R_{mpc}} + w_{\phi}(\phi_{i+1} - \phi_i)^2/\Delta t_{mpc}^2 \Bigr) + \|\mathbf{p}_N - \mathbf{p}_{d,N}\|^2_{P_f}$$

**3D obstacle constraints** replace the 2D half-planes:

For each cylindrical pillar $k$ with horizontal centre $\mathbf{o}_k^{xy}$, radius $r_k$,
and height $H_{pil}$, active when $p_{i,z} \leq H_{pil}$:

$$\hat{\mathbf{n}}_k^\top (\mathbf{p}_i^{xy} - \mathbf{o}_k^{xy}) \geq r_{safe}, \qquad
\hat{\mathbf{n}}_k = \frac{\mathbf{p}_0^{xy} - \mathbf{o}_k^{xy}}{\|\mathbf{p}_0^{xy} - \mathbf{o}_k^{xy}\|}$$

Above the pillar ($p_{i,z} > H_{pil}$) the constraint is relaxed, permitting the drone to
climb over the obstacle:

$$p_{i,z} \geq H_{pil} + z_{clearance}, \qquad z_{clearance} = 0.5\ \text{m}$$

For the bridge deck box obstacle (axis-aligned, defined by bounds $[x_a, x_b] \times [y_a, y_b]
\times [0, z_{deck}]$), the linearised constraint in the approach direction is:

$$n_{bridge} (p_{i,x} - x_{mid}) \geq d_{bridge}, \qquad n_{bridge} = \mathrm{sign}(p_{0,x} - x_{mid})$$

combined with the altitude override: if the drone is inside the x-y footprint, it must fly
either below $z_{deck} - z_{clearance}$ (following the car) or above $z_{deck} + z_{clearance}$
(overfly mode). The mode is selected once per planning cycle based on which altitude requires
the smaller $\Delta z$ from the current drone position.

**Velocity and acceleration bounds** gain a separate vertical axis limit:

$$|v_{i,z}| \leq v_{z,max} = 4.0\ \text{m/s}, \qquad |u_{i,z}| \leq a_{z,max} = 3.0\ \text{m/s}^2$$

$$\|\mathbf{v}_i^{xy}\|_\infty \leq v_{xy,max} = 12.0\ \text{m/s}, \qquad \|\mathbf{u}_i^{xy}\|_\infty \leq a_{xy,max} = 6.0\ \text{m/s}^2$$

### Camera Framing Metric in 3D

The 3D distance replaces the original 2D separation:

$$\theta_{frame}(t) = 2\arctan\!\left(\frac{h_{car}}{2\,\|\mathbf{p}_{drone}(t) - \mathbf{p}_{car}(t)\|}\right)$$

The gimbal-adjusted frame fraction accounts for the vertical look-down angle:

$$f_{height}(t) = \frac{\theta_{frame}(t)}{\theta_{FoV} \cos\phi_{gimbal}(t)}$$

The cosine factor corrects for the foreshortening that occurs when the camera pitches away
from horizontal — a detail absent in the flat-z original.

### Tracking Lag and Clearance Metrics

Identical to S092 but computed in full 3D:

$$\epsilon_{track} = \sqrt{\frac{1}{T}\int_0^T \|\mathbf{p}_{drone}(t) - \mathbf{p}_d(t)\|^2\,dt}$$

$$d_{clear} = \min_{t \in [0,T],\; k} \mathrm{dist}_{3D}(\mathbf{p}_{drone}(t),\, \mathrm{obstacle}_k)$$

where $\mathrm{dist}_{3D}$ is the signed minimum Euclidean distance to the obstacle surface
(cylinder lateral surface, top cap, or box face, whichever is closest).

---

## Key 3D Additions

- **6-state KF**: altitude and vertical velocity tracked and predicted, enabling anticipatory
  drone climb before the car reaches the ramp.
- **3D desired offset with gimbal envelope**: z-component of desired position adapts
  continuously so the camera pitch stays within [-30°, +30°], preventing footage blur from
  servo saturation.
- **Vertical evasion over pillars**: MPC selects over-fly mode automatically when the
  horizontal corridor is tighter than $r_{safe}$, using z as an additional degree of freedom.
- **Bridge deck box obstacle**: introduces a mode-switching altitude constraint — the drone
  must choose underfly (following the car) or overfly (abort cinematic shot, safety first).
- **Gimbal pitch rate penalty**: $w_{\phi}$ term in the MPC cost directly penalises rapid
  pitch changes, trading framing precision for visual smoothness.
- **Separate vertical dynamics limits**: $v_{z,max}$ = 4 m/s and $a_{z,max}$ = 3 m/s²
  reflect realistic multi-rotor vertical authority distinct from horizontal authority.

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Road length | 300 m |
| Car speed | 8.0 m/s |
| Road amplitude (A) | 15.0 m |
| Road wavelength (L_wave) | 80.0 m |
| Ramp height (h_ramp) | 2.0 m |
| Ramp slope width (w_ramp) | 3.0 m |
| Desired offset (behind / above) | 8.0 m / 5.0 m |
| Number of cylindrical pillars | 5 |
| Pillar height (H_pil) | 4.0 m |
| Pillar physical radius | 0.6 m |
| Obstacle inflation radius (r_safe) | 1.5 m |
| Bridge deck height (z_deck) | 3.0 m |
| Bridge deck x-span | 140–160 m |
| Bridge deck y-span | -20 to +20 m |
| Drone altitude bounds (z_floor / z_ceil) | 1.0 m / 12.0 m |
| KF measurement noise: xy / z | 0.3 m / 0.1 m |
| KF process noise density (q) | 1.0 m²/s³ |
| KF prediction horizon (T_pred) | 2.0 s |
| MPC horizon (N) | 10 steps |
| MPC timestep (dt_mpc) | 0.2 s |
| MPC position weight (Q_mpc) | 10 (per axis) |
| MPC control weight (R_mpc) | 0.1 (per axis) |
| Gimbal pitch rate weight (w_phi) | 2.0 |
| Drone max horizontal speed (v_xy_max) | 12.0 m/s |
| Drone max vertical speed (v_z_max) | 4.0 m/s |
| Drone max horizontal accel (a_xy_max) | 6.0 m/s² |
| Drone max vertical accel (a_z_max) | 3.0 m/s² |
| Gimbal safe pitch envelope | [-30°, +30°] |
| Gimbal max pitch rate | 30 deg/s |
| Camera vertical FoV | 60 deg |
| Target framing zone (f_height) | 20–40 % of frame |
| Car height for framing model | 1.5 m |

---

## Expected Output

- **3D trajectory plot**: car road (blue curve with altitude ramp visible), drone flight path
  (red curve), cylindrical pillars (grey), bridge deck (grey box), desired offset path (green
  dashed), over-fly segments highlighted in orange; full `mpl_toolkits.mplot3d` rendering.

- **Altitude time series**: $z_{car}(t)$, $z_{drone}(t)$, and $z_d(t)$ on the same axes;
  shaded band marks the bridge-deck passage; dashed lines at $z_{floor}$ and $z_{ceil}$.

- **Gimbal pitch angle time series**: $\phi_{gimbal}(t)$ in degrees; dashed bounds at ±30°;
  annotation of maximum pitch excursion and fraction of time within envelope.

- **Camera framing plot**: $f_{height}(t)$ vs time with 3D framing correction applied;
  shaded target zone [0.20, 0.40]; framing score $f_{score}$ annotated; comparison curve
  from the flat-z S092 result shown in grey for reference.

- **Obstacle clearance plot**: minimum 3D distance to nearest obstacle surface vs time; red
  dashed line at $r_{safe}$ = 1.5 m; bridge passage and over-fly event annotated.

- **Animation (GIF)**: dual-panel — top 3D perspective view (rotating azimuth) and bottom
  side-elevation view; car (blue), drone (red), desired position (green cross), KF prediction
  (cyan diamond), obstacles (grey); running framing score and gimbal pitch in title.

- **Summary metrics table** (printed to stdout):

  | Metric | Target |
  |--------|--------|
  | Framing score $f_{score}$ | >= 0.75 |
  | RMS tracking error | <= 2.0 m |
  | Min 3D obstacle clearance | >= 1.5 m |
  | Fraction of time gimbal within ±30° | >= 0.95 |
  | Max drone speed | <= 12.0 m/s |
  | Max vertical speed | <= 4.0 m/s |

---

## Extensions

1. **Dynamic vertical obstacles**: replace the static bridge with a lifting drawbridge whose
   deck rises at 0.5 m/s during the chase; the KF must track the deck altitude from a
   distance sensor, and the MPC must switch underfly/overfly mode with sufficient lead time.

2. **Wind-gust vertical disturbance**: inject a Dryden turbulence vertical gust field; the
   6-state KF augments with a disturbance integrator on $v_z$; evaluate altitude hold
   accuracy during bridge passage.

3. **Multi-drone 3D cinematic formation**: pair this drone with a second unit flying a lateral
   orbit (S086 style) at a different altitude; enforce 3D inter-drone separation $\geq$ 3 m
   via shared MPC coupling terms.

4. **Occlusion-aware 3D shot planning**: when a pillar passes between the drone and the car
   in 3D, the MPC detours vertically to restore line-of-sight; encode as a visibility cone
   constraint.

5. **Neural-network residual dynamics**: replace the double-integrator MPC plant model with a
   data-driven residual (trained on rotor downwash measurements near the bridge deck) to
   compensate the aerodynamic interaction between the deck and the rotors.

---

## Related Scenarios

- Original 2D version: [S092](../S092_movie_chase.md)
- 3D pursuit reference: [S003 Low-Altitude Tracking](../../01_pursuit_evasion/S003_low_altitude_tracking.md)
- Gimbal/framing cross-reference: [S081 Selfie Follow Mode](../S081_selfie_follow.md), [S086 Multi-Angle Cinema](../S086_multi_angle_cinema.md)
- Obstacle-course pursuit: [S004 Obstacle-Course Chase](../../01_pursuit_evasion/S004_obstacle_course_chase.md)
