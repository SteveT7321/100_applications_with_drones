# S094 3D Upgrade — Counter-Drone Intercept

**Domain**: Special Ops & Entertainment | **Difficulty**: ⭐⭐⭐⭐⭐ | **Status**: `[ ]` Not Started
**Based on**: [S094 original](../S094_counter_drone.md)

---

## What Changes in 3D

The original S094 is already nominally 3D (all state vectors are in $\mathbb{R}^3$), but the
scenario card and Monte Carlo study treat the altitude axis symmetrically with x and y — initial
positions are drawn uniformly from the full $50 \times 50 \times 20$ m airspace, yet there is no
altitude-specific intercept strategy, no altitude-band patrol for the target, and no vertical
component in the guidance comparison. This upgrade formalises and exploits the vertical axis in
four ways:

1. **Altitude-band patrol strategy for the target**: the rogue drone is no longer a random
   walker in z; it follows a scripted altitude pattern that mimics realistic rogue behaviour —
   nap-of-earth (NOE) flight at 2–4 m to exploit ground clutter, mid-band cruise at 8–12 m, and
   high-altitude dash above 15 m to outrun low-agility interceptors.
2. **3D intercept geometry**: closing geometry is decomposed into a horizontal (bearing) channel
   and a vertical (elevation) channel; each channel has its own LOS rate, enabling channel-specific
   acceleration budget allocation.
3. **3D Proportional Navigation with vertical bias**: the navigation constant $N$ is split into
   $N_{az}$ (azimuth channel) and $N_{el}$ (elevation channel), allowing the designer to tune
   vertical responsiveness independently of horizontal responsiveness.
4. **Multi-interceptor 3D assignment**: three interceptors are launched from boundary points at
   different altitudes; assignments are resolved by the 3D intercept time estimate (not 2D
   horizontal range), ensuring the interceptor with the best vertical intercept geometry is chosen.

---

## Problem Definition

**Setup**: A $50 \times 50 \times 20$ m restricted airspace contains a rogue drone (target) flying
an altitude-band patrol pattern. Three interceptor drones are stationed at distinct boundary
launch pads at altitudes $z_{pad} \in \{2, 10, 18\}$ m. On detection, all three compute their
individual time-to-intercept and the optimal assignment is solved in closed form (minimum-time
single assignment). The assigned interceptor applies **biaxial PNG**, while the two standby
interceptors maintain holding orbits. If the assigned interceptor fails to achieve capture within
15 s, the standby with the next-best time-to-intercept is re-assigned.

**Roles**:

- **Interceptors** ($v_I = 10$ m/s, $a_{I,max} = 15$ m/s², three agents): use 3D biaxial PNG;
  one active at a time, re-assignment triggered on timeout.
- **Target** ($v_T = 6$ m/s, $a_{T,max} = 8$ m/s²): follows an altitude-band patrol with
  Poisson-distributed mode switches; applies bang-bang evasion in the horizontal plane when an
  interceptor closes to within 15 m.

**Objective**: Capture the target — $\|\mathbf{p}_I - \mathbf{p}_T\| \leq r_{capture} = 0.5$ m —
within $T_{max} = 40$ s. A Monte Carlo study over $N_{MC} = 200$ trials compares:

1. **Single-interceptor biaxial PNG** (best initial assignment only, no re-assignment).
2. **Three-interceptor with re-assignment** (full multi-interceptor protocol).
3. **Three-interceptor with isotropic PNG** (standard $N_{az} = N_{el} = 4$, no vertical bias).

---

## Mathematical Model

### Coordinate Frame and State

All quantities are in an East-North-Up (ENU) inertial frame. The interceptor state vector is
$\mathbf{x}_I = [\mathbf{p}_I^\top, \mathbf{v}_I^\top]^\top \in \mathbb{R}^6$; similarly for the
target $\mathbf{x}_T$.

### 3D LOS Decomposition

The relative position vector and its scalar range:

$$\mathbf{r}_{IT} = \mathbf{p}_T - \mathbf{p}_I, \qquad R = \|\mathbf{r}_{IT}\|$$

Decompose into horizontal component (bearing channel) and vertical component (elevation channel):

$$R_h = \sqrt{\Delta x^2 + \Delta y^2}, \qquad R_v = |\Delta z|$$

The **LOS azimuth** and **LOS elevation** angles:

$$\lambda_{az} = \text{atan2}(\Delta y,\; \Delta x)$$

$$\lambda_{el} = \text{atan2}(\Delta z,\; R_h)$$

The unit LOS vector in 3D:

$$\hat{\mathbf{r}} = \frac{\mathbf{r}_{IT}}{R}$$

### LOS Angular Rate (Biaxial Decomposition)

The transverse LOS angular rate vector (component of relative velocity perpendicular to
$\hat{\mathbf{r}}$):

$$\boldsymbol{\Omega}_{LOS} = \frac{\mathbf{v}_{rel}}{R} - \frac{(\mathbf{v}_{rel} \cdot \hat{\mathbf{r}})}{R}\,\hat{\mathbf{r}}$$

where $\mathbf{v}_{rel} = \mathbf{v}_T - \mathbf{v}_I$.

Decompose $\boldsymbol{\Omega}_{LOS}$ into azimuth and elevation components by projecting onto the
horizontal-normal and vertical unit vectors of the LOS frame:

$$\hat{\mathbf{n}}_{az} = \frac{\hat{\mathbf{z}} \times \hat{\mathbf{r}}}{\|\hat{\mathbf{z}} \times \hat{\mathbf{r}}\|}, \qquad
\hat{\mathbf{n}}_{el} = \hat{\mathbf{r}} \times \hat{\mathbf{n}}_{az}$$

$$\dot{\lambda}_{az} = \boldsymbol{\Omega}_{LOS} \cdot \hat{\mathbf{n}}_{az}, \qquad
\dot{\lambda}_{el} = \boldsymbol{\Omega}_{LOS} \cdot \hat{\mathbf{n}}_{el}$$

### Closing Speed

$$V_c = -\dot{R} = -\frac{\mathbf{r}_{IT} \cdot \mathbf{v}_{rel}}{R}$$

### Biaxial Proportional Navigation (3D)

The navigation commands for each channel use independent navigation constants
$N_{az}$ and $N_{el}$:

$$\mathbf{a}_{az} = N_{az} \cdot V_c \cdot \dot{\lambda}_{az} \cdot \hat{\mathbf{n}}_{az}$$

$$\mathbf{a}_{el} = N_{el} \cdot V_c \cdot \dot{\lambda}_{el} \cdot \hat{\mathbf{n}}_{el}$$

The total PNG command is the vector sum:

$$\mathbf{a}_{I,cmd} = \mathbf{a}_{az} + \mathbf{a}_{el}$$

Saturated to the interceptor's acceleration budget:

$$\mathbf{a}_{I} = \mathbf{a}_{I,cmd} \cdot \min\!\left(1,\; \frac{a_{I,max}}{\|\mathbf{a}_{I,cmd}\|}\right)$$

With $N_{az} = 4$ and $N_{el} = 5$, the elevation channel receives a higher gain to compensate
for the target's altitude-band strategy placing it in a geometrically difficult vertical position
relative to boundary-launched interceptors.

### Zero-Effort Miss in 3D

The 3D ZEM vector predicts the positional error if both vehicles hold current velocities for the
time-to-go $t_{go} = R / (V_c + \varepsilon)$:

$$\mathbf{ZEM} = \mathbf{r}_{IT} + \mathbf{v}_{rel}\,t_{go}, \qquad ZEM = \|\mathbf{ZEM}\|$$

Biaxial ZEM components:

$$ZEM_{az} = \mathbf{ZEM} \cdot \hat{\mathbf{n}}_{az}, \qquad ZEM_{el} = \mathbf{ZEM} \cdot \hat{\mathbf{n}}_{el}$$

### Altitude-Band Patrol Target Model

The target altitude is governed by a finite-state machine with three bands:

| Band | Altitude range | Label |
|------|---------------|-------|
| NOE | $z \in [1,\; 4]$ m | nap-of-earth |
| MID | $z \in [7,\; 13]$ m | mid-band cruise |
| HIGH | $z \in [15,\; 19]$ m | high-altitude dash |

At each mode-switch event (Poisson, mean $\bar{\tau} = 4$ s) the target selects a new band
uniformly at random and commands a vertical velocity:

$$v_{z,cmd} = K_z \cdot (z_{target,band} - z_T), \qquad K_z = 1.5 \text{ s}^{-1}$$

where $z_{target,band}$ is the centre of the newly selected band. Horizontal evasion remains
bang-bang (as in S094) when $R < 15$ m:

$$\hat{\mathbf{e}}_{bang} = \frac{\hat{\mathbf{r}}_{IT,h} \times \hat{\mathbf{z}}}{\|\hat{\mathbf{r}}_{IT,h} \times \hat{\mathbf{z}}\|}$$

$$\mathbf{a}_{T,h} = a_{T,max} \cdot \text{sign}_{random} \cdot \hat{\mathbf{e}}_{bang}$$

where $\hat{\mathbf{r}}_{IT,h}$ is the horizontal projection of $\hat{\mathbf{r}}_{IT}$.

### Multi-Interceptor 3D Assignment

For $n_I = 3$ interceptors indexed $i \in \{1,2,3\}$, the estimated time-to-intercept uses a
linear closing model:

$$\hat{t}_{go,i} = \frac{\|\mathbf{p}_T(t_0) - \mathbf{p}_I^{(i)}(t_0)\|}{V_{c,i}^+ + \varepsilon}$$

where $V_{c,i}^+ = \max(V_{c,i},\; 0.1)$ prevents assignment of diverging interceptors.

The optimal assignment selects:

$$i^* = \arg\min_{i \in \{1,2,3\}} \hat{t}_{go,i}$$

Re-assignment is triggered at time $t_{re}$ when the active interceptor's ZEM exceeds
$ZEM_{threshold} = 3$ m with $t_{go} < 5$ s, or when $t_{re} - t_{assign} > 15$ s. The
new assignment excludes the currently failing interceptor:

$$i^{**} = \arg\min_{i \in \{1,2,3\} \setminus \{i^*\}} \hat{t}_{go,i}(t_{re})$$

### Capture Condition and Monte Carlo Metrics

Capture is declared when:

$$\|\mathbf{p}_I(t) - \mathbf{p}_T(t)\| \leq r_{capture} = 0.5 \text{ m}$$

Failure when $t > T_{max} = 40$ s. Over $N_{MC} = 200$ trials per strategy:

$$\text{Success rate} = \frac{N_{success}}{N_{MC}} \times 100\%$$

$$\bar{T}_{cap} = \mathbb{E}[T_{cap} \mid \text{success}], \qquad \bar{d}_{miss} = \mathbb{E}[d_{miss} \mid \text{failure}]$$

$$d_{miss} = \min_{t \in [0, T_{max}]} \|\mathbf{p}_I(t) - \mathbf{p}_T(t)\|$$

---

## Key 3D Additions

- **Altitude-band patrol**: target follows a three-band FSM (NOE / MID / HIGH) with proportional
  altitude-rate commands, replacing the uniform random z initialisation of S094.
- **Biaxial PNG**: navigation constants $N_{az}$ and $N_{el}$ are decoupled, allowing
  independent tuning of horizontal and vertical PNG gains to counter altitude-based evasion.
- **LOS frame decomposition**: $\hat{\mathbf{n}}_{az}$ and $\hat{\mathbf{n}}_{el}$ are derived
  analytically from the instantaneous 3D LOS, providing a geometrically correct channel split.
- **3D assignment metric**: interceptor assignment uses full 3D range and closing speed, not a
  2D horizontal approximation.
- **Re-assignment protocol**: a ZEM-based trigger allows the coordinator to replace a failing
  interceptor mid-engagement without requiring centralised communication.

---

## Key Parameters

| Parameter | Symbol | Value |
|-----------|--------|-------|
| Airspace dimensions | — | 50 × 50 × 20 m |
| Interceptor cruise speed | $v_I$ | 10 m/s |
| Target cruise speed | $v_T$ | 6 m/s |
| Interceptor max accel | $a_{I,max}$ | 15 m/s² |
| Target max horizontal accel | $a_{T,max}$ | 8 m/s² |
| Azimuth navigation constant | $N_{az}$ | 4 |
| Elevation navigation constant | $N_{el}$ | 5 |
| Capture radius | $r_{capture}$ | 0.5 m |
| Mission time budget | $T_{max}$ | 40 s |
| Simulation timestep | $\Delta t$ | 0.02 s (50 Hz) |
| Altitude-band switch mean | $\bar{\tau}$ | 4 s |
| Altitude vertical gain | $K_z$ | 1.5 s⁻¹ |
| NOE band | — | 1 – 4 m |
| MID band | — | 7 – 13 m |
| HIGH band | — | 15 – 19 m |
| Bang-bang trigger range | — | 15 m |
| Re-assignment ZEM threshold | $ZEM_{threshold}$ | 3 m |
| Re-assignment time limit | — | 15 s |
| Number of interceptors | $n_I$ | 3 |
| Launch pad altitudes | $z_{pad}$ | 2, 10, 18 m |
| Monte Carlo trials per strategy | $N_{MC}$ | 200 |

---

## Expected Output

- **3D trajectory plot** (one panel per strategy, three panels): interceptor paths in red, target
  path in blue; launch pads marked as grey squares at boundary; intercept point as a green star;
  airspace box outline in light grey; altitude-band boundaries drawn as horizontal grey planes at
  $z \in \{4, 7, 13, 15\}$ m; target altitude-band transitions visible as vertical path kinks.
- **Altitude time series**: $z_I(t)$ (red) and $z_T(t)$ (blue) on the same axes for a
  representative biaxial-PNG trial; altitude band boundaries as horizontal dashed lines; band
  labels (NOE, MID, HIGH) annotated in the margins; shows how the interceptor tracks the
  target's vertical excursions.
- **Biaxial ZEM time series**: two stacked panels showing $ZEM_{az}(t)$ and $ZEM_{el}(t)$
  for the biaxial-PNG sample trial; elevation ZEM decays faster with $N_{el} = 5$; re-assignment
  event (if any) marked with a vertical dashed line.
- **Monte Carlo summary**: left panel — capture-time CDF for all three strategies (biaxial
  single red, multi-interceptor biaxial green, isotropic multi blue); right panel — miss-distance
  histogram for failed trials; expected result: multi-interceptor biaxial PNG achieves the
  highest success rate and lowest mean miss distance.
- **Assignment event log** (text/console): for each re-assignment event across the Monte Carlo
  run, print the trial index, time of re-assignment, ZEM at re-assignment, and new interceptor
  index; aggregate counts of re-assignment events per strategy.

---

## Extensions

1. **Augmented biaxial PNG (A-BPNG)**: add an estimated target acceleration term per channel —
   $\mathbf{a}_{el,aug} = N_{el} V_c \dot{\lambda}_{el} \hat{\mathbf{n}}_{el} + \hat{a}_{T,z}$
   where $\hat{a}_{T,z}$ is estimated from an alpha-beta filter on the elevation channel LOS rate;
   compare A-BPNG capture rate against biaxial PNG under full altitude-band evasion.
2. **Optimal elevation gain sweep**: vary $N_{el} \in [2, 8]$ in steps of 0.5 and plot the
   success-rate curve against $N_{el}$; identify the value that maximises success rate without
   causing acceleration saturation in the elevation channel.
3. **Terrain-aware NOE evasion**: add a ground-level terrain map with obstacles up to 3 m
   height; constrain the target's NOE flight to stay within 1 m of the terrain surface; evaluate
   how terrain masking reduces interceptor success rate.
4. **EKF-based altitude estimation**: replace perfect target altitude knowledge with a noisy
   radar sensor ($\sigma_\rho = 0.2$ m, $\sigma_{el} = 0.5°$); feed a 3D Extended Kalman Filter;
   measure the degradation in success rate attributable to elevation estimation error relative
   to azimuth estimation error.
5. **Variable interceptor launch altitudes**: instead of fixed $z_{pad}$, optimise the three
   launch altitudes over $[1, 19]$ m to maximise expected success rate against a mixed
   NOE/MID/HIGH patrol target; formulate as a 3-variable Bayesian optimisation.
6. **Net-capture with vertical alignment**: extend the physical capture model from S094 Extension 6
   by adding a constraint that the interceptor's elevation angle to the target must be within 10°
   of horizontal at net deployment (nets are less effective in steep-angle engagements).

---

## Related Scenarios

- Original 2D/3D version: [S094](../S094_counter_drone.md)
- Truly 3D pursuit references: [S001 Basic Intercept](../../01_pursuit_evasion/S001_basic_intercept.md),
  [S003 Low Altitude Tracking](../../01_pursuit_evasion/S003_low_altitude_tracking.md)
- Multi-agent assignment: [S018 Multi-Target Intercept](../../01_pursuit_evasion/S018_multi_target_intercept.md)
- Differential game foundation: [S009 Differential Game](../../01_pursuit_evasion/S009_differential_game.md)
- Domain 5 follow-ups: [S092 Autonomous Combat Drone](../S092_autonomous_combat.md),
  [S099 Electronic Warfare Drone](../S099_obstacle_relay.md)
