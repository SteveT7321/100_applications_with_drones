# S060 3D Upgrade — Meteorological Profiling

**Domain**: Environmental & SAR | **Difficulty**: ⭐⭐⭐⭐ | **Status**: `[ ]` Not Started
**Based on**: [S060 original](../S060_meteorological.md)

---

## What Changes in 3D

The original S060 assigns each drone a fixed horizontal altitude band and covers it with a
boustrophedon lawnmower that sweeps $(x, y)$ at discrete $z$ levels. Altitude is treated as
a discrete band index, not a continuously controlled state: $z$ increments in fixed steps
between sweep layers and there is no feedback between current $z$ position and the guidance
law. Horizontal wind drift during ascent and descent is not modelled, so sample locations are
assumed to sit exactly on the planned waypoint grid.

This 3D upgrade introduces three structural changes:

1. **Continuous vertical profiling per drone**: each drone executes a true helical ascent
   where $z(t)$ is a continuously controlled state driven by a climb-rate command. Temperature
   $T$, relative humidity $\mathrm{RH}$, pressure $P$, and horizontal wind magnitude $U_w$
   are all sampled as a function of altitude along the real (drifted) flight path, not on an
   ideal grid.

2. **Horizontal drift correction during climb**: at each altitude step the drone measures
   the local horizontal wind vector $\mathbf{U}_w(z) = (U_x, U_y)$ and applies a feedforward
   compensation velocity to cancel lateral displacement before the next sample point is reached.

3. **Multi-drone simultaneous vertical and horizontal profiling**: rather than assigning each
   drone to a non-overlapping altitude band, drones are distributed across the horizontal
   domain and each climbs the full altitude column $[z_{min}, z_{max}]$ simultaneously,
   so that a 3D snapshot of the atmospheric state at a single moment is obtained rather than
   a time-sequential patchwork across three bands.

---

## Problem Definition

**Setup**: Four drones are deployed at the corners of a $600 \times 600$ m horizontal
domain. Each drone performs a continuous helical ascent from $z_{min} = 20$ m to
$z_{max} = 500$ m while measuring temperature $T$, relative humidity $\mathrm{RH}$,
barometric pressure $P$, and horizontal wind vector $\mathbf{U}_w$ at every time step.
A synthetic 3D atmospheric state is defined by smooth analytical fields for all four
variables. During ascent, the local wind field imparts a time-varying horizontal drift
on each drone; the onboard guidance law applies a feedforward correction to maintain
the planned helix track. After all drones complete their simultaneous ascents, the
ground station fuses all 3D sample sets via anisotropic Ordinary Kriging (with separate
horizontal and vertical range parameters) to produce volumetric estimates of all four
fields. A held-out validation set evaluates reconstruction quality per variable.

**Roles**:
- **Drone 1** (position: $(0, 0)$ m): climbs the north-west atmospheric column; primary
  sensor suite for temperature and relative humidity.
- **Drone 2** (position: $(600, 0)$ m): climbs the north-east column; sensors for
  temperature and wind vector.
- **Drone 3** (position: $(600, 600)$ m): climbs the south-east column; sensors for
  temperature and pressure.
- **Drone 4** (position: $(0, 600)$ m): climbs the south-west column; sensors for
  temperature, humidity, and wind vector.
- **Ground station**: receives streaming telemetry from all four drones, runs the
  anisotropic Kriging pipeline, and generates 3D volumetric reconstructions.

**Objective**: Execute simultaneous full-column helical ascents with real-time horizontal
drift correction, collect multi-variable atmospheric profiles, reconstruct all four
3D fields via anisotropic Ordinary Kriging, and evaluate reconstruction RMSE for each
variable against ground truth.

---

## Mathematical Model

### Synthetic 3D Atmospheric Fields

All four fields are smooth analytical functions of $\mathbf{x} = (x, y, z)$.

**Temperature** (dry adiabatic lapse with horizontal thermal structure):

$$T(x, y, z) = T_0 - \Gamma_d z + A_T \sin\!\left(\frac{2\pi x}{L_x}\right)\cos\!\left(\frac{2\pi y}{L_y}\right)e^{-z/H_T}$$

**Relative humidity** (surface-maximum, exponentially decaying with altitude, modulated by
a low-level moisture convergence zone):

$$\mathrm{RH}(x, y, z) = \mathrm{RH}_0 \cdot e^{-z/H_{RH}}
    + A_{RH}\cos\!\left(\frac{2\pi x}{L_x}\right)\sin\!\left(\frac{2\pi y}{L_y}\right)e^{-z/H_T}$$

**Barometric pressure** (hydrostatic column, referenced to mean sea level):

$$P(x, y, z) = P_0 \left(1 - \frac{\Gamma_d z}{T_0}\right)^{g / (R_d \Gamma_d)}$$

where $g = 9.81\ \mathrm{m\,s^{-2}}$ is gravitational acceleration and
$R_d = 287\ \mathrm{J\,kg^{-1}\,K^{-1}}$ is the dry-air gas constant.

**Horizontal wind magnitude** (jet-like structure peaking at mid-altitude):

$$U_w(x, y, z) = U_{ref} \cdot \frac{z}{H_w} e^{1 - z/H_w}
    + A_U \sin\!\left(\frac{2\pi x}{L_x}\right)\cos\!\left(\frac{2\pi y}{L_y}\right)$$

with $H_w = 200\ \mathrm{m}$ (height of wind-speed maximum) and $U_{ref} = 8\ \mathrm{m\,s^{-1}}$.

Additive Gaussian sensor noise is applied to all measurements:

$$\tilde{f}(\mathbf{x}_i) = f(\mathbf{x}_i) + \epsilon_i, \qquad \epsilon_i \sim \mathcal{N}(0, \sigma_f^2)$$

### Helical Ascent Trajectory

Each drone follows a helical path parameterised by continuous time $t$. The planned
position of drone $d$ is:

$$\mathbf{p}_d^{plan}(t) = \begin{bmatrix}
    x_d^0 + R_h \cos(\omega_h t + \phi_d) \\
    y_d^0 + R_h \sin(\omega_h t + \phi_d) \\
    z_{min} + \dot{z}_{cmd}\, t
\end{bmatrix}$$

where $(x_d^0, y_d^0)$ is the horizontal anchor of drone $d$, $R_h$ is the helix radius,
$\omega_h = 2\pi / T_{rev}$ is the angular rate for one horizontal revolution, $\phi_d$
is the initial phase offset, and $\dot{z}_{cmd}$ is the commanded climb rate. The helix
pitch $\alpha$ is:

$$\tan\alpha = \frac{\dot{z}_{cmd}}{R_h \omega_h}$$

Total ascent time: $T_{asc} = (z_{max} - z_{min}) / \dot{z}_{cmd}$.

### Horizontal Wind Drift

At altitude $z$, the local horizontal wind vector is:

$$\mathbf{U}_w(z) = U_w(x, y, z)\,\hat{\mathbf{u}}(z)$$

where $\hat{\mathbf{u}}(z)$ is the wind direction (assumed constant in this model,
rotating slowly with altitude at rate $\psi_w$ rad/m):

$$\hat{\mathbf{u}}(z) = \bigl(\cos(\psi_0 + \psi_w z),\ \sin(\psi_0 + \psi_w z)\bigr)$$

The resulting drift displacement accumulated over time step $\Delta t$ is:

$$\Delta \mathbf{p}_{drift}(t) = \mathbf{U}_w(z(t)) \cdot \Delta t$$

### Feedforward Drift Correction

To keep the drone on its planned helix track, the guidance law applies a correction
velocity equal and opposite to the predicted drift at each step. The commanded velocity
is the sum of the trajectory-following component and the drift compensation:

$$\mathbf{v}_{cmd}(t) = \underbrace{\dot{\mathbf{p}}_d^{plan}(t)}_{\text{helix tangent}}
    - \underbrace{\mathbf{U}_w(z(t))}_{\text{wind cancel}}$$

The correction is feedforward only; a proportional position-error feedback term is added
to handle residual tracking error from finite time-step integration:

$$\mathbf{v}_{cmd}(t) = \dot{\mathbf{p}}_d^{plan}(t) - \mathbf{U}_w(z(t))
    + K_p \bigl(\mathbf{p}_d^{plan}(t) - \mathbf{p}_d(t)\bigr)$$

with position-feedback gain $K_p = 1.0\ \mathrm{s^{-1}}$.

Total airspeed requirement check:

$$v_{air}(t) = \|\mathbf{v}_{cmd}(t)\| \leq v_{max}$$

If $v_{air} > v_{max}$, the climb rate is reduced proportionally to keep the drone within
its speed envelope.

### Horizontal Tracking Error

The horizontal displacement of the actual flight path from the planned helix centreline
is the key quality metric for drift correction:

$$e_{track}(t) = \left\|\bigl(\mathbf{p}_d(t) - \mathbf{p}_d^{plan}(t)\bigr)_{xy}\right\|$$

### Anisotropic Variogram Model

Atmospheric fields have different spatial correlation lengths in the horizontal
$(r_{xy})$ and vertical $(r_z)$ directions. The anisotropic semivariogram is computed
using a scaled distance metric:

$$h_{aniso}(i, j) = \sqrt{(x_i - x_j)^2 + (y_i - y_j)^2 + \left(\frac{z_i - z_j}{s_z}\right)^2}$$

where $s_z = r_{xy} / r_z$ is the anisotropy ratio that stretches the vertical axis so
that the isotropic exponential variogram formula can be applied:

$$\gamma(h_{aniso}) = C_0 + C_1\!\left(1 - e^{-h_{aniso}/a}\right)$$

Parameters $(C_0, C_1, a, s_z)$ are estimated by non-linear least squares over the
experimental semivariogram computed from all sample pairs across all four drones.

### Ordinary Kriging System (Anisotropic)

The Kriging estimator and its unbiasedness constraint are identical to S060 in form,
but use $h_{aniso}$ throughout:

$$\begin{bmatrix} \mathbf{\Gamma} & \mathbf{1} \\ \mathbf{1}^T & 0 \end{bmatrix}
\begin{bmatrix} \boldsymbol{\lambda} \\ \mu \end{bmatrix}
= \begin{bmatrix} \boldsymbol{\gamma}_0 \\ 1 \end{bmatrix}$$

$$\hat{f}(\mathbf{x}_0) = \sum_{i=1}^{N} \lambda_i\, \tilde{f}(\mathbf{x}_i), \qquad
\sigma^2_K(\mathbf{x}_0) = \sum_{i=1}^{N} \lambda_i\,\gamma(h_{aniso,i0}) + \mu$$

### Multi-Variable Reconstruction Quality

Reconstruction RMSE is evaluated independently for each field variable $f \in
\{T, \mathrm{RH}, P, U_w\}$ over $N_{val}$ held-out validation points:

$$\mathrm{RMSE}_f = \sqrt{\frac{1}{N_{val}} \sum_{j=1}^{N_{val}}
    \left[\hat{f}(\mathbf{x}_j^{val}) - f(\mathbf{x}_j^{val})\right]^2}$$

The simultaneous multi-drone profiling advantage is quantified by comparing the mean
Kriging variance $\bar{\sigma}^2_K$ for the 3D snapshot approach against the
time-sequential band-by-band approach of S060 (where samples from different bands
represent the atmosphere at different times).

---

## Key 3D Additions

- **Full vertical profile**: $T$, $\mathrm{RH}$, $P$, and $U_w$ all sampled as continuous
  functions of altitude along the real drone path, not on a discrete altitude grid.
- **Horizontal drift correction**: feedforward wind cancellation plus proportional
  position-error feedback maintains helix track to within $e_{track} < 2$ m even in
  $U_w = 8\ \mathrm{m\,s^{-1}}$ crosswind conditions.
- **Anisotropic Kriging**: separate horizontal and vertical correlation length scales
  $(r_{xy} \gg r_z$ for temperature; $r_{xy} \approx r_z$ for wind) via a stretching
  anisotropy ratio $s_z$.
- **Simultaneous 3D snapshot**: four drones climb concurrently so all samples
  correspond to the same atmospheric state, eliminating temporal aliasing across bands.
- **Multi-variable fusion**: four variables reconstructed from the same spatial sample
  set, enabling physical consistency checks (e.g., $T$-$\mathrm{RH}$ anti-correlation,
  hydrostatic consistency of $P(z)$).

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Number of drones $N$ | 4 |
| Horizontal domain $X \times Y$ | $600 \times 600$ m |
| Drone anchor positions | $(0,0)$, $(600,0)$, $(600,600)$, $(0,600)$ m |
| Altitude range $[z_{min}, z_{max}]$ | 20 – 500 m |
| Helix radius $R_h$ | 30 m |
| Helix revolution period $T_{rev}$ | 60 s |
| Commanded climb rate $\dot{z}_{cmd}$ | 1.6 m/s |
| Total ascent time $T_{asc}$ | 300 s |
| Helix pitch angle $\alpha$ | $\approx 3°$ |
| Max airspeed $v_{max}$ | 12 m/s |
| Position-feedback gain $K_p$ | 1.0 s$^{-1}$ |
| Reference wind speed $U_{ref}$ | 8 m/s |
| Wind direction rotation rate $\psi_w$ | $0.01$ rad/m |
| Sensor noise: temperature $\sigma_T$ | 0.3 K |
| Sensor noise: humidity $\sigma_{RH}$ | 1.5 % |
| Sensor noise: pressure $\sigma_P$ | 0.5 hPa |
| Sensor noise: wind speed $\sigma_U$ | 0.4 m/s |
| Surface reference temperature $T_0$ | 288 K |
| Dry adiabatic lapse rate $\Gamma_d$ | 6.5 $\times$ 10$^{-3}$ K/m |
| Thermal perturbation amplitude $A_T$ | 2 K |
| Thermal decay scale height $H_T$ | 300 m |
| Humidity surface value $\mathrm{RH}_0$ | 80 % |
| Humidity decay scale height $H_{RH}$ | 200 m |
| Surface pressure $P_0$ | 1013.25 hPa |
| Wind-speed peak height $H_w$ | 200 m |
| Anisotropy ratio $s_z$ (temperature) | 5.0 |
| Variogram model | Anisotropic exponential |
| Held-out validation points $N_{val}$ | 300 |
| Expected RMSE: temperature | $< 0.4$ K |
| Expected RMSE: humidity | $< 2.0$ % |
| Expected RMSE: pressure | $< 1.0$ hPa |
| Expected RMSE: wind speed | $< 0.6$ m/s |

---

## Expected Output

- **3D helix trajectory plot**: four simultaneous helical ascents visualised in 3D space,
  each drone in a distinct colour; planned helix centreline shown as a dashed reference;
  horizontal drift correction vectors shown as short arrows at sampled altitudes.
- **Horizontal tracking error vs altitude**: $e_{track}(z)$ for each drone, demonstrating
  that feedforward wind cancellation keeps the drone within 2 m of its planned track;
  comparison curve without drift correction showing accumulated lateral displacement.
- **Vertical profile plots**: altitude on the y-axis; measured $T$, $\mathrm{RH}$, $P$,
  and $U_w$ on four side-by-side subplots; all four drone profiles overlaid with the
  analytical ground-truth curve; sensor noise scatter visible.
- **Anisotropic variogram fits**: experimental semivariogram computed using anisotropic
  lag distance $h_{aniso}$; fitted exponential model overlaid; nugget, sill, and range
  annotated; separate panels for each of the four field variables.
- **3D reconstructed fields**: three orthogonal cross-sections at $x = 300$ m,
  $y = 300$ m, $z = 200$ m for each variable; diverging colormaps; helix sample
  locations projected onto the nearest slice.
- **3D Kriging variance maps**: same cross-section layout showing $\sigma^2_K$; regions
  near the four helical columns show low variance; the horizontal interior (far from any
  drone) shows elevated variance, quantifying the spatial information gap.
- **RMSE comparison bar chart**: side-by-side bars for each of the four variables comparing
  the 3D simultaneous snapshot approach (this scenario) against the S060 time-sequential
  band assignment approach; expected 10–20% RMSE reduction due to temporal consistency.
- **Validation scatter plots**: one panel per variable showing $\hat{f}$ vs. true $f$ at
  the 300 held-out points; diagonal reference line; RMSE and $R^2$ annotated.

---

## Extensions

1. **Adaptive helix radius vs altitude**: reduce $R_h$ in the boundary layer (strong
   shear, high gradients) and increase $R_h$ in the free atmosphere (smooth field,
   wide correlation length); optimise $R_h(z)$ to minimise total Kriging variance for
   a fixed flight-time budget.
2. **Online multi-variable co-Kriging**: use cross-variograms to capture the physical
   $T$–$\mathrm{RH}$ anti-correlation and $P$–$T$ hydrostatic coupling; derive the
   co-Kriging system and show RMSE reduction over independent per-variable Kriging.
3. **Wind estimation from drift telemetry**: invert the observed drone drift (difference
   between GPS trajectory and commanded helix) to estimate $\mathbf{U}_w(z)$ without
   a dedicated anemometer; assess wind-vector RMSE from trajectory inversion alone.
4. **Time-varying wind field**: introduce a slowly evolving background wind that advects
   horizontally at $1\ \mathrm{m\,s^{-1}}$; apply space-time Kriging with an
   exponential temporal variogram to fuse profiles taken at slightly different times;
   evaluate 10-minute forecast accuracy.
5. **Drone failure and replanning**: simulate loss of one drone mid-ascent; replan the
   remaining three drones to cover the missing column by widening their helix radii
   and adding a lateral excursion; quantify RMSE degradation and recovery time.

---

## Related Scenarios

- Original 2D version: [S060](../S060_meteorological.md)
- Prerequisites: [S048 Lawnmower Coverage](../S048_lawnmower.md),
  [S045 Chemical Plume Tracing](../S045_plume_tracing.md)
- Follow-ups: [S058 Typhoon Eye Probing](../S058_typhoon.md)
- Algorithmic cross-reference: [S046 Multi-Drone 3D Trilateration](../S046_trilateration.md),
  [S052 Glacier Surface Mapping](../S052_glacier.md)
