# S055 3D Upgrade — Oil Spill Tracking

**Domain**: Environmental & SAR | **Difficulty**: ⭐⭐⭐⭐ | **Status**: `[ ]` Not Started
**Based on**: [S055 original](../S055_oil_spill.md)

---

## What Changes in 3D

The original S055 locks both drones at a constant altitude $z = 15$ m and models the oil concentration field as a purely 2D surface scalar $C(x, y, t)$. Three key limitations follow from this flat-world assumption:

1. **Sensor footprint is altitude-independent.** A real hyperspectral or fluorescence sensor has a nadir-facing field of view whose ground footprint scales with altitude; the detection threshold for a given oil thickness also shifts with the standoff distance because of spectral path-length attenuation.
2. **Sea-state wave tilt is ignored.** Ocean surface waves tilt the air-water interface; a drone flying at fixed altitude experiences variable view angles across a wave field, which modulates the apparent reflectance and can produce false boundary detections.
3. **Spill thickness is unobservable.** Surface oil forms sheens of varying thickness (rainbow sheen < 0.0001 mm through thick brown oil > 0.1 mm). Different thicknesses exhibit different fluorescence intensities. A single-altitude pass cannot separate areal extent from thickness variation; two altitude layers provide an overdetermined system that allows simultaneous estimation of both.

This variant raises the drones into a dual-altitude patrol regime: one drone holds a low-altitude strip ($z_L = 10$ m) for fine-boundary delineation; the second holds a high-altitude strip ($z_H = 30$ m) for wide-area thickness mapping. The boundary Kalman filter is extended from 2D position tracking to a 3D state that includes altitude-corrected sensor confidence weights, and the advection-diffusion field acquires a shallow-water wave-correction layer that modulates surface slope statistics.

---

## Problem Definition

**Setup**: The same $4 \times 4$ km coastal ocean area and 2D advection-diffusion spill field as S055. In addition, a statistical sea-state model generates a spatially varying significant wave height $H_s(x, y, t)$ that tilts the local surface normal and attenuates spectral returns. Two drones are deployed at different altitudes to patrol the spill boundary and jointly estimate both the boundary shape and local oil thickness.

**Roles**:
- **Drone A (low, boundary delineation)**: flies at commanded altitude $z_A \in [8, 15]$ m, adjusted in real time to hold a fixed sensor footprint radius $r_{foot} = 5$ m on the surface. Responsible for high-spatial-resolution boundary crossing detections and Kalman filter updates.
- **Drone B (high, thickness mapping)**: flies at commanded altitude $z_B \in [25, 40]$ m, adjusted to maximise SNR for fluorescence-intensity-based thickness estimation while covering the full interior of the estimated spill. Produces a coarse $20 \times 20$ thickness grid updated every $T_{replan} = 30$ s.
- **Spill field**: 2D concentration $C(x, y, t)$ plus a thickness proxy field $H_{oil}(x, y, t)$ modelling the progressive weathering and spreading of the slick.
- **Sea-state layer**: spatially correlated significant wave height $H_s(x, y, t)$ evolving on a 5-minute timescale; provides the surface-slope variance that enters the sensor correction model.
- **Ground station**: fuses both drones' altitude-corrected observations, runs the extended boundary Kalman filter, and broadcasts the updated estimate at each replanning epoch.

**Objective**: Maintain boundary coverage $\eta(t) \geq 0.85$ (identical definition to S055) while simultaneously keeping the root-mean-square thickness estimation error $\epsilon_{th}(t) \leq 0.02$ mm across the spill interior, over a $T_{mission} = 30$ min mission.

**Comparison strategies**:
1. **Single altitude, no correction** — both drones fly at $z = 15$ m; no altitude-dependent sensor model; no sea-state correction. Identical to the S055 strategy-3 baseline.
2. **Dual altitude, no sea-state correction** — drones A and B fly at $z_L$ and $z_H$ respectively; altitude-dependent sensor model applied; wave tilt ignored.
3. **Dual altitude + sea-state correction** — full 3D sensor model including altitude, footprint, and wave-slope attenuation; altitude commanded adaptively to maximise joint coverage and thickness SNR.

---

## Mathematical Model

### 3D Sensor Footprint and Altitude Control

The nadir-facing sensor has a fixed half-angle $\phi_{fov}$. The ground footprint radius at altitude $z$ is:

$$r_{foot}(z) = z \tan\phi_{fov}$$

Drone A commands altitude to hold $r_{foot} = r_{target}$:

$$z_A^{cmd} = \frac{r_{target}}{\tan\phi_{fov}}$$

Altitude is tracked by a first-order model with time constant $\tau_z$:

$$\dot{z} = \frac{1}{\tau_z}(z^{cmd} - z)$$

with bounds $z \in [z_{min},\, z_{max}]$.

### Altitude-Dependent Spectral Sensor Model

The received spectral radiance at the sensor from a surface patch of oil thickness $h_{oil}$ (m) and concentration $C$ is modelled as:

$$L_{sensor}(z, h_{oil}, \theta_w) = L_0 \cdot \frac{A_{foot}}{\pi z^2} \cdot \alpha_{oil}(h_{oil}) \cdot \cos\theta_w \cdot \exp\!\left(-\mu_{atm} \cdot z\right)$$

where:
- $A_{foot} = \pi r_{foot}^2$ is the ground footprint area
- $\alpha_{oil}(h_{oil}) = 1 - \exp(-\kappa h_{oil})$ is the fluorescence efficiency with absorption coefficient $\kappa$ (m$^{-1}$)
- $\theta_w$ is the local surface tilt angle due to ocean waves
- $\mu_{atm}$ is the atmospheric attenuation coefficient (m$^{-1}$)

The binary detection indicator (oil present / absent) is:

$$\delta(z, h_{oil}, \theta_w) = \mathbf{1}\!\left[L_{sensor}(z, h_{oil}, \theta_w) \geq L_{thr}\right]$$

The effective detection threshold concentration is therefore altitude- and wave-dependent:

$$C_{thr}^{eff}(z, \theta_w) = \frac{1}{\kappa} \ln\!\left(\frac{L_0 A_{foot} \cos\theta_w \exp(-\mu_{atm} z)}{\pi z^2 L_{thr}}\right)^{-1}$$

### Sea-State Wave Correction

The significant wave height $H_s(x, y, t)$ is drawn from a spatially correlated Gaussian random field with exponential covariance:

$$\mathrm{Cov}[H_s(\mathbf{x}), H_s(\mathbf{x}')] = \sigma_{H}^2 \exp\!\left(-\frac{\|\mathbf{x} - \mathbf{x}'\|}{L_{wave}}\right)$$

The RMS surface slope $\bar{\sigma}_\theta$ is related to $H_s$ and the dominant wavelength $\lambda_w$ via:

$$\bar{\sigma}_\theta(x, y, t) = \arctan\!\left(\frac{\pi H_s(x, y, t)}{\lambda_w}\right)$$

The expected cosine attenuation factor for a Gaussian slope distribution with zero mean and variance $\bar{\sigma}_\theta^2$ is:

$$\langle\cos\theta_w\rangle = \exp\!\left(-\frac{\bar{\sigma}_\theta^2}{2}\right)$$

This correction factor multiplies the sensor model and is applied per-sample based on the drone's current $(x, y)$ position.

### Oil Thickness Proxy Field

The oil thickness proxy $H_{oil}(\mathbf{x}, t)$ evolves alongside the concentration field. A thin-film spreading law (Fay spreading) approximates the radial growth:

$$\frac{d R_{spill}}{d t} = k_F \left(\frac{V_{oil}}{R_{spill}}\right)^{1/3}$$

In the gridded simulation the thickness at each cell is modelled as proportional to the local concentration normalised by the spill age:

$$H_{oil}(i, j, t) = H_0 \cdot \frac{C(i, j, t)}{C_0} \cdot \left(\frac{t_0}{t + t_0}\right)^{1/2}$$

where $H_0$ is the initial peak thickness (m) and $t_0 = R_0 / k_F$ is a spreading time constant.

### Extended Boundary Kalman Filter (3D Landmark State)

Each boundary landmark is extended from $\hat{\mathbf{b}}_k \in \mathbb{R}^2$ to include an altitude-corrected sensor confidence weight $w_k \in [0, 1]$:

$$\mathbf{x}_k = \begin{bmatrix} \hat{\mathbf{b}}_k \\ w_k \end{bmatrix} \in \mathbb{R}^3$$

The confidence weight $w_k$ encodes how reliably landmark $k$ has been observed given its most recent observation altitude $z_{obs,k}$ and local wave height $H_s(\hat{\mathbf{b}}_k)$:

$$w_k = \langle\cos\theta_w(\hat{\mathbf{b}}_k)\rangle \cdot \exp\!\left(-\mu_{atm} z_{obs,k}\right) \cdot \mathbf{1}\!\left[r_{foot}(z_{obs,k}) \leq r_{assoc}\right]$$

The Kalman update for landmark position $\hat{\mathbf{b}}_k$ follows the same scalar-innovation form as S055 but uses the altitude-corrected effective threshold $C_{thr}^{eff}(z_{obs,k}, \theta_w)$ to determine whether a drone crossing constitutes a genuine boundary observation. Landmarks with $w_k < w_{min}$ are treated as uncertain and their process noise is inflated:

$$\mathbf{P}_k \leftarrow \mathbf{P}_k + \Delta\sigma_q^2 (1 - w_k) \mathbf{I}$$

### Altitude-Adaptive Patrol Law

Drone A adapts its commanded altitude to keep $r_{foot}(z_A) = r_{target}$ while also reacting to local wave height to maintain a minimum SNR:

$$z_A^{cmd}(t) = \max\!\left(z_{min},\; \min\!\left(z_{max},\; \frac{r_{target}}{\tan\phi_{fov}} \cdot \frac{1}{\langle\cos\theta_w\rangle^{1/2}}\right)\right)$$

The SNR-driven altitude increase counter-acts wave-slope attenuation by reducing the path length penalty $\exp(-\mu_{atm} z)$ relative to the cosine gain — the optimum balances these competing effects.

Drone B minimises its thickness estimation error by maximising coverage area per unit time while keeping $L_{sensor}$ above the fluorescence detection floor:

$$z_B^{cmd} = \arg\max_{z \in [z_{min,B},\, z_{max,B}]} \left[ r_{foot}(z) \cdot \alpha_{oil}(H_{oil}^{avg}) \cdot \exp(-\mu_{atm} z) \right]$$

where $H_{oil}^{avg}$ is the current mean interior thickness estimate.

### 3D Trajectory Command

Each drone's full 3D commanded velocity is:

$$\mathbf{v}_{cmd} = \begin{bmatrix} v_{arc}\, \hat{\boldsymbol{\tau}}_k + k_{lat}(\mathbf{b}_k^{proj} - \mathbf{p}_{d,xy}) \\ \frac{1}{\tau_z}(z^{cmd} - z_d) \end{bmatrix}$$

where $\mathbf{p}_{d,xy} = (p_{d,x}, p_{d,y})$ is the horizontal drone position, $\hat{\boldsymbol{\tau}}_k$ is the boundary tangent (2D), and $z_d$ is the current drone altitude.

### Thickness Estimation via Dual-Altitude Inversion

At positions sampled by both drones, the two-altitude measurement pair $(L_A, L_B)$ can be inverted to simultaneously estimate local $C$ and $H_{oil}$. Writing the sensor model in log form:

$$\ln L_A = \ln L_0 + \ln\alpha_{oil}(H_{oil}) - \mu_{atm} z_A + \text{const}(z_A)$$

$$\ln L_B = \ln L_0 + \ln\alpha_{oil}(H_{oil}) - \mu_{atm} z_B + \text{const}(z_B)$$

Subtracting eliminates $\alpha_{oil}$ dependence and gives a direct estimate of $\mu_{atm}$; adding constrains $\alpha_{oil}$ and thus $H_{oil}$. The closed-form thickness estimate is:

$$\hat{H}_{oil} = -\frac{1}{\kappa} \ln\!\left(1 - \exp\!\left(\frac{\ln L_A / L_B + \mu_{atm}(z_B - z_A)}{\Delta\text{const}}\right)\right)$$

The root-mean-square thickness estimation error over the spill interior $\Omega$ is:

$$\epsilon_{th}(t) = \sqrt{\frac{\int_\Omega (\hat{H}_{oil}(\mathbf{x}, t) - H_{oil}(\mathbf{x}, t))^2\, d\mathbf{x}}{\int_\Omega d\mathbf{x}}}$$

---

## Key 3D Additions

- **Altitude-dependent sensor model**: detection threshold $C_{thr}^{eff}$ is a function of drone altitude $z$ and local wave tilt angle $\theta_w$; replaces the flat $C_{thr} = 0.08$ of S055.
- **Sea-state wave correction**: spatially correlated $H_s(x, y, t)$ field modulates sensor SNR via $\langle\cos\theta_w\rangle$; causes spatially non-uniform detection reliability along the boundary.
- **3D plume drift**: identical advection-diffusion PDE in the horizontal plane, but altitude commands are now integrated into the drone state and affect which grid cells are within the sensor footprint.
- **Dual-altitude coverage for thickness estimation**: Drone A (low) and Drone B (high) form a multi-altitude stereo sensor pair whose paired measurements solve an overdetermined system for simultaneous boundary and thickness estimation.
- **Altitude-adaptive patrol law**: $z^{cmd}$ varies in real time to balance footprint size, wave-slope SNR, and atmospheric path loss.
- **Extended landmark Kalman state**: landmark confidence weights $w_k$ modulate process noise inflation and are updated alongside position estimates.

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Domain area | 4000 × 4000 m |
| Grid resolution | 200 × 200 cells (20 m/cell) |
| Initial spill radius $R_0$ | 200 m |
| Initial peak concentration $C_0$ | 1.0 (normalised) |
| Contamination threshold (2D baseline) $C_{thr}$ | 0.08 |
| Advection velocity $\mathbf{u}$ | $(0.40,\; 0.15)$ m/s |
| Diffusivity $D$ | 5.0 m$^2$/s |
| PDE timestep $\delta t$ | 2.0 s |
| Number of boundary landmarks $N_b$ | 36 |
| Process noise $\sigma_q$ | 0.5 m/s$^{1/2}$ |
| Measurement noise $\sigma_r$ | 15.0 m |
| Confidence inflation $\Delta\sigma_q$ | 0.3 m/s$^{1/2}$ |
| Minimum confidence weight $w_{min}$ | 0.3 |
| Landmark association radius $r_{assoc}$ | 30 m |
| Drone A altitude range $[z_{min}, z_{max}]$ | $[8,\; 15]$ m |
| Drone B altitude range $[z_{min,B}, z_{max,B}]$ | $[25,\; 40]$ m |
| Target footprint radius $r_{target}$ | 5 m |
| Sensor half-angle $\phi_{fov}$ | $\arctan(5/10) \approx 26.6°$ |
| Atmospheric attenuation $\mu_{atm}$ | $2 \times 10^{-4}$ m$^{-1}$ |
| Fluorescence absorption coefficient $\kappa$ | 500 m$^{-1}$ |
| Initial peak oil thickness $H_0$ | 0.10 mm |
| Thickness RMSE target $\epsilon_{th}$ | 0.02 mm |
| Sea-state std $\sigma_H$ | 0.3 m |
| Sea-state correlation length $L_{wave}$ | 500 m |
| Dominant wavelength $\lambda_w$ | 80 m |
| Altitude time constant $\tau_z$ | 2.0 s |
| Drone speed $v_{arc}$ | 8.0 m/s |
| Lateral correction gain $k_{lat}$ | 0.5 s$^{-1}$ |
| Sampling interval $\Delta s$ | 20 m |
| Replanning epoch $T_{replan}$ | 30 s |
| Staleness threshold $T_{stale}$ | 60 s |
| Mission duration $T_{mission}$ | 1800 s (30 min) |
| Simulation timestep $\Delta t_{sim}$ | 0.5 s |

---

## Expected Output

- **3D trajectory plot**: Matplotlib `mpl_toolkits.mplot3d` axes showing both drone paths with altitude variation; Drone A in orange (low, boundary-following), Drone B in cyan (high, interior sweeping); spill boundary contour projected onto the $z = 0$ surface plane in red; drone altitude shown as vertical drop-lines to the surface projection.
- **Altitude time series**: $z_A(t)$ and $z_B(t)$ over the full mission; $z^{cmd}$ overlaid as dashed; shaded band indicating altitude limits; annotations at wave-correction events where altitude is increased.
- **Sea-state wave field snapshot**: colour map of $H_s(x, y, t)$ at $t = 900$ s with the spill boundary contour overlaid; marker showing regions where wave correction raises the effective threshold by $> 20$%.
- **Sensor footprint comparison**: side-by-side plots of $C_{thr}^{eff}(z, \theta_w)$ as a function of $z$ for three representative wave heights ($H_s = 0.1, 0.3, 0.6$ m); shows how the flat-threshold assumption of S055 underestimates detection difficulty in rough sea states.
- **Boundary coverage time series**: $\eta(t)$ for all three comparison strategies plotted together; target $\eta = 0.85$ as horizontal dashed line; replanning epoch ticks on x-axis; wave-corrected strategy maintains coverage despite wave-induced false detections.
- **Thickness estimation RMSE**: $\epsilon_{th}(t)$ for single-altitude vs dual-altitude strategies; dual-altitude inversion converges to $\epsilon_{th} < 0.02$ mm after the first replanning epoch; single-altitude remains above threshold.
- **Landmark confidence weight heatmap**: $w_k$ mapped onto the estimated boundary landmarks at $t = 600, 1200, 1800$ s; shows spatial non-uniformity driven by the wave field; low-confidence landmarks identified with distinct markers.
- **Boundary estimation error**: mean positional RMSE $\bar{e}(t) = \frac{1}{N_b}\sum_k \|\hat{\mathbf{b}}_k - \mathbf{b}_k^{truth}\|$ for each strategy; 3D wave-corrected strategy achieves lower RMSE than the flat-altitude baseline.
- **Dual-altitude inversion validation scatter**: $\hat{H}_{oil}$ vs $H_{oil}^{truth}$ for all sampled interior cells at mission end; fit line, $R^2$ value, and RMSE annotation.
- **Animation (GIF)**: advancing concentration field (top-down view) with wave-height overlay (semi-transparent); truth boundary (red solid) and estimated boundary (white dashed); both drone icons annotated with current altitude; coverage fraction and thickness RMSE live text overlay.

---

## Extensions

1. **Variable pitch-and-roll drone dynamics**: replace the first-order altitude model $\dot{z} = (z^{cmd}-z)/\tau_z$ with a full 6-DOF rigid-body model where altitude changes require pitch-up manoeuvres that temporarily reduce horizontal groundspeed; study the coverage loss during altitude transitions.
2. **Wind-gust altitude disturbance**: add a stochastic vertical wind component $w_{gust}(t) \sim \mathcal{N}(0, \sigma_w^2)$ that perturbs drone altitude; design a robust altitude controller that maintains $r_{foot}$ within $\pm 1$ m despite gusts.
3. **Three-altitude formation for 3D plume profiling**: add a third drone at $z = 5$ m to detect sub-surface fluorescence from dispersed oil; jointly invert all three altitude measurements to estimate the vertical oil concentration profile $C(x, y, z, t)$ in the top 1 m of the water column.
4. **Nighttime operation with thermal IR**: replace the fluorescence sensor with a thermal infrared imager; oil slicks have different emissivity from water; rederive the sensor model replacing $\alpha_{oil}$ with an emissivity contrast $\Delta\varepsilon(h_{oil})$; assess how altitude and sea-state corrections change.
5. **Wave-following altitude control**: instead of flying at constant commanded altitude above mean sea level, command altitude above the instantaneous local wave surface; requires a lidar altimeter model; study whether this reduces footprint variance and improves boundary consistency.
6. **RL altitude policy**: train a PPO agent whose action space is $\Delta z \in [-1, 1]$ m/step for each drone; reward combines coverage fraction $\eta$, thickness RMSE penalty, and altitude-change fuel cost; compare against the analytic adaptive altitude law.

---

## Related Scenarios

- Original 2D version: [S055 Coastline Oil Spill Tracking](../S055_oil_spill.md)
- 3D sensor modelling reference: [S056 Radiation Hotspot Detection](../S056_radiation.md) (altitude-dependent sensor model in a different domain)
- Advection-diffusion field reference: [S045 Chemical Plume Tracing](../S045_plume_tracing.md) (same PDE structure, particle-filter tracking)
- Boundary tracking reference: [S041 Wildfire Boundary Scan](../S041_wildfire_boundary.md) (iso-contour boundary following, 2D)
- Multi-altitude coordination reference: [S049 Dynamic Zone Assignment](../S049_dynamic_zone.md) (heterogeneous agent roles and partition rebalancing)
- Follow-up: [S058 Typhoon Eye Penetration](../S058_typhoon.md) (extreme sea-state boundary tracking with 3D wind fields)
