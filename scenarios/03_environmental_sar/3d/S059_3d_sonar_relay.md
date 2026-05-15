# S059 3D Upgrade — Sonar Buoy Relay

**Domain**: Environmental & SAR | **Difficulty**: ⭐⭐⭐⭐ | **Status**: `[ ]` Not Started
**Based on**: [S059 original](../S059_sonar_relay.md)

---

## What Changes in 3D

The original S059 fixes all four drone hover positions at $z = 0$ m (ocean surface) and constrains buoy deployment to a flat 2D surface grid. The CRLB optimisation searches only over $(x_i, y_i)$ pairs, and buoy drop dynamics are instantaneous — a buoy appears directly below the drone at the moment of release with no fall time or lateral drift. In true 3D this variant adds three physically meaningful layers:

1. **Altitude-dependent buoy drop ballistics** — drones hover at a programmable altitude $z_{hover} \in [5, 50]$ m above sea level; the buoy free-falls through air and then sinks through water under combined drag, so the buoy's final splash position $\mathbf{p}_i^{splash}$ is offset from the drone's nadir by wind drift and lateral momentum.
2. **3D sonar coverage geometry** — each hydrophone buoy hangs at a programmable depth $z_{buoy} \in [-5, -30]$ m below the surface on a tether, rather than floating exactly at $z = 0$. The acoustic path from the underwater target to the receiver is therefore fully 3D (slant ranges replace horizontal ranges), changing the TDOA Jacobian in all three axes.
3. **Optimal hover altitude for listen quality** — a drone hovering at higher altitude receives the buoy's radio telemetry over a wider line-of-sight footprint but introduces greater buoy placement error; the scenario quantifies the trade-off and finds the hover altitude $z^*$ that minimises the combined CRLB penalty from buoy placement scatter and reduced TDOA discriminability.

---

## Problem Definition

**Setup**: An underwater acoustic source (sunken wreck or silent submarine) emits broadband pings at $T_{ping} = 5$ s interval. Four drones transit to staging positions at altitude $z_{hover}$ above the ocean surface and each releases one sonar buoy. Each buoy free-falls through air, splashes, and sinks to a tether depth $z_{buoy}$. Because release occurs at altitude, the buoy arrives at a 3D position $\mathbf{p}_i^{buoy} = (x_i + \delta x_i, \; y_i + \delta y_i, \; z_{buoy})$ where $(\delta x_i, \delta y_i)$ is the lateral displacement caused by wind and forward drone velocity during the fall. Once settled, the buoy records ping arrival times; TDOAs are relayed to the surface drone and then to a shore station for Gauss-Newton localisation.

**Roles**:
- **Drones** ($N = 4$): hover at 3D positions $\mathbf{q}_i = (x_i, y_i, z_{hover})$ above target waypoints; execute buoy release at $t = 0$ and maintain radio relay link to shore during the listen phase.
- **Sonar buoys** ($N = 4$): passive hydrophone receivers that settle at 3D positions $\mathbf{p}_i^{buoy} = (x_i + \delta x_i, \; y_i + \delta y_i, \; z_{buoy})$; each records arrival time $t_i^{rec}$ of each ping with per-buoy clock noise $\epsilon_i \sim \mathcal{N}(0, \sigma_t^2)$.
- **Underwater target**: stationary acoustic source at unknown position $\mathbf{p}_T = (x_T, y_T, z_T)$ with $z_T \in [-200, -10]$ m; emits a ping every $T_{ping}$ seconds.
- **Ground station**: fuses four noisy TDOA measurements via Gauss-Newton least squares to estimate $\hat{\mathbf{p}}_T$ in full 3D.

**Objective**: Jointly (1) model buoy drop ballistics to predict final 3D buoy positions, (2) incorporate tether depth into the TDOA measurement model, (3) optimise drone hover altitude $z_{hover}$ and buoy tether depth $z_{buoy}$ to minimise the total 3D localisation CRLB, and (4) compare three buoy layout strategies (square grid, tetrahedral projection, CRLB-optimal) under the realistic 3D placement-scatter model. Quantify how drop altitude degrades (or improves) the effective GDOP compared to the idealised $z = 0$ surface model.

---

## Mathematical Model

### Buoy Drop Ballistics

A buoy of mass $m_b = 0.8$ kg and cross-sectional area $A = 0.008$ m$^2$ is released from altitude $z_{hover}$ with zero initial vertical velocity. The air-phase drag equation is:

$$m_b \ddot{z} = -m_b g + \tfrac{1}{2} \rho_{air} C_D A \dot{z}^2, \qquad \dot{z}(0) = 0, \quad z(0) = z_{hover}$$

where $\rho_{air} = 1.225$ kg/m$^3$, $C_D = 1.0$ (bluff body). The terminal velocity in air is:

$$v_{term}^{air} = \sqrt{\frac{2 m_b g}{\rho_{air} C_D A}}$$

Horizontal drift during the fall time $t_{fall}$ (numerically integrated via Euler steps) due to a horizontal wind vector $\mathbf{w} = (w_x, w_y)$ m/s and drone forward velocity $\mathbf{v}_{drone}^{xy}$:

$$\begin{pmatrix}\delta x_i \\ \delta y_i\end{pmatrix} = \left(\mathbf{w} + \mathbf{v}_{drone,i}^{xy}\right) t_{fall}$$

The splash speed determines the initial sink rate in the water phase; the buoy sinks to tether depth $z_{buoy}$ and is assumed to be at rest by the time the first ping arrives (tether settles within $T_{settle} \approx 30$ s).

### 3D TDOA Measurement Model with Tether Depth

Let buoy $i$ be located at full 3D position $\mathbf{p}_i^{buoy} = (x_i + \delta x_i, \; y_i + \delta y_i, \; z_{buoy}) \in \mathbb{R}^3$. The slant acoustic range from target to buoy $i$ is:

$$r_i = \left\|\mathbf{p}_i^{buoy} - \mathbf{p}_T\right\|_2
     = \sqrt{(x_i + \delta x_i - x_T)^2 + (y_i + \delta y_i - y_T)^2 + (z_{buoy} - z_T)^2}$$

The TDOA between buoys $i$ and $1$ (reference):

$$\Delta t_{i1} = \frac{r_i - r_1}{c_s}, \qquad c_s = 1500 \text{ m/s}$$

Noisy measurement:

$$\Delta \hat{t}_{i1} = \Delta t_{i1} + \nu_{i1}, \qquad \nu_{i1} \sim \mathcal{N}(0,\; 2\sigma_t^2)$$

In range-difference units $\Delta \hat{d}_{i1} = c_s \cdot \Delta \hat{t}_{i1}$, the noise standard deviation is $\sigma_d = c_s \sqrt{2}\, \sigma_t$.

### 3D Jacobian with Tether Depth

Because $z_{buoy} \neq 0$, the unit direction vectors from the target estimate $\mathbf{p}$ to each buoy now have a non-trivial $z$-component. Define:

$$\hat{\mathbf{u}}_{k \leftarrow T}(\mathbf{p}) = \frac{\mathbf{p}_k^{buoy} - \mathbf{p}}{\left\|\mathbf{p}_k^{buoy} - \mathbf{p}\right\|}$$

The Jacobian row for pair $(i, 1)$ is:

$$\mathbf{H}_i(\mathbf{p}) = \hat{\mathbf{u}}_{1 \leftarrow T}(\mathbf{p}) - \hat{\mathbf{u}}_{i \leftarrow T}(\mathbf{p}) \in \mathbb{R}^3$$

This differs from S059 because $\hat{\mathbf{u}}_{k \leftarrow T}$ now has a vertical component $(z_{buoy} - p_z) / r_k$ that was zero when all buoys floated at $z = 0$.

### Fisher Information Matrix and 3D CRLB

The FIM and CRLB trace are unchanged in form but now use the 3D Jacobian above:

$$\mathbf{F}_{3D}(\mathbf{p}_T) = \frac{1}{\sigma_d^2} \mathbf{H}^T \mathbf{H} \in \mathbb{R}^{3 \times 3}$$

$$\mathcal{C}_{3D} = \text{tr}\!\left(\mathbf{F}_{3D}^{-1}\right) = \sigma_d^2\, \text{tr}\!\left[(\mathbf{H}^T \mathbf{H})^{-1}\right]$$

The key difference is that buoy tether depth $z_{buoy}$ increases the vertical baseline between receiver and source, which tightens the depth component of $\mathbf{F}^{-1}$:

$$[\mathbf{F}^{-1}]_{zz} \propto \frac{1}{\sum_i \sin^2\phi_i}$$

where $\phi_i = \arctan\!\left(\frac{|z_{buoy} - z_T|}{r_{i,xy}}\right)$ is the vertical grazing angle to buoy $i$. Deeper tether $\Rightarrow$ larger $\phi_i$ $\Rightarrow$ better depth resolution.

### Placement-Scatter CRLB Penalty

The buoy landing position is a random variable:

$$\mathbf{p}_i^{buoy} = \mathbf{p}_i^{nominal} + \boldsymbol{\delta}_i, \qquad \boldsymbol{\delta}_i \sim \mathcal{N}(\mathbf{0},\; \sigma_{drop}^2\, \mathbf{I}_2 \oplus 0)$$

where $\sigma_{drop}$ is the standard deviation of horizontal drop scatter (a function of $z_{hover}$, wind speed, and fall time). Via a first-order Taylor expansion around nominal buoy positions:

$$\mathbb{E}_{\boldsymbol{\delta}}\!\left[\mathcal{C}_{3D}\right] \approx \mathcal{C}_{3D}^{nominal} + \sigma_{drop}^2 \sum_i \left\|\nabla_{\mathbf{p}_i^{xy}} \mathcal{C}_{3D}\right\|^2$$

The total penalty is therefore minimised by a hover altitude $z^*_{hover}$ that balances:
- small $\sigma_{drop}$ (low altitude, less fall time, less drift) against
- adequate relay link margin for the drone to receive buoy telemetry (higher altitude preferred).

### Optimal Hover Altitude

Model drop scatter as linearly growing with fall time: $\sigma_{drop}(z_{hover}) = \kappa \sqrt{t_{fall}(z_{hover})}$ where $\kappa = w_{rms} / \sqrt{3}$ and $w_{rms}$ is RMS wind speed. Model relay link quality as a line-of-sight coverage radius $R_{link}(z_{hover}) = z_{hover} / \tan\theta_{min}$ where $\theta_{min}$ is the minimum elevation angle for reliable reception. The optimal altitude satisfies:

$$z^*_{hover} = \arg\min_{z_{hover} \in [5,\, 50]\, \text{m}} \left[\mathcal{C}_{3D}^{nominal}(z_{hover}) + \sigma_{drop}^2(z_{hover})\, G_{scatter}\right]$$

where $G_{scatter} = \sum_i \left\|\nabla_{\mathbf{p}_i^{xy}} \mathcal{C}_{3D}\right\|^2$ is a geometry-dependent scatter amplification factor evaluated at the nominal buoy layout.

### Gauss-Newton Localisation (3D)

Identical structure to S059, now using 3D buoy positions $\mathbf{p}_i^{buoy}$ (including tether depth):

$$\mathbf{p}^{(k+1)} = \mathbf{p}^{(k)} + \bigl(\mathbf{H}_k^T \mathbf{H}_k\bigr)^{-1} \mathbf{H}_k^T \mathbf{f}(\mathbf{p}^{(k)})$$

Convergence criterion: $\|\Delta \mathbf{p}^{(k)}\|_2 < \epsilon_{tol} = 10^{-3}$ m or $K_{max} = 200$ iterations.

---

## Key 3D Additions

- **Buoy drop ballistics**: Euler-integrated air-phase free-fall with $C_D$ drag; horizontal drift from wind vector $\mathbf{w}$ and drone velocity over fall time $t_{fall}(z_{hover})$.
- **Tether depth model**: buoys hang at $z_{buoy} \in [-5, -30]$ m rather than $z = 0$; all range and Jacobian calculations use the full 3D slant path.
- **Vertical grazing angle**: $\phi_i = \arctan(|z_{buoy} - z_T| / r_{i,xy})$ quantifies how much each buoy-target pair contributes to depth resolution; deeper tether increases $\phi_i$ and tightens $[\mathbf{F}^{-1}]_{zz}$.
- **Placement-scatter penalty**: first-order CRLB sensitivity to drop scatter $\sigma_{drop}$ added to the objective; scatter grows as $\kappa \sqrt{t_{fall}}$.
- **Optimal hover altitude search**: 1D line search over $z_{hover} \in [5, 50]$ m balancing scatter penalty against relay-link coverage; reports $z^*_{hover}$ and the corresponding CRLB improvement over the surface-hover ($z = 0$) baseline.
- **3D error ellipsoid visualisation**: CRLB error ellipsoids rendered in full 3D axes showing anisotropy reduction in the $z$ axis when tether depth is used.

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Number of buoys $N$ | 4 |
| Speed of sound $c_s$ | 1500 m/s |
| Per-buoy clock noise $\sigma_t$ | $10^{-4}$ s |
| TDOA range-difference noise $\sigma_d$ | $c_s \sqrt{2}\, \sigma_t \approx 0.212$ m |
| Ping interval $T_{ping}$ | 5 s |
| Drone hover altitude range $z_{hover}$ | 5 – 50 m |
| Buoy tether depth range $z_{buoy}$ | $-5$ to $-30$ m |
| Wind speed $w_{rms}$ | 3 m/s (light breeze) |
| Drag coefficient $C_D$ | 1.0 |
| Buoy mass $m_b$ | 0.8 kg |
| Buoy cross-section $A$ | 0.008 m$^2$ |
| Air density $\rho_{air}$ | 1.225 kg/m$^3$ |
| Terminal velocity in air $v_{term}^{air}$ | $\approx 12.9$ m/s |
| Drop scatter coefficient $\kappa$ | $w_{rms} / \sqrt{3}$ |
| True target position $\mathbf{p}_T$ | $(40, -30, -120)$ m |
| Target depth range | $-200$ to $-10$ m |
| Square-grid half-side | 100 m |
| Tetrahedral layout radius | 120 m |
| Max buoy spread $D_{max}$ | 500 m |
| Gauss-Newton tolerance $\epsilon_{tol}$ | $10^{-3}$ m |
| FIM regularisation $\delta$ | $10^{-6}$ |
| Monte Carlo trials | 1000 |
| CRLB optimisation restarts | 20 |
| Relay link min elevation angle $\theta_{min}$ | 5° |

---

## Expected Output

- **3D buoy geometry plot**: four buoys at tether depth $z_{buoy}$ (cyan markers), true target (red star at depth), acoustic path lines (dashed) from each buoy to the target; drone hover positions shown at altitude $z_{hover}$ (grey diamonds) with vertical drop lines to splash points; illustrates the fully 3D geometry versus the flat-surface S059 baseline.
- **Drop ballistics trajectory**: $z(t)$ and lateral drift $|\delta x(t)|$ during the air-phase fall for $z_{hover} \in \{5, 15, 30, 50\}$ m; marks the splash instant and shows how drift scales with hover altitude.
- **CRLB vs tether depth**: $\sqrt{\mathcal{C}_{3D}}$ plotted against $z_{buoy} \in [-30, 0]$ m for all three layout strategies; shows that $z_{buoy} = 0$ (S059 baseline) is the worst-case depth resolution and deeper tether consistently reduces the CRLB depth component $[\mathbf{F}^{-1}]_{zz}$.
- **CRLB vs hover altitude**: total CRLB (nominal + scatter penalty) plotted against $z_{hover} \in [5, 50]$ m; identifies the optimal hover altitude $z^*_{hover}$ where the scatter-induced CRLB inflation is minimised; separate curves for each buoy layout.
- **3D error ellipsoids**: $1\sigma$ CRLB ellipsoids centred on the true target for (a) S059 baseline ($z_{buoy} = 0$) and (b) 3D variant ($z_{buoy} = -20$ m); illustrates depth-axis shrinkage from the tether contribution.
- **Monte Carlo RMSE comparison table**: horizontal RMSE $\sqrt{\Delta x^2 + \Delta y^2}$ and depth RMSE $|\Delta z|$ for all three layout strategies under both surface-float and tether-depth models; shows depth RMSE improvement factor when using $z_{buoy} = -20$ m.
- **GDOP map (horizontal slice at $z_T = -120$ m)**: 2D heatmap of $\text{GDOP}_{TDOA}$ over target $(x, y)$ for the CRLB-optimal layout with and without tether depth; highlights regions where depth resolution is particularly poor under the surface-only constraint.
- **Optimal hover altitude summary**: annotated bar chart comparing total CRLB at $z_{hover} = 0$ m (surface hover baseline), $z^* _{hover}$ (optimal), and $z_{hover} = 50$ m (maximum range); scatter amplification factor $G_{scatter}$ shown for each case.

---

## Extensions

1. **Dynamic tether depth control**: allow each buoy to actively reel its hydrophone to a mission-chosen depth $z_{buoy,i}$ after the first ping is received; reformulate the CRLB optimisation over both horizontal positions and individual tether depths to find the jointly optimal 3D buoy configuration.
2. **Wind-compensated release**: given a known wind forecast $\mathbf{w}(t, z)$ profile, compute an upwind horizontal offset for the drone hover position so that the buoy drifts to the intended splash point; evaluate residual placement error after compensation.
3. **Multi-depth layered array**: deploy two sets of four buoys at different tether depths ($z_{buoy,1} = -5$ m, $z_{buoy,2} = -25$ m) to create a vertical array that further constrains the depth axis; derive the extended $7 \times 3$ Jacobian for the 8-buoy overdetermined system.
4. **Moving target with EKF**: the target moves at $v_T = 1$ m/s; an EKF fuses successive TDOA batches (one per ping) with the full 3D buoy-at-depth measurement model to jointly estimate position and velocity $[\mathbf{p}_T, \dot{\mathbf{p}}_T]$; compare track accuracy to the flat-surface S059 EKF.
5. **Acoustic shadow zones**: add a simplified range-dependent sound speed profile $c_s(z)$ (e.g., Munk profile); rays bend away from the surface layer, creating shadow zones where certain buoy-target pairs have no direct acoustic path; detect affected TDOA pairs and down-weight them in the FIM.

---

## Related Scenarios

- Original 2D version: [S059 Sonar Buoy Relay](../S059_sonar_relay.md)
- Prerequisites: [S046 Multi-Drone 3D Trilateration](../S046_trilateration.md), [S047 Base Station Signal Relay](../S047_signal_relay.md)
- Algorithmic cross-reference: [S046 Trilateration](../S046_trilateration.md) (range-based vs TDOA-based localisation with 3D receiver geometry), [S047 Signal Relay](../S047_signal_relay.md) (drone altitude vs link coverage trade-off)
- Pursuit & Evasion 3D reference: [S002 3D Upgrade](../../01_pursuit_evasion/3d/S002_3d_evasive_maneuver.md) (3D kinematics and altitude-strategy patterns)
