# S056 3D Upgrade — Radiation Hotspot Detection

**Domain**: Environmental & SAR | **Difficulty**: ⭐⭐⭐⭐ | **Status**: `[ ]` Not Started
**Based on**: [S056 original](../S056_radiation.md)

---

## What Changes in 3D

The original S056 fixes the drone at a constant survey altitude `h = 15 m` throughout the mission. The slant range formula uses only the horizontal offset from source to drone projected onto a single plane, and the grid posterior is purely 2D — source depth below grade is never considered. In real post-incident surveys the radiation source may be elevated (e.g., a ruptured pipe on a reactor building roof) or partially buried, so the vertical component of the source position contributes substantially to the inverse-square field. Crucially, altitude is also a free control variable for the drone: flying lower dramatically increases the dose-rate gradient near the source, sharpening the Bayesian likelihood and accelerating localisation — at the cost of higher dose accumulation. This 3D variant frees both the drone altitude and the source height, reformulates the dose-rate field as a full volumetric model, and adds an altitude-dependent EIG policy that trades information gain against cumulative dose.

---

## Problem Definition

**Setup**: A $500 \times 500$ m exclusion zone has been declared following a suspected leak at an industrial nuclear facility. The radiation source may be at any 3D position $\mathbf{p}_s^* = (x_s^*, y_s^*, z_s^*)$ within a volumetric search space: horizontal extent $[0, 500]^2$ m, vertical extent $z_s \in [0, 20]$ m (ground level to rooftop). A single drone equipped with a dose-rate detector surveys the zone along an adaptive 3D trajectory. The 3D search volume is discretised into a $25 \times 25 \times 5$ grid of $20 \times 20 \times 4$ m voxels ($N_v = 3\,125$ voxels). The drone altitude is no longer fixed; it selects both a horizontal $(x, y)$ waypoint and a flight altitude $z_{drone} \in [3, 60]$ m from a discrete set $\mathcal{Z} = \{3, 10, 20, 35, 60\}$ m at each step.

**Roles**:
- **Drone** (single agent): carries the dose-rate sensor, selects 3D waypoints $(x, y, z_{drone})$, flies point-to-point at cruise speed $v = 8$ m/s, dwells for $\Delta t_{int} = 5$ s at each waypoint.
- **Radiation source**: one point source of unknown 3D position $\mathbf{p}_s^* \in \mathbb{R}^3$ with unknown strength $A^*$; treated as a hidden 3D state to be inferred via a voxel-grid posterior.

**Objective**: Localise the 3D source position $\hat{\mathbf{p}}_s$ to within $\epsilon_{RMS} \leq 15$ m (3D Euclidean) using the fewest possible measurements, while keeping the drone's cumulative absorbed dose $D_{acc} \leq D_{max} = 2$ mSv (electronics hardness limit proxy).

**Comparison strategies**:
1. **3D EIG-guided** — jointly selects $(x, y, z_{drone})$ to maximise expected information gain subject to the dose budget; primary strategy.
2. **Fixed-altitude EIG** — EIG-optimal $(x, y)$ selection with altitude locked at $h = 15$ m (original S056 behaviour); control group.
3. **Altitude-sweeping lawnmower** — systematic boustrophedon at each altitude level, cycling through all five altitudes; coverage baseline.

---

## Mathematical Model

### 3D Dose-Rate Field

Let $\mathbf{p}_{drone} = (x_d, y_d, z_d)$ be the drone's 3D position and
$\mathbf{p}_s = (x_s, y_s, z_s)$ be the source's 3D position. The true 3D slant range is:

$$r_{3D}(\mathbf{p}_{drone},\, \mathbf{p}_s) = \|\mathbf{p}_{drone} - \mathbf{p}_s\|
  = \sqrt{(x_d - x_s)^2 + (y_d - y_s)^2 + (z_d - z_s)^2}$$

The expected dose rate (µSv/h) at the drone follows the inverse-square law with exponential air attenuation in 3D:

$$D_{3D}(\mathbf{p}_{drone},\, \mathbf{p}_s,\, A) = D_{bg}
  + \frac{A}{r_{3D}^2(\mathbf{p}_{drone},\, \mathbf{p}_s)}
  \exp\!\bigl(-\mu \cdot r_{3D}(\mathbf{p}_{drone},\, \mathbf{p}_s)\bigr)$$

where $\mu = 2 \times 10^{-4}$ m$^{-1}$ is the linear attenuation coefficient of air and
$D_{bg} = 0.10$ µSv/h is the ambient background dose rate.

**Altitude sensitivity**: the dose-rate gradient with respect to drone altitude is

$$\frac{\partial D_{3D}}{\partial z_d} = -A\,(z_d - z_s)\,
  \frac{(2 + \mu\, r_{3D})}{r_{3D}^4}
  \exp\!\bigl(-\mu\, r_{3D}\bigr)$$

This gradient is maximised when $z_d \approx z_s$, showing that flying at the source altitude yields the sharpest spatial gradient — the key motivation for altitude-variable survey.

### Volumetric Measurement Model

The $k$-th noisy dose-rate reading at $\mathbf{p}_{drone,k}$ is:

$$z_k = D_{3D}(\mathbf{p}_{drone,k},\, \mathbf{p}_s^*,\, A^*) + \varepsilon_k,
  \qquad \varepsilon_k \sim \mathcal{N}\bigl(0,\, \sigma_{rad}^2(\mathbf{p}_{drone,k})\bigr)$$

$$\sigma_{rad}(\mathbf{p}_{drone,k}) = \sigma_0
  \sqrt{D_{3D}(\mathbf{p}_{drone,k},\, \mathbf{p}_s^*,\, A^*) + D_{bg}}$$

with $\sigma_0 = 0.04$ µSv$^{0.5}$·h$^{0.5}$.

### 3D Bayesian Voxel Posterior

The 3D source position is a discrete latent variable over the voxel grid
$\mathcal{V} = \{v_1, \ldots, v_{N_v}\}$ with $N_v = 3\,125$ voxels, each centred at
$\mathbf{c}_j = (x_j, y_j, z_j)$.

**Prior**: uniform,

$$P_0(v_j) = \frac{1}{N_v}, \quad \forall\, j$$

**Likelihood** of observation $z_k$ given source voxel $v_j$:

$$P(z_k \mid v_j) = \frac{1}{\sqrt{2\pi}\,\hat{\sigma}_j}
  \exp\!\left(-\frac{\bigl(z_k - D_{3D}(\mathbf{p}_{drone,k},\, \mathbf{c}_j,\, A)\bigr)^2}
  {2\,\hat{\sigma}_j^2}\right)$$

where $\hat{\sigma}_j = \sigma_0\sqrt{D_{3D}(\mathbf{p}_{drone,k}, \mathbf{c}_j, A) + D_{bg}}$.

**Sequential Bayesian update** after measurement $k$:

$$P_k(v_j) = \frac{P(z_k \mid v_j)\cdot P_{k-1}(v_j)}
  {\displaystyle\sum_{l=1}^{N_v} P(z_k \mid v_l)\cdot P_{k-1}(v_l)}, \quad \forall\, j$$

**3D MAP and posterior mean estimates**:

$$\hat{\mathbf{p}}_s^{MAP} = \mathbf{c}_{\hat{j}}, \qquad
  \hat{j} = \arg\max_j\, P_k(v_j)$$

$$\hat{\mathbf{p}}_s^{mean} = \sum_{j=1}^{N_v} P_k(v_j)\cdot \mathbf{c}_j$$

**3D localisation RMS error** over $N_{trial}$ Monte Carlo trials:

$$\epsilon_{RMS} = \sqrt{\frac{1}{N_{trial}} \sum_{t=1}^{N_{trial}}
  \|\hat{\mathbf{p}}_s^{(t)} - \mathbf{p}_s^{*(t)}\|^2}$$

### Volumetric Entropy

Shannon entropy of the 3D voxel posterior:

$$H_k = -\sum_{j=1}^{N_v} P_k(v_j)\log P_k(v_j)$$

Initial entropy: $H_0 = \log N_v = \log 3125 \approx 8.05$ nats.

### 3D EIG Waypoint Selection

The candidate waypoint set is $\mathcal{W} = \mathcal{G}_{xy} \times \mathcal{Z}$, the Cartesian product of the $25 \times 25 = 625$ horizontal grid cell centres and the 5 permitted altitudes, giving 3 125 candidate 3D waypoints.

**Predictive mean** dose rate at candidate 3D position $\mathbf{w} = (x_w, y_w, z_w)$:

$$\bar{z}(\mathbf{w}) = \sum_{j=1}^{N_v} P_k(v_j)\cdot D_{3D}(\mathbf{w},\, \mathbf{c}_j,\, A)$$

**Approximate EIG** via Gaussian mutual information:

$$\text{EIG}(\mathbf{w}) = \frac{1}{2}\log\!\left(1 +
  \frac{\displaystyle\sum_j P_k(v_j)
  \bigl[D_{3D}(\mathbf{w},\, \mathbf{c}_j,\, A) - \bar{z}(\mathbf{w})\bigr]^2}
  {\bar{\sigma}_{rad}^2(\mathbf{w})}\right)$$

where $\bar{\sigma}_{rad}^2(\mathbf{w}) = \sigma_0^2
\sum_j P_k(v_j)\bigl[D_{3D}(\mathbf{w},\mathbf{c}_j,A) + D_{bg}\bigr]$ is the expected noise variance.

**Dose-aware next waypoint** selection penalises candidate positions with high local dose rate to protect the drone within the remaining dose budget $D_{rem,k}$:

$$\mathbf{w}_{k+1}^* = \arg\max_{\mathbf{w} \in \mathcal{W} \setminus \mathcal{R}_k}
  \Bigl[\text{EIG}(\mathbf{w}) - \lambda_D \cdot D_{3D}(\mathbf{w},\,
  \hat{\mathbf{p}}_s^{mean},\, A)\Bigr]$$

where $\lambda_D \geq 0$ is a Lagrange-like penalty weight that increases as $D_{rem,k}$ shrinks:

$$\lambda_D(k) = \lambda_0 \cdot \exp\!\left(\frac{D_{acc,k} - D_{max}/2}{D_{max}/4}\right)$$

and $\mathcal{R}_k$ is the cooldown exclusion set of the last $N_{cooldown} = 5$ visited waypoints.

### Cumulative Dose Accumulation

The absorbed dose accrued during transit and dwell at step $k$ is approximated by integrating dose rate along the straight-line segment from $\mathbf{w}_k$ to $\mathbf{w}_{k+1}$:

$$\Delta D_{transit,k} \approx \bar{D}_{segment,k} \cdot \Delta t_{travel,k}$$

$$\bar{D}_{segment,k} = \frac{1}{2}\Bigl[D_{3D}(\mathbf{w}_k, \mathbf{p}_s^*, A)
  + D_{3D}(\mathbf{w}_{k+1}, \mathbf{p}_s^*, A)\Bigr]$$

$$\Delta D_{dwell,k} = D_{3D}(\mathbf{w}_{k+1}, \mathbf{p}_s^*, A) \cdot \Delta t_{int}$$

$$D_{acc,k} = D_{acc,k-1} + \Delta D_{transit,k} + \Delta D_{dwell,k}$$

Mission terminates when $P_k^{max} \geq P_{conf} = 0.90$ or $D_{acc,k} \geq D_{max}$ or $k = N_{meas,max}$.

### 3D Drone Kinematics

Point-mass, constant-speed flight in 3D between successive waypoints:

$$\dot{\mathbf{p}}_{drone} = v \cdot \hat{\mathbf{u}}, \qquad
  \hat{\mathbf{u}} = \frac{\mathbf{w}_{next} - \mathbf{p}_{drone}}
  {\|\mathbf{w}_{next} - \mathbf{p}_{drone}\|}$$

3D travel time between consecutive waypoints:

$$\Delta t_{travel} = \frac{\|\mathbf{w}_{k+1} - \mathbf{w}_k\|}{v}$$

Altitude bounds: $z_{drone} \in \mathcal{Z} = \{3, 10, 20, 35, 60\}$ m.

---

## Key 3D Additions

- **3D voxel posterior**: source localisation over a $25 \times 25 \times 5$ volumetric grid rather than a 2D horizontal grid; posterior encodes both horizontal and vertical source depth uncertainty.
- **Altitude-variable survey**: drone selects from five discrete altitudes per waypoint; low-altitude passes ($z_d = 3$ m) maximise dose-rate gradient near the source; high-altitude passes ($z_d = 60$ m) provide long-range spatial context.
- **Full 3D slant range**: $r_{3D}$ replaces $\sqrt{\|\mathbf{p}_{xy} - \mathbf{p}_{s,xy}\|^2 + h^2}$; accounts for vertical separation between drone and elevated or buried source.
- **Altitude sensitivity gradient**: $\partial D_{3D}/\partial z_d$ quantifies the information value of altitude changes; used to motivate the dive phase of the adaptive policy.
- **Dose-aware EIG policy**: Lagrange penalty $\lambda_D(k)$ dynamically re-weights information gain against local dose rate as the cumulative dose budget $D_{acc}$ approaches $D_{max}$.
- **Cumulative dose tracking**: trapezoidal integration of dose rate along each flight segment enforces the electronics dose budget constraint $D_{acc} \leq D_{max} = 2$ mSv.
- **3D safe corridor planning**: after source MAP estimate is established, the return-to-home trajectory is re-routed through the 3D field to avoid the high-dose cone directly above the source; corridor defined by the iso-dose surface $D_{3D} = 1$ µSv/h.

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Survey area | 500 × 500 m (horizontal) |
| Source altitude range $z_s$ | 0 – 20 m |
| Voxel grid | 25 × 25 × 5 = 3 125 voxels |
| Voxel size | 20 × 20 × 4 m |
| Drone altitude set $\mathcal{Z}$ | {3, 10, 20, 35, 60} m |
| Cruise speed $v$ | 8 m/s |
| Measurement dwell $\Delta t_{int}$ | 5 s |
| Max measurements $N_{meas,max}$ | 80 |
| Confidence stop threshold $P_{conf}$ | 0.90 |
| Source strength $A$ | 2 000 µSv·m²/h |
| Air attenuation $\mu$ | 2 × 10$^{-4}$ m$^{-1}$ |
| Background dose rate $D_{bg}$ | 0.10 µSv/h |
| Detector noise coefficient $\sigma_0$ | 0.04 µSv$^{0.5}$·h$^{0.5}$ |
| Dose budget $D_{max}$ | 2 mSv |
| Dose penalty base $\lambda_0$ | 0.05 |
| Cooldown window $N_{cooldown}$ | 5 waypoints |
| Monte Carlo trials $N_{trial}$ | 30 |
| Target localisation RMS | $\leq$ 15 m (3D) |
| Initial entropy $H_0$ | $\ln(3125) \approx 8.05$ nats |

---

## Expected Output

- **3D trajectory plot**: drone flight path rendered in full 3D with colour-coded altitude; source true position marked with a gold sphere; MAP estimate with a red sphere; iso-dose surface at 1 µSv/h shown as a transparent mesh.
- **Altitude time series**: $z_{drone}(t)$ showing dive-and-climb patterns driven by the EIG policy; annotated with measurement events.
- **Voxel posterior slices**: horizontal ($z$) and vertical ($x = \hat{x}_s$) cross-sections of $P_k(v_j)$ at steps $k = 1, 10, 30, 60$; log-colour scale; true source voxel highlighted.
- **Entropy decay comparison**: $H_k$ vs measurement index for 3D EIG, fixed-altitude EIG, and altitude-sweeping lawnmower; 3D EIG should show fastest reduction.
- **Localisation RMS vs measurements**: $\epsilon_{RMS}(k)$ with 95 % confidence bands for all three strategies averaged over $N_{trial} = 30$ Monte Carlo trials; horizontal dashed line at 15 m target.
- **Cumulative dose profile**: $D_{acc}(k)$ for the 3D EIG strategy vs fixed-altitude EIG; horizontal line at $D_{max} = 2$ mSv; shows whether the dose-aware penalty keeps the drone within budget.
- **Safe corridor visualisation**: 3D plot of the return-to-home path after localisation, overlaid on the $D_{3D} = 1$ µSv/h iso-dose surface; demonstrates corridor clearance.
- **Animation (GIF)**: rotating 3D view showing belief voxels (opacity proportional to $P_k(v_j)$) concentrating toward the true source over successive measurements; drone icon moves along the adaptive 3D path.

---

## Extensions

1. **Elevated source on building facade**: restrict $z_s \in [5, 20]$ m and add a building-block obstacle at the source location; the EIG policy must plan around the occluded line-of-sight using ray-casting to mask shadowed voxels from the likelihood computation.
2. **Unknown source strength with joint inference**: treat $(A, \mathbf{p}_s)$ as joint latent variables; extend the voxel grid to include a discrete strength axis $A \in \{500, 1000, 2000, 5000\}$ µSv·m²/h; assess how strength uncertainty inflates the 3D localisation ellipsoid.
3. **Multi-drone 3D parallel survey**: deploy $N = 3$ drones sharing a common 3D voxel posterior via a communication network; implement altitude-separated de-confliction (assign each drone a preferred altitude band) to prevent near-misses and redundant measurements.
4. **Gaussian plume source in 3D wind field**: replace the point-source model with a 3D atmospheric dispersion plume $D \propto \exp(-y^2 / 2\sigma_y^2) \cdot \exp(-z^2 / 2\sigma_z^2)$ advected by wind vector $\mathbf{u}_{wind}$; the drone must simultaneously estimate 3D source location, emission rate, and wind direction.
5. **Particle filter in 3D continuous space**: replace the 3 125-voxel discrete posterior with a particle filter ($M = 10\,000$ particles) over continuous 3D source coordinates; compare posterior accuracy and computational load against the voxel grid approach.
6. **Dose-rate tomography**: treat the 3D dose-rate field as an unknown volumetric function and reconstruct it via compressed sensing using measurements along the adaptive 3D flight path; compare reconstructed field with the true inverse-square model.

---

## Related Scenarios

- Original 2D version: [S056](../S056_radiation.md)
- 3D SAR reference: [S042 Missing Person Localization](../S042_missing_person.md) (shared Bayesian EIG framework, extended to 3D)
- Volumetric mapping reference: [S050 SLAM](../S050_slam.md) (3D occupancy grid update analogous to 3D voxel posterior)
- Contaminant field mapping: [S055 Oil Spill](../S055_oil_spill.md), [S045 Plume Tracing](../S045_plume_tracing.md)
