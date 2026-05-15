# S045 3D Upgrade — Chemical Plume Tracing

**Domain**: Environmental & SAR | **Difficulty**: ⭐⭐⭐⭐ | **Status**: `[ ]` Not started
**Based on**: [S045 original](../S045_plume_tracing.md)

---

## What Changes in 3D

The original S045 hardcodes the drone altitude at $z_d = 2.0$ m throughout the entire simulation.
The vertical dispersion term $\exp(-z_d^2 / (2\sigma_z^2))$ is pre-computed as a constant scalar
and folded into an effective emission rate $Q_{eff}$, removing any dependence on altitude from the
plume model. The finite-difference gradient estimator probes only in the $x$-$y$ plane
(`grad = [(c_xp - c_xm), (c_yp - c_ym)] / (2*delta)`), so vertical concentration structure is
completely ignored. The cast-and-surge and particle filter both operate on 2D position vectors.

This 3D upgrade restores the full separable Gaussian plume in all three axes. The drone is now free
to vary altitude between 0.5 m and 8.0 m. Key additions:

- **Full 3D Gaussian plume model**: concentration depends explicitly on $(x, y, z)$; the vertical
  reflection term models ground-level emission with a reflecting lower boundary.
- **Atmospheric stratification**: Pasquill-Gifford $\sigma_z$ now acts as a true vertical spread,
  so the peak concentration altitude shifts with downwind distance.
- **3D gradient estimation**: six-point finite-difference stencil along $\pm x$, $\pm y$, $\pm z$
  axes; the gradient vector is used to command both lateral and vertical motion.
- **Altitude-hunting surge strategy**: during surges the drone tracks the vertical concentration
  maximum, climbing or descending to ride the plume centreline.
- **3D particle filter**: particles are drawn from a 3D volume $(x_s, y_s, z_s)$; source height
  $z_s$ is treated as unknown and estimated alongside the horizontal position.
- **Vertical source localisation**: the posterior marginal over $z_s$ enables determination of
  stack height or leak elevation, which is impossible with the 2D version.

---

## Problem Definition

**Setup**: A chemical release of unknown position occurs inside a $200 \times 200 \times 10$ m
volume (ground to 10 m altitude). The source emits continuously at rate $Q = 1.0$ g/s and is
elevated at some unknown height $z_s \in [0.5, 6.0]$ m (e.g. a stack vent or a pipe fitting on a
platform). Mean wind blows along $+x$ at $u = 3.0$ m/s; vertical wind is negligible. A single
drone starts downwind at position $(20, 80, 2)$ m and carries a chemical sensor sampling at 10 Hz.
The drone must trace the plume back to the source in all three dimensions, determining both the
horizontal location and the elevation of the release point.

**Roles**:
- **Drone**: single agent; samples 3D concentration, estimates the 3D gradient, executes a
  3D cast-and-surge policy with an altitude-hunting layer, and maintains a 3D particle filter
  posterior over source position $(x_s, y_s, z_s)$.
- **Source**: fixed point $(x_s, y_s, z_s)$ emitting at rate $Q$; all three coordinates unknown
  to the drone.
- **3D Particle filter**: 500-particle estimator over the 3D source volume; updated on every
  concentration measurement; posterior mean drives the particle-filter-guided navigation mode.

**Objective**: Navigate to within $r_{capture} = 3.0$ m (3D Euclidean) of the true source in
minimum time, while simultaneously estimating source height $z_s$ to within $\pm 0.5$ m.

**Comparison strategies**:
1. **2D baseline lifted to 3D**: original cast-and-surge confined to $z = 2$ m; locates $(x_s, y_s)$
   but cannot determine $z_s$.
2. **3D cast-and-surge with altitude hunting**: surge phase tracks the altitude of peak vertical
   concentration; cast phase sweeps both crosswind ($\pm y$) and vertically ($\pm z$).
3. **3D particle-filter guided**: posterior mean of the 3D particle filter provides a 3D waypoint
   attracting the drone; altitude converges alongside horizontal position.

---

## Mathematical Model

### Full 3D Gaussian Plume with Ground Reflection

For a continuous point source at $(x_s, y_s, z_s)$ with mean wind speed $u$ along $+x$, the
time-averaged concentration at position $(x, y, z)$ (downwind offset $\Delta x = x - x_s > 0$) is:

$$C(x,y,z) = \frac{Q}{2\pi \, \sigma_y(\Delta x)\, \sigma_z(\Delta x)\, u}
              \exp\!\left(-\frac{(y - y_s)^2}{2\sigma_y^2(\Delta x)}\right)
              \left[
                \exp\!\left(-\frac{(z - z_s)^2}{2\sigma_z^2(\Delta x)}\right)
              + \exp\!\left(-\frac{(z + z_s)^2}{2\sigma_z^2(\Delta x)}\right)
              \right]$$

The second exponential in the vertical bracket is the image-source reflection term that enforces
zero vertical flux at $z = 0$ (ground). For $z_s \ll \sigma_z$ (near-ground source) both terms
merge into a factor of 2. The concentration is zero for $\Delta x \leq 0$.

### Pasquill-Gifford Dispersion Coefficients (Neutral Class D)

$$\sigma_y(\Delta x) = 0.08\,\Delta x\,(1 + 0.0002\,\Delta x)^{-1/2}$$

$$\sigma_z(\Delta x) = 0.06\,\Delta x\,(1 + 0.0015\,\Delta x)^{-1/2}$$

### Altitude of Peak Vertical Concentration

For a given downwind distance $\Delta x$ and source height $z_s$, the altitude $z^*$ that maximises
$C$ (ignoring the reflection term) is:

$$z^* = z_s \quad \text{(peak at source elevation)}$$

Once the reflection term is included and $\sigma_z > z_s$, the vertical profile broadens
asymmetrically. The drone tracks $z^*$ by gradient ascent in the $z$ direction:

$$z_{cmd}(t) = z_d(t) + k_z \, \hat{g}_z$$

where $k_z = 0.5$ m is the altitude step gain and $\hat{g}_z$ is the $z$-component of the
normalised 3D concentration gradient.

### Turbulent Intermittency Model (3D)

Identical in form to S045, applied to the 3D concentration field:

$$C_{meas}(t) = C(x,y,z)\cdot\xi(t) + \eta(t)$$

$$p_{detect}(x,y,z) = 1 - \exp\!\left(-\frac{C(x,y,z)}{C_{thresh}}\right), \quad \xi(t) \sim \text{Bernoulli}(p_{detect})$$

$$\eta(t) \sim \mathcal{N}(0,\,\sigma_{noise}^2), \quad \sigma_{noise} = 0.05$$

### 3D Finite-Difference Gradient Estimation

Six-point stencil along each axis with probe offset $\delta = 1.0$ m:

$$\hat{\nabla}_x C \approx \frac{C(x{+}\delta,y,z) - C(x{-}\delta,y,z)}{2\delta}$$

$$\hat{\nabla}_y C \approx \frac{C(x,y{+}\delta,z) - C(x,y{-}\delta,z)}{2\delta}$$

$$\hat{\nabla}_z C \approx \frac{C(x,y,z{+}\delta) - C(x,y,z{-}\delta)}{2\delta}$$

Each of the six probe measurements is corrupted by independent noise. The normalised 3D gradient:

$$\hat{\mathbf{g}} = \frac{(\hat{\nabla}_x C,\;\hat{\nabla}_y C,\;\hat{\nabla}_z C)}
                         {\|(\hat{\nabla}_x C,\;\hat{\nabla}_y C,\;\hat{\nabla}_z C)\| + \varepsilon}$$

### 3D Cast-and-Surge with Altitude Hunting

The state machine is extended to a three-axis controller:

| State | Condition | Horizontal action | Vertical action |
|-------|-----------|-------------------|-----------------|
| **Surge** | $C_{meas} \geq C_{thresh}$ | Move at $v_{surge}$ biased upwind + gradient | Step toward $z^*$ via $k_z \hat{g}_z$ |
| **Cast** | $C_{meas} < C_{thresh}$, $t_{nc} \leq T_{cast}$ | Crosswind sweep $\pm y$ at $v_{cast}$ | Alternate $+z$ / $-z$ sweep at $v_{z,cast}$ |
| **Lost** | $C_{meas} < C_{thresh}$, $t_{nc} > T_{cast}$ | Return to last contact position | Return to last contact altitude |

The 3D surge velocity command:

$$\mathbf{v}_{cmd} = v_{surge}\!\left[\alpha\,\hat{\mathbf{u}}_{wind}
                    + (1-\alpha)\,\hat{\mathbf{g}}_{xy}\right]
                    + k_z\,\hat{g}_z\,\hat{\mathbf{z}}$$

where $\hat{\mathbf{u}}_{wind} = (1,0,0)^T$, $\hat{\mathbf{g}}_{xy}$ is the horizontal component
of the normalised gradient, and the vertical term adjusts altitude independently.

During the cast phase the drone sweeps in both $y$ and $z$ simultaneously on independent timers,
doubling the probability of re-entering the 3D plume cross-section.

### 3D Particle Filter Source Localisation

Particles represent hypotheses over 3D source position $\mathbf{p}_s^{(i)} = (x_s^{(i)}, y_s^{(i)}, z_s^{(i)})$.

**Initialisation**: particles drawn uniformly from
$x_s \in [0, 200]$ m, $y_s \in [0, 200]$ m, $z_s \in [0.5, 6.0]$ m.

**Prediction step**: source is stationary; no motion noise needed. An optional jitter
$\mathbf{p}_s^{(i)} \leftarrow \mathbf{p}_s^{(i)} + \mathcal{N}(\mathbf{0}, \sigma_{jitter}^2 \mathbf{I})$
with $\sigma_{jitter} = 0.1$ m prevents particle degeneracy when the posterior collapses early.

**Update step** at drone position $(x_d, y_d, z_d)$ with measurement $C_{meas}$:

For each particle $i$, compute predicted concentration using the full 3D plume model:

$$\hat{C}^{(i)} = C\!\left(x_d - x_s^{(i)},\; y_d - y_s^{(i)},\; z_d,\; z_s^{(i)}\right)$$

Likelihood and weight update:

$$w_i \propto \exp\!\left(-\frac{(C_{meas} - \hat{C}^{(i)})^2}{2\sigma_{noise}^2}\right)$$

**Resampling**: systematic resampling when $N_{eff} = \left(\sum_i w_i^2\right)^{-1} < N_p/2$.

**3D posterior estimate**:

$$\hat{\mathbf{p}}_s = \sum_{i=1}^{N_p} w_i\,\mathbf{p}_s^{(i)}
  = \left(\hat{x}_s,\; \hat{y}_s,\; \hat{z}_s\right)$$

The marginal posterior over $z_s$ is obtained by kernel density estimation on
$\{z_s^{(i)}, w_i\}$; the mode of this marginal is the estimated stack height.

### 3D Drone Kinematics

Point-mass model with first-order speed response in all three axes:

$$\dot{\mathbf{p}}_d = \mathbf{v}_d, \qquad \mathbf{p}_d \in \mathbb{R}^3$$

$$\dot{\mathbf{v}}_d = \frac{1}{\tau_v}\!\left(\mathbf{v}_{cmd} - \mathbf{v}_d\right)$$

Altitude is bounded: $z_d \in [z_{min}, z_{max}] = [0.5, 8.0]$ m with a reflecting boundary
(vertical velocity reversed if the boundary is reached during a cast sweep).

### 3D Source Capture Criterion

$$\|\mathbf{p}_d - \mathbf{p}_s\|_2 \leq r_{capture} = 3.0 \text{ m}$$

where the norm is the full Euclidean distance in $(x, y, z)$.

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Volume | 200 × 200 × 10 m |
| Source emission rate $Q$ | 1.0 g/s |
| Source height $z_s$ | unknown; $z_s \in [0.5, 6.0]$ m |
| Wind speed $u$ | 3.0 m/s (along $+x$) |
| Drone altitude range | 0.5 – 8.0 m |
| Sensor noise $\sigma_{noise}$ | 0.05 (normalised) |
| Detection threshold $C_{thresh}$ | 0.02 (normalised) |
| Surge speed $v_{surge}$ | 1.5 m/s (horizontal) |
| Cast speed $v_{cast}$ | 0.8 m/s (horizontal) |
| Vertical cast speed $v_{z,cast}$ | 0.4 m/s |
| Altitude step gain $k_z$ | 0.5 m |
| Upwind bias $\alpha$ | 0.4 |
| Gradient probe offset $\delta$ | 1.0 m (all axes) |
| Particle count $N_p$ | 500 |
| Particle jitter $\sigma_{jitter}$ | 0.1 m |
| Capture radius $r_{capture}$ | 3.0 m (3D) |
| Max drone speed $v_{max}$ | 2.0 m/s |
| Speed time constant $\tau_v$ | 0.5 s |
| Dispersion model | Pasquill-Gifford class D + ground reflection |
| $\sigma_y$ formula | $0.08\,\Delta x\,(1+0.0002\,\Delta x)^{-1/2}$ |
| $\sigma_z$ formula | $0.06\,\Delta x\,(1+0.0015\,\Delta x)^{-1/2}$ |
| Mission timeout $T_{max}$ | 600 s |
| Simulation timestep $\Delta t$ | 0.1 s |

---

## Expected Output

- **3D plume concentration volume**: isosurface or $xy$-slice stack at multiple altitudes
  ($z = 0.5, 1.0, 2.0, 3.0, z_s$ m) showing how plume width and peak altitude evolve downwind;
  true source marked with a red sphere.
- **3D trajectory plot**: drone path in full $(x, y, z)$ axes for all three strategies
  (2D-baseline, 3D cast-and-surge, PF-guided); altitude excursions shown on a side-panel
  elevation vs time subplot.
- **Altitude time series**: $z_d(t)$ for each strategy; highlight the altitude convergence
  event where the drone locks onto the plume centreline at $z \approx z_s$.
- **Particle filter 3D posterior snapshots**: four frames at $t = 0, 60, 180, 360$ s showing
  particle cloud projected onto $(xy)$ and $(xz)$ planes simultaneously; convergence toward
  $(x_s, y_s, z_s)$ visualised.
- **$z_s$ marginal posterior evolution**: KDE over the vertical coordinate of all particles at
  each snapshot; shows when the estimated source height $\hat{z}_s$ locks onto the true $z_s$.
- **Concentration vertical profiles**: measured $C_{meas}(z)$ transects at fixed $x$ downwind
  distances; Gaussian fit overlaid; estimated vs true peak altitude annotated.
- **Performance comparison table**: time-to-find, path length, final 3D position error, and
  $z_s$ estimation RMSE for each strategy across 20 Monte Carlo runs.
- **Animation (GIF)**: 3D trajectory animation with rotating viewpoint; particle cloud shown in
  $(xz)$ projection inset; concentration isosurface at $C_{thresh}$ rendered as a semi-transparent
  volume; drone altitude label updated each frame.

---

## Extensions

1. **Wind shear with altitude**: add a logarithmic wind profile $u(z) = u_{ref}\,\ln(z/z_0)/\ln(z_{ref}/z_0)$
   where $z_0 = 0.03$ m is the roughness length; the plume centreline tilts with height, requiring
   the drone to account for altitude-dependent advection when computing the upwind surge direction.
2. **Multiple sources at different heights**: two simultaneous releases at $(x_{s1}, y_{s1}, z_{s1})$
   and $(x_{s2}, y_{s2}, z_{s2})$; extend the particle filter to a mixture model with two source
   components and a mixing weight parameter.
3. **Unsteady wind direction**: wind rotates slowly over time $\phi(t) = \phi_0 + \Delta\phi\sin(\omega_\phi t)$;
   the particle filter must jointly estimate source position and current wind bearing.
4. **RL altitude policy**: train a PPO agent whose state includes $(C_{meas}, \hat{\mathbf{g}}, z_d, \hat{z}_s)$
   and whose action includes a continuous $v_z$ command; compare learning efficiency against the
   altitude-hunting heuristic.
5. **Multi-drone 3D cooperative tracing**: two drones at different initial altitudes share particle
   filter measurements via consensus; the drone at the better altitude provides more informative
   likelihood updates, accelerating 3D source convergence.
6. **Atmospheric stability classes**: repeat with Pasquill classes A (unstable) and F (stable);
   class A produces rapid vertical diffusion (large $\sigma_z$), making altitude hunting easier;
   class F produces a flat, narrow vertical plume concentrated near $z_s$.

---

## Related Scenarios

- Original 2D version: [S045 Chemical Plume Tracing](../S045_plume_tracing.md)
- 3D source localisation reference: [S046 3D Trilateration](../S046_trilateration.md)
- Advecting scalar field (oil spill analogue): [S055 Oil Spill Tracking](../S055_oil_spill.md)
- Radiation source-finding (same gradient-ascent structure): [S056 Radiation Hotspot Detection](../S056_radiation.md)
- Multi-agent extension: [S049 Dynamic Zone Assignment](../S049_dynamic_zone.md)
