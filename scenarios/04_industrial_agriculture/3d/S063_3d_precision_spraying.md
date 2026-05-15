# S063 3D Upgrade — Precision Per-Plant Spraying

**Domain**: Industrial & Agriculture | **Difficulty**: ⭐⭐ | **Status**: `[ ]` Not Started
**Based on**: [S063 original](../S063_precision_spraying.md)

---

## What Changes in 3D

The original S063 fixes the drone at a constant spray altitude (z is never modelled; horizontal
position is the only state). Two simplifications make this acceptable only as a 2D baseline:

1. **No canopy height variation** — real crops (maize, sunflowers, orchards) have per-plant
   heights $h_i$ that span 0.3 – 3.0 m. A fixed flight altitude produces highly variable
   nozzle-to-target standoff distances, which directly changes droplet deposition density.
2. **No vertical wind or rotor downwash** — the original wind model is purely horizontal
   ($x$-$y$ disturbance). In 3D, rotor downwash adds a strong vertical component that deflects
   spray droplets downward and outward; crosswind shear across the canopy height further displaces
   the droplet plume.

This variant introduces:
- A **canopy height model** $h_i$ per plant drawn from a realistic height distribution.
- An **altitude-adaptive flight law** that commands $z_{cmd,i} = h_i + \Delta z_{nom}$ so the
  nozzle standoff is held constant regardless of crop height.
- A **3D droplet drift model** combining horizontal wind shear, rotor downwash, and droplet
  settling velocity to compute the true ground-impact offset from the nozzle position.
- A **3D PID hover controller** with a decoupled vertical loop added to the two horizontal loops
  of S063.

---

## Problem Definition

**Setup**: A single drone carries a downward-facing nozzle over a $20 \times 20$ m field
containing $N = 20$ plants. Plant positions $(x_i, y_i)$ are drawn as in S063; each plant
additionally has a canopy height $h_i$ drawn from a truncated normal distribution representing
crop height variability. The drone plans a nearest-neighbour TSP path, then flies in 3D: it
climbs to $z_{cmd,i} = h_i + \Delta z_{nom}$ before hovering over each plant and spraying for
$T_{spray} = 3$ s. A 3D PID controller rejects both horizontal wind disturbances and vertical
updraft/downwash. Spray droplets follow a ballistic drift model to land at positions offset from
the nozzle.

**Roles**:
- **Drone**: single UAV with 3D state $(x, y, z, \dot{x}, \dot{y}, \dot{z})$; three decoupled
  PID loops (one per axis); altitude commanded per plant based on canopy height.
- **Plants**: $N = 20$ static targets at $(x_i, y_i, 0)$ with canopy heights
  $h_i \sim \mathcal{TN}(\mu_h, \sigma_h^2; h_{min}, h_{max})$.
- **Wind disturbance**: 3D vector field with horizontal sinusoidal gusts plus a vertical
  downwash component from the rotor.

**Objective**: Simulate the full 3D mission and report:

1. **Nozzle standoff distance** $\delta z_i = z_i(t) - h_i$ at each plant — should track
   $\Delta z_{nom} = 0.5$ m.
2. **3D hover RMS error** $\varepsilon_i^{3D}$ including both horizontal and vertical position
   errors.
3. **Droplet impact offset** $d_i$ — horizontal distance between actual droplet landing point
   and plant centre.
4. **Effective coverage fraction** $C_{eff}^{3D}$ using the altitude-corrected dose model.
5. Trade-off between canopy height variance $\sigma_h$ and achievable $C_{eff}^{3D}$ for three
   altitude-control strategies.

**Altitude-control strategies** (compared side by side):

1. **Fixed altitude** — $z_{cmd} = 1.5$ m for all plants (S063 baseline behaviour in 3D).
2. **Per-plant adaptive** — $z_{cmd,i} = h_i + \Delta z_{nom}$ (nominal standoff maintained).
3. **Terrain-following PID** — continuous altitude feedback tracking the canopy surface
   estimated by a lidar sensor model with noise $\sigma_{lidar} = 0.05$ m.

---

## Mathematical Model

### Canopy Height Model

Plant canopy heights are drawn from a truncated normal distribution:

$$h_i \sim \mathcal{TN}\!\left(\mu_h,\, \sigma_h^2;\, h_{min},\, h_{max}\right), \quad i = 1,\ldots,N$$

with default parameters $\mu_h = 1.0$ m, $\sigma_h = 0.3$ m, $h_{min} = 0.3$ m,
$h_{max} = 2.5$ m. The lidar measurement of plant $i$ is:

$$\hat{h}_i = h_i + \eta_{lidar}, \quad \eta_{lidar} \sim \mathcal{N}(0, \sigma_{lidar}^2)$$

### 3D Drone State and Altitude Command

The full drone state is $\mathbf{q}(t) = (x, y, z)^T$ with velocity
$\dot{\mathbf{q}}(t) = (\dot{x}, \dot{y}, \dot{z})^T$. For plant $i$ the 3D target hover
position is:

$$\mathbf{p}_i^{3D} = \begin{pmatrix} x_i \\ y_i \\ h_i + \Delta z_{nom} \end{pmatrix}$$

where $\Delta z_{nom} = 0.5$ m is the nominal nozzle standoff. The nozzle standoff error at
time $t$ is:

$$\delta z_i(t) = z(t) - h_i$$

### 3D PID Position Hold

Three decoupled single-axis PID loops share the same gain structure. For axis
$k \in \{x, y, z\}$:

$$e_k(t) = p_{i,k}^{3D} - q_k(t)$$

$$a_{k}^{PID}(t) = K_p^{(k)}\, e_k(t)
                 + K_i^{(k)} \int_0^t e_k(\tau)\, d\tau
                 + K_d^{(k)}\, \dot{e}_k(t)$$

The vertical loop uses stiffer gains than the horizontal loops because altitude errors directly
couple to standoff distance and hence spray deposition:

$$K_p^{(z)} = \alpha_z\, K_p^{(xy)}, \quad \alpha_z \geq 1$$

with default $\alpha_z = 1.5$. The integrator for each axis resets at the start of each hover
window.

### 3D Wind and Downwash Disturbance

The 3D acceleration disturbance at time $t$ is:

$$\mathbf{f}_w(t) = \begin{pmatrix}
  A_w \sin\!\left(\tfrac{2\pi t}{T_w} + \varphi_x\right) + \mathcal{N}(0,\sigma_w^2) \\
  A_w \sin\!\left(\tfrac{2\pi t}{T_w} + \varphi_y\right) + \mathcal{N}(0,\sigma_w^2) \\
  -f_{dw} + \mathcal{N}(0,\sigma_{dw}^2)
\end{pmatrix}$$

where $f_{dw} = 0.15$ m/s² is the mean downwash deceleration (upward thrust needed to hold
altitude), $\sigma_{dw} = 0.05$ m/s² is its stochastic variation, and $\varphi_x, \varphi_y$
are independent random phases resampled at each plant.

The equations of motion in 3D:

$$\dot{\mathbf{q}}(t+\Delta t) = \dot{\mathbf{q}}(t) + \bigl[\mathbf{a}^{PID}(t) + \mathbf{f}_w(t)\bigr]\Delta t$$
$$\mathbf{q}(t+\Delta t) = \mathbf{q}(t) + \dot{\mathbf{q}}(t)\,\Delta t$$

with altitude bounds enforced: $z(t) \in [z_{min}, z_{max}] = [0.3,\, 6.0]$ m.

### 3D Droplet Drift Model

A spray droplet released from nozzle position $\mathbf{q}_{nozzle} = \mathbf{q}(t) - (0,0,0)^T$
(nozzle is at the drone's CoM for this model) falls under gravity while being advected
horizontally by local wind velocity $\mathbf{u}_w$ and deflected by rotor downwash $w_{dw}$:

$$\text{Fall time:} \quad t_{fall} = \sqrt{\frac{2\,\delta z}{g + w_{dw}}}$$

where $\delta z = z(t) - h_i$ is the nozzle standoff and $g = 9.81$ m/s². The horizontal drift
of the droplet during free fall is:

$$\Delta \mathbf{r}_{drift} = \mathbf{u}_w^{(xy)}\, t_{fall}$$

where $\mathbf{u}_w^{(xy)} = (u_x, u_y)^T$ is the mean horizontal wind velocity (m/s) at spray
time. The droplet impact position on the ground plane is:

$$\mathbf{p}_{impact} = \begin{pmatrix} x(t) \\ y(t) \end{pmatrix} + \Delta \mathbf{r}_{drift}$$

The droplet impact offset from the plant centre is:

$$d_i = \left\| \mathbf{p}_{impact} - \begin{pmatrix} x_i \\ y_i \end{pmatrix} \right\|_2$$

### Altitude-Corrected Dose Model

The effective dose at plant $i$ accounts for both horizontal hover error and droplet drift:

$$D_i^{3D} = D_0 \exp\!\left(-\frac{(d_i)^2}{2\,\sigma_{spray}^2}\right)
             \cdot \exp\!\left(-\frac{(\delta z_i - \Delta z_{nom})^2}{2\,\sigma_z^2}\right)$$

The first factor penalises horizontal miss distance from the drift model; the second factor
penalises standoff deviation from the nominal, where $\sigma_z = 0.2$ m is the altitude
accuracy scale. The fleet-level coverage fraction is:

$$C_{eff}^{3D} = \frac{1}{N} \sum_{i=1}^{N}
  \exp\!\left(-\frac{d_i^2}{2\,\sigma_{spray}^2}\right)
  \exp\!\left(-\frac{(\delta z_i - \Delta z_{nom})^2}{2\,\sigma_z^2}\right)$$

### 3D Hover RMS Error

$$\varepsilon_i^{3D} = \sqrt{\frac{1}{M} \sum_{k=1}^{M} \|\mathbf{q}(t_k) - \mathbf{p}_i^{3D}\|^2}$$

where $M = \lfloor T_{spray} / \Delta t \rfloor$ and the norm is the full 3D Euclidean distance.

---

## Key 3D Additions

- **Canopy height model**: per-plant heights from truncated normal; lidar sensor noise on
  measurement; commands a unique hover altitude for each plant.
- **Altitude-adaptive command**: $z_{cmd,i} = h_i + \Delta z_{nom}$ decouples spray quality from
  crop height variability; the vertical PID loop tracks this command.
- **Vertical disturbance**: rotor downwash $f_{dw}$ and its stochastic component $\sigma_{dw}$
  added to the $z$-axis disturbance; vertical gains scaled by $\alpha_z$ for stiffer altitude
  hold.
- **3D droplet drift**: ballistic fall-time formula couples nozzle standoff to horizontal drift;
  wind velocity advects droplets before impact, shifting the effective spray target.
- **Altitude-corrected dose**: two-factor Gaussian dose model penalises both horizontal drift
  ($d_i$) and standoff deviation ($\delta z_i - \Delta z_{nom}$) independently.
- **3D trajectory visualisation**: full 3D trajectory with altitude subplot; standoff distance
  time series per plant; droplet impact scatter around each plant.

---

## Key Parameters

| Parameter | Symbol | Value |
|-----------|--------|-------|
| Field size | — | 20 x 20 m |
| Number of plants | $N$ | 20 |
| Mean canopy height | $\mu_h$ | 1.0 m |
| Canopy height std dev | $\sigma_h$ | 0.3 m |
| Canopy height range | $[h_{min}, h_{max}]$ | [0.3, 2.5] m |
| Lidar noise std dev | $\sigma_{lidar}$ | 0.05 m |
| Nominal nozzle standoff | $\Delta z_{nom}$ | 0.5 m |
| Altitude bounds | $[z_{min}, z_{max}]$ | [0.3, 6.0] m |
| Altitude gain scale | $\alpha_z$ | 1.5 |
| Spray width | $w_{spray}$ | 2.0 m |
| Horizontal accuracy scale | $\sigma_{spray}$ | 0.5 m |
| Altitude accuracy scale | $\sigma_z$ | 0.2 m |
| Mean rotor downwash decel | $f_{dw}$ | 0.15 m/s² |
| Downwash noise std dev | $\sigma_{dw}$ | 0.05 m/s² |
| Horizontal gust amplitude | $A_w$ | 0.3 m/s² |
| Gust period | $T_w$ | 4.0 s |
| Baseline turbulence intensity | $\sigma_w$ | 0.1 m/s² |
| Hover duration per plant | $T_{spray}$ | 3.0 s |
| Simulation timestep | $\Delta t$ | 0.02 s (50 Hz) |
| Transit speed | $v$ | 1.0 m/s |
| Tuned horizontal PID $(K_p, K_i, K_d)$ | — | 2.0, 0.2, 0.5 |
| Tuned vertical PID $(K_p, K_i, K_d)$ | — | 3.0, 0.3, 0.75 |

---

## Expected Output

- **3D field overview**: `mpl_toolkits.mplot3d` scatter of plant positions at their true canopy
  heights $(x_i, y_i, h_i)$; drone 3D trajectory shown as a red line; vertical dashed lines
  from plant base to hover altitude illustrating variable standoff.
- **Altitude time series**: $z(t)$ over the full mission with horizontal dashed lines marking
  each $z_{cmd,i}$; shaded bands showing $\pm \sigma_z$ around each command; clearly shows the
  drone stepping up/down between plants.
- **Standoff deviation per plant**: bar chart of $|\delta z_i - \Delta z_{nom}|$ for each plant
  under the three altitude-control strategies (Fixed / Adaptive / Terrain-Following); illustrates
  how fixed altitude fails at tall plants.
- **Droplet impact scatter**: top-down view with each plant centre marked green; droplet impact
  points shown as small blue dots around each plant; radial offset $d_i$ annotated; spray circle
  of radius $\sigma_{spray}$ drawn for reference.
- **$C_{eff}^{3D}$ vs canopy height variance sweep**: three curves (Fixed / Adaptive /
  Terrain-Following) plotted against $\sigma_h \in [0, 0.6]$ m; shows how adaptive altitude
  command maintains $C_{eff}^{3D}$ when Fixed strategy degrades severely at high $\sigma_h$.
- **Console summary**: TSP path length (m), mean 3D hover RMS error (cm), mean standoff
  deviation (cm), mean droplet drift (cm), per-strategy $C_{eff}^{3D}$.

---

## Extensions

1. **Lidar-based terrain following**: replace the pre-planned $z_{cmd,i}$ with a real-time
   lidar feedback loop that continuously adjusts altitude as the drone traverses between plants;
   compare dynamic tracking latency against the per-plant discrete step strategy.
2. **Wind-aware drift compensation**: use an onboard anemometer to measure $\mathbf{u}_w^{(xy)}$
   in real time; shift the nozzle horizontally by $-\Delta \mathbf{r}_{drift}$ before spraying
   to pre-compensate droplet drift; quantify improvement in $d_i$ distribution.
3. **Droplet size selection**: model multiple nozzle orifice sizes producing droplets of
   diameter $D_{drop} \in [100, 500]$ $\mu$m with different settling velocities
   $v_s = k D_{drop}^2$; find the optimal $D_{drop}$ that minimises drift while maintaining
   adequate canopy penetration.
4. **Multi-drone altitude bands**: assign two drones to different altitude bands
   ($z_1 = h_i + 0.5$ m and $z_2 = h_i + 1.5$ m) targeting the same plants from different
   standoffs; model the superimposed dose deposition and compare coverage uniformity.
5. **3D TSP with altitude cost**: augment the nearest-neighbour TSP path cost with the vertical
   transit energy between consecutive plants: $c_{ij} = \|p_i^{3D} - p_j^{3D}\|$; compare 3D
   TSP tour length against the 2D tour with altitude overhead added post-hoc.
6. **Variable-rate 3D spraying**: integrate a prescription map specifying both horizontal dose
   rate and per-plant target standoff; couple the dose rate controller (nozzle flow valve) to
   the altitude controller so dose-per-area is invariant to speed variation.

---

## Related Scenarios

- Original 2D version: [S063](../S063_precision_spraying.md)
- Truly 3D agricultural references: [S062 Crop Row Following](../S062_crop_row_following.md), [S067 Spray Overlap Optimisation](../S067_spray_overlap_optimization.md)
- Domain 3D format reference: [S002 3D Upgrade](../../01_pursuit_evasion/3d/S002_3d_evasive_maneuver.md)
