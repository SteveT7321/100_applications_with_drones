# S070 3D Upgrade — Swarm Weeding

**Domain**: Industrial & Agriculture | **Difficulty**: ⭐⭐⭐⭐ | **Status**: `[ ]` Not Started
**Based on**: [S070 original](../S070_swarm_weeding.md)

---

## What Changes in 3D

The original S070 fixes all drone altitude at a single cruise height throughout both phases. The
downward-facing sensor footprint is modelled as a flat 2D circle of radius $r_s = 1.5$ m, and the
PID hover controller drives only $(x, y)$ error. Weed positions are pure 2D coordinates — there is
no concept of canopy height, sub-canopy occlusion, or altitude-dependent sensor resolution.

In this 3D upgrade three major additions are made:

1. **Canopy penetration for sub-canopy weed targeting** — crop row height is modelled as a
   Gaussian random field $h_{canopy}(x,y)$. After a weed is detected during the Phase 1 sweep,
   the treating drone must descend to $z_{treat} = h_{canopy}(\mathbf{w}_j) + \Delta h_{clear}$
   (just above the weed), navigate under the canopy layer, and apply herbicide from close range.

2. **Altitude-dependent vision resolution** — the multispectral sensor has a fixed pixel count.
   Its ground-sampling distance (GSD) scales linearly with altitude, so detection probability
   degrades with height. Drones choose an optimal sweep altitude $z_{sweep}^*$ that balances
   detection quality against flight-time penalty from flying lower (slower safe speed).

3. **3D weed position estimation from stereo cameras** — during Phase 1, a stereo pair
   estimates weed $(x, y, z_{weed})$ in 3D. The depth estimate introduces an additional vertical
   uncertainty $\sigma_z$ that must be accounted for in the treatment descent profile.

---

## Problem Definition

**Setup**: A $40 \times 40$ m crop field contains $W = 30$ weed plants at random $(x, y)$
positions. Crop row canopy height follows a smooth Gaussian random field with mean
$\bar{h} = 0.6$ m and spatial correlation length $\ell = 4$ m. Each weed's true 3D position is
$\mathbf{w}_j = (x_j, y_j, z_{weed,j})$ where $z_{weed,j}$ is drawn from
$\mathcal{U}(0.0, h_{canopy}(x_j, y_j))$ — the weed sits at or below the canopy surface.

A swarm of $N = 3$ drones executes the same two-phase mission as S070, now extended to 3D:

**Phase 1 — 3D Detection Sweep**: Each drone sweeps its assigned sub-strip at a chosen altitude
$z_{sweep} \in [1.0, 3.5]$ m above ground. The stereo camera provides a 3D detection cone: a weed
at ground position $\mathbf{w}_j$ is within the sensor footprint if the horizontal range satisfies
$\|(x_j - p_x, y_j - p_y)\| \leq r_s(z)$, where $r_s(z) = r_{s,0} \cdot z / z_{ref}$ scales with
altitude. Detection probability also degrades with altitude according to GSD:
$P_{detect}(z) = P_{detect,0} \cdot (z_{ref} / z)^{\beta}$. Detected weed 3D centroids are
estimated with planar noise $\sigma_{GPS}$ and vertical noise $\sigma_z(z)$ from stereo depth.

**Phase 2 — 3D Treatment**: After Phase 1, detected weeds $\mathcal{W}_{det}$ with 3D estimated
positions $\hat{\mathbf{w}}_j = (\hat{x}_j, \hat{y}_j, \hat{z}_{canopy,j})$ are assigned to
drones via Hungarian algorithm on 3D Euclidean cost. Each drone navigates in full 3D, descending
to the treatment altitude $z_{treat,j} = \hat{z}_{canopy,j} + \Delta h_{clear}$, and holds a
6-DOF hover using a 3D PID controller for $T_{treat} = 2$ s.

**Roles**:
- **Drones** ($N = 3$): homogeneous platforms; sweep altitude $z_{sweep}^*$ selected once per
  mission; Phase 2 altitude is weed-specific based on canopy height estimates.
- **Weeds** ($W = 30$): fixed 3D positions; require sub-canopy approach for treatment.

**Objective**: Treat the maximum number of weeds (primary) while minimising total 3D travel
distance in Phase 2 (secondary). Hungarian assignment now operates on 3D cost; compare the
effect of sweep altitude choice on detection rate and overall mission success.

---

## Mathematical Model

### Canopy Height Field

Canopy height is modelled as a Gaussian random field (GRF):

$$h_{canopy}(\mathbf{x}) \sim \mathcal{GP}\!\left(\bar{h},\, k_{SE}(\mathbf{x}, \mathbf{x}')\right)$$

$$k_{SE}(\mathbf{x}, \mathbf{x}') = \sigma_h^2 \exp\!\left(-\frac{\|\mathbf{x} - \mathbf{x}'\|^2}{2\ell^2}\right)$$

with $\bar{h} = 0.6$ m, $\sigma_h = 0.15$ m, $\ell = 4.0$ m. The field is sampled on a
$100 \times 100$ grid and bilinearly interpolated at arbitrary $(x, y)$ positions.

### Altitude-Dependent Sensor Model

The sensor footprint radius scales linearly with altitude $z$:

$$r_s(z) = r_{s,0} \cdot \frac{z}{z_{ref}}, \qquad r_{s,0} = 1.5 \text{ m},\; z_{ref} = 2.0 \text{ m}$$

Ground-sampling distance (GSD) increases with altitude, degrading detection probability:

$$P_{detect}(z) = P_{detect,0} \cdot \left(\frac{z_{ref}}{z}\right)^{\beta}, \qquad
  P_{detect,0} = 0.85,\; \beta = 0.5$$

A weed at 3D position $\mathbf{w}_j = (x_j, y_j, z_{weed,j})$ is within the sensor footprint of
drone at $\mathbf{p}_i = (p_x, p_y, z)$ when:

$$\sqrt{(x_j - p_x)^2 + (y_j - p_y)^2} \leq r_s(z) \quad \text{and} \quad z > h_{canopy}(x_j, y_j)$$

(the drone must be above the canopy to have line-of-sight to the weed below).

Weed detection is drawn from $b_{ij} \sim \mathrm{Bernoulli}(P_{detect}(z))$ for each encounter.

### 3D Stereo Depth Estimation Error

The stereo camera estimates weed depth (vertical position) with uncertainty that grows with range.
For a drone at altitude $z$ above ground, the vertical position estimate of a weed at height
$z_{weed}$ has standard deviation:

$$\sigma_z(z) = \sigma_{z,0} \cdot \left(\frac{z}{z_{ref}}\right)^2, \qquad \sigma_{z,0} = 0.05 \text{ m}$$

The full 3D estimated weed position is:

$$\hat{\mathbf{w}}_j = \mathbf{w}_j + \boldsymbol{\eta}_{3D}, \qquad
  \boldsymbol{\eta}_{3D} \sim \mathcal{N}\!\left(\mathbf{0},\,
  \mathrm{diag}(\sigma_{GPS}^2,\, \sigma_{GPS}^2,\, \sigma_z(z)^2)\right)$$

### Optimal Sweep Altitude

Define the expected number of weeds detected as a function of sweep altitude $z$:

$$\mathbb{E}[N_{det}(z)] = W \cdot P_{detect}(z) = W \cdot P_{detect,0} \cdot \left(\frac{z_{ref}}{z}\right)^{\beta}$$

Lower altitude improves $P_{detect}$ but reduces safe cruise speed due to canopy proximity:

$$v_{sweep}(z) = v_{max} \cdot \min\!\left(1,\, \frac{z - \bar{h}}{\Delta h_{safe}}\right), \qquad
  \Delta h_{safe} = 1.0 \text{ m}$$

Total Phase 1 mission time scales as $T_{sweep}(z) \propto 1 / v_{sweep}(z)$. The optimal sweep
altitude balances detection gain against sweep duration:

$$z_{sweep}^* = \arg\max_{z \in [1.0,\, 3.5]} \frac{\mathbb{E}[N_{det}(z)]}{T_{sweep}(z)}$$

This can be solved analytically or by a 1D line search over $z$.

### 3D Hungarian Task Assignment

The 3D cost matrix uses full Euclidean distance including altitude:

$$C_{ij} = \|\mathbf{p}_i^{end} - \hat{\mathbf{w}}_j\|_2
          = \sqrt{(p_{i,x} - \hat{x}_j)^2 + (p_{i,y} - \hat{y}_j)^2 + (p_{i,z} - \hat{z}_{treat,j})^2}$$

where $\hat{z}_{treat,j} = \hat{z}_{canopy,j} + \Delta h_{clear}$ is the target treatment altitude
for weed $j$. The assignment minimises total 3D travel cost:

$$\min_{\mathbf{X}} \sum_{i \in \mathcal{D}} \sum_{j \in \mathcal{W}_{det}} C_{ij}\, x_{ij}$$

$$\text{subject to} \quad \sum_j x_{ij} \leq K_{max},\; \forall i; \qquad
  \sum_i x_{ij} = 1,\; \forall j; \qquad x_{ij} \in \{0,1\}$$

solved via `scipy.optimize.linear_sum_assignment` with rectangular padding.

### 3D PID Hover Controller

Phase 2 hover must regulate all three spatial axes. The control law is extended to 3D:

$$\mathbf{u}(t) = K_p\, \mathbf{e}(t) + K_i \sum_{\tau=0}^{t} \mathbf{e}(\tau)\,\Delta t
                + K_d\, \frac{\mathbf{e}(t) - \mathbf{e}(t-1)}{\Delta t}$$

where $\mathbf{e}(t) = \mathbf{w}_j^{target} - \tilde{\mathbf{p}}_i(t) \in \mathbb{R}^3$ now
includes the vertical error. The vertical axis uses the same gains but is subject to the stereo
depth estimation bias $\delta z = \hat{z}_{weed} - z_{weed}$ in addition to GPS noise.

Steady-state 3D RMS hover error:

$$\sigma_{hover,3D}^2 = \frac{\sigma_{GPS}^2 + \sigma_{GPS}^2 + \sigma_z^2}{1 + 2 K_p / K_d}$$

Treatment succeeds if $\|\mathbf{e}\|_2 \leq \epsilon_{pos,3D} = 0.20$ m (relaxed from 2D
threshold to account for vertical estimation error).

### Canopy Penetration Descent Profile

To reach $z_{treat,j}$, each drone follows a straight-line 3D approach from its current position
$\mathbf{p}_{start}$ to the treatment waypoint $\mathbf{w}_j^{target}$. The descent is
constrained to avoid collision with the canopy surface:

$$z(t) \geq h_{canopy}(x(t), y(t)) + \Delta h_{clear}, \qquad \Delta h_{clear} = 0.20 \text{ m}$$

If the straight-line path would violate this constraint, the drone first moves horizontally to the
$(x_j, y_j)$ column at safe altitude $z_{sweep}$, then descends vertically to $z_{treat,j}$:

$$\mathbf{p}_{waypoints} = \begin{cases}
  \mathbf{p}_{start} \to (x_j, y_j, z_{sweep}) & \text{horizontal transit} \\
  (x_j, y_j, z_{sweep}) \to (x_j, y_j, z_{treat,j}) & \text{vertical descent}
\end{cases}$$

### Weeding Success Rate (3D)

$$P_{success}^{3D} = P_{detect}(z_{sweep}^*) \cdot P\!\left(\|\mathbf{e}\|_2 \leq \epsilon_{pos,3D}\right)$$

$$= P_{detect,0} \cdot \left(\frac{z_{ref}}{z_{sweep}^*}\right)^{\!\beta}
  \cdot \left[1 - \exp\!\left(-\frac{\epsilon_{pos,3D}^2}{2\,\sigma_{hover,3D}^2}\right)\right]$$

---

## Key 3D Additions

- **Canopy height field**: Gaussian random field $h_{canopy}(x,y)$ with $\sigma_h = 0.15$ m and
  correlation length $\ell = 4$ m; bilinearly interpolated at each weed and drone position.
- **Altitude-dependent detection**: $P_{detect}(z) = 0.85 \cdot (z_{ref}/z)^{0.5}$; footprint
  radius $r_s(z) = 1.5 \cdot z / 2.0$ m; both degrade above $z_{ref} = 2.0$ m.
- **Stereo depth noise**: vertical weed position uncertainty $\sigma_z(z) = 0.05 \cdot (z/z_{ref})^2$ m;
  drives 3D weed centroid estimation error.
- **Optimal sweep altitude search**: 1D optimisation over $z \in [1.0, 3.5]$ m maximising
  expected detections per unit sweep time.
- **Sub-canopy descent profile**: two-segment waypoint plan (horizontal transit then vertical
  descent) with $\Delta h_{clear} = 0.20$ m clearance above $h_{canopy}$.
- **3D cost matrix for Hungarian**: Euclidean distance in $\mathbb{R}^3$ includes altitude delta
  to each weed's treatment depth.
- **3D PID controller**: extends 2D hover to full $(x, y, z)$ regulation; vertical channel
  additionally compensates stereo depth bias.
- **Altitude bounds**: $z \in [0.2, 4.0]$ m enforced at every simulation step.

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Field size | 40 x 40 m |
| Number of drones $N$ | 3 |
| Number of weeds $W$ | 30 |
| Mean canopy height $\bar{h}$ | 0.6 m |
| Canopy height std dev $\sigma_h$ | 0.15 m |
| Canopy correlation length $\ell$ | 4.0 m |
| Sensor footprint radius at $z_{ref}$ | 1.5 m |
| Reference altitude $z_{ref}$ | 2.0 m |
| Detection probability at $z_{ref}$, $P_{detect,0}$ | 0.85 |
| Detection degradation exponent $\beta$ | 0.5 |
| Stereo depth noise at $z_{ref}$, $\sigma_{z,0}$ | 0.05 m |
| Sweep altitude search range | 1.0 -- 3.5 m |
| Canopy clearance $\Delta h_{clear}$ | 0.20 m |
| Treatment altitude offset above canopy | 0.20 m |
| GPS planar noise $\sigma_{GPS}$ | 0.10 m |
| Treatment hover duration $T_{treat}$ | 2.0 s |
| 3D hover accuracy threshold $\epsilon_{pos,3D}$ | 0.20 m |
| PID gains $(K_p, K_i, K_d)$ | (2.0, 0.1, 0.5) |
| Altitude bounds | [0.2, 4.0] m |
| Simulation timestep $\Delta t$ | 0.05 s |
| Assignment algorithm | Hungarian (3D cost) |
| Random seed | 42 |

---

## Expected Output

- **Canopy height map**: pseudocolour $40 \times 40$ m heatmap of $h_{canopy}(x,y)$ with weed
  positions overlaid; colour bar showing 0.3 -- 0.9 m range; sweep altitude $z_{sweep}^*$ annotated.
- **Detection probability vs altitude curve**: $P_{detect}(z)$ and $r_s(z)$ plotted over
  $z \in [1.0, 3.5]$ m; optimal $z_{sweep}^*$ marked; shows the detection-altitude trade-off.
- **3D field overview**: Matplotlib 3D scatter of all weed positions (grey = missed, gold =
  detected) with Phase 1 boustrophedon paths at $z_{sweep}^*$ and Phase 2 descent trajectories
  for each drone colour-coded; canopy surface rendered as a semi-transparent mesh.
- **Altitude time series**: $z(t)$ for all three drones across the full mission; Phase 1 flat
  cruise at $z_{sweep}^*$, Phase 2 descent/ascent cycles per weed treatment visible.
- **Metrics figure (4-panel)**:
  (i) histogram of 3D hover RMS errors with $\epsilon_{pos,3D} = 0.20$ m threshold;
  (ii) bar chart comparing 3D Hungarian vs 2D Hungarian travel distance;
  (iii) vertical depth estimation error distribution ($\hat{z}_{weed} - z_{weed}$);
  (iv) weeding outcome pie chart (missed, detected but hover error, successfully treated).
- **Console metrics reported**:
  - Optimal sweep altitude $z_{sweep}^*$ (m) and expected detection count at that altitude
  - Weeds detected / total (Phase 1)
  - Phase 2 3D travel distance -- Hungarian (m)
  - Weeds successfully treated / detected (Phase 2)
  - Mean 3D hover RMS error (m) and mean vertical depth error (m)

---

## Extensions

1. **Multi-altitude sweep**: divide the sweep into two passes at different altitudes ($z_{high}$
   for coarse survey, $z_{low}$ for confirmation); model the joint detection probability
   $1 - (1 - P_{detect}(z_{high}))(1 - P_{detect}(z_{low}))$ and compare mission time vs
   detection improvement against the single-optimal-altitude baseline.
2. **Canopy-aware path planning**: replace the two-segment descent profile with a full 3D
   RRT* trajectory that avoids the canopy surface mesh; benchmark path length and computation
   time against the simple descent planner.
3. **Online canopy mapping**: fuse LiDAR returns during Phase 1 into a real-time occupancy voxel
   map; update treatment altitudes dynamically as the canopy map is refined; compare against
   offline canopy model assumed here.
4. **Heterogeneous drone altitudes**: allow each drone to independently optimise its sweep
   altitude per sub-strip based on local canopy density; evaluate whether strip-level altitude
   adaptation improves total detection count over the single global $z_{sweep}^*$.
5. **Vertical wind disturbance**: inject a vertical gust model $w_z(t) \sim \mathcal{GP}(0, k_w)$
   during sub-canopy hover; tune the $K_d$ gain of the altitude PID channel to maintain
   $\epsilon_{pos,3D}$ despite gusts; compare treatment success rate with and without feedforward
   gust compensation.

---

## Related Scenarios

- Original 2D version: [S070](../S070_swarm_weeding.md)
- 3D hover reference: [S063 Precision Hover Inspection](../S063_precision_hover_inspection.md)
- Canopy navigation reference: [S075 Greenhouse Autonomous Inspection](../S075_container_yard.md)
- Algorithmic reference: [S002 3D Upgrade](../../01_pursuit_evasion/3d/S002_3d_evasive_maneuver.md) (3D Gram-Schmidt, altitude bounds pattern)
- Multi-drone assignment: [S068 Multi-Drone Field Mapping](../S068_large_field_spray.md) (parallel sub-strip division)
