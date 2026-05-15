# S053 3D Upgrade — Coral Reef 3D Mapping

**Domain**: Environmental & SAR | **Difficulty**: ⭐⭐⭐⭐ | **Status**: `[ ]` Not Started
**Based on**: [S053 original](../S053_coral_reef.md)

---

## What Changes in 3D

The original S053 treats the reef survey as a 2D path-planning problem: the drone flies at
a fixed set of discrete altitude levels (`ALT_LEVELS = [5.0, 3.5, 2.0]` m), the spiral
waypoints are generated in the x-y plane and altitude is simply appended as a constant per
level, and the visibility check uses a purely downward-facing footprint cone with no
consideration of oblique horizontal viewing angles. The heightmap relief drives the GSD
calculation but never influences the drone's lateral trajectory — the path is the same
regardless of where bommies actually stand.

This 3D upgrade makes altitude a continuously controlled state variable. The drone tracks a
desired standoff surface $z_{cmd}(x, y) = h(x, y) + \Delta z_{standoff}$ in real time,
banking the camera both downward and sideways to target steep reef faces through a full 3D
pan-tilt gimbal model. Stereo photogrammetry is extended to a true point-cloud accumulator
that records the 3D intersection of left/right camera rays for every valid frame pair, giving
physically grounded reconstruction density rather than the analytical approximation used in
the 2D version. Underwater current disturbance is injected as a constant horizontal drift
and rejected by a PID depth-and-lateral controller.

---

## Problem Definition

**Setup**: A coral reef occupying a $60 \times 40$ m footprint lies at mean datum $z = 0$ m.
The reef morphology heightmap $h(x, y) \in [-3.0,\; 0.5]$ m (12 Gaussian bommies plus
background undulation) is now used as a live terrain reference for the drone's altitude
controller rather than a post-hoc GSD formula. The drone carries a stereo camera pair
(baseline $B = 0.12$ m, FOV $\alpha_{fov} = 70°$) mounted on a two-axis gimbal that can
pitch $\phi_{cam} \in [-90°, -20°]$ (downward) and yaw $\psi_{cam} \in [-45°, +45°]$
(lateral pan). An underwater current of constant velocity $\mathbf{v}_c = (v_{cx}, v_{cy}, 0)$
acts on the drone throughout the mission.

**Roles**:
- **AUV/drone**: single platform with full 3D position state $(x, y, z)$ and velocity
  $(v_x, v_y, v_z)$; altitude controlled by a PID loop; lateral trajectory follows one of
  three 3D path strategies.
- **Gimbal**: two-axis pan-tilt controller that orients the stereo camera toward a nominated
  target surface point at each waypoint, enabling oblique views of vertical reef faces.
- **Reef surface**: static heightmap $H \in \mathbb{R}^{G_x \times G_y}$; each cell carries
  a 3D surface normal $\hat{\mathbf{n}}_{ij}$; cells are tagged as vertical face, overhang,
  or horizontal substrate to drive targeted viewing.
- **Point cloud**: accumulator that records, for each valid stereo frame, the 3D triangulated
  positions of matched feature points together with their triangulation uncertainty ellipsoids.

**Objective**: Maximise the fraction of reef surface cells with $\geq N_{min} = 2$ valid
stereo views while maintaining altitude tracking error $\sigma_z \leq 0.15$ m against
current disturbance, staying within the battery path budget $L_{max} = 500$ m, and achieving
a mean 3D reconstruction error $\bar{\varepsilon} \leq 8$ mm over the full reef volume.

**Comparison strategies**:
1. **Constant-altitude spiral** (2D baseline): altitude held per level at $\{5.0, 3.5, 2.0\}$ m;
   downward-only camera; gimbal fixed at $\phi_{cam} = -90°$.
2. **Terrain-following spiral**: altitude PID tracks $h(x,y) + 3.0$ m; camera still nadir-only.
3. **Terrain-following + oblique gimbal**: altitude PID + gimbal pans toward steep faces
   detected by $|\hat{\mathbf{n}}_{ij} \cdot \hat{\mathbf{z}}| < 0.5$; covers vertical surfaces
   that nadir views cannot resolve.

---

## Mathematical Model

### 3D Altitude Control

The drone state is $\mathbf{q} = (x, y, z, \dot{x}, \dot{y}, \dot{z})^\top$. The desired
altitude above the local reef surface at the current $(x, y)$ is:

$$z_{cmd}(x, y, t) = h\!\left(x(t),\, y(t)\right) + \Delta z_{standoff}$$

with $\Delta z_{standoff} = 3.0$ m. The altitude PID controller outputs a vertical
acceleration command:

$$a_z = K_p \bigl(z_{cmd} - z\bigr) + K_d \bigl(\dot{z}_{cmd} - \dot{z}\bigr) + K_i \int_0^t \bigl(z_{cmd} - z\bigr)\, d\tau$$

where $K_p = 2.0$, $K_d = 1.2$, $K_i = 0.05$ (tuned for $\Delta z_{standoff} = 3.0$ m and
reef relief $\leq 3.5$ m). The desired altitude derivative is computed by the terrain gradient
projected onto the horizontal velocity:

$$\dot{z}_{cmd} = \frac{\partial h}{\partial x}\dot{x} + \frac{\partial h}{\partial y}\dot{y}$$

### Current Disturbance and Rejection

A steady horizontal current $\mathbf{v}_c = (v_{cx}, v_{cy})$ displaces the drone from its
planned $(x_p, y_p)$ trajectory. The lateral PID controller computes cross-track correction:

$$\mathbf{a}_{lat} = K_{p,xy}\bigl(\mathbf{p}_{plan}(t) - \mathbf{p}_{xy}(t)\bigr)
  + K_{d,xy}\bigl(\dot{\mathbf{p}}_{plan} - \dot{\mathbf{p}}_{xy}\bigr)$$

with $K_{p,xy} = 1.5$, $K_{d,xy} = 0.8$. Net drone acceleration:

$$\ddot{\mathbf{p}} = \mathbf{a}_{lat} + a_z\hat{\mathbf{z}} + \mathbf{v}_c^{(applied)}$$

where the current appears as an additive horizontal drift applied before the controller
corrects for it, modelling a realistic feedback lag.

### 3D Gimbal Pointing Law

At each waypoint $\mathbf{w}_k$, the gimbal target is the surface point that maximises
newly uncovered area subject to the oblique visibility constraint. For a candidate target
$\mathbf{p}_{tgt} = (x_t, y_t, h(x_t, y_t))^\top$, the unit bore-sight vector is:

$$\hat{\mathbf{b}} = \frac{\mathbf{p}_{tgt} - \mathbf{w}_k}{\|\mathbf{p}_{tgt} - \mathbf{w}_k\|}$$

The gimbal pitch and yaw angles are:

$$\phi_{cam} = \arcsin\!\left(-\hat{b}_z\right), \qquad
  \psi_{cam} = \arctan2\!\left(\hat{b}_y,\; \hat{b}_x\right) - \psi_{drone}$$

Both angles are clamped to their mechanical limits:
$\phi_{cam} \in [-90°, -20°]$, $\psi_{cam} \in [-45°, +45°]$.

### Full 3D Visibility Condition

A surface cell $\mathbf{p}_{ij}$ is visible from drone position $\mathbf{d}$ with gimbal
bore-sight $\hat{\mathbf{b}}$ if and only if:

**1. Incidence angle:**

$$\cos\theta_{ij} = \hat{\mathbf{n}}_{ij} \cdot \frac{\mathbf{d} - \mathbf{p}_{ij}}{\|\mathbf{d} - \mathbf{p}_{ij}\|} \geq \cos\theta_{max}$$

with $\theta_{max} = 45°$.

**2. Within 3D camera frustum:** the point lies inside the double cone centered on $\hat{\mathbf{b}}$:

$$\frac{(\mathbf{p}_{ij} - \mathbf{d}) \cdot \hat{\mathbf{b}}}{\|\mathbf{p}_{ij} - \mathbf{d}\|} \geq \cos\!\left(\frac{\alpha_{fov}}{2}\right)$$

and the slant range satisfies $r_{min} \leq \|\mathbf{p}_{ij} - \mathbf{d}\| \leq r_{max}$
with $r_{min} = 0.5$ m, $r_{max} = 8.0$ m.

**3. No heightmap occlusion** (ray-marching test with step $\delta_{ray} = 0.25$ m):

$$h(x_r, y_r) < d_z - \|\mathbf{p}_r - \mathbf{d}\| \cdot |\hat{b}_z|
  \quad \forall\; (x_r, y_r) \text{ sampled along ray}$$

### Stereo Triangulation and Point Cloud

For a stereo pair with baseline $B = 0.12$ m, the 3D position of a matched feature at
disparity $d_{px}$ is:

$$Z_{feat} = \frac{B \cdot f}{d_{px}}, \qquad
  X_{feat} = \frac{(u - c_x) \cdot Z_{feat}}{f}, \qquad
  Y_{feat} = \frac{(v - c_y) \cdot Z_{feat}}{f}$$

where $(u, v)$ is the left-image pixel coordinate of the feature, $(c_x, c_y)$ the principal
point, and $f$ the focal length in pixels ($f_{px} = f_{mm} \cdot W_{px} / s_w$).
The triangulation depth uncertainty (1-sigma) is:

$$\sigma_Z = \frac{Z_{feat}^2}{B \cdot f_{px}} \cdot \sigma_{d}$$

with $\sigma_d = 0.5$ px disparity noise. The accumulated point cloud $\mathcal{P}$ is the
union of all triangulated feature positions across all valid stereo frames.

### 3D Reconstruction Error

The mean reconstruction error over all accumulated points:

$$\bar{\varepsilon} = \frac{1}{|\mathcal{P}|} \sum_{\mathbf{p} \in \mathcal{P}} \sigma_Z(\mathbf{p})$$

For the terrain-following strategy at $\Delta z_{standoff} = 3.0$ m above the reef crest
($h_{max} = 0.5$ m), the slant range to a vertical face at the same altitude is
$r_{slant} \approx 3.5$ m, giving:

$$\sigma_Z = \frac{3.5^2}{0.12 \cdot 6349} \cdot 0.5 \approx 0.008 \text{ m}$$

confirming the 8 mm target is achievable for oblique stereo at this standoff.

### Coverage Metric (unchanged from S053)

$$C = \frac{1}{N_{surf}} \sum_{i,j} \mathbf{1}\bigl[V_{ij} \geq N_{min}\bigr] \geq 0.90$$

where $V_{ij}$ now counts views from both nadir and oblique gimbal positions.

### 3D Path Length with Altitude Excursions

$$L_{path}^{3D} = \sum_{k=1}^{K-1} \|\mathbf{w}_{k+1} - \mathbf{w}_k\|$$

Because terrain-following continuously varies $z_k$, the 3D path length always exceeds the
equivalent 2D lawnmower path length by the total vertical excursion:

$$\Delta L_{vert} = \sum_{k=1}^{K-1} |z_{k+1} - z_k|$$

The constraint $L_{path}^{3D} \leq L_{max} = 500$ m must still be satisfied.

---

## Key 3D Additions

- **Altitude PID controller**: $z_{cmd}(x, y)$ tracks terrain contour continuously; $K_p = 2.0$, $K_d = 1.2$, $K_i = 0.05$.
- **Current disturbance model**: constant horizontal drift $\mathbf{v}_c$ (e.g., 0.3 m/s at 45°) with lateral PID rejection.
- **Two-axis gimbal model**: pitch and yaw clamped to mechanical limits; bore-sight computed from 3D target vector.
- **Oblique visibility frustum**: 3D cone condition replaces 2D circular footprint; enables viewing of vertical reef faces.
- **Ray-marching occlusion**: heightmap intersection test along the 3D line of sight at 0.25 m step resolution.
- **Stereo triangulation model**: per-feature $\sigma_Z$ uncertainty from disparity noise; builds physical point-cloud density.
- **Vertical face targeting**: cells with $|\hat{\mathbf{n}}_{ij} \cdot \hat{\mathbf{z}}| < 0.5$ are flagged; gimbal pans toward them at each strip crossing.
- **3D trajectory visualization**: full $(x, y, z(t))$ path with altitude time series and gimbal orientation vectors shown.

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Reef footprint | 60 × 40 m |
| Heightmap resolution $\delta$ | 0.5 m |
| Reef height range | −3.0 to +0.5 m |
| Number of bommies $N_B$ | 12 |
| Altitude standoff $\Delta z_{standoff}$ | 3.0 m above local reef |
| Drone altitude range $z$ | 1.5 – 6.5 m |
| Altitude PID gains $(K_p, K_d, K_i)$ | (2.0, 1.2, 0.05) |
| Current speed $\|\mathbf{v}_c\|$ | 0.3 m/s at 45° bearing |
| Lateral PID gains $(K_{p,xy}, K_{d,xy})$ | (1.5, 0.8) |
| Stereo baseline $B$ | 0.12 m |
| Camera FOV $\alpha_{fov}$ | 70° |
| Focal length $f$ | 4.5 mm |
| Sensor width $s_w$ | 6.3 mm |
| Image resolution | 4000 × 3000 px |
| Disparity noise $\sigma_d$ | 0.5 px |
| Max incidence angle $\theta_{max}$ | 45° |
| Gimbal pitch range $\phi_{cam}$ | −90° to −20° |
| Gimbal yaw range $\psi_{cam}$ | −45° to +45° |
| Slant range limits $(r_{min}, r_{max})$ | 0.5 – 8.0 m |
| Ray-march step $\delta_{ray}$ | 0.25 m |
| Minimum views per cell $N_{min}$ | 2 |
| Battery path budget $L_{max}$ | 500 m |
| Cruise speed $v$ | 1.5 m/s |
| Target coverage $C$ | ≥ 90% |
| Target reconstruction error $\bar{\varepsilon}$ | ≤ 8 mm |
| Target altitude tracking error $\sigma_z$ | ≤ 0.15 m |

---

## Expected Output

- **3D trajectory plot**: Matplotlib 3D axes showing the reef heightmap surface coloured by
  depth, the drone's continuous 3D path coloured by altitude (terrain-following strategy shows
  visible undulation matching the bommie profile), and gimbal bore-sight vectors drawn as
  short arrows at every 10th waypoint.
- **Altitude time series**: $z_{drone}(t)$ vs $z_{cmd}(t)$ with shaded tracking-error band;
  overlaid with the terrain height $h(x(t), y(t))$ beneath the drone; current-induced lateral
  drift shown on a secondary axis.
- **Coverage heatmap comparison**: three side-by-side 2D grids (constant-altitude, terrain-
  following nadir, terrain-following oblique) showing per-cell view count $V_{ij}$; vertical
  face cells highlighted with a separate colour; coverage fractions annotated.
- **Point cloud density map**: top-down projection of $|\mathcal{P}|$ in each 0.5 m grid cell,
  colour-scaled from white (0 pts/m²) to dark blue (≥ 500 pts/m²); steep-face regions show
  higher density with the oblique strategy.
- **Reconstruction error map**: per-cell $\bar{\varepsilon}$ (mean $\sigma_Z$ of all triangulated
  points within each cell), with red contours where $\bar{\varepsilon} > 8$ mm.
- **Strategy comparison bar chart**: coverage $C$, mean $\bar{\varepsilon}$, path length
  $L_{path}^{3D}$, and vertical face coverage fraction for all three strategies.
- **Altitude tracking RMS**: $\sigma_z$ vs current speed sweep from 0 to 0.5 m/s, showing
  the PID's rejection bandwidth and the 0.15 m tolerance threshold.
- **Animation (GIF)**: 3D rotating view of the reef with the drone traversing its path;
  surface cells colour-shift from grey to blue to green as view count accumulates; gimbal
  target point shown as a red dot tracking across steep faces.

---

## Extensions

1. **Next-best-view replanning with 3D occlusion**: after each completed strip, re-evaluate
   which reef surface cells remain under-covered, select the next waypoint using greedy NBV
   maximisation over the full 3D frustum-occlusion model, and compare against the pre-planned
   spiral at equal path length budget.
2. **Adaptive stereo baseline**: model a variable-baseline stereo rig ($B \in [0.06, 0.24]$ m,
   adjustable in flight); optimise $B$ per waypoint to keep $\sigma_Z \leq 5$ mm over the
   expected slant range, trading off accuracy against rig complexity.
3. **Multi-AUV parallel survey with 3D collision avoidance**: split the reef into Voronoi
   partitions (relates to S049); assign one drone per partition; enforce minimum 3D separation
   $\|\mathbf{p}_i - \mathbf{p}_j\| \geq d_{safe} = 2.0$ m using potential-field repulsion.
4. **SfM tie-point graph simulation**: simulate SIFT feature extraction and epipolar matching
   between consecutive frame pairs; build the full visibility graph and compute its algebraic
   connectivity to predict reconstruction robustness before flying.
5. **Surge current rejection with model-predictive control**: replace the PID lateral
   controller with a 3-step MPC that predicts the current's effect over the next 3 waypoints
   and pre-compensates the trajectory, reducing steady-state cross-track error from ~0.2 m
   to < 0.05 m.

---

## Related Scenarios

- Original 2D version: [S053 Coral Reef 3D Reconstruction](../S053_coral_reef.md)
- Prerequisite coverage: [S048 Lawnmower Coverage](../S048_lawnmower.md)
- Swarm extension: [S049 Dynamic Zone Assignment](../S049_dynamic_zone.md), [S050 Swarm SLAM](../S050_slam.md)
- Industrial 3D analog: [S065 Building 3D Scan Path](../../04_industrial_agriculture/S065_building_3d_scan_path.md)
- Confined-space photogrammetry: [S074 Mine 3D Mapping](../../04_industrial_agriculture/S074_mine_3d_mapping.md)
- Pursuit & evasion 3D reference format: [S002 3D Upgrade](../../01_pursuit_evasion/3d/S002_3d_evasive_maneuver.md)
