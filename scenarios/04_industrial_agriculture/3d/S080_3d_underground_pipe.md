# S080 3D Upgrade — Underground Pipe Inspection

**Domain**: Industrial & Agriculture | **Difficulty**: ⭐⭐⭐ | **Status**: `[ ]` Not Started
**Based on**: [S080 original](../S080_underground_pipe.md)

---

## What Changes in 3D

The original S080 treats the pipe as an essentially planar problem: the four ultrasonic sensors
point along body-frame ±x and ±y, the dead-reckoning integrator collapses the vertical component
to a fixed $\hat{z}$ factor of zero in straight segments, and all heading updates at elbows are
axis-aligned 90° rotations with no roll component. Crucially, the 360° wall-inspection problem is
never addressed — only four discrete ray directions are measured, leaving the upper and lower pipe
quadrants completely uninspected. In addition, the pipe network is a single linear trunk; real
sewer and industrial systems contain T-junctions and Y-branches that require autonomous traversal
decisions.

This 3D upgrade introduces:

1. **Cylindrical diameter constraint in full 3D** — the drone must remain within a cylinder of
   radius $r_{clear} = 0.25$ m centred on the continuously curved centreline in all three spatial
   dimensions, including vertical segments (pipe segments that run along ±z).
2. **360° wall-inspection coverage** — eight ultrasonic sensors distributed uniformly around the
   drone body frame (N, NE, E, SE, S, SW, W, NW at 45° intervals) plus a continuous circumferential
   scan model that tracks what arc of the inner pipe wall has been imaged by the forward camera as
   the drone rotates about its roll axis.
3. **3D branching junction traversal** — two T-junctions are added to the pipe network; the drone
   must detect the opening using a sensor asymmetry signature, choose the correct branch via a
   pre-loaded network map, and realign its heading to the new branch axis in 3D.
4. **GPS-denied localisation using pipe geometry** — instead of pure velocity integration, the
   drone fuses dead-reckoning with pipe-geometry landmarks (elbows and junctions) using an
   Extended Kalman Filter (EKF); each detected elbow or junction provides a position fix that
   resets the accumulated drift.

---

## Problem Definition

**Setup**: A micro-drone (body diameter $d_{drone} = 0.20$ m) navigates a 70 m underground pipe
network with inner diameter $D = 0.50$ m. The network consists of five straight segments connected
by three 90° elbows and two T-junctions. Segments run in all three principal axis directions,
including one fully vertical segment (±z), so the drone must manage buoyancy-equivalent thrust
variation when hovering in a vertical pipe bore. GPS is completely denied underground. Eight
ultrasonic sensors are distributed at 45° intervals in the drone body-frame plane perpendicular to
the current heading, providing full-circumference proximity readings. A forward-facing camera
captures synthetic inspection frames; by rolling the drone $\phi \in [-\pi, \pi]$ about its
forward axis as it advances, the camera traces a helical scan path that achieves 360° circumferential
coverage of the inner pipe wall.

**Roles**:
- **Micro-drone** (×1): body diameter 0.20 m; 8-axis ultrasonic array (45° spacing) + forward
  camera; advances at $v = 0.20$ m/s; controlled by an 8-sensor PD centering loop, a roll-scan
  scheduler, and a geometry-aided EKF localiser.
- **Pipe network**: inner diameter 0.50 m, total centreline arc 70 m; segments along +x, +z, +y,
  −x, +x axes; three 90° elbows at $s = 15, 30, 43$ m; two T-junctions at $s = 22, 55$ m with
  branch pipes branching off in ±y; walls seeded with $N_{crack} = 15$ crack patches.

**Objective**: Navigate the full primary 70 m path without wall contact. Achieve $\geq 95\%$ pipe
coverage. Achieve $\geq 90\%$ circumferential wall inspection coverage. Report: pipe coverage
percentage, mean centering error $\bar{e}_{ctr}$, wall-inspection coverage percentage, cracks
detected, and EKF localisation terminal error.

---

## Mathematical Model

### 3D Pipe Centreline and Diameter Constraint

The pipe centreline is parameterised by arc length $s \in [0, 70]$ m with a piecewise-constant
heading vector $\hat{\mathbf{h}}(s) \in \{\pm\hat{x}, \pm\hat{y}, \pm\hat{z}\}$. The full 3D
drone position is:

$$\mathbf{p}(t) = \bigl(x(t),\; y(t),\; z(t)\bigr)^\top$$

The radial offset from the pipe centreline at arc position $s$ is computed in the plane perpendicular
to the local heading:

$$\rho(t) = \bigl\|\mathbf{p}(t) - \mathbf{c}(s(t)) - \bigl[(\mathbf{p}(t) - \mathbf{c}(s(t))) \cdot \hat{\mathbf{h}}(s(t))\bigr]\hat{\mathbf{h}}(s(t))\bigr\|$$

The 3D safety constraint remains:

$$\rho(t) \leq r_{clear} - r_{drone} = 0.15 \; \text{m} \quad \forall \, t$$

For vertical segments where $\hat{\mathbf{h}} = \pm\hat{z}$, the lateral budget applies in the
x-y plane; the centering controller automatically adapts because the body-frame lateral axes
$\hat{\mathbf{e}}_1, \hat{\mathbf{e}}_2$ are recomputed from the current heading at every timestep.

### 8-Sensor Ultrasonic Array

Eight sensors are placed at body-frame angles $\psi_k = k \cdot 45°$ for $k = 0, 1, \ldots, 7$
in the plane perpendicular to $\hat{\mathbf{h}}$. The unit sensing vector for sensor $k$ is:

$$\hat{\mathbf{s}}_k = \cos\psi_k\,\hat{\mathbf{e}}_1 + \sin\psi_k\,\hat{\mathbf{e}}_2$$

where $\hat{\mathbf{e}}_1, \hat{\mathbf{e}}_2$ are orthonormal body-frame lateral axes computed via
Gram-Schmidt from $\hat{\mathbf{h}}$ and $\hat{z}$ (or $\hat{x}$ when $\hat{\mathbf{h}} \parallel \hat{z}$):

$$\hat{\mathbf{e}}_1 = \frac{\hat{z} - (\hat{z} \cdot \hat{\mathbf{h}})\hat{\mathbf{h}}}{\|\hat{z} - (\hat{z} \cdot \hat{\mathbf{h}})\hat{\mathbf{h}}\|}, \qquad \hat{\mathbf{e}}_2 = \hat{\mathbf{h}} \times \hat{\mathbf{e}}_1$$

The true wall distance along sensor $k$ from the drone centre is:

$$d_k^{true}(t) = r_{clear} - \hat{\mathbf{s}}_k \cdot \boldsymbol{\delta}_\perp(t)$$

where $\boldsymbol{\delta}_\perp(t) = \mathbf{p}(t) - \mathbf{c}(s(t)) - [(\mathbf{p}(t) - \mathbf{c}(s(t))) \cdot \hat{\mathbf{h}}]\hat{\mathbf{h}}$ is the lateral offset vector.

The measured distance with additive Gaussian noise is:

$$d_k^{meas}(t) = d_k^{true}(t) + \eta_k(t), \qquad \eta_k(t) \sim \mathcal{N}(0,\;\sigma_{wall}^2)$$

with $\sigma_{wall} = 0.005$ m.

### 8-Sensor PD Centering Controller

The lateral error is reconstructed by projecting the noisy sensor readings back to body-frame
lateral components. Define the lateral force commands:

$$\mathbf{u}_\perp(t) = \sum_{k=0}^{7} \left(r_{clear} - d_k^{meas}(t)\right)\hat{\mathbf{s}}_k$$

This is a weighted radial-push: each sensor vote pushes the drone away from the nearest wall in
the direction of that sensor. The full velocity command is:

$$\dot{\mathbf{p}}(t) = v\,\hat{\mathbf{h}}(t) + K_p\,\mathbf{u}_\perp(t) + K_d\,\dot{\mathbf{u}}_\perp(t)$$

with gains $K_p = 2.0$ s$^{-1}$ and $K_d = 0.40$ s. For vertical segments the identical law
applies because $\mathbf{u}_\perp$ is already confined to the plane perpendicular to $\hat{\mathbf{h}}$.

### 360° Circumferential Inspection Coverage

The drone rolls about its heading axis at an angular rate $\dot{\phi}$, causing the forward camera
boresight to trace a helix on the pipe wall. The body-roll angle at time $t$ is:

$$\phi(t) = \phi_0 + \dot{\phi}\,t, \qquad \dot{\phi} = 2\pi / T_{roll}$$

The camera boresight points at circumferential wall angle $\psi_{cam}(t) = \phi(t)$ in the
body frame. The arc of inner pipe wall imaged per full drone advance of $\Delta s_{frame}$ is a
helical strip of width $w_{cam}$:

$$w_{cam} = 2\,r_{clear}\,\tan(\theta_{FOV}/2)$$

Coverage completeness requires the drone to advance at least $\Delta s_{full}$ before all
circumferential angles $[0, 2\pi]$ are visited:

$$\Delta s_{full} = \frac{v}{\dot{\phi}} \cdot 2\pi = v \cdot T_{roll}$$

The cumulative circumferential coverage fraction at arc position $s$ is:

$$C_{circ}(s) = \min\!\left(1,\; \frac{s}{\Delta s_{full}}\right) \times 100\%$$

Target: $C_{circ} \geq 90\%$ by mission end.

### 3D Junction Detection and Branch Selection

At a T-junction along the heading axis, one of the lateral sensor pairs (e.g. along $\hat{\mathbf{e}}_1$)
suddenly reads a much larger distance, indicating an opening:

$$d_{open}(t) = \max_k d_k^{meas}(t) > d_{junction}, \qquad d_{junction} = 0.40 \; \text{m}$$

The opening direction $\hat{\mathbf{b}}$ is the sensor unit vector $\hat{\mathbf{s}}_{k^*}$
corresponding to the maximum reading:

$$k^* = \arg\max_k d_k^{meas}(t)$$

The drone consults a pre-loaded network adjacency map $\mathcal{G}$ to decide whether to turn
into the branch or continue straight. The heading update for a branch turn uses SLERP over
$t_{blend} = 0.5$ s:

$$\hat{\mathbf{h}}(t) = \text{SLERP}\!\left(\hat{\mathbf{h}}_{old},\; \hat{\mathbf{b}},\; \min\!\left(\frac{t - t_{trigger}}{t_{blend}}, 1\right)\right)$$

### Geometry-Aided EKF Localisation

The state vector is $\mathbf{x}_{EKF} = (\hat{x}, \hat{y}, \hat{z}, \hat{v})^\top$ (position
and forward speed estimate). The prediction step uses the commanded velocity:

$$\hat{\mathbf{x}}_{t|t-1} = \hat{\mathbf{x}}_{t-1|t-1} + \begin{bmatrix} v\,\hat{\mathbf{h}}\,\Delta t \\ 0 \end{bmatrix}$$

$$\mathbf{P}_{t|t-1} = \mathbf{F}\,\mathbf{P}_{t-1|t-1}\,\mathbf{F}^\top + \mathbf{Q}$$

where $\mathbf{Q} = \text{diag}(\sigma_v^2\Delta t^2, \sigma_v^2\Delta t^2, \sigma_v^2\Delta t^2, \sigma_a^2\Delta t^2)$
with $\sigma_v = 0.002$ m/s and $\sigma_a = 0.001$ m/s$^2$.

When an elbow or junction landmark is detected (via the sensor asymmetry condition), the known
landmark position $\mathbf{p}_{LM}$ from the network map provides a 3D position measurement:

$$\mathbf{z}_{LM} = \mathbf{p}_{LM} + \boldsymbol{\nu}, \qquad \boldsymbol{\nu} \sim \mathcal{N}(\mathbf{0},\;\mathbf{R}_{LM})$$

$$\mathbf{R}_{LM} = \text{diag}(0.01, 0.01, 0.01) \; \text{m}^2$$

The EKF update equations are:

$$\mathbf{K}_t = \mathbf{P}_{t|t-1}\,\mathbf{H}^\top\left(\mathbf{H}\,\mathbf{P}_{t|t-1}\,\mathbf{H}^\top + \mathbf{R}_{LM}\right)^{-1}$$

$$\hat{\mathbf{x}}_{t|t} = \hat{\mathbf{x}}_{t|t-1} + \mathbf{K}_t\!\left(\mathbf{z}_{LM} - \mathbf{H}\,\hat{\mathbf{x}}_{t|t-1}\right)$$

$$\mathbf{P}_{t|t} = (\mathbf{I} - \mathbf{K}_t\,\mathbf{H})\,\mathbf{P}_{t|t-1}$$

where $\mathbf{H} = [\mathbf{I}_{3\times3} \mid \mathbf{0}_{3\times1}]$ selects the position components.
Between landmarks, position uncertainty grows as $\sigma_{DR}(t) = \sigma_v\sqrt{t}$; at each
landmark fix it is reset to $\approx 0.01$ m.

### 3D Bend and Junction Detection (Unified Asymmetry Condition)

A feature is triggered when any sensor pair in the perpendicular plane exhibits asymmetry
exceeding a threshold. For diametrically opposite sensor pair $(k, k+4)$:

$$\Delta_k(t) = d_{k+4}^{meas}(t) - d_k^{meas}(t), \quad k \in \{0, 1, 2, 3\}$$

- Elbow: $\max_k |\Delta_k(t)| > \Delta d_{bend} = 0.06$ m and $\max_k d_k^{meas}(t) \leq d_{junction}$
- Junction: $\max_k d_k^{meas}(t) > d_{junction} = 0.40$ m

### Centering Error and Coverage Metrics

RMS lateral centering error over the full $T$-step mission:

$$\bar{e}_{ctr} = \sqrt{\frac{1}{T}\sum_{t=1}^{T}\rho(t)^2}, \qquad \text{target: } \bar{e}_{ctr} \leq 0.03 \; \text{m}$$

Pipe longitudinal coverage:

$$C_{pipe} = \frac{\text{arc length with } \rho(t) \leq 0.15 \, \text{m}}{L_{pipe}} \times 100\%, \qquad \text{target: } \geq 95\%$$

Circumferential inspection coverage:

$$C_{circ} = \frac{|\{\psi : \psi \text{ has been imaged}\}|}{2\pi} \times 100\%, \qquad \text{target: } \geq 90\%$$

---

## Key 3D Additions

- **8-sensor circumferential array**: all 45°-spaced sensors active in the plane perpendicular to
  $\hat{\mathbf{h}}$, automatically recomputed for vertical pipe segments via Gram-Schmidt.
- **Vertical segment handling**: when $\hat{\mathbf{h}} = \pm\hat{z}$, the lateral axes become
  $\hat{\mathbf{e}}_1 = \hat{x}$, $\hat{\mathbf{e}}_2 = \hat{y}$; thrust in the heading direction
  must overcome gravity (upward segment) or be reduced (downward segment) by $\Delta T = mg = 0.3$ N.
- **Roll-scan helical inspection**: drone rolls at $\dot{\phi} = 2\pi / T_{roll}$ (one full rotation
  per $T_{roll} = 2.5$ s) while advancing at $v = 0.20$ m/s; the camera boresight traces a helix
  with pitch $p_{helix} = v \cdot T_{roll} = 0.50$ m/revolution.
- **T-junction detection and selection**: sensor max-reading asymmetry detects junction openings;
  pre-loaded network graph selects the correct branch heading in 3D.
- **EKF landmark fusion**: elbow and junction positions serve as 3D position fixes, bounding
  accumulated dead-reckoning drift to $< 0.05$ m between landmarks even under $\sigma_v = 0.002$ m/s
  velocity noise.
- **3D trajectory visualisation**: full Matplotlib 3D axes showing x/y/z path variation, separate
  altitude time-series, and a circumferential coverage polar diagram.

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Pipe inner diameter $D$ | 0.50 m |
| Pipe centreline total arc $L_{pipe}$ | 70 m |
| Number of 90° elbows | 3 |
| Number of T-junctions | 2 |
| Elbow arc positions | 15 m, 30 m, 43 m |
| Junction arc positions | 22 m, 55 m |
| Drone body diameter $d_{drone}$ | 0.20 m |
| Lateral safety budget $r_{clear} - r_{drone}$ | 0.15 m |
| Forward advance speed $v$ | 0.20 m/s |
| Number of ultrasonic sensors | 8 (45° spacing) |
| Ultrasonic noise std $\sigma_{wall}$ | 0.005 m |
| Centering proportional gain $K_p$ | 2.0 s$^{-1}$ |
| Centering derivative gain $K_d$ | 0.40 s |
| Elbow/bend detection threshold $\Delta d_{bend}$ | 0.06 m |
| Junction detection threshold $d_{junction}$ | 0.40 m |
| Heading blend time $t_{blend}$ | 0.5 s |
| Roll scan period $T_{roll}$ | 2.5 s |
| Camera FOV half-angle $\theta_{FOV}/2$ | 30° |
| Helix pitch per revolution $p_{helix}$ | 0.50 m |
| Dead-reckoning velocity noise $\sigma_v$ | 0.002 m/s |
| EKF landmark measurement noise | 0.01 m (per axis) |
| Process noise $\sigma_a$ | 0.001 m/s$^2$ |
| Wall-contact sensor threshold $d_{contact}$ | 0.02 m |
| Crack count $N_{crack}$ | 15 |
| Crack detection intensity threshold $I_{threshold}$ | 80 / 255 |
| Crack detection longitudinal range $\Delta s_{crack}$ | 0.30 m |
| Simulation timestep $\Delta t$ | 0.05 s |
| Vertical segment thrust offset $\Delta T$ | 0.30 N |
| Target pipe longitudinal coverage $C_{pipe}$ | $\geq 95\%$ |
| Target circumferential coverage $C_{circ}$ | $\geq 90\%$ |
| Target RMS centering error $\bar{e}_{ctr}$ | $\leq 0.03$ m |

---

## Expected Output

- **3D pipe network trajectory plot**: Matplotlib 3D axes showing the five-segment network
  centreline (black dashed), the true drone path (red), and the EKF position estimate (orange);
  elbows marked as purple spheres; T-junctions as cyan diamonds; detected cracks as green crosses,
  undetected as grey crosses; pipe bore rendered as a semi-transparent grey cylinder of radius
  $r_{clear}$ around each segment.
- **Altitude time series**: $z(t)$ of the true path and EKF estimate overlaid, showing the vertical
  segment traversal; EKF uncertainty band ($\pm 2\sigma$) shaded; landmark fix events marked as
  vertical dashed lines.
- **Centering error time series**: radial offset $\rho(t)$ in cm plotted over simulation time;
  red dashed line at the 15 cm safety limit; green dashed line at the 3 cm target; junction and
  elbow events annotated.
- **Circumferential coverage polar diagram**: polar histogram of wall angles $\psi$ imaged over
  the full mission; target $\geq 90\%$ shown as a filled reference circle; uncovered arcs
  highlighted in red.
- **EKF localisation error plot**: Euclidean error $\|\hat{\mathbf{p}}_{EKF} - \mathbf{p}_{true}\|$
  over time; landmark fix events shown as step reductions in error; comparison with pure
  dead-reckoning drift $\sigma_{DR}(t) = \sigma_v\sqrt{t}$ overlaid.
- **8-sensor wall-distance heatmap**: time vs sensor-index heat map of $d_k^{meas}(t)$, showing
  the centering transients at each elbow and junction.
- **Animation (GIF)**: cross-sectional view (perpendicular to heading) showing the 8 sensor rays
  as coloured spokes reaching from the drone body to the pipe wall; spoke colour encodes distance
  (green = near nominal, red = close to wall); the drone rolls visibly as the roll-scan progresses;
  frame rate 10 fps.
- **Console metrics table**: pipe longitudinal coverage % (target $\geq 95\%$), circumferential
  coverage % (target $\geq 90\%$), RMS centering error in cm (target $\leq 3$ cm), wall contact
  count (target = 0), cracks detected / total, EKF terminal position error vs pure dead-reckoning
  terminal drift.

---

## Extensions

1. **T-junction decision under map uncertainty**: add localisation noise large enough to create
   ambiguity about which junction has been reached; implement a Bayesian decision rule that picks
   the branch whose heading best aligns with the observed sensor asymmetry direction.
2. **Partial blockage detection in 3D**: seed one vertical segment with an asymmetric obstruction
   that reduces wall clearance on only one quadrant; the 8-sensor array must distinguish a
   partial blockage (one or two sensors close, others normal) from a genuine bend.
3. **Optimal roll-scan rate**: derive the minimum $\dot{\phi}$ required to achieve
   $C_{circ} \geq 90\%$ at a given $v$ and camera FOV; show the trade-off between scan speed
   (higher $\dot{\phi}$ stresses the actuators) and coverage.
4. **Pipe diameter variation**: extend the network to include a reducer fitting where $D$ drops
   from 0.50 m to 0.35 m; the EKF must detect the constraint change from sensor readings and
   tighten the lateral budget accordingly.
5. **Branch pipe full coverage**: after completing the primary trunk, navigate into each branch
   pipe and return; implement a depth-first traversal of the network graph using EKF position
   estimates to determine when the drone has fully exited a branch and is back at the junction.

---

## Related Scenarios

- Original 2D/3D-limited version: [S080](../S080_underground_pipe.md)
- GPS-denied indoor navigation: [S069 Warehouse Inventory](../S069_warehouse_inventory.md)
- Tunnel navigation reference: [S075 Tunnel Navigation](../S075_container_yard.md)
- 3D mapping in confined space: [S074 Mine 3D Mapping](../S074_mine_mapping.md)
- Surface crack detection: [S071 Bridge Underside Inspection](../S071_bridge_inspection.md)
- Probabilistic state estimation: [S013 Particle Filter Intercept](../../01_pursuit_evasion/S013_particle_filter_intercept.md)
