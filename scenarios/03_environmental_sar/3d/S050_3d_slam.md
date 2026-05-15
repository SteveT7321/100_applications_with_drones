# S050 3D Upgrade — Swarm SLAM

**Domain**: Environmental & SAR | **Difficulty**: ⭐⭐⭐⭐⭐ | **Status**: `[ ]` Not Started
**Based on**: [S050 original](../S050_slam.md)

---

## What Changes in 3D

The original S050 constrains all drones to a flat 2D plane: pose is $(x, y, \theta)$, landmarks are
2D points $(l_x, l_y)$, and the observation model uses only horizontal range and bearing. Altitude
is implicitly fixed and never modelled. This means the SLAM filter cannot distinguish landmarks at
different heights, frontiers are defined only on a 2D grid, and loop-closure geometry is entirely
planar.

This 3D upgrade lifts every component into full volumetric space:

- **Drone pose** extended to $(x, y, z, \phi, \theta, \psi)$ — position plus roll/pitch/yaw. For
  hover-capable drones at low speed, roll and pitch are small; the primary additions are $z$ and
  $\psi$ (heading), giving a reduced navigation state $(x, y, z, \psi)$.
- **Landmark state** extended to 3D: each feature is $(l_x, l_y, l_z)$, enabling the mapping of
  elevated structures (tree canopies, antenna masts, building facades, cliff faces).
- **Observation model** extended to range-bearing-elevation $(\rho, \phi, \epsilon)$ — a spherical
  coordinate measurement from the sensor frame.
- **Occupancy representation** replaced by a **3D voxel grid** (binary or log-odds) instead of a
  2D cell grid; frontier detection operates on voxel face boundaries in all six face directions.
- **Altitude-separated frontiers**: unexplored voxel surfaces at different height layers are
  treated independently; drones are assigned to altitude bands to avoid redundant vertical overlap.
- **3D loop closure**: when a drone returns to a previously visited volume, the accumulated 3D pose
  drift is corrected via a 3D pose-graph back-end using SE(3) constraints.
- **3D map merging**: the SVD rigid-alignment step is promoted from 2D (translation + one rotation
  angle) to full 3D (translation vector $\mathbf{t} \in \mathbb{R}^3$ + rotation matrix
  $\mathbf{R} \in SO(3)$), requiring at least 4 non-coplanar common landmarks.

---

## Problem Definition

**Setup**: A $200 \times 200 \times 40$ m volumetric environment is completely unknown at mission
start. The terrain contains $M_{true} = 50$ static 3D landmarks (treetops, rooftop antennae,
rock outcrops at varying heights) uniformly distributed in the volume with $z \in [0, 30]$ m.
No GPS is available. A swarm of $N = 4$ drones is deployed from a common origin at $z = 5$ m.
Each drone carries a 3D range-bearing-elevation sensor (e.g., a solid-state LiDAR cluster or
stereo-depth camera with wide FOV) that detects landmarks within spherical range $r_{sense} = 30$ m.
Each drone runs a local **3D EKF-SLAM** filter whose state encodes its 3D position, heading, and
the 3D positions of all landmarks it has encountered. Drones communicate pairwise within $r_{comm} =
50$ m and merge maps via 3D rigid-body alignment. Navigation follows an **altitude-separated
frontier policy**: the $N$ drones are assigned non-overlapping altitude layers and each
independently selects 3D frontier voxels within its layer, enabling simultaneous vertical coverage.

**Roles**:
- **Drones** ($N = 4$): homogeneous quadrotors, each running an independent 3D EKF-SLAM filter;
  state vector dimension grows as new 3D landmarks are initialised; communicate pairwise within
  $r_{comm}$.
- **Landmarks** ($M_{true} = 50$): fixed 3D features; positions and heights unknown at mission
  start; detected by range-bearing-elevation sensor when within $r_{sense}$.

**Objective**: Within $T_{max} = 800$ s, maximise the fraction of true 3D landmarks successfully
mapped (3D position error $< 1.5$ m) while minimising the 3D RMS error of the final merged map.
The 3D swarm result is compared against a 2D-constrained baseline (drones locked to a single
altitude layer, missing all elevated landmarks above their sensing plane) and an isolated 3D-SLAM
baseline (no communication).

---

## Mathematical Model

### 3D Drone State and EKF-SLAM State Vector

Each drone $i$ uses a reduced 4-DOF navigation state (position + heading) since roll/pitch are
regulated to near zero by the flight controller:

$$\mathbf{x}_i = \begin{bmatrix} x_i \\ y_i \\ z_i \\ \psi_i \\ \mathbf{l}_{i,1} \\ \vdots \\ \mathbf{l}_{i,n_i} \end{bmatrix} \in \mathbb{R}^{4 + 3 n_i}$$

where $\mathbf{l}_{i,k} = (l_x^{(k)}, l_y^{(k)}, l_z^{(k)})^\top$ is the estimated 3D global
position of the $k$-th landmark. The covariance matrix is
$\mathbf{P}_i \in \mathbb{R}^{(4 + 3 n_i) \times (4 + 3 n_i)}$.

### 3D Motion Model (Predict Step)

With forward speed $v$, vertical rate $\dot{z}$, and turn rate $\omega$ all commanded:

$$f(\mathbf{x}_i) = \begin{bmatrix}
x_i + v \Delta t \cos\psi_i \\
y_i + v \Delta t \sin\psi_i \\
z_i + \dot{z}_i \Delta t \\
\psi_i + \omega_i \Delta t \\
\mathbf{l}_{i,1} \\ \vdots \\ \mathbf{l}_{i,n_i}
\end{bmatrix}$$

The Jacobian $\mathbf{F}_i \in \mathbb{R}^{(4+3n_i) \times (4+3n_i)}$ has the pose block:

$$\mathbf{F}_{pose} = \begin{bmatrix}
1 & 0 & 0 & -v \Delta t \sin\psi_i \\
0 & 1 & 0 &  v \Delta t \cos\psi_i \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}$$

with landmark rows/columns forming an identity block as before. The process noise covariance is:

$$\mathbf{Q} = \mathrm{blkdiag}\!\left(
\mathrm{diag}\!\left(\sigma_{ax}^2 \Delta t^2,\; \sigma_{ay}^2 \Delta t^2,\; \sigma_{az}^2 \Delta t^2,\; \sigma_\omega^2 \Delta t^2\right),\;
\mathbf{0}_{3n_i}\right)$$

### 3D Observation Model (Range-Bearing-Elevation)

For landmark $k$ at 3D position $\mathbf{l}_{i,k} = (l_x, l_y, l_z)^\top$, observed from drone
pose $(x_i, y_i, z_i, \psi_i)$:

$$\boldsymbol{\delta} = \begin{bmatrix} \delta_x \\ \delta_y \\ \delta_z \end{bmatrix}
= \begin{bmatrix} l_x - x_i \\ l_y - y_i \\ l_z - z_i \end{bmatrix}$$

$$\rho_k = \|\boldsymbol{\delta}\|, \qquad
\phi_k = \mathrm{atan2}(\delta_y, \delta_x) - \psi_i, \qquad
\epsilon_k = \arcsin\!\left(\frac{\delta_z}{\rho_k}\right)$$

The predicted observation and its Jacobian $\mathbf{H}_{i,k} \in \mathbb{R}^{3 \times (4+3n_i)}$:

$$h(\mathbf{x}_i, k) = \begin{bmatrix} \rho_k \\ \phi_k \\ \epsilon_k \end{bmatrix}$$

Partial derivatives of $\rho$ with respect to drone pose position and landmark position are the
standard 3D range gradients. Partial derivatives of $\epsilon$ require the chain rule on the
$\arcsin$ term; letting $\rho_{xy} = \sqrt{\delta_x^2 + \delta_y^2}$:

$$\frac{\partial \epsilon}{\partial \delta_z} = \frac{1}{\rho_{xy}}, \qquad
\frac{\partial \epsilon}{\partial \delta_x} = -\frac{\delta_x \delta_z}{\rho_{xy} \rho_k^2}, \qquad
\frac{\partial \epsilon}{\partial \delta_y} = -\frac{\delta_y \delta_z}{\rho_{xy} \rho_k^2}$$

The measurement noise covariance is
$\mathbf{R} = \mathrm{diag}(\sigma_\rho^2, \sigma_\phi^2, \sigma_\epsilon^2)$.

### 3D EKF Update

Innovation vector (with angle-wrapping on $\phi$ and $\epsilon$ components):

$$\boldsymbol{\nu} = \mathbf{z}_{i,k} - h(\mathbf{x}_{i,k|k-1}, k), \qquad
\boldsymbol{\nu}_{[\phi,\epsilon]} \leftarrow \mathrm{atan2}(\sin(\cdot), \cos(\cdot))$$

The Kalman gain and posterior update follow the standard EKF equations with the 3D Jacobian
$\mathbf{H}_{i,k}$ (3 rows, one per measurement component).

### 3D Data Association: Mahalanobis Gate

The gate is now $\chi^2$ with 3 degrees of freedom ($\rho$, $\phi$, $\epsilon$) at 95%:

$$d_M(\mathbf{z}, j) = \boldsymbol{\nu}_j^\top \mathbf{S}_j^{-1} \boldsymbol{\nu}_j \leq \chi^2_{3, 0.95} = 7.815$$

### 3D Landmark Initialisation

From a spherical observation $(\rho^{obs}, \phi^{obs}, \epsilon^{obs})$ and drone pose
$(x_i, y_i, z_i, \psi_i)$, the global bearing angle is $\alpha = \psi_i + \phi^{obs}$:

$$\mathbf{l}_{new} = \begin{bmatrix}
x_i + \rho^{obs} \cos\epsilon^{obs} \cos\alpha \\
y_i + \rho^{obs} \cos\epsilon^{obs} \sin\alpha \\
z_i + \rho^{obs} \sin\epsilon^{obs}
\end{bmatrix}$$

The initial landmark covariance $\mathbf{P}_{ll}^{new} \in \mathbb{R}^{3 \times 3}$ is propagated
via the Jacobian $\mathbf{J}_{inv} \in \mathbb{R}^{3 \times 7}$ of the initialisation map with
respect to $(x_i, y_i, z_i, \psi_i, \rho^{obs}, \phi^{obs}, \epsilon^{obs})$:

$$\mathbf{P}_{ll}^{new} = \mathbf{J}_{inv} \, \mathrm{blkdiag}(\mathbf{P}_{pose}, \mathbf{R}) \, \mathbf{J}_{inv}^\top$$

### 3D Voxel Occupancy Grid

The environment is discretised into $N_x \times N_y \times N_z$ voxels of side $d_v = 2.0$ m:

$$N_x = N_y = \lceil 200 / d_v \rceil = 100, \qquad N_z = \lceil 40 / d_v \rceil = 20$$

Total voxels: $100 \times 100 \times 20 = 200{,}000$. Each voxel $\mathbf{c}$ stores a log-odds
belief $L(\mathbf{c}) \in \mathbb{R}$, updated when a landmark is observed within the voxel's
neighbourhood. A voxel is **explored** when $|L(\mathbf{c})| > L_{thresh}$. A **frontier voxel**
is an explored voxel adjacent to at least one unexplored voxel in any of the 26-connected
neighbours.

### Altitude-Separated Frontier Policy

Drone $i$ is assigned a preferred altitude band $[z_i^{lo}, z_i^{hi}]$, with the $N = 4$ bands
partitioning $[0, 40]$ m into layers of 10 m each. Frontier candidates outside the assigned band
are penalised by a factor $\gamma_{out} = 0.2$ in the gain-to-distance ratio:

$$\mathbf{w}_i^* = \arg\max_{\mathbf{w} \in \mathcal{W}_{frontier}}
  \frac{\mathrm{IG}_3(\mathbf{w}) \cdot \gamma_i(\mathbf{w})}{\|\mathbf{w} - \mathbf{p}_i\|}$$

where $\mathrm{IG}_3(\mathbf{w})$ counts unexplored voxels within a sphere of radius $r_{sense}$
around candidate $\mathbf{w}$, and $\gamma_i(\mathbf{w}) = 1$ if $\mathbf{w}$ is within drone
$i$'s band, $\gamma_{out}$ otherwise. Altitude boundary sharing is permitted when a layer is
fully explored.

### 3D Map Merging via SE(3) Alignment

When drones $i$ and $j$ share $m \geq 4$ non-coplanar common landmarks, the full SE(3) rigid
transform is recovered. Let $\mathcal{C}$ be the set of matched 3D landmark pairs
$\{(\mathbf{l}_i^{(a_k)}, \mathbf{l}_j^{(b_k)})\}_{k=1}^{m}$. Compute centroids and the
cross-covariance matrix:

$$\bar{\mathbf{l}}_i = \frac{1}{m} \sum_k \mathbf{l}_i^{(a_k)}, \quad
\bar{\mathbf{l}}_j = \frac{1}{m} \sum_k \mathbf{l}_j^{(b_k)}$$

$$\mathbf{W} = \sum_{k=1}^{m}
  \bigl(\mathbf{l}_i^{(a_k)} - \bar{\mathbf{l}}_i\bigr)
  \bigl(\mathbf{l}_j^{(b_k)} - \bar{\mathbf{l}}_j\bigr)^\top \in \mathbb{R}^{3 \times 3}$$

$$\mathbf{W} = \mathbf{U} \boldsymbol{\Sigma} \mathbf{V}^\top, \qquad
\mathbf{R}^* = \mathbf{U} \, \mathrm{diag}(1, 1, \det(\mathbf{U}\mathbf{V}^\top)) \, \mathbf{V}^\top$$

$$\mathbf{t}^* = \bar{\mathbf{l}}_i - \mathbf{R}^* \bar{\mathbf{l}}_j$$

The determinant fix $\det(\mathbf{U}\mathbf{V}^\top)$ prevents reflection solutions. Non-common
visitor landmarks are transformed by $(\mathbf{R}^*, \mathbf{t}^*)$ and fused into the host map
via the same covariance-intersection update as the 2D version, now with $3 \times 3$ covariance
blocks.

### 3D Loop Closure: SE(3) Pose Graph Back-End

When drone $i$ revisits a volume (detected by finding $\geq 5$ previously mapped landmarks within
$r_{sense}$), a loop closure constraint is created. The pose graph accumulates nodes
$\{\mathbf{T}_k \in SE(3)\}$ and edges $(\mathbf{T}_{rel}, \boldsymbol{\Omega})$ where
$\boldsymbol{\Omega}$ is the information matrix of the relative transform. The back-end
minimises the sum of squared SE(3) errors using Gauss-Newton iterations:

$$\min_{\{\mathbf{T}_k\}} \sum_{(i,j) \in \mathcal{E}}
  \boldsymbol{e}_{ij}^\top \boldsymbol{\Omega}_{ij} \boldsymbol{e}_{ij}$$

where $\boldsymbol{e}_{ij} = \log\!\left(\mathbf{T}_{ij,meas}^{-1} \mathbf{T}_i^{-1} \mathbf{T}_j\right)$
is the SE(3) error in the Lie algebra $\mathfrak{se}(3)$ (6-vector).

### 3D Map Quality Metrics

$$\text{RMS}_{3D} = \sqrt{\frac{1}{M_{found}} \sum_{k=1}^{M_{found}} \|\hat{\mathbf{l}}_k - \mathbf{l}^*_k\|^2}$$

$$\text{Coverage} = \frac{M_{found}}{M_{true}} \times 100\%, \quad
M_{found} = \#\left\{k : \min_j \|\hat{\mathbf{l}}_j - \mathbf{l}^*_k\| < 1.5 \text{ m}\right\}$$

$$\text{Vertical RMS} = \sqrt{\frac{1}{M_{found}} \sum_{k=1}^{M_{found}} (\hat{l}_{z,k} - l^*_{z,k})^2}$$

The vertical RMS is reported separately because altitude uncertainty is typically larger than
horizontal uncertainty when sensor elevation angle noise $\sigma_\epsilon$ dominates.

---

## Key 3D Additions

- **3D landmark state**: each feature stored as $(l_x, l_y, l_z)$; covariance blocks are $3 \times 3$.
- **Range-bearing-elevation sensor**: observation vector $(\rho, \phi, \epsilon)$; 3D Jacobian
  $\mathbf{H}_{i,k} \in \mathbb{R}^{3 \times (4 + 3n_i)}$; gate uses $\chi^2_{3, 0.95} = 7.815$.
- **3D initialisation Jacobian**: $\mathbf{J}_{inv} \in \mathbb{R}^{3 \times 7}$ propagates sensor
  and pose uncertainty into the initial 3D landmark covariance.
- **3D voxel occupancy grid**: $100 \times 100 \times 20$ voxels, 26-connected frontier detection.
- **Altitude-separated frontiers**: drones assigned to 10 m altitude bands; out-of-band frontier
  penalty $\gamma_{out} = 0.2$.
- **SE(3) map merging**: requires $\geq 4$ non-coplanar common landmarks; uses SVD with determinant
  reflection fix for correct $SO(3)$ alignment.
- **3D pose-graph loop closure**: SE(3) back-end with Gauss-Newton optimisation on $\mathfrak{se}(3)$.
- **Altitude bounds enforcement**: $z \in [0.5, 35.0]$ m.
- **Vertical accuracy metric**: separate reporting of $z$-axis RMS in addition to 3D RMS.

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Environment volume | 200 × 200 × 40 m |
| Number of drones $N$ | 4 |
| Altitude bands (per drone) | 0–10, 10–20, 20–30, 30–40 m |
| True 3D landmarks $M_{true}$ | 50 |
| Landmark $z$ range | 0 – 30 m |
| Drone cruise speed $v$ | 3.0 m/s |
| Vertical rate $\dot{z}_{max}$ | 1.5 m/s |
| Simulation timestep $\Delta t$ | 0.2 s |
| Mission horizon $T_{max}$ | 800 s |
| 3D sensor range $r_{sense}$ | 30.0 m |
| Communication range $r_{comm}$ | 50.0 m |
| Range noise std $\sigma_\rho$ | 0.3 m |
| Bearing noise std $\sigma_\phi$ | 0.03 rad |
| Elevation noise std $\sigma_\epsilon$ | 0.04 rad |
| Vertical process noise $\sigma_{az}$ | 0.05 m/s² |
| Data association gate $\chi^2_{3, 0.95}$ | 7.815 |
| Voxel side length $d_v$ | 2.0 m |
| Voxel grid dimensions | 100 × 100 × 20 |
| SE(3) merging minimum common landmarks | 4 (non-coplanar) |
| Common landmark proximity threshold | 2.5 m |
| Landmark localisation success radius | 1.5 m |
| Out-of-band frontier penalty $\gamma_{out}$ | 0.2 |
| Altitude bounds | 0.5 – 35.0 m |
| Initial pose covariance $\mathbf{P}_0$ | $0.01 \cdot \mathbf{I}_4$ |

---

## Expected Output

- **3D ground truth map**: scatter plot of 50 true 3D landmark positions colour-coded by height;
  drone starting positions and altitude band boundaries shown as horizontal planes.
- **3D trajectory and map panel**: one 3D axes per drone showing flight paths coloured by time,
  estimated 3D landmark positions as crosses, true positions as filled circles, matching lines
  between pairs; 3D uncertainty ellipsoids drawn for well-localised landmarks.
- **Altitude profile vs time**: $z(t)$ for each of the 4 drones; shows band-separated exploration
  and any cross-band incursions during communication-driven reassignment.
- **3D voxel occupancy animation (GIF)**: volumetric slice views at three fixed altitudes (5, 15,
  25 m) updated at 5 fps; unexplored voxels in grey, explored in white, drone positions as coloured
  spheres, communication links as lines.
- **Coverage vs time**: $M_{found}(t)/M_{true}$ for 3D swarm, isolated 3D, and 2D-constrained
  baseline; the 2D baseline plateaus early as it cannot detect elevated landmarks.
- **3D RMS and vertical RMS vs time**: two curves per strategy; vertical RMS expected to converge
  more slowly than horizontal RMS due to elevation angle noise amplification at long range.
- **SE(3) merge event log**: timeline of pairwise merge events; each event annotated with number of
  common landmarks found and alignment residual.
- **Loop closure correction plot**: before/after overlay of drone trajectory and landmark map at
  each loop closure event; shows magnitude of 3D pose correction applied.
- **Final vertical accuracy heatmap**: 2D top-down map of per-landmark $|l_z^{est} - l_z^{true}|$
  colour-coded from blue (accurate) to red (high vertical error); reveals which altitude regions
  benefited least from sensor coverage.

---

## Extensions

1. **Graph-SLAM back-end with iSAM2**: replace the sequential EKF with an incremental smoothing
   solver (iSAM2) that jointly optimises all drone poses and landmark positions; compare
   trajectory RMSE and map quality against the EKF approach.
2. **Non-coplanar landmark degeneracy handling**: when fewer than 4 non-coplanar common landmarks
   are found (e.g., all landmarks lie on a floor plane), fall back to 2D alignment constrained to
   $z$-axis rotation only; design an automatic degeneracy detector.
3. **Adaptive altitude reassignment**: if one altitude band is fully explored while another remains
   sparse, dynamically reassign a drone across the boundary; design a decentralised band-auction
   protocol so reassignment does not require a central coordinator.
4. **Elevation-angle-only initialisation**: for landmarks detected near the sensor's elevation
   angle limit ($|\epsilon| > 60°$), the depth estimate is poorly conditioned; implement an
   iterative triangulation initialiser using observations from multiple drone positions before
   committing the landmark to the map.
5. **Multi-resolution voxel grid (Octomap)**: replace the fixed-resolution voxel grid with an
   octree that allocates finer voxels near landmark-rich regions and coarser voxels in open space;
   measure memory savings and frontier detection speedup.

---

## Related Scenarios

- Original 2D version: [S050 Swarm SLAM](../S050_slam.md)
- 3D sensor geometry reference: [S001](../../01_pursuit_evasion/S001_basic_intercept.md), [S003](../../01_pursuit_evasion/S003_low_altitude_tracking.md)
- Probabilistic mapping foundation: [S042 Missing Person Localisation](../S042_missing_person.md)
- Communication topology under 3D relay constraints: [S047 Signal Relay](../S047_signal_relay.md)
- Loop closure graph-SLAM: [S013 Particle Filter Intercept](../../01_pursuit_evasion/S013_particle_filter_intercept.md)
