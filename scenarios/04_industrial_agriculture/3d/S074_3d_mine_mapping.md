# S074 3D Upgrade — Mine 3D Mapping

**Domain**: Industrial & Agriculture | **Difficulty**: ⭐⭐⭐⭐⭐ | **Status**: `[ ]` Not Started
**Based on**: [S074 original](../S074_mine_mapping.md)

---

## What Changes in 3D

The original S074 constrains the drone to a fixed horizontal flight altitude (`z = 1.5 m` hardcoded in every branch definition, scan rays emitted only in the horizontal plane, and the occupancy grid is updated exclusively from 2D LiDAR sweeps). Three structural limitations follow from this:

1. **Vertical tunnel geometry is invisible.** Shafts, ramps, and inclined drifts that change elevation cannot be represented — any tunnel segment with a non-zero z-gradient is treated as unreachable.
2. **The LiDAR sensor model is planar.** A single-layer 2D scan at `z = 1.5 m` misses wall anomalies above and below the scan plane, and cannot detect the floor-to-ceiling extent of cross-passages.
3. **Dust attenuation is ignored.** Underground mines accumulate coal or rock dust that degrades LiDAR beam intensity non-uniformly with distance, reducing effective range in high-dust zones.

This 3D upgrade introduces:

- A **multi-level tunnel network** with three horizontal galleries at different elevations connected by two vertical shafts, spanning z from 0 m to 12 m.
- A **3D multi-layer LiDAR model** (16 vertical beams, elevation $-15°$ to $+15°$) that observes both lateral walls and the floor/ceiling geometry.
- **Dust-impaired range attenuation**: effective maximum range degrades as $r_{eff}(z) = r_{lidar} \cdot \exp(-\mu_{dust} \cdot c_{dust}(z) \cdot r)$, where $c_{dust}(z)$ is the dust concentration profile, highest near active faces and shaft bottoms.
- **3D frontier exploration** that operates on the full $(x, y, z)$ occupancy grid, selecting frontier centroids with altitude components and commanding the drone through vertical shaft traversals.
- **Shaft-aware path planning**: the drone detects vertical frontier clusters and switches from lateral cruising to vertical climb/descent when entering a shaft bounding box.

---

## Problem Definition

**Setup**: A single drone is deployed at the entrance of a three-level underground mine network.
Level 1 (main haulage) runs at $z = 1.5$ m; Level 2 (secondary gallery) runs at $z = 7.0$ m;
Level 3 (upper ventilation drift) runs at $z = 11.0$ m. Two vertical shafts connect Level 1 to
Level 2 (at $x = 30$ m) and Level 2 to Level 3 (at $x = 60$ m). All horizontal tunnels have a
$3 \times 3$ m square cross-section ($w = 1.5$ m half-width). Each shaft has a $2 \times 2$ m
cross-section and an elevation span of $5.5$ m (Level 1 to Level 2) or $4.0$ m (Level 2 to Level 3).

No GPS signal penetrates underground. Localisation is derived solely from onboard sensors: a
**3D multi-layer LiDAR** (16 elevation layers, 360° azimuth, $r_{lidar} = 10$ m nominal) and an
**IMU** (accelerometer + gyroscope, used as dead-reckoning between scan corrections). Dust
concentration follows a depth-dependent profile $c_{dust}(z)$ that attenuates LiDAR returns
at lower levels where ventilation is poorest.

**Roles**:
- **Drone**: single UAV; maximum speed $v_{max} = 1.5$ m/s; ascent/descent speed capped at
  $v_z^{max} = 0.8$ m/s; carries 3D LiDAR and IMU; runs 3D ICP + occupancy update +
  3D frontier selection on-board; altitude bounds $z \in [0.3, 12.5]$ m.
- **Tunnel network**: static, multi-level; entrance at $(1.5, 0.0, 1.5)$; no dynamic obstacles;
  walls, floors and ceilings treated as planar; dust concentration field known only to the simulator.

**Objective**: Produce a full 3D occupancy map of all three levels and both shafts, and report:

1. **Volumetric coverage** $\xi$ (%) — fraction of reachable voxels correctly classified (occupied or free).
2. **3D pose RMSE** (m) — trajectory error including altitude component.
3. **Exploration time** $T_{exp}$ (s) — mission duration from entrance to no remaining frontiers.
4. **Dust-zone coverage deficit** — fraction of voxels within high-dust zones ($c_{dust} > 0.5$) that remain unknown at mission end, compared to the dust-free baseline.

---

## Mathematical Model

### 3D Tunnel Network Geometry

The network is a directed graph of branches. Each horizontal branch $b$ is defined by
$(\mathbf{s}_b, \mathbf{e}_b, w_b)$ where $w_b = 1.5$ m (half-width). Each vertical shaft
$\nu$ is defined by $(\mathbf{s}_\nu, z_{top,\nu}, r_\nu)$ where $r_\nu = 1.0$ m (half-width).

A world point $\mathbf{p} = (p_x, p_y, p_z)^\top$ is **free** if it lies inside at least one
element:

$$\text{free}(\mathbf{p}) =
  \left[\bigvee_{b} F_b(\mathbf{p})\right] \vee \left[\bigvee_{\nu} S_\nu(\mathbf{p})\right]$$

For a horizontal branch with axis unit vector $\hat{\mathbf{a}}_b = (\mathbf{e}_b - \mathbf{s}_b) / \|\mathbf{e}_b - \mathbf{s}_b\|$:

$$F_b(\mathbf{p}) = \bigl[0 \leq s_b(\mathbf{p}) \leq L_b\bigr]
  \;\wedge\; \bigl\|\mathbf{p}_\perp^{(b)}\bigr\|_\infty \leq w_b$$

where $s_b(\mathbf{p}) = (\mathbf{p} - \mathbf{s}_b) \cdot \hat{\mathbf{a}}_b$ is the
projection onto the branch axis and $\mathbf{p}_\perp^{(b)} = \mathbf{p} - \mathbf{s}_b - s_b(\mathbf{p})\hat{\mathbf{a}}_b$ is the perpendicular offset (both lateral y and vertical z components checked).

For a vertical shaft centred on $(x_\nu, y_\nu)$:

$$S_\nu(\mathbf{p}) = \bigl[z_{bot,\nu} \leq p_z \leq z_{top,\nu}\bigr]
  \;\wedge\; |p_x - x_\nu| \leq r_\nu \;\wedge\; |p_y - y_\nu| \leq r_\nu$$

### 3D Multi-Layer LiDAR Sensor Model

At each timestep the drone emits $N_{az} \times N_{el}$ rays where $N_{az} = 360$ and
$N_{el} = 16$. Azimuth angles $\phi_j = j \cdot 2\pi / N_{az}$ for $j = 0, \ldots, N_{az}-1$.
Elevation angles $\theta_k$ are uniformly spaced over $[-15°, +15°]$:

$$\theta_k = -15° + k \cdot \frac{30°}{N_{el} - 1}, \quad k = 0, \ldots, N_{el}-1$$

Ray direction in the drone body frame:

$$\hat{\mathbf{d}}_{j,k} = \begin{pmatrix}
  \cos\theta_k \cos\phi_j \\
  \cos\theta_k \sin\phi_j \\
  \sin\theta_k
\end{pmatrix}$$

Transformed to the world frame by the full 3D pose $(\mathbf{p}_{est}, \mathbf{R}_{est})$:

$$\hat{\mathbf{d}}_{j,k}^W = \mathbf{R}_{est}\,\hat{\mathbf{d}}_{j,k}$$

### Dust-Impaired Range Attenuation

Dust concentration at a world point $\mathbf{q}$ is modelled as a depth-dependent field:

$$c_{dust}(\mathbf{q}) = c_0 \cdot \exp\!\left(-\frac{p_z - z_{floor}}{H_{dust}}\right)$$

where $c_0 = 0.8$ (peak concentration near floor, typical of coal dust settling),
$z_{floor} = 0.0$ m, and $H_{dust} = 4.0$ m (scale height). The effective maximum range for
beam $(j,k)$ passing through dust is found by the Beer-Lambert attenuation law. The beam's
energy remaining after travelling a path of incremental length $ds$ through dust:

$$\frac{dI}{ds} = -\mu_{ext} \cdot c_{dust}\bigl(\mathbf{p}_{est} + s\,\hat{\mathbf{d}}_{j,k}^W\bigr) \cdot I$$

where $\mu_{ext} = 0.15$ m$^{-1}$ ppm$^{-1}$ is the mass extinction coefficient. The beam is
treated as lost (no return) when $I / I_0 < I_{threshold} = 0.05$, giving an effective range:

$$r_{eff} \approx r_{lidar} \cdot \exp\!\left(-\mu_{ext} \cdot \bar{c}_{dust} \cdot r_{lidar}\right)$$

where $\bar{c}_{dust}$ is the path-averaged dust concentration. In practice $r_{eff}$ is
evaluated numerically during ray-marching by accumulating the optical depth integral.

Noisy range for beam $(j,k)$ with true range $r_{j,k}^{true}$ clamped to $r_{eff}$:

$$r_{j,k} = \min\!\bigl(r_{j,k}^{true},\, r_{eff}\bigr) + \mathcal{N}(0,\, \sigma_r^2),
  \quad \sigma_r = 0.05 \text{ m}$$

### 3D ICP Scan-Matching

With $N_P = N_{az} \times N_{el}$ beams, the 3D ICP formulation is identical to S074 but the
point clouds $\mathcal{P}, \mathcal{Q} \subset \mathbb{R}^3$ now include beams with non-zero
elevation. The M-step cross-covariance matrix:

$$\mathbf{W} = \sum_{i=1}^{N_P} \bigl(\mathbf{p}_i - \bar{\mathbf{p}}\bigr)
  \bigl(\mathbf{q}_{\sigma(i)} - \bar{\mathbf{q}}\bigr)^\top \in \mathbb{R}^{3 \times 3}$$

The SVD solution $\mathbf{R}^* = \mathbf{U}\mathbf{V}^\top$ now captures full roll-pitch-yaw
rotation rather than yaw only. This reduces per-step drift standard deviation from
$\sigma_{ICP} = 0.02$ m (planar) to $\sigma_{ICP}^{3D} = 0.015$ m because the richer 3D
scan geometry provides stronger geometric constraints, particularly in shaft transitions where
vertical structure dominates.

### IMU Dead-Reckoning Between Scans

Between LiDAR scan epochs (separated by $\Delta t_{scan} = 0.5$ s), the drone propagates its
pose using the IMU at 100 Hz. Let $\mathbf{f}$ (specific force) and $\boldsymbol{\omega}$ (angular
rate) be IMU measurements with biases $\mathbf{b}_a$ and $\mathbf{b}_g$:

$$\dot{\mathbf{p}} = \mathbf{v}, \quad
  \dot{\mathbf{v}} = \mathbf{R}(\mathbf{f} - \mathbf{b}_a) - \mathbf{g}, \quad
  \dot{\mathbf{R}} = \mathbf{R}\,[\boldsymbol{\omega} - \mathbf{b}_g]_\times$$

Bias random walk: $\dot{\mathbf{b}}_a \sim \mathcal{N}(0, \sigma_{ba}^2 \mathbf{I})$,
$\dot{\mathbf{b}}_g \sim \mathcal{N}(0, \sigma_{bg}^2 \mathbf{I})$, with
$\sigma_{ba} = 0.005$ m/s$^2/\sqrt{\text{Hz}}$ and $\sigma_{bg} = 0.001$ rad/s$/ \sqrt{\text{Hz}}$.

### 3D Occupancy Grid Update

Voxel resolution $\Delta_{vox} = 0.2$ m; log-odds field $\ell(v)$ initialised to 0. For each
beam $(j,k)$ the ray update follows the same Bayesian log-odds scheme as S074 but now the ray
path is fully 3D, updating voxels at all elevations:

$$\ell(v_{hit}) \mathrel{+}= \log\frac{P_{occ}}{1-P_{occ}} = \log 9 \approx 2.20$$

$$\ell(v_{free}) \mathrel{+}= \log\frac{P_{free}}{1-P_{free}} = \log\tfrac{2}{3} \approx -0.41$$

with clamping to $[\ell_{min}, \ell_{max}] = [-10, 10]$.

For dust-attenuated beams that return no hit (range exceeded $r_{eff}$ without wall contact),
only the free update is applied up to $r_{eff}$; voxels beyond $r_{eff}$ remain unknown, creating
a **dust shadow** that the frontier planner must eventually resolve by repositioning.

### 3D Frontier Exploration with Shaft Awareness

Frontier detection follows S074's 26-connectivity criterion but now operates on all three
spatial dimensions. A voxel $v_f$ is a frontier if it is **unknown** and has at least one
**free** neighbour in its 26-neighbourhood.

After clustering frontier voxels by connected components, each cluster centroid
$\mathbf{g}_c = (g_x, g_y, g_z)$ is classified as either a **lateral frontier**
($|\Delta g_z| < 0.3$ m from current altitude) or a **vertical frontier**
($|\Delta g_z| \geq 0.3$ m, inside a shaft bounding box).

Navigation mode selection:

$$\text{mode} = \begin{cases}
  \textit{cruise} & \text{if nearest frontier is lateral} \\
  \textit{shaft-traverse} & \text{if nearest frontier is vertical}
\end{cases}$$

In **cruise** mode the drone navigates in the horizontal plane at constant altitude, identical
to S074. In **shaft-traverse** mode it first positions horizontally over the shaft centre
$(x_\nu, y_\nu)$, then climbs or descends at $v_z^{max} = 0.8$ m/s until it reaches the target
level, then resumes cruise.

Nearest frontier selection remains:

$$\mathbf{g}^* = \arg\min_{c} \|\mathbf{g}_c - \mathbf{p}_{est}\|_2$$

### 3D Pose RMSE

$$\mathrm{RMSE}_{pose}^{3D} = \sqrt{\frac{1}{N} \sum_{k=1}^{N}
  \bigl[(x_k^{est} - x_k^{gt})^2 + (y_k^{est} - y_k^{gt})^2 + (z_k^{est} - z_k^{gt})^2\bigr]}$$

Altitude drift is largest during shaft traversals because vertical LiDAR returns are sparse
(only a few elevation beams point toward the shaft walls rather than the open shaft volume).

### Dust-Zone Coverage Deficit

Define the high-dust reachable voxel set:

$$\mathcal{V}_{dust} = \{v \in \mathcal{V}_{gt} : c_{dust}(\mathbf{x}_v) > 0.5\}$$

The coverage deficit in dust-affected regions:

$$\Delta\xi_{dust} = \frac{|\{v \in \mathcal{V}_{dust} : 0.3 \leq P(v) \leq 0.7\}|}{|\mathcal{V}_{dust}|} \times 100\%$$

A smaller $\Delta\xi_{dust}$ indicates the planner successfully repositioned the drone to
illuminate dust-shadow voxels from multiple angles.

---

## Key 3D Additions

- **Multi-level tunnel geometry**: three horizontal galleries at $z \in \{1.5, 7.0, 11.0\}$ m connected by two vertical shafts; full 3D branch classification including $z$-axis extents.
- **3D multi-layer LiDAR**: 16 elevation layers over $\pm 15°$; ray directions are full 3D unit vectors; scan point clouds cover floor, ceiling, and shaft walls rather than a single horizontal slice.
- **Dust-impaired attenuation model**: Beer-Lambert optical depth integral accumulated per beam; effective range $r_{eff} < r_{lidar}$ in high-dust low-altitude zones; dust shadows leave frontier voxels unobserved until the drone repositions.
- **IMU dead-reckoning**: 100 Hz IMU propagation between 2 Hz LiDAR scan epochs; bias random walk modelled; particularly important during shaft traversals where scan geometry is weak.
- **Shaft-aware frontier planner**: frontier clusters classified as lateral or vertical; mode switching between horizontal cruise and vertical shaft-traverse; drone positions over shaft centre before committing to altitude change.
- **Altitude bounds**: $z \in [0.3, 12.5]$ m; ascent/descent rate limited to $v_z^{max} = 0.8$ m/s.
- **3D visualisation**: four-panel figure — 3D occupancy scatter with trajectory, XZ elevation cross-section, altitude time series, and dust-zone coverage deficit comparison.

---

## Key Parameters

| Parameter | Symbol | Value |
|-----------|--------|-------|
| Gallery altitudes | $z_{L1}, z_{L2}, z_{L3}$ | 1.5, 7.0, 11.0 m |
| Shaft 1 (L1 to L2) base / top | — | z = 1.5 / 7.0 m |
| Shaft 2 (L2 to L3) base / top | — | z = 7.0 / 11.0 m |
| Shaft half-width | $r_\nu$ | 1.0 m |
| Tunnel half-width | $w$ | 1.5 m |
| LiDAR azimuth beams | $N_{az}$ | 360 |
| LiDAR elevation layers | $N_{el}$ | 16 |
| LiDAR elevation range | $[\theta_{min}, \theta_{max}]$ | $[-15°, +15°]$ |
| LiDAR max range | $r_{lidar}$ | 10.0 m |
| Range noise std | $\sigma_r$ | 0.05 m |
| Dust peak concentration | $c_0$ | 0.8 ppm |
| Dust scale height | $H_{dust}$ | 4.0 m |
| Mass extinction coeff | $\mu_{ext}$ | 0.15 m$^{-1}$ ppm$^{-1}$ |
| Detection threshold | $I_{threshold}$ | 0.05 |
| ICP max iterations | $K_{ICP}$ | 20 |
| ICP convergence tol | $\epsilon_{ICP}$ | $10^{-4}$ m |
| 3D ICP per-step drift std | $\sigma_{ICP}^{3D}$ | 0.015 m/step |
| LiDAR scan epoch | $\Delta t_{scan}$ | 0.5 s |
| IMU update rate | — | 100 Hz |
| IMU accel bias noise | $\sigma_{ba}$ | 0.005 m/s$^2$/$\sqrt{\text{Hz}}$ |
| IMU gyro bias noise | $\sigma_{bg}$ | 0.001 rad/s/$\sqrt{\text{Hz}}$ |
| Voxel resolution | $\Delta_{vox}$ | 0.2 m |
| Occupied log-odds increment | $\ell_{occ}$ | $\log 9 \approx 2.20$ |
| Free log-odds increment | $\ell_{free}$ | $\log(2/3) \approx -0.41$ |
| Log-odds clamp range | $[\ell_{min}, \ell_{max}]$ | $[-10, 10]$ |
| Occupied threshold | $P_{occ}$ | 0.7 |
| Free threshold | $P_{free}$ | 0.3 |
| Lateral navigation speed | $v_{nav}$ | 1.0 m/s |
| Maximum horizontal speed | $v_{max}$ | 1.5 m/s |
| Maximum vertical speed | $v_z^{max}$ | 0.8 m/s |
| Altitude bounds | $[z_{min}, z_{max}]$ | 0.3 – 12.5 m |
| Simulation timestep | $\Delta t$ | 0.1 s |

---

## Expected Output

- **3D occupancy map + full trajectory** (`s074_3d_occupancy.png`): 3D scatter of occupied voxels in grey; ground-truth trajectory coloured by altitude (blue = Level 1, green = Level 2, orange = Level 3); ICP-estimated path in dashed red; shaft transitions visible as vertical segments; dust-shadow gaps visible as unclassified voxel regions near the shaft bottoms.
- **XZ elevation cross-section** (lateral panel): occupancy probability projected onto the $(x, z)$ plane; three gallery levels and two shaft channels clearly resolved; dust attenuation visible as reduced classification confidence at low $z$.
- **Altitude time series**: $z_{gt}(t)$ and $z_{est}(t)$ vs time; three plateau levels with sharp transitions during shaft traversals; altitude drift quantified per shaft crossing.
- **Coverage vs time (per level)**: three separate curves for Level 1, Level 2, Level 3, and combined; Level 3 coverage begins only after the drone completes its first shaft-traverse sequence.
- **Dust-zone coverage deficit bar chart**: side-by-side bars for dust-free scenario vs dust-impaired scenario; $\Delta\xi_{dust}$ in % remaining unknown at mission end; illustrates the impact of dust attenuation on map completeness.
- **Exploration animation** (`s074_3d_animation.gif`): rotating 3D view of the trajectory growing over time; drone marker coloured by current mode (blue = cruise, yellow = shaft-traverse); occupied voxels revealed progressively as the drone completes each level.

**Typical metric values** (seed = 42):

| Metric | Value |
|--------|-------|
| Volumetric coverage $\xi$ | ≥ 85 % |
| 3D Pose RMSE | 0.45 – 0.65 m |
| Exploration time $T_{exp}$ | 700 – 950 s |
| Dust-zone coverage deficit $\Delta\xi_{dust}$ | 8 – 15 % |

---

## Extensions

1. **Loop closure across levels**: when the drone re-enters Level 1 after completing the upper levels, compare the current scan against the stored Level 1 map segment; implement a pose-graph edge between the re-entry scan and the stored map to correct accumulated altitude drift.
2. **Variable shaft diameter**: replace the uniform $2 \times 2$ m shaft with a tapered profile (wider at top, narrower at bottom) to simulate ore chutes; assess whether the frontier planner can navigate a shaft whose width falls below the drone's rotor clearance margin.
3. **Multi-drone level-parallel mapping**: deploy one drone per level simultaneously; implement a shared occupancy grid with timestamped voxel updates; measure wall-clock exploration time speedup versus the single-drone baseline.
4. **Active dust mitigation path**: model dust dispersal as a convection-diffusion PDE with a ventilation airflow vector; plan drone paths that align with airflow to maximise LiDAR range in downwind directions; compare coverage completeness against the wind-agnostic baseline.
5. **Thermal camera fusion**: add a simulated thermal IR camera (80 × 60 pixels, FOV 45°) to detect hot surfaces (fires, equipment); fuse thermal evidence with the occupancy grid via a separate occupancy channel for fire-source voxels; demonstrate joint 3D structural + hazard mapping.
6. **Particle filter relocalisation after shaft transition**: after each shaft traversal ICP geometry is weak; use a particle filter seeded by the expected shaft exit position to recover localisation before resuming lateral cruise; compare 3D pose RMSE with and without the particle filter relocalisation step.

---

## Related Scenarios

- Original 2D version: [S074](../S074_mine_mapping.md)
- Multi-level localisation reference: [S013 Particle Filter Intercept](../../../scenarios/01_pursuit_evasion/S013_particle_filter_intercept.md)
- Frontier-based coverage baseline: [S041 Wildfire Boundary Mapping](../../../scenarios/03_environmental_sar/S041_wildfire_boundary.md)
- Collaborative mapping reference: [S050 Swarm Cooperative Mapping (EKF-SLAM)](../../../scenarios/03_environmental_sar/S050_slam.md)
- Follow-up gas detection scenario: [S075 Confined-Space Gas Detection](../S075_gas_detection.md)
