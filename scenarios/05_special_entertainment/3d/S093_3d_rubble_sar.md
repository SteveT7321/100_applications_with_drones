# S093 3D Upgrade — Rubble Search and Rescue

**Domain**: Special Ops & Entertainment | **Difficulty**: ⭐⭐⭐ | **Status**: `[ ]` Not Started
**Based on**: [S093 original](../S093_rubble_sar.md)

---

## What Changes in 3D

The original S093 is already framed in a $20 \times 20 \times 5$ m volume, but the LiDAR sensor
casts only horizontal rays (elevation fixed, azimuth swept 360°) and the occupancy update never
fully resolves overhanging debris. Voids above and below the scan plane remain classified as
unknown, and the drone holds a fixed altitude layer while transiting passages. This upgrade adds
three structural changes:

1. **Voxel map with overhangs**: the rubble generator explicitly places horizontal slab obstacles
   (overhang layers at $z = 1.5$ m and $z = 3.0$ m) that create enclosed tunnels only accessible
   by vertical descent or ascent. These features cannot be navigated without z-axis planning.

2. **Micro-drone tunnel navigation**: the path planner and motion controller are extended to
   command altitude as a continuous degree of freedom. The drone can dive under a slab, traverse a
   tunnel, and resurface — all driven by the 3D A* solution over the inflated voxel graph.

3. **3D victim probability map update**: instead of a single cumulative scalar count per victim,
   the acoustic detection model now maintains a 3D Gaussian probability density over the victim's
   unknown position. Each positive detection triggers a Bayesian update that narrows the spatial
   estimate; a directed search leg steers toward the estimated victim centroid when the distribution
   is sufficiently peaked.

---

## Problem Definition

**Setup**: A post-earthquake rubble field measuring $20 \times 20 \times 5$ m is discretised into
voxels of side $\Delta_{vox} = 0.1$ m ($200 \times 200 \times 50$ cells). The fill rate is 40 %,
augmented by two horizontal overhang slabs (each two voxels thick) at $z = 1.5$ m and $z = 3.0$ m
spanning random 30–60 % of the XY extent. Slabs generate enclosed tunnel cavities; the
connectivity guarantee ensures at least one navigable 3D path from the entrance to every victim.

Five buried victims are placed in distinct z-bands: one in each of the bands
$z \in [0.2, 1.4]$ m (ground floor), $z \in [1.6, 2.9]$ m (mid level), and three deeper.
Victims emit an omni-directional acoustic beacon. The drone carries a **full-3D LiDAR**
($N_{beams} = 64$ beams distributed over $\pm 30°$ elevation at 8 elevation rings, 8 azimuths
each) plus the same acoustic sensor as S093 ($\sigma_a = 0.8$ m, $r_{acoustic} = 2$ m).

**Roles**:

- **Drone** (1): micro-UAV, body radius 0.15 m, maximum speed $v_{max} = 0.8$ m/s; fully 3D
  path tracking with altitude commanded by A*; maintains a per-victim 3D probability map.
- **Victims** (5): stationary at unknown 3D positions; each emitting an acoustic beacon detectable
  within $r_{acoustic} = 2$ m nominal range.
- **Rubble obstacles**: static voxel grid with overhang slabs enforcing tunnel passages; minimum
  vertical clearance 0.4 m inside tunnels.

**Objective**: Locate all 5 victims within $T_{max} = 400$ s. Report:

1. **Victims found** $N_{found}$ — count of victims with posterior peak $P_{peak,i} \geq 0.85$.
2. **Volume coverage** $\xi = V_{explored} / V_{total\_free} \times 100\%$.
3. **Mission time** $T_{mission}$ (s).
4. **Localisation RMSE** — root mean squared error between estimated and true victim positions at
   mission end.

---

## Mathematical Model

### 3D Voxel Occupancy Grid (unchanged from S093)

The log-odds update rule, obstacle inflation at $r_{inf} = 0.25$ m, and voxel classification
thresholds ($P > 0.7$ occupied, $P < 0.3$ free) are inherited directly from S093. The key change
is that rays are cast in 3D directions, not only horizontally.

### Full-3D LiDAR Ray Model

The $N_{beams} = 64$ beams are distributed over $N_{el} = 8$ elevation rings at angles

$$\phi_k = -30° + k \cdot \frac{60°}{N_{el} - 1}, \quad k = 0, \ldots, N_{el}-1$$

with $N_{az} = 8$ azimuth beams per ring:

$$\theta_j = j \cdot \frac{360°}{N_{az}}, \quad j = 0, \ldots, N_{az}-1$$

Each beam direction unit vector:

$$\hat{\mathbf{d}}_{jk} = \begin{bmatrix} \cos\phi_k \cos\theta_j \\ \cos\phi_k \sin\theta_j \\ \sin\phi_k \end{bmatrix}$$

The Bresenham-3D ray marcher traverses from the drone position along $\hat{\mathbf{d}}_{jk}$ up to
$r_{lidar} = 3.0$ m, applying the identical log-odds free/occupied update from S093 at each voxel
along the ray. This populates the occupancy grid in the full $200 \times 200 \times 50$ volume
rather than a single scan plane.

### Overhang Slab Generation

Two horizontal slabs are added to the baseline rubble grid. Slab $s$ occupies:

$$\mathcal{V}_{slab,s} = \{(i,j,k) : k \in [k_{s}^{lo}, k_{s}^{hi}],\; i \in [i_s^{min}, i_s^{max}],\; j \in [0, N_Y)\}$$

with $k_{1}^{lo} = 14, k_{1}^{hi} = 16$ (slab at $z = 1.5$ m) and $k_{2}^{lo} = 29, k_{2}^{hi} = 31$
(slab at $z = 3.0$ m). Passage gaps of width $\geq 4$ voxels (0.4 m) are cut at random XY
positions before the connectivity check, guaranteeing navigable tunnels beneath each slab.

### 3D Altitude-Aware Path Planning

The 26-connected A* cost function is extended with a vertical penalty term to discourage
unnecessary altitude changes and to model increased energy cost of climbing:

$$c(\mathbf{n}, \mathbf{n}') = \|\mathbf{x}_{\mathbf{n}} - \mathbf{x}_{\mathbf{n}'}\|_2 + \lambda_z \cdot \max(0,\; z_{\mathbf{n}'} - z_{\mathbf{n}})$$

where $\lambda_z = 0.3$ penalises upward steps relative to horizontal moves. The admissibility of
the heuristic is preserved because $\lambda_z < 1$ and the 3D Euclidean heuristic lower-bounds the
true cost.

The motion controller tracks the A* waypoint sequence in full 3D:

$$\mathbf{v}_{cmd}(t) = v_{max} \cdot \frac{\mathbf{w}_k - \mathbf{p}(t)}{\|\mathbf{w}_k - \mathbf{p}(t)\|}$$

where $\mathbf{w}_k$ is the current target waypoint. Waypoint advance occurs when
$\|\mathbf{p}(t) - \mathbf{w}_k\| < \Delta_{vox} \cdot 0.6$, just as in S093.

Altitude bounds are enforced: $z(t) \in [0.2, 4.8]$ m.

### 3D Victim Probability Map

Each victim $i$ has an unknown true position $\mathbf{v}_i^* \in \mathbb{R}^3$. The drone
maintains a belief distribution modelled as a 3D Gaussian:

$$b_i(\mathbf{x}) = \mathcal{N}\!\left(\mathbf{x};\; \boldsymbol{\mu}_i,\; \sigma_i^2 \mathbf{I}\right)$$

initialised with $\boldsymbol{\mu}_i^{(0)} = \mathbf{0}$ (no prior) and $\sigma_i^{(0)} = 5$ m
(flat prior). At each timestep $t$, the drone at position $\mathbf{p}(t)$ receives acoustic
observation $z_i(t) \in \{0, 1\}$ drawn from:

$$P(z_i = 1 \mid \mathbf{x}) = \exp\!\left(-\frac{\|\mathbf{p}(t) - \mathbf{x}\|^2}{2\,\sigma_a^2}\right)$$

The Bayesian update on the unnormalised log-belief:

$$\log b_i^{(t+1)}(\mathbf{x}) \propto \log b_i^{(t)}(\mathbf{x}) + z_i(t) \log P(z_i=1|\mathbf{x}) + (1 - z_i(t)) \log P(z_i=0|\mathbf{x})$$

In the Gaussian approximation this becomes a mean-shift update:

$$\boldsymbol{\mu}_i \leftarrow \boldsymbol{\mu}_i + \alpha_i \cdot z_i(t) \cdot (\mathbf{p}(t) - \boldsymbol{\mu}_i)$$

$$\sigma_i^2 \leftarrow \sigma_i^2 \cdot (1 - \alpha_i \cdot z_i(t))$$

with learning rate $\alpha_i = P_{detect,i}(\mathbf{p}(t)) / (1 + P_{detect,i}(\mathbf{p}(t)))$.

Victim $i$ is declared **found** when the peak posterior probability:

$$P_{peak,i} = b_i(\boldsymbol{\mu}_i) = \frac{1}{(2\pi)^{3/2} \sigma_i^3} \geq \theta_{peak} = 0.85 / (2\pi)^{3/2}$$

equivalently $\sigma_i \leq \sigma_{conf} = 0.5$ m.

### Directed Search Toward Posterior Peak

Once $\sigma_i < \sigma_{switch} = 1.5$ m for any unconfirmed victim $i$, the frontier selector is
overridden and the drone is routed via A* toward the posterior mean $\boldsymbol{\mu}_i$, treating
it as a high-priority waypoint:

$$c^*_{directed} = \arg\min_{i:\, \sigma_i < \sigma_{switch},\, \neg found_i} \|\boldsymbol{\mu}_i - \mathbf{p}\|$$

### Localisation RMSE

At mission end, the position estimate error for each found victim:

$$\text{RMSE} = \sqrt{\frac{1}{N_{found}} \sum_{i:\, found_i} \|\boldsymbol{\mu}_i - \mathbf{v}_i^*\|^2}$$

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Rubble field dimensions | $20 \times 20 \times 5$ m |
| Voxel resolution $\Delta_{vox}$ | 0.1 m |
| Obstacle fill rate | 40 % |
| Overhang slab altitudes | 1.5 m, 3.0 m |
| Slab thickness | 0.2 m (2 voxels) |
| Minimum tunnel clearance | 0.4 m |
| Number of victims | 5 |
| Drone body radius | 0.15 m |
| Obstacle inflation radius $r_{inf}$ | 0.25 m |
| LiDAR elevation rings $N_{el}$ | 8 (at $-30°$ to $+30°$) |
| LiDAR azimuths per ring $N_{az}$ | 8 |
| Total LiDAR beams $N_{beams}$ | 64 |
| LiDAR max range $r_{lidar}$ | 3.0 m |
| Acoustic nominal range $r_{acoustic}$ | 2.0 m |
| Acoustic detection std $\sigma_a$ | 0.8 m |
| Prior belief std $\sigma_i^{(0)}$ | 5.0 m |
| Confirmation std threshold $\sigma_{conf}$ | 0.5 m |
| Directed search trigger $\sigma_{switch}$ | 1.5 m |
| Vertical path penalty $\lambda_z$ | 0.3 |
| Altitude bounds | 0.2 – 4.8 m |
| Drone max speed $v_{max}$ | 0.8 m/s |
| Simulation timestep $\Delta t$ | 0.1 s |
| Mission timeout $T_{max}$ | 400 s |

---

## Expected Output

- **3D rubble field + trajectory** with overhang slabs rendered as semi-transparent grey planes;
  drone path colour-coded by altitude (blue = low, red = high); victim markers as stars with
  posterior ellipsoid (1-sigma sphere) drawn around each estimated position.
- **XZ side-elevation view**: reveals the drone diving beneath the 1.5 m slab and re-emerging;
  vertical excursion profile clearly distinguishes tunnel traversal segments from surface exploration.
- **Altitude time series**: $z(t)$ showing descent/ascent events correlated with tunnel crossings;
  annotated with the two slab altitudes.
- **Per-victim posterior std $\sigma_i(t)$**: five curves converging from 5 m toward $\sigma_{conf} = 0.5$ m;
  vertical marker when each victim is confirmed; shows directed-search phase as the steepest
  descent segment.
- **3D probability map slice** at the z-level of each confirmed victim: heatmap of $b_i(\mathbf{x})$
  over the XY plane; posterior peak aligned with true victim position.
- **Localisation RMSE bar chart**: one bar per victim showing $\|\boldsymbol{\mu}_i - \mathbf{v}_i^*\|$
  at confirmation time.
- **Mission metrics summary**: victims found, volume coverage $\xi$, mission time, and mean RMSE.
- **Exploration animation** (`s093_3d_exploration.gif`): rotating 3D view of drone path growing
  frame-by-frame; slab obstacles shown as flat planes; victim ellipsoids updated each frame.

**Typical target values**:

| Metric | Target |
|--------|--------|
| Victims found | 5 / 5 |
| Volume coverage $\xi$ | 70 – 85 % |
| Mission time $T_{mission}$ | 250 – 380 s |
| Localisation RMSE | $\leq 0.4$ m |

---

## Extensions

1. **Multi-layer slab with collapse events**: model secondary slab failure at random times
   ($\lambda = 0.003$ events/s); the drone must re-plan around newly blocked tunnel exits;
   assess how collapse frequency degrades victim-find rate and RMSE.
2. **Heterogeneous sensor suite**: add a CO$_2$ sensor with a Gaussian plume model in addition to
   the acoustic beacon; fuse acoustic and chemical evidence in a joint 3D belief map using a
   product-of-experts formulation.
3. **Two cooperating micro-drones**: assign one drone to deep tunnel exploration (low altitude) and
   one to surface-level exploration (high altitude); merge occupancy grids and belief maps via radio
   when within $r_{comm} = 5$ m; compare single-drone vs cooperative RMSE and coverage.
4. **Optimal slab gap discovery**: replace random gap placement with an adversarial generator that
   maximises expected mission time; use a simple evolutionary search over gap positions to study
   worst-case rubble configurations.
5. **Energy budget with altitude cost**: model battery as $E_{max} = 600$ J; assign energy cost
   $\Delta E = m g \Delta z + \frac{1}{2} m v^2$ per step; add return-to-base constraint; study
   the trade-off between deep tunnel exploration and energy reserve for return flight.

---

## Related Scenarios

- Original 2D/flat version: [S093](../S093_rubble_sar.md)
- Multi-agent follow-up: [S094 Counter-Drone Defense](../S094_counter_drone.md)
- 3D occupancy mapping reference: [S074 Mine 3D Mapping](../../04_industrial_agriculture/S074_mine_mapping.md)
- Probabilistic detection reference: [S013 Particle Filter Intercept](../../01_pursuit_evasion/S013_particle_filter_intercept.md)
- Cooperative SLAM reference: [S050 Swarm Cooperative Mapping](../../03_environmental_sar/S050_slam.md)
