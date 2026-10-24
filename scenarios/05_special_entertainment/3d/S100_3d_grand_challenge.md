# S100 3D Upgrade — Grand Challenge: Multi-Domain Integration

**Domain**: Special Ops & Entertainment | **Difficulty**: ⭐⭐⭐⭐⭐ | **Status**: `[ ]` Not Started
**Based on**: [S100 original](../S100_grand_challenge.md)

---

## What Changes in 3D

The original S100 already declares a $100 \times 100 \times 30$ m arena, but in practice each phase collapses altitude to a single operating band: Phase 1 catches every ball inside S094's $[0.3, 4.0]$ m altitude window with an orientation-free net in still air, Phase 2 couriers fly at a fixed delivery height of $z = 5$ m, Phase 3 scout maintains a constant $z = 8$ m AGL, and Phase 4 lifts all five drones to a single star plane at $z = 50$ m. The z-axis is never used as a tactical or navigational resource.

This 3D upgrade activates the full volumetric arena across all four phases:

- **Phase 1**: the launcher throws from a 10 m mast, the catcher may take the ball anywhere on its descending branch inside a $[0.3, 20]$ m altitude window, the net cup only accepts balls that enter through its mouth (cup axis tied to the thrust direction, as in the [S094 3D upgrade](S094_3d_ball_catching.md)), and an altitude-dependent wind drifts the ball, so the planner must choose the catch altitude, not only the catch point.
- **Phase 2**: delivery waypoints are distributed across three altitude bands; couriers plan 3D TSP routes and must manage climb/descent energy costs separately from horizontal cruise cost.
- **Phase 3**: the SAR belief map is extended to a $50 \times 50 \times 12$ m volumetric search volume, with the scout varying altitude to optimise the 3D sensor footprint cone against a survivor who may be sheltering at elevation (rubble pile, rooftop).
- **Phase 4**: the star formation is assembled via a vertically stratified ascent choreography — each drone follows an altitude-stratified climb profile to avoid mid-air convergence conflicts, then locks into the $z = 50$ m show plane.

Additionally, a **heterogeneous fleet model** is introduced: each drone has distinct mass, maximum thrust, and battery capacity, making altitude-stratified task allocation a constrained optimisation problem rather than a symmetric assignment.

---

## Problem Definition

**Setup**: A team of $N = 5$ heterogeneous drones is deployed into a full $100 \times 100 \times 60$ m volumetric arena (ceiling raised from 30 m to 60 m to accommodate Phase 4 show altitude and climb clearances). The four-phase mission structure, sequencer logic, and 300 s budget are inherited from S100. All five drones maintain 3D situational awareness via a distributed broadcast channel at 10 Hz.

**Fleet specifications** (heterogeneous):

| Drone | Role | Mass (kg) | Max thrust (N) | Battery (Wh) | Max speed (m/s) |
|-------|------|-----------|----------------|--------------|-----------------|
| A | Catcher | 0.8 | 16.0 | 8.0 | 10.0 |
| B | Courier 1 | 1.2 | 20.0 | 14.0 | 7.0 |
| C | Courier 2 | 1.2 | 20.0 | 14.0 | 7.0 |
| D | Scout | 0.9 | 14.0 | 10.0 | 6.0 |
| E | Formation Lead | 0.8 | 16.0 | 8.0 | 8.0 |

**Roles** (3D extensions):
- **Drone A — Catcher**: catches balls thrown from a 10 m launcher mast anywhere in a $[0.3, 20]$ m altitude window, with a tilt-aware cup acceptance cone and an altitude-dependent wind acting on the ball.
- **Drones B and C — Couriers**: deliver packages to $M = 3$ waypoints distributed across three altitude bands ($z \in \{5, 12, 20\}$ m). Each must plan a 3D TSP route accounting for climb energy costs that scale with $\Delta z$ and drone mass.
- **Drone D — Scout**: searches a $50 \times 50 \times 12$ m volumetric SAR zone. Survivors may be located at any altitude within $z \in [0, 12]$ m; the scout optimises its altitude to maximise the 3D sensor cone coverage over the highest-belief voxels.
- **Drone E — Formation Lead**: holds a 3D figure-eight patrol orbit at $z \in [15, 20]$ m during Phases 1–3, then coordinates the altitude-stratified ascent to the $z = 60$ m star formation in Phase 4.

**Ball launcher (3D)**: the launcher sits on a mast of height $z_0 = 10$ m on an 8 m circle around the catch-zone centre $\mathbf{c}_1 = (80, 80, 0)$ m and throws up to $N_{balls} = 3$ tennis balls (quadratic drag, $k \approx 0.0202$ m⁻¹) one at a time, with speed $v_0 \sim U(10, 20)$ m/s, elevation $\theta_0 \sim U(-10°, 60°)$ and azimuth jitter $\pm 45°$ around the centre direction. Phase 1 completes at the first catch and fails when all balls are lost below $z_{min} = 0.5$ m or at $t = 60$ s.

**Four-phase mission overview** (3D):

| Phase | Name | Actor(s) | Key 3D Addition | Time Budget |
|-------|------|----------|-----------------|-------------|
| 1 | Aerial Ball Catching | Drone A | Altitude-choosing predictive catch point, tilt-aware cup, wind on the ball | 0–60 s |
| 2 | Logistics Delivery | Drones B, C | 3D TSP with altitude-band waypoints + climb energy model | 60–160 s |
| 3 | Search & Rescue | Drone D | Volumetric 3D belief map + altitude-optimised sensor cone | 160–240 s |
| 4 | Grand Formation | All 5 | Altitude-stratified ascent choreography + 3D ORCA | 240–300 s |

**Objective**: same mission score as S100

$$S_{mission} = \sum_{p=1}^{4} w_p \cdot \mathbf{1}[\text{phase } p \text{ completed within budget}]
  - \lambda_{OT} \cdot \max(0,\; T_{actual} - T_{total})$$

with phase weights $(w_1, w_2, w_3, w_4) = (2, 3, 3, 2)$ and overtime penalty $\lambda_{OT} = 0.05$ pts/s.

---

## Mathematical Model

### Phase 1 — 3D Aerial Ball Catching

**Ball dynamics with wind** (the ball model of [S094](../S094_ball_catching.md) plus a wind
$\mathbf{w}(z, t)$ that grows with height):

$$\dot{\mathbf{p}}_b = \mathbf{v}_b, \qquad
\dot{\mathbf{v}}_b = \mathbf{g} - k\,\|\mathbf{v}_b - \mathbf{w}\|\,(\mathbf{v}_b - \mathbf{w}), \qquad
\mathbf{w}(z) = \bar w_{ref} \left(\frac{z}{z_{ref}}\right)^{1/7} \hat{\mathbf{e}}_w$$

with $\bar w_{ref} = 3$ m/s at $z_{ref} = 10$ m. The EKF of S100 is extended with the wind speed as
a seventh state so that the predictor can learn the drift during the flight.

**Thrust-limited catcher**: Drone A's acceleration is limited by its thrust,
$\|\mathbf{u} + g\hat{\mathbf{z}}\| \le T_{max,A}/m_A = 20$ m/s², so it can accelerate
horizontally by up to $\sqrt{20^2 - g^2} = 17.4$ m/s² but upwards by only $20 - g = 10.2$ m/s².
Its speed limit is $v_{max,A} = 10$ m/s (fleet table) and its altitude window $[0.3, 20]$ m.

**Tilt-aware cup**: the cup axis follows the thrust direction,
$\hat{\mathbf{n}} = (\mathbf{u} + g\hat{\mathbf{z}})/\|\mathbf{u} + g\hat{\mathbf{z}}\|$, and a catch
requires closest approach $d \le r_c = 0.15$ m **and**
$\angle(-\mathbf{v}_{rel}, \hat{\mathbf{n}}) \le \alpha_{cup} = 40°$, with
$\mathbf{v}_{rel} = \mathbf{v}_b - \mathbf{v}_A$.

**Altitude-choosing predictive catch point**: the candidates of S100's planner are extended from
$\hat z_b \in [0.5, 4.0]$ m to the whole descending branch with $\hat z_b \in [0.5, 20]$ m. Among the
reachable candidates (thrust-limited reach time plus the 0.1 s margin), the planner picks the one
that minimises

$$J(t_j) = t_j + \beta \,\|\mathbf{u}_{arr}(t_j)\|, \qquad \beta = 0.05\ \text{s}^3/\text{m},$$

where $\mathbf{u}_{arr}$ is the acceleration the tracker still needs at arrival, so that the drone
arrives decelerated and level enough for the cup cone. The S100 law (earliest point inside
$[0.5, 4.0]$ m, no tilt term) is the 2D baseline on the same throws (common random numbers).

### Phase 2 — 3D TSP with Altitude-Band Energy Model

Delivery waypoints span three altitude bands:

$$\mathbf{w}_1 = (20, 70, 5)\,\text{m}, \quad \mathbf{w}_2 = (75, 60, 12)\,\text{m}, \quad \mathbf{w}_3 = (40, 40, 20)\,\text{m}$$

**Heterogeneous energy model**: for drone $i$ with mass $m_i$ flying a leg of 3D displacement $\Delta\mathbf{r}$:

$$\Delta E_i = c_{fly} \cdot \|\Delta\mathbf{r}_{xy}\| + c_{climb,i} \cdot \max(0, \Delta z) + c_{fly} \cdot |\Delta z|$$

where the climb penalty scales with mass:

$$c_{climb,i} = c_{fly} \cdot \frac{m_i \cdot g}{T_{max,i}} \cdot \gamma_{climb}$$

with gravitational acceleration $g = 9.81$ m/s$^2$, maximum thrust $T_{max,i}$ from the fleet table, and $\gamma_{climb} = 2.5$ (empirical climb efficiency factor). Descents consume $c_{fly} \cdot |\Delta z|$ (no recovery).

**3D nearest-neighbour TSP**: cost matrix uses the heterogeneous 3D energy cost $\Delta E_i(\mathbf{w}_j)$ rather than Euclidean distance, ensuring higher-mass couriers avoid altitude-intensive legs:

$$D_{total} = \sum_{k \in \mathcal{R}_B} \Delta E_B^{(k)} + \sum_{k \in \mathcal{R}_C} \Delta E_C^{(k)}$$

**Battery safety check** (3D leg pre-commitment):

$$E_i - \Delta E_i(d_{next}) - \Delta E_i(d_{home}) \geq E_{reserve}$$

where $d_{home}$ is the full 3D distance back to the base pad.

### Phase 3 — Volumetric Bayesian SAR Belief Map

The SAR search zone is extended to a 3D volume: $\mathcal{V} = [0, 50] \times [0, 50] \times [0, 12]$ m, discretised into a $100 \times 100 \times 24$ voxel grid (0.5 m resolution in all axes).

**3D Gaussian prior** (per survivor $s$):

$$P_0^{(s)}(\mathbf{x}) \propto \exp\!\left(-\frac{(x - x_{LKP}^{(s)})^2 + (y - y_{LKP}^{(s)})^2}{2\,\sigma_{xy}^2} - \frac{(z - z_{LKP}^{(s)})^2}{2\,\sigma_z^2}\right)$$

with $\sigma_{xy} = 8$ m (horizontal spread, same as original) and $\sigma_z = 3$ m (vertical spread, tighter due to structural constraints). Last-known positions: $\mathbf{x}_{LKP}^{(1)} = (15, 20, 2)$ m, $\mathbf{x}_{LKP}^{(2)} = (35, 35, 4)$ m.

**3D sensor cone model**: the scout at position $\mathbf{p}_D = (x_D, y_D, z_D)$ projects a cone downward and laterally. A voxel $\mathbf{x} = (x, y, z)$ lies within the sensor footprint if:

$$\sqrt{(x - x_D)^2 + (y - y_D)^2 + (z - z_D)^2} \leq r_s + \kappa \cdot |z_D - z|$$

where $r_s = 3$ m is the base sensor radius and $\kappa = 0.4$ is the cone half-angle tangent. Detection probability within footprint: $P_d = 0.85$; outside: $P_{fa} = 0.01$.

**Altitude-optimised greedy frontier**: the scout selects the voxel and altitude jointly:

$$(\mathbf{w}_D^*, z_D^*) = \arg\max_{\mathbf{w}_{xy}, z_D} \sum_{\mathbf{x} \in \mathcal{C}(\mathbf{w}_{xy}, z_D)} B_t(\mathbf{x})$$

where $\mathcal{C}(\mathbf{w}_{xy}, z_D)$ is the set of voxels in the sensor cone from position $(\mathbf{w}_{xy}, z_D)$. The scout trades off flying higher (wider cone, lower resolution) against flying lower (narrower cone, higher $P_d$) by choosing $z_D \in [4, 12]$ m to maximise total expected information gain.

**3D Bayesian update** (per survivor, identical update form extended to 3D voxels):

$$B_{t+1}^{(s)}(\mathbf{x}) = \frac{P(z \mid H_\mathbf{x}^{(s)}) \cdot B_t^{(s)}(\mathbf{x})}{\displaystyle\sum_{\mathbf{x}'} P(z \mid H_{\mathbf{x}'}^{(s)}) \cdot B_t^{(s)}(\mathbf{x}')}$$

**3D joint entropy**:

$$H_{joint}(t) = -\sum_{\mathbf{x} \in \mathcal{V}} \tilde{B}_t(\mathbf{x}) \log \tilde{B}_t(\mathbf{x})$$

### Phase 4 — Altitude-Stratified Ascent Choreography

**Ascent phase** (before star lock-in): to avoid mid-air conflicts during the climb from diverse end-of-Phase-3 positions to $z_{show} = 60$ m, each drone $k$ follows a stratified climb profile:

$$z_k^{climb}(t) = z_k^{start} + (z_{show} - z_k^{start}) \cdot \Phi\!\left(\frac{t - t_{phase4}}{\tau_{climb,k}}\right)$$

where $\Phi(\xi) = 3\xi^2 - 2\xi^3$ is a cubic smoothstep easing function ensuring zero acceleration at start and end, and each drone's climb schedule is staggered by $\delta t_{stagger} = 3$ s to maintain vertical separation $\Delta z_{sep} \geq 5$ m between consecutively ascending drones:

$$\tau_{climb,k} = \frac{|z_{show} - z_k^{start}|}{v_{z,max}} + k \cdot \delta t_{stagger}$$

with $v_{z,max} = 3$ m/s.

**3D Hungarian assignment** (same as S100, applied in full 3D at $z = 60$ m):

$$\mathbf{g}_k = \begin{bmatrix}
  50 + R_{star} \cos\!\bigl(\tfrac{2\pi k}{5} - \tfrac{\pi}{2}\bigr) \\
  50 + R_{star} \sin\!\bigl(\tfrac{2\pi k}{5} - \tfrac{\pi}{2}\bigr) \\
  z_{show}
\end{bmatrix}, \qquad k = 0,\ldots,4$$

$$\min_\sigma \sum_{k=0}^{4} \|\mathbf{p}_k(t_{phase4}) - \mathbf{g}_{\sigma(k)}\|$$

**3D ORCA-lite with vertical repulsion**: the repulsion law is extended to act in all three axes. When two drones $i, j$ satisfy $d_{ij} < r_{safe} = 1.5$ m (expanded from 1.2 m to account for vertical proximity):

$$\mathbf{F}_{rep,i}^{(j)} = k_{rep}\!\left(\frac{1}{d_{ij}} - \frac{1}{r_{safe}}\right)\frac{\hat{\mathbf{n}}_{ij}}{d_{ij}^2}$$

where $\hat{\mathbf{n}}_{ij} = (\mathbf{p}_i - \mathbf{p}_j)/d_{ij}$ is now a full 3D unit vector.

**3D PID position controller** with altitude gain scaling:

$$\mathbf{u}_k(t) = K_p\,\mathbf{e}_k(t) + K_i \sum_{\tau \leq t} \mathbf{e}_k(\tau)\,\Delta t + K_d\,\frac{\mathbf{e}_k(t) - \mathbf{e}_k(t-\Delta t)}{\Delta t}$$

with a separate vertical gain during ascent phase: $K_p^z = K_p \cdot m_k / m_{min}$ (heavier drones receive a proportionally stronger vertical command to overcome gravity), where $m_{min} = 0.8$ kg.

### Altitude-Stratified Task Allocation

Before Phase 2 begins, the mission sequencer solves a constrained assignment problem to determine which courier handles which altitude-band waypoint, subject to thrust-to-weight ratio constraints:

$$\text{Assign waypoint } j \text{ to drone } i \quad \text{if} \quad \frac{T_{max,i}}{m_i \cdot g} \geq 1 + \frac{\Delta z_j}{v_{z,max} \cdot \tau_{climb,max}}$$

where $\Delta z_j = z_{wp,j} - z_{base}$ is the altitude gain required. This ensures that high-altitude waypoints are reserved for drones with sufficient thrust margin.

### Mission Sequencer (3D)

Transition logic is identical to S100 with the additional condition that altitude must be within tolerance for Phase 4 convergence:

$$\text{Phase 4 complete} \iff \max_k \|\mathbf{p}_k - \mathbf{g}_{\sigma^*(k)}\| \leq \varepsilon_{form} = 0.20 \text{ m}$$

(tolerance relaxed slightly from 0.15 m to 0.20 m to account for 3D convergence difficulty at $z = 60$ m).

**Overall mission score** (unchanged):

$$S_{mission} = \sum_{p=1}^{4} w_p \cdot s_p - \lambda_{OT} \cdot \max(0,\; t_4^{end} - T_{total})$$

---

## Key Parameters

| Parameter | Symbol | Phase | Value |
|-----------|--------|-------|-------|
| Arena dimensions | — | All | $100 \times 100 \times 60$ m (ceiling raised from 30 m) |
| Fleet size | $N$ | All | 5 heterogeneous drones |
| Drone masses | $m_A \ldots m_E$ | All | 0.8, 1.2, 1.2, 0.9, 0.8 kg |
| Drone max speeds | $V_{max}$ | All | 10, 7, 7, 6, 8 m/s |
| **Phase 1 — 3D Ball Catching** | | | |
| Launcher mast height | $z_0$ | 1 | 10 m |
| Throw speed / elevation / azimuth jitter | $v_0$, $\theta_0$, $\Delta\psi$ | 1 | U(10, 20) m/s / U(−10°, 60°) / ±45° |
| Catcher altitude window | — | 1 | [0.3, 20] m |
| Catcher thrust-to-mass limit | $T_{max,A}/m_A$ | 1 | 20 m/s² |
| Cup acceptance half-angle | $\alpha_{cup}$ | 1 | 40° |
| Wind at reference height | $\bar w_{ref}$ ($z_{ref} = 10$ m) | 1 | 3 m/s |
| Catch-altitude cost weight | $\beta$ | 1 | 0.05 s³/m |
| Net catch radius / lowest catchable height | $r_c$, $z_{min}$ | 1 | 0.15 m / 0.5 m |
| Balls per phase | $N_{balls}$ | 1 | 3 |
| **Phase 2 — 3D Logistics** | | | |
| Waypoint altitudes | $z_{wp}$ | 2 | 5, 12, 20 m |
| Climb penalty factor | $\gamma_{climb}$ | 2 | 2.5 |
| Battery safety reserve | $E_{reserve}$ | 2 | 15 % |
| **Phase 3 — Volumetric SAR** | | | |
| SAR volume | $\mathcal{V}$ | 3 | $50 \times 50 \times 12$ m |
| Voxel resolution | — | 3 | 0.5 m (all axes) |
| Prior vertical spread | $\sigma_z$ | 3 | 3.0 m |
| Cone half-angle tangent | $\kappa$ | 3 | 0.4 |
| Scout altitude range | $z_D$ | 3 | 4–12 m |
| Detection probability | $P_d$ | 3 | 0.85 |
| **Phase 4 — 3D Formation** | | | |
| Show altitude | $z_{show}$ | 4 | 60.0 m (raised from 50 m) |
| Star circumradius | $R_{star}$ | 4 | 8.0 m |
| Ascent stagger interval | $\delta t_{stagger}$ | 4 | 3.0 s |
| Max vertical speed | $v_{z,max}$ | 4 | 3.0 m/s |
| ORCA activation radius | $r_{safe}$ | 4 | 1.5 m (increased for 3D) |
| Formation convergence threshold | $\varepsilon_{form}$ | 4 | 0.20 m |
| **Scoring** | | | |
| Phase weights | $(w_1, w_2, w_3, w_4)$ | All | (2, 3, 3, 2) |
| Overtime penalty | $\lambda_{OT}$ | All | 0.05 pts/s |
| Maximum score | $S_{max}$ | All | 10 pts |

---

## Expected Output

- **3D Phase 1 plot**: volumetric view of the ball trajectories (blue) and Drone A's path (red) from the 10 m mast to the catch, with the catch points (green) at their chosen altitudes; histogram of catch altitudes; per-throw catch rate of the altitude-choosing planner against the S100 law (4 m window, no tilt term) on the same throws.
- **3D Phase 2 plot**: isometric 3D view of courier routes spanning all three altitude bands; altitude-coloured waypoint markers (green = delivered, grey = missed); heterogeneous energy consumption comparison bar chart for Drones B and C.
- **Volumetric Phase 3 visualisation**: three orthogonal cross-section slices (xy, xz, yz) of the final 3D belief map for each survivor; voxel belief rendered as a 3D scatter with alpha proportional to probability; scout altitude profile over Phase 3 time window; 3D entropy decay curve.
- **Stratified ascent plot**: 3D trajectory of all five drones during Phase 4, colour-coded by drone; altitude vs time showing staggered climb profiles and final lock-in at $z = 60$ m; per-drone convergence error vs time with 0.20 m threshold line.
- **Mission dashboard**: identical four-panel layout to S100 with updated arena dimensions, heterogeneous battery profiles per drone, 3D score breakdown, and volumetric entropy decay.
- **Grand animation** (`s100_3d_grand_challenge.gif`): full-mission 3D animation at 15 fps with arena bounding box at $z \in [0, 60]$ m; Phase 4 shows staggered vertical ascent before star lock-in; the balls in flight are shown in Phase 1 together with the cup axis of Drone A.

---

## Extensions

1. **Wind field integration across altitude bands**: introduce a height-varying wind model (logarithmic wind profile $v_w(z) = v_{ref} (z/z_{ref})^{1/7}$) applied independently per drone; measure how altitude-stratified routing in Phase 2 interacts with the wind gradient and whether high-altitude waypoints become energetically prohibitive at $v_w > 5$ m/s.

2. **Drone failure mid-ascent (Phase 4 resilience)**: simulate a hardware fault in one drone during the stratified climb; the remaining four drones must re-solve the Hungarian assignment for a 4-vertex rhombus pattern at $z_{show}$ and re-stagger their ascent schedules — test whether a 3D replanning window of 10 s is achievable within the Phase 4 budget.

3. **Volumetric SAR with multiple altitude layers**: introduce survivors at known altitude uncertainty $\sigma_z \in \{1, 3, 6\}$ m and measure how the 3D scout's altitude-optimised cone policy reduces time-to-localisation compared to a fixed-altitude scan, as a function of $\sigma_z$.

---

## Related Scenarios

- Original 2D version: [S100 Ultimate Grand Challenge — Multi-Domain Integration](../S100_grand_challenge.md)
- 3D catching reference: [S094 Aerial Ball Catching](../S094_ball_catching.md), [S094 3D Upgrade — Aerial Ball Catching](S094_3d_ball_catching.md)
- Volumetric SAR reference: [S042 Missing Person Localization](../../03_environmental_sar/S042_missing_person.md)
- Formation reference: [S085 Drone Light Matrix Positioning](../S085_light_matrix.md), [S088 Formation Geometric Shape Morphing](../S088_formation_morphing.md)
