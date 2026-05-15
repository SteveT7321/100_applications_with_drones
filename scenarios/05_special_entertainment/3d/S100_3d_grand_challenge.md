# S100 3D Upgrade — Grand Challenge: Multi-Domain Integration

**Domain**: Special Ops & Entertainment | **Difficulty**: ⭐⭐⭐⭐⭐ | **Status**: `[ ]` Not Started
**Based on**: [S100 original](../S100_grand_challenge.md)

---

## What Changes in 3D

The original S100 already declares a $100 \times 100 \times 30$ m arena, but in practice each phase collapses altitude to a single operating band: Phase 1 evasion is restricted to the horizontal plane (perpendicular-escape velocity has `z = 0`), Phase 2 couriers fly at a fixed delivery height of $z = 5$ m, Phase 3 scout maintains a constant $z = 8$ m AGL, and Phase 4 lifts all five drones to a single star plane at $z = 50$ m. The z-axis is never used as a tactical or navigational resource.

This 3D upgrade activates the full volumetric arena across all four phases:

- **Phase 1**: the rogue target executes full 3D evasion (altitude hops, helix spiral, dive-and-run), and Drone A's PNG command is computed in $\mathbb{R}^3$ with no horizontal restriction.
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
| A | Interceptor | 0.8 | 16.0 | 8.0 | 10.0 |
| B | Courier 1 | 1.2 | 20.0 | 14.0 | 7.0 |
| C | Courier 2 | 1.2 | 20.0 | 14.0 | 7.0 |
| D | Scout | 0.9 | 14.0 | 10.0 | 6.0 |
| E | Formation Lead | 0.8 | 16.0 | 8.0 | 8.0 |

**Roles** (3D extensions):
- **Drone A — Interceptor**: executes full 3D PNG against a rogue target that now employs altitude-varying evasion tactics (helix spiral, dive-and-run, barrel roll).
- **Drones B and C — Couriers**: deliver packages to $M = 3$ waypoints distributed across three altitude bands ($z \in \{5, 12, 20\}$ m). Each must plan a 3D TSP route accounting for climb energy costs that scale with $\Delta z$ and drone mass.
- **Drone D — Scout**: searches a $50 \times 50 \times 12$ m volumetric SAR zone. Survivors may be located at any altitude within $z \in [0, 12]$ m; the scout optimises its altitude to maximise the 3D sensor cone coverage over the highest-belief voxels.
- **Drone E — Formation Lead**: holds a 3D figure-eight patrol orbit at $z \in [15, 20]$ m during Phases 1–3, then coordinates the altitude-stratified ascent to the $z = 60$ m star formation in Phase 4.

**Rogue target (3D)**: employs one of three 3D evasion tactics, selected randomly at mission start:
1. **Helix spiral**: $\dot{\mathbf{p}}_{target} = v_e [-\sin(\omega t),\; \cos(\omega t),\; \tan\alpha]^\top / \sec\alpha$ with $\omega = v_e \cos\alpha / R_{helix}$
2. **Dive-and-run**: drops to $z_{min} = 2$ m in 3 s, then escapes laterally at full speed
3. **Altitude hop**: alternates between $z_{lo} = 3$ m and $z_{hi} = 25$ m on 8 s intervals

**Four-phase mission overview** (3D):

| Phase | Name | Actor(s) | Key 3D Addition | Time Budget |
|-------|------|----------|-----------------|-------------|
| 1 | Pursuit & Intercept | Drone A | Full 3D PNG vs altitude-varying evader | 0–60 s |
| 2 | Logistics Delivery | Drones B, C | 3D TSP with altitude-band waypoints + climb energy model | 60–160 s |
| 3 | Search & Rescue | Drone D | Volumetric 3D belief map + altitude-optimised sensor cone | 160–240 s |
| 4 | Grand Formation | All 5 | Altitude-stratified ascent choreography + 3D ORCA | 240–300 s |

**Objective**: same mission score as S100

$$S_{mission} = \sum_{p=1}^{4} w_p \cdot \mathbf{1}[\text{phase } p \text{ completed within budget}]
  - \lambda_{OT} \cdot \max(0,\; T_{actual} - T_{total})$$

with phase weights $(w_1, w_2, w_3, w_4) = (2, 3, 3, 2)$ and overtime penalty $\lambda_{OT} = 0.05$ pts/s.

---

## Mathematical Model

### Phase 1 — Full 3D Proportional Navigation Guidance

The LOS vector in $\mathbb{R}^3$:

$$\mathbf{r}(t) = \mathbf{p}_{target}(t) - \mathbf{p}_A(t), \qquad R = \|\mathbf{r}\|, \qquad \hat{\boldsymbol{\lambda}} = \frac{\mathbf{r}}{R}$$

The LOS angular rate in 3D (no horizontal restriction):

$$\dot{\boldsymbol{\lambda}} = \frac{\dot{\mathbf{r}} - (\dot{\mathbf{r}} \cdot \hat{\boldsymbol{\lambda}})\hat{\boldsymbol{\lambda}}}{R}$$

PNG acceleration command fully in $\mathbb{R}^3$:

$$\ddot{\mathbf{p}}_A^{cmd} = N \cdot V_{close} \cdot \dot{\boldsymbol{\lambda}}, \qquad V_{close} = -\dot{\mathbf{r}} \cdot \hat{\boldsymbol{\lambda}}$$

Altitude safety saturation is applied after the guidance step:

$$z_A \leftarrow \text{clip}(z_A,\; z_{min,A},\; z_{max,A}) = \text{clip}(z_A,\; 1.0,\; 55.0)$$

**3D Helix evasion model** (tactic 1):

$$\dot{\mathbf{p}}_{target} = v_e \begin{bmatrix} -\sin(\omega t) \\ \cos(\omega t) \\ \tan\alpha \end{bmatrix} \cos\alpha, \qquad \omega = \frac{v_e \cos\alpha}{R_{helix}}$$

where $\alpha = 30°$ is the helix pitch angle and $R_{helix} = 6$ m. The z-component is clipped to keep $z_{target} \in [2, 28]$ m.

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
| **Phase 1 — 3D Pursuit** | | | |
| PNG nav constant | $N$ | 1 | 3 |
| Helix radius | $R_{helix}$ | 1 | 6.0 m |
| Helix pitch angle | $\alpha$ | 1 | 30° |
| Dive altitude floor | $z_{min,target}$ | 1 | 2.0 m |
| Altitude hop range | $[z_{lo}, z_{hi}]$ | 1 | [3, 25] m |
| Capture radius | $r_{capture}$ | 1 | 1.5 m |
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

- **3D Phase 1 plot**: full volumetric trajectory of Drone A (red) and rogue target (blue dashed) with z-variation visible; altitude vs time subplot distinguishing the three evasion tactics; LOS range vs time with capture radius line.
- **3D Phase 2 plot**: isometric 3D view of courier routes spanning all three altitude bands; altitude-coloured waypoint markers (green = delivered, grey = missed); heterogeneous energy consumption comparison bar chart for Drones B and C.
- **Volumetric Phase 3 visualisation**: three orthogonal cross-section slices (xy, xz, yz) of the final 3D belief map for each survivor; voxel belief rendered as a 3D scatter with alpha proportional to probability; scout altitude profile over Phase 3 time window; 3D entropy decay curve.
- **Stratified ascent plot**: 3D trajectory of all five drones during Phase 4, colour-coded by drone; altitude vs time showing staggered climb profiles and final lock-in at $z = 60$ m; per-drone convergence error vs time with 0.20 m threshold line.
- **Mission dashboard**: identical four-panel layout to S100 with updated arena dimensions, heterogeneous battery profiles per drone, 3D score breakdown, and volumetric entropy decay.
- **Grand animation** (`s100_3d_grand_challenge.gif`): full-mission 3D animation at 15 fps with arena bounding box at $z \in [0, 60]$ m; Phase 4 shows staggered vertical ascent before star lock-in; rogue target trajectory animates 3D evasion tactic in Phase 1.

---

## Extensions

1. **Wind field integration across altitude bands**: introduce a height-varying wind model (logarithmic wind profile $v_w(z) = v_{ref} (z/z_{ref})^{1/7}$) applied independently per drone; measure how altitude-stratified routing in Phase 2 interacts with the wind gradient and whether high-altitude waypoints become energetically prohibitive at $v_w > 5$ m/s.

2. **Drone failure mid-ascent (Phase 4 resilience)**: simulate a hardware fault in one drone during the stratified climb; the remaining four drones must re-solve the Hungarian assignment for a 4-vertex rhombus pattern at $z_{show}$ and re-stagger their ascent schedules — test whether a 3D replanning window of 10 s is achievable within the Phase 4 budget.

3. **Volumetric SAR with multiple altitude layers**: introduce survivors at known altitude uncertainty $\sigma_z \in \{1, 3, 6\}$ m and measure how the 3D scout's altitude-optimised cone policy reduces time-to-localisation compared to a fixed-altitude scan, as a function of $\sigma_z$.

---

## Related Scenarios

- Original 2D version: [S100 original](../S100_grand_challenge.md)
- 3D Pursuit reference: [S001 3D Upgrade](../../01_pursuit_evasion/3d/S001_3d_basic_intercept.md), [S002 3D Upgrade](../../01_pursuit_evasion/3d/S002_3d_evasive_maneuver.md)
- Volumetric SAR reference: [S042 Missing Person](../../03_environmental_sar/S042_missing_person.md)
- Formation reference: [S085 Light Matrix](../S085_light_matrix.md), [S088 Formation Morphing](../S088_formation_morphing.md)
