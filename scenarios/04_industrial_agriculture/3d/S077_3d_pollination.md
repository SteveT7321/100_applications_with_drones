# S077 3D Upgrade — Precision Pollination

**Domain**: Industrial & Agriculture | **Difficulty**: ⭐⭐⭐⭐ | **Status**: `[ ]` Not Started
**Based on**: [S077 original](../S077_pollination.md)

---

## What Changes in 3D

The original S077 already occupies a 3D bounding box ($z \in [1, 3]$ m), but the 3D treatment is
shallow in two important respects. First, the canopy is modelled as a uniform random cloud: every
flower height is drawn from a single $\text{Uniform}(1, 3)$ distribution with no vertical structure.
Real orchard canopies have discrete height layers (ground-level runners, mid-tier laterals, upper
scaffold branches), so bloom density and accessibility differ with altitude. Second, pollen
dispersal is treated as an instantaneous transfer event at the docking point; propeller downwash and
ambient airflow are ignored, even though rotor-induced flow at $z \approx 1$–$3$ m substantially
affects pollen viability and cross-contamination radius.

This upgrade adds three 3D-native components:

1. **Stratified canopy model** — flowers are drawn from $L = 3$ discrete vertical layers with
   layer-specific bloom densities, approach clearance constraints, and occlusion penalties.
2. **Altitude-resolved bloom detection** — the drone carries a downward-looking sensor with a
   field-of-view cone; bloom visibility depends on the drone's current altitude relative to each
   layer, enabling a top-down scanning pass before committing to the TSP tour.
3. **3D Gaussian pollen dispersal with rotor downwash** — pollen released at the applicator tip
   disperses according to a 3D Gaussian plume whose spread is modulated by rotor-induced vertical
   velocity; the effective transfer probability now depends on both docking error and the 3D
   downwash offset at the stigma.

---

## Problem Definition

**Setup**: An orchard column occupies a $10 \times 10 \times 4$ m volume ($z \in [0.5, 4.5]$ m).
The canopy is divided into $L = 3$ height layers:

| Layer | $z$ range (m) | Bloom fraction | Approach clearance |
|-------|--------------|----------------|--------------------|
| Lower (L1) | 0.5 – 1.5 | 30 % | Constrained — rotor ground effect below 1 m |
| Mid (L2)   | 1.5 – 3.0 | 50 % | Standard |
| Upper (L3) | 3.0 – 4.5 | 20 % | Constrained — canopy top turbulence above 4 m |

A single drone ($v_{transit} = 1.5$ m/s) equipped with a vibrating pollen applicator must
pollinate $F = 25$ flowers distributed across the three layers according to the bloom fractions.

The visit sequence is determined by a **3D layer-aware TSP**: the nearest-neighbour heuristic is
applied within each layer first (intra-layer legs), then inter-layer transitions are added with an
altitude-change penalty $w_z \Delta z$ to discourage unnecessary vertical excursions.

For each flower the drone must:

1. **Scan** — from a layer-specific survey altitude $z_{survey}^{(l)}$, confirm bloom presence with
   the conical sensor before descending.
2. **Approach** — along a minimum-snap trajectory that satisfies the $30°$ elevation constraint
   and respects the layer-specific clearance ceiling.
3. **Dock** — hover within $d_{dock} = 0.1$ m for $T_{dwell} = 1.5$ s while the applicator
   vibrates; rotor downwash displaces the effective pollen cloud downward by $\delta_z^{dw}$.
4. **Depart** — ascend to the intra-layer transit altitude $z_{transit}^{(l)}$ before the next
   leg.

**Roles**:
- **Drone**: single agent; 3D point-mass with first-order velocity response; carries pollen
  applicator and downward-facing bloom sensor (half-angle $\alpha_{fov} = 25°$).
- **Flowers**: $F = 25$ stationary targets in stratified 3D positions; pollen transfer probability
  depends on 3D docking error and downwash offset.
- **3D TSP planner**: layer-aware nearest-neighbour tour with altitude-change penalty; computed
  once after the bloom-detection scan.

**Objective**: maximise the expected number of pollinated flowers $E[N_{poll}]$ while minimising
total 3D mission time $T_{mission}$. Compare three strategies:

1. **Min-snap + layer-aware TSP** — reference method with full 3D optimisation.
2. **Min-snap + flat TSP** — same approach trajectories but tour ignores layer structure (S077
   original behaviour).
3. **Straight-line + flat TSP** — S077 baseline; used to quantify the combined benefit of 3D
   improvements.

---

## Mathematical Model

### Stratified Canopy Sampling

Let $F_l = \lfloor f_l \cdot F \rceil$ be the number of flowers in layer $l$, where $f_l$ is the
bloom fraction. Flower positions within layer $l$ are sampled as:

$$\mathbf{p}_i^{(l)} = \begin{pmatrix} x_i \\ y_i \\ z_i^{(l)} \end{pmatrix},
\quad x_i \sim \mathcal{U}(0, W), \quad y_i \sim \mathcal{U}(0, D),
\quad z_i^{(l)} \sim \mathcal{U}(z_{lo}^{(l)}, z_{hi}^{(l)})$$

where $W = D = 10$ m are the orchard dimensions.

### Altitude-Resolved Bloom Detection

The drone performs a pre-tour scan at survey altitude $z_{survey}^{(l)}$ above each layer. A
flower at position $\mathbf{p}_j$ is detectable from drone position $\mathbf{p}_{drone}$ if:

$$\|\mathbf{p}_j - \mathbf{p}_{drone}\| \leq \frac{z_{survey}^{(l)} - z_j}{\cos\alpha_{fov}}$$

equivalently, the angular offset from the nadir direction must satisfy:

$$\arctan\!\left(\frac{\sqrt{(x_j - x_d)^2 + (y_j - y_d)^2}}{z_d - z_j}\right) \leq \alpha_{fov}$$

Detection is probabilistic with success probability:

$$P_{detect}(\mathbf{p}_j, \mathbf{p}_{drone}) =
\exp\!\left(-\frac{\|\mathbf{p}_j^{xy} - \mathbf{p}_{drone}^{xy}\|^2}{2\,\sigma_{sensor}^2}\right),
\quad \sigma_{sensor} = (z_d - z_j)\tan\alpha_{fov} / 2$$

### Layer-Aware TSP with Altitude Penalty

Define the 3D inter-flower cost as:

$$c_{ij} = \|\mathbf{p}_i^{xy} - \mathbf{p}_j^{xy}\| + w_z\,|z_i - z_j|$$

where $w_z = 2.5$ is the altitude-change penalty weight (reflecting the higher energy cost and
hover-stabilisation time of vertical transitions). The nearest-neighbour heuristic greedily
minimises $c_{ij}$ at each step:

$$\pi_{k+1} = \arg\min_{j \in \mathcal{U}}\, c_{\pi_k,\, j}$$

The penalised tour cost is:

$$L_{tour}^{3D} = \sum_{k=0}^{F-1} c_{\pi_k,\,\pi_{k+1}}$$

### 3D Approach Direction Constraint

For a flower in layer $l$, the approach elevation angle $\theta_{el}$ is clamped to a
layer-specific range to respect clearance ceilings:

$$\theta_{el}^{(l)} = \begin{cases}
45° & l = 1 \text{ (lower layer — steeper to avoid ground effect)} \\
30° & l = 2 \text{ (mid layer — standard bee-like angle)} \\
20° & l = 3 \text{ (upper layer — shallower to stay below canopy top)}
\end{cases}$$

The 3D unit approach vector for flower $i$ in layer $l$ with horizontal bearing $\varphi_i$ is:

$$\hat{\mathbf{d}}_i = \begin{pmatrix}
\sin\theta_{el}^{(l)}\cos\varphi_i \\
\sin\theta_{el}^{(l)}\sin\varphi_i \\
\cos\theta_{el}^{(l)}
\end{pmatrix}$$

The approach start point is:

$$\mathbf{p}_{start}^{(i)} = \mathbf{p}_i^{flower} - d_{standoff}\,\hat{\mathbf{d}}_i$$

This places the start point below and to the side of the flower, consistent with the full 3D
geometry of the elevation angle rather than the 2D projection used in S077.

### Minimum-Snap Trajectory (3D, unchanged structure)

The degree-7 polynomial is solved per axis independently with boundary conditions adapted to the
3D approach start point and the layer-specific entry speed $v_{approach}^{(l)}$:

| Layer | $v_{approach}^{(l)}$ | Rationale |
|-------|----------------------|-----------|
| L1    | 0.3 m/s | Slow near ground to reduce downwash disturbance |
| L2    | 0.5 m/s | Standard |
| L3    | 0.4 m/s | Slow near canopy top to avoid foliage contact |

The boundary conditions at $t = 0$ (approach start) and $t = T_{seg}$ (flower centre) are:

$$\mathbf{b}_{bc} = \bigl[p_{start},\; v_{approach}^{(l)}\hat{d}_i,\; \mathbf{0},\; \mathbf{0},\;
p_{flower},\; \mathbf{0},\; \mathbf{0},\; \mathbf{0}\bigr]^\top$$

independently for each of the three spatial axes.

### 3D Pollen Dispersal with Rotor Downwash

During the dwell phase, pollen released at the applicator tip $\mathbf{p}_{tip}$ (offset
$\Delta z_{tip} = -0.05$ m below the drone body centre) disperses as a 3D Gaussian cloud. The
pollen concentration at the flower stigma $\mathbf{p}_{stigma}$ (the target point on the flower)
is:

$$C(\mathbf{p}_{stigma}) = \frac{Q}{(2\pi)^{3/2}\,\sigma_x\sigma_y\sigma_z}
\exp\!\left(-\frac{\Delta x^2}{2\sigma_x^2}
           -\frac{\Delta y^2}{2\sigma_y^2}
           -\frac{(\Delta z - \delta_z^{dw})^2}{2\sigma_z^2}\right)$$

where:
- $Q$ is the pollen release rate (normalised to 1)
- $(\Delta x, \Delta y, \Delta z) = \mathbf{p}_{stigma} - \mathbf{p}_{tip}$
- $\sigma_x = \sigma_y = \sigma_{horiz}$ is the horizontal pollen spread
- $\sigma_z$ is the vertical pollen spread
- $\delta_z^{dw}$ is the downwash-induced vertical displacement of the pollen centroid

The downwash displacement is modelled as a function of rotor thrust $T_{rotor}$ and the
ground-to-drone clearance $h = z_{drone} - z_{ground}$:

$$\delta_z^{dw} = k_{dw}\,\frac{T_{rotor}}{h^2}, \quad k_{dw} = 0.02 \text{ m}^3/\text{N}$$

The effective pollen transfer probability at flower $i$ becomes:

$$P_t^{(i)} = P_{transfer,base}^{(i)} \cdot \frac{C(\mathbf{p}_{stigma}^{(i)})}{C_{max}}$$

where $P_{transfer,base}^{(i)} = \exp(-e_{dock}^{(i)} / \sigma_{dock})$ is the geometric docking
probability from S077, and $C_{max}$ is the maximum concentration (achieved when
$\mathbf{p}_{tip}$ is perfectly aligned with the stigma and downwash is zero).

The 3D docking error accounts for the full 3D displacement during hover:

$$e_{dock}^{(i)} = \frac{1}{N_{dwell}}\sum_{k=1}^{N_{dwell}}
\left\|\mathbf{p}_{drone}(t_k) + \begin{pmatrix}0\\0\\\Delta z_{tip}\end{pmatrix}
- \mathbf{p}_{stigma}^{(i)}\right\|$$

### Layer-Specific Transit Altitudes

To avoid inter-layer collisions and respect canopy structure, transit between flowers uses a
layer-indexed safe altitude:

$$z_{transit}^{(l)} = \begin{cases}
z_{hi}^{(1)} + 0.3 = 1.8 \text{ m} & l = 1 \\
z_{hi}^{(2)} + 0.3 = 3.3 \text{ m} & l = 2 \\
z_{hi}^{(3)} + 0.3 = 4.8 \text{ m} & l = 3
\end{cases}$$

Cross-layer transits use the higher of the two layer transit altitudes.

### Expected Pollination and Mission Time

The expected number of pollinated flowers:

$$E[N_{poll}] = \sum_{i=1}^{F} P_t^{(i)}$$

Mission time accounting for all transit, descent, approach, dwell, and ascent segments:

$$T_{mission} = \sum_{k=0}^{F-1} \left(
  T_{transit,k} + T_{descent,k} + T_{approach,k} + T_{dwell} + T_{ascent,k}
\right)$$

where $T_{transit,k}$, $T_{descent,k}$, $T_{ascent,k}$ are computed from segment distances and
$v_{transit}$, and $T_{approach,k} = d_{standoff} / v_{approach}^{(l)}$.

---

## Key 3D Additions

- **Stratified canopy geometry**: flowers drawn from $L = 3$ discrete height layers with distinct
  bloom densities (30 / 50 / 20 %) rather than a single uniform $z$ distribution.
- **Altitude-resolved bloom detection**: conical sensor model ($\alpha_{fov} = 25°$) with
  Gaussian detection probability; pre-tour scan pass at each layer survey altitude.
- **Layer-specific approach elevation angles**: 45° / 30° / 20° for lower / mid / upper layers
  to respect ground-effect and canopy-top constraints.
- **Layer-specific approach speeds**: 0.3 / 0.5 / 0.4 m/s to modulate rotor downwash near
  boundaries.
- **3D Gaussian pollen dispersal with downwash offset** $\delta_z^{dw}$: replaces the binary
  docking-radius model; transfer probability is the product of geometric docking probability and
  normalised pollen concentration at the stigma.
- **Layer-aware TSP with altitude-change penalty** ($w_z = 2.5$): minimises unnecessary vertical
  excursions while still allowing cross-layer visits when bloom density warrants it.
- **Layer-indexed transit altitudes**: 1.8 / 3.3 / 4.8 m to prevent inter-layer trajectory
  conflicts.

---

## Key Parameters

| Parameter | Symbol | Value |
|-----------|--------|-------|
| Orchard footprint | $W \times D$ | 10 × 10 m |
| Canopy height range | $z$ | 0.5 – 4.5 m |
| Number of layers | $L$ | 3 |
| Bloom fractions (L1/L2/L3) | $f_l$ | 0.30 / 0.50 / 0.20 |
| Number of flowers | $F$ | 25 |
| Sensor half-angle | $\alpha_{fov}$ | 25° |
| Layer survey altitudes | $z_{survey}^{(l)}$ | 2.0 / 3.5 / 5.0 m |
| Approach elevation angles (L1/L2/L3) | $\theta_{el}^{(l)}$ | 45° / 30° / 20° |
| Approach standoff distance | $d_{standoff}$ | 0.8 m |
| Approach entry speeds (L1/L2/L3) | $v_{approach}^{(l)}$ | 0.3 / 0.5 / 0.4 m/s |
| Transit speed | $v_{transit}$ | 1.5 m/s |
| Layer transit altitudes (L1/L2/L3) | $z_{transit}^{(l)}$ | 1.8 / 3.3 / 4.8 m |
| Docking radius / transfer scale | $d_{dock}$, $\sigma_{dock}$ | 0.1 m |
| Dwell time | $T_{dwell}$ | 1.5 s |
| GPS noise | $\sigma_{GPS}$ | 0.1 m per axis |
| Applicator tip offset | $\Delta z_{tip}$ | -0.05 m |
| Horizontal pollen spread | $\sigma_{horiz}$ | 0.08 m |
| Vertical pollen spread | $\sigma_z$ | 0.06 m |
| Downwash coefficient | $k_{dw}$ | 0.02 m³/N |
| Altitude-change penalty weight | $w_z$ | 2.5 |
| Polynomial degree | — | 7 (minimum-snap) |
| Simulation timestep | $\Delta t$ | 0.05 s |
| Random seed | — | 42 |

---

## Expected Output

- **3D stratified canopy plot**: flowers colour-coded by layer (L1 = amber, L2 = green, L3 =
  purple) with layer bounding planes shown as translucent horizontal slabs; drone start position
  and survey altitude markers annotated.
- **Bloom detection scan visualisation**: conical sensor footprint projected at each survey
  altitude; detected vs. undetected flowers distinguished by marker style.
- **3D tour trajectory**: full drone path including transit, descent, approach, dwell, and ascent
  segments for all three strategies; layer-aware TSP in red, flat TSP in blue dashed, straight-line
  baseline in grey dotted.
- **Altitude time series**: $z(t)$ for all three strategies; layer boundaries shown as horizontal
  dashed lines; layer-aware strategy should show lower total altitude variation.
- **Per-flower transfer probability**: $P_t^{(i)}$ across the 25 visits for each strategy,
  annotated with layer membership.
- **Pollen concentration heatmap**: 2D cross-section of $C(\mathbf{p})$ at the flower stigma
  height for a representative lower-layer flower, showing how downwash shifts the concentration
  centroid downward.
- **Mission time vs. $E[N_{poll}]$ scatter**: one point per strategy; trade-off between
  thoroughness and speed.
- **Summary bar chart**: $E[N_{poll}]$ and $\eta_{poll}$ (\%) for all three strategies; total
  flower count reference line at $F = 25$.
- **Pollination tour animation (GIF)**: 3D drone trail growing frame-by-frame; flowers turn from
  hollow to filled circles when visited; layer boundaries visible as translucent planes.

**Expected metric targets**:

| Metric | Layer-aware min-snap | Flat min-snap | Straight baseline |
|--------|----------------------|---------------|-------------------|
| Mean docking error | $< 0.12$ m | $< 0.12$ m | $\approx 0.20$ m |
| Downwash offset $\delta_z^{dw}$ (L1) | $\approx 0.04$ m | $\approx 0.04$ m | $\approx 0.06$ m |
| Transfer success rate $\eta_{poll}$ | $> 30\%$ | $> 25\%$ | $< 20\%$ |
| $E[N_{poll}]$ | $> 8$ / 25 | $> 6$ / 25 | $< 5$ / 25 |
| $T_{mission}$ | lower than flat TSP | — | lowest (no smoothing) |

> The key 3D result is that the layer-aware tour reduces unnecessary vertical excursions
> (quantified by total $\sum |\Delta z|$ across all transit legs) while the downwash model
> reveals that lower-layer flowers incur a systematic transfer penalty absent in the 2D
> uniform-cloud model.

---

## Extensions

1. **Wind-layer interaction**: add altitude-varying horizontal wind (wind shear profile
   $u(z) = u_{ref}(z/z_{ref})^\alpha$) that tilts the pollen Gaussian plume; re-optimise the
   horizontal approach bearing $\varphi_i$ per flower to align the plume with the stigma.
2. **Adaptive survey path**: replace the fixed layer survey altitude with a frontier-based 3D
   coverage path that maximises bloom detections per unit flight time; compare detection rate
   against the fixed-altitude lawnmower scan.
3. **Multi-layer 2-opt TSP refinement**: apply 2-opt edge swaps to the layer-aware
   nearest-neighbour tour, accepting swaps that reduce $L_{tour}^{3D}$; measure improvement
   in $E[N_{poll}]$ per unit $T_{mission}$.
4. **Pollen load depletion across layers**: model the pollen reservoir as a finite quantity
   consumed at rate $P_t^{(i)} \cdot q_{flower}$ per visit; add a mid-mission recharging stop
   at a pollen depot at ground level; re-plan the inter-layer tour to minimise depot visits
   while maintaining $E[N_{poll}]$ above a threshold.
5. **Multi-drone layer partition**: assign one drone per canopy layer; solve the inter-layer
   conflict-avoidance problem (drones must not occupy the same $z$ band during transit);
   compare makespan and total $E[N_{poll}]$ against the single-drone 3D tour.
6. **Canopy occlusion model**: add cylindrical trunk and branch obstacles; use a 3D visibility
   graph to find collision-free approach paths; evaluate how occlusion increases tour length
   and reduces $E[N_{poll}]$ relative to the open-air canopy assumption.

---

## Related Scenarios

- Original 2D/3D version: [S077 Precision Pollination](../S077_pollination.md)
- Domain prerequisites: [S063 Single-Plant Precision Spraying](../S063_precision_spraying.md),
  [S070 Greenhouse Climate Patrol](../S070_greenhouse_climate_patrol.md)
- Truly 3D SAR reference (sensor cone model): [S041](../../03_environmental_sar/S041_area_coverage_search.md)
- Pollen plume physics reference (Gaussian dispersion): [S045](../../03_environmental_sar/S045_plume_tracing.md)
- Follow-ups: [S078 Fruit Size Estimation](../S078_fruit_size_estimation.md)
