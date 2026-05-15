# S051 3D Upgrade — Post-Disaster Comm Network

**Domain**: Environmental & SAR | **Difficulty**: ⭐⭐⭐⭐ | **Status**: `[ ]` Not Started
**Based on**: [S051 original](../S051_post_disaster_comm.md)

---

## What Changes in 3D

The original S051 fixes every relay drone at a single hardcoded hover altitude
$z = 50$ m and evaluates coverage only by ground-projected 2D distance
(`np.linalg.norm(centroids - p_dk, axis=1) <= r_cov`). This collapses all
link-budget and obstruction geometry into the horizontal plane and ignores three
critical real-world factors:

1. **Altitude-optimised relay placement** — hovering higher extends the ground
   footprint of each drone antenna (slant-range coverage grows with altitude), but
   also increases path loss and battery drain. The optimal altitude per drone is a
   function of its population-density neighbourhood and its position in the relay
   chain.
2. **Debris-field 3D obstruction map** — collapsed buildings create irregular
   vertical obstacles (height 5–30 m) that block line-of-sight between a
   low-flying drone and ground survivors or adjacent relay nodes. The obstruction
   model must be evaluated in 3D.
3. **3D link budget** — radio free-space path loss depends on true slant range
   $d_{3D}$, not horizontal range. The coverage radius is no longer a fixed circle
   on the ground; it is the set of ground cells for which FSPL is below the
   receiver sensitivity threshold.
4. **Vertical daisy-chain topology** — when the debris field breaks horizontal
   line-of-sight, drones can form a vertical relay stack (each at a different
   altitude) to bridge the gap, effectively creating a 3D Steiner tree in
   $\mathbb{R}^3$ rather than $\mathbb{R}^2$.

---

## Problem Definition

**Setup**: Same $2000 \times 2000$ m earthquake-affected urban area as S051.
Additionally, a synthetic building-debris height map $h(\mathbf{c}_u) \in [0, 30]$ m
is defined over the same $40 \times 40$ grid; cells with $h(\mathbf{c}_u) \geq 10$ m
are classified as **dense debris** and obstruct ground-level line-of-sight.
Each drone $k$ now occupies a full 3D position
$\mathbf{p}_{d_k} = (x_k, y_k, z_k) \in \mathbb{R}^3$, with altitude
$z_k \in [z_{min}, z_{max}] = [20, 120]$ m individually optimised.

**Roles**:
- **Charge station** (base): fixed 3D point
  $\mathbf{p}_{base} = (1000, 100, 0)$ m at ground level.
- **Drones** ($K = 5$): each drone $k$ hovers at $\mathbf{p}_{d_k}$ with altitude
  $z_k$ chosen by the optimiser. Ground coverage is determined by slant-range link
  budget, not a fixed horizontal radius.
- **Population grid** ($G = 40 \times 40$ cells): each cell $u$ has ground centroid
  $\mathbf{c}_u = (x_u, y_u, 0)$, population weight $\text{pop}_u$, and debris
  height $h_u$.
- **Debris obstacles**: voxel columns from $z = 0$ to $z = h_u$ at each dense-debris
  cell; any link whose ray intersects a debris column is considered blocked.

**Objective**: Jointly optimise the $K$ drone 3D positions
$\{\mathbf{p}_{d_1}, \ldots, \mathbf{p}_{d_K}\}$ to **maximise total covered
population** subject to:

1. **3D connectivity** — the relay graph (including base station at $z = 0$) is
   connected under line-of-sight link budget at drone-to-drone slant range.
2. **Battery budget** — 3D transit distance (including altitude change) and hover
   power at altitude $z_k$ must satisfy the energy budget $E_{max}$.
3. **LOS clearance** — each drone-to-ground link must clear all debris columns along
   the slant ray.
4. **Altitude bounds** — $z_k \in [20, 120]$ m for all $k$.

**Comparison strategies**:
1. **Flat 3D (baseline)** — reuse S051 SA solution, lift all drones to $z = 50$ m;
   evaluate with 3D link budget and debris obstruction (shows the cost of ignoring
   altitude).
2. **Altitude-grid SA** — extend the S051 SA state to include $z_k$ per drone;
   perturb both horizontal position and altitude at each SA step.
3. **Vertical daisy-chain** — heuristic: place drones in a vertical stack above the
   densest debris patch to relay from ground to the horizontal fleet; altitudes
   assigned by the relay chain geometry.

---

## Mathematical Model

### 3D Slant-Range Coverage (Link Budget)

True slant distance from drone $k$ to ground cell centroid $u$:

$$d_{3D}(k, u) = \sqrt{\|\mathbf{c}_u^{xy} - \mathbf{p}_{d_k}^{xy}\|^2 + z_k^2}$$

Free-space path loss at carrier frequency $f_c = 900$ MHz:

$$\text{FSPL}(d) = \left(\frac{4\pi f_c d}{c}\right)^2$$

A cell $u$ is covered by drone $k$ if the received power exceeds the sensitivity
threshold $P_{min}$:

$$P_{rx}(k, u) = \frac{P_{tx} \cdot G_{tx} \cdot G_{rx}}{\text{FSPL}(d_{3D}(k,u))} \geq P_{min}$$

Equivalently, define the maximum coverage slant range:

$$d_{cov} = \frac{c}{4\pi f_c}\sqrt{\frac{P_{tx} G_{tx} G_{rx}}{P_{min}}}$$

Coverage indicator accounting for LOS clearance:

$$I_{3D}(u, k) = \begin{cases}
1 & \text{if } d_{3D}(k,u) \leq d_{cov} \text{ and } \text{LOS}(k, u) = \text{clear}\\
0 & \text{otherwise}
\end{cases}$$

Total covered population (objective):

$$C_{3D}\!\left(\{\mathbf{p}_{d_k}\}\right) = \sum_{u \in \mathcal{U}} \text{pop}_u \cdot \mathbf{1}\!\left[\max_{k \in \mathcal{D}} I_{3D}(u, k) \geq 1\right]$$

### 3D Debris LOS Check

For a ray from drone $\mathbf{p}_{d_k} = (x_k, y_k, z_k)$ to cell centroid
$(x_u, y_u, 0)$, parameterise as:

$$\mathbf{r}(\tau) = (1-\tau)\,\mathbf{p}_{d_k} + \tau\,(x_u, y_u, 0), \quad \tau \in [0, 1]$$

The ray is blocked if there exists a debris cell $v$ with height $h_v$ such that the
ray passes through the vertical column over cell $v$:

$$\exists\,\tau^* : \mathbf{r}^{xy}(\tau^*) \in \text{footprint}(v)
\;\text{and}\; r_z(\tau^*) \leq h_v$$

In discrete form, sample the ray at $N_{ray} = 50$ points and flag any sample
within a debris column:

$$\text{LOS}(k, u) = \text{clear} \iff \forall\, n \in \{0,\ldots,N_{ray}\}:\; r_z\!\left(\tfrac{n}{N_{ray}}\right) > h\!\left(\mathbf{r}^{xy}\!\left(\tfrac{n}{N_{ray}}\right)\right)$$

### 3D Connectivity via Link Budget

Build directed relay graph $\mathcal{G}_{3D} = (\mathcal{V}, \mathcal{E}_{3D})$ where
edge $(i, j)$ exists if:

$$d_{3D}^{relay}(i,j) = \|\mathbf{p}_i - \mathbf{p}_j\| \leq d_{relay}$$

and the ray between $\mathbf{p}_i$ and $\mathbf{p}_j$ clears all debris columns.
Relay link budget maximum distance:

$$d_{relay} = \frac{c}{4\pi f_c}\sqrt{\frac{P_{tx} G_{tx} G_{rx}}{P_{relay,min}}}$$

Algebraic connectivity requirement:

$$\lambda_2(\mathbf{L}_{3D}) > 0$$

Connectivity penalty in the SA cost function:

$$\text{pen}_{3D}(\mathcal{G}_{3D}) = \alpha \cdot \bigl(K + 1 - |\text{nodes in largest connected component containing node 0}|\bigr)$$

### 3D Battery / Energy Budget

Hover power scales with altitude due to reduced air density; using a simplified
ISA model at low altitude:

$$P_{hover}(z_k) = P_{hover,0}\left(1 + \kappa \cdot z_k\right)$$

where $\kappa \approx 4 \times 10^{-5}$ m$^{-1}$ captures the density effect.

3D transit distance from base to hover position:

$$L_k = \sqrt{\|\mathbf{p}_{d_k}^{xy} - \mathbf{p}_{base}^{xy}\|^2 + z_k^2}$$

Total energy constraint:

$$\frac{2 L_k}{v} \cdot P_{transit} + T_{hover} \cdot P_{hover}(z_k) \leq E_{max}$$

Rearranging for maximum hover duration at altitude $z_k$:

$$T_{hover,max}(k) = \frac{E_{max} - 2 L_k P_{transit} / v}{P_{hover}(z_k)}$$

A position $\mathbf{p}_{d_k}$ with altitude $z_k$ is infeasible if
$T_{hover,max}(k) < T_{hover,min}$.

### 3D Simulated Annealing

State vector extended to 3D positions:

$$\mathbf{x} = \bigl[\mathbf{p}_{d_1}^\top,\, \ldots,\, \mathbf{p}_{d_K}^\top\bigr]^\top \in \mathbb{R}^{3K}$$

3D cost function (minimise):

$$E_{3D}(\mathbf{x}) = -C_{3D}(\mathbf{x}) + \text{pen}_{3D}(\mathcal{G}_{3D}(\mathbf{x})) + \text{pen}_{energy}(\mathbf{x})$$

Perturbation: randomly select drone $k$, apply displacement

$$\Delta\mathbf{p} = \begin{bmatrix}\Delta x \\ \Delta y \\ \Delta z\end{bmatrix} \sim \mathcal{U}\!\left([-\delta_{xy}, \delta_{xy}]^2 \times [-\delta_z, \delta_z]\right), \quad \delta_{xy} = 200\,\text{m},\; \delta_z = 20\,\text{m}$$

Altitude clipped to $[z_{min}, z_{max}]$ after each step.

Acceptance and cooling schedule unchanged from S051:

$$P_{accept} = \min\!\left(1,\,\exp\!\left(-\frac{E_{3D}(\mathbf{x}') - E_{3D}(\mathbf{x})}{T_{SA}}\right)\right), \qquad T_{SA}^{(n+1)} = 0.995\cdot T_{SA}^{(n)}$$

### Vertical Daisy-Chain Topology

Identify the densest-debris patch centroid $\mathbf{c}^*$ (cell with maximum $h_u$).
Place a vertical stack of $M \leq K$ drones at $(x^*, y^*, z_m)$ with:

$$z_m = h_{max} + m \cdot \Delta z_{stack}, \quad m = 1, \ldots, M, \quad \Delta z_{stack} = \frac{z_{max} - h_{max}}{M}$$

Each stacked drone acts purely as a relay node (contributes no ground coverage).
Remaining $K - M$ drones are placed by greedy coverage above the unobstructed area.
The optimal split $M^*$ minimises the coverage loss from the relay drones:

$$M^* = \arg\min_{M \in \{0,\ldots,K-1\}} \text{coverage loss}(M) \;\text{s.t.}\; \lambda_2(\mathbf{L}_{3D}) > 0$$

---

## Key 3D Additions

- **Altitude optimisation**: each drone's $z_k$ is a free variable in $[20, 120]$ m,
  jointly optimised with its horizontal position in the SA state vector.
- **3D link budget coverage**: coverage radius is derived from FSPL and $d_{cov}$
  (slant range), replacing the fixed $r_{cov} = 300$ m horizontal circle.
- **Debris LOS ray check**: 50-sample ray marching against a 3D voxel height map
  before each coverage or relay link is accepted.
- **Altitude-dependent hover power**: $P_{hover}(z_k) = P_{hover,0}(1 + \kappa z_k)$
  replaces the flat energy model; higher drones cost more power.
- **Vertical daisy-chain heuristic**: $M$ relay-only drones form a vertical stack
  above the densest debris patch to reconnect line-of-sight-blocked segments.
- **3D trajectory visualisation**: matplotlib 3D scatter with altitude colourbar,
  plus relay edge lines drawn in full 3D; separate altitude vs drone-index bar chart.

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Drone fleet size $K$ | 5 |
| Area | 2000 x 2000 m |
| Population grid resolution | 40 x 40 cells |
| Altitude range $[z_{min}, z_{max}]$ | 20 – 120 m |
| Carrier frequency $f_c$ | 900 MHz |
| Transmit power $P_{tx}$ | 1 W |
| Antenna gains $G_{tx} = G_{rx}$ | 3 dBi each |
| Receiver sensitivity $P_{min}$ | -100 dBm |
| Max coverage slant range $d_{cov}$ | ~450 m (derived) |
| Relay link max range $d_{relay}$ | ~900 m (derived) |
| Debris height range $h_u$ | 0 – 30 m |
| Dense-debris threshold | $h_u \geq 10$ m |
| LOS ray samples $N_{ray}$ | 50 |
| Hover power at sea level $P_{hover,0}$ | 80 W |
| Altitude power coefficient $\kappa$ | $4 \times 10^{-5}$ m$^{-1}$ |
| Transit speed $v$ | 15 m/s |
| Energy budget $E_{max}$ | 144 000 J (equivalent to S051 battery) |
| Connectivity penalty weight $\alpha$ | $10^6$ |
| SA initial temperature $T_0$ | 500 |
| SA cooling rate $\beta$ | 0.995 |
| SA iterations | 80 000 |
| xy perturbation $\delta_{xy}$ | 200 m |
| z perturbation $\delta_z$ | 20 m |
| Charge station $\mathbf{p}_{base}$ | (1000, 100, 0) m |

---

## Expected Output

- **3D scatter plot**: drone hover positions as coloured spheres in 3D space with
  altitude colourbar; relay edges drawn as lines between connected nodes including
  base; debris voxel columns rendered as semi-transparent grey pillars.
- **Population heatmap with coverage footprints**: top-down 2D heatmap overlaid with
  per-drone ground footprint ellipses (footprint is no longer a perfect circle due to
  off-nadir slant coverage and debris shadowing).
- **Altitude per drone bar chart**: $z_k$ for each drone under flat-50 m baseline vs
  altitude-SA strategy; illustrates optimiser lifting high-load relay drones higher.
- **Coverage fraction comparison bar chart**: covered population percentage for flat
  baseline, altitude-SA, and vertical daisy-chain strategies.
- **Debris LOS shadow map**: 2D top-down map shaded by the fraction of drones from
  which each ground cell has clear LOS; dark patches mark debris-shadowed zones.
- **SA convergence curve**: 3D cost $E_{3D}$ vs iteration; connectivity penalty
  trace and energy penalty trace overlaid to show when both constraints are
  satisfied simultaneously.
- **Altitude vs time animation (GIF)**: SA optimisation progress rendered in 3D —
  drone spheres translate and rise/fall as the algorithm converges; relay edges
  update each frame.

---

## Extensions

1. **Terrain-adaptive altitude floor**: replace the flat $z_{min} = 20$ m with a
   per-column floor $z_{min,k} = h(\mathbf{p}_{d_k}^{xy}) + 15$ m, so drones
   automatically clear local debris regardless of horizontal position.
2. **Multi-hop bandwidth model**: assign throughput $B_{link} \propto d_{3D}^{-2}$
   to each relay edge; add a minimum bottleneck bandwidth constraint on every root-to-drone
   path in the daisy-chain tree.
3. **Wind-field altitude trade-off**: introduce an altitude-dependent wind model
   $\mathbf{w}(z) = w_0 (z/z_{ref})^{0.14} \hat{\mathbf{x}}$; drones hovering
   higher face stronger headwinds and increased hover power; re-optimise altitude
   balancing coverage gain against wind-power cost.
4. **Dynamic debris collapse**: debris height map evolves over time (aftershock
   collapses); drones must replan positions online when a debris cell height changes;
   evaluate replanning latency and coverage gap.
5. **RL altitude policy**: train a PPO agent with the 3D population and debris maps
   as a 3-channel CNN input; action space includes $(x, y, z)$ placement for each
   drone sequentially; compare against SA in coverage quality and computation time.
6. **Physical antenna tilt**: model drone antenna as a directional pattern with
   half-power beam width; tilt angle per drone becomes an additional optimisation
   variable alongside $z_k$ to steer gain toward the densest population cluster.

---

## Related Scenarios

- Original 2D version: [S051](../S051_post_disaster_comm.md)
- Truly 3D references: [S001](../../01_pursuit_evasion/S001_basic_intercept.md), [S003](../../01_pursuit_evasion/S003_low_altitude_tracking.md)
- Signal relay geometry: [S047 Signal Relay Enhancement](../S047_signal_relay.md), [S015 Communication Relay Tracking](../../01_pursuit_evasion/S015_comm_relay_tracking.md)
- Debris-aware path planning cross-reference: [S043 Confined Space Inspection](../S043_confined_space.md)
- Multi-drone coverage follow-up: [S052 Glacier Area Monitoring](../S052_glacier.md)
