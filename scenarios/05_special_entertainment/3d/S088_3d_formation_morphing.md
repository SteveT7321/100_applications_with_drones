# S088 3D Upgrade — Formation Shape Morphing

**Domain**: Special Ops & Entertainment | **Difficulty**: ⭐⭐⭐⭐ | **Status**: `[ ]` Not Started
**Based on**: [S088 original](../S088_formation_morphing.md)

---

## What Changes in 3D

The original S088 already operates in 3D space for drone positions, but its shape vocabulary
(sphere, cube, helix, star) keeps the vertical extent shallow: the helix spans only
$\pm 5.7$ m in z, and the star uses just two altitude layers at $z = \pm 1.2$ m. More
critically, the Hungarian assignment minimises raw Euclidean distance without considering
the **altitude crossing problem** — when many drones swap between upper and lower altitude
bands simultaneously, their straight-line trajectories cross in 3D space and force the
ORCA layer to resolve dozens of near-simultaneous conflicts.

This variant introduces:

1. **Expanded 3D shape vocabulary** — four structurally distinct shapes that make full use
   of the vertical axis: sphere, cube, star, and pyramid. Each shape occupies a significantly
   different altitude distribution, so every morph requires genuine vertical displacement.
2. **Altitude-layered Hungarian assignment** — at each transition the cost matrix is augmented
   with a vertical crossing penalty, steering the solver toward assignments that keep crossing
   paths in separate altitude bands and thereby reduce the ORCA conflict load.
3. **Layer-aware morph sequencing** — within each morph interval, drones are sorted into
   altitude layers and their interpolation is staggered by a small phase offset
   $\delta_{layer}$ so that the upper and lower groups complete their vertical separation
   before the lateral morph begins, reducing the probability of 3D path crossings.

---

## Problem Definition

**Setup**: A swarm of $N = 20$ identical quadrotors performs a choreographed aerial display
by morphing continuously through four 3D geometric shapes: **sphere → cube → star → pyramid**.
Each shape is defined by $N = 20$ keypoints with a nominal shape radius of $R = 4.0$ m and
nominal inter-point spacing $d_{form} = 2.0$ m. The mission consists of three consecutive
transitions, each lasting $T_{morph} = 12$ s, giving a total mission time of $T_{total} = 36$ s.
The extended morph time (vs. 10 s in S088) accommodates the greater vertical displacements
introduced by the pyramid apex and the cube's distributed altitude range.

**Roles**:
- **Drones** ($N = 20$): homogeneous quadrotors; each tracks a smooth-step-interpolated
  target position while running a 3D ORCA velocity filter against all neighbours within
  sensing radius $r_{sense} = 4$ m.
- **Shape library**: four canonical 3D point clouds each normalised to mean inter-point
  spacing $d_{form}$; the pyramid replaces the helix to maximise altitude-distribution
  contrast across the sequence.
- **Altitude-layered assignment**: at each transition, drones and keypoints are first sorted
  into $L = 3$ altitude bands; within each band the standard Hungarian algorithm is applied;
  a vertical crossing penalty $\lambda_{vert} = 1.5$ is applied to cross-band assignments
  to discourage but not prohibit them when they are globally cheaper.

**Objective**: Complete the full transition sequence sphere → cube → star → pyramid while
satisfying:

1. **Zero collision constraint**: $\|\mathbf{p}_i(t) - \mathbf{p}_j(t)\| \geq 2 r_{coll} = 1.0$ m
   $\forall\, i \neq j, \forall\, t$.
2. **Formation accuracy**: mean formation error $\bar{\varepsilon}_f < 0.3$ m at the end
   of each morph interval.
3. **Speed constraint**: $\|\mathbf{v}_k(t)\| \leq v_{max} = 3.0$ m/s for all drones and
   all timesteps.
4. **Altitude bounds**: all drones remain within $z \in [0.5, 8.0]$ m throughout the mission.

**Comparison conditions**:

1. **Layered assignment + ORCA** (proposed): altitude-layered Hungarian assignment with
   vertical crossing penalty; ORCA collision avoidance; smooth-step interpolation.
2. **Flat assignment + ORCA**: standard single-level Hungarian assignment (no altitude
   layering); ORCA active; same interpolation. Baseline that shows the benefit of layering.
3. **Layered assignment, no ORCA**: altitude-layered assignment, linear interpolation, no
   collision avoidance layer. Shows the residual conflict load even with good assignment.

---

## Mathematical Model

### Shape Keypoint Generation

All shapes are centred at the origin with nominal bounding radius $R = 4.0$ m. The altitude
range of each shape is chosen to maximise contrast across the morphing sequence.

**Sphere** — Fibonacci-sphere sampling (unchanged from S088):

$$\mathbf{p}_k^{sphere} = R \begin{pmatrix} \sin\theta_k \cos\phi_k \\ \sin\theta_k \sin\phi_k \\ \cos\theta_k \end{pmatrix}, \quad \theta_k = \arccos\!\left(1 - \frac{2k}{N-1}\right), \quad \phi_k = \frac{2\pi k}{\varphi}$$

where $\varphi = (1 + \sqrt{5})/2$. Altitude range: $z \in [-4, +4]$ m.

**Cube** — $N = 20$ points distributed uniformly across the six faces of a cube with
half-side $a = 3.0$ m; 3–4 points per face on a regular sub-grid. Altitude range:
$z \in [-3, +3]$ m.

$$\mathbf{p}_k^{cube} \in \{\pm a\} \times [-a, a]^2 \;\cup\; [-a, a]^2 \times \{\pm a\}$$

**Star** — 10-pointed star with $N_l = N/2 = 10$ vertices per altitude layer at
$z \in \{-2.0, +2.0\}$ m. Outer radius $R_{out} = 4.5$ m, inner radius $R_{in} = 1.8$ m:

$$\mathbf{p}_k^{star} = \begin{pmatrix} R_k \cos(\pi k / 10) \\ R_k \sin(\pi k / 10) \\ z_{layer(k)} \end{pmatrix}, \quad R_k = \begin{cases} R_{out} & k \text{ even} \\ R_{in} & k \text{ odd} \end{cases}$$

The star's two-layer structure ($z = \pm 2$ m) creates a pronounced altitude bimodality
that contrasts with the sphere's continuous altitude distribution.

**Pyramid** — $N_{base} = 16$ points on a square base at $z = -2.5$ m in a $4 \times 4$
sub-grid with spacing $s = 2.0$ m, plus $N_{apex} = 4$ apex points clustered near
$(0, 0, +5.5)$ m with a small angular offset $\Delta\phi = \pi/2$:

$$\mathbf{p}_k^{base} = \begin{pmatrix} (k_x - 1.5)\,s \\ (k_y - 1.5)\,s \\ -2.5 \end{pmatrix}, \quad k_x, k_y \in \{0, 1, 2, 3\}$$

$$\mathbf{p}_m^{apex} = \begin{pmatrix} r_{apex} \cos(m \pi / 2) \\ r_{apex} \sin(m \pi / 2) \\ 5.5 \end{pmatrix}, \quad m \in \{0,1,2,3\}, \quad r_{apex} = 0.5 \text{ m}$$

The pyramid spans $z \in [-2.5, +5.5]$ m — the largest vertical range in the sequence —
making the star-to-pyramid transition the most altitude-intensive morph.

### Altitude-Layered Hungarian Assignment

At the start of transition $s$, let $\mathbf{P}^{curr} \in \mathbb{R}^{N \times 3}$ be
current drone positions and $\mathbf{P}^{tgt} \in \mathbb{R}^{N \times 3}$ be target
keypoints. Partition both sets into $L = 3$ altitude bands $B_1, B_2, B_3$ of roughly
equal size by sorting on the $z$-coordinate.

Define the penalised cost matrix $\tilde{\mathbf{C}} \in \mathbb{R}^{N \times N}$:

$$\tilde{C}_{ij} = \|\mathbf{p}_i^{curr} - \mathbf{p}_j^{tgt}\|_2 \cdot \Gamma_{ij}$$

where the altitude-band penalty factor is:

$$\Gamma_{ij} = \begin{cases} 1.0 & \text{if drone } i \text{ and keypoint } j \text{ are in the same altitude band} \\ \lambda_{vert} & \text{otherwise} \end{cases}$$

with $\lambda_{vert} = 1.5$. The Hungarian algorithm then solves:

$$\sigma^* = \arg\min_{\sigma \in S_N} \sum_{i=1}^{N} \tilde{C}_{i,\sigma(i)}$$

This steers — but does not force — same-band assignments, preserving global optimality
while reducing cross-band trajectories by approximately 30–40 % in practice.

### Smooth-Step Shape Interpolation

Drone $k$'s desired position at time $t$ within the morph interval $[t_s, t_s + T_{morph}]$
uses a cubic smooth-step function instead of linear interpolation to reduce peak velocities
at transition boundaries:

$$\tau(t) = 3u^2 - 2u^3, \qquad u = \frac{t - t_s}{T_{morph}} \in [0, 1]$$

$$\mathbf{p}_k^{des}(t) = \bigl(1 - \tau(t)\bigr)\,\mathbf{p}_k^A + \tau(t)\,\mathbf{p}_k^B$$

The instantaneous preferred velocity (derivative of the smooth-step interpolant):

$$\dot{\mathbf{p}}_k^{nom}(t) = 6u(1-u) \cdot \frac{\mathbf{p}_k^B - \mathbf{p}_k^A}{T_{morph}}$$

This satisfies $\dot{\mathbf{p}}_k^{nom}(t_s) = \dot{\mathbf{p}}_k^{nom}(t_s + T_{morph}) = \mathbf{0}$,
eliminating velocity discontinuities at shape-transition boundaries.

### Layer-Staggered Morph Phase Offset

Drones are divided into three altitude layers at the start of each morph. Layer $\ell \in \{0,1,2\}$
(sorted ascending by source $z$-coordinate) starts its interpolation with a small phase
offset $\delta_\ell$:

$$u_k(t) = \frac{t - t_s - \ell_k \cdot \delta_{layer}}{T_{morph} - L \cdot \delta_{layer}}$$

where $\ell_k \in \{0,1,2\}$ is drone $k$'s layer index, clamped to $[0, 1]$, and
$\delta_{layer} = 0.5$ s. Lower-altitude drones begin first; by the time upper-layer drones
start their lateral displacement, lower-layer drones have already partially cleared the
mid-altitude band, reducing the instantaneous density of crossing paths.

### 3D ORCA Velocity Obstacle (Full 3D)

The ORCA formulation operates in $\mathbb{R}^3$ throughout, matching S088. The velocity
obstacle for drone pair $(A, B)$ over time horizon $T_h$:

$$\mathrm{VO}_{A|B}^{T_h} = \left\{\mathbf{v} \in \mathbb{R}^3 : \exists\, t \in [0, T_h], \;\bigl\|\mathbf{p}_A + \mathbf{v}\,t - (\mathbf{p}_B + \mathbf{v}_B t)\bigr\| < 2 r_{coll}\right\}$$

Each drone solves the 3D linear programme:

$$\mathbf{v}_A^{new} = \arg\min_{\mathbf{v} \in \mathbb{R}^3} \;\bigl\|\mathbf{v} - \dot{\mathbf{p}}_A^{nom}\bigr\|^2$$

$$\text{subject to} \quad (\mathbf{v} - \mathbf{v}_{cand,j}) \cdot \hat{\mathbf{n}}_{AB_j} \geq 0 \;\; \forall\, j : \|\mathbf{p}_A - \mathbf{p}_j\| \leq r_{sense}$$

$$\|\mathbf{v}\| \leq v_{max}, \qquad v_z \in \left[-v_{z,max},\; v_{z,max}\right]$$

An additional hard constraint $v_z \in [-v_{z,max}, +v_{z,max}]$ with $v_{z,max} = 1.5$ m/s
limits vertical speed to stay within the altitude bounds and respect quadrotor climb-rate
limits that are tighter than horizontal speed limits.

### Performance Metrics

**Formation error** (identical to S088):

$$\varepsilon_f(t) = \frac{1}{N} \sum_{k=1}^{N} \bigl\|\mathbf{p}_k(t) - \mathbf{p}_k^{des}(t)\bigr\|$$

**Vertical spread** — standard deviation of drone altitudes at time $t$; tracks whether the
swarm is executing genuine 3D morphs:

$$\sigma_z(t) = \sqrt{\frac{1}{N} \sum_{k=1}^{N} \bigl(z_k(t) - \bar{z}(t)\bigr)^2}, \quad \bar{z}(t) = \frac{1}{N}\sum_{k=1}^N z_k(t)$$

**Altitude band crossing rate** — number of drone-pairs whose straight-line interpolation
paths cross an altitude band boundary in the same 0.05 s timestep; lower means fewer
simultaneous conflicts for ORCA to resolve:

$$\rho_{cross}(t) = \left|\left\{(i,j) : i < j,\; z_i(t)\,z_j(t) < 0,\; z_i(t-\Delta t)\,z_j(t-\Delta t) > 0\right\}\right|$$

**Morph smoothness** (RMS jerk proxy, identical to S088):

$$\mathcal{S} = \sqrt{\frac{1}{N \cdot T_{steps}} \sum_{k=1}^{N} \sum_{t} \bigl\|\Delta\mathbf{v}_k(t)\bigr\|^2}$$

---

## Key 3D Additions

- **Expanded altitude range**: pyramid apex at $z = +5.5$ m vs. star's maximum $z = +2.0$ m;
  total vertical excursion per drone can exceed 8 m in the star-to-pyramid transition.
- **Altitude-layered assignment**: vertical crossing penalty $\lambda_{vert} = 1.5$ in the
  cost matrix reduces cross-band assignments without forcing them.
- **Layer-staggered interpolation**: $\delta_{layer} = 0.5$ s phase offset per altitude layer
  creates sequential vertical separation before lateral displacement begins.
- **Smooth-step interpolation**: cubic ease-in/ease-out $\tau = 3u^2 - 2u^3$ removes
  velocity discontinuities at transition boundaries, reducing peak speed by ~35 % vs. linear.
- **Vertical speed cap**: $v_{z,max} = 1.5$ m/s in the ORCA LP enforces quadrotor climb-rate
  limits and keeps drones within altitude bounds.
- **Altitude bounds**: $z \in [0.5, 8.0]$ m enforced at every integration step.
- **Vertical spread metric** $\sigma_z(t)$: quantifies how well the swarm exploits the
  z-axis during each morph.

---

## Key Parameters

| Parameter | Symbol | Value |
|-----------|--------|-------|
| Number of drones | $N$ | 20 |
| Shape sequence | — | Sphere → Cube → Star → Pyramid |
| Morph time per transition | $T_{morph}$ | 12 s |
| Total mission time | $T_{total}$ | 36 s |
| Simulation timestep | $\Delta t$ | 0.05 s |
| Maximum horizontal speed | $v_{max}$ | 3.0 m/s |
| Maximum vertical speed | $v_{z,max}$ | 1.5 m/s |
| Physical collision radius | $r_{coll}$ | 0.5 m |
| Hard collision threshold | $2r_{coll}$ | 1.0 m |
| ORCA sensing radius | $r_{sense}$ | 4.0 m |
| ORCA time horizon | $T_h$ | 2.0 s |
| Nominal formation distance | $d_{form}$ | 2.0 m |
| Shape bounding radius | $R$ | 4.0 m |
| Cube half-side | $a$ | 3.0 m |
| Star outer / inner radius | $R_{out} / R_{in}$ | 4.5 m / 1.8 m |
| Star layer altitudes | $z_{star}$ | ±2.0 m |
| Pyramid base grid spacing | $s$ | 2.0 m |
| Pyramid base altitude | $z_{base}$ | −2.5 m |
| Pyramid apex altitude | $z_{apex}$ | +5.5 m |
| Pyramid apex cluster radius | $r_{apex}$ | 0.5 m |
| Altitude band penalty | $\lambda_{vert}$ | 1.5 |
| Number of altitude bands | $L$ | 3 |
| Layer phase offset | $\delta_{layer}$ | 0.5 s |
| Altitude bounds | $z \in$ | [0.5, 8.0] m |
| Formation error target | $\bar{\varepsilon}_f$ | < 0.3 m |

---

## Expected Output

- **3D trajectory plot** (`s088_3d_trajectories.png`): full 3D axes showing position trails
  for all 20 drones across the 36-second mission; four shape snapshots highlighted at morph
  endpoints with distinct marker symbols; axes in metres; vertical range clearly showing the
  pyramid apex at $z \approx 5.5$ m.
- **Altitude profile** (`s088_altitude_profile.png`): $z_k(t)$ for all 20 drones over time
  with morph interval shading; the star-to-pyramid transition should show a fan of ascending
  trajectories converging toward the apex; the sphere-to-cube transition should show a
  compression of vertical spread.
- **Vertical spread** (`s088_vertical_spread.png`): $\sigma_z(t)$ time series for all three
  conditions; confirms that the swarm genuinely varies altitude distribution at each morph and
  that the layered assignment does not suppress vertical spread.
- **Metrics panel** (`s088_metrics_panel.png`): 2 × 2 figure (formation error, minimum
  separation, cumulative collision events, mean nearest-neighbour distance) for all three
  conditions; morph interval shading; reference lines at 0.3 m and 1.0 m thresholds.
- **Morphing animation** (`s088_animation.gif`): 3D view at 20 fps; coloured dots with 2-second
  trails; title shows elapsed time and current transition; approximately 240 frames for the
  36-second mission; altitude variation clearly visible in the 3D perspective.
- **Terminal summary**: per-condition metrics — total collision events, final formation error,
  morph smoothness RMS, minimum inter-drone separation, and peak altitude band crossing rate.

**Typical results (layered assignment + ORCA)**:

| Metric | Expected Value |
|--------|----------------|
| Total collision events | 0 |
| Final formation error $\varepsilon_f$ | < 0.25 m |
| Morph smoothness $\mathcal{S}$ | < 0.04 m/s per step |
| Min inter-drone separation | > 1.0 m |
| Peak altitude band crossing rate $\rho_{cross}$ | < 5 pairs per timestep |

---

## Extensions

1. **Quintic interpolation for continuous acceleration**: replace the cubic smooth-step with
   a quintic polynomial $\tau = 6u^5 - 15u^4 + 10u^3$ that also zeroes the second derivative
   at transition boundaries; measure the reduction in RMS jerk $\mathcal{S}$ versus the cubic
   baseline.
2. **Wind disturbance in the vertical axis**: add a vertical gust model
   $w_z(t) = W_z \sin(2\pi f_{gust} t)$ with $W_z = 1.0$ m/s and $f_{gust} = 0.2$ Hz;
   implement a feedforward vertical compensation term in the preferred velocity; measure the
   increase in altitude spread $\sigma_z$ and formation error caused by the gust.
3. **Pyramid apex congestion analysis**: the four apex keypoints are tightly clustered at
   $r_{apex} = 0.5$ m; reduce ORCA time horizon $T_h$ and sensing radius $r_{sense}$ and
   measure the minimum achievable $r_{apex}$ before collision constraint violations emerge.
4. **Continuous five-shape loop**: extend the sequence to sphere → cube → star → pyramid →
   sphere (closed loop) and run for three full cycles; verify that the swarm returns to its
   initial configuration within $\varepsilon < 0.1$ m across all drones after each complete
   loop.
5. **$N = 40$ scalability**: double the swarm to 40 drones; regenerate all four shapes
   analytically at the larger $N$; profile the ORCA LP solve time per timestep and measure
   how $\rho_{cross}$ and total collision events scale; compare flat vs. layered assignment
   at the larger scale.
6. **Spectator viewpoint optimisation**: given a fixed ground-level spectator at position
   $\mathbf{q} = (20, 0, 0)$ m, optimise the shape orientations (yaw rotations) so that the
   projected silhouette seen from $\mathbf{q}$ maximises recognisability for each shape; keep
   the assignment and ORCA layers unchanged.

---

## Related Scenarios

- Original 2D version: [S088 Formation Shape Morphing](../S088_formation_morphing.md)
- Formation keeping foundation: [S005 Formation Keeping](../../01_pursuit_evasion/S005_formation_keeping.md)
- Multi-agent coordination: [S009 Multi-Pursuer Coordination](../../01_pursuit_evasion/S009_multi_pursuer.md)
- Cooperative 3D formation: [S066 Cooperative Crane](../../04_industrial_agriculture/S066_cooperative_crane.md)
- Related 3D upgrade reference: [S002 3D Evasive Maneuver](../../01_pursuit_evasion/3d/S002_3d_evasive_maneuver.md)
