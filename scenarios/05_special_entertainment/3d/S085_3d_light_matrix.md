# S085 3D Upgrade — Light Matrix Positioning

**Domain**: Special Ops & Entertainment | **Difficulty**: ⭐⭐⭐ | **Status**: `[ ]` Not Started
**Based on**: [S085 original](../S085_light_matrix.md)

---

## What Changes in 3D

The original S085 constrains all 25 target grid points to a single flat plane at the fixed show
altitude $z_{show} = 50$ m (`z_arr = np.full(n * n, z)`). Every drone's final position therefore
differs only in $x$ and $y$; the $z$-coordinate is identical across the entire formation. In true
3D, a volumetric light display requires a full $N \times M \times L$ grid where targets are spread
across multiple depth layers, enabling 3D shape rendering (spheres, cubes, extruded letters) that
are visible from any viewing angle. This variant adds a third grid dimension, a layer-aware
Hungarian assignment cost that accounts for the true 3D cost of reaching each voxel, and an
inter-layer collision avoidance term that prevents drones crossing through occupied depth layers
during flight.

---

## Problem Definition

**Setup**: A swarm of $N = 27$ drones must fill a $3 \times 3 \times 3$ volumetric voxel grid
centred at $(0, 0, z_{show})$ with $z_{show} = 30$ m. Voxel spacing is $d_{xy} = 2.0$ m in the
$x$–$y$ plane and $d_z = 3.0$ m between depth layers, producing a formation that spans
$4 \times 4$ m horizontally and $6$ m vertically. Drones begin from a $12 \times 12 \times 6$ m
staging volume at ground level ($z \in [1, 6]$ m). Three target shapes are supported: a **solid
cube** (all 27 voxels occupied), a **hollow sphere** (only voxels whose centres lie on the
surface of the inscribed sphere), and a **3D letter** (a voxelised character from a 3-layer
bitmap font). For shapes that require fewer than 27 drones, unassigned drones hold position in a
reserve layer below the formation.

**Roles**:
- **Drones** ($N = 27$): identical quadrotors with RGB LED payloads. A single pre-flight
  Hungarian assignment minimises total 3D Euclidean travel cost from initial positions to 3D voxel
  targets.
- **Voxel targets**: up to 27 fixed 3D points $\mathbf{g}_{ijl}$ arranged in a
  $3 \times 3 \times 3$ lattice; indices $i, j \in \{0,1,2\}$ (x–y plane), $l \in \{0,1,2\}$
  (depth layer). Layer $l = 0$ is the front face visible to the audience; layer $l = 2$ is the
  back face.

**Safety constraints**:
- Pairwise ORCA-lite repulsion activates when $d_{ij} < r_{safe} = 0.8$ m (same as S085).
- An additional **layer-separation constraint** prevents drones assigned to different depth layers
  from simultaneously occupying the same $x$–$y$ column: if two drones share a column
  $(i, j)$ but different layers $l$, their $z$ commands are offset by at least $\Delta z_{min}
  = 1.5$ m during the transit phase.
- Altitude bounds: $z \in [0.5, z_{show} + d_z + 2]$ m.

**Objective**: minimise volumetric convergence time $T_{conv,3D}$ — the first instant at which
every active drone satisfies $\|\mathbf{p}_k - \mathbf{g}_k\| \leq \varepsilon_{conv} = 0.15$ m —
while keeping total collision count at zero and mean 3D formation error $\varepsilon_{f,3D}$ below
0.05 m after convergence.

---

## Mathematical Model

### 3D Voxel Grid Targets

The $3 \times 3 \times 3$ volumetric grid with x–y spacing $d_{xy}$ and z spacing $d_z$,
centred at $(0, 0, z_{show})$, gives voxel target:

$$\mathbf{g}_{ijl} = \begin{bmatrix}
  (i - 1)\, d_{xy} \\
  (j - 1)\, d_{xy} \\
  z_{show} + (l - 1)\, d_z
\end{bmatrix},
\qquad i, j, l \in \{0, 1, 2\}$$

The total set $\mathcal{G} = \{\mathbf{g}_{ijl}\}$ contains up to 27 points. For shape rendering,
only the subset $\mathcal{G}_{shape} \subseteq \mathcal{G}$ of occupied voxels is used as active
targets; the remaining $|\mathcal{G}| - |\mathcal{G}_{shape}|$ drones are assigned to a reserve
hover layer at $z_{reserve} = z_{show} - d_z - 2$ m.

### Shape Rendering: Hollow Sphere

For the inscribed sphere of radius $R_s = d_{xy}$, voxel $(i, j, l)$ is active if:

$$\left|(i-1)^2 d_{xy}^2 + (j-1)^2 d_{xy}^2 + (l-1)^2 d_z^2 - R_s^2\right| \leq \delta_{shell}$$

where $\delta_{shell}$ is the shell thickness tolerance (typically $0.6\,d_{xy}$). Voxels inside
the sphere are dark; only surface voxels carry active LEDs.

### 3D Hungarian Assignment

Construct the $N \times |\mathcal{G}_{active}|$ cost matrix using full 3D Euclidean distance:

$$C_{k,m} = \|\mathbf{p}_k(0) - \mathbf{g}_m\|_2,
  \qquad k \in \{1,\ldots,N\},\; \mathbf{g}_m \in \mathcal{G}_{active}$$

The optimal bijection $\sigma^*$ solves:

$$\min_{\sigma} \sum_{k=1}^{N} C_{k,\sigma(k)}$$

via `scipy.optimize.linear_sum_assignment`. For non-square problems (shape with fewer active
voxels than drones), pad $\mathcal{G}_{active}$ with reserve hover positions to make the matrix
square before solving.

### Layer-Separation Altitude Command

For two drones $a$ and $b$ assigned to the same column $(i, j)$ but layers $l_a < l_b$, the
altitude command of drone $b$ is augmented during transit ($t < T_{layer,sep}$) to enforce
vertical clearance:

$$z_{cmd,b}(t) = \max\bigl(z_{cmd,b}^{PID}(t),\; z_a(t) + \Delta z_{min}\bigr)$$

where $z_a(t)$ is drone $a$'s current altitude and $\Delta z_{min} = 1.5$ m. This prevents
column-mate collisions when both drones are climbing through the same $x$–$y$ footprint.

### PID Position Controller (per drone, 3D)

Identical to S085 but operating on full 3D error $\mathbf{e}_k(t) = \mathbf{g}_k - \mathbf{p}_k(t) \in \mathbb{R}^3$:

$$\mathbf{u}_k(t) = K_p\,\mathbf{e}_k(t)
  + K_i \sum_{\tau=0}^{t} \mathbf{e}_k(\tau)\,\Delta t
  + K_d\,\frac{\mathbf{e}_k(t) - \mathbf{e}_k(t-\Delta t)}{\Delta t}$$

The $z$-axis gain may be tuned independently ($K_{p,z}$ larger than $K_{p,xy}$) to overcome
gravity bias and reach higher layers faster.

### ORCA-lite Repulsion (3D)

Unchanged from S085 but now operates in $\mathbb{R}^3$. The unit separation vector is:

$$\hat{\mathbf{n}}_{ij} = \frac{\mathbf{p}_i(t) - \mathbf{p}_j(t)}{\|\mathbf{p}_i(t) - \mathbf{p}_j(t)\|}
  \in \mathbb{R}^3$$

The repulsive force magnitude and activation range are the same:

$$\mathbf{F}_{rep,i}^{(j)} = k_{rep}
  \left(\frac{1}{d_{ij}} - \frac{1}{r_0}\right)\frac{1}{d_{ij}^2}\,\hat{\mathbf{n}}_{ij},
  \qquad d_{ij} < r_0 = r_{safe}$$

### 3D Volumetric Convergence Metrics

**Convergence time**:

$$T_{conv,3D} = \min\bigl\{t : \max_{k \in \mathcal{D}_{active}} \|\mathbf{p}_k(t) - \mathbf{g}_k\|
  \leq \varepsilon_{conv}\bigr\}$$

**Mean 3D formation error** (at and after $T_{conv,3D}$):

$$\varepsilon_{f,3D} = \frac{1}{|\mathcal{D}_{active}|}
  \sum_{k \in \mathcal{D}_{active}} \|\mathbf{p}_k - \mathbf{g}_k\|$$

**Per-layer formation error** (layer $l$):

$$\varepsilon_{f,l} = \frac{1}{|\mathcal{D}_l|}
  \sum_{k \in \mathcal{D}_l} \|\mathbf{p}_k - \mathbf{g}_k\|,
  \quad l \in \{0, 1, 2\}$$

where $\mathcal{D}_l$ is the set of drones assigned to layer $l$. This metric reveals whether
front-layer or back-layer drones converge later (due to longer altitude travel).

### Assignment Cost Reduction

Identical to S085 — compare the 3D Hungarian cost $D_{assign,3D}$ against $M_{trials} = 200$
random permutations:

$$\eta_{assign,3D} = 1 - \frac{D_{assign,3D}}{\bar{D}_{rand,3D}}$$

A higher $\eta_{assign,3D}$ is expected compared to the 2D case because the z-dimension creates
a larger spread in inter-drone distances, giving the Hungarian algorithm more room to optimise.

---

## Key 3D Additions

- **Volumetric grid**: 3D voxel lattice $3 \times 3 \times 3$ with independent $d_{xy}$ and $d_z$
  spacing, replacing the flat $5 \times 5$ plane.
- **Shape rendering**: solid cube, hollow sphere, and voxelised 3D letter via occupancy mask
  applied to $\mathcal{G}$.
- **3D Hungarian assignment**: cost matrix built from full 3D Euclidean distances; non-square
  extension handles partial-occupancy shapes with reserve hover positions.
- **Layer-separation altitude command**: per-column $z$ offset enforcement during transit to
  prevent column-mate collisions.
- **Depth-stratified convergence**: per-layer $\varepsilon_{f,l}$ tracks whether deeper layers
  are harder to fill, exposing the $d_z$ sensitivity.
- **3D ORCA-lite**: repulsion vectors are fully 3D; altitude component of repulsion is critical
  when drones from different layers share a vertical column.
- **Altitude bounds**: $z \in [0.5,\; z_{show} + d_z + 2]$ m replaces the implicit single-height
  constraint of S085.

---

## Key Parameters

| Parameter | Symbol | Value |
|-----------|--------|-------|
| Number of drones | $N$ | 27 |
| Grid dimensions | — | $3 \times 3 \times 3$ |
| x–y voxel spacing | $d_{xy}$ | 2.0 m |
| z (depth) voxel spacing | $d_z$ | 3.0 m |
| Show centre altitude | $z_{show}$ | 30 m |
| Reserve hover altitude | $z_{reserve}$ | $z_{show} - d_z - 2$ m |
| Altitude bounds | — | [0.5, 37] m |
| Staging volume | — | $12 \times 12 \times 6$ m |
| Collision radius | $r_{collision}$ | 0.5 m |
| Repulsion activation radius | $r_{safe}$ | 0.8 m |
| Repulsion gain | $k_{rep}$ | 1.5 |
| Layer-separation clearance | $\Delta z_{min}$ | 1.5 m |
| Max velocity | $v_{max}$ | 5.0 m/s |
| Convergence threshold | $\varepsilon_{conv}$ | 0.15 m |
| PID gains $(K_p, K_i, K_d)$ | — | (1.2, 0.05, 0.4) |
| z-axis PID gain | $K_{p,z}$ | 1.5 |
| Simulation timestep | $\Delta t$ | 0.05 s |
| Safety cutoff | $T_{max}$ | 80 s |
| Assignment algorithm | — | Hungarian (Kuhn–Munkres), 3D cost |
| Random seed | — | 42 |

---

## Expected Output

- **3D voxel formation plot** (`s085_3d_formation.png`): two-panel figure. Left: 3D trajectory
  plot for all 27 drones from staging volume to volumetric voxel grid; each drone coloured by
  plasma colormap; target voxels shown as coloured cubes (front layer = cyan, mid layer = yellow,
  back layer = magenta). Right: three top-down sub-panels (one per depth layer) showing per-layer
  final drone positions vs target voxels with residual error annotated.
- **Convergence metrics figure** (`s085_3d_convergence.png`): three-panel figure. Top: max and
  mean 3D formation error vs time with $T_{conv,3D}$ marked. Middle: per-layer mean error
  $\varepsilon_{f,l}(t)$ showing which layers converge last. Bottom: per-drone final 3D error bar
  chart colour-coded by assigned layer.
- **Shape comparison figure** (`s085_3d_shapes.png`): side-by-side 3D scatter plots of final
  drone positions for (a) solid cube, (b) hollow sphere, and (c) 3D letter; drone colours encode
  LED colour assigned per layer; reserve drones shown as grey markers below the main formation.
- **Assignment improvement figure** (`s085_3d_assignment.png`): histogram of 3D random
  assignment costs (200 trials) vs 3D Hungarian cost; cost reduction $\eta_{assign,3D}$ annotated;
  subplot comparing the 2D vs 3D improvement ratio to quantify the value of depth information.
- **Animation** (`s085_3d_light_matrix.gif`): 3D animation showing 27 drones ascending from the
  staging volume and filling the volumetric voxel grid layer by layer; depth layers distinguished
  by marker colour; time, max 3D error, and active layer count annotated per frame.
- **Console metrics**:
  - Volumetric convergence time $T_{conv,3D}$ (s)
  - Mean 3D formation error $\varepsilon_{f,3D}$ (m)
  - Per-layer formation errors $\varepsilon_{f,0}$, $\varepsilon_{f,1}$, $\varepsilon_{f,2}$ (m)
  - Total collision count (target: 0)
  - 3D assignment cost reduction $\eta_{assign,3D}$

---

## Extensions

1. **Dynamic shape morphing in 3D**: after converging to the hollow sphere, re-solve a 3D
   Hungarian assignment to a new shape (e.g., a heart or star polyhedron); measure 3D
   inter-shape travel cost and compare against the 2D morphing scenario S088.
2. **Viewer-direction adaptive rendering**: given a known audience azimuth angle $\phi_{audience}$,
   rotate the voxel grid so the densest face of the 3D shape is orthogonal to the viewing
   direction; study how grid rotation angle affects per-drone travel distance.
3. **Layer-prioritised ascent**: rather than letting all drones climb simultaneously, sequence
   ascent by layer (back layer first, then mid, then front) to eliminate column-mate conflicts
   entirely; compare $T_{conv,3D}$ against the simultaneous climb with layer-separation enforcement.
4. **Drone failure recovery in 3D**: if one drone drops out in mid-ascent, re-solve the 3D
   Hungarian on the remaining $N-1$ drones; quantify re-convergence overhead and whether the
   dropped voxel is visible to the audience based on its $(i, j, l)$ index.
5. **Energy-aware depth assignment**: model climbing cost as proportional to $\Delta z$ and assign
   drones to layers that minimise total energy $E = \sum_k \bigl(\Delta x_k^2 + \Delta y_k^2
   + w_z \Delta z_k^2\bigr)$ for weight $w_z > 1$; study the trade-off between formation
   symmetry and energy efficiency as $w_z$ increases.
6. **Voxel resolution sweep**: vary grid dimensions from $2 \times 2 \times 2$ (8 drones) through
   $4 \times 4 \times 4$ (64 drones); measure $T_{conv,3D}$, collision rate, and $\eta_{assign,3D}$
   as functions of swarm size to characterise how volumetric assignment scales with $N$.

---

## Related Scenarios

- Original 2D (flat grid) version: [S085](../S085_light_matrix.md)
- LED formation predecessor: [S083 LED Show Formation](../S083_light_show_single.md)
- 3D shape morphing follow-up: [S088 Formation Morphing](../S088_formation_morphing.md)
- Multi-agent assignment cross-reference: [S018 Multi-Target Interception](../../01_pursuit_evasion/S018_multi_target_interception.md)
- Agricultural swarm with Hungarian + PID: [S070 Swarm Weeding](../../04_industrial_agriculture/S070_swarm_weeding.md)
- 3D pursuit evasion reference for 3D ORCA: [S002 3D Upgrade](../../01_pursuit_evasion/3d/S002_3d_evasive_maneuver.md)
