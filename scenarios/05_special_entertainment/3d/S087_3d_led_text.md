# S087 3D Upgrade — LED Formation Text

**Domain**: Special Ops & Entertainment | **Difficulty**: ⭐⭐⭐ | **Status**: `[ ]` Not Started
**Based on**: [S087 original](../S087_led_text.md)

---

## What Changes in 3D

The original S087 fixes every drone at a single display altitude `z_show = 50.0 m` throughout the
simulation. Every target pixel position has an identical z-coordinate, making the formation
completely flat. The morph between "DRONE" and "SHOW" is therefore a 2D rearrangement executed at
constant altitude; depth is never used as a creative or structural dimension.

This 3D upgrade introduces:

1. **Extruded letter shapes** — each character is given a depth profile: stroke pixels are pushed
   forward or backward along the viewer axis (z) according to a per-stroke depth layer, turning flat
   bitmaps into volumetric glyphs visible from oblique angles.
2. **Depth-layer assignment per character stroke** — contour pixels, interior fill pixels, and serif
   accent pixels occupy distinct z-layers, so the word has a bas-relief silhouette when viewed from
   any azimuth.
3. **Transition animation between 3D words** — during the morph, each drone follows a 3D cubic
   Bezier arc rather than a straight line; the arc mid-point is offset in the z-direction to produce
   a wave-like depth sweep visible to spectators on the ground.
4. **Multi-viewer direction optimisation** — the formation is optimised to remain legible from
   multiple simultaneous ground viewpoints arranged at known azimuths, by rotating and scaling the
   extruded formation so that each viewer's 2D projection onto their own viewing plane reconstructs
   the intended word with maximum visual fidelity.

---

## Problem Definition

**Setup**: $N = 30$ quadrotors spell "DRONE" and then "SHOW" using the same $6 \times 5$ bitmap
font from S087, but each lit pixel is assigned a z-coordinate drawn from a discrete set of depth
layers $\mathcal{Z} = \{z_0, z_1, \ldots, z_{L-1}\}$ with $L = 3$ layers and inter-layer spacing
$\Delta z = 3.0$ m. The depth-layer index of pixel $(i, j)$ in character $c$ is determined by a
stroke classifier that labels each lit pixel as contour (layer 0, front), body (layer 1, mid), or
accent (layer 2, back).

**Roles**:
- **Drones** ($N = 30$): homogeneous quadrotors with decoupled PID position controllers. Each drone
  carries a single RGB LED and maintains its assigned 3D pixel position.
- **Font bitmap** ($6 \times 5$ per character, $L = 3$ depth layers): a three-channel binary tensor
  $\operatorname{bitmap}_c[i,j,\ell] \in \{0,1\}$ where channel $\ell$ encodes the depth layer of
  each lit pixel.
- **Viewer array** ($M$ ground viewers): known azimuths $\phi_1, \ldots, \phi_M$ from which the
  formation is simultaneously observed; each viewer sees a 2D projection of the 3D formation.
- **Assignment**: the Hungarian algorithm maps $N$ drones to $N$ target 3D positions minimising
  total 3D travel distance, identical in structure to S087 but now operating on 3D coordinates
  including the z-layer component.

**Objective**: Maximise **multi-view visual fidelity** $\bar{V}$ — the mean visual fidelity across
all $M$ viewer projections — across both words, while minimising total travel cost and morph time.

---

## Mathematical Model

### 3D Pixel Position Mapping with Depth Layers

For pixel $(i, j)$ in character $c$ assigned to depth layer $\ell$, the 3D world position is:

$$\mathbf{p}_{c,i,j,\ell} = \begin{bmatrix}
  \bigl(c(W + g) + j\bigr) \cdot d_{px} \\
  (H - 1 - i) \cdot d_{px} \\
  z_{show} + \ell \cdot \Delta z
\end{bmatrix}$$

where $d_{px} = 2.0$ m, $W = 5$, $H = 6$, $g = 2$, $z_{show} = 50$ m, $\Delta z = 3.0$ m, and
$\ell \in \{0, 1, 2\}$ is the depth-layer index.

### Stroke Depth-Layer Classifier

Each lit pixel $(i, j)$ in character $c$ is classified by its 4-connected neighbourhood in the
bitmap:

$$\ell(i,j,c) = \begin{cases}
  0 & \text{(contour)} \quad \text{if any 4-neighbour is unlit} \\
  2 & \text{(accent)} \quad \text{if all 8-neighbours are lit (interior fill)} \\
  1 & \text{(body)} \quad \text{otherwise}
\end{cases}$$

This produces an extrusion that pushes the visible outline to the foreground layer and recesses
solid interior regions, creating a bas-relief silhouette.

### Hungarian Assignment in 3D

The cost matrix is the Euclidean distance in $\mathbb{R}^3$:

$$C_{km} = \|\mathbf{q}_k - \mathbf{p}_m\|_2, \quad k,m \in \{1,\ldots,N\}$$

The Hungarian algorithm finds:

$$\sigma^* = \operatorname{argmin}_{\sigma} \sum_{k=1}^{N} C_{k,\sigma(k)}$$

During the "DRONE" $\to$ "SHOW" transition the same procedure is re-applied using the current 3D
hover positions (including z-layer) as start positions, so the solver exploits z-proximity to reduce
cost.

### Cubic Bezier Morph Arc

Instead of the straight-line interpolation from S087, each drone $k$ follows a cubic Bezier arc
between its current position $\mathbf{p}^A_k$ and its target $\mathbf{p}^B_k$:

$$\mathbf{q}_k(\tau) = (1-\tau)^3 \mathbf{p}^A_k
  + 3(1-\tau)^2\tau \mathbf{c}^A_k
  + 3(1-\tau)\tau^2 \mathbf{c}^B_k
  + \tau^3 \mathbf{p}^B_k, \quad \tau \in [0,1]$$

The control points are offset in the z-direction to produce the depth wave:

$$\mathbf{c}^A_k = \mathbf{p}^A_k + \begin{bmatrix}0\\0\\ h_{arc}\end{bmatrix}, \qquad
  \mathbf{c}^B_k = \mathbf{p}^B_k + \begin{bmatrix}0\\0\\ h_{arc}\end{bmatrix}$$

with arc height $h_{arc} = 6.0$ m. Drones assigned to different depth layers reach peak altitude at
staggered times $\tau_{peak,\ell} = 0.5 + (\ell - 1) \cdot 0.1$, creating a rolling depth-sweep
effect visible from the side.

### Multi-Viewer Projection Fidelity

For viewer $m$ at ground azimuth $\phi_m$, the projection of the 3D formation onto the viewer's
frontal plane (perpendicular to the horizontal line of sight) is:

$$\Pi_m(\mathbf{p}) = \begin{bmatrix}
  -\sin\phi_m & \cos\phi_m & 0 \\
  0           & 0          & 1
\end{bmatrix} \mathbf{p}$$

mapping each 3D position to a 2D point $(u, v)$ in the viewer's image plane. The fidelity of
the formation for viewer $m$ is:

$$V_m = \frac{\bigl|\bigl\{k : \|\Pi_m(\mathbf{q}_k) - \Pi_m(\mathbf{p}_{\sigma^*(k)})\| \leq \epsilon_{proj}\bigr\}\bigr|}{N}$$

where $\epsilon_{proj} = 0.8$ m is the projected position tolerance (larger than the 3D tolerance
$\epsilon_{pos} = 0.5$ m because depth-layer offsets introduce a small projected displacement for
oblique viewers). The overall multi-view fidelity is:

$$\bar{V} = \frac{1}{M} \sum_{m=1}^{M} V_m$$

### Viewer-Optimal Depth Scaling

To balance legibility across all $M$ viewers simultaneously, the depth-layer spacing $\Delta z$ is
chosen to minimise the worst-case projected distortion across viewers:

$$\Delta z^* = \operatorname{argmin}_{\Delta z \geq 0}
  \max_{m \in \{1,\ldots,M\}} \operatorname{distortion}_m(\Delta z)$$

where:

$$\operatorname{distortion}_m(\Delta z) = \frac{1}{N} \sum_{k=1}^{N}
  \|\Pi_m(\mathbf{p}_k(\Delta z)) - \Pi_{ref}(\mathbf{p}_k(0))\|$$

and $\Pi_{ref}$ is the frontal projection from the primary viewer at $\phi_{ref} = 0°$.

The optimum can be evaluated on a 1D grid $\Delta z \in [0, 8]$ m at 0.5 m resolution
($\mathcal{O}(16 \cdot N \cdot M)$ evaluations).

### Formation PID Controller (unchanged from S087)

Each drone $k$ uses a decoupled PID controller along each axis $\alpha \in \{x, y, z\}$:

$$u_\alpha(t) = K_p\,e_\alpha(t) + K_i \int_0^t e_\alpha(\tau)\,d\tau + K_d\,\dot{e}_\alpha(t)$$

where $e_\alpha = p_{\sigma^*(k),\alpha} - q_{k,\alpha}(t)$. The z-channel is now active (in S087 it
was effectively trivial since all targets had identical z). Velocity and acceleration are clipped as
before: $v_{max} = 5$ m/s, $a_{max} = 3$ m/s$^2$.

---

## Key 3D Additions

- **Depth-layer extrusion**: stroke classifier partitions lit pixels into 3 z-layers; the formation
  is a volumetric bas-relief rather than a flat plane.
- **3D Hungarian assignment**: cost matrix uses full $\mathbb{R}^3$ distances so the solver
  naturally assigns drones to nearby z-layers, reducing vertical travel during morphs.
- **Cubic Bezier morph arc**: control points are offset in z by $h_{arc} = 6.0$ m, producing a
  visible depth-sweep wave during the "DRONE" $\to$ "SHOW" transition.
- **Staggered depth timing**: drones in different z-layers reach peak arc height at offsets of
  $\Delta\tau_{peak} = 0.1$, so layers pass through the arc apex in sequence (layer 0 first).
- **Multi-viewer projection fidelity**: $\bar{V}$ computed over $M = 4$ viewers at $0°, 45°, 90°,
  135°$ azimuths; $\Delta z^*$ is selected by minimax optimisation over this viewer set.
- **Active z-axis PID control**: z-channel errors are non-trivial in every phase (ascent, display
  hold, morph), testing the full 3D controller.

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Number of drones $N$ | 30 |
| Display altitude $z_{show}$ | 50 m |
| Depth layers $L$ | 3 |
| Inter-layer spacing $\Delta z$ | 3.0 m (optimised over viewers) |
| z range (formation) | 50 – 56 m |
| Glyph size | 6 rows x 5 columns |
| Inter-pixel spacing $d_{px}$ | 2.0 m |
| Inter-character gap | 2 extra columns (= 4.0 m) |
| Word A | "DRONE" (5 characters) |
| Word B | "SHOW" (4 characters, padded to 30 lit pixels) |
| Bezier arc height $h_{arc}$ | 6.0 m |
| Layer arc-time offset $\Delta\tau_{peak}$ | 0.1 |
| Number of viewers $M$ | 4 (at 0°, 45°, 90°, 135°) |
| Projected position tolerance $\epsilon_{proj}$ | 0.8 m |
| PID gains $K_p / K_i / K_d$ | 2.0 / 0.05 / 0.8 |
| Max speed $v_{max}$ | 5.0 m/s |
| Max acceleration $a_{max}$ | 3.0 m/s$^2$ |
| Separation radius $r_{sep}$ | 1.5 m |
| 3D position tolerance $\epsilon_{pos}$ | 0.5 m |
| Simulation timestep $dt$ | 0.05 s |

---

## Expected Output

- **3D formation plot — "DRONE" with depth layers**: scatter plot using `mpl_toolkits.mplot3d`
  showing all 30 drone positions coloured by depth layer (layer 0 = red, layer 1 = orange, layer 2 =
  yellow); the bas-relief silhouette is visible when the plot is rotated to an oblique azimuth.
- **3D formation plot — "SHOW" with depth layers**: same layout for the second word; side-by-side
  with "DRONE" panel.
- **Viewer projection panels**: for each of the $M = 4$ viewers, a 2D panel showing the projected
  pixel grid (expected) and projected drone positions (actual); per-viewer fidelity $V_m$ annotated.
- **$\Delta z$ optimisation curve**: line plot of $\max_m \operatorname{distortion}_m(\Delta z)$
  vs $\Delta z \in [0, 8]$ m; the selected $\Delta z^*$ marked with a vertical dashed line.
- **Bezier morph animation (GIF)**: 3D animated view of the "DRONE" $\to$ "SHOW" transition; drone
  trails coloured by depth-layer; the rolling z-sweep wave is visible as drones in each layer crest
  the Bezier arc at staggered times; camera slowly orbits to reveal the 3D structure.
- **Altitude time series**: z-coordinate vs time for one representative drone from each layer;
  shows the Bezier arc peak and layer stagger.
- **Multi-view fidelity vs time**: $\bar{V}(t)$ and all four $V_m(t)$ on a single plot; 0.95
  acceptance threshold marked; demonstrates that 3D extrusion maintains acceptable fidelity even for
  the 90° side viewer.
- **Assignment cost comparison**: Hungarian 3D cost vs greedy 3D cost for both transitions,
  annotated with percentage saving $\Delta_{cost}$.

---

## Extensions

1. **Continuous depth-gradient font**: replace the 3-level discrete classifier with a signed
   distance field (SDF) computed on the $6 \times 5$ bitmap; map the SDF value directly to a
   continuous z-offset, producing smooth curved glyph surfaces rather than stepped layers.
2. **Spectator-position uncertainty**: model each viewer's azimuth as a Gaussian
   $\phi_m \sim \mathcal{N}(\bar\phi_m, \sigma_\phi^2)$ and optimise $\Delta z^*$ to minimise
   expected worst-case distortion under this uncertainty.
3. **Dynamic viewpoint tracking**: equip each drone with knowledge of a moving audience cluster;
   update the viewer-projection fidelity and $\Delta z$ in real time as the crowd moves, causing the
   formation to slowly tilt and re-extrude toward the majority viewing direction.
4. **Colour-coded depth**: assign distinct LED colours to each depth layer (e.g., warm tones at
   front, cool tones at back); extend the Hungarian cost matrix to include a colour-distance term so
   that layer colour is preserved after morph reassignment.
5. **Full word sequence choreography**: chain three or more words (e.g., "DRONE", "SHOW", "2025")
   with beat-synchronised Bezier morphs; the depth-wave timing is locked to the musical bar
   structure, with each layer completing its arc peak on a different beat subdivision.
6. **Observer-plane shadow fidelity**: project the extruded 3D formation onto the ground plane
   ($z = 0$) using a spot-light model; optimise $h_{arc}$ and $\Delta z$ jointly to maximise both
   the aerial visual fidelity and the ground-shadow legibility.

---

## Related Scenarios

- Original 2D version: [S087](../S087_led_text.md)
- Prerequisites: [S083](../S083_light_show_single.md), [S085](../S085_light_matrix.md)
- Follow-up: [S088](../S088_formation_morphing.md)
- Algorithmic cross-reference: [S018 Multi-Target Interception](../../01_pursuit_evasion/S018_multi_target_interception.md) (Hungarian assignment in 3D), [S019 Dynamic Reassignment](../../01_pursuit_evasion/S019_dynamic_reassignment.md) (real-time re-assignment)
