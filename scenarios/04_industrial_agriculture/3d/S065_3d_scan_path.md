# S065 3D Upgrade — Building 3D Scan Path

**Domain**: Industrial & Agriculture | **Difficulty**: ⭐⭐⭐ | **Status**: `[ ]` Not Started
**Based on**: [S065 original](../S065_3d_scan_path.md)

---

## What Changes in 3D

The original S065 models the building as a simple rectangular box ($20 \times 15 \times 10$ m)
with four flat, featureless facades and a flat roof. The drone scans each facade using uniform
horizontal strips at a fixed standoff of $d_{scan} = 3$ m, and all facade normals point exactly
along one cardinal axis. In a real inspection the building has architectural complexity that breaks
this assumption:

- **L-shaped floor plan**: the footprint is non-convex, so a single rectangular bounding standoff
  distance is no longer adequate — inner-corner facades are under-scanned at $d = 3$ m, while
  outer faces may require a different scan radius.
- **Atrium void** (open interior courtyard): the drone must descend into the atrium and scan the
  inward-facing facade sections, which face inward and cannot be reached from the exterior orbit.
- **Balcony protrusions**: balcony slabs protrude 1.5 m from certain facades, creating occlusion
  shadows directly below them. Additional below-balcony viewpoints at reduced altitude and
  steeper tilt are required to fill coverage gaps.
- **Multi-face helical orbit**: instead of independent per-facade strip plans, this variant flies a
  continuous helical orbit that circumnavigates the full building perimeter (following the L-shaped
  boundary at a constant standoff) while ascending from $z_{min}$ to $H_{bldg}$, guaranteeing
  seamless facade-to-facade transition coverage.
- **Viewpoint planning for full 360° × vertical coverage**: every facade segment (outer faces,
  inner atrium faces, balcony undersides) is represented as a discrete set of target surface
  patches; a greedy set-cover algorithm selects the minimum set of additional oblique viewpoints
  needed to cover patches missed by the helical sweep.

---

## Problem Definition

**Setup**: A single inspection drone must photogrammetrically scan a structurally complex building
whose floor plan is L-shaped (main wing $20 \times 15$ m, secondary wing $12 \times 8$ m,
total height $H_{bldg} = 12$ m). The building also features:

- An open atrium of $6 \times 6$ m cutting into the re-entrant corner of the L, with exposed
  internal facades rising to $H_{atrium} = 8$ m.
- Two rows of balconies ($1.5$ m protrusion, $0.8$ m slab thickness) on the main south facade
  at $z = 4$ m and $z = 8$ m.

The drone carries the same camera as S065 ($f = 16$ mm, $s_w = 17.6$ mm, $W_{px} = 4000$ px,
$\theta_h = 60°$, $\theta_v = 45°$) but now adds the capability to tilt its gimbal up to
$\pm 30°$ off nadir to access balcony undersides and atrium upper edges.

**Scan phases**:

1. **Exterior helical sweep**: drone follows the L-shaped perimeter at standoff $d_{ext} = 3$ m,
   ascending from $z_{min} = 1.0$ m to $z_{top} = H_{bldg} + 1 = 13$ m in a continuous helix.
   The helix pitch is set so that consecutive loops overlap by $p_z = 30\%$ vertically.
2. **Atrium descent**: drone enters the atrium through the open top, spirals downward from
   $z = H_{atrium}$ to $z = 1.5$ m at standoff $d_{atrium} = 1.5$ m from each inward-facing
   wall, with $p_x = 60\%$ forward overlap.
3. **Balcony underside passes**: for each balcony row, drone flies a horizontal pass at
   $z_{bal} - 0.5$ m (below the slab) with gimbal tilted $+20°$ upward to image the underside;
   standoff from facade $d_{bal} = 2$ m.
4. **Viewpoint gap-fill**: a greedy set-cover selects oblique viewpoints (position + gimbal tilt)
   to cover any surface patch with current coverage score below the threshold $C_{min} = 1$.

**Roles**:

- **Drone**: single UAV with 3D position $\mathbf{p} = [x, y, z]^\top$, cruise speed
  $v = 1.0$ m/s, gimbal tilt range $[-30°, +30°]$ off nadir, hover time $t_{cap} = 1.5$ s
  per capture.
- **Building**: static L-shaped structure; outer and inner facade polygons defined as a list of
  2D perimeter segments in the $x$-$y$ plane; extruded to height $H_{bldg}$.

**Objective**: Plan a complete multi-phase waypoint path that achieves GSD $\leq 2$ cm and
$\geq 60\% / 30\%$ forward/side overlap on all surface patches (exterior, atrium, balcony
undersides) while minimising total 3D path length across all phases.

---

## Mathematical Model

### Building Perimeter Parameterisation

The L-shaped perimeter is described by an ordered polygon $\mathcal{P} = \{P_0, P_1, \ldots, P_{M-1}\}$
in the $x$-$y$ plane. Each edge $e_j = \overrightarrow{P_j P_{j+1}}$ has outward unit normal
$\hat{\mathbf{n}}_j$ (computed by rotating the edge tangent 90° clockwise):

$$\hat{\mathbf{n}}_j = \frac{1}{\|e_j\|}\begin{bmatrix}
\Delta y_j \\ -\Delta x_j
\end{bmatrix}$$

The standoff orbit point opposite the midpoint of edge $j$ at altitude $z$ is:

$$\mathbf{q}_j(z) = \bar{\mathbf{P}}_j + d_{ext}\,\hat{\mathbf{n}}_j + z\,\hat{\mathbf{z}}$$

where $\bar{\mathbf{P}}_j = (P_j + P_{j+1})/2$.

### Exterior Helical Sweep

The total perimeter arc length is:

$$S_{perim} = \sum_{j=0}^{M-1} \|e_j\|$$

The vertical footprint at standoff $d_{ext}$:

$$h_{foot} = 2\,d_{ext}\tan\!\left(\frac{\theta_v}{2}\right)$$

With side overlap $p_z = 0.30$, the altitude gain per full perimeter loop is:

$$\Delta z_{loop} = h_{foot}\,(1 - p_z)$$

The number of complete helical loops from $z_{min}$ to $z_{top}$:

$$N_{loops} = \left\lceil \frac{z_{top} - z_{min}}{\Delta z_{loop}} \right\rceil$$

The helix is parameterised by arc parameter $s \in [0,\, N_{loops} \cdot S_{perim}]$.
At arc position $s$, the drone is at:

$$z(s) = z_{min} + \frac{z_{top} - z_{min}}{N_{loops} \cdot S_{perim}} \cdot s$$

The $x$-$y$ position follows the perimeter polygon cyclically:

$$[x(s),\,y(s)] = \text{PerimeterOffset}(s \bmod S_{perim},\; d_{ext})$$

where $\text{PerimeterOffset}$ maps arc length to the corresponding offset point on the outer
standoff polygon.

Capture positions are spaced at arc intervals:

$$\Delta s_{cap} = w_{foot}\,(1 - p_x), \qquad w_{foot} = 2\,d_{ext}\tan\!\left(\frac{\theta_h}{2}\right)$$

### Atrium Spiral Descent

The atrium is a $6 \times 6$ m open courtyard. The drone follows a rectangular spiral at standoff
$d_{atrium} = 1.5$ m from each inward wall. The perimeter of the atrium scan orbit is:

$$S_{atrium} = 4 \times (6 - 2\,d_{atrium}) = 4 \times 3 = 12 \text{ m}$$

The drone descends from $z = H_{atrium} = 8$ m to $z = 1.5$ m. The altitude drop per loop:

$$\Delta z_{atrium} = h_{foot,atrium}\,(1 - p_z), \quad h_{foot,atrium} = 2\,d_{atrium}\tan\!\left(\frac{\theta_v}{2}\right)$$

Capture arc interval within the atrium:

$$\Delta s_{atrium} = 2\,d_{atrium}\tan\!\left(\frac{\theta_h}{2}\right) \cdot (1 - p_x)$$

### Balcony Underside Viewpoints

For each balcony slab at elevation $z_{bal}$ and protrusion depth $\ell_{bal} = 1.5$ m, the
underside patch subtends an angle from a standoff point at $(x_{facade} - d_{bal},\, z_{bal} - 0.5)$.
The required gimbal tilt $\phi_{tilt}$ to centre the FOV on the underside patch:

$$\phi_{tilt} = \arctan\!\left(\frac{0.4}{\,d_{bal}\,}\right) \approx 11.3°$$

To ensure GSD $\leq 2$ cm on the underside at the slant distance
$d_{slant} = \sqrt{d_{bal}^2 + 0.4^2}$:

$$\mathrm{GSD}_{slant} = \frac{s_w \cdot d_{slant}}{f \cdot W_{px}} \leq 0.02 \text{ m}$$

Capture spacing along the balcony length $L_{bal}$:

$$B_{bal} = w_{foot,bal}\,(1 - p_x), \qquad w_{foot,bal} = 2\,d_{slant}\tan\!\left(\frac{\theta_h}{2}\right)$$

### Surface Patch Coverage Model

The building surfaces are discretised into a grid of rectangular patches of size
$\delta_u \times \delta_v = 0.5 \times 0.5$ m. Each patch $k$ has a coverage counter $C_k$
initialised to 0. A viewpoint at position $\mathbf{p}_{vp}$ with gimbal tilt $\phi$ covers patch
$k$ if and only if:

1. The patch centre $\mathbf{c}_k$ lies within the camera frustum projected from $\mathbf{p}_{vp}$
   at tilt $\phi$.
2. The slant distance $d_k = \|\mathbf{p}_{vp} - \mathbf{c}_k\|$ satisfies
   $\mathrm{GSD}(d_k) \leq 0.02$ m.
3. No building geometry occludes the line segment $[\mathbf{p}_{vp},\,\mathbf{c}_k]$.

$$C_k \mathrel{+}= 1 \quad \text{if all three conditions hold}$$

After the helical sweep and atrium descent, uncovered patches ($C_k = 0$) form the residual set
$\mathcal{U}$. The greedy set-cover selects viewpoints from a candidate set
$\mathcal{V}$ (dense grid of oblique positions around the building at multiple tilt angles):

$$\mathbf{vp}^* = \arg\max_{\mathbf{vp} \in \mathcal{V}} |\{k \in \mathcal{U} : \mathbf{vp} \text{ covers } k\}|$$

Repeat until $\mathcal{U} = \emptyset$.

### Total Path Length and Mission Time

Let all waypoints across the three phases and gap-fill be $\{\mathbf{w}_i\}_{i=1}^{N_{WP}}$,
already sequenced phase by phase (helix order, then atrium descent order, then balcony passes,
then gap-fill in greedy selection order). Total 3D path length:

$$L_{total} = \sum_{i=1}^{N_{WP}-1} \|\mathbf{w}_{i+1} - \mathbf{w}_i\|$$

Estimated mission time:

$$T_{mission} = \frac{L_{total}}{v} + N_{WP} \cdot t_{cap}$$

---

## Key 3D Additions

- **L-shaped perimeter**: the flat rectangular bounding box is replaced by a non-convex polygon;
  the standoff orbit follows the true building boundary, with special handling at re-entrant
  corners where the orbit polygon must be inset rather than offset.
- **Continuous helical orbit**: altitude increases monotonically along the perimeter arc, ensuring
  no strip-boundary jumps and providing seamless facade-to-facade coverage across all exterior
  faces of the L-shape.
- **Atrium descent spiral**: the drone enters the open courtyard from above and spirals downward;
  the smaller standoff ($d_{atrium} = 1.5$ m vs $d_{ext} = 3$ m) is computed from the atrium
  cavity width, with GSD re-verified at the closer range.
- **Balcony underside gimbal tilt**: viewpoints below each balcony row use a forward-tilted gimbal
  ($\phi_{tilt} \approx 11°$–$30°$) to fill occlusion shadows not reachable by the vertical helix.
- **Greedy set-cover gap-fill**: surface patches are explicitly tracked with per-patch coverage
  counters; remaining uncovered patches after the main sweep trigger selection of the minimum
  number of additional oblique viewpoints.
- **3D altitude profile**: z varies continuously throughout the mission, making this a genuine
  3D path rather than a sequence of constant-altitude strips.

---

## Key Parameters

| Parameter | Symbol | Value |
|-----------|--------|-------|
| Main wing dimensions | $L_{main} \times W_{main}$ | 20 × 15 m |
| Secondary wing dimensions | $L_{sec} \times W_{sec}$ | 12 × 8 m |
| Building height | $H_{bldg}$ | 12 m |
| Atrium size | $L_{atr} \times W_{atr}$ | 6 × 6 m |
| Atrium height | $H_{atrium}$ | 8 m |
| Balcony protrusion | $\ell_{bal}$ | 1.5 m |
| Balcony elevations | $z_{bal}$ | 4 m, 8 m |
| Exterior standoff | $d_{ext}$ | 3.0 m |
| Atrium standoff | $d_{atrium}$ | 1.5 m |
| Balcony pass standoff | $d_{bal}$ | 2.0 m |
| Minimum scan altitude | $z_{min}$ | 1.0 m |
| Top scan altitude | $z_{top}$ | 13.0 m |
| Vertical overlap | $p_z$ | 30% |
| Forward overlap | $p_x$ | 60% |
| GSD requirement | $\mathrm{GSD}_{max}$ | 2 cm |
| GSD at $d_{ext} = 3$ m | $\mathrm{GSD}_{ext}$ | ≈ 0.83 cm |
| GSD at $d_{atrium} = 1.5$ m | $\mathrm{GSD}_{atr}$ | ≈ 0.41 cm |
| Gimbal tilt range | $\phi$ | $[-30°, +30°]$ |
| Patch coverage grid | $\delta_u \times \delta_v$ | 0.5 × 0.5 m |
| Coverage threshold | $C_{min}$ | 1 image |
| Inspection speed | $v$ | 1.0 m/s |
| Capture hover time | $t_{cap}$ | 1.5 s |

---

## Expected Output

- **3D trajectory overview**: `mpl_toolkits.mplot3d` scene showing the L-shaped building as a
  grey transparent extrusion; the exterior helix coloured by altitude (cool-to-warm colormap);
  the atrium descent spiral in cyan; balcony underside passes in orange; gap-fill viewpoints
  marked as green stars; launch and landing positions marked with black triangles.
- **Altitude vs arc-length profile**: 2D plot showing $z(s)$ along the exterior helix arc,
  confirming monotonic ascent with the computed $\Delta z_{loop}$ pitch; overlaid with the
  atrium descent profile on a secondary x-axis.
- **Surface coverage heat map**: plan-view (top-down) and elevation-view of the building with
  each $0.5 \times 0.5$ m patch coloured by $C_k$ (coverage count); patches with $C_k = 0$
  after the helix highlighted in red; patches newly covered by gap-fill viewpoints highlighted
  in green; final state should show all patches with $C_k \geq 1$.
- **Phase breakdown bar chart**: waypoint count and path-length contribution for each phase
  (exterior helix, atrium descent, balcony passes, gap-fill); confirms the relative cost of
  each phase.
- **Scan tour animation (GIF)**: animated drone icon traversing the full multi-phase path
  phase by phase; colour of the completed path trace changes per phase; building rendered as
  a transparent L-shape; frame counter and current phase label in the title.
- **Metrics printed to stdout**:

```
============================================================
  Building footprint  : L-shape (20×15 + 12×8 m)
  Perimeter (exterior): ~96 m
  Helical loops       : 7
  GSD at d_ext        : 0.83 cm
  GSD at d_atrium     : 0.41 cm
  Phase 1 waypoints   : ~420  (exterior helix)
  Phase 2 waypoints   : ~88   (atrium descent)
  Phase 3 waypoints   : ~36   (balcony undersides)
  Phase 4 waypoints   : ~24   (gap-fill)
  Total waypoints     : ~568
  Total path length   : ~780 m
  Mission time        : ~1640 s  (~27.3 min)
  Uncovered patches   : 0
============================================================
```

---

## Extensions

1. **Re-entrant corner conflict resolution**: at concave corners of the L-shape, the naive
   outward-offset polygon self-intersects; implement a Minkowski-sum buffering algorithm to
   produce a valid convex-inset standoff polygon and compare path length against the naive
   version.
2. **Multi-rotor wind disturbance model**: add a horizontal wind field $\mathbf{w}_{wind}(z)$
   that increases with altitude (logarithmic wind profile); re-plan the helix pitch adaptively
   so that image blur due to wind-induced vibration remains below the GSD budget at every
   altitude.
3. **2-opt++ tour improvement across phases**: after sequencing phase-by-phase, apply 2-opt
   swaps that are allowed to exchange waypoints between adjacent phases; quantify the path
   length reduction versus the constraint that phase ordering is preserved.
4. **Occlusion-aware gap-fill with ray casting**: replace the simple frustum check in the
   set-cover with full ray-triangle intersection against a mesh of the building geometry;
   evaluate coverage accuracy improvement on balcony soffits and atrium upper edges.
5. **Battery endurance and recharge planning**: given a battery budget of $E_{max}$ Wh and
   energy consumption model $P(v, z\dot{})$, partition the multi-phase waypoint sequence into
   minimum-recharge-stop sub-missions; locate optimal recharge landing pads on the roof.

---

## Related Scenarios

- Original 2D version: [S065 Building 3D Scan Path](../S065_3d_scan_path.md)
- Prerequisites: [S053 Coral Reef 3D Reconstruction](../../03_environmental_sar/S053_coral_reef.md) (GSD-constrained viewpoint planning, set-cover), [S061 Power Line Inspection](../S061_power_line.md) (facade-parallel strip fundamentals)
- Follow-ups: [S074 Mine 3D Mapping](../S074_mine_mapping.md) (enclosed-space 3D scanning without GPS)
- Algorithmic cross-reference: [S002 3D Evasive Maneuver](../../01_pursuit_evasion/3d/S002_3d_evasive_maneuver.md) (3D helical trajectory parameterisation)
