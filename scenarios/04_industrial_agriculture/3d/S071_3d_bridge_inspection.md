# S071 3D Upgrade — Bridge Underside Inspection

**Domain**: Industrial & Agriculture | **Difficulty**: ⭐⭐⭐⭐ | **Status**: `[ ]` Not started
**Based on**: [S071 original](../S071_bridge_inspection.md)

---

## What Changes in 3D

The original S071 treats the bridge underside as a perfectly flat plane at a fixed height
z_deck = 8.0 m. Strip geometry is computed purely in (x, y); the height controller maintains
a scalar clearance from that single plane. Real bridge undersides are structurally complex:
I-girders run longitudinally, crossbeams (diaphragms) span transversely, and the deck soffit
may have a vertical camber of 0.1–0.3 m. These elements create a piecewise non-planar surface
that requires the drone to modulate altitude in both x and y rather than holding a constant z.

Three concrete limitations of S071 become active in this upgrade:

1. **Flat-surface assumption** — `z_deck` is a scalar; the controller error `e_z = d_meas - D_TARGET`
   has no spatial dependence. In 3D the ceiling height becomes a function
   `z_ceil(x, y)` that the drone must query at every position.
2. **Single-axis standoff** — the upward rangefinder measures only vertical clearance.
   In 3D the true standoff vector from the drone to the nearest soffit point is
   oblique whenever a girder web or crossbeam face is directly ahead; the drone
   must maintain a 3D minimum-distance constraint, not just a z-constraint.
3. **Camera frustum is axis-aligned** — the footprint model uses
   `|x - x_i| ≤ h_strip ∧ |y - y_i| ≤ h_strip`. When the camera is tilted to aim at an
   inclined girder web, the footprint on the surface is a rotated ellipse; detection geometry
   must use a 3D projection.

This variant replaces the flat deck with a full girder-and-crossbeam surface mesh, adds a
3D signed-distance standoff controller, models the tilted upward-facing camera frustum in 3D,
and introduces a viewpoint optimisation layer that selects per-strip altitude to maximise
surface coverage on non-planar faces.

---

## Problem Definition

**Setup**: A concrete girder bridge spans 50 m (x) × 10 m (y). The soffit geometry consists of:
- **N_g = 5 longitudinal I-girders** at y positions y_g ∈ {1.0, 3.0, 5.0, 7.0, 9.0} m,
  each with flange width 0.4 m and web height h_web = 0.6 m extending downward from
  z_deck = 8.0 m to z_flange = 7.4 m.
- **N_c = 10 crossbeams** at x positions x_c ∈ {5, 10, 15, 20, 25, 30, 35, 40, 45, 50} m,
  each with depth 0.4 m (z_cb ∈ [7.6, 8.0] m) spanning the full bridge width.
- **Soffit plane** at z_deck = 8.0 m between structural elements.

The drone must fly below all structural elements (z < 7.3 m at minimum safety margin) while
maintaining a 3D standoff of d_target = 0.5 m from the nearest soffit surface at every pose.
An upward-facing camera with full-cone half-angle α = 45° is mounted on a ±30° tilt gimbal
about the y-axis, allowing the camera bore-sight to be rotated toward girder webs without
changing the drone's lateral position.

**Roles**: Single inspection drone.

**Objective**: Cover 100 % of the soffit surface area (soffit plane + all girder web faces +
crossbeam faces) using adaptive boustrophedon strips at variable altitude, keeping 3D standoff
d_3D(t) ≥ d_min = 0.3 m from any surface element at all times, and maximising the fraction of
seeded surface cracks detected. Report total surface coverage fraction C_3D, crack detection
rate, gimbal tilt usage, and minimum standoff margin observed.

---

## Mathematical Model

### Soffit Surface Representation

Define the piecewise soffit height field as a function of drone (x, y):

$$z_{ceil}(x, y) = \min\!\left(z_{deck},\; \min_{g} z_{girder}(y;\, y_g),\; \min_{c} z_{cross}(x;\, x_c)\right)$$

Girder ceiling contribution (downward-protruding web):

$$z_{girder}(y;\, y_g) = \begin{cases} z_{flange} & |y - y_g| \leq w_{flange}/2 \\ z_{deck} & \text{otherwise} \end{cases}$$

Crossbeam ceiling contribution:

$$z_{cross}(x;\, x_c) = \begin{cases} z_{cb,bot} & |x - x_c| \leq w_{cb}/2 \\ z_{deck} & \text{otherwise} \end{cases}$$

The nominal flight altitude for a position (x, y) is:

$$z_{nom}(x, y) = z_{ceil}(x, y) - d_{target}$$

### 3D Standoff Vector and Nearest Surface Point

Let $\mathcal{S}$ denote the complete soffit surface mesh (plane + girder webs + crossbeam faces).
The signed 3D distance from drone position $\mathbf{p} = (x, y, z)^\top$ to the nearest surface point $\mathbf{s}^* \in \mathcal{S}$:

$$d_{3D}(\mathbf{p}) = \min_{\mathbf{s} \in \mathcal{S}} \|\mathbf{p} - \mathbf{s}\|$$

The standoff unit vector pointing from the drone toward the nearest surface element:

$$\hat{\mathbf{n}}(\mathbf{p}) = \frac{\mathbf{s}^* - \mathbf{p}}{\|\mathbf{s}^* - \mathbf{p}\|}$$

For the simplified analytical surface: the nearest point is computed by projecting $\mathbf{p}$ onto
each planar face (soffit plane, each girder web, each crossbeam face) and taking the minimum-distance candidate.

### 3D Standoff Controller

Standoff error from 3D nearest surface:

$$e_{3D}(t) = d_{3D}\bigl(\mathbf{p}(t)\bigr) - d_{target}$$

The control output is a correction along the current standoff direction:

$$\mathbf{u}_{standoff}(t) = -\bigl(K_p \, e_{3D}(t) + K_d \, \dot{e}_{3D}(t)\bigr) \cdot \hat{\mathbf{n}}\bigl(\mathbf{p}(t)\bigr)$$

Critically-damped gains with $\omega_n = 4$ rad/s:

$$K_p = \omega_n^2 = 16 \; \text{s}^{-2}, \qquad K_d = 2\,\omega_n = 8 \; \text{s}^{-1}$$

Safety hard constraint: if $d_{3D}(\mathbf{p}) < d_{min} = 0.3$ m, override with emergency repulsion:

$$\mathbf{u}_{emg}(t) = -K_{emg}\,\bigl(d_{min} - d_{3D}(\mathbf{p})\bigr) \cdot \hat{\mathbf{n}}\bigl(\mathbf{p}(t)\bigr), \qquad K_{emg} = 40 \; \text{s}^{-2}$$

### Gimbal Tilt Optimisation

At each drone position $\mathbf{p}$, the upward gimbal tilt angle $\phi \in [-30°, +30°]$ is selected
to maximise the projected area of the camera footprint on the nearest soffit face.

Camera bore-sight unit vector as a function of gimbal tilt $\phi$ (rotation about y-axis):

$$\hat{\mathbf{b}}(\phi) = \bigl(\sin\phi,\; 0,\; \cos\phi\bigr)^\top$$

The footprint centroid on the soffit face is the ray-surface intersection:

$$\mathbf{c}_{fp} = \mathbf{p} + t^* \hat{\mathbf{b}}(\phi), \quad t^* = \frac{(\mathbf{s}_{ref} - \mathbf{p}) \cdot \hat{\mathbf{n}}}{\hat{\mathbf{b}}(\phi) \cdot \hat{\mathbf{n}}}$$

Optimal tilt points the bore-sight directly at the nearest surface point:

$$\phi^*(x, y, z) = \arctan\!\left(\frac{s^*_x - x}{s^*_z - z}\right)$$

### 3D Camera Footprint on a Tilted Surface

For bore-sight direction $\hat{\mathbf{b}}$ and half-angle $\alpha$, the footprint on a planar face with
normal $\hat{\mathbf{n}}_{face}$ is an ellipse. The semi-axes of that ellipse are:

$$a = d_{3D} \tan\alpha / \cos\theta_i, \qquad b = d_{3D} \tan\alpha$$

where $\theta_i = \arccos(\hat{\mathbf{b}} \cdot \hat{\mathbf{n}}_{face})$ is the incidence angle.
A point $\mathbf{q}$ on the face is within the footprint when:

$$\bigl\|\mathbf{q} - \mathbf{c}_{fp} - \bigl[(\mathbf{q}-\mathbf{c}_{fp})\cdot\hat{\mathbf{b}}\bigr]\hat{\mathbf{b}}\bigr\| \leq d_{3D} \tan\alpha$$

### Adaptive Strip Altitude

Between structural elements the drone holds $z_{nom}(x,y)$ as computed above.
When crossing below a crossbeam at $x_c$, the transition height profile is:

$$z_{trans}(x) = z_{nom,far} + \bigl(z_{nom,near} - z_{nom,far}\bigr)\, \exp\!\!\left(-\frac{(x-x_c)^2}{2\sigma_{trans}^2}\right)$$

with $z_{nom,near} = z_{cb,bot} - d_{target}$ and $\sigma_{trans}$ = 1.0 m (transition half-width).

### Coverage Metric on Non-Planar Surface

Discretise each face of $\mathcal{S}$ into cells of area $\delta^2 = (0.05)^2$ m². Cell $c$ is
covered when at least one camera footprint projection contains its centroid:

$$C_{3D} = \frac{\#\{\text{covered surface cells}\}}{\#\{\text{total surface cells}\}}$$

Total inspectable area (flat soffit + girder web areas + crossbeam face areas):

$$A_{total} = L_x L_y + N_g \cdot h_{web} \cdot L_x + N_c \cdot h_{cb} \cdot L_y$$

### Boustrophedon Path with Altitude Modulation

The 2D strip centres $\{(x_k, y_k)\}$ are generated identically to S071. In the 3D upgrade, each
waypoint carries a dynamically computed z-coordinate:

$$\mathbf{w}_k = \bigl(x_k,\; y_k,\; z_{nom}(x_k, y_k)\bigr)^\top$$

where $z_{nom}$ is re-evaluated from the soffit height field at the planned (x, y). During
traversal, the standoff controller continuously corrects z (and x, y when near a girder web face)
to maintain $d_{3D} = d_{target}$.

---

## Key 3D Additions

- **Soffit height field** $z_{ceil}(x,y)$: piecewise function encoding girder flanges and crossbeam undersides, replacing the scalar `Z_DECK = 8.0`.
- **3D signed-distance standoff controller**: computes nearest surface point on full mesh and applies repulsion/attraction along the 3D standoff vector, not just the vertical axis.
- **Gimbal tilt optimisation** $\phi^*(x,y,z)$: rotates camera bore-sight to face girder webs during transverse passes, expanding coverage of vertical surfaces.
- **Elliptical footprint model**: replaces axis-aligned square footprint with incidence-angle-corrected ellipse projected onto each face normal.
- **Adaptive altitude waypoints**: strip z-coordinates vary with position, descending under girder flanges and ascending past crossbeams on-the-fly.
- **3D surface coverage grid**: separate coverage bitmaps per face (soffit, N_g girder webs, N_c crossbeam faces), unified into $C_{3D}$.

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Bridge length (x) | 50 m |
| Bridge width (y) | 10 m |
| Soffit height z_deck | 8.0 m |
| Number of longitudinal girders N_g | 5 |
| Girder spacing | 2.0 m |
| Girder web height h_web | 0.6 m |
| Girder flange width w_flange | 0.4 m |
| Number of crossbeams N_c | 10 |
| Crossbeam depth h_cb | 0.4 m |
| Crossbeam spacing | 5.0 m |
| Inspection standoff d_target | 0.5 m |
| Minimum standoff d_min | 0.3 m |
| Altitude range z | 6.5 – 7.5 m |
| 3D standoff gains (Kp, Kd) | 16, 8 s^-2, s^-1 |
| Emergency repulsion gain K_emg | 40 s^-2 |
| Rangefinder noise std | 0.02 m |
| Camera FOV half-angle | 45 deg |
| Gimbal tilt range | ±30 deg about y-axis |
| Strip width w_strip | 1.0 m |
| Strip overlap | 10 % |
| Lateral step Delta_y | 0.9 m |
| Number of x-passes N_pass | 12 |
| Cruise speed | 1.0 m/s |
| Surface grid resolution delta | 0.05 m |
| Transition half-width sigma_trans | 1.0 m |
| Crack detection probability | 0.90 |
| Synthetic crack count | 30 |
| Simulation timestep | 0.05 s |

---

## Expected Output

- **3D soffit geometry + trajectory plot**: Matplotlib 3D axes rendering the five girder webs
  (grey translucent vertical panels), ten crossbeams (dark grey boxes), and the soffit plane
  (light grey); flight path overlaid in red with altitude variation visible; detected cracks
  marked as green stars on the appropriate surface face.
- **Altitude time series z(t)**: shows the variable z profile — periodic dips when flying below
  girder flanges, periodic humps when crossing below crossbeams; reference lines for z_nom_max
  (soffit plane minus d_target) and d_min safety envelope.
- **3D standoff distance time series d_3D(t)**: illustrates how the standoff controller maintains
  d_3D ≈ 0.5 m across all surface types; emergency-repulsion events highlighted.
- **Gimbal tilt angle time series phi(t)**: shows transitions between vertical bore-sight (phi = 0°
  over flat soffit) and tilted positions (|phi| > 0° near girder webs); colour-coded by surface type.
- **Per-face coverage heatmap**: separate top-down or unfolded 2D maps for the soffit plane,
  one representative girder web, and one crossbeam face, each filled green for covered cells.
- **Strip-by-strip cumulative coverage bar chart**: 12 bars showing incremental C_3D contribution
  per pass, broken down by surface type (soffit / girder web / crossbeam).
- **Animation** (FuncAnimation): oblique 3D view following the drone through its adaptive
  boustrophedon path; coverage cells fill in real-time; gimbal tilt indicated by an arrow from the
  drone; crack detections flash as yellow bursts.
- **Summary metrics** printed to console: C_3D (target ≥ 95 %), crack detection rate
  (target ≥ 88 %), min observed standoff margin, safety violation count (target 0),
  total gimbal tilt distance (deg·m), total flight time (s).

---

## Extensions

1. **Curved soffit (barrel vault)**: replace the flat soffit plane with
   $z_{deck}(y) = 8.0 + 0.15\cos(\pi y / L_y)$; re-derive $z_{ceil}(x,y)$ analytically and
   verify that the 3D standoff controller tracks the curved surface without oscillation.
2. **Multi-drone assignment**: assign two drones to alternate x-passes (odd/even), communicating
   their real-time positions to avoid inter-drone proximity conflicts; compare total inspection
   time and coverage overlap against the single-drone baseline.
3. **GPS-denied localisation with visual landmarks**: use detected girder edges and crossbeam
   faces as geometric landmarks in an EKF to bound lateral drift (sigma < 0.05 m) in the
   absence of GPS beneath the deck.
4. **Active lighting model**: add a downward illumination model; in shadowed regions below girder
   flanges, crack detection probability drops to P_det,shadow = 0.60; decide where to hover and
   activate an onboard spotlight to restore P_det ≥ 0.85.
5. **Viewpoint optimisation via coverage gradient**: instead of fixed boustrophedon strips,
   use a greedy viewpoint planner that at each step selects the next pose maximising the
   marginal uncovered area seen by the tilted camera, subject to the 3D standoff constraint.

---

## Related Scenarios

- Original 2D version: [S071](../S071_bridge_inspection.md)
- 3D standoff control reference: [S065 3D Scan Path](../S065_3d_scan_path.md), [S074 Mine 3D Mapping](../S074_mine_mapping.md)
- GPS-denied close-proximity control: [S064 Greenhouse Interior Flight](../S064_greenhouse.md), [S080 Underground Pipe](../S080_underground_pipe.md)
- Cooperative multi-drone inspection: [S066 Cooperative Crane](../S066_cooperative_crane.md)
- Crack detection probabilistic model: [S044 Wall Crack Inspection](../../03_environmental_sar/S044_wall_crack.md)
