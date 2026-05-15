# S044 3D Upgrade — Wall Crack Inspection

**Domain**: Environmental & SAR | **Difficulty**: ⭐⭐⭐ | **Status**: `[ ]` Not started
**Based on**: [S044 original](../S044_wall_crack.md)

---

## What Changes in 3D

The original S044 treats the inspection surface as a flat vertical plane at $x = 0$, with
$d_{standoff} = 1.5$ m held along the global $x$-axis only. The standoff vector is therefore
trivially $\hat{\mathbf{n}} = -\hat{\mathbf{x}}$, and all scan geometry reduces to a 2D
$(y, z)$ boustrophedon problem. This collapses on two real-world structures:

- **Cylindrical surfaces** (chimneys, silos, water towers): the outward normal $\hat{\mathbf{n}}$
  rotates continuously in the $(x, y)$ plane as the drone orbits — the standoff direction at
  azimuth $\phi$ is $\hat{\mathbf{n}}(\phi) = (\cos\phi,\, \sin\phi,\, 0)^{\top}$, not a fixed
  axis. A flat-wall controller that holds $x = \text{const}$ drifts off-surface immediately.
- **Multi-face buildings**: corner transitions require the drone to re-align its standoff axis
  from one wall normal to the next, introducing a 3D rotation manoeuvre between faces.

This variant adds: (1) a surface-normal standoff law that tracks $\hat{\mathbf{n}}$ on arbitrary
convex surfaces, (2) a helical boustrophedon path on a cylinder, and (3) a four-face building
perimeter scan with corner re-alignment transitions.

---

## Problem Definition

**Setup**: Two canonical structures are considered.

**Structure A — Cylindrical chimney**: radius $R_c = 4.0$ m, height $H = 20.0$ m, centred at
the origin. The drone must inspect the full exterior by orbiting at standoff
$d_{standoff} = 1.5$ m from the curved surface, i.e., at orbit radius
$r_{orbit} = R_c + d_{standoff} = 5.5$ m, while ascending in a helical boustrophedon to cover
all azimuth strips.

**Structure B — Rectangular building**: four flat faces, each $W_f \times H = 20\ \text{m}
\times 20\ \text{m}$, forming a square footprint $20 \times 20$ m. The drone inspects each face
in sequence with a standard boustrophedon, then executes a 3D corner transition to re-align to
the adjacent face normal.

**Roles**:
- **Inspection drone**: single agent, camera always aimed at the nearest surface point along
  $-\hat{\mathbf{n}}$ at standoff $d_{standoff}$; cruise speed $v_{cruise} = 2.0$ m/s.

**Objective**: Achieve $\geq 99\%$ surface coverage (area fraction) on both structures with
zero collision margin, while maximising crack detection rate across $N_c = 40$ synthetic cracks
seeded on the surfaces. Report coverage fraction per structure, total flight distance, corner
transition time, and detection rate by crack-width bin.

---

## Mathematical Model

### Surface Parameterisation

**Cylinder**: surface point at azimuth $\phi \in [0, 2\pi)$ and height $z \in [0, H]$:

$$\mathbf{s}(\phi, z) = \bigl(R_c \cos\phi,\; R_c \sin\phi,\; z\bigr)^{\top}$$

Outward unit normal at $(\phi, z)$:

$$\hat{\mathbf{n}}(\phi) = \bigl(\cos\phi,\; \sin\phi,\; 0\bigr)^{\top}$$

**Flat face $i$** of the building (normal $\hat{\mathbf{n}}_i$ constant per face, $i = 1,\ldots,4$):

$$\hat{\mathbf{n}}_i \in \left\{ \hat{\mathbf{x}},\; -\hat{\mathbf{y}},\; -\hat{\mathbf{x}},\; \hat{\mathbf{y}} \right\}$$

### Normal-Vector Standoff Control

The drone's commanded position at surface parameter $(\phi, z)$ or face parameter $(u, z)$:

$$\mathbf{p}_{cmd}(\phi, z) = \mathbf{s}(\phi, z) + d_{standoff}\, \hat{\mathbf{n}}(\phi)$$

Standoff error in the normal direction:

$$e_n(t) = d_{standoff} - \bigl(\mathbf{p}_{drone}(t) - \mathbf{s}_{nearest}(t)\bigr) \cdot \hat{\mathbf{n}}(\phi_{nearest}(t))$$

where $\mathbf{s}_{nearest}(t)$ is the closest point on the surface to the drone. A PD law
drives the error to zero:

$$\mathbf{a}_{standoff}(t) = \bigl(K_p\, e_n(t) - K_d\, \dot{e}_n(t)\bigr)\, \hat{\mathbf{n}}(\phi_{nearest}(t))$$

Critically-damped gains with $\omega_n = 4\ \text{rad/s}$:

$$K_p = \omega_n^2 = 16\ \text{s}^{-2}, \qquad K_d = 2\,\omega_n = 8\ \text{s}^{-1}$$

### Helical Boustrophedon on the Cylinder

Arc length per orbit at orbit radius $r_{orbit}$:

$$L_{orbit} = 2\pi r_{orbit}$$

Camera strip half-width (same camera parameters as S044):

$$h_{strip} = d_{standoff} \cdot \tan(\alpha),\quad \alpha = 30°$$

$$h_{strip} = 1.5 \cdot \tan(30°) \approx 0.866\ \text{m}$$

Vertical step per half-orbit (azimuth $\Delta\phi = \pi$, direction reversal):

$$\Delta z = 2\, h_{strip}\,(1 - \rho_{ov}),\quad \rho_{ov} = 0.20 \implies \Delta z \approx 1.386\ \text{m}$$

Number of helical passes to cover height $H = 20\ \text{m}$:

$$N_{pass} = \left\lceil \frac{H}{\Delta z} \right\rceil = 15$$

Drone position at helix parameter $\theta \in [0, 2\pi N_{pass}]$:

$$\mathbf{p}_{helix}(\theta) = \begin{pmatrix} r_{orbit}\cos\theta \\ r_{orbit}\sin\theta \\ z_{start} + \dfrac{\Delta z}{2\pi}\,\theta \end{pmatrix}$$

Total helix arc length:

$$L_{helix} = \int_0^{2\pi N_{pass}} \sqrt{r_{orbit}^2 + \left(\tfrac{\Delta z}{2\pi}\right)^2}\; d\theta = 2\pi N_{pass} \sqrt{r_{orbit}^2 + \left(\tfrac{\Delta z}{2\pi}\right)^2}$$

### Cylindrical Surface Coverage Metric

Discretise the cylinder surface into $N_\phi \times N_z$ cells by azimuth angle $\delta\phi$ and
height $\delta z_{grid}$. Cell $(\ell, k)$ is covered at time $t$ when the drone's image footprint
(projected onto the cylinder tangent plane at $\phi_{nearest}$) contains it:

$$\text{covered}(\ell, k) = \mathbf{1}\!\left[\exists\, t : |\Delta s_\perp(t)| \leq h_{strip} \;\wedge\; |\Delta z(t)| \leq h_{strip}\right]$$

where $\Delta s_\perp = R_c\,(\phi_{nearest}(t) - \phi_\ell)$ is the arc-length offset in the
azimuth direction.

Coverage fraction:

$$C = \frac{\#\{\text{covered cells}\}}{N_\phi \cdot N_z}$$

### Corner Transition (Multi-Face Building)

Between face $i$ and face $i+1$ the drone must rotate its standoff direction from $\hat{\mathbf{n}}_i$
to $\hat{\mathbf{n}}_{i+1}$ (a $90°$ rotation) while clearing the building corner.

The transition waypoint is placed at the building corner $\mathbf{c}_i$ offset outward by
$d_{corner} = d_{standoff} + 0.5\ \text{m}$ along the bisector of the two face normals:

$$\hat{\mathbf{b}}_i = \frac{\hat{\mathbf{n}}_i + \hat{\mathbf{n}}_{i+1}}{\|\hat{\mathbf{n}}_i + \hat{\mathbf{n}}_{i+1}\|}, \qquad \mathbf{p}_{corner,i} = \mathbf{c}_i + d_{corner}\,\hat{\mathbf{b}}_i$$

The drone holds its current height $z_{last}$ during the horizontal corner sweep, then
re-initialises the boustrophedon for face $i+1$ from $z_{start} = 0.5\ \text{m}$.

Total inter-face transition arc length (straight-line approximation):

$$L_{trans,i} \approx \left\|\mathbf{p}_{corner,i} - \mathbf{p}_{end,i}\right\| + \left\|\mathbf{p}_{start,i+1} - \mathbf{p}_{corner,i}\right\|$$

### 3D Crack Detection on Curved Surface

Crack $c$ is located at surface point $\mathbf{s}_c$ with outward normal $\hat{\mathbf{n}}_c$.
The slant range from the drone to the crack accounts for the standoff geometry:

$$r_c(t) = \bigl\|\mathbf{p}_{drone}(t) - \mathbf{s}_c\bigr\|$$

The viewing angle (angle between the camera boresight $-\hat{\mathbf{n}}(\phi_{nearest})$ and
the crack-to-drone vector) must satisfy:

$$\cos\psi_c(t) = \frac{\bigl(\mathbf{p}_{drone}(t) - \mathbf{s}_c\bigr) \cdot \hat{\mathbf{n}}_c}{r_c(t)} \geq \cos(\alpha) = \cos(30°)$$

In-frame condition (arc-length separation on curved surface):

$$\Delta s_{arc} = R_c \cdot |\phi_{drone,nearest}(t) - \phi_c| \leq h_{strip}, \qquad |\Delta z| \leq h_{strip}$$

Detection probability (same sensor model as S044):

$$P_{det}(w_c, r_c) = \Phi\!\left(\frac{w_c - w_{min}(r_c)}{\sigma_{noise}}\right), \qquad w_{min}(r) = \frac{2r\tan(\alpha)}{R_{px}} \times 1000\ \text{mm}$$

with $\sigma_{noise} = 0.05\ \text{mm}$, $R_{px} = 2048$ px.

---

## Key 3D Additions

- **Surface-normal standoff**: The PD controller tracks the surface-local outward normal
  $\hat{\mathbf{n}}(\phi)$ rather than the fixed $\hat{\mathbf{x}}$ axis, making it valid on any
  convex surface.
- **Helical boustrophedon on cylinder**: replaces the flat-plane zigzag with a continuous
  ascending helix; the vertical step per half-orbit is matched to camera strip width exactly as
  in the flat case.
- **Arc-length coverage metric**: cylindrical cells are defined by arc-length on the surface,
  not Cartesian grid cells, to avoid polar over-counting near edges.
- **Corner bisector transitions**: 3D waypoints at building corners ensure the drone clears
  structural edges without reducing standoff below the safety margin.
- **Viewing-angle gate**: on curved surfaces the camera boresight can diverge from the surface
  normal at far azimuths — an explicit $\cos\psi_c \geq \cos\alpha$ check rejects off-axis
  detections that would be invalid on a flat-wall assumption.

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Cylinder radius $R_c$ | 4.0 m |
| Cylinder height $H$ | 20.0 m |
| Building face width $W_f$ | 20.0 m |
| Building face height $H$ | 20.0 m |
| Standoff distance $d_{standoff}$ | 1.5 m |
| Corner clearance offset | 0.5 m extra over $d_{standoff}$ |
| Orbit radius (cylinder) $r_{orbit}$ | 5.5 m |
| Camera FOV | 60° |
| Strip half-width $h_{strip}$ | $\approx 0.866$ m |
| Strip overlap $\rho_{ov}$ | 20% |
| Vertical step $\Delta z$ | $\approx 1.386$ m |
| Number of passes $N_{pass}$ | 15 per face / full orbit |
| Cruise speed $v_{cruise}$ | 2.0 m/s |
| Camera resolution $R_{px}$ | 2048 px |
| Crack width range | 0.1 – 2.0 mm |
| Detection noise $\sigma_{noise}$ | 0.05 mm |
| PD gains $(K_p, K_d)$ standoff | 16 s$^{-2}$, 8 s$^{-1}$ |
| Altitude range $z$ | 0.5 – 19.5 m |
| Synthetic crack count $N_c$ | 40 per structure |
| Simulation timestep $\Delta t$ | 0.05 s |
| Coverage grid resolution | $\delta\phi = 0.5°$, $\delta z = 0.1$ m (cylinder); 0.1 m cells (flat faces) |

---

## Expected Output

- **3D helix trajectory plot**: cylinder wireframe with the helical boustrophedon path rendered
  in a colour gradient from blue (start) to red (end); detected cracks shown as yellow stars on
  the cylinder surface.
- **Altitude vs azimuth plot**: $z(\phi)$ showing the monotonically ascending helix; overlaid
  strip boundaries indicating $z_k$ pass levels; colour-coded by coverage status.
- **Surface coverage heatmap — cylinder**: unwrapped azimuth-height map $(\phi, z)$ showing
  covered cells (green), uncovered cells (white), and crack positions; coverage fraction
  annotated.
- **3D building perimeter path**: top-down and isometric views of the four-face boustrophedon
  trajectory with corner transition waypoints marked; each face rendered in a different colour.
- **Corner transition time series**: standoff error $e_n(t)$ during the $90°$ normal re-alignment
  manoeuvre; shows transient overshoot and recovery within $\leq 2$ s.
- **Detection rate comparison**: side-by-side bar charts for cylinder and building, binned by
  crack width (0–0.5, 0.5–1.0, 1.0–1.5, 1.5–2.0 mm); stacked detected vs missed.
- **Viewing-angle rejection log**: fraction of detections blocked by the $\cos\psi_c$ gate vs
  azimuth offset, showing the penalty incurred by high curvature at $R_c = 4$ m.

---

## Extensions

1. **Variable-radius structures (cooling tower)**: inspect a hyperboloid cooling tower whose
   radius $R(z)$ varies with height; update the standoff command to $r_{orbit}(z) = R(z) +
   d_{standoff}$ and recompute the helical step at each altitude.
2. **Wind turbine tower**: cylindrical tower of radius $R_c = 2.5$ m; inspect also the nacelle
   (non-cylindrical cap) requiring a viewpoint-planning step to cover the irregular surface with
   minimum revisit.
3. **Adaptive standoff on surface defects**: when a protrusion (rebar, bolt head) is detected
   by a laser rangefinder, temporarily increase $d_{standoff}$ to $2.0$ m and re-derive the
   local strip width to avoid collision while maintaining detectability.
4. **Multi-drone parallel inspection**: assign drone $k$ to azimuth sector
   $[\phi_{k-1}, \phi_k)$; use Hungarian assignment to balance sector arc lengths accounting for
   drone start positions; measure speedup vs single-drone baseline.
5. **Photogrammetric 3D reconstruction**: accumulate image footprints with associated pose
   estimates; compute the surface reconstruction completeness (point cloud density on the
   cylinder surface) and compare against the coverage fraction metric.

---

## Related Scenarios

- Original 2D version: [S044](../S044_wall_crack.md)
- Cylindrical surface coverage reference: [S062 Wind Turbine Blade Inspection](../../04_industrial_agriculture/S062_wind_turbine_blade_inspection.md)
- Multi-face building reference: [S065 Building 3D Scan Path](../../04_industrial_agriculture/S065_building_3d_scan_path.md)
- Coral reef 3D surface coverage metrics: [S053 Coral Reef 3D Mapping](../S053_coral_reef.md)
- Corner clearance manoeuvring: [S043 Confined Space Exploration](../S043_confined_space.md)
