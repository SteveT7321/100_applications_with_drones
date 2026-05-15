# S052 3D Upgrade — Glacier Area Monitoring

**Domain**: Environmental & SAR | **Difficulty**: ⭐⭐⭐ | **Status**: `[ ]` Not Started
**Based on**: [S052 original](../S052_glacier.md)

---

## What Changes in 3D

The original S052 flies the survey drone at a single fixed altitude $h = 80$ m above an implicit
flat reference plane (`z = const` throughout). The glacier surface elevation is never modelled —
the boustrophedon grid is planned entirely in the x-y plane, and the camera footprint formula
assumes a constant slant range equal to $h$. On a real alpine glacier this causes three critical
failures: (1) terrain relief of 50–200 m across an 800 × 600 m glacier changes the actual GSD
by up to $\pm$40%, breaking the photogrammetric overlap guarantee; (2) lateral ice-cliff faces
(typical height 10–40 m) are invisible to a nadir-pointing camera; (3) the planar NDWI mosaic
provides no volumetric information, so mass-balance estimation requires an external DEM.

This 3D variant replaces the flat-plane model with a synthetic DEM, adapts survey altitude in
real time to maintain a constant AGL (above-ground-level) clearance, inserts oblique inspection
passes for ice-cliff faces, and derives a volumetric mass-balance estimate from the reconstructed
3D point cloud.

---

## Problem Definition

**Setup**: A survey drone operates over a mountain glacier whose surface is described by a
synthetic DEM $z_{dem}(x, y)$ combining a large-scale planar ramp (elevation gain $\Delta z_{ramp}$
across the glacier length) and superimposed Gaussian mounds representing crevasse ridges and
accumulation zones. The glacier footprint is $L_{area} \times W_{area} = 800 \times 600$ m with
an elevation range $[z_{base},\; z_{base} + \Delta z_{ramp}]$, where $z_{base} = 2{,}800$ m and
$\Delta z_{ramp} = 180$ m. Ice-cliff segments of height $h_{cliff} = 20$–$40$ m are located at
the glacier terminus and along the lateral margins. The drone flies:

1. A **terrain-following nadir pass** — the boustrophedon grid from S052, but with altitude
   continuously adjusted so that AGL clearance $h_{AGL} = 80$ m is maintained above the DEM,
   guaranteeing constant GSD across the full elevation range.
2. **Oblique cliff passes** — additional flight legs parallel to each ice-cliff face at a
   stand-off distance of 30 m and a camera pitch angle of 70° from nadir, providing image
   coverage of vertical surfaces.

**Roles**:
- **Survey drone**: single multirotor carrying a nadir + oblique dual-camera head; transitions
  between terrain-following strips and cliff inspection legs autonomously.

**Objective**: Maintain $\text{GSD} \leq 5$ cm/px and forward overlap $O_f \geq 80\%$ across
both horizontal and vertical surfaces; estimate the glacier volume $V_{glacier}$ from the
reconstructed DEM; compute a volumetric mass-balance proxy $\Delta V$ relative to a reference
survey; and map melt-water extent with NDWI.

**Comparison strategies**:
1. **Flat-plane boustrophedon** (S052 baseline): fixed $z$, no cliff passes — control group.
2. **Terrain-following nadir only**: AGL-constant altitude, boustrophedon grid, no cliff passes.
3. **Full 3D survey**: terrain-following nadir pass + oblique cliff inspection legs.

---

## Mathematical Model

### DEM Model

The synthetic glacier DEM is the sum of a linear ramp and $K$ Gaussian mounds:

$$z_{dem}(x, y) = z_{base} + \frac{\Delta z_{ramp}}{L_{area}} \cdot x
  + \sum_{k=1}^{K} A_k \exp\!\left(-\frac{(x - x_k)^2}{2\sigma_{x,k}^2}
    - \frac{(y - y_k)^2}{2\sigma_{y,k}^2}\right)$$

where $(x_k, y_k)$, $A_k$, and $(\sigma_{x,k}, \sigma_{y,k})$ are the centre, amplitude, and
spread of ridge $k$.

### Terrain-Following Altitude Command

At each horizontal waypoint $(x, y)$ along the boustrophedon path the commanded drone altitude is:

$$z_{cmd}(x, y) = z_{dem}(x, y) + h_{AGL}$$

The drone tracks $z_{cmd}$ with a first-order lag to model vertical speed limits:

$$\dot{z}(t) = \text{clip}\!\left(\frac{z_{cmd}(t) - z(t)}{\tau_z},\; -v_{z,max},\; v_{z,max}\right)$$

where $\tau_z = 1.0$ s is the altitude-control time constant and $v_{z,max} = 3.0$ m/s.

### Altitude-Adaptive GSD

Because the true AGL clearance may deviate from the nominal $h_{AGL}$ during rapid terrain
changes, the instantaneous GSD at trigger $k$ is:

$$\text{GSD}_k = \frac{(z_k - z_{dem}(x_k, y_k)) \cdot p}{f}$$

where $p = s_w / n_w$ is the pixel pitch (mm) and $f$ is the focal length (mm). The effective
camera footprint dimensions at trigger $k$:

$$W_{fp,k} = \frac{(z_k - z_{dem}(x_k, y_k)) \cdot s_w}{f}, \quad
  L_{fp,k} = \frac{(z_k - z_{dem}(x_k, y_k)) \cdot s_h}{f}$$

Strip spacing and base distance are computed using the nominal $h_{AGL}$ but overlap is verified
against $\text{GSD}_k$ at every trigger.

### 3D Camera Footprint on Sloped Terrain

For a nadir camera over a locally sloped terrain patch with surface normal
$\hat{\mathbf{n}} = (-\partial_x z_{dem},\; -\partial_y z_{dem},\; 1)^T / \|\cdot\|$,
the footprint is no longer a rectangle but a parallelogram. The four corner rays from the
camera optical centre $\mathbf{c}_k$ at half-angles $\pm\theta_h$, $\pm\theta_v$ are:

$$\mathbf{r}_{ij} = \mathbf{c}_k + t_{ij} \cdot \mathbf{d}_{ij}, \quad
t_{ij} = -\frac{\hat{\mathbf{n}} \cdot (\mathbf{c}_k - \mathbf{q})}{\hat{\mathbf{n}} \cdot \mathbf{d}_{ij}}$$

where $\mathbf{d}_{ij}$ are the four corner ray unit vectors and $\mathbf{q}$ is any point on the
terrain patch. The actual overlap of two consecutive footprints on sloped terrain is:

$$O_{actual} = 1 - \frac{d_{along}}{\min(L_{fp,k},\; L_{fp,k+1})}$$

where $d_{along}$ is the along-slope distance between adjacent nadir points projected onto the
terrain surface.

### Oblique Ice-Cliff Inspection

For a cliff face approximated as a vertical plane at position $x = x_{cliff}$, the drone flies
a horizontal leg at stand-off distance $d_s = 30$ m from the cliff face at altitude
$z_{mid} = z_{cliff,base} + h_{cliff}/2$. The camera is pitched by angle $\psi_{pitch} = 70°$
from nadir (i.e., $20°$ from horizontal). The slant range from drone to cliff-face centre:

$$R_{slant} = \frac{d_s}{\cos\psi_{pitch}}$$

The GSD on the cliff face:

$$\text{GSD}_{cliff} = \frac{R_{slant} \cdot p}{f}$$

The along-cliff footprint height (vertical extent) and along-leg footprint width:

$$H_{fp,cliff} = \frac{R_{slant} \cdot s_h}{f}, \quad W_{fp,cliff} = \frac{R_{slant} \cdot s_w}{f}$$

Trigger interval along the cliff leg to maintain $O_f = 80\%$:

$$B_{x,cliff} = W_{fp,cliff} \cdot (1 - O_f)$$

### Volumetric Mass-Balance Estimation

The glacier volume relative to a flat basal plane at $z_{base}$ is approximated by integrating
the DEM over the glacier footprint on a grid of cell size $\delta$:

$$V_{glacier} = \delta^2 \sum_{i,j} \max\!\bigl(z_{dem}(x_i, y_j) - z_{base},\; 0\bigr)$$

A mass-balance proxy between a current survey DEM $z_{dem}^{(2)}$ and a reference DEM
$z_{dem}^{(1)}$ (separated by a notional melt period):

$$\Delta V = \delta^2 \sum_{i,j} \bigl(z_{dem}^{(2)}(x_i, y_j) - z_{dem}^{(1)}(x_i, y_j)\bigr)$$

Ice mass loss (assuming ice density $\rho_{ice} = 917$ kg/m³):

$$\Delta M = \rho_{ice} \cdot \Delta V$$

### 3D Flight Distance and Time

Total 3D flight distance including altitude changes:

$$D_{3D} = \sum_{k=1}^{N_{steps}-1} \|\mathbf{p}_{k+1} - \mathbf{p}_k\|_3$$

where $\mathbf{p}_k = (x_k, y_k, z_k)^T$. The overhead of terrain-following over the flat-plane
baseline:

$$\Delta D_{TF} = D_{3D} - D_{flat}$$

$$\Delta D_{TF} = \sum_{k} \sqrt{(v \Delta t)^2 + (\Delta z_k)^2} - v \Delta t$$

This extra distance is dominated by the vertical speed constraint during steep terrain transitions.

### NDWI Melt-Area Estimation

Identical to S052 (applied to the terrain-corrected nadir mosaic):

$$\text{NDWI}(u,v) = \frac{\rho_G(u,v) - \rho_{NIR}(u,v)}{\rho_G(u,v) + \rho_{NIR}(u,v)}, \quad
A_{melt} = \sum_{u,v} \mathbf{1}[\text{NDWI}(u,v) > 0.2] \cdot \text{GSD}^2$$

---

## Key 3D Additions

- **DEM surface model**: synthetic ramp + Gaussian mounds representing real glacier topography
  with $\pm 90$ m relief; controls all altitude commands and GSD verification.
- **Terrain-following altitude law**: first-order lag controller tracking $z_{cmd}(x,y) = z_{dem}(x,y) + h_{AGL}$
  with $v_{z,max} = 3$ m/s, accumulating 3D path length correctly.
- **Altitude-adaptive GSD verification**: $\text{GSD}_k$ computed at every camera trigger from
  actual AGL; triggers flagged if GSD exceeds the 5 cm/px threshold during rapid elevation gain.
- **Oblique cliff inspection legs**: separate flight segments at $\psi_{pitch} = 70°$ covering
  vertical ice-cliff faces; slant-range GSD and footprint geometry derived independently.
- **Volumetric mass-balance proxy**: DEM integration over the glacier footprint yields $V_{glacier}$;
  difference between two simulated survey epochs gives $\Delta V$ and $\Delta M$.
- **3D trajectory visualisation**: full $(x, y, z)$ path with terrain surface rendered as a
  translucent mesh; altitude time series subplot showing terrain profile and drone AGL.

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| DEM ramp elevation gain $\Delta z_{ramp}$ | 180 m |
| Base elevation $z_{base}$ | 2 800 m |
| DEM Gaussian ridges $K$ | 6 |
| Nominal AGL clearance $h_{AGL}$ | 80 m |
| Altitude control time constant $\tau_z$ | 1.0 s |
| Max vertical speed $v_{z,max}$ | 3.0 m/s |
| Glacier area | 800 × 600 m |
| DEM grid resolution $\delta$ | 5 m |
| Focal length $f$ | 25 mm |
| Sensor size | 17.3 × 13.0 mm |
| Image resolution | 4 608 × 3 456 px |
| Nominal GSD at $h_{AGL}$ | $\approx$ 4.8 cm/px |
| GSD limit | 5 cm/px |
| Forward overlap $O_f$ | 80% |
| Side overlap $O_s$ | 60% |
| Ice-cliff height $h_{cliff}$ | 20–40 m |
| Cliff stand-off distance $d_s$ | 30 m |
| Camera pitch for cliff $\psi_{pitch}$ | 70° from nadir |
| Slant-range GSD on cliff | $\approx$ 8.4 cm/px |
| Drone cruise speed $v$ | 10 m/s |
| Ice density $\rho_{ice}$ | 917 kg/m³ |
| z altitude bounds | [2 780, 3 060] m |

---

## Expected Output

- **3D terrain-following trajectory**: translucent DEM mesh with the drone's 3D boustrophedon
  path rendered in red and cliff inspection legs in orange; camera trigger positions shown as
  dots coloured by instantaneous GSD (green = within spec, red = exceeds 5 cm/px).
- **Altitude vs distance profile**: along-track plot showing $z_{dem}(x,y)$ (brown filled),
  $z_{cmd}$ (dashed), and actual drone altitude $z(t)$ (blue), with shaded AGL clearance band.
- **GSD variation map**: 2D plan view of the glacier area heat-mapped by $\text{GSD}_k$ at
  each trigger; ideal flat-plane GSD annotated as reference; GSD exceedance zones highlighted.
- **Coverage count heatmap (3D-corrected)**: rasterised coverage grid using sloped terrain
  footprints; minimum and mean coverage values compared to flat-plane S052 baseline.
- **Synthetic NDWI mosaic + melt classification**: same output as S052 but computed on the
  terrain-corrected mosaic; $A_{melt}$ in m² annotated.
- **Volumetric mass-balance bar chart**: $V_{glacier}$ (m³) for the reference epoch and current
  epoch side by side; $\Delta V$ (m³) and $\Delta M$ (tonnes) annotated.
- **Strategy comparison table**: flat-plane vs terrain-following vs full-3D — total 3D flight
  distance, survey time, GSD exceedance count, and cliff face coverage percentage.

---

## Extensions

1. **RRT*-based cliff approach**: replace the straight stand-off approach with an RRT*-planned
   path that avoids entering the cliff face boundary while minimising inspection leg length.
2. **Multi-pass seasonal change detection**: simulate two survey epochs (pre-melt and post-melt)
   with different DEM realizations; compute $\Delta V$ and map spatially the elevation-change
   $\delta z(x,y) = z^{(2)} - z^{(1)}$.
3. **Crevasse-safe altitude floor**: add a crevasse mask to the DEM where $z_{cmd}$ is raised
   by an additional 20 m to prevent the drone entering a crevasse shadow; evaluate impact on
   GSD in crevasse zones.
4. **Wind disturbance model**: add a lateral wind field $\mathbf{w}(x,y,z)$ that displaces
   the nadir point from the planned strip centreline; evaluate cross-track footprint registration
   error and minimum wind speed that breaks the 60% side-overlap requirement.
5. **Photogrammetric accuracy model**: propagate GSD variation and overlap statistics through
   a simplified SfM error model to estimate 3D point-cloud accuracy as a function of terrain
   slope; identify the slope threshold above which the 5 cm/px GSD spec can no longer be met.

---

## Related Scenarios

- Original 2D version: [S052 Glacier Melt Area Measurement](../S052_glacier.md)
- Terrain-following reference: [S050 SLAM-Aided Inspection](../S050_slam.md)
- Volumetric reconstruction cross-reference: [S053 Coral Reef 3D Mapping](../S053_coral_reef.md)
- Industrial 3D scan analogue: [S065 Building 3D Scan Path](../../04_industrial_agriculture/S065_building_3d_scan_path.md)
- Coverage planning foundation: [S048 Lawnmower Coverage](../S048_lawnmower.md)
