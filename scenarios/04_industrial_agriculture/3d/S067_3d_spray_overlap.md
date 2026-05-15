# S067 3D Upgrade — Spray Overlap Optimization

**Domain**: Industrial & Agriculture | **Difficulty**: ⭐⭐⭐ | **Status**: `[ ]` Not Started
**Based on**: [S067 original](../S067_spray_overlap.md)

---

## What Changes in 3D

The original S067 treats the field as a perfectly flat, horizontal plane at a fixed spraying altitude $z_{spray} = \text{const}$. The spray cone half-angle $\theta_c$ and effective spray width $w_{spray} = 2\,z_{spray}\tan\theta_c$ are therefore constant across every strip. GPS error is the only source of dose non-uniformity; terrain relief is ignored entirely.

In a real agricultural setting three critical factors are neglected:

1. **Altitude-varying cone angle**: if the drone climbs or descends (e.g., to follow a headland slope or avoid an obstacle), the footprint diameter changes proportionally to $z$. A fixed strip spacing $d$ tuned for one altitude causes under- or over-coverage at other altitudes.
2. **Terrain-following**: undulating fields require the drone to adjust its AGL (above-ground-level) altitude along each strip in order to maintain a constant effective spray width. This introduces a coupled trajectory-planning and dose-uniformity problem that cannot be captured in 2D.
3. **3D overlap map on an undulating surface**: the dose accumulation surface is no longer a flat $x$-$y$ grid; it must be computed on the actual terrain mesh $z = h(x,y)$, and the Gaussian footprint must be projected onto the local slope at each grid point.

This 3D upgrade adds all three elements and reformulates the CV minimisation problem as a function of both strip spacing $d$ and terrain-following controller gain $k_{tf}$.

---

## Problem Definition

**Setup**: A single agricultural drone is tasked with spraying a $50 \times 50$ m field whose
terrain elevation follows a smooth undulating surface $h(x,y)$ (e.g., a bi-sinusoidal hill with
amplitude $A_{hill} = 1.5$ m). The drone flies east-west strips at a commanded AGL altitude
$z_{AGL}^{cmd} = 2.0$ m above the local terrain. A terrain-following controller adjusts the
drone's inertial altitude in real time. The boom sprayer produces a symmetric conical spray
pattern with half-angle $\theta_c = 26.6°$ (nominally $\tan\theta_c = 0.5$, giving
$w_{spray} = 2\,z_{AGL}\tan\theta_c = 2.0$ m at $z_{AGL} = 2.0$ m).

**Roles**:
- **Drone**: single UAV flying pre-planned parallel strips with active terrain-following; boom
  sprayer always active.
- **Field surface**: a $50 \times 50$ m undulating terrain mesh $h(x,y)$ discretised at
  $0.1 \times 0.1$ m; each surface cell accumulates dose from passing strips.

**Objective**: Find the joint optimum $(d^*, k_{tf}^*)$ that minimises the Coefficient of
Variation (CV) of the dose accumulated on the 3D terrain surface. Compare four configurations:

1. **2D baseline**: flat field, fixed $z = 2.0$ m, no terrain-following (original S067).
2. **3D fixed-altitude**: undulating field, drone flies at constant inertial altitude; spray
   footprint expands and contracts with terrain clearance.
3. **3D terrain-following (P-controller)**: drone adjusts $z$ to track $z_{AGL}^{cmd}$ via a
   proportional altitude controller; quantify residual AGL error.
4. **3D terrain-following + adaptive strip spacing**: strip spacing is locally widened or
   narrowed by $\delta d = \alpha_{slope}\, |\nabla h|$ to compensate for effective footprint
   dilation on cross-slopes.

**Key questions**: How much does 1.5 m of terrain relief degrade CV compared to the flat case?
What terrain-following bandwidth is sufficient to recover flat-field CV? Does adaptive strip
spacing provide additional benefit beyond terrain-following?

---

## Mathematical Model

### Terrain Model

The undulating field elevation is:

$$h(x,y) = A_{hill} \left[\sin\!\left(\frac{2\pi x}{\lambda_x}\right)\cos\!\left(\frac{2\pi y}{\lambda_y}\right)\right]$$

where $A_{hill} = 1.5$ m, $\lambda_x = \lambda_y = 30$ m. The terrain gradient vector is:

$$\nabla h(x,y) = A_{hill} \begin{bmatrix} \frac{2\pi}{\lambda_x}\cos\!\left(\frac{2\pi x}{\lambda_x}\right)\cos\!\left(\frac{2\pi y}{\lambda_y}\right) \\ -\frac{2\pi}{\lambda_y}\sin\!\left(\frac{2\pi x}{\lambda_x}\right)\sin\!\left(\frac{2\pi y}{\lambda_y}\right) \end{bmatrix}$$

The local terrain slope angle is $\phi(x,y) = \arctan\!\left(\|\nabla h(x,y)\|\right)$.

### Terrain-Following Altitude Controller

The drone inertial altitude command at along-track position $y$ for strip $k$ centred at
cross-track position $x_k$:

$$z_{cmd}(x_k, y) = h(x_k, y) + z_{AGL}^{cmd}$$

A first-order lag model represents the onboard altitude controller bandwidth $\omega_{tf}$:

$$\dot{z}_{drone}(t) = \omega_{tf}\bigl[z_{cmd}(t) - z_{drone}(t)\bigr]$$

The residual AGL tracking error is:

$$\varepsilon_{AGL}(x_k, y) = z_{drone}(x_k, y) - h(x_k, y) - z_{AGL}^{cmd}$$

### Altitude-Varying Spray Footprint

At each spraying event at drone position $(x_k, y, z_{drone})$ above terrain point
$(x_k, y, h(x_k,y))$, the actual AGL clearance is:

$$z_{AGL}(x_k, y) = z_{drone}(x_k, y) - h(x_k, y)$$

The effective spray half-width at that moment is:

$$r_{spray}(x_k, y) = z_{AGL}(x_k, y) \tan\theta_c$$

so $w_{spray}(x_k, y) = 2\,r_{spray}(x_k, y)$ and the local Gaussian sigma is:

$$\sigma_s(x_k, y) = \frac{w_{spray}(x_k, y)}{4} = \frac{z_{AGL}(x_k, y)\tan\theta_c}{2}$$

### 3D Dose Accumulation on the Terrain Surface

The dose deposited on terrain cell $(x, y)$ by strip $k$ is the Gaussian footprint projected
onto the local surface. The effective cross-track offset must account for the slope geometry.
Define the horizontal cross-track distance from the drone nadir to cell $(x, y)$:

$$\Delta x_k(x) = x - x_k - \varepsilon_k$$

where $\varepsilon_k \sim \mathcal{N}(0, \sigma_{gps}^2)$ is the GPS lateral error. The
slope-corrected dose contribution from strip $k$ is:

$$q_k(x, y) = \frac{Q_0}{\cos\phi(x,y)} \exp\!\left(-\frac{\Delta x_k(x)^2}{2\,\sigma_s(x_k, y)^2}\right)$$

The $\cos^{-1}\phi$ factor accounts for the larger surface area of a sloped cell relative to its
horizontal projection. The total accumulated dose on cell $(x,y)$ is:

$$D(x,y) = \sum_{k=0}^{N-1} q_k(x, y)$$

### 3D Coverage Uniformity (CV)

The CV of the 3D dose map is:

$$\text{CV}_{3D}(d, k_{tf}) = \frac{\sigma_D}{\mu_D}$$

where $\mu_D$ and $\sigma_D$ are computed over all terrain surface cells, weighted by cell
surface area $dA = dx\,dy\,/\cos\phi$:

$$\mu_D = \frac{\sum_{i,j} D(x_i, y_j)\,/\cos\phi_{ij}}{\sum_{i,j} 1/\cos\phi_{ij}}$$

### Adaptive Strip Spacing

On a cross-slope of angle $\phi$, the spray footprint appears compressed in the horizontal
plane by a factor $\cos\phi$. To maintain constant overlap, the strip spacing is locally widened:

$$d_{eff}(x_k) = d \cdot \cos\phi(x_k, y_{mid}) + \alpha_{slope}\,|\nabla h(x_k, y_{mid})|^{-1} \cdot d$$

A simpler first-order correction that generalises S067's flat-field result is:

$$d_{eff}(x_k) = d^* \cdot \frac{w_{spray}(x_k, y_{mid})}{w_{spray}^{nom}}
= d^* \cdot \frac{z_{AGL}(x_k, y_{mid})}{z_{AGL}^{cmd}}$$

This scales the strip spacing in proportion to the actual footprint width at the strip's
representative midpoint altitude.

### Optimal Spacing in 3D

The 3D optimal strip spacing generalises the flat-field result. For a terrain-following drone
with perfect AGL tracking ($\varepsilon_{AGL} = 0$), $\sigma_s$ is constant and $d^*$ matches
the flat-field optimum. For imperfect tracking, the effective dose sigma becomes location-
dependent and the CV minimisation must be solved over the full 2D field:

$$d^* = \underset{d}{\arg\min}\;\text{CV}_{3D}(d, k_{tf})$$

A practical sweep over $d \in [0.5\,\text{m},\; 4.0\,\text{m}]$ and
$k_{tf} \in [0.5\,\text{s}^{-1},\; 5.0\,\text{s}^{-1}]$ (bandwidth grid) yields a 2D CV heat
map $\text{CV}_{3D}(d, k_{tf})$ from which the joint optimum is read off.

### Strip Count and Mission Path Length on Undulating Terrain

The along-track path length of strip $k$ must integrate over the terrain profile:

$$L_k = \int_0^{L} \sqrt{1 + \left(\frac{\partial h(x_k, y)}{\partial y}\right)^2}\, dy$$

Total mission path length:

$$L_{total} = \sum_{k=0}^{N-1} L_k + (N-1)\,d$$

On the bi-sinusoidal terrain with $A_{hill} = 1.5$ m and $\lambda = 30$ m, the path length
overhead is approximately $1$–$3\%$ compared to the flat-field value.

---

## Key 3D Additions

- **Altitude-varying spray sigma**: $\sigma_s(x_k, y) = z_{AGL}(x_k, y)\tan\theta_c / 2$ replaces the constant $\sigma_s = 0.5$ m of the 2D model.
- **Terrain-following controller**: first-order lag $\dot{z} = \omega_{tf}(z_{cmd} - z)$ introduces a tunable bandwidth parameter $\omega_{tf}$ and residual AGL error $\varepsilon_{AGL}$.
- **Slope-corrected dose accumulation**: the $\cos^{-1}\phi$ factor projects the Gaussian footprint correctly onto the sloped terrain surface.
- **3D overlap map**: dose is accumulated on a 2D terrain mesh $h(x,y)$ instead of a flat grid; the heat map is rendered as a 3D surface coloured by dose.
- **Adaptive strip spacing**: local strip spacing adjusted in proportion to $z_{AGL}(x_k, y_{mid}) / z_{AGL}^{cmd}$ to compensate for footprint dilation on high terrain.
- **2D CV heat map** over $(d, \omega_{tf})$: reveals the joint optimum and shows that terrain-following bandwidth is as important as strip spacing for uniformity.
- **Altitude bound enforcement**: $z_{drone} \in [1.0, 8.0]$ m (safe minimum clearance above highest terrain point + AGL target).

---

## Key Parameters

| Parameter | Symbol | Value |
|-----------|--------|-------|
| Field size | $W \times L$ | $50 \times 50$ m |
| Terrain amplitude | $A_{hill}$ | 1.5 m |
| Terrain wavelength | $\lambda_x = \lambda_y$ | 30 m |
| Nominal AGL altitude | $z_{AGL}^{cmd}$ | 2.0 m |
| Spray cone half-angle | $\theta_c$ | 26.6° ($\tan\theta_c = 0.5$) |
| Nominal spray width | $w_{spray}^{nom}$ | 2.0 m |
| Nominal spray sigma | $\sigma_s^{nom}$ | 0.5 m |
| Nominal strip spacing | $d^*$ (flat-field) | 1.80 m |
| Terrain-following bandwidth range | $\omega_{tf}$ | 0.5 – 5.0 s$^{-1}$ |
| GPS positioning error | $\sigma_{gps}$ | 0.1 m |
| Dose grid resolution | — | 0.1 × 0.1 m |
| Monte Carlo trials | $M_{MC}$ | 100 |
| Altitude bounds | $z_{drone}$ | 1.0 – 8.0 m |
| Drone cruise speed | $v$ | 3.0 m/s |
| Strip spacing search range | $[d_{min}, d_{max}]$ | [0.5, 4.0] m |
| Redundancy threshold | — | $1.5 \times \mu_D$ |

---

## Expected Output

- **3D terrain surface + dose map**: Matplotlib 3D surface plot of $h(x,y)$ coloured by
  accumulated dose $D(x,y)$; four panels for the four configurations (2D baseline, fixed
  altitude, terrain-following, adaptive spacing); colour scale in normalised dose units.
- **CV heat map over $(d, \omega_{tf})$**: 2D colour map showing $\text{CV}_{3D}$ as a function
  of strip spacing (x-axis) and terrain-following bandwidth (y-axis); joint optimum marked;
  reveals the trade-off between controller bandwidth and spacing choice.
- **Altitude time series per strip**: $z_{drone}(y)$ and $z_{cmd}(y)$ plotted together for a
  representative strip crossing a hill; shaded band shows residual AGL error $\varepsilon_{AGL}$;
  demonstrates the lag effect at low $\omega_{tf}$.
- **Cross-track dose profiles**: normalised $D(x, y_{mid})/\mu_D$ at the field midline for all
  four configurations on a single axis; flat-field result shown as dashed reference; shows how
  terrain relief without following inflates CV and how adaptive spacing recovers uniformity.
- **CV vs strip spacing curves**: four overlaid curves (one per configuration); the 3D
  terrain-following curve shifts the optimal $d^*$ slightly relative to the flat-field
  minimum; adaptive spacing further flattens the CV trough (more robust to spacing error).
- **Redundant area map on terrain**: top-down heatmap flagging cells where
  $D(x,y) > 1.5\,\mu_D$; terrain contours overlaid to show spatial correlation between
  over-dosing and terrain highs (reduced AGL) or lows (increased AGL).
- **Animation (GIF)**: 3D view of dose accumulating strip by strip on the terrain surface;
  drone position shown as a red marker; terrain-following altitude correction visible in the
  drone's z trajectory; title updates with current CV.

---

## Extensions

1. **LiDAR-based terrain preview**: give the drone a forward-looking terrain sensor with look-
   ahead distance $d_{LA}$; compute the optimal $d_{LA}$ that minimises residual AGL error
   for a given flight speed $v$ and $\omega_{tf}$.
2. **Wind drift on slopes**: on a cross-slope, a constant crosswind $v_w$ displaces the spray
   plume both laterally and vertically (relative to the sloped surface); derive the combined
   drift correction and evaluate how slope angle interacts with wind angle of attack.
3. **Multi-drone terrain-following fleet**: extend to a two-drone parallel-strip mission on
   the same undulating field; ensure inter-drone collision avoidance respects minimum 3D
   separation while both execute terrain-following; assign strips using the slope-corrected
   path length as the balancing criterion for the Hungarian assignment.
4. **Variable spray pressure (flow control)**: model the boom nozzle flow rate as
   $Q = c_v \sqrt{\Delta P}$; instead of varying strip spacing, hold $d$ fixed and vary
   $\Delta P$ to compensate for footprint dilation; compare CV achieved by spacing vs
   pressure control.
5. **Photogrammetric validation model**: compute the ground sampling distance (GSD) on the
   sloped terrain as a by-product of the altitude data; assess whether the spray-optimised
   altitude profile also satisfies photogrammetric resolution requirements for plant health
   imaging ($\text{GSD} \leq 2$ cm/px).

---

## Related Scenarios

- Original 2D version: [S067 Spray Coverage Overlap Optimization](../S067_spray_overlap.md)
- Terrain-following context: [S063 Crop Row Following](../S063_crop_row_following.md) (strip
  following fundamentals applied to agricultural rows)
- Multi-drone extension: [S068 Multi-Drone Field Coverage](../S068_multi_drone_field_coverage.md)
  (parallel strip assignment across a fleet; add terrain-following to each agent)
- Coverage planning reference: [S048 Full-Area Coverage Scan](../../../scenarios/03_environmental_sar/S048_lawnmower.md)
  (boustrophedon strip planning; 3D terrain extension is analogous)
- Truly 3D format reference: [S002 3D Upgrade](../../../scenarios/01_pursuit_evasion/3d/S002_3d_evasive_maneuver.md)
