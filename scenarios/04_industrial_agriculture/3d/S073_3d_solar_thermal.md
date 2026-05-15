# S073 3D Upgrade — Solar Panel Thermal Inspection

**Domain**: Industrial & Agriculture | **Difficulty**: ⭐⭐ | **Status**: `[ ]` Not Started
**Based on**: [S073 original](../S073_solar_thermal.md)

---

## What Changes in 3D

The original S073 locks the drone at a fixed altitude `h = 5.0 m` throughout the entire mission. The boustrophedon path is generated in the x-y plane only; the thermal camera is assumed strictly nadir-pointing, and neither panel tilt nor solar glare enter the model. Three limitations follow directly from this:

1. **No panel-tilt geometry**: real utility-scale solar arrays are tilted at an elevation angle $\alpha_{tilt}$ (typically 20–35°, facing south) to maximise annual irradiance. A nadir camera flying at constant altitude captures an oblique projection of each tilted panel, distorting the thermal footprint shape and introducing altitude-dependent parallax error.
2. **No glare avoidance**: when the drone's viewing angle is close to the specular-reflection angle of the panel surface, reflected sunlight floods the thermal sensor and masks genuine hot spots. The specular condition depends on drone altitude, along-track position, and sun elevation — a purely 2D model cannot represent this.
3. **No inter-row obstacle clearance**: row-to-row transit legs in the original model are horizontal at fixed altitude. In reality the drone must clear panel edges and support-frame structures when crossing between rows; the minimum safe transit altitude depends on the panel height profile along the transition segment.

This upgrade adds: (a) full 3D panel-array geometry with tilt and row-gap structure; (b) a glare-free altitude band derived from specular-reflection geometry; (c) per-strip altitude optimisation that balances footprint width, GSD, and specular exclusion; (d) 3D transition arcs between strips that clear physical obstacles; and (e) a shadow-mask that suppresses hot-spot detection when a panel falls in its own or a neighbour's cast shadow.

---

## Problem Definition

**Setup**: A single drone inspects an $80 \times 60$ m solar farm containing 120 panels in 10 rows of 12. Each panel is tilted at elevation angle $\alpha_{tilt} = 25°$ facing due south (positive y-direction). Panels in a row share a common tilt axis running east-west; adjacent rows are separated by a ground gap $g_{row} = 2.0$ m used for maintenance access. The drone carries a thermal camera gimbal that can pitch to point perpendicular to the panel surface (tilt-normal pointing), eliminating the oblique-projection distortion present in a fixed nadir camera.

The sun is modelled at a fixed azimuth and elevation appropriate for a mid-morning inspection pass (azimuth $\phi_{sun} = 120°$ east of north, elevation $\varepsilon_{sun} = 35°$). The specular exclusion zone is computed from Snell's law of reflection for each panel surface; the drone must remain outside this zone at all times to avoid glare contamination.

Flight altitude now varies per strip: scan legs over panel surfaces fly at $h_{scan}$ chosen to keep the camera within the glare-free band; row-crossing transition legs climb to $h_{transit}$ to clear panel top edges plus a safety margin.

**Roles**:
- **Drone**: single UAV executing a 3D boustrophedon path; altitude and gimbal pitch are controlled jointly per strip.
- **Solar farm**: $80 \times 60$ m array; panels tilted at $\alpha_{tilt} = 25°$; row gap $g_{row} = 2.0$ m; panel top-edge height $h_{edge}(y)$ varies with row position.
- **Thermal camera**: gimbal-stabilised; pitch angle $\beta_{cam}$ tracks the panel-normal direction; effective FOV on the panel surface is $\theta_{FOV} = 45°$; glare contamination suppresses detection when the viewing vector falls within $\pm 5°$ of the specular direction.
- **Shadow model**: cast-shadow mask derived from sun geometry; panels in full shadow are not inspectable during that pass.

**Objective**: fly the complete 3D inspection path and report:

1. **Detection rate** $R_{det}$ — fraction of the $M = 8$ defective panels correctly identified under glare and shadow constraints.
2. **Glare-free coverage** $C_{gf}$ — fraction of the farm area inspected without specular contamination.
3. **Shadow-excluded area** $A_{shadow}$ — total panel area (m²) masked by cast shadows at the chosen inspection time.
4. **Coverage time** $T_{cov}$ (s) — total mission time including vertical transition legs.
5. **Path length** $L_{3D}$ (m) — full 3D arc length of the drone trajectory.

---

## Mathematical Model

### 3D Panel Array Geometry

Each panel $p_{ij}$ (row $i$, column $j$) has four corners. With the tilt axis running along $\hat{\mathbf{x}}$ (east-west), the panel-surface normal in world coordinates is:

$$\hat{\mathbf{n}}_{panel} = \begin{bmatrix} 0 \\ -\sin\alpha_{tilt} \\ \cos\alpha_{tilt} \end{bmatrix}$$

The bottom edge of row $i$ sits at ground-level y-coordinate:

$$y_i^{bot} = i \cdot (W_{panel} \cos\alpha_{tilt} + g_{row})$$

The top-edge height (z) of row $i$ is:

$$h_{edge,i} = W_{panel} \sin\alpha_{tilt}$$

where $W_{panel} = FARM\_W / N_{rows}$ is the panel width before projection. The 3D centre of panel $p_{ij}$ is:

$$\mathbf{c}_{ij} = \begin{bmatrix} (j + \tfrac{1}{2}) L_{panel} \\ y_i^{bot} + \tfrac{1}{2} W_{panel} \cos\alpha_{tilt} \\ \tfrac{1}{2} W_{panel} \sin\alpha_{tilt} \end{bmatrix}$$

### Specular-Reflection Glare Model

The incident solar direction unit vector (from the sun toward the panel) is:

$$\hat{\mathbf{s}} = \begin{bmatrix} -\sin\phi_{sun}\cos\varepsilon_{sun} \\ -\cos\phi_{sun}\cos\varepsilon_{sun} \\ -\sin\varepsilon_{sun} \end{bmatrix}$$

The specular-reflection direction off the panel surface is:

$$\hat{\mathbf{r}}_{spec} = \hat{\mathbf{s}} - 2(\hat{\mathbf{s}} \cdot \hat{\mathbf{n}}_{panel})\hat{\mathbf{n}}_{panel}$$

The drone's viewing direction from position $\mathbf{p}_d$ toward panel centre $\mathbf{c}_{ij}$ is:

$$\hat{\mathbf{v}}_{view} = \frac{\mathbf{c}_{ij} - \mathbf{p}_d}{\|\mathbf{c}_{ij} - \mathbf{p}_d\|}$$

Glare contamination occurs when the angular separation between the viewing direction and the specular direction falls below the glare half-angle $\gamma_{glare} = 5°$:

$$\text{glare} \iff \arccos\!\left(\hat{\mathbf{v}}_{view} \cdot \hat{\mathbf{r}}_{spec}\right) < \gamma_{glare}$$

### Glare-Free Altitude Band

For a drone flying directly above the panel row at along-track position $x$ and cross-track position $y_{strip}$, the glare condition reduces to a constraint on altitude $z$. Solving the specular geometry analytically gives the minimum glare-free altitude:

$$h_{gf,min}(y_{strip}) = \frac{(y_{strip} - y_{spec}(y_{strip}))\sin\varepsilon_{sun}}{\cos(\phi_{sun} - \phi_{view}) \cos\varepsilon_{sun} - \sin\varepsilon_{sun} \tan\alpha_{tilt}}$$

where $y_{spec}$ is the cross-track coordinate of the specular hotspot on the panel at ground level. In practice this is evaluated numerically: for each strip the drone sweeps $z \in [h_{edge,i} + \Delta_{safe},\; h_{max}]$ at 0.1 m resolution, tests the glare condition, and selects the lowest glare-free altitude:

$$h_{scan,i} = \min\!\left\{z \geq h_{edge,i} + \Delta_{safe} \;\Big|\; \arccos(\hat{\mathbf{v}}_{view} \cdot \hat{\mathbf{r}}_{spec}) \geq \gamma_{glare}\right\}$$

with safety clearance $\Delta_{safe} = 1.0$ m above the panel top edge.

### Camera Gimbal Pitch for Tilt-Normal Pointing

To point the camera perpendicular to the tilted panel surface from drone position $\mathbf{p}_d = (x, y_{strip}, h_{scan,i})$, the required gimbal pitch angle below horizontal is:

$$\beta_{cam} = \arctan\!\left(\frac{h_{scan,i} - h_{edge,i}/2}{\|y_{strip} - y_i^{bot} - \tfrac{1}{2}W_{panel}\cos\alpha_{tilt}\|}\right) - \alpha_{tilt}$$

The effective footprint on the tilted panel surface has semi-axes:

$$a_{foot} = \frac{h_{scan,i}}{\cos\beta_{cam}} \tan\!\left(\frac{\theta_{FOV}}{2}\right), \qquad b_{foot} = a_{foot} \cos\alpha_{tilt}$$

because the cross-track direction is foreshortened by the tilt angle. The strip spacing condition for full coverage becomes:

$$d_{strip} = 2 b_{foot} \cdot (1 - \rho)$$

### 3D Thermal Footprint on a Tilted Surface

The sensor footprint centre on the panel surface is found by intersecting the camera boresight ray with the panel plane. The panel plane through centre $\mathbf{c}_{ij}$ with normal $\hat{\mathbf{n}}_{panel}$ satisfies:

$$(\mathbf{p} - \mathbf{c}_{ij}) \cdot \hat{\mathbf{n}}_{panel} = 0$$

The boresight ray from drone position $\mathbf{p}_d$ in direction $\hat{\mathbf{d}}_{cam}$ hits the panel at parameter:

$$t^* = \frac{(\mathbf{c}_{ij} - \mathbf{p}_d) \cdot \hat{\mathbf{n}}_{panel}}{\hat{\mathbf{d}}_{cam} \cdot \hat{\mathbf{n}}_{panel}}$$

$$\mathbf{p}_{hit} = \mathbf{p}_d + t^* \hat{\mathbf{d}}_{cam}$$

A panel is within the sensor footprint when $\mathbf{p}_{hit}$ lies inside the panel boundary polygon (tested via a 2D point-in-rectangle check in the panel's local coordinate frame).

### Cast Shadow Mask

The shadow boundary of row $i$ on the ground plane is cast by the top edge of panel row $i$ in the direction opposite to $\hat{\mathbf{s}}$. The shadow tip on the ground is at:

$$\mathbf{p}_{shadow,i} = \mathbf{p}_{top,i} - \frac{h_{edge,i}}{\sin\varepsilon_{sun}} \hat{\mathbf{s}}_{xy}$$

where $\hat{\mathbf{s}}_{xy}$ is the horizontal component of $\hat{\mathbf{s}}$ (normalised). A panel $p_{kj}$ in the next row ($k = i+1$) is shadow-masked when its centre $\mathbf{c}_{kj}$ lies between $y_i^{top}$ and $p_{shadow,i,y}$:

$$\text{shadowed} \iff y_{kj} < p_{shadow,i,y} \quad \text{and} \quad |x_{kj} - x_{ij}| < L_{panel}/2$$

Detection is suppressed for shadow-masked panels during the inspection pass.

### 3D Boustrophedon Path with Altitude Variation

The 3D waypoint sequence for strip $i$ is:

$$\mathbf{w}_{i}^{start} = \begin{cases} (0,\; y_{strip,i},\; h_{scan,i}) & i \text{ even} \\ (L,\; y_{strip,i},\; h_{scan,i}) & i \text{ odd} \end{cases}$$

Row-crossing transition between strip $i$ and strip $i+1$ follows a three-segment arc:

1. **Climb**: from $(x_{end}, y_{strip,i}, h_{scan,i})$ vertically to $(x_{end}, y_{strip,i}, h_{transit})$
2. **Lateral traverse**: from $(x_{end}, y_{strip,i}, h_{transit})$ horizontally to $(x_{end}, y_{strip,i+1}, h_{transit})$
3. **Descend**: from $(x_{end}, y_{strip,i+1}, h_{transit})$ vertically to $(x_{end}, y_{strip,i+1}, h_{scan,i+1})$

The transit altitude clears the panel top edge with safety margin:

$$h_{transit} = h_{edge,i} + \Delta_{safe} + \delta_{transit}$$

with $\delta_{transit} = 0.5$ m additional clearance. Total 3D path length:

$$L_{3D} = \sum_{i=0}^{N_{strips}-1} L + (N_{strips}-1)\!\left[2|h_{transit} - \bar{h}_{scan}| + d_{strip}\right]$$

where $\bar{h}_{scan}$ is the mean scan altitude across strips.

### Motion Blur on a Tilted Surface

The ground sample distance on the tilted panel is modified by the oblique projection factor:

$$\text{GSD}_{tilt} = \frac{h_{scan,i}}{f \cdot n_{px}} \cdot \frac{s_w}{\cos(\beta_{cam} + \alpha_{tilt})}$$

The blur radius becomes:

$$b = \frac{v \cdot t_{exp}}{\text{GSD}_{tilt}}$$

and the maximum permissible speed is adjusted accordingly:

$$v_{max,tilt} = \frac{b_{max} \cdot \text{GSD}_{tilt}}{t_{exp}}$$

---

## Key 3D Additions

- **Tilted panel geometry**: each panel defined as a 3D planar polygon with normal $\hat{\mathbf{n}}_{panel}$; row-gap structure creates step discontinuities in the panel height profile.
- **Glare-free altitude selection**: per-strip altitude $h_{scan,i}$ determined by numerically solving the specular-reflection constraint; drone never enters the angular cone around $\hat{\mathbf{r}}_{spec}$.
- **Gimbal pitch control**: $\beta_{cam}$ computed per waypoint so the camera boresight remains perpendicular to the panel surface; eliminates oblique-projection footprint distortion.
- **Shadow avoidance**: cast-shadow mask derived from sun geometry suppresses hot-spot detection for panels behind the shadow boundary; mission planner schedules inspection time to minimise shadow-masked fraction.
- **3D transition arcs**: climb-traverse-descend three-segment legs replace flat row-crossing legs; transit altitude guarantees physical clearance over panel top edges.
- **Tilt-corrected blur model**: GSD scaled by the oblique viewing factor $1/\cos(\beta_{cam} + \alpha_{tilt})$; $v_{max}$ reduced relative to the nadir case.

---

## Key Parameters

| Parameter | Symbol | Value |
|-----------|--------|-------|
| Solar farm dimensions | — | 80 × 60 m |
| Panel array | — | 10 rows × 12 cols = 120 panels |
| Panel tilt angle | $\alpha_{tilt}$ | 25° (south-facing) |
| Row gap (maintenance aisle) | $g_{row}$ | 2.0 m |
| Panel top-edge height | $h_{edge}$ | $W_{panel}\sin 25° \approx 1.27$ m |
| Defective panels | $M$ | 8 |
| Thermal camera FOV | $\theta_{FOV}$ | 45° |
| Glare half-angle | $\gamma_{glare}$ | 5° |
| Safety clearance above edge | $\Delta_{safe}$ | 1.0 m |
| Transit clearance margin | $\delta_{transit}$ | 0.5 m |
| Scan altitude range | $h_{scan,i}$ | 2.3 – 5.0 m (per strip) |
| Transit altitude | $h_{transit}$ | $h_{edge} + 1.5$ m $\approx 2.77$ m |
| Cross-track overlap | $\rho$ | 20% |
| Nominal inspection speed | $v$ | 1.0 m/s |
| Max safe speed (tilt-corrected) | $v_{max,tilt}$ | ~1.2 m/s |
| Sun azimuth | $\phi_{sun}$ | 120° E of N |
| Sun elevation | $\varepsilon_{sun}$ | 35° |
| Nominal panel temperature | $T_{nominal}$ | 35°C |
| Hot spot excess temperature | $\Delta T_j$ | 10–20°C (uniform random) |
| Detection threshold | $\Delta T_{thresh}$ | 8°C |
| Camera exposure time | $t_{exp}$ | 0.033 s (30 fps) |
| Maximum blur radius | $b_{max}$ | 2.5 pixels |
| Simulation timestep | $\Delta t$ | 0.1 s |

---

## Expected Output

- **3D farm geometry plot** (`mpl_toolkits.mplot3d`): tilted panel surfaces rendered as flat polygons at their correct 3D positions; row gaps visible; sun direction vector drawn as an arrow; cast-shadow regions shaded dark grey on the ground plane; drone trajectory drawn as a red line with altitude variation.
- **Altitude profile vs along-track distance**: z-coordinate of the drone plotted against cumulative path distance; scan segments at $h_{scan,i}$ shown flat; transition climb-descend arcs shown as vertical spikes; glare-free altitude lower bound overlaid as a dashed green line.
- **Glare-free band visualisation** (`plt.subplots()`, cross-section view): for a representative strip, plot the angular separation $\arccos(\hat{\mathbf{v}}_{view} \cdot \hat{\mathbf{r}}_{spec})$ as a function of drone altitude $z$; mark the glare threshold $\gamma_{glare} = 5°$ and the selected scan altitude.
- **Detection results overlay** (2D top-down projected view): panel grid with healthy panels grey, detected hot panels red, missed panels (false negatives) orange, shadow-masked panels dark grey, glare-contaminated panels yellow; drone ground-track overlaid as a black line.
- **Shadow mask time series**: fraction of panels in shadow as a function of inspection time of day (08:00–16:00); mark the chosen inspection window where shadow fraction is below 10%.
- **Animation** (`FuncAnimation`): 3D perspective view of the drone sweeping the tilted panel array; panel surfaces colour-coded by thermal reading; drone marker moves along the 3D path including altitude transitions; panels flash red on detection; shadow boundary advances with time; saved as `outputs/04_industrial_agriculture/s073_3d_solar_thermal/s073_3d_animation.gif`.
- **Console metrics**: per-strip scan altitude, glare-free fraction, shadow-masked fraction, tilt-corrected $v_{max}$, 3D path length, coverage time, detection rate.

---

## Extensions

1. **Optimal inspection time-of-day**: sweep inspection start time from 07:00 to 15:00 at 15-minute resolution; compute the joint objective $J = w_1 A_{shadow} + w_2 C_{glare}$ and find the start time minimising $J$; compare with the fixed mid-morning assumption.
2. **Variable tilt angle sweep**: repeat the mission for $\alpha_{tilt} \in [0°, 45°]$ at 5° intervals; plot detection rate, path length, and glare-free fraction as functions of tilt; identify the critical tilt angle above which the standard lawnmower fails without altitude adaptation.
3. **Multi-row simultaneous coverage**: assign two drones to non-overlapping row bands; each drone independently selects its per-strip glare-free altitude; measure the reduction in total mission time relative to the single-drone baseline.
4. **Terrain-following on sloped ground**: extend the ground model from a flat plane to a gently sloped terrain ($\pm 3°$ slope); recompute all panel-edge heights and shadow boundaries; measure the increase in altitude variation along the trajectory.
5. **Adaptive gimbal for partial glare mitigation**: instead of avoiding the glare zone entirely, model the thermal signal degradation as a function of glare angle; allow the drone to enter a partial-glare region at reduced inspection speed to improve coverage time.
6. **Panel soiling detection**: add a soiling model where dirt accumulation reduces panel emissivity $\varepsilon_{panel}$; the apparent temperature drops by $\Delta T_{soil} = (1 - \varepsilon_{panel}) \cdot T_{sky}$; design a detection threshold that distinguishes soiling from hot-spot defects based on spatial extent.

---

## Related Scenarios

- Original 2D version: [S073 Solar Panel Thermal Inspection](../S073_solar_thermal.md)
- 3D path planning reference: [S002 3D Upgrade — Evasive Maneuver](../../01_pursuit_evasion/3d/S002_3d_evasive_maneuver.md)
- Boustrophedon fundamentals: [S048 Full-Area Coverage Scan](../../../scenarios/03_environmental_sar/S048_lawnmower.md)
- Curved surface inspection: [S074 Wind Turbine Blade Inspection](../S074_wind_turbine.md)
- Anomaly detection over mapped area: [S072 Greenhouse Gas Leak Detection](../S072_greenhouse_gas.md)
