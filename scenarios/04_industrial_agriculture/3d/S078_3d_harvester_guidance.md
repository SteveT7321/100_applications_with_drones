# S078 3D Upgrade — Harvester Guidance

**Domain**: Industrial & Agriculture | **Difficulty**: ⭐⭐⭐ | **Status**: `[ ]` Not Started
**Based on**: [S078 original](../S078_harvester_guidance.md)

---

## What Changes in 3D

The original S078 operates entirely in a flat 2D plane: the scout drone is implicitly held at a
fixed flight altitude while the harvester rolls along a horizontal surface. Obstacles are
characterised by a 2D radius only, terrain is featureless, and all A* planning is performed on a
uniform 400 × 100 occupancy grid with no vertical component. Three significant limitations follow:

1. **Flat-terrain assumption**: real agricultural fields are sloped. A hillside field tilts the
   harvester's effective travel axis; the energy-optimal harvesting direction changes with slope,
   and a fixed lead-distance measured along horizontal ground no longer corresponds to the same
   physical look-ahead on a 10° incline.
2. **No altitude-varying crop detection**: canopy height varies spatially (taller at dense patches,
   shorter at sparse or harvested rows). A fixed-altitude scout may fly into crop canopy or too
   far above it, reducing sensor effectiveness. Maintaining a constant Above-Canopy Level (ACL)
   requires the drone to actively track the local terrain elevation plus the crop-height map.
3. **No 3D relative positioning**: the vector from scout drone to harvester is currently a 2D
   offset. In 3D, drone-to-harvester handoff signals, corridor projections, and lawnmower
   sub-pattern geometry must all be expressed in 3D, including the correct projection of the
   horizontal corridor onto a tilted terrain surface.

This 3D upgrade adds a Digital Elevation Model (DEM), altitude-varying crop-height detection, and
full 3D drone-to-harvester relative positioning. The harvester's speed is modulated by slope (a
simple tractive-force energy model), and the scout drone maintains a target ACL above the local
canopy.

---

## Problem Definition

**Setup**: A 200 × 50 m agricultural field is modelled with a smooth sinusoidal DEM giving terrain
heights $z_{terrain}(x,y)$ that vary between 0 and 4 m, producing slopes up to approximately 12°.
A spatially varying crop-height map $h_{crop}(x,y) \in [0.3, 1.2]$ m (taller near field centre,
shorter near boundaries) is superimposed on the terrain. $N_{obs} = 15$ obstacles (rocks, stumps)
are placed at 3D positions; their effective detection height above terrain is $h_{obs} = 0.5$ m so
the scout must fly low enough to detect them.

The harvester advances along the field's long axis at a nominal speed of $v_0 = 1.5$ m/s, but its
actual speed is attenuated by terrain slope $\phi(x, y)$ (positive uphill):

$$v_{harvester}(x, y) = v_0 \cdot \cos\phi(x, y) \cdot \eta_{slope}(\phi)$$

The scout drone flies 45 m ahead of the harvester in the along-terrain direction, maintaining a
target ACL of $h_{ACL} = 2.0$ m above the local canopy top:

$$z_{drone,cmd}(x, y) = z_{terrain}(x, y) + h_{crop}(x, y) + h_{ACL}$$

The drone executes the same 3-pass boustrophedon sub-pattern as S078, but the strip geometry is
projected onto the tilted terrain surface so that strip widths remain 3.3 m in the horizontal
plane of the slope.

**Roles**:
- **Scout drone**: 3D position-controlled sensor platform; tracks $z_{drone,cmd}$ via an altitude
  PD controller; flies at airspeed $v_{drone} = 3.5$ m/s; detects any obstacle whose 3D distance
  to the drone is within $r_{sensor} = 2.5$ m.
- **Harvester**: slope-speed-adjusted ground vehicle; runs a 3D A* on a 400 × 100 × $N_z$ layered
  grid (but constrained to the terrain surface, so effectively 2.5D); replans when a new obstacle
  relay arrives, inflating each detected obstacle by $r_{inflate} = 2.5$ m on the terrain surface.

**Objective**: Quantify how terrain slope and crop-height variation alter (1) the scout's effective
scan coverage compared to the flat baseline, (2) the harvester speed profile and resulting mission
time, and (3) the detection rate and extra path length relative to the 2D flat scenario.

---

## Mathematical Model

### Terrain and Crop-Height Model

The DEM is a smooth bivariate sinusoid:

$$z_{terrain}(x, y) = A_{dem} \left[ \sin\!\left(\frac{2\pi x}{L_{dem,x}}\right)
  + 0.5\,\sin\!\left(\frac{2\pi y}{L_{dem,y}}\right) \right]$$

with $A_{dem} = 2.0$ m, $L_{dem,x} = 200$ m, $L_{dem,y} = 100$ m. Local terrain slope angle:

$$\phi(x, y) = \arctan\!\left(\sqrt{\left(\frac{\partial z}{\partial x}\right)^2
  + \left(\frac{\partial z}{\partial y}\right)^2}\right)$$

Crop height map (Gaussian-modulated):

$$h_{crop}(x, y) = 0.3 + 0.9\,\exp\!\left(-\frac{(x - 100)^2}{2 \times 60^2}
  - \frac{(y - 25)^2}{2 \times 15^2}\right)$$

### Slope-Attenuated Harvester Speed

A first-order tractive-force model relates speed to slope:

$$\eta_{slope}(\phi) = \begin{cases}
  1.0 & |\phi| \leq \phi_{ref} \\
  \max\!\left(0.4,\; 1.0 - k_{slope}\,(\phi - \phi_{ref})\right) & \phi > \phi_{ref} \\
  \min\!\left(1.3,\; 1.0 - k_{slope}\,(\phi - \phi_{ref})\right) & \phi < -\phi_{ref}
\end{cases}$$

where $\phi_{ref} = 3°$ is the reference flat-equivalent threshold and $k_{slope} = 5\,\text{rad}^{-1}$
scales the speed reduction per radian of excess slope. Downhill ($\phi < -\phi_{ref}$) is capped
at 1.3× to model engine braking.

### Drone Altitude Command (Above Canopy Level)

Target flight altitude above WGS84/MSL at position $(x, y)$:

$$z_{cmd}(x, y) = z_{terrain}(x, y) + h_{crop}(x, y) + h_{ACL}$$

Altitude tracking with a PD controller:

$$\dot{z}_{drone} = K_p \left(z_{cmd} - z_{drone}\right) + K_d \left(\dot{z}_{cmd} - \dot{z}_{drone}\right)$$

with $K_p = 2.0\,\text{s}^{-1}$, $K_d = 0.4\,\text{s}$, and maximum vertical rate
$\dot{z}_{max} = 1.5\,\text{m/s}$.

### 3D Drone Lead Position

The lead vector is defined along the terrain-projected heading $\hat{\mathbf{v}}_{H,xy}$ of the
harvester:

$$\mathbf{p}_{lead}(t) = \mathbf{p}_{harvester}(t)
  + L_{lead}\,\hat{\mathbf{v}}_{H,xy}
  + \Delta z_{lead}\,\hat{\mathbf{z}}$$

where $\Delta z_{lead} = z_{cmd}(x_{lead}, y_{lead}) - z_{harvester}$ places the nominal lead
point at the correct ACL above terrain, and $\hat{\mathbf{v}}_{H,xy}$ is the horizontal unit
vector of the harvester's current travel direction.

### 3D Obstacle Detection

An obstacle at 3D position $\mathbf{p}_{obs,j} = (x_j, y_j, z_{terrain,j} + h_{obs})$ is
detected when:

$$\left\|\mathbf{p}_{drone}(t) - \mathbf{p}_{obs,j}\right\|_3 \leq r_{sensor}$$

where $\|\cdot\|_3$ denotes the Euclidean distance in 3D. Compared to the 2D scenario, the drone
now also needs sufficient vertical proximity; if the drone flies too high above canopy, low-profile
obstacles may fall outside $r_{sensor}$ even when the 2D horizontal distance is within range.
The effective horizontal detection radius at altitude offset $\Delta z$ is:

$$r_{sensor,xy}(\Delta z) = \sqrt{r_{sensor}^2 - \Delta z^2}, \quad |\Delta z| < r_{sensor}$$

### 3D Lawnmower Strip Projection

On a terrain slope with surface normal $\hat{\mathbf{n}}_{terrain}$, the strip lateral offset
$\Delta y_{strip}$ in the horizontal plane corresponds to an arc length on the terrain surface:

$$\Delta s_{strip} = \frac{\Delta y_{strip}}{\cos\phi_{y}(x, y)}$$

where $\phi_y = \arctan(\partial z / \partial y)$ is the cross-slope angle. The drone waypoints
are adjusted so that horizontal strip width remains $w_{scan} = 3.3$ m regardless of terrain slope.

### 2.5D A* Terrain-Constrained Replanning

Because the harvester is a ground vehicle, path planning is performed on the 2D occupancy grid
(same as S078) but path-length costs are corrected for terrain:

$$f(n) = g_{terrain}(n) + h(n)$$

where the terrain-corrected arc-length cost between adjacent cells $n$ and $m$ is:

$$g_{terrain}(n \to m) = \Delta_{grid} \cdot \frac{1}{\cos\phi(x_m, y_m)}$$

This ensures the A* cost reflects actual ground distance rather than projected horizontal distance.
The detection and inflation logic from S078 carries over unchanged.

---

## Key 3D Additions

- **Terrain DEM**: bivariate sinusoidal elevation model, slopes up to ~12°; used to compute
  harvester speed attenuation and drone target altitude.
- **Slope-attenuated harvester speed**: $v_{harvester}$ is modulated by $\eta_{slope}(\phi)$
  (range 0.4–1.3×), extending or shortening mission time relative to the flat baseline.
- **Above-Canopy Level (ACL) altitude tracking**: drone altitude command is $z_{terrain} + h_{crop} + h_{ACL}$,
  tracked with a PD controller; altitude time series shows the drone rising over hills and
  descending into valleys.
- **3D obstacle detection geometry**: effective horizontal detection radius shrinks with vertical
  offset $\Delta z$, introducing new missed-detection cases absent in 2D.
- **Terrain-corrected A* arc length**: grid edge costs are divided by $\cos\phi$ to reflect true
  ground distance on sloped cells.
- **3D trajectory visualisation**: Matplotlib 3D axes showing drone flight path with altitude
  variation, terrain surface mesh, and crop-height colour map overlay.

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Field dimensions | 200 × 50 m (horizontal) |
| DEM amplitude $A_{dem}$ | 2.0 m |
| DEM wavelengths | $L_{dem,x}$ = 200 m, $L_{dem,y}$ = 100 m |
| Max terrain slope | ~12° |
| Crop height range $h_{crop}$ | 0.3 – 1.2 m |
| Above-canopy level $h_{ACL}$ | 2.0 m |
| Nominal harvester speed $v_0$ | 1.5 m/s |
| Slope reference angle $\phi_{ref}$ | 3° |
| Slope attenuation coefficient $k_{slope}$ | 5 rad$^{-1}$ |
| Speed range (slope effect) | 0.6 – 1.95 m/s |
| Scout drone airspeed $v_{drone}$ | 3.5 m/s |
| Max vertical rate $\dot{z}_{max}$ | 1.5 m/s |
| Altitude PD gains $(K_p, K_d)$ | (2.0 s$^{-1}$, 0.4 s) |
| Sensor detection radius $r_{sensor}$ (3D) | 2.5 m |
| Obstacle height above terrain $h_{obs}$ | 0.5 m |
| Lead distance $L_{lead}$ | 45 m |
| A* inflation radius $r_{inflate}$ | 2.5 m |
| A* grid resolution $\Delta_{grid}$ | 0.5 m |
| Grid size | 400 × 100 cells |
| Communication latency $T_{latency}$ | 2 s |
| GPS position error $\sigma_{GPS}$ | 0.1 m |
| Number of obstacles $N_{obs}$ | 15 |
| Simulation timestep $\Delta t$ | 0.1 s |
| Altitude bounds | 0.5 – 8.0 m MSL |

---

## Expected Output

- **3D field overview**: Matplotlib 3D axes with the terrain surface mesh (grey wireframe), crop
  height colour map draped over the terrain, the drone flight path as a blue 3D trace showing
  altitude variation over hills and valleys, and the harvester ground track as a red trace on
  the terrain surface; obstacles shown as 3D spheres (grey = undetected, orange = detected).
- **Altitude time series**: four stacked subplots for (1) drone MSL altitude, (2) terrain height
  beneath the drone, (3) ACL gap ($z_{drone} - z_{terrain} - h_{crop}$), and (4) harvester
  speed; the ACL gap should track near 2.0 m with transient excursions during rapid terrain
  transitions.
- **Detection coverage map (top-down)**: 2D projection of the field with obstacles coloured by
  detection status, annotated with the 3D effective horizontal detection radius contours for
  representative drone-altitude offsets.
- **Speed profile and mission time**: harvester speed $v_{harvester}(t)$ plotted against time,
  with terrain slope $\phi(t)$ on a secondary axis; compare flat-baseline mission time (133 s)
  versus the sloped scenario.
- **3D vs 2D detection rate comparison**: bar chart comparing detection rate, extra path length,
  and latency penalty between the flat S078 baseline and the 3D sloped scenario; quantifies the
  performance degradation due to terrain-induced altitude mismatch.
- **Mission animation (GIF)**: rotating 3D view of the drone and harvester moving across the
  terrain surface, with the drone altitude dynamically adjusting to the canopy profile.

---

## Extensions

1. **DEM sensitivity sweep**: vary $A_{dem} \in \{0, 1, 2, 3, 4\}$ m and plot detection rate and
   mission time; identify the slope amplitude at which terrain-induced altitude mismatch causes
   the detection rate to drop below 80% of the flat-terrain baseline.
2. **Adaptive ACL**: instead of a fixed $h_{ACL} = 2.0$ m, solve for the minimum ACL that keeps
   $r_{sensor,xy}(\Delta z) \geq 1.5$ m everywhere given the local terrain gradient; compare
   battery usage (altitude variation energy cost) against flat-ACL policy.
3. **Multi-drone hillside coverage**: deploy two scout drones on opposite sides of the ridge;
   assign each drone a half-field coverage zone; study how terrain occlusion (ridge blocking
   LOS to harvester) affects relay communication and whether store-and-forward buffering is
   needed.
4. **Terrain-aware lead distance**: make $L_{lead}$ a function of slope — shorter on steep uphill
   (harvester slower, less look-ahead needed) and longer on flat or downhill sections; formulate
   as a simple proportional rule and compare with fixed $L_{lead} = 45$ m.
5. **Energy model integration**: extend the slope-attenuation model with a full energy budget
   (battery state of charge) for both the harvester and the drone; stop the mission when either
   agent runs out of energy and quantify how terrain slope affects total area harvested per charge.
6. **Real DEM input**: replace the synthetic sinusoidal DEM with a GeoTIFF DEM tile (e.g., SRTM
   30 m resolution resampled to 0.5 m grid); demonstrate the pipeline on a real terraced rice
   field; connect to [S065 3D Scan Path Planning](../S065_3d_scan_path.md).

---

## Related Scenarios

- Original 2D version: [S078 Harvester Cooperative Guidance](../S078_harvester_guidance.md)
- Terrain scanning reference: [S065 3D Scan Path Planning](../S065_3d_scan_path.md)
- Boustrophedon geometry: [S048 Full-Area Coverage Scan](../../../scenarios/03_environmental_sar/S048_lawnmower.md)
- Large-field coordination: [S068 Large-Scale Farmland Cooperative Spraying](../S068_large_field_spray.md)
