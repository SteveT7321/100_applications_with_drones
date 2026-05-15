# S048 3D Upgrade — Lawnmower Coverage on Terrain

**Domain**: Environmental & SAR | **Difficulty**: ⭐⭐ | **Status**: `[ ]` Not Started
**Based on**: [S048 original](../S048_lawnmower.md)

---

## What Changes in 3D

The original S048 flies every waypoint at a single hardcoded altitude (`z = constant`) above a
flat reference plane. The sensor footprint radius is therefore a fixed $r_s = 5$ m regardless of
where the drone is in the area. On real terrain this assumption breaks in two places:

1. **Footprint bloat on slopes**: when the drone maintains a constant mean-sea-level (MSL) altitude
   over a hillside, its actual height above ground level (AGL) varies strip-by-strip. The nadir
   footprint radius scales linearly with AGL, so a drone that is 2 m too high casts a footprint
   12% wider than expected — causing false confidence in coverage — while being 2 m too low
   tightens the footprint and leaves gaps.
2. **Strip width must adapt**: on steep cross-track slopes the strip-width formula $d = 2 r_s$ no
   longer guarantees gap-free coverage because adjacent strip footprints are projected onto a
   tilted surface. The effective ground-projected width narrows by a factor of $\cos\phi$ where
   $\phi$ is the local terrain slope angle.

This variant replaces the flat altitude assumption with a **terrain-following AGL controller** that
reads a digital elevation model (DEM) and maintains a constant AGL clearance $h_{AGL}$. Strip
widths are recalculated at each turn leg using the **slope-adaptive formula**. Waypoints are
generated on the undulating 3D surface and the drone executes a true **3D boustrophedon** path
whose z-coordinates follow the terrain profile.

---

## Problem Definition

**Setup**: A $200 \times 200$ m search area overlies undulating terrain described by a synthetic
DEM. The terrain is modelled as a sum of smooth Gaussian hills with peak-to-valley relief of up to
30 m. A single drone must execute a complete boustrophedon coverage scan while maintaining a
constant AGL clearance of $h_{AGL} = 8$ m above the local ground. The sensor is a nadir-pointing
downward-facing camera whose circular footprint radius depends on the actual AGL:

$$r_s(h) = h_{AGL} \cdot \tan\!\left(\tfrac{\theta_{FOV}}{2}\right)$$

The drone flies the same three strip-width configurations as S048 (under-lap, optimal, over-lap)
but now uses the **slope-corrected strip width** derived from the local terrain slope measured
perpendicular to the strip direction (cross-track slope $\phi_{ct}$).

**Roles**:
- **Drone**: single UAV executing the 3D terrain-following boustrophedon path.
- **DEM**: a $200 \times 200$ m grid at 1 m resolution representing the terrain elevation
  $z_{terrain}(x, y)$.

**Objective**: For each configuration, report (1) coverage rate on the 3D surface mesh, (2) total
3D path length, (3) energy with terrain-following climb/descent cost, and (4) mean AGL deviation
from the target clearance — demonstrating that the terrain-following variant achieves the same
coverage guarantees as the flat baseline while operating over real topography.

**Comparison configurations**:
1. **Under-lap** — slope-corrected strip width $d = 12$ m (footprint gap on level ground).
2. **Optimal** — slope-corrected strip width $d^* = 2 r_s / \cos\phi_{ct}$ (guaranteed coverage).
3. **Over-lap** — slope-corrected strip width $d = 7$ m (30% redundant overlap on level ground).

---

## Mathematical Model

### Terrain Model (Synthetic DEM)

The terrain elevation at map coordinates $(x, y)$ is the superposition of $M$ Gaussian hills:

$$z_{terrain}(x, y) = \sum_{k=1}^{M} A_k \exp\!\left(-\frac{(x - x_k)^2 + (y - y_k)^2}{2\sigma_k^2}\right)$$

with peak amplitudes $A_k \in [5, 15]$ m, centres $(x_k, y_k)$ randomly placed in the area, and
radii $\sigma_k \in [20, 50]$ m. The gradient of the DEM at any point is:

$$\nabla z_{terrain}(x,y) = \left(\frac{\partial z}{\partial x},\; \frac{\partial z}{\partial y}\right)$$

computed from the discrete DEM using central differences.

### AGL Altitude Command

For a drone at map position $(x, y)$ the commanded flight altitude (MSL) is:

$$z_{cmd}(x, y) = z_{terrain}(x, y) + h_{AGL}$$

where $h_{AGL} = 8$ m is the constant target clearance. The actual AGL at each simulation step is:

$$h(t) = z_P(t) - z_{terrain}(x_P(t),\; y_P(t))$$

The AGL tracking error is $\varepsilon_{AGL}(t) = h(t) - h_{AGL}$.

### Altitude Controller

A proportional controller converts the AGL error into a vertical velocity command, capped at the
drone's maximum climb/descent rate $\dot{z}_{max}$:

$$\dot{z}_{cmd}(t) = \text{clip}\!\left(K_z \cdot \bigl[z_{cmd}(x_P, y_P) - z_P\bigr],\; -\dot{z}_{max},\; \dot{z}_{max}\right)$$

with $K_z = 1.5$ s$^{-1}$ and $\dot{z}_{max} = 3$ m/s.

### Sensor Footprint on Sloped Terrain

With a half-FOV angle $\theta_{FOV}/2 = 26.57°$ ($\tan = 0.5$) the nominal footprint radius at
target AGL is:

$$r_s = h_{AGL} \cdot \tan\!\left(\tfrac{\theta_{FOV}}{2}\right) = 8 \times 0.5 = 4 \text{ m}$$

On a surface patch with cross-track slope angle $\phi_{ct}$ (terrain slope measured perpendicular
to the strip direction), the ground-projected footprint half-width narrows to:

$$r_{eff} = r_s \cdot \cos\phi_{ct}$$

A strip must be spaced so that adjacent $r_{eff}$ discs remain tangent or overlapping. The
**slope-adaptive strip width** is therefore:

$$d_{slope}(y_i) = 2 r_{eff}(y_i) = 2 r_s \cos\phi_{ct}(y_i)$$

where $\phi_{ct}(y_i)$ is the mean cross-track slope angle sampled along strip $i$.

### 3D Boustrophedon Waypoint Generation

Strip $i$ is centred at cross-track position $y_i$ derived by accumulating the variable strip widths:

$$y_0 = d_{slope}(0)/2, \qquad y_i = y_{i-1} + \tfrac{1}{2}\bigl[d_{slope}(i-1) + d_{slope}(i)\bigr]$$

Each strip waypoint pair has a full 3D position:

$$\mathbf{w}_{i,start} = \begin{cases}
  \bigl(x_{min},\; y_i,\; z_{terrain}(x_{min}, y_i) + h_{AGL}\bigr) & i \text{ even} \\
  \bigl(x_{max},\; y_i,\; z_{terrain}(x_{max}, y_i) + h_{AGL}\bigr) & i \text{ odd}
\end{cases}$$

$$\mathbf{w}_{i,end} = \begin{cases}
  \bigl(x_{max},\; y_i,\; z_{terrain}(x_{max}, y_i) + h_{AGL}\bigr) & i \text{ even} \\
  \bigl(x_{min},\; y_i,\; z_{terrain}(x_{min}, y_i) + h_{AGL}\bigr) & i \text{ odd}
\end{cases}$$

Between waypoints the drone follows the commanded altitude profile $z_{cmd}(x_P(t), y_P(t))$
continuously, making the actual path a piecewise-smooth 3D curve rather than the flat parallel
lines of S048.

### 3D Path Length

The total 3D path length integrates the instantaneous speed including the vertical component:

$$L_{3D} = \int_0^T \|\dot{\mathbf{p}}_P(t)\|\, dt = \int_0^T \sqrt{\dot{x}_P^2 + \dot{y}_P^2 + \dot{z}_P^2}\; dt$$

discretised as:

$$L_{3D} \approx \sum_{k=1}^{K} \|\mathbf{p}_P(k) - \mathbf{p}_P(k-1)\|$$

The flat-ground lower bound $L_{2D} = N H + (N-1)\bar{d}$ is recovered when $\dot{z}_P \equiv 0$.

### 3D Coverage Metric

Coverage is measured on the DEM surface rather than a horizontal plane. Each 1 m $\times$ 1 m
surface patch at $(x_c, y_c)$ with elevation $z_c = z_{terrain}(x_c, y_c)$ is marked scanned when
the 3D distance from the drone to the patch centre is no greater than the slant range consistent
with the sensor footprint:

$$\text{scanned} \iff \sqrt{(x_P - x_c)^2 + (y_P - y_c)^2} \leq r_s \cdot \frac{h(t)}{h_{AGL}}$$

The right-hand side scales the nominal footprint radius linearly with the instantaneous AGL $h(t)$,
so transient altitude deviations alter the effective footprint in real time.

### Energy Model with Terrain-Following Cost

The energy model extends S048 to include the additional power required for vertical flight:

$$E = P_{cruise}\frac{L_{horiz}}{v} + (P_{hover} - P_{cruise}) t_{turn}(N-1) + P_{climb}\frac{D_{climb}}{v_z^{avg}}$$

where:
- $L_{horiz} = \sum_k \sqrt{\dot{x}_P^2 + \dot{y}_P^2}\,\Delta t$ — horizontal distance flown
- $D_{climb} = \sum_k |\dot{z}_P(k)|\,\Delta t$ — total vertical distance traversed (up + down)
- $P_{climb} = 250$ W — power during sustained climb
- $v_z^{avg}$ — mean absolute vertical speed during climb/descent segments

---

## Key 3D Additions

- **AGL altitude tracking**: $z_{cmd}(x,y) = z_{terrain}(x,y) + h_{AGL}$ with proportional controller $K_z = 1.5$ s$^{-1}$
- **Slope-adaptive strip width**: $d_{slope} = 2 r_s \cos\phi_{ct}$ recalculated at each turn leg
- **3D boustrophedon waypoints**: waypoint z-coordinates follow the DEM profile at strip endpoints
- **FOV-based footprint**: $r_s = h_{AGL} \tan(\theta_{FOV}/2)$ varies with actual AGL
- **3D path length integration**: $L_{3D}$ includes $\dot{z}$ contribution from terrain-following climbs
- **Terrain-following energy cost**: additional $P_{climb}$ term for vertical displacement
- **Altitude bounds**: $z_P \in [z_{terrain} + 2,\; z_{terrain} + 20]$ m (safety floor + ceiling)

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Search area | 200 × 200 m |
| DEM resolution | 1 m |
| Terrain relief | 0 – 30 m (Gaussian hills, $M = 5$ hills) |
| Target AGL clearance $h_{AGL}$ | 8 m |
| Camera half-FOV $\theta_{FOV}/2$ | 26.57° ($\tan = 0.5$) |
| Nominal footprint radius $r_s$ | 4 m |
| Drone cruise speed $v$ | 10 m/s |
| Max climb/descent rate $\dot{z}_{max}$ | 3 m/s |
| Altitude controller gain $K_z$ | 1.5 s$^{-1}$ |
| Cruise power $P_{cruise}$ | 150 W |
| Hover/turn power $P_{hover}$ | 200 W |
| Climb power $P_{climb}$ | 250 W |
| Turn time per reversal $t_{turn}$ | 4 s |
| Simulation timestep $\Delta t$ | 0.5 s |
| Altitude bounds (AGL) | 2 – 20 m |
| Under-lap strip spacing $d$ | 12 m |
| Optimal strip spacing $d^*$ | $2 r_s / \cos\phi_{ct}$ (adaptive) |
| Over-lap strip spacing $d$ | 7 m |

---

## Expected Output

- **3D trajectory plot**: all three configurations plotted on a 3D surface mesh of the DEM; the
  drone paths show visible altitude undulation following the terrain contours; strip colours
  alternate between configurations for legibility.
- **Altitude time series**: $z_P(t)$, $z_{terrain}(x_P(t), y_P(t))$, and $z_{cmd}(t)$ on the
  same axes, with AGL clearance $h(t)$ shown in a lower subplot — demonstrates that the
  controller maintains $h_{AGL} \approx 8$ m with bounded error.
- **AGL deviation histogram**: distribution of $\varepsilon_{AGL}(t)$ for the full mission,
  confirming that 95% of samples remain within $\pm 1$ m of the target clearance.
- **Coverage map on terrain**: top-down heatmap of scan-count per 1 m cell overlaid on the DEM
  contours; slope-adaptive footprints produce more uniform coverage density than fixed-width strips.
- **Slope map and adaptive strip widths**: side-by-side display of the cross-track slope field
  $\phi_{ct}(y)$ and the resulting per-strip spacing $d_{slope}(y_i)$, illustrating how steep
  slopes force narrower strips.
- **Comparison bar chart**: coverage rate, 3D path length, and total energy for all three
  configurations — analogous to S048's flat-ground chart but with the additional terrain-following
  energy overhead shown as a stacked bar segment.
- **Coverage vs distance curve**: cumulative coverage rate against 3D distance flown for each
  configuration; the adaptive-optimal line reaches 100% while the under-lap line falls short
  on steep-terrain patches.

---

## Extensions

1. **Real DEM ingestion**: replace the synthetic Gaussian DEM with a real SRTM or Copernicus DEM
   tile loaded via `rasterio`; validate that the slope-adaptive formula maintains coverage on
   genuine mountainous terrain (e.g., alpine meadow inspection).
2. **Optimal FOV tilt**: allow the camera to tilt forward or sideward by up to 15° to maintain
   a vertical look-angle on steep slopes; optimise the tilt schedule jointly with the altitude
   profile to minimise footprint distortion.
3. **Wind-disturbance rejection**: add turbulence generated from the terrain-induced updraft model
   (proportional to slope and cruise speed); extend the AGL controller with an integral term to
   null steady-state wind-induced altitude bias.
4. **Multi-drone terrain-following formation**: assign $K$ drones to parallel column bands (S049);
   add cross-track altitude coordination so that no two drones simultaneously occupy the same
   elevation band above a ridge, reducing wake-turbulence risk.
5. **Energy-optimal AGL profile**: solve a 1D optimal control problem along each strip for the
   AGL profile $h(x)$ that minimises $\int P_{climb}\,|\dot{z}|\,dt$ subject to the coverage
   constraint $r_s(h) \geq r_{min}$ and the safety floor $h \geq h_{floor}$.
6. **Online DEM refinement**: treat the DEM as uncertain; fuse range-finder altimeter readings
   during the mission into a Gaussian Process terrain estimate and update strip widths in real
   time as the estimate improves (links to S050 Swarm SLAM).

---

## Related Scenarios

- Original 2D version: [S048](../S048_lawnmower.md)
- Truly 3D references: [S001](../../01_pursuit_evasion/S001_basic_intercept.md), [S043 Confined Space](../S043_confined_space.md)
- Multi-drone extension: [S049 Dynamic Zone Assignment](../S049_dynamic_zone.md)
- Mapping feedback: [S050 Swarm SLAM](../S050_slam.md)
- Agricultural parallel: [S067 Spray Overlap Optimization](../../04_industrial_agriculture/S067_spray_overlap_optimization.md)
