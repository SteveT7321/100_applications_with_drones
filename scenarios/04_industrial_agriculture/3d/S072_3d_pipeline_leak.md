# S072 3D Upgrade — Pipeline Leak Detection

**Domain**: Industrial & Agriculture | **Difficulty**: ⭐⭐⭐ | **Status**: `[ ]` Not Started
**Based on**: [S072 original](../S072_pipeline_leak.md)

---

## What Changes in 3D

The original S072 fixes the pipeline polyline entirely at $z = 0$ m (ground level) and flies the drone at a hardcoded constant offset `PATROL_HEIGHT = 3.0 m` above that flat plane. The Gaussian plume is advected only in the horizontal plane (`WIND_VEL[2] = 0`), the gradient-ascent localisation probes only $x$ and $y$ (`grad[2] = 0`), and the Frenet binormal $\hat{B}$ is always parallel to $\hat{z}$ by construction.

In a realistic pipeline survey the pipe follows terrain: it climbs over ridges, descends into valleys, and crosses ravines at varying elevations. This upgrade adds:

- **Elevation-varying pipeline geometry**: the polyline vertices carry non-zero $z$ components, producing segments that rise and fall across terrain.
- **Terrain-following patrol altitude**: the drone maintains a constant stand-off distance $d_{so}$ measured along the local Frenet binormal $\hat{B}_i$ (perpendicular to the pipe and terrain slope), not a fixed global $z$.
- **3D Gaussian plume with vertical wind component**: wind carries the methane plume diagonally upward or downward; the full 3D concentration gradient is exploited during localisation.
- **Altitude-optimal sensor swath**: the drone adjusts its stand-off dynamically so that the sensor footprint (modelled as a cone of half-angle $\phi$) achieves a target ground-coverage width $w_{target}$, balancing detection range against concentration sensitivity.
- **Full 3D gradient-ascent localisation**: finite differences along all three axes guide the drone to the leak, with altitude bounds tracking the local terrain height rather than a fixed floor.

---

## Problem Definition

**Setup**: A 200 m gas pipeline follows a terrain profile defined by a 5-vertex piecewise-linear polyline in $\mathbb{R}^3$, with vertices spanning elevation changes of up to $\pm 4$ m relative to a nominal $z_{base}$. A single UAV patrols the pipeline in the Frenet frame of the elevated pipe, maintaining a stand-off distance $d_{so} = 3$ m along $\hat{B}_i$ (the local binormal, which now has a nonzero horizontal component when the pipe is inclined). The drone carries a photoionisation detector (PID) sampling gas concentration every $\Delta t = 0.5$ s.

One methane leak exists at an unknown arc-length $s_{leak}$ along the pipe. The leak emits a continuous Gaussian puff train; wind advects puffs with a full 3D velocity $\mathbf{u}_{wind} = (u_x, u_y, u_z)$ m/s, carrying the plume both downwind and vertically (e.g., uphill slope trapping or updraft dispersion). On detection, the drone exits patrol mode and performs a 3D gradient-ascent spiral that also varies altitude, converging on the peak-concentration point.

**Roles**:
- **Drone**: single UAV; follows the elevated pipeline Frenet frame during patrol at stand-off $d_{so}$; switches to full 3D gradient-ascent localisation on detection.
- **Pipeline**: 200 m piecewise-linear polyline with 5 vertices at varying elevations $z \in [0, 4]$ m above ground; the drone tracks the normal-offset path $\mathbf{p}_{nom}(s) = \mathbf{p}_{ref}(s) + d_{so}\,\hat{B}_i$.
- **Leak source**: fixed point at arc-length $s_{leak}$ (unknown to drone) emitting methane at rate $\dot{Q} = 1.0$ g/s; disperses as a 3D instantaneous Gaussian puff train advected by $\mathbf{u}_{wind} \in \mathbb{R}^3$.
- **Terrain floor**: elevation function $z_{floor}(x, y)$ (linear interpolation of a DEM grid) that enforces the minimum safe altitude $z_{min}(x,y) = z_{floor}(x,y) + z_{safety}$ during both patrol and localisation.

**Objective**: Detect the leak within the patrol pass and localise the leak to within accuracy $\varepsilon_{target} = 2$ m in 3D Euclidean distance via full 3D gradient ascent. Report detection time $t_{detect}$, 3D localisation error $\varepsilon = \|\hat{\mathbf{p}}_{leak} - \mathbf{p}_{leak,true}\|$, terrain-clearance margin during flight, and sensor swath coverage along the pipeline.

**Comparison strategies**:
1. **Flat-terrain baseline** — pipeline at constant $z = 0$ m, fixed $h = 3$ m patrol height, horizontal-only gradient ascent (reproduces S072 original).
2. **Elevated pipeline, fixed global height** — polyline has elevation changes but drone uses a fixed $z = h_{global}$ offset, causing variable stand-off and potential terrain collision on descending segments.
3. **Elevated pipeline, Frenet stand-off** — drone maintains constant $d_{so}$ along $\hat{B}_i$; terrain-following patrol with Frenet PID on all three error components.
4. **Elevated pipeline, altitude-optimal swath** — stand-off varies segment-by-segment so that the sensor cone footprint achieves width $w_{target}$; trades detection sensitivity for coverage uniformity.

---

## Mathematical Model

### Elevated Pipeline Polyline and 3D Frenet Frame

Let the pipeline vertices be $\mathbf{p}_0, \ldots, \mathbf{p}_M \in \mathbb{R}^3$ with arbitrary $z$ components. Each segment $i$ has:

$$\hat{T}_i = \frac{\mathbf{p}_{i+1} - \mathbf{p}_i}{\|\mathbf{p}_{i+1} - \mathbf{p}_i\|}$$

Because $\hat{T}_i$ is no longer horizontal, the binormal is no longer parallel to $\hat{z}$. Compute the lateral normal by projecting out the along-pipe component of the world up vector:

$$\hat{N}_i = \frac{\hat{z} - (\hat{z} \cdot \hat{T}_i)\,\hat{T}_i}{\|\hat{z} - (\hat{z} \cdot \hat{T}_i)\,\hat{T}_i\|}$$

The binormal (pointing "outward-and-upward" perpendicular to the pipe):

$$\hat{B}_i = \hat{T}_i \times \hat{N}_i$$

For a level segment $\hat{B}_i = \hat{z}$, recovering the original S072 geometry.

### Drone Nominal Path with Terrain-Following Stand-Off

The drone tracks a path offset from the pipeline by stand-off $d_{so}$ along $\hat{B}_i$:

$$\mathbf{p}_{nom}(s) = \mathbf{p}_{ref}(s) + d_{so}\,\hat{B}_i(s)$$

For a segment inclined at angle $\psi_i = \arcsin(\hat{T}_i \cdot \hat{z})$ from horizontal, the drone's altitude above the pipe joint is:

$$z_{drone}(s) = z_{pipe}(s) + d_{so}\cos\psi_i$$

which is less than the flat-terrain value $d_{so}$ for steeply inclined segments, demanding active vertical correction.

### Altitude-Optimal Sensor Swath

The sensor is modelled as a downward-pointing cone of half-angle $\phi = 15°$. The footprint width on the pipe surface (perpendicular to the segment tangent) at stand-off $d_{so}$ is:

$$w = 2\,d_{so}\tan\phi$$

To achieve a target coverage width $w_{target} = 1.5$ m per patrol pass, the required stand-off is:

$$d_{so}^*(s) = \frac{w_{target}}{2\tan\phi}$$

However, increasing $d_{so}$ decreases the peak detectable concentration. The maximum detectable range $r_{max}$ satisfies:

$$C_{thresh} = \frac{\dot{Q}}{(4\pi D r_{max})^{3/2}}$$

$$\Rightarrow\quad r_{max} = \left(\frac{\dot{Q}}{C_{thresh}}\right)^{2/3} \frac{1}{4\pi D}$$

The operational stand-off is therefore clamped:

$$d_{so}(s) = \min\!\bigl(d_{so}^*,\; r_{max} - \epsilon_{margin}\bigr)$$

### Frenet Error Decomposition in 3D

At drone position $\mathbf{p}$ on segment $i$, define the error vector relative to the nominal path:

$$\boldsymbol{\Delta} = \mathbf{p} - \mathbf{p}_{nom}(s)$$

Decompose into Frenet components:

$$e_T = \boldsymbol{\Delta} \cdot \hat{T}_i \quad \text{(along-track lag)}$$

$$e_N = \boldsymbol{\Delta} \cdot \hat{N}_i \quad \text{(lateral cross-track error)}$$

$$e_B = \boldsymbol{\Delta} \cdot \hat{B}_i \quad \text{(stand-off error, now not purely vertical)}$$

Three independent PID loops regulate these errors. Because $\hat{B}_i$ is not generally aligned with $\hat{z}$, the stand-off PID corrects a mix of vertical and horizontal displacement:

$$\mathbf{a}_{PID} = a_T\,\hat{T}_i + a_N\,\hat{N}_i + a_B\,\hat{B}_i$$

Terrain-clearance enforcement: after each position update, project $\mathbf{p}$ onto the terrain DEM and enforce:

$$p_z \geq z_{floor}(p_x, p_y) + z_{safety}$$

### 3D Gaussian Plume Dispersion

The continuous leak is modelled as a superposition of discrete puffs released at $\Delta t_{puff}$ intervals. Each puff is advected by the full 3D wind $\mathbf{u}_{wind} = (u_x, u_y, u_z)$, producing a plume that rises or descends depending on wind direction:

$$C(\mathbf{r}, t) = \sum_{k} \frac{Q_{puff}}{\left(4\pi D (t - t_k)\right)^{3/2}}
  \exp\!\left(-\frac{\|\mathbf{r} - \mathbf{r}_{leak} - \mathbf{u}_{wind}(t - t_k)\|^2}{4D(t - t_k)}\right)$$

where $\mathbf{r}_{leak}$ is the leak position in 3D (on the elevated pipe), and the puff centroid after age $\tau = t - t_k$ is:

$$\mathbf{c}_k(t) = \mathbf{r}_{leak} + \mathbf{u}_{wind}\,\tau$$

A vertical wind component $u_z > 0$ lifts puffs above the pipe level, increasing the stand-off required for detection; $u_z < 0$ pushes the plume toward the ground.

### Sensor Model (3D)

$$C_{meas}(t) = C(\mathbf{p}_{drone}(t),\, t) + \eta(t), \qquad \eta(t) \sim \mathcal{N}(0,\,\sigma_{sensor}^2)$$

Detection is declared when $C_{meas}(t) > C_{thresh} = 0.01$.

### Full 3D Gradient-Ascent Localisation

On detection the drone estimates the 3D concentration gradient by finite differences with probe offset $\delta = 0.5$ m along all three axes:

$$\hat{\nabla}_x C \approx \frac{C_{meas}(x+\delta,y,z) - C_{meas}(x-\delta,y,z)}{2\delta}$$

$$\hat{\nabla}_y C \approx \frac{C_{meas}(x,y+\delta,z) - C_{meas}(x,y-\delta,z)}{2\delta}$$

$$\hat{\nabla}_z C \approx \frac{C_{meas}(x,y,z+\delta) - C_{meas}(x,y,z-\delta)}{2\delta}$$

The full 3D ascent direction:

$$\hat{\mathbf{g}}_{3D} = \frac{(\hat{\nabla}_x C,\;\hat{\nabla}_y C,\;\hat{\nabla}_z C)}
                              {\|(\hat{\nabla}_x C,\;\hat{\nabla}_y C,\;\hat{\nabla}_z C)\| + \varepsilon}$$

Command velocity during 3D gradient ascent:

$$\mathbf{v}_{cmd} = v_{ascent}\,\hat{\mathbf{g}}_{3D}$$

subject to altitude bounds:

$$p_z \;\in\; \bigl[z_{floor}(p_x, p_y) + z_{safety},\;\; z_{max}\bigr]$$

The leak position estimate remains:

$$\hat{\mathbf{p}}_{leak} = \mathbf{p}\!\left(\arg\max_{t \geq t_{detect}} C_{meas}(t)\right)$$

with 3D localisation error $\varepsilon = \|\hat{\mathbf{p}}_{leak} - \mathbf{p}_{leak,true}\|$.

---

## Key 3D Additions

- **Elevation-varying pipeline**: vertices with $z \in [0, 4]$ m produce inclined segments; the Frenet binormal $\hat{B}_i$ is no longer $\hat{z}$.
- **Terrain-following patrol altitude**: stand-off $d_{so}$ measured along $\hat{B}_i$ rather than a global $z$ offset; the drone climbs and descends with the pipe.
- **Altitude-optimal sensor swath**: stand-off clamped to $\min(d_{so}^*, r_{max} - \epsilon_{margin})$ so the sensor footprint covers $w_{target}$ without losing detection range.
- **3D wind advection**: $\mathbf{u}_{wind}$ includes a vertical component $u_z \neq 0$, lifting or suppressing the plume and changing the optimal patrol altitude.
- **Full 3D gradient-ascent**: finite differences along $x$, $y$, and $z$ allow the drone to ascend or descend toward the plume peak rather than only spiralling horizontally.
- **Terrain DEM enforcement**: minimum safe altitude $z_{floor}(x,y) + z_{safety}$ replaces the fixed scalar $z_{min}$ of S072.

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Pipeline vertices (elevated) | $(0,0,0)$, $(50,5,2)$, $(100,-3,4)$, $(150,4,1)$, $(200,0,0)$ m |
| Pipeline total arc-length | $\approx 200$ m |
| Nominal stand-off | $d_{so} = 3.0$ m along $\hat{B}_i$ |
| Sensor cone half-angle | $\phi = 15°$ |
| Target swath width | $w_{target} = 1.5$ m |
| Patrol speed | $v_{patrol} = 2.0$ m/s |
| Altitude range | $z \in [0.5,\; 8.0]$ m above ground |
| Safety clearance above terrain | $z_{safety} = 0.5$ m |
| 3D wind velocity | $\mathbf{u}_{wind} = (0.5, 0.2, 0.15)$ m/s |
| Emission rate | $\dot{Q} = 1.0$ g/s |
| Puff mass | $Q_{puff} = 0.5$ g |
| Turbulent diffusion coefficient | $D = 0.5$ m²/s |
| Puff release interval | $\Delta t_{puff} = 0.5$ s |
| Sensor noise std dev | $\sigma_{sensor} = 5 \times 10^{-4}$ (normalised) |
| Detection threshold | $C_{thresh} = 0.01$ (normalised) |
| 3D gradient probe offset | $\delta = 0.5$ m |
| Gradient-ascent speed | $v_{ascent} = 1.0$ m/s |
| Localisation timeout | $T_{max,loc} = 120$ s |
| GPS noise std dev | $\sigma_{GPS} = 0.1$ m |
| Simulation timestep | $\Delta t_{sim} = 0.05$ s |

---

## Expected Output

- **3D scene plot** (`mpl_toolkits.mplot3d`): elevated pipeline polyline in grey with terrain mesh underneath; nominal stand-off patrol path in green; actual drone trajectory colour-mapped by phase (blue = terrain-following patrol, orange = 3D gradient-ascent localisation); true leak position as a red sphere; estimated leak position as a white diamond; 3D wind vector arrow; altitude bounds shown as transparent horizontal planes.
- **Altitude time series**: drone $z(t)$ vs time showing elevation changes as the drone follows the pipe profile; terrain floor $z_{floor}(t)$ shown as a shaded baseline; safety clearance margin annotated; detection event marked with a vertical dashed line.
- **Stand-off and swath width vs arc-length**: plots of $d_{so}(s)$, footprint width $w(s)$, and the target $w_{target}$ for the altitude-optimal strategy; highlights segments where terrain forces stand-off clamping.
- **3D plume snapshot**: volumetric concentration isosurface (or 3D scatter coloured by $C$) at the moment of detection; vertical plume centroid drift due to $u_z$ visible as upward displacement from the pipe level.
- **Localisation convergence in 3D**: trajectory of the drone during gradient ascent plotted in 3D with a colour ramp from detection to best estimate; altitude profile shows whether the drone rises or descends toward the plume peak.
- **Strategy comparison table**: detection time, 3D localisation error, minimum terrain clearance, and mean swath coverage for all four comparison strategies.
- **Monte Carlo error histograms** ($N_{MC} = 30$ trials): 3D localisation error $\varepsilon$ distribution; separate bars for horizontal $\varepsilon_{xy}$ and vertical $\varepsilon_z$ error components to diagnose 3D gradient quality.
- **Animation (GIF)**: rotating 3D view showing the drone climbing and descending along the elevated pipe, then executing 3D gradient ascent toward the leak; plume concentration shown as a semi-transparent 3D cloud evolving with wind; mode label (PATROL / LOCALISE 3D) in corner.

---

## Extensions

1. **Steep-slope terrain**: increase pipeline inclination to $\psi > 30°$ on one segment; evaluate how the Frenet stand-off law handles near-vertical pipe sections and whether the sensor swath model degrades.
2. **3D wind estimation**: replace the known constant $\mathbf{u}_{wind}$ with an online estimator that back-computes wind from observed puff centroid drift; assess localisation error as a function of wind-estimation latency.
3. **Multi-elevation leak scenario**: place two leaks at different elevations ($z_{leak,1} = 0.5$ m, $z_{leak,2} = 3.5$ m); the 3D gradient-ascent must distinguish which altitude layer contains the stronger source.
4. **Terrain-occlusion shadow zones**: add a ridge that blocks the plume from reaching the patrol side of the pipeline; evaluate cast-and-surge strategies adapted to 3D terrain geometry.
5. **RL localisation in 3D**: train a PPO agent with state $\bigl(C_{meas},\;\hat{\nabla}_{3D}C,\;\mathbf{p}_{drone},\;z_{floor}\bigr)$ and continuous 3D velocity action; compare against the gradient-ascent heuristic across terrain configurations.
6. **Fixed-wing adaptation**: replace the multirotor with a fixed-wing UAV constrained to $v_{min} = 8$ m/s and minimum turn radius $R_{min} = 5$ m; redesign the localisation spiral as a banked orbit converging on the leak.

---

## Related Scenarios

- Original 2D version: [S072 Pipeline Leak Detection](../S072_pipeline_leak.md)
- Terrain-following patrol reference: [S061 Power Line Inspection](../S061_power_line.md) (Frenet-frame pipe following), [S071 Bridge Inspection](../S071_bridge_inspection.md) (complex 3D geometry patrol)
- 3D plume model reference: [S045 Chemical Plume Tracing](../../03_environmental_sar/S045_plume_tracing.md), [S056 Radiation Hotspot Detection](../../03_environmental_sar/S056_radiation.md)
- Pipeline follow-up: [S073 Solar Thermal Inspection](../S073_solar_thermal.md), [S080 Underground Pipe](../S080_underground_pipe.md)
