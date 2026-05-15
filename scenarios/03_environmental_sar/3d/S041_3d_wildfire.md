# S041 3D Upgrade — Wildfire Boundary Scan

**Domain**: Environmental & SAR | **Difficulty**: ⭐⭐⭐ | **Status**: `[ ]` Not started
**Based on**: [S041 original](../S041_wildfire_boundary.md)

---

## What Changes in 3D

The original S041 fixes all three drones at a single scan altitude of $z = 10$ m throughout the
entire mission. The fire front is modelled as a purely 2D expanding polygon on a flat ground plane,
and the IR sensor footprint is a horizontal disc with no dependence on altitude. This 2D abstraction
ignores three physical realities that matter for real wildfire surveillance:

1. **Smoke column height**: wildfire smoke and superheated gas form a volumetric plume that rises
   to 50–200 m. A drone flying through or below the plume loses IR visibility of the ground
   boundary and risks sensor saturation or structural damage.
2. **Terrain relief**: real wildland-urban interface terrain has elevation gradients of 5–30 m
   across a 200 m patch; drones must follow terrain to maintain a constant altitude-above-ground
   (AGL) sensor footprint.
3. **Vertical fire structure**: the fire front is not a thin 2D edge but a volumetric combustion
   zone with active flame height $h_{flame}(t)$ that changes the safe approach angle.

This variant adds: altitude-layered smoke sensing with a safe-altitude floor above the smoke column,
terrain-following AGL control for constant footprint size, and a volumetric fire-front model that
tracks both ground spread and flame height.

---

## Problem Definition

**Setup**: The arena is a $200 \times 200$ m patch of terrain with a continuous elevation map
$z_{terrain}(x, y)$ modelled as a smooth ridge function. An active wildfire occupies a region
$\mathcal{F}(t) \subset \mathbb{R}^2$ that expands according to the elliptical Rothermel model from
S041. Each burning cell additionally produces a smoke/heat column of height:

$$h_{smoke}(x, y, t) = h_{flame}(t) + \Delta h_{plume}(x, y, t)$$

where $h_{flame}(t)$ is the instantaneous flame height (m above ground) and $\Delta h_{plume}$ is
the thermal plume height derived from a Gaussian dispersion model.

A fleet of $N = 3$ drones carries downward-facing IR sensors and must track the moving fire
boundary while respecting:
- **AGL altitude constraint**: each drone maintains $z_{AGL,k}(t) = z_{scan}$ above its ground
  projection, i.e. $z_k(t) = z_{terrain}(x_k, y_k) + z_{scan}$.
- **Safe-altitude floor**: the drone must fly above the smoke column,
  $z_k(t) \geq z_{terrain}(x_k, y_k) + h_{smoke}(x_k, y_k, t) + z_{safety}$.
- **3D sensor cone**: the IR footprint on the ground is now a function of both the sensor
  half-angle $\alpha_s$ and the instantaneous AGL altitude.

**Roles**:
- **Fire** (environment): a volumetric combustion region with a 2D ground-spread footprint
  $\mathcal{F}(t)$ and a time-varying flame height $h_{flame}(t)$.
- **Smoke plume** (environment): a Gaussian dispersion column above each burning cell that
  defines the safe-altitude floor.
- **Drones** ($N = 3$): agents flying at terrain-following AGL altitude, each carrying a
  downward-pointing IR cone sensor of half-angle $\alpha_s = 15°$.

**Objective**: Minimise the mean Hausdorff distance $d_H(t)$ between the true fire perimeter and
the fleet's boundary estimate while satisfying the AGL altitude and smoke-clearance constraints at
all times.

**Key question**: How does the safe-altitude floor imposed by the smoke column reduce effective IR
sensor footprint coverage, and what fleet density is needed to compensate?

---

## Mathematical Model

### Terrain Model

The terrain elevation is a smooth ridge function:

$$z_{terrain}(x, y) = A_{ridge} \cdot \exp\!\left(-\frac{(x - x_c)^2}{2\sigma_x^2}\right) \cdot \cos\!\left(\frac{\pi (y - y_c)}{L_y}\right)$$

with $A_{ridge} = 8$ m, $\sigma_x = 40$ m, $L_y = 200$ m, so terrain relief spans approximately
$\pm 8$ m across the arena.

### Volumetric Fire Front

The ground-spread fire occupancy $F(x, y, t) \in \{0, 1\}$ follows the same elliptical Huygens
model as S041. Additionally, each burning cell generates a flame of height:

$$h_{flame}(t) = h_0 + k_{wind} \cdot \|\mathbf{U}_{wind}\|$$

where $h_0 = 3$ m is the calm-wind flame height and $k_{wind} = 0.5$ m/(m/s) is the wind
amplification coefficient. The thermal plume above each burning cell $(x_b, y_b)$ disperses as:

$$\Delta h_{plume}(x, y, t) = h_{rise} \cdot \exp\!\left(-\frac{(x-x_b)^2 + (y-y_b)^2}{2\sigma_{plume}^2}\right)$$

with $h_{rise} = 15$ m and $\sigma_{plume} = 5$ m. The total safe-altitude floor at position
$(x, y)$ is:

$$z_{floor}(x, y, t) = z_{terrain}(x, y) + \max_{(x_b,y_b) \in \mathcal{F}(t)} \Delta h_{plume}(x, y, t) \cdot \mathbf{1}[F(x_b,y_b,t)=1] + h_{flame}(t) + z_{safety}$$

with $z_{safety} = 2$ m clearance above the plume top.

### AGL Altitude Controller

Each drone $k$ has a 3-DOF state $\mathbf{p}_k = (x_k, y_k, z_k)^T$. The commanded altitude is:

$$z_{cmd,k}(t) = \max\!\left(z_{terrain}(x_k, y_k) + z_{scan},\; z_{floor}(x_k, y_k, t)\right)$$

Altitude is tracked by a first-order controller with time constant $\tau_z = 1.0$ s:

$$\dot{z}_k = \frac{z_{cmd,k} - z_k}{\tau_z}$$

The effective AGL altitude at any moment is $z_{AGL,k}(t) = z_k(t) - z_{terrain}(x_k, y_k)$.

### 3D IR Sensor Cone Model

The sensor footprint on the ground is a disc whose radius depends on the current AGL altitude:

$$r_{foot,k}(t) = z_{AGL,k}(t) \cdot \tan(\alpha_s)$$

with half-angle $\alpha_s = 15°$, so at the nominal $z_{scan} = 10$ m AGL the footprint radius
equals $r_{foot} = 10 \cdot \tan(15°) \approx 2.68$ m. When the safe-altitude floor forces the
drone higher (e.g., $z_{AGL,k} = 20$ m above the smoke column), the footprint expands to
$\approx 5.36$ m but the IR signal-to-noise degrades because the radiant flux density falls as
$z_{AGL}^{-2}$:

$$R_{sensor,k}(t) = R_{sensor,0} \cdot \left(\frac{z_{AGL,k}(t)}{z_{scan}}\right)^2$$

This altitude-dependent noise variance is substituted directly into the IR observation model
from S041:

$$z_{ij} = T_{true}(\mathbf{x}_{ij}) + w_{ij}, \qquad w_{ij} \sim \mathcal{N}(0,\, R_{sensor,k}(t))$$

### 3D Boundary Gradient-Following Guidance

The horizontal guidance law is inherited from S041 (boundary-tangent + stand-off correction):

$$\dot{\mathbf{p}}_{k,xy} = v_{cruise} \cdot \hat{\mathbf{t}}_k + K_\phi \cdot \phi_k(t) \cdot \hat{\mathbf{n}}_k$$

The altitude command is handled separately by the AGL controller above, making the full 3D
commanded velocity:

$$\dot{\mathbf{p}}_k = \begin{pmatrix} \dot{x}_k \\ \dot{y}_k \\ \dot{z}_k \end{pmatrix} = \begin{pmatrix} v_{cruise}\,\hat{t}_{k,x} + K_\phi\,\phi_k\,\hat{n}_{k,x} \\ v_{cruise}\,\hat{t}_{k,y} + K_\phi\,\phi_k\,\hat{n}_{k,y} \\ (z_{cmd,k} - z_k)/\tau_z \end{pmatrix}$$

The horizontal speed is clipped to $v_{cruise}$ before applying; the vertical channel is
independent and not speed-clipped (vertical dynamics are typically slower than horizontal patrol).

### Safe-Approach Angle Constraint

When a drone approaches the fire boundary to initiate close patrol, it must descend from a safe
stand-off altitude to the scan AGL. The maximum descent approach angle is:

$$\gamma_{max} = \arctan\!\left(\frac{|\dot{z}_{max}|}{v_{cruise}}\right)$$

with $|\dot{z}_{max}| = 2.0$ m/s, giving $\gamma_{max} \approx 38.7°$ at $v_{cruise} = 2.5$ m/s.
This limits how quickly a drone can close altitude after being forced up by a smoke column.

### Performance Metrics

The 3D variant adds two new metrics alongside the Hausdorff distance $d_H(t)$ and coverage
freshness $\eta(t)$ from S041:

**Smoke-floor violation rate**:

$$V_{smoke}(t) = \frac{1}{N} \sum_{k=1}^{N} \mathbf{1}\!\left[z_k(t) < z_{floor}(x_k, y_k, t)\right]$$

**Mean effective footprint radius**:

$$\bar{r}_{foot}(t) = \frac{1}{N} \sum_{k=1}^{N} z_{AGL,k}(t) \cdot \tan(\alpha_s)$$

A large $\bar{r}_{foot}$ indicates drones were pushed high by smoke columns and are operating
with degraded IR precision.

---

## Key 3D Additions

- **Terrain-following AGL control**: altitude command $z_{cmd} = z_{terrain}(x, y) + z_{scan}$
  continuously recalculated from a bilinear-interpolated terrain map; first-order lag with
  $\tau_z = 1.0$ s.
- **Smoke plume safe-altitude floor**: Gaussian dispersion column above each burning cell imposes
  a height-dependent keep-out zone; drones automatically climb to clear the floor.
- **Altitude-dependent sensor cone**: footprint radius $r_{foot} = z_{AGL} \tan(\alpha_s)$ and
  noise variance $R_{sensor} \propto z_{AGL}^2$ both track the current AGL in real time.
- **Volumetric fire-front model**: flame height $h_{flame}(t)$ responds to wind speed, coupling
  the spread dynamics to the 3D safe-altitude constraint.
- **3D trajectory visualization**: 3D matplotlib axes with terrain surface mesh, drone 3D paths,
  and altitude-time subplots showing AGL excursions above the smoke floor.

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Arena size | $200 \times 200$ m |
| Terrain relief amplitude $A_{ridge}$ | 8 m |
| Terrain ridge width $\sigma_x$ | 40 m |
| Number of drones $N$ | 3 |
| Nominal scan AGL $z_{scan}$ | 10 m |
| IR sensor half-angle $\alpha_s$ | 15 deg |
| Nominal footprint radius $r_{foot}$ | $\approx 2.68$ m |
| IR noise variance at $z_{scan}$ ($R_{sensor,0}$) | 25 K$^2$ |
| Drone cruise speed $v_{cruise}$ | 2.5 m/s |
| Altitude time constant $\tau_z$ | 1.0 s |
| Maximum vertical rate $|\dot{z}_{max}|$ | 2.0 m/s |
| Calm-wind flame height $h_0$ | 3 m |
| Wind amplification $k_{wind}$ | 0.5 m/(m/s) |
| Plume rise height $h_{rise}$ | 15 m |
| Plume dispersion width $\sigma_{plume}$ | 5 m |
| Smoke safety clearance $z_{safety}$ | 2 m |
| Head-fire spread rate $R_0$ | 0.04 m/s |
| Ellipse eccentricity $e_w$ | 0.6 |
| Wind speed | 5.0 m/s at 45 deg (NE) |
| Initial fire radius $r_0$ | 20 m |
| Stand-off gain $K_\phi$ | 0.8 s$^{-1}$ |
| Lost-contact replan distance $d_{replan}$ | 3.0 m |
| Max coverage gap $\Delta t_{gap}$ | 30 s |
| Simulation timestep $\Delta t$ | 0.5 s |
| Simulation duration $T_{sim}$ | 300 s |
| Altitude bounds (absolute) | 0.5 – 40 m |

---

## Expected Output

- **3D terrain + trajectory plot**: Matplotlib 3D axes with the terrain surface mesh (grey), fire
  footprint projected onto terrain (orange patch), smoke plume iso-surfaces (translucent grey
  cone), and full 3D drone paths colour-coded by time (blue-early to red-late).
- **Altitude time series**: $z_k(t)$ for each drone overlaid with the instantaneous smoke-floor
  height $z_{floor}(x_k, y_k, t)$ and the terrain elevation $z_{terrain}(x_k, y_k)$; shaded
  region between terrain and smoke floor marks the keep-out zone.
- **AGL deviation plot**: $z_{AGL,k}(t) - z_{scan}$ showing how far each drone deviates from the
  nominal 10 m AGL due to terrain relief and smoke avoidance.
- **Effective footprint radius**: $\bar{r}_{foot}(t)$ vs time, annotated with moments when the
  smoke floor forces drones above $z_{scan}$ and footprint expands beyond the nominal value.
- **Hausdorff distance comparison**: $d_H(t)$ for the 2D flat-terrain baseline (S041) vs the 3D
  terrain-following variant; quantifies the additional tracking error introduced by smoke avoidance
  altitude excursions.
- **Coverage freshness $\eta(t)$**: fraction of boundary cells scanned within $\Delta t_{gap}$,
  showing degradation episodes when smoke columns force drones to high altitude.
- **Animation (GIF)**: rotating 3D view of terrain mesh with fire spreading across it, smoke
  columns rising above the boundary, and three drones following the terrain contour while climbing
  over smoke obstacles.

---

## Extensions

1. **Wind-driven plume tilt**: model the smoke plume as a tilted Gaussian cone displaced
   downwind rather than a vertical column; drones on the leeward side of the fire must climb
   higher to clear the tilted plume, creating asymmetric altitude profiles around the perimeter.
2. **Adaptive sensor altitude selection**: allow each drone to choose its own AGL altitude
   within $[z_{scan}, z_{scan,max}]$ to trade off footprint size against IR noise — formulate
   as a per-drone altitude optimisation that maximises the expected boundary detection probability
   at each patrol step.
3. **Slope-accelerated fire spread**: extend the Rothermel model with a terrain-slope correction
   factor $\phi_{slope}(\psi)$ where $\psi$ is the upslope angle; fire racing up the ridge will
   advance faster than the flat-terrain model predicts, testing whether the fleet can track the
   accelerated head-fire segment without losing boundary contact.

---

## Related Scenarios

- Original 2D version: [S041](../S041_wildfire_boundary.md)
- Terrain-following reference: [S003 Low Altitude Tracking](../../01_pursuit_evasion/S003_low_altitude_tracking.md)
- Volumetric plume sensing: [S045 Chemical Plume Tracing](../S045_plume_tracing.md)
- Dynamic boundary tracking: [S055 Oil Spill Tracking](../S055_oil_spill.md)
