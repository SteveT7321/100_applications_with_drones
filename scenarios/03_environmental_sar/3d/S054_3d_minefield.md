# S054 3D Upgrade — Minefield Detection

**Domain**: Environmental & SAR | **Difficulty**: ⭐⭐⭐⭐ | **Status**: `[ ]` Not Started
**Based on**: [S054 original](../S054_minefield.md)

---

## What Changes in 3D

The original S054 fixes the survey drone at `z = 3.0 m` throughout the entire mission
(`drone_pos` is a 2D array; altitude never appears in the GPR probability calculation). This
is physically inaccurate for two reasons:

1. **GPR sensitivity is altitude-dependent.** A ground-penetrating radar's effective detection
   radius shrinks and its probability of detection degrades as the drone climbs away from the
   ground surface. The 2D model uses a flat $r_{gpr} = 4$ m radius regardless of altitude.

2. **Terrain is not flat.** Real minefields lie on uneven ground. A fixed-altitude flight path
   (constant world-frame $z$) produces a varying above-ground-level (AGL) altitude, which
   further degrades GPR performance on slopes and ridges.

This 3D upgrade adds:
- An **altitude-sensitive GPR gain model** that makes $P_d$ a function of both horizontal
  range and AGL altitude.
- An **optimal scan altitude** derivation that maximises the footprint-weighted detection
  probability per unit of ground area.
- A **terrain-following controller** that maintains a precise constant AGL altitude
  $z_{AGL}^* $ over a digital elevation model (DEM), replacing the fixed world-frame $z$.
- A **multi-pass 3D anomaly confirmation** protocol: a first-pass alarm at any altitude
  triggers a second low-altitude confirmation pass directly over the suspect cell to achieve
  the required $\theta_{confirm}$ threshold.

---

## Problem Definition

**Setup**: A $200 \times 200$ m suspected minefield with an underlying digital elevation model
(DEM) must be surveyed before ground personnel can enter. The terrain height is described by
$h_{terrain}(x, y)$ (metres above datum), with relief $\Delta h \leq 5$ m across the field.
A single drone equipped with a downward-facing GPR flies at a commanded AGL altitude
$z_{AGL}^{cmd}$ and builds a 3D probabilistic risk map. The drone's world-frame altitude is:

$$z(x, y) = h_{terrain}(x, y) + z_{AGL}^{cmd}$$

The GPR effective radius and detection probability now depend on $z_{AGL}$. An unknown number
of mines $N_{mine} \sim \mathcal{U}\{8, 20\}$ are buried at fixed $(x, y)$ positions with
their surface at $h_{terrain}(x, y)$; each mine is a point target unknown to the drone at
mission start.

The drone begins and ends at a **safe zone** entry point on the southern field boundary. The
mission has two operational phases:

1. **Survey phase** — boustrophedon coverage at the optimal scan altitude
   $z_{AGL}^* = \arg\max_{z} \eta(z)$ to maximise covered ground area per unit path length.
2. **Confirmation phase** — for each alarmed cell, a low-altitude confirmation pass at
   $z_{AGL}^{lo} = 1.5$ m directly over the suspect $(x, y)$ position to push cumulative
   $P_d^{cum} > \theta_{confirm} = 0.95$.

**Roles**:
- **Drone**: single GPR-equipped platform; 3D point-mass dynamics with terrain-following
  altitude controller; horizontal speed $v_{xy} = 3$ m/s; maximum vertical rate
  $\dot{z}_{max} = 1.5$ m/s.
- **Mines**: $N_{mine}$ fixed buried point targets; detectable only when the drone is within
  the 3D GPR detection envelope.
- **Safe Zone**: $10 \times 200$ m strip along the southern edge ($y \leq 10$ m);
  entry/exit waypoint at $(100, 0, h_{terrain}(100, 0) + z_{AGL}^{cmd})$.

**Comparison strategies**:
1. **Fixed altitude 2D baseline** — constant world-frame $z = 3$ m (same as S054 original);
   flat GPR model; no terrain following.
2. **Constant AGL, survey altitude** — terrain-following at $z_{AGL}^* $; altitude-sensitive
   GPR; single-pass survey only (no confirmation pass).
3. **Constant AGL with confirmation passes** — terrain-following at $z_{AGL}^* $ for survey,
   drops to $z_{AGL}^{lo} = 1.5$ m for each anomaly confirmation; full 3D anomaly protocol.

**Objective**: achieve coverage completeness $C \geq 0.95$ and mine confirmation rate
$D_{confirm} \geq 0.90$ while keeping the maintained escape-path risk $R_{path} < 0.1$.
Minimise total mission time subject to these constraints.

---

## Mathematical Model

### Altitude-Sensitive GPR Detection Model

The 3D range from the drone at world position $(x_d, y_d, z_d)$ to a mine surface point at
$(x_m, y_m, h_{terrain}(x_m, y_m))$ is:

$$\rho = \sqrt{(x_d - x_m)^2 + (y_d - y_m)^2 + (z_d - h_{terrain}(x_m, y_m))^2}$$

The AGL altitude of the drone above that surface point is:

$$z_{AGL} = z_d - h_{terrain}(x_d, y_d)$$

GPR effective detection radius at altitude $z_{AGL}$:

$$r_{gpr}(z_{AGL}) = r_0 \cdot \exp\!\left(-\frac{z_{AGL}}{H_{ref}}\right)$$

where $r_0 = 5.0$ m is the reference radius at ground level and $H_{ref} = 4.0$ m is the
altitude decay constant (approximately one antenna footprint width). At the nominal survey
altitude $z_{AGL}^{*}$ this gives $r_{gpr}(z_{AGL}^{*}) \approx 4$ m, matching the 2D model.

The instantaneous single-step detection probability:

$$P_d(\rho,\, z_{AGL}) = \left(1 - \exp\!\left(-\frac{r_{gpr}(z_{AGL})^2}{2\,\rho^2}\right)\right)
  \cdot G(z_{AGL})$$

where $G(z_{AGL})$ is the altitude gain factor:

$$G(z_{AGL}) = \exp\!\left(-\frac{z_{AGL}^2}{2\, H_{ref}^2}\right)$$

$G$ models the additional signal-to-noise degradation with altitude beyond the geometric
footprint shrinkage. At $z_{AGL} = 0$ we have $G = 1$ (maximum sensitivity); at
$z_{AGL} = H_{ref}$ we have $G \approx 0.61$.

### Optimal Scan Altitude

Define the detection efficiency $\eta(z_{AGL})$ as the expected ground area scanned per
metre of drone path at that altitude, accounting for both footprint size and detection
probability:

$$\eta(z_{AGL}) = \pi\, r_{gpr}(z_{AGL})^2 \cdot \bar{P}_d(z_{AGL})$$

where $\bar{P}_d(z_{AGL})$ is the mean detection probability averaged over the footprint
disc (horizon range $\rho \in [z_{AGL},\, \sqrt{z_{AGL}^2 + r_{gpr}^2}]$):

$$\bar{P}_d(z_{AGL}) = \frac{1}{\pi\, r_{gpr}^2}
  \int_0^{r_{gpr}(z_{AGL})} P_d\!\left(\sqrt{z_{AGL}^2 + s^2},\, z_{AGL}\right) 2\pi s\, ds$$

The optimal scan altitude is:

$$z_{AGL}^{*} = \arg\max_{z_{AGL} \in [z_{min},\, z_{max}]} \eta(z_{AGL})$$

with $z_{min} = 1.0$ m and $z_{max} = 6.0$ m. In practice $z_{AGL}^{*}$ is found by
numerical line-search (golden-section or scipy `minimize_scalar`); typical value is
$z_{AGL}^{*} \approx 2.5$ m.

### Terrain-Following Controller

The drone's commanded world-frame altitude at horizontal position $(x, y)$ is:

$$z_{cmd}(x, y) = h_{terrain}(x, y) + z_{AGL}^{cmd}$$

The vertical dynamics are governed by a first-order tracker with rate saturation:

$$\dot{z} = \text{clip}\!\left(\frac{z_{cmd} - z}{\tau_z},\; -\dot{z}_{max},\; \dot{z}_{max}\right)$$

with $\tau_z = 0.5$ s (altitude time constant) and $\dot{z}_{max} = 1.5$ m/s. Because
horizontal speed is constant at $v_{xy} = 3$ m/s, the achieved AGL error is bounded by:

$$\epsilon_{AGL} \leq \frac{\dot{z}_{max} \cdot \tau_z}{\cos\varphi}$$

where $\varphi$ is the terrain slope angle. For slopes $\varphi \leq 15°$ and the given
controller parameters, $\epsilon_{AGL} \leq 0.78$ m.

### 3D Drone Kinematics

Full 3D point-mass model:

$$\dot{\mathbf{p}} = \begin{bmatrix} v_{xy}\,\hat{\mathbf{u}}_{xy} \\ \dot{z} \end{bmatrix}$$

where $\hat{\mathbf{u}}_{xy}$ is the unit horizontal direction toward the next 2D waypoint:

$$\hat{\mathbf{u}}_{xy} = \frac{\mathbf{w}_{next}^{xy} - \mathbf{p}^{xy}}
                               {\|\mathbf{w}_{next}^{xy} - \mathbf{p}^{xy}\|}$$

and $\dot{z}$ is the terrain-following vertical rate defined above. The 3D state is
$\mathbf{p} = (x, y, z)^T \in \mathbb{R}^3$.

### 3D GPR Coverage Map

The 3D coverage accumulation for cell centre $(c_x, c_y)$ with surface height
$h_{terrain}(c_x, c_y)$:

$$P_d^{cum}(c_x, c_y) = 1 - \prod_{t=1}^{T}
  \bigl(1 - P_d(\rho_t,\, z_{AGL,t})\bigr)$$

where $\rho_t = \sqrt{(x_t - c_x)^2 + (y_t - c_y)^2 + (z_t - h_{terrain}(c_x, c_y))^2}$
and $z_{AGL,t} = z_t - h_{terrain}(x_t, y_t)$.

### Multi-Pass 3D Anomaly Confirmation

When a cell $(c_x, c_y)$ reaches alarm status ($P_d(r) > \theta_{alarm} = 0.5$ at any
single timestep), a confirmation waypoint is inserted at:

$$\mathbf{w}_{confirm} = (c_x,\; c_y,\; h_{terrain}(c_x, c_y) + z_{AGL}^{lo})$$

with $z_{AGL}^{lo} = 1.5$ m. The drone interrupts its current boustrophedon strip,
executes the detour, hovers for $t_{hover} = 2$ s, then resumes. The detour cost in time is:

$$\Delta t_{detour} = \frac{\|\mathbf{p}_{detour} - \mathbf{p}_{current}\|_{xy}}{v_{xy}}
  + \frac{|z_{AGL}^{lo} - z_{AGL}^{cmd}|}{\dot{z}_{max}} + t_{hover}$$

At $z_{AGL}^{lo} = 1.5$ m the peak single-step detection probability rises to:
$G(1.5) \approx 0.84$ and $r_{gpr}(1.5) \approx 4.7$ m, so $P_d^{cum}$ reaches
$\theta_{confirm} = 0.95$ within $\lceil 2\,r_{gpr}^{lo} / (v_{xy}\,\Delta t) \rceil$ steps.

### Risk Map and A* Replanning

Unchanged from the 2D model (S054) — risk is evaluated on the 2D horizontal grid using
confirmed/alarmed mine positions. The 3D drone position projects to the 2D grid by
$(x, y)$ only:

$$R(i,j) = \sum_{k \in \mathcal{M}_{active}} P_d^{cum}(\mathbf{m}_k)
  \cdot \exp\!\left(-\frac{\|c_{ij} - \mathbf{m}_k^{xy}\|^2}{2\, r_{influence}^2}\right)$$

A* escape path cost and replanning triggers are identical to S054.

---

## Key 3D Additions

- **Altitude-sensitive GPR model**: $P_d(\rho, z_{AGL}) = \bigl(1 - e^{-r_{gpr}(z_{AGL})^2 / 2\rho^2}\bigr) \cdot G(z_{AGL})$; both footprint radius and gain decay with altitude.
- **Optimal scan altitude search**: numerical maximisation of $\eta(z_{AGL}) = \pi r_{gpr}^2 \cdot \bar{P}_d$ over $z_{AGL} \in [1, 6]$ m; result $z_{AGL}^* \approx 2.5$ m.
- **Terrain-following controller**: rate-saturated first-order tracker on $z_{cmd}(x,y) = h_{terrain}(x,y) + z_{AGL}^{cmd}$; bounds AGL error to $< 0.78$ m for slopes $\leq 15°$.
- **Multi-pass confirmation detour**: low-altitude ($z_{AGL}^{lo} = 1.5$ m) confirmation pass inserted on first alarm; resumes boustrophedon strip after confirmation or timeout.
- **Full 3D trajectory**: $(x, y, z)$ state with 3D path visualisation and per-step AGL altitude logging.

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Field size | 200 × 200 m |
| Terrain relief $\Delta h$ | 0 – 5 m (DEM) |
| Drone horizontal speed $v_{xy}$ | 3 m/s |
| Maximum vertical rate $\dot{z}_{max}$ | 1.5 m/s |
| Altitude time constant $\tau_z$ | 0.5 s |
| AGL altitude range $[z_{min}, z_{max}]$ | 1.0 – 6.0 m |
| Reference GPR radius $r_0$ | 5.0 m |
| Altitude decay constant $H_{ref}$ | 4.0 m |
| Optimal scan altitude $z_{AGL}^*$ | $\approx$ 2.5 m (computed) |
| Confirmation altitude $z_{AGL}^{lo}$ | 1.5 m |
| Hover time at confirmation point $t_{hover}$ | 2 s |
| Alarm threshold $\theta_{alarm}$ | 0.50 |
| Confirmation threshold $\theta_{confirm}$ | 0.95 |
| False alarm probability $P_{fa}$ | 0.05 |
| Risk safe threshold $R_{safe}$ | 0.10 |
| Risk caution threshold $R_{caution}$ | 0.40 |
| A* risk penalty weight $\alpha$ | 50 |
| Number of mines $N_{mine}$ | $\mathcal{U}\{8, 20\}$, default 15 |
| Strip width | 8 m (based on $r_{gpr}(z_{AGL}^*)$) |
| Grid resolution | 1 × 1 m |
| Safe zone entry point | (100, 0) m |

---

## Expected Output

- **3D trajectory plot**: full $(x, y, z)$ drone path colour-coded by AGL altitude; terrain
  surface rendered as a semi-transparent mesh; mine positions marked at their surface height;
  confirmation detour legs highlighted in orange.
- **AGL altitude time series**: $z_{AGL}(t)$ for all three strategies; reference lines at
  $z_{AGL}^*$ (survey) and $z_{AGL}^{lo}$ (confirmation); terrain relief range shown as a
  shaded band.
- **Optimal altitude curve**: $\eta(z_{AGL})$ vs altitude plot showing the numerical optimum;
  annotated with $z_{AGL}^*$ and the corresponding $\bar{P}_d$ and footprint radius.
- **AGL error histogram**: distribution of $|z_{AGL}(t) - z_{AGL}^{cmd}|$ for the
  terrain-following strategy; theoretical bound $\epsilon_{AGL}$ annotated.
- **3D risk-map overlay**: top-down view of $R(i,j)$ heatmap with drone ground-track overlaid;
  confirmed mines (red circles), alarmed mines (orange triangles), confirmation detour
  waypoints (orange stars).
- **Coverage completeness vs time**: $C(t)$ for all three strategies; vertical tick marks at
  each confirmation detour event; horizontal dashed line at $C = 0.95$.
- **Confirmation efficiency panel**: per-mine bar chart of $P_d^{cum}$ at mission end; bars
  coloured by status (confirmed / alarmed / missed); detour time cost annotated per mine.
- **Strategy comparison table**: mission time, final coverage completeness, mines confirmed,
  mines missed, mean AGL error, number of detour events, total detour time, and mean escape
  path risk for all three strategies.
- **Animation (GIF)**: combined top-down risk map and side-view altitude profile panels,
  updated in real time; terrain profile shown in side view; drone marker moves in 3D; altitude
  commanded vs achieved plotted live.

---

## Extensions

1. **Slope-adaptive strip width**: increase $r_{gpr}$ and reduce strip spacing automatically
   on steep terrain segments where terrain-following error $\epsilon_{AGL}$ is large; recompute
   $N_{strips}$ dynamically to maintain $C \geq 0.95$.
2. **Sensor fusion in 3D**: combine GPR (vertical penetration) with a side-looking
   magnetometer (maximum sensitivity at low altitude); formulate a joint log-odds update
   $\Lambda(i,j) = \Lambda_{GPR} + \Lambda_{mag}$ and find the altitude that maximises the
   combined fused information gain $\mathbb{E}[\Lambda]$ per unit path length.
3. **Multi-drone 3D cooperative sweep**: assign $K = 3$ drones to non-overlapping terrain
   zones with altitude partitioning; one drone flies at $z_{AGL}^*$ for coverage while a
   second follows at $z_{AGL}^{lo}$ to confirm alarms raised by the first, reducing total
   mission time by eliminating detour interruptions.
4. **Stochastic DEM uncertainty**: treat $h_{terrain}(x,y)$ as a Gaussian process with known
   covariance; propagate DEM uncertainty into $z_{AGL}$ estimation error and then into the
   GPR probability model; update the terrain estimate in-flight using barometric altitude and
   a particle filter.
5. **Energy-aware altitude profile**: model power consumption as $P(v_{xy}, \dot{z}) =
   k_1 v_{xy}^2 + k_2 \dot{z}^2 + k_3$; find the altitude trajectory $z_{AGL}(t)$ that
   minimises total energy while maintaining $C \geq 0.95$ and $D_{confirm} \geq 0.90$.

---

## Related Scenarios

- Original 2D version: [S054](../S054_minefield.md)
- Terrain-following reference: [S003 Low Altitude Tracking](../../01_pursuit_evasion/S003_low_altitude_tracking.md)
- Probabilistic sensor model reference: [S056 Radiation Hotspot Detection](../S056_radiation_hotspot.md)
- Multi-drone coordination: [S049 Dynamic Zone Assignment](../S049_dynamic_zone_assignment.md)
- A* replanning under dynamic hazards: [S034 Weather Rerouting](../../02_logistics_delivery/S034_weather_rerouting.md)
