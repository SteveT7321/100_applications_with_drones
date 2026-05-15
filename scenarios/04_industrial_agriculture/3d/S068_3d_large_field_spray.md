# S068 3D Upgrade — Large-Scale Field Spraying

**Domain**: Industrial & Agriculture | **Difficulty**: ⭐⭐⭐⭐ | **Status**: `[ ]` Not Started
**Based on**: [S068 original](../S068_large_field_spray.md)

---

## What Changes in 3D

The original S068 is strictly 2D: all four drones fly at a fixed, implicit altitude of
$z = 2.0$ m throughout the mission, the spray nozzle is modelled as a 1D line segment in the
x-y plane, and the coverage grid is a 2D $M_x \times M_y$ boolean array. Three physically
significant phenomena are therefore missing:

1. **Terrain following**: real farmland is not flat. Each drone must independently track a
   digital elevation model (DEM) and maintain a constant spray altitude $h_{AGL}$ above ground
   level, not above the datum. Without this, the effective swath width $w_{spray}$ varies
   with terrain slope because the nozzle-to-canopy distance changes.

2. **Altitude coordination between adjacent drones**: when two drones fly adjacent strips
   simultaneously, their rotor wash fields can overlap in the vertical direction if both drones
   are at the same altitude. The 3D upgrade adds an **altitude stagger** protocol: odd-indexed
   drones fly at $h_{AGL} + \Delta h_{stagger}$ to keep their wash envelopes separate.

3. **3D rotor wash interference model**: the downwash from a hovering or slow-flying
   agricultural drone is a turbulent jet whose velocity decays with distance. In 3D this jet
   has a cylindrical core of radius $r_{core}$ and an outer mixing region; the intersection
   of two wash cones determines the spray quality degradation at any field point.

---

## Problem Definition

**Setup**: A $200 \times 100$ m field is defined over a sinusoidal terrain DEM:

$$z_{terrain}(x, y) = A_{DEM} \sin\!\left(\frac{2\pi x}{\lambda_x}\right) \cos\!\left(\frac{2\pi y}{\lambda_y}\right)$$

with amplitude $A_{DEM} = 1.5$ m and wavelengths $\lambda_x = 80$ m, $\lambda_y = 120$ m.
The four cooperative drones must each maintain a constant spray altitude above the canopy:

$$z_i(t) = z_{terrain}(x_i(t),\, y_i(t)) + h_{AGL} + \Delta h_{stagger,i}$$

where $h_{AGL} = 2.0$ m is the nominal spray height and
$\Delta h_{stagger,i} = (i \bmod 2) \cdot \Delta h$ is the inter-drone altitude offset
($\Delta h = 0.4$ m for adjacent strips).

All other mission parameters (field size, number of drones, battery limits, spray swath, cruise
speed, charging station) carry over from S068. Each drone still executes a boustrophedon sweep
within its $50 \times 100$ m strip, but waypoints are now 3D tuples
$\mathbf{w}_k = (x_k, y_k, z_{terrain}(x_k, y_k) + h_{AGL} + \Delta h_{stagger,i})$.

**Roles**:
- **Drones** ($N = 4$): agricultural UAVs with altitude-tracking capability. Each drone
  maintains a 3D waypoint queue with terrain-adjusted $z$ coordinates and applies the altitude
  stagger offset based on its strip index.
- **Charging station**: located at $\mathbf{p}_{sta} = (0, 50, z_{terrain}(0,50))$ m.
  Battery swap time $T_{charge} = 120$ s; unlimited capacity.
- **Field**: $200 \times 100$ m with DEM-defined terrain; coverage grid extended to a 3D
  spray-deposit model where effective coverage depends on nozzle-to-canopy distance.

**Objective**: Minimise total mission time $T_{mission}$ while maintaining:
- Spray altitude error $|z_i - z_{i,cmd}| < \epsilon_z = 0.1$ m at all times.
- Wash interference index $\Phi < \Phi_{max} = 0.15$ between any two adjacent drones.
- Full field coverage $C(T_{mission}) = 1.0$.

---

## Mathematical Model

### 3D Terrain-Following Waypoint Generation

The terrain-adjusted altitude command for drone $i$ at horizontal position $(x, y)$ is:

$$z_{cmd,i}(x, y) = z_{terrain}(x, y) + h_{AGL} + (i \bmod 2)\,\Delta h$$

The vertical rate required to follow the terrain while moving at cruise speed $v$ is:

$$\dot{z}_{cmd} = \frac{\partial z_{terrain}}{\partial x}\,\dot{x} + \frac{\partial z_{terrain}}{\partial y}\,\dot{y}$$

where the partial derivatives of the DEM are:

$$\frac{\partial z_{terrain}}{\partial x} = \frac{2\pi A_{DEM}}{\lambda_x} \cos\!\left(\frac{2\pi x}{\lambda_x}\right) \cos\!\left(\frac{2\pi y}{\lambda_y}\right)$$

$$\frac{\partial z_{terrain}}{\partial y} = -\frac{2\pi A_{DEM}}{\lambda_y} \sin\!\left(\frac{2\pi x}{\lambda_x}\right) \sin\!\left(\frac{2\pi y}{\lambda_y}\right)$$

The altitude tracking error for drone $i$ at time $t$ is:

$$\epsilon_{z,i}(t) = z_i(t) - z_{cmd,i}(x_i(t), y_i(t))$$

A proportional altitude controller closes the loop:

$$\ddot{z}_i = K_z \bigl(z_{cmd,i} - z_i\bigr) - K_{zd}\,\dot{z}_i, \quad K_z = 4.0\;\text{s}^{-2},\; K_{zd} = 2.0\;\text{s}^{-1}$$

### 3D Swath Width Correction

Because the nozzle sprays downward, the actual ground-level swath width depends on the
perpendicular distance from nozzle to canopy. Let $d_{noz}(t)$ be the instantaneous
nozzle-to-canopy distance projected along the gravity vector:

$$d_{noz}(t) = z_i(t) - z_{terrain}(x_i(t), y_i(t))$$

The effective swath radius at the canopy surface for a conical spray with half-angle $\theta_{cone}$ is:

$$r_{swath}(t) = d_{noz}(t) \tan\theta_{cone}, \quad \theta_{cone} = 27.5°$$

yielding an effective swath width:

$$w_{eff}(t) = 2\,r_{swath}(t) = 2\,d_{noz}(t)\tan\theta_{cone}$$

At nominal $d_{noz} = h_{AGL} = 2.0$ m this recovers $w_{eff} \approx 2.0$ m.
Terrain undulations that shift $d_{noz}$ by $\pm 0.3$ m produce a $\pm 15\%$ swath
variation — significant for dose uniformity.

### Altitude Stagger Protocol

Adjacent strips $i$ and $i+1$ are assigned altitudes:

$$h_{AGL,i} = h_{AGL} + (i \bmod 2)\,\Delta h$$

so that even-indexed drones fly at $h_{AGL} = 2.0$ m and odd-indexed drones fly at
$h_{AGL} + \Delta h = 2.4$ m. This guarantees a minimum vertical separation:

$$\delta z_{ij} = |h_{AGL,i} - h_{AGL,j}| = \Delta h = 0.4 \text{ m}$$

between every adjacent pair $(i, j)$ with $|i - j| = 1$. Non-adjacent drones share the same
stagger parity and their horizontal separation is sufficient to prevent wash overlap.

### 3D Rotor Wash Interference Model

The downwash from drone $i$ at position $\mathbf{p}_i = (x_i, y_i, z_i)$ is modelled as an
axisymmetric Gaussian jet directed downward. The wash velocity magnitude at a field point
$\mathbf{q} = (x, y, z_{terrain}(x,y))$ is:

$$v_{wash,i}(\mathbf{q}) = V_0 \exp\!\left(-\frac{\rho_{i}^2}{2\sigma_{wash}^2} - \frac{(\Delta z_i)^2}{2\sigma_z^2}\right)$$

where:
- $V_0 = 8.0$ m/s — peak wash velocity at the rotor disc
- $\rho_i = \sqrt{(x - x_i)^2 + (y - y_i)^2}$ — horizontal distance from drone $i$
- $\Delta z_i = z_i - z_{terrain}(x, y)$ — nozzle height above the field point
- $\sigma_{wash} = 0.6$ m — horizontal wash spread radius
- $\sigma_z = 1.5$ m — vertical wash decay length

The wash interference index between drones $i$ and $j$ at their midpoint is:

$$\Phi_{ij} = \frac{v_{wash,i}(\mathbf{p}_j) + v_{wash,j}(\mathbf{p}_i)}{V_0}$$

The system-level interference index is:

$$\Phi(t) = \max_{|i-j|=1} \Phi_{ij}(t)$$

The altitude stagger is deemed sufficient when $\Phi(t) < \Phi_{max} = 0.15$ for all $t$.

### 3D Coverage Accounting

Coverage is now assessed on a 2D surface grid (the canopy surface) rather than a flat plane.
Cell $(m, n)$ is marked sprayed at time $t$ when:

$$r_{swath}(t) \geq \sqrt{(x_i(t) - x_m)^2 + (y_i(t) - y_n)^2}$$

The coverage fraction remains:

$$C(t) = \frac{|\{(m,n) : \text{cell sprayed by time }t\}|}{M_x \cdot M_y}$$

### 3D Mission Time and Battery Model

The 3D ferry distance from a mid-field position back to the charging station now includes
the vertical component:

$$d_{3D,return} = \|\mathbf{p}_i(t) - \mathbf{p}_{sta}\|_3 = \sqrt{(x_i - p_{sta,x})^2 + (y_i - p_{sta,y})^2 + (z_i - z_{sta})^2}$$

The battery return trigger is updated accordingly:

$$t_{elapsed} + \frac{d_{3D,return}}{v} \geq T_{bat}$$

The overall mission time formula is unchanged:

$$T_{mission} = \max_{i \in \{0,\ldots,N-1\}} T_i$$

---

## Key 3D Additions

- **Terrain-following altitude**: each drone independently tracks $z_{cmd,i}(x,y) = z_{terrain}(x,y) + h_{AGL} + \Delta h_{stagger,i}$ via a second-order altitude controller.
- **Altitude stagger protocol**: even/odd strip drones fly at 2.0 m / 2.4 m AGL to prevent wash overlap between adjacent strips; $\Delta h = 0.4$ m.
- **3D wash interference model**: axisymmetric Gaussian jet; interference index $\Phi_{ij}$ between adjacent drones must remain below 0.15.
- **Conical spray geometry**: swath width $w_{eff}(t) = 2\,h_{AGL,eff}(t)\tan\theta_{cone}$ varies with terrain height, tracked per timestep.
- **3D ferry distance**: return-to-station distance and battery trigger use the full 3D Euclidean distance.
- **3D trajectory visualisation**: Matplotlib 3D axes showing all four drone paths coloured over the DEM surface.

---

## Key Parameters

| Parameter | Symbol | Value |
|-----------|--------|-------|
| Field dimensions | $W_{total} \times D$ | 200 × 100 m |
| Number of drones | $N$ | 4 |
| DEM amplitude | $A_{DEM}$ | 1.5 m |
| DEM wavelengths | $\lambda_x,\, \lambda_y$ | 80 m, 120 m |
| Nominal spray altitude AGL | $h_{AGL}$ | 2.0 m |
| Altitude stagger offset | $\Delta h$ | 0.4 m |
| Spray cone half-angle | $\theta_{cone}$ | 27.5° |
| Nominal swath width | $w_{spray}$ | 2.0 m |
| Peak wash velocity | $V_0$ | 8.0 m/s |
| Wash horizontal spread | $\sigma_{wash}$ | 0.6 m |
| Wash vertical decay | $\sigma_z$ | 1.5 m |
| Max interference index | $\Phi_{max}$ | 0.15 |
| Altitude controller gain | $K_z$ | 4.0 s$^{-2}$ |
| Altitude damping gain | $K_{zd}$ | 2.0 s$^{-1}$ |
| Altitude tracking tolerance | $\epsilon_z$ | 0.1 m |
| Cruise speed | $v$ | 3.0 m/s |
| Battery flight time | $T_{bat}$ | 600 s |
| Battery swap time | $T_{charge}$ | 120 s |
| Charging station | $\mathbf{p}_{sta}$ | $(0, 50, z_{ter})$ m |
| Coverage grid resolution | $\Delta$ | 0.5 m |
| Altitude range | $z$ | 1.5 – 5.0 m |
| Simulation timestep | $\Delta t$ | 0.2 s |

---

## Expected Output

- **3D trajectory plot**: Matplotlib 3D axes showing the DEM surface mesh in grey, all four drone flight paths in distinct colours (red, orange, blue, purple), and altitude stagger visually evident between adjacent strips.
- **Altitude vs time**: four time series (one per drone) showing terrain-following $z_i(t)$ against the commanded $z_{cmd,i}(t)$; tracking error $|\epsilon_{z,i}|$ should remain below 0.1 m throughout.
- **Wash interference index vs time**: $\Phi(t)$ for each adjacent pair $(0,1)$, $(1,2)$, $(2,3)$; horizontal dashed line at $\Phi_{max} = 0.15$; all curves should stay below this threshold once stagger is applied.
- **Effective swath width vs time**: $w_{eff,i}(t)$ for each drone illustrating how terrain slope modulates the spray footprint; reference line at $w_{spray} = 2.0$ m.
- **Coverage map**: top-down view of the field with green heatmap on the canopy surface showing dose uniformity; strip boundaries as dashed lines; charging station as gold star.
- **Coverage progress curve**: $C(t)$ vs wall-clock time $t$; compared to the 2D flat-terrain baseline from S068 to quantify the overhead from terrain following and stagger.
- **Per-drone time breakdown**: stacked bar chart of flight, ferry, and charging time per drone; mission time $T_{mission}$ marked as a dashed line.
- **Animation (GIF)**: oblique 3D view of all four drones following the undulating terrain; coloured trajectory tails; live altitude stagger visible; wash interference zones rendered as translucent cylinders between adjacent drones.

---

## Extensions

1. **Adaptive stagger from live wash sensing**: replace the fixed $\Delta h = 0.4$ m offset with a feedback loop that reads $\Phi_{ij}$ in real time and adjusts $h_{AGL,j}$ dynamically to maintain $\Phi < \Phi_{max}$ while minimising spray-height deviation from nominal.
2. **Irregular terrain with obstacle avoidance**: add tree rows or irrigation channels as 3D obstacles; re-route boustrophedon paths around obstacles using RRT* in the 3D slice at the drone's altitude layer.
3. **Wind field integration**: add a 3D wind vector $\mathbf{v}_{wind}(x,y,z)$ that tilts the spray cone; compute the shifted spray footprint centroid and adjust lateral run spacing to compensate for wind-induced drift in each altitude layer.
4. **Multi-rotor wash interference suppression via phase-offset scheduling**: stagger the drone launch times so that adjacent drones are at opposite ends of their strips when flying in parallel, maximising horizontal separation and eliminating the need for altitude stagger.
5. **Dose uniformity optimisation**: formulate a spray-rate modulation problem where each drone varies flow rate as a function of $w_{eff}(t)$ to maintain constant dose $D_{dose}$ (L/m²) regardless of terrain-induced swath variation.
6. **3D battery energy model**: replace the flight-time battery model with a power consumption model $P = P_0 + k_v v^2 + k_z \dot{z}^2$ that penalises climbing; optimise the terrain-following gain $K_z$ to trade altitude accuracy against battery consumption.

---

## Related Scenarios

- Original 2D version: [S068](../S068_large_field_spray.md)
- 3D terrain-following reference: [S003 Low-Altitude Tracking](../../01_pursuit_evasion/S003_low_altitude_tracking.md)
- Adjacent 3D agriculture: [S063 Orchard Row Spraying](../S063_orchard_row_spraying.md), [S067 Spray Overlap Optimisation](../S067_spray_overlap_optimization.md)
- Multi-drone coordination: [S049 Dynamic Zone Assignment](../../03_environmental_sar/S049_dynamic_zone.md)
- Wash interference background: [S070 Precision Variable-Rate Spraying](../S070_precision_variable_rate_spraying.md)
