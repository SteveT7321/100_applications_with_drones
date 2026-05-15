# S024 3D Upgrade — Wind Compensation

**Domain**: Logistics & Delivery | **Difficulty**: ⭐⭐⭐ | **Status**: `[ ]` Not Started
**Based on**: [S024 original](../S024_wind_compensation.md)

---

## What Changes in 3D

The original S024 fixes the drone at a constant cruise altitude of $z = 2.5$ m throughout
the flight; the wind field is a 2D horizontal vector $\mathbf{w}(t) \in \mathbb{R}^2$, and
the crab-angle correction operates only on the horizontal bearing. The altitude hold is a
simple decoupled proportional controller that is never stressed.

In this 3D upgrade:

- The delivery goal is a **rooftop waypoint** at $(40, 0, 15)$ m — the drone must
  simultaneously make forward progress, climb 12.5 m, and fight the wind.
- The wind field gains a **vertical component** (updraft/downdraft), modeled as a separate
  Gauss-Markov process with mean $\bar{w}_z = 0.4$ m/s and gust std $\sigma_{g,z} = 0.3$ m/s.
- The **altitude-varying wind shear** means that the mean crosswind magnitude changes with
  altitude — a drone climbing through the wind profile encounters an evolving disturbance.
- The **crab-angle feed-forward** is extended to 3D: it corrects both the horizontal heading
  (to null mean $w_y$) and the vertical pitch angle (to null mean $w_z$).
- The **3D PID** controls two independent cross-track error axes — lateral ($y$-perpendicular)
  and vertical ($z$-perpendicular) — and blends both corrections into a single airspeed
  command clipped to $v_{max}$.
- Performance is evaluated in both the lateral and vertical cross-track dimensions, and the
  final miss distance is the true 3D Euclidean distance to the rooftop goal.

---

## Problem Definition

**Setup**: A delivery drone departs from $\mathbf{p}_0 = (0, 0, 2.5)$ m and must reach a
rooftop landing pad at $\mathbf{p}_{goal} = (40, 0, 15)$ m. The environment contains a
persistent 3D wind field with mean horizontal crosswind $\bar{w}_y = 2.5$ m/s, mean updraft
$\bar{w}_z = 0.4$ m/s, and correlated gusts in all three axes. Three guidance strategies are
compared in full 3D.

**Roles**:
- **Drone (controlled agent)**: multirotor with first-order velocity dynamics; commands a 3D
  airspeed vector $\mathbf{v}_{air} \in \mathbb{R}^3$ with $\|\mathbf{v}_{air}\| \leq v_{max}$
- **3D wind field**: exogenous disturbance with horizontal and vertical stochastic components

**Objective**: reach $\mathbf{p}_{goal}$ within a terminal radius $r_{land} = 0.3$ m (3D);
compare:

1. **No compensation** — aim at the goal in 3D, ignore wind
2. **Feed-forward crab-angle + pitch correction** — analytically solve for heading and pitch
   offsets that null both mean horizontal and mean vertical wind components
3. **3D PID cross-track** — closed-loop feedback on both the lateral and vertical signed
   perpendicular distances from the nominal 3D straight-line path

---

## Mathematical Model

### Ground Velocity Kinematics (3D)

$$\dot{\mathbf{p}} = \mathbf{v}_{air} + \mathbf{w}(t), \quad \mathbf{p},\, \mathbf{v}_{air},\, \mathbf{w} \in \mathbb{R}^3$$

### 3D Wind Model

The total wind is the sum of constant mean and correlated gust:

$$\mathbf{w}(t) = \bar{\mathbf{w}} + \mathbf{w}_g(t), \quad \bar{\mathbf{w}} = \begin{bmatrix}0 \\ \bar{w}_y \\ \bar{w}_z\end{bmatrix}$$

Each component of the gust evolves as a first-order Gauss-Markov process:

$$w_{g,k+1}^{(i)} = e^{-\Delta t / \tau_g}\, w_{g,k}^{(i)} + \sqrt{1 - e^{-2\Delta t / \tau_g}}\;\sigma_g^{(i)}\;\varepsilon_k^{(i)}, \quad \varepsilon_k \sim \mathcal{N}(0, 1)$$

where $\sigma_g^{(i)}$ is $\sigma_{g,h} = 0.8$ m/s for horizontal axes and $\sigma_{g,v} = 0.3$ m/s for the vertical axis.

### Nominal Path and 3D Cross-Track Errors

The nominal path is the straight line from $\mathbf{p}_0$ to $\mathbf{p}_{goal}$ with unit
direction:

$$\hat{\mathbf{d}}_0 = \frac{\mathbf{p}_{goal} - \mathbf{p}_0}{\|\mathbf{p}_{goal} - \mathbf{p}_0\|}$$

The perpendicular deviation vector is:

$$\mathbf{p}_\perp = (\mathbf{p} - \mathbf{p}_0) - \bigl[(\mathbf{p} - \mathbf{p}_0) \cdot \hat{\mathbf{d}}_0\bigr]\hat{\mathbf{d}}_0$$

The signed lateral and vertical cross-track errors are the $y$- and $z$-projections of $\mathbf{p}_\perp$:

$$e_{lat} = \mathbf{p}_\perp \cdot \hat{\mathbf{e}}_y, \qquad e_{vert} = \mathbf{p}_\perp \cdot \hat{\mathbf{e}}_z$$

### Strategy 1 — No Compensation

$$\mathbf{v}_{air} = v_{max} \cdot \hat{\mathbf{d}}(t), \quad \hat{\mathbf{d}}(t) = \frac{\mathbf{p}_{goal} - \mathbf{p}(t)}{\|\mathbf{p}_{goal} - \mathbf{p}(t)\|}$$

The drone points directly at the goal in 3D but accumulates crosswind drift in $y$ and
updraft-induced altitude deviation in $z$.

### Strategy 2 — Feed-Forward Crab-Angle + Pitch Correction

**Horizontal crab angle** to null mean $\bar{w}_y$. For horizontal nominal bearing $\psi_0 = \arctan2(d_y, d_x)$:

$$\alpha = \arcsin\!\left(-\frac{\bar{w}_y}{v_{max}}\right)$$

**Vertical pitch correction** to null mean $\bar{w}_z$. For the current elevation angle $\gamma = \arctan2(d_z, \|\mathbf{d}_{xy}\|)$:

$$\beta = \arcsin\!\left(-\frac{\bar{w}_z}{v_{max}}\right)$$

The corrected airspeed command:

$$\mathbf{v}_{air} = v_{max}\begin{bmatrix}\cos(\gamma + \beta)\cos(\psi_0 + \alpha) \\ \cos(\gamma + \beta)\sin(\psi_0 + \alpha) \\ \sin(\gamma + \beta)\end{bmatrix}$$

This eliminates mean drift in both horizontal and vertical axes; gust residuals cause remaining
cross-track error.

### Strategy 3 — 3D PID Cross-Track Controller

Two independent SISO PIDs act on $e_{lat}$ and $e_{vert}$:

$$v_\perp^{lat} = k_{p,l}\,e_{lat} + k_{i,l}\int_0^t e_{lat}\,d\tau + k_{d,l}\,\dot{e}_{lat}$$

$$v_\perp^{vert} = k_{p,v}\,e_{vert} + k_{i,v}\int_0^t e_{vert}\,d\tau + k_{d,v}\,\dot{e}_{vert}$$

The airspeed command blends the goal-pointing direction with both correction terms:

$$\mathbf{v}_{air} = v_{max}\,\hat{\mathbf{d}} - v_\perp^{lat}\,\hat{\mathbf{n}}_{lat} - v_\perp^{vert}\,\hat{\mathbf{n}}_{vert}$$

where $\hat{\mathbf{n}}_{lat} = [-d_{hat,y},\, d_{hat,x},\, 0]^\top$ (left-of-track) and
$\hat{\mathbf{n}}_{vert} = \hat{\mathbf{e}}_z$, then rescaled to $\|\cdot\| \leq v_{max}$.

---

## Key 3D Additions

- **3D wind field**: separate horizontal ($\sigma_{g,h} = 0.8$ m/s) and vertical
  ($\sigma_{g,v} = 0.3$ m/s) Gauss-Markov gust components; mean updraft $\bar{w}_z = 0.4$ m/s
- **Climbing delivery path**: goal at altitude 15 m requires simultaneous forward progress
  and climb — the wind shear stress accumulates over the entire ascent
- **3D crab-angle**: horizontal heading correction $\alpha$ and vertical pitch correction $\beta$
  derived analytically from the mean wind components
- **Dual-axis PID**: lateral and vertical cross-track PIDs with independent gain sets;
  integral anti-windup clipping prevents steady-state windup during climb
- **3D cross-track decomposition**: perpendicular deviation projected onto $\hat{\mathbf{e}}_y$
  and $\hat{\mathbf{e}}_z$ separates horizontal drift from altitude deviation
- **Two-panel animation**: top-down XY view reveals lateral drift; side-view XZ view reveals
  altitude tracking quality

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Cruise speed $v_{max}$ | 5.0 m/s |
| Mean crosswind $\bar{w}_y$ | 2.5 m/s |
| Mean updraft $\bar{w}_z$ | 0.4 m/s |
| Gust std horizontal $\sigma_{g,h}$ | 0.8 m/s |
| Gust std vertical $\sigma_{g,v}$ | 0.3 m/s |
| Gust correlation time $\tau_g$ | 1.5 s |
| Control frequency | 48 Hz |
| Start position $\mathbf{p}_0$ | (0, 0, 2.5) m |
| Goal position $\mathbf{p}_{goal}$ | (40, 0, 15) m |
| Delivery distance (3D) | $\approx 42.1$ m |
| Arrival radius $r_{land}$ | 0.3 m (3D) |
| Lateral PID gains $(k_{p,l}, k_{i,l}, k_{d,l})$ | 1.2, 0.05, 0.3 |
| Vertical PID gains $(k_{p,v}, k_{i,v}, k_{d,v})$ | 1.5, 0.04, 0.2 |
| Simulation timeout $T_{max}$ | 90 s |
| z range | 2.5 – 18 m |

---

## Expected Output

Simulation code: `src/02_logistics_delivery/3d/s024_3d_wind_compensation.py`

Outputs: `outputs/02_logistics_delivery/s024_3d_wind_compensation/`

- **`trajectory_3d.png`**: 3D trajectory plot — three colored paths (no-comp gray, crab-angle
  orange, PID blue) ascending from origin to rooftop, with wind arrows overlaid on the
  $z = 2$ m plane; nominal dashed straight-line path for reference
- **`crosstrack_error.png`**: three-panel time series — lateral cross-track error,
  vertical cross-track error, and ground speed for all strategies
- **`lateral_bar.png`**: grouped bar chart with RMS lateral error, RMS vertical error, and
  final 3D miss distance for each strategy
- **`animation.gif`**: two-panel animated comparison — top-down XY view (lateral drift) and
  side XZ view (altitude tracking); time counter overlaid
- 3D PID reduces lateral RMS to $< 0.2$ m and vertical RMS to $< 0.3$ m; crab-angle strategy
  eliminates mean drift but retains gust residuals; no-compensation strategy accumulates
  several metres of lateral and vertical error over the 40 m climb

---

## Extensions

1. **Altitude-varying wind shear model**: replace constant $\bar{\mathbf{w}}$ with a
   logarithmic wind profile $w(z) = w_{ref}(z/z_{ref})^{1/7}$ and update the feed-forward
   correction at each altitude
2. **Online wind estimation**: implement a recursive least-squares estimator that tracks
   the mean wind vector online and feeds updated $\hat{\mathbf{w}}$ into the crab-angle
   correction
3. **Energy-optimal climb angle**: under headwind/tailwind conditions, vary the climb rate
   to trade altitude gain against speed loss; minimise total energy $\int_0^T (v_{air}^2 + k_h v_z^2)\,dt$
4. **Combine with S031 path deconfliction**: multiple drones flying similar rooftop delivery
   routes in wind; manage both wind compensation and separation simultaneously

---

## Related Scenarios

- Original 2D version: [S024](../S024_wind_compensation.md)
- Prerequisites: [S021](../S021_point_delivery.md), [S022](../S022_obstacle_avoidance_delivery.md)
- Follow-ups: [S034](../S034_weather_rerouting.md), [S031](../S031_path_deconfliction.md)
- Other 3D logistics upgrades: [S022 3D](S022_3d_obstacle_avoidance.md), [S027 3D](S027_3d_aerial_refueling.md)
