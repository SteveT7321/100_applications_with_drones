# S003 3D Upgrade — Low-Altitude Terrain Tracking

**Domain**: Pursuit & Evasion | **Difficulty**: ⭐⭐⭐ | **Status**: `[ ]` Not Started
**Based on**: [S003 original](../S003_low_altitude_tracking.md)

---

## What Changes in 3D

The original S003 models terrain as a single Gaussian hill in the x-y plane with a 1D altitude safety constraint (`z_safe = h_terrain(x,y) + 0.3`). The pursuer navigates only in 2D horizontally while bumping its z-command up over hills. Three critical limitations are ignored:

1. **Lateral terrain walls**: canyon walls, ridge sides, and cliff faces exert no repulsive force — the pursuer clips through vertical rock faces.
2. **Full DEM surface**: terrain is a scalar field $h(x,y)$ but lateral gradients $\partial h/\partial x$ and $\partial h/\partial y$ are never used for sidewall avoidance.
3. **Banking dynamics**: bank turns used during canyon navigation require roll coupling ($\phi \neq 0$), which a flat 2D-in-x-y model ignores entirely.

This 3D upgrade introduces a full terrain mesh (sampled from a 2D DEM), lateral terrain avoidance via surface normal repulsion, side-canyon navigation with roll-coordinated bank turns, and 6-DOF control that couples roll to lateral acceleration.

---

## Problem Definition

A pursuer drone chases an evader drone at low altitude over a complex 3D terrain that includes Gaussian hills, ridge lines, and narrow canyon corridors. Both agents must avoid the terrain surface at all times.

**Arena**: $[-10, 10] \times [-10, 10] \times [0, 8]$ m

**Pursuer**:
- Runs Proportional Navigation Guidance (PNG) projected onto the terrain-safe 3D velocity envelope
- Applies lateral repulsion from terrain surface normals to avoid canyon walls
- Executes coordinated roll turns when changing heading in a banked maneuver

**Evader**:
- Exploits terrain masking: flies into narrow canyons and behind ridge lines to break pursuer line-of-sight
- Uses altitude hopping to force the pursuer over ridge crests, gaining separation time
- Applies a dive-to-floor tactic inside canyons where the pursuer's vertical tracking lag is maximal

**Objective**: Quantify how terrain complexity (canyon width, hill density) extends evader survival time; measure the pursuer's terrain-avoidance cost (path length overhead, energy).

---

## Mathematical Model

### Terrain DEM

The terrain height field is a superposition of Gaussian hills and ridge functions:

$$h(x, y) = \sum_{i=1}^{N_h} A_i \exp\!\left(-\frac{(x - x_i)^2}{2\sigma_{x,i}^2} - \frac{(y - y_i)^2}{2\sigma_{y,i}^2}\right)$$

The terrain surface normal at $(x, y)$ is:

$$\hat{\mathbf{n}}(x,y) = \frac{(-\partial h/\partial x,\; -\partial h/\partial y,\; 1)^\top}{\|(-\partial h/\partial x,\; -\partial h/\partial y,\; 1)\|}$$

### Altitude Safety Constraint (Extended)

The vertical clearance constraint from S003 is retained:

$$z_{cmd} \geq h(x, y) + h_{safe}, \quad h_{safe} = 0.3 \text{ m}$$

### Lateral Terrain Repulsion

Define the signed horizontal distance from the nearest terrain wall as $d_\perp$ (negative means inside the wall). The lateral repulsion acceleration added to the guidance command is:

$$\mathbf{a}_{rep} = \begin{cases} k_{rep}\left(\dfrac{1}{d_\perp} - \dfrac{1}{d_0}\right)\dfrac{1}{d_\perp^2}\,\hat{\mathbf{n}}_{xy} & d_\perp < d_0 \\ \mathbf{0} & d_\perp \geq d_0 \end{cases}$$

where $d_0 = 1.0$ m is the activation distance, $k_{rep} = 2.0$ m$^3$/s$^2$, and $\hat{\mathbf{n}}_{xy}$ is the horizontal component of the terrain surface normal, renormalised.

### 3D Proportional Navigation Guidance (PNG)

Line-of-sight (LOS) vector from pursuer $P$ to evader $E$:

$$\hat{\mathbf{r}} = \frac{\mathbf{p}_E - \mathbf{p}_P}{\|\mathbf{p}_E - \mathbf{p}_P\|}$$

LOS rate vector:

$$\dot{\boldsymbol{\lambda}} = \frac{\mathbf{v}_E - \mathbf{v}_P - [(\mathbf{v}_E - \mathbf{v}_P)\cdot\hat{\mathbf{r}}]\hat{\mathbf{r}}}{\|\mathbf{p}_E - \mathbf{p}_P\|}$$

PNG acceleration command (full 3D):

$$\mathbf{a}_{PNG} = N \cdot V_c \cdot \dot{\boldsymbol{\lambda}}$$

where $N = 3$ is the navigation constant and $V_c = -(\mathbf{v}_E - \mathbf{v}_P)\cdot\hat{\mathbf{r}}$ is the closing speed.

The total pursuer acceleration command is:

$$\mathbf{a}_{cmd} = \mathbf{a}_{PNG} + \mathbf{a}_{rep} + \mathbf{a}_{GE}$$

where $\mathbf{a}_{GE}$ is the ground-effect altitude correction (see below).

### Ground Effect Altitude Correction

The pursuer's z-acceleration is augmented to compensate for ground-effect thrust variation:

$$a_{z,GE} = -\frac{g}{k_{GE}(h)} + g, \quad k_{GE}(h) = 1 + \frac{0.16\,(R/h)^2}{1 + (R/h)^2}$$

where $R = 0.05$ m is the propeller radius and $h = z - h_{terrain}(x,y)$ is the AGL height.

### Coordinated Roll (Bank Turn)

For a drone executing a horizontal turn with lateral acceleration $a_\perp$ at speed $v$, the required bank angle to remain coordinated (no sideslip) is:

$$\phi = \arctan\!\left(\frac{a_\perp}{g}\right)$$

The body-frame thrust vector is then:

$$\mathbf{T} = m\,(a_{cmd,z} + g)\,(\sin\phi\,\hat{\mathbf{y}}_b + \cos\phi\,\hat{\mathbf{z}}_b)$$

where $\hat{\mathbf{y}}_b$ and $\hat{\mathbf{z}}_b$ are the body-frame lateral and vertical axes derived from the current heading. This roll coupling is absent in the original 2D-in-x-y model.

### Line-of-Sight Masking by Terrain

The pursuer's PNG law is only effective when the LOS is unobstructed. The LOS is blocked if any point along the segment $\mathbf{p}_P + t(\mathbf{p}_E - \mathbf{p}_P)$, $t \in [0,1]$, satisfies:

$$z(t) < h\!\left(x(t), y(t)\right)$$

When blocked, the pursuer switches to a **terrain-following dead-reckoning** mode: maintain last known evader heading at the current safe altitude until LOS is restored.

### 3D Capture Condition

$$\|\mathbf{p}_P - \mathbf{p}_E\| < r_{capture} = 0.20 \text{ m}$$

---

## Key 3D Additions

- **Full DEM mesh**: terrain is a $N \times N$ grid sampled at 0.2 m resolution; surface normals computed via finite differences
- **Lateral repulsion field**: inverse-distance repulsion from canyon walls and ridge flanks; prevents wall collisions ignored by the original z-only constraint
- **Side-canyon navigation**: evader path planner routes through corridors narrower than 2 m; pursuer must follow with lateral obstacle avoidance active
- **Roll-coordinated bank turns**: 6-DOF state includes roll angle $\phi$; bank turn equations couple lateral acceleration to roll for physically consistent cornering
- **LOS masking detection**: ray-terrain intersection test at each timestep; pursuer switches guidance mode when ridge blocks LOS
- **Altitude hop strategy (evader)**: evader command climbs 1.5 m above ridge crest when a blocking ridge is detected ahead, then descends behind it to regain cover

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Arena | $[-10,10]^2 \times [0,8]$ m |
| DEM resolution | 0.2 m |
| Number of Gaussian hills $N_h$ | 6 |
| Hill amplitude $A_i$ | 1.0 – 2.5 m |
| Hill width $\sigma$ | 0.8 – 2.0 m |
| Safety clearance $h_{safe}$ | 0.3 m |
| Repulsion activation distance $d_0$ | 1.0 m |
| Repulsion gain $k_{rep}$ | 2.0 m$^3$/s$^2$ |
| PNG navigation constant $N$ | 3 |
| Pursuer speed | 4.5 m/s |
| Evader speed | 3.0 m/s |
| Propeller radius $R$ | 0.05 m |
| Capture radius $r_{capture}$ | 0.20 m |
| Altitude range AGL | 0.3 – 6.0 m |
| Control frequency | 50 Hz |
| Simulation duration | 30 s |

---

## Expected Output

- **3D trajectory plot**: pursuer (red) and evader (blue) trajectories rendered above the shaded DEM surface mesh
- **Altitude vs time**: AGL height for both agents; shows terrain-following bumps and altitude hop events
- **LOS blockage timeline**: binary mask indicating when terrain occludes pursuer-to-evader LOS
- **Repulsion force magnitude vs time**: lateral $|\mathbf{a}_{rep}|$ showing canyon wall proximity events
- **Roll angle vs time**: $\phi(t)$ for the pursuer during coordinated bank turns in canyon sections
- **Capture time vs terrain complexity**: bar chart sweeping hill density $N_h \in \{2, 4, 6, 8\}$

---

## Extensions

1. Replace synthetic DEM with a real SRTM tile (e.g., mountain valley at 30 m resolution, downsampled to simulation scale) to test on geographically realistic terrain
2. Add an ultrasonic altimeter noise model ($\sigma = 0.05$ m) to the pursuer's terrain height estimate and analyse how sensor noise degrades the repulsion field
3. Multi-pursuer encirclement over canyon terrain: two pursuers coordinate to drive the evader out of a canyon into open airspace where the third pursuer intercepts
4. Energy-optimal canyon transit: solve the pursuer's path as a constrained optimisation minimising $\int \|\mathbf{a}\|^2\,dt$ subject to terrain clearance and LOS recovery time

---

## Related Scenarios

- Original 2D version: [S003](../S003_low_altitude_tracking.md)
- 3D evasion reference: [S002 3D](S002_3d_evasive_maneuver.md)
- Obstacle chase extension: [S004](../S004_obstacle_avoidance_pursuit.md)
