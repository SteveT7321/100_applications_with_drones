# S042 3D Upgrade — Missing Person Search

**Domain**: Environmental & SAR | **Difficulty**: ⭐⭐⭐⭐ | **Status**: `[ ]` Not started
**Based on**: [S042 original](../S042_missing_person.md)

---

## What Changes in 3D

The original S042 is geometrically flat: all drones cruise at a fixed altitude `h = 10.0 m`
and the sensor footprint radius is constant at `r_s = 2.0 m` regardless of any terrain below.
The belief map is a 2D grid over a flat 200 × 200 m plane, and drone kinematics carry no
z-component beyond the hardcoded scan height.

This 3D upgrade replaces the flat arena with a mountainous terrain generated from a synthetic
Digital Elevation Model (DEM). Three consequences follow:

1. **Altitude-varying sensor footprint**: a drone maintaining constant Above-Ground-Level (AGL)
   altitude $h_{AGL}$ over undulating terrain must vary its absolute z-coordinate; the footprint
   radius on the ground scales with the slant-range geometry.
2. **Terrain-following flight**: drones must clear a minimum safety margin above the DEM surface
   at every waypoint; straight-line transit between waypoints requires intermediate altitude
   checks to avoid collision with ridgelines.
3. **Probability map on a 3D surface**: the belief map is still defined per ground-cell $(x, y)$,
   but the terrain height $z_{dem}(x,y)$ modulates cell accessibility and the sensor cone
   projection — cells hidden behind ridges from the drone's viewing angle receive no observation
   update (terrain occlusion).

---

## Problem Definition

**Setup**: A hiker has gone missing in a 200 × 200 m mountainous wilderness area. The terrain
is described by a DEM $z_{dem}(x,y)$ synthesised with Perlin-noise superimposed on a smooth
base (elevation range 0–40 m, typical ridge spacing 40–60 m). The last known position (LKP)
is on a valley floor. Elapsed time since LKP is $\Delta t = 7200$ s; the prior probability
distribution is a Gaussian diffused over 2D horizontal coordinates (the person moves along the
terrain surface, not in free air). A fleet of $N = 3$ drones is deployed. Each drone maintains
a constant AGL altitude $h_{AGL} = 10.0$ m by commanding its absolute altitude to
$z_{cmd}(x,y) = z_{dem}(x,y) + h_{AGL}$, interpolated along its transit path. The sensor is a
downward-looking thermal imager with a half-angle $\phi_s = 11.3°$ that projects a circular
footprint of radius $r_s(h_{AGL}) = h_{AGL} \tan\phi_s \approx 2.0$ m on level ground; on
sloped terrain the elliptical projection is approximated by the effective radius formula below.

**Roles**:
- **Missing person**: stationary on the terrain surface; true position $(x^*, y^*)$ drawn from
  the prior at mission start; the person's ground-truth cell may be on a slope or in a valley.
- **Drones** ($N = 3$): terrain-following searchers sharing one 2D belief map; each observes a
  terrain-projected footprint and uploads binary hit/miss observations to the shared state.

**Objective**: Minimise expected time to first confirmed detection ($P_D \geq 0.95$) while
accounting for terrain occlusion — cells blocked from view by intervening ridges accumulate no
information even when a drone is nominally nearby.

**Comparison strategies**:
1. **Terrain-aware greedy frontier** — selects the highest-belief unoccluded cell, commanding
   the terrain-following altitude profile en route.
2. **Flat-altitude lawnmower** — systematic boustrophedon at fixed MSL altitude of 20 m
   (ignores DEM; occlusion model still applied); serves as a baseline showing the cost of
   ignoring terrain.
3. **Valley-biased prior + greedy** — reweights the prior by a valley-affinity factor
   $w_{valley}(x,y)$ before search; tests whether topographic priors accelerate detection when
   lost hikers tend to shelter in valleys.

---

## Mathematical Model

### Terrain Model (Synthetic DEM)

The DEM is generated as a sum of Perlin-noise octaves:

$$z_{dem}(x, y) = A_{base} \sum_{k=1}^{K} \frac{1}{2^{k-1}}
  \sin\!\left(\frac{2\pi x}{L_k} + \phi_{x,k}\right)
  \cos\!\left(\frac{2\pi y}{L_k} + \phi_{y,k}\right)$$

with $K = 4$ octaves, base wavelength $L_1 = 80$ m, amplitude $A_{base} = 10$ m, and random
phase offsets $\phi_{x,k}, \phi_{y,k}$. The result is clipped to $[0, 40]$ m. The DEM is
sampled on the same $400 \times 400$ grid as the belief map (resolution 0.5 m/cell).

### Drone Altitude Command

At horizontal position $(x, y)$, the commanded absolute altitude is:

$$z_{cmd}(x, y) = z_{dem}(x, y) + h_{AGL}$$

During transit from waypoint $\mathbf{w}_i$ to $\mathbf{w}_{i+1}$, the drone follows a
piecewise-linear altitude profile sampled at $\Delta s = 1.0$ m intervals to ensure clearance
above the terrain ridge between the two waypoints:

$$z_{transit}(s) = \max\!\left(z_{cmd}(\mathbf{p}(s)),\;
  \max_{s' \in [0, s]} z_{dem}(\mathbf{p}(s')) + h_{AGL}\right)$$

where $\mathbf{p}(s)$ is the horizontal position linearly interpolated along the transit arc
parameterised by arc length $s$.

### Altitude-Dependent Effective Footprint Radius

For a drone at 3D position $\mathbf{p}_d = (x_d, y_d, z_d)$ observing ground cell
$(x_c, y_c, z_{dem}(x_c,y_c))$, the slant range is:

$$\rho = \sqrt{(x_d - x_c)^2 + (y_d - y_c)^2 + (z_d - z_{dem}(x_c,y_c))^2}$$

The sensor half-angle $\phi_s$ subtends footprint radius:

$$r_s^{eff} = \rho \tan\phi_s$$

A ground cell is inside the sensor cone if the nadir angle $\psi$ satisfies:

$$\psi = \arctan\!\frac{\sqrt{(x_d-x_c)^2+(y_d-y_c)^2}}{z_d - z_{dem}(x_c,y_c)} \leq \phi_s$$

This reduces to the flat-ground formula $r_s = h_{AGL} \tan\phi_s$ when terrain is level.

### Terrain Occlusion Model

Cell $(x_c, y_c)$ is visible from drone position $\mathbf{p}_d$ if no intervening DEM voxel
blocks the line of sight. A cell is declared occluded if:

$$\exists\, t \in (0, 1):\quad z_{dem}(\mathbf{p}_d + t(\mathbf{x}_c - \mathbf{p}_d)) >
  z_d + t\,(z_{dem}(x_c,y_c) - z_d)$$

where the DEM value along the LOS ray is linearly interpolated. In practice, the LOS is
sampled at $N_{ray} = 20$ equally spaced points; if any sample exceeds the interpolated
drone-to-cell altitude by more than 0.1 m, the cell is flagged as occluded for that timestep.

### Prior Distribution

The prior follows the same diffused Gaussian as S042, defined over 2D horizontal coordinates:

$$P_0(x, y) = \frac{1}{Z} \exp\!\left(-\frac{(x - x_{LKP})^2 + (y - y_{LKP})^2}{2\,\sigma_0^2}\right)$$

$$\sigma_0^2 = \sigma_{LKP}^2 + 2D\Delta t, \qquad \sigma_0 \approx 17.2 \text{ m}$$

**Valley-biased variant**: multiply by a terrain affinity weight before normalization:

$$P_0^{valley}(x,y) \propto P_0(x,y) \cdot \exp\!\left(-\frac{z_{dem}(x,y)}{\tau_{valley}}\right)$$

where $\tau_{valley} = 15$ m. This exponentially down-weights high-elevation cells, encoding
the prior belief that a lost hiker preferentially shelters in valleys.

### Bayesian Belief Update with Occlusion

At each timestep, visible cells (not occluded) receive the standard Bayes update; occluded
cells are left unchanged. Let $\mathcal{V}_i \subseteq \mathcal{G}$ be the set of
non-occluded cells visible to drone $i$, and $\mathcal{F}_i \subseteq \mathcal{V}_i$ the
subset within the sensor cone. The likelihood of observation $z_i \in \{0, 1\}$ for cell
$(x,y)$ is:

$$P(z_i \mid H_{(x,y)}) =
\begin{cases}
P_d      & z_i = 1,\; (x,y) \in \mathcal{F}_i \\
P_{fa}   & z_i = 1,\; (x,y) \in \mathcal{V}_i \setminus \mathcal{F}_i \\
1        & z_i = 0,\; (x,y) \notin \mathcal{V}_i \quad \text{(occluded, no info)} \\
1 - P_d  & z_i = 0,\; (x,y) \in \mathcal{F}_i \\
1-P_{fa} & z_i = 0,\; (x,y) \in \mathcal{V}_i \setminus \mathcal{F}_i
\end{cases}$$

The posterior is then normalised over all grid cells as in S042.

### 3D Drone Kinematics

The drone's 3D state vector is $\mathbf{q}_i = (x_i, y_i, z_i)$. Transit between consecutive
waypoints $\mathbf{w}_i^{(k)}$ and $\mathbf{w}_i^{(k+1)}$ is governed by:

$$\dot{\mathbf{q}}_i = v_d \cdot \hat{\mathbf{u}}_i, \qquad
  \hat{\mathbf{u}}_i = \frac{\mathbf{w}_i^{(k+1)} - \mathbf{q}_i}{\|\mathbf{w}_i^{(k+1)} - \mathbf{q}_i\|}$$

where the waypoint z-coordinate is $w_{z} = z_{cmd}(w_x, w_y)$. The transit time accounts for
the full 3D distance including altitude change:

$$\Delta t_{transit} = \frac{\|\mathbf{w}_i^{(k+1)} - \mathbf{w}_i^{(k)}\|_3}{v_d}$$

### Belief Entropy and Waypoint Selection

Shannon entropy, identical to S042:

$$H(B_t) = -\sum_{(x,y) \in \mathcal{G}} B_t(x,y) \log B_t(x,y)$$

Terrain-aware greedy waypoint selection excludes occluded cells and cells on steep slopes
(gradient $\|\nabla z_{dem}\| > \tan 45°$, implying a person is unlikely to be there):

$$\mathbf{w}_i^* = \arg\max_{(x,y) \in \mathcal{G}_{safe}} B_t(x,y)
  \quad \text{s.t.} \quad \|(x,y) - w_j^*\|_2 > 2\,r_s \;\forall j \neq i$$

where $\mathcal{G}_{safe} = \{(x,y) : \|\nabla z_{dem}(x,y)\| \leq 1.0,\; (x,y) \in \mathcal{V}_i\}$.

---

## Key 3D Additions

- **DEM generation**: Perlin-noise synthetic terrain (elevation 0–40 m, 400 × 400 grid at 0.5 m/cell)
- **Terrain-following altitude**: drone z-command tracks $z_{dem}(x,y) + h_{AGL}$ along each transit segment
- **Altitude-dependent footprint**: sensor cone half-angle $\phi_s = 11.3°$; footprint radius $r_s^{eff}$ depends on slant range
- **Occlusion masking**: LOS ray sampling blocks Bayes updates for cells hidden behind ridges
- **Valley-biased prior**: exponential terrain-affinity weighting $\exp(-z_{dem}/\tau_{valley})$
- **3D transit distance**: waypoint spacing computed in full $(x,y,z)$ space

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Search area | 200 × 200 m |
| Grid resolution | 0.5 m/cell (400 × 400 = 160 000 cells) |
| Number of drones $N$ | 3 |
| AGL scan altitude $h_{AGL}$ | 10.0 m |
| Sensor half-angle $\phi_s$ | 11.3° |
| Nominal footprint radius $r_s$ | 2.0 m (flat ground) |
| Drone cruise speed $v_d$ | 5.0 m/s |
| Sensor dwell per waypoint $\tau_{dwell}$ | 1.0 s |
| Probability of detection $P_d$ | 0.90 |
| False alarm rate $P_{fa}$ | 0.02 |
| LKP positional uncertainty $\sigma_{LKP}$ | 10.0 m |
| Brownian diffusion coefficient $D$ | 0.02 m²/s |
| Time since LKP $\Delta t$ | 7200 s (2 hours) |
| Prior spread $\sigma_0$ | $\approx$ 17.2 m |
| DEM elevation range | 0 – 40 m |
| DEM Perlin octaves $K$ | 4 |
| DEM base wavelength $L_1$ | 80 m |
| Valley affinity temperature $\tau_{valley}$ | 15 m |
| Occlusion ray samples $N_{ray}$ | 20 |
| Transit altitude sample interval $\Delta s$ | 1.0 m |
| Safe slope threshold $\|\nabla z_{dem}\|$ | $\leq 1.0$ (45°) |
| Hard mission cutoff | 1800 s (30 minutes) |

---

## Expected Output

- **DEM surface plot**: 3D Matplotlib surface of $z_{dem}(x,y)$ with the LKP marked; colour
  by elevation; overlaid with the 2D prior $P_0(x,y)$ as a translucent heat layer.
- **3D trajectory plots**: drone flight paths in $(x,y,z)$ for each strategy; terrain surface
  shown as a semi-transparent mesh; terrain-following altitude variation visible on z-axis.
- **Altitude time series**: $z_i(t)$ for each drone over the mission duration; shows that
  terrain-following drones vary altitude by up to 40 m while flat-altitude baseline stays fixed.
- **Belief map snapshots (top-down)**: four-panel evolution of $B_t(x,y)$ at $t = 0, 60, 120,
  t_{det}$ s; occluded cells at each snapshot highlighted in a separate colour channel.
- **Occlusion coverage map**: cumulative fraction of mission time each cell was occluded;
  reveals ridge-shadow dead zones never observed by terrain-following vs flat-altitude strategies.
- **Entropy reduction curves**: $H(B_t)$ vs $t$ for all three strategies on the same axes;
  flat-altitude lawnmower shows slower entropy reduction due to occlusion dead zones.
- **Detection time box-plot**: 50 Monte Carlo trials (random true person locations drawn from
  prior) per strategy; compares terrain-aware greedy, flat lawnmower, and valley-biased greedy.
- **Animation (GIF)**: rotating 3D view of drones following terrain contours while belief map
  updates frame by frame; terrain mesh fixed; drone markers update in 3D; entropy displayed in title.

---

## Extensions

1. **Particle filter on 3D terrain**: replace the 400 × 400 grid belief map with
   $N_p = 5000$ surface-constrained particles; each particle moves along the terrain surface
   using a projected random walk; apply systematic resampling when effective count drops below
   $N_p / 2$.
2. **Ridgeline avoidance path planning**: replace straight-line transit with an A* planner on
   a 3D cost grid where altitude change and proximity to ridges add traversal cost; compare
   mission time vs flat-route baseline.
3. **Wind-drift correction**: model a person's passive movement downhill along gradient
   $\dot{\mathbf{x}}^* = -\alpha \nabla z_{dem}(\mathbf{x}^*)$ (gravity drift) and apply a
   terrain-aware prediction step to the belief map at each timestep before Bayes update.
4. **Multi-resolution DEM search**: use a coarse 2 m/cell DEM for initial rapid coverage to
   narrow the search region to a 50 × 50 m zone, then switch to the fine 0.5 m/cell grid for
   precise localisation; measure the time saved vs single-resolution search.
5. **Thermal updraft exploitation**: in mountainous terrain, drones can exploit terrain-induced
   thermals to reduce power consumption; model a simplified updraft map and allow drones to
   loiter in updrafts while waiting for their assigned sector to gain priority.

---

## Related Scenarios

- Original 2D version: [S042](../S042_missing_person.md)
- Terrain-aware SAR reference: [S041 Area Coverage Sweep](../S041_wildfire_boundary.md)
- Occlusion and 3D sensing: [S003 Low-Altitude Tracking](../../01_pursuit_evasion/S003_low_altitude_tracking.md)
- Particle filter estimation: [S013 Particle Filter Intercept](../../01_pursuit_evasion/S013_particle_filter_intercept.md)
- Truly 3D pursuit reference: [S001](../../01_pursuit_evasion/S001_basic_intercept.md), [S002 3D](../../01_pursuit_evasion/3d/S002_3d_evasive_maneuver.md)
