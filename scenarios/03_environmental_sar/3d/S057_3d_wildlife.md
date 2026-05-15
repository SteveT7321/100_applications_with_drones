# S057 3D Upgrade — Wildlife Census

**Domain**: Environmental & SAR | **Difficulty**: ⭐⭐⭐ | **Status**: `[ ]` Not Started
**Based on**: [S057 original](../S057_wildlife.md)

---

## What Changes in 3D

The original S057 fixes the survey drone at a single constant altitude $h$ for the entire
lawnmower mission; the terrain is implicitly assumed flat, and the camera footprint formula
$w(h) = 2h\tan(\theta_{FOV})$ uses a scalar $h$ measured from the launch pad datum.  In
real wildlife reserves the ground surface rises and falls by tens of metres, so a
constant-barometric-altitude drone is actually flying at varying heights above ground level
(AGL), producing an inconsistent and unpredictable camera footprint that violates the
strip-transect census assumptions.

This 3D upgrade introduces:

1. **Terrain-following flight** — the drone adjusts its barometric altitude at each
   waypoint so that its AGL height stays at the commanded survey altitude $h_{AGL}$,
   reading ground elevation from a Digital Elevation Model (DEM) grid.
2. **Altitude-dependent camera footprint** — because AGL height is now truly constant,
   the strip width $w = 2 h_{AGL}\tan(\theta_{FOV})$ is uniform per pass and the
   strip-transect correction remains valid.
3. **Multi-altitude strip overlap** — three altitude tiers ($h_{AGL} \in \{8, 20, 40\}$ m)
   are compared; the overlap fraction between adjacent passes is computed from the actual
   footprint width, and double-count probability is derived exactly.
4. **3D trajectory visualisation** — barometric altitude profile shows the drone
   climbing and descending over hills while maintaining constant AGL, making the terrain
   coupling explicit.

---

## Problem Definition

**Setup**: A wildlife reserve occupies a $500 \times 500$ m area with a procedurally
generated terrain DEM $z_{terrain}(x, y)$ (Perlin-noise hill field, peak-to-valley
relief $\Delta z_{max} = 30$ m).  $G = 20$ animal groups are placed on the terrain
surface (their z-coordinate equals the DEM elevation at their centroid).  The survey
drone executes a boustrophedon lawnmower pattern in 3D: at each strip waypoint the
commanded barometric altitude is:

$$z_{cmd}(x, y) = z_{terrain}(x, y) + h_{AGL}$$

The drone follows these waypoints with a simple altitude controller at each strip
segment before advancing to the next column.  Detection geometry is evaluated in 3D:
an animal is within the camera footprint if its horizontal ground-plane distance from
the nadir point falls within the strip half-width $w/2$, irrespective of the
individual's ground elevation relative to the drone.

**Roles**:
- **Survey drone**: single agent flying a 3D terrain-following lawnmower at commanded
  AGL altitude $h_{AGL}$.
- **Animal groups** ($G = 20$): static clusters whose 3D positions
  $\mathbf{x}_{g,i} = (x_{g,i},\; y_{g,i},\; z_{terrain}(x_{g,i}, y_{g,i}))^T$
  lie on the terrain surface.

**Objective**: Evaluate three terrain-following strategies:

1. **High-AGL fast** — $h_{AGL} = 40$ m, $v = 12$ m/s.
2. **Mid-AGL medium** — $h_{AGL} = 20$ m, $v = 6$ m/s.
3. **Low-AGL slow** — $h_{AGL} = 8$ m, $v = 2$ m/s.

For each strategy report: corrected population estimate $\hat{N}_{corrected}$, relative
error $\epsilon$, total 3D path length $L_{3D}$, mission duration, and RMS AGL deviation
caused by terrain response lag.

---

## Mathematical Model

### Terrain Model (Synthetic DEM)

Terrain elevation is generated as a sum of $K$ sinusoidal hills:

$$z_{terrain}(x, y) = \frac{\Delta z_{max}}{K} \sum_{k=1}^{K}
  \sin\!\left(\frac{2\pi x}{\lambda_{x,k}} + \phi_{x,k}\right)
  \cos\!\left(\frac{2\pi y}{\lambda_{y,k}} + \phi_{y,k}\right)$$

with $K = 5$ harmonics, wavelengths $\lambda \in [80, 300]$ m, and random phase offsets
$\phi \in [0, 2\pi)$.  The DEM is evaluated on a $100 \times 100$ grid and bilinearly
interpolated at arbitrary $(x, y)$.

### Terrain-Following Altitude Command

At every lawnmower waypoint the commanded barometric altitude is:

$$z_{cmd}(x, y) = z_{terrain}(x, y) + h_{AGL}$$

The drone's actual barometric altitude follows a first-order lag:

$$\dot{z}_{drone}(t) = \frac{z_{cmd}(x(t), y(t)) - z_{drone}(t)}{\tau_z}$$

where $\tau_z = 1.0$ s is the altitude response time constant.  The instantaneous AGL
height is:

$$h(t) = z_{drone}(t) - z_{terrain}(x(t), y(t))$$

RMS AGL deviation over the mission:

$$\sigma_{AGL} = \sqrt{\frac{1}{T_{mission}} \int_0^{T_{mission}}
  \bigl(h(t) - h_{AGL}\bigr)^2\, dt}$$

### 3D Camera Footprint and Strip Width

The camera footprint half-width at the commanded AGL altitude is:

$$w(h_{AGL}) = h_{AGL}\,\tan(\theta_{FOV})$$

Full strip width:

$$W_{strip}(h_{AGL}) = 2\,h_{AGL}\,\tan(\theta_{FOV})$$

Because $h(t)$ deviates from $h_{AGL}$ due to terrain lag, the effective instantaneous
footprint half-width at time $t$ is:

$$w_{eff}(t) = h(t)\,\tan(\theta_{FOV})$$

The actual detection footprint for each pass is thus slightly variable; the simulation
evaluates each individual animal position against $w_{eff}$ at the moment of overflight.

### Number of Passes and Total 3D Path Length

The number of parallel lawnmower passes (columns) to cover the reserve width $W = 500$ m:

$$n_{passes} = \left\lceil \frac{W}{W_{strip}(h_{AGL})} \right\rceil$$

Each pass has horizontal length $W = 500$ m.  The total 3D path length accounts for
vertical excursions:

$$L_{3D} = \sum_{k=0}^{N_{wp}-1}
  \left\|\mathbf{p}_{k+1} - \mathbf{p}_k\right\|_2$$

where $\mathbf{p}_k = (x_k,\; y_k,\; z_{drone,k})^T$ are the drone's 3D positions at
each simulation timestep.  $L_{3D} \geq L_{2D}$ because climbing over hills adds path
length.

### Overlap Fraction and Double-Count Probability

Adjacent passes are separated by $W_{strip}$ m (no nominal overlap intended).  Due to
terrain-induced footprint variation, if the effective width on pass $j$ is
$W_{eff,j}$ then the overlap between passes $j$ and $j+1$ is:

$$\delta_j = \max\!\left(0,\;\frac{W_{eff,j} + W_{eff,j+1}}{2} - W_{strip}\right)
  / W_{strip}$$

The probability of double-counting an animal in the overlap zone of pass $j$:

$$P_{double,j}(h_{AGL}, v, c_g) =
  P_d(h_{AGL}, v, c_g)^2 \cdot \delta_j$$

### Detection Probability (inherited from S057)

$$P_d(h_{AGL}, v, c_g) =
  P_{d0}\,\exp(-\alpha\,h_{AGL})\,\exp(-\beta\,v)\,(1 - c_g)$$

Parameters: $P_{d0} = 0.95$, $\alpha = 0.04$ m$^{-1}$, $\beta = 0.08$ s/m.

### Population Estimate and Relative Error

Raw count accumulated across all passes and Bernoulli trials:

$$\hat{N}_{raw} = \sum_{g=1}^{G}\sum_{j=1}^{n_{passes}}\hat{n}_{g,j}$$

Strip-transect correction using mean detection probability:

$$\hat{N}_{corrected} = \frac{\hat{N}_{raw}}{\overline{P}_d(h_{AGL}, v)}$$

Relative counting error:

$$\epsilon = \frac{\left|\hat{N}_{corrected} - N^*\right|}{N^*}$$

### Mission Duration

$$T_{mission}(h_{AGL}, v) = \frac{L_{3D}}{v}$$

Note that $L_{3D} > L_{2D}$ on hilly terrain, so $T_{mission}$ is slightly longer than
the flat-terrain estimate from S057.

---

## Key 3D Additions

- **Terrain DEM**: procedural Perlin-style hill field with $\Delta z_{max} = 30$ m relief
  evaluated on a $100 \times 100$ grid; bilinearly interpolated at all waypoints.
- **Altitude command law**: $z_{cmd}(x,y) = z_{terrain}(x,y) + h_{AGL}$, followed by
  first-order lag $\tau_z = 1.0$ s.
- **Effective footprint variation**: $w_{eff}(t) = h(t)\tan(\theta_{FOV})$ — footprint
  widens when terrain lag causes the drone to fly too high and narrows on upslope.
- **3D path length accounting**: $L_{3D}$ sums Euclidean 3D waypoint-to-waypoint
  distances, reflecting true energy expenditure on hilly terrain.
- **Overlap from footprint drift**: overlap fraction $\delta_j$ derived per-pass pair
  from effective widths, fed into $P_{double,j}$.
- **RMS AGL deviation metric** $\sigma_{AGL}$: quantifies how well the drone tracks the
  commanded terrain-following altitude; larger $\tau_z$ or faster horizontal speed
  degrades tracking.
- **3D visualisation**: 3D surface mesh of DEM with colour-coded animal clusters; drone
  trajectory drawn as a 3D ribbon showing altitude oscillations.

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Reserve area | 500 × 500 m |
| DEM relief $\Delta z_{max}$ | 30 m |
| DEM harmonics $K$ | 5 |
| DEM grid resolution | 100 × 100 (5 m/cell) |
| Animal groups $G$ | 20 |
| Animals per group $n_g$ | 1 – 20 |
| Intra-group spread $\sigma_g$ | 8 m |
| Camera half-angle $\theta_{FOV}$ | 25° |
| Baseline detection $P_{d0}$ | 0.95 |
| Altitude decay $\alpha$ | 0.04 m$^{-1}$ |
| Speed decay $\beta$ | 0.08 s/m |
| Camouflage factor $c_g$ | 0.0 – 0.6 |
| Altitude response lag $\tau_z$ | 1.0 s |
| High-AGL fast $(h_{AGL}, v)$ | 40 m, 12 m/s |
| Mid-AGL medium $(h_{AGL}, v)$ | 20 m, 6 m/s |
| Low-AGL slow $(h_{AGL}, v)$ | 8 m, 2 m/s |
| Strip width at 40 / 20 / 8 m AGL | ~37 m / ~19 m / ~7 m |
| AGL bounds | 5 – 50 m |
| Monte Carlo trials per strategy | 50 |

---

## Expected Output

- **3D terrain + trajectory plot**: surface mesh of the DEM coloured by elevation with all
  20 animal group centroids plotted on the surface; the drone's 3D lawnmower path shown as
  a red ribbon for the mid-AGL strategy, clearly tracking terrain contours.
- **Altitude time series**: barometric altitude $z_{drone}(t)$ vs. commanded altitude
  $z_{cmd}(t)$ and terrain elevation $z_{terrain}(x(t), y(t))$ for each strategy on a
  shared time axis; AGL height $h(t)$ plotted in a lower subplot with $h_{AGL}$ as a
  dashed reference and $\pm\sigma_{AGL}$ shading.
- **Effective footprint width vs. time**: $w_{eff}(t) = h(t)\tan(\theta_{FOV})$ for each
  strategy; highlights where terrain lag causes footprint deviation from nominal.
- **Count distribution box plots**: corrected count $\hat{N}_{corrected}$ across 50 Monte
  Carlo trials per strategy; true count $N^*$ as a red dashed line; compared side-by-side
  with the original flat-terrain S057 results.
- **Terrain penalty analysis**: bar chart of $L_{3D} - L_{2D}$ (extra path length due to
  hills) and fractional mission-time increase $\Delta T_{mission}/T_{mission,flat}$ for
  each strategy.
- **Strategy comparison table**: printed columns — AGL altitude, speed, nominal strip
  width, $n_{passes}$, $L_{3D}$, $T_{mission}$, $\sigma_{AGL}$, mean $P_d$, corrected
  count mean, relative error $\epsilon$, mean $\delta$ overlap fraction.
- **Animation (GIF)**: oblique 3D view of the drone completing one lawnmower pass over
  the hilly reserve, animal groups revealed as their 3D positions enter the instantaneous
  footprint ellipse; colour of each group transitions from hollow (undetected) to filled
  (detected).

---

## Extensions

1. **Adaptive AGL altitude per strip**: use a prior terrain-roughness map to choose a
   higher $h_{AGL}$ over steep ridgelines (to avoid collision) and lower $h_{AGL}$ in
   flat valleys (to boost $P_d$); measure improvement in $\epsilon$ vs. uniform AGL.
2. **Fast terrain-lag compensation**: use a predictive feedforward command
   $z_{cmd,ff}(t) = z_{terrain}(x(t + \tau_z), y(t + \tau_z)) + h_{AGL}$ to cancel the
   first-order lag; quantify reduction in $\sigma_{AGL}$.
3. **Multi-drone interleaved 3D sweeps**: assign $N = 3$ drones to interleaved lawnmower
   lanes with independent terrain-following; study how inter-drone altitude differences
   over shared terrain affect strip-overlap and double-count rate.
4. **Moving animals on sloped terrain**: add a gradient-biased random walk
   $\dot{\mathbf{x}} \sim \mathcal{N}(-\nabla z_{terrain}, \sigma_v^2 \mathbf{I})$
   (animals drift downhill); measure how downslope clustering changes census accuracy.
5. **Obstacle-aware path replanning**: add a no-fly zone over a forested ridge
   (terrain + tree height $> z_{threshold}$); use RRT* to plan a 3D lawnmower detour and
   assess coverage gaps introduced by the avoidance manoeuvre.

---

## Related Scenarios

- Original 2D version: [S057](../S057_wildlife.md)
- Terrain-following reference: [S043 Confined Space](../S043_confined_space.md) (altitude control near surfaces)
- Coverage geometry reference: [S048 Lawnmower](../S048_lawnmower.md) (strip-pattern coverage)
- Sensor footprint reference: [S044 Wall Crack Inspection](../S044_wall_crack.md) (altitude-dependent footprint)
- Pursuit-evasion 3D reference: [S001](../../01_pursuit_evasion/S001_basic_intercept.md), [S002 3D](../../01_pursuit_evasion/3d/S002_3d_evasive_maneuver.md)
