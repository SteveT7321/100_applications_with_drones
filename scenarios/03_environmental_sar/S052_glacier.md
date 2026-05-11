# S052 Glacier Melt Area Measurement

**Domain**: Environmental Monitoring & SAR | **Difficulty**: ⭐⭐ | **Status**: `[ ]` Not Started
**Algorithm**: Boustrophedon Coverage with Overlap Control | **Dimension**: 2D

---

## Problem Definition

**Setup**: A single drone is tasked with producing a complete orthophoto mosaic of a glacier
survey area ($100 \times 80$ m) in order to measure the current melt boundary. The drone flies at
a fixed altitude $h$, and its nadir-pointing camera covers a ground footprint of width
$w = 2h\tan(\theta_{FOV}/2)$. Adjacent flight strips must overlap by at least 20% in the
cross-track direction so that image-stitching software can register adjacent frames. The drone
begins at one corner of the glacier area and executes a **strip-based boustrophedon (lawnmower)**
path, flying alternating parallel strips along the long axis, with lateral steps at each end.

A simulated melt boundary is generated as a Gaussian-blurred binary mask representing the
ice/water interface. After coverage, the boundary polygon is extracted from the intensity map
by thresholding and contour detection.

**Roles**:
- **Drone**: single UAV executing a pre-planned orthophoto mosaic path; no in-flight replanning.
- **Glacier area**: static $100 \times 80$ m rectangle with a simulated melt boundary embedded as
  an intensity mask; discretised into $0.5 \times 0.5$ m grid cells for coverage accounting.

**Objective**: Minimise total flight distance (equivalently, total mission time) while guaranteeing
100% image coverage with at least 20% cross-track overlap throughout the survey area. Secondary
objective: extract the melt boundary as a closed polygon from the simulated orthophoto intensity
mask and compute the melt area.

**Key question**: How does the required overlap ratio affect total path length and mission time?
What is the minimum altitude that satisfies the Ground Sample Distance (GSD) constraint while
keeping the strip count manageable?

---

## Mathematical Model

### Camera Footprint and Ground Sample Distance

The drone flies at altitude $h$ with a camera whose horizontal field of view is $\theta_{FOV}$.
The ground footprint width (cross-track swath) is:

$$w = 2h \tan\!\left(\frac{\theta_{FOV}}{2}\right)$$

The Ground Sample Distance — the real-world distance represented by one image pixel — is:

$$\text{GSD} = \frac{s_w}{n_{px}} \cdot \frac{h}{f}$$

where $s_w$ is the physical sensor width (m), $n_{px}$ is the image width in pixels, and $f$ is
the focal length (m). For a fixed camera, GSD scales linearly with altitude $h$.

### Optimal Strip Spacing

Given a required cross-track overlap ratio $\rho \in [0, 1)$, the effective strip spacing
(centre-to-centre distance between adjacent parallel strips) is:

$$d^* = w \cdot (1 - \rho)$$

At $\rho = 0.20$ and $h = 10$ m with $\theta_{FOV} = 60°$:

$$w = 2 \times 10 \times \tan(30°) \approx 11.55 \text{ m}, \qquad d^* = 11.55 \times 0.80 \approx 9.24 \text{ m}$$

### Strip Count and Waypoint Generation

For a survey area of cross-track width $W$ with strips running along the along-track dimension
$L_{area}$, the number of strips required is:

$$N = \left\lceil \frac{W}{d^*} \right\rceil$$

Strip $i$ ($i = 0, 1, \ldots, N-1$) is centred at cross-track position:

$$y_i = y_{start} + \left(i + \tfrac{1}{2}\right) d^*$$

The along-track start and end of strip $i$ alternate direction (boustrophedon):

$$\mathbf{w}_{i}^{start} = \begin{cases} (x_{min},\; y_i) & i \text{ even} \\ (x_{max},\; y_i) & i \text{ odd} \end{cases}, \qquad \mathbf{w}_{i}^{end} = \begin{cases} (x_{max},\; y_i) & i \text{ even} \\ (x_{min},\; y_i) & i \text{ odd} \end{cases}$$

The full waypoint sequence is:

$$\mathbf{w}_0^{start},\; \mathbf{w}_0^{end},\; \mathbf{w}_1^{start},\; \mathbf{w}_1^{end},\; \ldots,\; \mathbf{w}_{N-1}^{start},\; \mathbf{w}_{N-1}^{end}$$

where each transition segment $\mathbf{w}_{i}^{end} \to \mathbf{w}_{i+1}^{start}$ is a lateral
step of length $d^*$ (the turn leg).

### Total Path Length

The total flight distance decomposes into $N$ along-track scan strips and $N-1$ lateral turn legs:

$$L_{total} = \underbrace{N \cdot L_{area}}_{\text{scan strips}} + \underbrace{(N-1) \cdot d^*}_{\text{turn legs}}$$

where $L_{area} = 100$ m is the along-track strip length and $W = 80$ m is the cross-track width.

### Coverage Efficiency

The survey area is discretised into a grid of $0.5 \times 0.5$ m cells. A cell at position
$(x_c, y_c)$ is marked **imaged** when the drone's ground projection passes within half the
footprint width of the cell's $y$-coordinate:

$$\text{cell}(x_c, y_c) \text{ imaged} \iff \exists\, i : |y_c - y_i| \leq \tfrac{w}{2}$$

The coverage rate and efficiency are:

$$C = \frac{\bigl|\{(x_c, y_c) : \text{imaged}\}\bigr|}{W \cdot L_{area}} \times 100\%$$

$$\eta = \frac{\text{unique area covered}}{N \cdot w \cdot L_{area}}$$

where $\eta < 1$ reflects that strip edges overlap and $\eta = 1$ would be the idealised
non-overlapping case.

### Melt Boundary Extraction

The simulated orthophoto intensity mask $I(x, y)$ is constructed as a Gaussian-blurred binary
field separating ice (high intensity) from melt water (low intensity). The melt boundary is
extracted by:

1. Applying a binary threshold at intensity $I_{thresh} = 0.5$:

$$M(x, y) = \mathbf{1}\!\left[I(x, y) \geq I_{thresh}\right]$$

2. Running contour extraction on $M$ (e.g., marching squares) to obtain the boundary polygon
   $\mathcal{B} = \{(x_k, y_k)\}_{k=1}^{K}$.

3. Computing the melt area as the complement of the ice area within the survey region:

$$A_{melt} = (W \cdot L_{area}) - \frac{1}{2} \left|\sum_{k=0}^{K-1}(x_k y_{k+1} - x_{k+1} y_k)\right|$$

using the shoelace formula for the ice polygon area.

---

## Implementation

```python
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
from scipy.ndimage import gaussian_filter

# ── Key constants ──────────────────────────────────────────────────────────────
AREA_L          = 100.0    # m — along-track length of survey area
AREA_W          = 80.0     # m — cross-track width of survey area
SCAN_HEIGHT     = 10.0     # m — drone altitude (fixed)
SENSOR_FOV_DEG  = 60.0     # deg — camera horizontal field of view
OVERLAP_RATIO   = 0.20     # — required cross-track overlap (20%)
GRID_RESOLUTION = 0.5      # m — coverage grid cell size
V_CRUISE        = 5.0      # m/s — drone cruise speed
DT              = 0.2      # s — simulation timestep
I_THRESH        = 0.5      # — intensity threshold for melt boundary

# ── Derived geometry ───────────────────────────────────────────────────────────
FOV_RAD   = np.deg2rad(SENSOR_FOV_DEG)
FOOTPRINT = 2.0 * SCAN_HEIGHT * np.tan(FOV_RAD / 2.0)   # swath width w (m)
STRIP_D   = FOOTPRINT * (1.0 - OVERLAP_RATIO)            # optimal strip spacing d* (m)
N_STRIPS  = int(np.ceil(AREA_W / STRIP_D))               # number of strips N


def generate_lawnmower_waypoints(area_l, area_w, strip_d):
    """Return ordered (x, y) waypoints for a boustrophedon orthophoto survey."""
    n_strips = int(np.ceil(area_w / strip_d))
    waypoints = []
    for i in range(n_strips):
        y_centre = (i + 0.5) * strip_d
        if i % 2 == 0:          # left to right (increasing x)
            waypoints.append((0.0,    y_centre))
            waypoints.append((area_l, y_centre))
        else:                   # right to left (decreasing x)
            waypoints.append((area_l, y_centre))
            waypoints.append((0.0,    y_centre))
    return waypoints, n_strips


def compute_path_length(waypoints):
    """Total Euclidean path length along the waypoint sequence."""
    total = 0.0
    pts = np.array(waypoints)
    for k in range(len(pts) - 1):
        total += np.linalg.norm(pts[k + 1] - pts[k])
    return total


def simulate_coverage(waypoints, area_l, area_w, footprint_w, grid_res, v_cruise, dt):
    """Simulate drone flight; return coverage grid (scan-count) and trajectory."""
    nx = int(area_l / grid_res)
    ny = int(area_w / grid_res)
    grid = np.zeros((ny, nx), dtype=np.int32)

    xs = np.arange(nx) * grid_res + grid_res / 2.0   # cell-centre x coords
    ys = np.arange(ny) * grid_res + grid_res / 2.0   # cell-centre y coords
    CX, CY = np.meshgrid(xs, ys)

    trajectory = []
    pos = np.array(waypoints[0], dtype=float)
    trajectory.append(pos.copy())

    for wp in waypoints[1:]:
        target = np.array(wp, dtype=float)
        diff   = target - pos
        dist   = np.linalg.norm(diff)
        if dist < 1e-6:
            continue
        unit  = diff / dist
        steps = max(1, int(np.ceil(dist / (v_cruise * dt))))
        step_dist = dist / steps
        for _ in range(steps):
            pos = pos + unit * step_dist
            trajectory.append(pos.copy())
            # Mark cells within cross-track half-swath
            in_swath = np.abs(CY - pos[1]) <= footprint_w / 2.0
            grid[in_swath] += 1

    return np.array(trajectory), grid


def generate_melt_mask(area_l, area_w, grid_res, sigma=12.0, seed=42):
    """
    Simulate an orthophoto intensity mask with a smooth melt boundary.
    Ice (high intensity) occupies the upper portion; melt water (low) below.
    A Gaussian-blurred random field creates a realistic irregular boundary.
    """
    rng = np.random.default_rng(seed)
    nx  = int(area_l / grid_res)
    ny  = int(area_w / grid_res)

    # Base gradient: ice in upper half (high y), melt water in lower half
    ys = np.linspace(0, 1, ny)
    base = np.tile(ys[:, np.newaxis], (1, nx))

    # Add random perturbation and smooth to simulate fractal boundary
    noise = rng.standard_normal((ny, nx))
    noise_smooth = gaussian_filter(noise, sigma=sigma / grid_res)
    mask = base + 0.3 * noise_smooth / noise_smooth.std()

    # Normalise to [0, 1]
    mask = (mask - mask.min()) / (mask.max() - mask.min())
    return mask


def extract_melt_boundary(mask, i_thresh, grid_res):
    """
    Extract melt boundary polygon by thresholding the intensity mask.
    Returns binary ice map and approximate boundary cell coordinates.
    """
    ice_map = (mask >= i_thresh).astype(np.uint8)

    # Find boundary cells (ice adjacent to non-ice) using finite differences
    pad = np.pad(ice_map, 1, constant_values=0)
    boundary_mask = np.zeros_like(ice_map, dtype=bool)
    for di, dj in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
        neighbour = pad[1 + di:ice_map.shape[0] + 1 + di,
                        1 + dj:ice_map.shape[1] + 1 + dj]
        boundary_mask |= (ice_map == 1) & (neighbour == 0)

    rows, cols = np.where(boundary_mask)
    bx = cols * grid_res + grid_res / 2.0   # x coordinate (along-track)
    by = rows * grid_res + grid_res / 2.0   # y coordinate (cross-track)

    # Approximate melt area as complement of ice area
    ice_cells  = np.sum(ice_map == 1)
    ice_area   = ice_cells * grid_res**2
    total_area = ice_map.size * grid_res**2
    melt_area  = total_area - ice_area

    return ice_map, bx, by, melt_area


def compute_metrics(waypoints, n_strips, area_l, area_w, footprint_w,
                    strip_d, grid_res, v_cruise, dt):
    """Compute and print mission metrics; return trajectory and grid."""
    traj, grid = simulate_coverage(
        waypoints, area_l, area_w, footprint_w, grid_res, v_cruise, dt
    )
    L          = n_strips * area_l + (n_strips - 1) * strip_d
    t_mission  = L / v_cruise
    total_cells = grid.size
    covered     = int(np.sum(grid > 0))
    coverage_pct = 100.0 * covered / total_cells
    eta          = (covered * grid_res**2) / (n_strips * footprint_w * area_l)

    print(f"Footprint width w     : {footprint_w:.2f} m")
    print(f"Strip spacing d*      : {strip_d:.2f} m")
    print(f"Number of strips N    : {n_strips}")
    print(f"Total path length L   : {L:.1f} m")
    print(f"Mission time          : {t_mission:.1f} s  ({t_mission/60:.2f} min)")
    print(f"Coverage rate         : {coverage_pct:.1f}%")
    print(f"Coverage efficiency η : {eta:.3f}")
    return traj, grid, L, t_mission, coverage_pct, eta


def run_simulation():
    waypoints, n_strips = generate_lawnmower_waypoints(AREA_L, AREA_W, STRIP_D)

    traj, grid, L, t_miss, cov_pct, eta = compute_metrics(
        waypoints, n_strips, AREA_L, AREA_W, FOOTPRINT,
        STRIP_D, GRID_RESOLUTION, V_CRUISE, DT
    )

    mask = generate_melt_mask(AREA_L, AREA_W, GRID_RESOLUTION)
    ice_map, bx, by, melt_area = extract_melt_boundary(mask, I_THRESH, GRID_RESOLUTION)

    print(f"\nGlacier total area    : {AREA_L * AREA_W:.0f} m²")
    print(f"Ice area              : {np.sum(ice_map) * GRID_RESOLUTION**2:.1f} m²")
    print(f"Melt area             : {melt_area:.1f} m²")
    print(f"Melt fraction         : {melt_area / (AREA_L * AREA_W) * 100:.1f}%")

    return {
        "waypoints"   : waypoints,
        "trajectory"  : traj,
        "grid"        : grid,
        "mask"        : mask,
        "ice_map"     : ice_map,
        "boundary_x"  : bx,
        "boundary_y"  : by,
        "melt_area"   : melt_area,
        "n_strips"    : n_strips,
        "path_length" : L,
        "mission_time": t_miss,
        "coverage_pct": cov_pct,
        "efficiency"  : eta,
    }
```

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Survey area | 100 × 80 m |
| Scan altitude $h$ | 10.0 m |
| Camera FOV $\theta_{FOV}$ | 60° |
| Image footprint width $w$ | $\approx 11.55$ m |
| Required cross-track overlap $\rho$ | 20% |
| Optimal strip spacing $d^*$ | $\approx 9.24$ m |
| Number of strips $N$ | $\lceil 80 / 9.24 \rceil = 9$ |
| Total path length $L$ | $\approx 974$ m |
| Drone cruise speed $v$ | 5.0 m/s |
| Estimated mission time | $\approx 195$ s |
| Grid resolution | 0.5 × 0.5 m |
| Simulation timestep $\Delta t$ | 0.2 s |
| Melt boundary intensity threshold $I_{thresh}$ | 0.5 |
| Gaussian blur sigma for melt mask | 12.0 m (effective) |

---

## Expected Output

- **Flight path diagram**: 2D top-down view of the full boustrophedon waypoint sequence over the
  $100 \times 80$ m glacier area; strips colour-coded by index (blue early, red late); turn legs
  shown as dashed segments; footprint swath boundaries drawn as translucent grey bands along each
  strip.
- **Coverage heatmap**: scan-count grid rendered as a top-down 2D heatmap (white = 0 scans,
  light blue = 1 scan, dark blue = 2+ scans in overlap zones); drone trajectory overlaid as an
  orange line; colour bar labelled "Scan count per cell".
- **Simulated orthophoto + melt boundary**: intensity mask $I(x,y)$ displayed as a grayscale
  image with the extracted melt boundary polygon overlaid as a red contour; ice region labelled
  in white, melt water region in blue; melt area printed in the figure title.
- **Overlap sensitivity plot**: total path length $L$ and strip count $N$ plotted as functions of
  overlap ratio $\rho \in [0\%, 50\%]$; vertical dashed line at the required $\rho = 20\%$;
  secondary y-axis showing mission time in minutes.
- **Animation (GIF)**: real-time top-down view of the drone sweeping the lawnmower path strip by
  strip; coverage cells fill in blue as the drone passes; melt boundary contour visible in the
  background; current strip index and cumulative coverage rate displayed in the title.
- **Printed metrics** (console): footprint width, strip spacing, strip count, total path length,
  mission time, coverage rate, coverage efficiency $\eta$, ice area, melt area, melt fraction.

---

## Extensions

1. **Non-rectangular glacier polygon**: replace the rectangular survey area with an arbitrary
   convex or concave glacier boundary polygon; compute the boustrophedon decomposition by clipping
   each strip against the polygon and flying only the clipped segments, reducing wasted overflights
   outside the ice extent.
2. **Altitude-GSD trade-off**: sweep altitude $h \in [5, 30]$ m and plot GSD and mission time
   jointly; identify the Pareto-optimal altitude that minimises mission time subject to a
   maximum GSD constraint (e.g. GSD $\leq 2$ cm/pixel for change-detection accuracy).
3. **Longitudinal overlap constraint**: add a minimum along-track overlap $\rho_{lon}$ (e.g. 60%)
   driven by forward motion blur and compute the maximum permissible cruise speed
   $v_{max} = (1 - \rho_{lon}) \cdot f_{image} \cdot w_{footprint}$ where $f_{image}$ is the
   image capture rate; integrate this speed constraint into path-length optimisation.
4. **Multi-temporal change detection**: run the simulation at $t = 0$ and $t = T_{season}$ with
   different melt masks; compute the symmetric difference polygon between the two ice boundaries
   to estimate seasonal melt area and compare with ground-truth reference polygons.
5. **Wind-corrected crab-angle flight**: add a constant crosswind that displaces the drone
   laterally; compute the required crab angle $\alpha = \arcsin(v_{wind}/v_{air})$ to maintain
   straight strips; adjust effective swath width for the resulting ground track angle.

---

## Related Scenarios

- Prerequisites: [S041 Wildfire Boundary Scan](S041_wildfire_boundary.md) (boundary extraction from sensor masks), [S048 Full-Area Coverage Scan](S048_lawnmower.md) (boustrophedon path planning fundamentals)
- Follow-ups: [S055 Coastline Oil Spill Tracking](S055_oil_spill.md) (dynamic contamination boundary over a mapped region)
- Algorithmic cross-reference: [S067 Spray Overlap Optimisation](../../04_industrial_agriculture/S067_spray_overlap_optimization.md) (same strip-spacing trade-off in precision agriculture)
