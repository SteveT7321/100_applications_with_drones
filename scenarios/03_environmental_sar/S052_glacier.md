# S052 Glacier Melt Area Measurement

**Domain**: Environmental Monitoring & SAR | **Difficulty**: ⭐⭐ | **Status**: `[ ]` Not Started

---

## Problem Definition

**Setup**: A single survey drone is deployed over a mountain glacier to measure changes in melt area
between seasonal surveys. The glacier occupies a roughly rectangular region of
$L_{area} \times W_{area} = 800 \times 600$ m. The drone carries a downward-facing RGB+NIR camera
and flies a systematic boustrophedon (lawnmower) grid at a fixed survey altitude $h = 80$ m above
the glacier surface. The camera's field of view and the required overlap ratios determine strip
spacing and photo interval, ensuring every point on the glacier is captured by at least three
overlapping images — the minimum for reliable Structure-from-Motion (SfM) photogrammetry and
orthophoto stitching. After the survey flight, NDWI (Normalised Difference Water Index) is computed
from the mosaicked orthophoto to segment melt-water pools from clean ice, and the total melt area
$A_{melt}$ is reported in m².

**Roles**:
- **Survey drone**: single fixed-wing or multirotor at altitude $h$; flies the coverage path,
  triggers the camera at intervals $\Delta x = B_x$, and returns to the launch point at the end of
  each flight leg.

**Objective**: Design a photogrammetric coverage path that guarantees forward overlap
$O_f = 80\%$ and side overlap $O_s = 60\%$ across the entire glacier area, minimise total flight
distance, and produce an NDWI-derived melt-area estimate $A_{melt}$ with ground sampling distance
$\text{GSD} \leq 5$ cm/pixel.

**Comparison strategies**:
1. **Unidirectional lawnmower** — all strips flown in the same direction; drone repositions at the
   far end of each strip before starting the next.
2. **Boustrophedon lawnmower** — strips flown alternately left-to-right and right-to-left;
   eliminates the long repositioning leg between strips.
3. **Adaptive strip spacing** — strip spacing reduced in marginal zones identified by a prior coarse
   pass to ensure overlap is maintained on sloped glacier flanks.

---

## Mathematical Model

### Ground Sampling Distance

The ground sampling distance (GSD) relates sensor pixel pitch $p$ (mm), focal length $f$ (mm), and
survey altitude $h$ (m):

$$\text{GSD} = \frac{h \cdot p}{f}$$

For a sensor of total width $s_w$ (mm) and height $s_h$ (mm) with image resolution
$n_w \times n_h$ pixels, the pixel pitch is $p = s_w / n_w$, and the camera footprint on the
ground is:

$$W_{fp} = \frac{h \cdot s_w}{f}, \qquad L_{fp} = \frac{h \cdot s_h}{f}$$

Equivalently, using the horizontal and vertical half-angles of the field of view:

$$W_{fp} = 2h \tan\!\left(\frac{\text{FOV}_h}{2}\right), \qquad
L_{fp} = 2h \tan\!\left(\frac{\text{FOV}_v}{2}\right)$$

### Coverage Path Geometry

With forward overlap $O_f$ (fraction) along the flight direction and side overlap $O_s$ (fraction)
across strips, the base distance (trigger interval along-track) and strip spacing are:

$$B_x = L_{fp} \cdot (1 - O_f)$$

$$B_y = W_{fp} \cdot (1 - O_s)$$

The number of parallel strips required to cover the glacier width $W_{area}$:

$$N_{strips} = \left\lceil \frac{W_{area}}{B_y} \right\rceil$$

The number of photo triggers per strip to cover the glacier length $L_{area}$:

$$N_{photos} = \left\lceil \frac{L_{area}}{B_x} \right\rceil$$

Total photos acquired during the survey:

$$N_{total} = N_{strips} \times N_{photos}$$

### Flight Path Length

For the boustrophedon pattern, the drone flies $N_{strips}$ legs each of length $L_{area}$, with
$N_{strips} - 1$ short cross-track transitions of length $B_y$:

$$D_{flight} = N_{strips} \cdot L_{area} + (N_{strips} - 1) \cdot B_y$$

For the unidirectional pattern, each transition is a full repositioning leg of length
$L_{area} + B_y$, giving a longer total:

$$D_{uni} = N_{strips} \cdot L_{area} + (N_{strips} - 1) \cdot (L_{area} + B_y)$$

The boustrophedon saving over unidirectional flight:

$$\Delta D = D_{uni} - D_{flight} = (N_{strips} - 1) \cdot L_{area}$$

### Drone Kinematics

The drone flies at constant speed $v$ and fixed altitude $h$. Position update at timestep $\Delta t$:

$$\mathbf{p}(t + \Delta t) = \mathbf{p}(t) + v \cdot \hat{\mathbf{u}}(t) \cdot \Delta t$$

where $\hat{\mathbf{u}}(t)$ is the unit vector toward the current waypoint. Total survey time:

$$T_{survey} = \frac{D_{flight}}{v}$$

### Camera Trigger Model

A photo is triggered when the along-track distance from the last trigger position exceeds $B_x$:

$$\text{trigger at } \mathbf{p}_k \iff \|\mathbf{p}_k - \mathbf{p}_{k-1}\|_{along} \geq B_x$$

The footprint of photo $k$ is the axis-aligned rectangle centred at the nadir point
$\mathbf{p}_k^{xy}$ with dimensions $W_{fp} \times L_{fp}$.

### NDWI Melt-Area Estimation

After flight, the orthophoto mosaic is assembled from all photo footprints. Each pixel $(u, v)$ of
the mosaic carries a green-band radiance $\rho_G(u,v)$ and a near-infrared radiance $\rho_{NIR}(u,v)$.
The Normalised Difference Water Index is:

$$\text{NDWI}(u,v) = \frac{\rho_G(u,v) - \rho_{NIR}(u,v)}{\rho_G(u,v) + \rho_{NIR}(u,v)}$$

Melt-water pixels are classified by thresholding:

$$\text{melt}(u,v) = \begin{cases} 1 & \text{if } \text{NDWI}(u,v) > \theta \\ 0 & \text{otherwise} \end{cases}$$

with threshold $\theta = 0.2$ (standard open-water threshold). The total melt area in m² is:

$$A_{melt} = \sum_{u,v} \text{melt}(u,v) \cdot \text{GSD}^2$$

To simulate realistic NDWI, the synthetic glacier mosaic models clean ice as having low NDWI
(mean $\mu_{ice} = -0.3$, $\sigma = 0.05$) and melt pools as having high NDWI (mean $\mu_{melt} = 0.4$,
$\sigma = 0.08$):

$$\text{NDWI}_{synthetic}(u,v) \sim \begin{cases} \mathcal{N}(-0.3,\; 0.05^2) & \text{pixel in ice region} \\ \mathcal{N}(0.4,\; 0.08^2) & \text{pixel in melt region} \end{cases}$$

### Overlap Coverage Map

The coverage count at ground point $\mathbf{q} = (x, y)$ is the number of photo footprints that
contain it:

$$C(\mathbf{q}) = \sum_{k=1}^{N_{total}} \mathbf{1}\bigl[\mathbf{q} \in \text{footprint}_k\bigr]$$

The minimum coverage over the survey area must satisfy $\min_{\mathbf{q}} C(\mathbf{q}) \geq 3$
for SfM reconstruction. The mean coverage:

$$\bar{C} = \frac{1}{|O_f|} \cdot \frac{1}{|O_s|}$$

where $\bar{C} = 1 / ((1 - O_f)(1 - O_s))$ follows directly from the overlap fractions.

---

## Implementation

```python
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.colors import Normalize
import matplotlib.cm as cm

# Key constants — photogrammetry parameters
ALTITUDE        = 80.0      # m — survey altitude above glacier surface
FOCAL_LENGTH    = 25.0      # mm — camera focal length
SENSOR_W        = 17.3      # mm — sensor width (micro four-thirds)
SENSOR_H        = 13.0      # mm — sensor height
IMAGE_W         = 4608      # px — image width
IMAGE_H         = 3456      # px — image height

FORWARD_OVERLAP = 0.80      # fraction — along-track overlap
SIDE_OVERLAP    = 0.60      # fraction — cross-track overlap

GLACIER_L       = 800.0     # m — glacier length (along flight strips)
GLACIER_W       = 600.0     # m — glacier width (across strips)
DRONE_SPEED     = 10.0      # m/s — cruise speed
DT              = 0.5       # s — simulation timestep
NDWI_THRESHOLD  = 0.2       # NDWI melt-water classification threshold

def compute_footprint(altitude, focal_length_mm, sensor_w_mm, sensor_h_mm):
    """Compute ground footprint width and height from camera parameters."""
    fp_w = altitude * sensor_w_mm / focal_length_mm
    fp_h = altitude * sensor_h_mm / focal_length_mm
    gsd  = altitude * (sensor_w_mm / IMAGE_W) / focal_length_mm * 1000  # m/px → cm/px
    return fp_w, fp_h, gsd

def compute_strip_geometry(fp_w, fp_h, forward_overlap, side_overlap,
                           glacier_l, glacier_w):
    """Derive strip spacing, base distance, and photo counts."""
    Bx = fp_h * (1 - forward_overlap)   # trigger interval (along strip)
    By = fp_w * (1 - side_overlap)       # strip spacing (cross-track)
    n_strips = int(np.ceil(glacier_w / By))
    n_photos = int(np.ceil(glacier_l / Bx))
    return Bx, By, n_strips, n_photos

def boustrophedon_waypoints(n_strips, By, glacier_l, origin=(0.0, 0.0)):
    """Generate boustrophedon strip waypoints."""
    waypoints = []
    ox, oy = origin
    for i in range(n_strips):
        x = ox + i * By + By / 2
        if i % 2 == 0:
            waypoints.append((x, oy))
            waypoints.append((x, oy + glacier_l))
        else:
            waypoints.append((x, oy + glacier_l))
            waypoints.append((x, oy))
    return waypoints

def simulate_survey_flight(waypoints, drone_speed, Bx, dt):
    """Simulate drone flight and record trigger positions."""
    positions   = []
    triggers    = []
    pos         = np.array(waypoints[0], dtype=float)
    last_trig   = pos.copy()
    t           = 0.0

    for wp in waypoints[1:]:
        target = np.array(wp, dtype=float)
        while True:
            diff = target - pos
            dist = np.linalg.norm(diff)
            if dist < drone_speed * dt:
                pos = target.copy()
                positions.append(pos.copy())
                t += dist / drone_speed
                break
            step = diff / dist * drone_speed * dt
            pos += step
            positions.append(pos.copy())
            t += dt
            # Camera trigger check (along y-axis for N-S strips)
            along_dist = np.linalg.norm(pos - last_trig)
            if along_dist >= Bx:
                triggers.append(pos.copy())
                last_trig = pos.copy()

    return np.array(positions), np.array(triggers), t

def build_coverage_map(triggers, fp_w, fp_h, glacier_l, glacier_w,
                       gsd_m, resolution=1.0):
    """Rasterise photo footprints into a coverage-count grid."""
    nx = int(glacier_w / resolution)
    ny = int(glacier_l / resolution)
    coverage = np.zeros((ny, nx), dtype=np.int32)

    for tx, ty in triggers:
        x0 = int((tx - fp_w / 2) / resolution)
        x1 = int((tx + fp_w / 2) / resolution)
        y0 = int((ty - fp_h / 2) / resolution)
        y1 = int((ty + fp_h / 2) / resolution)
        x0, x1 = max(0, x0), min(nx, x1)
        y0, y1 = max(0, y0), min(ny, y1)
        coverage[y0:y1, x0:x1] += 1

    return coverage

def synthetic_ndwi(glacier_l, glacier_w, gsd_m, melt_fraction=0.25, seed=42):
    """Generate a synthetic NDWI mosaic with Gaussian noise per class."""
    rng = np.random.default_rng(seed)
    nx  = int(glacier_w / gsd_m)
    ny  = int(glacier_l / gsd_m)
    ndwi = rng.normal(-0.3, 0.05, (ny, nx))

    # Place circular melt pools
    n_pools = 8
    for _ in range(n_pools):
        cx = rng.integers(nx // 6, 5 * nx // 6)
        cy = rng.integers(ny // 6, 5 * ny // 6)
        r  = rng.integers(nx // 20, nx // 8)
        yy, xx = np.ogrid[:ny, :nx]
        mask = (xx - cx)**2 + (yy - cy)**2 <= r**2
        ndwi[mask] = rng.normal(0.4, 0.08, mask.sum())

    return ndwi

def compute_melt_area(ndwi, gsd_m, threshold=0.2):
    """Threshold NDWI and compute melt area in m²."""
    melt_mask = ndwi > threshold
    return melt_mask.sum() * gsd_m**2, melt_mask

def run_simulation():
    fp_w, fp_h, gsd_cm = compute_footprint(
        ALTITUDE, FOCAL_LENGTH, SENSOR_W, SENSOR_H
    )
    gsd_m = gsd_cm / 100.0  # convert cm/px to m/px (gsd_cm is actually in m here)
    # Recompute properly: gsd in metres
    gsd_m = ALTITUDE * (SENSOR_W / IMAGE_W) / FOCAL_LENGTH  # m/px

    Bx, By, n_strips, n_photos = compute_strip_geometry(
        fp_w, fp_h, FORWARD_OVERLAP, SIDE_OVERLAP, GLACIER_L, GLACIER_W
    )

    waypoints = boustrophedon_waypoints(n_strips, By, GLACIER_L)
    positions, triggers, T_survey = simulate_survey_flight(
        waypoints, DRONE_SPEED, Bx, DT
    )

    coverage = build_coverage_map(triggers, fp_w, fp_h,
                                  GLACIER_L, GLACIER_W, gsd_m, resolution=1.0)
    ndwi     = synthetic_ndwi(GLACIER_L, GLACIER_W, gsd_m)
    A_melt, melt_mask = compute_melt_area(ndwi, gsd_m, NDWI_THRESHOLD)

    return {
        "fp_w": fp_w, "fp_h": fp_h, "gsd_m": gsd_m,
        "Bx": Bx, "By": By,
        "n_strips": n_strips, "n_photos": n_photos,
        "n_total": len(triggers),
        "T_survey": T_survey,
        "positions": positions,
        "triggers": triggers,
        "coverage": coverage,
        "ndwi": ndwi,
        "melt_mask": melt_mask,
        "A_melt": A_melt,
    }
```

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Survey altitude $h$ | 80 m |
| Focal length $f$ | 25 mm |
| Sensor size | 17.3 × 13.0 mm (micro four-thirds) |
| Image resolution | 4608 × 3456 px |
| GSD | $\approx 4.8$ cm/px |
| Camera footprint | $\approx 55.4 \times 41.6$ m |
| Forward overlap $O_f$ | 80% |
| Side overlap $O_s$ | 60% |
| Base distance $B_x$ | $\approx 8.3$ m |
| Strip spacing $B_y$ | $\approx 22.2$ m |
| Glacier area | 800 × 600 m |
| Number of strips $N_{strips}$ | $\approx 27$ |
| Photos per strip $N_{photos}$ | $\approx 97$ |
| Total photos $N_{total}$ | $\approx 2619$ |
| Drone cruise speed $v$ | 10 m/s |
| Survey flight distance | $\approx 21.7$ km |
| Estimated survey time | $\approx 36$ min |
| NDWI threshold $\theta$ | 0.2 |
| Minimum coverage count | 3 images/point |
| Mean coverage $\bar{C}$ | $= 1 / ((1-O_f)(1-O_s)) = 12.5$ images/point |

---

## Expected Output

- **Coverage path map**: 2D top-down view of the glacier area showing the boustrophedon strip
  flight path (grey line), photo trigger positions (blue dots), and camera footprint rectangles
  at a sampled subset of triggers; strip numbering and flight direction arrows annotated.
- **Coverage count heatmap**: rasterised grid showing the number of overlapping photo footprints
  per ground cell; colour scale from 0 (uncovered, red) to $\geq 15$ (deep blue); annotated with
  minimum and mean coverage values.
- **Synthetic NDWI mosaic**: false-colour map of the entire glacier area with NDWI values rendered
  on a diverging blue-to-red colormap; melt-water pools appear in blue ($\text{NDWI} > 0.2$) and
  clean ice in red/white.
- **Melt area classification map**: binary mask overlaid on the NDWI mosaic highlighting classified
  melt pixels; total melt area $A_{melt}$ displayed in the title in m² and as a percentage of the
  glacier area.
- **Strategy comparison bar chart**: total flight distance and survey time for unidirectional
  vs boustrophedon vs adaptive patterns; percentage saving annotated on each bar.
- **GSD vs altitude curve**: plot of GSD (cm/px) as a function of survey altitude (40–160 m) for
  the given camera, with the selected operating point and $\text{GSD} = 5$ cm threshold marked.

---

## Extensions

1. **Terrain-following altitude**: use a digital elevation model (DEM) of the glacier surface to
   maintain constant GSD despite elevation variation; vary $h$ along-track to keep
   $\text{GSD} = \text{const}$.
2. **Multi-temporal change detection**: run two simulated surveys separated by a notional 30-day
   melt period; compute $\Delta A_{melt} = A_{melt}^{(2)} - A_{melt}^{(1)}$ and map the spatial
   pattern of advance and retreat.
3. **Oblique strip insertion**: add side-looking oblique strips at the glacier margins to capture
   vertical ice-cliff faces; model the 3D footprint geometry on vertical surfaces.
4. **3D SfM point cloud**: replace the 2D orthophoto model with a simplified SfM depth estimation
   (planar homography + triangulation) to reconstruct glacier surface elevation and estimate
   volumetric ice loss.
5. **Wind-drift compensation**: model lateral wind displacing the nadir point from the planned
   strip centreline; compute the resulting along-track and cross-track footprint registration
   errors and the minimum wind speed that violates the overlap constraint.

---

## Related Scenarios

- Prerequisites: [S041 Wildfire Boundary Scan](S041_wildfire_boundary_scan.md), [S048 Lawnmower Coverage](S048_lawnmower_coverage.md)
- Follow-ups: [S053 Coral Reef 3D Mapping](S053_coral_reef_3d_mapping.md) (underwater photogrammetry), [S057 Wildlife Census](S057_wildlife_census.md) (orthophoto-based counting)
- Algorithmic cross-reference: [S049 Dynamic Zone Assignment](S049_dynamic_zone_assignment.md) (adaptive coverage), [S065 Building 3D Scan Path](../../04_industrial_agriculture/S065_building_3d_scan_path.md) (viewpoint planning)
