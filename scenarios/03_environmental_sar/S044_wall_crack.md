# S044 Wall Crack Inspection

**Domain**: Environmental Monitoring & SAR | **Difficulty**: ⭐⭐ | **Status**: `[x]` Completed

---

## Problem Definition

**Setup**: A reinforced-concrete building facade is modelled as a vertical plane
$50\ \text{m}$ wide $\times$ $20\ \text{m}$ tall. The facade contains $N_c$ synthetic cracks at
known ground-truth positions (generated for evaluation); crack widths $w_i$ are drawn from
$\mathcal{U}(0.1, 2.0)$ mm. A single inspection drone must fly a boustrophedon (serpentine)
path in the $(y, z)$ side-view plane — horizontal position along the wall $y \in [0, 50]$ m,
height $z \in [0.5, 19.5]$ m — while maintaining a constant standoff distance $d_{standoff} =
1.5$ m perpendicular to the wall ($x$-axis). A PD controller continuously corrects lateral drift
to hold the standoff. Camera images are acquired at each step; a crack is detected when the drone
is within the detection range and the crack is wide enough to exceed the angular resolution limit
of the camera sensor.

**Objective**: Achieve complete surface coverage (zero unvisited strips) while maximising the
number of cracks detected, subject to the constraint that every wall point is observed at least
once within detection range. Report coverage fraction, detection rate by crack width bin, and
total flight distance.

---

## Mathematical Model

### Wall Coordinate Frame

The wall occupies the plane $x = 0$. Drone state in the side-view plane:

$$\mathbf{q}(t) = \bigl(y(t),\; z(t),\; \dot{y}(t),\; \dot{z}(t)\bigr)^{\top}$$

The standoff distance from the wall is $d(t) = x_{drone}(t)$, held fixed at $d_{standoff} = 1.5$ m
by a separate depth-hold loop (not modelled in the 2D scan path).

### Camera Strip Width and Overlap

Camera half-angle $\alpha = \text{FOV}/2 = 30°$. At standoff $d_{standoff}$, the ground-projected
strip half-width is:

$$h_{strip} = d_{standoff} \cdot \tan(\alpha) = 1.5 \cdot \tan(30°) \approx 0.866\ \text{m}$$

Full strip width: $w_{strip} = 2\, h_{strip} \approx 1.732\ \text{m}$.

To guarantee no gap between adjacent horizontal passes an overlap fraction $\rho_{ov} \in [0, 1)$
is applied. The vertical step between pass centres:

$$\Delta z = w_{strip} \cdot (1 - \rho_{ov})$$

With $\rho_{ov} = 0.2$: $\Delta z = 1.732 \times 0.8 \approx 1.386\ \text{m}$.

Number of horizontal passes required to cover the full $H = 20$ m wall:

$$N_{pass} = \left\lceil \frac{H}{{\Delta z}} \right\rceil = \left\lceil \frac{20}{1.386} \right\rceil = 15$$

Pass centre heights:

$$z_k = z_{start} + \left(k - \frac{1}{2}\right)\Delta z, \quad k = 1, \ldots, N_{pass}$$

where $z_{start} = 0.5\ \text{m}$ (minimum safe altitude).

### Boustrophedon Waypoint Sequence

Pass $k$ is flown left-to-right for odd $k$ and right-to-left for even $k$:

$$\mathbf{w}_k^{start} = \begin{cases} (0,\; z_k) & k \text{ odd} \\ (W,\; z_k) & k \text{ even} \end{cases}, \qquad \mathbf{w}_k^{end} = \begin{cases} (W,\; z_k) & k \text{ odd} \\ (0,\; z_k) & k \text{ even} \end{cases}$$

with $W = 50$ m (wall width). The vertical transition between pass $k$ and pass $k+1$ is a
straight climb/descent at the turnaround $y$-position.

Total path length:

$$L_{total} = N_{pass} \cdot W + (N_{pass} - 1) \cdot \Delta z$$

### PD Standoff Controller

The drone's $x$-position is regulated to $d_{standoff}$ by a PD law:

$$a_x(t) = K_p \bigl(d_{standoff} - x(t)\bigr) - K_d\, \dot{x}(t)$$

Critically-damped gains with natural frequency $\omega_n = 4\ \text{rad/s}$:

$$K_p = \omega_n^2 = 16\ \text{s}^{-2}, \qquad K_d = 2\,\omega_n = 8\ \text{s}^{-1}$$

Steady-state standoff error (with additive Gaussian wall-distance noise $\sigma_x = 0.05$ m):

$$e_{ss} = \frac{\sigma_x}{K_p / K_d} \approx 0.025\ \text{m}$$

### Along-Wall Speed Controller

Cruise speed along the wall $v_{cruise} = 2.0$ m/s. A proportional height controller holds
the drone on the current pass altitude:

$$a_z(t) = K_p^{(z)} \bigl(z_{target}(t) - z(t)\bigr) - K_d^{(z)}\, \dot{z}(t)$$

with $K_p^{(z)} = 9\ \text{s}^{-2}$, $K_d^{(z)} = 6\ \text{s}^{-1}$.

### Crack Detection Model

Crack $i$ at wall position $(y_i, z_i)$ with width $w_i$ (mm) is detected if two conditions hold
simultaneously:

**Condition 1 — Range**: the drone is within maximum detection range $d_{max}$:

$$d_{det}(i, t) = \sqrt{(y(t) - y_i)^2 + (z(t) - z_i)^2} \leq d_{max}$$

with $d_{max} = h_{strip} / \cos(0) = 0.866\ \text{m}$ (crack must fall within the current image
frame, accounting for the range component along the wall direction).

**Condition 2 — Angular resolution**: the crack subtends at least one pixel at the sensor.
Camera resolution $R_{px} = 2048$ px across the FOV. The minimum resolvable crack width:

$$w_{min}(d) = \frac{2\, d \cdot \tan(\alpha)}{R_{px}} \cdot 1000\ \text{mm}
             = \frac{2 \times 1.5 \times \tan(30°)}{2048} \times 1000 \approx 0.847\ \mu\text{m}$$

In practice, detection is probabilistic; the detection probability at slant range $r$:

$$P_{det}(w_i, r) = \Phi\!\left(\frac{w_i - w_{min}(r)}{\sigma_{noise}}\right)$$

where $\Phi$ is the standard normal CDF, $w_{min}(r) = \frac{2r\tan(\alpha)}{R_{px}} \times 1000$
mm, and $\sigma_{noise} = 0.05$ mm models sensor noise and focus uncertainty.

### Coverage Metric

Discretise the wall into a $N_y \times N_z$ grid of cells with cell size $\delta = 0.1$ m.
Cell $(j, k)$ is **covered** if at any timestep $t$ the drone's image footprint contains it:

$$\text{covered}(j, k) = \mathbf{1}\!\left[\exists\, t : \left|y(t) - y_j\right| \leq h_{strip}
  \;\wedge\; \left|z(t) - z_k\right| \leq h_{strip}\right]$$

Coverage fraction (from MATH\_FOUNDATIONS §7):

$$C = \frac{\#\{\text{covered cells}\}}{N_y \cdot N_z}$$

---

## Implementation

```python
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.collections import LineCollection

# Key constants
WALL_WIDTH    = 50.0      # m — y axis
WALL_HEIGHT   = 20.0      # m — z axis
D_STANDOFF    = 1.5       # m — perpendicular to wall (x axis)
V_CRUISE      = 2.0       # m/s — along-wall speed
FOV_DEG       = 60.0      # degrees — total camera FOV
OVERLAP       = 0.20      # strip overlap fraction
CAM_RES_PX    = 2048      # pixels across FOV
SIGMA_NOISE   = 0.05      # mm — crack width detection noise
Z_MIN         = 0.5       # m — minimum safe flight altitude
Z_MAX         = 19.5      # m — maximum altitude
DT            = 0.05      # s — simulation timestep

# Derived geometry
ALPHA         = np.radians(FOV_DEG / 2.0)
H_STRIP       = D_STANDOFF * np.tan(ALPHA)       # ~0.866 m
W_STRIP       = 2.0 * H_STRIP                    # ~1.732 m
DELTA_Z       = W_STRIP * (1.0 - OVERLAP)        # ~1.386 m
N_PASS        = int(np.ceil(WALL_HEIGHT / DELTA_Z))

# PD controller gains
KP_X  = 16.0;  KD_X  = 8.0    # standoff control
KP_Z  = 9.0;   KD_Z  = 6.0    # height control

N_CRACKS = 40   # synthetic crack count for evaluation

def generate_pass_heights():
    z_start = Z_MIN + H_STRIP
    return [z_start + k * DELTA_Z for k in range(N_PASS)]

def build_waypoints(pass_heights):
    """Generate boustrophedon waypoint list [(y, z), ...]."""
    wps = []
    for k, z in enumerate(pass_heights):
        if k % 2 == 0:
            wps += [(0.0, z), (WALL_WIDTH, z)]
        else:
            wps += [(WALL_WIDTH, z), (0.0, z)]
    return wps

def generate_cracks(n, seed=42):
    """Random crack positions and widths on the wall."""
    rng = np.random.default_rng(seed)
    y = rng.uniform(0.5, WALL_WIDTH - 0.5, n)
    z = rng.uniform(0.5, WALL_HEIGHT - 0.5, n)
    w = rng.uniform(0.1, 2.0, n)    # mm
    return np.stack([y, z, w], axis=1)   # (n, 3)

def w_min_at_range(r):
    """Minimum detectable crack width (mm) at slant range r (m)."""
    return (2.0 * r * np.tan(ALPHA) / CAM_RES_PX) * 1000.0

def detection_prob(crack_w_mm, slant_range):
    from scipy.special import ndtr
    w_min = w_min_at_range(slant_range)
    return float(ndtr((crack_w_mm - w_min) / SIGMA_NOISE))

def simulate_scan(pass_heights, cracks):
    """
    Simulate boustrophedon wall scan. Returns trajectory and detection log.
    State: [y, z, vy, vz]
    """
    waypoints = build_waypoints(pass_heights)
    state = np.array([waypoints[0][0], waypoints[0][1], 0.0, 0.0])
    wp_idx = 1
    traj = [state[:2].copy()]
    detected = np.zeros(len(cracks), dtype=bool)
    rng = np.random.default_rng(0)

    while wp_idx < len(waypoints):
        wp = np.array(waypoints[wp_idx])
        to_wp = wp - state[:2]
        dist = np.linalg.norm(to_wp)

        if dist < 0.1:
            wp_idx += 1
            continue

        # Desired velocity: cruise along wall, hold height
        dir_y = np.sign(to_wp[0]) if abs(to_wp[0]) > 0.1 else 0.0
        dir_z = np.sign(to_wp[1]) if abs(to_wp[1]) > 0.01 else 0.0
        v_des = np.array([dir_y * V_CRUISE, dir_z * V_CRUISE])

        # Simple velocity step (kinematic model)
        state[2:] = v_des
        state[:2] += state[2:] * DT
        state[0]  = np.clip(state[0], 0.0, WALL_WIDTH)
        state[1]  = np.clip(state[1], Z_MIN, Z_MAX)
        traj.append(state[:2].copy())

        # Crack detection check
        for ci, crack in enumerate(cracks):
            if detected[ci]:
                continue
            dy = state[0] - crack[0]
            dz = state[1] - crack[1]
            slant = np.sqrt(dy**2 + dz**2 + D_STANDOFF**2)
            in_frame = abs(dy) <= H_STRIP and abs(dz) <= H_STRIP
            if in_frame:
                p = detection_prob(crack[2], slant)
                if rng.random() < p:
                    detected[ci] = True

    return np.array(traj), detected

def coverage_fraction(traj, grid_res=0.1):
    """Compute wall coverage fraction on a fine grid."""
    ny = int(WALL_WIDTH  / grid_res)
    nz = int(WALL_HEIGHT / grid_res)
    covered = np.zeros((ny, nz), dtype=bool)
    for pos in traj:
        j0 = max(0, int((pos[0] - H_STRIP) / grid_res))
        j1 = min(ny, int((pos[0] + H_STRIP) / grid_res) + 1)
        k0 = max(0, int((pos[1] - H_STRIP) / grid_res))
        k1 = min(nz, int((pos[1] + H_STRIP) / grid_res) + 1)
        covered[j0:j1, k0:k1] = True
    return covered.sum() / (ny * nz)

def run_simulation():
    pass_heights = generate_pass_heights()
    cracks = generate_cracks(N_CRACKS)
    traj, detected = simulate_scan(pass_heights, cracks)
    C = coverage_fraction(traj)
    detect_rate = detected.sum() / len(cracks)
    path_len = np.sum(np.linalg.norm(np.diff(traj, axis=0), axis=1))
    return traj, cracks, detected, C, detect_rate, path_len
```

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Wall width $W$ | 50 m |
| Wall height $H$ | 20 m |
| Standoff distance $d_{standoff}$ | 1.5 m |
| Camera FOV | 60° |
| Strip half-width $h_{strip}$ | $1.5 \tan 30° \approx 0.866$ m |
| Strip width $w_{strip}$ | $\approx 1.732$ m |
| Strip overlap $\rho_{ov}$ | 20% |
| Vertical step $\Delta z$ | $\approx 1.386$ m |
| Number of passes $N_{pass}$ | 15 |
| Cruise speed $v_{cruise}$ | 2.0 m/s |
| Camera resolution | 2048 px across FOV |
| Crack width range | 0.1 – 2.0 mm |
| Detection noise $\sigma_{noise}$ | 0.05 mm |
| PD gains $(K_p, K_d)$ standoff | 16, 8 s$^{-2}$, s$^{-1}$ |
| PD gains $(K_p, K_d)$ height | 9, 6 s$^{-2}$, s$^{-1}$ |
| Synthetic crack count $N_c$ | 40 |
| Simulation timestep $\Delta t$ | 0.05 s |
| Coverage grid resolution | 0.1 m |

---

## Expected Output

- **Wall coverage map**: 2D $(y, z)$ heatmap of the $50 \times 20$ m facade; cells visited by at
  least one image footprint shown in green, unvisited cells in white; strip overlap regions shown
  in a lighter green shade; detected cracks marked as red crosses, undetected cracks as grey
  circles scaled by crack width.
- **Boustrophedon trajectory plot**: the drone's $(y, z)$ path overlaid on the wall, with pass
  numbers annotated; turnaround points highlighted; total flight distance printed in the title.
- **PD standoff error time series**: $x(t) - d_{standoff}$ vs time showing convergence to near
  zero; shaded band indicating $\pm 1\sigma$ noise level.
- **Detection probability curve**: $P_{det}(w_i)$ vs crack width (mm) for a fixed slant range
  equal to $d_{standoff}$; vertical dashed line at $w_{min}$; scatter of simulated crack widths
  colour-coded by detected/undetected.
- **Coverage fraction vs pass number**: bar chart showing cumulative $C$ after each completed
  horizontal pass, reaching $\geq 0.99$ by the final pass.
- **Detection rate by width bin**: histogram of $N_c = 40$ cracks binned by width
  (0–0.5, 0.5–1.0, 1.0–1.5, 1.5–2.0 mm); stacked bars showing detected vs missed counts per
  bin.

---

## Extensions

1. **Variable standoff with range sensing**: equip the drone with a laser rangefinder;
   on detecting surface protrusions (scaffolding, window frames) the standoff controller
   increases $d_{standoff}$ dynamically to avoid collision while re-computing $w_{strip}$.
2. **Tilted-wall geometry**: allow the wall to be non-vertical (facade lean angle
   $\phi \in [-5°, +5°]$); update the standoff control to track the wall normal rather than
   the global $x$-axis, and re-derive strip width as a function of $\phi$.
3. **Adaptive path re-planning on live detections**: if a crack wider than $1.5$ mm is
   detected mid-pass, insert a hover waypoint to acquire a high-resolution close-up image
   at $d_{close} = 0.5$ m before resuming the boustrophedon; measure the impact on total
   flight time and missed-crack rate.
4. **Multi-facade inspection**: extend to a full building perimeter (four walls); plan the
   inter-facade transitions to minimise total turnaround distance; formulate as a 2D TSP over
   the four wall start/end positions.
5. **Wind disturbance rejection**: add a horizontal crosswind $\mathbf{v}_{wind}(t)$ with
   gusts modelled as a Dryden turbulence spectrum; compare PD versus an LQR regulator on
   standoff error variance and detection rate degradation.

---

## Related Scenarios

- Prerequisites: [S041 Wildfire Boundary Scan](S041_wildfire_boundary_scan.md), [S048 Lawnmower Coverage](S048_lawnmower_coverage.md)
- Follow-ups: [S061 Power Line Inspection](../04_industrial_agriculture/S061_power_line_inspection.md), [S062 Wind Turbine Blade Inspection](../04_industrial_agriculture/S062_wind_turbine_blade_inspection.md), [S065 Building 3D Scan Path](../04_industrial_agriculture/S065_building_3d_scan_path.md)
- Algorithmic cross-reference: [S053 Coral Reef 3D Mapping](S053_coral_reef_3d_mapping.md) (surface coverage metrics), [S043 Confined Space Exploration](S043_confined_space_exploration.md) (close-proximity flight control)
