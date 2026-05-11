# S071 Bridge Underside Structural Inspection

**Domain**: Industrial & Agriculture | **Difficulty**: ⭐⭐⭐ | **Status**: `[ ]` Not started

---

## Problem Definition

**Setup**: A bridge deck spans 50 m in the x-direction and 10 m in the y-direction at a fixed
height of z = 8 m. The drone must fly parallel-strip passes 0.5 m below the deck (z = 7.5 m),
scanning the entire underside for simulated surface cracks. GPS is unavailable beneath the bridge
structure; height is estimated from a barometric altimeter, and horizontal distance to the deck is
measured by an upward-facing rangefinder. Cracks are modelled as a binary anomaly mask generated
from a procedural surface texture.

**Objective**: Cover 100 % of the bridge underside in boustrophedon strips while keeping the
drone below the safety ceiling z_max = 7.8 m (0.2 m margin below deck), detecting the maximum
fraction of seeded cracks, and recording each detected crack's (x, y) position for a maintenance
report. Report coverage fraction, crack detection rate, and total safety-violation count (target
zero).

---

## Mathematical Model

### Coordinate Frame

The bridge deck occupies the plane z = z_deck = 8.0 m over the region
x ∈ [0, 50] m, y ∈ [0, 10] m. The drone flies at nominal height z_nom = 7.5 m.
Drone state in 3-D:

$$\mathbf{q}(t) = \bigl(x(t),\; y(t),\; z(t),\; \dot{x}(t),\; \dot{y}(t),\; \dot{z}(t)\bigr)^{\top}$$

The upward clearance from the drone to the deck is:

$$d_{up}(t) = z_{deck} - z(t)$$

Target clearance: d_target = 0.5 m. Safety constraint: d_up(t) ≥ 0.2 m at all times.

### Upward-Facing Height (Inverse Surface-Following) Controller

Height error:

$$e_z(t) = z_{deck} - d_{target} - z(t)$$

PD control law for the vertical thrust component:

$$u_z(t) = K_p \, e_z(t) + K_d \, \dot{e}_z(t)$$

Critically-damped gains with natural frequency ω_n = 4 rad/s:

$$K_p = \omega_n^2 = 16 \; \text{s}^{-2}, \qquad K_d = 2\,\omega_n = 8 \; \text{s}^{-1}$$

### Rangefinder Measurement Model

The upward rangefinder returns a noisy measurement of d_up:

$$d_{meas}(t) = d_{up}(t) + \eta(t), \qquad \eta(t) \sim \mathcal{N}\!\bigl(0,\, \sigma_{range}^2\bigr)$$

with σ_range = 0.02 m. The controller uses d_meas in place of d_up:

$$e_z(t) = \bigl(z_{deck} - d_{target}\bigr) - \bigl(z_{deck} - d_{meas}(t)\bigr) = d_{meas}(t) - d_{target}$$

### Safety Violation Condition

A safety violation is recorded at each timestep when:

$$z(t) > z_{max}, \qquad z_{max} = z_{deck} - 0.2 = 7.8 \; \text{m}$$

On violation, an emergency descent command overrides the controller:

$$u_z^{emg} = -K_{emg} \bigl(z(t) - z_{max}\bigr), \qquad K_{emg} = 30 \; \text{s}^{-2}$$

### Strip Coverage Geometry

Camera FOV half-angle α = 45°. At inspection distance d_target = 0.5 m, the upward-facing
sensor footprint half-width on the deck:

$$h_{strip} = d_{target} \cdot \tan(\alpha) = 0.5 \cdot \tan(45°) = 0.5 \; \text{m}$$

Full strip width: w_strip = 2 h_strip = 1.0 m.

With overlap fraction ρ_ov = 0.10, the lateral step between adjacent along-x passes:

$$\Delta y = w_{strip} \cdot (1 - \rho_{ov}) = 1.0 \times 0.9 = 0.9 \; \text{m}$$

Number of along-x passes required to cover the full bridge width W_y = 10 m:

$$N_{pass} = \left\lceil \frac{W_y}{\Delta y} \right\rceil = \left\lceil \frac{10}{0.9} \right\rceil = 12$$

Pass centre y-coordinates:

$$y_k = \frac{\Delta y}{2} + (k-1)\,\Delta y, \qquad k = 1, \ldots, N_{pass}$$

### Boustrophedon (Lawnmower) Waypoints

Pass k is flown in the x-direction; direction alternates:

$$x_k^{start} = \begin{cases} 0 & k \text{ odd} \\ L & k \text{ even} \end{cases}, \qquad x_k^{end} = \begin{cases} L & k \text{ odd} \\ 0 & k \text{ even} \end{cases}$$

with L = 50 m (bridge length). Total planned path length:

$$L_{total} = N_{pass} \cdot L + (N_{pass} - 1) \cdot \Delta y$$

### Crack Detection Model

Cracks are seeded as circular anomaly patches of radius r_crack drawn from
U(0.05, 0.20) m at random deck positions (x_i, y_i). Detection probability
at each timestep when crack i is within the sensor footprint:

$$P_{det}(i) = 0.90 \quad \text{(constant, independent per timestep)}$$

Crack i is marked detected on the first successful Bernoulli trial while it is
within the footprint. The footprint inclusion condition is:

$$|x(t) - x_i| \leq h_{strip} \;\wedge\; |y(t) - y_i| \leq h_{strip}$$

### Coverage Metric

Discretise the deck surface into an N_x × N_y grid with cell size δ = 0.1 m.
Cell (j, k) is covered if:

$$\text{covered}(j,k) = \mathbf{1}\!\left[\exists\,t : |x(t) - x_j| \leq h_{strip} \;\wedge\; |y(t) - y_k| \leq h_{strip}\right]$$

Overall coverage fraction:

$$C = \frac{\#\{\text{covered cells}\}}{N_x \cdot N_y}$$

---

## Implementation

```python
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

# ── Domain constants ────────────────────────────────────────────────────────
BRIDGE_LEN    = 50.0    # m  x-axis
BRIDGE_WID    = 10.0    # m  y-axis
Z_DECK        = 8.0     # m  deck height
D_TARGET      = 0.5     # m  inspection standoff below deck
Z_NOM         = Z_DECK - D_TARGET          # 7.5 m nominal flight height
Z_MAX         = Z_DECK - 0.2              # 7.8 m safety ceiling
SIGMA_RANGE   = 0.02    # m  rangefinder noise std
V_CRUISE      = 1.0     # m/s
FOV_HALF_DEG  = 45.0    # degrees
OVERLAP       = 0.10
P_DETECT      = 0.90
N_CRACKS      = 30
DT            = 0.05    # s
KP, KD        = 16.0, 8.0
K_EMG         = 30.0

# ── Derived geometry ────────────────────────────────────────────────────────
ALPHA     = np.radians(FOV_HALF_DEG)
H_STRIP   = D_TARGET * np.tan(ALPHA)          # 0.5 m
W_STRIP   = 2.0 * H_STRIP                     # 1.0 m
DELTA_Y   = W_STRIP * (1.0 - OVERLAP)         # 0.9 m
N_PASS    = int(np.ceil(BRIDGE_WID / DELTA_Y)) # 12

def build_waypoints():
    """Boustrophedon waypoints [(x, y, z), ...] at nominal height."""
    wps = []
    for k in range(N_PASS):
        yc = DELTA_Y / 2.0 + k * DELTA_Y
        if k % 2 == 0:
            wps += [(0.0, yc, Z_NOM), (BRIDGE_LEN, yc, Z_NOM)]
        else:
            wps += [(BRIDGE_LEN, yc, Z_NOM), (0.0, yc, Z_NOM)]
    return wps

def seed_cracks(n, seed=7):
    rng = np.random.default_rng(seed)
    x  = rng.uniform(1.0, BRIDGE_LEN - 1.0, n)
    y  = rng.uniform(0.5, BRIDGE_WID  - 0.5, n)
    r  = rng.uniform(0.05, 0.20, n)
    return np.stack([x, y, r], axis=1)   # (n, 3)

def simulate(waypoints, cracks):
    state  = np.array([*waypoints[0], 0.0, 0.0, 0.0])  # x,y,z,vx,vy,vz
    wp_idx = 1
    traj, violations = [state[:3].copy()], 0
    detected   = np.zeros(len(cracks), dtype=bool)
    rng        = np.random.default_rng(42)

    while wp_idx < len(waypoints):
        wp  = np.array(waypoints[wp_idx])
        err = wp[:2] - state[:2]                  # xy error only
        if np.linalg.norm(err) < 0.1:
            wp_idx += 1; continue

        # Horizontal velocity towards waypoint
        state[3:5] = (err / np.linalg.norm(err)) * V_CRUISE

        # Vertical PD with noisy rangefinder
        d_meas  = (Z_DECK - state[2]) + rng.normal(0, SIGMA_RANGE)
        ez      = d_meas - D_TARGET
        dez     = -state[5]
        uz      = KP * ez + KD * dez
        if state[2] > Z_MAX:            # safety override
            uz = -K_EMG * (state[2] - Z_MAX)
            violations += 1
        state[5] = uz * DT
        state[:3] += state[3:] * DT

        traj.append(state[:3].copy())

        # Crack detection
        for ci, ck in enumerate(cracks):
            if detected[ci]: continue
            in_fp = abs(state[0]-ck[0]) <= H_STRIP and abs(state[1]-ck[1]) <= H_STRIP
            if in_fp and rng.random() < P_DETECT:
                detected[ci] = True

    return np.array(traj), detected, violations

def coverage_fraction(traj, res=0.1):
    nx = int(BRIDGE_LEN / res)
    ny = int(BRIDGE_WID  / res)
    grid = np.zeros((nx, ny), dtype=bool)
    for pos in traj:
        j0 = max(0, int((pos[0]-H_STRIP)/res))
        j1 = min(nx, int((pos[0]+H_STRIP)/res)+1)
        k0 = max(0, int((pos[1]-H_STRIP)/res))
        k1 = min(ny, int((pos[1]+H_STRIP)/res)+1)
        grid[j0:j1, k0:k1] = True
    return grid.sum() / (nx * ny), grid
```

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Bridge length (x) | 50 m |
| Bridge width (y) | 10 m |
| Deck height z_deck | 8.0 m |
| Nominal flight height z_nom | 7.5 m |
| Inspection standoff d_target | 0.5 m |
| Safety ceiling z_max | 7.8 m (0.2 m margin) |
| Rangefinder noise std | 0.02 m |
| Camera FOV half-angle | 45 deg |
| Strip half-width h_strip | 0.5 m |
| Strip width w_strip | 1.0 m |
| Strip overlap | 10 % |
| Lateral step Delta_y | 0.9 m |
| Number of passes N_pass | 12 |
| Cruise speed | 1.0 m/s |
| PD gains (Kp, Kd) height | 16, 8 s^-2, s^-1 |
| Emergency descent gain K_emg | 30 s^-2 |
| Crack detection probability | 0.90 |
| Synthetic crack count | 30 |
| GPS error (horizontal) | 0.1 m (barometer only, z) |
| Simulation timestep | 0.05 s |
| Coverage grid resolution | 0.1 m |

---

## Expected Output

- **3-D flight path plot**: Matplotlib 3D axes showing the boustrophedon passes at z ≈ 7.5 m
  rendered as a red line beneath the translucent grey bridge deck plane; detected cracks marked as
  green stars on the deck, undetected cracks as grey circles; strip boundaries shown as faint blue
  lines on the deck underside.
- **Coverage heatmap**: top-down 2-D map of the 50 × 10 m deck; cells visited at least once
  shown in green, unvisited in white; detected crack positions overlaid as red crosses; coverage
  fraction C printed in the title.
- **Height time series**: z(t) vs time with three reference lines — z_nom = 7.5 m (blue dashed),
  z_max = 7.8 m (red dashed), z_deck = 8.0 m (grey solid); shaded envelope ± σ_range around
  z_nom; any safety violation timesteps highlighted in red.
- **Strip-by-strip coverage bar chart**: cumulative coverage fraction after each completed pass
  (12 bars), showing the incremental contribution of each strip.
- **Animation** (FuncAnimation): top-down or 3-D view of the drone moving through its lawnmower
  path, with the coverage grid filling in green in real time and crack-detection events marked as
  flashes.
- **Summary metrics table** printed to console: coverage % (target ≥ 99 %), crack detection rate
  (target ≥ 90 %), safety violations (target = 0), total flight time (s), total path length (m).

---

## Extensions

1. **GPS-denied lateral drift**: add a random walk to the horizontal position (σ_drift =
   0.05 m/√s) simulating the absence of GPS; implement a visual odometry correction using
   detected crack positions as landmarks to bound drift below GPS_ERROR = 0.1 m.
2. **Non-planar deck with camber**: replace the flat deck with a slightly arched surface
   z_deck(x) = 8.0 + 0.1 sin(πx/L); the height controller must adapt to the slowly varying
   ceiling height while maintaining d_target = 0.5 m throughout.
3. **Variable-width crack detection threshold**: model crack detection as a function of crack
   radius (larger cracks detected at lower p) using the same probabilistic model as S044; plot
   detection rate by crack-size bin.
4. **Multi-drone cooperative coverage**: deploy two drones side by side, each responsible for
   half the bridge width (y ∈ [0, 5] and y ∈ [5, 10]); coordinate strip assignments to avoid
   overlap conflicts and halve total inspection time (→ S066 pattern).
5. **Structural anomaly localisation**: replace the binary detection flag with a position
   estimate (centroid of contiguous anomaly pixels); evaluate localisation accuracy vs ground
   truth and report mean position error across all detected cracks.

---

## Related Scenarios

- Prerequisites: [S044 Wall Crack Inspection](../03_environmental_sar/S044_wall_crack.md), [S061 Power Line Inspection](S061_power_line.md), [S065 Building 3D Scan Path](S065_3d_scan_path.md)
- Next: [S072 Oil Pipeline Leak Detection](S072_pipeline_leak.md), [S074 Mine 3D Mapping](S074_mine_mapping.md)
- Algorithmic cross-reference: [S067 Spray Coverage Overlap Optimisation](S067_spray_overlap.md) (strip planning), [S064 Greenhouse Interior Flight](S064_greenhouse.md) (GPS-denied close-proximity control)
