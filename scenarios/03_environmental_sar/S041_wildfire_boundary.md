# S041 Wildfire Boundary Scan

**Domain**: Environmental Monitoring & SAR | **Difficulty**: ⭐⭐ | **Status**: `[x]` Complete

---

## Problem Definition

**Setup**: An active wildfire is burning inside a $200 \times 200$ m operational area. The fire
front is modelled as a closed boundary that expands stochastically over time following an elliptical
spread model driven by a constant ambient wind. A fleet of $N = 3$ drones carrying downward-facing
IR sensors flies at a fixed scan altitude of $z = 10$ m. At $t = 0$ the fire perimeter is a circle
of radius $r_0 = 20$ m centred at the ignition point $\mathbf{c}_{ign}$. The fire boundary is not
directly observable; each drone only detects whether its footprint is above a burning or unburned
cell via a noisy IR temperature reading.

**Roles**:
- **Fire** (environment): a stochastically expanding region whose boundary is the unknown quantity
  to be tracked.
- **Drones** ($N = 3$): identical agents, each with IR sensor radius $r_s = 2.0$ m; they maintain
  a shared occupancy grid and collectively track the moving boundary by following its gradient.

**Objective**: Keep the estimated fire boundary error small at all times — minimising the mean
Hausdorff distance between the true perimeter and the fleet's current boundary estimate — while
maintaining continuous perimeter coverage so that no boundary segment is left unscanned for more
than $\Delta t_{gap} = 30$ s.

**Key question**: How does boundary-tracking error grow as a function of fire spread rate? At what
wind speed does the gradient-following strategy fail to maintain contact with the advancing front?

---

## Mathematical Model

### Fire Spread Model (Elliptical Huygens)

The fire is described by a scalar field $F(\mathbf{x}, t) \in \{0, 1\}$: cell $\mathbf{x}$ is
burning if $F = 1$. Fire spread follows the elliptical Rothermel model. At each timestep $\Delta t$,
the fire front advances outward with a local rate-of-spread that depends on the angle $\theta$
between the spread direction and the prevailing wind:

$$R(\theta) = R_0 \cdot \frac{(1 + e_w)^2}{1 + e_w \cos\theta}$$

where $R_0$ is the head-fire rate of spread (m/s), $e_w = (R_{head}/R_{back} - 1)/2$ is the
eccentricity parameter, and $\theta$ is measured from the wind direction. Stochastic spotting is
added by sampling a Gaussian perturbation on each boundary cell's advance:

$$\Delta r_i = R(\theta_i) \cdot \Delta t + \epsilon_i, \qquad \epsilon_i \sim \mathcal{N}(0,\,\sigma_{spot}^2)$$

with spotting noise $\sigma_{spot} = 0.05$ m per step. The burning region at time $t + \Delta t$ is
updated by expanding all boundary cells by $\Delta r_i$ in the outward normal direction.

### Occupancy Grid and Boundary Extraction

The arena is discretised into a grid of cells with resolution $\delta = 0.5$ m. Each cell
$(i, j)$ holds a binary burn state $F_{ij}(t)$ and a drone-observed flag. The estimated boundary
$\hat{\partial \mathcal{F}}(t)$ is the set of cells that are burning and adjacent to at least one
unburned cell:

$$\hat{\partial \mathcal{F}} = \bigl\{(i,j) : F_{ij} = 1 \text{ and } \exists\,(i',j') \in \mathcal{N}(i,j) \text{ s.t. } F_{i'j'} = 0\bigr\}$$

where $\mathcal{N}(i,j)$ denotes the 4-connected neighbourhood.

### IR Sensor Model

Each drone at position $\mathbf{p}_k$ observes a disc of radius $r_s$ centred on its ground
projection. The noisy temperature reading for cell $(i,j)$ within the footprint is:

$$z_{ij} = T_{true}(\mathbf{x}_{ij}) + w_{ij}, \qquad w_{ij} \sim \mathcal{N}(0,\, R_{sensor})$$

with $T_{true} = T_{fire} = 800$ K if $F_{ij} = 1$, else $T_{ambient} = 300$ K, and
$R_{sensor} = 25$ K$^2$. A cell is classified as burning if the reading exceeds the threshold
$T_{thresh} = 550$ K:

$$\hat{F}_{ij} = \mathbf{1}\!\left[z_{ij} \geq T_{thresh}\right]$$

### Boundary Gradient-Following Guidance

Each drone maintains a local boundary segment to patrol. The guidance law drives the drone along
the boundary in a direction that keeps the fire edge within one sensor radius. Define the signed
boundary proximity function:

$$\phi_k(t) = d_k^{fire}(t) - r_s$$

where $d_k^{fire}$ is the drone's distance to the nearest detected burning cell. The commanded
velocity for drone $k$ is a superposition of a boundary-tangent component and a stand-off
correction:

$$\dot{\mathbf{p}}_k = v_{cruise} \cdot \hat{\mathbf{t}}_k + K_\phi \cdot \phi_k(t) \cdot \hat{\mathbf{n}}_k$$

where $\hat{\mathbf{t}}_k$ is the unit tangent to the boundary at the nearest point (directed to
maintain clockwise patrol ordering), $\hat{\mathbf{n}}_k$ is the unit inward normal, $K_\phi = 0.8$
s$^{-1}$ is the stand-off gain, and $v_{cruise} = 2.5$ m/s.

### Replanning Trigger

A replanning event is triggered for drone $k$ when either condition holds:

1. **Lost contact**: $d_k^{fire} > r_s + d_{replan}$ with $d_{replan} = 3.0$ m — the drone has
   drifted too far from the boundary.
2. **Coverage gap**: a boundary segment has not been overflown for $\Delta t_{gap} = 30$ s,
   detected by maintaining a per-cell timestamp grid $T_{last\_scan}(i,j)$.

On replanning, a new target point on the boundary is assigned by selecting the boundary cell with
the oldest timestamp:

$$\mathbf{p}_{k}^{target} = \arg\max_{(i,j) \in \hat{\partial \mathcal{F}}} \bigl(t - T_{last\_scan}(i,j)\bigr)$$

The drone then navigates directly to $\mathbf{p}_{k}^{target}$ and resumes gradient-following.

### Performance Metrics

**Mean Hausdorff distance** between true boundary $\partial \mathcal{F}(t)$ and tracked estimate
$\hat{\partial \mathcal{F}}(t)$:

$$d_H(t) = \frac{1}{2}\left[
  \max_{\mathbf{x} \in \partial \mathcal{F}} \min_{\hat{\mathbf{x}} \in \hat{\partial \mathcal{F}}} \|\mathbf{x} - \hat{\mathbf{x}}\|
  + \max_{\hat{\mathbf{x}} \in \hat{\partial \mathcal{F}}} \min_{\mathbf{x} \in \partial \mathcal{F}} \|\mathbf{x} - \hat{\mathbf{x}}\|
\right]$$

**Coverage freshness**: fraction of boundary cells scanned within the last $\Delta t_{gap}$:

$$\eta(t) = \frac{\bigl|\{(i,j) \in \hat{\partial \mathcal{F}} : t - T_{last\_scan}(i,j) \leq \Delta t_{gap}\}\bigr|}{|\hat{\partial \mathcal{F}}|}$$

---

## Implementation

```python
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Key constants
N_DRONES        = 3
ARENA_SIZE      = 200.0       # m — square arena side length
GRID_RES        = 0.5         # m — occupancy grid cell size
SCAN_HEIGHT     = 10.0        # m — drone altitude (fixed)
SENSOR_RADIUS   = 2.0         # m — IR footprint radius on ground
R_SENSOR_VAR    = 25.0        # K^2 — IR noise variance
T_FIRE          = 800.0       # K — temperature of burning cells
T_AMBIENT       = 300.0       # K — background temperature
T_THRESH        = 550.0       # K — burn/no-burn classification threshold
V_CRUISE        = 2.5         # m/s — drone cruise speed
K_PHI           = 0.8         # s^-1 — stand-off correction gain
D_REPLAN        = 3.0         # m — lost-contact replanning distance
DT_GAP          = 30.0        # s — max allowed scan gap per boundary cell
R0_SPREAD       = 0.04        # m/s — head-fire rate of spread
ECC             = 0.6         # ellipse eccentricity parameter e_w
WIND_DIR_DEG    = 45.0        # degrees — prevailing wind direction
SIGMA_SPOT      = 0.05        # m — spotting noise std per timestep
IGN_CENTRE      = np.array([100.0, 100.0])   # m — ignition point
R_INIT          = 20.0        # m — initial fire radius
DT              = 0.5         # s — simulation timestep
T_SIM           = 300.0       # s — total simulation duration

# Derived grid dimensions
N_CELLS = int(ARENA_SIZE / GRID_RES)

def init_grid():
    """Initialise binary burn grid: circle of radius R_INIT centred at IGN_CENTRE."""
    xs = np.arange(N_CELLS) * GRID_RES + GRID_RES / 2
    ys = np.arange(N_CELLS) * GRID_RES + GRID_RES / 2
    X, Y = np.meshgrid(xs, ys, indexing='ij')
    dist = np.hypot(X - IGN_CENTRE[0], Y - IGN_CENTRE[1])
    return (dist <= R_INIT).astype(np.float32)

def spread_fire(F, dt, wind_dir_rad, r0, ecc, sigma_spot):
    """Expand fire by one timestep using elliptical Huygens model."""
    from scipy.ndimage import binary_dilation
    boundary_mask = np.zeros_like(F, dtype=bool)
    # Find boundary cells (burning, adjacent to unburned)
    padded = np.pad(F, 1, constant_values=0)
    for di, dj in [(-1,0),(1,0),(0,-1),(0,1)]:
        rolled = padded[1+di:N_CELLS+1+di, 1+dj:N_CELLS+1+dj]
        boundary_mask |= ((F == 1) & (rolled == 0))

    cells_i, cells_j = np.where(boundary_mask)
    for ci, cj in zip(cells_i, cells_j):
        # Outward normal direction (away from centroid of burning region)
        cx = np.mean(np.where(F == 1)[0]) * GRID_RES
        cy = np.mean(np.where(F == 1)[1]) * GRID_RES
        nx = ci * GRID_RES - cx
        ny = cj * GRID_RES - cy
        norm = np.hypot(nx, ny) + 1e-8
        nx /= norm; ny /= norm
        theta = np.arctan2(ny, nx) - wind_dir_rad
        R_theta = r0 * (1 + ecc)**2 / (1 + ecc * np.cos(theta))
        advance = R_theta * dt + np.random.normal(0, sigma_spot)
        advance = max(0, advance)
        # Advance fire along outward normal
        new_i = int(round(ci + advance * nx / GRID_RES))
        new_j = int(round(cj + advance * ny / GRID_RES))
        if 0 <= new_i < N_CELLS and 0 <= new_j < N_CELLS:
            F[new_i, new_j] = 1.0
    return F

def ir_observation(F, drone_pos):
    """Return noisy IR observations for all cells within sensor footprint."""
    xs = np.arange(N_CELLS) * GRID_RES + GRID_RES / 2
    ys = np.arange(N_CELLS) * GRID_RES + GRID_RES / 2
    X, Y = np.meshgrid(xs, ys, indexing='ij')
    dist_to_drone = np.hypot(X - drone_pos[0], Y - drone_pos[1])
    footprint = dist_to_drone <= SENSOR_RADIUS
    T_true = np.where(F == 1, T_FIRE, T_AMBIENT)
    noise = np.random.normal(0, np.sqrt(R_SENSOR_VAR), F.shape)
    z = T_true + noise
    return footprint, (z >= T_THRESH).astype(np.float32)

def get_boundary_cells(F):
    """Return (i, j) indices of all boundary cells."""
    cells = []
    for ci in range(1, N_CELLS - 1):
        for cj in range(1, N_CELLS - 1):
            if F[ci, cj] == 1:
                if any(F[ci+di, cj+dj] == 0
                       for di, dj in [(-1,0),(1,0),(0,-1),(0,1)]):
                    cells.append((ci, cj))
    return cells

def boundary_gradient_step(pos, F_est, last_scan, t_now):
    """Compute commanded velocity for one drone using gradient-following law."""
    boundary = get_boundary_cells(F_est)
    if not boundary:
        return np.zeros(2), False

    bx = np.array([c[0] * GRID_RES for c in boundary])
    by = np.array([c[1] * GRID_RES for c in boundary])
    dists = np.hypot(bx - pos[0], by - pos[1])
    nearest_idx = np.argmin(dists)
    d_fire = dists[nearest_idx]

    # Stand-off correction (inward normal when too far, outward when too close)
    phi = d_fire - SENSOR_RADIUS
    nearest_pt = np.array([bx[nearest_idx], by[nearest_idx]])
    n_hat = (pos - nearest_pt) / (d_fire + 1e-8)   # inward normal

    # Boundary tangent (perpendicular to inward normal, clockwise)
    t_hat = np.array([-n_hat[1], n_hat[0]])

    vel = V_CRUISE * t_hat - K_PHI * phi * n_hat

    # Clip to cruise speed
    speed = np.linalg.norm(vel)
    if speed > V_CRUISE:
        vel = vel * V_CRUISE / speed

    # Check replanning trigger
    replan = (d_fire > SENSOR_RADIUS + D_REPLAN)
    return vel, replan

def select_replan_target(F_est, last_scan, t_now):
    """Find boundary cell with the oldest scan timestamp."""
    boundary = get_boundary_cells(F_est)
    if not boundary:
        return None
    oldest_age = -1
    target = boundary[0]
    for ci, cj in boundary:
        age = t_now - last_scan[ci, cj]
        if age > oldest_age:
            oldest_age = age
            target = (ci, cj)
    return np.array([target[0] * GRID_RES, target[1] * GRID_RES])

def hausdorff_distance(true_boundary, est_boundary):
    """Symmetric mean Hausdorff distance between two boundary cell lists."""
    if not true_boundary or not est_boundary:
        return 0.0
    tb = np.array([[c[0], c[1]] for c in true_boundary], dtype=float) * GRID_RES
    eb = np.array([[c[0], c[1]] for c in est_boundary], dtype=float) * GRID_RES
    from scipy.spatial.distance import cdist
    D = cdist(tb, eb)
    d1 = np.mean(np.min(D, axis=1))
    d2 = np.mean(np.min(D, axis=0))
    return 0.5 * (d1 + d2)

def run_simulation():
    wind_rad = np.deg2rad(WIND_DIR_DEG)
    F_true = init_grid()
    F_est  = F_true.copy()

    # Per-cell last-scan timestamp (initialised to -inf)
    last_scan = np.full((N_CELLS, N_CELLS), -np.inf)

    # Initialise drones evenly spaced around initial fire perimeter
    angles = np.linspace(0, 2 * np.pi, N_DRONES, endpoint=False)
    drone_pos = np.array([
        IGN_CENTRE + (R_INIT + SENSOR_RADIUS) * np.array([np.cos(a), np.sin(a)])
        for a in angles
    ])
    replan_targets = [None] * N_DRONES

    t_steps = int(T_SIM / DT)
    hausdorff_history = []
    coverage_freshness = []
    drone_trajectories = [[] for _ in range(N_DRONES)]

    for step in range(t_steps):
        t = step * DT

        # 1. Spread fire
        F_true = spread_fire(F_true, DT, wind_rad, R0_SPREAD, ECC, SIGMA_SPOT)

        # 2. Each drone observes and updates shared estimate
        for k in range(N_DRONES):
            footprint, obs = ir_observation(F_true, drone_pos[k])
            # Update estimated map: trust observations within footprint
            F_est[footprint] = obs[footprint]
            ci_arr, cj_arr = np.where(footprint)
            for ci, cj in zip(ci_arr, cj_arr):
                last_scan[ci, cj] = t

        # 3. Move each drone
        for k in range(N_DRONES):
            if replan_targets[k] is not None:
                # Navigate toward replan target
                diff = replan_targets[k] - drone_pos[k]
                dist = np.linalg.norm(diff)
                if dist < V_CRUISE * DT:
                    drone_pos[k] = replan_targets[k].copy()
                    replan_targets[k] = None
                else:
                    drone_pos[k] += V_CRUISE * diff / dist * DT
            else:
                vel, need_replan = boundary_gradient_step(
                    drone_pos[k], F_est, last_scan, t
                )
                drone_pos[k] += vel * DT
                if need_replan:
                    replan_targets[k] = select_replan_target(F_est, last_scan, t)

            # Gap coverage trigger (any boundary cell stale)
            boundary = get_boundary_cells(F_est)
            if boundary:
                ages = np.array([t - last_scan[ci, cj] for ci, cj in boundary])
                if np.any(ages > DT_GAP) and replan_targets[k] is None:
                    replan_targets[k] = select_replan_target(F_est, last_scan, t)

            drone_pos[k] = np.clip(drone_pos[k], 0, ARENA_SIZE)
            drone_trajectories[k].append(drone_pos[k].copy())

        # 4. Compute metrics
        true_bnd = get_boundary_cells(F_true)
        est_bnd  = get_boundary_cells(F_est)
        hd = hausdorff_distance(true_bnd, est_bnd)
        hausdorff_history.append(hd)

        if est_bnd:
            fresh = np.mean([
                float(t - last_scan[ci, cj] <= DT_GAP)
                for ci, cj in est_bnd
            ])
        else:
            fresh = 1.0
        coverage_freshness.append(fresh)

    return {
        "F_true": F_true,
        "F_est":  F_est,
        "hausdorff": np.array(hausdorff_history),
        "freshness": np.array(coverage_freshness),
        "drone_trajectories": drone_trajectories,
        "last_scan": last_scan,
        "t_axis": np.arange(t_steps) * DT,
    }
```

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Arena size | $200 \times 200$ m |
| Grid resolution $\delta$ | 0.5 m |
| Number of drones $N$ | 3 |
| Scan altitude $z$ | 10.0 m |
| IR sensor radius $r_s$ | 2.0 m |
| IR noise variance $R_{sensor}$ | 25 K$^2$ |
| Classification threshold $T_{thresh}$ | 550 K |
| Drone cruise speed $v_{cruise}$ | 2.5 m/s |
| Stand-off gain $K_\phi$ | 0.8 s$^{-1}$ |
| Head-fire spread rate $R_0$ | 0.04 m/s |
| Ellipse eccentricity $e_w$ | 0.6 |
| Wind direction | 45 deg (NE) |
| Spotting noise $\sigma_{spot}$ | 0.05 m/step |
| Initial fire radius $r_0$ | 20 m |
| Ignition point | (100, 100) m |
| Lost-contact replan distance $d_{replan}$ | 3.0 m |
| Max coverage gap $\Delta t_{gap}$ | 30 s |
| Simulation timestep $\Delta t$ | 0.5 s |
| Simulation duration $T_{sim}$ | 300 s |

---

## Expected Output

- **Fire evolution map**: 2D top-down plot with true burn region (orange fill), estimated boundary
  (red contour), drone positions (blue markers), and sensor footprint circles overlaid; shown at
  $t = 0$, $t = 100$, $t = 200$, $t = 300$ s.
- **Hausdorff distance plot**: $d_H(t)$ vs time showing boundary tracking error over the full
  simulation; annotated with replanning events for each drone.
- **Coverage freshness plot**: $\eta(t)$ vs time (fraction of boundary cells scanned within
  $\Delta t_{gap}$); horizontal dashed line at the 90% target threshold.
- **Drone trajectory overlay**: full 2D paths of all three drones colour-coded by time (blue-early
  to red-late), superimposed on the final burn footprint.
- **Replanning event log**: scatter markers on the trajectory plot indicating where each drone
  triggered a lost-contact or gap-coverage replan.
- **Animation (GIF)**: real-time top-down view of fire expanding outward (orange fill growing),
  three drones orbiting the boundary with their IR footprints visible, and boundary estimate
  updating cell-by-cell as new observations arrive.

---

## Extensions

1. **Wind shift event**: introduce a sudden 90-degree wind direction change at $t = 150$ s; measure
   how quickly the fleet detects the new dominant fire front and reassigns drones to cover the
   accelerated head-fire segment. This tests replanning latency under non-stationary spread dynamics.
2. **Variable fleet size sweep**: run the scenario with $N \in \{1, 2, 3, 4, 5\}$ drones and plot
   mean Hausdorff error and coverage freshness vs $N$; identify the minimum fleet size that keeps
   $d_H < 5$ m and $\eta > 90\%$ simultaneously throughout the mission.
3. **Probabilistic boundary representation**: replace the binary occupancy grid with a per-cell
   burn probability $p_{ij}(t) \in [0, 1]$ updated via a Kalman-style likelihood update using the
   IR sensor model; track the 0.5-isoprobability contour as the boundary estimate and compare
   Hausdorff error against the binary-threshold baseline.

---

## Related Scenarios

- Prerequisites: [S048 Full-Area Coverage Scan](S048_lawnmower.md) (basic coverage planning before dynamic tracking)
- Follow-ups: [S049 Dynamic Zone Search](S049_dynamic_zone.md) (multi-drone zoning with reallocation), [S055 Coastline Oil Spill Tracking](S055_oil_spill.md) (dynamic contamination boundary, analogous structure)
- Algorithmic cross-reference: [S045 Chemical Plume Tracing](S045_plume_tracing.md) (gradient-ascent boundary following), [S042 Missing Person Localization](S042_missing_person.md) (Bayesian sensor update pattern)
