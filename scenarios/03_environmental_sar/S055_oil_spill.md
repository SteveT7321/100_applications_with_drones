# S055 Coastline Oil Spill Tracking

**Domain**: Environmental Monitoring & SAR | **Difficulty**: ⭐⭐⭐ | **Status**: `[ ]` Not Started

---

## Problem Definition

**Setup**: An oil tanker accident has released a large surface slick in a $4 \times 4$ km coastal
ocean area. The slick evolves continuously under the combined influence of a steady wind drift and
a tidal current. Two drones are deployed from a shore base and must keep a real-time estimate of
the spill boundary accurate enough to guide containment boom deployment. The spill is modelled as
a 2D concentration field $C(x, y, t)$ on the ocean surface; the boundary is the iso-contour at a
fixed contamination threshold $C_{thr}$.

Each drone carries a downward-facing **hyperspectral or fluorescence sensor** that returns a
binary above/below-threshold reading at its current position. The drones fly at constant altitude
$z = 15$ m and constant speed $v = 8$ m/s. They circulate along the estimated boundary, taking
samples every $\Delta s = 20$ m, and feed each observation into a **Kalman filter** that tracks
the displacement of individual boundary landmark points. A **predictive replanning** step
periodically advances the nominal boundary forward in time using the known advection velocity,
enabling the drones to stay ahead of the spill rather than always chasing its last observed state.

**Roles**:
- **Drone A** and **Drone B**: symmetric boundary-patrol agents; each owns one half of the
  boundary arc (split at the two points of maximum curvature). They share boundary-state estimates
  via a common ground-station uplink.
- **Spill field**: dynamic scalar field governed by advection-diffusion; truth known only to the
  simulator, sampled by the drones at their flight positions.
- **Ground station**: fuses both drones' observations, runs the Kalman filter update, and
  broadcasts the updated boundary estimate back to both drones at each replanning epoch
  $T_{replan} = 30$ s.

**Objective**: Maintain **boundary coverage** $\eta(t) \geq 0.85$ at all times, where $\eta$ is
the fraction of boundary arc length whose most recent observation is fresher than $T_{stale} = 60$
s, while the spill expands under advection-diffusion dynamics over a $T_{mission} = 30$ min
mission.

**Comparison strategies**:
1. **Static partition, no prediction** — each drone patrols its fixed initial semicircle at
   constant angular speed; boundary estimate never replanned.
2. **Adaptive partition, no prediction** — partitions are rebalanced at each epoch to equalise
   per-drone uncovered arc length; no forward prediction.
3. **Adaptive partition + predictive replanning** — partitions rebalanced and boundary estimate
   propagated forward by one epoch using advection velocity before each replanning step.

---

## Mathematical Model

### Advection-Diffusion Spill Dynamics

The oil concentration field $C(\mathbf{x}, t)$ with $\mathbf{x} = (x, y)$ satisfies the 2D
advection-diffusion equation:

$$\frac{\partial C}{\partial t} = -\mathbf{u} \cdot \nabla C + D \nabla^2 C$$

where $\mathbf{u} = (u_x, u_y)$ is the combined wind-drift and tidal-current velocity vector
(m/s) and $D$ (m$^2$/s) is the effective horizontal diffusivity. In the simulation the field
is advanced in time on a $200 \times 200$ grid using a finite-difference scheme.

**Initial condition**: a circular patch of radius $R_0 = 200$ m centred at $\mathbf{x}_0$:

$$C(\mathbf{x}, 0) = C_0 \exp\!\left(-\frac{\|\mathbf{x} - \mathbf{x}_0\|^2}{2 R_0^2}\right)$$

**Finite-difference update** (forward Euler, timestep $\delta t$):

$$C_{i,j}^{n+1} = C_{i,j}^n
  - \frac{u_x \delta t}{2h}\!\left(C_{i+1,j}^n - C_{i-1,j}^n\right)
  - \frac{u_y \delta t}{2h}\!\left(C_{i,j+1}^n - C_{i,j-1}^n\right)
  + \frac{D \delta t}{h^2}\!\left(C_{i+1,j}^n + C_{i-1,j}^n + C_{i,j+1}^n + C_{i,j-1}^n - 4C_{i,j}^n\right)$$

where $h$ is the grid cell size (m) and indices $(i, j)$ correspond to the $(x, y)$ directions.

### Spill Boundary Representation

The boundary $\partial \Omega(t)$ is the level-set contour:

$$\partial \Omega(t) = \{\mathbf{x} : C(\mathbf{x}, t) = C_{thr}\}$$

For the Kalman filter the boundary is parametrised as $N_b = 36$ landmark points evenly spaced
in arc length: $\mathbf{b}_k(t) \in \mathbb{R}^2$, $k = 0, \ldots, N_b - 1$.

### Landmark Motion Model

Each boundary landmark drifts with the local advection velocity:

$$\dot{\mathbf{b}}_k = \mathbf{u} + \boldsymbol{\xi}_k, \qquad
\boldsymbol{\xi}_k \sim \mathcal{N}(\mathbf{0},\, \sigma_q^2 \mathbf{I})$$

The process noise $\sigma_q$ captures diffusion-induced boundary deformation not explained by
pure advection. In discrete time with epoch $T_{replan}$:

$$\mathbf{b}_k^{n+1} = \mathbf{b}_k^n + \mathbf{u}\, T_{replan} + \boldsymbol{\xi}_k$$

### Kalman Filter Boundary Estimation

Each landmark $k$ has state $\hat{\mathbf{b}}_k \in \mathbb{R}^2$ and covariance
$\mathbf{P}_k \in \mathbb{R}^{2 \times 2}$.

**Predict** (propagate between replan epochs):

$$\hat{\mathbf{b}}_k^{-} = \hat{\mathbf{b}}_k + \mathbf{u}\, T_{replan}$$

$$\mathbf{P}_k^{-} = \mathbf{P}_k + \sigma_q^2 T_{replan} \mathbf{I}$$

**Update** when a drone samples at position $\mathbf{p}_d$ near landmark $k$ (nearest-landmark
association, acceptance radius $r_{assoc} = 30$ m):

The measurement is a projected position reading. A drone crossing the estimated boundary at
measured position $\mathbf{m}$ provides a 1D innovation along the boundary normal
$\hat{\mathbf{n}}_k$:

$$z_k = \hat{\mathbf{n}}_k^T (\mathbf{m} - \hat{\mathbf{b}}_k^{-})$$

$$S_k = \hat{\mathbf{n}}_k^T \mathbf{P}_k^{-} \hat{\mathbf{n}}_k + \sigma_r^2$$

$$\mathbf{K}_k = \mathbf{P}_k^{-} \hat{\mathbf{n}}_k\, S_k^{-1}$$

$$\hat{\mathbf{b}}_k = \hat{\mathbf{b}}_k^{-} + \mathbf{K}_k z_k$$

$$\mathbf{P}_k = (\mathbf{I} - \mathbf{K}_k \hat{\mathbf{n}}_k^T)\, \mathbf{P}_k^{-}$$

where $\sigma_r$ is the sensor position noise (GPS + sensor footprint combined).

### Drone Boundary-Following Trajectory

Each drone follows the estimated boundary at constant arc-length speed $v_{arc}$. The commanded
velocity at drone position $\mathbf{p}_d$ assigned to boundary segment $[k, k+1]$ is:

$$\mathbf{v}_{cmd} = v_{arc}\, \hat{\boldsymbol{\tau}}_k + k_{lat}\!\left(\mathbf{b}_k^{proj} - \mathbf{p}_d\right)$$

where $\hat{\boldsymbol{\tau}}_k$ is the unit tangent of the boundary arc at the nearest landmark,
$\mathbf{b}_k^{proj}$ is the orthogonal projection of $\mathbf{p}_d$ onto the estimated boundary
segment, and $k_{lat}$ is a lateral correction gain that drives the drone back onto the boundary.

### Predictive Boundary Propagation

Before each replanning step the estimated boundary is propagated forward by one epoch duration
$T_{replan}$ using the known advection velocity:

$$\tilde{\mathbf{b}}_k = \hat{\mathbf{b}}_k + \mathbf{u}\, T_{replan}, \qquad k = 0, \ldots, N_b - 1$$

Drone waypoints for the next epoch are computed from $\{\tilde{\mathbf{b}}_k\}$ rather than
$\{\hat{\mathbf{b}}_k\}$, so the drones intercept the boundary at its predicted future location.

### Partition Rebalancing

Let $L_A$ and $L_B$ be the arc lengths currently assigned to drone A and drone B. The split
point index $k^*$ is chosen to minimise the imbalance in uncovered arc length:

$$k^* = \arg\min_{k} \left|\sum_{i=0}^{k} \ell_i - \frac{L_{total}}{2}\right|$$

where $\ell_i = \|\mathbf{b}_{i+1} - \mathbf{b}_i\|$ is the $i$-th landmark segment length and
$L_{total} = \sum_i \ell_i$ is the total estimated perimeter.

### Coverage Metric

The staleness age of boundary landmark $k$ at time $t$ is:

$$\tau_k(t) = t - t_k^{last}$$

where $t_k^{last}$ is the last time a drone sampled within $r_{assoc}$ of $\mathbf{b}_k$.
Boundary coverage is:

$$\eta(t) = \frac{\sum_{k=0}^{N_b - 1} \ell_k \cdot \mathbf{1}[\tau_k(t) \leq T_{stale}]}
                 {\sum_{k=0}^{N_b - 1} \ell_k}$$

---

## Implementation

```python
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.ndimage import gaussian_filter

# Key constants — domain and grid
AREA_SIZE     = 4000.0       # m, square domain side length
GRID_N        = 200          # grid cells per axis
H_CELL        = AREA_SIZE / GRID_N   # m, cell size

# Spill parameters
C0            = 1.0          # initial peak concentration (normalised)
R0            = 200.0        # m, initial spill radius
X0            = np.array([2000.0, 2000.0])   # m, spill centre

# Advection-diffusion
U_WIND        = np.array([0.4, 0.15])   # m/s, combined wind+current drift
D_DIFF        = 5.0          # m^2/s, horizontal diffusivity
DT_FIELD      = 2.0          # s, PDE timestep (CFL must be satisfied)

# Boundary tracking
C_THR         = 0.08         # contamination threshold (normalised)
N_LANDMARKS   = 36           # boundary landmark points
SIGMA_Q       = 0.5          # m/s^0.5, process noise std
SIGMA_R       = 15.0         # m, measurement noise std
R_ASSOC       = 30.0         # m, landmark association radius

# Drone parameters
V_ARC         = 8.0          # m/s, boundary-following speed
K_LAT         = 0.5          # lateral correction gain (1/s)
DELTA_S       = 20.0         # m, sampling interval along boundary
T_REPLAN      = 30.0         # s, replanning epoch
T_STALE       = 60.0         # s, max observation age for "fresh"
T_MISSION     = 1800.0       # s, total mission duration
DT_SIM        = 0.5          # s, drone simulation timestep

def init_concentration_field(grid_x, grid_y):
    """Gaussian initial spill patch."""
    dist2 = (grid_x - X0[0])**2 + (grid_y - X0[1])**2
    return C0 * np.exp(-dist2 / (2.0 * R0**2))

def advance_field(C, u, D, h, dt):
    """One forward-Euler advection-diffusion step on a 2D field."""
    # Central-difference advection
    adv_x = u[0] * (np.roll(C, -1, axis=0) - np.roll(C, 1, axis=0)) / (2.0 * h)
    adv_y = u[1] * (np.roll(C, -1, axis=1) - np.roll(C, 1, axis=1)) / (2.0 * h)
    # Five-point Laplacian diffusion
    lap   = (np.roll(C, -1, axis=0) + np.roll(C, 1, axis=0)
           + np.roll(C, -1, axis=1) + np.roll(C, 1, axis=1) - 4.0 * C) / h**2
    return C - dt * (adv_x + adv_y) + dt * D * lap

def extract_boundary_landmarks(C, grid_x, grid_y, n_landmarks, c_thr):
    """Extract n_landmarks points evenly spaced in angle around the C=c_thr contour."""
    from matplotlib.contour import QuadContourSet
    fig_tmp, ax_tmp = plt.subplots()
    cs = ax_tmp.contour(grid_x, grid_y, C, levels=[c_thr])
    paths = cs.collections[0].get_paths() if cs.collections else []
    plt.close(fig_tmp)
    if not paths:
        return None
    # Take the longest contour path
    verts = max(paths, key=lambda p: len(p.vertices)).vertices
    # Resample to n_landmarks equally arc-length spaced points
    diffs  = np.diff(verts, axis=0)
    seg_l  = np.linalg.norm(diffs, axis=1)
    cumlen = np.concatenate([[0.0], np.cumsum(seg_l)])
    total  = cumlen[-1]
    sample_d = np.linspace(0, total, n_landmarks, endpoint=False)
    landmarks = np.array([
        np.interp(d, cumlen, verts[:, col]) for col in range(2)
    ]).T
    return landmarks

class BoundaryKalmanFilter:
    """Independent Kalman filter for each boundary landmark (x, y)."""

    def __init__(self, landmarks_init, sigma_q, sigma_r):
        self.n   = len(landmarks_init)
        self.mu  = landmarks_init.copy()          # (N, 2)
        self.P   = np.tile(sigma_r**2 * np.eye(2), (self.n, 1, 1))  # (N, 2, 2)
        self.sq  = sigma_q
        self.sr  = sigma_r

    def predict(self, u_vec, dt):
        self.mu += u_vec * dt
        self.P  += self.sq**2 * dt * np.eye(2)[None]

    def update(self, k, m_pos, n_hat):
        """Scalar innovation update along boundary normal n_hat for landmark k."""
        innov  = n_hat @ (m_pos - self.mu[k])
        S      = n_hat @ self.P[k] @ n_hat + self.sr**2
        K      = self.P[k] @ n_hat / S           # (2,) Kalman gain
        self.mu[k]  += K * innov
        self.P[k]   -= np.outer(K, n_hat) @ self.P[k]

    def boundary_normals(self):
        """Unit outward normals at each landmark (finite-difference tangent rotated 90 deg)."""
        prev_idx = (np.arange(self.n) - 1) % self.n
        next_idx = (np.arange(self.n) + 1) % self.n
        tangent  = self.mu[next_idx] - self.mu[prev_idx]   # (N, 2)
        norms    = np.linalg.norm(tangent, axis=1, keepdims=True) + 1e-9
        tangent  /= norms
        # Rotate 90 deg CCW: (tx, ty) -> (-ty, tx)
        normals  = np.stack([-tangent[:, 1], tangent[:, 0]], axis=1)
        return normals

class BoundaryDrone:
    """Single drone that patrols an arc of the estimated boundary."""

    def __init__(self, drone_id, start_pos, landmark_indices, kf):
        self.id          = drone_id
        self.pos         = start_pos.copy()
        self.indices     = landmark_indices   # owned landmark range
        self.kf          = kf
        self.arc_ptr     = 0                  # index into self.indices
        self.dist_accum  = 0.0               # distance since last sample

    def step(self, dt, t_now, last_obs_times):
        """Advance drone along boundary, record samples, update KF."""
        idx  = self.indices[self.arc_ptr % len(self.indices)]
        tgt  = self.kf.mu[idx]
        diff = tgt - self.pos
        dist = np.linalg.norm(diff)
        if dist < 5.0:
            self.arc_ptr += 1
        else:
            lateral_err = diff / (dist + 1e-9)
            v_cmd = V_ARC * lateral_err
            self.pos = self.pos + v_cmd * dt

        # Sample when enough arc length accumulated
        self.dist_accum += V_ARC * dt
        if self.dist_accum >= DELTA_S:
            self.dist_accum = 0.0
            nearest_k = min(self.indices,
                            key=lambda k: np.linalg.norm(self.kf.mu[k] - self.pos))
            if np.linalg.norm(self.kf.mu[nearest_k] - self.pos) < R_ASSOC:
                n_hat = self.kf.boundary_normals()[nearest_k]
                self.kf.update(nearest_k, self.pos, n_hat)
                last_obs_times[nearest_k] = t_now

def compute_coverage(kf, last_obs_times, t_now, t_stale):
    """Arc-length-weighted fraction of boundary with fresh observations."""
    next_idx = (np.arange(kf.n) + 1) % kf.n
    seg_len  = np.linalg.norm(kf.mu[next_idx] - kf.mu, axis=1)
    fresh    = (t_now - last_obs_times) <= t_stale
    return np.sum(seg_len * fresh) / (np.sum(seg_len) + 1e-9)

def split_boundary(kf, n_landmarks):
    """Rebalance the arc partition at the midpoint index by total arc length."""
    next_idx = (np.arange(n_landmarks) + 1) % n_landmarks
    seg_len  = np.linalg.norm(kf.mu[next_idx] - kf.mu, axis=1)
    cumlen   = np.cumsum(seg_len)
    half     = cumlen[-1] / 2.0
    split_k  = int(np.searchsorted(cumlen, half))
    idx_A    = list(range(0, split_k))
    idx_B    = list(range(split_k, n_landmarks))
    return idx_A, idx_B
```

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Domain area | 4000 × 4000 m |
| Grid resolution | 200 × 200 cells (20 m/cell) |
| Initial spill radius $R_0$ | 200 m |
| Initial peak concentration $C_0$ | 1.0 (normalised) |
| Contamination threshold $C_{thr}$ | 0.08 |
| Advection velocity $\mathbf{u}$ | $(0.40,\; 0.15)$ m/s |
| Diffusivity $D$ | 5.0 m$^2$/s |
| PDE timestep $\delta t$ | 2.0 s |
| Number of boundary landmarks $N_b$ | 36 |
| Process noise $\sigma_q$ | 0.5 m/s$^{1/2}$ |
| Measurement noise $\sigma_r$ | 15.0 m |
| Landmark association radius $r_{assoc}$ | 30 m |
| Number of drones | 2 |
| Drone speed $v_{arc}$ | 8.0 m/s |
| Sampling interval $\Delta s$ | 20 m |
| Lateral correction gain $k_{lat}$ | 0.5 s$^{-1}$ |
| Replanning epoch $T_{replan}$ | 30 s |
| Staleness threshold $T_{stale}$ | 60 s |
| Mission duration $T_{mission}$ | 1800 s (30 min) |
| Simulation timestep $\Delta t_{sim}$ | 0.5 s |

---

## Expected Output

- **Spill evolution snapshots**: four top-down concentration field heatmaps at $t =$ 0, 600, 1200,
  1800 s; estimated boundary contour (dashed white) overlaid on truth contour (solid red);
  Kalman filter landmark positions shown as coloured dots sized by their covariance trace.
- **Drone trajectory overlay**: full flight paths of drone A (orange) and drone B (cyan) plotted
  on the final-frame concentration map; sample points marked; partition split boundary indicated.
- **Boundary coverage time series**: $\eta(t)$ for all three strategies plotted together;
  target threshold $\eta = 0.85$ shown as a horizontal dashed line; replanning epoch ticks
  on the x-axis.
- **Boundary estimation error**: mean positional error $\bar{e}(t) = \frac{1}{N_b}\sum_k
  \|\hat{\mathbf{b}}_k - \mathbf{b}_k^{truth}\|$ vs time for each strategy; shaded band
  showing $\pm 1$ standard deviation across landmark ensemble.
- **Kalman covariance trace evolution**: $\frac{1}{N_b}\sum_k \mathrm{tr}(\mathbf{P}_k)$ vs
  time showing how uncertainty grows during flight gaps and contracts on observation updates.
- **Perimeter growth curve**: estimated total spill perimeter $L_{total}(t)$ vs time compared
  against the truth contour perimeter; illustrates that predictive replanning tracks the growth
  more accurately.
- **Strategy comparison bar chart**: mean boundary coverage $\bar{\eta}$, mean estimation error
  $\bar{e}$, and total uncovered arc-time (staleness violations in km·s) for all three strategies
  over $N_{trials} = 10$ Monte Carlo runs with randomised wind directions.
- **Animation (GIF)**: advancing concentration field with truth boundary (red) and estimated
  boundary (white dashed); both drone icons moving along boundary; coverage fraction displayed
  as a live text overlay; replanning event flashes indicated.

---

## Extensions

1. **Irregular coastline constraint**: add a no-fly zone representing the shoreline; drones must
   plan boundary-following paths that avoid land using a visibility-graph or RRT* layer on top of
   the boundary-following controller.
2. **Time-varying current**: replace the constant drift $\mathbf{u}$ with a tidal-cycle velocity
   field $\mathbf{u}(t) = U_0 \cos(2\pi t / T_{tide})\,\hat{\mathbf{e}}$; update the Kalman
   prediction model with the known tidal phase to maintain accurate boundary forecasts.
3. **Three-drone relay with battery swap**: introduce a battery constraint ($E_{max} = 900$ s
   flight time); a third drone waits at the shore base and is dispatched to relieve the drone
   with lowest remaining battery, maintaining two active boundary-patrollers at all times.
4. **Multi-source spill**: initialise two separate patches that merge as they diffuse; the
   boundary becomes non-convex; test whether the landmark parametrisation and Kalman filter
   remain stable through a topology change and add a landmark resampling step if needed.
5. **Level-set boundary representation**: replace the landmark Kalman filter with a full level-set
   PDE propagation model; the boundary $\phi(\mathbf{x}, t) = 0$ is advected as a signed-distance
   function and corrected by drone observations via a data-assimilation update.
6. **RL dispatch policy**: train a PPO agent to decide, at each replanning epoch, how to split the
   boundary between the two drones; state includes the current coverage map, staleness ages, and
   drone positions; reward is $\eta(t)$ minus a fuel-cost penalty.

---

## Related Scenarios

- Prerequisites: [S041 Wildfire Boundary Scan](S041_wildfire_boundary.md), [S048 Lawnmower Coverage](S048_lawnmower.md)
- Follow-ups: [S056 Radiation Hotspot Detection](S056_radiation.md) (adaptive sampling of a scalar field), [S058 Typhoon Eye Penetration](S058_typhoon.md) (extreme-environment boundary tracking)
- Algorithmic cross-reference: [S045 Chemical Plume Tracing](S045_plume_tracing.md) (advecting scalar field, same PDE structure), [S049 Dynamic Zone Assignment](S049_dynamic_zone.md) (partition rebalancing between agents), [S052 Glacier Area Monitoring](S052_glacier.md) (slowly evolving boundary)
