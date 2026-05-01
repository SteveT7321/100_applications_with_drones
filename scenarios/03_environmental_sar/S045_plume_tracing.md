# S045 Chemical Plume Tracing

**Domain**: Environmental Monitoring & SAR | **Difficulty**: ⭐⭐⭐ | **Status**: `[x]` Complete

---

## Problem Definition

**Setup**: A gas leak of unknown location has occurred somewhere upwind in a $200 \times 200$ m open
area. A single drone is deployed at a known starting position downwind of the suspected source
region. The drone carries a chemical sensor (e.g. photoionisation detector or metal-oxide gas
sensor) that measures local concentration with additive noise. Wind blows steadily along the
positive $x$-axis at $u = 3$ m/s. The plume disperses according to Gaussian crosswind-diffusion
theory; turbulence causes intermittent concentration dropouts that can trap a naive gradient-climber
in a local minimum.

**Roles**:
- **Drone**: single agent; measures concentration at its current position, estimates the local
  gradient, and executes a cast-and-surge navigation policy to trace the plume back to the source.
- **Source**: fixed point $(x_s, y_s)$ emitting chemical at rate $Q = 1.0$ g/s; position unknown
  to the drone.
- **Particle filter**: onboard estimator that maintains a probability distribution over possible
  source locations given the history of concentration measurements.

**Objective**: Navigate from the initial position to within $r_{capture} = 3$ m of the true source
in minimum time, using only local concentration readings and the known mean wind direction.

**Comparison strategies**:
1. **Pure gradient ascent** — always move in the direction of the estimated concentration gradient;
   fails when the drone exits the detectable plume region.
2. **Cast-and-surge (chemotaxis)** — surge upwind when concentration is above threshold; cast
   crosswind (alternating left/right) when contact is lost; re-engage on re-detection.
3. **Particle filter guided** — use the particle filter posterior mean as an attracting waypoint,
   blended with local gradient cues; the most robust strategy.

---

## Mathematical Model

### Gaussian Plume Dispersion

The time-averaged concentration at position $(x, y)$ due to a continuous ground-level point source
at origin $(0, 0)$ with mean wind speed $u$ along $+x$ is:

$$C(x, y) = \frac{Q}{2\pi \, \sigma_y(x) \, \sigma_z(x) \, u}
             \exp\!\left(-\frac{y^2}{2\sigma_y^2(x)}\right)
             \exp\!\left(-\frac{z_d^2}{2\sigma_z^2(x)}\right)$$

where $Q$ is the emission rate (g/s), $u$ is the mean wind speed (m/s), $z_d$ is the drone flight
altitude (taken as $z_d = 2$ m), and $\sigma_y$, $\sigma_z$ are the lateral and vertical dispersion
coefficients (m). The concentration is zero for $x \leq 0$ (upwind of the source).

In the 2D top-down simulation the vertical term is absorbed into an effective emission rate
$Q_{eff} = Q \exp(-z_d^2 / (2\sigma_z^2)) / \sigma_z$, giving:

$$C(x, y) = \frac{Q_{eff}}{2\pi \, \sigma_y(x) \, u}
             \exp\!\left(-\frac{y^2}{2\sigma_y^2(x)}\right), \qquad x > 0$$

### Pasquill-Gifford Dispersion Coefficients

For Pasquill stability class D (neutral atmosphere) and downwind distance $x$ (m):

$$\sigma_y(x) = 0.08 \, x \,(1 + 0.0002 \, x)^{-1/2}$$

$$\sigma_z(x) = 0.06 \, x \,(1 + 0.0015 \, x)^{-1/2}$$

Both coefficients grow with downwind distance, causing the plume to widen and dilute farther from
the source.

### Turbulent Intermittency Model

Real plumes are intermittent: the instantaneous concentration fluctuates around the time-averaged
value $C(x, y)$. The measured concentration at time $t$ is modelled as:

$$C_{meas}(t) = C(x, y) \cdot \xi(t) + \eta(t)$$

where $\xi(t) \in \{0, 1\}$ is a Bernoulli intermittency flag with detection probability
$p_{detect}(x, y) = 1 - \exp(-C(x,y)/C_{thresh})$, and $\eta(t) \sim \mathcal{N}(0, \sigma_{noise}^2)$
is additive sensor noise with $\sigma_{noise} = 0.05$ (normalised units). The intermittency
parameter $C_{thresh}$ governs how often the drone loses contact with the plume.

### Finite-Difference Gradient Estimation

The drone estimates the local gradient by taking four measurements at positions offset by
$\delta = 1.0$ m along each axis:

$$\hat{\nabla}_x C \approx \frac{C(x + \delta, y) - C(x - \delta, y)}{2\delta}$$

$$\hat{\nabla}_y C \approx \frac{C(x, y + \delta) - C(x, y - \delta)}{2\delta}$$

Each offset measurement is corrupted by independent sensor noise. The estimated gradient direction:

$$\hat{\mathbf{g}} = \frac{(\hat{\nabla}_x C,\; \hat{\nabla}_y C)}
                         {\|(\hat{\nabla}_x C,\; \hat{\nabla}_y C)\| + \epsilon}$$

### Cast-and-Surge Algorithm

The drone maintains a state machine with three modes:

| State | Condition | Action |
|-------|-----------|--------|
| **Surge** | $C_{meas}(t) \geq C_{thresh}$ | Move at speed $v_{surge}$ in direction $\hat{\mathbf{g}}$ biased upwind |
| **Cast** | $C_{meas}(t) < C_{thresh}$ for $\leq T_{cast}$ s | Move crosswind at $\pm v_{cast}$, alternating direction every $L_{cast}$ m |
| **Lost** | $C_{meas}(t) < C_{thresh}$ for $> T_{cast}$ s | Return to last known high-concentration position and restart cast |

The surge velocity command in world frame:

$$\mathbf{v}_{cmd} = v_{surge}\left[\alpha \, \hat{\mathbf{u}}_{wind} + (1 - \alpha)\, \hat{\mathbf{g}}\right]$$

where $\hat{\mathbf{u}}_{wind} = (1, 0)^T$ is the upwind unit vector, $\alpha = 0.4$ is the upwind
bias weight, and $\hat{\mathbf{g}}$ is the normalised gradient estimate.

### Particle Filter Source Localisation

A particle filter with $N_p = 500$ particles maintains a posterior distribution over the source
position $\mathbf{p}_s = (x_s, y_s)$.

**Initialisation**: particles drawn uniformly from the search area.

**Prediction step** (source is stationary): no motion model; particles remain fixed.

**Update step** at drone position $\mathbf{p}_d$ with measurement $C_{meas}$:

For each particle $i$ with hypothesised source at $\mathbf{p}_s^{(i)}$, compute the predicted
concentration $\hat{C}^{(i)} = C(\mathbf{p}_d - \mathbf{p}_s^{(i)})$ (coordinate shift so the
source is at origin). The likelihood:

$$w_i \propto \exp\!\left(-\frac{(C_{meas} - \hat{C}^{(i)})^2}{2\sigma_{noise}^2}\right)$$

**Resampling**: systematic resampling whenever the effective sample size
$N_{eff} = \left(\sum_i w_i^2\right)^{-1} < N_p / 2$.

**Posterior estimate** of source position:

$$\hat{\mathbf{p}}_s = \sum_{i=1}^{N_p} w_i \, \mathbf{p}_s^{(i)}$$

### Drone Kinematics

Point-mass model with first-order speed response:

$$\dot{\mathbf{p}}_d = \mathbf{v}_d$$

$$\dot{\mathbf{v}}_d = \frac{1}{\tau_v}\left(\mathbf{v}_{cmd} - \mathbf{v}_d\right)$$

with time constant $\tau_v = 0.5$ s and maximum speed $v_{max} = 2.0$ m/s.

### Source Capture Criterion

The drone is declared to have found the source when:

$$\|\mathbf{p}_d - \mathbf{p}_s\| \leq r_{capture} = 3.0 \text{ m}$$

---

## Implementation

```python
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Key constants
WIND_SPEED    = 3.0     # m/s, along +x axis
Q_SOURCE      = 1.0     # g/s, emission rate
Z_DRONE       = 2.0     # m, flight altitude
SIGMA_NOISE   = 0.05    # normalised concentration noise
C_THRESH      = 0.02    # detection threshold (normalised)
V_SURGE       = 1.5     # m/s, surge speed
V_CAST        = 0.8     # m/s, cast speed
L_CAST        = 8.0     # m, cast leg length before reversing
T_CAST_MAX    = 30.0    # s, max cast duration before declaring lost
ALPHA_UPWIND  = 0.4     # upwind bias in surge command
GRAD_DELTA    = 1.0     # m, finite-difference offset
N_PARTICLES   = 500     # particle filter population
R_CAPTURE     = 3.0     # m, source-found radius
V_MAX         = 2.0     # m/s
TAU_V         = 0.5     # s, speed response time constant
DT            = 0.1     # s, simulation timestep
T_MAX         = 600.0   # s, mission timeout

# Dispersion coefficients (Pasquill class D)
def sigma_y(x):
    x = max(x, 1e-3)
    return 0.08 * x / np.sqrt(1.0 + 0.0002 * x)

def sigma_z(x):
    x = max(x, 1e-3)
    return 0.06 * x / np.sqrt(1.0 + 0.0015 * x)

def gaussian_concentration(pos_drone, pos_source):
    """2D top-down Gaussian plume concentration at pos_drone from source at pos_source."""
    dx = pos_drone[0] - pos_source[0]   # downwind offset
    dy = pos_drone[1] - pos_source[1]   # crosswind offset
    if dx <= 0:
        return 0.0                       # upwind of source: no plume
    sy = sigma_y(dx)
    sz = sigma_z(dx)
    # Absorb vertical term into effective Q
    q_eff = Q_SOURCE * np.exp(-Z_DRONE**2 / (2.0 * sz**2)) / sz
    return (q_eff / (2.0 * np.pi * sy * WIND_SPEED)) * np.exp(-dy**2 / (2.0 * sy**2))

def measure_concentration(pos_drone, pos_source, rng):
    """Noisy, intermittent concentration reading."""
    c_mean = gaussian_concentration(pos_drone, pos_source)
    # Intermittency: Bernoulli with p proportional to signal strength
    p_detect = 1.0 - np.exp(-c_mean / max(C_THRESH, 1e-6))
    detected = rng.random() < p_detect
    noise = rng.normal(0.0, SIGMA_NOISE)
    return max(0.0, c_mean * detected + noise)

def estimate_gradient(pos_drone, pos_source, rng):
    """Finite-difference gradient estimate with noisy measurements."""
    dx_vec = np.array([GRAD_DELTA, 0.0])
    dy_vec = np.array([0.0, GRAD_DELTA])
    c_xp = measure_concentration(pos_drone + dx_vec, pos_source, rng)
    c_xm = measure_concentration(pos_drone - dx_vec, pos_source, rng)
    c_yp = measure_concentration(pos_drone + dy_vec, pos_source, rng)
    c_ym = measure_concentration(pos_drone - dy_vec, pos_source, rng)
    grad = np.array([(c_xp - c_xm) / (2 * GRAD_DELTA),
                     (c_yp - c_ym) / (2 * GRAD_DELTA)])
    norm = np.linalg.norm(grad) + 1e-9
    return grad / norm

class CastSurgeNavigator:
    """State machine implementing the cast-and-surge chemotaxis strategy."""

    def __init__(self):
        self.state = 'surge'
        self.cast_dir = 1.0           # +1 = +y, -1 = -y
        self.cast_dist_accum = 0.0    # distance accumulated in current cast leg
        self.t_no_contact = 0.0       # time elapsed without plume contact
        self.last_contact_pos = None  # position of last plume detection

    def command(self, pos, concentration, grad_hat, dt):
        """Return velocity command vector given current sensor state."""
        upwind = np.array([1.0, 0.0])

        if concentration >= C_THRESH:
            self.state = 'surge'
            self.t_no_contact = 0.0
            self.cast_dist_accum = 0.0
            self.last_contact_pos = pos.copy()
            v_cmd = V_SURGE * (ALPHA_UPWIND * upwind + (1 - ALPHA_UPWIND) * grad_hat)
            return v_cmd / (np.linalg.norm(v_cmd) + 1e-9) * V_SURGE

        # No contact
        self.t_no_contact += dt

        if self.t_no_contact > T_CAST_MAX and self.last_contact_pos is not None:
            self.state = 'lost'
            toward_last = self.last_contact_pos - pos
            d = np.linalg.norm(toward_last)
            if d < 1.0:
                self.t_no_contact = 0.0
                self.state = 'cast'
            return V_CAST * toward_last / (d + 1e-9)

        self.state = 'cast'
        cast_vec = np.array([0.0, self.cast_dir])
        self.cast_dist_accum += V_CAST * dt
        if self.cast_dist_accum >= L_CAST:
            self.cast_dir *= -1.0
            self.cast_dist_accum = 0.0
        # Slight upwind bias during cast
        v_cmd = V_CAST * cast_vec + 0.2 * upwind
        return v_cmd

class ParticleFilter:
    """Source localisation particle filter."""

    def __init__(self, bounds, n_particles, rng):
        self.n = n_particles
        self.particles = rng.uniform(
            [bounds[0], bounds[2]], [bounds[1], bounds[3]],
            size=(n_particles, 2)
        )
        self.weights = np.ones(n_particles) / n_particles

    def update(self, pos_drone, c_meas):
        for i, ps in enumerate(self.particles):
            c_pred = gaussian_concentration(pos_drone, ps)
            self.weights[i] *= np.exp(-(c_meas - c_pred)**2 / (2 * SIGMA_NOISE**2))
        w_sum = self.weights.sum()
        if w_sum < 1e-300:
            self.weights[:] = 1.0 / self.n
        else:
            self.weights /= w_sum
        # Resample if ESS drops below half
        ess = 1.0 / np.sum(self.weights**2)
        if ess < self.n / 2:
            idx = np.random.choice(self.n, size=self.n, p=self.weights)
            self.particles = self.particles[idx]
            self.weights[:] = 1.0 / self.n

    def estimate(self):
        return np.average(self.particles, weights=self.weights, axis=0)

def run_simulation(strategy='cast_surge', seed=42):
    rng = np.random.default_rng(seed)

    # True source position (unknown to drone)
    pos_source = np.array([140.0, 95.0])
    # Drone starts downwind (low x), random crosswind position
    pos = np.array([20.0, 80.0])
    vel = np.zeros(2)
    bounds = [0, 200, 0, 200]

    navigator = CastSurgeNavigator()
    pf = ParticleFilter(bounds, N_PARTICLES, rng)

    trajectory = [pos.copy()]
    concentrations = []
    pf_estimates = []
    states = []
    t = 0.0
    found = False

    while t < T_MAX:
        c_meas = measure_concentration(pos, pos_source, rng)
        concentrations.append(c_meas)
        pf.update(pos, c_meas)
        pf_est = pf.estimate()
        pf_estimates.append(pf_est.copy())

        if strategy == 'gradient':
            grad_hat = estimate_gradient(pos, pos_source, rng)
            v_cmd = V_SURGE * grad_hat if c_meas >= C_THRESH else np.array([0.2, 0.0])

        elif strategy == 'cast_surge':
            grad_hat = estimate_gradient(pos, pos_source, rng)
            v_cmd = navigator.command(pos, c_meas, grad_hat, DT)
            states.append(navigator.state)

        elif strategy == 'pf_guided':
            grad_hat = estimate_gradient(pos, pos_source, rng)
            toward_est = pf_est - pos
            d_est = np.linalg.norm(toward_est) + 1e-9
            nav_cmd = navigator.command(pos, c_meas, grad_hat, DT)
            v_cmd = 0.5 * nav_cmd + 0.5 * V_SURGE * toward_est / d_est

        # First-order dynamics
        vel += (v_cmd - vel) / TAU_V * DT
        speed = np.linalg.norm(vel)
        if speed > V_MAX:
            vel = vel / speed * V_MAX

        pos = pos + vel * DT
        pos = np.clip(pos, [bounds[0], bounds[2]], [bounds[1], bounds[3]])
        trajectory.append(pos.copy())
        t += DT

        if np.linalg.norm(pos - pos_source) <= R_CAPTURE:
            found = True
            break

    return {
        'trajectory': np.array(trajectory),
        'concentrations': np.array(concentrations),
        'pf_estimates': np.array(pf_estimates),
        'pos_source': pos_source,
        'found': found,
        'time': t,
        'states': states,
    }
```

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Area | 200 × 200 m |
| Source emission rate $Q$ | 1.0 g/s |
| Wind speed $u$ | 3.0 m/s (along $+x$) |
| Drone flight altitude $z_d$ | 2.0 m |
| Sensor noise $\sigma_{noise}$ | 0.05 (normalised) |
| Detection threshold $C_{thresh}$ | 0.02 (normalised) |
| Surge speed $v_{surge}$ | 1.5 m/s |
| Cast speed $v_{cast}$ | 0.8 m/s |
| Cast leg length $L_{cast}$ | 8.0 m |
| Max cast duration $T_{cast}$ | 30.0 s |
| Upwind bias $\alpha$ | 0.4 |
| Gradient probe offset $\delta$ | 1.0 m |
| Particle count $N_p$ | 500 |
| Capture radius $r_{capture}$ | 3.0 m |
| Max drone speed $v_{max}$ | 2.0 m/s |
| Speed time constant $\tau_v$ | 0.5 s |
| Dispersion model | Pasquill-Gifford class D |
| $\sigma_y$ formula | $0.08 x (1 + 0.0002 x)^{-1/2}$ |
| $\sigma_z$ formula | $0.06 x (1 + 0.0015 x)^{-1/2}$ |
| Mission timeout $T_{max}$ | 600 s |
| Simulation timestep $\Delta t$ | 0.1 s |

---

## Expected Output

- **Plume concentration map**: 2D top-down heatmap of $C(x, y)$ with the true source marked (red
  star), wind direction arrow, and $\sigma_y$ plume boundary contours overlaid.
- **Trajectory comparison plot**: drone paths for all three strategies (gradient ascent in orange,
  cast-and-surge in blue, particle-filter guided in green) overlaid on the concentration map;
  source capture marked with a circle; lost/cast/surge segments colour-coded for cast-and-surge.
- **Concentration time series**: measured $C_{meas}(t)$ vs time for each strategy, with the
  detection threshold $C_{thresh}$ shown as a dashed line and intermittency dropout events
  highlighted.
- **Particle filter posterior evolution**: four snapshots of the particle cloud (at $t =$ 0, 30,
  90, 180 s) overlaid on the map, showing convergence of the posterior toward the true source.
- **State machine timeline** (cast-and-surge only): colour bar showing which state (surge / cast /
  lost) the drone occupied at each timestep.
- **Performance summary bar chart**: time-to-find and path length for each strategy across
  $N_{trials} = 20$ Monte Carlo runs; box-and-whisker showing median and spread; failure rate
  (timeout without capture) annotated.
- **Animation (GIF)**: drone moving across the concentration field in real time; particle cloud
  updating each frame; plume boundary contour and wind arrow shown; state label displayed.

---

## Extensions

1. **3D plume tracing**: extend the dispersion model to full 3D (latitude, longitude, altitude);
   drone varies altitude to exploit vertical concentration gradients and determine stack height.
2. **Moving source**: the gas leak migrates slowly (e.g. ruptured pipeline with propagating
   crack); add a source motion model to the particle filter prediction step and evaluate tracking
   lag.
3. **Multi-drone cooperative search**: deploy $N = 3$ drones with different starting crosswind
   positions; fuse their particle filter posteriors via consensus averaging to localise the source
   faster (see S049 Dynamic Zone Assignment for multi-agent coordination).
4. **Realistic wind field**: replace uniform mean wind with a spatially varying field generated by
   a 2D potential-flow solver or a pre-recorded meteorological dataset; evaluate robustness of
   cast-and-surge under wind shear.
5. **RL navigation policy**: train a PPO agent with state $(C_{meas}, \hat{\nabla}C, \hat{\mathbf{p}}_s,
   \mathbf{v}_d)$ and continuous action $\mathbf{v}_{cmd}$; compare data efficiency and
   generalisation to novel wind directions against the cast-and-surge heuristic.
6. **Sensor fusion**: combine chemical sensor readings with thermal infrared (warm gas leak) and
   acoustic (hissing pipe) modalities; update a joint likelihood in the particle filter.

---

## Related Scenarios

- Prerequisites: [S041 Wildfire Boundary Scan](S041_wildfire_boundary_scan.md), [S042 Missing Person Search](S042_missing_person_search.md)
- Follow-ups: [S055 Oil Spill Tracking](S055_oil_spill_tracking.md) (advecting scalar field), [S056 Radiation Hotspot Detection](S056_radiation_hotspot_detection.md) (similar gradient-ascent source-finding structure)
- Algorithmic cross-reference: [S048 Lawnmower Coverage](S048_lawnmower_coverage.md) (systematic coverage as fallback), [S049 Dynamic Zone Assignment](S049_dynamic_zone_assignment.md) (multi-agent extension), [S046 3D Trilateration](S046_3d_trilateration.md) (3D source localisation)
