# S063 Single-Plant Precision Spraying

**Domain**: Industrial & Agriculture | **Difficulty**: ⭐ | **Status**: `[ ]` Not Started
**Algorithm**: TSP Nearest-Neighbour + PID Hover | **Dimension**: 2D

---

## Problem Definition

**Setup**: A single drone must apply a fixed spray dose to each of $N = 20$ target plants randomly
distributed in a $20 \times 20$ m agricultural field. The drone carries a downward-facing nozzle
with an effective spray width of $w_{spray} = 2.0$ m. Before the mission starts, the drone plans
a **nearest-neighbour TSP path** through all plant positions to minimise total transit distance.
At each plant the drone hovers for $T_{spray} = 3$ s while a PID position-hold controller
compensates for a stochastic wind disturbance. Spray effectiveness degrades with hover error:
a Gaussian dose model converts RMS position error into an effective coverage fraction.

**Roles**:
- **Drone**: single UAV executing the pre-planned TSP path; position control at 50 Hz via PID.
- **Plants**: $N = 20$ static targets at uniformly random positions in the $[1, 19] \times [1, 19]$ m
  interior (1 m margin from field boundary).
- **Wind disturbance**: sinusoidal + Gaussian noise acting on both axes, modelling real-world gusts.

**Objective**: Given the planned TSP route, simulate the full spray mission and report:

1. **Total path length** $L_{TSP}$ (m) — transit distance between all plant stops.
2. **Per-plant hover RMS error** $\varepsilon_i$ (m) — position accuracy during the $T_{spray}$
   window at plant $i$.
3. **Effective coverage fraction** $C_{eff} \in [0, 1]$ — fleet-level dose delivery quality
   averaged over all plants.
4. The trade-off between wind severity $\sigma_w$ and achievable $C_{eff}$ for three PID gain sets.

**Comparison configurations** (PID gain sets):

1. **Loose** — $K_p = 0.5$, $K_i = 0.05$, $K_d = 0.1$ (under-damped, high steady-state error).
2. **Tuned** — $K_p = 2.0$, $K_i = 0.2$, $K_d = 0.5$ (critically damped, good disturbance rejection).
3. **Tight** — $K_p = 5.0$, $K_i = 0.5$, $K_d = 1.0$ (aggressive, risk of actuator saturation).

---

## Mathematical Model

### TSP Nearest-Neighbour Path

Given plant positions $\{\mathbf{p}_i\}_{i=1}^{N}$ with $\mathbf{p}_i = (x_i, y_i) \in \mathbb{R}^2$,
the nearest-neighbour heuristic builds the tour greedily. Starting from the depot at the field
corner $(0, 0)$, at each step visit the closest unvisited plant:

$$\pi(k+1) = \arg\min_{j \notin \mathcal{V}} \|\mathbf{p}_{\pi(k)} - \mathbf{p}_j\|_2$$

where $\mathcal{V}$ is the set of already-visited plant indices. The total TSP path length is:

$$L_{TSP} = \sum_{k=0}^{N-1} \|\mathbf{p}_{\pi(k+1)} - \mathbf{p}_{\pi(k)}\|_2$$

This greedy heuristic typically yields a tour within 20–25% of the true optimum for random
uniform point sets in 2D, and runs in $O(N^2)$ time — acceptable for $N = 20$.

### PID Position Hold

The drone state during hover is the 2D horizontal position $\mathbf{q}(t) = (q_x(t), q_y(t))$
and velocity $\dot{\mathbf{q}}(t)$. The position error relative to target plant $\mathbf{p}_i$ is:

$$\mathbf{e}(t) = \mathbf{p}_i - \mathbf{q}(t)$$

The PID control acceleration command (applied independently per axis) is:

$$\mathbf{a}_{PID}(t) = K_p \, \mathbf{e}(t) + K_i \int_0^t \mathbf{e}(\tau)\, d\tau + K_d \, \dot{\mathbf{e}}(t)$$

The integrator is reset to zero at the start of each new plant's hover window to prevent
integral wind-up from transit phases affecting spray accuracy.

### Wind Disturbance Model

The wind force per unit mass (acceleration disturbance) applied to both axes independently is a
sum of a sinusoidal gust and Gaussian turbulence:

$$f_w(t) = A_w \sin\!\left(\frac{2\pi t}{T_w} + \varphi_{rand}\right) + \mathcal{N}(0,\, \sigma_w^2)$$

where:

- $A_w = 0.3$ m/s² — gust amplitude
- $T_w = 4.0$ s — gust period
- $\varphi_{rand} \sim \mathcal{U}(0, 2\pi)$ — random phase, resampled at each plant
- $\sigma_w \in [0.0, 0.5]$ m/s² — turbulence intensity (swept for sensitivity analysis)

The equations of motion for the drone are integrated with the Euler method at timestep $\Delta t$:

$$\dot{\mathbf{q}}(t + \Delta t) = \dot{\mathbf{q}}(t) + \bigl[\mathbf{a}_{PID}(t) + \mathbf{f}_w(t)\bigr] \Delta t$$
$$\mathbf{q}(t + \Delta t) = \mathbf{q}(t) + \dot{\mathbf{q}}(t)\, \Delta t$$

### Hover RMS Error

For plant $i$, the hover window spans $T_{spray}$ seconds sampled at $\Delta t$ intervals, giving
$M = \lfloor T_{spray} / \Delta t \rfloor$ position samples. The per-plant RMS error is:

$$\varepsilon_i = \sqrt{\frac{1}{M} \sum_{k=1}^{M} \|\mathbf{e}_k\|^2}$$

where $\mathbf{e}_k = \mathbf{p}_i - \mathbf{q}(t_k)$ is the position error at sample $k$.

### Effective Coverage Fraction

The spray dose effectiveness follows a Gaussian footprint model. For a nominal dose $D_0$, the
effective dose delivered to plant $i$ is:

$$D_i = D_0 \exp\!\left(-\frac{\varepsilon_i^2}{2\,\sigma_{spray}^2}\right)$$

where $\sigma_{spray} = 0.5$ m is the spray accuracy scale (half the spray width). The
fleet-level **effective coverage fraction** is:

$$C_{eff} = \frac{1}{N} \sum_{i=1}^{N} \exp\!\left(-\frac{\varepsilon_i^2}{2\,\sigma_{spray}^2}\right)$$

$C_{eff} = 1.0$ means perfect dose delivery at every plant; $C_{eff} < 1.0$ reflects dose loss
due to hover error. At $\varepsilon_i = \sigma_{spray} = 0.5$ m the dose drops to $e^{-0.5} \approx 0.61$.

---

## Implementation

```python
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation

# ── Domain constants ──────────────────────────────────────────────────────────
FIELD_SIZE    = 20.0    # m — square field side length
N_PLANTS      = 20      # number of target plants
SPRAY_WIDTH   = 2.0     # m — nozzle effective spray diameter
GPS_ERROR     = 0.1     # m — GPS noise std dev (position sensor)
INSPECTION_SPEED = 1.0  # m/s — transit speed between plants
T_SPRAY       = 3.0     # s — hover duration at each plant
DT            = 0.02    # s — simulation timestep (50 Hz)
SIGMA_SPRAY   = SPRAY_WIDTH / 4.0  # m — dose falloff scale (0.5 m)

# ── Wind disturbance parameters ────────────────────────────────────────────────
A_WIND        = 0.3     # m/s² — sinusoidal gust amplitude
T_WIND        = 4.0     # s — gust period
SIGMA_W_BASE  = 0.1     # m/s² — baseline turbulence intensity

# ── PID gain configurations ────────────────────────────────────────────────────
PID_CONFIGS = {
    "Loose":  dict(kp=0.5, ki=0.05, kd=0.1),
    "Tuned":  dict(kp=2.0, ki=0.2,  kd=0.5),
    "Tight":  dict(kp=5.0, ki=0.5,  kd=1.0),
}

# ── TSP nearest-neighbour planner ─────────────────────────────────────────────
def tsp_nearest_neighbour(plants, depot=np.zeros(2)):
    """Return visit order (0-indexed plant indices) via greedy nearest-neighbour."""
    unvisited = list(range(len(plants)))
    tour = []
    current = depot.copy()
    while unvisited:
        dists = [np.linalg.norm(plants[j] - current) for j in unvisited]
        idx   = int(np.argmin(dists))
        tour.append(unvisited.pop(idx))
        current = plants[tour[-1]]
    return tour

def tsp_path_length(plants, tour, depot=np.zeros(2)):
    """Total path length for a given TSP tour starting from depot."""
    pts = [depot] + [plants[i] for i in tour]
    return sum(np.linalg.norm(pts[k+1] - pts[k]) for k in range(len(pts)-1))

# ── PID hover simulation ───────────────────────────────────────────────────────
def simulate_hover(target, kp, ki, kd, sigma_w, rng, t_spray=T_SPRAY, dt=DT):
    """
    Simulate PID position hold at `target` for T_spray seconds under wind.
    Returns array of position errors (m) at each timestep.
    """
    pos  = target.copy()          # start at target (drone arrives closely)
    vel  = np.zeros(2)
    integral = np.zeros(2)
    phi  = rng.uniform(0, 2*np.pi, size=2)  # per-axis random gust phase

    steps  = int(np.ceil(t_spray / dt))
    errors = np.zeros(steps)

    for k in range(steps):
        t = k * dt
        error     = target - pos
        d_error   = -vel              # derivative of error = -velocity (target fixed)
        integral += error * dt

        accel_pid = kp * error + ki * integral + kd * d_error
        wind      = (A_WIND * np.sin(2*np.pi * t / T_WIND + phi)
                     + rng.normal(0, sigma_w, size=2))
        accel     = accel_pid + wind

        vel  = vel  + accel * dt
        pos  = pos  + vel   * dt
        errors[k] = np.linalg.norm(error)

    return errors

# ── Full mission simulation ────────────────────────────────────────────────────
def run_mission(plants, tour, kp, ki, kd, sigma_w, seed=42):
    """
    Execute the full spray mission: transit + hover at each plant in TSP order.
    Returns per-plant RMS errors and the drone trajectory for plotting.
    """
    rng = np.random.default_rng(seed)
    rms_errors   = np.zeros(N_PLANTS)
    traj_x, traj_y = [], []
    pos = np.zeros(2)   # start at depot (0, 0)

    for visit_idx, plant_idx in enumerate(tour):
        target = plants[plant_idx]

        # ── Transit to plant (straight-line at INSPECTION_SPEED) ──────────────
        dist     = np.linalg.norm(target - pos)
        if dist > 1e-6:
            n_steps  = max(1, int(np.ceil(dist / (INSPECTION_SPEED * DT))))
            step_vec = (target - pos) / n_steps
            for _ in range(n_steps):
                pos = pos + step_vec
                traj_x.append(pos[0])
                traj_y.append(pos[1])

        # ── Hover and spray ───────────────────────────────────────────────────
        errors = simulate_hover(target, kp, ki, kd, sigma_w, rng)
        rms_errors[visit_idx] = np.sqrt(np.mean(errors**2))
        # Record hover positions (approximated at target for trajectory display)
        n_hover = len(errors)
        traj_x.extend([target[0]] * n_hover)
        traj_y.extend([target[1]] * n_hover)
        pos = target.copy()

    trajectory = np.column_stack([traj_x, traj_y])
    return rms_errors, trajectory

# ── Coverage metric ────────────────────────────────────────────────────────────
def effective_coverage(rms_errors, sigma_spray=SIGMA_SPRAY):
    """Gaussian dose model: C_eff = mean over plants of exp(-eps^2 / 2*sigma^2)."""
    return float(np.mean(np.exp(-rms_errors**2 / (2 * sigma_spray**2))))

# ── Main entry point ──────────────────────────────────────────────────────────
def run_simulation():
    rng = np.random.default_rng(0)

    # Generate random plant positions (1 m margin from field boundary)
    plants = rng.uniform(1.0, FIELD_SIZE - 1.0, size=(N_PLANTS, 2))
    depot  = np.zeros(2)
    tour   = tsp_nearest_neighbour(plants, depot)
    L_tsp  = tsp_path_length(plants, tour, depot)
    print(f"TSP path length: {L_tsp:.1f} m  ({N_PLANTS} plants)")

    # ── Baseline run: Tuned PID, sigma_w = SIGMA_W_BASE ──────────────────────
    cfg    = PID_CONFIGS["Tuned"]
    rms_errors, trajectory = run_mission(plants, tour, sigma_w=SIGMA_W_BASE, **cfg)
    c_eff  = effective_coverage(rms_errors)
    print(f"Tuned PID | sigma_w={SIGMA_W_BASE} m/s² | C_eff={c_eff:.3f} "
          f"| mean RMS error={np.mean(rms_errors)*100:.1f} cm")

    # ── Sensitivity sweep: C_eff vs sigma_w for each PID config ──────────────
    sigma_w_range = np.linspace(0.0, 0.5, 20)
    sweep_results = {}
    for label, gains in PID_CONFIGS.items():
        c_vals = []
        for sw in sigma_w_range:
            errs, _ = run_mission(plants, tour, sigma_w=sw, **gains)
            c_vals.append(effective_coverage(errs))
        sweep_results[label] = np.array(c_vals)

    return {
        "plants":        plants,
        "tour":          tour,
        "L_tsp":         L_tsp,
        "rms_errors":    rms_errors,
        "trajectory":    trajectory,
        "c_eff":         c_eff,
        "sigma_w_range": sigma_w_range,
        "sweep_results": sweep_results,
    }
```

---

## Key Parameters

| Parameter | Symbol | Value |
|-----------|--------|-------|
| Field size | — | 20 × 20 m |
| Number of plants | $N$ | 20 |
| Spray width | $w_{spray}$ | 2.0 m |
| Spray accuracy scale | $\sigma_{spray}$ | 0.5 m |
| GPS error std dev | $\sigma_{GPS}$ | 0.1 m |
| Transit speed | $v$ | 1.0 m/s |
| Hover duration per plant | $T_{spray}$ | 3.0 s |
| Simulation timestep | $\Delta t$ | 0.02 s (50 Hz) |
| Gust amplitude | $A_w$ | 0.3 m/s² |
| Gust period | $T_w$ | 4.0 s |
| Baseline turbulence intensity | $\sigma_w$ | 0.1 m/s² |
| Loose PID gains $(K_p, K_i, K_d)$ | — | 0.5, 0.05, 0.1 |
| Tuned PID gains $(K_p, K_i, K_d)$ | — | 2.0, 0.2, 0.5 |
| Tight PID gains $(K_p, K_i, K_d)$ | — | 5.0, 0.5, 1.0 |

---

## Expected Output

- **Field overview plot** (`plt.subplots()` top-down view): 20 × 20 m field boundary drawn as a
  grey rectangle; plant positions marked as green circles scaled by $C_{eff,i}$ (full circle = 1.0,
  smaller = lower dose); TSP tour drawn as a thin black line connecting plants in visit order;
  depot at origin marked with a black square; colour bar showing effective coverage fraction.
- **Per-plant RMS error bar chart**: horizontal bar chart with one bar per plant (sorted by visit
  order), bar length = $\varepsilon_i$ in centimetres; $\sigma_{spray} = 50$ cm reference line;
  bars colour-coded green ($\varepsilon_i < \sigma_{spray}/2$) through red ($\varepsilon_i > \sigma_{spray}$).
- **$C_{eff}$ vs wind intensity sweep**: three curves (Loose / Tuned / Tight PID) plotted against
  $\sigma_w \in [0, 0.5]$ m/s²; horizontal dashed lines at $C_{eff} = 0.9$ and $0.8$ marking
  operationally acceptable thresholds; shaded region below 0.8 indicating mission failure.
- **Hover error time series**: for a representative plant (worst-error and best-error cases),
  plot $\|\mathbf{e}(t)\|$ vs time during the $T_{spray} = 3$ s window for all three PID
  configurations; show transient settling and steady-state fluctuation.
- **Mission animation** (`FuncAnimation`): top-down 2D animation of the drone traversing the TSP
  path; drone shown as a red dot with a circle of radius $\sigma_{spray}$ when hovering; plant dots
  turn green as they are successfully sprayed ($C_{eff,i} > 0.9$) or orange otherwise; frame rate
  10 fps; saved as `outputs/04_industrial_agriculture/s063_precision_spraying/s063_animation.gif`.
- **Console summary**: TSP path length (m), mean hover RMS error (cm), per-config $C_{eff}$, and
  estimated mission time (s).

---

## Extensions

1. **2-opt TSP improvement**: after the greedy nearest-neighbour initialisation, apply the 2-opt
   local search to iteratively uncross tour edges; compare final path length and total mission time
   against the greedy baseline for $N = 20, 50, 100$ plants.
2. **Wind-aware hover time**: extend $T_{spray}$ dynamically until the running RMS error drops
   below a threshold $\varepsilon^* = 0.1$ m; plot the distribution of actual spray times and the
   resulting improvement in $C_{eff}$.
3. **GPS-denied operation**: replace GPS with optical flow (simulated as velocity measurement plus
   Gaussian noise); re-tune the PID gains for the noisier feedback signal and compare $C_{eff}$
   degradation.
4. **Multi-nozzle dose model**: model the spray as a 2D Gaussian plume rather than a single point;
   integrate the dose received at the plant centre as the drone hovers at position $\mathbf{q}(t)$
   and show how plume width reduces sensitivity to hover error.
5. **Energy-aware speed profiling**: replace constant transit speed with a trapezoidal velocity
   profile (accelerate–cruise–decelerate); minimise total mission energy subject to a $C_{eff}$
   constraint, jointly optimising cruise speed and $T_{spray}$.
6. **Terrain-following altitude hold**: extend to 3D with a digital elevation model (DEM); add a
   vertical PID loop to maintain constant height above crop canopy and assess how altitude
   variation affects spray uniformity.

---

## Related Scenarios

- Prerequisites: [S061 Field Boundary Mapping](S061_field_boundary_mapping.md), [S062 Crop Row Following](S062_crop_row_following.md)
- Follow-ups: [S067 Spray Overlap Optimisation](S067_spray_overlap_optimization.md) (strip-width trade-off for area spray), [S068 Variable-Rate Application](S068_variable_rate_application.md) (prescription maps + dose control), [S070 Swarm Coordinated Spraying](S070_swarm_coordinated_spraying.md) (multi-drone coverage)
- Algorithmic cross-reference: [S048 Lawnmower Coverage](../../03_environmental_sar/S048_lawnmower.md) (same strip-geometry trade-off), [S001 Basic Intercept](../../01_pursuit_evasion/S001_basic_intercept.md) (PID position control foundations)
