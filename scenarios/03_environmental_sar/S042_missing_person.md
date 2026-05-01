# S042 Missing Person Localization

**Domain**: Environmental Monitoring & SAR | **Difficulty**: ⭐⭐⭐ | **Status**: `[x]` Complete

---

## Problem Definition

**Setup**: A hiker has gone missing in a flat 200 × 200 m wilderness area. The last known position
(LKP) was recorded 2 hours before the search begins. Because the person has been moving
(or blown by wind) since the LKP, the initial probability of presence is modelled as a Gaussian
centred on the LKP, broadened by a diffusion term proportional to elapsed time. A fleet of
$N = 3$ drones is deployed to localise the person as quickly as possible. Each drone carries a
downward-facing optical/thermal sensor of radius $r_s = 2.0$ m at a scan altitude of
$h = 10.0$ m. The search area is discretised into a $400 \times 400$ grid of $0.5$ m cells. At
every timestep each drone reports a binary detection: **hit** (person detected inside sensor
footprint) or **miss** (person not detected). These observations are fused into a shared
**belief map** via Bayes' theorem. Drones navigate toward the cell with the highest marginal
belief that has not yet been adequately covered — a greedy informativeness frontier policy.

**Roles**:
- **Missing person**: stationary target whose true position is fixed but unknown; occupies a
  single ground-truth cell drawn from the prior distribution at mission start.
- **Drones** ($N = 3$): homogeneous searchers sharing one belief map; each independently
  observes a circular footprint of radius $r_s$ and uploads its binary detection to the shared
  state after every scan.

**Objective**: Minimise the expected time to first confirmed detection (probability of detection
$P_D \geq 0.95$ within sensor footprint) while maximising the rate of **belief entropy reduction**
$\dot{H}$ over the search area. The search terminates when any drone obtains a hit observation,
confirming the person's cell.

**Comparison strategies**:
1. **Greedy max-belief frontier** — each drone always flies toward the unvisited cell with the
   highest current belief value.
2. **Lawnmower sweep** — systematic boustrophedon coverage, ignoring the belief map.
3. **Entropy-weighted random walk** — probabilistic waypoint selection weighted by per-cell
   belief, with temperature annealing.

---

## Mathematical Model

### Prior Distribution

Let $\mathbf{x} = (x, y)$ denote a 2D cell centre and $\mathbf{x}_{LKP}$ the last known
position. The prior probability of the missing person being at cell $\mathbf{x}$ is:

$$P_0(\mathbf{x}) = \frac{1}{Z} \exp\!\left(-\frac{\|\mathbf{x} - \mathbf{x}_{LKP}\|^2}{2\,\sigma_0^2}\right)$$

where $\sigma_0^2 = \sigma_{LKP}^2 + 2 D \Delta t$ combines positional uncertainty at the LKP
($\sigma_{LKP} = 10$ m) with Brownian diffusion ($D = 0.02$ m$^2$/s, $\Delta t = 7200$ s), giving
$\sigma_0 \approx 17.2$ m. $Z$ is the normalisation constant over all $N_c$ grid cells:

$$Z = \sum_{\mathbf{x} \in \mathcal{G}} \exp\!\left(-\frac{\|\mathbf{x} - \mathbf{x}_{LKP}\|^2}{2\,\sigma_0^2}\right)$$

### Sensor (Likelihood) Model

Drone $i$ at position $\mathbf{p}_i$ produces observation $z_i \in \{0, 1\}$ (miss / hit) at
each timestep. The detection footprint is a disc of radius $r_s$. The detection probability for
cell $\mathbf{x}$ given the person is there ($H_{\mathbf{x}}$: person at $\mathbf{x}}$) is:

$$P(z_i = 1 \mid H_{\mathbf{x}}) =
\begin{cases}
P_d & \text{if } \|\mathbf{x} - \mathbf{p}_i\| \leq r_s \\
0   & \text{otherwise}
\end{cases}$$

with probability of detection $P_d = 0.90$ (imperfect sensor: 10 % miss rate). The
false-alarm rate is $P_{fa} = 0.02$ (sensor occasionally fires in empty footprint). The full
likelihood of observation $z_i$ given cell hypothesis $H_{\mathbf{x}}$:

$$P(z_i \mid H_{\mathbf{x}}) =
\begin{cases}
P_d  \cdot \mathbf{1}[\|\mathbf{x}-\mathbf{p}_i\|\leq r_s] + P_{fa} \cdot \mathbf{1}[\|\mathbf{x}-\mathbf{p}_i\|> r_s]
  & z_i = 1 \\
(1-P_d) \cdot \mathbf{1}[\|\mathbf{x}-\mathbf{p}_i\|\leq r_s] + (1-P_{fa}) \cdot \mathbf{1}[\|\mathbf{x}-\mathbf{p}_i\|> r_s]
  & z_i = 0
\end{cases}$$

### Bayesian Belief Update

Let $B_t(\mathbf{x})$ be the normalised belief map at time $t$, initialised as
$B_0(\mathbf{x}) = P_0(\mathbf{x})$. After drone $i$ reports observation $z_i$ at time $t$:

$$B_{t+1}(\mathbf{x}) = \frac{P(z_i \mid H_{\mathbf{x}}) \cdot B_t(\mathbf{x})}
  {\displaystyle\sum_{\mathbf{x}' \in \mathcal{G}} P(z_i \mid H_{\mathbf{x}'}) \cdot B_t(\mathbf{x}')}$$

When $N$ drones observe independently at the same timestep, the updates are applied
sequentially (drone order is arbitrary; each update uses the posterior of the previous):

$$B_{t+1} = \text{Bayes}\!\left(\text{Bayes}\!\left(\cdots \text{Bayes}(B_t, z_1) \cdots, z_{N-1}\right), z_N\right)$$

### Cumulative Probability of Detection

The probability that the person has been detected at least once by time $t$:

$$P_{det}(t) = 1 - \prod_{\tau=0}^{t} \prod_{i=1}^{N}
  \bigl[1 - P(z_i^\tau = 1 \mid H_{\mathbf{x}^*})\bigr]$$

where $\mathbf{x}^*$ is the true person location. In practice $\mathbf{x}^*$ is unknown; the
**expected** cumulative detection probability marginalises over the belief:

$$\mathbb{E}[P_{det}(t)] = \sum_{\mathbf{x}} B_t(\mathbf{x}) \cdot
  \left(1 - \prod_{\tau \leq t,\, i} [1 - P(z_i^\tau \mid H_{\mathbf{x}})] \right)$$

### Belief Entropy

The Shannon entropy of the belief map quantifies remaining uncertainty:

$$H(B_t) = -\sum_{\mathbf{x} \in \mathcal{G}} B_t(\mathbf{x}) \log B_t(\mathbf{x})$$

The greedy frontier policy selects each drone's next waypoint $\mathbf{w}_i^*$ as:

$$\mathbf{w}_i^* = \arg\max_{\mathbf{w} \in \mathcal{W}} \; B_t(\mathbf{w})$$

subject to $\mathbf{w}$ not being within $r_s$ of any waypoint already assigned to another
drone at the same timestep (collision and redundancy avoidance):

$$\|\mathbf{w}_i^* - \mathbf{w}_j^*\| > 2\,r_s \quad \forall\, j \neq i$$

### Expected Time to Detection

The objective to minimise is the expected number of timesteps until first hit:

$$\mathbb{E}[T_{det}] = \sum_{t=0}^{\infty} t \cdot P(\text{first hit at } t)
= \sum_{t=0}^{\infty} \mathbb{E}[P_{det}(t)] \cdot \prod_{\tau < t}(1 - \mathbb{E}[P_{det}(\tau)])$$

### Drone Kinematics

Point-mass, constant-speed straight-line flight between waypoints at cruise altitude $h$:

$$\dot{\mathbf{p}}_i = v_d \cdot \hat{\mathbf{r}}_i, \qquad
  \hat{\mathbf{r}}_i = \frac{\mathbf{w}_i^* - \mathbf{p}_i}{\|\mathbf{w}_i^* - \mathbf{p}_i\|}$$

with $v_d = 5$ m/s. A new observation is recorded each time drone $i$ arrives at waypoint
$\mathbf{w}_i^*$ (sensor dwell of $\tau_{dwell} = 1$ s). Transit time between waypoints:

$$\Delta t_{transit} = \frac{\|\mathbf{w}_i^* - \mathbf{p}_i\|}{v_d}$$

---

## Implementation

```python
import numpy as np
import matplotlib.pyplot as plt
from dataclasses import dataclass, field

# Key constants
N_DRONES       = 3
AREA_SIZE      = 200.0      # m, square area side
GRID_RES       = 0.5        # m per cell
GRID_N         = int(AREA_SIZE / GRID_RES)   # 400 cells per side
SENSOR_RADIUS  = 2.0        # m
SCAN_HEIGHT    = 10.0       # m (not used kinematically in 2D, but logged)
DRONE_SPEED    = 5.0        # m/s
TAU_DWELL      = 1.0        # s — sensor dwell at each waypoint

P_DETECT       = 0.90       # P(hit | person in footprint)
P_FALSE_ALARM  = 0.02       # P(hit | person NOT in footprint)

SIGMA_LKP      = 10.0       # m — LKP positional uncertainty
DIFFUSION_D    = 0.02       # m^2/s — Brownian motion diffusion coefficient
DELTA_T_HOURS  = 2.0        # hours since LKP
DELTA_T_SEC    = DELTA_T_HOURS * 3600.0

LKP = np.array([100.0, 120.0])   # last known position (m, within 200x200 area)

# Grid cell centres
xs = np.arange(0.5 * GRID_RES, AREA_SIZE, GRID_RES)   # shape (400,)
ys = np.arange(0.5 * GRID_RES, AREA_SIZE, GRID_RES)
XX, YY = np.meshgrid(xs, ys)                            # shape (400, 400)
CELLS = np.stack([XX.ravel(), YY.ravel()], axis=-1)     # shape (160000, 2)

def build_prior():
    """Gaussian prior centred on LKP, broadened by Brownian diffusion."""
    sigma2 = SIGMA_LKP**2 + 2.0 * DIFFUSION_D * DELTA_T_SEC
    dist2 = np.sum((CELLS - LKP)**2, axis=1)
    log_p = -dist2 / (2.0 * sigma2)
    log_p -= log_p.max()
    p = np.exp(log_p)
    return p / p.sum()          # normalised, shape (160000,)

def sensor_footprint(drone_pos):
    """Boolean mask of cells within sensor radius of drone position."""
    dist = np.linalg.norm(CELLS - drone_pos, axis=1)
    return dist <= SENSOR_RADIUS  # shape (160000,)

def bayes_update(belief, drone_pos, hit):
    """
    Update belief map with one binary observation.
    hit=True  -> z=1 (detection)
    hit=False -> z=0 (miss)
    """
    in_fp = sensor_footprint(drone_pos)
    if hit:
        likelihood = np.where(in_fp, P_DETECT, P_FALSE_ALARM)
    else:
        likelihood = np.where(in_fp, 1.0 - P_DETECT, 1.0 - P_FALSE_ALARM)
    posterior = likelihood * belief
    total = posterior.sum()
    if total < 1e-300:
        return belief          # numerical guard: belief unchanged
    return posterior / total

def belief_entropy(belief):
    """Shannon entropy of the belief map (nats)."""
    mask = belief > 0
    return -np.sum(belief[mask] * np.log(belief[mask]))

def greedy_next_waypoint(belief, drone_pos, other_waypoints):
    """
    Select the highest-belief cell not redundantly covered by other drones.
    Cells within 2*r_s of an already-assigned waypoint are excluded.
    """
    scores = belief.copy()
    for wp in other_waypoints:
        too_close = np.linalg.norm(CELLS - wp, axis=1) < 2.0 * SENSOR_RADIUS
        scores[too_close] = 0.0
    best_idx = np.argmax(scores)
    return CELLS[best_idx].copy()

@dataclass
class Drone:
    idx: int
    pos: np.ndarray
    waypoint: np.ndarray
    t_arrive_waypoint: float = 0.0   # simulation time at which drone reaches waypoint

def simulate(strategy="greedy", seed=42):
    """
    Run one full search mission.
    Returns: time-series of (t, entropy, expected_pdet), detection_time or None.
    """
    rng = np.random.default_rng(seed)
    belief = build_prior()

    # Draw true person location from prior
    person_idx = rng.choice(len(CELLS), p=belief)
    person_pos = CELLS[person_idx].copy()

    # Initialise drones at spread-out positions along the south edge
    start_xs = [50.0, 100.0, 150.0]
    drones = [
        Drone(idx=k,
              pos=np.array([start_xs[k], 10.0]),
              waypoint=np.array([start_xs[k], 10.0]))
        for k in range(N_DRONES)
    ]

    sim_time = 0.0
    history = []
    detection_time = None
    max_sim_time = 1800.0    # 30 minutes hard cutoff

    while sim_time < max_sim_time:
        # Advance to next drone arrival at its waypoint
        next_arrival = min(d.t_arrive_waypoint for d in drones)
        sim_time = next_arrival

        # All drones that have just arrived perform a scan
        for drone in sorted(drones, key=lambda d: d.t_arrive_waypoint):
            if abs(drone.t_arrive_waypoint - sim_time) > 1e-6:
                continue

            # Simulate the sensor observation
            dist_to_person = np.linalg.norm(drone.pos - person_pos)
            in_fp = dist_to_person <= SENSOR_RADIUS
            if in_fp:
                hit = rng.random() < P_DETECT
            else:
                hit = rng.random() < P_FALSE_ALARM

            # Bayesian update
            belief = bayes_update(belief, drone.pos, hit)

            # Check termination
            if hit and in_fp:
                detection_time = sim_time
                history.append((sim_time, belief_entropy(belief), None))
                return history, detection_time, belief

            # Assign next waypoint (greedy strategy)
            if strategy == "greedy":
                other_wps = [d.waypoint for d in drones if d.idx != drone.idx]
                new_wp = greedy_next_waypoint(belief, drone.pos, other_wps)
            elif strategy == "lawnmower":
                # Pre-computed boustrophedon: advance along row
                new_wp = _lawnmower_next(drone)
            else:
                raise ValueError(f"Unknown strategy: {strategy}")

            transit_dist = np.linalg.norm(new_wp - drone.pos)
            drone.pos = new_wp.copy()
            drone.waypoint = new_wp.copy()
            drone.t_arrive_waypoint = sim_time + transit_dist / DRONE_SPEED + TAU_DWELL

        h = belief_entropy(belief)
        history.append((sim_time, h, None))

    return history, detection_time, belief   # None means not found within cutoff

def plot_belief_map(belief, drones, person_pos, title="Belief Map"):
    """Heatmap of belief with drone positions and true person location overlaid."""
    grid = belief.reshape(GRID_N, GRID_N)
    fig, ax = plt.subplots(figsize=(7, 7))
    im = ax.imshow(grid, origin="lower", extent=[0, AREA_SIZE, 0, AREA_SIZE],
                   cmap="hot_r", interpolation="nearest")
    plt.colorbar(im, ax=ax, label="Belief P(person at cell)")
    for d in drones:
        circle = plt.Circle(d.pos, SENSOR_RADIUS, color="cyan", fill=False, lw=1.5)
        ax.add_patch(circle)
        ax.plot(*d.pos, "c^", ms=8, label=f"Drone {d.idx}")
    ax.plot(*person_pos, "g*", ms=14, label="True position")
    ax.plot(*LKP, "wx", ms=12, mew=2.5, label="LKP")
    ax.set_xlim(0, AREA_SIZE)
    ax.set_ylim(0, AREA_SIZE)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title(title)
    ax.legend(loc="upper right", fontsize=8)
    plt.tight_layout()
    return fig

def plot_entropy_history(histories_dict):
    """Entropy over time for each strategy."""
    fig, ax = plt.subplots(figsize=(8, 4))
    for label, (history, det_time, _) in histories_dict.items():
        ts = [h[0] for h in history]
        hs = [h[1] for h in history]
        ax.plot(ts, hs, label=label)
        if det_time is not None:
            ax.axvline(det_time, color="gray", ls="--", lw=0.8)
    ax.set_xlabel("Simulation time (s)")
    ax.set_ylabel("Belief entropy H(B) (nats)")
    ax.set_title("Entropy Reduction over Time")
    ax.legend()
    plt.tight_layout()
    return fig

def run_simulation():
    results = {}
    for strategy in ("greedy", "lawnmower"):
        history, det_time, final_belief = simulate(strategy=strategy, seed=0)
        results[strategy] = (history, det_time, final_belief)
        status = f"{det_time:.1f} s" if det_time else "NOT FOUND"
        print(f"[{strategy:>10s}] detection time: {status}")
    return results
```

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Search area | 200 × 200 m |
| Grid resolution | 0.5 m/cell (400 × 400 = 160 000 cells) |
| Number of drones $N$ | 3 |
| Sensor footprint radius $r_s$ | 2.0 m |
| Scan altitude $h$ | 10.0 m |
| Drone cruise speed $v_d$ | 5.0 m/s |
| Sensor dwell per waypoint $\tau_{dwell}$ | 1.0 s |
| Probability of detection $P_d$ | 0.90 |
| False alarm rate $P_{fa}$ | 0.02 |
| LKP positional uncertainty $\sigma_{LKP}$ | 10.0 m |
| Brownian diffusion coefficient $D$ | 0.02 m²/s |
| Time since LKP $\Delta t$ | 7200 s (2 hours) |
| Prior spread $\sigma_0$ | $\approx$ 17.2 m |
| LKP coordinates | (100, 120) m |
| Hard mission cutoff | 1800 s (30 minutes) |
| Redundancy exclusion radius | $2\,r_s = 4.0$ m |

---

## Expected Output

- **Prior belief heatmap**: 2D top-down plot of $B_0(\mathbf{x})$ over the 400 × 400 grid;
  Gaussian centred on LKP with $\sigma_0 \approx 17.2$ m; colour scale from white (low) to red
  (high belief); LKP marked with white cross.
- **Belief evolution snapshots**: four-panel figure showing $B_t$ at $t = 0, 60, 120,$ and
  $t_{det}$ s; drone positions overlaid as cyan triangles; sensor footprints as circles; true
  person position as green star (revealed only at detection).
- **Entropy reduction curve**: $H(B_t)$ vs simulation time for Greedy and Lawnmower strategies
  on the same axes; detection event marked with a vertical dashed line per strategy.
- **Expected $P_{det}(t)$ curve**: $\mathbb{E}[P_{det}(t)]$ vs $t$ for both strategies; 0.95
  threshold marked as horizontal dashed line; the $t$ at which each strategy crosses this
  threshold is annotated.
- **Detection time box-plot**: distribution of $T_{det}$ over 50 Monte Carlo trials (random
  true positions drawn from prior) for each strategy; median and IQR reported in the figure.
- **Animation (GIF)**: top-down view with drones moving in real time, belief heatmap updating
  after each scan, entropy value displayed in title; frame rate 10 fps.

---

## Extensions

1. **Non-stationary target**: replace the fixed person assumption with a correlated random walk
   model $\dot{\mathbf{x}}^* = \mathbf{v}^* + \boldsymbol{\eta}$, $\boldsymbol{\eta} \sim
   \mathcal{N}(\mathbf{0}, Q)$; apply a predict-step (convolution with a diffusion kernel)
   to the belief map at each timestep before the Bayes update.
2. **Terrain-aware prior**: incorporate terrain features (cliffs, rivers, dense vegetation) as
   a non-Gaussian prior by reweighting cells using a terrain traversal cost map; test on a
   synthetic DEM generated with Perlin noise.
3. **Particle filter for large areas**: replace the grid belief map with $N_p = 5000$ weighted
   particles for a 1 km × 1 km area where a full grid is computationally prohibitive; apply
   systematic resampling when effective particle count drops below $N_p / 2$.
4. **Coordinated entropy-based path planning**: replace the greedy next-waypoint policy with a
   receding-horizon planner that optimises joint entropy reduction over a $T = 60$ s planning
   window across all drones simultaneously (decentralised POMDP approximation).
5. **Communication-degraded search**: introduce intermittent communication loss so drones cannot
   share belief updates for windows of $\Delta t_{blackout} \sim 30$ s; implement a
   divergence-aware merge step when contact is restored (KL-divergence between local and
   shared beliefs).

---

## Related Scenarios

- Prerequisites: [S041 Area Coverage Sweep](S041_area_coverage_sweep.md)
- Follow-ups: [S043 Wildfire Detection](S043_wildfire_detection.md), [S044 Flood Mapping](S044_flood_mapping.md)
- Algorithmic cross-reference: [S003 Low-Altitude Tracking](../01_pursuit_evasion/S003_low_altitude_tracking.md) (sensor-range pursuit), [S013 Particle Filter Intercept](../01_pursuit_evasion/S013_particle_filter_intercept.md) (particle filter state estimation)
