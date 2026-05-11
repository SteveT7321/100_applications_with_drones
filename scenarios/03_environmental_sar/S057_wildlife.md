# S057 Wildlife Population Census

**Domain**: Environmental Monitoring & SAR | **Difficulty**: ⭐⭐ | **Status**: `[ ]` Not Started
**Algorithm**: Distance-Sampling Census + Lawnmower Coverage | **Dimension**: 2D

---

## Problem Definition

**Setup**: A single drone surveys a $500 \times 300$ m savanna reserve to count a target wildlife
species. The true population size is $N_{true} = 40$ animals. Animals are modelled as point
targets whose locations are drawn from a **2D spatial Poisson process** with non-uniform intensity
$\lambda(x, y)$: individuals cluster near two water sources, producing a bimodal spatial
distribution rather than uniform scatter.

The drone flies a **boustrophedon (lawnmower) path** at a fixed altitude $h$. Its nadir-pointing
sensor has a half-power detection radius $r_{half}$: an animal at ground-range $r$ from the
drone's nadir point is detected with probability given by a Gaussian roll-off model. The strip
width is set to $d = 2\,r_{half}$, which is the classical optimal spacing for the half-power
sensor model. Missed detections and false alarms are both possible.

After the flight the raw count $N_{detected}$ is corrected using the **distance-sampling
estimator**: the population estimate $N_{est}$ adjusts for the fraction of animals within each
strip that the sensor is expected to miss. The quality of the estimator is assessed by running
100 Monte Carlo trials (different random animal placements from the same Poisson process) and
computing the bias and coefficient of variation of $N_{est}$.

**Roles**:
- **Animals**: $N_{true}$ stationary point targets placed at mission start via a Poisson draw
  from $\lambda(x, y)$; they do not move during the flight.
- **Drone**: single UAV executing a pre-planned lawnmower path; no replanning.

**Objective**: Quantify the accuracy of the distance-sampling population estimator by reporting
bias $= \mathbb{E}[N_{est}] - N_{true}$, variance $\text{Var}(N_{est})$, and coefficient of
variation $\text{CV}(N_{est}) = \sigma(N_{est}) / \mathbb{E}[N_{est}]$ over the Monte Carlo ensemble.

---

## Mathematical Model

### Animal Spatial Distribution

Animals are distributed according to a 2D inhomogeneous Poisson process with intensity:

$$\lambda(x, y) = \lambda_0 \left[
    \exp\!\left(-\frac{(x-x_1)^2 + (y-y_1)^2}{2\sigma_c^2}\right)
  + \exp\!\left(-\frac{(x-x_2)^2 + (y-y_2)^2}{2\sigma_c^2}\right)
\right]$$

where $(x_1, y_1)$ and $(x_2, y_2)$ are water-source locations, $\sigma_c$ controls cluster
spread, and $\lambda_0$ is scaled so that $\iint_{\mathcal{A}} \lambda(x, y)\,dx\,dy = N_{true}$.
The total animal count per realisation $N \sim \text{Poisson}(N_{true})$; individual locations
are drawn by thinning a uniform Poisson process with acceptance probability
$\lambda(x, y) / \lambda_{max}$.

### Detection Probability Model

The drone at position $\mathbf{p}(t)$ detects an animal at ground location $\mathbf{a}_k$ with
probability:

$$P_d(r_k) = \exp\!\left(-\frac{r_k^2}{r_{half}^2}\right), \qquad r_k = \|\mathbf{p}(t) - \mathbf{a}_k\|$$

Here $r_{half}$ is the **half-power radius**: the range at which $P_d = 1/e \approx 0.368$.
Detection events are drawn as independent Bernoulli trials:

$$Z_k \sim \text{Bernoulli}(P_d(r_k)), \quad k = 1, \ldots, N$$

Only animals within the sensor footprint $r_k \leq r_{half}$ (the active strip half-width) are
considered at each drone position.

### Lawnmower Path and Strip Geometry

The reserve is divided into parallel east-west strips of width $d = 2\,r_{half}$. The number of
strips is:

$$N_{strips} = \left\lceil \frac{W}{d} \right\rceil$$

where $W = 300$ m is the north-south width of the reserve. Strip $i$ is centred at:

$$y_i = y_0 + \left(i + \tfrac{1}{2}\right) d, \quad i = 0, 1, \ldots, N_{strips} - 1$$

The boustrophedon direction alternates per strip:

$$\mathbf{w}_{i}^{start} = \begin{cases}(x_{min},\; y_i) & i \text{ even} \\ (x_{max},\; y_i) & i \text{ odd}\end{cases}, \qquad \mathbf{w}_{i}^{end} = \begin{cases}(x_{max},\; y_i) & i \text{ even} \\ (x_{min},\; y_i) & i \text{ odd}\end{cases}$$

Total path length:

$$L = N_{strips} \cdot H + (N_{strips} - 1) \cdot d$$

where $H = 500$ m is the east-west (along-track) length.

### Effective Strip Half-Width (Distance Sampling)

The key quantity for the distance-sampling correction is the **effective strip half-width** $\mu$,
the distance at which an animal "could just as well have been at zero range with certainty":

$$\mu = \int_0^{\infty} P_d(r)\,dr = \int_0^{\infty} \exp\!\left(-\frac{r^2}{r_{half}^2}\right)dr = \frac{r_{half}\,\sqrt{\pi}}{2}$$

The effective strip area swept per unit flight distance is:

$$w_{eff} = 2\mu = r_{half}\,\sqrt{\pi}$$

### Population Estimator

The expected number of detections per strip of along-track length $H$ given $N_{true}$ animals
uniformly distributed in area $A_{total} = H \times W$ is:

$$\mathbb{E}[N_{det}] = N_{true} \cdot \frac{N_{strips} \cdot w_{eff} \cdot H}{A_{total}}$$

Inverting this gives the **distance-sampling population estimator**:

$$N_{est} = N_{det} \cdot \frac{A_{total}}{N_{strips} \cdot w_{eff} \cdot H}$$

Because $N_{det}$ is a random variable, $N_{est}$ is also random. The estimator is **unbiased**
when animals are uniformly distributed; with the non-uniform (clustered) distribution the bias
measures the effect of spatial heterogeneity.

### Monte Carlo Performance Metrics

Over $M = 100$ independent realisations indexed by $m$:

$$\mathbb{E}[N_{est}] \approx \frac{1}{M}\sum_{m=1}^{M} N_{est}^{(m)}, \qquad
\text{Bias} = \mathbb{E}[N_{est}] - N_{true}$$

$$\text{Var}(N_{est}) \approx \frac{1}{M-1}\sum_{m=1}^{M}\bigl(N_{est}^{(m)} - \mathbb{E}[N_{est}]\bigr)^2$$

$$\text{CV}(N_{est}) = \frac{\sqrt{\text{Var}(N_{est})}}{\mathbb{E}[N_{est}]}$$

A well-calibrated survey achieves $|\text{Bias}| / N_{true} < 5\%$ and $\text{CV} < 20\%$.

---

## Implementation

```python
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Circle

# ── Key constants ─────────────────────────────────────────────────────────────
AREA_W      = 500.0    # m — along-track (east-west) length
AREA_H      = 300.0    # m — cross-track (north-south) width
N_TRUE      = 40       # true animal count (expected Poisson mean)
R_HALF      = 20.0     # m — half-power detection radius
STRIP_W     = 2 * R_HALF           # m — optimal strip width for half-power model
V_DRONE     = 10.0     # m/s — cruise speed
DT          = 1.0      # s — simulation timestep
N_MC        = 100      # Monte Carlo trials

# Water source locations and cluster spread
WATER_SOURCES = [(150.0, 80.0), (350.0, 220.0)]
SIGMA_CLUSTER = 60.0   # m — std dev of animal clustering around each source

# ── Poisson animal placement ──────────────────────────────────────────────────
def intensity(x, y):
    """Non-uniform intensity lambda(x,y); un-normalised."""
    val = 0.0
    for (wx, wy) in WATER_SOURCES:
        val += np.exp(-((x - wx)**2 + (y - wy)**2) / (2 * SIGMA_CLUSTER**2))
    return val

def place_animals(n_true, rng):
    """
    Rejection sampling from 2D Poisson process with intensity lambda(x,y).
    Expected count = n_true; actual count ~ Poisson(n_true).
    """
    # Thinning: draw N ~ Poisson(n_true) from uniform, accept with P=lambda/lambda_max
    n_candidates = rng.poisson(n_true * 4)   # oversample before thinning
    xs = rng.uniform(0, AREA_W, n_candidates)
    ys = rng.uniform(0, AREA_H, n_candidates)
    lam = intensity(xs, ys)
    lam_max = intensity(*WATER_SOURCES[0])    # approximate max at a water source
    lam_max = max(lam_max, intensity(*WATER_SOURCES[1]))
    accept = rng.random(n_candidates) < lam / lam_max
    return xs[accept], ys[accept]

# ── Lawnmower path ─────────────────────────────────────────────────────────────
def build_lawnmower(strip_w=STRIP_W):
    """Return list of (x, y) waypoints for boustrophedon scan of the reserve."""
    n_strips = int(np.ceil(AREA_H / strip_w))
    waypoints = []
    for i in range(n_strips):
        yc = (i + 0.5) * strip_w
        if i % 2 == 0:
            waypoints.append((0.0, yc))
            waypoints.append((AREA_W, yc))
        else:
            waypoints.append((AREA_W, yc))
            waypoints.append((0.0, yc))
    return waypoints, n_strips

# ── Detection simulation ───────────────────────────────────────────────────────
def simulate_survey(ax, ay, waypoints, rng):
    """
    Fly the lawnmower path; simulate detections for each animal.
    Returns number of detections and list of (drone_x, drone_y) trajectory.
    """
    positions = []
    detected = np.zeros(len(ax), dtype=bool)

    pos = np.array(waypoints[0], dtype=float)
    positions.append(pos.copy())

    for wp in waypoints[1:]:
        target = np.array(wp, dtype=float)
        direction = target - pos
        dist = np.linalg.norm(direction)
        if dist < 1e-6:
            continue
        unit = direction / dist
        n_steps = max(1, int(np.ceil(dist / (V_DRONE * DT))))
        step = dist / n_steps * unit

        for _ in range(n_steps):
            pos = pos + step
            positions.append(pos.copy())
            # Detection check for all animals
            dx = ax - pos[0]
            dy = ay - pos[1]
            r = np.sqrt(dx**2 + dy**2)
            in_strip = r <= R_HALF
            pd = np.exp(-(r**2) / (R_HALF**2))
            hits = in_strip & ~detected & (rng.random(len(ax)) < pd)
            detected |= hits

    return int(detected.sum()), np.array(positions)

# ── Distance-sampling estimator ────────────────────────────────────────────────
MU = R_HALF * np.sqrt(np.pi) / 2.0      # effective strip half-width (m)
W_EFF = 2 * MU                           # effective strip full-width (m)

def population_estimate(n_det, n_strips):
    """Distance-sampling correction: N_est = N_det * A_total / (n_strips * w_eff * H)."""
    effective_area = n_strips * W_EFF * AREA_W
    return n_det * (AREA_W * AREA_H) / effective_area

# ── Monte Carlo loop ───────────────────────────────────────────────────────────
def run_monte_carlo(n_trials=N_MC, seed=0):
    rng = np.random.default_rng(seed)
    waypoints, n_strips = build_lawnmower()
    n_est_list = []
    n_det_list = []

    for _ in range(n_trials):
        ax, ay = place_animals(N_TRUE, rng)
        n_det, _ = simulate_survey(ax, ay, waypoints, rng)
        n_est = population_estimate(n_det, n_strips)
        n_det_list.append(n_det)
        n_est_list.append(n_est)

    n_est_arr = np.array(n_est_list)
    mean_est  = n_est_arr.mean()
    bias      = mean_est - N_TRUE
    std_est   = n_est_arr.std(ddof=1)
    cv        = std_est / mean_est

    print(f"N_true     : {N_TRUE}")
    print(f"Mean N_det : {np.mean(n_det_list):.1f}")
    print(f"Mean N_est : {mean_est:.1f}")
    print(f"Bias       : {bias:+.2f}  ({100*bias/N_TRUE:+.1f}%)")
    print(f"Std(N_est) : {std_est:.2f}")
    print(f"CV(N_est)  : {cv*100:.1f}%")

    return n_est_arr, n_det_list, waypoints, n_strips

# ── Visualisation: single-trial animal map + detections ───────────────────────
def plot_animal_map(ax_locs, ay_locs, detected_mask, waypoints, trajectory):
    fig, ax = plt.subplots(figsize=(10, 6))
    ax.scatter(ax_locs[~detected_mask], ay_locs[~detected_mask],
               c="steelblue", s=30, zorder=3, label="Missed animals")
    ax.scatter(ax_locs[detected_mask], ay_locs[detected_mask],
               c="red", s=50, marker="*", zorder=4, label="Detected animals")
    traj = np.array(trajectory)
    ax.plot(traj[:, 0], traj[:, 1], "k-", lw=0.6, alpha=0.5, label="Drone path")
    for (wx, wy) in WATER_SOURCES:
        ax.plot(wx, wy, "g^", ms=10, zorder=5)
    ax.set_xlim(0, AREA_W)
    ax.set_ylim(0, AREA_H)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("Wildlife Census — Single Survey (animal map + detections)")
    ax.legend(loc="upper right", fontsize=8)
    plt.tight_layout()
    return fig

# ── Visualisation: N_est distribution over MC trials ─────────────────────────
def plot_mc_distribution(n_est_arr):
    fig, ax = plt.subplots(figsize=(7, 4))
    ax.hist(n_est_arr, bins=20, color="steelblue", edgecolor="white", alpha=0.8)
    ax.axvline(N_TRUE, color="green", lw=2, linestyle="--", label=f"N_true = {N_TRUE}")
    ax.axvline(n_est_arr.mean(), color="red", lw=2, linestyle="-",
               label=f"Mean N_est = {n_est_arr.mean():.1f}")
    ax.set_xlabel("Population estimate N_est")
    ax.set_ylabel("Frequency")
    ax.set_title(f"Distance-Sampling Estimator Distribution ({N_MC} Monte Carlo trials)")
    ax.legend()
    plt.tight_layout()
    return fig

# ── Animation: drone survey ───────────────────────────────────────────────────
def build_animation(trajectory, ax_locs, ay_locs, n_strips):
    """Top-down animation of the drone sweeping the reserve."""
    fig, ax = plt.subplots(figsize=(10, 6))
    ax.set_xlim(0, AREA_W)
    ax.set_ylim(0, AREA_H)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.scatter(ax_locs, ay_locs, c="steelblue", s=25, zorder=3, label="Animals")
    for (wx, wy) in WATER_SOURCES:
        ax.plot(wx, wy, "g^", ms=10, zorder=5, label="Water source")
    path_line, = ax.plot([], [], "k-", lw=0.8, alpha=0.5)
    drone_pt,  = ax.plot([], [], "ro", ms=8, zorder=6, label="Drone")
    footprint  = Circle((0, 0), R_HALF, color="red", fill=False, lw=1.2, alpha=0.6)
    ax.add_patch(footprint)
    title = ax.set_title("Wildlife Census — t = 0 s")
    ax.legend(loc="upper right", fontsize=8)

    step = max(1, len(trajectory) // 400)   # cap at ~400 frames
    frames = trajectory[::step]

    def init():
        path_line.set_data([], [])
        drone_pt.set_data([], [])
        return path_line, drone_pt, footprint, title

    def update(i):
        fx, fy = frames[i]
        path_line.set_data(frames[:i+1, 0], frames[:i+1, 1])
        drone_pt.set_data([fx], [fy])
        footprint.center = (fx, fy)
        title.set_text(f"Wildlife Census — step {i*step}")
        return path_line, drone_pt, footprint, title

    ani = animation.FuncAnimation(fig, update, frames=len(frames),
                                  init_func=init, blit=True, interval=50)
    return fig, ani

def run_simulation():
    n_est_arr, n_det_list, waypoints, n_strips = run_monte_carlo()

    # Single-trial visual
    rng_vis = np.random.default_rng(42)
    ax_locs, ay_locs = place_animals(N_TRUE, rng_vis)
    n_det_vis, traj_vis = simulate_survey(ax_locs, ay_locs, waypoints, rng_vis)
    detected_mask = np.zeros(len(ax_locs), dtype=bool)
    # Recompute which animals were detected for plotting
    detected_mask_full = np.zeros(len(ax_locs), dtype=bool)
    pos = np.array(waypoints[0], dtype=float)
    rng_vis2 = np.random.default_rng(42)
    ax_locs2, ay_locs2 = place_animals(N_TRUE, rng_vis2)
    _, traj = simulate_survey(ax_locs2, ay_locs2, waypoints, np.random.default_rng(42))

    fig1 = plot_animal_map(ax_locs, ay_locs, detected_mask, waypoints, traj_vis)
    fig2 = plot_mc_distribution(n_est_arr)
    fig3, ani = build_animation(traj_vis, ax_locs, ay_locs, n_strips)

    return fig1, fig2, fig3, ani
```

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Reserve area | 500 × 300 m |
| True population $N_{true}$ | 40 animals |
| Half-power detection radius $r_{half}$ | 20 m |
| Optimal strip width $d = 2\,r_{half}$ | 40 m |
| Effective strip half-width $\mu = r_{half}\sqrt{\pi}/2$ | ≈ 17.72 m |
| Effective strip width $w_{eff} = 2\mu$ | ≈ 35.45 m |
| Number of strips $N_{strips}$ | $\lceil 300/40 \rceil = 8$ |
| Total path length $L$ | $8 \times 500 + 7 \times 40 = 4280$ m |
| Drone cruise speed $v$ | 10 m/s |
| Simulation timestep $\Delta t$ | 1 s |
| Water source 1 $(x_1, y_1)$ | (150, 80) m |
| Water source 2 $(x_2, y_2)$ | (350, 220) m |
| Cluster spread $\sigma_c$ | 60 m |
| Monte Carlo trials $M$ | 100 |
| Target CV threshold | < 20 % |
| Target bias threshold | $< 5\%$ of $N_{true}$ |

---

## Expected Output

- **Animal map + detections plot**: 2D top-down view of the $500 \times 300$ m reserve; blue dots
  for missed animals, red stars for detected animals; green triangles at water-source locations;
  black lawnmower path overlaid; title reporting $N_{true}$, $N_{detected}$, and $N_{est}$ for
  the single representative trial.
- **$N_{est}$ distribution histogram**: frequency histogram of the population estimate over
  $M = 100$ Monte Carlo trials; green dashed vertical line at $N_{true} = 40$; red solid
  vertical line at $\mathbb{E}[N_{est}]$; title or legend reporting bias, $\sigma(N_{est})$, and
  CV.
- **Animation (drone survey)**: top-down view of the drone sweeping the reserve strip by strip;
  red drone marker with sensor footprint circle; trail of path history; animal locations visible
  as static blue dots; frame counter in title.
- **Console metrics** (printed):
  - $N_{true}$, mean $N_{detected}$, mean $N_{est}$
  - Bias (absolute and as % of $N_{true}$)
  - $\sigma(N_{est})$ and $\text{CV}(N_{est})$

---

## Extensions

1. **Variable strip width optimisation**: sweep $d$ from $r_{half}$ to $4\,r_{half}$ and plot
   CV$(N_{est})$ vs $d$; verify that $d = 2\,r_{half}$ (equivalently, $d = w_{eff}/\sqrt{\pi/2}$)
   minimises CV for the half-power detection model.
2. **Non-uniform density correction**: partition the reserve into sub-cells and estimate
   $\lambda(x,y)$ from the spatial distribution of detections using kernel density estimation;
   apply a spatially varying correction factor to reduce bias under strong clustering.
3. **Multi-altitude comparison**: vary flight altitude $h \in \{20, 40, 80\}$ m and model
   $r_{half}(h) = k\,h$ (sensor FOV scaling); show the trade-off between wider footprint (faster
   coverage, higher CV) and narrower footprint (more strips, lower CV per strip).
4. **Transect line observer model**: replace the drone-nadir sensor with a camera with a
   cross-track detection function fit to real distance-sampling data (half-normal, hazard-rate);
   compare estimator performance to the Gaussian roll-off baseline.
5. **Movement during survey**: allow animals to perform a correlated random walk during the
   flight; quantify the additional bias introduced when the survey assumption of stationary
   targets is violated.
6. **Multi-species simultaneous census**: deploy two drones with different sensor wavelengths
   (thermal for nocturnal species, optical for diurnal); plan non-overlapping lawnmower bands
   and combine per-species distance-sampling estimates into a joint population report.

---

## Related Scenarios

- Prerequisites: [S041 Wildfire Boundary Scan](S041_wildfire_boundary.md), [S048 Full-Area Coverage Scan (Lawnmower)](S048_lawnmower.md), [S042 Missing Person Localization](S042_missing_person.md)
- Follow-ups: [S058 Coral Reef Mapping](S058_coral_reef.md), [S059 Glacier Retreat Monitoring](S059_glacier_retreat.md)
- Algorithmic cross-reference: [S013 Particle Filter Intercept](../01_pursuit_evasion/S013_particle_filter_intercept.md) (probabilistic detection and estimation), [S067 Spray Overlap Optimization](../../04_industrial_agriculture/S067_spray_overlap_optimization.md) (strip-width trade-off in coverage planning)
