# S056 Nuclear Plant Leak Detection (Radiation Hotspot Localization)

**Domain**: Environmental Monitoring & SAR | **Difficulty**: ⭐⭐⭐ | **Status**: `[ ]` Not Started

---

## Problem Definition

**Setup**: Following a suspected leak at an industrial nuclear facility, a $500 \times 500$ m
exclusion zone has been declared. Ground teams cannot enter until the source is localised. A
single drone equipped with a **dose-rate detector** (e.g., Geiger–Müller tube or scintillator) is
deployed to survey the area and identify the position of the radiation source(s). The survey area
is discretised into a $50 \times 50$ grid of $10$ m cells. The drone carries out an
**adaptive measurement campaign**: it flies to one cell centre at a time, records a dose-rate
reading $z_k$, then updates a Bayesian probability map over all candidate source cells. The next
waypoint is chosen to maximise **expected information gain** — the cell whose measurement is
predicted to reduce map entropy the most. The mission terminates when the maximum posterior
probability exceeds a confidence threshold $P_{conf} = 0.90$, or the budget of $N_{meas} = 80$
measurements is exhausted.

The true source position is drawn uniformly within the grid at the start of each trial.
Measurement noise accounts for detector counting statistics (Poisson / Gaussian approximation)
and background radiation $D_{bg}$.

**Roles**:
- **Drone** (single agent): carries the dose-rate sensor, navigates at a fixed survey altitude
  $h = 15$ m, flies straight-line segments between successive waypoints at cruise speed $v = 8$ m/s.
- **Radiation source**: one point source of unknown position $\mathbf{p}_s^*$ with unknown
  strength $A^*$; treated as a hidden state to be inferred.

**Objective**: Localise the radiation source position $\hat{\mathbf{p}}_s$ to within a root-mean-
square error $\epsilon_{RMS} \leq 10$ m using the fewest possible measurements, while minimising
the **total flight path length** (proxy for drone radiation exposure).

**Comparison strategies**:
1. **Greedy maximum information gain** — next waypoint is $\arg\max_{\mathbf{x}} \,
   \text{EIG}(\mathbf{x})$; the primary strategy.
2. **Greedy maximum belief** — next waypoint is the cell with the highest current posterior
   probability; simpler but less principled than EIG.
3. **Lawnmower sweep** — systematic boustrophedon coverage, ignoring measurement feedback;
   baseline for comparison.

---

## Mathematical Model

### Radiation Field Model

Let $\mathbf{p} = (x, y)$ be a 2D position at the survey altitude $h$ and
$\mathbf{p}_s = (x_s, y_s)$ be the source position (ground level, projected to the horizontal
plane). The slant range from the drone to the source is:

$$r(\mathbf{p}, \mathbf{p}_s) = \sqrt{\|\mathbf{p} - \mathbf{p}_s\|^2 + h^2}$$

The **expected dose rate** (µSv/h) at position $\mathbf{p}$ due to a point source of strength
$A$ (µSv·m²/h) follows the inverse-square law with exponential attenuation in air:

$$D(\mathbf{p},\, \mathbf{p}_s,\, A) = D_{bg} + \frac{A}{r(\mathbf{p}, \mathbf{p}_s)^2}
  \exp\!\bigl(-\mu \cdot r(\mathbf{p}, \mathbf{p}_s)\bigr)$$

where $\mu$ (m$^{-1}$) is the linear attenuation coefficient of air and $D_{bg}$ (µSv/h) is
the ambient background dose rate.

### Measurement Model

Dose-rate detectors operate by counting ionisation events; for integration time $\Delta t_{int}$
the measured count is Poisson-distributed. In the Gaussian approximation, the $k$-th measurement
at drone position $\mathbf{p}_k$ is:

$$z_k = D(\mathbf{p}_k,\, \mathbf{p}_s^*,\, A^*) + \varepsilon_k, \qquad
  \varepsilon_k \sim \mathcal{N}(0,\, \sigma_{rad}^2)$$

where the measurement noise standard deviation $\sigma_{rad}$ scales with the true dose rate
(reflecting Poisson counting statistics):

$$\sigma_{rad}(\mathbf{p}_k) = \sigma_0 \sqrt{D(\mathbf{p}_k,\, \mathbf{p}_s^*,\, A^*) + D_{bg}}$$

with $\sigma_0$ (µSv$^{1/2}$·h$^{1/2}$) a detector-dependent noise coefficient.

### Bayesian Source Localization

The source position is treated as a discrete latent variable over the grid
$\mathcal{G} = \{g_1, \ldots, g_{N_c}\}$ with $N_c = 2500$ cells. Source strength $A$ is
assumed known (or marginalised over a discrete set); for the primary model $A = A^*$ is fixed.

**Prior**: uniform over all cells,

$$P_0(g_j) = \frac{1}{N_c}, \quad \forall\, j$$

**Likelihood** of observation $z_k$ given source at cell $g_j$:

$$P(z_k \mid g_j) = \frac{1}{\sqrt{2\pi}\,\hat{\sigma}_{rad}(g_j)}
  \exp\!\left(-\frac{\bigl(z_k - D(\mathbf{p}_k, g_j, A)\bigr)^2}{2\,\hat{\sigma}_{rad}^2(g_j)}\right)$$

where $\hat{\sigma}_{rad}(g_j) = \sigma_0\sqrt{D(\mathbf{p}_k, g_j, A) + D_{bg}}$ uses the
model-predicted dose rate at cell $g_j$.

**Sequential Bayesian update** after measurement $k$:

$$P_k(g_j) = \frac{P(z_k \mid g_j) \cdot P_{k-1}(g_j)}{\displaystyle\sum_{l=1}^{N_c}
  P(z_k \mid g_l) \cdot P_{k-1}(g_l)}, \quad \forall\, j$$

This is applied at each step; after $k$ measurements the posterior $P_k$ encodes all information
from $z_1, \ldots, z_k$.

### Point Estimate and Localization Error

The MAP estimate of the source position after $k$ measurements:

$$\hat{\mathbf{p}}_s^{MAP} = g_{\hat{j}}, \qquad \hat{j} = \arg\max_j\, P_k(g_j)$$

The posterior mean estimate:

$$\hat{\mathbf{p}}_s^{mean} = \sum_{j=1}^{N_c} P_k(g_j) \cdot g_j$$

Localisation RMS error over $N_{trial}$ Monte Carlo trials:

$$\epsilon_{RMS} = \sqrt{\frac{1}{N_{trial}} \sum_{t=1}^{N_{trial}}
  \|\hat{\mathbf{p}}_s^{(t)} - \mathbf{p}_s^{*(t)}\|^2}$$

### Belief Map Entropy

Shannon entropy of the current posterior over grid cells:

$$H_k = -\sum_{j=1}^{N_c} P_k(g_j) \log P_k(g_j)$$

Initial entropy: $H_0 = \log N_c \approx 7.82$ nats. The mission aims to reduce $H_k$ as fast
as possible. The entropy reduction per measurement is used to compare strategies.

### Expected Information Gain (EIG) Waypoint Selection

Before committing to candidate position $\mathbf{x}$, compute the expected reduction in
posterior entropy if a measurement were taken there. Using the current posterior $P_{k}$ as a
predictive distribution over possible readings $\tilde{z}$:

**Predictive mean** of a hypothetical measurement at $\mathbf{x}$:

$$\bar{z}(\mathbf{x}) = \sum_{j=1}^{N_c} P_k(g_j) \cdot D(\mathbf{x},\, g_j,\, A)$$

**Predictive variance**:

$$\text{Var}(\tilde{z} \mid \mathbf{x}) = \sum_{j=1}^{N_c} P_k(g_j)
  \bigl[D(\mathbf{x},\, g_j,\, A) - \bar{z}(\mathbf{x})\bigr]^2
  + \bar{\sigma}_{rad}^2(\mathbf{x})$$

where $\bar{\sigma}_{rad}^2(\mathbf{x}) = \sigma_0^2 \sum_j P_k(g_j)\bigl[D(\mathbf{x}, g_j, A)
+ D_{bg}\bigr]$ is the expected noise variance.

**Approximate EIG** via the Gaussian mutual information approximation:

$$\text{EIG}(\mathbf{x}) = \frac{1}{2} \log\!\left(1 +
  \frac{\displaystyle\sum_j P_k(g_j)\bigl[D(\mathbf{x}, g_j, A) - \bar{z}(\mathbf{x})\bigr]^2}
  {\bar{\sigma}_{rad}^2(\mathbf{x})}\right)$$

This is the expected reduction in differential entropy of the predictive distribution; it is high
when candidate positions yield readings that strongly discriminate between hypotheses.

**Next waypoint selection**:

$$\mathbf{x}_{k+1}^* = \arg\max_{\mathbf{x} \in \mathcal{G}}\; \text{EIG}(\mathbf{x})$$

Cells already visited within the last $N_{cooldown} = 5$ steps are excluded to promote spatial
diversity.

### Drone Kinematics

Point-mass, constant-speed flight between waypoints at survey altitude $h$:

$$\dot{\mathbf{p}}_{drone} = v \cdot \hat{\mathbf{r}}, \qquad
  \hat{\mathbf{r}} = \frac{\mathbf{x}_{next} - \mathbf{p}_{drone}}
  {\|\mathbf{x}_{next} - \mathbf{p}_{drone}\|}$$

Time to travel between successive waypoints $\mathbf{x}_k$ and $\mathbf{x}_{k+1}$:

$$\Delta t_{travel} = \frac{\|\mathbf{x}_{k+1} - \mathbf{x}_k\|}{v}$$

Hover dwell at each waypoint for measurement integration:

$$\Delta t_{dwell} = \Delta t_{int} = 5 \text{ s}$$

Total mission time: $T_{mission} = \sum_{k=1}^{N_{meas}} (\Delta t_{travel,k} + \Delta t_{int})$.

---

## Implementation

```python
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import LogNorm

# Key constants
AREA_SIZE    = 500.0      # m — square exclusion zone side
GRID_N       = 50         # cells per axis
CELL_SIZE    = AREA_SIZE / GRID_N   # 10 m
H_SURVEY     = 15.0       # m — drone altitude above ground
V_DRONE      = 8.0        # m/s — cruise speed
DT_INT       = 5.0        # s — measurement dwell per waypoint
N_MEAS_MAX   = 80         # maximum measurements per mission
P_CONF       = 0.90       # posterior peak threshold — stop condition
N_COOLDOWN   = 5          # waypoints excluded from re-visit
A_SOURCE     = 2000.0     # µSv·m²/h — source strength
MU_ATTEN     = 0.0002     # m^-1 — air attenuation coefficient
D_BG         = 0.10       # µSv/h — ambient background
SIGMA_0      = 0.04       # µSv^0.5·h^0.5 — detector noise coefficient
N_TRIALS     = 50         # Monte Carlo trials for RMS error

# Build cell centres grid
xs = (np.arange(GRID_N) + 0.5) * CELL_SIZE
ys = (np.arange(GRID_N) + 0.5) * CELL_SIZE
GX, GY = np.meshgrid(xs, ys, indexing='ij')   # shape (50, 50)
CELLS = np.stack([GX.ravel(), GY.ravel()], axis=-1)  # (2500, 2)

def slant_range(p_drone, cell_xy):
    """3D slant range from drone (2D horiz) to ground cell centre."""
    horiz = np.linalg.norm(p_drone[:2] - cell_xy, axis=-1)
    return np.sqrt(horiz**2 + H_SURVEY**2)

def dose_rate(p_drone, cell_xy, A=A_SOURCE):
    """Expected dose rate at drone position due to source at cell_xy."""
    r = slant_range(p_drone, cell_xy)
    return D_BG + (A / r**2) * np.exp(-MU_ATTEN * r)

def measure(p_drone, true_source_xy, rng):
    """Simulate a noisy dose-rate measurement."""
    d_true = dose_rate(p_drone, true_source_xy)
    sigma = SIGMA_0 * np.sqrt(d_true)
    return d_true + rng.normal(0.0, sigma)

def bayesian_update(belief, z, p_drone, A=A_SOURCE):
    """Update belief (N_c,) with new measurement z at drone position p_drone."""
    d_pred = dose_rate(p_drone, CELLS, A)          # (N_c,)
    sigma_pred = SIGMA_0 * np.sqrt(d_pred)
    log_lik = -0.5 * ((z - d_pred) / sigma_pred)**2 - np.log(sigma_pred)
    log_belief = np.log(belief + 1e-300) + log_lik
    log_belief -= log_belief.max()
    belief_new = np.exp(log_belief)
    return belief_new / belief_new.sum()

def eig_map(belief, A=A_SOURCE):
    """Compute approximate EIG for every grid cell under current belief."""
    d_pred = dose_rate(CELLS[:, np.newaxis, :],
                       CELLS[np.newaxis, :, :], A)     # (N_c, N_c)
    # predictive mean at each candidate cell c: sum_j belief[j] * d_pred[c,j]
    z_bar = (belief[np.newaxis, :] * d_pred).sum(axis=1)     # (N_c,)
    var_model = (belief[np.newaxis, :] * (d_pred - z_bar[:, np.newaxis])**2
                 ).sum(axis=1)                               # (N_c,)
    sigma_noise_sq = (SIGMA_0**2 * (belief[np.newaxis, :]
                      * (d_pred + D_BG)).sum(axis=1))        # (N_c,)
    eig = 0.5 * np.log1p(var_model / (sigma_noise_sq + 1e-9))
    return eig                                               # (N_c,)

def entropy(belief):
    b = belief[belief > 0]
    return -np.sum(b * np.log(b))

def run_eig_mission(true_source_xy, rng):
    """Run one EIG-guided mission; return measurements, belief history, path."""
    belief = np.ones(len(CELLS)) / len(CELLS)
    pos = np.array([AREA_SIZE / 2, AREA_SIZE / 2])  # start at centre
    path = [pos.copy()]
    entropies = [entropy(belief)]
    recent = []

    for k in range(N_MEAS_MAX):
        # Termination check
        if belief.max() >= P_CONF:
            break

        # Compute EIG; mask recently visited cells
        eig = eig_map(belief)
        if recent:
            eig[recent] = -np.inf

        next_idx = int(np.argmax(eig))
        next_pos = CELLS[next_idx]

        # Fly to next_pos (update path)
        pos = next_pos.copy()
        path.append(pos.copy())

        # Measure and update
        z = measure(pos, true_source_xy, rng)
        belief = bayesian_update(belief, z, pos)
        entropies.append(entropy(belief))

        recent.append(next_idx)
        if len(recent) > N_COOLDOWN:
            recent.pop(0)

    # MAP estimate
    map_idx = int(np.argmax(belief))
    p_est = CELLS[map_idx]
    return np.array(path), belief, np.array(entropies), p_est

def run_simulation():
    rng = np.random.default_rng(42)
    # Single illustrative run
    true_xy = np.array([175.0, 310.0])
    path, belief, entropies, p_est = run_eig_mission(true_xy, rng)

    rms_err = np.linalg.norm(p_est - true_xy)
    print(f"Measurements used : {len(path) - 1}")
    print(f"Final peak belief  : {belief.max():.4f}")
    print(f"MAP estimate       : {p_est}")
    print(f"True source        : {true_xy}")
    print(f"Localisation error : {rms_err:.1f} m")

    return path, belief.reshape(GRID_N, GRID_N), entropies, true_xy, p_est
```

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Survey area | 500 × 500 m |
| Grid resolution | 50 × 50 cells (10 m/cell) |
| Grid cells $N_c$ | 2 500 |
| Survey altitude $h$ | 15 m |
| Drone cruise speed $v$ | 8 m/s |
| Measurement dwell $\Delta t_{int}$ | 5 s |
| Max measurements $N_{meas}$ | 80 |
| Confidence stop threshold $P_{conf}$ | 0.90 |
| Source strength $A$ | 2 000 µSv·m²/h |
| Air attenuation $\mu$ | 2 × 10$^{-4}$ m$^{-1}$ |
| Background dose rate $D_{bg}$ | 0.10 µSv/h |
| Detector noise coefficient $\sigma_0$ | 0.04 µSv$^{0.5}$·h$^{0.5}$ |
| Cooldown window $N_{cooldown}$ | 5 waypoints |
| Monte Carlo trials $N_{trial}$ | 50 |
| Initial entropy $H_0$ | $\ln(2500) \approx 7.82$ nats |
| Target localisation RMS | $\leq$ 10 m |

---

## Expected Output

- **Belief map sequence**: 2D top-down heatmaps of the posterior $P_k(g_j)$ at measurement steps
  $k = 1, 5, 20, 50$; colour scale log-normalised; true source marked with a white cross, MAP
  estimate with a red dot; drone path overlaid as a cyan trail.
- **EIG map**: snapshot of $\text{EIG}(\mathbf{x})$ at $k = 10$, showing the information
  landscape that drives the next waypoint choice; highest-EIG cell highlighted.
- **Entropy decay curve**: $H_k$ vs measurement index $k$ for all three strategies (EIG, max-
  belief greedy, lawnmower); EIG should show fastest entropy reduction.
- **Localisation error vs measurements**: $\epsilon_{RMS}(k)$ averaged over $N_{trial} = 50$
  Monte Carlo trials for each strategy; with 95 % confidence bands.
- **Dose-rate field**: background image of the true dose-rate field $D(\mathbf{p})$ over the
  500 × 500 m grid at $h = 15$ m, shown on log scale; contour rings at 0.5, 1, 5, 20 µSv/h.
- **Flight path comparison**: top-down overlay of cumulative flight paths for all three strategies
  after 80 measurements; total path length printed in the legend.
- **Animation (GIF)**: step-by-step update of the belief map and drone position; belief heatmap
  fades from blue (uniform) toward red (concentrated); waypoint sequence shown as growing cyan
  line; current measurement reading displayed in the title bar.

---

## Extensions

1. **Multiple sources**: extend to $N_s = 2$ or $3$ point sources; reformulate the posterior
   over a joint source-position hypothesis space (or use a mixture model with EM updates);
   compare single-source model mismatch vs correctly specified multi-source model.
2. **Unknown source strength**: treat $A$ as an additional latent variable; marginalise over a
   discrete grid of $A$ values $[500, 1000, 2000, 5000]$ µSv·m²/h; assess how strength
   uncertainty widens the localisation confidence ellipse.
3. **Wind-driven dispersion plume**: replace the point-source field with a Gaussian dispersion
   plume model (wind speed $u$, diffusion $\kappa_y$, $\kappa_z$); the drone must simultaneously
   estimate source position, emission rate, and wind vector.
4. **Dose accumulation constraint**: enforce a maximum cumulative dose budget for the drone
   (proxy for electronics hardness); penalise waypoints near the source when remaining budget is
   low; trade off information gain vs self-protection.
5. **Multi-drone parallel survey**: deploy $N = 3$ drones sharing a common belief map; implement
   a decentralised EIG policy with waypoint de-confliction (no two drones assigned the same
   candidate in the same step) to prevent redundant measurements.
6. **Particle filter extension**: replace the discrete grid posterior with a particle filter
   ($M = 5000$ particles) for continuous-space source localisation; compare accuracy and
   computational cost against the grid Bayesian approach.

---

## Related Scenarios

- Prerequisites: [S041 Wildfire Boundary](S041_wildfire_boundary.md), [S042 Missing Person Localization](S042_missing_person.md)
- Follow-ups: [S057](S057_oil_spill_mapping.md) (contaminant mapping), [S058](S058_gas_leak.md) (gas plume tracing)
- Algorithmic cross-reference: [S042 Missing Person Localization](S042_missing_person.md) (shared Bayesian update / entropy-reduction framework), [S044 Wall Crack Detection](S044_wall_crack.md) (adaptive inspection path planning)
