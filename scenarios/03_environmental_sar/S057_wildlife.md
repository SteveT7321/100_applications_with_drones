# S057 Wildlife Population Census

**Domain**: Environmental Monitoring & SAR | **Difficulty**: ⭐⭐ | **Status**: `[ ]` Not Started

---

## Problem Definition

**Setup**: A single survey drone systematically photographs a $500 \times 500$ m wildlife reserve
to estimate the total population of a ground-dwelling species (e.g., wildebeest, deer). Animals
are distributed in $G = 20$ spatially clustered groups; each group $g$ occupies a centroid
$\mathbf{c}_g$ with a Gaussian spread $\sigma_g = 8$ m and contains $n_g \in [1, 20]$ individuals.
The drone executes a **lawnmower (boustrophedon) scan pattern** at a chosen constant altitude $h$
and ground speed $v$. Its downward-facing camera has a fixed half-angle field of view
$\theta_{FOV} = 25°$, producing a ground-level strip of width $w(h) = 2h\tan(\theta_{FOV})$ on
each pass. Detection of any individual within the scan strip is probabilistic: the probability
declines with altitude (sensor resolution) and speed (motion blur), and is reduced by a per-group
camouflage factor $c_g \in [0, 1]$.

The core trade-off is **altitude vs. speed**: flying low and slow maximises per-pass detection
probability but increases mission time; flying high and fast covers the area quickly but misses
more animals. The scenario compares three fixed operating points and reports estimated vs. true
counts.

**Roles**:
- **Survey drone**: single agent executing a pre-planned lawnmower trajectory at fixed $(h, v)$.
- **Animal groups** ($G = 20$): static clusters with known ground-truth size $n_g$, random
  centroid positions, and individual camouflage ratings $c_g$.

**Objective**: Estimate total population $\hat{N}$ and compare to ground truth $N^* = \sum_g n_g$.
Evaluate three operating strategies:

1. **High-altitude fast** — $h = 40$ m, $v = 12$ m/s (wide strip, low $P_d$, short mission).
2. **Mid-altitude medium** — $h = 20$ m, $v = 6$ m/s (balanced).
3. **Low-altitude slow** — $h = 8$ m, $v = 2$ m/s (narrow strip, high $P_d$, long mission).

Report for each strategy: estimated count $\hat{N}$, miss count, double-count rate, mission
duration $T_{mission}$, and counting error $\epsilon = |\hat{N} - N^*| / N^*$.

---

## Mathematical Model

### Scan Strip and Coverage

At altitude $h$ with half-angle FOV $\theta_{FOV}$, each lawnmower pass sweeps a strip of width:

$$w(h) = 2\,h\,\tan(\theta_{FOV})$$

The number of parallel passes required to cover the $W = 500$ m wide reserve (no overlap) is:

$$n_{passes} = \left\lceil \frac{W}{w(h)} \right\rceil$$

Total lawnmower path length (including turn legs, neglecting turn radius):

$$L_{total}(h) = n_{passes} \cdot W + (n_{passes} - 1) \cdot w(h)$$

Mission duration at ground speed $v$:

$$T_{mission}(h, v) = \frac{L_{total}(h)}{v}$$

### Detection Probability Model

For a drone flying at altitude $h$ and ground speed $v$, the probability of detecting a single
animal that falls within the scan strip footprint is:

$$P_d(h, v, c_g) = P_{d0} \cdot \exp(-\alpha\,h) \cdot \exp(-\beta\,v) \cdot (1 - c_g)$$

where:
- $P_{d0} = 0.95$ — baseline detection probability at ground level, zero speed, no camouflage.
- $\alpha = 0.04$ m$^{-1}$ — altitude decay coefficient (sensor resolution degradation).
- $\beta = 0.08$ s/m — speed decay coefficient (motion blur at image capture).
- $c_g \in [0, 1]$ — camouflage factor for group $g$ (0 = fully visible, 1 = invisible).

### Expected Detections per Group

Group $g$ is in the scan footprint of pass $j$ if any individual from that group lies within the
strip boundaries. Individual $i$ of group $g$ is at position:

$$\mathbf{x}_{g,i} = \mathbf{c}_g + \boldsymbol{\epsilon}_{g,i}, \qquad
\boldsymbol{\epsilon}_{g,i} \sim \mathcal{N}(\mathbf{0},\, \sigma_g^2 \mathbf{I})$$

Let $\mathcal{F}_j(h)$ denote the footprint rectangle of pass $j$ (a $500 \times w(h)$ m strip).
The indicator $\mathbb{1}[g \in j]$ is 1 if $\mathbf{c}_g$ projects into strip $j$'s lateral
extent. The expected number of individuals detected from group $g$ on pass $j$ is:

$$E[\hat{n}_{g,j}] = n_g \cdot \mathbb{1}[g \in j] \cdot P_d(h, v, c_g)$$

Summing over all passes and groups gives the expected total count:

$$E[\hat{N}] = \sum_{g=1}^{G} \sum_{j=1}^{n_{passes}} E[\hat{n}_{g,j}]$$

In simulation each individual detection is drawn as an independent Bernoulli trial:

$$\hat{n}_{g,j} = \sum_{i=1}^{n_g} \mathbb{1}[i \in \mathcal{F}_j] \cdot \text{Bernoulli}(P_d(h, v, c_g))$$

### Miss Rate and Double-Count Rate

Miss rate (fraction of animals present in footprint but undetected):

$$R_{miss}(h, v, c_g) = 1 - P_d(h, v, c_g)$$

If adjacent passes overlap (overlap fraction $\delta > 0$), an animal near the strip edge may be
detected twice. For a strip overlap $\delta = \max(0,\; 2w - W/n_{passes}) / w$, the probability
an animal in the overlap zone is counted on both passes:

$$P_{double} = P_d^2(h, v, c_g) \cdot \delta$$

The expected number of double-counts contributed by group $g$:

$$E[D_g] = n_g \cdot \mathbb{1}[g \text{ in overlap zone}] \cdot P_{double}$$

### Population Estimate and Error

The raw simulation count $\hat{N}_{raw}$ sums all individual Bernoulli detections across all
passes, including double-counts. The corrected estimate applies a standard strip-transect
correction for detection probability:

$$\hat{N}_{corrected} = \frac{\hat{N}_{raw}}{\overline{P}_d(h, v)}$$

where $\overline{P}_d(h, v) = P_{d0} \cdot \exp(-\alpha h) \cdot \exp(-\beta v) \cdot (1 - \bar{c})$
uses the mean camouflage $\bar{c} = \frac{1}{G}\sum_g c_g$.

Relative counting error:

$$\epsilon = \frac{|\hat{N}_{corrected} - N^*|}{N^*}$$

---

## Implementation

```python
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

# Key constants
AREA_SIZE    = 500.0      # m — reserve side length
N_GROUPS     = 20         # number of animal clusters
N_ANIMALS_RANGE = (1, 20) # animals per group
SIGMA_GROUP  = 8.0        # m — intra-group spatial spread
THETA_FOV    = 25.0       # degrees — camera half-angle FOV
P_D0         = 0.95       # baseline detection probability
ALPHA        = 0.04       # m^-1 — altitude decay
BETA         = 0.08       # s/m — speed decay
N_MONTE      = 50         # Monte Carlo trials per strategy

# Three operating strategies: (altitude m, speed m/s, label)
STRATEGIES = [
    (40.0, 12.0, "High-alt fast"),
    (20.0,  6.0, "Mid-alt medium"),
    ( 8.0,  2.0, "Low-alt slow"),
]

def strip_width(h):
    return 2.0 * h * np.tan(np.radians(THETA_FOV))

def mission_duration(h, v):
    w = strip_width(h)
    n_passes = int(np.ceil(AREA_SIZE / w))
    path_len = n_passes * AREA_SIZE + (n_passes - 1) * w
    return path_len / v, n_passes, w

def detection_prob(h, v, camouflage):
    return P_D0 * np.exp(-ALPHA * h) * np.exp(-BETA * v) * (1.0 - camouflage)

def generate_reserve(rng):
    """Randomly place G animal groups with sizes and camouflage factors."""
    centroids   = rng.uniform(20, AREA_SIZE - 20, size=(N_GROUPS, 2))
    group_sizes = rng.integers(*N_ANIMALS_RANGE, size=N_GROUPS)
    camouflage  = rng.uniform(0.0, 0.6, size=N_GROUPS)
    return centroids, group_sizes, camouflage

def simulate_census(h, v, centroids, group_sizes, camouflage, rng):
    """Run one lawnmower census; return raw detection count and per-group info."""
    w = strip_width(h)
    n_passes = int(np.ceil(AREA_SIZE / w))
    strip_centres_y = np.array([w * (j + 0.5) for j in range(n_passes)])

    total_detected = 0
    total_missed   = 0

    for g in range(N_GROUPS):
        pd = detection_prob(h, v, camouflage[g])
        # Sample individual positions for this group
        positions = centroids[g] + rng.normal(0, SIGMA_GROUP, size=(group_sizes[g], 2))
        positions = np.clip(positions, 0, AREA_SIZE)

        seen_this_group = np.zeros(group_sizes[g], dtype=bool)

        for j, cy in enumerate(strip_centres_y):
            y_lo = cy - w / 2.0
            y_hi = cy + w / 2.0
            in_strip = (positions[:, 1] >= y_lo) & (positions[:, 1] < y_hi)
            for idx in np.where(in_strip)[0]:
                if rng.random() < pd:
                    total_detected += 1
                    seen_this_group[idx] = True

        total_missed += np.sum(~seen_this_group)

    return total_detected, total_missed

def run_simulation():
    rng = np.random.default_rng(seed=42)
    centroids, group_sizes, camouflage = generate_reserve(rng)
    N_true = int(group_sizes.sum())

    results = {}
    for (h, v, label) in STRATEGIES:
        T_mission, n_passes, w = mission_duration(h, v)
        pd_mean = detection_prob(h, v, float(np.mean(camouflage)))

        counts_raw = []
        counts_corr = []
        misses = []
        for _ in range(N_MONTE):
            raw, missed = simulate_census(h, v, centroids, group_sizes, camouflage, rng)
            corr = raw / pd_mean if pd_mean > 0 else raw
            counts_raw.append(raw)
            counts_corr.append(corr)
            misses.append(missed)

        results[label] = {
            "h": h, "v": v,
            "strip_width": w,
            "n_passes": n_passes,
            "T_mission": T_mission,
            "pd_mean": pd_mean,
            "raw_mean": np.mean(counts_raw),
            "corr_mean": np.mean(counts_corr),
            "corr_std": np.std(counts_corr),
            "miss_mean": np.mean(misses),
            "error": abs(np.mean(counts_corr) - N_true) / N_true,
        }

    return results, N_true, centroids, group_sizes, camouflage
```

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Reserve area | 500 × 500 m |
| Animal groups $G$ | 20 |
| Animals per group $n_g$ | 1 – 20 (uniform random) |
| Intra-group spread $\sigma_g$ | 8 m |
| Camera half-angle $\theta_{FOV}$ | 25° |
| Baseline detection $P_{d0}$ | 0.95 |
| Altitude decay $\alpha$ | 0.04 m$^{-1}$ |
| Speed decay $\beta$ | 0.08 s/m |
| Camouflage factor $c_g$ | 0.0 – 0.6 (uniform random) |
| High-alt fast $(h, v)$ | 40 m, 12 m/s |
| Mid-alt medium $(h, v)$ | 20 m, 6 m/s |
| Low-alt slow $(h, v)$ | 8 m, 2 m/s |
| Strip width at 40 m / 20 m / 8 m | ~37 m / ~19 m / ~7 m |
| Monte Carlo trials per strategy | 50 |

---

## Expected Output

- **Reserve map**: 2D top-down scatter plot of the $500 \times 500$ m reserve showing all animal
  group centroids sized by $n_g$ and colour-coded by camouflage factor; lawnmower strip boundaries
  overlaid as dashed lines for the mid-altitude strategy.
- **Detection probability surface**: 2D heatmap of $P_d(h, v, \bar{c})$ over an $h \in [5, 50]$
  m vs. $v \in [1, 15]$ m/s grid, with the three operating points marked.
- **Count distribution**: box-and-whisker plots of corrected count $\hat{N}_{corrected}$ across
  Monte Carlo trials for each strategy; true count $N^*$ shown as a red dashed horizontal line.
- **Miss vs. mission time scatter**: scatter plot with each strategy as a labelled point, x-axis =
  $T_{mission}$ (s), y-axis = mean miss count; Pareto-front curve connecting the strategies.
- **Strategy comparison table**: printed summary of strip width, number of passes, mission time,
  mean $P_d$, mean raw count, mean corrected count, miss rate, and relative error $\epsilon$ for
  all three strategies.
- **Animation (GIF)**: top-down view of the drone executing the mid-altitude lawnmower scan,
  animal groups revealed as they enter the camera footprint (filled circles = detected,
  hollow circles = undetected so far).

---

## Extensions

1. **Adaptive altitude control**: allow the drone to vary $h$ per strip based on a prior density
   map of expected animal locations — descend over high-probability clusters, climb over sparse
   terrain — and measure the improvement in $\epsilon$ against fixed-altitude baselines.
2. **Multi-drone parallel sweeps**: deploy $N = 3$ drones on interleaved lawnmower lanes to
   reduce $T_{mission}$; implement lane-assignment to minimise overlap while maintaining full
   coverage and study the throughput vs. coordination overhead trade-off.
3. **Moving animals**: introduce a random-walk drift $\mathbf{v}_{drift} \sim \mathcal{N}(\mathbf{0},
   \sigma_v^2 \mathbf{I})$ per timestep; quantify how animal motion inflates double-counts and
   degrades the strip-transect correction.
4. **Camera sensor model with false positives**: add a false-alarm rate $P_{fa}$ (e.g., rocks or
   shadows misclassified as animals); derive a likelihood-ratio test threshold that balances
   $P_d$ and $P_{fa}$, and assess impact on population estimate bias.
5. **RL altitude policy**: train a PPO agent to select $h_j$ for each lawnmower pass $j$ based
   on a running partial count map; evaluate against the fixed-altitude oracle on reserves with
   heterogeneous camouflage distributions.

---

## Related Scenarios

- Prerequisites: [S041 Wildfire Boundary](S041_wildfire_boundary.md), [S042 Missing Person Localization](S042_missing_person.md)
- Follow-ups: [S058](S058_flood_mapping.md) (area coverage with sensor fusion), [S059](S059_oil_spill_detection.md) (contaminant census)
- Algorithmic cross-reference: [S044 Wall Crack Inspection](S044_wall_crack.md) (sensor footprint geometry), [S042 Missing Person Localization](S042_missing_person.md) (probabilistic detection model)
