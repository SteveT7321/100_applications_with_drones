# S051 Post-Disaster Communication Network Restoration

**Domain**: Environmental & SAR | **Difficulty**: ⭐⭐⭐ | **Status**: `[ ]` Not Started

---

## Problem Definition

**Setup**: A major earthquake has destroyed ground-based cellular infrastructure across a
$2000 \times 2000$ m urban area. A population density map divides the area into $G = 40 \times 40$
grid cells; each cell $u$ has a known population weight $\text{pop}_u \geq 0$ (people). A fleet of
$K = 5$ drones is available at a single charge station $\mathbf{p}_{base}$. Each drone can hover
at a fixed position and act as a temporary airborne cell tower, covering all population within a
ground-projected radius $r_{cov} = 300$ m. Drones must remain airborne for the duration of the
mission; the battery constraint limits total hover time to $T_{max} = 1800$ s before each drone
must return to charge.

**Roles**:
- **Charge station** (base): single fixed point $\mathbf{p}_{base}$; all drones depart from and
  return to this point for recharging.
- **Drones** ($K = 5$): each drone $k$ hovers at position $\mathbf{p}_{d_k} \in \mathbb{R}^2$ at
  a fixed altitude $z = 50$ m; ground-projected coverage radius is $r_{cov}$.
- **Population grid** ($G$ cells): each cell $u$ has centroid $\mathbf{c}_u$ and weight
  $\text{pop}_u$; a cell is served if at least one drone is within $r_{cov}$.

**Objective**: Jointly optimise the $K$ drone hover positions
$\{\mathbf{p}_{d_1}, \ldots, \mathbf{p}_{d_K}\}$ to **maximise total covered population** subject
to:

1. The $K$ drone positions form a **connected communication graph** (adjacent drones within radio
   range $r_{comm} = 600$ m) so that each drone can relay data back to the charge station.
2. Each drone can reach its assigned hover position and return to base within the battery time
   budget $T_{max}$, given transit speed $v = 15$ m/s.

**Comparison strategies**:
1. **Greedy placement** — sequentially place each drone at the grid cell centroid with the highest
   uncovered population, without enforcing connectivity.
2. **Simulated annealing (SA)** — optimise positions from a random initialisation, accepting
   connectivity-violating moves with decreasing probability; connectivity enforced as a hard
   penalty.
3. **Greedy + connectivity repair** — greedy placement followed by a Steiner-tree-inspired
   reconnection pass that repositions disconnected drones to restore the relay chain.

---

## Mathematical Model

### Coverage Indicator and Objective

Let $\mathcal{U}$ be the set of all grid cells, $\mathcal{D} = \{1, \ldots, K\}$ be the set of
drone indices, and $\mathbf{p}_{d_k} \in \mathbb{R}^2$ be the ground-projected hover position of
drone $k$.

Coverage indicator for cell $u$ by drone $k$:

$$I(u, k) = \begin{cases} 1 & \text{if } \|\mathbf{c}_u - \mathbf{p}_{d_k}\| \leq r_{cov} \\ 0 & \text{otherwise} \end{cases}$$

Total covered population (objective to maximise):

$$C\!\left(\{\mathbf{p}_{d_k}\}\right) = \sum_{u \in \mathcal{U}} \text{pop}_u \cdot \mathbf{1}\!\left[\max_{k \in \mathcal{D}} I(u, k) \geq 1\right]$$

Equivalently, using union coverage:

$$C = \sum_{u \in \mathcal{U}} \text{pop}_u \cdot \mathbf{1}\!\left[\bigcup_{k=1}^{K} \left\{\|\mathbf{c}_u - \mathbf{p}_{d_k}\| \leq r_{cov}\right\}\right]$$

### Connectivity Constraint

Build an undirected graph $\mathcal{G} = (\mathcal{V}, \mathcal{E})$ where
$\mathcal{V} = \{0\} \cup \mathcal{D}$, node $0$ represents the charge station, and edge
$(i, j) \in \mathcal{E}$ exists if:

$$\|\mathbf{p}_i - \mathbf{p}_j\| \leq r_{comm}$$

The connectivity constraint requires $\mathcal{G}$ to be **connected** (there exists a path from
every drone $k$ to node $0$). Equivalently, the graph's Laplacian matrix $\mathbf{L}$ must have
exactly one zero eigenvalue:

$$\lambda_2(\mathbf{L}) > 0 \qquad \text{(algebraic connectivity)}$$

In the simulated annealing cost function the connectivity violation is penalised as:

$$\text{penalty}(\mathcal{G}) = \alpha \cdot \bigl(K + 1 - |\text{nodes in largest connected component containing node 0}|\bigr)$$

with penalty weight $\alpha$ chosen large enough to dominate the coverage term when any drone is
disconnected.

### Battery (Range) Constraint

Drone $k$ must transit from base to hover position and back within $T_{max}$:

$$\frac{2 \|\mathbf{p}_{d_k} - \mathbf{p}_{base}\|}{v} + T_{hover} \leq T_{max}$$

where $T_{hover}$ is the intended hover duration. Rearranging, the maximum reachable distance from
base is:

$$D_{max} = \frac{v \cdot (T_{max} - T_{hover})}{2}$$

Hover positions outside $D_{max}$ are infeasible and receive an additional hard penalty.

### Simulated Annealing

State: continuous positions $\mathbf{x} = [\mathbf{p}_{d_1}^\top, \ldots, \mathbf{p}_{d_K}^\top]^\top \in \mathbb{R}^{2K}$.

Cost function (to minimise):

$$E(\mathbf{x}) = -C(\mathbf{x}) + \text{penalty}(\mathcal{G}(\mathbf{x}))$$

Perturbation at each SA step: randomly select one drone $k$ and displace it by
$\Delta \mathbf{p} \sim \mathcal{U}([-\delta, \delta]^2)$ where $\delta$ decreases with temperature.

Acceptance probability for a move $\mathbf{x} \to \mathbf{x}'$:

$$P_{accept} = \min\!\left(1,\; \exp\!\left(-\frac{E(\mathbf{x}') - E(\mathbf{x})}{T_{SA}}\right)\right)$$

Temperature schedule (geometric cooling):

$$T_{SA}^{(n+1)} = \beta \cdot T_{SA}^{(n)}, \qquad \beta = 0.995, \quad T_{SA}^{(0)} = T_0$$

### Greedy Connectivity Repair

After greedy placement, for each disconnected drone $k$ (not reachable from node $0$ in
$\mathcal{G}$):

1. Find the nearest drone $k^*$ that **is** connected to node $0$.
2. Compute the midpoint $\mathbf{m} = \frac{1}{2}(\mathbf{p}_{d_k} + \mathbf{p}_{d_{k^*}})$.
3. Relocate $k$ to $\mathbf{m}$ if $\|\mathbf{p}_{d_k} - \mathbf{m}\| \leq r_{comm}$ and
   $\|\mathbf{m} - \mathbf{p}_{d_{k^*}}\| \leq r_{comm}$; otherwise step $k$ toward $k^*$ by
   $r_{comm} - \epsilon$.
4. Recompute coverage loss and accept if coverage decrease is below threshold $\Delta C_{tol}$.

### Set Cover Relaxation (Lower Bound)

The weighted maximum coverage problem is NP-hard in general. A greedy $(1 - 1/e)$-approximation
bound applies when each drone covers a fixed circular region and the cells are discrete:

$$C_{greedy} \geq \left(1 - \frac{1}{e}\right) \cdot C_{OPT} \approx 0.632 \cdot C_{OPT}$$

This bound is used to evaluate how close the SA solution is to the theoretical optimum.

---

## Implementation

```python
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import distance_matrix
import networkx as nx

# Key constants
K_DRONES   = 5          # number of relay drones
AREA_SIZE  = 2000.0     # m — side length of square area
GRID_N     = 40         # grid resolution per axis
R_COV      = 300.0      # m — coverage radius per drone
R_COMM     = 600.0      # m — inter-drone communication radius
V_DRONE    = 15.0       # m/s — transit speed
T_MAX      = 1800.0     # s — total battery budget
T_HOVER    = 1500.0     # s — intended hover duration
ALPHA      = 1e6        # connectivity penalty weight
T0_SA      = 500.0      # initial SA temperature
BETA_SA    = 0.995      # SA cooling rate
N_SA_STEPS = 80_000     # total SA iterations
DELTA_MAX  = 200.0      # m — max perturbation at T0

BASE_POS   = np.array([1000.0, 100.0])   # charge station position

# Derived feasibility radius
D_MAX = V_DRONE * (T_MAX - T_HOVER) / 2.0   # = 2250 m > AREA_SIZE/2 => all positions reachable

def build_population_map(grid_n, area_size, seed=42):
    """Generate synthetic population density with 3 urban clusters."""
    rng = np.random.default_rng(seed)
    # Cell centroids
    cell_size = area_size / grid_n
    xs = np.linspace(cell_size / 2, area_size - cell_size / 2, grid_n)
    ys = np.linspace(cell_size / 2, area_size - cell_size / 2, grid_n)
    cx, cy = np.meshgrid(xs, ys)
    centroids = np.stack([cx.ravel(), cy.ravel()], axis=1)  # (N, 2)

    # Three population clusters
    pop = np.zeros(grid_n * grid_n)
    clusters = [(400, 1600, 8000, 350), (1200, 1200, 12000, 400), (1700, 700, 6000, 250)]
    for (mx, my, peak, sigma) in clusters:
        d2 = (centroids[:, 0] - mx)**2 + (centroids[:, 1] - my)**2
        pop += peak * np.exp(-d2 / (2 * sigma**2))
    pop += rng.uniform(0, 200, size=pop.shape)   # background noise
    return centroids, pop

def coverage(positions, centroids, pop, r_cov):
    """Total covered population for given drone hover positions."""
    # positions: (K, 2); centroids: (N, 2)
    dists = distance_matrix(centroids, positions)          # (N, K)
    covered_mask = np.any(dists <= r_cov, axis=1)          # (N,)
    return np.sum(pop[covered_mask])

def connectivity_penalty(positions, base_pos, r_comm, alpha):
    """Penalty for disconnected relay graph."""
    nodes = np.vstack([base_pos, positions])               # (K+1, 2)
    dists = distance_matrix(nodes, nodes)
    adj = dists <= r_comm
    G = nx.from_numpy_array(adj.astype(int))
    component = nx.node_connected_component(G, 0)
    n_connected = len(component)
    n_total = len(nodes)
    return alpha * (n_total - n_connected)

def sa_cost(positions, centroids, pop, r_cov, base_pos, r_comm, alpha):
    cov = coverage(positions, centroids, pop, r_cov)
    pen = connectivity_penalty(positions, base_pos, r_comm, alpha)
    return -cov + pen

def simulated_annealing(centroids, pop, base_pos, k_drones,
                        r_cov, r_comm, alpha, area_size,
                        t0, beta, n_steps):
    """SA optimisation of drone hover positions."""
    rng = np.random.default_rng(0)
    # Initialise positions randomly within area
    positions = rng.uniform(0, area_size, size=(k_drones, 2))
    best_pos = positions.copy()
    best_cost = sa_cost(positions, centroids, pop, r_cov, base_pos, r_comm, alpha)
    current_cost = best_cost
    T = t0

    for step in range(n_steps):
        k = rng.integers(k_drones)
        delta = rng.uniform(-DELTA_MAX, DELTA_MAX, size=2) * (T / t0)
        new_pos = positions.copy()
        new_pos[k] = np.clip(new_pos[k] + delta, 0, area_size)
        new_cost = sa_cost(new_pos, centroids, pop, r_cov, base_pos, r_comm, alpha)
        dE = new_cost - current_cost
        if dE < 0 or rng.random() < np.exp(-dE / T):
            positions = new_pos
            current_cost = new_cost
            if current_cost < best_cost:
                best_cost = current_cost
                best_pos = positions.copy()
        T *= beta

    return best_pos, best_cost

def greedy_placement(centroids, pop, base_pos, k_drones, r_cov):
    """Sequential greedy placement: maximise marginal covered population."""
    positions = []
    remaining_pop = pop.copy()
    for _ in range(k_drones):
        best_cov = -1
        best_pos = None
        for c in centroids:
            d = np.linalg.norm(centroids - c, axis=1)
            new_cov = np.sum(remaining_pop[d <= r_cov])
            if new_cov > best_cov:
                best_cov = new_cov
                best_pos = c.copy()
        positions.append(best_pos)
        d = np.linalg.norm(centroids - best_pos, axis=1)
        remaining_pop[d <= r_cov] = 0.0
    return np.array(positions)

def run_simulation():
    centroids, pop = build_population_map(GRID_N, AREA_SIZE)
    total_pop = np.sum(pop)

    # Strategy 1: greedy (no connectivity)
    greedy_pos = greedy_placement(centroids, pop, BASE_POS, K_DRONES, R_COV)
    greedy_cov = coverage(greedy_pos, centroids, pop, R_COV)

    # Strategy 2: SA with connectivity penalty
    sa_pos, _ = simulated_annealing(
        centroids, pop, BASE_POS, K_DRONES, R_COV, R_COMM,
        ALPHA, AREA_SIZE, T0_SA, BETA_SA, N_SA_STEPS
    )
    sa_cov = coverage(sa_pos, centroids, pop, R_COV)

    results = {
        'greedy': (greedy_pos, greedy_cov / total_pop),
        'sa': (sa_pos, sa_cov / total_pop),
    }
    return centroids, pop, results
```

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Drone fleet size $K$ | 5 |
| Area | 2000 × 2000 m |
| Population grid resolution | 40 × 40 cells |
| Coverage radius $r_{cov}$ | 300 m |
| Communication radius $r_{comm}$ | 600 m |
| Hover altitude | 50 m |
| Transit speed $v$ | 15 m/s |
| Battery horizon $T_{max}$ | 1800 s |
| Intended hover duration $T_{hover}$ | 1500 s |
| Max reachable distance $D_{max}$ | 2250 m |
| Connectivity penalty weight $\alpha$ | $10^6$ |
| SA initial temperature $T_0$ | 500 |
| SA cooling rate $\beta$ | 0.995 |
| SA iterations | 80 000 |
| Population clusters | 3 (peaks at 8 000, 12 000, 6 000 persons) |
| Charge station $\mathbf{p}_{base}$ | (1000, 100) m |
| Greedy approximation ratio | $\geq 1 - 1/e \approx 0.632$ |

---

## Expected Output

- **Population density map**: 2D heatmap of $\text{pop}_u$ across the grid; urban cluster peaks
  labelled; charge station marked with a black star.
- **Coverage comparison map**: side-by-side top-down plots for each strategy showing drone hover
  positions (coloured circles with radius $r_{cov}$), communication edges (dashed lines between
  drones within $r_{comm}$), and coloured grid cells indicating covered vs uncovered population.
- **Communication graph overlay**: networkx-drawn relay graph superimposed on the map; connected
  components highlighted in different colours; disconnected drones flagged in red.
- **Coverage fraction bar chart**: total covered population as a percentage of total population for
  each strategy (Greedy, SA, Greedy+Repair); horizontal reference line at the $(1-1/e)$ greedy
  bound.
- **SA convergence curve**: cost $E$ (negative coverage + penalty) vs SA iteration number; penalty
  trace overlaid to show when connectivity is first achieved and maintained.
- **Sensitivity analysis**: coverage fraction vs $r_{cov}$ (250 – 450 m) and vs $K$ (1 – 8 drones)
  for the SA solution; plotted as two line charts.
- **Animation (GIF)**: SA optimisation progress over iterations — drone positions move across the
  map as the algorithm converges, covered region shading updating in real time.

---

## Extensions

1. **Time-varying population**: population density changes throughout the day (e.g., daytime
   concentration in commercial zones, nighttime in residential zones); optimise a hover-position
   schedule with at most one repositioning per drone per hour.
2. **Heterogeneous coverage radii**: drones carry different antenna configurations giving
   $r_{cov,k} \in \{200, 300, 400\}$ m; incorporate the asymmetric disk model into the SA cost.
3. **Relay chain bandwidth**: model data throughput $B_{link} \propto r_{comm}^{-2}$ along each
   relay edge; add a minimum-bandwidth constraint so that the bottleneck link meets a threshold
   $B_{min}$.
4. **Simultaneous coverage and relay charging**: drones hover in a rotating schedule —
   one drone at a time returns to base for recharging while the remaining $K-1$ maintain coverage;
   minimise coverage gap during the rotation cycle.
5. **RL placement policy**: train a PPO agent to place $K$ drones sequentially given the
   population map as a 2D input feature; compare sample efficiency and solution quality against SA
   on held-out city layouts.
6. **3D building shadow model**: tall buildings block ground-level signal; extend coverage
   indicator to account for line-of-sight obstructions using a city building height map.

---

## Related Scenarios

- Prerequisites: [S047 Signal Relay Enhancement](S047_signal_relay_enhancement.md), [S048 Lawnmower Coverage](S048_lawnmower_coverage.md), [S049 Dynamic Zone Assignment](S049_dynamic_zone_assignment.md)
- Follow-ups: [S052 Glacier Area Monitoring](S052_glacier_area_monitoring.md) (persistent multi-drone coverage)
- Algorithmic cross-reference: [S029 Urban Logistics Scheduling](../02_logistics_delivery/S029_urban_logistics_scheduling.md) (multi-agent area assignment), [S015 Communication Relay Tracking](../01_pursuit_evasion/S015_comm_relay_tracking.md) (relay chain geometry)
