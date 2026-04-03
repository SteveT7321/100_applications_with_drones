# S030 Multi-Depot Delivery

**Domain**: Logistics & Delivery | **Difficulty**: ⭐⭐⭐ | **Status**: `[ ]` Not started

---

## Problem Definition

**Setup**: A drone delivery network consists of $D = 4$ depots with heterogeneous fleet sizes and
storage capacities, spread across a $1200 \times 1200$ m operational area. A total of $N = 20$
customer orders must be fulfilled. Each drone may start at any depot in its home set and — crucially —
is permitted to **return to a different depot** than its origin (open-route MD-VRP). This flexibility
allows drones to avoid long back-tracking legs when the last delivery is closer to another depot.

**Roles**:
- **Depots** $\mathcal{D} = \{d_1, d_2, d_3, d_4\}$: each has a fleet of $F_m$ drones and a remaining
  inventory capacity $I_m$ (total kg available for dispatch); a drone may depart any depot $d_m$
  with up to $\min(Q_{drone}, I_m)$ kg of cargo.
- **Drones** $\mathcal{K}$ (10 total, distributed across depots): each carries at most $Q = 3.0$ kg
  per sortie and has a maximum range of $R_{max} = 3500$ m.
- **Customers** $\mathcal{C} = \{c_1, \ldots, c_{20}\}$: each has a demand $q_c \in [0.3, 1.8]$ kg
  and a preferred service depot (soft constraint only).

**Objective**: Partition the customer set across the four depots and sequence each drone's route so as to:
1. Minimise total fleet flight distance (primary)
2. Satisfy per-drone range and payload constraints
3. Balance the number of deliveries across depots (secondary, fairness term)

**Comparison strategies**:
1. **Nearest-depot partition** — assign each customer to the geometrically closest depot, then solve
   a single-depot VRP per depot using a nearest-neighbour heuristic
2. **K-Means partition** — cluster customers into $D$ clusters whose centroids are attracted toward
   depot positions; assign each cluster to its nearest depot
3. **Alternating optimisation (MD-VRP AO)** — iteratively reassign boundary customers to whichever
   depot reduces total route cost, then re-optimise each sub-fleet's tours with 2-opt, until
   convergence

---

## Mathematical Model

### Network and Cost Matrix

Let $\mathcal{N} = \mathcal{D} \cup \mathcal{C}$ be all nodes, $|\mathcal{N}| = D + N$.
The Euclidean distance between nodes $i$ and $j$:

$$d_{ij} = \|\mathbf{p}_i - \mathbf{p}_j\|_2$$

Travel time at cruising speed $v$:

$$t_{ij} = \frac{d_{ij}}{v}$$

### Multi-Depot VRP Formulation

Decision variables:
- $x_{ijk} \in \{0,1\}$ — drone $k$ (originating at depot $\delta(k)$) traverses arc $(i,j)$
- $y_{km} \in \{0,1\}$ — drone $k$ returns to depot $m$ (open-route return assignment)

**Objective** (distance minimisation with load-balance penalty):

$$\min \; \sum_{k \in \mathcal{K}} \sum_{i,j \in \mathcal{N}} d_{ij} \cdot x_{ijk}
  \;+\; \lambda \cdot \sum_{m \in \mathcal{D}} \left( n_m - \bar{n} \right)^2$$

where $n_m = \sum_{k: \delta(k)=m} |\text{route}_k|$ is the number of deliveries dispatched from
depot $m$ and $\bar{n} = N/D$ is the ideal balanced load; $\lambda$ is the balance weight.

**Capacity constraint** per route:

$$\sum_{c \in \mathcal{C}} q_c \cdot \sum_j x_{cjk} \leq Q \quad \forall k \in \mathcal{K}$$

**Range constraint** per route (including the final return leg to whichever depot):

$$\sum_{i,j \in \mathcal{N}} d_{ij} \cdot x_{ijk}
  + \min_{m \in \mathcal{D}} d_{last_k, m} \cdot y_{km} \leq R_{max} \quad \forall k$$

where $last_k$ is the final customer node visited by drone $k$.

**Open-route return**: drone $k$ returns to exactly one depot:

$$\sum_{m \in \mathcal{D}} y_{km} = 1 \quad \forall k \in \mathcal{K}$$

**Inventory constraint** at depot $m$:

$$\sum_{k: \delta(k)=m} \sum_{c} q_c \cdot \sum_j x_{cjk} \leq I_m \quad \forall m \in \mathcal{D}$$

**Flow conservation** at each customer node:

$$\sum_{k} \sum_j x_{cjk} = 1 \quad \forall c \in \mathcal{C}$$

$$\sum_j x_{ick} = \sum_j x_{cjk} \quad \forall c \in \mathcal{C},\; k \in \mathcal{K}$$

### Customer Partition via Alternating Optimisation

Define the partition $\Pi = \{\Pi_1, \Pi_2, \Pi_3, \Pi_4\}$ where $\Pi_m \subseteq \mathcal{C}$ and
$\bigcup_m \Pi_m = \mathcal{C}$, $\Pi_m \cap \Pi_{m'} = \emptyset$.

**Assignment step** — for each boundary customer $c$ (adjacent to two partition regions), compute the
marginal cost of moving $c$ from partition $\Pi_m$ to $\Pi_{m'}$:

$$\Delta_{m \to m'} = \text{cost}(\Pi_{m'} \cup \{c\}) + \text{cost}(\Pi_m \setminus \{c\})
  - \text{cost}(\Pi_m) - \text{cost}(\Pi_{m'})$$

Move $c$ if $\Delta_{m \to m'} < 0$ and feasibility is preserved.

**Route optimisation step** — for each depot sub-fleet, improve tours with 2-opt:

Given a route sequence $\sigma = (v_0, v_1, \ldots, v_n, v_0)$ (where $v_0$ is the origin depot),
a 2-opt swap reverses the sub-sequence $[v_i, v_{i+1}, \ldots, v_j]$:

$$\text{gain}(i,j) = d_{v_{i-1},v_i} + d_{v_j,v_{j+1}} - d_{v_{i-1},v_j} - d_{v_i,v_{j+1}}$$

Accept the swap when $\text{gain}(i,j) > 0$. Repeat until no improving swap exists.

**Convergence criterion**:

$$\left| \text{TotalDist}^{(t+1)} - \text{TotalDist}^{(t)} \right| < \varepsilon_{AO}$$

### Savings Initialisation (Clarke-Wright, per depot)

For each depot $d_m$, compute inter-customer savings to seed initial tours:

$$S_{ij}^{(m)} = d_{d_m, i} + d_{d_m, j} - d_{ij} \quad \forall i,j \in \Pi_m,\; i \neq j$$

Merge routes greedily in decreasing order of $S_{ij}^{(m)}$, subject to capacity and range.

### Open-Route Return Selection

After drone $k$ completes its final delivery at node $last_k$, it flies to the nearest depot with
available docking capacity $A_m > 0$:

$$m^*_k = \arg\min_{m \in \mathcal{D}} d_{last_k, m} \quad \text{s.t. } A_m > 0$$

This produces a return distance saving relative to forced home-base return:

$$\Delta R_k = d_{last_k, \delta(k)} - d_{last_k, m^*_k}$$

Total open-route saving across the fleet:

$$\Delta R_{total} = \sum_{k \in \mathcal{K}} \Delta R_k$$

### Simulation Kinematics

Each drone flies constant-speed legs. Position at time $t$ during leg from node $i$ to node $j$
(leg start time $t_i^{(k)}$, leg duration $\tau_{ij} = d_{ij}/v$):

$$\mathbf{p}_k(t) = \mathbf{p}_i + v \cdot (t - t_i^{(k)}) \cdot \hat{\mathbf{r}}_{ij}, \quad
  \hat{\mathbf{r}}_{ij} = \frac{\mathbf{p}_j - \mathbf{p}_i}{d_{ij}}$$

Accumulated range consumed after leg $(i,j)$:

$$R_k^{used} \leftarrow R_k^{used} + d_{ij}$$

---

## Implementation

```python
import numpy as np
from scipy.spatial.distance import cdist
from itertools import combinations

# Key constants
N_DEPOTS    = 4
N_CUSTOMERS = 20
N_DRONES    = 10          # total fleet
V_DRONE     = 14.0        # m/s cruising speed
Q_MAX       = 3.0         # kg payload capacity per drone
R_MAX       = 3500.0      # m maximum range per sortie
SVC_TIME    = 8.0         # s service time per customer stop
LAMBDA      = 50.0        # balance penalty weight (m^2 per delivery^2)
EPS_AO      = 1.0         # m convergence threshold for AO loop
DT          = 0.5         # s simulation timestep

# Fixed depot positions and properties
DEPOTS = np.array([
    [150.,  150.],   # D1 — SW
    [150.,  1050.],  # D2 — NW
    [1050., 150.],   # D3 — SE
    [1050., 1050.],  # D4 — NE
])
FLEET_SIZE  = np.array([3, 2, 3, 2])   # drones per depot
INVENTORY   = np.array([25., 18., 25., 20.])  # kg total available per depot

# Random customers (fixed seed)
rng = np.random.default_rng(7)
CUSTOMERS = rng.uniform(100, 1100, (N_CUSTOMERS, 2))
DEMANDS   = rng.uniform(0.3, 1.8, N_CUSTOMERS)

# --- Nearest-depot partition ---
def nearest_depot_partition(customers, depots):
    depot_dist = cdist(customers, depots)
    return np.argmin(depot_dist, axis=1)   # partition[c] = depot index

# --- 2-opt improvement for a single depot's routes ---
def two_opt(route, dist_matrix):
    """Improve a single-drone route with 2-opt; route includes depot at [0] and [-1]."""
    improved = True
    while improved:
        improved = False
        for i in range(1, len(route) - 2):
            for j in range(i + 1, len(route) - 1):
                gain = (dist_matrix[route[i-1], route[i]]
                        + dist_matrix[route[j],   route[j+1]]
                        - dist_matrix[route[i-1], route[j]]
                        - dist_matrix[route[i],   route[j+1]])
                if gain > 1e-6:
                    route[i:j+1] = route[i:j+1][::-1]
                    improved = True
    return route

# --- Open-route return assignment ---
def best_return_depot(last_node_pos, depots, avail_slots):
    dists = np.linalg.norm(depots - last_node_pos, axis=1)
    dists[avail_slots == 0] = np.inf     # exclude full depots
    return int(np.argmin(dists))

# --- Alternating optimisation main loop ---
def alternating_optimisation(customers, demands, depots, fleet_size,
                              q_max, r_max, lambda_bal, eps):
    partition = nearest_depot_partition(customers, depots)
    prev_cost = np.inf
    for iteration in range(50):
        # Route optimisation step: 2-opt per depot
        routes, total_dist = build_and_improve_routes(
            customers, demands, depots, partition, fleet_size, q_max, r_max)
        # Balance penalty
        n_per_depot = np.bincount(partition, minlength=N_DEPOTS).astype(float)
        n_bar = N_CUSTOMERS / N_DEPOTS
        cost = total_dist + lambda_bal * np.sum((n_per_depot - n_bar) ** 2)
        if abs(prev_cost - cost) < eps:
            break
        prev_cost = cost
        # Assignment step: try moving boundary customers
        for c in range(len(customers)):
            m = partition[c]
            for m2 in range(N_DEPOTS):
                if m2 == m:
                    continue
                partition[c] = m2
                routes2, td2 = build_and_improve_routes(
                    customers, demands, depots, partition, fleet_size, q_max, r_max)
                n2 = np.bincount(partition, minlength=N_DEPOTS).astype(float)
                cost2 = td2 + lambda_bal * np.sum((n2 - n_bar) ** 2)
                if cost2 < cost:
                    cost = cost2
                    m = m2
                    break
                else:
                    partition[c] = m   # revert
    return partition, routes, cost

def route_distance(route_nodes, all_positions):
    """Total distance for a sequence of node indices (positions lookup)."""
    return sum(
        np.linalg.norm(all_positions[route_nodes[k+1]] - all_positions[route_nodes[k]])
        for k in range(len(route_nodes) - 1)
    )

def build_and_improve_routes(customers, demands, depots, partition,
                              fleet_size, q_max, r_max):
    """Build Clarke-Wright initial routes per depot, then 2-opt improve."""
    # Node index mapping: depots 0..D-1, customers D..D+N-1
    all_pos = np.vstack([depots, customers])
    D = len(depots)
    total_dist = 0.0
    all_routes = []
    for m in range(D):
        cust_idxs = [c + D for c in range(len(customers)) if partition[c] == m]
        depot_node = m
        if not cust_idxs:
            all_routes.append([])
            continue
        # Split into capacity-feasible sub-routes (greedy bin-packing)
        sub_routes = []
        current_route = [depot_node]
        current_load = 0.0
        current_dist = 0.0
        # Nearest-neighbour ordering from depot
        remaining = list(cust_idxs)
        pos_now = all_pos[depot_node]
        while remaining:
            dists = [np.linalg.norm(all_pos[c] - pos_now) for c in remaining]
            nearest = remaining[int(np.argmin(dists))]
            demand_c = demands[nearest - D]
            leg = np.linalg.norm(all_pos[nearest] - pos_now)
            return_leg = np.linalg.norm(all_pos[nearest] - all_pos[depot_node])
            if (current_load + demand_c <= q_max
                    and current_dist + leg + return_leg <= r_max):
                current_route.append(nearest)
                current_load += demand_c
                current_dist += leg
                pos_now = all_pos[nearest]
                remaining.remove(nearest)
            else:
                current_route.append(depot_node)
                sub_routes.append(current_route)
                current_route = [depot_node]
                current_load  = 0.0
                current_dist  = 0.0
                pos_now = all_pos[depot_node]
        current_route.append(depot_node)
        sub_routes.append(current_route)
        # 2-opt each sub-route
        dist_matrix = cdist(all_pos, all_pos)
        for i, sr in enumerate(sub_routes):
            sub_routes[i] = two_opt(sr, dist_matrix)
            total_dist += route_distance(sr, all_pos)
        all_routes.append(sub_routes)
    return all_routes, total_dist
```

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Depots | 4, corners of a 1200 m grid: (150,150), (150,1050), (1050,150), (1050,1050) |
| Fleet distribution | [3, 2, 3, 2] drones per depot = 10 total |
| Depot inventory | [25, 18, 25, 20] kg available |
| Customers | 20, random in [100, 1100] m grid, seed 7 |
| Demand per customer | 0.3 – 1.8 kg |
| Drone payload $Q$ | 3.0 kg |
| Max range $R_{max}$ | 3500 m per sortie |
| Cruising speed $v$ | 14 m/s |
| Service time per stop | 8 s |
| Balance weight $\lambda$ | 50 m² / delivery² |
| AO convergence $\varepsilon_{AO}$ | 1.0 m |
| Max AO iterations | 50 |
| Open-route return | enabled (drone returns to nearest depot) |
| Simulation timestep $\Delta t$ | 0.5 s |

---

## Expected Output

- Top-down 2D map: depot markers (coloured triangles by depot index), customer markers coloured by
  assigned depot, drone routes as line segments, open-return arcs as dashed lines to a different depot
- Bar chart: total fleet flight distance for Nearest-Depot vs K-Means vs AO strategies
- Bar chart: deliveries-per-depot distribution for each strategy (showing load balance)
- Line plot: AO objective (total distance + balance penalty) vs iteration number (convergence curve)
- Table: per-drone summary — origin depot, return depot, route length, payload used, range used
- Scalar summary: total open-route return distance saving $\Delta R_{total}$ vs forced home-base return
- Animated GIF: all 10 drones flying simultaneously, colour-coded by originating depot

---

## Extensions

1. Heterogeneous drone speeds — assign faster drones to depots with larger service areas; revisit
   the savings computation with drone-specific arc costs
2. Dynamic depot failure — mid-mission removal of one depot forces re-routing of its in-flight drones
   to neighbouring depots; benchmark re-optimisation latency
3. Time-window integration (S029 combined) — augment the MD-VRP with per-customer time windows,
   yielding the full MD-VRPTW; solve with OR-Tools CP-SAT
4. Depot replenishment — inventory at each depot is replenished by a supply truck on a known schedule;
   model as a time-indexed availability constraint
5. 3D extension — depots at different altitudes (rooftop vs ground); add altitude-change energy cost
   $E_{alt} = m \cdot g \cdot |\Delta z|$ to the arc cost and re-derive AO assignment step
6. Stochastic demand — customer demands are Gaussian; use sample average approximation (SAA) within
   the AO loop to obtain robust routes under uncertainty

---

## Related Scenarios

- Prerequisites: [S021](S021_point_delivery.md), [S029](S029_urban_logistics_scheduling.md)
- Follow-ups: [S031](S031_path_deconfliction.md), [S033](S033_online_order_insertion.md)
- Range-constrained delivery: [S027](S027_aerial_refueling_relay.md), [S036](S036_last_mile_relay.md)
- Algorithmic cross-reference: [S019](../01_pursuit_evasion/S019_dynamic_reassignment.md) (Hungarian
  assignment), [S018](../01_pursuit_evasion/S018_multi_target_intercept.md) (TSP tour building)

## References

- Cordeau, J.-F., Gendreau, M. & Laporte, G. (1997). "A tabu search heuristic for periodic and
  multi-depot vehicle routing problems." *Networks*, 30(2), 105–119.
- Lin, S. & Kernighan, B.W. (1973). "An Effective Heuristic Algorithm for the
  Travelling-Salesman Problem." *Operations Research*, 21(2), 498–516.
- Clarke, G. & Wright, J.W. (1964). "Scheduling of Vehicles from a Central Depot to a Number of
  Delivery Points." *Operations Research*, 12(4), 568–581.
- Montoya-Torres, J.R. et al. (2015). "A literature review on the vehicle routing problem with
  multiple depots." *Computers & Industrial Engineering*, 79, 115–129.
