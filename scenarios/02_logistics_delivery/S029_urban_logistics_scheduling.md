# S029 Urban Logistics Scheduling

**Domain**: Logistics & Delivery | **Difficulty**: ⭐⭐⭐⭐ | **Status**: `[ ]` Not started

---

## Problem Definition

**Setup**: An urban delivery network contains $D = 3$ depots and $C = 12$ customer delivery points
scattered across a $1000 \times 1000$ m grid. A heterogeneous fleet of $K = 6$ drones is distributed
across the depots (2 per depot). Each drone has a maximum payload capacity $Q$ and a maximum flight
range $R_{max}$ (battery energy equivalent). Each customer order has a demand weight $q_c$ and a
hard time-window $[e_c, l_c]$ within which delivery must occur.

**Objective**: Assign customers to drones and sequence each drone's route so that:
1. Total weighted tardiness is minimised (primary objective)
2. Total flight distance is minimised (secondary objective, lexicographic or weighted sum)
3. All capacity and range constraints are satisfied

**Comparison strategies**:
1. **Greedy nearest-depot** — each order assigned to the closest depot drone, nearest-neighbour tour
2. **Clarke-Wright Savings** — merge individual routes by savings criterion
3. **VRP with Time Windows (VRPTW)** — full formulation solved by OR-Tools CP-SAT

---

## Mathematical Model

### Fleet and Network

Let $\mathcal{D} = \{d_1, d_2, d_3\}$ be depot positions, $\mathcal{C} = \{c_1, \ldots, c_{12}\}$
customer positions, and $\mathcal{K} = \{1, \ldots, 6\}$ drone indices with $k \in \mathcal{K}$
originating from depot $\delta(k)$.

Travel time between nodes $i$ and $j$:

$$t_{ij} = \frac{\|\mathbf{p}_i - \mathbf{p}_j\|}{v_k}$$

### Vehicle Routing Problem with Time Windows (VRPTW)

Decision variables:

- $x_{ijk} \in \{0,1\}$ — drone $k$ traverses arc $(i, j)$
- $s_{ik} \geq 0$ — service start time of drone $k$ at node $i$

**Objective** (weighted sum of tardiness and total distance):

$$\min \; \alpha \sum_{k \in \mathcal{K}} \sum_{c \in \mathcal{C}} w_c \cdot \tau_{ck}
  + (1-\alpha) \sum_{k \in \mathcal{K}} \sum_{i,j} d_{ij} \cdot x_{ijk}$$

where tardiness $\tau_{ck} = \max(0,\; s_{ck} - l_c) \cdot x_{ick}$ for some $i$.

**Capacity constraint** (each drone's total loaded weight per route):

$$\sum_{c \in \mathcal{C}} q_c \cdot \sum_j x_{cjk} \leq Q \quad \forall k \in \mathcal{K}$$

**Range constraint** (total arc distance per route):

$$\sum_{i,j} d_{ij} \cdot x_{ijk} \leq R_{max} \quad \forall k \in \mathcal{K}$$

**Time-window constraint** (arrival no earlier than $e_c$, service start no later than $l_c$):

$$e_c \leq s_{ck} \leq l_c \quad \forall c \in \mathcal{C},\; k \in \mathcal{K}$$

**Arrival time propagation** (with service time $\tau^{svc}$ at each stop):

$$s_{jk} \geq s_{ik} + \tau^{svc} + t_{ij} - M(1 - x_{ijk}) \quad \forall i,j,k$$

where $M$ is a large constant (big-M linearisation).

**Flow conservation** (each customer visited exactly once):

$$\sum_{k \in \mathcal{K}} \sum_j x_{cjk} = 1 \quad \forall c \in \mathcal{C}$$

$$\sum_j x_{ijk} = \sum_j x_{jik} \quad \forall i \in \mathcal{C},\; k \in \mathcal{K}$$

### Clarke-Wright Savings

Initial solution: each customer served by a dedicated drone directly from its nearest depot.
Savings from merging routes $r_a$ (serving $i$) and $r_b$ (serving $j$) into a combined route:

$$S_{ij} = d_{d_a, i} + d_{d_b, j} - d_{ij}$$

Merge greedily in decreasing order of $S_{ij}$, subject to capacity and range feasibility.

### Time-Window Penalty

For a solution with service times $\{s_{ck}\}$, the total weighted tardiness:

$$\text{TWT} = \sum_{c \in \mathcal{C}} w_c \cdot \max\!\bigl(0,\; s_c - l_c\bigr)$$

where $s_c = s_{ck}$ for the unique drone $k$ assigned to $c$.

### Route Execution Kinematics

Each drone flies at constant cruising speed $v_k$ toward successive waypoints.
Position at time $t$ during leg from $\mathbf{p}_i$ to $\mathbf{p}_j$ (leg start time $t_i$):

$$\mathbf{p}_k(t) = \mathbf{p}_i + v_k \cdot (t - t_i) \cdot \hat{\mathbf{r}}_{ij}, \qquad
  \hat{\mathbf{r}}_{ij} = \frac{\mathbf{p}_j - \mathbf{p}_i}{\|\mathbf{p}_j - \mathbf{p}_i\|}$$

Remaining range after completing leg $(i,j)$:

$$R_{rem} \leftarrow R_{rem} - \|\mathbf{p}_j - \mathbf{p}_i\|$$

---

## Implementation

```python
import numpy as np
from scipy.spatial.distance import cdist
from itertools import combinations

# Key constants
N_DEPOTS    = 3
N_CUSTOMERS = 12
N_DRONES    = 6          # 2 per depot
V_DRONE     = 15.0       # m/s cruising speed (all drones homogeneous)
Q_MAX       = 3.0        # kg payload capacity per drone
R_MAX       = 4000.0     # m maximum range per sortie
SVC_TIME    = 10.0       # s service (landing + drop + takeoff) at each customer
ALPHA       = 0.7        # tardiness vs distance weight
DT          = 0.5        # s simulation timestep

# Fixed depot positions
DEPOTS = np.array([
    [100., 100., 2.],
    [500., 900., 2.],
    [900., 200., 2.],
])

# Random seed customers with time windows
rng = np.random.default_rng(42)
CUSTOMERS = rng.uniform(50, 950, (N_CUSTOMERS, 2))
CUSTOMERS = np.c_[CUSTOMERS, np.full(N_CUSTOMERS, 2.0)]

# Time windows: [earliest, latest] in seconds from t=0
EARLY = rng.uniform(0, 120, N_CUSTOMERS)
LATE  = EARLY + rng.uniform(60, 180, N_CUSTOMERS)
DEMAND = rng.uniform(0.3, 1.5, N_CUSTOMERS)   # kg

# --- Clarke-Wright Savings ---
def cw_savings(depots, customers, demands, drone_depot_map, q_max, r_max, v):
    """Return list of routes (lists of customer indices) per drone."""
    # Build initial single-customer routes, each from nearest depot
    depot_dist = cdist(customers[:, :2], depots[:, :2])
    nearest_depot = np.argmin(depot_dist, axis=1)

    routes = [[c] for c in range(len(customers))]     # one route per customer
    route_depot = [nearest_depot[c] for c in range(len(customers))]

    # Compute all pairwise savings
    savings = []
    for i, j in combinations(range(len(customers)), 2):
        if route_depot[i] != route_depot[j]:
            continue          # only merge routes from same depot
        d_di = depot_dist[i, route_depot[i]]
        d_dj = depot_dist[j, route_depot[j]]
        d_ij = np.linalg.norm(customers[i, :2] - customers[j, :2])
        s = d_di + d_dj - d_ij
        savings.append((s, i, j))
    savings.sort(reverse=True)

    # Greedy merge
    for s, i, j in savings:
        ri = next((r for r in routes if i in r), None)
        rj = next((r for r in routes if j in r), None)
        if ri is None or rj is None or ri is rj:
            continue
        # Feasibility checks
        merged = ri + rj
        total_demand = sum(demands[c] for c in merged)
        total_dist = route_length(merged, depots[route_depot[ri]], customers, v)
        if total_demand <= q_max and total_dist <= r_max:
            ri.extend(rj)
            routes.remove(rj)
    return routes

def route_length(route, depot, customers, v):
    pts = [depot[:2]] + [customers[c, :2] for c in route] + [depot[:2]]
    return sum(np.linalg.norm(pts[k+1] - pts[k]) for k in range(len(pts)-1))

# --- Simulation execution ---
def simulate_routes(routes, drone_assignments, depots, customers, early, late, v, svc_time, dt):
    """Fly each drone along its assigned route; record positions and delivery times."""
    history = {k: [] for k in range(N_DRONES)}
    delivery_times = {}

    for k, route in enumerate(drone_assignments):
        depot_idx = k // 2
        pos = depots[depot_idx].copy()
        t = 0.0
        for c in route:
            target = customers[c]
            dist = np.linalg.norm(target[:2] - pos[:2])
            leg_time = dist / v
            # Enforce earliest arrival
            arrive = t + leg_time
            start_svc = max(arrive, early[c])
            delivery_times[c] = start_svc
            # Log trajectory
            steps = int(leg_time / dt) + 1
            for step in range(steps):
                frac = min(step * dt / leg_time, 1.0)
                history[k].append(pos[:2] + frac * (target[:2] - pos[:2]))
            pos = target.copy()
            t = start_svc + svc_time
        # Return to depot
        history[k].append(depots[depot_idx, :2].copy())
    return history, delivery_times
```

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Depots | 3, positions (100,100), (500,900), (900,200) m |
| Customers | 12, random in [50, 950] m grid |
| Drones | 6 total (2 per depot) |
| Cruising speed | 15 m/s |
| Payload capacity | 3.0 kg per drone |
| Max range per sortie | 4000 m |
| Service time per stop | 10 s |
| Time window width | 60 – 180 s |
| Tardiness weight $\alpha$ | 0.7 |
| Demand per customer | 0.3 – 1.5 kg |
| Simulation timestep | 0.5 s |

---

## Expected Output

- Top-down 2D map: depot markers (triangles), customer markers (circles), drone routes colour-coded by drone index
- Gantt chart: each drone's timeline (travel, service, waiting, return) with time-window bars overlaid
- Bar chart: total weighted tardiness for Greedy vs Clarke-Wright vs VRPTW
- Bar chart: total fleet flight distance for each strategy
- Table: per-customer delivery time, time window, and tardiness for the best solution
- Convergence curve (if OR-Tools or custom local search used): objective vs wall-clock time

---

## Extensions

1. Heterogeneous fleet — drones with different speeds and capacities; revisit Clarke-Wright savings with drone-specific arc costs
2. Online order insertion (S033) — new orders arrive after routes are committed; rolling-horizon re-optimisation
3. Charging queue integration (S032) — range constraint replaced by battery model with recharge stops at depots
4. Multi-objective Pareto front — plot tardiness vs distance trade-off across $\alpha \in [0,1]$
5. Stochastic demand — customer demands drawn from distributions; chance-constrained VRPTW

---

## Related Scenarios

- Prerequisites: [S021](S021_point_delivery.md), [S022](S022_obstacle_avoidance_delivery.md)
- Follow-ups: [S030](S030_multi_depot_delivery.md), [S031](S031_path_deconfliction.md), [S033](S033_online_order_insertion.md)
- Algorithmic cross-reference: [S018](../01_pursuit_evasion/S018_multi_target_intercept.md) (TSP formulation), [S019](../01_pursuit_evasion/S019_dynamic_reassignment.md) (Hungarian assignment)

## References

- Solomon, M.M. (1987). "Algorithms for the Vehicle Routing and Scheduling Problems with Time Window Constraints." *Operations Research*, 35(2), 254–265.
- Clarke, G. & Wright, J.W. (1964). "Scheduling of Vehicles from a Central Depot to a Number of Delivery Points." *Operations Research*, 12(4), 568–581.
- Toth, P. & Vigo, D. (2014). *Vehicle Routing: Problems, Methods, and Applications* (2nd ed.). SIAM.
