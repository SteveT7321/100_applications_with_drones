# S037 Reverse Logistics

**Domain**: Logistics & Delivery | **Difficulty**: ⭐⭐⭐ | **Status**: `[ ]` Not started

---

## Problem Definition

**Setup**: A fleet of $N = 5$ drones is deployed to collect returned parcels from $M = 12$ customer locations scattered across a $500 \times 500$ m arena and consolidate them at a central depot. Unlike outbound delivery, reverse logistics imposes asymmetric constraints: parcels have heterogeneous weights $q_i$ and pick-up time windows $[e_i, l_i]$ during which a drone must arrive; the drone payload capacity $Q_{max}$ limits how many returns it can consolidate on a single sweep before flying back to deposit at the depot. The problem is a **Capacitated Vehicle Routing Problem with Time Windows (CVRPTW)** solved with a combination of Clarke-Wright savings and 2-opt local search.

**Roles**:
- **Fleet drones** ($N$ units): each starts and ends at the depot $\mathbf{d} = (250, 250)$ m; each carries at most $Q_{max}$ kg of returned parcels.
- **Return customers** ($M$ locations): each customer $i$ has a parcel of weight $q_i$ and a time window $[e_i, l_i]$ for pick-up; a drone arriving before $e_i$ must wait; arriving after $l_i$ incurs a penalty.
- **Depot**: the consolidation hub where all returns are deposited; drones recharge here between sweeps.

**Objective**: Assign customers to drones and sequence each drone's pick-up route to

$$\min \sum_{k=1}^{N} \left( L_k + \lambda \cdot V_k \right)$$

where $L_k$ is the total route length of drone $k$, $V_k = \sum_{i \in \mathcal{R}_k} \max(0,\; t_{arrive,k,i} - l_i)$ is the total lateness (time-window violation), and $\lambda$ is the penalty weight. Subject to:

1. **Capacity**: $\sum_{i \in \mathcal{R}_k} q_i \leq Q_{max}$ for each drone $k$.
2. **Time windows**: drone $k$ arrives at customer $i$ at time $t_{arrive,k,i} \in [e_i,\; l_i + \epsilon]$ (hard window preferred; soft penalty applied otherwise).
3. **Range**: each drone's route length $L_k \leq R_{max}$ (battery range).

---

## Mathematical Model

### Route Representation

Let $\mathcal{R}_k = \{i_1, i_2, \ldots, i_{n_k}\}$ be the ordered pick-up sequence for drone $k$, with $\mathbf{w}^k_0 = \mathbf{d}$ (depot) and $\mathbf{w}^k_{n_k+1} = \mathbf{d}$ (return to depot). Route length:

$$L_k = \|\mathbf{w}^k_1 - \mathbf{d}\| + \sum_{j=1}^{n_k - 1} \|\mathbf{w}^k_{j+1} - \mathbf{w}^k_j\| + \|\mathbf{d} - \mathbf{w}^k_{n_k}\|$$

### Arrival Time Propagation

Flying at cruise speed $v_c$, the arrival time at stop $j$ of drone $k$:

$$t^k_j = \max\!\left(e_{i_j},\; t^k_{j-1} + \frac{\|\mathbf{w}^k_j - \mathbf{w}^k_{j-1}\|}{v_c}\right)$$

where $t^k_0 = 0$ (dispatch time at depot). The $\max$ with $e_{i_j}$ models the drone waiting at a location if it arrives early. Time-window violation for stop $j$:

$$v^k_j = \max\!\left(0,\; t^k_j - l_{i_j}\right)$$

Total lateness for drone $k$:

$$V_k = \sum_{j=1}^{n_k} v^k_j$$

### Clarke-Wright Savings Algorithm (Initialisation)

For every pair of customers $(i, j)$ not yet on the same route, the **savings** of merging them onto one drone instead of two separate depot-out-and-back trips is:

$$s_{ij} = \|\mathbf{c}_i - \mathbf{d}\| + \|\mathbf{c}_j - \mathbf{d}\| - \|\mathbf{c}_i - \mathbf{c}_j\|$$

Procedure:

1. Initialise $N$ singleton routes: each drone serves exactly one customer.
2. Sort all pairs $(i,j)$ by $s_{ij}$ descending.
3. For each pair in order, merge the two routes if:
   - The capacity constraint is not violated: $\sum q \leq Q_{max}$,
   - Customer $i$ is at the tail of its current route and $j$ is at the head of its current route (or vice versa),
   - The resulting time-window feasibility score does not worsen.
4. Continue until no more feasible merges exist or all $N$ routes are filled.

The result is a set of initial routes that are near-optimal in distance, though not time-window optimal.

### 2-opt Local Search (Improvement)

For each drone route, iteratively test reversing every sub-sequence $[j, k]$ within the route. The reversal is accepted if it reduces the combined cost $L_k + \lambda V_k$:

$$\Delta_{j,k} = \left(L_k' + \lambda V_k'\right) - \left(L_k + \lambda V_k\right) < 0$$

The new route after a successful 2-opt swap replacing edges $(j-1 \to j)$ and $(k \to k+1)$ with $(j-1 \to k)$ and $(j \to k+1)$:

$$\mathcal{R}_k' = [i_1, \ldots, i_{j-1},\; i_k, i_{k-1}, \ldots, i_j,\; i_{k+1}, \ldots, i_{n_k}]$$

Iterate until no improving swap is found (local optimum).

### Inter-Route Or-opt Move

To balance load across drones, apply Or-opt moves: relocate a single customer $i$ from drone $k$ to drone $k'$ at the cheapest insertion position (identical to the cheapest-insertion rule of S033):

$$\Delta_{relocate}(i, k \to k') = \delta_{k'}^*(i) - \delta_k^{remove}(i)$$

where $\delta_{k'}^*(i)$ is the cheapest insertion cost into route $k'$ and

$$\delta_k^{remove}(i) = \|\mathbf{c}_{prev} - \mathbf{c}_i\| + \|\mathbf{c}_i - \mathbf{c}_{next}\| - \|\mathbf{c}_{prev} - \mathbf{c}_{next}\|$$

is the distance saved by removing $i$ from route $k$. Accept if $\Delta_{relocate} < 0$ and capacity and range constraints remain satisfied in both routes.

### Payload State Model

The cumulative payload carried by drone $k$ after picking up stop $j$ is:

$$Q^k_j = \sum_{m=1}^{j} q_{i_m}$$

Payload constraint at each stop:

$$Q^k_j \leq Q_{max} \quad \forall\; j = 1, \ldots, n_k$$

### Objective Function (Full)

$$J = \sum_{k=1}^{N} L_k + \lambda \sum_{k=1}^{N} V_k + \mu \sum_{k=1}^{N} \max(0,\; L_k - R_{max})$$

where $\mu \gg \lambda$ is a hard range-violation penalty to ensure battery feasibility.

---

## Implementation

```python
import numpy as np
import matplotlib.pyplot as plt
from dataclasses import dataclass, field
from typing import List, Tuple, Optional
import itertools

# Key constants
N_DRONES     = 5         # fleet size
N_CUSTOMERS  = 12        # return-parcel pick-up locations
Q_MAX        = 3.0       # kg — drone payload capacity
R_MAX        = 2000.0    # m — max route length per charge
V_CRUISE     = 8.0       # m/s
LAMBDA_TW    = 50.0      # penalty weight per second of lateness
MU_RANGE     = 1000.0    # penalty weight per metre of range excess
DT           = 0.1       # s simulation timestep
ARENA_SIZE   = 500.0     # m
DEPOT        = np.array([250.0, 250.0])


@dataclass
class Customer:
    id: int
    pos: np.ndarray    # shape (2,) in metres
    weight: float      # kg
    e: float           # earliest pick-up time (s)
    l: float           # latest pick-up time (s)


@dataclass
class DroneRoute:
    drone_id: int
    sequence: List[int]      # ordered customer IDs
    arrival_times: List[float] = field(default_factory=list)
    payload_trace: List[float] = field(default_factory=list)


def route_length(sequence: List[int], customers: List[Customer],
                 depot: np.ndarray) -> float:
    if not sequence:
        return 0.0
    pts = [depot] + [customers[i].pos for i in sequence] + [depot]
    return sum(np.linalg.norm(pts[j+1] - pts[j]) for j in range(len(pts)-1))


def compute_arrival_times(sequence: List[int], customers: List[Customer],
                          depot: np.ndarray, v: float) -> List[float]:
    arrivals = []
    t = 0.0
    prev = depot
    for cid in sequence:
        travel = np.linalg.norm(customers[cid].pos - prev) / v
        t = max(customers[cid].e, t + travel)
        arrivals.append(t)
        prev = customers[cid].pos
    return arrivals


def total_violation(sequence: List[int], customers: List[Customer],
                    depot: np.ndarray, v: float) -> float:
    arrivals = compute_arrival_times(sequence, customers, depot, v)
    return sum(max(0.0, arrivals[j] - customers[sequence[j]].l)
               for j in range(len(sequence)))


def route_cost(sequence: List[int], customers: List[Customer],
               depot: np.ndarray, v: float,
               lam: float = LAMBDA_TW, mu: float = MU_RANGE) -> float:
    L = route_length(sequence, customers, depot)
    V = total_violation(sequence, customers, depot, v)
    range_exc = max(0.0, L - R_MAX)
    return L + lam * V + mu * range_exc


def savings_matrix(customers: List[Customer], depot: np.ndarray) -> np.ndarray:
    M = len(customers)
    S = np.zeros((M, M))
    for i, j in itertools.combinations(range(M), 2):
        s = (np.linalg.norm(customers[i].pos - depot) +
             np.linalg.norm(customers[j].pos - depot) -
             np.linalg.norm(customers[i].pos - customers[j].pos))
        S[i, j] = S[j, i] = s
    return S


def clarke_wright(customers: List[Customer], depot: np.ndarray,
                  n_drones: int) -> List[List[int]]:
    """Initialise routes with Clarke-Wright savings heuristic."""
    routes = [[i] for i in range(len(customers))]  # singleton routes
    S = savings_matrix(customers, depot)
    # List all (i,j) pairs sorted by savings descending
    pairs = sorted(itertools.combinations(range(len(customers)), 2),
                   key=lambda p: -S[p[0], p[1]])

    for (i, j) in pairs:
        if len(routes) <= n_drones:
            break  # already consolidated to fleet size
        ri = next((r for r in routes if r[-1] == i), None)
        rj = next((r for r in routes if r[0] == j), None)
        if ri is None or rj is None or ri is rj:
            continue
        merged = ri + rj
        cap_ok = sum(customers[c].weight for c in merged) <= Q_MAX
        if cap_ok:
            routes.remove(ri)
            routes.remove(rj)
            routes.append(merged)

    # Pad with empty routes if fewer customers than drones
    while len(routes) < n_drones:
        routes.append([])
    return routes[:n_drones]


def two_opt(route: List[int], customers: List[Customer],
            depot: np.ndarray) -> List[int]:
    """Apply 2-opt improvement to a single route."""
    best = route[:]
    improved = True
    while improved:
        improved = False
        for j in range(1, len(best) - 1):
            for k in range(j + 1, len(best)):
                candidate = best[:j] + best[j:k+1][::-1] + best[k+1:]
                if route_cost(candidate, customers, depot, V_CRUISE) < \
                   route_cost(best, customers, depot, V_CRUISE) - 1e-6:
                    best = candidate
                    improved = True
    return best


def or_opt_relocate(routes: List[List[int]], customers: List[Customer],
                    depot: np.ndarray) -> List[List[int]]:
    """Single-customer relocation across routes (Or-opt)."""
    routes = [r[:] for r in routes]
    improved = True
    while improved:
        improved = False
        for k_src in range(len(routes)):
            for pos_src, cid in enumerate(routes[k_src]):
                for k_dst in range(len(routes)):
                    if k_dst == k_src:
                        continue
                    cap_ok = (sum(customers[c].weight for c in routes[k_dst])
                              + customers[cid].weight <= Q_MAX)
                    if not cap_ok:
                        continue
                    # Try inserting cid at every position in k_dst
                    for pos_dst in range(len(routes[k_dst]) + 1):
                        src_new = routes[k_src][:pos_src] + routes[k_src][pos_src+1:]
                        dst_new = routes[k_dst][:pos_dst] + [cid] + routes[k_dst][pos_dst:]
                        delta = (route_cost(src_new, customers, depot, V_CRUISE) +
                                 route_cost(dst_new, customers, depot, V_CRUISE) -
                                 route_cost(routes[k_src], customers, depot, V_CRUISE) -
                                 route_cost(routes[k_dst], customers, depot, V_CRUISE))
                        if delta < -1e-6:
                            routes[k_src] = src_new
                            routes[k_dst] = dst_new
                            improved = True
                            break
                    if improved:
                        break
                if improved:
                    break
    return routes


def run_simulation():
    rng = np.random.default_rng(42)
    customers = [
        Customer(
            id=i,
            pos=rng.uniform(50, 450, size=2),
            weight=rng.uniform(0.3, 1.2),
            e=rng.uniform(0, 60),
            l=rng.uniform(90, 180)
        )
        for i in range(N_CUSTOMERS)
    ]

    # Phase 1: Clarke-Wright initialisation
    routes_cw = clarke_wright(customers, DEPOT, N_DRONES)

    # Phase 2: 2-opt intra-route improvement
    routes_2opt = [two_opt(r, customers, DEPOT) for r in routes_cw]

    # Phase 3: Or-opt inter-route relocation
    routes_final = or_opt_relocate(routes_2opt, customers, DEPOT)

    return customers, routes_cw, routes_2opt, routes_final
```

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Fleet size $N$ | 5 drones |
| Pick-up locations $M$ | 12 customers |
| Payload capacity $Q_{max}$ | 3.0 kg |
| Drone max range $R_{max}$ | 2 000 m |
| Cruise speed $v_c$ | 8.0 m/s |
| Arena | $[50, 450]^2$ m |
| Depot | $(250, 250)$ m |
| Parcel weight $q_i$ | 0.3 – 1.2 kg (uniform) |
| Earliest pick-up $e_i$ | 0 – 60 s (uniform) |
| Latest pick-up $l_i$ | 90 – 180 s (uniform) |
| Time-window penalty $\lambda$ | 50 s/m |
| Range violation penalty $\mu$ | 1 000 |
| Simulation timestep $\Delta t$ | 0.1 s |

---

## Expected Output

- **2D top-down route map**: depot as a black square, customer locations as circles sized by parcel weight, each drone's route in a distinct colour; pick-up order annotated with sequence numbers; time windows shown as a heatmap overlay (green = wide window, red = tight window).
- **Route cost comparison bar chart**: total distance, total lateness, and combined cost $J$ for three solution stages: (1) Clarke-Wright only, (2) after 2-opt, (3) after Or-opt — showing the incremental improvement at each phase.
- **Payload trace per drone**: stacked bar chart of parcel weights picked up by each drone, with the $Q_{max}$ limit marked as a horizontal dashed line.
- **Arrival time vs time window plot**: for each customer, a horizontal bar showing $[e_i, l_i]$ and a marker for the actual arrival time; colour-coded green (on time) or red (late).
- **Fleet utilisation table**: route length, total payload, number of stops, and lateness per drone.
- **Animation (GIF)**: top-down view; drones move simultaneously along their routes; parcel pick-up events cause a brief colour flash at the customer marker; payload counter shown per drone.

---

## Extensions

1. **Stochastic availability windows**: model $l_i$ as a random variable with exponential tail; use robust optimisation (min-max regret) to hedge against tight-window customers being unavailable.
2. **Heterogeneous fleet**: drones with different $Q_{max}$ and $R_{max}$; extend Clarke-Wright to assign heavier-return clusters to high-capacity drones using a min-cost bipartite matching pre-step.
3. **Multi-depot reverse logistics**: two depot hubs; solve the location-routing problem jointly, assigning each customer to a depot and a drone, minimising total system distance.
4. **Drone recharge mid-route**: if a route exceeds $R_{max}$, introduce a charging station stop modelled as a mandatory waypoint with a dwell time of $\tau_{charge}$ s; re-optimise the sub-route around the charge stop.
5. **Real-time cancellation**: a customer cancels their return during the mission; implement online Or-opt relocation to redistribute their time slot to an already-en-route drone that can absorb the freed capacity.
6. **Integration with outbound delivery (S021/S036)**: co-optimise outbound delivery and reverse pick-up on the same drone tour (simultaneous pickup and delivery, SPDP variant).

---

## Related Scenarios

- Prerequisites: [S021](S021_point_delivery.md), [S029](S029_urban_logistics_scheduling.md), [S033](S033_online_order_insertion.md)
- Follow-ups: [S036](S036_last_mile_relay.md) (relay chain for long-haul returns), [S038](S038_disaster_relief.md) (multi-objective routing under urgency)
- Algorithm reference: Clarke-Wright (1964) savings algorithm; CVRPTW standard benchmark (Solomon instances)
