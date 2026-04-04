# S033 Online Order Insertion

**Domain**: Logistics & Delivery | **Difficulty**: ⭐⭐⭐ | **Status**: `[x]` Complete

---

## Problem Definition

**Setup**: A fleet of $K = 4$ drones is actively executing a pre-planned delivery schedule. Each drone holds an ordered waypoint queue (current position → depot → waypoint sequence) and has consumed some battery. At time $t_{arrive}$, a new customer order arrives at location $\mathbf{q}_{new}$ with urgency weight $w$. The dispatcher must immediately assign the order to one drone and splice it into that drone's remaining route — without halting any in-flight drone or replanning from scratch.

**Roles**:
- **Drones** $D_1, \ldots, D_K$: each carries a partial route $R_k = [\mathbf{w}^k_0, \mathbf{w}^k_1, \ldots, \mathbf{w}^k_{n_k}]$ where $\mathbf{w}^k_0$ is the current position
- **Dispatcher**: greedy online algorithm that evaluates every feasible insertion position across all drones and picks the global minimum-cost insertion
- **New order**: point $\mathbf{q}_{new}$ with deadline $t_{deadline}$

**Objective**: Minimise the total additional flight distance (detour cost) introduced by the insertion, subject to:
1. Battery feasibility: the drone's updated route must not exceed remaining range $B_k$
2. Deadline feasibility: the drone can reach $\mathbf{q}_{new}$ before $t_{deadline}$

---

## Mathematical Model

### Route Representation

Drone $k$ at insertion time has remaining waypoints:

$$R_k = [\mathbf{w}^k_0,\; \mathbf{w}^k_1,\; \ldots,\; \mathbf{w}^k_{n_k}]$$

The remaining route length is:

$$L_k = \sum_{i=0}^{n_k - 1} \|\mathbf{w}^k_{i+1} - \mathbf{w}^k_i\|$$

### Cheapest-Insertion Cost

For drone $k$, inserting $\mathbf{q}_{new}$ between waypoints $i$ and $i+1$:

$$\Delta_{k,i} = \|\mathbf{w}^k_i - \mathbf{q}_{new}\| + \|\mathbf{q}_{new} - \mathbf{w}^k_{i+1}\| - \|\mathbf{w}^k_i - \mathbf{w}^k_{i+1}\|$$

The best insertion position for drone $k$:

$$i^*_k = \arg\min_{0 \le i < n_k} \Delta_{k,i}$$

The associated cheapest-insertion cost for drone $k$:

$$\delta_k = \Delta_{k,\, i^*_k}$$

### Battery Feasibility Constraint

Remaining battery expressed as equivalent flight distance:

$$B_k^{rem} = B_{full} \cdot \frac{E_k^{rem}}{E_{full}}$$

Feasibility requires:

$$L_k + \delta_k \le B_k^{rem}$$

### Deadline Feasibility Constraint

Time to reach $\mathbf{q}_{new}$ via the cheapest insertion point, flying at speed $v_k$:

$$t_{arrive,k} = t_{now} + \frac{\sum_{j=0}^{i^*_k - 1} \|\mathbf{w}^k_{j+1} - \mathbf{w}^k_j\| + \|\mathbf{w}^k_{i^*_k} - \mathbf{q}_{new}\|}{v_k}$$

Feasibility requires:

$$t_{arrive,k} \le t_{deadline}$$

### Dispatcher Decision Rule

Among all feasible $(k, i^*_k)$ pairs, select:

$$k^* = \arg\min_{k \in \mathcal{F}} \delta_k$$

where $\mathcal{F} = \{k : \text{battery feasible} \cap \text{deadline feasible}\}$.

If $\mathcal{F} = \emptyset$, the order is queued for the next available drone (earliest route completion).

### Route Update

After assignment, drone $k^*$ route becomes:

$$R_{k^*} \leftarrow [\mathbf{w}^{k^*}_0,\; \ldots,\; \mathbf{w}^{k^*}_{i^*},\; \mathbf{q}_{new},\; \mathbf{w}^{k^*}_{i^*+1},\; \ldots,\; \mathbf{w}^{k^*}_{n_{k^*}}]$$

### Comparison Strategies

| Strategy | Assignment rule |
|----------|----------------|
| **Cheapest Insertion** (proposed) | Global $\min \delta_k$ across all feasible drones |
| **Nearest Drone** | Drone with smallest $\|\mathbf{w}^k_0 - \mathbf{q}_{new}\|$ (current position only) |
| **Least Loaded** | Drone with smallest remaining $n_k$ |
| **Random Feasible** | Uniform random draw from $\mathcal{F}$ |

---

## Implementation

```python
import numpy as np
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

# Key constants
K            = 4          # number of drones
V_DRONE      = 8.0        # m/s cruise speed (all drones identical)
B_FULL       = 10_000.0   # m — full-battery range equivalent
DT           = 0.1        # s simulation timestep
N_ORDERS     = 12         # pre-planned waypoints per drone at t=0
N_INSERTIONS = 8          # online orders that arrive mid-mission

DEPOT = np.array([0.0, 0.0, 2.0])

@dataclass
class Drone:
    id: int
    pos: np.ndarray
    route: List[np.ndarray]   # remaining waypoints (index 0 = next target)
    battery_rem: float        # metres of flight remaining
    speed: float = V_DRONE

    def route_length(self) -> float:
        pts = [self.pos] + self.route
        return sum(np.linalg.norm(pts[i+1] - pts[i]) for i in range(len(pts)-1))

    def insertion_cost(self, q: np.ndarray, idx: int) -> float:
        pts = [self.pos] + self.route
        a, b = pts[idx], pts[idx + 1]
        return np.linalg.norm(a - q) + np.linalg.norm(q - b) - np.linalg.norm(b - a)

    def time_to_reach_via_insertion(self, q: np.ndarray, idx: int,
                                    t_now: float) -> float:
        pts = [self.pos] + self.route
        dist = sum(np.linalg.norm(pts[i+1] - pts[i]) for i in range(idx))
        dist += np.linalg.norm(pts[idx] - q)
        return t_now + dist / self.speed


def cheapest_insertion(drones: List[Drone], q_new: np.ndarray,
                       t_now: float, t_deadline: float
                       ) -> Optional[Tuple[int, int, float]]:
    """Return (drone_id, insert_idx, delta_cost) or None if infeasible."""
    best = None
    for drone in drones:
        pts = [drone.pos] + drone.route
        n_segments = len(pts) - 1
        if n_segments < 1:
            continue
        for idx in range(n_segments):
            delta = drone.insertion_cost(q_new, idx)
            new_len = drone.route_length() + delta
            t_arr   = drone.time_to_reach_via_insertion(q_new, idx, t_now)
            battery_ok  = new_len <= drone.battery_rem
            deadline_ok = t_arr   <= t_deadline
            if battery_ok and deadline_ok:
                if best is None or delta < best[2]:
                    best = (drone.id, idx, delta)
    return best


def insert_order(drone: Drone, q_new: np.ndarray, insert_idx: int) -> None:
    """Splice q_new into drone's route after position insert_idx."""
    # insert_idx=0 means between current pos and route[0]
    drone.route.insert(insert_idx, q_new)
    delta = drone.insertion_cost.__func__  # recalculate battery
    # recompute remaining battery
    drone.battery_rem -= (
        np.linalg.norm(drone.pos - q_new)           # simplified delta
    )


# --- Simulation loop (sketch) ---
def run_simulation(strategy: str = "cheapest_insertion"):
    drones  = initialise_fleet(K, N_ORDERS, DEPOT)
    orders  = generate_online_orders(N_INSERTIONS, t_window=(10, 80))
    metrics = {"total_detour": 0.0, "feasible": 0, "queued": 0,
               "strategy": strategy}

    t = 0.0
    order_queue = list(orders)
    order_queue.sort(key=lambda o: o.t_arrive)

    while any(not d.route_done for d in drones) or order_queue:
        # Advance drones one step
        for drone in drones:
            drone.step(DT)

        # Dispatch newly arrived orders
        due = [o for o in order_queue if o.t_arrive <= t]
        for order in due:
            order_queue.remove(order)
            result = dispatch(drones, order, t, strategy)
            if result:
                metrics["feasible"]      += 1
                metrics["total_detour"]  += result.delta
            else:
                metrics["queued"] += 1

        t += DT

    return metrics
```

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Fleet size $K$ | 4 drones |
| Cruise speed $v$ | 8.0 m/s |
| Full battery range $B_{full}$ | 10 000 m |
| Arena | $[-500, 500]^2$ m, altitude 2–8 m |
| Pre-planned waypoints per drone | 12 |
| Online orders $N_{ins}$ | 8 |
| Order arrival window | $t \in [10, 80]$ s |
| Deadline slack | $+30$ s beyond nearest-drone ETA |
| Simulation timestep $\Delta t$ | 0.1 s |
| Depot | $(0, 0, 2)$ m |

---

## Expected Output

- 2D top-down route map: original planned routes (dashed) vs updated routes after all insertions (solid), colour-coded per drone, new order locations marked as stars
- Timeline plot: order arrival events overlaid on drone battery consumption curves
- Detour cost comparison bar chart: Cheapest Insertion vs Nearest Drone vs Least Loaded vs Random Feasible
- Feasibility rate table: fraction of orders served on-time per strategy
- Per-drone load balance histogram: number of inserted orders per drone under each strategy

---

## Extensions

1. Rolling-horizon re-optimisation: after each insertion, apply 2-opt local search to the affected drone's updated route
2. Battery-aware priority: weight insertion cost by remaining battery fraction so heavily loaded drones are deprioritised
3. Multi-depot variant: drones return to nearest of $M$ depots for recharging, insertion algorithm must account for en-route recharge stops
4. Stochastic deadlines: model $t_{deadline}$ as a soft constraint with penalty function $p(t_{arrive,k} - t_{deadline})$, enabling partial-late trade-offs
5. Online learning: track historical insertion costs vs actual detours to calibrate the cheapest-insertion heuristic over time

---

## Related Scenarios

- Prerequisites: S021 Single-Drone Delivery, S022 Multi-Stop Route Planning, S025 VRP Fleet Assignment
- Follow-ups: S034 Dynamic Re-routing with No-Fly Zones, S035 Battery-Constrained Multi-Depot

## References

- Psaraftis, H.N. (1988). "Dynamic Vehicle Routing Problems." *Vehicle Routing: Methods and Studies*. North-Holland.
- Solomon, M.M. (1987). "Algorithms for the Vehicle Routing and Scheduling Problems with Time Window Constraints." *Operations Research*, 35(2), 254–265.
- Pillac, V. et al. (2013). "A Review of Dynamic Vehicle Routing Problems." *European Journal of Operational Research*, 225(1), 1–11.
