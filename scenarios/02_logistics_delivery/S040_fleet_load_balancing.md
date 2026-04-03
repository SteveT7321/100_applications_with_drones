# S040 Fleet Load Balancing

**Domain**: Logistics & Delivery | **Difficulty**: ⭐⭐⭐⭐ | **Status**: `[ ]` Not started

---

## Problem Definition

**Setup**: A fleet of $N = 8$ delivery drones operates from a single base depot over a
$600 \times 600$ m urban grid. A stream of $M = 40$ delivery orders arrives dynamically over a
horizon $T_{sim} = 400$ s. Each drone carries a battery with normalised state-of-charge (SoC)
$s_i \in [0, 1]$, a current workload $w_i$ (number of pending orders in its queue), and an
accumulated flight distance $f_i$ (a proxy for wear). A central load-balancer reassigns newly
arrived orders — and may also swap queued-but-not-yet-started orders between drones — to keep
the fleet as evenly loaded as possible.

**Roles**:
- **Drones** ($N = 8$): each executes its private FIFO delivery queue, monitors SoC, and returns
  for recharging when the battery falls below a threshold.
- **Load-balancer**: a real-time dispatcher that scores each drone on a composite load index and
  assigns new orders (and initiates queue swaps) to minimise the maximum load imbalance.
- **Depot**: single base at $(300, 300)$ m; provides recharging (2 pads) and order injection.

**Objective**: Minimise the peak-to-mean load imbalance across the fleet at every dispatch
decision point:

$$\min_{\text{assignment}} \; \Lambda(t) = \frac{\max_i L_i(t)}{\bar{L}(t)} - 1$$

where $L_i(t)$ is the load index of drone $i$ at time $t$ and $\bar{L}(t)$ is the fleet mean.
Secondary objective: maximise total completed deliveries within $T_{sim}$.

**Comparison strategies**:
1. **Round-Robin** — assign orders cyclically to drones regardless of state
2. **Least-Queue** — assign to the drone with fewest pending orders
3. **Load-Index Balancer** (proposed) — composite score using SoC, queue depth, and travel distance
4. **RL-dispatched** (extension) — trained PPO agent observes the full fleet state vector

---

## Mathematical Model

### Drone Load Index

The load index $L_i(t)$ for drone $i$ at time $t$ combines three factors:

$$L_i(t) = w_i(t) \cdot \phi_w + \frac{f_i(t)}{F_{norm}} \cdot \phi_f + \bigl(1 - s_i(t)\bigr) \cdot \phi_s$$

where:
- $w_i(t)$ — number of pending orders in drone $i$'s queue (not yet serviced)
- $f_i(t)$ — cumulative flight distance (metres) since mission start
- $s_i(t) \in [0, 1]$ — current SoC (1 = full)
- $F_{norm}$ — normalisation constant (maximum expected flight distance per drone)
- $\phi_w, \phi_f, \phi_s \geq 0$ — tunable weights, $\phi_w + \phi_f + \phi_s = 1$

### Fleet Imbalance Metric

At any dispatch event, the imbalance ratio is:

$$\Lambda(t) = \frac{L_{max}(t) - L_{min}(t)}{\bar{L}(t)}, \qquad
  \bar{L}(t) = \frac{1}{N}\sum_{i=1}^N L_i(t)$$

The time-averaged imbalance over the full horizon:

$$\bar{\Lambda} = \frac{1}{|\mathcal{E}|} \sum_{e \in \mathcal{E}} \Lambda(t_e)$$

where $\mathcal{E}$ is the set of dispatch decision epochs.

### Order Assignment Rule

When order $o$ arrives at time $t_o$, assign it to drone $k^*$:

$$k^* = \arg\min_{i \in \mathcal{A}(t_o)} \bigl[ L_i(t_o) + \Delta L_i(o) \bigr]$$

where $\mathcal{A}(t_o) = \{i : s_i(t_o) \geq s_{min}(i, o)\}$ is the set of battery-feasible
drones, and $\Delta L_i(o)$ is the marginal load increment if order $o$ is appended to drone $i$'s
queue:

$$\Delta L_i(o) = \phi_w \cdot 1 + \phi_f \cdot \frac{\|\mathbf{q}_o - \mathbf{p}_{last,i}\|}{F_{norm}}$$

Here $\mathbf{p}_{last,i}$ is the position of the last waypoint currently in drone $i$'s queue (or
its current position if idle), and $\mathbf{q}_o$ is the delivery location of order $o$.

### Battery Feasibility for Assignment

Drone $i$ can accept order $o$ only if its projected SoC after completing its current queue plus
the new order exceeds the safety reserve $\beta_{safe}$:

$$s_i(t_o) - \frac{\bigl(L_i^{dist}(t_o) + d_{extra}(i,o)\bigr)}{D_{range}} \geq \beta_{safe}$$

where $L_i^{dist}(t_o)$ is the total remaining flight distance in drone $i$'s queue, $d_{extra}$
is the extra distance incurred by adding order $o$, and $D_{range}$ is the full-battery range in
metres.

### Queue Swap Rebalancing

At regular rebalancing epochs $t_r \in \{0, \Delta t_r, 2\Delta t_r, \ldots\}$, the balancer may
transfer a pending (not yet started) order from the most-loaded drone $i^+$ to the least-loaded
feasible drone $i^-$. Let the candidate swap order be the last in $i^+$'s queue. Accept the swap
if:

$$\Delta \Lambda_{swap} = \Lambda\bigl(\text{after swap}\bigr) - \Lambda\bigl(\text{before swap}\bigr) < 0$$

and the transferred order remains battery-feasible for drone $i^-$.

The number of beneficial swaps per rebalancing epoch is bounded by $S_{max}$ to limit
computational overhead.

### Battery Discharge Model

Energy consumed flying distance $d$ (normalised SoC units):

$$\Delta s = \frac{\alpha \cdot (m_{body} + m_p) \cdot g \cdot d}{E_{cap}}$$

Return-feasibility threshold (dynamic):

$$s_{min}(i) = \frac{\alpha \cdot (m_{body} + m_p) \cdot g \cdot \|\mathbf{p}_i - \mathbf{p}_{depot}\|}{E_{cap}} + \beta_{safe}$$

### Throughput and Jain's Fairness Index

Total deliveries completed:

$$D_{total} = \sum_{i=1}^{N} D_i$$

Jain's fairness index on per-drone delivery counts $\{D_i\}$:

$$\mathcal{J} = \frac{\bigl(\sum_{i=1}^{N} D_i\bigr)^2}{N \cdot \sum_{i=1}^{N} D_i^2} \in \left[\frac{1}{N},\, 1\right]$$

$\mathcal{J} = 1$ indicates perfect load balance (all drones complete equal deliveries).

---

## Implementation

```python
import numpy as np
from collections import deque
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

# Key constants
N_DRONES      = 8
N_PADS        = 2
M_ORDERS      = 40
T_SIM         = 400.0        # s
DT            = 0.2          # s simulation timestep
REBAL_PERIOD  = 20.0         # s  rebalancing epoch interval
S_MAX_SWAPS   = 2            # max queue swaps per rebalancing epoch

V_DRONE       = 8.0          # m/s cruise speed
M_BODY        = 1.5          # kg
M_PAYLOAD     = 0.3          # kg
G             = 9.81         # m/s²
E_CAP         = 1.0          # normalised battery capacity
ALPHA         = 1.5e-4       # discharge coefficient (1/m)
P_CHG         = 0.20         # charge rate (E_cap/s)
BETA_SAFE     = 0.05         # safety SoC reserve
D_RANGE       = E_CAP / (ALPHA * (M_BODY + M_PAYLOAD) * G)  # ~3560 m full-range
F_NORM        = D_RANGE * 0.5  # normalisation for flight distance

# Load-index weights
PHI_W = 0.5    # queue depth weight
PHI_F = 0.3    # cumulative distance weight
PHI_S = 0.2    # battery depletion weight

DEPOT = np.array([300.0, 300.0, 2.0])
GRID  = (0.0, 600.0)

@dataclass
class Order:
    idx: int
    location: np.ndarray
    t_arrive: float

@dataclass
class Drone:
    idx: int
    pos: np.ndarray
    soc: float = 1.0
    queue: deque = field(default_factory=deque)  # deque of Order
    deliveries: int = 0
    flight_dist: float = 0.0
    state: str = 'idle'   # idle | flying | returning | charging | waiting
    target: Optional[np.ndarray] = None
    current_order: Optional[Order] = None

    def load_index(self) -> float:
        return (PHI_W * len(self.queue)
                + PHI_F * (self.flight_dist / F_NORM)
                + PHI_S * (1.0 - self.soc))

    def remaining_queue_distance(self) -> float:
        pts = [self.pos if self.target is None else self.target]
        for o in self.queue:
            pts.append(o.location)
        pts.append(DEPOT)
        return sum(np.linalg.norm(pts[k+1] - pts[k]) for k in range(len(pts)-1))

    def can_accept(self, order: Order) -> bool:
        extra = 0.0
        if self.queue:
            last = self.queue[-1].location
        elif self.target is not None:
            last = self.target
        else:
            last = self.pos
        extra = np.linalg.norm(order.location - last)
        projected_dist = self.remaining_queue_distance() + extra
        projected_soc  = self.soc - projected_dist * ALPHA * (M_BODY + M_PAYLOAD) * G
        return projected_soc >= BETA_SAFE


def marginal_load(drone: Drone, order: Order) -> float:
    last = drone.queue[-1].location if drone.queue else (
           drone.target if drone.target is not None else drone.pos)
    extra_dist = np.linalg.norm(order.location - last)
    return PHI_W * 1.0 + PHI_F * extra_dist / F_NORM


def assign_order(drones: List[Drone], order: Order) -> Optional[int]:
    """Load-index balancer: assign order to minimum post-assignment load drone."""
    best_k, best_score = None, float('inf')
    for drone in drones:
        if drone.state in ('returning', 'charging', 'waiting'):
            continue
        if not drone.can_accept(order):
            continue
        score = drone.load_index() + marginal_load(drone, order)
        if score < best_score:
            best_score = score
            best_k = drone.idx
    return best_k


def rebalance(drones: List[Drone]) -> int:
    """Attempt up to S_MAX_SWAPS queue transfers from most-loaded to least-loaded drone."""
    swaps = 0
    for _ in range(S_MAX_SWAPS):
        active = [d for d in drones if d.queue]
        if len(active) < 2:
            break
        most  = max(active, key=lambda d: d.load_index())
        least = min(drones,  key=lambda d: d.load_index())
        if most.idx == least.idx:
            break
        candidate = most.queue[-1]   # last (not yet started) order
        imbal_before = max(d.load_index() for d in drones) - min(d.load_index() for d in drones)
        # Tentatively transfer
        most.queue.pop()
        least.queue.append(candidate)
        imbal_after = max(d.load_index() for d in drones) - min(d.load_index() for d in drones)
        if imbal_after >= imbal_before or not least.can_accept(candidate):
            # Revert
            least.queue.pop()
            most.queue.append(candidate)
            break
        swaps += 1
    return swaps


def fleet_imbalance(drones: List[Drone]) -> float:
    loads = [d.load_index() for d in drones]
    mean  = np.mean(loads)
    if mean == 0:
        return 0.0
    return (max(loads) - min(loads)) / mean


def jain_fairness(drones: List[Drone]) -> float:
    counts = np.array([d.deliveries for d in drones], dtype=float)
    if counts.sum() == 0:
        return 1.0
    return counts.sum()**2 / (N_DRONES * (counts**2).sum())


def run_simulation(strategy: str = 'load_index', seed: int = 42) -> dict:
    rng   = np.random.default_rng(seed)
    drones = [Drone(idx=i, pos=DEPOT.copy()) for i in range(N_DRONES)]
    pads   = [None] * N_PADS
    charge_rem = [0.0] * N_PADS
    wait_queue = deque()

    # Pre-generate order stream
    orders = [
        Order(idx=j,
              location=np.array([rng.uniform(*GRID), rng.uniform(*GRID), 2.0]),
              t_arrive=rng.uniform(0, T_SIM * 0.8))
        for j in range(M_ORDERS)
    ]
    orders.sort(key=lambda o: o.t_arrive)
    order_stream = deque(orders)

    imbalance_log = []
    rebal_log     = []
    t = 0.0
    next_rebal = REBAL_PERIOD

    while t <= T_SIM:
        # --- Inject arriving orders ---
        while order_stream and order_stream[0].t_arrive <= t:
            order = order_stream.popleft()
            if strategy == 'load_index':
                k = assign_order(drones, order)
            elif strategy == 'least_queue':
                k = min(range(N_DRONES), key=lambda i: len(drones[i].queue))
            elif strategy == 'round_robin':
                k = order.idx % N_DRONES
            else:
                k = assign_order(drones, order)   # default
            if k is not None:
                drones[k].queue.append(order)

        # --- Rebalancing epoch ---
        if t >= next_rebal and strategy == 'load_index':
            swaps = rebalance(drones)
            rebal_log.append((t, swaps))
            next_rebal += REBAL_PERIOD

        # --- Step each drone ---
        for d in drones:
            if d.state == 'idle' and d.queue:
                d.current_order = d.queue.popleft()
                d.target = d.current_order.location.copy()
                d.state  = 'flying'

            if d.state == 'flying' and d.target is not None:
                to_target = d.target - d.pos
                dist_rem  = np.linalg.norm(to_target)
                step_dist = V_DRONE * DT
                if dist_rem <= step_dist:
                    d.pos = d.target.copy()
                    d.soc -= ALPHA * (M_BODY + M_PAYLOAD) * G * dist_rem
                    d.flight_dist += dist_rem
                    d.deliveries  += 1
                    d.current_order = None
                    d.target = None
                    # Check battery for returning
                    dist_home = np.linalg.norm(d.pos - DEPOT)
                    if d.soc <= ALPHA * (M_BODY + M_PAYLOAD) * G * dist_home + BETA_SAFE + 0.1:
                        d.state  = 'returning'
                        d.target = DEPOT.copy()
                    else:
                        d.state = 'idle'   # try next order
                else:
                    direction = to_target / dist_rem
                    d.pos     += direction * step_dist
                    d.soc     -= ALPHA * (M_BODY + M_PAYLOAD) * G * step_dist
                    d.flight_dist += step_dist
                    # Mid-flight battery check
                    dist_home = np.linalg.norm(d.pos - DEPOT)
                    if d.soc <= ALPHA * (M_BODY + M_PAYLOAD) * G * dist_home + BETA_SAFE:
                        d.state  = 'returning'
                        d.target = DEPOT.copy()

            elif d.state == 'returning':
                to_base  = DEPOT - d.pos
                dist_rem = np.linalg.norm(to_base)
                step_dist = V_DRONE * DT
                if dist_rem <= step_dist:
                    d.pos  = DEPOT.copy()
                    d.soc -= ALPHA * (M_BODY + M_PAYLOAD) * G * dist_rem
                    d.soc  = max(d.soc, 0.0)
                    d.flight_dist += dist_rem
                    free = next((k for k, v in enumerate(pads) if v is None), None)
                    if free is not None:
                        pads[free] = d.idx
                        charge_rem[free] = (1.0 - d.soc) / P_CHG
                        d.state = 'charging'
                    else:
                        wait_queue.append(d.idx)
                        d.state = 'waiting'
                else:
                    d.pos     += (to_base / dist_rem) * step_dist
                    d.soc     -= ALPHA * (M_BODY + M_PAYLOAD) * G * step_dist
                    d.flight_dist += step_dist

        # --- Advance charging pads ---
        for k in range(N_PADS):
            if pads[k] is not None:
                charge_rem[k] -= DT
                d = drones[pads[k]]
                d.soc = min(1.0, d.soc + P_CHG * DT)
                if charge_rem[k] <= 0:
                    d.soc   = 1.0
                    d.state = 'idle'
                    pads[k] = None
                    if wait_queue:
                        nxt = wait_queue.popleft()
                        pads[k] = nxt
                        nd = drones[nxt]
                        charge_rem[k] = (1.0 - nd.soc) / P_CHG
                        nd.state = 'charging'

        imbalance_log.append((t, fleet_imbalance(drones)))
        t += DT

    return {
        'strategy':       strategy,
        'total_deliveries': sum(d.deliveries for d in drones),
        'jain_fairness':   jain_fairness(drones),
        'mean_imbalance':  np.mean([v for _, v in imbalance_log]),
        'imbalance_log':   imbalance_log,
        'rebal_log':       rebal_log,
        'drones':          drones,
    }
```

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Fleet size $N$ | 8 drones |
| Charging pads | 2 |
| Total orders $M$ | 40 |
| Simulation horizon $T_{sim}$ | 400 s |
| Cruise speed $v$ | 8.0 m/s |
| Discharge coefficient $\alpha$ | $1.5 \times 10^{-4}$ m$^{-1}$ |
| Body mass $m_{body}$ | 1.5 kg |
| Payload mass $m_p$ | 0.3 kg |
| Full-battery range $D_{range}$ | ~3560 m |
| Charging rate $P_{chg}$ | 0.20 $E_{cap}$/s |
| Safety SoC reserve $\beta_{safe}$ | 0.05 |
| Load-index weights $(\phi_w, \phi_f, \phi_s)$ | (0.5, 0.3, 0.2) |
| Rebalancing period $\Delta t_r$ | 20 s |
| Max queue swaps per epoch $S_{max}$ | 2 |
| Delivery area | $[0, 600]^2$ m |
| Depot | $(300, 300, 2)$ m |
| Simulation timestep $\Delta t$ | 0.2 s |

---

## Expected Output

- **Imbalance time series**: $\Lambda(t)$ vs time for all four strategies on the same axes;
  shaded bands show rebalancing epochs for the Load-Index Balancer
- **Jain's fairness bar chart**: $\mathcal{J}$ per strategy — visual comparison of how fairly
  deliveries are distributed across the fleet
- **Per-drone delivery count histogram**: side-by-side bars for all $N = 8$ drones under each
  strategy; target is a flat profile
- **Cumulative deliveries vs time**: step-function curves for each strategy; identifies when
  throughput diverges
- **Fleet state Gantt chart**: for the Load-Index Balancer, show each drone's state
  (flying / returning / charging / waiting / idle) as a horizontal timeline
- **Load index heatmap**: $L_i(t)$ as a colour grid (drones × time) — reveals persistent
  overloading of individual drones under Round-Robin
- **Rebalancing events log**: scatter plot of swap counts per epoch vs time; annotated with
  imbalance reduction achieved

---

## Extensions

1. **Weight sweep**: grid-search $(\phi_w, \phi_f, \phi_s)$ over the probability simplex and plot
   Pareto surface of $\bar{\Lambda}$ vs $D_{total}$
2. **Heterogeneous fleet**: drones with different speeds and battery capacities; adapt the load
   index to use drone-specific $F_{norm,i}$ and $D_{range,i}$
3. **Demand surge**: inject a burst of 10 orders in a 10 s window at $t = 200$ s; measure
   recovery time to return $\Lambda(t)$ below 0.3
4. **RL dispatcher**: replace the hand-crafted load index with a PPO agent; state vector
   $\mathbf{o} = [s_i, w_i, f_i/F_{norm}, \|\mathbf{p}_i - \mathbf{p}_{depot}\|]_{i=1}^{N}$,
   action = drone index to assign, reward = $-\Lambda(t)$
5. **Multi-depot extension**: $M_{dep} = 3$ depots with shared order stream; load index extended
   with depot-assignment variable; compare centralised vs per-depot balancing

---

## Related Scenarios

- Prerequisites: [S029](S029_urban_logistics_scheduling.md), [S032](S032_charging_queue.md), [S033](S033_online_order_insertion.md)
- Follow-ups: [S041](../03_environmental_sar/S041_area_coverage.md) (swarm coverage, multi-agent coordination)
- Algorithmic cross-reference: [S036](S036_last_mile_relay.md) (relay chain, battery-constrained routing), [S019](../01_pursuit_evasion/S019_dynamic_reassignment.md) (real-time Hungarian reassignment)

## References

- Jain, R., Chiu, D., & Hawe, W. (1984). "A Quantitative Measure of Fairness and Discrimination for Resource Allocation in Shared Computer Systems." *DEC Technical Report DEC-TR-301*.
- Pillac, V. et al. (2013). "A Review of Dynamic Vehicle Routing Problems." *European Journal of Operational Research*, 225(1), 1–11.
- Toth, P. & Vigo, D. (2014). *Vehicle Routing: Problems, Methods, and Applications* (2nd ed.). SIAM.
- Li, J. et al. (2021). "Multi-UAV Task Assignment with Load Balancing via Reinforcement Learning." *IEEE Transactions on Vehicular Technology*, 70(10), 10818–10831.
