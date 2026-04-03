# S038 Disaster Relief Drop

**Domain**: Logistics & Delivery | **Difficulty**: ⭐⭐⭐⭐ | **Status**: `[ ]` Not started

---

## Problem Definition

**Setup**: A natural disaster (flood / earthquake) has severed ground-access roads to $M = 8$ relief
sites spread across a $1000 \times 1000$ m area. A forward operating base (FOB) at a known position
holds a heterogeneous fleet of $K = 5$ drones. Each relief site $s_i$ has a **priority level**
$\pi_i \in \{1, 2, 3\}$ (1 = critical, 2 = urgent, 3 = standard) and a **demand weight** $q_i$ (kg).
Drones have different payload capacities $Q_k$ and max ranges $R_k$; all drones must return to the
FOB after each sortie. No ground infrastructure exists beyond the FOB: drones are the only delivery
vector. The mission has a hard **time horizon** $T_{max} = 600$ s; any site not served before
$T_{max}$ incurs an unserved penalty scaled by its priority.

**Roles**:
- **FOB** (base of operations): single depot at $\mathbf{p}_{FOB}$; all drones depart from and
  return to this point.
- **Drones** ($K = 5$): heterogeneous fleet; each drone $k$ has payload capacity $Q_k$, maximum
  range $R_k$ (one-way equivalent), and cruise speed $v_k$.
- **Relief sites** ($M = 8$): fixed positions $\mathbf{s}_i$, each with demand $q_i$, priority
  $\pi_i$, and a criticality deadline $d_i$ (site becomes inaccessible or demand expires at $d_i$).

**Objective**: Construct a sortie schedule — assigning drones to ordered sequences of relief sites
per sortie — to **maximise total weighted delivery value** within $T_{max}$, where the value of
serving site $i$ at time $t_i^{arrive}$ is:

$$V_i(t) = \frac{\pi_{max} + 1 - \pi_i}{\pi_{max}} \cdot \exp\!\left(-\lambda \cdot \max(0,\, t - d_i^{soft})\right)$$

with $\pi_{max} = 3$, $\lambda = 0.01$ s$^{-1}$ (urgency decay), and $d_i^{soft}$ a soft deadline
after which value degrades.

**Comparison strategies**:
1. **Greedy priority-first** — always dispatch the idle drone to the highest-priority unserved site,
   nearest-neighbour sequencing within each sortie.
2. **Regret-based insertion** — build routes by maximum-regret criterion to hedge against poor
   early decisions.
3. **Iterated local search (ILS)** — random restart + 2-opt / Or-opt improvements on the full
   multi-sortie schedule.

---

## Mathematical Model

### Fleet and Demand Model

Let $\mathcal{K} = \{1, \ldots, K\}$ be drone indices and $\mathcal{S} = \{1, \ldots, M\}$ be
relief-site indices. The FOB is node $0$.

Drone $k$ parameters:

$$Q_k \in \mathbb{R}_{>0} \quad \text{(kg payload)}, \quad R_k \in \mathbb{R}_{>0} \quad
\text{(m max range per sortie)}, \quad v_k \in \mathbb{R}_{>0} \quad \text{(m/s)}$$

Each sortie for drone $k$ is a sequence $\sigma_k^{(r)} = (0, i_1, i_2, \ldots, i_{n_r}, 0)$
(depart FOB, visit sites in order, return to FOB). Multiple sorties per drone are allowed;
sortie $r+1$ begins only after sortie $r$ is complete (returned to FOB) and a turnaround time
$\tau_{turn} = 30$ s has elapsed (battery swap).

### Payload and Range Feasibility

Payload constraint for sortie $\sigma_k^{(r)}$:

$$\sum_{i \in \sigma_k^{(r)}} q_i \leq Q_k$$

Range constraint (round-trip, full sortie):

$$\sum_{j=0}^{n_r} \|\mathbf{s}_{i_{j+1}} - \mathbf{s}_{i_j}\| \leq R_k$$

where $\mathbf{s}_0 = \mathbf{p}_{FOB}$ and $\mathbf{s}_{i_{n_r+1}} = \mathbf{p}_{FOB}$.

### Delivery Time Propagation

For sortie $\sigma_k^{(r)}$ with departure time $t_0^{(r)}$, the arrival time at site $i_j$ is:

$$t_{arrive}(i_j) = t_0^{(r)} + \sum_{l=1}^{j} \frac{\|\mathbf{s}_{i_l} - \mathbf{s}_{i_{l-1}}\|}{v_k}
  + (j-1) \cdot \tau_{drop}$$

where $\tau_{drop} = 8$ s is the drop time at each site (hover + release). The sortie return time:

$$t_{return}^{(r)} = t_{arrive}(i_{n_r}) + \tau_{drop} + \frac{\|\mathbf{p}_{FOB} - \mathbf{s}_{i_{n_r}}\|}{v_k}$$

Next sortie departure time:

$$t_0^{(r+1)} = t_{return}^{(r)} + \tau_{turn}$$

### Weighted Delivery Value Objective

Total mission value (to maximise):

$$J = \sum_{i \in \mathcal{S}_{served}} V_i\!\left(t_{arrive}(i)\right) \cdot q_i
    - \mu \sum_{i \in \mathcal{S}_{unserved}} \frac{\pi_{max} + 1 - \pi_i}{\pi_{max}} \cdot q_i$$

where $\mathcal{S}_{served}$ is the set of sites delivered within $T_{max}$,
$\mathcal{S}_{unserved} = \mathcal{S} \setminus \mathcal{S}_{served}$, and $\mu = 5$ is the
penalty multiplier for unserved critical demand.

Equivalently, maximising $J$ is equivalent to minimising the sum of **priority-weighted lateness**
plus unserved-demand penalty:

$$\min \; \sum_{i \in \mathcal{S}_{served}} \frac{\pi_{max} + 1 - \pi_i}{\pi_{max}}
  \cdot \max\!\bigl(0,\; t_{arrive}(i) - d_i^{soft}\bigr)
  + \mu \sum_{i \in \mathcal{S}_{unserved}} \frac{\pi_{max} + 1 - \pi_i}{\pi_{max}} \cdot q_i$$

### Greedy Priority-First Dispatch

At each dispatch decision, score each unserved site $i$ for idle drone $k$:

$$\text{score}(i, k) = \frac{w_{\pi}(\pi_i)}{d_{FOB,i} / v_k}$$

where $w_{\pi}(\pi_i) = \pi_{max} + 1 - \pi_i$ (higher priority $\Rightarrow$ larger weight) and
$d_{FOB,i} = \|\mathbf{s}_i - \mathbf{p}_{FOB}\|$. Assign site with highest score to drone $k$.
Pack additional sites into the same sortie (nearest-neighbour from current site) as long as payload
and range constraints remain feasible.

### Regret-Based Insertion

For each unassigned site $i$, compute the best and second-best insertion costs across all drones
and insertion positions:

$$\Delta_{i}^{(1)} \leq \Delta_{i}^{(2)} \quad \text{(sorted insertion cost increase)}$$

$$\text{regret}(i) = \Delta_{i}^{(2)} - \Delta_{i}^{(1)}$$

Insert the site with the **largest regret** into its best position. Repeat until all sites are
assigned or no feasible insertion exists. Sites with large regret have few good alternatives and
should be locked in early to avoid costly reassignment.

### 2-opt Improvement for Single-Sortie Routes

Within a sortie $\sigma = (0, i_1, \ldots, i_n, 0)$, a 2-opt swap reverses the sub-sequence
$[i_a, \ldots, i_b]$:

$$\Delta L_{2opt}(a, b) = \bigl(d_{i_{a-1}, i_b} + d_{i_{a}, i_{b+1}}\bigr)
                         - \bigl(d_{i_{a-1}, i_a} + d_{i_b, i_{b+1}}\bigr)$$

Accept if $\Delta L_{2opt}(a, b) < 0$ (reduces route length). Apply to all sorties of all drones;
repeat until no improving swap exists.

### Or-Opt Move (Single-Site Relocation)

Move site $i_a$ from sortie $\sigma_k$ to sortie $\sigma_{k'}$ at position $p$:

$$\Delta J_{relocate}(i_a, k \to k', p) = V_{i_a}(t_{arrive}^{new}(i_a)) - V_{i_a}(t_{arrive}^{old}(i_a))
  - \text{(detour cost in } \sigma_k\text{)} + \text{(detour cost in } \sigma_{k'}\text{)}$$

Accept if $\Delta J_{relocate} > 0$. This allows rebalancing load between drones to improve
value delivery.

### Drone Kinematics

Point-mass, constant-speed straight-line segments between waypoints:

$$\dot{\mathbf{p}}_k = v_k \cdot \hat{\mathbf{r}}_k, \qquad
\hat{\mathbf{r}}_k = \frac{\mathbf{w}_{next} - \mathbf{p}_k}{\|\mathbf{w}_{next} - \mathbf{p}_k\|}$$

Altitude maintained at $z = 20$ m cruise; descent to $z = 2$ m at drop site, then ascent back to
cruise before proceeding to next waypoint. Vertical transition adds $\delta t_{vert} = 4$ s per
drop (included in $\tau_{drop}$).

---

## Implementation

```python
import numpy as np
import matplotlib.pyplot as plt
from dataclasses import dataclass, field
from typing import List, Optional
import itertools

# Key constants
K_DRONES      = 5
M_SITES       = 8
T_MAX         = 600.0    # s — mission horizon
TAU_DROP      = 8.0      # s — hover + release at each site
TAU_TURN      = 30.0     # s — battery swap turnaround at FOB
LAMBDA_DECAY  = 0.01     # s^-1 — value decay rate after soft deadline
MU_PENALTY    = 5.0      # unserved demand penalty multiplier
PI_MAX        = 3        # maximum priority level
DT            = 0.1      # s — simulation timestep

FOB = np.array([500.0, 100.0, 20.0])

# Heterogeneous fleet: [payload kg, range m, speed m/s]
FLEET = np.array([
    [5.0, 2000.0, 12.0],   # drone 0 — heavy-lift, slow
    [5.0, 2000.0, 12.0],   # drone 1
    [2.5, 2800.0, 16.0],   # drone 2 — mid-range
    [2.5, 2800.0, 16.0],   # drone 3
    [1.0, 3500.0, 20.0],   # drone 4 — fast scout, light payload
])

# Relief sites: [x, y, demand_kg, priority, soft_deadline_s]
SITES = np.array([
    [150., 800., 3.0, 1, 120.],   # s0 critical
    [820., 750., 2.0, 1, 150.],   # s1 critical
    [300., 600., 1.5, 2, 200.],   # s2 urgent
    [700., 500., 1.0, 2, 240.],   # s3 urgent
    [200., 350., 2.5, 1, 100.],   # s4 critical — nearest critical
    [600., 300., 1.0, 3, 300.],   # s5 standard
    [850., 200., 0.8, 3, 350.],   # s6 standard
    [450., 700., 1.5, 2, 180.],   # s7 urgent
])

@dataclass
class Drone:
    idx: int
    payload: float       # kg
    max_range: float     # m
    speed: float         # m/s
    pos: np.ndarray = field(default_factory=lambda: FOB.copy())
    t_available: float = 0.0   # next available departure time
    sorties: list = field(default_factory=list)

@dataclass
class Sortie:
    drone_idx: int
    site_sequence: List[int]
    depart_time: float
    arrival_times: List[float] = field(default_factory=list)
    return_time: float = 0.0

def delivery_value(priority, t_arrive, soft_deadline):
    """Priority-weighted time-decayed delivery value."""
    w = (PI_MAX + 1 - priority) / PI_MAX
    decay = np.exp(-LAMBDA_DECAY * max(0.0, t_arrive - soft_deadline))
    return w * decay

def sortie_range(site_seq, sites, fob):
    """Total round-trip distance for a sortie visiting sites in given order."""
    pts = [fob[:2]] + [sites[i, :2] for i in site_seq] + [fob[:2]]
    return sum(np.linalg.norm(pts[j+1] - pts[j]) for j in range(len(pts)-1))

def sortie_arrivals(site_seq, sites, fob, depart_time, speed, tau_drop):
    """Compute arrival time at each site in the sortie."""
    t = depart_time
    pos = fob[:2].copy()
    arrivals = []
    for idx in site_seq:
        target = sites[idx, :2]
        t += np.linalg.norm(target - pos) / speed
        arrivals.append(t)
        t += tau_drop
        pos = target.copy()
    return_t = t + np.linalg.norm(fob[:2] - pos) / speed
    return arrivals, return_t

def greedy_priority_dispatch(drones, sites, fob, t_max, tau_drop, tau_turn):
    """Greedy priority-first dispatch: assign highest-priority site to each idle drone."""
    served = set()
    all_sorties = []

    def score(site_idx, drone):
        dist = np.linalg.norm(sites[site_idx, :2] - fob[:2])
        priority = int(sites[site_idx, 3])
        w = (PI_MAX + 1 - priority) / PI_MAX
        return w / (dist / drone.speed + 1e-6)

    while True:
        # Find the first available drone
        idle_drone = min(drones, key=lambda d: d.t_available)
        if idle_drone.t_available >= t_max:
            break

        unserved = [i for i in range(len(sites)) if i not in served]
        if not unserved:
            break

        # Rank unserved sites by score for this drone
        ranked = sorted(unserved, key=lambda i: -score(i, idle_drone))
        seed = ranked[0]

        # Build sortie: seed site + nearest-neighbour packing
        sortie_sites = [seed]
        total_demand = sites[seed, 2]
        pos = sites[seed, :2].copy()

        remaining = [i for i in ranked[1:] if i != seed]
        for candidate in sorted(remaining,
                                key=lambda i: np.linalg.norm(sites[i, :2] - pos)):
            if total_demand + sites[candidate, 2] > idle_drone.payload:
                continue
            trial = sortie_sites + [candidate]
            if sortie_range(trial, sites, fob) > idle_drone.max_range:
                continue
            sortie_sites = trial
            total_demand += sites[candidate, 2]
            pos = sites[candidate, :2].copy()

        arrivals, return_t = sortie_arrivals(
            sortie_sites, sites, fob,
            idle_drone.t_available, idle_drone.speed, tau_drop
        )

        s = Sortie(
            drone_idx=idle_drone.idx,
            site_sequence=sortie_sites,
            depart_time=idle_drone.t_available,
            arrival_times=arrivals,
            return_time=return_t,
        )
        all_sorties.append(s)
        served.update(sortie_sites)
        idle_drone.t_available = return_t + tau_turn

    return all_sorties, served

def two_opt_sortie(site_seq, sites, fob):
    """Apply 2-opt improvement to a single sortie's site ordering."""
    best = site_seq[:]
    improved = True
    while improved:
        improved = False
        for a in range(len(best) - 1):
            for b in range(a + 1, len(best)):
                candidate = best[:a] + best[a:b+1][::-1] + best[b+1:]
                if sortie_range(candidate, sites, fob) < sortie_range(best, sites, fob):
                    best = candidate
                    improved = True
    return best

def compute_total_value(sorties, sites, t_max):
    """Sum delivery values for all served sites within T_max."""
    total = 0.0
    served_ids = set()
    for s in sorties:
        for idx, (site_id, t_arr) in enumerate(zip(s.site_sequence, s.arrival_times)):
            if t_arr <= t_max:
                total += delivery_value(
                    int(sites[site_id, 3]), t_arr, sites[site_id, 4]
                ) * sites[site_id, 2]
                served_ids.add(site_id)
    # Penalty for unserved critical/urgent sites
    for i in range(len(sites)):
        if i not in served_ids:
            w = (PI_MAX + 1 - int(sites[i, 3])) / PI_MAX
            total -= MU_PENALTY * w * sites[i, 2]
    return total, served_ids

def run_simulation():
    drones = [
        Drone(idx=k, payload=FLEET[k, 0], max_range=FLEET[k, 1], speed=FLEET[k, 2])
        for k in range(K_DRONES)
    ]
    sorties, served = greedy_priority_dispatch(
        drones, SITES, FOB, T_MAX, TAU_DROP, TAU_TURN
    )
    # 2-opt improvement on each sortie
    for s in sorties:
        if len(s.site_sequence) > 2:
            s.site_sequence = two_opt_sortie(s.site_sequence, SITES, FOB)
            s.arrival_times, s.return_time = sortie_arrivals(
                s.site_sequence, SITES, FOB,
                s.depart_time, FLEET[s.drone_idx, 2], TAU_DROP
            )
    total_value, served_ids = compute_total_value(sorties, SITES, T_MAX)
    return sorties, served_ids, total_value
```

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Relief sites $M$ | 8 |
| Fleet size $K$ | 5 (heterogeneous) |
| Heavy-lift drones (0–1) | 5.0 kg, 2000 m range, 12 m/s |
| Mid-range drones (2–3) | 2.5 kg, 2800 m range, 16 m/s |
| Scout drone (4) | 1.0 kg, 3500 m range, 20 m/s |
| Mission horizon $T_{max}$ | 600 s |
| Drop time per site $\tau_{drop}$ | 8 s |
| Battery swap time $\tau_{turn}$ | 30 s |
| Value decay rate $\lambda$ | 0.01 s$^{-1}$ |
| Unserved penalty $\mu$ | 5.0 |
| Priority levels | 1 = critical, 2 = urgent, 3 = standard |
| Critical soft deadlines | 100 – 150 s |
| Urgent soft deadlines | 180 – 240 s |
| FOB position | (500, 100) m |
| Area | 1000 × 1000 m |
| Cruise altitude | 20 m |
| Simulation timestep | 0.1 s |

---

## Expected Output

- **Mission map**: 2D top-down plot showing FOB (black star), relief sites colour-coded by
  priority (red = critical, orange = urgent, blue = standard), annotated with demand weights;
  sortie routes drawn as coloured arcs per drone.
- **Gantt chart**: per-drone timeline showing sortie legs (flight segments), drop dwells, battery
  swap pauses; sites marked with priority colour on the bar.
- **Delivery value timeline**: cumulative total value $J(t)$ vs time for each strategy (Greedy /
  Regret-based / ILS); horizontal dashed line at theoretical maximum.
- **Coverage bar chart**: fraction of demand served by priority tier (critical / urgent / standard)
  for each strategy; stacked bars highlighting unserved demand.
- **Arrival lateness histogram**: $\max(0, t_{arrive}(i) - d_i^{soft})$ for all served sites,
  grouped by strategy.
- **Animation (GIF)**: top-down view with drones moving along their sortie paths, sites turning
  from hollow to filled as deliveries are made, FOB pulsing during battery swaps.

---

## Extensions

1. **Stochastic demand revelation**: site demands and priorities are uncertain until the drone is
   within radio range $r_{comm} = 200$ m; implement an adaptive re-dispatch policy that updates
   sortie plans mid-flight when new information arrives.
2. **Multiple FOBs / forward staging**: allow drones to land at served sites and recharge from
   portable power units, effectively creating dynamic relay nodes (see S036) to extend coverage
   depth.
3. **Aerial supply convoy**: very heavy payloads (e.g., water tanks) require cooperative lift
   (see S026); model two-drone sling-load sorties with coordinated flight paths.
4. **RL dispatch policy**: train a PPO agent to make dispatch decisions (which drone, which site,
   when to depart) given the full state vector (drone positions, battery levels, unserved site
   priorities); compare to ILS on out-of-distribution disaster layouts.
5. **Communication blackout zones**: certain regions have GPS/radio jamming; drones must rely on
   inertial navigation with drift $\sigma_{INS} = 0.5$ m/s and re-localise at known visual
   landmarks.

---

## Related Scenarios

- Prerequisites: [S021 Point Delivery](S021_point_delivery.md), [S029 Urban Logistics Scheduling](S029_urban_logistics_scheduling.md), [S033 Online Order Insertion](S033_online_order_insertion.md)
- Follow-ups: [S039](S039_perimeter_patrol_logistics.md) (sustained resupply), [S040](S040_fleet_load_balancing.md) (load balancing)
- Algorithmic cross-reference: [S034 Weather Rerouting](S034_weather_rerouting.md) (replanning under disruption), [S036 Last-Mile Relay](S036_last_mile_relay.md) (range extension), [S026 Cooperative Heavy Lift](S026_cooperative_heavy_lift.md) (multi-drone payload)
