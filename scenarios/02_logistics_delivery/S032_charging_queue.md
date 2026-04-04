# S032 Charging Queue

**Domain**: Logistics & Delivery | **Difficulty**: ⭐⭐⭐ | **Status**: `[x]` Complete

---

## Problem Definition

**Setup**: A fleet of $N = 6$ delivery drones operates continuously over a city grid, executing
delivery tasks drawn from a queue. The base station provides $K = 2$ charging pads; a drone that
needs to recharge must fly back to base, wait in a FIFO queue if both pads are occupied, charge to
full, and then resume missions. The simulation runs for a fixed horizon $T_{sim} = 300$ s and
measures total deliveries completed and fleet utilisation.

**Roles**:
- **Drones** (×6): fly to assigned delivery waypoints, monitor battery state-of-charge (SoC), and
  decide when to return for charging.
- **Charging pads** (×2): serve one drone each; charging time depends on SoC deficit.
- **Mission dispatcher**: maintains a queue of pending deliveries and assigns idle drones.

**Objective**: maximise total deliveries completed within $T_{sim}$ by choosing the optimal
**return-to-charge threshold** $\beta^* \in (0, 1]$ — the SoC level at which a drone abandons its
current mission and heads home.

**Comparison**:
1. **Threshold policy** — drone returns when SoC $\leq \beta$; sweep $\beta \in \{0.1, 0.2, \ldots, 0.5\}$
2. **Greedy (always-finish)** — drone completes its delivery before returning regardless of SoC
3. **Predictive policy** — drone returns early only if predicted SoC on return flight falls below a
   safety margin $\beta_{safe} = 0.05$

---

## Mathematical Model

### Battery Discharge Model

Drone $i$ carries payload $m_p$ and flies at constant airspeed $v$. Power draw is approximated as
proportional to total thrust:

$$P_i = \alpha \left( m_{body} + m_p \right) g \cdot v$$

where $\alpha > 0$ is an efficiency constant. Energy consumed over a flight leg of distance $d$:

$$\Delta E_i = P_i \cdot \frac{d}{v} = \alpha \left( m_{body} + m_p \right) g \cdot d$$

State-of-charge after travelling distance $d$ from SoC $s$:

$$s' = s - \frac{\Delta E_i}{E_{cap}}$$

### Return-Flight Feasibility

Let $\mathbf{p}_i$ be the drone's current position and $\mathbf{p}_{base}$ the base. The minimum
SoC needed to return safely (with margin $\beta_{safe}$):

$$s_{min}(i) = \frac{\alpha (m_{body} + m_p) g \cdot \|\mathbf{p}_i - \mathbf{p}_{base}\|}{E_{cap}} + \beta_{safe}$$

The drone should return if its current SoC satisfies:

$$s_i \leq \beta \quad \text{(threshold policy)}$$

or, for the predictive policy, if:

$$s_i \leq s_{min}(i)$$

### Charging Dynamics

When a drone docks on a pad, it charges at constant power $P_{chg}$ until full:

$$t_{chg}(i) = \frac{(1 - s_i^{arrive}) \cdot E_{cap}}{P_{chg}}$$

### Queue Waiting Time

With $K$ pads and $n_q$ drones ahead in the FIFO queue, the expected waiting time (assuming
deterministic, equal charge times $\bar{t}_{chg}$):

$$t_{wait}(n_q) = \left\lfloor \frac{n_q}{K} \right\rfloor \cdot \bar{t}_{chg}$$

### Fleet Utilisation

For drone $i$ over horizon $T_{sim}$:

$$u_i = \frac{t_{fly,i}}{T_{sim} - t_{grounded,i}}$$

where $t_{fly,i}$ is total airborne time and $t_{grounded,i}$ is any mandatory grounding period
(e.g. maintenance). Mean fleet utilisation:

$$\bar{u} = \frac{1}{N} \sum_{i=1}^{N} u_i$$

### Total Deliveries

$$D_{total}(\beta) = \sum_{i=1}^{N} D_i(\beta)$$

where $D_i(\beta)$ counts completed deliveries for drone $i$ under threshold $\beta$.

### Optimal Threshold

$$\beta^* = \arg\max_{\beta \in \mathcal{B}} D_{total}(\beta)$$

where $\mathcal{B} = \{0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.45, 0.50\}$.

---

## Implementation

```python
import numpy as np
import heapq
from collections import deque

# Key constants
N_DRONES      = 6
N_PADS        = 2
T_SIM         = 300.0      # s
DT            = 0.5        # s
V_DRONE       = 8.0        # m/s cruise speed
E_CAP         = 1.0        # normalised battery capacity
P_CHG         = 0.25       # charge rate (fraction of E_cap per second)
ALPHA         = 1.8e-4     # efficiency constant (1/m), tuned so ~5 km range at full charge
M_BODY        = 1.5        # kg
M_PAYLOAD     = 0.5        # kg
G             = 9.81       # m/s²
BETA_SAFE     = 0.05       # emergency reserve
BASE          = np.array([0.0, 0.0, 2.0])
GRID_HALF     = 30.0       # delivery area half-width (m)

def discharge_per_metre(m_payload=M_PAYLOAD):
    return ALPHA * (M_BODY + m_payload) * G

def time_to_fly(dist):
    return dist / V_DRONE

def random_waypoint(rng):
    x = rng.uniform(-GRID_HALF, GRID_HALF)
    y = rng.uniform(-GRID_HALF, GRID_HALF)
    return np.array([x, y, 2.0])

class Drone:
    def __init__(self, idx, rng):
        self.idx   = idx
        self.soc   = 1.0
        self.pos   = BASE.copy()
        self.state = 'idle'   # idle | flying | returning | charging | waiting
        self.deliveries = 0
        self.target = None
        self.rng   = rng

    def dist_to_base(self):
        return np.linalg.norm(self.pos - BASE)

    def min_soc_to_return(self):
        return discharge_per_metre() * self.dist_to_base() / E_CAP + BETA_SAFE

def run_simulation(beta_threshold, seed=42):
    rng = np.random.default_rng(seed)
    drones = [Drone(i, rng) for i in range(N_DRONES)]
    pads   = [None] * N_PADS          # drone index occupying each pad, or None
    charge_remaining = [0.0] * N_PADS # seconds of charge remaining on each pad
    queue  = deque()                   # FIFO: indices of drones waiting for a pad

    mission_queue = deque(random_waypoint(rng) for _ in range(50))

    t = 0.0
    while t < T_SIM:
        # --- Assign missions to idle drones ---
        for d in drones:
            if d.state == 'idle' and mission_queue:
                d.target = mission_queue.popleft()
                d.state  = 'flying'

        # --- Step each drone ---
        for d in drones:
            if d.state == 'flying':
                dist_step = V_DRONE * DT
                to_target = d.target - d.pos
                dist_rem  = np.linalg.norm(to_target)

                # Check threshold: should we abort and return?
                if d.soc <= beta_threshold or d.soc <= d.min_soc_to_return():
                    d.state  = 'returning'
                    d.target = BASE.copy()
                    continue

                if dist_rem <= dist_step:
                    # Arrived at delivery waypoint
                    d.soc -= discharge_per_metre() * dist_rem / E_CAP
                    d.pos  = d.target.copy()
                    d.deliveries += 1
                    mission_queue.append(random_waypoint(rng))
                    d.state  = 'returning'
                    d.target = BASE.copy()
                else:
                    d.pos  += (to_target / dist_rem) * dist_step
                    d.soc  -= discharge_per_metre() * dist_step / E_CAP

            elif d.state == 'returning':
                to_base  = BASE - d.pos
                dist_rem = np.linalg.norm(to_base)
                dist_step = V_DRONE * DT
                if dist_rem <= dist_step:
                    d.pos  = BASE.copy()
                    d.soc -= discharge_per_metre() * dist_rem / E_CAP
                    d.soc  = max(d.soc, 0.0)
                    # Try to acquire a free pad
                    free = next((k for k, v in enumerate(pads) if v is None), None)
                    if free is not None:
                        pads[free] = d.idx
                        charge_remaining[free] = (1.0 - d.soc) / P_CHG
                        d.state = 'charging'
                    else:
                        queue.append(d.idx)
                        d.state = 'waiting'
                else:
                    d.pos  += (to_base / dist_rem) * dist_step
                    d.soc  -= discharge_per_metre() * dist_step / E_CAP

        # --- Advance charging pads ---
        for k in range(N_PADS):
            if pads[k] is not None:
                charge_remaining[k] -= DT
                d = drones[pads[k]]
                d.soc = min(1.0, d.soc + P_CHG * DT)
                if charge_remaining[k] <= 0:
                    d.soc   = 1.0
                    d.state = 'idle'
                    pads[k] = None
                    # Serve next waiting drone
                    if queue:
                        nxt = queue.popleft()
                        pads[k] = nxt
                        nd = drones[nxt]
                        charge_remaining[k] = (1.0 - nd.soc) / P_CHG
                        nd.state = 'charging'

        t += DT

    return sum(d.deliveries for d in drones), drones
```

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Fleet size $N$ | 6 drones |
| Charging pads $K$ | 2 |
| Simulation horizon $T_{sim}$ | 300 s |
| Cruise speed $v$ | 8.0 m/s |
| Battery capacity $E_{cap}$ | 1.0 (normalised) |
| Charging rate $P_{chg}$ | 0.25 $E_{cap}$/s (full charge in 4 s at empty) |
| Discharge constant $\alpha$ | $1.8 \times 10^{-4}$ m$^{-1}$ |
| Body mass $m_{body}$ | 1.5 kg |
| Payload mass $m_p$ | 0.5 kg |
| Safety reserve $\beta_{safe}$ | 0.05 |
| Delivery area | $[-30, 30]^2$ m grid |
| Base position | (0, 0, 2) m |
| Threshold sweep $\mathcal{B}$ | 0.10 – 0.50 in steps of 0.05 |

---

## Expected Output

- **Threshold sweep curve**: $D_{total}(\beta)$ vs $\beta$ — identifies $\beta^*$
- **Policy comparison bar chart**: total deliveries for threshold-optimal, greedy, and predictive
  policies
- **State timeline**: Gantt-style chart showing each drone's state (flying / returning / charging /
  waiting) over $T_{sim}$
- **Queue length time series**: number of drones waiting for a charging pad vs time
- **SoC traces**: SoC over time for all 6 drones, coloured by drone index, with charge events
  marked
- **Fleet utilisation $\bar{u}$** printed per policy

---

## Extensions

1. **Variable number of pads**: sweep $K \in \{1, 2, 3, 4\}$ and plot $D_{total}(K)$ — diminishing
   returns analysis
2. **Heterogeneous batteries**: drones have different $E_{cap}$ values; does the shared threshold
   $\beta^*$ still hold?
3. **Priority queuing**: high-urgency deliveries jump the mission queue; analyse impact on mean
   delivery latency
4. **Adaptive threshold via RL**: replace the fixed $\beta$ with a PPO agent whose observation is
   $(s_i, \|\mathbf{p}_i - \mathbf{p}_{base}\|, n_q, n_{pending})$ and reward is deliveries per
   unit time
5. **Distributed charging depots**: place $M$ depots across the grid and add depot-assignment as an
   optimisation variable

---

## Related Scenarios

- Prerequisites: [S021](S021_point_delivery.md), [S029](S029_urban_logistics_scheduling.md)
- Follow-ups: [S033](S033_online_order_insertion.md), [S040](S040_fleet_load_balancing.md)
- See also: [S027](S027_aerial_refueling_relay.md) (airborne battery replenishment analogue)
