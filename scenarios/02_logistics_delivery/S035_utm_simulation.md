# S035 UTM Simulation

**Domain**: Logistics & Delivery | **Difficulty**: ⭐⭐⭐⭐ | **Status**: `[x]` Complete

---

## Problem Definition

**Setup**: A low-altitude urban airspace of $1000 \times 1000 \times 120$ m is divided into a set of pre-defined flight corridors arranged in a layered altitude band structure (UTM corridors). $N = 12$ delivery drones simultaneously request flight plans from a centralised UTM service. Each drone submits a 4D trajectory intent (position + time) from its origin depot to its delivery waypoint. The UTM service performs:

1. **Intent registration**: accept or deny each 4D intent based on available corridor capacity.
2. **Conflict detection**: identify all pairwise separation violations in the registered intent set.
3. **Conflict resolution**: resolve detected conflicts through corridor reassignment, altitude layer reallocation, or time-based sequencing (holding delays).
4. **Separation assurance monitoring**: during execution, monitor real positions and issue resolution advisories (RAs) when predicted separation drops below the safety margin.

**Roles**:
- **UTM Service**: centralised authority that maintains the airspace state, processes intent registrations, detects conflicts, and issues resolution advisories.
- **Drones** ($N = 12$): execute assigned 4D trajectories, respond to RAs by adjusting speed or altitude, and report their state to the UTM at each telemetry interval.
- **Corridors**: directed airways defined by a start/end node pair, an altitude band $[z_{lo}, z_{hi}]$, a lane width $w_c$, and a maximum drone density $\rho_{max}$ (drones/km).

**Objective**: All $N$ drones complete their deliveries while:

1. Maintaining horizontal separation $\geq d_{sep}^{H} = 30$ m and vertical separation $\geq d_{sep}^{V} = 10$ m at all times.
2. Minimising total fleet delay introduced by conflict resolution (holding + rerouting time).
3. Maximising corridor throughput (drones delivered per minute).

**Comparison strategies**:
1. **First-come first-served (FCFS)**: intents registered in submission order; conflicts resolved by delaying the lower-priority flight.
2. **Priority-based**: drones ranked by urgency score $u_k$; higher-priority drones hold their trajectory, lower-priority drones are rerouted or delayed.
3. **Centralised optimisation**: resolve all conflicts jointly by minimising total delay using linear programming over departure time offsets.

---

## Mathematical Model

### Airspace Structure

The airspace is partitioned into $L = 4$ altitude layers:

| Layer | Altitude band | Nominal direction |
|-------|--------------|-------------------|
| L1 | 20 – 40 m | East–West |
| L2 | 40 – 60 m | North–South |
| L3 | 60 – 80 m | East–West (return) |
| L4 | 80 – 100 m | North–South (return) |

Each layer hosts $M_l$ directed corridor segments. A corridor $c$ in layer $l$ is defined by:

$$c = \bigl(\mathbf{p}_c^{start},\; \mathbf{p}_c^{end},\; z_{lo,l},\; z_{hi,l},\; w_c\bigr)$$

The corridor centre-line unit vector:

$$\hat{\mathbf{e}}_c = \frac{\mathbf{p}_c^{end} - \mathbf{p}_c^{start}}{\|\mathbf{p}_c^{end} - \mathbf{p}_c^{start}\|}$$

### 4D Trajectory Intent

Drone $k$ submits a piecewise-linear 4D trajectory:

$$\Pi_k = \bigl\{(\mathbf{p}_{k,0}, t_{k,0}),\; (\mathbf{p}_{k,1}, t_{k,1}),\; \ldots,\; (\mathbf{p}_{k,n_k}, t_{k,n_k})\bigr\}$$

Position at time $t$ during leg $[t_{k,i}, t_{k,i+1}]$:

$$\mathbf{p}_k(t) = \mathbf{p}_{k,i} + \frac{t - t_{k,i}}{t_{k,i+1} - t_{k,i}} \bigl(\mathbf{p}_{k,i+1} - \mathbf{p}_{k,i}\bigr)$$

### Conflict Detection

A **conflict** between drones $k$ and $j$ exists if there exists any time $t \in [t_{start}, t_{end}]$ such that:

$$\|\mathbf{p}_k(t) - \mathbf{p}_j(t)\|_{H} < d_{sep}^{H} \quad \text{AND} \quad |z_k(t) - z_j(t)| < d_{sep}^{V}$$

where $\|\cdot\|_{H}$ is the horizontal (xy-plane) Euclidean distance. For piecewise-linear trajectories, the minimum separation over leg pair $(i, j)$ is found by minimising the squared inter-drone distance, which reduces to a 1D quadratic:

$$\delta(\tau) = \|\mathbf{p}_k(t_{k,i} + \tau) - \mathbf{p}_j(t_{j,i} + \tau)\|^2$$

$$\tau^* = -\frac{\dot{\delta}(0)}{2\ddot{\delta}(0)}, \qquad \tau^* \in [0,\; \min(\Delta t_{k,i}, \Delta t_{j,i})]$$

The minimum separation during the leg pair is $\sqrt{\delta(\tau^*)}$. The full conflict scan over all $\binom{N}{2}$ pairs and all leg combinations runs in $O(N^2 n^2)$ time (pre-filtered by bounding box overlap).

### Corridor Capacity Constraint

The instantaneous drone density on corridor $c$ at time $t$:

$$\rho_c(t) = \frac{\#\{k : \mathbf{p}_k(t) \in c\}}{L_c / 1000} \quad \text{[drones/km]}$$

Capacity constraint:

$$\rho_c(t) \leq \rho_{max} \quad \forall\, c,\, t$$

### FCFS Conflict Resolution (Time-Based Sequencing)

Under FCFS, when a conflict between drones $k$ (high priority) and $j$ (low priority) is detected, drone $j$ is issued a **holding delay** $\Delta t_j^{hold}$ such that their time separation at the conflict point exceeds the minimum safe temporal gap $\Delta t_{sep}$:

$$\Delta t_j^{hold} = \Delta t_{sep} - \bigl(t_j^{conflict} - t_k^{conflict}\bigr)$$

The shifted trajectory for drone $j$ is:

$$\Pi_j' = \bigl\{(\mathbf{p}_{j,i},\; t_{j,i} + \Delta t_j^{hold})\bigr\}_{i=0}^{n_j}$$

The minimum temporal gap needed to guarantee horizontal separation $d_{sep}^H$ at approach speed $v_{close}$:

$$\Delta t_{sep} = \frac{d_{sep}^H}{v_{close}} + t_{buffer}$$

### Priority-Based Resolution

Each drone is assigned a priority score:

$$u_k = \alpha_u \cdot \frac{1}{t_{k,deadline} - t_{now}} + \beta_u \cdot \text{IsEmergency}_k + \gamma_u \cdot q_k$$

where $q_k$ is payload mass (heavier = higher priority), $\alpha_u, \beta_u, \gamma_u$ are weighting coefficients. Conflicts are resolved so that the lower-$u_k$ drone receives the full holding delay or is rerouted to an alternate corridor.

### Centralised LP Delay Optimisation

Let $\delta_k \geq 0$ be the departure time delay assigned to drone $k$. The conflict-free constraint between each conflicting pair $(k, j)$ requires either:

$$\delta_k - \delta_j \geq \Delta t_{sep} \quad \text{(k departs after j)}$$
$$\text{or} \quad \delta_j - \delta_k \geq \Delta t_{sep} \quad \text{(j departs after k)}$$

This is modelled as a binary-augmented LP (MILP). For the simulation, we relax it to a continuous LP by pre-assigning precedence from the FCFS order, reducing it to:

$$\min \sum_{k=1}^{N} \delta_k$$

$$\text{subject to: } \delta_k - \delta_j \geq \Delta t_{sep} \quad \forall (k,j) \in \mathcal{C}_{FCFS}$$

$$\delta_k \geq 0 \quad \forall k$$

where $\mathcal{C}_{FCFS}$ is the directed conflict graph under FCFS precedence. This LP has $N$ variables and $|\mathcal{C}|$ inequality constraints and is solvable in milliseconds by `scipy.optimize.linprog`.

### Separation Assurance Monitoring

During execution, the UTM monitors predicted position $\hat{\mathbf{p}}_k(t + T_{lookahead})$ using current velocity and trajectory plan. A **resolution advisory (RA)** is issued to drone $j$ when:

$$\|\hat{\mathbf{p}}_k(t + T_{lookahead}) - \hat{\mathbf{p}}_j(t + T_{lookahead})\|_H < d_{warn}^H = 1.5 \cdot d_{sep}^H$$

The RA command instructs drone $j$ to reduce speed to $v_{hold} = 0$ (hover) for duration $t_{RA}$:

$$t_{RA} = \frac{d_{warn}^H - \|\hat{\mathbf{p}}_k - \hat{\mathbf{p}}_j\|_H}{v_{close}} + t_{buffer}$$

### Total Delay and Throughput Metrics

Total fleet delay:

$$T_{delay}^{total} = \sum_{k=1}^{N} \bigl(t_k^{actual\_arrival} - t_k^{planned\_arrival}\bigr)$$

Mean delay per drone:

$$\bar{T}_{delay} = \frac{T_{delay}^{total}}{N}$$

Throughput (deliveries per minute):

$$\Theta = \frac{N}{T_{mission}} \times 60$$

Conflict rate (conflicts per drone-pair per minute):

$$\lambda_{conflict} = \frac{|\mathcal{C}|}{\binom{N}{2} \cdot T_{mission} / 60}$$

---

## Implementation

```python
import numpy as np
from scipy.optimize import linprog
from dataclasses import dataclass, field
from typing import List, Tuple, Optional, Dict

# Key constants
N_DRONES        = 12
N_CORRIDORS     = 16        # 4 per layer × 4 layers
V_CRUISE        = 10.0      # m/s
V_HOLD          = 0.0       # m/s during holding
D_SEP_H         = 30.0      # m horizontal separation minimum
D_SEP_V         = 10.0      # m vertical separation minimum
D_WARN_H        = 45.0      # m warning distance (1.5 × D_SEP_H)
T_LOOKAHEAD     = 15.0      # s separation assurance lookahead
T_BUFFER        = 2.0       # s extra buffer beyond geometric separation
RHO_MAX         = 3.0       # drones/km corridor capacity
DT              = 0.1       # s simulation timestep
ARENA           = (1000.0, 1000.0, 120.0)   # m

# Altitude layers [z_lo, z_hi, nominal heading (degrees)]
LAYERS = [
    (20.0,  40.0,  90.0),   # L1: Eastbound
    (40.0,  60.0,   0.0),   # L2: Northbound
    (60.0,  80.0, 270.0),   # L3: Westbound
    (80.0, 100.0, 180.0),   # L4: Southbound
]

@dataclass
class Corridor:
    id: int
    start: np.ndarray       # (x, y) m
    end:   np.ndarray       # (x, y) m
    z_lo:  float
    z_hi:  float
    width: float = 50.0     # m lane half-width

    @property
    def length(self) -> float:
        return float(np.linalg.norm(self.end - self.start))

    @property
    def unit_vec(self) -> np.ndarray:
        return (self.end - self.start) / self.length

    def contains_xy(self, p: np.ndarray) -> bool:
        """True if p is within corridor bounding lane."""
        r = p[:2] - self.start
        along   = float(np.dot(r, self.unit_vec))
        perp    = float(np.linalg.norm(r - along * self.unit_vec))
        return 0 <= along <= self.length and perp <= self.width


@dataclass
class DroneIntent:
    id: int
    waypoints: List[np.ndarray]    # 3D positions
    timestamps: List[float]        # arrival time at each waypoint
    priority: float = 1.0
    corridor_ids: List[int] = field(default_factory=list)

    def position_at(self, t: float) -> np.ndarray:
        if t <= self.timestamps[0]:
            return self.waypoints[0].copy()
        if t >= self.timestamps[-1]:
            return self.waypoints[-1].copy()
        for i in range(len(self.timestamps) - 1):
            if self.timestamps[i] <= t <= self.timestamps[i+1]:
                alpha = (t - self.timestamps[i]) / (
                    self.timestamps[i+1] - self.timestamps[i])
                return (1 - alpha) * self.waypoints[i] + alpha * self.waypoints[i+1]
        return self.waypoints[-1].copy()


def detect_conflicts(intents: List[DroneIntent],
                     d_sep_h: float, d_sep_v: float,
                     dt_check: float = 0.5
                     ) -> List[Tuple[int, int, float, float]]:
    """
    Return list of (id_k, id_j, t_conflict, min_separation) for all
    pairwise conflicts. Sampled at dt_check resolution.
    """
    conflicts = []
    t_start = min(i.timestamps[0] for i in intents)
    t_end   = max(i.timestamps[-1] for i in intents)
    t_grid  = np.arange(t_start, t_end, dt_check)

    for a in range(len(intents)):
        for b in range(a + 1, len(intents)):
            ka, kb = intents[a], intents[b]
            in_conflict = False
            t_first = None
            min_sep = np.inf
            for t in t_grid:
                pa = ka.position_at(t)
                pb = kb.position_at(t)
                dh = np.linalg.norm(pa[:2] - pb[:2])
                dv = abs(pa[2] - pb[2])
                sep = np.sqrt(dh**2 + dv**2)
                if dh < d_sep_h and dv < d_sep_v:
                    if not in_conflict:
                        t_first = t
                        in_conflict = True
                    min_sep = min(min_sep, sep)
            if in_conflict:
                conflicts.append((ka.id, kb.id, t_first, min_sep))
    return conflicts


def resolve_fcfs(intents: List[DroneIntent],
                 conflicts: List[Tuple[int, int, float, float]],
                 v_close: float,
                 d_sep_h: float,
                 t_buffer: float) -> Dict[int, float]:
    """
    Assign holding delays under FCFS precedence.
    Returns dict {drone_id: delay_seconds}.
    """
    delays = {i.id: 0.0 for i in intents}
    # Sort by first conflict time
    for id_k, id_j, t_conf, _ in sorted(conflicts, key=lambda x: x[2]):
        dt_sep = d_sep_h / v_close + t_buffer
        current_gap = delays[id_k] - delays[id_j]
        if current_gap < dt_sep:
            delays[id_j] += dt_sep - current_gap
    return delays


def resolve_lp(intents: List[DroneIntent],
               conflicts: List[Tuple[int, int, float, float]],
               v_close: float,
               d_sep_h: float,
               t_buffer: float) -> Dict[int, float]:
    """
    Minimise total delay using LP over departure time offsets.
    Precedence assigned by FCFS order (id_k has priority over id_j).
    """
    N = len(intents)
    id_to_idx = {i.id: n for n, i in enumerate(intents)}
    dt_sep = d_sep_h / v_close + t_buffer

    # Minimise: sum of delta_k
    c_obj = np.ones(N)
    A_ub, b_ub = [], []

    for id_k, id_j, *_ in conflicts:
        # delta_k - delta_j <= -dt_sep  =>  j must delay more than k by dt_sep
        row = np.zeros(N)
        row[id_to_idx[id_k]] =  1.0
        row[id_to_idx[id_j]] = -1.0
        A_ub.append(row)
        b_ub.append(-dt_sep)

    A_ub_arr = np.array(A_ub) if A_ub else np.empty((0, N))
    b_ub_arr = np.array(b_ub) if b_ub else np.empty(0)
    bounds = [(0, None)] * N

    result = linprog(c_obj, A_ub=A_ub_arr, b_ub=b_ub_arr,
                     bounds=bounds, method='highs')
    if result.success:
        return {i.id: float(result.x[n]) for n, i in enumerate(intents)}
    else:
        return {i.id: 0.0 for i in intents}


def simulate_utm(intents: List[DroneIntent],
                 delays: Dict[int, float],
                 dt: float = DT,
                 t_lookahead: float = T_LOOKAHEAD
                 ) -> Dict:
    """
    Execute all drone trajectories with applied delays.
    Monitor separation in real-time; issue RAs when separation < D_WARN_H.
    Returns trajectory history, RA events, and delivery times.
    """
    t_end = max(i.timestamps[-1] for i in intents) + max(delays.values()) + 60.0
    t_grid = np.arange(0, t_end, dt)

    history = {i.id: [] for i in intents}
    ra_events = []
    delivery_times = {}

    for i in intents:
        ts_shifted = [t + delays[i.id] for t in i.timestamps]
        i.timestamps = ts_shifted

    for t in t_grid:
        positions = {i.id: i.position_at(t) for i in intents}

        # Check real-time separation (separation assurance)
        for a_idx in range(len(intents)):
            for b_idx in range(a_idx + 1, len(intents)):
                ka, kb = intents[a_idx], intents[b_idx]
                pa_pred = ka.position_at(t + t_lookahead)
                pb_pred = kb.position_at(t + t_lookahead)
                dh_pred = np.linalg.norm(pa_pred[:2] - pb_pred[:2])
                dv_pred = abs(pa_pred[2] - pb_pred[2])
                if dh_pred < D_WARN_H and dv_pred < D_SEP_V:
                    ra_events.append({
                        't': t, 'drone_a': ka.id, 'drone_b': kb.id,
                        'dh_pred': dh_pred, 'dv_pred': dv_pred
                    })

        for i in intents:
            history[i.id].append((t, positions[i.id].copy()))
            if np.linalg.norm(positions[i.id] - i.waypoints[-1]) < 5.0:
                if i.id not in delivery_times:
                    delivery_times[i.id] = t

    return {'history': history, 'ra_events': ra_events,
            'delivery_times': delivery_times}
```

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Airspace volume | 1000 m × 1000 m × 120 m |
| Number of drones | 12 |
| Number of corridors | 16 (4 per altitude layer) |
| Number of altitude layers | 4 |
| Altitude layer bands | 20–40, 40–60, 60–80, 80–100 m |
| Horizontal separation minimum $d_{sep}^H$ | 30 m |
| Vertical separation minimum $d_{sep}^V$ | 10 m |
| Warning distance $d_{warn}^H$ | 45 m |
| Separation assurance lookahead $T_{la}$ | 15 s |
| Cruise speed $v_{cruise}$ | 10 m/s |
| Maximum corridor density $\rho_{max}$ | 3 drones/km |
| Temporal buffer $t_{buffer}$ | 2 s |
| LP solver | `scipy.optimize.linprog` (HiGHS) |
| Simulation timestep $\Delta t$ | 0.1 s |
| Conflict scan resolution | 0.5 s |

---

## Expected Output

- **3D airspace visualisation**: all 12 drone trajectories in distinct colours, corridor centre-lines shown as translucent tubes, conflict points marked as red spheres, RA events as orange diamonds; altitude layers shaded by band.
- **Conflict timeline**: horizontal bar chart showing each drone's mission duration with conflict intervals overlaid in red and RA events in orange; x-axis is simulation time.
- **Separation histogram**: distribution of minimum pairwise drone separations (horizontal and vertical) during simulation under each strategy; vertical dashed lines at $d_{sep}^H$ and $d_{sep}^V$.
- **Delay comparison bar chart**: total fleet delay and mean per-drone delay for FCFS vs Priority-Based vs LP Optimisation strategies.
- **Corridor utilisation heat-map**: instantaneous drone density $\rho_c(t)$ for each corridor vs time; horizontal dashed line at $\rho_{max}$.
- **Throughput and conflict rate table**: $\Theta$ (deliveries/min) and $\lambda_{conflict}$ for each strategy.
- **Animation (GIF)**: top-down 2D view, drones as coloured dots moving through corridors, separation violation regions pulsing red, RA events as brief flashes.

---

## Extensions

1. **Dynamic intent amendment**: allow drones to submit trajectory amendments mid-flight (e.g., emergency detour); UTM re-runs conflict detection and LP resolution in real time against all active intents.
2. **Geofence integration**: define no-fly zones as polygonal volumes; the intent registration step rejects any trajectory that intersects a geofence, and the routing layer automatically proposes an alternate corridor.
3. **Stochastic wind perturbation**: add Gaussian noise to drone positions ($\sigma = 2$ m) at each timestep; the separation assurance monitor must account for positional uncertainty using a buffered warning distance $d_{warn}^H = d_{sep}^H + 3\sigma$.
4. **Decentralised UTM (U-space)**: replace the centralised UTM with a peer-to-peer conflict negotiation protocol (TCAS-inspired); each drone pair independently negotiates resolution advisories through direct inter-drone communication.
5. **RL-based corridor assignment**: train a PPO agent to assign altitude layer and corridor to each incoming intent; reward = negative total delay, penalty = separation violation; compare against the LP baseline.

---

## Related Scenarios

- Prerequisites: [S029 Urban Logistics Scheduling](S029_urban_logistics_scheduling.md), [S033 Online Order Insertion](S033_online_order_insertion.md), [S034 Weather Rerouting](S034_weather_rerouting.md)
- Follow-ups: [S036 Last-Mile Relay](S036_last_mile_relay.md)
- Algorithm reference: LP conflict resolution relates to [S019 Dynamic Reassignment](../01_pursuit_evasion/S019_dynamic_reassignment.md) (Hungarian / LP assignment)

## References

- FAA (2020). *Unmanned Aircraft System (UAS) Traffic Management (UTM) Concept of Operations v2.0*. FAA.
- Prevot, T. et al. (2016). "UAS Traffic Management (UTM) Concept of Operations to Safely Enable Low Altitude Flight Operations." *AIAA Aviation Forum*, AIAA 2016-3292.
- Paielli, R.A. & Erzberger, H. (1997). "Conflict Probability Estimation for Free Flight." *Journal of Guidance, Control, and Dynamics*, 20(3), 588–596.
- Beard, R.W. & McLain, T.W. (2012). *Small Unmanned Aircraft: Theory and Practice*. Princeton University Press, Ch. 10.
