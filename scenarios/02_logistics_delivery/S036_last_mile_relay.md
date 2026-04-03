# S036 Last-Mile Relay

**Domain**: Logistics & Delivery | **Difficulty**: ⭐⭐⭐ | **Status**: `[ ]` Not started

---

## Problem Definition

**Setup**: A single parcel must be delivered from a source depot $D_0$ to a destination $D_K$ whose straight-line distance $L$ exceeds the maximum range $R$ of any individual drone. A chain of $K$ intermediate relay stations $\{W_1, W_2, \ldots, W_{K-1}\}$ is placed along (or near) the route. A fleet of $N \geq K$ drones is available; each drone handles exactly one leg $[W_{k-1}, W_k]$, lands, hands off the parcel, and recharges while a fresh drone continues. The total route is fixed (a known sequence of waypoints); the optimization decides the waypoint positions and leg assignment.

**Roles**:
- **Relay drones** ($N$ units): each carries the parcel for one leg, then returns to its home base or stays at the relay station for recharging.
- **Parcel**: a single unit of cargo passed from drone to drone at each relay waypoint.
- **Relay stations** ($K-1$ intermediate + 2 terminals): rendezvous points where handoff and recharging occur.

**Objective**:

1. **Feasibility**: every leg distance $d_k = \|\mathbf{w}_k - \mathbf{w}_{k-1}\|$ must satisfy $d_k \leq R$ (range constraint).
2. **Optimality**: place the $K-1$ relay stations to minimise the **bottleneck leg** (the longest single leg), which directly minimises the worst-case battery consumption per drone:

$$\min_{\{W_k\}} \max_{k=1}^{K} \|\mathbf{w}_k - \mathbf{w}_{k-1}\|$$

3. **Timing**: minimise total end-to-end delivery time including handoff dwell time $\tau_h$ at each relay station.

**Comparison**: relay chain (K legs) vs direct single-drone attempt (fails when $L > R$); relay chain vs naive equal-spacing (suboptimal on non-straight routes).

---

## Mathematical Model

### Leg Distance and Range Constraint

Let waypoints be $\mathbf{w}_0 = D_0,\; \mathbf{w}_1, \ldots, \mathbf{w}_{K-1},\; \mathbf{w}_K = D_K \in \mathbb{R}^2$.

Each leg length:

$$d_k = \|\mathbf{w}_k - \mathbf{w}_{k-1}\|_2, \quad k = 1, \ldots, K$$

Feasibility requires:

$$d_k \leq R \quad \forall\, k$$

### Optimal Relay Station Placement (Collinear Case)

When the route is a straight line of length $L$, the optimal equal-spacing solution gives:

$$d_k^* = \frac{L}{K}, \quad \mathbf{w}_k^* = \mathbf{w}_0 + \frac{k}{K}(\mathbf{w}_K - \mathbf{w}_0)$$

The minimum number of legs needed to cover distance $L$ with range $R$:

$$K^* = \left\lceil \frac{L}{R} \right\rceil$$

### General Route: Relay TSP Segmentation

For a route with mandatory intermediate via-points $\{V_1, \ldots, V_M\}$ (e.g., forced by airspace restrictions), the relay stations are chosen from candidate locations $\mathcal{C}$. Define the assignment as a segmentation of the ordered via-point sequence. The bottleneck minimisation is solved greedily:

```
# Greedy furthest-reachable segmentation
legs = []
start = w_0
remaining = [V_1, ..., V_M, D_K]
while remaining:
    # advance as far as possible within range R
    reach = [p for p in remaining if dist(start, p) <= R]
    if not reach:
        raise InfeasibleRouteError
    cut = reach[-1]   # furthest reachable point
    legs.append((start, cut))
    start = cut
    remaining = remaining[len(reach):]
```

This greedy strategy is **optimal** for minimising the number of legs $K$ on a fixed ordered waypoint sequence.

### Battery State per Leg

Each drone starts each leg fully charged. Normalised battery consumption for leg $k$:

$$B_k = \frac{d_k}{R} \in (0,\; 1]$$

Reserve constraint (10% safety margin):

$$B_k \leq 1 - B_{reserve} = 0.90$$

Effective range with reserve:

$$R_{eff} = 0.90 \cdot R$$

### Delivery Time Model

Flight time for leg $k$ at cruise speed $v_c$:

$$t_k^{flight} = \frac{d_k}{v_c}$$

Total delivery time including $K-1$ handoff dwells:

$$T_{delivery} = \sum_{k=1}^{K} t_k^{flight} + (K - 1) \cdot \tau_h$$

where $\tau_h$ is the handoff dwell time (parcel transfer + fresh drone spin-up).

### Handoff Protocol

At relay station $W_k$, drone $k$ arrives at time $t_{arrive,k}$. Drone $k+1$ must be ready (charged and positioned) before drone $k$ arrives. If drone $k+1$ departs its home base at time $t_{depart,k+1}$, the rendezvous condition is:

$$t_{depart,k+1} + \frac{d_{home,k+1}}{v_c} \leq t_{arrive,k} + \tau_h$$

Pre-positioning time for drone $k+1$ to be at $W_k$ on time:

$$t_{depart,k+1} \leq t_{arrive,k} + \tau_h - \frac{\|\mathbf{w}_k - \mathbf{p}_{home,k+1}\|}{v_c}$$

### 2-opt Improvement for Multi-Leg Routes

When relay station positions are free to optimise (not constrained to a fixed corridor), a 2-opt swap on the ordered leg sequence tests whether reversing a sub-segment reduces the bottleneck leg:

$$\text{gain}(i,j) = \max(d_i, d_{i+1}, \ldots, d_j) - \max(d_i', d_{i+1}', \ldots, d_j')$$

Accept the swap if $\text{gain}(i,j) > 0$.

---

## Implementation

```python
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

# Key constants
TOTAL_DISTANCE  = 120.0   # m  (source to destination, straight line)
DRONE_RANGE     = 35.0    # m  (max range per charge)
RANGE_RESERVE   = 0.10    # 10% battery reserve
CRUISE_SPEED    = 5.0     # m/s
HANDOFF_DWELL   = 3.0     # s  (parcel transfer time at each relay)
DT              = 0.05    # s  (simulation timestep)

R_EFF = DRONE_RANGE * (1 - RANGE_RESERVE)   # 31.5 m

# Source and destination (2D top-down)
SOURCE = np.array([0.0, 0.0])
DEST   = np.array([120.0, 0.0])

def greedy_segment(waypoints, r_eff):
    """Greedy furthest-reachable segmentation of an ordered waypoint list."""
    legs = []
    start_idx = 0
    while start_idx < len(waypoints) - 1:
        reachable = [
            i for i in range(start_idx + 1, len(waypoints))
            if np.linalg.norm(waypoints[i] - waypoints[start_idx]) <= r_eff
        ]
        if not reachable:
            raise ValueError(f"No reachable waypoint from index {start_idx}")
        end_idx = reachable[-1]
        legs.append((start_idx, end_idx))
        start_idx = end_idx
    return legs

def delivery_time(legs, waypoints, speed, dwell):
    total = 0.0
    for start_idx, end_idx in legs:
        d = np.linalg.norm(waypoints[end_idx] - waypoints[start_idx])
        total += d / speed
    total += (len(legs) - 1) * dwell
    return total

def simulate_relay(waypoints, legs, speed, dt):
    """Simulate parcel movement through the relay chain."""
    records = []   # list of (time, position) tuples
    t = 0.0
    for leg_num, (si, ei) in enumerate(legs):
        p_start = waypoints[si].copy()
        p_end   = waypoints[ei].copy()
        direction = (p_end - p_start) / np.linalg.norm(p_end - p_start)
        dist = np.linalg.norm(p_end - p_start)
        elapsed = 0.0
        while elapsed < dist / speed:
            pos = p_start + direction * speed * elapsed
            records.append((t, pos.copy(), leg_num))
            elapsed += dt
            t += dt
        # Arrive at relay station; dwell
        records.append((t, p_end.copy(), leg_num))
        if leg_num < len(legs) - 1:
            t += HANDOFF_DWELL   # handoff pause
    return records
```

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Source to destination distance | 120 m |
| Drone max range | 35 m |
| Battery reserve margin | 10% |
| Effective range per leg | 31.5 m |
| Minimum legs required $K^*$ | $\lceil 120 / 31.5 \rceil = 4$ |
| Cruise speed | 5 m/s |
| Handoff dwell time $\tau_h$ | 3 s |
| Relay station positions (equal spacing) | 30, 60, 90 m along route |
| Total flight time (no dwell) | 120 / 5 = 24 s |
| Total delivery time (with dwell) | 24 + 3 × 3 = 33 s |
| Simulation timestep | 0.05 s |

---

## Expected Output

- **2D top-down route map**: source (green square), destination (red star), relay stations (orange circles), each leg drawn in a distinct colour; battery consumption percentage annotated on each leg.
- **Parcel position vs time**: x-coordinate of the parcel over the full delivery timeline; vertical dashed lines mark handoff events.
- **Battery state per drone**: bar chart of $B_k$ for each leg drone; horizontal dashed line at 0.90 (reserve limit).
- **Delivery time breakdown**: stacked bar showing flight time per leg and dwell time at each relay station.
- **Comparison table**: equal-spacing relay (optimal for straight route) vs greedy segmentation on a bent corridor route; number of legs, bottleneck leg length, total delivery time.

---

## Extensions

1. **Non-straight corridor**: add 2–3 forced via-points (simulating airspace restrictions) and compare greedy segmentation vs equal-spacing.
2. **Heterogeneous fleet**: drones with different ranges $R_i$; assign longer legs to higher-range drones using a min-cost assignment (Hungarian).
3. **Parallel relay chains**: two parcels dispatched simultaneously on staggered chains sharing relay stations; analyse throughput vs single-chain latency.
4. **Dynamic handoff failure**: simulate a relay drone that fails to appear at $W_k$; the upstream drone must loiter until a backup arrives, trading battery reserve for resilience.
5. **Multi-parcel TSP relay**: multiple source-destination pairs share a common relay infrastructure; optimise relay station placement jointly across all routes.

---

## Related Scenarios

- Prerequisites: [S021](S021_point_delivery.md), [S029](S029_urban_scheduling.md)
- Follow-ups: [S033](S033_online_order_insertion.md) (online insertion), [S037](S037_reverse_logistics.md) (return path optimisation), [S040](S040_fleet_load_balancing.md) (load balancing)
- Domain reference: [domains/02_logistics_delivery/README.md](../../domains/02_logistics_delivery/README.md)
