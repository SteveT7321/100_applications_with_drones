# Domain 2: Logistics & Delivery

## Algorithm Overview

The core problems of drone logistics span three mathematical domains:

```
Path Planning ──→ Trajectory Optimization ──→ Task Assignment
(where to go)       (how to fly)               (who does what)
```

---

## Part 1 — Path Planning

### 1.1 Graph Search

| Algorithm | Complexity | Optimality | Best Used When |
|-----------|-----------|------------|----------------|
| Dijkstra | O((V+E) log V) | Optimal | No heuristic estimate available |
| A\* | O((V+E) log V) | Optimal (admissible heuristic required) | Goal position is known |
| D\* Lite | O(k log k) incremental | Optimal | Dynamic environment, replanning needed |
| Theta\* | Same as A\* | Optimal (any-angle) | Smooth, non-grid paths required |

Heuristic function selection:
- **Manhattan distance**: grid maps, 4-directional movement only
- **Euclidean distance**: continuous space, any-direction movement
- **Octile distance**: grid maps, 8-directional movement (including diagonals)

Admissibility condition: h(n) ≤ h*(n) (heuristic never overestimates true cost) — required for A\* to guarantee optimal solution.

### 1.2 Sampling-Based Planning

| Algorithm | Completeness | Optimality | Key Property |
|-----------|-------------|------------|--------------|
| RRT | Probabilistically complete | No | Fast exploration in high-dimensional spaces; no analytical obstacle representation needed |
| RRT\* | Probabilistically complete | Asymptotically optimal | Continuous rewiring improves path quality as samples grow |
| PRM | Probabilistically complete | No | Pre-builds roadmap; efficient for multiple queries on same scene |
| Informed RRT\* | Probabilistically complete | Asymptotically optimal | After initial solution found, restricts sampling to an ellipsoid for faster convergence |

RRT\* rewire condition:
```
# After adding x_new, try to improve paths to nearby nodes
for x_near in Near(x_new, r):
    if cost(x_new) + d(x_new, x_near) < cost(x_near):
        parent(x_near) = x_new   # re-parent the neighbor
        cost(x_near) = cost(x_new) + d(x_new, x_near)
```

### 1.3 Multi-Agent Path Finding (MAPF)

| Method | Complete | Optimal | Notes |
|--------|----------|---------|-------|
| Priority Planning | No | No | Simple and fast; higher-priority agents plan first |
| CBS (Conflict-Based Search) | Yes | Yes | Branch-and-bound; adds constraints at each conflict |
| Time Separation | No | No | Fixed altitude offsets per agent; practical engineering solution |
| Velocity Obstacle (VO) | No | No | Continuous space; real-time local collision avoidance |

CBS two-level architecture:
```
High-level tree:
  Node = a set of constraints {(agent_i, location, time), ...}
  Find the first conflict (i, j, v, t) in the current solution
  Branch: add constraint (i, v, t) OR (j, v, t)

Low-level:
  Each agent runs space-time A* under its own constraints
```

---

## Part 2 — Trajectory Optimization

### 2.1 Minimum Snap Trajectory

Smooth trajectories are essential for drone dynamics. Minimizing **snap** (4th derivative of position) reduces thrust oscillation:

```
min ∫₀ᵀ ||d⁴r/dt⁴||² dt

subject to:
  r(t_i) = waypoint_i          # waypoint position constraints
  ṙ(0) = v₀, r̈(0) = a₀        # initial boundary conditions
  ṙ(T) = vT, r̈(T) = aT        # final boundary conditions
  continuity at segment joins: position / velocity / acceleration / jerk
```

Piecewise polynomial (degree k per segment):
```
r_j(t) = Σᵢ cᵢⱼ · tⁱ,  i = 0..k
Formulated as QP:  min cᵀQc,  s.t. Ac = b
```

### 2.2 Time-Optimal vs Energy-Optimal

| Objective | Cost Function | Typical Use Case |
|-----------|--------------|-----------------|
| Minimum time | min T | Emergency delivery, racing |
| Minimum energy | min ∫ \|\|u\|\|² dt | Long range, battery-constrained |
| Weighted multi-objective | min αT + β∫ \|\|u\|\|² dt | General logistics (tune α/β) |

Pontryagin's minimum principle (Bang-Bang time-optimal control):
```
For thrust-limited systems, the time-optimal solution is:
u*(t) ∈ {u_max, u_min}  (thrust switches between upper and lower bounds)
```

### 2.3 Disturbance Rejection

Dryden wind turbulence model (low-frequency transfer function):
```
W(s) = σ · √(2L / πV) · 1 / (1 + (L/πV)·s)

σ = turbulence intensity, L = turbulence scale length, V = airspeed
```

PID + Feedforward compensation:
```
u = Kp·e + Ki·∫e dt + Kd·ė + u_ff

u_ff = estimated wind force (from EKF or disturbance observer)
Goal: steady-state error → 0, wind disturbance rejected quickly
```

### 2.4 CoG Offset Compensation

Asymmetric payload shifts the center of gravity by Δr = [Δx, Δy, Δz]:
```
Corrective torque: τ_correction = -m_payload · g × Δr
The attitude controller requires a feedforward term;
without it, the drone tilts at hover to maintain position.
```

---

## Part 3 — Task Assignment

### 3.1 TSP and VRP Problem Family

```
TSP ────→ VRP ────→ CVRP ────→ DVRP
(1 vehicle) (multi)  (+capacity)  (+dynamic orders)
                ↓
           Multi-depot VRP
                ↓
           VRP with Time Windows (VRPTW)
```

| Problem | Decision Variables | Additional Constraints | Complexity |
|---------|-------------------|----------------------|------------|
| TSP | Visit order (single vehicle) | None | NP-hard |
| VRP | Route assignment (multi-vehicle) | Fleet size | NP-hard |
| CVRP | Same | Capacity limit Q | NP-hard |
| DVRP | Same + replanning | Dynamic order arrivals | NP-hard + online |
| Multi-depot VRP | Same | Multiple depots | NP-hard |

### 3.2 TSP Heuristics

| Algorithm | Solution Quality | Time Complexity | Description |
|-----------|-----------------|-----------------|-------------|
| Nearest Neighbor | ~20–25% above optimal | O(n²) | Greedy: always visit the closest unvisited node |
| 2-opt | ~5–10% above optimal | O(n²) per iter | Swap two edges; reverse the segment between them |
| 3-opt | ~3–5% above optimal | O(n³) per iter | Swap three edges simultaneously |
| Or-opt | ~5% above optimal | O(n²) per iter | Relocate 1/2/3 consecutive nodes to a better position |
| Christofides | ≤ 1.5× optimal | O(n³) | Only heuristic with a proven approximation guarantee |

2-opt improvement step:
```
for i in range(n):
  for j in range(i+2, n):
    # Compare original: ...→A→B→...→C→D→...
    # vs reversed:      ...→A→C→...→B→D→...
    if d(A,B) + d(C,D) > d(A,C) + d(B,D):
        reverse(route[i+1:j+1])
```

### 3.3 VRP Heuristics

**Clarke-Wright Savings Algorithm**:
```
Start: each order has its own depot→i→depot route
savings(i, j) = d(depot, i) + d(depot, j) - d(i, j)
Sort savings descending; merge routes greedily (respecting capacity)
```

**Online Insertion (Cheapest Insertion Heuristic)**:
```
When new order o arrives:
for each existing route r:
  for each insertion position k in r:
    cost_insert(r, k) = d(k-1, o) + d(o, k) - d(k-1, k)
Choose (r, k) with minimum cost_insert; open a new route if none fits
```

### 3.4 Hungarian Algorithm (Optimal Assignment)

Fully implemented in S018 (Domain 01): O(n³) minimum-weight perfect matching on a cost matrix.
Reusable in Domain 02 for:
- S030 Multi-Depot: drone ↔ depot assignment
- S032 Charging Queue: drone ↔ charging slot assignment
- S040 Fleet Load Balancing: task ↔ drone reassignment

---

## Part 4 — Special Multi-Drone Problems

### 4.1 Cooperative Heavy Lift

n drones suspend a single load via cables. Cable tension distribution:

Static equilibrium conditions:
```
Σᵢ Fᵢ = m_load · g · ẑ    (resultant force equals gravity)
Σᵢ (rᵢ × Fᵢ) = 0          (moment balance)
```

This forms an underdetermined linear system A·f = b (when n > 3).
Minimum-tension solution via pseudo-inverse:
```
f* = Aᵀ(AAᵀ)⁻¹b
```
Additional freedom can be used in QP to enforce fᵢ ≥ f_min (prevent cable slack).

### 4.2 Moving Platform Landing (Offshore Exchange)

Ship deck motion modeled as superimposed sinusoids (simplified 6-DOF):
```
z(t) = A_heave · sin(ω_heave · t + φ_h)
θ(t) = A_roll  · sin(ω_roll  · t + φ_r)
ψ(t) = A_pitch · sin(ω_pitch · t + φ_p)
```

Landing window detection:
```
Safe to land iff:
  |θ(t)| < θ_max  AND
  |ψ(t)| < ψ_max  AND
  |θ̇(t)| < ω_max
Predict the center of the next window → plan approach trajectory
```

### 4.3 Aerial Refueling Rendezvous

Relative motion model (tanker as reference frame):
```
Δr = r_receiver - r_tanker
Δv = v_receiver - v_tanker

Goal: Δr → 0, Δv → 0  (soft docking)
Control law: u = -Kp·Δr - Kd·Δv
```

---

## 2D / 3D Classification

Domain 02 dimensions are decided at design time — **no `3d/` subfolder will be added later**:

| Dimension | Scenarios | Reason |
|-----------|-----------|--------|
| **Native 3D** | S021–S028, S034, S038, S039 | Flight physics, wind fields, attitude, cable forces, landing — z-axis dynamics matter |
| **Native 2D** (top-down) | S029–S033, S035–S037, S040 | Routing / scheduling optimization — z-axis is irrelevant to the decision |

Visualization conventions:
- 3D scenes: `mpl_toolkits.mplot3d`, same color scheme as Domain 01
- 2D scenes: `plt.subplots()`, top-down routing map with depot/order nodes

---

## Scenario List with Algorithm Mapping

### Single-Drone Basics (S021–S025) — All 3D

| Scenario | Core Algorithm | Key Math |
|----------|---------------|----------|
| S021 Point Delivery | A\* | Heuristic search, grid map, admissibility |
| S022 Obstacle Avoidance | RRT\* | Sampling-based planning, asymptotic optimality, rewire |
| S023 Moving Landing Pad | Prediction + landing window | Dynamic target prediction, landing window detection |
| S024 Wind Compensation | PID + Dryden wind model | Disturbance modeling, feedforward compensation |
| S025 Payload CoG Offset | Attitude feedforward | CoG shift, corrective torque |

### Nv1 (S026–S028) — All 3D

| Scenario | Core Algorithm | Key Math |
|----------|---------------|----------|
| S026 Cooperative Heavy Lift | Cable tension QP | Static equilibrium, pseudo-inverse, QP |
| S027 Aerial Refueling | Rendezvous control | Relative motion, soft docking |
| S028 Cargo Escort | Formation control | Leader-follower, APF formation keeping |

### NvM (S029–S040) — Mixed

| Scenario | Core Algorithm | Dimension |
|----------|---------------|-----------|
| S029 Urban Scheduling | VRP + Clarke-Wright savings | 2D |
| S030 Multi-Depot | Multi-depot VRP + Hungarian | 2D |
| S031 Path Deconfliction | CBS / time separation | 2D+t |
| S032 Charging Queue | Battery-aware scheduling | 2D |
| S033 Online Order Insertion | Online VRP + cheapest insertion | 2D |
| S034 Weather Rerouting | D\* Lite dynamic replanning | 3D |
| S035 UTM Simulation | Time-slot corridor allocation | 2D |
| S036 Last-Mile Relay | Relay TSP (segmented handoff) | 2D |
| S037 Reverse Logistics | Return path TSP + 2-opt | 2D |
| S038 Disaster Relief | Minimum snap airdrop trajectory | 3D |
| S039 Offshore Exchange | Moving platform landing window | 3D |
| S040 Fleet Load Balancing | Dynamic reallocation DP | 2D |

---

## Common Physical Settings

```python
# Standard logistics scenario settings
DELIVERY_HEIGHT   = 20.0  # m (urban delivery flight altitude)
MAX_PAYLOAD       = 0.1   # kg (CF2X maximum payload, conservative estimate)
BATTERY_CAPACITY  = 1.0   # normalized (1.0 = full charge)
HOVER_DRAIN       = 0.01  # consumption per second (hovering)
FLIGHT_DRAIN      = 0.015 # consumption per second (flying)
LANDING_THRESHOLD = 0.05  # m (successful landing detection threshold)
```

---

## Key Metrics

```python
metrics = {
    "delivery_success_rate": float,   # fraction of orders successfully delivered
    "total_distance":        float,   # total flight distance across all drones (m)
    "total_time":            float,   # time to complete all deliveries (s)
    "battery_remaining":     list,    # remaining battery per drone [0.0, 1.0]
    "landing_accuracy":      float,   # landing RMSE (m)
    "conflict_count":        int,     # number of path conflicts detected
    "route_cost":            float,   # total VRP route cost
}
```

---

## Related Documents

- [MATH_FOUNDATIONS.md §3](../MATH_FOUNDATIONS.md) — Path planning mathematics
- [docs/algorithm_index.md](../docs/algorithm_index.md) — VRP, A*, RRT* reference index
- [S018 Multi-Target Intercept](../scenarios/01_pursuit_evasion/S018_multi_target_intercept.md) — Hungarian algorithm implementation reference
