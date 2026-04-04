# S031 Path De-confliction

**Domain**: Logistics & Delivery | **Difficulty**: ⭐⭐⭐ | **Status**: `[x]` Complete

---

## Problem Definition

**Setup**: A fleet of $N = 8$ delivery drones departs simultaneously from a shared depot and flies
pre-planned straight-line routes to distinct delivery waypoints within a $500 \times 500$ m urban
airspace. All drones cruise at the same altitude $z = 20$ m and the same speed $v = 12$ m/s. Because
routes are generated independently (e.g., by S029 VRPTW), pairs of routes may cross or come
dangerously close in both space and time, creating a potential mid-air conflict.

The simulation implements three de-confliction strategies and compares the number of conflicts
resolved, total added flight distance, and maximum throughput delay.

**Roles**:
- **Drones** $D_1, \ldots, D_N$: each assigned one origin–destination pair
  $(\mathbf{o}_i, \mathbf{g}_i)$; their nominal paths are straight-line segments
- **Conflict detector**: continuously monitors all $\binom{N}{2}$ drone pairs and flags a conflict
  when predicted closest point of approach (CPA) falls below the separation threshold $d_{sep}$
- **De-confliction planner**: applies one of three resolution strategies to the flagged pair

**Objective**:
1. Ensure no two drones ever violate the minimum separation $d_{sep} = 5$ m (safety constraint)
2. Minimise total added flight distance summed over the fleet
3. Minimise maximum arrival delay over all drones

**Comparison strategies**:
1. **Priority-based speed adjustment** — lower-priority drone decelerates until the conflict clears
2. **Horizontal waypoint insertion** — one drone detours around a lateral avoidance waypoint
3. **Altitude layer assignment** — each drone is pre-assigned a unique altitude layer, eliminating
   horizontal conflicts by vertical separation

---

## Mathematical Model

### Nominal Straight-Line Trajectory

Drone $i$ departs at $t = 0$ from $\mathbf{o}_i$ and flies to $\mathbf{g}_i$ at constant speed $v$.
Its nominal position at time $t$:

$$\mathbf{p}_i(t) = \mathbf{o}_i + v \cdot t \cdot \hat{\mathbf{u}}_i,
  \qquad \hat{\mathbf{u}}_i = \frac{\mathbf{g}_i - \mathbf{o}_i}{\|\mathbf{g}_i - \mathbf{o}_i\|}$$

Nominal arrival time for drone $i$:

$$T_i^{nom} = \frac{\|\mathbf{g}_i - \mathbf{o}_i\|}{v}$$

### Conflict Detection via Closest Point of Approach (CPA)

For a pair $(i, j)$ flying with velocity vectors $\mathbf{v}_i$ and $\mathbf{v}_j$, the relative
position and velocity are:

$$\Delta\mathbf{p}_{ij}(t) = \mathbf{p}_i(t) - \mathbf{p}_j(t), \qquad
  \Delta\mathbf{v}_{ij} = \mathbf{v}_i - \mathbf{v}_j$$

The time of closest point of approach:

$$t_{CPA} = -\frac{\Delta\mathbf{p}_{ij}(0) \cdot \Delta\mathbf{v}_{ij}}
                  {\|\Delta\mathbf{v}_{ij}\|^2}$$

The CPA distance (minimum predicted separation):

$$d_{CPA} = \|\Delta\mathbf{p}_{ij}(t_{CPA})\|$$

A conflict is declared when:

$$d_{CPA} < d_{sep} \quad \text{and} \quad t_{CPA} \in [0,\; T_{horizon}]$$

where $T_{horizon} = 30$ s is the look-ahead window.

### Strategy 1 — Priority-Based Speed Adjustment

Drones are ranked by priority $\pi_i$ (lower index = higher priority). When pair $(i, j)$ with
$\pi_i > \pi_j$ conflicts, drone $i$ decelerates to speed $v' < v$ so that $t_{CPA}$ shifts beyond
the conflict window. The required speed reduction:

Solve for $v'$ such that, under the modified trajectory
$\mathbf{p}'_i(t) = \mathbf{o}_i + v' t \hat{\mathbf{u}}_i$, the new CPA distance satisfies
$d'_{CPA} \geq d_{sep} + \delta_{margin}$.

Since $t_{CPA}$ depends on $v'$, the scalar equation is solved numerically (bisection on
$v' \in [v_{min},\, v]$). The drone resumes $v$ once the conflict has passed.

Added distance from speed reduction alone is zero; the delay penalty is:

$$\Delta T_i^{speed} = T_i^{nom} \cdot \left(\frac{v}{v'} - 1\right)$$

### Strategy 2 — Horizontal Waypoint Insertion

For the lower-priority drone $i$ in conflict pair $(i, j)$, insert a lateral avoidance waypoint
$\mathbf{w}_{avoid}$ at the predicted conflict midpoint plus a lateral offset $\rho$:

$$\mathbf{w}_{avoid} = \mathbf{p}_{mid} + \rho \cdot \hat{\mathbf{n}}_{ij}$$

where $\mathbf{p}_{mid} = \tfrac{1}{2}(\mathbf{p}_i(t_{CPA}) + \mathbf{p}_j(t_{CPA}))$ is the
predicted midpoint at CPA time, and $\hat{\mathbf{n}}_{ij}$ is the unit normal to the relative
velocity in the horizontal plane:

$$\hat{\mathbf{n}}_{ij} = \frac{\Delta\mathbf{v}_{ij}^{\perp}}{\|\Delta\mathbf{v}_{ij}^{\perp}\|}$$

The detour offset is chosen as $\rho = d_{sep} + \delta_{margin}$.

Added flight distance for the detour:

$$\Delta L_i = \|\mathbf{o}_i - \mathbf{w}_{avoid}\| + \|\mathbf{w}_{avoid} - \mathbf{g}_i\|
              - \|\mathbf{g}_i - \mathbf{o}_i\|$$

### Strategy 3 — Altitude Layer Assignment

Each drone $i$ is pre-assigned a unique altitude layer before departure:

$$z_i = z_{base} + (i - 1) \cdot \Delta z, \qquad i = 1, \ldots, N$$

with $z_{base} = 18$ m and $\Delta z = 2$ m, giving a layer range of $[18, 32]$ m for $N = 8$.

Vertical separation between any two drones is guaranteed as long as $\Delta z \geq d_{sep}$. No
horizontal manoeuvres are required, so the horizontal paths are unchanged. The only cost is the
per-drone altitude adjustment from the nominal cruise altitude:

$$\Delta z_i = |z_i - z_{nom}|, \qquad z_{nom} = 20\text{ m}$$

### Conflict Count and Fleet Metrics

Total residual conflicts at time $t$ under strategy $s$:

$$C_s(t) = \sum_{i < j} \mathbf{1}\!\left[\|\mathbf{p}_i(t) - \mathbf{p}_j(t)\| < d_{sep}\right]$$

Total added flight distance across the fleet:

$$\Delta L_{total}^{(s)} = \sum_{i=1}^{N} \Delta L_i^{(s)}$$

Maximum arrival delay:

$$\Delta T_{max}^{(s)} = \max_{i \in \{1,\ldots,N\}} \left(T_i^{actual} - T_i^{nom}\right)$$

---

## Implementation

```python
import numpy as np
from itertools import combinations

# Key constants
N_DRONES    = 8
V_DRONE     = 12.0      # m/s cruise speed (nominal)
V_MIN       = 4.0       # m/s minimum speed for strategy 1
D_SEP       = 5.0       # m minimum separation threshold
D_MARGIN    = 2.0       # m extra clearance added on top of D_SEP
T_HORIZON   = 30.0      # s conflict look-ahead window
DT          = 0.1       # s simulation timestep
Z_NOM       = 20.0      # m nominal cruise altitude
DZ_LAYER    = 2.0       # m altitude layer spacing (strategy 3)
ARENA       = 500.0     # m half-width of delivery area
DEPOT       = np.array([250.0, 250.0, Z_NOM])

rng = np.random.default_rng(7)
GOALS = rng.uniform(20, 480, (N_DRONES, 2))
GOALS = np.c_[GOALS, np.full(N_DRONES, Z_NOM)]

# --- CPA computation ---
def cpa(pos_i, vel_i, pos_j, vel_j):
    """Return (t_cpa, d_cpa) for two drones with constant velocities."""
    dp = pos_i - pos_j
    dv = vel_i - vel_j
    dv2 = dv @ dv
    if dv2 < 1e-12:
        return 0.0, np.linalg.norm(dp)
    t = -dp @ dv / dv2
    t = max(t, 0.0)
    d = np.linalg.norm(dp + dv * t)
    return t, d

# --- Detect all conflicts in current state ---
def detect_conflicts(positions, velocities, t_horizon, d_sep):
    conflicts = []
    for i, j in combinations(range(len(positions)), 2):
        t_c, d_c = cpa(positions[i], velocities[i], positions[j], velocities[j])
        if d_c < d_sep and 0 < t_c <= t_horizon:
            conflicts.append((i, j, t_c, d_c))
    return conflicts

# --- Strategy 1: speed adjustment (bisection) ---
def resolve_speed(drone_i, origin_i, goal_i, vel_j, pos_j, d_sep, d_margin, v_min):
    unit_i = (goal_i - origin_i) / np.linalg.norm(goal_i - origin_i)
    lo, hi = v_min, np.linalg.norm(drone_i['vel'])
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        new_vel = mid * unit_i
        _, d_c = cpa(drone_i['pos'], new_vel, pos_j, vel_j)
        if d_c >= d_sep + d_margin:
            lo = mid
        else:
            hi = mid
    return lo * unit_i   # reduced velocity vector

# --- Strategy 2: lateral waypoint insertion ---
def avoidance_waypoint(pos_i, pos_j, vel_i, vel_j, d_sep, d_margin):
    t_c, _ = cpa(pos_i, vel_i, pos_j, vel_j)
    mid = 0.5 * (pos_i + vel_i * t_c + pos_j + vel_j * t_c)
    dv = vel_i - vel_j
    # perpendicular in horizontal plane
    perp = np.array([-dv[1], dv[0], 0.0])
    norm = np.linalg.norm(perp)
    if norm < 1e-9:
        perp = np.array([1.0, 0.0, 0.0])
    else:
        perp /= norm
    return mid + (d_sep + d_margin) * perp

# --- Strategy 3: pre-assign altitude layers ---
def assign_altitude_layers(n, z_base, dz):
    return np.array([z_base + i * dz for i in range(n)])

# --- Simulation loop skeleton ---
def simulate(strategy, seed=7):
    """
    strategy: 'speed' | 'waypoint' | 'altitude'
    Returns: history (list of position arrays per timestep),
             total_added_distance, max_arrival_delay, conflict_count
    """
    rng_sim = np.random.default_rng(seed)
    goals = rng_sim.uniform(20, 480, (N_DRONES, 2))
    goals = np.c_[goals, np.full(N_DRONES, Z_NOM)]

    # Altitude layer pre-assignment
    if strategy == 'altitude':
        z_layers = assign_altitude_layers(N_DRONES, Z_NOM - 2.0, DZ_LAYER)
        goals[:, 2] = z_layers

    origins = np.tile(DEPOT, (N_DRONES, 1)).astype(float)
    origins[:, 2] = goals[:, 2] if strategy == 'altitude' else Z_NOM
    positions = origins.copy()
    nominal_times = np.linalg.norm(goals - origins, axis=1) / V_DRONE
    speeds = np.full(N_DRONES, V_DRONE)
    extra_waypoints = [None] * N_DRONES   # optional avoidance waypoints
    arrived = np.zeros(N_DRONES, dtype=bool)
    arrival_times = np.full(N_DRONES, np.nan)

    history = [positions.copy()]
    total_conflict_events = 0
    t = 0.0

    while not np.all(arrived) and t < 300.0:
        # Compute velocities toward current targets
        targets = np.where(
            np.array([w is not None for w in extra_waypoints])[:, None],
            np.array([w if w is not None else goals[k] for k, w in enumerate(extra_waypoints)]),
            goals
        )
        directions = targets - positions
        distances  = np.linalg.norm(directions, axis=1, keepdims=True)
        safe_dist  = np.where(distances > 1e-6, distances, 1.0)
        unit_dirs  = directions / safe_dist
        velocities = speeds[:, None] * unit_dirs

        if strategy != 'altitude':
            conflicts = detect_conflicts(positions, velocities, T_HORIZON, D_SEP)
            total_conflict_events += len(conflicts)
            for (i, j, t_c, d_c) in conflicts:
                if strategy == 'speed':
                    velocities[i] = resolve_speed(
                        {'pos': positions[i], 'vel': velocities[i]},
                        origins[i], goals[i], velocities[j], positions[j],
                        D_SEP, D_MARGIN, V_MIN
                    )
                    speeds[i] = np.linalg.norm(velocities[i])
                elif strategy == 'waypoint' and extra_waypoints[i] is None:
                    extra_waypoints[i] = avoidance_waypoint(
                        positions[i], positions[j],
                        velocities[i], velocities[j],
                        D_SEP, D_MARGIN
                    )

        # Step positions
        for k in range(N_DRONES):
            if arrived[k]:
                continue
            step = velocities[k] * DT
            target = extra_waypoints[k] if extra_waypoints[k] is not None else goals[k]
            if np.linalg.norm(target - positions[k]) <= np.linalg.norm(step):
                if extra_waypoints[k] is not None:
                    positions[k] = extra_waypoints[k].copy()
                    extra_waypoints[k] = None
                    speeds[k] = V_DRONE   # resume nominal speed
                else:
                    positions[k] = goals[k].copy()
                    arrived[k] = True
                    arrival_times[k] = t
            else:
                positions[k] += step

        history.append(positions.copy())
        t += DT

    added_dist   = np.nansum(arrival_times - nominal_times) * V_DRONE  # approx
    max_delay    = np.nanmax(arrival_times - nominal_times)
    return history, added_dist, max_delay, total_conflict_events
```

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Fleet size $N$ | 8 drones |
| Cruise speed $v$ | 12.0 m/s |
| Minimum speed (strategy 1) $v_{min}$ | 4.0 m/s |
| Separation threshold $d_{sep}$ | 5.0 m |
| Clearance margin $\delta_{margin}$ | 2.0 m |
| Look-ahead window $T_{horizon}$ | 30 s |
| Nominal cruise altitude $z_{nom}$ | 20.0 m |
| Altitude layer spacing $\Delta z$ | 2.0 m |
| Altitude layer range | 18 – 32 m |
| Airspace arena | $500 \times 500$ m |
| All drones depart from | depot at (250, 250) m |
| Goals | 8 random points in $[20, 480]^2$ m |
| Simulation timestep $\Delta t$ | 0.1 s |

---

## Expected Output

- **Top-down 2D trajectory map**: planned straight-line routes overlaid with actual flown paths for
  each strategy; conflict zones circled in red
- **3D trajectory plot**: all drone paths in 3D for strategy 3 (altitude layers), showing vertical
  separation
- **Separation time series**: minimum pairwise separation $\min_{i<j}\|\mathbf{p}_i - \mathbf{p}_j\|$
  vs time for each strategy; $d_{sep}$ plotted as dashed red line
- **Strategy comparison bar chart**: total added flight distance and maximum arrival delay for all
  three strategies
- **Conflict event count** printed per strategy (strategy 3 should be zero)
- **Altitude time series** (strategy 3): $z_i(t)$ for all drones, showing constant layer assignment

---

## Extensions

1. **Mixed-speed fleet** — drones with different cruise speeds; update CPA computation and test
   whether altitude layers still guarantee separation when overtaking is possible
2. **Dynamic conflict re-check** — after a speed or waypoint resolution, re-run conflict detection
   to catch secondary (cascading) conflicts introduced by the manoeuvre
3. **Velocity Obstacle (VO) method** — each drone continuously steers away from velocity cones that
   would produce conflicts, replacing the two strategy-specific rules with a unified reactive planner
4. **3D conflict zones** — no-fly cylinders (buildings, restricted airspace) added to the arena;
   extend the waypoint insertion strategy to account for static obstacles using RRT*
5. **Scalability study** — sweep $N \in \{4, 8, 16, 32\}$ drones and plot conflict count vs fleet
   size for each strategy; identify the crossover point where altitude layers become impractical

---

## Related Scenarios

- Prerequisites: [S021](S021_point_delivery.md), [S022](S022_obstacle_avoidance_delivery.md), [S029](S029_urban_logistics_scheduling.md)
- Follow-ups: [S032](S032_charging_queue.md), [S033](S033_online_order_insertion.md)
- Algorithmic cross-reference: [S028](S028_cargo_escort_formation.md) (formation separation), [S022](S022_obstacle_avoidance_delivery.md) (obstacle avoidance)
