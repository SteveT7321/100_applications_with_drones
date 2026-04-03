# S039 Offshore Platform Exchange

**Domain**: Logistics & Delivery | **Difficulty**: ⭐⭐⭐⭐ | **Status**: `[ ]` Not started

---

## Problem Definition

**Setup**: A fleet of $N = 4$ drones must redistribute spare parts and medical supplies among $M = 5$
offshore platforms (oil rigs or fixed wind-turbine service pads) scattered across a $6 \times 6$ km sea
area. Platforms are fixed structures with small deck landing zones ($2 \times 2$ m). A support vessel
(supply ship) sails a predetermined route through the area at constant speed $V_s$, acting as a
mobile recharging depot. Each drone has limited range $R_{max}$ that may be insufficient to fly
directly between distant platform pairs; the vessel provides intermediate battery swaps.

Wind is present as a steady offset with a random gust component (Dryden model), making actual
groundspeed and heading deviate from air-speed commands. Drones must compensate for wind drift to
achieve precise deck landings.

**Roles**:
- **Drones** ($N = 4$ units): carry cargo from origin platforms to destination platforms; may divert
  to the vessel for a battery swap mid-route when predicted remaining range falls below threshold.
- **Support vessel**: mobile recharging depot sailing at $V_s = 4$ m/s; drones must intercept the
  vessel's projected position and land on its moving deck (see S023).
- **Platforms** ($M = 5$ fixed offshore structures): origin/destination nodes with cargo exchange
  requests; each has a single landing pad and a queue of at most one waiting drone.

**Objective**:

1. **Feasibility**: every cargo request $r_i = (o_i, d_i, w_i)$ (origin, destination, payload weight)
   is satisfied within deadline $T_{dl,i}$.
2. **Optimality**: minimise total fleet energy consumption subject to deadline satisfaction:

$$\min \sum_{k=1}^{N} E_k^{total} \quad \text{s.t.} \quad t_{deliver,i} \leq T_{dl,i} \; \forall\, i$$

3. **Safety**: no two drones occupy the same platform or vessel deck simultaneously.

**Comparison strategies**:
1. **Greedy direct**: each drone flies the shortest feasible direct route; diverts to vessel only on
   battery emergency.
2. **Vessel-relay scheduled**: vessel waypoints are pre-computed and drone routes planned to use
   vessel swaps proactively, reducing peak battery draw.
3. **Rolling-horizon replanning**: every $\Delta T_{plan} = 30$ s, re-solve the assignment and
   routing given current vessel position and drone states.

---

## Mathematical Model

### Platform Graph and Inter-Platform Distances

Let $\mathbf{p}_m \in \mathbb{R}^2$ denote the 2D position of platform $m$, $m = 1, \ldots, M$.
The vessel position at time $t$ follows a pre-planned piecewise-linear route:

$$\mathbf{p}_v(t) = \mathbf{p}_{v,j} + V_s \cdot (t - t_j) \cdot \hat{\mathbf{r}}_{j,j+1}$$

for the vessel on leg $j$ (from waypoint $j$ to $j+1$), where $\hat{\mathbf{r}}_{j,j+1}$ is the
unit direction vector.

The effective cost of flying from node $i$ to node $j$ (platform or vessel intercept point) for
drone $k$ with wind $\mathbf{w}$:

$$c_{ij}^k = \frac{d_{ij}}{V_{g,ij}^k} \quad \text{(time cost)} \qquad \text{or} \qquad
c_{ij}^k = \frac{P_k \cdot d_{ij}}{V_{g,ij}^k} \quad \text{(energy cost)}$$

where $V_{g,ij}^k$ is the groundspeed on leg $(i,j)$ accounting for wind.

### Wind Model and Groundspeed

Steady wind vector $\mathbf{w}_{mean} \in \mathbb{R}^2$. Drone $k$ commands airspeed $V_a^k$ in
direction $\hat{\mathbf{u}}_k$. Groundspeed vector:

$$\mathbf{v}_g^k = V_a^k \hat{\mathbf{u}}_k + \mathbf{w}_{mean} + \boldsymbol{\xi}(t)$$

where $\boldsymbol{\xi}(t)$ is a zero-mean Dryden gust (low-pass filtered white noise):

$$\dot{\boldsymbol{\xi}} = -\frac{V_a^k}{L_g}\boldsymbol{\xi} + \sigma_g \sqrt{\frac{2 V_a^k}{L_g}}\, \boldsymbol{\eta}(t)$$

with gust length scale $L_g = 200$ m and gust intensity $\sigma_g$.

To fly a desired ground-track bearing $\psi_{des}$ against wind, the required crab angle $\delta$ satisfies:

$$\sin\delta = \frac{\|\mathbf{w}_{mean}\| \sin(\psi_{des} - \psi_w)}{V_a^k}$$

where $\psi_w$ is the wind direction. The effective groundspeed along the desired track:

$$V_g = V_a^k \cos\delta + \|\mathbf{w}_{mean}\| \cos(\psi_{des} - \psi_w)$$

### Battery Model and Range Prediction

Battery state of charge $E_k(t)$ (Joules) depletes at rate:

$$\dot{E}_k = -P_{cruise} - k_{wind} \|\mathbf{w}_{mean}\|^2$$

where $P_{cruise}$ is nominal cruise power and $k_{wind}$ is the aerodynamic penalty coefficient for
crosswind operation.

Predicted range at time $t$ in current wind conditions:

$$d_{rem}^k(t) = \frac{E_k(t) \cdot V_g}{P_{cruise} + k_{wind}\|\mathbf{w}_{mean}\|^2}$$

Battery-swap trigger: drone $k$ diverts to the vessel when:

$$d_{rem}^k(t) < d_k^{dest}(t) + d_{safety}$$

where $d_k^{dest}(t)$ is the remaining distance to the drone's current destination and
$d_{safety} = 0.15 \cdot R_{max}$ is the reserve margin.

### Vessel Intercept Point

Given drone position $\mathbf{p}_k$, airspeed $V_a^k$, wind $\mathbf{w}$, and vessel trajectory
$\mathbf{p}_v(t)$, the intercept time $t^*$ is the solution of:

$$\|\mathbf{p}_v(t^*) - \mathbf{p}_k\|^2 = \left(V_g \cdot (t^* - t)\right)^2$$

This is solved iteratively (fixed-point iteration on $t^*$) since $V_g$ depends on the bearing to
the intercept point.

**Fixed-point iteration** (converges in $\leq 5$ steps):

$$t^*_{n+1} = t + \frac{\|\mathbf{p}_v(t^*_n) - \mathbf{p}_k\|}{V_g(\psi_{to\_vessel}(t^*_n))}$$

### Moving Deck Landing Control

During the final approach to the vessel (radius $\leq d_{dock}$), the drone tracks the vessel deck
using a PD controller on the relative state. Define:

$$\Delta\mathbf{r}_k = \mathbf{p}_k - \mathbf{p}_v(t), \qquad
\Delta\mathbf{v}_k = \dot{\mathbf{p}}_k - \mathbf{v}_v(t)$$

Approach command:

$$\mathbf{u}_k = -K_p \Delta\mathbf{r}_k - K_d \Delta\mathbf{v}_k + \dot{\mathbf{v}}_v(t)$$

Landing declared successful when:

$$\|\Delta\mathbf{r}_k\| < \epsilon_p \quad \text{and} \quad \|\Delta\mathbf{v}_k\| < \epsilon_v$$

### Conflict-Free Scheduling: Platform Queue Constraint

No two drones may land on the same platform simultaneously. For platform $m$, let
$\mathcal{A}_m(t) = \{k : \mathbf{p}_k \approx \mathbf{p}_m, \text{drone } k \text{ on deck}\}$.
The constraint is:

$$|\mathcal{A}_m(t)| \leq 1 \quad \forall\, m, \; \forall\, t$$

Conflict avoidance is enforced by a priority queue at each platform: a drone arriving while
$|\mathcal{A}_m| = 1$ enters a circular holding orbit of radius $R_{hold} = 30$ m at altitude
$z_{hold} = 25$ m above the platform until the pad is clear.

Holding orbit cost (battery drain while waiting for pad clearance, dwell time $\tau_{hold}$):

$$\Delta E_{hold} = (P_{cruise} + k_{wind}\|\mathbf{w}_{mean}\|^2) \cdot \tau_{hold}$$

### Multi-Drone Assignment: Rolling-Horizon MIP

At each replanning epoch $t_r$, the assignment problem is formulated as a min-cost flow on graph
$G = (\mathcal{V}, \mathcal{E})$ where nodes are platforms, vessel intercept points, and drone
current positions. Each arc $(i,j)$ carries cost $c_{ij}^k$ and a capacity of 1 (single drone per
arc). The LP relaxation is solved with scipy's `linprog` (or PuLP) and rounded greedily.

Formal assignment model (binary program, per replanning epoch):

$$\min_{x_{ijk}} \sum_{k=1}^N \sum_{(i,j) \in \mathcal{E}} c_{ij}^k \, x_{ijk}$$

$$\text{s.t.} \quad \sum_{k} \sum_j x_{ijk} \leq 1 \quad \forall\, i \text{ (platform outflow)}$$

$$\sum_j x_{o_r j k} \geq 1 \quad \forall\, r \text{ (each request served)}$$

$$x_{ijk} \in \{0,1\}$$

---

## Implementation

```python
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import linprog

# Key constants
N_DRONES    = 4
N_PLATFORMS = 5
V_AIRSPEED  = 12.0     # m/s drone cruise airspeed
V_VESSEL    = 4.0      # m/s support vessel speed
R_MAX       = 5000.0   # m max range per battery
D_SAFETY    = 0.15     # fractional battery reserve
P_CRUISE    = 80.0     # W nominal cruise power
K_WIND      = 0.8      # W·s²/m²  wind penalty coefficient
W_MEAN      = np.array([4.0, 1.5])  # m/s mean wind vector
SIGMA_GUST  = 1.2      # m/s gust intensity
L_GUST      = 200.0    # m  gust length scale
D_DOCK      = 15.0     # m  moving-deck engagement radius
EPS_P       = 1.0      # m  landing position tolerance
EPS_V       = 0.5      # m/s landing velocity tolerance
KP, KD      = 1.8, 1.2 # deck-tracking PD gains
R_HOLD      = 30.0     # m  holding orbit radius
DT          = 0.1      # s  simulation timestep
T_MAX       = 600.0    # s  mission horizon
DT_PLAN     = 30.0     # s  replanning interval

# Platform positions (2D, metres; offshore area 0–6000 m)
PLATFORMS = np.array([
    [500.,  500.],
    [2000., 4500.],
    [4000., 1000.],
    [5500., 3500.],
    [3000., 2800.],
])

# Vessel route waypoints (piecewise-linear, looping)
VESSEL_ROUTE = np.array([
    [1000., 1000.],
    [3000., 500.],
    [5000., 2000.],
    [4500., 4500.],
    [1500., 3500.],
    [1000., 1000.],
])

# Cargo requests: (origin_idx, dest_idx, payload_kg, deadline_s)
REQUESTS = [
    (0, 3, 1.5, 400.0),
    (2, 1, 0.8, 350.0),
    (4, 0, 2.0, 500.0),
    (1, 4, 1.2, 450.0),
]

def vessel_position(t, route, speed):
    """Return vessel 2D position at time t along piecewise-linear route."""
    leg_lengths = np.linalg.norm(np.diff(route, axis=0), axis=1)
    cumulative  = np.concatenate([[0.], np.cumsum(leg_lengths)])
    dist_covered = (speed * t) % cumulative[-1]
    seg = np.searchsorted(cumulative, dist_covered, side='right') - 1
    seg = min(seg, len(route) - 2)
    frac = (dist_covered - cumulative[seg]) / leg_lengths[seg]
    return route[seg] + frac * (route[seg + 1] - route[seg])

def intercept_time(p_drone, v_ground, p_vessel_func, t_now, max_iter=10):
    """Fixed-point iteration to find vessel intercept time."""
    t_star = t_now
    for _ in range(max_iter):
        pv = p_vessel_func(t_star)
        dist = np.linalg.norm(pv - p_drone)
        t_star = t_now + dist / v_ground
    return t_star

def dryden_gust_step(xi, v_air, dt, L_g, sigma_g, rng):
    """First-order Dryden gust update (2D)."""
    tau = L_g / v_air
    a   = np.exp(-dt / tau)
    b   = sigma_g * np.sqrt(2.0 / tau) * np.sqrt(1.0 - a**2)
    return a * xi + b * rng.standard_normal(2)

def groundspeed_vector(heading_deg, v_air, wind):
    """Compute groundspeed vector given air heading, airspeed, and wind."""
    psi = np.radians(heading_deg)
    air_vec = v_air * np.array([np.cos(psi), np.sin(psi)])
    return air_vec + wind

def crab_angle_and_groundspeed(target_dir, v_air, wind):
    """
    Compute crab angle and groundspeed magnitude to fly toward target_dir
    under wind. target_dir is a unit 2D vector.
    """
    psi_des = np.arctan2(target_dir[1], target_dir[0])
    psi_w   = np.arctan2(wind[1], wind[0])
    w_mag   = np.linalg.norm(wind)
    sin_d   = np.clip(w_mag * np.sin(psi_des - psi_w) / v_air, -1.0, 1.0)
    delta   = np.arcsin(sin_d)
    Vg      = v_air * np.cos(delta) + w_mag * np.cos(psi_des - psi_w)
    return delta, max(Vg, 1.0)

class Drone:
    def __init__(self, idx, pos, battery):
        self.idx    = idx
        self.pos    = np.array(pos, dtype=float)
        self.vel    = np.zeros(2)
        self.energy = float(battery)
        self.gust   = np.zeros(2)
        self.state  = "idle"    # idle | en_route | approach_vessel | holding
        self.target = None      # (pos, type) tuple
        self.cargo  = None      # request index being carried

    def range_remaining(self, wind):
        P_total = P_CRUISE + K_WIND * np.dot(wind, wind)
        _, Vg = crab_angle_and_groundspeed(np.array([1., 0.]), V_AIRSPEED, wind)
        return (self.energy / P_TOTAL) * Vg if P_TOTAL > 0 else 0.0

    def step(self, dt, wind_total, rng):
        if self.target is None:
            return
        direction = self.target - self.pos
        dist = np.linalg.norm(direction)
        if dist < 1.0:
            self.pos = self.target.copy()
            return
        unit = direction / dist
        _, Vg = crab_angle_and_groundspeed(unit, V_AIRSPEED, wind_total)
        self.pos += unit * Vg * dt
        P_total = P_CRUISE + K_WIND * np.dot(wind_total, wind_total)
        self.energy = max(0.0, self.energy - P_total * dt)
        # Gust update
        self.gust = dryden_gust_step(self.gust, V_AIRSPEED, dt, L_GUST, SIGMA_GUST, rng)
```

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Platforms $M$ | 5 offshore structures in 6 km x 6 km area |
| Drones $N$ | 4 |
| Drone cruise airspeed $V_a$ | 12 m/s |
| Maximum range $R_{max}$ | 5000 m per battery |
| Battery reserve margin $d_{safety}$ | 15% |
| Cruise power $P_{cruise}$ | 80 W |
| Wind penalty coefficient $k_{wind}$ | 0.8 W·s²/m² |
| Mean wind vector $\mathbf{w}_{mean}$ | (4.0, 1.5) m/s |
| Gust intensity $\sigma_g$ | 1.2 m/s |
| Gust length scale $L_g$ | 200 m |
| Support vessel speed $V_s$ | 4 m/s |
| Moving-deck engagement radius $d_{dock}$ | 15 m |
| Landing position tolerance $\epsilon_p$ | 1.0 m |
| Landing velocity tolerance $\epsilon_v$ | 0.5 m/s |
| Deck-tracking PD gains $(K_p, K_d)$ | (1.8, 1.2) |
| Holding orbit radius $R_{hold}$ | 30 m |
| Replanning interval $\Delta T_{plan}$ | 30 s |
| Mission horizon $T_{max}$ | 600 s |
| Simulation timestep $dt$ | 0.1 s |
| Cargo requests | 4 (origin, destination, weight, deadline) |

---

## Expected Output

- **2D sea area map**: platforms (blue squares), vessel route (dashed grey), vessel position at key
  times (ship icon), drone trajectories colour-coded by drone index, battery-swap events marked with
  a lightning bolt symbol, holding orbit circles where pad conflicts occurred.
- **Battery SoC vs time** for all 4 drones: vertical lines for battery-swap events, horizontal
  dashed line at 15% reserve; shade below reserve red.
- **Vessel intercept geometry inset**: zoom-in showing vessel track, intercept point, and drone
  spiral approach during moving-deck landing phase.
- **Relative distance and velocity to vessel deck** during each landing approach: convergence of
  $\|\Delta\mathbf{r}\|$ and $\|\Delta\mathbf{v}\|$ to tolerance bands.
- **Platform queue timeline**: Gantt-style chart showing which drone occupies which platform pad at
  each time step; holding events shown as hatched bars.
- **Comparison bar chart**: total fleet energy consumption for (a) greedy direct, (b) vessel-relay
  scheduled, (c) rolling-horizon replanning; alongside a bar for number of deadline violations per
  strategy.

---

## Extensions

1. **Dynamic vessel routing**: optimise the vessel's own waypoint sequence jointly with drone routes
   using alternating optimisation (fix vessel route, optimise drones; fix drones, optimise vessel).
2. **Weather escalation**: increase wind speed in steps during the mission; observe how rolling-horizon
   replanning adapts crab angles, intercept points, and battery-swap trigger thresholds.
3. **Platform pad failure**: one platform landing pad becomes unavailable mid-mission (simulating
   deck equipment obstruction); re-route affected drones to the vessel as a temporary depot.
4. **Heterogeneous cargo urgency**: weight deadline penalties by cargo criticality (medical supply >
   spare part); solve weighted-tardiness variant of the assignment MIP.
5. **Night operations**: add a visibility constraint (drone must be within line of sight of the vessel
   or a platform for communications); re-route drones to maintain connectivity for telemetry relay.

---

## Related Scenarios

- Prerequisites: [S021](S021_point_delivery.md), [S023](S023_moving_landing_pad.md),
  [S024](S024_wind_compensation.md), [S027](S027_aerial_refueling_relay.md)
- Follow-ups: [S040](S040_fleet_load_balancing.md) (fleet load balancing across a larger network)
- Relay chain reference: [S036](S036_last_mile_relay.md) (relay chain range extension)
- Scheduling reference: [S029](S029_urban_logistics_scheduling.md) (VRPTW formulation)
