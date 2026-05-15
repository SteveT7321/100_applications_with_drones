# S096 3D Upgrade — Drone Relay Race

**Domain**: Special Ops & Entertainment | **Difficulty**: ⭐⭐⭐⭐ | **Status**: `[ ]` Not Started
**Based on**: [S096 original](../S096_relay_race.md)

---

## What Changes in 3D

The original S096 runs the entire race on a flat horizontal plane: all three drones maintain
`z_race = 10.0 m` throughout, and the relay course is a straight line along the x-axis with
`y = 0`. The altitude hold controller is a trivial proportional term
(`v_desired[2] = -2.0 * (p[2] - Z_RACE)`) — effectively a 2D simulation with a z-stabiliser
bolted on. The baton handoff rendezvous and minimum-snap intercept trajectory are also computed
entirely in the horizontal plane.

This 3D upgrade replaces the flat straight course with a fully 3D racecourse featuring:

1. **Altitude checkpoints**: each relay leg passes through one or more mandatory altitude waypoints,
   forcing drones to climb and descend between them. The racecourse is no longer co-planar.
2. **3D aerial rendezvous**: the incoming drone arrives at the relay point from a different altitude
   than the waiting outgoing drone. The KF prediction, intercept time search, and minimum-snap
   trajectory planner all operate in true 3D with non-trivial z components.
3. **Per-leg 3D speed optimisation**: the optimal cruise speed for each leg is computed jointly
   over the horizontal and vertical components of the trajectory, accounting for the extra distance
   and energy cost of altitude changes.
4. **Altitude-bounded flight**: all drones operate within `z ∈ [2.0, 20.0]` m, with the relay
   checkpoints and handoff zones placed at different altitudes to create genuine vertical variation.

---

## Problem Definition

**Setup**: A 300 m relay race course is divided into three legs of 100 m each. The course is no
longer flat: each leg has one intermediate altitude waypoint that the drone must pass through en
route to the relay point. The two relay (handoff) zones are located at different altitudes — RP1 at
$(100, 0, 8)$ m and RP2 at $(200, 0, 14)$ m — requiring the outgoing drone to position itself at
the correct 3D rendezvous altitude before the incoming drone arrives.

The handoff protocol is identical to S096 (KF prediction + minimum-snap intercept + velocity
matching), but the intercept trajectory must now plan a full 3D path from the outgoing drone's
waiting altitude to the predicted 3D intercept position.

**Roles**:

- **Drone A (Leg 1)**: starts at $(0, 0, 10)$ m; passes through waypoint $(50, 0, 6)$ m; is the
  incoming drone at RP1 near $(100, 0, 8)$ m. Cruise speed $v_A$.
- **Drone B (Leg 2)**: pre-positioned at $(100, 0, 12)$ m (above RP1 altitude); is the outgoing
  drone at RP1 and the incoming drone at RP2 near $(200, 0, 14)$ m; passes through waypoint
  $(150, 0, 16)$ m on Leg 2. Cruise speed $v_B$.
- **Drone C (Leg 3)**: pre-positioned at $(200, 0, 10)$ m (below RP2 altitude); is the outgoing
  drone at RP2; passes through waypoint $(250, 0, 12)$ m; finishes at $(300, 0, 10)$ m. Cruise
  speed $v_C$.
- **Baton**: logical token transferred when the 3D rendezvous condition is satisfied
  (position error $\leq d_{handoff} = 0.3$ m, velocity error $\leq v_{tol} = 0.5$ m/s) for a
  continuous window $T_{handoff} = 1$ s.

**Objective**: complete the 3D race with both handoffs successful while minimising total race time
$T_{total}$. Success criteria are identical to S096 but applied to full 3D position and velocity
vectors.

**Comparison configurations**:

1. **Flat-course baseline** — race run at constant $z = 10$ m (S096 original); serves as the
   control for measuring the time penalty and handoff difficulty introduced by altitude variation.
2. **3D fixed-speed** — altitude checkpoints active, drone speeds fixed at $(v_A, v_B, v_C) =
   (12, 14, 16)$ m/s; handoff planner uses full 3D intercept but no speed optimisation.
3. **3D optimised** — altitude checkpoints active, per-leg 3D speed optimisation applied;
   handoff planner uses full 3D intercept with continuous KF replanning.

---

## Mathematical Model

### 3D Racecourse Geometry

The course is defined by a sequence of 3D waypoints for each leg:

$$\text{Leg 1: } W_A = \{(0,0,10),\; (50,0,6),\; (100,0,8)\}$$

$$\text{Leg 2: } W_B = \{(100,0,8),\; (150,0,16),\; (200,0,14)\}$$

$$\text{Leg 3: } W_C = \{(200,0,14),\; (250,0,12),\; (300,0,10)\}$$

The geometric length of each leg accounting for altitude changes:

$$L_{leg,i} = \sum_{k} \left\|\mathbf{w}_{k+1}^{(i)} - \mathbf{w}_k^{(i)}\right\|$$

For Leg 1: $L_{leg,1} = \sqrt{50^2 + 4^2} + \sqrt{50^2 + 2^2} \approx 100.3$ m.

### 3D Rendezvous Condition

The handoff condition is extended to the full 3D position and velocity vectors:

$$\left\|\mathbf{p}_{in}(t_0) - \mathbf{p}_{out}(t_0)\right\| \leq d_{handoff} = 0.3 \text{ m}$$

$$\left\|\mathbf{v}_{in}(t_0) - \mathbf{v}_{out}(t_0)\right\| \leq v_{tol} = 0.5 \text{ m/s}$$

where $\mathbf{p}, \mathbf{v} \in \mathbb{R}^3$ now include the z component.

### 3D Kalman Filter (unchanged structure, full 3D)

The KF state vector $\mathbf{x}_{KF} = [p_x, p_y, p_z, v_x, v_y, v_z]^\top \in \mathbb{R}^6$
is identical to S096. The altitude channel $p_z$ is now meaningful — the KF must accurately
predict the incoming drone's altitude trajectory, not just its horizontal progress, to plan the
3D intercept.

The critical new requirement is that the KF altitude prediction must converge quickly enough
because the incoming drone is descending or climbing toward the relay altitude, making
$\hat{v}_z$ estimation accuracy directly affect intercept quality:

$$\hat{p}_z(t + \tau) = \hat{p}_z(t) + \tau \hat{v}_z(t)$$

A mismatch in $\hat{v}_z$ by $\delta v_z$ produces an intercept altitude error of
$\tau \cdot \delta v_z$ at time $\tau$ — for $\tau = 3$ s and $\delta v_z = 0.5$ m/s this
gives a 1.5 m vertical miss, which would fail the $d_{handoff} = 0.3$ m criterion.

### 3D Altitude Checkpoint Tracking

Each cruising drone follows a waypoint sequence using a 3D proportional-derivative velocity
controller. For current waypoint $\mathbf{w}_{target}$:

$$\mathbf{v}_{desired} = v_{cruise} \cdot \frac{\mathbf{w}_{target} - \mathbf{p}}{\|\mathbf{w}_{target} - \mathbf{p}\|}$$

$$\mathbf{u}_{cmd} = K_p \left(\mathbf{v}_{desired} - \mathbf{v}\right), \qquad K_p = 5.0$$

When $\|\mathbf{p} - \mathbf{w}_{target}\| < r_{switch} = 3.0$ m, the drone advances to the
next waypoint in its sequence. This replaces the flat `cruise_command()` that only tracked
the x-axis.

### 3D Minimum-Snap Intercept Trajectory

The minimum-snap trajectory planner from S096 is applied identically in all three axes. The
key difference is that the target rendezvous state $(\mathbf{p}^*, \mathbf{v}^*)$ now has a
non-trivial $z^*$ component predicted by the KF:

$$\mathbf{p}^* = \hat{\mathbf{p}}_{in}(t + \tau^*) = \begin{pmatrix} \hat{p}_x \\ \hat{p}_y \\ \hat{p}_z \end{pmatrix}$$

$$\mathbf{v}^* = \hat{\mathbf{v}}_{in}(t + \tau^*) = \begin{pmatrix} \hat{v}_x \\ \hat{v}_y \\ \hat{v}_z \end{pmatrix}$$

The z-axis polynomial must satisfy both $p_z(T_f) = z^*$ and $\dot{p}_z(T_f) = v_z^*$, driving
the outgoing drone to arrive at the correct altitude with matched vertical velocity. The
acceleration budget $a_{max} = 5$ m/s² is shared across all three axes:

$$\sqrt{a_x(t)^2 + a_y(t)^2 + a_z(t)^2} \leq a_{max}$$

### Per-Leg 3D Speed Optimisation

The 3D leg time for drone $i$ travelling waypoint sequence $W_i$ at speed $v_i$ is:

$$t_{leg,i}(v_i) = \frac{L_{leg,i}}{v_i}$$

The intercept feasibility constraint requires that the outgoing drone can reach the predicted
rendezvous point $\mathbf{p}^*$ with matched velocity before the incoming drone passes it.
Define the intercept margin for relay $j$:

$$\Delta_{j}(v_{in}, v_{out}) = \tau^*(v_{in}, v_{out}) - \tau_{min}$$

where $\tau^*$ is the optimal intercept time and $\tau_{min} = 0.5$ s is the minimum feasible
lookahead. The speed optimisation problem is:

$$\min_{v_A, v_B, v_C} T_{total} = \sum_{i=1}^{3} \frac{L_{leg,i}}{v_i} + 2 T_{handoff}$$

$$\text{subject to: } v_{min} \leq v_i \leq v_{max}, \quad \Delta_j \geq 0 \;\forall j$$

In 3D the feasibility constraint is tighter than in 2D because the outgoing drone must also
traverse a vertical distance $\Delta z_j = |z_{wait,j} - z_{RP,j}|$ as part of its intercept
trajectory, consuming part of the acceleration budget that would otherwise serve horizontal
convergence.

### Altitude Bounds

All drone altitudes are bounded at all times:

$$z_{min} = 2.0 \text{ m} \leq z(t) \leq z_{max} = 20.0 \text{ m}$$

A soft-constraint correction term is added to $u_z$ whenever a drone approaches a bound:

$$u_{z,bound} = \begin{cases}
  K_{bound}(z_{min} - z) & \text{if } z < z_{min} + 0.5 \\
  K_{bound}(z_{max} - z) & \text{if } z > z_{max} - 0.5 \\
  0 & \text{otherwise}
\end{cases}, \qquad K_{bound} = 10.0$$

---

## Key 3D Additions

- **3D racecourse waypoints**: altitude varies between 6 m and 16 m across the three legs,
  creating genuine climb-and-dive segments on each leg.
- **Mismatched waiting altitude**: outgoing drones wait at altitudes 2–4 m above or below the
  relay point, requiring the minimum-snap intercept to descend or climb to match the incoming
  drone's trajectory.
- **Full 3D KF altitude tracking**: $v_z$ estimation becomes load-bearing — the incoming drone
  is changing altitude near the relay point, so a flat-z assumption in the predictor would cause
  systematic intercept misses.
- **3D waypoint-following controller**: replaces the 1D x-axis cruise command with a 3D velocity
  director that switches through altitude checkpoints.
- **Per-leg geometric length $L_{leg,i}$**: leg times computed from 3D path length (slightly
  longer than 100 m) rather than horizontal distance alone.
- **Altitude bounds enforcement**: soft-constraint correction added to $u_z$ to keep all drones
  within $[2, 20]$ m throughout.

---

## Key Parameters

| Parameter | Symbol | Value |
|-----------|--------|-------|
| Race course horizontal length | $L_x$ | 300 m |
| Relay Point 1 position | $\mathbf{p}_{RP1}$ | $(100, 0, 8)$ m |
| Relay Point 2 position | $\mathbf{p}_{RP2}$ | $(200, 0, 14)$ m |
| Finish position | $\mathbf{p}_{finish}$ | $(300, 0, 10)$ m |
| Leg 1 altitude waypoint | $\mathbf{w}_A$ | $(50, 0, 6)$ m |
| Leg 2 altitude waypoint | $\mathbf{w}_B$ | $(150, 0, 16)$ m |
| Leg 3 altitude waypoint | $\mathbf{w}_C$ | $(250, 0, 12)$ m |
| Drone A wait altitude | — | 10 m (start) |
| Drone B wait altitude (above RP1) | — | 12 m |
| Drone C wait altitude (below RP2) | — | 10 m |
| Altitude range | $[z_{min}, z_{max}]$ | $[2, 20]$ m |
| Waypoint switch radius | $r_{switch}$ | 3.0 m |
| Altitude bound stiffness | $K_{bound}$ | 10.0 |
| Handoff position tolerance | $d_{handoff}$ | 0.3 m |
| Handoff velocity tolerance | $v_{tol}$ | 0.5 m/s |
| Required handoff window | $T_{handoff}$ | 1 s |
| Drone A cruise speed (fixed) | $v_A$ | 12 m/s |
| Drone B cruise speed (fixed) | $v_B$ | 14 m/s |
| Drone C cruise speed (fixed) | $v_C$ | 16 m/s |
| Maximum acceleration | $a_{max}$ | 5 m/s² |
| Dynamics disturbance std dev | $\sigma_d$ | 0.1 m/s² |
| KF process noise | $\sigma_a$ | 0.5 m/s² |
| KF measurement noise | $\sigma_{obs}$ | 0.05 m |
| Simulation timestep | $\Delta t$ | 0.02 s (50 Hz) |
| Intercept lookahead range | $[\tau_{min}, \tau_{max}]$ | $[0.5, 10]$ s |
| Minimum-snap polynomial degree | — | 7 |

---

## Expected Output

- **3D trajectory plot** (`Axes3D`): full 3D view of all three drone paths; drone A in red,
  B in blue, C in green; altitude checkpoints shown as orange diamonds; RP1 and RP2 handoff
  zones shown as orange spheres (radius $d_{handoff}$); start marked with a black triangle,
  finish with a gold star; solid lines for the KF+3D strategy, dashed for the flat-course
  baseline; legend identifies each drone and configuration.
- **Altitude time series**: three stacked subplots showing $z(t)$ for drones A, B, C over the
  full race; altitude checkpoints marked as horizontal dashed lines; handoff instants marked
  with vertical lines; demonstrates genuine altitude variation as opposed to the flat baseline.
- **RP1 and RP2 3D handoff detail** (2 panels each): position separation
  $\|\mathbf{p}_{in} - \mathbf{p}_{out}\|$ and velocity mismatch
  $\|\mathbf{v}_{in} - \mathbf{v}_{out}\|$ vs time; green shaded success band at
  $d_{handoff}$ and $v_{tol}$; KF+3D strategy expected to sustain the band cleanly;
  flat-baseline strategy used as reference.
- **Speed optimisation comparison** (bar chart): total race time and per-leg leg time for
  (i) flat baseline, (ii) 3D fixed-speed, (iii) 3D optimised; illustrates the time penalty
  from altitude checkpoints and the partial recovery from speed optimisation.
- **3D relay race animation** (GIF): three coloured drone dots with 3D trails; altitude shown
  on a side-elevation inset (xz-plane view) alongside the top-down (xy-plane) view; baton
  diamond icon tracks the current baton-holder; RP zone spheres pulse at handoff instant;
  live time, altitude, and baton-holder label updated each frame.

**Expected metric targets** (KF+3D fixed-speed, seed 0):

| Metric | Target |
|--------|--------|
| RP1 handoff success | YES |
| RP2 handoff success | YES |
| RP1 position error at handoff | $\leq 0.3$ m |
| RP2 position error at handoff | $\leq 0.3$ m |
| RP1 velocity mismatch at handoff | $\leq 0.5$ m/s |
| RP2 velocity mismatch at handoff | $\leq 0.5$ m/s |
| Max altitude excursion above RP altitude | $\geq 2$ m (demonstrates genuine 3D variation) |
| Total race time vs flat baseline | $\leq 10\%$ longer (altitude overhead) |

---

## Extensions

1. **Optimal altitude profile**: formulate the altitude waypoints as optimisation variables and
   search for the $z$ profile that minimises $T_{total}$ while keeping all handoff margins
   non-negative; compare against the hand-crafted waypoints used here.
2. **Crosswind with altitude shear**: model a wind field where horizontal wind speed varies with
   altitude ($w_x(z) = w_0 (z / z_{ref})^{0.3}$, a power-law wind profile); study how altitude
   choice affects the effective headwind or tailwind on each leg and whether the KF can compensate
   without wind-state augmentation.
3. **Four-leg 3D course with vertical obstacles**: add cylindrical no-fly columns (e.g., buildings)
   and require RRT*-planned leg trajectories; measure the compounding timing penalty across
   three relay handoffs when any leg must detour.
4. **Energy-optimal 3D leg**: replace the minimum-time objective with minimum-energy for each leg,
   modelling battery discharge as $P = k_1 v^2 + k_2 (dz/dt)^2$; compare energy consumption
   between the flat baseline and the altitude-varied 3D course at matched total race time.
5. **Imperfect altitude sensor**: add correlated noise to the z-channel measurement in the KF
   ($\sigma_{obs,z} = 0.2$ m vs $\sigma_{obs,xy} = 0.05$ m) to simulate barometric altitude
   uncertainty; quantify the increase in intercept altitude error at the relay points and
   propose an augmented measurement model (e.g., fusing barometer + optical flow).

---

## Related Scenarios

- Original 2D version: [S096](../S096_relay_race.md)
- Truly 3D references: [S001](../../01_pursuit_evasion/S001_basic_intercept.md) (3D intercept
  geometry), [S023](../../02_logistics_delivery/S023_moving_landing_pad.md) (3D velocity-matched
  rendezvous with a moving target)
- Domain 3D peers: [S081 3D Selfie Follow](S081_3d_selfie_follow.md),
  [S083 3D Light Show](S083_3d_light_show.md), [S085 3D Light Matrix](S085_3d_light_matrix.md)
- Follow-up: [S099 Obstacle Relay](../S099_obstacle_relay.md) (relay race with dynamic obstacles
  and collision avoidance, natural next step after mastering the 3D handoff geometry)
