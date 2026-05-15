# S021 3D Upgrade — Point Delivery

**Domain**: Logistics & Delivery | **Difficulty**: ⭐⭐ | **Status**: `[ ]` Not Started
**Based on**: [S021 original](../S021_point_delivery.md)

---

## What Changes in 3D

The original S021 hardcodes `z_cruise = 2.0 m` as a flat, fixed-altitude cruise layer and treats vertical motion only as a final-phase straight descent (`dz = -v_z_max`). The horizontal and vertical axes are fully decoupled: cruise ignores altitude dynamics entirely, and descent ignores horizontal drift. Two important 3D effects are absent:

1. **Vertical takeoff profile** — the drone is assumed to start exactly at cruise altitude; a realistic departure includes a powered climb from ground level with bounded vertical acceleration.
2. **Altitude-dependent wind drag** — aerodynamic drag grows with air-density-weighted dynamic pressure, which varies with altitude in a real atmosphere and also changes the optimal cruise altitude choice.

This variant adds a four-phase mission FSM (takeoff, cruise, approach descent, land), replaces the decoupled descent with a minimum-time 3D trajectory derived from Pontryagin's Minimum Principle, and couples altitude to an altitude-dependent quadratic drag model.

---

## Problem Definition

**Setup**: A delivery drone departs from a ground depot at $\mathbf{p}_0 = (x_0, y_0, 0)$ m and must deliver to a ground pad at $\mathbf{p}_{goal} = (x_g, y_g, 0)$ m. Flight occurs in full 3D space with altitude bounds $z \in [0, z_{max}]$ m. Wind speed increases linearly with altitude (logarithmic-law profile approximated as linear in the low-altitude regime).

**Roles**:
- **Drone**: point-mass delivery agent with bounded speed $v_{max}$, bounded vertical acceleration $a_{z,max}$, and altitude-dependent aerodynamic drag
- **Depot**: ground-level departure point at $z = 0$ m
- **Landing pad**: ground-level target at $z = 0$ m, capture radius $r_{land}$

**Mission phases**:

| Phase | Description |
|-------|-------------|
| **Takeoff** | Climb from $z = 0$ to $z_{cruise}$ with bounded vertical acceleration |
| **Cruise** | Fly at chosen altitude $z_{cruise}$ toward horizontal waypoint above goal |
| **Descent** | 3D minimum-time trajectory from cruise altitude to ground |
| **Land** | Touch-down check within $r_{land}$ |

**Objective**: Minimise total mission time $T_{mission}$ subject to altitude and speed constraints, jointly optimising the cruise altitude $z_{cruise}^*$.

---

## Mathematical Model

### Altitude-Dependent Wind Drag

Wind speed follows a log-law profile approximated linearly up to $z_{max} = 8$ m:

$$v_{wind}(z) = v_{wind,0} \cdot \frac{z}{z_{ref}}$$

where $v_{wind,0} = 1.5$ m/s is the reference wind speed at $z_{ref} = 2$ m.

Aerodynamic drag force on the drone:

$$\mathbf{F}_{drag}(z) = -k_{drag}(z) \cdot \|\mathbf{v}_{rel}\| \cdot \mathbf{v}_{rel}$$

$$k_{drag}(z) = k_0 \left(1 + \alpha_z \cdot z\right)$$

where $\mathbf{v}_{rel} = \mathbf{v}_{drone} - \mathbf{v}_{wind}(z)\hat{\mathbf{x}}$ is the relative airspeed, $k_0 = 0.08$ kg/m, and $\alpha_z = 0.05$ m$^{-1}$ encodes altitude-dependent air-density and frontal-area effects.

### Phase 1 — Vertical Takeoff with Bounded Acceleration

State: $z(t)$, $\dot{z}(t)$. Control: $u_z \in [-a_{z,max}, +a_{z,max}]$.

Minimum-time bang-bang climb to $z_{cruise}$:

$$\ddot{z}(t) = \begin{cases} +a_{z,max} & t < t_{switch} \\ -a_{z,max} & t \ge t_{switch} \end{cases}$$

Switch time (symmetric bang-bang):

$$t_{switch} = \sqrt{\frac{z_{cruise}}{a_{z,max}}}$$

Takeoff phase duration:

$$T_{takeoff} = 2\,t_{switch} = 2\sqrt{\frac{z_{cruise}}{a_{z,max}}}$$

Final vertical velocity at top of climb is zero (full deceleration to cruise altitude).

### Phase 2 — Cruise (Velocity-Controlled Point-Mass)

Horizontal velocity at maximum speed corrected for headwind component:

$$\mathbf{v}_{cmd,xy}(t) = v_{max} \cdot \hat{\mathbf{d}}_{xy} - v_{wind}(z_{cruise})\,\hat{\mathbf{x}}_{corr}$$

where $\hat{\mathbf{d}}_{xy}$ is the unit vector toward the horizontal goal projection. The wind correction $\hat{\mathbf{x}}_{corr}$ is a feedforward compensation for the dominant wind direction.

Effective ground speed:

$$v_{gnd} = \sqrt{v_{max}^2 - v_{wind}^2(z_{cruise})} \approx v_{max} - \frac{v_{wind}^2(z_{cruise})}{2\,v_{max}}$$

Cruise time:

$$T_{cruise} = \frac{d_{xy}}{v_{gnd}(z_{cruise})}$$

### Phase 3 — Minimum-Time 3D Descent (Pontryagin)

State vector: $\mathbf{x} = [x, y, z, \dot{x}, \dot{y}, \dot{z}]^\top \in \mathbb{R}^6$.

Control input: $\mathbf{u} = [u_x, u_y, u_z]^\top$ with $\|\mathbf{u}\| \le a_{max}$.

Dynamics (point-mass, no drag during descent for tractability):

$$\ddot{\mathbf{p}} = \mathbf{u}$$

Pontryagin Minimum Principle for minimum time: the Hamiltonian is

$$H = 1 + \boldsymbol{\lambda}_p^\top \dot{\mathbf{p}} + \boldsymbol{\lambda}_v^\top \mathbf{u}$$

where $\boldsymbol{\lambda}_v$ is the co-state for velocity. Minimising $H$ over $\mathbf{u}$:

$$\mathbf{u}^* = -a_{max} \cdot \frac{\boldsymbol{\lambda}_v}{\|\boldsymbol{\lambda}_v\|}$$

Co-state equations:

$$\dot{\boldsymbol{\lambda}}_p = \mathbf{0}, \quad \dot{\boldsymbol{\lambda}}_v = -\boldsymbol{\lambda}_p$$

implying $\boldsymbol{\lambda}_p = \text{const}$, $\boldsymbol{\lambda}_v(t) = \boldsymbol{\lambda}_{v,0} - \boldsymbol{\lambda}_p \, t$ (linear).

The optimal thrust direction therefore rotates linearly in time — the 3D generalisation of the classical bang-bang/gravity-turn law. In the vertical channel:

$$u_z^*(t) = -a_{max} \cdot \frac{\lambda_{v,z}(t)}{\|\boldsymbol{\lambda}_v(t)\|}$$

Terminal boundary conditions enforce $\mathbf{p}(T_3) = \mathbf{p}_{goal}$ and $\dot{\mathbf{p}}(T_3) = \mathbf{0}$ (soft landing). The co-state initial values $\boldsymbol{\lambda}_{v,0}$ and $T_3$ are solved numerically via a shooting method.

### Optimal Cruise Altitude

Total mission time as a function of $z_{cruise}$:

$$T_{total}(z_{cruise}) = T_{takeoff}(z_{cruise}) + T_{cruise}(z_{cruise}) + T_{descent}(z_{cruise})$$

$$= 2\sqrt{\frac{z_{cruise}}{a_{z,max}}} + \frac{d_{xy}}{v_{gnd}(z_{cruise})} + T_3(z_{cruise})$$

Optimal cruise altitude:

$$z_{cruise}^* = \arg\min_{z \in [z_{min}, z_{max}]} T_{total}(z)$$

solved by 1D grid search or golden-section search.

### Energy Model (3D)

Power consumption accounts for both horizontal and vertical motion:

$$P(\mathbf{v}, z) = P_{hover} + k_{drag}(z) \cdot \|\mathbf{v}_{rel}(z)\|^3 + m \cdot g \cdot \dot{z}^+$$

where $\dot{z}^+ = \max(\dot{z}, 0)$ is the climb-power term and the cubic drag term follows from $P = F_{drag} \cdot v$. Total energy:

$$E_{mission} = \int_0^{T_{mission}} P(\mathbf{v}(t), z(t)) \, dt$$

### Position Update (Euler Integration)

$$\mathbf{p}(t + \Delta t) = \mathbf{p}(t) + \mathbf{v}(t)\,\Delta t + \tfrac{1}{2}\mathbf{u}(t)\,\Delta t^2$$
$$\mathbf{v}(t + \Delta t) = \mathbf{v}(t) + \mathbf{u}(t)\,\Delta t$$

with $\Delta t = 1/48$ s.

### Landing Condition

$$\|\mathbf{p}(t) - \mathbf{p}_{goal}\| < r_{land} \quad \text{and} \quad z(t) \le 0.05 \text{ m} \quad \text{and} \quad \|\mathbf{v}(t)\| < v_{land,max}$$

---

## Implementation

```python
# Key constants
V_MAX        = 5.0    # m/s   max horizontal speed
A_Z_MAX      = 2.0    # m/s²  max vertical acceleration
A_MAX        = 3.0    # m/s²  max 3D acceleration (descent phase)
Z_CRUISE     = None   # m     set by optimiser (search range 1–6 m)
Z_MIN        = 0.5    # m     minimum allowed altitude
Z_MAX        = 8.0    # m     maximum allowed altitude
R_LAND       = 0.20   # m     landing capture radius
V_LAND_MAX   = 0.3    # m/s   maximum speed at touchdown
DT           = 1/48   # s     control timestep
P_HOVER      = 30.0   # W     hover power
K_DRAG_0     = 0.08   # kg/m  sea-level drag coefficient
ALPHA_Z      = 0.05   # 1/m   altitude drag scaling
V_WIND_0     = 1.5    # m/s   wind speed at z_ref
Z_REF        = 2.0    # m     wind reference altitude
MASS         = 0.5    # kg
G            = 9.81   # m/s²

DEPOT = np.array([-4.0,  0.0, 0.0])   # ground level
GOAL  = np.array([ 3.5,  2.5, 0.0])   # ground level

# Phase FSM states: TAKEOFF → CRUISE → DESCENT → LANDED
def wind_speed(z):
    return V_WIND_0 * z / Z_REF

def k_drag(z):
    return K_DRAG_0 * (1 + ALPHA_Z * z)

def total_mission_time(z_cruise, d_xy):
    T_to = 2 * np.sqrt(z_cruise / A_Z_MAX)
    v_wind = wind_speed(z_cruise)
    v_gnd  = np.sqrt(max(V_MAX**2 - v_wind**2, 0.1))
    T_cr   = d_xy / v_gnd
    T_desc = shooting_descent_time(z_cruise)  # numerical shooting
    return T_to + T_cr + T_desc

def optimal_cruise_altitude(d_xy, z_range=np.linspace(0.5, 6.0, 56)):
    times = [total_mission_time(z, d_xy) for z in z_range]
    return z_range[np.argmin(times)]
```

---

## Key Parameters

| Parameter | Value | Notes |
|-----------|-------|-------|
| Depot position | (-4.0, 0.0, 0.0) m | Ground level |
| Goal position | (3.5, 2.5, 0.0) m | Ground level |
| Max horizontal speed | 5.0 m/s | |
| Max vertical acceleration | 2.0 m/s² | Takeoff bang-bang |
| Max 3D acceleration (descent) | 3.0 m/s² | Pontryagin control bound |
| Cruise altitude search range | 0.5 – 6.0 m | Grid of 56 points |
| Landing capture radius | 0.20 m | |
| Max touchdown speed | 0.3 m/s | Soft-landing constraint |
| Control frequency | 48 Hz | dt = 1/48 s |
| Hover power | 30 W | |
| Sea-level drag coefficient | 0.08 kg/m | |
| Altitude drag scaling $\alpha_z$ | 0.05 m$^{-1}$ | |
| Wind speed at 2 m | 1.5 m/s | Linear profile |
| Drone mass | 0.5 kg | |
| Altitude bounds | [0.5, 8.0] m | |

---

## Expected Output

- **3D trajectory plot**: four phases color-coded (takeoff: orange, cruise: red, descent: magenta, land: green star), depot blue square
- **Altitude vs time**: S-curve takeoff, flat cruise, Pontryagin-optimal curved descent reaching $z = 0$
- **Speed vs time**: bang-bang vertical speed during takeoff, near-constant cruise, controlled deceleration in descent
- **Optimal altitude sweep**: $T_{total}(z_{cruise})$ curve with minimum marked — shows trade-off between longer takeoff and less wind drag at high altitudes
- **Wind correction vector field**: 2D slice at $z = z_{cruise}^*$ showing effective ground-speed vectors
- **Energy breakdown**: stacked bar for each phase (takeoff, cruise, descent)
- **Terminal metrics**: optimal $z_{cruise}^*$, total mission time, energy consumed, final landing error, touchdown speed
- **Animation (GIF)**: drone icon traversing all four phases with altitude color-map

---

## Extensions

1. Curved 3D takeoff: replace bang-bang climb with a minimum-jerk spline that smoothly connects $\mathbf{p}_0$ and the cruise waypoint
2. Obstacle corridor avoidance: add altitude-exclusion bands (e.g., a building at $z \in [3, 5]$ m forces the cruise layer above or below) and re-solve $z_{cruise}^*$ subject to the exclusion
3. Stochastic wind: model $v_{wind}(z, t)$ as a Dryden turbulence model and run Monte-Carlo trials to find the $z_{cruise}$ that minimises $\mathbb{E}[T_{mission}]$
4. Battery-aware optimisation: add a secondary objective minimising energy $E_{mission}$ and find the Pareto front in the $(T_{mission}, E_{mission})$ plane as a function of $z_{cruise}$
5. Multi-delivery extension: chain two deliveries (depot → A → B → depot) and solve the joint altitude profile using dynamic programming

---

## Related Scenarios

- Original 2D version: [S021](../S021_point_delivery.md)
- 3D obstacle avoidance: [S022](../S022_obstacle_avoidance_delivery.md)
- Wind disturbance: [S024](../S024_wind_compensation.md)
- 3D reference (pursuit domain): [S001](../../01_pursuit_evasion/S001_basic_intercept.md), [S003](../../01_pursuit_evasion/S003_low_altitude_tracking.md)
