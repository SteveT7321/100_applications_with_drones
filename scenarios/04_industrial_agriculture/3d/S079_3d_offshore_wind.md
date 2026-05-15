# S079 3D Upgrade — Offshore Wind Installation

**Domain**: Industrial & Agriculture | **Difficulty**: ⭐⭐⭐⭐ | **Status**: `[ ]` Not Started
**Based on**: [S079 original](../S079_offshore_wind.md)

---

## What Changes in 3D

The original S079 already carries a three-dimensional state vector, but its 3D treatment is
shallow in two respects. First, the turbine mast is modelled as a single featureless point at
$(x_p, y_p) = (10, 0)$ m; the nacelle sits at only $z_{mast} = 20$ m, well within the flight
envelope of a standard delivery drone. Real offshore wind turbines have hub heights of 100 m or
more, nacelle lengths of 15–20 m, and rotating blades that sweep a disc of radius 60–90 m. The
drone must plan a full-altitude approach corridor, fly past the rotor disc without entering the
swept volume, and descend vertically to the nacelle service hatch. Second, the wake shed
downstream of each turbine is a cone-shaped region of high turbulence that must be avoided to
prevent loss of control. A wind farm contains many turbines; the drone must route between them
while side-stepping each wake cone. Neither the hub-height approach geometry, the rotating-blade
swept-volume constraint, nor the inter-turbine wake avoidance is captured in the original.

This upgrade adds:

1. **Nacelle-height approach (100 m+)**: drone departs supply ship at sea level, climbs to hub
   height $ z_{hub} = 100$ m, then executes a horizontal approach corridor to the nacelle before
   descending 5 m to the service hatch on the heaving nacelle top.
2. **Turbine 3D geometry with rotating blades**: the rotor disc is modelled as a forbidden
   cylinder of radius $R_{blade} = 63$ m centred on the hub; the blade phase $\psi(t)$ rotates
   at $\omega_{rotor} = 1.57$ rad/s (15 RPM); a per-blade swept envelope is enforced as a
   keep-out zone in 3D.
3. **3D wake avoidance**: each operating turbine casts a downstream Gaussian wake cone; the drone
   must re-route if its planned path intersects any cone above the wake-turbulence threshold.

---

## Problem Definition

**Setup**: A service vessel is stationed at $(0, 0, 0)$ m (sea level). An offshore wind farm
contains $N_{turb} = 6$ turbines arranged in a $2 \times 3$ grid with $D_{spacing} = 500$ m
spacing. The target turbine for component delivery is at $(x_{hub}, y_{hub}) = (500, 250)$ m,
hub height $z_{hub} = 100$ m. The nacelle top (service hatch) heaves with ocean swell:

$$z_{hatch}(t) = z_{hub} + \delta_{tilt}(t), \quad \delta_{tilt}(t) = A_{swell}
  \sin\!\left(\frac{2\pi t}{T_{swell}}\right)$$

where $A_{swell} = 0.8$ m and $T_{swell} = 6$ s (same swell model as S079 but now applied at
altitude). Wind blows from the west (+x direction) at a steady $U_\infty = 12$ m/s, imposing
both a horizontal drift disturbance on the drone and wake cones extending in the +x direction
behind each turbine. Three rotor blades are modelled; each sweeps the rotor disc plane (the
$yz$-plane at hub position $x_{hub}$) at $\omega_{rotor}$ rad/s.

**Roles**:
- **Drone**: single service agent; 3D quadrotor with full 6-DOF linearised dynamics; LQR +
  swell feedforward controller (inherited from S079); additionally equipped with a path planner
  that computes a collision-free corridor past the rotor disc and around all wake cones.
- **Turbine target**: nacelle service hatch at $(x_{hub}, y_{hub}, z_{hatch}(t))$; heave
  motion imposes a time-varying vertical target identical in structure to S079.
- **Rotor disc**: rotating keep-out volume — three blades of length $R_{blade} = 63$ m attached
  to the hub; each blade occupies a swept cylinder of radius $r_{blade} = 1.5$ m centred on its
  span axis; the drone may not enter any blade swept volume or the rotor disc interior.
- **Wake cones**: each turbine generates a downstream wake cone of half-angle
  $\beta_{wake} = 5°$ and length $L_{wake} = 6 D_{rotor}$ (approximately 756 m); wind speed
  inside the cone is reduced and turbulence intensity is elevated; the drone avoids entering
  any cone by routing above or laterally around the cone axis.

**Objective**: Deliver a component to the nacelle hatch precisely and safely at altitude 100 m.
Success requires all of the following:

1. Final 3D position error $\|\mathbf{p}_{drone} - \mathbf{p}_{hatch}\| \leq r_{land} = 0.20$ m
   at the moment of landing.
2. Relative vertical velocity $|\dot{z}_{drone} - \dot{z}_{hatch}| \leq v_{land,max} = 0.1$ m/s.
3. Drone never enters the rotor disc forbidden cylinder at any time during approach.
4. Drone never enters any wake cone of any turbine in the farm.
5. Monte Carlo success rate $\geq 85\%$ over $N_{trials} = 100$ wind and swell realisations.

**Controller variants** (three compared, as in S079):
1. **PID no feedforward** — phase-lag limited; now also fails to pre-correct for hub altitude.
2. **LQR no feedforward** — optimal feedback; reduced lag but no swell prediction.
3. **LQR + swell feedforward + 3D path planner** (proposed) — feedforward inherited from S079;
   new 3D path planner pre-computes a waypoint corridor that clears the rotor disc and all wakes.

---

## Mathematical Model

### Platform Heave at Altitude

The hatch vertical position inherits the S079 swell model but is anchored at $z_{hub}$:

$$z_{hatch}(t) = z_{hub} + A_{swell} \sin\!\left(\frac{2\pi t}{T_{swell}}\right)$$

$$\dot{z}_{hatch}(t) = A_{swell} \cdot \frac{2\pi}{T_{swell}}
  \cos\!\left(\frac{2\pi t}{T_{swell}}\right)$$

The predictive reference with look-ahead $\tau = 0.5$ s is:

$$z_{ref}(t + \tau) = z_{hub} + A_{swell}
  \sin\!\left(\frac{2\pi (t + \tau)}{T_{swell}}\right)$$

### 3D Nacelle Approach Corridor

The drone is assigned a three-phase waypoint sequence in 3D:

$$\mathbf{w}_0 = (0,\; 0,\; 0)\ \text{m} \quad \text{(ship deck)}$$

$$\mathbf{w}_1 = (0,\; 0,\; z_{hub})\ \text{m} \quad \text{(climb phase — clear of all wakes)}$$

$$\mathbf{w}_2 = (x_{hub} - d_{safe},\; y_{hub},\; z_{hub})\ \text{m}
  \quad \text{(approach holding point, } d_{safe} = R_{blade} + 20\ \text{m)}$$

$$\mathbf{w}_3 = (x_{hub},\; y_{hub},\; z_{hatch}(t_{arrive}))\ \text{m}
  \quad \text{(final descent to hatch)}$$

The segment $\mathbf{w}_1 \to \mathbf{w}_2$ is routed laterally by $y_{bypass} = R_{blade} + 30$ m
to stay clear of the wake cone axis. The segment $\mathbf{w}_2 \to \mathbf{w}_3$ is a direct
vertical descent executed only when the blade phase places no blade within a safety cone of
half-angle $\gamma_{safe} = 15°$ of the descent corridor.

### Rotor Disc Keep-Out Geometry

The hub is at $\mathbf{h} = (x_{hub}, y_{hub}, z_{hub})$. The rotor disc lies in the plane
$x = x_{hub}$ with normal vector $\hat{\mathbf{x}}$. Blade $k$ ($k = 0, 1, 2$) has tip position:

$$\mathbf{b}_k(t) = \mathbf{h} + R_{blade}
  \begin{pmatrix}
    0 \\
    \cos\!\left(\psi_k(t)\right) \\
    \sin\!\left(\psi_k(t)\right)
  \end{pmatrix}, \quad
  \psi_k(t) = \omega_{rotor}\,t + \frac{2\pi k}{3}$$

The forbidden swept volume at any instant is the union of three cylinders:

$$\mathcal{F}(t) = \bigcup_{k=0}^{2} \left\{\,
  \mathbf{p} \;\Big|\;
  \text{dist}\!\left(\mathbf{p},\; \overline{\mathbf{h}\,\mathbf{b}_k(t)}\right) < r_{blade}
\right\}$$

where $\text{dist}(\mathbf{p}, \overline{\mathbf{h}\,\mathbf{b}_k})$ denotes the point-to-segment
distance in 3D. The disc interior is additionally forbidden:

$$\mathcal{D} = \left\{\, \mathbf{p} \;\Big|\;
  |p_x - x_{hub}| < \epsilon_{disc} \;\wedge\;
  \sqrt{(p_y - y_{hub})^2 + (p_z - z_{hub})^2} < R_{blade}
\right\}$$

with $\epsilon_{disc} = 2$ m (half-thickness of the disc forbidden plane).

A collision event is declared if at any timestep:

$$\mathbf{p}_{drone}(t) \in \mathcal{F}(t) \cup \mathcal{D}$$

### Gate-Open Condition for Descent

The drone may commit to final descent from $\mathbf{w}_2$ only when all three blades are
sufficiently far from the descent corridor. Define the minimum blade clearance angle:

$$\Phi_{clear}(t) = \min_{k \in \{0,1,2\}} \left|
  \psi_k(t) \;\mathrm{mod}\; 2\pi - \frac{\pi}{2}
\right|$$

(angle to the nearest blade from the 12-o'clock position directly above the hub).

Descent is gated on $\Phi_{clear}(t) > \gamma_{gate} = 30°$. Since blades are equally spaced
at $120°$ intervals, this gate opens once per $T_{gate} = 120° / \omega_{rotor}$ seconds.

### 3D Wake Cone Model

Turbine $i$ at position $(x_i, y_i, z_{hub})$ generates a downstream wake cone with axis
aligned with the wind direction $\hat{\mathbf{U}} = (1, 0, 0)$:

$$\mathcal{W}_i = \left\{\, \mathbf{p} \;\Big|\;
  (p_x - x_i) > 0 \;\wedge\;
  \frac{\sqrt{(p_y - y_i)^2 + (p_z - z_{hub})^2}}{p_x - x_i} < \tan(\beta_{wake})
  \;\wedge\; (p_x - x_i) < L_{wake}
\right\}$$

The drone avoids all wakes by planning routes with lateral offset $y_{bypass}$ or vertical
climb above $z_{hub} + R_{blade}$ before traversing any $x$-column that contains a downstream
wake. The wake avoidance penalty added to the path cost functional is:

$$J_{wake} = \sum_{i=1}^{N_{turb}} \int_0^{T_{mission}}
  \mathbb{1}\!\left[\mathbf{p}(t) \in \mathcal{W}_i\right] \cdot C_{wake}\, dt$$

where $C_{wake} = 100$ (dimensionless penalty per second inside a wake).

### Extended State Vector (Altitude Dynamics)

The state vector from S079 is retained unchanged:

$$\mathbf{x} = \begin{pmatrix}
  p_x & p_y & p_z & v_x & v_y & v_z & \phi & \theta & \dot{\phi} & \dot{\theta}
\end{pmatrix}^\top \in \mathbb{R}^{10}$$

A steady wind disturbance $U_\infty$ in the $x$-direction is added to $v_x$ at each timestep
in addition to the Gaussian gust used in S079. The LQR $Q$-matrix position weight on $p_z$ is
increased to $Q_{p_z} = 200$ (from 50 in S079) to enforce tighter altitude tracking at 100 m
where the consequence of vertical error is a rotor-strike.

### LQR + Feedforward (Altitude-Augmented)

The LQR gain $K$ is re-solved with the altitude-augmented weight matrix:

$$Q_{3D} = \text{diag}(20,\; 20,\; 200,\; 3,\; 3,\; 8,\; 5,\; 5,\; 1,\; 1)$$

$$R_{3D} = \text{diag}(0.05,\; 1.0,\; 1.0)$$

(thrust weight halved to allow faster altitude corrections at 100 m). The control law is:

$$\mathbf{u}(t) = -K\,\mathbf{e}(t), \quad \mathbf{e}(t) = \mathbf{x}(t) - \mathbf{x}_{ref}(t)$$

where $\mathbf{x}_{ref}(t)$ tracks the current waypoint with swell feedforward active only
during the final descent phase.

### Landing Window Detection (Unchanged from S079)

$$\Delta\dot{z}(t) = \dot{z}_{drone}(t) - \dot{z}_{hatch}(t)$$

Landing window opens at first $t_w$ with $|\Delta\dot{z}(t_w)| \leq v_{land,max}$. Success
requires all three conditions inside $[t_w,\; t_w + T_{window}]$:

$$\left| z_{drone}(t) - z_{hatch}(t) \right| \leq r_{land}, \quad
  \left| \Delta\dot{z}(t) \right| \leq v_{land,max}, \quad
  \left\| \mathbf{p}_{xy,drone}(t) - (x_{hub}, y_{hub}) \right\| \leq r_{land}$$

### Wind Gust and Steady-Wind Disturbance

Horizontal disturbances combine steady wind with zero-mean gusts:

$$d_x(t) = U_\infty + w_x(t), \quad d_y(t) = w_y(t)$$

$$w_x(t),\; w_y(t) \sim \mathcal{N}(0,\; \sigma_w^2), \quad \sigma_w = 0.05\ \text{m/s}$$

The steady component $U_\infty = 12$ m/s is the dominant disturbance at hub height; the LQR
must reject this via position-error integral action embedded in the gain structure.

### Monte Carlo Success Rate

$$P_{land} = \frac{N_{successes}}{N_{trials}} \times 100\%, \quad N_{trials} = 100$$

Each trial independently samples gust sequences, a random swell phase offset
$\phi_0 \sim \mathcal{U}[0, 2\pi)$, and a random wind speed perturbation
$\Delta U \sim \mathcal{N}(0, 2^2)$ m/s.

---

## Key 3D Additions

- **Nacelle-height approach**: hub at $z_{hub} = 100$ m; drone must climb 100 m before any
  horizontal approach; the LQR reference waypoint sequence drives this climb explicitly.
- **Rotating blade keep-out volume**: three-blade rotor modelled as three point-to-segment
  forbidden cylinders rotating at 15 RPM; gate-open logic delays final descent until blade
  phase is safe.
- **Rotor disc interior forbidden zone**: a 4 m thick disc slab of radius 63 m centred on the
  hub is always forbidden regardless of blade phase.
- **3D wake cone avoidance**: six-turbine farm; each turbine emits a $5°$ half-angle downstream
  cone of length $6 D_{rotor} \approx 756$ m; lateral bypass route planned at $y_{bypass}$
  offset to clear all cones.
- **Altitude-weighted LQR**: $Q_{p_z}$ raised from 50 to 200; thrust weight halved; necessary
  to achieve sub-0.20 m vertical error at 100 m altitude in 12 m/s steady wind.
- **Steady wind disturbance**: $U_\infty = 12$ m/s added to $d_x$ at every timestep; dominant
  disturbance source at hub height; requires stronger integral-like position-feedback action.

---

## Key Parameters

| Parameter | Symbol | Value |
|-----------|--------|-------|
| Hub height | $z_{hub}$ | 100 m |
| Rotor blade length | $R_{blade}$ | 63 m |
| Blade cylinder radius | $r_{blade}$ | 1.5 m |
| Rotor speed | $\omega_{rotor}$ | 1.57 rad/s (15 RPM) |
| Disc forbidden half-thickness | $\epsilon_{disc}$ | 2 m |
| Descent gate clearance angle | $\gamma_{gate}$ | 30° |
| Safe approach hold-off distance | $d_{safe}$ | $R_{blade} + 20$ m = 83 m |
| Lateral bypass offset | $y_{bypass}$ | $R_{blade} + 30$ m = 93 m |
| Wake cone half-angle | $\beta_{wake}$ | 5° |
| Wake cone length | $L_{wake}$ | 756 m ($6 D_{rotor}$) |
| Wake penalty coefficient | $C_{wake}$ | 100 |
| Turbine grid spacing | $D_{spacing}$ | 500 m |
| Number of turbines | $N_{turb}$ | 6 |
| Steady wind speed | $U_\infty$ | 12 m/s |
| Swell amplitude | $A_{swell}$ | 0.8 m |
| Swell period | $T_{swell}$ | 6 s |
| Feedforward look-ahead | $\tau$ | 0.5 s |
| Landing position tolerance | $r_{land}$ | 0.20 m |
| Landing velocity tolerance | $v_{land,max}$ | 0.1 m/s |
| Landing window duration | $T_{window}$ | 3 s |
| Wind gust std dev | $\sigma_w$ | 0.05 m/s |
| Drone mass | $m$ | 1.0 kg |
| LQR $Q_{p_z}$ (altitude weight) | — | 200 |
| LQR $R$ (thrust weight) | — | 0.05 |
| Monte Carlo trials | $N_{trials}$ | 100 |
| Simulation timestep | $\Delta t$ | 0.02 s |
| Mission horizon | $T_{max}$ | 120 s |

---

## Expected Output

- **3D farm overview plot**: top-down view of the $2 \times 3$ turbine grid; wake cones shaded
  in light grey extending in the +x direction; planned drone path shown in red; bypass route
  visible as a lateral detour around the target turbine's upstream approach.
- **3D approach trajectory**: `mpl_toolkits.mplot3d` plot showing the full drone path from sea
  level to nacelle; turbine mast as a vertical grey cylinder; rotor disc as a grey translucent
  disc at hub height; three blade lines rotating (shown at $t = t_{arrive}$); LQR+FF (red) and
  LQR-only (blue dashed) compared.
- **Altitude vs time**: $p_z(t)$ time series for drone (both controllers) and hatch $z_{hatch}(t)$;
  shows three-phase climb: vertical rise to 100 m, horizontal transit, final descent; $\pm r_{land}$
  band around hatch altitude during descent phase.
- **Blade clearance angle vs time**: $\Phi_{clear}(t)$ during the final descent phase; gate
  threshold $\gamma_{gate} = 30°$ shown as a dashed horizontal line; descent commit moment
  highlighted.
- **Relative vertical velocity vs time**: $\Delta\dot{z}(t)$ for both controllers; $\pm v_{land,max}$
  band; shows that phase-lag is amplified at 100 m altitude with stronger wind; LQR+FF maintains
  tighter synchronisation.
- **Wake zone violation map**: 2D $xy$-projection showing each turbine's wake cone footprint
  at $z = z_{hub}$; drone path coloured by in-wake / out-of-wake status; zero violations
  expected for the planned bypass route.
- **Monte Carlo success rate bar chart**: 100-trial success rates for three controller variants;
  LQR+FF+3D planner expected $\geq 85\%$; PID expected $\leq 40\%$.
- **Animation (GIF)**: 3D side-on view of the full mission; rotor blades rotate in real time;
  drone dot climbs, transits, and descends; hatch platform heaves; blade clearance gate opens
  and closes; annotations show $|\Delta\dot{z}|$, $z_{err}$, and current mission phase.

**Expected metric targets** (LQR+FF + 3D planner, seed 0):

| Metric | Target |
|--------|--------|
| Landing success (single run) | YES |
| Rotor disc violations | 0 |
| Wake cone violations | 0 |
| Window miss count | $\leq 1$ |
| Final 3D position error | $< 0.20$ m |
| Peak $|\Delta\dot{z}|$ during descent | $< 0.5$ m/s |
| MC success rate (LQR+FF+planner) | $\geq 85\%$ |
| MC success rate (LQR only) | $\leq 55\%$ |
| MC success rate (PID) | $\leq 40\%$ |

---

## Extensions

1. **JONSWAP swell spectrum at hub height**: replace the single-frequency hatch heave with a
   broadband JONSWAP spectrum; re-derive the feedforward as an FIR filter fitted to the
   spectrum peak; compare landing success rate under irregular versus regular swell at 100 m.
2. **Variable wind shear profile**: model the atmospheric boundary layer with a power-law
   wind-speed profile $U(z) = U_{10}(z/10)^\alpha$, $\alpha = 0.14$; the drone experiences
   increasing head-wind as it climbs; augment the LQR disturbance-rejection term with
   altitude-scheduled feedforward based on the known wind profile.
3. **Multi-drone maintenance swarm**: assign three drones to three different turbines
   simultaneously; use a central scheduler to stagger departure times and bypass corridors such
   that drones never share a wake cone or rotor approach corridor; minimise total fleet mission
   time subject to the blade-phase gate constraint at each turbine.
4. **Online blade-phase estimation**: remove the assumption of known $\omega_{rotor}$ and
   $\psi_0$; implement an onboard vision-based estimator that detects blade edges and predicts
   the next gate-open time; evaluate estimation latency and its effect on safe descent timing.
5. **MPC with rotor-strike hard constraint**: replace the LQR + waypoint planner with a
   receding-horizon MPC that treats the rotating blade keep-out volumes as time-varying hard
   constraints; enforce zero-penetration at every prediction horizon step; compare computational
   load and safety margin versus the gate-open heuristic.

---

## Related Scenarios

- Original 2D/shallow-3D version: [S079 Offshore Wind Installation](../S079_offshore_wind.md)
- Turbine proximity operations reference: [S062 Wind Turbine Blade Inspection](../S062_wind_turbine.md)
- High-altitude precision delivery: [S066 Cooperative Crane Lift](../S066_cooperative_crane.md)
- Moving target landing (low altitude): [S023 Moving Landing Pad](../../02_logistics_delivery/S023_moving_landing_pad.md)
- LQR under extreme wind disturbance: [S058 Typhoon Eye Probing](../../03_environmental_sar/S058_typhoon.md)
- Follow-up (6-DOF moving deck): [S080 Underground Pipe Inspection](../S080_underground_pipe.md)
