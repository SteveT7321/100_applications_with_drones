# S076 3D Upgrade — Thermal Management in Hot Environment

**Domain**: Industrial & Agriculture | **Difficulty**: ⭐⭐⭐ | **Status**: `[ ]` Not Started
**Based on**: [S076 original](../S076_thermal_management.md)

---

## What Changes in 3D

The original S076 fixes altitude at a constant z = 2.0 m throughout the entire lawnmower mission. The thermal model accounts only for speed-dependent horizontal power $P_{motor}(v) = P_{hover} + k_v v^2$, ignoring the substantial additional power cost of climbing, the cooling benefit of higher-altitude flight where ambient air temperature decreases with altitude, and the feasibility of routing the drone through a 3D safe thermal corridor that avoids superheated air masses close to the ground surface.

In the 3D upgrade:
- The ambient temperature field becomes a 3D scalar function $T_{amb}(x, y, z)$, with ground-surface heat rising from sun-baked soil and a standard atmospheric lapse rate above the surface boundary layer.
- Vertical flight incurs additional climb/descent motor power $P_{vert}(v_z)$, so altitude changes couple directly into the thermal load.
- The drone searches for a 3D safe corridor through the heat field that satisfies $T_{motor}(t) \leq T_{max}$ while minimising total mission time.
- Battery capacity is derated as a function of both cell temperature and ambient temperature, adding a coupled battery-thermal constraint alongside the motor constraint.

---

## Problem Definition

**Setup**: A single inspection drone surveys a $100 \times 100$ m agricultural field using a
boustrophedon pattern. The field surface is at $z = 0$ m. The drone operates in a hot, calm
environment with a nonuniform 3D temperature field: near the surface ($z < 2$ m) radiative
heating produces a superheated boundary layer; above 2 m the temperature decreases at the
dry adiabatic lapse rate. The drone may vary altitude between $z_{min} = 1.0$ m and
$z_{max} = 6.0$ m during the mission.

**Roles**:
- **Drone**: single UAV executing a 3D lawnmower path; strip width $d = 5$ m; subject to
  coupled motor thermal dynamics and battery thermal derating at every timestep.
- **3D thermal environment**: spatially varying $T_{amb}(x, y, z)$ driven by surface
  heating, a superheated boundary layer near the ground, and the atmospheric lapse rate
  above 2 m; no lateral (x, y) variation assumed — the thermal structure is horizontally
  homogeneous and altitude-dependent only.

**Objective**: Complete the full lawnmower inspection while satisfying:
1. $T_{motor}(t) \leq T_{max} = 85°C$ at all times.
2. Battery available capacity $Q_{avail}(T_{bat}) \geq Q_{min}$ (no thermal derating cutoff).
3. Minimum total mission time $T_{mission}$.

**Comparison strategies**:
1. **Fixed altitude 2D baseline** — fly at constant $z = 2.0$ m (replicates S076 original);
   reactive throttle only; no altitude adjustment.
2. **Altitude-optimised cruise** — before each strip, select the altitude $z^*$ that minimises
   the steady-state motor temperature $T_{m,\infty}(v_0, z)$ subject to $z \in [z_{min}, z_{max}]$;
   fly each strip at that altitude.
3. **3D thermal corridor MPC** — at each timestep use the 3D ambient temperature ahead on
   the planned path to predictively schedule both altitude adjustments and cooling pauses,
   minimising the weighted sum of mission time and peak motor temperature.

---

## Mathematical Model

### 3D Ambient Temperature Field

The ambient temperature is a function of altitude only (horizontally homogeneous):

$$T_{amb}(z) = \begin{cases}
T_{surf} + \Gamma_{bl}(z - z_{ref}) & z < z_{bl} \\
T_{bl} - \Gamma_{atm}(z - z_{bl})  & z \geq z_{bl}
\end{cases}$$

where:
- $T_{surf} = 55°C$ — surface temperature of superheated soil at midday
- $z_{ref} = 0$ m — ground reference altitude
- $\Gamma_{bl} = -5°C/m$ — boundary-layer lapse rate (temperature decreases sharply upward within
  the hot boundary layer; note the sign convention: $\Gamma_{bl} < 0$ means temperature falls
  with increasing $z$ inside the layer)
- $z_{bl} = 2.0$ m — top of the superheated boundary layer; $T_{bl} = T_{surf} + \Gamma_{bl} z_{bl} = 45°C$
- $\Gamma_{atm} = 0.0065°C/m$ — dry adiabatic lapse rate above the boundary layer

The resulting profile delivers $T_{amb}(0) = 55°C$ at the surface, $T_{amb}(2) = 45°C$ at the
top of the boundary layer, and $T_{amb}(6) \approx 44.97°C$ at the maximum inspection altitude.
Flying higher always reduces ambient temperature, but the benefit saturates above 2 m.

### 3D Motor Power Model

Total motor power now includes horizontal cruise power and vertical climb/descent power:

$$P_{motor}(v_h, v_z) = P_{hover} + k_v v_h^2 + k_z v_z^2$$

where $v_h = \sqrt{v_x^2 + v_y^2}$ is the horizontal speed, $v_z$ is the vertical speed
(positive upward), and $k_z$ is the climb power coefficient. The hover power $P_{hover}$
already represents the minimum thrust needed to maintain a fixed altitude.

### First-Order Motor Thermal Model (3D)

The motor thermal ODE is identical in structure to S076 but now uses $T_{amb}(z(t))$:

$$\tau \frac{dT_m}{dt} = R_{th} \cdot P_{motor}(v_h, v_z) - \bigl(T_m - T_{amb}(z(t))\bigr)$$

The altitude $z(t)$ is now a state variable updated at each timestep. The steady-state
motor temperature at horizontal speed $v_h$, zero vertical speed, and altitude $z$ is:

$$T_{m,\infty}(v_h, z) = T_{amb}(z) + R_{th} \bigl(P_{hover} + k_v v_h^2\bigr)$$

### Optimal Cruise Altitude

For a fixed horizontal speed $v_h = v_0$, the altitude that minimises $T_{m,\infty}$ is the
altitude that minimises $T_{amb}(z)$. Because $T_{amb}(z)$ is monotonically non-increasing
above the boundary layer, the optimal cruise altitude is the maximum permissible altitude:

$$z^* = z_{max}$$

subject to $z \in [z_{min}, z_{max}]$. Within the boundary layer ($z < z_{bl}$) the drone
should climb as fast as possible through the hot zone. The altitude gain $\Delta z = z^* - z_{ref}$
incurs a thermal transient cost from vertical power; the altitude selection must account for
the time needed to reach $z^*$ and the extra thermal load during the climb.

### 3D Altitude Command — Climb/Descent Law

The altitude command at each waypoint transition uses a trapezoidal velocity profile:

$$z_{cmd}(t) = z_{current} + \Delta z \cdot f_{trap}(t, t_{climb})$$

$$f_{trap}(t, T) = \begin{cases}
\frac{1}{2}\left(\frac{t}{T/2}\right)^2          & t \leq T/2 \\
1 - \frac{1}{2}\left(\frac{T-t}{T/2}\right)^2   & t > T/2
\end{cases}$$

The peak climb/descent speed is $v_{z,max} = 1.5$ m/s. The time required to change altitude
by $\Delta z$ at peak speed is $t_{climb} = |\Delta z| / v_{z,max}$.

### Battery Thermal Derating

Battery available capacity $Q_{avail}$ is derated when the battery cell temperature $T_{bat}$
rises above the nominal operating temperature $T_{bat,nom} = 25°C$:

$$Q_{avail}(T_{bat}) = Q_{nominal} \cdot \left[1 - \beta_{bat} \max\!\left(0,\; T_{bat} - T_{bat,nom}\right)\right]$$

where $\beta_{bat} = 0.005\ °C^{-1}$ is the capacity derating coefficient. Battery temperature
evolves by a similar first-order model:

$$\tau_{bat} \frac{dT_{bat}}{dt} = R_{bat} \cdot P_{total} - \bigl(T_{bat} - T_{amb}(z(t))\bigr)$$

where $P_{total} = P_{motor}(v_h, v_z) / \eta_{esc}$ is the total drawn power (including ESC
losses, $\eta_{esc} = 0.92$), $R_{bat} = 0.3°C/W$ is the battery pack thermal resistance,
and $\tau_{bat} = 120$ s is the battery pack thermal time constant (larger mass than motor).

The constraint $Q_{avail}(T_{bat}) \geq Q_{min} = 0.80 \cdot Q_{nominal}$ limits the maximum
permissible battery temperature to:

$$T_{bat,max} = T_{bat,nom} + \frac{1 - Q_{min}/Q_{nominal}}{\beta_{bat}} = 65°C$$

### 3D Safe Corridor Definition

A position $(x, y, z)$ lies inside the 3D safe thermal corridor if both constraints are met:

$$T_{m,\infty}(v_0, z) \leq T_{max} \quad \text{and} \quad T_{bat,ss}(z) \leq T_{bat,max}$$

where $T_{bat,ss}(z) = T_{amb}(z) + R_{bat} \cdot P_{total}(v_0, 0) / \eta_{esc}$ is the
steady-state battery temperature at altitude $z$ during cruise. Since both constraints depend
only on altitude, the safe corridor reduces to a single altitude band $[z_{safe,lo}, z_{safe,hi}]$:

$$z_{safe,lo} = \min\!\left\{z : T_{amb}(z) + R_{th} \bigl(P_{hover} + k_v v_0^2\bigr) \leq T_{max}\right\}$$
$$z_{safe,hi} = z_{max}$$

For the given parameters, $z_{safe,lo}$ is computed numerically by inverting $T_{amb}(z)$. The
drone must remain inside $[z_{safe,lo}, z_{max}]$ during cruise to avoid triggering reactive
cooling pauses.

### Mission Time with 3D Transitions

Total mission time includes the overhead of altitude changes at strip boundaries:

$$T_{mission} = \frac{L_{h,total}}{v_{eff}} + N_{turn} \cdot t_{turn} + N_{climb} \cdot t_{climb} + N_{pause} \cdot T_{pause}$$

where $L_{h,total}$ is the total horizontal path length, $v_{eff}$ is the effective horizontal
cruise speed, $N_{climb}$ is the number of altitude-change manoeuvres, and $t_{climb}$ is the
time cost per altitude change.

---

## Key 3D Additions

- **3D temperature gradient field**: altitude-dependent $T_{amb}(z)$ with superheated boundary
  layer below 2 m and atmospheric lapse rate above; surface temperature 55°C drops to 45°C at
  the boundary-layer top.
- **Altitude-varying thermal soak**: steady-state motor temperature $T_{m,\infty}(v_0, z)$ is a
  function of altitude; flying higher reduces ambient temperature and lowers the thermal equilibrium;
  gain is approximately $5°C$ between $z = 1$ m and $z = 6$ m.
- **3D safe corridor**: altitude band $[z_{safe,lo}, z_{max}]$ derived analytically from the
  motor and battery thermal constraints; the drone must remain above $z_{safe,lo}$ to cruise
  indefinitely without triggering a cooling pause.
- **Battery thermal derating vs altitude and temperature**: battery available capacity decreases
  above 65°C; at low altitude the higher ambient temperature accelerates battery heating and
  reduces the usable capacity, coupling battery health to the altitude selection.
- **Climb/descent power penalty**: vertical speed $v_z$ adds $k_z v_z^2$ to motor power; rapid
  altitude changes momentarily spike $T_m$; the trapezoidal velocity profile limits the peak
  thermal transient during altitude transitions.
- **3D trajectory visualisation**: true 3D Matplotlib trajectory with z-axis showing altitude
  variation, plus altitude-vs-time and battery-temperature-vs-time subplots.

---

## Key Parameters

| Parameter | Symbol | Value |
|-----------|--------|-------|
| Field dimensions | — | 100 × 100 m (horizontal) |
| Altitude range | $[z_{min}, z_{max}]$ | 1.0 – 6.0 m |
| Strip spacing | $d$ | 5 m |
| Nominal inspection speed | $v_0$ | 1.0 m/s |
| Surface temperature | $T_{surf}$ | 55 °C |
| Boundary layer top | $z_{bl}$ | 2.0 m |
| Ambient at $z_{bl}$ | $T_{bl}$ | 45 °C |
| Atmospheric lapse rate | $\Gamma_{atm}$ | 0.0065 °C/m |
| Motor temperature limit | $T_{max}$ | 85 °C |
| Warning threshold | $T_{warn}$ | 80 °C |
| Thermal resistance (motor) | $R_{th}$ | 0.8 °C/W |
| Thermal time constant (motor) | $\tau$ | 40 s |
| Thermal time constant (cooling) | $\tau_{cool}$ | 20 s |
| Hover power | $P_{hover}$ | 30 W |
| Horizontal drag power coefficient | $k_v$ | 15 W·s²/m² |
| Vertical climb power coefficient | $k_z$ | 20 W·s²/m² |
| Max climb/descent speed | $v_{z,max}$ | 1.5 m/s |
| Battery thermal resistance | $R_{bat}$ | 0.3 °C/W |
| Battery thermal time constant | $\tau_{bat}$ | 120 s |
| Battery derating coefficient | $\beta_{bat}$ | 0.005 °C⁻¹ |
| Battery temperature limit | $T_{bat,max}$ | 65 °C |
| Nominal battery capacity | $Q_{nominal}$ | 5000 mAh |
| ESC efficiency | $\eta_{esc}$ | 0.92 |
| Cooling pause duration | $T_{pause}$ | 30 s |
| Simulation timestep | $\Delta t$ | 0.1 s |
| Number of strips | $N$ | 20 |

---

## Expected Output

- **3D trajectory plot**: Matplotlib 3D axes showing the lawnmower path with real z-variation
  for each strategy; altitude encoded in both z-axis position and point colour (hot colourmap
  for motor temperature); boundary-layer top shown as a translucent horizontal plane at z = 2 m.
- **Altitude time series**: z(t) for each strategy; fixed-altitude baseline flat at 2 m;
  altitude-optimised strategy shows step changes at strip boundaries climbing toward $z_{max}$;
  MPC strategy shows smooth altitude modulation.
- **Motor temperature trace**: $T_m(t)$ for all three strategies; the baseline strategy breaches
  or approaches $T_{warn}$ while the altitude-optimised and MPC strategies maintain larger margins;
  exponential dips during cooling pauses visible.
- **Battery temperature trace**: $T_{bat}(t)$ showing slower thermal dynamics ($\tau_{bat} = 120$ s);
  lower altitudes produce higher battery temperatures; battery temperature approaches or exceeds
  $T_{bat,max}$ only in the fixed-altitude baseline.
- **3D safe corridor diagram**: altitude-vs-motor-temperature and altitude-vs-battery-temperature
  curves showing $z_{safe,lo}$ and the safe band; vertical coloured bands mark each strategy's
  nominal operating altitude.
- **Mission metrics bar chart**: mission time (min), pause count, peak motor temperature, and
  minimum $Q_{avail}/Q_{nominal}$ ratio grouped by strategy; illustrates the altitude-optimised
  strategy's advantage in reducing pauses and battery derating.

---

## Extensions

1. **Spatial surface temperature variation**: introduce a 2D heat map $T_{surf}(x, y)$ reflecting
   soil type, crop cover, and irrigation; route the lawnmower path through cooler strips first to
   minimise early-mission thermal soak.
2. **Wind-enhanced convective cooling**: add an altitude-dependent wind speed $W(z)$ that modifies
   the effective cooling time constant $\tau_{cool}(z) = \tau_0 / (1 + k_w W(z))$; higher
   altitude often means stronger wind and faster motor cooling.
3. **Diurnal temperature profile**: replace the constant surface temperature with a sinusoidal
   diurnal cycle peaking at solar noon; optimise the mission start time and altitude schedule
   jointly across the day.
4. **Four-motor heterogeneous thermal model**: each motor has independently varying $R_{th}$ and
   $C_{th}$ due to manufacturing tolerance; trigger altitude adjustment or speed reduction on the
   hottest motor; compare per-motor scheduling with the single-worst-case model.
5. **Hardware-in-the-loop validation**: feed the 3D thermal model with real telemetry from an
   ESC temperature sensor and a GPS altitude stream; run the MPC altitude scheduler on an
   embedded controller to validate the safe corridor predictions against measured motor temperatures.

---

## Related Scenarios

- Original 2D version: [S076](../S076_thermal_management.md)
- Lawnmower path geometry: [S048 Full-Area Coverage Scan](../../03_environmental_sar/S048_lawnmower.md)
- Environment-constrained mission: [S079 Night Flight Navigation](../S079_night_flight_navigation.md)
- Field coverage planning: [S067 Spray Overlap Optimisation](../S067_spray_overlap_optimization.md)
- 3D atmospheric environment reference: [S058 Typhoon Edge Sampling](../../03_environmental_sar/S058_typhoon.md)
