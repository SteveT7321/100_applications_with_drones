# S021 Point Delivery

**Domain**: Logistics & Delivery | **Difficulty**: ⭐ | **Status**: ✅ Completed

---

## Problem Definition

**Setup**: A single delivery drone departs from a depot at $\mathbf{p}_0 = (-4.0,\ 0.0,\ 2.0)$ m and must deliver a payload to a ground-level landing pad at $\mathbf{p}_{goal} = (3.5,\ 2.5,\ 0.0)$ m. The arena is obstacle-free.

**Objective**: Minimise total mission time $T_{mission} = T_{cruise} + T_{descent}$ subject to speed/altitude constraints and a terminal landing accuracy requirement.

---

## Mathematical Model Summary

The mission decomposes into two sequential phases:

| Phase | Control law | End condition |
|-------|-------------|---------------|
| **Cruise** | $\mathbf{v}_{cmd} = v_{max} \cdot \hat{\mathbf{d}}_{xy}$ at $z = z_{cruise}$ | $\|\mathbf{p}_{xy} - \mathbf{p}_{goal,xy}\| < \epsilon_{align}$ |
| **Descent** | $\dot{z} = -v_{z,max}$ | $z \le z_{goal} + 0.05$ m |

Analytical optimum:

$$T_1^* = \frac{d_{xy}}{v_{max}} = \frac{7.906}{5.0} \approx 1.581\ \text{s}$$

$$T_2^* = \frac{\Delta z}{v_{z,max}} = \frac{2.0}{1.5} \approx 1.333\ \text{s}$$

$$T_{mission}^* \approx 2.914\ \text{s}$$

Energy model:

$$E = \int_0^{T_{mission}} \left( P_{hover} + k_{drag} \cdot v(t)^2 \right) dt$$

---

## Key Parameters

| Parameter | Value | Notes |
|-----------|-------|-------|
| Depot position | (-4.0, 0.0, 2.0) m | Start at cruise altitude |
| Goal position | (3.5, 2.5, 0.0) m | Ground-level pad |
| Max horizontal speed | 5.0 m/s | |
| Max descent rate | 1.5 m/s | |
| Cruise altitude | 2.0 m | |
| Landing capture radius | 0.20 m | |
| Alignment threshold | 0.10 m | Switch cruise → descent |
| Control frequency | 48 Hz | dt = 1/48 s |
| Hover power | 30 W | |
| Drag power coefficient | 0.6 W·s²/m² | |
| Max mission time | 60 s | Safety cut-off (not reached) |

### Deviations from the scenario card

None in method or parameters — the code implements the card's FSM, constants and energy model exactly. Two reporting details are added: the simulated phase durations are measured from the logged switch step (not assumed equal to the analytical $T_1^*$, $T_2^*$), and energy is integrated with the trapezoidal rule over the logged power samples rather than with the card's closed-form approximation.

---

## Simulation Results

| Metric | Predicted | Simulated |
|--------|-----------|-----------|
| Cruise time $T_1$ (time of Cruise→Descent switch) | 1.581 s | **1.562 s** |
| Descent time $T_2$ | 1.333 s | **1.354 s** |
| Total mission time | 2.914 s | **2.917 s** |
| Landing error | — | **9.83 cm** < 20 cm ✅ |
| Energy consumed | — | **112.55 J** |
| Landing success | — | **Yes** ✅ |

The total mission time matches the analytical prediction to within 0.1 %. The two phases individually differ slightly, for discretisation reasons:

- **Cruise ends early (1.562 s vs 1.581 s)**: the FSM switches to descent as soon as the horizontal distance is below $\epsilon_{align} = 0.10$ m, i.e. the drone stops about 0.1 m short of the pad (≈ 0.02 s at 5 m/s).
- **Descent is slightly longer (1.354 s vs 1.333 s)**: the switch step and the touchdown step each output zero velocity (2 × 1/48 s), while the 5 cm touchdown tolerance saves one 1/48 s step (touchdown at z = 0.031 m).

The 9.83 cm landing error is therefore mostly the horizontal residual left by the 0.10 m alignment threshold (≈ 9.3 cm horizontal, 3.1 cm vertical). Energy sanity check: $30 \times 2.917 + 0.6 \times 5^2 \times 1.562 + 0.6 \times 1.5^2 \times 1.354 \approx 112.7$ J, consistent with the trapezoidal-integrated 112.55 J.

---

## Output Files

### 3D Trajectory

Depot (blue square) → cruise segment (red solid at z = 2 m) → descent segment (red dashed vertical) → landing pad (green star):

![3D Trajectory](trajectory_3d.png)

### Telemetry (Altitude / Speed / Power vs Time)

Altitude holds at 2 m during cruise then drops linearly; speed is constant at 5 m/s during cruise and 1.5 m/s during descent; power is highest during cruise due to drag (45 W vs ≈ 31.4 W in descent). The orange dashed line marks the **simulated** Cruise→Descent switch (1.562 s); the black dotted line marks the analytical $T_1^*$ (1.581 s) for comparison:

![Telemetry](telemetry.png)

### Animation

141 frames (every simulation step, 48 Hz source) played at 15 fps:

![Animation](animation.gif)

---

## Extensions

1. Add a trapezoidal velocity profile (acceleration/deceleration ramps) for more realistic kinematics
2. Compare three descent strategies: vertical drop, diagonal glide, helical spiral — plot time vs landing accuracy trade-off
3. Add wind disturbance during cruise and implement cross-track error correction
4. Extend to S022: place static obstacles along the corridor and switch to RRT* planning

---

## Related Scenarios

- Prerequisites: none (entry-level logistics scenario)
- Follow-ups:
  - [S022](../../../scenarios/02_logistics_delivery/S022_obstacle_avoidance_delivery.md) — RRT* obstacle avoidance
  - [S023](../../../scenarios/02_logistics_delivery/S023_moving_landing_pad.md) — Moving landing pad
  - [S024](../../../scenarios/02_logistics_delivery/S024_wind_compensation.md) — Wind disturbance compensation
