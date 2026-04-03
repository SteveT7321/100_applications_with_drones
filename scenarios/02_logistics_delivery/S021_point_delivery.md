# S021 Point Delivery

**Domain**: Logistics & Delivery | **Difficulty**: ⭐ | **Status**: `[ ]` Not started

---

## Problem Definition

**Setup**: A single delivery drone departs from a depot at position $\mathbf{p}_0$ and must deliver a payload to a ground target at position $\mathbf{p}_{goal}$. The flight arena is obstacle-free. The drone must reach the target, descend to a landing altitude, and touch down within a capture radius — all while minimising total flight time and energy.

**Roles**:
- **Drone**: point-mass delivery agent with maximum speed $v_{max}$ and vertical descent rate $v_z$
- **Target pad**: stationary ground marker at a fixed 3D position

**Objective**:
- Minimise total mission time $T_{mission} = T_{cruise} + T_{descent}$
- Satisfy terminal constraint: $\|\mathbf{p}(T_{mission}) - \mathbf{p}_{goal}\| < r_{land}$
- Respect speed and altitude bounds throughout the trajectory

**Constraints**:
- Drone max horizontal speed: $v_{max} = 5$ m/s
- Max vertical descent rate: $v_{z,max} = 1.5$ m/s
- Minimum safe cruise altitude: $z_{cruise} = 2.0$ m
- Landing radius: $r_{land} = 0.20$ m
- Maximum mission time: 60 s

---

## Mathematical Model

### Phase Decomposition

The mission is split into three sequential phases:

| Phase | Description | Duration |
|-------|-------------|----------|
| **Cruise** | Fly at $z_{cruise}$ from depot to a waypoint above the target | $T_1$ |
| **Descent** | Descend vertically from $z_{cruise}$ to ground | $T_2$ |
| **Land** | Touch-down check within $r_{land}$ | instant |

### Phase 1 — Cruise (Bang-Bang Time-Optimal)

The time-optimal control for a double-integrator with bounded acceleration $|a| \le a_{max}$ is bang-bang. For the simpler velocity-controlled point-mass (no inertia), the optimal strategy reduces to flying at maximum speed directly toward the horizontal waypoint:

$$\mathbf{v}_{cmd}(t) = v_{max} \cdot \hat{\mathbf{d}}_{xy}, \quad \hat{\mathbf{d}}_{xy} = \frac{\mathbf{p}_{goal,xy} - \mathbf{p}_{xy}(t)}{\|\mathbf{p}_{goal,xy} - \mathbf{p}_{xy}(t)\|}$$

Minimum cruise time (analytical):

$$T_1^* = \frac{\|\mathbf{p}_{goal,xy} - \mathbf{p}_{0,xy}\|}{v_{max}}$$

### Phase 2 — Vertical Descent

Once horizontally aligned ($\|\mathbf{p}_{xy} - \mathbf{p}_{goal,xy}\| < \epsilon_{align}$), the drone descends at maximum rate:

$$\dot{z} = -v_{z,max}$$

Minimum descent time (analytical):

$$T_2^* = \frac{z_{cruise} - z_{goal}}{v_{z,max}}$$

### Total Mission Time

$$T_{mission} = T_1^* + T_2^* = \frac{d_{xy}}{v_{max}} + \frac{\Delta z}{v_{z,max}}$$

where $d_{xy} = \|\mathbf{p}_{goal,xy} - \mathbf{p}_{0,xy}\|$ and $\Delta z = z_{cruise} - z_{goal}$.

### Energy Model

Thrust power is approximated as proportional to the square of speed plus a hover term:

$$P(v) = P_{hover} + k_{drag} \cdot v^2$$

where $P_{hover} = 30$ W and $k_{drag} = 0.6$ W·s²/m².

Total energy consumed:

$$E_{mission} = \int_0^{T_{mission}} P(v(t)) \, dt \approx P_{hover} \cdot T_{mission} + k_{drag} \cdot v_{max}^2 \cdot T_1 + k_{drag} \cdot v_{z,max}^2 \cdot T_2$$

### Position Update (Euler Integration)

At each control step $\Delta t = 1/48$ s:

$$\mathbf{p}(t + \Delta t) = \mathbf{p}(t) + \mathbf{v}_{cmd}(t) \cdot \Delta t$$

where $\mathbf{v}_{cmd}$ is clamped to respect $v_{max}$ during cruise and $v_{z,max}$ during descent.

### Landing Condition

$$\|\mathbf{p}(t) - \mathbf{p}_{goal}\| < r_{land} \quad \text{and} \quad z(t) \le z_{goal} + 0.05 \text{ m}$$

---

## Implementation

```python
# Key constants
V_MAX       = 5.0    # m/s  horizontal cruise speed
V_Z_MAX     = 1.5    # m/s  descent rate
Z_CRUISE    = 2.0    # m    cruise altitude
R_LAND      = 0.20   # m    landing capture radius
EPSILON_ALN = 0.10   # m    horizontal alignment threshold
DT          = 1/48   # s    control timestep
P_HOVER     = 30.0   # W    hover power
K_DRAG      = 0.6    # W·s²/m²

DEPOT = np.array([-4.0,  0.0, Z_CRUISE])
GOAL  = np.array([ 3.5,  2.5, 0.0])

# Phase FSM
def get_velocity_cmd(pos, goal, phase):
    if phase == "CRUISE":
        diff = goal[:2] - pos[:2]
        dist = np.linalg.norm(diff)
        if dist < EPSILON_ALN:
            return np.zeros(3), "DESCENT"
        return np.append(diff / dist * V_MAX, 0.0), "CRUISE"
    elif phase == "DESCENT":
        if pos[2] <= goal[2] + 0.05:
            return np.zeros(3), "LANDED"
        return np.array([0.0, 0.0, -V_Z_MAX]), "DESCENT"
    return np.zeros(3), "LANDED"
```

---

## Key Parameters

| Parameter | Value | Notes |
|-----------|-------|-------|
| Depot position | (-4.0, 0.0, 2.0) m | Start at cruise altitude |
| Goal position | (3.5, 2.5, 0.0) m | Ground-level pad |
| Max horizontal speed | 5.0 m/s | |
| Max descent rate | 1.5 m/s | |
| Cruise altitude | 2.0 m | |
| Landing radius | 0.20 m | |
| Alignment threshold | 0.10 m | Switch cruise → descent |
| Control frequency | 48 Hz | dt = 1/48 s |
| Hover power | 30 W | |
| Drag power coefficient | 0.6 W·s²/m² | |
| Max mission time | 60 s | |

---

## Expected Output

- **3D trajectory plot**: depot (blue square) → cruise segment (red line at $z=2$ m) → descent segment (red dashed vertical) → landing pad (green star)
- **Altitude vs time plot**: flat cruise phase, then linear descent, touchdown marked
- **Speed vs time plot**: constant cruise speed, transition to zero at landing
- **Power vs time plot**: higher power during cruise, lower during descent
- **Terminal metrics printed**: total mission time, energy consumed, final landing error
- **Animation (GIF)**: drone icon moving through all three phases

Analytical predictions:
- $d_{xy} = \sqrt{7.5^2 + 2.5^2} \approx 7.91$ m → $T_1^* \approx 1.58$ s
- $\Delta z = 2.0$ m → $T_2^* \approx 1.33$ s
- $T_{mission}^* \approx 2.91$ s

---

## Extensions

1. Add a trapezoidal velocity profile (acceleration and deceleration ramps) for more realistic drone kinematics
2. Compare three descent strategies: (a) vertical drop, (b) diagonal glide, (c) helical spiral — plot trade-off between time and landing accuracy
3. Add wind disturbance in the cruise phase and implement a cross-track error correction term
4. Extend to S022: place static obstacles along the cruise corridor and switch to RRT* planning

---

## Related Scenarios

- Prerequisites: none (entry-level logistics scenario; compare with [S001](../01_pursuit_evasion/S001_basic_intercept.md) for flight dynamics baseline)
- Follow-ups: [S022](S022_obstacle_avoidance_delivery.md) (RRT* obstacle avoidance), [S023](S023_moving_landing_pad.md) (moving target), [S024](S024_wind_compensation.md) (wind disturbance)
