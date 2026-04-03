# S023 Moving Landing Pad

**Domain**: Logistics & Delivery | **Difficulty**: ⭐⭐⭐ | **Status**: `[x]` Completed

---

## Problem Definition

**Setup**: A delivery drone must land on a moving ground vehicle (simulated truck) that travels along a straight road at constant speed. The drone starts at altitude $z_0$ above a fixed base station offset from the road. The truck's position is known via telemetry (perfect state knowledge). The drone must intercept the truck's future position, descend, and achieve a soft landing — touchdown velocity within limits — on a pad of radius $r_{pad}$.

**Roles**:
- **Drone**: quadrotor executing a two-phase guidance law — intercept phase (close lateral gap) followed by descent phase (reduce altitude and match truck velocity)
- **Truck**: ground vehicle moving at constant speed $v_T$ along the $x$-axis; represents the moving landing platform

**Objective**: guide the drone to touchdown on the pad with:
1. Lateral miss distance $\|\mathbf{p}_{xy,D} - \mathbf{p}_{xy,T}\| \leq r_{pad}$
2. Vertical touchdown speed $|\dot{z}| \leq v_{land,max}$
3. Horizontal velocity mismatch $\|\dot{\mathbf{p}}_{xy,D} - \dot{\mathbf{p}}_{xy,T}\| \leq \Delta v_{max}$

---

## Mathematical Model

### State Variables

Drone state: $\mathbf{x}_D = [\mathbf{p}_D,\ \mathbf{v}_D]^T \in \mathbb{R}^6$, where $\mathbf{p}_D = [x_D,\ y_D,\ z_D]^T$ and $\mathbf{v}_D = [\dot x_D,\ \dot y_D,\ \dot z_D]^T$.

Truck state: $\mathbf{p}_T(t) = [x_{T0} + v_T t,\ 0,\ 0]^T$ (road along $x$-axis at $y=0$, $z=0$).

### Intercept Point Prediction

At time $t$, the drone predicts the truck's position at a future intercept time $t^* = t + \Delta t_{fly}$:

$$\mathbf{p}_T^*(t) = \mathbf{p}_T(t) + v_T \cdot \Delta t_{fly} \cdot \hat{\mathbf{x}}$$

Time-to-go estimate (iterative):

$$\Delta t_{fly} = \frac{\|\mathbf{p}_T^* - \mathbf{p}_D\|}{v_{D,max}}$$

Converged after 3–5 fixed-point iterations.

### Phase 1 — Intercept Guidance (Proportional Navigation)

The drone applies a 3D Proportional Navigation law to close on the predicted intercept point $\mathbf{p}_T^*$:

$$\dot{\mathbf{v}}_D = N \cdot V_c \cdot \dot{\hat{\boldsymbol{\lambda}}}$$

where:
- $N = 3$ — navigation constant
- $V_c = -\frac{d}{dt}\|\mathbf{p}_T^* - \mathbf{p}_D\|$ — closing speed
- $\hat{\boldsymbol{\lambda}} = \frac{\mathbf{p}_T^* - \mathbf{p}_D}{\|\mathbf{p}_T^* - \mathbf{p}_D\|}$ — line-of-sight unit vector
- $\dot{\hat{\boldsymbol{\lambda}}} \approx \frac{\hat{\boldsymbol{\lambda}}(t) - \hat{\boldsymbol{\lambda}}(t-dt)}{dt}$ — LOS rate (finite difference)

Acceleration is saturated: $\|\dot{\mathbf{v}}_D\| \leq a_{max}$.

### Phase 2 — Descent and Velocity Matching

Once the drone is within a cone of radius $r_{switch}$ horizontally and $z_D \leq z_{switch}$, control transitions to a PD law that simultaneously:

1. **Matches truck horizontal velocity**:

$$a_{xy} = K_v \left( \mathbf{v}_{T,xy} - \mathbf{v}_{D,xy} \right) + K_p \left( \mathbf{p}_{T,xy} - \mathbf{p}_{D,xy} \right)$$

2. **Controls descent rate**:

$$a_z = -K_z \cdot z_D - K_{\dot z} \cdot \dot z_D - g + g$$

simplified to a commanded descent rate profile:

$$\dot z_{cmd}(z) = -v_{land,max} \cdot \tanh\!\left(\frac{z}{z_{flare}}\right)$$

The $\tanh$ flare ensures $\dot z \to 0$ smoothly as $z \to 0$ (flare maneuver).

### Drone Kinematics (Double-Integrator)

$$\dot{\mathbf{p}}_D = \mathbf{v}_D, \qquad \dot{\mathbf{v}}_D = \mathbf{a}_{cmd} - g\hat{\mathbf{z}} + g\hat{\mathbf{z}} = \mathbf{a}_{cmd}$$

(gravity cancelled by rotor thrust; net acceleration equals commanded acceleration):

$$\mathbf{a}_{cmd} = \text{clip}\!\left(\mathbf{a}_{ctrl},\ -a_{max},\ +a_{max}\right)$$

### Phase Transition Logic

$$\text{Phase} = \begin{cases} 1 & \text{if } z_D > z_{switch} \text{ or } \|\mathbf{p}_{xy,D} - \mathbf{p}_{xy,T}\| > r_{switch} \\ 2 & \text{otherwise} \end{cases}$$

### Landing Success Criterion

Touchdown is declared when $z_D \leq z_{touchdown} = 0.05$ m. Success requires:

$$\|\mathbf{p}_{xy,D} - \mathbf{p}_{xy,T}\| \leq r_{pad} \quad \text{and} \quad |\dot z_D| \leq v_{land,max} \quad \text{and} \quad \|\dot{\mathbf{p}}_{xy,D} - \dot{\mathbf{p}}_{xy,T}\| \leq \Delta v_{max}$$

---

## Implementation

```python
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Key constants
DT             = 0.02       # time step (s), 50 Hz
SIM_TIME       = 30.0       # total simulation time (s)
N_STEPS        = int(SIM_TIME / DT)

DRONE_VMAX     = 8.0        # m/s
DRONE_AMAX     = 5.0        # m/s^2
TRUCK_SPEED    = 4.0        # m/s (along x)

N_PNG          = 3          # PNG navigation constant
Z0             = 15.0       # initial drone altitude (m)
Z_SWITCH       = 5.0        # altitude to enter descent phase (m)
R_SWITCH       = 3.0        # lateral radius to enter descent phase (m)

R_PAD          = 0.5        # landing pad radius (m)
V_LAND_MAX     = 1.0        # max touchdown vertical speed (m/s)
DV_MAX         = 0.5        # max horizontal velocity mismatch at touchdown (m/s)
Z_FLARE        = 2.0        # flare start altitude (m)
Z_TOUCHDOWN    = 0.05       # touchdown threshold (m)

# PD gains (Phase 2)
KP             = 1.2
KV             = 1.8
KZ_RATE        = 2.0        # descent rate controller gain


def predict_intercept(p_drone, p_truck, v_truck_vec, v_drone_max, n_iter=5):
    """Iterative intercept point prediction."""
    dt_fly = np.linalg.norm(p_truck - p_drone) / v_drone_max
    for _ in range(n_iter):
        p_intercept = p_truck + v_truck_vec * dt_fly
        dt_fly = np.linalg.norm(p_intercept - p_drone) / v_drone_max
    return p_intercept


def png_acceleration(p_drone, v_drone, p_target, v_target, N, a_max, dt):
    """3D Proportional Navigation guidance acceleration."""
    r = p_target - p_drone
    r_norm = np.linalg.norm(r) + 1e-9
    los = r / r_norm

    # Closing speed
    v_rel = v_target - v_drone
    vc = -np.dot(v_rel, los)    # positive when closing

    # LOS rate via relative velocity component perpendicular to LOS
    v_perp = v_rel - np.dot(v_rel, los) * los
    los_rate = v_perp / (r_norm + 1e-9)

    a = N * vc * los_rate
    a_norm = np.linalg.norm(a)
    if a_norm > a_max:
        a = a * (a_max / a_norm)
    return a


def descent_acceleration(p_drone, v_drone, p_truck, v_truck):
    """Phase 2: match truck velocity + flare descent."""
    # Lateral: PD to match truck position and velocity
    dp_xy = p_truck[:2] - p_drone[:2]
    dv_xy = v_truck[:2] - v_drone[:2]
    a_xy = KP * dp_xy + KV * dv_xy

    # Vertical: tanh flare profile
    z = p_drone[2]
    z_dot_cmd = -V_LAND_MAX * np.tanh(z / Z_FLARE)
    a_z = KZ_RATE * (z_dot_cmd - v_drone[2])

    return np.array([a_xy[0], a_xy[1], a_z])


def run_simulation():
    # Initial conditions
    p_drone = np.array([-20.0, 8.0, Z0])   # drone starts offset from road
    v_drone = np.zeros(3)
    p_truck = np.array([0.0, 0.0, 0.0])
    v_truck = np.array([TRUCK_SPEED, 0.0, 0.0])

    los_prev = None
    history = {"drone": [], "truck": [], "phase": []}
    landed = False

    for step in range(N_STEPS):
        t = step * DT

        # Update truck position
        p_truck = np.array([0.0 + TRUCK_SPEED * t, 0.0, 0.0])

        # Determine phase
        lat_dist = np.linalg.norm(p_drone[:2] - p_truck[:2])
        phase = 2 if (p_drone[2] <= Z_SWITCH and lat_dist <= R_SWITCH) else 1

        if phase == 1:
            p_intercept = predict_intercept(p_drone, p_truck, v_truck, DRONE_VMAX)
            a_cmd = png_acceleration(p_drone, v_drone, p_intercept,
                                     v_truck, N_PNG, DRONE_AMAX, DT)
        else:
            a_cmd = descent_acceleration(p_drone, v_drone, p_truck, v_truck)
            a_norm = np.linalg.norm(a_cmd)
            if a_norm > DRONE_AMAX:
                a_cmd = a_cmd * (DRONE_AMAX / a_norm)

        # Integrate
        v_drone = v_drone + a_cmd * DT
        speed = np.linalg.norm(v_drone)
        if speed > DRONE_VMAX:
            v_drone = v_drone * (DRONE_VMAX / speed)
        p_drone = p_drone + v_drone * DT
        p_drone[2] = max(p_drone[2], 0.0)   # ground clamp

        history["drone"].append(p_drone.copy())
        history["truck"].append(p_truck.copy())
        history["phase"].append(phase)

        # Check touchdown
        if p_drone[2] <= Z_TOUCHDOWN:
            miss = np.linalg.norm(p_drone[:2] - p_truck[:2])
            vz   = abs(v_drone[2])
            dv   = np.linalg.norm(v_drone[:2] - v_truck[:2])
            success = (miss <= R_PAD) and (vz <= V_LAND_MAX) and (dv <= DV_MAX)
            print(f"Touchdown at t={t:.2f}s | miss={miss:.3f}m | vz={vz:.3f}m/s | dv={dv:.3f}m/s | {'SUCCESS' if success else 'FAIL'}")
            landed = True
            break

    return history, landed
```

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Drone max speed | 8.0 m/s |
| Drone max acceleration | 5.0 m/s² |
| Truck speed | 4.0 m/s |
| Initial drone altitude | 15.0 m |
| PNG navigation constant $N$ | 3 |
| Phase-switch altitude $z_{switch}$ | 5.0 m |
| Phase-switch lateral radius $r_{switch}$ | 3.0 m |
| Landing pad radius $r_{pad}$ | 0.5 m |
| Max touchdown vertical speed | 1.0 m/s |
| Max horizontal velocity mismatch | 0.5 m/s |
| Flare altitude $z_{flare}$ | 2.0 m |
| Simulation time step | 0.02 s (50 Hz) |

---

## Expected Output

- **3D trajectory plot**: drone intercept arc in red, truck road path in orange, pad circle in green; phase-switch point marked
- **Altitude time series**: $z_D(t)$ showing intercept phase plateau, then smooth $\tanh$ flare descent
- **Lateral miss distance time series**: convergence to within $r_{pad}$ before touchdown
- **Velocity mismatch time series**: $\|\dot{\mathbf{p}}_{xy,D} - \dot{\mathbf{p}}_{xy,T}\|$ decreasing to near-zero during Phase 2
- **Phase indicator**: horizontal bar on time axis showing Phase 1 / Phase 2 transition
- **Printed touchdown report**: miss distance, vertical speed, horizontal mismatch, success/fail verdict

---

## Extensions

1. **Variable truck speed**: truck accelerates or brakes during approach — re-solve intercept prediction with $v_T(t)$ lookup
2. **Curved road**: truck follows a circular arc; LOS prediction must integrate along arc
3. **Wind disturbance**: add crosswind vector $\mathbf{w}$ to drone kinematics; test robustness of PNG gain
4. **Multiple trucks**: dispatch drone to whichever truck minimizes time-to-land (greedy assignment)
5. **Offshore variant** (S039 preview): add sinusoidal pitch/roll to the pad so the pad itself oscillates in $z$ and $y$

---

## Related Scenarios

- Prerequisites: [S021](S021_point_delivery.md) (basic A-to-B landing), [S022](S022_obstacle_avoidance_delivery.md) (path planning)
- Follow-ups: [S024](S024_wind_compensation.md) (wind disturbance), [S039](S039_offshore_platform_exchange.md) (moving pad with vessel pitch/roll)
