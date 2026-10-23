# S025 Payload CoG Offset

**Domain**: Logistics & Delivery | **Difficulty**: ⭐⭐ | **Status**: `[x]` Completed

→ **[Full details, results, and visualizations](../../outputs/02_logistics_delivery/s025_payload_cog_offset/)**

---

## Problem Definition

**Setup**: A delivery drone (airframe 1.5 kg) carries a 0.8 kg payload whose centre of gravity is not under the airframe centre but offset by $\mathbf{r} = (r_x, r_y) = (+0.08,\ +0.12)$ m. The payload weight acting at this offset produces constant roll and pitch disturbance torques. The drone starts at $(0, 0, 2)$ m and must capture three waypoints — $(4, 0, 2)$, $(4, 4, 2)$ and the delivery point $(4, 4, 0.3)$ m — each within a capture radius of 0.15 m, in at most 20 s.

**Roles**:
- **Drone**: cascaded controller — PD position loop that produces thrust and roll/pitch references, and an attitude loop that produces torques
- **Payload**: rigid, fixed offset mass producing a constant disturbance torque

**Objective**: compare three attitude-control strategies under the same position loop:
1. **No compensation** — PD attitude control, offset ignored
2. **Static feed-forward** — PD attitude control with the reference trimmed using perfect knowledge of the offset torque
3. **PID (integral)** — PD + integral attitude control that learns the offset torque without a model

Metrics: waypoints captured (out of 3; the start is not counted), mission time, lateral deviation on the first leg, steady-state attitude-tracking error.

---

## Mathematical Model

### Disturbance Torque

Sign convention of the model: +roll tilts thrust toward +y, +pitch tilts it toward −x ($a_y = g\phi$, $a_x = -g\theta$ for small angles). The payload side dips:

$$\tau_{roll} = +m_p g\, r_y = 0.8 \times 9.81 \times 0.12 = 0.942\ \text{N·m}$$

$$\tau_{pitch} = -m_p g\, r_x = -0.8 \times 9.81 \times 0.08 = -0.628\ \text{N·m}$$

### Rigid-Body Model (linearised about hover)

$$I_{xx}\dot p = \tau_{cmd,x} + \tau_{roll}, \quad I_{yy}\dot q = \tau_{cmd,y} + \tau_{pitch}, \quad \dot\phi = p,\ \dot\theta = q$$

$$a_x = -\frac{T\sin\theta}{m}, \quad a_y = \frac{T\sin\phi}{m}, \quad a_z = \frac{T\cos\phi\cos\theta}{m} - g$$

with Euler integration at 200 Hz, $|\omega| \le 10$ rad/s, $|\phi|, |\theta| \le 25°$, $\|\mathbf{v}\| \le 8$ m/s.

### Position Loop (all strategies)

$$\mathbf{a}_d = K_{p,pos}(\mathbf{p}_{wp} - \mathbf{p}) - K_{d,pos}\mathbf{v}, \quad T = m(g + a_{d,z}), \quad \phi_{ref} = \frac{a_{d,y}\, m}{T}, \quad \theta_{ref} = -\frac{a_{d,x}\, m}{T}$$

(references clipped to ±25°). The waypoint index advances when $\|\mathbf{p} - \mathbf{p}_{wp}\| < 0.15$ m.

### Attitude Controllers

With $\mathbf{e} = \boldsymbol\eta_{ref} - \boldsymbol\eta$ and torque saturation ±5 N·m:

| Strategy | Torque command |
|----------|----------------|
| No compensation | $\boldsymbol\tau = K_P\mathbf{e} - K_D\boldsymbol\omega$ |
| Static feed-forward | $\boldsymbol\tau = K_P(\mathbf{e} + \boldsymbol\eta_{trim}) - K_D\boldsymbol\omega$, $\ \boldsymbol\eta_{trim} = -\boldsymbol\tau_{dist}/K_P$ |
| PID (integral) | $\boldsymbol\tau = K_P\mathbf{e} - K_D\boldsymbol\omega + K_I\int\mathbf{e}\,dt$, integral clamped to ±0.5 rad·s |

The feed-forward trim enters through the $K_P$ term, so it is equivalent to a feed-forward torque $\boldsymbol\tau_{ff} = K_P\boldsymbol\eta_{trim} = -\boldsymbol\tau_{dist}$. Note the units: $\tau_{cmd}$ is a torque, so the trim angle is $-\tau/K_P$ (not $-\tau/(I K_P)$).

### Analytical Predictions

PD steady-state attitude-tracking error:

$$K_P(\eta_{ref} - \eta) + \tau = 0 \ \Rightarrow\ \eta - \eta_{ref} = \frac{\tau}{K_P}: \quad \phi - \phi_{ref} = \frac{0.942}{8} = 0.118\ \text{rad} = 6.74°, \quad \theta - \theta_{ref} = -\frac{0.628}{8} = -0.0785\ \text{rad} = -4.50°$$

At a hover the actual tilt is zero, so the uncompensated position loop must hold $\eta_{ref} = -\tau/K_P$, i.e. a standing position error

$$\Delta y = \frac{g\,(\tau_{roll}/K_P)}{K_{p,pos}} = \frac{9.81 \times 0.118}{1.5} = 0.770\ \text{m}, \qquad \Delta x = -\frac{g\,(\tau_{pitch}/K_P)}{K_{p,pos}} = 0.513\ \text{m}$$

which exceeds the 0.15 m capture radius: without compensation no waypoint can be captured. The integral term can supply at most $K_I \times 0.5 = 1.0$ N·m, enough for the 0.942 N·m roll torque.

---

## Implementation

```
src/02_logistics_delivery/s025_payload_cog_offset.py
```

```python
DRONE_MASS, PAYLOAD_MASS = 1.5, 0.8          # kg
Ixx = Iyy = 0.015; Izz = 0.025               # kg·m²
COG_X, COG_Y = +0.08, +0.12                  # m
TAU_DIST_ROLL  =  PAYLOAD_MASS * G * COG_Y   # +0.942 N·m
TAU_DIST_PITCH = -PAYLOAD_MASS * G * COG_X   # -0.628 N·m
DT = 1 / 200; MAX_TIME = 20.0; MAX_TILT = np.deg2rad(25)
KP_POS, KD_POS = 1.5, 1.8
KP_ATT, KD_ATT, KI_ATT, I_LIMIT = 8.0, 3.0, 2.0, 0.5

trim_roll  = -TAU_DIST_ROLL  / KP_ATT        # feed-forward reference trim [rad]
trim_pitch = -TAU_DIST_PITCH / KP_ATT

def compute_attitude_command(drone, euler_ref, dt, use_integral):
    err = euler_ref - drone.euler
    tau = KP_ATT * err - KD_ATT * drone.omega
    if use_integral:
        drone.integral_att = np.clip(drone.integral_att + err * dt, -I_LIMIT, I_LIMIT)
        tau += KI_ATT * drone.integral_att
    return np.clip(tau, -5.0, 5.0)
```

```bash
conda activate drones
python src/02_logistics_delivery/s025_payload_cog_offset.py
```

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Airframe / payload / total mass | 1.5 / 0.8 / 2.3 kg |
| Inertia $I_{xx}, I_{yy}, I_{zz}$ | 0.015, 0.015, 0.025 kg·m² |
| CoG offset $(r_x, r_y)$ | (+0.08, +0.12) m |
| Disturbance torque roll / pitch | +0.942 / −0.628 N·m |
| Position gains $K_{p,pos}, K_{d,pos}$ | 1.5, 1.8 |
| Attitude gains $K_P, K_D, K_I$ | 8.0, 3.0, 2.0 |
| Integral clamp | ±0.5 rad·s |
| Torque / rate / speed / tilt limits | ±5 N·m / ±10 rad/s / 8 m/s / ±25° |
| Waypoints | start (0,0,2) → (4,0,2) → (4,4,2) → (4,4,0.3) m |
| Capture radius | 0.15 m |
| Control rate | 200 Hz |
| Time limit | 20 s |

---

## Expected Output

- **3D trajectory plot**: three paths (no compensation, feed-forward, PID) with start and waypoints
- **Attitude & position plot**: roll, pitch, x, y, z and distance to the final waypoint vs time
- **Attitude-tracking-error plot**: $\eta - \eta_{ref}$ for roll and pitch with the PD steady-state value $\tau/K_P$
- **Animation (GIF)**: top-down view of all three runs
- **Printed metrics**: waypoints captured (out of 3), capture times, final position, steady tracking error, leg-1 lateral deviation; predicted vs simulated no-compensation hover offset

---

## Extensions

1. Payload mass uncertainty (±20 %): the feed-forward trim becomes wrong — compare FF + integral vs integral alone
2. Mid-flight payload shift (sudden CoG change): measure recovery time of each strategy
3. Adaptive / disturbance-observer compensation that estimates $\tau_{dist}$ online
4. Extend to S026: several drones sharing a suspended load with a moving CoG

---

## Related Scenarios

- Prerequisites: [S021 Point Delivery](S021_point_delivery.md) — basic delivery
- Follow-ups: [S026 Cooperative Heavy Lift](S026_cooperative_heavy_lift.md) — multi-drone suspended load with cable tension
