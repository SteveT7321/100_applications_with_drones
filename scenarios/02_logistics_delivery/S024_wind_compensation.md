# S024 Wind Compensation

**Domain**: Logistics & Delivery | **Difficulty**: ⭐⭐ | **Status**: `[x]` Completed

---

## Problem Definition

**Setup**: A delivery drone must fly from a fixed origin $\mathbf{p}_0$ to a target waypoint $\mathbf{p}_{goal}$ (e.g., a rooftop landing pad) in the presence of a persistent crosswind field. The wind is modeled as a spatially uniform, time-varying horizontal vector $\mathbf{w}(t) \in \mathbb{R}^2$ whose mean direction and magnitude are known approximately but include a stochastic gust component.

**Roles**:
- **Drone (controlled agent)**: a multirotor with first-order velocity dynamics; selects its airspeed command at each step
- **Wind field**: exogenous disturbance acting on the drone's ground velocity

**Objective**: reach $\mathbf{p}_{goal}$ within a terminal radius $r_{land} = 0.3$ m while minimizing lateral drift (cross-track error) and total flight time; compare three guidance strategies:

1. **No compensation** — fly the nominal bearing, ignore wind
2. **Feed-forward crab-angle correction** — analytically solve for the heading offset that nullifies mean crosswind drift
3. **PID cross-track correction** — closed-loop lateral error feedback on top of the nominal path

---

## Mathematical Model

### Ground Velocity Kinematics

The drone's ground velocity is the vector sum of its commanded airspeed vector and the wind:

$$\dot{\mathbf{p}} = \mathbf{v}_{air} + \mathbf{w}(t)$$

with the airspeed magnitude bounded by $\|\mathbf{v}_{air}\| \leq v_{max}$.

### Wind Model

$$\mathbf{w}(t) = \bar{\mathbf{w}} + \boldsymbol{\eta}(t), \quad \boldsymbol{\eta}(t) \sim \mathcal{N}(\mathbf{0},\, \sigma_g^2 \mathbf{I})$$

Mean wind: $\bar{\mathbf{w}} = w_0 \hat{\mathbf{e}}_y$ (pure crosswind). Gusts $\boldsymbol{\eta}(t)$ are sampled at every control step and low-pass filtered with time constant $\tau_g = 1.5$ s to produce correlated turbulence:

$$\mathbf{w}_{g,k+1} = e^{-\Delta t / \tau_g}\,\mathbf{w}_{g,k} + \sqrt{1 - e^{-2\Delta t/\tau_g}}\,\sigma_g\,\boldsymbol{\varepsilon}_k, \quad \boldsymbol{\varepsilon}_k \sim \mathcal{N}(\mathbf{0}, \mathbf{I})$$

### Strategy 1 — No Compensation

Airspeed direction points directly at the goal:

$$\mathbf{v}_{air} = v_{max} \cdot \hat{\mathbf{d}}, \quad \hat{\mathbf{d}} = \frac{\mathbf{p}_{goal} - \mathbf{p}}{\|\mathbf{p}_{goal} - \mathbf{p}\|}$$

Resulting ground track drifts downwind by the accumulated crosswind displacement.

### Strategy 2 — Feed-Forward Crab Angle

Solve analytically for the heading $\psi$ such that the lateral component of ground velocity is zero. For a straight flight along nominal bearing $\psi_0$ (the $x$-axis) and crosswind $w_y$:

$$\sin\alpha = -\frac{\bar{w}_y}{v_{max}}, \quad \alpha = \arcsin\!\left(-\frac{\bar{w}_y}{v_{max}}\right)$$

The corrected airspeed command rotates the nominal bearing by the crab angle $\alpha$:

$$\mathbf{v}_{air} = v_{max}\begin{bmatrix}\cos(\psi_0 + \alpha) \\ \sin(\psi_0 + \alpha)\end{bmatrix}$$

This yields a ground speed along track of $v_g = \sqrt{v_{max}^2 - \bar{w}_y^2}$ with zero mean lateral drift. Gust residuals cause residual cross-track error.

### Strategy 3 — PID Cross-Track Controller

Define the cross-track error $e_{ct}$ as the signed perpendicular distance from the drone's current position to the nominal straight-line path:

$$e_{ct} = (\mathbf{p} - \mathbf{p}_0) \times \hat{\mathbf{d}}_0$$

where $\hat{\mathbf{d}}_0 = (\mathbf{p}_{goal} - \mathbf{p}_0)/\|\mathbf{p}_{goal} - \mathbf{p}_0\|$ and $\times$ denotes the 2D cross product (scalar).

The lateral velocity correction is:

$$v_\perp = k_p\, e_{ct} + k_i \int_0^t e_{ct}\,d\tau + k_d\, \dot{e}_{ct}$$

The full airspeed command blends the along-track nominal direction with the lateral correction:

$$\mathbf{v}_{air} = \mathrm{clip}\!\left(v_{max}\,\hat{\mathbf{d}} + v_\perp\,\hat{\mathbf{n}},\; v_{max}\right)$$

where $\hat{\mathbf{n}}$ is the unit normal to the nominal path (positive = left of track) and $\mathrm{clip}$ rescales to $\|\cdot\| \leq v_{max}$.

### Altitude Hold

Altitude is decoupled and held constant at $z = 2.5$ m via a proportional controller:

$$\dot{z}_{cmd} = k_z (z_{ref} - z)$$

---

## Implementation

```python
# Key constants
V_MAX       = 5.0    # m/s, maximum airspeed
W_MEAN      = 2.5    # m/s, mean crosswind magnitude (y-direction)
SIGMA_GUST  = 0.8    # m/s, gust standard deviation
TAU_GUST    = 1.5    # s, gust correlation time constant
DT          = 1/48   # s, simulation timestep (48 Hz)
Z_REF       = 2.5    # m, cruise altitude
R_LAND      = 0.3    # m, arrival radius
P0          = np.array([0.0, 0.0, Z_REF])
P_GOAL      = np.array([20.0, 0.0, Z_REF])   # 20 m straight ahead


def wind_model_step(w_gust, dt, tau_g, sigma_g, w_mean_vec):
    """First-order Gauss-Markov gust + mean wind."""
    alpha = np.exp(-dt / tau_g)
    noise = np.sqrt(1 - alpha**2) * sigma_g * np.random.randn(2)
    w_gust_new = alpha * w_gust + noise
    return w_gust_new, w_mean_vec + w_gust_new


def crab_angle(v_max, w_mean_y, bearing):
    """Compute feed-forward crab angle correction."""
    sin_alpha = -w_mean_y / v_max
    sin_alpha = np.clip(sin_alpha, -1.0, 1.0)
    alpha = np.arcsin(sin_alpha)
    return bearing + alpha


class CrossTrackPID:
    def __init__(self, kp, ki, kd, dt):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.dt = dt
        self.integral = 0.0
        self.prev_err = 0.0

    def step(self, e_ct):
        self.integral += e_ct * self.dt
        deriv = (e_ct - self.prev_err) / self.dt
        self.prev_err = e_ct
        return self.kp * e_ct + self.ki * self.integral + self.kd * deriv


def cross_track_error(pos, p0, d_hat):
    """Signed perpendicular distance to nominal path."""
    rel = pos[:2] - p0[:2]
    # 2D cross product: rel x d_hat
    return rel[0] * d_hat[1] - rel[1] * d_hat[0]


def run_strategy(strategy, seed=42):
    np.random.seed(seed)
    pos = P0.copy().astype(float)
    w_gust = np.zeros(2)
    pid = CrossTrackPID(kp=1.2, ki=0.05, kd=0.3, dt=DT)
    traj = [pos.copy()]
    d_hat0 = (P_GOAL - P0)[:2]; d_hat0 /= np.linalg.norm(d_hat0)
    n_hat0 = np.array([-d_hat0[1], d_hat0[0]])  # left-of-track normal

    while np.linalg.norm(pos[:2] - P_GOAL[:2]) > R_LAND:
        w_gust, w_total = wind_model_step(w_gust, DT, TAU_GUST, SIGMA_GUST,
                                          np.array([0.0, W_MEAN]))
        d = P_GOAL[:2] - pos[:2]
        bearing = np.arctan2(d[1], d[0])

        if strategy == "none":
            heading = bearing
        elif strategy == "crab":
            heading = crab_angle(V_MAX, W_MEAN, bearing)
        # PID handled below

        if strategy in ("none", "crab"):
            v_air_2d = V_MAX * np.array([np.cos(heading), np.sin(heading)])
        else:  # PID
            e_ct = cross_track_error(pos, P0, d_hat0)
            v_perp = pid.step(e_ct)
            v_air_2d = V_MAX * d_hat0 + v_perp * n_hat0
            norm = np.linalg.norm(v_air_2d)
            if norm > V_MAX:
                v_air_2d *= V_MAX / norm

        v_ground = v_air_2d + w_total
        pos[:2] += v_ground * DT
        pos[2]  += 2.0 * (Z_REF - pos[2]) * DT   # altitude hold (kz=2.0)
        traj.append(pos.copy())

    return np.array(traj)
```

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Cruise speed $v_{max}$ | 5.0 m/s |
| Mean crosswind $w_0$ | 2.5 m/s |
| Gust std $\sigma_g$ | 0.8 m/s |
| Gust correlation time $\tau_g$ | 1.5 s |
| Control frequency | 48 Hz |
| Cruise altitude $z_{ref}$ | 2.5 m |
| Delivery distance | 20.0 m |
| Arrival radius $r_{land}$ | 0.3 m |
| PID gains $(k_p, k_i, k_d)$ | 1.2, 0.05, 0.3 |

---

## Expected Output

- **3D trajectory plot**: three colored paths (no-comp gray, crab-angle orange, PID blue) with wind arrows overlaid on the $xy$-plane
- **Cross-track error vs time**: time series for all three strategies showing drift magnitude and PID convergence
- **Lateral displacement at arrival**: bar chart comparing final miss distance for each strategy
- **Ground speed time series**: illustrating speed loss due to headwind component and gust variation
- Crab-angle strategy reduces mean lateral error to near-zero; PID additionally suppresses gust residuals to $< 0.15$ m RMS

---

## Extensions

1. Wind estimation with an online least-squares wind observer and adaptive feed-forward update
2. 2D wind-field with spatial variation (Dryden turbulence model); replan path if wind exceeds $v_{max}$
3. Energy-optimal cruise speed selection accounting for headwind/tailwind components
4. Combine with [S022](S022_obstacle_avoidance_delivery.md) to handle wind + obstacles simultaneously

---

## Related Scenarios

- Prerequisites: [S021](S021_point_delivery.md), [S022](S022_obstacle_avoidance_delivery.md)
- Follow-ups: [S025](S025_payload_cog_offset.md), [S034](S034_weather_rerouting.md)
