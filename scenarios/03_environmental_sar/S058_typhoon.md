# S058 Typhoon Eye Probing

**Domain**: Environmental Monitoring & SAR | **Difficulty**: ⭐⭐⭐⭐ | **Status**: `[ ]` Not Started

---

## Problem Definition

**Setup**: A research drone is tasked with flying from the outer periphery of a tropical typhoon to
its calm eye, collecting continuous meteorological profiles (pressure, humidity, wind speed) along
the way. The typhoon occupies a $400 \times 400$ m simulation domain centred on the eye $\mathbf{c}_{eye}
= (0, 0)$ m. The wind field is an axisymmetric Rankine vortex with a maximum wind radius of
$r_{max} = 150$ m; peak tangential wind speed reaches $v_{max} = 50$ m/s. The drone begins at
$r_{start} = 190$ m from the eye and must reach the eye region ($r < r_{eye} = 15$ m) while
following a prescribed inbound spiral reference trajectory. The outer bands are turbulent but
moderate; the eyewall ($r \approx r_{max}$) is the most dangerous passage, with combined
tangential and radial gust components pushing the drone off course.

**Roles**:
- **Drone**: single research platform with mass $m = 1.5$ kg, maximum thrust $F_{max} = 30$ N,
  roll/pitch attitude limits of $\pm 45°$; carries an onboard MPC controller with prediction horizon
  $N_p = 20$ steps.
- **Typhoon wind field**: a deterministic Rankine vortex base flow plus a stochastic turbulence
  overlay modelled as band-limited Gaussian noise; the wind field is treated as an external
  disturbance to the drone dynamics.

**Objective**: Drive the drone from the entry point to the eye along the spiral reference path,
minimising cumulative trajectory tracking error while satisfying actuator and attitude constraints
at all times. Simultaneously, quantify the disturbance rejection performance of three controllers
under increasingly severe wind conditions.

**Comparison strategies**:
1. **PD baseline** — proportional-derivative controller that tracks the reference position with no
   explicit disturbance model; wind acts as an uncompensated external force.
2. **MPC with disturbance feed-forward** — Model Predictive Control that uses the known Rankine
   wind model as a measured disturbance input, optimising thrust over a receding horizon.
3. **H-infinity robust controller** — minimises the worst-case $\mathcal{H}_\infty$ gain from wind
   disturbance $w$ to tracking error $z$, providing a certified $\gamma$ bound on disturbance
   amplification.

---

## Mathematical Model

### Rankine Vortex Wind Field

The typhoon wind field at polar coordinates $(r, \theta)$ from the eye centre is decomposed into
tangential and radial components. The tangential (azimuthal) velocity profile is:

$$v_{\tan}(r) = \begin{cases}
  v_{max} \cdot \dfrac{r}{r_{max}} & r \leq r_{max} \quad \text{(inner vortex, solid-body rotation)} \\[6pt]
  v_{max} \cdot \dfrac{r_{max}}{r} & r > r_{max}  \quad \text{(outer vortex, potential flow)}
\end{cases}$$

A radial inflow component (convergence towards the eye) models the secondary circulation:

$$v_{rad}(r) = -v_{inflow} \cdot \exp\!\left(-\frac{(r - r_{max})^2}{2\sigma_{rad}^2}\right)$$

where $v_{inflow} = 8$ m/s is the peak radial inflow speed and $\sigma_{rad} = 40$ m is the radial
width of the inflow layer. The negative sign denotes inward flow.

The full wind vector at Cartesian position $\mathbf{p} = (x, y)$ is:

$$\mathbf{f}_{wind}(\mathbf{p}) = v_{\tan}(r)\,\hat{\mathbf{e}}_\theta + v_{rad}(r)\,\hat{\mathbf{e}}_r$$

where the unit vectors in polar form are:

$$\hat{\mathbf{e}}_r = \frac{\mathbf{p}}{r}, \qquad
  \hat{\mathbf{e}}_\theta = \frac{1}{r}\begin{pmatrix} -y \\ x \end{pmatrix}, \qquad
  r = \|\mathbf{p}\|$$

A stochastic turbulence overlay adds band-limited noise with spectral density $S_w(f)$ scaled to
10% of the local $v_{\tan}(r)$:

$$\mathbf{f}_{turb}(t) \sim \mathcal{N}\!\left(\mathbf{0},\; \sigma_w^2(r)\,\mathbf{I}_2\right),
  \qquad \sigma_w(r) = 0.1 \cdot v_{\tan}(r)$$

### Drone Dynamics

The drone is modelled as a 2D point mass (the x-y plane of the spiral trajectory) under thrust
control $\mathbf{u} = (u_x, u_y)$ (horizontal thrust components in N) and the wind disturbance:

$$m\ddot{\mathbf{p}} = \mathbf{u} + m\,\mathbf{f}_{wind}(\mathbf{p}) + \mathbf{f}_{turb}(t) - \mathbf{b}\,\dot{\mathbf{p}}$$

where $\mathbf{b} = b\,\mathbf{I}_2$ is an aerodynamic drag coefficient ($ b = 0.4$ N·s/m).
Attitude limits translate to a thrust cone constraint:

$$\|\mathbf{u}\| \leq F_{max}, \qquad
  \tan(\phi) = \frac{\|\mathbf{u}\|}{mg} \leq \tan(45°) = 1 \implies \|\mathbf{u}\| \leq mg$$

The combined saturation constraint is therefore $\|\mathbf{u}\| \leq \min(F_{max}, mg) = mg$.

The state vector is $\mathbf{x} = (\mathbf{p}, \dot{\mathbf{p}})^\top \in \mathbb{R}^4$; the
linearised continuous-time state-space model about hover is:

$$\dot{\mathbf{x}} = A\mathbf{x} + B\mathbf{u} + E\mathbf{w}$$

$$A = \begin{pmatrix} \mathbf{0}_2 & \mathbf{I}_2 \\ \mathbf{0}_2 & -\frac{b}{m}\mathbf{I}_2 \end{pmatrix}, \quad
  B = \begin{pmatrix} \mathbf{0}_2 \\ \frac{1}{m}\mathbf{I}_2 \end{pmatrix}, \quad
  E = \begin{pmatrix} \mathbf{0}_2 \\ \mathbf{I}_2 \end{pmatrix}$$

where $\mathbf{w} = \mathbf{f}_{wind} + \mathbf{f}_{turb}/m \in \mathbb{R}^2$ is the total
disturbance acceleration.

### Spiral Reference Trajectory

The inbound spiral in polar coordinates is parameterised by time $t$:

$$r_{ref}(t) = r_{start} - v_{inbound} \cdot t, \qquad
  \theta_{ref}(t) = \theta_0 + \omega_{spiral} \cdot t$$

with $v_{inbound} = 0.8$ m/s (radial closure rate), $\omega_{spiral} = 0.12$ rad/s
(angular sweep rate), and $\theta_0 = 0$ (entry angle). The Cartesian reference is:

$$\mathbf{p}_{ref}(t) = r_{ref}(t) \begin{pmatrix} \cos\theta_{ref}(t) \\ \sin\theta_{ref}(t) \end{pmatrix}$$

The reference velocity:

$$\dot{\mathbf{p}}_{ref}(t) = \dot{r}_{ref}\,\hat{\mathbf{e}}_r + r_{ref}\,\dot{\theta}_{ref}\,\hat{\mathbf{e}}_\theta
  = -v_{inbound}\,\hat{\mathbf{e}}_r + r_{ref}\,\omega_{spiral}\,\hat{\mathbf{e}}_\theta$$

The trajectory terminates when $r_{ref}(t) \leq r_{eye}$, giving a total mission time
$T_{mission} = (r_{start} - r_{eye})/v_{inbound} \approx 219$ s.

### MPC Formulation

Discretise the dynamics with timestep $\Delta t = 0.1$ s:

$$\mathbf{x}_{k+1} = A_d\,\mathbf{x}_k + B_d\,\mathbf{u}_k + E_d\,\mathbf{w}_k$$

At each control step $k$, solve the finite-horizon QP over prediction horizon $N_p = 20$:

$$\min_{\{u_k, \ldots, u_{k+N_p-1}\}} \sum_{j=0}^{N_p-1} \Bigl(
  \|\mathbf{x}_{k+j} - \mathbf{x}_{ref,k+j}\|^2_Q + \|\mathbf{u}_{k+j}\|^2_R
\Bigr) + \|\mathbf{x}_{k+N_p} - \mathbf{x}_{ref,k+N_p}\|^2_P$$

subject to:

$$\|\mathbf{u}_{k+j}\|_\infty \leq u_{max} = \frac{mg}{\sqrt{2}}, \qquad j = 0, \ldots, N_p - 1$$

The disturbance term $\mathbf{w}_k$ is provided as a feed-forward input using the known Rankine
wind model evaluated at the predicted positions. The state weighting matrix
$Q = \mathrm{diag}(10, 10, 1, 1)$ penalises position error more than velocity error; the control
weight $R = 0.1\,\mathbf{I}_2$; the terminal weight $P$ is the solution to the discrete algebraic
Riccati equation (DARE).

### H-Infinity Disturbance Rejection

Define the tracking error output $\mathbf{z} = C_z(\mathbf{x} - \mathbf{x}_{ref})$ with
$C_z = \mathrm{diag}(\sqrt{Q_{pos}}, \sqrt{Q_{pos}}, 0, 0)$ selecting position states. The closed-loop
transfer function from disturbance $\mathbf{w}$ to error $\mathbf{z}$ is $T_{zw}(s)$.

The $\mathcal{H}_\infty$ control problem seeks a state-feedback gain $K_\infty$ such that the
closed-loop system $(A - BK_\infty)$ is stable and the $\mathcal{H}_\infty$ norm is bounded:

$$\|T_{zw}\|_\infty = \sup_{\omega \in \mathbb{R}} \bar{\sigma}\!\left[T_{zw}(j\omega)\right] < \gamma$$

The optimal $\gamma^*$ and gain $K_\infty$ are found by solving the $\mathcal{H}_\infty$ Riccati
equation via bisection on $\gamma$. The closed-loop control is:

$$\mathbf{u} = -K_\infty(\mathbf{x} - \mathbf{x}_{ref}) + \mathbf{u}_{ref}$$

where $\mathbf{u}_{ref}$ is the feed-forward reference thrust needed to track $\dot{\mathbf{p}}_{ref}$.

### Tracking Error Metric

Mean radial tracking error over the mission:

$$\bar{e}_{pos} = \frac{1}{T_{steps}} \sum_{k=1}^{T_{steps}} \|\mathbf{p}_k - \mathbf{p}_{ref,k}\|$$

Peak error (worst-case passage through the eyewall):

$$e_{peak} = \max_{k} \|\mathbf{p}_k - \mathbf{p}_{ref,k}\|$$

Control effort (total thrust expenditure):

$$J_{effort} = \Delta t \sum_{k=1}^{T_{steps}} \|\mathbf{u}_k\|$$

---

## Implementation

```python
import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import solve_discrete_are, expm

# Key constants
M_DRONE      = 1.5       # kg — drone mass
G            = 9.81      # m/s^2
B_DRAG       = 0.4       # N·s/m — aerodynamic drag coefficient
F_MAX        = M_DRONE * G  # N — max horizontal thrust (45 deg attitude limit)
R_MAX        = 150.0     # m — Rankine vortex max-wind radius
V_MAX        = 50.0      # m/s — peak tangential wind speed
V_INFLOW     = 8.0       # m/s — peak radial inflow speed
SIGMA_RAD    = 40.0      # m — radial width of inflow layer
R_START      = 190.0     # m — drone entry radius
R_EYE        = 15.0      # m — eye radius (mission success threshold)
V_INBOUND    = 0.8       # m/s — radial closure rate
W_SPIRAL     = 0.12      # rad/s — spiral angular rate
DT           = 0.1       # s — simulation timestep
N_PRED       = 20        # MPC prediction horizon (steps)
Q_MPC        = np.diag([10., 10., 1., 1.])
R_MPC        = 0.1 * np.eye(2)

def rankine_wind(pos):
    """Rankine vortex wind vector at Cartesian position pos = (x, y)."""
    r = np.linalg.norm(pos)
    if r < 1e-6:
        return np.zeros(2)
    e_r = pos / r
    e_th = np.array([-pos[1], pos[0]]) / r
    v_tan = V_MAX * r / R_MAX if r <= R_MAX else V_MAX * R_MAX / r
    v_rad = -V_INFLOW * np.exp(-((r - R_MAX)**2) / (2 * SIGMA_RAD**2))
    return v_tan * e_th + v_rad * e_r

def spiral_reference(t):
    """Return (p_ref, v_ref) on the inbound Archimedean spiral at time t."""
    r_ref = max(R_EYE, R_START - V_INBOUND * t)
    theta_ref = W_SPIRAL * t
    e_r = np.array([np.cos(theta_ref), np.sin(theta_ref)])
    e_th = np.array([-np.sin(theta_ref), np.cos(theta_ref)])
    p_ref = r_ref * e_r
    v_ref = -V_INBOUND * e_r + r_ref * W_SPIRAL * e_th
    return p_ref, v_ref

def build_discrete_ss(dt):
    """Exact ZOH discretisation of the 2D double-integrator + drag model."""
    A = np.block([[np.zeros((2, 2)), np.eye(2)],
                  [np.zeros((2, 2)), -(B_DRAG / M_DRONE) * np.eye(2)]])
    B = np.block([[np.zeros((2, 2))],
                  [(1 / M_DRONE) * np.eye(2)]])
    E = np.block([[np.zeros((2, 2))],
                  [np.eye(2)]])
    n, m = A.shape[0], B.shape[1]
    # Matrix exponential for ZOH
    M_exp = expm(np.block([[A, B, E],
                            [np.zeros((m + 2, n + m + 2))]]) * dt)
    Ad = M_exp[:n, :n]
    Bd = M_exp[:n, n:n + m]
    Ed = M_exp[:n, n + m:]
    return Ad, Bd, Ed

def mpc_step(x, x_ref_seq, w_seq, Ad, Bd, Ed, P_terminal):
    """
    Solve a simplified MPC QP via unconstrained LQR rollout (with clipping).
    Returns the first optimal control action.
    """
    Np = len(x_ref_seq) - 1
    u_seq = []
    x_k = x.copy()
    # Single-step greedy approximation (replace with full QP solver for exact MPC)
    for k in range(Np):
        x_ref_k = x_ref_seq[k]
        w_k = w_seq[k]
        e_k = x_k - x_ref_k
        # LQR gain from terminal P (one-step approximation)
        K_lqr = np.linalg.inv(R_MPC + Bd.T @ P_terminal @ Bd) @ Bd.T @ P_terminal @ Ad
        u_k = -K_lqr @ e_k - Ed @ w_k  # feed-forward wind cancellation
        u_k = np.clip(u_k, -F_MAX / np.sqrt(2), F_MAX / np.sqrt(2))
        u_seq.append(u_k)
        x_k = Ad @ x_k + Bd @ u_k + Ed @ w_k
    return u_seq[0]

def run_simulation(controller='mpc'):
    Ad, Bd, Ed = build_discrete_ss(DT)
    # Terminal cost: DARE solution
    P_terminal = solve_discrete_are(Ad, Bd, Q_MPC, R_MPC)

    T_mission = (R_START - R_EYE) / V_INBOUND
    t_steps = int(T_mission / DT)

    pos = np.array([R_START, 0.0])
    vel = np.zeros(2)
    x = np.concatenate([pos, vel])

    traj = [pos.copy()]
    refs = []
    errors = []
    wind_at_drone = []

    for k in range(t_steps):
        t = k * DT
        p_ref, v_ref = spiral_reference(t)
        x_ref = np.concatenate([p_ref, v_ref])
        refs.append(p_ref.copy())

        # Wind disturbance at current position
        w_now = rankine_wind(x[:2]) + (0.1 * np.linalg.norm(rankine_wind(x[:2]))
                                        * np.random.randn(2))
        wind_at_drone.append(np.linalg.norm(w_now))

        if controller == 'pd':
            Kp, Kd = 4.0, 2.5
            u = Kp * (p_ref - x[:2]) + Kd * (v_ref - x[2:])
            u = np.clip(u, -F_MAX / np.sqrt(2), F_MAX / np.sqrt(2))

        elif controller == 'mpc':
            # Build prediction sequences
            x_ref_seq = []
            w_seq = []
            for j in range(N_PRED + 1):
                p_r, v_r = spiral_reference(t + j * DT)
                x_ref_seq.append(np.concatenate([p_r, v_r]))
                w_seq.append(rankine_wind(x[:2]))  # zero-order hold on wind
            u = mpc_step(x, x_ref_seq, w_seq, Ad, Bd, Ed, P_terminal)

        elif controller == 'hinf':
            # H-inf state feedback (pre-computed gain K_inf)
            # Placeholder: use LQR gain with disturbance feed-forward
            K_inf = np.linalg.inv(R_MPC + Bd.T @ P_terminal @ Bd) @ Bd.T @ P_terminal @ Ad
            u_ref = M_DRONE * (v_ref - x[2:]) / DT  # reference feed-forward
            u = -K_inf @ (x - x_ref) + np.clip(u_ref, -F_MAX, F_MAX) - M_DRONE * w_now
            u = np.clip(u, -F_MAX / np.sqrt(2), F_MAX / np.sqrt(2))

        # Propagate dynamics
        x = Ad @ x + Bd @ u + Ed @ w_now
        traj.append(x[:2].copy())
        errors.append(np.linalg.norm(x[:2] - p_ref))

        if np.linalg.norm(x[:2]) < R_EYE:
            print(f"Eye reached at t = {t:.1f} s (step {k})")
            break

    return np.array(traj), np.array(refs), np.array(errors), np.array(wind_at_drone)
```

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Typhoon domain radius | 200 m |
| Eye radius $r_{eye}$ | 15 m |
| Max-wind radius $r_{max}$ | 150 m |
| Peak tangential wind $v_{max}$ | 50 m/s |
| Peak radial inflow $v_{inflow}$ | 8 m/s |
| Inflow layer width $\sigma_{rad}$ | 40 m |
| Turbulence intensity | 10 % of local $v_{tan}$ |
| Drone mass $m$ | 1.5 kg |
| Max horizontal thrust $F_{max}$ | $mg \approx 14.7$ N (45 deg limit) |
| Aerodynamic drag $b$ | 0.4 N·s/m |
| Entry radius $r_{start}$ | 190 m |
| Radial closure rate $v_{inbound}$ | 0.8 m/s |
| Spiral angular rate $\omega_{spiral}$ | 0.12 rad/s |
| Mission duration $T_{mission}$ | ~219 s |
| Simulation timestep $\Delta t$ | 0.1 s |
| MPC prediction horizon $N_p$ | 20 steps (2 s) |
| MPC position weight $Q_{pos}$ | 10 |
| MPC control weight $R$ | 0.1 |

---

## Expected Output

- **Spiral trajectory plot**: x-y plane showing the prescribed spiral reference (dashed) and
  actual drone trajectories for all three controllers (PD in orange, MPC in green, H-inf in
  purple); eye boundary drawn as a filled green circle; eyewall ring at $r_{max}$ shown as a
  light-grey annulus; wind speed colour-mapped in the background.
- **Wind field quiver plot**: 2D vector field of $\mathbf{f}_{wind}(\mathbf{p})$ over the domain,
  showing the rotational vortex structure with peak vectors at $r_{max}$; drone entry point marked
  with an arrow.
- **Tracking error time series**: $\|\mathbf{p}(t) - \mathbf{p}_{ref}(t)\|$ vs time for all three
  controllers on the same axes; vertical dashed line at the eyewall crossing ($r \approx r_{max}$);
  shaded region indicating the critical passage zone.
- **Wind speed profile**: $v_{tan}(r)$ and $|v_{rad}(r)|$ vs radial distance $r$, highlighting the
  Rankine discontinuity of slope at $r_{max}$ and the inflow layer.
- **Controller comparison bar chart**: mean tracking error $\bar{e}_{pos}$, peak error $e_{peak}$,
  and control effort $J_{effort}$ for PD, MPC, and H-inf; success/failure indicator (did the drone
  reach the eye?).
- **Control input time series**: $u_x(t)$ and $u_y(t)$ vs time showing saturation events during
  eyewall crossing; thrust magnitude $\|\mathbf{u}(t)\|$ with $F_{max}$ dashed.
- **Animation (GIF)**: drone moving along the spiral inbound path with a wind-vector arrow
  updating in real time; colour trace fading behind the drone; eye region lighting up green on
  successful arrival.

---

## Extensions

1. **Adaptive wind estimation**: the true wind field is not known to the controller; implement an
   online Extended Kalman Filter (EKF) to estimate wind speed and direction from the drone's IMU
   residuals, and feed the estimate into the MPC disturbance term. Evaluate convergence rate vs
   estimation error within the eyewall.
2. **3D vertical profile**: extend the simulation to three dimensions and assign a vertical
   profile mission — the drone descends from 500 m (upper troposphere) to 50 m (boundary layer)
   while spiralling inward, collecting a full meteorological sounding. The vertical wind shear adds
   an additional z-disturbance layer.
3. **Multi-drone constellation**: deploy $N = 3$ drones at evenly-spaced entry angles
   ($120°$ apart) to sample the azimuthal wind variability and compare simultaneous eyewall
   crossings; coordinate so that drones reach the eye simultaneously for a joint pressure
   measurement.
4. **Moving eye**: allow the eye centre to translate at $v_{eye} = 5$ m/s (typhoon track), so the
   reference spiral must be continuously recomputed in the eye-fixed frame; assess how eye
   translation degrades tracking performance and what look-ahead horizon the MPC needs.
5. **RL disturbance rejection**: train a PPO agent on randomised vortex intensities
   ($v_{max} \in [20, 60]$ m/s) and eye radii ($r_{max} \in [80, 200]$ m); test zero-shot
   generalisation to a Category-5 vortex profile not seen during training; compare with the
   H-inf controller on worst-case wind scenarios.

---

## Related Scenarios

- Prerequisites: [S041 Wildfire Boundary Scan](S041_wildfire_boundary.md), [S060 Meteorological Profiling](S060_meteorological_profiling.md)
- Follow-ups: [S059 Sonar Buoy Relay](S059_sonar_buoy_relay.md) (ocean hazard environment), [S055 Oil Spill Tracking](S055_oil_spill_tracking.md) (extreme environment monitoring)
- Algorithmic cross-reference: [S034 Weather Rerouting](../02_logistics_delivery/S034_weather_rerouting.md) (wind disturbance replanning), [S004 Disturbance Rejection Chase](../01_pursuit_evasion/S004_disturbance_rejection.md) (H-inf control in pursuit context)
