# S092 Movie Car Chase Scene

**Domain**: Special Ops & Entertainment | **Difficulty**: ⭐⭐⭐ | **Status**: `[ ]` Not started

**Algorithm**: Predictive KF Tracking + MPC Obstacle-Aware Trajectory | **Dimension**: 3D

---

## Problem Definition

**Setup**: A film production deploys a single camera drone to track a stunt car driving a winding
300 m road at 8 m/s. The road includes 5 static concrete pillars (obstacles) arranged along its
length. The drone must maintain a cinematic offset — 5 m above and 8 m directly behind the car —
while flying around the pillars. Because radio-control latency and aerodynamic lag prevent the
drone from reacting instantly, it uses a **Kalman Filter** to predict the car's position 2 s into
the future, then feeds that prediction into a **Model Predictive Controller** (MPC) that plans a
10-step receding-horizon trajectory respecting obstacle-clearance constraints.

**Roles**:
- **Car (target)**: follows a pre-scripted sinusoidal winding road at constant tangential speed
  v = 8 m/s; acts as a disturbance source for the drone controller.
- **Camera drone**: executes KF-predicted MPC trajectory to maintain desired relative position
  while avoiding 5 pillars; broadcasts no active signals — purely reactive observer.

**Objective**: Maximise the fraction of mission time during which (a) the drone is within a
tolerance ball of the desired offset position and (b) the car occupies 20–40 % of the camera
frame height, while maintaining a minimum safety clearance r_safe = 1.5 m from every pillar.

---

## Mathematical Model

### Car Kinematics

The car follows a parametric winding road. Its 2D ground-plane position at arc-length parameter
s(t) = v · t is:

$$x_{car}(t) = s(t), \qquad y_{car}(t) = A \sin\!\left(\frac{2\pi\, s(t)}{L_{wave}}\right)$$

with amplitude A = 15 m and wavelength L_wave = 80 m. The heading angle is:

$$\psi_{car}(t) = \arctan\!\left(\frac{\dot{y}_{car}}{\dot{x}_{car}}\right) = \arctan\!\left(\frac{2\pi A}{L_{wave}} \cos\!\left(\frac{2\pi s}{L_{wave}}\right)\right)$$

The car's rotation matrix from body to world frame is:

$$R_{car}(t) = \begin{pmatrix} \cos\psi_{car} & -\sin\psi_{car} & 0 \\ \sin\psi_{car} & \cos\psi_{car} & 0 \\ 0 & 0 & 1 \end{pmatrix}$$

### Kalman Filter — Car State Estimation

The KF state vector is position and velocity in the world frame:

$$\mathbf{x}_{KF} = \begin{pmatrix} x \\ y \\ v_x \\ v_y \end{pmatrix}$$

The constant-velocity process model with timestep dt propagates state as:

$$\mathbf{x}_{k+1} = F\,\mathbf{x}_k + \mathbf{w}_k, \qquad \mathbf{w}_k \sim \mathcal{N}(\mathbf{0}, Q)$$

$$F = \begin{pmatrix} I_{2\times2} & \Delta t\, I_{2\times2} \\ 0_{2\times2} & I_{2\times2} \end{pmatrix}$$

Process noise covariance (models unmodelled centripetal accelerations):

$$Q = q\, \begin{pmatrix} \tfrac{\Delta t^3}{3} I & \tfrac{\Delta t^2}{2} I \\ \tfrac{\Delta t^2}{2} I & \Delta t\, I \end{pmatrix}, \qquad q = 1.0\ \text{m}^2/\text{s}^3$$

The observation model receives noisy GPS-like position measurements:

$$\mathbf{z}_k = H\,\mathbf{x}_k + \mathbf{v}_k, \qquad H = \begin{pmatrix} I_{2\times2} & 0_{2\times2} \end{pmatrix}, \qquad \mathbf{v}_k \sim \mathcal{N}(\mathbf{0}, R_{meas})$$

$$R_{meas} = \sigma_{meas}^2\, I_{2\times2}, \qquad \sigma_{meas} = 0.3\ \text{m}$$

The predicted car position T_pred = 2 s ahead is:

$$\hat{\mathbf{x}}_{k+n} = F^n\, \hat{\mathbf{x}}_{k|k}, \qquad n = \left\lfloor \frac{T_{pred}}{\Delta t} \right\rfloor$$

### Desired Drone Position

Given the predicted car position p_car and heading R_car, the desired drone position maintains a
fixed body-frame offset:

$$\mathbf{p}_{d}(t) = \mathbf{p}_{car}(t) + R_{car}(t)\, \mathbf{d}_{offset}$$

$$\mathbf{d}_{offset} = \begin{pmatrix} 0 \\ -8 \\ 5 \end{pmatrix}\ \text{m} \quad \text{(behind and above in car body frame)}$$

The third component (height) is added as a world-frame z offset; R_car only rotates the
horizontal plane components.

### MPC Formulation

At each control step k the MPC solves a finite-horizon quadratic program over N = 10 steps with
timestep dt_mpc = 0.2 s. The state is drone position p_i and velocity v_i; the input is drone
acceleration u_i (treated as direct thrust command in the simplified model).

**Cost function** (tracking + control effort):

$$J = \sum_{i=0}^{N-1} \left( \|\mathbf{p}_i - \mathbf{p}_{d,i}\|^2_{Q_{mpc}} + \|\mathbf{u}_i\|^2_{R_{mpc}} \right) + \|\mathbf{p}_N - \mathbf{p}_{d,N}\|^2_{P_f}$$

with weights Q_mpc = diag(10, 10, 10), R_mpc = diag(0.1, 0.1, 0.1), P_f = diag(20, 20, 20).

**Drone dynamics** (double integrator, dt_mpc = 0.2 s):

$$\begin{pmatrix} \mathbf{p}_{i+1} \\ \mathbf{v}_{i+1} \end{pmatrix} = \begin{pmatrix} I & \Delta t_{mpc}\, I \\ 0 & I \end{pmatrix} \begin{pmatrix} \mathbf{p}_i \\ \mathbf{v}_i \end{pmatrix} + \begin{pmatrix} \tfrac{\Delta t_{mpc}^2}{2} I \\ \Delta t_{mpc}\, I \end{pmatrix} \mathbf{u}_i$$

**Obstacle constraints** (linearised as half-plane cuts at each MPC step):

For each pillar k with centre o_k and inflation radius r_safe:

$$\|\mathbf{p}_i^{xy} - \mathbf{o}_k^{xy}\| \geq r_{safe} \qquad \forall\, k \in \{1,\ldots,5\},\; i \in \{0,\ldots,N\}$$

This nonlinear constraint is linearised around the current drone position p_0 as a half-plane:

$$\hat{\mathbf{n}}_k^\top (\mathbf{p}_i^{xy} - \mathbf{o}_k^{xy}) \geq r_{safe}, \qquad \hat{\mathbf{n}}_k = \frac{\mathbf{p}_0^{xy} - \mathbf{o}_k^{xy}}{\|\mathbf{p}_0^{xy} - \mathbf{o}_k^{xy}\|}$$

**Velocity and acceleration bounds**:

$$\|\mathbf{v}_i\|_\infty \leq v_{max} = 12\ \text{m/s}, \qquad \|\mathbf{u}_i\|_\infty \leq a_{max} = 6\ \text{m/s}^2$$

### Camera Framing Metric

The car subtends a vertical angle in the camera frame proportional to the inverse of the
drone-to-car distance. For camera focal length f and car height h_car:

$$\theta_{frame}(t) = 2 \arctan\!\left(\frac{h_{car}}{2\, \|\mathbf{p}_{drone}(t) - \mathbf{p}_{car}(t)\|}\right)$$

The fraction of frame height occupied by the car (assuming vertical FoV = 60 deg):

$$f_{height}(t) = \frac{\theta_{frame}(t)}{\theta_{FoV}}$$

The framing score over the mission of duration T is:

$$f_{score} = \frac{1}{T} \int_0^T \mathbf{1}\!\left[0.20 \leq f_{height}(t) \leq 0.40\right] dt$$

### Tracking Lag

The root-mean-square position error between the actual drone position and the desired offset
position measures how closely the controller tracks the cinematic framing target:

$$\epsilon_{track} = \sqrt{\frac{1}{T} \int_0^T \|\mathbf{p}_{drone}(t) - \mathbf{p}_{d}(t)\|^2\, dt}$$

### Obstacle Clearance

The minimum clearance over the full trajectory:

$$d_{clear} = \min_{t \in [0,T],\; k \in \{1,\ldots,5\}} \|\mathbf{p}_{drone}^{xy}(t) - \mathbf{o}_k^{xy}\|$$

A successful mission requires d_clear >= r_safe = 1.5 m.

---

## Implementation

```python
import numpy as np
from scipy.optimize import linprog   # LP fallback for obstacle linearisation
import cvxpy as cp                   # QP solver for MPC
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# ── Scenario geometry ──────────────────────────────────────────────────────────
ROAD_LEN   = 300.0   # m
CAR_SPEED  = 8.0     # m/s
A_WAVE     = 15.0    # m  (road amplitude)
L_WAVE     = 80.0    # m  (road wavelength)
D_OFFSET   = np.array([0.0, -8.0, 5.0])   # body-frame offset (behind, above)

PILLARS = np.array([          # (x, y, z_base) world coords; cylindrical, height 6 m
    [ 40.0,  8.0, 0.0],
    [ 90.0, -10.0, 0.0],
    [150.0,  12.0, 0.0],
    [210.0, -6.0,  0.0],
    [260.0,  9.0,  0.0],
])
PILLAR_RADIUS = 0.6  # m (physical)
R_SAFE        = 1.5  # m (inflated for MPC)

# ── Kalman filter ──────────────────────────────────────────────────────────────
DT_KF   = 0.05   # s — KF update rate
Q_NOISE = 1.0    # process noise spectral density
R_MEAS  = 0.3**2 * np.eye(2)

def kf_matrices(dt):
    F = np.block([[np.eye(2), dt*np.eye(2)], [np.zeros((2,2)), np.eye(2)]])
    Q = Q_NOISE * np.block([
        [(dt**3/3)*np.eye(2), (dt**2/2)*np.eye(2)],
        [(dt**2/2)*np.eye(2),  dt       *np.eye(2)],
    ])
    H = np.hstack([np.eye(2), np.zeros((2,2))])
    return F, Q, H

def kf_predict(x, P, F, Q):
    return F @ x, F @ P @ F.T + Q

def kf_update(x, P, z, H, R):
    S = H @ P @ H.T + R
    K = P @ H.T @ np.linalg.inv(S)
    x_new = x + K @ (z - H @ x)
    P_new = (np.eye(4) - K @ H) @ P
    return x_new, P_new

def predict_ahead(x_hat, F, steps):
    for _ in range(steps): x_hat = F @ x_hat
    return x_hat[:2]    # return position only

# ── MPC ───────────────────────────────────────────────────────────────────────
DT_MPC = 0.2;  N_HOR = 10
Q_MPC  = 10.0 * np.eye(3);  R_MPC = 0.1 * np.eye(3);  P_F = 20.0 * np.eye(3)
V_MAX  = 12.0;  A_MAX = 6.0

def solve_mpc(p0, v0, p_ref_seq, pillars):
    """Solve one MPC QP; p_ref_seq shape (N+1, 3)."""
    p = cp.Variable((N_HOR+1, 3))
    v = cp.Variable((N_HOR+1, 3))
    u = cp.Variable((N_HOR,   3))
    cost, constraints = 0, []
    constraints += [p[0] == p0, v[0] == v0]
    for i in range(N_HOR):
        cost += cp.quad_form(p[i]-p_ref_seq[i], Q_MPC) + cp.quad_form(u[i], R_MPC)
        constraints += [
            p[i+1] == p[i] + DT_MPC*v[i] + 0.5*DT_MPC**2*u[i],
            v[i+1] == v[i] + DT_MPC*u[i],
            cp.norm_inf(v[i+1]) <= V_MAX,
            cp.norm_inf(u[i])   <= A_MAX,
        ]
        # Linearised obstacle half-plane constraints (xy only)
        for ok in pillars[:, :2]:
            diff = p0[:2] - ok
            n_hat = diff / (np.linalg.norm(diff) + 1e-6)
            constraints += [n_hat @ (p[i+1, :2] - ok) >= R_SAFE]
    cost += cp.quad_form(p[N_HOR]-p_ref_seq[N_HOR], P_F)
    prob = cp.Problem(cp.Minimize(cost), constraints)
    prob.solve(solver=cp.OSQP, warm_start=True, verbose=False)
    return (p.value[1], v.value[1]) if prob.status in ("optimal","optimal_inaccurate") \
           else (p0 + DT_MPC*v0, v0)  # fallback: hold velocity

# ── Car model ─────────────────────────────────────────────────────────────────
def car_state(t):
    s   = CAR_SPEED * t
    x   = s
    y   = A_WAVE * np.sin(2*np.pi*s / L_WAVE)
    vx  = CAR_SPEED
    vy  = CAR_SPEED * (2*np.pi*A_WAVE/L_WAVE) * np.cos(2*np.pi*s/L_WAVE)
    psi = np.arctan2(vy, vx)
    R   = np.array([[np.cos(psi), -np.sin(psi), 0],
                    [np.sin(psi),  np.cos(psi), 0],
                    [0,            0,            1]])
    pos = np.array([x, y, 0.0])
    return pos, R

def desired_drone_pos(t):
    p_car, R_car = car_state(t)
    return p_car + R_car @ D_OFFSET

# ── Main simulation loop ───────────────────────────────────────────────────────
def run_simulation(T_total=37.5, rng_seed=0):
    rng   = np.random.default_rng(rng_seed)
    F, Q, H = kf_matrices(DT_KF)
    p_car0, _ = car_state(0.0)
    x_kf = np.array([p_car0[0], p_car0[1], CAR_SPEED, 0.0])
    P_kf = np.eye(4) * 1.0

    p_drone = desired_drone_pos(0.0).copy()
    v_drone = np.zeros(3)
    t, history = 0.0, []

    while t <= T_total:
        p_car, R_car = car_state(t)

        # KF update with noisy observation
        z = p_car[:2] + rng.normal(0, 0.3, 2)
        x_kf, P_kf = kf_predict(x_kf, P_kf, F, Q)
        x_kf, P_kf = kf_update(x_kf, P_kf, z, H, R_MEAS)

        # Predict car 2 s ahead; build reference sequence for MPC horizon
        steps_ahead = int(2.0 / DT_KF)
        p_car_pred = predict_ahead(x_kf.copy(), F, steps_ahead)
        psi_pred   = np.arctan2(x_kf[3], x_kf[2])
        R_pred     = np.array([[np.cos(psi_pred), -np.sin(psi_pred), 0],
                                [np.sin(psi_pred),  np.cos(psi_pred), 0],
                                [0, 0, 1]])
        p_d_now  = np.array([p_car_pred[0], p_car_pred[1], 0.0]) + R_pred @ D_OFFSET

        p_ref_seq = np.array([
            desired_drone_pos(t + i*DT_MPC) for i in range(N_HOR+1)
        ])

        p_drone, v_drone = solve_mpc(p_drone, v_drone, p_ref_seq, PILLARS)
        history.append({
            "t":       t,
            "p_drone": p_drone.copy(),
            "p_car":   p_car.copy(),
            "p_d":     desired_drone_pos(t),
        })
        t += DT_MPC
    return history
```

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Road length | 300 m |
| Car speed | 8.0 m/s |
| Road amplitude (A) | 15.0 m |
| Road wavelength (L_wave) | 80.0 m |
| Mission duration | 37.5 s (car traverses full road) |
| Desired offset — longitudinal (behind) | 8.0 m |
| Desired offset — vertical (above) | 5.0 m |
| Number of pillars | 5 |
| Pillar physical radius | 0.6 m |
| Obstacle inflation radius (r_safe) | 1.5 m |
| KF measurement noise std | 0.3 m |
| KF process noise density (q) | 1.0 m²/s³ |
| KF prediction horizon (T_pred) | 2.0 s |
| MPC horizon (N) | 10 steps |
| MPC timestep (dt_mpc) | 0.2 s |
| MPC position weight (Q_mpc) | 10 (per axis) |
| MPC control weight (R_mpc) | 0.1 (per axis) |
| Drone max speed (v_max) | 12.0 m/s |
| Drone max acceleration (a_max) | 6.0 m/s² |
| Camera vertical FoV | 60 deg |
| Target framing zone (f_height) | 20–40 % of frame |
| Car height for framing model (h_car) | 1.5 m |

---

## Expected Output

- **3D trajectory plot**: car road (blue curve), drone flight path (red curve), pillars (grey
  cylinders), desired offset path (green dashed), start/end markers; axes labelled x/y/z (m).
  Uses `from mpl_toolkits.mplot3d import Axes3D`.

- **Tracking error time-series**: RMS position error ||p_drone(t) - p_d(t)|| vs time; shaded
  bands mark windows where car is near a pillar; dashed horizontal line at 1.0 m tolerance.

- **Camera framing plot**: f_height(t) vs time with shaded target zone [0.20, 0.40];
  framing score f_score annotated in legend.

- **Obstacle clearance plot**: minimum distance from drone to nearest pillar vs time; red
  dashed horizontal line at r_safe = 1.5 m; must never dip below this line.

- **Animation (MP4 / GIF)**: top-down 2D view animated at 10 fps; car (blue dot), drone (red
  dot), pillars (grey circles), desired position (green cross), KF prediction (cyan diamond);
  running framing score displayed in title.

- **Summary metrics table** (printed to stdout):

  | Metric | Value |
  |--------|-------|
  | Framing score f_score | target >= 0.80 |
  | RMS tracking error | target <= 1.5 m |
  | Min obstacle clearance | target >= 1.5 m |
  | Max drone speed | target <= 12 m/s |

---

## Extensions

1. **Dynamic obstacles**: replace static pillars with oncoming vehicles driving the opposite
   lane; extend the MPC to track predicted obstacle trajectories using a separate KF per vehicle,
   requiring time-varying obstacle half-planes in the QP.

2. **Occlusion-aware framing**: add a visibility constraint ensuring the line-of-sight from
   drone camera to car does not pass within a pillar radius; encode as a mixed-integer
   constraint or handle via penalty shaping in the cost function.

3. **Multi-drone cinematic coverage**: add a second drone maintaining a lateral angle (S086
   style arc orbit), coordinated so the two drones never occlude each other's shot; requires
   joint MPC with inter-drone separation constraints.

4. **Wind disturbance rejection**: inject a spatially-varying wind field (Dryden turbulence
   model); augment the MPC model with a disturbance integrator and compare with/without
   feedforward wind compensation.

5. **Real-time NMPC**: replace the linearised obstacle half-planes with a full nonlinear MPC
   (CasADi + IPOPT) to handle tighter clearances and evaluate the computational budget
   for embedded deployment.

---

## Related Scenarios

- Prerequisites: [S081 Selfie Follow Mode](S081_selfie_follow.md), [S086 Cooperative Multi-Angle Filming](S086_multi_angle_cinema.md)
- Next: [S094 Counter-Drone Intercept](S094_counter_drone.md)
- Pursuit-evasion cross-reference: [S003 Low-Altitude Tracking](../01_pursuit_evasion/S003_low_altitude_tracking.md), [S004 Obstacle-Course Chase](../01_pursuit_evasion/S004_obstacle_course_chase.md)

## References

- Rawlings, J. B., Mayne, D. Q., & Diehl, M. (2017). *Model Predictive Control: Theory, Computation, and Design* (2nd ed.). Nob Hill Publishing.
- Welch, G. & Bishop, G. (2006). *An Introduction to the Kalman Filter*. UNC Chapel Hill TR 95-041.
- Falanga, D., Kleber, K., Mintchev, S., Floreano, D., & Scaramuzza, D. (2018). The Foldable Drone: A Morphing Quadrotor. *IEEE Robotics and Automation Letters*, 3(2), 1168–1175.
