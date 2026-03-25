# Mathematical Foundations

> This document collects the mathematical tools shared across all scenarios. Individual scenario cards reference sections from here.

## 1. Quadrotor Dynamics Model

### 1.1 Coordinate Frames

- **World frame (NED / ENU)**: x points north (or east), y points east (or north), z points up
- **Body frame**: x points forward, y points right, z points down

gym-pybullet-drones uses the **ENU + z-up** world coordinate frame.

### 1.2 Rigid Body Equations of Motion

Position: $\mathbf{p} = [x, y, z]^T$, velocity: $\mathbf{v} = \dot{\mathbf{p}}$

Attitude (RPY): $\boldsymbol{\eta} = [\phi, \theta, \psi]^T$ (Roll, Pitch, Yaw)

**Translational equations (Newton's second law):**

$$m\ddot{\mathbf{p}} = \mathbf{R}\begin{bmatrix}0\\0\\F_{thrust}\end{bmatrix} - \begin{bmatrix}0\\0\\mg\end{bmatrix} + \mathbf{f}_{drag}$$

**Rotational equations (Euler):**

$$\mathbf{I}\dot{\boldsymbol{\omega}} = \boldsymbol{\tau} - \boldsymbol{\omega} \times \mathbf{I}\boldsymbol{\omega}$$

where $\mathbf{I}$ is the inertia tensor, $\boldsymbol{\omega}$ is the angular velocity, and $\boldsymbol{\tau}$ is the torque.

### 1.3 Quadrotor Thrust Model

Four rotor speeds $\Omega_i$ ($i = 1..4$):

$$F_i = k_f \Omega_i^2, \quad \tau_i = k_m \Omega_i^2$$

Total thrust $F = \sum F_i$, total torques:

$$\begin{bmatrix}\tau_\phi\\\tau_\theta\\\tau_\psi\end{bmatrix} = \begin{bmatrix}l(F_2 - F_4)\\l(F_1 - F_3)\\-({\tau_1 - \tau_2 + \tau_3 - \tau_4})\end{bmatrix}$$

where $l$ is the arm length, and $k_f$, $k_m$ are rotor constants.

### 1.4 Linearization (Near Hover)

Hover point: $\phi = \theta = 0$, $\psi = \psi_0$, $z = z_0$

Under small-angle approximation, four decoupled subsystems ($x$, $y$, $z$, $\psi$) can be derived, each amenable to PID or LQR controller design.

---

## 2. Control Theory Fundamentals

### 2.1 PID Controller

$$u(t) = K_p e(t) + K_i \int_0^t e(\tau)d\tau + K_d \dot{e}(t)$$

gym-pybullet-drones' `DSLPIDControl` is a cascade PID: outer loop (position) → inner loop (attitude).

### 2.2 LQR Optimal Control

Minimize the cost function:

$$J = \int_0^\infty (x^T Q x + u^T R u) \, dt$$

Solve the Riccati equation to obtain the optimal gain $K$: $u = -Kx$

### 2.3 Model Predictive Control (MPC)

Receding horizon optimization: at each time step, solve a finite-horizon optimal control problem:

$$\min_{\mathbf{u}} \sum_{k=0}^{N-1} \left[ x_k^T Q x_k + u_k^T R u_k \right] + x_N^T P x_N$$

subject to: $x_{k+1} = A x_k + B u_k$, $u_{min} \le u_k \le u_{max}$

---

## 3. Path Planning

### 3.1 RRT / RRT*

- **RRT**: Rapidly-exploring Random Tree, random expansion in configuration space
- **RRT***: Asymptotically optimal variant, introduces a "rewiring" step

Complexity: $O(n \log n)$ (RRT*)

### 3.2 A* Search

On a discrete grid, $f(n) = g(n) + h(n)$, where $h$ is a heuristic (e.g., Euclidean distance).

3D extension: 26-connected grid (including diagonal directions).

### 3.3 Dubins / Reeds-Shepp Paths

Applicable to drones with a minimum turning radius constraint; shortest path composed of three arcs or straight-line segments.

---

## 4. Pursuit and Differential Games

### 4.1 Proportional Navigation Guidance (PNG)

$$\dot{\gamma} = N \dot{\lambda}$$

where $\gamma$ is the pursuer's heading, $\lambda$ is the Line of Sight (LOS) angle, and $N$ is the navigation constant (typically $N=3\sim5$).

### 4.2 Lion and Man Problem

Lion speed $v_L$, Man speed $v_M$, circular domain of radius $R$:
- $v_L \ge v_M$: Lion is guaranteed to catch the Man
- Open space: if $v_L > v_M$, capture time is finite

### 4.3 Isaacs Differential Game

Zero-sum two-player game; pursuer minimizes capture time, evader maximizes it:

$$V(x) = \min_u \max_d \int_0^T L(x, u, d) \, dt$$

Hamilton-Jacobi-Isaacs (HJI) equation:

$$\frac{\partial V}{\partial t} + H\left(x, \frac{\partial V}{\partial x}\right) = 0$$

---

## 5. Multi-Agent Coordination

### 5.1 Potential Field Method

Net force = target attraction + obstacle repulsion + other agent repulsion

$$\mathbf{F}_{total} = \mathbf{F}_{att} + \mathbf{F}_{rep}$$

### 5.2 Flocking (Reynolds Rules)

1. **Separation**: avoid colliding with neighbors
2. **Alignment**: match neighbors' velocity
3. **Cohesion**: move toward the neighbors' center of mass

### 5.3 Task Assignment (Hungarian Algorithm)

Given $n$ drones and $m$ tasks, cost matrix $C_{ij}$, find the minimum-cost perfect matching. Complexity $O(n^3)$.

---

## 6. Sensor Models

### 6.1 Gaussian Noise Model

$$z = h(x) + w, \quad w \sim \mathcal{N}(0, R)$$

### 6.2 Kalman Filter (KF)

Predict: $\hat{x}_{k|k-1} = A\hat{x}_{k-1|k-1}$, $P_{k|k-1} = AP_{k-1|k-1}A^T + Q$

Update: $K_k = P_{k|k-1}H^T(HP_{k|k-1}H^T + R)^{-1}$, $\hat{x}_{k|k} = \hat{x}_{k|k-1} + K_k(z_k - H\hat{x}_{k|k-1})$

### 6.3 Particle Filter (for nonlinear/non-Gaussian systems)

Approximate the posterior distribution with $N$ particles $\{x_i, w_i\}$; each step performs: propagate → weight → resample.

---

## 7. Coverage Metrics

### 7.1 Area Coverage Rate

$$\text{Coverage} = \frac{|\text{scanned cells}|}{|\text{total cells}|} \times 100\%$$

### 7.2 Lawnmower Trajectory Planning

Optimal strip width $d^*$ (a function of sensor FOV and overlap ratio), number of strips $N = \lceil W / (d \cdot (1-\text{overlap})) \rceil$.

---

## References

- Isaacs, R. (1965). *Differential Games*. Wiley.
- LaValle, S. (2006). *Planning Algorithms*. Cambridge.
- Mahony, R., et al. (2012). Multirotor aerial vehicles: Modeling, estimation, and control. *IEEE Robotics & Automation Magazine*.
- Panerati, J., et al. (2021). Learning to fly—a gym environment with PyBullet physics. *IROS 2021*.
