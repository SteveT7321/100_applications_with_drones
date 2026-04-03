# S026 Cooperative Heavy Lift

**Domain**: Logistics & Delivery | **Difficulty**: ⭐⭐⭐ | **Status**: `[ ]` Not started

---

## Problem Definition

**Setup**: $N = 4$ quadrotor drones are rigidly attached to an oversized cargo package via inextensible cables of known length $l_i$. The package mass $m_L = 0.8$ kg exceeds the single-drone payload limit of 0.1 kg. The drones hover in a symmetric cross formation above the load and must jointly lift it from ground level to a delivery altitude of $z_d = 4.0$ m, translate horizontally to a target position, and perform a coordinated descent.

**Roles**:
- **Drones 1–4**: each contributes upward thrust $T_i$ and maintains a fixed cable attachment point $\mathbf{b}_i$ on the package body frame
- **Load**: a rigid body of mass $m_L$ suspended at the centroid of four cable attachment points; no independent actuation

**Objective**: compute the minimum-tension cable force distribution that satisfies static equilibrium at every waypoint, enforce $f_i \geq f_{min}$ (no cable slack), and simulate the closed-loop trajectory tracking with a PD position controller on each drone.

---

## Mathematical Model

### Cable Geometry

Each drone $i$ is located at world position $\mathbf{p}_i \in \mathbb{R}^3$. The cable attachment point on the load is $\mathbf{b}_i$ (body frame). When the load centroid is at $\mathbf{q} \in \mathbb{R}^3$, the cable unit vector pointing from attachment point to drone is:

$$\hat{\mathbf{c}}_i = \frac{\mathbf{p}_i - (\mathbf{q} + \mathbf{b}_i)}{\|\mathbf{p}_i - (\mathbf{q} + \mathbf{b}_i)\|}$$

The tension force applied to the load by cable $i$ is:

$$\mathbf{F}_i = f_i \hat{\mathbf{c}}_i, \quad f_i \geq 0$$

### Static Equilibrium

For the load to be in static equilibrium (zero translational and rotational acceleration), six conditions must hold:

**Force balance (3 equations):**

$$\sum_{i=1}^{N} \mathbf{F}_i = m_L g \hat{\mathbf{z}}$$

**Moment balance about load centroid (3 equations):**

$$\sum_{i=1}^{N} \mathbf{b}_i \times \mathbf{F}_i = \mathbf{0}$$

Stacking these into a linear system $\mathbf{A} \mathbf{f} = \mathbf{b}$, where $\mathbf{A} \in \mathbb{R}^{6 \times N}$ and $\mathbf{f} = [f_1, \ldots, f_N]^T$:

$$\mathbf{A} = \begin{bmatrix} \hat{\mathbf{c}}_1 & \hat{\mathbf{c}}_2 & \cdots & \hat{\mathbf{c}}_N \\ \mathbf{b}_1 \times \hat{\mathbf{c}}_1 & \mathbf{b}_2 \times \hat{\mathbf{c}}_2 & \cdots & \mathbf{b}_N \times \hat{\mathbf{c}}_N \end{bmatrix}$$

$$\mathbf{b}_{rhs} = \begin{bmatrix} 0 \\ 0 \\ m_L g \\ 0 \\ 0 \\ 0 \end{bmatrix}$$

### Minimum-Norm Tension (Pseudo-Inverse)

For $N > 3$ the system is underdetermined. The minimum $\ell_2$-norm solution via the Moore–Penrose pseudo-inverse is:

$$\mathbf{f}^* = \mathbf{A}^T (\mathbf{A} \mathbf{A}^T)^{-1} \mathbf{b}_{rhs}$$

This minimises $\|\mathbf{f}\|_2$ subject to $\mathbf{A}\mathbf{f} = \mathbf{b}_{rhs}$.

### Anti-Slack QP

The pseudo-inverse solution may produce $f_i < 0$ (cable pushes — physically impossible). To enforce $f_i \geq f_{min} > 0$ (cable tension margin), solve the constrained Quadratic Program:

$$\min_{\mathbf{f}} \quad \mathbf{f}^T \mathbf{W} \mathbf{f}$$

$$\text{subject to} \quad \mathbf{A}\mathbf{f} = \mathbf{b}_{rhs}, \quad \mathbf{f} \geq f_{min} \cdot \mathbf{1}$$

where $\mathbf{W} = \text{diag}(w_1, \ldots, w_N)$ is a positive-definite weighting matrix (default: $\mathbf{W} = \mathbf{I}$). Solved with `scipy.optimize.minimize` (SLSQP) or `cvxpy`.

### Required Drone Thrust

Each drone must produce the upward component of its cable tension plus compensate for its own weight:

$$T_i^{cmd} = \frac{f_i}{\cos \alpha_i} + m_d g$$

where $\alpha_i$ is the cable-to-vertical angle and $m_d$ is the drone body mass.

### Formation Waypoints

The four drones fly in a symmetric cross pattern offset from the load centroid $\mathbf{q}$:

$$\mathbf{p}_i^{des}(t) = \mathbf{q}^{des}(t) + \mathbf{d}_i$$

$$\mathbf{d}_1 = [+s,\ 0,\ h]^T, \quad \mathbf{d}_2 = [-s,\ 0,\ h]^T, \quad \mathbf{d}_3 = [0,\ +s,\ h]^T, \quad \mathbf{d}_4 = [0,\ -s,\ h]^T$$

where $s = 0.6$ m is the horizontal formation radius and $h = l_c \cos\alpha_0$ is the vertical cable projection at nominal angle $\alpha_0$.

### PD Position Controller (per Drone)

Each drone tracks its desired waypoint with a simple PD law:

$$\mathbf{a}_i^{cmd} = K_p \left(\mathbf{p}_i^{des} - \mathbf{p}_i\right) + K_d \left(\dot{\mathbf{p}}_i^{des} - \dot{\mathbf{p}}_i\right)$$

The total thrust command adds the tension feedforward:

$$\mathbf{F}_i^{total} = m_d \left(\mathbf{a}_i^{cmd} + g\hat{\mathbf{z}}\right) + \mathbf{F}_i^{cable}$$

where $\mathbf{F}_i^{cable} = f_i \hat{\mathbf{c}}_i$ is the reaction of the cable on the drone (equal and opposite to the force on the load).

### Load Kinematics (Simulation)

The load centroid is propagated by aggregating the net cable force:

$$m_L \ddot{\mathbf{q}} = \sum_{i=1}^{N} \mathbf{F}_i - m_L g \hat{\mathbf{z}}$$

integrated with Euler steps at $\Delta t = 0.01$ s.

### Tension Feasibility Metric

The slack margin across all cables at each timestep:

$$\delta_{slack}(t) = \min_{i} f_i(t) - f_{min}$$

Mission succeeds only if $\delta_{slack}(t) \geq 0$ for all $t$.

---

## Implementation

```python
import numpy as np
from scipy.optimize import minimize

# Key constants
N_DRONES      = 4
MASS_LOAD     = 0.8    # kg
MASS_DRONE    = 0.027  # kg (Crazyflie 2X body)
CABLE_LENGTH  = 0.5    # m
FORMATION_S   = 0.6    # m  horizontal offset radius
FORMATION_H   = 0.40   # m  vertical cable projection
F_MIN         = 0.05   # N  minimum cable tension (anti-slack)
KP            = 4.0    # PD position gain
KD            = 2.0    # PD derivative gain
DT            = 0.01   # s  simulation timestep
G             = 9.81   # m/s^2

# Nominal attachment points on load body frame (flat square, z=0)
B_ATTACH = np.array([
    [+0.15,  0.00, 0.0],
    [-0.15,  0.00, 0.0],
    [ 0.00, +0.15, 0.0],
    [ 0.00, -0.15, 0.0],
])

# Formation offsets drone_i relative to load centroid
D_OFFSETS = np.array([
    [+FORMATION_S,  0.0,          FORMATION_H],
    [-FORMATION_S,  0.0,          FORMATION_H],
    [ 0.0,         +FORMATION_S,  FORMATION_H],
    [ 0.0,         -FORMATION_S,  FORMATION_H],
])

def build_A_matrix(drone_pos, load_pos, b_attach):
    """Build 6xN equilibrium matrix A for given geometry."""
    N = len(drone_pos)
    A = np.zeros((6, N))
    for i in range(N):
        attach_world = load_pos + b_attach[i]
        diff = drone_pos[i] - attach_world
        c_hat = diff / np.linalg.norm(diff)
        A[:3, i] = c_hat
        A[3:, i] = np.cross(b_attach[i], c_hat)
    return A

def solve_tensions_pseudoinverse(A, mass_load):
    """Minimum-norm tension via Moore-Penrose pseudo-inverse."""
    b_rhs = np.array([0., 0., mass_load * G, 0., 0., 0.])
    f_star = A.T @ np.linalg.solve(A @ A.T, b_rhs)
    return f_star

def solve_tensions_qp(A, mass_load, f_min=F_MIN):
    """Anti-slack QP: minimise ||f||^2 s.t. Af=b, f>=f_min."""
    b_rhs = np.array([0., 0., mass_load * G, 0., 0., 0.])
    N = A.shape[1]
    f0 = np.ones(N) * (mass_load * G / N)

    def objective(f):
        return float(f @ f)

    def jac(f):
        return 2 * f

    constraints = [{'type': 'eq',
                    'fun': lambda f: A @ f - b_rhs,
                    'jac': lambda f: A}]
    bounds = [(f_min, None)] * N

    result = minimize(objective, f0, jac=jac,
                      constraints=constraints, bounds=bounds, method='SLSQP')
    return result.x

def pd_control(p_des, p, v_des, v):
    """Per-drone PD position controller; returns acceleration command."""
    return KP * (p_des - p) + KD * (v_des - v)
```

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Number of drones | 4 |
| Load mass $m_L$ | 0.8 kg |
| Drone body mass $m_d$ | 0.027 kg (CF2X) |
| Cable length $l_c$ | 0.5 m |
| Formation radius $s$ | 0.6 m |
| Formation height offset $h$ | 0.40 m |
| Minimum cable tension $f_{min}$ | 0.05 N |
| Delivery altitude $z_d$ | 4.0 m |
| Target position | (3.0, 2.0, 4.0) m |
| PD gains $(K_p, K_d)$ | 4.0, 2.0 |
| Simulation timestep $\Delta t$ | 0.01 s |
| Total mission time | 20 s |

---

## Expected Output

- 3D trajectory plot: four drone paths (red/orange/magenta/yellow) + load centroid path (blue) from liftoff through translation to landing
- Cable tension time series: $f_1(t), \ldots, f_4(t)$ with $f_{min}$ threshold line; verify no slack events
- Slack margin $\delta_{slack}(t)$ plot: shows safety envelope throughout mission
- Load position error vs time: $\|\mathbf{q}(t) - \mathbf{q}^{des}(t)\|$ for each mission phase (lift, cruise, descend)
- Comparison bar chart: pseudo-inverse vs QP maximum individual tension (shows QP distributes load more evenly)
- Formation geometry snapshot at $t = 0$ and $t = t_{cruise}$: top-down view showing cross pattern alignment with load centroid

---

## Extensions

1. Increase to $N = 6$ drones in hexagonal formation; observe how additional redundancy reduces per-drone tension and improves slack margin
2. Add cable elasticity: model each cable as a spring-damper $f_i = k_c(\|\mathbf{p}_i - \mathbf{q} - \mathbf{b}_i\| - l_c) + b_c \dot{\ell}_i$, examine oscillation modes
3. One-drone failure: at $t = 10$ s, drone 3 fails ($f_3 = 0$); test if the remaining three drones can rebalance via QP redistribution to prevent load drop
4. Wind disturbance: add lateral wind impulse at $t = 8$ s and measure load swing damping time constant
5. Asymmetric load: shift load CoM off-centroid by $[0.05, 0.03, 0]$ m; re-solve QP for unequal tension distribution and verify moment balance

---

## Related Scenarios

- Prerequisites: [S021](S021_point_delivery.md), [S025](S025_payload_cog_offset.md)
- Follow-ups: [S027](S027_aerial_refueling.md), [S028](S028_cargo_escort.md)
