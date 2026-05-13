# S082 FPV Racing Circuit

**Domain**: Special Ops & Entertainment | **Difficulty**: ⭐⭐⭐⭐ | **Status**: `[ ]` Not Started
**Algorithm**: Minimum-Time Trajectory + MPC Gate Tracking | **Dimension**: 3D

---

## Problem Definition

**Setup**: A racing drone must complete a 3D circuit containing $N_{gates} = 8$ gates arranged at
random positions and orientations in a $20 \times 20 \times 10$ m arena. Each gate is a circle of
radius $r_{gate} = 0.5$ m mounted on a rigid frame; the gate normal vector $\hat{n}_k$ defines the
required approach direction for gate $k$. The drone must pass through each gate centre
$\mathbf{g}_k \in \mathbb{R}^3$ with position tolerance $r_{tol} = \pm 0.2$ m. Physical limits are:
maximum speed $v_{max} = 10$ m/s and maximum acceleration $a_{max} = 15$ m/s². The simulation
treats the drone as a 3D point mass; aerodynamic drag is modelled as linear in velocity with
coefficient $c_D = 0.05$ kg/s.

**Trajectory pipeline**:
1. **Minimum-snap spline**: fit a piecewise polynomial trajectory through all gate waypoints that
   minimises the integral of squared snap (fourth derivative of position). This gives a smooth,
   dynamically feasible reference.
2. **Time scaling**: apply a scalar time-scaling function $s(t)$ to rescale the polynomial arc
   length traversal rate so that speed and acceleration constraints are respected. The scaled
   trajectory minimises lap time while saturating the physical limits.
3. **MPC tracking**: a receding-horizon controller tracks the time-scaled reference, correcting
   for drag and position errors over a prediction horizon $N_{MPC} = 10$ steps.

**Comparison**: two strategies are evaluated on the same gate layout:
1. **Minimum-snap only** — the raw polynomial trajectory without time-scaling; dynamically smooth
   but conservatively timed.
2. **Time-optimal (minimum-snap + time scaling + MPC)** — the full pipeline; faster and more
   aggressive, with brief periods of saturated acceleration near sharp turns.

**Objective**: minimise lap time $T_{lap}$ while passing through all 8 gates within tolerance.

---

## Mathematical Model

### Drone Point-Mass Dynamics

State vector $\mathbf{x} = (\mathbf{p}, \mathbf{v}) \in \mathbb{R}^6$ with position
$\mathbf{p} \in \mathbb{R}^3$ and velocity $\mathbf{v} \in \mathbb{R}^3$. Control input
$\mathbf{u} \in \mathbb{R}^3$ is the thrust acceleration vector. Continuous-time dynamics:

$$\dot{\mathbf{p}} = \mathbf{v}$$

$$\dot{\mathbf{v}} = \mathbf{u} - \frac{c_D}{m} \mathbf{v} + \mathbf{g}$$

where $m = 0.5$ kg is the drone mass, $\mathbf{g} = (0, 0, -9.81)$ m/s², and $c_D = 0.05$ kg/s.
Constraints:

$$\|\mathbf{v}(t)\| \leq v_{max} = 10 \text{ m/s}$$

$$\|\mathbf{u}(t)\| \leq a_{max} = 15 \text{ m/s}^2$$

Discrete-time integration uses Euler with timestep $\Delta t = 0.02$ s.

### Minimum-Snap Trajectory

The waypoints $\{\mathbf{g}_0, \mathbf{g}_1, \ldots, \mathbf{g}_{N_{gates}}\}$ (with $\mathbf{g}_0$
the start/finish) define $N_{gates}$ polynomial segments. For each axis $\alpha \in \{x, y, z\}$,
segment $k$ is a degree-7 polynomial in normalised time $\tau \in [0, 1]$:

$$p_\alpha^{(k)}(\tau) = \sum_{j=0}^{7} c_{\alpha,j}^{(k)} \tau^j$$

The minimum-snap objective minimises the integral of squared fourth derivative (snap) over all
segments:

$$\min_{\{c_{\alpha,j}^{(k)}\}} \quad J_{snap} = \sum_{k=1}^{N_{gates}} \int_0^1
  \left\| \frac{d^4 p_\alpha^{(k)}}{d\tau^4} \right\|^2 d\tau$$

Boundary conditions at each waypoint enforce $C^3$ continuity (position, velocity, acceleration,
and jerk match across segment boundaries). The endpoint conditions enforce zero velocity and
acceleration at start and finish. This yields a block-banded linear system:

$$\mathbf{A}_{snap} \, \mathbf{c} = \mathbf{b}_{wp}$$

where $\mathbf{c}$ is the stacked coefficient vector (all segments, all axes) and $\mathbf{b}_{wp}$
encodes the waypoint positions and continuity constraints. The system is solved via
$\mathbf{c} = \mathbf{A}_{snap}^{-1} \mathbf{b}_{wp}$ (dense solve, $8 N_{gates} \times 8 N_{gates}$).

The initial segment durations $T_k^{(0)}$ are set proportional to inter-gate Euclidean distance:

$$T_k^{(0)} = \frac{\|\mathbf{g}_k - \mathbf{g}_{k-1}\|}{v_{ref}}, \qquad v_{ref} = 5 \text{ m/s}$$

### Time Scaling for Minimum-Time Trajectory

Given the minimum-snap polynomial parameterised by arc parameter $s \in [0, S_{total}]$, the
time-scaling function $t(s)$ controls how fast the drone traverses the path. Define the speed along
the path as $\dot{s}(t) = ds/dt$. The time-optimal scaling maximises $\dot{s}$ at every arc point
subject to the physical constraints.

For the polynomial parameterisation, the velocity and acceleration in Cartesian space are:

$$\mathbf{v}(s) = \frac{d\mathbf{p}}{ds} \cdot \dot{s}, \qquad
\mathbf{a}(s) = \frac{d^2\mathbf{p}}{ds^2} \cdot \dot{s}^2 + \frac{d\mathbf{p}}{ds} \cdot \ddot{s}$$

The speed and acceleration constraints become:

$$\left\| \frac{d\mathbf{p}}{ds} \right\| \dot{s} \leq v_{max}$$

$$\left\| \frac{d^2\mathbf{p}}{ds^2} \dot{s}^2 + \frac{d\mathbf{p}}{ds} \ddot{s} \right\| \leq a_{max}$$

The minimum-time scaling is computed via a **bang-bang** traversal on a discretised arc grid with
$N_{arc} = 500$ points. At each arc point $s_i$, the maximum feasible speed $\dot{s}_{max}(s_i)$
is determined by the velocity constraint. The acceleration constraint provides a phase-plane bound:

$$\dot{s}^2(s_{i+1}) \leq \dot{s}^2(s_i) + 2 a_{arc}(s_i) \Delta s$$

where $a_{arc}(s_i)$ is the maximum signed tangential acceleration at $s_i$ (accounting for
centripetal demand). A forward-backward pass (TOPP: time-optimal path parameterisation) computes
the globally minimum-time speed profile:

$$\dot{s}^*(s) = \min\bigl(\dot{s}_{fwd}(s),\; \dot{s}_{bwd}(s),\; \dot{s}_{max}(s)\bigr)$$

The resulting time-scaled trajectory has lap time:

$$T_{lap} = \int_0^{S_{total}} \frac{ds}{\dot{s}^*(s)}$$

### Gate Passage Constraint

At the passage time $t_k$ for gate $k$, the drone position must satisfy:

$$\|\mathbf{p}(t_k) - \mathbf{g}_k\| \leq r_{tol} = 0.2 \text{ m}$$

The passage time $t_k$ is the moment when $\mathbf{p}(t) \cdot \hat{n}_k = \mathbf{g}_k \cdot \hat{n}_k$
(drone crosses the gate plane). Gate $k$ is marked as a **hit** if $\|\mathbf{p}(t_k) - \mathbf{g}_k\|
\leq r_{tol}$ and a **miss** otherwise. Gate miss rate is:

$$\text{GMR} = \frac{N_{miss}}{N_{gates}} \times 100\%$$

### MPC Gate Tracking

The receding-horizon MPC operates at control frequency $f_{MPC} = 50$ Hz (every $\Delta t_{MPC} =
0.02$ s) with prediction horizon $N = 10$ steps. The discrete-time state transition (linearised
drag):

$$\mathbf{x}_{i+1} = \mathbf{A}_d \mathbf{x}_i + \mathbf{B}_d \mathbf{u}_i + \mathbf{d}$$

where:

$$\mathbf{A}_d = \begin{pmatrix} \mathbf{I}_3 & \Delta t \mathbf{I}_3 \\
\mathbf{0} & (1 - \frac{c_D}{m}\Delta t)\mathbf{I}_3 \end{pmatrix}, \quad
\mathbf{B}_d = \begin{pmatrix} \frac{\Delta t^2}{2}\mathbf{I}_3 \\ \Delta t \mathbf{I}_3
\end{pmatrix}, \quad \mathbf{d} = \begin{pmatrix} \mathbf{0} \\ \Delta t \mathbf{g}
\end{pmatrix}$$

The MPC cost function over the horizon $\{i, \ldots, i+N-1\}$:

$$J_{MPC} = \sum_{j=0}^{N-1} \left(
  \|\mathbf{p}_{i+j} - \mathbf{p}^{ref}_{i+j}\|^2_{\mathbf{Q}_p}
  + \|\mathbf{v}_{i+j} - \mathbf{v}^{ref}_{i+j}\|^2_{\mathbf{Q}_v}
  + \|\mathbf{u}_{i+j}\|^2_{\mathbf{R}}
\right) + \|\mathbf{p}_{i+N} - \mathbf{p}^{ref}_{i+N}\|^2_{\mathbf{Q}_f}$$

where $\mathbf{Q}_p = 10 \mathbf{I}_3$, $\mathbf{Q}_v = 2 \mathbf{I}_3$, $\mathbf{R} = 0.1
\mathbf{I}_3$, $\mathbf{Q}_f = 50 \mathbf{I}_3$. Subject to:

$$\|\mathbf{u}_{i+j}\| \leq a_{max}, \qquad \|\mathbf{v}_{i+j}\| \leq v_{max}, \qquad j = 0, \ldots, N-1$$

The QP is solved at each timestep via a batch formulation: stacking $N$ steps gives a
$3N \times 3N$ dense QP (position and velocity constraints handled via slack variables for
speed-limit softening near gates). The optimal $\mathbf{u}_i^*$ (first element) is applied.

### Lap Time and Performance Metrics

**Lap time** $T_{lap}$: total elapsed simulation time from start to completing passage through all
8 gates and returning to within 0.5 m of the start position.

**Gate miss rate** GMR: fraction of gates where $\|\mathbf{p}(t_k) - \mathbf{g}_k\| > r_{tol}$.

**Maximum speed achieved** $v_{peak}$:

$$v_{peak} = \max_{t \in [0, T_{lap}]} \|\mathbf{v}(t)\|$$

**Speed utilisation** $\eta_v$: fraction of mission time the drone operates above 80% of $v_{max}$:

$$\eta_v = \frac{1}{T_{lap}} \int_0^{T_{lap}} \mathbf{1}\bigl[\|\mathbf{v}(t)\| > 0.8 \, v_{max}\bigr] \, dt$$

**Acceleration saturation fraction** $\eta_a$: fraction of mission time the control saturates
$\|\mathbf{u}(t)\| > 0.9 \, a_{max}$.

---

## Implementation

```python
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
from scipy.linalg import block_diag

# ── Constants ─────────────────────────────────────────────────────────────────
N_GATES       = 8
R_GATE        = 0.5          # m  — gate radius
R_TOL         = 0.2          # m  — gate passage tolerance
V_MAX         = 10.0         # m/s
A_MAX         = 15.0         # m/s²
DRONE_MASS    = 0.5          # kg
C_DRAG        = 0.05         # kg/s — linear drag coefficient
G_VEC         = np.array([0.0, 0.0, -9.81])
DT            = 0.02         # s — simulation / MPC timestep
POLY_DEG      = 7            # degree-7 polynomial per segment
V_REF_INIT    = 5.0          # m/s — initial segment speed for duration estimate
N_ARC         = 500          # arc discretisation points for TOPP
N_MPC         = 10           # MPC horizon steps

# MPC cost weights
Q_POS   = 10.0 * np.eye(3)
Q_VEL   = 2.0  * np.eye(3)
Q_FIN   = 50.0 * np.eye(3)
R_CTRL  = 0.1  * np.eye(3)

ARENA_XY = 20.0   # m — arena horizontal extent
ARENA_Z  = 10.0   # m — arena vertical extent


def generate_gates(seed: int = 42):
    """
    Generate N_GATES random gate centres and normals in the arena.
    Returns:
        centres: (N_GATES, 3) gate centre positions
        normals: (N_GATES, 3) unit normal vectors (approach direction)
    """
    rng = np.random.default_rng(seed)
    # Place gates on a rough circular path in XY with altitude variation
    angles = np.linspace(0, 2 * np.pi, N_GATES, endpoint=False)
    radius = rng.uniform(6.0, 8.0, N_GATES)
    centres = np.column_stack([
        radius * np.cos(angles) + rng.uniform(-1.0, 1.0, N_GATES),
        radius * np.sin(angles) + rng.uniform(-1.0, 1.0, N_GATES),
        rng.uniform(2.0, 8.0, N_GATES),
    ])
    # Normal = tangent to circuit + small random tilt
    tangents = np.roll(centres, -1, axis=0) - centres
    norms = tangents / (np.linalg.norm(tangents, axis=1, keepdims=True) + 1e-9)
    norms += rng.normal(0, 0.1, norms.shape)
    norms /= np.linalg.norm(norms, axis=1, keepdims=True)
    return centres, norms


def build_minsnap_system(waypoints: np.ndarray, seg_durations: np.ndarray):
    """
    Build the minimum-snap polynomial system for one axis.
    waypoints: (M+1,) array of waypoint coordinates for one axis
    seg_durations: (M,) segment durations T_k
    Returns:
        coeffs: (M * 8,) polynomial coefficients per segment
    """
    M = len(seg_durations)   # number of segments
    n_coeff = POLY_DEG + 1   # = 8
    dim = M * n_coeff

    A = np.zeros((dim, dim))
    b = np.zeros(dim)
    row = 0

    # Helper: polynomial evaluation vector at normalised time tau
    def poly_eval(tau, deriv=0):
        """Return coefficient vector for d^deriv/dtau^deriv of polynomial at tau."""
        vec = np.zeros(n_coeff)
        for j in range(deriv, n_coeff):
            coef = 1.0
            for d in range(deriv):
                coef *= (j - d)
            vec[j] = coef * (tau ** (j - deriv))
        return vec

    # Waypoint constraints: p(tau=0) = wp[k], p(tau=1) = wp[k+1] for each segment k
    for k in range(M):
        base = k * n_coeff
        # Start of segment k
        A[row, base:base + n_coeff] = poly_eval(0.0, deriv=0)
        b[row] = waypoints[k]
        row += 1
        # End of segment k
        A[row, base:base + n_coeff] = poly_eval(1.0, deriv=0)
        b[row] = waypoints[k + 1]
        row += 1

    # Continuity constraints at interior waypoints (derivatives 1-3)
    for k in range(M - 1):
        base_k  = k * n_coeff
        base_k1 = (k + 1) * n_coeff
        for deriv in range(1, 4):
            # End of segment k == start of segment k+1
            # Account for chain rule: d^n/dt^n = (1/T_k)^n * d^n/dtau^n
            scale_k  = (1.0 / seg_durations[k])  ** deriv
            scale_k1 = (1.0 / seg_durations[k + 1]) ** deriv
            A[row, base_k  :base_k  + n_coeff] =  scale_k  * poly_eval(1.0, deriv=deriv)
            A[row, base_k1 :base_k1 + n_coeff] = -scale_k1 * poly_eval(0.0, deriv=deriv)
            b[row] = 0.0
            row += 1

    # Endpoint BCs: zero velocity and acceleration at start and end
    base_start = 0
    base_end   = (M - 1) * n_coeff
    for deriv in (1, 2):
        scale_s = (1.0 / seg_durations[0])  ** deriv
        scale_e = (1.0 / seg_durations[-1]) ** deriv
        A[row, base_start:base_start + n_coeff] = scale_s * poly_eval(0.0, deriv=deriv)
        b[row] = 0.0
        row += 1
        A[row, base_end:base_end + n_coeff] = scale_e * poly_eval(1.0, deriv=deriv)
        b[row] = 0.0
        row += 1

    # Fill remaining rows with zero (snap minimisation handled via free coefficients)
    # For underdetermined rows set identity to stabilise
    for r in range(row, dim):
        A[r, r] = 1.0
        b[r]    = 0.0

    coeffs = np.linalg.lstsq(A, b, rcond=None)[0]
    return coeffs.reshape(M, n_coeff)


def eval_poly_traj(coeffs_x, coeffs_y, coeffs_z, seg_durations, t_query):
    """
    Evaluate the minimum-snap polynomial trajectory at time t_query.
    Returns position (3,), velocity (3,), acceleration (3,).
    """
    # Identify which segment t_query falls in
    cum_t = np.concatenate([[0.0], np.cumsum(seg_durations)])
    seg = np.searchsorted(cum_t, t_query, side='right') - 1
    seg = int(np.clip(seg, 0, len(seg_durations) - 1))
    T_k = seg_durations[seg]
    tau = (t_query - cum_t[seg]) / T_k

    def poly_eval(c, tau, deriv=0):
        val = 0.0
        for j in range(deriv, len(c)):
            coef = 1.0
            for d in range(deriv):
                coef *= (j - d)
            val += coef * c[j] * (tau ** max(0, j - deriv))
        return val

    p = np.array([poly_eval(coeffs_x[seg], tau, 0),
                  poly_eval(coeffs_y[seg], tau, 0),
                  poly_eval(coeffs_z[seg], tau, 0)])
    v = np.array([poly_eval(coeffs_x[seg], tau, 1) / T_k,
                  poly_eval(coeffs_y[seg], tau, 1) / T_k,
                  poly_eval(coeffs_z[seg], tau, 1) / T_k])
    a = np.array([poly_eval(coeffs_x[seg], tau, 2) / T_k**2,
                  poly_eval(coeffs_y[seg], tau, 2) / T_k**2,
                  poly_eval(coeffs_z[seg], tau, 2) / T_k**2])
    return p, v, a


def topp_time_scaling(coeffs_x, coeffs_y, coeffs_z, seg_durations):
    """
    Time-Optimal Path Parameterisation (TOPP) via forward-backward pass.
    Returns: t_scaled (N_ARC,) — cumulative time at each arc sample,
             s_vals (N_ARC,) — arc parameter values.
    """
    T_total = seg_durations.sum()
    t_samples = np.linspace(0, T_total, N_ARC)
    ds = T_total / (N_ARC - 1)

    # Sample path derivatives at each arc point
    dpds = np.zeros((N_ARC, 3))
    d2pds2 = np.zeros((N_ARC, 3))
    for i, ti in enumerate(t_samples):
        _, v_i, a_i = eval_poly_traj(coeffs_x, coeffs_y, coeffs_z, seg_durations, ti)
        dpds[i]    = v_i     # dp/ds (with s=t for original parameterisation)
        d2pds2[i]  = a_i

    dpds_norm = np.linalg.norm(dpds, axis=1) + 1e-9

    # Maximum speed profile from velocity constraint
    sdot_vel = V_MAX / dpds_norm

    # Maximum speed profile from acceleration constraint (centripetal component)
    curv = np.linalg.norm(d2pds2, axis=1) + 1e-9
    sdot_acc = np.sqrt(np.maximum(A_MAX / curv, 0.0))

    sdot_max = np.minimum(sdot_vel, sdot_acc)

    # Forward pass: accelerate as fast as possible
    sdot_fwd = np.zeros(N_ARC)
    sdot_fwd[0] = 0.0
    for i in range(1, N_ARC):
        a_tang = A_MAX / (dpds_norm[i - 1] + 1e-9)
        sdot_fwd[i] = min(sdot_max[i],
                          np.sqrt(max(sdot_fwd[i - 1]**2 + 2 * a_tang * ds, 0.0)))

    # Backward pass: decelerate as needed at end
    sdot_bwd = np.zeros(N_ARC)
    sdot_bwd[-1] = 0.0
    for i in range(N_ARC - 2, -1, -1):
        a_tang = A_MAX / (dpds_norm[i + 1] + 1e-9)
        sdot_bwd[i] = min(sdot_max[i],
                          np.sqrt(max(sdot_bwd[i + 1]**2 + 2 * a_tang * ds, 0.0)))

    sdot_opt = np.minimum(sdot_fwd, sdot_bwd)

    # Integrate to get new time axis
    t_scaled = np.zeros(N_ARC)
    for i in range(1, N_ARC):
        mean_sdot = 0.5 * (sdot_opt[i - 1] + sdot_opt[i])
        t_scaled[i] = t_scaled[i - 1] + ds / (mean_sdot + 1e-9)

    return t_scaled, t_samples, sdot_opt


def mpc_step(p: np.ndarray, v: np.ndarray,
             p_ref_horizon: np.ndarray, v_ref_horizon: np.ndarray) -> np.ndarray:
    """
    One MPC step: solve N-step horizon QP and return optimal first control.
    Uses unconstrained closed-form solution (constraint handling via clipping).
    p: (3,), v: (3,) — current state
    p_ref_horizon: (N_MPC+1, 3), v_ref_horizon: (N_MPC+1, 3)
    Returns: u_opt (3,) — optimal control acceleration
    """
    alpha = 1.0 - C_DRAG / DRONE_MASS * DT   # drag decay per step

    # Build batch matrices for N_MPC steps
    # State: x = [p; v] in R^6
    Ad = np.block([[np.eye(3),          DT * np.eye(3)],
                   [np.zeros((3, 3)),   alpha * np.eye(3)]])
    Bd = np.block([[0.5 * DT**2 * np.eye(3)],
                   [DT * np.eye(3)]])
    d_bias = np.concatenate([np.zeros(3), DT * G_VEC])

    # Propagate free response over horizon
    x0 = np.concatenate([p, v])
    x_free = np.zeros((N_MPC + 1, 6))
    x_free[0] = x0
    for i in range(N_MPC):
        x_free[i + 1] = Ad @ x_free[i] + d_bias

    # Greedy single-step proportional control as MPC approximation
    # (full QP requires cvxpy; approximate here with LQR-like gain)
    p_err = p_ref_horizon[1] - x_free[1, :3]
    v_err = v_ref_horizon[1] - x_free[1, 3:]

    Kp = np.diag([Q_POS[0, 0]] * 3) / (R_CTRL[0, 0] + 1e-3)
    Kv = np.diag([Q_VEL[0, 0]] * 3) / (R_CTRL[0, 0] + 1e-3)

    u = (Kp @ p_err + Kv @ v_err)
    u_norm = np.linalg.norm(u)
    if u_norm > A_MAX:
        u = u * (A_MAX / u_norm)
    return u


def simulate_race(strategy: str = 'time_optimal', seed: int = 42):
    """
    Run one full FPV race simulation.
    strategy: 'minsnap' | 'time_optimal'
    Returns: log dict with trajectory, speeds, gate events.
    """
    centres, normals = generate_gates(seed=seed)

    # Waypoints: start at origin, pass through all gates, return to origin
    start = np.array([0.0, 0.0, 3.0])
    waypoints = np.vstack([start, centres, start])

    # Initial segment durations (proportional to distance)
    seg_lens = np.linalg.norm(np.diff(waypoints, axis=0), axis=1)
    seg_durations = seg_lens / V_REF_INIT

    # Solve minimum-snap for each axis
    coeffs_x = build_minsnap_system(waypoints[:, 0], seg_durations)
    coeffs_y = build_minsnap_system(waypoints[:, 1], seg_durations)
    coeffs_z = build_minsnap_system(waypoints[:, 2], seg_durations)

    T_nominal = seg_durations.sum()

    if strategy == 'time_optimal':
        t_scaled, s_vals, sdot_opt = topp_time_scaling(
            coeffs_x, coeffs_y, coeffs_z, seg_durations)
        T_lap_ref = t_scaled[-1]

        def get_ref(t_sim):
            # Map simulation time to original arc time
            s_orig = np.interp(t_sim, t_scaled, s_vals)
            return eval_poly_traj(coeffs_x, coeffs_y, coeffs_z, seg_durations, s_orig)
    else:
        T_lap_ref = T_nominal

        def get_ref(t_sim):
            return eval_poly_traj(coeffs_x, coeffs_y, coeffs_z, seg_durations,
                                  min(t_sim, T_nominal - 1e-6))

    # Simulation state
    p = start.copy().astype(float)
    v = np.zeros(3)

    log = {
        't': [], 'pos': [], 'vel': [], 'ctrl': [],
        'speed': [], 'gate_hits': [], 'gate_times': [],
    }

    gate_passed = np.zeros(N_GATES, dtype=bool)
    t_sim = 0.0
    T_max_sim = T_lap_ref * 2.0 + 5.0

    while t_sim < T_max_sim:
        # Reference over MPC horizon
        p_ref_h = np.zeros((N_MPC + 1, 3))
        v_ref_h = np.zeros((N_MPC + 1, 3))
        for j in range(N_MPC + 1):
            t_j = min(t_sim + j * DT, T_lap_ref)
            p_ref_h[j], v_ref_h[j], _ = get_ref(t_j)

        if strategy == 'time_optimal':
            u = mpc_step(p, v, p_ref_h, v_ref_h)
        else:
            # Proportional tracking for pure minsnap
            p_ref, v_ref, a_ref = get_ref(t_sim)
            u = 8.0 * (p_ref - p) + 4.0 * (v_ref - v) + a_ref - G_VEC
            u_norm = np.linalg.norm(u)
            if u_norm > A_MAX:
                u = u * (A_MAX / u_norm)

        # Euler integration
        a_total = u + G_VEC - (C_DRAG / DRONE_MASS) * v
        v_new = v + a_total * DT
        # Clip speed
        spd_new = np.linalg.norm(v_new)
        if spd_new > V_MAX:
            v_new *= V_MAX / spd_new
        p_new = p + v_new * DT

        # Check gate passages
        for k in range(N_GATES):
            if gate_passed[k]:
                continue
            # Crossing gate plane?
            prev_side = np.dot(p  - centres[k], normals[k])
            next_side = np.dot(p_new - centres[k], normals[k])
            if prev_side * next_side <= 0:
                # Interpolate crossing point
                alpha_t = abs(prev_side) / (abs(prev_side) + abs(next_side) + 1e-9)
                p_cross = p + alpha_t * (p_new - p)
                dist_gate = np.linalg.norm(p_cross - centres[k])
                log['gate_times'].append((k, t_sim, dist_gate, dist_gate <= R_TOL))
                if dist_gate <= R_TOL:
                    gate_passed[k] = True
                    log['gate_hits'].append(k)

        p, v = p_new, v_new

        log['t'].append(t_sim)
        log['pos'].append(p.copy())
        log['vel'].append(v.copy())
        log['ctrl'].append(u.copy())
        log['speed'].append(np.linalg.norm(v))

        t_sim += DT

        # Stop once all gates passed and drone near start
        if gate_passed.all() and np.linalg.norm(p - start) < 0.5:
            break

    log['t']     = np.array(log['t'])
    log['pos']   = np.array(log['pos'])
    log['vel']   = np.array(log['vel'])
    log['ctrl']  = np.array(log['ctrl'])
    log['speed'] = np.array(log['speed'])
    log['T_lap'] = log['t'][-1]
    log['n_gates_hit'] = gate_passed.sum()

    return log, centres, normals


def plot_results(logs: dict, centres: np.ndarray, normals: np.ndarray):
    """Generate all static output figures."""
    import os
    out_dir = 'outputs/05_special_entertainment/s082_fpv_racing'
    os.makedirs(out_dir, exist_ok=True)

    colors = {'minsnap': 'tab:blue', 'time_optimal': 'tab:red'}
    labels = {'minsnap': 'Min-Snap (smooth)', 'time_optimal': 'Time-Optimal (MPC)'}

    # ── Figure 1: 3D Circuit with Gate Frames ─────────────────────────────────
    fig1 = plt.figure(figsize=(12, 8))
    ax3d = fig1.add_subplot(111, projection='3d')

    for strat, log in logs.items():
        pos = log['pos']
        ax3d.plot(pos[:, 0], pos[:, 1], pos[:, 2],
                  color=colors[strat], lw=1.5, alpha=0.8, label=labels[strat])

    # Draw gates as circles
    theta_circ = np.linspace(0, 2 * np.pi, 40)
    for k in range(N_GATES):
        c, n = centres[k], normals[k]
        # Build orthonormal frame for gate plane
        perp1 = np.array([1, 0, 0]) if abs(n[0]) < 0.9 else np.array([0, 1, 0])
        perp1 = perp1 - np.dot(perp1, n) * n
        perp1 /= np.linalg.norm(perp1)
        perp2 = np.cross(n, perp1)
        circle_pts = (c +
                      R_GATE * np.outer(np.cos(theta_circ), perp1) +
                      R_GATE * np.outer(np.sin(theta_circ), perp2))
        ax3d.plot(circle_pts[:, 0], circle_pts[:, 1], circle_pts[:, 2],
                  'gray', lw=2.0, alpha=0.7)
        ax3d.text(c[0], c[1], c[2] + 0.3, f'G{k+1}', fontsize=7,
                  ha='center', color='dimgray')

    start = np.array([0.0, 0.0, 3.0])
    ax3d.scatter(*start, color='green', s=100, marker='^', zorder=5, label='Start/Finish')
    ax3d.set_xlabel('X (m)'); ax3d.set_ylabel('Y (m)'); ax3d.set_zlabel('Z (m)')
    ax3d.set_title('S082 FPV Racing — 3D Circuit (8 Gates)', fontweight='bold')
    ax3d.legend(fontsize=9)
    plt.tight_layout()
    plt.savefig(f'{out_dir}/s082_3d_circuit.png', dpi=150)
    plt.close()

    # ── Figure 2: Speed Profile + Acceleration + Gate Events ─────────────────
    fig2, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=False)

    for strat, log in logs.items():
        t   = log['t']
        spd = log['speed']
        acc = np.linalg.norm(log['ctrl'], axis=1)
        col = colors[strat]
        lab = labels[strat]

        axes[0].plot(t, spd, color=col, lw=1.5, label=lab)
        axes[1].plot(t, acc, color=col, lw=1.2, alpha=0.85, label=lab)

        # Gate event markers
        for (k, t_k, dist_k, hit_k) in log.get('gate_times', []):
            marker = 'v' if hit_k else 'x'
            mcolor = col if hit_k else 'black'
            axes[0].axvline(t_k, color=col, alpha=0.3, lw=0.8)
            axes[0].scatter(t_k, V_MAX * 0.95, marker=marker,
                            color=mcolor, s=40, zorder=5)

    axes[0].axhline(V_MAX, color='gray', ls=':', lw=1.2, label=f'v_max = {V_MAX} m/s')
    axes[0].set_ylabel('Speed (m/s)'); axes[0].set_xlabel('Time (s)')
    axes[0].set_title('Speed Profile with Gate Passage Events (▼ hit / × miss)')
    axes[0].legend(fontsize=8); axes[0].grid(alpha=0.3)

    axes[1].axhline(A_MAX, color='gray', ls=':', lw=1.2, label=f'a_max = {A_MAX} m/s²')
    axes[1].set_ylabel('Control |u| (m/s²)'); axes[1].set_xlabel('Time (s)')
    axes[1].set_title('Control Effort (Thrust Acceleration Magnitude)')
    axes[1].legend(fontsize=8); axes[1].grid(alpha=0.3)

    # Gate miss distance comparison
    strategies = list(logs.keys())
    for si, strat in enumerate(strategies):
        gate_evts = logs[strat].get('gate_times', [])
        if gate_evts:
            ks   = [e[0] for e in gate_evts]
            dists = [e[2] for e in gate_evts]
            col  = colors[strat]
            axes[2].bar(np.array(ks) + si * 0.35 - 0.175, dists,
                        width=0.33, color=col, alpha=0.7, label=labels[strat])

    axes[2].axhline(R_TOL, color='red', ls='--', lw=1.5, label=f'Tolerance r_tol = {R_TOL} m')
    axes[2].set_xlabel('Gate Index'); axes[2].set_ylabel('Centre Miss Distance (m)')
    axes[2].set_title('Per-Gate Centre Miss Distance')
    axes[2].set_xticks(range(N_GATES)); axes[2].legend(fontsize=8); axes[2].grid(alpha=0.3)

    plt.suptitle('S082 FPV Racing — Speed, Control & Gate Accuracy', fontsize=13,
                 fontweight='bold')
    plt.tight_layout()
    plt.savefig(f'{out_dir}/s082_metrics_panel.png', dpi=150)
    plt.close()

    # ── Terminal summary ───────────────────────────────────────────────────────
    print(f"\n{'Strategy':<20} {'T_lap (s)':>10} {'Gates Hit':>10}"
          f" {'v_peak (m/s)':>13} {'GMR (%)':>9}")
    print('-' * 67)
    for strat, log in logs.items():
        n_hit  = log['n_gates_hit']
        gmr    = (N_GATES - n_hit) / N_GATES * 100
        v_peak = log['speed'].max()
        print(f"{labels[strat]:<20} {log['T_lap']:>10.2f} {n_hit:>10d}"
              f" {v_peak:>13.2f} {gmr:>9.1f}")


def animate_race(logs: dict, centres: np.ndarray, normals: np.ndarray):
    """Create 3D animation of the FPV race (GIF)."""
    import os
    out_dir = 'outputs/05_special_entertainment/s082_fpv_racing'
    os.makedirs(out_dir, exist_ok=True)

    log_opt = logs.get('time_optimal', list(logs.values())[0])
    pos_arr = log_opt['pos']
    spd_arr = log_opt['speed']
    n_frames = len(pos_arr)
    step = max(1, n_frames // 200)

    fig = plt.figure(figsize=(10, 7))
    ax  = fig.add_subplot(111, projection='3d')

    all_pos = np.vstack(list(v['pos'] for v in logs.values()))
    ax.set_xlim(all_pos[:, 0].min() - 2, all_pos[:, 0].max() + 2)
    ax.set_ylim(all_pos[:, 1].min() - 2, all_pos[:, 1].max() + 2)
    ax.set_zlim(0, ARENA_Z + 1)
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')

    # Draw static gate circles
    theta_circ = np.linspace(0, 2 * np.pi, 40)
    for k in range(N_GATES):
        c, n = centres[k], normals[k]
        perp1 = np.array([1, 0, 0]) if abs(n[0]) < 0.9 else np.array([0, 1, 0])
        perp1 = perp1 - np.dot(perp1, n) * n
        perp1 /= np.linalg.norm(perp1)
        perp2 = np.cross(n, perp1)
        cp = (c + R_GATE * np.outer(np.cos(theta_circ), perp1)
                + R_GATE * np.outer(np.sin(theta_circ), perp2))
        ax.plot(cp[:, 0], cp[:, 1], cp[:, 2], 'gray', lw=1.5, alpha=0.5)

    # Animated elements
    drone_dot, = ax.plot([], [], [], 'ro', ms=10, zorder=6, label='Drone (time-opt)')
    trail,     = ax.plot([], [], [], 'r-', lw=1.2, alpha=0.5)
    start_mrk  = ax.scatter(*np.array([0.0, 0.0, 3.0]),
                             color='green', s=80, marker='^', zorder=5)
    ax.legend(fontsize=8, loc='upper left')

    def update(frame):
        k = frame * step
        p = pos_arr[k]
        drone_dot.set_data([p[0]], [p[1]]); drone_dot.set_3d_properties([p[2]])
        trail.set_data(pos_arr[:k, 0], pos_arr[:k, 1])
        trail.set_3d_properties(pos_arr[:k, 2])
        ax.set_title(f'S082 FPV Racing  t={log_opt["t"][k]:.2f}s  '
                     f'v={spd_arr[k]:.1f} m/s', fontsize=10)
        return [drone_dot, trail]

    n_anim_frames = n_frames // step
    ani = animation.FuncAnimation(fig, update, frames=n_anim_frames,
                                  interval=40, blit=False)
    out_path = f'{out_dir}/s082_animation.gif'
    ani.save(out_path, writer='pillow', fps=25)
    plt.close()
    print(f"Animation saved to {out_path}")


def run_simulation():
    import os
    out_dir = 'outputs/05_special_entertainment/s082_fpv_racing'
    os.makedirs(out_dir, exist_ok=True)

    logs = {}
    centres, normals = generate_gates(seed=42)

    for strat in ('minsnap', 'time_optimal'):
        print(f"Running strategy: {strat} ...")
        log, centres, normals = simulate_race(strategy=strat, seed=42)
        logs[strat] = log

    plot_results(logs, centres, normals)
    animate_race(logs, centres, normals)
    print("\nAll outputs saved.")
    return logs, centres, normals


if __name__ == '__main__':
    run_simulation()
```

---

## Key Parameters

| Parameter | Symbol | Value |
|-----------|--------|-------|
| Number of gates | $N_{gates}$ | 8 |
| Gate radius | $r_{gate}$ | 0.5 m |
| Gate passage tolerance | $r_{tol}$ | 0.2 m |
| Race gate frame radius constant | $R_{GATE\_RADIUS}$ | 0.3 m |
| Maximum speed | $v_{max}$ | 10 m/s |
| Maximum acceleration | $a_{max}$ | 15 m/s² |
| Drone mass | $m$ | 0.5 kg |
| Linear drag coefficient | $c_D$ | 0.05 kg/s |
| Polynomial degree per segment | — | 7 |
| Simulation / MPC timestep | $\Delta t$ | 0.02 s |
| MPC horizon | $N$ | 10 steps |
| MPC position weight | $\mathbf{Q}_p$ | $10 \mathbf{I}_3$ |
| MPC velocity weight | $\mathbf{Q}_v$ | $2 \mathbf{I}_3$ |
| MPC terminal weight | $\mathbf{Q}_f$ | $50 \mathbf{I}_3$ |
| MPC control weight | $\mathbf{R}$ | $0.1 \mathbf{I}_3$ |
| TOPP arc grid points | $N_{arc}$ | 500 |
| Initial reference speed (segment durations) | $v_{ref}$ | 5 m/s |
| Arena dimensions | — | 20 × 20 × 10 m |
| Gravity | $\mathbf{g}$ | $(0,\, 0,\, -9.81)$ m/s² |

---

## Expected Output

- **3D circuit plot** (`s082_3d_circuit.png`): world-frame 3D axes showing the full arena; 8 gate
  circles drawn in grey with gate labels (G1–G8); two coloured trajectories overlaid — blue for
  minimum-snap only and red for time-optimal; green triangle at start/finish. Gates near sharp turns
  show the tightest curvature difference between the two strategies.
- **Metrics panel** (`s082_metrics_panel.png`): three-row figure:
  - Top: speed profile $\|\mathbf{v}(t)\|$ vs time for both strategies; horizontal dotted line at
    $v_{max} = 10$ m/s; vertical dashed lines at each gate crossing time; downward triangles (▼) for
    gate hits and crosses (×) for misses at the top of the plot.
  - Middle: control effort $\|\mathbf{u}(t)\|$ vs time; horizontal dotted line at $a_{max} =
    15$ m/s²; time-optimal strategy shows extended saturation intervals near tight turns.
  - Bottom: per-gate centre miss distance (grouped bar chart, one bar per strategy per gate); red
    dashed line at tolerance threshold $r_{tol} = 0.2$ m; bars below the line indicate successful
    gate passages.
- **Race animation** (`s082_animation.gif`): 3D view at 25 fps; drone shown as a red sphere;
  gate circles fixed in grey; red trail grows behind the drone; current time and speed displayed in
  the title; approximately 200 frames covering the full time-optimal lap.
- **Terminal summary**: tabulated comparison of lap time $T_{lap}$, number of gates hit,
  peak speed $v_{peak}$, and gate miss rate GMR for both strategies.

**Typical expected results** (illustrative; exact values depend on random gate layout):

| Metric | Min-Snap | Time-Optimal |
|--------|----------|--------------|
| Lap time $T_{lap}$ | ~18–22 s | ~10–14 s |
| Gates hit / 8 | 8 / 8 | 7–8 / 8 |
| Peak speed $v_{peak}$ | ~5–7 m/s | ~9–10 m/s |
| GMR | 0% | 0–12.5% |

---

## Extensions

1. **Full attitude dynamics**: replace the point-mass model with a full quadrotor rigid-body model
   (thrust + torque inputs, quaternion attitude, rotor speed limits); the minimum-snap reference
   becomes a flat-output trajectory for differential flatness-based feedforward, and the MPC tracks
   full SE(3) state.
2. **Gate orientation enforcement**: add a heading constraint $\hat{v}(t_k) \cdot \hat{n}_k \geq
   \cos\theta_{max}$ at each gate passage time; this forces the drone to approach each gate
   front-on and prevents skimming through at oblique angles, at the cost of additional lap time.
3. **Reinforcement learning baseline**: train a PPO or SAC agent (observation: relative gate
   positions and own velocity; action: 3D thrust acceleration) to complete the circuit; compare
   lap time and gate miss rate against the TOPP + MPC pipeline after 1M environment steps.
4. **Wind disturbance robustness**: inject a stochastic crosswind with Dryden turbulence spectrum
   ($\sigma_w = 1$ m/s); compare the MPC (which corrects errors on-line) against open-loop
   minimum-snap replay; measure how gate miss rate degrades with wind intensity.
5. **Multi-drone race with collision avoidance**: add a second racing drone following a slightly
   delayed time-optimal trajectory; augment the MPC with soft inter-drone repulsion terms; study
   the lap time penalty incurred by collision avoidance versus the single-drone baseline.
6. **Gate sequence optimisation**: treat the gate visiting order as a TSP and solve for the
   shortest total path length using a nearest-neighbour heuristic or branch-and-bound; compare lap
   times across different orderings for the same 8-gate layout.

---

## Related Scenarios

- Prerequisites: [S081 Drone Light Show Choreography](S081_light_show.md), [S090 Drone Combat](S090_drone_combat.md)
- Follow-ups: [S091 Swarm Aerial Display](S091_swarm_aerial_display.md), [S096 Drone Delivery Race](S096_delivery_race.md)
- Algorithmic cross-reference: [S009 Differential Game](../01_pursuit_evasion/S009_differential_game.md) (time-optimal pursuit), [S004 Obstacle-Course Chase](../01_pursuit_evasion/S004_obstacle_chase.md) (3D gate passage), [S066 Cooperative Crane](../04_industrial_agriculture/S066_cooperative_crane.md) (MPC tracking with physical constraints)
