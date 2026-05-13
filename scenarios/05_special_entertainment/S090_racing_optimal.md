# S090 Racing Drone Optimal Path

**Domain**: Special Ops & Entertainment | **Difficulty**: ⭐⭐⭐⭐ | **Status**: `[ ]` Not Started
**Algorithm**: Minimum-Snap QP + Time Allocation Optimisation | **Dimension**: 3D

---

## Problem Definition

**Setup**: A racing drone must plan a smooth, minimum-snap polynomial trajectory through $N_{gates} = 6$
gates arranged in 3D space. Each gate is defined by a centre position
$\mathbf{g}_i \in \mathbb{R}^3$ and a unit heading vector $\hat{\mathbf{h}}_i$ that specifies the
direction the drone must be travelling when it passes through. The drone must pass through each gate
centre within a positional tolerance $r_{tol} = 0.2$ m (gate radius $r_{gate} = 0.3$ m). The
trajectory is parameterised as a sequence of $N_{gates}$ piecewise degree-7 polynomials — one
segment per inter-gate interval — and must satisfy $C^3$ continuity (position, velocity,
acceleration, and jerk are continuous) at every intermediate waypoint.

The total trajectory time $T_{total} = \sum_{i=1}^{N_{gates}} T_i$ is minimised subject to the snap
cost remaining bounded, where $T_i$ is the time allocated to segment $i$. Time allocation is
initialised uniformly (proportional to Euclidean inter-gate distance) and then refined by gradient
descent on the total snap cost $J$. The simulation compares two strategies:

1. **Fixed-time minimum-snap** — segment durations fixed at the initial uniform allocation; QP
   solved once; baseline reference.
2. **Time-optimised minimum-snap** — gradient descent iteratively adjusts $T_i$ to minimise $J$
   while keeping $T_{total}$ constant (time re-distribution) or to jointly minimise $J + \lambda
   T_{total}$ (total-time reduction); the proposed method.

**Roles**:
- **Drone**: single racing agent; 3D point-mass with quadrotor dynamics; maximum speed
  $v_{max} = 12$ m/s; maximum thrust $a_{max} = 2g$.
- **Gates**: $N_{gates} = 6$ fixed rectangular frames; each defines a required passage position and
  heading constraint; gate radius $r_{gate} = 0.3$ m.

**Objective**: Generate a trajectory that passes through all 6 gates in order, minimises total snap
cost $J$ (a proxy for rotor wear and battery consumption), and achieves the shortest feasible lap
time. Report gate miss count, total lap time, and snap cost ratio (optimised vs fixed-time).

---

## Mathematical Model

### Polynomial Trajectory Parameterisation

The trajectory for segment $i$ (connecting gate $i-1$ to gate $i$) is a degree-7 polynomial per
spatial axis. For axis $\alpha \in \{x, y, z\}$:

$$p_i^\alpha(t) = \sum_{k=0}^{7} c_{ik}^\alpha \, t^k, \quad t \in [0, T_i]$$

Stacking all coefficients for segment $i$ into a single vector
$\mathbf{c}_i \in \mathbb{R}^{24}$ (8 coefficients $\times$ 3 axes), the full trajectory is
described by $\mathbf{c} = [\mathbf{c}_1^\top, \ldots, \mathbf{c}_{N_{gates}}^\top]^\top \in
\mathbb{R}^{24 N_{gates}}$.

### Snap Cost Matrix

The snap (fourth derivative of position) cost for segment $i$ along one axis is:

$$J_i = \int_0^{T_i} \left(\frac{d^4 p_i}{dt^4}\right)^2 dt = \mathbf{c}_i^\top \mathbf{Q}(T_i) \, \mathbf{c}_i$$

where $\mathbf{Q}(T_i) \in \mathbb{R}^{8 \times 8}$ is the snap cost matrix. Its entries are:

$$Q_{jk}(T) = \begin{cases}
\dfrac{j!\;k!}{(j-4)!\,(k-4)!} \cdot \dfrac{T^{j+k-7}}{j+k-7}, & j \geq 4,\; k \geq 4 \\[6pt]
0, & \text{otherwise}
\end{cases}$$

The total snap cost summed over all segments and axes is:

$$J = \sum_{i=1}^{N_{gates}} \sum_{\alpha \in \{x,y,z\}} \mathbf{c}_i^{\alpha\top} \mathbf{Q}(T_i) \, \mathbf{c}_i^\alpha$$

### Boundary Conditions and Continuity Constraints

Each segment must satisfy 8 boundary conditions (position, velocity, acceleration, and jerk at both
endpoints), giving a boundary-condition matrix $\mathbf{A}_{bc}(T_i) \in \mathbb{R}^{8 \times 8}$:

$$\mathbf{A}_{bc}(T) = \begin{bmatrix}
1 & 0 & 0 & \cdots & 0 \\
0 & 1 & 0 & \cdots & 0 \\
0 & 0 & 2 & \cdots & 0 \\
0 & 0 & 0 & 6 & \cdots \\
T^0 & T^1 & T^2 & \cdots & T^7 \\
0 & 1 & 2T & \cdots & 7T^6 \\
0 & 0 & 2 & \cdots & 42T^5 \\
0 & 0 & 0 & 6 & \cdots & 210T^4
\end{bmatrix}$$

The **gate position constraint** for gate $i$ (segment endpoint) is:

$$p_i^\alpha(T_i) = g_i^\alpha, \quad \alpha \in \{x, y, z\}$$

The **gate heading constraint** aligns the velocity direction at the endpoint with the required gate
normal:

$$\dot{\mathbf{p}}_i(T_i) = v_{gate} \, \hat{\mathbf{h}}_i$$

where $v_{gate}$ is a prescribed gate passage speed (set to $v_{gate} = 5$ m/s for all gates).

**$C^3$ continuity** at each intermediate gate $i$ ($1 \leq i < N_{gates}$) requires that the
derivatives up to order 3 match between consecutive segments:

$$\frac{d^r p_i^\alpha}{dt^r}\bigg|_{t=T_i} = \frac{d^r p_{i+1}^\alpha}{dt^r}\bigg|_{t=0},
\quad r = 0, 1, 2, 3, \quad \alpha \in \{x, y, z\}$$

### Closed-Form Solution for Fixed Time Allocation

For a fixed set of segment times $\{T_i\}$, the minimum-snap trajectory is the unique solution to
the constrained linear system obtained from the derivative-matching conditions. Per axis $\alpha$,
the global coefficient vector is found by solving:

$$\mathbf{M}(\{T_i\}) \, \mathbf{c}^\alpha = \mathbf{b}^\alpha$$

where $\mathbf{M} \in \mathbb{R}^{8 N_{gates} \times 8 N_{gates}}$ encodes the boundary conditions
and continuity constraints block-diagonally, and $\mathbf{b}^\alpha$ stacks the waypoint values and
zero-derivative boundary conditions. For a single segment this reduces to:

$$\mathbf{c}_i^\alpha = \mathbf{A}_{bc}(T_i)^{-1} \, \mathbf{b}_i^\alpha$$

### Time Allocation Gradient

Given the optimal coefficients $\mathbf{c}^*(T_i)$ for a fixed time allocation, the gradient of the
snap cost with respect to segment duration $T_i$ is obtained by implicit differentiation:

$$\frac{\partial J}{\partial T_i} = \mathbf{c}_i^{\alpha\top} \frac{\partial \mathbf{Q}(T_i)}{\partial T_i} \mathbf{c}_i^\alpha
+ 2 \mathbf{c}_i^{\alpha\top} \mathbf{Q}(T_i) \frac{\partial \mathbf{c}_i^\alpha}{\partial T_i}$$

The dominant contribution comes from the explicit $T_i$ dependence of $\mathbf{Q}(T_i)$:

$$\frac{\partial Q_{jk}}{\partial T} = \frac{j!\;k!}{(j-4)!\,(k-4)!} \cdot T^{j+k-8}, \quad j,k \geq 4$$

Time re-distribution gradient descent maintains $T_{total} = \mathrm{const}$ by projecting the
gradient onto the constraint manifold $\sum_i T_i = T_{total}$:

$$\nabla^{\perp}_i J = \frac{\partial J}{\partial T_i} - \frac{1}{N_{gates}}\sum_{j=1}^{N_{gates}} \frac{\partial J}{\partial T_j}$$

The update rule with step size $\eta$ is:

$$T_i \leftarrow T_i - \eta \, \nabla^{\perp}_i J, \qquad T_i \leftarrow \max(T_{min}, T_i)$$

where $T_{min} = 0.2$ s prevents degenerate zero-duration segments.

### Total-Time Optimisation

When joint minimisation of snap cost and total time is desired, the augmented objective is:

$$J_{aug} = J + \lambda \, T_{total} = \sum_i \mathbf{c}_i^\top \mathbf{Q}(T_i) \mathbf{c}_i + \lambda \sum_i T_i$$

The gradient $\partial J_{aug} / \partial T_i = \partial J / \partial T_i + \lambda$ drives all
segments shorter until the snap cost gradient balances the time penalty. The trade-off weight
$\lambda$ controls the Pareto front between lap time and snap cost.

### Gate Miss Distance

At the time the trajectory passes through gate $i$ (at $t = \sum_{k=1}^{i} T_k$ from the start),
the positional miss distance is:

$$e_i^{gate} = \|\mathbf{p}(\textstyle\sum_{k=1}^{i} T_k) - \mathbf{g}_i\|$$

A gate is considered **passed** if $e_i^{gate} \leq r_{tol} = 0.2$ m and the velocity direction
satisfies:

$$\frac{\dot{\mathbf{p}} \cdot \hat{\mathbf{h}}_i}{\|\dot{\mathbf{p}}\|} \geq \cos(20°) \approx 0.940$$

---

## Implementation

```python
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
from scipy.linalg import block_diag

# ── Domain constants ──────────────────────────────────────────────────────────
RACE_GATE_RADIUS = 0.3      # m  physical gate half-width
R_TOL            = 0.2      # m  positional passage tolerance

# ── Scenario parameters ───────────────────────────────────────────────────────
N_GATES       = 6           # number of gates in the course
POLY_DEG      = 7           # polynomial degree (degree-7 = minimum-snap)
N_COEFF       = POLY_DEG + 1  # = 8 coefficients per axis per segment
V_GATE        = 5.0         # m/s  required speed at each gate
V_MAX         = 12.0        # m/s  drone speed limit
A_MAX         = 2 * 9.81    # m/s² drone acceleration limit
T_MIN         = 0.2         # s  minimum allowed segment duration
ETA           = 1e-3        # gradient-descent step size
N_ITER        = 200         # gradient-descent iterations
LAMBDA        = 0.5         # snap-vs-time trade-off weight
DT_EVAL       = 0.02        # s  trajectory evaluation timestep
SEED          = 42

# ── Gate layout (positions + heading unit vectors) ────────────────────────────
GATE_POSITIONS = np.array([
    [ 5.0,  0.0, 1.5],
    [ 8.0,  4.0, 2.5],
    [ 5.0,  8.0, 1.8],
    [ 0.0,  6.0, 2.2],
    [-3.0,  2.0, 1.5],
    [ 0.0,  0.0, 1.0],   # finish = near start for lap
])  # shape (N_GATES, 3)

_raw_headings = np.array([
    [ 0.5,  1.0,  0.1],
    [-0.5,  1.0,  0.2],
    [-1.0, -0.2,  0.0],
    [-0.8, -0.8, -0.1],
    [ 0.5, -0.8,  0.0],
    [ 1.0,  0.3,  0.0],
])
GATE_HEADINGS = _raw_headings / np.linalg.norm(_raw_headings, axis=1, keepdims=True)


def snap_cost_matrix(T: float) -> np.ndarray:
    """
    Build the 8x8 snap (4th-derivative squared) cost matrix Q(T)
    for a degree-7 polynomial segment of duration T.
    """
    Q = np.zeros((N_COEFF, N_COEFF))
    for j in range(4, N_COEFF):
        for k in range(4, N_COEFF):
            coeff = (np.math.factorial(j) / np.math.factorial(j - 4) *
                     np.math.factorial(k) / np.math.factorial(k - 4))
            exp = j + k - 7
            Q[j, k] = coeff * T**exp / exp
    return Q


def dQ_dT(T: float) -> np.ndarray:
    """Derivative of snap cost matrix Q(T) with respect to T."""
    dQ = np.zeros((N_COEFF, N_COEFF))
    for j in range(4, N_COEFF):
        for k in range(4, N_COEFF):
            coeff = (np.math.factorial(j) / np.math.factorial(j - 4) *
                     np.math.factorial(k) / np.math.factorial(k - 4))
            exp = j + k - 7
            dQ[j, k] = coeff * T**(exp - 1)
    return dQ


def boundary_matrix(T: float) -> np.ndarray:
    """
    Build 8x8 boundary-condition matrix A_bc(T) for a degree-7 polynomial.
    Rows: [p(0), v(0), a(0), j(0), p(T), v(T), a(T), j(T)]
    Columns: coefficients c_0 ... c_7.
    """
    A = np.zeros((8, 8))
    # t=0 rows
    A[0, 0] = 1.0                                       # p(0)
    A[1, 1] = 1.0                                       # v(0)
    A[2, 2] = 2.0                                       # a(0)
    A[3, 3] = 6.0                                       # j(0)
    # t=T rows
    for i in range(8):
        A[4, i] = T**i                                  # p(T)
    for i in range(1, 8):
        A[5, i] = i * T**(i - 1)                       # v(T)
    for i in range(2, 8):
        A[6, i] = i * (i - 1) * T**(i - 2)            # a(T)
    for i in range(3, 8):
        A[7, i] = i * (i - 1) * (i - 2) * T**(i - 3) # j(T)
    return A


def solve_segment(p_start, v_start, a_start, j_start,
                  p_end,   v_end,   a_end,   j_end, T: float):
    """
    Solve for degree-7 polynomial coefficients (3 axes) given full
    position/velocity/acceleration/jerk boundary conditions.
    Returns coeffs array of shape (3, 8).
    """
    A = boundary_matrix(T)
    coeffs = np.zeros((3, N_COEFF))
    for ax in range(3):
        b = np.array([p_start[ax], v_start[ax], a_start[ax], j_start[ax],
                      p_end[ax],   v_end[ax],   a_end[ax],   j_end[ax]])
        coeffs[ax] = np.linalg.solve(A, b)
    return coeffs  # (3, 8)


def eval_segment(coeffs, t_vals):
    """
    Evaluate polynomial position, velocity, acceleration at array of times.
    coeffs: (3, 8)
    Returns pos (len, 3), vel (len, 3), acc (len, 3).
    """
    n = len(t_vals)
    pos = np.zeros((n, 3))
    vel = np.zeros((n, 3))
    acc = np.zeros((n, 3))
    for ax in range(3):
        c = coeffs[ax]
        for i, t in enumerate(t_vals):
            pos[i, ax] = sum(c[k] * t**k for k in range(8))
            vel[i, ax] = sum(k * c[k] * t**(k-1) for k in range(1, 8))
            acc[i, ax] = sum(k*(k-1) * c[k] * t**(k-2) for k in range(2, 8))
    return pos, vel, acc


def segment_snap_cost(coeffs, T):
    """Total snap cost for one segment (sum over 3 axes)."""
    Q = snap_cost_matrix(T)
    return float(sum(coeffs[ax] @ Q @ coeffs[ax] for ax in range(3)))


def plan_trajectory(times, start_pos=None, verbose=False):
    """
    Plan minimum-snap trajectory through N_GATES gates given segment times array.
    Returns list of (coeffs, T) per segment and total snap cost.
    start_pos: if None, start 2 m behind gate 0 along reversed heading.
    """
    if start_pos is None:
        start_pos = GATE_POSITIONS[0] - 2.0 * GATE_HEADINGS[0]

    segments = []
    total_J = 0.0

    # State at start of each segment: [pos, vel, acc, jerk]
    p_cur = start_pos.copy()
    v_cur = V_GATE * GATE_HEADINGS[0]   # approach first gate on-heading
    a_cur = np.zeros(3)
    j_cur = np.zeros(3)

    for i in range(N_GATES):
        Ti = times[i]
        p_end = GATE_POSITIONS[i]
        v_end = V_GATE * GATE_HEADINGS[i]
        a_end = np.zeros(3)
        j_end = np.zeros(3)

        coeffs = solve_segment(p_cur, v_cur, a_cur, j_cur,
                               p_end,  v_end,  a_end,  j_end, Ti)
        J_i = segment_snap_cost(coeffs, Ti)
        total_J += J_i
        segments.append((coeffs, Ti))

        if verbose:
            pos_T, vel_T, _ = eval_segment(coeffs, np.array([Ti]))
            miss = np.linalg.norm(pos_T[0] - p_end)
            print(f"  Gate {i+1}: T={Ti:.3f}s  J_i={J_i:.2e}  miss={miss:.4f}m")

        # Next segment starts where this one ends
        p_cur, v_cur, a_cur, j_cur = p_end.copy(), v_end.copy(), a_end.copy(), j_end.copy()

    return segments, total_J


def compute_gradient(times):
    """
    Compute dJ/dT_i for each segment using finite differences
    (analytic gradient via dQ/dT for reference in the docstring).
    Returns gradient array of shape (N_GATES,).
    """
    J0 = plan_trajectory(times)[1]
    grad = np.zeros(N_GATES)
    eps = 1e-4
    for i in range(N_GATES):
        T_pert = times.copy()
        T_pert[i] += eps
        J_pert = plan_trajectory(T_pert)[1]
        grad[i] = (J_pert - J0) / eps
    return grad


def optimise_times(times_init, mode='redistribute', n_iter=N_ITER, eta=ETA, lam=LAMBDA):
    """
    Gradient-descent optimisation of segment time allocation.
    mode: 'redistribute'  — keep T_total fixed, redistribute among segments
          'reduce'        — minimise J + lambda * T_total jointly
    Returns optimised times array and history of (J, T_total).
    """
    times = times_init.copy()
    T_total = times.sum()
    history = []

    for it in range(n_iter):
        grad = compute_gradient(times)
        _, J = plan_trajectory(times)
        history.append((J, times.sum()))

        if mode == 'redistribute':
            grad_proj = grad - grad.mean()   # project onto sum-constant manifold
            times = times - eta * grad_proj
        else:  # 'reduce'
            times = times - eta * (grad + lam)

        times = np.maximum(times, T_MIN)

        if mode == 'redistribute':
            # Re-normalise to keep T_total constant
            times = times / times.sum() * T_total

    return times, history


def evaluate_trajectory(segments):
    """
    Sample full trajectory and compute per-gate miss distances.
    Returns full_pos (N, 3), full_vel (N, 3), gate_misses (N_GATES,).
    """
    all_pos, all_vel = [], []
    gate_misses = []

    for i, (coeffs, Ti) in enumerate(segments):
        t_vals = np.arange(0.0, Ti, DT_EVAL)
        pos, vel, _ = eval_segment(coeffs, t_vals)
        all_pos.append(pos)
        all_vel.append(vel)

        # Miss at segment endpoint
        p_end, v_end, _ = eval_segment(coeffs, np.array([Ti]))
        miss = np.linalg.norm(p_end[0] - GATE_POSITIONS[i])
        gate_misses.append(miss)

    return (np.vstack(all_pos), np.vstack(all_vel), np.array(gate_misses))


def plot_results(seg_fixed, seg_opt, times_fixed, times_opt, hist_fixed, hist_opt):
    """Generate analysis figures: 3D trajectories, snap cost, gate miss, time bars."""
    pos_f, vel_f, miss_f = evaluate_trajectory(seg_fixed)
    pos_o, vel_o, miss_o = evaluate_trajectory(seg_opt)

    fig = plt.figure(figsize=(18, 12))

    # ── Plot 1: 3D Trajectory Comparison ─────────────────────────────────────
    ax1 = fig.add_subplot(2, 3, 1, projection='3d')
    ax1.plot(pos_f[:, 0], pos_f[:, 1], pos_f[:, 2],
             'r-', linewidth=1.5, alpha=0.8, label='Fixed-time')
    ax1.plot(pos_o[:, 0], pos_o[:, 1], pos_o[:, 2],
             'b-', linewidth=1.5, alpha=0.8, label='Optimised-time')
    ax1.scatter(GATE_POSITIONS[:, 0], GATE_POSITIONS[:, 1], GATE_POSITIONS[:, 2],
                c='green', s=120, marker='D', zorder=6, label='Gates')
    for i, (gp, gh) in enumerate(zip(GATE_POSITIONS, GATE_HEADINGS)):
        ax1.quiver(gp[0], gp[1], gp[2],
                   gh[0] * 0.5, gh[1] * 0.5, gh[2] * 0.5,
                   color='green', linewidth=1.0, alpha=0.7)
        ax1.text(gp[0], gp[1], gp[2] + 0.3, f'G{i+1}', fontsize=7, color='darkgreen')
    ax1.set_xlabel('x (m)')
    ax1.set_ylabel('y (m)')
    ax1.set_zlabel('z (m)')
    ax1.set_title('3D Racing Trajectory — Fixed vs Optimised Time')
    ax1.legend(fontsize=8)

    # ── Plot 2: Segment Time Allocation ──────────────────────────────────────
    ax2 = fig.add_subplot(2, 3, 2)
    x_idx = np.arange(1, N_GATES + 1)
    ax2.bar(x_idx - 0.2, times_fixed, width=0.4, color='red',
            alpha=0.75, label=f'Fixed  $T_{{total}}$={times_fixed.sum():.2f} s')
    ax2.bar(x_idx + 0.2, times_opt,   width=0.4, color='steelblue',
            alpha=0.75, label=f'Optimised  $T_{{total}}$={times_opt.sum():.2f} s')
    ax2.set_xlabel('Gate segment index')
    ax2.set_ylabel('Segment duration $T_i$ (s)')
    ax2.set_title('Time Allocation per Segment')
    ax2.legend(fontsize=8)

    # ── Plot 3: Gate Miss Distance ────────────────────────────────────────────
    ax3 = fig.add_subplot(2, 3, 3)
    ax3.bar(x_idx - 0.2, miss_f, width=0.4, color='red',
            alpha=0.75, label='Fixed-time')
    ax3.bar(x_idx + 0.2, miss_o, width=0.4, color='steelblue',
            alpha=0.75, label='Optimised-time')
    ax3.axhline(R_TOL, color='k', linestyle='--', linewidth=1.5,
                label=f'$r_{{tol}}$ = {R_TOL} m')
    ax3.axhline(RACE_GATE_RADIUS, color='gray', linestyle=':', linewidth=1.2,
                label=f'$r_{{gate}}$ = {RACE_GATE_RADIUS} m')
    ax3.set_xlabel('Gate index')
    ax3.set_ylabel('Miss distance (m)')
    ax3.set_title('Gate Passage Miss Distance')
    ax3.legend(fontsize=8)

    # ── Plot 4: Snap Cost Convergence ─────────────────────────────────────────
    ax4 = fig.add_subplot(2, 3, 4)
    if hist_opt:
        J_hist = [h[0] for h in hist_opt]
        ax4.semilogy(J_hist, 'b-', linewidth=1.5, label='Optimised (redistrib.)')
    J_fixed = plan_trajectory(times_fixed)[1]
    ax4.axhline(J_fixed, color='r', linestyle='--', linewidth=1.5,
                label=f'Fixed-time $J$={J_fixed:.3e}')
    ax4.set_xlabel('Gradient-descent iteration')
    ax4.set_ylabel('Snap cost $J$ (m$^4$/s$^7$ · s)')
    ax4.set_title('Snap Cost Convergence')
    ax4.legend(fontsize=8)

    # ── Plot 5: Speed Profile ─────────────────────────────────────────────────
    ax5 = fig.add_subplot(2, 3, 5)
    speed_f = np.linalg.norm(vel_f, axis=1)
    speed_o = np.linalg.norm(vel_o, axis=1)
    t_f = np.linspace(0, times_fixed.sum(), len(speed_f))
    t_o = np.linspace(0, times_opt.sum(),   len(speed_o))
    ax5.plot(t_f, speed_f, 'r-', linewidth=1.2, alpha=0.8, label='Fixed-time')
    ax5.plot(t_o, speed_o, 'b-', linewidth=1.2, alpha=0.8, label='Optimised-time')
    ax5.axhline(V_MAX, color='k', linestyle=':', linewidth=1.2,
                label=f'$v_{{max}}$ = {V_MAX} m/s')
    # Mark gate passages
    t_gates_f = np.cumsum(times_fixed)
    for tg in t_gates_f:
        ax5.axvline(tg, color='red', linewidth=0.6, alpha=0.4)
    ax5.set_xlabel('Time (s)')
    ax5.set_ylabel('Speed (m/s)')
    ax5.set_title('Speed Profile Along Trajectory')
    ax5.legend(fontsize=8)

    # ── Plot 6: Summary Metrics Bar ───────────────────────────────────────────
    ax6 = fig.add_subplot(2, 3, 6)
    _, J_f = plan_trajectory(times_fixed)
    _, J_o = plan_trajectory(times_opt)
    labels   = ['Fixed-time', 'Optimised']
    snap_vals = [J_f, J_o]
    bars = ax6.bar([0, 1], snap_vals, width=0.5,
                   color=['red', 'steelblue'], alpha=0.8)
    for bar, val in zip(bars, snap_vals):
        ax6.text(bar.get_x() + bar.get_width() / 2, val * 1.02,
                 f'{val:.3e}', ha='center', va='bottom', fontsize=8)
    ax6.set_xticks([0, 1])
    ax6.set_xticklabels(labels)
    ax6.set_ylabel('Total snap cost $J$')
    ax6.set_title(f'Snap Cost Comparison\n'
                  f'Reduction: {(J_f-J_o)/J_f*100:.1f}%')

    plt.tight_layout()
    plt.savefig('outputs/05_special_entertainment/s090_racing_optimal/trajectory_analysis.png',
                dpi=150, bbox_inches='tight')
    plt.show()


def animate_race(seg_fixed, seg_opt, filename='race_trajectory.gif'):
    """
    Animate racing drone following both trajectories simultaneously in 3D.
    Fixed-time trajectory shown in red, optimised in blue.
    """
    pos_f, _, _ = evaluate_trajectory(seg_fixed)
    pos_o, _, _ = evaluate_trajectory(seg_opt)

    # Subsample to ~200 frames for GIF
    n_frames = 200
    step_f = max(1, len(pos_f) // n_frames)
    step_o = max(1, len(pos_o) // n_frames)
    pos_f_sub = pos_f[::step_f]
    pos_o_sub = pos_o[::step_o]
    n_f = len(pos_f_sub)
    n_o = len(pos_o_sub)

    fig = plt.figure(figsize=(9, 7))
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(GATE_POSITIONS[:, 0], GATE_POSITIONS[:, 1], GATE_POSITIONS[:, 2],
               c='green', s=120, marker='D', zorder=6, label='Gates')
    for i, gp in enumerate(GATE_POSITIONS):
        ax.text(gp[0], gp[1], gp[2] + 0.3, f'G{i+1}', fontsize=7, color='darkgreen')

    trail_f, = ax.plot([], [], [], 'r-', linewidth=1.2, alpha=0.7, label='Fixed-time')
    trail_o, = ax.plot([], [], [], 'b-', linewidth=1.2, alpha=0.7, label='Optimised')
    drone_f, = ax.plot([], [], [], 'ro', markersize=8)
    drone_o, = ax.plot([], [], [], 'bs', markersize=8)
    info_txt = ax.text2D(0.02, 0.95, '', transform=ax.transAxes, fontsize=9)

    ax.set_xlim(-5, 12); ax.set_ylim(-2, 11); ax.set_zlim(0, 4)
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)'); ax.set_zlabel('z (m)')
    ax.set_title('Racing Drone — Minimum-Snap Trajectory')
    ax.legend(fontsize=8, loc='upper right')

    def init():
        for artist in (trail_f, trail_o, drone_f, drone_o):
            artist.set_data([], [])
            if hasattr(artist, 'set_3d_properties'):
                artist.set_3d_properties([])
        return trail_f, trail_o, drone_f, drone_o, info_txt

    def update(frame):
        if_idx = min(frame, n_f - 1)
        io_idx = min(frame, n_o - 1)
        tf = pos_f_sub[:if_idx]
        to = pos_o_sub[:io_idx]
        trail_f.set_data(tf[:, 0], tf[:, 1]); trail_f.set_3d_properties(tf[:, 2])
        trail_o.set_data(to[:, 0], to[:, 1]); trail_o.set_3d_properties(to[:, 2])
        drone_f.set_data([pos_f_sub[if_idx, 0]], [pos_f_sub[if_idx, 1]])
        drone_f.set_3d_properties([pos_f_sub[if_idx, 2]])
        drone_o.set_data([pos_o_sub[io_idx, 0]], [pos_o_sub[io_idx, 1]])
        drone_o.set_3d_properties([pos_o_sub[io_idx, 2]])
        info_txt.set_text(f'Frame {frame}/{n_frames}  '
                          f'Red: fixed  Blue: optimised')
        return trail_f, trail_o, drone_f, drone_o, info_txt

    anim = FuncAnimation(fig, update, frames=n_frames,
                         init_func=init, blit=True, interval=50)
    anim.save(f'outputs/05_special_entertainment/s090_racing_optimal/{filename}',
              writer='pillow', fps=20)
    plt.close()


def run_simulation():
    rng = np.random.default_rng(SEED)

    # Initial time allocation proportional to inter-gate distances
    start_pos = GATE_POSITIONS[0] - 2.0 * GATE_HEADINGS[0]
    waypoints  = np.vstack([start_pos, GATE_POSITIONS])
    dists = np.linalg.norm(np.diff(waypoints, axis=0), axis=1)  # N_GATES distances
    T_total_init = 10.0   # s  initial total lap time
    times_init = dists / dists.sum() * T_total_init
    times_init = np.maximum(times_init, T_MIN)

    print('=== S090 Racing Drone Optimal Path ===')
    print(f'Initial time allocation: {times_init.round(3)}  T_total={times_init.sum():.2f} s')

    # ── Fixed-time plan ───────────────────────────────────────────────────────
    print('\n[1/2] Fixed-time minimum-snap...')
    seg_fixed, J_fixed = plan_trajectory(times_init, verbose=True)
    _, _, miss_fixed = evaluate_trajectory(seg_fixed)
    gate_pass_f = int((miss_fixed <= R_TOL).sum())
    print(f'  Fixed-time  J={J_fixed:.4e}  gates_passed={gate_pass_f}/{N_GATES}')

    # ── Time-optimised plan ───────────────────────────────────────────────────
    print('\n[2/2] Optimising time allocation (gradient descent)...')
    times_opt, hist_opt = optimise_times(
        times_init, mode='redistribute', n_iter=N_ITER, eta=ETA)
    seg_opt, J_opt = plan_trajectory(times_opt, verbose=True)
    _, _, miss_opt = evaluate_trajectory(seg_opt)
    gate_pass_o = int((miss_opt <= R_TOL).sum())
    print(f'  Optimised   J={J_opt:.4e}  gates_passed={gate_pass_o}/{N_GATES}')
    print(f'  Snap cost reduction: {(J_fixed - J_opt) / J_fixed * 100:.2f}%')
    print(f'  Lap time: fixed={times_init.sum():.2f}s  opt={times_opt.sum():.2f}s')

    # ── Visualise ─────────────────────────────────────────────────────────────
    plot_results(seg_fixed, seg_opt, times_init, times_opt, [], hist_opt)
    animate_race(seg_fixed, seg_opt, filename='race_trajectory.gif')
    print('\nDone. Outputs saved to outputs/05_special_entertainment/s090_racing_optimal/')
    return seg_fixed, seg_opt, times_init, times_opt, J_fixed, J_opt


if __name__ == '__main__':
    run_simulation()
```

---

## Key Parameters

| Parameter | Symbol | Value |
|-----------|--------|-------|
| Number of gates | $N_{gates}$ | 6 |
| Gate physical radius | $r_{gate}$ | 0.3 m |
| Gate passage tolerance | $r_{tol}$ | 0.2 m |
| Required gate passage speed | $v_{gate}$ | 5.0 m/s |
| Maximum drone speed | $v_{max}$ | 12.0 m/s |
| Maximum acceleration | $a_{max}$ | $2g \approx 19.6$ m/s² |
| Polynomial degree | — | 7 (minimum-snap) |
| Coefficients per axis per segment | $N_{coeff}$ | 8 |
| Continuity order | — | $C^3$ (pos / vel / acc / jerk) |
| Initial total lap time | $T_{total}$ | 10.0 s |
| Minimum segment duration | $T_{min}$ | 0.2 s |
| Gradient-descent step size | $\eta$ | $10^{-3}$ |
| Gradient-descent iterations | — | 200 |
| Snap-vs-time trade-off weight | $\lambda$ | 0.5 |
| Trajectory evaluation timestep | $\Delta t_{eval}$ | 0.02 s |
| Heading alignment threshold | — | $\cos 20° \approx 0.940$ |
| Random seed | — | 42 |

---

## Expected Output

- **3D trajectory comparison plot**: `mpl_toolkits.mplot3d` visualisation of the full racing path for
  both strategies (fixed-time in red, optimised in blue); gate positions marked as green diamonds
  with heading arrows; start position annotated; lap bounding box visible.
- **Segment time allocation bar chart**: side-by-side bars for each of the 6 segments showing how
  the optimiser redistributes time compared to the uniform initialisation; total lap time annotated
  in the legend.
- **Gate miss distance bar chart**: per-gate positional miss distance for both strategies; $r_{tol}
  = 0.2$ m and $r_{gate} = 0.3$ m thresholds drawn as dashed reference lines; gates with miss $>
  r_{tol}$ highlighted.
- **Snap cost convergence plot**: semi-logarithmic plot of total snap cost $J$ vs gradient-descent
  iteration for the redistribution optimiser; fixed-time baseline shown as a dashed horizontal line;
  expected reduction $\geq 15\%$.
- **Speed profile plot**: instantaneous drone speed vs time for both trajectories; $v_{max} = 12$
  m/s limit shown as a dotted line; gate passage times marked as vertical tick marks on the x-axis.
- **Summary snap cost bar chart**: total snap cost $J$ for fixed-time vs optimised trajectories with
  percentage reduction annotated.
- **Race trajectory animation (GIF)**: 3D animation of both drones flying simultaneously — red drone
  follows fixed-time plan, blue drone follows optimised plan — with growing trail; gate positions
  highlighted in green; frame counter and strategy labels overlaid.

**Expected metric targets**:

| Metric | Fixed-time | Optimised |
|--------|-----------|-----------|
| Gates passed (miss $\leq r_{tol}$) | 6 / 6 | 6 / 6 |
| Total snap cost $J$ | baseline | $\geq 15\%$ reduction |
| Total lap time $T_{total}$ | 10.0 s (fixed) | $\leq 10.0$ s (redistrib.) |
| Max segment speed | $\leq v_{max}$ | $\leq v_{max}$ |

> **Note**: The redistribution optimiser holds total lap time fixed at 10.0 s and achieves snap
> cost reduction purely by reallocating time between segments — assigning more time to dynamically
> demanding segments and less to straight inter-gate runs. The total-time reduction mode (with
> $\lambda = 0.5$) will additionally shorten the lap at the cost of higher peak snap.

---

## Extensions

1. **Jerk-limited dynamic feasibility**: add inequality constraints bounding velocity and
   acceleration along the optimised trajectory ($\|\dot{\mathbf{p}}\| \leq v_{max}$,
   $\|\ddot{\mathbf{p}}\| \leq a_{max}$); enforce these via augmented Lagrangian penalty terms
   added to the snap cost QP and re-optimise; compare feasibility certificates before and after
   enforcement.
2. **Gate-order optimisation**: treat the gate visitation order as an additional optimisation
   variable; apply a branch-and-bound search over the $6! = 720$ permutations, evaluating minimum
   snap cost for each order; identify the lap sequence that minimises total snap cost for a fixed
   lap time budget.
3. **Adaptive time allocation via SQP**: replace gradient descent with a Sequential Quadratic
   Programming (SQP) solver that jointly updates both segment times and polynomial coefficients in a
   single Newton step; compare convergence rate and final snap cost against the decoupled
   gradient-descent approach.
4. **Wind disturbance and re-planning**: introduce a stochastic horizontal wind field during the
   race; detect gate miss events online (miss $> r_{tol}$); trigger a real-time re-plan from the
   current drone state using a warm-started QP (re-using the previous solution as initial point);
   measure re-plan latency and resulting gate pass rate.
5. **Multi-lap race with tire analogue (rotor wear)**: model cumulative snap cost as a proxy for
   rotor wear; after each lap, reduce $a_{max}$ by a small amount; optimise the time allocation
   strategy across 10 laps to minimise total lap time while keeping per-lap snap cost below a
   maintenance threshold.
6. **Heterogeneous gate constraints**: allow each gate to specify an independent passage speed
   $v_{gate,i}$ and approach cone half-angle $\psi_i$ (replacing the fixed 20° threshold); solve
   the resulting more tightly constrained QP; visualise the trade-off between tight gate constraints
   and achievable snap cost.

---

## Related Scenarios

- Prerequisites: [S082 Drone Light Show Formation](S082_light_show_formation.md) (3D waypoint
  trajectory planning in the entertainment domain), [S091 FPV Acrobatic Manoeuvre](S091_fpv_acrobatic.md)
  (high-speed drone dynamics and snap-limited flight envelopes)
- Follow-ups: [S091 FPV Acrobatic Manoeuvre](S091_fpv_acrobatic.md) (extends minimum-snap to
  acrobatic flip sequences), [S099 Multi-Drone Race](S099_multi_drone_race.md) (competitive
  trajectory planning with inter-drone collision avoidance)
- Algorithmic cross-reference: [S077 Precision Pollination](../04_industrial_agriculture/S077_pollination.md)
  (minimum-snap single-segment approach trajectories with the same degree-7 polynomial formulation),
  [S021 Package Delivery](../02_logistics_delivery/S021_package_delivery.md) (waypoint trajectory
  planning with time constraints)
