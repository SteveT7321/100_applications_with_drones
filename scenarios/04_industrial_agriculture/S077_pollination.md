# S077 Precision Pollination Flight Pattern

**Domain**: Industrial & Agriculture | **Difficulty**: ⭐⭐⭐ | **Status**: `[ ]` Not Started
**Algorithm**: TSP Ordering + Minimum-Snap Approach Trajectory | **Dimension**: 3D

---

## Problem Definition

**Setup**: An orchard row occupies a $10 \times 10 \times 2$ m volume ($z \in [1, 3]$ m). A drone
equipped with a vibrating pollen applicator must visit $F = 25$ flowers whose 3D positions are drawn
uniformly at random within this volume. For each flower the drone must:

1. **Approach** from below at a $30°$ elevation angle (mimicking a bee's natural approach vector)
   along a smooth minimum-snap polynomial trajectory.
2. **Dock** — hover within $d_{dock} = 0.1$ m of the flower centre for a dwell time of
   $T_{dwell} = 1.5$ s while the applicator vibrates.
3. **Depart** vertically upward to a safe transit altitude before proceeding to the next flower.

The visit order is determined by a nearest-neighbour TSP heuristic applied to the 3D flower
positions. GPS positioning noise is modelled as $\sigma_{GPS} = 0.1$ m per axis.

**Roles**:
- **Drone**: single agent; 3D point-mass dynamics with first-order velocity response; carries pollen
  applicator; executes minimum-snap approach trajectories.
- **Flowers**: $F = 25$ stationary targets; each has a random 3D position; pollen transfer succeeds
  probabilistically based on docking error.
- **TSP planner**: nearest-neighbour greedy tour computed once at the start; provides the visit
  sequence $\pi = (\pi_1, \pi_2, \ldots, \pi_F)$.

**Objective**: Maximise the expected number of pollinated flowers $E[N_{poll}]$ while minimising
total mission time. Compare two approach strategies:

1. **Minimum-snap trajectory** — smooth polynomial approach respecting the $30°$ elevation
   constraint; lower position error at docking; reference method.
2. **Straight-line approach** — direct linear interpolation from current position to flower; faster
   per-visit but higher docking error due to velocity overshoot.

---

## Mathematical Model

### TSP Nearest-Neighbour Tour

Starting from the drone's initial position $\mathbf{p}_0$, the nearest-neighbour heuristic builds
the visit sequence greedily. Let $\mathcal{U}$ be the set of unvisited flowers. At each step:

$$\pi_{k+1} = \arg\min_{j \in \mathcal{U}} \|\mathbf{p}_{\pi_k} - \mathbf{p}_j\|$$

The total tour length is:

$$L_{tour} = \sum_{k=0}^{F-1} \|\mathbf{p}_{\pi_k} - \mathbf{p}_{\pi_{k+1}}\|$$

### Approach Direction Constraint

For each flower $i$ at position $\mathbf{p}_i^{flower}$, the approach must arrive from a direction
making a $30°$ elevation angle above the horizontal. Let $\varphi_i$ be the horizontal bearing from
the drone's transit position to the flower. The required unit approach vector is:

$$\hat{\mathbf{d}}_i = \begin{pmatrix} \sin 30° \cos\varphi_i \\ \sin 30° \sin\varphi_i \\ \cos 30° \end{pmatrix}
= \begin{pmatrix} 0.5\cos\varphi_i \\ 0.5\sin\varphi_i \\ \tfrac{\sqrt{3}}{2} \end{pmatrix}$$

The approach start point is offset from the flower by a standoff distance $d_{standoff} = 0.8$ m
along the reversed approach direction:

$$\mathbf{p}_{start}^{(i)} = \mathbf{p}_i^{flower} - d_{standoff}\,\hat{\mathbf{d}}_i$$

### Minimum-Snap Trajectory

The minimum-snap trajectory minimises the integral of the squared fourth derivative of position,
yielding smooth acceleration profiles that reduce positioning overshoot at the docking point. For a
single segment from $\mathbf{p}_A$ to $\mathbf{p}_B$ over duration $T_{seg}$, the trajectory is
represented as a degree-7 polynomial per axis:

$$p(t) = c_0 + c_1 t + c_2 t^2 + c_3 t^3 + c_4 t^4 + c_5 t^5 + c_6 t^6 + c_7 t^7, \quad t \in [0, T_{seg}]$$

The minimum-snap cost is:

$$J_{snap} = \int_0^{T_{seg}} \left(\frac{d^4 p}{dt^4}\right)^2 dt$$

This is minimised subject to boundary conditions specifying position, velocity, acceleration, and
jerk at both endpoints. Collecting the 8 coefficients $\mathbf{c} = (c_0, \ldots, c_7)^\top$, the
boundary-condition matrix $\mathbf{A}_{bc} \in \mathbb{R}^{8 \times 8}$ maps coefficients to
boundary values $\mathbf{b}_{bc}$:

$$\mathbf{A}_{bc}\,\mathbf{c} = \mathbf{b}_{bc}$$

The cost matrix $\mathbf{Q} \in \mathbb{R}^{8 \times 8}$ for the snap integral has entries:

$$Q_{ij} = \int_0^{T_{seg}} \frac{d^4 p_i}{dt^4}\cdot\frac{d^4 p_j}{dt^4}\,dt$$

where $p_i(t) = t^i$. The closed-form entry is:

$$Q_{ij} = \frac{i!\;j!}{(i-4)!\,(j-4)!}\cdot\frac{T_{seg}^{i+j-7}}{i+j-7}, \quad i,j \geq 4$$

and $Q_{ij} = 0$ if $i < 4$ or $j < 4$. The unconstrained minimum is found by solving the linear
system arising from the boundary conditions:

$$\mathbf{c} = \mathbf{A}_{bc}^{-1}\,\mathbf{b}_{bc}$$

Each spatial axis ($x$, $y$, $z$) is solved independently with the same $\mathbf{A}_{bc}$ but
different right-hand sides $\mathbf{b}_{bc}^x$, $\mathbf{b}_{bc}^y$, $\mathbf{b}_{bc}^z$.
The boundary conditions for the approach segment to flower $i$ are:

| Endpoint | Position | Velocity | Acceleration | Jerk |
|----------|----------|----------|--------------|------|
| $t = 0$ | $\mathbf{p}_{start}^{(i)}$ | $v_{approach}\,\hat{\mathbf{d}}_i$ | $\mathbf{0}$ | $\mathbf{0}$ |
| $t = T_{seg}$ | $\mathbf{p}_i^{flower}$ | $\mathbf{0}$ | $\mathbf{0}$ | $\mathbf{0}$ |

### Hover Docking Error

After completing the approach trajectory the drone hovers at the commanded flower position for
$T_{dwell} = 1.5$ s. GPS noise is added each timestep:

$$\mathbf{p}_{drone}(t) = \mathbf{p}_i^{flower} + \boldsymbol{\eta}(t), \quad
\boldsymbol{\eta}(t) \sim \mathcal{N}(\mathbf{0},\, \sigma_{GPS}^2\,\mathbf{I}_3)$$

The docking error is the mean position deviation over the dwell period:

$$e_{dock}^{(i)} = \frac{1}{N_{dwell}}\sum_{k=1}^{N_{dwell}} \|\mathbf{p}_{drone}(t_k) - \mathbf{p}_i^{flower}\|$$

### Pollen Transfer Probability

The probability of successful pollen transfer at flower $i$ decays exponentially with docking error:

$$P_t^{(i)} = \exp\!\left(-\frac{e_{dock}^{(i)}}{\sigma_{dock}}\right)$$

where $\sigma_{dock} = d_{dock} = 0.1$ m is the characteristic docking scale. The expected number of
pollinated flowers over the full tour is:

$$E[N_{poll}] = \sum_{i=1}^{F} P_t^{(i)}$$

The overall transfer success rate is:

$$\eta_{poll} = \frac{E[N_{poll}]}{F}$$

### Straight-Line Approach (Baseline)

For the straight-line baseline the drone moves at constant speed $v_{approach}$ directly from
$\mathbf{p}_{start}^{(i)}$ to $\mathbf{p}_i^{flower}$ without polynomial smoothing. Because there
is no jerk-limiting, velocity is not zeroed at the flower, resulting in a larger terminal position
error and thus a lower $P_t^{(i)}$.

---

## Implementation

```python
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

# ── Domain constants ──────────────────────────────────────────────────────────
GPS_ERROR       = 0.1    # m  per-axis GPS noise (σ_GPS)
INSPECTION_DIST = 0.5    # m  (domain reference, not used directly)

# ── Scenario parameters ───────────────────────────────────────────────────────
F_FLOWERS       = 25     # number of flowers
ORCHARD_X       = 10.0   # m  orchard width
ORCHARD_Y       = 10.0   # m  orchard depth
Z_MIN           = 1.0    # m  minimum flower height
Z_MAX           = 3.0    # m  maximum flower height
D_DOCK          = 0.1    # m  docking radius (σ_dock)
SIGMA_DOCK      = D_DOCK # m  transfer probability scale
T_DWELL         = 1.5    # s  hover dwell time at each flower
D_STANDOFF      = 0.8    # m  approach start distance from flower
ELEV_ANGLE      = 30.0   # degrees  bee-like approach elevation
V_APPROACH      = 0.5    # m/s  speed along approach segment start
V_TRANSIT       = 1.5    # m/s  transit speed between flowers
Z_TRANSIT       = 3.5    # m  safe transit altitude above orchard
DT              = 0.05   # s  simulation timestep
SEED            = 42


def tsp_nearest_neighbour(positions, start_pos):
    """
    Greedy nearest-neighbour TSP tour.
    Returns ordered list of indices into `positions`.
    """
    n = len(positions)
    unvisited = list(range(n))
    tour = []
    current = start_pos.copy()
    while unvisited:
        dists = [np.linalg.norm(current - positions[i]) for i in unvisited]
        nearest_idx = unvisited[int(np.argmin(dists))]
        tour.append(nearest_idx)
        current = positions[nearest_idx]
        unvisited.remove(nearest_idx)
    return tour


def approach_direction(flower_pos, transit_pos):
    """
    Compute unit approach vector: 30° elevation, horizontal bearing
    from transit_pos toward flower_pos.
    """
    delta_xy = flower_pos[:2] - transit_pos[:2]
    bearing = np.arctan2(delta_xy[1], delta_xy[0])
    elev_rad = np.deg2rad(ELEV_ANGLE)
    d_hat = np.array([
        np.sin(elev_rad) * np.cos(bearing),
        np.sin(elev_rad) * np.sin(bearing),
        np.cos(elev_rad),
    ])
    return d_hat


def build_boundary_matrix(T):
    """
    Build the 8×8 boundary-condition matrix A_bc for a degree-7 polynomial
    evaluated at t=0 and t=T, with position/velocity/acceleration/jerk BCs.
    Row order: [p(0), v(0), a(0), j(0), p(T), v(T), a(T), j(T)]
    """
    A = np.zeros((8, 8))
    # p(0): coefficients 1, 0, 0, ...
    A[0, 0] = 1.0
    # v(0) = c1
    A[1, 1] = 1.0
    # a(0) = 2*c2
    A[2, 2] = 2.0
    # j(0) = 6*c3
    A[3, 3] = 6.0
    # p(T) = sum c_i * T^i
    for i in range(8):
        A[4, i] = T**i
    # v(T) = sum i*c_i * T^(i-1)
    for i in range(1, 8):
        A[5, i] = i * T**(i - 1)
    # a(T) = sum i*(i-1)*c_i * T^(i-2)
    for i in range(2, 8):
        A[6, i] = i * (i - 1) * T**(i - 2)
    # j(T) = sum i*(i-1)*(i-2)*c_i * T^(i-3)
    for i in range(3, 8):
        A[7, i] = i * (i - 1) * (i - 2) * T**(i - 3)
    return A


def minimum_snap_segment(p_start, p_end, v_start, T_seg):
    """
    Solve for degree-7 polynomial coefficients (per axis) that satisfy
    boundary conditions: position and velocity at both ends,
    zero acceleration and jerk at both ends.
    Returns coefficient arrays cx, cy, cz each of shape (8,).
    """
    A_bc = build_boundary_matrix(T_seg)
    coeffs = []
    for axis in range(3):
        b = np.array([
            p_start[axis],          # p(0)
            v_start[axis],          # v(0)
            0.0,                    # a(0) = 0
            0.0,                    # j(0) = 0
            p_end[axis],            # p(T)
            0.0,                    # v(T) = 0  (arrive at rest)
            0.0,                    # a(T) = 0
            0.0,                    # j(T) = 0
        ])
        c = np.linalg.solve(A_bc, b)
        coeffs.append(c)
    return coeffs   # list of 3 arrays, each (8,)


def eval_poly(coeffs, t):
    """Evaluate polynomial position at time t given coefficient array."""
    return sum(c * t**i for i, c in enumerate(coeffs))


def simulate_approach(flower_pos, start_pos, v_start, method='min_snap', rng=None):
    """
    Simulate approach from start_pos to flower_pos.
    method: 'min_snap' or 'straight'
    Returns (trajectory array, docking_error).
    """
    if rng is None:
        rng = np.random.default_rng(SEED)

    d_hat = approach_direction(flower_pos, start_pos)
    dist = np.linalg.norm(flower_pos - start_pos)
    T_seg = max(dist / V_APPROACH, 0.5)   # at least 0.5 s

    traj = []
    if method == 'min_snap':
        coeffs = minimum_snap_segment(start_pos, flower_pos, v_start, T_seg)
        t_vals = np.arange(0.0, T_seg, DT)
        for t in t_vals:
            p = np.array([eval_poly(c, t) for c in coeffs])
            traj.append(p)
    else:
        # Straight-line: constant-speed linear interpolation
        n_steps = max(int(T_seg / DT), 1)
        for k in range(n_steps + 1):
            alpha = k / n_steps
            p = (1.0 - alpha) * start_pos + alpha * flower_pos
            traj.append(p)

    # Dwell phase: hover with GPS noise
    dwell_steps = max(int(T_DWELL / DT), 1)
    dwell_errors = []
    for _ in range(dwell_steps):
        noise = rng.normal(0.0, GPS_ERROR, size=3)
        p_hover = flower_pos + noise
        traj.append(p_hover)
        dwell_errors.append(np.linalg.norm(noise))

    e_dock = float(np.mean(dwell_errors))
    return np.array(traj), e_dock


def run_mission(method='min_snap', seed=SEED):
    """
    Run full pollination mission for one strategy.
    Returns trajectory, per-flower stats, and summary metrics.
    """
    rng = np.random.default_rng(seed)

    # Generate random flower positions
    flowers = np.column_stack([
        rng.uniform(0.0, ORCHARD_X, F_FLOWERS),
        rng.uniform(0.0, ORCHARD_Y, F_FLOWERS),
        rng.uniform(Z_MIN, Z_MAX, F_FLOWERS),
    ])

    start_pos = np.array([0.0, 0.0, Z_TRANSIT])
    tour = tsp_nearest_neighbour(flowers, start_pos)

    full_traj = [start_pos.copy()]
    dock_errors = []
    transfer_probs = []
    current_pos = start_pos.copy()

    for flower_idx in tour:
        flower_pos = flowers[flower_idx]

        # 1. Transit to approach start point
        d_hat = approach_direction(flower_pos, current_pos)
        p_approach_start = flower_pos - D_STANDOFF * d_hat

        # Move to transit altitude, then to approach start
        p_via = np.array([p_approach_start[0],
                          p_approach_start[1],
                          Z_TRANSIT])
        # Simple linear transit (not scored for docking)
        n_transit = max(int(np.linalg.norm(p_via - current_pos) / (V_TRANSIT * DT)), 2)
        for k in range(1, n_transit + 1):
            alpha = k / n_transit
            full_traj.append((1 - alpha) * current_pos + alpha * p_via)
        # Descend to approach start
        n_desc = max(int(np.linalg.norm(p_approach_start - p_via) / (V_APPROACH * DT)), 2)
        for k in range(1, n_desc + 1):
            alpha = k / n_desc
            full_traj.append((1 - alpha) * p_via + alpha * p_approach_start)
        current_pos = p_approach_start.copy()

        # 2. Approach + dwell
        v_start = V_APPROACH * d_hat
        seg_traj, e_dock = simulate_approach(
            flower_pos, current_pos, v_start, method=method, rng=rng)
        full_traj.extend(seg_traj.tolist())
        dock_errors.append(e_dock)

        p_transfer = np.exp(-e_dock / SIGMA_DOCK)
        transfer_probs.append(p_transfer)
        current_pos = flower_pos.copy()

    full_traj = np.array(full_traj)
    dock_errors = np.array(dock_errors)
    transfer_probs = np.array(transfer_probs)
    e_poll = float(transfer_probs.sum())
    success_rate = float(transfer_probs.mean())

    return {
        'trajectory': full_traj,
        'flowers': flowers,
        'tour': tour,
        'dock_errors': dock_errors,
        'transfer_probs': transfer_probs,
        'E_N_poll': e_poll,
        'success_rate': success_rate,
    }


def plot_results(res_snap, res_line):
    """Generate analysis figures comparing min-snap vs straight-line."""
    fig = plt.figure(figsize=(18, 12))

    # ── Plot 1: 3D Tour + Trajectories ───────────────────────────────────────
    ax1 = fig.add_subplot(2, 3, 1, projection='3d')
    traj_s = res_snap['trajectory']
    traj_l = res_line['trajectory']
    ax1.plot(traj_s[:, 0], traj_s[:, 1], traj_s[:, 2],
             'r-', linewidth=0.8, alpha=0.7, label='Min-snap')
    ax1.plot(traj_l[:, 0], traj_l[:, 1], traj_l[:, 2],
             'b--', linewidth=0.8, alpha=0.5, label='Straight-line')
    flowers = res_snap['flowers']
    ax1.scatter(flowers[:, 0], flowers[:, 1], flowers[:, 2],
                c='green', s=40, zorder=5, label='Flowers')
    ax1.set_xlabel('x (m)')
    ax1.set_ylabel('y (m)')
    ax1.set_zlabel('z (m)')
    ax1.set_title('3D Pollination Tour')
    ax1.legend(fontsize=8)

    # ── Plot 2: Top-down tour with visit order ────────────────────────────────
    ax2 = fig.add_subplot(2, 3, 2)
    tour = res_snap['tour']
    ax2.scatter(flowers[:, 0], flowers[:, 1], c='green', s=50, zorder=5)
    for k in range(len(tour) - 1):
        f_a = flowers[tour[k]]
        f_b = flowers[tour[k + 1]]
        ax2.annotate('', xy=(f_b[0], f_b[1]), xytext=(f_a[0], f_a[1]),
                     arrowprops=dict(arrowstyle='->', color='gray', lw=1.2))
    probs = res_snap['transfer_probs']
    sc = ax2.scatter(flowers[tour, 0], flowers[tour, 1],
                     c=probs, cmap='RdYlGn', s=100, vmin=0, vmax=1, zorder=6)
    plt.colorbar(sc, ax=ax2, label='$P_t$ (min-snap)')
    ax2.set_xlabel('x (m)')
    ax2.set_ylabel('y (m)')
    ax2.set_title('TSP Tour (colour = transfer prob.)')

    # ── Plot 3: Docking error per flower ─────────────────────────────────────
    ax3 = fig.add_subplot(2, 3, 3)
    x_idx = np.arange(1, F_FLOWERS + 1)
    ax3.bar(x_idx - 0.2, res_snap['dock_errors'], width=0.4,
            color='red', alpha=0.75, label='Min-snap')
    ax3.bar(x_idx + 0.2, res_line['dock_errors'], width=0.4,
            color='steelblue', alpha=0.75, label='Straight-line')
    ax3.axhline(D_DOCK, color='k', linestyle='--', linewidth=1.5,
                label=f'$d_{{dock}}$ = {D_DOCK} m')
    ax3.set_xlabel('Flower visit index')
    ax3.set_ylabel('Docking error (m)')
    ax3.set_title('Per-Flower Docking Error')
    ax3.legend(fontsize=8)

    # ── Plot 4: Transfer probability per flower ───────────────────────────────
    ax4 = fig.add_subplot(2, 3, 4)
    ax4.plot(x_idx, res_snap['transfer_probs'], 'ro-', markersize=4,
             label=f"Min-snap  $E[N_{{poll}}]$={res_snap['E_N_poll']:.1f}")
    ax4.plot(x_idx, res_line['transfer_probs'], 'bs--', markersize=4,
             label=f"Straight  $E[N_{{poll}}]$={res_line['E_N_poll']:.1f}")
    ax4.set_ylim(0, 1.05)
    ax4.set_xlabel('Flower visit index')
    ax4.set_ylabel('$P_t^{(i)}$')
    ax4.set_title('Transfer Probability per Flower')
    ax4.legend(fontsize=8)

    # ── Plot 5: Docking error histogram ──────────────────────────────────────
    ax5 = fig.add_subplot(2, 3, 5)
    ax5.hist(res_snap['dock_errors'], bins=10, color='red',
             alpha=0.6, label='Min-snap', density=True)
    ax5.hist(res_line['dock_errors'], bins=10, color='steelblue',
             alpha=0.6, label='Straight-line', density=True)
    ax5.axvline(D_DOCK, color='k', linestyle='--', linewidth=1.5,
                label=f'$d_{{dock}}$ = {D_DOCK} m')
    ax5.set_xlabel('Docking error (m)')
    ax5.set_ylabel('Density')
    ax5.set_title('Docking Error Distribution')
    ax5.legend(fontsize=8)

    # ── Plot 6: Summary bar chart ─────────────────────────────────────────────
    ax6 = fig.add_subplot(2, 3, 6)
    labels = ['Min-snap', 'Straight-line']
    e_polls = [res_snap['E_N_poll'], res_line['E_N_poll']]
    rates   = [res_snap['success_rate'] * 100, res_line['success_rate'] * 100]
    x_pos = np.array([0.0, 1.0])
    bars = ax6.bar(x_pos, e_polls, width=0.4, color=['red', 'steelblue'], alpha=0.8)
    ax6.axhline(F_FLOWERS, color='k', linestyle=':', linewidth=1,
                label=f'Total flowers ({F_FLOWERS})')
    for bar, ep, r in zip(bars, e_polls, rates):
        ax6.text(bar.get_x() + bar.get_width() / 2,
                 ep + 0.3,
                 f'{ep:.1f}\n({r:.0f}%)',
                 ha='center', va='bottom', fontsize=9)
    ax6.set_xticks(x_pos)
    ax6.set_xticklabels(labels)
    ax6.set_ylabel('$E[N_{poll}]$')
    ax6.set_title('Expected Pollinated Flowers')
    ax6.legend(fontsize=8)

    plt.tight_layout()
    plt.savefig('outputs/04_industrial_agriculture/s077_pollination/trajectory_analysis.png',
                dpi=150, bbox_inches='tight')
    plt.show()


def animate_pollination(res, filename='pollination_tour.gif'):
    """Animate drone following pollination tour in 3D."""
    traj = res['trajectory']
    flowers = res['flowers']
    n_frames = min(len(traj), 300)
    step = max(1, len(traj) // n_frames)

    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(flowers[:, 0], flowers[:, 1], flowers[:, 2],
               c='green', s=50, label='Flowers', zorder=5)
    ax.set_xlim(0, ORCHARD_X)
    ax.set_ylim(0, ORCHARD_Y)
    ax.set_zlim(0, Z_TRANSIT + 0.5)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_zlabel('z (m)')
    ax.set_title('Precision Pollination Tour (min-snap)')

    trail, = ax.plot([], [], [], 'r-', linewidth=1.2, alpha=0.7)
    drone_dot, = ax.plot([], [], [], 'ro', markersize=8)
    info_text = ax.text2D(0.02, 0.95, '', transform=ax.transAxes, fontsize=9)

    def init():
        trail.set_data([], [])
        trail.set_3d_properties([])
        drone_dot.set_data([], [])
        drone_dot.set_3d_properties([])
        return trail, drone_dot, info_text

    def update(frame):
        idx = frame * step
        xs = traj[:idx, 0]
        ys = traj[:idx, 1]
        zs = traj[:idx, 2]
        trail.set_data(xs, ys)
        trail.set_3d_properties(zs)
        drone_dot.set_data([traj[idx, 0]], [traj[idx, 1]])
        drone_dot.set_3d_properties([traj[idx, 2]])
        info_text.set_text(
            f"Frame {frame}/{n_frames}  "
            f"E[N_poll]={res['E_N_poll']:.1f}/{F_FLOWERS}")
        return trail, drone_dot, info_text

    anim = FuncAnimation(fig, update, frames=n_frames,
                         init_func=init, blit=True, interval=50)
    anim.save(f'outputs/04_industrial_agriculture/s077_pollination/{filename}',
              writer='pillow', fps=20)
    plt.close()


if __name__ == '__main__':
    print('Running S077 Precision Pollination Flight Pattern ...')

    res_snap = run_mission(method='min_snap', seed=SEED)
    res_line = run_mission(method='straight',  seed=SEED)

    print(f"Min-snap   : E[N_poll]={res_snap['E_N_poll']:.2f}/{F_FLOWERS}  "
          f"success_rate={res_snap['success_rate']*100:.1f}%  "
          f"mean_dock_err={res_snap['dock_errors'].mean()*100:.1f} cm")
    print(f"Straight   : E[N_poll]={res_line['E_N_poll']:.2f}/{F_FLOWERS}  "
          f"success_rate={res_line['success_rate']*100:.1f}%  "
          f"mean_dock_err={res_line['dock_errors'].mean()*100:.1f} cm")

    plot_results(res_snap, res_line)
    animate_pollination(res_snap, filename='pollination_tour.gif')
    print('Done. Outputs saved to outputs/04_industrial_agriculture/s077_pollination/')
```

---

## Key Parameters

| Parameter | Symbol | Value |
|-----------|--------|-------|
| Number of flowers | $F$ | 25 |
| Orchard footprint | — | 10 × 10 m |
| Flower height range | $z$ | 1 – 3 m |
| Docking radius / transfer scale | $d_{dock}$, $\sigma_{dock}$ | 0.1 m |
| Dwell time per flower | $T_{dwell}$ | 1.5 s |
| Approach elevation angle | $\theta_{approach}$ | 30° |
| Approach standoff distance | $d_{standoff}$ | 0.8 m |
| Approach entry speed | $v_{approach}$ | 0.5 m/s |
| Transit speed | $v_{transit}$ | 1.5 m/s |
| Safe transit altitude | $z_{transit}$ | 3.5 m |
| GPS positioning noise | $\sigma_{GPS}$ | 0.1 m |
| Polynomial degree | — | 7 (minimum-snap) |
| Transfer probability model | $P_t$ | $\exp(-e_{dock}/\sigma_{dock})$ |
| Simulation timestep | $\Delta t$ | 0.05 s |
| Random seed | — | 42 |

---

## Expected Output

- **3D tour trajectory plot**: `mpl_toolkits.mplot3d` visualisation of the full drone path for both
  strategies (min-snap in red, straight-line in blue dashed); flower positions marked in green;
  transit altitude plane visible.
- **Top-down TSP tour map**: 2D bird's-eye view of the nearest-neighbour tour with directed arrows
  between consecutive flowers; flowers colour-coded by transfer probability $P_t^{(i)}$ using a
  red-green diverging colormap.
- **Per-flower docking error bar chart**: side-by-side bars comparing docking error for each visit
  in min-snap vs straight-line; $d_{dock} = 0.1$ m threshold shown as a dashed reference line.
- **Transfer probability per flower**: line plot of $P_t^{(i)}$ across the 25 visits for both
  strategies; $E[N_{poll}]$ annotated in the legend.
- **Docking error histogram**: overlapping histograms of docking error distribution across all
  flowers for both strategies; $d_{dock}$ threshold marked.
- **Summary bar chart**: $E[N_{poll}]$ and success rate (%) for min-snap vs straight-line side by
  side; total-flowers reference line at $F = 25$.
- **Pollination tour animation (GIF)**: 3D drone trail growing frame-by-frame as the drone visits
  each flower; flowers highlighted green; current $E[N_{poll}]$ annotated.

**Expected metric targets**:

| Metric | Min-snap | Straight-line |
|--------|----------|---------------|
| Mean docking error | $< 0.12$ m | $\approx 0.15$–$0.25$ m |
| Transfer success rate $\eta_{poll}$ | $> 35\%$ | $< 25\%$ |
| $E[N_{poll}]$ | $> 9$ / 25 | $< 7$ / 25 |

> **Note**: absolute values depend on the GPS noise realisation; the key result is that min-snap
> consistently outperforms straight-line by reducing velocity overshoot at the docking point.

---

## Extensions

1. **Wind disturbance during dwell**: add a horizontal wind gust model during the $T_{dwell}$ hover;
   evaluate whether a PID position controller can maintain $e_{dock} < d_{dock}$ and re-optimise
   $T_{dwell}$ to recover the target transfer probability under gusts.
2. **Adaptive dwell time**: instead of a fixed $T_{dwell}$, terminate the dwell when a force sensor
   on the applicator detects contact with the stamen (simulated as proximity $< 0.02$ m); this
   reduces total mission time by skipping unnecessary hovering on easy approaches.
3. **Multi-segment minimum-snap**: plan the entire TSP tour as a single multi-waypoint minimum-snap
   trajectory (penalising velocity discontinuities at waypoints) rather than per-segment; compare
   tour time and total $E[N_{poll}]$ against the segmented approach.
4. **Optimal TSP with docking-constraint routing**: replace nearest-neighbour with 2-opt or
   Lin-Kernighan TSP; additionally, co-optimise the approach bearing $\varphi_i$ for each flower to
   minimise total trajectory jerk rather than choosing the horizontal bearing greedily.
5. **Pollen load depletion model**: the drone carries a finite pollen reservoir; each transfer
   consumes an amount proportional to $P_t^{(i)}$; add a recharging waypoint and re-plan the tour
   to maximise total pollen delivered subject to the reservoir constraint.
6. **Multi-drone parallel pollination**: deploy $N = 3$ drones covering separate sub-rows of a
   larger orchard; add conflict-resolution constraints to prevent mid-air proximity violations;
   compare makespan (last drone finishes) and total $E[N_{poll}]$ versus a single-drone baseline.

---

## Related Scenarios

- Prerequisites: [S063 Single-Plant Precision Spraying](S063_precision_spraying.md) (TSP tour +
  hover control over agricultural targets), [S070 Greenhouse Climate Patrol](S070_greenhouse_climate_patrol.md)
  (3D waypoint navigation in a structured indoor environment)
- Follow-ups: [S078 Fruit Size Estimation](S078_fruit_size_estimation.md) (close-range precision
  inspection with similar docking geometry)
- Algorithmic cross-reference: [S001 Basic Intercept](../01_pursuit_evasion/S001_basic_intercept.md)
  (quadrotor dynamics reference), [S021 Package Delivery](../02_logistics_delivery/S021_package_delivery.md)
  (TSP tour planning over 3D waypoints)
