# S089 Large-Scale Swarm Collision Avoidance Test

**Domain**: Special Ops & Entertainment | **Difficulty**: ⭐⭐⭐⭐ | **Status**: `[ ]` Not Started
**Algorithm**: ORCA — Optimal Reciprocal Collision Avoidance | **Dimension**: 3D

---

## Problem Definition

**Setup**: $N = 50$ drones are placed at random positions on the surface of a sphere of radius
$R_{sphere} = 20$ m centred at the origin. Each drone is assigned a target at its **antipodal**
point — the point diametrically opposite on the sphere surface. Every drone therefore flies a path
that passes through (or near) the sphere's centre, creating the worst-case traffic scenario: without
collision avoidance all 50 drones converge on the same region simultaneously and collide. The
simulation tests three strategies: (1) **No avoidance** (baseline — mass collision), (2) **Potential
field repulsion**, and (3) **ORCA** (Optimal Reciprocal Collision Avoidance). Drones move in open
3-D space; there are no static obstacles. A drone is considered to have **reached its target** when
it comes within $d_{goal} = 0.5$ m of the antipodal waypoint.

**Roles**:
- **Drones** ($N = 50$): homogeneous quadrotors modelled as point masses in 3-D; body radius
  $r = 0.25$ m (collision diameter $2r = 0.50$ m = `COLLISION_RADIUS`); maximum speed
  $v_{max} = 3.0$ m/s; preferred speed $v_{pref} = 2.0$ m/s; communicate positions and velocities
  to all drones within neighbourhood radius $R_{nb} = 10$ m.
- **Targets**: fixed points at antipodal positions; no dynamics.

**Objective**: All 50 drones reach their antipodal targets with **zero physical collisions**
(inter-drone separation never drops below `COLLISION_RADIUS = 0.5` m at any timestep). Secondary
metrics are: time-to-goal for all drones, mean velocity smoothness (acceleration magnitude averaged
over the mission), and number of ORCA constraint-active timesteps. Compare the three strategies
across these metrics.

---

## Mathematical Model

### State Space

Drone $i$ has position $\mathbf{p}_i \in \mathbb{R}^3$ and velocity $\mathbf{v}_i \in \mathbb{R}^3$.
The preferred velocity pointing toward the goal at preferred speed is:

$$\mathbf{v}_{i}^{pref} = v_{pref} \cdot \frac{\mathbf{g}_i - \mathbf{p}_i}{\|\mathbf{g}_i - \mathbf{p}_i\|}$$

where $\mathbf{g}_i = -\mathbf{p}_i^{(0)}$ is the antipodal starting position of drone $i$.

### Velocity Obstacle (VO)

The **velocity obstacle** of drone $A$ induced by drone $B$ with time horizon $\tau$ is the set of
velocities $\mathbf{v}_A$ that, if held constant, would cause a collision within time $[0, \tau]$:

$$\mathrm{VO}_{A|B}^{\tau}(\mathbf{v}_B) = \left\{ \mathbf{v}_A \;\middle|\; \exists\, t \in [0, \tau],\;
\bigl\|\bigl(\mathbf{p}_A - \mathbf{p}_B\bigr) + \bigl(\mathbf{v}_A - \mathbf{v}_B\bigr)t\bigr\| < 2r \right\}$$

Geometrically, $\mathrm{VO}_{A|B}^{\tau}(\mathbf{v}_B)$ is a truncated cone in velocity space: apex at
$\mathbf{v}_B + (\mathbf{p}_A - \mathbf{p}_B)/\tau$, half-angle
$\theta_{VO} = \arcsin\!\bigl(2r/\|\mathbf{p}_A - \mathbf{p}_B\|\bigr)$, truncated at sphere of radius
$2r/\tau$.

### Reciprocal Velocity Obstacle (RVO)

The **Reciprocal Velocity Obstacle** splits responsibility equally between agents:

$$\mathrm{RVO}_{A|B}^{\tau}(\mathbf{v}_A, \mathbf{v}_B) = \left\{ \mathbf{v}_A \;\middle|\;
2\mathbf{v}_A - \mathbf{v}_B \in \mathrm{VO}_{A|B}^{\tau}(\mathbf{v}_B) \right\}$$

The apex is shifted to the average of both current velocities:
$\frac{1}{2}(\mathbf{v}_A + \mathbf{v}_B) + \frac{\mathbf{p}_A - \mathbf{p}_B}{\tau}$,
guaranteeing each agent takes exactly half the avoidance burden.

### ORCA Half-Plane Constraint

For each pair $(A, B)$, ORCA defines the **minimum velocity change** $\mathbf{u}_{A|B}$ needed to
exit the velocity obstacle:

$$\mathbf{u}_{A|B} = \arg\min_{\mathbf{u}} \|\mathbf{u}\|
\quad \text{s.t.} \quad \mathbf{v}_A^{cur} + \mathbf{u} \notin \mathrm{VO}_{A|B}^{\tau}(\mathbf{v}_B^{cur})$$

The solution is the vector from $\mathbf{v}_A^{cur}$ to the boundary of
$\mathrm{VO}_{A|B}^{\tau}$ (closest point on the cone surface). The ORCA constraint for agent $A$
with respect to $B$ is the half-plane:

$$\mathrm{ORCA}_{A|B}^{\tau} = \left\{ \mathbf{v} \;\middle|\;
\bigl(\mathbf{v} - (\mathbf{v}_A^{cur} + \tfrac{1}{2}\mathbf{u}_{A|B})\bigr) \cdot \hat{\mathbf{n}}_{A|B} \geq 0 \right\}$$

where $\hat{\mathbf{n}}_{A|B}$ is the outward normal of the VO boundary at the closest point
(the half-plane's normal).

### Quadratic Program per Drone

At each timestep, drone $A$ solves:

$$\mathbf{v}_A^* = \arg\min_{\mathbf{v}} \;\|\mathbf{v} - \mathbf{v}_A^{pref}\|^2$$

$$\text{subject to:} \quad \mathbf{n}_{A|B}^\top \mathbf{v} \geq b_{A|B} \quad \forall B \in \mathcal{N}(A)$$

$$\|\mathbf{v}\| \leq v_{max}$$

where the half-plane parameters are:

$$\mathbf{n}_{A|B} = \hat{\mathbf{n}}_{A|B}, \qquad
b_{A|B} = \hat{\mathbf{n}}_{A|B} \cdot \bigl(\mathbf{v}_A^{cur} + \tfrac{1}{2}\mathbf{u}_{A|B}\bigr)$$

and $\mathcal{N}(A)$ is the set of neighbours within $R_{nb}$. In practice the QP is solved via the
**linear programming (LP) formulation** of van den Berg et al. (2011): constraints are processed
one by one with random permutation; an infeasible set is resolved by the 3-D minimum-penalty
relaxation (closest feasible point).

### LP-based ORCA Solver (3-D)

The iterative LP solver processes half-plane constraints $\{(\mathbf{n}_j, b_j)\}_{j=1}^{K}$:

1. Start with $\mathbf{v}^* = \mathbf{v}^{pref}$, clamped to $\|\mathbf{v}^*\| \leq v_{max}$.
2. For each constraint $j$ in random order:
   - If $\mathbf{n}_j \cdot \mathbf{v}^* \geq b_j$: constraint satisfied, continue.
   - Otherwise: project $\mathbf{v}^*$ onto the boundary half-plane $\mathbf{n}_j \cdot \mathbf{v} = b_j$,
     then find the feasible point minimising $\|\mathbf{v} - \mathbf{v}^{pref}\|^2$ within all
     previous constraints and the speed cap.
3. If the LP is infeasible (all constraints cannot be satisfied simultaneously within $v_{max}$):
   use the **3-D penalty relaxation** — find the velocity that minimises the maximum constraint
   violation weighted by $1/\|\mathbf{n}_j\|$:

$$\mathbf{v}^{relax} = \arg\min_{\mathbf{v},\, s} \; s
\quad \text{s.t.} \quad \mathbf{n}_j^\top \mathbf{v} \geq b_j - s\|\mathbf{n}_j\|, \;\; \|\mathbf{v}\| \leq v_{max}$$

### Potential Field Baseline

The repulsive potential from drone $B$ on drone $A$ (active when $d_{AB} < d_{rep} = 5.0$ m):

$$U_{rep}^{A|B} = \frac{1}{2} k_{rep} \left(\frac{1}{d_{AB}} - \frac{1}{d_{rep}}\right)^2, \quad d_{AB} < d_{rep}$$

$$\mathbf{F}_{rep}^{A|B} = -\nabla_{\mathbf{p}_A} U_{rep}^{A|B}
= k_{rep} \left(\frac{1}{d_{AB}} - \frac{1}{d_{rep}}\right) \frac{1}{d_{AB}^2}
  \frac{\mathbf{p}_A - \mathbf{p}_B}{\|\mathbf{p}_A - \mathbf{p}_B\|}$$

The attractive force toward goal $\mathbf{g}_i$:

$$\mathbf{F}_{att}^{i} = k_{att} \, (\mathbf{g}_i - \mathbf{p}_i)$$

The combined acceleration (clamped to $a_{max} = 5.0$ m/s²) drives velocity updates in the Euler
integrator. Potential field avoidance is reactive and local; it does not guarantee collision-freedom
in dense traffic.

### Collision Detection

A collision between drones $A$ and $B$ is recorded whenever:

$$\|\mathbf{p}_A(t) - \mathbf{p}_B(t)\| < 2r = 0.50 \; \text{m}$$

The minimum separation achieved is logged for each pair over the full trajectory.

### Performance Metrics

**Time-to-goal** for drone $i$:

$$T_{goal}^{(i)} = \min \{ t \geq 0 \;\mid\; \|\mathbf{p}_i(t) - \mathbf{g}_i\| \leq d_{goal} \}$$

**Mission completion time**: $T_{mission} = \max_i T_{goal}^{(i)}$.

**Velocity smoothness** (mean acceleration magnitude over the mission):

$$\bar{a} = \frac{1}{N \cdot T_{steps}} \sum_{i=1}^{N} \sum_{k=1}^{T_{steps}}
\left\|\frac{\mathbf{v}_i^{(k)} - \mathbf{v}_i^{(k-1)}}{\Delta t}\right\|$$

**ORCA constraint activity rate** for drone $i$ at timestep $k$:

$$\delta_i^{(k)} = \mathbf{1}\bigl[\mathbf{v}_i^{*(k)} \neq \mathbf{v}_i^{pref\,(k)}\bigr]$$

**Collision count**: total number of (ordered pair, timestep) triples where
$\|\mathbf{p}_A - \mathbf{p}_B\| < 2r$.

---

## Implementation

```python
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation, PillowWriter
from scipy.spatial.distance import cdist

# ── Constants ──────────────────────────────────────────────────────────────────
N_DRONES        = 50
SPHERE_RADIUS   = 20.0          # m — initial placement sphere radius
DRONE_RADIUS    = 0.25          # m — physical body radius
COLLISION_RADIUS= 2 * DRONE_RADIUS  # 0.5 m
FORMATION_DIST  = 2.0           # m — minimum desired separation
V_PREF          = 2.0           # m/s — preferred cruise speed
V_MAX           = 3.0           # m/s — speed cap
TAU             = 3.0           # s — ORCA time horizon
DT              = 0.05          # s — simulation timestep
T_MAX           = 60.0          # s — mission time limit
D_GOAL          = 0.5           # m — goal arrival radius
NB_RADIUS       = 10.0          # m — neighbour sensing radius
D_REP           = 5.0           # m — potential field repulsion activation radius
K_REP           = 5.0           # potential field repulsion gain
K_ATT           = 1.0           # potential field attraction gain
A_MAX           = 5.0           # m/s² — max acceleration (potential field)
SEED            = 42


# ── Initial positions and goals ────────────────────────────────────────────────
def init_sphere_positions(n: int, radius: float, rng: np.random.Generator):
    """
    Sample n points uniformly on a sphere surface via Gaussian normalisation.
    Returns positions (n, 3) and antipodal goals (n, 3).
    """
    raw = rng.standard_normal((n, 3))
    norms = np.linalg.norm(raw, axis=1, keepdims=True)
    positions = raw / norms * radius
    goals = -positions.copy()          # antipodal targets
    return positions.astype(float), goals.astype(float)


# ── ORCA solver ────────────────────────────────────────────────────────────────
def compute_orca_velocity(pos_a, vel_a, v_pref_a, neighbours_pos, neighbours_vel):
    """
    Compute ORCA-optimal velocity for agent A given its neighbours.

    Parameters
    ----------
    pos_a           : (3,) current position of A
    vel_a           : (3,) current velocity of A
    v_pref_a        : (3,) preferred velocity of A
    neighbours_pos  : (K, 3) positions of K neighbours
    neighbours_vel  : (K, 3) velocities of K neighbours

    Returns
    -------
    v_new : (3,) collision-free velocity
    n_active : int — number of ORCA constraints that were active
    """
    half_planes = []   # list of (n_vec, b_scalar)

    for pos_b, vel_b in zip(neighbours_pos, neighbours_vel):
        rel_pos = pos_a - pos_b          # p_A - p_B
        rel_vel = vel_a - vel_b          # v_A - v_B
        dist = np.linalg.norm(rel_pos)

        combined_radius = COLLISION_RADIUS

        if dist < 1e-6:
            continue   # coincident agents (degenerate)

        # --- Compute u (minimum velocity change to exit VO) ---
        if dist < combined_radius:
            # Already overlapping: use simple push-out along rel_pos direction
            n_vec = rel_pos / dist
            b_val = np.dot(vel_a + 0.5 * n_vec * V_MAX, n_vec)
            half_planes.append((n_vec, b_val))
            continue

        # Time-horizon cone geometry
        tau_inv = 1.0 / TAU
        w = rel_vel - tau_inv * rel_pos        # rel_vel - (p_A-p_B)/tau

        dot_ww = np.dot(w, w)
        dot_wr = np.dot(w, rel_pos)
        dot_rr = np.dot(rel_pos, rel_pos)

        # Is the relative velocity inside the VO cone?
        # Point of VO boundary closest to rel_vel:
        # Check whether the foot is on the disk (truncation) or side of cone
        rr = combined_radius
        sin_alpha = rr / dist             # half-angle sin
        cos_alpha = np.sqrt(max(1.0 - sin_alpha**2, 0.0))

        # Project w onto cone axis and find penetration
        cone_axis = -rel_pos / dist      # axis from apex pointing inward
        # t-value along axis from apex to closest circle edge
        t_proj = np.dot(w, -rel_pos) / (dist * tau_inv * dist)  # dimensionless

        # Use simple closest-point-on-cone-surface computation
        # Decompose w into parallel (along cone axis) and perp
        w_para = np.dot(w, -rel_pos / dist) * (-rel_pos / dist)
        w_perp = w - w_para
        w_perp_norm = np.linalg.norm(w_perp)

        # Tangent condition: |w_perp|/|w_para| == tan(alpha)
        # Two candidates: project onto truncation disk or onto cone side
        inside_cone = (w_perp_norm <= np.linalg.norm(w_para) * (sin_alpha / cos_alpha + 1e-9)
                       and np.dot(w, -rel_pos) >= 0)

        if not inside_cone:
            # No collision threat within tau: half-plane constraint still added
            # but with minimal velocity change (zero adjustment needed)
            continue

        # Closest point on cone: either side surface or truncation disk
        # Side surface: project w perpendicularly onto the cone surface
        if w_perp_norm < 1e-9:
            # rel_vel along axis: push out radially (degenerate)
            perp_dir = np.array([1.0, 0.0, 0.0]) \
                       if abs(rel_pos[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
            perp_dir -= perp_dir.dot(-rel_pos/dist) * (-rel_pos/dist)
            perp_dir /= np.linalg.norm(perp_dir)
        else:
            perp_dir = w_perp / w_perp_norm

        # Normal to cone surface at closest point
        n_vec_raw = cos_alpha * perp_dir + sin_alpha * (-rel_pos / dist)
        # (points outward from cone)
        n_norm = np.linalg.norm(n_vec_raw)
        if n_norm < 1e-9:
            continue
        n_vec = n_vec_raw / n_norm

        # Closest point on cone side surface to rel_vel
        u_vec = (np.dot(w, n_vec) - 0) * n_vec   # signed distance to hyperplane * normal
        # Actually: u = (projection of rel_vel onto outward cone surface) - rel_vel
        # More precisely: u = (dot(w, n_vec)) * n_vec  (minimum correction)
        u_magnitude = np.dot(w, n_vec)             # how deep inside (negative = safe)
        # u_vec is the minimum velocity change: half given to A
        u_half = 0.5 * u_magnitude * n_vec

        # ORCA half-plane: v_new · n_vec >= (v_A + u_half) · n_vec
        b_val = np.dot(vel_a + u_half, n_vec)
        half_planes.append((n_vec, b_val))

    # --- LP: find v* closest to v_pref satisfying all half-planes + speed cap ---
    v_star = v_pref_a.copy()
    # Clamp to speed cap
    speed = np.linalg.norm(v_star)
    if speed > V_MAX:
        v_star = v_star / speed * V_MAX

    n_active = 0
    for n_vec, b_val in half_planes:
        if np.dot(n_vec, v_star) < b_val:
            n_active += 1
            # Project onto half-plane boundary, then minimise distance to v_pref
            # subject to previous constraints (greedy, not globally optimal)
            # Project v_pref onto the constraint boundary
            t = (b_val - np.dot(n_vec, v_pref_a)) / (np.dot(n_vec, n_vec) + 1e-12)
            v_candidate = v_pref_a + t * n_vec
            speed_c = np.linalg.norm(v_candidate)
            if speed_c > V_MAX:
                v_candidate = v_candidate / speed_c * V_MAX
            # Check if this candidate still satisfies previous constraints
            feasible = all(
                np.dot(n2, v_candidate) >= b2 - 1e-6
                for n2, b2 in half_planes[:half_planes.index((n_vec, b_val))]
            )
            if feasible or len(half_planes) == 1:
                v_star = v_candidate
            else:
                # Fallback: stop (zero velocity) to avoid collision
                v_star = np.zeros(3)

    return v_star, n_active


# ── Potential field velocity ───────────────────────────────────────────────────
def compute_pf_velocity(pos_a, vel_a, goal_a, neighbours_pos, dt):
    """Compute velocity update via potential field (attraction + repulsion)."""
    f_att = K_ATT * (goal_a - pos_a)

    f_rep = np.zeros(3)
    for pos_b in neighbours_pos:
        diff = pos_a - pos_b
        d = np.linalg.norm(diff)
        if 0 < d < D_REP:
            magnitude = K_REP * (1.0/d - 1.0/D_REP) / (d**2)
            f_rep += magnitude * diff / d

    accel = f_att + f_rep
    a_norm = np.linalg.norm(accel)
    if a_norm > A_MAX:
        accel = accel / a_norm * A_MAX

    v_new = vel_a + accel * dt
    speed = np.linalg.norm(v_new)
    if speed > V_MAX:
        v_new = v_new / speed * V_MAX
    return v_new


# ── Simulation loop ───────────────────────────────────────────────────────────
def run_simulation(strategy: str, positions_init, goals):
    """
    Simulate N drones from sphere surface to antipodal targets.

    strategy : "orca" | "potential_field" | "none"
    Returns  : history dict with keys 'positions', 'collisions', 'arrivals',
               'n_active_constraints', 'min_separation'
    """
    rng = np.random.default_rng(SEED)
    N = len(positions_init)
    pos = positions_init.copy()   # (N, 3)
    vel = np.zeros((N, 3))        # (N, 3)
    arrived = np.zeros(N, dtype=bool)
    arrival_time = np.full(N, np.nan)

    T_steps = int(T_MAX / DT)
    pos_history = np.zeros((T_steps, N, 3))
    collision_counts = 0
    min_sep_global = np.inf
    total_active = 0
    accel_log = []

    for step in range(T_steps):
        t = step * DT
        pos_history[step] = pos.copy()

        # --- Collision detection ---
        dists = cdist(pos, pos)
        np.fill_diagonal(dists, np.inf)
        min_sep = dists.min()
        min_sep_global = min(min_sep_global, min_sep)
        col_mask = dists < COLLISION_RADIUS
        collision_counts += int(col_mask.sum() // 2)   # unordered pairs

        new_vel = vel.copy()
        step_active = 0

        for i in range(N):
            if arrived[i]:
                new_vel[i] = np.zeros(3)
                continue

            # Preferred velocity toward goal
            to_goal = goals[i] - pos[i]
            dist_goal = np.linalg.norm(to_goal)
            if dist_goal < D_GOAL:
                arrived[i] = True
                arrival_time[i] = t
                new_vel[i] = np.zeros(3)
                continue

            v_pref_i = V_PREF * to_goal / dist_goal

            # Neighbours within sensing radius
            nb_mask = (dists[i] < NB_RADIUS) & ~arrived
            nb_mask[i] = False
            nb_pos = pos[nb_mask]
            nb_vel = vel[nb_mask]

            if strategy == "orca":
                v_new_i, n_act = compute_orca_velocity(
                    pos[i], vel[i], v_pref_i, nb_pos, nb_vel)
                step_active += n_act
                new_vel[i] = v_new_i

            elif strategy == "potential_field":
                new_vel[i] = compute_pf_velocity(
                    pos[i], vel[i], goals[i], nb_pos, DT)

            else:  # no avoidance
                speed = np.linalg.norm(v_pref_i)
                new_vel[i] = v_pref_i if speed <= V_MAX else v_pref_i/speed*V_MAX

        # Velocity smoothness logging
        accel_log.append(np.linalg.norm(
            (new_vel - vel) / DT, axis=1).mean())

        vel = new_vel
        pos = pos + vel * DT
        total_active += step_active

    return {
        "positions":    pos_history,
        "collisions":   collision_counts,
        "arrivals":     arrival_time,
        "min_sep":      min_sep_global,
        "mean_accel":   float(np.nanmean(accel_log)),
        "orca_active":  total_active,
        "mission_time": float(np.nanmax(arrival_time)) if np.any(~np.isnan(arrival_time)) else T_MAX,
        "pct_arrived":  float(np.mean(~np.isnan(arrival_time))) * 100.0,
    }


# ── Visualisation ─────────────────────────────────────────────────────────────
def plot_trajectories_3d(results, goals, title, output_path):
    """3-D trajectory plot for one strategy."""
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    pos_hist = results["positions"]
    N = pos_hist.shape[1]
    cmap = plt.cm.tab20

    for i in range(N):
        color = cmap(i % 20)
        traj = pos_hist[:, i, :]
        ax.plot(traj[:, 0], traj[:, 1], traj[:, 2],
                color=color, alpha=0.4, linewidth=0.6)
        ax.scatter(*traj[0], color='green', s=12, zorder=5)    # start
        ax.scatter(*goals[i], color='red', marker='*', s=30, zorder=5)  # goal

    ax.set_xlabel('X (m)');  ax.set_ylabel('Y (m)');  ax.set_zlabel('Z (m)')
    ax.set_title(title)
    ax.set_xlim(-25, 25);  ax.set_ylim(-25, 25);  ax.set_zlim(-25, 25)
    plt.tight_layout()
    plt.savefig(output_path, dpi=120)
    plt.close(fig)
    print(f"Saved: {output_path}")


def plot_metrics_comparison(results_dict, output_path):
    """4-panel bar/time comparison across strategies."""
    strategies = list(results_dict.keys())
    labels = {"orca": "ORCA", "potential_field": "Potential\nField", "none": "No\nAvoidance"}
    colors = {"orca": "#2196F3", "potential_field": "#FF9800", "none": "#F44336"}

    fig, axes = plt.subplots(1, 4, figsize=(16, 5))

    # 1. Collision count
    ax = axes[0]
    vals = [results_dict[s]["collisions"] for s in strategies]
    bars = ax.bar([labels[s] for s in strategies], vals,
                  color=[colors[s] for s in strategies], edgecolor='black')
    ax.set_title("Collision Count\n(pair·timestep)")
    ax.set_ylabel("Count")
    for bar, val in zip(bars, vals):
        ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.5,
                str(val), ha='center', va='bottom', fontsize=10, fontweight='bold')

    # 2. Mission completion time
    ax = axes[1]
    vals = [results_dict[s]["mission_time"] for s in strategies]
    ax.bar([labels[s] for s in strategies], vals,
           color=[colors[s] for s in strategies], edgecolor='black')
    ax.set_title("Mission Time (s)\n(all drones arrived)")
    ax.set_ylabel("Seconds")

    # 3. Velocity smoothness (mean acceleration)
    ax = axes[2]
    vals = [results_dict[s]["mean_accel"] for s in strategies]
    ax.bar([labels[s] for s in strategies], vals,
           color=[colors[s] for s in strategies], edgecolor='black')
    ax.set_title("Velocity Smoothness\n(mean |accel| m/s²)")
    ax.set_ylabel("m/s²")

    # 4. Minimum separation ever recorded
    ax = axes[3]
    vals = [results_dict[s]["min_sep"] for s in strategies]
    ax.bar([labels[s] for s in strategies], vals,
           color=[colors[s] for s in strategies], edgecolor='black')
    ax.axhline(COLLISION_RADIUS, color='red', linestyle='--', label='Collision threshold')
    ax.set_title("Min Separation (m)\n(global over mission)")
    ax.set_ylabel("metres")
    ax.legend(fontsize=8)

    plt.suptitle("S089 — ORCA vs Potential Field vs No Avoidance (N=50 drones)",
                 fontsize=12, fontweight='bold')
    plt.tight_layout()
    plt.savefig(output_path, dpi=120)
    plt.close(fig)
    print(f"Saved: {output_path}")


def plot_separation_histogram(results_dict, output_path):
    """Minimum pairwise separation distribution for each strategy."""
    fig, axes = plt.subplots(1, 3, figsize=(15, 5), sharey=False)
    strategies = ["orca", "potential_field", "none"]
    titles = {"orca": "ORCA", "potential_field": "Potential Field", "none": "No Avoidance"}
    colors = {"orca": "#2196F3", "potential_field": "#FF9800", "none": "#F44336"}

    for ax, s in zip(axes, strategies):
        pos_hist = results_dict[s]["positions"]
        T, N, _ = pos_hist.shape
        min_seps = []
        sample_steps = range(0, T, max(1, T//100))  # sample 100 timesteps
        for step in sample_steps:
            d = cdist(pos_hist[step], pos_hist[step])
            np.fill_diagonal(d, np.inf)
            min_seps.append(d.min())

        ax.hist(min_seps, bins=30, color=colors[s], edgecolor='black', alpha=0.8)
        ax.axvline(COLLISION_RADIUS, color='red', linestyle='--', linewidth=2,
                   label=f'Collision threshold ({COLLISION_RADIUS} m)')
        ax.set_title(f"{titles[s]}\nMin Separation Distribution")
        ax.set_xlabel("Min pairwise separation (m)")
        ax.set_ylabel("Count (timestep samples)")
        ax.legend(fontsize=7)

    plt.suptitle("S089 — Pairwise Minimum Separation Histogram", fontsize=12, fontweight='bold')
    plt.tight_layout()
    plt.savefig(output_path, dpi=120)
    plt.close(fig)
    print(f"Saved: {output_path}")


def animate_orca_swarm(results, goals, output_path):
    """
    Animated top-down (XY) view of the ORCA swarm.
    Each drone shown as coloured dot; red circle = collision radius around each.
    """
    pos_hist = results["positions"]
    T, N, _ = pos_hist.shape
    cmap = plt.cm.tab20

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_xlim(-25, 25);  ax.set_ylim(-25, 25)
    ax.set_aspect('equal')
    ax.set_xlabel('X (m)');  ax.set_ylabel('Y (m)')
    ax.set_title("S089 ORCA Swarm — XY Projection")

    # Goal markers
    for i in range(N):
        ax.plot(goals[i, 0], goals[i, 1], 'r*', markersize=6, alpha=0.4)

    scatter = ax.scatter(pos_hist[0, :, 0], pos_hist[0, :, 1],
                         c=range(N), cmap='tab20', s=30, zorder=5)
    time_text = ax.text(0.02, 0.97, '', transform=ax.transAxes, fontsize=10,
                        verticalalignment='top')

    # Subsample frames for animation
    frame_indices = list(range(0, T, max(1, T//120)))  # ~120 frames

    def update(frame_idx):
        step = frame_indices[frame_idx]
        scatter.set_offsets(pos_hist[step, :, :2])
        time_text.set_text(f't = {step * DT:.1f} s')
        return scatter, time_text

    anim = FuncAnimation(fig, update, frames=len(frame_indices),
                         interval=50, blit=True)
    writer = PillowWriter(fps=20)
    anim.save(output_path, writer=writer)
    plt.close(fig)
    print(f"Saved animation: {output_path}")


# ── Main entry point ──────────────────────────────────────────────────────────
def main():
    import os
    output_dir = "outputs/05_special_entertainment/s089_collision_avoidance"
    os.makedirs(output_dir, exist_ok=True)

    rng = np.random.default_rng(SEED)
    positions_init, goals = init_sphere_positions(N_DRONES, SPHERE_RADIUS, rng)

    print(f"N={N_DRONES} drones | sphere R={SPHERE_RADIUS} m | DT={DT} s | T_MAX={T_MAX} s\n")

    results_dict = {}
    for strategy in ("orca", "potential_field", "none"):
        print(f"Running strategy: {strategy} ...")
        res = run_simulation(strategy, positions_init, goals)
        results_dict[strategy] = res
        print(f"  Collisions      : {res['collisions']}")
        print(f"  Min separation  : {res['min_sep']:.3f} m")
        print(f"  Mission time    : {res['mission_time']:.1f} s")
        print(f"  % arrived       : {res['pct_arrived']:.1f}%")
        print(f"  Mean |accel|    : {res['mean_accel']:.3f} m/s²\n")

    # --- Plots ---
    plot_trajectories_3d(
        results_dict["orca"], goals,
        "ORCA — 50 Drone Trajectories (3-D)",
        f"{output_dir}/trajectories_orca.png")

    plot_trajectories_3d(
        results_dict["none"], goals,
        "No Avoidance — 50 Drone Trajectories (3-D)",
        f"{output_dir}/trajectories_none.png")

    plot_metrics_comparison(
        results_dict,
        f"{output_dir}/metrics_comparison.png")

    plot_separation_histogram(
        results_dict,
        f"{output_dir}/separation_histogram.png")

    animate_orca_swarm(
        results_dict["orca"], goals,
        f"{output_dir}/orca_animation.gif")

    print("\nDone. All outputs saved to:", output_dir)


if __name__ == "__main__":
    main()
```

---

## Key Parameters

| Parameter | Symbol | Value |
|-----------|--------|-------|
| Number of drones | $N$ | 50 |
| Sphere radius (initial placement) | $R_{sphere}$ | 20.0 m |
| Drone body radius | $r$ | 0.25 m |
| Collision diameter | $2r$ = `COLLISION_RADIUS` | 0.50 m |
| Formation / desired separation | `FORMATION_DIST` | 2.0 m |
| Preferred cruise speed | $v_{pref}$ | 2.0 m/s |
| Maximum speed cap | $v_{max}$ | 3.0 m/s |
| ORCA time horizon | $\tau$ | 3.0 s |
| Simulation timestep | $\Delta t$ | 0.05 s |
| Mission time limit | $T_{max}$ | 60.0 s |
| Goal arrival radius | $d_{goal}$ | 0.5 m |
| Neighbour sensing radius | $R_{nb}$ | 10.0 m |
| Potential field repulsion radius | $d_{rep}$ | 5.0 m |
| Potential field repulsion gain | $k_{rep}$ | 5.0 |
| Potential field attraction gain | $k_{att}$ | 1.0 |
| Maximum acceleration (PF) | $a_{max}$ | 5.0 m/s² |
| Random seed | — | 42 |

---

## Expected Output

- **3-D trajectory plot (ORCA)**: Matplotlib 3-D figure showing 50 coloured trajectory lines
  from sphere surface to antipodal goals; the congestion zone near the origin is clearly visible
  but drones navigate through without crossing; start positions marked with green dots, goals
  with red stars.
- **3-D trajectory plot (No avoidance)**: Same layout; trajectories converge directly to origin
  and collide; overlapping paths visualise the crash region.
- **Metrics comparison bar chart**: 4-panel figure comparing `ORCA`, `Potential Field`, and
  `No Avoidance` on collision count (ORCA = 0), mission completion time, mean acceleration
  magnitude (ORCA smoothest), and global minimum pairwise separation (ORCA above threshold).
- **Separation histogram**: For each strategy, histogram of minimum pairwise separation
  sampled across 100 timesteps; ORCA histogram entirely to the right of the 0.5 m threshold;
  no-avoidance histogram contains a large spike near 0 m; potential field intermediate.
- **Animated GIF (XY projection)**: 120-frame top-down animation of the ORCA swarm; each drone
  shown as a coloured dot; goals marked as red stars; time counter overlaid; clearly shows
  drones weaving around each other through the central congestion zone and arriving at targets.

### Expected Numerical Results

| Metric | ORCA | Potential Field | No Avoidance |
|--------|------|-----------------|--------------|
| Collision count | **0** | ~15–40 | ~200–500 |
| Min separation (m) | > 0.50 | ~0.1–0.4 | < 0.05 |
| Mission time (s) | ~25–35 | ~30–50 | ~10* |
| % drones arrived | 100% | ~60–85% | ~20–50%* |
| Mean \|accel\| (m/s²) | lowest | medium | lowest* |

*No-avoidance drones arrive quickly if they happen not to collide head-on, but most become
entangled and stop progressing. Mission time reflects the last arrival only among those that do
reach the goal.

---

## Extensions

1. **Deadlock resolution**: In symmetric configurations (e.g. perfectly antipodal drones on a
   2-D circle) ORCA can produce oscillatory deadlocks where agents continuously dodge each other
   without making progress. Implement a **random perturbation injection** (small noise added to
   $\mathbf{v}^{pref}$ when speed drops below $0.1 \cdot v_{pref}$ for more than 2 s) and
   measure deadlock frequency vs perturbation magnitude.
2. **Density scaling**: Sweep $N \in \{10, 20, 50, 100, 200\}$ with $R_{sphere}$ fixed;
   measure collision rate and mission time vs swarm density; identify the critical density at
   which ORCA first produces collisions (LP infeasibility rises faster than the penalty relaxation
   can handle).
3. **Communication-limited ORCA**: Restrict each drone's neighbourhood to the $K$ nearest
   drones (rather than all within $R_{nb}$); sweep $K \in \{3, 5, 10, 20\}$; show the trade-off
   between communication overhead and collision avoidance quality.
4. **Dynamic obstacles**: Add $M = 5$ static pillars (cylinders of radius 1.0 m) in the central
   sphere region; model each pillar as a stationary agent with $\mathbf{v} = \mathbf{0}$;
   verify ORCA continues to guarantee collision-freedom with mixed mobile/static obstacles.
5. **Real-time hardware-in-the-loop**: Deploy the ORCA velocity computation on a Raspberry Pi 4
   with inter-drone messaging via UDP broadcast; measure compute latency vs $N$ and identify the
   maximum swarm size that keeps the ORCA solve time below $\Delta t = 0.05$ s.

---

## Related Scenarios

- Prerequisites: [S085 Light Matrix Positioning](S085_light_matrix.md),
  [S088 Formation Shape Morphing](S088_formation_morphing.md)
- Follow-ups: [S098 Swarm Synchronized Dance](S098_swarm_dance.md)
- Algorithmic cross-reference:
  [S009 Differential Game Pursuit](../01_pursuit_evasion/S009_differential_game.md)
  (multi-agent velocity reasoning),
  [S070 Swarm Weeding](../04_industrial_agriculture/S070_swarm_weeding.md)
  (dense swarm coordination),
  [S013 Particle Filter Intercept](../01_pursuit_evasion/S013_particle_filter_intercept.md)
  (probabilistic multi-agent state tracking)
