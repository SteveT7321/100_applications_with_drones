# S088 Formation Geometric Shape Morphing

**Domain**: Special Ops & Entertainment | **Difficulty**: ⭐⭐⭐ | **Status**: `[ ]` Not Started
**Algorithm**: Shape Interpolation + ORCA Collision Avoidance | **Dimension**: 3D

---

## Problem Definition

**Setup**: A swarm of $N = 20$ identical quadrotor drones performs a choreographed aerial display by
transitioning continuously through four distinct 3D geometric shapes: **sphere → cube → helix →
star**. Each shape is defined by a set of $N = 20$ target keypoints in 3D space. At the start of
each transition the swarm receives the next shape's keypoint set and morphs toward it over a morph
time of $T_{morph} = 10$ s. A nominal formation distance of $d_{form} = 2.0$ m separates adjacent
keypoints within each target shape. Throughout the entire flight, an ORCA (Optimal Reciprocal
Collision Avoidance) velocity planner prevents inter-drone collisions: each drone continuously
adjusts its commanded velocity to remain outside the velocity obstacle induced by every neighbour
within a sensing radius $r_{sense} = 4$ m. Drones have a physical collision radius of
$r_{coll} = 0.5$ m.

**Roles**:
- **Drones** ($N = 20$): homogeneous quadrotors, each tracking a linearly interpolated target
  position while running a local ORCA velocity filter to avoid neighbours.
- **Shape library**: four canonical 3D point clouds, each normalised so that the mean inter-point
  spacing equals $d_{form} = 2.0$ m; generated analytically at initialisation.
- **Hungarian assignment**: at each shape transition a one-shot assignment optimisation maps the
  current drone positions to the incoming shape's keypoints so as to minimise the total travel
  distance and reduce cross-trajectory collisions during the morph.

**Objective**: Complete the full sequence of three transitions (sphere → cube, cube → helix,
helix → star) while simultaneously satisfying:

1. **Zero collision constraint**: no pair of drones ever comes within $2 r_{coll} = 1.0$ m of
   each other throughout the mission ($\|p_i - p_j\| \geq 1.0$ m $\forall\, i \neq j$).
2. **Formation accuracy**: mean formation error $\bar{\varepsilon}_f$ at the end of each morph
   $< 0.3$ m.
3. **Smooth velocity profile**: no drone's commanded speed exceeds $v_{max} = 3.0$ m/s at any
   timestep; acceleration changes remain bounded.

**Comparison conditions**:

1. **Linear interpolation + ORCA** (proposed): positions interpolated linearly, ORCA adjusts
   velocities in real time to resolve conflicts; Hungarian re-assignment at each transition.
2. **Linear interpolation only (no ORCA)**: same interpolation and assignment, but no collision
   avoidance layer; demonstrates why ORCA is necessary.
3. **Random assignment + ORCA**: ORCA active but assignment at each transition is random (not
   cost-minimising); shows the benefit of Hungarian pre-assignment on collision load.

---

## Mathematical Model

### Shape Keypoint Generation

Each shape produces exactly $N = 20$ 3D keypoints. Let the shape centre be the origin
$\mathbf{c} = \mathbf{0}$ and the nominal radius be $R = 4.0$ m.

**Sphere** — uniform Fibonacci-sphere sampling:

$$\mathbf{p}_k^{sphere} = R \begin{pmatrix} \sin\theta_k \cos\phi_k \\ \sin\theta_k \sin\phi_k \\ \cos\theta_k \end{pmatrix}, \quad \theta_k = \arccos\!\left(1 - \tfrac{2k}{N-1}\right), \quad \phi_k = 2\pi k / \varphi$$

where $\varphi = (1 + \sqrt{5})/2$ is the golden ratio and $k = 0, \ldots, N{-}1$.

**Cube** — $N = 20$ points distributed uniformly across the six faces of a cube with half-side
$a = 3.0$ m; 3–4 points per face, placed on a regular sub-grid:

$$\mathbf{p}_k^{cube} \in \{\pm a\} \times [-a, a]^2 \cup [-a,a]^2 \times \{\pm a\}$$

**Helix** — $N$ turns along a right-handed helix with radius $R_h = 3.0$ m and vertical pitch
$h = 0.6$ m per drone:

$$\mathbf{p}_k^{helix} = \begin{pmatrix} R_h \cos(2\pi k / N) \\ R_h \sin(2\pi k / N) \\ h\,k - h(N-1)/2 \end{pmatrix}, \quad k = 0, \ldots, N{-}1$$

**Star** — alternating inner/outer vertices of a 10-pointed star extruded along $z$ to two altitude
layers (10 points per layer). Outer radius $R_{out} = 4.5$ m, inner radius $R_{in} = 1.8$ m,
layer offsets $z \in \{-1.2, +1.2\}$ m:

$$\mathbf{p}_k^{star} = \begin{pmatrix} R_k \cos\!\left(\pi k / 10\right) \\ R_k \sin\!\left(\pi k / 10\right) \\ z_{layer(k)} \end{pmatrix}, \quad R_k = \begin{cases} R_{out} & k \text{ even} \\ R_{in} & k \text{ odd} \end{cases}$$

### Hungarian Assignment

At the start of transition $s \in \{1, 2, 3\}$, let $\mathbf{P}^{curr} \in \mathbb{R}^{N \times 3}$
be the matrix of current drone positions and $\mathbf{P}^{tgt} \in \mathbb{R}^{N \times 3}$ be the
next shape's keypoints. Define the cost matrix $\mathbf{C} \in \mathbb{R}^{N \times N}$:

$$C_{ij} = \|\mathbf{p}_i^{curr} - \mathbf{p}_j^{tgt}\|_2, \qquad i, j \in \{1, \ldots, N\}$$

The Hungarian algorithm finds the permutation $\sigma^* \in S_N$ that minimises total travel cost:

$$\sigma^* = \arg\min_{\sigma \in S_N} \sum_{i=1}^{N} C_{i,\sigma(i)}$$

Each drone $i$ is then assigned target keypoint $\mathbf{p}_{\sigma^*(i)}^{tgt}$. The assignment
runs in $O(N^3)$ time and is executed once per transition before interpolation begins.

### Linear Shape Interpolation

After assignment, drone $k$'s target position at time $t$ within the current morph interval
$[t_s,\; t_s + T_{morph}]$ is:

$$\mathbf{p}_k^{des}(t) = (1 - \tau)\,\mathbf{p}_k^A + \tau\,\mathbf{p}_k^B, \qquad \tau = \frac{t - t_s}{T_{morph}} \in [0, 1]$$

where $\mathbf{p}_k^A$ is drone $k$'s position at transition start (shape $A$) and
$\mathbf{p}_k^B = \mathbf{p}_{\sigma^*(k)}^{tgt}$ is its assigned keypoint in shape $B$. The
nominal velocity derived from differentiation is:

$$\dot{\mathbf{p}}_k^{nom}(t) = \frac{\mathbf{p}_k^B - \mathbf{p}_k^A}{T_{morph}}$$

which is constant during each morph interval. This nominal velocity is then modified by the ORCA
layer before application.

### ORCA Velocity Obstacle and Half-Plane

For any pair of drones $A$ and $B$ with positions $\mathbf{p}_A$, $\mathbf{p}_B$, velocities
$\mathbf{v}_A$, $\mathbf{v}_B$, and combined collision radius $2r = 2r_{coll}$, the
**velocity obstacle** $\mathrm{VO}_{A|B}^{T_h}$ is the set of velocities for drone $A$ that
would cause a collision within time horizon $T_h$:

$$\mathrm{VO}_{A|B}^{T_h} = \left\{\mathbf{v} : \exists\, t \in [0, T_h],\;
\bigl\|\mathbf{p}_A + \mathbf{v}\,t - (\mathbf{p}_B + \mathbf{v}_B\,t)\bigr\| < 2r\right\}$$

ORCA symmetrises responsibility: each drone assumes exactly half the collision-avoidance effort.
The **ORCA half-plane** $\mathrm{ORCA}_{A|B}$ constrains drone $A$'s new velocity $\mathbf{v}_A'$
to lie on the safe side of a half-plane in velocity space:

$$\mathrm{ORCA}_{A|B} = \left\{\mathbf{v} : (\mathbf{v} - \mathbf{v}_{cand}) \cdot \hat{\mathbf{n}}_{AB} \geq 0\right\}$$

where $\hat{\mathbf{n}}_{AB}$ is the outward normal of the closest point on the boundary of
$\mathrm{VO}_{A|B}^{T_h}$ to the current relative velocity $\mathbf{v}_A - \mathbf{v}_B$, and
$\mathbf{v}_{cand}$ is the candidate velocity on the $\mathrm{VO}$ boundary:

$$\mathbf{v}_{cand} = \mathbf{v}_A + \frac{1}{2}\,\mathbf{u}_{AB}$$

with $\mathbf{u}_{AB}$ being the minimum-change vector pushing $\mathbf{v}_A - \mathbf{v}_B$
out of $\mathrm{VO}_{A|B}^{T_h}$.

### ORCA Velocity Selection (Linear Programme)

Drone $A$ considers all neighbours $B_j$ within $r_{sense}$. Each neighbour induces one
half-plane constraint. Drone $A$ solves a small 3D linear programme to find the velocity
$\mathbf{v}_A^{new}$ closest to its nominal velocity $\dot{\mathbf{p}}_A^{nom}$ that satisfies
all half-plane constraints simultaneously:

$$\mathbf{v}_A^{new} = \arg\min_{\mathbf{v}} \;\bigl\|\mathbf{v} - \dot{\mathbf{p}}_A^{nom}\bigr\|^2$$

$$\text{subject to} \quad (\mathbf{v} - \mathbf{v}_{cand,j}) \cdot \hat{\mathbf{n}}_{AB_j} \geq 0
\quad \forall\, B_j \text{ with } \|\mathbf{p}_A - \mathbf{p}_{B_j}\| \leq r_{sense}$$

$$\|\mathbf{v}\| \leq v_{max}$$

If the LP is infeasible (too many conflicting constraints), a secondary minimisation selects the
velocity that violates the fewest constraints, using the approach of RVO2 (priority-based
violation weighting).

### Performance Metrics

**Formation error** at time $t$ — mean Euclidean distance between each drone and its current
interpolated target:

$$\varepsilon_f(t) = \frac{1}{N} \sum_{k=1}^{N} \bigl\|\mathbf{p}_k(t) - \mathbf{p}_k^{des}(t)\bigr\|$$

**Collision count** — total number of discrete timesteps at which any pair violates the hard
separation threshold:

$$n_{coll} = \sum_{t} \mathbf{1}\!\left[\exists\, i \neq j : \|\mathbf{p}_i(t) - \mathbf{p}_j(t)\| < 2r_{coll}\right]$$

**Morph smoothness** — root-mean-square of drone velocity changes (jerk proxy) across the full
mission of duration $T_{total} = 3 \times T_{morph} = 30$ s:

$$\mathcal{S} = \sqrt{\frac{1}{N \cdot T_{steps}} \sum_{k=1}^{N} \sum_{t} \left\|\Delta\mathbf{v}_k(t)\right\|^2}$$

where $\Delta\mathbf{v}_k(t) = \mathbf{v}_k(t) - \mathbf{v}_k(t - \Delta t)$ is the per-step
velocity change for drone $k$.

**Mean nearest-neighbour distance** — average closest inter-drone separation across the swarm:

$$\bar{d}_{nn}(t) = \frac{1}{N} \sum_{k=1}^{N} \min_{j \neq k} \|\mathbf{p}_k(t) - \mathbf{p}_j(t)\|$$

---

## Implementation

```python
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
from scipy.optimize import linear_sum_assignment

# ── Constants ─────────────────────────────────────────────────────────────────
N_DRONES        = 20
T_MORPH         = 10.0      # s — morph duration per shape transition
DT              = 0.05      # s — simulation timestep
V_MAX           = 3.0       # m/s — per-drone speed limit
R_COLL          = 0.5       # m — physical collision radius
R_SENSE         = 4.0       # m — ORCA sensing radius
T_HORIZON       = 2.0       # s — ORCA time horizon
FORMATION_DIST  = 2.0       # m — nominal inter-drone spacing
R_SHAPE         = 4.0       # m — shape bounding radius (sphere/helix)
R_OUTER_STAR    = 4.5       # m — star outer radius
R_INNER_STAR    = 1.8       # m — star inner radius
SHAPE_SEQUENCE  = ['sphere', 'cube', 'helix', 'star']

# Colour conventions
DRONE_COLORS    = plt.cm.tab20(np.linspace(0, 1, N_DRONES))


# ── Shape generators ──────────────────────────────────────────────────────────

def make_sphere(n: int, radius: float = R_SHAPE) -> np.ndarray:
    """Fibonacci-sphere sampling: n points on a sphere of given radius."""
    golden = (1.0 + np.sqrt(5.0)) / 2.0
    pts = []
    for k in range(n):
        theta = np.arccos(1.0 - 2.0 * k / max(n - 1, 1))
        phi   = 2.0 * np.pi * k / golden
        pts.append([radius * np.sin(theta) * np.cos(phi),
                    radius * np.sin(theta) * np.sin(phi),
                    radius * np.cos(theta)])
    return np.array(pts)


def make_cube(n: int, half_side: float = 3.0) -> np.ndarray:
    """Distribute n points roughly evenly across the 6 faces of a cube."""
    a = half_side
    # Pre-defined 20-point layout on cube faces
    pts = []
    # ±X faces (4 pts each)
    for sx in [-a, a]:
        for sy in [-a * 0.5, a * 0.5]:
            for sz in [-a * 0.5, a * 0.5]:
                pts.append([sx, sy, sz])
    # ±Y faces (4 pts each)
    for sy in [-a, a]:
        for sx in [-a * 0.5, a * 0.5]:
            for sz in [-a * 0.5, a * 0.5]:
                pts.append([sx, sy, sz])
    # ±Z faces (4 pts each)
    for sz in [-a, a]:
        for sx in [-a * 0.5, a * 0.5]:
            for sy in [-a * 0.5, a * 0.5]:
                pts.append([sx, sy, sz])
    pts = np.array(pts[:n], dtype=float)
    return pts


def make_helix(n: int, radius: float = 3.0, pitch: float = 0.6) -> np.ndarray:
    """Right-handed helix with n points, centred vertically."""
    k = np.arange(n, dtype=float)
    z = pitch * k - pitch * (n - 1) / 2.0
    pts = np.column_stack([radius * np.cos(2.0 * np.pi * k / n),
                           radius * np.sin(2.0 * np.pi * k / n),
                           z])
    return pts


def make_star(n: int, r_out: float = R_OUTER_STAR,
              r_in: float = R_INNER_STAR,
              z_layers: tuple = (-1.2, 1.2)) -> np.ndarray:
    """
    10-pointed star extruded to two altitude layers.
    n//2 points per layer; each layer alternates outer/inner vertices.
    """
    pts = []
    per_layer = n // 2
    for iz, zval in enumerate(z_layers):
        for k in range(per_layer):
            r = r_out if k % 2 == 0 else r_in
            angle = np.pi * k / (per_layer / 2)
            pts.append([r * np.cos(angle), r * np.sin(angle), zval])
    return np.array(pts[:n], dtype=float)


SHAPES = {
    'sphere': make_sphere(N_DRONES),
    'cube':   make_cube(N_DRONES),
    'helix':  make_helix(N_DRONES),
    'star':   make_star(N_DRONES),
}


# ── Hungarian assignment ──────────────────────────────────────────────────────

def hungarian_assign(curr_pos: np.ndarray,
                     target_pts: np.ndarray) -> np.ndarray:
    """
    Compute cost-minimising one-to-one assignment from current positions to
    target keypoints using the Hungarian algorithm.
    Returns assigned_targets (N, 3): row i is drone i's assigned target point.
    """
    n = len(curr_pos)
    C = np.linalg.norm(
        curr_pos[:, None, :] - target_pts[None, :, :], axis=2
    )  # (N, N)
    row_ind, col_ind = linear_sum_assignment(C)
    assigned = np.zeros_like(curr_pos)
    assigned[row_ind] = target_pts[col_ind]
    return assigned


# ── ORCA velocity filter ──────────────────────────────────────────────────────

def orca_filter(pos: np.ndarray, vel: np.ndarray,
                pref_vel: np.ndarray) -> np.ndarray:
    """
    Apply ORCA half-plane constraints to each drone.
    pos:      (N, 3) current positions
    vel:      (N, 3) current velocities
    pref_vel: (N, 3) preferred (nominal) velocities from interpolation
    Returns new_vel (N, 3) after collision avoidance.
    """
    n = len(pos)
    new_vel = pref_vel.copy()

    for i in range(n):
        half_planes_n = []   # outward normals
        half_planes_p = []   # points on the boundary

        for j in range(n):
            if i == j:
                continue
            rel_pos = pos[j] - pos[i]
            dist_ij = np.linalg.norm(rel_pos)

            if dist_ij > R_SENSE:
                continue  # outside sensing range

            rel_vel = vel[i] - vel[j]
            combined_r = 2.0 * R_COLL

            # Centre of velocity obstacle cone in relative-velocity space
            inv_dt = 1.0 / T_HORIZON
            vo_centre = rel_pos * inv_dt  # centre of VO truncation disc

            # Vector from VO centre to relative velocity
            w = rel_vel - vo_centre
            w_norm = np.linalg.norm(w)

            if dist_ij < combined_r:
                # Already overlapping: push apart with maximum urgency
                if w_norm < 1e-9:
                    w = np.array([1.0, 0.0, 0.0]) * combined_r
                    w_norm = combined_r
                n_ij = w / w_norm
                u_ij = (combined_r / T_HORIZON - np.dot(w, n_ij)) * n_ij
            else:
                # Check if relative velocity is inside the VO cone
                leg = np.sqrt(max(dist_ij**2 - combined_r**2, 0.0))

                # Project w onto disc or side leg
                if np.dot(w, rel_pos) < 0:
                    # Front half of cone
                    u_norm = combined_r * inv_dt
                    if w_norm < 1e-9:
                        n_ij = -rel_pos / (dist_ij + 1e-9)
                    else:
                        n_ij = w / w_norm
                    u_ij = (u_norm - w_norm) * n_ij
                else:
                    # Side legs of cone
                    n_ij = rel_pos / (dist_ij + 1e-9)
                    u_ij = (combined_r / dist_ij - 1.0) * rel_vel

            # ORCA: drone i takes half the responsibility
            v_cand = vel[i] + 0.5 * u_ij
            half_planes_n.append(u_ij / (np.linalg.norm(u_ij) + 1e-9))
            half_planes_p.append(v_cand)

        if not half_planes_n:
            # No neighbours in sensing range — use preferred velocity directly
            new_vel[i] = pref_vel[i]
            speed = np.linalg.norm(new_vel[i])
            if speed > V_MAX:
                new_vel[i] *= V_MAX / speed
            continue

        # Greedy LP: start from preferred velocity, project onto each half-plane
        v_opt = pref_vel[i].copy()
        for hp_n, hp_p in zip(half_planes_n, half_planes_p):
            if np.dot(v_opt - hp_p, hp_n) < 0:
                # Violated: project v_opt onto the boundary of this half-plane
                v_opt = v_opt - np.dot(v_opt - hp_p, hp_n) * hp_n

        # Clip to speed limit
        speed = np.linalg.norm(v_opt)
        if speed > V_MAX:
            v_opt *= V_MAX / speed

        new_vel[i] = v_opt

    return new_vel


# ── Main simulation ───────────────────────────────────────────────────────────

def simulate_morphing(use_orca: bool = True,
                      use_hungarian: bool = True,
                      seed: int = 42) -> dict:
    """
    Simulate the full formation morphing sequence.

    Returns log dict with per-timestep positions, velocities, metrics.
    """
    rng = np.random.default_rng(seed)

    # Initialise drones at sphere keypoints with small noise
    pos = SHAPES['sphere'].copy()
    pos += rng.normal(0, 0.05, pos.shape)   # slight position uncertainty
    vel = np.zeros((N_DRONES, 3))

    log = {
        'pos':            [],   # (T, N, 3)
        'vel':            [],   # (T, N, 3)
        'pref_vel':       [],   # (T, N, 3)
        'eps_f':          [],   # (T,) formation error
        'n_coll':         [],   # (T,) collision count
        'min_sep':        [],   # (T,) min inter-drone dist
        'mean_nn':        [],   # (T,) mean nearest-neighbour dist
        'shape_labels':   [],   # (T,) current shape name
        'transition_t':   [],   # list of (t_start, shape_A, shape_B) tuples
    }

    t = 0.0
    shape_idx = 0   # 0=sphere (initial), transitions: 0→1, 1→2, 2→3

    while shape_idx < len(SHAPE_SEQUENCE) - 1:
        shape_A_name = SHAPE_SEQUENCE[shape_idx]
        shape_B_name = SHAPE_SEQUENCE[shape_idx + 1]
        tgt_B = SHAPES[shape_B_name].copy()

        # Hungarian or random assignment
        if use_hungarian:
            assigned_B = hungarian_assign(pos, tgt_B)
        else:
            perm = rng.permutation(N_DRONES)
            assigned_B = tgt_B[perm]

        pos_A = pos.copy()   # positions at start of this morph
        t_start = t
        log['transition_t'].append((t_start, shape_A_name, shape_B_name))

        # Morph over T_MORPH seconds
        n_steps = int(T_MORPH / DT)
        for step in range(n_steps):
            tau = step / n_steps   # normalised progress in [0, 1)

            # Interpolated target position and preferred velocity
            des = (1.0 - tau) * pos_A + tau * assigned_B
            pref = (assigned_B - pos_A) / T_MORPH   # (N, 3) constant during morph

            # ORCA or straight preferred velocity
            if use_orca:
                new_vel = orca_filter(pos, vel, pref)
            else:
                new_vel = pref.copy()
                speeds  = np.linalg.norm(new_vel, axis=1, keepdims=True)
                mask    = speeds > V_MAX
                new_vel = np.where(mask, new_vel * V_MAX / (speeds + 1e-9), new_vel)

            # Integrate positions (Euler)
            pos = pos + new_vel * DT
            vel = new_vel

            t += DT

            # ── Metrics ──────────────────────────────────────────────────
            des_now = (1.0 - (step + 1) / n_steps) * pos_A + \
                      ((step + 1) / n_steps) * assigned_B
            eps_f = np.mean(np.linalg.norm(pos - des_now, axis=1))

            diffs = pos[:, None, :] - pos[None, :, :]    # (N, N, 3)
            dists = np.linalg.norm(diffs, axis=2)         # (N, N)
            np.fill_diagonal(dists, np.inf)
            nn_dists = dists.min(axis=1)                  # (N,)
            min_sep  = nn_dists.min()
            mean_nn  = nn_dists.mean()
            n_coll   = int(np.any(dists < 2.0 * R_COLL))

            log['pos'].append(pos.copy())
            log['vel'].append(vel.copy())
            log['pref_vel'].append(pref.copy())
            log['eps_f'].append(eps_f)
            log['n_coll'].append(n_coll)
            log['min_sep'].append(min_sep)
            log['mean_nn'].append(mean_nn)
            log['shape_labels'].append(f"{shape_A_name}→{shape_B_name}")

        shape_idx += 1

    # Convert to arrays
    for key in ('pos', 'vel', 'pref_vel'):
        log[key] = np.array(log[key])   # (T, N, 3)
    for key in ('eps_f', 'n_coll', 'min_sep', 'mean_nn'):
        log[key] = np.array(log[key])

    return log


# ── Plotting ──────────────────────────────────────────────────────────────────

def plot_results(logs: dict, out_dir: str = 'outputs/05_special_entertainment/'
                 's088_formation_morphing'):
    import os
    os.makedirs(out_dir, exist_ok=True)

    T_total = len(next(iter(logs.values()))['eps_f'])
    t_axis  = np.arange(T_total) * DT

    cmap = {'ORCA+Hungarian': 'tab:blue',
            'No ORCA':        'tab:red',
            'Random Assign':  'tab:orange'}

    # ── Figure 1: 3D trajectory snapshot (ORCA+Hungarian) ────────────────────
    log_main = logs['ORCA+Hungarian']
    pos_arr  = log_main['pos']   # (T, N, 3)
    T_steps  = pos_arr.shape[0]

    fig1 = plt.figure(figsize=(12, 8))
    ax3d = fig1.add_subplot(111, projection='3d')

    # Draw full trajectory trails for each drone
    for k in range(N_DRONES):
        ax3d.plot(pos_arr[:, k, 0], pos_arr[:, k, 1], pos_arr[:, k, 2],
                  lw=0.6, alpha=0.4, color=DRONE_COLORS[k])

    # Mark 4 shape keypoints at key instants
    snap_steps = [0,
                  int(T_MORPH / DT) - 1,
                  int(2 * T_MORPH / DT) - 1,
                  T_steps - 1]
    snap_labels = ['Sphere', 'Cube', 'Helix', 'Star']
    snap_markers = ['o', 's', '^', '*']
    snap_colors_3d = ['tab:blue', 'tab:green', 'tab:orange', 'tab:red']

    for snap, label, marker, col in zip(snap_steps, snap_labels,
                                        snap_markers, snap_colors_3d):
        p_snap = pos_arr[snap]
        ax3d.scatter(p_snap[:, 0], p_snap[:, 1], p_snap[:, 2],
                     s=60, marker=marker, color=col, label=f'{label} snapshot',
                     zorder=5, depthshade=False)

    ax3d.set_xlabel('X (m)')
    ax3d.set_ylabel('Y (m)')
    ax3d.set_zlabel('Z (m)')
    ax3d.set_title('S088 — 3D Swarm Trajectories: Sphere → Cube → Helix → Star\n'
                   '(ORCA + Hungarian Assignment, N=20 drones)', fontsize=11)
    ax3d.legend(fontsize=8, loc='upper left')
    plt.tight_layout()
    plt.savefig(f'{out_dir}/s088_3d_trajectories.png', dpi=150)
    plt.close()

    # ── Figure 2: Metrics panel (2 × 2 subplots) ─────────────────────────────
    fig2, axes = plt.subplots(2, 2, figsize=(14, 9))

    # Shade morph intervals
    transition_times = [(0, T_MORPH), (T_MORPH, 2 * T_MORPH), (2 * T_MORPH, 3 * T_MORPH)]
    transition_names = ['Sphere→Cube', 'Cube→Helix', 'Helix→Star']
    shade_colors = ['#e8f4f8', '#f8f4e8', '#f4f8e8']

    for ax in axes.ravel():
        for (ts, te), sc in zip(transition_times, shade_colors):
            ax.axvspan(ts, te, alpha=0.35, color=sc, zorder=0)
        for ts, _ in transition_times[1:]:
            ax.axvline(ts, color='gray', lw=0.8, ls='--', alpha=0.6)

    # Top-left: Formation error
    for name, log in logs.items():
        axes[0, 0].plot(t_axis, log['eps_f'], color=cmap[name], lw=1.5,
                        label=name)
    axes[0, 0].axhline(0.3, color='gray', ls=':', lw=1.2, label='0.3 m target')
    axes[0, 0].set_title('Formation Error $\\varepsilon_f$ (m)')
    axes[0, 0].set_xlabel('Time (s)')
    axes[0, 0].set_ylabel('Mean error (m)')
    axes[0, 0].legend(fontsize=8)

    # Top-right: Minimum inter-drone separation
    for name, log in logs.items():
        axes[0, 1].plot(t_axis, log['min_sep'], color=cmap[name], lw=1.5,
                        label=name)
    axes[0, 1].axhline(2.0 * R_COLL, color='red', ls=':', lw=1.5,
                       label=f'Collision threshold ({2*R_COLL:.1f} m)')
    axes[0, 1].set_title('Minimum Inter-Drone Separation (m)')
    axes[0, 1].set_xlabel('Time (s)')
    axes[0, 1].set_ylabel('Distance (m)')
    axes[0, 1].legend(fontsize=8)

    # Bottom-left: Cumulative collision count
    for name, log in logs.items():
        axes[1, 0].plot(t_axis, np.cumsum(log['n_coll']),
                        color=cmap[name], lw=1.5, label=name)
    axes[1, 0].set_title('Cumulative Collision Events')
    axes[1, 0].set_xlabel('Time (s)')
    axes[1, 0].set_ylabel('Count')
    axes[1, 0].legend(fontsize=8)

    # Bottom-right: Mean nearest-neighbour distance
    for name, log in logs.items():
        axes[1, 1].plot(t_axis, log['mean_nn'], color=cmap[name], lw=1.5,
                        label=name)
    axes[1, 1].axhline(FORMATION_DIST, color='green', ls=':', lw=1.2,
                       label=f'd_form = {FORMATION_DIST} m')
    axes[1, 1].set_title('Mean Nearest-Neighbour Distance (m)')
    axes[1, 1].set_xlabel('Time (s)')
    axes[1, 1].set_ylabel('Distance (m)')
    axes[1, 1].legend(fontsize=8)

    # Transition labels along top edge
    for ax in axes[0]:
        for mid_t, label in [(T_MORPH / 2, 'Sphere→Cube'),
                              (1.5 * T_MORPH, 'Cube→Helix'),
                              (2.5 * T_MORPH, 'Helix→Star')]:
            ax.text(mid_t, ax.get_ylim()[1] * 0.97, label,
                    ha='center', va='top', fontsize=7, color='dimgray',
                    style='italic')

    plt.suptitle('S088 — Formation Morphing: Performance Metrics\n'
                 '(N=20 drones, T_morph=10 s per transition)',
                 fontsize=12, fontweight='bold')
    plt.tight_layout()
    plt.savefig(f'{out_dir}/s088_metrics_panel.png', dpi=150)
    plt.close()


def animate_morphing(log: dict,
                     out_path: str = 'outputs/05_special_entertainment/'
                     's088_formation_morphing/s088_animation.gif'):
    """Create 3D animation of the full shape morphing sequence (GIF)."""
    pos_arr = log['pos']   # (T, N, 3)
    T_steps = pos_arr.shape[0]
    step    = max(1, T_steps // 200)   # target ~200 animation frames

    fig  = plt.figure(figsize=(8, 7))
    ax   = fig.add_subplot(111, projection='3d')

    ax.set_xlim(-6, 6)
    ax.set_ylim(-6, 6)
    ax.set_zlim(-5, 5)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')

    dots = [ax.plot([], [], [], 'o', color=DRONE_COLORS[k], ms=7)[0]
            for k in range(N_DRONES)]
    trails = [ax.plot([], [], [], '-', color=DRONE_COLORS[k],
                      lw=0.8, alpha=0.3)[0] for k in range(N_DRONES)]
    trail_len = int(2.0 / DT)   # 2-second trail

    title_obj = ax.set_title('')

    def update(frame):
        t_idx = frame * step
        label = log['shape_labels'][t_idx] if t_idx < len(log['shape_labels']) else ''
        elapsed = t_idx * DT
        title_obj.set_text(f'S088 Formation Morphing  |  t = {elapsed:.1f} s  |  {label}')

        for k in range(N_DRONES):
            p = pos_arr[t_idx, k]
            dots[k].set_data([p[0]], [p[1]])
            dots[k].set_3d_properties([p[2]])

            start = max(0, t_idx - trail_len)
            tx = pos_arr[start:t_idx, k, 0]
            ty = pos_arr[start:t_idx, k, 1]
            tz = pos_arr[start:t_idx, k, 2]
            trails[k].set_data(tx, ty)
            trails[k].set_3d_properties(tz)

        return dots + trails + [title_obj]

    n_frames = T_steps // step
    ani = animation.FuncAnimation(fig, update, frames=n_frames,
                                  interval=50, blit=False)
    ani.save(out_path, writer='pillow', fps=20)
    plt.close()
    print(f"Animation saved to {out_path}")


def run_simulation():
    import os
    out_dir = 'outputs/05_special_entertainment/s088_formation_morphing'
    os.makedirs(out_dir, exist_ok=True)

    configs = {
        'ORCA+Hungarian': dict(use_orca=True,  use_hungarian=True),
        'No ORCA':        dict(use_orca=False, use_hungarian=True),
        'Random Assign':  dict(use_orca=True,  use_hungarian=False),
    }

    logs = {}
    for name, kwargs in configs.items():
        print(f"Running: {name} ...")
        log = simulate_morphing(**kwargs)
        logs[name] = log

        total_coll = int(log['n_coll'].sum())
        final_eps  = float(log['eps_f'][-1])
        smoothness = float(np.sqrt(np.mean(
            np.diff(log['vel'], axis=0)**2
        )))
        print(f"  Total collision events : {total_coll}")
        print(f"  Final formation error  : {final_eps:.3f} m")
        print(f"  Morph smoothness (RMS) : {smoothness:.4f} m/s per step")
        print(f"  Min separation ever    : {log['min_sep'].min():.3f} m")

    plot_results(logs, out_dir)
    animate_morphing(logs['ORCA+Hungarian'],
                     out_path=f'{out_dir}/s088_animation.gif')
    print("All outputs saved.")
    return logs


if __name__ == '__main__':
    run_simulation()
```

---

## Key Parameters

| Parameter | Symbol | Value |
|-----------|--------|-------|
| Number of drones | $N$ | 20 |
| Shape sequence | — | Sphere → Cube → Helix → Star |
| Morph time per transition | $T_{morph}$ | 10 s |
| Total mission time | $T_{total}$ | 30 s |
| Simulation timestep | $\Delta t$ | 0.05 s |
| Maximum drone speed | $v_{max}$ | 3.0 m/s |
| Physical collision radius | $r_{coll}$ | 0.5 m |
| Hard collision threshold | $2r_{coll}$ | 1.0 m |
| ORCA sensing radius | $r_{sense}$ | 4.0 m |
| ORCA time horizon | $T_h$ | 2.0 s |
| Nominal formation distance | $d_{form}$ | 2.0 m |
| Shape bounding radius (sphere/helix) | $R$ | 4.0 m |
| Cube half-side | $a$ | 3.0 m |
| Helix radius | $R_h$ | 3.0 m |
| Helix vertical pitch | $h$ | 0.6 m per drone |
| Star outer / inner radius | $R_{out} / R_{in}$ | 4.5 m / 1.8 m |
| Star layer offsets | $z_{layer}$ | ±1.2 m |
| Formation error target at morph end | $\bar{\varepsilon}_f$ | < 0.3 m |
| Assignment algorithm | — | Hungarian ($O(N^3)$) |

---

## Expected Output

- **3D trajectory plot** (`s088_3d_trajectories.png`): world-frame 3D axes showing the full
  flight paths of all 20 drones as thin coloured trails; four snapshot point clouds at the end
  of each shape highlighted by distinct marker symbols (circle = sphere, square = cube, triangle =
  helix, star = star); legend identifying each snapshot; axis labels in metres.
- **Metrics panel** (`s088_metrics_panel.png`): 2 × 2 figure with background shading per morph
  interval and vertical dashed lines at each transition boundary:
  - Top-left: Formation error $\varepsilon_f(t)$ for all three conditions; horizontal dotted
    reference line at 0.3 m.
  - Top-right: Minimum inter-drone separation over time for all conditions; red dotted line at
    the 1.0 m collision threshold.
  - Bottom-left: Cumulative collision event count; ORCA+Hungarian expected at 0 throughout.
  - Bottom-right: Mean nearest-neighbour distance; green dotted reference at $d_{form} = 2.0$ m.
- **Morphing animation** (`s088_animation.gif`): 3D view at 20 fps; each drone rendered as a
  coloured dot with a 2-second position trail; title bar displays elapsed time and current
  transition label; approximately 200 frames covering the full 30-second mission.
- **Terminal summary**: per-condition printed metrics — total collision events, final formation
  error, morph smoothness RMS, minimum observed inter-drone separation.

**Typical results (ORCA + Hungarian)**:

| Metric | Expected Value |
|--------|----------------|
| Total collision events | 0 |
| Final formation error $\varepsilon_f$ | < 0.25 m |
| Morph smoothness $\mathcal{S}$ | < 0.05 m/s per step |
| Min inter-drone separation | > 1.0 m |

---

## Extensions

1. **SLERP quaternion-based interpolation**: replace the linear position interpolation with
   spherical linear interpolation (SLERP) on a unit-quaternion parameterisation of the shape
   manifold; this ensures constant angular speed during the morph and reduces peak velocity spikes
   at transition boundaries.
2. **Dynamic obstacle avoidance**: introduce $M = 5$ moving obstacles (e.g., balloons) crossing
   the swarm volume during the helix → star transition; extend the ORCA sensing to treat obstacles
   as zero-velocity drones with inflated radius; measure the formation error cost of detours.
3. **Wind disturbance rejection**: add a horizontal uniform wind field $\mathbf{w} = (1.5, 0, 0)$
   m/s; implement a feedforward compensation term in the preferred velocity so that the
   interpolation target is reached despite drift; compare final formation error with and without
   compensation.
4. **$N = 50$ scalability test**: scale the swarm to 50 drones and regenerate all four shapes
   analytically; profile the ORCA computation time per timestep (expected $O(N^2)$ per step) and
   measure how formation error and collision count scale with $N$.
5. **Music-synchronised choreography**: parameterise $\tau(t)$ by a non-linear ease-in/ease-out
   function $\tau(t) = 3(t/T_{morph})^2 - 2(t/T_{morph})^3$ (smooth-step); visualise the effect
   on velocity profile smoothness and compare RMS jerk against the linear interpolation baseline.
6. **Distributed ORCA without a central clock**: implement a token-passing protocol so that each
   drone executes its ORCA update using only the last broadcasted position/velocity of neighbours
   with a communication delay $\delta = 0.1$ s; measure the collision rate increase as a function
   of $\delta$.

---

## Related Scenarios

- Prerequisites: [S085 Light Show Synchronisation](S085_light_show_sync.md), [S087 Aerial Calligraphy](S087_aerial_calligraphy.md)
- Follow-ups: [S089 Swarm Music Visualisation](S089_swarm_music_viz.md)
- Algorithmic cross-reference: [S005 Formation Keeping](../01_pursuit_evasion/S005_formation_keeping.md) (multi-drone formation PID), [S009 Multi-Pursuer Coordination](../01_pursuit_evasion/S009_multi_pursuer.md) (distributed multi-agent planning), [S066 Cooperative Crane](../04_industrial_agriculture/S066_cooperative_crane.md) (ORCA-style soft repulsion in 3D formation)
