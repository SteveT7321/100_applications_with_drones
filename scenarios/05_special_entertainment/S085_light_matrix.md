# S085 Drone Light Matrix Positioning

**Domain**: Special Ops & Entertainment | **Difficulty**: ⭐⭐ | **Status**: `[ ]` Not Started
**Algorithm**: Hungarian Assignment + Independent PID + Repulsion | **Dimension**: 3D

---

## Problem Definition

**Setup**: A swarm of $N = 25$ drones must arrange themselves into a $5 \times 5$ grid at show
altitude $z_{show} = 50$ m. The drones begin from random initial positions scattered within a
$20 \times 20 \times 10$ m staging volume near the ground. Grid points are evenly spaced at
$d = 2.0$ m in both the $x$- and $y$-directions, producing a $8 \times 8$ m footprint centred
above the launch pad.

**Roles**:
- **Drones** ($N = 25$): identical quadrotors, each carrying an RGB LED payload. Before flight
  begins, the Hungarian algorithm computes a one-shot, globally optimal assignment of drones to
  grid points, minimising total travel distance. Once assigned, each drone flies independently
  from its start position to its target grid point under an independent three-axis PID controller.
- **Grid targets**: 25 fixed 3D points $\mathbf{g}_{ij}$ arranged in a $5 \times 5$ square lattice
  at $z_{show} = 50$ m; indices $i, j \in \{0, 1, 2, 3, 4\}$.

**Safety constraint**: if any two drones come within $r_{safe} = 0.8$ m of each other during
flight, a pairwise ORCA-lite repulsive force is applied to both drones' velocity commands so that
collisions are avoided. The mission succeeds only when all 25 drones are simultaneously within
$\epsilon_{conv} = 0.1$ m of their assigned targets.

**Objective**: minimise the convergence time $T_{conv}$ — the first instant at which every drone
satisfies $\|\mathbf{p}_k - \mathbf{g}_k\| \leq \epsilon_{conv}$ simultaneously — while keeping
the total collision count at zero and the mean formation error $\varepsilon_f$ as small as
possible after convergence.

---

## Mathematical Model

### Grid Target Positions

The 25 target positions form a regular $5 \times 5$ lattice at show altitude. With inter-drone
spacing $d = 2.0$ m and grid origin $\mathbf{o} = (0, 0, z_{show})$, the target for cell
$(i, j)$ is:

$$\mathbf{g}_{ij} = \begin{bmatrix} i \cdot d \\ j \cdot d \\ z_{show} \end{bmatrix},
  \qquad i, j \in \{0, 1, 2, 3, 4\}$$

The grid is centred by subtracting the array centroid: $\mathbf{g}_{ij} \leftarrow
\mathbf{g}_{ij} - \bar{\mathbf{g}}$, where $\bar{\mathbf{g}} = (2d, 2d, z_{show})^\top$, so the
centre of the formation is at $(0, 0, 50)$ m.

### Hungarian Assignment

Let $\mathcal{D} = \{1, \ldots, 25\}$ be the drone set and
$\mathcal{G} = \{\mathbf{g}_{00}, \mathbf{g}_{01}, \ldots, \mathbf{g}_{44}\}$ be the 25 target
points (linearised in row-major order). Construct the $25 \times 25$ cost matrix:

$$C_{k,m} = \|\mathbf{p}_k(0) - \mathbf{g}_m\|, \qquad k \in \mathcal{D},\; m \in \{1, \ldots, 25\}$$

where $\mathbf{p}_k(0)$ is drone $k$'s initial position. The optimal assignment
$\sigma^* : \mathcal{D} \to \mathcal{G}$ solves:

$$\min_{\sigma} \sum_{k=1}^{25} C_{k,\sigma(k)} \quad \text{subject to } \sigma \text{ bijective}$$

This $25 \times 25$ linear sum assignment problem is solved in $O(N^3)$ time using the
Kuhn–Munkres (Hungarian) algorithm via `scipy.optimize.linear_sum_assignment`. After assignment,
drone $k$ is committed to target $\mathbf{g}_k = \mathbf{g}_{\sigma^*(k)}$ for the remainder of
the flight.

### PID Position Controller (per drone)

Each drone uses an independent three-axis discrete-time PID controller. Let
$\mathbf{e}_k(t) = \mathbf{g}_k - \mathbf{p}_k(t)$ be the position error vector. The velocity
command is:

$$\mathbf{u}_k(t) = K_p\,\mathbf{e}_k(t)
  + K_i \sum_{\tau=0}^{t} \mathbf{e}_k(\tau)\,\Delta t
  + K_d\,\frac{\mathbf{e}_k(t) - \mathbf{e}_k(t - \Delta t)}{\Delta t}$$

The position is integrated at each timestep:

$$\mathbf{p}_k(t + \Delta t) = \mathbf{p}_k(t) + \mathbf{u}_k(t)\,\Delta t$$

with the velocity command clipped to $\|\mathbf{u}_k\| \leq v_{max}$ to respect actuator limits.

### ORCA-lite Repulsive Potential

Whenever two drones $i$ and $j$ satisfy $d_{ij} = \|\mathbf{p}_i - \mathbf{p}_j\| < r_{safe}$,
a pairwise repulsive force is added to each drone's velocity command. Define the unit separation
vector $\hat{\mathbf{n}}_{ij} = (\mathbf{p}_i - \mathbf{p}_j) / d_{ij}$. The repulsive
acceleration applied to drone $i$ is:

$$\mathbf{F}_{rep,i}^{(j)} = k_{rep} \left(\frac{1}{d_{ij}} - \frac{1}{r_0}\right)
  \frac{1}{d_{ij}^2}\,\hat{\mathbf{n}}_{ij}, \qquad d_{ij} < r_0 = r_{safe}$$

The total repulsion on drone $i$ is the sum over all neighbours within range:

$$\mathbf{F}_{rep,i} = \sum_{j \neq i,\, d_{ij} < r_0} \mathbf{F}_{rep,i}^{(j)}$$

The combined velocity command becomes
$\mathbf{u}_k^{total}(t) = \text{clip}(\mathbf{u}_k(t) + \mathbf{F}_{rep,k}(t),\; v_{max})$.
The repulsion strength $k_{rep}$ is tuned so that drones decelerate smoothly before entering the
collision zone $r_{collision} = 0.5$ m.

### Convergence and Formation Error

**Convergence time**: the first time step at which every drone is within tolerance of its target:

$$T_{conv} = \min\bigl\{t : \max_{k}\|\mathbf{p}_k(t) - \mathbf{g}_k\| \leq \varepsilon_{conv}\bigr\}$$

**Mean formation error** (evaluated at and after $T_{conv}$):

$$\varepsilon_f = \frac{1}{N}\sum_{k=1}^{N} \|\mathbf{p}_k - \mathbf{g}_k\|$$

**Collision count**: total number of pairwise $(i, j)$ events where
$d_{ij} < r_{collision} = 0.5$ m at any timestep. The target is zero.

### Assignment Quality

To quantify the benefit of the Hungarian solution, compare against a random permutation baseline.
Define the total assignment cost:

$$D_{assign} = \sum_{k=1}^{N} \|\mathbf{p}_k(0) - \mathbf{g}_{\sigma^*(k)}\|$$

The reduction relative to a random assignment $\sigma_{rand}$ is:

$$\eta_{assign} = 1 - \frac{D_{assign}}{D_{rand}}, \qquad
  D_{rand} = \frac{1}{M_{trials}}\sum_{m=1}^{M_{trials}} \sum_{k=1}^{N}
  \|\mathbf{p}_k(0) - \mathbf{g}_{\sigma_m(k)}\|$$

---

## Implementation

```python
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
from scipy.optimize import linear_sum_assignment

# ── Domain and scenario constants ─────────────────────────────────────────────
N_DRONES        = 25            # swarm size
GRID_N          = 5             # grid side length
LED_SHOW_HEIGHT = 50.0          # m — show altitude (domain constant)
FORMATION_DIST  = 2.0           # m — grid spacing (domain constant)
COLLISION_RADIUS= 0.5           # m — hard collision radius (domain constant)
R_SAFE          = 0.8           # m — repulsion activation radius
R0              = R_SAFE        # m — repulsion potential cutoff
K_REP           = 1.5           # repulsion gain
V_MAX           = 5.0           # m/s — maximum velocity
CONV_THRESH     = 0.1           # m — convergence criterion
DT              = 0.05          # s — simulation timestep
T_MAX           = 60.0          # s — safety cutoff

# PID gains
KP, KI, KD     = 1.2, 0.05, 0.4


# ── Grid target generation ────────────────────────────────────────────────────

def make_grid_targets(n=GRID_N, d=FORMATION_DIST, z=LED_SHOW_HEIGHT):
    """Return (N, 3) array of 5×5 grid targets centred at (0,0,z)."""
    idx = np.arange(n)
    ii, jj = np.meshgrid(idx, idx, indexing='ij')
    x = ii.ravel() * d
    y = jj.ravel() * d
    z_arr = np.full(n * n, z)
    targets = np.stack([x, y, z_arr], axis=1).astype(float)
    # Centre the grid
    targets[:, 0] -= (n - 1) * d / 2
    targets[:, 1] -= (n - 1) * d / 2
    return targets  # shape (25, 3)


# ── Hungarian assignment ──────────────────────────────────────────────────────

def hungarian_assign(start_pos, targets):
    """
    Solve the 25×25 linear sum assignment (minimise total Euclidean distance).
    start_pos : (N, 3)  targets : (N, 3)
    Returns: assignment array of length N — assignment[k] = target index for drone k.
    """
    N = len(start_pos)
    C = np.linalg.norm(start_pos[:, None, :] - targets[None, :, :], axis=2)  # (N, N)
    row_ind, col_ind = linear_sum_assignment(C)
    assignment = np.empty(N, dtype=int)
    assignment[row_ind] = col_ind
    return assignment


# ── Simulation ────────────────────────────────────────────────────────────────

def run_simulation(seed=42):
    rng = np.random.default_rng(seed)

    # Random initial positions in a 20×20×10 m staging volume near ground
    pos = rng.uniform(
        low=[-10.0, -10.0,  1.0],
        high=[ 10.0,  10.0, 10.0],
        size=(N_DRONES, 3)
    )
    vel = np.zeros_like(pos)
    integral = np.zeros_like(pos)
    prev_err  = np.zeros_like(pos)

    targets    = make_grid_targets()
    assignment = hungarian_assign(pos, targets)
    assigned_targets = targets[assignment]  # shape (N, 3)

    # Trajectory storage
    history     = [pos.copy()]
    max_err_log = []
    mean_err_log= []
    t_log       = []
    collision_count = 0
    T_conv      = None

    t = 0.0
    n_steps = int(T_MAX / DT)

    for step in range(n_steps):
        err = assigned_targets - pos                   # (N, 3)
        integral += err * DT
        deriv = (err - prev_err) / DT
        u = KP * err + KI * integral + KD * deriv     # PID command

        # ORCA-lite repulsion
        F_rep = np.zeros_like(pos)
        for i in range(N_DRONES):
            for j in range(i + 1, N_DRONES):
                diff = pos[i] - pos[j]
                d_ij = np.linalg.norm(diff)
                if d_ij < COLLISION_RADIUS:
                    collision_count += 1
                if 0 < d_ij < R0:
                    n_hat = diff / d_ij
                    mag   = K_REP * (1.0 / d_ij - 1.0 / R0) / (d_ij ** 2)
                    F_rep[i] += mag * n_hat
                    F_rep[j] -= mag * n_hat

        u_total = u + F_rep
        # Clip to v_max
        speeds = np.linalg.norm(u_total, axis=1, keepdims=True)
        mask   = speeds > V_MAX
        u_total = np.where(mask, u_total / speeds * V_MAX, u_total)

        pos      = pos + u_total * DT
        prev_err = err

        # Metrics
        dists    = np.linalg.norm(pos - assigned_targets, axis=1)
        max_err  = dists.max()
        mean_err = dists.mean()
        max_err_log.append(max_err)
        mean_err_log.append(mean_err)
        t_log.append(t)
        history.append(pos.copy())

        if T_conv is None and max_err <= CONV_THRESH:
            T_conv = t

        t += DT

    history = np.array(history)  # (n_steps+1, N, 3)

    return {
        "history":        history,
        "targets":        targets,
        "assignment":     assignment,
        "assigned_targets": assigned_targets,
        "max_err_log":    np.array(max_err_log),
        "mean_err_log":   np.array(mean_err_log),
        "t_log":          np.array(t_log),
        "T_conv":         T_conv,
        "eps_f":          mean_err_log[-1] if mean_err_log else None,
        "collision_count": collision_count,
    }


# ── Plot 1: 3D trajectories + final formation ─────────────────────────────────

def plot_3d_trajectories(res):
    """3D trajectory plot: start positions, paths, and final formation grid."""
    fig = plt.figure(figsize=(12, 6))

    # Left panel — full trajectories
    ax1 = fig.add_subplot(121, projection='3d')
    history  = res["history"]          # (steps+1, N, 3)
    targets  = res["assigned_targets"] # (N, 3)
    n_steps  = history.shape[0]
    cmap     = plt.cm.plasma

    for k in range(N_DRONES):
        color = cmap(k / N_DRONES)
        ax1.plot(history[:, k, 0], history[:, k, 1], history[:, k, 2],
                 lw=0.7, alpha=0.6, color=color)
        ax1.scatter(*history[0, k, :],   s=20, color=color, marker='o', zorder=3)
        ax1.scatter(*history[-1, k, :],  s=30, color=color, marker='^', zorder=4)

    # Target grid
    ax1.scatter(targets[:, 0], targets[:, 1], targets[:, 2],
                s=50, color='lime', marker='*', zorder=5, label='Target grid')
    ax1.set_xlabel('x (m)')
    ax1.set_ylabel('y (m)')
    ax1.set_zlabel('z (m)')
    ax1.set_title('3D Drone Trajectories\n(circles=start, triangles=end)')
    ax1.legend(fontsize=8)

    # Right panel — final formation top-down (x-y)
    ax2 = fig.add_subplot(122)
    final_pos = history[-1]  # (N, 3)
    ax2.scatter(targets[:, 0], targets[:, 1],
                s=200, color='lime', marker='*', zorder=2, label='Grid target')
    ax2.scatter(final_pos[:, 0], final_pos[:, 1],
                s=60, color='tab:red', zorder=3, label='Final drone position')

    # Draw assignment lines
    for k in range(N_DRONES):
        ax2.plot([final_pos[k, 0], targets[k, 0]],
                 [final_pos[k, 1], targets[k, 1]],
                 'k--', lw=0.5, alpha=0.4)

    ax2.set_xlabel('x (m)')
    ax2.set_ylabel('y (m)')
    ax2.set_aspect('equal')
    ax2.set_title(f'Final Formation (top-down, z={LED_SHOW_HEIGHT} m)\n'
                  f'$\\varepsilon_f$ = {res["eps_f"]:.4f} m')
    ax2.legend(fontsize=8)
    ax2.grid(True, ls='--', alpha=0.4)

    plt.suptitle('S085 Drone Light Matrix — 3D Trajectories & Final Formation',
                 fontsize=12, fontweight='bold')
    plt.tight_layout()
    return fig


# ── Plot 2: convergence metrics ───────────────────────────────────────────────

def plot_convergence_metrics(res):
    """Two-panel figure: max/mean error vs time + per-drone final error bar chart."""
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))

    t      = res["t_log"]
    ax     = axes[0]
    ax.plot(t, res["max_err_log"],  lw=1.8, color='tab:red',    label='Max error (all drones)')
    ax.plot(t, res["mean_err_log"], lw=1.8, color='tab:blue',   label='Mean error $\\varepsilon_f$')
    ax.axhline(CONV_THRESH, color='k', ls='--', lw=1.2, label=f'Threshold {CONV_THRESH} m')
    if res["T_conv"] is not None:
        ax.axvline(res["T_conv"], color='green', ls=':', lw=1.5,
                   label=f'$T_{{conv}}$ = {res["T_conv"]:.2f} s')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Position error (m)')
    ax.set_title('Convergence Profile')
    ax.legend(fontsize=8)
    ax.grid(True, ls='--', alpha=0.4)

    # Per-drone final position error
    ax2    = axes[1]
    final_err = np.linalg.norm(
        res["history"][-1] - res["assigned_targets"], axis=1
    )
    drone_ids = np.arange(1, N_DRONES + 1)
    colors    = ['tab:green' if e <= CONV_THRESH else 'tab:red' for e in final_err]
    ax2.bar(drone_ids, final_err, color=colors, edgecolor='k', linewidth=0.5)
    ax2.axhline(CONV_THRESH, color='k', ls='--', lw=1.2,
                label=f'Threshold {CONV_THRESH} m')
    ax2.set_xlabel('Drone index')
    ax2.set_ylabel('Final position error (m)')
    ax2.set_title('Per-Drone Final Error\n(green = converged)')
    ax2.legend(fontsize=8)
    ax2.grid(True, ls='--', alpha=0.3, axis='y')

    plt.suptitle(
        f'S085 Drone Light Matrix — Convergence Metrics\n'
        f'$T_{{conv}}$ = {res["T_conv"]:.2f} s | '
        f'$\\varepsilon_f$ = {res["eps_f"]:.4f} m | '
        f'Collisions = {res["collision_count"]}',
        fontsize=11, fontweight='bold'
    )
    plt.tight_layout()
    return fig


# ── Plot 3: Hungarian vs random assignment comparison ─────────────────────────

def plot_assignment_comparison(res, n_trials=200, seed=0):
    """Bar + distribution plot comparing Hungarian total cost vs random permutations."""
    rng     = np.random.default_rng(seed)
    start   = res["history"][0]          # (N, 3) initial positions
    targets = res["targets"]             # (N, 3) grid targets

    # Hungarian cost
    assignment = res["assignment"]
    d_hungarian = np.sum(
        np.linalg.norm(start - targets[assignment], axis=1)
    )

    # Random permutation costs
    d_random = []
    for _ in range(n_trials):
        perm  = rng.permutation(N_DRONES)
        d_random.append(np.sum(np.linalg.norm(start - targets[perm], axis=1)))
    d_random = np.array(d_random)

    fig, ax = plt.subplots(figsize=(8, 5))
    ax.hist(d_random, bins=30, color='steelblue', edgecolor='white', alpha=0.85,
            label=f'Random ({n_trials} trials)')
    ax.axvline(d_hungarian, color='tab:red', lw=2.5, ls='--',
               label=f'Hungarian: {d_hungarian:.1f} m')
    ax.axvline(d_random.mean(), color='orange', lw=1.8, ls='-',
               label=f'Random mean: {d_random.mean():.1f} m')
    eta = 1.0 - d_hungarian / d_random.mean()
    ax.set_xlabel('Total assignment cost $D_{assign}$ (m)')
    ax.set_ylabel('Count')
    ax.set_title(f'S085 — Hungarian vs Random Assignment\n'
                 f'Cost reduction $\\eta_{{assign}}$ = {eta:.1%}')
    ax.legend(fontsize=9)
    ax.grid(True, ls='--', alpha=0.4)
    plt.tight_layout()
    return fig


# ── Animation ─────────────────────────────────────────────────────────────────

def make_animation(res, filename, fps=25, stride=4):
    """
    3D animation of the swarm converging to the light matrix grid.
    Each drone is coloured by plasma colormap; target grid shown as green stars.
    Saves GIF to filename.
    """
    history = res["history"]       # (steps+1, N, 3)
    targets = res["assigned_targets"]

    frames = range(0, history.shape[0], stride)
    fig    = plt.figure(figsize=(8, 7))
    ax     = fig.add_subplot(111, projection='3d')
    cmap   = plt.cm.plasma

    ax.scatter(targets[:, 0], targets[:, 1], targets[:, 2],
               s=60, color='lime', marker='*', zorder=5)

    scatters = []
    for k in range(N_DRONES):
        sc, = ax.plot([], [], [], 'o', ms=6, color=cmap(k / N_DRONES))
        scatters.append(sc)

    time_text = ax.text2D(0.02, 0.95, '', transform=ax.transAxes, fontsize=9)

    ax.set_xlim(-6, 6)
    ax.set_ylim(-6, 6)
    ax.set_zlim(0, 55)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_zlabel('z (m)')
    ax.set_title('S085 — Drone Light Matrix Formation')

    def update(frame_idx):
        f   = list(frames)[frame_idx]
        pos = history[f]                            # (N, 3)
        t   = res["t_log"][min(f, len(res["t_log"]) - 1)]
        max_e = np.linalg.norm(pos - targets, axis=1).max()
        for k, sc in enumerate(scatters):
            sc.set_data([pos[k, 0]], [pos[k, 1]])
            sc.set_3d_properties([pos[k, 2]])
        time_text.set_text(f't = {t:.2f} s | max err = {max_e:.3f} m')
        return scatters + [time_text]

    ani = animation.FuncAnimation(
        fig, update, frames=len(list(frames)),
        interval=int(1000 / fps), blit=False
    )
    ani.save(filename, writer='pillow', fps=fps)
    plt.close(fig)
    print(f"Animation saved → {filename}")
    return ani


# ── Entry point ───────────────────────────────────────────────────────────────

if __name__ == "__main__":
    import os
    out_dir = "outputs/05_special_entertainment/s085_light_matrix"
    os.makedirs(out_dir, exist_ok=True)

    res = run_simulation(seed=42)

    print(f"T_conv       = {res['T_conv']:.2f} s")
    print(f"eps_f        = {res['eps_f']:.4f} m")
    print(f"Collisions   = {res['collision_count']}")

    fig1 = plot_3d_trajectories(res)
    fig1.savefig(f"{out_dir}/s085_trajectories.png", dpi=150, bbox_inches="tight")
    plt.close(fig1)

    fig2 = plot_convergence_metrics(res)
    fig2.savefig(f"{out_dir}/s085_convergence.png", dpi=150, bbox_inches="tight")
    plt.close(fig2)

    fig3 = plot_assignment_comparison(res)
    fig3.savefig(f"{out_dir}/s085_assignment_comparison.png", dpi=150, bbox_inches="tight")
    plt.close(fig3)

    make_animation(res, f"{out_dir}/s085_light_matrix.gif")
    print("Done.")
```

---

## Key Parameters

| Parameter | Symbol | Value |
|-----------|--------|-------|
| Number of drones | $N$ | 25 |
| Grid dimensions | — | $5 \times 5$ |
| Grid spacing (domain constant) | $d$ | 2.0 m |
| Show altitude (domain constant) | $z_{show}$ | 50 m |
| Collision radius (domain constant) | $r_{collision}$ | 0.5 m |
| Repulsion activation radius | $r_{safe}$ | 0.8 m |
| Repulsion potential cutoff | $r_0$ | 0.8 m |
| Repulsion gain | $k_{rep}$ | 1.5 |
| Max velocity | $v_{max}$ | 5.0 m/s |
| Convergence threshold | $\varepsilon_{conv}$ | 0.1 m |
| PID gains $(K_p, K_i, K_d)$ | — | (1.2, 0.05, 0.4) |
| Simulation timestep | $\Delta t$ | 0.05 s |
| Safety cutoff | $T_{max}$ | 60 s |
| Staging volume | — | $20 \times 20 \times 10$ m |
| Assignment algorithm | — | Hungarian (Kuhn–Munkres) |
| Random seed | — | 42 |

---

## Expected Output

- **3D trajectories figure** (`s085_trajectories.png`): two-panel figure. Left: 3D plot of all 25
  drone paths from staging volume to show altitude; each drone coloured by plasma colormap; start
  positions marked as circles, end positions as triangles; target grid points shown as green stars.
  Right: top-down ($x$–$y$) view of the final formation at $z = 50$ m; red dots = final drone
  positions; green stars = grid targets; dashed lines connect each drone to its assigned target;
  mean formation error $\varepsilon_f$ annotated.
- **Convergence metrics figure** (`s085_convergence.png`): two-panel figure. Left: max position
  error (red) and mean formation error (blue) vs time; convergence threshold dashed black line;
  $T_{conv}$ marked with a green dotted vertical line. Right: per-drone bar chart of final position
  error; bars coloured green (converged) or red (not converged); threshold line annotated.
  Figure title shows $T_{conv}$, $\varepsilon_f$, and collision count.
- **Assignment comparison figure** (`s085_assignment_comparison.png`): histogram of total
  assignment costs over 200 random permutations (steelblue); Hungarian cost marked as red dashed
  vertical line; random mean marked in orange; cost reduction $\eta_{assign}$ annotated in title.
- **Animation** (`s085_light_matrix.gif`): 3D animation at 25 fps; each of the 25 drones shown as
  a coloured sphere moving from its staging position up to its grid slot; target grid (green stars)
  visible throughout; time and max error annotated per frame; plays until all drones have converged.
- **Console metrics**:
  - Convergence time $T_{conv}$ (s)
  - Final mean formation error $\varepsilon_f$ (m)
  - Total collision count (target: 0)

---

## Extensions

1. **Dynamic re-tasking on drone failure**: if one drone drops out mid-flight (e.g., battery
   failure), re-run the Hungarian algorithm on the remaining $N - 1$ drones and $N - 1$ grid
   positions (dropping the outermost corner) to maintain a valid formation; measure re-convergence
   time overhead.
2. **Sequential formation morphing**: after the $5 \times 5$ grid is achieved, solve a second
   Hungarian assignment to a new target shape (e.g., a circle or letter); animate the transition
   and measure inter-formation travel cost and collision-free guarantee under repulsion.
3. **Wind disturbance rejection**: add a constant horizontal wind field
   $\mathbf{w} = (w_x, w_y, 0)$ as an external force; retune the PID integral gain $K_i$ to
   reject the steady-state disturbance; compare $\varepsilon_f$ under wind vs. calm conditions.
4. **Decentralised assignment**: replace the centralised Hungarian solver with a distributed
   auction algorithm where each drone bids for its preferred grid point via local broadcast;
   compare convergence time and assignment optimality against the centralised solution.
5. **LED colour choreography**: couple the PID convergence signal to the LED colour output —
   drones transition from red (far from target) through yellow to green (converged); log the
   per-drone colour state over time as a visual indicator of formation quality.
6. **Scalability study**: sweep $N \in \{9, 16, 25, 36, 49\}$ (corresponding to $3^2$ through
   $7^2$ grids); measure $T_{conv}$, $\varepsilon_f$, and collision count as a function of $N$
   to characterise how the Hungarian + PID approach scales with swarm size.

---

## Related Scenarios

- Prerequisites: [S083 LED Show Formation](S083_led_show_formation.md) (PID hover, domain constants), [S084 Swarm Takeoff Sequencing](S084_swarm_takeoff_sequencing.md) (staged launch from ground)
- Follow-ups: [S087 Multi-Pattern Morphing](S087_multi_pattern_morphing.md) (sequential formation changes), [S088 Swarm Synchronised Dance](S088_swarm_dance.md) (time-coordinated choreography)
- Algorithmic cross-reference: [S018 Multi-Target Interception](../01_pursuit_evasion/S018_multi_target_interception.md) (Hungarian algorithm, multi-agent assignment), [S070 Swarm Weeding](../04_industrial_agriculture/S070_swarm_weeding.md) (Hungarian + PID per agent), [S049 Dynamic Zone Search](../03_environmental_sar/S049_dynamic_zone.md) (pairwise repulsion, Voronoi reallocation)
