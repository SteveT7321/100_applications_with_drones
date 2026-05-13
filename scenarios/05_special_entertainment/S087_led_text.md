# S087 Aerial Light Show Text Display

**Domain**: Special Ops & Entertainment | **Difficulty**: ⭐⭐ | **Status**: `[ ]` Not started

**Algorithm**: Bitmap Font Encoding + Hungarian Assignment + Morph Interpolation | **Dimension**: 3D

---

## Problem Definition

**Setup**: $N = 30$ quadrotors hover at $z = 50$ m and spell the word "DRONE" using a $6 \times 5$
pixel bitmap font (6 rows, 5 columns per character). Each lit pixel in the font bitmap corresponds
to one drone position. Characters are spaced $1.5$ column-widths apart. The 30 drones start from
random initial positions below the display altitude, fly up to their assigned pixel positions, hold
the formation long enough to display the word, then morph to a second formation spelling "SHOW"
(four characters, also 30 lit pixels in total). Collision avoidance during the morph is handled by
a priority queue: drones with longer remaining paths move first, and short-path drones yield.

**Roles**:
- **Drones** ($N = 30$): homogeneous quadrotors, each controlled by a PID position controller.
  Each drone stores its current position, its target pixel position, and a priority value equal to
  its remaining travel distance.
- **Font bitmap** ($6 \times 5$ per character): a binary matrix encoding which pixels are lit for
  each ASCII character. The union of lit pixels across all characters in a word defines the set of
  formation target positions.
- **Assignment**: the Hungarian algorithm maps the $N$ drones to the $N$ lit pixel positions so as
  to minimise total travel distance (optimal linear assignment).

**Objective**: Maximise **visual fidelity** $V$ — the fraction of lit pixel positions correctly
occupied by a drone at display time — across both the "DRONE" and "SHOW" formations, while
minimising total travel cost and morph time.

---

## Mathematical Model

### Pixel Position Mapping

For a word with characters $c = 0, 1, \ldots, C{-}1$, each character occupies columns
$c \cdot (W + g)$ through $c \cdot (W + g) + W - 1$ where $W = 5$ is the glyph width and
$g = \lfloor 1.5 W \rfloor - W$ is the inter-character gap (in pixel units). The 3D world
position of pixel $(i, j)$ in character $c$ is:

$$\mathbf{p}_{c,i,j} = \begin{bmatrix} \bigl(c(W + g) + j\bigr) \cdot d_{px} \\ i \cdot d_{px} \\ z_{show} \end{bmatrix}$$

where $d_{px} = 2.0$ m is the inter-pixel spacing, $W = 5$ columns, $H = 6$ rows, $z_{show} = 50$ m,
and the $x$-axis runs along the word length while the $y$-axis spans the character height.

### Lit Pixel Set

The full set of formation target positions for a word $\mathcal{W}$ is:

$$\mathcal{P}(\mathcal{W}) = \bigl\{\mathbf{p}_{c,i,j} : \operatorname{bitmap}_c[i,j] = 1,\; c \in \{0,\ldots,C{-}1\},\; i \in \{0,\ldots,H{-}1\},\; j \in \{0,\ldots,W{-}1\}\bigr\}$$

The total number of lit pixels must equal $N$:

$$N_{lit}(\mathcal{W}) = \sum_{c=0}^{C-1} \sum_{i=0}^{H-1} \sum_{j=0}^{W-1} \operatorname{bitmap}_c[i,j] = N = 30$$

Font bitmaps are constructed so that "DRONE" and "SHOW" each produce exactly 30 lit pixels.

### Hungarian Optimal Assignment

Given $N$ drone positions $\{\mathbf{q}_k\}_{k=1}^{N}$ and $N$ target pixel positions
$\{\mathbf{p}_m\}_{m=1}^{N}$, construct the cost matrix:

$$C_{km} = \|\mathbf{q}_k - \mathbf{p}_m\|_2, \quad k,m \in \{1,\ldots,N\}$$

The Hungarian algorithm finds a bijection $\sigma : \{1,\ldots,N\} \to \{1,\ldots,N\}$ that
minimises total travel cost:

$$\sigma^* = \operatorname{argmin}_{\sigma} \sum_{k=1}^{N} C_{k,\sigma(k)}$$

The same procedure is applied a second time when transitioning from "DRONE" to "SHOW", using the
current hover positions (word A formation) as the new start positions.

### Formation PID Controller

Each drone $k$ is governed by a decoupled PID controller in each spatial axis
$\alpha \in \{x, y, z\}$:

$$u_\alpha(t) = K_p \, e_\alpha(t) + K_i \int_0^t e_\alpha(\tau)\,d\tau + K_d \, \dot{e}_\alpha(t)$$

where $e_\alpha(t) = p_{\sigma(k),\alpha} - q_{k,\alpha}(t)$ is the position error along axis
$\alpha$. Velocity is clipped to $v_{max} = 5$ m/s and acceleration to $a_{max} = 3$ m/s$^2$.

### Morph Interpolation and Priority Queue

During the morph from word A to word B, drone $k$ travels along the straight-line path:

$$\mathbf{q}_k(t) = (1 - \tau_k(t))\,\mathbf{p}^A_{\sigma^*_A(k)} + \tau_k(t)\,\mathbf{p}^B_{\sigma^*_B(k)}, \quad \tau_k(t) \in [0,1]$$

where $\tau_k(t)$ advances at speed proportional to $v_{max}$. To avoid collisions, drones are
sorted into a priority queue by remaining path length $d_k^{rem}(t) = \|\mathbf{q}_k(t) -
\mathbf{p}^B_{\sigma^*_B(k)}\|$. At each timestep, drones with larger $d_k^{rem}$ are granted
right-of-way; drones within separation distance $r_{sep} = 1.5$ m of a higher-priority drone slow
to $50\%$ speed.

### Visual Fidelity Metric

At display time $t_{disp}$ (when all drones have converged), the visual fidelity of the formation
is:

$$V = \frac{\bigl|\bigl\{k : \|\mathbf{q}_k(t_{disp}) - \mathbf{p}_{\sigma^*(k)}\| \leq \epsilon_{pos}\bigr\}\bigr|}{N_{lit}}$$

where $\epsilon_{pos} = 0.5$ m is the position tolerance. A fidelity of $V = 1.0$ means all drones
are within tolerance of their assigned pixel.

### Total Assignment Cost

The optimality gap compares Hungarian cost against a naive nearest-neighbour greedy assignment:

$$\Delta_{cost} = \frac{\sum_k C_{k,\sigma^*_{greedy}(k)} - \sum_k C_{k,\sigma^*(k)}}{\sum_k C_{k,\sigma^*(k)}} \times 100\%$$

A positive $\Delta_{cost}$ quantifies the percentage saving from using the Hungarian algorithm.

---

## Implementation

```python
import numpy as np
from scipy.optimize import linear_sum_assignment
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# ── Constants ────────────────────────────────────────────────────────────────
N_DRONES      = 30
Z_SHOW        = 50.0     # m — display altitude
D_PX          = 2.0      # m — inter-pixel spacing
FONT_H        = 6        # rows per glyph
FONT_W        = 5        # columns per glyph
GAP_COLS      = 2        # extra blank columns between characters
V_MAX         = 5.0      # m/s — maximum drone speed
R_SEP         = 1.5      # m — collision avoidance separation radius
EPS_POS       = 0.5      # m — fidelity position tolerance
DT            = 0.05     # s — simulation timestep
KP, KI, KD   = 2.0, 0.05, 0.8

# ── 6×5 bitmap font (excerpt — D, R, O, N, E, S, H, W) ─────────────────────
FONT = {
    'D': [[1,1,1,0,0],[1,0,0,1,0],[1,0,0,1,0],[1,0,0,1,0],[1,0,0,1,0],[1,1,1,0,0]],
    'R': [[1,1,1,0,0],[1,0,0,1,0],[1,1,1,0,0],[1,0,1,0,0],[1,0,0,1,0],[1,0,0,0,1]],
    'O': [[0,1,1,1,0],[1,0,0,0,1],[1,0,0,0,1],[1,0,0,0,1],[1,0,0,0,1],[0,1,1,1,0]],
    'N': [[1,0,0,0,1],[1,1,0,0,1],[1,0,1,0,1],[1,0,0,1,1],[1,0,0,0,1],[1,0,0,0,1]],
    'E': [[1,1,1,1,1],[1,0,0,0,0],[1,1,1,1,0],[1,0,0,0,0],[1,0,0,0,0],[1,1,1,1,1]],
    'S': [[0,1,1,1,1],[1,0,0,0,0],[0,1,1,0,0],[0,0,0,1,0],[0,0,0,0,1],[1,1,1,1,0]],
    'H': [[1,0,0,0,1],[1,0,0,0,1],[1,1,1,1,1],[1,0,0,0,1],[1,0,0,0,1],[1,0,0,0,1]],
    'W': [[1,0,0,0,1],[1,0,0,0,1],[1,0,1,0,1],[1,0,1,0,1],[1,1,0,1,1],[1,0,0,0,1]],
}

def word_to_positions(word: str) -> np.ndarray:
    """Convert a word string to an (N_lit, 3) array of pixel world positions."""
    positions = []
    x_offset = 0
    for char in word:
        bitmap = np.array(FONT[char])          # (6, 5)
        for i in range(FONT_H):
            for j in range(FONT_W):
                if bitmap[i, j]:
                    x = (x_offset + j) * D_PX
                    y = (FONT_H - 1 - i) * D_PX  # flip so row 0 is top
                    positions.append([x, y, Z_SHOW])
        x_offset += FONT_W + GAP_COLS
    return np.array(positions)  # shape (N_lit, 3)

def hungarian_assign(starts: np.ndarray, targets: np.ndarray):
    """Return optimal assignment indices via Hungarian algorithm."""
    cost = np.linalg.norm(starts[:, None] - targets[None, :], axis=2)  # (N, M)
    row_ind, col_ind = linear_sum_assignment(cost)
    total_cost = cost[row_ind, col_ind].sum()
    return col_ind, total_cost  # col_ind[k] = target index for drone k

def pid_step(pos, target, vel, integral, dt):
    """Single PID step; returns new pos, vel, integral."""
    err   = target - pos
    integral = integral + err * dt
    deriv = (err - (target - (pos + vel * dt))) / dt  # approximate
    accel = KP * err + KI * integral + KD * (err / dt - vel)
    accel = np.clip(accel, -3.0, 3.0)
    vel   = np.clip(vel + accel * dt, -V_MAX, V_MAX)
    pos   = pos + vel * dt
    return pos, vel, integral

def morph_with_priority(pos_A, pos_B, dt=DT):
    """
    Fly N drones from pos_A to pos_B with priority-queue collision avoidance.
    Returns trajectory array of shape (T, N, 3).
    """
    N      = len(pos_A)
    pos    = pos_A.copy()
    vel    = np.zeros_like(pos)
    integ  = np.zeros_like(pos)
    traj   = [pos.copy()]

    while True:
        rem_dist = np.linalg.norm(pos_B - pos, axis=1)  # (N,)
        if rem_dist.max() < EPS_POS:
            break
        priority_order = np.argsort(-rem_dist)  # highest remaining dist first

        occupied = []
        new_pos = pos.copy()
        new_vel = vel.copy()
        new_int = integ.copy()

        for k in priority_order:
            speed_factor = 1.0
            for prev_k in occupied:
                if np.linalg.norm(pos[k] - pos[prev_k]) < R_SEP:
                    speed_factor = 0.5
                    break
            target_step = pos[k] + (pos_B[k] - pos[k]) * speed_factor
            new_pos[k], new_vel[k], new_int[k] = pid_step(
                pos[k], pos_B[k], vel[k], integ[k], dt * speed_factor
            )
            occupied.append(k)

        pos, vel, integ = new_pos, new_vel, new_int
        traj.append(pos.copy())

    return np.array(traj)  # (T, N, 3)

# ── Main simulation ───────────────────────────────────────────────────────────
rng            = np.random.default_rng(42)
targets_drone  = word_to_positions("DRONE")   # (30, 3)
targets_show   = word_to_positions("SHOW")    # (30, 3)  — padded to 30 pixels

init_pos       = rng.uniform([0, 0, 0], [60, 12, 5], size=(N_DRONES, 3))

assign_A, cost_A = hungarian_assign(init_pos,    targets_drone)
assign_B, cost_B = hungarian_assign(targets_drone, targets_show)

traj_to_A  = morph_with_priority(init_pos,          targets_drone[assign_A])
traj_A_to_B = morph_with_priority(targets_drone[assign_A], targets_show[assign_B])
```

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Number of drones N | 30 |
| Display altitude z_show | 50 m |
| Glyph size | 6 rows x 5 columns |
| Inter-pixel spacing d_px | 2.0 m |
| Inter-character gap | 2 extra columns (= 4.0 m) |
| Word A | "DRONE" (5 characters) |
| Word B | "SHOW" (4 characters, padded to 30 lit pixels) |
| PID gains Kp / Ki / Kd | 2.0 / 0.05 / 0.8 |
| Max drone speed v_max | 5.0 m/s |
| Max acceleration a_max | 3.0 m/s^2 |
| Separation radius r_sep | 1.5 m |
| Position tolerance eps_pos | 0.5 m |
| Simulation timestep dt | 0.05 s |
| Assignment algorithm | Hungarian (scipy.optimize.linear_sum_assignment) |
| Collision avoidance | Priority queue (longest path first, 50% yield) |

---

## Expected Output

- **Formation plot — "DRONE"**: 3D scatter plot (using `mpl_toolkits.mplot3d`) at $z = 50$ m
  showing all 30 drone positions as coloured dots overlaid on the expected pixel grid; lit pixels
  drawn as translucent grey markers; position errors shown as thin vertical lines from drone to
  target.
- **Formation plot — "SHOW"**: same 3D layout for the second word; side-by-side with the "DRONE"
  panel for direct visual comparison.
- **Assignment cost comparison bar chart**: grouped bars for "DRONE" and "SHOW" transitions,
  each bar showing Hungarian optimal cost vs greedy nearest-neighbour cost, annotated with
  percentage savings $\Delta_{cost}$.
- **Morph trajectory animation (GIF)**: 3D animated view of all 30 drones flying from "DRONE"
  positions to "SHOW" positions; drone paths drawn as fading trails; priority-yielding drones shown
  in a lighter colour while slowed.
- **Visual fidelity vs time**: line plot of $V(t)$ during both the ascent-to-"DRONE" phase and the
  "DRONE"-to-"SHOW" morph phase; a horizontal dashed line marks the $V = 0.95$ acceptance
  threshold; time-to-threshold annotated.
- **Summary metrics table**: assignment cost (Hungarian vs greedy) for both transitions, total
  morph time, final visual fidelity $V$, minimum inter-drone separation during morph.

---

## Extensions

1. **Variable word length**: generalise the pixel-count normalisation so the fleet size $N$
   adjusts automatically to $N_{lit}(\mathcal{W})$ for any input word, removing the fixed-30
   constraint.
2. **Smooth curve fonts**: replace the binary $6 \times 5$ bitmap with a vector glyph sampled at
   higher resolution ($12 \times 10$), using a larger fleet; study how assignment cost scales with
   $N$ and glyph complexity.
3. **Timed choreography**: add a music beat-synchronised morph scheduler; morph transitions are
   triggered at beat onsets derived from an audio BPM signal, and the morph duration equals one
   musical bar.
4. **Colour assignment**: equip each drone with an RGB LED; extend the cost matrix to a
   colour-and-position joint cost $C_{km} = \alpha \|\mathbf{q}_k - \mathbf{p}_m\| +
   (1-\alpha)\|\mathbf{colour}_k - \mathbf{colour}_m\|$ and re-solve with the Hungarian
   algorithm.
5. **Wind disturbance**: inject a horizontal wind field $\mathbf{w}(t)$ during the morph; measure
   degradation of visual fidelity and add a feedforward wind compensation term to the PID.
6. **Simultaneous ground-level shadow**: project the 3D formation onto a flat ground plane and
   render the shadow shape; optimise drone $z$-offsets to maximise shadow fidelity for an
   audience viewing from a fixed azimuth.

---

## Related Scenarios

- Prerequisites: [S083](S083_swarm_formation.md), [S085](S085_light_matrix.md)
- Next: [S088](S088_logo_transition.md)
- Algorithmic cross-reference: [S018 Multi-Target Interception](../01_pursuit_evasion/S018_multi_target_interception.md) (Hungarian assignment), [S019 Dynamic Reassignment](../01_pursuit_evasion/S019_dynamic_reassignment.md) (real-time re-assignment), [S040 Fleet Load Balancing](../02_logistics_delivery/S040_fleet_load_balancing.md) (priority scheduling)

## References

- Kuhn, H. W. (1955). *The Hungarian method for the assignment problem*. Naval Research Logistics Quarterly, 2(1–2), 83–97.
- Honig, W. et al. (2018). Trajectory planning for quadrotor swarms. *IEEE Transactions on Robotics*, 34(4), 856–869.
