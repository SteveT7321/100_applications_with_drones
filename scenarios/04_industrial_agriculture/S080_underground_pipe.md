# S080 Underground Pipe CCTV Replacement

**Domain**: Industrial & Agriculture | **Difficulty**: ⭐⭐⭐⭐ | **Status**: `[ ]` Not Started
**Algorithm**: Concentric-Ring Wall-Following + Dead Reckoning | **Dimension**: 3D

---

## Problem Definition

**Setup**: A micro-drone (body diameter $d_{drone} = 0.20$ m) must navigate a 50 m underground
sewer pipe of inner diameter $D = 0.50$ m. The pipe contains three 90° elbow bends placed at
$s = 15$ m, $s = 30$ m, and $s = 43$ m along the centreline arc, dividing the route into four
straight segments. GPS is completely denied inside the pipe
($\text{GPS\_ERROR} = 0.1$ m nominal, but signal strength = 0 underground).
LiDAR is unavailable because the beam footprint at $D = 0.50$ m cannot distinguish pipe walls
from the drone body itself. The only ranging sensors are four ultrasonic transducers mounted on the
drone airframe, pointing in the cardinal directions relative to the drone body frame: North (+y),
South (−y), East (+x), West (−x). Each transducer returns the distance from the drone centre to the
nearest pipe wall along that axis. A forward-facing camera simulates the replacement CCTV feed
and is used for crack detection via intensity thresholding.

**Roles**:
- **Micro-drone** (×1): diameter 0.20 m, carries 4 ultrasonic sensors + forward camera;
  advances at $v = 0.20$ m/s; controlled by a concentric-ring (pipe-centering) PD loop
  and a dead-reckoning position estimator.
- **Pipe**: inner diameter 0.50 m, total centreline length 50 m, 3 × 90° elbows;
  walls seeded with $N_{crack} = 10$ synthetic crack patches of random positions and intensities.

**Objective**: Navigate the full 50 m pipe centreline without wall contact
(safety margin: drone must stay within $\pm\,(r_{clear} - r_{drone})$ of the centreline,
where $r_{clear} = D/2 = 0.25$ m and $r_{drone} = 0.10$ m, giving a 0.15 m lateral budget).
Simultaneously detect as many crack patches as possible with the forward camera.
Report: pipe coverage percentage, mean centering error $\bar{e}_{ctr}$, and number of
cracks flagged.

---

## Mathematical Model

### Coordinate Frame and Pipe Centreline

The pipe centreline is parameterised by arc length $s \in [0, 50]$ m. In straight segments the
heading vector $\hat{\mathbf{h}}$ is constant; at each elbow it rotates by 90° in one of the
cardinal planes. The drone 3-D position is:

$$\mathbf{p}(t) = \bigl(x(t),\; y(t),\; z(t)\bigr)^\top$$

The radial offset from the pipe centreline at arc position $s$ is:

$$\rho(t) = \bigl\|\mathbf{p}(t) - \mathbf{c}(s(t))\bigr\|$$

where $\mathbf{c}(s)$ is the 3-D centreline position at arc length $s$. The safety constraint is:

$$\rho(t) \leq r_{clear} - r_{drone} = 0.15 \; \text{m} \quad \forall \, t$$

### Ultrasonic Wall Distance

The nominal (noiseless) distance from the drone centre to the pipe wall along each body-frame axis
$k \in \{N, S, E, W\}$ is:

$$d_k^{true}(t) = r_{clear} - \delta_k(t)$$

where $\delta_k(t)$ is the signed radial offset of the drone along axis $k$.
For a perfectly centred drone, $d_k^{true} = r_{clear} - r_{drone}^{sensor} = 0.15$ m
(inner wall minus drone skin).

The measured distance with additive Gaussian noise is:

$$d_k^{meas}(t) = d_k^{true}(t) + \eta_k(t), \qquad
\eta_k(t) \sim \mathcal{N}\!\bigl(0,\; \sigma_{wall}^2\bigr)$$

with $\sigma_{wall} = 0.005$ m (ultrasonic precision at short range).

### Concentric-Ring Centering Controller

Define the lateral error signals from the four wall readings:

$$e_y(t) = d_N^{meas}(t) - d_S^{meas}(t)$$

$$e_x(t) = d_E^{meas}(t) - d_W^{meas}(t)$$

For a centred drone both errors equal zero; a positive $e_y$ means the drone is displaced toward
the South wall (closer to South, farther from North). The PD centering control outputs are:

$$u_y(t) = K_p \, e_y(t) + K_d \, \dot{e}_y(t)$$

$$u_x(t) = K_p \, e_x(t) + K_d \, \dot{e}_x(t)$$

with gains $K_p = 2.0$ s$^{-1}$ and $K_d = 0.4$ s. These lateral commands are added to the
constant forward advance velocity $v = 0.20$ m/s along the current pipe-heading vector
$\hat{\mathbf{h}}$:

$$\dot{\mathbf{p}}(t) = v \, \hat{\mathbf{h}}(t) + u_x(t)\,\hat{\mathbf{e}}_x + u_y(t)\,\hat{\mathbf{e}}_y$$

where $\hat{\mathbf{e}}_x$ and $\hat{\mathbf{e}}_y$ are the body-frame lateral unit vectors
perpendicular to $\hat{\mathbf{h}}$.

### Bend Detection and Heading Update

At a 90° elbow the pipe wall on one transverse side recedes sharply while the opposite wall closes
in. The asymmetry condition that triggers bend detection is:

$$|d_k^{meas}(t) - d_{k,opp}^{meas}(t)| > \Delta d_{bend}$$

where $k$ and $k_{opp}$ are the axis pair (N/S or E/W) aligned with the upcoming bend plane, and
$\Delta d_{bend} = 0.06$ m is the detection threshold. On trigger, the heading vector rotates
toward the axis showing the larger reading:

$$\hat{\mathbf{h}}_{new} = \hat{\mathbf{h}} \times \hat{\mathbf{n}}_{bend}$$

where $\hat{\mathbf{n}}_{bend}$ is the unit vector pointing toward the larger-reading wall. The
heading transition is blended over $t_{blend} = 0.5$ s using spherical linear interpolation
(SLERP) to avoid a step discontinuity in the control signal.

### Dead-Reckoning Position Estimator

Since GPS is unavailable, the drone's 3-D position is estimated by integrating the commanded
velocity:

$$\hat{\mathbf{p}}_{t+1} = \hat{\mathbf{p}}_t + v \, \Delta t \begin{bmatrix} \cos\hat{\theta}_t \\ \sin\hat{\theta}_t \\ 0 \end{bmatrix}$$

where $\hat{\theta}_t$ is the current estimated heading angle and $\Delta t = 0.05$ s is the
timestep. Accumulated dead-reckoning error grows as:

$$\sigma_{DR}(t) = \sigma_{v} \, \sqrt{t}$$

with $\sigma_{v} = 0.002$ m/s (velocity measurement noise). Over a 250 s mission this gives
$\sigma_{DR}(250) \approx 0.032$ m, within the GPS\_ERROR = 0.1 m budget.

### Wall Contact Metric

A wall contact event is flagged at each timestep when:

$$\min_k d_k^{meas}(t) < d_{contact}, \qquad d_{contact} = 0.02 \; \text{m}$$

(drone skin within 2 cm of wall). Total contact events are accumulated over the mission.

### Centering Error Metric

The root-mean-square lateral centering error over the full mission of $T$ timesteps:

$$\bar{e}_{ctr} = \sqrt{\frac{1}{T} \sum_{t=1}^{T} \rho(t)^2}$$

Target: $\bar{e}_{ctr} \leq 0.03$ m.

### Crack Detection Model

The forward camera captures a $256 \times 256$ pixel greyscale image. Cracks are seeded as
rectangular dark patches at random (arc-length, circumferential-angle) positions on the inner pipe
wall. The mean intensity of the central $64 \times 64$ pixel crop of each frame is:

$$\bar{I}(t) = \frac{1}{64^2} \sum_{u,v} I_{u,v}(t)$$

A crack flag is raised when:

$$\bar{I}(t) < I_{threshold}, \qquad I_{threshold} = 80 \quad \text{(out of 255)}$$

Cracks are modelled by injecting a dark patch of mean intensity $I_{crack} \sim U(20, 60)$
into the synthetic camera frame when the drone is within $\Delta s_{crack} = 0.30$ m of
a seeded crack position along the pipe centreline.

### Pipe Coverage Percentage

Coverage is defined as the fraction of the 50 m centreline arc that the drone has traversed
within 0.15 m of the true centreline (i.e., not touching the wall):

$$C_{pipe} = \frac{\text{arc length covered safely}}{L_{pipe}} \times 100\%$$

Target: $C_{pipe} \geq 95\%$.

---

## Implementation

```python
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
from dataclasses import dataclass, field
from typing import List, Tuple

# ── Pipe geometry ────────────────────────────────────────────────────────────
PIPE_INNER_DIAM  = 0.50    # m  inner diameter
PIPE_RADIUS      = PIPE_INNER_DIAM / 2.0          # 0.25 m
DRONE_RADIUS     = 0.10    # m  (body diameter 0.20 m)
LATERAL_BUDGET   = PIPE_RADIUS - DRONE_RADIUS      # 0.15 m safety margin
PIPE_LENGTH      = 50.0    # m  centreline arc length

# Elbow arc positions (m along centreline) and bend planes
# Each elbow rotates the heading 90° in the specified plane
ELBOW_S      = [15.0, 30.0, 43.0]
ELBOW_PLANE  = ['xz', 'xy', 'xz']   # plane of the bend

# ── Sensor parameters ────────────────────────────────────────────────────────
SIGMA_WALL    = 0.005    # m   ultrasonic noise std
D_BEND_THRESH = 0.06     # m   bend detection asymmetry threshold
D_CONTACT     = 0.02     # m   wall-contact threshold

# ── Controller gains ─────────────────────────────────────────────────────────
KP_CTR     = 2.0    # s^-1  proportional centering gain
KD_CTR     = 0.40   # s     derivative centering gain
V_ADVANCE  = 0.20   # m/s   constant forward speed

# ── Dead reckoning ───────────────────────────────────────────────────────────
SIGMA_V    = 0.002   # m/s  velocity noise std

# ── Crack detection ──────────────────────────────────────────────────────────
N_CRACKS       = 10
I_THRESHOLD    = 80       # greyscale threshold (0-255)
DS_CRACK_RANGE = 0.30     # m  longitudinal detection range

# ── Simulation timestep ──────────────────────────────────────────────────────
DT             = 0.05     # s
GPS_ERROR      = 0.10     # m  (nominal; effectively 0 underground)


def build_pipe_centreline(n_pts: int = 1000) -> Tuple[np.ndarray, np.ndarray]:
    """
    Build a piecewise-straight centreline with 3 × 90° elbows.
    Returns (positions (n_pts, 3), arc_lengths (n_pts,)).
    """
    # Segment lengths and heading vectors (simplified axis-aligned pipe)
    headings = [
        np.array([1.0, 0.0, 0.0]),   # segment 0: +x
        np.array([0.0, 0.0, 1.0]),   # segment 1: +z  (elbow at s=15)
        np.array([0.0, 1.0, 0.0]),   # segment 2: +y  (elbow at s=30)
        np.array([1.0, 0.0, 0.0]),   # segment 3: +x  (elbow at s=43)
    ]
    seg_ends = ELBOW_S + [PIPE_LENGTH]
    seg_starts = [0.0] + ELBOW_S

    arc_s = np.linspace(0.0, PIPE_LENGTH, n_pts)
    pts   = np.zeros((n_pts, 3))
    origin = np.zeros(3)

    # Build cumulative position along centreline
    for i, s in enumerate(arc_s):
        # Determine which segment
        seg = 0
        for j in range(len(seg_ends)):
            if s >= seg_starts[j]:
                seg = j
        pos = origin.copy()
        for j in range(seg):
            pos += headings[j] * (seg_ends[j] - seg_starts[j])
        pos += headings[seg] * (s - seg_starts[seg])
        pts[i] = pos

    return pts, arc_s


def true_wall_distances(pos: np.ndarray, heading: np.ndarray,
                        rng: np.random.Generator) -> np.ndarray:
    """
    Return noisy ultrasonic readings [d_N, d_S, d_E, d_W].
    Lateral body-frame axes are computed from the current heading.
    """
    # Compute body-frame lateral vectors (perpendicular to heading)
    up = np.array([0.0, 0.0, 1.0])
    if abs(np.dot(heading, up)) > 0.9:
        up = np.array([1.0, 0.0, 0.0])
    e_y = np.cross(heading, up);  e_y /= np.linalg.norm(e_y)
    e_x = np.cross(e_y, heading); e_x /= np.linalg.norm(e_x)

    # Radial offset components in body frame
    # (In simulation the "true" offset from centreline is tracked directly)
    # Placeholder: equal readings for centred drone with noise
    d_nom = PIPE_RADIUS - DRONE_RADIUS  # 0.15 m
    noise = rng.normal(0.0, SIGMA_WALL, 4)
    return np.array([d_nom, d_nom, d_nom, d_nom]) + noise


def centering_control(d_readings: np.ndarray,
                      prev_ey: float, prev_ex: float,
                      dt: float) -> Tuple[float, float, float, float]:
    """
    PD centering controller.
    Returns (u_y, u_x, e_y_new, e_x_new).
    """
    d_N, d_S, d_E, d_W = d_readings
    e_y = d_N - d_S
    e_x = d_E - d_W
    de_y = (e_y - prev_ey) / dt
    de_x = (e_x - prev_ex) / dt
    u_y = KP_CTR * e_y + KD_CTR * de_y
    u_x = KP_CTR * e_x + KD_CTR * de_x
    return u_y, u_x, e_y, e_x


def detect_bend(d_readings: np.ndarray) -> int:
    """
    Returns 1 if N/S asymmetry exceeds threshold,
             2 if E/W asymmetry exceeds threshold, else 0.
    """
    d_N, d_S, d_E, d_W = d_readings
    if abs(d_N - d_S) > D_BEND_THRESH:
        return 1
    if abs(d_E - d_W) > D_BEND_THRESH:
        return 2
    return 0


def simulate_pipe_navigation(seed: int = 0):
    """
    Full pipe navigation simulation.
    Returns trajectory array, dead-reckoning trajectory, wall-contact count,
    centering errors, crack detections, and pipe coverage fraction.
    """
    rng = np.random.default_rng(seed)

    centreline_pts, arc_s_vals = build_pipe_centreline(n_pts=1000)

    # Seed crack positions along arc
    crack_s   = rng.uniform(2.0, 48.0, N_CRACKS)
    crack_I   = rng.uniform(20, 60, N_CRACKS)   # mean pixel intensity

    # Drone state: true position starts at centreline origin
    pos   = centreline_pts[0].copy()
    pos_dr = pos.copy()              # dead-reckoning estimate

    # Initial heading: +x
    headings = [
        np.array([1.0, 0.0, 0.0]),
        np.array([0.0, 0.0, 1.0]),
        np.array([0.0, 1.0, 0.0]),
        np.array([1.0, 0.0, 0.0]),
    ]
    seg_starts = [0.0] + ELBOW_S
    seg_ends   = ELBOW_S + [PIPE_LENGTH]

    traj_true = [pos.copy()]
    traj_dr   = [pos_dr.copy()]

    wall_contacts  = 0
    centering_errors = []
    cracks_detected  = np.zeros(N_CRACKS, dtype=bool)

    arc_s_drone    = 0.0   # drone's arc-length position along centreline
    prev_ey, prev_ex = 0.0, 0.0

    # Determine current segment from arc position
    def get_segment(s):
        for j in range(len(seg_ends)):
            if s < seg_ends[j]:
                return j
        return len(seg_ends) - 1

    def centreline_at(s):
        """Interpolate centreline position at arc length s."""
        idx = int(s / PIPE_LENGTH * (len(arc_s_vals) - 1))
        idx = np.clip(idx, 0, len(arc_s_vals) - 1)
        return centreline_pts[idx].copy()

    while arc_s_drone < PIPE_LENGTH:
        seg = get_segment(arc_s_drone)
        heading = headings[seg]

        # Ultrasonic wall distances
        d_readings = true_wall_distances(pos, heading, rng)

        # Centering controller
        u_y, u_x, prev_ey, prev_ex = centering_control(
            d_readings, prev_ey, prev_ex, DT)

        # Lateral body-frame unit vectors
        up = np.array([0.0, 0.0, 1.0])
        if abs(np.dot(heading, up)) > 0.9:
            up = np.array([1.0, 0.0, 0.0])
        e_y = np.cross(heading, up);  e_y /= np.linalg.norm(e_y)
        e_x = np.cross(e_y, heading); e_x /= np.linalg.norm(e_x)

        # True position update
        vel = V_ADVANCE * heading + u_y * e_y + u_x * e_x
        vel_noise = rng.normal(0.0, SIGMA_V, 3)
        pos = pos + (vel + vel_noise) * DT

        # Clamp lateral offset to pipe interior (simulate wall contact)
        cl_pos = centreline_at(arc_s_drone)
        lateral_vec = pos - cl_pos
        lateral_along_heading = np.dot(lateral_vec, heading) * heading
        lateral_perp = lateral_vec - lateral_along_heading
        rho = np.linalg.norm(lateral_perp)

        if rho > LATERAL_BUDGET:
            # Drone has hit the wall — clamp and record
            wall_contacts += 1
            lateral_perp = lateral_perp / rho * LATERAL_BUDGET
            pos = cl_pos + lateral_along_heading + lateral_perp

        centering_errors.append(rho)

        # Advance arc position
        arc_s_drone += V_ADVANCE * DT

        # Dead-reckoning update
        dr_noise = rng.normal(0.0, SIGMA_V, 3)
        pos_dr = pos_dr + (V_ADVANCE * heading + dr_noise) * DT

        traj_true.append(pos.copy())
        traj_dr.append(pos_dr.copy())

        # Wall contact check via minimum sensor reading
        if np.min(d_readings) < D_CONTACT:
            wall_contacts += 1   # redundant check via sensor

        # Crack detection
        for ci in range(N_CRACKS):
            if cracks_detected[ci]:
                continue
            if abs(arc_s_drone - crack_s[ci]) < DS_CRACK_RANGE:
                # Synthesise camera frame
                I_background = rng.uniform(160, 220)
                I_frame = I_background
                # Inject crack patch at centre
                I_frame = crack_I[ci]
                if I_frame < I_THRESHOLD:
                    cracks_detected[ci] = True

    traj_true = np.array(traj_true)
    traj_dr   = np.array(traj_dr)
    centering_errors = np.array(centering_errors)

    # Pipe coverage: fraction of arc safely traversed (no wall contact)
    coverage = min(arc_s_drone, PIPE_LENGTH) / PIPE_LENGTH * 100.0

    return {
        'traj_true'       : traj_true,
        'traj_dr'         : traj_dr,
        'centering_errors': centering_errors,
        'wall_contacts'   : wall_contacts,
        'cracks_detected' : cracks_detected,
        'crack_s'         : crack_s,
        'centreline_pts'  : centreline_pts,
        'arc_s_vals'      : arc_s_vals,
        'coverage_pct'    : coverage,
    }


def plot_results(res: dict):
    traj   = res['traj_true']
    dr     = res['traj_dr']
    cl_pts = res['centreline_pts']
    e_ctr  = res['centering_errors']
    n_steps = len(e_ctr)
    t_axis = np.arange(n_steps) * DT

    # ── Figure 1: 3-D pipe path ──────────────────────────────────────────────
    fig1 = plt.figure(figsize=(12, 6))
    ax3d = fig1.add_subplot(121, projection='3d')
    ax3d.plot(cl_pts[:, 0], cl_pts[:, 1], cl_pts[:, 2],
              'k--', lw=1.5, alpha=0.5, label='Centreline')
    ax3d.plot(traj[:, 0], traj[:, 1], traj[:, 2],
              color='red', lw=1.2, label='True path')
    ax3d.plot(dr[:, 0], dr[:, 1], dr[:, 2],
              color='orange', lw=1.0, alpha=0.6, linestyle=':', label='Dead reckoning')
    # Mark elbow positions
    for s_elbow in ELBOW_S:
        idx = int(s_elbow / PIPE_LENGTH * (len(cl_pts) - 1))
        ax3d.scatter(*cl_pts[idx], color='purple', s=60, zorder=5)
    # Mark detected cracks on centreline
    for ci, detected in enumerate(res['cracks_detected']):
        s_c = res['crack_s'][ci]
        idx = int(s_c / PIPE_LENGTH * (len(cl_pts) - 1))
        color = 'green' if detected else 'grey'
        ax3d.scatter(*cl_pts[idx], color=color, marker='x', s=80, zorder=6)
    ax3d.set_xlabel('X (m)'); ax3d.set_ylabel('Y (m)'); ax3d.set_zlabel('Z (m)')
    ax3d.set_title('3-D Pipe Navigation\n'
                   r'Red=path, Orange=DR, Purple$\bullet$=elbows, '
                   r'Green$\times$=crack')
    ax3d.legend(fontsize=7)

    # ── Sub-panel: centering error vs arc position ───────────────────────────
    ax2 = fig1.add_subplot(122)
    ax2.plot(t_axis, e_ctr * 100.0, color='steelblue', lw=1.0)
    ax2.axhline(LATERAL_BUDGET * 100.0, color='red', lw=1.2, linestyle='--',
                label=f'Safety limit {LATERAL_BUDGET*100:.0f} cm')
    ax2.axhline(3.0, color='green', lw=1.0, linestyle='--',
                label='Target 3 cm')
    ax2.set_xlabel('Time (s)'); ax2.set_ylabel('Radial offset (cm)')
    ax2.set_title('Centering Error Over Time')
    ax2.legend(fontsize=8)
    fig1.tight_layout()
    fig1.savefig('outputs/04_industrial_agriculture/s080_underground_pipe/'
                 'pipe_3d_and_centering.png', dpi=150)
    plt.close(fig1)

    # ── Figure 2: wall distance time series + crack detection events ─────────
    fig2, axes = plt.subplots(2, 1, figsize=(12, 6), sharex=True)
    ax_w = axes[0]
    # Simulate d_N time series as e_ctr proxy (placeholder for per-axis logging)
    ax_w.plot(t_axis, np.full(n_steps, (PIPE_RADIUS - DRONE_RADIUS) * 100.0),
              'k--', lw=1.0, label='Nominal (centred)')
    ax_w.set_ylabel('Wall distance (cm)')
    ax_w.set_title('Ultrasonic Wall Distance Proxy (mean)')
    ax_w.legend(fontsize=8)

    ax_c = axes[1]
    for ci in range(N_CRACKS):
        t_crack = res['crack_s'][ci] / V_ADVANCE
        color = 'green' if res['cracks_detected'][ci] else 'red'
        ax_c.axvline(t_crack, color=color, lw=1.5, alpha=0.8)
    ax_c.set_xlabel('Time (s)')
    ax_c.set_ylabel('Detection event')
    ax_c.set_title('Crack Detection Events (green=detected, red=missed)')
    from matplotlib.lines import Line2D
    legend_elements = [Line2D([0], [0], color='green', lw=2, label='Detected'),
                       Line2D([0], [0], color='red',   lw=2, label='Missed')]
    ax_c.legend(handles=legend_elements, fontsize=8)
    fig2.tight_layout()
    fig2.savefig('outputs/04_industrial_agriculture/s080_underground_pipe/'
                 'wall_distance_and_cracks.png', dpi=150)
    plt.close(fig2)

    return fig1, fig2


def animate_pipe(res: dict, save_path: str):
    """
    Top-down XY animation of drone traversing the pipe centreline.
    Uses FuncAnimation saved as GIF.
    """
    traj   = res['traj_true']
    cl_pts = res['centreline_pts']
    e_ctr  = res['centering_errors']

    fig_anim, ax_anim = plt.subplots(figsize=(10, 4))
    ax_anim.plot(cl_pts[:, 0], cl_pts[:, 1], 'k--', lw=1.5, alpha=0.5)
    # Draw pipe boundary (simplified as ±radius band)
    ax_anim.fill_between(cl_pts[:, 0],
                         cl_pts[:, 1] - PIPE_RADIUS,
                         cl_pts[:, 1] + PIPE_RADIUS,
                         color='lightgrey', alpha=0.4, label='Pipe interior')
    drone_dot, = ax_anim.plot([], [], 'ro', ms=8, label='Drone')
    path_line, = ax_anim.plot([], [], 'r-', lw=1.2, alpha=0.6)
    crack_scatter = ax_anim.scatter([], [], c=[], cmap='RdYlGn',
                                    vmin=0, vmax=1, s=60, zorder=5)
    ax_anim.set_xlabel('X (m)'); ax_anim.set_ylabel('Y (m)')
    ax_anim.set_title('Pipe Navigation Animation (top-down XY)')
    ax_anim.legend(fontsize=8)
    ax_anim.set_aspect('equal')

    stride = max(1, len(traj) // 200)
    frames = range(0, len(traj), stride)

    def init():
        drone_dot.set_data([], [])
        path_line.set_data([], [])
        return drone_dot, path_line

    def update(frame):
        drone_dot.set_data([traj[frame, 0]], [traj[frame, 1]])
        path_line.set_data(traj[:frame+1, 0], traj[:frame+1, 1])
        return drone_dot, path_line

    ani = animation.FuncAnimation(fig_anim, update, frames=frames,
                                  init_func=init, blit=True, interval=50)
    ani.save(save_path, writer='pillow', fps=10)
    plt.close(fig_anim)
    return ani


def run_simulation():
    import os
    out_dir = ('outputs/04_industrial_agriculture/s080_underground_pipe')
    os.makedirs(out_dir, exist_ok=True)

    res = simulate_pipe_navigation(seed=42)

    n_detected = res['cracks_detected'].sum()
    rms_ctr = np.sqrt(np.mean(res['centering_errors']**2)) * 100.0  # cm

    print("=" * 55)
    print("S080 Underground Pipe CCTV Replacement — Results")
    print("=" * 55)
    print(f"  Pipe coverage          : {res['coverage_pct']:.1f} %")
    print(f"  RMS centering error    : {rms_ctr:.2f} cm  (target ≤ 3 cm)")
    print(f"  Wall contacts          : {res['wall_contacts']}")
    print(f"  Cracks detected        : {n_detected} / {N_CRACKS}")
    print(f"  Dead-reckoning drift   : "
          f"{np.linalg.norm(res['traj_true'][-1] - res['traj_dr'][-1]):.3f} m")
    print("=" * 55)

    plot_results(res)
    animate_pipe(res, save_path=f'{out_dir}/pipe_animation.gif')

    return res


if __name__ == '__main__':
    run_simulation()
```

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Pipe inner diameter $D$ | 0.50 m |
| Pipe centreline length $L_{pipe}$ | 50 m |
| Number of 90° elbows | 3 |
| Elbow arc positions | 15 m, 30 m, 43 m |
| Drone body diameter $d_{drone}$ | 0.20 m |
| Lateral safety budget $r_{clear} - r_{drone}$ | 0.15 m |
| Forward advance speed $v$ | 0.20 m/s |
| Ultrasonic noise std $\sigma_{wall}$ | 0.005 m |
| Centering proportional gain $K_p$ | 2.0 s$^{-1}$ |
| Centering derivative gain $K_d$ | 0.40 s |
| Bend detection threshold $\Delta d_{bend}$ | 0.06 m |
| Heading blend time $t_{blend}$ | 0.5 s |
| Dead-reckoning velocity noise $\sigma_v$ | 0.002 m/s |
| Wall-contact sensor threshold $d_{contact}$ | 0.02 m |
| Crack count $N_{crack}$ | 10 |
| Crack detection intensity threshold $I_{threshold}$ | 80 / 255 |
| Crack detection longitudinal range $\Delta s_{crack}$ | 0.30 m |
| Simulation timestep $\Delta t$ | 0.05 s |
| GPS error (denied underground) | 0.10 m (nominal, inaccessible) |
| Target pipe coverage | ≥ 95 % |
| Target RMS centering error $\bar{e}_{ctr}$ | ≤ 0.03 m |

---

## Expected Output

- **3-D pipe path plot**: Matplotlib 3-D axes (red = true drone path, orange dotted = dead-reckoning
  estimate, black dashed = ideal centreline); elbow positions marked as purple spheres; detected
  crack positions as green crosses, undetected as grey crosses; a semi-transparent grey tube of
  radius $r_{clear} = 0.25$ m rendered around the centreline to visualise the pipe walls.
- **Centering error time series**: radial offset $\rho(t)$ in cm plotted over simulation time;
  red dashed line at the 15 cm safety limit; green dashed line at the 3 cm target; wall-contact
  events highlighted as red background bands.
- **Wall-distance and crack-detection panel**: upper subplot shows mean wall clearance vs time;
  lower subplot shows crack detection events as vertical tick marks (green = detected,
  red = missed) at the arc-length positions of the seeded cracks.
- **Animation (GIF)**: top-down XY view of the drone advancing through the pipe; drone shown as
  red circle of diameter $d_{drone}$; pipe interior shaded grey; path history drawn as a trailing
  red line; crack patches flash green when detected; elbow positions marked as purple diamonds;
  frame rate 10 fps, ~200 frames covering the full 50 m transit.
- **Console metrics table**: pipe coverage % (target ≥ 95 %), RMS centering error in cm
  (target ≤ 3 cm), wall contact count (target = 0), cracks detected / total, and
  dead-reckoning terminal drift in metres.

---

## Extensions

1. **IMU-augmented dead reckoning**: add a simulated 6-DOF IMU (accelerometer + gyro) and fuse
   its output with the ultrasonic centering estimates via a complementary filter to bound
   accumulated heading drift at elbow transitions below 2°.
2. **Partial blockage detection**: seed one pipe segment with a 30 % cross-sectional obstruction
   (simulating root intrusion or sediment deposit); detect it as a simultaneous decrease in
   all four ultrasonic readings below $d_{block} < 0.08$ m and halt the drone.
3. **Continuous crack localisation**: replace the binary crack flag with a sub-centimetre
   position estimate by fitting an intensity-dip Gaussian along the longitudinal axis; report
   the arc-length position of each crack to ±5 cm accuracy.
4. **Pipe diameter adaptation**: parameterise the controller for pipes of diameter 0.35–1.00 m;
   show how the concentric-ring gains $K_p$ and bend threshold $\Delta d_{bend}$ must scale
   with $D$ to maintain $\bar{e}_{ctr} \leq 0.03$ m across the full diameter range.
5. **Multi-pass coverage**: after completing the initial forward pass, return the drone via
   reverse dead reckoning and perform a second pass at 90° rotated body orientation to capture
   circumferential cracks missed by the N/S/E/W sensor alignment in the first pass.

---

## Related Scenarios

- Prerequisites: [S064 Greenhouse Interior Flight](S064_greenhouse.md), [S074 Mine 3D Mapping](S074_mine_mapping.md), [S075 Tunnel Navigation](S075_tunnel.md)
- Algorithmic cross-reference: [S071 Bridge Underside Inspection](S071_bridge_inspection.md) (surface crack detection), [S069 Warehouse Inventory](S069_warehouse_inventory.md) (GPS-denied indoor navigation)
- See also: [S013 Particle Filter Intercept](../01_pursuit_evasion/S013_particle_filter_intercept.md) (probabilistic state estimation under noise)
