# S064 Greenhouse Interior Precision Flight

**Domain**: Industrial & Agriculture | **Difficulty**: ⭐⭐⭐ | **Status**: `[ ]` Not Started
**Algorithm**: Dead Reckoning + Ultrasonic Wall Fusion | **Dimension**: 3D

---

## Problem Definition

**Setup**: A drone must navigate autonomously inside a commercial greenhouse (40 × 10 × 3 m) to
inspect all five plant rows. Each row runs parallel to the X-axis and is modelled as a continuous
wall of plant obstacles centred at $y \in \{2, 4, 6, 8, 10\}$ m and extending from $x = 0$ to
$x = 40$ m at height $z = 0$ to $z = 1.5$ m. No GPS signal penetrates the metal structure, so
the drone cannot rely on global positioning. Instead, it fuses two complementary sources of
information: a dead-reckoning estimate obtained by integrating IMU velocity measurements (subject
to cumulative drift), and lateral distance readings from a pair of short-range ultrasonic sensors
that measure the distance to the nearest plant wall on each side. A complementary filter combines
these two estimates to maintain bounded lateral positioning error throughout the mission.

The drone executes a **boustrophedon (serpentine) coverage pattern**: it flies along the aisle
between rows 1 and 2, turns at $x = 40$ m, flies the aisle between rows 2 and 3, and so on,
visiting all five row-end turning points before returning to the start. The mission succeeds if
all five row-ends are reached without any collision (minimum clearance $d_{safe} = 0.25$ m to any
plant wall or greenhouse boundary).

**Roles**:
- **Drone**: single inspection agent with dimensions 0.3 × 0.3 × 0.15 m; carries IMU and two
  ultrasonic transducers (left wall and right wall); starts at $\mathbf{p}_0 = (0.0, 1.0, 1.0)$ m
  facing $+X$.
- **Plant rows**: five static wall obstacles at $y = 2, 4, 6, 8, 10$ m; each 0.2 m thick
  (occupying $y \pm 0.1$ m), full X-extent, height 0 – 1.5 m.
- **Greenhouse walls**: hard boundaries at $x = 0$, $x = 40$ m, $y = 0$, $y = 10$ m, $z = 0$,
  $z = 3$ m.

**Objective**: Complete the full boustrophedon sweep (visit all five row-ends) within
$T_{max} = 300$ s without collision, minimising total lateral position error accumulated over
the flight due to dead-reckoning drift.

---

## Mathematical Model

### Dead-Reckoning Position Update

The drone integrates body-frame velocity $\mathbf{v} = (v_x, v_y, v_z)^\top$ from the IMU to
propagate its estimated position. Stochastic drift enters additively at each timestep:

$$\mathbf{p}_{t+1} = \mathbf{p}_t + \mathbf{v}\,\Delta t + \boldsymbol{\eta}_t, \qquad
\boldsymbol{\eta}_t \sim \mathcal{N}\!\left(\mathbf{0},\; \sigma_{drift}^2\,\Delta t\,\mathbf{I}_3\right)$$

where $\sigma_{drift} = 0.02$ m/s is the drift noise standard deviation. Over a $T = 300$ s
mission the accumulated one-axis standard deviation grows as
$\sigma_{acc} = \sigma_{drift}\sqrt{T} \approx 0.35$ m — comparable to the aisle half-width —
making drift correction essential.

### Ultrasonic Wall Sensor Model

The drone carries two ultrasonic sensors, one pointing in the $-Y$ direction (left wall) and one
in the $+Y$ direction (right wall). Each sensor returns the range to the nearest plant wall or
greenhouse boundary within the operative band $[0.1, 2.0]$ m. Measurement noise is additive
Gaussian:

$$z_{wall} = d_{true} + w, \qquad w \sim \mathcal{N}(0,\; \sigma_{range}^2)$$

with $\sigma_{range} = 0.05$ m. If $d_{true}$ lies outside $[0.1, 2.0]$ m the sensor returns no
valid reading (out-of-range flag). Given the row positions $y_{row,k}$, the true left and right
distances for a drone at lateral position $y$ flying in the aisle between rows $k$ and $k+1$:

$$d_{left} = y - y_{row,k} - 0.1, \qquad d_{right} = y_{row,k+1} - 0.1 - y$$

The aisle centre is $y_{centre} = (y_{row,k} + y_{row,k+1})/2$ and the aisle half-width is
$W/2 = (y_{row,k+1} - y_{row,k} - 0.2)/2 = 0.9$ m.

### Complementary Filter for Lateral Position

A complementary filter fuses the dead-reckoning $Y$ estimate with the wall-range-derived $Y$
measurement at each timestep:

$$\hat{y}_{DR} = \hat{y}_{t} + v_y\,\Delta t + \eta_y$$

$$\hat{y}_{wall} = y_{row,k} + 0.1 + z_{left} \quad \text{(from left sensor, if valid)}$$

$$\hat{y}_{t+1} = \alpha\,\hat{y}_{DR} + (1 - \alpha)\,\hat{y}_{wall}$$

where $\alpha \in (0, 1)$ is the complementary filter gain. Setting $\alpha = 0.85$ gives the
dead-reckoning model 85 % weight (preserving short-term smoothness) while the wall measurement
corrects the accumulated 15 % at each step. When both left and right sensors are valid, the
wall-derived position is the average of both estimates:

$$\hat{y}_{wall} = \tfrac{1}{2}\!\left[(y_{row,k} + 0.1 + z_{left}) + (y_{row,k+1} - 0.1 - z_{right})\right]$$

### Row-Following Lateral Error and Proportional Control

The lateral tracking error relative to the aisle centre:

$$e_y = \hat{y} - y_{centre}$$

A proportional lateral correction velocity is applied:

$$v_y^{cmd} = -K_p\,e_y, \qquad K_p = 0.8 \;\text{s}^{-1}$$

The commanded forward speed along the row is held constant at $v_x^{cmd} = 1.0$ m/s. Altitude is
regulated separately at $z_{cruise} = 1.0$ m via a proportional height controller.

### Collision Avoidance: Repulsive Correction

At each timestep the controller checks the minimum clearance to all plant walls and greenhouse
boundaries. If the minimum distance falls below the safety threshold $d_{safe} = 0.25$ m, an
additional repulsive velocity is injected:

$$v_y^{rep} = K_{rep} \cdot \frac{d_{safe} - d_{min}}{d_{safe}} \cdot \text{sign}(e_{wall})$$

where $e_{wall}$ is the signed penetration (positive = too close to right wall), and
$K_{rep} = 1.5$ m/s is the repulsion gain. The total commanded lateral velocity is:

$$v_y^{total} = v_y^{cmd} + v_y^{rep}$$

### Boustrophedon Waypoint Sequence

The drone navigates a sequence of $N_{wp} = 10$ waypoints defining the serpentine path through
all five aisles. For aisle $k$ (between rows $k$ and $k+1$), the centre lateral coordinate is
$y_k^{centre}$. The waypoints alternate between $x = 40$ m (right end) and $x = 0$ m (left end):

$$\mathbf{w}_i = \begin{cases}
(40.0,\; y_k^{centre},\; z_{cruise}) & \text{if } k \text{ is even (outbound leg)} \\
(0.0,\; y_k^{centre},\; z_{cruise}) & \text{if } k \text{ is odd (return leg)}
\end{cases}$$

Waypoint $\mathbf{w}_i$ is considered reached when $\|\mathbf{p} - \mathbf{w}_i\| < d_{wp} = 0.5$ m.
At each turn, the drone adjusts its heading and updates the active aisle index.

### Drift Error Metric

The cumulative lateral position error quantifies the quality of the sensor-fusion scheme:

$$E_{drift} = \frac{1}{N_t} \sum_{t=1}^{N_t} \left|\hat{y}_t - y_t^{true}\right|$$

where $y_t^{true}$ is the ground-truth $Y$ position and $\hat{y}_t$ is the filter estimate. A
secondary metric is the **peak lateral error** $E_{max} = \max_t |\hat{y}_t - y_t^{true}|$.

---

## Implementation

```python
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

# ── Environment ────────────────────────────────────────────────────────────────
GH_X      = 40.0          # m  greenhouse length
GH_Y      = 10.0          # m  greenhouse width
GH_Z      = 3.0           # m  greenhouse height
ROW_Y     = [2.0, 4.0, 6.0, 8.0, 10.0]   # m  plant-row centre Y positions
ROW_THICK = 0.2           # m  wall thickness (±0.1 m each side)
ROW_H     = 1.5           # m  plant wall height

# ── Drone / sensor parameters ───────────────────────────────────────────────────
GPS_ERROR      = 0.1      # m  (GPS denied indoors — value retained for reference)
INSP_DIST      = 0.5      # m  inspection stand-off distance
INSP_SPEED     = 1.0      # m/s  nominal cruise speed along row
Z_CRUISE       = 1.0      # m  cruise altitude
D_SAFE         = 0.25     # m  minimum wall clearance
D_WP           = 0.5      # m  waypoint capture radius

# ── IMU dead-reckoning noise ────────────────────────────────────────────────────
SIGMA_DRIFT    = 0.02     # m/s  drift noise std per sqrt-second

# ── Ultrasonic sensor ────────────────────────────────────────────────────────────
SIGMA_RANGE    = 0.05     # m  range noise std
RANGE_MIN      = 0.1      # m  sensor dead band
RANGE_MAX      = 2.0      # m  sensor max range

# ── Complementary filter ────────────────────────────────────────────────────────
ALPHA          = 0.85     # dead-reckoning weight

# ── Controllers ────────────────────────────────────────────────────────────────
KP_LAT         = 0.8      # s⁻¹  proportional lateral gain
KP_ALT         = 1.2      # s⁻¹  altitude controller gain
K_REP          = 1.5      # m/s  repulsion gain

# ── Simulation ──────────────────────────────────────────────────────────────────
DT             = 0.05     # s   timestep
T_MAX          = 300.0    # s   mission horizon


def aisle_centre(aisle_idx):
    """Y centre of aisle between row aisle_idx and row aisle_idx+1 (0-indexed rows)."""
    y_left  = ROW_Y[aisle_idx] + ROW_THICK / 2
    y_right = ROW_Y[aisle_idx + 1] - ROW_THICK / 2
    return (y_left + y_right) / 2.0


def build_waypoints():
    """Boustrophedon waypoints through 4 aisles (between 5 rows)."""
    wps = []
    for k in range(4):
        yc = aisle_centre(k)
        x_end = GH_X if (k % 2 == 0) else 0.0
        wps.append(np.array([x_end, yc, Z_CRUISE]))
    return wps


def true_wall_distances(y):
    """
    Returns (d_left, d_right) — true lateral distances to nearest row wall
    or greenhouse boundary at the drone's current Y position.
    Works for any aisle; finds the nearest wall on each side.
    """
    walls_left  = [0.0] + [ry + ROW_THICK / 2 for ry in ROW_Y]
    walls_right = [ry - ROW_THICK / 2 for ry in ROW_Y] + [GH_Y]
    d_left  = y - max(w for w in walls_left  if w <= y)
    d_right = min(w for w in walls_right if w >= y) - y
    return d_left, d_right


def ultrasonic_measure(y):
    """Simulate both ultrasonic sensors with noise and range clipping."""
    d_left, d_right = true_wall_distances(y)
    rng = np.random.default_rng()  # seeded externally in run_simulation
    meas = {}
    for name, d in (('left', d_left), ('right', d_right)):
        noisy = d + rng.normal(0.0, SIGMA_RANGE)
        meas[name] = noisy if RANGE_MIN <= noisy <= RANGE_MAX else None
    return meas


def complementary_filter(y_hat, vy, meas, aisle_idx):
    """
    Fuse dead-reckoning prediction with wall range measurement.

    Parameters
    ----------
    y_hat    : float — current filtered Y estimate
    vy       : float — commanded Y velocity (m/s)
    meas     : dict  — {'left': float|None, 'right': float|None}
    aisle_idx: int   — current aisle (0-indexed)

    Returns
    -------
    y_new    : float — updated filtered Y estimate
    y_dr     : float — dead-reckoning prediction (for logging)
    y_wall   : float|None — wall-derived Y (for logging)
    """
    noise_y = np.random.normal(0.0, SIGMA_DRIFT * np.sqrt(DT))
    y_dr = y_hat + vy * DT + noise_y

    # Wall-derived Y estimates
    y_wall_est = []
    if meas['left'] is not None:
        y_wall_est.append(ROW_Y[aisle_idx] + ROW_THICK / 2 + meas['left'])
    if meas['right'] is not None:
        y_wall_est.append(ROW_Y[aisle_idx + 1] - ROW_THICK / 2 - meas['right'])

    if y_wall_est:
        y_wall = float(np.mean(y_wall_est))
        y_new  = ALPHA * y_dr + (1.0 - ALPHA) * y_wall
    else:
        y_wall = None
        y_new  = y_dr   # pure dead reckoning when out of range

    return y_new, y_dr, y_wall


def lateral_controller(y_hat, aisle_idx, y_true):
    """Compute commanded lateral velocity with repulsion."""
    yc = aisle_centre(aisle_idx)
    ey = y_hat - yc
    vy_cmd = -KP_LAT * ey

    # Repulsion from nearest walls
    d_left, d_right = true_wall_distances(y_true)
    if d_left < D_SAFE:
        vy_cmd += K_REP * (D_SAFE - d_left) / D_SAFE
    if d_right < D_SAFE:
        vy_cmd -= K_REP * (D_SAFE - d_right) / D_SAFE

    return vy_cmd


def run_simulation(seed=42):
    """
    Full boustrophedon greenhouse mission simulation.

    Returns
    -------
    traj_true : (N, 3) ground-truth trajectory
    traj_est  : (N, 3) filtered-estimate trajectory
    t_vec     : (N,)   time vector
    log       : dict   with drift error, wall distances, filter components
    """
    rng = np.random.default_rng(seed)
    np.random.seed(seed)   # for normal() calls inside helpers

    waypoints  = build_waypoints()
    wp_idx     = 0
    aisle_idx  = 0

    # State
    p_true  = np.array([0.0, aisle_centre(0), Z_CRUISE])
    p_est   = p_true.copy()   # filter starts at truth (known start pose)

    traj_true, traj_est, t_vec = [p_true.copy()], [p_est.copy()], [0.0]
    log = {'e_lat': [], 'd_left': [], 'd_right': [], 'y_dr': [], 'y_wall': []}

    t = 0.0
    row_ends_visited = []

    while t < T_MAX and wp_idx < len(waypoints):
        wp = waypoints[wp_idx]

        # ── Compute velocity commands ──────────────────────────────────────────
        # Forward: proportional to distance along X to waypoint
        dx = wp[0] - p_true[0]
        vx = np.sign(dx) * INSP_SPEED if abs(dx) > D_WP else 0.0

        # Lateral: complementary-filter-based controller
        meas = {'left': None, 'right': None}
        d_l, d_r = true_wall_distances(p_true[1])
        for name, d in (('left', d_l), ('right', d_r)):
            noisy = d + rng.normal(0.0, SIGMA_RANGE)
            meas[name] = noisy if RANGE_MIN <= noisy <= RANGE_MAX else None

        vy_cmd = lateral_controller(p_est[1], aisle_idx, p_true[1])

        # Altitude: proportional hold
        vz = KP_ALT * (Z_CRUISE - p_true[2])

        # ── Propagate true position ────────────────────────────────────────────
        drift = rng.normal(0.0, SIGMA_DRIFT * np.sqrt(DT), size=3)
        p_true = p_true + np.array([vx, vy_cmd, vz]) * DT + drift
        # Hard clamp to greenhouse interior
        p_true[0] = np.clip(p_true[0], 0.0, GH_X)
        p_true[1] = np.clip(p_true[1], 0.02, GH_Y - 0.02)
        p_true[2] = np.clip(p_true[2], 0.05, GH_Z - 0.05)

        # ── Propagate filter estimate ──────────────────────────────────────────
        y_new, y_dr, y_wall = complementary_filter(
            p_est[1], vy_cmd, meas, aisle_idx)
        p_est = np.array([
            p_true[0],   # X is well-observed (odometry, not drifted laterally)
            y_new,
            p_true[2]    # Z controlled to fixed cruise altitude
        ])

        # ── Logging ───────────────────────────────────────────────────────────
        e_lat = abs(p_est[1] - p_true[1])
        log['e_lat'].append(e_lat)
        log['d_left'].append(d_l)
        log['d_right'].append(d_r)
        log['y_dr'].append(y_dr)
        log['y_wall'].append(y_wall if y_wall is not None else np.nan)

        traj_true.append(p_true.copy())
        traj_est.append(p_est.copy())
        t_vec.append(t)
        t += DT

        # ── Waypoint check ────────────────────────────────────────────────────
        if np.linalg.norm(p_true - wp) < D_WP:
            row_ends_visited.append((t, wp_idx, p_true.copy()))
            wp_idx += 1
            if wp_idx < len(waypoints):
                # Advance to next aisle
                aisle_idx = min(aisle_idx + 1, 3)

    traj_true = np.array(traj_true)
    traj_est  = np.array(traj_est)
    t_vec     = np.array(t_vec)

    print(f"Row-ends visited : {len(row_ends_visited)} / {len(waypoints)}")
    print(f"Mean lateral error: {np.mean(log['e_lat']):.4f} m")
    print(f"Peak lateral error: {np.max(log['e_lat']):.4f} m")
    print(f"Min wall clearance: "
          f"{min(min(log['d_left']), min(log['d_right'])):.3f} m")
    for i, (t_wp, idx, pos) in enumerate(row_ends_visited):
        print(f"  WP{i+1} reached at t={t_wp:.1f}s  pos=({pos[0]:.1f}, {pos[1]:.2f}, {pos[2]:.2f})")

    return traj_true, traj_est, t_vec, log


def plot_3d_trajectory(traj_true, traj_est):
    """3D flight path with plant-row walls and greenhouse bounding box."""
    fig = plt.figure(figsize=(14, 7))
    ax = fig.add_subplot(111, projection='3d')

    # Plant row walls (grey slabs)
    for ry in ROW_Y:
        xs = [0, GH_X, GH_X, 0, 0]
        ys = [ry, ry, ry, ry, ry]
        zs = [0, 0, ROW_H, ROW_H, 0]
        ax.plot(xs, ys, zs, color='grey', alpha=0.4, linewidth=1)
        # Filled face via poly collection
        from mpl_toolkits.mplot3d.art3d import Poly3DCollection
        verts = [list(zip([0, GH_X, GH_X, 0],
                          [ry]*4,
                          [0, 0, ROW_H, ROW_H]))]
        poly = Poly3DCollection(verts, alpha=0.15, facecolor='grey', edgecolor='grey')
        ax.add_collection3d(poly)

    # Trajectories
    ax.plot(traj_true[:, 0], traj_true[:, 1], traj_true[:, 2],
            color='steelblue', linewidth=1.2, label='True path')
    ax.plot(traj_est[:, 0], traj_est[:, 1], traj_est[:, 2],
            color='red', linewidth=1.0, linestyle='--', label='Estimated path')

    # Start / end markers
    ax.scatter(*traj_true[0], color='green', s=80, zorder=5, label='Start')
    ax.scatter(*traj_true[-1], color='red', s=80, marker='x', zorder=5, label='End')

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('S064 Greenhouse Boustrophedon Flight — 3D View')
    ax.legend(loc='upper left', fontsize=8)
    ax.set_xlim(0, GH_X)
    ax.set_ylim(0, GH_Y)
    ax.set_zlim(0, GH_Z)
    plt.tight_layout()
    plt.savefig('outputs/04_industrial_agriculture/s064_greenhouse/s064_3d_trajectory.png', dpi=150)
    plt.show()


def plot_analysis(t_vec, log, traj_true, traj_est):
    """
    Four-panel analysis figure:
      (a) Lateral position — true vs estimated vs aisle centres
      (b) Complementary filter components (DR, wall, fused)
      (c) Wall clearance over time with d_safe threshold
      (d) Cumulative mean lateral error
    """
    fig, axes = plt.subplots(2, 2, figsize=(14, 8))
    t = t_vec[:len(log['e_lat'])]

    # (a) Lateral position comparison
    ax = axes[0, 0]
    ax.plot(t, traj_true[:len(t), 1], color='steelblue', label='True Y', linewidth=1.2)
    ax.plot(t, traj_est[:len(t), 1],  color='red', linestyle='--',
            label='Estimated Y', linewidth=1.0)
    for ry in ROW_Y:
        ax.axhline(ry, color='grey', linewidth=0.8, linestyle=':')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Y position (m)')
    ax.set_title('(a) Lateral Position: True vs Estimated')
    ax.legend(fontsize=8)
    ax.set_ylim(0, GH_Y)

    # (b) Filter components
    ax = axes[0, 1]
    ax.plot(t, log['y_dr'], color='orange', alpha=0.6, label='Dead reckoning', linewidth=0.8)
    ax.plot(t, log['y_wall'], color='cyan', alpha=0.7, label='Wall-derived', linewidth=0.8,
            linestyle='-.')
    ax.plot(t, traj_est[:len(t), 1], color='red', label='Fused (filter)', linewidth=1.2)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Y estimate (m)')
    ax.set_title('(b) Complementary Filter Components')
    ax.legend(fontsize=8)
    ax.set_ylim(0, GH_Y)

    # (c) Wall clearance
    ax = axes[1, 0]
    ax.plot(t, log['d_left'],  color='royalblue', label='Left clearance', linewidth=1.0)
    ax.plot(t, log['d_right'], color='tomato',    label='Right clearance', linewidth=1.0)
    ax.axhline(D_SAFE, color='black', linestyle='--', linewidth=1.0, label=f'd_safe={D_SAFE} m')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Distance to wall (m)')
    ax.set_title('(c) Wall Clearance vs Time')
    ax.legend(fontsize=8)
    ax.set_ylim(0, 1.2)

    # (d) Cumulative mean lateral error
    ax = axes[1, 1]
    cum_mean = np.cumsum(log['e_lat']) / (np.arange(len(log['e_lat'])) + 1)
    ax.plot(t, cum_mean, color='purple', linewidth=1.2, label='Cumulative mean |e_y|')
    ax.plot(t, log['e_lat'], color='purple', alpha=0.25, linewidth=0.6, label='Instant |e_y|')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Lateral error (m)')
    ax.set_title('(d) Lateral Position Error (Filter vs Truth)')
    ax.legend(fontsize=8)

    plt.suptitle('S064 Greenhouse Interior Precision Flight — Analysis', fontsize=12)
    plt.tight_layout()
    plt.savefig('outputs/04_industrial_agriculture/s064_greenhouse/s064_analysis.png', dpi=150)
    plt.show()


def animate_topdown(traj_true, traj_est, t_vec, log, interval=40):
    """
    Top-down (X–Y) animation of the boustrophedon sweep.
    Drone icon moves in real time; estimated position shown as a dashed
    red circle; wall-sensor beam segments drawn in cyan.
    """
    fig, ax = plt.subplots(figsize=(14, 5))

    # Static: plant rows
    for ry in ROW_Y:
        ax.add_patch(plt.Rectangle((0, ry - ROW_THICK / 2), GH_X, ROW_THICK,
                                   color='grey', alpha=0.35, zorder=1))
    # Greenhouse boundary
    ax.set_xlim(-1, GH_X + 1)
    ax.set_ylim(-0.5, GH_Y + 0.5)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('S064 Greenhouse Flight — Top-Down Animation')
    ax.set_aspect('equal')

    # Dynamic elements
    trail_true, = ax.plot([], [], color='steelblue', linewidth=1.2, label='True path', zorder=3)
    trail_est,  = ax.plot([], [], color='red', linestyle='--', linewidth=0.9,
                          label='Estimated path', zorder=3)
    drone_dot,  = ax.plot([], [], 'o', color='steelblue', markersize=8, zorder=5)
    est_dot,    = ax.plot([], [], 's', color='red', markersize=6, zorder=5, alpha=0.7)
    sensor_l,   = ax.plot([], [], color='cyan', linewidth=1.5, alpha=0.8, zorder=4)
    sensor_r,   = ax.plot([], [], color='cyan', linewidth=1.5, alpha=0.8, zorder=4)
    ax.legend(loc='upper right', fontsize=8)

    N = len(traj_true)
    step = max(1, N // 400)   # target ~400 animation frames

    def init():
        for artist in (trail_true, trail_est, drone_dot, est_dot, sensor_l, sensor_r):
            artist.set_data([], [])
        return trail_true, trail_est, drone_dot, est_dot, sensor_l, sensor_r

    def update(frame):
        i = frame * step
        i = min(i, N - 1)
        trail_true.set_data(traj_true[:i, 0], traj_true[:i, 1])
        trail_est.set_data(traj_est[:i, 0],  traj_est[:i, 1])
        drone_dot.set_data([traj_true[i, 0]], [traj_true[i, 1]])
        est_dot.set_data([traj_est[i, 0]],   [traj_est[i, 1]])

        # Draw ultrasonic beams
        x0, y0 = traj_true[i, 0], traj_true[i, 1]
        dl = log['d_left'][min(i, len(log['d_left']) - 1)]
        dr = log['d_right'][min(i, len(log['d_right']) - 1)]
        sensor_l.set_data([x0, x0], [y0, y0 - dl])
        sensor_r.set_data([x0, x0], [y0, y0 + dr])
        return trail_true, trail_est, drone_dot, est_dot, sensor_l, sensor_r

    n_frames = N // step
    anim = FuncAnimation(fig, update, frames=n_frames, init_func=init,
                         interval=interval, blit=True)
    anim.save('outputs/04_industrial_agriculture/s064_greenhouse/s064_animation.gif',
              writer='pillow', fps=25)
    plt.show()
    return anim


if __name__ == '__main__':
    import os
    os.makedirs('outputs/04_industrial_agriculture/s064_greenhouse', exist_ok=True)

    traj_true, traj_est, t_vec, log = run_simulation(seed=42)
    plot_3d_trajectory(traj_true, traj_est)
    plot_analysis(t_vec, log, traj_true, traj_est)
    animate_topdown(traj_true, traj_est, t_vec, log)
```

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Greenhouse dimensions | 40 × 10 × 3 m |
| Number of plant rows | 5 |
| Row Y positions | 2, 4, 6, 8, 10 m |
| Row wall thickness | 0.2 m |
| Row wall height | 1.5 m |
| Cruise altitude $z_{cruise}$ | 1.0 m |
| Cruise speed $v_x$ | 1.0 m/s |
| IMU drift noise $\sigma_{drift}$ | 0.02 m/s |
| Ultrasonic noise $\sigma_{range}$ | 0.05 m |
| Ultrasonic range band | [0.1, 2.0] m |
| Complementary filter gain $\alpha$ | 0.85 |
| Lateral proportional gain $K_p$ | 0.8 s⁻¹ |
| Repulsion gain $K_{rep}$ | 1.5 m/s |
| Safety clearance $d_{safe}$ | 0.25 m |
| Waypoint capture radius $d_{wp}$ | 0.5 m |
| Mission horizon $T_{max}$ | 300 s |
| Simulation timestep $\Delta t$ | 0.05 s |
| Start position $\mathbf{p}_0$ | (0, 1.0, 1.0) m |
| GPS error (denied indoors) | 0.1 m (N/A) |
| Inspection stand-off $d_{insp}$ | 0.5 m |

---

## Expected Output

- **3D trajectory plot** (`s064_3d_trajectory.png`): isometric view of the 40 × 10 × 3 m
  greenhouse; grey semi-transparent slabs show the five plant-row walls; blue polyline is the
  true flight path, red dashed polyline is the filter estimate; green circle at start, red cross
  at end; visible serpentine boustrophedon turns at $x = 40$ m and $x = 0$ m.
- **Four-panel analysis figure** (`s064_analysis.png`):
  - **(a) Lateral position** — true $Y(t)$ (blue) vs filtered estimate $\hat{Y}(t)$ (red dashed);
    horizontal grey dotted lines mark row-wall positions; visible bounded tracking error.
  - **(b) Filter components** — dead-reckoning $Y_{DR}$ (orange, drifting), wall-derived $Y_{wall}$
    (cyan dash-dot), fused output (red solid); shows how the wall corrections suppress drift growth.
  - **(c) Wall clearance** — left (blue) and right (red) distances to nearest plant wall over time;
    horizontal dashed line at $d_{safe} = 0.25$ m; no crossings indicate a collision-free mission.
  - **(d) Lateral error** — instantaneous $|\hat{y} - y_{true}|$ (light purple) and cumulative
    mean (solid purple); expected cumulative mean $< 0.05$ m with fusion active.
- **Top-down animation** (`s064_animation.gif`): X–Y plane view; grey rectangles show plant rows;
  blue trail is true path, red dashed trail is filter estimate; drone icon (blue dot) and estimate
  icon (red square) move in sync; cyan line segments radiate left/right to show live ultrasonic
  sensor range readings at each frame; 25 fps, ~400 frames.
- **Stdout metrics** — row-ends visited count, mean and peak lateral position error, minimum wall
  clearance recorded during the mission, and a per-waypoint log of arrival time and position.

**Typical expected metrics** (seed 42):
- Row-ends visited: 4 / 4
- Mean lateral error: ~0.02 – 0.04 m
- Peak lateral error: ~0.08 – 0.12 m
- Minimum wall clearance: $> d_{safe}$ (collision-free)

---

## Extensions

1. **EKF lateral estimator**: replace the complementary filter with a full scalar EKF that
   propagates a Gaussian state $(y, v_y)$ with IMU as the prediction step and wall range as the
   measurement update; compare RMSE against the complementary filter baseline across 100 Monte
   Carlo trials with varying drift levels.
2. **Variable row spacing**: relax the assumption of uniform 2 m row spacing; generate random
   row spacings $\sim \mathcal{U}(1.2, 2.5)$ m; the aisle-centre estimator must infer spacing
   online from both sensor readings rather than relying on a pre-loaded map.
3. **Plant protrusion obstacles**: model individual plant canopies as randomly placed spheres
   (radius 0.15 – 0.30 m) extending into the aisle; add a forward-facing distance sensor and
   implement a vertical dodge manoeuvre when a protrusion is detected within 1.0 m.
4. **Multi-drone coordination**: deploy two drones starting from opposite ends of the greenhouse
   ($x = 0$ and $x = 40$ m); assign disjoint aisle sets; add a collision-avoidance layer that
   holds the trailing drone when predicted inter-drone separation drops below 1.5 m.
5. **Fault injection — sensor outage**: simulate a blocked ultrasonic sensor (one side returns no
   valid reading for a 10 s window at a random time); measure the resulting drift excursion and
   implement an outage-aware fall-back that widens the safety buffer automatically when only one
   sensor is active.

---

## Related Scenarios

- Prerequisites: [S061 Power Line Inspection](S061_powerline_inspection.md) (structured linear inspection path), [S063 Precision Per-Plant Spraying](S063_per_plant_spraying.md) (row-based navigation)
- Follow-ups: [S069 Automated Warehouse Inventory](S069_warehouse_inventory.md) (GPS-denied indoor 3D flight), [S074 Mine 3D Mapping](S074_mine_mapping.md) (dead-reckoning in large GPS-denied tunnels)
- Algorithmic cross-reference: [S043 Confined Space Exploration](../03_environmental_sar/S043_confined_space.md) (indoor navigation without GPS), [S050 Distributed EKF-SLAM](../03_environmental_sar/S050_slam.md) (sensor-fusion localisation), [S046 Trilateration Localisation](../03_environmental_sar/S046_trilateration.md) (range-based position estimation)
