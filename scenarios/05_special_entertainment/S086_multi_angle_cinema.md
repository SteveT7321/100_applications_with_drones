# S086 Cooperative Multi-Angle Filming

**Domain**: Special Ops & Entertainment | **Difficulty**: ⭐⭐ | **Status**: `[ ]` Not Started
**Algorithm**: Phase-Locked Orbit Formation + KF Subject Tracking | **Dimension**: 3D

---

## Problem Definition

**Setup**: Three camera drones simultaneously orbit a walking subject moving along a
figure-8 path at $v = 1.5$ m/s. Each drone maintains a fixed orbital radius of $r = 6$ m
around the subject, but at a different angular phase: $0°$, $120°$, and $240°$. This
provides three simultaneous, evenly-spaced camera perspectives at all times. The subject's
position is not measured directly — each drone uses a shared Kalman Filter (KF) that fuses
GPS-class position observations to produce a smooth, lag-compensated estimate of where the
subject is and where they will be next.

**Roles**:
- **Subject**: human walking at $v = 1.5$ m/s on a figure-8 path of half-axis
  $a = 10$ m, $b = 6$ m; acts as the shared tracking target.
- **Camera drones** ($N = 3$): each flies at a fixed altitude $z_{fly} = 4$ m, orbiting
  the KF-estimated subject position at radius $r = 6$ m with a prescribed phase offset
  $\varphi_k = 2\pi k/3$ for $k = 0, 1, 2$.
- **Shared KF** (centralised): receives subject observations every $\Delta t_{obs} = 0.2$ s
  and broadcasts state estimates to all three drones.

**Objective**: Maintain continuous, stable three-angle coverage of the subject with:

1. **Phase synchronisation**: each drone's actual phase relative to the subject must track
   its desired phase to within $|\Delta\varphi_k| \leq 5°$.
2. **Collision avoidance**: all pairwise inter-drone distances must remain $\geq d_{min} = 3$ m
   at all times.
3. **Framing quality**: the subject must stay within $\pm 15\%$ of the frame centre for each
   camera, measured as a normalised offset in the image plane.

**Domain constants**: `FORMATION_DIST = 2.0 m`, `COLLISION_RADIUS = 0.5 m`.

---

## Mathematical Model

### Subject Trajectory (Figure-8)

The subject moves on a Lissajous figure-8 parametrised by arc-length time $t$:

$$\mathbf{q}(t) = \bigl(a \sin(\omega t),\; b \sin(2\omega t),\; 0\bigr)^\top$$

where $a = 10$ m, $b = 6$ m, and angular frequency $\omega$ is chosen so that the RMS
path speed equals $v_{sub} = 1.5$ m/s. The subject's velocity is:

$$\dot{\mathbf{q}}(t) = \bigl(a\omega\cos(\omega t),\; 2b\omega\cos(2\omega t),\; 0\bigr)^\top$$

### Shared Kalman Filter for Subject Tracking

The KF models the subject as a constant-velocity (CV) point mass in $x$-$y$. State vector:

$$\mathbf{s} = (x,\; y,\; \dot{x},\; \dot{y})^\top \in \mathbb{R}^4$$

**Prediction step** with time step $\Delta t$:

$$\mathbf{s}_{k|k-1} = \mathbf{F}\,\mathbf{s}_{k-1|k-1}, \qquad
\mathbf{F} = \begin{pmatrix} 1 & 0 & \Delta t & 0 \\ 0 & 1 & 0 & \Delta t \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{pmatrix}$$

$$\mathbf{P}_{k|k-1} = \mathbf{F}\,\mathbf{P}_{k-1|k-1}\,\mathbf{F}^\top + \mathbf{Q}$$

Process noise covariance (discrete Wiener model, acceleration noise $\sigma_a$):

$$\mathbf{Q} = \sigma_a^2 \begin{pmatrix} \tfrac{\Delta t^4}{4} & 0 & \tfrac{\Delta t^3}{2} & 0 \\ 0 & \tfrac{\Delta t^4}{4} & 0 & \tfrac{\Delta t^3}{2} \\ \tfrac{\Delta t^3}{2} & 0 & \Delta t^2 & 0 \\ 0 & \tfrac{\Delta t^3}{2} & 0 & \Delta t^2 \end{pmatrix}$$

**Update step** when a GPS observation $\mathbf{z}_k = (x_{obs}, y_{obs})^\top$ arrives:

$$\mathbf{H} = \begin{pmatrix} 1 & 0 & 0 & 0 \\ 0 & 1 & 0 & 0 \end{pmatrix}$$

$$\mathbf{K}_k = \mathbf{P}_{k|k-1}\,\mathbf{H}^\top \bigl(\mathbf{H}\,\mathbf{P}_{k|k-1}\,\mathbf{H}^\top + \mathbf{R}\bigr)^{-1}$$

$$\mathbf{s}_{k|k} = \mathbf{s}_{k|k-1} + \mathbf{K}_k\,(\mathbf{z}_k - \mathbf{H}\,\mathbf{s}_{k|k-1})$$

$$\mathbf{P}_{k|k} = (\mathbf{I} - \mathbf{K}_k\,\mathbf{H})\,\mathbf{P}_{k|k-1}$$

Observation noise covariance: $\mathbf{R} = \sigma_{obs}^2\,\mathbf{I}_2$ with $\sigma_{obs} = 0.3$ m.

### Phase-Locked Orbit Formation

Let $\hat{\mathbf{q}}(t) = (\hat{x}, \hat{y})^\top$ be the KF position estimate of the subject.
The global base phase is driven by the subject's instantaneous heading:

$$\varphi_{base}(t) = \operatorname{atan2}\!\bigl(\dot{\hat{y}},\; \dot{\hat{x}}\bigr)$$

The desired phase for drone $k \in \{0, 1, 2\}$ is:

$$\varphi_k^{des}(t) = \varphi_{base}(t) + \frac{2\pi k}{3}$$

The desired 3D orbit position for drone $k$ is:

$$\mathbf{p}_k^{des}(t) = \begin{pmatrix} \hat{x}(t) + r\cos\!\bigl(\varphi_k^{des}(t)\bigr) \\ \hat{y}(t) + r\sin\!\bigl(\varphi_k^{des}(t)\bigr) \\ z_{fly} \end{pmatrix}$$

The drone tracks this setpoint with a proportional position controller:

$$\mathbf{u}_k(t) = K_p\,\bigl(\mathbf{p}_k^{des}(t) - \mathbf{p}_k(t)\bigr)$$

with drone dynamics approximated as a first-order lag:

$$\dot{\mathbf{p}}_k = \mathbf{u}_k, \qquad \|\mathbf{u}_k\|_\infty \leq v_{max}$$

### Phase Synchronisation Error

The actual phase of drone $k$ relative to the subject is:

$$\varphi_k^{act}(t) = \operatorname{atan2}\!\bigl(p_{k,y}(t) - \hat{y}(t),\; p_{k,x}(t) - \hat{x}(t)\bigr)$$

The phase tracking error for drone $k$:

$$\Delta\varphi_k(t) = \bigl|\varphi_k^{act}(t) - \varphi_k^{des}(t)\bigr|_{\text{wrap}}$$

where $|\cdot|_{\text{wrap}}$ denotes the principal value in $(-\pi, \pi]$. The pairwise
phase separation error between drones $i$ and $j$ (target: $2\pi/3$):

$$\Delta\varphi_{ij}(t) = \Bigl|\varphi_i^{act}(t) - \varphi_j^{act}(t) - \tfrac{2\pi}{3}\Bigr|_{\text{wrap}}$$

### Inter-Drone Distance Constraint

All pairwise distances must satisfy the separation constraint at all times:

$$d_{ij}(t) = \bigl\|\mathbf{p}_i(t) - \mathbf{p}_j(t)\bigr\|_2 \geq d_{min} = 3\text{ m}$$

For the ideal symmetric formation (all drones at exactly $r = 6$ m, evenly spaced at $120°$):

$$d_{ij}^{ideal} = 2\,r\,\sin\!\Bigl(\frac{\pi}{3}\Bigr) = r\sqrt{3} \approx 10.39\text{ m}$$

Any transient narrowing below $d_{min}$ is a collision-avoidance violation.

### Framing Score

Each drone's camera is assumed to have a field-of-view such that the subject projects to a
normalised image-plane offset $\delta_k \in [-1, 1]^2$. The per-drone framing score at time $t$:

$$F_k(t) = \begin{cases} 1 & \text{if } \|\delta_k(t)\|_\infty \leq 0.15 \\ 0 & \text{otherwise} \end{cases}$$

where the normalised offset is:

$$\delta_k(t) = \frac{\mathbf{q}_{proj,k}(t)}{r_{fov}}$$

with $r_{fov}$ the half-width of the field of view at orbit radius $r$. The fleet-wide
framing score averaged over time and drones:

$$\bar{F} = \frac{1}{3\,T} \int_0^T \sum_{k=0}^{2} F_k(t)\,dt$$

Target: $\bar{F} \geq 0.90$.

---

## Implementation

```python
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation

# ── Scenario constants ─────────────────────────────────────────────────────────
N_DRONES       = 3
ORBIT_RADIUS   = 6.0          # m — orbit radius around subject
FLY_ALT        = 4.0          # m — constant flight altitude
SUBJECT_SPEED  = 1.5          # m/s — walking speed (RMS target)
FIG8_A         = 10.0         # m — figure-8 semi-axis x
FIG8_B         = 6.0          # m — figure-8 semi-axis y
FIG8_OMEGA     = 0.18         # rad/s — tuned to hit ~1.5 m/s RMS
D_MIN          = 3.0          # m — minimum inter-drone separation
FORMATION_DIST = 2.0          # m — domain constant
COLLISION_RAD  = 0.5          # m — domain constant

# ── Controller & integrator ────────────────────────────────────────────────────
KP             = 2.0          # proportional gain (position controller)
V_MAX          = 4.0          # m/s — drone speed limit
DT             = 0.05         # s — simulation time step
T_SIM          = 60.0         # s — total simulation duration
OBS_INTERVAL   = 0.2          # s — KF observation period

# ── Kalman filter ──────────────────────────────────────────────────────────────
SIGMA_A        = 0.8          # m/s² — process noise (acceleration)
SIGMA_OBS      = 0.3          # m — observation noise std dev

# ── Framing ───────────────────────────────────────────────────────────────────
FRAME_THRESH   = 0.15         # normalised offset threshold for good framing
FOV_HALF_WIDTH = 2.0          # m — half-FOV projected at orbit radius

# ── Subject trajectory ─────────────────────────────────────────────────────────
def subject_pos(t):
    """Figure-8 position of the walking subject."""
    x = FIG8_A * np.sin(FIG8_OMEGA * t)
    y = FIG8_B * np.sin(2 * FIG8_OMEGA * t)
    return np.array([x, y, 0.0])

def subject_vel(t):
    """Analytical velocity of the walking subject."""
    vx = FIG8_A * FIG8_OMEGA * np.cos(FIG8_OMEGA * t)
    vy = 2 * FIG8_B * FIG8_OMEGA * np.cos(2 * FIG8_OMEGA * t)
    return np.array([vx, vy, 0.0])

# ── Kalman Filter ──────────────────────────────────────────────────────────────
class SubjectKF:
    """2-D constant-velocity Kalman filter for subject tracking."""

    def __init__(self, dt, sigma_a, sigma_obs):
        self.dt   = dt
        self.F    = np.array([[1, 0, dt, 0],
                              [0, 1, 0,  dt],
                              [0, 0, 1,  0],
                              [0, 0, 0,  1]], dtype=float)
        self.H    = np.array([[1, 0, 0, 0],
                              [0, 1, 0, 0]], dtype=float)
        self.R    = sigma_obs**2 * np.eye(2)
        dt4, dt3, dt2 = dt**4, dt**3, dt**2
        self.Q    = sigma_a**2 * np.array(
            [[dt4/4, 0,     dt3/2, 0    ],
             [0,     dt4/4, 0,     dt3/2],
             [dt3/2, 0,     dt2,   0    ],
             [0,     dt3/2, 0,     dt2  ]])
        self.x    = np.zeros(4)   # state: [x, y, vx, vy]
        self.P    = np.eye(4) * 1.0

    def predict(self):
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, z):
        """Update with 2-D position observation z = [x_obs, y_obs]."""
        S    = self.H @ self.P @ self.H.T + self.R
        K    = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ (z - self.H @ self.x)
        self.P = (np.eye(4) - K @ self.H) @ self.P

    @property
    def pos_est(self):
        return self.x[:2]

    @property
    def vel_est(self):
        return self.x[2:]

# ── Phase & formation helpers ─────────────────────────────────────────────────
def desired_phase(k, base_phase):
    """Desired orbital phase for drone k."""
    return base_phase + 2 * np.pi * k / N_DRONES

def desired_position(k, subject_xy, base_phase):
    """3D desired position for drone k on the orbit."""
    phi = desired_phase(k, base_phase)
    return np.array([
        subject_xy[0] + ORBIT_RADIUS * np.cos(phi),
        subject_xy[1] + ORBIT_RADIUS * np.sin(phi),
        FLY_ALT
    ])

def wrap_angle(a):
    """Wrap angle to (-pi, pi]."""
    return (a + np.pi) % (2 * np.pi) - np.pi

def framing_score(drone_pos, subject_pos_3d):
    """1 if subject is within ±15% of frame centre, else 0."""
    dx = subject_pos_3d[0] - drone_pos[0]
    dy = subject_pos_3d[1] - drone_pos[1]
    offset = np.array([dx, dy]) / FOV_HALF_WIDTH
    return 1.0 if np.max(np.abs(offset)) <= FRAME_THRESH else 0.0

# ── Main simulation loop ───────────────────────────────────────────────────────
def run_simulation():
    rng      = np.random.default_rng(42)
    kf       = SubjectKF(DT, SIGMA_A, SIGMA_OBS)
    n_steps  = int(T_SIM / DT)

    # Initialise KF from true position
    q0       = subject_pos(0.0)
    kf.x[:2] = q0[:2]

    # Initialise drone positions at t=0 with ideal phases
    drone_pos = np.array([
        desired_position(k, q0[:2], 0.0)
        for k in range(N_DRONES)
    ], dtype=float)

    # Storage
    times          = np.arange(n_steps) * DT
    sub_true       = np.zeros((n_steps, 3))
    sub_est        = np.zeros((n_steps, 2))
    drone_hist     = np.zeros((n_steps, N_DRONES, 3))
    phase_err      = np.zeros((n_steps, N_DRONES))     # per-drone phase error
    pair_sep       = np.zeros((n_steps, 3))             # d_01, d_02, d_12
    frame_scores   = np.zeros((n_steps, N_DRONES))

    obs_timer = 0.0

    for i, t in enumerate(times):
        q_true = subject_pos(t)
        v_true = subject_vel(t)

        # KF predict
        kf.predict()

        # KF update when observation is due
        obs_timer += DT
        if obs_timer >= OBS_INTERVAL:
            z = q_true[:2] + rng.normal(0, SIGMA_OBS, size=2)
            kf.update(z)
            obs_timer = 0.0

        sub_xy  = kf.pos_est
        vel_xy  = kf.vel_est
        base_ph = np.arctan2(vel_xy[1], vel_xy[0])

        # Control each drone
        for k in range(N_DRONES):
            p_des     = desired_position(k, sub_xy, base_ph)
            err       = p_des - drone_pos[k]
            speed     = np.linalg.norm(err)
            if speed > 0:
                u = KP * err
                if np.linalg.norm(u) > V_MAX:
                    u = u / np.linalg.norm(u) * V_MAX
            else:
                u = np.zeros(3)
            drone_pos[k] += u * DT

            # Actual phase
            ph_act = np.arctan2(drone_pos[k, 1] - sub_xy[1],
                                drone_pos[k, 0] - sub_xy[0])
            ph_des = desired_phase(k, base_ph)
            phase_err[i, k] = np.abs(wrap_angle(ph_act - ph_des))
            frame_scores[i, k] = framing_score(drone_pos[k], q_true)

        # Pairwise separations
        pair_sep[i, 0] = np.linalg.norm(drone_pos[0] - drone_pos[1])
        pair_sep[i, 1] = np.linalg.norm(drone_pos[0] - drone_pos[2])
        pair_sep[i, 2] = np.linalg.norm(drone_pos[1] - drone_pos[2])

        sub_true[i]    = q_true
        sub_est[i]     = sub_xy
        drone_hist[i]  = drone_pos.copy()

    return times, sub_true, sub_est, drone_hist, phase_err, pair_sep, frame_scores


def plot_results(times, sub_true, sub_est, drone_hist, phase_err, pair_sep, frame_scores):
    COLORS = ['red', 'orangered', 'firebrick']
    SUB_C  = 'blue'

    # ── Plot 1: 3D formation snapshot ─────────────────────────────────────────
    fig1 = plt.figure(figsize=(10, 8))
    ax1  = fig1.add_subplot(111, projection='3d')
    ax1.plot(sub_true[:, 0], sub_true[:, 1], sub_true[:, 2],
             color=SUB_C, lw=1.5, label='Subject path', alpha=0.5)
    ax1.scatter(*sub_true[-1], color='green', s=120, zorder=5, label='Subject (final)')
    for k in range(N_DRONES):
        ax1.plot(drone_hist[:, k, 0], drone_hist[:, k, 1], drone_hist[:, k, 2],
                 color=COLORS[k], lw=1.2, alpha=0.6, label=f'Drone {k} path')
        ax1.scatter(*drone_hist[-1, k], color=COLORS[k], s=100, marker='^', zorder=5)
    ax1.set_xlabel('X (m)'); ax1.set_ylabel('Y (m)'); ax1.set_zlabel('Z (m)')
    ax1.set_title('S086 — 3D Formation Trajectories')
    ax1.legend(loc='upper left', fontsize=8)
    plt.tight_layout()
    plt.savefig('outputs/05_special_entertainment/s086_multi_angle_cinema/s086_3d_formation.png',
                dpi=150)

    # ── Plot 2: Phase synchronisation error ───────────────────────────────────
    fig2, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    for k in range(N_DRONES):
        axes[0].plot(times, np.degrees(phase_err[:, k]),
                     color=COLORS[k], lw=1.2, label=f'Drone {k}')
    axes[0].axhline(5.0, color='gray', ls='--', lw=1, label='5° threshold')
    axes[0].set_ylabel('Phase error (°)')
    axes[0].set_title('Phase Tracking Error per Drone')
    axes[0].legend(); axes[0].grid(True, alpha=0.3)

    labels_pair = ['d(0,1)', 'd(0,2)', 'd(1,2)']
    colors_pair = ['purple', 'teal', 'darkorange']
    for j in range(3):
        axes[1].plot(times, pair_sep[:, j],
                     color=colors_pair[j], lw=1.2, label=labels_pair[j])
    axes[1].axhline(D_MIN, color='red', ls='--', lw=1.5, label=f'd_min = {D_MIN} m')
    axes[1].axhline(ORBIT_RADIUS * np.sqrt(3), color='gray', ls=':', lw=1,
                    label=f'Ideal = {ORBIT_RADIUS*np.sqrt(3):.2f} m')
    axes[1].set_xlabel('Time (s)')
    axes[1].set_ylabel('Inter-drone distance (m)')
    axes[1].set_title('Pairwise Drone Separations')
    axes[1].legend(); axes[1].grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig('outputs/05_special_entertainment/s086_multi_angle_cinema/s086_sync_separation.png',
                dpi=150)

    # ── Plot 3: Framing score timeline ────────────────────────────────────────
    fig3, ax3 = plt.subplots(figsize=(12, 4))
    fleet_frame = frame_scores.mean(axis=1)
    ax3.plot(times, fleet_frame, color='steelblue', lw=1.5, label='Fleet mean framing score')
    for k in range(N_DRONES):
        ax3.plot(times, frame_scores[:, k], color=COLORS[k],
                 lw=0.8, alpha=0.5, label=f'Drone {k}')
    ax3.axhline(0.90, color='green', ls='--', lw=1.2, label='Target ≥ 0.90')
    ax3.set_ylim(-0.05, 1.15)
    ax3.set_xlabel('Time (s)'); ax3.set_ylabel('Framing score')
    ax3.set_title('Camera Framing Score (1 = subject in centre ±15%)')
    ax3.legend(fontsize=8); ax3.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig('outputs/05_special_entertainment/s086_multi_angle_cinema/s086_framing_score.png',
                dpi=150)

    # ── Animation: top-down rotating formation ────────────────────────────────
    fig4 = plt.figure(figsize=(8, 8))
    ax4  = fig4.add_subplot(111, projection='3d')

    def animate(frame_idx):
        ax4.cla()
        i = min(frame_idx * 5, len(times) - 1)
        ax4.plot(sub_true[:i+1, 0], sub_true[:i+1, 1], sub_true[:i+1, 2],
                 color=SUB_C, lw=1.5, alpha=0.4)
        ax4.scatter(*sub_true[i], color='green', s=140, zorder=5)
        for k in range(N_DRONES):
            ax4.plot(drone_hist[:i+1, k, 0], drone_hist[:i+1, k, 1], drone_hist[:i+1, k, 2],
                     color=COLORS[k], lw=1, alpha=0.4)
            ax4.scatter(*drone_hist[i, k], color=COLORS[k], s=120, marker='^', zorder=5)
            ax4.plot([drone_hist[i, k, 0], sub_true[i, 0]],
                     [drone_hist[i, k, 1], sub_true[i, 1]],
                     [drone_hist[i, k, 2], 0], color=COLORS[k], ls=':', lw=0.8, alpha=0.6)
        ax4.set_xlim(-18, 18); ax4.set_ylim(-14, 14); ax4.set_zlim(0, 8)
        ax4.set_xlabel('X (m)'); ax4.set_ylabel('Y (m)'); ax4.set_zlabel('Z (m)')
        ax4.set_title(f'S086 Multi-Angle Cinema  t = {times[i]:.1f}s')
        ax4.view_init(elev=30, azim=frame_idx * 1.5)

    n_frames = len(times) // 5
    anim = animation.FuncAnimation(fig4, animate, frames=n_frames,
                                   interval=50, blit=False)
    anim.save('outputs/05_special_entertainment/s086_multi_angle_cinema/s086_animation.gif',
              writer='pillow', fps=20)
    plt.close('all')


if __name__ == '__main__':
    import os
    os.makedirs('outputs/05_special_entertainment/s086_multi_angle_cinema', exist_ok=True)
    results = run_simulation()
    plot_results(*results)

    times, sub_true, sub_est, drone_hist, phase_err, pair_sep, frame_scores = results
    mean_phase_err = np.degrees(phase_err.mean())
    min_sep        = pair_sep.min()
    fleet_frame    = frame_scores.mean()
    print(f"Mean phase tracking error : {mean_phase_err:.2f}°  (target ≤ 5°)")
    print(f"Minimum inter-drone dist  : {min_sep:.2f} m  (must ≥ {D_MIN} m)")
    print(f"Fleet framing score       : {fleet_frame:.3f}  (target ≥ 0.90)")
```

---

## Key Parameters

| Parameter | Symbol | Value |
|-----------|--------|-------|
| Number of camera drones | $N$ | 3 |
| Orbit radius | $r$ | 6.0 m |
| Drone flight altitude | $z_{fly}$ | 4.0 m |
| Subject walking speed (RMS) | $v_{sub}$ | 1.5 m/s |
| Figure-8 semi-axis x | $a$ | 10.0 m |
| Figure-8 semi-axis y | $b$ | 6.0 m |
| Figure-8 angular frequency | $\omega$ | 0.18 rad/s |
| Nominal inter-drone separation (ideal) | $d^{ideal}$ | $r\sqrt{3} \approx 10.39$ m |
| Minimum inter-drone distance | $d_{min}$ | 3.0 m |
| Phase offset between drones | $\Delta\varphi$ | $120°$ ($2\pi/3$ rad) |
| Phase error tolerance | $\|\Delta\varphi_k\|$ | $\leq 5°$ |
| Framing threshold | — | $\pm 15\%$ of frame centre |
| Target fleet framing score | $\bar{F}$ | $\geq 0.90$ |
| KF process noise std dev | $\sigma_a$ | 0.8 m/s² |
| KF observation noise std dev | $\sigma_{obs}$ | 0.3 m |
| KF observation interval | $\Delta t_{obs}$ | 0.2 s |
| Controller proportional gain | $K_p$ | 2.0 |
| Drone max speed | $v_{max}$ | 4.0 m/s |
| Simulation time step | $\Delta t$ | 0.05 s |
| Total simulation duration | $T$ | 60.0 s |
| Domain formation distance | `FORMATION_DIST` | 2.0 m |
| Domain collision radius | `COLLISION_RADIUS` | 0.5 m |

---

## Expected Output

- **3D formation trajectories** (`s086_3d_formation.png`): single 3D axes showing the figure-8
  subject path in blue, the three drone helical/orbital paths in shades of red, final positions
  marked with triangles; the symmetry of the 120°-spaced formation is clearly visible from an
  elevated viewpoint.
- **Phase sync and separation plot** (`s086_sync_separation.png`): upper panel shows per-drone
  phase tracking error in degrees versus time, with a dashed 5° threshold line; lower panel shows
  all three pairwise inter-drone distances against the $d_{min} = 3$ m red safety line and the
  ideal $r\sqrt{3} \approx 10.39$ m dashed reference — all curves should stay above $d_{min}$ at
  all times.
- **Framing score timeline** (`s086_framing_score.png`): fleet-mean framing score overlaid with
  individual drone scores; a green dashed line at 0.90 marks the target; brief dips below target
  correspond to tight figure-8 turns where the subject accelerates laterally.
- **3D animation** (`s086_animation.gif`): rotating azimuth view that slowly orbits the formation,
  showing all three drones tracking the walking subject around the figure-8; camera sight-lines
  drawn as dotted lines from each drone to the subject; playback at 20 fps over the full 60-second
  mission.

**Representative metric values** (expected at successful convergence):

| Metric | Expected value |
|--------|---------------|
| Mean phase tracking error | $\leq 3°$ |
| Minimum inter-drone distance | $\geq 8$ m (well above $d_{min}$) |
| Fleet framing score $\bar{F}$ | $\geq 0.92$ |
| KF position RMSE | $\leq 0.5$ m |

---

## Extensions

1. **Altitude-staggered formation**: assign each drone a different altitude
   ($z = 3, 4, 5$ m) to add vertical parallax; re-derive the 3D orbital setpoint and analyse
   the effect on framing score and collision margins when the subject changes speed abruptly.
2. **Adaptive phase assignment**: when one drone falls behind its phase target (e.g., due to wind
   disturbance), reassign phases dynamically to maintain the $120°$ symmetry using a Hungarian
   algorithm on phase-error cost.
3. **Variable orbit radius**: allow each drone to independently adjust $r$ within
   $[4, 10]$ m to keep the subject within a fixed angular field of view regardless of zoom level;
   compare cinematic quality metrics against fixed-radius baseline.
4. **Decentralised KF**: replace the shared centralised KF with three independent drone-local
   filters that exchange only compressed likelihood summaries; quantify the state-estimate
   divergence penalty.
5. **Obstacle-aware phase routing**: introduce a stationary obstacle inside the figure-8 loop;
   implement a phase-skip manoeuvre where the affected drone momentarily widens its orbit to
   $r + \Delta r$ to clear the obstacle, then snaps back to the formation.

---

## Related Scenarios

- Prerequisites: [S081 Selfie Follow](S081_selfie_follow.md) (single-drone KF orbit),
  [S085 Drone Racing Gate Sequence](S085_drone_racing_gate_sequence.md)
- Follow-ups: [S088 Aerial Acrobatics Synchronisation](S088_aerial_acrobatics_sync.md),
  [S092 Multi-Drone Stage Coverage](S092_multi_drone_stage_coverage.md)
- Algorithmic cross-reference: [S046 Multi-Drone 3D Trilateration](../03_environmental_sar/S046_trilateration.md)
  (shared KF update pattern), [S007 Blind Pursuit under Jamming](../01_pursuit_evasion/S007_blind_pursuit_jamming.md)
  (noisy subject estimation)
