# S081 Selfie Follow Mode

**Domain**: Special Ops & Entertainment | **Difficulty**: ⭐⭐ | **Status**: `[ ]` Not Started
**Algorithm**: Kalman Filter Tracking + Orbit PD Control | **Dimension**: 3D

---

## Problem Definition

**Setup**: A single drone operates in **selfie follow mode**: it autonomously tracks a moving person
and maintains a fixed relative position so that the on-board camera always frames the subject in the
centre third of the image. The person walks a **figure-8 path** of radius $R = 8\ \text{m}$ at a
constant ground speed of $v_{target} = 1.5\ \text{m/s}$ at ground level ($z = 0$). GPS measurements
of the person's position are corrupted by zero-mean Gaussian noise with standard deviation
$\sigma_{GPS} = 0.3\ \text{m}$, simulating consumer-grade on-body sensor accuracy.

The drone flies at a constant cruise altitude of $h = 4\ \text{m}$ above ground. It receives
noisy GPS fixes of the target at $f_{GPS} = 10\ \text{Hz}$ and runs a **Kalman Filter (KF)** to
produce a low-latency, smoothed estimate of target position and velocity. An **orbit PD controller**
uses the KF output to command drone velocity so that the drone maintains:

- **Standoff distance**: $r = 5\ \text{m}$ from the target in the horizontal plane.
- **Bearing angle**: $\varphi = 135°$ measured counter-clockwise from the target's heading
  (rear-left of the target).

A pinhole camera model with a horizontal field-of-view of $\theta_{FOV} = 60°$ is used to project
the target into the image plane. **Camera framing quality** is defined as the fraction of time the
projected target position falls within the central $\pm 10\%$ of the image half-width.

**Roles**:

- **Target (person)**: non-adversarial, follows a pre-programmed figure-8 trajectory; position
  measurements are corrupted by GPS noise before being fed to the drone's KF.
- **Drone**: single UAV executing KF estimation and orbit PD control; camera framing is evaluated
  in post-processing from the drone's logged position and heading.

**Objective**: Maximise the **framing score** $F$ (fraction of time target is centred in frame)
while minimising the **orbit distance error** $\bar{e}_{orbit}$ (RMS deviation from $r = 5\ \text{m}$
standoff). A baseline comparison replaces the KF with raw noisy GPS measurements fed directly to
the PD controller.

**Comparison strategies**:

1. **KF + Orbit PD** — Kalman Filter smoothing feeds the orbit controller.
2. **Raw GPS + Orbit PD** — noisy measurements fed directly without filtering.

---

## Mathematical Model

### Target Trajectory (Figure-8)

The figure-8 path is parameterised by arc-length parameter $\lambda(t) = v_{target}\, t / L$,
where $L$ is the total path length. In Cartesian coordinates:

$$x_{tgt}(t) = R \sin\!\bigl(2\pi\lambda(t)\bigr)$$

$$y_{tgt}(t) = \frac{R}{2} \sin\!\bigl(4\pi\lambda(t)\bigr)$$

$$z_{tgt} = 0$$

The instantaneous heading angle of the target:

$$\theta_{tgt}(t) = \operatorname{atan2}\!\left(\dot{y}_{tgt},\, \dot{x}_{tgt}\right)$$

### Kalman Filter — Target State Estimation

The KF state vector is $\mathbf{x} = [x,\; y,\; v_x,\; v_y]^\top \in \mathbb{R}^4$, representing
horizontal position and velocity. The discrete-time constant-velocity dynamics model:

$$\mathbf{x}_{k|k-1} = F\, \mathbf{x}_{k-1}, \qquad
F = \begin{bmatrix} 1 & 0 & \Delta t & 0 \\ 0 & 1 & 0 & \Delta t \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix}$$

**Predict step** (prior covariance):

$$P_{k|k-1} = F\, P_{k-1}\, F^\top + Q$$

where $Q = q_\sigma^2 \operatorname{diag}[\tfrac{\Delta t^3}{3}, \tfrac{\Delta t^3}{3}, \Delta t, \Delta t]$
is the process noise covariance parameterised by spectral density $q_\sigma = 0.5\ \text{m/s}^2$.

**Measurement model**: GPS observes position only, $\mathbf{z}_k = H\, \mathbf{x}_k + \boldsymbol{\eta}$:

$$H = \begin{bmatrix} 1 & 0 & 0 & 0 \\ 0 & 1 & 0 & 0 \end{bmatrix}, \qquad
\boldsymbol{\eta} \sim \mathcal{N}(\mathbf{0},\, R_{gps}), \quad
R_{gps} = \sigma_{GPS}^2\, I_2$$

**Update step** (Kalman gain and posterior):

$$K_k = P_{k|k-1}\, H^\top \bigl(H\, P_{k|k-1}\, H^\top + R_{gps}\bigr)^{-1}$$

$$\mathbf{x}_k = \mathbf{x}_{k|k-1} + K_k\bigl(\mathbf{z}_k - H\, \mathbf{x}_{k|k-1}\bigr)$$

$$P_k = (I - K_k H)\, P_{k|k-1}$$

The KF outputs estimated target position $\hat{\mathbf{p}}_{tgt} = [\hat{x}, \hat{y}]^\top$ and
velocity $\hat{\mathbf{v}}_{tgt} = [\hat{v}_x, \hat{v}_y]^\top$.

### Orbit Controller — Desired Drone Position

The estimated target heading from the KF velocity estimate:

$$\hat{\theta}_{tgt} = \operatorname{atan2}(\hat{v}_y,\, \hat{v}_x)$$

The desired drone position in the horizontal plane, offset at bearing $\varphi = 135°$ from the
target heading, at standoff radius $r = 5\ \text{m}$:

$$\mathbf{p}_d = \hat{\mathbf{p}}_{tgt} +
r \begin{bmatrix} \cos(\hat{\theta}_{tgt} + \varphi) \\ \sin(\hat{\theta}_{tgt} + \varphi) \end{bmatrix},
\qquad z_d = h$$

### Orbit PD Control Law

Let the orbit position error and its time derivative be:

$$\mathbf{e}(t) = \mathbf{p}_d(t) - \mathbf{p}_{drone}(t)$$

$$\dot{\mathbf{e}}(t) = \dot{\mathbf{p}}_d(t) - \dot{\mathbf{p}}_{drone}(t)$$

The commanded drone acceleration:

$$\mathbf{a}_{cmd}(t) = K_p\, \mathbf{e}(t) + K_d\, \dot{\mathbf{e}}(t)$$

The drone velocity is then integrated with aerodynamic drag:

$$\dot{\mathbf{v}}_{drone} = \mathbf{a}_{cmd} - c_{drag}\, \mathbf{v}_{drone}$$

$$\dot{\mathbf{p}}_{drone} = \mathbf{v}_{drone}$$

The drone's heading (yaw) is commanded to always point toward the target:

$$\psi_{drone} = \operatorname{atan2}\!\bigl(\hat{y}_{tgt} - y_{drone},\; \hat{x}_{tgt} - x_{drone}\bigr)$$

### Camera Framing Model

A pinhole camera is mounted on the drone facing the target. The horizontal pixel offset of the
projected target in the image plane (normalised to $[-1, 1]$):

$$u = \frac{\tan^{-1}\!\left(\dfrac{(\mathbf{p}_{tgt} - \mathbf{p}_{drone}) \cdot \hat{n}_{cam}}
{d_{horiz}}\right)}{\theta_{FOV}/2}$$

where $\hat{n}_{cam}$ is the camera's horizontal right-vector (perpendicular to the look direction
in the horizontal plane) and $d_{horiz}$ is the horizontal distance from drone to target.

**Framing score** over mission duration $T$:

$$F = \frac{1}{T} \int_0^T \mathbf{1}\!\bigl[|u(t)| \leq 0.10\bigr]\, dt$$

### Orbit Distance Error

$$e_{orbit}(t) = \left\|\mathbf{p}_{drone}^{xy}(t) - \mathbf{p}_{tgt}^{xy}(t)\right\| - r$$

$$\bar{e}_{orbit} = \sqrt{\frac{1}{T}\int_0^T e_{orbit}^2(t)\, dt}$$

---

## Implementation

```python
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation

# ── Domain constants (Domain 5: Special Ops & Entertainment) ──────────────────
LED_SHOW_HEIGHT   = 50.0   # m
FORMATION_DIST    = 2.0    # m
COLLISION_RADIUS  = 0.5    # m

# ── Scenario parameters ───────────────────────────────────────────────────────
R_FIGURE8       = 8.0      # m — figure-8 lobe radius
V_TARGET        = 1.5      # m/s — target walking speed
GPS_SIGMA       = 0.3      # m — GPS noise std dev (on-body sensor)
GPS_FREQ        = 10.0     # Hz — GPS measurement rate

STANDOFF_R      = 5.0      # m — desired horizontal standoff
BEARING_PHI     = np.radians(135.0)   # rad — rear-left bearing offset
CRUISE_ALT      = 4.0      # m — drone flight altitude

CAMERA_FOV      = np.radians(60.0)    # rad — horizontal field of view
FRAME_TOL       = 0.10     # fraction of half-FOV width for framing success

# ── KF process noise ──────────────────────────────────────────────────────────
Q_SIGMA         = 0.5      # m/s^2 — spectral density for constant-velocity model

# ── Orbit PD gains ────────────────────────────────────────────────────────────
KP_ORBIT        = 1.5      # proportional gain
KD_ORBIT        = 0.8      # derivative gain
C_DRAG          = 0.3      # s^{-1} — aerodynamic drag coefficient

# ── Simulation ────────────────────────────────────────────────────────────────
DT              = 0.05     # s — simulation timestep
T_TOTAL         = 60.0     # s — total mission time
N_STEPS         = int(T_TOTAL / DT)


def figure8_trajectory(t):
    """Return (position [x,y,z], velocity [vx,vy,vz]) of target at time t."""
    # Use numerical derivative for heading; parameterise by time directly.
    # Period chosen so that tangential speed ≈ V_TARGET on average.
    omega = V_TARGET / (np.pi * R_FIGURE8)   # angular frequency (rad/s)
    x  =  R_FIGURE8 * np.sin(2 * np.pi * omega * t)
    y  =  R_FIGURE8 / 2 * np.sin(4 * np.pi * omega * t)
    vx =  R_FIGURE8 * 2 * np.pi * omega * np.cos(2 * np.pi * omega * t)
    vy =  R_FIGURE8 / 2 * 4 * np.pi * omega * np.cos(4 * np.pi * omega * t)
    # Normalise to desired speed
    speed = np.hypot(vx, vy)
    if speed > 1e-6:
        vx, vy = vx / speed * V_TARGET, vy / speed * V_TARGET
    return np.array([x, y, 0.0]), np.array([vx, vy, 0.0])


def build_kf_matrices(dt):
    """Constant-velocity KF matrices."""
    F = np.array([[1, 0, dt, 0],
                  [0, 1,  0, dt],
                  [0, 0,  1,  0],
                  [0, 0,  0,  1]], dtype=float)
    H = np.array([[1, 0, 0, 0],
                  [0, 1, 0, 0]], dtype=float)
    Q = Q_SIGMA**2 * np.diag([dt**3/3, dt**3/3, dt, dt])
    R_gps = GPS_SIGMA**2 * np.eye(2)
    return F, H, Q, R_gps


def kf_predict(x, P, F, Q):
    x_pred = F @ x
    P_pred = F @ P @ F.T + Q
    return x_pred, P_pred


def kf_update(x_pred, P_pred, z, H, R_gps):
    S  = H @ P_pred @ H.T + R_gps
    K  = P_pred @ H.T @ np.linalg.inv(S)
    x  = x_pred + K @ (z - H @ x_pred)
    P  = (np.eye(len(x)) - K @ H) @ P_pred
    return x, P


def desired_drone_position(pos_est, vel_est):
    """Compute desired (x, y, z) for the drone given KF estimates."""
    theta = np.arctan2(vel_est[1], vel_est[0])
    angle = theta + BEARING_PHI
    p_d   = np.array([
        pos_est[0] + STANDOFF_R * np.cos(angle),
        pos_est[1] + STANDOFF_R * np.sin(angle),
        CRUISE_ALT,
    ])
    return p_d


def framing_score_instant(p_drone, p_tgt):
    """Normalised horizontal pixel offset u in [-1, 1]."""
    diff      = p_tgt - p_drone
    look_dir  = diff[:2] / (np.linalg.norm(diff[:2]) + 1e-8)
    # Right vector (90° CCW of look direction in horizontal plane)
    right_vec = np.array([-look_dir[1], look_dir[0]])
    horiz_dist = np.linalg.norm(diff[:2])
    lateral    = np.dot(diff[:2], right_vec)
    if horiz_dist < 1e-6:
        return 0.0
    angle_off  = np.arctan2(lateral, horiz_dist)
    u          = angle_off / (CAMERA_FOV / 2)
    return float(u)


def simulate(use_kf=True, seed=42):
    """
    Run selfie-follow simulation.

    Parameters
    ----------
    use_kf : bool
        True  → Kalman Filter on GPS measurements (nominal).
        False → raw GPS measurements fed directly to PD controller.
    seed : int
        RNG seed for GPS noise.

    Returns
    -------
    dict with logged arrays: t, pos_tgt, pos_drone, u_frame, e_orbit,
                              kf_pos_est (if use_kf), kf_vel_est (if use_kf).
    """
    rng = np.random.default_rng(seed)
    F_mat, H_mat, Q_mat, R_gps = build_kf_matrices(DT)

    # Initial target state
    p_tgt0, v_tgt0 = figure8_trajectory(0.0)

    # Initialise KF state and covariance
    x_kf = np.array([p_tgt0[0], p_tgt0[1], v_tgt0[0], v_tgt0[1]])
    P_kf = np.eye(4) * 1.0

    # Initialise drone at desired position
    p_d0 = desired_drone_position(p_tgt0[:2], v_tgt0[:2])
    p_drone = p_d0.copy()
    v_drone = np.zeros(3)

    # GPS step counter (GPS at 10 Hz, sim at 20 Hz)
    gps_every = max(1, int(1.0 / (GPS_FREQ * DT)))

    # Logging arrays
    log_t          = np.zeros(N_STEPS)
    log_pos_tgt    = np.zeros((N_STEPS, 3))
    log_pos_drone  = np.zeros((N_STEPS, 3))
    log_u_frame    = np.zeros(N_STEPS)
    log_e_orbit    = np.zeros(N_STEPS)
    log_kf_pos     = np.zeros((N_STEPS, 2))
    log_kf_vel     = np.zeros((N_STEPS, 2))

    p_d_prev = p_d0.copy()

    for k in range(N_STEPS):
        t = k * DT

        # True target state
        p_tgt, v_tgt = figure8_trajectory(t)

        # GPS measurement (only at GPS frequency)
        if k % gps_every == 0:
            noise = rng.normal(0.0, GPS_SIGMA, 2)
            z_gps = p_tgt[:2] + noise

            if use_kf:
                # Kalman Filter predict + update
                x_kf, P_kf = kf_predict(x_kf, P_kf, F_mat, Q_mat)
                x_kf, P_kf = kf_update(x_kf, P_kf, z_gps, H_mat, R_gps)
                pos_est = x_kf[:2]
                vel_est = x_kf[2:]
            else:
                # Raw GPS: position from measurement, velocity from finite diff
                if k > 0:
                    vel_est = (z_gps - pos_est_prev) / (gps_every * DT)
                else:
                    vel_est = v_tgt[:2].copy()
                pos_est = z_gps.copy()
            pos_est_prev = pos_est.copy()   # noqa: F821 (assigned before use on k>0)
        # Between GPS updates: propagate KF prediction only
        elif use_kf:
            x_kf, P_kf = kf_predict(x_kf, P_kf, F_mat, Q_mat)
            pos_est = x_kf[:2]
            vel_est = x_kf[2:]

        # Desired drone position
        p_d = desired_drone_position(pos_est, vel_est)

        # PD control
        e      = p_d - p_drone
        e_dot  = (p_d - p_d_prev) / DT - v_drone
        a_cmd  = KP_ORBIT * e + KD_ORBIT * e_dot

        # Drone dynamics (double integrator with drag)
        v_drone = v_drone + (a_cmd - C_DRAG * v_drone) * DT
        p_drone = p_drone + v_drone * DT
        p_d_prev = p_d.copy()

        # Metrics
        u_frame  = framing_score_instant(p_drone, p_tgt)
        e_orbit  = np.linalg.norm(p_drone[:2] - p_tgt[:2]) - STANDOFF_R

        # Log
        log_t[k]         = t
        log_pos_tgt[k]   = p_tgt
        log_pos_drone[k] = p_drone
        log_u_frame[k]   = u_frame
        log_e_orbit[k]   = e_orbit
        log_kf_pos[k]    = pos_est
        log_kf_vel[k]    = vel_est

    return {
        "t":         log_t,
        "pos_tgt":   log_pos_tgt,
        "pos_drone": log_pos_drone,
        "u_frame":   log_u_frame,
        "e_orbit":   log_e_orbit,
        "kf_pos":    log_kf_pos,
        "kf_vel":    log_kf_vel,
    }


def compute_metrics(log):
    """Return framing score F, RMS orbit error, and max orbit error."""
    framing = np.mean(np.abs(log["u_frame"]) <= FRAME_TOL)
    rms_err = np.sqrt(np.mean(log["e_orbit"]**2))
    max_err = np.max(np.abs(log["e_orbit"]))
    return framing, rms_err, max_err


def plot_3d_trajectory(log_kf, log_raw):
    """3D view of drone and target trajectories for both strategies."""
    fig = plt.figure(figsize=(10, 7))
    ax  = fig.add_subplot(111, projection="3d")

    ax.plot(log_kf["pos_tgt"][:, 0],
            log_kf["pos_tgt"][:, 1],
            log_kf["pos_tgt"][:, 2],
            "b-", lw=2, label="Target (figure-8)")

    ax.plot(log_kf["pos_drone"][:, 0],
            log_kf["pos_drone"][:, 1],
            log_kf["pos_drone"][:, 2],
            "r-", lw=1.5, label="Drone (KF)")

    ax.plot(log_raw["pos_drone"][:, 0],
            log_raw["pos_drone"][:, 1],
            log_raw["pos_drone"][:, 2],
            color="orange", lw=1.2, ls="--", label="Drone (raw GPS)")

    # Start markers
    ax.scatter(*log_kf["pos_tgt"][0],  color="blue",   s=60, marker="o", zorder=5)
    ax.scatter(*log_kf["pos_drone"][0], color="red",   s=60, marker="^", zorder=5)

    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_zlabel("z (m)")
    ax.set_title("S081 Selfie Follow — 3D Trajectory")
    ax.legend(loc="upper left", fontsize=8)
    plt.tight_layout()
    return fig


def plot_metrics(log_kf, log_raw):
    """Three-panel metrics figure: orbit error, framing u, framing histogram."""
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=False)

    t = log_kf["t"]

    # Panel 1: orbit distance error over time
    ax = axes[0]
    ax.plot(t, log_kf["e_orbit"],  "r-",  lw=1.2, label="KF")
    ax.plot(t, log_raw["e_orbit"], "orange", lw=1.0, ls="--", label="Raw GPS")
    ax.axhline(0, color="k", lw=0.7)
    ax.fill_between(t, -0.5, 0.5, alpha=0.10, color="green",
                    label="±0.5 m band")
    ax.set_ylabel("Orbit error $e_{orbit}$ (m)")
    ax.set_xlabel("Time (s)")
    ax.set_title("Orbit Distance Error vs Time")
    ax.legend(fontsize=8)

    # Panel 2: normalised pixel offset u over time
    ax = axes[1]
    ax.plot(t, log_kf["u_frame"],  "r-",  lw=1.0, label="KF")
    ax.plot(t, log_raw["u_frame"], "orange", lw=0.8, ls="--", label="Raw GPS")
    ax.axhline( FRAME_TOL, color="g", lw=1.0, ls=":", label=f"±{FRAME_TOL:.0%} tol")
    ax.axhline(-FRAME_TOL, color="g", lw=1.0, ls=":")
    ax.set_ylim(-1.0, 1.0)
    ax.set_ylabel("Normalised frame offset $u$")
    ax.set_xlabel("Time (s)")
    ax.set_title("Camera Framing Offset vs Time")
    ax.legend(fontsize=8)

    # Panel 3: histogram of |u| — fraction within tolerance
    ax = axes[2]
    bins = np.linspace(0, 1, 40)
    ax.hist(np.abs(log_kf["u_frame"]),  bins=bins, alpha=0.6,
            color="red",    label="KF",      density=True)
    ax.hist(np.abs(log_raw["u_frame"]), bins=bins, alpha=0.6,
            color="orange", label="Raw GPS", density=True)
    ax.axvline(FRAME_TOL, color="g", lw=1.5, ls="--",
               label=f"Tolerance {FRAME_TOL:.0%}")
    ax.set_xlabel("$|u|$ (normalised)")
    ax.set_ylabel("Density")
    ax.set_title("Distribution of Camera Frame Offset")
    ax.legend(fontsize=8)

    plt.tight_layout()
    return fig


def animate_follow(log_kf):
    """
    2D top-down animation of drone following target.
    Saved as outputs/05_special_entertainment/s081_selfie_follow/s081_selfie_follow.gif
    """
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.set_xlim(-15, 15)
    ax.set_ylim(-15, 15)
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("S081 Selfie Follow — Top-Down View")

    # Static: full target path
    ax.plot(log_kf["pos_tgt"][:, 0],
            log_kf["pos_tgt"][:, 1],
            "b--", lw=0.8, alpha=0.4, label="Target path")

    tgt_dot,  = ax.plot([], [], "bo", ms=10, label="Target")
    drone_dot, = ax.plot([], [], "r^", ms=10, label="Drone")
    link_line, = ax.plot([], [], "k-", lw=0.8, alpha=0.5)
    standoff_circle = plt.Circle((0, 0), STANDOFF_R, color="gray",
                                  fill=False, lw=1.0, ls=":")
    ax.add_patch(standoff_circle)
    framing_text = ax.text(-13, 13, "", fontsize=9, color="darkred")
    ax.legend(loc="upper right", fontsize=8)

    step = 4   # animate every 4th simstep (≈12 fps at DT=0.05)

    def init():
        tgt_dot.set_data([], [])
        drone_dot.set_data([], [])
        link_line.set_data([], [])
        return tgt_dot, drone_dot, link_line, framing_text

    def update(frame):
        k = frame * step
        px, py = log_kf["pos_tgt"][k, 0],  log_kf["pos_tgt"][k, 1]
        dx, dy = log_kf["pos_drone"][k, 0], log_kf["pos_drone"][k, 1]
        tgt_dot.set_data([px], [py])
        drone_dot.set_data([dx], [dy])
        link_line.set_data([px, dx], [py, dy])
        standoff_circle.center = (px, py)
        u = log_kf["u_frame"][k]
        framed = "IN FRAME" if abs(u) <= FRAME_TOL else "off-frame"
        framing_text.set_text(f"t={log_kf['t'][k]:.1f}s  u={u:+.2f}  [{framed}]")
        return tgt_dot, drone_dot, link_line, standoff_circle, framing_text

    n_frames = N_STEPS // step
    ani = animation.FuncAnimation(
        fig, update, frames=n_frames, init_func=init,
        interval=50, blit=True
    )
    return fig, ani


def run_simulation():
    log_kf  = simulate(use_kf=True,  seed=42)
    log_raw = simulate(use_kf=False, seed=42)

    F_kf,  rms_kf,  max_kf  = compute_metrics(log_kf)
    F_raw, rms_raw, max_raw = compute_metrics(log_raw)

    print("=== S081 Selfie Follow Mode ===")
    print(f"{'Strategy':<18} {'Framing %':>10} {'RMS e_orbit':>12} {'Max e_orbit':>12}")
    print(f"{'KF + Orbit PD':<18} {F_kf*100:>9.1f}% {rms_kf:>12.3f} m {max_kf:>11.3f} m")
    print(f"{'Raw GPS + PD':<18} {F_raw*100:>9.1f}% {rms_raw:>12.3f} m {max_raw:>11.3f} m")

    return log_kf, log_raw
```

---

## Key Parameters

| Parameter | Symbol | Value |
|-----------|--------|-------|
| Figure-8 lobe radius | $R$ | 8.0 m |
| Target walking speed | $v_{target}$ | 1.5 m/s |
| GPS noise std dev | $\sigma_{GPS}$ | 0.3 m |
| GPS update rate | $f_{GPS}$ | 10 Hz |
| Standoff distance | $r$ | 5.0 m |
| Bearing angle (rear-left) | $\varphi$ | 135° |
| Cruise altitude | $h$ | 4.0 m |
| Camera horizontal FOV | $\theta_{FOV}$ | 60° |
| Framing tolerance | — | ±10% of half-FOV |
| KF process noise spectral density | $q_\sigma$ | 0.5 m/s² |
| KF measurement noise | $R_{gps}$ | $\sigma_{GPS}^2 I_2$ |
| Orbit PD proportional gain | $K_p$ | 1.5 |
| Orbit PD derivative gain | $K_d$ | 0.8 |
| Aerodynamic drag coefficient | $c_{drag}$ | 0.3 s⁻¹ |
| Simulation timestep | $\Delta t$ | 0.05 s |
| Total mission time | $T$ | 60.0 s |
| Domain collision radius | $r_{coll}$ | 0.5 m |
| Domain formation distance | $d_{form}$ | 2.0 m |

---

## Expected Output

- **3D trajectory plot**: `mpl_toolkits.mplot3d` scene with the figure-8 target path in blue,
  the KF-controlled drone path in red, and the raw-GPS drone path in orange dashed; start positions
  marked as coloured symbols; altitude $h = 4\ \text{m}$ shown on the $z$-axis; axes labelled in
  metres; legend identifying target, KF drone, and raw-GPS drone.

- **Metrics panel (3 subplots)**:
  - *Top*: orbit distance error $e_{orbit}(t)$ vs time for both strategies; $\pm 0.5\ \text{m}$
    tolerance band shaded green; zero-error reference dashed black.
  - *Middle*: normalised camera frame offset $u(t)$ vs time; $\pm 10\%$ framing tolerance marked
    with green dotted lines; values outside $[-1, 1]$ indicate the target is off-screen.
  - *Bottom*: histogram of $|u|$ for both strategies (density-normalised); tolerance threshold at
    $|u| = 0.10$ shown as vertical green dashed line; visual evidence of KF advantage.

- **Animation (GIF)**: top-down 2D view at 12 fps showing the target (blue circle) and drone (red
  triangle) moving together; the standoff circle of radius $r = 5\ \text{m}$ centred on the target
  in grey; a connecting line between target and drone; framing status text (IN FRAME / off-frame)
  and current $u$ value displayed in the corner; saved to
  `outputs/05_special_entertainment/s081_selfie_follow/s081_selfie_follow.gif`.

Terminal output at mission end:

```
=== S081 Selfie Follow Mode ===
Strategy           Framing %  RMS e_orbit  Max e_orbit
KF + Orbit PD         82.0%       0.31 m       1.12 m
Raw GPS + PD          54.0%       0.89 m       2.74 m
```

---

## Extensions

1. **Adaptive standoff**: add a rule that widens $r$ when the target accelerates above
   $a_{tgt} > 1\ \text{m/s}^2$ (avoiding camera motion blur); tune the schedule
   $r(a_{tgt}) = r_0 + k_r \cdot a_{tgt}$ and measure the effect on framing score.
2. **Vertical framing**: extend the camera model to 2-axis (pan + tilt) and add a second KF state
   for target height; allow the drone to adjust cruise altitude dynamically to keep the target
   vertically centred in the frame.
3. **Obstacle avoidance**: place static cylindrical obstacles along the figure-8 path; integrate a
   repulsive potential field (as in S061) into the orbit PD controller and measure the increase in
   standoff error when detours are required.
4. **Multi-drone handoff**: deploy two drones at opposite bearings; when the target changes
   direction abruptly, transfer the "active camera" role to whichever drone has the lower framing
   offset $|u|$; measure transition latency and continuity of framing.
5. **Real GPS latency**: introduce a GPS measurement latency of $\tau_{lat} = 0.2\ \text{s}$ and
   compare with a predictor-corrector KF extension (state prediction to present time using
   $\hat{\mathbf{x}}_{now} = F^{n}\hat{\mathbf{x}}_{k}$ where $n = \tau_{lat} / \Delta t$).

---

## Related Scenarios

- Prerequisites: [S003 Low-Altitude Tracking](../01_pursuit_evasion/S003_low_altitude_tracking.md), [S013 Particle Filter Intercept](../01_pursuit_evasion/S013_particle_filter_intercept.md)
- Follow-ups: [S086 Formation Light Show](S086_formation_light_show.md), [S092 Crowd Surveillance](S092_crowd_surveillance.md), [S094 Actor Chase Cam](S094_actor_chase_cam.md)
- Algorithmic cross-reference: [S042 Missing Person Localization](../03_environmental_sar/S042_missing_person.md) (Bayesian target tracking), [S061 Power Line Inspection](../04_industrial_agriculture/S061_power_line.md) (PD offset controller pattern)
