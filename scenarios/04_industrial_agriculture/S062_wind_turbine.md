# S062 Wind Turbine Blade Inspection

**Domain**: Industrial & Agriculture | **Difficulty**: ⭐⭐⭐ | **Status**: `[ ]` Not Started
**Algorithm**: Phase-Locked Orbital Inspection + Safety Margin Controller | **Dimension**: 3D

---

## Problem Definition

**Setup**: A horizontal-axis wind turbine stands at hub height $h_{hub} = 30$ m with three blades of
length $R_{blade} = 15$ m and rotor angular velocity $\omega = 0.5$ rad/s (period $T_{rot} \approx
12.6$ s). The blades are equally spaced at $\phi_i = 2\pi i/3$ ($i = 0, 1, 2$) and rotate in the
rotor plane (the $xz$-plane with the hub as origin). A single inspection drone orbits the rotor hub
in the same plane at radius $r_{orbit} = 17$ m (hub-centred), maintaining a fixed angular phase
offset $\phi_{offset} = 60°$ ahead of each blade tip. GPS positioning carries an isotropic error
$\sigma_{GPS} = 0.1$ m. The minimum safe separation from any blade surface is
$d_{safe} = 1.5$ m (hard constraint), and the target inspection standoff is
$d_{inspect} = 1.0$ m from the blade surface. The drone speed during an active inspection pass
must satisfy $v_{drone} \leq v_{max,inspect} = 1.0$ m/s.

**Roles**:
- **Drone**: single inspection agent orbiting the rotor plane; carries a downward- and side-facing
  optical/thermal camera; must complete at least one full inspection pass over every blade section
  at the prescribed standoff distance.
- **Rotor**: three rotating blades modelled as line segments from hub to tip; angular state is
  known to the drone in real time via a turbine SCADA feed (simulated encoder signal).

**Objective**: Complete a full-blade inspection of all three blades — visiting every 1 m segment
along each blade at standoff distance $d_{inspect}$ — while never violating the safety margin
$d_{safe}$ from any blade surface. Total inspection time and safety margin violations are the
primary metrics.

**Controller variants** (three compared):
1. **Open-loop orbit** — drone advances at constant angular rate $\dot{\theta} = \omega$; no
   blade-phase feedback; baseline, expected to accumulate safety violations.
2. **Phase-locked orbit** — drone maintains $\theta_{target}(t) = \omega t + \phi_{offset}$
   relative to the nearest blade; advances through inspection waypoints between blade passes.
3. **Phase-locked + safety margin controller** (proposed) — phase-locked orbit augmented with an
   emergency lateral thrust when $\|\mathbf{p}_{drone} - \mathbf{p}_{tip}(t)\| < d_{safe} + 0.3$ m
   (warning zone); inspection speed reduced to $v_{max,inspect}$ during active blade coverage.

---

## Mathematical Model

### Coordinate Frame and Blade Kinematics

The global frame has its origin at ground level directly below the turbine hub. The hub position
is $\mathbf{p}_{hub} = (0,\, 0,\, h_{hub})^\top$. The rotor plane is the $xz$-plane (turbine
shaft along $y$). The drone orbits at a lateral offset $y_{offset}$ from the rotor plane to avoid
the nacelle and allow camera coverage; a small offset $y_{offset} = 1.0$ m is used.

The tip position of blade $i$ at time $t$ is:

$$\mathbf{p}_{tip,i}(t) = \mathbf{p}_{hub} + R_{blade}
  \begin{bmatrix} \cos(\omega t + \phi_i) \\ 0 \\ \sin(\omega t + \phi_i) \end{bmatrix},
  \quad \phi_i = \frac{2\pi i}{3}, \quad i = 0, 1, 2$$

The full blade $i$ is the line segment parameterised by $s \in [0, 1]$:

$$\mathbf{p}_{blade,i}(s, t) = \mathbf{p}_{hub} + s \cdot R_{blade}
  \begin{bmatrix} \cos(\omega t + \phi_i) \\ 0 \\ \sin(\omega t + \phi_i) \end{bmatrix}$$

### Drone Orbital Parameterisation

The drone moves on a circle of radius $r_{orbit}$ centred on the hub:

$$\mathbf{p}_{drone}(\theta) = \mathbf{p}_{hub} +
  \begin{bmatrix} r_{orbit} \cos\theta \\ y_{offset} \\ r_{orbit} \sin\theta \end{bmatrix}$$

The drone's angular position $\theta(t)$ is the controlled degree of freedom. The nominal
inspection path is to track blade $i$'s angular position with lead angle $\phi_{offset}$:

$$\theta_{target,i}(t) = \omega t + \phi_i + \phi_{offset}$$

The nearest blade index is selected as:

$$i^*(t) = \arg\min_{i \in \{0,1,2\}} \left|\, \theta(t) - (\omega t + \phi_i)\,\right|_{\,2\pi}$$

where $|\cdot|_{2\pi}$ denotes the minimum angular distance modulo $2\pi$.

### Phase-Locked Control Law

The phase error with respect to the target is:

$$e_\theta(t) = \theta_{target,i^*}(t) - \theta(t)$$

A proportional-derivative angular rate command:

$$\dot{\theta}_{cmd}(t) = \omega + K_p \, e_\theta(t) + K_d \, \dot{e}_\theta(t)$$

with gains $K_p = 2.0$ rad/(rad·s) and $K_d = 0.3$ s. This keeps the drone locked $\phi_{offset}$
ahead of the leading blade tip regardless of small perturbations or GPS noise.

The Cartesian velocity commanded to the drone is:

$$\dot{\mathbf{p}}_{cmd} = \dot{\theta}_{cmd}
  \begin{bmatrix} -r_{orbit} \sin\theta \\ 0 \\ r_{orbit} \cos\theta \end{bmatrix}$$

Speed is clamped to $v_{max,inspect}$ when the drone is within $d_{inspect} + \varepsilon$ of any
blade surface ($\varepsilon = 0.2$ m tolerance).

### Safety Margin Controller

The signed safety clearance to blade tip $i$ is:

$$\delta_i(t) = \|\mathbf{p}_{drone}(t) - \mathbf{p}_{tip,i}(t)\| - d_{safe}$$

More precisely, the minimum distance to blade $i$ (modelled as a line segment) is:

$$d_{seg,i}(t) = \min_{s \in [0,1]}
  \|\mathbf{p}_{drone}(t) - \mathbf{p}_{blade,i}(s,t)\|$$

The emergency condition is triggered when $d_{seg,i^*}(t) < d_{safe}$:

$$\mathbf{f}_{safety} = \begin{cases}
  K_{safe} \cdot \dfrac{\mathbf{p}_{drone} - \mathbf{p}_{tip,i^*}}
  {\|\mathbf{p}_{drone} - \mathbf{p}_{tip,i^*}\|}
  & \text{if } d_{seg,i^*} < d_{safe} \\[6pt]
  \mathbf{0} & \text{otherwise}
\end{cases}$$

with $K_{safe} = 3.0$ m/s² (lateral thrust gain). This repulsion is added to the nominal
Cartesian velocity command and takes precedence over the phase-lock term.

### Inspection Quality Metric

Discretise each blade into $N_{seg} = 15$ segments of 1 m each. Segment $k$ of blade $i$ is
considered **inspected** at time $t$ when all three conditions hold simultaneously:

$$\text{inspected}_{i,k}(t) = \mathbf{1}\!\left[
  d_{blade,i,k}(t) \leq d_{inspect} + \varepsilon
  \;\wedge\;
  v_{drone}(t) \leq v_{max,inspect}
  \;\wedge\;
  \cos\angle(\hat{\mathbf{n}}_{camera},\, \hat{\mathbf{r}}_{i,k}) \geq \cos 30°
\right]$$

where $d_{blade,i,k}(t)$ is the drone distance to the midpoint of segment $k$,
$\hat{\mathbf{n}}_{camera}$ is the camera boresight direction, and
$\hat{\mathbf{r}}_{i,k}$ is the unit vector from the drone to the segment midpoint.

Overall inspection coverage:

$$\text{Coverage} = \frac{1}{3 N_{seg}} \sum_{i=0}^{2} \sum_{k=1}^{N_{seg}}
  \mathbf{1}\!\left[\exists\, t: \text{inspected}_{i,k}(t) = 1\right] \times 100\%$$

### GPS Noise Model

The drone's measured position is corrupted by isotropic Gaussian noise:

$$\tilde{\mathbf{p}}_{drone}(t) = \mathbf{p}_{drone}(t) + \boldsymbol{\eta}(t),
  \qquad \boldsymbol{\eta}(t) \sim \mathcal{N}(\mathbf{0},\, \sigma_{GPS}^2 \mathbf{I}_3)$$

The phase-lock controller operates on $\tilde{\mathbf{p}}_{drone}$ while the true position is used
to evaluate inspection coverage and safety violations.

---

## Implementation

```python
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation

# ── Domain constants ──────────────────────────────────────────────────────────
INSPECTION_SPEED = 1.0          # m/s  — v_max_inspect
INSPECTION_DIST  = 0.5          # m    — target standoff from blade surface
GPS_ERROR        = 0.1          # m    — isotropic GPS noise std dev

# ── Turbine geometry ──────────────────────────────────────────────────────────
HUB_HEIGHT    = 30.0            # m
BLADE_LENGTH  = 15.0            # m
OMEGA         = 0.5             # rad/s  (period ≈ 12.57 s)
N_BLADES      = 3
BLADE_PHASES  = np.array([2 * np.pi * i / N_BLADES for i in range(N_BLADES)])

# ── Drone orbit ───────────────────────────────────────────────────────────────
R_ORBIT       = 17.0            # m   hub-centred orbit radius
Y_OFFSET      = 1.0             # m   lateral offset from rotor plane
PHI_OFFSET    = np.deg2rad(60)  # rad phase lead ahead of blade tip
D_SAFE        = 1.5             # m   hard safety margin from blade surface
D_INSPECT     = 1.0             # m   target inspection standoff
EPS_INSPECT   = 0.2             # m   inspection distance tolerance
N_SEG         = 15              # blade inspection segments

# ── Controller gains ──────────────────────────────────────────────────────────
KP            = 2.0             # rad/(rad·s)
KD            = 0.3             # s
K_SAFE        = 3.0             # m/s²  safety repulsion gain

# ── Simulation ────────────────────────────────────────────────────────────────
DT            = 0.05            # s
T_MAX         = 120.0           # s  (≈ 9.5 rotor revolutions)

P_HUB = np.array([0.0, 0.0, HUB_HEIGHT])


def blade_tip(t, i):
    """3-D position of blade i tip at time t."""
    ang = OMEGA * t + BLADE_PHASES[i]
    return P_HUB + BLADE_LENGTH * np.array([np.cos(ang), 0.0, np.sin(ang)])


def blade_point(t, i, s):
    """Point at fractional arc-length s in [0,1] along blade i."""
    ang = OMEGA * t + BLADE_PHASES[i]
    return P_HUB + s * BLADE_LENGTH * np.array([np.cos(ang), 0.0, np.sin(ang)])


def dist_to_blade_segment(p, t, i):
    """Minimum distance from point p to the full line segment of blade i."""
    tip = blade_tip(t, i)
    # Blade vector from hub to tip
    blade_vec = tip - P_HUB
    blade_len_sq = np.dot(blade_vec, blade_vec)
    dp = p - P_HUB
    s_clamp = np.clip(np.dot(dp, blade_vec) / blade_len_sq, 0.0, 1.0)
    closest = P_HUB + s_clamp * blade_vec
    return np.linalg.norm(p - closest)


def nearest_blade(theta, t):
    """Index of blade whose angular position is closest to theta."""
    blade_angles = OMEGA * t + BLADE_PHASES
    diffs = np.abs(((theta - blade_angles + np.pi) % (2 * np.pi)) - np.pi)
    return int(np.argmin(diffs))


def drone_position(theta):
    """Cartesian position of drone given orbital angle theta."""
    return P_HUB + np.array([R_ORBIT * np.cos(theta),
                              Y_OFFSET,
                              R_ORBIT * np.sin(theta)])


def phase_lock_controller(theta, theta_dot, t, i_near):
    """PD phase-lock angular rate command."""
    theta_target = OMEGA * t + BLADE_PHASES[i_near] + PHI_OFFSET
    e_theta = ((theta_target - theta + np.pi) % (2 * np.pi)) - np.pi
    e_dot   = OMEGA - theta_dot      # target rate minus current rate
    return OMEGA + KP * e_theta + KD * e_dot


def safety_repulsion(p_drone, t):
    """Emergency lateral repulsion force when inside warning zone."""
    f_rep = np.zeros(3)
    for i in range(N_BLADES):
        d = dist_to_blade_segment(p_drone, t, i)
        if d < D_SAFE:
            tip = blade_tip(t, i)
            direction = p_drone - tip
            norm = np.linalg.norm(direction) + 1e-9
            f_rep += K_SAFE * direction / norm
    return f_rep


def run_simulation(strategy="phase_locked_safety", seed=42):
    """
    strategy: "open_loop" | "phase_locked" | "phase_locked_safety"
    Returns time array, drone positions, safety violations, inspection log.
    """
    rng = np.random.default_rng(seed)

    theta     = PHI_OFFSET          # initial orbital angle
    theta_dot = OMEGA               # initial angular rate

    steps   = int(T_MAX / DT)
    times   = np.zeros(steps)
    pos_log = np.zeros((steps, 3))
    dist_log = np.zeros((steps, N_BLADES))
    safety_violations = 0
    inspected = np.zeros((N_BLADES, N_SEG), dtype=bool)

    for k in range(steps):
        t = k * DT
        times[k] = t

        # True and measured drone position
        p_true = drone_position(theta)
        noise  = rng.normal(0, GPS_ERROR, 3)
        p_meas = p_true + noise
        pos_log[k] = p_true

        # Nearest blade index (uses measured position)
        meas_theta = np.arctan2(p_meas[2] - HUB_HEIGHT, p_meas[0])
        i_near = nearest_blade(meas_theta, t)

        # Record distances to all blade segments
        for i in range(N_BLADES):
            dist_log[k, i] = dist_to_blade_segment(p_true, t, i)

        # Safety violation check (uses true position)
        if dist_log[k].min() < D_SAFE:
            safety_violations += 1

        # ── Angular rate command ──────────────────────────────────────────────
        if strategy == "open_loop":
            theta_dot_cmd = OMEGA

        elif strategy == "phase_locked":
            theta_dot_cmd = phase_lock_controller(theta, theta_dot, t, i_near)

        else:   # phase_locked_safety
            theta_dot_cmd = phase_lock_controller(theta, theta_dot, t, i_near)
            # Safety repulsion in Cartesian, project back to angular
            f_safe = safety_repulsion(p_true, t)
            # Tangent vector at current theta
            tangent = np.array([-R_ORBIT * np.sin(theta), 0.0, R_ORBIT * np.cos(theta)])
            theta_dot_cmd += np.dot(f_safe, tangent) / (R_ORBIT ** 2 + 1e-9)

        # Speed cap during close-approach inspection
        min_blade_dist = dist_log[k].min()
        v_linear = np.abs(theta_dot_cmd) * R_ORBIT
        if min_blade_dist <= D_INSPECT + EPS_INSPECT and v_linear > INSPECTION_SPEED:
            theta_dot_cmd *= INSPECTION_SPEED / (v_linear + 1e-9)

        # Euler integration
        theta_dot = theta_dot_cmd
        theta    += theta_dot * DT

        # ── Inspection coverage update ────────────────────────────────────────
        for i in range(N_BLADES):
            for seg in range(N_SEG):
                s_mid = (seg + 0.5) / N_SEG
                p_seg = blade_point(t, i, s_mid)
                d_seg = np.linalg.norm(p_true - p_seg)
                v_now = np.abs(theta_dot) * R_ORBIT
                if d_seg <= D_INSPECT + EPS_INSPECT and v_now <= INSPECTION_SPEED * 1.05:
                    inspected[i, seg] = True

    coverage = inspected.sum() / (N_BLADES * N_SEG) * 100.0
    return times, pos_log, dist_log, safety_violations, coverage, inspected


def plot_results():
    from mpl_toolkits.mplot3d import Axes3D   # noqa: F401 — required for 3D projection

    strategies = ["open_loop", "phase_locked", "phase_locked_safety"]
    labels     = ["Open-Loop Orbit", "Phase-Locked", "Phase-Locked + Safety"]
    colors     = ["tab:gray", "tab:orange", "tab:blue"]

    results = {s: run_simulation(s) for s in strategies}

    # ── Plot 1: 3D inspection trajectory (proposed strategy) ─────────────────
    fig1 = plt.figure(figsize=(10, 8))
    ax = fig1.add_subplot(111, projection='3d')

    times, pos, dist, viols, cov, inspected = results["phase_locked_safety"]

    # Turbine tower
    tower_z = np.linspace(0, HUB_HEIGHT, 50)
    ax.plot(np.zeros(50), np.zeros(50), tower_z, color='gray', lw=3, label='Tower')

    # Rotor disk outline (circle in xz at hub height)
    theta_ring = np.linspace(0, 2 * np.pi, 200)
    rx = BLADE_LENGTH * np.cos(theta_ring)
    rz = BLADE_LENGTH * np.sin(theta_ring) + HUB_HEIGHT
    ax.plot(rx, np.zeros(200), rz, color='silver', lw=1, linestyle='--', label='Rotor disk')

    # Blade snapshots at three equally-spaced times
    for snap_t in [0.0, T_MAX / 3, 2 * T_MAX / 3]:
        for i in range(N_BLADES):
            tip = blade_tip(snap_t, i)
            ax.plot([P_HUB[0], tip[0]], [P_HUB[1], tip[1]], [P_HUB[2], tip[2]],
                    color='dimgray', lw=1.5, alpha=0.4)

    # Drone trajectory (colour-coded by minimum blade distance)
    min_dist = dist.min(axis=1)
    for k in range(len(times) - 1):
        c = 'red' if min_dist[k] < D_SAFE else ('gold' if min_dist[k] < D_SAFE + 0.5 else 'tab:blue')
        ax.plot(pos[k:k+2, 0], pos[k:k+2, 1], pos[k:k+2, 2], color=c, lw=1.0)

    ax.scatter(*pos[0], color='green', s=60, zorder=5, label='Start')
    ax.scatter(*pos[-1], color='red', s=60, zorder=5, label='End')

    ax.set_xlabel('X (m)');  ax.set_ylabel('Y (m)');  ax.set_zlabel('Z (m)')
    ax.set_title(f'S062 Wind Turbine Blade Inspection — Drone Trajectory\n'
                 f'Coverage: {cov:.1f}%  |  Safety violations: {viols} steps')
    ax.legend(loc='upper left', fontsize=8)
    plt.tight_layout()
    plt.savefig('outputs/04_industrial_agriculture/s062_wind_turbine/trajectory_3d.png', dpi=150)
    plt.show()

    # ── Plot 2: Blade clearance distance over time for all three strategies ───
    fig2, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    for ax, strategy, label, color in zip(axes, strategies, labels, colors):
        t_arr, _, d_arr, viols, cov, _ = results[strategy]
        min_d = d_arr.min(axis=1)
        ax.plot(t_arr, min_d, color=color, lw=1.2, label=label)
        ax.axhline(D_SAFE, color='red', linestyle='--', lw=1.0, label=f'd_safe = {D_SAFE} m')
        ax.axhline(D_INSPECT, color='orange', linestyle=':', lw=1.0,
                   label=f'd_inspect = {D_INSPECT} m')
        ax.fill_between(t_arr, 0, D_SAFE, alpha=0.08, color='red')
        ax.set_ylabel('Min clearance (m)')
        ax.set_title(f'{label}  —  Coverage: {cov:.1f}%  |  Violations: {viols} steps')
        ax.legend(fontsize=8, loc='upper right')
        ax.set_ylim(0, None)
    axes[-1].set_xlabel('Time (s)')
    fig2.suptitle('S062 — Minimum Blade Clearance Comparison', fontsize=13)
    plt.tight_layout()
    plt.savefig('outputs/04_industrial_agriculture/s062_wind_turbine/clearance_comparison.png',
                dpi=150)
    plt.show()

    # ── Plot 3: Inspection coverage heatmap ───────────────────────────────────
    fig3, axes3 = plt.subplots(1, 3, figsize=(13, 4))
    for ax, strategy, label in zip(axes3, strategies, labels):
        _, _, _, _, _, insp = results[strategy]
        im = ax.imshow(insp.astype(float), aspect='auto', cmap='RdYlGn',
                       vmin=0, vmax=1, origin='lower',
                       extent=[0.5, N_SEG + 0.5, -0.5, N_BLADES - 0.5])
        ax.set_xlabel('Blade segment (1 m each)')
        ax.set_ylabel('Blade index')
        ax.set_yticks([0, 1, 2]);  ax.set_yticklabels(['Blade 0', 'Blade 1', 'Blade 2'])
        coverage_val = insp.sum() / (N_BLADES * N_SEG) * 100
        ax.set_title(f'{label}\nCoverage: {coverage_val:.1f}%')
        plt.colorbar(im, ax=ax, fraction=0.04, label='Inspected')
    fig3.suptitle('S062 — Blade Inspection Coverage Heatmap', fontsize=13)
    plt.tight_layout()
    plt.savefig('outputs/04_industrial_agriculture/s062_wind_turbine/coverage_heatmap.png',
                dpi=150)
    plt.show()


def animate_inspection():
    """Top-down (XZ) animation of drone orbiting the rotor plane."""
    from mpl_toolkits.mplot3d import Axes3D   # noqa: F401

    times, pos, dist, _, _, _ = run_simulation("phase_locked_safety")
    subsample = 4   # animate every 4th step (DT_anim ≈ 0.2 s)
    frames = range(0, len(times), subsample)

    fig = plt.figure(figsize=(7, 7))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim(-25, 25);  ax.set_ylim(-3, 3);  ax.set_zlim(10, 55)
    ax.set_xlabel('X (m)');  ax.set_ylabel('Y (m)');  ax.set_zlabel('Z (m)')
    ax.set_title('S062 Wind Turbine Inspection — Animation')

    # Static elements
    tower_z = np.linspace(0, HUB_HEIGHT, 30)
    ax.plot(np.zeros(30), np.zeros(30), tower_z, color='gray', lw=3)
    theta_ring = np.linspace(0, 2 * np.pi, 200)
    ax.plot(BLADE_LENGTH * np.cos(theta_ring),
            np.zeros(200),
            BLADE_LENGTH * np.sin(theta_ring) + HUB_HEIGHT,
            color='silver', lw=1, linestyle='--')

    blade_lines = [ax.plot([], [], [], color='steelblue', lw=3)[0]
                   for _ in range(N_BLADES)]
    drone_dot,  = ax.plot([], [], [], 'ro', ms=8)
    trail_line, = ax.plot([], [], [], color='tomato', lw=1.0, alpha=0.5)
    time_text   = ax.text2D(0.02, 0.96, '', transform=ax.transAxes, fontsize=10)

    trail_x, trail_y, trail_z = [], [], []
    TRAIL_LEN = 60   # frames

    def update(frame_idx):
        k = list(frames)[frame_idx]
        t = times[k]
        p = pos[k]

        trail_x.append(p[0]);  trail_y.append(p[1]);  trail_z.append(p[2])
        if len(trail_x) > TRAIL_LEN:
            trail_x.pop(0);  trail_y.pop(0);  trail_z.pop(0)

        for i, line in enumerate(blade_lines):
            tip = blade_tip(t, i)
            line.set_data([P_HUB[0], tip[0]], [P_HUB[1], tip[1]])
            line.set_3d_properties([P_HUB[2], tip[2]])

        drone_dot.set_data([p[0]], [p[1]])
        drone_dot.set_3d_properties([p[2]])
        trail_line.set_data(trail_x, trail_y)
        trail_line.set_3d_properties(trail_z)
        time_text.set_text(f't = {t:.1f} s  |  min clearance = {dist[k].min():.2f} m')
        return blade_lines + [drone_dot, trail_line, time_text]

    ani = animation.FuncAnimation(fig, update, frames=len(list(frames)),
                                  interval=100, blit=False)
    ani.save('outputs/04_industrial_agriculture/s062_wind_turbine/inspection_animation.gif',
             writer='pillow', fps=10)
    plt.show()


if __name__ == '__main__':
    import os
    os.makedirs('outputs/04_industrial_agriculture/s062_wind_turbine', exist_ok=True)
    plot_results()
    animate_inspection()
```

---

## Key Parameters

| Parameter | Symbol | Value |
|-----------|--------|-------|
| Hub height | $h_{hub}$ | 30 m |
| Blade length | $R_{blade}$ | 15 m |
| Rotor angular velocity | $\omega$ | 0.5 rad/s |
| Rotor period | $T_{rot}$ | $\approx$ 12.6 s |
| Number of blades | $N_{blades}$ | 3 |
| Blade phase spacing | $\phi_i$ | $2\pi i/3$ |
| Orbit radius (hub-centred) | $r_{orbit}$ | 17 m |
| Lateral rotor-plane offset | $y_{offset}$ | 1.0 m |
| Phase lead angle | $\phi_{offset}$ | 60° ($\pi/3$ rad) |
| Hard safety margin | $d_{safe}$ | 1.5 m |
| Target inspection standoff | $d_{inspect}$ | 1.0 m |
| Max inspection speed | $v_{max,inspect}$ | 1.0 m/s |
| GPS noise std dev | $\sigma_{GPS}$ | 0.1 m |
| PD gain $K_p$ | — | 2.0 rad/(rad·s) |
| PD gain $K_d$ | — | 0.3 s |
| Safety repulsion gain | $K_{safe}$ | 3.0 m/s² |
| Blade segments per blade | $N_{seg}$ | 15 |
| Simulation timestep | $\Delta t$ | 0.05 s |
| Mission horizon | $T_{max}$ | 120 s ($\approx$ 9.5 revolutions) |

---

## Expected Output

- **3D trajectory plot**: rotor plane cross-section showing tower (grey vertical bar), rotor disk
  outline (dashed silver circle), three blade positions at three snapshot times (translucent grey
  segments), and the drone's full orbital trajectory colour-coded by minimum blade clearance (blue
  = safe, gold = warning, red = violation); start marked green, end marked red.
- **Clearance comparison panel**: three stacked subplots (one per strategy) showing minimum blade
  clearance vs. time; horizontal dashed red line at $d_{safe}$; orange dotted line at
  $d_{inspect}$; shaded red band in the violation zone; per-strategy coverage percentage and
  violation count in the subplot title.
- **Inspection coverage heatmap**: three $3 \times 15$ boolean grids (Blade × Segment) rendered
  as a red-yellow-green image; green = inspected, red = not inspected; coverage percentage
  annotated per strategy.
- **3D inspection animation (GIF)**: rotating view of turbine hub, three blades sweeping in
  real time (blue), drone orbit (red dot + trailing red arc); blade clearance displayed as live
  text annotation; frame rate 10 fps; trail length 60 frames.
- **Console metrics** (printed): per-strategy table of coverage %, safety violations (steps and
  fraction of mission), mean blade clearance, and minimum observed clearance.

Expected quantitative results for the proposed strategy over $T_{max} = 120$ s:
- Coverage $\geq 90\%$ across all three blades
- Zero hard safety violations ($d < d_{safe}$)
- Mean blade clearance $\approx 2.5$–$3.5$ m during transit, dropping to $\approx 1.2$ m during
  active inspection passes

---

## Extensions

1. **Variable rotor speed tracking**: replace the constant $\omega$ assumption with a wind-speed-
   dependent rotor speed model $\omega(t) = \omega_0 + \Delta\omega \sin(0.05 t)$; extend the
   phase-lock controller to use a Kalman filter estimating real-time $\omega$ from the SCADA encoder
   feed, enabling operation during transient speed changes (e.g., gusting conditions).
2. **Inspection path optimisation along the blade**: instead of fixing the drone at $r_{orbit}$,
   plan a spiral radial approach for each blade — sweeping from root ($s = 0$) to tip ($s = 1$) at
   $d_{inspect}$ standoff while the blade rotates through its upper half; optimise dwell time per
   segment to minimise total inspection time subject to the safety constraint.
3. **Thermal anomaly detection**: augment the inspection quality criterion with a thermal camera
   model; simulate delamination hotspots as elevated temperature regions $\Delta T = 5$ K along
   two randomly selected blade segments; require the drone to hover within $d_{inspect} \pm 0.1$ m
   for $\geq 2$ s at each suspected anomaly before flagging it.
4. **Tower shadow turbulence**: model the aerodynamic tower shadow as a periodic reduction in
   effective rotor torque when blades pass $\theta \approx 180°$; introduce a velocity perturbation
   to the drone when it is in the wake corridor ($|x| < 2$ m, $y \approx 0$); test controller
   robustness by quantifying phase error spike amplitude during shadow passages.
5. **Multi-turbine farm inspection**: schedule three identical drones across a $3 \times 3$ wind
   farm grid (9 turbines, 300 m spacing); solve the assignment problem (Hungarian algorithm) to
   minimise total travel time between inspection sequences; enforce no-fly zones within 5 m of
   any non-assigned turbine's rotor disk.

---

## Related Scenarios

- Prerequisites: [S061 Power Line Inspection](S061_power_line_inspection.md) (proximity tracking along linear structures), [S071 Bridge Underside Inspection](S071_bridge_underside.md) (confined-space proximity control)
- Follow-ups: [S079 Offshore Wind Installation](S079_offshore_wind.md) (multi-turbine coordination and marine environment)
- Algorithmic cross-reference: [S005 Stealth Approach](../01_pursuit_evasion/S005_stealth_approach.md) (phase-based angular positioning), [S023 Moving Landing Pad](../02_logistics_delivery/S023_moving_landing_pad.md) (tracking a moving reference), [S044 Wall Crack Inspection](../03_environmental_sar/S044_wall_crack_inspection.md) (close-range surface inspection)
