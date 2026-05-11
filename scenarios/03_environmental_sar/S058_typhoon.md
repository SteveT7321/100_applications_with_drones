# S058 Typhoon Eye Probing

**Domain**: Environmental Monitoring & SAR | **Difficulty**: ⭐⭐⭐⭐ | **Status**: `[ ]` Not Started
**Algorithm**: Rankine Vortex Model + LQR Disturbance Rejection | **Dimension**: 3D

---

## Problem Definition

**Setup**: A typhoon is simulated as a Rankine vortex centred at the origin. The eye radius is
$r_{eye} = 20$ m. Tangential wind speed peaks at $V_{max} = 15$ m/s at the eyewall and decreases
both inward (solid-body rotation inside the eye) and outward (potential-vortex decay beyond the
eyewall). A single specially hardened drone begins at $r_{start} = 60$ m from the vortex centre at
altitude $z_{target} = 30$ m. Its mission is to spiral inward through the eyewall turbulence, reach
the calm eye ($r < r_{eye}$), and deploy a radiosonde payload at the eye centre.

**Roles**:
- **Drone**: single agent; 3D quadrotor dynamics subject to Rankine wind disturbances; controlled
  by an LQR with wind feedforward; follows a shrinking-radius spiral reference trajectory.
- **Wind field**: Rankine vortex giving tangential velocity $V_\theta(r)$ at every point in the
  $xy$-plane; converted to a body-frame disturbance force experienced by the drone.
- **Radiosonde**: passive payload; deployed automatically when the drone enters $r < r_{eye}$ and
  $\|p - p_{ref}\| \leq 2$ m.

**Objective**: Navigate from $r_{start}$ to the eye centre while satisfying:
1. Altitude deviation $|z - z_{target}| \leq 1$ m at all times.
2. Lateral drift from the planned spiral $\|e_{xy}(t)\| \leq d_{limit} = 5$ m; mission aborts if
   this bound is exceeded.
3. Minimise the peak lateral drift and total path length to the eye centre.

**Comparison strategies**:
1. **PD controller (no feedforward)** — pure error feedback; no knowledge of wind; expected to
   exceed $d_{limit}$ in the eyewall.
2. **LQR without feedforward** — optimal state feedback but no wind compensation; reduced drift
   versus PD but still vulnerable in the eyewall.
3. **LQR with wind feedforward** (proposed) — wind force is estimated from the Rankine model and
   cancelled before the LQR acts on the residual error.

---

## Mathematical Model

### Rankine Vortex Wind Field

The tangential wind speed at radial distance $r = \sqrt{x^2 + y^2}$ from the vortex centre is:

$$V_\theta(r) = V_{max} \min\!\left(\frac{r}{r_{eye}},\; \frac{r_{eye}}{r}\right)$$

In Cartesian coordinates the wind velocity vector (tangential, counter-clockwise) is:

$$\mathbf{V}_w(x, y) = \frac{V_\theta(r)}{r}
  \begin{pmatrix} -y \\ x \\ 0 \end{pmatrix}$$

where $r = \sqrt{x^2 + y^2} + \epsilon$ (small $\epsilon$ avoids the singularity at the origin).
The vertical wind component is neglected (horizontal vortex model).

### Aerodynamic Disturbance Force

The net wind disturbance force on the drone in the world frame, accounting for the drone's own
velocity $\mathbf{v}_d$, is modelled as quadratic drag:

$$\mathbf{F}_w = \tfrac{1}{2}\,\rho_{air}\,C_D\,A_{ref}\;
  (\mathbf{V}_w - \mathbf{v}_d)\,|\mathbf{V}_w - \mathbf{v}_d|$$

where the element-wise signed-square notation $\mathbf{a}\,|\mathbf{a}|$ means
$(a_x |a_x|,\; a_y |a_y|,\; a_z |a_z|)^\top$. Key constants: air density
$\rho_{air} = 1.225$ kg/m³, drag coefficient $C_D = 0.6$, effective frontal area
$A_{ref} = 0.04$ m².

### Simplified 3D Quadrotor Dynamics

For control design the drone is treated as a 6-DOF rigid body with linearised attitude dynamics.
The state vector is:

$$\mathbf{x} = \begin{pmatrix} p_x & p_y & p_z & v_x & v_y & v_z &
  \phi & \theta & \dot{\phi} & \dot{\theta} \end{pmatrix}^\top \in \mathbb{R}^{10}$$

where $(p_x, p_y, p_z)$ is position, $(v_x, v_y, v_z)$ is velocity, $(\phi, \theta)$ are roll and
pitch, and $(\dot{\phi}, \dot{\theta})$ are angular rates. The control input is:

$$\mathbf{u} = \begin{pmatrix} T & \tau_\phi & \tau_\theta \end{pmatrix}^\top$$

with $T$ the total thrust and $(\tau_\phi, \tau_\theta)$ the roll and pitch torques. The linearised
continuous-time plant about hover (ignoring yaw) is $\dot{\mathbf{x}} = A\mathbf{x} + B\mathbf{u}
+ B_w \mathbf{F}_w$, where:

$$A = \begin{pmatrix}
  \mathbf{0}_{3\times3} & \mathbf{I}_3 & \mathbf{0}_{3\times4} \\
  \mathbf{0}_{3\times3} & \mathbf{0}_{3\times3} & \begin{pmatrix} 0 & g & 0 & 0 \\ -g & 0 & 0 & 0 \\ 0 & 0 & 0 & 0 \end{pmatrix} \\
  \mathbf{0}_{4\times3} & \mathbf{0}_{4\times3} & \begin{pmatrix} \mathbf{0}_{2\times2} & \mathbf{I}_2 \\ \mathbf{0}_{2\times2} & \mathbf{0}_{2\times2} \end{pmatrix}
\end{pmatrix}$$

with $g = 9.81$ m/s², and $B_w = \mathrm{blkdiag}(\mathbf{0}_3,\, \frac{1}{m}\mathbf{I}_3,\, \mathbf{0}_4)$
(wind force enters as a linear acceleration).

### LQR Disturbance Rejection

The LQR solves the infinite-horizon optimal control problem:

$$J = \int_0^\infty \left(\mathbf{x}^\top Q\, \mathbf{x} + \mathbf{u}^\top R\, \mathbf{u}\right) dt$$

The optimal gain matrix $K \in \mathbb{R}^{3 \times 10}$ is obtained from the algebraic Riccati
equation:

$$A^\top P + P A - P B R^{-1} B^\top P + Q = 0, \qquad K = R^{-1} B^\top P$$

The control law with wind feedforward is:

$$\mathbf{u}(t) = -K\,\mathbf{e}(t) - K_{ff}\,\mathbf{F}_w(t)$$

where $\mathbf{e}(t) = \mathbf{x}(t) - \mathbf{x}_{ref}(t)$ is the tracking error relative to the
spiral reference, and $K_{ff} = (B^\top B)^{-1} B^\top B_w$ is the least-squares feedforward gain
that pre-cancels the wind disturbance in the input channel.

### Spiral Reference Trajectory

The reference trajectory spirals inward at constant altitude:

$$r_{ref}(t) = r_{start} - \dot{r}\, t, \qquad \psi_{ref}(t) = \omega_{spiral}\, t$$

$$p_x^{ref}(t) = r_{ref}(t)\cos\psi_{ref}(t), \quad
  p_y^{ref}(t) = r_{ref}(t)\sin\psi_{ref}(t), \quad
  p_z^{ref}(t) = z_{target}$$

Reference velocity is obtained by differentiating analytically:

$$v_x^{ref} = -\dot{r}\cos\psi_{ref} - r_{ref}\,\omega_{spiral}\sin\psi_{ref}$$
$$v_y^{ref} = -\dot{r}\sin\psi_{ref} + r_{ref}\,\omega_{spiral}\cos\psi_{ref}$$

The spiral terminates when $r_{ref} \leq r_{eye}$, after which the drone holds position at the eye
centre.

### Tracking Error and Abort Condition

The instantaneous lateral drift from the spiral reference is:

$$e(t) = \|\mathbf{p}_{xy}(t) - \mathbf{p}_{xy}^{ref}(t)\|$$

where $\mathbf{p}_{xy} = (p_x, p_y)^\top$. The altitude deviation is $|p_z(t) - z_{target}|$. The
mission is aborted at the first time $t^*$ such that:

$$e(t^*) > d_{limit} = 5 \text{ m}$$

The overall performance metric is:

$$\text{drift}_{peak} = \max_{t \in [0, T_{mission}]} e(t)$$

---

## Implementation

```python
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
from scipy.linalg import solve_continuous_are

# ── Physical constants ────────────────────────────────────────────────────────
G           = 9.81      # m/s²
MASS        = 1.0       # kg
RHO_AIR     = 1.225     # kg/m³
C_D         = 0.6       # drag coefficient
A_REF       = 0.04      # m²  effective frontal area
I_XX        = 0.01      # kg·m²  roll inertia
I_YY        = 0.01      # kg·m²  pitch inertia

# ── Typhoon parameters ────────────────────────────────────────────────────────
R_EYE       = 20.0      # m  eyewall radius
V_MAX       = 15.0      # m/s  peak tangential wind speed

# ── Trajectory parameters ────────────────────────────────────────────────────
R_START     = 60.0      # m  initial spiral radius
Z_TARGET    = 30.0      # m  target altitude
R_DOT       = 0.3       # m/s  radial inward speed
OMEGA_SPIRAL = 0.12     # rad/s  angular rate of spiral
D_LIMIT     = 5.0       # m  abort drift threshold
DEPLOY_RADIUS = 2.0     # m  radiosonde deployment radius

# ── LQR weights ───────────────────────────────────────────────────────────────
# State: [px, py, pz, vx, vy, vz, phi, theta, dphi, dtheta]
Q_DIAG = np.array([10., 10., 20.,   # position (z weighted higher)
                    2.,  2.,  4.,    # velocity
                    5.,  5.,          # attitude
                    1.,  1.])         # angular rate
R_DIAG = np.array([0.1, 1.0, 1.0])  # [thrust, tau_phi, tau_theta]

# ── Simulation ────────────────────────────────────────────────────────────────
DT          = 0.02      # s  integration timestep
T_MAX       = 300.0     # s  mission timeout


def rankine_wind(x, y):
    """Return 3D wind velocity vector [Vx, Vy, 0] from Rankine vortex."""
    r = np.sqrt(x**2 + y**2) + 1e-9
    V_theta = V_MAX * min(r / R_EYE, R_EYE / r)
    # Counter-clockwise tangential direction: (-y/r, x/r)
    Vx = -V_theta * y / r
    Vy =  V_theta * x / r
    return np.array([Vx, Vy, 0.0])


def wind_force(drone_vel, wind_vel):
    """Quadratic aerodynamic drag disturbance force (world frame)."""
    dv = wind_vel - drone_vel
    return 0.5 * RHO_AIR * C_D * A_REF * dv * np.abs(dv)


def build_linear_system():
    """
    Construct linearised continuous-time matrices A (10x10), B (10x3),
    Bw (10x3) for hover about origin.
    State: [px, py, pz, vx, vy, vz, phi, theta, dphi, dtheta]
    Input: [T (thrust deviation), tau_phi, tau_theta]
    """
    n, m = 10, 3
    A = np.zeros((n, n))
    B = np.zeros((n, m))
    Bw = np.zeros((n, 3))

    # Position kinematics
    A[0, 3] = 1.0   # dpx = vx
    A[1, 4] = 1.0   # dpy = vy
    A[2, 5] = 1.0   # dpz = vz

    # Velocity dynamics (linearised about hover)
    # dvx/dt = g * theta,  dvy/dt = -g * phi,  dvz/dt = T/m - g
    A[3, 7] =  G          # dvx from pitch
    A[4, 6] = -G          # dvy from roll
    B[5, 0] =  1.0 / MASS # dvz from thrust deviation

    # Attitude kinematics
    A[6, 8] = 1.0   # dphi  = dphi_dot
    A[7, 9] = 1.0   # dtheta = dtheta_dot

    # Attitude dynamics (torques -> angular accelerations)
    B[8, 1]  = 1.0 / I_XX   # tau_phi  -> dphi_dot
    B[9, 2]  = 1.0 / I_YY   # tau_theta -> dtheta_dot

    # Wind force enters as linear acceleration on vx, vy, vz
    Bw[3, 0] = 1.0 / MASS
    Bw[4, 1] = 1.0 / MASS
    Bw[5, 2] = 1.0 / MASS

    return A, B, Bw


def compute_lqr_gain(A, B):
    """Solve algebraic Riccati equation and return LQR gain K."""
    Q = np.diag(Q_DIAG)
    R = np.diag(R_DIAG)
    P = solve_continuous_are(A, B, Q, R)
    K = np.linalg.solve(R, B.T @ P)
    return K


def compute_feedforward_gain(B, Bw):
    """Least-squares feedforward gain: K_ff such that B @ u_ff = Bw @ Fw."""
    K_ff, _, _, _ = np.linalg.lstsq(B, Bw, rcond=None)
    return K_ff


def spiral_reference(t):
    """
    Return reference state [px, py, pz, vx, vy, vz, 0, 0, 0, 0] at time t.
    Holds at eye centre once r_ref reaches zero.
    """
    r_ref = max(R_START - R_DOT * t, 0.0)
    psi   = OMEGA_SPIRAL * t

    px_r = r_ref * np.cos(psi)
    py_r = r_ref * np.sin(psi)
    pz_r = Z_TARGET

    if r_ref > 0.0:
        vx_r = -R_DOT * np.cos(psi) - r_ref * OMEGA_SPIRAL * np.sin(psi)
        vy_r = -R_DOT * np.sin(psi) + r_ref * OMEGA_SPIRAL * np.cos(psi)
    else:
        vx_r, vy_r = 0.0, 0.0

    x_ref = np.array([px_r, py_r, pz_r, vx_r, vy_r, 0.0,
                       0.0, 0.0, 0.0, 0.0])
    return x_ref, r_ref


def run_simulation(use_feedforward=True, seed=0):
    """
    Simulate typhoon eye-probing mission.
    Returns trajectory dict and mission outcome.
    """
    rng = np.random.default_rng(seed)

    A, B, Bw = build_linear_system()
    K  = compute_lqr_gain(A, B)
    Kff = compute_feedforward_gain(B, Bw) if use_feedforward else np.zeros((3, 3))

    # Initialise state at r_start, z_target
    x = np.array([R_START, 0.0, Z_TARGET,
                  0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0], dtype=float)

    t = 0.0
    log = dict(t=[], pos=[], vel=[], drift=[], alt_dev=[], wind=[])
    mission_success = False
    aborted = False
    radiosonde_deployed = False

    while t < T_MAX:
        x_ref, r_ref = spiral_reference(t)
        e = x - x_ref

        # Current wind at drone position
        Vw = rankine_wind(x[0], x[1])
        Fw = wind_force(x[3:6], Vw)

        # LQR control + optional feedforward
        u = -K @ e - Kff @ Fw

        # Thrust saturation (physical limits)
        u[0] = np.clip(u[0], -MASS * G * 0.8, MASS * G * 1.2)

        # Compute acceleration from full nonlinear force (simplified)
        acc = np.array([
            G * x[7],                         # ax = g*theta
           -G * x[6],                          # ay = -g*phi
            u[0] / MASS - G,                   # az = T/m - g
        ]) + Fw / MASS

        # Attitude dynamics
        alpha_phi   = u[1] / I_XX
        alpha_theta = u[2] / I_YY

        # Euler integration
        x[0:3] += x[3:6] * DT
        x[3:6] += acc * DT
        x[6]   += x[8] * DT
        x[7]   += x[9] * DT
        x[8]   += alpha_phi * DT
        x[9]   += alpha_theta * DT

        # Attitude angle clamp (small-angle regime)
        x[6:8] = np.clip(x[6:8], -0.5, 0.5)

        # Metrics
        r_drone = np.sqrt(x[0]**2 + x[1]**2)
        drift   = np.linalg.norm(x[0:2] - x_ref[0:2])
        alt_dev = abs(x[2] - Z_TARGET)

        log['t'].append(t)
        log['pos'].append(x[0:3].copy())
        log['vel'].append(x[3:6].copy())
        log['drift'].append(drift)
        log['alt_dev'].append(alt_dev)
        log['wind'].append(Vw.copy())

        # Abort check
        if drift > D_LIMIT:
            aborted = True
            break

        # Radiosonde deployment
        if not radiosonde_deployed and r_drone < R_EYE and drift < DEPLOY_RADIUS:
            radiosonde_deployed = True

        # Success: reached eye centre and deployed
        if radiosonde_deployed and r_ref == 0.0:
            mission_success = True
            break

        t += DT

    # Convert logs
    for k in ('pos', 'vel', 'wind'):
        log[k] = np.array(log[k])
    for k in ('t', 'drift', 'alt_dev'):
        log[k] = np.array(log[k])

    outcome = {
        'success': mission_success,
        'aborted': aborted,
        'radiosonde_deployed': radiosonde_deployed,
        'peak_drift': float(log['drift'].max()) if len(log['drift']) else np.nan,
        'peak_alt_dev': float(log['alt_dev'].max()) if len(log['alt_dev']) else np.nan,
        'mission_time': float(log['t'][-1]) if len(log['t']) else np.nan,
    }
    return log, outcome


def plot_wind_field(ax=None):
    """Plot 2D Rankine vortex wind field as a quiver plot."""
    if ax is None:
        fig, ax = plt.subplots(figsize=(7, 7))
    grid = np.linspace(-70, 70, 25)
    X, Y = np.meshgrid(grid, grid)
    Vx = np.zeros_like(X)
    Vy = np.zeros_like(Y)
    Vmag = np.zeros_like(X)
    for i in range(X.shape[0]):
        for j in range(X.shape[1]):
            Vw = rankine_wind(X[i, j], Y[i, j])
            Vx[i, j], Vy[i, j] = Vw[0], Vw[1]
            Vmag[i, j] = np.sqrt(Vw[0]**2 + Vw[1]**2)
    ax.quiver(X, Y, Vx, Vy, Vmag, cmap='plasma', alpha=0.8)
    circle_eye = plt.Circle((0, 0), R_EYE, color='cyan', fill=False,
                             linestyle='--', linewidth=2, label=f'Eye ($r_{{eye}}={R_EYE}$ m)')
    ax.add_patch(circle_eye)
    ax.set_aspect('equal')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title('Rankine Vortex Wind Field')
    ax.legend()
    return ax


def plot_results(log_ff, log_noff, outcome_ff, outcome_noff):
    """Generate comparison figures for LQR+feedforward vs LQR-only."""
    fig = plt.figure(figsize=(18, 12))

    # ── Plot 1: 2D wind field with spiral overlay ─────────────────────────────
    ax1 = fig.add_subplot(2, 3, 1)
    plot_wind_field(ax1)
    pos_ff = log_ff['pos']
    ax1.plot(pos_ff[:, 0], pos_ff[:, 1], 'r-', linewidth=1.5, label='LQR+FF')
    pos_noff = log_noff['pos']
    ax1.plot(pos_noff[:, 0], pos_noff[:, 1], 'b--', linewidth=1.5, label='LQR only')
    ax1.scatter([R_START], [0], c='green', zorder=5, s=80, label='Start')
    ax1.scatter([0], [0], c='magenta', zorder=5, s=120, marker='*', label='Eye centre')
    ax1.legend(fontsize=8)
    ax1.set_title('Horizontal trajectories + Wind field')

    # ── Plot 2: 3D spiral trajectory ──────────────────────────────────────────
    ax2 = fig.add_subplot(2, 3, 2, projection='3d')
    ax2.plot(pos_ff[:, 0], pos_ff[:, 1], pos_ff[:, 2],
             'r-', linewidth=1.5, label='LQR+FF')
    ax2.plot(pos_noff[:, 0], pos_noff[:, 1], pos_noff[:, 2],
             'b--', linewidth=1.5, label='LQR only')
    # Reference spiral
    t_ref = np.linspace(0, (R_START / R_DOT), 400)
    px_r = np.maximum(R_START - R_DOT * t_ref, 0) * np.cos(OMEGA_SPIRAL * t_ref)
    py_r = np.maximum(R_START - R_DOT * t_ref, 0) * np.sin(OMEGA_SPIRAL * t_ref)
    ax2.plot(px_r, py_r, np.full_like(px_r, Z_TARGET),
             'g:', linewidth=1, label='Reference spiral')
    ax2.set_xlabel('x (m)')
    ax2.set_ylabel('y (m)')
    ax2.set_zlabel('z (m)')
    ax2.set_title('3D Spiral Trajectory')
    ax2.legend(fontsize=8)

    # ── Plot 3: Lateral drift over time ───────────────────────────────────────
    ax3 = fig.add_subplot(2, 3, 3)
    ax3.plot(log_ff['t'], log_ff['drift'], 'r-', label='LQR+FF')
    ax3.plot(log_noff['t'], log_noff['drift'], 'b--', label='LQR only')
    ax3.axhline(D_LIMIT, color='k', linestyle=':', linewidth=1.5, label=f'Abort limit ({D_LIMIT} m)')
    ax3.axhline(R_EYE, color='cyan', linestyle=':', linewidth=1,
                label=f'Eye radius ({R_EYE} m)')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Lateral drift (m)')
    ax3.set_title('Tracking Error vs Time')
    ax3.legend(fontsize=8)

    # ── Plot 4: Altitude deviation ────────────────────────────────────────────
    ax4 = fig.add_subplot(2, 3, 4)
    ax4.plot(log_ff['t'], log_ff['alt_dev'], 'r-', label='LQR+FF')
    ax4.plot(log_noff['t'], log_noff['alt_dev'], 'b--', label='LQR only')
    ax4.axhline(1.0, color='k', linestyle=':', linewidth=1.5, label='±1 m limit')
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Altitude deviation (m)')
    ax4.set_title('Altitude Deviation vs Time')
    ax4.legend(fontsize=8)

    # ── Plot 5: Wind speed along trajectory ───────────────────────────────────
    ax5 = fig.add_subplot(2, 3, 5)
    wind_mag_ff = np.linalg.norm(log_ff['wind'], axis=1)
    r_ff = np.linalg.norm(log_ff['pos'][:, :2], axis=1)
    ax5.scatter(r_ff, wind_mag_ff, s=2, c='red', alpha=0.4, label='LQR+FF')
    r_vals = np.linspace(0, 70, 300)
    V_theory = V_MAX * np.minimum(r_vals / R_EYE, R_EYE / r_vals)
    ax5.plot(r_vals, V_theory, 'k-', linewidth=1.5, label='Rankine profile')
    ax5.axvline(R_EYE, color='cyan', linestyle='--', label=f'$r_{{eye}}$')
    ax5.set_xlabel('Radial distance r (m)')
    ax5.set_ylabel('Wind speed (m/s)')
    ax5.set_title('Wind Speed vs Radial Position')
    ax5.legend(fontsize=8)

    # ── Plot 6: Mission outcome summary ───────────────────────────────────────
    ax6 = fig.add_subplot(2, 3, 6)
    labels = ['LQR+FF', 'LQR only']
    peak_drifts = [outcome_ff['peak_drift'], outcome_noff['peak_drift']]
    colours = ['red', 'steelblue']
    bars = ax6.bar(labels, peak_drifts, color=colours, alpha=0.8)
    ax6.axhline(D_LIMIT, color='k', linestyle=':', linewidth=1.5,
                label=f'Abort limit ({D_LIMIT} m)')
    for bar, od, val in zip(bars,
                            [outcome_ff, outcome_noff],
                            peak_drifts):
        label = f'{val:.2f} m'
        if od['aborted']:
            label += '\n(ABORTED)'
        elif od['success']:
            label += '\n(SUCCESS)'
        ax6.text(bar.get_x() + bar.get_width() / 2, val + 0.1,
                 label, ha='center', va='bottom', fontsize=9)
    ax6.set_ylabel('Peak lateral drift (m)')
    ax6.set_title('Mission Outcome Summary')
    ax6.legend(fontsize=8)

    plt.tight_layout()
    plt.savefig('outputs/03_environmental_sar/s058_typhoon/trajectory_analysis.png',
                dpi=150, bbox_inches='tight')
    plt.show()


def animate_spiral(log):
    """Animate drone spiralling into the typhoon eye (top-down view)."""
    pos = log['pos']
    n_frames = len(pos)
    step = max(1, n_frames // 200)

    fig, ax = plt.subplots(figsize=(7, 7))
    plot_wind_field(ax)
    trail, = ax.plot([], [], 'r-', linewidth=1.5, label='Drone path')
    dot,   = ax.plot([], [], 'ro', markersize=8)
    time_text = ax.text(-65, 60, '', fontsize=10)

    def init():
        trail.set_data([], [])
        dot.set_data([], [])
        return trail, dot, time_text

    def update(frame):
        idx = frame * step
        trail.set_data(pos[:idx, 0], pos[:idx, 1])
        dot.set_data([pos[idx, 0]], [pos[idx, 1]])
        time_text.set_text(f't = {log["t"][idx]:.1f} s  drift = {log["drift"][idx]:.2f} m')
        return trail, dot, time_text

    anim = FuncAnimation(fig, update, frames=n_frames // step,
                         init_func=init, blit=True, interval=50)
    anim.save('outputs/03_environmental_sar/s058_typhoon/spiral_entry.gif',
              writer='pillow', fps=20)
    plt.close()


if __name__ == '__main__':
    print('Running S058 Typhoon Eye Probing ...')

    log_ff,   outcome_ff   = run_simulation(use_feedforward=True,  seed=0)
    log_noff, outcome_noff = run_simulation(use_feedforward=False, seed=0)

    print(f"LQR+FF   : success={outcome_ff['success']}  "
          f"aborted={outcome_ff['aborted']}  "
          f"peak_drift={outcome_ff['peak_drift']:.2f} m  "
          f"peak_alt_dev={outcome_ff['peak_alt_dev']:.2f} m")
    print(f"LQR only : success={outcome_noff['success']}  "
          f"aborted={outcome_noff['aborted']}  "
          f"peak_drift={outcome_noff['peak_drift']:.2f} m  "
          f"peak_alt_dev={outcome_noff['peak_alt_dev']:.2f} m")

    plot_results(log_ff, log_noff, outcome_ff, outcome_noff)
    animate_spiral(log_ff)
    print('Done. Outputs saved to outputs/03_environmental_sar/s058_typhoon/')
```

---

## Key Parameters

| Parameter | Symbol | Value |
|-----------|--------|-------|
| Eye radius | $r_{eye}$ | 20 m |
| Peak tangential wind speed | $V_{max}$ | 15 m/s |
| Initial spiral radius | $r_{start}$ | 60 m |
| Target altitude | $z_{target}$ | 30 m |
| Radial inward speed | $\dot{r}$ | 0.3 m/s |
| Spiral angular rate | $\omega_{spiral}$ | 0.12 rad/s |
| Abort drift threshold | $d_{limit}$ | 5 m |
| Radiosonde deploy radius | — | 2 m |
| Drone mass | $m$ | 1.0 kg |
| Air density | $\rho_{air}$ | 1.225 kg/m³ |
| Drag coefficient | $C_D$ | 0.6 |
| Effective frontal area | $A_{ref}$ | 0.04 m² |
| Roll / pitch inertia | $I_{xx}, I_{yy}$ | 0.01 kg·m² |
| LQR position weights | $Q_{p}$ | diag(10, 10, 20) |
| LQR velocity weights | $Q_{v}$ | diag(2, 2, 4) |
| LQR attitude weights | $Q_{\phi\theta}$ | diag(5, 5) |
| LQR input weight | $R$ | diag(0.1, 1, 1) |
| Simulation timestep | $\Delta t$ | 0.02 s |
| Mission timeout | $T_{max}$ | 300 s |

---

## Expected Output

- **Wind field quiver plot**: 2D top-down quiver map of the Rankine vortex over a $140 \times 140$ m
  domain; colour-coded by wind speed magnitude; eyewall circle ($r = r_{eye}$) marked; overlaid
  horizontal trajectories of LQR+FF (red) and LQR-only (blue dashed).
- **3D spiral trajectory**: `mpl_toolkits.mplot3d` plot showing actual drone path versus reference
  spiral (green dotted); altitude deviation visible on the $z$-axis; both controllers compared.
- **Lateral drift vs time**: time series of $e(t)$ for both strategies; abort limit $d_{limit}$
  shown as a dashed horizontal line; eyewall transit interval shaded; radiosonde deployment instant
  marked with a vertical line.
- **Altitude deviation vs time**: $|p_z - z_{target}|$ over the mission; ±1 m bound shown;
  LQR+FF expected to stay well within bound throughout.
- **Wind speed vs radial position**: scatter of wind magnitude experienced by the drone versus
  instantaneous $r$, overlaid on the theoretical Rankine profile; shows peak exposure at $r_{eye}$.
- **Mission outcome bar chart**: peak lateral drift for each strategy with abort-limit reference;
  SUCCESS / ABORTED label on each bar.
- **Spiral entry animation (GIF)**: top-down drone trail growing frame-by-frame on the quiver
  background; current time and drift annotated; mission result displayed on completion.

**Expected metric targets** (LQR+FF strategy):

| Metric | Target |
|--------|--------|
| Peak lateral drift | $< 4$ m |
| Peak altitude deviation | $< 0.5$ m |
| Mission outcome | SUCCESS (radiosonde deployed) |
| LQR-only outcome | ABORTED (drift $> 5$ m in eyewall) |

---

## Extensions

1. **Stochastic wind gusts**: superimpose a turbulence model (e.g. Dryden or von Kármán spectrum)
   on the deterministic Rankine field; evaluate LQR robustness versus an $H_\infty$ robust controller
   designed for a bounded gust amplitude.
2. **Online wind estimation**: replace the known-wind feedforward with an onboard EKF that estimates
   local wind velocity from the difference between commanded and actual acceleration; evaluate
   convergence speed and drift penalty during the estimation transient.
3. **Asymmetric eyewall**: replace the perfectly circular Rankine model with an elliptic eyewall
   (different $r_{eye}$ in cardinal directions); re-optimise the spiral trajectory and assess the
   increased directional sensitivity of the controller.
4. **Multi-drone coordinated entry**: deploy three drones simultaneously at equal angular spacing
   ($120°$ apart) with a shared entry trajectory plan; study wind interference and collision
   avoidance inside the constrained eye volume.
5. **Real radiosonde descent modelling**: after deployment, simulate the parachute-borne radiosonde
   drifting with the wind as it descends from $z_{target}$; compute the ground impact point and
   compare with and without the Rankine eye correction.

---

## Related Scenarios

- Prerequisites: [S041 Wildfire Boundary Scan](S041_wildfire_boundary_scan.md) (environmental
  monitoring structure), [S045 Chemical Plume Tracing](S045_plume_tracing.md) (wind-field
  navigation), [S060 Glacier Crack Survey](S060_glacier_crack_survey.md) (high-difficulty SAR
  culmination)
- Algorithmic cross-reference: [S001 Basic Intercept](../01_pursuit_evasion/S001_basic_intercept.md)
  (quadrotor dynamics reference), [S050 Swarm Cooperative Mapping](S050_slam.md) (complex 3D
  mission structure)
- Follow-ups: deploy the trained controller in a full physics simulation using gym-pybullet-drones
  with custom wind plugin; extend to adaptive MPC for real-time eyewall trajectory optimisation
