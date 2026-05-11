# S079 Offshore Wind Farm Installation Assistance

**Domain**: Industrial & Agriculture | **Difficulty**: ⭐⭐⭐ | **Status**: `[ ]` Not Started
**Algorithm**: Swell Feedforward Prediction + LQR Landing Controller | **Dimension**: 3D

---

## Problem Definition

**Setup**: An offshore wind turbine installation vessel is moored at sea. Its deck platform heaves
vertically with ocean swell modelled as a deterministic sinusoid with amplitude $A_{swell} = 0.8$ m
and period $T_{swell} = 6$ s, producing a target point that rises and falls continuously. A delivery
drone must carry a small component (bolt cluster, sensor module) from a fixed supply station at
$(x_0, y_0, z_0) = (0, 0, 30)$ m to the moving target point on top of the platform mast at nominal
height $z_{mast} = 20$ m above mean sea level. Wind gusts are modelled as zero-mean Gaussian
horizontal disturbances $\mathbf{w}(t) \sim \mathcal{N}(\mathbf{0}, \sigma_w^2 \mathbf{I}_2)$
added to the horizontal drone velocity at each timestep.

**Roles**:
- **Drone**: single delivery agent; 3D quadrotor dynamics; controlled by an LQR with swell
  feedforward; must match the platform's vertical velocity and position simultaneously within the
  landing window.
- **Platform target**: a point on top of the installation mast moving with heave motion
  $z_p(t) = z_{mast} + A_{swell} \sin(2\pi t / T_{swell})$; horizontal position fixed at
  $(x_p, y_p) = (10, 0)$ m.
- **Landing window**: a 3 s interval centred on the moment of minimum relative $z$-velocity; the
  drone must touch down (position error $< r_{land}$, velocity error $< v_{land,max}$) within
  this window.

**Objective**: Deliver the component to the heaving platform precisely and safely. Success requires:
1. Position error $\|\mathbf{p}_{drone} - \mathbf{p}_{target}\| \leq r_{land} = 0.15$ m at
   the moment of landing.
2. Relative vertical velocity $|\dot{z}_{drone} - \dot{z}_p| \leq v_{land,max} = 0.1$ m/s.
3. Horizontal gust does not exceed $\sigma_w = 0.05$ m/s (standard deviation); robustness tested
   via Monte Carlo over $N_{trials} = 200$ randomised gust realisations.

**Controller variants** (three compared):
1. **PID (no feedforward)** — pure position error feedback; unaware of platform motion; expected to
   miss the landing window frequently due to phase lag.
2. **LQR (no feedforward)** — optimal state-feedback on tracking error; reduced lag versus PID but
   no predictive swell compensation.
3. **LQR + swell feedforward** (proposed) — LQR feedback augmented with a look-ahead reference
   $z_{ref}(t+\tau)$ predicted $\tau = 0.5$ s ahead; near-zero phase lag; highest landing success
   rate.

---

## Mathematical Model

### Platform Heave Motion

The vertical position and velocity of the platform landing target are:

$$z_p(t) = z_{mast} + A_{swell} \sin\!\left(\frac{2\pi t}{T_{swell}}\right)$$

$$\dot{z}_p(t) = A_{swell} \cdot \frac{2\pi}{T_{swell}} \cos\!\left(\frac{2\pi t}{T_{swell}}\right)$$

The horizontal position of the target is fixed: $(x_p, y_p) = (10, 0)$ m. The full target state
vector is therefore:

$$\mathbf{p}_{target}(t) = \begin{pmatrix} x_p \\ y_p \\ z_p(t) \end{pmatrix}, \quad
  \mathbf{v}_{target}(t) = \begin{pmatrix} 0 \\ 0 \\ \dot{z}_p(t) \end{pmatrix}$$

### Swell Feedforward Prediction

The controller predicts the platform state $\tau = 0.5$ s ahead to compensate for actuator and
dynamics delay. The predictive reference altitude and its rate are:

$$z_{ref}(t+\tau) = z_{mast} + A_{swell} \sin\!\left(\frac{2\pi(t+\tau)}{T_{swell}}\right)$$

$$\dot{z}_{ref}(t+\tau) = A_{swell} \cdot \frac{2\pi}{T_{swell}}
  \cos\!\left(\frac{2\pi(t+\tau)}{T_{swell}}\right)$$

The full 6-DOF reference state fed to the LQR is:

$$\mathbf{x}_{ref}(t) = \begin{pmatrix}
  x_p \\ y_p \\ z_{ref}(t+\tau) \\ 0 \\ 0 \\ \dot{z}_{ref}(t+\tau) \\
  0 \\ 0 \\ 0 \\ 0
\end{pmatrix}$$

### Landing Window Detection

The drone is eligible to commit to a landing attempt when the relative vertical velocity between
drone and platform is sufficiently small. Define the signed relative velocity:

$$\Delta\dot{z}(t) = \dot{z}_{drone}(t) - \dot{z}_p(t)$$

A landing window of duration $T_{window} = 3$ s opens at the first time $t_w$ satisfying:

$$|\Delta\dot{z}(t_w)| \leq v_{land,max} = 0.1 \text{ m/s}$$

Landing is declared **successful** if, within the window $[t_w,\; t_w + T_{window}]$, all three
conditions hold simultaneously:

$$\left| z_{drone}(t) - z_p(t) \right| \leq r_{land}, \qquad
  \left| \Delta\dot{z}(t) \right| \leq v_{land,max}, \qquad
  \left\| \mathbf{p}_{xy,drone}(t) - \mathbf{p}_{xy,target} \right\| \leq r_{land}$$

If no such instant exists inside the window, the attempt is counted as a **miss** and the window
counter increments.

### Simplified 3D Quadrotor Dynamics

The state vector is:

$$\mathbf{x} = \begin{pmatrix}
  p_x & p_y & p_z & v_x & v_y & v_z & \phi & \theta & \dot{\phi} & \dot{\theta}
\end{pmatrix}^\top \in \mathbb{R}^{10}$$

The linearised continuous-time hover dynamics are $\dot{\mathbf{x}} = A\mathbf{x} + B\mathbf{u}
+ \mathbf{d}$, where the disturbance vector $\mathbf{d}$ carries both wind gust accelerations on
$v_x, v_y$ and a swell velocity term on $v_z$.

$$A = \begin{pmatrix}
  \mathbf{0}_{3\times3} & \mathbf{I}_3 & \mathbf{0}_{3\times4} \\
  \mathbf{0}_{3\times3} & \mathbf{0}_{3\times3}
    & \begin{pmatrix} 0 & g & 0 & 0 \\ -g & 0 & 0 & 0 \\ 0 & 0 & 0 & 0 \end{pmatrix} \\
  \mathbf{0}_{4\times3} & \mathbf{0}_{4\times3}
    & \begin{pmatrix} \mathbf{0}_{2\times2} & \mathbf{I}_2 \\
      \mathbf{0}_{2\times2} & \mathbf{0}_{2\times2} \end{pmatrix}
\end{pmatrix}$$

The control input $\mathbf{u} = (T,\, \tau_\phi,\, \tau_\theta)^\top$ enters through:

$$B[5,0] = \frac{1}{m}, \quad B[8,1] = \frac{1}{I_{xx}}, \quad B[9,2] = \frac{1}{I_{yy}}$$

with $g = 9.81$ m/s², drone mass $m = 1.0$ kg, and inertias $I_{xx} = I_{yy} = 0.01$ kg·m².

### LQR Formulation

The LQR minimises the infinite-horizon cost:

$$J = \int_0^\infty \left(\mathbf{e}^\top Q\,\mathbf{e} + \mathbf{u}^\top R\,\mathbf{u}\right) dt,
  \quad \mathbf{e}(t) = \mathbf{x}(t) - \mathbf{x}_{ref}(t)$$

The optimal gain $K \in \mathbb{R}^{3 \times 10}$ satisfies the algebraic Riccati equation:

$$A^\top P + P A - P B R^{-1} B^\top P + Q = 0, \qquad K = R^{-1} B^\top P$$

The LQR control law with swell feedforward is:

$$\mathbf{u}(t) = -K\,\mathbf{e}(t)$$

where $\mathbf{e}(t)$ already incorporates the predicted reference $\mathbf{x}_{ref}(t)$, so the
feedforward action is implicit: tracking a time-varying sinusoidal reference with a $\tau$-second
look-ahead effectively cancels the swell phase lag.

### Wind Gust Disturbance

Horizontal wind gusts are modelled as independent zero-mean white-noise accelerations:

$$w_x(t),\, w_y(t) \;\sim\; \mathcal{N}(0,\, \sigma_w^2), \quad \sigma_w = 0.05 \text{ m/s}$$

These are added directly to the $v_x$ and $v_y$ components each timestep. The LQR's integral-like
accumulation of position error provides passive rejection; no explicit gust observer is needed for
this difficulty level.

### Monte Carlo Landing Success Rate

Run $N_{trials} = 200$ independent simulations, each with a freshly sampled gust sequence:

$$P_{land} = \frac{N_{successes}}{N_{trials}} \times 100\%$$

where $N_{successes}$ is the count of trials in which the drone achieves a successful landing
within the first opened window. Window miss count $N_{miss}$ is also recorded per trial.

---

## Implementation

```python
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
from scipy.linalg import solve_continuous_are

# ── Physical constants ────────────────────────────────────────────────────────
G        = 9.81      # m/s²
MASS     = 1.0       # kg
I_XX     = 0.01      # kg·m²
I_YY     = 0.01      # kg·m²

# ── Swell parameters ──────────────────────────────────────────────────────────
A_SWELL  = 0.8       # m   heave amplitude
T_SWELL  = 6.0       # s   swell period
Z_MAST   = 20.0      # m   nominal mast height above sea level
X_TARGET = 10.0      # m   horizontal target position
Y_TARGET = 0.0       # m

# ── Drone start position ──────────────────────────────────────────────────────
Z_START  = 30.0      # m   initial altitude (supply station)
X_START  = 0.0       # m
Y_START  = 0.0       # m

# ── Landing criteria ──────────────────────────────────────────────────────────
R_LAND        = 0.15   # m    landing position tolerance
V_LAND_MAX    = 0.10   # m/s  landing relative velocity tolerance
T_WINDOW      = 3.0    # s    landing window duration
TAU_PREDICT   = 0.5    # s    feedforward look-ahead

# ── Disturbance ───────────────────────────────────────────────────────────────
SIGMA_W       = 0.05   # m/s  horizontal gust std dev

# ── LQR weights ───────────────────────────────────────────────────────────────
# State: [px, py, pz, vx, vy, vz, phi, theta, dphi, dtheta]
Q_DIAG = np.array([20., 20., 50.,   # position (z weighted heavily)
                    3.,  3.,  8.,   # velocity (vz weighted for swell tracking)
                    5.,  5.,        # attitude
                    1.,  1.])       # angular rates
R_DIAG = np.array([0.1, 1.0, 1.0]) # [thrust, tau_phi, tau_theta]

# ── Simulation ────────────────────────────────────────────────────────────────
DT     = 0.02    # s
T_MAX  = 40.0    # s   mission horizon (several swell cycles)
N_TRIALS = 200   # Monte Carlo runs


# ── Platform kinematics ───────────────────────────────────────────────────────

def platform_z(t):
    """Vertical position of the platform landing target."""
    return Z_MAST + A_SWELL * np.sin(2 * np.pi * t / T_SWELL)


def platform_zdot(t):
    """Vertical velocity of the platform landing target."""
    return A_SWELL * (2 * np.pi / T_SWELL) * np.cos(2 * np.pi * t / T_SWELL)


def reference_state(t, use_feedforward=True):
    """
    Return the 10-D reference state the LQR should track at time t.
    With feedforward: target altitude is predicted TAU_PREDICT seconds ahead.
    """
    t_pred = t + TAU_PREDICT if use_feedforward else t
    z_ref    = platform_z(t_pred)
    vz_ref   = platform_zdot(t_pred)
    return np.array([X_TARGET, Y_TARGET, z_ref,
                     0.0, 0.0, vz_ref,
                     0.0, 0.0, 0.0, 0.0])


# ── Linear system matrices ────────────────────────────────────────────────────

def build_linear_system():
    """
    Linearised 3D quadrotor about hover.
    State: [px, py, pz, vx, vy, vz, phi, theta, dphi, dtheta]
    Input: [T (thrust deviation from mg), tau_phi, tau_theta]
    """
    n, m = 10, 3
    A  = np.zeros((n, n))
    B  = np.zeros((n, m))

    # Position kinematics
    A[0, 3] = 1.0
    A[1, 4] = 1.0
    A[2, 5] = 1.0

    # Translational dynamics
    A[3, 7] =  G          # dvx = g * theta
    A[4, 6] = -G          # dvy = -g * phi
    B[5, 0] =  1.0 / MASS # dvz = T/m

    # Attitude kinematics
    A[6, 8] = 1.0
    A[7, 9] = 1.0

    # Attitude dynamics
    B[8, 1] = 1.0 / I_XX
    B[9, 2] = 1.0 / I_YY

    return A, B


def compute_lqr(A, B):
    """Solve CARE and return optimal gain K."""
    Q = np.diag(Q_DIAG)
    R = np.diag(R_DIAG)
    P = solve_continuous_are(A, B, Q, R)
    K = np.linalg.solve(R, B.T @ P)
    return K


# ── Simulation core ───────────────────────────────────────────────────────────

def run_simulation(use_feedforward=True, seed=0):
    """
    Simulate the drone landing approach.
    Returns trajectory dict and outcome metrics.
    """
    rng = np.random.default_rng(seed)

    A, B = build_linear_system()
    K    = compute_lqr(A, B)

    # Initial state: drone at supply station, hovering
    x = np.array([X_START, Y_START, Z_START,
                  0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0], dtype=float)

    t = 0.0
    log = dict(t=[], pos=[], vel=[], z_platform=[], dz_rel=[], window_open=False)
    landed         = False
    window_open    = False
    window_start   = None
    window_miss    = 0

    while t < T_MAX and not landed:
        x_ref = reference_state(t, use_feedforward)
        e     = x - x_ref

        # LQR command
        u = -K @ e
        u[0] = np.clip(u[0], -MASS * G * 0.9, MASS * G * 1.5)

        # Wind gust disturbance on horizontal velocity
        wx = rng.normal(0.0, SIGMA_W)
        wy = rng.normal(0.0, SIGMA_W)

        # Acceleration from control + gravity
        ax = G * x[7]
        ay = -G * x[6]
        az = u[0] / MASS - G

        # Euler integration
        x[0:3] += x[3:6] * DT
        x[3]   += (ax + wx) * DT
        x[4]   += (ay + wy) * DT
        x[5]   += az * DT
        x[6]   += x[8] * DT
        x[7]   += x[9] * DT
        x[8]   += (u[1] / I_XX) * DT
        x[9]   += (u[2] / I_YY) * DT
        x[6:8]  = np.clip(x[6:8], -0.4, 0.4)

        # Platform state
        zp      = platform_z(t)
        zdot_p  = platform_zdot(t)
        dz_rel  = x[5] - zdot_p   # relative vertical velocity

        # Landing window logic
        if not window_open and abs(dz_rel) <= V_LAND_MAX:
            window_open  = True
            window_start = t

        if window_open:
            elapsed = t - window_start
            pos_err_xy = np.sqrt((x[0] - X_TARGET)**2 + (x[1] - Y_TARGET)**2)
            pos_err_z  = abs(x[2] - zp)
            if pos_err_xy <= R_LAND and pos_err_z <= R_LAND and abs(dz_rel) <= V_LAND_MAX:
                landed = True
            elif elapsed > T_WINDOW:
                window_open  = False
                window_start = None
                window_miss += 1

        # Logging
        log['t'].append(t)
        log['pos'].append(x[0:3].copy())
        log['vel'].append(x[3:6].copy())
        log['z_platform'].append(zp)
        log['dz_rel'].append(dz_rel)

        t += DT

    for k in ('pos', 'vel'):
        log[k] = np.array(log[k])
    log['t']          = np.array(log['t'])
    log['z_platform'] = np.array(log['z_platform'])
    log['dz_rel']     = np.array(log['dz_rel'])

    outcome = {
        'landed':       landed,
        'window_miss':  window_miss,
        'final_pos_err': float(np.linalg.norm(log['pos'][-1] - [X_TARGET, Y_TARGET,
                                                                  platform_z(log['t'][-1])])),
        'peak_dz_rel':  float(np.max(np.abs(log['dz_rel']))),
        'mission_time': float(log['t'][-1]),
    }
    return log, outcome


def run_monte_carlo(use_feedforward=True, n_trials=N_TRIALS):
    """Run Monte Carlo trials; return success rate and per-trial metrics."""
    successes   = 0
    miss_counts = []
    for seed in range(n_trials):
        _, outcome = run_simulation(use_feedforward=use_feedforward, seed=seed)
        if outcome['landed']:
            successes += 1
        miss_counts.append(outcome['window_miss'])
    p_land = successes / n_trials * 100.0
    return p_land, np.array(miss_counts)


# ── Visualisation ─────────────────────────────────────────────────────────────

def plot_results(log_ff, log_noff, outcome_ff, outcome_noff):
    """Four-panel comparison figure."""
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

    fig = plt.figure(figsize=(18, 14))

    # ── Panel 1: 3D approach trajectory ──────────────────────────────────────
    ax1 = fig.add_subplot(2, 2, 1, projection='3d')
    pos_ff   = log_ff['pos']
    pos_noff = log_noff['pos']
    ax1.plot(pos_ff[:, 0],   pos_ff[:, 1],   pos_ff[:, 2],
             'r-', lw=1.5, label='LQR+FF')
    ax1.plot(pos_noff[:, 0], pos_noff[:, 1], pos_noff[:, 2],
             'b--', lw=1.5, label='LQR only')
    # Platform column
    z_col = np.linspace(0, Z_MAST + A_SWELL + 0.5, 30)
    ax1.plot(np.full(30, X_TARGET), np.zeros(30), z_col,
             color='gray', lw=4, label='Mast')
    # Target heave band
    t_band = np.linspace(0, T_MAX, 300)
    zp_band = platform_z(t_band)
    ax1.scatter(np.full(300, X_TARGET), np.zeros(300), zp_band,
                c='green', s=2, alpha=0.4, label='Platform motion')
    ax1.scatter([X_START], [Y_START], [Z_START],
                c='blue', s=80, zorder=5, label='Start')
    ax1.set_xlabel('x (m)');  ax1.set_ylabel('y (m)');  ax1.set_zlabel('z (m)')
    ax1.set_title('3D Approach Trajectory')
    ax1.legend(fontsize=8)

    # ── Panel 2: Vertical position vs time ───────────────────────────────────
    ax2 = fig.add_subplot(2, 2, 2)
    ax2.plot(log_ff['t'],   log_ff['pos'][:, 2],   'r-',  lw=1.5, label='Drone z (LQR+FF)')
    ax2.plot(log_noff['t'], log_noff['pos'][:, 2],  'b--', lw=1.5, label='Drone z (LQR only)')
    ax2.plot(log_ff['t'],   log_ff['z_platform'],    'g-',  lw=2,   label='Platform $z_p(t)$')
    ax2.axhline(Z_MAST, color='gray', linestyle=':', lw=1, label=f'$z_{{mast}}$ = {Z_MAST} m')
    ax2.fill_between(log_ff['t'],
                     log_ff['z_platform'] - R_LAND,
                     log_ff['z_platform'] + R_LAND,
                     alpha=0.15, color='green', label=f'$\\pm r_{{land}}$ band')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Altitude (m)')
    ax2.set_title('Vertical Tracking: Drone vs Platform')
    ax2.legend(fontsize=8)

    # ── Panel 3: Relative vertical velocity ──────────────────────────────────
    ax3 = fig.add_subplot(2, 2, 3)
    ax3.plot(log_ff['t'],   log_ff['dz_rel'],   'r-',  lw=1.5, label='LQR+FF')
    ax3.plot(log_noff['t'], log_noff['dz_rel'], 'b--', lw=1.5, label='LQR only')
    ax3.axhline( V_LAND_MAX, color='k', linestyle=':', lw=1.5,
                 label=f'$v_{{land,max}}$ = ±{V_LAND_MAX} m/s')
    ax3.axhline(-V_LAND_MAX, color='k', linestyle=':', lw=1.5)
    ax3.fill_between(log_ff['t'], -V_LAND_MAX, V_LAND_MAX, alpha=0.10, color='green',
                     label='Landing velocity band')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('$\\Delta\\dot{z}$ (m/s)')
    ax3.set_title('Relative Vertical Velocity')
    ax3.legend(fontsize=8)

    # ── Panel 4: Horizontal position error ───────────────────────────────────
    ax4 = fig.add_subplot(2, 2, 4)
    err_xy_ff   = np.sqrt((log_ff['pos'][:, 0] - X_TARGET)**2
                         + (log_ff['pos'][:, 1] - Y_TARGET)**2)
    err_xy_noff = np.sqrt((log_noff['pos'][:, 0] - X_TARGET)**2
                         + (log_noff['pos'][:, 1] - Y_TARGET)**2)
    ax4.plot(log_ff['t'],   err_xy_ff,   'r-',  lw=1.5, label='LQR+FF')
    ax4.plot(log_noff['t'], err_xy_noff, 'b--', lw=1.5, label='LQR only')
    ax4.axhline(R_LAND, color='k', linestyle=':', lw=1.5,
                label=f'$r_{{land}}$ = {R_LAND} m')
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Horizontal position error (m)')
    ax4.set_title('XY Positioning Error')
    ax4.legend(fontsize=8)

    plt.suptitle('S079 Offshore Wind Farm Installation — Approach Analysis', fontsize=14)
    plt.tight_layout()
    plt.savefig('outputs/04_industrial_agriculture/s079_offshore_wind/approach_analysis.png',
                dpi=150, bbox_inches='tight')
    plt.show()


def plot_monte_carlo(p_ff, misses_ff, p_noff, misses_noff):
    """Bar chart and histogram for Monte Carlo results."""
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))

    # Success rate bar chart
    ax1 = axes[0]
    bars = ax1.bar(['LQR+FF', 'LQR only'], [p_ff, p_noff],
                   color=['tomato', 'steelblue'], alpha=0.85, width=0.4)
    for bar, val in zip(bars, [p_ff, p_noff]):
        ax1.text(bar.get_x() + bar.get_width() / 2, val + 0.5,
                 f'{val:.1f}%', ha='center', va='bottom', fontsize=12, fontweight='bold')
    ax1.axhline(95, color='k', linestyle='--', lw=1, label='95 % target')
    ax1.set_ylim(0, 105)
    ax1.set_ylabel('Landing success rate (%)')
    ax1.set_title(f'Monte Carlo Landing Success Rate\n($N$ = {N_TRIALS} trials)')
    ax1.legend(fontsize=9)

    # Window-miss histogram
    ax2 = axes[1]
    bins = np.arange(0, max(misses_ff.max(), misses_noff.max()) + 2) - 0.5
    ax2.hist(misses_ff,   bins=bins, alpha=0.65, color='tomato',
             label='LQR+FF',   density=True)
    ax2.hist(misses_noff, bins=bins, alpha=0.65, color='steelblue',
             label='LQR only', density=True)
    ax2.set_xlabel('Window miss count per trial')
    ax2.set_ylabel('Probability density')
    ax2.set_title('Distribution of Landing Window Misses')
    ax2.legend(fontsize=9)

    plt.tight_layout()
    plt.savefig('outputs/04_industrial_agriculture/s079_offshore_wind/monte_carlo.png',
                dpi=150, bbox_inches='tight')
    plt.show()


def animate_landing(log):
    """Animate drone descending to heaving platform (side-on XZ view)."""
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

    pos   = log['pos']
    zp    = log['z_platform']
    times = log['t']
    n     = len(times)
    step  = max(1, n // 250)

    fig = plt.figure(figsize=(9, 6))
    ax  = fig.add_subplot(111, projection='3d')
    ax.set_xlim(-2, 14);  ax.set_ylim(-4, 4);  ax.set_zlim(0, 32)
    ax.set_xlabel('x (m)');  ax.set_ylabel('y (m)');  ax.set_zlabel('z (m)')
    ax.set_title('S079 Offshore Wind Installation — Landing Approach')

    # Static mast base
    mast_z = np.linspace(0, Z_MAST - A_SWELL - 0.5, 20)
    ax.plot(np.full(20, X_TARGET), np.zeros(20), mast_z, 'gray', lw=5)

    # Dynamic platform top (heaving segment)
    plat_line, = ax.plot([], [], [], color='dimgray', lw=6)
    # Drone dot and trail
    drone_dot,  = ax.plot([], [], [], 'ro', ms=8)
    trail_line, = ax.plot([], [], [], 'r-', lw=1.2, alpha=0.6)
    # Landing target marker
    target_dot, = ax.plot([], [], [], 'g^', ms=10)
    time_ann    = ax.text2D(0.02, 0.95, '', transform=ax.transAxes, fontsize=10)

    trail_x, trail_y, trail_z = [], [], []
    TRAIL = 80

    def init():
        plat_line.set_data([], []);   plat_line.set_3d_properties([])
        drone_dot.set_data([], []);   drone_dot.set_3d_properties([])
        trail_line.set_data([], []); trail_line.set_3d_properties([])
        target_dot.set_data([], []); target_dot.set_3d_properties([])
        return plat_line, drone_dot, trail_line, target_dot, time_ann

    def update(fi):
        k = fi * step
        t   = times[k]
        p   = pos[k]
        zpk = zp[k]

        # Heaving platform cap (small horizontal segment)
        xs = [X_TARGET - 0.5, X_TARGET + 0.5]
        plat_line.set_data(xs, [0, 0])
        plat_line.set_3d_properties([zpk, zpk])

        # Drone trail
        trail_x.append(p[0]); trail_y.append(p[1]); trail_z.append(p[2])
        if len(trail_x) > TRAIL:
            trail_x.pop(0); trail_y.pop(0); trail_z.pop(0)
        trail_line.set_data(trail_x, trail_y)
        trail_line.set_3d_properties(trail_z)

        drone_dot.set_data([p[0]], [p[1]])
        drone_dot.set_3d_properties([p[2]])

        # Target on platform top
        target_dot.set_data([X_TARGET], [Y_TARGET])
        target_dot.set_3d_properties([zpk])

        dz = abs(log['dz_rel'][k])
        time_ann.set_text(
            f't = {t:.1f} s\n'
            f'$|\\Delta\\dot{{z}}|$ = {dz:.3f} m/s\n'
            f'$z_{{err}}$ = {abs(p[2] - zpk):.3f} m'
        )
        return plat_line, drone_dot, trail_line, target_dot, time_ann

    n_frames = n // step
    anim = FuncAnimation(fig, update, frames=n_frames, init_func=init,
                         blit=False, interval=50)
    anim.save('outputs/04_industrial_agriculture/s079_offshore_wind/landing_approach.gif',
              writer='pillow', fps=20)
    plt.close()


if __name__ == '__main__':
    import os
    os.makedirs('outputs/04_industrial_agriculture/s079_offshore_wind', exist_ok=True)

    print('Running S079 Offshore Wind Farm Installation Assistance ...')

    log_ff,   outcome_ff   = run_simulation(use_feedforward=True,  seed=0)
    log_noff, outcome_noff = run_simulation(use_feedforward=False, seed=0)

    print(f"LQR+FF  : landed={outcome_ff['landed']}  "
          f"window_miss={outcome_ff['window_miss']}  "
          f"pos_err={outcome_ff['final_pos_err']:.3f} m")
    print(f"LQR only: landed={outcome_noff['landed']}  "
          f"window_miss={outcome_noff['window_miss']}  "
          f"pos_err={outcome_noff['final_pos_err']:.3f} m")

    plot_results(log_ff, log_noff, outcome_ff, outcome_noff)
    animate_landing(log_ff)

    print(f'Running Monte Carlo ({N_TRIALS} trials each) ...')
    p_ff,   misses_ff   = run_monte_carlo(use_feedforward=True)
    p_noff, misses_noff = run_monte_carlo(use_feedforward=False)
    print(f"LQR+FF   success rate: {p_ff:.1f}%  mean_misses={misses_ff.mean():.2f}")
    print(f"LQR only success rate: {p_noff:.1f}%  mean_misses={misses_noff.mean():.2f}")

    plot_monte_carlo(p_ff, misses_ff, p_noff, misses_noff)
    print('Done. Outputs saved to outputs/04_industrial_agriculture/s079_offshore_wind/')
```

---

## Key Parameters

| Parameter | Symbol | Value |
|-----------|--------|-------|
| Swell amplitude | $A_{swell}$ | 0.8 m |
| Swell period | $T_{swell}$ | 6 s |
| Nominal mast height | $z_{mast}$ | 20 m |
| Platform horizontal position | $(x_p, y_p)$ | (10, 0) m |
| Drone start position | $(x_0, y_0, z_0)$ | (0, 0, 30) m |
| Landing position tolerance | $r_{land}$ | 0.15 m |
| Landing velocity tolerance | $v_{land,max}$ | 0.1 m/s |
| Landing window duration | $T_{window}$ | 3 s |
| Feedforward look-ahead | $\tau$ | 0.5 s |
| Wind gust std dev | $\sigma_w$ | 0.05 m/s |
| Drone mass | $m$ | 1.0 kg |
| Roll / pitch inertia | $I_{xx}, I_{yy}$ | 0.01 kg·m² |
| LQR position weights | $Q_p$ | diag(20, 20, 50) |
| LQR velocity weights | $Q_v$ | diag(3, 3, 8) |
| LQR attitude weights | $Q_{\phi\theta}$ | diag(5, 5) |
| LQR input weight | $R$ | diag(0.1, 1, 1) |
| Monte Carlo trials | $N_{trials}$ | 200 |
| Simulation timestep | $\Delta t$ | 0.02 s |
| Mission horizon | $T_{max}$ | 40 s |

---

## Expected Output

- **3D approach trajectory**: `mpl_toolkits.mplot3d` plot showing the full drone path from supply
  station to platform mast; grey vertical mast column at $(10, 0)$ m; scattered green points
  tracing the heaving platform target over the mission; LQR+FF (red) and LQR only (blue dashed)
  paths compared; start point marked.
- **Vertical tracking panel**: altitude time series for drone (both strategies) and platform
  $z_p(t)$ on the same axes; ±$r_{land}$ shaded band around the platform line; shows phase lag
  of the no-feedforward controller and tight synchronisation of LQR+FF near landing.
- **Relative vertical velocity panel**: $\Delta\dot{z}(t)$ time series for both controllers;
  ±$v_{land,max}$ band shaded green; LQR+FF expected to spend more total time inside the band,
  producing more and longer landing windows.
- **XY position error panel**: horizontal distance from target $(10, 0)$ over time; $r_{land}$
  reference line; demonstrates gust rejection performance.
- **Monte Carlo bar chart**: landing success rate (%) for LQR+FF and LQR-only over 200 trials;
  95 % target line; annotated with exact percentages.
- **Window-miss histogram**: empirical distribution of per-trial window-miss counts for both
  controllers; LQR+FF expected to cluster near zero misses.
- **Landing approach animation (GIF)**: 3D side-on view; heaving platform cap moves vertically
  in real time; drone dot descends with red trail; green triangle marks the landing target;
  live annotations show $|\Delta\dot{z}|$ and $z_{err}$; saved at 20 fps.

**Expected metric targets** (LQR+FF, deterministic seed 0):

| Metric | Target |
|--------|--------|
| Landing success (single run) | YES |
| Window miss count | 0 |
| Final position error | $< 0.15$ m |
| Peak $\|\Delta\dot{z}\|$ | $< 0.5$ m/s |
| MC success rate (LQR+FF) | $\geq 90\%$ |
| MC success rate (LQR only) | $\leq 60\%$ |

---

## Extensions

1. **Wave spectrum generalisation**: replace the single-frequency swell with a JONSWAP sea spectrum
   (sum of $N$ sinusoids with random phases and a Pierson-Moskowitz amplitude envelope); re-derive
   the feedforward as a finite-impulse-response filter fitted to the spectrum peak; compare
   landing success rate under irregular versus regular swell.
2. **EKF swell estimator**: remove the assumption of known $A_{swell}$ and $T_{swell}$; implement
   an extended Kalman filter that estimates both parameters online from the onboard altimeter
   measuring the drone-to-platform gap; evaluate convergence time and the degradation in success
   rate during the estimation transient.
3. **Multi-drone coordinated delivery**: assign three drones to deliver components to three
   separate masts simultaneously (different heave phases); use a central scheduler to stagger
   approach times such that radio interference zones and rotor downwash corridors never overlap;
   minimise total delivery time.
4. **Tether-assisted landing**: model an elastic tether of stiffness $k_t = 50$ N/m and natural
   length $L_0 = 1$ m between drone and platform hook; modify the LQR cost to include tether
   tension as a soft constraint; simulate the final capture phase and measure peak tether force.
5. **Model Predictive Control (MPC)**: replace the LQR+feedforward with a receding-horizon MPC
   that directly optimises over a 2 s prediction window; enforce hard constraints on position
   error and velocity at each horizon step; compare computational load, success rate, and
   robustness to swell period uncertainty versus the LQR approach.

---

## Related Scenarios

- Prerequisites: [S062 Wind Turbine Blade Inspection](S062_wind_turbine.md) (offshore turbine
  proximity operations), [S066 Cooperative Crane Lift](S066_cooperative_crane.md) (precision
  delivery to a constrained target)
- Algorithmic cross-reference: [S058 Typhoon Eye Probing](../03_environmental_sar/S058_typhoon.md)
  (LQR under extreme disturbance, feedforward architecture),
  [S023 Moving Landing Pad](../02_logistics_delivery/S023_moving_landing_pad.md) (landing on a
  moving target, velocity matching)
- Follow-ups: [S080 Autonomous Ship Deck Landing](S080_ship_deck_landing.md) (6-DOF moving
  platform, full attitude coupling)
