# S083 Light Show Single-Drone Test

**Domain**: Special Ops & Entertainment | **Difficulty**: ⭐ | **Status**: `[ ]` Not Started
**Algorithm**: Cubic Spline Interpolation + PID + Feedforward | **Dimension**: 3D

---

## Problem Definition

**Setup**: A single drone executes a pre-scripted light show path — a **figure-8 curve at
$z = 50$ m** — over a duration of $T_{total} = 60$ s. The path is parameterised by 200 waypoints
sampled uniformly in time, interpolated into a smooth reference trajectory via **cubic spline**.
A **PID position controller** drives the drone along the trajectory; an optional **feedforward
velocity term** augments the PID command by anticipating the reference motion. A stochastic wind
disturbance $\mathbf{w}(t) \sim \mathcal{N}(\mathbf{0},\, \sigma_w^2 \mathbf{I})$ acts on the
drone throughout the mission. Point-mass dynamics are integrated with
`scipy.integrate.odeint` (no PyBullet).

**Roles**:

- **Drone**: single UAV commanded by a 3D position controller at every integration step.
- **Reference path**: figure-8 at constant altitude $z = 50$ m, parameterised as a Lissajous
  curve in $(x, y)$ and held fixed in $z$.
- **Wind disturbance**: independent Gaussian noise on each axis, modelling atmospheric turbulence
  at show altitude.

**Objective**: For each control variant, simulate the full 60 s mission and report:

1. **RMS tracking error** $\varepsilon_{RMS}$ (m) — must be $\leq 0.1$ m to pass the light show
   spec.
2. **Maximum instantaneous deviation** $\varepsilon_{max}$ (m) — worst-case gap from the scripted
   path.
3. **Path completion** — fraction of time steps where the drone remains within 0.5 m of the
   reference.

**Comparison configurations** (control variants):

1. **PID-only** — $K_p = 2.0$, $K_i = 0.1$, $K_d = 0.8$; no feedforward term.
2. **PID + feedforward** — same PID gains; feedforward acceleration $\mathbf{a}_{ff} = \dot{\mathbf{v}}_{ref}$ derived from the spline velocity derivative.

---

## Mathematical Model

### Figure-8 Reference Path

The scripted path is a **Lissajous figure-8** in the horizontal plane at altitude
$z_{show} = 50$ m. With a normalised time parameter $\tau \in [0, 1]$ (where $\tau = t / T_{total}$),
the reference position is:

$$\mathbf{p}_{ref}(\tau) = \begin{pmatrix} A_x \sin(2\pi \tau) \\ A_y \sin(4\pi \tau) \\ z_{show} \end{pmatrix}$$

where $A_x = A_y = 20$ m are the semi-amplitudes of the figure-8 lobes. At $N_{wp} = 200$
waypoints uniformly spaced in $\tau$, the reference positions $\{\mathbf{p}_{ref,k}\}_{k=0}^{199}$
are pre-computed and stored.

### Cubic Spline Interpolation

A **cubic spline** $\mathbf{s}(t)$ is fitted through the $N_{wp}$ waypoints using
`scipy.interpolate.CubicSpline`. Each axis is splined independently with the *not-a-knot* end
condition. Given query time $t$:

$$\mathbf{p}_{ref}(t) = \mathbf{s}(t), \qquad \mathbf{v}_{ref}(t) = \mathbf{s}'(t), \qquad \dot{\mathbf{v}}_{ref}(t) = \mathbf{s}''(t)$$

The spline ensures $C^2$ continuity everywhere along the path, eliminating kinks that would
demand infinite acceleration.

### PID Position Controller

The 3D position error and its time derivative are:

$$\mathbf{e}(t) = \mathbf{p}_{ref}(t) - \mathbf{p}(t), \qquad \dot{\mathbf{e}}(t) = \mathbf{v}_{ref}(t) - \mathbf{v}(t)$$

The PID commanded acceleration is:

$$\mathbf{a}_{PID}(t) = K_p\, \mathbf{e}(t) + K_i \int_0^t \mathbf{e}(\tau)\, d\tau + K_d\, \dot{\mathbf{e}}(t)$$

where $K_p$, $K_i$, $K_d$ are scalar gains applied identically to each axis.

### Feedforward Term

When feedforward is enabled, the reference acceleration (second derivative of the spline) is
added directly to the command:

$$\mathbf{a}_{ff}(t) = \dot{\mathbf{v}}_{ref}(t) = \mathbf{s}''(t)$$

The total acceleration command becomes:

$$\mathbf{a}_{cmd}(t) = \mathbf{a}_{PID}(t) + \mathbf{a}_{ff}(t)$$

The feedforward term cancels the nominal inertial load of the curved reference, leaving the PID
to correct only the residual error caused by wind disturbance.

### Point-Mass Dynamics

The drone is modelled as a point mass with no attitude dynamics. The state
$\mathbf{x} = [\mathbf{p}^\top, \mathbf{v}^\top]^\top \in \mathbb{R}^6$ evolves as:

$$\dot{\mathbf{p}}(t) = \mathbf{v}(t)$$
$$\dot{\mathbf{v}}(t) = \mathbf{a}_{cmd}(t) + \mathbf{w}(t)$$

where the wind disturbance is sampled independently on each axis and at each time step:

$$\mathbf{w}(t) \sim \mathcal{N}(\mathbf{0},\, \sigma_w^2 \mathbf{I}_3), \qquad \sigma_w = 0.5 \text{ m/s}$$

Integration is performed by `scipy.integrate.odeint` with a fixed timestep $\Delta t = 0.05$ s
(20 Hz), re-sampling the wind term at each step.

### Tracking Error Metrics

Let $N_{steps}$ be the total number of integration steps and $\mathbf{e}_k = \mathbf{p}_{ref}(t_k) - \mathbf{p}(t_k)$ the position error at step $k$. The metrics are:

$$\varepsilon_{RMS} = \sqrt{\frac{1}{N_{steps}} \sum_{k=1}^{N_{steps}} \|\mathbf{e}_k\|^2}$$

$$\varepsilon_{max} = \max_{k}\; \|\mathbf{e}_k\|$$

$$\text{Path completion} = \frac{\bigl|\{k : \|\mathbf{e}_k\| \leq 0.5\}\bigr|}{N_{steps}} \times 100\%$$

---

## Implementation

```python
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import CubicSpline
from scipy.integrate import odeint

# ── Domain constants ──────────────────────────────────────────────────────────
LED_SHOW_HEIGHT = 50.0    # m — light show altitude (domain constant)
FORMATION_DIST  = 2.0     # m — inter-drone formation spacing (domain constant)
AMP_X           = 20.0    # m — figure-8 semi-amplitude in x
AMP_Y           = 20.0    # m — figure-8 semi-amplitude in y
T_TOTAL         = 60.0    # s — total show duration
N_WAYPOINTS     = 200     # number of scripted waypoints
DT              = 0.05    # s — integration timestep (20 Hz)
SIGMA_W         = 0.5     # m/s — wind disturbance std dev (per axis)
PASS_THRESHOLD  = 0.5     # m — max error for "on-path" classification
RMS_SPEC        = 0.1     # m — light show RMS tracking spec

# ── PID gains ─────────────────────────────────────────────────────────────────
KP = 2.0
KI = 0.1
KD = 0.8

# ── Reference path generation ─────────────────────────────────────────────────
def make_reference(n_wp=N_WAYPOINTS, t_total=T_TOTAL):
    """Build the figure-8 waypoints and fit a cubic spline to each axis."""
    tau   = np.linspace(0.0, 1.0, n_wp, endpoint=False)
    t_wp  = tau * t_total
    px_wp = AMP_X * np.sin(2 * np.pi * tau)
    py_wp = AMP_Y * np.sin(4 * np.pi * tau)
    pz_wp = np.full(n_wp, LED_SHOW_HEIGHT)

    # Fit cubic splines (not-a-knot, periodic-compatible by manual closure)
    # Close the loop: append first waypoint at t=T_TOTAL for periodic boundary
    t_closed  = np.append(t_wp, t_total)
    px_closed = np.append(px_wp, px_wp[0])
    py_closed = np.append(py_wp, py_wp[0])
    pz_closed = np.append(pz_wp, pz_wp[0])

    cs_x = CubicSpline(t_closed, px_closed)
    cs_y = CubicSpline(t_closed, py_closed)
    cs_z = CubicSpline(t_closed, pz_closed)

    return cs_x, cs_y, cs_z, t_wp, np.column_stack([px_wp, py_wp, pz_wp])

def ref_state(t, cs_x, cs_y, cs_z):
    """Return (p_ref, v_ref, a_ref) at time t from the spline."""
    p = np.array([cs_x(t),    cs_y(t),    cs_z(t)])
    v = np.array([cs_x(t, 1), cs_y(t, 1), cs_z(t, 1)])
    a = np.array([cs_x(t, 2), cs_y(t, 2), cs_z(t, 2)])
    return p, v, a

# ── ODE right-hand side ───────────────────────────────────────────────────────
def make_ode(cs_x, cs_y, cs_z, feedforward, rng):
    """Factory: returns an odeint-compatible derivative function."""
    integral_e = np.zeros(3)  # mutable accumulator (closure state)

    def _deriv(state, t):
        p = state[:3]
        v = state[3:]

        p_ref, v_ref, a_ref = ref_state(t, cs_x, cs_y, cs_z)
        e     = p_ref - p
        e_dot = v_ref - v

        integral_e[:] += e * DT   # Euler integration of integral term

        a_pid = KP * e + KI * integral_e + KD * e_dot
        a_ff  = a_ref if feedforward else np.zeros(3)
        wind  = rng.normal(0.0, SIGMA_W, size=3)

        a_cmd = a_pid + a_ff + wind

        dpdt = v
        dvdt = a_cmd
        return np.concatenate([dpdt, dvdt])

    return _deriv

def run_simulation(feedforward=False, seed=42):
    """
    Simulate the full light show mission.
    Returns time array, drone trajectory, reference trajectory, and metrics.
    """
    rng       = np.random.default_rng(seed)
    cs_x, cs_y, cs_z, t_wp, wp_pos = make_reference()

    # Initial state: start at first waypoint, zero velocity
    p0    = np.array([wp_pos[0, 0], wp_pos[0, 1], LED_SHOW_HEIGHT])
    state0 = np.concatenate([p0, np.zeros(3)])

    t_span = np.arange(0.0, T_TOTAL, DT)
    deriv  = make_ode(cs_x, cs_y, cs_z, feedforward, rng)
    sol    = odeint(deriv, state0, t_span)

    drone_pos = sol[:, :3]

    # Reference positions at each timestep
    ref_pos = np.column_stack([
        cs_x(t_span), cs_y(t_span), cs_z(t_span)
    ])

    # ── Compute metrics ───────────────────────────────────────────────────────
    errors      = np.linalg.norm(drone_pos - ref_pos, axis=1)
    rms_error   = float(np.sqrt(np.mean(errors**2)))
    max_error   = float(np.max(errors))
    completion  = float(np.mean(errors <= PASS_THRESHOLD)) * 100.0

    label = "PID + FF" if feedforward else "PID-only"
    print(f"[{label}]")
    print(f"  RMS error   : {rms_error*100:.2f} cm  "
          f"({'PASS' if rms_error <= RMS_SPEC else 'FAIL'} spec={RMS_SPEC*100:.0f} cm)")
    print(f"  Max error   : {max_error*100:.2f} cm")
    print(f"  Path compl. : {completion:.1f}%")

    return {
        "label":       label,
        "t":           t_span,
        "drone_pos":   drone_pos,
        "ref_pos":     ref_pos,
        "errors":      errors,
        "rms_error":   rms_error,
        "max_error":   max_error,
        "completion":  completion,
        "waypoints":   wp_pos,
    }

# ── Plotting helpers ──────────────────────────────────────────────────────────
def plot_3d_trajectory(results_list, save_path=None):
    """3D figure-8 path: reference (green) vs PID-only (red) vs PID+FF (blue)."""
    fig = plt.figure(figsize=(10, 7))
    ax  = fig.add_subplot(111, projection='3d')

    ref = results_list[0]["ref_pos"]
    ax.plot(ref[:, 0], ref[:, 1], ref[:, 2],
            color='green', lw=2, label='Reference', zorder=5)

    colors = ['red', 'blue']
    for res, col in zip(results_list, colors):
        ax.plot(res["drone_pos"][:, 0],
                res["drone_pos"][:, 1],
                res["drone_pos"][:, 2],
                color=col, lw=1, alpha=0.8, label=res["label"])

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('S083 — Figure-8 Light Show Path (3D)')
    ax.legend()
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
    plt.show()

def plot_error_timeseries(results_list, save_path=None):
    """Tracking error ||e(t)|| over time for both control variants."""
    fig, ax = plt.subplots(figsize=(10, 4))
    colors  = ['red', 'blue']
    for res, col in zip(results_list, colors):
        ax.plot(res["t"], res["errors"] * 100,
                color=col, lw=1.2, alpha=0.85, label=res["label"])

    ax.axhline(RMS_SPEC * 100, color='black', ls='--', lw=1.5,
               label=f'RMS spec ({RMS_SPEC*100:.0f} cm)')
    ax.axhline(PASS_THRESHOLD * 100, color='grey', ls=':', lw=1.2,
               label=f'Path threshold ({PASS_THRESHOLD*100:.0f} cm)')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Position error (cm)')
    ax.set_title('S083 — Tracking Error vs Time')
    ax.legend()
    ax.grid(True, alpha=0.3)
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
    plt.show()

def animate_drone(result, save_path=None):
    """
    2D top-down animation of the figure-8 with the drone moving along the path.
    Reference shown in green, drone trail in red (PID-only) or blue (PID+FF).
    Saved as an MP4 or GIF to save_path if provided.
    """
    fig, ax = plt.subplots(figsize=(7, 7))
    ref  = result["ref_pos"]
    traj = result["drone_pos"]
    col  = 'red' if 'FF' not in result["label"] else 'blue'

    ax.plot(ref[:, 0], ref[:, 1], color='green', lw=1.5,
            alpha=0.5, label='Reference')
    trail_line, = ax.plot([], [], color=col, lw=1.0, alpha=0.6, label=result["label"])
    drone_dot,  = ax.plot([], [], 'o', color=col, ms=8, zorder=5)
    ref_dot,    = ax.plot([], [], 'o', color='green', ms=6, zorder=4)
    time_text   = ax.text(0.02, 0.95, '', transform=ax.transAxes)

    ax.set_xlim(-AMP_X * 1.3, AMP_X * 1.3)
    ax.set_ylim(-AMP_Y * 1.3, AMP_Y * 1.3)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title(f'S083 — {result["label"]} (top-down view)')
    ax.set_aspect('equal')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)

    stride = 5   # animate every 5th timestep for smooth 4 fps at DT=0.05 s

    def _init():
        trail_line.set_data([], [])
        drone_dot.set_data([], [])
        ref_dot.set_data([], [])
        time_text.set_text('')
        return trail_line, drone_dot, ref_dot, time_text

    def _update(frame):
        idx = frame * stride
        trail_line.set_data(traj[:idx, 0], traj[:idx, 1])
        drone_dot.set_data([traj[idx, 0]], [traj[idx, 1]])
        ref_dot.set_data([ref[idx, 0]], [ref[idx, 1]])
        time_text.set_text(f't = {result["t"][idx]:.1f} s')
        return trail_line, drone_dot, ref_dot, time_text

    n_frames = len(result["t"]) // stride
    anim = animation.FuncAnimation(
        fig, _update, frames=n_frames, init_func=_init,
        interval=50, blit=True
    )
    if save_path:
        anim.save(save_path, fps=20, dpi=120)
    plt.show()
    return anim

# ── Main entry point ──────────────────────────────────────────────────────────
if __name__ == '__main__':
    import os
    OUT_DIR = 'outputs/05_special_entertainment/s083_light_show_single'
    os.makedirs(OUT_DIR, exist_ok=True)

    res_pid    = run_simulation(feedforward=False, seed=42)
    res_pidff  = run_simulation(feedforward=True,  seed=42)
    results    = [res_pid, res_pidff]

    plot_3d_trajectory(results,
        save_path=f'{OUT_DIR}/s083_3d_trajectory.png')
    plot_error_timeseries(results,
        save_path=f'{OUT_DIR}/s083_error_timeseries.png')
    animate_drone(res_pidff,
        save_path=f'{OUT_DIR}/s083_animation.gif')
```

---

## Key Parameters

| Parameter | Symbol | Value |
|-----------|--------|-------|
| Light show altitude | $z_{show}$ | 50 m |
| Formation distance (domain) | $d_{form}$ | 2.0 m |
| Figure-8 semi-amplitude | $A_x = A_y$ | 20 m |
| Mission duration | $T_{total}$ | 60 s |
| Number of scripted waypoints | $N_{wp}$ | 200 |
| Simulation timestep | $\Delta t$ | 0.05 s (20 Hz) |
| Wind disturbance std dev (per axis) | $\sigma_w$ | 0.5 m/s |
| Proportional gain | $K_p$ | 2.0 |
| Integral gain | $K_i$ | 0.1 |
| Derivative gain | $K_d$ | 0.8 |
| RMS tracking spec | $\varepsilon_{RMS}^*$ | 0.1 m |
| Path completion threshold | — | 0.5 m |
| Spline end condition | — | not-a-knot (periodic closure) |
| Dynamics model | — | point-mass, `scipy.integrate.odeint` |

---

## Expected Output

- **3D trajectory plot** (`Axes3D`): the full figure-8 reference path in green ($z = 50$ m);
  PID-only drone trajectory in red; PID+FF trajectory in blue; both plotted over the 60 s
  mission so loop repetition is visible; legend, axis labels, and a brief title including RMS
  values for each variant.
- **Tracking error time series**: a single panel with $\|\mathbf{e}(t)\|$ in centimetres on the
  $y$-axis and time (s) on the $x$-axis; red curve for PID-only, blue for PID+FF; horizontal
  dashed black line at the RMS spec (10 cm); horizontal dotted grey line at the path completion
  threshold (50 cm); both curves shown simultaneously to highlight the transient settling and
  steady-state wind rejection of each variant.
- **Top-down 2D animation** (`FuncAnimation`, saved as GIF): drone dot (coloured) moving along
  the figure-8 above the static green reference; a trailing path of the last 5 seconds shown as a
  fading line; current time displayed in the upper-left corner; 20 fps playback.
- **Console metrics table** (printed): for each variant, RMS error (cm), max error (cm), path
  completion (%), and PASS/FAIL against the 10 cm RMS spec.

---

## Extensions

1. **Wind gust model**: replace white Gaussian noise with a coloured-noise Dryden turbulence
   model (low-pass filtered white noise); compare RMS error degradation as gust bandwidth
   increases from 0.1 Hz to 2 Hz, matching real atmospheric spectra at 50 m altitude.
2. **Gain scheduling**: tune separate PID gains for the high-curvature lobes of the figure-8
   (where $\|\dot{\mathbf{v}}_{ref}\|$ is largest) versus the straight crossing segment; verify
   that scheduled gains reduce peak error relative to fixed gains.
3. **Actuator saturation**: add a thrust-limit $\|\mathbf{a}_{cmd}\|_{max} = 15$ m/s² to the
   simulation; observe how saturation during tight turns increases tracking error and propose an
   online trajectory re-timing strategy that slows the path near high-curvature sections.
4. **Multi-loop attitude controller**: replace the point-mass model with a rigid-body quadrotor
   (roll/pitch/yaw + position); implement an inner attitude loop and outer position loop;
   compare tracking bandwidth limitations against the simpler point-mass model.
5. **Synchronised swarm preview**: extend to $K = 5$ drones each following phase-shifted copies
   of the same figure-8, creating a symmetrical formation pattern; assess formation-keeping error
   as a function of wind correlation between drones (see S085).
6. **Path re-planning on large deviation**: if $\|\mathbf{e}(t)\| > 1.0$ m, trigger an emergency
   re-spline from the current drone position back onto the scripted path within the next 2 s;
   log recovery events and measure the fraction of show time affected.

---

## Related Scenarios

- Follow-ups: [S085 Multi-Drone Formation Light Show](S085_light_show_formation.md) (synchronised swarm with the same figure-8 script), [S087 Dynamic Shape Morphing](S087_shape_morphing.md) (formation transitions mid-show), [S098 Grand Finale Burst](S098_grand_finale_burst.md) (high-density simultaneous launch)
- Algorithmic cross-reference: [S063 Precision Spraying](../../04_industrial_agriculture/S063_precision_spraying.md) (PID position hold under wind disturbance), [S048 Lawnmower Coverage](../../03_environmental_sar/S048_lawnmower.md) (pre-scripted path following without feedback)
