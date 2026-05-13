# S094 Counter-Drone Intercept

**Domain**: Special Ops & Entertainment | **Difficulty**: ⭐⭐⭐⭐ | **Status**: `[ ]` Not Started
**Algorithm**: 3D Proportional Navigation Guidance | **Dimension**: 3D

---

## Problem Definition

**Setup**: A $50 \times 50 \times 20$ m restricted airspace contains a rogue small drone (target)
that must be neutralised. A single interceptor drone is launched from a fixed ground station. The
target manoeuvres evasively and switches between two behavioural modes at random intervals: a
**constant-velocity (CV)** mode (straight-line flight at fixed heading) and a **bang-bang evasion**
mode (maximum lateral acceleration in the direction that maximises the line-of-sight angular
rate). The interceptor applies **Proportional Navigation Guidance (PNG)** as its primary law,
with an optional switch to **lead pursuit** when closing speed drops below a threshold. A
**Monte Carlo study** over 200 randomised trials evaluates the success rate, mean capture time,
and miss distance distribution across three guidance laws.

**Roles**:

- **Interceptor** ($v_I = 10$ m/s, $a_{I,max} = 15$ m/s²): uses PNG or an alternative guidance
  law; starts from a random point on the airspace boundary.
- **Target** ($v_T = 6$ m/s, $a_{T,max} = 8$ m/s²): alternates between CV flight and bang-bang
  evasion; heading and switch timing are randomised per trial.

**Objective**: Minimise the time to satisfy the capture condition
$\|\mathbf{p}_I - \mathbf{p}_T\| \leq r_{capture} = 0.5$ m, within a time budget of
$T_{max} = 30$ s. The three guidance laws compared are:

1. **PNG** — Proportional Navigation with navigation constant $N = 4$.
2. **Pure pursuit** — interceptor always points directly at the current target position.
3. **Lead pursuit** — interceptor aims at the predicted future target position using a
   constant-velocity extrapolation over the estimated time-to-go $t_{go}$.

---

## Mathematical Model

### Coordinate System and State

All quantities are expressed in an inertial North-East-Down (NED) frame. The interceptor state
is $\mathbf{x}_I = [\mathbf{p}_I^\top, \mathbf{v}_I^\top]^\top \in \mathbb{R}^6$ and the
target state is $\mathbf{x}_T = [\mathbf{p}_T^\top, \mathbf{v}_T^\top]^\top \in \mathbb{R}^6$.

### Line-of-Sight Geometry (3D)

The relative position vector from interceptor to target and its magnitude are:

$$\mathbf{r}_{IT} = \mathbf{p}_T - \mathbf{p}_I, \qquad R = \|\mathbf{r}_{IT}\|$$

The **LOS azimuth** (horizontal plane) and **LOS elevation** angles are:

$$\lambda_{az} = \text{atan2}(\Delta y,\; \Delta x)$$

$$\lambda_{el} = \text{atan2}\!\left(\Delta z,\; \sqrt{\Delta x^2 + \Delta y^2}\right)$$

where $(\Delta x, \Delta y, \Delta z) = \mathbf{r}_{IT}$.

The **LOS angular rate vector** in 3D is obtained by differentiating the unit LOS vector
$\hat{\mathbf{r}} = \mathbf{r}_{IT} / R$:

$$\dot{\hat{\mathbf{r}}} = \frac{\mathbf{v}_{rel} - (\mathbf{v}_{rel} \cdot \hat{\mathbf{r}})\hat{\mathbf{r}}}{R}$$

where $\mathbf{v}_{rel} = \mathbf{v}_T - \mathbf{v}_I$ is the relative velocity.

### Closing Speed

The scalar closing speed (positive when the range is decreasing) is:

$$V_c = -\dot{R} = -\frac{\mathbf{r}_{IT} \cdot \mathbf{v}_{rel}}{R}$$

### Proportional Navigation Guidance (PNG)

The PNG lateral acceleration command is perpendicular to the current LOS direction and
proportional to the LOS angular rate:

$$\mathbf{a}_{PNG} = N \cdot V_c \cdot \dot{\hat{\mathbf{r}}} \times \hat{\mathbf{r}} \times \hat{\mathbf{r}}$$

Written more compactly using the component of $\dot{\hat{\mathbf{r}}}$ perpendicular to
$\hat{\mathbf{r}}$ (the LOS rotation rate vector $\boldsymbol{\omega}_{LOS} = \hat{\mathbf{r}} \times \dot{\hat{\mathbf{r}}} \times \hat{\mathbf{r}}$
is already perpendicular):

$$\dot{\mathbf{n}} = N \cdot V_c \cdot \boldsymbol{\Omega}_{LOS}$$

where $\boldsymbol{\Omega}_{LOS} = \dot{\hat{\mathbf{r}}} - (\dot{\hat{\mathbf{r}}} \cdot \hat{\mathbf{r}})\hat{\mathbf{r}}$
is the transverse LOS angular rate (3-vector, already perpendicular to $\hat{\mathbf{r}}$),
and $N = 4$ is the **navigation constant**. The full commanded acceleration vector is:

$$\mathbf{a}_{I,cmd}^{PNG} = N \cdot V_c \cdot \boldsymbol{\Omega}_{LOS}$$

This is then clipped to the interceptor's acceleration budget:

$$\mathbf{a}_{I} = \mathbf{a}_{I,cmd} \cdot \min\!\left(1,\; \frac{a_{I,max}}{\|\mathbf{a}_{I,cmd}\|}\right)$$

### Zero-Effort Miss (ZEM)

The Zero-Effort Miss distance predicts how far the interceptor will miss the target if both
vehicles maintain current velocities for the remaining time-to-go $t_{go} = R / V_c$:

$$\mathbf{ZEM} = \mathbf{r}_{IT} + \mathbf{v}_{rel}\, t_{go}$$

$$ZEM = \|\mathbf{ZEM}\|$$

A large ZEM signals that a significant course correction is required; $ZEM \to 0$ as capture
approaches under PNG.

### Pure Pursuit

The interceptor simply steers toward the current target position:

$$\mathbf{a}_{I,cmd}^{PP} = a_{I,max} \cdot \hat{\mathbf{r}}_{IT} - \mathbf{v}_I$$

Practically, the commanded velocity direction is set to $\hat{\mathbf{r}}_{IT}$ and the
resulting acceleration is the first-order feedback correction:

$$\mathbf{a}_{I,cmd}^{PP} = K_{pp}\!\left(v_I \cdot \hat{\mathbf{r}}_{IT} - \mathbf{v}_I\right)$$

with $K_{pp} = 5$ s$^{-1}$.

### Lead Pursuit

The interceptor aims at the predicted target position after estimated time-to-go:

$$t_{go} = \frac{R}{V_c + \varepsilon}, \qquad \varepsilon = 0.01 \text{ m/s (regulariser)}$$

$$\mathbf{p}_{T,lead} = \mathbf{p}_T + \mathbf{v}_T \cdot t_{go}$$

$$\hat{\mathbf{r}}_{lead} = \frac{\mathbf{p}_{T,lead} - \mathbf{p}_I}{\|\mathbf{p}_{T,lead} - \mathbf{p}_I\|}$$

$$\mathbf{a}_{I,cmd}^{LP} = K_{lp}\!\left(v_I \cdot \hat{\mathbf{r}}_{lead} - \mathbf{v}_I\right)$$

with $K_{lp} = 5$ s$^{-1}$.

### Target Evasion Model

The target switches between two modes at Poisson-distributed intervals with mean $\bar{\tau} = 3$ s:

**CV mode**: target flies at constant heading $\psi_T$ and altitude; $\mathbf{a}_T = \mathbf{0}$.

**Bang-bang evasion mode**: target applies maximum lateral acceleration in the direction that
maximises $|\dot{\lambda}_{az}|$. The optimal bang-bang direction is perpendicular to the LOS
and to the current velocity:

$$\hat{\mathbf{e}}_{bang} = \frac{\hat{\mathbf{r}}_{IT} \times \mathbf{v}_T}{\|\hat{\mathbf{r}}_{IT} \times \mathbf{v}_T\|}$$

$$\mathbf{a}_T = a_{T,max} \cdot \hat{\mathbf{e}}_{bang}$$

The sign (left or right bank) is chosen randomly at each mode-switch event, creating an
unpredictable jinking pattern.

### Capture Condition and Metrics

The trial terminates with **success** when:

$$\|\mathbf{p}_I(t) - \mathbf{p}_T(t)\| \leq r_{capture} = 0.5 \text{ m}$$

and with **failure** when $t > T_{max} = 30$ s without capture. Over $N_{MC} = 200$ randomised
Monte Carlo trials per guidance law, the following metrics are computed:

$$\text{Success rate} = \frac{N_{success}}{N_{MC}} \times 100\%$$

$$T_{cap} = \text{first } t \text{ such that } \|\mathbf{p}_I - \mathbf{p}_T\| \leq r_{capture}$$

$$d_{miss} = \min_{t \in [0, T_{max}]} \|\mathbf{p}_I(t) - \mathbf{p}_T(t)\|$$

$$\bar{T}_{cap} = \mathbb{E}[T_{cap} \mid \text{success}], \qquad \bar{d}_{miss} = \mathbb{E}[d_{miss} \mid \text{failure}]$$

---

## Implementation

```python
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
from dataclasses import dataclass, field
from typing import Tuple, List, Optional

# ── Domain constants ──────────────────────────────────────────────────────────
AIRSPACE_X      = 50.0    # m — airspace width  (x)
AIRSPACE_Y      = 50.0    # m — airspace depth  (y)
AIRSPACE_Z      = 20.0    # m — airspace height (z)

V_INTERCEPTOR   = 10.0    # m/s — interceptor cruise speed
V_TARGET        =  6.0    # m/s — target cruise speed
A_I_MAX         = 15.0    # m/s² — interceptor max lateral acceleration
A_T_MAX         =  8.0    # m/s² — target max evasion acceleration

N_PNG           =  4.0    # navigation constant for PNG
K_PP            =  5.0    # s⁻¹ — pure-pursuit gain
K_LP            =  5.0    # s⁻¹ — lead-pursuit gain

R_CAPTURE       =  0.5    # m — capture radius
T_MAX           = 30.0    # s — mission time budget
DT              =  0.02   # s — simulation timestep (50 Hz)
TAU_SWITCH_MEAN =  3.0    # s — mean evasion mode dwell time (Poisson)

N_MC            = 200     # Monte Carlo trials per guidance law
SEED_BASE       = 42


@dataclass
class DroneState:
    pos: np.ndarray   # shape (3,)
    vel: np.ndarray   # shape (3,)


def los_geometry(state_i: DroneState, state_t: DroneState
                 ) -> Tuple[np.ndarray, float, np.ndarray, float]:
    """
    Returns:
        r_vec : relative position T - I  (3,)
        R     : range (scalar)
        r_hat : unit LOS vector          (3,)
        V_c   : closing speed (>0 closing)
    """
    r_vec = state_t.pos - state_i.pos
    R     = np.linalg.norm(r_vec)
    r_hat = r_vec / (R + 1e-9)
    v_rel = state_t.vel - state_i.vel
    V_c   = -float(np.dot(r_vec, v_rel)) / (R + 1e-9)
    return r_vec, R, r_hat, V_c


def png_command(state_i: DroneState, state_t: DroneState) -> np.ndarray:
    """3-D Proportional Navigation acceleration command."""
    r_vec, R, r_hat, V_c = los_geometry(state_i, state_t)
    v_rel = state_t.vel - state_i.vel
    # Transverse LOS angular rate (component of v_rel perp. to r_hat)
    omega_los = v_rel / (R + 1e-9) - np.dot(v_rel, r_hat) / (R + 1e-9) * r_hat
    a_cmd = N_PNG * V_c * omega_los
    # Saturate
    mag = np.linalg.norm(a_cmd)
    if mag > A_I_MAX:
        a_cmd = a_cmd * A_I_MAX / mag
    return a_cmd


def pure_pursuit_command(state_i: DroneState, state_t: DroneState) -> np.ndarray:
    """Pure-pursuit: steer velocity toward current target position."""
    _, _, r_hat, _ = los_geometry(state_i, state_t)
    v_desired = V_INTERCEPTOR * r_hat
    a_cmd     = K_PP * (v_desired - state_i.vel)
    mag = np.linalg.norm(a_cmd)
    if mag > A_I_MAX:
        a_cmd = a_cmd * A_I_MAX / mag
    return a_cmd


def lead_pursuit_command(state_i: DroneState, state_t: DroneState) -> np.ndarray:
    """Lead pursuit: aim at predicted target position after t_go."""
    r_vec, R, r_hat, V_c = los_geometry(state_i, state_t)
    t_go   = R / (V_c + 0.01)
    t_go   = np.clip(t_go, 0.0, T_MAX)
    p_lead = state_t.pos + state_t.vel * t_go
    r_lead = p_lead - state_i.pos
    r_lead_norm = np.linalg.norm(r_lead)
    r_lead_hat  = r_lead / (r_lead_norm + 1e-9)
    v_desired   = V_INTERCEPTOR * r_lead_hat
    a_cmd       = K_LP * (v_desired - state_i.vel)
    mag = np.linalg.norm(a_cmd)
    if mag > A_I_MAX:
        a_cmd = a_cmd * A_I_MAX / mag
    return a_cmd


def zero_effort_miss(state_i: DroneState, state_t: DroneState) -> float:
    """Scalar ZEM distance."""
    r_vec, R, _, V_c = los_geometry(state_i, state_t)
    v_rel = state_t.vel - state_i.vel
    t_go  = R / (V_c + 0.01)
    zem_vec = r_vec + v_rel * t_go
    return float(np.linalg.norm(zem_vec))


def target_bang_bang_dir(state_i: DroneState, state_t: DroneState,
                         sign: float) -> np.ndarray:
    """Bang-bang evasion: max lateral accel perp to LOS and velocity."""
    _, _, r_hat, _ = los_geometry(state_i, state_t)
    cross = np.cross(r_hat, state_t.vel)
    mag   = np.linalg.norm(cross)
    if mag < 1e-6:
        cross = np.array([0.0, 0.0, 1.0])
    else:
        cross = cross / mag
    return sign * cross


def integrate_step(state: DroneState, accel: np.ndarray, dt: float,
                   speed_limit: Optional[float] = None) -> DroneState:
    """Simple forward Euler integration; optionally clamp speed."""
    new_vel = state.vel + accel * dt
    if speed_limit is not None:
        speed = np.linalg.norm(new_vel)
        if speed > speed_limit:
            new_vel = new_vel * speed_limit / speed
    new_pos = state.pos + new_vel * dt
    return DroneState(pos=new_pos, vel=new_vel)


def run_trial(guidance: str, rng: np.random.Generator
              ) -> Tuple[bool, float, float, List[np.ndarray], List[np.ndarray]]:
    """
    Run one Monte Carlo trial.
    Returns: (success, T_cap_or_nan, d_miss, interceptor_traj, target_traj)
    """
    # Randomise initial positions
    p_i0 = rng.uniform([0, 0, 2], [AIRSPACE_X, AIRSPACE_Y, AIRSPACE_Z])
    p_t0 = rng.uniform([0, 0, 2], [AIRSPACE_X, AIRSPACE_Y, AIRSPACE_Z])
    while np.linalg.norm(p_i0 - p_t0) < 5.0:     # ensure non-trivial start
        p_t0 = rng.uniform([0, 0, 2], [AIRSPACE_X, AIRSPACE_Y, AIRSPACE_Z])

    # Randomise initial velocities at nominal speeds
    def rand_vel(speed):
        v = rng.standard_normal(3)
        v[2] *= 0.3              # bias toward horizontal flight
        return v / np.linalg.norm(v) * speed

    state_i = DroneState(pos=p_i0.copy(), vel=rand_vel(V_INTERCEPTOR))
    state_t = DroneState(pos=p_t0.copy(), vel=rand_vel(V_TARGET))

    # Evasion mode state
    evasion_mode = False
    bang_sign    = 1.0
    time_to_switch = rng.exponential(TAU_SWITCH_MEAN)

    traj_i: List[np.ndarray] = [state_i.pos.copy()]
    traj_t: List[np.ndarray] = [state_t.pos.copy()]
    d_min  = np.inf
    t      = 0.0

    while t < T_MAX:
        t += DT

        # ── guidance law ──────────────────────────────────────────────────────
        if guidance == "png":
            a_i = png_command(state_i, state_t)
        elif guidance == "pure":
            a_i = pure_pursuit_command(state_i, state_t)
        else:  # "lead"
            a_i = lead_pursuit_command(state_i, state_t)

        # ── target evasion ────────────────────────────────────────────────────
        time_to_switch -= DT
        if time_to_switch <= 0:
            evasion_mode   = not evasion_mode
            bang_sign      = rng.choice([-1.0, 1.0])
            time_to_switch = rng.exponential(TAU_SWITCH_MEAN)

        if evasion_mode:
            a_t = A_T_MAX * target_bang_bang_dir(state_i, state_t, bang_sign)
        else:
            a_t = np.zeros(3)

        # ── integrate ─────────────────────────────────────────────────────────
        state_i = integrate_step(state_i, a_i, DT, speed_limit=V_INTERCEPTOR * 1.5)
        state_t = integrate_step(state_t, a_t, DT, speed_limit=V_TARGET * 1.5)

        # Soft airspace boundary reflection for target
        for ax, lim in enumerate([AIRSPACE_X, AIRSPACE_Y, AIRSPACE_Z]):
            if state_t.pos[ax] < 0 or state_t.pos[ax] > lim:
                state_t.vel[ax] *= -1
                state_t.pos[ax] = np.clip(state_t.pos[ax], 0, lim)

        traj_i.append(state_i.pos.copy())
        traj_t.append(state_t.pos.copy())

        dist = np.linalg.norm(state_i.pos - state_t.pos)
        if dist < d_min:
            d_min = dist

        if dist <= R_CAPTURE:
            return True, t, d_min, traj_i, traj_t

    return False, np.nan, d_min, traj_i, traj_t


def monte_carlo(guidance: str, n_trials: int = N_MC, seed: int = SEED_BASE
                ) -> dict:
    """Run n_trials and aggregate statistics."""
    rng      = np.random.default_rng(seed)
    successes, cap_times, miss_dists = [], [], []
    sample_traj = None    # store one representative trial for plotting

    for k in range(n_trials):
        ok, t_cap, d_miss, traj_i, traj_t = run_trial(guidance, rng)
        successes.append(ok)
        cap_times.append(t_cap)
        miss_dists.append(d_miss)
        if k == 0:
            sample_traj = (traj_i, traj_t)

    success_rate = np.mean(successes) * 100.0
    cap_arr      = np.array([t for t, ok in zip(cap_times, successes) if ok])
    miss_arr     = np.array([d for d, ok in zip(miss_dists, successes) if not ok])

    print(f"[{guidance.upper():5s}]  success={success_rate:5.1f}%  "
          f"mean T_cap={np.mean(cap_arr):.2f} s  "
          f"mean d_miss={np.mean(miss_arr) if len(miss_arr) else 0:.3f} m")

    return {
        "guidance":      guidance,
        "success_rate":  success_rate,
        "cap_times":     cap_arr,
        "miss_dists":    miss_arr,
        "sample_traj_i": np.array(sample_traj[0]),
        "sample_traj_t": np.array(sample_traj[1]),
    }


# ── Plotting ──────────────────────────────────────────────────────────────────
def plot_3d_trajectories(results: dict, save_path: str = None):
    """
    Two-panel 3D plot:
      Left  — PNG sample trajectory (interceptor red, target blue).
      Right — lead pursuit sample trajectory for comparison.
    """
    fig = plt.figure(figsize=(14, 6))
    titles = [("png", "PNG (N=4)"), ("lead", "Lead Pursuit")]
    colors_i = ["red",  "red"]
    colors_t = ["blue", "blue"]

    for col_idx, (key, title) in enumerate(titles):
        ax = fig.add_subplot(1, 2, col_idx + 1, projection='3d')
        res = results[key]
        ti  = res["sample_traj_i"]
        tt  = res["sample_traj_t"]

        ax.plot(ti[:, 0], ti[:, 1], ti[:, 2],
                color=colors_i[col_idx], lw=1.5, label='Interceptor', alpha=0.9)
        ax.plot(tt[:, 0], tt[:, 1], tt[:, 2],
                color=colors_t[col_idx], lw=1.5, label='Target', alpha=0.9)
        ax.scatter(*ti[0], color='red',   s=50, marker='o', label='I start')
        ax.scatter(*tt[0], color='blue',  s=50, marker='s', label='T start')
        ax.scatter(*ti[-1], color='green', s=80, marker='*', label='End')

        ax.set_xlim(0, AIRSPACE_X)
        ax.set_ylim(0, AIRSPACE_Y)
        ax.set_zlim(0, AIRSPACE_Z)
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.set_title(f'S094 — {title} (sample trial)')
        ax.legend(fontsize=7, loc='upper left')

    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
    plt.show()


def plot_monte_carlo_summary(results: dict, save_path: str = None):
    """
    Two-panel summary:
      Left  — capture-time CDF for all three guidance laws.
      Right — miss-distance histogram for failed trials.
    """
    fig, axes = plt.subplots(1, 2, figsize=(13, 5))
    labels  = {"png": "PNG (N=4)", "pure": "Pure Pursuit", "lead": "Lead Pursuit"}
    colors  = {"png": "red",       "pure": "orange",       "lead": "steelblue"}

    # ── CDF of capture times ──────────────────────────────────────────────────
    ax = axes[0]
    for key, label in labels.items():
        cap = results[key]["cap_times"]
        if len(cap) == 0:
            continue
        cap_sorted = np.sort(cap)
        cdf        = np.arange(1, len(cap_sorted) + 1) / N_MC
        ax.step(cap_sorted, cdf * 100, color=colors[key], lw=2, label=label)

    ax.set_xlabel('Capture time $T_{cap}$ (s)')
    ax.set_ylabel('Cumulative success (%)')
    ax.set_title('S094 — Capture Time CDF (Monte Carlo)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_xlim(0, T_MAX)
    ax.set_ylim(0, 100)

    # ── Miss distance histogram (failed trials only) ──────────────────────────
    ax = axes[1]
    for key, label in labels.items():
        md = results[key]["miss_dists"]
        if len(md) == 0:
            continue
        ax.hist(md, bins=20, color=colors[key], alpha=0.55, label=label, density=True)

    ax.axvline(R_CAPTURE, color='black', ls='--', lw=1.5,
               label=f'Capture radius ({R_CAPTURE} m)')
    ax.set_xlabel('Miss distance $d_{miss}$ (m)')
    ax.set_ylabel('Probability density')
    ax.set_title('S094 — Miss Distance Distribution (failed trials)')
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
    plt.show()


def plot_zem_and_range(results: dict, save_path: str = None):
    """
    Time-series panel for the PNG sample trial:
      Top    — range R(t) with capture threshold dashed.
      Bottom — approximate ZEM(t) reconstructed from saved trajectories.
    (Actual ZEM requires velocity; we plot range as a proxy here.)
    """
    res = results["png"]
    ti  = res["sample_traj_i"]
    tt  = res["sample_traj_t"]
    t   = np.arange(len(ti)) * DT
    R   = np.linalg.norm(ti - tt, axis=1)

    fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)

    axes[0].plot(t, R, color='red', lw=1.5, label='Range $R(t)$')
    axes[0].axhline(R_CAPTURE, color='green', ls='--', lw=1.5,
                    label=f'Capture radius {R_CAPTURE} m')
    axes[0].set_ylabel('Range (m)')
    axes[0].set_title('S094 — PNG Sample Trial: Range and Closing Dynamics')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    # Pseudo-ZEM: finite-difference velocity from trajectory
    v_i   = np.gradient(ti, DT, axis=0)
    v_t   = np.gradient(tt, DT, axis=0)
    v_rel = v_t - v_i
    r_vec = tt - ti
    mag_r = np.linalg.norm(r_vec, axis=1, keepdims=True) + 1e-9
    V_c   = -np.sum(r_vec * v_rel, axis=1) / mag_r[:, 0]
    t_go  = R / (V_c + 0.01)
    t_go  = np.clip(t_go, 0, T_MAX)
    zem   = np.linalg.norm(r_vec + v_rel * t_go[:, np.newaxis], axis=1)

    axes[1].plot(t, zem, color='steelblue', lw=1.5, label='ZEM(t)')
    axes[1].axhline(R_CAPTURE, color='green', ls='--', lw=1.5,
                    label=f'Capture radius {R_CAPTURE} m')
    axes[1].set_xlabel('Time (s)')
    axes[1].set_ylabel('ZEM (m)')
    axes[1].set_title('Zero-Effort Miss vs Time (PNG sample trial)')
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
    plt.show()


def animate_intercept(results: dict, save_path: str = None):
    """
    Top-down (X-Y) 2D animation of the PNG sample trial.
    Interceptor: red dot + trail; Target: blue dot + trail.
    Airspace boundary drawn in grey.
    """
    res  = results["png"]
    ti   = res["sample_traj_i"]
    tt   = res["sample_traj_t"]
    n    = len(ti)

    fig, ax = plt.subplots(figsize=(7, 7))
    ax.set_xlim(0, AIRSPACE_X)
    ax.set_ylim(0, AIRSPACE_Y)
    ax.set_aspect('equal')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('S094 — PNG Intercept (top-down view)')
    # Airspace boundary
    rect = plt.Rectangle((0, 0), AIRSPACE_X, AIRSPACE_Y,
                          linewidth=1.5, edgecolor='grey', facecolor='none',
                          linestyle='--', label='Airspace')
    ax.add_patch(rect)

    trail_i, = ax.plot([], [], '-', color='red',      lw=1.0, alpha=0.6)
    trail_t, = ax.plot([], [], '-', color='blue',     lw=1.0, alpha=0.6)
    dot_i,   = ax.plot([], [], 'o', color='red',      ms=8,   label='Interceptor')
    dot_t,   = ax.plot([], [], 's', color='blue',     ms=8,   label='Target')
    time_txt = ax.text(0.02, 0.96, '', transform=ax.transAxes, fontsize=9)
    ax.legend(loc='upper right', fontsize=8)

    stride = max(1, n // 300)   # ~300 animation frames

    def _init():
        for artist in (trail_i, trail_t, dot_i, dot_t):
            artist.set_data([], [])
        time_txt.set_text('')
        return trail_i, trail_t, dot_i, dot_t, time_txt

    def _update(frame):
        idx = min(frame * stride, n - 1)
        trail_i.set_data(ti[:idx, 0], ti[:idx, 1])
        trail_t.set_data(tt[:idx, 0], tt[:idx, 1])
        dot_i.set_data([ti[idx, 0]], [ti[idx, 1]])
        dot_t.set_data([tt[idx, 0]], [tt[idx, 1]])
        time_txt.set_text(f't = {idx * DT:.1f} s')
        return trail_i, trail_t, dot_i, dot_t, time_txt

    n_frames = n // stride
    anim = animation.FuncAnimation(
        fig, _update, frames=n_frames, init_func=_init,
        interval=40, blit=True
    )
    if save_path:
        anim.save(save_path, fps=25, dpi=120)
    plt.show()
    return anim


# ── Main entry point ──────────────────────────────────────────────────────────
if __name__ == '__main__':
    import os
    OUT_DIR = 'outputs/05_special_entertainment/s094_counter_drone'
    os.makedirs(OUT_DIR, exist_ok=True)

    print("Running Monte Carlo simulations …")
    results = {}
    for guidance in ("png", "pure", "lead"):
        results[guidance] = monte_carlo(guidance, n_trials=N_MC, seed=SEED_BASE)

    plot_3d_trajectories(results,
        save_path=f'{OUT_DIR}/s094_3d_trajectories.png')
    plot_monte_carlo_summary(results,
        save_path=f'{OUT_DIR}/s094_monte_carlo_summary.png')
    plot_zem_and_range(results,
        save_path=f'{OUT_DIR}/s094_range_zem.png')
    animate_intercept(results,
        save_path=f'{OUT_DIR}/s094_animation.gif')
```

---

## Key Parameters

| Parameter | Symbol | Value |
|-----------|--------|-------|
| Airspace dimensions | — | 50 × 50 × 20 m |
| Interceptor cruise speed | $v_I$ | 10 m/s |
| Target cruise speed | $v_T$ | 6 m/s |
| Interceptor max lateral accel | $a_{I,max}$ | 15 m/s² |
| Target max evasion accel | $a_{T,max}$ | 8 m/s² |
| Navigation constant (PNG) | $N$ | 4 |
| Pure/lead pursuit gain | $K_{pp} = K_{lp}$ | 5 s⁻¹ |
| Capture radius | $r_{capture}$ | 0.5 m |
| Mission time budget | $T_{max}$ | 30 s |
| Simulation timestep | $\Delta t$ | 0.02 s (50 Hz) |
| Evasion mode mean dwell | $\bar{\tau}$ | 3 s |
| Monte Carlo trials per law | $N_{MC}$ | 200 |
| Speed ratio (interceptor/target) | $v_I / v_T$ | 1.67 |

---

## Expected Output

- **3D trajectory plot** (`Axes3D`, two panels): left panel shows a representative PNG trial —
  interceptor path in red, target path in blue, start markers as circle/square, intercept point
  as a green star; right panel shows the equivalent lead-pursuit trial for visual comparison;
  both panels share the same 50 × 50 × 20 m airspace limits; qualitative difference in
  tail-chase vs lead geometry clearly visible.
- **Range and ZEM time series** (two stacked panels): top panel shows $R(t)$ (red) and the
  $r_{capture}$ threshold (green dashed) for the PNG sample trial; bottom panel shows $ZEM(t)$
  (blue) converging toward zero as the PNG law nulls the miss; x-axis shared; evasion mode
  switch instants visible as kinks in both curves.
- **Monte Carlo summary** (two panels): left panel shows the cumulative success CDF vs capture
  time for all three guidance laws (PNG red, pure orange, lead blue), x-axis 0–30 s; right
  panel shows histogram of miss distances for failed trials, with the capture-radius line
  overlaid; PNG expected to show highest success rate and shortest mean capture time.
- **Top-down 2D animation** (GIF): PNG sample trial from above; interceptor red dot with fading
  red trail, target blue square with fading blue trail; airspace boundary in grey dashed; time
  counter in the upper left; 25 fps playback; evasion jinks visible in the target trail.
- **Console metrics table** (printed): for each guidance law — success rate (%), mean capture
  time (s, successful trials only), mean miss distance (m, failed trials only).

---

## Extensions

1. **Augmented PNG (APNG)**: add an estimated target acceleration term to the guidance command —
   $\mathbf{a}_{APNG} = N V_c \boldsymbol{\Omega}_{LOS} + \hat{\mathbf{a}}_T$ — where
   $\hat{\mathbf{a}}_T$ is obtained from an alpha-beta filter on the observed LOS data; compare
   APNG capture rate against standard PNG under maximum bang-bang evasion.
2. **Swarm intercept**: deploy 3 interceptors launched from different boundary points; share
   target position estimates via inter-drone messaging and assign intercepts by minimum time-to-go;
   evaluate how cooperation reduces escape probability when the single-interceptor success rate
   is below 70%.
3. **Sensor noise and estimation**: replace perfect state knowledge with a noisy radar
   ($\sigma_\rho = 0.2$ m, $\sigma_\theta = 0.5°$) feeding an Extended Kalman Filter; re-run
   Monte Carlo with the estimated target state and measure the degradation in success rate
   attributable to estimation error alone.
4. **Speed advantage sweep**: vary the speed ratio $v_I / v_T \in [1.0, 2.5]$ in steps of 0.1
   and plot the success-rate curve for PNG vs pure pursuit; identify the critical ratio at which
   pure pursuit fails while PNG still achieves $> 90\%$ success.
5. **Differential game optimal evasion**: replace the bang-bang heuristic with the HJI-optimal
   evasion strategy from [S009 Differential Game](../../01_pursuit_evasion/S009_differential_game.md);
   measure the reduction in PNG success rate and estimate the value-of-the-game capture time
   under mutual optimality.
6. **Physical capture mechanism**: attach a net-deployment model ($r_{net} = 1.5$ m) that
   activates when $R < r_{deploy} = 2.0$ m and the interceptor's heading is within 15° of the
   LOS; this softens the capture radius requirement and models a real counter-drone payload.

---

## Related Scenarios

- Prerequisites: [S081 Selfie Follow](S081_selfie_follow.md) (basic drone tracking in domain 5),
  [S009 Differential Game](../../01_pursuit_evasion/S009_differential_game.md) (HJI optimal
  guidance), [S016 Airspace Defense](../../01_pursuit_evasion/S016_airspace_defense.md) (zone
  protection against intruder)
- Follow-ups: [S092 Autonomous Combat Drone](S092_autonomous_combat.md) (multi-agent armed
  intercept), [S099 Electronic Warfare Drone](S099_electronic_warfare.md) (jamming and
  counter-measures)
- Algorithmic cross-reference: [S018 Multi-Target Intercept](../../01_pursuit_evasion/S018_multi_target_intercept.md)
  (multiple simultaneous engagements), [S009 Differential Game](../../01_pursuit_evasion/S009_differential_game.md)
  (game-theoretic guidance analysis)

## References

- Shneydor, N. A. (1998). *Missile Guidance and Pursuit: Kinematics, Dynamics and Control*. Woodhead Publishing.
- Zarchan, P. (2012). *Tactical and Strategic Missile Guidance*, 6th ed. AIAA.
- Isaacs, R. (1965). *Differential Games*. Wiley.
- [MATH_FOUNDATIONS.md §4.1](../../MATH_FOUNDATIONS.md) (PNG kinematics)
- [MATH_FOUNDATIONS.md §4.3](../../MATH_FOUNDATIONS.md) (differential game formulation)
