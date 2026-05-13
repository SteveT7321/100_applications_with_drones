# S095 Water Surface Landing Simulation

**Domain**: Special Ops & Entertainment | **Difficulty**: ⭐⭐⭐ | **Status**: `[ ]` Not Started
**Algorithm**: FFT Wave Prediction + Feedforward Timed Descent | **Dimension**: 3D

---

## Problem Definition

**Setup**: A drone must land on a 1×1 m boat deck rocking with two-frequency ocean swell. The
deck's vertical position follows:

$$z_{deck}(t) = A_1 \sin\!\left(\frac{2\pi t}{T_1} + \varphi_1\right)
              + A_2 \sin\!\left(\frac{2\pi t}{T_2} + \varphi_2\right)$$

with primary swell amplitude $A_1 = 0.4$ m, period $T_1 = 6$ s, and secondary chop
$A_2 = 0.15$ m, period $T_2 = 2.5$ s. The deck centre is at nominal horizontal position
$(x_d, y_d) = (8, 0)$ m; horizontal motion is neglected (vertical-only rocking). The drone begins
at a fixed station at $(x_0, y_0, z_0) = (0, 0, 15)$ m.

**Roles**:

- **Drone**: single quadrotor agent; 3D linearised hover dynamics; controlled by an FFT-based
  feedforward timed descent in the vertical axis and a PD hold in the horizontal axes.
- **Deck**: a 1×1 m square platform heaving vertically with two-frequency swell; the phase
  parameters $\varphi_1, \varphi_2$ are fixed but unknown to the controller — they are recovered
  from the FFT of recent sensor measurements.
- **Landing window**: a brief interval centred on each deck crest (local maximum of $z_{deck}$)
  where relative vertical velocity is smallest and the surface is momentarily flat.

**Objective**: Reach the deck within the landing tolerance in a single coordinated descent.
Success requires both conditions simultaneously at the moment of contact:

1. Position error $\|\mathbf{p}_{drone} - \mathbf{p}_{deck}\| \leq r_{land} = 0.3$ m.
2. Relative vertical velocity $|\dot{z}_{drone} - \dot{z}_{deck}| \leq v_{land,max} = 0.1$ m/s.

**Comparison strategies** (two compared):

1. **Naive descent** — constant-velocity descent at $v_{desc} = 0.4$ m/s, ignoring wave phase;
   timing determined only by horizontal arrival; expected high miss rate due to phase mismatch.
2. **FFT-timed descent** (proposed) — collects $N_{obs} = 64$ recent samples of $z_{deck}$
   measurements, applies FFT to identify dominant frequencies and phases, predicts the next crest
   time $t_{crest}$, and initiates descent to arrive exactly at the crest. Expected landing success
   rate $\geq 85\%$ over Monte Carlo trials.

---

## Mathematical Model

### Two-Frequency Wave Model

The deck vertical position and velocity are:

$$z_{deck}(t) = \sum_{i=1}^{2} A_i \sin\!\left(\frac{2\pi t}{T_i} + \varphi_i\right)$$

$$\dot{z}_{deck}(t) = \sum_{i=1}^{2} A_i \cdot \frac{2\pi}{T_i}
  \cos\!\left(\frac{2\pi t}{T_i} + \varphi_i\right)$$

The total heave amplitude is bounded by $A_1 + A_2 = 0.55$ m. A crest occurs at time $t^*$
satisfying $\dot{z}_{deck}(t^*) = 0$ and $\ddot{z}_{deck}(t^*) < 0$.

### FFT Wave Prediction

The drone observes a window of $N_{obs} = 64$ deck height measurements sampled at $f_s = 10$ Hz
(window duration $6.4$ s, capturing more than one full primary swell cycle). Denote the
measurement vector $\mathbf{z}_{obs} \in \mathbb{R}^{N_{obs}}$.

The discrete Fourier transform gives:

$$\hat{Z}[k] = \sum_{n=0}^{N_{obs}-1} z_{obs}[n]\, e^{-j 2\pi k n / N_{obs}}, \quad
  k = 0, 1, \ldots, N_{obs}-1$$

Frequency bins with significant energy identify the dominant components. For each detected peak
at bin $k_i$ (frequency $f_i = k_i f_s / N_{obs}$):

- **Amplitude**: $\hat{A}_i = 2|\hat{Z}[k_i]| / N_{obs}$
- **Phase**: $\hat{\varphi}_i = \angle\hat{Z}[k_i]$ (unwrapped to current time)

### Predicted Crest Time

Using the recovered parameters $\{\hat{A}_i, \hat{f}_i, \hat{\varphi}_i\}$, the predicted deck
height at a future time $t$ is:

$$\hat{z}_{deck}(t) = \sum_{i} \hat{A}_i \sin\!\left(2\pi \hat{f}_i t + \hat{\varphi}_i\right)$$

The derivative is:

$$\dot{\hat{z}}_{deck}(t) = \sum_{i} \hat{A}_i \cdot 2\pi \hat{f}_i
  \cos\!\left(2\pi \hat{f}_i t + \hat{\varphi}_i\right)$$

The next crest time $t_{crest}$ is the smallest $t > t_{now}$ satisfying:

$$\dot{\hat{z}}_{deck}(t_{crest}) = 0, \qquad \ddot{\hat{z}}_{deck}(t_{crest}) < 0$$

In practice this is solved by scanning $\dot{\hat{z}}_{deck}$ over a dense time grid and finding
the first upward-to-downward zero crossing.

### Timed Descent Trajectory

Once $t_{crest}$ is determined, the descent is initiated at time $t_{start}$ such that the drone
arrives at deck height at $t_{crest}$:

$$t_{start} = t_{crest} - \frac{z_{hold} - z_{deck,0}}{v_{desc}}$$

where $z_{hold}$ is the holding altitude just above the deck ($z_{hold} = z_{deck,0} + 1.5$ m),
$z_{deck,0} = \hat{z}_{deck}(t_{start})$, and $v_{desc} = 0.4$ m/s is the nominal descent speed.
The vertical reference trajectory during descent is:

$$z_{drone}^{ref}(t) = z_{hold} - v_{desc}\,(t - t_{start}), \qquad t \in [t_{start},\; t_{crest}]$$

### Simplified 3D Quadrotor Dynamics

The state vector is:

$$\mathbf{x} = \begin{pmatrix}
  p_x & p_y & p_z & v_x & v_y & v_z & \phi & \theta & \dot{\phi} & \dot{\theta}
\end{pmatrix}^\top \in \mathbb{R}^{10}$$

The linearised continuous-time hover dynamics are $\dot{\mathbf{x}} = A\mathbf{x} + B\mathbf{u}$:

$$A = \begin{pmatrix}
  \mathbf{0}_{3\times3} & \mathbf{I}_3 & \mathbf{0}_{3\times4} \\
  \mathbf{0}_{3\times3} & \mathbf{0}_{3\times3} &
    \begin{pmatrix} 0 & g & 0 & 0 \\ -g & 0 & 0 & 0 \\ 0 & 0 & 0 & 0 \end{pmatrix} \\
  \mathbf{0}_{4\times3} & \mathbf{0}_{4\times3} &
    \begin{pmatrix} \mathbf{0}_{2\times2} & \mathbf{I}_2 \\
      \mathbf{0}_{2\times2} & \mathbf{0}_{2\times2} \end{pmatrix}
\end{pmatrix}$$

The control input $\mathbf{u} = (T,\, \tau_\phi,\, \tau_\theta)^\top$ enters through:

$$B[5,0] = \frac{1}{m}, \quad B[8,1] = \frac{1}{I_{xx}}, \quad B[9,2] = \frac{1}{I_{yy}}$$

with $g = 9.81$ m/s², drone mass $m = 1.0$ kg, inertias $I_{xx} = I_{yy} = 0.01$ kg·m².

### PD Horizontal and Vertical Controller

The control policy is split by axis:

**Horizontal** (hold above deck during observation; close to $(x_d, y_d)$ before descent):

$$u_{x} = K_{p,xy}(x_d - p_x) + K_{d,xy}(0 - v_x)$$
$$u_{y} = K_{p,xy}(y_d - p_y) + K_{d,xy}(0 - v_y)$$

**Vertical** (PD on the timed reference $z_{drone}^{ref}$):

$$u_z = K_{p,z}(z_{drone}^{ref}(t) - p_z) + K_{d,z}(\dot{z}_{drone}^{ref}(t) - v_z) + m g$$

with $K_{p,xy} = 2.0$, $K_{d,xy} = 1.2$, $K_{p,z} = 6.0$, $K_{d,z} = 3.0$.

### Landing Success Metric

For a single trial, landing is **successful** if, at the first instant $t_l$ that $p_z \leq
z_{deck}(t_l) + r_{land}$, all of the following hold:

$$\left\|\mathbf{p}_{xy,drone}(t_l) - (x_d, y_d)\right\| \leq r_{land}$$
$$\left|p_z(t_l) - z_{deck}(t_l)\right| \leq r_{land}$$
$$\left|\dot{z}_{drone}(t_l) - \dot{z}_{deck}(t_l)\right| \leq v_{land,max}$$

The Monte Carlo landing success probability over $N_{trials}$ independent trials with randomised
initial phases $\varphi_1, \varphi_2 \sim \mathcal{U}[0, 2\pi)$ is:

$$P_{land} = \frac{N_{success}}{N_{trials}} \times 100\%$$

---

## Implementation

```python
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

# ── Physical constants ────────────────────────────────────────────────────────
G        = 9.81      # m/s²
MASS     = 1.0       # kg
I_XX     = 0.01      # kg·m²
I_YY     = 0.01      # kg·m²

# ── Wave parameters ───────────────────────────────────────────────────────────
A1       = 0.40      # m   primary swell amplitude
T1       = 6.00      # s   primary swell period
A2       = 0.15      # m   secondary chop amplitude
T2       = 2.50      # s   secondary chop period

# ── Deck geometry ─────────────────────────────────────────────────────────────
X_DECK   = 8.0       # m   deck centre x
Y_DECK   = 0.0       # m   deck centre y
Z_DECK_NOMINAL = 2.0 # m   mean deck height above sea level
DECK_SIZE = 1.0      # m   deck edge length (1×1 m)

# ── Drone start ───────────────────────────────────────────────────────────────
X_START  = 0.0       # m
Y_START  = 0.0       # m
Z_START  = 15.0      # m
Z_HOLD   = Z_DECK_NOMINAL + 1.5  # m   observation hold altitude

# ── Landing criteria ──────────────────────────────────────────────────────────
R_LAND       = 0.30   # m    position tolerance
V_LAND_MAX   = 0.10   # m/s  relative vertical velocity tolerance

# ── Descent parameters ────────────────────────────────────────────────────────
V_DESC   = 0.40      # m/s  nominal descent speed
T_SCAN   = 8.0       # s    total future-time window scanned for next crest

# ── FFT observation ───────────────────────────────────────────────────────────
F_SAMPLE = 10.0      # Hz   deck sensor sample rate
N_OBS    = 64        # samples per FFT window (6.4 s)

# ── PD gains ─────────────────────────────────────────────────────────────────
KP_XY    = 2.0
KD_XY    = 1.2
KP_Z     = 6.0
KD_Z     = 3.0

# ── Simulation ────────────────────────────────────────────────────────────────
DT       = 0.02      # s    integration timestep
T_MAX    = 60.0      # s    mission horizon
N_TRIALS = 200       # Monte Carlo trials


# ── Wave kinematics ───────────────────────────────────────────────────────────

def deck_z(t, phi1, phi2):
    """Deck vertical position (two-frequency swell)."""
    return (Z_DECK_NOMINAL
            + A1 * np.sin(2 * np.pi * t / T1 + phi1)
            + A2 * np.sin(2 * np.pi * t / T2 + phi2))


def deck_zdot(t, phi1, phi2):
    """Deck vertical velocity."""
    return (A1 * (2 * np.pi / T1) * np.cos(2 * np.pi * t / T1 + phi1)
          + A2 * (2 * np.pi / T2) * np.cos(2 * np.pi * t / T2 + phi2))


# ── FFT wave prediction ───────────────────────────────────────────────────────

def fft_predict_crest(t_now, z_obs, dt_obs=1.0 / F_SAMPLE):
    """
    Estimate wave parameters via FFT from N_OBS recent deck height samples,
    then find the next crest time after t_now.

    Parameters
    ----------
    t_now   : float  — current simulation time
    z_obs   : array  — recent N_OBS deck height measurements (oldest first)
    dt_obs  : float  — sensor sampling interval (s)

    Returns
    -------
    t_crest : float  — predicted next crest time (> t_now)
    z_hat   : callable  — predicted z_deck(t) from recovered parameters
    zdot_hat: callable  — predicted dz_deck/dt(t) from recovered parameters
    """
    n = len(z_obs)
    # Remove mean
    z_centered = z_obs - np.mean(z_obs)
    # FFT
    Z_fft = np.fft.rfft(z_centered)
    freqs  = np.fft.rfftfreq(n, d=dt_obs)

    # Find two dominant frequency peaks (skip DC at k=0)
    magnitudes = np.abs(Z_fft)
    magnitudes[0] = 0.0   # suppress DC
    # Identify top-2 peaks
    peak_indices = np.argsort(magnitudes)[::-1][:2]

    amps   = []
    phases = []
    freq_peaks = []
    for k in peak_indices:
        if freqs[k] <= 0:
            continue
        amp_k   = 2.0 * magnitudes[k] / n
        phase_k = np.angle(Z_fft[k])
        # Unwrap phase to t_now reference: phase in z_obs corresponds to
        # t_start = t_now - (n-1)*dt_obs (oldest sample)
        t_start = t_now - (n - 1) * dt_obs
        # Phase at t=0 is phase_k; convert to phase at t_start
        phase_at_tnow = phase_k + 2 * np.pi * freqs[k] * t_start
        amps.append(amp_k)
        phases.append(phase_at_tnow)
        freq_peaks.append(freqs[k])

    def z_hat(t):
        return Z_DECK_NOMINAL + sum(
            a * np.sin(2 * np.pi * f * t + p)
            for a, f, p in zip(amps, freq_peaks, phases)
        )

    def zdot_hat(t):
        return sum(
            a * (2 * np.pi * f) * np.cos(2 * np.pi * f * t + p)
            for a, f, p in zip(amps, freq_peaks, phases)
        )

    # Scan for next crest: smallest t > t_now where zdot_hat crosses zero
    # downward (positive → negative)
    t_scan = np.linspace(t_now + DT, t_now + T_SCAN, int(T_SCAN / DT))
    zdot_vals = np.array([zdot_hat(t) for t in t_scan])

    t_crest = t_now + T1  # fallback: one primary period ahead
    for i in range(len(zdot_vals) - 1):
        if zdot_vals[i] > 0 and zdot_vals[i + 1] <= 0:
            # Linear interpolation to zero crossing
            alpha = zdot_vals[i] / (zdot_vals[i] - zdot_vals[i + 1])
            t_crest = t_scan[i] + alpha * (t_scan[i + 1] - t_scan[i])
            break

    return t_crest, z_hat, zdot_hat


# ── Controller ────────────────────────────────────────────────────────────────

def pd_control(state, x_ref, y_ref, z_ref, vz_ref=0.0):
    """
    Simple PD controller returning thrust deviation and attitude torques.
    state: [px, py, pz, vx, vy, vz, phi, theta, dphi, dtheta]
    """
    px, py, pz, vx, vy, vz = state[0:6]

    # Desired horizontal acceleration → desired pitch/roll angles (small-angle)
    ax_des = KP_XY * (x_ref - px) + KD_XY * (0.0 - vx)
    ay_des = KP_XY * (y_ref - py) + KD_XY * (0.0 - vy)
    az_des = KP_Z  * (z_ref - pz) + KD_Z  * (vz_ref - vz)

    # Thrust to produce az_des + hover
    thrust = MASS * (az_des + G)
    thrust = np.clip(thrust, 0.0, MASS * G * 2.0)

    # Desired pitch/roll (linearised: ax = g*theta, ay = -g*phi)
    theta_des = ax_des / G
    phi_des   = -ay_des / G
    theta_des = np.clip(theta_des, -0.4, 0.4)
    phi_des   = np.clip(phi_des,   -0.4, 0.4)

    # Attitude torques (simple P controller on angles)
    tau_phi   = 20.0 * (phi_des   - state[6]) - 5.0 * state[8]
    tau_theta = 20.0 * (theta_des - state[7]) - 5.0 * state[9]

    return thrust, tau_phi, tau_theta


# ── Euler integration step ────────────────────────────────────────────────────

def step(state, thrust, tau_phi, tau_theta):
    """Euler integration of linearised quadrotor dynamics."""
    px, py, pz, vx, vy, vz, phi, theta, dphi, dtheta = state

    ax = G * theta
    ay = -G * phi
    az = thrust / MASS - G

    new_state = state.copy()
    new_state[0:3] += state[3:6] * DT
    new_state[3] += ax * DT
    new_state[4] += ay * DT
    new_state[5] += az * DT
    new_state[6] += dphi   * DT
    new_state[7] += dtheta * DT
    new_state[8] += (tau_phi  / I_XX) * DT
    new_state[9] += (tau_theta / I_YY) * DT
    new_state[6:8] = np.clip(new_state[6:8], -0.5, 0.5)
    return new_state


# ── Single simulation run ─────────────────────────────────────────────────────

def run_simulation(use_fft=True, phi1=0.0, phi2=0.0, seed=0):
    """
    Simulate one landing attempt.

    Phases
    ------
    Phase 0 – transit: fly from start to hold point above deck.
    Phase 1 – observe: hover at Z_HOLD, accumulate N_OBS deck samples.
    Phase 2 – wait:    hold and wait until t_start (descent trigger time).
    Phase 3 – descent: timed constant-velocity descent toward crest.
    Phase 4 – landed / missed.

    Returns
    -------
    log     : dict with trajectory arrays
    outcome : dict with success flag and metrics
    """
    rng = np.random.default_rng(seed)

    state = np.array([X_START, Y_START, Z_START,
                      0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.0], dtype=float)

    t = 0.0
    phase = 0   # 0=transit, 1=observe, 2=wait, 3=descent
    obs_buffer = []
    obs_times  = []
    t_start_desc = None
    t_crest_pred = None
    z_hat_fn     = None
    zdot_hat_fn  = None
    z_ref_traj   = Z_START

    landed  = False
    missed  = False

    log = dict(t=[], pos=[], vel=[], z_deck=[], zdot_deck=[],
               z_ref=[], phase=[])

    while t < T_MAX and not landed and not missed:
        zd     = deck_z(t, phi1, phi2)
        zdotd  = deck_zdot(t, phi1, phi2)

        # ── Phase 0: transit to hold altitude above deck ──────────────────────
        if phase == 0:
            z_ref_traj = Z_HOLD
            thrust, tau_phi, tau_theta = pd_control(
                state, X_DECK, Y_DECK, z_ref_traj)
            # Transition: within 0.5 m of hold point
            err_xy = np.sqrt((state[0]-X_DECK)**2 + (state[1]-Y_DECK)**2)
            err_z  = abs(state[2] - Z_HOLD)
            if err_xy < 0.5 and err_z < 0.3 and abs(state[5]) < 0.05:
                phase = 1

        # ── Phase 1: accumulate N_OBS deck height samples ────────────────────
        elif phase == 1:
            z_ref_traj = Z_HOLD
            thrust, tau_phi, tau_theta = pd_control(
                state, X_DECK, Y_DECK, z_ref_traj)
            obs_buffer.append(zd)
            obs_times.append(t)
            if len(obs_buffer) >= N_OBS:
                if use_fft:
                    t_crest_pred, z_hat_fn, zdot_hat_fn = fft_predict_crest(
                        t, np.array(obs_buffer))
                else:
                    # Naive: ignore phase, descend immediately
                    t_crest_pred = t + 0.5
                    z_hat_fn     = lambda s: zd          # noqa
                    zdot_hat_fn  = lambda s: 0.0         # noqa
                # Compute descent start time
                z_arrival = z_hat_fn(t_crest_pred)
                t_start_desc = t_crest_pred - (Z_HOLD - z_arrival) / V_DESC
                if t_start_desc <= t:
                    # Crest too soon; wait for the next one
                    t_crest_pred += T1
                    t_start_desc  = t_crest_pred - (Z_HOLD - z_hat_fn(t_crest_pred)) / V_DESC
                phase = 2

        # ── Phase 2: hold and wait for descent trigger ────────────────────────
        elif phase == 2:
            z_ref_traj = Z_HOLD
            thrust, tau_phi, tau_theta = pd_control(
                state, X_DECK, Y_DECK, z_ref_traj)
            if t >= t_start_desc:
                phase = 3

        # ── Phase 3: timed descent ────────────────────────────────────────────
        elif phase == 3:
            elapsed    = t - t_start_desc
            z_ref_traj = Z_HOLD - V_DESC * elapsed
            vz_ref     = -V_DESC
            thrust, tau_phi, tau_theta = pd_control(
                state, X_DECK, Y_DECK, z_ref_traj, vz_ref=vz_ref)

            # Check landing condition
            err_xy  = np.sqrt((state[0]-X_DECK)**2 + (state[1]-Y_DECK)**2)
            err_z   = abs(state[2] - zd)
            rel_vz  = abs(state[5] - zdotd)

            if state[2] <= zd + R_LAND:
                if err_xy <= R_LAND and err_z <= R_LAND and rel_vz <= V_LAND_MAX:
                    landed = True
                else:
                    missed = True

        # Simulate measurement noise on deck sensor (±1 cm)
        zd_noisy = zd + rng.normal(0, 0.01)

        state = step(state, thrust, tau_phi, tau_theta)

        # Logging
        log['t'].append(t)
        log['pos'].append(state[0:3].copy())
        log['vel'].append(state[3:6].copy())
        log['z_deck'].append(zd)
        log['zdot_deck'].append(zdotd)
        log['z_ref'].append(z_ref_traj)
        log['phase'].append(phase)

        t += DT

    for k in ('pos', 'vel'):
        log[k] = np.array(log[k])
    for k in ('t', 'z_deck', 'zdot_deck', 'z_ref', 'phase'):
        log[k] = np.array(log[k])

    timing_err = float(abs(log['t'][-1] - t_crest_pred)) if t_crest_pred else np.nan
    rel_vz_final = float(abs(log['vel'][-1, 2] - log['zdot_deck'][-1]))

    outcome = dict(
        landed       = landed,
        missed       = missed,
        timing_error = timing_err,
        approach_vz  = float(abs(log['vel'][-1, 2])),
        rel_vz_final = rel_vz_final,
        mission_time = float(log['t'][-1]),
        t_crest_pred = t_crest_pred if t_crest_pred else np.nan,
    )
    return log, outcome


# ── Monte Carlo ───────────────────────────────────────────────────────────────

def run_monte_carlo(use_fft=True, n_trials=N_TRIALS):
    """Run N_TRIALS trials with random phase pairs; return success rate."""
    rng = np.random.default_rng(42)
    successes = 0
    timing_errors = []
    approach_vzs  = []

    for i in range(n_trials):
        phi1 = rng.uniform(0, 2 * np.pi)
        phi2 = rng.uniform(0, 2 * np.pi)
        _, outcome = run_simulation(use_fft=use_fft, phi1=phi1, phi2=phi2, seed=i)
        if outcome['landed']:
            successes += 1
        timing_errors.append(outcome['timing_error'])
        approach_vzs.append(outcome['approach_vz'])

    p_land = successes / n_trials * 100.0
    return p_land, np.array(timing_errors), np.array(approach_vzs)


# ── Visualisation ─────────────────────────────────────────────────────────────

def plot_results(log_fft, log_naive, outcome_fft, outcome_naive):
    """Four-panel analysis figure comparing FFT-timed vs naive descent."""
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

    fig = plt.figure(figsize=(18, 14))

    # ── Panel 1: 3D trajectory ────────────────────────────────────────────────
    ax1 = fig.add_subplot(2, 2, 1, projection='3d')
    pos_fft   = log_fft['pos']
    pos_naive = log_naive['pos']
    ax1.plot(pos_fft[:, 0],   pos_fft[:, 1],   pos_fft[:, 2],
             'r-',  lw=1.5, label='FFT-timed')
    ax1.plot(pos_naive[:, 0], pos_naive[:, 1], pos_naive[:, 2],
             'b--', lw=1.5, label='Naive')
    # Boat deck envelope
    t_env  = np.linspace(0, T_MAX, 400)
    z_env  = Z_DECK_NOMINAL + A1 * np.sin(2 * np.pi * t_env / T1) \
                            + A2 * np.sin(2 * np.pi * t_env / T2)
    ax1.scatter(np.full(400, X_DECK), np.full(400, Y_DECK), z_env,
                c='green', s=2, alpha=0.3, label='Deck motion')
    ax1.scatter([X_START], [Y_START], [Z_START], c='blue',  s=80,
                zorder=5, label='Start')
    ax1.scatter([X_DECK],  [Y_DECK],  [Z_DECK_NOMINAL], c='green', s=100,
                marker='^', zorder=5, label='Deck centre')
    ax1.set_xlabel('x (m)');  ax1.set_ylabel('y (m)');  ax1.set_zlabel('z (m)')
    ax1.set_title('3D Approach Trajectory')
    ax1.legend(fontsize=8)

    # ── Panel 2: Vertical position vs time ───────────────────────────────────
    ax2 = fig.add_subplot(2, 2, 2)
    ax2.plot(log_fft['t'],   log_fft['pos'][:, 2],   'r-',  lw=1.5,
             label='Drone z (FFT-timed)')
    ax2.plot(log_naive['t'], log_naive['pos'][:, 2],  'b--', lw=1.5,
             label='Drone z (Naive)')
    ax2.plot(log_fft['t'],   log_fft['z_deck'],        'g-',  lw=2,
             label='Deck $z_{deck}(t)$')
    ax2.fill_between(log_fft['t'],
                     log_fft['z_deck'] - R_LAND,
                     log_fft['z_deck'] + R_LAND,
                     alpha=0.15, color='green', label=f'$\\pm r_{{land}}$ band')
    if outcome_fft['t_crest_pred'] and not np.isnan(outcome_fft['t_crest_pred']):
        ax2.axvline(outcome_fft['t_crest_pred'], color='red', linestyle=':',
                    lw=1.5, label='Predicted crest')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Altitude (m)')
    ax2.set_title('Vertical Tracking: Drone vs Deck')
    ax2.legend(fontsize=8)

    # ── Panel 3: Relative vertical velocity ──────────────────────────────────
    ax3 = fig.add_subplot(2, 2, 3)
    rel_vz_fft   = log_fft['vel'][:, 2]   - log_fft['zdot_deck']
    rel_vz_naive = log_naive['vel'][:, 2] - log_naive['zdot_deck']
    ax3.plot(log_fft['t'],   rel_vz_fft,   'r-',  lw=1.5, label='FFT-timed')
    ax3.plot(log_naive['t'], rel_vz_naive, 'b--', lw=1.5, label='Naive')
    ax3.axhline( V_LAND_MAX, color='k', linestyle=':', lw=1.5,
                 label=f'$v_{{land,max}}$ = ±{V_LAND_MAX} m/s')
    ax3.axhline(-V_LAND_MAX, color='k', linestyle=':', lw=1.5)
    ax3.fill_between(log_fft['t'], -V_LAND_MAX, V_LAND_MAX,
                     alpha=0.10, color='green', label='Landing velocity band')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('$\\Delta\\dot{z}$ (m/s)')
    ax3.set_title('Relative Vertical Velocity')
    ax3.legend(fontsize=8)

    # ── Panel 4: XY position error ────────────────────────────────────────────
    ax4 = fig.add_subplot(2, 2, 4)
    err_xy_fft   = np.sqrt((log_fft['pos'][:, 0]   - X_DECK)**2
                         + (log_fft['pos'][:, 1]   - Y_DECK)**2)
    err_xy_naive = np.sqrt((log_naive['pos'][:, 0] - X_DECK)**2
                         + (log_naive['pos'][:, 1] - Y_DECK)**2)
    ax4.plot(log_fft['t'],   err_xy_fft,   'r-',  lw=1.5, label='FFT-timed')
    ax4.plot(log_naive['t'], err_xy_naive, 'b--', lw=1.5, label='Naive')
    ax4.axhline(R_LAND, color='k', linestyle=':', lw=1.5,
                label=f'$r_{{land}}$ = {R_LAND} m')
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Horizontal position error (m)')
    ax4.set_title('XY Positioning Error')
    ax4.legend(fontsize=8)

    plt.suptitle('S095 Water Surface Landing — Approach Analysis', fontsize=14)
    plt.tight_layout()
    plt.savefig('outputs/05_special_entertainment/s095_water_landing/approach_analysis.png',
                dpi=150, bbox_inches='tight')
    plt.show()


def plot_monte_carlo(p_fft, timing_fft, p_naive, timing_naive):
    """Two-panel Monte Carlo summary: success rate bar + timing error histogram."""
    fig, axes = plt.subplots(1, 2, figsize=(13, 5))

    # Success rate bar chart
    ax1 = axes[0]
    bars = ax1.bar(['FFT-timed', 'Naive'], [p_fft, p_naive],
                   color=['tomato', 'steelblue'], alpha=0.85, width=0.4)
    for bar, val in zip(bars, [p_fft, p_naive]):
        ax1.text(bar.get_x() + bar.get_width() / 2, val + 0.5,
                 f'{val:.1f}%', ha='center', va='bottom', fontsize=12, fontweight='bold')
    ax1.axhline(85, color='k', linestyle='--', lw=1.2, label='85 % target')
    ax1.set_ylim(0, 105)
    ax1.set_ylabel('Landing success rate (%)')
    ax1.set_title(f'Monte Carlo Landing Success\n($N$ = {N_TRIALS} trials)')
    ax1.legend(fontsize=9)

    # Timing error histogram
    ax2 = axes[1]
    valid_fft   = timing_fft[np.isfinite(timing_fft)]
    valid_naive = timing_naive[np.isfinite(timing_naive)]
    bins = np.linspace(0, max(valid_fft.max(), valid_naive.max()) + 0.1, 30)
    ax2.hist(valid_fft,   bins=bins, alpha=0.65, color='tomato',
             label='FFT-timed', density=True)
    ax2.hist(valid_naive, bins=bins, alpha=0.65, color='steelblue',
             label='Naive',     density=True)
    ax2.set_xlabel('Timing error |$t_{land}$ − $t_{crest}$| (s)')
    ax2.set_ylabel('Probability density')
    ax2.set_title('Crest Timing Error Distribution')
    ax2.legend(fontsize=9)

    plt.tight_layout()
    plt.savefig('outputs/05_special_entertainment/s095_water_landing/monte_carlo.png',
                dpi=150, bbox_inches='tight')
    plt.show()


def animate_landing(log, phi1=0.0, phi2=0.0):
    """Animate drone descending to heaving boat deck (3D side-on view)."""
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

    pos   = log['pos']
    zd_arr = log['z_deck']
    times  = log['t']
    n      = len(times)
    step   = max(1, n // 250)

    fig = plt.figure(figsize=(10, 6))
    ax  = fig.add_subplot(111, projection='3d')
    ax.set_xlim(-1, 12);  ax.set_ylim(-3, 3);  ax.set_zlim(0, 16)
    ax.set_xlabel('x (m)');  ax.set_ylabel('y (m)');  ax.set_zlabel('z (m)')
    ax.set_title('S095 Water Surface Landing — Descent Animation')

    # Static sea surface (z = 0 plane, rendered as a grid)
    xg = np.array([0, 12])
    yg = np.array([-2, 2])
    Xg, Yg = np.meshgrid(xg, yg)
    ax.plot_surface(Xg, Yg, np.zeros_like(Xg), alpha=0.15, color='cyan')

    # Boat hull (static box below deck)
    hull_xs = [X_DECK - 0.5, X_DECK + 0.5, X_DECK + 0.5, X_DECK - 0.5, X_DECK - 0.5]
    hull_ys = [Y_DECK - 0.5, Y_DECK - 0.5, Y_DECK + 0.5, Y_DECK + 0.5, Y_DECK - 0.5]
    hull_z_base = 0.0
    ax.plot(hull_xs, hull_ys, [hull_z_base]*5, 'gray', lw=2)

    # Dynamic deck (heaving rectangle)
    deck_line, = ax.plot([], [], [], color='saddlebrown', lw=4)

    # Drone dot and trail
    drone_dot,  = ax.plot([], [], [], 'ro', ms=8)
    trail_line, = ax.plot([], [], [], 'r-', lw=1.2, alpha=0.6)
    target_dot, = ax.plot([], [], [], 'g^', ms=10)
    ann = ax.text2D(0.02, 0.92, '', transform=ax.transAxes, fontsize=9)

    trail_x, trail_y, trail_z = [], [], []
    TRAIL = 100

    def init():
        deck_line.set_data([], []);  deck_line.set_3d_properties([])
        drone_dot.set_data([], []);  drone_dot.set_3d_properties([])
        trail_line.set_data([], []); trail_line.set_3d_properties([])
        target_dot.set_data([], []); target_dot.set_3d_properties([])
        return deck_line, drone_dot, trail_line, target_dot, ann

    def update(fi):
        k  = fi * step
        t  = times[k]
        p  = pos[k]
        zd = zd_arr[k]

        # Moving deck outline
        deck_line.set_data(hull_xs, hull_ys)
        deck_line.set_3d_properties([zd] * 5)
        target_dot.set_data([X_DECK], [Y_DECK])
        target_dot.set_3d_properties([zd])

        # Drone trail
        trail_x.append(p[0]); trail_y.append(p[1]); trail_z.append(p[2])
        if len(trail_x) > TRAIL:
            trail_x.pop(0); trail_y.pop(0); trail_z.pop(0)
        trail_line.set_data(trail_x, trail_y)
        trail_line.set_3d_properties(trail_z)
        drone_dot.set_data([p[0]], [p[1]])
        drone_dot.set_3d_properties([p[2]])

        rel_vz = abs(log['vel'][k, 2] - log['zdot_deck'][k])
        ann.set_text(
            f't = {t:.1f} s\n'
            f'$z_{{err}}$ = {abs(p[2] - zd):.3f} m\n'
            f'$|\\Delta\\dot{{z}}|$ = {rel_vz:.3f} m/s'
        )
        return deck_line, drone_dot, trail_line, target_dot, ann

    n_frames = n // step
    anim = FuncAnimation(fig, update, frames=n_frames, init_func=init,
                         blit=False, interval=50)
    anim.save(
        'outputs/05_special_entertainment/s095_water_landing/landing_approach.gif',
        writer='pillow', fps=20)
    plt.close()


if __name__ == '__main__':
    import os
    os.makedirs('outputs/05_special_entertainment/s095_water_landing', exist_ok=True)

    print('Running S095 Water Surface Landing Simulation ...')

    log_fft,   outcome_fft   = run_simulation(use_fft=True,  phi1=0.3, phi2=1.1, seed=0)
    log_naive, outcome_naive = run_simulation(use_fft=False, phi1=0.3, phi2=1.1, seed=0)

    print(f"FFT-timed : landed={outcome_fft['landed']}  "
          f"timing_err={outcome_fft['timing_error']:.3f} s  "
          f"|Δvz|={outcome_fft['rel_vz_final']:.3f} m/s")
    print(f"Naive     : landed={outcome_naive['landed']}  "
          f"timing_err={outcome_naive['timing_error']:.3f} s  "
          f"|Δvz|={outcome_naive['rel_vz_final']:.3f} m/s")

    plot_results(log_fft, log_naive, outcome_fft, outcome_naive)
    animate_landing(log_fft, phi1=0.3, phi2=1.1)

    print(f'Running Monte Carlo ({N_TRIALS} trials each) ...')
    p_fft,   t_err_fft,   vz_fft   = run_monte_carlo(use_fft=True)
    p_naive, t_err_naive, vz_naive = run_monte_carlo(use_fft=False)
    print(f"FFT-timed success rate : {p_fft:.1f}%  "
          f"mean_timing_err={np.nanmean(t_err_fft):.3f} s")
    print(f"Naive     success rate : {p_naive:.1f}%  "
          f"mean_timing_err={np.nanmean(t_err_naive):.3f} s")

    plot_monte_carlo(p_fft, t_err_fft, p_naive, t_err_naive)
    print('Done. Outputs saved to outputs/05_special_entertainment/s095_water_landing/')
```

---

## Key Parameters

| Parameter | Symbol | Value |
|-----------|--------|-------|
| Primary swell amplitude | $A_1$ | 0.40 m |
| Primary swell period | $T_1$ | 6.0 s |
| Secondary chop amplitude | $A_2$ | 0.15 m |
| Secondary chop period | $T_2$ | 2.5 s |
| Nominal deck height | $z_{deck,0}$ | 2.0 m |
| Deck size | — | 1×1 m |
| Deck horizontal position | $(x_d, y_d)$ | (8, 0) m |
| Drone start position | $(x_0, y_0, z_0)$ | (0, 0, 15) m |
| Observation hold altitude | $z_{hold}$ | 3.5 m |
| FFT window size | $N_{obs}$ | 64 samples |
| Sensor sample rate | $f_s$ | 10 Hz |
| Descent speed | $v_{desc}$ | 0.40 m/s |
| Landing position tolerance | $r_{land}$ | 0.30 m |
| Landing velocity tolerance | $v_{land,max}$ | 0.10 m/s |
| Sensor noise std dev | — | 0.01 m |
| Drone mass | $m$ | 1.0 kg |
| Roll / pitch inertia | $I_{xx}, I_{yy}$ | 0.01 kg·m² |
| Horizontal PD gains | $K_{p,xy},\, K_{d,xy}$ | 2.0, 1.2 |
| Vertical PD gains | $K_{p,z},\, K_{d,z}$ | 6.0, 3.0 |
| Monte Carlo trials | $N_{trials}$ | 200 |
| Simulation timestep | $\Delta t$ | 0.02 s |
| Mission horizon | $T_{max}$ | 60 s |

---

## Expected Output

- **3D approach trajectory**: `mpl_toolkits.mplot3d` plot showing the full drone path from start
  position at $(0, 0, 15)$ m to the heaving deck at $(8, 0)$ m; FFT-timed descent (red) and
  naive descent (blue dashed) compared; scattered green points trace the deck's vertical envelope
  over the mission; start (blue dot) and mean deck centre (green triangle) marked.
- **Vertical tracking panel**: altitude time series for drone (both strategies) and deck
  $z_{deck}(t)$ on the same axes; ±$r_{land}$ shaded band around the deck line; predicted crest
  time marked with a vertical dotted line; shows phase alignment of FFT strategy and phase
  mismatch of naive approach.
- **Relative vertical velocity panel**: $\Delta\dot{z}(t)$ for both controllers; ±$v_{land,max}$
  band shaded; FFT-timed controller expected to enter the band near the predicted crest moment,
  enabling a soft touchdown.
- **XY position error panel**: horizontal distance from deck centre $(8, 0)$ over time;
  $r_{land}$ reference line; demonstrates horizontal hold quality during observation and descent.
- **Monte Carlo success rate bar chart**: landing success percentage for FFT-timed and naive
  strategies over 200 trials; 85 % target line; annotated with exact percentages.
- **Crest timing error histogram**: empirical distribution of $|t_{land} - t_{crest}|$ per trial
  for both strategies; FFT-timed expected to cluster near zero, naive broadly distributed.
- **Landing approach animation (GIF)**: 3D side-on view; heaving deck rectangle (brown) moves
  vertically in real time on a transparent sea-surface plane; drone dot descends with red trail;
  green triangle marks the deck target; live annotations show $z_{err}$ and $|\Delta\dot{z}|$;
  saved at 20 fps.

**Expected metric targets** (FFT-timed, deterministic seed 0):

| Metric | Target |
|--------|--------|
| Landing success (single run) | YES |
| Crest timing error | $< 0.3$ s |
| Approach vertical velocity | $< 0.5$ m/s |
| Relative vertical velocity at contact | $\leq 0.1$ m/s |
| MC success rate (FFT-timed) | $\geq 85\%$ |
| MC success rate (Naive) | $\leq 45\%$ |

---

## Extensions

1. **Irregular sea state**: replace the two-frequency deterministic model with a JONSWAP spectrum
   realisation (sum of $N = 20$ random sinusoids with Pierson-Moskowitz amplitude envelope);
   retune the FFT window length and evaluate the degradation in prediction accuracy and landing
   success as significant wave height increases from 0.5 m to 2.0 m.
2. **Horizontal deck drift**: extend the boat model to include slow horizontal surge and sway
   ($\pm 0.3$ m at 0.05 Hz); add a 2D Kalman filter to the drone that tracks the deck centroid
   from onboard camera measurements; report joint landing success under coupled position and
   timing uncertainty.
3. **EKF wave estimator**: remove the FFT batch processing and replace it with a recursive
   Extended Kalman Filter that estimates $\{A_i, f_i, \varphi_i\}$ online from a rolling
   altimeter stream; compare convergence time and prediction accuracy versus the FFT batch approach.
4. **Multi-attempt retry policy**: allow the drone to execute up to three landing attempts per
   mission; if the first attempt misses, the drone pulls up to $z_{hold}$ and re-observes for a
   shorter window ($N_{obs} = 32$); study the trade-off between retry delay and cumulative success
   probability.
5. **Model Predictive Control descent**: replace the fixed-speed timed descent with a short-horizon
   MPC (prediction horizon $T_p = 1$ s) that optimises the vertical thrust profile to arrive at
   the deck crest with minimal relative velocity while respecting thrust saturation constraints.

---

## Related Scenarios

- Prerequisites: [S079 Offshore Wind Farm Installation](../04_industrial_agriculture/S079_offshore_wind.md)
  (heaving platform landing, LQR + feedforward architecture),
  [S084 Aerial Refuelling Rendezvous](S084_aerial_refuelling.md) (precision approach to a moving
  target)
- Algorithmic cross-reference: [S023 Moving Landing Pad](../02_logistics_delivery/S023_moving_landing_pad.md)
  (velocity-matching landing), [S058 Typhoon Eye Probing](../03_environmental_sar/S058_typhoon.md)
  (disturbance rejection under environmental dynamics)
- Follow-ups: [S096 Ship Deck Night Landing](S096_ship_deck_night.md) (vision-impaired deck
  localisation + wave prediction under sensor noise)
