# S094 Aerial Ball Catching

**Domain**: Special Ops & Entertainment | **Difficulty**: ⭐⭐⭐⭐ | **Status**: `[x]` Complete
**Algorithm**: Kalman Ballistic Prediction + Time-Feasible Catch-Point Planning vs Pure Pursuit / Proportional Navigation | **Dimension**: 3D

---

## Problem Definition

**Setup**: A $20 \times 20 \times 8$ m indoor flying arena (the kind used for public drone demos,
e.g. ETH Zürich's Flying Machine Arena) hosts a crowd-pleasing act: a person on the arena edge
throws a tennis ball into the air and a drone carrying a small net cup on top must catch it
before it falls below $z_{min} = 0.5$ m. The ball follows a ballistic trajectory with quadratic
air drag. A motion-capture system reports the ball position at 100 Hz with Gaussian noise and a
small latency; the drone must estimate the ball's state and decide where to go while the ball is
still in flight (total flight time to $z_{min}$ ≈ 1.0–2.2 s, so there is no time to wait for a perfect
estimate).

Each throw is randomised: the thrower stands at a random point on a circle of radius 8 m around
the arena centre, throws roughly towards the centre (azimuth jitter ±30°) with speed
7–13 m/s and elevation 35°–60°. The drone starts hovering at a random point within 4 m of the
centre at $z = 2$ m and reacts only after a detection delay $t_{react} = 0.15$ s.

**Roles**:

- **Drone** ($v_{max} = 8$ m/s, $a_{max} = 10$ m/s²): point-mass with a net cup of catch radius
  $r_c = 0.15$ m; altitude limited to $[0.3, 4.0]$ m.
- **Ball** (tennis ball, $m_b = 0.057$ kg, $r_b = 0.033$ m, $C_d = 0.55$): passive projectile.

**Objective**: Maximise the catch success rate over a Monte Carlo set of throws. Three
strategies are compared on **identical throws** (common random numbers):

1. **Pure pursuit (PP)** — fly straight at the *current* estimated ball position at full speed.
2. **Proportional navigation (PN)** — 3D PN on the line of sight to the ball, $N = 3$.
3. **Predictive catch point (PCP)** — propagate the estimated ball state forward with the drag
   model, pick the **earliest point on the predicted trajectory the drone can reach in time**
   (within the altitude window), fly a time-constrained trajectory to it and re-plan at 20 Hz.

A fourth, non-causal reference — the **oracle** (true trajectory known at launch, same drone
limits) — gives an upper bound on what any strategy could achieve for each throw, so failures
can be split into "physically unreachable" and "strategy failure".

---

## Mathematical Model

### Ball Dynamics

$$\dot{\mathbf{p}}_b = \mathbf{v}_b, \qquad
\dot{\mathbf{v}}_b = \mathbf{g} - k \, \|\mathbf{v}_b\| \, \mathbf{v}_b, \qquad
k = \frac{\rho \, C_d \, \pi r_b^2}{2 m_b}$$

With $\rho = 1.225$ kg/m³ this gives $k \approx 0.0202$ m⁻¹ and a terminal speed
$v_{term} = \sqrt{g/k} \approx 22$ m/s, so drag changes the landing point (at $z_{min}$) by 0.4–3.7 m (mean 1.6 m over the
300 throws of seed 0) — far larger than the catch radius — and cannot be ignored by the predictor.

### Measurement Model

$$\mathbf{z}_k = \mathbf{p}_b(t_k - \tau_{lat}) + \boldsymbol{\eta}_k, \qquad
\boldsymbol{\eta}_k \sim \mathcal{N}(\mathbf{0}, \sigma_z^2 \mathbf{I}_3)$$

at $f_z = 100$ Hz, $\sigma_z = 0.02$ m, $\tau_{lat} = 0.02$ s.

### Ball State Estimator (EKF)

State $\mathbf{x} = [\mathbf{p}_b, \mathbf{v}_b] \in \mathbb{R}^6$, prediction with the drag model
above, Jacobian

$$\frac{\partial \dot{\mathbf{v}}_b}{\partial \mathbf{v}_b}
= -k \left( \|\mathbf{v}_b\| \mathbf{I}_3 + \frac{\mathbf{v}_b \mathbf{v}_b^\top}{\|\mathbf{v}_b\|} \right),$$

process noise $q$ on acceleration, position measurement $H = [\mathbf{I}_3 \; \mathbf{0}]$.
The estimate is propagated over the known latency before use.

### Drone Model

Double integrator with saturations (thrust already compensates gravity):

$$\dot{\mathbf{p}}_d = \mathbf{v}_d, \quad \dot{\mathbf{v}}_d = \mathbf{u}, \quad
\|\mathbf{u}\| \le a_{max}, \quad \|\mathbf{v}_d\| \le v_{max}, \quad 0.3 \le z_d \le 4.0$$

### Catch Condition

A catch occurs at the first time the ball passes within $r_c$ of the net centre
$\mathbf{p}_n = \mathbf{p}_d + [0, 0, 0.1]^\top$ **while descending** ($v_{b,z} < 0$) and with
$z_b \ge z_{min}$. Because the relative speed can reach 15 m/s, the check uses the closest
approach of the two straight-line segments within each time step, not only the sample points
(otherwise a fast ball can "tunnel" through the net between samples).

A throw fails if the ball drops below $z_{min}$ without a catch.

### Strategy 1 — Pure Pursuit

$$\mathbf{v}_{des} = v_{max} \frac{\hat{\mathbf{p}}_b - \mathbf{p}_n}{\|\hat{\mathbf{p}}_b - \mathbf{p}_n\|},
\qquad \mathbf{u} = \mathrm{sat}_{a_{max}}\!\big(K_v(\mathbf{v}_{des} - \mathbf{v}_d)\big)$$

### Strategy 2 — 3D Proportional Navigation

With LOS $\mathbf{r} = \hat{\mathbf{p}}_b - \mathbf{p}_n$, relative velocity
$\mathbf{v}_r = \hat{\mathbf{v}}_b - \mathbf{v}_d$ and LOS rate
$\boldsymbol{\Omega} = \dfrac{\mathbf{r} \times \mathbf{v}_r}{\|\mathbf{r}\|^2}$:

$$\mathbf{u}_{PN} = N \, V_c \, \boldsymbol{\Omega} \times \hat{\mathbf{r}}, \qquad
V_c = -\frac{\mathbf{r} \cdot \mathbf{v}_r}{\|\mathbf{r}\|}$$

PN needs closing speed to work; while $V_c \le 0$ (the ball is still flying away), the drone
falls back to pure pursuit. Note that PN treats the ball as a non-accelerating target — the
ball's gravity acceleration is an unmodelled 9.81 m/s² target manoeuvre (an *augmented PN*
term $\tfrac{N}{2}\mathbf{a}_{b,\perp}$ is an extension).

### Strategy 3 — Predictive Catch Point

1. Propagate the EKF estimate forward to obtain the predicted path
   $\hat{\mathbf{p}}_b(t)$, $t \in [t_{now}, t_{fall}]$ where $\hat z_b(t_{fall}) = z_{min}$.
2. For candidate times $t_j$ (every 0.02 s) with $\hat z_b(t_j) \in [0.5, 4.0]$ and
   $\hat v_{b,z}(t_j) < 0$, compute the minimum reach time $T_{reach}(\mathbf{p}_d, \mathbf{v}_d
   \to \hat{\mathbf{p}}_b(t_j))$ under the drone limits (per-axis bang–coast–bang bound is
   acceptable; document the approximation).
3. Choose the **earliest** $t_j$ with $T_{reach} + t_{margin} \le t_j - t_{now}$
   ($t_{margin} = 0.1$ s); if none exists, choose the $t_j$ that minimises
   $T_{reach} - (t_j - t_{now})$.
4. Track a time-parameterised trajectory to arrive at $\hat{\mathbf{p}}_b(t_j)$ at time $t_j$
   (e.g. a minimum-jerk polynomial with saturation), re-planning at 20 Hz as the estimate
   improves.

### Oracle Upper Bound

Same as PCP but using the true ball trajectory from $t = 0$ and the same $t_{react}$, giving the
fraction of throws that are reachable at all. A strategy's **efficiency** is
$\eta = N_{catch} / N_{reachable}$.

### Metrics

- Catch rate (with 95 % Wilson interval) and efficiency $\eta$ per strategy.
- Miss distance $d_{miss} = \min_t \|\mathbf{p}_b - \mathbf{p}_n\|$ (descending, $z_b \ge z_{min}$)
  — distribution / CDF per strategy.
- Paired comparison on common throws: win/tie/loss counts between strategies.
- Control effort $\int \|\mathbf{u}\|^2 dt$ and peak speed per catch.
- Sensitivity: catch rate vs throw speed bin, and vs measurement noise
  $\sigma_z \in \{0.005, 0.02, 0.05, 0.1\}$ m.

---

## Implementation

```python
# ── Parameters ────────────────────────────────────────────────────────────────
ARENA      = (20.0, 20.0, 8.0)   # m  (fastest, steepest throw peaks at 7.1 m)
G          = 9.81
RHO, CD    = 1.225, 0.55
M_BALL, R_BALL = 0.057, 0.033
K_DRAG     = RHO * CD * np.pi * R_BALL**2 / (2 * M_BALL)   # ≈ 0.0202 1/m
V_MAX, A_MAX = 8.0, 10.0
Z_LIM      = (0.3, 4.0)
R_CATCH    = 0.15
NET_OFFSET = np.array([0.0, 0.0, 0.1])
Z_MIN_BALL = 0.5
T_REACT    = 0.15
F_MEAS, SIGMA_Z, T_LAT = 100.0, 0.02, 0.02
PN_N       = 3.0
DT         = 0.002
N_THROWS   = 300

def sample_throw(rng):
    phi = rng.uniform(0, 2*np.pi)
    p0  = np.array([8*np.cos(phi), 8*np.sin(phi), 1.5])
    v0, el = rng.uniform(7, 13), np.radians(rng.uniform(35, 60))
    head = phi + np.pi + np.radians(rng.uniform(-30, 30))
    vb0 = v0 * np.array([np.cos(el)*np.cos(head), np.cos(el)*np.sin(head), np.sin(el)])
    r, th = 4*np.sqrt(rng.uniform()), rng.uniform(0, 2*np.pi)
    pd0 = np.array([r*np.cos(th), r*np.sin(th), 2.0])
    noise_seed = rng.integers(2**31)          # same measurement noise for every strategy
    return p0, vb0, pd0, noise_seed

def run_simulation(strategy, throw):
    # integrate ball (RK4) + drone, EKF on noisy delayed measurements,
    # strategy -> acceleration command, segment-closest-approach catch check,
    # assert speed/accel/altitude limits every step
    ...

def run_monte_carlo(seed=0):
    rng = np.random.default_rng(seed)
    throws = [sample_throw(rng) for _ in range(N_THROWS)]
    return {s: [run_simulation(s, th) for th in throws]
            for s in ('pure_pursuit', 'pn', 'predictive', 'oracle')}
```

---

## Key Parameters

| Parameter | Symbol | Value |
|-----------|--------|-------|
| Arena size | — | $20 \times 20 \times 8$ m |
| Thrower distance from centre | $R_{throw}$ | 8 m |
| Release height | $z_0$ | 1.5 m |
| Throw speed | $v_0$ | U(7, 13) m/s |
| Throw elevation | $\theta_0$ | U(35°, 60°) |
| Azimuth jitter | $\Delta\psi$ | U(−30°, 30°) |
| Ball mass / radius / drag coeff. | $m_b, r_b, C_d$ | 0.057 kg / 0.033 m / 0.55 |
| Drag constant | $k$ | ≈ 0.0202 m⁻¹ |
| Lowest catchable ball height | $z_{min}$ | 0.5 m |
| Drone start | — | uniform in 4 m disk, $z = 2$ m |
| Drone max speed / accel | $v_{max}, a_{max}$ | 8 m/s / 10 m/s² |
| Drone altitude window | — | [0.3, 4.0] m |
| Net catch radius / offset | $r_c$ | 0.15 m / +0.1 m above drone |
| Reaction delay | $t_{react}$ | 0.15 s |
| Measurement rate / noise / latency | $f_z, \sigma_z, \tau_{lat}$ | 100 Hz / 0.02 m / 0.02 s |
| PN constant | $N$ | 3 |
| PCP re-plan rate / time margin | — | 20 Hz / 0.1 s |
| Simulation timestep | $\Delta t$ | 0.002 s |
| Monte Carlo throws | $N_{throws}$ | 300 |

---

## Expected Output

- **3D trajectory plot** of one representative throw: ball path (true and EKF-predicted catch
  point updates), drone paths for PP, PN and PCP, catch/miss markers.
- **Catch-rate bar chart** with 95 % Wilson intervals for PP, PN, PCP and the oracle bound.
- **Miss-distance CDF** per strategy with the $r_c$ line marked.
- **Catch rate vs throw speed** (binned) for each strategy.
- **Noise sensitivity**: catch rate vs $\sigma_z$.
- **Estimator convergence**: predicted catch-point error vs time since launch.
- **Animation (GIF)**: 3D view of the representative throw, all strategies side by side or
  overlaid.

Hypotheses to test (report honestly whether they hold):

1. PCP achieves the highest catch rate and is close to the oracle bound.
2. Pure pursuit trails behind the falling ball and fails on most long throws.
3. PN beats pure pursuit but loses to PCP, because gravity acts as an unmodelled target
   acceleration and PN has no notion of the altitude window.
4. Catch rate degrades gracefully with $\sigma_z$ up to 0.05 m for PCP.

Approximate feasibility check made while writing this card: with straight-line minimum-time
reach from hover and $t_{react} = 0.15$ s, about 92 % of throws drawn from the distribution
above are reachable (600 samples) — the implementation must compute its own oracle bound.

---

## Extensions

1. **Chapman's gaze heuristic**: the strategy human outfielders use — move so that the tangent
   of the ball's elevation angle rises at a constant rate — compared with PCP.
2. **Augmented PN**: add the ball's gravity acceleration term $\tfrac{N}{2}\mathbf{a}_{b,\perp}$.
3. **Soft catch**: require relative speed at contact below 3 m/s to keep the ball in the cup;
   plan a velocity-matching arrival.
4. **Spin and Magnus force** on the ball; estimator with unknown spin.
5. **Two-drone catching**: assign each throw to the drone with the earliest feasible catch
   time, with handover when the prediction changes.
6. **Juggling**: catch and re-launch the ball towards a second drone, as in the ETH
   ball-juggling demonstrations.

---

## Related Scenarios

- Prerequisites: [S001 Basic Intercept](../01_pursuit_evasion/S001_basic_intercept.md) (proportional navigation), [S008 Stochastic Pursuit — Kalman Filter Tracking](../01_pursuit_evasion/S008_stochastic_pursuit.md) (estimation under noise)
- Algorithmic cross-reference: [S023 Moving Landing Pad](../02_logistics_delivery/S023_moving_landing_pad.md) (predictive rendezvous), [S027 Aerial Refueling Relay](../02_logistics_delivery/S027_aerial_refueling_relay.md) (moving-target rendezvous)
- Same domain: [S081 Selfie Follow Mode](S081_selfie_follow.md), [S092 Movie Car Chase Scene](S092_movie_chase.md), [S099 Cross-Obstacle Relay Pass](S099_obstacle_relay.md)
- 3D upgrade: [S094 3D Upgrade — Aerial Ball Catching](3d/S094_3d_ball_catching.md)
