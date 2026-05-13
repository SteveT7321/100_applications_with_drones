# S096 Drone Relay Race

**Domain**: Special Ops & Entertainment | **Difficulty**: ⭐⭐⭐ | **Status**: `[ ]` Not Started
**Algorithm**: KF Rendezvous Prediction + Velocity-Matched Intercept | **Dimension**: 3D

---

## Problem Definition

**Setup**: A 300 m straight-line relay race course is divided into three equal legs of 100 m each.
Three drones — **Drone A**, **Drone B**, and **Drone C** — each carry a virtual baton and are
responsible for one leg. Drone A starts at the origin with the baton and accelerates along the
course; near the 100 m mark it must rendezvous mid-air with Drone B. At the rendezvous the two
drones must fly in parallel at matched velocities for $T_{handoff} = 1$ s within
$d_{handoff} = 0.3$ m of each other — long enough to transfer the baton. Once the handoff
succeeds, Drone B sprints the second leg and passes the baton to Drone C at the 200 m mark via an
identical rendezvous. Drone C then completes the final leg to the finish line at 300 m.

The outgoing drone (recipient) predicts the incoming drone's arrival position using a **Kalman
filter** on the measured position/velocity, then plans a **minimum-snap intercept trajectory** to
arrive at the predicted rendezvous point with matched velocity. A speed optimiser selects each
drone's cruise velocity to minimise total race time $T_{total}$ subject to the rendezvous
constraint being satisfied.

**Roles**:

- **Drone A (Leg 1)**: starts at $(0, 0, 10)$ m with the baton; cruises at $v_A$ m/s; is the
  *incoming* drone at Relay Point 1 near $(100, 0, 10)$ m.
- **Drone B (Leg 2)**: pre-positioned at the relay zone near $(100, 0, 12)$ m; is the *outgoing*
  drone at RP1 and the *incoming* drone at RP2 near $(200, 0, 10)$ m.
- **Drone C (Leg 3)**: pre-positioned at the relay zone near $(200, 0, 12)$ m; is the *outgoing*
  drone at RP2; sprints the final 100 m to the finish at $(300, 0, 10)$ m.
- **Baton**: a logical token transferred to the outgoing drone the moment the rendezvous condition
  is satisfied (position error $\leq d_{handoff}$, velocity error $\leq v_{tol}$) for a
  continuous window of $T_{handoff} = 1$ s.

**Objective**: Complete the race with both handoffs successful. Success criteria:

1. **Position rendezvous**: $\|\mathbf{p}_{in}(t) - \mathbf{p}_{out}(t)\| \leq d_{handoff} = 0.3$ m
   for at least $T_{handoff} = 1$ s continuously at each relay point.
2. **Velocity matching**: $\|\mathbf{v}_{in}(t) - \mathbf{v}_{out}(t)\| \leq v_{tol} = 0.5$ m/s
   simultaneously with the position criterion.
3. **Minimum total race time**: $T_{total} = t_{leg1} + t_{handoff1} + t_{leg2} + t_{handoff2} + t_{leg3}$
   is minimised over drone cruise speeds subject to the above constraints.

**Comparison configurations** (control variants):

1. **Naive handoff** — the outgoing drone begins moving only when it visually detects the incoming
   drone inside $d_{detect} = 5$ m; no KF prediction; arrives late, large position/velocity errors.
2. **KF + Intercept** (proposed) — the outgoing drone uses KF to predict the incoming drone's
   trajectory $\Delta t_{plan}$ ahead and plans a minimum-snap intercept trajectory; achieves
   near-zero position and velocity error at handoff.

---

## Mathematical Model

### Race Geometry

The course is a straight line along the $x$-axis at fixed altitude $z_{race} = 10$ m and
$y = 0$. The two relay points are at nominal $x$-coordinates:

$$x_{RP1} = 100 \text{ m}, \qquad x_{RP2} = 200 \text{ m}$$

The finish line is at $x_{finish} = 300$ m. All positions lie in the $xz$-plane with
$y = 0$; the KF and intercept planner are formulated in full 3D for generality.

### Rendezvous Condition

At each relay point, the handoff window opens at time $t_0$ when both criteria are first
simultaneously satisfied:

$$\left\|\mathbf{p}_{in}(t_0) - \mathbf{p}_{out}(t_0)\right\| \leq d_{handoff}$$

$$\left\|\mathbf{v}_{in}(t_0) - \mathbf{v}_{out}(t_0)\right\| \leq v_{tol}$$

The baton transfer is declared **successful** if both conditions hold continuously for
$T_{handoff} = 1$ s. If the window closes before $T_{handoff}$ elapses, the attempt is a
**failure** and the simulation is flagged.

### Kalman Filter: Incoming Drone State Estimation

The outgoing drone observes noisy measurements of the incoming drone's position at rate
$f_{obs} = 20$ Hz. The KF state vector is:

$$\mathbf{x}_{KF} = \begin{pmatrix} p_x & p_y & p_z & v_x & v_y & v_z \end{pmatrix}^\top
  \in \mathbb{R}^6$$

The constant-velocity prediction model (discrete, timestep $\Delta t_{KF}$):

$$\mathbf{x}_{KF}[k+1] = F\,\mathbf{x}_{KF}[k] + \mathbf{q}[k], \qquad
  F = \begin{pmatrix} I_3 & \Delta t_{KF}\,I_3 \\ 0_3 & I_3 \end{pmatrix}$$

Process noise $\mathbf{q}[k] \sim \mathcal{N}(\mathbf{0}, Q)$ models acceleration uncertainty:

$$Q = \sigma_a^2 \begin{pmatrix}
  \tfrac{\Delta t_{KF}^4}{4}\,I_3 & \tfrac{\Delta t_{KF}^3}{2}\,I_3 \\
  \tfrac{\Delta t_{KF}^3}{2}\,I_3 & \Delta t_{KF}^2\,I_3
\end{pmatrix}, \qquad \sigma_a = 0.5 \text{ m/s}^2$$

Measurement model with observation noise $R = \sigma_{obs}^2 I_3$, $\sigma_{obs} = 0.05$ m:

$$\mathbf{z}[k] = H\,\mathbf{x}_{KF}[k] + \mathbf{r}[k], \qquad
  H = \begin{pmatrix} I_3 & 0_3 \end{pmatrix}, \qquad
  \mathbf{r}[k] \sim \mathcal{N}(\mathbf{0}, R)$$

KF predict–update equations (standard):

$$\hat{\mathbf{x}}^- = F\hat{\mathbf{x}}, \quad P^- = FPF^\top + Q$$

$$K = P^- H^\top (H P^- H^\top + R)^{-1}$$

$$\hat{\mathbf{x}} = \hat{\mathbf{x}}^- + K(\mathbf{z} - H\hat{\mathbf{x}}^-), \quad P = (I - KH)P^-$$

### Intercept Time Selection

The outgoing drone at position $\mathbf{p}_{out}(t)$ uses the KF estimate to predict the
incoming drone's future position $\hat{\mathbf{p}}_{in}(t + \tau)$ for lookahead $\tau > 0$:

$$\hat{\mathbf{p}}_{in}(t + \tau) = \hat{\mathbf{p}}_{in}(t) + \tau\,\hat{\mathbf{v}}_{in}(t)$$

The optimal intercept time $t^*$ is selected as the earliest future moment at which the outgoing
drone can reach $\hat{\mathbf{p}}_{in}(t^*)$ with matched velocity, solving:

$$t^* = \arg\min_{\tau > 0} \left\|\mathbf{p}_{out}(t + \tau) - \hat{\mathbf{p}}_{in}(t + \tau)\right\|
  \leq d_{handoff}$$

subject to the minimum-snap trajectory being physically feasible (thrust and velocity limits). In
practice $t^*$ is found by a 1D grid search over $\tau \in [t_{min}, t_{max}]$ with resolution
$\Delta\tau = 0.1$ s.

### Minimum-Snap Intercept Trajectory

Given the outgoing drone's current state $(\mathbf{p}_0, \mathbf{v}_0)$ and the target rendezvous
state $(\mathbf{p}^*, \mathbf{v}^*)$ to be reached at time $t^*$, the minimum-snap trajectory is
a degree-7 polynomial in each axis. For a single axis with flight time $T_f = t^* - t_{now}$:

$$p(t) = \sum_{k=0}^{7} c_k \left(\frac{t}{T_f}\right)^k, \qquad t \in [0, T_f]$$

Boundary conditions (8 equations, 8 unknowns per axis):

$$p(0) = p_0, \quad \dot{p}(0) = v_0, \quad \ddot{p}(0) = 0, \quad \dddot{p}(0) = 0$$

$$p(T_f) = p^*, \quad \dot{p}(T_f) = v^*, \quad \ddot{p}(T_f) = 0, \quad \dddot{p}(T_f) = 0$$

The coefficients $\mathbf{c} \in \mathbb{R}^8$ are found by solving a linear system $M\mathbf{c} = \mathbf{b}$
constructed from the boundary conditions. Acceleration $\ddot{p}(t) = \sum_{k=2}^{7} k(k-1)c_k t^{k-2}/T_f^k$
is clipped to $a_{max} = 5$ m/s² to respect actuator limits.

### Total Race Time

The total race time decomposes as:

$$T_{total} = t_{leg1} + t_{handoff1} + t_{leg2} + t_{handoff2} + t_{leg3}$$

where $t_{legi}$ is the cruise time for leg $i$ and $t_{handoff j} = T_{handoff} = 1$ s for each
successful baton exchange. Leg times depend on the cruise velocity $v_i$ chosen for each drone:

$$t_{legi} = \frac{L_{leg}}{v_i} = \frac{100}{v_i} \text{ s}$$

The optimum cruise velocity $v_i^*$ is bounded by the rendezvous feasibility constraint — the
outgoing drone must have enough time to reach the intercept point before the incoming drone
passes it:

$$v_i^* = \arg\min_{v_{min} \leq v_i \leq v_{max}} t_{legi} \quad \text{subject to rendezvous feasibility}$$

### Drone Dynamics

Each drone is modelled as a point mass with acceleration-limited motion. The state vector for
drone $j$ is $\mathbf{x}_j = [\mathbf{p}_j^\top,\, \mathbf{v}_j^\top]^\top \in \mathbb{R}^6$:

$$\dot{\mathbf{p}}_j = \mathbf{v}_j, \qquad \dot{\mathbf{v}}_j = \mathbf{u}_j + \mathbf{w}_j$$

where $\mathbf{u}_j$ is the commanded acceleration (from the minimum-snap trajectory or cruise
controller) and $\mathbf{w}_j \sim \mathcal{N}(\mathbf{0}, \sigma_d^2 I_3)$ is a disturbance
with $\sigma_d = 0.1$ m/s² modelling aerodynamic turbulence. The commanded acceleration is
clipped element-wise: $\|\mathbf{u}_j\| \leq a_{max} = 5$ m/s². Euler integration is used with
$\Delta t = 0.02$ s (50 Hz).

---

## Implementation

```python
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

# ── Race course constants ─────────────────────────────────────────────────────
L_LEG        = 100.0    # m  — leg length
Z_RACE       = 10.0     # m  — race altitude
N_LEGS       = 3        # number of relay legs
RELAY_XS     = [0.0, 100.0, 200.0, 300.0]   # start / RP1 / RP2 / finish x-coords

# ── Handoff parameters ────────────────────────────────────────────────────────
D_HANDOFF    = 0.3      # m    — max position gap during handoff
V_TOL        = 0.5      # m/s  — max velocity mismatch during handoff
T_HANDOFF    = 1.0      # s    — required continuous handoff window
D_DETECT     = 5.0      # m    — naive-mode detection range

# ── Drone dynamics ────────────────────────────────────────────────────────────
A_MAX        = 5.0      # m/s² — acceleration limit
V_CRUISE_MIN = 5.0      # m/s  — minimum cruise speed
V_CRUISE_MAX = 20.0     # m/s  — maximum cruise speed
SIGMA_D      = 0.1      # m/s² — disturbance std dev
DT           = 0.02     # s    — simulation timestep

# ── KF parameters ─────────────────────────────────────────────────────────────
SIGMA_A_KF   = 0.5      # m/s² — process noise (acceleration uncertainty)
SIGMA_OBS    = 0.05     # m    — position measurement noise std dev
DT_KF        = DT       # s    — KF update rate matches simulation

# ── Intercept planner ─────────────────────────────────────────────────────────
TAU_MIN      = 0.5      # s    — minimum lookahead for intercept
TAU_MAX      = 10.0     # s    — maximum lookahead for intercept
DTAU         = 0.1      # s    — lookahead grid resolution

# ── Default cruise speeds (one per leg) ──────────────────────────────────────
V_CRUISE     = [12.0, 14.0, 16.0]   # m/s — Drone A, B, C


# ── Kalman filter ─────────────────────────────────────────────────────────────

class KalmanFilter6D:
    """
    6-state (position + velocity) constant-velocity KF for 3D tracking.
    State: [px, py, pz, vx, vy, vz]
    """

    def __init__(self, dt=DT_KF, sigma_a=SIGMA_A_KF, sigma_obs=SIGMA_OBS):
        self.dt = dt
        n = 6
        # Transition matrix
        self.F = np.eye(n)
        self.F[:3, 3:] = dt * np.eye(3)

        # Observation matrix
        self.H = np.zeros((3, n))
        self.H[:, :3] = np.eye(3)

        # Process noise
        dt2 = dt ** 2;  dt3 = dt ** 3;  dt4 = dt ** 4
        Q_block = sigma_a ** 2 * np.block([
            [dt4 / 4 * np.eye(3), dt3 / 2 * np.eye(3)],
            [dt3 / 2 * np.eye(3), dt2       * np.eye(3)],
        ])
        self.Q = Q_block

        # Measurement noise
        self.R = sigma_obs ** 2 * np.eye(3)

        # Initial covariance
        self.P = np.eye(n) * 1.0

        # State estimate (initialised at first measurement)
        self.x = np.zeros(n)
        self.initialised = False

    def init(self, p_meas, v_guess=None):
        """Initialise state from first position measurement."""
        self.x[:3] = p_meas
        self.x[3:] = v_guess if v_guess is not None else np.zeros(3)
        self.initialised = True

    def predict(self):
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, z_meas):
        y = z_meas - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.solve(S.T, np.eye(3)).T
        self.x = self.x + K @ y
        self.P = (np.eye(6) - K @ self.H) @ self.P

    def predict_ahead(self, tau):
        """Predict position and velocity tau seconds into the future."""
        steps = int(round(tau / self.dt))
        x_pred = self.x.copy()
        for _ in range(steps):
            x_pred = self.F @ x_pred
        return x_pred[:3].copy(), x_pred[3:].copy()


# ── Minimum-snap trajectory ───────────────────────────────────────────────────

def minimum_snap_coeffs(p0, v0, pf, vf, Tf):
    """
    Solve for degree-7 minimum-snap polynomial coefficients per axis.
    Boundary conditions: p, v, a, j at t=0 and t=Tf (a=j=0 at endpoints).
    Returns (8,) coefficient array c so that p(t) = sum c[k]*(t/Tf)^k.
    """
    # Normalised time s = t/Tf; boundary conditions in s-domain
    # p(0)=p0, p'(0)/Tf=v0, p''(0)/Tf²=0, p'''(0)/Tf³=0
    # p(1)=pf, p'(1)/Tf=vf, p''(1)/Tf²=0, p'''(1)/Tf³=0
    M = np.zeros((8, 8))
    b = np.zeros(8)

    # Derivatives of s^k at s=0 and s=1
    def poly_row(s, deriv_order, Tf_scale):
        row = np.zeros(8)
        for k in range(deriv_order, 8):
            coeff = 1.0
            for d in range(deriv_order):
                coeff *= (k - d)
            row[k] = coeff * (s ** (k - deriv_order)) / (Tf_scale ** deriv_order)
        return row

    M[0] = poly_row(0, 0, Tf);  b[0] = p0
    M[1] = poly_row(0, 1, Tf);  b[1] = v0
    M[2] = poly_row(0, 2, Tf);  b[2] = 0.0
    M[3] = poly_row(0, 3, Tf);  b[3] = 0.0
    M[4] = poly_row(1, 0, Tf);  b[4] = pf
    M[5] = poly_row(1, 1, Tf);  b[5] = vf
    M[6] = poly_row(1, 2, Tf);  b[6] = 0.0
    M[7] = poly_row(1, 3, Tf);  b[7] = 0.0

    c = np.linalg.solve(M, b)
    return c


def eval_poly(c, t, Tf, deriv=0):
    """Evaluate degree-7 polynomial (or its derivative) at time t."""
    s = t / Tf
    val = 0.0
    for k in range(deriv, 8):
        coeff = 1.0
        for d in range(deriv):
            coeff *= (k - d)
        val += coeff * c[k] * (s ** (k - deriv)) / (Tf ** deriv)
    return val


def plan_intercept(p_out, v_out, p_target, v_target, Tf):
    """
    Plan a minimum-snap trajectory from (p_out, v_out) to (p_target, v_target) in Tf seconds.
    Returns list of coefficient arrays [(cx, cy, cz)] for each axis.
    """
    coeffs = []
    for axis in range(3):
        c = minimum_snap_coeffs(
            p_out[axis], v_out[axis],
            p_target[axis], v_target[axis], Tf
        )
        coeffs.append(c)
    return coeffs


# ── Drone simulation ──────────────────────────────────────────────────────────

class Drone:
    """Single relay drone with point-mass dynamics and state logging."""

    def __init__(self, drone_id, p0, v0, v_cruise, rng):
        self.drone_id  = drone_id
        self.p         = np.array(p0, dtype=float)
        self.v         = np.array(v0, dtype=float)
        self.v_cruise  = v_cruise
        self.rng       = rng
        self.log_p     = [self.p.copy()]
        self.log_v     = [self.v.copy()]
        self.log_t     = [0.0]

    def step(self, u_cmd, t):
        """Euler integrate dynamics with acceleration clip and disturbance."""
        norm_u = np.linalg.norm(u_cmd)
        if norm_u > A_MAX:
            u_cmd = u_cmd / norm_u * A_MAX
        disturbance = self.rng.normal(0.0, SIGMA_D, size=3)
        self.v += (u_cmd + disturbance) * DT
        self.p += self.v * DT
        self.log_p.append(self.p.copy())
        self.log_v.append(self.v.copy())
        self.log_t.append(t)

    def cruise_command(self, x_target):
        """Simple proportional command to maintain cruise speed toward x_target."""
        dx = x_target - self.p[0]
        v_desired = np.array([self.v_cruise, 0.0, 0.0])
        # Altitude hold: proportional on z
        v_desired[2] = -2.0 * (self.p[2] - Z_RACE)
        a_cmd = 5.0 * (v_desired - self.v)
        return a_cmd

    def follow_snap_traj(self, coeffs, t_elapsed, Tf):
        """Track a minimum-snap trajectory at local time t_elapsed."""
        p_ref = np.array([eval_poly(coeffs[ax], t_elapsed, Tf, deriv=0) for ax in range(3)])
        v_ref = np.array([eval_poly(coeffs[ax], t_elapsed, Tf, deriv=1) for ax in range(3)])
        a_ref = np.array([eval_poly(coeffs[ax], t_elapsed, Tf, deriv=2) for ax in range(3)])
        # PD tracking + feedforward
        a_cmd = 8.0 * (p_ref - self.p) + 4.0 * (v_ref - self.v) + a_ref
        return a_cmd

    def arrays(self):
        return (np.array(self.log_t),
                np.array(self.log_p),
                np.array(self.log_v))


# ── Intercept time search ─────────────────────────────────────────────────────

def find_intercept_time(kf, p_out, v_out, tau_min=TAU_MIN, tau_max=TAU_MAX, dtau=DTAU):
    """
    Grid-search the earliest lookahead tau such that a minimum-snap trajectory
    from (p_out, v_out) can reach the predicted incoming position within D_HANDOFF.
    Returns (best_tau, p_target, v_target).
    """
    best_tau  = tau_max
    best_dist = np.inf

    for tau in np.arange(tau_min, tau_max + dtau, dtau):
        p_pred, v_pred = kf.predict_ahead(tau)
        # Estimate distance: rough PD estimate of reachable position at time tau
        # (assume max accel throughout — conservative reachability check)
        dist = np.linalg.norm(p_pred - p_out)
        if dist < best_dist:
            best_dist = dist
            best_tau  = tau
            best_p    = p_pred.copy()
            best_v    = v_pred.copy()

    return best_tau, best_p, best_v


# ── Full relay race simulation ────────────────────────────────────────────────

def run_relay_race(use_kf=True, v_cruise=None, seed=0):
    """
    Simulate the full 3-leg relay race.
    Returns per-drone logs and handoff metrics.
    """
    if v_cruise is None:
        v_cruise = V_CRUISE

    rng = np.random.default_rng(seed)

    # Initialise drones
    drone_A = Drone('A', [RELAY_XS[0], 0.0, Z_RACE], [0.0, 0.0, 0.0], v_cruise[0], rng)
    drone_B = Drone('B', [RELAY_XS[1], 0.0, Z_RACE + 2.0], [0.0, 0.0, 0.0], v_cruise[1], rng)
    drone_C = Drone('C', [RELAY_XS[2], 0.0, Z_RACE + 2.0], [0.0, 0.0, 0.0], v_cruise[2], rng)

    drones = [drone_A, drone_B, drone_C]
    relay_pairs = [(drone_A, drone_B, RELAY_XS[1]),
                   (drone_B, drone_C, RELAY_XS[2])]

    # KF instances for outgoing drones tracking incoming
    kfs = [KalmanFilter6D() for _ in range(2)]

    handoff_metrics = []
    t          = 0.0
    T_MAX      = 120.0

    # Phase tracking:
    # 0 = Leg1 only (A cruising, B/C waiting)
    # 1 = RP1 handoff (A incoming, B intercepting)
    # 2 = Leg2 only (B cruising, C waiting)
    # 3 = RP2 handoff (B incoming, C intercepting)
    # 4 = Leg3 only (C sprinting)
    # 5 = finished
    phase          = 0
    handoff_window = [0.0, 0.0]    # accumulated handoff time per relay
    handoff_done   = [False, False]
    snap_coeffs    = [None, None]   # intercept trajectory coefficients
    intercept_Tf   = [None, None]   # flight times for intercept trajectories
    intercept_t0   = [None, None]   # wall-clock start of intercept
    baton_time     = [None, None]   # wall-clock when baton successfully transferred

    RP_APPROACH  = 15.0   # m before relay x to start KF / intercept planning

    while t < T_MAX and phase < 5:

        # ── Phase 0: Drone A cruises; B and C hold altitude ──────────────────
        if phase == 0:
            drone_A.step(drone_A.cruise_command(RELAY_XS[1]), t)
            drone_B.step(np.array([0.0, 0.0, -2.0 * (drone_B.p[2] - Z_RACE)]), t)
            drone_C.step(np.array([0.0, 0.0, -2.0 * (drone_C.p[2] - Z_RACE)]), t)

            # Begin intercept planning when A enters approach zone
            dist_to_rp1 = RELAY_XS[1] - drone_A.p[0]
            if dist_to_rp1 <= RP_APPROACH:
                # Initialise KF from first noisy measurement of A
                p_meas = drone_A.p + rng.normal(0.0, SIGMA_OBS, size=3)
                kfs[0].init(p_meas, v_guess=drone_A.v.copy())
                phase = 1
                print(f"  [t={t:.2f}s] Phase 0→1: A entered RP1 approach zone "
                      f"(dist={dist_to_rp1:.1f} m)")

        # ── Phase 1: RP1 handoff — B intercepts A ────────────────────────────
        elif phase == 1:
            # KF: measure and update on A's position
            p_meas = drone_A.p + rng.normal(0.0, SIGMA_OBS, size=3)
            kfs[0].predict()
            kfs[0].update(p_meas)

            # Plan / replan intercept trajectory each step
            if use_kf:
                tau_star, p_tgt, v_tgt = find_intercept_time(kfs[0], drone_B.p, drone_B.v)
            else:
                # Naive: only react when A within D_DETECT
                if np.linalg.norm(drone_A.p - drone_B.p) <= D_DETECT:
                    p_tgt = drone_A.p.copy()
                    v_tgt = drone_A.v.copy()
                    tau_star = 0.5
                else:
                    p_tgt, v_tgt, tau_star = None, None, None

            if p_tgt is not None:
                snap_coeffs[0]  = plan_intercept(drone_B.p, drone_B.v, p_tgt, v_tgt, tau_star)
                intercept_Tf[0] = tau_star
                intercept_t0[0] = t

            # Command B along snap trajectory if planned, else hold
            if snap_coeffs[0] is not None and intercept_t0[0] is not None:
                t_local = t - intercept_t0[0]
                Tf      = intercept_Tf[0]
                t_local = min(t_local, Tf)
                u_B = drone_B.follow_snap_traj(snap_coeffs[0], t_local, Tf)
            else:
                u_B = np.array([0.0, 0.0, -2.0 * (drone_B.p[2] - Z_RACE)])

            drone_A.step(drone_A.cruise_command(RELAY_XS[1] + 20), t)
            drone_B.step(u_B, t)
            drone_C.step(np.array([0.0, 0.0, -2.0 * (drone_C.p[2] - Z_RACE)]), t)

            # Check rendezvous condition
            pos_err = np.linalg.norm(drone_A.p - drone_B.p)
            vel_err = np.linalg.norm(drone_A.v - drone_B.v)
            if pos_err <= D_HANDOFF and vel_err <= V_TOL:
                handoff_window[0] += DT
            else:
                handoff_window[0] = 0.0   # reset: must be *continuous*

            if handoff_window[0] >= T_HANDOFF and not handoff_done[0]:
                handoff_done[0] = True
                baton_time[0]   = t
                final_pos_err   = pos_err
                final_vel_err   = vel_err
                handoff_metrics.append({
                    'relay':    1,
                    'success':  True,
                    'pos_err':  final_pos_err,
                    'vel_err':  final_vel_err,
                    'baton_t':  baton_time[0],
                })
                print(f"  [t={t:.2f}s] RP1 HANDOFF SUCCESS  "
                      f"pos_err={pos_err:.3f} m  vel_err={vel_err:.3f} m/s")
                phase = 2

        # ── Phase 2: B cruises Leg 2; C holds ────────────────────────────────
        elif phase == 2:
            drone_A.step(np.zeros(3), t)   # A done — coast to stop
            drone_B.step(drone_B.cruise_command(RELAY_XS[2]), t)
            drone_C.step(np.array([0.0, 0.0, -2.0 * (drone_C.p[2] - Z_RACE)]), t)

            dist_to_rp2 = RELAY_XS[2] - drone_B.p[0]
            if dist_to_rp2 <= RP_APPROACH:
                p_meas = drone_B.p + rng.normal(0.0, SIGMA_OBS, size=3)
                kfs[1].init(p_meas, v_guess=drone_B.v.copy())
                phase = 3
                print(f"  [t={t:.2f}s] Phase 2→3: B entered RP2 approach zone "
                      f"(dist={dist_to_rp2:.1f} m)")

        # ── Phase 3: RP2 handoff — C intercepts B ────────────────────────────
        elif phase == 3:
            p_meas = drone_B.p + rng.normal(0.0, SIGMA_OBS, size=3)
            kfs[1].predict()
            kfs[1].update(p_meas)

            if use_kf:
                tau_star, p_tgt, v_tgt = find_intercept_time(kfs[1], drone_C.p, drone_C.v)
            else:
                if np.linalg.norm(drone_B.p - drone_C.p) <= D_DETECT:
                    p_tgt = drone_B.p.copy()
                    v_tgt = drone_B.v.copy()
                    tau_star = 0.5
                else:
                    p_tgt, v_tgt, tau_star = None, None, None

            if p_tgt is not None:
                snap_coeffs[1]  = plan_intercept(drone_C.p, drone_C.v, p_tgt, v_tgt, tau_star)
                intercept_Tf[1] = tau_star
                intercept_t0[1] = t

            if snap_coeffs[1] is not None and intercept_t0[1] is not None:
                t_local = t - intercept_t0[1]
                Tf      = intercept_Tf[1]
                t_local = min(t_local, Tf)
                u_C = drone_C.follow_snap_traj(snap_coeffs[1], t_local, Tf)
            else:
                u_C = np.array([0.0, 0.0, -2.0 * (drone_C.p[2] - Z_RACE)])

            drone_A.step(np.zeros(3), t)
            drone_B.step(drone_B.cruise_command(RELAY_XS[2] + 20), t)
            drone_C.step(u_C, t)

            pos_err = np.linalg.norm(drone_B.p - drone_C.p)
            vel_err = np.linalg.norm(drone_B.v - drone_C.v)
            if pos_err <= D_HANDOFF and vel_err <= V_TOL:
                handoff_window[1] += DT
            else:
                handoff_window[1] = 0.0

            if handoff_window[1] >= T_HANDOFF and not handoff_done[1]:
                handoff_done[1] = True
                baton_time[1]   = t
                handoff_metrics.append({
                    'relay':    2,
                    'success':  True,
                    'pos_err':  pos_err,
                    'vel_err':  vel_err,
                    'baton_t':  baton_time[1],
                })
                print(f"  [t={t:.2f}s] RP2 HANDOFF SUCCESS  "
                      f"pos_err={pos_err:.3f} m  vel_err={vel_err:.3f} m/s")
                phase = 4

        # ── Phase 4: C sprints Leg 3 to finish ───────────────────────────────
        elif phase == 4:
            drone_A.step(np.zeros(3), t)
            drone_B.step(np.zeros(3), t)
            drone_C.step(drone_C.cruise_command(RELAY_XS[3]), t)

            if drone_C.p[0] >= RELAY_XS[3]:
                phase = 5
                finish_time = t
                print(f"  [t={t:.2f}s] FINISH LINE crossed by Drone C")
                print(f"  Total race time: {finish_time:.2f} s")

        t += DT

    # If simulation exceeded T_MAX without finishing, flag failure
    if phase < 5:
        finish_time = T_MAX
        if len(handoff_metrics) < 2:
            handoff_metrics.append({
                'relay':   2 - len(handoff_metrics),
                'success': False,
                'pos_err': np.nan,
                'vel_err': np.nan,
                'baton_t': np.nan,
            })
        print(f"  WARNING: race did not finish within {T_MAX} s (phase={phase})")

    results = {
        'drones':          [drone_A, drone_B, drone_C],
        'handoff_metrics': handoff_metrics,
        'finish_time':     finish_time,
        'all_done':        phase == 5 and all(handoff_done),
    }
    return results


# ── Plotting ──────────────────────────────────────────────────────────────────

def plot_race_3d(results_kf, results_naive, save_path=None):
    """3D bird's-eye view of all three drone trajectories for both strategies."""
    fig = plt.figure(figsize=(16, 7))
    ax  = fig.add_subplot(111, projection='3d')

    colors  = ['red', 'blue', 'green']
    labels  = ['Drone A (Leg 1)', 'Drone B (Leg 2)', 'Drone C (Leg 3)']
    ls_kf   = '-'
    ls_naive = '--'

    for i, drone in enumerate(results_kf['drones']):
        t_arr, p_arr, _ = drone.arrays()
        ax.plot(p_arr[:, 0], p_arr[:, 1], p_arr[:, 2],
                color=colors[i], ls=ls_kf, lw=1.5,
                label=f'{labels[i]} KF')

    for i, drone in enumerate(results_naive['drones']):
        t_arr, p_arr, _ = drone.arrays()
        ax.plot(p_arr[:, 0], p_arr[:, 1], p_arr[:, 2],
                color=colors[i], ls=ls_naive, lw=1.0, alpha=0.5,
                label=f'{labels[i]} Naive')

    # Relay point markers
    for xrp, label in [(RELAY_XS[1], 'RP1'), (RELAY_XS[2], 'RP2')]:
        ax.scatter([xrp], [0.0], [Z_RACE], c='orange', s=120, zorder=5)
        ax.text(xrp, 0.5, Z_RACE + 0.5, label, fontsize=9, color='darkorange')

    # Start and finish markers
    ax.scatter([RELAY_XS[0]], [0.0], [Z_RACE], c='black', s=80, marker='^', zorder=5, label='Start')
    ax.scatter([RELAY_XS[3]], [0.0], [Z_RACE], c='gold',  s=120, marker='*', zorder=5, label='Finish')

    ax.set_xlabel('x (m)');  ax.set_ylabel('y (m)');  ax.set_zlabel('z (m)')
    ax.set_title('S096 Drone Relay Race — 3D Trajectories (solid=KF, dashed=Naive)')
    ax.legend(fontsize=7, ncol=2, loc='upper left')

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
    plt.show()


def plot_handoff_detail(results_kf, results_naive, relay_idx=0, save_path=None):
    """
    Two-panel plot zooming in on one relay point:
    left — position separation ||p_in - p_out|| vs time;
    right — velocity mismatch ||v_in - v_out|| vs time.
    Both KF and Naive variants overlaid.
    """
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))

    for results, style, label in [(results_kf, '-', 'KF'), (results_naive, '--', 'Naive')]:
        in_drone  = results['drones'][relay_idx]       # incoming (A or B)
        out_drone = results['drones'][relay_idx + 1]   # outgoing (B or C)

        t_in,  p_in,  v_in  = in_drone.arrays()
        t_out, p_out, v_out = out_drone.arrays()

        # Align arrays to the shorter timeline
        n = min(len(t_in), len(t_out))
        t_plot  = t_in[:n]
        pos_sep = np.linalg.norm(p_in[:n] - p_out[:n], axis=1)
        vel_sep = np.linalg.norm(v_in[:n] - v_out[:n], axis=1)

        axes[0].plot(t_plot, pos_sep, ls=style, lw=1.5, label=label)
        axes[1].plot(t_plot, vel_sep, ls=style, lw=1.5, label=label)

    axes[0].axhline(D_HANDOFF, color='k', ls=':', lw=1.5, label=f'$d_{{handoff}}$={D_HANDOFF} m')
    axes[0].fill_between(t_in[:n], 0, D_HANDOFF, alpha=0.1, color='green', label='Success band')
    axes[0].set_xlabel('Time (s)');  axes[0].set_ylabel('Position separation (m)')
    axes[0].set_title(f'RP{relay_idx+1} Position Separation vs Time')
    axes[0].legend(fontsize=9)

    axes[1].axhline(V_TOL, color='k', ls=':', lw=1.5, label=f'$v_{{tol}}$={V_TOL} m/s')
    axes[1].fill_between(t_in[:n], 0, V_TOL, alpha=0.1, color='green', label='Success band')
    axes[1].set_xlabel('Time (s)');  axes[1].set_ylabel('Velocity mismatch (m/s)')
    axes[1].set_title(f'RP{relay_idx+1} Velocity Mismatch vs Time')
    axes[1].legend(fontsize=9)

    plt.suptitle(f'S096 Relay Point {relay_idx+1} Handoff Detail', fontsize=13)
    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
    plt.show()


def animate_relay(results_kf, save_path=None):
    """
    Top-down 2D animation of the relay race (x–y plane).
    Three drone dots with colour-coded trails move along the course.
    Relay point zones shown as orange circles. Baton icon switches between drones.
    """
    drones = results_kf['drones']
    arrays = [d.arrays() for d in drones]

    # Find shortest trajectory length for synchronised animation
    n_min = min(len(a[0]) for a in arrays)
    stride = max(1, n_min // 400)

    fig, ax = plt.subplots(figsize=(14, 5))
    ax.set_xlim(-10, 320)
    ax.set_ylim(-8, 8)
    ax.set_xlabel('x (m)');  ax.set_ylabel('y (m)')
    ax.set_title('S096 Drone Relay Race — Top-Down Animation (KF strategy)')
    ax.set_aspect('equal')
    ax.axhline(0, color='lightgray', lw=0.5)

    # Static course elements
    for xrp, lbl in [(RELAY_XS[1], 'RP1'), (RELAY_XS[2], 'RP2')]:
        circle = plt.Circle((xrp, 0), D_HANDOFF * 5, color='orange', alpha=0.2)
        ax.add_patch(circle)
        ax.axvline(xrp, color='orange', lw=1.0, ls='--', alpha=0.6)
        ax.text(xrp, 5.5, lbl, ha='center', fontsize=9, color='darkorange')
    ax.axvline(RELAY_XS[0], color='black', lw=1.5, ls='-')
    ax.axvline(RELAY_XS[3], color='gold',  lw=2.0, ls='-')
    ax.text(RELAY_XS[0] - 5, 0, 'START', ha='right', fontsize=9, fontweight='bold')
    ax.text(RELAY_XS[3] + 2, 0, 'FINISH', ha='left', fontsize=9, color='goldenrod', fontweight='bold')

    colors = ['red', 'blue', 'green']
    dots   = [ax.plot([], [], 'o', color=c, ms=9, zorder=5)[0] for c in colors]
    trails = [ax.plot([], [], '-', color=c, lw=1.2, alpha=0.5)[0] for c in colors]
    baton_marker = ax.plot([], [], 'D', color='white', ms=6, markeredgecolor='black', zorder=6)[0]
    time_text = ax.text(0.02, 0.92, '', transform=ax.transAxes, fontsize=10)

    TRAIL = 100

    def init():
        for d, tr in zip(dots, trails):
            d.set_data([], []);   tr.set_data([], [])
        baton_marker.set_data([], [])
        time_text.set_text('')
        return dots + trails + [baton_marker, time_text]

    def update(frame):
        k = frame * stride
        k = min(k, n_min - 1)
        t_now = arrays[0][0][k]

        # Determine which drone currently holds the baton
        hm = results_kf['handoff_metrics']
        baton_drone = 0
        for metric in hm:
            if metric['success'] and t_now >= metric['baton_t']:
                baton_drone = metric['relay']

        for i, (arr, dot, trail) in enumerate(zip(arrays, dots, trails)):
            _, p_arr, _ = arr
            idx = min(k, len(p_arr) - 1)
            dot.set_data([p_arr[idx, 0]], [p_arr[idx, 1]])
            lo = max(0, idx - TRAIL)
            trail.set_data(p_arr[lo:idx+1, 0], p_arr[lo:idx+1, 1])

        # Place baton icon on current baton-holder
        _, p_baton, _ = arrays[baton_drone]
        idx_b = min(k, len(p_baton) - 1)
        baton_marker.set_data([p_baton[idx_b, 0]], [p_baton[idx_b, 1] + 0.8])

        time_text.set_text(f't = {t_now:.1f} s  |  Baton: Drone {chr(65+baton_drone)}')
        return dots + trails + [baton_marker, time_text]

    n_frames = n_min // stride
    anim = FuncAnimation(fig, update, frames=n_frames, init_func=init,
                         blit=False, interval=40)
    if save_path:
        anim.save(save_path, writer='pillow', fps=25)
    plt.show()
    return anim


# ── Main ──────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    import os
    OUT_DIR = 'outputs/05_special_entertainment/s096_relay_race'
    os.makedirs(OUT_DIR, exist_ok=True)

    print('=== S096 Drone Relay Race ===')
    print('\n-- KF + Intercept strategy --')
    results_kf    = run_relay_race(use_kf=True,  seed=0)

    print('\n-- Naive (detection-only) strategy --')
    results_naive = run_relay_race(use_kf=False, seed=0)

    # Print summary metrics
    print('\n=== Summary ===')
    for label, res in [('KF+Intercept', results_kf), ('Naive', results_naive)]:
        print(f'\n[{label}]')
        print(f"  Race complete : {res['all_done']}")
        print(f"  Finish time   : {res['finish_time']:.2f} s")
        for hm in res['handoff_metrics']:
            status = 'SUCCESS' if hm['success'] else 'FAIL'
            print(f"  RP{hm['relay']} handoff : {status}  "
                  f"pos_err={hm['pos_err']:.3f} m  vel_err={hm['vel_err']:.3f} m/s")

    # Visualisations
    plot_race_3d(
        results_kf, results_naive,
        save_path=f'{OUT_DIR}/race_3d_trajectories.png'
    )
    plot_handoff_detail(
        results_kf, results_naive, relay_idx=0,
        save_path=f'{OUT_DIR}/handoff_rp1_detail.png'
    )
    plot_handoff_detail(
        results_kf, results_naive, relay_idx=1,
        save_path=f'{OUT_DIR}/handoff_rp2_detail.png'
    )
    animate_relay(
        results_kf,
        save_path=f'{OUT_DIR}/relay_race_animation.gif'
    )
    print(f'\nDone. Outputs saved to {OUT_DIR}/')
```

---

## Key Parameters

| Parameter | Symbol | Value |
|-----------|--------|-------|
| Race course length | $L_{total}$ | 300 m |
| Leg length | $L_{leg}$ | 100 m each |
| Race altitude | $z_{race}$ | 10 m |
| Handoff position tolerance | $d_{handoff}$ | 0.3 m |
| Handoff velocity tolerance | $v_{tol}$ | 0.5 m/s |
| Required handoff window | $T_{handoff}$ | 1 s |
| Naive detection range | $d_{detect}$ | 5 m |
| RP approach zone trigger | — | 15 m before relay $x$ |
| Drone A cruise speed | $v_A$ | 12 m/s |
| Drone B cruise speed | $v_B$ | 14 m/s |
| Drone C cruise speed | $v_C$ | 16 m/s |
| Maximum acceleration | $a_{max}$ | 5 m/s² |
| Dynamics disturbance std dev | $\sigma_d$ | 0.1 m/s² |
| KF process noise (acceleration) | $\sigma_a$ | 0.5 m/s² |
| KF measurement noise | $\sigma_{obs}$ | 0.05 m |
| KF / simulation timestep | $\Delta t$ | 0.02 s (50 Hz) |
| Intercept lookahead range | $[\tau_{min}, \tau_{max}]$ | [0.5, 10] s |
| Intercept grid resolution | $\Delta\tau$ | 0.1 s |
| Minimum-snap polynomial degree | — | 7 |
| Simulation horizon | $T_{max}$ | 120 s |

---

## Expected Output

- **3D trajectory plot** (`Axes3D`): full bird's-eye view of all three drone paths over the 300 m
  course; solid lines for KF+Intercept, dashed for Naive; Drone A in red, B in blue, C in green;
  orange circles mark the relay zone at RP1 (100 m) and RP2 (200 m); start (black triangle) and
  finish (gold star) annotated; legend identifies each drone and strategy.
- **RP1 handoff detail** (2 panels): position separation $\|\mathbf{p}_{in} - \mathbf{p}_{out}\|$
  and velocity mismatch $\|\mathbf{v}_{in} - \mathbf{v}_{out}\|$ vs time for both strategies;
  green shaded bands indicate the success region ($< d_{handoff}$ and $< v_{tol}$); KF strategy
  expected to enter and sustain the success band cleanly, Naive strategy expected to show
  oscillation or late entry.
- **RP2 handoff detail**: same two-panel format for the second relay point.
- **Top-down relay race animation** (GIF): 14 × 5 inch figure; three coloured drone dots with
  fading trails move from left to right; orange relay-zone circles at RP1 and RP2; a white diamond
  baton icon rides on the current baton-holder and jumps to the next drone at successful handoff;
  live time and baton-holder label in upper-left; finish line in gold; 25 fps.
- **Console metrics** (printed): for each strategy, whether the race completed, total finish time,
  and per-relay handoff status (SUCCESS/FAIL), position error, and velocity error at the moment of
  transfer.

**Expected metric targets** (KF+Intercept, seed 0):

| Metric | Target |
|--------|--------|
| RP1 handoff success | YES |
| RP2 handoff success | YES |
| RP1 position error at handoff | $\leq 0.3$ m |
| RP2 position error at handoff | $\leq 0.3$ m |
| RP1 velocity mismatch at handoff | $\leq 0.5$ m/s |
| RP2 velocity mismatch at handoff | $\leq 0.5$ m/s |
| Total race time | $\leq 60$ s |
| Naive handoff success (at least one failure) | At least 1 FAIL or large error |

---

## Extensions

1. **Speed optimisation**: implement a grid or gradient search over $(v_A, v_B, v_C)$ subject to
   the rendezvous feasibility constraint; report the Pareto frontier of total race time vs handoff
   margin; compare the optimal speed profile to the fixed-speed baseline used here.
2. **Obstacle-avoidance leg**: introduce static cylindrical obstacles (radius 1 m) scattered along
   one leg; replant the cruise trajectory using RRT* while preserving the handoff timing budget;
   measure the time penalty versus the obstacle-free race.
3. **Wind disturbance**: add a constant crosswind $w_y = 2$ m/s along the full course; quantify
   the lateral drift at the relay points and evaluate whether the KF intercept planner adapts
   without modification or requires a wind-state augmentation.
4. **Four-drone relay**: extend to $N = 4$ legs of 75 m each with three relay points; study how
   the handoff timing errors cascade — does a late RP1 transfer push RP2 and RP3 out of the
   feasible window? Propose a re-timing protocol.
5. **Real-time replanning**: instead of a one-shot intercept plan computed at the approach zone,
   continuously replan the intercept trajectory every 0.1 s using the latest KF estimate; compare
   final handoff error against the single-plan version to quantify the benefit of online
   replanning under disturbance.

---

## Related Scenarios

- Prerequisites: [S082 FPV Racing](S082_fpv_racing.md) (high-speed drone navigation along a
  course), [S095 Aerial Acrobatics](S095_aerial_acrobatics.md) (precise trajectory following under
  dynamic constraints)
- Algorithmic cross-reference: [S081 Fireworks Synchronisation](S081_fireworks_sync.md) (KF state
  estimation and timing, same filter architecture used here),
  [S023 Moving Landing Pad](../../02_logistics_delivery/S023_moving_landing_pad.md) (velocity
  matching at a moving target, analogous to the handoff rendezvous condition)
- Follow-ups: [S099 Multi-Drone Grand Prix](S099_grand_prix.md) (full race with multiple
  competing relay teams and collision avoidance)
