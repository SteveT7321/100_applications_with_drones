"""
S094 Aerial Ball Catching
=========================
A thrower on the edge of an indoor arena throws a tennis ball (quadratic drag) and a
drone with a net cup on top must catch it while it descends, before it falls below
z_min = 0.5 m. Motion capture reports the ball position at 100 Hz with Gaussian noise
and 20 ms latency; an EKF estimates the ball state. Three causal strategies are compared
on identical throws AND identical measurement-noise realisations (common random numbers):

  1. Pure pursuit (PP)            - fly at full speed at the current estimated ball position
  2. Proportional navigation (PN) - 3D PN, N = 3, pure-pursuit fallback when V_c <= 0
  3. Predictive catch point (PCP) - propagate the EKF estimate with the drag model, pick the
                                    earliest reachable descending point, re-plan at 20 Hz

Two non-causal references bound what is possible:
  * Oracle planner  - PCP with the true trajectory known at launch (card's oracle)
  * Reachability bound - a straight-line minimum-time relaxation that is a strict upper
                         bound for ANY strategy (checked: every catch must lie inside it)

The Monte Carlo is vectorised across throws; every catch check uses the closest approach
of the ball and net straight-line segments inside each 2 ms step.

Usage:
    conda activate drones
    python src/05_special_entertainment/s094_ball_catching.py
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401
from scipy.stats import binomtest

# ── Parameters (card) ─────────────────────────────────────────────────────────
ARENA      = (20.0, 20.0, 8.0)   # m (card said 6 m high; see README deviations)
G          = 9.81
RHO, CD    = 1.225, 0.55
M_BALL, R_BALL = 0.057, 0.033
K_DRAG     = RHO * CD * np.pi * R_BALL**2 / (2 * M_BALL)   # ~0.0202 1/m
V_MAX, A_MAX = 8.0, 10.0
Z_LO, Z_HI = 0.3, 4.0            # drone altitude window (m)
R_CATCH    = 0.15
NET_OFFSET = np.array([0.0, 0.0, 0.1])
Z_MIN_BALL = 0.5
T_REACT    = 0.15
F_MEAS, SIGMA_Z, T_LAT = 100.0, 0.02, 0.02
PN_N       = 3.0
DT         = 0.002
N_THROWS   = 300
R_THROW, Z_RELEASE = 8.0, 1.5
V0_RANGE, EL_RANGE, JITTER_DEG = (7.0, 13.0), (35.0, 60.0), 30.0
R_START, Z_START = 4.0, 2.0
REPLAN_HZ  = 20.0
T_MARGIN   = 0.1
H_CAND     = 0.02                # candidate catch-time spacing (s)
CAND_Z     = (0.5, 4.0)          # candidate ball-height window (card)
SIGMA_SWEEP = [0.005, 0.02, 0.05, 0.1]

# ── Parameters (added by this implementation; documented in README) ───────────
K_V        = 8.0                 # 1/s  velocity-loop gain (PP and PCP tracker)
A_BRAKE    = 7.0                 # m/s^2 braking-curve deceleration of the PCP tracker
K_POS      = 8.0                 # 1/s  linear final-approach gain of the PCP tracker
A_GUARD    = 6.0                 # m/s^2 altitude-barrier deceleration (all strategies)
Q_ACC      = 0.1                 # m^2/s^3 EKF process-noise density on acceleration
T_HORIZON  = 2.5                 # s    PCP prediction horizon
T_TRUTH    = 3.0 + T_HORIZON     # s    length of the stored true trajectory
BOUND_SLACK = 0.10               # m    extra slack in the reachability bound (discretisation)
MC_SEEDS   = [0, 1, 2, 3, 4]     # seed 0 = main run; 1-4 = replication
SPEED_BINS = [7.0, 9.0, 11.0, 13.0]
GIF_STEP   = 10                  # sim steps per GIF frame (0.02 s)

MEAS_EVERY   = int(round(1.0 / (F_MEAS * DT)))      # 5 steps
LAT_STEPS    = int(round(T_LAT / DT))               # 10 steps
REACT_STEP   = int(round(T_REACT / DT))             # 75
REPLAN_EVERY = int(round(1.0 / (REPLAN_HZ * DT)))   # 25 steps
CAND_STEPS   = int(round(H_CAND / DT))              # 10 steps
N_CAND       = int(round(T_HORIZON / H_CAND))       # 125
N_TRUTH      = int(round(T_TRUTH / DT)) + 1
T_MEAS       = 1.0 / F_MEAS

STRATS = ['pp', 'pn', 'pcp', 'oracle']
CAUSAL = ['pp', 'pn', 'pcp']
LABEL = {'pp': 'Pure pursuit', 'pn': 'Prop. navigation', 'pcp': 'Predictive catch pt',
         'oracle': 'Oracle planner'}
COLOR = {'pp': '#ff9896', 'pn': '#d62728', 'pcp': '#67000d', 'oracle': '#444444'}
BALL_C, TARGET_C = '#1f77b4', '#2ca02c'

OUT_DIR = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..',
                                        'outputs', '05_special_entertainment',
                                        's094_ball_catching'))
I3 = np.eye(3)


# ── Ball model ────────────────────────────────────────────────────────────────
def ball_deriv(x, k=K_DRAG):
    v = x[..., 3:]
    s = np.linalg.norm(v, axis=-1, keepdims=True)
    a = -k * s * v
    a[..., 2] -= G
    return np.concatenate([v, a], axis=-1)


def rk4(x, h, k=K_DRAG):
    k1 = ball_deriv(x, k)
    k2 = ball_deriv(x + 0.5 * h * k1, k)
    k3 = ball_deriv(x + 0.5 * h * k2, k)
    k4 = ball_deriv(x + h * k3, k)
    return x + h / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)


def sample_throw(rng):
    """Card's sampler (same draw order)."""
    phi = rng.uniform(0, 2 * np.pi)
    p0 = np.array([R_THROW * np.cos(phi), R_THROW * np.sin(phi), Z_RELEASE])
    v0, el = rng.uniform(*V0_RANGE), np.radians(rng.uniform(*EL_RANGE))
    head = phi + np.pi + np.radians(rng.uniform(-JITTER_DEG, JITTER_DEG))
    vb0 = v0 * np.array([np.cos(el) * np.cos(head), np.cos(el) * np.sin(head), np.sin(el)])
    r, th = R_START * np.sqrt(rng.uniform()), rng.uniform(0, 2 * np.pi)
    pd0 = np.array([r * np.cos(th), r * np.sin(th), Z_START])
    noise_seed = rng.integers(2**31)          # same measurement noise for every strategy
    return p0, vb0, pd0, noise_seed, v0


def make_throws(seed, n=N_THROWS):
    rng = np.random.default_rng(seed)
    th = [sample_throw(rng) for _ in range(n)]
    p0 = np.array([t[0] for t in th])
    vb0 = np.array([t[1] for t in th])
    pd0 = np.array([t[2] for t in th])
    v0 = np.array([t[4] for t in th])
    n_meas = N_TRUTH // MEAS_EVERY + 2
    noise = np.array([np.random.default_rng(int(t[3])).standard_normal((n_meas, 3))
                      for t in th])
    # true trajectory, RK4 at DT
    x = np.concatenate([p0, vb0], axis=1)
    truth = np.empty((n, N_TRUTH, 6))
    truth[:, 0] = x
    for i in range(1, N_TRUTH):
        x = rk4(x, DT)
        truth[:, i] = x
    below = (truth[:, :, 2] < Z_MIN_BALL) & (truth[:, :, 5] < 0)
    assert below.any(axis=1).all(), 'some ball does not fall below z_min in T_TRUTH'
    n_end = np.argmax(below, axis=1)          # first sample below z_min (descending)
    return dict(p0=p0, vb0=vb0, pd0=pd0, v0=v0, noise=noise, truth=truth, n_end=n_end)


# ── Drone helpers ─────────────────────────────────────────────────────────────
def _norm(a):
    return np.linalg.norm(a, axis=-1)


def sat(u, lim):
    n = _norm(u)[..., None]
    return np.where(n > lim, u * (lim / np.maximum(n, 1e-12)), u)


def altitude_guard(pd, vd, u):
    """Velocity barrier: keep |v_z| below sqrt(2 A_GUARD d) towards each altitude limit,
    then rescale the horizontal command so that |u| <= A_MAX still holds."""
    d_lo = np.maximum(pd[:, 2] - Z_LO, 0.0)
    d_hi = np.maximum(Z_HI - pd[:, 2], 0.0)
    uz_min = (-np.sqrt(2 * A_GUARD * d_lo) - vd[:, 2]) / DT
    uz_max = (np.sqrt(2 * A_GUARD * d_hi) - vd[:, 2]) / DT
    uz = np.clip(np.clip(u[:, 2], uz_min, uz_max), -A_MAX, A_MAX)
    uh = u[:, :2]
    hn = _norm(uh)
    hmax = np.sqrt(np.maximum(A_MAX**2 - uz**2, 0.0))
    scale = np.where(hn > hmax, hmax / np.maximum(hn, 1e-12), 1.0)
    return np.column_stack([uh * scale[:, None], uz])


def goto_accel(pd, vd, q):
    """PCP tracker: braking-curve go-to (arrive as early as possible and hold)."""
    e = q - pd
    D = _norm(e)
    d_hat = e / np.maximum(D, 1e-9)[:, None]
    sq = np.sqrt(2 * A_BRAKE * D)
    speed = np.minimum(V_MAX, np.minimum(sq, K_POS * D))
    v_des = d_hat * speed[:, None]
    on_curve = (sq < V_MAX) & (sq <= K_POS * D)
    v_par = np.einsum('ij,ij->i', vd, d_hat)
    ff = np.where(on_curve, -A_BRAKE * np.clip(v_par / np.maximum(speed, 1e-9), 0, 1.5), 0.0)
    return sat(K_V * (v_des - vd) + ff[:, None] * d_hat, A_MAX)


def _t_r2r(D, a_acc, a_dec, vmax):
    vp = np.sqrt(2 * a_acc * a_dec * D / (a_acc + a_dec))
    t_tri = vp / a_acc + vp / a_dec
    t_trap = vmax / a_acc + vmax / a_dec + (D - vmax**2 / (2 * a_acc) - vmax**2 / (2 * a_dec)) / vmax
    return np.where(vp > vmax, t_trap, t_tri)


def reach_time_1d(D, u0, a_acc=A_MAX, a_dec=A_BRAKE, vmax=V_MAX):
    """Minimum time to move D >= 0 along a line and stop, from speed u0 along the line,
    accelerating at a_acc and braking at a_dec (bang-coast-bang)."""
    u0 = np.minimum(u0, vmax)
    # (A) moving away: stop first, then rest-to-rest over the longer distance
    tA = -u0 / a_acc + _t_r2r(D + u0**2 / (2 * a_acc), a_acc, a_dec, vmax)
    # (B) too fast to stop: overshoot, stop, come back
    stop = u0**2 / (2 * a_dec)
    tB = u0 / a_dec + _t_r2r(np.maximum(stop - D, 0.0), a_acc, a_dec, vmax)
    # (C) accelerate to v_peak (maybe cruise), then brake
    vp = np.sqrt((2 * a_acc * a_dec * D + a_dec * u0**2) / (a_acc + a_dec))
    tC_tri = (vp - u0) / a_acc + vp / a_dec
    d_cr = D - (vmax**2 - u0**2) / (2 * a_acc) - vmax**2 / (2 * a_dec)
    tC_trap = (vmax - u0) / a_acc + vmax / a_dec + np.maximum(d_cr, 0.0) / vmax
    tC = np.where(vp > vmax, tC_trap, tC_tri)
    return np.where(u0 < 0, tA, np.where(stop > D, tB, tC))


def reach_time_free_1d(D, u0, a=A_MAX, vmax=V_MAX):
    """Minimum time to move D >= 0 along a line from speed u0, free arrival speed."""
    u0 = np.minimum(u0, vmax)
    d_acc = (vmax**2 - u0**2) / (2 * a)      # distance to reach vmax (u0 < 0 included)
    t_acc = (np.sqrt(np.maximum(u0**2 + 2 * a * D, 0.0)) - u0) / a
    t_cr = (vmax - u0) / a + (D - d_acc) / vmax
    return np.where(D <= d_acc, t_acc, t_cr)


def reach_time(pd, vd, q, free_end=False):
    """PCP reach-time model: 1D bang-coast-bang along the line of sight using the velocity
    component along it, plus |v_perp| / A_MAX to cancel the perpendicular velocity.
    free_end=False: arrive and stop (hold); free_end=True: fly through the point."""
    e = q - pd
    D = _norm(e)
    d_hat = e / np.maximum(D, 1e-9)[..., None]
    v_par = np.einsum('...j,...j->...', np.broadcast_to(vd, e.shape), d_hat)
    v_perp = _norm(np.broadcast_to(vd, e.shape) - v_par[..., None] * d_hat)
    t_line = reach_time_free_1d(D, v_par) if free_end else reach_time_1d(D, v_par)
    return t_line + v_perp / A_MAX


def intercept_accel(pd, vd, q, t_go):
    """PCP tracker, intercept mode: fly the straight line to q at the constant speed
    D / t_go that arrives at the planned catch time (capped at V_MAX)."""
    e = q - pd
    D = _norm(e)
    d_hat = e / np.maximum(D, 1e-9)[:, None]
    speed = np.minimum(V_MAX, D / np.maximum(t_go, DT))
    return sat(K_V * (d_hat * speed[:, None] - vd), A_MAX)


def t_lower_bound(D):
    """Minimum time to cover D from rest with |a| <= A_MAX, |v| <= V_MAX (free end speed)."""
    d1 = V_MAX**2 / (2 * A_MAX)
    return np.where(D <= d1, np.sqrt(2 * np.maximum(D, 0) / A_MAX), V_MAX / A_MAX + (D - d1) / V_MAX)


# ── EKF ───────────────────────────────────────────────────────────────────────
def drag_jacobian(v):
    s = np.maximum(_norm(v), 1e-6)
    return -K_DRAG * (s[:, None, None] * I3 + v[:, :, None] * v[:, None, :] / s[:, None, None])


def ekf_predict(x, P, h):
    B = x.shape[0]
    A = np.zeros((B, 6, 6))
    A[:, :3, 3:] = I3
    A[:, 3:, 3:] = drag_jacobian(x[:, 3:])
    Ah = A * h
    F = np.eye(6) + Ah + Ah @ Ah / 2.0
    Q = Q_ACC * np.block([[h**3 / 3 * I3, h**2 / 2 * I3], [h**2 / 2 * I3, h * I3]])
    return rk4(x, h), F @ P @ np.transpose(F, (0, 2, 1)) + Q


def ekf_update(x, P, z, sigma):
    R = sigma**2 * I3
    S = P[:, :3, :3] + R
    K = np.transpose(np.linalg.solve(S, np.transpose(P[:, :, :3], (0, 2, 1))), (0, 2, 1))
    x = x + np.einsum('bij,bj->bi', K, z - x[:, :3])
    IKH = np.eye(6) - np.concatenate([K, np.zeros_like(K)], axis=2)
    P = IKH @ P @ np.transpose(IKH, (0, 2, 1)) + K @ R @ np.transpose(K, (0, 2, 1))
    return x, P


# ── PCP planner ───────────────────────────────────────────────────────────────
def plan_catch(xb, pd, vd, n_now, truth=None, k_pred=K_DRAG, hold_only=False):
    """Return drone target (B,3), predicted ball catch point (B,3), time-to-go (B,),
    catch index offset (B,), has_valid (B,), has_feasible (B,)."""
    B = xb.shape[0]
    if truth is None:
        traj = np.empty((B, N_CAND, 6))
        x = xb
        for j in range(N_CAND):
            x = rk4(x, H_CAND, k_pred)
            traj[:, j] = x
    else:
        idx = np.minimum(n_now + (np.arange(N_CAND) + 1) * CAND_STEPS, N_TRUTH - 1)
        traj = truth[:, idx]
    tgo = (np.arange(N_CAND) + 1) * H_CAND
    z, vz = traj[..., 2], traj[..., 5]
    fallen = np.logical_or.accumulate((z < Z_MIN_BALL) & (vz < 0), axis=1)
    valid = (z >= CAND_Z[0]) & (z <= CAND_Z[1]) & (vz < 0) & ~fallen
    q = traj[..., :3] - NET_OFFSET
    T_stop = reach_time(pd[:, None, :], vd[:, None, :], q)
    T_free = reach_time(pd[:, None, :], vd[:, None, :], q, free_end=True)
    feas_hold = valid & (T_stop + T_MARGIN <= tgo)
    feas_free = valid & (T_free + T_MARGIN <= tgo)
    has_hold = feas_hold.any(axis=1)
    has_free = feas_free.any(axis=1)
    has_valid = valid.any(axis=1)
    j_hold = np.argmax(feas_hold, axis=1)
    j_free = np.argmax(feas_free, axis=1)
    j_best = np.argmin(np.where(valid, T_free - tgo, np.inf), axis=1)
    j = np.where(has_hold, j_hold, np.where(has_free, j_free, j_best))
    mode = np.where(has_hold, 0, np.where(has_free, 1, 2))    # 0 hold, 1 intercept, 2 late
    if hold_only:   # ablation: arrive-and-stop plans only (least-late stop plan as fallback)
        j = np.where(has_hold, j_hold, np.argmin(np.where(valid, T_stop - tgo, np.inf), axis=1))
        mode = np.zeros(B, int)
    b = np.arange(B)
    return q[b, j], traj[b, j, :3], tgo[j], j, has_valid, mode


# ── Ball-state estimator run (shared by all strategies: identical estimates = CRN) ──
def run_ekf(th, sigma):
    """EKF on the delayed noisy measurements. Returns the current-time estimate for every
    step (n_max, B, 6) and the first step at which it is available. The estimate does not
    depend on the drone, so one run serves every strategy with the same noise draws."""
    truth, n_end, noise = th['truth'], th['n_end'], th['noise']
    B = truth.shape[0]
    n_max = int(n_end.max())
    hist = np.full((n_max, B, 6), np.nan)
    xf = np.zeros((B, 6)); P = np.zeros((B, 6, 6)); xc = np.zeros((B, 6))
    z_first = None
    ready = None
    for n in range(n_max):
        if n % MEAS_EVERY == 0 and n >= LAT_STEPS:
            m = (n - LAT_STEPS) // MEAS_EVERY
            z = truth[:, n - LAT_STEPS, :3] + sigma * noise[:, m]
            if m == 0:
                z_first = z
            else:
                if m == 1:      # two-point initialisation
                    xf = np.concatenate([z, (z - z_first) / T_MEAS
                                         + np.array([0, 0, -G * T_MEAS / 2])], axis=1)
                    s2 = sigma**2
                    P = np.zeros((B, 6, 6))
                    P[:, :3, :3] = s2 * I3
                    P[:, :3, 3:] = P[:, 3:, :3] = s2 / T_MEAS * I3
                    P[:, 3:, 3:] = (2 * s2 / T_MEAS**2 + (G * T_MEAS)**2) * I3
                    ready = n
                else:
                    xf, P = ekf_predict(xf, P, T_MEAS)
                    xf, P = ekf_update(xf, P, z, sigma)
                xc = rk4(rk4(xf, T_LAT / 2), T_LAT / 2)     # compensate latency
        elif ready is not None:
            xc = rk4(xc, DT)
        if ready is not None:
            hist[n] = xc
    assert ready is not None and ready < REACT_STEP
    return dict(hist=hist, ready=ready, sigma=sigma)


# ── Closed-loop simulation (vectorised over throws) ───────────────────────────
def simulate(strategy, th, est=None, record=False):
    """est: output of run_ekf (ignored by the oracle strategies, which use the truth)."""
    truth, n_end, pd0 = th['truth'], th['n_end'], th['pd0']
    B = len(pd0)
    pd, vd = pd0.copy(), np.zeros((B, 3))
    active = np.ones(B, bool)
    caught = np.zeros(B, bool)
    t_catch = np.full(B, np.nan)
    d_miss = np.full(B, np.inf)
    effort = np.zeros(B)
    v_peak = np.zeros(B)
    at_limit = np.zeros(B, bool)
    target = pd0.copy()
    t_tgt = np.full(B, np.inf)
    mode = np.zeros(B, int)
    mode_first = np.full(B, -1)
    v_rel_catch = np.full(B, np.nan)
    rec = dict(pd=[pd.copy()], plans=[], est_err=[]) if record else None
    n_max = int(n_end.max())
    oracle = strategy.startswith('oracle')
    for n in range(n_max):
        t = n * DT
        # 1) ball state: EKF current-time estimate (causal) or truth (oracle)
        xb = truth[:, n] if oracle else est['hist'][n]
        ekf_ready = (not oracle) and n >= est['ready']
        if record and ekf_ready and not oracle:
            rec['est_err'].append((t, _norm(xb[:, :3] - truth[:, n, :3]),
                                   _norm(xb[:, 3:] - truth[:, n, 3:]), active.copy()))
        # 2) strategy -> acceleration command
        pn_ = pd + NET_OFFSET
        if n < REACT_STEP:
            u = np.zeros((B, 3))
        elif strategy == 'pp':
            r = xb[:, :3] - pn_
            v_des = V_MAX * r / np.maximum(_norm(r), 1e-9)[:, None]
            u = sat(K_V * (v_des - vd), A_MAX)
        elif strategy == 'pn':
            r = xb[:, :3] - pn_
            vr = xb[:, 3:] - vd
            rn = np.maximum(_norm(r), 1e-9)
            r_hat = r / rn[:, None]
            omega = np.cross(r, vr) / (rn**2)[:, None]
            vc = -np.einsum('ij,ij->i', r, vr) / rn
            u_pn = PN_N * vc[:, None] * np.cross(omega, r_hat)
            u_pp = K_V * (V_MAX * r_hat - vd)
            u = sat(np.where((vc > 0)[:, None], u_pn, u_pp), A_MAX)
        else:   # pcp / oracle
            if (n - REACT_STEP) % REPLAN_EVERY == 0:
                q, cpt, tgo, j, hv, md = plan_catch(xb, pd, vd, n,
                                                    truth if oracle else None,
                                                    hold_only=strategy.endswith('_hold'))
                target = np.where(hv[:, None], q, target)
                t_tgt = np.where(hv, t + tgo, t_tgt)
                mode = np.where(hv, md, mode)
                if n == REACT_STEP:
                    mode_first = np.where(hv, md, -1)
                if record:
                    plan = dict(t=t, ball_pt=cpt, tgo=tgo, valid=hv & active, mode=md,
                                err=_norm(cpt - truth[np.arange(B), n + (j + 1) * CAND_STEPS, :3]))
                    if strategy == 'pcp':
                        _, cpt0, _, j0, hv0, _ = plan_catch(xb, pd, vd, n, None, k_pred=0.0)
                        plan['err_nodrag'] = _norm(
                            cpt0 - truth[np.arange(B), n + (j0 + 1) * CAND_STEPS, :3])
                        plan['valid_nodrag'] = hv0 & active
                    rec['plans'].append(plan)
            u = np.where((mode == 0)[:, None], goto_accel(pd, vd, target),
                         intercept_accel(pd, vd, target, t_tgt - t))
        u = altitude_guard(pd, vd, u)
        # 3) drone integration with speed saturation (semi-implicit Euler)
        vd_new = sat(vd + u * DT, V_MAX)
        u_app = (vd_new - vd) / DT
        pd_new = pd + vd_new * DT
        a_ = active[:, None]
        vd_new = np.where(a_, vd_new, vd)
        pd_new = np.where(a_, pd_new, pd)
        u_app = np.where(a_, u_app, 0.0)
        # invariants (X4)
        assert np.all(_norm(vd_new) <= V_MAX * (1 + 1e-9)), 'speed limit violated'
        assert np.all(_norm(u_app) <= A_MAX * (1 + 1e-6)), 'acceleration limit violated'
        assert np.all(pd_new[:, 2] >= Z_LO - 1e-3) and np.all(pd_new[:, 2] <= Z_HI + 1e-3), \
            'altitude window violated'
        assert np.all(np.isfinite(pd_new)), 'non-finite state'
        # 4) catch check: closest approach of ball and net segments in [t_n, t_n+1]
        b0, b1 = truth[:, n, :3], truth[:, n + 1, :3]
        n0, n1 = pd + NET_OFFSET, pd_new + NET_OFFSET
        r0 = b0 - n0
        dr = (b1 - n1) - r0
        s = np.clip(-np.einsum('ij,ij->i', r0, dr) / np.maximum(np.einsum('ij,ij->i', dr, dr),
                                                               1e-18), 0.0, 1.0)
        dist = _norm(r0 + s[:, None] * dr)
        zb_s = b0[:, 2] + s * (b1[:, 2] - b0[:, 2])
        vz_s = truth[:, n, 5] + s * (truth[:, n + 1, 5] - truth[:, n, 5])
        ok = active & (vz_s < 0) & (zb_s >= Z_MIN_BALL)
        d_miss = np.where(ok, np.minimum(d_miss, dist), d_miss)
        new_c = ok & (dist <= R_CATCH) & ~caught
        pre = active & ~caught            # effort / speed / limits counted until the catch
        caught |= new_c
        t_catch = np.where(new_c, t + s * DT, t_catch)
        v_rel_catch = np.where(new_c, _norm(truth[:, n, 3:] - vd_new), v_rel_catch)
        effort += np.where(pre, np.sum(u_app**2, axis=1) * DT, 0.0)
        v_peak = np.maximum(v_peak, np.where(pre, _norm(vd_new), 0.0))
        at_limit |= pre & ((pd_new[:, 2] < Z_LO + 0.01) | (pd_new[:, 2] > Z_HI - 0.01))
        pd, vd = pd_new, vd_new
        # the flight (and the closest-approach metric d_miss) continues after a catch, as if
        # the ball passed through the cup, so that d_miss is not fixed at r_c for catches
        active &= (n + 1 < n_end)
        if record:
            rec['pd'].append(pd.copy())
        if not active.any():
            break
    assert not active.any()
    assert np.all(d_miss[caught] <= R_CATCH)
    out = dict(caught=caught, t_catch=t_catch, d_miss=d_miss, effort=effort, v_peak=v_peak,
               at_limit=at_limit, mode_first=mode_first,
               v_rel_catch=v_rel_catch)
    if record:
        rec['pd'] = np.array(rec['pd'])
        out['rec'] = rec
    return out


def reachability_bound(th):
    """Strict upper bound for any strategy: some descending ball sample with
    z in [Z_MIN_BALL - 0.05, Z_HI + 0.1 + R_CATCH] is within (straight-line, from-rest,
    max-accel, free-end-speed) reach after T_REACT, with slack BOUND_SLACK."""
    truth, n_end = th['truth'], th['n_end']
    B = truth.shape[0]
    n_idx = np.arange(N_TRUTH)
    t = n_idx * DT
    zb, vz = truth[:, :, 2], truth[:, :, 5]
    in_flight = n_idx[None, :] <= n_end[:, None]
    win = (zb >= Z_MIN_BALL - 0.05) & (zb <= Z_HI + NET_OFFSET[2] + R_CATCH) & (vz < 0) & in_flight
    D = _norm(truth[:, :, :3] - (th['pd0'] + NET_OFFSET)[:, None, :])
    D_eff = np.maximum(D - R_CATCH - BOUND_SLACK, 0.0)
    ok = win & (t_lower_bound(D_eff) <= np.maximum(t - T_REACT, 0.0)[None, :])
    reach = ok.any(axis=1)
    # card's rough estimate: straight-line from-rest reach of the ball point at z_min,
    # no catch radius, no slack, landing point only
    b_land = truth[np.arange(B), n_end - 1, :3]
    d_land = _norm(b_land - (th['pd0'] + NET_OFFSET))
    card_like = t_lower_bound(d_land) <= (n_end - 1) * DT - T_REACT
    return reach, card_like


def wilson(c, n, z=1.96):
    p = c / n
    den = 1 + z**2 / n
    ctr = (p + z**2 / (2 * n)) / den
    hw = z * np.sqrt(p * (1 - p) / n + z**2 / (4 * n**2)) / den
    return ctr - hw, ctr + hw


def validate_reach_model():
    """Fly the PCP tracker from rest to targets at distance D and compare the time to get
    within 0.05 m with the reach-time model."""
    Ds = np.array([0.5, 1.0, 2.0, 4.0, 8.0, 12.0])
    B = len(Ds)
    pd = np.tile([0.0, 0.0, 2.0], (B, 1))
    vd = np.zeros((B, 3))
    q = pd + np.column_stack([Ds, np.zeros(B), np.zeros(B)])
    T_model = reach_time(pd, vd, q)
    t_arr = np.full(B, np.nan)
    for n in range(1, 2000):
        u = altitude_guard(pd, vd, goto_accel(pd, vd, q))
        vd = sat(vd + u * DT, V_MAX)
        pd = pd + vd * DT
        hit = np.isnan(t_arr) & (_norm(q - pd) < 0.05)
        t_arr[hit] = n * DT
    return Ds, T_model, t_arr


# ── Main simulation ───────────────────────────────────────────────────────────
def run_simulation():
    res = {}
    th0 = make_throws(MC_SEEDS[0])
    res['th0'] = th0
    reach0, card_like0 = reachability_bound(th0)
    res['reach0'], res['card_like0'] = reach0, card_like0
    est0 = run_ekf(th0, SIGMA_Z)
    res['main'] = {s: simulate(s, th0, est0, record=True) for s in STRATS}
    # noise sensitivity (same throws, same standard normals scaled by sigma)
    res['sens'] = {}
    for sg in SIGMA_SWEEP:
        est = est0 if sg == SIGMA_Z else run_ekf(th0, sg)
        for s in CAUSAL:
            res['sens'][(s, sg)] = (res['main'][s]['caught'] if sg == SIGMA_Z
                                    else simulate(s, th0, est)['caught'])
    # seed replication
    res['seeds'] = {MC_SEEDS[0]: {s: res['main'][s]['caught'] for s in STRATS}}
    res['seed_reach'] = {MC_SEEDS[0]: reach0}
    for sd in MC_SEEDS[1:]:
        th = make_throws(sd)
        est = run_ekf(th, SIGMA_Z)
        res['seeds'][sd] = {s: simulate(s, th, est)['caught'] for s in STRATS}
        res['seed_reach'][sd] = reachability_bound(th)[0]
        for s in STRATS:
            assert not np.any(res['seeds'][sd][s] & ~res['seed_reach'][sd]), \
                'catch outside the reachability bound'
    for s in STRATS:
        assert not np.any(res['main'][s]['caught'] & ~reach0), 'catch outside the bound'
    for k, c in res['sens'].items():
        assert not np.any(c & ~reach0), 'catch outside the bound'
    # ablation: PCP / oracle restricted to arrive-and-stop (hold) plans
    res['ablation'] = {s: simulate(s, th0, est0)['caught'] for s in ['pcp_hold', 'oracle_hold']}
    for c in res['ablation'].values():
        assert not np.any(c & ~reach0), 'catch outside the bound'
    res['reach_check'] = validate_reach_model()
    return res


def pick_representative(res):
    m = res['main']
    idx = np.where(m['pcp']['caught'] & ~m['pp']['caught'] & ~m['pn']['caught'])[0]
    if len(idx) == 0:
        idx = np.where(m['pcp']['caught'])[0]
    return int(idx[0]) if len(idx) else 0


# ── Plots ─────────────────────────────────────────────────────────────────────
def _arena_axes(ax, pts):
    """Zoom to the throw: square x-y window around all plotted points (inside the arena)."""
    lo, hi = pts[:, :2].min(axis=0) - 1.0, pts[:, :2].max(axis=0) + 1.0
    c, half = (lo + hi) / 2, max(hi - lo) / 2
    ax.set_xlim(max(c[0] - half, -ARENA[0] / 2), min(c[0] + half, ARENA[0] / 2))
    ax.set_ylim(max(c[1] - half, -ARENA[1] / 2), min(c[1] + half, ARENA[1] / 2))
    ax.set_zlim(0, max(4.5, pts[:, 2].max() + 0.5))
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)'); ax.set_zlabel('z (m)')


def plot_trajectory_3d(res, k, out_dir):
    th, m = res['th0'], res['main']
    ne = int(th['n_end'][k])
    ball = th['truth'][k, :ne + 1, :3]
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(*ball.T, color=BALL_C, lw=2, label='Ball (true)')
    ax.scatter(*ball[0], color=BALL_C, marker='^', s=60)
    plans = m['pcp']['rec']['plans']
    pts = np.array([p['ball_pt'][k] for p in plans if p['valid'][k]])
    tt = np.array([p['t'] for p in plans if p['valid'][k]])
    if len(pts):
        sc = ax.scatter(*pts.T, c=tt, cmap='Greens', s=30, edgecolor='k', lw=0.3,
                        label='PCP predicted catch point')
        fig.colorbar(sc, ax=ax, shrink=0.55, pad=0.08, label='re-plan time (s)')
    for s in STRATS:
        r = m[s]
        nn = ne if not r['caught'][k] else int(np.floor(r['t_catch'][k] / DT)) + 1
        pdk = r['rec']['pd'][:nn + 1, k] + NET_OFFSET
        ls = '--' if s == 'oracle' else '-'
        tag = f"caught t={r['t_catch'][k]:.2f} s" if r['caught'][k] else f"miss {r['d_miss'][k]:.2f} m"
        ax.plot(*pdk.T, ls, color=COLOR[s], lw=2, label=f'{LABEL[s]} net ({tag})')
        ax.scatter(*pdk[-1], color=COLOR[s], marker='o' if r['caught'][k] else 'x', s=70)
    _arena_axes(ax, np.vstack([ball] + [m[s]['rec']['pd'][:ne + 1, k] for s in STRATS]))
    ax.set_title(f'S094 representative throw #{k}: v0 = {th["v0"][k]:.1f} m/s')
    ax.legend(loc='upper left', fontsize=8)
    ax.view_init(elev=22, azim=-60)
    path = os.path.join(out_dir, 'trajectory_3d.png')
    plt.savefig(path, dpi=130, bbox_inches='tight'); plt.close(fig)
    print(f'Saved: {path}')


def plot_catch_rate(res, out_dir):
    m, n = res['main'], N_THROWS
    fig, ax = plt.subplots(figsize=(8, 5))
    for i, s in enumerate(STRATS):
        c = int(m[s]['caught'].sum())
        lo, hi = wilson(c, n)
        ax.bar(i, c / n * 100, color=COLOR[s], edgecolor='k',
               hatch='//' if s == 'oracle' else None)
        ax.errorbar(i, c / n * 100, yerr=[[c / n * 100 - lo * 100], [hi * 100 - c / n * 100]],
                    color='k', capsize=6)
        ax.text(i, hi * 100 + 1.5, f'{100 * c / n:.1f}%', ha='center')
    rb = res['reach0'].mean() * 100
    ax.axhline(rb, color='k', ls=':', lw=1.5, label=f'Reachability bound {rb:.1f}%')
    ax.set_xticks(range(len(STRATS))); ax.set_xticklabels([LABEL[s] for s in STRATS])
    ax.set_ylabel('Catch rate (%)'); ax.set_ylim(0, 105)
    ax.set_title(f'S094 catch rate, {n} common throws (95% Wilson CI)')
    ax.legend(loc='upper left')
    path = os.path.join(out_dir, 'catch_rate.png')
    plt.savefig(path, dpi=150, bbox_inches='tight'); plt.close(fig)
    print(f'Saved: {path}')


def plot_miss_cdf(res, out_dir):
    m = res['main']
    fig, ax = plt.subplots(figsize=(8, 5))
    for s in STRATS:
        d = np.sort(np.where(np.isfinite(m[s]['d_miss']), m[s]['d_miss'], 99.0))
        ax.step(d, np.arange(1, len(d) + 1) / len(d) * 100, where='post', color=COLOR[s],
                ls='--' if s == 'oracle' else '-', lw=2, label=LABEL[s])
    ax.axvline(R_CATCH, color='k', ls=':', label=f'$r_c$ = {R_CATCH} m')
    ax.set_xscale('log'); ax.set_xlim(0.005, 20)
    ax.set_xlabel(r'Miss distance $d_{miss}$ (m, descending, $z_b \geq z_{min}$)')
    ax.set_ylabel('Throws (%)'); ax.set_title('S094 miss-distance CDF (seed 0, 300 throws)')
    ax.legend(); ax.grid(alpha=0.3, which='both')
    path = os.path.join(out_dir, 'miss_cdf.png')
    plt.savefig(path, dpi=150, bbox_inches='tight'); plt.close(fig)
    print(f'Saved: {path}')


def speed_bin_rates(res):
    v0 = res['th0']['v0']
    b = np.digitize(v0, SPEED_BINS[1:-1])
    rows = {}
    for s in STRATS:
        rows[s] = [res['main'][s]['caught'][b == i].mean() for i in range(len(SPEED_BINS) - 1)]
    rows['bound'] = [res['reach0'][b == i].mean() for i in range(len(SPEED_BINS) - 1)]
    counts = [int((b == i).sum()) for i in range(len(SPEED_BINS) - 1)]
    return rows, counts


def plot_catch_vs_speed(res, out_dir):
    rows, counts = speed_bin_rates(res)
    nb = len(counts)
    x = np.arange(nb)
    w = 0.2
    fig, ax = plt.subplots(figsize=(9, 5))
    for i, s in enumerate(STRATS):
        ax.bar(x + (i - 1.5) * w, np.array(rows[s]) * 100, w, color=COLOR[s], edgecolor='k',
               hatch='//' if s == 'oracle' else None, label=LABEL[s])
    for i in range(nb):
        ax.hlines(rows['bound'][i] * 100, i - 2 * w, i + 2 * w, colors='k', linestyles=':',
                  label='Reachability bound' if i == 0 else None)
    ax.set_xticks(x)
    ax.set_xticklabels([f'{SPEED_BINS[i]:.0f}-{SPEED_BINS[i + 1]:.0f} m/s\n(n={counts[i]})'
                        for i in range(nb)])
    ax.set_ylabel('Catch rate (%)'); ax.set_ylim(0, 105)
    ax.set_title('S094 catch rate vs throw speed')
    ax.legend(fontsize=8, loc='upper left', bbox_to_anchor=(1.01, 1.0))
    path = os.path.join(out_dir, 'catch_vs_speed.png')
    plt.savefig(path, dpi=150, bbox_inches='tight'); plt.close(fig)
    print(f'Saved: {path}')


def plot_noise_sensitivity(res, out_dir):
    fig, ax = plt.subplots(figsize=(8, 5))
    for s in CAUSAL:
        r = np.array([res['sens'][(s, sg)].mean() * 100 for sg in SIGMA_SWEEP])
        ci = np.array([wilson(int(res['sens'][(s, sg)].sum()), N_THROWS) for sg in SIGMA_SWEEP])
        ax.errorbar(SIGMA_SWEEP, r, yerr=[r - ci[:, 0] * 100, ci[:, 1] * 100 - r], marker='o',
                    color=COLOR[s], capsize=4, lw=2, label=LABEL[s])
    ax.axhline(res['main']['oracle']['caught'].mean() * 100, color=COLOR['oracle'], ls='--',
               label='Oracle planner (noise-free)')
    ax.axhline(res['reach0'].mean() * 100, color='k', ls=':', label='Reachability bound')
    ax.set_xscale('log'); ax.set_xticks(SIGMA_SWEEP); ax.set_xticklabels([str(s) for s in SIGMA_SWEEP])
    ax.set_xlabel(r'Measurement noise $\sigma_z$ (m)'); ax.set_ylabel('Catch rate (%)')
    ax.set_ylim(0, 105); ax.set_title('S094 noise sensitivity (same throws and noise draws)')
    ax.legend(fontsize=8); ax.grid(alpha=0.3)
    path = os.path.join(out_dir, 'noise_sensitivity.png')
    plt.savefig(path, dpi=150, bbox_inches='tight'); plt.close(fig)
    print(f'Saved: {path}')


def convergence_stats(res):
    plans = res['main']['pcp']['rec']['plans']
    t, med, p90, med0, nv = [], [], [], [], []
    for p in plans:
        v = p['valid']
        v0 = p['valid_nodrag']
        if v.sum() < 20:
            continue
        t.append(p['t']); nv.append(int(v.sum()))
        med.append(np.median(p['err'][v])); p90.append(np.percentile(p['err'][v], 90))
        med0.append(np.median(p['err_nodrag'][v0]) if v0.sum() else np.nan)
    ee = res['main']['pcp']['rec']['est_err']
    te = np.array([e[0] for e in ee])
    pe = np.array([np.median(e[1][e[3]]) if e[3].any() else np.nan for e in ee])
    ve = np.array([np.median(e[2][e[3]]) if e[3].any() else np.nan for e in ee])
    return dict(t=np.array(t), med=np.array(med), p90=np.array(p90), med0=np.array(med0),
                nv=np.array(nv), te=te, pe=pe, ve=ve)


def plot_estimator_convergence(res, out_dir):
    c = convergence_stats(res)
    fig, axs = plt.subplots(1, 2, figsize=(12, 4.6))
    ax = axs[0]
    ax.plot(c['t'], c['med'] * 100, 'o-', color=COLOR['pcp'], label='EKF + drag model: median')
    ax.fill_between(c['t'], c['med'] * 100, c['p90'] * 100, color=COLOR['pcp'], alpha=0.15,
                    label='EKF + drag model: median to 90th percentile')
    ax.plot(c['t'], c['med0'] * 100, 's--', color='grey', label='Same EKF, drag-free predictor: median')
    ax.axhline(R_CATCH * 100, color='k', ls=':', label=f'$r_c$ = {R_CATCH * 100:.0f} cm')
    ax.set_xlabel('Time since launch (s)'); ax.set_ylabel('Predicted catch-point error (cm)')
    ax.set_yscale('log'); ax.set_title('Planned catch point vs true ball at the same time')
    ax.legend(fontsize=8); ax.grid(alpha=0.3, which='both')
    ax = axs[1]
    ax.plot(c['te'], c['pe'] * 100, color=BALL_C, label='position error (cm)')
    ax.plot(c['te'], c['ve'] * 100, color=TARGET_C, label='velocity error (cm/s)')
    ax.set_yscale('log'); ax.set_xlabel('Time since launch (s)')
    ax.set_title(r'EKF current-time estimate, median over throws in flight ($\sigma_z$=0.02 m)')
    ax.legend(fontsize=8); ax.grid(alpha=0.3, which='both')
    plt.tight_layout()
    path = os.path.join(out_dir, 'estimator_convergence.png')
    plt.savefig(path, dpi=150, bbox_inches='tight'); plt.close(fig)
    print(f'Saved: {path}')


def save_animation(res, k, out_dir):
    th, m = res['th0'], res['main']
    ne = int(th['n_end'][k])
    ball = th['truth'][k, :ne + 1, :3]
    ends = {}
    for s in STRATS:
        r = m[s]
        ends[s] = ne if not r['caught'][k] else int(np.floor(r['t_catch'][k] / DT)) + 1
    plans = m['pcp']['rec']['plans']
    n_frames = int(np.ceil(ne / GIF_STEP)) + 1          # ceil((n-1)/step) + 1 samples 0..ne
    fig = plt.figure(figsize=(7, 6))
    ax = fig.add_subplot(111, projection='3d')
    _arena_axes(ax, np.vstack([ball] + [m[s]['rec']['pd'][:ne + 1, k] for s in STRATS]))
    ax.view_init(elev=22, azim=-60)
    ax.plot(*ball.T, color=BALL_C, lw=0.8, alpha=0.3)
    b_dot, = ax.plot([], [], [], 'o', color=BALL_C, ms=7, label='Ball')
    tgt, = ax.plot([], [], [], '*', color=TARGET_C, ms=12, label='PCP catch point')
    lines, dots = {}, {}
    for s in STRATS:
        lines[s], = ax.plot([], [], [], '--' if s == 'oracle' else '-', color=COLOR[s], lw=2,
                            label=LABEL[s])
        dots[s], = ax.plot([], [], [], 'o', color=COLOR[s], ms=6)
    ax.legend(loc='upper left', fontsize=7)
    title = ax.set_title('')

    def update(i):
        n = min(i * GIF_STEP, ne)
        b_dot.set_data_3d([ball[n, 0]], [ball[n, 1]], [ball[n, 2]])
        cur = [p for p in plans if p['t'] <= n * DT + 1e-9 and p['valid'][k]]
        if cur:
            cp = cur[-1]['ball_pt'][k]
            tgt.set_data_3d([cp[0]], [cp[1]], [cp[2]])
        status = []
        for s in STRATS:
            nn = min(n, ends[s])
            p = m[s]['rec']['pd'][:nn + 1, k] + NET_OFFSET
            lines[s].set_data_3d(p[:, 0], p[:, 1], p[:, 2])
            dots[s].set_data_3d([p[-1, 0]], [p[-1, 1]], [p[-1, 2]])
            if m[s]['caught'][k] and n >= ends[s]:
                status.append(f'{s.upper()} caught')
        title.set_text(f'S094 throw #{k}  t = {n * DT:.2f} s  ' + ', '.join(status))
        return [b_dot, tgt, title] + list(lines.values()) + list(dots.values())

    anim = animation.FuncAnimation(fig, update, frames=n_frames, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    anim.save(path, writer='pillow', fps=12, dpi=80)
    plt.close(fig)
    print(f'Saved: {path} ({n_frames} frames)')


# ── Report ────────────────────────────────────────────────────────────────────
def report(res, k_rep):
    th, m, n = res['th0'], res['main'], N_THROWS
    truth, n_end = th['truth'], th['n_end']
    print('=== S094 Aerial Ball Catching ===')
    print(f'  Drag constant k = {K_DRAG:.5f} 1/m, terminal speed = {np.sqrt(G / K_DRAG):.2f} m/s')
    ft = (n_end - 1) * DT
    apex = np.array([truth[i, :n_end[i] + 1, 2].max() for i in range(n)])
    print(f'  Flight time to z_min (seed 0): min {ft.min():.2f} s, mean {ft.mean():.2f} s, '
          f'max {ft.max():.2f} s')
    print(f'  Apex height: min {apex.min():.2f} m, max {apex.max():.2f} m; '
          f'{100 * np.mean(apex > 6.0):.1f}% of throws peak above 6 m')
    for v0_, el_ in [(V0_RANGE[0], EL_RANGE[0]), (V0_RANGE[1], EL_RANGE[1])]:
        x = np.array([[0, 0, Z_RELEASE, v0_ * np.cos(np.radians(el_)), 0,
                       v0_ * np.sin(np.radians(el_))]])
        zmax, ti = Z_RELEASE, 0
        while not (x[0, 2] < Z_MIN_BALL and x[0, 5] < 0):
            x = rk4(x, DT); ti += 1; zmax = max(zmax, x[0, 2])
        print(f'  Extreme throw v0 = {v0_:.0f} m/s, elevation {el_:.0f} deg: apex {zmax:.2f} m, '
              f'flight to z_min {ti * DT:.2f} s')
    # drag effect: no-drag landing point vs drag landing point at z_min
    p0, v0v = th['p0'], th['vb0']
    tz = (v0v[:, 2] + np.sqrt(v0v[:, 2]**2 + 2 * G * (p0[:, 2] - Z_MIN_BALL))) / G
    land_nd = p0[:, :2] + v0v[:, :2] * tz[:, None]
    i0, i1 = n_end - 1, n_end
    z0_, z1_ = truth[np.arange(n), i0, 2], truth[np.arange(n), i1, 2]
    f = (z0_ - Z_MIN_BALL) / (z0_ - z1_)
    land_d = truth[np.arange(n), i0, :2] + f[:, None] * (truth[np.arange(n), i1, :2]
                                                        - truth[np.arange(n), i0, :2])
    dl = _norm(land_nd - land_d)
    print(f'  Drag shift of landing point at z_min: min {dl.min():.2f} m, mean {dl.mean():.2f} m, '
          f'max {dl.max():.2f} m')
    print(f'  Reachability bound (strict, seed 0): {res["reach0"].sum()}/{n} = '
          f'{100 * res["reach0"].mean():.1f}%')
    print(f'  Card-style landing-point estimate (seed 0): {res["card_like0"].sum()}/{n} = '
          f'{100 * res["card_like0"].mean():.1f}%')
    print('  Reach-model check (PCP tracker from rest, time to within 0.05 m):')
    for D, tm, ta in zip(*res['reach_check']):
        print(f'    D = {D:5.1f} m: model {tm:.3f} s, simulated {ta:.3f} s')
    print(f'\n  Main run: seed 0, {n} common throws, sigma_z = {SIGMA_Z} m')
    print(f'  {"strategy":<22}{"catch":>7}{"rate":>8}{"95% Wilson":>17}{"eta_bound":>11}'
          f'{"eta_oracle":>12}')
    nr = int(res['reach0'].sum())
    no = int(m['oracle']['caught'].sum())
    for s in STRATS:
        c = int(m[s]['caught'].sum())
        lo, hi = wilson(c, n)
        print(f'  {LABEL[s]:<22}{c:>7d}{100 * c / n:>7.1f}%   [{100 * lo:5.1f}, {100 * hi:5.1f}]%'
              f'{100 * c / nr:>10.1f}%{100 * c / no:>11.1f}%')
    for s in ['pcp', 'oracle']:
        mf, c = m[s]['mode_first'], m[s]['caught']
        txt = ', '.join(f'{nm} {int(np.sum(mf == i))} (caught {int(np.sum(c & (mf == i)))})'
                        for i, nm in enumerate(['hold', 'intercept', 'late']))
        print(f'  {LABEL[s]} plan mode at t_react: {txt}')
    for s in STRATS:
        vr = m[s]['v_rel_catch'][m[s]['caught']]
        print(f'  {LABEL[s]} ball-net relative speed at catch: mean {vr.mean():.2f} m/s, '
              f'max {vr.max():.2f} m/s')
    print(f'  PN vs a constant target acceleration g needs up to N/(N-2) g = '
          f'{PN_N / (PN_N - 2) * G:.2f} m/s^2 (a_max = {A_MAX:.0f} m/s^2)')
    for s in STRATS:
        miss = ~m[s]['caught']
        print(f'  {LABEL[s]} misses: {int(np.sum(miss & ~res["reach0"]))} unreachable (outside '
              f'bound), {int(np.sum(miss & res["reach0"]))} strategy failures')
    for s, c in res['ablation'].items():
        print(f'  Ablation {s} (arrive-and-stop plans only): {int(c.sum())}/{n} = {100 * c.mean():.1f}%')
    print('  Miss distance d_miss (m) quantiles [10, 25, 50, 75, 90]%:')
    for s in STRATS:
        d = np.where(np.isfinite(m[s]['d_miss']), m[s]['d_miss'], np.inf)
        q = np.percentile(d, [10, 25, 50, 75, 90])
        print(f'    {LABEL[s]:<22}' + ' '.join(f'{x:7.3f}' for x in q))
    print('  Effort and speed (mean +- std):')
    for s in STRATS:
        r = m[s]
        c = r['caught']
        e_c = r['effort'][c]; v_c = r['v_peak'][c]
        print(f'    {LABEL[s]:<22} per catch: effort {e_c.mean():6.1f} +- {e_c.std():5.1f} m^2/s^3, '
              f'peak speed {v_c.mean():.2f} +- {v_c.std():.2f} m/s, '
              f'catch time {np.nanmean(r["t_catch"]):.2f} s | all throws: effort '
              f'{r["effort"].mean():6.1f}, peak speed {r["v_peak"].mean():.2f} m/s, '
              f'altitude limit touched {100 * r["at_limit"].mean():.1f}%')
    print('  Paired comparison on common throws (A wins / ties / B wins; exact McNemar p):')
    for a, b in [('pcp', 'pp'), ('pcp', 'pn'), ('pn', 'pp'), ('oracle', 'pcp')]:
        ca, cb = m[a]['caught'], m[b]['caught']
        w, l = int(np.sum(ca & ~cb)), int(np.sum(~ca & cb))
        p = binomtest(w, w + l, 0.5).pvalue if w + l else 1.0
        print(f'    {a.upper():>6} vs {b.upper():<6}: {w:3d} / {n - w - l:3d} / {l:3d}   p = {p:.2e}')
    rows, counts = speed_bin_rates(res)
    print('  Catch rate by throw speed bin:')
    print('    ' + ' ' * 22 + ''.join(f'{SPEED_BINS[i]:.0f}-{SPEED_BINS[i + 1]:.0f} m/s (n={counts[i]})'.rjust(22)
                                       for i in range(len(counts))))
    for s in STRATS + ['bound']:
        lab = LABEL.get(s, 'Reachability bound')
        print(f'    {lab:<22}' + ''.join(f'{100 * x:21.1f}%' for x in rows[s]))
    print('  Noise sensitivity (catch rate, same throws and noise draws):')
    print('    ' + ' ' * 22 + ''.join(f'sigma={sg}'.rjust(14) for sg in SIGMA_SWEEP))
    for s in CAUSAL:
        print(f'    {LABEL[s]:<22}' + ''.join(f'{100 * res["sens"][(s, sg)].mean():13.1f}%'
                                              for sg in SIGMA_SWEEP))
    c = convergence_stats(res)
    print('  PCP predicted catch-point error (median / 90th pct, cm) vs time since launch:')
    for i in range(0, len(c['t']), 4):
        print(f'    t = {c["t"][i]:.2f} s (n={c["nv"][i]:3d}): drag model {100 * c["med"][i]:6.1f} / '
              f'{100 * c["p90"][i]:6.1f};  drag-free predictor median {100 * c["med0"][i]:6.1f}')
    print(f'\n  Seed replication ({len(MC_SEEDS)} seeds x {n} throws, sigma_z = {SIGMA_Z} m):')
    tab = {s: np.array([res['seeds'][sd][s].mean() for sd in MC_SEEDS]) for s in STRATS}
    rb = np.array([res['seed_reach'][sd].mean() for sd in MC_SEEDS])
    for sd_i, sd in enumerate(MC_SEEDS):
        print(f'    seed {sd}: ' + ', '.join(f'{s.upper()} {100 * tab[s][sd_i]:.1f}%' for s in STRATS)
              + f', bound {100 * rb[sd_i]:.1f}%')
    for s in STRATS:
        print(f'    {LABEL[s]:<22} mean {100 * tab[s].mean():5.1f}% +- {100 * tab[s].std(ddof=1):4.1f}%')
    print(f'    {"Reachability bound":<22} mean {100 * rb.mean():5.1f}% +- {100 * rb.std(ddof=1):4.1f}%')
    order_ok = sum(int(tab['pcp'][i] > tab['pn'][i] > tab['pp'][i]) for i in range(len(MC_SEEDS)))
    print(f'    Seeds with PCP > PN > PP: {order_ok}/{len(MC_SEEDS)}; '
          f'PCP > PN: {int(np.sum(tab["pcp"] > tab["pn"]))}/{len(MC_SEEDS)}; '
          f'PN > PP: {int(np.sum(tab["pn"] > tab["pp"]))}/{len(MC_SEEDS)}; '
          f'PCP > PP: {int(np.sum(tab["pcp"] > tab["pp"]))}/{len(MC_SEEDS)}')
    print(f'\n  Representative throw #{k_rep}: v0 = {th["v0"][k_rep]:.2f} m/s, flight '
          f'{(n_end[k_rep] - 1) * DT:.2f} s; ' + ', '.join(
              f"{s.upper()} {'caught at %.2f s' % m[s]['t_catch'][k_rep] if m[s]['caught'][k_rep] else 'miss %.2f m' % m[s]['d_miss'][k_rep]}"
              for s in STRATS))
    print('  Invariants asserted every step: |v| <= v_max, |a| <= a_max, '
          'z in [0.3, 4.0] m; every catch lies inside the reachability bound')


if __name__ == '__main__':
    os.makedirs(OUT_DIR, exist_ok=True)
    res = run_simulation()
    k_rep = pick_representative(res)
    report(res, k_rep)
    plot_trajectory_3d(res, k_rep, OUT_DIR)
    plot_catch_rate(res, OUT_DIR)
    plot_miss_cdf(res, OUT_DIR)
    plot_catch_vs_speed(res, OUT_DIR)
    plot_noise_sensitivity(res, OUT_DIR)
    plot_estimator_convergence(res, OUT_DIR)
    save_animation(res, k_rep, OUT_DIR)
