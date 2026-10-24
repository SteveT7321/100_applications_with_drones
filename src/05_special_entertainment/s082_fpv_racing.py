"""
S082 FPV Racing Circuit
=======================
A racing drone flies a lap through 8 random gates (20 x 20 x 10 m arena) and back to the start.
Two planning pipelines are compared on the same layouts:

  1. Minimum-snap only  - degree-7 minimum-snap spline, segment times = distance / 5 m/s.
  2. Time-optimal       - the same spline re-timed by a time-optimal path parameterisation
                          (TOPP, forward-backward phase-plane pass) under |v| <= 10 m/s and
                          |thrust acceleration| <= 15 m/s^2.

Both references are tracked by the same receding-horizon MPC (horizon 10 x 0.02 s, card weights)
and flown on two plant models:

  * point mass  - the card's model (v' = u - c/m v + g), Euler at 0.02 s;
  * rigid body  - src/base/rigid_body_drone.py (6-DOF, quaternion attitude, motor lag, mixer),
                  fpv_5inch preset with the card's mass 0.5 kg; MPC thrust vector -> attitude ->
                  body-rate PI at 500 Hz.

The headline layout is the card's seed 42; a 20-layout Monte Carlo (seeds 42-61) gives the
statistics.

Usage:
    conda activate drones
    python src/05_special_entertainment/s082_fpv_racing.py
"""

import os
import sys
from math import factorial
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'base'))
from rigid_body_drone import (RigidBodyDrone, make_params, default_gains,  # noqa: E402
                              quat_to_rot, tilt_angle, G)

# ── Parameters (card) ─────────────────────────────────────────────────────────
N_GATES     = 8
R_GATE      = 0.5            # m    gate ring radius
R_TOL       = 0.2            # m    passage tolerance
V_MAX       = 10.0           # m/s
A_MAX       = 15.0           # m/s^2  thrust acceleration limit |u|
MASS        = 0.5            # kg
C_DRAG      = 0.05           # kg/s
G_VEC       = np.array([0.0, 0.0, -G])
DT_MPC      = 0.02           # s    MPC / point-mass step (50 Hz)
N_MPC       = 10             # MPC horizon steps
V_REF_INIT  = 5.0            # m/s  speed used for the initial (min-snap) segment times
N_ARC       = 500            # TOPP grid points
Q_P, Q_V, Q_F, R_U = 10.0, 2.0, 50.0, 0.1
START       = np.array([0.0, 0.0, 3.0])
ARENA_XY    = 20.0
ARENA_Z     = 10.0
LAYOUT_SEED = 42

# ── Parameters (added) ────────────────────────────────────────────────────────
DT_INNER    = 0.002          # s    rigid-body control / physics step (500 Hz)
N_SUB       = int(round(DT_MPC / DT_INNER))
MC_SEEDS    = list(range(42, 62))   # 20 layouts, first = card seed
FISTA_ITERS = 40             # projected-gradient iterations of the constrained MPC QP
R_SEARCH    = 2.0            # m    a gate visit starts when the path comes this close
END_RADIUS  = 0.5            # m    lap ends when back within this distance of the start
T_PAD       = 4.0            # s    simulated time after the reference ends
GUP         = np.array([0.0, 0.0, G])
MAX_TILT    = np.radians(89.0)  # racing: no practical tilt limit (preset default is 70 deg)

STRATS = ['minsnap', 'scaled', 'topp']
MODELS = ['pm', 'rb']
S_LABEL = {'minsnap': 'Min-snap only', 'scaled': 'Min-snap, uniform scale',
           'topp': 'Time-optimal (TOPP)'}
M_LABEL = {'pm': 'point mass', 'rb': 'rigid body'}
S_COLOR = {'minsnap': 'tab:blue', 'scaled': 'tab:purple', 'topp': 'tab:red'}
M_STYLE = {'pm': ':', 'rb': '-'}

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), '..', '..',
                          'outputs', '05_special_entertainment', 's082_fpv_racing')


# ── Course ────────────────────────────────────────────────────────────────────
def generate_gates(seed=LAYOUT_SEED):
    """Card's generator: 8 gates on a rough circle, normals = tangent + small tilt."""
    rng = np.random.default_rng(seed)
    angles = np.linspace(0, 2 * np.pi, N_GATES, endpoint=False)
    radius = rng.uniform(6.0, 8.0, N_GATES)
    centres = np.column_stack([
        radius * np.cos(angles) + rng.uniform(-1.0, 1.0, N_GATES),
        radius * np.sin(angles) + rng.uniform(-1.0, 1.0, N_GATES),
        rng.uniform(2.0, 8.0, N_GATES),
    ])
    tangents = np.roll(centres, -1, axis=0) - centres
    norms = tangents / (np.linalg.norm(tangents, axis=1, keepdims=True) + 1e-9)
    norms += rng.normal(0, 0.1, norms.shape)
    norms /= np.linalg.norm(norms, axis=1, keepdims=True)
    return centres, norms


# ── Minimum-snap spline (exact QP via KKT) ───────────────────────────────────
def _drow(t, r):
    """Row vector of d^r/dt^r [1, t, ..., t^7]."""
    row = np.zeros(8)
    for j in range(r, 8):
        row[j] = factorial(j) / factorial(j - r) * t ** (j - r)
    return row


def _snap_Q(T):
    Q = np.zeros((8, 8))
    for j in range(4, 8):
        for k in range(4, 8):
            cj = factorial(j) / factorial(j - 4)
            ck = factorial(k) / factorial(k - 4)
            Q[j, k] = cj * ck * T ** (j + k - 7) / (j + k - 7)
    return Q


def min_snap(waypoints, T):
    """Degree-7 minimum-snap spline through waypoints, C3 at interior points, zero velocity and
    acceleration at both ends. Returns coeffs (M, 8, 3) in real time t in [0, T_k], and J."""
    M = len(T)
    n = 8 * M
    H = np.zeros((n, n))
    for k in range(M):
        H[8 * k:8 * k + 8, 8 * k:8 * k + 8] = _snap_Q(T[k])
    rows, rhs = [], []

    def add(row_parts, val):
        row = np.zeros(n)
        for k, v in row_parts:
            row[8 * k:8 * k + 8] += v
        rows.append(row)
        rhs.append(val)

    for k in range(M):
        add([(k, _drow(0.0, 0))], waypoints[k])
        add([(k, _drow(T[k], 0))], waypoints[k + 1])
    for k in range(M - 1):
        for r in (1, 2, 3):
            add([(k, _drow(T[k], r)), (k + 1, -_drow(0.0, r))], np.zeros(3))
    for r in (1, 2):
        add([(0, _drow(0.0, r))], np.zeros(3))
        add([(M - 1, _drow(T[-1], r))], np.zeros(3))
    A = np.array(rows)
    b = np.array(rhs)
    m = A.shape[0]
    K = np.block([[2 * H, A.T], [A, np.zeros((m, m))]])
    sol = np.linalg.solve(K, np.vstack([np.zeros((n, 3)), b]))
    c = sol[:n]
    J = float(sum(c[:, a] @ H @ c[:, a] for a in range(3)))
    assert np.allclose(A @ c, b, atol=1e-8)
    return c.reshape(M, 8, 3), J


def eval_spline(coeffs, T, t, r=0):
    """Derivative r of the spline at times t (array) -> (len(t), 3)."""
    t = np.atleast_1d(t)
    cum = np.concatenate([[0.0], np.cumsum(T)])
    seg = np.clip(np.searchsorted(cum, t, side='right') - 1, 0, len(T) - 1)
    tau = np.clip(t - cum[seg], 0.0, T[seg])
    out = np.zeros((len(t), 3))
    for j in range(r, 8):
        out += (factorial(j) / factorial(j - r)) * (tau ** (j - r))[:, None] * coeffs[seg, j]
    return out


# ── Time-optimal path parameterisation ────────────────────────────────────────
def _udot_bounds(x, d1, d2, amax=A_MAX):
    """Feasible path acceleration sdd for squared path speed x at the grid points.
    Constraint |d2 x + d1 sdd + (c/m) d1 sqrt(x) + g_up| <= amax. Returns (lo, hi, feasible)."""
    w = d2 * x[:, None] + (C_DRAG / MASS) * d1 * np.sqrt(x)[:, None] + GUP
    bb = np.sum(d1 * d1, axis=1)
    wb = np.sum(w * d1, axis=1)
    ww = np.sum(w * w, axis=1)
    disc = wb ** 2 - bb * (ww - amax ** 2)
    sq = np.sqrt(np.maximum(disc, 0.0))
    return (-wb - sq) / bb, (-wb + sq) / bb, disc >= 0


def _arc_derivs(coeffs, T, s):
    """Unit tangent d1 = dp/dsigma and d2 = d^2p/dsigma^2 at spline times s (arc length sigma)."""
    p1 = eval_spline(coeffs, T, s, 1)
    p2 = eval_spline(coeffs, T, s, 2)
    sp = np.maximum(np.linalg.norm(p1, axis=1), 1e-6)
    d1 = p1 / sp[:, None]
    d2 = (p2 - np.sum(p2 * d1, axis=1)[:, None] * d1) / (sp ** 2)[:, None]
    return d1, d2


def arc_table(coeffs, T, n=20001):
    s_f = np.linspace(0.0, T.sum(), n)
    sp = np.linalg.norm(eval_spline(coeffs, T, s_f, 1), axis=1)
    sig_f = np.concatenate([[0.0], np.cumsum(0.5 * (sp[1:] + sp[:-1]) * np.diff(s_f))])
    return s_f, sig_f


def topp(coeffs, T):
    """Forward-backward phase-plane TOPP on N_ARC points uniformly spaced in arc length."""
    s_f, sig_f = arc_table(coeffs, T)
    sig = np.linspace(0.0, sig_f[-1], N_ARC)
    ds = sig[1] - sig[0]
    s_grid = np.interp(sig, sig_f, s_f)
    d1, d2 = _arc_derivs(coeffs, T, s_grid)
    # the rest-to-rest end points have p' = p'' = 0 (tangent undefined, curvature ~ sigma^-2/3):
    # use the neighbouring grid point's tangent and curvature there (speed is zero anyway)
    d1[0], d2[0] = d1[1], d2[1]
    d1[-1], d2[-1] = d1[-2], d2[-2]
    x_v = np.full(N_ARC, V_MAX ** 2)
    # maximum-velocity curve from the thrust limit: bisection on x (feasible set is [0, x*])
    lo, hi = np.zeros(N_ARC), x_v.copy()
    ok_hi = _udot_bounds(hi, d1, d2)[2]
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        ok = _udot_bounds(mid, d1, d2)[2]
        lo, hi = np.where(ok, mid, lo), np.where(ok, hi, mid)
    mvc = np.where(ok_hi, x_v, lo)
    # forward pass (max acceleration) and backward pass (max deceleration)
    xf = np.zeros(N_ARC)
    for i in range(N_ARC - 1):
        _, uh, _ = _udot_bounds(xf[i:i + 1], d1[i:i + 1], d2[i:i + 1])
        xf[i + 1] = min(mvc[i + 1], max(xf[i] + 2 * uh[0] * ds, 0.0))
    xb = np.zeros(N_ARC)
    for i in range(N_ARC - 1, 0, -1):
        ul, _, _ = _udot_bounds(xb[i:i + 1], d1[i:i + 1], d2[i:i + 1])
        xb[i - 1] = min(mvc[i - 1], max(xb[i] - 2 * ul[0] * ds, 0.0))
    x = np.minimum(xf, xb)
    sdot = np.sqrt(x)
    dt = 2 * ds / np.maximum(sdot[1:] + sdot[:-1], 1e-9)
    t_grid = np.concatenate([[0.0], np.cumsum(dt)])
    return dict(sig=sig, sdot=sdot, t=t_grid, mvc=np.sqrt(mvc), T=t_grid[-1], s_f=s_f,
                sig_f=sig_f, length=sig_f[-1], d1=d1, d2=d2)


def scale_spline(coeffs, T, k):
    """Same path flown k times slower: p'(t) = p(t / k)  ->  c_j' = c_j / k^j, T' = k T."""
    return coeffs / (k ** np.arange(8))[None, :, None], T * k


def feasible_scale(coeffs, T, n=4000):
    """Smallest uniform time scale k with |v| <= V_MAX and |u| <= A_MAX on a dense sample."""
    def ok(k):
        c, Tk = scale_spline(coeffs, T, k)
        t = np.linspace(0, Tk.sum(), n)
        v = eval_spline(c, Tk, t, 1)
        u = eval_spline(c, Tk, t, 2) + (C_DRAG / MASS) * v + GUP
        return (np.linalg.norm(v, axis=1).max() <= V_MAX
                and np.linalg.norm(u, axis=1).max() <= A_MAX)
    lo, hi = 0.2, 5.0
    assert ok(hi) and not ok(lo)
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        lo, hi = (lo, mid) if ok(mid) else (mid, hi)
    return hi


# ── References sampled on a fixed time grid ──────────────────────────────────
def build_reference(coeffs, T, strategy, dt):
    """Reference p, v, a (net acceleration) and u = a + c/m v - g on a dt grid."""
    scale = 1.0
    if strategy == 'scaled':
        scale = feasible_scale(coeffs, T)
        coeffs, T = scale_spline(coeffs, T, scale)
    if strategy in ('minsnap', 'scaled'):
        t_end = T.sum()
        t = np.arange(0.0, t_end + T_PAD + dt / 2, dt)
        tq = np.minimum(t, t_end)
        p = eval_spline(coeffs, T, tq, 0)
        v = eval_spline(coeffs, T, tq, 1) * (t <= t_end)[:, None]
        a = eval_spline(coeffs, T, tq, 2) * (t <= t_end)[:, None]
        info = dict(T_ref=t_end, scale=scale)
    else:
        tp = topp(coeffs, T)
        t_end = tp['T']
        t = np.arange(0.0, t_end + T_PAD + dt / 2, dt)
        tq = np.minimum(t, t_end)
        sig_t = np.interp(tq, tp['t'], tp['sig'])
        sd_t = np.interp(tq, tp['t'], tp['sdot'])
        # piecewise-constant path acceleration between grid points
        ds = tp['sig'][1] - tp['sig'][0]
        sdd_seg = (tp['sdot'][1:] ** 2 - tp['sdot'][:-1] ** 2) / (2 * ds)
        idx = np.clip(np.searchsorted(tp['t'], tq, side='right') - 1, 0, N_ARC - 2)
        sdd_t = np.where(t < t_end, sdd_seg[idx], 0.0)
        s_t = np.interp(sig_t, tp['sig_f'], tp['s_f'])
        # path derivatives interpolated from the TOPP grid (the profile is feasible there;
        # the analytic curvature is singular at the rest-to-rest end points)
        d1 = np.column_stack([np.interp(sig_t, tp['sig'], tp['d1'][:, j]) for j in range(3)])
        d1 /= np.linalg.norm(d1, axis=1, keepdims=True)
        d2 = np.column_stack([np.interp(sig_t, tp['sig'], tp['d2'][:, j]) for j in range(3)])
        p = eval_spline(coeffs, T, s_t, 0)
        v = d1 * sd_t[:, None]
        a = d2 * (sd_t ** 2)[:, None] + d1 * sdd_t[:, None]
        info = dict(T_ref=t_end, topp=tp)
    u = a + (C_DRAG / MASS) * v + GUP
    return dict(t=t, p=p, v=v, a=a, u=u, **info)


# ── MPC (condensed QP, |u_j| <= A_MAX, accelerated projected gradient) ───────
class MPC:
    def __init__(self):
        dt, al = DT_MPC, 1.0 - C_DRAG / MASS * DT_MPC
        I3 = np.eye(3)
        Ad = np.block([[I3, dt * I3], [np.zeros((3, 3)), al * I3]])
        Bd = np.vstack([0.5 * dt ** 2 * I3, dt * I3])
        d = Bd @ G_VEC
        N = N_MPC
        Sx = np.zeros((6 * N, 6))
        Su = np.zeros((6 * N, 3 * N))
        Sd = np.zeros(6 * N)
        Ak = np.eye(6)
        for i in range(N):
            Ak = Ad @ Ak
            Sx[6 * i:6 * i + 6] = Ak
            acc = np.zeros(6)
            for j in range(i + 1):
                Aij = np.linalg.matrix_power(Ad, i - j)
                Su[6 * i:6 * i + 6, 3 * j:3 * j + 3] = Aij @ Bd
                acc += Aij @ d
            Sd[6 * i:6 * i + 6] = acc
        qd = np.zeros(6 * N)
        for i in range(N):
            qd[6 * i:6 * i + 3] = Q_P if i < N - 1 else Q_F
            qd[6 * i + 3:6 * i + 6] = Q_V if i < N - 1 else 0.0
        self.Sx, self.Su, self.Sd, self.qd = Sx, Su, Sd, qd
        self.H = Su.T @ (qd[:, None] * Su) + R_U * np.eye(3 * N)
        self.L = np.linalg.eigvalsh(self.H).max()

    def solve(self, x0, Xref, Uref, U0):
        """x0 (B,6); Xref (B,N,6) states at steps 1..N; Uref, U0 (B,N,3). Returns U (B,N,3)."""
        B, N = x0.shape[0], N_MPC
        free = x0 @ self.Sx.T + self.Sd                               # (B, 6N)
        f = ((free - Xref.reshape(B, -1)) * self.qd) @ self.Su - R_U * Uref.reshape(B, -1)
        U = U0.reshape(B, -1).copy()
        Y, tk = U.copy(), 1.0
        for _ in range(FISTA_ITERS):
            Un = Y - (Y @ self.H + f) / self.L
            Un = Un.reshape(B, N, 3)
            nrm = np.linalg.norm(Un, axis=2, keepdims=True)
            Un = np.where(nrm > A_MAX, Un * (A_MAX / np.maximum(nrm, 1e-12)), Un).reshape(B, -1)
            tn = 0.5 * (1 + np.sqrt(1 + 4 * tk * tk))
            Y = Un + (tk - 1) / tn * (Un - U)
            U, tk = Un, tn
        return U.reshape(B, N, 3)


def _horizon(ref, k0, stride):
    """Reference states/inputs at MPC steps 1..N (states) and 0..N-1 (inputs)."""
    n = len(ref['t'])
    i_x = np.minimum(k0 + stride * np.arange(1, N_MPC + 1), n - 1)
    i_u = np.minimum(k0 + stride * np.arange(0, N_MPC), n - 1)
    return np.hstack([ref['p'][i_x], ref['v'][i_x]]), ref['u'][i_u]


def _rate_ff(drone, u_now, u_next):
    """Body-rate feed-forward from the MPC's planned change of the thrust vector."""
    R = quat_to_rot(drone.quat)
    nrm = np.linalg.norm(u_now, axis=1, keepdims=True)
    z = u_now / np.maximum(nrm, 1e-6)
    du = (u_next - u_now) / DT_MPC
    zdot = (du - np.sum(du * z, axis=1, keepdims=True) * z) / np.maximum(nrm, 1e-6)
    wx = -np.sum(R[:, :, 1] * zdot, axis=1)
    wy = np.sum(R[:, :, 0] * zdot, axis=1)
    return np.column_stack([wx, wy, np.zeros(len(wx))])


# ── Simulation ────────────────────────────────────────────────────────────────
def simulate(model, refs):
    """Fly every reference in refs (one per layout) with the given plant model, batched."""
    B = len(refs)
    mpc = MPC()
    stride = N_SUB if model == 'rb' else 1
    dt_ref = DT_INNER if model == 'rb' else DT_MPC
    n_steps = max(len(r['t']) for r in refs) - 1
    # pad references to equal length (hold final state)
    L = n_steps + 1
    padded = []
    for r in refs:
        e = len(r['t'])
        pr = {k: np.concatenate([r[k], np.repeat(r[k][-1:], L - e, axis=0)])
              for k in ('p', 'v', 'u')}
        pr['t'] = np.arange(L) * dt_ref
        padded.append(pr)
    U = np.repeat(np.stack([r['u'][:N_MPC] for r in refs])[:, :, :], 1, axis=0)
    log_u, log_t = [], []
    if model == 'pm':
        p = np.repeat(START[None], B, axis=0)
        v = np.zeros((B, 3))
        P, V = [p.copy()], [v.copy()]
        for k in range(n_steps):
            hz = [_horizon(r, k, 1) for r in padded]
            Xr = np.stack([h[0] for h in hz])
            Ur = np.stack([h[1] for h in hz])
            U = mpc.solve(np.hstack([p, v]), Xr, Ur, np.concatenate([U[:, 1:], U[:, -1:]], 1))
            u = U[:, 0]
            v = v + (u + G_VEC - (C_DRAG / MASS) * v) * DT_MPC
            p = p + v * DT_MPC
            crash = p[:, 2] <= 0.0          # ground contact ends the run (crash)
            p[crash, 2] = 0.0
            v[crash] = 0.0
            assert np.all(np.linalg.norm(u, axis=1) <= A_MAX + 1e-9)
            P.append(p.copy())
            V.append(v.copy())
            log_u.append(u.copy())
        t = np.arange(n_steps + 1) * DT_MPC
        out = dict(t=t, pos=np.array(P), vel=np.array(V))
        out['u'] = np.array(log_u + [log_u[-1]])
        out['thrust'] = MASS * np.linalg.norm(out['u'], axis=2)
        return out
    # rigid body
    params = make_params('fpv_5inch', mass=MASS)
    drone = RigidBodyDrone(np.repeat(START[None], B, axis=0), preset='fpv_5inch',
                           params=params, gains=default_gains('fpv_5inch', max_tilt=MAX_TILT), dt=DT_INNER,
                           drag_ff=False, ground_z=0.0)
    u = U[:, 0]
    rate_ff = np.zeros((B, 3))
    for k in range(n_steps):
        if k % N_SUB == 0:
            hz = [_horizon(r, k, N_SUB) for r in padded]
            Xr = np.stack([h[0] for h in hz])
            Ur = np.stack([h[1] for h in hz])
            U = mpc.solve(np.hstack([drone.pos, drone.vel]), Xr, Ur,
                          np.concatenate([U[:, 1:], U[:, -1:]], 1))
            u = U[:, 0]
            rate_ff = _rate_ff(drone, U[:, 0], U[:, 1])
        drone.step_acc(u - GUP, yaw=0.0, rate_ff=rate_ff)
        log_u.append(u.copy())
    lg = drone.get_log()
    out = dict(t=lg['t'], pos=lg['pos'], vel=lg['vel'], quat=lg['quat'], omega=lg['omega'],
               rotor=lg['rotor'], thrust=lg['thrust'], power=lg['power'])
    out['u'] = np.array([log_u[0]] + log_u)
    out['n_sat'] = drone.n_sat
    out['energy'] = drone.energy.copy()
    out['w_max'] = params.w_max
    out['w_min'] = params.w_min
    out['thrust_max'] = params.thrust_max
    return out


def gate_events(pos, t, centres, normals, vel):
    """In-order gate passages. A passage is the closest approach (on the piecewise-linear path)
    during the first visit within R_SEARCH of the gate centre, searched after the previous
    passage. Returns miss (8,), passage time (8,), |cos| of the angle between the velocity and
    the gate normal (8,), and the index of the last passage."""
    miss = np.full(N_GATES, np.nan)
    tc = np.full(N_GATES, np.nan)
    cosn = np.full(N_GATES, np.nan)
    i0 = 0
    for k in range(N_GATES):
        a, b = pos[i0:-1], pos[i0 + 1:]
        ab = b - a
        lam = np.clip(np.sum((centres[k] - a) * ab, axis=1)
                      / np.maximum(np.sum(ab * ab, axis=1), 1e-12), 0.0, 1.0)
        d = np.linalg.norm(a + lam[:, None] * ab - centres[k], axis=1)
        near = np.where(d <= R_SEARCH)[0]
        if len(near) == 0:
            miss[k] = d.min() if len(d) else np.nan
            continue
        j0 = near[0]
        brk = np.where(d[j0:] > R_SEARCH)[0]
        j1 = j0 + (brk[0] if len(brk) else len(d) - j0)
        j = j0 + int(np.argmin(d[j0:j1]))
        miss[k] = d[j]
        tc[k] = t[i0 + j] + lam[j] * (t[i0 + j + 1] - t[i0 + j])
        v = vel[i0 + j] + lam[j] * (vel[i0 + j + 1] - vel[i0 + j])
        cosn[k] = abs(v @ normals[k]) / max(np.linalg.norm(v), 1e-9)
        i0 = i0 + j + 1
    return miss, tc, cosn, i0


def metrics(sim, b, centres, normals, T_ref):
    t = sim['t']
    pos, vel = sim['pos'][:, b], sim['vel'][:, b]
    assert np.all(np.isfinite(pos)) and np.all(pos[:, 2] >= 0.0), 'NaN / below ground'
    hit_ground = np.where(pos[:, 2] <= 1e-9)[0]
    crashed = len(hit_ground) > 0
    n_fly = hit_ground[0] + 1 if crashed else len(t)
    miss, tc, cosn, i_last = gate_events(pos[:n_fly], t[:n_fly], centres, normals,
                                         vel[:n_fly])
    back = np.where(np.linalg.norm(pos[i_last:n_fly] - START, axis=1) < END_RADIUS)[0]
    finished = (not crashed) and len(back) > 0 and np.all(np.isfinite(tc))
    i_end = i_last + back[0] if finished else n_fly - 1
    T_lap = t[i_end] if finished else np.nan
    spd = np.linalg.norm(vel[:i_end + 1], axis=1)
    un = np.linalg.norm(sim['u'][:i_end + 1, b], axis=1)
    dt = np.diff(t[:i_end + 1])
    m = dict(T_lap=T_lap, T_ref=T_ref, miss=miss, t_cross=tc, cos_normal=cosn,
             hits=int(np.sum(miss <= R_TOL)),
             crashed=crashed, finished=finished, t_crash=t[n_fly - 1] if crashed else np.nan,
             v_peak=spd.max(), eta_v=np.sum(dt * (spd[1:] > 0.8 * V_MAX)) / t[i_end],
             eta_a=np.sum(dt * (un[1:] > 0.9 * A_MAX)) / t[i_end], i_end=i_end,
             frac_above_vmax=np.sum(dt * (spd[1:] > V_MAX)) / t[i_end],
             out_of_arena=bool(np.any(np.abs(pos[:i_end + 1, :2]) > ARENA_XY / 2)
                               or np.any(pos[:i_end + 1, 2] > ARENA_Z)),
             max_dev=np.nan)
    if 'quat' in sim:
        q = sim['quat'][:i_end + 1, b]
        r = sim['rotor'][:i_end + 1, b]
        m['tilt_max'] = np.degrees(tilt_angle(q).max())
        m['rate_max'] = np.linalg.norm(sim['omega'][:i_end + 1, b], axis=1).max()
        m['rotor_max_frac'] = r.max() / sim['w_max']
        m['thrust_max_frac'] = sim['thrust'][:i_end + 1, b].max() / sim['thrust_max']
        m['energy'] = np.sum(0.5 * (sim['power'][1:i_end + 1, b] + sim['power'][:i_end, b]) * dt)
        assert np.all(r <= sim['w_max'] + 1e-9) and np.all(r >= sim['w_min'] - 1e-9)
    return m


def run_simulation():
    layouts = {s: generate_gates(s) for s in MC_SEEDS}
    splines, refs = {}, {(st, mo): [] for st in STRATS for mo in MODELS}
    for s in MC_SEEDS:
        c, _ = layouts[s]
        wp = np.vstack([START, c, START])
        T0 = np.linalg.norm(np.diff(wp, axis=0), axis=1) / V_REF_INIT
        coeffs, J = min_snap(wp, T0)
        splines[s] = (coeffs, T0, J)
        for st in STRATS:
            for mo in MODELS:
                refs[(st, mo)].append(build_reference(coeffs, T0, st,
                                                      DT_INNER if mo == 'rb' else DT_MPC))
    sims, res = {}, {}
    for st in STRATS:
        for mo in MODELS:
            sims[(st, mo)] = simulate(mo, refs[(st, mo)])
            res[(st, mo)] = [metrics(sims[(st, mo)], b, *layouts[s], refs[(st, mo)][b]['T_ref'])
                             for b, s in enumerate(MC_SEEDS)]
    return layouts, splines, refs, sims, res


# ── Plots ─────────────────────────────────────────────────────────────────────
def _gate_ring(c, n, r=R_GATE, k=40):
    a = np.array([1.0, 0, 0]) if abs(n[0]) < 0.9 else np.array([0, 1.0, 0])
    a = a - (a @ n) * n
    a /= np.linalg.norm(a)
    b = np.cross(n, a)
    th = np.linspace(0, 2 * np.pi, k)
    return c + r * np.outer(np.cos(th), a) + r * np.outer(np.sin(th), b)


def _draw_gates(ax, centres, normals):
    for k in range(N_GATES):
        ring = _gate_ring(centres[k], normals[k])
        ax.plot(ring[:, 0], ring[:, 1], ring[:, 2], color='grey', lw=2)
        ax.text(*(centres[k] + [0, 0, 0.7]), f'G{k + 1}', fontsize=8, color='dimgray',
                ha='center')
    ax.scatter(*START, color='green', marker='^', s=90, label='Start / finish')


def plot_circuit_3d(layouts, sims, res, out_dir):
    c, n = layouts[LAYOUT_SEED]
    fig = plt.figure(figsize=(11, 8))
    ax = fig.add_subplot(111, projection='3d')
    _draw_gates(ax, c, n)
    for st in STRATS:
        for mo in MODELS:
            m = res[(st, mo)][0]
            P = sims[(st, mo)]['pos'][:m['i_end'] + 1, 0]
            ax.plot(P[:, 0], P[:, 1], P[:, 2], M_STYLE[mo], color=S_COLOR[st],
                    lw=1.6 if mo == 'rb' else 1.2,
                    label=f"{S_LABEL[st]}, {M_LABEL[mo]} ("
                          + (f"{m['T_lap']:.2f} s" if np.isfinite(m['T_lap']) else 'DNF')
                          + f", {m['hits']}/8 gates)")
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_zlabel('z (m)')
    ax.set_xlim(-10, 10)
    ax.set_ylim(-10, 10)
    ax.set_zlim(0, 10)
    ax.set_title(f'S082 FPV racing circuit (layout seed {LAYOUT_SEED}): flown paths')
    ax.legend(fontsize=8, loc='upper left')
    ax.view_init(elev=28, azim=-60)
    path = os.path.join(out_dir, 'circuit_3d.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_metrics_panel(sims, res, out_dir):
    fig, axes = plt.subplots(3, 1, figsize=(15, 11))
    for st in STRATS:
        for mo in MODELS:
            m = res[(st, mo)][0]
            s = sims[(st, mo)]
            i = m['i_end'] + 1
            t = s['t'][:i]
            lab = f'{S_LABEL[st]}, {M_LABEL[mo]}'
            axes[0].plot(t, np.linalg.norm(s['vel'][:i, 0], axis=1), M_STYLE[mo],
                         color=S_COLOR[st], lw=1.3, label=lab)
            axes[1].plot(t, np.linalg.norm(s['u'][:i, 0], axis=1), M_STYLE[mo],
                         color=S_COLOR[st], lw=1.0, label=lab)
            if mo == 'rb':
                for k in range(N_GATES):
                    hit = m['miss'][k] <= R_TOL
                    axes[0].plot(m['t_cross'][k], V_MAX * 1.05, 'v' if hit else 'x',
                                 color=S_COLOR[st] if hit else 'k', ms=7)
    axes[0].axhline(V_MAX, color='grey', ls='--', lw=1, label=r'$v_{max}$ = 10 m/s')
    axes[0].set_ylabel('speed (m/s)')
    axes[0].set_title('Speed (markers: rigid-body gate passages, triangle = hit, x = miss)')
    axes[1].axhline(A_MAX, color='grey', ls='--', lw=1, label=r'$a_{max}$ = 15 m/s$^2$')
    axes[1].set_ylabel(r'MPC command $|u|$ (m/s$^2$)')
    axes[1].set_title('Thrust-acceleration command')
    for ax in axes[:2]:
        ax.set_xlabel('time (s)')
        ax.grid(alpha=0.3)
        ax.legend(fontsize=7, loc='upper left', bbox_to_anchor=(1.01, 1.0))
    w = 0.13
    x = np.arange(N_GATES)
    for j, (st, mo) in enumerate([(s_, m_) for s_ in STRATS for m_ in MODELS]):
        miss = res[(st, mo)][0]['miss']
        axes[2].bar(x + (j - 2.5) * w, 100 * miss, w, color=S_COLOR[st],
                    alpha=0.45 if mo == 'pm' else 0.95, label=f'{S_LABEL[st]}, {M_LABEL[mo]}')
    axes[2].axhline(100 * R_TOL, color='red', ls='--', lw=1.2, label=r'$r_{tol}$ = 0.2 m')
    axes[2].set_xticks(x)
    axes[2].set_xticklabels([f'G{k + 1}' for k in range(N_GATES)])
    axes[2].set_ylabel('miss distance (cm)')
    axes[2].set_title('Closest approach to each gate centre')
    axes[2].legend(fontsize=7, loc='upper left', bbox_to_anchor=(1.01, 1.0))
    axes[2].grid(alpha=0.3, axis='y')
    plt.tight_layout()
    path = os.path.join(out_dir, 'metrics_panel.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_rigid_body_states(sims, res, out_dir):
    st = 'topp'
    s = sims[(st, 'rb')]
    m = res[(st, 'rb')][0]
    i = m['i_end'] + 1
    t = s['t'][:i]
    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    axes[0].plot(t, np.degrees(tilt_angle(s['quat'][:i, 0])), color='tab:red', lw=1.2,
                 label='tilt (rigid body)')
    u = s['u'][:i, 0]
    tilt_cmd = np.degrees(np.arccos(np.clip(u[:, 2] / np.linalg.norm(u, axis=1), -1, 1)))
    axes[0].plot(t, tilt_cmd, color='k', ls=':', lw=1.0,
                 label='tilt of MPC thrust vector (= point-mass attitude)')
    axes[0].set_ylabel('tilt (deg)')
    axes[0].legend(fontsize=8)
    axes[1].plot(t, np.linalg.norm(s['omega'][:i, 0], axis=1), color='tab:red', lw=1.0)
    axes[1].set_ylabel(r'$|\omega|$ (rad/s)')
    r = s['rotor'][:i, 0]
    for j in range(4):
        axes[2].plot(t, r[:, j] / s['w_max'], lw=0.8, label=f'motor {j}')
    axes[2].axhline(1.0, color='grey', ls='--', lw=1)
    axes[2].set_ylabel(r'rotor speed / $\omega_{max}$')
    axes[2].set_xlabel('time (s)')
    axes[2].legend(fontsize=8, ncol=4)
    for ax in axes:
        ax.grid(alpha=0.3)
    axes[0].set_title(f'Rigid-body states on the time-optimal lap (seed {LAYOUT_SEED})')
    plt.tight_layout()
    path = os.path.join(out_dir, 'rigid_body_states.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_monte_carlo(res, out_dir):
    fig, axes = plt.subplots(1, 3, figsize=(18, 5.2))
    keys = [(s_, m_) for s_ in STRATS for m_ in MODELS]
    short = {'minsnap': 'Min-snap', 'scaled': 'Scaled', 'topp': 'TOPP'}
    labels = [f"{short[s_]}\n{'PM' if m_ == 'pm' else 'RB'}" for s_, m_ in keys]
    data_t = [[r['T_lap'] for r in res[k] if np.isfinite(r['T_lap'])] for k in keys]
    data_m = [np.maximum(np.concatenate([r['miss'] for r in res[k]]) * 100, 0.1) for k in keys]
    data_h = [[r['hits'] for r in res[k]] for k in keys]
    for ax, data, yl in ((axes[0], data_t, 'lap time (s)'), (axes[1], data_m, 'miss distance (cm)')):
        bp = ax.boxplot(data, patch_artist=True, widths=0.6)
        for patch, (s_, m_) in zip(bp['boxes'], keys):
            patch.set_facecolor(S_COLOR[s_])
            patch.set_alpha(0.4 if m_ == 'pm' else 0.85)
        ax.set_xticks(range(1, len(keys) + 1))
        ax.set_xticklabels(labels, fontsize=8)
        ax.set_ylabel(yl)
        ax.grid(alpha=0.3, axis='y')
    axes[1].axhline(100 * R_TOL, color='red', ls='--', lw=1.2, label=r'$r_{tol}$')
    axes[1].set_yscale('log')
    axes[1].set_ylabel('miss distance (cm, log)')
    axes[1].legend(fontsize=8)
    axes[0].set_title(f'Lap time over {len(MC_SEEDS)} layouts (finished laps only)')
    axes[1].set_title(f'All {len(MC_SEEDS) * N_GATES} gate passages (PM = point mass, '
                      'RB = rigid body)')
    for j, (k, h) in enumerate(zip(keys, data_h)):
        axes[2].bar(j, np.mean(h), color=S_COLOR[k[0]], alpha=0.4 if k[1] == 'pm' else 0.85)
        axes[2].text(j, np.mean(h) + 0.05, f'{np.mean(h):.2f}', ha='center', fontsize=9)
    axes[2].set_xticks(range(len(keys)))
    axes[2].set_xticklabels(labels, fontsize=8)
    axes[2].set_ylim(0, 8.8)
    axes[2].set_ylabel('mean gates hit (of 8)')
    axes[2].set_title(r'Gates hit within $r_{tol}$ = 0.2 m')
    axes[2].grid(alpha=0.3, axis='y')
    plt.tight_layout()
    path = os.path.join(out_dir, 'monte_carlo.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def save_animation(layouts, sims, res, refs, out_dir, step=30):
    """Time-optimal reference, seed 42, flown by the point mass (black) and the rigid body
    (red, with body axes) over T_ref + 1 s."""
    c, n = layouts[LAYOUT_SEED]
    s = sims[('topp', 'rb')]
    spm = sims[('topp', 'pm')]
    t_show = refs[('topp', 'rb')][0]['T_ref'] + 1.0
    n_rb = min(int(round(t_show / DT_INNER)), len(s['t']) - 1)
    idx = np.unique(np.minimum(np.arange(0, n_rb + step, step), n_rb))
    P = s['pos'][:, 0]
    Ppm = spm['pos'][:, 0]
    R = quat_to_rot(s['quat'][:, 0])
    fig = plt.figure(figsize=(8, 7))
    ax = fig.add_subplot(111, projection='3d')
    _draw_gates(ax, c, n)
    ax.set_xlim(-10, 10)
    ax.set_ylim(-10, 10)
    ax.set_zlim(0, 10)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_zlabel('z (m)')
    ax.view_init(elev=28, azim=-60)
    trail_pm, = ax.plot([], [], [], ':', color='k', lw=1.3, label='point mass')
    dot_pm, = ax.plot([], [], [], 'o', color='k', ms=5)
    trail, = ax.plot([], [], [], color='tab:red', lw=1.5, label='rigid body')
    dot, = ax.plot([], [], [], 'o', color='tab:red', ms=7)
    arms = [ax.plot([], [], [], color=col, lw=2)[0] for col in ('k', 'k', 'tab:green')]
    ax.legend(fontsize=8, loc='upper left')

    def update(f):
        i = idx[f]
        j = min(int(round(s['t'][i] / DT_MPC)), len(Ppm) - 1)
        trail_pm.set_data(Ppm[:j + 1, 0], Ppm[:j + 1, 1])
        trail_pm.set_3d_properties(Ppm[:j + 1, 2])
        dot_pm.set_data([Ppm[j, 0]], [Ppm[j, 1]])
        dot_pm.set_3d_properties([Ppm[j, 2]])
        trail.set_data(P[:i + 1, 0], P[:i + 1, 1])
        trail.set_3d_properties(P[:i + 1, 2])
        dot.set_data([P[i, 0]], [P[i, 1]])
        dot.set_3d_properties([P[i, 2]])
        for k, a in enumerate(arms):
            e = R[i][:, k] * (1.2 if k < 2 else 1.5)
            seg = np.vstack([P[i] - (e if k < 2 else 0), P[i] + e])
            a.set_data(seg[:, 0], seg[:, 1])
            a.set_3d_properties(seg[:, 2])
        ax.set_title(f'S082 time-optimal reference  t = {s["t"][i]:.2f} s\n'
                     f'rigid body |v| = {np.linalg.norm(s["vel"][i, 0]):.1f} m/s, tilt '
                     f'{np.degrees(tilt_angle(s["quat"][i:i + 1, 0])[0]):.0f} deg')
        return [trail_pm, dot_pm, trail, dot] + arms

    anim = animation.FuncAnimation(fig, update, frames=len(idx), interval=40, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    anim.save(path, writer='pillow', fps=25, dpi=80)
    plt.close(fig)
    print(f'Saved: {path} ({len(idx)} frames)')


# ── Main ──────────────────────────────────────────────────────────────────────
def _fmt(x, f='{:.2f}'):
    return 'nan' if not np.isfinite(x) else f.format(x)


if __name__ == '__main__':
    layouts, splines, refs, sims, res = run_simulation()
    out = os.path.normpath(OUTPUT_DIR)
    os.makedirs(out, exist_ok=True)

    print('\n=== S082 FPV Racing Circuit - Results ===')
    c42, n42 = layouts[LAYOUT_SEED]
    coeffs, T0, J = splines[LAYOUT_SEED]
    wp = np.vstack([START, c42, START])
    print(f'  Layout seed {LAYOUT_SEED}: path through 8 gates, {len(T0)} segments, '
          f'straight-line length {np.linalg.norm(np.diff(wp, axis=0), axis=1).sum():.1f} m, '
          f'snap cost J = {J:.1f}')
    print('  Gate centres (m):', '; '.join(f'G{k + 1} ({g[0]:.1f}, {g[1]:.1f}, {g[2]:.1f})'
                                          for k, g in enumerate(c42)))
    tp = refs[('topp', 'pm')][0]['topp']
    for st in STRATS:
        r = refs[(st, 'rb')][0]
        i = r['t'] <= r['T_ref']
        print(f"  Reference [{S_LABEL[st]}]: T_ref = {r['T_ref']:.2f} s, peak speed "
              f"{np.linalg.norm(r['v'][i], axis=1).max():.2f} m/s, peak |u| "
              f"{np.linalg.norm(r['u'][i], axis=1).max():.2f} m/s^2, peak |a| "
              f"{np.linalg.norm(r['a'][i], axis=1).max():.2f} m/s^2")
    print(f"  TOPP: {N_ARC} grid points over {tp['length']:.1f} m of spline; T_topp / T_minsnap = "
          f"{tp['T'] / T0.sum():.3f}; uniform feasible scale k = "
          f"{refs[('scaled', 'pm')][0]['scale']:.3f}")
    print(f'  Rigid body: fpv_5inch preset with m = {MASS} kg, '
          f"T/W = {make_params('fpv_5inch', mass=MASS).t2w:.2f}, inner loop "
          f'{1 / DT_INNER:.0f} Hz, MPC {1 / DT_MPC:.0f} Hz')
    print(f'\n  Headline layout (seed {LAYOUT_SEED}):')
    print('  strategy             model        T_ref   T_lap  hits  GMR    v_peak  '
          'eta_v  eta_a  max miss')
    for st in STRATS:
        for mo in MODELS:
            m = res[(st, mo)][0]
            print(f"  {S_LABEL[st]:20s} {M_LABEL[mo]:11s} {m['T_ref']:6.2f}  "
                  f"{_fmt(m['T_lap']):>6s}  {m['hits']}/8  "
                  f"{100 * (N_GATES - m['hits']) / N_GATES:5.1f}%  {m['v_peak']:5.2f}  "
                  f"{100 * m['eta_v']:5.1f}% {100 * m['eta_a']:5.1f}%  "
                  f"{100 * np.nanmax(m['miss']):.1f} cm")
            print('      miss per gate (cm): ' + ', '.join(f'{100 * x:.1f}' for x in m['miss']))
    for st in STRATS:
        m = res[(st, 'rb')][0]
        print(f"  Rigid body [{S_LABEL[st]}]: max tilt {m['tilt_max']:.1f} deg, max |omega| "
              f"{m['rate_max']:.2f} rad/s, max rotor {100 * m['rotor_max_frac']:.1f} % of "
              f"w_max, max thrust {100 * m['thrust_max_frac']:.1f} % of T_max, shaft energy "
              f"{m['energy']:.0f} J, time above v_max {100 * m['frac_above_vmax']:.1f} %, "
              f"left arena: {m['out_of_arena']}")
    print(f"  Mixer desaturation steps (all {len(MC_SEEDS)} layouts): " + ', '.join(
        f"{S_LABEL[st]} {sims[(st, 'rb')]['n_sat']}" for st in STRATS))

    print(f'\n  Monte Carlo over {len(MC_SEEDS)} layouts (seeds {MC_SEEDS[0]}-{MC_SEEDS[-1]}):')
    print('  strategy                 model        finished crashed  T_lap mean+-std (finished)'
          '  hits mean  all-8  miss mean / p95 / max (cm)')
    for st in STRATS:
        for mo in MODELS:
            R_ = res[(st, mo)]
            tl = np.array([r['T_lap'] for r in R_])
            h = np.array([r['hits'] for r in R_])
            ms = np.concatenate([r['miss'] for r in R_]) * 100
            print(f"  {S_LABEL[st]:24s} {M_LABEL[mo]:11s}  {np.sum(np.isfinite(tl)):2d}/{len(R_)}"
                  f"    {sum(r['crashed'] for r in R_):2d}     {_fmt(np.nanmean(tl)):>6s} +- "
                  f"{_fmt(np.nanstd(tl))} s            {h.mean():.2f}    "
                  f"{np.sum(h == N_GATES):2d}/{len(R_)}  {ms.mean():.1f} / "
                  f"{np.percentile(ms, 95):.1f} / {ms.max():.1f}")
            crashes = [f"seed {MC_SEEDS[i]} at {r['t_crash']:.2f} s" for i, r in enumerate(R_)
                       if r['crashed']]
            if crashes:
                print('      crashes: ' + ', '.join(crashes))
    for mo in MODELS:
        b = np.array([r['T_lap'] for r in res[('topp', mo)]])
        for base in ('minsnap', 'scaled'):
            a = np.array([r['T_lap'] for r in res[(base, mo)]])
            both = np.isfinite(a) & np.isfinite(b)
            if not np.any(both):
                print(f"  [{M_LABEL[mo]}] time-optimal vs {S_LABEL[base]}: no layout finished "
                      f"by both")
                continue
            print(f"  [{M_LABEL[mo]}] time-optimal vs {S_LABEL[base]} on the {np.sum(both)} "
                  f"layouts both finished: faster on {np.sum(b[both] < a[both])}; lap-time ratio "
                  f"{np.mean(b[both] / a[both]):.3f} +- {np.std(b[both] / a[both]):.3f}")
    sc = np.array([refs[('scaled', 'pm')][b]['scale'] for b in range(len(MC_SEEDS))])
    tr = np.array([refs[('topp', 'pm')][b]['T_ref'] / refs[('minsnap', 'pm')][b]['T_ref']
                   for b in range(len(MC_SEEDS))])
    tf = np.array([refs[('minsnap', 'pm')][b]['T_ref'] for b in range(len(MC_SEEDS))])
    uf = np.array([np.linalg.norm(refs[('minsnap', 'pm')][b]['u'], axis=1).max()
                   for b in range(len(MC_SEEDS))])
    print(f"  References over layouts: min-snap T_ref {tf.mean():.2f} +- {tf.std():.2f} s, peak |u| "
          f"{uf.min():.1f}-{uf.max():.1f} m/s^2 (> A_MAX on {np.sum(uf > A_MAX)}/{len(uf)}); "
          f"feasible uniform scale k = {sc.min():.3f}-{sc.max():.3f}; T_topp / T_minsnap = "
          f"{tr.mean():.3f} +- {tr.std():.3f}")
    for st in STRATS:
        a = np.array([r['hits'] for r in res[(st, 'pm')]])
        b = np.array([r['hits'] for r in res[(st, 'rb')]])
        ma = np.concatenate([r['miss'] for r in res[(st, 'pm')]])
        mb = np.concatenate([r['miss'] for r in res[(st, 'rb')]])
        print(f"  [{S_LABEL[st]}] rigid body vs point mass: fewer hits on {np.sum(b < a)}, "
              f"equal on {np.sum(b == a)}, more on {np.sum(b > a)} layouts; mean miss "
              f"{100 * ma.mean():.1f} -> {100 * mb.mean():.1f} cm; larger miss at "
              f"{np.sum(mb > ma)}/{len(ma)} gates")
    for st in STRATS:
        dirr, down, umin = [], [], []
        for r in refs[(st, 'rb')]:
            i = r['t'] <= r['T_ref']
            u = r['u'][i]
            z = u / np.linalg.norm(u, axis=1, keepdims=True)
            ang = np.arccos(np.clip(np.sum(z[1:] * z[:-1], axis=1), -1, 1)) / DT_INNER
            dirr.append(ang.max())
            down.append(np.mean(u[:, 2] < 0.0))
            umin.append(np.degrees(np.arccos(np.clip(z[:, 2], -1, 1))).max())
        print(f"  Reference [{S_LABEL[st]}] over layouts: max thrust-direction rate "
              f"{np.min(dirr):.1f}-{np.max(dirr):.1f} rad/s (rate limit "
              f"{default_gains('fpv_5inch')['max_rate'][0]:.0f} rad/s), max thrust tilt "
              f"{np.min(umin):.0f}-{np.max(umin):.0f} deg, time with downward thrust "
              f"{100 * np.mean(down):.1f} %")
    for st in STRATS:
        cn = np.concatenate([r['cos_normal'] for r in res[(st, 'pm')]])
        ang = np.degrees(np.arccos(np.clip(cn[np.isfinite(cn)], 0, 1)))
        print(f"  [{S_LABEL[st]}, point mass] angle between flight direction and gate normal at "
              f"the passage: median {np.median(ang):.0f} deg, > 45 deg at {np.sum(ang > 45)}/"
              f"{len(ang)} gates, max {ang.max():.0f} deg")
    for k in [(s_, m_) for s_ in STRATS for m_ in MODELS]:
        n_out = sum(r['out_of_arena'] for r in res[k])
        print(f"  [{S_LABEL[k[0]]}, {M_LABEL[k[1]]}] layouts leaving the 20x20x10 m arena: "
              f"{n_out}/{len(MC_SEEDS)}")
    tm = np.array([r['tilt_max'] for r in res[('topp', 'rb')]])
    print(f"  [rigid body, time-optimal] max tilt over layouts {tm.min():.1f}-{tm.max():.1f} deg")

    plot_circuit_3d(layouts, sims, res, out)
    plot_metrics_panel(sims, res, out)
    plot_rigid_body_states(sims, res, out)
    plot_monte_carlo(res, out)
    save_animation(layouts, sims, res, refs, out)
