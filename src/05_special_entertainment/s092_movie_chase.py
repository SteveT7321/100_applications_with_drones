"""
S092 Movie Car Chase Scene
==========================
A camera drone follows a stunt car along a winding road (x = 8 t, y = 15 sin(2 pi x / 80))
and tries to hold a cinematic offset 8 m behind and 5 m above the car while flying around
five concrete pillars. The car position is measured with 0.3 m noise at 20 Hz and filtered
by a constant-velocity Kalman filter. A 10-step MPC (dt 0.2 s, horizon 2 s) tracks a
reference built from the KF prediction over the next 2 s, with linearised pillar
half-plane constraints, per-axis speed and acceleration limits.

Variants (same measurement noise per seed):
    kf_mpc        KF-predicted reference over the horizon + obstacle constraints (proposed)
    no_pred       reference held at the current KF estimate (no prediction)
    oracle        true future reference (perfect knowledge, upper bound)
    kf_noobs      proposed without obstacle constraints
    kf_card       proposed with the card's original pillar layout

The MPC QP (accelerations over the horizon plus one slack per pillar, i.e. soft
obstacle constraints) is solved with a small dense primal-dual interior-point method.

Usage:
    conda activate drones
    python src/05_special_entertainment/s092_movie_chase.py
"""

import os
import math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Circle
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401

# ── Parameters ────────────────────────────────────────────────────────────────
ROAD_X_LEN  = 300.0       # m  road extent along x
CAR_VX      = 8.0         # m/s forward (x) speed; along-road speed 8.0-12.4 m/s
A_WAVE      = 15.0        # m
L_WAVE      = 80.0        # m
T_TOTAL     = ROAD_X_LEN / CAR_VX            # 37.5 s
D_OFFSET    = np.array([-8.0, 0.0, 5.0])     # body frame: x forward, y left, z up
H_CAR       = 1.5         # m
FOV_DEG     = 60.0
BAND_CARD   = (0.20, 0.40)                   # card framing band
BAND_FIXED  = (0.10, 0.20)                   # corrected band (contains the nominal 15.2 %)
TOL_BALL    = 1.0         # m tracking tolerance ball

PILLAR_RADIUS = 0.6       # m physical
PILLAR_HEIGHT = 6.0       # m
R_SAFE        = 1.5       # m MPC clearance
PILLARS_CARD  = np.array([[40.0, 8.0], [90.0, -10.0], [150.0, 12.0],
                          [210.0, -6.0], [260.0, 9.0]])
PILLAR_LATERAL = 0.5      # m offset of relocated pillars from the desired camera path

DT_SIM   = 0.05           # s simulation and KF rate (20 Hz)
SIGMA_MEAS = 0.3          # m
Q_NOISE  = 1.0            # m^2/s^3
T_PRED   = 2.0            # s
DT_MPC   = 0.2            # s
N_HOR    = 10
SUB      = int(round(DT_MPC / DT_SIM))        # 4 sim steps per MPC step
Q_MPC    = 10.0
R_MPC    = 0.1
P_F      = 20.0
Q_VEL    = 10.0           # velocity-reference weight (added; card cost uses 0)
V_MAX    = 12.0           # m/s per axis
A_MAX    = 6.0            # m/s^2 per axis
W_SLACK_Q = 1e4           # slack penalties (soft obstacle constraints)
W_SLACK_L = 1e3
OBS_RANGE = 40.0          # m pillars farther than this are not added to the QP
T_SKIP   = 3.0            # s start-up transient excluded from the "steady" RMS
N_MC     = 10
SEED_FREE = ('oracle', 'oracle_card')   # no measurement noise -> run once
OFFSET_SWEEP = (0.5, 1.0, 1.5, 2.0, 3.0)   # m pillar distance from the desired camera path
QV_SWEEP = (1.0, 3.0, 10.0, 30.0, 100.0)  # velocity-tracking weight sensitivity
N_WORKERS = 6                             # process pool; results collected in input order
CLEAR_SUB = 10                            # clearance check points per 0.05 s step
SEEDS    = list(range(N_MC))
MAIN_SEED = 0

VARIANTS = ['kf_mpc', 'kf_knots', 'kf_cardcost', 'no_pred', 'oracle', 'oracle_card',
            'kf_noobs', 'kf_cardlin', 'kf_card']
V_LABEL = {'kf_mpc': 'KF prediction + MPC (proposed)',
           'kf_knots': 'Proposed, pillar constraints only at the 0.2 s knots',
           'kf_cardcost': 'KF prediction + card MPC cost (no velocity term)',
           'no_pred': 'No prediction (hold KF estimate)',
           'oracle': 'Oracle future reference',
           'oracle_card': 'Oracle reference + card MPC cost',
           'kf_noobs': 'Proposed, no obstacle constraints',
           'kf_cardlin': 'Proposed, card linearisation (at p0)',
           'kf_card': 'Proposed, card pillar layout'}
V_SHORT = {'kf_mpc': 'proposed', 'kf_knots': 'knots\nonly', 'kf_cardcost': 'card\ncost', 'no_pred': 'no\nprediction',
           'oracle': 'oracle', 'oracle_card': 'oracle,\ncard cost', 'kf_noobs': 'no obstacle\nconstraints',
           'kf_cardlin': 'card\nlinearisation', 'kf_card': 'card\npillars'}
V_COLOR = {'kf_mpc': 'red', 'kf_knots': 'tab:olive', 'kf_cardcost': 'tab:pink', 'no_pred': 'tab:purple',
           'oracle': 'black', 'oracle_card': 'tab:gray', 'kf_noobs': 'tab:orange',
           'kf_cardlin': 'tab:cyan', 'kf_card': 'tab:brown'}

OUTPUT_DIR = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', '..',
    'outputs', '05_special_entertainment', 's092_movie_chase'))


# ── Car and desired camera position ───────────────────────────────────────────

def car_state(t):
    """Position (x, y, 0), velocity (vx, vy) and heading of the car (vectorised in t)."""
    t = np.asarray(t, dtype=float)
    x = CAR_VX * t
    k = 2 * np.pi / L_WAVE
    y = A_WAVE * np.sin(k * x)
    vx = np.full_like(t, CAR_VX)
    vy = CAR_VX * A_WAVE * k * np.cos(k * x)
    psi = np.arctan2(vy, vx)
    return np.stack([x, y, np.zeros_like(t)], -1), np.stack([vx, vy], -1), psi


def offset_point(p_car_xy, psi):
    c, s = np.cos(psi), np.sin(psi)
    dx = c * D_OFFSET[0] - s * D_OFFSET[1]
    dy = s * D_OFFSET[0] + c * D_OFFSET[1]
    return np.stack([p_car_xy[..., 0] + dx, p_car_xy[..., 1] + dy,
                     np.full_like(dx, D_OFFSET[2])], -1)


def desired_pos(t):
    p, _, psi = car_state(t)
    return offset_point(p[..., :2], psi)


def relocated_pillars(lateral=PILLAR_LATERAL):
    """Pillars at the card's x stations, moved onto the desired camera path (lateral m to
    the side, alternating), so that the 1.5 m clearance constraint actually binds."""
    tt = np.linspace(-1.0, T_TOTAL + 1.0, 200001)
    pd = desired_pos(tt)
    out = []
    for k, xs in enumerate(PILLARS_CARD[:, 0]):
        i = int(np.argmin(np.abs(pd[:, 0] - xs)))
        tan = pd[min(i + 1, len(tt) - 1), :2] - pd[max(i - 1, 0), :2]
        tan /= np.linalg.norm(tan)
        nrm = np.array([-tan[1], tan[0]]) * (1 if k % 2 == 0 else -1)
        out.append(pd[i, :2] + lateral * nrm)
    return np.array(out)


PILLARS = relocated_pillars()


# ── Kalman filter ─────────────────────────────────────────────────────────────

def kf_matrices(dt=DT_SIM):
    F = np.block([[np.eye(2), dt * np.eye(2)], [np.zeros((2, 2)), np.eye(2)]])
    Q = Q_NOISE * np.block([[(dt ** 3 / 3) * np.eye(2), (dt ** 2 / 2) * np.eye(2)],
                            [(dt ** 2 / 2) * np.eye(2), dt * np.eye(2)]])
    H = np.hstack([np.eye(2), np.zeros((2, 2))])
    return F, Q, H


F_KF, Q_KF, H_KF = kf_matrices()
R_KF = SIGMA_MEAS ** 2 * np.eye(2)


# ── MPC (condensed QP) ────────────────────────────────────────────────────────

_IDX = np.arange(1, N_HOR + 1)
GP = np.zeros((N_HOR, N_HOR))         # p_i = p0 + i dt v0 + GP u
GV = np.zeros((N_HOR, N_HOR))         # v_i = v0 + GV u
for _i in range(1, N_HOR + 1):
    for _j in range(_i):
        GP[_i - 1, _j] = DT_MPC ** 2 * (_i - _j - 0.5)
        GV[_i - 1, _j] = DT_MPC
W_POS = np.array([Q_MPC] * (N_HOR - 1) + [P_F])
N_C = N_HOR * SUB                      # obstacle constraint points: every 0.05 s of the horizon
TAU_C = np.arange(1, N_C + 1) * DT_SIM
GS = np.zeros((N_C, N_HOR))            # p(tau_j) = p0 + tau_j v0 + GS u (exact, ZOH accel)
for _j in range(N_C):
    _i = _j // SUB
    _s = TAU_C[_j] - _i * DT_MPC
    for _m in range(_i):
        GS[_j, _m] = DT_MPC ** 2 * (_i - _m - 0.5) + DT_MPC * _s
    GS[_j, _i] = 0.5 * _s ** 2
KNOT_ROWS = np.arange(SUB - 1, N_C, SUB)   # rows of GS that coincide with the knots


def qp_ipm(H, g, G, h, tol=1e-8, max_iter=60):
    """Dense primal-dual interior-point method (Mehrotra predictor-corrector) for
    min 0.5 x'Hx + g'x  s.t.  Gx <= h.  Returns (x, converged, iterations)."""
    n, m = H.shape[0], G.shape[0]
    x = np.zeros(n)
    s = np.maximum(h - G @ x, 1.0)
    z = np.ones(m)
    for it in range(1, max_iter + 1):
        r_d = H @ x + g + G.T @ z
        r_p = G @ x + s - h
        mu = s @ z / m
        if (np.linalg.norm(r_d, np.inf) < tol * (1 + np.linalg.norm(g, np.inf)) and
                np.linalg.norm(r_p, np.inf) < tol * (1 + np.linalg.norm(h, np.inf)) and
                mu < tol):
            return x, True, it
        w = z / s
        M = H + G.T @ (w[:, None] * G)
        M += 1e-10 * (1.0 + np.abs(np.diag(H)).max()) * np.eye(n)
        if not np.all(np.isfinite(M)):
            return x, False, it

        def solve(r_c):
            rhs = -r_d - G.T @ ((z * r_p - r_c) / s)
            try:
                dx = np.linalg.solve(M, rhs)
            except np.linalg.LinAlgError:
                dx = np.linalg.lstsq(M, rhs, rcond=None)[0]
            ds = -r_p - G @ dx
            dz = (-r_c - z * ds) / s
            return dx, ds, dz

        def step_len(v, dv):
            neg = dv < 0
            return min(1.0, float(np.min(-v[neg] / dv[neg]))) if neg.any() else 1.0

        dx, ds, dz = solve(s * z)
        a_aff = min(step_len(s, ds), step_len(z, dz))
        mu_aff = (s + a_aff * ds) @ (z + a_aff * dz) / m
        sigma = (mu_aff / mu) ** 3
        dx, ds, dz = solve(s * z + ds * dz - sigma * mu)
        alpha = 0.99 * min(step_len(s, ds), step_len(z, dz))
        x, s, z = x + alpha * dx, s + alpha * ds, z + alpha * dz
    return x, False, max_iter


def solve_mpc(p0, v0, ref_full, pillars, use_obs, p_lin=None, q_vel=Q_VEL, knots_only=False):
    """Condensed MPC QP in the accelerations u (3 x N) plus one slack per nearby pillar.
    ref_full: (N_HOR + 1, 3) references at tau = 0..N; positions p_1..p_N track ref_full[1:]
    and, if q_vel > 0, velocities v_1..v_N track the reference finite differences.
    Obstacle half-planes are imposed at every 0.05 s point of the horizon (N_C points, or only
    at the N_HOR knots if knots_only) and linearised at p_lin (N_C, 2), one normal per point;
    p_lin=None uses the current position p0 for every point (card).
    Returns (u0, predicted positions at the N_C points, (converged, slack > 0))."""
    ref = ref_full[1:]
    v_ref = np.diff(ref_full, axis=0) / DT_MPC
    free = p0[None, :] + _IDX[:, None] * DT_MPC * v0[None, :]           # (N, 3)
    E = ref - free
    rows_c = KNOT_ROWS if knots_only else np.arange(N_C)
    GC = GS[rows_c]
    free_c = p0[None, :] + TAU_C[rows_c, None] * v0[None, :]
    obs = []
    if use_obs:
        lin = np.repeat(p0[None, :2], N_C, 0) if p_lin is None else p_lin
        lin = lin[rows_c]
        for o in pillars:
            if np.linalg.norm(p0[:2] - o) < OBS_RANGE:
                d = lin - o
                obs.append((d / np.linalg.norm(d, axis=1, keepdims=True), o))
    K = len(obs)
    nU = 3 * N_HOR
    nz = nU + K
    H = np.zeros((nz, nz))
    g = np.zeros(nz)
    G2 = GP.T @ (W_POS[:, None] * GP) + R_MPC * np.eye(N_HOR)
    for a in range(3):
        sl = slice(a * N_HOR, (a + 1) * N_HOR)
        H[sl, sl] = 2 * G2 + 2 * q_vel * GV.T @ GV
        g[sl] = -2 * GP.T @ (W_POS * E[:, a]) - 2 * q_vel * GV.T @ (v_ref[:, a] - v0[a])
    for k in range(K):
        H[nU + k, nU + k] = 2 * W_SLACK_Q
        g[nU + k] = W_SLACK_L
    rows, rhs = [], []
    for a in range(3):                                   # |v_i| <= V_MAX, |u_i| <= A_MAX
        for sign in (1.0, -1.0):
            C = np.zeros((N_HOR, nz))
            C[:, a * N_HOR:(a + 1) * N_HOR] = sign * GV
            rows.append(C)
            rhs.append(V_MAX - sign * np.full(N_HOR, v0[a]))
            C = np.zeros((N_HOR, nz))
            C[:, a * N_HOR:(a + 1) * N_HOR] = sign * np.eye(N_HOR)
            rows.append(C)
            rhs.append(np.full(N_HOR, A_MAX))
    for k, (n, o) in enumerate(obs):                     # n.(p(tau) - o) + s_k >= R_SAFE
        C = np.zeros((len(rows_c), nz))
        C[:, 0:N_HOR] = -n[:, 0:1] * GC
        C[:, N_HOR:2 * N_HOR] = -n[:, 1:2] * GC
        C[:, nU + k] = -1.0
        rows.append(C)
        rhs.append(np.sum((free_c[:, :2] - o) * n, axis=1) - R_SAFE)
        C = np.zeros((1, nz))
        C[0, nU + k] = -1.0                              # s_k >= 0
        rows.append(C)
        rhs.append(np.zeros(1))
    z, ok, _ = qp_ipm(H, g, np.vstack(rows), np.concatenate(rhs))
    u = z[:nU].reshape(3, N_HOR)
    slack = bool(K and z[nU:].max() > 1e-6)
    pred = p0[None, :] + TAU_C[:, None] * v0[None, :] + GS @ u.T
    return np.clip(u[:, 0], -A_MAX, A_MAX), pred, (ok, slack)


# ── Simulation ────────────────────────────────────────────────────────────────

def framing(dist):
    return 2 * np.arctan(H_CAR / (2 * dist)) / np.radians(FOV_DEG)


def simulate(variant, seed, keep=False, lateral=PILLAR_LATERAL, q_vel=None):
    rng = np.random.default_rng(seed)
    n_steps = int(round(T_TOTAL / DT_SIM))
    t_all = np.arange(n_steps + 1) * DT_SIM
    noise = rng.normal(0.0, SIGMA_MEAS, (n_steps + 1, 2))      # common random numbers
    pillars = PILLARS_CARD if variant == 'kf_card' else relocated_pillars(lateral)
    use_obs = variant != 'kf_noobs'
    p_car_all, v_car_all, psi_all = car_state(t_all)
    p_des_all = offset_point(p_car_all[:, :2], psi_all)

    p = p_des_all[0].copy()
    v = np.zeros(3)
    u = np.zeros(3)
    x_kf = None
    P_kf = np.eye(4)
    pred_prev = None
    n_pred = int(math.floor(T_PRED / DT_SIM + 1e-9))
    log = {k: [] for k in ('p', 'v', 'u', 'kf', 'pred2')}
    info_count = {'fail': 0, 'slack': 0, 'solves': 0}
    for k in range(n_steps + 1):
        t = t_all[k]
        z = p_car_all[k, :2] + noise[k]
        if x_kf is None:
            x_kf = np.array([z[0], z[1], CAR_VX, 0.0])
        else:
            x_kf = F_KF @ x_kf
            P_kf = F_KF @ P_kf @ F_KF.T + Q_KF
        S = H_KF @ P_kf @ H_KF.T + R_KF
        K = P_kf @ H_KF.T @ np.linalg.inv(S)
        x_kf = x_kf + K @ (z - H_KF @ x_kf)
        P_kf = (np.eye(4) - K @ H_KF) @ P_kf
        pred2 = x_kf[:2] + n_pred * DT_SIM * x_kf[2:]           # F^n x, position part
        if k % SUB == 0 and k < n_steps:
            tau = np.arange(N_HOR + 1) * DT_MPC
            if variant in ('oracle', 'oracle_card'):
                ref = desired_pos(t + tau)
            else:
                psi_hat = math.atan2(x_kf[3], x_kf[2])
                if variant == 'no_pred':
                    cpos = np.repeat(x_kf[None, :2], N_HOR + 1, 0)
                else:
                    cpos = x_kf[None, :2] + tau[:, None] * x_kf[None, 2:]
                ref = offset_point(cpos, np.full(N_HOR + 1, psi_hat))
            if variant == 'kf_cardlin':
                p_lin = None
            elif pred_prev is None:
                p_lin = np.column_stack([np.interp(TAU_C, tau, ref[:, a]) for a in range(2)])
            else:
                p_lin = np.vstack([pred_prev[SUB:, :2], np.repeat(pred_prev[-1:, :2], SUB, 0)])
            qv = 0.0 if variant in ('kf_cardcost', 'oracle_card') else \
                (Q_VEL if q_vel is None else q_vel)
            u, pred_prev, info = solve_mpc(p, v, ref, pillars, use_obs, p_lin, qv,
                                           knots_only=(variant == 'kf_knots'))
            info_count['solves'] += 1
            info_count['fail'] += int(not info[0])
            info_count['slack'] += int(info[1])
        log['p'].append(p.copy()); log['v'].append(v.copy()); log['u'].append(u.copy())
        log['kf'].append(x_kf.copy()); log['pred2'].append(pred2.copy())
        if k == n_steps:
            break
        p = p + v * DT_SIM + 0.5 * u * DT_SIM ** 2
        v = v + u * DT_SIM
        assert np.all(np.abs(u) <= A_MAX + 1e-9), 'acceleration limit violated'
        assert p[2] > 0.0, 'drone below ground'
    for key in log:
        log[key] = np.array(log[key])
    P = log['p']
    err = np.linalg.norm(P - p_des_all, axis=1)
    dist_car = np.linalg.norm(P - p_car_all, axis=1)
    f_h = framing(dist_car)
    dxy = np.linalg.norm(P[:, None, :2] - pillars[None], axis=2)      # (T, 5)
    clear = dxy.min(axis=1)
    # true integrated path between samples (constant acceleration within each 0.05 s step)
    ss = np.linspace(0.0, DT_SIM, CLEAR_SUB, endpoint=False)
    Pf = (P[:-1, None, :] + log['v'][:-1, None, :] * ss[None, :, None]
          + 0.5 * log['u'][:-1, None, :] * ss[None, :, None] ** 2).reshape(-1, 3)
    Pf = np.vstack([Pf, P[-1:]])
    clear_f = np.linalg.norm(Pf[:, None, :2] - pillars[None], axis=2).min(axis=1)
    near_h = Pf[:, 2] <= PILLAR_HEIGHT
    inside = near_h & (clear_f <= PILLAR_RADIUS)
    pillar_hits = int(np.sum(inside))
    if use_obs and variant != 'kf_knots':
        assert pillar_hits == 0, 'drone inside a pillar'
    steady = t_all >= T_SKIP
    vmax_inf = float(np.abs(log['v']).max())
    assert vmax_inf <= V_MAX + 0.05, f'speed limit violated {vmax_inf} {info_count}'
    # 2-s-ahead KF prediction error against the true car position
    idx = np.arange(len(t_all) - n_pred)
    pred_err = np.linalg.norm(log['pred2'][idx] - p_car_all[idx + n_pred, :2], axis=1)
    kf_err = np.linalg.norm(log['kf'][:, :2] - p_car_all[:, :2], axis=1)
    res = {
        't': t_all, 'err': err, 'f_h': f_h, 'clear': clear, 'dist_car': dist_car,
        'rms': float(np.sqrt(np.mean(err ** 2))),
        'rms_steady': float(np.sqrt(np.mean(err[steady] ** 2))),
        'in_ball': float(np.mean(err <= TOL_BALL)),
        'in_ball_steady': float(np.mean(err[steady] <= TOL_BALL)),
        'f_card': float(np.mean((f_h >= BAND_CARD[0]) & (f_h <= BAND_CARD[1]))),
        'f_fixed': float(np.mean((f_h >= BAND_FIXED[0]) & (f_h <= BAND_FIXED[1]))),
        'min_clear': float(clear_f.min()),
        'hit_run': bool(pillar_hits > 0),
        'max_speed': float(np.linalg.norm(log['v'], axis=1).max()),
        'max_speed_inf': vmax_inf,
        'max_err': float(err.max()),
        'pred_rms': float(np.sqrt(np.mean(pred_err ** 2))),
        'pred_max': float(pred_err.max()),
        'kf_rms': float(np.sqrt(np.mean(kf_err[20:] ** 2))),
        'n_solves': info_count['solves'], 'n_fail': info_count['fail'],
        'pillar_hits': pillar_hits,
        'n_slack': info_count['slack'],
        'pillars': pillars,
    }
    if keep:
        res.update({'P': P, 'p_des': p_des_all, 'p_car': p_car_all, 'kf': log['kf'],
                    'pred2': log['pred2'], 'pred_err': pred_err, 'v': log['v']})
    return res


def offline_floor(a_max):
    """Best tracking any controller could achieve with full knowledge of the future:
    one QP over the whole mission (per axis), card weights, same limits, no pillars,
    drone starting at the desired point at rest."""
    n = int(round(T_TOTAL / DT_MPC))
    idx = np.arange(1, n + 1)
    gp = np.zeros((n, n))
    gv = np.zeros((n, n))
    for i in range(1, n + 1):
        gp[i - 1, :i] = DT_MPC ** 2 * (i - np.arange(i) - 0.5)
        gv[i - 1, :i] = DT_MPC
    t = idx * DT_MPC
    pd = desired_pos(np.concatenate([[0.0], t]))
    err = np.zeros((n, 3))
    for a in range(3):
        e = pd[1:, a] - pd[0, a]
        H = 2 * (Q_MPC * gp.T @ gp + R_MPC * np.eye(n))
        g = -2 * Q_MPC * gp.T @ e
        G = np.vstack([gv, -gv, np.eye(n), -np.eye(n)])
        h = np.concatenate([np.full(2 * n, V_MAX), np.full(2 * n, a_max)])
        u, ok, _ = qp_ipm(H, g, G, h, max_iter=100)
        assert ok, 'offline QP did not converge'
        err[:, a] = gp @ u - e
    en = np.linalg.norm(err, axis=1)
    st = t >= T_SKIP
    return (float(np.sqrt(np.mean(en ** 2))), float(np.sqrt(np.mean(en[st] ** 2))),
            float(np.mean(en <= TOL_BALL)))


SUMMARY_KEYS = ('rms', 'rms_steady', 'in_ball', 'in_ball_steady', 'f_card', 'f_fixed',
                'min_clear', 'max_speed', 'max_speed_inf', 'max_err', 'pred_rms', 'pred_max',
                'kf_rms', 'n_solves', 'n_fail', 'n_slack', 'pillar_hits', 'hit_run')


def _job(args):
    variant, seed, keep, lateral, q_vel = args
    r = simulate(variant, seed, keep=keep, lateral=lateral, q_vel=q_vel)
    return r if keep else {k: r[k] for k in SUMMARY_KEYS}


def run_simulation():
    """All runs in a process pool; results are returned in job order (deterministic)."""
    from concurrent.futures import ProcessPoolExecutor
    jobs = []
    for seed in SEEDS:
        for var in VARIANTS:
            if var in SEED_FREE and seed != MAIN_SEED:
                continue
            jobs.append(('main', var, (var, seed, seed == MAIN_SEED, PILLAR_LATERAL, None)))
    for off in OFFSET_SWEEP:
        for var in ('kf_mpc', 'kf_knots'):
            for seed in SEEDS:
                jobs.append((('offset', off, var), var, (var, seed, False, off, None)))
    for qv in QV_SWEEP:
        for seed in SEEDS:
            jobs.append((('qv', qv), 'kf_mpc', ('kf_mpc', seed, False, PILLAR_LATERAL, qv)))
    with ProcessPoolExecutor(max_workers=min(N_WORKERS, os.cpu_count() or 1)) as ex:
        results = list(ex.map(_job, [j[2] for j in jobs], chunksize=1))
    main, mc, sweep = {}, {v: [] for v in VARIANTS}, {}
    for (tag, var, args), r in zip(jobs, results):
        if tag == 'main':
            if args[2]:
                main[var] = r
            mc[var].append({k: r[k] for k in SUMMARY_KEYS})
        else:
            sweep.setdefault(tag, []).append(r)
    return main, mc, sweep


# ── Plotting ──────────────────────────────────────────────────────────────────

def pillar_windows(p_car, t, pillars, radius=15.0):
    near = np.min(np.linalg.norm(p_car[:, None, :2] - pillars[None], axis=2), axis=1) < radius
    wins, start = [], None
    for k, nflag in enumerate(near):
        if nflag and start is None:
            start = t[k]
        if not nflag and start is not None:
            wins.append((start, t[k])); start = None
    if start is not None:
        wins.append((start, t[-1]))
    return wins


def _cylinder(ax, c, r, h, color='grey'):
    th = np.linspace(0, 2 * np.pi, 30)
    zz = np.linspace(0, h, 2)
    TH, ZZ = np.meshgrid(th, zz)
    ax.plot_surface(c[0] + r * np.cos(TH), c[1] + r * np.sin(TH), ZZ, color=color,
                    alpha=0.6, linewidth=0)


def plot_trajectory_3d(main, out_dir):
    r = main['kf_mpc']
    fig = plt.figure(figsize=(14, 7))
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(r['p_car'][:, 0], r['p_car'][:, 1], r['p_car'][:, 2], color='blue', lw=2,
            label='car (road)')
    ax.plot(r['p_des'][:, 0], r['p_des'][:, 1], r['p_des'][:, 2], 'g--', lw=1.3,
            label='desired camera position')
    ax.plot(r['P'][:, 0], r['P'][:, 1], r['P'][:, 2], color='red', lw=1.6,
            label='camera drone (proposed)')
    for c in r['pillars']:
        _cylinder(ax, c, PILLAR_RADIUS, PILLAR_HEIGHT)
    ax.scatter(*r['P'][0], color='red', marker='o', s=40)
    ax.scatter(*r['P'][-1], color='red', marker='s', s=40)
    ax.plot([], [], color='grey', lw=6, label='pillars (r = 0.6 m, h = 6 m)')
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)'); ax.set_zlabel('z (m)')
    ax.set_box_aspect((6, 1.6, 0.5))
    ax.view_init(elev=35, azim=-75)
    ax.legend(loc='upper left', fontsize=9)
    ax.set_title(f'S092 Car chase (seed {MAIN_SEED}): RMS tracking error '
                 f"{r['rms']:.2f} m, min pillar clearance {r['min_clear']:.2f} m")
    path = os.path.join(out_dir, 'trajectory_3d.png')
    plt.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_tracking_error(main, out_dir):
    fig, ax = plt.subplots(figsize=(13, 5))
    r0 = main['kf_mpc']
    for a, b in pillar_windows(r0['p_car'], r0['t'], PILLARS):
        ax.axvspan(a, b, color='grey', alpha=0.2)
    for var in ('kf_mpc', 'kf_cardcost', 'no_pred', 'oracle', 'oracle_card'):
        r = main[var]
        ax.plot(r['t'], r['err'], color=V_COLOR[var], lw=1.3,
                label=f"{V_LABEL[var]} (RMS {r['rms']:.2f} m)")
    ax.axhline(TOL_BALL, color='k', ls='--', lw=1, label=f'{TOL_BALL} m tolerance')
    ax.set_yscale('log')
    ax.set_xlabel('time (s)')
    ax.set_ylabel(r'$\|p_{drone} - p_d\|$ (m)')
    ax.set_title(f'S092 Tracking error (seed {MAIN_SEED}); grey = car within 15 m of a pillar')
    ax.legend(fontsize=8, loc='upper right')
    ax.grid(alpha=0.3, which='both')
    path = os.path.join(out_dir, 'tracking_error.png')
    plt.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_framing(main, out_dir):
    fig, ax = plt.subplots(figsize=(13, 4.8))
    ax.axhspan(*BAND_CARD, color='tab:red', alpha=0.10, label='card band 20–40 %')
    ax.axhspan(*BAND_FIXED, color='green', alpha=0.15, label='corrected band 10–20 %')
    nominal = framing(np.linalg.norm(D_OFFSET))
    ax.axhline(nominal, color='green', ls=':', lw=1.2,
               label=f'at the nominal offset ({100 * nominal:.1f} %)')
    for var in ('kf_mpc', 'kf_cardcost', 'no_pred', 'oracle'):
        r = main[var]
        ax.plot(r['t'], r['f_h'], color=V_COLOR[var], lw=1.2,
                label=f"{V_LABEL[var]}: score {100 * r['f_fixed']:.1f} % "
                      f"(card band {100 * r['f_card']:.1f} %)")
    ax.set_xlabel('time (s)')
    ax.set_ylabel('car height / frame height')
    ax.set_ylim(0, 0.45)
    ax.set_title(f'S092 Camera framing $f_{{height}}(t)$ (seed {MAIN_SEED})')
    ax.legend(fontsize=8, loc='upper right', ncol=2)
    ax.grid(alpha=0.3)
    path = os.path.join(out_dir, 'framing.png')
    plt.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_clearance(main, out_dir):
    fig, ax = plt.subplots(figsize=(13, 4.8))
    for var in ('kf_mpc', 'kf_knots', 'kf_noobs', 'kf_cardlin', 'kf_card'):
        r = main[var]
        ax.plot(r['t'], r['clear'], color=V_COLOR[var], lw=1.3,
                label=f"{V_LABEL[var]} (min {r['min_clear']:.2f} m)")
    ax.axhline(R_SAFE, color='red', ls='--', lw=1.3, label=f'r_safe = {R_SAFE} m')
    ax.axhline(PILLAR_RADIUS, color='grey', ls='-', lw=2, label='pillar surface (0.6 m)')
    ax.set_yscale('log')
    ax.set_xlabel('time (s)')
    ax.set_ylabel('horizontal distance to nearest pillar centre (m)')
    ax.set_title(f'S092 Obstacle clearance (seed {MAIN_SEED})')
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3, which='both')
    path = os.path.join(out_dir, 'clearance.png')
    plt.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_kf_prediction(main, out_dir):
    r = main['kf_mpc']
    n_pred = int(math.floor(T_PRED / DT_SIM + 1e-9))
    fig, axes = plt.subplots(2, 1, figsize=(13, 7), sharex=False)
    ax = axes[0]
    ax.plot(r['p_car'][:, 0], r['p_car'][:, 1], color='blue', lw=2, label='car (true)')
    sel = np.arange(0, len(r['t']) - n_pred, 20)
    ax.plot(r['pred2'][sel, 0], r['pred2'][sel, 1], 'c D', ms=4,
            label='KF constant-velocity prediction 2 s ahead (every 1 s)')
    ax.plot(r['p_car'][sel + n_pred, 0], r['p_car'][sel + n_pred, 1], 'o', color='blue',
            ms=3, mfc='none', label='true car position 2 s later')
    for k in sel:
        ax.plot([r['pred2'][k, 0], r['p_car'][k + n_pred, 0]],
                [r['pred2'][k, 1], r['p_car'][k + n_pred, 1]], color='c', lw=0.6)
    ax.set_aspect('equal')
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')
    ax.legend(fontsize=8, loc='upper right')
    ax.set_title('2-s-ahead KF prediction vs truth (lines = prediction error)')
    ax = axes[1]
    ax.plot(r['t'][:len(r['pred_err'])], r['pred_err'], color='c')
    ax.set_xlabel('prediction time t (s)')
    ax.set_ylabel('|prediction − truth| (m)')
    ax.set_title(f"2-s prediction error: RMS {r['pred_rms']:.2f} m, max {r['pred_max']:.2f} m "
                 f"(KF filtered position RMS {r['kf_rms']:.2f} m vs measurement σ {SIGMA_MEAS} m)")
    ax.grid(alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'kf_prediction.png')
    plt.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_monte_carlo(mc, out_dir):
    panels = [('rms_steady', 'RMS tracking error, t ≥ 3 s (m)'),
              ('in_ball', 'Time within 1 m of target (%)'),
              ('f_fixed', 'Framing score, 10–20 % band (%)'),
              ('min_clear', 'Min pillar clearance (m)')]
    fig, axes = plt.subplots(2, 2, figsize=(18, 10))
    axes = axes.ravel()
    for ax, (key, title) in zip(axes, panels):
        for x, var in enumerate(VARIANTS):
            vals = np.array([m[key] for m in mc[var]], dtype=float)
            if key in ('in_ball', 'f_fixed'):
                vals = 100 * vals
            ax.bar(x, vals.mean(), yerr=vals.std(), color=V_COLOR[var], alpha=0.75,
                   edgecolor='k', capsize=4)
            ax.scatter(x + np.linspace(-0.2, 0.2, len(vals)), vals, color='k', s=6, zorder=3)
        if key == 'min_clear':
            ax.axhline(R_SAFE, color='red', ls='--', lw=1.2, label='r_safe')
            ax.axhline(PILLAR_RADIUS, color='grey', lw=2, label='pillar surface')
            ax.legend(fontsize=8)
        ax.set_xticks(range(len(VARIANTS)))
        ax.set_xticklabels([V_SHORT[v] for v in VARIANTS], fontsize=8)
        ax.set_title(title, fontsize=10)
        ax.grid(alpha=0.3, axis='y')
    fig.suptitle(f'S092 Monte Carlo over {N_MC} measurement-noise seeds '
                 '(bar = mean, error bar = std, dots = seeds; oracle variants are noise-free, '
                 'one run)')
    plt.tight_layout()
    path = os.path.join(out_dir, 'monte_carlo.png')
    plt.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def save_animation(main, out_dir, step=4):
    r = main['kf_mpc']
    n = len(r['t'])
    n_frames = math.ceil((n - 1) / step) + 1
    n_pred = int(math.floor(T_PRED / DT_SIM + 1e-9))
    fig, ax = plt.subplots(figsize=(8, 6))
    ax.plot(r['p_car'][:, 0], r['p_car'][:, 1], color='blue', lw=0.8, alpha=0.4)
    ax.plot(r['p_des'][:, 0], r['p_des'][:, 1], 'g--', lw=0.6, alpha=0.5)
    for c in r['pillars']:
        ax.add_patch(Circle(c, PILLAR_RADIUS, color='grey'))
        ax.add_patch(Circle(c, R_SAFE, fill=False, ec='grey', ls=':'))
    trail, = ax.plot([], [], color='red', lw=1.2)
    car, = ax.plot([], [], 'o', color='blue', ms=9, label='car')
    drone, = ax.plot([], [], 'o', color='red', ms=7, label='drone')
    des, = ax.plot([], [], 'x', color='green', ms=10, mew=2, label='desired position')
    pred, = ax.plot([], [], 'D', color='c', ms=6, label='KF 2-s prediction')
    ax.set_aspect('equal')
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')
    ax.legend(loc='lower right', fontsize=8)
    title = ax.set_title('')
    f_ok = (r['f_h'] >= BAND_FIXED[0]) & (r['f_h'] <= BAND_FIXED[1])

    def update(f):
        k = min(f * step, n - 1)
        cx = r['p_car'][k, 0]
        ax.set_xlim(cx - 30, cx + 20)
        ax.set_ylim(-25, 25)
        trail.set_data(r['P'][max(0, k - 60):k + 1, 0], r['P'][max(0, k - 60):k + 1, 1])
        car.set_data([r['p_car'][k, 0]], [r['p_car'][k, 1]])
        drone.set_data([r['P'][k, 0]], [r['P'][k, 1]])
        des.set_data([r['p_des'][k, 0]], [r['p_des'][k, 1]])
        pred.set_data([r['pred2'][k, 0]], [r['pred2'][k, 1]])
        title.set_text(f"S092 t = {r['t'][k]:4.1f} s  error {r['err'][k]:.2f} m  "
                       f"framing score so far {100 * f_ok[:k + 1].mean():.0f} %")
        return trail, car, drone, des, pred, title

    anim = animation.FuncAnimation(fig, update, frames=n_frames, interval=100, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    anim.save(path, writer='pillow', fps=10, dpi=72)
    plt.close(fig)
    print(f'Saved: {path} ({n_frames} frames)')


# ── Main ──────────────────────────────────────────────────────────────────────

def _ms(vals, fmt='{:.3f}', scale=1.0):
    v = scale * np.asarray(vals, dtype=float)
    return f'{fmt.format(v.mean())} ± {fmt.format(v.std())}'


if __name__ == '__main__':
    main, mc, sweep = run_simulation()
    tt = np.linspace(0, T_TOTAL, 75001)
    pc, vc, psi = car_state(tt)
    pd = desired_pos(tt)
    vd = np.gradient(pd, tt, axis=0)
    ad = np.gradient(vd, tt, axis=0)
    print('\n=== S092 Movie Car Chase — Results ===')
    print(f'  Car along-road speed: {np.linalg.norm(vc, axis=1).min():.2f}–'
          f'{np.linalg.norm(vc, axis=1).max():.2f} m/s; road arc length '
          f'{np.sum(np.linalg.norm(np.diff(pc[:, :2], axis=0), axis=1)):.1f} m '
          f'over x = 0–{ROAD_X_LEN:.0f} m')
    print(f'  Heading range: ±{np.degrees(np.abs(psi).max()):.1f} deg')
    print(f'  Desired camera point: max speed {np.linalg.norm(vd, axis=1).max():.2f} m/s, '
          f'max |a_x| {np.abs(ad[:, 0]).max():.2f}, max |a_y| {np.abs(ad[:, 1]).max():.2f} '
          f'm/s² (limit {A_MAX} per axis)')
    c0, s0 = math.cos(psi[0]), math.sin(psi[0])
    card_off = np.array([-s0 * -8.0, c0 * -8.0])
    print(f'  Card offset (0, -8) at t = 0 rotates to ({card_off[0]:.2f}, {card_off[1]:.2f}) m; '
          f'angle to the car heading {np.degrees(math.atan2(card_off[1], card_off[0]) - psi[0]):.1f} deg')
    dn = np.linalg.norm(D_OFFSET)
    print(f'  Nominal camera distance {dn:.2f} m -> card framing f = {100 * framing(dn):.1f} %; '
          f'f = 20 % needs d = {H_CAR / (2 * math.tan(math.radians(0.2 * FOV_DEG / 2))):.2f} m, '
          f'f = 40 % needs d = {H_CAR / (2 * math.tan(math.radians(0.4 * FOV_DEG / 2))):.2f} m, '
          f'f = 10 % needs d = {H_CAR / (2 * math.tan(math.radians(0.1 * FOV_DEG / 2))):.2f} m')
    d_card = np.min(np.linalg.norm(pd[:, None, :2] - PILLARS_CARD[None], axis=2), axis=0)
    d_new = np.min(np.linalg.norm(pd[:, None, :2] - PILLARS[None], axis=2), axis=0)
    print('  Card pillars: min distance to desired camera path = ' +
          ', '.join(f'{d:.2f}' for d in d_card) + ' m')
    print('  Relocated pillars: ' + ', '.join(f'({c[0]:.1f}, {c[1]:.1f})' for c in PILLARS) +
          '; min distance to desired path = ' + ', '.join(f'{d:.2f}' for d in d_new) + ' m')
    print(f'\n  Main seed {MAIN_SEED}:')
    for var in VARIANTS:
        r = main[var]
        print(f'  [{V_LABEL[var]}]')
        print(f"    RMS tracking error  : {r['rms']:.3f} m (t >= {T_SKIP:.0f} s: "
              f"{r['rms_steady']:.3f} m), max {r['max_err']:.2f} m")
        print(f"    Time within {TOL_BALL} m  : {100 * r['in_ball']:.1f} % "
              f"(t >= {T_SKIP:.0f} s: {100 * r['in_ball_steady']:.1f} %)")
        print(f"    Framing score       : {100 * r['f_fixed']:.1f} % (10–20 % band), "
              f"{100 * r['f_card']:.1f} % (card 20–40 % band)")
        print(f"    Min pillar clearance: {r['min_clear']:.3f} m "
              f"(timesteps inside a pillar: {r['pillar_hits']})")
        print(f"    Max speed           : {r['max_speed']:.2f} m/s (max per-axis "
              f"{r['max_speed_inf']:.2f} m/s)")
        print(f"    QP solves           : {r['n_solves']}, not converged {r['n_fail']}, "
              f"with obstacle slack > 0: {r['n_slack']}")
    r = main['kf_mpc']
    print(f"  KF: filtered position RMS {r['kf_rms']:.3f} m (after the first 1 s); "
          f"2-s-ahead prediction RMS {r['pred_rms']:.2f} m, max {r['pred_max']:.2f} m")
    print(f'\n  Monte Carlo over {N_MC} noise seeds (mean ± std; oracle variants are '
          'noise-free and run once):')
    for var in VARIANTS:
        m = mc[var]
        print(f'  [{V_LABEL[var]}]')
        print(f'    RMS error        : {_ms([x["rms"] for x in m])} m, '
              f't >= {T_SKIP:.0f} s: {_ms([x["rms_steady"] for x in m])} m')
        print(f'    Within {TOL_BALL} m      : {_ms([x["in_ball"] for x in m], "{:.1f}", 100)} %, '
              f't >= {T_SKIP:.0f} s: {_ms([x["in_ball_steady"] for x in m], "{:.1f}", 100)} %')
        print(f'    Framing score    : {_ms([x["f_fixed"] for x in m], "{:.1f}", 100)} % '
              f'(card band {_ms([x["f_card"] for x in m], "{:.1f}", 100)} %)')
        print(f'    Min clearance    : {_ms([x["min_clear"] for x in m])} m '
              f'(worst {min(x["min_clear"] for x in m):.3f} m, runs below r_safe: '
              f'{sum(x["min_clear"] < R_SAFE for x in m)}/{len(m)}, runs hitting a pillar: '
              f'{sum(x["pillar_hits"] > 0 for x in m)}/{len(m)})')
        print(f'    Max speed        : {max(x["max_speed"] for x in m):.2f} m/s (norm; '
              f'the card bound applies per axis: max per-axis {max(x["max_speed_inf"] for x in m):.2f} m/s)')
        print(f'    QP not converged : {sum(x["n_fail"] for x in m)} of '
              f'{sum(x["n_solves"] for x in m)}; solves with obstacle slack > 0: '
              f'{_ms([x["n_slack"] for x in m], "{:.1f}")} per run')
    print(f'    KF 2-s prediction RMS: {_ms([x["pred_rms"] for x in mc["kf_mpc"]], "{:.2f}")} m')
    a = [x['rms_steady'] for x in mc['kf_mpc']]
    b = [x['rms_steady'] for x in mc['no_pred']]
    c = [x['rms_steady'] for x in mc['kf_cardcost']]
    print('\n  Paired per-seed comparison (steady RMS error):')
    print(f'    proposed < no prediction in {sum(x < y for x, y in zip(a, b))}/{N_MC} seeds')
    print(f'    proposed < card cost in {sum(x < y for x, y in zip(a, c))}/{N_MC} seeds')
    print(f"    oracle (steady RMS {mc['oracle'][0]['rms_steady']:.3f} m) < proposed in "
          f"{sum(mc['oracle'][0]['rms_steady'] < x for x in a)}/{N_MC} seeds")
    print(f'\n  Pillar-offset sweep ({N_MC} seeds each; offset = pillar distance from the desired '
          'camera path; clearance on the integrated path, 10 points per 0.05 s):')
    for off in OFFSET_SWEEP:
        for var in ('kf_mpc', 'kf_knots'):
            rows = sweep[('offset', off, var)]
            print(f'    {off:3.1f} m  {"0.05 s constraints" if var == "kf_mpc" else "knots only        "}: '
                  f'steady RMS {_ms([x["rms_steady"] for x in rows])} m, min clearance '
                  f'{_ms([x["min_clear"] for x in rows])} m (worst {min(x["min_clear"] for x in rows):.3f} m), '
                  f'runs < r_safe {sum(x["min_clear"] < R_SAFE for x in rows)}/{len(rows)}, '
                  f'runs inside a pillar {sum(x["hit_run"] for x in rows)}/{len(rows)}, '
                  f'slack-active solves {_ms([x["n_slack"] for x in rows], "{:.1f}")}')
    print(f'\n  Velocity-weight sensitivity (proposed, pillars {PILLAR_LATERAL} m from the path, '
          f'{N_MC} seeds each):')
    for qv in QV_SWEEP:
        rows = sweep[('qv', qv)]
        print(f'    Q_v = {qv:5.1f}: steady RMS {_ms([x["rms_steady"] for x in rows])} m, '
              f'within 1 m {_ms([x["in_ball"] for x in rows], "{:.1f}", 100)} %, '
              f'min clearance worst {min(x["min_clear"] for x in rows):.3f} m, '
              f'runs inside a pillar {sum(x["hit_run"] for x in rows)}/{len(rows)}')
    print('\n  Offline optimum (one QP over the whole 37.5 s with the true reference: minimises the '
          'card-weighted position/control cost at the 0.2 s knots, same speed and acceleration '
          'limits, no pillars; a reference point, not a strict bound on RMS):')
    for amax in (A_MAX, 10.0):
        fl = offline_floor(amax)
        print(f'    a_max {amax:4.1f} m/s²: RMS {fl[0]:.3f} m, t >= {T_SKIP:.0f} s: {fl[1]:.3f} m, '
              f'within {TOL_BALL} m {100 * fl[2]:.1f} % of the time')

    os.makedirs(OUTPUT_DIR, exist_ok=True)
    plot_trajectory_3d(main, OUTPUT_DIR)
    plot_tracking_error(main, OUTPUT_DIR)
    plot_framing(main, OUTPUT_DIR)
    plot_clearance(main, OUTPUT_DIR)
    plot_kf_prediction(main, OUTPUT_DIR)
    plot_monte_carlo(mc, OUTPUT_DIR)
    save_animation(main, OUTPUT_DIR)
