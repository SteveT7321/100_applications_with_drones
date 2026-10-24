"""
S084 Extreme Weather Wind Resistance Test
=========================================
A 1 kg validation quadrotor flies 100 m along x at 1 m/s and 10 m altitude while a 12 m/s mean
crosswind (+y) with Dryden turbulence (sigma 4 m/s lateral, 2 m/s vertical) pushes it sideways
through quadratic aerodynamic drag. Three controllers are compared on identical wind
realisations (common random numbers):

  1. PID only   - cascaded position P / velocity PI / attitude / body-rate PI controller of
                  src/base/rigid_body_drone.py, no wind knowledge.
  2. LQR        - LQR on the card's 10-state hover linearisation, outputs thrust + roll/pitch
                  torques directly, no wind estimate.
  3. LQR + RLS  - the same LQR around an equilibrium shifted by a recursive estimate of the wind
                  acceleration (feed-forward tilt and thrust).

All three are hard-clamped to the same commanded tilt (card: 30 deg); a sensitivity run repeats
everything at other common limits (30.5, 31, 35, 39 deg) and with a cone-shaped LQR clamp.

The plant is the 6-DOF rigid-body model (quaternion attitude, motor lag, mixer), fpv_5inch motors
with the card's mass 1.0 kg and inertia 0.01 kg m^2. The headline run is seed 42; a 20-seed
Monte Carlo gives success counts and mean +- std.

Usage:
    conda activate drones
    python src/05_special_entertainment/s084_wind_endurance.py
"""

import os
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401
from mpl_toolkits.mplot3d.art3d import Line3DCollection
from scipy.linalg import expm, solve_continuous_are

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'base'))
from rigid_body_drone import (RigidBodyDrone, make_params, default_gains,  # noqa: E402
                              quat_to_euler, quat_to_rot, tilt_angle, G)

# ── Parameters (card) ─────────────────────────────────────────────────────────
MASS        = 1.0            # kg
I_XX = I_YY = 0.01           # kg m^2
RHO_AIR     = 1.225          # kg/m^3
C_D         = 0.6
A_REF       = 0.04           # m^2
MEAN_WIND_Y = 12.0           # m/s
SIGMA_Y     = 4.0            # m/s  lateral gust std
L_Y         = 200.0          # m    lateral scale length
V_AIR       = 12.0           # m/s  filter airspeed
Z_REF       = 10.0           # m
V_REF       = 1.0            # m/s
X_GOAL      = 100.0          # m
THETA_LIM   = np.radians(30.0)
THETA_FAIL  = np.radians(40.0)
E_Y_FAIL    = 5.0            # m
GAMMA       = 0.05           # RLS forgetting gain (per 0.02 s update)
DT_EST      = 0.02           # s    RLS / card time step
T_MAX       = 120.0          # s
Q_DIAG      = np.array([1., 20., 20., 0.5, 4., 4., 8., 8., 1., 1.])
R_DIAG      = np.array([0.1, 1.0, 1.0])

# ── Parameters (added / changed) ──────────────────────────────────────────────
SIGMA_Z     = 2.0            # m/s  vertical gust std (card code: 0.5 x lateral)
L_Z         = 100.0          # m    vertical scale length (card code: 0.5 x lateral)
I_ZZ        = 0.018          # kg m^2 (not in card; estimate)
DT          = 0.005          # s    rigid-body control / physics step (200 Hz)
EST_EVERY   = int(round(DT_EST / DT))
LOG_EVERY   = EST_EVERY      # log at 50 Hz
ACC_NOISE   = 0.1            # m/s^2 accelerometer noise std (RLS input)
YAW_KP, YAW_KD = 16.0, 8.0   # yaw hold for the LQR controllers (1/s^2, 1/s)
MC_SEEDS    = list(range(42, 62))
# Tilt-limit groups (limit deg, LQR clamp shape); the PID always clips the total tilt (a cone).
# 'box' clips the LQR's commanded roll and pitch separately (card), 'cone' clips their norm.
# Group 0 (30 deg, box) is the card's setting used for the headline and Monte Carlo.
TILT_GROUPS = [(30.0, 'box'), (30.0, 'cone'), (30.5, 'box'), (31.0, 'box'), (35.0, 'box'),
               (39.0, 'box')]
BIND_MARGIN = np.radians(0.5)   # 'clamp binding' = total tilt within 0.5 deg of the limit
HEAD_SEED   = 42
W_EFF_MEAN  = 0.5 * RHO_AIR * C_D * A_REF * MEAN_WIND_Y ** 2 / MASS   # m/s^2

CTRLS = ['pid', 'lqr', 'lqr_rls']
LABEL = {'pid': 'PID only', 'lqr': 'LQR', 'lqr_rls': 'LQR + RLS'}
COLOR = {'pid': 'tab:blue', 'lqr': 'tab:orange', 'lqr_rls': 'tab:red'}

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), '..', '..',
                          'outputs', '05_special_entertainment', 's084_wind_endurance')


# ── Dryden turbulence ─────────────────────────────────────────────────────────
def dryden_matrices(sigma, L, V, dt, card_gain=False):
    """ZOH discretisation of (1 + sqrt3 tau s) / (1 + tau s)^2, tau = L / V, driven by
    unit-intensity white noise n_k = xi_k / sqrt(dt). Output gain sigma * sqrt(tau) gives a
    stationary std of sigma; the card's gain sigma * sqrt(2 V / (pi L)) is kept for checking."""
    tau = L / V
    Ac = np.array([[-2.0 / tau, -1.0 / tau ** 2], [1.0, 0.0]])
    Bc = np.array([1.0, 0.0])
    k = sigma * np.sqrt(2.0 * V / (np.pi * L)) if card_gain else sigma * np.sqrt(tau)
    Cc = k * np.array([np.sqrt(3.0) / tau, 1.0 / tau ** 2])
    M = np.zeros((3, 3))
    M[:2, :2] = Ac * dt
    M[:2, 2] = Bc * dt
    eM = expm(M)
    return eM[:2, :2], eM[:2, 2], Cc


def dryden_series(sigma, L, V, dt, n, rng, card_gain=False):
    Ad, Bd, Cc = dryden_matrices(sigma, L, V, dt, card_gain)
    # start from the stationary distribution (discrete Lyapunov by iteration)
    P = np.zeros((2, 2))
    Q = np.outer(Bd, Bd) / dt
    for _ in range(20000):
        P = Ad @ P @ Ad.T + Q
    x = np.linalg.cholesky(P + 1e-15 * np.eye(2)) @ rng.standard_normal(2)
    xi = rng.standard_normal(n) / np.sqrt(dt)
    out = np.empty(n)
    for k in range(n):
        out[k] = Cc @ x
        x = Ad @ x + Bd * xi[k]
    return out


def wind_series(seed, n):
    rng = np.random.default_rng(seed)
    wy = dryden_series(SIGMA_Y, L_Y, V_AIR, DT, n, rng)
    wz = dryden_series(SIGMA_Z, L_Z, V_AIR, DT, n, rng)
    return np.column_stack([np.zeros(n), MEAN_WIND_Y + wy, wz])


def aero_force(v_drone, v_wind):
    dv = v_wind - v_drone
    return 0.5 * RHO_AIR * C_D * A_REF * dv * np.abs(dv)


# ── LQR ───────────────────────────────────────────────────────────────────────
def lqr_gain():
    A = np.zeros((10, 10))
    B = np.zeros((10, 3))
    A[0, 3] = A[1, 4] = A[2, 5] = 1.0
    A[3, 7] = G
    A[4, 6] = -G
    A[6, 8] = A[7, 9] = 1.0
    B[5, 0] = 1.0 / MASS
    B[8, 1] = 1.0 / I_XX
    B[9, 2] = 1.0 / I_YY
    P = solve_continuous_are(A, B, np.diag(Q_DIAG), np.diag(R_DIAG))
    K = np.linalg.solve(np.diag(R_DIAG), B.T @ P)
    Bw = np.zeros((10, 3))
    Bw[3:6] = np.eye(3) / MASS
    # steady-state error under a constant force: (A - B K) x_ss + Bw F = 0
    x_ss = -np.linalg.solve(A - B @ K, Bw @ np.array([0.0, MASS * W_EFF_MEAN, 0.0]))
    eig = np.linalg.eigvals(A - B @ K)
    return K, x_ss, eig


def clamp_attitude(ang, lim, cone):
    """(B, 2) roll/pitch commands -> clipped: per axis ('box') or on their norm ('cone')."""
    box = np.clip(ang, -lim[:, None], lim[:, None])
    n = np.linalg.norm(ang, axis=1)
    scale = np.where(n > lim, lim / np.maximum(n, 1e-12), 1.0)
    return np.where(cone[:, None], ang * scale[:, None], box)


def euler_rates(eul, w):
    ph, th = eul[:, 0], eul[:, 1]
    phd = w[:, 0] + np.sin(ph) * np.tan(th) * w[:, 1] + np.cos(ph) * np.tan(th) * w[:, 2]
    thd = np.cos(ph) * w[:, 1] - np.sin(ph) * w[:, 2]
    return phd, thd


# ── Simulation ────────────────────────────────────────────────────────────────
def run_simulation(seeds=MC_SEEDS, groups=TILT_GROUPS):
    """All controllers x all tilt-limit groups x all seeds in one batch.
    Drone index b = (l * C + c) * S + s; group 0 (the card's 30 deg) is logged in full."""
    K, x_ss, eig = lqr_gain()
    S, C, NL = len(seeds), len(CTRLS), len(groups)
    B = NL * C * S
    H = C * S                                    # fully logged headline group (limit 0)
    lim_of = np.repeat(np.radians([g[0] for g in groups]), C * S)
    cone_of = np.repeat(np.array([g[1] == 'cone' for g in groups]), C * S)
    ctrl_of = np.tile(np.repeat(np.arange(C), S), NL)
    seed_of = np.tile(np.arange(S), NL * C)
    n_max = int(round(T_MAX / DT))
    winds = np.stack([wind_series(sd, n_max + 1) for sd in seeds])      # (S, n, 3)
    params = make_params('fpv_5inch', mass=MASS, inertia=np.array([I_XX, I_YY, I_ZZ]),
                         drag=np.zeros(3))
    gains = default_gains('fpv_5inch', max_tilt=lim_of)      # PID: hard limit on commanded tilt
    start = np.tile([0.0, 0.0, Z_REF], (B, 1))
    drone = RigidBodyDrone(start, preset='fpv_5inch', params=params, gains=gains, dt=DT,
                           drag_ff=False, record=False)
    rng_acc = np.random.default_rng(1000)
    is_pid = ctrl_of == 0
    is_rls = ctrl_of == 2
    w_hat = np.zeros((B, 3))
    v_prev = drone.vel.copy()
    att_ff = np.zeros((B, 2))
    thrust_ff = np.full(B, MASS * G)
    alive = np.ones(B, bool)
    done = np.zeros(B, bool)
    t_end = np.full(B, np.nan)
    fail_reason = [''] * B
    energy_end = np.full(B, np.nan)
    peak_y = np.zeros(B)                 # running maxima at the full control rate
    peak_att = np.zeros(B)
    acc = {k: np.zeros(B) for k in ('n', 'y2', 'att', 'dz2')}   # 50 Hz running sums
    bind_time = np.zeros(B)              # time with the total tilt within BIND_MARGIN of the limit
    bind_run = np.zeros(B)
    bind_max = np.zeros(B)               # longest continuous binding interval
    tilt_max = np.zeros(B)
    logs = {k: [] for k in ('t', 'pos', 'eul', 'w_hat', 'w_true', 'thrust', 'wind', 'tilt')}
    # LQR roll / pitch rows: tau = -K_a (angle - angle_cmd) - K_r rate, with the commanded
    # angle angle_cmd = angle_ref - (other terms) / K_a hard-clipped to the tilt limit
    rows = [(1, 6, 8), (2, 7, 9)]
    t = 0.0
    for k in range(n_max):
        wind = winds[seed_of, k]
        f_aero = aero_force(drone.vel, wind)
        drone.ext_force = f_aero
        px_ref = np.full(B, min(V_REF * t, X_GOAL))
        p_ref = np.column_stack([px_ref, np.zeros(B), np.full(B, Z_REF)])
        v_ref = np.column_stack([np.where(px_ref < X_GOAL, V_REF, 0.0), np.zeros(B), np.zeros(B)])
        eul = quat_to_euler(drone.quat)
        # ── RLS wind-acceleration estimate (50 Hz) ──
        if k % EST_EVERY == 0 and k > 0:
            a_meas = (drone.vel - v_prev) / DT_EST + ACC_NOISE * rng_acc.standard_normal((B, 3))
            Rm = quat_to_rot(drone.quat)
            a_model = Rm[:, :, 2] * (drone.thrust_total() / MASS)[:, None] - [0.0, 0.0, G]
            w_hat = w_hat + GAMMA * (a_meas - a_model - w_hat)
            v_prev = drone.vel.copy()
            f = MASS * (np.array([0.0, 0.0, G]) - w_hat)
            fn = np.linalg.norm(f, axis=1)
            att_ff = np.column_stack([np.arcsin(np.clip(-f[:, 1] / fn, -1, 1)),
                                      np.arctan2(f[:, 0], f[:, 2])])
            att_ff = clamp_attitude(att_ff, lim_of, cone_of)
            thrust_ff = fn
        # ── PID (cascade of the base model) ──
        v_sp = v_ref + gains['kp_pos'] * (p_ref - drone.pos)
        a_cmd = drone._vel_ctrl(v_sp, drone.vel, np.zeros((B, 3)))
        T_pid, R_des = drone._acc_to_att(a_cmd, drone.vel, np.zeros(B))
        tau_pid = drone._rate_ctrl(drone._att_ctrl(R_des))
        # ── LQR (thrust + roll/pitch torque), optionally around the RLS equilibrium ──
        phd, thd = euler_rates(eul, drone.omega)
        x = np.column_stack([drone.pos, drone.vel, eul[:, :2], phd, thd])
        xr = np.column_stack([p_ref, v_ref, np.where(is_rls[:, None], att_ff, 0.0),
                              np.zeros((B, 2))])
        e = x - xr
        u = -e @ K.T
        T_lqr = np.where(is_rls, thrust_ff, MASS * G) + u[:, 0]
        T_lqr = T_lqr / np.maximum(np.cos(eul[:, 0]) * np.cos(eul[:, 1]), 0.5)
        ang_cmd = np.zeros((B, 2))
        for j, (r, ia, ir) in enumerate(rows):
            other = e @ K[r] - K[r, ia] * e[:, ia] - K[r, ir] * e[:, ir]
            ang_cmd[:, j] = xr[:, ia] - other / K[r, ia]
        ang_cmd = clamp_attitude(ang_cmd, lim_of, cone_of)
        tau_rp = np.zeros((B, 2))
        for j, (r, ia, ir) in enumerate(rows):
            tau_rp[:, j] = -K[r, ia] * (x[:, ia] - ang_cmd[:, j]) - K[r, ir] * e[:, ir]
        tau_z = I_ZZ * (-YAW_KP * eul[:, 2] - YAW_KD * drone.omega[:, 2])
        tau_lqr = np.column_stack([tau_rp, tau_z])
        T = np.where(is_pid, T_pid, np.clip(T_lqr, 0.0, params.thrust_max))
        tau = np.where(is_pid[:, None], tau_pid, tau_lqr)
        drone.step_wrench(T, tau)
        t = (k + 1) * DT
        # ── invariants and mission logic ──
        assert np.all(np.isfinite(drone.pos)), 'NaN state'
        assert np.all(drone.rotor <= drone.w_max + 1e-9)
        eul = quat_to_euler(drone.quat)
        att = np.max(np.abs(eul[:, :2]), axis=1)
        lat = np.abs(drone.pos[:, 1])
        flying = alive & ~done
        tilt = tilt_angle(drone.quat)
        binding = flying & (tilt >= lim_of - BIND_MARGIN)
        bind_time += binding * DT
        bind_run = np.where(binding, bind_run + DT, 0.0)
        bind_max = np.maximum(bind_max, bind_run)
        tilt_max = np.where(flying, np.maximum(tilt_max, tilt), tilt_max)
        peak_y = np.where(flying, np.maximum(peak_y, lat), peak_y)
        peak_att = np.where(flying, np.maximum(peak_att, att), peak_att)
        if (k + 1) % LOG_EVERY == 0:
            acc['n'] += flying
            acc['y2'] += flying * lat ** 2
            acc['att'] += flying * att
            acc['dz2'] += flying * (drone.pos[:, 2] - Z_REF) ** 2
        newly_fail = flying & ((lat > E_Y_FAIL) | (att > THETA_FAIL))
        for b in np.where(newly_fail)[0]:
            fail_reason[b] = (f'|y| = {lat[b]:.2f} m > {E_Y_FAIL:.0f} m' if lat[b] > E_Y_FAIL
                              else f'attitude {np.degrees(att[b]):.1f} deg > 40 deg')
        alive &= ~newly_fail
        t_end = np.where(newly_fail, t, t_end)
        energy_end = np.where(newly_fail, drone.energy, energy_end)
        finished = alive & ~done & (drone.pos[:, 0] >= X_GOAL)
        done |= finished
        t_end = np.where(finished, t, t_end)
        energy_end = np.where(finished, drone.energy, energy_end)
        assert np.all(drone.pos[alive & ~done, 2] > 0.0), 'below ground'
        if (k + 1) % LOG_EVERY == 0:
            logs['t'].append(t)
            logs['pos'].append(drone.pos[:H].copy())
            logs['eul'].append(eul[:H].copy())
            logs['w_hat'].append(w_hat[:H].copy())
            logs['w_true'].append(f_aero[:H] / MASS)
            logs['thrust'].append(drone.thrust_total()[:H])
            logs['wind'].append(wind[:H].copy())
            logs['tilt'].append(tilt_angle(drone.quat[:H]))
        if not np.any(alive & ~done):
            break
    logs = {k: np.array(v) for k, v in logs.items()}
    # ── metrics ──
    res = {}
    tl = logs['t']
    for b in range(B):
        te = t_end[b] if np.isfinite(t_end[b]) else tl[-1]
        r = dict(ctrl=CTRLS[ctrl_of[b]], seed=seeds[seed_of[b]], lim=float(np.degrees(lim_of[b])),
                 bind_time=float(bind_time[b]), bind_max=float(bind_max[b]),
                 tilt_max=float(np.degrees(tilt_max[b])),
                 success=bool(done[b]), fail=fail_reason[b], t_end=te,
                 rms_y=float(np.sqrt(acc['y2'][b] / acc['n'][b])),
                 peak_y=float(peak_y[b]), peak_att=float(np.degrees(peak_att[b])),
                 mean_att=float(np.degrees(acc['att'][b] / acc['n'][b])),
                 z_rms=float(np.sqrt(acc['dz2'][b] / acc['n'][b])),
                 energy=float(energy_end[b]) if np.isfinite(energy_end[b]) else np.nan,
                 t_conv=np.nan, est_rms=np.nan)
        if b < H:
            sel = tl <= te + 1e-9
            wy_hat = logs['w_hat'][sel, b, 1]
            conv = np.where(np.abs(wy_hat - W_EFF_MEAN) < 0.5)[0]
            after = sel & (tl > 2.0)
            r['t_conv'] = float(tl[sel][conv[0]]) if len(conv) else np.nan
            r['est_rms'] = float(np.sqrt(np.mean((logs['w_hat'][after, b, 1]
                                                  - logs['w_true'][after, b, 1]) ** 2)))
        res[b] = r
    meta = dict(K=K, x_ss=x_ss, eig=eig, S=S, C=C, seeds=seeds, groups=list(groups),
                ctrl_of=ctrl_of, seed_of=seed_of, params=params)
    return logs, res, meta, winds


# ── Plots ─────────────────────────────────────────────────────────────────────
def _b(meta, c, seed=HEAD_SEED):
    return CTRLS.index(c) * meta['S'] + meta['seeds'].index(seed)


def _sel(logs, res, b):
    return logs['t'] <= res[b]['t_end'] + 1e-9


def plot_trajectory_3d(logs, res, meta, out_dir):
    fig = plt.figure(figsize=(11, 7))
    ax = fig.add_subplot(111, projection='3d')
    ax.plot([0, X_GOAL], [0, 0], [Z_REF, Z_REF], ':', color='green', lw=2, label='reference')
    for c in CTRLS:
        b = _b(meta, c)
        s = _sel(logs, res, b)
        P = logs['pos'][s, b]
        r = res[b]
        ax.plot(P[:, 0], P[:, 1], P[:, 2], color=COLOR[c], lw=1.5,
                label=f"{LABEL[c]}: {'SUCCESS' if r['success'] else 'FAIL'}, "
                      f"RMS |y| {r['rms_y']:.2f} m")
        if not r['success']:
            ax.scatter(*P[-1], color=COLOR[c], marker='x', s=80)
    for sgn in (-1, 1):
        ax.plot([0, X_GOAL], [sgn * E_Y_FAIL] * 2, [Z_REF] * 2, '--', color='grey', lw=1)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_zlabel('z (m)')
    ax.set_ylim(-6, 6)
    ax.set_zlim(Z_REF - 3, Z_REF + 3)
    ax.set_title(f'S084 straight-line flight in 12 m/s crosswind (seed {HEAD_SEED}); '
                 f'grey dashes = 5 m fail bound')
    ax.legend(fontsize=8, loc='upper left')
    ax.view_init(elev=25, azim=-70)
    path = os.path.join(out_dir, 'trajectory_3d.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_lateral_and_attitude(logs, res, meta, out_dir):
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    for c in CTRLS:
        b = _b(meta, c)
        s = _sel(logs, res, b)
        axes[0].plot(logs['t'][s], logs['pos'][s, b, 1], color=COLOR[c], lw=1.2, label=LABEL[c])
        axes[1].plot(logs['t'][s], np.degrees(logs['eul'][s, b, 0]), color=COLOR[c], lw=1.0,
                     label=LABEL[c])
    for sgn in (-1, 1):
        axes[0].axhline(sgn * E_Y_FAIL, color='grey', ls='--', lw=1)
        axes[1].axhline(sgn * 30, color='grey', ls=':', lw=1)
        axes[1].axhline(sgn * 40, color='k', ls='--', lw=1)
    axes[0].set_ylabel('lateral deviation y (m)')
    axes[0].set_title(f'Lateral deviation (seed {HEAD_SEED}); dashed = 5 m fail limit')
    axes[1].set_ylabel(r'roll $\phi$ (deg)')
    axes[1].set_title('Roll angle; dotted = 30 deg clamp, dashed = 40 deg fail limit')
    axes[1].set_xlabel('time (s)')
    for ax in axes:
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
    plt.tight_layout()
    path = os.path.join(out_dir, 'lateral_and_attitude.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_rls_convergence(logs, res, meta, out_dir):
    b = _b(meta, 'lqr_rls')
    s = _sel(logs, res, b)
    t = logs['t'][s]
    fig, ax = plt.subplots(figsize=(12, 4.5))
    ax.plot(t, logs['w_true'][s, b, 1], color='grey', lw=0.8,
            label=r'true wind acceleration $F_{w,y}/m$')
    ax.plot(t, logs['w_hat'][s, b, 1], color='tab:red', lw=1.2, label=r'RLS estimate $\hat w_y$')
    ax.axhline(W_EFF_MEAN, color='k', ls='--', lw=1,
               label=f'mean-wind acceleration {W_EFF_MEAN:.2f} m/s$^2$')
    ax.fill_between(t, W_EFF_MEAN - 0.5, W_EFF_MEAN + 0.5, color='k', alpha=0.08,
                    label=r'$\pm$0.5 m/s$^2$ band')
    if np.isfinite(res[b]['t_conv']):
        ax.axvline(res[b]['t_conv'], color='tab:red', ls=':', lw=1)
    ax.set_xlabel('time (s)')
    ax.set_ylabel(r'lateral wind acceleration (m/s$^2$)')
    ax.set_title(f"RLS wind estimate (seed {HEAD_SEED}): first inside band at "
                 f"{res[b]['t_conv']:.2f} s, RMS error vs true {res[b]['est_rms']:.3f} m/s$^2$")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)
    path = os.path.join(out_dir, 'rls_convergence.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_dryden(winds, meta, out_dir, t_show=110.0):
    w = winds[meta['seeds'].index(HEAD_SEED)]
    n = int(round(t_show / DT))
    t = np.arange(n) * DT
    fig, ax = plt.subplots(figsize=(12, 4.5))
    ax.plot(t, w[:n, 1], color='tab:blue', lw=1.0, label=r'$W_y^{total}$ (mean 12 m/s + gust)')
    ax.plot(t, w[:n, 2], color='tab:green', lw=1.0, label=r'$W_z^{turb}$')
    ax.axhline(MEAN_WIND_Y, color='k', ls='--', lw=1)
    ax.axhline(MEAN_WIND_Y + 2 * SIGMA_Y, color='grey', ls=':', lw=1,
               label=r'$\bar W_y \pm 2\sigma_w$')
    ax.axhline(MEAN_WIND_Y - 2 * SIGMA_Y, color='grey', ls=':', lw=1)
    ax.set_xlabel('time (s)')
    ax.set_ylabel('wind speed (m/s)')
    ax.set_title(f'Dryden wind realisation (seed {HEAD_SEED})')
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)
    path = os.path.join(out_dir, 'dryden_gusts.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_metrics_bar(res, meta, out_dir):
    S = meta['S']
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.6))
    x = np.arange(len(CTRLS))
    for j, key in enumerate(('rms_y', 'peak_y')):
        mean = [np.mean([res[c * S + s][key] for s in range(S)]) for c in range(len(CTRLS))]
        std = [np.std([res[c * S + s][key] for s in range(S)]) for c in range(len(CTRLS))]
        head = [res[_b(meta, c)][key] for c in CTRLS]
        axes[j].bar(x, mean, yerr=std, color=[COLOR[c] for c in CTRLS], alpha=0.8, capsize=5,
                    label=f'mean $\\pm$ std, {S} seeds')
        axes[j].plot(x, head, 'k^', ms=8, label=f'seed {HEAD_SEED}')
        axes[j].axhline(E_Y_FAIL, color='grey', ls='--', lw=1, label='fail threshold 5 m')
        axes[j].set_xticks(x)
        axes[j].set_xticklabels([LABEL[c] for c in CTRLS])
        axes[j].set_ylabel('m')
        axes[j].set_title('RMS lateral deviation' if key == 'rms_y' else 'Peak lateral deviation')
        axes[j].set_ylim(0, 8.0)
        axes[j].legend(fontsize=8, loc='upper center', ncol=2)
        axes[j].grid(alpha=0.3, axis='y')
    succ = [sum(res[c * S + s]['success'] for s in range(S)) for c in range(len(CTRLS))]
    pk = [np.mean([res[c * S + s]['peak_att'] for s in range(S)]) for c in range(len(CTRLS))]
    axes[2].bar(x - 0.2, succ, 0.4, color=[COLOR[c] for c in CTRLS], label='successes')
    for i, v in enumerate(succ):
        axes[2].text(i - 0.2, v + 0.3, f'{v}/{S}', ha='center', fontsize=9)
    ax2 = axes[2].twinx()
    ax2.bar(x + 0.2, pk, 0.4, color='grey', alpha=0.6, label='mean peak attitude')
    ax2.axhline(40, color='k', ls='--', lw=1)
    ax2.set_ylabel('mean peak attitude (deg)')
    axes[2].set_ylim(0, S + 3)
    axes[2].set_xticks(x)
    axes[2].set_xticklabels([LABEL[c] for c in CTRLS])
    axes[2].set_ylabel('missions completed')
    axes[2].set_title('Success count (left bars) and peak attitude (grey)')
    plt.tight_layout()
    path = os.path.join(out_dir, 'metrics_bar.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_path_deviation_3d(logs, res, meta, out_dir):
    b = _b(meta, 'lqr_rls')
    s = _sel(logs, res, b)
    P = logs['pos'][s, b]
    dev = np.abs(P[:, 1])
    segs = np.stack([P[:-1], P[1:]], axis=1)
    fig = plt.figure(figsize=(11, 6))
    ax = fig.add_subplot(111, projection='3d')
    lc = Line3DCollection(segs, cmap='plasma', linewidth=2.5)
    lc.set_array(dev[:-1])
    ax.add_collection3d(lc)
    tl = logs['t'][s]
    W = logs['wind'][s, b]
    for i in np.where(np.isclose(np.mod(tl, 5.0), 0.0, atol=DT_EST / 2))[0]:
        ax.quiver(P[i, 0], P[i, 1], P[i, 2] + 1.5, 0, W[i, 1] / 6, W[i, 2] / 6,
                  color='tab:blue', lw=1, arrow_length_ratio=0.3)
    ax.plot([0, X_GOAL], [0, 0], [Z_REF, Z_REF], ':', color='green', lw=1.5)
    ax.set_xlim(0, X_GOAL)
    ax.set_ylim(-1.5, max(3.5, P[:, 1].max() + 0.5))
    ax.set_zlim(Z_REF - 1, Z_REF + 3)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_zlabel('z (m)')
    fig.colorbar(lc, ax=ax, shrink=0.6, label='|y| (m)')
    ax.set_title(f'LQR + RLS path coloured by |y| (seed {HEAD_SEED}); blue arrows = wind '
                 f'(y, z) every 5 s, length / 6')
    ax.view_init(elev=30, azim=-75)
    path = os.path.join(out_dir, 'path_deviation_3d.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_envelope(res, meta, out_dir):
    S, C = meta['S'], meta['C']
    gi = [i for i, g in enumerate(meta['groups']) if g[1] == 'box']
    lims = np.array([meta['groups'][i][0] for i in gi])
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.5))
    for ci, c in enumerate(CTRLS):
        succ, rms = [], []
        for li in gi:
            R_ = [res[(li * C + ci) * S + s] for s in range(S)]
            succ.append(sum(r['success'] for r in R_))
            rms.append(np.mean([r['rms_y'] for r in R_]))
        axes[0].plot(lims, succ, 'o-', color=COLOR[c], label=LABEL[c])
        axes[1].plot(lims, rms, 'o-', color=COLOR[c], label=LABEL[c])
    axes[0].set_ylabel(f'missions completed (of {S})')
    axes[0].set_ylim(0, S + 1)
    axes[1].set_ylabel('mean RMS |y| (m)')
    axes[1].set_yscale('log')
    for ax in axes:
        ax.set_xlabel('common hard tilt limit (deg); fail limit stays 40 deg')
        ax.set_xticks([30, 31, 35, 39])
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
    axes[0].set_title('Success vs tilt limit (same limit for all controllers)')
    axes[1].set_title('Lateral tracking vs tilt limit')
    plt.tight_layout()
    path = os.path.join(out_dir, 'envelope_sensitivity.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def save_animation(logs, res, meta, out_dir, step=50):
    """Top-down XY view, all three controllers, seed 42; side bar = lateral wind speed."""
    t = logs['t']
    t_last = max(res[_b(meta, c)]['t_end'] for c in CTRLS)
    n = int(np.searchsorted(t, t_last + 1e-9))
    idx = np.unique(np.minimum(np.arange(0, n + step - 1, step), n - 1))
    fig = plt.figure(figsize=(11, 5))
    ax = fig.add_axes([0.07, 0.12, 0.72, 0.76])
    axw = fig.add_axes([0.86, 0.12, 0.05, 0.76])
    ax.plot([0, X_GOAL], [0, 0], ':', color='green', lw=1.5)
    for sgn in (-1, 1):
        ax.axhline(sgn * E_Y_FAIL, color='grey', ls='--', lw=1)
    ax.set_xlim(-2, X_GOAL + 2)
    ax.set_ylim(-6, 6)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    lines, dots = {}, {}
    for c in CTRLS:
        lines[c], = ax.plot([], [], color=COLOR[c], lw=1.4, label=LABEL[c])
        dots[c], = ax.plot([], [], 'o', color=COLOR[c], ms=6)
    ax.legend(fontsize=8, loc='lower right')
    bar = axw.bar([0], [0], color='tab:blue')[0]
    axw.set_ylim(0, 25)
    axw.set_xticks([])
    axw.set_ylabel(r'$W_y$ (m/s)')
    bh = _b(meta, 'lqr_rls')

    def update(f):
        i = idx[f]
        for c in CTRLS:
            b = _b(meta, c)
            j = min(i, int(np.searchsorted(t, res[b]['t_end'] + 1e-9)) - 1)
            lines[c].set_data(logs['pos'][:j + 1, b, 0], logs['pos'][:j + 1, b, 1])
            dots[c].set_data([logs['pos'][j, b, 0]], [logs['pos'][j, b, 1]])
        bar.set_height(logs['wind'][i, bh, 1])
        ax.set_title(f"S084 seed {HEAD_SEED}  t = {t[i]:.1f} s   W_y = "
                     f"{logs['wind'][i, bh, 1]:.1f} m/s   y(LQR+RLS) = "
                     f"{logs['pos'][i, bh, 1]:+.2f} m")
        return list(lines.values()) + list(dots.values()) + [bar]

    anim = animation.FuncAnimation(fig, update, frames=len(idx), interval=80, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    anim.save(path, writer='pillow', fps=12)
    plt.close(fig)
    print(f'Saved: {path} ({len(idx)} frames)')


# ── Main ──────────────────────────────────────────────────────────────────────
if __name__ == '__main__':
    # card claim check: output std of the Dryden filter with the card's gain vs corrected gain
    rng = np.random.default_rng(0)
    n_chk = int(2000 / DT)
    w_card = dryden_series(SIGMA_Y, L_Y, V_AIR, DT, n_chk, rng, card_gain=True)
    w_fix = dryden_series(SIGMA_Y, L_Y, V_AIR, DT, n_chk, np.random.default_rng(0))
    logs, res, meta, winds = run_simulation()
    out = os.path.normpath(OUTPUT_DIR)
    os.makedirs(out, exist_ok=True)
    S = meta['S']
    print('\n=== S084 Extreme Weather Wind Resistance - Results ===')
    tau = L_Y / V_AIR
    print(f'  Dryden check ({n_chk * DT:.0f} s sample, sigma_w = {SIGMA_Y} m/s, tau = {tau:.2f} s): '
          f'card gain sigma*sqrt(2V/(pi L)) -> std {w_card.std():.3f} m/s '
          f'(theory {SIGMA_Y * np.sqrt(2 / np.pi) / tau:.3f}); corrected gain sigma*sqrt(tau) '
          f'-> std {w_fix.std():.3f} m/s')
    wh = winds[meta['seeds'].index(HEAD_SEED)][:int(100 / DT)]
    print(f'  Wind seed {HEAD_SEED} (first 100 s): W_y mean {wh[:, 1].mean():.2f}, std '
          f'{wh[:, 1].std():.2f}, min {wh[:, 1].min():.2f}, max {wh[:, 1].max():.2f} m/s; '
          f'W_z std {wh[:, 2].std():.2f} m/s')
    allw = winds[:, :int(100 / DT)]
    print(f'  Wind over {S} seeds (first 100 s): W_y std per seed {allw[:, :, 1].std(axis=1).min():.2f}'
          f'-{allw[:, :, 1].std(axis=1).max():.2f} m/s, overall max {allw[:, :, 1].max():.2f} m/s')
    print(f'  Drag constant 0.5 rho C_D A = {0.5 * RHO_AIR * C_D * A_REF:.4f} kg/m')
    print(f'  Mean-wind drag: F = {MASS * W_EFF_MEAN:.3f} N -> {W_EFF_MEAN:.3f} m/s^2, '
          f'equilibrium tilt {np.degrees(np.arctan(W_EFF_MEAN / G)):.1f} deg; at 20 m/s: '
          f'{0.5 * RHO_AIR * C_D * A_REF * 400:.2f} N, tilt '
          f'{np.degrees(np.arctan(0.5 * RHO_AIR * C_D * A_REF * 400 / (MASS * G))):.1f} deg')
    print(f"  LQR: closed-loop eigenvalue real parts {np.sort(meta['eig'].real)[-1]:.3f} (slowest)"
          f" to {np.sort(meta['eig'].real)[0]:.2f}; steady-state offset under the mean wind "
          f"(linear model): y = {meta['x_ss'][1]:.3f} m, roll = "
          f"{np.degrees(meta['x_ss'][6]):.2f} deg")
    print(f"  Plant: fpv_5inch motors, m = {MASS} kg, T/W = {meta['params'].t2w:.2f}, "
          f"{1 / DT:.0f} Hz control, RLS {1 / DT_EST:.0f} Hz (Gamma = {GAMMA}, time constant "
          f"{DT_EST / GAMMA:.2f} s)")
    print(f'  Common hard tilt limit (commanded attitude clipped, all controllers): '
          f'{np.degrees(THETA_LIM):.0f} deg (card) for the headline and Monte Carlo; '
          f'sensitivity groups: ' + ', '.join(f'{g[0]:g} deg {g[1]}' for g in TILT_GROUPS[1:])
          + ' (box = LQR roll and pitch clipped separately, cone = their norm clipped; the PID '
          'always clips the total tilt)')
    print(f'\n  Headline seed {HEAD_SEED}:')
    print('  controller  result   t_end    RMS|y|  peak|y|  peak att  mean att  z RMS   '
          'energy   RLS conv  est RMS')
    for c in CTRLS:
        r = res[_b(meta, c)]
        est = (f"{r['t_conv']:6.2f} s  {r['est_rms']:.3f}" if c == 'lqr_rls'
               else '     -        -  ')
        print(f"  {LABEL[c]:10s}  {'SUCCESS' if r['success'] else 'FAIL   '}  {r['t_end']:6.2f}  "
              f"{r['rms_y']:6.3f}  {r['peak_y']:6.3f}   {r['peak_att']:6.2f}    "
              f"{r['mean_att']:6.2f}   {r['z_rms']:.3f}  {r['energy']:7.0f} J  "
              f"{est}  {r['fail']}")
    print(f'\n  Monte Carlo, {S} seeds ({MC_SEEDS[0]}-{MC_SEEDS[-1]}), identical wind per seed:')
    print('  controller  success  RMS|y| mean+-std   peak|y| mean+-std  peak att mean+-std  '
          'energy mean (successes)')
    for ci, c in enumerate(CTRLS):
        R_ = [res[ci * S + s] for s in range(S)]
        f = lambda k: (np.mean([r[k] for r in R_]), np.std([r[k] for r in R_]))  # noqa: E731
        en = [r['energy'] for r in R_ if r['success']]
        print(f"  {LABEL[c]:10s}  {sum(r['success'] for r in R_):2d}/{S}    "
              f"{f('rms_y')[0]:.3f} +- {f('rms_y')[1]:.3f}     {f('peak_y')[0]:.3f} +- "
              f"{f('peak_y')[1]:.3f}      {f('peak_att')[0]:5.2f} +- {f('peak_att')[1]:4.2f}"
              f"        {np.mean(en) if en else np.nan:.0f} J")
        fails = [f"seed {r['seed']}: {r['fail']} at {r['t_end']:.1f} s" for r in R_ if r['fail']]
        if fails:
            print('      failures: ' + '; '.join(fails))
    for ci, c in enumerate(CTRLS):
        pa = [res[ci * S + s]['peak_att'] for s in range(S) if res[ci * S + s]['success']]
        print(f'  {LABEL[c]}: peak attitude over successful runs {np.mean(pa):.2f} +- '
              f'{np.std(pa):.2f} deg (max {np.max(pa):.2f}); runs below 25 deg: '
              f'{sum(p < 25 for p in pa)}/{len(pa)}')
    for a, bb in (('lqr_rls', 'lqr'), ('lqr_rls', 'pid'), ('lqr', 'pid')):
        ia, ib = CTRLS.index(a), CTRLS.index(bb)
        wins = sum(res[ia * S + s]['rms_y'] < res[ib * S + s]['rms_y'] for s in range(S))
        print(f'  {LABEL[a]} has lower RMS |y| than {LABEL[bb]} on {wins}/{S} seeds')
    ic = CTRLS.index('lqr_rls')
    tc = np.array([res[ic * S + s]['t_conv'] for s in range(S)])
    er = np.array([res[ic * S + s]['est_rms'] for s in range(S)])
    print(f'  RLS: first within 0.5 m/s^2 of the mean-wind acceleration after '
          f'{np.nanmean(tc):.2f} +- {np.nanstd(tc):.2f} s (max {np.nanmax(tc):.2f} s, never on '
          f'{int(np.sum(~np.isfinite(tc)))} seeds); RMS error vs the true instantaneous wind '
          f'acceleration {er.mean():.3f} +- {er.std():.3f} m/s^2')

    C = meta['C']
    print(f'\n  Clamp binding, card limit 30 deg (total tilt within 0.5 deg of the limit):')
    for ci, c in enumerate(CTRLS):
        R_ = [res[ci * S + s] for s in range(S)]
        ok = [r for r in R_ if r['success']]
        bad = [r for r in R_ if not r['success']]
        nb_ok = sum(r['tilt_max'] >= 29.5 for r in ok)
        line = (f"  {LABEL[c]:10s}: peak total tilt >= 29.5 deg on {nb_ok}/{len(ok)} successful "
                f"and {sum(r['tilt_max'] >= 29.5 for r in bad)}/{len(bad)} failed runs; longest "
                f"continuous binding: successes max {max(r['bind_max'] for r in ok):.2f} s, ")
        line += (f"failures min {min(r['bind_max'] for r in bad):.2f} s" if bad else 'no failures')
        line += (f"; total binding time: successes median "
                 f"{np.median([r['bind_time'] for r in ok]):.2f} s")
        print(line)
    print("  Max total tilt at 30 deg box: " + ', '.join(
        f"{LABEL[c]} {max(res[ci * S + s]['tilt_max'] for s in range(S)):.2f} deg"
        for ci, c in enumerate(CTRLS)))
    print(f'\n  Tilt-limit sensitivity ({S} seeds each, fail limits unchanged: 40 deg, 5 m):')
    print('  limit  clamp  controller  success  RMS|y| mean+-std  peak|y| mean  peak att mean  '
          'failures')
    for li, (lim, mode) in enumerate(meta['groups']):
        for ci, c in enumerate(CTRLS):
            R_ = [res[(li * C + ci) * S + s] for s in range(S)]
            ry = np.array([r['rms_y'] for r in R_])
            fails = ', '.join(f"{r['seed']} ({'y' if r['fail'].startswith('|y|') else 'att'})"
                              for r in R_ if r['fail'])
            print(f"  {lim:5.1f}  {mode:4s}   {LABEL[c]:10s}  {sum(r['success'] for r in R_):2d}/{S}"
                  f"    {ry.mean():.3f} +- {ry.std():.3f}     "
                  f"{np.mean([r['peak_y'] for r in R_]):.3f}        "
                  f"{np.mean([r['peak_att'] for r in R_]):5.2f}       {fails if fails else '-'}")
        rr = {c: np.array([res[(li * C + ci) * S + s]['rms_y'] for s in range(S)])
              for ci, c in enumerate(CTRLS)}
        print(f"         paired RMS |y| wins: LQR + RLS < PID on "
              f"{np.sum(rr['lqr_rls'] < rr['pid'])}/{S}, LQR + RLS < LQR on "
              f"{np.sum(rr['lqr_rls'] < rr['lqr'])}/{S}, LQR < PID on "
              f"{np.sum(rr['lqr'] < rr['pid'])}/{S}")
    plot_envelope(res, meta, out)
    plot_trajectory_3d(logs, res, meta, out)
    plot_lateral_and_attitude(logs, res, meta, out)
    plot_rls_convergence(logs, res, meta, out)
    plot_dryden(winds, meta, out)
    plot_metrics_bar(res, meta, out)
    plot_path_deviation_3d(logs, res, meta, out)
    save_animation(logs, res, meta, out)
