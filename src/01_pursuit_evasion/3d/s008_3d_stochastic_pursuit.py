"""
S008 3D Upgrade — Stochastic Pursuit (Kalman Filter Tracking)
=============================================================
The evader moves in true 3D.  The pursuer receives noisy 3D position
measurements (σ = 0.3 m per axis) and compares five tracker modes:

  - kalman_3d         : 6-state KF on x, y, z; lead guidance with the KF
                        velocity (card: two-pass intercept prediction)
  - kalman_2d_althold : 4-state KF on x, y; z estimate HELD at the altitude
                        of the first measurement, v_z = 0 (card's
                        "2D KF + altitude hold")
  - kalman_2d_rawz    : 4-state KF on x, y; z taken from the raw measurement
                        every step, v_z = 0 (what the previous version did
                        under the AltHold label)
  - naive             : pure pursuit at the raw noisy measurement
  - oracle            : true position and velocity, same lead guidance

Every KF starts at the first measurement with zero velocity and
P0 = diag(σ²·I, 36·I) (same as the fixed 2D S008).

Process noise for the filter: continuous white-noise-acceleration model
    Q = q·[[Δt³/3·D, Δt²/2·D], [Δt²/2·D, Δt·D]],  D = diag(1, 1, q_z/q_xy)
(off-diagonal blocks as in the card's structure, with the standard
discretisation coefficients), q_xy = 1.0 m²/s³ (card 0.1 cannot follow the
helix's 2.67 m/s² turn; see the q sweep).  The previous version used a diagonal
per-step Q = q·diag(1,1,½,1,1,½); it is run as an ablation.

Evader types (all start crossing the line of sight, +y):
  - random_walk : 3-axis random acceleration (σ_a = 0.5 m/s² per axis)
  - helix       : radius 3 m, horizontal speed 2.83 m/s, climb 1.0 m/s
  - z_hop       : near-constant x-y velocity, ±0.75 m altitude hop every 1 s
                  starting at 0.5 s (hop: 1.5 m/s for 0.5 s)

Monte Carlo: run i uses evader seed EVADER_SEED0 + i and measurement seed
NOISE_SEED0 + i for EVERY mode (common random numbers).  Plotted run: run 0.

Usage:
    conda activate drones
    python src/01_pursuit_evasion/3d/s008_3d_stochastic_pursuit.py
"""

import sys
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from matplotlib.animation import FuncAnimation, PillowWriter

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..'))
from src.base.drone_base import DroneBase

# ── Parameters ───────────────────────────────────────────────
DT          = 1 / 48
Q_XY        = 1.0     # m²/s³ KF process noise intensity (x, y); card: 0.1, see Q_SWEEP
Q_Z_RATIO   = 0.5     # q_z / q_xy
MEAS_SIGMA  = 0.3     # m measurement noise per axis
P0_VEL_VAR  = 36.0    # (m/s)² initial KF velocity variance
ACCEL_STD   = 0.5     # m/s² evader random acceleration (per axis)
V_PURSUER   = 5.0     # m/s
V_EVADER    = 3.0     # m/s
CAPTURE_R   = 0.15    # m
MAX_TIME    = 30.0
Z_MIN, Z_MAX = 0.3, 8.0
HELIX_R     = 3.0     # m
HELIX_VZ    = 1.0     # m/s
HOP_START, HOP_PERIOD, HOP_LEN, HOP_VZ = 0.5, 1.0, 0.5, 1.5   # s, s, s, m/s
N_MC        = 100
Q_SWEEP     = [0.1, 1.0, 10.0, 100.0]   # m²/s³ — KF 3D process-noise sweep
EVADER_SEED0 = 1000
NOISE_SEED0  = 0

INIT_PURSUER = np.array([-4.0, 0.0, 2.0])
INIT_EVADER  = np.array([ 4.0, 0.0, 3.0])

TRACKER_MODES = ['kalman_3d', 'kalman_2d_althold', 'kalman_2d_rawz', 'naive', 'oracle']
EVADER_TYPES  = ['random_walk', 'helix', 'z_hop']
MAX_STEPS     = int(round(MAX_TIME / DT))

TRACKER_COLORS = {'kalman_3d': 'firebrick', 'kalman_2d_althold': 'darkorange',
                  'kalman_2d_rawz': 'goldenrod', 'naive': 'purple', 'oracle': 'forestgreen'}
TRACKER_LABELS = {'kalman_3d': 'Kalman 3D', 'kalman_2d_althold': '2D KF + AltHold',
                  'kalman_2d_rawz': '2D KF + raw z', 'naive': 'Naive', 'oracle': 'Oracle'}

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), '..', '..', '..',
                          'outputs', '01_pursuit_evasion', '3d', 's008_3d_stochastic_pursuit')


# ── Kalman filter (n-axis constant-velocity) ──────────────────

class KalmanCV:
    """Constant-velocity KF on n axes: state [p (n), v (n)].

    q_axes: per-axis process-noise intensity (m²/s³).
    q_form: 'cwna'  — Q = [[Δt³/3, Δt²/2], [Δt²/2, Δt]] ⊗ diag(q_axes)
            'diag'  — Q = diag(q_axes, q_axes) per step (previous version)
    """

    def __init__(self, q_axes, dt=DT, sigma=MEAS_SIGMA, q_form='cwna'):
        q_axes = np.asarray(q_axes, float)
        n = len(q_axes)
        I, Z = np.eye(n), np.zeros((n, n))
        D = np.diag(q_axes)
        self.n = n
        self.F = np.block([[I, dt * I], [Z, I]])
        self.H = np.hstack([I, Z])
        if q_form == 'cwna':
            self.Q = np.block([[dt ** 3 / 3 * D, dt ** 2 / 2 * D],
                               [dt ** 2 / 2 * D, dt * D]])
        else:
            self.Q = np.diag(np.concatenate([q_axes, q_axes]))
        self.R = sigma ** 2 * I
        self.x = np.zeros(2 * n)
        self.P = np.eye(2 * n)

    def init_from_measurement(self, z, p0_vel_var=P0_VEL_VAR):
        self.x = np.concatenate([z, np.zeros(self.n)])
        self.P = np.diag([MEAS_SIGMA ** 2] * self.n + [p0_vel_var] * self.n)

    def step(self, z):
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ (z - self.H @ self.x)
        self.P = (np.eye(2 * self.n) - K @ self.H) @ self.P


# ── Evader trajectory generators (positions and velocities) ───

def gen_random_walk(rng):
    pos = INIT_EVADER.astype(float).copy()
    vel = np.array([0.0, V_EVADER, 0.0])
    P, V = [pos.copy()], [vel.copy()]
    for _ in range(MAX_STEPS):
        vel = vel + rng.normal(0, ACCEL_STD, 3) * DT
        spd = np.linalg.norm(vel)
        if spd > V_EVADER:
            vel = vel / spd * V_EVADER
        vel[2] = np.clip(vel[2], -1.0, 1.0)
        pos = pos + vel * DT
        if not (Z_MIN <= pos[2] <= Z_MAX):
            pos[2] = np.clip(pos[2], Z_MIN, Z_MAX); vel[2] = 0.0
        P.append(pos.copy()); V.append(vel.copy())
    return np.array(P), np.array(V)


def gen_helix():
    vh = np.sqrt(V_EVADER ** 2 - HELIX_VZ ** 2)
    om = vh / HELIX_R
    centre = INIT_EVADER[:2] - np.array([HELIX_R, 0.0])
    t = np.arange(MAX_STEPS + 1) * DT
    z = np.minimum(INIT_EVADER[2] + HELIX_VZ * t, Z_MAX)
    vz = np.where(INIT_EVADER[2] + HELIX_VZ * t < Z_MAX, HELIX_VZ, 0.0)
    P = np.column_stack([centre[0] + HELIX_R * np.cos(om * t),
                         centre[1] + HELIX_R * np.sin(om * t), z])
    V = np.column_stack([-vh * np.sin(om * t), vh * np.cos(om * t), vz])
    return P, V


def gen_z_hop(rng):
    pos = INIT_EVADER.astype(float).copy()
    vel = np.array([0.0, 0.9 * V_EVADER, 0.0])
    P, V = [pos.copy()], [vel.copy()]
    hop_sign = 0.0
    for k in range(MAX_STEPS):
        t = k * DT
        vel[:2] = vel[:2] + rng.normal(0, ACCEL_STD, 2) * DT
        sxy = np.linalg.norm(vel[:2])
        if sxy > 0.9 * V_EVADER:
            vel[:2] *= 0.9 * V_EVADER / sxy
        ph = (t - HOP_START) % HOP_PERIOD if t >= HOP_START else -1.0
        if 0 <= ph < DT / 2 or (t >= HOP_START and abs(ph - HOP_PERIOD) < DT / 2):
            hop_sign = rng.choice([-1.0, 1.0])            # new hop
        vel[2] = hop_sign * HOP_VZ if 0 <= ph < HOP_LEN - 1e-9 else 0.0
        pos = pos + vel * DT
        if not (Z_MIN <= pos[2] <= Z_MAX):
            pos[2] = np.clip(pos[2], Z_MIN, Z_MAX); vel[2] = 0.0
        P.append(pos.copy()); V.append(vel.copy())
    return np.array(P), np.array(V)


def get_evader_trajectory(evader_type, seed):
    rng = np.random.default_rng(seed)
    if evader_type == 'random_walk':
        return gen_random_walk(rng)
    if evader_type == 'helix':
        return gen_helix()
    if evader_type == 'z_hop':
        return gen_z_hop(rng)
    raise ValueError(evader_type)


# ── Guidance ─────────────────────────────────────────────────

def predictive_pursuit_cmd(pos_p, pos_hat, vel_hat, v_p=V_PURSUER):
    """Card: two-pass lead prediction p_pred = p_hat + v_hat·T_int."""
    p_pred = pos_hat.copy()
    for _ in range(2):
        t_int = np.linalg.norm(p_pred - pos_p) / v_p
        p_pred = pos_hat + vel_hat * t_int
    d = p_pred - pos_p
    n = np.linalg.norm(d)
    return v_p * d / n if n > 1e-8 else np.zeros(3)


# ── Simulation ───────────────────────────────────────────────

def run_simulation(tracker_mode, evader_traj, evader_vel, noise_seed, q_form='cwna',
                   q_xy=Q_XY):
    """Same evader trajectory and measurement noise for every mode: one 3-axis
    noise sample is drawn per step regardless of mode."""
    rng = np.random.default_rng(noise_seed)
    pursuer = DroneBase(INIT_PURSUER.copy(), max_speed=V_PURSUER, dt=DT)
    kf3 = KalmanCV([q_xy, q_xy, q_xy * Q_Z_RATIO], q_form=q_form)
    kf2 = KalmanCV([q_xy, q_xy], q_form=q_form)
    z_hold = None

    p_l, est_l, meas_l, err_l, vol_l, t_l = ([pursuer.pos.copy()], [np.full(3, np.nan)],
                                             [np.full(3, np.nan)], [np.full(3, np.nan)],
                                             [np.nan], [0.0])
    captured, cap_time = False, None
    for step in range(1, len(evader_traj)):
        t = step * DT
        true_pos = evader_traj[step]
        meas = true_pos + rng.normal(0, MEAS_SIGMA, 3)
        vol = np.nan
        if tracker_mode == 'kalman_3d':
            if step == 1:
                kf3.init_from_measurement(meas)
            else:
                kf3.step(meas)
            pos_hat, vel_hat = kf3.x[:3].copy(), kf3.x[3:].copy()
            ev = np.maximum(np.linalg.eigvalsh(kf3.P[:3, :3]), 0)
            vol = 4 * np.pi / 3 * np.prod(np.sqrt(ev))
        elif tracker_mode in ('kalman_2d_althold', 'kalman_2d_rawz'):
            if step == 1:
                kf2.init_from_measurement(meas[:2]); z_hold = meas[2]
            else:
                kf2.step(meas[:2])
            z_est = z_hold if tracker_mode == 'kalman_2d_althold' else meas[2]
            pos_hat = np.array([kf2.x[0], kf2.x[1], z_est])
            vel_hat = np.array([kf2.x[2], kf2.x[3], 0.0])
        elif tracker_mode == 'naive':
            pos_hat, vel_hat = meas.copy(), None
        else:
            pos_hat, vel_hat = true_pos.copy(), evader_vel[step].copy()

        if vel_hat is None:
            d = pos_hat - pursuer.pos
            n = np.linalg.norm(d)
            cmd = V_PURSUER * d / n if n > 1e-8 else np.zeros(3)
        else:
            cmd = predictive_pursuit_cmd(pursuer.pos, pos_hat, vel_hat)
        pursuer.step(cmd)
        pursuer.pos[2] = np.clip(pursuer.pos[2], Z_MIN, Z_MAX)

        p_l.append(pursuer.pos.copy()); est_l.append(pos_hat.copy())
        meas_l.append(meas.copy()); err_l.append(pos_hat - true_pos)
        vol_l.append(vol); t_l.append(t)
        if np.linalg.norm(pursuer.pos - true_pos) < CAPTURE_R:
            captured, cap_time = True, t
            break

    return dict(p_traj=np.array(p_l), est_traj=np.array(est_l), meas_log=np.array(meas_l),
                err_xyz=np.array(err_l), cov_vol=np.array(vol_l), times=np.array(t_l),
                captured=captured, cap_time=cap_time, mode=tracker_mode)


def run_monte_carlo():
    """caps[et][mode], ez[et][mode] (mean |e_z|), exy (mean horizontal error);
    plus ablations of the Kalman 3D filter."""
    caps = {et: {m: [] for m in TRACKER_MODES + ['kf3d_diagQ']} for et in EVADER_TYPES}
    ez = {et: {m: [] for m in TRACKER_MODES + ['kf3d_diagQ']} for et in EVADER_TYPES}
    exy = {et: {m: [] for m in TRACKER_MODES + ['kf3d_diagQ']} for et in EVADER_TYPES}
    for et_name in EVADER_TYPES:
        for i in range(N_MC):
            et, ev = get_evader_trajectory(et_name, EVADER_SEED0 + i)
            for m in TRACKER_MODES + ['kf3d_diagQ']:
                if m == 'kf3d_diagQ':
                    r = run_simulation('kalman_3d', et, ev, NOISE_SEED0 + i, q_form='diag')
                else:
                    r = run_simulation(m, et, ev, NOISE_SEED0 + i)
                caps[et_name][m].append(r['cap_time'] if r['captured'] else np.nan)
                e = r['err_xyz'][1:]
                ez[et_name][m].append(np.mean(np.abs(e[:, 2])))
                exy[et_name][m].append(np.mean(np.linalg.norm(e[:, :2], axis=1)))
    conv = lambda d: {k: {m: np.array(v) for m, v in dd.items()} for k, dd in d.items()}
    return conv(caps), conv(ez), conv(exy)


def run_q_sweep():
    """Kalman 3D with q_xy in Q_SWEEP (q_z = ratio·q_xy), same N_MC runs."""
    out = {et: {} for et in EVADER_TYPES}
    for et_name in EVADER_TYPES:
        for q in Q_SWEEP:
            c, e = [], []
            for i in range(N_MC):
                et, ev = get_evader_trajectory(et_name, EVADER_SEED0 + i)
                r = run_simulation('kalman_3d', et, ev, NOISE_SEED0 + i, q_xy=q)
                c.append(r['cap_time'] if r['captured'] else np.nan)
                e.append(np.mean(np.linalg.norm(r['err_xyz'][1:], axis=1)))
            out[et_name][q] = (np.array(c), np.array(e))
    return out


# ── Plots ────────────────────────────────────────────────────

def plot_q_sweep(sweep, out_dir):
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(13, 4.8))
    cols = {'random_walk': 'steelblue', 'helix': 'firebrick', 'z_hop': 'darkorange'}
    for et_name in EVADER_TYPES:
        med = [np.nanmedian(sweep[et_name][q][0]) if np.any(~np.isnan(sweep[et_name][q][0]))
               else np.nan for q in Q_SWEEP]
        ncap = [int(np.sum(~np.isnan(sweep[et_name][q][0]))) for q in Q_SWEEP]
        err = [sweep[et_name][q][1].mean() for q in Q_SWEEP]
        a1.semilogx(Q_SWEEP, med, 'o-', color=cols[et_name], label=et_name)
        for q, m_, n_ in zip(Q_SWEEP, med, ncap):
            a1.annotate(f'{n_}/{N_MC}', (q, m_), textcoords='offset points', xytext=(4, 4),
                        fontsize=7, color=cols[et_name])
        a2.semilogx(Q_SWEEP, err, 'o-', color=cols[et_name], label=et_name)
    a1.axvline(Q_XY, color='grey', linestyle=':', label=f'used q = {Q_XY} (card: 0.1)')
    a2.axvline(Q_XY, color='grey', linestyle=':')
    a2.axhline(MEAS_SIGMA * np.sqrt(3), color='grey', linestyle='--',
               label=f'raw measurement RMS σ√3 = {MEAS_SIGMA * np.sqrt(3):.2f} m')
    a1.set_xlabel('KF process noise q_xy (m²/s³)'); a1.set_ylabel('Median capture time (s)')
    a1.set_title('Kalman 3D: median capture time (labels = captured runs)', fontsize=10)
    a2.set_xlabel('KF process noise q_xy (m²/s³)'); a2.set_ylabel('Mean 3D position error (m)')
    a2.set_title('Kalman 3D: mean position estimation error', fontsize=10)
    for a in (a1, a2):
        a.grid(True, which='both', alpha=0.3); a.legend(fontsize=8)
    fig.suptitle(f'S008 3D — process-noise sweep ({N_MC} runs each, common random numbers)',
                 fontsize=11)
    plt.tight_layout()
    path = os.path.join(out_dir, 'q_sweep.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def plot_trajectories_3d(results, evader_traj, evader_name, out_dir):
    fig = plt.figure(figsize=(22, 10))
    for idx, mode in enumerate(TRACKER_MODES):
        res = results[mode]
        ax = fig.add_subplot(2, 3, idx + 1, projection='3d')
        n = len(res['p_traj'])
        et = evader_traj[:n]
        ax.plot(et[:, 0], et[:, 1], et[:, 2], color='royalblue', linewidth=1.6,
                label='Evader (true)')
        if mode != 'oracle':
            ax.scatter(res['meas_log'][1::8, 0], res['meas_log'][1::8, 1],
                       res['meas_log'][1::8, 2], color='lightcoral', s=6, alpha=0.5,
                       label='Measurements')
        if mode.startswith('kalman'):
            ax.plot(res['est_traj'][1:, 0], res['est_traj'][1:, 1], res['est_traj'][1:, 2],
                    color=TRACKER_COLORS[mode], linewidth=1.0, linestyle='--',
                    label='Tracker estimate')
        ax.plot(res['p_traj'][:, 0], res['p_traj'][:, 1], res['p_traj'][:, 2],
                color='red', linewidth=1.8, label='Pursuer')
        ax.scatter(*et[0], color='blue', s=40); ax.scatter(*res['p_traj'][0], color='red', s=40)
        if res['captured']:
            ax.scatter(*res['p_traj'][-1], color='black', s=80, marker='X')
            status = f"Captured {res['cap_time']:.2f} s"
        else:
            status = f'Not captured in {MAX_TIME:.0f} s'
        ax.set_title(f"{TRACKER_LABELS[mode]}\n{status}", fontsize=9)
        ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
        ax.legend(fontsize=6, loc='upper left')
    fig.suptitle(f'S008 3D Stochastic Pursuit — {evader_name} evader, MC run 0', fontsize=11)
    plt.tight_layout()
    path = os.path.join(out_dir, 'trajectories_3d.png')
    plt.savefig(path, dpi=130, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_per_axis_error(results, evader_name, out_dir):
    modes = ['kalman_3d', 'kalman_2d_althold', 'kalman_2d_rawz', 'naive']
    fig, axes = plt.subplots(1, 3, figsize=(16, 4.5), sharey=True)
    t_end = min(4.0, max(results[m]['times'][-1] for m in modes))
    for col, lab in enumerate(['x', 'y', 'z']):
        ax = axes[col]
        for m in modes:
            r = results[m]
            ax.plot(r['times'][1:], np.abs(r['err_xyz'][1:, col]), color=TRACKER_COLORS[m],
                    linewidth=1.0 if m == 'naive' else 1.5, alpha=0.6 if m == 'naive' else 1.0,
                    label=TRACKER_LABELS[m])
        ax.axhline(CAPTURE_R, color='grey', linestyle=':', label=f'capture radius {CAPTURE_R} m')
        ax.set_xlim(0, t_end); ax.set_ylim(0, 1.2)
        ax.set_title(f'|e_{lab}|', fontsize=10); ax.set_xlabel('Time (s)')
        ax.grid(True, alpha=0.3)
    axes[0].set_ylabel('Estimation error (m)')
    axes[2].legend(fontsize=7)
    ah = results['kalman_2d_althold']
    axes[2].text(0.02, 0.97, f'AltHold |e_z| leaves the axis; it reaches '
                 f'{np.nanmax(np.abs(ah["err_xyz"][ah["times"] <= t_end, 2])):.2f} m by '
                 f'{t_end:.1f} s', transform=axes[2].transAxes, fontsize=7, va='top',
                 color=TRACKER_COLORS['kalman_2d_althold'])
    fig.suptitle(f'S008 3D — per-axis estimation error, {evader_name} evader, MC run 0 '
                 f'(first {t_end:.1f} s)', fontsize=11)
    plt.tight_layout()
    path = os.path.join(out_dir, 'per_axis_error.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def plot_covariance_volume(res_by_type, out_dir):
    fig, ax = plt.subplots(figsize=(10, 5))
    cols = {'random_walk': 'steelblue', 'helix': 'firebrick', 'z_hop': 'darkorange'}
    for et_name, r in res_by_type.items():
        ax.semilogy(r['times'][1:], r['cov_vol'][1:], color=cols[et_name], linewidth=1.8,
                    label=f'{et_name} (capture {r["cap_time"]:.2f} s)' if r['captured']
                    else et_name)
    v0 = 4 * np.pi / 3 * MEAS_SIGMA ** 3
    ax.axhline(v0, color='grey', linestyle=':', label=f'one raw measurement: 4π/3·σ³ = {v0:.3f} m³')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Position covariance ellipsoid volume (m³, log)')
    ax.set_title('S008 3D — Kalman 3D: 4π/3·∏√λ(P_pos) vs time (MC run 0)\n'
                 'The three curves coincide: in a linear KF, P does not depend on the measurements', fontsize=10)
    ax.grid(True, which='both', alpha=0.3); ax.legend(fontsize=8)
    plt.tight_layout()
    path = os.path.join(out_dir, 'covariance_volume.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def plot_capture_table(caps, out_dir):
    fig, ax = plt.subplots(figsize=(15, 3.6))
    ax.axis('off')
    data, colors = [], []
    for et in EVADER_TYPES:
        row, rc = [], []
        for m in TRACKER_MODES:
            c = caps[et][m]
            nc = int(np.sum(~np.isnan(c)))
            if nc == 0:
                row.append(f'0/{N_MC}'); rc.append('#f9ebea')
            else:
                row.append(f'{nc}/{N_MC}, median {np.nanmedian(c):.2f} s')
                rc.append('#d4efdf' if nc == N_MC else '#fcf3cf')
        data.append(row); colors.append(rc)
    tb = ax.table(cellText=data, rowLabels=EVADER_TYPES,
                  colLabels=[TRACKER_LABELS[m] for m in TRACKER_MODES],
                  cellColours=colors, loc='center', cellLoc='center')
    tb.auto_set_font_size(False); tb.set_fontsize(10); tb.scale(1.1, 2.0)
    ax.set_title(f'S008 3D — captures and median capture time over {N_MC} runs '
                 f'(common random numbers, timeout {MAX_TIME:.0f} s)', fontsize=10, pad=18)
    plt.tight_layout()
    path = os.path.join(out_dir, 'capture_table.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def save_animation(results, evader_traj, evader_name, out_dir):
    """Kalman 3D vs 2D KF + AltHold against the same evader and noise (run 0)."""
    runs = [results['kalman_3d'], results['kalman_2d_althold']]
    T_SHOW = 6.0
    n_max = min(max(len(r['p_traj']) for r in runs), int(T_SHOW / DT) + 1)
    step = 3
    n_frames = -(-(n_max - 1) // step) + 1   # ceil, so the last step is shown once
    fig = plt.figure(figsize=(14, 6.5))
    arts = []
    for k, r in enumerate(runs):
        ax = fig.add_subplot(1, 2, k + 1, projection='3d')
        n = min(len(r['p_traj']), n_max)
        pts = np.vstack([r['p_traj'][:n], evader_traj[:n]])
        lo, hi = pts.min(axis=0) - 0.8, pts.max(axis=0) + 0.8
        ax.set_xlim(lo[0], hi[0]); ax.set_ylim(lo[1], hi[1]); ax.set_zlim(lo[2], hi[2])
        ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
        lt, = ax.plot([], [], [], color='royalblue', linewidth=1.5, label='Evader (true)')
        le, = ax.plot([], [], [], color=TRACKER_COLORS[r['mode']], linewidth=1.0,
                      linestyle='--', label='Estimate')
        lp, = ax.plot([], [], [], color='red', linewidth=1.8, label='Pursuer')
        dt_, = ax.plot([], [], [], 'bs', markersize=7)
        dp, = ax.plot([], [], [], 'r^', markersize=8)
        ax.legend(fontsize=7, loc='upper left')
        ti = ax.set_title('', fontsize=9)
        arts.append((lt, le, lp, dt_, dp, ti))

    def update(i):
        out = []
        for r, (lt, le, lp, dt_, dp, ti) in zip(runs, arts):
            n = min(len(r['p_traj']), n_max)
            si = min(i * step, n - 1)
            et, est, p = evader_traj, r['est_traj'], r['p_traj']
            lt.set_data(et[:si + 1, 0], et[:si + 1, 1]); lt.set_3d_properties(et[:si + 1, 2])
            le.set_data(est[1:si + 1, 0], est[1:si + 1, 1]); le.set_3d_properties(est[1:si + 1, 2])
            lp.set_data(p[:si + 1, 0], p[:si + 1, 1]); lp.set_3d_properties(p[:si + 1, 2])
            dt_.set_data([et[si, 0]], [et[si, 1]]); dt_.set_3d_properties([et[si, 2]])
            dp.set_data([p[si, 0]], [p[si, 1]]); dp.set_3d_properties([p[si, 2]])
            if si == len(r['p_traj']) - 1 and r['captured']:
                ti.set_text(f'{TRACKER_LABELS[r["mode"]]}\nCaptured {r["cap_time"]:.2f} s')
            else:
                ti.set_text(f'{TRACKER_LABELS[r["mode"]]}\nt = {r["times"][si]:.2f} s, '
                            f'|e_z| = {abs(r["err_xyz"][si, 2]) if si else 0:.2f} m')
            out += [lt, le, lp, dt_, dp, ti]
        return out

    fig.suptitle(f'S008 3D — {evader_name} evader, MC run 0 (first {T_SHOW:.0f} s)', fontsize=10)
    ani = FuncAnimation(fig, update, frames=n_frames, interval=60, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer=PillowWriter(fps=16), dpi=80)
    plt.close()
    print(f'Saved: {path}  ({n_frames} frames, every {step} steps)')


# ── Main ─────────────────────────────────────────────────────

if __name__ == '__main__':
    out_dir = os.path.normpath(OUTPUT_DIR)
    os.makedirs(out_dir, exist_ok=True)

    print('Evader checks (run 0):')
    for et_name in EVADER_TYPES:
        et, ev = get_evader_trajectory(et_name, EVADER_SEED0)
        k25 = int(2.5 / DT)
        print(f'  {et_name:<12} z range in first 2.5 s: {et[:k25, 2].min():.2f}–'
              f'{et[:k25, 2].max():.2f} m, speed range {np.linalg.norm(ev, axis=1).min():.2f}–'
              f'{np.linalg.norm(ev, axis=1).max():.2f} m/s')

    run0 = {}
    for et_name in EVADER_TYPES:
        et, ev = get_evader_trajectory(et_name, EVADER_SEED0)
        run0[et_name] = {m: run_simulation(m, et, ev, NOISE_SEED0) for m in TRACKER_MODES}
    print('\nMC run 0:')
    for et_name in EVADER_TYPES:
        for m in TRACKER_MODES:
            r = run0[et_name][m]
            st = f"captured @ {r['cap_time']:.3f} s" if r['captured'] else 'timeout'
            e = r['err_xyz'][1:]
            print(f'  [{et_name:<12}][{TRACKER_LABELS[m]:<16}] {st:<20} mean |e_z| '
                  f'{np.mean(np.abs(e[:, 2])):.3f} m, mean horiz. err '
                  f'{np.mean(np.linalg.norm(e[:, :2], axis=1)):.3f} m')

    r = run0['helix']['kalman_3d']
    vt = [(tq, r['cov_vol'][int(np.argmin(np.abs(r['times'] - tq)))]) for tq in (DT, 0.5, 1.0)]
    print('Kalman 3D covariance-ellipsoid volume (same for every evader): ' +
          ', '.join(f't={tq:.3f} s: {v:.4f} m³' for tq, v in vt))

    caps, ez, exy = run_monte_carlo()
    print(f'\nMonte Carlo ({N_MC} runs; run i: evader seed {EVADER_SEED0}+i, noise seed '
          f'{NOISE_SEED0}+i, same for every mode):')
    for et_name in EVADER_TYPES:
        for m in TRACKER_MODES + ['kf3d_diagQ']:
            c = caps[et_name][m]
            nc = int(np.sum(~np.isnan(c)))
            lab = TRACKER_LABELS.get(m, 'KF 3D, diagonal Q (old)')
            ctxt = (f'median {np.nanmedian(c):.3f} s, mean {np.nanmean(c):.3f} s, '
                    f'min {np.nanmin(c):.3f}, max {np.nanmax(c):.3f}') if nc else '—'
            print(f'  [{et_name:<12}][{lab:<24}] captured {nc:3d}/{N_MC} | {ctxt} | '
                  f'mean |e_z| {ez[et_name][m].mean():.3f} m | mean horiz. err '
                  f'{exy[et_name][m].mean():.3f} m')
        a, b = caps[et_name]['kalman_3d'], caps[et_name]['kalman_2d_rawz']
        both = ~np.isnan(a) & ~np.isnan(b)
        d = a[both] - b[both]
        print(f'  [{et_name:<12}] paired KF3D - 2D+raw z (both captured, {both.sum()} runs): '
              f'mean {d.mean():+.3f} s, KF3D faster in {int(np.sum(d < -1e-9))}, slower in '
              f'{int(np.sum(d > 1e-9))}, equal in {int(np.sum(np.abs(d) <= 1e-9))}')
        a2 = caps[et_name]['kf3d_diagQ']
        both = ~np.isnan(a) & ~np.isnan(a2)
        d = a[both] - a2[both]
        print(f'  [{et_name:<12}] paired KF3D (CWNA Q) - KF3D (diag Q) ({both.sum()} runs): '
              f'mean {d.mean():+.3f} s, CWNA faster in {int(np.sum(d < -1e-9))}, slower in '
              f'{int(np.sum(d > 1e-9))}')

    sweep = run_q_sweep()
    print(f'\nKalman 3D process-noise sweep (q_z = {Q_Z_RATIO}·q_xy, {N_MC} runs each):')
    for et_name in EVADER_TYPES:
        for q in Q_SWEEP:
            c, e = sweep[et_name][q]
            nc = int(np.sum(~np.isnan(c)))
            med = f'median {np.nanmedian(c):.3f} s, mean {np.nanmean(c):.3f} s' if nc else '—'
            print(f'  [{et_name:<12}] q_xy = {q:6.1f}: captured {nc:3d}/{N_MC} | {med} | '
                  f'mean 3D pos. error {e.mean():.3f} m')

    show = 'helix'
    et, ev = get_evader_trajectory(show, EVADER_SEED0)
    plot_trajectories_3d(run0[show], et, show, out_dir)
    plot_per_axis_error(run0[show], show, out_dir)
    plot_covariance_volume({k: run0[k]['kalman_3d'] for k in EVADER_TYPES}, out_dir)
    plot_capture_table(caps, out_dir)
    plot_q_sweep(sweep, out_dir)
    save_animation(run0[show], et, show, out_dir)
    print('Done.')
