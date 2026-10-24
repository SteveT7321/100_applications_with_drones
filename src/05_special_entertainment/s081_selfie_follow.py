"""
S081 Selfie Follow Mode
=======================
A single drone follows a person who walks a figure-8 (lobe radius R = 8 m) at a constant
1.5 m/s. The drone receives 10 Hz GPS fixes of the person (sigma = 0.3 m), estimates the
person's position and velocity, and holds a 5 m standoff at a bearing of 135 deg from the
person's heading (rear-left) with an orbit PD controller. The camera yaw points at the
*estimated* person position; the framing score is the fraction of time the *true* person
lies within +-10 % of the image half-width.

Estimators compared (same PD controller, same GPS noise for every estimator):
  (1) KF            - constant-velocity Kalman filter (card's method)
  (2) Raw GPS       - latest fix as position, one-step finite difference as velocity (card baseline)
  (3) Window LS     - least-squares line fit over the last 1 s of fixes (added, reasonable baseline)
  (4) Oracle        - true position and velocity (reference for the controller-only error)

Results are a Monte Carlo over N_MC GPS-noise seeds (common random numbers); the figures and
animation show seed 0.

Usage:
    conda activate drones
    python src/05_special_entertainment/s081_selfie_follow.py
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401

# ── Parameters (card) ─────────────────────────────────────────────────────────
R_FIGURE8    = 8.0                 # m   — figure-8 lobe radius
V_TARGET     = 1.5                 # m/s — person's walking speed (constant)
GPS_SIGMA    = 0.3                 # m   — GPS noise std per axis
GPS_FREQ     = 10.0                # Hz
STANDOFF_R   = 5.0                 # m   — horizontal standoff
BEARING_PHI  = np.radians(135.0)   # rad — offset from the person's heading (rear-left)
CRUISE_ALT   = 4.0                 # m
CAMERA_FOV   = np.radians(60.0)    # rad — horizontal field of view
FRAME_TOL    = 0.10                # |u| <= 0.10 counts as centred
Q_SIGMA      = 0.5                 # m/s^2 — KF process-noise spectral density
KP_ORBIT     = 1.5                 # 1/s^2
KD_ORBIT     = 0.8                 # 1/s
C_DRAG       = 0.3                 # 1/s
DT           = 0.05                # s
T_TOTAL      = 60.0                # s
N_STEPS      = int(round(T_TOTAL / DT))
GPS_EVERY    = int(round(1.0 / (GPS_FREQ * DT)))   # sim steps per GPS fix (2)

# ── Parameters (added) ────────────────────────────────────────────────────────
A_MAX        = 6.0                 # m/s^2 — horizontal acceleration limit (about 31 deg tilt)
V_MAX        = 10.0                # m/s   — horizontal speed limit
LS_WINDOW    = 10                  # fixes (1 s) for the windowed least-squares baseline
N_MC         = 100                 # Monte Carlo GPS-noise seeds
SEED         = 2026
SIGMA_SWEEP  = [0.1, 0.2, 0.3, 0.5, 0.8]   # m — GPS noise sensitivity
ANIM_STEP    = 4                   # sim steps per animation frame (0.2 s)

STRATS = ['kf', 'raw', 'ls', 'oracle']
LABELS = {'kf': 'KF + Orbit PD', 'raw': 'Raw GPS + Orbit PD',
          'ls': 'Window LS (1 s) + Orbit PD', 'oracle': 'Oracle (true state)'}
COLORS = {'kf': 'red', 'raw': 'orange', 'ls': 'purple', 'oracle': 'grey'}

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), '..', '..',
                          'outputs', '05_special_entertainment', 's081_selfie_follow')


# ── Target trajectory (true arc-length parameterisation) ─────────────────────

def _figure8_table(n=200_001):
    s = np.linspace(0.0, 2 * np.pi, n)
    x = R_FIGURE8 * np.sin(s)
    y = 0.5 * R_FIGURE8 * np.sin(2 * s)
    arc = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(x), np.diff(y)))])
    return s, arc


S_TAB, ARC_TAB = _figure8_table()
PATH_LEN = float(ARC_TAB[-1])


def figure8(t):
    """True position (N,2), velocity (N,2) and heading (N,) at times t, speed exactly V_TARGET."""
    t = np.atleast_1d(t)
    s = np.interp(np.mod(V_TARGET * t, PATH_LEN), ARC_TAB, S_TAB)
    pos = np.column_stack([R_FIGURE8 * np.sin(s), 0.5 * R_FIGURE8 * np.sin(2 * s)])
    tan = np.column_stack([R_FIGURE8 * np.cos(s), R_FIGURE8 * np.cos(2 * s)])
    tan /= np.linalg.norm(tan, axis=1, keepdims=True)
    return pos, V_TARGET * tan, np.arctan2(tan[:, 1], tan[:, 0])


def curvature_radius_min():
    s = S_TAB[::100]
    dx, dy = R_FIGURE8 * np.cos(s), R_FIGURE8 * np.cos(2 * s)
    ddx, ddy = -R_FIGURE8 * np.sin(s), -2 * R_FIGURE8 * np.sin(2 * s)
    kappa = np.abs(dx * ddy - dy * ddx) / (dx ** 2 + dy ** 2) ** 1.5
    return 1.0 / kappa.max()


# ── Kalman filter (vectorised over seeds; P and K are seed-independent) ──────

def kf_matrices(dt, sigma_gps):
    F = np.array([[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]], dtype=float)
    H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], dtype=float)
    Q = Q_SIGMA ** 2 * np.diag([dt ** 3 / 3, dt ** 3 / 3, dt, dt])
    R = sigma_gps ** 2 * np.eye(2)
    return F, H, Q, R


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


# ── Simulation ────────────────────────────────────────────────────────────────

def simulate(strategy, noise, sigma_gps=GPS_SIGMA, log=False):
    """Run the follow mission for all seeds at once.

    noise : (S, n_fix, 2) standard-normal draws, shared by every strategy (CRN).
    Returns per-seed metrics and, if log=True, the full time series of every seed.
    """
    S = noise.shape[0]
    t_all = np.arange(N_STEPS) * DT
    p_tgt, v_tgt, _ = figure8(t_all)

    F, H, Q, Rg = kf_matrices(DT, sigma_gps)
    x_kf = np.tile(np.array([*p_tgt[0], *v_tgt[0]]), (S, 1))
    P = np.eye(4)

    fix_t = np.zeros((S, 0))
    fixes = np.zeros((S, 0, 2))
    pos_est = np.tile(p_tgt[0], (S, 1))
    vel_est = np.tile(v_tgt[0], (S, 1))
    z_prev = None

    def desired(pe, ve):
        ang = np.arctan2(ve[:, 1], ve[:, 0]) + BEARING_PHI
        return pe + STANDOFF_R * np.column_stack([np.cos(ang), np.sin(ang)])

    p_drone = desired(pos_est, vel_est)
    v_drone = np.zeros((S, 2))
    p_d_prev = p_drone.copy()

    u_log = np.zeros((N_STEPS, S))
    e_log = np.zeros((N_STEPS, S))
    est_err = np.zeros((N_STEPS, S))
    if log:
        drone_log = np.zeros((N_STEPS, S, 2))
        est_log = np.zeros((N_STEPS, S, 2))
        yaw_log = np.zeros((N_STEPS, S))
    a_peak = 0.0
    v_peak = 0.0
    n_sat = 0
    a_raw_peak = 0.0

    for k in range(N_STEPS):
        t = k * DT
        got_fix = (k % GPS_EVERY == 0)
        if got_fix:
            z = p_tgt[k] + sigma_gps * noise[:, k // GPS_EVERY]

        if strategy == 'kf':
            if k > 0:
                x_kf = x_kf @ F.T
                P = F @ P @ F.T + Q
            if got_fix:
                Sm = H @ P @ H.T + Rg
                K = P @ H.T @ np.linalg.inv(Sm)
                x_kf = x_kf + (z - x_kf[:, :2]) @ K.T
                P = (np.eye(4) - K @ H) @ P
            pos_est, vel_est = x_kf[:, :2], x_kf[:, 2:]
        elif strategy == 'raw':
            if got_fix:
                if z_prev is not None:
                    vel_est = (z - z_prev) / (GPS_EVERY * DT)
                pos_est = z.copy()
                z_prev = z.copy()
        elif strategy == 'ls':
            if got_fix:
                fix_t = np.concatenate([fix_t, np.full((S, 1), t)], axis=1)[:, -LS_WINDOW:]
                fixes = np.concatenate([fixes, z[:, None, :]], axis=1)[:, -LS_WINDOW:]
            n = fixes.shape[1]
            if n >= 2:
                tc = fix_t - fix_t.mean(axis=1, keepdims=True)
                zc = fixes - fixes.mean(axis=1, keepdims=True)
                slope = np.einsum('sn,snd->sd', tc, zc) / np.sum(tc ** 2, axis=1)[:, None]
                vel_est = slope
                pos_est = fixes.mean(axis=1) + slope * (t - fix_t.mean(axis=1))[:, None]
            else:
                pos_est = fixes[:, -1]
        elif strategy == 'oracle':
            pos_est, vel_est = np.tile(p_tgt[k], (S, 1)), np.tile(v_tgt[k], (S, 1))

        # Orbit PD (card): a = Kp e + Kd (p_d_dot - v), then drag
        p_d = desired(pos_est, vel_est)
        e = p_d - p_drone
        e_dot = (p_d - p_d_prev) / DT - v_drone
        a_cmd = KP_ORBIT * e + KD_ORBIT * e_dot
        a_norm = np.linalg.norm(a_cmd, axis=1, keepdims=True)
        n_sat += int(np.sum(a_norm > A_MAX))
        a_raw_peak = max(a_raw_peak, float(a_norm.max()))
        a_cmd = np.where(a_norm > A_MAX, a_cmd / np.maximum(a_norm, 1e-12) * A_MAX, a_cmd)
        v_new = v_drone + (a_cmd - C_DRAG * v_drone) * DT
        v_norm = np.linalg.norm(v_new, axis=1, keepdims=True)
        v_new = np.where(v_norm > V_MAX, v_new / np.maximum(v_norm, 1e-12) * V_MAX, v_new)
        p_new = p_drone + v_new * DT

        # Physical invariants
        assert np.all(np.linalg.norm(a_cmd, axis=1) <= A_MAX + 1e-9)
        assert np.all(np.linalg.norm(v_new, axis=1) <= V_MAX + 1e-9)
        assert np.all(np.linalg.norm(p_new - p_drone, axis=1) <= V_MAX * DT + 1e-9)
        a_peak = max(a_peak, float(np.linalg.norm(a_cmd, axis=1).max()))
        v_peak = max(v_peak, float(np.linalg.norm(v_new, axis=1).max()))
        v_drone, p_drone, p_d_prev = v_new, p_new, p_d

        # Camera yaw points at the estimate; framing measured on the true person
        yaw = np.arctan2(pos_est[:, 1] - p_drone[:, 1], pos_est[:, 0] - p_drone[:, 0])
        d = p_tgt[k] - p_drone
        bearing = np.arctan2(d[:, 1], d[:, 0])
        u_log[k] = wrap(bearing - yaw) / (CAMERA_FOV / 2)
        e_log[k] = np.linalg.norm(d, axis=1) - STANDOFF_R
        est_err[k] = np.linalg.norm(pos_est - p_tgt[k], axis=1)
        if log:
            drone_log[k], est_log[k], yaw_log[k] = p_drone, pos_est, yaw

    assert CRUISE_ALT > 0.0      # the drone holds z = h > 0 (never below ground)
    res = {
        'F': np.mean(np.abs(u_log) <= FRAME_TOL, axis=0),
        'rms': np.sqrt(np.mean(e_log ** 2, axis=0)),
        'max': np.max(np.abs(e_log), axis=0),
        'est_rmse': np.sqrt(np.mean(est_err ** 2, axis=0)),
        'min_dist': np.min(e_log + STANDOFF_R, axis=0),
        'a_peak': a_peak, 'v_peak': v_peak, 'sat_frac': n_sat / (N_STEPS * S),
        'a_raw_peak': a_raw_peak,
    }
    if log:
        res.update({'t': t_all, 'p_tgt': p_tgt, 'u': u_log, 'e': e_log,
                    'drone': drone_log, 'est': est_log, 'yaw': yaw_log})
    return res


def run_simulation():
    rng = np.random.default_rng(SEED)
    n_fix = (N_STEPS - 1) // GPS_EVERY + 1
    noise = rng.standard_normal((N_MC, n_fix, 2))          # CRN: shared by all strategies
    results = {s: simulate(s, noise, log=True) for s in STRATS}

    sweep = {s: [] for s in ['kf', 'raw', 'ls']}
    for sig in SIGMA_SWEEP:
        for s in sweep:
            r = simulate(s, noise, sigma_gps=sig)
            sweep[s].append((r['F'].mean(), r['F'].std(), r['rms'].mean(), r['rms'].std()))
    return results, sweep


# ── Plotting ──────────────────────────────────────────────────────────────────

def plot_trajectory_3d(results, out_dir, seed=0):
    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(111, projection='3d')
    p = results['kf']['p_tgt']
    ax.plot(p[:, 0], p[:, 1], np.zeros(len(p)), 'b-', lw=2, label='Person (figure-8, z = 0)')
    for s, ls in [('kf', '-'), ('raw', '--'), ('ls', ':')]:
        d = results[s]['drone'][:, seed]
        ax.plot(d[:, 0], d[:, 1], np.full(len(d), CRUISE_ALT), color=COLORS[s], ls=ls,
                lw=1.4 if s == 'kf' else 1.0, label=f'Drone, {LABELS[s]}')
    ax.scatter(*p[0], 0, color='blue', s=60, marker='o')
    d0 = results['kf']['drone'][0, seed]
    ax.scatter(d0[0], d0[1], CRUISE_ALT, color='red', s=60, marker='^', label='Start')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_zlabel('z (m)')
    ax.set_zlim(0, 6)
    ax.set_box_aspect((1.6, 1.2, 0.5))
    ax.view_init(elev=30, azim=-60)
    ax.set_title(f'S081 Selfie Follow — 3D trajectories (seed {seed}, h = {CRUISE_ALT:.0f} m)')
    ax.legend(loc='upper left', fontsize=8)
    path = os.path.join(out_dir, 'trajectory_3d.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_metrics(results, out_dir, seed=0):
    fig, axes = plt.subplots(3, 1, figsize=(11, 10))
    t = results['kf']['t']
    ax = axes[0]
    for s in ['kf', 'raw', 'ls', 'oracle']:
        ax.plot(t, results[s]['e'][:, seed], color=COLORS[s], lw=1.0 if s != 'raw' else 0.7,
                ls='--' if s == 'oracle' else '-', label=LABELS[s])
    ax.axhline(0, color='k', lw=0.7, ls='--')
    ax.fill_between(t, -0.5, 0.5, alpha=0.12, color='green', label='±0.5 m band')
    ax.set_ylabel('Orbit error $e_{orbit}$ (m)')
    ax.set_title(f'Orbit distance error (seed {seed})')
    ax.legend(fontsize=8, ncol=3)
    ax.grid(alpha=0.3)

    ax = axes[1]
    for s in ['raw', 'ls', 'kf']:
        ax.plot(t, results[s]['u'][:, seed], color=COLORS[s], lw=0.8, label=LABELS[s])
    ax.axhline(FRAME_TOL, color='g', ls=':', lw=1.2, label=f'±{FRAME_TOL:.0%} tolerance')
    ax.axhline(-FRAME_TOL, color='g', ls=':', lw=1.2)
    ax.set_ylim(-1, 1)
    ax.set_ylabel('Frame offset $u$')
    ax.set_xlabel('Time (s)')
    ax.set_title('Camera framing offset (oracle: u = 0 by construction, not shown)')
    ax.legend(fontsize=8, ncol=4)
    ax.grid(alpha=0.3)

    ax = axes[2]
    bins = np.linspace(0, 1, 41)
    for s in ['raw', 'ls', 'kf']:
        ax.hist(np.abs(results[s]['u'][:, seed]), bins=bins, density=True, alpha=0.5,
                color=COLORS[s], label=LABELS[s])
    ax.axvline(FRAME_TOL, color='g', ls='--', lw=1.5, label=f'Tolerance {FRAME_TOL:.0%}')
    ax.set_xlabel('$|u|$')
    ax.set_ylabel('Density')
    ax.set_title('Distribution of |u|')
    ax.legend(fontsize=8)
    fig.suptitle('S081 Selfie Follow — metrics', fontsize=13)
    plt.tight_layout()
    path = os.path.join(out_dir, 'metrics.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_monte_carlo(results, out_dir):
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5))
    est = ['kf', 'raw', 'ls']
    ax1.boxplot([100 * results[s]['F'] for s in est], showmeans=True)
    ax1.set_xticks(range(1, len(est) + 1), [LABELS[s].replace(' + Orbit PD', '') for s in est])
    ax1.set_ylabel('Framing score F (%)')
    ax1.set_title(f'Framing score over {N_MC} GPS-noise seeds')
    ax1.grid(alpha=0.3)
    ax2.boxplot([results[s]['rms'] for s in STRATS], showmeans=True)
    ax2.set_xticks(range(1, len(STRATS) + 1),
                   [LABELS[s].replace(' + Orbit PD', '') for s in STRATS])
    ax2.set_ylabel('RMS orbit error (m)')
    ax2.set_title(f'RMS orbit error over {N_MC} seeds')
    ax2.grid(alpha=0.3)
    fig.suptitle('S081 Monte Carlo (same GPS noise for every estimator; triangle = mean)')
    plt.tight_layout()
    path = os.path.join(out_dir, 'monte_carlo.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_sensitivity(sweep, out_dir):
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4.5))
    for s, vals in sweep.items():
        v = np.array(vals)
        ax1.errorbar(SIGMA_SWEEP, 100 * v[:, 0], yerr=100 * v[:, 1], fmt='o-', capsize=3,
                     color=COLORS[s], label=LABELS[s])
        ax2.errorbar(SIGMA_SWEEP, v[:, 2], yerr=v[:, 3], fmt='o-', capsize=3,
                     color=COLORS[s], label=LABELS[s])
    ax1.set_xlabel('GPS noise σ (m)')
    ax1.set_ylabel('Framing score F (%)')
    ax1.set_title('Framing vs GPS noise')
    ax2.set_xlabel('GPS noise σ (m)')
    ax2.set_ylabel('RMS orbit error (m)')
    ax2.set_title('Orbit error vs GPS noise')
    for ax in (ax1, ax2):
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
    fig.suptitle(f'S081 Sensitivity to GPS noise (mean ± std over {N_MC} seeds)')
    plt.tight_layout()
    path = os.path.join(out_dir, 'sensitivity.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def anim_indices(n, step):
    n_frames = int(np.ceil((n - 1) / step)) + 1
    return [min(i * step, n - 1) for i in range(n_frames)]


def save_animation(results, out_dir, seed=0):
    r = results['kf']
    idx = anim_indices(N_STEPS, ANIM_STEP)
    fig, ax = plt.subplots(figsize=(7, 6))
    ax.set_xlim(-15, 15)
    ax.set_ylim(-11, 11)
    ax.set_aspect('equal')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.plot(r['p_tgt'][:, 0], r['p_tgt'][:, 1], 'b--', lw=0.8, alpha=0.4, label='Person path')
    tgt, = ax.plot([], [], 'bo', ms=9, label='Person')
    drn, = ax.plot([], [], 'r^', ms=10, label='Drone (KF)')
    link, = ax.plot([], [], 'k-', lw=0.8, alpha=0.5)
    fov_l, = ax.plot([], [], 'r:', lw=0.8)
    fov_r, = ax.plot([], [], 'r:', lw=0.8)
    circ = plt.Circle((0, 0), STANDOFF_R, color='grey', fill=False, ls=':', lw=1.0)
    ax.add_patch(circ)
    txt = ax.text(-14.5, 9.5, '', fontsize=9, color='darkred')
    ax.legend(loc='lower right', fontsize=8)
    ax.set_title('S081 Selfie Follow — top-down (KF, seed 0)')

    def update(i):
        k = idx[i]
        px, py = r['p_tgt'][k]
        dx, dy = r['drone'][k, seed]
        yaw = r['yaw'][k, seed]
        tgt.set_data([px], [py])
        drn.set_data([dx], [dy])
        link.set_data([px, dx], [py, dy])
        for line, sgn in ((fov_l, 1), (fov_r, -1)):
            a = yaw + sgn * CAMERA_FOV / 2
            line.set_data([dx, dx + 7 * np.cos(a)], [dy, dy + 7 * np.sin(a)])
        circ.center = (px, py)
        u = r['u'][k, seed]
        state = 'IN FRAME' if abs(u) <= FRAME_TOL else 'off-centre'
        txt.set_text(f't = {r["t"][k]:.1f} s   u = {u:+.2f}   [{state}]')
        return tgt, drn, link, fov_l, fov_r, circ, txt

    ani = animation.FuncAnimation(fig, update, frames=len(idx), blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer='pillow', fps=12, dpi=80)
    plt.close(fig)
    print(f'Saved: {path} ({len(idx)} frames)')


# ── Main ──────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    results, sweep = run_simulation()

    print('\n=== S081 Selfie Follow Mode — Results ===')
    print(f'  Figure-8 path length {PATH_LEN:.2f} m, lap time {PATH_LEN / V_TARGET:.2f} s, '
          f'{T_TOTAL * V_TARGET / PATH_LEN:.2f} laps in {T_TOTAL:.0f} s')
    rho = curvature_radius_min()
    print(f'  Min radius of curvature {rho:.2f} m -> max heading rate {V_TARGET / rho:.3f} rad/s, '
          f'peak lateral accel {V_TARGET ** 2 / rho:.3f} m/s^2')
    print(f'  Card code period 1/omega = {np.pi * R_FIGURE8 / V_TARGET:.2f} s -> mean speed '
          f'{PATH_LEN / (np.pi * R_FIGURE8 / V_TARGET):.2f} m/s')
    print(f'  Monte Carlo: {N_MC} seeds, same GPS noise for every estimator')
    print(f"  {'Strategy':<28}{'Framing F (%)':>18}{'RMS e_orbit (m)':>20}"
          f"{'Max |e_orbit| (m)':>20}{'Est. RMSE (m)':>16}")
    for s in STRATS:
        r = results[s]
        fstr = (f"{100 * r['F'].mean():6.1f} ± {100 * r['F'].std():4.1f}"
                if s != 'oracle' else '  n/a (u = 0)')
        print(f"  {LABELS[s]:<28}{fstr:>18}"
              f"{r['rms'].mean():>12.3f} ± {r['rms'].std():.3f}"
              f"{r['max'].mean():>12.3f} ± {r['max'].std():.3f}"
              f"{r['est_rmse'].mean():>10.3f}")
    print('  Seed 0 (figures):')
    for s in STRATS:
        r = results[s]
        print(f"    {LABELS[s]:<28} F = {100 * r['F'][0]:5.1f}%  RMS = {r['rms'][0]:.3f} m  "
              f"max = {r['max'][0]:.3f} m")
    for b in ['raw', 'ls']:
        wf = int(np.sum(results['kf']['F'] > results[b]['F']))
        tf = int(np.sum(results['kf']['F'] == results[b]['F']))
        wr = int(np.sum(results['kf']['rms'] < results[b]['rms']))
        print(f'  KF vs {LABELS[b]}: higher F in {wf}/{N_MC} seeds (ties {tf}), '
              f'lower RMS in {wr}/{N_MC} seeds')
    print('  Physical limits (all seeds): peak commanded accel / time at accel limit / '
          'peak speed / min horizontal distance to person')
    for s in STRATS:
        r = results[s]
        print(f"    {LABELS[s]:<28} {r['a_peak']:.2f} m/s^2 (limit {A_MAX:.0f}) / "
              f"{100 * r['sat_frac']:.2f}% / "
              f"{r['v_peak']:.2f} m/s (limit {V_MAX:.0f}) / {r['min_dist'].min():.2f} m; "
              f"peak unsaturated PD command {r['a_raw_peak']:.1f} m/s^2")
    for s in STRATS:
        e = results[s]['e']
        print(f"  {LABELS[s]}: mean e_orbit {e.mean():+.3f} m (negative = closer than {STANDOFF_R:.0f} m)")
    for s in ['raw', 'ls', 'kf']:
        sat = np.mean(np.abs(results[s]['u']) > 1.0) * 100
        print(f'  {LABELS[s]}: person outside the image (|u| > 1) {sat:.2f}% of the time')
    print('  GPS sigma sweep (F % / RMS m, mean over seeds):')
    for j, sig in enumerate(SIGMA_SWEEP):
        print(f'    sigma = {sig:.1f} m: ' + ', '.join(
            f"{s}: {100 * sweep[s][j][0]:.1f}% / {sweep[s][j][2]:.3f} m" for s in sweep))

    out = os.path.normpath(OUTPUT_DIR)
    os.makedirs(out, exist_ok=True)
    plot_trajectory_3d(results, out)
    plot_metrics(results, out)
    plot_monte_carlo(results, out)
    plot_sensitivity(sweep, out)
    save_animation(results, out)
