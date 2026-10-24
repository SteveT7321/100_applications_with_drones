"""
S086 Cooperative Multi-Angle Filming
====================================
Three camera drones orbit a walking subject (figure-8, a = 10 m, b = 6 m, RMS speed 1.5 m/s)
at radius 6 m and altitude 4 m, 120 deg apart. A shared constant-velocity Kalman filter
fuses 5 Hz position fixes (sigma = 0.3 m). Each drone tracks its orbit set-point with a
proportional velocity command (|u| <= 4 m/s); its camera points at the KF estimate.

Formation strategies compared (same observation noise in every strategy):
  (1) Heading-locked, KF   (card): base phase = KF heading of the subject
  (2) World-fixed,   KF   (added): base phase = 0 (formation does not rotate)
  (3) Heading-locked, raw (added): latest fix as position, two-fix difference as velocity

Monte Carlo over N_MC noise seeds; figures and animation show seed 0.

Usage:
    conda activate drones
    python src/05_special_entertainment/s086_multi_angle_cinema.py
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401

# ── Parameters (card) ─────────────────────────────────────────────────────────
N_DRONES = 3
ORBIT_RADIUS = 6.0        # m
FLY_ALT = 4.0             # m
SUBJECT_SPEED = 1.5       # m/s (RMS)
FIG8_A = 10.0             # m
FIG8_B = 6.0              # m
FIG8_OMEGA_CARD = 0.18    # rad/s (card value; gives 1.99 m/s RMS, see below)
D_MIN = 3.0               # m
KP = 2.0                  # 1/s
V_MAX = 4.0               # m/s
DT = 0.05                 # s
T_SIM = 60.0              # s
OBS_INTERVAL = 0.2        # s
SIGMA_A = 0.8             # m/s^2
SIGMA_OBS = 0.3           # m
FRAME_THRESH = 0.15
FOV_HALF_WIDTH = 2.0      # m at the orbit radius
PHASE_TOL = np.radians(5.0)

# ── Parameters (added / corrected) ────────────────────────────────────────────
FIG8_OMEGA = SUBJECT_SPEED / np.sqrt((FIG8_A ** 2 + 4 * FIG8_B ** 2) / 2)   # RMS speed = 1.5 m/s
N_MC = 50
SEED = 86
ANIM_STEP = 8             # sim steps per GIF frame (0.4 s)
T_LATE = 5.0              # s — phase errors also reported after the KF start-up

N_STEPS = int(round(T_SIM / DT))
OBS_EVERY = int(round(OBS_INTERVAL / DT))
STRATS = ['locked', 'fixed', 'raw']
LABELS = {'locked': 'Heading-locked, KF', 'fixed': 'World-fixed, KF',
          'raw': 'Heading-locked, raw fixes'}
S_COLORS = {'locked': 'tab:red', 'fixed': 'tab:green', 'raw': 'tab:orange'}
D_COLORS = ['red', 'orangered', 'firebrick']
PAIRS = [(0, 1), (0, 2), (1, 2)]

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), '..', '..',
                          'outputs', '05_special_entertainment', 's086_multi_angle_cinema')


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def subject(t, omega=FIG8_OMEGA):
    t = np.atleast_1d(t)
    pos = np.column_stack([FIG8_A * np.sin(omega * t), FIG8_B * np.sin(2 * omega * t)])
    vel = np.column_stack([FIG8_A * omega * np.cos(omega * t),
                           2 * FIG8_B * omega * np.cos(2 * omega * t)])
    return pos, vel


def kf_matrices():
    F = np.array([[1, 0, DT, 0], [0, 1, 0, DT], [0, 0, 1, 0], [0, 0, 0, 1]], dtype=float)
    H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], dtype=float)
    d4, d3, d2 = DT ** 4 / 4, DT ** 3 / 2, DT ** 2
    Q = SIGMA_A ** 2 * np.array([[d4, 0, d3, 0], [0, d4, 0, d3], [d3, 0, d2, 0], [0, d3, 0, d2]])
    return F, H, Q, SIGMA_OBS ** 2 * np.eye(2)


def framing_offset(p_drone, aim, s_true):
    """Normalised image offset (lateral, vertical) of the true subject for a camera at p_drone
    aimed at `aim`; |delta| = 1 at the image edge (half-width FOV_HALF_WIDTH at ORBIT_RADIUS)."""
    f = aim - p_drone
    f /= np.linalg.norm(f, axis=-1, keepdims=True)
    right = np.cross(f, np.array([0.0, 0.0, 1.0]))
    right /= np.linalg.norm(right, axis=-1, keepdims=True)
    up = np.cross(right, f)
    d = s_true - p_drone
    depth = np.sum(d * f, axis=-1)
    tan_half = FOV_HALF_WIDTH / ORBIT_RADIUS
    return np.stack([np.sum(d * right, axis=-1), np.sum(d * up, axis=-1)], axis=-1) \
        / (depth[..., None] * tan_half)


def simulate(strategy, noise, log=False):
    """All seeds at once. noise: (S, n_obs, 2) standard normal, shared by every strategy."""
    S = noise.shape[0]
    t_all = np.arange(N_STEPS) * DT
    q, qv = subject(t_all)
    F, H, Q, R = kf_matrices()
    x = np.zeros((S, 4))
    x[:, :2] = q[0]                          # card: position only, velocity starts at 0
    P = np.eye(4)
    pos_est = np.tile(q[0], (S, 1))
    vel_est = np.zeros((S, 2))
    z_prev = None
    offs = 2 * np.pi * np.arange(N_DRONES) / N_DRONES

    def setpoints(pe, base):
        ph = base[:, None] + offs[None, :]
        sp = np.zeros((S, N_DRONES, 3))
        sp[:, :, 0] = pe[:, None, 0] + ORBIT_RADIUS * np.cos(ph)
        sp[:, :, 1] = pe[:, None, 1] + ORBIT_RADIUS * np.sin(ph)
        sp[:, :, 2] = FLY_ALT
        return sp, ph

    drones, _ = setpoints(pos_est, np.zeros(S))
    phase_err = np.zeros((N_STEPS, S, N_DRONES))
    sep_err = np.zeros((N_STEPS, S, 3))
    dist = np.zeros((N_STEPS, S, 3))
    frame = np.zeros((N_STEPS, S, N_DRONES), dtype=bool)
    est_err = np.zeros((N_STEPS, S))
    if log:
        hist = np.zeros((N_STEPS, S, N_DRONES, 3))
        est_hist = np.zeros((N_STEPS, S, 2))
    for k in range(N_STEPS):
        obs = (k % OBS_EVERY == OBS_EVERY - 1)
        if obs:
            z = q[k] + SIGMA_OBS * noise[:, k // OBS_EVERY]
        if strategy in ('locked', 'fixed'):
            x = x @ F.T
            P = F @ P @ F.T + Q
            if obs:
                Sm = H @ P @ H.T + R
                K = P @ H.T @ np.linalg.inv(Sm)
                x = x + (z - x[:, :2]) @ K.T
                P = (np.eye(4) - K @ H) @ P
            pos_est, vel_est = x[:, :2], x[:, 2:]
        elif obs:
            if z_prev is not None:
                vel_est = (z - z_prev) / OBS_INTERVAL
            pos_est = z.copy()
            z_prev = z.copy()

        base = np.zeros(S) if strategy == 'fixed' else np.arctan2(vel_est[:, 1], vel_est[:, 0])
        sp, ph_des = setpoints(pos_est, base)
        u = KP * (sp - drones)
        n = np.linalg.norm(u, axis=2, keepdims=True)
        u = np.where(n > V_MAX, u / np.maximum(n, 1e-12) * V_MAX, u)
        assert np.all(np.linalg.norm(u, axis=2) <= V_MAX + 1e-9)
        drones = drones + u * DT
        assert np.all(drones[:, :, 2] > 0.0)

        rel = drones[:, :, :2] - pos_est[:, None, :]
        ph_act = np.arctan2(rel[:, :, 1], rel[:, :, 0])
        phase_err[k] = np.abs(wrap(ph_act - ph_des))
        for j, (a, b) in enumerate(PAIRS):
            dist[k, :, j] = np.linalg.norm(drones[:, a] - drones[:, b], axis=1)
            sep_err[k, :, j] = np.abs(wrap(ph_act[:, b] - ph_act[:, a] - 2 * np.pi * (b - a) / 3))
        aim = np.concatenate([pos_est, np.zeros((S, 1))], axis=1)[:, None, :]
        s_true = np.array([q[k, 0], q[k, 1], 0.0])
        delta = framing_offset(drones, np.broadcast_to(aim, drones.shape), s_true)
        frame[k] = np.max(np.abs(delta), axis=2) <= FRAME_THRESH
        est_err[k] = np.linalg.norm(pos_est - q[k], axis=1)
        if log:
            hist[k], est_hist[k] = drones, pos_est

    late = t_all >= T_LATE
    res = {
        'phase_mean_late': np.degrees(phase_err[late].mean(axis=(0, 2))),
        'phase_max_late': np.degrees(phase_err[late].max(axis=(0, 2))),
        'phase_mean': np.degrees(phase_err.mean(axis=(0, 2))),
        'phase_max': np.degrees(phase_err.max(axis=(0, 2))),
        'phase_ok': 100 * np.mean(phase_err <= PHASE_TOL, axis=(0, 2)),
        'sep_mean': np.degrees(sep_err.mean(axis=(0, 2))),
        'min_dist': dist.min(axis=(0, 2)),
        'frame': np.mean(frame, axis=(0, 2)),
        'kf_rmse': np.sqrt(np.mean(est_err ** 2, axis=0)),
    }
    if log:
        res.update({'t': t_all, 'q': q, 'hist': hist, 'est': est_hist,
                    'phase_err': phase_err, 'dist': dist, 'frame_t': frame})
    return res


def run_simulation():
    rng = np.random.default_rng(SEED)
    n_obs = N_STEPS // OBS_EVERY + 1
    noise = rng.standard_normal((N_MC, n_obs, 2))
    return {s: simulate(s, noise, log=True) for s in STRATS}


# ── Plotting ──────────────────────────────────────────────────────────────────

def plot_3d_formation(res, out_dir, seed=0):
    r = res['locked']
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    q = r['q']
    ax.plot(q[:, 0], q[:, 1], 0 * q[:, 0], color='blue', lw=1.5, alpha=0.6, label='Subject path')
    ax.scatter(q[-1, 0], q[-1, 1], 0, color='blue', s=100, label='Subject (final)')
    for k in range(N_DRONES):
        h = r['hist'][:, seed, k]
        ax.plot(h[:, 0], h[:, 1], h[:, 2], color=D_COLORS[k], lw=1.0, alpha=0.7,
                label=f'Drone {k} path')
        ax.scatter(*h[-1], color=D_COLORS[k], s=90, marker='^')
        ax.plot([h[-1, 0], q[-1, 0]], [h[-1, 1], q[-1, 1]], [h[-1, 2], 0], ':', color=D_COLORS[k])
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_zlabel('z (m)')
    ax.set_zlim(0, 8)
    ax.set_box_aspect((1.6, 1.3, 0.4))
    ax.set_title(f'S086 Heading-locked formation, KF (seed {seed}); triangles = final positions')
    ax.legend(loc='upper left', fontsize=8)
    path = os.path.join(out_dir, '3d_formation.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_sync_separation(res, out_dir, seed=0):
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    t = res['locked']['t']
    for s, ls in [('locked', '-'), ('fixed', '--')]:
        pe = np.degrees(res[s]['phase_err'][:, seed])
        axes[0].plot(t, pe.max(axis=1), color=S_COLORS[s], lw=1.1, ls=ls,
                     label=f'{LABELS[s]} (worst drone)')
    axes[0].axhline(5.0, color='grey', ls='--', lw=1, label='5° tolerance')
    axes[0].set_ylabel('Phase error (°)')
    axes[0].set_title(f'Phase tracking error, worst of the three drones (seed {seed})')
    axes[0].legend(fontsize=8)
    axes[0].grid(alpha=0.3)
    for s, ls in [('locked', '-'), ('fixed', '--'), ('raw', ':')]:
        axes[1].plot(t, res[s]['dist'][:, seed].min(axis=1), color=S_COLORS[s], lw=1.1, ls=ls,
                     label=f'{LABELS[s]} (closest pair)')
    axes[1].axhline(D_MIN, color='red', ls='--', lw=1.5, label=f'd_min = {D_MIN:.0f} m')
    axes[1].axhline(ORBIT_RADIUS * np.sqrt(3), color='grey', ls=':', lw=1,
                    label=f'ideal r√3 = {ORBIT_RADIUS * np.sqrt(3):.2f} m')
    axes[1].set_xlabel('Time (s)')
    axes[1].set_ylabel('Closest inter-drone distance (m)')
    axes[1].set_title('Closest pair separation')
    axes[1].legend(fontsize=8)
    axes[1].grid(alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'sync_separation.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_framing(res, out_dir, seed=0):
    fig, ax = plt.subplots(figsize=(12, 4))
    t = res['locked']['t']
    win = int(round(2.0 / DT))
    kern = np.ones(win) / win
    for s in STRATS:
        fleet = res[s]['frame_t'][:, seed].mean(axis=1)
        smooth = np.convolve(fleet, kern, mode='valid')
        ax.plot(t[win // 2:win // 2 + len(smooth)], smooth, color=S_COLORS[s], lw=1.3,
                label=f'{LABELS[s]} (2 s moving average)')
    ax.axhline(0.90, color='green', ls='--', lw=1.2, label='target 0.90')
    ax.set_ylim(0, 1.05)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Fleet framing score')
    ax.set_title(f'Framing: share of the three cameras with the subject within ±15 % (seed {seed})')
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)
    path = os.path.join(out_dir, 'framing_score.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_monte_carlo(res, out_dir):
    fig, axes = plt.subplots(1, 4, figsize=(18, 4.6))
    items = [('phase_mean', 'Mean phase error (°)'), ('phase_ok', 'Time within 5° (%)'),
             ('min_dist', 'Minimum inter-drone distance (m)'), ('frame', 'Fleet framing score')]
    for ax, (key, lab) in zip(axes, items):
        bp = ax.boxplot([res[s][key] for s in STRATS], showmeans=True, patch_artist=True)
        for p, s in zip(bp['boxes'], STRATS):
            p.set_facecolor(S_COLORS[s])
            p.set_alpha(0.4)
        ax.set_xticks([1, 2, 3], ['Locked\nKF', 'Fixed\nKF', 'Locked\nraw'])
        ax.set_ylabel(lab)
        ax.grid(alpha=0.3)
    axes[2].axhline(D_MIN, color='red', ls='--', lw=1)
    axes[3].axhline(0.90, color='green', ls='--', lw=1)
    fig.suptitle(f'S086 Monte Carlo over {N_MC} observation-noise seeds (same noise for every '
                 'strategy; triangle = mean)')
    plt.tight_layout()
    path = os.path.join(out_dir, 'monte_carlo.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def save_animation(res, out_dir, seed=0):
    r = res['locked']
    q, hist = r['q'], r['hist'][:, seed]
    n_frames = int(np.ceil((N_STEPS - 1) / ANIM_STEP)) + 1
    idx = [min(i * ANIM_STEP, N_STEPS - 1) for i in range(n_frames)]
    fig = plt.figure(figsize=(6.5, 6))
    ax = fig.add_subplot(111, projection='3d')

    def update(fi):
        i = idx[fi]
        ax.cla()
        ax.plot(q[:i + 1, 0], q[:i + 1, 1], 0 * q[:i + 1, 0], color='blue', lw=1.3, alpha=0.4)
        ax.scatter(q[i, 0], q[i, 1], 0, color='blue', s=110)
        for k in range(N_DRONES):
            h = hist[max(0, i - 100):i + 1, k]          # last 5 s of the drone's path
            ax.plot(h[:, 0], h[:, 1], h[:, 2], color=D_COLORS[k], lw=1, alpha=0.5)
            ax.scatter(*hist[i, k], color=D_COLORS[k], s=90, marker='^')
            ax.plot([hist[i, k, 0], q[i, 0]], [hist[i, k, 1], q[i, 1]], [hist[i, k, 2], 0],
                    color=D_COLORS[k], ls=':', lw=0.9)
        ax.set_xlim(-18, 18)
        ax.set_ylim(-14, 14)
        ax.set_zlim(0, 8)
        ax.set_xlabel('x (m)')
        ax.set_ylabel('y (m)')
        ax.set_zlabel('z (m)')
        ax.set_title(f'S086 Multi-angle filming (heading-locked, KF)  t = {r["t"][i]:.1f} s')
        ax.view_init(elev=30, azim=-60 + fi * 0.8)

    ani = animation.FuncAnimation(fig, update, frames=len(idx), blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer='pillow', fps=10, dpi=64)
    plt.close(fig)
    print(f'Saved: {path} ({len(idx)} frames)')


# ── Main ──────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    res = run_simulation()
    tt = np.linspace(0, 2 * np.pi / FIG8_OMEGA_CARD, 20001)
    _, v_card = subject(tt, FIG8_OMEGA_CARD)
    tt2 = np.linspace(0, 2 * np.pi / FIG8_OMEGA, 20001)
    _, v_used = subject(tt2)
    sp = np.linalg.norm(v_used, axis=1)
    print('\n=== S086 Cooperative Multi-Angle Filming — Results ===')
    print(f'  Card omega {FIG8_OMEGA_CARD} rad/s -> RMS speed '
          f'{np.sqrt(np.mean(np.sum(v_card ** 2, axis=1))):.2f} m/s; used omega {FIG8_OMEGA:.4f} '
          f'rad/s -> RMS {np.sqrt(np.mean(sp ** 2)):.2f} m/s (min {sp.min():.2f}, max {sp.max():.2f})')
    q, qv = subject(res['locked']['t'])
    hd = np.unwrap(np.arctan2(qv[:, 1], qv[:, 0]))
    hr = np.abs(np.diff(hd)) / DT
    i_hr = int(np.argmax(hr))
    i_tip = int(np.argmax(np.abs(q[:-1, 0])))
    print(f'  Subject heading rate: max {hr.max():.3f} rad/s at (±{abs(q[i_hr, 0]):.1f}, '
          f'±{abs(q[i_hr, 1]):.1f}) m -> set-point speed on the orbit up to '
          f'{ORBIT_RADIUS * hr.max():.2f} m/s from rotation alone (V_MAX {V_MAX:.0f} m/s); '
          f'at the lobe tips (x = ±{abs(q[i_tip, 0]):.1f} m) {hr[i_tip]:.3f} rad/s')
    print(f'  Lap time {2 * np.pi / FIG8_OMEGA:.1f} s; ideal separation r*sqrt(3) = '
          f'{ORBIT_RADIUS * np.sqrt(3):.2f} m')
    print(f'  Monte Carlo: {N_MC} seeds, same observation noise for every strategy (mean ± std)')
    print(f"  {'Strategy':<27}{'mean phase err (°)':>20}{'max (°)':>10}{'within 5° (%)':>16}"
          f"{'pair-sep err (°)':>18}{'min dist (m)':>16}{'framing':>16}{'est RMSE (m)':>14}")
    for s in STRATS:
        r = res[s]
        print(f"  {LABELS[s]:<27}{r['phase_mean'].mean():>12.2f} ± {r['phase_mean'].std():4.2f}"
              f"{r['phase_max'].mean():>10.1f}{r['phase_ok'].mean():>10.1f} ± {r['phase_ok'].std():3.1f}"
              f"{r['sep_mean'].mean():>12.2f} ± {r['sep_mean'].std():3.2f}"
              f"{r['min_dist'].mean():>9.2f} ± {r['min_dist'].std():4.2f}"
              f"{r['frame'].mean():>9.3f} ± {r['frame'].std():4.3f}"
              f"{r['kf_rmse'].mean():>10.3f}")
    for s in STRATS:
        r = res[s]
        print(f"  {LABELS[s]}: after t = {T_LATE:.0f} s mean phase error "
              f"{r['phase_mean_late'].mean():.2f} ± {r['phase_mean_late'].std():.2f}°, "
              f"max {r['phase_max_late'].mean():.1f}° (mean over seeds)")
    for s in STRATS:
        r = res[s]
        print(f"  {LABELS[s]}: min distance over all seeds {r['min_dist'].min():.2f} m; "
              f"seeds with framing >= 0.90: {int(np.sum(r['frame'] >= 0.90))}/{N_MC}; "
              f"seeds with mean phase error <= 3°: {int(np.sum(r['phase_mean'] <= 3.0))}/{N_MC}")
    L, Fx, Rw = res['locked'], res['fixed'], res['raw']
    print(f"  Locked KF vs fixed KF: lower mean phase error in "
          f"{int(np.sum(L['phase_mean'] < Fx['phase_mean']))}/{N_MC} seeds; "
          f"higher framing in {int(np.sum(L['frame'] > Fx['frame']))}/{N_MC} "
          f"(ties {int(np.sum(L['frame'] == Fx['frame']))})")
    print(f"  Locked KF vs locked raw: lower mean phase error in "
          f"{int(np.sum(L['phase_mean'] < Rw['phase_mean']))}/{N_MC}; higher framing in "
          f"{int(np.sum(L['frame'] > Rw['frame']))}/{N_MC}; larger min distance in "
          f"{int(np.sum(L['min_dist'] > Rw['min_dist']))}/{N_MC}")
    pe0 = np.degrees(L['phase_err'][:, 0])
    k_max = int(np.argmax(pe0.max(axis=1)))
    print(f"  Seed 0 heading-locked KF: mean phase err {L['phase_mean'][0]:.2f}°, max {pe0.max():.1f}° "
          f"at t = {L['t'][k_max]:.2f} s, "
          f"framing {L['frame'][0]:.3f}, min dist {L['min_dist'][0]:.2f} m")

    out = os.path.normpath(OUTPUT_DIR)
    os.makedirs(out, exist_ok=True)
    plot_3d_formation(res, out)
    plot_sync_separation(res, out)
    plot_framing(res, out)
    plot_monte_carlo(res, out)
    save_animation(res, out)
