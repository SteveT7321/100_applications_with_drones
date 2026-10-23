"""
S008 Stochastic Pursuit — Kalman Filter Tracking
==================================================
Evader moves under a constant-velocity model with Gaussian acceleration noise.
Pursuer receives noisy position measurements (sigma = 0.3 m) of the evader.

Strategies (same evader trajectory and the same measurement-noise sequence for
every strategy — common random numbers):

  - kf_lead    : Kalman filter position + velocity → aim at the predicted
                 intercept point (uses the KF velocity estimate)
  - kf_pure    : Kalman filter position only → pure pursuit
  - naive      : pure pursuit at the raw noisy measurement
  - oracle_lead: true position + velocity → intercept point (upper bound)
  - oracle_pure: true position → pure pursuit

The evader starts 8 m ahead of the pursuer and crosses the line of sight
(mean velocity +y), so leading the target genuinely shortens the chase.

Monte Carlo: N_MC runs; run i uses evader seed EVADER_SEED0 + i and
measurement seed NOISE_SEED0 + i.  The plotted run is run 0.

Usage:
    conda activate drones
    python src/01_pursuit_evasion/s008_stochastic_pursuit.py
"""

import sys
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from src.base.drone_base import DroneBase

# ── Parameters ───────────────────────────────────────────────
Q_PROCESS   = 0.1    # m²/s³ — KF process noise intensity
SIGMA_MEAS  = 0.3    # m     — measurement noise std (per axis)
ACCEL_STD   = 0.5    # m/s²  — evader random acceleration std (per step, x/y)
V_PURSUER   = 5.0    # m/s
V_EVADER    = 3.0    # m/s (initial / mean speed)
V_EVADER_MAX = 6.0   # m/s — evader speed clamp
R0          = 8.0    # m
CAPTURE_R   = 0.15   # m
DT          = 1 / 48
MAX_TIME    = 25.0
N_MC        = 100
ANIM_TIME   = 6.0    # s — animation shows the first 6 s
EVADER_SEED0 = 1000
NOISE_SEED0  = 0

INIT_PURSUER = np.array([0.0, 0.0, 2.0])
INIT_EVADER  = np.array([R0,  0.0, 2.0])
EVADER_VELO  = np.array([0.0, V_EVADER, 0.0])   # crossing the line of sight

STRATEGIES = ['kf_lead', 'kf_pure', 'naive', 'oracle_lead', 'oracle_pure']
LABELS = {
    'kf_lead': 'KF + lead (intercept)',
    'kf_pure': 'KF + pure pursuit',
    'naive': 'Naive (raw meas.)',
    'oracle_lead': 'Oracle + lead',
    'oracle_pure': 'Oracle + pure pursuit',
}
COLORS = {
    'kf_lead': 'steelblue',
    'kf_pure': 'purple',
    'naive': 'darkorange',
    'oracle_lead': 'mediumseagreen',
    'oracle_pure': 'olive',
}

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '01_pursuit_evasion', 's008_stochastic_pursuit',
)


# ── Kalman Filter ────────────────────────────────────────────

class KalmanFilter3D:
    """Linear Kalman filter for 3-D position+velocity tracking."""

    def __init__(self, dt, q, sigma_meas):
        I3 = np.eye(3)
        Z3 = np.zeros((3, 3))
        self.F = np.block([[I3, dt * I3], [Z3, I3]])
        self.H = np.hstack([I3, Z3])
        self.Q = q * np.block([
            [dt ** 3 / 3 * I3, dt ** 2 / 2 * I3],
            [dt ** 2 / 2 * I3, dt * I3],
        ])
        self.R = sigma_meas ** 2 * I3
        self.x = np.zeros(6)   # [px,py,pz, vx,vy,vz]
        self.P = np.eye(6)

    def init_state(self, pos, vel):
        self.x[:3] = pos
        self.x[3:] = vel

    def step(self, z):
        # Predict
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        # Update
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ (z - self.H @ self.x)
        self.P = (np.eye(6) - K @ self.H) @ self.P
        return self.x.copy(), self.P[:3, :3].copy()


# ── Guidance ─────────────────────────────────────────────────

def intercept_point(p_p, p_e, v_e, v_p=V_PURSUER):
    """
    Point where a pursuer at p_p flying straight at speed v_p meets an evader
    at p_e moving with constant v_e:  |p_e + v_e t - p_p| = v_p t.
    Falls back to p_e if no positive solution exists (|v_e| >= v_p).
    """
    r = p_e - p_p
    a = v_e @ v_e - v_p ** 2
    b = 2.0 * (r @ v_e)
    c = r @ r
    if a >= 0:
        return p_e
    t = (-b - np.sqrt(b * b - 4 * a * c)) / (2 * a)   # the positive root
    return p_e + v_e * max(t, 0.0)


# ── Simulation ───────────────────────────────────────────────

def generate_evader_trajectory(seed):
    """Evader positions and velocities with random acceleration (x/y only)."""
    rng = np.random.default_rng(seed)
    pos = INIT_EVADER.astype(float).copy()
    vel = EVADER_VELO.astype(float).copy()
    traj, vels = [pos.copy()], [vel.copy()]
    for _ in range(int(MAX_TIME / DT)):
        acc = rng.normal(0, ACCEL_STD, 3)
        acc[2] = 0.0
        vel = vel + acc * DT
        spd = np.linalg.norm(vel)
        if spd > V_EVADER_MAX:
            vel = vel / spd * V_EVADER_MAX
        pos = pos + vel * DT
        traj.append(pos.copy())
        vels.append(vel.copy())
    return np.array(traj), np.array(vels)


def run_simulation(strategy, evader_traj, evader_vel, noise_seed):
    """
    Same evader trajectory and same measurement-noise sequence for every
    strategy (the noise generator is re-seeded per run and one sample is
    drawn per step regardless of strategy).

    Returns dict: p_traj, est_pos, est_vel, meas, pos_err, vel_err, cov,
                  times, captured, cap_time
    """
    rng = np.random.default_rng(noise_seed)
    pursuer = DroneBase(INIT_PURSUER.copy(), max_speed=V_PURSUER, dt=DT)
    kf = KalmanFilter3D(DT, Q_PROCESS, SIGMA_MEAS)
    kf.init_state(evader_traj[0], EVADER_VELO.copy())

    p_list = [pursuer.pos.copy()]
    est_list, estv_list = [evader_traj[0].copy()], [EVADER_VELO.copy()]
    meas_list = [evader_traj[0].copy()]
    perr, verr = [0.0], [0.0]
    cov_list = [np.eye(3)]
    time_list = [0.0]
    captured, cap_time = False, None

    for step in range(1, len(evader_traj)):
        t = step * DT
        # The evader moves from step-1 to step while the pursuer flies the
        # command computed from the measurement of the evader at step-1.
        p_e_prev = evader_traj[step - 1]
        v_e_prev = evader_vel[step - 1]
        meas = p_e_prev + rng.normal(0, SIGMA_MEAS, 3)
        x_kf, cov = kf.step(meas)

        if strategy in ('kf_lead', 'kf_pure'):
            est_p, est_v = x_kf[:3], x_kf[3:]
        elif strategy == 'naive':
            est_p, est_v = meas, np.full(3, np.nan)
        else:
            est_p, est_v = p_e_prev, v_e_prev

        if strategy in ('kf_lead', 'oracle_lead'):
            aim = intercept_point(pursuer.pos, est_p, est_v)
        else:
            aim = est_p
        diff = aim - pursuer.pos
        n = np.linalg.norm(diff)
        pursuer.step(V_PURSUER * diff / n if n > 1e-8 else np.zeros(3))

        p_list.append(pursuer.pos.copy())
        est_list.append(est_p.copy())
        estv_list.append(est_v.copy())
        meas_list.append(meas.copy())
        perr.append(np.linalg.norm(est_p - p_e_prev))
        verr.append(np.linalg.norm(est_v - v_e_prev))
        cov_list.append(cov)
        time_list.append(t)

        if np.linalg.norm(pursuer.pos - evader_traj[step]) < CAPTURE_R:
            captured, cap_time = True, t
            break

    return dict(
        p_traj=np.array(p_list), est_pos=np.array(est_list),
        est_vel=np.array(estv_list), meas=np.array(meas_list),
        pos_err=np.array(perr), vel_err=np.array(verr),
        cov=np.array(cov_list), times=np.array(time_list),
        captured=captured, cap_time=cap_time,
    )


def run_monte_carlo():
    caps = {s: [] for s in STRATEGIES}
    perr = {s: [] for s in STRATEGIES}
    for i in range(N_MC):
        et, ev = generate_evader_trajectory(EVADER_SEED0 + i)
        for s in STRATEGIES:
            r = run_simulation(s, et, ev, NOISE_SEED0 + i)
            caps[s].append(r['cap_time'] if r['captured'] else np.nan)
            perr[s].append(np.mean(r['pos_err'][1:]))
    return ({s: np.array(v) for s, v in caps.items()},
            {s: np.array(v) for s, v in perr.items()})


# ── Plots ────────────────────────────────────────────────────

def plot_trajectories(results, evader_traj, out_dir):
    fig, axes = plt.subplots(1, len(STRATEGIES), figsize=(22, 5.5))
    n_show = int(round(ANIM_TIME / DT)) + 1
    for ax, s in zip(axes, STRATEGIES):
        r = results[s]
        n = min(len(r['p_traj']), n_show)
        et = evader_traj[:n]
        p, est, meas = r['p_traj'][:n], r['est_pos'][:n], r['meas'][:n]
        if s in ('kf_lead', 'kf_pure', 'naive'):
            ax.scatter(meas[1::6, 0], meas[1::6, 1], color='lightcoral',
                       s=8, alpha=0.6, label='Meas (noisy)')
        ax.plot(et[:, 0], et[:, 1], color='royalblue', linewidth=1.8, label='Evader (true)')
        if s in ('kf_lead', 'kf_pure'):
            ax.plot(est[:, 0], est[:, 1], color=COLORS[s],
                    linestyle='--', linewidth=1.2, label='KF estimate')
        ax.plot(p[:, 0], p[:, 1], color='red', linewidth=1.8, label='Pursuer')
        ax.scatter(*p[0, :2], color='red', s=50)
        ax.scatter(*et[0, :2], color='blue', s=50)
        if r['captured'] and r['cap_time'] <= ANIM_TIME:
            ax.scatter(*p[-1, :2], color='black', s=90, marker='X',
                       label=f'Captured {r["cap_time"]:.2f}s')
            status = f'Captured {r["cap_time"]:.2f} s'
        elif r['captured']:
            status = f'Captured {r["cap_time"]:.2f} s (after the {ANIM_TIME:.0f} s shown)'
        else:
            status = f'Not captured in {MAX_TIME:.0f} s (first {ANIM_TIME:.0f} s shown)'
        ax.set_aspect('equal'); ax.grid(True, alpha=0.3)
        ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
        ax.set_title(f'{LABELS[s]}\n{status}', fontsize=9)
        ax.legend(fontsize=6, loc='upper left')
    fig.suptitle(f'S008 Stochastic Pursuit — XY Trajectories (MC run 0, first '
                 f'{ANIM_TIME:.0f} s)', fontsize=11)
    plt.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'trajectories_xy.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_estimation_error(results, out_dir):
    """Position error (KF vs naive) and KF velocity error, MC run 0."""
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 4.5))
    for s in ('kf_lead', 'naive'):
        r = results[s]
        lab = 'Kalman filter' if s == 'kf_lead' else 'Naive (raw measurement)'
        ax1.plot(r['times'][1:], r['pos_err'][1:], color=COLORS[s], linewidth=1.4, label=lab)
    ax1.axhline(SIGMA_MEAS * np.sqrt(3), color='grey', linestyle=':',
                label=f'RMS of raw noise = σ√3 = {SIGMA_MEAS*np.sqrt(3):.2f} m')
    ax1.set_xlabel('Time (s)'); ax1.set_ylabel('Position error (m)')
    ax1.set_title('Evader position estimation error', fontsize=10)
    ax1.legend(fontsize=8); ax1.grid(True, alpha=0.3)

    r = results['kf_lead']
    ax2.plot(r['times'][1:], r['vel_err'][1:], color=COLORS['kf_lead'], linewidth=1.4)
    ax2.set_xlabel('Time (s)'); ax2.set_ylabel('Velocity error (m/s)')
    ax2.set_title('KF evader velocity estimation error (used by the lead law)', fontsize=10)
    ax2.grid(True, alpha=0.3)
    fig.suptitle('S008 Stochastic Pursuit — Estimation Error (MC run 0, KF+lead run)',
                 fontsize=11)
    plt.tight_layout()
    path = os.path.join(out_dir, 'estimation_error.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def plot_covariance_ellipses(results, evader_traj, out_dir):
    """KF 2σ position-uncertainty ellipses sampled every 0.5 s (KF+lead run)."""
    r = results['kf_lead']
    n = len(r['p_traj'])
    et = evader_traj[:n]
    fig, ax = plt.subplots(figsize=(8, 7))
    ax.plot(et[:, 0], et[:, 1], color='royalblue', linewidth=1.8, label='Evader (true)')
    ax.plot(r['est_pos'][:, 0], r['est_pos'][:, 1], color='steelblue',
            linewidth=1.4, linestyle='--', label='KF estimate')
    ax.plot(r['p_traj'][:, 0], r['p_traj'][:, 1], color='red', linewidth=1.8,
            label='Pursuer (KF + lead)')
    sample_step = int(round(0.5 / DT))
    for k in range(0, n, sample_step):
        vals, vecs = np.linalg.eigh(r['cov'][k][:2, :2])
        vals = np.maximum(vals, 0)
        w, h = 2 * 2 * np.sqrt(vals)          # 2σ full width / height
        ang = np.degrees(np.arctan2(vecs[1, 0], vecs[0, 0]))
        ax.add_patch(Ellipse(xy=r['est_pos'][k, :2], width=w, height=h, angle=ang,
                             edgecolor='steelblue', facecolor='none', linewidth=1.0))
    if r['captured']:
        ax.scatter(*r['p_traj'][-1, :2], color='black', s=100, marker='X',
                   label=f'Captured {r["cap_time"]:.2f}s')
    ax.set_aspect('equal'); ax.grid(True, alpha=0.3)
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
    ax.set_title('S008 — Kalman filter 2σ position ellipses every 0.5 s\n'
                 '(first ellipse = initial P = 1 m² per axis)', fontsize=10)
    ax.legend(fontsize=8)
    plt.tight_layout()
    path = os.path.join(out_dir, 'covariance_ellipses.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def plot_capture_bar(caps, out_dir):
    fig, ax = plt.subplots(figsize=(10, 5))
    xs = np.arange(len(STRATEGIES))
    for x, s in zip(xs, STRATEGIES):
        c = caps[s][~np.isnan(caps[s])]
        m = c.mean()
        ax.bar(x, m, color=COLORS[s], edgecolor='black', width=0.55)
        ax.errorbar(x, m, yerr=[[m - np.percentile(c, 5)], [np.percentile(c, 95) - m]],
                    color='black', capsize=6)
        ax.text(x, np.percentile(c, 95) + 0.05,
                f'{m:.2f} s\n({len(c)}/{N_MC} captured)', ha='center', va='bottom',
                fontsize=9)
    ax.set_xticks(xs)
    ax.set_xticklabels([LABELS[s] for s in STRATEGIES], fontsize=8)
    ax.set_ylabel('Capture time (s)')
    ax.set_title(f'S008 — Mean capture time over the captured runs of {N_MC} '
                 '(whiskers = 5th–95th percentile of captured runs)', fontsize=10)
    top = max(np.nanpercentile(caps[s], 95) for s in STRATEGIES)
    ax.set_ylim(0, top * 1.25)
    ax.grid(axis='y', alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'capture_bar.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def save_animation(results, evader_traj, out_dir):
    import matplotlib.animation as animation
    shown = ['kf_lead', 'kf_pure', 'naive']
    # Show only the first ANIM_TIME seconds (naive may run to MAX_TIME)
    max_len = min(max(len(results[s]['p_traj']) for s in shown),
                  int(round(ANIM_TIME / DT)) + 1)
    step = max(1, int(np.ceil(max_len / 140)))
    n_frames = (max_len - 1) // step + 2

    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    pts = np.vstack([np.vstack([results[s]['p_traj'][:max_len],
                                evader_traj[:min(max_len, len(results[s]['p_traj']))]])
                     for s in shown])
    lo, hi = pts.min(axis=0)[:2] - 1.0, pts.max(axis=0)[:2] + 1.0
    arts = []
    for ax, s in zip(axes, shown):
        ax.set_xlim(lo[0], hi[0]); ax.set_ylim(lo[1], hi[1])
        ax.set_aspect('equal'); ax.grid(True, alpha=0.3)
        ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
        lp, = ax.plot([], [], color='red', linewidth=1.6)
        le, = ax.plot([], [], color='royalblue', linewidth=1.6)
        dp, = ax.plot([], [], 'r^', markersize=8)
        de, = ax.plot([], [], 'bs', markersize=8)
        dx, = ax.plot([], [], 'x', color=COLORS[s], markersize=9, markeredgewidth=2)
        ti = ax.set_title(LABELS[s], fontsize=9)
        arts.append((lp, le, dp, de, dx, ti))

    def update(i):
        out = []
        for s, (lp, le, dp, de, dx, ti) in zip(shown, arts):
            r = results[s]
            n = len(r['p_traj'])
            si = min(i * step, n - 1, max_len - 1)
            et = evader_traj[:n]
            lp.set_data(r['p_traj'][:si + 1, 0], r['p_traj'][:si + 1, 1])
            le.set_data(et[:si + 1, 0], et[:si + 1, 1])
            dp.set_data([r['p_traj'][si, 0]], [r['p_traj'][si, 1]])
            de.set_data([et[si, 0]], [et[si, 1]])
            dx.set_data([r['est_pos'][si, 0]], [r['est_pos'][si, 1]])
            if si == n - 1 and r['captured']:
                ti.set_text(f'{LABELS[s]}\nCaptured {r["cap_time"]:.2f} s')
            else:
                ti.set_text(f'{LABELS[s]}\nt={r["times"][si]:.2f}s  '
                            f'est_err={r["pos_err"][si]:.2f}m')
            out += [lp, le, dp, de, dx, ti]
        return out

    ani = animation.FuncAnimation(fig, update, frames=n_frames, interval=60, blit=False)
    fig.suptitle('S008 Stochastic Pursuit (MC run 0) — red ▲ pursuer, blue ■ evader, '
                 '× = evader estimate used for guidance', fontsize=10)
    plt.tight_layout()
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer='pillow', fps=16, dpi=80)
    plt.close()
    print(f'Saved: {path}  ({n_frames} frames, every {step} steps)')


# ── Main ─────────────────────────────────────────────────────

if __name__ == '__main__':
    evader_traj, evader_vel = generate_evader_trajectory(EVADER_SEED0)
    results = {}
    print('MC run 0 (plotted):')
    for s in STRATEGIES:
        r = run_simulation(s, evader_traj, evader_vel, NOISE_SEED0)
        results[s] = r
        status = f'captured @ {r["cap_time"]:.3f} s' if r['captured'] else 'timeout'
        print(f'  [{s:<11}] {status} | mean pos err {np.mean(r["pos_err"][1:]):.3f} m'
              f' | path {np.sum(np.linalg.norm(np.diff(r["p_traj"], axis=0), axis=1)):.2f} m')
    rn = results['naive']
    rng_n = np.linalg.norm(rn['p_traj'][1:] - evader_traj[1:len(rn['p_traj'])], axis=1)
    late = rn['times'][1:] >= 3.0
    print(f'  naive range to evader for t >= 3 s: median {np.median(rng_n[late]):.3f} m, '
          f'min {rng_n[late].min():.3f} m, 95th pct {np.percentile(rng_n[late], 95):.3f} m')
    rk = results['kf_lead']
    print(f'  KF mean velocity error (kf_lead run): {np.mean(rk["vel_err"][1:]):.3f} m/s')
    same = np.allclose(results['kf_lead']['meas'][:20], results['naive']['meas'][:20])
    print(f'  same measurement noise for all strategies (first 20 steps): {same}')

    caps, perr = run_monte_carlo()
    print(f'\nMonte Carlo ({N_MC} runs, common evader trajectory + noise per run):')
    for s in STRATEGIES:
        c = caps[s]
        print(f'  [{s:<11}] captured {np.sum(~np.isnan(c))}/{N_MC} | cap time mean '
              f'{np.nanmean(c):.3f} s, std {np.nanstd(c):.3f}, min {np.nanmin(c):.3f}, '
              f'max {np.nanmax(c):.3f} | mean pos err {np.mean(perr[s]):.3f} m')
    for a, b in [('kf_lead', 'kf_pure'), ('kf_lead', 'naive'), ('kf_lead', 'oracle_lead'),
                 ('kf_pure', 'naive'), ('oracle_lead', 'oracle_pure')]:
        ca = np.where(np.isnan(caps[a]), np.inf, caps[a])   # timeout = never
        cb = np.where(np.isnan(caps[b]), np.inf, caps[b])
        both = np.isfinite(ca) & np.isfinite(cb)
        print(f'  paired {a} vs {b}: {a} faster in {int(np.sum(ca < cb))}/{N_MC} runs, '
              f'mean difference {np.mean(ca[both] - cb[both]):+.3f} s over the '
              f'{int(both.sum())} runs where both capture')

    out_dir = os.path.normpath(OUTPUT_DIR)
    plot_trajectories(results, evader_traj, out_dir)
    plot_estimation_error(results, out_dir)
    plot_covariance_ellipses(results, evader_traj, out_dir)
    plot_capture_bar(caps, out_dir)
    save_animation(results, evader_traj, out_dir)
