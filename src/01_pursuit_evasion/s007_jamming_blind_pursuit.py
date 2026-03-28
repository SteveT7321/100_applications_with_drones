"""
S007 Jamming & Blind Pursuit
==============================
Pursuer's GPS is periodically jammed.  During blackout it must use dead
reckoning (last known velocity + IMU drift noise) to estimate its own position.

Comparison:
  - dead_reckoning : keep moving at last known velocity + Gaussian drift
  - freeze         : stop during jam, resume when GPS returns
  - baseline       : perfect GPS, no jamming

Jam schedule: jammed during [n·T_period, n·T_period + T_jam] for integer n≥0.

Usage:
    conda activate drones
    python src/01_pursuit_evasion/s007_jamming_blind_pursuit.py
"""

import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from src.base.drone_base import DroneBase

# ── Scenario parameters ─────────────────────────────────────
JAM_PERIOD   = 3.0    # s — time between jam onsets
JAM_DURATION = 1.0    # s — jam window length (33% duty cycle)
SIGMA_DRIFT  = 0.05   # m/s — IMU drift noise std (per-axis)
V_PURSUER    = 5.0    # m/s
V_EVADER     = 3.0    # m/s
R0           = 6.0    # m — initial separation
CAPTURE_R    = 0.15   # m
DT           = 1 / 48
MAX_TIME     = 25.0

INIT_PURSUER = np.array([0.0, 0.0, 2.0])
INIT_EVADER  = np.array([R0,  0.0, 2.0])
EVADER_DIR   = np.array([1.0, 0.0, 0.0])

STRATEGIES = ['baseline', 'dead_reckoning', 'freeze']

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '01_pursuit_evasion', 's007_jamming_blind_pursuit',
)

RNG = np.random.default_rng(42)


# ── Helpers ──────────────────────────────────────────────────

def is_jammed(t):
    return (t % JAM_PERIOD) < JAM_DURATION


# ── Simulation ───────────────────────────────────────────────

def run_simulation(strategy):
    """
    strategy: 'baseline' | 'dead_reckoning' | 'freeze'

    Returns:
        p_traj       (N,3)
        e_traj       (N,3)
        est_traj     (N,3)  — pursuer's self-position estimate
        est_err      (N,)   — |true - estimated|
        jam_log      (N,)   — 1 if jammed that step
        times        (N,)
        captured     bool
        cap_time     float or None
    """
    pursuer = DroneBase(INIT_PURSUER.copy(), max_speed=V_PURSUER, dt=DT)
    evader  = DroneBase(INIT_EVADER.copy(),  max_speed=V_EVADER,  dt=DT)

    pos_estimate = pursuer.pos.copy()
    v_last       = np.zeros(3)

    p_traj_list  = [pursuer.pos.copy()]
    e_traj_list  = [evader.pos.copy()]
    est_list     = [pos_estimate.copy()]
    err_list     = [0.0]
    jam_list     = [0]
    time_list    = [0.0]

    captured = False
    cap_time = None
    max_steps = int(MAX_TIME / DT)

    for step in range(1, max_steps + 1):
        t = step * DT
        jammed = is_jammed(t) and strategy != 'baseline'

        # ── Update position estimate ──
        if not jammed:
            pos_estimate = pursuer.pos.copy()   # GPS fix
            v_last       = pursuer.vel.copy()
        else:
            if strategy == 'dead_reckoning':
                drift = RNG.normal(0, SIGMA_DRIFT * np.sqrt(DT), 3)
                pos_estimate = pos_estimate + v_last * DT + drift
            # freeze: pos_estimate unchanged

        # ── Pursuer command ──
        if jammed and strategy == 'freeze':
            cmd = np.zeros(3)
        else:
            diff = evader.pos - pos_estimate
            n    = np.linalg.norm(diff)
            cmd  = V_PURSUER * diff / n if n > 1e-8 else np.zeros(3)

        pursuer.step(cmd)

        # ── Evader straight flight ──
        evader.step(EVADER_DIR * V_EVADER)

        # ── Capture check ──
        if np.linalg.norm(pursuer.pos - evader.pos) < CAPTURE_R:
            captured = True
            cap_time = t
            p_traj_list.append(pursuer.pos.copy())
            e_traj_list.append(evader.pos.copy())
            est_list.append(pos_estimate.copy())
            err_list.append(np.linalg.norm(pursuer.pos - pos_estimate))
            jam_list.append(1 if jammed else 0)
            time_list.append(t)
            break

        p_traj_list.append(pursuer.pos.copy())
        e_traj_list.append(evader.pos.copy())
        est_list.append(pos_estimate.copy())
        err_list.append(np.linalg.norm(pursuer.pos - pos_estimate))
        jam_list.append(1 if jammed else 0)
        time_list.append(t)

    return (
        np.array(p_traj_list),
        np.array(e_traj_list),
        np.array(est_list),
        np.array(err_list),
        np.array(jam_list),
        np.array(time_list),
        captured,
        cap_time,
    )


# ── Plots ────────────────────────────────────────────────────

def plot_trajectories_3d(results, out_dir):
    colors   = ['mediumseagreen', 'steelblue', 'darkorange']
    labels   = ['Baseline (no jam)', 'Dead Reckoning', 'Freeze on Jam']
    fig = plt.figure(figsize=(16, 5))

    for idx, (res, color, label) in enumerate(zip(results, colors, labels)):
        p_traj, e_traj, est_traj, est_err, jam_log, times, captured, cap_t = res

        ax = fig.add_subplot(1, 3, idx + 1, projection='3d')
        ax.view_init(elev=25, azim=-55)

        ax.plot(p_traj[:, 0], p_traj[:, 1], p_traj[:, 2],
                color=color, linewidth=1.8, label='Pursuer (true)')
        ax.plot(est_traj[:, 0], est_traj[:, 1], est_traj[:, 2],
                color=color, linewidth=1.0, linestyle=':', alpha=0.6,
                label='Pursuer (estimate)')
        ax.plot(e_traj[:, 0], e_traj[:, 1], e_traj[:, 2],
                color='royalblue', linewidth=1.8, label='Evader')

        ax.scatter(*p_traj[0], color=color, s=50, marker='o')
        ax.scatter(*e_traj[0], color='blue', s=50, marker='o')
        if captured:
            ax.scatter(*p_traj[-1], color='black', s=100, marker='X',
                       label=f'Captured {cap_t:.2f}s')
            status = f'Captured {cap_t:.2f} s'
        else:
            status = 'Timeout'

        ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
        ax.set_title(f'{label}\n{status}', fontsize=9)
        ax.legend(fontsize=7, loc='upper left')

    fig.suptitle('S007 Jamming & Blind Pursuit — 3D Trajectories', fontsize=11)
    plt.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'trajectories_3d.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_estimation_error(results, out_dir):
    colors = ['mediumseagreen', 'steelblue', 'darkorange']
    labels = ['Baseline (no jam)', 'Dead Reckoning', 'Freeze on Jam']

    fig, ax = plt.subplots(figsize=(12, 5))
    for res, color, label in zip(results, colors, labels):
        p_traj, e_traj, est_traj, est_err, jam_log, times, captured, cap_t = res
        ax.plot(times, est_err, color=color, linewidth=1.8, label=label)
        if captured:
            ax.axvline(cap_t, color=color, linestyle='--', linewidth=1.0, alpha=0.7)

    # Shade jam windows for reference
    res0 = results[1]  # dead_reckoning has jam windows
    jam_log = res0[4]; times0 = res0[5]
    in_jam = False
    for k in range(len(jam_log)):
        if jam_log[k] and not in_jam:
            j_start = times0[k]; in_jam = True
        elif not jam_log[k] and in_jam:
            ax.axvspan(j_start, times0[k], color='salmon', alpha=0.15)
            in_jam = False
    if in_jam:
        ax.axvspan(j_start, times0[-1], color='salmon', alpha=0.15)

    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Position Estimation Error (m)')
    ax.set_title('S007 Jamming — Position Estimation Error vs Time\n'
                 '(salmon bands = jammed periods)', fontsize=10)
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'estimation_error.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def plot_capture_bar(results, out_dir):
    labels     = ['Baseline\n(no jam)', 'Dead\nReckoning', 'Freeze\non Jam']
    colors     = ['mediumseagreen', 'steelblue', 'darkorange']
    cap_times  = []
    bar_colors = []
    anns       = []

    for res, color in zip(results, colors):
        _, _, _, _, _, times, captured, cap_t = res
        if captured:
            cap_times.append(cap_t)
            bar_colors.append(color)
            anns.append(f'{cap_t:.2f} s')
        else:
            cap_times.append(MAX_TIME)
            bar_colors.append('tomato')
            anns.append('Timeout')

    fig, ax = plt.subplots(figsize=(8, 5))
    bars = ax.bar(labels, cap_times, color=bar_colors, edgecolor='black', width=0.4)
    for bar, ann in zip(bars, anns):
        ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.3,
                ann, ha='center', va='bottom', fontsize=10, fontweight='bold')

    ax.set_ylabel('Capture Time (s)')
    ax.set_title('S007 Jamming — Capture Time by Strategy', fontsize=10)
    ax.set_ylim(0, MAX_TIME * 1.2)
    ax.grid(axis='y', alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'capture_bar.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def save_animation(results, out_dir):
    import matplotlib.animation as animation

    colors = ['mediumseagreen', 'steelblue', 'darkorange']
    labels = ['Baseline', 'Dead Reckoning', 'Freeze']

    max_len = max(len(res[0]) for res in results)
    step = 3

    fig, axes = plt.subplots(1, 3, figsize=(15, 5))

    lines_p, lines_e, dots_p, dots_e, dots_est, titles = [], [], [], [], [], []
    for ax, color, label, res in zip(axes, colors, labels, results):
        p_traj, e_traj = res[0], res[1]
        all_pts = np.vstack([p_traj, e_traj, res[2]])
        lo = all_pts.min(axis=0)[:2] - 1.0
        hi = all_pts.max(axis=0)[:2] + 1.0

        ax.set_xlim(lo[0], hi[0]); ax.set_ylim(lo[1], hi[1])
        ax.set_aspect('equal'); ax.grid(True, alpha=0.3)
        ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
        ax.scatter(*p_traj[0, :2], color=color, s=60, zorder=5)
        ax.scatter(*e_traj[0, :2], color='blue', s=60, zorder=5)

        lp, = ax.plot([], [], color=color,      linewidth=1.8, alpha=0.8)
        le, = ax.plot([], [], color='royalblue', linewidth=1.8, alpha=0.8)
        dp, = ax.plot([], [], '^', color=color, markersize=9, zorder=6)
        de, = ax.plot([], [], 'bs', markersize=9, zorder=6)
        dest, = ax.plot([], [], 'x', color=color, markersize=8, zorder=7,
                        markeredgewidth=2, alpha=0.7)
        ti = ax.set_title(label, fontsize=9)

        lines_p.append(lp); lines_e.append(le)
        dots_p.append(dp); dots_e.append(de)
        dots_est.append(dest); titles.append(ti)

    n_frames = max_len // step

    def update(i):
        artists = []
        for idx, res in enumerate(results):
            p_traj, e_traj, est_traj, est_err, jam_log, times, captured, cap_t = res
            si = min(i * step, len(p_traj) - 1)
            done = (i * step >= len(p_traj) - 1)

            px = p_traj[:si+1, 0]; py = p_traj[:si+1, 1]
            ex = e_traj[:si+1, 0]; ey = e_traj[:si+1, 1]
            lines_p[idx].set_data(px, py)
            lines_e[idx].set_data(ex, ey)
            dots_p[idx].set_data([float(px[-1])], [float(py[-1])])
            dots_e[idx].set_data([float(ex[-1])], [float(ey[-1])])
            dots_est[idx].set_data([float(est_traj[si, 0])], [float(est_traj[si, 1])])

            t     = times[si]
            jammed = bool(jam_log[si])
            err   = est_err[si]
            jam_str = ' [JAM]' if jammed else ''

            if done:
                if captured:
                    titles[idx].set_text(f'{labels[idx]}  t={t:.1f}s ✓ Captured [DONE]')
                else:
                    titles[idx].set_text(f'{labels[idx]}  t={t:.1f}s  Timeout [DONE]')
            else:
                titles[idx].set_text(
                    f'{labels[idx]}  t={t:.1f}s{jam_str}\nerr={err:.3f}m'
                )
            artists += [lines_p[idx], lines_e[idx], dots_p[idx],
                        dots_e[idx], dots_est[idx], titles[idx]]
        return artists

    ani = animation.FuncAnimation(fig, update, frames=n_frames,
                                  interval=60, blit=True)
    fig.suptitle('S007 Jamming & Blind Pursuit\n'
                 '(triangle=pursuer true, ×=pursuer estimate, square=evader)',
                 fontsize=10)
    plt.tight_layout()

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer='pillow', fps=16, dpi=100)
    plt.close()
    print(f'Saved: {path}')


# ── Main ─────────────────────────────────────────────────────

if __name__ == '__main__':
    results = []
    for strategy in STRATEGIES:
        res = run_simulation(strategy)
        _, _, _, _, _, times, captured, cap_t = res
        status = f'captured @ {cap_t:.2f} s' if captured else f'timeout @ {times[-1]:.2f} s'
        print(f'[{strategy:<16}]  {status}')
        results.append(res)

    out_dir = os.path.normpath(OUTPUT_DIR)
    plot_trajectories_3d(results, out_dir)
    plot_estimation_error(results, out_dir)
    plot_capture_bar(results, out_dir)
    save_animation(results, out_dir)
