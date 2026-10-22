"""
S007 Jamming & Blind Pursuit
==============================
Pursuer's GPS is periodically jammed.  During a blackout it must estimate its
own position without GPS.  The evader weaves (sinusoidal heading), so the
pursuer has to keep turning: an estimate that assumes the old velocity drifts
away from the truth and bends the pursuit direction.

The pursuer always knows the evader's true position (e.g. from an onboard
tracker); only its own position is uncertain during jamming.

Strategies:
  - baseline       : perfect GPS, no jamming
  - dr_last        : dead reckoning with the last GPS velocity (card model):
                     p_hat += v_last·dt + drift, while the drone keeps turning
  - dr_imu         : dead reckoning with the drone's own commanded velocity
                     (IMU-style): p_hat += v_cmd·dt + drift
  - freeze         : hover during the jam, resume when GPS returns

Drift: per step N(0, sigma²·dt·I) → after τ s of jamming the noise part of
the error has std sigma·sqrt(τ) per axis (card model).

Jam schedule: jammed during [JAM_OFFSET + n·T_period, JAM_OFFSET + n·T_period
+ T_jam) for integer n ≥ 0.  The offset makes the first jam start after the
drone is already moving, so v_last is defined.

Dead-reckoning cases are run N_MC times with seeds 0..N_MC-1; dr_last and
dr_imu use the same seed per run (common random numbers).  The plotted run is
seed PLOT_SEED.  A sweep over the first-jam onset (SWEEP_OFFSETS, N_SWEEP_SEEDS
seeds each) shows how the result depends on when the jam hits.

Usage:
    conda activate drones
    python src/01_pursuit_evasion/s007_jamming_blind_pursuit.py
"""

import sys
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from src.base.drone_base import DroneBase

# ── Scenario parameters ─────────────────────────────────────
JAM_PERIOD    = 3.0    # s — time between jam onsets
JAM_DURATION  = 1.0    # s — jam window length (33% duty cycle)
JAM_OFFSET    = 0.5    # s — first jam onset (drone already moving)
SIGMA_DRIFT   = 0.05   # m/√s — IMU drift noise intensity (per axis)
V_PURSUER     = 5.0    # m/s
V_EVADER      = 3.0    # m/s
R0            = 6.0    # m — initial separation
WEAVE_AMP     = np.radians(60.0)   # rad — evader heading amplitude
WEAVE_PERIOD  = 2.0    # s — evader weave period
CAPTURE_R     = 0.15   # m
DT            = 1 / 48
MAX_TIME      = 25.0   # s
N_MC          = 100    # Monte-Carlo runs (drift noise seeds 0..N_MC-1)
PLOT_SEED     = 0      # seed of the run shown in the figures
SWEEP_OFFSETS = np.arange(0.25, 3.01, 0.25)   # s — first-jam onsets swept
N_SWEEP_SEEDS = 20     # seeds per DR case per offset in the sweep

INIT_PURSUER = np.array([0.0, 0.0, 2.0])
INIT_EVADER  = np.array([R0,  0.0, 2.0])

STRATEGIES = ['baseline', 'dr_last', 'dr_imu', 'freeze']
LABELS = {
    'baseline': 'Baseline (no jam)',
    'dr_last':  'DR: last GPS velocity',
    'dr_imu':   'DR: own commanded velocity',
    'freeze':   'Freeze on jam',
}
COLORS = {
    'baseline': 'mediumseagreen',
    'dr_last':  'darkorange',
    'dr_imu':   'purple',
    'freeze':   'saddlebrown',
}

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '01_pursuit_evasion', 's007_jamming_blind_pursuit',
)


# ── Helpers ──────────────────────────────────────────────────

def is_jammed(t, offset=JAM_OFFSET):
    return t >= offset and ((t - offset) % JAM_PERIOD) < JAM_DURATION


def evader_velocity(t):
    """Weaving escape: heading = WEAVE_AMP·sin(2πt/T_w) around +x."""
    th = WEAVE_AMP * np.sin(2 * np.pi * t / WEAVE_PERIOD)
    return V_EVADER * np.array([np.cos(th), np.sin(th), 0.0])


# ── Simulation ───────────────────────────────────────────────

def run_simulation(strategy, seed=PLOT_SEED, jam_offset=JAM_OFFSET):
    """
    strategy:   'baseline' | 'dr_last' | 'dr_imu' | 'freeze'
    seed:       seed of the drift-noise generator (only dr_* draw from it)
    jam_offset: time of the first jam onset

    The estimate at each logged time is the one the pursuer holds at that
    time: the GPS fix when not jammed, the dead-reckoned value when jammed.

    Returns dict with
        p_traj (N,3), e_traj (N,3), est_traj (N,3), est_err (N,),
        jam_log (N,), times (N,), captured, cap_time
    """
    rng = np.random.default_rng(seed)
    pursuer = DroneBase(INIT_PURSUER.copy(), max_speed=V_PURSUER, dt=DT)
    evader  = DroneBase(INIT_EVADER.copy(),  max_speed=V_EVADER,  dt=DT)

    pos_estimate = pursuer.pos.copy()
    v_last = np.zeros(3)          # velocity over the last GPS-covered step
    jammed = False                # t = 0 is never jammed (offset > 0)

    p_list, e_list, est_list = [pursuer.pos.copy()], [evader.pos.copy()], [pos_estimate.copy()]
    err_list, jam_list, time_list = [0.0], [0], [0.0]

    captured, cap_time = False, None
    max_steps = int(MAX_TIME / DT)

    for step in range(max_steps):
        t = step * DT
        t_next = (step + 1) * DT
        jammed_next = is_jammed(t_next, jam_offset) and strategy != 'baseline'

        # ── Pursuer command (pure pursuit from the current estimate) ──
        if jammed and strategy == 'freeze':
            cmd = np.zeros(3)
        else:
            diff = evader.pos - pos_estimate
            n = np.linalg.norm(diff)
            cmd = V_PURSUER * diff / n if n > 1e-8 else np.zeros(3)

        pursuer.step(cmd)
        evader.step(evader_velocity(t))

        # ── Estimate at t_next ──
        if not jammed_next:
            pos_estimate = pursuer.pos.copy()       # GPS fix
            v_last = pursuer.vel.copy()
        elif strategy in ('dr_last', 'dr_imu'):
            drift = rng.normal(0.0, SIGMA_DRIFT * np.sqrt(DT), 3)
            v_used = v_last if strategy == 'dr_last' else pursuer.vel
            pos_estimate = pos_estimate + v_used * DT + drift
        # freeze: the drone hovers while jammed, so the estimate stays correct
        jammed = jammed_next

        p_list.append(pursuer.pos.copy())
        e_list.append(evader.pos.copy())
        est_list.append(pos_estimate.copy())
        err_list.append(np.linalg.norm(pursuer.pos - pos_estimate))
        jam_list.append(1 if jammed_next else 0)
        time_list.append(t_next)

        if np.linalg.norm(pursuer.pos - evader.pos) < CAPTURE_R:
            captured, cap_time = True, t_next
            break

    return dict(
        p_traj=np.array(p_list), e_traj=np.array(e_list),
        est_traj=np.array(est_list), est_err=np.array(err_list),
        jam_log=np.array(jam_list), times=np.array(time_list),
        captured=captured, cap_time=cap_time,
    )


def run_monte_carlo(jam_offset=JAM_OFFSET, n_seeds=N_MC):
    """Capture time and max estimation error per strategy over n_seeds seeds
    (baseline and freeze have no noise and are run once)."""
    stats = {}
    for s in STRATEGIES:
        seeds = range(n_seeds) if s in ('dr_last', 'dr_imu') else [PLOT_SEED]
        caps, errs, n_cap = [], [], 0
        for sd in seeds:
            r = run_simulation(s, sd, jam_offset)
            caps.append(r['cap_time'] if r['captured'] else np.nan)
            errs.append(r['est_err'].max())
            n_cap += int(r['captured'])
        stats[s] = dict(cap=np.array(caps), err=np.array(errs), n_cap=n_cap,
                        n=len(caps))
    return stats


def run_offset_sweep():
    """Mean capture time vs first-jam onset (N_SWEEP_SEEDS seeds per DR case)."""
    table = {s: [] for s in STRATEGIES}
    for off in SWEEP_OFFSETS:
        st = run_monte_carlo(off, N_SWEEP_SEEDS)
        for s in STRATEGIES:
            table[s].append((np.nanmean(st[s]['cap']), st[s]['n_cap'], st[s]['n']))
    return table


def window_error(res, t_end):
    """Estimation error at the logged time closest to t_end."""
    k = int(np.argmin(np.abs(res['times'] - t_end)))
    return res['est_err'][k]


# ── Plots ────────────────────────────────────────────────────

def plot_trajectories_3d(results, out_dir):
    fig = plt.figure(figsize=(20, 5))
    for idx, s in enumerate(STRATEGIES):
        r = results[s]
        p, e, est = r['p_traj'], r['e_traj'], r['est_traj']
        ax = fig.add_subplot(1, 4, idx + 1, projection='3d')
        ax.view_init(elev=35, azim=-70)
        ax.plot(p[:, 0], p[:, 1], p[:, 2], color='red', linewidth=1.8,
                label='Pursuer (true)')
        jam = r['jam_log'].astype(bool)
        est_j = est.copy()
        est_j[~jam] = np.nan
        ax.plot(est_j[:, 0], est_j[:, 1], est_j[:, 2], color=COLORS[s],
                linewidth=1.4, linestyle='--', label='Pursuer estimate (jammed)')
        ax.plot(e[:, 0], e[:, 1], e[:, 2], color='royalblue', linewidth=1.8,
                label='Evader')
        ax.scatter(*p[0], color='red', s=40)
        ax.scatter(*e[0], color='blue', s=40)
        if r['captured']:
            ax.scatter(*p[-1], color='black', s=80, marker='X',
                       label=f'Captured {r["cap_time"]:.2f}s')
            status = f'Captured {r["cap_time"]:.2f} s'
        else:
            status = 'Timeout'
        ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
        ax.set_zlim(1.5, 2.5)
        ax.set_title(f'{LABELS[s]}\n{status}', fontsize=9)
        ax.legend(fontsize=6, loc='upper left')
    fig.suptitle(f'S007 Jamming & Blind Pursuit — 3D Trajectories (seed {PLOT_SEED})',
                 fontsize=11, y=1.04)
    plt.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'trajectories_3d.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_estimation_error(results, out_dir):
    fig, ax = plt.subplots(figsize=(12, 5))
    for s in STRATEGIES:
        r = results[s]
        ax.plot(r['times'], r['est_err'], color=COLORS[s], linewidth=1.8,
                label=LABELS[s])
        if r['captured']:
            ax.axvline(r['cap_time'], color=COLORS[s], linestyle='--',
                       linewidth=1.0, alpha=0.7)
    t_max = max(results[s]['times'][-1] for s in STRATEGIES)
    t0 = JAM_OFFSET
    while t0 < t_max:
        ax.axvspan(t0, t0 + JAM_DURATION, color='salmon', alpha=0.15)
        t0 += JAM_PERIOD
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Own-position estimation error (m)')
    ax.set_title('S007 Jamming — Pursuer Self-Position Error vs Time '
                 f'(seed {PLOT_SEED}; salmon = jam windows, dashed = capture)',
                 fontsize=10)
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'estimation_error.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def plot_capture_bar(stats, out_dir):
    """Mean capture time over the Monte-Carlo runs (whiskers = min–max)."""
    fig, ax = plt.subplots(figsize=(9, 5))
    xs = np.arange(len(STRATEGIES))
    for x, s in zip(xs, STRATEGIES):
        c = stats[s]['cap']
        c = c[~np.isnan(c)]
        mean = c.mean()
        ax.bar(x, mean, color=COLORS[s], edgecolor='black', width=0.5)
        if stats[s]['n'] > 1:
            ax.errorbar(x, mean, yerr=[[mean - c.min()], [c.max() - mean]],
                        color='black', capsize=6)
        txt = f'{mean:.2f} s'
        if stats[s]['n'] > 1:
            txt += f'\n(n={stats[s]["n"]}, {stats[s]["n_cap"]} captured)'
        ax.text(x, c.max() + 0.1, txt, ha='center', va='bottom', fontsize=9)
    ax.set_xticks(xs)
    ax.set_xticklabels([LABELS[s].replace(': ', ':\n') for s in STRATEGIES])
    ax.set_ylabel('Capture time (s)')
    ax.set_title('S007 Jamming — Capture Time by Strategy\n'
                 f'(DR cases: mean over {N_MC} drift seeds, whiskers = min–max)',
                 fontsize=10)
    top = max(np.nanmax(stats[s]['cap']) for s in STRATEGIES)
    ax.set_ylim(0, top * 1.3)
    ax.grid(axis='y', alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'capture_bar.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def plot_offset_sweep(table, out_dir):
    """Mean capture time vs first-jam onset for every strategy."""
    fig, ax = plt.subplots(figsize=(10, 5))
    for s in STRATEGIES:
        means = [m for m, _, _ in table[s]]
        ax.plot(SWEEP_OFFSETS, means, 'o-', color=COLORS[s], label=LABELS[s])
    ax.axvline(JAM_OFFSET, color='grey', linestyle=':', label='Offset used in main run')
    ax.set_xlabel('First jam onset (s)  [jam = onset … onset + 1 s, repeats every 3 s]')
    ax.set_ylabel('Mean capture time (s)')
    ax.set_title(f'S007 Jamming — Capture Time vs Jam Timing '
                 f'(DR cases: mean of {N_SWEEP_SEEDS} seeds)', fontsize=10)
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'jam_offset_sweep.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def save_animation(results, out_dir):
    import matplotlib.animation as animation

    max_len = max(len(results[s]['p_traj']) for s in STRATEGIES)
    step = int(np.ceil(max_len / 140))
    n_frames = (max_len - 1) // step + 2

    fig, axes = plt.subplots(1, 4, figsize=(20, 5))
    arts = []
    for ax, s in zip(axes, STRATEGIES):
        r = results[s]
        pts = np.vstack([r['p_traj'], r['e_traj'], r['est_traj']])
        lo = pts.min(axis=0)[:2] - 0.8
        hi = pts.max(axis=0)[:2] + 0.8
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
        for s, (lp, le, dp, de, dx, ti) in zip(STRATEGIES, arts):
            r = results[s]
            n = len(r['p_traj'])
            si = min(i * step, n - 1)
            p, e, est = r['p_traj'], r['e_traj'], r['est_traj']
            lp.set_data(p[:si + 1, 0], p[:si + 1, 1])
            le.set_data(e[:si + 1, 0], e[:si + 1, 1])
            dp.set_data([p[si, 0]], [p[si, 1]])
            de.set_data([e[si, 0]], [e[si, 1]])
            dx.set_data([est[si, 0]], [est[si, 1]])
            t = r['times'][si]
            if si == n - 1 and r['captured']:
                ti.set_text(f'{LABELS[s]}\nCaptured {r["cap_time"]:.2f} s')
            else:
                jam = ' [JAM]' if r['jam_log'][si] else ''
                ti.set_text(f'{LABELS[s]}\nt={t:.2f}s{jam} err={r["est_err"][si]:.2f}m')
            out += [lp, le, dp, de, dx, ti]
        return out

    ani = animation.FuncAnimation(fig, update, frames=n_frames, interval=60, blit=False)
    fig.suptitle('S007 Jamming & Blind Pursuit '
                 '(red ▲ = pursuer true, × = pursuer estimate, blue ■ = evader)',
                 fontsize=10)
    plt.tight_layout()
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer='pillow', fps=16, dpi=80)
    plt.close()
    print(f'Saved: {path}  ({n_frames} frames, every {step} steps)')


# ── Main ─────────────────────────────────────────────────────

if __name__ == '__main__':
    results = {s: run_simulation(s, PLOT_SEED) for s in STRATEGIES}
    print(f'Plotted run (seed {PLOT_SEED}):')
    for s in STRATEGIES:
        r = results[s]
        status = (f'captured @ {r["cap_time"]:.3f} s' if r['captured']
                  else f'timeout @ {r["times"][-1]:.2f} s')
        t_last = JAM_OFFSET + JAM_DURATION - DT
        print(f'  [{s:<9}] {status}  | max est_err = {r["est_err"].max():.3f} m'
              f'  | err at last jammed step of 1st jam (t={t_last:.3f}s) = '
              f'{window_error(r, t_last):.3f} m')

    stats = run_monte_carlo()
    print(f'\nMonte Carlo (DR cases: seeds 0..{N_MC - 1}; baseline/freeze are deterministic):')
    for s in STRATEGIES:
        c = stats[s]['cap']
        e = stats[s]['err']
        print(f'  [{s:<9}] captured {stats[s]["n_cap"]}/{stats[s]["n"]} | '
              f'cap time mean {np.nanmean(c):.3f} s, min {np.nanmin(c):.3f}, '
              f'max {np.nanmax(c):.3f} | max est_err mean {e.mean():.3f} m, '
              f'max {e.max():.3f} m')
    d = stats['dr_last']['cap'] - stats['dr_imu']['cap']
    print(f'  paired difference dr_last - dr_imu: mean {np.nanmean(d):.3f} s, '
          f'dr_last slower in {int(np.sum(d > 0))}/{N_MC} runs')

    table = run_offset_sweep()
    print(f'\nJam-onset sweep (mean capture time, s; DR cases over {N_SWEEP_SEEDS} seeds):')
    print('  onset  ' + '  '.join(f'{s:>9}' for s in STRATEGIES))
    for k, off in enumerate(SWEEP_OFFSETS):
        row = []
        for s in STRATEGIES:
            m, nc, n = table[s][k]
            row.append(f'{m:9.3f}' + ('' if nc == n else f'({nc}/{n})'))
        print(f'  {off:5.2f}  ' + '  '.join(row))

    # End-game example: jam starting at 2.0 s, last-velocity DR, plotted seed
    ex = run_simulation('dr_last', PLOT_SEED, 2.0)
    rng_ = np.linalg.norm(ex['p_traj'] - ex['e_traj'], axis=1)
    print(f'\nEnd-game example (dr_last, first jam at 2.00 s, seed {PLOT_SEED}):')
    for t_q in [2.0, 2.2, 2.4, 2.6, 2.8, 2.98, 3.2]:
        k = int(np.argmin(np.abs(ex['times'] - t_q)))
        print(f'  t={ex["times"][k]:.3f} s  range={rng_[k]:.3f} m  '
              f'est_err={ex["est_err"][k]:.3f} m')
    print(f'  capture @ {ex["cap_time"]:.3f} s')
    k = int(np.argmin(np.abs(results['dr_last']['times'] - (JAM_OFFSET + JAM_DURATION - DT))))
    r_main = np.linalg.norm(results['dr_last']['p_traj'][k] - results['dr_last']['e_traj'][k])
    print(f'Main run dr_last: range at t={results["dr_last"]["times"][k]:.3f} s = {r_main:.3f} m')

    out_dir = os.path.normpath(OUTPUT_DIR)
    plot_trajectories_3d(results, out_dir)
    plot_estimation_error(results, out_dir)
    plot_capture_bar(stats, out_dir)
    plot_offset_sweep(table, out_dir)
    save_animation(results, out_dir)
