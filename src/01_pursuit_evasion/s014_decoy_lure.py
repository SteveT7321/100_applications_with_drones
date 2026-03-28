"""
S014 Decoy & Lure
==================
Evader deploys a decoy drone that mimics its motion while the real evader
escapes smoothly.  The decoy oscillates (high jerk) to confuse the pursuer
while the real evader flies a constant-velocity escape path (low jerk).

The pursuer uses Bayesian trajectory-smoothness classification (jerk-based)
to distinguish the real evader from the decoy.

Comparison:
  - correct_id   : pursuer correctly identifies real evader (low-jerk track)
  - misid        : pursuer is fooled — always chases the decoy (high-jerk track)
  - perfect_id   : oracle pursuer — always knows the real target (upper bound)

Also runs 20-seed Monte-Carlo to measure confusion rate.

Usage:
    conda activate drones
    python src/01_pursuit_evasion/s014_decoy_lure.py
"""

import sys, os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from src.base.drone_base import DroneBase

# ── Parameters ───────────────────────────────────────────────
SIGMA_DECOY  = 0.2    # m — decoy position noise std
SIGMA_OBS    = 0.05   # m — sensor noise on real evader observation
T_LURE       = 5.0    # s — lure phase: decoy oscillates, then hovers
T_UPDATE     = 0.5    # s — Bayesian update interval
LAMBDA_BAYES = 0.6    # update gain
DECOY_AMP    = 1.5    # m — decoy lateral oscillation amplitude
DECOY_FREQ   = 1.2    # Hz — decoy oscillation frequency
V_PURSUER    = 5.0    # m/s
V_EVADER     = 3.0    # m/s
V_DECOY      = 3.0    # m/s — decoy forward speed (same as evader to mimic)
CAPTURE_R    = 0.15   # m
DT           = 1 / 48
MAX_TIME     = 20.0
JERK_WINDOW  = 20     # steps for jerk computation

# Evader and decoy start well-separated so pursuer must choose which to chase
INIT_PURSUER = np.array([ 0.0,  0.0, 2.0])
INIT_EVADER  = np.array([ 8.0,  3.0, 2.0])   # evader above centerline
INIT_DECOY   = np.array([ 8.0, -3.0, 2.0])   # decoy below centerline

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '01_pursuit_evasion', 's014_decoy_lure',
)


# ── Helpers ───────────────────────────────────────────────────

def mean_jerk(vel_history):
    """Mean absolute acceleration (jerk proxy) over recent window."""
    if len(vel_history) < 2:
        return 0.0
    accs = [np.linalg.norm((vel_history[k] - vel_history[k-1]) / DT)
            for k in range(1, len(vel_history))]
    return float(np.mean(accs))


# ── Simulation ────────────────────────────────────────────────

def run_simulation(strategy='correct_id', seed=42):
    """
    strategy: 'correct_id' | 'misid' | 'perfect_id'
    Returns (p_traj, e_traj, d_traj, prob_log, times, captured, cap_time, final_target)
    """
    rng = np.random.default_rng(seed)

    pursuer   = DroneBase(INIT_PURSUER.copy(), max_speed=V_PURSUER, dt=DT)
    evader    = DroneBase(INIT_EVADER.copy(),  max_speed=V_EVADER,  dt=DT)
    decoy_pos = INIT_DECOY.copy().astype(float)

    # Track observations: track0 = evader (smooth), track1 = decoy (jittery)
    track0_pos = INIT_EVADER.copy().astype(float)
    track1_pos = INIT_DECOY.copy().astype(float)
    track0_vel_hist = [np.zeros(3)]
    track1_vel_hist = [np.zeros(3)]

    log_ratio  = 0.0   # log P(real|track0) / P(real|track1)
    prob_log   = [0.5] # P(track0 = real evader)
    last_update_t = 0.0

    p_traj   = [pursuer.pos.copy()]
    e_traj   = [evader.pos.copy()]
    d_traj   = [decoy_pos.copy()]
    time_log = [0.0]

    # Real evader: constant smooth velocity in +x (low jerk)
    evader_vel = np.array([V_EVADER, 0.0, 0.0])

    captured = False; cap_time = None; final_target = None
    max_steps = int(MAX_TIME / DT)

    for step in range(1, max_steps + 1):
        t = step * DT

        # ── Real evader: smooth constant escape in +x ──
        evader.step(evader_vel)

        # ── Decoy: +x motion with sinusoidal y-oscillation (high jerk) ──
        if t <= T_LURE:
            dy = DECOY_AMP * np.sin(2 * np.pi * DECOY_FREQ * t)
            dy_prev = DECOY_AMP * np.sin(2 * np.pi * DECOY_FREQ * (t - DT))
            decoy_vel = np.array([V_DECOY, (dy - dy_prev) / DT, 0.0])
            # Clamp to reasonable speed
            dn = np.linalg.norm(decoy_vel)
            if dn > V_DECOY * 3:
                decoy_vel = V_DECOY * 3 * decoy_vel / dn
            decoy_pos = decoy_pos + decoy_vel * DT
        else:
            # After lure phase: decoy hovers in place
            pass
        # Add Gaussian noise to decoy
        decoy_pos_noisy = decoy_pos + rng.normal(0, SIGMA_DECOY, 3)

        # ── Noisy observations ──
        obs_e = evader.pos + rng.normal(0, SIGMA_OBS, 3)   # low noise
        obs_d = decoy_pos_noisy.copy()                       # high noise

        prev_t0 = track0_pos.copy(); prev_t1 = track1_pos.copy()
        track0_pos = obs_e.copy(); track1_pos = obs_d.copy()
        track0_vel_hist.append((track0_pos - prev_t0) / DT)
        track1_vel_hist.append((track1_pos - prev_t1) / DT)
        if len(track0_vel_hist) > JERK_WINDOW:
            track0_vel_hist.pop(0); track1_vel_hist.pop(0)

        # ── Bayesian classification update every T_UPDATE ──
        if t - last_update_t >= T_UPDATE:
            last_update_t = t
            s0 = mean_jerk(track0_vel_hist)
            s1 = mean_jerk(track1_vel_hist)
            log_ratio += LAMBDA_BAYES * (s1 - s0)   # lower jerk → more likely real
        p_real_t0 = 1.0 / (1.0 + np.exp(-log_ratio))
        prob_log.append(p_real_t0)

        # ── Pursuer target selection ──
        if strategy == 'perfect_id':
            target_pos = evader.pos
        elif strategy == 'misid':
            target_pos = track1_pos   # always chase decoy
        else:  # correct_id (Bayesian)
            target_pos = track0_pos if p_real_t0 >= 0.5 else track1_pos

        d = target_pos - pursuer.pos
        n = np.linalg.norm(d)
        pursuer.step(V_PURSUER * d / n if n > 1e-8 else np.zeros(3))

        p_traj.append(pursuer.pos.copy())
        e_traj.append(evader.pos.copy())
        d_traj.append(decoy_pos.copy())
        time_log.append(t)

        # Capture checks
        if np.linalg.norm(pursuer.pos - evader.pos) < CAPTURE_R:
            captured = True; cap_time = t; final_target = 'evader'
            break
        if np.linalg.norm(pursuer.pos - decoy_pos) < CAPTURE_R:
            captured = True; cap_time = t; final_target = 'decoy'
            break

    return (
        np.array(p_traj), np.array(e_traj), np.array(d_traj),
        np.array(prob_log), np.array(time_log),
        captured, cap_time, final_target,
    )


def run_monte_carlo(n_seeds=20):
    results = {'correct_id': [], 'misid': [], 'perfect_id': []}
    for seed in range(n_seeds):
        for strat in results:
            _, _, _, _, _, captured, cap_t, final_target = run_simulation(strat, seed)
            results[strat].append({
                'captured': captured, 'cap_time': cap_t,
                'correct': final_target == 'evader',
            })
    return results


# ── Plots ─────────────────────────────────────────────────────

def plot_trajectories(results_dict, out_dir):
    labels = ['correct_id', 'misid', 'perfect_id']
    titles = ['Correct ID (Bayesian)', 'Misidentification\n(chases decoy)', 'Oracle (perfect ID)']
    colors = ['steelblue', 'tomato', 'mediumseagreen']
    fig = plt.figure(figsize=(15, 5))

    for idx, (strat, title, color) in enumerate(zip(labels, titles, colors)):
        p_traj, e_traj, d_traj, prob_log, times, captured, cap_t, final_target = results_dict[strat]
        ax = fig.add_subplot(1, 3, idx+1)
        ax.plot(e_traj[:, 0], e_traj[:, 1], color='royalblue',
                linewidth=1.8, label='Evader (real)')
        ax.plot(d_traj[:, 0], d_traj[:, 1], color='limegreen',
                linewidth=1.4, linestyle='--', label='Decoy')
        ax.plot(p_traj[:, 0], p_traj[:, 1], color=color,
                linewidth=1.8, label='Pursuer')
        ax.scatter(*p_traj[0, :2], color=color, s=60, zorder=5)
        ax.scatter(*e_traj[0, :2], color='blue', s=60, zorder=5)
        ax.scatter(*d_traj[0, :2], color='limegreen', s=60, zorder=5)
        if captured:
            ax.scatter(*p_traj[-1, :2], color='black', s=120, marker='X', zorder=7,
                       label=f'Hit {final_target} @ {cap_t:.2f}s')
        status = f'Hit {final_target} @ {cap_t:.2f}s' if captured else 'Timeout'
        ax.set_aspect('equal'); ax.grid(True, alpha=0.3)
        ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
        ax.set_title(f'{title}\n{status}', fontsize=9)
        ax.legend(fontsize=7, loc='upper left')

    fig.suptitle('S014 Decoy & Lure — XY Trajectories', fontsize=11)
    plt.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'trajectories_xy.png')
    plt.savefig(path, dpi=150, bbox_inches='tight'); plt.close(); print(f'Saved: {path}')


def plot_classification_prob(results_dict, out_dir):
    p_traj, e_traj, d_traj, prob_log, times, captured, cap_t, _ = results_dict['correct_id']
    fig, ax = plt.subplots(figsize=(11, 5))
    t_prob = np.linspace(0, times[-1], len(prob_log))
    ax.plot(t_prob, prob_log, color='steelblue', linewidth=2.0,
            label='P(track0 = real evader)')
    ax.axhline(0.5, color='grey', linestyle='--', linewidth=1.0, label='Decision boundary')
    ax.axvline(T_LURE, color='orange', linestyle=':', linewidth=1.5,
               label=f'End of lure phase ({T_LURE}s)')
    if cap_t:
        ax.axvline(cap_t, color='black', linestyle='--', linewidth=1.2,
                   label=f'Captured @ {cap_t:.2f}s')
    ax.fill_between(t_prob, 0.5, prob_log,
                    where=np.array(prob_log) >= 0.5, alpha=0.15, color='steelblue',
                    label='Believes track0 = real')
    ax.set_xlabel('Time (s)'); ax.set_ylabel('P(real evader = track0)')
    ax.set_title('S014 Decoy & Lure — Bayesian Classification Probability vs Time')
    ax.set_ylim(0, 1); ax.legend(fontsize=8); ax.grid(True, alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'classification_prob.png')
    plt.savefig(path, dpi=150); plt.close(); print(f'Saved: {path}')


def plot_monte_carlo(mc_results, out_dir):
    strats = ['correct_id', 'misid', 'perfect_id']
    labels = ['Correct ID\n(Bayesian)', 'Misidentification', 'Oracle']
    capture_rates = []
    correct_rates = []
    mean_cap_times = []

    for strat in strats:
        data = mc_results[strat]
        n = len(data)
        capture_rates.append(100 * sum(d['captured'] for d in data) / n)
        correct_rates.append(100 * sum(d['correct'] for d in data if d['captured']) /
                             max(sum(d['captured'] for d in data), 1))
        valid_times = [d['cap_time'] for d in data if d['captured'] and d['cap_time']]
        mean_cap_times.append(np.mean(valid_times) if valid_times else MAX_TIME)

    fig, axes = plt.subplots(1, 3, figsize=(13, 5))
    colors = ['steelblue', 'tomato', 'mediumseagreen']

    for ax, vals, ylabel, title in zip(
        axes,
        [capture_rates, correct_rates, mean_cap_times],
        ['Capture rate (%)', 'Correct-target rate (%)', 'Mean capture time (s)'],
        ['Capture Rate', 'Correct Target Hit Rate', 'Mean Capture Time'],
    ):
        bars = ax.bar(labels, vals, color=colors, edgecolor='black', width=0.4)
        for bar, v in zip(bars, vals):
            ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.5,
                    f'{v:.1f}', ha='center', va='bottom', fontsize=10, fontweight='bold')
        ax.set_title(title, fontsize=9); ax.set_ylabel(ylabel)
        ax.grid(axis='y', alpha=0.3)

    fig.suptitle(f'S014 Decoy & Lure — Monte-Carlo Results ({len(mc_results["correct_id"])} seeds)',
                 fontsize=10)
    plt.tight_layout()
    path = os.path.join(out_dir, 'monte_carlo.png')
    plt.savefig(path, dpi=150); plt.close(); print(f'Saved: {path}')


def save_animation(results_dict, out_dir):
    import matplotlib.animation as animation

    labels = ['correct_id', 'misid', 'perfect_id']
    titles = ['Correct ID', 'Misid', 'Oracle']
    colors = ['steelblue', 'tomato', 'mediumseagreen']

    max_len = max(len(results_dict[s][0]) for s in labels)
    step = 3
    fig, axes = plt.subplots(1, 3, figsize=(15, 5))

    store = []
    for ax, strat, title, color in zip(axes, labels, titles, colors):
        p_traj, e_traj, d_traj, prob_log, times, captured, cap_t, final_target = results_dict[strat]
        all_pts = np.vstack([p_traj, e_traj, d_traj])
        lo = all_pts.min(axis=0)[:2] - 1.0; hi = all_pts.max(axis=0)[:2] + 1.0
        ax.set_xlim(lo[0], hi[0]); ax.set_ylim(lo[1], hi[1])
        ax.set_aspect('equal'); ax.grid(True, alpha=0.3)
        ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
        for pts, c, s in [(p_traj, color, 60), (e_traj, 'blue', 60), (d_traj, 'limegreen', 60)]:
            ax.scatter(*pts[0, :2], color=c, s=s, zorder=5)
        lp, = ax.plot([], [], color=color,       linewidth=1.8)
        le, = ax.plot([], [], color='royalblue', linewidth=1.8)
        ld, = ax.plot([], [], color='limegreen', linewidth=1.4, linestyle='--')
        dp, = ax.plot([], [], '^', color=color,   markersize=9, zorder=6)
        de, = ax.plot([], [], 'bs', markersize=9, zorder=6)
        dd, = ax.plot([], [], 'g^', markersize=8, zorder=6, alpha=0.7)
        ti  = ax.set_title(title, fontsize=9)
        t_prob = np.linspace(0, times[-1], len(prob_log))
        store.append((lp, le, ld, dp, de, dd, ti,
                      p_traj, e_traj, d_traj, prob_log, t_prob, times,
                      captured, cap_t, final_target, title))

    n_frames = max_len // step

    def update(i):
        arts = []
        for (lp, le, ld, dp, de, dd, ti,
             p_traj, e_traj, d_traj, prob_log, t_prob, times,
             captured, cap_t, final_target, title) in store:
            si = min(i * step, len(e_traj)-1)
            done = (i * step >= len(e_traj)-1)
            t = times[si]
            for l, pt in [(lp, p_traj), (le, e_traj), (ld, d_traj)]:
                sj = min(si, len(pt)-1)
                l.set_data(pt[:sj+1, 0], pt[:sj+1, 1])
            for d, pt in [(dp, p_traj), (de, e_traj), (dd, d_traj)]:
                sj = min(si, len(pt)-1)
                d.set_data([float(pt[sj, 0])], [float(pt[sj, 1])])
            prob_idx = min(int(si / len(e_traj) * len(prob_log)), len(prob_log)-1)
            prob = prob_log[prob_idx]
            if done:
                st = f'Hit {final_target} {cap_t:.2f}s [DONE]' if captured else 'Timeout [DONE]'
                ti.set_text(f'{title}  t={t:.1f}s\n{st}')
            else:
                ti.set_text(f'{title}  t={t:.1f}s  P(real=trk0)={prob:.2f}')
            arts += [lp, le, ld, dp, de, dd, ti]
        return arts

    ani = animation.FuncAnimation(fig, update, frames=n_frames, interval=60, blit=True)
    fig.suptitle('S014 Decoy & Lure (blue=pursuer, royalblue=evader, green=decoy)', fontsize=10)
    plt.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer='pillow', fps=16, dpi=100); plt.close(); print(f'Saved: {path}')


if __name__ == '__main__':
    strategies = ['correct_id', 'misid', 'perfect_id']
    results_dict = {}
    for strat in strategies:
        res = run_simulation(strat, seed=42)
        p_traj, e_traj, d_traj, prob_log, times, captured, cap_t, final_target = res
        status = f'hit {final_target} @ {cap_t:.2f}s' if captured else 'timeout'
        print(f'[{strat:<12}]  {status}')
        results_dict[strat] = res

    print('\nRunning Monte-Carlo (20 seeds)...')
    mc = run_monte_carlo(20)

    out_dir = os.path.normpath(OUTPUT_DIR)
    plot_trajectories(results_dict, out_dir)
    plot_classification_prob(results_dict, out_dir)
    plot_monte_carlo(mc, out_dir)
    save_animation(results_dict, out_dir)
