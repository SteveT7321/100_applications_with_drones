"""
S019 Dynamic Target Reassignment
==================================
3 pursuers vs 3 evaders with mixed strategies (straight, spiral, random).
Compares three assignment strategies:
  1. Static   — Hungarian at t=0, never updated
  2. Dynamic  — re-optimise every T_CHECK seconds using the Hungarian algorithm;
                only reassign if improvement > DC_SWITCH (switching cost)
  3. Greedy   — each pursuer always chases its nearest evader (no coordination)

The dynamic approach handles unpredictable evader behaviour by periodically
recomputing the optimal assignment, balancing responsiveness against oscillation.

Usage:
    conda activate drones
    python src/01_pursuit_evasion/s019_dynamic_reassignment.py
"""

import sys, os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.optimize import linear_sum_assignment

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

# ── Parameters ─────────────────────────────────
M           = 3            # number of pursuers
N           = 3            # number of evaders
V_PURSUER   = 5.0          # m/s  pursuer speed
V_EVADER    = 3.5          # m/s  evader speed
CAPTURE_R   = 0.30         # m    capture radius
DT          = 0.05         # s    simulation timestep
T_MAX       = 120.0        # s    max simulation time
T_CHECK     = 1.0          # s    reassignment check period
DC_SWITCH   = 0.5          # s    switching cost

# ── Starting positions ──
# Pursuers: 3 positions forming a triangle.
# Evaders: 3 positions also forming a triangle, on the right side.
#
# Initial Hungarian assigns: P0→E0, P1→E1, P2→E2 (by proximity).
#
# Evader strategies designed to create TWO reassignment events:
#   E0 (straight): runs strongly DOWNWARD (-Y), heading toward P2's zone.
#                  After ~5 s it is closer to P2 than P0 → swap P0↔P2.
#   E1 (spiral):   runs strongly UPWARD (+Y), heading toward P0's zone.
#                  After ~5 s it is closer to P0 (now chasing lower) than P1.
#   E2 (random):   random walk with +X bias.
#
# All pursuers start at x=0 (centre). Evaders start at x=+20 (far right).
# This gives ~20 m initial separation → minimum intercept time = 20/1.5 ≈ 13 s
# (if evader runs away from pursuer at V_EVADER).
# ── Two-band layout for guaranteed reassignment improvement ──
#
# P0 (top)  initially chases E0 (also top) — straight evader runs DOWN → toward P1
# P1 (bot)  initially chases E1 (also bot) — spiral evader runs UP   → toward P0
# P2 (mid)  chases E2 (random mid)
#
# After ~4 s:
#   E0 has moved DOWN 12 m (from y=12 to y=0), now MUCH closer to P1 than P0
#   E1 has moved UP  12 m (from y=-12 to y=0), now MUCH closer to P0 than P1
#   Static keeps P0→E0, P1→E1: both pursuers have to reverse directions (SLOW)
#   Dynamic swaps: P1→E0 (right next to P1 now), P0→E1 (right next to P0 now)
#   → Dynamic saves both pursuer reversal costs → lower total time
#
# Expected: Static total >> Dynamic total (demonstrating 2+ reassignment events)
PURSUER_STARTS = np.array([
    [ 0.,  12., 2.],   # P0 — top
    [ 0.,   0., 2.],   # P1 — mid
    [ 0., -12., 2.],   # P2 — bottom
], dtype=float)

EVADER_STARTS = np.array([
    [ 20.,  12., 2.],   # E0 straight — top-right  → runs DOWNWARD toward P1 zone
    [ 20., -12., 2.],   # E1 spiral   — bot-right  → runs UPWARD  toward P0 zone
    [ 20.,   0., 2.],   # E2 random   — mid-right
], dtype=float)

ARENA_XY = 50.0   # m  soft arena boundary

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '01_pursuit_evasion', 's019_dynamic_reassignment',
)

RNG = np.random.default_rng(0)   # fixed seed for reproducibility


# ── Helpers ────────────────────────────────────

def cost_matrix(pursuers, evaders, captured=None):
    """Cost matrix C[i,j] = distance(P_i, E_j) / V_PURSUER.
    Captured evaders get cost = 1e6 (excluded from assignment).
    """
    C = np.zeros((M, N))
    for i in range(M):
        for j in range(N):
            if captured is not None and captured[j]:
                C[i, j] = 1e6
            else:
                C[i, j] = np.linalg.norm(pursuers[i] - evaders[j]) / V_PURSUER
    return C


def hungarian_assignment(C):
    """Return assignment dict {pursuer_i: evader_j} via Hungarian algorithm."""
    row_ind, col_ind = linear_sum_assignment(C)
    return dict(zip(row_ind.tolist(), col_ind.tolist()))


def pure_pursuit_step(pursuer_pos, target_pos, speed, dt):
    """Move pursuer toward target at fixed speed. Returns new position."""
    diff = target_pos - pursuer_pos
    dist = np.linalg.norm(diff)
    if dist < 1e-6:
        return pursuer_pos.copy()
    move = min(speed * dt, dist)
    return pursuer_pos + (diff / dist) * move


def evader_straight_step(pos, vel, dt):
    """Straight-line escape evader. vel is a fixed direction * speed."""
    new_pos = pos + vel * dt
    new_pos[2] = 2.0
    return new_pos


def evader_spiral_step(pos, t, dt):
    """Spiral evader: runs UPWARD toward P0's zone.
    E1 starts bottom-right; moves at full speed upward.
    The direction sweeps slowly from +Y toward +X (spiral).
    omega = 0.15 rad/s → full quarter turn in ~10 s.

    After ~4 s E1 has moved ~14 m upward (from y=-12 to y=+2),
    now MUCH closer to P0 (top, y=12) than to P1 (which has chased
    downward from y=0).
    """
    omega = 0.15   # rad/s  — slow rightward sweep
    phi   = omega * t
    # Starts going +Y, slowly sweeps toward +X
    direction = np.array([np.sin(phi), np.cos(phi), 0.0])
    new_pos   = pos + direction * V_EVADER * dt
    new_pos[2] = 2.0
    return new_pos


def evader_random_step(pos, rng, dt):
    """Random-walk evader with +X bias. Reflects at arena boundary."""
    angle = rng.uniform(0, 2 * np.pi)
    vel = np.array([np.cos(angle), np.sin(angle), 0.0]) * V_EVADER
    new_pos = pos + vel * dt
    new_pos[2] = 2.0
    new_pos[0] = np.clip(new_pos[0], -ARENA_XY, ARENA_XY)
    new_pos[1] = np.clip(new_pos[1], -ARENA_XY, ARENA_XY)
    return new_pos


# ── Simulation ─────────────────────────────────

def run_one(strategy):
    """
    Simulate one assignment strategy.
    strategy: 'static' | 'dynamic' | 'greedy'
    Returns dict with trajectories, capture times, assignment history.
    """
    rng_ev = np.random.default_rng(42)

    pursuers = PURSUER_STARTS.copy()
    evaders  = EVADER_STARTS.copy()

    captured        = [False] * N
    capture_times   = [None]  * N

    # Initial Hungarian assignment
    C0 = cost_matrix(pursuers, evaders, captured)
    assignment = hungarian_assignment(C0)

    # E0 straight: runs DOWNWARD toward P1's zone
    # E0 starts top-right; runs down. After ~4 s it's near mid-right — close to P1.
    # Static: P0 keeps chasing E0 downward (P0 is now mid-zone, far from E1 top)
    # Dynamic: P1 takes E0 (near P1), P0 takes E1 (near P0) — much faster!
    straight_vel = np.array([0.0, -1.0, 0.0])
    straight_vel = straight_vel / np.linalg.norm(straight_vel) * V_EVADER

    max_steps = int(T_MAX / DT)

    p_traj       = [pursuers.copy()]
    e_traj       = [evaders.copy()]
    assign_hist  = [assignment.copy()]
    reassign_events = []

    for step in range(max_steps):
        t = step * DT

        # ── Evader moves
        new_evaders = evaders.copy()
        if not captured[0]:
            new_evaders[0] = evader_straight_step(evaders[0], straight_vel, DT)
        if not captured[1]:
            new_evaders[1] = evader_spiral_step(evaders[1], t, DT)
        if not captured[2]:
            new_evaders[2] = evader_random_step(evaders[2], rng_ev, DT)
        evaders = new_evaders

        # ── Reassignment logic
        if strategy == 'static':
            pass

        elif strategy == 'dynamic':
            # Check every T_CHECK seconds (aligned to DT grid)
            if step > 0 and abs((t % T_CHECK)) < DT * 0.5:
                C = cost_matrix(pursuers, evaders, captured)
                new_assign = hungarian_assignment(C)

                current_cost = sum(C[i, assignment[i]] for i in range(M))
                new_cost     = sum(C[i, new_assign[i]]  for i in range(M))
                improvement  = current_cost - new_cost

                if improvement > DC_SWITCH:
                    assignment = new_assign
                    reassign_events.append(step)

        elif strategy == 'greedy':
            for i in range(M):
                dists = [(np.linalg.norm(pursuers[i] - evaders[j]) if not captured[j] else 1e9, j)
                         for j in range(N)]
                assignment[i] = min(dists)[1]

        # ── Pursuer moves
        new_pursuers = pursuers.copy()
        for i in range(M):
            j = assignment[i]
            if not captured[j]:
                new_pursuers[i] = pure_pursuit_step(pursuers[i], evaders[j], V_PURSUER, DT)
        pursuers = new_pursuers

        # ── Capture check
        for i in range(M):
            j = assignment[i]
            if not captured[j]:
                if np.linalg.norm(pursuers[i] - evaders[j]) <= CAPTURE_R:
                    captured[j] = True
                    capture_times[j] = (step + 1) * DT

        p_traj.append(pursuers.copy())
        e_traj.append(evaders.copy())
        assign_hist.append(assignment.copy())

        if all(captured):
            break

    total_time = max(t for t in capture_times if t is not None) if any(capture_times) else T_MAX

    return {
        'p_traj':          np.array(p_traj),
        'e_traj':          np.array(e_traj),
        'assign_hist':     assign_hist,
        'capture_times':   capture_times,
        'total_time':      total_time,
        'reassign_events': reassign_events,
        'n_reassigns':     len(reassign_events),
    }


def run_simulation():
    """Run all three strategies and return combined results."""
    static  = run_one('static')
    dynamic = run_one('dynamic')
    greedy  = run_one('greedy')
    return {'static': static, 'dynamic': dynamic, 'greedy': greedy}


# ── Plot colours ────────────────────────────────────────────────────────────

PURSUER_COLORS = ['crimson', 'tomato', 'firebrick']
EVADER_COLORS  = ['royalblue', 'steelblue', 'navy']
STRATEGY_COLORS = {
    'static':  'darkorange',
    'dynamic': 'seagreen',
    'greedy':  'mediumpurple',
}


# ── Plots ──────────────────────────────────────

def plot_trajectories(data, out_dir):
    """3D trajectories for all three strategies side-by-side."""
    fig = plt.figure(figsize=(16, 5))

    strategies = ['static', 'dynamic', 'greedy']
    titles = ['Static (Hungarian at t=0)', 'Dynamic (Re-optimise)', 'Greedy (Nearest)']

    for col, (strat, title) in enumerate(zip(strategies, titles)):
        d = data[strat]
        p_traj = d['p_traj']
        e_traj = d['e_traj']
        ct     = d['capture_times']
        T      = d['total_time']

        ax = fig.add_subplot(1, 3, col + 1, projection='3d')

        for i in range(M):
            ax.plot(p_traj[:, i, 0], p_traj[:, i, 1], p_traj[:, i, 2],
                    color=PURSUER_COLORS[i], lw=1.5, alpha=0.85)
            ax.scatter(*p_traj[0, i], color=PURSUER_COLORS[i], s=60, marker='D', zorder=5)

        for j in range(N):
            ax.plot(e_traj[:, j, 0], e_traj[:, j, 1], e_traj[:, j, 2],
                    color=EVADER_COLORS[j], lw=1.5, alpha=0.7, linestyle='--')
            ax.scatter(*e_traj[0, j], color=EVADER_COLORS[j], s=60, marker='^', zorder=5)

        # Capture markers
        for j in range(N):
            if ct[j] is not None:
                step_idx = min(int(ct[j] / DT), len(e_traj) - 1)
                ax.scatter(*e_traj[step_idx, j], color='gold', s=120, marker='*', zorder=6)

        # Reassignment event markers (dynamic only)
        if strat == 'dynamic':
            for ev_step in d['reassign_events']:
                if ev_step < len(p_traj):
                    ax.scatter(p_traj[ev_step, :, 0],
                               p_traj[ev_step, :, 1],
                               p_traj[ev_step, :, 2],
                               color='lime', s=30, marker='x', zorder=7, alpha=0.8)

        ax.set_title(f'{title}\nT={T:.2f}s | Reassigns:{d["n_reassigns"]}', fontsize=8)
        ax.set_xlabel('X (m)', fontsize=7)
        ax.set_ylabel('Y (m)', fontsize=7)
        ax.set_zlabel('Z (m)', fontsize=7)

        all_pos = np.concatenate([p_traj.reshape(-1, 3), e_traj.reshape(-1, 3)], axis=0)
        margin = 2.0
        ax.set_xlim(all_pos[:, 0].min() - margin, all_pos[:, 0].max() + margin)
        ax.set_ylim(all_pos[:, 1].min() - margin, all_pos[:, 1].max() + margin)
        ax.set_zlim(1.5, 2.5)
        ax.tick_params(labelsize=6)

    fig.suptitle('S019 Dynamic Reassignment — 3D Trajectories', fontsize=11, y=1.01)
    plt.tight_layout()

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'trajectories.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_assignment_heatmap(data, out_dir):
    """Assignment matrix over time: static vs dynamic."""
    fig, axes = plt.subplots(1, 2, figsize=(13, 5))

    for ax, strat, title in zip(axes,
                                 ['static', 'dynamic'],
                                 ['Static Assignment', 'Dynamic Assignment']):
        d = data[strat]
        n_steps = len(d['assign_hist'])
        A = np.zeros((M, n_steps), dtype=int)
        for t_idx, asgn in enumerate(d['assign_hist']):
            for i in range(M):
                A[i, t_idx] = asgn.get(i, -1)

        im = ax.imshow(A, aspect='auto', cmap='tab10', vmin=0, vmax=N - 1,
                       extent=[0, n_steps * DT, M - 0.5, -0.5])

        if strat == 'dynamic':
            for ev_step in d['reassign_events']:
                ax.axvline(ev_step * DT, color='red', lw=1.5, linestyle='--', alpha=0.8)

        ax.set_title(f'{title}  ({d["n_reassigns"]} reassignment events)', fontsize=10)
        ax.set_xlabel('Time (s)', fontsize=9)
        ax.set_ylabel('Pursuer Index', fontsize=9)
        ax.set_yticks([0, 1, 2])
        ax.set_yticklabels(['P0', 'P1', 'P2'], fontsize=8)

        cbar = fig.colorbar(im, ax=ax, ticks=[0, 1, 2])
        cbar.ax.set_yticklabels(['E0 (straight)', 'E1 (spiral)', 'E2 (random)'], fontsize=7)

    fig.suptitle('S019 — Assignment Matrix over Time', fontsize=11)
    plt.tight_layout()

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'assignment_heatmap.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_capture_comparison(data, out_dir):
    """Bar chart: total mission time and per-evader capture times."""
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))

    strategies = ['static', 'dynamic', 'greedy']
    labels     = ['Static', 'Dynamic', 'Greedy']
    colors     = [STRATEGY_COLORS[s] for s in strategies]

    # Total mission time
    ax = axes[0]
    total_times = [data[s]['total_time'] for s in strategies]
    bars = ax.bar(labels, total_times, color=colors, edgecolor='black', linewidth=0.8)
    for bar, t in zip(bars, total_times):
        ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.2,
                f'{t:.2f} s', ha='center', va='bottom', fontsize=9, fontweight='bold')
    ax.set_ylabel('Total Mission Time (s)', fontsize=10)
    ax.set_title('Total Mission Time by Strategy', fontsize=10)
    ax.set_ylim(0, max(total_times) * 1.25)

    # Per-evader capture time
    ax = axes[1]
    x = np.arange(N)
    width = 0.25
    evader_labels = ['E0 (straight)', 'E1 (spiral)', 'E2 (random)']
    for k, (strat, label, color) in enumerate(zip(strategies, labels, colors)):
        ct = data[strat]['capture_times']
        vals = [c if c is not None else T_MAX for c in ct]
        ax.bar(x + k * width, vals, width, label=label, color=color,
               edgecolor='black', linewidth=0.6)
    ax.set_xlabel('Evader', fontsize=10)
    ax.set_ylabel('Capture Time (s)', fontsize=10)
    ax.set_title('Per-Evader Capture Time by Strategy', fontsize=10)
    ax.set_xticks(x + width)
    ax.set_xticklabels(evader_labels, fontsize=8)
    ax.legend(fontsize=9)
    ax.set_ylim(0, max(max(data[s]['capture_times'][j] or T_MAX
                           for j in range(N))
                       for s in strategies) * 1.15)

    plt.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'capture_comparison.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def save_animation(data, out_dir):
    """Animate all three strategies side-by-side in 3D."""
    import matplotlib.animation as animation

    strategies = ['static', 'dynamic', 'greedy']
    titles     = ['Static', 'Dynamic', 'Greedy']

    max_steps = max(len(data[s]['p_traj']) for s in strategies)
    step_dec  = max(1, max_steps // 120)

    fig = plt.figure(figsize=(15, 5))
    axes, p_lines, e_lines, p_dots, e_dots = [], [], [], [], []

    for col, (strat, title) in enumerate(zip(strategies, titles)):
        ax = fig.add_subplot(1, 3, col + 1, projection='3d')
        axes.append(ax)
        d = data[strat]
        p_traj = d['p_traj']
        e_traj = d['e_traj']

        all_pos = np.concatenate([p_traj.reshape(-1, 3), e_traj.reshape(-1, 3)], axis=0)
        margin  = 2.0
        ax.set_xlim(all_pos[:, 0].min() - margin, all_pos[:, 0].max() + margin)
        ax.set_ylim(all_pos[:, 1].min() - margin, all_pos[:, 1].max() + margin)
        ax.set_zlim(1.5, 2.5)
        ax.set_title(title, fontsize=9)
        ax.set_xlabel('X', fontsize=6); ax.set_ylabel('Y', fontsize=6)
        ax.set_zlabel('Z', fontsize=6); ax.tick_params(labelsize=5)

        for i in range(M):
            ax.scatter(*p_traj[0, i], color=PURSUER_COLORS[i], s=40, marker='D', zorder=5)
        for j in range(N):
            ax.scatter(*e_traj[0, j], color=EVADER_COLORS[j], s=40, marker='^', zorder=5)

        pls = [ax.plot([], [], [], color=PURSUER_COLORS[i], lw=1.5)[0] for i in range(M)]
        els = [ax.plot([], [], [], color=EVADER_COLORS[j],  lw=1.5, linestyle='--', alpha=0.8)[0]
               for j in range(N)]
        pds = [ax.plot([], [], [], 'o', color=PURSUER_COLORS[i], ms=6)[0] for i in range(M)]
        eds = [ax.plot([], [], [], '^', color=EVADER_COLORS[j],  ms=6)[0] for j in range(N)]

        p_lines.append(pls); e_lines.append(els)
        p_dots.append(pds);  e_dots.append(eds)

    all_artists = ([l for ls in p_lines for l in ls] + [l for ls in e_lines for l in ls] +
                   [d for ds in p_dots  for d in ds] + [d for ds in e_dots  for d in ds])

    def init():
        for art in all_artists:
            art.set_data([], [])
            art.set_3d_properties([])
        return all_artists

    def update(frame):
        for col, strat in enumerate(strategies):
            d = data[strat]
            sub_p = d['p_traj'][::step_dec]
            sub_e = d['e_traj'][::step_dec]
            fp = min(frame, len(sub_p) - 1)
            fe = min(frame, len(sub_e) - 1)
            for i in range(M):
                p_lines[col][i].set_data(sub_p[:fp+1, i, 0], sub_p[:fp+1, i, 1])
                p_lines[col][i].set_3d_properties(sub_p[:fp+1, i, 2])
                p_dots[col][i].set_data([sub_p[fp, i, 0]], [sub_p[fp, i, 1]])
                p_dots[col][i].set_3d_properties([sub_p[fp, i, 2]])
            for j in range(N):
                e_lines[col][j].set_data(sub_e[:fe+1, j, 0], sub_e[:fe+1, j, 1])
                e_lines[col][j].set_3d_properties(sub_e[:fe+1, j, 2])
                e_dots[col][j].set_data([sub_e[fe, j, 0]], [sub_e[fe, j, 1]])
                e_dots[col][j].set_3d_properties([sub_e[fe, j, 2]])
        return all_artists

    n_frames = max_steps // step_dec + 1
    ani = animation.FuncAnimation(fig, update, frames=n_frames,
                                  init_func=init, blit=False, interval=50)
    fig.suptitle('S019 Dynamic Reassignment — Strategy Comparison', fontsize=10)
    plt.tight_layout()

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer='pillow', fps=20, dpi=100)
    plt.close()
    print(f'Saved: {path}')


if __name__ == '__main__':
    data = run_simulation()

    print('=' * 60)
    print('S019 Dynamic Target Reassignment — Results')
    print('=' * 60)
    for strat in ['static', 'dynamic', 'greedy']:
        d = data[strat]
        ct_str = ', '.join(f'{c:.2f}s' if c else 'N/A' for c in d['capture_times'])
        print(f'{strat.capitalize():8s}  Total={d["total_time"]:.2f}s  '
              f'Captures=[{ct_str}]  Reassigns={d["n_reassigns"]}')
    print('=' * 60)

    t_stat = data['static']['total_time']
    t_dyn  = data['dynamic']['total_time']
    t_grd  = data['greedy']['total_time']
    print(f'Dynamic improvement vs Static  : {(t_stat - t_dyn) / t_stat * 100:+.1f}%')
    print(f'Dynamic improvement vs Greedy  : {(t_grd  - t_dyn) / t_grd  * 100:+.1f}%')
    print(f'Dynamic reassignment events    : {data["dynamic"]["n_reassigns"]}')
    print('=' * 60)

    out_dir = os.path.normpath(OUTPUT_DIR)
    plot_trajectories(data, out_dir)
    plot_assignment_heatmap(data, out_dir)
    plot_capture_comparison(data, out_dir)
    save_animation(data, out_dir)
