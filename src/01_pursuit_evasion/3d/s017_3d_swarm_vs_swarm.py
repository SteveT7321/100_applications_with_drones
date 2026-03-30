"""
S017 3D Upgrade — Swarm vs Swarm
Usage:
    conda activate drones
    python src/01_pursuit_evasion/3d/s017_3d_swarm_vs_swarm.py
"""
import sys, os, numpy as np, matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa
from matplotlib.animation import FuncAnimation, PillowWriter
from scipy.optimize import linear_sum_assignment
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..'))
from src.base.drone_base import DroneBase
OUTPUT_DIR = os.path.join(os.path.dirname(__file__), '..', '..', '..', 'outputs', '01_pursuit_evasion', '3d', 's017_3d_swarm_vs_swarm')
os.makedirs(OUTPUT_DIR, exist_ok=True)

# ── Parameters ────────────────────────────────────────────────────────────────
V_PURSUER   = 5.0
V_EVADER    = 3.5
CAPTURE_R   = 0.15
DT          = 0.02
T_MAX       = 30.0
ETA_DIVE    = 0.15
K_Z_ESCAPE  = 0.5
LAMBDA_LAYER = 2.0
Z_LAYERS    = [1.0, 3.0, 5.0]
Z_MIN, Z_MAX = 0.5, 5.5

# Initial positions as specified in task
PURSUERS = np.array([[-5., -2., 1.0], [-5.,  0., 3.0], [-5.,  2., 5.0]])
EVADERS  = np.array([[ 0., -2., 1.0], [ 0.,  0., 3.0], [ 0.,  2., 5.0]])

# Extra pursuers for 5v3
EXTRA_PURSUERS = np.array([[-5., -4., 2.0], [-5.,  4., 4.0]])

PURSUER_COLORS = ['#e63946', '#f4a261', '#2a9d8f', '#8338ec', '#06d6a0']
EVADER_COLORS  = ['#023e8a', '#0077b6', '#00b4d8']


# ── Assignment strategies ─────────────────────────────────────────────────────

def assign_hungarian_3d(pos_p, pos_e):
    """Hungarian assignment with 3D Euclidean cost + altitude penalty."""
    M, N = len(pos_p), len(pos_e)
    C = np.zeros((M, N))
    for i, pp in enumerate(pos_p):
        for j, pe in enumerate(pos_e):
            dist_3d = np.linalg.norm(pp - pe)
            alt_pen = 2.0 * abs(pp[2] - pe[2])
            C[i, j] = dist_3d + alt_pen
    # For M > N, duplicate columns with large cost
    if M > N:
        pad = np.full((M, M - N), 999.0)
        C_sq = np.hstack([C, pad])
    else:
        C_sq = C
    row, col = linear_sum_assignment(C_sq)
    assignment = {}
    for r, c in zip(row.tolist(), col.tolist()):
        assignment[r] = c % N  # map back to real evader indices
    return assignment


def assign_greedy_3d(pos_p, pos_e):
    """Greedy: each pursuer picks nearest evader; conflicts resolved by priority."""
    M, N = len(pos_p), len(pos_e)
    C = np.array([[np.linalg.norm(pos_p[i] - pos_e[j]) for j in range(N)]
                  for i in range(M)])
    assignment = {}
    taken = set()
    # Sort pursuers by their nearest-evader distance (priority)
    pursuer_order = sorted(range(M), key=lambda i: C[i].min())
    for i in pursuer_order:
        available = [j for j in range(N) if j not in taken]
        if not available:
            available = list(range(N))
        j_best = min(available, key=lambda j: C[i, j])
        assignment[i] = j_best
        taken.add(j_best)
    return assignment


# ── Motion helpers ────────────────────────────────────────────────────────────

def dive_speed(pos_p, pos_e, v_max, eta=ETA_DIVE):
    dz = pos_p[2] - pos_e[2]
    dist = np.linalg.norm(pos_e - pos_p) + 1e-8
    if dz > 0.5:
        boost = 1.0 + eta * dz / dist
        return min(v_max * boost, 1.3 * v_max)
    return v_max


def evader_escape_3d(pos_e, pos_pursuers, v_max):
    centroid = pos_pursuers.mean(axis=0)
    away = pos_e - centroid
    layer_coverage = [sum(abs(p[2] - z) < 1.0 for p in pos_pursuers) for z in Z_LAYERS]
    target_layer = Z_LAYERS[int(np.argmin(layer_coverage))]
    dz = target_layer - pos_e[2]
    vel = away + np.array([0.0, 0.0, K_Z_ESCAPE * dz])
    nv = np.linalg.norm(vel)
    if nv < 1e-8:
        vel = np.array([0.0, 0.0, K_Z_ESCAPE * dz + 1e-6])
        nv  = np.linalg.norm(vel)
    return v_max * vel / nv


# ── Simulation ────────────────────────────────────────────────────────────────

def simulate_swarm_3d(pursuers_init, evaders_init, method='hungarian'):
    pursuers = pursuers_init.copy().astype(float)
    evaders  = evaders_init.copy().astype(float)
    M, N = len(pursuers), len(evaders)
    captured      = [False] * N
    capture_times = [None]  * N

    # Initial assignment
    if method == 'hungarian':
        assignment = assign_hungarian_3d(pursuers, evaders)
    else:
        assignment = assign_greedy_3d(pursuers, evaders)

    traj_p = [pursuers.copy()]
    traj_e = [evaders.copy()]
    t = 0.0

    for step in range(int(T_MAX / DT)):
        t = step * DT

        # Move evaders
        for j in range(N):
            if not captured[j]:
                vel = evader_escape_3d(evaders[j], pursuers, V_EVADER)
                evaders[j] = evaders[j] + vel * DT
                evaders[j, 2] = np.clip(evaders[j, 2], Z_MIN, Z_MAX)

        # Move pursuers
        for i in range(M):
            target_j = assignment.get(i)
            if target_j is None or captured[target_j]:
                remaining = [j for j in range(N) if not captured[j]]
                if not remaining:
                    break
                target_j = min(remaining, key=lambda j: np.linalg.norm(pursuers[i] - evaders[j]))
                assignment[i] = target_j

            v_cmd = dive_speed(pursuers[i], evaders[target_j], V_PURSUER)
            direction = evaders[target_j] - pursuers[i]
            nd = np.linalg.norm(direction)
            if nd > 1e-8:
                pursuers[i] = pursuers[i] + v_cmd * DT * direction / nd
            pursuers[i, 2] = np.clip(pursuers[i, 2], Z_MIN, Z_MAX)

            if np.linalg.norm(pursuers[i] - evaders[target_j]) < CAPTURE_R:
                if not captured[target_j]:
                    captured[target_j] = True
                    capture_times[target_j] = t + DT

        traj_p.append(pursuers.copy())
        traj_e.append(evaders.copy())

        if all(captured):
            break

    valid_times = [ct for ct in capture_times if ct is not None]
    total_time = max(valid_times) if valid_times else T_MAX
    return np.array(traj_p), np.array(traj_e), capture_times, total_time


# ── Run all 4 cases ───────────────────────────────────────────────────────────

def run_all_cases():
    pursuers_5 = np.vstack([PURSUERS, EXTRA_PURSUERS])
    cases = [
        ('3v3 Hungarian', PURSUERS, EVADERS, 'hungarian'),
        ('3v3 Greedy',    PURSUERS, EVADERS, 'greedy'),
        ('5v3 Hungarian', pursuers_5, EVADERS, 'hungarian'),
        ('5v3 Greedy',    pursuers_5, EVADERS, 'greedy'),
    ]
    results = {}
    print('S017 3D Swarm vs Swarm — Simulation Results')
    print('=' * 60)
    for case_name, pos_p, pos_e, method in cases:
        traj_p, traj_e, capture_times, total_time = simulate_swarm_3d(pos_p, pos_e, method)
        results[case_name] = {
            'traj_p': traj_p, 'traj_e': traj_e,
            'capture_times': capture_times, 'total_time': total_time,
            'pos_p_init': pos_p, 'pos_e_init': pos_e,
        }
        ct_str = ', '.join(f'{ct:.2f}s' if ct else 'X' for ct in capture_times)
        print(f'  {case_name:16s} | {method:9s} | total={total_time:.2f}s | captures=[{ct_str}]')
    return results


# ── Plot 1: Trajectories 3D — 2×2 subplots ───────────────────────────────────

def plot_trajectories_3d(results, out_dir):
    case_names = list(results.keys())
    fig = plt.figure(figsize=(18, 14))

    for idx, case_name in enumerate(case_names):
        ax = fig.add_subplot(2, 2, idx + 1, projection='3d')
        res = results[case_name]
        traj_p = res['traj_p']  # shape (T, M, 3)
        traj_e = res['traj_e']  # shape (T, N, 3)
        M = traj_p.shape[1]
        N = traj_e.shape[1]

        for i in range(M):
            ax.plot(traj_p[:, i, 0], traj_p[:, i, 1], traj_p[:, i, 2],
                    color=PURSUER_COLORS[i % len(PURSUER_COLORS)], linewidth=1.5,
                    linestyle='-', alpha=0.85, label=f'P{i}')
            ax.scatter(*traj_p[0, i], color=PURSUER_COLORS[i % len(PURSUER_COLORS)],
                       s=60, marker='D', zorder=8)

        for j in range(N):
            ax.plot(traj_e[:, j, 0], traj_e[:, j, 1], traj_e[:, j, 2],
                    color=EVADER_COLORS[j % len(EVADER_COLORS)], linewidth=1.5,
                    linestyle=':', alpha=0.85, label=f'E{j}')
            ax.scatter(*traj_e[0, j], color=EVADER_COLORS[j % len(EVADER_COLORS)],
                       s=60, marker='^', zorder=8)

        ax.set_xlim(-8, 8); ax.set_ylim(-6, 6); ax.set_zlim(0, 6)
        ax.set_xlabel('X', fontsize=7); ax.set_ylabel('Y', fontsize=7); ax.set_zlabel('Z', fontsize=7)
        ax.set_title(f'{case_name}  (total={res["total_time"]:.2f}s)', fontsize=9, fontweight='bold')
        ax.legend(fontsize=6, loc='upper left', ncol=2)
        ax.tick_params(labelsize=6)

    fig.suptitle('S017 3D Swarm vs Swarm — Trajectories (4 Cases)\n'
                 'Solid = pursuer  |  Dotted = evader  |  Diamond = pursuer start  |  Triangle = evader start',
                 fontsize=11, fontweight='bold')
    plt.tight_layout()
    path = os.path.join(out_dir, 'trajectories_3d.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


# ── Plot 2: Capture time comparison bar chart ─────────────────────────────────

def plot_capture_time_comparison(results, out_dir):
    case_names = list(results.keys())
    total_times = [results[cn]['total_time'] for cn in case_names]
    colors = ['#2a9d8f', '#e76f51', '#457b9d', '#f4a261']

    fig, ax = plt.subplots(figsize=(10, 6))
    bars = ax.bar(case_names, total_times, color=colors, alpha=0.85, edgecolor='black', linewidth=0.8)
    for bar, val in zip(bars, total_times):
        ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.2,
                f'{val:.2f}s', ha='center', va='bottom', fontsize=10, fontweight='bold')
    ax.set_ylabel('Total Mission Time (s)', fontsize=11)
    ax.set_title('S017 3D Swarm vs Swarm — Mission Time Comparison\n'
                 'All 4 cases: 3v3 and 5v3, Hungarian vs Greedy', fontsize=11, fontweight='bold')
    ax.set_ylim(0, max(total_times) * 1.2)
    ax.grid(True, alpha=0.3, axis='y')
    plt.tight_layout()
    path = os.path.join(out_dir, 'capture_time_comparison.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


# ── Plot 3: Altitude vs time — 3v3 Hungarian ─────────────────────────────────

def plot_altitude_layers(results, out_dir):
    res = results['3v3 Hungarian']
    traj_p = res['traj_p']  # (T, 3, 3)
    traj_e = res['traj_e']  # (T, 3, 3)
    T = traj_p.shape[0]
    time_arr = np.arange(T) * DT

    fig, ax = plt.subplots(figsize=(12, 6))

    for i in range(3):
        ax.plot(time_arr, traj_p[:, i, 2], color=PURSUER_COLORS[i],
                linewidth=2.0, linestyle='-', label=f'P{i}', alpha=0.9)
    for j in range(3):
        ax.plot(time_arr, traj_e[:, j, 2], color=EVADER_COLORS[j],
                linewidth=2.0, linestyle=':', label=f'E{j}', alpha=0.9)

    for z_lyr in Z_LAYERS:
        ax.axhline(z_lyr, color='gray', linestyle='--', linewidth=0.8, alpha=0.5)
        ax.text(time_arr[-1] * 0.98, z_lyr + 0.05, f'z={z_lyr}m',
                fontsize=8, color='gray', ha='right')

    ct = res['capture_times']
    for j, t_cap in enumerate(ct):
        if t_cap is not None:
            ax.axvline(t_cap, color=EVADER_COLORS[j], linestyle='-.', linewidth=1.2, alpha=0.7)
            ax.text(t_cap + 0.1, 0.6, f'E{j}\ncaptured', fontsize=7,
                    color=EVADER_COLORS[j], va='bottom')

    ax.set_xlabel('Time (s)', fontsize=11)
    ax.set_ylabel('Altitude z (m)', fontsize=11)
    ax.set_title('S017 3D Swarm vs Swarm — Altitude vs Time (3v3 Hungarian)\n'
                 'Solid = pursuer altitude  |  Dotted = evader altitude  |  Dashed lines = altitude layers',
                 fontsize=10, fontweight='bold')
    ax.legend(fontsize=9, loc='upper right', ncol=2)
    ax.set_ylim(0, 6.5)
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'altitude_layers.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


# ── Plot 4: Animation — 3v3 Hungarian ────────────────────────────────────────

def save_animation(results, out_dir):
    res = results['3v3 Hungarian']
    traj_p = res['traj_p']  # (T, 3, 3)
    traj_e = res['traj_e']  # (T, 3, 3)
    M, N = traj_p.shape[1], traj_e.shape[1]
    max_len = len(traj_p)
    step_size = 5

    fig = plt.figure(figsize=(9, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim(-8, 8); ax.set_ylim(-6, 6); ax.set_zlim(0, 6)
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')

    for z_lyr in Z_LAYERS:
        xx, yy = np.meshgrid([-8, 8], [-6, 6])
        ax.plot_surface(xx, yy, np.full_like(xx, z_lyr), alpha=0.04, color='gray')

    p_lines = [ax.plot([], [], [], color=PURSUER_COLORS[i], lw=1.5, label=f'P{i}')[0] for i in range(M)]
    e_lines = [ax.plot([], [], [], color=EVADER_COLORS[j], lw=1.5, ls=':', label=f'E{j}')[0] for j in range(N)]
    p_dots  = [ax.plot([], [], [], color=PURSUER_COLORS[i], marker='D', ms=9, ls='')[0] for i in range(M)]
    e_dots  = [ax.plot([], [], [], color=EVADER_COLORS[j],  marker='^', ms=9, ls='')[0] for j in range(N)]
    title_txt = ax.set_title('')
    ax.legend(fontsize=8, loc='upper left')

    n_frames = max_len // step_size

    def update(frame):
        si = min(frame * step_size, max_len - 1)
        for i in range(M):
            p_lines[i].set_data_3d(traj_p[:si+1, i, 0], traj_p[:si+1, i, 1], traj_p[:si+1, i, 2])
            p_dots[i].set_data_3d([traj_p[si, i, 0]], [traj_p[si, i, 1]], [traj_p[si, i, 2]])
        for j in range(N):
            e_lines[j].set_data_3d(traj_e[:si+1, j, 0], traj_e[:si+1, j, 1], traj_e[:si+1, j, 2])
            e_dots[j].set_data_3d([traj_e[si, j, 0]], [traj_e[si, j, 1]], [traj_e[si, j, 2]])
        title_txt.set_text(f'S017 3D — 3v3 Swarm (Hungarian)   t={si * DT:.2f}s')
        return p_lines + e_lines + p_dots + e_dots + [title_txt]

    ani = FuncAnimation(fig, update, frames=n_frames, interval=50, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer=PillowWriter(fps=20), dpi=100)
    plt.close()
    print(f'Saved: {path}')


# ── Main ──────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    results = run_all_cases()
    plot_trajectories_3d(results, OUTPUT_DIR)
    plot_capture_time_comparison(results, OUTPUT_DIR)
    plot_altitude_layers(results, OUTPUT_DIR)
    save_animation(results, OUTPUT_DIR)
    print('Done.')
