"""
S011 3D Upgrade — Swarm Encirclement
Usage:
    conda activate drones
    python src/01_pursuit_evasion/3d/s011_3d_swarm_encirclement.py
"""
import sys, os, numpy as np, matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa
from matplotlib.animation import FuncAnimation, PillowWriter
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..'))
from src.base.drone_base import DroneBase

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), '..', '..', '..',
                          'outputs', '01_pursuit_evasion', '3d', 's011_3d_swarm_encirclement')
os.makedirs(OUTPUT_DIR, exist_ok=True)

# ── Parameters ───────────────────────────────────────────────
N_PURSUERS  = 6
R_INIT      = 4.0    # m
T_CONVERGE  = 15.0   # s
R_CAPTURE   = 0.3    # m
V_PURSUER   = 5.0    # m/s
V_EVADER    = 3.5    # m/s
Z_MIN, Z_MAX = 0.5, 6.0
DT          = 0.05   # s
MAX_TIME    = 30.0   # s
K_BREAKOUT  = 200    # random candidates for evader direction

PHI_GOLDEN = (1 + np.sqrt(5)) / 2


# ── Fibonacci sphere ─────────────────────────────────────────

def fibonacci_sphere(n):
    """Return n unit vectors approximately uniformly on S²."""
    pts = []
    for i in range(n):
        y = 1 - 2 * i / max(n - 1, 1)
        r = np.sqrt(max(1 - y * y, 0))
        phi = i * 2 * np.pi / PHI_GOLDEN ** 2
        pts.append(np.array([r * np.cos(phi), r * np.sin(phi), y]))
    return np.array(pts)


# ── Radius schedule ──────────────────────────────────────────

def shrink_radius(t, r0=R_INIT, r_min=R_CAPTURE, t_conv=T_CONVERGE):
    v_shrink = (r0 - r_min) / t_conv
    return max(r0 - v_shrink * t, r_min)


# ── Evader breakout strategy ──────────────────────────────────

def evader_breakout_3d(pos_e, pursuer_positions, v_max, k=K_BREAKOUT, rng=None):
    """Find direction on S² maximising min angular distance from all pursuers."""
    if rng is None:
        rng = np.random.default_rng()
    dirs_to_p = pursuer_positions - pos_e[np.newaxis, :]
    norms = np.linalg.norm(dirs_to_p, axis=1, keepdims=True) + 1e-8
    dirs_to_p = dirs_to_p / norms

    # Sample K random unit vectors
    u = rng.standard_normal((k, 3))
    u /= (np.linalg.norm(u, axis=1, keepdims=True) + 1e-10)

    dots = np.clip(u @ dirs_to_p.T, -1.0, 1.0)   # (K, N)
    min_angles = np.arccos(dots).min(axis=1)       # (K,)
    best = int(np.argmax(min_angles))
    return v_max * u[best]


# ── Main simulation ──────────────────────────────────────────

def run_simulation(n_pursuers=N_PURSUERS, seed=42):
    rng = np.random.default_rng(seed)
    sphere_dirs = fibonacci_sphere(n_pursuers)

    pos_e = np.array([0.0, 0.0, 2.0])
    pos_p = pos_e[np.newaxis, :] + R_INIT * sphere_dirs.copy()
    pos_p[:, 2] = np.clip(pos_p[:, 2], Z_MIN, Z_MAX)

    traj_e = [pos_e.copy()]
    traj_p = [pos_p.copy()]
    r_hist = [R_INIT]
    t_hist = [0.0]

    captured = False
    cap_time = None
    cap_who  = None

    max_steps = int(MAX_TIME / DT)
    for step in range(1, max_steps + 1):
        t = step * DT
        R = shrink_radius(t)

        targets = pos_e[np.newaxis, :] + R * sphere_dirs
        targets[:, 2] = np.clip(targets[:, 2], Z_MIN, Z_MAX)

        for i in range(n_pursuers):
            d = targets[i] - pos_p[i]
            dn = np.linalg.norm(d)
            step_v = V_PURSUER * d / dn if dn > 1e-8 else np.zeros(3)
            pos_p[i] += step_v * DT
            pos_p[i, 2] = np.clip(pos_p[i, 2], Z_MIN, Z_MAX)

        v_e = evader_breakout_3d(pos_e, pos_p, V_EVADER, rng=rng)
        pos_e = pos_e + v_e * DT
        pos_e[2] = np.clip(pos_e[2], Z_MIN, Z_MAX)

        traj_e.append(pos_e.copy())
        traj_p.append(pos_p.copy())
        r_hist.append(R)
        t_hist.append(t)

        dists = np.linalg.norm(pos_p - pos_e[np.newaxis, :], axis=1)
        if dists.min() < R_CAPTURE:
            captured = True
            cap_time = t
            cap_who  = int(np.argmin(dists))
            break

    return (np.array(traj_e), np.array(traj_p),
            np.array(r_hist), np.array(t_hist),
            captured, cap_time, cap_who)


# ── Plot 1: 3D trajectories ───────────────────────────────────

def plot_trajectories_3d(traj_e, traj_p, captured, cap_time, cap_who, out_dir):
    n = traj_p.shape[1]
    cmap = plt.cm.Reds
    p_colors = [cmap(0.4 + 0.5 * i / max(n - 1, 1)) for i in range(n)]

    fig = plt.figure(figsize=(10, 8))
    ax  = fig.add_subplot(111, projection='3d')
    ax.view_init(elev=25, azim=-55)

    for i in range(n):
        ax.plot(traj_p[:, i, 0], traj_p[:, i, 1], traj_p[:, i, 2],
                color=p_colors[i], linewidth=1.4, alpha=0.85, label=f'P{i+1}')
        ax.scatter(*traj_p[0, i], color=p_colors[i], s=40, marker='o')

    ax.plot(traj_e[:, 0], traj_e[:, 1], traj_e[:, 2],
            color='royalblue', linewidth=2.0, label='Evader', zorder=5)
    ax.scatter(*traj_e[0], color='blue', s=60, marker='o', zorder=6)

    if captured:
        ax.scatter(*traj_e[-1], color='black', s=120, marker='X', zorder=7,
                   label=f'Captured by P{cap_who+1} @ {cap_time:.2f}s')

    status = f'Captured @ {cap_time:.2f} s' if captured else 'Timeout — evader escaped'
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
    ax.set_title(f'S011 3D Swarm Encirclement — N={n} Pursuers\n{status}', fontsize=10)
    ax.legend(fontsize=7, loc='upper left', ncol=2)
    plt.tight_layout()
    path = os.path.join(out_dir, 'trajectories_3d.png')
    plt.savefig(path, dpi=120, bbox_inches='tight')
    plt.close('all')
    print(f'Saved: {path}')


# ── Plot 2: Altitude vs time ──────────────────────────────────

def plot_altitude_time(traj_e, traj_p, t_hist, out_dir):
    n = traj_p.shape[1]
    cmap = plt.cm.Reds
    p_colors = [cmap(0.4 + 0.5 * i / max(n - 1, 1)) for i in range(n)]

    fig, ax = plt.subplots(figsize=(11, 5))
    for i in range(n):
        ax.plot(t_hist, traj_p[:, i, 2], color=p_colors[i],
                linewidth=1.4, alpha=0.85, label=f'P{i+1}')
    ax.plot(t_hist, traj_e[:, 2], color='royalblue', linewidth=2.0,
            label='Evader', zorder=5)
    ax.axhline(Z_MIN, color='grey', linestyle='--', linewidth=0.8, alpha=0.6, label='z bounds')
    ax.axhline(Z_MAX, color='grey', linestyle='--', linewidth=0.8, alpha=0.6)
    ax.set_xlabel('Time (s)'); ax.set_ylabel('Altitude z (m)')
    ax.set_title('S011 3D Swarm Encirclement — Altitude vs Time (N=6)', fontsize=10)
    ax.legend(fontsize=8, ncol=4); ax.grid(True, alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'altitude_time.png')
    plt.savefig(path, dpi=120, bbox_inches='tight')
    plt.close('all')
    print(f'Saved: {path}')


# ── Plot 3: Radius vs time ────────────────────────────────────

def plot_radius_time(r_hist, t_hist, cap_time, out_dir):
    fig, ax = plt.subplots(figsize=(9, 4))
    ax.plot(t_hist, r_hist, color='steelblue', linewidth=2.0, label='R(t)')
    ax.axhline(R_CAPTURE, color='red', linestyle='--', linewidth=1.2,
               label=f'R_capture = {R_CAPTURE} m')
    if cap_time:
        ax.axvline(cap_time, color='black', linestyle='--', linewidth=1.2,
                   label=f'Captured @ {cap_time:.2f}s')
    ax.set_xlabel('Time (s)'); ax.set_ylabel('Encirclement Radius (m)')
    ax.set_title('S011 3D Swarm Encirclement — Shrinking Sphere Radius R(t)')
    ax.legend(fontsize=9); ax.grid(True, alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'radius_time.png')
    plt.savefig(path, dpi=120, bbox_inches='tight')
    plt.close('all')
    print(f'Saved: {path}')


# ── Plot 4: Breakout vs N sweep ───────────────────────────────

def plot_breakout_vs_N(out_dir, n_seeds=10):
    N_list = [4, 6, 8, 10, 12]
    results = {}   # N -> list of (cap_time or None)

    for n in N_list:
        cap_times = []
        for seed in range(n_seeds):
            _, _, _, _, captured, cap_time, _ = run_simulation(n_pursuers=n, seed=seed)
            cap_times.append(cap_time if captured else None)
        results[n] = cap_times

    fig, ax = plt.subplots(figsize=(10, 5))
    colors = plt.cm.viridis(np.linspace(0.15, 0.85, len(N_list)))

    for idx, n in enumerate(N_list):
        vals = results[n]
        for s, v in enumerate(vals):
            if v is not None:
                ax.scatter(n, v, color=colors[idx], s=60, alpha=0.8, zorder=4)
            else:
                ax.scatter(n, MAX_TIME + 1.5, color=colors[idx], s=80,
                           marker='^', alpha=0.8, zorder=4)

        captured_ct = [v for v in vals if v is not None]
        escaped_ct  = [v for v in vals if v is None]
        if captured_ct:
            ax.plot([n, n], [min(captured_ct), max(captured_ct)],
                    color=colors[idx], linewidth=2.0, alpha=0.6)

    ax.axhline(MAX_TIME + 1.5, color='grey', linestyle='--',
               linewidth=0.8, alpha=0.5, label=f'Escape (timeout > {MAX_TIME}s)')
    ax.set_xlabel('Number of Pursuers N'); ax.set_ylabel('Capture Time (s)')
    ax.set_title('S011 3D Swarm Encirclement — Breakout Success vs N\n'
                 f'({n_seeds} random seeds; triangle = escape)', fontsize=10)
    ax.set_xticks(N_list)
    ax.set_ylim(0, MAX_TIME + 4)
    ax.grid(True, alpha=0.3)

    # Summary text
    for n in N_list:
        n_escape = sum(1 for v in results[n] if v is None)
        ax.text(n, -1.5, f'{n_escape}/{n_seeds}\nesc', ha='center',
                fontsize=7, color='darkred')

    plt.tight_layout()
    path = os.path.join(out_dir, 'breakout_vs_N.png')
    plt.savefig(path, dpi=120, bbox_inches='tight')
    plt.close('all')
    print(f'Saved: {path}')


# ── Animation ─────────────────────────────────────────────────

def save_animation(traj_e, traj_p, r_hist, t_hist, captured, cap_time, out_dir):
    n = traj_p.shape[1]
    cmap = plt.cm.Reds
    p_colors = [cmap(0.4 + 0.5 * i / max(n - 1, 1)) for i in range(n)]

    T = len(t_hist)
    step_size = max(1, T // 200)   # target ~200 frames

    fig = plt.figure(figsize=(9, 7))
    ax  = fig.add_subplot(111, projection='3d')
    ax.view_init(elev=25, azim=-55)

    # Axis limits from full trajectory
    all_pts = np.vstack([traj_e] + [traj_p[:, i] for i in range(n)])
    lo = all_pts.min(axis=0) - 0.5
    hi = all_pts.max(axis=0) + 0.5
    ax.set_xlim(lo[0], hi[0]); ax.set_ylim(lo[1], hi[1]); ax.set_zlim(lo[2], hi[2])
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')

    lines_p  = [ax.plot([], [], [], color=c, linewidth=1.2, alpha=0.7)[0] for c in p_colors]
    dots_p   = [ax.plot([], [], [], '^', color=c, markersize=8, zorder=6)[0] for c in p_colors]
    line_e,  = ax.plot([], [], [], color='royalblue', linewidth=1.8)
    dot_e,   = ax.plot([], [], [], 'bs', markersize=10, zorder=7)
    title    = ax.set_title('')

    frames = list(range(0, T, step_size))
    if frames[-1] != T - 1:
        frames.append(T - 1)

    def update(frame_idx):
        si = frames[frame_idx]
        t  = t_hist[si]
        R  = r_hist[si]

        for i in range(n):
            lines_p[i].set_data(traj_p[:si+1, i, 0], traj_p[:si+1, i, 1])
            lines_p[i].set_3d_properties(traj_p[:si+1, i, 2])
            dots_p[i].set_data([traj_p[si, i, 0]], [traj_p[si, i, 1]])
            dots_p[i].set_3d_properties([traj_p[si, i, 2]])

        line_e.set_data(traj_e[:si+1, 0], traj_e[:si+1, 1])
        line_e.set_3d_properties(traj_e[:si+1, 2])
        dot_e.set_data([traj_e[si, 0]], [traj_e[si, 1]])
        dot_e.set_3d_properties([traj_e[si, 2]])

        done = (frame_idx == len(frames) - 1)
        if done and captured:
            title.set_text(f'S011 3D Swarm Encirclement  t={t:.2f}s  Captured! [DONE]')
        else:
            title.set_text(f'S011 3D Swarm Encirclement  t={t:.2f}s  R={R:.2f}m')
        return lines_p + dots_p + [line_e, dot_e, title]

    ani = FuncAnimation(fig, update, frames=len(frames), interval=60, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer=PillowWriter(fps=15))
    plt.close('all')
    print(f'Saved: {path}')


# ── Main ─────────────────────────────────────────────────────

if __name__ == '__main__':
    print('Running S011 3D Swarm Encirclement (N=6)...')
    traj_e, traj_p, r_hist, t_hist, captured, cap_time, cap_who = run_simulation()
    status = f'Captured by P{cap_who+1} @ {cap_time:.2f}s' if captured else 'Evader escaped (timeout)'
    print(f'Result: {status}')

    plot_trajectories_3d(traj_e, traj_p, captured, cap_time, cap_who, OUTPUT_DIR)
    plot_altitude_time(traj_e, traj_p, t_hist, OUTPUT_DIR)
    plot_radius_time(r_hist, t_hist, cap_time, OUTPUT_DIR)

    print('Running breakout vs N sweep (N=4,6,8,10,12, 10 seeds each)...')
    plot_breakout_vs_N(OUTPUT_DIR, n_seeds=10)

    print('Saving animation...')
    save_animation(traj_e, traj_p, r_hist, t_hist, captured, cap_time, OUTPUT_DIR)

    print('All outputs saved to:', OUTPUT_DIR)
