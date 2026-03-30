"""
S016 3D Upgrade — Airspace Defense
Usage:
    conda activate drones
    python src/01_pursuit_evasion/3d/s016_3d_airspace_defense.py
"""
import sys, os, numpy as np, matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa
from matplotlib.animation import FuncAnimation, PillowWriter
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..'))
from src.base.drone_base import DroneBase
OUTPUT_DIR = os.path.join(os.path.dirname(__file__), '..', '..', '..', 'outputs', '01_pursuit_evasion', '3d', 's016_3d_airspace_defense')
os.makedirs(OUTPUT_DIR, exist_ok=True)

# ── Parameters ────────────────────────────────────────────────────────────────
R_ZONE      = 2.0
V_DEFENDER  = 6.0
V_INTRUDER  = 4.0
SPEED_RATIO = V_DEFENDER / V_INTRUDER
PN_GAIN     = 3.0
CAPTURE_R   = 0.15
DT          = 0.02
T_MAX       = 10.0

DEFENDER_START = np.array([0.0, 0.0, 2.0])

INTRUDER_STARTS = [
    np.array([ 8.0,  0.0,  0.0]),   # +x
    np.array([-8.0,  0.0,  0.0]),   # -x
    np.array([ 0.0,  8.0,  0.0]),   # +y
    np.array([ 6.0,  6.0,  0.0]),   # diagonal
    np.array([ 0.0,  0.0,  8.0]),   # top-down dive
    np.array([ 4.0,  0.0, -4.0]),   # below-diagonal
]

INTRUDER_LABELS = [
    'I1 (+x horiz)', 'I2 (-x horiz)', 'I3 (+y horiz)',
    'I4 (diagonal)', 'I5 (top-down)', 'I6 (low-diag)',
]
INTRUDER_COLORS = ['#e63946', '#f4a261', '#2a9d8f', '#8338ec', '#06d6a0', '#fb8500']


# ── Guidance helpers ──────────────────────────────────────────────────────────

def apollonius_intercept_3d(p_D, p_I, v_D, v_I, zone_r=R_ZONE):
    """Binary-search-based Apollonius intercept point prediction in 3D."""
    d_to_zone = -p_I / (np.linalg.norm(p_I) + 1e-8)
    for t in np.linspace(0.01, 10.0, 500):
        p_I_pred = p_I + d_to_zone * v_I * t
        if np.linalg.norm(p_I_pred) < zone_r:
            t = max(t - 0.1, 0.01)
            p_I_pred = p_I + d_to_zone * v_I * t
            return p_I_pred
        if np.linalg.norm(p_D - p_I_pred) <= v_D * t:
            return p_I_pred
    return p_I  # fallback


def pure_pursuit_step_3d(p_def, p_int, speed, dt):
    d = p_int - p_def
    nd = np.linalg.norm(d)
    if nd < 1e-8:
        return p_def
    return p_def + (d / nd) * speed * dt


def pn_step_3d(p_def, p_int, v_def, v_int, N, speed, dt):
    los = p_int - p_def
    los_dist = np.linalg.norm(los) + 1e-8
    los_hat = los / los_dist
    v_rel = v_int - v_def
    los_dot = (v_rel - np.dot(v_rel, los_hat) * los_hat) / los_dist
    v_c = -np.dot(v_rel, los_hat)
    a_cmd = N * v_c * los_dot
    v_new = v_def + a_cmd * dt
    norm_v = np.linalg.norm(v_new) + 1e-8
    v_new = (v_new / norm_v) * speed
    return p_def + v_new * dt, v_new


# ── Simulation ────────────────────────────────────────────────────────────────

def simulate(p_int_0, strategy='pure_pursuit'):
    p_def = DEFENDER_START.copy().astype(float)
    p_int = p_int_0.copy().astype(float)
    v_def = np.zeros(3)
    v_int_dir = -p_int_0 / np.linalg.norm(p_int_0)
    v_int = v_int_dir * V_INTRUDER

    traj_d = [p_def.copy()]
    traj_i = [p_int.copy()]

    n_steps = int(T_MAX / DT)
    for step in range(n_steps):
        p_int = p_int + v_int * DT

        if np.linalg.norm(p_int) <= R_ZONE:
            traj_d.append(p_def.copy())
            traj_i.append(p_int.copy())
            return np.array(traj_d), np.array(traj_i), 'intruder_wins', (step + 1) * DT

        if strategy == 'pure_pursuit':
            p_def = pure_pursuit_step_3d(p_def, p_int, V_DEFENDER, DT)
        elif strategy == 'apollonius_3d':
            intercept_pt = apollonius_intercept_3d(p_def, p_int, V_DEFENDER, V_INTRUDER)
            p_def = pure_pursuit_step_3d(p_def, intercept_pt, V_DEFENDER, DT)
        else:  # proportional_nav
            p_def, v_def = pn_step_3d(p_def, p_int, v_def, v_int, PN_GAIN, V_DEFENDER, DT)

        traj_d.append(p_def.copy())
        traj_i.append(p_int.copy())

        if np.linalg.norm(p_def - p_int) < CAPTURE_R:
            return np.array(traj_d), np.array(traj_i), 'defender_wins', (step + 1) * DT

    return np.array(traj_d), np.array(traj_i), 'timeout', T_MAX


def run_simulation():
    strategies = ['pure_pursuit', 'apollonius_3d']
    results = {}
    print('S016 3D Airspace Defense — Simulation Results')
    print('=' * 60)
    for p0, label in zip(INTRUDER_STARTS, INTRUDER_LABELS):
        for strat in strategies:
            traj_d, traj_i, outcome, t_end = simulate(p0, strat)
            results[(label, strat)] = {
                'traj_d': traj_d, 'traj_i': traj_i,
                'outcome': outcome, 'time': t_end, 'p0': p0,
            }
            sym = 'D' if outcome == 'defender_wins' else ('I' if outcome == 'intruder_wins' else 'T')
            print(f'  [{sym}] {label:18s} | {strat:16s} | {outcome:16s} | t={t_end:.3f}s')
    return results


# ── Sphere mesh helper ────────────────────────────────────────────────────────

def sphere_mesh(radius, center=(0, 0, 0), n=24):
    u = np.linspace(0, 2 * np.pi, n)
    v = np.linspace(0, np.pi, n)
    X = center[0] + radius * np.outer(np.cos(u), np.sin(v))
    Y = center[1] + radius * np.outer(np.sin(u), np.sin(v))
    Z = center[2] + radius * np.outer(np.ones(n), np.cos(v))
    return X, Y, Z


# ── Plot 1: 6 subplots — one per intruder ─────────────────────────────────────

def plot_trajectories_3d(results, out_dir):
    fig = plt.figure(figsize=(18, 12))
    strategies = ['pure_pursuit', 'apollonius_3d']
    strat_ls   = ['-', '--']
    strat_lbl  = ['Pure Pursuit', 'Apollonius 3D']

    for idx, (p0, label, color) in enumerate(zip(INTRUDER_STARTS, INTRUDER_LABELS, INTRUDER_COLORS)):
        ax = fig.add_subplot(2, 3, idx + 1, projection='3d')

        # Protected zone sphere
        Xz, Yz, Zz = sphere_mesh(R_ZONE)
        ax.plot_surface(Xz, Yz, Zz, color='royalblue', alpha=0.10, linewidth=0)

        # Defender start
        ax.scatter(*DEFENDER_START, color='black', s=80, marker='D', zorder=8)

        # Intruder start
        ax.scatter(*p0, color=color, s=80, marker='^', zorder=8)

        for strat, ls, sl in zip(strategies, strat_ls, strat_lbl):
            res = results[(label, strat)]
            td, ti = res['traj_d'], res['traj_i']
            outcome = res['outcome']

            # Intruder path (once, dotted)
            if strat == 'pure_pursuit':
                ax.plot(ti[:, 0], ti[:, 1], ti[:, 2], color=color,
                        linewidth=1.2, linestyle=':', alpha=0.7)

            # Defender path
            def_color = '#e63946' if strat == 'pure_pursuit' else '#457b9d'
            ax.plot(td[:, 0], td[:, 1], td[:, 2], color=def_color,
                    linewidth=1.5, linestyle=ls, alpha=0.9,
                    label=f'Def {sl} ({outcome.replace("_"," ")})')

        lim = max(np.linalg.norm(p0), 4.0) + 1.5
        ax.set_xlim(-lim, lim); ax.set_ylim(-lim, lim); ax.set_zlim(-lim, lim)
        ax.set_xlabel('X', fontsize=7); ax.set_ylabel('Y', fontsize=7); ax.set_zlabel('Z', fontsize=7)
        ax.set_title(f'{label}', fontsize=8, fontweight='bold')
        ax.legend(fontsize=6, loc='upper left')
        ax.tick_params(labelsize=6)

    fig.suptitle('S016 3D Airspace Defense — Trajectories (6 Intruder Directions)\n'
                 'Blue sphere = protected zone  |  Triangle = intruder start  |  Diamond = defender start',
                 fontsize=10, fontweight='bold')
    plt.tight_layout()
    path = os.path.join(out_dir, 'trajectories_3d.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


# ── Plot 2: Outcome table ─────────────────────────────────────────────────────

def plot_outcome_table(results, out_dir):
    strategies = ['pure_pursuit', 'apollonius_3d']
    strat_labels = ['Pure Pursuit', 'Apollonius 3D']

    n_int = len(INTRUDER_LABELS)
    n_str = len(strategies)

    outcome_grid = np.empty((n_int, n_str), dtype=object)
    time_grid    = np.zeros((n_int, n_str))
    miss_grid    = np.zeros((n_int, n_str))

    for i, label in enumerate(INTRUDER_LABELS):
        for j, strat in enumerate(strategies):
            res = results[(label, strat)]
            outcome_grid[i, j] = res['outcome']
            time_grid[i, j]    = res['time']
            td, ti = res['traj_d'], res['traj_i']
            n = min(len(td), len(ti))
            dists = np.linalg.norm(td[:n] - ti[:n], axis=1)
            miss_grid[i, j] = float(np.min(dists))

    fig, ax = plt.subplots(figsize=(12, 5))
    colors_map = {'defender_wins': '#2a9d8f', 'intruder_wins': '#e63946', 'timeout': '#adb5bd'}
    cell_colors = [[colors_map[outcome_grid[i, j]] for j in range(n_str)] for i in range(n_int)]
    cell_text   = [[f'{outcome_grid[i,j].replace("_"," ")}\nt={time_grid[i,j]:.2f}s\nmiss={miss_grid[i,j]:.3f}m'
                    for j in range(n_str)] for i in range(n_int)]

    table = ax.table(
        cellText=cell_text,
        rowLabels=INTRUDER_LABELS,
        colLabels=strat_labels,
        cellColours=cell_colors,
        loc='center', cellLoc='center',
    )
    table.auto_set_font_size(False)
    table.set_fontsize(8)
    table.scale(1.6, 3.2)
    ax.axis('off')
    ax.set_title('S016 3D Airspace Defense — Outcome Table\n'
                 'Green = defender wins  |  Red = intruder wins  |  Grey = timeout',
                 fontsize=10, fontweight='bold')
    plt.tight_layout()
    path = os.path.join(out_dir, 'outcome_table.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


# ── Plot 3: Miss distance bar chart ───────────────────────────────────────────

def plot_miss_distance(results, out_dir):
    strategies = ['pure_pursuit', 'apollonius_3d']
    strat_labels = ['Pure Pursuit', 'Apollonius 3D']
    colors_bars  = ['#457b9d', '#e9c46a']

    n_int = len(INTRUDER_LABELS)
    miss_data = np.zeros((n_int, 2))

    for i, label in enumerate(INTRUDER_LABELS):
        for j, strat in enumerate(strategies):
            res = results[(label, strat)]
            td, ti = res['traj_d'], res['traj_i']
            n = min(len(td), len(ti))
            dists = np.linalg.norm(td[:n] - ti[:n], axis=1)
            miss_data[i, j] = float(np.min(dists))

    x = np.arange(n_int)
    w = 0.35
    fig, ax = plt.subplots(figsize=(12, 6))
    for j, (sl, bc) in enumerate(zip(strat_labels, colors_bars)):
        bars = ax.bar(x + (j - 0.5) * w, miss_data[:, j], w, label=sl, color=bc, alpha=0.85)
        for bar in bars:
            h = bar.get_height()
            ax.text(bar.get_x() + bar.get_width() / 2, h + 0.01, f'{h:.3f}',
                    ha='center', va='bottom', fontsize=7)

    ax.axhline(CAPTURE_R, color='red', linestyle='--', linewidth=1.4,
               label=f'Capture radius ({CAPTURE_R} m)')
    ax.set_xticks(x)
    ax.set_xticklabels(INTRUDER_LABELS, fontsize=8, rotation=15, ha='right')
    ax.set_ylabel('Minimum Miss Distance (m)')
    ax.set_title('S016 3D Airspace Defense — Miss Distance per Engagement\n'
                 'Values below red dashed line = successful capture', fontsize=10, fontweight='bold')
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3, axis='y')
    plt.tight_layout()
    path = os.path.join(out_dir, 'miss_distance.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


# ── Plot 4: Animation — top-down dive intruder (I5) ──────────────────────────

def save_animation(results, out_dir):
    # Animate the top-down dive engagement (most interesting — I5)
    label = 'I5 (top-down)'
    strat = 'apollonius_3d'
    res = results[(label, strat)]
    td, ti = res['traj_d'], res['traj_i']
    max_len = max(len(td), len(ti))
    step_size = 4

    fig = plt.figure(figsize=(9, 8))
    ax = fig.add_subplot(111, projection='3d')

    # Protected zone
    Xz, Yz, Zz = sphere_mesh(R_ZONE)
    ax.plot_wireframe(Xz, Yz, Zz, color='royalblue', alpha=0.15, linewidth=0.5)

    lim = 9.0
    ax.set_xlim(-lim, lim); ax.set_ylim(-lim, lim); ax.set_zlim(-lim, lim)
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')

    # Starting markers
    ax.scatter(*DEFENDER_START, color='black', s=100, marker='D', zorder=8)
    p0 = INTRUDER_STARTS[4]
    ax.scatter(*p0, color='#06d6a0', s=100, marker='^', zorder=8)

    int_line, = ax.plot([], [], [], color='#06d6a0', linewidth=1.5, linestyle=':', alpha=0.8, label='Intruder')
    def_line, = ax.plot([], [], [], color='#e63946', linewidth=1.8, linestyle='-', alpha=0.9, label='Defender')
    int_dot,  = ax.plot([], [], [], color='#06d6a0', marker='^', markersize=10, linestyle='', zorder=9)
    def_dot,  = ax.plot([], [], [], color='#e63946', marker='o', markersize=10, linestyle='',
                         zorder=9, markeredgecolor='black', markeredgewidth=0.8)
    title_txt = ax.set_title('')
    ax.legend(fontsize=8, loc='upper left')

    n_frames = max_len // step_size

    def update(frame):
        si = min(frame * step_size, max_len - 1)
        ni = min(si + 1, len(ti))
        nd = min(si + 1, len(td))
        int_line.set_data_3d(ti[:ni, 0], ti[:ni, 1], ti[:ni, 2])
        def_line.set_data_3d(td[:nd, 0], td[:nd, 1], td[:nd, 2])
        int_dot.set_data_3d([ti[ni - 1, 0]], [ti[ni - 1, 1]], [ti[ni - 1, 2]])
        def_dot.set_data_3d([td[nd - 1, 0]], [td[nd - 1, 1]], [td[nd - 1, 2]])
        title_txt.set_text(f'S016 3D — Top-Down Dive (Apollonius)   t={si * DT:.2f}s')
        return int_line, def_line, int_dot, def_dot, title_txt

    ani = FuncAnimation(fig, update, frames=n_frames, interval=50, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer=PillowWriter(fps=20), dpi=100)
    plt.close()
    print(f'Saved: {path}')


# ── Main ──────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    results = run_simulation()

    strategies = ['pure_pursuit', 'apollonius_3d']
    for strat in strategies:
        wins = sum(1 for (l, s), r in results.items()
                   if s == strat and r['outcome'] == 'defender_wins')
        print(f'  {strat}: {wins}/{len(INTRUDER_STARTS)} defender wins')

    plot_trajectories_3d(results, OUTPUT_DIR)
    plot_outcome_table(results, OUTPUT_DIR)
    plot_miss_distance(results, OUTPUT_DIR)
    save_animation(results, OUTPUT_DIR)
    print('Done.')
