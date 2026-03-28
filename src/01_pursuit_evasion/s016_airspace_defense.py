"""
S016 Airspace Defense
======================
A protected zone (sphere of radius 2 m at the origin) must be defended against
intruders that fly straight toward its center.  A single defender starts near the
zone boundary and attempts to intercept each intruder before it crosses the
zone surface.  Two guidance strategies are compared: Pure Pursuit and
Proportional Navigation (PN, N=3).  The Apollonius guarantee region is
visualized as the analytical boundary inside which the defender is certain to
intercept any straight-flying intruder.

Usage:
    conda activate drones
    python src/01_pursuit_evasion/s016_airspace_defense.py
"""

import sys, os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib.animation as animation

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

# ── Parameters ─────────────────────────────────────────────────────────────
R_ZONE      = 2.0                        # m  — protected zone radius
DEFENDER_START = np.array([0.0, 0.0, 2.0])  # m  — defender initial position
V_DEFENDER  = 6.0                        # m/s — defender speed
V_INTRUDER  = 4.0                        # m/s — intruder speed
SPEED_RATIO = V_DEFENDER / V_INTRUDER    # 1.5
PN_GAIN     = 3.0                        # —   — proportional navigation gain N
CAPTURE_R   = 0.15                       # m  — capture radius
DT          = 0.02                       # s  — timestep
T_MAX       = 10.0                       # s  — max simulation time

INTRUDER_STARTS = [
    np.array([ 8.0,  0.0, 2.0]),
    np.array([-8.0,  0.0, 2.0]),
    np.array([ 0.0,  8.0, 2.0]),
    np.array([ 6.0,  6.0, 2.0]),
]

INTRUDER_LABELS = ['I1 (+x)', 'I2 (-x)', 'I3 (+y)', 'I4 (+x+y)']
INTRUDER_COLORS = ['#e63946', '#f4a261', '#2a9d8f', '#8338ec']
STRATEGY_LINESTYLES = {'pure_pursuit': '-', 'proportional_nav': '--'}

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '01_pursuit_evasion', 's016_airspace_defense',
)

RNG = np.random.default_rng(0)


# ── Helpers ─────────────────────────────────────────────────────────────────

def sphere_mesh(radius, center=(0, 0, 0), n=30):
    """Return (X, Y, Z) mesh arrays for a sphere surface."""
    u = np.linspace(0, 2 * np.pi, n)
    v = np.linspace(0, np.pi, n)
    X = center[0] + radius * np.outer(np.cos(u), np.sin(v))
    Y = center[1] + radius * np.outer(np.sin(u), np.sin(v))
    Z = center[2] + radius * np.outer(np.ones(n), np.cos(v))
    return X, Y, Z


def apollonius_radius(p_intruder, r_zone=R_ZONE, ratio=SPEED_RATIO):
    """
    Apollonius intercept condition:
        |p_D - p_I| <= ratio * (|p_I| - r_zone)
    Returns the intercept-guarantee distance threshold for this intruder position.
    """
    dist_to_center = np.linalg.norm(p_intruder)
    return ratio * max(dist_to_center - r_zone, 0.0)


# ── Guidance laws ────────────────────────────────────────────────────────────

def pure_pursuit_step(p_def, p_int, speed, dt):
    """Move defender directly toward current intruder position."""
    d = p_int - p_def
    norm_d = np.linalg.norm(d)
    if norm_d < 1e-8:
        return p_def
    return p_def + (d / norm_d) * speed * dt


def pn_step(p_def, p_int, v_def, v_int, N, speed, dt):
    """
    Proportional Navigation:
        a_cmd = N * v_c * lambda_dot
    Integrate acceleration to get new velocity, clip to max speed.
    """
    los = p_int - p_def
    los_dist = np.linalg.norm(los) + 1e-8
    los_hat = los / los_dist

    v_rel = v_int - v_def
    # LOS rate: perpendicular component of v_rel / range
    los_dot = (v_rel - np.dot(v_rel, los_hat) * los_hat) / los_dist
    # Closing speed (positive when approaching)
    v_c = -np.dot(v_rel, los_hat)

    a_cmd = N * v_c * los_dot
    v_new = v_def + a_cmd * dt
    norm_v = np.linalg.norm(v_new) + 1e-8
    v_new = (v_new / norm_v) * speed
    return p_def + v_new * dt, v_new


# ── Simulation ───────────────────────────────────────────────────────────────

def simulate(p_int_0, strategy='pure_pursuit'):
    """
    Simulate one defender vs one intruder engagement.
    Returns (traj_defender, traj_intruder, outcome, capture_time).
    """
    p_def = DEFENDER_START.copy().astype(float)
    p_int = p_int_0.copy().astype(float)
    v_def = np.zeros(3)
    # Intruder flies straight toward zone center
    v_int = -p_int_0 / np.linalg.norm(p_int_0) * V_INTRUDER

    traj_d = [p_def.copy()]
    traj_i = [p_int.copy()]

    n_steps = int(T_MAX / DT)
    for step in range(n_steps):
        # Move intruder first
        p_int = p_int + v_int * DT

        # Check penetration
        if np.linalg.norm(p_int) <= R_ZONE:
            traj_d.append(p_def.copy())
            traj_i.append(p_int.copy())
            return np.array(traj_d), np.array(traj_i), 'intruder_wins', (step + 1) * DT

        # Move defender
        if strategy == 'pure_pursuit':
            p_def = pure_pursuit_step(p_def, p_int, V_DEFENDER, DT)
        else:
            p_def, v_def = pn_step(p_def, p_int, v_def, v_int, PN_GAIN, V_DEFENDER, DT)

        traj_d.append(p_def.copy())
        traj_i.append(p_int.copy())

        # Check capture
        if np.linalg.norm(p_def - p_int) < CAPTURE_R:
            return np.array(traj_d), np.array(traj_i), 'defender_wins', (step + 1) * DT

    return np.array(traj_d), np.array(traj_i), 'timeout', T_MAX


def run_simulation():
    """Run all 8 engagements (4 intruder starts × 2 strategies)."""
    results = {}
    for p0, label in zip(INTRUDER_STARTS, INTRUDER_LABELS):
        for strat in ['pure_pursuit', 'proportional_nav']:
            traj_d, traj_i, outcome, t_end = simulate(p0, strat)
            results[(label, strat)] = {
                'traj_d': traj_d,
                'traj_i': traj_i,
                'outcome': outcome,
                'time':    t_end,
                'p0':      p0,
            }
            sym = 'D' if outcome == 'defender_wins' else ('I' if outcome == 'intruder_wins' else 'T')
            print(f'  [{sym}] {label:12s} | {strat:18s} | {outcome:16s} | t={t_end:.3f}s')

    # Apollonius check — compare theoretical vs simulated
    print('\nApollonius guarantee check:')
    for p0, label in zip(INTRUDER_STARTS, INTRUDER_LABELS):
        apo_r = apollonius_radius(p0)
        dist_def = np.linalg.norm(DEFENDER_START - p0)
        can_guarantee = dist_def <= apo_r
        print(f'  {label:12s} | |D-I|={dist_def:.2f}m  apo_threshold={apo_r:.2f}m'
              f'  → {"GUARANTEE" if can_guarantee else "UNCERTAIN"}')

    return results


# ── Plots ────────────────────────────────────────────────────────────────────

def plot_3d_trajectories(results, out_dir):
    """3D trajectory plot — protected zone + Apollonius sphere + all tracks."""
    fig = plt.figure(figsize=(14, 10))
    ax = fig.add_subplot(111, projection='3d')

    # Protected zone (transparent blue sphere)
    Xz, Yz, Zz = sphere_mesh(R_ZONE)
    ax.plot_surface(Xz, Yz, Zz, color='royalblue', alpha=0.12, linewidth=0)

    # Apollonius sphere for first intruder as representative illustration
    # The Apollonius surface is centred NOT at the origin but depends on geometry.
    # For a straight approach from (8,0,2), the analytically tractable surface is a
    # sphere of radius r_apo around the intruder start — we draw it as a hint.
    p0_ref = INTRUDER_STARTS[0]
    apo_r_ref = apollonius_radius(p0_ref)
    Xa, Ya, Za = sphere_mesh(apo_r_ref, center=tuple(p0_ref), n=20)
    ax.plot_surface(Xa, Ya, Za, color='gold', alpha=0.08, linewidth=0)

    for (label, strat), res in results.items():
        idx = INTRUDER_LABELS.index(label)
        color = INTRUDER_COLORS[idx]
        ls    = STRATEGY_LINESTYLES[strat]
        td, ti = res['traj_d'], res['traj_i']

        # Intruder path (only once per intruder — use pure_pursuit pass)
        if strat == 'pure_pursuit':
            ax.plot(ti[:, 0], ti[:, 1], ti[:, 2],
                    color=color, linewidth=1.6, linestyle=':', alpha=0.8,
                    label=f'{label} intruder')
            ax.scatter(*res['p0'], color=color, s=80, marker='^', zorder=6)

        # Defender path
        strat_label = 'PP' if strat == 'pure_pursuit' else 'PN'
        ax.plot(td[:, 0], td[:, 1], td[:, 2],
                color=color, linewidth=1.4, linestyle=ls, alpha=0.9,
                label=f'{label} defender ({strat_label})')

    # Defender start
    ax.scatter(*DEFENDER_START, color='black', s=120, marker='D', zorder=8, label='Defender start')
    ax.scatter(0, 0, 0, color='royalblue', s=100, marker='o', zorder=8, label='Zone center')

    # Axis formatting
    all_pts = np.concatenate([res['traj_d'] for res in results.values()] +
                             [res['traj_i'] for res in results.values()])
    margin = 1.5
    lim = max(np.abs(all_pts).max(), R_ZONE) + margin
    ax.set_xlim(-lim, lim); ax.set_ylim(-lim, lim); ax.set_zlim(-lim, lim)
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
    ax.set_title('S016 Airspace Defense — 3D Trajectories\n'
                 'Dotted = intruder, Solid/Dashed = defender PP / PN\n'
                 'Blue sphere = protected zone  Gold sphere = Apollonius region (I1 ref)', fontsize=9)
    ax.legend(fontsize=7, loc='upper left', ncol=2)
    plt.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'trajectories_3d.png')
    plt.savefig(path, dpi=150); plt.close(); print(f'Saved: {path}')


def plot_outcome_table(results, out_dir):
    """Outcome summary table as a Matplotlib figure."""
    strategies = ['pure_pursuit', 'proportional_nav']
    strat_labels = ['Pure Pursuit', 'Prop. Nav.']

    n_int = len(INTRUDER_LABELS)
    n_str = len(strategies)

    outcome_grid  = np.empty((n_int, n_str), dtype=object)
    time_grid     = np.zeros((n_int, n_str))
    miss_grid     = np.zeros((n_int, n_str))

    for i, label in enumerate(INTRUDER_LABELS):
        for j, strat in enumerate(strategies):
            res = results[(label, strat)]
            outcome_grid[i, j] = res['outcome']
            time_grid[i, j]    = res['time']
            td, ti = res['traj_d'], res['traj_i']
            n = min(len(td), len(ti))
            dists = np.linalg.norm(td[:n] - ti[:n], axis=1)
            miss_grid[i, j] = float(np.min(dists))

    fig, axes = plt.subplots(1, 2, figsize=(13, 4))

    # Left: outcome + time heatmap
    ax = axes[0]
    colors_map = {'defender_wins': '#2a9d8f', 'intruder_wins': '#e63946', 'timeout': '#adb5bd'}
    cell_colors = [[colors_map[outcome_grid[i, j]] for j in range(n_str)] for i in range(n_int)]
    cell_text   = [[f'{outcome_grid[i,j].replace("_"," ")}\nt={time_grid[i,j]:.2f}s'
                    for j in range(n_str)] for i in range(n_int)]

    table = ax.table(
        cellText=cell_text,
        rowLabels=INTRUDER_LABELS,
        colLabels=strat_labels,
        cellColours=cell_colors,
        loc='center', cellLoc='center',
    )
    table.auto_set_font_size(False); table.set_fontsize(9)
    table.scale(1.6, 2.8)
    ax.axis('off')
    ax.set_title('Outcome & Intercept Time', fontsize=10, fontweight='bold')

    # Right: miss distance bar chart
    ax2 = axes[1]
    x = np.arange(n_int)
    w = 0.35
    bars_pp = ax2.bar(x - w/2, miss_grid[:, 0], w, label='Pure Pursuit', color='#457b9d', alpha=0.85)
    bars_pn = ax2.bar(x + w/2, miss_grid[:, 1], w, label='Prop. Nav.',   color='#e9c46a', alpha=0.85)
    ax2.axhline(CAPTURE_R, color='red', linestyle='--', linewidth=1.2, label=f'Capture radius ({CAPTURE_R} m)')
    ax2.set_xticks(x); ax2.set_xticklabels(INTRUDER_LABELS, fontsize=8)
    ax2.set_ylabel('Min Miss Distance (m)'); ax2.set_xlabel('Intruder Start')
    ax2.set_title('Minimum Miss Distance per Engagement', fontsize=10, fontweight='bold')
    ax2.legend(fontsize=8); ax2.grid(True, alpha=0.3, axis='y')
    # Annotate bars
    for bar in list(bars_pp) + list(bars_pn):
        h = bar.get_height()
        ax2.text(bar.get_x() + bar.get_width()/2, h + 0.01, f'{h:.3f}',
                 ha='center', va='bottom', fontsize=7)

    plt.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'outcome_table.png')
    plt.savefig(path, dpi=150); plt.close(); print(f'Saved: {path}')


def plot_apollonius(results, out_dir):
    """
    Top-down 2D view showing the Apollonius intercept circles for each
    intruder start position, overlaid with the protected zone.
    """
    fig, ax = plt.subplots(figsize=(10, 10))

    # Protected zone circle
    theta = np.linspace(0, 2 * np.pi, 360)
    ax.fill(R_ZONE * np.cos(theta), R_ZONE * np.sin(theta),
            color='royalblue', alpha=0.18, label='Protected zone')
    ax.plot(R_ZONE * np.cos(theta), R_ZONE * np.sin(theta),
            color='royalblue', linewidth=2)

    # Defender start (projected to 2D)
    ax.scatter(*DEFENDER_START[:2], color='black', s=200, marker='D', zorder=8,
               label='Defender start')

    for p0, label, color in zip(INTRUDER_STARTS, INTRUDER_LABELS, INTRUDER_COLORS):
        # Intruder start
        ax.scatter(*p0[:2], color=color, s=120, marker='^', zorder=7)
        ax.text(p0[0]+0.2, p0[1]+0.2, label, fontsize=8, color=color)

        # Apollonius circle (radius around intruder start: defender must be inside this to guarantee)
        apo_r = apollonius_radius(p0)
        apo_circle = plt.Circle(p0[:2], apo_r,
                                fill=False, edgecolor=color,
                                linestyle='--', linewidth=1.8, alpha=0.7)
        ax.add_patch(apo_circle)
        ax.text(p0[0] + apo_r * 0.7, p0[1] + 0.1,
                f'r_apo={apo_r:.2f}m', fontsize=7, color=color, alpha=0.8)

        # Draw intruder approach arrow
        direction = -p0[:2] / np.linalg.norm(p0[:2])
        ax.annotate('', xy=p0[:2] + direction * 1.5, xytext=p0[:2],
                    arrowprops=dict(arrowstyle='->', color=color, lw=1.5))

        # Check guarantee
        dist_def = np.linalg.norm(DEFENDER_START[:2] - p0[:2])
        guaranteed = dist_def <= apo_r
        marker = 'GUARANTEED' if guaranteed else 'UNCERTAIN'
        ax.text(p0[0], p0[1] - 0.6, marker, fontsize=7, color=color,
                ha='center', fontstyle='italic')

    ax.set_aspect('equal')
    ax.set_xlim(-11, 11); ax.set_ylim(-11, 11)
    ax.grid(True, alpha=0.3)
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
    ax.set_title('S016 Airspace Defense — Apollonius Guarantee Region (Top-Down View)\n'
                 'Dashed circles = Apollonius boundary; Defender inside circle → intercept guaranteed',
                 fontsize=9)
    ax.legend(fontsize=8, loc='upper right')
    plt.tight_layout()
    path = os.path.join(out_dir, 'apollonius_region.png')
    plt.savefig(path, dpi=150); plt.close(); print(f'Saved: {path}')


def plot_miss_vs_angle(results, out_dir):
    """Scatter plot: minimum miss distance vs intruder approach angle."""
    fig, ax = plt.subplots(figsize=(9, 6))

    for strat, marker, color_shift in [('pure_pursuit', 'o', 0), ('proportional_nav', 's', 0)]:
        angles, miss_dists, colors_used = [], [], []
        for label_i, (p0, label) in enumerate(zip(INTRUDER_STARTS, INTRUDER_LABELS)):
            res = results[(label, strat)]
            td, ti = res['traj_d'], res['traj_i']
            n = min(len(td), len(ti))
            dists = np.linalg.norm(td[:n] - ti[:n], axis=1)
            miss = float(np.min(dists))

            # Approach angle = angle of intruder velocity vector in XY-plane
            v_int = -p0[:2] / np.linalg.norm(p0[:2])
            angle_deg = float(np.degrees(np.arctan2(v_int[1], v_int[0])))
            angles.append(angle_deg)
            miss_dists.append(miss)
            colors_used.append(INTRUDER_COLORS[label_i])

        strat_label = 'Pure Pursuit' if strat == 'pure_pursuit' else 'Prop. Nav.'
        for angle, miss, c in zip(angles, miss_dists, colors_used):
            ax.scatter(angle, miss, color=c, s=120, marker=marker,
                       edgecolors='black', linewidths=0.8, zorder=5)
        # Dummy scatter for legend
        ax.scatter([], [], color='grey', marker=marker, s=80,
                   edgecolors='black', linewidths=0.8, label=strat_label)

    ax.axhline(CAPTURE_R, color='red', linestyle='--', linewidth=1.4,
               label=f'Capture radius ({CAPTURE_R} m)')
    ax.axhline(0, color='black', linewidth=0.5)

    # Annotate intruder labels
    for p0, label, color in zip(INTRUDER_STARTS, INTRUDER_LABELS, INTRUDER_COLORS):
        v_int = -p0[:2] / np.linalg.norm(p0[:2])
        angle_deg = float(np.degrees(np.arctan2(v_int[1], v_int[0])))
        ax.text(angle_deg + 2, CAPTURE_R + 0.04, label, fontsize=8, color=color)

    ax.set_xlabel('Intruder Approach Angle (°)'); ax.set_ylabel('Min Miss Distance (m)')
    ax.set_title('S016 Airspace Defense — Miss Distance vs Approach Angle\n'
                 'Circles = Pure Pursuit, Squares = Prop. Nav.', fontsize=9)
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)
    y_max = max(0.6, ax.get_ylim()[1])
    ax.set_ylim(-0.05, y_max)
    plt.tight_layout()
    path = os.path.join(out_dir, 'miss_vs_angle.png')
    plt.savefig(path, dpi=150); plt.close(); print(f'Saved: {path}')


def save_animation(results, out_dir):
    """Animate all 4 intruders vs defender using Pure Pursuit."""
    # Use only pure_pursuit for animation clarity
    strat = 'pure_pursuit'
    trajs = [(INTRUDER_LABELS[i], results[(INTRUDER_LABELS[i], strat)])
             for i in range(len(INTRUDER_LABELS))]

    max_len = max(len(res['traj_d']) for _, res in trajs)
    step = 4  # frame decimation

    fig = plt.figure(figsize=(9, 9))
    ax = fig.add_subplot(111, projection='3d')

    # Static: protected zone sphere wireframe
    u = np.linspace(0, 2 * np.pi, 20)
    v = np.linspace(0, np.pi, 20)
    Xz = R_ZONE * np.outer(np.cos(u), np.sin(v))
    Yz = R_ZONE * np.outer(np.sin(u), np.sin(v))
    Zz = R_ZONE * np.outer(np.ones(20), np.cos(v))
    ax.plot_wireframe(Xz, Yz, Zz, color='royalblue', alpha=0.15, linewidth=0.5)

    # Defender start
    ax.scatter(*DEFENDER_START, color='black', s=100, marker='D', zorder=8)

    # Lines for dynamic paths
    int_lines  = []
    def_lines  = []
    int_dots   = []
    def_dots   = []

    for i, (label, res) in enumerate(trajs):
        color = INTRUDER_COLORS[i]
        il, = ax.plot([], [], [], color=color, linewidth=1.2, linestyle=':', alpha=0.8)
        dl, = ax.plot([], [], [], color=color, linewidth=1.4, linestyle='-', alpha=0.9)
        id_, = ax.plot([], [], [], color=color, marker='^', markersize=8, linestyle='',
                       zorder=7)
        dd_, = ax.plot([], [], [], color=color, marker='o', markersize=8, linestyle='',
                       zorder=7, markeredgecolor='black', markeredgewidth=0.8)
        int_lines.append(il); def_lines.append(dl)
        int_dots.append(id_); def_dots.append(dd_)

    # Compute global axis limits
    all_pts = np.concatenate(
        [res['traj_d'] for _, res in trajs] + [res['traj_i'] for _, res in trajs]
    )
    margin = 1.5
    lim = max(np.abs(all_pts).max(), R_ZONE) + margin
    ax.set_xlim(-lim, lim); ax.set_ylim(-lim, lim); ax.set_zlim(-lim, lim)
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
    title = ax.set_title('')

    n_frames = max_len // step

    def update(frame):
        si = min(frame * step, max_len - 1)
        for i, (label, res) in enumerate(trajs):
            td = res['traj_d']
            ti = res['traj_i']
            n  = min(si + 1, len(td), len(ti))
            int_lines[i].set_data_3d(ti[:n, 0], ti[:n, 1], ti[:n, 2])
            def_lines[i].set_data_3d(td[:n, 0], td[:n, 1], td[:n, 2])
            int_dots[i].set_data_3d([ti[n-1, 0]], [ti[n-1, 1]], [ti[n-1, 2]])
            def_dots[i].set_data_3d([td[n-1, 0]], [td[n-1, 1]], [td[n-1, 2]])
        t_cur = si * DT
        title.set_text(f'S016 Airspace Defense (Pure Pursuit)   t={t_cur:.2f}s')
        return int_lines + def_lines + int_dots + def_dots + [title]

    ani = animation.FuncAnimation(fig, update, frames=n_frames, interval=50, blit=False)
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer='pillow', fps=20, dpi=100)
    plt.close(); print(f'Saved: {path}')


if __name__ == '__main__':
    print('S016 Airspace Defense — Simulation Results')
    print('=' * 55)
    results = run_simulation()

    # Print outcome summary
    print('\nOutcome summary:')
    pp_wins = sum(1 for (l, s), r in results.items()
                  if s == 'pure_pursuit' and r['outcome'] == 'defender_wins')
    pn_wins = sum(1 for (l, s), r in results.items()
                  if s == 'proportional_nav' and r['outcome'] == 'defender_wins')
    print(f'  Pure Pursuit wins:      {pp_wins}/{len(INTRUDER_STARTS)}')
    print(f'  Prop. Nav.  wins:       {pn_wins}/{len(INTRUDER_STARTS)}')

    all_times_pp = [r['time'] for (l, s), r in results.items()
                    if s == 'pure_pursuit' and r['outcome'] == 'defender_wins']
    all_times_pn = [r['time'] for (l, s), r in results.items()
                    if s == 'proportional_nav' and r['outcome'] == 'defender_wins']
    if all_times_pp:
        print(f'  Mean intercept time PP: {np.mean(all_times_pp):.3f}s')
    if all_times_pn:
        print(f'  Mean intercept time PN: {np.mean(all_times_pn):.3f}s')

    # Min miss distances
    for strat, label in [('pure_pursuit', 'PP'), ('proportional_nav', 'PN')]:
        misses = []
        for intruder_label in INTRUDER_LABELS:
            res = results[(intruder_label, strat)]
            td, ti = res['traj_d'], res['traj_i']
            n = min(len(td), len(ti))
            misses.append(float(np.min(np.linalg.norm(td[:n] - ti[:n], axis=1))))
        print(f'  Min miss distance {label}: {min(misses):.4f}m  mean={np.mean(misses):.4f}m')

    out_dir = os.path.normpath(OUTPUT_DIR)
    plot_3d_trajectories(results, out_dir)
    plot_outcome_table(results, out_dir)
    plot_apollonius(results, out_dir)
    plot_miss_vs_angle(results, out_dir)
    save_animation(results, out_dir)
