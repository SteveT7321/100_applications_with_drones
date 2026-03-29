"""
S010 3D Upgrade — Asymmetric Speed Bounded Arena Trapping
=========================================================
Evader (v_E=4.5 m/s) is faster than pursuer (v_P=3.0 m/s).
Arena is now a 3D cube [-5,5]³ m.

4 pursuer strategies:
  1. direct        — pure 3D pursuit
  2. wall_herd     — blend toward nearest cube face (6 faces)
  3. corner        — aim at 3D corner nearest to evader (8 corners)
  4. dive_boost    — dive steeply when pursuer has altitude advantage

Evader: straight 3D escape with vertical bias β=1.5

Usage:
    conda activate drones
    python src/01_pursuit_evasion/3d/s010_3d_asymmetric_speed.py
"""

import sys
import os
import numpy as np
from numpy.linalg import norm
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from mpl_toolkits.mplot3d.art3d import Line3DCollection
from matplotlib.animation import FuncAnimation, PillowWriter

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..'))
from src.base.drone_base import DroneBase

# ── Parameters ───────────────────────────────────────────────
ARENA      = 5.0
V_PURSUER  = 3.0
V_EVADER   = 4.5
ALPHA      = 0.5
BETA_VERT  = 1.5
DIVE_ANGLE = np.radians(30)
Z_MIN      = 0.5
Z_MAX      = 5.5
DT         = 0.05
MAX_TIME   = 60.0
CAPTURE_R  = 0.15

SPEED_RATIO = V_PURSUER / V_EVADER  # 0.667

INIT_PURSUER = np.array([0.0, 0.0, 2.0])
INIT_EVADER  = np.array([4.0, 0.0, 2.0])

CORNERS_3D = np.array([[sx, sy, sz]
                        for sx in [-5, 5]
                        for sy in [-5, 5]
                        for sz in [-5, 5]], dtype=float)

FACE_NORMALS = np.array([
    [1, 0, 0], [-1, 0, 0],
    [0, 1, 0], [0, -1, 0],
    [0, 0, 1], [0,  0, -1],
], dtype=float)

STRATEGIES = ['direct', 'wall_herd', 'corner', 'dive_boost']
STRATEGY_COLORS = {
    'direct':     'darkorange',
    'wall_herd':  'steelblue',
    'corner':     'mediumseagreen',
    'dive_boost': 'firebrick',
}
STRATEGY_LABELS = {
    'direct':     'Direct 3D',
    'wall_herd':  'Wall Herd 3D',
    'corner':     'Corner Target 3D',
    'dive_boost': 'Dive Boost 3D',
}

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..', '..',
    'outputs', '01_pursuit_evasion', '3d', 's010_3d_asymmetric_speed',
)
os.makedirs(OUTPUT_DIR, exist_ok=True)


# ── Pursuer strategies ────────────────────────────────────────

def wall_herd_3d(pos_p, pos_e, v_max=V_PURSUER):
    r_pe = pos_e - pos_p
    n = norm(r_pe)
    r_pe = r_pe / (n + 1e-8)
    # Find nearest face to evader
    face_dists = [5.0 - abs(float(pos_e @ fn)) for fn in FACE_NORMALS]
    nearest_idx = int(np.argmin(face_dists))
    n_face = FACE_NORMALS[nearest_idx]
    p_wall = pos_e.copy()
    p_wall -= face_dists[nearest_idx] * n_face * np.sign(pos_e @ n_face)
    r_wall = p_wall - pos_p
    r_wall = r_wall / (norm(r_wall) + 1e-8)
    v_blend = ALPHA * r_pe + (1 - ALPHA) * r_wall
    return v_max * v_blend / (norm(v_blend) + 1e-8)


def corner_target_3d(pos_p, pos_e, v_max=V_PURSUER):
    nearest_corner = CORNERS_3D[np.argmin([norm(pos_e - c) for c in CORNERS_3D])]
    r = nearest_corner - pos_p
    return v_max * r / (norm(r) + 1e-8)


def dive_boost_3d(pos_p, pos_e, v_max=V_PURSUER):
    """Dive toward evader if pursuer has altitude advantage > 1m and z > 2m."""
    dz = pos_p[2] - pos_e[2]
    if dz > 1.0 and pos_p[2] > 2.0:
        horiz = pos_e[:2] - pos_p[:2]
        phi = np.arctan2(horiz[1], horiz[0])
        v_dive = v_max * np.array([
            np.cos(phi) * np.cos(DIVE_ANGLE),
            np.sin(phi) * np.cos(DIVE_ANGLE),
            -np.sin(DIVE_ANGLE),
        ])
        return v_dive
    else:
        # Normal direct pursuit
        r = pos_e - pos_p
        return v_max * r / (norm(r) + 1e-8)


def evader_escape_3d(pos_e, pos_p, v_max=V_EVADER):
    """Escape with vertical bias."""
    away = pos_e - pos_p
    vertical_sign = np.sign(pos_e[2] - pos_p[2])
    if vertical_sign == 0:
        vertical_sign = 1.0
    away = away.copy()
    away[2] += BETA_VERT * vertical_sign
    return v_max * away / (norm(away) + 1e-8)


def apollonius_sphere_radius(pos_p, pos_e, r_speed=SPEED_RATIO):
    d = norm(pos_e - pos_p)
    if abs(1 - r_speed ** 2) < 1e-9:
        return np.inf
    return r_speed * d / (1 - r_speed ** 2)


def clip_to_3d_arena(pos):
    p = pos.copy()
    p[:2] = np.clip(p[:2], -ARENA, ARENA)
    p[2]  = np.clip(p[2], Z_MIN, Z_MAX)
    return p


# ── Simulation ────────────────────────────────────────────────

def run_simulation(strategy, seed=13):
    rng = np.random.default_rng(seed)
    pos_p = INIT_PURSUER.copy().astype(float)
    pos_e = INIT_EVADER.copy().astype(float)

    p_traj    = [pos_p.copy()]
    e_traj    = [pos_e.copy()]
    apo_radii = []
    times     = [0.0]
    dive_events = []  # timesteps where dive occurs

    max_steps = int(MAX_TIME / DT)
    captured = False
    cap_time = None

    for step in range(1, max_steps + 1):
        t = step * DT

        if strategy == 'direct':
            r = pos_e - pos_p
            v_p = V_PURSUER * r / (norm(r) + 1e-8)
        elif strategy == 'wall_herd':
            v_p = wall_herd_3d(pos_p, pos_e)
        elif strategy == 'corner':
            v_p = corner_target_3d(pos_p, pos_e)
        elif strategy == 'dive_boost':
            dz = pos_p[2] - pos_e[2]
            is_diving = dz > 1.0 and pos_p[2] > 2.0
            v_p = dive_boost_3d(pos_p, pos_e)
            if is_diving:
                dive_events.append(t)
        else:
            raise ValueError(f'Unknown strategy: {strategy}')

        v_e = evader_escape_3d(pos_e, pos_p)

        pos_p = clip_to_3d_arena(pos_p + v_p * DT)
        pos_e = clip_to_3d_arena(pos_e + v_e * DT)

        apo_radii.append(apollonius_sphere_radius(pos_p, pos_e))
        p_traj.append(pos_p.copy())
        e_traj.append(pos_e.copy())
        times.append(t)

        if norm(pos_p - pos_e) < CAPTURE_R:
            captured = True
            cap_time = t
            break

    return {
        'p_traj':      np.array(p_traj),
        'e_traj':      np.array(e_traj),
        'apo_radii':   np.array(apo_radii),
        'times':       np.array(times),
        'captured':    captured,
        'cap_time':    cap_time,
        'dive_events': dive_events,
    }


# ── Arena wireframe helper ────────────────────────────────────

def draw_arena_cube(ax):
    """Draw wireframe of arena cube [-5,5]³."""
    r = [-ARENA, ARENA]
    edges = []
    for x in r:
        for y in r:
            edges.append([(x, y, -ARENA), (x, y,  ARENA)])
    for x in r:
        for z in r:
            edges.append([(x, -ARENA, z), (x,  ARENA, z)])
    for y in r:
        for z in r:
            edges.append([(-ARENA, y, z), ( ARENA, y, z)])
    for edge in edges:
        xs = [edge[0][0], edge[1][0]]
        ys = [edge[0][1], edge[1][1]]
        zs = [edge[0][2], edge[1][2]]
        ax.plot(xs, ys, zs, color='gray', linewidth=0.5, alpha=0.4)


# ── Plots ─────────────────────────────────────────────────────

def plot_trajectories_3d(results, out_dir):
    """4 subplots (one per strategy) on 3D axes with arena cube wireframe."""
    fig = plt.figure(figsize=(16, 12))
    for idx, strategy in enumerate(STRATEGIES):
        res = results[strategy]
        ax = fig.add_subplot(2, 2, idx + 1, projection='3d')
        draw_arena_cube(ax)

        ax.plot(res['p_traj'][:, 0], res['p_traj'][:, 1], res['p_traj'][:, 2],
                color='red', linewidth=1.8, label='Pursuer', alpha=0.9)
        ax.plot(res['e_traj'][:, 0], res['e_traj'][:, 1], res['e_traj'][:, 2],
                color='royalblue', linewidth=1.8, label='Evader', alpha=0.9)

        ax.scatter(*res['p_traj'][0], color='red', s=50, zorder=6)
        ax.scatter(*res['e_traj'][0], color='blue', s=50, zorder=6)

        if res['captured']:
            ax.scatter(*res['p_traj'][-1], color='black', s=100,
                       marker='X', zorder=7, label=f"Captured {res['cap_time']:.2f}s")
            status = f"Captured {res['cap_time']:.2f}s"
        else:
            status = 'Timeout (60s)'

        ax.set_title(f"{STRATEGY_LABELS[strategy]}\n{status}", fontsize=9)
        ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
        ax.legend(fontsize=7, loc='upper left')
        ax.set_xlim(-ARENA, ARENA)
        ax.set_ylim(-ARENA, ARENA)
        ax.set_zlim(0, Z_MAX + 0.5)

    fig.suptitle(f'S010 3D Asymmetric Speed — Trajectories\n'
                 f'(v_P={V_PURSUER}, v_E={V_EVADER} m/s, r={SPEED_RATIO:.2f})',
                 fontsize=11)
    plt.tight_layout()
    path = os.path.join(out_dir, 'trajectories_3d.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_altitude_time(results, out_dir):
    """z_P and z_E vs time, 4 strategies (2×2 subplots)."""
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    for idx, strategy in enumerate(STRATEGIES):
        ax = axes[idx // 2][idx % 2]
        res = results[strategy]
        ax.plot(res['times'], res['p_traj'][:, 2], color='red',
                linewidth=1.8, label='Pursuer z')
        ax.plot(res['times'], res['e_traj'][:, 2], color='royalblue',
                linewidth=1.8, label='Evader z')
        ax.axhline(Z_MIN, color='gray', linestyle=':', linewidth=1.0, alpha=0.6)
        ax.axhline(Z_MAX, color='gray', linestyle=':', linewidth=1.0, alpha=0.6)

        # Mark dive events for dive_boost
        if strategy == 'dive_boost' and res['dive_events']:
            for de in res['dive_events'][::5]:  # sparse markers
                ax.axvline(de, color='firebrick', linewidth=0.4, alpha=0.3)

        if res['captured']:
            ax.axvline(res['cap_time'], color='black', linestyle='--',
                       linewidth=1.2, label=f"Captured {res['cap_time']:.2f}s")

        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Altitude z (m)')
        ax.set_title(STRATEGY_LABELS[strategy], fontsize=9)
        ax.legend(fontsize=7, loc='upper right')
        ax.grid(True, alpha=0.3)
        ax.set_ylim(0, Z_MAX + 0.5)

    fig.suptitle('S010 3D — Altitude vs Time (z_P and z_E)', fontsize=11)
    plt.tight_layout()
    path = os.path.join(out_dir, 'altitude_time.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_apollonius_radius(results, out_dir):
    """Apollonius sphere radius vs time for all 4 strategies."""
    fig, ax = plt.subplots(figsize=(12, 5))
    for strategy in STRATEGIES:
        res = results[strategy]
        # Times for apo_radii are one step ahead (step 1 to end)
        t_apo = res['times'][1:len(res['apo_radii']) + 1]
        apo = np.minimum(res['apo_radii'], 50.0)  # clip for display
        ax.plot(t_apo, apo, color=STRATEGY_COLORS[strategy],
                linewidth=1.8, label=STRATEGY_LABELS[strategy])
        if res['captured']:
            ax.axvline(res['cap_time'], color=STRATEGY_COLORS[strategy],
                       linestyle='--', linewidth=1.0, alpha=0.7)

    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Apollonius Sphere Radius (m, capped at 50)')
    ax.set_title(f'S010 3D — Apollonius Sphere Radius vs Time\n'
                 f'(r=v_P/v_E={SPEED_RATIO:.2f}, R_apo=r·d/(1-r²))',
                 fontsize=10)
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'apollonius_radius.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def plot_capture_comparison(results, out_dir):
    """Bar chart: capture time or total path distance for each strategy."""
    labels    = [STRATEGY_LABELS[s] for s in STRATEGIES]
    colors    = [STRATEGY_COLORS[s] for s in STRATEGIES]
    cap_times = []
    bar_colors = []
    anns      = []

    for strategy in STRATEGIES:
        res = results[strategy]
        if res['captured']:
            cap_times.append(res['cap_time'])
            bar_colors.append(STRATEGY_COLORS[strategy])
            anns.append(f"{res['cap_time']:.2f}s")
        else:
            cap_times.append(MAX_TIME)
            bar_colors.append('tomato')
            anns.append('Timeout')

    fig, ax = plt.subplots(figsize=(9, 5))
    bars = ax.bar(labels, cap_times, color=bar_colors, edgecolor='black', width=0.5)
    for bar, ann in zip(bars, anns):
        ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.5,
                ann, ha='center', va='bottom', fontsize=10, fontweight='bold')
    ax.set_ylabel('Capture Time (s)')
    ax.set_title(f'S010 3D Asymmetric Speed — Capture Comparison\n'
                 f'(v_P={V_PURSUER}, v_E={V_EVADER} m/s, r={SPEED_RATIO:.2f})',
                 fontsize=10)
    ax.set_ylim(0, MAX_TIME * 1.15)
    ax.grid(axis='y', alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'capture_comparison.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def save_animation(results, out_dir):
    """Animate dive_boost strategy (most interesting), showing pursuer dive events."""
    res = results['dive_boost']
    p_traj = res['p_traj']
    e_traj = res['e_traj']
    times  = res['times']
    n = len(p_traj)
    step = 3

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    draw_arena_cube(ax)

    ax.set_xlim(-ARENA, ARENA)
    ax.set_ylim(-ARENA, ARENA)
    ax.set_zlim(0, Z_MAX + 0.5)
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')

    line_p, = ax.plot([], [], [], color='red', linewidth=1.8, label='Pursuer', alpha=0.9)
    line_e, = ax.plot([], [], [], color='royalblue', linewidth=1.8,
                      label='Evader', alpha=0.9)
    dot_p,  = ax.plot([], [], [], 'r^', markersize=10, zorder=7)
    dot_e,  = ax.plot([], [], [], 'bs', markersize=10, zorder=7)
    ax.legend(fontsize=8)
    title = ax.set_title('S010 3D — Dive Boost Strategy', fontsize=10)

    def update(i):
        si = min(i * step, n - 1)
        line_p.set_data(p_traj[:si+1, 0], p_traj[:si+1, 1])
        line_p.set_3d_properties(p_traj[:si+1, 2])
        line_e.set_data(e_traj[:si+1, 0], e_traj[:si+1, 1])
        line_e.set_3d_properties(e_traj[:si+1, 2])
        dot_p.set_data([p_traj[si, 0]], [p_traj[si, 1]])
        dot_p.set_3d_properties([p_traj[si, 2]])
        dot_e.set_data([e_traj[si, 0]], [e_traj[si, 1]])
        dot_e.set_3d_properties([e_traj[si, 2]])
        t_now = times[si]
        dist = norm(p_traj[si] - e_traj[si])
        # Check if currently diving
        dz = p_traj[si, 2] - e_traj[si, 2]
        diving_str = ' [DIVING]' if (dz > 1.0 and p_traj[si, 2] > 2.0) else ''
        title.set_text(f'S010 3D — Dive Boost  t={t_now:.2f}s  dist={dist:.2f}m{diving_str}')
        return [line_p, line_e, dot_p, dot_e, title]

    n_frames = n // step
    ani = FuncAnimation(fig, update, frames=n_frames, interval=60, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer=PillowWriter(fps=16), dpi=100)
    plt.close()
    print(f'Saved: {path}')


# ── Main ─────────────────────────────────────────────────────

if __name__ == '__main__':
    print(f'Speed ratio r = {SPEED_RATIO:.3f}  (< 1 → open-space capture impossible)\n')
    out_dir = os.path.normpath(OUTPUT_DIR)

    results = {}
    for strategy in STRATEGIES:
        res = run_simulation(strategy)
        status = (f"captured @ {res['cap_time']:.2f}s"
                  if res['captured'] else f"timeout @ {res['times'][-1]:.2f}s")
        print(f'[{strategy:<12}]  {status}')
        results[strategy] = res

    print('\nGenerating plots...')
    plot_trajectories_3d(results, out_dir)
    plot_altitude_time(results, out_dir)
    plot_apollonius_radius(results, out_dir)
    plot_capture_comparison(results, out_dir)
    save_animation(results, out_dir)
    print('Done.')
