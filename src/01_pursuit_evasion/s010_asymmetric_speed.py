"""
S010 Asymmetric Speed — Bounded Arena Trapping
================================================
Evader is faster than pursuer (v_E=4.5 > v_P=3.0).  In open space, capture
is impossible.  A bounded square arena lets the pursuer exploit walls/corners.

Strategies compared:
  1. direct      — pure pursuit, fails when r<1
  2. wall_herd   — blend toward evader + toward wall nearest the evader (α=0.5)
  3. corner_trap — always aim at corner closest to evader

Also produces:
  - Apollonius circle snapshots (t=0,5,10,20s)
  - Capture time vs blend factor α sensitivity plot

Usage:
    conda activate drones
    python src/01_pursuit_evasion/s010_asymmetric_speed.py
"""

import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from matplotlib.patches import Circle

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from src.base.drone_base import DroneBase

# ── Parameters ───────────────────────────────────────────────
ARENA      = 5.0    # ±ARENA m square (10×10 m floor)
V_PURSUER  = 3.0    # m/s
V_EVADER   = 4.5    # m/s
ALPHA      = 0.5    # wall-herd blend
CAPTURE_R  = 0.15   # m
Z_FLY      = 2.0    # m fixed altitude
DT         = 1 / 48
MAX_TIME   = 60.0

INIT_PURSUER = np.array([0.0,  0.0,  Z_FLY])
INIT_EVADER  = np.array([2.0,  1.5,  Z_FLY])

CORNERS = np.array([[-ARENA, -ARENA, Z_FLY],
                     [ ARENA, -ARENA, Z_FLY],
                     [-ARENA,  ARENA, Z_FLY],
                     [ ARENA,  ARENA, Z_FLY]])

WALLS_DIRS = np.array([[-1, 0, 0],   # -x wall
                        [ 1, 0, 0],   # +x wall
                        [0, -1, 0],   # -y wall
                        [0,  1, 0]])  # +y wall
WALL_POS   = np.array([[-ARENA, 0, 0],
                        [ ARENA, 0, 0],
                        [0, -ARENA, 0],
                        [0,  ARENA, 0]], dtype=float)

STRATEGIES = ['direct', 'wall_herd', 'corner_trap']

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '01_pursuit_evasion', 's010_asymmetric_speed',
)

RNG = np.random.default_rng(13)


# ── Arena boundary enforcement ───────────────────────────────

def clip_to_arena(pos):
    pos = pos.copy()
    pos[0] = np.clip(pos[0], -ARENA, ARENA)
    pos[1] = np.clip(pos[1], -ARENA, ARENA)
    return pos


# ── Pursuer strategies ────────────────────────────────────────

def cmd_direct(pos_p, pos_e):
    d = pos_e - pos_p
    n = np.linalg.norm(d)
    return V_PURSUER * d / n if n > 1e-8 else np.zeros(3)


def cmd_wall_herd(pos_p, pos_e, alpha=ALPHA):
    # Direction toward evader
    r_pe = pos_e - pos_p
    n    = np.linalg.norm(r_pe)
    r_pe = r_pe / n if n > 1e-8 else r_pe

    # Nearest wall to evader (wall that evader should be pushed toward)
    dists_to_walls = [abs(pos_e[0] + ARENA),   # -x
                      abs(pos_e[0] - ARENA),    # +x
                      abs(pos_e[1] + ARENA),    # -y
                      abs(pos_e[1] - ARENA)]    # +y
    wi = np.argmin(dists_to_walls)
    wall_pt = WALL_POS[wi].copy()
    wall_pt[2] = Z_FLY
    r_wall = wall_pt - pos_p
    nw     = np.linalg.norm(r_wall)
    r_wall = r_wall / nw if nw > 1e-8 else r_wall

    v = alpha * r_pe + (1 - alpha) * r_wall
    nv = np.linalg.norm(v)
    return V_PURSUER * v / nv if nv > 1e-8 else np.zeros(3)


def cmd_corner_trap(pos_p, pos_e):
    # Aim at corner closest to evader
    dists = [np.linalg.norm(pos_e - c) for c in CORNERS]
    corner = CORNERS[np.argmin(dists)]
    r = corner - pos_p
    n = np.linalg.norm(r)
    return V_PURSUER * r / n if n > 1e-8 else np.zeros(3)


# ── Evader strategy — flee from pursuer ──────────────────────

def cmd_evader(pos_p, pos_e):
    """Straight escape away from pursuer, with wall bounce."""
    d = pos_e - pos_p
    n = np.linalg.norm(d)
    if n < 1e-8:
        return V_EVADER * np.array([1.0, 0.0, 0.0])
    return V_EVADER * d / n


# ── Apollonius circle ─────────────────────────────────────────

def apollonius_circle(pos_p, pos_e, v_p=V_PURSUER, v_e=V_EVADER):
    """
    Centre and radius of the Apollonius circle in XY plane.
    Points inside are reachable by pursuer first.
    """
    r = v_p / v_e
    if abs(1 - r ** 2) < 1e-6:
        return None, None   # equal speeds — circle at infinity
    d = np.linalg.norm(pos_e[:2] - pos_p[:2])
    R_apo    = r * d / (1 - r ** 2)
    offset   = d / (1 - r ** 2)
    # Centre lies on the line from P to E, offset from P
    direction = (pos_e[:2] - pos_p[:2]) / (d + 1e-8)
    centre = pos_p[:2] + offset * direction
    return centre, abs(R_apo)


# ── Simulation ────────────────────────────────────────────────

def run_simulation(strategy, alpha=ALPHA):
    pursuer = DroneBase(INIT_PURSUER.copy(), max_speed=V_PURSUER, dt=DT)
    evader  = DroneBase(INIT_EVADER.copy(),  max_speed=V_EVADER,  dt=DT)

    p_traj = [pursuer.pos.copy()]
    e_traj = [evader.pos.copy()]
    time_log = [0.0]

    captured = False
    cap_time = None
    max_steps = int(MAX_TIME / DT)

    for step in range(1, max_steps + 1):
        t = step * DT

        if strategy == 'direct':
            cmd_p = cmd_direct(pursuer.pos, evader.pos)
        elif strategy == 'wall_herd':
            cmd_p = cmd_wall_herd(pursuer.pos, evader.pos, alpha)
        else:
            cmd_p = cmd_corner_trap(pursuer.pos, evader.pos)

        cmd_e = cmd_evader(pursuer.pos, evader.pos)

        pursuer.step(cmd_p)
        evader.step(cmd_e)

        # Enforce arena walls
        pursuer.pos = clip_to_arena(pursuer.pos)
        evader.pos  = clip_to_arena(evader.pos)

        p_traj.append(pursuer.pos.copy())
        e_traj.append(evader.pos.copy())
        time_log.append(t)

        if np.linalg.norm(pursuer.pos - evader.pos) < CAPTURE_R:
            captured = True
            cap_time = t
            break

    return (
        np.array(p_traj),
        np.array(e_traj),
        np.array(time_log),
        captured,
        cap_time,
    )


# ── Plots ────────────────────────────────────────────────────

def _draw_arena(ax):
    rect = plt.Rectangle((-ARENA, -ARENA), 2*ARENA, 2*ARENA,
                          fill=False, edgecolor='grey',
                          linestyle='--', linewidth=1.2)
    ax.add_patch(rect)


def plot_trajectories_2d(results, out_dir):
    colors = ['darkorange', 'steelblue', 'mediumseagreen']
    labels = ['Direct Pursuit', 'Wall Herding (α=0.5)', 'Corner Trap']

    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    for ax, res, color, label in zip(axes, results, colors, labels):
        p_traj, e_traj, times, captured, cap_t = res
        _draw_arena(ax)

        # Corners
        for c in CORNERS:
            ax.scatter(*c[:2], color='grey', s=60, marker='s', zorder=3)

        ax.plot(p_traj[:, 0], p_traj[:, 1], color=color,
                linewidth=1.8, label='Pursuer')
        ax.plot(e_traj[:, 0], e_traj[:, 1], color='royalblue',
                linewidth=1.8, alpha=0.7, label='Evader')
        ax.scatter(*p_traj[0, :2], color=color, s=70, zorder=5)
        ax.scatter(*e_traj[0, :2], color='blue', s=70, zorder=5)

        if captured:
            ax.scatter(*p_traj[-1, :2], color='black', s=120, marker='X', zorder=6,
                       label=f'Captured {cap_t:.2f}s')
            status = f'Captured {cap_t:.2f} s'
        else:
            status = 'Timeout (60 s)'

        ax.set_xlim(-ARENA - 1, ARENA + 1)
        ax.set_ylim(-ARENA - 1, ARENA + 1)
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
        ax.set_title(f'{label}\n{status}', fontsize=9)
        ax.legend(fontsize=7, loc='upper left')

    fig.suptitle('S010 Asymmetric Speed — Top-Down Trajectories '
                 f'(v_P={V_PURSUER}, v_E={V_EVADER} m/s)', fontsize=11)
    plt.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'trajectories_2d.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_apollonius_snapshots(results, out_dir):
    """Apollonius circle snapshots at t=0,5,10,20s for wall_herd strategy."""
    res = results[1]   # wall_herd
    p_traj, e_traj, times, captured, cap_t = res

    snap_times = [0.0, 5.0, 10.0, 20.0]
    snap_idxs  = [np.argmin(np.abs(times - t)) for t in snap_times]

    fig, axes = plt.subplots(1, 4, figsize=(16, 4))
    colors_snap = plt.cm.plasma(np.linspace(0.1, 0.9, 4))

    for ax, si, st, color in zip(axes, snap_idxs, snap_times, colors_snap):
        _draw_arena(ax)
        ax.plot(p_traj[:si+1, 0], p_traj[:si+1, 1],
                color='red', linewidth=1.4, alpha=0.5)
        ax.plot(e_traj[:si+1, 0], e_traj[:si+1, 1],
                color='royalblue', linewidth=1.4, alpha=0.5)
        ax.scatter(*p_traj[si, :2], color='red',  s=80, zorder=5, label='Pursuer')
        ax.scatter(*e_traj[si, :2], color='blue', s=80, zorder=5, label='Evader')

        centre, radius = apollonius_circle(p_traj[si], e_traj[si])
        if centre is not None and radius < 50:
            circ = Circle(centre, radius, fill=False,
                          edgecolor=color, linewidth=2.0,
                          linestyle='-', label=f'Apollonius r={radius:.1f}m')
            ax.add_patch(circ)
            ax.scatter(*centre, color=color, s=40, marker='+', zorder=4)

        ax.set_xlim(-ARENA - 2, ARENA + 2)
        ax.set_ylim(-ARENA - 2, ARENA + 2)
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
        ax.set_title(f't = {st:.0f} s', fontsize=9)
        ax.legend(fontsize=7, loc='upper right')

    fig.suptitle('S010 Asymmetric Speed — Apollonius Circle Snapshots (Wall Herd)',
                 fontsize=11)
    plt.tight_layout()
    path = os.path.join(out_dir, 'apollonius_snapshots.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_alpha_sensitivity(out_dir):
    """Capture time vs blend factor alpha for wall_herd strategy."""
    alphas     = np.linspace(0.0, 1.0, 21)
    cap_times  = []
    captured_flags = []

    for a in alphas:
        res = run_simulation('wall_herd', alpha=a)
        _, _, times, captured, cap_t = res
        cap_times.append(cap_t if captured else MAX_TIME)
        captured_flags.append(captured)

    fig, ax = plt.subplots(figsize=(10, 5))
    c_arr = np.array(cap_times)
    f_arr = np.array(captured_flags)

    ax.plot(alphas, c_arr, color='steelblue', linewidth=2.0, marker='o', markersize=5)
    ax.scatter(alphas[f_arr],  c_arr[f_arr],  color='mediumseagreen', s=60, zorder=5,
               label='Captured')
    ax.scatter(alphas[~f_arr], c_arr[~f_arr], color='tomato', s=60, zorder=5,
               marker='X', label='Timeout')

    ax.axvline(ALPHA, color='grey', linestyle='--', linewidth=1.0,
               label=f'Default α={ALPHA}')
    ax.set_xlabel('Blend factor α  (0=wall-only, 1=pure pursuit)')
    ax.set_ylabel('Capture time (s)  [MAX_TIME=60 if timeout]')
    ax.set_title('S010 Asymmetric Speed — Capture Time vs Wall-Herd Blend Factor α',
                 fontsize=10)
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'alpha_sensitivity.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def plot_capture_bar(results, out_dir):
    labels     = ['Direct\nPursuit', 'Wall\nHerding', 'Corner\nTrap']
    colors     = ['darkorange', 'steelblue', 'mediumseagreen']
    cap_times  = []
    bar_colors = []
    anns       = []

    for res, color in zip(results, colors):
        _, _, times, captured, cap_t = res
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
        ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.5,
                ann, ha='center', va='bottom', fontsize=10, fontweight='bold')
    ax.set_ylabel('Capture Time (s)')
    ax.set_title(f'S010 Asymmetric Speed — Capture Time by Strategy\n'
                 f'(v_P={V_PURSUER}, v_E={V_EVADER} m/s, r={V_PURSUER/V_EVADER:.2f})',
                 fontsize=10)
    ax.set_ylim(0, MAX_TIME * 1.15)
    ax.grid(axis='y', alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'capture_bar.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def save_animation(results, out_dir):
    import matplotlib.animation as animation

    colors = ['darkorange', 'steelblue', 'mediumseagreen']
    labels = ['Direct', 'Wall Herd', 'Corner Trap']

    max_len = max(len(res[0]) for res in results)
    step = 3
    fig, axes = plt.subplots(1, 3, figsize=(15, 5))

    lines_p, lines_e, dots_p, dots_e, titles = [], [], [], [], []
    for ax, color, label, res in zip(axes, colors, labels, results):
        _draw_arena(ax)
        for c in CORNERS:
            ax.scatter(*c[:2], color='grey', s=40, marker='s', zorder=3)
        ax.set_xlim(-ARENA - 1, ARENA + 1)
        ax.set_ylim(-ARENA - 1, ARENA + 1)
        ax.set_aspect('equal'); ax.grid(True, alpha=0.3)
        ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')

        lp, = ax.plot([], [], color=color,       linewidth=1.8, alpha=0.8)
        le, = ax.plot([], [], color='royalblue', linewidth=1.8, alpha=0.8)
        dp, = ax.plot([], [], '^', color=color, markersize=9, zorder=6)
        de, = ax.plot([], [], 'bs', markersize=9, zorder=6)
        ti  = ax.set_title(label, fontsize=9)
        lines_p.append(lp); lines_e.append(le)
        dots_p.append(dp); dots_e.append(de); titles.append(ti)

    n_frames = max_len // step

    def update(i):
        artists = []
        for idx, res in enumerate(results):
            p_traj, e_traj, times, captured, cap_t = res
            si   = min(i * step, len(p_traj) - 1)
            done = (i * step >= len(p_traj) - 1)
            t    = times[si]
            dist = np.linalg.norm(p_traj[si, :2] - e_traj[si, :2])

            lines_p[idx].set_data(p_traj[:si+1, 0], p_traj[:si+1, 1])
            lines_e[idx].set_data(e_traj[:si+1, 0], e_traj[:si+1, 1])
            dots_p[idx].set_data([float(p_traj[si, 0])], [float(p_traj[si, 1])])
            dots_e[idx].set_data([float(e_traj[si, 0])], [float(e_traj[si, 1])])

            if done:
                st = f'✓ Captured {cap_t:.2f}s [DONE]' if captured else f'Timeout [DONE]'
                titles[idx].set_text(f'{labels[idx]}  t={t:.1f}s\n{st}')
            else:
                titles[idx].set_text(f'{labels[idx]}  t={t:.1f}s  dist={dist:.2f}m')
            artists += [lines_p[idx], lines_e[idx], dots_p[idx], dots_e[idx], titles[idx]]
        return artists

    ani = animation.FuncAnimation(fig, update, frames=n_frames,
                                  interval=60, blit=True)
    fig.suptitle(f'S010 Asymmetric Speed — Arena Trapping '
                 f'(v_P={V_PURSUER}, v_E={V_EVADER} m/s)',
                 fontsize=10)
    plt.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer='pillow', fps=16, dpi=100)
    plt.close()
    print(f'Saved: {path}')


# ── Main ─────────────────────────────────────────────────────

if __name__ == '__main__':
    print(f'Speed ratio r = {V_PURSUER/V_EVADER:.3f}  (< 1 → open-space capture impossible)\n')
    results = []
    for strategy in STRATEGIES:
        res = run_simulation(strategy)
        p_traj, e_traj, times, captured, cap_t = res
        status = f'captured @ {cap_t:.2f} s' if captured else f'timeout @ {times[-1]:.2f} s'
        print(f'[{strategy:<12}]  {status}')
        results.append(res)

    out_dir = os.path.normpath(OUTPUT_DIR)
    plot_trajectories_2d(results, out_dir)
    plot_apollonius_snapshots(results, out_dir)
    plot_alpha_sensitivity(out_dir)
    plot_capture_bar(results, out_dir)
    save_animation(results, out_dir)
