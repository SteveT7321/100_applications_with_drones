"""
S011 Swarm Encirclement
========================
3 pursuer drones simultaneously encircle 1 evader, synchronously shrinking
the encirclement radius.  Evader attempts to break through the weakest gap.

Strategy:
  Each pursuer i targets position p_E + R(t)·[cos(2πi/N), sin(2πi/N), 0].
  R(t) shrinks linearly from R0 to CAPTURE_R over T_CONVERGE seconds.
  Evader moves toward the widest gap between adjacent pursuers.

Usage:
    conda activate drones
    python src/pursuit/s011_swarm_encirclement.py
"""

import sys, os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from src.base.drone_base import DroneBase

# ── Parameters ───────────────────────────────────────────────
N_PURSUERS  = 3
R_INIT      = 4.0    # m
T_CONVERGE  = 15.0   # s
V_PURSUER   = 4.0    # m/s
V_EVADER    = 3.0    # m/s
CAPTURE_R   = 0.15   # m
DT          = 1 / 48
MAX_TIME    = 20.0

INIT_EVADER = np.array([0.0, 0.0, 2.0])

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '01_pursuit_evasion', 's011_swarm_encirclement',
)


def make_pursuer_inits(n=N_PURSUERS, r=R_INIT, z=2.0):
    inits = []
    for i in range(n):
        a = 2 * np.pi * i / n
        inits.append(np.array([r * np.cos(a), r * np.sin(a), z]))
    return inits


def encirclement_radius(t, r0=R_INIT, r_min=CAPTURE_R, t_conv=T_CONVERGE):
    return max(r0 - (r0 - r_min) * t / t_conv, r_min)


def encirclement_targets(pos_e, t, n=N_PURSUERS):
    R = encirclement_radius(t)
    targets = []
    for i in range(n):
        a = 2 * np.pi * i / n
        targets.append(pos_e + np.array([R * np.cos(a), R * np.sin(a), 0.0]))
    return targets


def evader_breakout(pursuer_positions, pos_e):
    """Move toward the widest angular gap between adjacent pursuers."""
    angles = []
    for p in pursuer_positions:
        d = p[:2] - pos_e[:2]
        angles.append(np.arctan2(d[1], d[0]))
    angles.sort()
    angles.append(angles[0] + 2 * np.pi)   # wrap

    gaps = [angles[i+1] - angles[i] for i in range(len(angles)-1)]
    widest_idx = int(np.argmax(gaps))
    mid_angle  = angles[widest_idx] + gaps[widest_idx] / 2
    # escape through gap mid-point (outward)
    return V_EVADER * np.array([np.cos(mid_angle), np.sin(mid_angle), 0.0])


# ── Simulation ────────────────────────────────────────────────

def run_simulation():
    inits   = make_pursuer_inits()
    pursuers = [DroneBase(p.copy(), max_speed=V_PURSUER, dt=DT) for p in inits]
    evader   = DroneBase(INIT_EVADER.copy(), max_speed=V_EVADER, dt=DT)

    p_trajs  = [[p.pos.copy()] for p in pursuers]
    e_traj   = [evader.pos.copy()]
    r_log    = [R_INIT]
    dist_log = [[np.linalg.norm(p.pos - evader.pos)] for p in pursuers]
    time_log = [0.0]

    captured = False; cap_time = None; cap_who = None
    max_steps = int(MAX_TIME / DT)

    for step in range(1, max_steps + 1):
        t = step * DT

        targets = encirclement_targets(evader.pos, t)
        for i, (pursuer, tgt) in enumerate(zip(pursuers, targets)):
            d = tgt - pursuer.pos
            n = np.linalg.norm(d)
            pursuer.step(V_PURSUER * d / n if n > 1e-8 else np.zeros(3))

        e_vel = evader_breakout([p.pos for p in pursuers], evader.pos)
        evader.step(e_vel)

        r_log.append(encirclement_radius(t))
        time_log.append(t)
        for i, p in enumerate(pursuers):
            p_trajs[i].append(p.pos.copy())
            dist_log[i].append(np.linalg.norm(p.pos - evader.pos))
        e_traj.append(evader.pos.copy())

        for i, p in enumerate(pursuers):
            if np.linalg.norm(p.pos - evader.pos) < CAPTURE_R:
                captured = True; cap_time = t; cap_who = i
                break
        if captured:
            break

    return (
        [np.array(t) for t in p_trajs],
        np.array(e_traj),
        np.array(r_log),
        [np.array(d) for d in dist_log],
        np.array(time_log),
        captured, cap_time, cap_who,
    )


# ── Plots ─────────────────────────────────────────────────────

P_COLORS = ['red', 'darkorange', 'mediumseagreen']

def plot_trajectories_2d(p_trajs, e_traj, times, captured, cap_time, out_dir):
    fig, ax = plt.subplots(figsize=(8, 8))

    for i, (traj, color) in enumerate(zip(p_trajs, P_COLORS)):
        ax.plot(traj[:, 0], traj[:, 1], color=color, linewidth=1.8,
                label=f'Pursuer {i+1}')
        ax.scatter(*traj[0, :2], color=color, s=80, zorder=5)

    ax.plot(e_traj[:, 0], e_traj[:, 1], color='royalblue',
            linewidth=2.0, label='Evader')
    ax.scatter(*e_traj[0, :2], color='blue', s=80, zorder=5)

    # Encirclement circles at t=0, 5, 10 s
    for t_snap in [0.0, 5.0, 10.0]:
        R = encirclement_radius(t_snap)
        theta = np.linspace(0, 2*np.pi, 100)
        idx = min(int(t_snap / DT), len(e_traj)-1)
        cx, cy = e_traj[idx, 0], e_traj[idx, 1]
        ax.plot(cx + R*np.cos(theta), cy + R*np.sin(theta),
                color='grey', linestyle='--', linewidth=0.8, alpha=0.5)
        ax.text(cx + R + 0.1, cy, f't={t_snap:.0f}s R={R:.1f}m',
                fontsize=7, color='grey')

    if captured:
        ax.scatter(*p_trajs[cap_who][-1, :2], color='black', s=140,
                   marker='X', zorder=7, label=f'Captured by P{cap_who+1} @ {cap_time:.2f}s')

    ax.set_aspect('equal'); ax.grid(True, alpha=0.3)
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
    ax.set_title(f'S011 Swarm Encirclement — Top-Down View\n'
                 f'({"Captured @ "+f"{cap_time:.2f}s" if captured else "Timeout"})',
                 fontsize=10)
    ax.legend(fontsize=8, loc='upper right')
    plt.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'trajectories_2d.png')
    plt.savefig(path, dpi=150); plt.close(); print(f'Saved: {path}')


def plot_radius_and_distances(r_log, dist_log, times, cap_time, out_dir):
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(11, 7), sharex=True)

    ax1.plot(times, r_log, color='steelblue', linewidth=2.0, label='Encirclement R(t)')
    ax1.axhline(CAPTURE_R, color='red', linestyle='--', linewidth=1,
                label=f'Capture radius {CAPTURE_R}m')
    if cap_time:
        ax1.axvline(cap_time, color='black', linestyle='--', linewidth=1.2,
                    label=f'Captured {cap_time:.2f}s')
    ax1.set_ylabel('Radius (m)'); ax1.legend(fontsize=8); ax1.grid(True, alpha=0.3)
    ax1.set_title('S011 Swarm Encirclement — Radius & Distances vs Time')

    for i, (dist, color) in enumerate(zip(dist_log, P_COLORS)):
        ax2.plot(times, dist, color=color, linewidth=1.8, label=f'Pursuer {i+1}')
    ax2.axhline(CAPTURE_R, color='red', linestyle='--', linewidth=1)
    if cap_time:
        ax2.axvline(cap_time, color='black', linestyle='--', linewidth=1.2)
    ax2.set_xlabel('Time (s)'); ax2.set_ylabel('Distance to Evader (m)')
    ax2.legend(fontsize=8); ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    path = os.path.join(out_dir, 'radius_and_distances.png')
    plt.savefig(path, dpi=150); plt.close(); print(f'Saved: {path}')


def save_animation(p_trajs, e_traj, times, cap_time, out_dir):
    import matplotlib.animation as animation

    max_len = max(len(p_trajs[0]), len(e_traj))
    step = 3
    fig, ax = plt.subplots(figsize=(7, 7))

    margin = R_INIT + 1.5
    ax.set_xlim(-margin, margin); ax.set_ylim(-margin, margin)
    ax.set_aspect('equal'); ax.grid(True, alpha=0.3)
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')

    lines_p = [ax.plot([], [], color=c, linewidth=1.6, alpha=0.8)[0] for c in P_COLORS]
    dots_p  = [ax.plot([], [], '^', color=c, markersize=10, zorder=6)[0] for c in P_COLORS]
    line_e, = ax.plot([], [], color='royalblue', linewidth=1.8, alpha=0.8)
    dot_e,  = ax.plot([], [], 'bs', markersize=10, zorder=6)
    # Encirclement circle
    circ_th = np.linspace(0, 2*np.pi, 80)
    circ_line, = ax.plot([], [], 'k--', linewidth=1.0, alpha=0.4)
    title = ax.set_title('')

    n_frames = max_len // step

    def update(i):
        si = min(i * step, len(e_traj) - 1)
        t  = times[si]
        R  = encirclement_radius(t)
        ex, ey = e_traj[si, 0], e_traj[si, 1]
        circ_line.set_data(ex + R*np.cos(circ_th), ey + R*np.sin(circ_th))
        line_e.set_data(e_traj[:si+1, 0], e_traj[:si+1, 1])
        dot_e.set_data([float(ex)], [float(ey)])
        for j, (lp, dp) in enumerate(zip(lines_p, dots_p)):
            sj = min(si, len(p_trajs[j])-1)
            lp.set_data(p_trajs[j][:sj+1, 0], p_trajs[j][:sj+1, 1])
            dp.set_data([float(p_trajs[j][sj, 0])], [float(p_trajs[j][sj, 1])])
        done = (i * step >= len(e_traj)-1)
        st = f'✓ Captured {cap_time:.2f}s [DONE]' if (done and cap_time) else f'R={R:.2f}m'
        title.set_text(f'S011 Swarm Encirclement  t={t:.2f}s  {st}')
        return lines_p + dots_p + [line_e, dot_e, circ_line, title]

    ani = animation.FuncAnimation(fig, update, frames=n_frames, interval=60, blit=True)
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer='pillow', fps=16, dpi=100); plt.close(); print(f'Saved: {path}')


if __name__ == '__main__':
    p_trajs, e_traj, r_log, dist_log, times, captured, cap_time, cap_who = run_simulation()
    status = f'captured by P{cap_who+1} @ {cap_time:.2f}s' if captured else 'timeout'
    print(f'Result: {status}')
    out_dir = os.path.normpath(OUTPUT_DIR)
    plot_trajectories_2d(p_trajs, e_traj, times, captured, cap_time, out_dir)
    plot_radius_and_distances(r_log, dist_log, times, cap_time, out_dir)
    save_animation(p_trajs, e_traj, times, cap_time, out_dir)
