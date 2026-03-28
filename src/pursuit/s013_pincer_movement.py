"""
S013 Pincer Movement
=====================
Two pursuers approach the evader from opposite directions (~180° apart),
narrowing its escape cone.  Evader escapes away from the centroid of pursuers.

Comparison: coordinated pincer (±y flank, shrinking offset) vs two independent
pure-pursuit drones (both head straight for evader).

Usage:
    conda activate drones
    python src/pursuit/s013_pincer_movement.py
"""

import sys, os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from src.base.drone_base import DroneBase

# ── Parameters ───────────────────────────────────────────────
V_PURSUER  = 5.0
V_EVADER   = 3.5
R0         = 4.0    # initial offset radius for pincer targets
R_MIN      = 0.05   # must be < CAPTURE_R so pursuer can reach evader
V_SHRINK   = 0.5    # m/s — offset shrink rate
CAPTURE_R  = 0.15
DT         = 1 / 48
MAX_TIME   = 20.0

# Pursuers start ahead of the evader, flanking from ±y
# Evader will run in +x; pursuers intercept from both y-sides
INIT_P1    = np.array([ 6.0, -4.0, 2.0])   # ahead and below evader
INIT_P2    = np.array([ 6.0,  4.0, 2.0])   # ahead and above evader
INIT_E     = np.array([ 0.0,  0.0, 2.0])

P_COLORS   = ['red', 'darkorange']

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '01_pursuit_evasion', 's013_pincer_movement',
)


# ── Helpers ───────────────────────────────────────────────────

def pincer_angle(p1, p2, pe):
    d1 = p1 - pe; d2 = p2 - pe
    n1 = np.linalg.norm(d1); n2 = np.linalg.norm(d2)
    if n1 < 1e-8 or n2 < 1e-8:
        return 0.0
    cos_a = np.dot(d1, d2) / (n1 * n2)
    return np.degrees(np.arccos(np.clip(cos_a, -1, 1)))


def evader_escape(p1, p2, pe):
    """Escape in +x (away from pursuers' initial flanking positions)."""
    return V_EVADER * np.array([1.0, 0.0, 0.0])


# ── Simulation ────────────────────────────────────────────────

def run_simulation(coordinated=True):
    p1 = DroneBase(INIT_P1.copy(), max_speed=V_PURSUER, dt=DT)
    p2 = DroneBase(INIT_P2.copy(), max_speed=V_PURSUER, dt=DT)
    ev = DroneBase(INIT_E.copy(),  max_speed=V_EVADER,  dt=DT)

    p1_traj = [p1.pos.copy()]; p2_traj = [p2.pos.copy()]
    e_traj  = [ev.pos.copy()]
    angle_log = [pincer_angle(p1.pos, p2.pos, ev.pos)]
    dist_log  = [[np.linalg.norm(p1.pos - ev.pos)],
                 [np.linalg.norm(p2.pos - ev.pos)]]
    time_log  = [0.0]

    captured = False; cap_time = None; cap_who = None

    for step in range(1, int(MAX_TIME / DT) + 1):
        t = step * DT

        if coordinated:
            R_cur  = max(R0 - V_SHRINK * t, R_MIN)
            # Pincer from ±y: flank positions above/below evader
            tgt1   = ev.pos + R_cur * np.array([0.0, -1.0, 0.0])
            tgt2   = ev.pos + R_cur * np.array([0.0,  1.0, 0.0])
        else:
            tgt1 = ev.pos
            tgt2 = ev.pos

        for pursuer, tgt in [(p1, tgt1), (p2, tgt2)]:
            d = tgt - pursuer.pos
            n = np.linalg.norm(d)
            pursuer.step(V_PURSUER * d / n if n > 1e-8 else np.zeros(3))

        e_vel = evader_escape(p1.pos, p2.pos, ev.pos)
        ev.step(e_vel)

        for i, (pursuer, cap_i) in enumerate([(p1, 0), (p2, 1)]):
            if np.linalg.norm(pursuer.pos - ev.pos) < CAPTURE_R:
                captured = True; cap_time = t; cap_who = cap_i
                break

        p1_traj.append(p1.pos.copy()); p2_traj.append(p2.pos.copy())
        e_traj.append(ev.pos.copy())
        angle_log.append(pincer_angle(p1.pos, p2.pos, ev.pos))
        dist_log[0].append(np.linalg.norm(p1.pos - ev.pos))
        dist_log[1].append(np.linalg.norm(p2.pos - ev.pos))
        time_log.append(t)

        if captured:
            break

    return (
        np.array(p1_traj), np.array(p2_traj), np.array(e_traj),
        np.array(angle_log), [np.array(d) for d in dist_log],
        np.array(time_log), captured, cap_time, cap_who,
    )


# ── Plots ─────────────────────────────────────────────────────

def plot_trajectories_2d(results, out_dir):
    labels = ['Coordinated Pincer', 'Independent Pure Pursuit']
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))

    for ax, (p1_t, p2_t, e_t, angle_log, dist_log, times, captured, cap_t, cap_who), label \
            in zip(axes, results, labels):
        ax.plot(p1_t[:, 0], p1_t[:, 1], color=P_COLORS[0], linewidth=1.8, label='Pursuer 1')
        ax.plot(p2_t[:, 0], p2_t[:, 1], color=P_COLORS[1], linewidth=1.8, label='Pursuer 2')
        ax.plot(e_t[:, 0],  e_t[:, 1],  color='royalblue', linewidth=1.8, label='Evader')
        ax.scatter(*p1_t[0, :2], color=P_COLORS[0], s=70, zorder=5)
        ax.scatter(*p2_t[0, :2], color=P_COLORS[1], s=70, zorder=5)
        ax.scatter(*e_t[0, :2],  color='blue',       s=70, zorder=5)
        if captured:
            traj = p1_t if cap_who == 0 else p2_t
            ax.scatter(*traj[-1, :2], color='black', s=140, marker='X', zorder=7,
                       label=f'Captured P{cap_who+1} @ {cap_t:.2f}s')
            status = f'Captured {cap_t:.2f}s'
        else:
            status = 'Timeout'
        ax.set_aspect('equal'); ax.grid(True, alpha=0.3)
        ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
        ax.set_title(f'{label}\n{status}', fontsize=9)
        ax.legend(fontsize=7, loc='upper right')

    fig.suptitle('S013 Pincer Movement — Top-Down Trajectories', fontsize=11)
    plt.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'trajectories_2d.png')
    plt.savefig(path, dpi=150, bbox_inches='tight'); plt.close(); print(f'Saved: {path}')


def plot_pincer_angle(results, out_dir):
    labels = ['Coordinated Pincer', 'Independent']
    colors = ['steelblue', 'darkorange']
    fig, ax = plt.subplots(figsize=(11, 5))
    for (p1_t, p2_t, e_t, angle_log, dist_log, times, captured, cap_t, _), label, color \
            in zip(results, labels, colors):
        ax.plot(times, angle_log, color=color, linewidth=1.8, label=label)
        if captured:
            ax.axvline(cap_t, color=color, linestyle='--', linewidth=1.0, alpha=0.7)
    ax.axhline(180, color='grey', linestyle=':', linewidth=1.0, alpha=0.7,
               label='Ideal pincer (180°)')
    ax.set_xlabel('Time (s)'); ax.set_ylabel('Pincer Angle (°)')
    ax.set_title('S013 Pincer Movement — Pincer Angle vs Time')
    ax.legend(fontsize=9); ax.grid(True, alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'pincer_angle.png')
    plt.savefig(path, dpi=150); plt.close(); print(f'Saved: {path}')


def plot_distances(results, out_dir):
    labels = ['Coordinated Pincer', 'Independent']
    fig, axes = plt.subplots(1, 2, figsize=(13, 5), sharey=True)
    for ax, (p1_t, p2_t, e_t, angle_log, dist_log, times, captured, cap_t, _), label \
            in zip(axes, results, labels):
        ax.plot(times, dist_log[0], color=P_COLORS[0], linewidth=1.8, label='Pursuer 1')
        ax.plot(times, dist_log[1], color=P_COLORS[1], linewidth=1.8, label='Pursuer 2')
        ax.axhline(CAPTURE_R, color='red', linestyle='--', linewidth=0.8,
                   label=f'Capture r={CAPTURE_R}m')
        if captured:
            ax.axvline(cap_t, color='black', linestyle='--', linewidth=1.0,
                       label=f'Captured {cap_t:.2f}s')
        ax.set_xlabel('Time (s)'); ax.set_ylabel('Distance to Evader (m)')
        ax.set_title(f'{label}', fontsize=9)
        ax.legend(fontsize=7); ax.grid(True, alpha=0.3)
    fig.suptitle('S013 Pincer Movement — Pursuer Distances to Evader', fontsize=11)
    plt.tight_layout()
    path = os.path.join(out_dir, 'distances.png')
    plt.savefig(path, dpi=150); plt.close(); print(f'Saved: {path}')


def save_animation(results, out_dir):
    import matplotlib.animation as animation

    labels = ['Coordinated Pincer', 'Independent']
    max_len = max(len(r[2]) for r in results)
    step = 3
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))

    artists_store = []
    for ax, (p1_t, p2_t, e_t, angle_log, dist_log, times, captured, cap_t, _), label \
            in zip(axes, results, labels):
        all_pts = np.vstack([p1_t, p2_t, e_t])
        lo = all_pts.min(axis=0)[:2] - 1.0
        hi = all_pts.max(axis=0)[:2] + 1.0
        ax.set_xlim(lo[0], hi[0]); ax.set_ylim(lo[1], hi[1])
        ax.set_aspect('equal'); ax.grid(True, alpha=0.3)
        ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
        for pts, c in [(p1_t, P_COLORS[0]), (p2_t, P_COLORS[1]), (e_t, 'blue')]:
            ax.scatter(*pts[0, :2], color=c, s=50, zorder=5)
        lp1, = ax.plot([], [], color=P_COLORS[0], linewidth=1.8)
        lp2, = ax.plot([], [], color=P_COLORS[1], linewidth=1.8)
        le,  = ax.plot([], [], color='royalblue', linewidth=1.8)
        dp1, = ax.plot([], [], '^', color=P_COLORS[0], markersize=10, zorder=6)
        dp2, = ax.plot([], [], '^', color=P_COLORS[1], markersize=10, zorder=6)
        de,  = ax.plot([], [], 'bs', markersize=10, zorder=6)
        ti   = ax.set_title(label, fontsize=9)
        artists_store.append((lp1, lp2, le, dp1, dp2, de, ti,
                               p1_t, p2_t, e_t, angle_log, times, captured, cap_t, label))

    n_frames = max_len // step

    def update(i):
        arts = []
        for (lp1, lp2, le, dp1, dp2, de, ti,
             p1_t, p2_t, e_t, angle_log, times, captured, cap_t, label) in artists_store:
            si = min(i * step, len(e_t)-1)
            done = (i * step >= len(e_t)-1)
            t = times[si]
            ang = angle_log[si]
            for l, pt in [(lp1, p1_t), (lp2, p2_t), (le, e_t)]:
                sj = min(si, len(pt)-1)
                l.set_data(pt[:sj+1, 0], pt[:sj+1, 1])
            for d, pt in [(dp1, p1_t), (dp2, p2_t), (de, e_t)]:
                sj = min(si, len(pt)-1)
                d.set_data([float(pt[sj, 0])], [float(pt[sj, 1])])
            if done:
                st = f'✓ Captured {cap_t:.2f}s [DONE]' if captured else 'Timeout [DONE]'
                ti.set_text(f'{label}  t={t:.1f}s\n{st}')
            else:
                ti.set_text(f'{label}  t={t:.1f}s  angle={ang:.1f}°')
            arts += [lp1, lp2, le, dp1, dp2, de, ti]
        return arts

    ani = animation.FuncAnimation(fig, update, frames=n_frames, interval=60, blit=True)
    fig.suptitle('S013 Pincer Movement', fontsize=10)
    plt.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer='pillow', fps=16, dpi=100); plt.close(); print(f'Saved: {path}')


if __name__ == '__main__':
    results = []
    for coord, label in [(True, 'Coordinated pincer'), (False, 'Independent')]:
        res = run_simulation(coordinated=coord)
        _, _, _, angle_log, _, times, captured, cap_t, cap_who = res
        status = f'captured P{cap_who+1} @ {cap_t:.2f}s' if captured else 'timeout'
        print(f'[{label:<22}]  {status}  max_angle={angle_log.max():.1f}°')
        results.append(res)

    out_dir = os.path.normpath(OUTPUT_DIR)
    plot_trajectories_2d(results, out_dir)
    plot_pincer_angle(results, out_dir)
    plot_distances(results, out_dir)
    save_animation(results, out_dir)
