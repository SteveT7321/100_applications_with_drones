"""
S012 Relay Pursuit
===================
3 pursuers take turns chasing an evader; each has limited battery.
When the active pursuer's battery drops below a handoff threshold the
standby with the highest remaining energy takes over.  Standby drones
fly toward the predicted intercept position to be ready.

Comparison: 3-pursuer relay vs single pursuer (battery dies before capture).

Usage:
    conda activate drones
    python src/pursuit/s012_relay_pursuit.py
"""

import sys, os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from src.base.drone_base import DroneBase

# ── Parameters ───────────────────────────────────────────────
N_PURSUERS      = 3
E0              = 60.0    # J each
K_ENERGY        = 0.4     # W·s²/m²
HANDOFF_FRAC    = 0.20    # trigger handoff at 20% battery
V_ACTIVE        = 5.0     # m/s — active pursuer speed
V_STANDBY       = 4.0     # m/s — standby reposition speed
V_EVADER        = 3.5     # m/s — straight escape
CAPTURE_R       = 0.15    # m
DT              = 1 / 48
MAX_TIME        = 35.0

INIT_EVADER   = np.array([10.0, 0.0, 2.0])
EVADER_DIR    = np.array([ 1.0, 0.0, 0.0])
# P0: directly behind evader; P1/P2 flank positions ready to intercept
# T_max(P0)=6s, T_cap_single=10/1.5=6.67s → P0 runs out before capture
# P1/P2 positioned so they arrive near evader just after P0 hands off
PURSUER_INITS = [
    np.array([ 0.0,  0.0, 2.0]),   # P0 — 10 m behind evader (sole active)
    np.array([ 2.0,  4.0, 2.0]),   # P1 — flanking standby (closer)
    np.array([ 2.0, -4.0, 2.0]),   # P2 — flanking standby (closer)
]

P_COLORS = ['red', 'steelblue', 'mediumseagreen']

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '01_pursuit_evasion', 's012_relay_pursuit',
)


# ── Simulation ────────────────────────────────────────────────

def run_simulation(relay=True):
    pursuers = [DroneBase(p.copy(), max_speed=max(V_ACTIVE, V_STANDBY), dt=DT)
                for p in PURSUER_INITS]
    evader   = DroneBase(INIT_EVADER.copy(), max_speed=V_EVADER, dt=DT)

    energy   = [E0] * N_PURSUERS
    active   = 0
    handoff_times = []

    p_trajs  = [[p.pos.copy()] for p in pursuers]
    e_traj   = [evader.pos.copy()]
    en_log   = [[E0] for _ in range(N_PURSUERS)]
    act_log  = [active]
    time_log = [0.0]
    dist_log = [np.linalg.norm(pursuers[0].pos - evader.pos)]

    captured = False; cap_time = None
    max_steps = int(MAX_TIME / DT) if relay else int(MAX_TIME / DT)

    for step in range(1, max_steps + 1):
        t = step * DT

        # ── Energy drain for active pursuer ──
        energy[active] -= K_ENERGY * V_ACTIVE ** 2 * DT
        energy[active]  = max(energy[active], 0.0)

        # ── Handoff check (relay only) ──
        if relay and energy[active] / E0 < HANDOFF_FRAC:
            candidates = [(energy[j], j) for j in range(N_PURSUERS) if j != active
                          and energy[j] > 0]
            if candidates:
                new_active = max(candidates)[1]
                if new_active != active:
                    handoff_times.append(t)
                    active = new_active

        # ── Pursuer movements ──
        for i, pursuer in enumerate(pursuers):
            if i == active:
                # Pure pursuit
                if energy[i] > 0:
                    d = evader.pos - pursuer.pos
                    n = np.linalg.norm(d)
                    pursuer.step(V_ACTIVE * d / n if n > 1e-8 else np.zeros(3))
                # else pursuer is dead — stays put
            else:
                if relay and energy[i] > 0:
                    # Fly toward predicted intercept
                    t_handoff = max(energy[active] / (K_ENERGY * V_ACTIVE**2), 0.5)
                    p_intercept = evader.pos + EVADER_DIR * V_EVADER * t_handoff
                    d = p_intercept - pursuer.pos
                    n = np.linalg.norm(d)
                    pursuer.step(V_STANDBY * d / n if n > 1e-8 else np.zeros(3))

        # ── Evader straight escape ──
        evader.step(EVADER_DIR * V_EVADER)

        # ── Capture check ──
        for i, p in enumerate(pursuers):
            if np.linalg.norm(p.pos - evader.pos) < CAPTURE_R and energy[i] > 0:
                captured = True; cap_time = t
                break

        for i, p in enumerate(pursuers):
            p_trajs[i].append(p.pos.copy())
            en_log[i].append(energy[i])
        e_traj.append(evader.pos.copy())
        time_log.append(t)
        dist_log.append(np.linalg.norm(pursuers[active].pos - evader.pos))
        act_log.append(active)

        if captured:
            break

        # Single pursuer: stop when battery dead
        if not relay and energy[0] <= 0:
            break

    return (
        [np.array(tr) for tr in p_trajs],
        np.array(e_traj),
        [np.array(e) for e in en_log],
        np.array(time_log),
        np.array(dist_log),
        np.array(act_log),
        handoff_times,
        captured, cap_time,
    )


# ── Plots ─────────────────────────────────────────────────────

def plot_trajectories_3d(relay_res, single_res, out_dir):
    fig = plt.figure(figsize=(14, 5))

    for col, (res, title) in enumerate([(relay_res, '3-Pursuer Relay'),
                                         (single_res, 'Single Pursuer')]):
        p_trajs, e_traj, en_log, times, dist, act_log, handoffs, captured, cap_t = res
        ax = fig.add_subplot(1, 2, col+1, projection='3d')
        ax.view_init(elev=25, azim=-55)

        for i, (traj, color) in enumerate(zip(p_trajs, P_COLORS)):
            if len(traj) > 1:
                ax.plot(traj[:, 0], traj[:, 1], traj[:, 2],
                        color=color, linewidth=1.6, label=f'P{i+1}')
                ax.scatter(*traj[0], color=color, s=40, marker='o')

        ax.plot(e_traj[:, 0], e_traj[:, 1], e_traj[:, 2],
                color='royalblue', linewidth=1.8, label='Evader')
        ax.scatter(*e_traj[0], color='blue', s=40, marker='o')
        if captured:
            ax.scatter(*e_traj[-1], color='black', s=100, marker='X',
                       label=f'Captured {cap_t:.2f}s')
        status = f'Captured {cap_t:.2f}s' if captured else 'Battery out / Timeout'
        ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
        ax.set_title(f'{title}\n{status}', fontsize=9)
        ax.legend(fontsize=7, loc='upper left')

    fig.suptitle('S012 Relay Pursuit — 3D Trajectories', fontsize=11)
    plt.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'trajectories_3d.png')
    plt.savefig(path, dpi=150, bbox_inches='tight'); plt.close(); print(f'Saved: {path}')


def plot_battery_levels(relay_res, out_dir):
    p_trajs, e_traj, en_log, times, dist, act_log, handoffs, captured, cap_t = relay_res
    fig, ax = plt.subplots(figsize=(12, 5))

    for i, (en, color) in enumerate(zip(en_log, P_COLORS)):
        t_plot = times[:len(en)]
        ax.plot(t_plot, en, color=color, linewidth=1.8, label=f'P{i+1}')

    ax.axhline(E0 * HANDOFF_FRAC, color='grey', linestyle='--', linewidth=1.0,
               label=f'Handoff threshold ({HANDOFF_FRAC*100:.0f}%)')
    for ht in handoffs:
        ax.axvline(ht, color='orange', linestyle=':', linewidth=1.5, alpha=0.8)
    if cap_t:
        ax.axvline(cap_t, color='black', linestyle='--', linewidth=1.2,
                   label=f'Captured {cap_t:.2f}s')
    if handoffs:
        ax.axvline(handoffs[0], color='orange', linestyle=':', linewidth=1.5,
                   label='Handoff event')

    ax.set_xlabel('Time (s)'); ax.set_ylabel('Energy (J)')
    ax.set_title('S012 Relay Pursuit — Battery Level vs Time (3-pursuer relay)')
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'battery_levels.png')
    plt.savefig(path, dpi=150); plt.close(); print(f'Saved: {path}')


def plot_distance_comparison(relay_res, single_res, out_dir):
    fig, ax = plt.subplots(figsize=(12, 5))

    for res, label, color in [(relay_res, '3-Pursuer Relay', 'steelblue'),
                               (single_res, 'Single Pursuer', 'tomato')]:
        _, _, _, times, dist, _, _, captured, cap_t = res
        ax.plot(times, dist, color=color, linewidth=1.8, label=label)
        if captured:
            ax.axvline(cap_t, color=color, linestyle='--', linewidth=1.0, alpha=0.7)

    for ht in relay_res[6]:
        ax.axvline(ht, color='orange', linestyle=':', linewidth=1.2, alpha=0.7)
    if relay_res[6]:
        ax.axvline(relay_res[6][0], color='orange', linestyle=':',
                   linewidth=1.2, alpha=0.7, label='Handoff')

    ax.axhline(CAPTURE_R, color='black', linestyle='--', linewidth=0.8,
               label=f'Capture radius {CAPTURE_R}m')
    ax.set_xlabel('Time (s)'); ax.set_ylabel('Active Pursuer Distance to Evader (m)')
    ax.set_title('S012 Relay Pursuit — Distance to Evader vs Time')
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'distance_comparison.png')
    plt.savefig(path, dpi=150); plt.close(); print(f'Saved: {path}')


def save_animation(relay_res, out_dir):
    import matplotlib.animation as animation

    p_trajs, e_traj, en_log, times, dist, act_log, handoffs, captured, cap_t = relay_res
    max_len = len(e_traj); step = 3
    fig, ax = plt.subplots(figsize=(9, 7))

    all_pts = np.vstack([e_traj] + list(p_trajs))
    lo = all_pts.min(axis=0)[:2] - 1.5
    hi = all_pts.max(axis=0)[:2] + 1.5
    ax.set_xlim(lo[0], hi[0]); ax.set_ylim(lo[1], hi[1])
    ax.set_aspect('equal'); ax.grid(True, alpha=0.3)
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')

    for i, (traj, color) in enumerate(zip(p_trajs, P_COLORS)):
        ax.scatter(*traj[0, :2], color=color, s=60, zorder=5)
    ax.scatter(*e_traj[0, :2], color='blue', s=60, zorder=5)

    lines_p = [ax.plot([], [], color=c, linewidth=1.8, alpha=0.8)[0] for c in P_COLORS]
    dots_p  = [ax.plot([], [], '^', color=c, markersize=10, zorder=6)[0] for c in P_COLORS]
    line_e, = ax.plot([], [], color='royalblue', linewidth=1.8, alpha=0.8)
    dot_e,  = ax.plot([], [], 'bs', markersize=10, zorder=6)
    title   = ax.set_title('')
    n_frames = max_len // step

    def update(i):
        si  = min(i * step, len(e_traj)-1)
        ai  = act_log[si]
        t   = times[si]
        done = (i * step >= len(e_traj)-1)
        line_e.set_data(e_traj[:si+1, 0], e_traj[:si+1, 1])
        dot_e.set_data([float(e_traj[si, 0])], [float(e_traj[si, 1])])
        for j, (lp, dp, color) in enumerate(zip(lines_p, dots_p, P_COLORS)):
            sj = min(si, len(p_trajs[j])-1)
            lp.set_data(p_trajs[j][:sj+1, 0], p_trajs[j][:sj+1, 1])
            dp.set_data([float(p_trajs[j][sj, 0])], [float(p_trajs[j][sj, 1])])
            dp.set_markersize(13 if j == ai else 7)
        en_str = '  '.join([f'P{j+1}:{en_log[j][min(si,len(en_log[j])-1)]:.0f}J'
                             for j in range(N_PURSUERS)])
        if done:
            title.set_text(f't={t:.1f}s  ✓ Captured {cap_t:.2f}s [DONE]' if captured
                           else f't={t:.1f}s  Battery out [DONE]')
        else:
            title.set_text(f't={t:.1f}s  Active:P{ai+1}  {en_str}')
        return lines_p + dots_p + [line_e, dot_e, title]

    ani = animation.FuncAnimation(fig, update, frames=n_frames, interval=60, blit=True)
    fig.suptitle('S012 Relay Pursuit (larger triangle = active pursuer)', fontsize=10)
    plt.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer='pillow', fps=16, dpi=100); plt.close(); print(f'Saved: {path}')


if __name__ == '__main__':
    print('Running relay (3 pursuers)...')
    relay_res  = run_simulation(relay=True)
    print('Running single pursuer...')
    single_res = run_simulation(relay=False)

    for res, label in [(relay_res, 'Relay'), (single_res, 'Single')]:
        _, _, _, times, _, _, handoffs, captured, cap_t = res
        status = f'captured @ {cap_t:.2f}s' if captured else f'failed @ {times[-1]:.2f}s'
        print(f'[{label:<6}]  {status}  handoffs={handoffs}')

    out_dir = os.path.normpath(OUTPUT_DIR)
    plot_trajectories_3d(relay_res, single_res, out_dir)
    plot_battery_levels(relay_res, out_dir)
    plot_distance_comparison(relay_res, single_res, out_dir)
    save_animation(relay_res, out_dir)
