"""
S014 3D Upgrade — Decoy Lure
==============================
1 decoy + 2 flanker drones vs 1 evader.
- Decoy flies toward evader from front, climbs to z=5m to lure toward high altitude.
- Evader reacts away from nearest drone; clips to z bounds.
- Flanker0: approaches from below (z target=0.5m); Flanker1: approaches from side.
- Phase 1 (t<5s): decoy approaches, flankers orbit.
- Phase 2 (t>=5s or decoy within 2m): all pursue at full speed.

Usage:
    conda activate drones
    python src/01_pursuit_evasion/3d/s014_3d_decoy_lure.py
"""
import sys, os, numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from matplotlib.animation import FuncAnimation, PillowWriter

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..'))
from src.base.drone_base import DroneBase

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..', '..', 'outputs',
    '01_pursuit_evasion', '3d', 's014_3d_decoy_lure',
)
os.makedirs(OUTPUT_DIR, exist_ok=True)

# ── Parameters ────────────────────────────────────────────────
PHASE1_TIME      = 5.0     # s
DECOY_LURE_DIST  = 2.0     # m — triggers phase 2
V_PURSUER        = 5.0     # m/s (decoy & flankers)
V_EVADER         = 3.5     # m/s
Z_MIN            = 0.3
Z_MAX            = 6.0
DT               = 1 / 48
MAX_TIME         = 25.0
CAPTURE_R        = 0.15
ORBIT_R          = 4.0     # m flanker orbit radius in phase 1

# Initial positions
INIT_E   = np.array([ 4.0,  0.0, 2.0])
INIT_D   = np.array([-3.0,  0.0, 2.0])   # decoy
INIT_F0  = np.array([-2.0, -3.0, 0.5])   # flanker 0: below
INIT_F1  = np.array([-2.0,  3.0, 2.0])   # flanker 1: side

AGENT_COLORS = {
    'evader':  'royalblue',
    'decoy':   'limegreen',
    'flanker0':'firebrick',
    'flanker1':'darkorange',
}


# ── Strategy functions ────────────────────────────────────────

def decoy_strategy(pos_d: np.ndarray, pos_e: np.ndarray) -> np.ndarray:
    """Fly toward evader + constant climb component."""
    toward = pos_e - pos_d
    toward_norm = np.linalg.norm(toward) + 1e-8
    toward = toward / toward_norm
    climb = np.array([0.0, 0.0, 0.3])
    v = toward + climb
    return V_PURSUER * v / (np.linalg.norm(v) + 1e-8)


def flanker_patrol(pos_f: np.ndarray, pos_e: np.ndarray) -> np.ndarray:
    """Phase 1: orbit evader at ORBIT_R in XY, maintain own Z."""
    d_xy = pos_f[:2] - pos_e[:2]
    dist = np.linalg.norm(d_xy) + 1e-8
    radial     = d_xy / dist * (dist - ORBIT_R)
    tangential = np.array([-d_xy[1], d_xy[0]]) / dist  # CCW
    v_xy = -0.5 * radial + 2.0 * tangential
    return np.array([v_xy[0], v_xy[1], 0.0])


def pure_pursue(pos_agent: np.ndarray, pos_e: np.ndarray) -> np.ndarray:
    """Pure pursuit at full speed."""
    d = pos_e - pos_agent
    return V_PURSUER * d / (np.linalg.norm(d) + 1e-8)


def evader_escape(pos_e: np.ndarray, agents: list) -> np.ndarray:
    """Flee away from nearest agent."""
    dists = [np.linalg.norm(a - pos_e) for a in agents]
    nearest = agents[int(np.argmin(dists))]
    away = pos_e - nearest
    nn = np.linalg.norm(away) + 1e-8
    return V_EVADER * away / nn


# ── Simulation ────────────────────────────────────────────────

def run_simulation():
    pos_e  = INIT_E.copy().astype(float)
    pos_d  = INIT_D.copy().astype(float)
    pos_f0 = INIT_F0.copy().astype(float)
    pos_f1 = INIT_F1.copy().astype(float)

    traj_e  = [pos_e.copy()]
    traj_d  = [pos_d.copy()]
    traj_f0 = [pos_f0.copy()]
    traj_f1 = [pos_f1.copy()]
    times   = [0.0]
    phases  = [1]

    dist_d  = [np.linalg.norm(pos_d  - pos_e)]
    dist_f0 = [np.linalg.norm(pos_f0 - pos_e)]
    dist_f1 = [np.linalg.norm(pos_f1 - pos_e)]

    phase2_triggered = False
    phase2_time      = None
    capture_time     = None
    capture_agent    = None

    for step in range(1, int(MAX_TIME / DT) + 1):
        t = step * DT

        decoy_evader_dist = np.linalg.norm(pos_d - pos_e)
        if (not phase2_triggered) and (t >= PHASE1_TIME or decoy_evader_dist <= DECOY_LURE_DIST):
            phase2_triggered = True
            phase2_time      = t
            print(f'Phase 2 triggered at t={t:.2f}s')

        phase = 2 if phase2_triggered else 1

        # Move decoy
        v_d = decoy_strategy(pos_d, pos_e)
        pos_d += v_d * DT
        pos_d[2] = np.clip(pos_d[2], Z_MIN, Z_MAX)

        # Move flankers
        if phase == 1:
            v_f0 = flanker_patrol(pos_f0, pos_e)
            v_f1 = flanker_patrol(pos_f1, pos_e)
        else:
            v_f0 = pure_pursue(pos_f0, pos_e)
            v_f1 = pure_pursue(pos_f1, pos_e)
        pos_f0 += v_f0 * DT
        pos_f1 += v_f1 * DT
        pos_f0[2] = np.clip(pos_f0[2], Z_MIN, 1.5)   # flanker 0 stays low
        pos_f1[2] = np.clip(pos_f1[2], Z_MIN, Z_MAX)

        # Move evader
        all_agents = [pos_d, pos_f0, pos_f1]
        v_e = evader_escape(pos_e, all_agents)
        pos_e += v_e * DT
        pos_e[2] = np.clip(pos_e[2], Z_MIN, Z_MAX)

        # Record
        traj_e.append(pos_e.copy())
        traj_d.append(pos_d.copy())
        traj_f0.append(pos_f0.copy())
        traj_f1.append(pos_f1.copy())
        times.append(t)
        phases.append(phase)
        dist_d.append(np.linalg.norm(pos_d - pos_e))
        dist_f0.append(np.linalg.norm(pos_f0 - pos_e))
        dist_f1.append(np.linalg.norm(pos_f1 - pos_e))

        # Capture check
        for agent_pos, name in [(pos_d, 'decoy'), (pos_f0, 'flanker0'), (pos_f1, 'flanker1')]:
            if np.linalg.norm(agent_pos - pos_e) < CAPTURE_R:
                capture_time  = t
                capture_agent = name
                print(f'Captured by {name} at t={t:.2f}s')
                break
        if capture_time is not None:
            break

    if capture_time is None:
        print('Timeout.')

    return (
        np.array(traj_e), np.array(traj_d),
        np.array(traj_f0), np.array(traj_f1),
        np.array(times), np.array(phases),
        np.array(dist_d), np.array(dist_f0), np.array(dist_f1),
        phase2_time, capture_time, capture_agent,
    )


# ── Plots ─────────────────────────────────────────────────────

def plot_trajectories_3d(traj_e, traj_d, traj_f0, traj_f1,
                          phase2_time, capture_time, capture_agent, out_dir):
    fig = plt.figure(figsize=(12, 9))
    ax  = fig.add_subplot(111, projection='3d')

    ax.plot(traj_e[:, 0],  traj_e[:, 1],  traj_e[:, 2],
            color=AGENT_COLORS['evader'],  linewidth=2.0, label='Evader')
    ax.plot(traj_d[:, 0],  traj_d[:, 1],  traj_d[:, 2],
            color=AGENT_COLORS['decoy'],   linewidth=1.6, linestyle='--', label='Decoy')
    ax.plot(traj_f0[:, 0], traj_f0[:, 1], traj_f0[:, 2],
            color=AGENT_COLORS['flanker0'], linewidth=1.6, label='Flanker 0 (low)')
    ax.plot(traj_f1[:, 0], traj_f1[:, 1], traj_f1[:, 2],
            color=AGENT_COLORS['flanker1'], linewidth=1.6, label='Flanker 1 (side)')

    # Mark phase transition
    if phase2_time is not None:
        idx = int(phase2_time / DT)
        idx = min(idx, len(traj_e) - 1)
        ax.scatter(*traj_e[idx], color='purple', s=120, marker='D',
                   zorder=8, label=f'Phase 2 t={phase2_time:.1f}s')

    # Start markers
    for traj, color, marker in [
        (traj_e,  AGENT_COLORS['evader'],   'o'),
        (traj_d,  AGENT_COLORS['decoy'],    's'),
        (traj_f0, AGENT_COLORS['flanker0'], '^'),
        (traj_f1, AGENT_COLORS['flanker1'], '^'),
    ]:
        ax.scatter(*traj[0], color=color, s=70, marker=marker, zorder=6)

    if capture_time is not None:
        ax.scatter(*traj_e[-1], color='black', s=160, marker='X',
                   zorder=9, label=f'Captured by {capture_agent} t={capture_time:.2f}s')

    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
    ax.set_title('S014 3D Decoy Lure — Full 3D Trajectories')
    ax.legend(fontsize=8, loc='upper left')
    plt.tight_layout()
    path = os.path.join(out_dir, 'trajectories_3d.png')
    plt.savefig(path, dpi=150, bbox_inches='tight'); plt.close()
    print(f'Saved: {path}')


def plot_altitude_time(traj_e, traj_d, traj_f0, traj_f1,
                        times, phase2_time, capture_time, out_dir):
    fig, ax = plt.subplots(figsize=(12, 5))

    ax.plot(times, traj_e[:, 2],  color=AGENT_COLORS['evader'],   linewidth=2.0, label='Evader z')
    ax.plot(times, traj_d[:, 2],  color=AGENT_COLORS['decoy'],    linewidth=1.6, linestyle='--', label='Decoy z')
    ax.plot(times, traj_f0[:, 2], color=AGENT_COLORS['flanker0'], linewidth=1.6, label='Flanker 0 z')
    ax.plot(times, traj_f1[:, 2], color=AGENT_COLORS['flanker1'], linewidth=1.6, label='Flanker 1 z')

    ax.axhline(Z_MIN, color='gray', linestyle=':', alpha=0.5)
    ax.axhline(Z_MAX, color='gray', linestyle=':', alpha=0.5, label='z bounds')

    # Shade phases
    t_max = times[-1]
    if phase2_time is not None:
        ax.axvspan(0, phase2_time, alpha=0.07, color='steelblue', label='Phase 1')
        ax.axvspan(phase2_time, t_max, alpha=0.07, color='tomato', label='Phase 2')
        ax.axvline(phase2_time, color='purple', linestyle='--', linewidth=1.2,
                   label=f'Phase 2 t={phase2_time:.1f}s')

    if capture_time is not None:
        ax.axvline(capture_time, color='black', linestyle='--',
                   linewidth=1.2, label=f'Capture t={capture_time:.2f}s')

    ax.set_xlabel('Time (s)'); ax.set_ylabel('Altitude z (m)')
    ax.set_title('S014 3D Decoy Lure — Altitude vs Time')
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'altitude_time.png')
    plt.savefig(path, dpi=150); plt.close()
    print(f'Saved: {path}')


def plot_distance_time(times, dist_d, dist_f0, dist_f1,
                        phase2_time, capture_time, out_dir):
    fig, ax = plt.subplots(figsize=(12, 5))
    ax.plot(times, dist_d,  color=AGENT_COLORS['decoy'],    linewidth=1.6, linestyle='--', label='Decoy')
    ax.plot(times, dist_f0, color=AGENT_COLORS['flanker0'], linewidth=1.6, label='Flanker 0')
    ax.plot(times, dist_f1, color=AGENT_COLORS['flanker1'], linewidth=1.6, label='Flanker 1')
    ax.axhline(CAPTURE_R, color='red', linestyle='--', linewidth=0.9,
               label=f'Capture r={CAPTURE_R}m')
    ax.axhline(DECOY_LURE_DIST, color='purple', linestyle=':', linewidth=0.9,
               label=f'Lure trigger r={DECOY_LURE_DIST}m')
    if phase2_time is not None:
        ax.axvline(phase2_time, color='purple', linestyle='--', linewidth=1.2,
                   label=f'Phase 2 t={phase2_time:.1f}s')
    if capture_time is not None:
        ax.axvline(capture_time, color='black', linestyle='--',
                   linewidth=1.2, label=f'Capture t={capture_time:.2f}s')
    ax.set_xlabel('Time (s)'); ax.set_ylabel('Distance to Evader (m)')
    ax.set_title('S014 3D Decoy Lure — Agent Distances to Evader')
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'distance_time.png')
    plt.savefig(path, dpi=150); plt.close()
    print(f'Saved: {path}')


def save_animation(traj_e, traj_d, traj_f0, traj_f1, times, phases, out_dir):
    step = 4
    n_frames = max(1, len(times) // step)

    fig = plt.figure(figsize=(10, 8))
    ax  = fig.add_subplot(111, projection='3d')

    all_pts = np.vstack([traj_e, traj_d, traj_f0, traj_f1])
    margin  = 0.5
    lo = all_pts.min(axis=0) - margin
    hi = all_pts.max(axis=0) + margin
    ax.set_xlim(lo[0], hi[0]); ax.set_ylim(lo[1], hi[1]); ax.set_zlim(lo[2], hi[2])
    ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')

    line_e,  = ax.plot([], [], [], color=AGENT_COLORS['evader'],   linewidth=2.0, label='Evader')
    line_d,  = ax.plot([], [], [], color=AGENT_COLORS['decoy'],    linewidth=1.4, linestyle='--', label='Decoy')
    line_f0, = ax.plot([], [], [], color=AGENT_COLORS['flanker0'], linewidth=1.4, label='Flanker0')
    line_f1, = ax.plot([], [], [], color=AGENT_COLORS['flanker1'], linewidth=1.4, label='Flanker1')
    dot_e,   = ax.plot([], [], [], 'bs',  markersize=10)
    dot_d,   = ax.plot([], [], [], 'gs',  markersize=8)
    dot_f0,  = ax.plot([], [], [], '^',  color=AGENT_COLORS['flanker0'], markersize=9)
    dot_f1,  = ax.plot([], [], [], '^',  color=AGENT_COLORS['flanker1'], markersize=9)
    title    = ax.set_title('')
    ax.legend(fontsize=7, loc='upper left')

    def update(fi):
        si = min(fi * step, len(times) - 1)
        t  = times[si]
        ph = phases[si]

        for line, traj in [(line_e, traj_e), (line_d, traj_d),
                           (line_f0, traj_f0), (line_f1, traj_f1)]:
            line.set_data(traj[:si+1, 0], traj[:si+1, 1])
            line.set_3d_properties(traj[:si+1, 2])

        for dot, traj in [(dot_e, traj_e), (dot_d, traj_d),
                          (dot_f0, traj_f0), (dot_f1, traj_f1)]:
            dot.set_data([traj[si, 0]], [traj[si, 1]])
            dot.set_3d_properties([traj[si, 2]])

        title.set_text(f'S014 3D Decoy Lure  t={t:.1f}s  Phase={ph}')
        return [line_e, line_d, line_f0, line_f1,
                dot_e, dot_d, dot_f0, dot_f1, title]

    ani = FuncAnimation(fig, update, frames=n_frames, interval=60, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer=PillowWriter(fps=16), dpi=90)
    plt.close()
    print(f'Saved: {path}')


# ── Main ──────────────────────────────────────────────────────
if __name__ == '__main__':
    (traj_e, traj_d, traj_f0, traj_f1,
     times, phases, dist_d, dist_f0, dist_f1,
     phase2_time, capture_time, capture_agent) = run_simulation()

    out_dir = os.path.normpath(OUTPUT_DIR)
    plot_trajectories_3d(traj_e, traj_d, traj_f0, traj_f1,
                          phase2_time, capture_time, capture_agent, out_dir)
    plot_altitude_time(traj_e, traj_d, traj_f0, traj_f1,
                        times, phase2_time, capture_time, out_dir)
    plot_distance_time(times, dist_d, dist_f0, dist_f1,
                        phase2_time, capture_time, out_dir)
    save_animation(traj_e, traj_d, traj_f0, traj_f1, times, phases, out_dir)
    print('S014 3D complete.')
