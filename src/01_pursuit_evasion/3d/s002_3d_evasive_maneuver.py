"""
S002 3D Upgrade — Evasive Maneuver
====================================
Pursuer (Pure Pursuit, full 3D) vs Evader with 5 evasion strategies inside the
bounded arena x, y in [-8, 8] m, z in [0.3, 8] m:
  1. straight         — fly directly away from the pursuer in 3D (baseline added
                        by the review; it is the best strategy in 2D)
  2. horizontal_perp  — perpendicular to the LOS in x-y only (2D card strategy)
  3. perp_3d          — Gram-Schmidt 3D perpendicular to the LOS (plane of r_hat, z_hat)
  4. helix            — constant angular rate around z while climbing
  5. dive_and_run     — drop from 2 m to 0.5 m in 1 s, then straight escape

Both drones are clipped to the arena. When the evader's command pushes into a
wall, the outward component is removed and the remainder is rescaled to full
speed (the evader slides along the wall).

Also runs a helix pitch-angle sweep (card Extension 1) to test the card's claim
that a pitched helix delays capture.

Usage:
    conda activate drones
    python src/01_pursuit_evasion/3d/s002_3d_evasive_maneuver.py
"""
import sys
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from matplotlib.animation import FuncAnimation, PillowWriter

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..'))
from src.base.drone_base import DroneBase

# ── Scenario parameters ─────────────────────────────────────
PURSUER_SPEED = 5.0
EVADER_SPEED  = 3.5
DT            = 1 / 48
MAX_TIME      = 30.0
CAPTURE_R     = 0.15

INIT_PURSUER = np.array([-2.0, 0.0, 2.0])
INIT_EVADER  = np.array([ 2.0, 0.0, 2.0])

Z_MIN = 0.3
Z_MAX = 8.0
ARENA = 8.0                      # x, y in [-ARENA, ARENA]
LO = np.array([-ARENA, -ARENA, Z_MIN])
HI = np.array([ ARENA,  ARENA, Z_MAX])

# Helix parameters
HELIX_ALPHA   = np.radians(30.0)   # pitch angle
R_HELIX       = 1.5                # helix radius (m)

# Dive-and-run parameters (card: z drops from 2 m to 0.5 m in 1 s)
DIVE_Z        = 0.5
DIVE_T        = 1.0

PITCH_SWEEP_DEG = np.arange(0, 91, 10)

OUTPUT_DIR = os.path.normpath(os.path.join(
    os.path.dirname(__file__), '..', '..', '..',
    'outputs', '01_pursuit_evasion', '3d', 's002_3d_evasive_maneuver',
))


# ── Evader strategy functions ────────────────────────────────

def strategy_straight(pos_e, pos_p, v_max, t):
    """Fly directly away from the pursuer along the 3D LOS."""
    r3 = pos_e - pos_p
    return v_max * r3 / (np.linalg.norm(r3) + 1e-8)


def strategy_horizontal_perp(pos_e, pos_p, v_max, t):
    """2D baseline: perpendicular in x-y plane, z constant."""
    r_xy = pos_e[:2] - pos_p[:2]
    r_hat = r_xy / (np.linalg.norm(r_xy) + 1e-8)
    return v_max * np.array([-r_hat[1], r_hat[0], 0.0])


def strategy_perp_3d(pos_e, pos_p, v_max, t):
    """Gram-Schmidt: perpendicular to 3D LOS in the plane containing z-hat."""
    r3 = pos_e - pos_p
    r_hat = r3 / (np.linalg.norm(r3) + 1e-8)
    z_hat = np.array([0.0, 0.0, 1.0])
    e_perp = z_hat - np.dot(z_hat, r_hat) * r_hat
    if np.linalg.norm(e_perp) < 1e-4:
        x_hat = np.array([1.0, 0.0, 0.0])
        e_perp = x_hat - np.dot(x_hat, r_hat) * r_hat
    return v_max * e_perp / (np.linalg.norm(e_perp) + 1e-8)


def make_helix(alpha):
    """Helix with pitch angle alpha: horizontal rotation at omega + climb."""
    omega = EVADER_SPEED * np.cos(alpha) / R_HELIX

    def _helix(pos_e, pos_p, v_max, t):
        vz = v_max * np.sin(alpha)
        vh = v_max * np.cos(alpha)
        return np.array([-vh * np.sin(omega * t), vh * np.cos(omega * t), vz])

    return _helix


strategy_helix = make_helix(HELIX_ALPHA)


def strategy_dive_and_run(pos_e, pos_p, v_max, t):
    """Card dive: descend at (2 - 0.5)/1 s = 1.5 m/s for the first 1 s while
    escaping horizontally with the rest of the speed; then straight 3D escape."""
    if t < DIVE_T and pos_e[2] > DIVE_Z:
        vz = -(INIT_EVADER[2] - DIVE_Z) / DIVE_T
        vh = np.sqrt(max(v_max**2 - vz**2, 0.0))
        r_xy = pos_e[:2] - pos_p[:2]
        r_hat_xy = r_xy / (np.linalg.norm(r_xy) + 1e-8)
        return np.array([vh * r_hat_xy[0], vh * r_hat_xy[1], vz])
    return strategy_straight(pos_e, pos_p, v_max, t)


# ── Arena ────────────────────────────────────────────────────

def enforce_arena(pos):
    """Clip a position to the arena box."""
    return np.clip(pos, LO, HI)


def wall_slide(pos, v):
    """Remove velocity components that push out of the arena and rescale the
    remainder to the commanded speed. If nothing is left (command exactly into
    a wall), move along the free axis direction with the most room."""
    speed = np.linalg.norm(v)
    if speed < 1e-12:
        return v
    v = v.copy()
    blocked = np.zeros(3, dtype=bool)
    for k in range(3):
        if (pos[k] <= LO[k] + 1e-9 and v[k] < 0) or (pos[k] >= HI[k] - 1e-9 and v[k] > 0):
            v[k] = 0.0
            blocked[k] = True
    if not blocked.any():
        return v
    n = np.linalg.norm(v)
    if n > 1e-6 * speed:
        return v * speed / n
    best, best_dir = -1.0, None
    for k in range(3):
        if blocked[k]:
            continue
        for sgn in (+1.0, -1.0):
            room = (HI[k] - pos[k]) if sgn > 0 else (pos[k] - LO[k])
            if room > best + 1e-12:
                best = room
                best_dir = np.zeros(3)
                best_dir[k] = sgn
    return speed * best_dir


# ── Simulation ───────────────────────────────────────────────

def run_simulation(evader_fn, bounded=True):
    """Returns pursuer traj, evader traj, captured, capture time, LOS angles (deg),
    and the number of steps the evader spent sliding along a wall."""
    pursuer = DroneBase(INIT_PURSUER, max_speed=PURSUER_SPEED, dt=DT)
    evader  = DroneBase(INIT_EVADER,  max_speed=EVADER_SPEED,  dt=DT)

    max_steps = int(MAX_TIME / DT)
    captured = False
    capture_time = None
    los_angles = []
    wall_steps = 0

    for step in range(max_steps):
        t = step * DT
        if np.linalg.norm(pursuer.pos - evader.pos) < CAPTURE_R:
            captured = True
            capture_time = t
            break

        # Pursuer: pure pursuit in full 3D
        r_pe = evader.pos - pursuer.pos
        pursuer.step(PURSUER_SPEED * r_pe / (np.linalg.norm(r_pe) + 1e-8))
        if bounded:
            pursuer.pos = enforce_arena(pursuer.pos)
            pursuer.trajectory[-1] = pursuer.pos.copy()

        # Evader: strategy, then wall sliding
        v_cmd = evader_fn(evader.pos, pursuer.pos, EVADER_SPEED, t)
        v_evader = wall_slide(evader.pos, v_cmd) if bounded else v_cmd
        if bounded and not np.allclose(v_evader, v_cmd):
            wall_steps += 1
        los = evader.pos - pursuer.pos
        evader.step(v_evader)
        if bounded:
            evader.pos = enforce_arena(evader.pos)
            evader.trajectory[-1] = evader.pos.copy()

        # angle between the evader's applied velocity and the LOS (P -> E)
        cos_theta = np.dot(v_evader, los) / (
            (np.linalg.norm(v_evader) + 1e-8) * (np.linalg.norm(los) + 1e-8))
        los_angles.append(np.degrees(np.arccos(np.clip(cos_theta, -1.0, 1.0))))

    return (pursuer.get_trajectory(), evader.get_trajectory(), captured,
            capture_time, np.array(los_angles), wall_steps)


# ── Plotting ─────────────────────────────────────────────────

STRATEGY_NAMES = ['straight', 'horizontal_perp', 'perp_3d', 'helix', 'dive_and_run']
STRATEGY_LABELS = ['Straight\n(3D, away from P)', 'Horizontal Perp\n(2D baseline)',
                   '3D Perp\n(Gram-Schmidt)', 'Helix\n(30° climb)', 'Dive-and-Run']


def plot_trajectories_3d(results):
    fig = plt.figure(figsize=(24, 5))
    for i, r in enumerate(results):
        ax = fig.add_subplot(1, len(results), i + 1, projection='3d')
        p_traj, e_traj = r['p'], r['e']
        ax.plot(p_traj[:, 0], p_traj[:, 1], p_traj[:, 2],
                color='red', linewidth=1.5, label='Pursuer')
        ax.plot(e_traj[:, 0], e_traj[:, 1], e_traj[:, 2],
                color='blue', linewidth=1.5, label='Evader')
        ax.scatter(*p_traj[0], color='red',  s=50, marker='o')
        ax.scatter(*e_traj[0], color='blue', s=50, marker='o')
        if r['captured']:
            ax.scatter(*p_traj[-1], color='black', s=120, marker='X',
                       label=f'Capture {r["t"]:.2f} s')
        ax.set_xlim(-ARENA, ARENA)
        ax.set_ylim(-ARENA, ARENA)
        ax.set_zlim(0, Z_MAX)
        ax.set_xlabel('X (m)', fontsize=7)
        ax.set_ylabel('Y (m)', fontsize=7)
        ax.set_zlabel('Z (m)', fontsize=7)
        cap_str = f'Capture: {r["t"]:.2f} s' if r['captured'] else 'Not captured'
        ax.set_title(f'{r["label"]}\n{cap_str}', fontsize=9)
        ax.legend(loc='upper left', fontsize=6)
    fig.suptitle('S002 3D — Evasive Maneuver: Trajectories (axes = arena)', fontsize=13)
    path = os.path.join(OUTPUT_DIR, 'trajectories_3d.png')
    plt.savefig(path, dpi=110, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_altitude_time(results):
    fig, axes = plt.subplots(1, len(results), figsize=(20, 4), sharey=True)
    for ax, r in zip(axes, results):
        n = min(len(r['p']), len(r['e']))
        t_arr = np.arange(n) * DT
        ax.plot(t_arr, r['p'][:n, 2], color='red',  linewidth=1.5, label='Pursuer z')
        ax.plot(t_arr, r['e'][:n, 2], color='blue', linewidth=1.5, label='Evader z')
        ax.axhline(Z_MIN, color='gray', linestyle=':', linewidth=0.8)
        ax.axhline(Z_MAX, color='gray', linestyle=':', linewidth=0.8)
        if r['captured']:
            ax.axvline(r['t'], color='black', linestyle='--', linewidth=1.0, alpha=0.7)
        ax.set_title(r['label'].replace('\n', ' '), fontsize=9)
        ax.set_xlabel('Time (s)')
        ax.grid(True, alpha=0.3)
        ax.set_ylim(-0.2, Z_MAX + 0.5)
    axes[0].set_ylabel('Altitude z (m)')
    axes[0].legend(fontsize=7)
    fig.suptitle('S002 3D — Altitude vs Time (dotted: z bounds 0.3 / 8 m)', fontsize=12)
    plt.tight_layout()
    path = os.path.join(OUTPUT_DIR, 'altitude_time.png')
    plt.savefig(path, dpi=120, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_capture_times(results, t_straight_unbounded):
    labels = [r['label'] for r in results]
    times = [r['t'] if r['captured'] else MAX_TIME for r in results]
    colors = ['orange', 'steelblue', 'teal', 'purple', 'saddlebrown']

    fig, ax = plt.subplots(figsize=(11, 5))
    bars = ax.bar(labels, times, color=colors, edgecolor='black', linewidth=0.8)
    for bar, r in zip(bars, results):
        lbl = f'{r["t"]:.2f} s' if r['captured'] else 'Timeout'
        ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.04, lbl,
                ha='center', va='bottom', fontsize=9)
    ax.axhline(t_straight_unbounded, color='orange', linestyle='--', linewidth=1.0,
               label=f'Straight, no arena (= 2D result): {t_straight_unbounded:.2f} s')
    ax.set_ylabel('Capture Time (s)')
    ax.set_title(f'S002 3D — Capture Time by Evasion Strategy (bounded arena)\n'
                 f'Pursuer {PURSUER_SPEED} m/s vs Evader {EVADER_SPEED} m/s, start range 4 m')
    ax.set_ylim(0, max(max(times), t_straight_unbounded) * 1.25)
    ax.legend(loc='upper right')
    ax.grid(True, axis='y', alpha=0.3)
    plt.tight_layout()
    path = os.path.join(OUTPUT_DIR, 'capture_times.png')
    plt.savefig(path, dpi=120, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_los_angle(results):
    fig, axes = plt.subplots(1, len(results), figsize=(20, 4), sharey=True)
    for ax, r in zip(axes, results):
        t_arr = np.arange(len(r['los'])) * DT
        ax.plot(t_arr, r['los'], color='darkorange', linewidth=1.5)
        ax.axhline(90.0, color='gray', linestyle='--', linewidth=1.0, alpha=0.7,
                   label='90° (range rate = -v_P)')
        if r['captured']:
            ax.axvline(r['t'], color='black', linestyle='--', linewidth=1.0, alpha=0.7)
        ax.set_title(r['label'].replace('\n', ' '), fontsize=9)
        ax.set_xlabel('Time (s)')
        ax.set_ylim(-5, 185)
        ax.grid(True, alpha=0.3)
    axes[0].set_ylabel('Angle between v_E and LOS (°)')
    axes[0].legend(fontsize=7)
    fig.suptitle('S002 3D — Evader velocity vs LOS angle φ  '
                 '(range rate = -v_P + v_E cos φ; 0° is the slowest closing)', fontsize=12)
    plt.tight_layout()
    path = os.path.join(OUTPUT_DIR, 'los_angle.png')
    plt.savefig(path, dpi=120, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_pitch_sweep(sweep, t_fixed_heading):
    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.plot(PITCH_SWEEP_DEG, sweep, 'o-', color='purple', label='Helix capture time')
    perp_t = (np.linalg.norm(INIT_EVADER - INIT_PURSUER) - CAPTURE_R) / PURSUER_SPEED
    ax.axhline(perp_t, color='gray', linestyle='--',
               label=f'Always-perpendicular evader (r0 - r_cap)/v_P = {perp_t:.2f} s')
    ax.axhline(t_fixed_heading, color='orange', linestyle=':',
               label=f'Constant horizontal heading +y: {t_fixed_heading:.2f} s')
    ax.set_xlabel('Helix pitch angle α (°)')
    ax.set_ylabel('Capture time (s)')
    ax.set_title('S002 3D — Helix pitch sweep (card Extension 1)')
    ax.set_ylim(0, max(sweep) * 1.3)
    ax.grid(True, alpha=0.3)
    ax.legend()
    plt.tight_layout()
    path = os.path.join(OUTPUT_DIR, 'helix_pitch_sweep.png')
    plt.savefig(path, dpi=120, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def save_animation(results):
    """All five strategies side by side; runs to the longest capture time and
    freezes each panel after its capture."""
    step_skip = 2
    n_steps = max(len(r['p']) for r in results)
    frame_steps = list(range(0, n_steps, step_skip))
    if frame_steps[-1] != n_steps - 1:
        frame_steps.append(n_steps - 1)       # always show the final capture

    fig = plt.figure(figsize=(20, 4.2))
    fig.subplots_adjust(left=0.0, right=0.98, bottom=0.02, top=0.88, wspace=0.05)
    panels = []
    for i, r in enumerate(results):
        ax = fig.add_subplot(1, len(results), i + 1, projection='3d')
        ax.set_xlim(-ARENA, ARENA)
        ax.set_ylim(-ARENA, ARENA)
        ax.set_zlim(0, Z_MAX)
        ax.set_xlabel('X', fontsize=7)
        ax.set_ylabel('Y', fontsize=7)
        ax.set_zlabel('Z', fontsize=7)
        ax.tick_params(labelsize=6)
        ax.set_title(r['label'].replace('\n', ' '), fontsize=9)
        p_trail, = ax.plot([], [], [], color='red',  linewidth=1.2)
        e_trail, = ax.plot([], [], [], color='blue', linewidth=1.2)
        p_dot, = ax.plot([], [], [], 'r^', markersize=7)
        e_dot, = ax.plot([], [], [], 'bs', markersize=7)
        panels.append((r, p_trail, e_trail, p_dot, e_dot))
    title = fig.suptitle('', fontsize=11)

    def update(i):
        k = frame_steps[i]
        for r, p_trail, e_trail, p_dot, e_dot in panels:
            j = min(k, len(r['p']) - 1)
            p, e = r['p'][:j + 1], r['e'][:j + 1]
            p_trail.set_data(p[:, 0], p[:, 1]); p_trail.set_3d_properties(p[:, 2])
            e_trail.set_data(e[:, 0], e[:, 1]); e_trail.set_3d_properties(e[:, 2])
            p_dot.set_data([p[-1, 0]], [p[-1, 1]]); p_dot.set_3d_properties([p[-1, 2]])
            e_dot.set_data([e[-1, 0]], [e[-1, 1]]); e_dot.set_3d_properties([e[-1, 2]])
        title.set_text(f'S002 3D — pursuer red, evader blue   t = {k * DT:.2f} s')
        return []

    ani = FuncAnimation(fig, update, frames=len(frame_steps), interval=50, blit=False)
    path = os.path.join(OUTPUT_DIR, 'animation.gif')
    ani.save(path, writer=PillowWriter(fps=20), dpi=80)
    plt.close()
    print(f'Saved: {path}')


# ── Main ─────────────────────────────────────────────────────

def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    fns = [strategy_straight, strategy_horizontal_perp, strategy_perp_3d,
           strategy_helix, strategy_dive_and_run]

    results = []
    print('Bounded arena x,y in [-8, 8] m, z in [0.3, 8] m:')
    for name, label, fn in zip(STRATEGY_NAMES, STRATEGY_LABELS, fns):
        p, e, cap, tc, los, wall = run_simulation(fn)
        n = min(len(p), len(e))
        d = np.linalg.norm(p[:n] - e[:n], axis=1)
        status = f'captured at {tc:.2f} s' if cap else 'not captured (timeout)'
        closing = (d[0] - d[-1]) / tc if cap else float('nan')
        print(f'[{name:>16}]  {status}; mean closing {closing:.2f} m/s; '
              f'evader z range [{e[:, 2].min():.2f}, {e[:, 2].max():.2f}] m; '
              f'wall-slide steps {wall}; capture point ({e[-1, 0]:.2f}, {e[-1, 1]:.2f}, {e[-1, 2]:.2f})')
        results.append(dict(name=name, label=label, p=p, e=e, captured=cap, t=tc,
                            los=los, wall=wall))

    _, _, cap_u, t_u, _, _ = run_simulation(strategy_straight, bounded=False)
    print(f'\nStraight escape without the arena (matches the 2D S002 result): {t_u:.2f} s')

    r0 = np.linalg.norm(INIT_EVADER - INIT_PURSUER)
    t_wall = (ARENA - INIT_EVADER[0]) / EVADER_SPEED
    r_wall = r0 - (PURSUER_SPEED - EVADER_SPEED) * t_wall
    print('Analytical checks (r0 = 4 m):')
    print(f'  perpendicular (any direction): (r0 - r_cap)/vP = {(r0 - CAPTURE_R) / PURSUER_SPEED:.2f} s')
    print(f'  straight, no arena: (r0 - r_cap)/(vP - vE) = {(r0 - CAPTURE_R) / (PURSUER_SPEED - EVADER_SPEED):.2f} s')
    print(f'  straight, arena: hits x = 8 wall at {t_wall:.2f} s with range {r_wall:.2f} m, '
          f'lower bound if it then closes at the full vP: '
          f'{t_wall + (r_wall - CAPTURE_R) / PURSUER_SPEED:.2f} s')

    hp = results[1]['e']; p3 = results[2]['e']
    print(f'  horizontal_perp vs perp_3d final evader position: '
          f'({hp[-1, 0]:.2f}, {hp[-1, 1]:.2f}, {hp[-1, 2]:.2f}) vs '
          f'({p3[-1, 0]:.2f}, {p3[-1, 1]:.2f}, {p3[-1, 2]:.2f})')

    print('\nHelix pitch sweep (alpha deg -> capture time s):')
    sweep = []
    for a in PITCH_SWEEP_DEG:
        _, _, cap, tc, _, _ = run_simulation(make_helix(np.radians(a)))
        sweep.append(tc if cap else MAX_TIME)
        print(f'  alpha = {a:2d}°: {sweep[-1]:.2f} s')
    sweep = np.array(sweep)
    fixed_y = lambda pe, pp, v, t: np.array([0.0, v, 0.0])  # noqa: E731
    _, _, _, t_fy, _, _ = run_simulation(fixed_y)
    print(f'  check: constant horizontal heading +y (no climb, no turn): {t_fy:.2f} s '
          f'(same as alpha = 90°, i.e. the gain comes from not turning, not from climbing)')

    print('\nGenerating plots...')
    plot_trajectories_3d(results)
    plot_altitude_time(results)
    plot_capture_times(results, t_u)
    plot_los_angle(results)
    plot_pitch_sweep(sweep, t_fy)
    print('Generating animation (all strategies)...')
    save_animation(results)
    print('\nDone. All outputs saved to:', OUTPUT_DIR)


if __name__ == '__main__':
    main()
