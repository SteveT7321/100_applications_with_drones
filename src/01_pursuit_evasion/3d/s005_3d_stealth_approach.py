"""
S005 3D Upgrade — Stealth Approach
Usage:
    conda activate drones
    python src/01_pursuit_evasion/3d/s005_3d_stealth_approach.py
"""
import sys, os, numpy as np, matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.animation import FuncAnimation, PillowWriter
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..'))
from src.base.drone_base import DroneBase

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), '..', '..', '..',
                          'outputs', '01_pursuit_evasion', '3d', 's005_3d_stealth_approach')
os.makedirs(OUTPUT_DIR, exist_ok=True)

# ── Parameters ──────────────────────────────────────────────
CONE_HALF_ANGLE_DEG = 60.0
DETECT_RANGE        = 5.0       # m
PURSUER_SPEED_CLEAR = 5.5       # m/s
PURSUER_SPEED_DET   = 4.5       # m/s
EVADER_SPEED        = 3.0       # m/s
CAPTURE_R           = 0.15      # m
DT                  = 1 / 48
MAX_TIME            = 20.0
Z_MIN               = 0.3
Z_MAX               = 10.0
REAR_OFFSET         = 2.0       # m
H_CLIMB             = 2.5       # m above evader for high approach
H_DIVE              = -1.5      # m below evader for low approach

INIT_PURSUER = np.array([-4.0, 2.0, 2.0])
INIT_EVADER  = np.array([0.0,  0.0, 2.0])
EVADER_VEL   = np.array([3.0,  0.0, 0.0])   # constant +x

STRATEGIES = ['horiz_flank', 'high_approach', 'low_approach', 'optimal_3d']
STRAT_LABELS = ['Horiz Flank (2D)', 'High Approach', 'Low Approach', 'Optimal 3D']

COS_CONE = np.cos(np.radians(CONE_HALF_ANGLE_DEG))

RNG = np.random.default_rng(0)


# ── Helpers ──────────────────────────────────────────────────

def is_detected(p_pursuer, p_evader, v_evader):
    d = p_pursuer - p_evader
    dist = np.linalg.norm(d)
    if dist < 1e-8 or dist > DETECT_RANGE:
        return False, 0.0
    v_hat = v_evader / (np.linalg.norm(v_evader) + 1e-8)
    cos_alpha = np.dot(d, v_hat) / dist
    return cos_alpha > COS_CONE, cos_alpha


def rear_waypoint(p_evader, v_evader, offset=REAR_OFFSET):
    v_hat = v_evader / (np.linalg.norm(v_evader) + 1e-8)
    return p_evader - offset * v_hat


def project_outside_cone(p_pursuer, p_evader, v_evader):
    """Push the pursuer to the nearest point just outside the detection cone."""
    v_hat = v_evader / (np.linalg.norm(v_evader) + 1e-8)
    d = p_pursuer - p_evader
    dist = np.linalg.norm(d)
    if dist < 1e-8:
        # Go sideways
        return p_evader + np.array([0, 1.0, 0]) * (DETECT_RANGE * 0.5)
    d_hat = d / dist
    # Component along v_hat
    along = np.dot(d_hat, v_hat)
    perp = d_hat - along * v_hat
    perp_len = np.linalg.norm(perp)
    if perp_len < 1e-6:
        # Perfectly aligned: go laterally
        perp = np.array([0, 1.0, 0])
        perp_len = 1.0
    perp_hat = perp / perp_len
    # On the cone boundary: cos_alpha = COS_CONE => along component = COS_CONE, perp = sin
    sin_cone = np.sqrt(max(0, 1 - COS_CONE**2))
    safe_d = (COS_CONE * v_hat + sin_cone * perp_hat)
    # Scale to just outside cone at same distance
    safe_pt = p_evader + dist * safe_d
    safe_pt[2] = np.clip(safe_pt[2], Z_MIN, Z_MAX)
    return safe_pt


def clip_z(pos):
    pos[2] = np.clip(pos[2], Z_MIN, Z_MAX)
    return pos


# ── Simulation ───────────────────────────────────────────────

def run_simulation(strategy):
    pursuer = DroneBase(INIT_PURSUER.copy(), max_speed=PURSUER_SPEED_CLEAR, dt=DT)
    evader  = DroneBase(INIT_EVADER.copy(),  max_speed=EVADER_SPEED, dt=DT)
    v_evader = EVADER_VEL.copy().astype(float)

    max_steps    = int(MAX_TIME / DT)
    captured     = False
    cap_time     = None
    det_log      = []
    cos_alpha_log = []

    for step in range(max_steps):
        t = step * DT
        det, cos_a = is_detected(pursuer.pos, evader.pos, v_evader)
        det_log.append(1 if det else 0)
        cos_alpha_log.append(cos_a)

        # Capture check
        if np.linalg.norm(pursuer.pos - evader.pos) < CAPTURE_R:
            captured  = True
            cap_time  = t
            break

        speed = PURSUER_SPEED_DET if det else PURSUER_SPEED_CLEAR

        # ── Strategy logic ──
        if strategy == 'horiz_flank':
            # Stay at fixed z=2m, circle around to rear horizontally
            target = rear_waypoint(evader.pos, np.array([v_evader[0], v_evader[1], 0.0]))
            target[2] = 2.0
            if det:
                # Sidestep laterally (y-direction)
                lateral = np.array([0.0, np.sign(pursuer.pos[1] - evader.pos[1]) or 1.0, 0.0])
                cmd_dir = lateral
            else:
                cmd_dir = target - pursuer.pos
                n = np.linalg.norm(cmd_dir)
                cmd_dir = cmd_dir / n if n > 1e-8 else cmd_dir

        elif strategy == 'high_approach':
            # Climb to z_E + 2.5m, then approach rear from above
            z_target = evader.pos[2] + H_CLIMB
            # Intermediate: climb while going behind
            mid = rear_waypoint(evader.pos, v_evader)
            mid[2] = np.clip(z_target, Z_MIN, Z_MAX)
            target = mid
            if det:
                target = project_outside_cone(pursuer.pos, evader.pos, v_evader)
            cmd_dir = target - pursuer.pos
            n = np.linalg.norm(cmd_dir)
            cmd_dir = cmd_dir / n if n > 1e-8 else cmd_dir

        elif strategy == 'low_approach':
            # Drop to z_E - 1.5m (min 0.3), approach rear from below
            z_target = max(evader.pos[2] + H_DIVE, Z_MIN)
            mid = rear_waypoint(evader.pos, v_evader)
            mid[2] = z_target
            target = mid
            if det:
                target = project_outside_cone(pursuer.pos, evader.pos, v_evader)
            cmd_dir = target - pursuer.pos
            n = np.linalg.norm(cmd_dir)
            cmd_dir = cmd_dir / n if n > 1e-8 else cmd_dir

        else:  # optimal_3d
            if det:
                target = project_outside_cone(pursuer.pos, evader.pos, v_evader)
            else:
                target = rear_waypoint(evader.pos, v_evader)
            cmd_dir = target - pursuer.pos
            n = np.linalg.norm(cmd_dir)
            cmd_dir = cmd_dir / n if n > 1e-8 else cmd_dir

        pursuer.step(cmd_dir * speed)
        clip_z(pursuer.pos)

        # Evader: constant velocity
        evader.step(v_evader)

    return (
        pursuer.get_trajectory(),
        evader.get_trajectory(),
        captured,
        cap_time,
        np.array(det_log),
        np.array(cos_alpha_log),
    )


# ── Cone wireframe helper ────────────────────────────────────

def cone_wireframe(apex, axis, half_deg, length=DETECT_RANGE, n=20):
    """Return arrays suitable for plot_wireframe."""
    axis = axis / (np.linalg.norm(axis) + 1e-8)
    r = length * np.tan(np.radians(half_deg))
    theta = np.linspace(0, 2 * np.pi, n)
    cx = r * np.cos(theta)
    cy = r * np.sin(theta)
    cz = np.full(n, length)

    z_ref = np.array([0.0, 0.0, 1.0])
    cross = np.cross(z_ref, axis)
    cn = np.linalg.norm(cross)
    if cn < 1e-6:
        R = np.eye(3) if np.dot(z_ref, axis) > 0 else np.diag([1.0, 1.0, -1.0])
    else:
        k = cross / cn
        c = np.dot(z_ref, axis); s = cn
        K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
        R = np.eye(3) + s * K + (1 - c) * (K @ K)

    pts = R @ np.vstack([cx, cy, cz])
    X = np.column_stack([np.full(n, apex[0]), apex[0] + pts[0]])
    Y = np.column_stack([np.full(n, apex[1]), apex[1] + pts[1]])
    Z = np.column_stack([np.full(n, apex[2]), apex[2] + pts[2]])
    return X, Y, Z


# ── Plots ────────────────────────────────────────────────────

def plot_trajectories_3d(all_results, out_dir):
    fig = plt.figure(figsize=(18, 12))
    for idx, (res, label) in enumerate(zip(all_results, STRAT_LABELS)):
        p_traj, e_traj, captured, cap_t, det_log, _ = res
        ax = fig.add_subplot(2, 2, idx + 1, projection='3d')
        ax.view_init(elev=25, azim=-50)

        # Pursuer trajectory
        ax.plot(p_traj[:, 0], p_traj[:, 1], p_traj[:, 2],
                color='red', linewidth=1.8, label='Pursuer')
        # Evader trajectory
        ax.plot(e_traj[:, 0], e_traj[:, 1], e_traj[:, 2],
                color='royalblue', linewidth=1.8, label='Evader')

        ax.scatter(*p_traj[0], color='red',  s=60, marker='o')
        ax.scatter(*e_traj[0], color='blue', s=60, marker='o')

        if captured:
            ax.scatter(*p_traj[-1], color='black', s=120, marker='X',
                       zorder=5, label=f'Captured {cap_t:.2f}s')
            status = f'Captured {cap_t:.2f} s'
        else:
            ax.scatter(*p_traj[-1], color='grey', s=80, marker='X', zorder=5)
            status = 'Timeout'

        # Draw detection cone at final evader position
        e_last = e_traj[-1]
        v_hat = EVADER_VEL / np.linalg.norm(EVADER_VEL)
        X, Y, Z = cone_wireframe(e_last, v_hat, CONE_HALF_ANGLE_DEG, length=DETECT_RANGE)
        ax.plot_wireframe(X, Y, Z, color='gold', alpha=0.25, linewidth=0.6)

        ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
        ax.set_title(f'{label}\n{status}', fontsize=10)
        ax.legend(fontsize=7, loc='upper left')

    fig.suptitle('S005 3D Stealth Approach — Trajectories\n'
                 '(gold wireframe = detection cone at final evader position)',
                 fontsize=12)
    plt.tight_layout()
    path = os.path.join(out_dir, 'trajectories_3d.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_detection_status(all_results, out_dir):
    fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
    colors = ['steelblue', 'darkorange', 'mediumseagreen', 'mediumpurple']
    for ax, res, label, color in zip(axes, all_results, STRAT_LABELS, colors):
        p_traj, e_traj, captured, cap_t, det_log, _ = res
        t_ax = np.arange(len(det_log)) * DT
        ax.fill_between(t_ax, det_log, step='mid', color=color, alpha=0.5,
                        label='Detected')
        if captured:
            ax.axvline(cap_t, color='black', linestyle='--', linewidth=1.2,
                       label=f'Captured {cap_t:.2f}s')
        pct = 100 * det_log.mean()
        ax.set_title(f'{label} — in-cone {pct:.1f}% of sim', fontsize=9)
        ax.set_ylim(-0.1, 1.4)
        ax.set_ylabel('Detected')
        ax.legend(fontsize=7, loc='upper right')
        ax.grid(True, alpha=0.3)

    axes[-1].set_xlabel('Time (s)')
    fig.suptitle('S005 3D Stealth Approach — Detection Status vs Time', fontsize=11)
    plt.tight_layout()
    path = os.path.join(out_dir, 'detection_status.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def plot_approach_angle(all_results, out_dir):
    fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
    colors = ['steelblue', 'darkorange', 'mediumseagreen', 'mediumpurple']
    cone_thresh = np.degrees(np.arccos(COS_CONE))

    for ax, res, label, color in zip(axes, all_results, STRAT_LABELS, colors):
        p_traj, e_traj, captured, cap_t, det_log, cos_alpha_log = res
        t_ax = np.arange(len(cos_alpha_log)) * DT
        alpha_log = np.degrees(np.arccos(np.clip(cos_alpha_log, -1.0, 1.0)))

        ax.plot(t_ax, alpha_log, color=color, linewidth=1.5, label=label)
        ax.axhline(cone_thresh, color='gold', linestyle='--', linewidth=1.5,
                   label=f'Cone boundary {cone_thresh:.0f}°')
        ax.fill_between(t_ax, 0, alpha_log,
                        where=(alpha_log < cone_thresh),
                        color='red', alpha=0.15, label='Inside cone')
        if captured:
            ax.axvline(cap_t, color='black', linestyle='--', linewidth=1.0,
                       label=f'Captured {cap_t:.2f}s')
        ax.set_ylabel('α₃D (°)')
        ax.set_ylim(0, 185)
        ax.legend(fontsize=7, loc='upper right')
        ax.set_title(label, fontsize=9)
        ax.grid(True, alpha=0.3)

    axes[-1].set_xlabel('Time (s)')
    fig.suptitle('S005 3D Stealth Approach — Approach Angle vs Time', fontsize=11)
    plt.tight_layout()
    path = os.path.join(out_dir, 'approach_angle.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def plot_capture_times(all_results, out_dir):
    colors = ['steelblue', 'darkorange', 'mediumseagreen', 'mediumpurple']
    cap_times, bar_colors, anns = [], [], []
    for res, color in zip(all_results, colors):
        _, _, captured, cap_t, _, _ = res
        if captured:
            cap_times.append(cap_t)
            bar_colors.append(color)
            anns.append(f'{cap_t:.2f} s')
        else:
            cap_times.append(MAX_TIME)
            bar_colors.append('tomato')
            anns.append('Timeout')

    fig, ax = plt.subplots(figsize=(9, 5))
    bars = ax.bar(STRAT_LABELS, cap_times, color=bar_colors, edgecolor='black', width=0.5)
    for bar, ann in zip(bars, anns):
        ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.2,
                ann, ha='center', va='bottom', fontsize=10, fontweight='bold')
    ax.set_ylabel('Capture Time (s)')
    ax.set_title('S005 3D Stealth Approach — Capture Times by Strategy', fontsize=11)
    ax.set_ylim(0, MAX_TIME * 1.25)
    ax.grid(axis='y', alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'capture_times.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def save_animation(all_results, out_dir):
    """Animate optimal_3d strategy with moving detection cone."""
    res = all_results[3]  # optimal_3d
    p_traj, e_traj, captured, cap_t, det_log, _ = res

    step_skip = 3
    p_frames = p_traj[::step_skip]
    e_frames = e_traj[::step_skip]
    d_frames = det_log[::step_skip]
    n_frames = min(len(p_frames), len(e_frames))

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    all_pts = np.vstack([p_traj, e_traj])
    lo = all_pts.min(axis=0) - 1.5
    hi = all_pts.max(axis=0) + 1.5
    ax.set_xlim(lo[0], hi[0])
    ax.set_ylim(lo[1], hi[1])
    ax.set_zlim(max(0, lo[2]), hi[2])
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
    ax.view_init(elev=25, azim=-50)

    p_line, = ax.plot([], [], [], 'r-', linewidth=1.8, label='Pursuer')
    e_line, = ax.plot([], [], [], 'b-', linewidth=1.8, label='Evader')
    p_dot,  = ax.plot([], [], [], 'r^', markersize=10, zorder=8)
    e_dot,  = ax.plot([], [], [], 'bs', markersize=10, zorder=8)
    ax.legend(fontsize=9, loc='upper left')
    title = ax.set_title('')

    cone_wires = []

    def update(i):
        nonlocal cone_wires
        # Remove old cone
        for w in cone_wires:
            w.remove()
        cone_wires = []

        si = min(i, len(p_frames) - 1)
        p_line.set_data(p_frames[:si+1, 0], p_frames[:si+1, 1])
        p_line.set_3d_properties(p_frames[:si+1, 2])
        e_line.set_data(e_frames[:si+1, 0], e_frames[:si+1, 1])
        e_line.set_3d_properties(e_frames[:si+1, 2])
        p_dot.set_data([p_frames[si, 0]], [p_frames[si, 1]])
        p_dot.set_3d_properties([p_frames[si, 2]])
        e_dot.set_data([e_frames[si, 0]], [e_frames[si, 1]])
        e_dot.set_3d_properties([e_frames[si, 2]])

        # Draw cone at current evader position
        v_hat = EVADER_VEL / np.linalg.norm(EVADER_VEL)
        X, Y, Z = cone_wireframe(e_frames[si], v_hat, CONE_HALF_ANGLE_DEG, length=DETECT_RANGE)
        w = ax.plot_wireframe(X, Y, Z, color='gold', alpha=0.25, linewidth=0.5)
        cone_wires.append(w)

        detected = bool(d_frames[si]) if si < len(d_frames) else False
        t = si * step_skip * DT
        status = '[IN CONE]' if detected else '[Stealth]'
        p_line.set_color('tomato' if detected else 'red')
        title.set_text(f'S005 3D Optimal Stealth  t={t:.2f}s  {status}')
        return p_line, e_line, p_dot, e_dot, title

    ani = FuncAnimation(fig, update, frames=n_frames, interval=80, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer=PillowWriter(fps=12), dpi=90)
    plt.close()
    print(f'Saved: {path}')


# ── Main ─────────────────────────────────────────────────────

if __name__ == '__main__':
    all_results = []
    for strat, label in zip(STRATEGIES, STRAT_LABELS):
        res = run_simulation(strat)
        _, _, captured, cap_t, det_log, _ = res
        det_pct = 100 * det_log.mean()
        status = f'captured {cap_t:.2f}s' if captured else 'timeout'
        print(f'[{label:<20}]  {status}  | in-cone {det_pct:.1f}%')
        all_results.append(res)

    out_dir = os.path.normpath(OUTPUT_DIR)
    plot_trajectories_3d(all_results, out_dir)
    plot_detection_status(all_results, out_dir)
    plot_approach_angle(all_results, out_dir)
    plot_capture_times(all_results, out_dir)
    save_animation(all_results, out_dir)
    print('S005 3D done.')
