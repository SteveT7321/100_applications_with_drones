"""
S005 Stealth Approach
======================
Pursuer tries to reach the evader without entering the evader's forward
detection cone, then closes from the rear blind spot.

Setup:
  - Evader at (0,0,2) patrolling +x at 3 m/s.  Its detection cone
    (half-angle 60 deg, range 5 m) points along its velocity.
  - Pursuer starts ahead of the evader at (6, 2, 2): outside detection range
    (6.32 m) but near the cone axis, so a straight approach must cross the cone.
  - The FIRST detection alerts the evader (latched); it then flees directly
    away from the pursuer at 3 m/s for the rest of the run.

Same rules for both strategies:
  - Pursuer speed 5.5 m/s while undetected and the evader is not alerted,
    4.5 m/s when detected / once alerted.

Comparison:
  - Stealth flanking : aims for a waypoint 2 m behind the evader; each step it
    takes the heading closest to that aim whose predicted next position is
    outside the cone (5 deg heading search).
  - Direct frontal   : always flies straight at the evader.

Usage:
    conda activate drones
    python src/01_pursuit_evasion/s005_stealth_approach.py
"""

import sys
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from src.base.drone_base import DroneBase

# ── Scenario parameters (speeds, cone, offset, max time as in the card) ──
# Speed rules are IDENTICAL for both pursuer strategies:
#   pursuer flies PURSUER_SPEED_STEALTH while undetected and the evader is
#   not alerted, PURSUER_SPEED_DETECTED otherwise.
PURSUER_SPEED_STEALTH  = 4.5 + 1.0   # m/s, bonus speed while undetected
PURSUER_SPEED_DETECTED = 4.5         # m/s, when detected / once evader is alerted
EVADER_SPEED_NORMAL    = 3.0         # m/s constant heading (patrol)
EVADER_SPEED_ALERT     = 3.0         # m/s, same speed — only direction changes
CONE_HALF_ANGLE_DEG    = 60.0
DETECT_RANGE           = 5.0         # m
CAPTURE_R              = 0.15        # m
BEHIND_OFFSET          = 2.0         # m behind evader for stealth waypoint
HEADING_STEP_DEG       = 5.0         # resolution of the stealth heading search
DT                     = 1 / 48
MAX_TIME               = 20.0

# Pursuer starts AHEAD of the evader, outside detection range (6.32 m) but
# on the cone axis side (alpha = 18.4 deg), so a straight approach must pass
# through the cone. (The card's (-4, 2, 2) is already behind the evader,
# where a direct approach is never detected and both strategies coincide.)
INIT_PURSUER   = np.array([6.0, 2.0, 2.0])
INIT_EVADER    = np.array([0.0, 0.0, 2.0])
CARD_INIT_PURSUER = np.array([-4.0, 2.0, 2.0])   # card's start, run as a check
EVADER_HEADING = np.array([1.0, 0.0, 0.0])   # constant patrol heading +x

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '01_pursuit_evasion', 's005_stealth_approach',
)


# ── Detection helpers ───────────────────────────────────────

def is_detected(pos_p, pos_e, vel_e, cone_deg, r_detect):
    """True if pursuer is inside evader's forward detection cone (range + angle)."""
    diff = pos_p - pos_e
    dist = np.linalg.norm(diff)
    if dist > r_detect:
        return False
    v_hat = vel_e / (np.linalg.norm(vel_e) + 1e-8)
    cos_alpha = np.dot(diff, v_hat) / (dist + 1e-8)
    alpha_deg = np.degrees(np.arccos(np.clip(cos_alpha, -1.0, 1.0)))
    return alpha_deg < cone_deg


def behind_waypoint(pos_e, vel_e, offset=BEHIND_OFFSET):
    """Point offset behind the evader along its heading."""
    v_hat = vel_e / (np.linalg.norm(vel_e) + 1e-8)
    return pos_e - offset * v_hat


def stealth_heading(pos_p, pos_e, vel_e, desired_dir, speed):
    """
    Cone-avoiding heading: among horizontal headings (every HEADING_STEP_DEG),
    pick the one closest to desired_dir whose predicted next pursuer position
    (evader advanced one step along its patrol heading) is NOT detected.
    Falls back to desired_dir if every heading would be detected.
    """
    e_next = pos_e + vel_e / (np.linalg.norm(vel_e) + 1e-8) * EVADER_SPEED_NORMAL * DT
    best, best_dot = None, -np.inf
    for ang in np.radians(np.arange(0.0, 360.0, HEADING_STEP_DEG)):
        d = np.array([np.cos(ang), np.sin(ang), 0.0])
        if is_detected(pos_p + d * speed * DT, e_next, vel_e,
                       CONE_HALF_ANGLE_DEG, DETECT_RANGE):
            continue
        dot = np.dot(d, desired_dir)
        if dot > best_dot:
            best, best_dot = d, dot
    return desired_dir if best is None else best


# ── Simulation ──────────────────────────────────────────────

def run_simulation(stealth_mode, init_pursuer=INIT_PURSUER):
    """
    stealth_mode=True  → aim for the behind-waypoint (BEHIND_OFFSET m behind the
                         evader); each step choose the heading closest to that
                         aim whose predicted next position is outside the cone.
                         Within BEHIND_OFFSET + 0.5 m of the evader, aim at the
                         evader itself (still cone-avoiding).
    stealth_mode=False → direct: always fly straight at the evader.

    Same rules for both strategies:
      - pursuer speed = PURSUER_SPEED_STEALTH if not detected and evader not
        alerted, else PURSUER_SPEED_DETECTED;
      - the first detection alerts the evader (latched): it then flees
        directly away from the pursuer at EVADER_SPEED_ALERT for the rest
        of the run.
    """
    pursuer = DroneBase(np.array(init_pursuer, dtype=float), max_speed=PURSUER_SPEED_STEALTH, dt=DT)
    evader  = DroneBase(INIT_EVADER.copy(),
                        max_speed=max(EVADER_SPEED_NORMAL, EVADER_SPEED_ALERT), dt=DT)

    vel_e       = EVADER_HEADING.copy().astype(float)
    alerted     = False
    alarm_step  = None        # timestep when alarm latched
    max_steps   = int(MAX_TIME / DT)

    captured     = False
    capture_time = None
    detected_log = []         # per-step 0/1
    alerted_log  = []         # per-step 0/1 (latched after first detection)
    speed_log    = []         # pursuer commanded speed per step

    for step in range(max_steps):
        t = step * DT

        # ── Momentary detection (only meaningful while patrolling) ──
        detected_now = (not alerted) and is_detected(
            pursuer.pos, evader.pos, vel_e, CONE_HALF_ANGLE_DEG, DETECT_RANGE)
        detected_log.append(1 if detected_now else 0)

        # ── First detection alerts the evader (latched) ──
        if detected_now and not alerted:
            alerted    = True
            alarm_step = step
        alerted_log.append(1 if alerted else 0)

        # ── Capture check ──
        if np.linalg.norm(pursuer.pos - evader.pos) < CAPTURE_R:
            captured     = True
            capture_time = t
            break

        # ── Pursuer speed: identical rule for both strategies ──
        speed = PURSUER_SPEED_DETECTED if (detected_now or alerted) else PURSUER_SPEED_STEALTH
        speed_log.append(speed)

        # ── Pursuer heading ──
        to_e = evader.pos - pursuer.pos
        if stealth_mode and not alerted:
            if np.linalg.norm(to_e) > BEHIND_OFFSET + 0.5:
                target = behind_waypoint(evader.pos, vel_e)
            else:
                target = evader.pos            # close enough — finish directly
            desired = target - pursuer.pos
            desired = desired / (np.linalg.norm(desired) + 1e-8)
            cmd_dir = stealth_heading(pursuer.pos, evader.pos, vel_e, desired, speed)
        else:
            # direct pursuit (and stealth pursuer once the evader is alerted)
            cmd_dir = to_e / (np.linalg.norm(to_e) + 1e-8)

        pursuer.step(cmd_dir * speed)

        # ── Evader velocity ──
        if alerted:
            flee = evader.pos - pursuer.pos
            n    = np.linalg.norm(flee)
            if n > 1e-6:
                vel_e = flee / n
            evader.step(vel_e * EVADER_SPEED_ALERT)
        else:
            vel_e = EVADER_HEADING.copy().astype(float)
            evader.step(vel_e * EVADER_SPEED_NORMAL)

    alarm_time = alarm_step * DT if alarm_step is not None else None
    return (
        pursuer.get_trajectory(),
        evader.get_trajectory(),
        captured,
        capture_time,
        alerted,
        alarm_time,
        np.array(detected_log),
        np.array(alerted_log),
        np.array(speed_log),
    )


# ── Plots ───────────────────────────────────────────────────

# 每個子圖各自算軸範圍，避免 timeout 那條軌跡把軸撐爆
def _traj_limits(p_traj, e_traj, margin=2.0):
    pts = np.vstack([p_traj, e_traj])
    lo  = pts.min(axis=0) - margin
    hi  = pts.max(axis=0) + margin
    return lo, hi


def _draw_cone_2d(ax, apex, heading, half_angle_deg, length, color='gold', alpha=0.3):
    """在 XY 俯瞰圖上畫偵測錐（扇形）。"""
    h = heading / (np.linalg.norm(heading) + 1e-8)
    angle0 = np.arctan2(h[1], h[0])
    half   = np.radians(half_angle_deg)
    angles = np.linspace(angle0 - half, angle0 + half, 40)
    xs = [apex[0]] + [apex[0] + length * np.cos(a) for a in angles] + [apex[0]]
    ys = [apex[1]] + [apex[1] + length * np.sin(a) for a in angles] + [apex[1]]
    ax.fill(xs, ys, color=color, alpha=alpha, zorder=2)
    ax.plot(xs, ys, color='goldenrod', linewidth=1.0, zorder=3)


def plot_trajectories_3d(results, out_dir):
    """俯瞰 XY 圖（z=2 平面）+ 右側小 3D 圖，清楚展示側翼路徑。"""
    labels = ['Stealth Flanking', 'Direct Frontal']

    fig = plt.figure(figsize=(14, 10))
    gs = fig.add_gridspec(2, 2, width_ratios=[2.2, 1])
    snap_times = [0.0, 0.6, 1.2]          # cone snapshots (cone moves with evader)
    snap_alpha = [0.30, 0.18, 0.10]

    for i, (label, res) in enumerate(zip(labels, results)):
        p_traj, e_traj, captured, cap_t, alerted, alarm_t, det_log, _, _ = res

        pt, et = p_traj, e_traj

        # ── 俯瞰 XY ──────────────────────────────────────
        ax = fig.add_subplot(gs[i, 0])

        # Detection cone snapshots while the evader is still patrolling
        # (after the alarm the evader flees and detection no longer matters)
        for ts, al in zip(snap_times, snap_alpha):
            k = int(round(ts / DT))
            if k >= len(et) or (alarm_t is not None and ts > alarm_t):
                continue
            _draw_cone_2d(ax, et[k], EVADER_HEADING,
                          CONE_HALF_ANGLE_DEG, DETECT_RANGE, alpha=al)
            ax.scatter(*pt[k, :2], color='red', s=25, marker='o',
                       edgecolor='black', zorder=9)
            ax.scatter(*et[k, :2], color='blue', s=25, marker='o',
                       edgecolor='black', zorder=9)
            ax.annotate(f't={ts:.1f}s', pt[k, :2], textcoords='offset points',
                        xytext=(5, 5), fontsize=7, color='darkred')

        # 偵測圈 (at t = 0)
        circle = plt.Circle(et[0, :2], DETECT_RANGE,
                             fill=False, color='goldenrod', linestyle='--',
                             linewidth=1.0, alpha=0.5)
        ax.add_patch(circle)

        # 軌跡（顏色標記是否被偵測到）
        dl = det_log[:len(pt)]
        for k in range(len(pt) - 1):
            col = 'tomato' if (k < len(dl) and dl[k]) else 'red'
            ax.plot(pt[k:k+2, 0], pt[k:k+2, 1], color=col,
                    linewidth=2.2, zorder=5, solid_capstyle='round')
        ax.plot(et[:, 0], et[:, 1], color='royalblue',
                linewidth=2.0, zorder=4, label='Evader')

        ax.scatter(*pt[0, :2],  color='red',   s=80, zorder=7, label='P start')
        ax.scatter(*et[0, :2],  color='blue',  s=80, zorder=7, label='E start')

        # 方向箭頭
        ax.annotate('', xy=et[0, :2] + np.array([1.5, 0]),
                    xytext=et[0, :2],
                    arrowprops=dict(arrowstyle='->', color='royalblue', lw=1.5))

        alarm_str = (f'alarm @ {alarm_t:.2f} s' if alarm_t is not None
                     else 'no alarm')
        if captured:
            ax.scatter(*pt[-1, :2], color='black', s=150, marker='X', zorder=8,
                       label=f'Captured {cap_t:.2f}s')
            status = f'{alarm_str}, captured {cap_t:.2f} s'
        else:
            status = f'{alarm_str} → timeout'
            ax.scatter(*pt[-1, :2], color='grey', s=100, marker='X', zorder=8)

        lo, hi = _traj_limits(pt, et, margin=1.5)
        ax.set_xlim(lo[0], hi[0]); ax.set_ylim(lo[1], hi[1])
        ax.set_aspect('equal'); ax.grid(True, alpha=0.3)
        ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
        ax.set_title(f'{label}\n{status}', fontsize=10)
        ax.legend(fontsize=7, loc='upper left')

        # ── 小 3D 圖 ─────────────────────────────────────
        ax3 = fig.add_subplot(gs[i, 1], projection='3d')
        ax3.view_init(elev=30, azim=-50)

        ax3.plot(pt[:, 0], pt[:, 1], pt[:, 2],
                 color='red',  linewidth=1.8, label='Pursuer')
        ax3.plot(et[:, 0], et[:, 1], et[:, 2],
                 color='blue', linewidth=1.8, label='Evader')
        ax3.scatter(*pt[0],  color='red',  s=50, marker='o')
        ax3.scatter(*et[0],  color='blue', s=50, marker='o')
        if captured:
            ax3.scatter(*pt[-1], color='black', s=100, marker='X')

        lo3, hi3 = _traj_limits(pt, et, margin=1.0)
        ax3.set_xlim(lo3[0], hi3[0]); ax3.set_ylim(lo3[1], hi3[1])
        ax3.set_zlim(max(0, lo3[2]), hi3[2])
        ax3.set_xlabel('X'); ax3.set_ylabel('Y'); ax3.set_zlabel('Z')
        ax3.set_title('3D View', fontsize=8)

    fig.suptitle('S005 Stealth Approach — Trajectories\n'
                 '(gold = detection cone at t = 0 / 0.6 / 1.2 s, dots = positions at those times  |  light red = pursuer detected)',
                 fontsize=11)
    plt.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'trajectories_3d.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_detection_status(results, out_dir):
    labels = ['Stealth Flanking', 'Direct Frontal']
    colors = ['steelblue', 'darkorange']

    fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    for ax, label, color, res in zip(axes, labels, colors, results):
        p_traj, e_traj, captured, cap_t, alerted, alarm_t, det_log, alert_log, _ = res
        t_ax = np.arange(len(det_log)) * DT

        ax.fill_between(t_ax, det_log, step='mid', color=color, alpha=0.4,
                        label='Momentary detection')
        ax.fill_between(t_ax, alert_log, step='mid', color='red', alpha=0.2,
                        label='Evader alerted (latched)')
        if alarm_t is not None:
            ax.axvline(alarm_t, color='red', linestyle='-', linewidth=1.5,
                       label=f'ALARM @ {alarm_t:.2f}s')
        if captured:
            ax.axvline(cap_t, color='black', linestyle='--', linewidth=1.2,
                       label=f'Captured {cap_t:.2f}s')

        pct = 100 * det_log.mean()
        ax.set_title(f'{label} — in-cone {pct:.1f}% of sim time', fontsize=9)
        ax.set_ylim(-0.1, 1.4)
        ax.set_ylabel('Detection / Alarm')
        ax.legend(fontsize=7, loc='upper right')
        ax.grid(True, alpha=0.3)

    axes[-1].set_xlabel('Time (s)')
    fig.suptitle('S005 Stealth Approach — Detection Status vs Time', fontsize=11)
    plt.tight_layout()
    path = os.path.join(out_dir, 'detection_status.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def plot_distance_time(results, out_dir):
    labels = ['Stealth Flanking', 'Direct Frontal']
    colors = ['steelblue', 'darkorange']

    fig, ax = plt.subplots(figsize=(11, 5))
    for label, color, res in zip(labels, colors, results):
        p_traj, e_traj, captured, cap_t, _, alarm_t, _, _, _ = res
        n     = min(len(p_traj), len(e_traj))
        dists = np.linalg.norm(p_traj[:n] - e_traj[:n], axis=1)
        times = np.arange(n) * DT
        ax.plot(times, dists, color=color, linewidth=1.8, label=label)
        if captured:
            ax.axvline(cap_t, color=color, linestyle='--', linewidth=1, alpha=0.8)
        if alarm_t is not None:
            ax.axvline(alarm_t, color=color, linestyle=':', linewidth=1.5,
                       alpha=0.9, label=f'{label[:7]} alarm {alarm_t:.2f}s')

    ax.axhline(CAPTURE_R, color='red', linestyle='--', linewidth=1,
               label=f'Capture radius {CAPTURE_R} m')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Pursuer–Evader Distance (m)')
    ax.set_title('S005 Stealth Approach — Distance vs Time')
    ax.legend()
    ax.grid(True, alpha=0.3)

    path = os.path.join(out_dir, 'distance_time.png')
    plt.tight_layout()
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def plot_comparison_bar(results, out_dir):
    labels = ['Stealth\nFlanking', 'Direct\nFrontal']
    colors = ['steelblue', 'darkorange']
    cap_times = []
    det_pcts  = []
    alerted_flags = []
    for res in results:
        _, _, captured, cap_t, alerted_flag, _, det_log, _, _ = res
        cap_times.append(cap_t if captured else MAX_TIME)
        det_pcts.append(100 * det_log.mean())
        alerted_flags.append(alerted_flag)

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 5))

    bars = ax1.bar(labels, cap_times, color=colors, width=0.4, edgecolor='black')
    for bar, t, res, af in zip(bars, cap_times, results, alerted_flags):
        txt = f'{t:.2f} s' if res[2] else f'>{t:.0f} s\n(timeout)'
        ax1.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.3,
                 txt, ha='center', va='bottom', fontsize=9)
    ax1.set_ylabel('Capture Time (s)')
    ax1.set_title('Capture Time')
    ax1.set_ylim(0, MAX_TIME * 1.2)
    ax1.grid(axis='y', alpha=0.3)

    bars2 = ax2.bar(labels, det_pcts, color=colors, width=0.4, edgecolor='black')
    for bar, pct in zip(bars2, det_pcts):
        ax2.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.5,
                 f'{pct:.1f}%', ha='center', va='bottom', fontsize=10)
    ax2.set_ylabel('Time in Detection Cone (%)')
    ax2.set_title('Detection Exposure')
    ax2.set_ylim(0, 110)
    ax2.grid(axis='y', alpha=0.3)

    fig.suptitle('S005 Stealth Approach — Comparison', fontsize=11)
    plt.tight_layout()
    path = os.path.join(out_dir, 'comparison_bar.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def save_animation(results, out_dir):
    """Top-down XY animation of the stealth case; the detection cone (gold) moves with the evader."""
    import matplotlib.animation as animation

    p_traj, e_traj, captured, cap_t, alerted, alarm_t, det_log, alert_log, _ = results[0]
    lo, hi = _traj_limits(p_traj, e_traj, margin=DETECT_RANGE)

    step     = 2
    p_frames = p_traj[::step]
    e_frames = e_traj[::step]
    d_frames = det_log[::step]
    n_frames = min(len(p_frames), len(e_frames))

    fig, ax = plt.subplots(figsize=(9, 7))
    ax.set_xlim(lo[0], hi[0]); ax.set_ylim(lo[1], hi[1])
    ax.set_aspect('equal'); ax.grid(True, alpha=0.3)
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')

    # Detection cone (sector) and range circle, updated every frame
    from matplotlib.patches import Polygon
    half = np.radians(CONE_HALF_ANGLE_DEG)

    def _sector(apex, heading):
        a0 = np.arctan2(heading[1], heading[0])
        angs = np.linspace(a0 - half, a0 + half, 40)
        pts = [apex[:2]] + [apex[:2] + DETECT_RANGE * np.array([np.cos(a), np.sin(a)])
                            for a in angs]
        return np.array(pts)

    e_head = np.diff(e_traj, axis=0)
    e_head = np.vstack([e_head, e_head[-1:]])[::step]
    cone_patch = Polygon(_sector(e_frames[0], EVADER_HEADING), closed=True,
                         color='gold', alpha=0.25, zorder=2,
                         label=f'Detection cone ({CONE_HALF_ANGLE_DEG:.0f} deg, {DETECT_RANGE:.0f} m)')
    ax.add_patch(cone_patch)
    circle = plt.Circle(e_frames[0, :2], DETECT_RANGE,
                        fill=False, color='goldenrod', linestyle='--',
                        linewidth=1.2, alpha=0.6, label=f'Detection range {DETECT_RANGE}m')
    ax.add_patch(circle)

    # 起點標記
    ax.scatter(*p_traj[0, :2], color='red',  s=80, zorder=7,
               marker='o', label='P start')
    ax.scatter(*e_traj[0, :2], color='blue', s=80, zorder=7,
               marker='o', label='E start')
    ax.legend(fontsize=8, loc='upper left')

    p_trail, = ax.plot([], [], color='red',  linewidth=1.8, alpha=0.7)
    e_trail, = ax.plot([], [], color='blue', linewidth=1.8, alpha=0.7)
    p_dot,   = ax.plot([], [], 'r^', markersize=10, zorder=8)
    e_dot,   = ax.plot([], [], 'bs', markersize=10, zorder=8)
    title_txt = ax.set_title('')

    def update(i):
        px = p_frames[:i+1, 0]; py = p_frames[:i+1, 1]
        ex = e_frames[:i+1, 0]; ey = e_frames[:i+1, 1]
        p_trail.set_data(px, py)
        e_trail.set_data(ex, ey)
        p_dot.set_data([float(px[-1])], [float(py[-1])])
        e_dot.set_data([float(ex[-1])], [float(ey[-1])])
        detected = bool(d_frames[i]) if i < len(d_frames) else False
        t = i * step * DT
        status = '[IN CONE]' if detected else '[Clear]'
        p_trail.set_color('tomato' if detected else 'red')
        cone_patch.set_xy(_sector(e_frames[i], e_head[i]))
        circle.center = (float(e_frames[i, 0]), float(e_frames[i, 1]))
        title_txt.set_text(f'S005 Stealth Approach  t={t:.2f}s  {status}')
        return p_trail, e_trail, p_dot, e_dot, title_txt, cone_patch, circle

    ani = animation.FuncAnimation(fig, update, frames=n_frames, interval=70, blit=True)

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer='pillow', fps=14, dpi=100)
    plt.close()
    print(f'Saved: {path}')


# ── Main ────────────────────────────────────────────────────

if __name__ == '__main__':
    cases = [
        ('Stealth flanking approach', True),
        ('Direct frontal approach',   False),
    ]

    results = []
    for label, stealth in cases:
        res = run_simulation(stealth)
        captured, cap_t, alerted_flag, alarm_t, det_log = res[2], res[3], res[4], res[5], res[6]
        det_pct = 100 * det_log.mean()
        if captured:
            status = f'captured {cap_t:.2f} s'
        else:
            status = f'TIMEOUT (alarm @ {alarm_t:.2f}s)' if alarm_t is not None else 'timeout (no alarm)'
        print(f'[{label:<28}]  {status}  |  in-cone {det_pct:.1f}% of sim')
        p_traj, e_traj, speed_log = res[0], res[1], res[8]
        n = min(len(p_traj), len(e_traj))
        d = np.linalg.norm(p_traj[:n] - e_traj[:n], axis=1)
        p_len = np.sum(np.linalg.norm(np.diff(p_traj, axis=0), axis=1))
        e_len = np.sum(np.linalg.norm(np.diff(e_traj, axis=0), axis=1))
        print(f'{"":>32}alarm: {"t = %.2f s" % alarm_t if alarm_t is not None else "never"}'
              f'  |  in-cone steps {int(det_log.sum())}'
              f'  |  time at {PURSUER_SPEED_STEALTH} m/s: '
              f'{np.sum(speed_log == PURSUER_SPEED_STEALTH) * DT:.2f} s')
        print(f'{"":>32}pursuer path {p_len:.2f} m, evader path {e_len:.2f} m, '
              f'min distance {d.min():.3f} m')
        if alarm_t is not None:
            k_al = int(round(alarm_t / DT))
            print(f'{"":>32}distance at alarm {d[k_al]:.2f} m -> pure chase at '
                  f'{PURSUER_SPEED_DETECTED} vs {EVADER_SPEED_ALERT} m/s needs '
                  f'~{(d[k_al] - CAPTURE_R) / (PURSUER_SPEED_DETECTED - EVADER_SPEED_ALERT):.2f} s more')
        # Closing speed once the pursuer is behind the evader (bearing > 90 deg)
        e_v = np.diff(e_traj[:n], axis=0)
        rel = p_traj[:n - 1] - e_traj[:n - 1]
        bear = np.degrees(np.arccos(np.clip(
            np.sum(rel * e_v, axis=1) /
            (np.linalg.norm(rel, axis=1) * np.linalg.norm(e_v, axis=1) + 1e-12), -1, 1)))
        behind = np.nonzero(bear > 90.0)[0]
        if captured and len(behind):
            k0 = behind[0]
            n_last = 6
            print(f'{"":>32}behind the evader (bearing > 90 deg) from t = {k0 * DT:.2f} s: '
                  f'mean closing speed {(d[k0] - d[n - 1]) / ((n - 1 - k0) * DT):.2f} m/s; '
                  f'last {n_last * DT:.3f} s: '
                  f'{(d[n - 1 - n_last] - d[n - 1]) / (n_last * DT):.2f} m/s')
        # Bearing of the pursuer relative to the evader's heading at capture
        if captured and n >= 2:
            e_vel = e_traj[n - 1] - e_traj[n - 2]
            rel = p_traj[n - 1] - e_traj[n - 1]
            cosb = np.dot(rel, e_vel) / (np.linalg.norm(rel) * np.linalg.norm(e_vel) + 1e-12)
            print(f'{"":>32}capture bearing from evader heading: '
                  f'{np.degrees(np.arccos(np.clip(cosb, -1, 1))):.0f} deg '
                  f'(>{CONE_HALF_ANGLE_DEG:.0f} = outside cone, 180 = directly behind)')
        results.append(res)

    # Check: the card's start position (-4, 2, 2) lies behind the evader
    print(f'Card start ({CARD_INIT_PURSUER[0]:g}, {CARD_INIT_PURSUER[1]:g}, {CARD_INIT_PURSUER[2]:g}) (bearing '
          f'{np.degrees(np.arccos(CARD_INIT_PURSUER[0] / np.linalg.norm(CARD_INIT_PURSUER[:2] - INIT_EVADER[:2]))):.1f} deg '
          f'from the evader heading):')
    for label, stealth in cases:
        r = run_simulation(stealth, init_pursuer=CARD_INIT_PURSUER)
        print(f'  [{label:<28}]  '
              + (f'captured {r[3]:.2f} s' if r[2] else 'timeout')
              + f'  |  alarm: {"t = %.2f s" % r[5] if r[5] is not None else "never"}')

    out_dir = os.path.normpath(OUTPUT_DIR)
    plot_trajectories_3d(results, out_dir)
    plot_detection_status(results, out_dir)
    plot_distance_time(results, out_dir)
    plot_comparison_bar(results, out_dir)
    save_animation(results, out_dir)
