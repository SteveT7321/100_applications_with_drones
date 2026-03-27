"""
S005 Stealth Approach
======================
Pursuer flanks laterally to stay outside the evader's forward detection cone,
then closes from the rear blind-spot.

Setup:
  - Evader at (0,0,2) heading +x.  Its detection cone (60°, 5m) points in the
    +x direction (forward).
  - Pursuer starts at (4, 2, 2) — inside the evader's forward cone from the start.
  - Detection alarm latches permanently once the pursuer is inside the cone for
    ≥ DETECTION_THRESHOLD consecutive steps (~0.2 s).  Once alerted the evader
    sprints away at EVADER_SPEED_FLEE (6 m/s) — faster than the detected pursuer
    (4 m/s) — so it can never be caught via direct chase.

Comparison:
  - Stealth flanking : when in cone → sidestep laterally;  pursuer exits detection
    range in ~3 steps (< threshold) → no alarm triggered → evader stays on patrol
    (2.5 m/s) → pursuer approaches from blind-spot rear at 7 m/s → captures easily.
  - Direct frontal   : pursuer charges straight at evader → stays inside the cone
    for ≥ threshold steps → permanent alarm → evader flees at 6 m/s forever →
    pursuer (4 m/s) can NEVER catch up → timeout.

Usage:
    conda activate drones
    python src/pursuit/s005_stealth_approach.py
"""

import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from src.base.drone_base import DroneBase

# ── Scenario parameters ─────────────────────────────────────
PURSUER_SPEED_CLEAR    = 7.0   # bonus speed while outside detection cone (unalerted)
PURSUER_SPEED_DETECTED = 4.0   # speed when inside cone (pre-alarm)
EVADER_SPEED_PATROL    = 2.5   # m/s constant heading when no alert
EVADER_SPEED_FLEE      = 6.0   # m/s flee speed when ALERTED — faster than detected pursuer!
CONE_HALF_ANGLE_DEG    = 60.0
DETECT_RANGE           = 5.0   # m
CAPTURE_R              = 0.15  # m
BEHIND_OFFSET          = 3.0   # m behind evader for stealth waypoint
DETECTION_THRESHOLD    = 30    # consecutive steps in cone before permanent alarm (~0.625s)
DT                     = 1 / 48
MAX_TIME               = 25.0

INIT_PURSUER   = np.array([4.0, 2.0, 2.0])   # inside cone from start
INIT_EVADER    = np.array([0.0, 0.0, 2.0])
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


def lateral_escape_dir(pos_p, pos_e, vel_e):
    """
    Unit vector perpendicular to evader heading that increases the pursuer's
    angular separation from the forward cone (same lateral side as current offset).
    """
    v_hat = vel_e / (np.linalg.norm(vel_e) + 1e-8)
    perp  = np.array([-v_hat[1], v_hat[0], 0.0])   # 2-D perpendicular in XY
    diff  = pos_p - pos_e
    lat_sign = np.sign(np.dot(diff, perp))
    if lat_sign == 0:
        lat_sign = 1.0
    return lat_sign * perp


# ── Simulation ──────────────────────────────────────────────

def run_simulation(stealth_mode):
    """
    stealth_mode=True  → reactive flanking: sidestep when in cone,
                         aim for behind-waypoint when clear.
    stealth_mode=False → direct: always charge straight at evader.

    Alarm latches permanently once the pursuer is detected for
    DETECTION_THRESHOLD consecutive steps.  Alerted evader flees at
    EVADER_SPEED_FLEE (6 m/s) — faster than the detected pursuer (4 m/s).
    """
    pursuer = DroneBase(INIT_PURSUER.copy(), max_speed=PURSUER_SPEED_CLEAR, dt=DT)
    evader  = DroneBase(INIT_EVADER.copy(),  max_speed=EVADER_SPEED_FLEE,   dt=DT)

    vel_e       = EVADER_HEADING.copy().astype(float)
    alerted     = False
    consec_det  = 0           # consecutive steps inside cone
    alarm_step  = None        # timestep when alarm latched
    max_steps   = int(MAX_TIME / DT)

    captured     = False
    capture_time = None
    detected_log = []         # per-step 0/1
    alerted_log  = []         # per-step 0/1 (permanent after alarm)

    for step in range(max_steps):
        t = step * DT

        # ── Momentary detection ──
        detected_now = is_detected(pursuer.pos, evader.pos, vel_e,
                                   CONE_HALF_ANGLE_DEG, DETECT_RANGE)
        detected_log.append(1 if detected_now else 0)

        # ── Alarm debounce: latch permanently after threshold ──
        if detected_now:
            consec_det += 1
            if consec_det >= DETECTION_THRESHOLD and not alerted:
                alerted    = True
                alarm_step = step
        else:
            consec_det = 0
        alerted_log.append(1 if alerted else 0)

        # ── Capture check ──
        if np.linalg.norm(pursuer.pos - evader.pos) < CAPTURE_R:
            captured     = True
            capture_time = t
            break

        # ── Pursuer velocity ──
        if stealth_mode:
            if detected_now:
                # sidestep laterally to exit the cone — use full clear speed to escape fast
                speed   = PURSUER_SPEED_CLEAR
                cmd_dir = lateral_escape_dir(pursuer.pos, evader.pos, vel_e)
            else:
                speed = PURSUER_SPEED_CLEAR
                dist  = np.linalg.norm(pursuer.pos - evader.pos)
                if dist > BEHIND_OFFSET + 0.5:
                    target = behind_waypoint(evader.pos, vel_e)
                else:
                    target = evader.pos            # close enough — finish directly
                cmd_dir = target - pursuer.pos
                n = np.linalg.norm(cmd_dir)
                cmd_dir = cmd_dir / n if n > 1e-6 else cmd_dir
        else:
            # direct: always charge at evader
            speed   = PURSUER_SPEED_DETECTED
            cmd_dir = evader.pos - pursuer.pos
            n = np.linalg.norm(cmd_dir)
            cmd_dir = cmd_dir / n if n > 1e-6 else cmd_dir

        pursuer.step(cmd_dir * speed)

        # ── Evader velocity ──
        if alerted:
            # PERMANENT: sprint away from pursuer forever
            flee = evader.pos - pursuer.pos
            n    = np.linalg.norm(flee)
            if n > 1e-6:
                vel_e = flee / n
            evader.step(vel_e * EVADER_SPEED_FLEE)
        else:
            # patrol: resume constant heading
            vel_e = EVADER_HEADING.copy().astype(float)
            if detected_now:
                # brief pre-alarm reaction: sidestep slightly but don't sprint
                evader.step(vel_e * EVADER_SPEED_PATROL)
            else:
                evader.step(vel_e * EVADER_SPEED_PATROL)

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
    )


# ── Detection cone mesh ─────────────────────────────────────

def _cone_mesh(apex, axis, half_angle_deg, length, n=30):
    theta = np.linspace(0, 2 * np.pi, n)
    r     = length * np.tan(np.radians(half_angle_deg))
    cx    = r * np.cos(theta)
    cy    = r * np.sin(theta)
    cz    = np.full_like(theta, length)

    axis = axis / (np.linalg.norm(axis) + 1e-8)
    z    = np.array([0.0, 0.0, 1.0])
    cross = np.cross(z, axis)
    cross_norm = np.linalg.norm(cross)
    if cross_norm < 1e-6:
        R = np.eye(3) if np.dot(z, axis) > 0 else np.diag([1, 1, -1])
    else:
        k = cross / cross_norm
        c = np.dot(z, axis); s = cross_norm
        K = np.array([[0,-k[2],k[1]],[k[2],0,-k[0]],[-k[1],k[0],0]])
        R = np.eye(3) + s * K + (1 - c) * K @ K

    pts = R @ np.vstack([cx, cy, cz])
    X = np.column_stack([np.full(n, apex[0]), apex[0] + pts[0]])
    Y = np.column_stack([np.full(n, apex[1]), apex[1] + pts[1]])
    Z = np.column_stack([np.full(n, apex[2]), apex[2] + pts[2]])
    return X, Y, Z


# ── Plots ───────────────────────────────────────────────────

# 每個子圖各自算軸範圍，避免 timeout 那條軌跡把軸撐爆
def _traj_limits(p_traj, e_traj, margin=2.0):
    pts = np.vstack([p_traj, e_traj])
    lo  = pts.min(axis=0) - margin
    hi  = pts.max(axis=0) + margin
    return lo, hi


def _clip_traj(traj, max_steps):
    """只保留前 max_steps 步，讓 timeout 那張不要畫到太遠。"""
    return traj[:max_steps]


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
    clip_steps = [None, int(5.0 / DT)]   # direct 只顯示前 5 秒

    fig = plt.figure(figsize=(16, 7))

    for i, (label, res, clip) in enumerate(zip(labels, results, clip_steps)):
        p_traj, e_traj, captured, cap_t, alerted, alarm_t, det_log, _ = res

        pt = p_traj if clip is None else _clip_traj(p_traj, clip)
        et = e_traj if clip is None else _clip_traj(e_traj, clip)

        # ── 俯瞰 XY ──────────────────────────────────────
        ax = fig.add_subplot(2, 2, i * 2 + 1)

        # 偵測錐投影（從 evader 起點畫）
        _draw_cone_2d(ax, INIT_EVADER, EVADER_HEADING,
                      CONE_HALF_ANGLE_DEG, DETECT_RANGE)

        # 偵測圈
        circle = plt.Circle(INIT_EVADER[:2], DETECT_RANGE,
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

        if captured:
            ax.scatter(*pt[-1, :2], color='black', s=150, marker='X', zorder=8,
                       label=f'Captured {cap_t:.2f}s')
            status = f'Captured {cap_t:.2f} s'
        else:
            status = f'ALARM @ {alarm_t:.2f}s → Timeout'
            ax.scatter(*pt[-1, :2], color='grey', s=100, marker='X', zorder=8)
            if clip:
                ax.text(0.02, 0.02, f'(showing first {clip*DT:.0f}s)',
                        transform=ax.transAxes, fontsize=7, color='grey')

        lo, hi = _traj_limits(pt, et, margin=1.5)
        ax.set_xlim(lo[0], hi[0]); ax.set_ylim(lo[1], hi[1])
        ax.set_aspect('equal'); ax.grid(True, alpha=0.3)
        ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
        ax.set_title(f'{label}\n{status}', fontsize=10)
        ax.legend(fontsize=7, loc='upper left')

        # ── 小 3D 圖 ─────────────────────────────────────
        ax3 = fig.add_subplot(2, 2, i * 2 + 2, projection='3d')
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
                 '(gold = detection cone  |  bright red = pursuer detected)',
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
        p_traj, e_traj, captured, cap_t, alerted, alarm_t, det_log, alert_log = res
        t_ax = np.arange(len(det_log)) * DT

        ax.fill_between(t_ax, det_log, step='mid', color=color, alpha=0.4,
                        label='Momentary detection')
        ax.fill_between(t_ax, alert_log, step='mid', color='red', alpha=0.2,
                        label='Permanent alarm active')
        if alarm_t:
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
        p_traj, e_traj, captured, cap_t, _, alarm_t, _, _ = res
        n     = min(len(p_traj), len(e_traj))
        dists = np.linalg.norm(p_traj[:n] - e_traj[:n], axis=1)
        times = np.arange(n) * DT
        ax.plot(times, dists, color=color, linewidth=1.8, label=label)
        if captured:
            ax.axvline(cap_t, color=color, linestyle='--', linewidth=1, alpha=0.8)
        if alarm_t:
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
        _, _, captured, cap_t, alerted_flag, _, det_log, _ = res
        cap_times.append(cap_t if captured else MAX_TIME)
        det_pcts.append(100 * det_log.mean())
        alerted_flags.append(alerted_flag)

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 5))

    bars = ax1.bar(labels, cap_times, color=colors, width=0.4, edgecolor='black')
    for bar, t, res, af in zip(bars, cap_times, results, alerted_flags):
        txt = f'{t:.2f} s' if res[2] else f'>{t:.0f} s\n(ALARM→Timeout)'
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
    """俯瞰 XY 動畫：stealth pursuer（紅）繞過偵測錐（金色）從後方捕獲 evader（藍）。"""
    import matplotlib.animation as animation

    p_traj, e_traj, captured, cap_t, alerted, alarm_t, det_log, alert_log = results[0]
    lo, hi = _traj_limits(p_traj, e_traj, margin=2.0)

    step     = 3
    p_frames = p_traj[::step]
    e_frames = e_traj[::step]
    d_frames = det_log[::step]
    n_frames = min(len(p_frames), len(e_frames))

    fig, ax = plt.subplots(figsize=(9, 7))
    ax.set_xlim(lo[0], hi[0]); ax.set_ylim(lo[1], hi[1])
    ax.set_aspect('equal'); ax.grid(True, alpha=0.3)
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')

    # 靜態：偵測錐 + 偵測圈
    _draw_cone_2d(ax, INIT_EVADER, EVADER_HEADING,
                  CONE_HALF_ANGLE_DEG, DETECT_RANGE, alpha=0.20)
    circle = plt.Circle(INIT_EVADER[:2], DETECT_RANGE,
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
        title_txt.set_text(f'S005 Stealth Approach  t={t:.2f}s  {status}')
        return p_trail, e_trail, p_dot, e_dot, title_txt

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
            status = f'TIMEOUT (alarm @ {alarm_t:.2f}s)' if alarm_t else 'timeout (no alarm)'
        print(f'[{label:<28}]  {status}  |  in-cone {det_pct:.1f}% of sim')
        results.append(res)

    out_dir = os.path.normpath(OUTPUT_DIR)
    plot_trajectories_3d(results, out_dir)
    plot_detection_status(results, out_dir)
    plot_distance_time(results, out_dir)
    plot_comparison_bar(results, out_dir)
    save_animation(results, out_dir)
