"""
S007 3D Upgrade — Jamming & Blind Pursuit
Usage:
    conda activate drones
    python src/01_pursuit_evasion/3d/s007_3d_jamming_blind_pursuit.py
"""
import sys, os, numpy as np, matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from matplotlib.animation import FuncAnimation, PillowWriter
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..'))
from src.base.drone_base import DroneBase

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), '..', '..', '..',
                          'outputs', '01_pursuit_evasion', '3d', 's007_3d_jamming_blind_pursuit')
os.makedirs(OUTPUT_DIR, exist_ok=True)

# ── Parameters ───────────────────────────────────────────────
JAM_PERIOD   = 3.0    # s
JAM_DURATION = 1.0    # s  (33% duty cycle)
SIGMA_XY     = 0.05   # m/s  horizontal drift
SIGMA_Z_IMU  = 0.02   # m/s  vertical drift (IMU only)
SIGMA_Z_BARO = 0.005  # m/s  vertical drift (baro-aided)
V_PURSUER    = 5.0    # m/s
V_EVADER     = 3.0    # m/s
CAPTURE_R    = 0.15   # m
DT           = 1 / 48
MAX_TIME     = 30.0
Z_MIN        = 0.3
Z_MAX        = 8.0

INIT_PURSUER = np.array([-3.0, 0.0, 2.0])
INIT_EVADER  = np.array([ 3.0, 0.0, 2.0])

PURSUIT_MODES = ['perfect_gps', 'dead_reckoning_imu', 'baro_aided', 'freeze']
MODE_LABELS   = ['Perfect GPS', 'Dead Reckoning IMU', 'Baro-Aided', 'Freeze']
EVADER_TACTICS = ['straight', 'altitude_jump']

RNG = np.random.default_rng(42)


# ── Helpers ──────────────────────────────────────────────────

def is_jammed(t):
    return (t % JAM_PERIOD) < JAM_DURATION


# ── Simulation ───────────────────────────────────────────────

def run_simulation(mode, evader_tactic):
    pursuer = DroneBase(INIT_PURSUER.copy(), max_speed=V_PURSUER, dt=DT)
    evader  = DroneBase(INIT_EVADER.copy(),  max_speed=V_EVADER,  dt=DT)

    evader_base_vel = np.array([V_EVADER, 0.0, 0.0])
    evader_vel      = evader_base_vel.copy()

    p_est     = pursuer.pos.copy()   # pursuer's self-estimate of evader position
    v_last    = np.zeros(3)          # last known evader velocity
    t_jam_start  = None
    p_est_at_jam = None
    v_last_at_jam = None
    prev_jammed  = False

    # altitude_jump: toggle per jam window
    jump_sign    = 1
    jump_targets = {}   # t_jam_start -> target z

    max_steps = int(MAX_TIME / DT)
    captured  = False
    cap_time  = None

    p_traj_list  = [pursuer.pos.copy()]
    e_traj_list  = [evader.pos.copy()]
    est_list     = [p_est.copy()]
    err_list     = [0.0]
    jam_list     = [0]
    time_list    = [0.0]

    for step in range(1, max_steps + 1):
        t = step * DT
        jammed = is_jammed(t)

        # Track jam window start
        if jammed and not prev_jammed:
            t_jam_start   = t
            p_est_at_jam  = pursuer.pos.copy()
            v_last_at_jam = v_last.copy()
            # altitude_jump evader: record target
            if evader_tactic == 'altitude_jump':
                new_z = np.clip(evader.pos[2] + jump_sign * 2.0, Z_MIN, Z_MAX)
                jump_targets[t_jam_start] = new_z
                jump_sign = -jump_sign

        prev_jammed = jammed

        # ── Update position estimate ──
        if mode == 'perfect_gps':
            p_est = evader.pos.copy()
            v_last = evader.vel.copy()
        elif not jammed:
            p_est  = evader.pos.copy()
            v_last = evader.vel.copy()
        else:
            elapsed = t - t_jam_start
            if mode == 'dead_reckoning_imu':
                noise_xy = RNG.normal(0, SIGMA_XY  * np.sqrt(elapsed), 2)
                noise_z  = RNG.normal(0, SIGMA_Z_IMU * np.sqrt(elapsed))
                noise    = np.array([noise_xy[0], noise_xy[1], noise_z])
                p_est    = p_est_at_jam + v_last_at_jam * elapsed + noise
            elif mode == 'baro_aided':
                noise_xy = RNG.normal(0, SIGMA_XY   * np.sqrt(elapsed), 2)
                noise_z  = RNG.normal(0, SIGMA_Z_BARO * np.sqrt(elapsed))
                noise    = np.array([noise_xy[0], noise_xy[1], noise_z])
                p_est    = p_est_at_jam + v_last_at_jam * elapsed + noise
            # freeze: p_est unchanged (pursuer knows its own position from last GPS)

        # Altitude safety check on estimate
        if p_est[2] < Z_MIN:
            p_est[2] = 0.5  # hover up

        # ── Pursuer command ──
        if jammed and mode == 'freeze':
            cmd = np.zeros(3)
        else:
            diff = p_est - pursuer.pos
            n    = np.linalg.norm(diff)
            cmd  = V_PURSUER * diff / n if n > 1e-8 else np.zeros(3)

        pursuer.step(cmd)
        # Clip pursuer z
        pursuer.pos[2] = np.clip(pursuer.pos[2], Z_MIN, Z_MAX)

        # ── Evader movement ──
        if evader_tactic == 'altitude_jump' and t_jam_start is not None and jammed:
            # During jam, climb to target z
            target_z = jump_targets.get(t_jam_start, evader.pos[2])
            dz = target_z - evader.pos[2]
            # Blend horizontal + vertical
            vx = V_EVADER * 0.8
            vz = np.sign(dz) * V_EVADER * 0.6 if abs(dz) > 0.05 else 0.0
            evader_vel = np.array([vx, 0.0, vz])
            spd = np.linalg.norm(evader_vel)
            if spd > V_EVADER:
                evader_vel = evader_vel / spd * V_EVADER
        else:
            evader_vel = evader_base_vel.copy()

        evader.step(evader_vel)
        evader.pos[2] = np.clip(evader.pos[2], Z_MIN, Z_MAX)

        # Capture check
        if np.linalg.norm(pursuer.pos - evader.pos) < CAPTURE_R:
            captured = True
            cap_time = t
            p_traj_list.append(pursuer.pos.copy())
            e_traj_list.append(evader.pos.copy())
            est_list.append(p_est.copy())
            err_list.append(np.linalg.norm(p_est - evader.pos))
            jam_list.append(1 if jammed else 0)
            time_list.append(t)
            break

        p_traj_list.append(pursuer.pos.copy())
        e_traj_list.append(evader.pos.copy())
        est_list.append(p_est.copy())
        err_list.append(np.linalg.norm(p_est - evader.pos))
        jam_list.append(1 if jammed else 0)
        time_list.append(t)

    return {
        'p_traj':   np.array(p_traj_list),
        'e_traj':   np.array(e_traj_list),
        'est_traj': np.array(est_list),
        'est_err':  np.array(err_list),
        'jam_log':  np.array(jam_list),
        'times':    np.array(time_list),
        'captured': captured,
        'cap_time': cap_time,
        'mode':     mode,
        'tactic':   evader_tactic,
    }


# ── Jam window shading helper ────────────────────────────────

def shade_jam_windows(ax, times, jam_log, color='salmon', alpha=0.15, label=True):
    in_jam = False
    j_start = 0.0
    for k in range(len(jam_log)):
        if jam_log[k] and not in_jam:
            j_start = times[k]
            in_jam = True
        elif not jam_log[k] and in_jam:
            kw = {'color': color, 'alpha': alpha}
            if label:
                kw['label'] = 'Jam window'
                label = False
            ax.axvspan(j_start, times[k], **kw)
            in_jam = False
    if in_jam:
        ax.axvspan(j_start, times[-1], color=color, alpha=alpha)


# ── Plots ─────────────────────────────────────────────────────

def plot_trajectories_3d(results_straight, out_dir):
    """4 subplots — 4 pursuit modes, straight evader."""
    colors = ['mediumseagreen', 'steelblue', 'darkorange', 'mediumpurple']
    fig = plt.figure(figsize=(18, 12))
    for idx, (res, label, color) in enumerate(zip(results_straight, MODE_LABELS, colors)):
        ax = fig.add_subplot(2, 2, idx + 1, projection='3d')
        ax.view_init(elev=25, azim=-55)
        ax.plot(res['p_traj'][:, 0], res['p_traj'][:, 1], res['p_traj'][:, 2],
                color=color, linewidth=1.8, label='Pursuer (true)')
        ax.plot(res['est_traj'][:, 0], res['est_traj'][:, 1], res['est_traj'][:, 2],
                color=color, linewidth=0.8, linestyle=':', alpha=0.5, label='Estimate')
        ax.plot(res['e_traj'][:, 0], res['e_traj'][:, 1], res['e_traj'][:, 2],
                color='royalblue', linewidth=1.8, label='Evader')
        ax.scatter(*res['p_traj'][0], color=color, s=60, marker='o')
        ax.scatter(*res['e_traj'][0], color='blue', s=60, marker='o')
        if res['captured']:
            ax.scatter(*res['p_traj'][-1], color='black', s=100, marker='X',
                       label=f'Captured {res["cap_time"]:.2f}s')
            status = f'Captured {res["cap_time"]:.2f} s'
        else:
            status = 'Timeout'
        ax.set_title(f'{label}\n{status}', fontsize=10)
        ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
        ax.legend(fontsize=7, loc='upper left')

    fig.suptitle('S007 3D Jamming — 4 Pursuit Modes (straight evader)', fontsize=12)
    plt.tight_layout()
    path = os.path.join(out_dir, 'trajectories_3d.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_position_error_3d(results_straight, out_dir):
    """3D estimation error vs time for IMU, baro, freeze."""
    show_modes = ['dead_reckoning_imu', 'baro_aided', 'freeze']
    show_labels = ['Dead Reckoning IMU', 'Baro-Aided', 'Freeze']
    colors = ['steelblue', 'darkorange', 'mediumpurple']

    fig, ax = plt.subplots(figsize=(12, 5))
    res_imu = None
    for res in results_straight:
        if res['mode'] in show_modes:
            i = show_modes.index(res['mode'])
            ax.plot(res['times'], res['est_err'], color=colors[i],
                    linewidth=1.8, label=show_labels[i])
            if res['captured']:
                ax.axvline(res['cap_time'], color=colors[i], linestyle='--',
                           linewidth=1.0, alpha=0.7)
            if res['mode'] == 'dead_reckoning_imu':
                res_imu = res

    if res_imu is not None:
        shade_jam_windows(ax, res_imu['times'], res_imu['jam_log'])

    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Position Estimation Error (m)')
    ax.set_title('S007 3D Jamming — Estimation Error vs Time\n'
                 '(salmon bands = jam windows)', fontsize=10)
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'position_error_3d.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def plot_altitude_subplot(results_altjump, out_dir):
    """Pursuer z estimate vs true evader z, dead_reckoning_imu + altitude_jump."""
    res = next((r for r in results_altjump if r['mode'] == 'dead_reckoning_imu'), None)
    if res is None:
        return
    fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True)

    ax0 = axes[0]
    ax0.plot(res['times'], res['est_traj'][:, 2], color='red', linewidth=1.5,
             label='Pursuer z-estimate (dead reckoning)')
    ax0.plot(res['times'], res['e_traj'][:, 2], color='royalblue', linewidth=1.5,
             linestyle='--', label='True evader z')
    shade_jam_windows(ax0, res['times'], res['jam_log'])
    ax0.set_ylabel('Altitude (m)')
    ax0.set_title('Pursuer Z-estimate vs Evader Z (altitude-jump tactic)', fontsize=10)
    ax0.legend(fontsize=8); ax0.grid(True, alpha=0.3)

    # z error
    ax1 = axes[1]
    z_err = np.abs(res['est_traj'][:, 2] - res['e_traj'][:, 2])
    ax1.plot(res['times'], z_err, color='tomato', linewidth=1.5, label='Z error |est - true|')
    shade_jam_windows(ax1, res['times'], res['jam_log'])
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Z Error (m)')
    ax1.set_title('Vertical estimation error', fontsize=10)
    ax1.legend(fontsize=8); ax1.grid(True, alpha=0.3)

    fig.suptitle('S007 3D Jamming — Altitude Subplot (Dead Reckoning IMU + Altitude Jump)',
                 fontsize=11)
    plt.tight_layout()
    path = os.path.join(out_dir, 'altitude_subplot.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def plot_capture_table(all_results, out_dir):
    """4x2 table figure: modes x evader tactics."""
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.axis('off')

    col_labels = ['Evader: Straight', 'Evader: Alt Jump']
    row_labels  = MODE_LABELS
    table_data  = []
    cell_colors = []

    for mode in PURSUIT_MODES:
        row = []
        row_c = []
        for tactic in EVADER_TACTICS:
            r = all_results[mode][tactic]
            if r['captured']:
                txt = f'{r["cap_time"]:.2f} s'
                col = '#b7e4b7'
            else:
                txt = 'Missed'
                col = '#f5b7b1'
            row.append(txt)
            row_c.append(col)
        table_data.append(row)
        cell_colors.append(row_c)

    table = ax.table(
        cellText=table_data,
        rowLabels=row_labels,
        colLabels=col_labels,
        cellColours=cell_colors,
        loc='center',
        cellLoc='center',
    )
    table.auto_set_font_size(False)
    table.set_fontsize(12)
    table.scale(1.4, 2.0)
    ax.set_title('S007 3D Jamming — Capture Time Table\n(green=captured, red=missed)',
                 fontsize=12, pad=20)

    plt.tight_layout()
    path = os.path.join(out_dir, 'capture_table.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def save_animation(all_results, out_dir):
    """Animate dead_reckoning_imu vs altitude_jump evader."""
    res = all_results['dead_reckoning_imu']['altitude_jump']
    p_traj   = res['p_traj']
    e_traj   = res['e_traj']
    est_traj = res['est_traj']
    jam_log  = res['jam_log']

    step_skip = 3
    n_frames  = len(p_traj) // step_skip

    fig = plt.figure(figsize=(10, 8))
    ax  = fig.add_subplot(111, projection='3d')
    all_pts = np.vstack([p_traj, e_traj, est_traj])
    lo = all_pts.min(axis=0) - 1.5
    hi = all_pts.max(axis=0) + 1.5
    ax.set_xlim(lo[0], hi[0])
    ax.set_ylim(lo[1], hi[1])
    ax.set_zlim(max(0, lo[2]), hi[2])
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
    ax.view_init(elev=25, azim=-50)

    p_line,   = ax.plot([], [], [], color='red',       linewidth=1.8, label='Pursuer (true)')
    e_line,   = ax.plot([], [], [], color='royalblue',  linewidth=1.8, label='Evader (alt jump)')
    est_line, = ax.plot([], [], [], color='tomato',     linewidth=0.8, linestyle=':', label='Estimate')
    p_dot,    = ax.plot([], [], [], 'r^', markersize=10, zorder=8)
    e_dot,    = ax.plot([], [], [], 'bs', markersize=10, zorder=8)
    ax.legend(fontsize=8, loc='upper left')
    title = ax.set_title('')

    def update(i):
        si = min(i * step_skip, len(p_traj) - 1)
        p_line.set_data(p_traj[:si+1, 0], p_traj[:si+1, 1])
        p_line.set_3d_properties(p_traj[:si+1, 2])
        e_line.set_data(e_traj[:si+1, 0], e_traj[:si+1, 1])
        e_line.set_3d_properties(e_traj[:si+1, 2])
        est_line.set_data(est_traj[:si+1, 0], est_traj[:si+1, 1])
        est_line.set_3d_properties(est_traj[:si+1, 2])
        p_dot.set_data([p_traj[si, 0]], [p_traj[si, 1]])
        p_dot.set_3d_properties([p_traj[si, 2]])
        e_dot.set_data([e_traj[si, 0]], [e_traj[si, 1]])
        e_dot.set_3d_properties([e_traj[si, 2]])
        t = si * DT
        jammed = bool(jam_log[si]) if si < len(jam_log) else False
        jam_str = ' [JAM]' if jammed else ''
        title.set_text(f'S007 3D Dead Reckoning IMU + Alt Jump  t={t:.2f}s{jam_str}')
        return p_line, e_line, est_line, p_dot, e_dot, title

    ani = FuncAnimation(fig, update, frames=n_frames, interval=70, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer=PillowWriter(fps=14), dpi=90)
    plt.close()
    print(f'Saved: {path}')


# ── Main ─────────────────────────────────────────────────────

if __name__ == '__main__':
    # Run all 4 modes × 2 tactics = 8 simulations
    all_results = {}
    for mode in PURSUIT_MODES:
        all_results[mode] = {}
        for tactic in EVADER_TACTICS:
            r = run_simulation(mode, tactic)
            status = f'captured {r["cap_time"]:.2f}s' if r['captured'] else 'timeout'
            print(f'[{mode:<20} | {tactic:<14}]  {status}')
            all_results[mode][tactic] = r

    # Extract lists for plotting
    results_straight  = [all_results[m]['straight']     for m in PURSUIT_MODES]
    results_altjump   = [all_results[m]['altitude_jump'] for m in PURSUIT_MODES]

    out_dir = os.path.normpath(OUTPUT_DIR)
    plot_trajectories_3d(results_straight, out_dir)
    plot_position_error_3d(results_straight, out_dir)
    plot_altitude_subplot(results_altjump, out_dir)
    plot_capture_table(all_results, out_dir)
    save_animation(all_results, out_dir)
    print('S007 3D done.')
