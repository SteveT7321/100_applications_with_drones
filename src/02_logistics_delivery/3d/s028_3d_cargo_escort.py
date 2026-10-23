"""
S028 3D Cargo Escort Formation
================================
A cargo drone (Carrier) flies a 5-waypoint route that climbs and descends
between 5 m and 20 m.  Four escorts hold the card's 3-D diamond around it:
Left / Right at +/-d in the carrier's lateral axis and Top / Bottom at +/-d in
z (d = 3 m).  Slots follow p_slot = p_carrier + R(yaw) f_i, where yaw is the
carrier's horizontal velocity heading (held when the carrier is at rest).

Escorts are double-integrator point masses under PD position control
(card: "PD position control") with slot-velocity feed-forward but no
acceleration feed-forward, an acceleration limit and repulsion from every
other drone, the Carrier included.  The Carrier is an acceleration-limited
point mass that tracks a speed command toward the active waypoint.

The run ends HOLD_TIME seconds after the carrier arrives; formation error is
reported for the mission (take-off to arrival) and for the hold separately.
Separation is checked over all pairs, Carrier included.

Outputs:
    trajectory_3d.png        -- 3-D trajectory of all drones
    topdown_xy.png           -- XY top-down view with formation snapshots
    altitude_profile.png     -- Z(t) for all drones and vertical slot error
    formation_metrics.png    -- slot error, separation, speed / acceleration
    animation.gif            -- animated 3-D view of the escort mission

Usage:
    conda activate drones
    python src/02_logistics_delivery/3d/s028_3d_cargo_escort.py
"""

import matplotlib
matplotlib.use('Agg')

import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
import matplotlib.animation as animation
from matplotlib.lines import Line2D

# ── Output directory ─────────────────────────────────────────────────────────
OUTPUT_DIR = os.path.normpath(os.path.join(
    os.path.dirname(__file__), '..', '..', '..',
    'outputs', '02_logistics_delivery', '3d', 's028_3d_cargo_escort',
))

# ── Parameters ────────────────────────────────────────────────────────────────
D_FORM      = 3.0       # m   formation offset d (card)
V_CARRIER   = 3.0       # m/s carrier cruise speed
A_CARRIER   = 1.5       # m/s^2 carrier acceleration limit
K_VC        = 3.0       # 1/s carrier speed-tracking gain
WP_SWITCH   = 1.0       # m   intermediate waypoint switch radius
ARRIVAL_TOL = 0.3       # m   final arrival radius (used for both stop and check)
V_ESCORT    = 6.0       # m/s escort speed limit
A_ESCORT    = 4.0       # m/s^2 escort acceleration limit
KP_E        = 4.0       # 1/s^2 escort PD position gain
KD_E        = 4.0       # 1/s   escort PD velocity gain (critically damped)
K_REP       = 3.0       # repulsion gain
D_SAFE      = 1.0       # m   minimum safe separation
HEADING_HOLD_V = 0.2    # m/s below this horizontal speed the yaw is held
DT          = 1 / 48    # s
T_MAX       = 80.0      # s   safety horizon
HOLD_TIME   = 5.0       # s   simulated after arrival

# Carrier waypoints: 5 waypoints, altitude 5-20 m (card)
WAYPOINTS = np.array([
    [  0.0,  0.0,  5.0],   # WP0 start
    [ 25.0, 10.0, 12.0],   # WP1 climb + left turn
    [ 50.0, -5.0, 20.0],   # WP2 top of route
    [ 75.0, 10.0, 12.0],   # WP3 descend
    [100.0,  0.0,  5.0],   # WP4 goal
])

# Diamond slots in the carrier body frame [long, lat, up] (card: lateral and vertical pairs)
SLOT_OFFSETS = np.array([
    [0.0,  D_FORM, 0.0],    # Left
    [0.0, -D_FORM, 0.0],    # Right
    [0.0,  0.0,  D_FORM],   # Top
    [0.0,  0.0, -D_FORM],   # Bottom
])
N_ESCORTS = len(SLOT_OFFSETS)
ESCORT_COLORS = ['blue', 'orange', 'purple', 'saddlebrown']
ESCORT_LABELS = ['Left (+d lat)', 'Right (-d lat)', 'Top (+d z)', 'Bottom (-d z)']
NAMES = ['Carrier'] + ESCORT_LABELS


# ── Helpers ───────────────────────────────────────────────────────────────────

def rotation_z(psi):
    c, s = np.cos(psi), np.sin(psi)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def slot_positions(pos_c, psi):
    """Desired slots: p_c + R(yaw) f_i (yaw-only rotation, as on the card)."""
    return pos_c + SLOT_OFFSETS @ rotation_z(psi).T


def clamp(v, vmax):
    n = np.linalg.norm(v)
    return v * (vmax / n) if n > vmax else v


def carrier_accel(pos_c, vel_c, wp_idx):
    goal = WAYPOINTS[wp_idx]
    d = goal - pos_c
    dist = np.linalg.norm(d)
    if wp_idx == len(WAYPOINTS) - 1:
        speed = min(V_CARRIER, np.sqrt(2.0 * 0.5 * A_CARRIER * dist))  # brake to stop
    else:
        speed = V_CARRIER
    v_des = d / dist * speed if dist > 1e-9 else np.zeros(3)
    return clamp(K_VC * (v_des - vel_c), A_CARRIER)


# ── Simulation ────────────────────────────────────────────────────────────────

def run_simulation():
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    pos_c = WAYPOINTS[0].astype(float).copy()
    vel_c = np.zeros(3)
    wp_idx = 1
    d0 = WAYPOINTS[1] - WAYPOINTS[0]
    psi = float(np.arctan2(d0[1], d0[0]))       # initial heading: toward WP1
    pos_e = slot_positions(pos_c, psi)
    vel_e = np.zeros((N_ESCORTS, 3))
    slots_prev = pos_e.copy()

    t_log, pc_log, vc_log, pe_log, slot_log, psi_log, rep_log, ae_log = ([] for _ in range(8))
    wp_switch_t = []
    arrive_step = None
    n_steps = int(round(T_MAX / DT))
    for k in range(n_steps + 1):
        t = k * DT
        # heading: horizontal velocity heading; held at low speed and after arrival
        if arrive_step is None and np.linalg.norm(vel_c[:2]) > HEADING_HOLD_V:
            psi = float(np.arctan2(vel_c[1], vel_c[0]))
        slots = slot_positions(pos_c, psi)
        v_slot = (slots - slots_prev) / DT if k > 0 else np.zeros((N_ESCORTS, 3))
        slots_prev = slots

        t_log.append(t); pc_log.append(pos_c.copy()); vc_log.append(vel_c.copy())
        pe_log.append(pos_e.copy()); slot_log.append(slots.copy()); psi_log.append(psi)

        if arrive_step is not None and k >= arrive_step + int(round(HOLD_TIME / DT)):
            ae_log.append(np.zeros(N_ESCORTS)); rep_log.append(False)
            break

        # carrier waypoint logic
        dist_wp = np.linalg.norm(WAYPOINTS[wp_idx] - pos_c)
        if wp_idx < len(WAYPOINTS) - 1 and dist_wp < WP_SWITCH:
            wp_idx += 1
            wp_switch_t.append(t)
        elif wp_idx == len(WAYPOINTS) - 1 and arrive_step is None and dist_wp < ARRIVAL_TOL:
            arrive_step = k
        a_c = carrier_accel(pos_c, vel_c, wp_idx)

        # escorts: PD on slot error + slot-velocity feed-forward + repulsion
        others = np.vstack([pos_c[None, :], pos_e])
        a_e = np.zeros((N_ESCORTS, 3))
        rep_any = False
        for i in range(N_ESCORTS):
            a = KP_E * (slots[i] - pos_e[i]) + KD_E * (v_slot[i] - vel_e[i])
            for j in range(N_ESCORTS + 1):
                if j == i + 1:
                    continue
                r = pos_e[i] - others[j]
                d = np.linalg.norm(r)
                if d < D_SAFE:
                    a += K_REP * (D_SAFE - d) / d * (r / d)
                    rep_any = True
            a_e[i] = clamp(a, A_ESCORT)
        ae_log.append(np.linalg.norm(a_e, axis=1)); rep_log.append(rep_any)

        vel_c = clamp(vel_c + a_c * DT, V_CARRIER)
        pos_c = pos_c + vel_c * DT
        for i in range(N_ESCORTS):
            vel_e[i] = clamp(vel_e[i] + a_e[i] * DT, V_ESCORT)
        pos_e = pos_e + vel_e * DT

    t = np.array(t_log)
    pc = np.array(pc_log); pe = np.array(pe_log); sl = np.array(slot_log)
    form_err = np.linalg.norm(pe - sl, axis=2)
    z_err = pe[:, :, 2] - sl[:, :, 2]
    allp = np.concatenate([pc[:, None, :], pe], axis=1)        # (T, 5, 3)
    n = allp.shape[1]
    pair_d = {}
    for i in range(n):
        for j in range(i + 1, n):
            pair_d[(i, j)] = np.linalg.norm(allp[:, i] - allp[:, j], axis=1)
    min_sep_ts = np.min(np.array(list(pair_d.values())), axis=0)
    pair_min = {p: float(v.min()) for p, v in pair_d.items()}
    worst_pair = min(pair_min, key=pair_min.get)
    esc_pairs = {p: v for p, v in pair_min.items() if p[0] > 0}
    worst_esc = min(esc_pairs, key=esc_pairs.get)

    arrived = arrive_step is not None
    a = arrive_step if arrived else len(t) - 1
    mis = slice(0, a + 1)
    hold = slice(a + 1, len(t))
    res = dict(
        t=t, traj_c=pc, vel_c=np.array(vc_log), traj_e=pe, slot_traj=sl, psi=np.array(psi_log),
        form_err=form_err, z_err=z_err, min_sep_ts=min_sep_ts, pair_min=pair_min,
        worst_pair=worst_pair, worst_esc=worst_esc, rep=np.array(rep_log),
        acc_e=np.array(ae_log), arrive_step=a, arrived=arrived, wp_switch_t=wp_switch_t,
        rms_mission=float(np.sqrt(np.mean(form_err[mis] ** 2))),
        rms_mission_esc=np.sqrt(np.mean(form_err[mis] ** 2, axis=0)),
        max_mission_esc=form_err[mis].max(axis=0),
        rms_hold=float(np.sqrt(np.mean(form_err[hold] ** 2))) if a + 1 < len(t) else float('nan'),
        rms_all=float(np.sqrt(np.mean(form_err ** 2))),
        rms_z_mission_esc=np.sqrt(np.mean(z_err[mis] ** 2, axis=0)),
        max_z_mission_esc=np.abs(z_err[mis]).max(axis=0),
        final_error=float(np.linalg.norm(pc[-1] - WAYPOINTS[-1])),
        arrive_error=float(np.linalg.norm(pc[a] - WAYPOINTS[-1])),
        heading_end_change=float(np.degrees(abs(np.angle(np.exp(1j * (psi_log[-1] - psi_log[a])))))),
        route_len=float(np.sum(np.linalg.norm(np.diff(WAYPOINTS, axis=0), axis=1))),
    )

    print(f'Route: {len(WAYPOINTS)} waypoints, {res["route_len"]:.1f} m, '
          f'altitude {WAYPOINTS[:, 2].min():.0f}-{WAYPOINTS[:, 2].max():.0f} m; '
          f'{N_ESCORTS} escorts, d = {D_FORM} m')
    print(f'Waypoint switches at t = {", ".join(f"{s:.2f}" for s in wp_switch_t)} s')
    print(f'Carrier arrived (within {ARRIVAL_TOL} m): {arrived} at t = {t[a]:.2f} s '
          f'(error {res["arrive_error"]:.3f} m); run ends at t = {t[-1]:.2f} s '
          f'after a {HOLD_TIME:.0f} s hold, final error {res["final_error"]:.3f} m')
    print(f'Heading change during the hold: {res["heading_end_change"]:.2f} deg (held at rest)')
    print(f'RMS 3-D formation error, mission (0-{t[a]:.2f} s): {res["rms_mission"]:.4f} m')
    print(f'RMS 3-D formation error, hold after arrival     : {res["rms_hold"]:.4f} m')
    print(f'RMS 3-D formation error, whole run              : {res["rms_all"]:.4f} m')
    for i in range(N_ESCORTS):
        print(f'  {ESCORT_LABELS[i]:<15s} mission RMS {res["rms_mission_esc"][i]:.4f} m, '
              f'max {res["max_mission_esc"][i]:.4f} m; vertical RMS '
              f'{res["rms_z_mission_esc"][i]:.4f} m, max |dz| {res["max_z_mission_esc"][i]:.4f} m')
    wp_, we_ = res['worst_pair'], res['worst_esc']
    print(f'Min separation, all pairs (Carrier included): {pair_min[wp_]:.4f} m '
          f'({NAMES[wp_[0]]} - {NAMES[wp_[1]]})  d_safe = {D_SAFE} m')
    print(f'Min separation, escort pairs only          : {pair_min[we_]:.4f} m '
          f'({NAMES[we_[0]]} - {NAMES[we_[1]]})')
    print(f'Steps with repulsion active: {int(res["rep"].sum())} of {len(t)}')
    print(f'Steps with an escort at the {A_ESCORT} m/s^2 limit: '
          f'{int((res["acc_e"] >= A_ESCORT - 1e-9).any(axis=1).sum())}')
    pass_sep = pair_min[wp_] >= D_SAFE
    pass_arr = arrived and res['arrive_error'] < ARRIVAL_TOL
    print(f'[{"PASS" if pass_sep else "FAIL"}] min separation over all pairs >= d_safe')
    print(f'[{"PASS" if pass_arr else "FAIL"}] carrier arrived within {ARRIVAL_TOL} m')
    return res


# ── Plots ─────────────────────────────────────────────────────────────────────

def _snap_idx(res, every=6.0):
    k = max(1, int(round(every / DT)))
    return list(range(0, res['arrive_step'] + 1, k))


def plot_trajectory_3d(res, out_dir):
    pc, pe = res['traj_c'], res['traj_e']
    fig = plt.figure(figsize=(13, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(*pc.T, color='green', lw=2.2, label='Carrier')
    for i in range(N_ESCORTS):
        ax.plot(*pe[:, i].T, color=ESCORT_COLORS[i], lw=1.2, alpha=0.85, label=ESCORT_LABELS[i])
    for k in _snap_idx(res):
        for i in range(N_ESCORTS):
            ax.plot(*np.vstack([pc[k], pe[k, i]]).T, color='gray', lw=0.6, alpha=0.6)
    ax.scatter(*WAYPOINTS[1:-1].T, color='lime', s=50, marker='D', label='Waypoints')
    ax.scatter(*WAYPOINTS[0], color='green', s=70, marker='o')
    ax.scatter(*WAYPOINTS[-1], color='green', s=180, marker='*', label='Goal')
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
    ax.set_xlim(-5, 105); ax.set_ylim(-15, 20); ax.set_zlim(0, 25)
    ax.set_box_aspect((3.0, 1.0, 1.0))
    ax.set_title('S028 3D Cargo Escort — trajectories (grey spokes: formation every 6 s)')
    ax.legend(loc='upper left', fontsize=8)
    ax.view_init(elev=15, azim=-105)
    path = os.path.join(out_dir, 'trajectory_3d.png')
    plt.savefig(path, dpi=140, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_topdown_xy(res, out_dir):
    pc, pe = res['traj_c'], res['traj_e']
    fig, ax = plt.subplots(figsize=(13, 5.5))
    ax.plot(pc[:, 0], pc[:, 1], color='green', lw=2, label='Carrier')
    for i in range(N_ESCORTS):
        ax.plot(pe[:, i, 0], pe[:, i, 1], color=ESCORT_COLORS[i], lw=1.2, alpha=0.85,
                label=ESCORT_LABELS[i])
    for k in _snap_idx(res):
        for i in range(N_ESCORTS):
            ax.plot([pc[k, 0], pe[k, i, 0]], [pc[k, 1], pe[k, i, 1]], color='gray', lw=0.7)
    ax.scatter(WAYPOINTS[1:-1, 0], WAYPOINTS[1:-1, 1], color='lime', s=50, marker='D', zorder=5)
    ax.scatter(*WAYPOINTS[-1, :2], color='green', s=180, marker='*', zorder=5, label='Goal')
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_aspect('equal'); ax.grid(True, alpha=0.3)
    ax.set_title('S028 3D — top view (Top / Bottom escorts sit over / under the Carrier)')
    ax.legend(fontsize=8, loc='lower left', ncol=3)
    path = os.path.join(out_dir, 'topdown_xy.png')
    plt.savefig(path, dpi=140, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def _mark_wp(ax, res):
    for s in res['wp_switch_t']:
        ax.axvline(s, color='lime', lw=1.0, ls='--')
    ax.axvline(res['t'][res['arrive_step']], color='green', lw=1.2, ls='-.')


def plot_altitude_profile(res, out_dir):
    t, pc, pe = res['t'], res['traj_c'], res['traj_e']
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    ax = axes[0]
    ax.plot(t, pc[:, 2], color='green', lw=2.0, label='Carrier')
    for i in range(N_ESCORTS):
        ax.plot(t, pe[:, i, 2], color=ESCORT_COLORS[i], lw=1.2, label=ESCORT_LABELS[i])
    _mark_wp(ax, res)
    ax.set_ylabel('Altitude Z (m)'); ax.set_ylim(0, 25); ax.grid(True, alpha=0.3)
    ax.set_title('Altitude (dashed: waypoint switches, dash-dot: arrival)')
    ax.legend(fontsize=8, ncol=5, loc='upper left')
    ax = axes[1]
    for i in range(N_ESCORTS):
        ax.plot(t, res['z_err'][:, i], color=ESCORT_COLORS[i], lw=1.2, label=ESCORT_LABELS[i])
    _mark_wp(ax, res)
    ax.axhline(0, color='k', lw=0.6)
    ax.set_xlabel('Time (s)'); ax.set_ylabel('z_escort - z_slot (m)'); ax.grid(True, alpha=0.3)
    ax.set_title('Vertical slot error (carrier vertical accelerations are not fed forward)')
    ax.legend(fontsize=8, ncol=4, loc='lower left')
    plt.suptitle('S028 3D — Altitude profile', fontsize=12)
    plt.tight_layout()
    path = os.path.join(out_dir, 'altitude_profile.png')
    plt.savefig(path, dpi=140)
    plt.close()
    print(f'Saved: {path}')


def plot_formation_metrics(res, out_dir):
    t = res['t']
    fig, axes = plt.subplots(2, 2, figsize=(14, 9))
    ax = axes[0, 0]
    for i in range(N_ESCORTS):
        ax.plot(t, res['form_err'][:, i], color=ESCORT_COLORS[i], lw=1.2, label=ESCORT_LABELS[i])
    _mark_wp(ax, res)
    ax.set_ylabel('3-D slot error (m)'); ax.set_title('3-D formation error per escort')
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3); ax.set_ylim(bottom=0)
    ax = axes[0, 1]
    ax.plot(t, res['min_sep_ts'], color='purple', lw=1.5, label='Min over all 10 pairs (Carrier incl.)')
    ax.axhline(D_SAFE, color='red', ls='--', lw=1.2, label=f'd_safe = {D_SAFE} m')
    _mark_wp(ax, res)
    ax.set_ylabel('Separation (m)'); ax.set_title('Minimum separation')
    ax.legend(fontsize=8, loc='lower right'); ax.grid(True, alpha=0.3); ax.set_ylim(bottom=0)
    ax = axes[1, 0]
    ax.plot(t, np.linalg.norm(res['vel_c'], axis=1), color='green', lw=1.5, label='Carrier speed')
    ax.plot(t, res['vel_c'][:, 2], color='green', lw=1.0, ls=':', label='Carrier v_z')
    _mark_wp(ax, res)
    ax.set_xlabel('Time (s)'); ax.set_ylabel('m/s'); ax.set_title('Carrier speed')
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)
    ax = axes[1, 1]
    for i in range(N_ESCORTS):
        ax.plot(t, res['acc_e'][:, i], color=ESCORT_COLORS[i], lw=1.0, label=ESCORT_LABELS[i])
    ax.axhline(A_ESCORT, color='red', ls='--', lw=1, label=f'a_max = {A_ESCORT} m/s²')
    _mark_wp(ax, res)
    ax.set_xlabel('Time (s)'); ax.set_ylabel('|a| (m/s²)'); ax.set_title('Escort acceleration command')
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)
    plt.suptitle('S028 3D — Cargo escort formation metrics (dashed: waypoint switches, dash-dot: arrival)',
                 fontsize=12)
    plt.tight_layout(rect=[0, 0, 1, 0.97])
    path = os.path.join(out_dir, 'formation_metrics.png')
    plt.savefig(path, dpi=140)
    plt.close()
    print(f'Saved: {path}')


def save_animation(res, out_dir):
    t, pc, pe = res['t'], res['traj_c'], res['traj_e']
    step = max(1, int(round(0.4 / DT)))
    idx = list(range(0, len(t), step))
    if idx[-1] != len(t) - 1:
        idx.append(len(t) - 1)
    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim(-5, 105); ax.set_ylim(-15, 20); ax.set_zlim(0, 25)
    ax.set_box_aspect((3.0, 1.0, 1.0))
    ax.set_xlabel('X (m)', fontsize=8); ax.set_ylabel('Y (m)', fontsize=8); ax.set_zlabel('Z (m)', fontsize=8)
    ax.view_init(elev=15, azim=-105)
    ax.plot(*WAYPOINTS.T, color='lime', lw=0.8, ls=':')
    ax.scatter(*WAYPOINTS[-1], color='green', s=150, marker='*')
    dot_c, = ax.plot([], [], [], 'o', color='green', ms=8)
    tr_c, = ax.plot([], [], [], '-', color='green', lw=1.5, alpha=0.6)
    dots = [ax.plot([], [], [], '^', color=c, ms=6)[0] for c in ESCORT_COLORS]
    spokes = [ax.plot([], [], [], '-', color='gray', lw=0.8)[0] for _ in range(N_ESCORTS)]
    title = ax.set_title('')
    ax.legend(handles=[Line2D([0], [0], color='green', lw=2, label='Carrier')] +
              [Line2D([0], [0], marker='^', color=c, ls='None', label=l)
               for c, l in zip(ESCORT_COLORS, ESCORT_LABELS)], fontsize=7, loc='upper left')

    def update(f):
        i = idx[f]
        dot_c.set_data([pc[i, 0]], [pc[i, 1]]); dot_c.set_3d_properties([pc[i, 2]])
        tr_c.set_data(pc[:i + 1, 0], pc[:i + 1, 1]); tr_c.set_3d_properties(pc[:i + 1, 2])
        for j in range(N_ESCORTS):
            dots[j].set_data([pe[i, j, 0]], [pe[i, j, 1]]); dots[j].set_3d_properties([pe[i, j, 2]])
            spokes[j].set_data([pc[i, 0], pe[i, j, 0]], [pc[i, 1], pe[i, j, 1]])
            spokes[j].set_3d_properties([pc[i, 2], pe[i, j, 2]])
        title.set_text(f'S028 3D Cargo Escort  |  t = {t[i]:5.1f} s  |  '
                       f'max slot error {res["form_err"][i].max():.2f} m')
        return [dot_c, tr_c, title] + dots + spokes

    ani = animation.FuncAnimation(fig, update, frames=len(idx), interval=80, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer='pillow', fps=12, dpi=80)
    plt.close()
    print(f'Saved: {path}')


if __name__ == '__main__':
    print('=' * 65)
    print('S028 3D Cargo Escort Formation Simulation')
    print('=' * 65)
    res = run_simulation()
    out_dir = OUTPUT_DIR
    os.makedirs(out_dir, exist_ok=True)
    print()
    plot_trajectory_3d(res, out_dir)
    plot_topdown_xy(res, out_dir)
    plot_altitude_profile(res, out_dir)
    plot_formation_metrics(res, out_dir)
    save_animation(res, out_dir)
    print('All outputs saved.')
