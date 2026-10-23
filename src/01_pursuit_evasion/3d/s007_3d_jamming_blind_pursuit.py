"""
S007 3D Upgrade — Jamming & Blind Pursuit
==========================================
The pursuer's GPS is periodically jammed.  During a blackout it must estimate
its OWN 3D position without GPS (card model: last fix + last GPS velocity ×
elapsed time + anisotropic IMU drift).  It always knows the evader's true
position (onboard tracker), so the only error source is the self-position
estimate.

The evader weaves horizontally (heading 60°·sin(2πt/2 s), as in the fixed 2D
S007).  With the altitude-jump tactics it also climbs or dives 2 m at the
start of every jam window, which forces the pursuer to change its vertical
velocity while it is blind.

Pursuit modes:
  - perfect_gps        : no jamming
  - dead_reckoning_imu : x, y, z all dead-reckoned with the last GPS velocity,
                         drift σ = (0.05, 0.05, 0.02) m/√s
  - baro_aided         : x, y dead-reckoned as above; z measured by a
                         barometer whose error is a random walk with
                         σ = 0.005 m/√s
  - freeze             : hover during the jam, resume when GPS returns

Evader tactics:
  - weave      : horizontal weave, constant z = 2 m
  - jump_up    : weave + altitude jump at each jam onset, first jump +2 m,
                 then alternating
  - jump_down  : same, first jump −2 m (clipped to the 0.3 m floor)

Altitude safety: if the pursuer's estimated z falls below Z_MIN while jammed,
it climbs vertically until GPS returns (card rule).  True altitude is
clipped to [Z_MIN, Z_MAX]; the number of steps at which the commanded motion
would have taken the pursuer below Z_MIN is logged as floor contacts.

Drift: per step N(0, σ_k²·dt) on axis k → after τ s the std is σ_k·√τ.
Jam schedule: jammed during [JAM_OFFSET + n·T, JAM_OFFSET + n·T + T_jam).
Dead-reckoning modes are run with N_MC seeds; both DR modes and all tactics
use the same per-step noise array for a given seed (common random numbers).

Usage:
    conda activate drones
    python src/01_pursuit_evasion/3d/s007_3d_jamming_blind_pursuit.py
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

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), '..', '..', '..',
                          'outputs', '01_pursuit_evasion', '3d', 's007_3d_jamming_blind_pursuit')

# ── Parameters ───────────────────────────────────────────────
JAM_PERIOD    = 3.0    # s
JAM_DURATION  = 1.0    # s  (33% duty cycle)
JAM_OFFSET    = 0.5    # s  first jam onset (drone already moving)
SIGMA_XY      = 0.05   # m/√s horizontal IMU drift
SIGMA_Z_IMU   = 0.02   # m/√s vertical IMU drift (IMU only)
SIGMA_Z_BARO  = 0.005  # m/√s barometer altitude error (baro-aided)
V_PURSUER     = 5.0    # m/s
V_EVADER      = 3.0    # m/s
WEAVE_AMP     = np.radians(60.0)
WEAVE_PERIOD  = 2.0    # s
JUMP_DZ       = 2.0    # m  altitude jump per jam window
JUMP_VZ_FRAC  = 0.7    # vertical share of evader speed while jumping (2.1 m/s)
CAPTURE_R     = 0.15   # m
DT            = 1 / 48
MAX_TIME      = 30.0
Z_MIN         = 0.3
Z_MAX         = 8.0
N_MC          = 100
PLOT_SEED     = 0
SWEEP_OFFSETS = np.arange(0.25, 3.01, 0.25)
N_SWEEP_SEEDS = 20

INIT_PURSUER = np.array([-3.0, 0.0, 2.0])
INIT_EVADER  = np.array([ 3.0, 0.0, 2.0])

PURSUIT_MODES = ['perfect_gps', 'dead_reckoning_imu', 'baro_aided', 'freeze']
MODE_LABELS   = {'perfect_gps': 'Perfect GPS', 'dead_reckoning_imu': 'DR IMU-only',
                 'baro_aided': 'DR + baro', 'freeze': 'Freeze'}
MODE_COLORS   = {'perfect_gps': 'mediumseagreen', 'dead_reckoning_imu': 'darkorange',
                 'baro_aided': 'purple', 'freeze': 'saddlebrown'}
DR_MODES      = ['dead_reckoning_imu', 'baro_aided']
EVADER_TACTICS = ['weave', 'jump_up', 'jump_down']
TACTIC_LABELS  = {'weave': 'Weave (z const)', 'jump_up': 'Weave + jump (up first)',
                  'jump_down': 'Weave + jump (down first)'}
MAX_STEPS = int(round(MAX_TIME / DT))


# ── Helpers ──────────────────────────────────────────────────

def is_jammed(t, offset=JAM_OFFSET):
    return t >= offset and ((t - offset) % JAM_PERIOD) < JAM_DURATION


def noise_array(seed):
    """Per-step standard-normal drift draws, shared by every DR mode/tactic."""
    return np.random.default_rng(seed).standard_normal((MAX_STEPS, 3))


# ── Simulation ───────────────────────────────────────────────

def run_simulation(mode, evader_tactic, seed=PLOT_SEED, jam_offset=JAM_OFFSET):
    """Returns dict with p_traj, e_traj, est_traj (pursuer self-estimate),
    est_err, z_err, jam_log, times, captured, cap_time, floor_contacts,
    safety_steps."""
    noise = noise_array(seed)
    pursuer = DroneBase(INIT_PURSUER.copy(), max_speed=V_PURSUER, dt=DT)
    evader = DroneBase(INIT_EVADER.copy(), max_speed=V_EVADER, dt=DT)

    p_hat = pursuer.pos.copy()
    v_last = np.zeros(3)
    baro_err = 0.0
    jammed = False
    climb_latch = False
    jump_sign = 1.0 if evader_tactic == 'jump_up' else -1.0
    jam_e_prev = False
    z_target = INIT_EVADER[2]

    p_l, e_l, est_l = [pursuer.pos.copy()], [evader.pos.copy()], [p_hat.copy()]
    err_l, zerr_l, jam_l, t_l = [0.0], [0.0], [0], [0.0]
    captured, cap_time = False, None
    floor_contacts, safety_steps = 0, 0

    for step in range(MAX_STEPS):
        t = step * DT
        t_next = (step + 1) * DT
        jam_e_now = is_jammed(t, jam_offset)
        jammed_next = is_jammed(t_next, jam_offset) and mode != 'perfect_gps'

        # Evader: new altitude target at each jam onset (the jammer's schedule
        # is known to the evader, also in the perfect-GPS reference run)
        # (onset detected from the previous step's flag, not from is_jammed(t - DT),
        #  which can disagree with it by floating-point rounding)
        if evader_tactic != 'weave' and jam_e_now and not jam_e_prev:
            z_target = float(np.clip(evader.pos[2] + jump_sign * JUMP_DZ, Z_MIN, Z_MAX))
            jump_sign = -jump_sign
        jam_e_prev = jam_e_now

        # ── Pursuer command ──
        if jammed and mode == 'freeze':
            cmd = np.zeros(3)
        elif climb_latch:
            cmd = np.array([0.0, 0.0, V_PURSUER])
            safety_steps += 1
        else:
            diff = evader.pos - p_hat
            n = np.linalg.norm(diff)
            cmd = V_PURSUER * diff / n if n > 1e-8 else np.zeros(3)
        pursuer.step(cmd)
        if pursuer.pos[2] < Z_MIN:
            floor_contacts += 1
        pursuer.pos[2] = np.clip(pursuer.pos[2], Z_MIN, Z_MAX)

        # ── Evader ──
        th = WEAVE_AMP * np.sin(2 * np.pi * t / WEAVE_PERIOD)
        dz = z_target - evader.pos[2]
        if abs(dz) > 1e-6:
            vz = np.sign(dz) * min(JUMP_VZ_FRAC * V_EVADER, abs(dz) / DT)
            vh = np.sqrt(V_EVADER ** 2 - vz ** 2)
        else:
            vz, vh = 0.0, V_EVADER
        evader.step(np.array([vh * np.cos(th), vh * np.sin(th), vz]))
        evader.pos[2] = np.clip(evader.pos[2], Z_MIN, Z_MAX)

        # ── Self-position estimate at t_next ──
        if not jammed_next:
            p_hat = pursuer.pos.copy()            # GPS fix
            v_last = pursuer.vel.copy()
            baro_err = 0.0
            climb_latch = False
        elif mode in DR_MODES:
            sq = np.sqrt(DT)
            p_hat = p_hat + v_last * DT
            p_hat[0] += SIGMA_XY * sq * noise[step, 0]
            p_hat[1] += SIGMA_XY * sq * noise[step, 1]
            if mode == 'dead_reckoning_imu':
                p_hat[2] += SIGMA_Z_IMU * sq * noise[step, 2]
            else:
                baro_err += SIGMA_Z_BARO * sq * noise[step, 2]
                p_hat[2] = pursuer.pos[2] + baro_err
            if p_hat[2] < Z_MIN:
                climb_latch = True
        # freeze: drone hovers while jammed, the estimate stays (one-step lag)
        jammed = jammed_next

        p_l.append(pursuer.pos.copy()); e_l.append(evader.pos.copy())
        est_l.append(p_hat.copy())
        err_l.append(np.linalg.norm(pursuer.pos - p_hat))
        zerr_l.append(p_hat[2] - pursuer.pos[2])
        jam_l.append(1 if jammed_next else 0); t_l.append(t_next)

        if np.linalg.norm(pursuer.pos - evader.pos) < CAPTURE_R:
            captured, cap_time = True, t_next
            break

    return dict(p_traj=np.array(p_l), e_traj=np.array(e_l), est_traj=np.array(est_l),
                est_err=np.array(err_l), z_err=np.array(zerr_l), jam_log=np.array(jam_l),
                times=np.array(t_l), captured=captured, cap_time=cap_time,
                floor_contacts=floor_contacts, safety_steps=safety_steps,
                mode=mode, tactic=evader_tactic)


def run_monte_carlo(jam_offset=JAM_OFFSET, n_seeds=N_MC):
    """stats[mode][tactic] = dict(cap, err, zerr, floor, safety, n_cap, n)."""
    stats = {}
    for mode in PURSUIT_MODES:
        stats[mode] = {}
        seeds = range(n_seeds) if mode in DR_MODES else [PLOT_SEED]
        for tac in EVADER_TACTICS:
            caps, errs, zerrs, flo, saf = [], [], [], [], []
            for sd in seeds:
                r = run_simulation(mode, tac, sd, jam_offset)
                caps.append(r['cap_time'] if r['captured'] else np.nan)
                errs.append(r['est_err'].max())
                zerrs.append(np.abs(r['z_err']).max())
                flo.append(r['floor_contacts']); saf.append(r['safety_steps'])
            caps = np.array(caps)
            stats[mode][tac] = dict(cap=caps, err=np.array(errs), zerr=np.array(zerrs),
                                    floor=np.array(flo), safety=np.array(saf),
                                    n_cap=int(np.sum(~np.isnan(caps))), n=len(caps))
    return stats


def run_offset_sweep():
    """table[tactic][mode] = list of (mean cap, n_cap, n, runs with floor
    contact, runs with safety climb) per onset."""
    table = {tac: {m: [] for m in PURSUIT_MODES} for tac in EVADER_TACTICS}
    for off in SWEEP_OFFSETS:
        st = run_monte_carlo(off, N_SWEEP_SEEDS)
        for tac in EVADER_TACTICS:
            for m in PURSUIT_MODES:
                s = st[m][tac]
                table[tac][m].append((np.nanmean(s['cap']) if s['n_cap'] else np.nan,
                                      s['n_cap'], s['n'], int(np.sum(s['floor'] > 0)),
                                      int(np.sum(s['safety'] > 0))))
    return table


# ── Plots ─────────────────────────────────────────────────────

def shade_jams(ax, t_max, offset=JAM_OFFSET, label=True):
    t0 = offset
    while t0 < t_max:
        ax.axvspan(t0, t0 + JAM_DURATION, color='salmon', alpha=0.15,
                   label='Jam window' if label else None)
        label = False
        t0 += JAM_PERIOD


def plot_trajectories_3d(results, tactic, out_dir):
    """4 subplots — 4 pursuit modes against one evader tactic (seed PLOT_SEED)."""
    fig = plt.figure(figsize=(18, 12))
    for idx, mode in enumerate(PURSUIT_MODES):
        res = results[mode][tactic]
        ax = fig.add_subplot(2, 2, idx + 1, projection='3d')
        ax.view_init(elev=20, azim=-60)
        p, e, est = res['p_traj'], res['e_traj'], res['est_traj']
        ax.plot(p[:, 0], p[:, 1], p[:, 2], color='red', linewidth=1.8, label='Pursuer (true)')
        est_j = est.copy(); est_j[~res['jam_log'].astype(bool)] = np.nan
        ax.plot(est_j[:, 0], est_j[:, 1], est_j[:, 2], color=MODE_COLORS[mode],
                linewidth=1.4, linestyle='--', label='Self-estimate (jammed)')
        ax.plot(e[:, 0], e[:, 1], e[:, 2], color='royalblue', linewidth=1.8, label='Evader')
        ax.scatter(*p[0], color='red', s=50); ax.scatter(*e[0], color='blue', s=50)
        if res['captured']:
            ax.scatter(*p[-1], color='black', s=90, marker='X')
            status = f'Captured {res["cap_time"]:.2f} s'
        else:
            status = 'Timeout'
        status += f' | floor contacts {res["floor_contacts"]}'
        ax.set_title(f'{MODE_LABELS[mode]}\n{status}', fontsize=10)
        ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
        ax.set_zlim(0, 5)
        ax.legend(fontsize=7, loc='upper left')
    fig.suptitle(f'S007 3D Jamming — 4 pursuit modes vs "{TACTIC_LABELS[tactic]}" '
                 f'(seed {PLOT_SEED})', fontsize=12)
    plt.tight_layout()
    path = os.path.join(out_dir, 'trajectories_3d.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_position_error_3d(results, out_dir):
    """Self-position error vs time, one panel per evader tactic (seed PLOT_SEED)."""
    fig, axes = plt.subplots(1, 3, figsize=(18, 5), sharey=True)
    for ax, tac in zip(axes, EVADER_TACTICS):
        t_max = 0.0
        for mode in ['dead_reckoning_imu', 'baro_aided', 'freeze']:
            r = results[mode][tac]
            ax.plot(r['times'], r['est_err'], color=MODE_COLORS[mode], linewidth=1.6,
                    label=MODE_LABELS[mode])
            if r['captured']:
                ax.axvline(r['cap_time'], color=MODE_COLORS[mode], linestyle='--',
                           linewidth=1.0, alpha=0.7)
            t_max = max(t_max, r['times'][-1])
        shade_jams(ax, t_max)
        ax.set_title(TACTIC_LABELS[tac], fontsize=10)
        ax.set_xlabel('Time (s)'); ax.grid(True, alpha=0.3)
    axes[0].set_ylabel('Self-position error ||p_hat - p|| (m)')
    axes[0].legend(fontsize=8)
    fig.suptitle(f'S007 3D Jamming — own-position estimation error (seed {PLOT_SEED}; '
                 'dashed = capture)', fontsize=11)
    plt.tight_layout()
    path = os.path.join(out_dir, 'position_error_3d.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def plot_altitude_subplot(cols, out_dir):
    """True vs estimated pursuer z and evader z, IMU-only (top) vs baro (bottom).
    cols = list of (title, jam_offset, {mode: result}) — one column each."""
    fig, axes = plt.subplots(2, len(cols), figsize=(8 * len(cols), 8), squeeze=False)
    for c, (ctitle, offset, runs) in enumerate(cols):
        for row, mode in enumerate(DR_MODES):
            ax = axes[row, c]
            r = runs[mode]
            ax.plot(r['times'], r['p_traj'][:, 2], color='red', linewidth=1.6,
                    label='Pursuer z (true)')
            ax.plot(r['times'], r['est_traj'][:, 2], color=MODE_COLORS[mode], linewidth=1.4,
                    linestyle='--', label='Pursuer z (estimate)')
            ax.plot(r['times'], r['e_traj'][:, 2], color='royalblue', linewidth=1.6,
                    label='Evader z')
            ax.axhline(Z_MIN, color='grey', linestyle=':', label=f'z_min = {Z_MIN} m')
            shade_jams(ax, r['times'][-1], offset)
            cap = f'captured {r["cap_time"]:.2f} s' if r['captured'] else 'timeout'
            ax.set_title(f'{ctitle}\n{MODE_LABELS[mode]} — {cap}, max |z err| '
                         f'{np.abs(r["z_err"]).max():.2f} m, floor contacts '
                         f'{r["floor_contacts"]}', fontsize=9)
            ax.set_ylabel('Altitude (m)'); ax.set_xlabel('Time (s)')
            ax.grid(True, alpha=0.3); ax.legend(fontsize=7)
    fig.suptitle(f'S007 3D Jamming — altitude, "{TACTIC_LABELS["jump_down"]}" evader '
                 f'(seed {PLOT_SEED})', fontsize=11)
    plt.tight_layout()
    path = os.path.join(out_dir, 'altitude_subplot.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def plot_capture_table(stats, out_dir):
    """Mean capture time over the MC runs, modes × tactics."""
    fig, ax = plt.subplots(figsize=(13, 4.2))
    ax.axis('off')
    data, colors = [], []
    for mode in PURSUIT_MODES:
        row, rc = [], []
        for tac in EVADER_TACTICS:
            s = stats[mode][tac]
            if s['n_cap'] == 0:
                txt, col = 'Missed', '#f5b7b1'
            else:
                txt = f'{np.nanmean(s["cap"]):.2f} s'
                if s['n'] > 1:
                    txt += f' [{np.nanmin(s["cap"]):.2f}, {np.nanmax(s["cap"]):.2f}]'
                if s['n_cap'] < s['n']:
                    txt += f' ({s["n_cap"]}/{s["n"]})'
                col = '#b7e4b7'
            row.append(txt); rc.append(col)
        data.append(row); colors.append(rc)
    tb = ax.table(cellText=data, rowLabels=[MODE_LABELS[m] for m in PURSUIT_MODES],
                  colLabels=[TACTIC_LABELS[t] for t in EVADER_TACTICS],
                  cellColours=colors, loc='center', cellLoc='center')
    tb.auto_set_font_size(False); tb.set_fontsize(11); tb.scale(1.2, 2.0)
    ax.set_title(f'S007 3D Jamming — capture time (DR modes: mean [min, max] over {N_MC} '
                 'drift seeds, common random numbers)', fontsize=11, pad=20)
    plt.tight_layout()
    path = os.path.join(out_dir, 'capture_table.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_offset_sweep(table, out_dir):
    fig, axes = plt.subplots(1, 3, figsize=(18, 5), sharey=True)
    for ax, tac in zip(axes, EVADER_TACTICS):
        for m in PURSUIT_MODES:
            means = [v[0] for v in table[tac][m]]
            ax.plot(SWEEP_OFFSETS, means, 'o-', color=MODE_COLORS[m], label=MODE_LABELS[m])
        ax.axvline(JAM_OFFSET, color='grey', linestyle=':')
        ax.set_title(TACTIC_LABELS[tac], fontsize=10)
        ax.set_xlabel('First jam onset (s)'); ax.grid(True, alpha=0.3)
    axes[0].set_ylabel(f'Mean capture time (s; DR: {N_SWEEP_SEEDS} seeds)')
    axes[0].legend(fontsize=8)
    fig.suptitle('S007 3D Jamming — capture time vs jam timing (dotted = main-run onset)',
                 fontsize=11)
    plt.tight_layout()
    path = os.path.join(out_dir, 'jam_offset_sweep.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def save_animation(res_imu, res_baro, out_dir):
    """IMU-only vs baro-aided against the same evader (seed PLOT_SEED)."""
    runs = [res_imu, res_baro]
    max_len = max(len(r['p_traj']) for r in runs)
    step_skip = 3
    n_frames = (max_len - 1) // step_skip + 2
    fig = plt.figure(figsize=(14, 6.5))
    arts = []
    for k, r in enumerate(runs):
        ax = fig.add_subplot(1, 2, k + 1, projection='3d')
        pts = np.vstack([r['p_traj'], r['e_traj']])
        lo = pts.min(axis=0) - 0.8; hi = pts.max(axis=0) + 0.8
        ax.set_xlim(lo[0], hi[0]); ax.set_ylim(lo[1], hi[1]); ax.set_zlim(0, max(5, hi[2]))
        ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
        ax.view_init(elev=15, azim=-60)
        lp, = ax.plot([], [], [], color='red', linewidth=1.6, label='Pursuer (true)')
        le, = ax.plot([], [], [], color='royalblue', linewidth=1.6, label='Evader')
        dp, = ax.plot([], [], [], 'r^', markersize=9)
        de, = ax.plot([], [], [], 'bs', markersize=8)
        dx, = ax.plot([], [], [], 'x', color=MODE_COLORS[r['mode']], markersize=9,
                      markeredgewidth=2, label='Self-estimate')
        ax.legend(fontsize=7, loc='upper left')
        ti = ax.set_title('', fontsize=9)
        arts.append((lp, le, dp, de, dx, ti))

    def update(i):
        out = []
        for r, (lp, le, dp, de, dx, ti) in zip(runs, arts):
            n = len(r['p_traj']); si = min(i * step_skip, n - 1)
            p, e, est = r['p_traj'], r['e_traj'], r['est_traj']
            lp.set_data(p[:si + 1, 0], p[:si + 1, 1]); lp.set_3d_properties(p[:si + 1, 2])
            le.set_data(e[:si + 1, 0], e[:si + 1, 1]); le.set_3d_properties(e[:si + 1, 2])
            dp.set_data([p[si, 0]], [p[si, 1]]); dp.set_3d_properties([p[si, 2]])
            de.set_data([e[si, 0]], [e[si, 1]]); de.set_3d_properties([e[si, 2]])
            dx.set_data([est[si, 0]], [est[si, 1]]); dx.set_3d_properties([est[si, 2]])
            if si == n - 1 and r['captured']:
                ti.set_text(f'{MODE_LABELS[r["mode"]]}\nCaptured {r["cap_time"]:.2f} s')
            else:
                jam = ' [JAM]' if r['jam_log'][si] else ''
                ti.set_text(f'{MODE_LABELS[r["mode"]]}\nt={r["times"][si]:.2f}s{jam} '
                            f'err={r["est_err"][si]:.2f} m')
            out += [lp, le, dp, de, dx, ti]
        return out

    fig.suptitle(f'S007 3D Jamming — "{TACTIC_LABELS[res_imu["tactic"]]}" evader '
                 f'(seed {PLOT_SEED}; red ▲ pursuer, × self-estimate, blue ■ evader)',
                 fontsize=10)
    ani = FuncAnimation(fig, update, frames=n_frames, interval=70, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer=PillowWriter(fps=14), dpi=80)
    plt.close()
    print(f'Saved: {path}  ({n_frames} frames, every {step_skip} steps)')


# ── Main ─────────────────────────────────────────────────────

if __name__ == '__main__':
    out_dir = os.path.normpath(OUTPUT_DIR)
    os.makedirs(out_dir, exist_ok=True)
    results = {m: {t: run_simulation(m, t, PLOT_SEED) for t in EVADER_TACTICS}
               for m in PURSUIT_MODES}
    print(f'Plotted runs (seed {PLOT_SEED}, first jam {JAM_OFFSET} s):')
    for m in PURSUIT_MODES:
        for t in EVADER_TACTICS:
            r = results[m][t]
            st = f'captured @ {r["cap_time"]:.3f} s' if r['captured'] else 'timeout'
            print(f'  [{m:<18} | {t:<9}] {st:<20} max err {r["est_err"].max():.3f} m, '
                  f'max |z err| {np.abs(r["z_err"]).max():.3f} m, floor contacts '
                  f'{r["floor_contacts"]}, safety-climb steps {r["safety_steps"]}')

    stats = run_monte_carlo()
    print(f'\nMonte Carlo (DR modes: seeds 0..{N_MC - 1}, common random numbers; '
          'perfect_gps/freeze deterministic):')
    for m in PURSUIT_MODES:
        for t in EVADER_TACTICS:
            s = stats[m][t]
            c = s['cap']
            ctxt = (f'mean {np.nanmean(c):.3f} s, min {np.nanmin(c):.3f}, max {np.nanmax(c):.3f}'
                    if s['n_cap'] else 'no capture')
            print(f'  [{m:<18} | {t:<9}] captured {s["n_cap"]}/{s["n"]} | {ctxt} | '
                  f'max err mean {s["err"].mean():.3f} m | max |z err| mean {s["zerr"].mean():.3f} m'
                  f' | runs with floor contact {int(np.sum(s["floor"] > 0))}/{s["n"]}'
                  f' | runs with safety climb {int(np.sum(s["safety"] > 0))}/{s["n"]}')
    for t in EVADER_TACTICS:
        d = stats['dead_reckoning_imu'][t]['cap'] - stats['baro_aided'][t]['cap']
        print(f'  paired IMU-only - baro [{t:<9}]: mean {np.nanmean(d):+.3f} s, '
              f'IMU-only slower in {int(np.sum(d > 1e-9))}/{N_MC}, faster in '
              f'{int(np.sum(d < -1e-9))}/{N_MC}')

    table = run_offset_sweep()
    print(f'\nJam-onset sweep (mean capture time, s; DR modes over {N_SWEEP_SEEDS} seeds):')
    for t in EVADER_TACTICS:
        print(f'  tactic {t}')
        print('    onset  ' + '  '.join(f'{m:>18}' for m in PURSUIT_MODES))
        for k, off in enumerate(SWEEP_OFFSETS):
            row = []
            for m in PURSUIT_MODES:
                mu, nc, n = table[t][m][k][:3]
                row.append(f'{mu:18.3f}' if nc == n else f'{mu:12.3f}({nc}/{n})')
            print(f'    {off:5.2f}  ' + '  '.join(row))
        for m in PURSUIT_MODES:
            nf = [(round(float(SWEEP_OFFSETS[k]), 2), v[3])
                  for k, v in enumerate(table[t][m]) if v[3]]
            ns = sum(v[4] for v in table[t][m])
            print(f'    {m:<18} runs with floor contact (onset, runs): {nf} | '
                  f'runs with safety climb: {ns}')

    print('\nFloor-contact example (first jam at 1.50 s, jump_down, seed 0):')
    end_runs = {}
    for m in DR_MODES:
        ex = run_simulation(m, 'jump_down', PLOT_SEED, 1.5)
        end_runs[m] = ex
        at_floor = np.where(ex['p_traj'][:, 2] <= Z_MIN + 1e-9)[0]
        txt = (f'first at t={ex["times"][at_floor[0]]:.3f} s, estimated z there '
               f'{ex["est_traj"][at_floor[0], 2]:.3f} m' if len(at_floor) else 'never at floor')
        k_ret = int(np.argmin(np.abs(ex['times'] - (1.5 + JAM_DURATION))))
        k_on = int(np.argmin(np.abs(ex['times'] - 1.5)))
        rng_ = np.linalg.norm(ex['p_traj'] - ex['e_traj'], axis=1)
        print(f'  [{m:<18}] floor contacts {ex["floor_contacts"]} steps ({txt}); '
              f'min estimated z while jammed '
              f'{ex["est_traj"][ex["jam_log"] == 1, 2].min():.3f} m; range at jam onset '
              f'{rng_[k_on]:.3f} m, min range in jam {rng_[k_on:k_ret].min():.3f} m, '
              f'range when GPS returns (t={ex["times"][k_ret]:.3f} s) {rng_[k_ret]:.3f} m, '
              f'max horizontal est. error {np.linalg.norm(ex["est_traj"][:, :2] - ex["p_traj"][:, :2], axis=1).max():.3f} m; '
              f'capture {ex["cap_time"]:.3f} s')

    plot_trajectories_3d(results, 'jump_down', out_dir)
    plot_position_error_3d(results, out_dir)
    plot_altitude_subplot([(f'Main run: first jam {JAM_OFFSET} s', JAM_OFFSET,
                            {m: results[m]['jump_down'] for m in DR_MODES}),
                           ('End-game jam: first jam 1.5 s', 1.5, end_runs)], out_dir)
    plot_capture_table(stats, out_dir)
    plot_offset_sweep(table, out_dir)
    save_animation(results['dead_reckoning_imu']['jump_down'],
                   results['baro_aided']['jump_down'], out_dir)
    print('S007 3D done.')
