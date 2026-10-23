"""
S028 Cargo Escort Formation
===========================
A cargo drone (Carrier) flies a waypoint route while three escorts keep a
triangle formation around it with the card's virtual-structure (VS) law
v_E = K_pE (s_i - p_E) + v_C plus pairwise repulsion.

Four test cases (identical controller unless stated):
  A  straight line (-8,0,2) -> (8,0,2), no wind  (original setup; zero error by
     construction, kept as a baseline)
  B  S-route through two intermediate waypoints (turns of ~129° and ~127°),
     carrier speed 3.0 -> 1.5 -> 3.0 m/s per leg, no wind
  C  route B + a travelling lateral gust w = [0, w_y sin(w t - k x), 0] that
     differs between drones (card Extension 1), no wind compensation
  D  route C with wind compensation: escorts feed forward the carrier's actual
     ground velocity and subtract their own measured wind

All drones are kinematic point masses (velocity commands, clamped).
Separation metrics include every pair, carrier included; escorts are also
repelled by the carrier.

Usage:
    conda activate drones
    python src/02_logistics_delivery/s028_cargo_escort_formation.py
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401
import matplotlib.animation as animation

OUT_DIR = os.path.normpath(os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '02_logistics_delivery', 's028_cargo_escort_formation'))
os.makedirs(OUT_DIR, exist_ok=True)

# ── Constants ─────────────────────────────────────────────────────────────────
N_ESCORTS   = 3
V_CARRIER   = 3.0     # m/s carrier cruise speed (legs 1 and 3)
V_SLOW      = 1.5     # m/s carrier speed on the middle leg of the S-route
V_ESCORT    = 6.0     # m/s escort max speed
K_PC        = 1.5     # carrier position gain
K_DC        = 0.4     # carrier damping gain
K_PE        = 3.0     # escort slot-tracking gain
K_REP       = 3.0     # repulsion gain
D_SAFE      = 1.0     # m
DT          = 1 / 48  # s
T_MAX       = 30.0    # s
ARRIVAL_TOL = 0.2     # m  final goal
WP_RADIUS   = 0.5     # m  intermediate waypoint switch radius
TARGET_RMS  = 0.15    # m  card target
TURN_WINDOW = 3.0     # s  after a waypoint switch counted as turn transient
CRUISE_MIN_V = 1.0    # m/s carrier speed above which a sample counts as cruise

# Wind (travelling lateral gust, differs between drones)
WIND_AMP    = 1.0     # m/s
WIND_OMEGA  = 1.0     # rad/s
WIND_LAMBDA = 12.0    # m   wavelength along x
WIND_K      = 2 * np.pi / WIND_LAMBDA

SLOT_OFFSETS = np.array([
    [2.0, 0.0, 0.0],      # Escort-1 Lead
    [-1.5, 2.0, 0.0],     # Escort-2 Left
    [-1.5, -2.0, 0.0],    # Escort-3 Right
])

ROUTE_STRAIGHT = ([np.array([-8.0, 0.0, 2.0]), np.array([8.0, 0.0, 2.0])],
                  [V_CARRIER])
ROUTE_S = ([np.array([-7.0, -6.0, 2.0]), np.array([-3.0, 5.0, 2.0]),
            np.array([3.0, -5.0, 2.0]), np.array([7.0, 5.0, 2.0])],
           [V_CARRIER, V_SLOW, V_CARRIER])

CASES = {
    'A': dict(route=ROUTE_STRAIGHT, wind=False, comp=False,
              name='A: straight, no wind'),
    'B': dict(route=ROUTE_S, wind=False, comp=False,
              name='B: S-route, no wind'),
    'C': dict(route=ROUTE_S, wind=True, comp=False,
              name='C: S-route + gust'),
    'D': dict(route=ROUTE_S, wind=True, comp=True,
              name='D: S-route + gust, wind comp.'),
}

ESCORT_COLORS = ['red', 'blue', 'orange']
ESCORT_LABELS = ['Escort-1 (Lead)', 'Escort-2 (Left)', 'Escort-3 (Right)']
DRONE_NAMES   = ['Carrier', 'Lead', 'Left', 'Right']


def rotation_z(psi):
    c, s = np.cos(psi), np.sin(psi)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def slot_positions(pos_c, psi):
    R = rotation_z(psi)
    return pos_c + SLOT_OFFSETS @ R.T


def wind(pos, t, on):
    if not on:
        return np.zeros_like(pos)
    w = np.zeros_like(pos)
    w[..., 1] = WIND_AMP * np.sin(WIND_OMEGA * t - WIND_K * pos[..., 0])
    return w


def repulsion(p_i, p_j):
    r = p_i - p_j
    d = np.linalg.norm(r) + 1e-8
    if d < D_SAFE:
        return K_REP * (D_SAFE - d) / d * (r / d)
    return np.zeros(3)


# ── Simulation ────────────────────────────────────────────────────────────────

def run_simulation(case):
    cfg = CASES[case]
    wps, speeds = cfg['route']
    pos_c = wps[0].copy()
    leg = 0
    d0 = wps[1] - wps[0]
    psi = np.arctan2(d0[1], d0[0])
    pos_e = slot_positions(pos_c, psi)
    vel_c = np.zeros(3)

    T, PC, PE, SL, ERR, PSI, WP, SPD = [], [], [], [], [], [], [], []
    rep_steps = 0
    t_arrive = None
    switch_t = []
    for k in range(int(T_MAX / DT) + 1):
        t = k * DT
        slots = slot_positions(pos_c, psi)
        T.append(t); PC.append(pos_c.copy()); PE.append(pos_e.copy())
        SPD.append(np.linalg.norm(vel_c))
        SL.append(slots); PSI.append(psi); WP.append(leg)
        ERR.append(np.linalg.norm(pos_e - slots, axis=1))

        goal = wps[leg + 1]
        final = leg + 1 == len(wps) - 1
        dist = np.linalg.norm(goal - pos_c)
        if final and dist < ARRIVAL_TOL:
            t_arrive = t
            break
        if not final and dist < WP_RADIUS:
            leg += 1
            switch_t.append(t)
            goal = wps[leg + 1]

        # Carrier PD guidance (card), speed limit per leg
        v_cmd_c = K_PC * (goal - pos_c) - K_DC * vel_c
        sp = np.linalg.norm(v_cmd_c)
        if sp > speeds[leg]:
            v_cmd_c *= speeds[leg] / sp
        vel_c = v_cmd_c
        if np.linalg.norm(vel_c[:2]) > 1e-6:           # hold heading at rest
            psi = np.arctan2(vel_c[1], vel_c[0])
        slots_next = slot_positions(pos_c, psi)

        w_c = wind(pos_c, t, cfg['wind'])
        w_e = wind(pos_e, t, cfg['wind'])
        ff = vel_c + (w_c if cfg['comp'] else 0.0)     # carrier ground velocity
        v_e = K_PE * (slots_next - pos_e) + ff
        if cfg['comp']:
            v_e = v_e - w_e
        active = False
        for i in range(N_ESCORTS):
            others = [pos_c] + [pos_e[j] for j in range(N_ESCORTS) if j != i]
            for p_o in others:
                r = repulsion(pos_e[i], p_o)
                if r.any():
                    active = True
                v_e[i] += r
            s = np.linalg.norm(v_e[i])
            if s > V_ESCORT:
                v_e[i] *= V_ESCORT / s
        rep_steps += active

        pos_c = pos_c + (vel_c + w_c) * DT
        pos_e = pos_e + (v_e + w_e) * DT

    T = np.array(T); PC = np.array(PC); PE = np.array(PE)
    ERR = np.array(ERR)
    allp = np.concatenate([PC[:, None, :], PE], axis=1)       # (K, 4, 3)
    pair_d = {}
    for a in range(4):
        for b in range(a + 1, 4):
            pair_d[(a, b)] = np.linalg.norm(allp[:, a] - allp[:, b], axis=1)
    min_pair = min(pair_d, key=lambda p: pair_d[p].min())
    min_sep_ts = np.min(np.stack(list(pair_d.values())), axis=0)
    esc_pairs = [p for p in pair_d if p[0] > 0]
    min_esc = min(pair_d[p].min() for p in esc_pairs)
    steady = np.ones(len(T), dtype=bool)
    for ts in switch_t:
        steady &= ~((T >= ts) & (T < ts + TURN_WINDOW))
    leg_ang = np.array([np.arctan2(*(wps[l + 1] - wps[l])[1::-1])
                        for l in WP])
    hdg_dev = np.degrees(np.abs(np.angle(np.exp(1j * (np.array(PSI)
                                                      - leg_ang)))))
    return dict(
        case=case, name=cfg['name'], t=T, traj_c=PC, traj_e=PE,
        slots=np.array(SL), psi=np.array(PSI), form_err=ERR,
        pair_d=pair_d, min_sep_ts=min_sep_ts, min_pair=min_pair,
        min_sep=float(min_sep_ts.min()), min_sep_escorts=float(min_esc),
        rms_total=float(np.sqrt(np.mean(ERR**2))),
        rms_per=np.sqrt(np.mean(ERR**2, axis=0)),
        max_err=float(ERR.max()), rep_steps=int(rep_steps),
        rms_steady=float(np.sqrt(np.mean(ERR[steady]**2))),
        max_steady=float(ERR[steady].max()),
        hdg_dev_steady=float(hdg_dev[steady].max()),
        rms_cruise=float(np.sqrt(np.mean(
            ERR[steady & (np.array(SPD) >= CRUISE_MIN_V)]**2))),
        hdg_dev_cruise=float(hdg_dev[steady & (np.array(SPD)
                                               >= CRUISE_MIN_V)].max()),
        t_arrive=t_arrive, switch_t=switch_t,
        final_err=float(np.linalg.norm(PC[-1] - wps[-1])),
        wps=wps, wind=cfg['wind'])


# ── Plots ─────────────────────────────────────────────────────────────────────

def _snapshots(ax, res, every=2.0, three_d=False):
    step = int(round(every / DT))
    for k in list(range(0, len(res['t']), step)) + [len(res['t']) - 1]:
        pts = np.vstack([res['traj_e'][k], res['traj_e'][k, :1]])
        if three_d:
            ax.plot(pts[:, 0], pts[:, 1], pts[:, 2], color='gray', lw=0.7,
                    ls='--', alpha=0.5)
        else:
            ax.plot(pts[:, 0], pts[:, 1], color='gray', lw=0.7, ls='--',
                    alpha=0.5)
            ax.plot(*res['traj_c'][k, :2], 'o', color='green', ms=3)


def _route(ax, res, three_d=False):
    w = np.array(res['wps'])
    if three_d:
        ax.plot(w[:, 0], w[:, 1], w[:, 2], 'k:', lw=1, label='Route')
        ax.scatter(*w[-1], color='green', s=150, marker='*', label='Goal')
    else:
        ax.plot(w[:, 0], w[:, 1], 'k:', lw=1, label='Route')
        ax.scatter(*w[-1, :2], color='green', s=150, marker='*',
                   label='Goal', zorder=6)


def plot_trajectory_3d(res):
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')
    _route(ax, res, True)
    ax.plot(*res['traj_c'].T, color='green', lw=2, label='Carrier')
    for i in range(N_ESCORTS):
        ax.plot(*res['traj_e'][:, i].T, color=ESCORT_COLORS[i], lw=1.4,
                label=ESCORT_LABELS[i])
    _snapshots(ax, res, three_d=True)
    ax.set_zlim(0, 5)
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
    ax.set_title(f'S028 — 3D trajectories, case {res["name"]}\n'
                 '(grey dashed: formation every 2 s)')
    ax.legend(loc='upper left', fontsize=8)
    plt.tight_layout()
    _save(fig, 'trajectory_3d.png')


def plot_topdown_xy(runs):
    fig, axes = plt.subplots(1, 3, figsize=(18, 6.5))
    for ax, c in zip(axes, ['B', 'C', 'D']):
        res = runs[c]
        _route(ax, res)
        ax.plot(res['traj_c'][:, 0], res['traj_c'][:, 1], color='green',
                lw=2, label='Carrier')
        for i in range(N_ESCORTS):
            ax.plot(res['traj_e'][:, i, 0], res['traj_e'][:, i, 1],
                    color=ESCORT_COLORS[i], lw=1.2, label=ESCORT_LABELS[i])
        _snapshots(ax, res)
        ax.set_xlim(-11, 11); ax.set_ylim(-9, 9)
        ax.set_aspect('equal'); ax.grid(True, alpha=0.3)
        ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
        ax.set_title(f'{res["name"]}\nRMS {res["rms_total"]:.3f} m, '
                     f'max {res["max_err"]:.3f} m')
    axes[0].legend(fontsize=7, loc='lower right')
    plt.suptitle('S028 — Top-down view (formation every 2 s)')
    plt.tight_layout()
    _save(fig, 'topdown_xy.png')


def plot_formation_error(runs):
    fig, axes = plt.subplots(3, 1, figsize=(11, 10), sharex=True)
    for ax, c in zip(axes, ['B', 'C', 'D']):
        res = runs[c]
        for i in range(N_ESCORTS):
            ax.plot(res['t'], res['form_err'][:, i], color=ESCORT_COLORS[i],
                    lw=1.3, label=ESCORT_LABELS[i])
        for ts in res['switch_t']:
            ax.axvline(ts, color='k', ls=':', lw=1)
        ax.axhline(TARGET_RMS, color='grey', ls='--', lw=1,
                   label='Card RMS target 0.15 m')
        ax.set_ylabel('Slot error (m)')
        ax.set_title(f'{res["name"]}  (dotted: waypoint switches)')
        ax.grid(True, alpha=0.3); ax.set_ylim(bottom=0)
    axes[0].legend(fontsize=8, ncol=2)
    axes[-1].set_xlabel('Time (s)')
    plt.tight_layout()
    _save(fig, 'formation_metrics.png')


def plot_separation(res):
    fig, ax = plt.subplots(figsize=(11, 4.5))
    for (a, b), d in res['pair_d'].items():
        ax.plot(res['t'], d, lw=1.2,
                label=f'{DRONE_NAMES[a]}–{DRONE_NAMES[b]}')
    ax.axhline(D_SAFE, color='red', ls='--', label=f'd_safe = {D_SAFE} m')
    for ts in res['switch_t']:
        ax.axvline(ts, color='k', ls=':', lw=1)
    ax.set_xlabel('Time (s)'); ax.set_ylabel('Distance (m)')
    ax.set_title(f'S028 — Pairwise separation (all 6 pairs), case '
                 f'{res["name"]}')
    ax.legend(fontsize=8, ncol=4); ax.grid(True, alpha=0.3)
    ax.set_ylim(bottom=0)
    plt.tight_layout()
    _save(fig, 'separation.png')


def plot_combined_metrics(runs):
    cases = list(runs)
    x = np.arange(len(cases))
    fig, axes = plt.subplots(1, 3, figsize=(16, 4.5))
    axes[0].bar(x - 0.27, [runs[c]['rms_total'] for c in cases], width=0.27,
                color='steelblue', label='whole flight')
    axes[0].bar(x, [runs[c]['rms_steady'] for c in cases], width=0.27,
                color='lightgreen',
                label=f'outside {TURN_WINDOW:.0f} s turn windows')
    axes[0].bar(x + 0.27, [runs[c]['rms_cruise'] for c in cases], width=0.27,
                color='darkgreen',
                label=f'... and carrier speed >= {CRUISE_MIN_V} m/s')
    axes[0].axhline(TARGET_RMS, color='k', ls='--', label='target 0.15 m')
    axes[0].set_title('RMS formation error (m)')
    axes[1].bar(x, [runs[c]['max_err'] for c in cases], color='darkorange')
    axes[1].set_title('Max slot error (m)')
    axes[2].bar(x, [runs[c]['min_sep'] for c in cases], color='purple',
                label='all pairs')
    axes[2].scatter(x, [runs[c]['min_sep_escorts'] for c in cases],
                    color='k', marker='D', zorder=5, label='escort pairs only')
    axes[2].axhline(D_SAFE, color='red', ls='--', label='d_safe')
    axes[2].set_title('Minimum separation (m)')
    for ax in axes:
        ax.set_xticks(x)
        ax.set_xticklabels([runs[c]['name'] for c in cases], rotation=15,
                           fontsize=8, ha='right')
        ax.grid(axis='y', alpha=0.3)
    axes[0].legend(fontsize=8); axes[2].legend(fontsize=8)
    plt.suptitle('S028 — Test-case summary')
    plt.tight_layout()
    _save(fig, 'combined_metrics.png')


def _save(fig, name):
    fig.savefig(os.path.join(OUT_DIR, name), dpi=130)
    plt.close(fig)
    print(f'Saved: {name}')


def save_animation(res, n_frames=140, fps=12):
    """Top-down GIF of one case (<= 150 frames)."""
    n = len(res['t'])
    idx = np.linspace(0, n - 1, n_frames).astype(int)
    fig, ax = plt.subplots(figsize=(7, 6))
    _route(ax, res)
    ax.set_xlim(-11, 11); ax.set_ylim(-9, 9); ax.set_aspect('equal')
    ax.grid(True, alpha=0.3); ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
    ax.set_title(f'S028 — {res["name"]}')
    cdot, = ax.plot([], [], 'o', color='green', ms=9, label='Carrier')
    edots = [ax.plot([], [], '^', color=ESCORT_COLORS[i], ms=8,
                     label=ESCORT_LABELS[i])[0] for i in range(N_ESCORTS)]
    sdots, = ax.plot([], [], 'x', color='k', ms=6, label='Slots')
    tri, = ax.plot([], [], '--', color='gray', lw=0.8)
    trails = [ax.plot([], [], '-', color=c, lw=1, alpha=0.4)[0]
              for c in ['green'] + ESCORT_COLORS]
    txt = ax.text(0.02, 0.95, '', transform=ax.transAxes, fontsize=9)
    ax.legend(fontsize=7, loc='lower right')

    def update(f):
        k = idx[f]
        lo = max(0, k - 96)
        cdot.set_data([res['traj_c'][k, 0]], [res['traj_c'][k, 1]])
        trails[0].set_data(res['traj_c'][lo:k+1, 0], res['traj_c'][lo:k+1, 1])
        for i in range(N_ESCORTS):
            edots[i].set_data([res['traj_e'][k, i, 0]],
                              [res['traj_e'][k, i, 1]])
            trails[i+1].set_data(res['traj_e'][lo:k+1, i, 0],
                                 res['traj_e'][lo:k+1, i, 1])
        sdots.set_data(res['slots'][k, :, 0], res['slots'][k, :, 1])
        pts = np.vstack([res['traj_e'][k], res['traj_e'][k, :1]])
        tri.set_data(pts[:, 0], pts[:, 1])
        txt.set_text(f't = {res["t"][k]:.1f} s   max slot error '
                     f'{res["form_err"][k].max():.2f} m')
        return [cdot, sdots, tri, txt] + edots + trails

    anim = animation.FuncAnimation(fig, update, frames=n_frames, blit=False)
    anim.save(os.path.join(OUT_DIR, 'formation_animation.gif'),
              writer=animation.PillowWriter(fps=fps))
    plt.close(fig)
    print(f'Saved: formation_animation.gif ({n_frames} frames, {fps} fps)')


# ── Entry point ───────────────────────────────────────────────────────────────

if __name__ == '__main__':
    print('=' * 64)
    print('S028 — Cargo Escort Formation Simulation')
    print('=' * 64)
    runs = {c: run_simulation(c) for c in CASES}
    for c, r in runs.items():
        a, b = r['min_pair']
        print(f'\n--- Case {r["name"]} ---')
        print(f'  Carrier arrival: t = {r["t_arrive"]:.2f} s, final error '
              f'{r["final_err"]:.4f} m; waypoint switches at '
              + ', '.join(f'{s:.2f}' for s in r['switch_t']) + ' s')
        print(f'  RMS formation error: {r["rms_total"]:.4f} m  (per escort '
              + ', '.join(f'{v:.4f}' for v in r['rms_per']) + ' m)')
        print(f'  Max slot error:      {r["max_err"]:.4f} m')
        print(f'  Outside turn windows ({TURN_WINDOW:.0f} s after each switch): '
              f'RMS {r["rms_steady"]:.4f} m, max {r["max_steady"]:.4f} m; '
              f'max carrier heading deviation from leg direction '
              f'{r["hdg_dev_steady"]:.1f} deg')
        print(f'  Cruise only (also carrier speed >= {CRUISE_MIN_V} m/s): '
              f'RMS {r["rms_cruise"]:.4f} m; max heading deviation '
              f'{r["hdg_dev_cruise"]:.1f} deg')
        print(f'  Min separation, all pairs: {r["min_sep"]:.4f} m '
              f'({DRONE_NAMES[a]}–{DRONE_NAMES[b]}); escort pairs only: '
              f'{r["min_sep_escorts"]:.4f} m')
        print(f'  Steps with repulsion active: {r["rep_steps"]} of '
              f'{len(r["t"]) - 1}')
        print(f'  [{"PASS" if r["rms_total"] < TARGET_RMS else "FAIL"}] RMS '
              f'< {TARGET_RMS} m   [{"PASS" if r["min_sep"] >= D_SAFE else "FAIL"}]'
              f' min sep >= d_safe   [{"PASS" if r["t_arrive"] else "FAIL"}]'
              f' arrival')
    print('\n--- Generating plots ---')
    plot_trajectory_3d(runs['C'])
    plot_topdown_xy(runs)
    plot_formation_error(runs)
    plot_separation(runs['C'])
    plot_combined_metrics(runs)
    save_animation(runs['C'])
