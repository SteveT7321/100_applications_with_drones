"""
S061 Power Line Inspection
==========================
A drone follows a 100.55 m high-voltage cable (5-vertex 3D polyline, 3 support poles)
at a lateral standoff of 1.5 m and 0.5 m above it, at 1.0 m/s. A Frenet-frame
controller (along-track speed loop, cross-track PID, vertical PD) tracks the
nominal offset path from noisy GPS (sigma = 0.1 m); a repulsive potential field
pushes the drone away from each pole.

Four controller configurations are compared over 20 GPS-noise seeds (common
random numbers):
  A  card law      : finite-difference error derivatives on GPS, no drag feed-forward
  B  vel-deriv     : error derivatives from the true (noise-free) velocity, no feed-forward
  C  vel-deriv + FF: B plus drag / slope feed-forward (proposed)
  D  C without the pole potential field (ablation)

Usage:
    conda activate drones
    python src/04_industrial_agriculture/s061_power_line.py
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Circle
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401

# ── Parameters ────────────────────────────────────────────────────────────────
CABLE_VERTICES = np.array([
    [0.0,   0.0,  8.0],    # start attachment
    [25.0,  1.0, 10.0],    # pole 1 top
    [50.0, -1.0, 12.0],    # pole 2 top
    [75.0,  1.5, 10.5],    # pole 3 top
    [100.0, 0.0,  8.5],    # end attachment
])
POLE_XY     = CABLE_VERTICES[1:4, :2]
POLE_TOP    = CABLE_VERTICES[1:4, 2]     # pole height = cable attachment height
POLE_RADIUS = 0.3        # m
R_AVOID     = 2.0        # m   potential-field activation radius d0
K_REP       = 1.5        # repulsive gain (m^3/s^2)

D_LAT       = 1.5        # m   lateral standoff
DELTA_Z     = 0.5        # m   height above cable
V_INSP      = 1.0        # m/s inspection speed
GPS_SIGMA   = 0.1        # m   GPS noise (measurement only)
C_DRAG      = 0.5        # 1/s linear drag
DT          = 0.05       # s
T_LIMIT     = 400.0      # s   safety stop for the integration loop
V_MAX       = 5.0        # m/s airframe speed limit (added; card has none)

KP_T, KD_T       = 1.2, 0.4
KP_N, KD_N, KI_N = 2.0, 0.8, 0.1
KP_B, KD_B       = 2.5, 1.0

N_SEG       = 200        # cable arc-length segments
EPS_INSP    = 0.5        # m   inspection radius = D_LAT + EPS_INSP around a segment midpoint
DT_MIN_INS  = 0.5        # s   minimum dwell to count a segment inspected
DEV_TOL     = 0.3        # m   path-deviation requirement
S_END_MARGIN = 0.2       # m   mission ends when s >= S - margin (card)

N_SEEDS     = 20
SEEDS       = list(range(N_SEEDS))
ANIM_SEED   = 0
ANIM_STEP   = 20         # simulation steps per GIF frame (1 s)
SWEEP_SIGMA = [0.1, 0.25, 0.5, 1.0]   # m  GPS noise levels for the robustness sweep
SWEEP_SEEDS = list(range(10))
SWEEP_CFGS  = ['A', 'C', 'D']

CONFIGS = {
    'A': dict(label='A: card law (FD derivative, no FF)', deriv='fd', ff=False, rep=True),
    'B': dict(label='B: velocity derivative, no FF',      deriv='vel', ff=False, rep=True),
    'C': dict(label='C: velocity derivative + FF',        deriv='vel', ff=True, rep=True),
    'D': dict(label='D: C without potential field',       deriv='vel', ff=True, rep=False),
}
CFG_KEYS = list(CONFIGS)
CFG_COLORS = {'A': '#888888', 'B': '#E87722', 'C': '#1F77B4', 'D': '#9467BD'}
MAIN = 'C'

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), '..', '..',
                          'outputs', '04_industrial_agriculture', 's061_power_line')

Z_HAT = np.array([0.0, 0.0, 1.0])


# ── Geometry ──────────────────────────────────────────────────────────────────

def build_frames(vertices=CABLE_VERTICES):
    """Per-segment tangent T, horizontal normal N = T x z, start point and length."""
    P0 = vertices[:-1]
    D = vertices[1:] - vertices[:-1]
    L = np.linalg.norm(D, axis=1)
    T = D / L[:, None]
    N = np.cross(T, Z_HAT)
    N /= np.linalg.norm(N, axis=1)[:, None]
    S0 = np.concatenate([[0.0], np.cumsum(L)[:-1]])
    return P0, T, N, L, S0


P0, T_SEG, N_SEG_VEC, L_SEG, S0_SEG = build_frames()
S_TOTAL = float(L_SEG.sum())
SEG_LEN = S_TOTAL / N_SEG
MID_S = (np.arange(N_SEG) + 0.5) * SEG_LEN


def cable_point(s):
    """Point on the cable at arc length s (vectorised)."""
    s = np.atleast_1d(s)
    i = np.clip(np.searchsorted(S0_SEG, s, side='right') - 1, 0, len(L_SEG) - 1)
    return P0[i] + (s - S0_SEG[i])[:, None] * T_SEG[i]


MID_PTS = cable_point(MID_S)


def project(p):
    """Nearest-segment projection: (s, ref point, segment index)."""
    v = p[None, :] - P0
    t = np.clip(np.einsum('ij,ij->i', v, T_SEG), 0.0, L_SEG)
    ref = P0 + t[:, None] * T_SEG
    d = np.linalg.norm(p[None, :] - ref, axis=1)
    i = int(np.argmin(d))
    return S0_SEG[i] + t[i], ref[i], i


def nominal(s_ref, ref, i):
    return ref + D_LAT * N_SEG_VEC[i] + DELTA_Z * Z_HAT


def pole_clearance(p):
    """Distance from point p to the nearest pole cylinder surface (pole from z=0 to top)."""
    best = np.inf
    for (px, py), top in zip(POLE_XY, POLE_TOP):
        rho = np.hypot(p[0] - px, p[1] - py)
        dr = max(rho - POLE_RADIUS, 0.0)
        dz = max(p[2] - top, 0.0, -p[2])
        best = min(best, np.hypot(dr, dz))
    return best


def cable_clearance(p):
    v = p[None, :] - P0
    t = np.clip(np.einsum('ij,ij->i', v, T_SEG), 0.0, L_SEG)
    return float(np.min(np.linalg.norm(p[None, :] - (P0 + t[:, None] * T_SEG), axis=1)))


def repulsive_accel(p):
    acc = np.zeros(3)
    for px, py in POLE_XY:
        dx, dy = p[0] - px, p[1] - py
        d = np.hypot(dx, dy)
        if 1e-4 < d < R_AVOID:
            mag = K_REP * (1.0 / d - 1.0 / R_AVOID) / d ** 2
            acc[0] += mag * dx / d
            acc[1] += mag * dy / d
    return acc


# ── Simulation ────────────────────────────────────────────────────────────────

def simulate(cfg, seed, gps_sigma=GPS_SIGMA):
    """One inspection flight. GPS noise corrupts only the measured position."""
    rng = np.random.default_rng(seed)
    pos = CABLE_VERTICES[0] + D_LAT * N_SEG_VEC[0] + DELTA_Z * Z_HAT
    vel = T_SEG[0] * V_INSP
    i_N = 0.0
    prev_eN_m = prev_eB_m = None
    prev_sdot = V_INSP
    dwell = np.zeros(N_SEG)
    insp_r = D_LAT + EPS_INSP
    n_sat = 0

    log = {k: [] for k in ('t', 'pos', 'meas', 's', 'eN', 'eB', 'sdot', 'pole_clr',
                           'cable_clr', 'arep')}
    t = 0.0
    while True:
        meas = pos + rng.normal(0.0, gps_sigma, 3)
        s_m, ref_m, i_m = project(meas)
        T_hat, N_hat = T_SEG[i_m], N_SEG_VEC[i_m]
        p_nom_m = nominal(s_m, ref_m, i_m)
        diff_m = meas - p_nom_m
        eN_m = float(diff_m @ N_hat)
        eB_m = float(diff_m[2])
        sdot = float(vel @ T_hat)

        # truth-based errors for metrics
        s_t, ref_t, i_t = project(pos)
        diff_t = pos - nominal(s_t, ref_t, i_t)
        eN_t = float(diff_t @ N_SEG_VEC[i_t])
        eB_t = float(diff_t[2])

        # derivative terms
        if cfg['deriv'] == 'fd':
            deN = 0.0 if prev_eN_m is None else (eN_m - prev_eN_m) / DT
            deB = 0.0 if prev_eB_m is None else (eB_m - prev_eB_m) / DT
        else:
            deN = float(vel @ N_hat)
            deB = float(vel[2] - sdot * T_hat[2])
        sddot = (sdot - prev_sdot) / DT

        a_T = KP_T * (V_INSP - sdot) - KD_T * sddot
        i_N += eN_m * DT
        a_N = -(KP_N * eN_m + KD_N * deN + KI_N * i_N)
        a_B = -(KP_B * eB_m + KD_B * deB)
        a_cmd = a_T * T_hat + a_N * N_hat + a_B * Z_HAT
        if cfg['ff']:
            # cancel linear drag along the nominal velocity V_INSP * T (incl. cable slope)
            a_cmd = a_cmd + C_DRAG * V_INSP * T_hat
        a_rep = repulsive_accel(meas) if cfg['rep'] else np.zeros(3)
        a_cmd = a_cmd + a_rep

        # dwell accounting (truth)
        dmid = np.linalg.norm(MID_PTS - pos[None, :], axis=1)
        dwell[dmid <= insp_r] += DT

        pc = pole_clearance(pos)
        cc = cable_clearance(pos)
        assert pc > 0.0, 'drone hit a pole'
        assert cc > 0.0, 'drone hit the cable'
        assert pos[2] > 0.0, 'drone below ground'

        log['t'].append(t)
        log['pos'].append(pos.copy())
        log['meas'].append(meas)
        log['s'].append(s_t)
        log['eN'].append(eN_t)
        log['eB'].append(eB_t)
        log['sdot'].append(sdot)
        log['pole_clr'].append(pc)
        log['cable_clr'].append(cc)
        log['arep'].append(float(np.linalg.norm(a_rep)))

        if s_m >= S_TOTAL - S_END_MARGIN or t >= T_LIMIT:
            break

        new_vel = vel + (a_cmd - C_DRAG * vel) * DT
        sp = np.linalg.norm(new_vel)
        if sp > V_MAX:                     # airframe speed limit
            new_vel *= V_MAX / sp
            n_sat += 1
        step = new_vel * DT
        assert np.linalg.norm(new_vel) <= V_MAX + 1e-9, 'speed above V_MAX'
        pos = pos + step
        vel = new_vel
        prev_eN_m, prev_eB_m, prev_sdot = eN_m, eB_m, sdot
        t += DT

    for k in log:
        log[k] = np.array(log[k])
    log['dwell'] = dwell
    log['n_sat'] = n_sat
    return log


def metrics(log):
    eN, eB = log['eN'], log['eB']
    dev = np.sqrt(eN ** 2 + eB ** 2)
    insp = log['dwell'] >= DT_MIN_INS
    ss = (log['s'] > 10.0) & (log['s'] < S_TOTAL - 10.0)
    return {
        'time': float(log['t'][-1]),
        'coverage': 100.0 * insp.mean(),
        'rms': float(np.sqrt(np.mean(eN ** 2 + eB ** 2))),
        'max_eN': float(np.max(np.abs(eN))),
        'max_eB': float(np.max(np.abs(eB))),
        'max_dev': float(dev.max()),
        'frac_dev_ok': 100.0 * float(np.mean(dev <= DEV_TOL)),
        'mean_speed': float(np.mean(log['sdot'][ss])),
        'min_pole_clr': float(log['pole_clr'].min()),
        'min_dwell': float(log['dwell'].min()),
        'rep_steps': int(np.sum(log['arep'] > 0)),
        'n_sat': int(log['n_sat']),
    }


def run_simulation():
    results = {k: [] for k in CFG_KEYS}
    logs = {}
    for k in CFG_KEYS:
        for seed in SEEDS:
            lg = simulate(CONFIGS[k], seed)
            results[k].append(metrics(lg))
            if seed == ANIM_SEED:
                logs[k] = lg
    sweep = {k: {sg: [metrics(simulate(CONFIGS[k], seed, sg)) for seed in SWEEP_SEEDS]
                 for sg in SWEEP_SIGMA} for k in SWEEP_CFGS}
    return results, logs, sweep


# ── Plotting ──────────────────────────────────────────────────────────────────

def _draw_poles_3d(ax):
    th = np.linspace(0, 2 * np.pi, 24)
    for (px, py), top in zip(POLE_XY, POLE_TOP):
        zz = np.linspace(0, top, 2)
        TH, ZZ = np.meshgrid(th, zz)
        ax.plot_surface(px + POLE_RADIUS * np.cos(TH), py + POLE_RADIUS * np.sin(TH), ZZ,
                        color='grey', alpha=0.6, linewidth=0)


def plot_trajectory_3d(log, out_dir):
    fig = plt.figure(figsize=(13, 6))
    ax = fig.add_subplot(111, projection='3d')
    _draw_poles_3d(ax)
    ax.plot(*CABLE_VERTICES.T, 'k-', lw=2, label='Cable')
    s_line = np.linspace(0, S_TOTAL, 400)
    cp = cable_point(s_line)
    i_line = np.clip(np.searchsorted(S0_SEG, s_line, side='right') - 1, 0, 3)
    nom = cp + D_LAT * N_SEG_VEC[i_line] + DELTA_Z * Z_HAT
    ax.plot(*nom.T, color='green', lw=1.2, ls='--', label='Nominal offset path')
    P = log['pos']
    dev = np.sqrt(log['eN'] ** 2 + log['eB'] ** 2)
    sc = ax.scatter(P[::4, 0], P[::4, 1], P[::4, 2], c=dev[::4], cmap='coolwarm',
                    vmin=0, vmax=DEV_TOL, s=4, label='Drone (colour = deviation)')
    ax.scatter(*P[0], color='green', s=50, label='Start')
    ax.scatter(*P[-1], color='red', s=50, label='End')
    fig.colorbar(sc, ax=ax, shrink=0.6, pad=0.12, label='|deviation| (m)')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_zlabel('z (m)')
    ax.set_ylim(-6, 6)
    ax.set_zlim(0, 14)
    ax.set_box_aspect((4, 1, 0.8))
    ax.view_init(elev=22, azim=-70)
    ax.set_title(f'S061 Power Line Inspection — config {MAIN}, seed {ANIM_SEED}')
    ax.legend(loc='upper left', fontsize=8)
    path = os.path.join(out_dir, 'trajectory_3d.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def _pole_s():
    return S0_SEG[1:4]


def plot_error_timeseries(logs, out_dir):
    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    for ax, key in zip(axes[:2], ['A', MAIN]):
        lg = logs[key]
        ax.axhspan(-DEV_TOL, DEV_TOL, color='yellow', alpha=0.25, label='±0.3 m tolerance')
        ax.plot(lg['s'], lg['eN'], color='tab:red', lw=0.8, label='Cross-track $e_N$')
        ax.plot(lg['s'], lg['eB'], color='tab:blue', lw=0.8, label='Vertical $e_B$')
        for sp in _pole_s():
            ax.axvspan(sp - 1.5, sp + 1.5, color='grey', alpha=0.25)
        ax.set_ylabel('error (m)')
        ax.set_ylim(-0.6, 0.6)
        ax.set_title(CONFIGS[key]['label'] + f' — seed {ANIM_SEED}', fontsize=10)
        ax.legend(loc='upper right', fontsize=8, ncol=3)
        ax.grid(alpha=0.3)
    ax = axes[2]
    for key in CFG_KEYS[:3]:
        lg = logs[key]
        ax.plot(lg['s'], lg['sdot'], color=CFG_COLORS[key], lw=0.8, label=CONFIGS[key]['label'])
    ax.axhline(V_INSP, color='k', ls='--', lw=1, label='$v_{insp}$ = 1.0 m/s')
    for sp in _pole_s():
        ax.axvspan(sp - 1.5, sp + 1.5, color='grey', alpha=0.25)
    ax.set_ylabel('along-track speed (m/s)')
    ax.set_xlabel('arc length s (m)   (grey bands = poles)')
    ax.legend(loc='center right', fontsize=8)
    ax.grid(alpha=0.3)
    fig.suptitle('S061 Frenet tracking errors (true position) and along-track speed')
    plt.tight_layout()
    path = os.path.join(out_dir, 'error_timeseries.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_coverage_dwell(log, out_dir):
    fig, ax = plt.subplots(figsize=(12, 4))
    ok = log['dwell'] >= DT_MIN_INS
    ax.bar(MID_S, log['dwell'], width=SEG_LEN * 0.9,
           color=np.where(ok, 'teal', 'coral'))
    ax.axhline(DT_MIN_INS, color='k', ls='--', lw=1, label=r'$\Delta t_{min}$ = 0.5 s')
    for sp in _pole_s():
        ax.axvline(sp, color='grey', ls='--', lw=1)
    ax.set_xlabel('arc length s (m)   (dashed grey = poles)')
    ax.set_ylabel('dwell time (s)')
    ax.set_title(f'S061 Dwell per 0.5 m cable segment (within {D_LAT + EPS_INSP:.1f} m of '
                 f'midpoint) — config {MAIN}, seed {ANIM_SEED}: coverage '
                 f'{100 * ok.mean():.1f}%')
    ax.legend()
    path = os.path.join(out_dir, 'coverage_dwell.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_config_comparison(results, out_dir):
    fields = [('rms', 'RMS deviation (m)'), ('max_dev', 'max deviation (m)'),
              ('time', 'task time (s)'), ('mean_speed', 'mean along-track speed (m/s)')]
    fig, axes = plt.subplots(1, 4, figsize=(17, 4.5))
    for ax, (f, lab) in zip(axes, fields):
        data = [[r[f] for r in results[k]] for k in CFG_KEYS]
        bp = ax.boxplot(data, patch_artist=True, widths=0.6)
        for patch, k in zip(bp['boxes'], CFG_KEYS):
            patch.set_facecolor(CFG_COLORS[k])
            patch.set_alpha(0.7)
        ax.set_xticks(range(1, len(CFG_KEYS) + 1))
        ax.set_xticklabels(CFG_KEYS)
        ax.set_title(lab)
        ax.grid(alpha=0.3)
    axes[1].axhline(DEV_TOL, color='red', ls='--', lw=1, label='0.3 m requirement')
    axes[1].legend(fontsize=8)
    axes[3].axhline(V_INSP, color='k', ls='--', lw=1)
    txt = '   '.join(f"{k} = {CONFIGS[k]['label'][3:]}" for k in CFG_KEYS)
    fig.suptitle(f'S061 Controller comparison over {N_SEEDS} GPS-noise seeds (CRN)\n{txt}',
                 fontsize=10)
    plt.tight_layout()
    path = os.path.join(out_dir, 'config_comparison.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_gps_sweep(sweep, out_dir):
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.2))
    fields = [('rms', 'RMS deviation (m)'), ('max_dev', 'max deviation (m)'),
              ('coverage', 'coverage (%)')]
    for ax, (f, lab) in zip(axes, fields):
        for k in SWEEP_CFGS:
            m = [np.mean([r[f] for r in sweep[k][sg]]) for sg in SWEEP_SIGMA]
            sd = [np.std([r[f] for r in sweep[k][sg]]) for sg in SWEEP_SIGMA]
            ax.errorbar(SWEEP_SIGMA, m, yerr=sd, marker='o', capsize=3,
                        color=CFG_COLORS[k], label=CONFIGS[k]['label'])
        ax.set_xlabel(r'GPS noise $\sigma_{GPS}$ (m)')
        ax.set_title(lab)
        ax.grid(alpha=0.3)
    axes[1].axhline(DEV_TOL, color='red', ls='--', lw=1, label='0.3 m requirement')
    axes[2].axhline(98.0, color='red', ls='--', lw=1, label='98 % requirement')
    axes[1].legend(fontsize=8)
    axes[2].legend(fontsize=8)
    fig.suptitle(f'S061 GPS-noise robustness (mean ± std over {len(SWEEP_SEEDS)} seeds, CRN)')
    plt.tight_layout()
    path = os.path.join(out_dir, 'gps_sweep.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def save_animation(log, out_dir):
    P, M = log['pos'], log['meas']
    n = len(P)
    n_frames = int(np.ceil((n - 1) / ANIM_STEP)) + 1
    fig, ax = plt.subplots(figsize=(12, 3.2))
    ax.plot(CABLE_VERTICES[:, 0], CABLE_VERTICES[:, 1], 'k-', lw=2, label='Cable')
    for px, py in POLE_XY:
        ax.add_patch(Circle((px, py), R_AVOID, fc='grey', alpha=0.15, ec='grey', ls='--'))
        ax.add_patch(Circle((px, py), POLE_RADIUS, fc='grey', ec='k'))
    ax.set_xlim(-3, 103)
    ax.set_ylim(-7.0, 3.5)
    ax.set_aspect('equal')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    trail, = ax.plot([], [], color='tab:red', lw=1)
    gps = ax.scatter([], [], s=6, color='orange', label='GPS fixes (last 1 s)')
    dot, = ax.plot([], [], 'o', color='red', ms=7, label='Drone')
    offs, = ax.plot([], [], ':', color='tab:red', lw=1)
    ax.legend(loc='lower left', fontsize=8, ncol=3)

    def update(fi):
        i = min(fi * ANIM_STEP, n - 1)
        trail.set_data(P[:i + 1, 0], P[:i + 1, 1])
        lo = max(0, i - ANIM_STEP)
        gps.set_offsets(M[lo:i + 1, :2])
        dot.set_data([P[i, 0]], [P[i, 1]])
        cp = cable_point(log['s'][i])[0]
        offs.set_data([cp[0], P[i, 0]], [cp[1], P[i, 1]])
        ax.set_title(f"S061 top view — t = {log['t'][i]:.0f} s, s = {log['s'][i]:.1f} m, "
                     f"e_N = {log['eN'][i]:+.2f} m (config {MAIN}, seed {ANIM_SEED})")
        return trail, gps, dot, offs

    anim = animation.FuncAnimation(fig, update, frames=n_frames, interval=50, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    anim.save(path, writer='pillow', fps=20)
    plt.close(fig)
    print(f'Saved: {path}  ({n_frames} frames)')


def _ms(vals, fmt='.3f'):
    v = np.asarray(vals, dtype=float)
    return f'{v.mean():{fmt}} ± {v.std():{fmt}}'


if __name__ == '__main__':
    results, logs, sweep = run_simulation()

    print('\n=== S061 Power Line Inspection — Results ===')
    print(f'  Cable arc length S = {S_TOTAL:.2f} m, {N_SEG} segments of {SEG_LEN:.3f} m')
    v_ss = KP_T * V_INSP / (KP_T + C_DRAG)
    print(f'  Steady along-track speed without feed-forward: Kp*v/(Kp+c) = {v_ss:.3f} m/s '
          f'-> ideal time {S_TOTAL / v_ss:.1f} s; with FF {S_TOTAL / V_INSP:.1f} s')
    a_rep_nom = K_REP * (1.0 / D_LAT - 1.0 / R_AVOID) / D_LAT ** 2
    print(f'  Repulsion at the nominal 1.5 m pole distance: {a_rep_nom:.3f} m/s^2 '
          f'(static cross-track offset ~ {a_rep_nom / KP_N:.3f} m before the integrator acts)')
    print(f'  Drone height above pole top at nominal: {DELTA_Z:.1f} m '
          f'(horizontal pole clearance {D_LAT - POLE_RADIUS:.1f} m)')
    print(f'  Monte Carlo: {N_SEEDS} seeds, common random numbers across configs')
    for k in CFG_KEYS:
        R = results[k]
        g = lambda f: [r[f] for r in R]          # noqa: E731
        print(f"  [{CONFIGS[k]['label']}]")
        print(f"    Task time          : {_ms(g('time'), '.1f')} s")
        print(f"    Mean speed (10 m from ends excluded): {_ms(g('mean_speed'))} m/s")
        print(f"    Coverage           : {_ms(g('coverage'), '.1f')} %  "
              f"(min dwell {np.min(g('min_dwell')):.2f} s)")
        print(f"    RMS deviation      : {_ms(g('rms'))} m")
        print(f"    Max |e_N|          : {_ms(g('max_eN'))} m")
        print(f"    Max |e_B|          : {_ms(g('max_eB'))} m")
        print(f"    Max deviation      : {_ms(g('max_dev'))} m  "
              f"(runs with max <= 0.3 m: {sum(d <= DEV_TOL for d in g('max_dev'))}/{N_SEEDS})")
        print(f"    Time within 0.3 m  : {_ms(g('frac_dev_ok'), '.2f')} %")
        print(f"    Min pole clearance : {np.min(g('min_pole_clr')):.3f} m "
              f"(mean {np.mean(g('min_pole_clr')):.3f} m)")
        print(f"    Steps with repulsion active: {_ms(g('rep_steps'), '.0f')}")
        print(f"    Steps at the 5 m/s speed limit: {_ms(g('n_sat'), '.0f')}")
    rmsA = np.array([r['rms'] for r in results['A']])
    rmsB = np.array([r['rms'] for r in results['B']])
    rmsC = np.array([r['rms'] for r in results['C']])
    rmsD = np.array([r['rms'] for r in results['D']])
    print(f'  Per-seed RMS wins: B < A in {int(np.sum(rmsB < rmsA))}/{N_SEEDS}; '
          f'C < B in {int(np.sum(rmsC < rmsB))}/{N_SEEDS}; '
          f'C < D in {int(np.sum(rmsC < rmsD))}/{N_SEEDS}')
    for k in ('C', 'D'):
        lg = logs[k]
        near = [float(np.max(np.abs(lg['eN'][np.abs(lg['s'] - sp) <= 3.0]))) for sp in S0_SEG[1:4]]
        print(f"  Seed {ANIM_SEED} config {k}: max |e_N| within 3 m of poles 1/2/3: "
              + ' / '.join(f'{v:.3f}' for v in near) + ' m')
    mC = metrics(logs[MAIN])
    print(f'  Seed {ANIM_SEED} config {MAIN}: time {mC["time"]:.1f} s, coverage '
          f'{mC["coverage"]:.1f} %, RMS {mC["rms"]:.3f} m, max|e_N| {mC["max_eN"]:.3f} m, '
          f'max|e_B| {mC["max_eB"]:.3f} m')

    print(f'  GPS sweep ({len(SWEEP_SEEDS)} seeds each): sigma -> RMS / max dev / '
          f'runs with max dev <= 0.3 m / coverage / time')
    for k in SWEEP_CFGS:
        for sg in SWEEP_SIGMA:
            R = sweep[k][sg]
            g = lambda f: [r[f] for r in R]          # noqa: E731
            print(f"    {k} sigma={sg:.2f} m: RMS {_ms(g('rms'))} m, max {_ms(g('max_dev'))} m, "
                  f"{sum(d <= DEV_TOL for d in g('max_dev'))}/{len(R)} ok, "
                  f"coverage {_ms(g('coverage'), '.1f')} %, time {_ms(g('time'), '.1f')} s, "
                  f"speed-limit steps {_ms(g('n_sat'), '.0f')}")

    out = os.path.normpath(OUTPUT_DIR)
    os.makedirs(out, exist_ok=True)
    plot_trajectory_3d(logs[MAIN], out)
    plot_error_timeseries(logs, out)
    plot_coverage_dwell(logs[MAIN], out)
    plot_config_comparison(results, out)
    plot_gps_sweep(sweep, out)
    save_animation(logs[MAIN], out)
