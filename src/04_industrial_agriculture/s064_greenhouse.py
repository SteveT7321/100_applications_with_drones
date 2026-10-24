"""
S064 Greenhouse Interior Precision Flight
=========================================
A drone flies a serpentine route through the five aisles of a GPS-denied greenhouse
(40 x 10 x 3 m, plant rows 1.5 m tall at y = 2, 4, 6, 8, 10 m with 2 m headlands at
both ends). Its lateral position comes from dead reckoning (drifting) fused with two
side-looking ultrasonic range sensors in a complementary filter.

Three estimators/safety set-ups are compared on the same random numbers (30 seeds):
  DR only        : dead reckoning, no ultrasonic sensing at all
  DR + repulsion : dead reckoning for navigation, ultrasonic ranges only for the
                   repulsive safety layer
  Fusion (card)  : complementary filter (alpha = 0.85) + repulsive safety layer
plus a sweep of the filter gain alpha.

Usage:
    conda activate drones
    python src/04_industrial_agriculture/s064_greenhouse.py
"""

import os
import math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Rectangle
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# ── Parameters ────────────────────────────────────────────────────────────────
GH_X, GH_Y, GH_Z = 40.0, 10.0, 3.0
ROW_Y      = [2.0, 4.0, 6.0, 8.0, 10.0]
ROW_THICK  = 0.2
ROW_H      = 1.5
ROW_X0, ROW_X1 = 2.0, 38.0       # rows end 2 m before each gable wall (headlands, added)
DRONE_R    = 0.15                # m  half-size of the 0.3 x 0.3 m drone

Z_CRUISE   = 1.0
V_FWD      = 1.0                 # m/s
D_SAFE     = 0.25                # m  clearance from the drone edge
D_WP       = 0.5                 # m  waypoint capture radius
SIGMA_DRIFT = 0.02               # m/s/sqrt(s)
SIGMA_RANGE = 0.05               # m
RANGE_MIN, RANGE_MAX = 0.1, 2.0  # m
GATE       = 0.5                 # m  reject wall fixes this far from the DR prediction (added)
ALPHA      = 0.85
KP_LAT     = 0.8                 # 1/s
KP_ALT     = 1.2                 # 1/s
KP_X_TURN  = 0.8                 # 1/s  x hold in the headland (added)
V_LAT_MAX  = 1.0                 # m/s  lateral speed limit (added)
K_REP      = 1.5                 # m/s
DT         = 0.05
T_MAX      = 300.0
X_END      = [1.0, 39.0]         # headland centre lines
START      = np.array([1.0, 1.0, 1.0])   # card (0, 1, 1) sits on the gable wall

AISLE_Y = [(0.0 + ROW_Y[0] - ROW_THICK / 2) / 2] + \
          [(ROW_Y[k] + ROW_Y[k + 1]) / 2 for k in range(4)]   # 0.95, 3, 5, 7, 9

STRATS = ['dr', 'dr_rep', 'fusion']
LABELS = {'dr': 'DR only', 'dr_rep': 'DR + repulsion', 'fusion': 'Fusion α=0.85 (card)'}
COLORS = {'dr': '#888888', 'dr_rep': '#E87722', 'fusion': '#1F77B4'}
N_SEEDS = 30
SEEDS = list(range(N_SEEDS))
ALPHAS = [0.5, 0.7, 0.85, 0.95, 0.99]
ALPHA_SEEDS = list(range(10))
PLOT_SEED = 0
ANIM_STEP = 20

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), '..', '..', 'outputs',
                          '04_industrial_agriculture', 's064_greenhouse')


# ── Geometry ──────────────────────────────────────────────────────────────────

def waypoints():
    """Serpentine route: leg ends alternate x = 39 / x = 1; headland moves between aisles."""
    wps, ends = [], []
    for k, yc in enumerate(AISLE_Y):
        x_far = X_END[1] if k % 2 == 0 else X_END[0]
        if k > 0:
            x_here = X_END[0] if k % 2 == 0 else X_END[1]
            wps.append((x_here, yc, 'turn'))
        wps.append((x_far, yc, 'leg'))
        ends.append(len(wps) - 1)
    return wps, ends


WPS, ROW_END_IDX = waypoints()


def rows_present(x):
    return ROW_X0 <= x <= ROW_X1


def side_faces(x):
    """Faces facing +y (left obstacles) and -y (right obstacles) at abscissa x."""
    left = [0.0]
    right = [GH_Y]
    if rows_present(x):
        left += [r + ROW_THICK / 2 for r in ROW_Y]
        right += [r - ROW_THICK / 2 for r in ROW_Y]
    return left, right


def lateral_ranges(x, y):
    """True centre-to-face distances to the nearest obstacle on each side."""
    left, right = side_faces(x)
    dl = y - max(f for f in left if f <= y + 1e-12)
    dr = min(f for f in right if f >= y - 1e-12) - y
    return dl, dr


def clearance(x, y):
    """Distance from the drone edge (radius 0.15 m) to the nearest row box or wall."""
    best = min(x, GH_X - x, y, GH_Y - y)
    for r in ROW_Y:
        dx = max(ROW_X0 - x, 0.0, x - ROW_X1)
        dy = max(r - ROW_THICK / 2 - y, 0.0, y - r - ROW_THICK / 2)
        best = min(best, math.hypot(dx, dy))
    return best - DRONE_R


# ── Simulation ────────────────────────────────────────────────────────────────

def noise(seed, n):
    rng = np.random.default_rng(seed)
    eta_true = rng.normal(0, SIGMA_DRIFT * math.sqrt(DT), (n, 3))
    eta_est = rng.normal(0, SIGMA_DRIFT * math.sqrt(DT), n)
    rng_noise = rng.normal(0, SIGMA_RANGE, (n, 2))
    return eta_true, eta_est, rng_noise


def simulate(strategy, seed, alpha=ALPHA, keep_log=False):
    n = int(round(T_MAX / DT))
    eta_t, eta_e, rn = noise(seed, n)
    use_fusion = strategy == 'fusion'
    use_rep = strategy in ('dr_rep', 'fusion')
    x, y, z = START
    y_hat = y
    y_pure = y            # parallel pure dead-reckoning integrator (logging only)
    wp = 0
    reached = []
    err, clr = [], []
    viol = 0
    crash = False
    n_fix = 0
    lg = {k: [] for k in ('t', 'x', 'y', 'z', 'yhat', 'ydr', 'ypure', 'ywall', 'dl', 'dr')} \
        if keep_log else None
    k = 0
    for k in range(n):
        t = k * DT
        wx, wy, kind = WPS[wp]
        # ── sensing ──
        dl_t, dr_t = lateral_ranges(x, y)
        zl, zr = dl_t + rn[k, 0], dr_t + rn[k, 1]
        zl = zl if RANGE_MIN <= zl <= RANGE_MAX else None
        zr = zr if RANGE_MIN <= zr <= RANGE_MAX else None
        # ── commands (use the estimate; x and z are taken as known, as in the card) ──
        if kind == 'leg':
            dx = wx - x
            vx = math.copysign(V_FWD, dx) if abs(dx) > D_WP else KP_X_TURN * dx
        else:
            vx = KP_X_TURN * (wx - x)
        vy = -KP_LAT * (y_hat - wy)
        vy = max(-V_LAT_MAX, min(V_LAT_MAX, vy))
        if use_rep:                       # repulsion from measured ranges (edge clearance)
            if zl is not None and zl - DRONE_R < D_SAFE:
                vy += K_REP * (D_SAFE - (zl - DRONE_R)) / D_SAFE
            if zr is not None and zr - DRONE_R < D_SAFE:
                vy -= K_REP * (D_SAFE - (zr - DRONE_R)) / D_SAFE
        vz = KP_ALT * (Z_CRUISE - z)
        # ── truth propagation (unmeasured air-current drift, card) ──
        x += vx * DT + eta_t[k, 0]
        y += vy * DT + eta_t[k, 1]
        z += vz * DT + eta_t[k, 2]
        assert 0.2 < z < GH_Z - 0.2
        # ── estimator: DR prediction, then complementary correction ──
        y_dr = y_hat + vy * DT + eta_e[k]
        y_pure = y_pure + vy * DT + eta_e[k]
        y_wall = None
        if use_fusion:
            left, right = side_faces(x)
            cands = []
            if zl is not None:
                lf = max((f for f in left if f <= y_dr + 1e-9), default=None)
                if lf is not None:
                    cands.append(lf + zl)
            if zr is not None:
                rf = min((f for f in right if f >= y_dr - 1e-9), default=None)
                if rf is not None:
                    cands.append(rf - zr)
            cands = [c for c in cands if abs(c - y_dr) < GATE]
            if cands:
                y_wall = sum(cands) / len(cands)
                n_fix += 1
        y_hat = alpha * y_dr + (1 - alpha) * y_wall if y_wall is not None else y_dr

        c = clearance(x, y)
        err.append(abs(y_hat - y))
        clr.append(c)
        if c < D_SAFE:
            viol += 1
        if keep_log:
            for key, val in (('t', t), ('x', x), ('y', y), ('z', z), ('yhat', y_hat),
                             ('ydr', y_dr), ('ypure', y_pure), ('ywall', np.nan if y_wall is None else y_wall),
                             ('dl', dl_t), ('dr', dr_t)):
                lg[key].append(val)
        if c <= 0.0:
            crash = True
            break
        if math.hypot(x - wx, y_hat - wy) < D_WP:
            if wp in ROW_END_IDX:
                reached.append(t + DT)
            wp += 1
            if wp == len(WPS):
                break
    err = np.array(err)
    res = {
        'row_ends': len(reached), 'reach_t': reached, 'time': (k + 1) * DT,
        'complete': wp == len(WPS) and not crash, 'crash': crash,
        'mean_err': float(err.mean()), 'max_err': float(err.max()),
        'min_clear': float(np.min(clr)), 'viol': viol, 'fix_frac': n_fix / (k + 1),
    }
    if keep_log:
        for key in lg:
            lg[key] = np.array(lg[key])
        res['log'] = lg
    return res


def run_simulation():
    mc = {s: [simulate(s, sd, keep_log=(sd == PLOT_SEED)) for sd in SEEDS] for s in STRATS}
    asweep = {a: [simulate('fusion', sd, alpha=a) for sd in ALPHA_SEEDS] for a in ALPHAS}
    return mc, asweep


# ── Plotting ──────────────────────────────────────────────────────────────────

def plot_3d_trajectory(mc, out_dir):
    lg = mc['fusion'][PLOT_SEED]['log']
    fig = plt.figure(figsize=(14, 6))
    ax = fig.add_subplot(111, projection='3d')
    for r in ROW_Y:
        for yy in (r - ROW_THICK / 2, r + ROW_THICK / 2):
            if yy > GH_Y:
                continue
            verts = [[(ROW_X0, yy, 0), (ROW_X1, yy, 0), (ROW_X1, yy, ROW_H), (ROW_X0, yy, ROW_H)]]
            ax.add_collection3d(Poly3DCollection(verts, alpha=0.18, facecolor='grey',
                                                 edgecolor='grey'))
    ax.plot(lg['x'], lg['y'], lg['z'], color='tab:blue', lw=1.2, label='true path')
    ax.plot(lg['x'], lg['yhat'], lg['z'], color='red', lw=0.9, ls='--', label='estimated path')
    ax.scatter(*START, color='green', s=60, label='start')
    ax.scatter(lg['x'][-1], lg['y'][-1], lg['z'][-1], color='red', marker='x', s=60, label='end')
    ax.set_xlim(0, GH_X)
    ax.set_ylim(0, GH_Y)
    ax.set_zlim(0, GH_Z)
    ax.set_box_aspect((4, 1.2, 0.6))
    ax.set_zticks([0, 1.5, 3.0])
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_zlabel('z (m)')
    ax.view_init(elev=35, azim=-75)
    ax.set_title(f'S064 Greenhouse serpentine flight — fusion, seed {PLOT_SEED}')
    ax.legend(loc='upper left', fontsize=8)
    path = os.path.join(out_dir, 'trajectory_3d.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_analysis(mc, out_dir):
    lf = mc['fusion'][PLOT_SEED]['log']
    ld = mc['dr'][PLOT_SEED]['log']
    fig, axes = plt.subplots(2, 2, figsize=(15, 8.5))
    ax = axes[0, 0]
    for lg, lab, c in ((lf, 'fusion', 'tab:blue'), (ld, 'DR only', 'grey')):
        ax.plot(lg['t'], lg['y'] - lg['yhat'], color=c, lw=0.9, label=f'{lab}: true − estimate')
    ax.axhline(0, color='k', lw=0.6)
    ax.set_ylabel('lateral estimation error (m)')
    ax.set_title('(a) Estimation error y − ŷ')
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)
    ax = axes[0, 1]
    ax.plot(lf['t'], lf['ywall'] - lf['y'], '.', color='c', ms=1.0, label='wall-derived − true')
    ax.plot(lf['t'], lf['ypure'] - lf['y'], color='orange', lw=1.0,
            label='pure dead reckoning − true (same flight)')
    ax.plot(lf['t'], lf['yhat'] - lf['y'], color='red', lw=0.8, label='fused − true')
    ax.axhline(0, color='k', lw=0.6)
    ax.set_xlabel('time (s)')
    ax.set_ylabel('estimate − true y (m)')
    ax.set_title('(b) Filter components (fusion flight): DR drifts, fused stays bounded')
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)
    ax = axes[1, 0]
    for lg, lab, c in ((ld, 'DR only', 'grey'), (lf, 'fusion', 'tab:blue')):
        clr = np.array([clearance(a, b) for a, b in zip(lg['x'], lg['y'])])
        ax.plot(lg['t'], clr, color=c, lw=0.8, label=lab)
    ax.axhline(D_SAFE, color='k', ls='--', lw=1, label=f'd_safe = {D_SAFE} m')
    ax.set_ylim(0, 1.0)
    ax.set_xlabel('time (s)')
    ax.set_ylabel('edge clearance (m)')
    ax.set_title('(c) Clearance to the nearest row or wall')
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)
    ax = axes[1, 1]
    for lg, lab, c in ((ld, 'DR only', 'grey'), (lf, 'fusion', 'tab:blue')):
        e = np.abs(lg['yhat'] - lg['y'])
        ax.plot(lg['t'], np.cumsum(e) / np.arange(1, len(e) + 1), color=c, lw=1.4,
                label=f'{lab}: cumulative mean')
        ax.plot(lg['t'], e, color=c, lw=0.5, alpha=0.4)
    ax.set_yscale('log')
    ax.set_xlabel('time (s)')
    ax.set_ylabel('|ŷ − y| (m, log)')
    ax.set_title('(d) Lateral error, instantaneous (thin) and cumulative mean')
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)
    axes[0, 0].set_xlabel('time (s)')
    fig.suptitle(f'S064 Analysis, seed {PLOT_SEED}')
    plt.tight_layout()
    path = os.path.join(out_dir, 'analysis.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_comparison(mc, asweep, out_dir):
    fig, axes = plt.subplots(1, 4, figsize=(19, 4.5))
    fields = [('mean_err', 'mean |ŷ − y| (m)', True), ('min_clear', 'minimum edge clearance (m)', False),
              ('viol', f'steps with clearance < {D_SAFE} m', False)]
    for ax, (f, lab, logy) in zip(axes[:3], fields):
        data = [[r[f] for r in mc[s]] for s in STRATS]
        bp = ax.boxplot(data, patch_artist=True, widths=0.6)
        for patch, s in zip(bp['boxes'], STRATS):
            patch.set_facecolor(COLORS[s])
            patch.set_alpha(0.7)
        ax.set_xticks([1, 2, 3])
        ax.set_xticklabels([LABELS[s] for s in STRATS], fontsize=8)
        if logy:
            ax.set_yscale('log')
        ax.set_title(lab)
        ax.grid(alpha=0.3)
    axes[1].axhline(D_SAFE, color='k', ls='--', lw=1)
    axes[1].axhline(0.0, color='red', ls=':', lw=1)
    ax = axes[3]
    m = [np.mean([r['mean_err'] for r in asweep[a]]) for a in ALPHAS]
    mx = [np.mean([r['max_err'] for r in asweep[a]]) for a in ALPHAS]
    ax.plot(ALPHAS, m, 'o-', color='tab:blue', label='mean |ŷ − y|')
    ax.plot(ALPHAS, mx, 's--', color='tab:blue', label='max |ŷ − y|')
    ax.set_yscale('log')
    ax.set_xlabel('complementary filter α (DR weight)')
    ax.set_title(f'α sweep ({len(ALPHA_SEEDS)} seeds, mean)')
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)
    fig.suptitle(f'S064 Estimator comparison over {N_SEEDS} seeds (common random numbers)')
    plt.tight_layout()
    path = os.path.join(out_dir, 'strategy_comparison.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def save_animation(mc, out_dir):
    lg = mc['fusion'][PLOT_SEED]['log']
    n = len(lg['t'])
    n_frames = int(np.ceil((n - 1) / ANIM_STEP)) + 1
    fig, ax = plt.subplots(figsize=(12, 3.8))
    for r in ROW_Y:
        ax.add_patch(Rectangle((ROW_X0, r - ROW_THICK / 2), ROW_X1 - ROW_X0, ROW_THICK,
                               color='grey', alpha=0.45))
    ax.add_patch(Rectangle((0, 0), GH_X, GH_Y, fill=False, ec='k', lw=1.5))
    ax.set_xlim(-0.5, GH_X + 0.5)
    ax.set_ylim(-0.5, GH_Y + 0.5)
    ax.set_aspect('equal')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    tr, = ax.plot([], [], color='tab:blue', lw=1.0, label='true path')
    te, = ax.plot([], [], color='red', ls='--', lw=0.8, label='estimate')
    dot, = ax.plot([], [], 'o', color='tab:blue', ms=6)
    est, = ax.plot([], [], 's', color='red', ms=4)
    bl, = ax.plot([], [], color='c', lw=1.5)
    br, = ax.plot([], [], color='c', lw=1.5, label='ultrasonic beams')
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.2), ncol=3, fontsize=8)
    fig.subplots_adjust(bottom=0.3, top=0.88)

    def update(fi):
        i = min(fi * ANIM_STEP, n - 1)
        tr.set_data(lg['x'][:i + 1], lg['y'][:i + 1])
        te.set_data(lg['x'][:i + 1], lg['yhat'][:i + 1])
        dot.set_data([lg['x'][i]], [lg['y'][i]])
        est.set_data([lg['x'][i]], [lg['yhat'][i]])
        x0, y0 = lg['x'][i], lg['y'][i]
        dl, dr = min(lg['dl'][i], RANGE_MAX), min(lg['dr'][i], RANGE_MAX)
        bl.set_data([x0, x0], [y0, y0 - dl])
        br.set_data([x0, x0], [y0, y0 + dr])
        ax.set_title(f"S064 top view — t = {lg['t'][i]:.0f} s, "
                     f"|ŷ − y| = {abs(lg['yhat'][i] - lg['y'][i]) * 100:.1f} cm (fusion, seed {PLOT_SEED})")
        return tr, te, dot, est, bl, br

    anim = animation.FuncAnimation(fig, update, frames=n_frames, interval=80, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    anim.save(path, writer='pillow', fps=12)
    plt.close(fig)
    print(f'Saved: {path}  ({n_frames} frames)')


def _ms(v, fmt='.4f'):
    v = np.asarray(v, dtype=float)
    return f'{v.mean():{fmt}} ± {v.std():{fmt}}'


if __name__ == '__main__':
    mc, asweep = run_simulation()
    print('\n=== S064 Greenhouse Interior Precision Flight — Results ===')
    print(f"  Aisle centres y: {', '.join(f'{v:.2f}' for v in AISLE_Y)} m; "
          f'{len(WPS)} waypoints, {len(ROW_END_IDX)} row-ends')
    print(f'  Card drift claim: sigma_drift*sqrt(300 s) = {SIGMA_DRIFT * math.sqrt(300):.3f} m '
          f'per random walk; estimate - truth (two independent walks) = '
          f'{SIGMA_DRIFT * math.sqrt(600):.3f} m at 300 s')
    print(f'  Lateral margin: aisle half-width 0.90 m - drone half-size {DRONE_R} m - d_safe '
          f'{D_SAFE} m = {0.9 - DRONE_R - D_SAFE:.2f} m of error before a d_safe violation')
    for s in STRATS:
        R = mc[s]
        g = lambda f: [r[f] for r in R]          # noqa: E731
        print(f'  [{LABELS[s]}]')
        print(f"    Missions complete  : {sum(g('complete'))}/{N_SEEDS}  "
              f"(crashes {sum(g('crash'))}); row-ends {_ms(g('row_ends'), '.2f')} of 5")
        done = [r['time'] for r in R if r['complete']]
        if done:
            print(f'    Mission time       : {np.mean(done):.1f} ± {np.std(done):.1f} s (completed runs)')
        print(f"    Mean |y_hat - y|   : {_ms(g('mean_err'))} m")
        print(f"    Peak |y_hat - y|   : {_ms(g('max_err'))} m (worst {np.max(g('max_err')):.3f} m)")
        print(f"    Min edge clearance : {_ms(g('min_clear'), '.3f')} m (worst {np.min(g('min_clear')):.3f} m)")
        print(f"    Steps below d_safe : {_ms(g('viol'), '.1f')}; runs with zero "
              f"{sum(v == 0 for v in g('viol'))}/{N_SEEDS}")
        if s == 'fusion':
            print(f"    Steps with a wall fix: {100 * np.mean(g('fix_frac')):.1f} %")
    e_f = np.array([r['mean_err'] for r in mc['fusion']])
    e_d = np.array([r['mean_err'] for r in mc['dr_rep']])
    print(f'  Per-seed mean error: fusion < DR + repulsion in {int(np.sum(e_f < e_d))}/{N_SEEDS}')
    print('  Alpha sweep (10 seeds): alpha -> mean / max |y_hat - y|, min clearance, complete')
    for a in ALPHAS:
        R = asweep[a]
        print(f"    {a:.2f}: {np.mean([r['mean_err'] for r in R]):.4f} / "
              f"{np.mean([r['max_err'] for r in R]):.4f} m, "
              f"{np.min([r['min_clear'] for r in R]):.3f} m, "
              f"{sum(r['complete'] for r in R)}/{len(R)}")
    rf = mc['fusion'][PLOT_SEED]
    print(f"  Seed {PLOT_SEED} fusion: row-ends at " +
          ', '.join(f'{t:.1f}' for t in rf['reach_t']) +
          f" s; mean err {rf['mean_err']:.4f} m, peak {rf['max_err']:.4f} m, "
          f"min clearance {rf['min_clear']:.3f} m")
    rd = mc['dr'][PLOT_SEED]
    print(f"  Seed {PLOT_SEED} DR only: mean err {rd['mean_err']:.4f} m, peak {rd['max_err']:.4f} m, "
          f"min clearance {rd['min_clear']:.3f} m, complete {rd['complete']}")

    out = os.path.normpath(OUTPUT_DIR)
    os.makedirs(out, exist_ok=True)
    plot_3d_trajectory(mc, out)
    plot_analysis(mc, out)
    plot_comparison(mc, asweep, out)
    save_animation(mc, out)
