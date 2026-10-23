"""
S015 3D Upgrade — Relay Tracking
=================================
A target flies a helix (r = 3 m orbit, z = 2 + 1.5 sin 0.4t).  Three bearing-
only sensor drones measure azimuth and elevation with Gaussian noise.  Every
step the two highest-SNR sensors inside their 8 m zone are triangulated with
the card's closest-approach midpoint.  With fewer than two sensors in zone the
estimate is held (bearing-only sensors cannot measure range).

Geometry quality: bearing-only Cramér–Rao bound.  Each sensor contributes an
azimuth row (1/r_xy)[−sin α, cos α, 0] and an elevation row
(1/r)[−sin β cos α, −sin β sin α, cos β], weighted by 1/σ; the position std
bound is σ_pos = sqrt(tr((HᵀWH)⁻¹)).  (The card's DOP stacks unit range
vectors, which is range geometry and is rank-deficient for coplanar sensors.)

Comparison (card): altitude-stratified sensors (z = 1.5 / 3.5 / 5.0 m) vs all
sensors at z = 3 m (card) and at z = 2 m (the target's mean altitude), over
50 noise seeds with common random numbers.

Usage:
    conda activate drones
    python src/01_pursuit_evasion/3d/s015_3d_relay_tracking.py
"""
import sys, os, numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from matplotlib.animation import FuncAnimation, PillowWriter

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..'))

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..', '..', 'outputs',
    '01_pursuit_evasion', '3d', 's015_3d_relay_tracking',
)
os.makedirs(OUTPUT_DIR, exist_ok=True)

# ── Parameters (card) ─────────────────────────────────────────
SENSOR_XY = np.array([[0.0, 4.0], [-3.46, -2.0], [3.46, -2.0]])
Z_STACKED = [1.5, 3.5, 5.0]
Z_FLAT3 = [3.0, 3.0, 3.0]     # card comparison
Z_FLAT2 = [2.0, 2.0, 2.0]     # target's mean altitude
CONFIGS = [('stacked', Z_STACKED), ('flat z=3', Z_FLAT3), ('flat z=2', Z_FLAT2)]
R_ZONE = 8.0
SIGMA_ALPHA = 0.05
SIGMA_BETA = 0.07
TARGET_SPEED = 2.0
ORBIT_R = 3.0
HANDOFF_THRESH = 0.4
SIGMA_Z_ALT = 2.0
DT = 0.05
T_MAX = 25.0
Z_MIN, Z_MAX = 0.5, 6.5
BIG_ERR = 1.0          # m  "large error" threshold for attribution
N_SEEDS = 50
MAIN_SEED = 0
SENSOR_COLORS = ['tomato', 'peru', 'mediumseagreen']


def sensors(z):
    return np.column_stack([SENSOR_XY, z]).astype(float)


def target_path_3d(t):
    w = TARGET_SPEED / ORBIT_R
    return np.array([ORBIT_R * np.cos(w * t), ORBIT_R * np.sin(w * t), 2.0 + 1.5 * np.sin(0.4 * t)])


# ── Sensor model ──────────────────────────────────────────────

def snr_3d(sp, pt):
    base = max(0.0, 1.0 - np.linalg.norm(pt - sp) / R_ZONE)
    return base * float(np.exp(-0.5 * ((pt[2] - sp[2]) / SIGMA_Z_ALT) ** 2))


def true_bearing(sp, pt):
    d = pt - sp
    return np.arctan2(d[1], d[0]), np.arctan2(d[2], np.hypot(d[0], d[1]))


def bearing_to_direction(a, b):
    return np.array([np.cos(b) * np.cos(a), np.cos(b) * np.sin(a), np.sin(b)])


def triangulate_3d(p1, d1, p2, d2):
    """Card closest-approach midpoint between two rays."""
    w = p1 - p2
    b = float(d1 @ d2)
    denom = 1.0 - b * b
    if denom < 1e-8:
        return None
    dv, ev = float(d1 @ w), float(d2 @ w)
    s1 = (b * ev - dv) / denom
    s2 = (ev - b * dv) / denom
    return 0.5 * ((p1 + s1 * d1) + (p2 + s2 * d2))


def bearing_crlb(sps, pt):
    """Cramér–Rao position std bound (m) for azimuth/elevation sensors at sps.
    Returns (sigma_pos, sigma_z)."""
    rows = []
    for sp in sps:
        d = pt - sp
        rxy, r = np.hypot(d[0], d[1]), np.linalg.norm(d)
        a, b = np.arctan2(d[1], d[0]), np.arctan2(d[2], rxy)
        rows.append(np.array([-np.sin(a), np.cos(a), 0.0]) / (rxy * SIGMA_ALPHA))
        rows.append(np.array([-np.sin(b) * np.cos(a), -np.sin(b) * np.sin(a), np.cos(b)]) / (r * SIGMA_BETA))
    H = np.array(rows)
    P = np.linalg.inv(H.T @ H)
    return float(np.sqrt(np.trace(P))), float(np.sqrt(P[2, 2]))


# ── Simulation ────────────────────────────────────────────────

def run_simulation(z_sensors=Z_STACKED, seed=MAIN_SEED):
    rng = np.random.default_rng(seed)
    S = sensors(z_sensors)
    n = int(round(T_MAX / DT))
    L = {k: [] for k in ['t', 'true', 'est', 'err', 'err_z', 'primary', 'pair', 'n_zone', 'n_snr',
                         'crlb', 'crlb_z', 'cross', 'mode', 'snr']}
    handoffs = []
    primary = None
    last = None
    for step in range(n):
        t = step * DT
        pt = target_path_3d(t)
        noise = rng.normal(0, 1, (3, 2)) * [SIGMA_ALPHA, SIGMA_BETA]   # all sensors, every step (CRN)
        snr = np.array([snr_3d(S[i], pt) for i in range(3)])
        in_zone = [i for i in range(3) if np.linalg.norm(pt - S[i]) <= R_ZONE]
        p_new = int(np.argmax(snr))
        if primary is not None and p_new != primary:
            handoffs.append(t)
        primary = p_new
        ranked = sorted(in_zone, key=lambda i: -snr[i])
        pair, cross, crlb, crlb_z = None, np.nan, np.nan, np.nan
        if len(ranked) >= 2:
            i, j = ranked[0], ranked[1]
            ai, bi = true_bearing(S[i], pt); aj, bj = true_bearing(S[j], pt)
            di = bearing_to_direction(ai + noise[i, 0], bi + noise[i, 1])
            dj = bearing_to_direction(aj + noise[j, 0], bj + noise[j, 1])
            est = triangulate_3d(S[i], di, S[j], dj)
            ui = (pt - S[i]) / np.linalg.norm(pt - S[i]); uj = (pt - S[j]) / np.linalg.norm(pt - S[j])
            cross = float(np.degrees(np.arccos(np.clip(ui @ uj, -1, 1))))
            crlb, crlb_z = bearing_crlb(S[[i, j]], pt)
            if est is None:
                est, mode = (last if last is not None else S[i]), 'hold (parallel rays)'
            else:
                mode = 'triangulated'
            pair = (i, j)
        else:
            est, mode = (last if last is not None else S[ranked[0]] if ranked else S[0]), 'hold (<2 in zone)'
        last = est.copy()
        L['t'].append(t); L['true'].append(pt); L['est'].append(est.copy())
        L['err'].append(float(np.linalg.norm(est - pt))); L['err_z'].append(float(abs(est[2] - pt[2])))
        L['primary'].append(primary); L['pair'].append(pair); L['n_zone'].append(len(in_zone))
        L['n_snr'].append(int((snr > HANDOFF_THRESH).sum())); L['snr'].append(snr)
        L['crlb'].append(crlb); L['crlb_z'].append(crlb_z); L['cross'].append(cross); L['mode'].append(mode)
    out = {k: (np.array(v) if k not in ('pair', 'mode') else v) for k, v in L.items()}
    out['handoffs'] = handoffs
    out['S'] = S
    return out


def summarize(r):
    e = r['err']
    tri = np.array([m == 'triangulated' for m in r['mode']])
    return dict(mean=float(e.mean()), median=float(np.median(e)), p95=float(np.percentile(e, 95)),
                max=float(e.max()), t_max=float(r['t'][np.argmax(e)]), mean_z=float(r['err_z'].mean()),
                median_z=float(np.median(r['err_z'])), tri=int(tri.sum()), n=len(e),
                crlb_med=float(np.nanmedian(r['crlb'])), crlb_z_med=float(np.nanmedian(r['crlb_z'])))


def attribution(r):
    """Large errors vs geometry and handoff timing."""
    e, t = r['err'], r['t']
    big = np.where(e > BIG_ERR)[0]
    if not len(big):
        return None
    h = np.array(r['handoffs']) if r['handoffs'] else np.array([np.inf])
    off = np.array([t[k] - h[np.argmin(np.abs(h - t[k]))] for k in big])
    tri = np.array([r['mode'][k] == 'triangulated' for k in big])
    cr = r['crlb'][big]
    return dict(n=len(big), n_tri=int(tri.sum()), n_hold=int((~tri).sum()), off_min=float(off.min()),
                off_max=float(off.max()), abs_off_min=float(np.abs(off).min()),
                crlb_big_med=float(np.nanmedian(cr)) if tri.any() else np.nan,
                crlb_all_med=float(np.nanmedian(r['crlb'])),
                cross_big=(float(np.nanmin(r['cross'][big])), float(np.nanmax(r['cross'][big]))),
                near_handoff=int((np.abs(off) <= 0.25).sum()))


# ── Plots ────────────────────────────────────────────────────

def plot_trajectories_3d(r, out_dir):
    fig = plt.figure(figsize=(12, 9))
    ax = fig.add_subplot(111, projection='3d'); ax.view_init(elev=22, azim=-60)
    tp, est = r['true'], r['est']
    ax.plot(*tp.T, color='royalblue', lw=2.0, label='Target (helix)')
    tri = np.array([m == 'triangulated' for m in r['mode']])
    sc = ax.scatter(*est[tri].T, c=np.clip(r['err'][tri], 0, 2), cmap='viridis', s=6, label='Triangulated estimate')
    if (~tri).any():
        ax.scatter(*est[~tri].T, color='black', marker='x', s=12, label='Held estimate (<2 sensors in zone)')
    for i, (sp, c) in enumerate(zip(r['S'], SENSOR_COLORS)):
        ax.scatter(*sp, color=c, s=220, marker='*', label=f'Sensor {i} z={sp[2]} m')
    fig.colorbar(sc, ax=ax, shrink=0.6, label='3D error (m), clipped at 2')
    ax.set_xlim(-6, 6); ax.set_ylim(-6, 6); ax.set_zlim(Z_MIN, Z_MAX)
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
    ax.set_title(f'S015 3D Relay Tracking — stacked sensors, seed {MAIN_SEED}')
    ax.legend(fontsize=7, loc='upper left')
    plt.tight_layout()
    path = os.path.join(out_dir, 'trajectories_3d.png')
    plt.savefig(path, dpi=120, bbox_inches='tight'); plt.close('all'); print(f'Saved: {path}')


def plot_tracking_error(r, out_dir):
    fig, axes = plt.subplots(4, 1, figsize=(12, 12), sharex=True)
    t = r['t']
    hold = np.array([m != 'triangulated' for m in r['mode']])
    axes[0].plot(t, r['err'], color='steelblue', lw=1.2, label='3D error')
    if hold.any():
        axes[0].scatter(t[hold], r['err'][hold], color='black', marker='x', s=10, label='held estimate')
    axes[0].set_yscale('log'); axes[0].set_ylabel('3D error (m)')
    axes[1].plot(t, r['err_z'], color='tomato', lw=1.2, label='z error'); axes[1].set_yscale('log')
    axes[1].set_ylabel('z error (m)')
    axes[2].plot(t, r['crlb'], color='purple', lw=1.2, label='σ_pos bound, pair used')
    axes[2].plot(t, r['crlb_z'], color='violet', lw=1.0, ls='--', label='σ_z bound, pair used')
    axes[2].set_yscale('log'); axes[2].set_ylabel('Bearing CRLB (m)')
    axes[3].plot(t, r['cross'], color='darkgreen', lw=1.2, label='angle between the two lines of sight')
    axes[3].axhline(90, color='grey', ls=':', lw=1)
    axes[3].set_ylabel('Crossing angle (deg)'); axes[3].set_xlabel('Time (s)')
    for ax in axes:
        for h in r['handoffs']:
            ax.axvline(h, color='orange', ls=':', lw=1.0)
        ax.grid(True, alpha=0.3, which='both'); ax.legend(fontsize=8, loc='upper right')
    axes[0].set_title('S015 3D Relay Tracking — error, bearing-only geometry bound and crossing angle '
                      '(orange = handoff)', fontsize=10)
    plt.tight_layout()
    path = os.path.join(out_dir, 'tracking_error.png')
    plt.savefig(path, dpi=120, bbox_inches='tight'); plt.close('all'); print(f'Saved: {path}')


def plot_error_vs_geometry(r, out_dir):
    tri = np.array([m == 'triangulated' for m in r['mode']])
    fig, ax = plt.subplots(figsize=(9, 6))
    sc = ax.scatter(r['crlb'][tri], r['err'][tri], c=r['cross'][tri], cmap='coolwarm', s=14, vmin=0, vmax=180)
    x = np.logspace(np.log10(np.nanmin(r['crlb'])), np.log10(np.nanmax(r['crlb'])), 50)
    ax.plot(x, x, 'k--', lw=1, label='error = σ_pos bound'); ax.plot(x, 3 * x, 'k:', lw=1, label='3 × bound')
    ax.axhline(BIG_ERR, color='grey', lw=0.8)
    ax.set_xscale('log'); ax.set_yscale('log')
    ax.set_xlabel('Bearing CRLB σ_pos of the pair used (m)'); ax.set_ylabel('3D error (m)')
    fig.colorbar(sc, ax=ax, label='crossing angle (deg)')
    ax.set_title('S015 3D — triangulation error vs geometry (stacked, seed 0)', fontsize=10)
    ax.grid(True, alpha=0.3, which='both'); ax.legend(fontsize=8)
    plt.tight_layout()
    path = os.path.join(out_dir, 'error_vs_geometry.png')
    plt.savefig(path, dpi=120, bbox_inches='tight'); plt.close('all'); print(f'Saved: {path}')


def plot_coverage_status(r, out_dir):
    fig, axes = plt.subplots(4, 1, figsize=(12, 7), sharex=True)
    t = r['t']; snr = r['snr']
    for i, (ax, c) in enumerate(zip(axes[:3], SENSOR_COLORS)):
        ax.plot(t, snr[:, i], color=c, lw=1.4)
        used = np.array([p is not None and i in p for p in r['pair']])
        ax.fill_between(t, 0, 1, where=used, color=c, alpha=0.15, step='mid', label='in triangulation pair')
        ax.axhline(HANDOFF_THRESH, color='grey', ls='--', lw=0.8)
        ax.set_ylabel(f'S{i} SNR\nz={r["S"][i, 2]} m', fontsize=8); ax.set_ylim(0, 1)
        ax.legend(fontsize=7, loc='upper right'); ax.grid(True, alpha=0.3)
    axes[3].step(t, r['n_zone'], color='black', where='mid', label='sensors in 8 m zone')
    axes[3].step(t, r['n_snr'], color='grey', where='mid', ls='--', label='sensors with SNR > 0.4')
    axes[3].set_ylabel('count'); axes[3].set_xlabel('Time (s)'); axes[3].legend(fontsize=7)
    axes[3].grid(True, alpha=0.3)
    for ax in axes:
        for h in r['handoffs']:
            ax.axvline(h, color='orange', ls=':', lw=1.0)
    axes[0].set_title('S015 3D — altitude-weighted SNR, sensors used, and coverage (stacked, seed 0)', fontsize=10)
    plt.tight_layout()
    path = os.path.join(out_dir, 'coverage_status.png')
    plt.savefig(path, dpi=120, bbox_inches='tight'); plt.close('all'); print(f'Saved: {path}')


def plot_altitude_time(r, out_dir):
    fig, ax = plt.subplots(figsize=(12, 5))
    ax.plot(r['t'], r['true'][:, 2], color='royalblue', lw=2.0, label='Target z')
    ax.plot(r['t'], r['est'][:, 2], color='red', lw=0.8, alpha=0.7, label='Estimated z (stacked)')
    for i, (sp, c) in enumerate(zip(r['S'], SENSOR_COLORS)):
        ax.axhline(sp[2], color=c, ls='--', lw=1.3, label=f'Sensor {i} z={sp[2]} m')
    ax.axhline(3.0, color='grey', ls=':', lw=1.0, label='flat config z = 3 m')
    ax.set_ylim(0, 7)
    ax.set_xlabel('Time (s)'); ax.set_ylabel('z (m)'); ax.legend(fontsize=8, ncol=3); ax.grid(True, alpha=0.3)
    ax.set_title('S015 3D Relay Tracking — Altitude vs Time (seed 0)')
    plt.tight_layout()
    path = os.path.join(out_dir, 'altitude_time.png')
    plt.savefig(path, dpi=120, bbox_inches='tight'); plt.close('all'); print(f'Saved: {path}')


def plot_flat_vs_stacked(mc, out_dir):
    names = [c[0] for c in CONFIGS]
    fig, axes = plt.subplots(1, 3, figsize=(16, 5))
    for ax, key, lab in zip(axes[:2], ['median', 'median_z'],
                            ['median 3D error per seed (m)', 'median z error per seed (m)']):
        data = [[s[key] for s in mc[nm]] for nm in names]
        ax.boxplot(data, tick_labels=names, showmeans=True)
        ax.set_ylabel(lab); ax.grid(True, alpha=0.3)
    x = np.arange(len(names)); w = 0.38
    pos = [mc[nm][0]['crlb_med'] for nm in names]; zz = [mc[nm][0]['crlb_z_med'] for nm in names]
    axes[2].bar(x - w / 2, pos, w, color='purple', label='σ_pos bound')
    axes[2].bar(x + w / 2, zz, w, color='violet', label='σ_z bound')
    for xi, (p_, z_) in enumerate(zip(pos, zz)):
        axes[2].text(xi - w / 2, p_ + 0.005, f'{p_:.3f}', ha='center', fontsize=8)
        axes[2].text(xi + w / 2, z_ + 0.005, f'{z_:.3f}', ha='center', fontsize=8)
    axes[2].set_xticks(x); axes[2].set_xticklabels(names)
    axes[2].set_ylabel('median bearing CRLB of the pair used (m)\n(noise-free geometry, same for every seed)')
    axes[2].legend(fontsize=8); axes[2].grid(True, alpha=0.3, axis='y')
    axes[0].set_title(f'{N_SEEDS} noise seeds, common random numbers', fontsize=9)
    fig.suptitle('S015 3D — altitude-stratified vs flat sensors', fontsize=11)
    plt.tight_layout()
    path = os.path.join(out_dir, 'flat_vs_stacked.png')
    plt.savefig(path, dpi=120, bbox_inches='tight'); plt.close('all'); print(f'Saved: {path}')


def save_animation(r, out_dir):
    t, tp, est, S = r['t'], r['true'], r['est'], r['S']
    frames = list(range(0, len(t), 4))
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d'); ax.view_init(elev=22, azim=-60)
    for i, (sp, c) in enumerate(zip(S, SENSOR_COLORS)):
        ax.scatter(*sp, color=c, s=200, marker='*')
        ax.text(sp[0] + 0.2, sp[1] + 0.2, sp[2] + 0.1, f'S{i}', fontsize=9, color=c)
    ax.set_xlim(-6, 6); ax.set_ylim(-6, 6); ax.set_zlim(Z_MIN, Z_MAX)
    ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
    tl, = ax.plot([], [], [], color='royalblue', lw=2.0, label='Target')
    td, = ax.plot([], [], [], 'bo', ms=9)
    ed, = ax.plot([], [], [], 'rx', ms=11, mew=2.5, label='Estimate')
    rays = [ax.plot([], [], [], color=SENSOR_COLORS[k], lw=1.0)[0] for k in range(3)]
    title = ax.set_title(''); ax.legend(fontsize=8, loc='upper left')

    def update(fi):
        si = frames[fi]
        tl.set_data(tp[:si+1, 0], tp[:si+1, 1]); tl.set_3d_properties(tp[:si+1, 2])
        td.set_data([tp[si, 0]], [tp[si, 1]]); td.set_3d_properties([tp[si, 2]])
        ed.set_data([est[si, 0]], [est[si, 1]]); ed.set_3d_properties([est[si, 2]])
        pair = r['pair'][si] or ()
        for k in range(3):
            if k in pair:
                rays[k].set_data([S[k, 0], tp[si, 0]], [S[k, 1], tp[si, 1]])
                rays[k].set_3d_properties([S[k, 2], tp[si, 2]])
            else:
                rays[k].set_data([], []); rays[k].set_3d_properties([])
        title.set_text(f"S015 3D  t={t[si]:.2f}s  err={r['err'][si]:.2f}m  {r['mode'][si]}")
        return [tl, td, ed, title] + rays

    ani = FuncAnimation(fig, update, frames=len(frames), interval=60, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer=PillowWriter(fps=16), dpi=90); plt.close('all')
    print(f'Saved: {path}  ({len(frames)} frames)')


# ── Main ─────────────────────────────────────────────────────

if __name__ == '__main__':
    print(f'S015 3D Relay Tracking — main run: stacked sensors, seed {MAIN_SEED}')
    main = run_simulation(Z_STACKED, MAIN_SEED)
    s = summarize(main)
    modes = {m: main['mode'].count(m) for m in sorted(set(main['mode']))}
    print(f"  handoffs: {len(main['handoffs'])} at t = {[round(h, 2) for h in main['handoffs']]}")
    print(f"  steps: {s['n']}; " + ', '.join(f'{k} {v}' for k, v in modes.items()))
    two = main['t'][main['n_zone'] < 3]
    print(f"  sensors in 8 m zone: 3 in {int((main['n_zone'] == 3).sum())} steps, fewer in {len(two)} steps"
          + (f" (t = {two.min():.2f}–{two.max():.2f} s, minimum {int(main['n_zone'].min())})" if len(two) else ''))
    print(f"  steps with SNR > {HANDOFF_THRESH}: 0 sensors {int((main['n_snr'] == 0).sum())}, 1 sensor "
          f"{int((main['n_snr'] == 1).sum())}, 2+ sensors {int((main['n_snr'] >= 2).sum())}")
    print(f"  3D error: mean {s['mean']:.3f} m, median {s['median']:.3f} m, p95 {s['p95']:.3f} m, "
          f"max {s['max']:.3f} m at t = {s['t_max']:.2f} s; z error mean {s['mean_z']:.3f} m, "
          f"median {s['median_z']:.3f} m")
    k = int(np.argmax(main['err']))
    used2 = sum(p is not None and 2 in p for p in main['pair'])
    print(f"  sensor 2 (highest tier) is in the triangulation pair in {used2}/{s['n']} steps")
    h = np.array(main['handoffs'])
    print(f"  max-error step: mode '{main['mode'][k]}', pair {main['pair'][k]}, crossing angle "
          f"{main['cross'][k]:.1f} deg, σ_pos bound {main['crlb'][k]:.3f} m, offset to nearest handoff "
          f"{main['t'][k] - h[np.argmin(np.abs(h - main['t'][k]))]:+.2f} s")
    a = attribution(main)
    if a:
        print(f"  errors > {BIG_ERR} m: {a['n']} steps ({a['n_tri']} triangulated, {a['n_hold']} held); "
              f"offset to nearest handoff {a['off_min']:+.2f} … {a['off_max']:+.2f} s "
              f"(closest {a['abs_off_min']:.2f} s; {a['near_handoff']} within ±0.25 s); crossing angle "
              f"{a['cross_big'][0]:.1f}–{a['cross_big'][1]:.1f} deg; median σ_pos bound {a['crlb_big_med']:.3f} m "
              f"vs {a['crlb_all_med']:.3f} m over all triangulated steps")
    tri = np.array([m == 'triangulated' for m in main['mode']])
    rho = np.corrcoef(np.log(main['crlb'][tri]), np.log(main['err'][tri]))[0, 1]
    print(f"  log(error) vs log(σ_pos bound), triangulated steps: correlation {rho:.2f}")
    ratio = main['err'][tri] / main['crlb'][tri]
    print(f"  error / σ_pos bound: median {np.median(ratio):.2f}, p95 {np.percentile(ratio, 95):.2f}, "
          f"max {ratio.max():.2f}; error at the max-bound step {main['err'][tri][np.argmax(main['crlb'][tri])]:.3f} m "
          f"(bound {np.nanmax(main['crlb']):.3f} m)")
    hd = np.array(main['handoffs'])
    near = np.array([np.min(np.abs(hd - x)) <= 0.5 for x in main['t']])
    print(f"  mean 3D error within ±0.5 s of a handoff {main['err'][near].mean():.3f} m ({near.sum()} steps) "
          f"vs elsewhere {main['err'][~near].mean():.3f} m; median {np.median(main['err'][near]):.3f} vs "
          f"{np.median(main['err'][~near]):.3f} m")

    print(f'\nFlat vs stacked: {N_SEEDS} noise seeds, common random numbers (same noise per sensor and step)')
    mc = {nm: [summarize(run_simulation(z, sd)) for sd in range(N_SEEDS)] for nm, z in CONFIGS}
    for nm, _ in CONFIGS:
        m = mc[nm]
        f = lambda key: np.array([x[key] for x in m])
        print(f"  {nm:<9} median 3D error {f('median').mean():.3f} m (per-seed mean of medians), mean 3D "
              f"error {f('mean').mean():.3f} m, p95 {f('p95').mean():.3f} m, max over seeds {f('max').max():.2f} m; "
              f"median z error {f('median_z').mean():.3f} m, mean z error {f('mean_z').mean():.3f} m; "
              f"triangulated {f('tri').mean():.0f}/{m[0]['n']} steps; median σ_pos bound {f('crlb_med').mean():.3f} m "
              f"(σ_z {f('crlb_z_med').mean():.3f} m)")
    for other in ('flat z=3', 'flat z=2'):
        for key, lab in (('median', 'median 3D error'), ('median_z', 'median z error')):
            d = np.array([b[key] - a_[key] for a_, b in zip(mc['stacked'], mc[other])])
            print(f"  paired {lab}: {other} − stacked = {d.mean():+.3f} ± {d.std():.3f} m; "
                  f"stacked better in {(d > 0).sum()}/{N_SEEDS} seeds")

    plot_trajectories_3d(main, OUTPUT_DIR)
    plot_tracking_error(main, OUTPUT_DIR)
    plot_error_vs_geometry(main, OUTPUT_DIR)
    plot_coverage_status(main, OUTPUT_DIR)
    plot_altitude_time(main, OUTPUT_DIR)
    plot_flat_vs_stacked(mc, OUTPUT_DIR)
    save_animation(main, OUTPUT_DIR)
    print('All outputs saved to:', OUTPUT_DIR)
