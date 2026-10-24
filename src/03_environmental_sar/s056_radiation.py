"""
S056 Nuclear Plant Leak Detection (Radiation Hotspot Localization)
==================================================================
A drone with a dose-rate detector surveys a 500 x 500 m exclusion zone at 15 m altitude to
localise one point source (inverse-square law with air attenuation, plus background).
The source position is a discrete latent variable over a 50 x 50 grid of 10 m cells with a
uniform prior; every measurement updates the posterior with a Gaussian likelihood whose
noise grows with the dose (counting statistics). The mission stops when the posterior
peak exceeds 0.90 or after 80 measurements.

Strategies (card):
  1. Greedy maximum expected information gain (EIG, Gaussian approximation)
  2. Greedy maximum belief (fly to the current MAP cell)
  3. Lawnmower: fixed 8 x 10 lattice flown boustrophedon, no feedback
All three take their first reading at the launch point (250, 250) m. A 50-trial Monte Carlo
draws the true source uniformly (continuous, so it is generally not at a cell centre); every
strategy sees the same sources and the same noise draws. The run is repeated with a 10x
noisier detector to show where the strategies separate.

Usage:
    conda activate drones
    python src/03_environmental_sar/s056_radiation.py
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.colors import LogNorm
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401

# ── Parameters (card) ─────────────────────────────────────────────────────────
AREA_SIZE  = 500.0
GRID_N     = 50
CELL_SIZE  = AREA_SIZE / GRID_N        # 10 m
H_SURVEY   = 15.0                      # m
V_DRONE    = 8.0                       # m/s
DT_INT     = 5.0                       # s
N_MEAS_MAX = 80
P_CONF     = 0.90
N_COOLDOWN = 5
A_SOURCE   = 2000.0                    # uSv m^2 / h
MU_ATTEN   = 0.0002                    # 1/m
D_BG       = 0.10                      # uSv/h
SIGMA_0    = 0.04                      # uSv^0.5 h^-0.5
N_TRIALS   = 50
EPS_TARGET = 10.0                      # m

# ── Parameters (added) ────────────────────────────────────────────────────────
LAUNCH     = np.array([250.0, 250.0])
LAWN_NX, LAWN_NY = 8, 10               # lattice for the lawnmower (80 points)
SIGMA_MULT = [1.0, 10.0]               # detector noise multipliers in the Monte Carlo
DEMO_SOURCE = np.array([175.0, 310.0]) # card's illustrative source
N_BOOT     = 1000

STRATS = ['eig', 'belief', 'lawn']
LABELS = {'eig': 'Greedy EIG', 'belief': 'Greedy max belief', 'lawn': 'Lawnmower'}
COLORS = {'eig': '#2CA02C', 'belief': '#1F77B4', 'lawn': '#7F7F7F'}

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), '..', '..',
                          'outputs', '03_environmental_sar', 's056_radiation')

XS = (np.arange(GRID_N) + 0.5) * CELL_SIZE
GX, GY = np.meshgrid(XS, XS, indexing='ij')
CELLS = np.stack([GX.ravel(), GY.ravel()], axis=-1)          # (2500, 2)


# ── Physics ───────────────────────────────────────────────────────────────────

def dose_rate(p, src, A=A_SOURCE):
    r = np.sqrt(np.sum((np.asarray(p) - np.asarray(src)) ** 2, axis=-1) + H_SURVEY ** 2)
    return D_BG + A / r ** 2 * np.exp(-MU_ATTEN * r)


DMAT = dose_rate(CELLS[:, None, :], CELLS[None, :, :])       # [candidate, source]
DMAT2 = DMAT ** 2


def measure(p, src, rng, sigma0):
    d = float(dose_rate(p, src))
    return d + rng.normal(0.0, sigma0 * np.sqrt(d))


def bayes_update(logb, z, d_pred, sigma0):
    sig = sigma0 * np.sqrt(d_pred)
    logb = logb - 0.5 * ((z - d_pred) / sig) ** 2 - np.log(sig)
    return logb - logb.max()


def normalise(logb):
    b = np.exp(logb)
    return b / b.sum()


def entropy(b):
    q = b[b > 0]
    return float(-np.sum(q * np.log(q)))


def eig_map(b, sigma0):
    zbar = DMAT @ b
    var_model = np.maximum(DMAT2 @ b - zbar ** 2, 0.0)
    noise = sigma0 ** 2 * zbar
    return 0.5 * np.log1p(var_model / noise)


def pick_max(score, pos):
    """Deterministic argmax: scores equal to 1e-9 (relative) are ties, broken by distance
    from the drone, then by cell index. Without this, exact symmetric ties (e.g. the first EIG
    choice after the launch reading) are settled by BLAS rounding and depend on the thread count."""
    finite = np.isfinite(score)
    scale = np.abs(score[finite]).max()
    q = np.full(score.shape, -np.inf)
    q[finite] = np.round(score[finite] / (scale if scale > 0 else 1.0) * 1e9)
    cand = np.flatnonzero(q == q.max())
    d = np.round(np.sum((CELLS[cand] - pos) ** 2, axis=1), 6)
    return int(cand[np.lexsort((cand, d))[0]])


def lawn_points():
    xs = (np.arange(LAWN_NX) + 0.5) * AREA_SIZE / LAWN_NX
    ys = (np.arange(LAWN_NY) + 0.5) * AREA_SIZE / LAWN_NY
    pts = []
    for i, x in enumerate(xs):
        col = ys if i % 2 == 0 else ys[::-1]
        pts += [(x, y) for y in col]
    pts = np.array(pts)
    # snap to cell centres
    return CELLS[[int(np.argmin(np.sum((CELLS - p) ** 2, axis=1))) for p in pts]]


LAWN = lawn_points()


# ── Mission ───────────────────────────────────────────────────────────────────

def run_mission(strategy, src, noise, sigma0, record=False):
    """noise: pre-drawn standard normals (CRN); the k-th reading uses noise[k]."""
    logb = np.zeros(len(CELLS))
    pos = LAUNCH.copy()
    path = [pos.copy()]
    k = 0
    d_true = float(dose_rate(pos, src))
    z = d_true + sigma0 * np.sqrt(d_true) * noise[k]
    logb = bayes_update(logb, z, dose_rate(pos, CELLS), sigma0)
    b = normalise(logb)
    hist = {'H': [entropy(b)], 'err': [], 'pmax': [b.max()], 'z': [z], 'beliefs': [], 'eig': []}
    hist['err'].append(float(np.linalg.norm(CELLS[int(np.argmax(b))] - src)))
    if record:
        hist['beliefs'].append(b.copy())
    recent = []
    lawn_i = 0
    while b.max() < P_CONF and len(path) < N_MEAS_MAX:
        if strategy == 'eig':
            score = eig_map(b, sigma0)
            if record:
                hist['eig'].append(score.copy())
        elif strategy == 'belief':
            score = b.copy()
        if strategy == 'lawn':
            if lawn_i >= len(LAWN):
                break
            nxt = LAWN[lawn_i]
            idx = int(np.argmin(np.sum((CELLS - nxt) ** 2, axis=1)))
            lawn_i += 1
        else:
            if recent:
                score[recent] = -np.inf
            idx = pick_max(score, pos)
            recent.append(idx)
            if len(recent) > N_COOLDOWN:
                recent.pop(0)
        new = CELLS[idx]
        assert np.all(new >= 0) and np.all(new <= AREA_SIZE)
        pos = new.copy()
        path.append(pos.copy())
        k += 1
        d_true = float(dose_rate(pos, src))
        z = d_true + sigma0 * np.sqrt(d_true) * noise[k]
        logb = bayes_update(logb, z, DMAT[idx], sigma0)
        b = normalise(logb)
        hist['H'].append(entropy(b))
        hist['pmax'].append(b.max())
        hist['z'].append(z)
        hist['err'].append(float(np.linalg.norm(CELLS[int(np.argmax(b))] - src)))
        if record:
            hist['beliefs'].append(b.copy())
    path = np.array(path)
    seg = np.linalg.norm(np.diff(path, axis=0), axis=1)
    est = CELLS[int(np.argmax(b))]
    mean_est = b @ CELLS
    return {
        'n_meas': len(path), 'converged': bool(b.max() >= P_CONF), 'pmax': float(b.max()),
        'err_map': float(np.linalg.norm(est - src)), 'err_mean': float(np.linalg.norm(mean_est - src)),
        'path_len': float(seg.sum()), 'time': float(seg.sum() / V_DRONE + len(path) * DT_INT),
        'path': path, 'hist': hist, 'est': est, 'belief': b,
        'true_in_top': float(b[int(np.argmin(np.sum((CELLS - src) ** 2, axis=1)))]),
    }


def run_simulation():
    rng = np.random.default_rng(56)
    sources = rng.uniform(0.0, AREA_SIZE, (N_TRIALS, 2))
    noises = rng.standard_normal((N_TRIALS, N_MEAS_MAX + 1))
    mc = {}
    for m in SIGMA_MULT:
        for s in STRATS:
            mc[(s, m)] = [run_mission(s, sources[t], noises[t], SIGMA_0 * m) for t in range(N_TRIALS)]
    demo_noise = np.random.default_rng(42).standard_normal(N_MEAS_MAX + 1)
    demo = {s: run_mission(s, DEMO_SOURCE, demo_noise, SIGMA_0, record=True) for s in STRATS}
    demo10 = run_mission('eig', DEMO_SOURCE, demo_noise, SIGMA_0 * 10, record=True)
    return sources, mc, demo, demo10


def curve(runs, key):
    """Per-measurement curve, carrying the final value forward after a mission stops."""
    out = np.empty((len(runs), N_MEAS_MAX))
    for i, r in enumerate(runs):
        v = np.array(r['hist'][key], dtype=float)
        out[i, :len(v)] = v[:N_MEAS_MAX]
        out[i, len(v):] = v[-1]
    return out


def rms_band(err, rng):
    rms = np.sqrt(np.mean(err ** 2, axis=0))
    boots = np.empty((N_BOOT, err.shape[1]))
    for b in range(N_BOOT):
        i = rng.integers(0, err.shape[0], err.shape[0])
        boots[b] = np.sqrt(np.mean(err[i] ** 2, axis=0))
    return rms, np.percentile(boots, 2.5, axis=0), np.percentile(boots, 97.5, axis=0)


# ── Plotting ──────────────────────────────────────────────────────────────────

def _save(fig, out_dir, name):
    path = os.path.join(out_dir, name)
    fig.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def _img(v):
    return v.reshape(GRID_N, GRID_N).T


def plot_belief_sequence(demo, demo10, out_dir):
    fig, axes = plt.subplots(2, 4, figsize=(17, 8.6))
    for row, (r, lab) in enumerate([(demo['eig'], 'σ0 = 0.04 (card)'), (demo10, 'σ0 = 0.4 (10x)')]):
        n = len(r['hist']['beliefs'])
        ks = sorted(set([0, min(1, n - 1), min(2, n - 1), n - 1]))
        while len(ks) < 4:
            ks.append(n - 1)
        for ax, k in zip(axes[row], ks):
            b = r['hist']['beliefs'][k]
            im = ax.imshow(_img(np.maximum(b, 1e-12)), origin='lower', extent=[0, AREA_SIZE, 0, AREA_SIZE],
                           cmap='coolwarm', norm=LogNorm(vmin=1e-8, vmax=1))
            p = r['path'][:k + 1]
            ax.plot(p[:, 0], p[:, 1], '-o', color='cyan', ms=3, lw=1)
            ax.plot(*DEMO_SOURCE, 'x', color='white', ms=10, mew=2.5)
            ax.plot(*CELLS[int(np.argmax(b))], 'o', color='red', ms=6, mec='k')
            ax.set_title(f'{lab}: after {k + 1} measurement(s)\npeak {b.max():.3f}, '
                         f'H = {entropy(b):.2f} nats', fontsize=9)
            ax.set_xticks([0, 250, 500])
            ax.set_yticks([0, 250, 500])
    fig.colorbar(im, ax=axes, shrink=0.7, label='posterior P(g_j) (log scale)')
    fig.suptitle('S056 Belief maps, greedy EIG (white x = true source, red dot = MAP, cyan = drone path)')
    _save(fig, out_dir, 'belief_sequence.png')


def plot_eig_map(demo10, out_dir):
    k = min(9, len(demo10['hist']['eig']) - 1)
    e = demo10['hist']['eig'][k]
    fig, ax = plt.subplots(figsize=(6.5, 5.6))
    im = ax.imshow(_img(e), origin='lower', extent=[0, AREA_SIZE, 0, AREA_SIZE], cmap='viridis')
    fig.colorbar(im, ax=ax, label='EIG (nats)')
    i = pick_max(e, demo10['path'][k])
    ax.plot(*CELLS[i], 's', mfc='none', mec='red', ms=12, mew=2, label='max EIG over all cells')
    ax.plot(*demo10['path'][k + 1], '*', color='red', ms=16, mec='k',
            label='next waypoint (max EIG outside cooldown)')
    p = demo10['path'][:k + 1]
    ax.plot(p[:, 0], p[:, 1], '-o', color='cyan', ms=3, lw=1, label='path so far')
    ax.plot(*DEMO_SOURCE, 'x', color='white', ms=10, mew=2.5, label='true source')
    ax.legend(fontsize=8, loc='lower right')
    ax.set_title(f'S056 EIG map before measurement {k + 2} (σ0 = 0.4)', fontsize=10)
    _save(fig, out_dir, 'eig_map.png')


def plot_curves(mc, out_dir):
    rng = np.random.default_rng(7)
    fig, axes = plt.subplots(2, 2, figsize=(14, 9))
    for row, m in enumerate(SIGMA_MULT):
        for s in STRATS:
            H = curve(mc[(s, m)], 'H')
            k = np.arange(1, N_MEAS_MAX + 1)
            axes[row, 0].plot(k, H.mean(axis=0), color=COLORS[s], label=LABELS[s])
            err = curve(mc[(s, m)], 'err')
            rms, lo, hi = rms_band(err, rng)
            axes[row, 1].plot(k, rms, color=COLORS[s], label=LABELS[s])
            axes[row, 1].fill_between(k, lo, hi, color=COLORS[s], alpha=0.2)
        axes[row, 0].set_ylabel('mean entropy H_k (nats)')
        axes[row, 0].set_title(f'Entropy decay, σ0 = {SIGMA_0 * m:g}', fontsize=10)
        axes[row, 1].axhline(EPS_TARGET, color='red', ls='--', lw=1, label='10 m target')
        axes[row, 1].set_yscale('log')
        axes[row, 1].set_ylim(1, 1000)
        axes[row, 1].set_ylabel('RMS MAP error (m)')
        axes[row, 1].set_title(f'Localisation error, {N_TRIALS} trials (95 % bootstrap band), '
                               f'σ0 = {SIGMA_0 * m:g}', fontsize=10)
        for ax in axes[row]:
            ax.set_xscale('log')
            ax.set_xlabel('measurement index k (log scale)')
            ax.grid(alpha=0.3)
            ax.legend(fontsize=8)
    fig.suptitle('S056 Entropy and localisation error vs number of measurements '
                 '(values held after a mission stops)')
    plt.tight_layout()
    _save(fig, out_dir, 'entropy_error_curves.png')


def plot_dose_field(out_dir):
    fine = np.linspace(0, AREA_SIZE, 251)
    X, Y = np.meshgrid(fine, fine, indexing='ij')
    D = dose_rate(np.stack([X, Y], axis=-1), DEMO_SOURCE)
    fig, ax = plt.subplots(figsize=(6.8, 5.8))
    im = ax.imshow(D.T, origin='lower', extent=[0, AREA_SIZE, 0, AREA_SIZE], cmap='inferno',
                   norm=LogNorm(vmin=D.min(), vmax=D.max()))
    fig.colorbar(im, ax=ax, label='dose rate at h = 15 m (µSv/h)')
    cs = ax.contour(X, Y, D, levels=[0.5, 1, 5], colors='cyan', linewidths=1)
    ax.clabel(cs, fmt='%g µSv/h', fontsize=8)
    ax.plot(*DEMO_SOURCE, 'x', color='white', ms=10, mew=2.5)
    ax.set_title(f'S056 Dose-rate field, source at (175, 310) m\npeak {D.max():.2f} µSv/h; '
                 f'the 20 µSv/h ring does not exist at 15 m altitude', fontsize=10)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    _save(fig, out_dir, 'dose_field.png')


def plot_paths(demo, demo10, mc, out_dir):
    fig, axes = plt.subplots(1, 2, figsize=(14, 6.4))
    for s in STRATS:
        r = demo[s]
        axes[0].plot(r['path'][:, 0], r['path'][:, 1], '-o', color=COLORS[s], ms=4,
                     label=f"{LABELS[s]}: {r['n_meas']} meas., {r['path_len']:.0f} m")
    axes[0].plot(*DEMO_SOURCE, 'x', color='red', ms=12, mew=3, label='true source')
    axes[0].plot(*LAUNCH, 's', color='k', ms=8, label='launch')
    axes[0].set_xlim(0, AREA_SIZE)
    axes[0].set_ylim(0, AREA_SIZE)
    axes[0].set_aspect('equal')
    axes[0].legend(fontsize=8)
    axes[0].set_title('Demo source (175, 310) m, σ0 = 0.04: flight paths', fontsize=10)
    m = SIGMA_MULT[-1]
    data = [[r['path_len'] for r in mc[(s, m)]] for s in STRATS]
    bp = axes[1].boxplot(data, tick_labels=[LABELS[s] for s in STRATS], patch_artist=True)
    for patch, s in zip(bp['boxes'], STRATS):
        patch.set_facecolor(COLORS[s])
    axes[1].set_ylabel('flight path length (m)')
    axes[1].set_title(f'Path length over {N_TRIALS} trials, σ0 = {SIGMA_0 * m:g}', fontsize=10)
    fig.suptitle('S056 Flight paths and path length')
    _save(fig, out_dir, 'flight_paths.png')


def plot_dose_3d(demo10, out_dir):
    fine = np.linspace(0, AREA_SIZE, 101)
    X, Y = np.meshgrid(fine, fine, indexing='ij')
    D = dose_rate(np.stack([X, Y], axis=-1), DEMO_SOURCE)
    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(111, projection='3d', computed_zorder=False)
    ax.plot_surface(X, Y, np.log10(D) * 5, cmap='inferno', linewidth=0, alpha=0.8, zorder=0)
    p = demo10['path']
    ax.plot(p[:, 0], p[:, 1], np.full(len(p), H_SURVEY), '-o', color='cyan', ms=3, lw=1.2,
            zorder=3, label=f'EIG drone path at h = {H_SURVEY:.0f} m (σ0 = 0.4)')
    ax.scatter(*DEMO_SOURCE, 0, color='lime', s=60, zorder=4, label='true source (ground)')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_zlabel('altitude (m) / 5 log10(dose)')
    ax.view_init(elev=28, azim=-60)
    ax.legend(fontsize=8)
    ax.set_title('S056 Dose surface (5 log10 µSv/h) and survey path')
    _save(fig, out_dir, 'dose_3d.png')


def save_animation(demo10, out_dir):
    beliefs = demo10['hist']['beliefs']
    n = len(beliefs)
    step = 1
    idx = [min(i * step, n - 1) for i in range(int(np.ceil((n - 1) / step)) + 1)]
    fig, ax = plt.subplots(figsize=(6, 5.6))
    im = ax.imshow(_img(np.maximum(beliefs[0], 1e-12)), origin='lower', extent=[0, AREA_SIZE, 0, AREA_SIZE],
                   cmap='coolwarm', norm=LogNorm(vmin=1e-8, vmax=1))
    line, = ax.plot([], [], '-o', color='cyan', ms=3, lw=1)
    ax.plot(*DEMO_SOURCE, 'x', color='white', ms=10, mew=2.5)
    mp, = ax.plot([], [], 'o', color='red', ms=6, mec='k')

    def update(f):
        k = idx[f]
        b = beliefs[k]
        im.set_data(_img(np.maximum(b, 1e-12)))
        line.set_data(demo10['path'][:k + 1, 0], demo10['path'][:k + 1, 1])
        mp.set_data([CELLS[int(np.argmax(b))][0]], [CELLS[int(np.argmax(b))][1]])
        ax.set_title(f"S056 Greedy EIG (σ0 = 0.4), measurement {k + 1}: "
                     f"z = {demo10['hist']['z'][k]:.3f} µSv/h\npeak belief {b.max():.3f}", fontsize=10)
        return im, line, mp

    anim = animation.FuncAnimation(fig, update, frames=len(idx), blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    anim.save(path, writer='pillow', fps=2, dpi=80)
    plt.close(fig)
    from PIL import Image
    with Image.open(path) as gif:
        n_file = gif.n_frames
    print(f'Saved: {path} ({len(idx)} frames rendered, {n_file} in the file, one per measurement)')


if __name__ == '__main__':
    sources, mc, demo, demo10 = run_simulation()
    print('\n=== S056 Radiation Hotspot Localization — Results ===')
    d0 = float(dose_rate(DEMO_SOURCE, DEMO_SOURCE))
    print(f'  Dose directly above a source at 15 m: {d0:.2f} uSv/h (noise sigma {SIGMA_0 * np.sqrt(d0):.3f}); '
          f'at 250 m horizontal: {float(dose_rate([250.0, 0.0], [0.0, 0.0])):.4f} uSv/h '
          f'(noise sigma {SIGMA_0 * np.sqrt(float(dose_rate([250.0, 0.0], [0.0, 0.0]))):.4f})')
    print(f'  Dose 5 m off-centre at 15 m altitude: {float(dose_rate([5.0, 0.0], [0.0, 0.0])):.2f} uSv/h; '
          f'7.07 m: {float(dose_rate([5.0, 5.0], [0.0, 0.0])):.2f} uSv/h')
    d5 = float(dose_rate([5.0, 0.0], [0.0, 0.0]))
    print(f'  Noise / dose above the source: {100 * SIGMA_0 / np.sqrt(d0):.1f} %; '
          f'dose drop 5 m off-centre: {100 * (d0 - d5) / d0:.1f} %')
    print(f'  Initial entropy ln(2500) = {np.log(2500):.2f} nats')
    for s in STRATS:
        r = demo[s]
        print(f"  Demo source ({DEMO_SOURCE[0]:.0f}, {DEMO_SOURCE[1]:.0f}) [{LABELS[s]}]: "
              f"{r['n_meas']} measurements, peak {r['pmax']:.3f}, "
              f"MAP ({r['est'][0]:.0f}, {r['est'][1]:.0f}), error {r['err_map']:.1f} m, "
              f"path {r['path_len']:.0f} m, time {r['time']:.0f} s")
    print(f"  EIG map figure: after {min(9, len(demo10['hist']['eig']) - 1) + 1} readings "
          f"(before reading {min(9, len(demo10['hist']['eig']) - 1) + 2}) of the sigma0 x10 demo")
    print(f"  Demo, EIG, sigma0 x10: {demo10['n_meas']} measurements, peak {demo10['pmax']:.3f}, "
          f"error {demo10['err_map']:.1f} m, path {demo10['path_len']:.0f} m")
    for m in SIGMA_MULT:
        print(f'  Monte Carlo, sigma0 = {SIGMA_0 * m:g} ({N_TRIALS} sources, same sources and noise draws):')
        res = {}
        for s in STRATS:
            runs = mc[(s, m)]
            a = {k: np.array([r[k] for r in runs], dtype=float)
                 for k in ('n_meas', 'converged', 'err_map', 'err_mean', 'path_len', 'time', 'pmax')}
            res[s] = a
            print(f"    {LABELS[s]:18s} RMS MAP error {np.sqrt(np.mean(a['err_map'] ** 2)):.1f} m "
                  f"(median {np.median(a['err_map']):.1f}, max {a['err_map'].max():.1f}, "
                  f"<= 10 m in {int(np.sum(a['err_map'] <= EPS_TARGET))}/{N_TRIALS}), "
                  f"posterior-mean RMS {np.sqrt(np.mean(a['err_mean'] ** 2)):.1f} m, "
                  f"measurements {a['n_meas'].mean():.1f} +- {a['n_meas'].std():.1f}, "
                  f"converged {int(a['converged'].sum())}/{N_TRIALS}, "
                  f"path {a['path_len'].mean():.0f} +- {a['path_len'].std():.0f} m, "
                  f"time {a['time'].mean():.0f} s")
        g = 0.5 * (res['eig']['n_meas'].mean() + res['belief']['n_meas'].mean())
        gp = 0.5 * (res['eig']['path_len'].mean() + res['belief']['path_len'].mean())
        print(f"    lawnmower / greedy (mean of EIG and max belief): readings "
              f"{res['lawn']['n_meas'].mean() / g:.1f}x, path {res['lawn']['path_len'].mean() / gp:.1f}x")
        nm = np.array([res[s]['n_meas'] for s in STRATS])
        pl = np.array([res[s]['path_len'] for s in STRATS])
        fewest = np.argmin(nm, axis=0)
        ties = np.sum(np.sort(nm, axis=0)[0] == np.sort(nm, axis=0)[1])
        print('    fewest measurements: ' + ', '.join(
            f'{LABELS[s]} {int(np.sum((fewest == i) & (np.sort(nm, axis=0)[0] != np.sort(nm, axis=0)[1])))}'
            for i, s in enumerate(STRATS)) + f', ties {int(ties)}')
        spl = np.sort(pl, axis=0)
        strict = spl[0] < spl[1] - 1e-9
        print('    shortest path: ' + ', '.join(
            f'{LABELS[s]} {int(np.sum((np.argmin(pl, axis=0) == i) & strict))}'
            for i, s in enumerate(STRATS)) + f', ties {int(np.sum(~strict))}')
        for s in ('eig', 'belief'):
            runs = mc[(s, m)]
            worst = int(np.argmax([r['err_map'] for r in runs]))
            r = runs[worst]
            dmin = np.min(np.linalg.norm(r['path'] - sources[worst], axis=1))
            print(f"    worst {LABELS[s]} trial: source ({sources[worst][0]:.1f}, {sources[worst][1]:.1f}), "
                  f"MAP ({r['est'][0]:.0f}, {r['est'][1]:.0f}), error {r['err_map']:.1f} m, "
                  f"peak {r['pmax']:.3f} after {r['n_meas']} measurements, closest reading "
                  f"{dmin:.1f} m from the source; posterior on the cell containing the source "
                  f"{r['true_in_top']:.2e}")
        big = [int(np.sum(np.array([r['err_map'] for r in mc[(s, m)]]) > 2 * EPS_TARGET)) for s in STRATS]
        print('    trials with error > 20 m: ' + ', '.join(f'{LABELS[s]} {b}' for s, b in zip(STRATS, big)))
        print(f"    EIG uses fewer measurements than max belief in "
              f"{int(np.sum(nm[0] < nm[1]))}, equal in {int(np.sum(nm[0] == nm[1]))}, more in "
              f"{int(np.sum(nm[0] > nm[1]))} of {N_TRIALS}")
        H = {s: curve(mc[(s, m)], 'H') for s in STRATS}
        for kk in (1, 2, 3, 5):
            print(f'    mean entropy after {kk} measurement(s): ' + ', '.join(
                f'{LABELS[s]} {H[s][:, kk - 1].mean():.2f}' for s in STRATS))

    out = os.path.normpath(OUTPUT_DIR)
    os.makedirs(out, exist_ok=True)
    plot_belief_sequence(demo, demo10, out)
    plot_eig_map(demo10, out)
    plot_curves(mc, out)
    plot_dose_field(out)
    plot_paths(demo, demo10, mc, out)
    plot_dose_3d(demo10, out)
    save_animation(demo10, out)
