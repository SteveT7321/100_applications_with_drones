"""
S060 Multi-Drone Cooperative Meteorological Measurement
========================================================
Three drones sample the temperature field of a 600 x 600 x (20-500) m atmospheric volume.
Each drone flies a boustrophedon pattern through one altitude band (20-180, 180-340,
340-500 m; 6 levels x 8 rows). Bands are assigned with the Hungarian algorithm on
transit + path length. The ground station fits an exponential variogram to the noisy
samples and reconstructs the field with Ordinary Kriging; accuracy is measured on 200
held-out validation points.

Compared reconstructions (same samples and noise draws):
  * Ordinary Kriging, continuous sampling every 50 m along the path (main)
  * Ordinary Kriging, card sampling (row end points only, x = 0 and x = 600 m)
  * Inverse-distance weighting (12 nearest samples, power 2) on the main samples
  * Ordinary Kriging with the nugget fixed at the sensor noise variance (diagnostic)

Usage:
    conda activate drones
    python src/03_environmental_sar/s060_meteorological.py
"""

import os
for _v in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[_v] = '1'          # single-threaded BLAS: reproducible and fast on a shared machine
import itertools
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D              # noqa: F401
from scipy.optimize import linear_sum_assignment, least_squares
from scipy.spatial.distance import cdist, pdist
from scipy.linalg import lu_factor, lu_solve

# ── Parameters (card) ─────────────────────────────────────────────────────────
N_DRONES     = 3
X_RANGE      = 600.0
Y_RANGE      = 600.0
Z_MIN        = 20.0
Z_MAX        = 500.0
T0           = 288.0       # K
LAPSE_RATE   = 6.5e-3      # K/m — standard-atmosphere (environmental) lapse rate
A_TEMP       = 2.0         # K
H_DECAY      = 300.0       # m
SIGMA_SENSOR = 0.3         # K
N_Z_LEVELS   = 6
N_ROWS       = 8
N_VAL        = 200
N_LAGS       = 15
BAND_EDGES   = np.array([20.0, 180.0, 340.0, 500.0])
DRONE_STARTS = np.array([[0.0, 0.0, Z_MIN], [X_RANGE, 0.0, Z_MIN], [X_RANGE / 2, Y_RANGE, Z_MIN]])
NZ_SWEEP     = [3, 4, 6, 8, 10]

# ── Parameters (added) ────────────────────────────────────────────────────────
V_CRUISE     = 15.0        # m/s
ENDURANCE    = 3000.0      # s — 50 min battery
SAMPLE_DS    = 50.0        # m — one sample every 50 m of path (every 3.3 s at 15 m/s)
IDW_K        = 12
IDW_POWER    = 2.0
N_SEEDS      = 10          # sensor-noise seeds, main comparison
N_SEEDS_SWEEP = 3
VAL_SEED     = 42
SEED0        = 100
GRID_N       = (13, 13, 13)   # grid for the mean kriging variance
ANIM_DT      = 30.0        # s per GIF frame

COLORS = {0: '#2CA02C', 1: '#FF7F0E', 2: '#9467BD'}   # low / mid / high band (card)
METHOD_COLORS = {'ok': '#1F77B4', 'ok_card': '#7F7F7F', 'idw': '#D62728', 'ok_nug': '#17BECF'}
METHOD_LABELS = {'ok': 'OK, samples every 50 m', 'ok_card': 'OK, card samples (row ends)',
                 'idw': 'IDW (k = 12), samples every 50 m',
                 'ok_nug': 'OK, nugget fixed at sigma_s^2 (diagnostic)'}

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '03_environmental_sar', 's060_meteorological',
)


# ── Field and paths ───────────────────────────────────────────────────────────

def true_temperature(x, y, z):
    base = T0 - LAPSE_RATE * z
    perturb = A_TEMP * np.sin(2 * np.pi * x / X_RANGE) * np.cos(2 * np.pi * y / Y_RANGE)
    return base + perturb * np.exp(-z / H_DECAY)


def plan_band_waypoints(z_lo, z_hi, n_z, n_rows):
    """Card boustrophedon: every level restarts at y = 0; rows alternate direction."""
    wps = []
    for z in np.linspace(z_lo, z_hi, n_z):
        for ir, y in enumerate(np.linspace(0.0, Y_RANGE, n_rows)):
            xs = (0.0, X_RANGE) if ir % 2 == 0 else (X_RANGE, 0.0)
            for x in xs:
                wps.append([x, y, z])
    return np.array(wps)


def path_length(pts):
    return float(np.sum(np.linalg.norm(np.diff(pts, axis=0), axis=1)))


def sample_along(wps, ds=SAMPLE_DS):
    """Points every ds metres of arc length along the waypoint polyline, plus every waypoint
    (so the card's waypoint samples are a subset of these)."""
    seg = np.linalg.norm(np.diff(wps, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    q = np.union1d(np.round(np.arange(0.0, s[-1] + 1e-9, ds), 6), np.round(s, 6))
    q = q[np.concatenate([[True], np.diff(q) > 1e-3])]
    return np.column_stack([np.interp(q, s, wps[:, k]) for k in range(3)]), q


def assign_bands(all_wps):
    entries = np.array([w[0] for w in all_wps])
    lengths = np.array([path_length(w) for w in all_wps])
    C = cdist(DRONE_STARTS, entries) + lengths[None, :]
    rows, cols = linear_sum_assignment(C)
    perms = {p: float(sum(C[d, p[d]] for d in range(N_DRONES)))
             for p in itertools.permutations(range(N_DRONES))}
    return rows, cols, C, perms


def fly(start, wps, v=V_CRUISE, dt=1.0):
    """Constant-speed point-mass flight start -> waypoints; returns (t, pos) samples."""
    pts = np.vstack([start, wps])
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    t = np.arange(0.0, s[-1] / v + dt, dt)
    q = np.minimum(t * v, s[-1])
    pos = np.column_stack([np.interp(q, s, pts[:, k]) for k in range(3)])
    step = np.linalg.norm(np.diff(pos, axis=0), axis=1)
    assert np.all(step <= v * dt + 1e-6), 'speed limit exceeded'
    assert np.all(pos[:, 2] >= Z_MIN - 1e-9) and np.all(pos[:, 2] <= Z_MAX + 1e-9), 'left volume'
    return t, pos


# ── Variogram and kriging ─────────────────────────────────────────────────────

def exponential_variogram(h, C0, C1, a):
    return C0 + C1 * (1.0 - np.exp(-h / a))


def fit_variogram(pts, vals, n_lags=N_LAGS, fix_c0=None):
    d = pdist(pts)
    g = 0.5 * pdist(vals[:, None], 'sqeuclidean')
    edges = np.linspace(0, d.max() * 0.6, n_lags + 1)
    idx = np.digitize(d, edges) - 1
    lags, gam, cnt = [], [], []
    for k in range(n_lags):
        m = (idx == k) & (d > 0)
        if m.sum() < 2:
            continue
        lags.append(d[m].mean()); gam.append(g[m].mean()); cnt.append(int(m.sum()))
    lags, gam = np.array(lags), np.array(gam)
    if fix_c0 is not None:
        res = least_squares(lambda p: exponential_variogram(lags, fix_c0, *p) - gam,
                            x0=[np.var(vals), d.max() / 3],
                            bounds=([1e-6, 1.0], [np.inf, 20 * d.max()]))
        return (fix_c0, *res.x), lags, gam, np.array(cnt)
    res = least_squares(lambda p: exponential_variogram(lags, *p) - gam,
                        x0=[0.1, np.var(vals), d.max() / 3],
                        bounds=([1e-4, 1e-6, 1.0], [np.inf, np.inf, 20 * d.max()]))
    return tuple(res.x), lags, gam, np.array(cnt)


class OrdinaryKriging:
    """OK with the nugget treated as measurement noise: distinct samples get gamma(h) + C0
    even at h = 0, the diagonal is 0. Predictions estimate the noise-free field and the
    reported variance is the error variance of that estimate (lambda.gamma0 + mu - C0)."""

    def __init__(self, pts, vals, params):
        self.pts, self.vals, self.params = pts, vals, params
        n = len(vals)
        G = exponential_variogram(cdist(pts, pts), *params)
        np.fill_diagonal(G, 0.0)
        A = np.zeros((n + 1, n + 1))
        A[:n, :n] = G
        A[:n, n] = 1.0
        A[n, :n] = 1.0
        self.lu = lu_factor(A)
        self.n = n

    def predict(self, q, chunk=4000):
        C0 = self.params[0]
        z = np.zeros(len(q)); var = np.zeros(len(q))
        for i in range(0, len(q), chunk):
            g0 = exponential_variogram(cdist(self.pts, q[i:i + chunk]), *self.params)
            rhs = np.vstack([g0, np.ones((1, g0.shape[1]))])
            sol = lu_solve(self.lu, rhs)
            lam, mu = sol[:-1], sol[-1]
            z[i:i + chunk] = lam.T @ self.vals
            var[i:i + chunk] = np.sum(lam * g0, axis=0) + mu - C0
        assert np.allclose(lu_solve(self.lu, np.append(np.zeros(self.n), 1.0))[:-1].sum(), 1.0)
        return z, var


def idw_predict(pts, vals, q, k=IDW_K, p=IDW_POWER):
    d = cdist(q, pts)
    nn = np.argsort(d, axis=1)[:, :k]
    dn = np.take_along_axis(d, nn, axis=1)
    w = 1.0 / np.maximum(dn, 1e-9) ** p
    return np.sum(w * vals[nn], axis=1) / w.sum(axis=1)


# ── Simulation ────────────────────────────────────────────────────────────────

def build_survey(n_z=N_Z_LEVELS):
    all_wps = [plan_band_waypoints(BAND_EDGES[b], BAND_EDGES[b + 1], n_z, N_ROWS)
               for b in range(N_DRONES)]
    rows, cols, C, perms = assign_bands(all_wps)
    drones = []
    for d, b in zip(rows, cols):
        wps = all_wps[b]
        samp, _ = sample_along(wps)
        transit = float(np.linalg.norm(wps[0] - DRONE_STARTS[d]))
        L = path_length(wps)
        drones.append({'drone': d, 'band': b, 'wps': wps, 'samples': samp, 'card_samples': wps,
                       'transit': transit, 'L': L, 'T': (transit + L) / V_CRUISE})
    return drones, C, perms, (rows, cols)


def reconstruct(drones, noise_rng, val_pts, val_true, grid_pts=None, methods=('ok', 'ok_card', 'idw', 'ok_nug')):
    pts = np.vstack([d['samples'] for d in drones])
    vals = true_temperature(*pts.T) + noise_rng.normal(0, SIGMA_SENSOR, len(pts))
    out = {'n_samples': len(pts)}
    if 'ok' in methods:
        params, lags, gam, cnt = fit_variogram(pts, vals)
        ok = OrdinaryKriging(pts, vals, params)
        zh, var = ok.predict(val_pts)
        out['ok'] = {'rmse': np.sqrt(np.mean((zh - val_true) ** 2)), 'pred': zh, 'var': var,
                     'params': params, 'lags': lags, 'gam': gam,
                     'r2': 1 - np.sum((zh - val_true) ** 2) / np.sum((val_true - val_true.mean()) ** 2)}
        out['ok']['model'] = ok
        if grid_pts is not None:
            out['ok']['grid_var'] = ok.predict(grid_pts)[1].mean()
        out['pts'], out['vals'] = pts, vals
    if 'ok_nug' in methods:
        params_n, *_ = fit_variogram(pts, vals, fix_c0=SIGMA_SENSOR ** 2)
        zn, _ = OrdinaryKriging(pts, vals, params_n).predict(val_pts)
        out['ok_nug'] = {'rmse': np.sqrt(np.mean((zn - val_true) ** 2)), 'pred': zn,
                         'params': params_n}
    if 'idw' in methods:
        zi = idw_predict(pts, vals, val_pts)
        out['idw'] = {'rmse': np.sqrt(np.mean((zi - val_true) ** 2)), 'pred': zi}
    if 'ok_card' in methods:
        cp = np.vstack([d['card_samples'] for d in drones])
        # card samples (waypoints) are a subset of the path samples: reuse the same noisy
        # readings so the comparison has common random numbers. Waypoints shared by two
        # bands (z = 180, 340 m) map to the first matching path sample.
        dmin = cdist(cp, pts)
        j = np.argmin(dmin, axis=1)
        assert np.all(dmin[np.arange(len(cp)), j] < 1e-6)
        j = np.unique(j)
        cv = vals[j]
        params_c, *_ = fit_variogram(pts[j], cv)
        okc = OrdinaryKriging(pts[j], cv, params_c)
        zc, _ = okc.predict(val_pts)
        out['ok_card'] = {'rmse': np.sqrt(np.mean((zc - val_true) ** 2)), 'pred': zc,
                          'params': params_c, 'n': len(j),
                          'perturb_at_samples': float(np.max(np.abs(
                              true_temperature(*pts[j].T) - (T0 - LAPSE_RATE * pts[j, 2]))))}
    return out


def grid_points():
    nx, ny, nz = GRID_N
    X, Y, Z = np.meshgrid(np.linspace(0, X_RANGE, nx), np.linspace(0, Y_RANGE, ny),
                          np.linspace(Z_MIN, Z_MAX, nz), indexing='ij')
    return np.column_stack([X.ravel(), Y.ravel(), Z.ravel()])


def run_simulation():
    rng_v = np.random.default_rng(VAL_SEED)
    val_pts = np.column_stack([rng_v.uniform(0, X_RANGE, N_VAL), rng_v.uniform(0, Y_RANGE, N_VAL),
                               rng_v.uniform(Z_MIN, Z_MAX, N_VAL)])
    val_true = true_temperature(*val_pts.T)
    grid = grid_points()

    drones, C, perms, assign = build_survey()
    for d in drones:
        assert d['T'] <= ENDURANCE, f"drone {d['drone']} needs {d['T']:.0f} s > endurance"
    main = [reconstruct(drones, np.random.default_rng(SEED0 + s), val_pts, val_true,
                        grid if s == 0 else None) for s in range(N_SEEDS)]

    sweep = {}
    for nz in NZ_SWEEP:
        dr, *_ = build_survey(nz)
        runs = [reconstruct(dr, np.random.default_rng(SEED0 + s), val_pts, val_true,
                            grid, methods=('ok',)) for s in range(N_SEEDS_SWEEP)]
        sweep[nz] = {'rmse': np.array([r['ok']['rmse'] for r in runs]),
                     'var': np.array([r['ok']['grid_var'] for r in runs]),
                     'n': runs[0]['n_samples'], 'T': max(d['T'] for d in dr)}
    return {'drones': drones, 'C': C, 'perms': perms, 'assign': assign, 'main': main,
            'sweep': sweep, 'val_pts': val_pts, 'val_true': val_true}


# ── Plots ─────────────────────────────────────────────────────────────────────

def _save(fig, out_dir, name):
    path = os.path.join(out_dir, name)
    fig.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def _box(ax):
    for x in (0, X_RANGE):
        for y in (0, Y_RANGE):
            ax.plot([x, x], [y, y], [Z_MIN, Z_MAX], color='grey', lw=0.6)
    for z in (Z_MIN, Z_MAX):
        ax.plot([0, X_RANGE, X_RANGE, 0, 0], [0, 0, Y_RANGE, Y_RANGE, 0], [z] * 5, color='grey', lw=0.6)


def plot_flight_paths(res, out_dir):
    fig = plt.figure(figsize=(9, 8))
    ax = fig.add_subplot(111, projection='3d')
    _box(ax)
    names = ['low (20-180 m)', 'mid (180-340 m)', 'high (340-500 m)']
    for d in res['drones']:
        c = COLORS[d['band']]
        p = np.vstack([DRONE_STARTS[d['drone']], d['wps']])
        ax.plot(p[:, 0], p[:, 1], p[:, 2], color=c, lw=0.9,
                label=f"drone {d['drone'] + 1} -> band {names[d['band']]}")
        s = d['samples']
        ax.scatter(s[:, 0], s[:, 1], s[:, 2], color=c, s=2)
        ax.scatter(*DRONE_STARTS[d['drone']], color=c, s=80, edgecolors='k', depthshade=False)
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)'); ax.set_zlabel('z (m)')
    ax.set_title(f"S060 flight paths and samples ({res['main'][0]['n_samples']} samples, "
                 f'one every {SAMPLE_DS:.0f} m)')
    ax.legend(fontsize=8, loc='upper left')
    ax.view_init(elev=22, azim=-60)
    _save(fig, out_dir, 'flight_paths_3d.png')


def plot_cost_matrix(res, out_dir):
    C = res['C']
    rows, cols = res['assign']
    fig, ax = plt.subplots(figsize=(6.5, 5.2))
    im = ax.imshow(C, cmap='viridis')
    for d in range(N_DRONES):
        for b in range(N_DRONES):
            sel = cols[list(rows).index(d)] == b
            ax.text(b, d, f'{C[d, b]:.0f}', ha='center', va='center',
                    color='w' if not sel else 'k', fontsize=10,
                    bbox=dict(fc='yellow', ec='k') if sel else None)
    perms = res['perms']
    opt = min(perms.values()); ident = perms[(0, 1, 2)]; worst = max(perms.values())
    ax.set_xticks(range(3), ['band 1 (low)', 'band 2 (mid)', 'band 3 (high)'])
    ax.set_yticks(range(3), [f'drone {i + 1}' for i in range(3)])
    fig.colorbar(im, ax=ax, label='$C_{db}$ = transit + band path (m)')
    ax.set_title(f'S060 Hungarian assignment (yellow)\noptimal {opt:.0f} m, identity {ident:.0f} m, '
                 f'worst {worst:.0f} m', fontsize=10)
    _save(fig, out_dir, 'cost_matrix.png')


def plot_variogram(res, out_dir):
    ok = res['main'][0]['ok']
    C0, C1, a = ok['params']
    h = np.linspace(0, ok['lags'].max() * 1.05, 300)
    fig, ax = plt.subplots(figsize=(8, 5))
    ax.plot(ok['lags'], ok['gam'], 'o', color='k', label='experimental')
    ax.plot(h, exponential_variogram(h, C0, C1, a), '-', color=METHOD_COLORS['ok'], label='fitted exponential')
    ax.axhline(C0, color='grey', ls='--', lw=1, label=f'nugget $C_0$ = {C0:.3f} K$^2$')
    ax.axhline(C0 + C1, color='grey', ls=':', lw=1, label=f'sill $C_0 + C_1$ = {C0 + C1:.2f} K$^2$')
    ax.axvline(a, color='grey', ls='-.', lw=1, label=f'range a = {a:.0f} m')
    ax.axhline(SIGMA_SENSOR ** 2, color='red', ls='--', lw=0.8,
               label=rf'sensor noise $\sigma_s^2$ = {SIGMA_SENSOR ** 2:.2f} K$^2$')
    ax.set_xlabel('lag h (m)'); ax.set_ylabel(r'$\gamma(h)$ (K$^2$)')
    ax.set_ylim(0, max(ok['gam'].max(), (C0 + C1) if C0 + C1 < 3 * ok['gam'].max() else 0) * 1.1)
    ax.set_title('S060 experimental variogram and exponential fit (seed 0)')
    ax.legend(fontsize=8)
    _save(fig, out_dir, 'variogram.png')


def _slices():
    n = 50
    xs = np.linspace(0, X_RANGE, n); zs = np.linspace(Z_MIN, Z_MAX, n)
    A, B = np.meshgrid(xs, zs)
    U, V = np.meshgrid(xs, xs)
    return {
        'x = 300 m': (np.column_stack([np.full(A.size, 300.0), A.ravel(), B.ravel()]), A, B, 'y (m)', 'z (m)'),
        'y = 300 m': (np.column_stack([A.ravel(), np.full(A.size, 300.0), B.ravel()]), A, B, 'x (m)', 'z (m)'),
        'z = 200 m': (np.column_stack([U.ravel(), V.ravel(), np.full(U.size, 200.0)]), U, V, 'x (m)', 'y (m)'),
    }


def plot_slices(res, out_dir):
    ok = res['main'][0]['ok']['model']
    sl = _slices()
    fig, axs = plt.subplots(3, 3, figsize=(15, 13))
    for j, (name, (q, A, B, xl, yl)) in enumerate(sl.items()):
        z, v = ok.predict(q)
        truth = true_temperature(*q.T)
        trend = T0 - LAPSE_RATE * q[:, 2]
        for i, (data, title, cmap) in enumerate([
                (z - trend, f'kriged T - lapse trend (K), {name}', 'RdBu_r'),
                (z - truth, f'kriging error (K), {name}', 'RdBu_r'),
                (v, f'kriging variance (K$^2$), {name}', 'magma')]):
            ax = axs[i, j]
            D = data.reshape(A.shape)
            if cmap == 'RdBu_r':
                lim = 2.0 if i == 0 else 0.6
                im = ax.pcolormesh(A, B, D, cmap=cmap, vmin=-lim, vmax=lim, shading='auto')
            else:
                im = ax.pcolormesh(A, B, D, cmap=cmap, vmin=0, vmax=0.15, shading='auto')
            fig.colorbar(im, ax=ax)
            ax.set_title(title, fontsize=9); ax.set_xlabel(xl); ax.set_ylabel(yl)
            if name != 'z = 200 m':
                for e in BAND_EDGES[1:-1]:
                    ax.axhline(e, color='k', ls=':', lw=0.8)
    fig.suptitle('S060 Ordinary Kriging on three orthogonal slices (seed 0). Row 1: estimate with the '
                 'lapse-rate trend removed; row 2: estimate - truth; row 3: kriging variance '
                 '(dotted: band boundaries)')
    _save(fig, out_dir, 'kriging_slices.png')


def plot_validation(res, out_dir):
    m = res['main'][0]
    vt = res['val_true']
    fig, axs = plt.subplots(1, 4, figsize=(21, 5.3))
    for ax, k in zip(axs, ['ok', 'ok_card', 'idw', 'ok_nug']):
        p = m[k]['pred']
        r2 = 1 - np.sum((p - vt) ** 2) / np.sum((vt - vt.mean()) ** 2)
        ax.plot(vt, p, 'o', ms=3, color=METHOD_COLORS[k])
        lo, hi = vt.min() - 0.5, vt.max() + 0.5
        ax.plot([lo, hi], [lo, hi], 'k--', lw=1)
        ax.set_xlabel('true T (K)'); ax.set_ylabel('predicted T (K)')
        ax.set_title(f"{METHOD_LABELS[k]}\nRMSE {m[k]['rmse']:.3f} K, R$^2$ = {r2:.4f}", fontsize=9)
        ax.set_aspect('equal')
    fig.suptitle(f'S060 validation at {N_VAL} held-out points (seed 0)')
    _save(fig, out_dir, 'validation_scatter.png')


def plot_rmse_vs_nz(res, out_dir):
    sw = res['sweep']
    nz = np.array(NZ_SWEEP)
    rm = np.array([sw[n]['rmse'].mean() for n in NZ_SWEEP]); rs = np.array([sw[n]['rmse'].std() for n in NZ_SWEEP])
    vm = np.array([sw[n]['var'].mean() for n in NZ_SWEEP])
    fig, ax = plt.subplots(figsize=(8, 5))
    ax.errorbar(nz, rm, rs, marker='o', color=METHOD_COLORS['ok'], capsize=3, label='validation RMSE (K)')
    ax.plot(nz, np.sqrt(vm), 's--', color='k', label=r'$\sqrt{\bar\sigma_K^2}$ on a 13$^3$ grid (K)')
    ax.axhline(0.5, color='red', ls=':', label='card target RMSE 0.5 K')
    ax.axvline(N_Z_LEVELS, color='grey', ls=':', label='operating point $N_z$ = 6')
    for n in NZ_SWEEP:
        flag = '' if sw[n]['T'] <= ENDURANCE else ' (> battery)'
        ax.annotate(f"{sw[n]['n']} samples\n{sw[n]['T'] / 60:.0f} min{flag}", (n, rm[NZ_SWEEP.index(n)]),
                    xytext=(4, 8), textcoords='offset points', fontsize=7)
    ax.set_xlabel('altitude levels per band $N_z$'); ax.set_ylabel('K')
    ax.set_ylim(0, None)
    ax.set_title(f'S060 reconstruction error vs sampling density ({N_SEEDS_SWEEP} noise seeds, mean ± std)')
    ax.legend(fontsize=8)
    _save(fig, out_dir, 'rmse_vs_nz.png')


def save_animation(res, out_dir):
    flights = []
    for d in res['drones']:
        t, pos = fly(DRONE_STARTS[d['drone']], d['wps'], dt=ANIM_DT)
        flights.append((d, t, pos))
    n = max(len(t) for _, t, _ in flights)
    pts, vals = res['main'][0]['pts'], res['main'][0]['vals']
    # time at which each sample is taken (samples are along each drone's own path)
    sample_t = []
    off = 0
    for d in res['drones']:
        k = len(d['samples'])
        _, s = sample_along(d['wps'])
        sample_t.append((d['transit'] + s) / V_CRUISE)
        off += k
    sample_t = np.concatenate(sample_t)
    step = 1
    n_frames = int(np.ceil((n - 1) / step)) + 1
    idx = [min(f * step, n - 1) for f in range(n_frames)]
    assert len(set(idx)) == n_frames
    fig = plt.figure(figsize=(7, 6.5))
    ax = fig.add_subplot(111, projection='3d')
    _box(ax)
    sc = ax.scatter([], [], [], c=[], cmap='coolwarm', vmin=vals.min(), vmax=vals.max(), s=4)
    dots = [ax.plot([], [], [], 'o', color=COLORS[d['band']], ms=7, mec='k')[0] for d, _, _ in flights]
    ax.set_xlim(0, X_RANGE); ax.set_ylim(0, Y_RANGE); ax.set_zlim(0, Z_MAX)
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)'); ax.set_zlabel('z (m)')
    ax.view_init(elev=20, azim=-60)
    fig.colorbar(sc, ax=ax, shrink=0.6, label='measured T (K)')
    title = ax.set_title('')

    def update(f):
        k = idx[f]
        t = k * ANIM_DT
        for (d, tt, pos), dot in zip(flights, dots):
            p = pos[min(k, len(tt) - 1)]
            dot.set_data([p[0]], [p[1]]); dot.set_3d_properties([p[2]])
        m = sample_t <= t
        sc._offsets3d = (pts[m, 0], pts[m, 1], pts[m, 2])
        sc.set_array(vals[m])
        title.set_text(f'S060 cooperative sounding  t = {t / 60:5.1f} min  samples {int(m.sum())}/{len(pts)}')
        return dots + [sc, title]

    anim = animation.FuncAnimation(fig, update, frames=n_frames, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    anim.save(path, writer='pillow', fps=10, dpi=70)
    plt.close(fig)
    print(f'Saved: {path} ({n_frames} frames, {ANIM_DT:.0f} s each)')


# ── Main ──────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    out = os.path.normpath(OUTPUT_DIR)
    os.makedirs(out, exist_ok=True)
    res = run_simulation()

    print('\n=== S060 Multi-Drone Cooperative Meteorological Measurement — Results ===')
    C = res['C']
    print('  Cost matrix C[d, b] (m):')
    for d in range(N_DRONES):
        print('    drone %d: ' % (d + 1) + ', '.join(f'{C[d, b]:.1f}' for b in range(N_DRONES)))
    rows, cols = res['assign']
    print('  Hungarian assignment: ' + ', '.join(f'drone {d + 1} -> band {b + 1}' for d, b in zip(rows, cols)))
    for p, v in sorted(res['perms'].items(), key=lambda kv: kv[1]):
        print(f'    permutation {tuple(int(i) + 1 for i in p)}: total {v:.1f} m')
    tot = sum(d['L'] for d in res['drones'])
    print(f'  Optimal - identity = {res["perms"][tuple(cols)] - res["perms"][(0, 1, 2)]:.1f} m; '
          f'worst - optimal = {max(res["perms"].values()) - min(res["perms"].values()):.1f} m '
          f'({100 * (max(res["perms"].values()) - min(res["perms"].values())) / tot:.2f}% of the '
          f'{tot:.0f} m of band paths)')
    for d in res['drones']:
        print(f"  drone {d['drone'] + 1} band {d['band'] + 1}: transit {d['transit']:.1f} m, band path "
              f"{d['L']:.0f} m, flight {d['T']:.0f} s = {d['T'] / 60:.1f} min at {V_CRUISE:.0f} m/s "
              f"(endurance {ENDURANCE / 60:.0f} min); {len(d['samples'])} samples "
              f"(card: {len(d['card_samples'])} waypoints)")
    m0 = res['main'][0]
    C0, C1, a = m0['ok']['params']
    print(f"  Variogram fit (seed 0): C0 = {C0:.4f} K^2 (sensor noise variance {SIGMA_SENSOR ** 2:.2f}), "
          f"C1 = {C1:.3f} K^2, range a = {a:.0f} m; samples {m0['n_samples']}")
    print(f"  Mean kriging variance on the 13^3 grid (seed 0): {m0['ok']['grid_var']:.4f} K^2 "
          f"(sqrt {np.sqrt(m0['ok']['grid_var']):.3f} K)")
    print(f"  Card sampling: {m0['ok_card']['n']} distinct points; max |field perturbation| at those "
          f"points = {m0['ok_card']['perturb_at_samples']:.2e} K (sin(2 pi x / 600) = 0 at x = 0, 600)")
    print(f'\n  --- {N_SEEDS} sensor-noise seeds (CRN), validation RMSE (K) ---')
    rm = {k: np.array([r[k]['rmse'] for r in res['main']]) for k in METHOD_LABELS}
    for k in METHOD_LABELS:
        extra = ''
        if k == 'ok':
            r2 = np.array([r['ok']['r2'] for r in res['main']])
            extra = f', R^2 {r2.mean():.4f}, below 0.5 K in {int(np.sum(rm[k] < 0.5))}/{N_SEEDS}'
        print(f'  [{METHOD_LABELS[k]}] {rm[k].mean():.4f} +/- {rm[k].std():.4f} '
              f'(min {rm[k].min():.4f}, max {rm[k].max():.4f}){extra}')
    print(f"  OK beats IDW in {int(np.sum(rm['ok'] < rm['idw']))}/{N_SEEDS} seeds; "
          f"OK beats card-sampling OK in {int(np.sum(rm['ok'] < rm['ok_card']))}/{N_SEEDS}; "
          f"fixed-nugget OK beats fitted OK in {int(np.sum(rm['ok_nug'] < rm['ok']))}/{N_SEEDS} "
          f"and IDW in {int(np.sum(rm['ok_nug'] < rm['idw']))}/{N_SEEDS}")
    pn = m0['ok_nug']['params']
    print(f'  Fixed-nugget fit (seed 0): C0 = {pn[0]:.4f}, C1 = {pn[1]:.3f} K^2, a = {pn[2]:.0f} m')
    vt = res['val_true']
    pert = vt - (T0 - LAPSE_RATE * res['val_pts'][:, 2])
    print(f'  RMS of the horizontal perturbation at the validation points: {np.sqrt(np.mean(pert ** 2)):.4f} K '
          f'(error of a pure lapse-rate profile)')
    # band-boundary variance check on the x = 300 m slice
    q = np.column_stack([np.full(len(np.arange(Z_MIN, Z_MAX + 1, 4)), 300.0), np.full(len(np.arange(Z_MIN, Z_MAX + 1, 4)), 300.0),
                         np.arange(Z_MIN, Z_MAX + 1, 4.0)])
    _, vz = m0['ok']['model'].predict(q)
    near = np.min(np.abs(q[:, 2, None] - BAND_EDGES[None, 1:-1]), axis=1) <= 8
    print(f'  Kriging variance along the vertical line x = y = 300 m: within 8 m of a band boundary '
          f'{vz[near].mean():.4f} K^2, elsewhere {vz[~near].mean():.4f} K^2')
    print(f'\n  --- N_z sweep ({N_SEEDS_SWEEP} seeds) ---')
    for n in NZ_SWEEP:
        s = res['sweep'][n]
        print(f"  N_z = {n:2d}: {s['n']} samples, longest flight {s['T']:.0f} s "
              f"({'within' if s['T'] <= ENDURANCE else 'EXCEEDS'} endurance), RMSE "
              f"{s['rmse'].mean():.4f} +/- {s['rmse'].std():.4f} K, mean kriging variance "
              f"{s['var'].mean():.4f} K^2")
    print()

    plot_flight_paths(res, out)
    plot_cost_matrix(res, out)
    plot_variogram(res, out)
    plot_slices(res, out)
    plot_validation(res, out)
    plot_rmse_vs_nz(res, out)
    save_animation(res, out)
