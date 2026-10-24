"""
S057 3D Upgrade — Wildlife Census
=================================
The S057 census (500 x 500 m reserve, 20 clustered groups, nadir camera of half-angle
25 deg, P_d = P_d0 exp(-alpha H) exp(-beta v) (1 - c_g), contiguous lawnmower strips
of width 2 h tan 25 deg, estimator N_raw / P_d(h, v, c_bar)) is moved onto a hilly
DEM (5 sin*cos harmonics, 30 m peak-to-valley).  Animals stand on the terrain.

Every pass is flown as a 3D trajectory at constant 3D speed v:
  * flat     the S057 reference: flat terrain, constant height h
  * baro     2D baseline (X5): the S057 plan flown at one barometric altitude that
             keeps h above the highest terrain point (max DEM + h)
  * tf       terrain following: z_cmd = DEM(x, y) + h, first-order lag tau_z = 1 s
             (card); tau_z = 0 and 3 s as a sensitivity sweep
An animal is seen on a pass when its horizontal cross-track offset is inside the
camera cone measured from the drone down to the animal's own elevation, H tan 25 deg,
H = z_drone - z_animal at overflight; its detection probability uses that H.
Footprints that are wider than the planned spacing overlap (double counts), narrower
ones leave gaps (misses).  Each k-th covering pass of an animal uses the k-th
detection uniform of that animal, shared by every strategy (CRN).

Estimators: the card's N_raw / P_d(h_nom, v, c_bar) and an added DEM-aware one,
N_raw / ((1 - c_bar) f_bar), with f_bar the area-mean of the summed per-pass P_d over
the reserve from the flown trajectory and the DEM.
A battery walk (S057: 30 min, 180 s reserve, 300 s swap, 12 m/s transit, here over
the terrain at max DEM + 10 m) gives sorties and total time.

Usage:
    conda activate drones
    python src/03_environmental_sar/3d/s057_3d_wildlife.py
"""

import os
import importlib.util
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401

_HERE = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location('s057_2d', os.path.join(_HERE, '..', 's057_wildlife.py'))
m2d = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(m2d)

OUTPUT_DIR = os.path.normpath(os.path.join(
    _HERE, '..', '..', '..', 'outputs', '03_environmental_sar', '3d', 's057_3d_wildlife'))

# ── Parameters (card / S057) ──────────────────────────────────────────────────
AREA_SIZE  = m2d.AREA_SIZE             # 500 m
TAN_FOV    = np.tan(np.radians(m2d.THETA_FOV))
STRATEGIES = m2d.STRATEGIES            # (40, 12), (20, 6), (8, 2)
SHORT      = ['High', 'Mid', 'Low']
DZ_RELIEF  = 30.0                      # m peak-to-valley
K_HARM     = 5
LAM_RANGE  = (80.0, 300.0)
DEM_N      = 101                       # nodes -> 100 x 100 cells of 5 m
TAU_Z      = 1.0                       # s
AGL_BOUNDS = (5.0, 50.0)
N_MONTE    = 50
RESERVE_SEED = m2d.RESERVE_SEED        # 42, N* = 229

# ── Parameters (added) ────────────────────────────────────────────────────────
DEM_SEED   = 57
TAU_SWEEP  = [0.0, 1.0, 3.0]
DS         = 0.1                       # m horizontal integration step
MC_SEED    = 57000                     # per-trial seeds MC_SEED + trial
KMAX       = 8                         # detection uniforms per animal (max passes covering it)
N_TERRAIN  = 30                        # random DEM + reserve pairs, 5 censuses each
N_PER_TERR = 5
TERR_SEED0 = 57100
GRID_DS    = 2.0                       # m, ground grid for the DEM-aware estimator
E_BATT, T_RESERVE, T_SWAP, V_TRANSIT = m2d.E_BATT, m2d.T_RESERVE, m2d.T_SWAP, m2d.V_TRANSIT
TRANSIT_CLEAR = 10.0                   # m above the highest terrain on transits
HOME_XY    = np.array([0.0, 0.0])
BATT_DS    = 2.0                       # m, battery check spacing (S057)
MODES      = ['flat', 'baro', 'tf']
MODE_LABEL = {'flat': 'Flat terrain (S057)', 'baro': 'Constant baro (2D plan)',
              'tf': 'Terrain-following'}
MODE_COLOR = {'flat': '#7F7F7F', 'baro': '#9467BD', 'tf': '#2CA02C'}


# ── Terrain ───────────────────────────────────────────────────────────────────

class DEM:
    def __init__(self, seed, flat=False):
        g = np.linspace(0, AREA_SIZE, DEM_N)
        self.g = g
        self.dx = g[1] - g[0]
        if flat:
            self.Z = np.zeros((DEM_N, DEM_N))
            self.raw_p2v = 0.0
            return
        rng = np.random.default_rng(seed)
        lx = rng.uniform(*LAM_RANGE, K_HARM)
        ly = rng.uniform(*LAM_RANGE, K_HARM)
        px = rng.uniform(0, 2 * np.pi, K_HARM)
        py = rng.uniform(0, 2 * np.pi, K_HARM)
        X, Y = np.meshgrid(g, g, indexing='ij')
        Z = DZ_RELIEF / K_HARM * sum(np.sin(2 * np.pi * X / lx[k] + px[k]) * np.cos(2 * np.pi * Y / ly[k] + py[k])
                                     for k in range(K_HARM))
        self.raw_p2v = float(Z.max() - Z.min())
        self.Z = (Z - Z.min()) * DZ_RELIEF / self.raw_p2v            # rescaled to 0..30 m

    def __call__(self, x, y):
        x = np.clip(np.asarray(x, float), 0, AREA_SIZE)
        y = np.clip(np.asarray(y, float), 0, AREA_SIZE)
        i = np.minimum((x / self.dx).astype(int), DEM_N - 2)
        j = np.minimum((y / self.dx).astype(int), DEM_N - 2)
        fx = x / self.dx - i
        fy = y / self.dx - j
        Z = self.Z
        return ((1 - fx) * (1 - fy) * Z[i, j] + fx * (1 - fy) * Z[i + 1, j]
                + (1 - fx) * fy * Z[i, j + 1] + fx * fy * Z[i + 1, j + 1])

    @property
    def zmax(self):
        return float(self.Z.max())


# ── Flight ────────────────────────────────────────────────────────────────────

def lawn_polyline(h):
    w, s, n, c = m2d.strip_layout(h)
    wps, kind = [], []
    for j, yc in enumerate(c):
        xs = (0.0, AREA_SIZE) if j % 2 == 0 else (AREA_SIZE, 0.0)
        wps += [(xs[0], yc), (xs[1], yc)]
    return np.array(wps), c, w


def fly(h, v, mode, tau, dem):
    """Integrate the pass in horizontal arc length with constant 3D speed v."""
    wps, centres, w = lawn_polyline(h)
    X, Y, leg = [], [], []
    for k in range(len(wps) - 1):
        a, b = wps[k], wps[k + 1]
        L = float(np.linalg.norm(b - a))
        m = max(1, int(round(L / DS)))
        t = np.arange(m) / m
        X.append(a[0] + t * (b[0] - a[0]))
        Y.append(a[1] + t * (b[1] - a[1]))
        leg.append(np.full(m, k // 2 if k % 2 == 0 else -1))      # survey legs carry the strip index
    X.append([wps[-1][0]]); Y.append([wps[-1][1]]); leg.append([len(centres) - 1])
    X = np.concatenate(X); Y = np.concatenate(Y); leg = np.concatenate(leg)
    terr = dem(X, Y)
    zc = terr + h
    n = len(X)
    dsv = np.hypot(np.diff(X), np.diff(Y))
    if mode == 'baro':
        Z = np.full(n, dem.zmax + h)
        dt = dsv / v
    elif mode == 'flat' or tau == 0.0:
        Z = zc.copy()
        g = np.diff(Z) / dsv
        dt = dsv * np.sqrt(1 + g ** 2) / v
    else:
        zc_l, ds_l = zc.tolist(), dsv.tolist()
        Zl = [zc_l[0]]
        dtl = []
        z = zc_l[0]
        tv = tau * v
        emax = 0.0
        for k in range(n - 1):
            e = (zc_l[k] - z) / tv
            ae = abs(e)
            if ae > emax:
                emax = ae
            g = e / (1.0 - e * e) ** 0.5
            z += g * ds_l[k]
            Zl.append(z)
            dtl.append(ds_l[k] * (1.0 + g * g) ** 0.5 / v)
        assert emax < 1.0, 'climb rate would exceed the airspeed'
        Z = np.array(Zl)
        dt = np.array(dtl)
    t = np.concatenate([[0.0], np.cumsum(dt)])
    agl = Z - terr
    assert np.all(agl > 0), 'drone below the terrain'
    seg3 = np.sqrt(dsv ** 2 + np.diff(Z) ** 2)
    assert np.all(seg3 / dt <= v * (1 + 1e-9)), 'speed above v'
    # time-weighted AGL deviation over the whole pass
    dev2 = 0.5 * ((agl[1:] - h) ** 2 + (agl[:-1] - h) ** 2)
    strips = []
    for j in range(len(centres)):
        sel = leg == j
        xs, zs = X[sel], Z[sel]
        o = np.argsort(xs, kind='stable')
        strips.append((xs[o], zs[o]))
    return {
        'X': X, 'Y': Y, 'Z': Z, 't': t, 'terr': terr, 'zc': zc, 'agl': agl, 'leg': leg,
        'centres': centres, 'w': w, 'strips': strips, 'h': h, 'v': v, 'mode': mode, 'tau': tau,
        'L2': float(dsv.sum()), 'L3': float(seg3.sum()), 'T': float(t[-1]),
        'sig_agl': float(np.sqrt(np.sum(dev2 * dt) / t[-1])),
        'agl_min': float(agl.min()), 'agl_max': float(agl.max()), 'agl_mean': float(np.sum(0.5 * (agl[1:] + agl[:-1]) * dt) / t[-1]),
        'frac_out_bounds': float(np.sum(dt * ((0.5 * (agl[1:] + agl[:-1]) < AGL_BOUNDS[0]) |
                                              (0.5 * (agl[1:] + agl[:-1]) > AGL_BOUNDS[1]))) / t[-1]),
    }


def coverage_factor(fl, dem):
    """Area mean of sum over passes of P_d0 exp(-alpha H) exp(-beta v), and of the multiplicity."""
    gx = np.arange(GRID_DS / 2, AREA_SIZE, GRID_DS)
    GX, GY = np.meshgrid(gx, gx, indexing='ij')
    GZ = dem(GX, GY)
    f = np.zeros_like(GX)
    mult = np.zeros_like(GX)
    for (xs, zs), c in zip(fl['strips'], fl['centres']):
        zd = np.interp(gx, xs, zs)[:, None]
        H = zd - GZ
        cov = np.abs(GY - c) <= H * TAN_FOV
        f += cov * m2d.P_D0 * np.exp(-m2d.ALPHA * H) * np.exp(-m2d.BETA * fl['v'])
        mult += cov
    gaps = float(np.mean(mult == 0))
    # card's delta_j between adjacent passes, evaluated per x and averaged
    deltas = []
    for j in range(len(fl['centres']) - 1):
        (x1, z1), (x2, z2) = fl['strips'][j], fl['strips'][j + 1]
        c1, c2 = fl['centres'][j], fl['centres'][j + 1]
        w1 = (np.interp(gx, x1, z1) - dem(gx, np.full_like(gx, c1))) * TAN_FOV
        w2 = (np.interp(gx, x2, z2) - dem(gx, np.full_like(gx, c2))) * TAN_FOV
        deltas.append(np.mean(np.maximum(0.0, w1 + w2 - fl['w']) / fl['w']))
    return float(f.mean()), float(mult.mean()), gaps, float(np.mean(deltas)) if deltas else 0.0


def census(fl, pos, zpos, gid, u, camo):
    n = len(pos)
    k = np.zeros(n, dtype=int)
    raw = 0
    det_any = np.zeros(n, dtype=bool)
    for (xs, zs), c in zip(fl['strips'], fl['centres']):
        H = np.interp(pos[:, 0], xs, zs) - zpos               # < 0 only far off-track (not in the cone)
        cov = np.abs(pos[:, 1] - c) <= H * TAN_FOV
        assert np.all(H[cov] > 0)
        if not cov.any():
            continue
        idx = np.flatnonzero(cov)
        assert np.all(k[idx] < KMAX), 'more covering passes than detection uniforms'
        pd = m2d.detection_prob(H[idx], fl['v'], camo[gid[idx]])
        d = u[idx, k[idx]] < pd
        raw += int(d.sum())
        det_any[idx[d]] = True
        k[idx] += 1
    return raw, int(det_any.sum()), int(np.sum(k == 0)), det_any, k


# ── Battery walk over the terrain ─────────────────────────────────────────────

def battery_walk(fl, dem):
    zT = dem.zmax + TRANSIT_CLEAR
    z_home = float(dem(*HOME_XY))
    step = max(1, int(round(BATT_DS / DS)))
    idx = np.arange(0, len(fl['X']), step)
    if idx[-1] != len(fl['X']) - 1:
        idx = np.append(idx, len(fl['X']) - 1)
    P = np.column_stack([fl['X'][idx], fl['Y'][idx], fl['Z'][idx]])
    T = fl['t'][idx]

    def transit_time(p, q):
        return (abs(zT - p[2]) + np.hypot(*(q[:2] - p[:2])) + abs(zT - q[2])) / V_TRANSIT

    home = np.array([HOME_XY[0], HOME_XY[1], z_home])
    t = transit_time(home, P[0])
    batt = E_BATT - t
    sorties, t_transit = 1, t
    for i in range(len(P) - 1):
        dts = T[i + 1] - T[i]
        need = dts + transit_time(P[i + 1], home) + T_RESERVE
        if batt < need:
            tr = transit_time(P[i], home)
            t += tr; batt -= tr; t_transit += tr
            assert batt >= T_RESERVE - 1e-6, 'landed below the battery reserve'
            t += T_SWAP
            batt = E_BATT
            sorties += 1
            tr = transit_time(home, P[i])
            t += tr; batt -= tr; t_transit += tr
        t += dts
        batt -= dts
        assert batt >= 0.0, 'flying on an empty battery'
    tr = transit_time(P[-1], home)
    t += tr; batt -= tr; t_transit += tr
    assert batt >= T_RESERVE - 1e-6, 'landed below the battery reserve'
    return {'sorties': sorties, 't_total': t, 't_transit': t_transit}


# ── Simulation ────────────────────────────────────────────────────────────────

def draw_trial(centroids, sizes, seed):
    rng = np.random.default_rng(seed)
    pos, gid, _ = m2d.draw_individuals(centroids, sizes, rng)
    u = rng.random((len(gid), KMAX))
    return pos, gid, u


def summarise(raws, uniq, gaps, n_true, denom):
    raws = np.asarray(raws, float)
    corr = raws / denom
    rel = (corr - n_true) / n_true
    return {'corr': corr, 'rel': rel, 'mean': corr.mean(), 'std': corr.std(),
            'bias': corr.mean() / n_true - 1, 'mae': float(np.mean(np.abs(rel))),
            'rmse': float(np.sqrt(np.mean(rel ** 2))), 'raw': raws.mean(),
            'miss': 1 - np.mean(uniq) / n_true, 'gap': np.mean(gaps) / n_true,
            'dbl': float(np.mean((raws - np.asarray(uniq)) / np.maximum(raws, 1)))}


def evaluate(fl, dem, trials, sizes, camo, c_bar):
    f_bar, k_bar, gap_area, delta = coverage_factor(fl, dem)
    pd_nom = m2d.detection_prob(fl['h'], fl['v'], c_bar)
    raws, uniq, gaps = [], [], []
    for (pos, gid, u) in trials:
        r, un, gp, _, _ = census(fl, pos, dem(pos[:, 0], pos[:, 1]), gid, u, camo)
        raws.append(r); uniq.append(un); gaps.append(gp)
    n_true = int(sizes.sum())
    return {'card': summarise(raws, uniq, gaps, n_true, pd_nom),
            'dem': summarise(raws, uniq, gaps, n_true, (1 - c_bar) * f_bar),
            'f_bar': f_bar, 'k_bar': k_bar, 'gap_area': gap_area, 'delta': delta, 'pd_nom': pd_nom,
            'pd_eff': (1 - c_bar) * f_bar / k_bar if k_bar > 0 else 0.0}


def run_simulation():
    dem = DEM(DEM_SEED)
    flat = DEM(0, flat=True)
    centroids, sizes, camo = m2d.generate_reserve(np.random.default_rng(RESERVE_SEED))
    c_bar = float(camo.mean())
    trials = [draw_trial(centroids, sizes, MC_SEED + t) for t in range(N_MONTE)]
    res = {}
    for (h, v, _), sh in zip(STRATEGIES, SHORT):
        for mode in MODES:
            taus = TAU_SWEEP if mode == 'tf' else [0.0]
            for tau in taus:
                d = flat if mode == 'flat' else dem
                fl = fly(h, v, mode, tau, d)
                ev = evaluate(fl, d, trials, sizes, camo, c_bar)
                ev['fl'] = fl
                ev['batt'] = battery_walk(fl, d)
                res[(sh, mode, tau)] = ev
    # multi-terrain Monte Carlo: new DEM and reserve per seed (CRN across strategies)
    terr = {(sh, m): {'card': [], 'dem': []} for sh in SHORT for m in ('baro', 'tf')}
    p2v_raw = []
    for k in range(N_TERRAIN):
        d = DEM(TERR_SEED0 + k)
        p2v_raw.append(d.raw_p2v)
        rng = np.random.default_rng(TERR_SEED0 + 1000 + k)
        cen, siz, cam = m2d.generate_reserve(rng)
        tri = [draw_trial(cen, siz, TERR_SEED0 + 2000 + 100 * k + q) for q in range(N_PER_TERR)]
        for (h, v, _), sh in zip(STRATEGIES, SHORT):
            for mode in ('baro', 'tf'):
                fl = fly(h, v, mode, TAU_Z if mode == 'tf' else 0.0, d)
                ev = evaluate(fl, d, tri, siz, cam, float(cam.mean()))
                terr[(sh, mode)]['card'] += list(ev['card']['rel'])
                terr[(sh, mode)]['dem'] += list(ev['dem']['rel'])
    terr = {k: {e: np.array(v) for e, v in val.items()} for k, val in terr.items()}
    reserve = {'centroids': centroids, 'sizes': sizes, 'camo': camo, 'c_bar': c_bar,
               'n_true': int(sizes.sum()), 'trial0': trials[0]}
    return dem, reserve, res, terr, np.array(p2v_raw)


# ── Plots ─────────────────────────────────────────────────────────────────────

def _save(fig, out_dir, name):
    path = os.path.join(out_dir, name)
    fig.savefig(path, dpi=120, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_terrain_3d(dem, reserve, res, out_dir):
    fig = plt.figure(figsize=(11, 8))
    ax = fig.add_subplot(111, projection='3d', computed_zorder=False)
    g = dem.g[::2]
    GX, GY = np.meshgrid(g, g, indexing='ij')
    ax.plot_surface(GX, GY, dem(GX, GY), cmap='terrain', alpha=0.65, linewidth=0, zorder=0)
    for mode, ls in [('tf', '-'), ('baro', '--')]:
        fl = res[('Mid', mode, TAU_Z if mode == 'tf' else 0.0)]['fl']
        s = slice(None, None, 40)
        ax.plot(fl['X'][s], fl['Y'][s], fl['Z'][s], ls, color='red' if mode == 'tf' else MODE_COLOR['baro'],
                lw=0.9, zorder=3, label=f"{MODE_LABEL[mode]}, mid (20 m, 6 m/s)")
    cen, siz = reserve['centroids'], reserve['sizes']
    ax.scatter(cen[:, 0], cen[:, 1], dem(cen[:, 0], cen[:, 1]) + 0.5, s=10 + 6 * siz, color='green',
               edgecolor='k', zorder=4, label='animal group centroids')
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)'); ax.set_zlabel('z (m)')
    ax.view_init(elev=32, azim=-60)
    ax.legend(fontsize=8, loc='upper left')
    ax.set_title(f'S057 3D reserve DEM (relief {DZ_RELIEF:.0f} m) and mid-altitude passes\n'
                 f'terrain-following (red) vs constant barometric altitude {dem.zmax + 20:.1f} m (purple)', fontsize=10)
    _save(fig, out_dir, 'terrain_trajectory_3d.png')


def plot_altitude(res, out_dir):
    fig, axes = plt.subplots(2, 3, figsize=(17, 7.5), sharex='col')
    for c, sh in enumerate(SHORT):
        tf = res[(sh, 'tf', TAU_Z)]['fl']
        ba = res[(sh, 'baro', 0.0)]['fl']
        sel = tf['leg'] == 0                                     # first survey pass
        x = tf['X'][sel]
        ax = axes[0, c]
        ax.plot(x, tf['terr'][sel], color='saddlebrown', label='terrain under the drone')
        ax.plot(x, tf['zc'][sel], 'k:', label='commanded z (terrain + h)')
        ax.plot(x, tf['Z'][sel], color='green', label=f'terrain-following, τ = {TAU_Z:g} s')
        ax.plot(x, ba['Z'][ba['leg'] == 0], color=MODE_COLOR['baro'], ls='--', label='constant baro')
        ax.set_ylabel('z (m)')
        ax.set_title(f"{sh}: h = {tf['h']:g} m, v = {tf['v']:g} m/s (pass 1)", fontsize=10)
        ax.legend(fontsize=7)
        ax = axes[1, c]
        ax.plot(x, tf['agl'][sel], color='green', label='AGL, terrain-following')
        ax.plot(x, ba['agl'][ba['leg'] == 0], color=MODE_COLOR['baro'], ls='--', label='AGL, constant baro')
        ax.axhline(tf['h'], color='k', ls=':', label='nominal h')
        ax.fill_between(x, tf['h'] - tf['sig_agl'], tf['h'] + tf['sig_agl'], color='green', alpha=0.2,
                        label=f"± σ_AGL = {tf['sig_agl']:.2f} m (whole mission)")
        ax.set_xlabel('x along pass (m)')
        ax.set_ylabel('AGL (m)')
        ax.legend(fontsize=7)
    fig.suptitle('S057 3D Altitude and AGL along the first pass')
    plt.tight_layout()
    _save(fig, out_dir, 'altitude_profile.png')


def plot_footprint(res, out_dir):
    fig, axes = plt.subplots(1, 3, figsize=(17, 4.6))
    for ax, sh in zip(axes, SHORT):
        for mode, tau in [('baro', 0.0), ('tf', 3.0), ('tf', 1.0), ('tf', 0.0)]:
            fl = res[(sh, mode, tau)]['fl']
            sel = fl['leg'] >= 0
            lab = MODE_LABEL[mode] + (f', τ = {tau:g} s' if mode == 'tf' else '')
            ax.plot(fl['t'][sel][::50] / 60, 2 * fl['agl'][sel][::50] * TAN_FOV, lw=0.6, label=lab,
                    color=MODE_COLOR[mode] if mode == 'baro' else {0.0: '#98DF8A', 1.0: '#2CA02C', 3.0: '#17BECF'}[tau])
        ax.axhline(res[(sh, 'tf', 1.0)]['fl']['w'], color='k', ls=':', label='planned strip width')
        ax.set_xlabel('mission time (min)')
        ax.set_ylabel('effective strip width 2 h(t) tan 25° (m)')
        ax.set_title(f'{sh}-altitude strategy', fontsize=10)
        ax.legend(fontsize=7)
    fig.suptitle('S057 3D Effective footprint width over the survey legs')
    _save(fig, out_dir, 'footprint_width.png')


def plot_counts(reserve, res, terr, out_dir):
    fig, axes = plt.subplots(1, 2, figsize=(15, 5.5))
    ax = axes[0]
    data, cols, labels = [], [], []
    for sh in SHORT:
        for mode, tau, est in [('flat', 0.0, 'card'), ('baro', 0.0, 'card'), ('baro', 0.0, 'dem'),
                               ('tf', TAU_Z, 'card')]:
            data.append(res[(sh, mode, tau)][est]['corr'])
            cols.append(MODE_COLOR[mode] if est == 'card' else '#C5B0D5')
            labels.append(f"{sh}\n{ {'flat': 'flat', 'baro': 'baro', 'tf': 'TF'}[mode] }{chr(10) + 'DEM-est' if est == 'dem' else ''}")
    bp = ax.boxplot(data, patch_artist=True, widths=0.6)
    for p, c in zip(bp['boxes'], cols):
        p.set_facecolor(c); p.set_alpha(0.7)
    ax.axhline(reserve['n_true'], color='red', ls='--', label=f"N* = {reserve['n_true']}")
    ax.set_xticks(range(1, len(labels) + 1), labels, fontsize=7)
    ax.set_yscale('log')
    ax.set_ylabel('corrected count (log)')
    ax.set_title(f'Card reserve on the hill DEM, {N_MONTE} CRN trials', fontsize=10)
    ax.legend(fontsize=8)
    ax = axes[1]
    data, cols, labels = [], [], []
    for sh in SHORT:
        for mode, est in [('baro', 'card'), ('baro', 'dem'), ('tf', 'card')]:
            data.append(100 * terr[(sh, mode)][est])
            cols.append(MODE_COLOR[mode] if est == 'card' else '#C5B0D5')
            labels.append(f"{sh}\n{'baro' if mode == 'baro' else 'TF'}{chr(10) + 'DEM-est' if est == 'dem' else ''}")
    bp = ax.boxplot(data, patch_artist=True, widths=0.6)
    for p, c in zip(bp['boxes'], cols):
        p.set_facecolor(c); p.set_alpha(0.7)
    ax.axhline(0, color='red', ls='--')
    ax.set_xticks(range(1, len(labels) + 1), labels, fontsize=7)
    ax.set_ylabel('relative error (%)')
    ax.set_title(f'{N_TERRAIN} random DEMs and reserves × {N_PER_TERR} censuses', fontsize=10)
    fig.suptitle('S057 3D Population estimates: flat S057, constant-baro 2D plan, terrain-following')
    _save(fig, out_dir, 'count_distribution.png')


def plot_penalty(res, out_dir):
    fig, axes = plt.subplots(1, 3, figsize=(16, 4.6))
    x = np.arange(len(SHORT))
    w = 0.25
    for i, (mode, tau) in enumerate([('baro', 0.0), ('tf', 0.0), ('tf', 1.0)]):
        lab = MODE_LABEL[mode] + (f', τ = {tau:g} s' if mode == 'tf' else '')
        col = MODE_COLOR[mode] if not (mode == 'tf' and tau == 0.0) else '#98DF8A'
        dL = [res[(sh, mode, tau)]['fl']['L3'] - res[(sh, 'flat', 0.0)]['fl']['L3'] for sh in SHORT]
        dT = [100 * (res[(sh, mode, tau)]['fl']['T'] / res[(sh, 'flat', 0.0)]['fl']['T'] - 1) for sh in SHORT]
        sg = [res[(sh, mode, tau)]['fl']['sig_agl'] for sh in SHORT]
        axes[0].bar(x + (i - 1) * w, dL, w, color=col, label=lab)
        axes[1].bar(x + (i - 1) * w, dT, w, color=col, label=lab)
        axes[2].bar(x + (i - 1) * w, sg, w, color=col, label=lab)
    for ax, yl in zip(axes, ['L_3D − L_flat (m)', 'mission-time increase vs flat (%)', 'σ_AGL (m)']):
        ax.set_xticks(x, SHORT)
        ax.set_ylabel(yl)
        ax.legend(fontsize=7)
        ax.grid(axis='y', alpha=0.3)
    axes[2].set_ylim(bottom=0)
    fig.suptitle('S057 3D Terrain penalty: extra path, extra time and AGL deviation')
    _save(fig, out_dir, 'terrain_penalty.png')


def save_animation(dem, reserve, res, out_dir):
    fl = res[('Mid', 'tf', TAU_Z)]['fl']
    pos, gid, u = reserve['trial0']
    zpos = dem(pos[:, 0], pos[:, 1])
    _, _, _, det, _ = census(fl, pos, zpos, gid, u, reserve['camo'])
    # detection time per animal: first covering pass in which it was detected
    t_det = np.full(len(pos), np.inf)
    k = np.zeros(len(pos), dtype=int)
    tt_strip = []
    for j, ((xs, zs), c) in enumerate(zip(fl['strips'], fl['centres'])):
        sel = fl['leg'] == j
        o = np.argsort(fl['X'][sel], kind='stable')
        tt = fl['t'][sel][o]
        tt_strip.append(tt)
        H = np.interp(pos[:, 0], xs, zs) - zpos
        cov = np.abs(pos[:, 1] - c) <= H * TAN_FOV
        idx = np.flatnonzero(cov)
        pd = m2d.detection_prob(H[idx], fl['v'], reserve['camo'][gid[idx]])
        d = u[idx, k[idx]] < pd
        tcross = np.interp(pos[idx, 0], xs, tt)
        new = idx[d]
        t_det[new] = np.minimum(t_det[new], tcross[d])
        k[idx] += 1
    n = len(fl['X'])
    step = 4000
    nf = int(np.ceil((n - 1) / step)) + 1
    frames = [min(i * step, n - 1) for i in range(nf)]
    fig = plt.figure(figsize=(7.5, 6))
    ax = fig.add_subplot(111, projection='3d', computed_zorder=False)
    g = dem.g[::4]
    GX, GY = np.meshgrid(g, g, indexing='ij')

    def update(f):
        i = frames[f]
        ax.cla()
        ax.plot_surface(GX, GY, dem(GX, GY), cmap='terrain', alpha=0.45, linewidth=0, zorder=0)
        ax.plot(fl['X'][:i + 1:20], fl['Y'][:i + 1:20], fl['Z'][:i + 1:20], color='red', lw=0.8, zorder=3)
        t = fl['t'][i]
        seen = t_det <= t
        ax.scatter(pos[~seen, 0], pos[~seen, 1], zpos[~seen] + 0.3, s=6, facecolors='none',
                   edgecolors='green', linewidths=0.5, zorder=4)
        ax.scatter(pos[seen, 0], pos[seen, 1], zpos[seen] + 0.3, s=8, color='green', zorder=5)
        ax.scatter(fl['X'][i], fl['Y'][i], fl['Z'][i], color='red', s=30, zorder=6)
        ax.set_xlim(0, AREA_SIZE); ax.set_ylim(0, AREA_SIZE); ax.set_zlim(0, 60)
        ax.view_init(elev=35, azim=-60)
        ax.set_title(f'S057 3D mid-altitude terrain-following census, t = {t / 60:.1f} min\n'
                     f'detected (filled) {int(seen.sum())} of {len(pos)} individuals', fontsize=9)
        return []

    anim = animation.FuncAnimation(fig, update, frames=nf, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    anim.save(path, writer='pillow', fps=6, dpi=70)
    plt.close(fig)
    from PIL import Image
    with Image.open(path) as gif:
        nfile = gif.n_frames
    print(f'Saved: {path} ({nf} frames rendered, {nfile} in the file, every {step} integration steps, '
          f'{n} steps in total; final detected {int(np.sum(np.isfinite(t_det)))} = census {int(det.sum())})')


# ── Report ────────────────────────────────────────────────────────────────────

def wins(a, b):
    ea, eb = np.abs(a), np.abs(b)
    return int(np.sum(ea < eb - 1e-12)), int(np.sum(np.abs(ea - eb) <= 1e-12)), int(np.sum(ea > eb + 1e-12))


def report(dem, reserve, res, terr, p2v_raw):
    n_true = reserve['n_true']
    print('\n=== S057 3D Wildlife Census — Results ===')
    print(f"  DEM seed {DEM_SEED}: raw harmonic sum peak-to-valley {dem.raw_p2v:.2f} m, rescaled to "
          f"{dem.Z.max() - dem.Z.min():.2f} m; max elevation {dem.zmax:.2f} m")
    gx = np.linspace(0, AREA_SIZE, 501)
    GX, GY = np.meshgrid(gx, gx, indexing='ij')
    Zf = dem(GX, GY)
    sx = np.gradient(Zf, gx, axis=0)
    sy = np.gradient(Zf, gx, axis=1)
    print(f'  Max slope along x {np.abs(sx).max():.3f} (= {np.degrees(np.arctan(np.abs(sx).max())):.1f} deg), '
          f'max gradient magnitude {np.hypot(sx, sy).max():.3f}')
    print(f'  Random DEMs ({N_TERRAIN}): raw peak-to-valley {p2v_raw.min():.1f} - {p2v_raw.max():.1f} m '
          f'(mean {p2v_raw.mean():.1f}); the card formula allows up to {2 * DZ_RELIEF:.0f} m')
    print(f"  Reserve: N* = {n_true}, c_bar = {reserve['c_bar']:.3f}; strip widths " +
          ' / '.join(f'{m2d.strip_width(h):.2f}' for (h, _, _) in STRATEGIES) + ' m')
    sz, cm = reserve['sizes'], reserve['camo']
    print(f"  Camouflage-weighting bias of the card estimator on this reserve (S057): "
          f"sum n_g (1 - c_g) / ((1 - c_bar) N*) - 1 = {100 * (np.sum(sz * (1 - cm)) / ((1 - cm.mean()) * n_true) - 1):+.1f} %; "
          f"population-weighted camouflage {np.sum(sz * cm) / n_true:.3f}")
    for (h, v, lab), sh in zip(STRATEGIES, SHORT):
        mean_level = float(dem.Z.mean()) + h
        print(f'  {sh}: a constant altitude of mean terrain + h = {mean_level:.1f} m would be below '
              f'terrain + 0 on {100 * np.mean(Zf >= mean_level):.1f} % of the reserve and below terrain + h/2 on '
              f'{100 * np.mean(Zf + h / 2 >= mean_level):.1f} %')
    print('\n  ---- Card reserve, hill DEM, 50 CRN trials ----')
    print('  strategy | mode | tau | passes | L_flat(2D plan) | L_3D | T (L_3D/v) | sorties | total time | '
          'sigma_AGL | AGL min/mean/max | out of 5-50 m | k_bar | gap area | card delta | P_d nominal | P_d effective')
    for sh in SHORT:
        for mode in MODES:
            for tau in (TAU_SWEEP if mode == 'tf' else [0.0]):
                r = res[(sh, mode, tau)]
                fl = r['fl']
                print(f"    {sh:4s} {mode:4s} {tau:3.0f} | {len(fl['centres']):3d} | {fl['L2']:7.0f} m | {fl['L3']:7.0f} m | "
                      f"{fl['T']:7.0f} s | {r['batt']['sorties']:2d} | {r['batt']['t_total']:7.0f} s | "
                      f"{fl['sig_agl']:7.3f} m | {fl['agl_min']:.2f}/{fl['agl_mean']:.2f}/{fl['agl_max']:.2f} m | "
                      f"{100 * fl['frac_out_bounds']:5.1f} % | {r['k_bar']:.3f} | {100 * r['gap_area']:.2f} % | "
                      f"{r['delta']:.4f} | {r['pd_nom']:.4f} | {r['pd_eff']:.4f}")
    print('\n  strategy | mode | tau | estimator | raw count | N_hat mean +- std | bias | per-survey mean |err| | '
          'RMS err | miss rate | gap rate (no coverage) | double-count rate')
    for sh in SHORT:
        for mode in MODES:
            for tau in (TAU_SWEEP if mode == 'tf' else [0.0]):
                for est in ('card', 'dem'):
                    s = res[(sh, mode, tau)][est]
                    print(f"    {sh:4s} {mode:4s} {tau:3.0f} {est:4s} | {s['raw']:6.1f} | {s['mean']:7.1f} +- {s['std']:6.1f} | "
                          f"{100 * s['bias']:+7.1f} % | {100 * s['mae']:6.1f} % | {100 * s['rmse']:6.1f} % | "
                          f"{100 * s['miss']:5.1f} % | {100 * s['gap']:5.2f} % | {100 * s['dbl']:5.2f} %")
    print('\n  Head-to-head per survey (|error| smaller / equal / larger), card reserve:')
    for sh in SHORT:
        tf = res[(sh, 'tf', TAU_Z)]
        ba = res[(sh, 'baro', 0.0)]
        fl = res[(sh, 'flat', 0.0)]
        print(f"    {sh}: TF(card est) vs baro(card est) {wins(tf['card']['rel'], ba['card']['rel'])}; "
              f"TF(card est) vs baro(DEM est) {wins(tf['card']['rel'], ba['dem']['rel'])}; "
              f"TF(card est) vs flat S057 {wins(tf['card']['rel'], fl['card']['rel'])}; "
              f"TF(DEM est) vs TF(card est) {wins(tf['dem']['rel'], tf['card']['rel'])}; "
              f"mean |err| ratio TF/baro-DEM {tf['card']['mae'] / ba['dem']['mae']:.2f}; "
              f"time ratio TF/baro {tf['fl']['T'] / ba['fl']['T']:.3f}, total-time ratio "
              f"{tf['batt']['t_total'] / ba['batt']['t_total']:.3f}")
    print('\n  Lag probe (terrain-following, card estimator):')
    for sh in SHORT:
        r0, r1, r3 = (res[(sh, 'tf', t)] for t in TAU_SWEEP)
        print(f"    {sh}: sigma_AGL tau 0/1/3 s = {r0['fl']['sig_agl']:.3f}/{r1['fl']['sig_agl']:.3f}/{r3['fl']['sig_agl']:.3f} m; "
              f"AGL min {r0['fl']['agl_min']:.2f}/{r1['fl']['agl_min']:.2f}/{r3['fl']['agl_min']:.2f} m; "
              f"bias {100 * r0['card']['bias']:+.1f}/{100 * r1['card']['bias']:+.1f}/{100 * r3['card']['bias']:+.1f} %; "
              f"mean |err| {100 * r0['card']['mae']:.1f}/{100 * r1['card']['mae']:.1f}/{100 * r3['card']['mae']:.1f} %; "
              f"gap area {100 * r0['gap_area']:.2f}/{100 * r1['gap_area']:.2f}/{100 * r3['gap_area']:.2f} %; "
              f"k_bar {r0['k_bar']:.3f}/{r1['k_bar']:.3f}/{r3['k_bar']:.3f}")
    print(f'\n  ---- {N_TERRAIN} random DEMs x {N_PER_TERR} censuses (new reserve per DEM, CRN across strategies) ----')
    for sh in SHORT:
        out = []
        for mode in ('baro', 'tf'):
            for est in ('card', 'dem'):
                r = terr[(sh, mode)][est]
                out.append(f"{mode}/{est}: bias {100 * r.mean():+.1f} % (std {100 * r.std():.1f}), "
                           f"mean |err| {100 * np.mean(np.abs(r)):.1f} %")
        w_ = wins(terr[(sh, 'tf')]['card'], terr[(sh, 'baro')]['dem'])
        w2 = wins(terr[(sh, 'tf')]['card'], terr[(sh, 'baro')]['card'])
        print(f'    {sh}: ' + '; '.join(out) + f'; TF(card) vs baro(DEM) wins {w_}; TF(card) vs baro(card) wins {w2}')


if __name__ == '__main__':
    import sys
    import time
    t0 = time.time()
    dem, reserve, res, terr, p2v_raw = run_simulation()
    report(dem, reserve, res, terr, p2v_raw)
    out = OUTPUT_DIR
    os.makedirs(out, exist_ok=True)
    plot_terrain_3d(dem, reserve, res, out)
    plot_altitude(res, out)
    plot_footprint(res, out)
    plot_counts(reserve, res, terr, out)
    plot_penalty(res, out)
    save_animation(dem, reserve, res, out)
    print(f'[runtime {time.time() - t0:.0f} s]', file=sys.stderr)
