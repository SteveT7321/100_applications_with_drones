"""
S056 3D Upgrade — Radiation Hotspot Detection
=============================================
The S056 survey (500 x 500 m zone, inverse-square dose with air attenuation and
background, Gaussian counting noise sigma = sigma0 sqrt(D), sequential Bayes on a
grid, stop at posterior peak >= 0.90 or 80 readings, 5-cell cooldown, launch
reading at the zone centre) is extended to a 3D source and a free survey altitude:
  * the source sits anywhere in [0, 500]^2 x [0, 20] m (ground to rooftop) and is
    inferred on a 25 x 25 x 5 voxel posterior (20 x 20 x 4 m voxels);
  * the drone chooses its next waypoint from 625 horizontal cell centres x 5
    altitudes {3, 10, 20, 35, 60} m, flies straight 3D legs at 8 m/s and dwells 5 s;
  * the absorbed dose is integrated along every leg (true field) and the EIG score
    is penalised by lambda_D(k) * D(w, posterior mean) as on the card.

Strategies (same sources, same noise draws, same launch reading):
  1. eig3d   3D EIG over (x, y, z) waypoints                          (card 1)
  2. eigfix  same EIG and posterior, altitude locked at 15 m          (card 2)
  3. lawn    altitude-sweeping lawnmower: a 4 x 4 boustrophedon at each of the
             five altitudes, 60 m down to 3 m (80 readings)          (card 3)
  4. eig2d   2D baseline: the S056 model as written (ground source, 2D posterior
             over the same 25 x 25 cells, h = 15 m) run on the 3D sources (X5)
Ablations: eig3d with lambda0 = 0 and 0.5 (card 0.05).  Two detector noise levels
(card sigma0 = 0.04 and the 10x level the S056 review added).

Usage:
    conda activate drones
    python src/03_environmental_sar/3d/s056_3d_radiation.py
"""

import os
import heapq
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.colors import LogNorm
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401

_HERE = os.path.dirname(os.path.abspath(__file__))
OUTPUT_DIR = os.path.normpath(os.path.join(
    _HERE, '..', '..', '..', 'outputs', '03_environmental_sar', '3d', 's056_3d_radiation'))

# ── Parameters (card) ─────────────────────────────────────────────────────────
AREA_SIZE  = 500.0
NXY, NZ    = 25, 5
VOX_XY     = AREA_SIZE / NXY           # 20 m
Z_SRC_MAX  = 20.0
VOX_Z      = Z_SRC_MAX / NZ            # 4 m
Z_SET      = np.array([3.0, 10.0, 20.0, 35.0, 60.0])
H_FIX      = 15.0                      # original S056 altitude
V_DRONE    = 8.0
DT_INT     = 5.0
N_MEAS_MAX = 80
P_CONF     = 0.90
N_COOLDOWN = 5
A_SOURCE   = 2000.0                    # uSv m^2 / h
MU_ATTEN   = 2e-4
D_BG       = 0.10
SIGMA_0    = 0.04
D_MAX      = 2000.0                    # 2 mSv in uSv
LAMBDA_0   = 0.05
EPS_TARGET = 15.0                      # m, 3D
ISO_DOSE   = 1.0                       # uSv/h, corridor iso-surface

# ── Parameters (added) ────────────────────────────────────────────────────────
N_TRIALS   = 50                        # card 30; 50 as in the S056 2D Monte Carlo
SIGMA_MULT = [1.0, 10.0]
LAMBDA_ABL = [0.0, 0.5]
LAUNCH     = np.array([250.0, 250.0, H_FIX])
R_MIN      = 1.0                       # m, detector stand-off floor in the 1/r^2 law
SEG_DS     = 2.0                       # m, dose-integration step along a leg
LAWN_N     = 4                         # 4 x 4 lattice per altitude layer
DEMO_SOURCE = np.array([175.0, 310.0, 13.0])
N_BOOT     = 1000
SEED_BASE  = 56000

STRATS = ['eig3d', 'eigfix', 'lawn', 'eig2d', 'lawn15']
LABELS = {'eig3d': '3D EIG', 'eigfix': 'Fixed-altitude EIG (15 m)',
          'lawn': 'Altitude-sweep lawnmower', 'eig2d': '2D baseline (S056 model)',
          'lawn15': 'Lawnmower at 15 m (S056 8 x 10)',
          'eig3d_l0': '3D EIG, lambda0 = 0', 'eig3d_l0.5': '3D EIG, lambda0 = 0.5'}
COLORS = {'eig3d': '#D62728', 'eigfix': '#1F77B4', 'lawn': '#7F7F7F', 'eig2d': '#9467BD', 'lawn15': '#BCBD22'}

XS = (np.arange(NXY) + 0.5) * VOX_XY
ZS = (np.arange(NZ) + 0.5) * VOX_Z
_gx, _gy, _gz = np.meshgrid(XS, XS, ZS, indexing='ij')
VOX = np.stack([_gx.ravel(), _gy.ravel(), _gz.ravel()], axis=-1)          # (3125, 3)
_cx, _cy = np.meshgrid(XS, XS, indexing='ij')
XY = np.stack([_cx.ravel(), _cy.ravel()], axis=-1)                        # (625, 2)
CAND3 = np.array([[x, y, z] for (x, y) in XY for z in Z_SET])            # (3125, 3)
CAND15 = np.column_stack([XY, np.full(len(XY), H_FIX)])                  # (625, 3)
HYP2D = np.column_stack([XY, np.zeros(len(XY))])                         # ground sources


# ── Physics ───────────────────────────────────────────────────────────────────

def slant(p, s):
    return np.sqrt(np.sum((np.asarray(p, float) - np.asarray(s, float)) ** 2, axis=-1))


def dose_rate(p, s, A=A_SOURCE):
    r = np.maximum(slant(p, s), R_MIN)
    return D_BG + A / r ** 2 * np.exp(-MU_ATTEN * r)


def dz_gradient(rho, dz, A=A_SOURCE):
    """Card: dD/dz_d = -A (z_d - z_s)(2 + mu r) / r^4 exp(-mu r)."""
    r = np.sqrt(rho ** 2 + dz ** 2)
    return -A * dz * (2 + MU_ATTEN * r) / r ** 4 * np.exp(-MU_ATTEN * r)


MODELS = {}


def model(name):
    """Candidate set, hypothesis set and the precomputed dose matrix [candidate, hypothesis]."""
    if name not in MODELS:
        if name == '3d':
            cand, hyp = CAND3, VOX
        elif name == 'fix':
            cand, hyp = CAND15, VOX
        else:
            cand, hyp = CAND15, HYP2D
        D = dose_rate(cand[:, None, :], hyp[None, :, :])
        MODELS[name] = (cand, hyp, D, D ** 2)
    return MODELS[name]


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


def eig_map(b, D, D2, sigma0):
    zbar = D @ b
    var = np.maximum(D2 @ b - zbar ** 2, 0.0)
    return 0.5 * np.log1p(var / (sigma0 ** 2 * zbar))


def pick_max(score, cand, pos):
    """Deterministic argmax (S056 fix): relative 1e-9 ties broken by 3D distance, then index."""
    finite = np.isfinite(score)
    scale = np.abs(score[finite]).max()
    q = np.full(score.shape, -np.inf)
    q[finite] = np.round(score[finite] / (scale if scale > 0 else 1.0) * 1e9)
    c = np.flatnonzero(q == q.max())
    d = np.round(np.sum((cand[c] - pos) ** 2, axis=1), 6)
    return int(c[np.lexsort((c, d))[0]])


def leg_dose(p0, p1, src):
    """True absorbed dose (uSv) on a straight leg: dose rate sampled every <= 2 m (midpoint rule)."""
    L = float(np.linalg.norm(p1 - p0))
    if L == 0.0:
        return 0.0, 0.0
    n = max(1, int(np.ceil(L / SEG_DS)))
    t = (np.arange(n) + 0.5) / n
    pts = p0[None, :] + t[:, None] * (p1 - p0)[None, :]
    return float(np.mean(dose_rate(pts, src)) * L / V_DRONE / 3600.0), L


def lawn_points():
    xs = (np.arange(LAWN_N) + 0.5) * AREA_SIZE / LAWN_N
    pts = []
    for li, z in enumerate(Z_SET[::-1]):                         # 60 m down to 3 m
        layer = []
        for i, x in enumerate(xs):
            col = xs if i % 2 == 0 else xs[::-1]
            layer += [(x, y) for y in col]
        if li % 2 == 1:
            layer = layer[::-1]
        pts += [(x, y, z) for (x, y) in layer]
    pts = np.array(pts)
    idx = [int(np.argmin(np.sum((CAND3 - p) ** 2, axis=1))) for p in pts]   # snap to cells
    return np.array(idx)


LAWN = lawn_points()


def lawn15_points():
    """The S056 2D lawnmower: 8 columns x 10 rows flown boustrophedon at 15 m, snapped to cells."""
    xs = (np.arange(8) + 0.5) * AREA_SIZE / 8
    ys = (np.arange(10) + 0.5) * AREA_SIZE / 10
    pts = []
    for i, x in enumerate(xs):
        col = ys if i % 2 == 0 else ys[::-1]
        pts += [(x, y, H_FIX) for y in col]
    return np.array([int(np.argmin(np.sum((CAND15 - p) ** 2, axis=1))) for p in np.array(pts)])


LAWN15 = lawn15_points()


# ── Mission ───────────────────────────────────────────────────────────────────

def run_mission(strategy, src, noise, sigma0, lam0=LAMBDA_0, record=False):
    mname = {'eig3d': '3d', 'eigfix': 'fix', 'lawn': '3d', 'eig2d': '2d', 'lawn15': 'fix'}[strategy]
    cand, hyp, D, D2 = model(mname)
    pos = LAUNCH.copy()
    path = [pos.copy()]
    d_true = float(dose_rate(pos, src))
    n_floor = int(slant(pos, src) < R_MIN)
    z = d_true + sigma0 * np.sqrt(d_true) * noise[0]
    logb = bayes_update(np.zeros(len(hyp)), z, dose_rate(pos, hyp), sigma0)
    b = normalise(logb)
    dacc = d_true * DT_INT / 3600.0
    hist = {'H': [entropy(b)], 'err': [float(slant(hyp[int(np.argmax(b))], src))],
            'pmax': [b.max()], 'z': [z], 'dacc': [dacc], 'beliefs': [], 'eig': []}
    if record:
        hist['beliefs'].append(b.copy())
    recent, k, path_len, budget_stop = [], 0, 0.0, False
    while b.max() < P_CONF and len(path) < N_MEAS_MAX:
        if dacc >= D_MAX:
            budget_stop = True
            break
        if strategy in ('lawn', 'lawn15'):
            idx = int((LAWN if strategy == 'lawn' else LAWN15)[k])
        else:
            score = eig_map(b, D, D2, sigma0)
            if record:
                hist['eig'].append(score.copy())
            lam = lam0 * np.exp((dacc - D_MAX / 2) / (D_MAX / 4))
            if lam > 0:
                score = score - lam * dose_rate(cand, b @ hyp)
            if recent:
                score[recent] = -np.inf
            idx = pick_max(score, cand, pos)
            recent.append(idx)
            if len(recent) > N_COOLDOWN:
                recent.pop(0)
        new = cand[idx].copy()
        assert 0 <= new[0] <= AREA_SIZE and 0 <= new[1] <= AREA_SIZE and new[2] >= Z_SET[0] - 1e-9
        dl, L = leg_dose(pos, new, src)
        path_len += L
        pos = new
        path.append(pos.copy())
        k += 1
        d_true = float(dose_rate(pos, src))
        n_floor += int(slant(pos, src) < R_MIN)
        dacc += dl + d_true * DT_INT / 3600.0
        z = d_true + sigma0 * np.sqrt(d_true) * noise[k]
        logb = bayes_update(logb, z, D[idx], sigma0)
        b = normalise(logb)
        hist['H'].append(entropy(b))
        hist['pmax'].append(b.max())
        hist['z'].append(z)
        hist['dacc'].append(dacc)
        hist['err'].append(float(slant(hyp[int(np.argmax(b))], src)))
        if record:
            hist['beliefs'].append(b.copy())
    path = np.array(path)
    seg = np.linalg.norm(np.diff(path, axis=0), axis=1)
    assert abs(seg.sum() - path_len) < 1e-6                          # no teleport: path = flown legs
    assert np.all(path[:, 2] > 0) and np.isfinite(dacc) and dacc >= 0
    est = hyp[int(np.argmax(b))]
    true_vox = int(np.argmin(np.sum((hyp - src) ** 2, axis=1)))
    return {
        'n_meas': len(path), 'converged': bool(b.max() >= P_CONF), 'pmax': float(b.max()),
        'err3': float(slant(est, src)), 'errh': float(np.linalg.norm(est[:2] - src[:2])),
        'errz': float(abs(est[2] - src[2])), 'est': est, 'err_mean': float(slant(b @ hyp, src)),
        'path_len': float(path_len), 'time': float(path_len / V_DRONE + len(path) * DT_INT),
        'dacc': float(dacc), 'budget_stop': budget_stop, 'path': path, 'hist': hist, 'belief': b,
        'p_true': float(b[true_vox]), 'n_floor': n_floor,
        'n_alt': len(np.unique(np.round(path[1:, 2], 6))) if len(path) > 1 else 0,
    }


# ── Safe return-to-home corridor ─────────────────────────────────────────────

RTH_NODES = np.vstack([CAND3, LAUNCH[None, :]])
_li = {(round(x, 3), round(y, 3)): i for i, (x, y) in enumerate(XY)}


def _neighbours():
    nb = [[] for _ in range(len(RTH_NODES))]
    for i, (x, y) in enumerate(XY):
        for a in range(NZ):
            n = i * NZ + a
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    j = _li.get((round(x + dx * VOX_XY, 3), round(y + dy * VOX_XY, 3)))
                    if j is None:
                        continue
                    for da in (-1, 0, 1):
                        if 0 <= a + da < NZ and (dx, dy, da) != (0, 0, 0):
                            nb[n].append(j * NZ + a + da)
    home = len(RTH_NODES) - 1
    for a in range(NZ):                                     # launch connects to the 3 x 3 columns around it
        for (x, y) in [(250 + dx, 250 + dy) for dx in (-20, 0, 20) for dy in (-20, 0, 20)]:
            j = _li[(float(x), float(y))] * NZ + a
            nb[home].append(j)
            nb[j].append(home)
    return nb


RTH_NB = _neighbours()


def rth_paths(start, est, src):
    """Straight return vs. a corridor that never enters the estimated 1 uSv/h iso-surface except
    to leave it (inside it, a step may only go to a node of lower estimated dose)."""
    d_est = dose_rate(RTH_NODES, est)
    blocked = d_est > ISO_DOSE
    s = int(np.argmin(np.sum((CAND3 - start) ** 2, axis=1)))
    home = len(RTH_NODES) - 1
    dist = np.full(len(RTH_NODES), np.inf)
    prev = np.full(len(RTH_NODES), -1)
    dist[s] = 0.0
    pq = [(0.0, s)]
    while pq:
        d, u = heapq.heappop(pq)
        if d > dist[u]:
            continue
        if u == home:
            break
        for v in RTH_NB[u]:
            if blocked[v] and not (blocked[u] and d_est[v] < d_est[u]):
                continue
            nd = d + float(np.linalg.norm(RTH_NODES[v] - RTH_NODES[u]))
            if nd < dist[v] - 1e-12 or (abs(nd - dist[v]) <= 1e-12 and u < prev[v]):
                dist[v], prev[v] = nd, u
                heapq.heappush(pq, (nd, v))
    assert np.isfinite(dist[home]), 'corridor: home unreachable'
    nodes = [home]
    while nodes[-1] != s:
        nodes.append(prev[nodes[-1]])
    corr = np.vstack([start[None, :], RTH_NODES[nodes[::-1]]])
    straight = np.vstack([start[None, :], LAUNCH[None, :]])

    def dose_of(p):
        return sum(leg_dose(p[i], p[i + 1], src)[0] for i in range(len(p) - 1))

    def worst_after_exit(p, ref):
        fine = np.vstack([p[i] + t * (p[i + 1] - p[i]) for i in range(len(p) - 1)
                          for t in np.linspace(0, 1, 21)])
        de = dose_rate(fine, ref)
        out = np.flatnonzero(de <= ISO_DOSE)
        return float(de[out[0]:].max()) if len(out) else float(de.max())

    sf = np.vstack([straight[0] + t * (straight[1] - straight[0]) for t in np.linspace(0, 1, 201)])
    return {
        'corr': corr, 'straight': straight,
        'L_corr': float(np.sum(np.linalg.norm(np.diff(corr, axis=0), axis=1))),
        'L_straight': float(np.linalg.norm(LAUNCH - start)),
        'D_corr': dose_of(corr), 'D_straight': dose_of(straight),
        'crosses': bool(np.any(dose_rate(sf, est) > ISO_DOSE) and dose_rate(start, est) <= ISO_DOSE),
        'start_inside': bool(dose_rate(start, est) > ISO_DOSE),
        'max_est_after_exit': worst_after_exit(corr, est),
        'max_true_after_exit_straight': worst_after_exit(straight, src),
        'max_true_after_exit_corr': worst_after_exit(corr, src),
    }


# ── Simulation ────────────────────────────────────────────────────────────────

def trial_draw(t, on_grid=False):
    rng = np.random.default_rng(SEED_BASE + t)               # per-seed CRN
    src = np.array([rng.uniform(0, AREA_SIZE), rng.uniform(0, AREA_SIZE), rng.uniform(0, Z_SRC_MAX)])
    if on_grid:                                              # ablation: no voxel-model mismatch
        src = VOX[int(np.argmin(np.sum((VOX - src) ** 2, axis=1)))].copy()
    return src, rng.standard_normal(N_MEAS_MAX + 1)


def run_simulation():
    draws = [trial_draw(t) for t in range(N_TRIALS)]
    mc = {}
    for m in SIGMA_MULT:
        for s in STRATS:
            mc[(s, m)] = [run_mission(s, src, nz, SIGMA_0 * m) for (src, nz) in draws]
        for lam in LAMBDA_ABL:
            mc[(f'eig3d_l{lam:g}', m)] = [run_mission('eig3d', src, nz, SIGMA_0 * m, lam0=lam)
                                         for (src, nz) in draws]
    grid_draws = [trial_draw(t, on_grid=True) for t in range(N_TRIALS)]
    for m in SIGMA_MULT:
        for s in ('eig3d', 'eigfix'):
            mc[(s + '_grid', m)] = [run_mission(s, src, nz, SIGMA_0 * m) for (src, nz) in grid_draws]
    rth = {m: [rth_paths(r['path'][-1], r['est'], draws[t][0]) for t, r in enumerate(mc[('eig3d', m)])]
           for m in SIGMA_MULT}
    demo_noise = np.random.default_rng(42).standard_normal(N_MEAS_MAX + 1)
    demo = {(s, m): run_mission(s, DEMO_SOURCE, demo_noise, SIGMA_0 * m, record=True)
            for s in STRATS for m in SIGMA_MULT}
    demo_rth = rth_paths(demo[('eig3d', 10.0)]['path'][-1], demo[('eig3d', 10.0)]['est'], DEMO_SOURCE)
    return draws, mc, rth, demo, demo_rth


def curve(runs, key):
    out = np.empty((len(runs), N_MEAS_MAX))
    for i, r in enumerate(runs):
        v = np.array(r['hist'][key], dtype=float)
        out[i, :len(v)] = v[:N_MEAS_MAX]
        out[i, len(v):] = v[-1]
    return out


def rms_band(err, rng):
    rms = np.sqrt(np.mean(err ** 2, axis=0))
    boots = np.empty((N_BOOT,) + err.shape[1:])
    for b in range(N_BOOT):
        i = rng.integers(0, err.shape[0], err.shape[0])
        boots[b] = np.sqrt(np.mean(err[i] ** 2, axis=0))
    return rms, np.percentile(boots, 2.5, axis=0), np.percentile(boots, 97.5, axis=0)


def rms(v):
    return float(np.sqrt(np.mean(np.asarray(v) ** 2)))


# ── Plotting ──────────────────────────────────────────────────────────────────

def _save(fig, out_dir, name):
    path = os.path.join(out_dir, name)
    fig.savefig(path, dpi=120, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def _sphere(ax, c, r, color, alpha=0.15):
    u, v = np.meshgrid(np.linspace(0, 2 * np.pi, 30), np.linspace(0, np.pi, 16))
    x = c[0] + r * np.cos(u) * np.sin(v)
    y = c[1] + r * np.sin(u) * np.sin(v)
    zz = np.maximum(c[2] + r * np.cos(v), 0.0)
    ax.plot_wireframe(x, y, zz, color=color, alpha=alpha, lw=0.6)


R_ISO = None


def iso_radius():
    lo, hi = 1.0, 500.0
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        if D_BG + A_SOURCE / mid ** 2 * np.exp(-MU_ATTEN * mid) > ISO_DOSE:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def plot_trajectory_3d(demo, out_dir):
    fig = plt.figure(figsize=(15, 6.5))
    for j, m in enumerate(SIGMA_MULT):
        r = demo[('eig3d', m)]
        ax = fig.add_subplot(1, 2, j + 1, projection='3d')
        p = r['path']
        ax.plot(p[:, 0], p[:, 1], p[:, 2], '-', color='0.5', lw=0.8)
        sc = ax.scatter(p[:, 0], p[:, 1], p[:, 2], c=p[:, 2], cmap='viridis', vmin=0, vmax=60, s=18)
        _sphere(ax, DEMO_SOURCE, R_ISO, 'orange')
        ax.scatter(*DEMO_SOURCE, color='gold', s=120, edgecolor='k', label='true source')
        ax.scatter(*r['est'], color='red', s=80, edgecolor='k', label='MAP voxel')
        ax.scatter(*LAUNCH, color='k', marker='s', s=40, label='launch')
        ax.set_xlim(0, AREA_SIZE)
        ax.set_ylim(0, AREA_SIZE)
        ax.set_zlim(0, 65)
        ax.set_xlabel('x (m)')
        ax.set_ylabel('y (m)')
        ax.set_zlabel('z (m)')
        ax.view_init(elev=24, azim=-58)
        ax.set_title(f"σ0 = {SIGMA_0 * m:g}: {r['n_meas']} readings, 3D error {r['err3']:.1f} m\n"
                     f"orange mesh = true {ISO_DOSE:g} µSv/h iso-surface (r = {R_ISO:.1f} m)", fontsize=9)
        ax.legend(fontsize=7, loc='upper left')
    fig.colorbar(sc, ax=fig.axes, shrink=0.6, label='waypoint altitude (m)')
    fig.suptitle('S056 3D EIG survey, demo source (175, 310, 13) m')
    _save(fig, out_dir, 'trajectory_3d.png')


def plot_altitude(demo, mc, out_dir):
    fig, axes = plt.subplots(1, 3, figsize=(17, 4.8))
    for m, ls in zip(SIGMA_MULT, ['-', '--']):
        r = demo[('eig3d', m)]
        p = r['path']
        seg = np.linalg.norm(np.diff(p, axis=0), axis=1)
        t_arr = np.concatenate([[0], np.cumsum(seg / V_DRONE + DT_INT)])
        tt, zz = [], []
        for i in range(len(p)):                              # dwell then fly
            tt += [t_arr[i], t_arr[i] + DT_INT]
            zz += [p[i, 2], p[i, 2]]
        axes[0].plot(tt, zz, ls, color='#D62728', label=f'σ0 = {SIGMA_0 * m:g}')
        axes[0].plot(t_arr, p[:, 2], 'o', color='#D62728', ms=3)
    axes[0].set_xlabel('mission time (s)')
    axes[0].set_ylabel('altitude (m)')
    axes[0].set_title('Demo, 3D EIG: altitude vs time (dots = readings)', fontsize=10)
    axes[0].legend(fontsize=8)
    axes[0].grid(alpha=0.3)
    for ax, m in zip(axes[1:], SIGMA_MULT):
        w = 0.38
        for j, s in enumerate(['eig3d', 'lawn']):
            alts = np.concatenate([r['path'][1:, 2] for r in mc[(s, m)]])
            cnt = [int(np.sum(np.isclose(alts, z))) for z in Z_SET]
            ax.bar(np.arange(len(Z_SET)) + (j - 0.5) * w, np.array(cnt) / N_TRIALS, w,
                   color=COLORS[s], label=LABELS[s])
        ax.set_xticks(range(len(Z_SET)))
        ax.set_xticklabels([f'{z:g} m' for z in Z_SET])
        ax.set_ylabel('readings per mission')
        ax.set_title(f'Altitude use, {N_TRIALS} trials, σ0 = {SIGMA_0 * m:g} (launch reading at 15 m excluded)',
                     fontsize=9)
        ax.legend(fontsize=8)
    _save(fig, out_dir, 'altitude_profile.png')


def plot_posterior_slices(demo, out_dir):
    r = demo[('eig3d', 10.0)]
    n = len(r['hist']['beliefs'])
    ks = sorted(set(min(k, n) for k in (1, 10, 30, 60)))
    iz = min(int(DEMO_SOURCE[2] // VOX_Z), NZ - 1)
    ix = min(int(DEMO_SOURCE[0] // VOX_XY), NXY - 1)
    fig, axes = plt.subplots(2, len(ks), figsize=(4.2 * len(ks), 7.6), squeeze=False)
    for c, k in enumerate(ks):
        b = r['hist']['beliefs'][k - 1].reshape(NXY, NXY, NZ)
        im = axes[0, c].imshow(np.maximum(b[:, :, iz].T, 1e-12), origin='lower',
                               extent=[0, AREA_SIZE, 0, AREA_SIZE], cmap='coolwarm',
                               norm=LogNorm(vmin=1e-8, vmax=1))
        axes[0, c].plot(*DEMO_SOURCE[:2], 'x', color='white', ms=10, mew=2.5)
        axes[0, c].plot(r['path'][:k, 0], r['path'][:k, 1], '-o', color='cyan', ms=2, lw=0.8)
        axes[0, c].set_title(f'k = {k}: z-slice at {ZS[iz]:g} m (source voxel)\n'
                             f'peak {b.max():.3f}, H = {entropy(b.ravel()):.2f} nats', fontsize=9)
        axes[1, c].imshow(np.maximum(b[ix, :, :].T, 1e-12), origin='lower', aspect='auto',
                          extent=[0, AREA_SIZE, 0, Z_SRC_MAX], cmap='coolwarm', norm=LogNorm(vmin=1e-8, vmax=1))
        axes[1, c].plot(DEMO_SOURCE[1], DEMO_SOURCE[2], 'x', color='white', ms=10, mew=2.5)
        axes[1, c].set_title(f'vertical slice x = {XS[ix]:g} m', fontsize=9)
        axes[1, c].set_xlabel('y (m)')
        axes[1, c].set_ylabel('z (m)')
    fig.colorbar(im, ax=axes, shrink=0.6, label='voxel posterior (log)')
    fig.suptitle('S056 3D voxel posterior, 3D EIG, demo source, σ0 = 0.4 (white x = true source)')
    _save(fig, out_dir, 'posterior_slices.png')


def plot_curves(mc, out_dir):
    rng = np.random.default_rng(7)
    fig, axes = plt.subplots(2, 2, figsize=(14, 9))
    k = np.arange(1, N_MEAS_MAX + 1)
    for row, m in enumerate(SIGMA_MULT):
        for s in STRATS:
            if s != 'eig2d':
                axes[row, 0].plot(k, curve(mc[(s, m)], 'H').mean(axis=0), color=COLORS[s], label=LABELS[s])
            err = curve(mc[(s, m)], 'err')
            rr, lo, hi = rms_band(err, rng)
            axes[row, 1].plot(k, rr, color=COLORS[s], label=LABELS[s] + (' (z = 0 assumed)' if s == 'eig2d' else ''))
            axes[row, 1].fill_between(k, lo, hi, color=COLORS[s], alpha=0.18)
        axes[row, 0].set_ylabel('mean voxel entropy H_k (nats)')
        axes[row, 0].set_title(f'Entropy (3125-voxel posteriors), σ0 = {SIGMA_0 * m:g}', fontsize=10)
        axes[row, 1].axhline(EPS_TARGET, color='red', ls='--', lw=1, label='15 m target')
        axes[row, 1].set_yscale('log')
        axes[row, 1].set_ylabel('RMS 3D MAP error (m)')
        axes[row, 1].set_title(f'3D localisation error, {N_TRIALS} trials, 95 % bootstrap band, '
                               f'σ0 = {SIGMA_0 * m:g}', fontsize=10)
        for ax in axes[row]:
            ax.set_xscale('log')
            ax.set_xlabel('reading index k (log)')
            ax.grid(alpha=0.3)
            ax.legend(fontsize=7)
    fig.suptitle('S056 3D Entropy and RMS error vs readings (values held after a mission stops)')
    plt.tight_layout()
    _save(fig, out_dir, 'entropy_error_curves.png')


def plot_dose(mc, out_dir):
    fig, axes = plt.subplots(1, 2, figsize=(13, 4.8))
    k = np.arange(1, N_MEAS_MAX + 1)
    for ax, m in zip(axes, SIGMA_MULT):
        for s in ['eig3d', 'eigfix', 'lawn']:
            D = curve(mc[(s, m)], 'dacc')
            ax.plot(k, D.mean(axis=0), color=COLORS[s], label=LABELS[s] + ' (mean)')
            ax.plot(k, D.max(axis=0), ':', color=COLORS[s], lw=1, label=LABELS[s] + ' (max)')
        ax.axhline(D_MAX, color='red', ls='--', label='D_max = 2 mSv')
        ax.set_yscale('log')
        ax.set_xlabel('reading index k')
        ax.set_ylabel('cumulative absorbed dose (µSv)')
        ax.set_title(f'Cumulative dose, σ0 = {SIGMA_0 * m:g}', fontsize=10)
        ax.grid(alpha=0.3)
        ax.legend(fontsize=7)
    fig.suptitle('S056 3D Cumulative dose: the 2 mSv budget is never approached')
    _save(fig, out_dir, 'dose_profile.png')


def plot_corridor(demo, demo_rth, out_dir):
    r = demo[('eig3d', 10.0)]
    fig = plt.figure(figsize=(9, 7))
    ax = fig.add_subplot(111, projection='3d')
    _sphere(ax, r['est'], R_ISO, 'orange', alpha=0.25)
    c, st = demo_rth['corr'], demo_rth['straight']
    ax.plot(st[:, 0], st[:, 1], st[:, 2], '--', color='k',
            label=f"straight return: {demo_rth['L_straight']:.0f} m, {1000 * demo_rth['D_straight']:.2f} nSv")
    ax.plot(c[:, 0], c[:, 1], c[:, 2], '-o', color='#2CA02C', ms=3,
            label=f"corridor return: {demo_rth['L_corr']:.0f} m, {1000 * demo_rth['D_corr']:.2f} nSv")
    ax.scatter(*DEMO_SOURCE, color='gold', s=120, edgecolor='k', label='true source')
    ax.scatter(*r['est'], color='red', s=60, edgecolor='k', label='MAP voxel (iso-surface centre)')
    ax.scatter(*LAUNCH, color='k', marker='s', s=40, label='home (launch)')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_zlabel('z (m)')
    ax.view_init(elev=30, azim=-70)
    ax.legend(fontsize=8)
    ax.set_title(f'S056 3D Return to home around the estimated {ISO_DOSE:g} µSv/h iso-surface\n'
                 f'(demo, σ0 = 0.4)', fontsize=10)
    _save(fig, out_dir, 'safe_corridor.png')


def save_animation(demo, out_dir):
    r = demo[('eig3d', 10.0)]
    beliefs = r['hist']['beliefs']
    n = len(beliefs)
    step = 1
    idx = [min(i * step, n - 1) for i in range(int(np.ceil((n - 1) / step)) + 1)]
    fig = plt.figure(figsize=(6.4, 5.6))
    ax = fig.add_subplot(111, projection='3d')

    def update(f):
        k = idx[f]
        ax.cla()
        b = beliefs[k]
        top = np.argsort(-b, kind='stable')[:300]
        a = np.clip(b[top] / b.max(), 0.03, 1.0)
        cols = np.zeros((len(top), 4))
        cols[:, 0] = 0.85
        cols[:, 1] = 0.15
        cols[:, 2] = 0.15
        cols[:, 3] = a
        ax.scatter(VOX[top, 0], VOX[top, 1], VOX[top, 2], c=cols, s=30, marker='s')
        p = r['path'][:k + 1]
        ax.plot(p[:, 0], p[:, 1], p[:, 2], '-o', color='#1F77B4', ms=2, lw=0.8)
        ax.scatter(*DEMO_SOURCE, color='gold', s=80, edgecolor='k')
        ax.set_xlim(0, AREA_SIZE)
        ax.set_ylim(0, AREA_SIZE)
        ax.set_zlim(0, 60)
        ax.set_xticks([0, 250, 500])
        ax.set_yticks([0, 250, 500])
        ax.view_init(elev=25, azim=-60 + 4 * f)
        ax.set_title(f'S056 3D EIG (σ0 = 0.4), reading {k + 1}/{n}\n'
                     f'peak voxel {b.max():.3f}; red = top-300 voxels, opacity ∝ P', fontsize=9)
        return []

    anim = animation.FuncAnimation(fig, update, frames=len(idx), blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    anim.save(path, writer='pillow', fps=2, dpi=72)
    plt.close(fig)
    from PIL import Image
    with Image.open(path) as gif:
        n_file = gif.n_frames
    print(f'Saved: {path} ({len(idx)} frames rendered, {n_file} in the file, one per reading)')


# ── Report ────────────────────────────────────────────────────────────────────

def report(draws, mc, rth, demo, demo_rth):
    srcs = np.array([d[0] for d in draws])
    print('\n=== S056 3D Radiation Hotspot Detection — Results ===')
    print(f'  Voxels {len(VOX)}, candidate 3D waypoints {len(CAND3)}; H0 = ln({len(VOX)}) = {np.log(len(VOX)):.3f} nats')
    print(f'  {ISO_DOSE:g} uSv/h iso-surface radius: {R_ISO:.2f} m')
    for zd in Z_SET:
        print(f'  Dose directly above a ground source at {zd:g} m: {float(dose_rate([0, 0, zd], [0, 0, 0])):.2f} uSv/h; '
              f'20 m off-centre: {float(dose_rate([20, 0, zd], [0, 0, 0])):.3f} uSv/h')
    print(f"  Fixed 15 m mirror ambiguity: dose 20 m off-centre for z_s = 12 m {float(dose_rate([20, 0, 15], [0, 0, 12])):.6f}, "
          f"z_s = 18 m {float(dose_rate([20, 0, 15], [0, 0, 18])):.6f} uSv/h (identical)")
    print(f'  Card check dD/dz_d at z_d = z_s (rho = 20 m): {dz_gradient(20.0, 0.0):.3e}')
    dzs = np.linspace(0.01, 60, 6000)
    g = np.abs(dz_gradient(20.0, dzs))
    print(f'  |dD/dz_d| at rho = 20 m peaks at |z_d - z_s| = {dzs[np.argmax(g)]:.2f} m '
          f'(rho/sqrt(3) = {20 / np.sqrt(3):.2f} m), value {g.max():.3f} uSv/h/m')
    for m in SIGMA_MULT:
        sig = SIGMA_0 * m
        print(f'\n  ---- Monte Carlo, sigma0 = {sig:g} ({N_TRIALS} sources uniform in the volume, same draws) ----')
        res = {}
        for s in STRATS + [f'eig3d_l{l:g}' for l in LAMBDA_ABL]:
            runs = mc[(s, m)]
            a = {k: np.array([r[k] for r in runs], dtype=float)
                 for k in ('n_meas', 'converged', 'err3', 'errh', 'errz', 'err_mean', 'path_len',
                           'time', 'dacc', 'budget_stop', 'p_true', 'n_floor', 'n_alt')}
            res[s] = a
            print(f"    {LABELS[s]:28s} RMS 3D {rms(a['err3']):6.1f} m (median {np.median(a['err3']):.1f}, "
                  f"max {a['err3'].max():.1f}, <= 15 m {int(np.sum(a['err3'] <= EPS_TARGET))}/{N_TRIALS}) | "
                  f"RMS horiz {rms(a['errh']):.1f} m, RMS vert {rms(a['errz']):.1f} m | "
                  f"readings {a['n_meas'].mean():.1f} +- {a['n_meas'].std():.1f}, "
                  f"converged {int(a['converged'].sum())}/{N_TRIALS} | path {a['path_len'].mean():.0f} m, "
                  f"time {a['time'].mean():.0f} s | dose mean {a['dacc'].mean():.3f} max {a['dacc'].max():.3f} uSv, "
                  f"budget stops {int(a['budget_stop'].sum())} | readings with r < R_MIN {int(a['n_floor'].sum())}")
        for s in STRATS:
            if s in ('lawn', 'lawn15'):
                continue
            big = int(np.sum(res[s]['err3'] > 2 * EPS_TARGET))
            mean_ptrue = np.median(res[s]['p_true'])
            print(f"    {LABELS[s]:28s} gross errors (> 30 m) {big}; median posterior on the source's own "
                  f"cell {mean_ptrue:.2e}; distinct altitudes used (mean) {res[s]['n_alt'].mean():.2f}")
        # head-to-head
        for a, bname in [('eig3d', 'eigfix'), ('eig3d', 'lawn'), ('eigfix', 'eig2d'), ('lawn', 'lawn15')]:
            ea, eb = res[a]['err3'], res[bname]['err3']
            ha, hb = res[a]['errh'], res[bname]['errh']
            na, nb = res[a]['n_meas'], res[bname]['n_meas']
            print(f"    {LABELS[a]} vs {LABELS[bname]}: lower 3D error {int(np.sum(ea < eb - 1e-9))} / "
                  f"equal {int(np.sum(np.abs(ea - eb) <= 1e-9))} / higher {int(np.sum(ea > eb + 1e-9))}; "
                  f"lower horiz error {int(np.sum(ha < hb - 1e-9))} / equal {int(np.sum(np.abs(ha - hb) <= 1e-9))} / "
                  f"higher {int(np.sum(ha > hb + 1e-9))}; fewer readings {int(np.sum(na < nb))} / "
                  f"equal {int(np.sum(na == nb))} / more {int(np.sum(na > nb))}; "
                  f"RMS 3D ratio {rms(ea) / rms(eb):.2f}, mean-readings ratio {na.mean() / nb.mean():.2f}")
        # vertical mirror probe
        for s in ['eig3d', 'eigfix', 'lawn']:
            est_z = np.array([r['est'][2] for r in mc[(s, m)]])
            hz_ok = res[s]['errh'] <= VOX_XY / 2 * np.sqrt(2) + 1e-9
            mirror = hz_ok & (np.sign(est_z - H_FIX) != np.sign(srcs[:, 2] - H_FIX)) & (res[s]['errz'] > VOX_Z)
            print(f"    vertical probe {LABELS[s]:28s}: horizontal voxel correct {int(hz_ok.sum())}/{N_TRIALS}; "
                  f"of these, vertical error > 1 voxel {int(np.sum(hz_ok & (res[s]['errz'] > VOX_Z)))}, "
                  f"on the mirror side of 15 m {int(mirror.sum())}; RMS vert error (horiz-correct trials) "
                  f"{rms(res[s]['errz'][hz_ok]) if hz_ok.any() else float('nan'):.2f} m")
        for s in ('eig3d', 'eig3d_l0', 'eig3d_l0.5'):
            alts = np.concatenate([r['path'][1:, 2] for r in mc[(s, m)]])
            print(f"    altitude use {LABELS[s]:22s} (readings per mission after launch): " + ', '.join(
                f'{z:g} m {np.sum(np.isclose(alts, z)) / N_TRIALS:.2f}' for z in Z_SET)
                + f'; share at 3 m {np.mean(np.isclose(alts, Z_SET[0])):.3f}')
        dmax_all = max(r['dacc'] for key, runs in mc.items() if key[1] == m for r in runs)
        print(f'    largest mission dose (all strategies and ablations) {dmax_all:.3f} uSv = '
              f'{100 * dmax_all / D_MAX:.3f} % of D_max')
        g3 = {s: {k: np.array([r[k] for r in mc[(s + '_grid', m)]], dtype=float)
                  for k in ('err3', 'errh', 'errz', 'n_meas', 'converged', 'n_alt')} for s in ('eig3d', 'eigfix')}
        for s in ('eig3d', 'eigfix'):
            a = g3[s]
            print(f"    ON-GRID ablation (source at a voxel centre) {LABELS[s]:28s}: RMS 3D {rms(a['err3']):.1f} m, "
                  f"exact voxel {int(np.sum(a['err3'] < 1e-6))}/{N_TRIALS}, readings {a['n_meas'].mean():.1f} +- "
                  f"{a['n_meas'].std():.1f}, converged {int(a['converged'].sum())}/{N_TRIALS}, "
                  f"distinct altitudes {a['n_alt'].mean():.2f}")
        na, nb = g3['eig3d']['n_meas'], g3['eigfix']['n_meas']
        print(f"    ON-GRID 3D EIG vs fixed: fewer readings {int(np.sum(na < nb))} / equal {int(np.sum(na == nb))} / "
              f"more {int(np.sum(na > nb))}; mean-readings ratio {na.mean() / nb.mean():.2f}; lower 3D error "
              f"{int(np.sum(g3['eig3d']['err3'] < g3['eigfix']['err3'] - 1e-9))} / higher "
              f"{int(np.sum(g3['eig3d']['err3'] > g3['eigfix']['err3'] + 1e-9))}")
        Hs = {s: curve(mc[(s, m)], 'H') for s in ['eig3d', 'eigfix', 'lawn', 'lawn15']}
        for kk in (1, 2, 3, 5, 10):
            print(f'    mean voxel entropy after {kk} reading(s): ' + ', '.join(
                f'{LABELS[s]} {Hs[s][:, kk - 1].mean():.2f}' for s in Hs))
        R = rth[m]
        Lc = np.array([x['L_corr'] for x in R])
        Ls = np.array([x['L_straight'] for x in R])
        Dc = np.array([x['D_corr'] for x in R]) * 1000
        Ds = np.array([x['D_straight'] for x in R]) * 1000
        print(f"    RTH after 3D EIG: start inside est. iso-surface {sum(x['start_inside'] for x in R)}/{N_TRIALS}, "
              f"straight line crosses it (start outside) {sum(x['crosses'] for x in R)}/{N_TRIALS}; "
              f"length straight {Ls.mean():.0f} m vs corridor {Lc.mean():.0f} m (ratio {Lc.mean() / Ls.mean():.2f}); "
              f"true dose straight {Ds.mean():.2f} nSv vs corridor {Dc.mean():.2f} nSv (mean), corridor lower in "
              f"{int(np.sum(Dc < Ds - 1e-9))}/{N_TRIALS}; max est. dose on corridor after exit "
              f"{max(x['max_est_after_exit'] for x in R):.3f} uSv/h; max true dose after exit straight "
              f"{max(x['max_true_after_exit_straight'] for x in R):.3f} / corridor "
              f"{max(x['max_true_after_exit_corr'] for x in R):.3f} uSv/h")
    print('\n  ---- Demo source (175, 310, 13) m ----')
    for m in SIGMA_MULT:
        for s in STRATS:
            r = demo[(s, m)]
            print(f"    sigma0 = {SIGMA_0 * m:g} {LABELS[s]:28s} {r['n_meas']:2d} readings, peak {r['pmax']:.3f}, "
                  f"MAP ({r['est'][0]:.0f}, {r['est'][1]:.0f}, {r['est'][2]:.0f}), 3D error {r['err3']:.1f} m, "
                  f"path {r['path_len']:.0f} m, dose {r['dacc']:.3f} uSv, altitudes "
                  f"{sorted(set(np.round(r['path'][1:, 2], 1).tolist()))}")
    print(f"    RTH demo (sigma0 = 0.4): straight {demo_rth['L_straight']:.0f} m / {1000 * demo_rth['D_straight']:.2f} nSv, "
          f"corridor {demo_rth['L_corr']:.0f} m / {1000 * demo_rth['D_corr']:.2f} nSv, "
          f"start inside iso-surface {demo_rth['start_inside']}")


if __name__ == '__main__':
    import time
    t0 = time.time()
    R_ISO = iso_radius()
    draws, mc, rth, demo, demo_rth = run_simulation()
    report(draws, mc, rth, demo, demo_rth)
    out = OUTPUT_DIR
    os.makedirs(out, exist_ok=True)
    plot_trajectory_3d(demo, out)
    plot_altitude(demo, mc, out)
    plot_posterior_slices(demo, out)
    plot_curves(mc, out)
    plot_dose(mc, out)
    plot_corridor(demo, demo_rth, out)
    save_animation(demo, out)
    import sys
    print(f'[runtime {time.time() - t0:.0f} s]', file=sys.stderr)
