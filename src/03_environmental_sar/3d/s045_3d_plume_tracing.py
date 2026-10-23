"""
S045 3D Upgrade — Chemical Plume Tracing
========================================
The S045 plume (Pasquill-Gifford class D Gaussian plume with ground reflection, wind
3 m/s along +x, Q = 1 g/s, concentrations in mg/m^3, Bernoulli intermittency with
p = 1 - exp(-C/2), additive noise sigma = 0.05, contact threshold 0.15) now comes from an
ELEVATED source at unknown height z_s in [0.5, 6] m, and the concentration depends on
the drone altitude:
    C = Q/(2 pi sy sz u) exp(-dy^2/2sy^2) [exp(-(z-z_s)^2/2sz^2) + exp(-(z+z_s)^2/2sz^2)]
Close to the source sigma_z is small (0.6 m at 10 m downwind), so a drone at the
wrong altitude loses the plume before it reaches the source.

Strategies (same sources, start, speeds, sensor seed per trial):
  2d    S045 cast-and-surge at a fixed 2 m altitude, 4-probe horizontal gradient
        (the 2D baseline lifted to 3D); z_s estimate = 2 m
  cs3d  3D cast-and-surge: 6-probe gradient, surge adds k_z g_z vertically,
        cast sweeps +-y (growing legs, S045) and +-z (0.4 m/s between 0.5 and 8 m);
        lost -> return to the last contact point in 3D; z_s estimate = final altitude
  pf3d  cs3d blended 50/50 with the direction to the 3D particle-filter posterior mean
        once the horizontal posterior std < 15 m (S045 rule); z_s estimate = posterior
        mean (the weighted-KDE mode is reported as well)
Capture = 3D distance to the source <= 3 m.

Usage:
    conda activate drones
    python src/03_environmental_sar/3d/s045_3d_plume_tracing.py
"""

import os
import importlib.util
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401
from matplotlib.animation import FuncAnimation, PillowWriter
from scipy.special import ndtr

_HERE = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location('s045_2d', os.path.join(_HERE, '..', 's045_plume_tracing.py'))
m2d = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(m2d)

OUTPUT_DIR = os.path.normpath(os.path.join(
    _HERE, '..', '..', '..', 'outputs', '03_environmental_sar', '3d', 's045_3d_plume_tracing'))

# ── Parameters ──────────────────────────────────────────────────────────────
WIND_SPEED  = m2d.WIND_SPEED     # 3 m/s along +x
Q_SOURCE    = m2d.Q_SOURCE       # 1 g/s
C_REF       = m2d.C_REF          # g/m^3 per normalised unit (mg/m^3)
SIGMA_NOISE = m2d.SIGMA_NOISE    # 0.05
C_THRESH    = m2d.C_THRESH       # 0.15 (S045 fix of the card's 0.02)
C_INTERMIT  = m2d.C_INTERMIT     # 2.0
V_SURGE     = m2d.V_SURGE        # 1.5 m/s
V_CAST      = m2d.V_CAST         # 0.8 m/s
V_Z_CAST    = 0.4                # m/s vertical cast speed
K_Z         = 0.5                # m/s altitude-hunting gain (card: 0.5 m step gain per second)
GRAD_DELTA  = m2d.GRAD_DELTA     # 1 m
N_PARTICLES = m2d.N_PARTICLES    # 500
PF_JITTER_XY = m2d.PF_JITTER     # 1.0 m (S045 fix)
PF_JITTER_Z = 0.1                # m (card)
PF_CONF_STD = m2d.PF_CONF_STD    # 15 m
R_CAPTURE   = m2d.R_CAPTURE      # 3 m (3D)
V_MAX       = m2d.V_MAX          # 2 m/s
TAU_V       = m2d.TAU_V          # 0.5 s
DT          = m2d.DT             # 0.1 s
T_MAX       = m2d.T_MAX          # 600 s
AREA        = m2d.AREA           # 200 m
Z_MIN, Z_MAX = 0.5, 8.0          # drone altitude band
ZS_RANGE    = (0.5, 6.0)         # source height prior / MC range
Z_FIXED     = 2.0                # 2D baseline altitude
POS_START   = np.array([170.0, 100.0, 2.0])   # downwind start (card's (20, 80) was upwind)
DEMO_SOURCE = np.array([55.0, 107.0, 4.5])
DEMO_SEED   = 999
SRC_X_RANGE = m2d.SRC_X_RANGE    # (40, 70)
SRC_Y_RANGE = m2d.SRC_Y_RANGE    # (60, 140)
N_TRIALS    = 40
ZS_TOL      = 0.5                # m — card's height-estimation target
PF_SNAP_T   = (0.0, 60.0, 180.0, 360.0)
UPWIND      = np.array([-1.0, 0.0, 0.0])
N_ANIM      = 120

STRATS = ['2d', 'cs3d', 'pf3d']
LABELS = {'2d': '2D baseline (z = 2 m)', 'cs3d': '3D cast-and-surge + altitude hunting',
          'pf3d': '3D PF-guided'}
COLORS = {'2d': 'tab:orange', 'cs3d': 'tab:blue', 'pf3d': 'tab:green'}


# ── Plume ───────────────────────────────────────────────────────────────────

def concentration(pos, src):
    """Mean concentration (mg/m^3) at pos (..., 3) from source(s) src (..., 3)."""
    pos = np.asarray(pos, float); src = np.asarray(src, float)
    dx = pos[..., 0] - src[..., 0]
    dy = pos[..., 1] - src[..., 1]
    z, zs = pos[..., 2], src[..., 2]
    x = np.maximum(dx, 1e-3)
    sy, sz = m2d.sigma_y(x), m2d.sigma_z(x)
    vert = np.exp(-(z - zs) ** 2 / (2 * sz ** 2)) + np.exp(-(z + zs) ** 2 / (2 * sz ** 2))
    c = Q_SOURCE / (2 * np.pi * sy * sz * WIND_SPEED) * np.exp(-dy ** 2 / (2 * sy ** 2)) * vert / C_REF
    return np.where(dx > 0, c, 0.0)


def measure(pos, src, rng):
    c = float(concentration(pos, src))
    xi = rng.random() < m2d.p_detect(c)
    return max(0.0, c * xi + rng.normal(0.0, SIGMA_NOISE))


def unit(v):
    return v / (np.linalg.norm(v) + 1e-9)


def gradient_3d(pos, src, rng, axes=3):
    """Normalised finite-difference gradient from 2*axes noisy probes (z probes kept >= 0 m)."""
    g = np.zeros(3)
    for k in range(axes):
        e = np.zeros(3); e[k] = GRAD_DELTA
        hi, lo = pos + e, pos - e
        if k == 2:
            lo[2] = max(lo[2], 0.0)
        g[k] = (measure(hi, src, rng) - measure(lo, src, rng)) / (hi[k] - lo[k])
    return unit(g)


# ── Navigation ──────────────────────────────────────────────────────────────

class Nav3D:
    """S045 cast-and-surge in xy (reused) + the card's vertical channel."""

    def __init__(self, vertical):
        self.h = m2d.CastSurgeNavigator()
        self.vertical = vertical
        self.zdir = 1.0
        self.last_z = None

    def command(self, pos, c, g):
        vh = self.h.command(pos[:2], c, unit(g[:2]) if np.linalg.norm(g[:2]) > 1e-9 else g[:2])
        state = self.h.state
        vz = 0.0
        if self.vertical:
            if state == 'surge':
                self.last_z = pos[2]
                vz = K_Z * g[2]
            elif state == 'lost' and self.last_z is not None:
                vz = np.clip(self.last_z - pos[2], -V_Z_CAST, V_Z_CAST)
            else:
                if pos[2] >= Z_MAX - 1e-6:
                    self.zdir = -1.0
                elif pos[2] <= Z_MIN + 1e-6:
                    self.zdir = 1.0
                vz = V_Z_CAST * self.zdir
        return np.array([vh[0], vh[1], vz]), state


class ParticleFilter3D:
    def __init__(self, rng):
        self.rng = rng
        lo = [0.0, 0.0, ZS_RANGE[0]]; hi = [AREA, AREA, ZS_RANGE[1]]
        self.p = rng.uniform(lo, hi, size=(N_PARTICLES, 3))
        self.logw = np.zeros(N_PARTICLES)
        self.n_resample = 0

    def weights(self):
        w = np.exp(self.logw - self.logw.max())
        return w / w.sum()

    def update(self, pos, z):
        c = concentration(pos[None, :], self.p)
        pd = m2d.p_detect(c)
        if z <= 0.0:
            lik = pd * ndtr(-c / SIGMA_NOISE) + (1 - pd) * 0.5
        else:
            phi = lambda u: np.exp(-0.5 * u ** 2) / (np.sqrt(2 * np.pi) * SIGMA_NOISE)
            lik = pd * phi((z - c) / SIGMA_NOISE) + (1 - pd) * phi(z / SIGMA_NOISE)
        self.logw += np.log(np.maximum(lik, 1e-300))
        w = self.weights()
        if 1.0 / np.sum(w ** 2) < N_PARTICLES / 2:
            u = (self.rng.random() + np.arange(N_PARTICLES)) / N_PARTICLES
            idx = np.minimum(np.searchsorted(np.cumsum(w), u), N_PARTICLES - 1)
            jit = self.rng.normal(0, 1, size=(N_PARTICLES, 3)) * [PF_JITTER_XY, PF_JITTER_XY, PF_JITTER_Z]
            self.p = self.p[idx] + jit
            self.p[:, :2] = np.clip(self.p[:, :2], 0, AREA)
            self.p[:, 2] = np.clip(self.p[:, 2], ZS_RANGE[0], ZS_RANGE[1])
            self.logw[:] = 0.0
            self.n_resample += 1

    def estimate(self):
        w = self.weights()
        m = w @ self.p
        sd_h = np.sqrt(np.sum(w * np.sum((self.p[:, :2] - m[:2]) ** 2, axis=1)))
        return m, sd_h

    def zs_kde(self, grid, bw=0.2):
        w = self.weights()
        k = np.exp(-0.5 * ((grid[:, None] - self.p[None, :, 2]) / bw) ** 2) @ w
        return k / (k.sum() * (grid[1] - grid[0]))


# ── Mission ─────────────────────────────────────────────────────────────────

def run_one(strategy, src, seed, record=False):
    rng = np.random.default_rng([seed, 1])
    rng_pf = np.random.default_rng([seed, 2])
    pos = POS_START.copy()
    vel = np.zeros(3)
    nav = Nav3D(vertical=(strategy != '2d'))
    pf = ParticleFilter3D(rng_pf) if strategy == 'pf3d' else None
    traj, concs, states, pf_est = [pos.copy()], [], [], []
    pf_snaps, frames = {}, []
    zgrid = np.linspace(0, 7, 141)
    n_steps = int(round(T_MAX / DT))
    found, t = False, T_MAX
    z_bounce = 0
    for step in range(n_steps):
        tt = step * DT
        z = measure(pos, src, rng)
        concs.append(z)
        g = gradient_3d(pos, src, rng, axes=2 if strategy == '2d' else 3)
        if pf is not None:
            if record and any(abs(tt - s) < DT / 2 for s in PF_SNAP_T):
                pf_snaps[round(tt)] = (pf.p.copy(), pf.weights(), pf.zs_kde(zgrid))
            pf.update(pos, z)
            m, sd = pf.estimate()
            pf_est.append(m)
        v_cmd, state = nav.command(pos, z, g)
        states.append(state)
        if pf is not None and pf.estimate()[1] < PF_CONF_STD:
            v_cmd = V_SURGE * unit(0.5 * unit(v_cmd) + 0.5 * unit(pf_est[-1] - pos))
        if record and step % 50 == 0:
            frames.append((tt, pos.copy(), None if pf is None else pf.p.copy()))
        vel = vel + (v_cmd - vel) / TAU_V * DT
        sp = np.linalg.norm(vel)
        if sp > V_MAX:
            vel *= V_MAX / sp
        pos = pos + vel * DT
        pos[:2] = np.clip(pos[:2], 0.0, AREA)
        if pos[2] < Z_MIN or pos[2] > Z_MAX:          # reflecting altitude bounds (card)
            pos[2] = np.clip(pos[2], Z_MIN, Z_MAX)
            vel[2] = -vel[2]
            z_bounce += 1
        if strategy == '2d':
            assert abs(pos[2] - Z_FIXED) < 1e-9
        assert Z_MIN - 1e-9 <= pos[2] <= Z_MAX + 1e-9 and np.linalg.norm(vel) <= V_MAX + 1e-9
        traj.append(pos.copy())
        if np.linalg.norm(pos - src) <= R_CAPTURE:
            found, t = True, (step + 1) * DT
            break
    traj = np.array(traj)
    if pf is not None:
        m, _ = pf.estimate()
        kde = pf.zs_kde(zgrid)
        zs_hat, zs_mode = m[2], zgrid[np.argmax(kde)]
        if record:
            pf_snaps['final'] = (pf.p.copy(), pf.weights(), kde)
    else:
        zs_hat = zs_mode = pos[2]
    d_h = np.linalg.norm(traj[:, :2] - src[:2], axis=1)
    return dict(traj=traj, conc=np.array(concs), states=states, pf_est=np.array(pf_est),
                pf_snaps=pf_snaps, frames=frames, found=found, time=t, src=src,
                path=float(np.sum(np.linalg.norm(np.diff(traj, axis=0), axis=1))),
                final_err=float(np.linalg.norm(traj[-1] - src)),
                min_dist=float(np.min(np.linalg.norm(traj - src, axis=1))),
                min_dist_h=float(d_h.min()), found_h=bool(d_h.min() <= R_CAPTURE),
                zs_hat=float(zs_hat), zs_mode=float(zs_mode), z_bounce=z_bounce,
                contact=float(np.mean(np.array(concs) >= C_THRESH)))


def run_simulation():
    print('=== Plume check (demo source z_s = %.1f m, centreline y = y_s) ===' % DEMO_SOURCE[2])
    for d in (5, 10, 20, 40, 115):
        p = DEMO_SOURCE + np.array([d, 0.0, 0.0])
        cs = [float(concentration(np.array([p[0], p[1], zz]), DEMO_SOURCE)) for zz in (Z_FIXED, DEMO_SOURCE[2])]
        zz = np.linspace(0, 8, 801)
        prof = concentration(np.column_stack([np.full(801, p[0]), np.full(801, p[1]), zz]), DEMO_SOURCE)
        print(f'  {d:4d} m downwind: sigma_z = {m2d.sigma_z(d):5.2f} m, C(z = 2 m) = {cs[0]:8.3f}, '
              f'C(z = z_s) = {cs[1]:8.3f}, peak altitude {zz[np.argmax(prof)]:.2f} m')
    print(f'  C(start) = {float(concentration(POS_START, DEMO_SOURCE)):.3f}; 2D baseline cannot capture when '
          f'|z_s - 2| > 3 m (z_s > 5 m: {100 * (ZS_RANGE[1] - 5.0) / (ZS_RANGE[1] - ZS_RANGE[0]):.0f} % of the z_s range)')
    print('=== Consistency check: 2D baseline with the source at the flight altitude (z_s = 2 m), '
          'S045 trials 0-19 ===')
    chk = []
    for trial in range(20):
        rs = np.random.default_rng([trial, 0])
        src = np.array([rs.uniform(*SRC_X_RANGE), rs.uniform(*SRC_Y_RANGE), Z_FIXED])
        chk.append(run_one('2d', src, trial))
    fails = [r['min_dist'] for r in chk if not r['found']]
    print(f'  found {sum(r["found"] for r in chk)}/20; closest approach of failures '
          + ', '.join(f'{v:.1f}' for v in fails) + ' m')
    data = {}
    print('=== Demonstration runs (source (%.0f, %.0f, %.1f) m) ===' % tuple(DEMO_SOURCE))
    for s in STRATS:
        r = run_one(s, DEMO_SOURCE, DEMO_SEED, record=True)
        data[s] = r
        st = r['states']
        print(f'  {LABELS[s]:38s}: {"FOUND" if r["found"] else "TIMEOUT"} t = {r["time"]:.1f} s, path {r["path"]:.1f} m, '
              f'closest 3D {r["min_dist"]:.2f} m, closest horizontal {r["min_dist_h"]:.2f} m, '
              f'contact {100 * r["contact"]:.1f} %, z_s estimate {r["zs_hat"]:.2f} m (true {DEMO_SOURCE[2]:.1f}), '
              f'states surge/cast/lost {100 * st.count("surge") / len(st):.0f}/{100 * st.count("cast") / len(st):.0f}/'
              f'{100 * st.count("lost") / len(st):.0f} %')
    rpf = data['pf3d']
    print(f'  PF final estimate ({rpf["pf_est"][-1][0]:.1f}, {rpf["pf_est"][-1][1]:.1f}, {rpf["pf_est"][-1][2]:.2f}), '
          f'KDE mode z_s {rpf["zs_mode"]:.2f} m')
    print(f'=== Monte Carlo: {N_TRIALS} trials, source ~ U{SRC_X_RANGE} x U{SRC_Y_RANGE} x U{ZS_RANGE} ===')
    mc = {s: [] for s in STRATS}
    srcs, in_plume = [], []
    for trial in range(N_TRIALS):
        rs = np.random.default_rng([trial, 0])
        src = np.array([rs.uniform(*SRC_X_RANGE), rs.uniform(*SRC_Y_RANGE), rs.uniform(*ZS_RANGE)])
        srcs.append(src)
        in_plume.append(bool(concentration(POS_START, src) >= C_THRESH))
        for s in STRATS:
            mc[s].append(run_one(s, src, trial))
    srcs = np.array(srcs); ip = np.array(in_plume)
    print(f'  trials whose start has mean C >= C_thresh: {ip.sum()}/{N_TRIALS}; z_s > 5 m in {int((srcs[:, 2] > 5).sum())} trials')
    summ = {}
    for s in STRATS:
        f = np.array([r['found'] for r in mc[s]]); fh = np.array([r['found_h'] for r in mc[s]])
        tt = np.array([r['time'] for r in mc[s]]); pp = np.array([r['path'] for r in mc[s]])
        fe = np.array([r['final_err'] for r in mc[s]])
        ze = np.array([r['zs_hat'] for r in mc[s]]) - srcs[:, 2]
        zm = np.array([r['zs_mode'] for r in mc[s]]) - srcs[:, 2]
        summ[s] = dict(f=f, fh=fh, tt=tt, pp=pp, fe=fe, ze=ze, zm=zm)
        print(f'  {LABELS[s]:38s}: found (3D) {f.sum()}/{N_TRIALS}, horizontal within 3 m {fh.sum()}/{N_TRIALS}; '
              f'time (found) median {np.median(tt[f]) if f.any() else float("nan"):.1f} s '
              f'[{np.percentile(tt[f], 25) if f.any() else float("nan"):.1f}, {np.percentile(tt[f], 75) if f.any() else float("nan"):.1f}]; '
              f'path (found) median {np.median(pp[f]) if f.any() else float("nan"):.1f} m; final 3D error median '
              f'{np.median(fe):.2f} m; z_s RMSE {np.sqrt(np.mean(ze ** 2)):.2f} m (|err| <= 0.5 m in {int(np.sum(np.abs(ze) <= ZS_TOL))}/{N_TRIALS})')
        print(f'  {"":38s}  start in plume: {f[ip].sum()}/{ip.sum()} found; outside: {f[~ip].sum()}/{(~ip).sum()}; '
              f'z_s <= 5 m: {f[srcs[:, 2] <= 5].sum()}/{int((srcs[:, 2] <= 5).sum())} found, '
              f'z_s > 5 m: {f[srcs[:, 2] > 5].sum()}/{int((srcs[:, 2] > 5).sum())}')
        if s == '2d':
            mh = np.array([r['min_dist_h'] for r in mc[s]])
            near2 = np.abs(srcs[:, 2] - Z_FIXED) < 1.0
            print(f'  {"":38s}  |z_s - 2| < 1 m: found {f[near2].sum()}/{near2.sum()}; failures in plume-start trials: '
                  f'median closest horizontal approach {np.median(mh[~f & ip]):.1f} m '
                  f'(range {mh[~f & ip].min():.1f}-{mh[~f & ip].max():.1f} m)')
        if s == 'pf3d':
            print(f'  {"":38s}  z_s from the KDE mode: RMSE {np.sqrt(np.mean(zm ** 2)):.2f} m, '
                  f'|err| <= 0.5 m in {int(np.sum(np.abs(zm) <= ZS_TOL))}/{N_TRIALS}; '
                  f'among found trials RMSE (mean) {np.sqrt(np.mean(ze[f] ** 2)) if f.any() else float("nan"):.2f} m')
    for a, b in [('cs3d', '2d'), ('pf3d', 'cs3d'), ('pf3d', '2d')]:
        ta = np.where(summ[a]['f'], summ[a]['tt'], np.inf); tb = np.where(summ[b]['f'], summ[b]['tt'], np.inf)
        print(f'  head-to-head {a} vs {b}: {a} faster {int(np.sum(ta < tb))}, slower {int(np.sum(ta > tb))}, '
              f'both failed {int(np.sum(np.isinf(ta) & np.isinf(tb)))}; z_s error smaller in '
              f'{int(np.sum(np.abs(summ[a]["ze"]) < np.abs(summ[b]["ze"])))}/{N_TRIALS}')
    data.update(mc=mc, summ=summ, srcs=srcs, in_plume=ip)
    return data


# ── Plots ───────────────────────────────────────────────────────────────────

def plot_plume_slices(data, out_dir):
    zs = DEMO_SOURCE[2]
    levels = [0.5, 1.0, 2.0, 3.0, zs]
    xs = np.linspace(40, 200, 321); ys = np.linspace(60, 160, 201)
    X, Y = np.meshgrid(xs, ys)
    fig, axes = plt.subplots(1, len(levels) + 1, figsize=(24, 4.3))
    norm = mcolors.PowerNorm(gamma=0.4, vmin=0, vmax=20)
    for ax, z in zip(axes, levels):
        C = concentration(np.stack([X, Y, np.full_like(X, z)], axis=-1), DEMO_SOURCE)
        im = ax.pcolormesh(X, Y, np.minimum(C, 20), cmap='YlOrRd', norm=norm, shading='auto')
        ax.contour(X, Y, C, levels=[C_THRESH], colors='b', linewidths=0.8)
        ax.plot(*DEMO_SOURCE[:2], 'r*', ms=12)
        ax.set_title(f'z = {z:.1f} m' + (' (= z_s)' if z == zs else ''), fontsize=10)
        ax.set_xlabel('x (m)')
    axes[0].set_ylabel('y (m)')
    fig.colorbar(im, ax=axes.tolist(), fraction=0.012, pad=0.01, label='mean C (mg/m³, clipped at 20)')
    ax = axes[-1]
    xz = np.linspace(40, 200, 321); zz = np.linspace(0, 8, 161)
    XZ, ZZ = np.meshgrid(xz, zz)
    C = concentration(np.stack([XZ, np.full_like(XZ, DEMO_SOURCE[1]), ZZ], axis=-1), DEMO_SOURCE)
    ax.pcolormesh(XZ, ZZ, np.minimum(C, 20), cmap='YlOrRd', norm=norm, shading='auto')
    ax.contour(XZ, ZZ, C, levels=[C_THRESH], colors='b', linewidths=0.8)
    ax.axhline(Z_FIXED, color='tab:orange', ls='--', lw=1, label='2D baseline z = 2 m')
    ax.plot(DEMO_SOURCE[0], zs, 'r*', ms=12)
    ax.set_xlabel('x (m)'); ax.set_ylabel('z (m)'); ax.set_title('xz slice through y_s'); ax.legend(fontsize=7)
    fig.suptitle(f'Demo plume, source at ({DEMO_SOURCE[0]:.0f}, {DEMO_SOURCE[1]:.0f}, {zs:.1f}) m; '
                 f'blue = C_thresh {C_THRESH}', fontsize=11)
    path = os.path.join(out_dir, 'plume_slices.png')
    plt.savefig(path, dpi=110, bbox_inches='tight'); plt.close()
    print(f'Saved: {path}')


def plot_trajectory_3d(data, out_dir):
    fig = plt.figure(figsize=(16, 6.5))
    ax = fig.add_subplot(1, 2, 1, projection='3d')
    for s in STRATS:
        tr = data[s]['traj']
        ax.plot(tr[:, 0], tr[:, 1], tr[:, 2], color=COLORS[s], lw=1.2,
                label=f'{LABELS[s]} ({"found" if data[s]["found"] else "timeout"}, {data[s]["time"]:.0f} s)')
    ax.scatter(*DEMO_SOURCE, color='red', s=80, marker='o', label='source')
    ax.scatter(*POS_START, color='k', s=40, marker='s')
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)'); ax.set_zlabel('z (m)'); ax.set_zlim(0, 8)
    ax.view_init(elev=22, azim=-120)
    ax.legend(fontsize=8, loc='upper left')
    ax.set_title('Demonstration run, 3D paths')
    ax2 = fig.add_subplot(1, 2, 2)
    for s in STRATS:
        tr = data[s]['traj']
        ax2.plot(np.arange(len(tr)) * DT, tr[:, 2], color=COLORS[s], lw=1.2, label=LABELS[s])
    ax2.axhline(DEMO_SOURCE[2], color='red', ls='--', lw=1, label=f'true z_s = {DEMO_SOURCE[2]} m')
    ax2.set_xlabel('time (s)'); ax2.set_ylabel('z (m)'); ax2.set_title('Altitude time series')
    ax2.legend(fontsize=8); ax2.grid(True, alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'trajectory_3d.png')
    plt.savefig(path, dpi=120); plt.close()
    print(f'Saved: {path}')


def plot_pf_snapshots(data, out_dir):
    snaps = data['pf3d']['pf_snaps']
    keys = [k for k in (0, 60, 180, 360) if k in snaps] + (['final'] if 'final' in snaps else [])
    fig, axes = plt.subplots(3, len(keys), figsize=(4.2 * len(keys), 11))
    if len(keys) == 1:
        axes = axes[:, None]
    zgrid = np.linspace(0, 7, 141)
    for j, k in enumerate(keys):
        p, w, kde = snaps[k]
        sz = 2 + 300 * w / w.max()
        axes[0, j].scatter(p[:, 0], p[:, 1], s=sz, alpha=0.5)
        axes[0, j].plot(*DEMO_SOURCE[:2], 'r*', ms=12)
        axes[0, j].set_xlim(0, 200); axes[0, j].set_ylim(0, 200)
        axes[0, j].set_title(f't = {k} s' if k != 'final' else f'end (t = {data["pf3d"]["time"]:.0f} s)')
        axes[1, j].scatter(p[:, 0], p[:, 2], s=sz, alpha=0.5)
        axes[1, j].plot(DEMO_SOURCE[0], DEMO_SOURCE[2], 'r*', ms=12)
        axes[1, j].set_xlim(0, 200); axes[1, j].set_ylim(0, 6.5)
        axes[2, j].plot(zgrid, kde, color='tab:green')
        axes[2, j].axvline(DEMO_SOURCE[2], color='red', ls='--')
        axes[2, j].set_xlabel('z_s (m)')
    axes[0, 0].set_ylabel('y (m)'); axes[1, 0].set_ylabel('z_s (m)'); axes[2, 0].set_ylabel('KDE of z_s')
    fig.suptitle('3D particle filter (demo run): xy projection, xz projection, z_s marginal', fontsize=12)
    plt.tight_layout()
    path = os.path.join(out_dir, 'pf_snapshots.png')
    plt.savefig(path, dpi=100); plt.close()
    print(f'Saved: {path}')


def plot_vertical_profiles(data, out_dir):
    fig, ax = plt.subplots(figsize=(8, 5))
    zz = np.linspace(0, 8, 401)
    for d, col in zip((5, 10, 20, 40, 80), plt.cm.viridis(np.linspace(0, 0.9, 5))):
        p = np.column_stack([np.full(401, DEMO_SOURCE[0] + d), np.full(401, DEMO_SOURCE[1]), zz])
        c = concentration(p, DEMO_SOURCE)
        ax.plot(c / c.max(), zz, color=col, label=f'{d} m downwind (peak {c.max():.1f} at z = {zz[np.argmax(c)]:.2f} m)')
    ax.axhline(DEMO_SOURCE[2], color='red', ls='--', lw=1, label='z_s')
    ax.axhline(Z_FIXED, color='tab:orange', ls=':', lw=1, label='2D baseline altitude')
    ax.set_xlabel('C / max C at that distance'); ax.set_ylabel('z (m)')
    ax.set_title('Vertical concentration profiles on the plume centreline (demo source)')
    ax.legend(fontsize=7); ax.grid(True, alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'vertical_profiles.png')
    plt.savefig(path, dpi=130); plt.close()
    print(f'Saved: {path}')


def plot_mc(data, out_dir):
    s_ = data['summ']; srcs = data['srcs']
    fig, axes = plt.subplots(1, 3, figsize=(17, 4.8))
    xs = np.arange(len(STRATS))
    axes[0].bar(xs - 0.2, [s_[s]['f'].sum() for s in STRATS], 0.4, color=[COLORS[s] for s in STRATS], label='3D capture')
    axes[0].bar(xs + 0.2, [s_[s]['fh'].sum() for s in STRATS], 0.4, color=[COLORS[s] for s in STRATS], alpha=0.4,
                hatch='//', label='horizontal within 3 m')
    axes[0].set_xticks(xs, ['2D', 'CS 3D', 'PF 3D']); axes[0].set_ylabel(f'trials (of {N_TRIALS})')
    axes[0].legend(fontsize=8); axes[0].set_title('Sources found')
    for s in STRATS:
        f = s_[s]['f']
        axes[1].scatter(srcs[f, 2], s_[s]['tt'][f], color=COLORS[s], s=25, label=LABELS[s])
        axes[1].scatter(srcs[~f, 2], np.full((~f).sum(), T_MAX + 15), color=COLORS[s], marker='x', s=25)
        axes[2].scatter(srcs[:, 2], s_[s]['ze'] + srcs[:, 2], color=COLORS[s], s=20, label=LABELS[s])
    axes[1].axvline(5.0, color='grey', ls=':', lw=1)
    axes[1].set_xlabel('true z_s (m)'); axes[1].set_ylabel('time to capture (s); x = not found (plotted at 615 s)')
    axes[1].legend(fontsize=7); axes[1].set_title('Time to capture vs source height')
    axes[2].plot([0.5, 6], [0.5, 6], 'k--', lw=1)
    axes[2].set_xlabel('true z_s (m)'); axes[2].set_ylabel('estimated z_s (m)'); axes[2].legend(fontsize=7)
    axes[2].set_title('Source-height estimate')
    plt.tight_layout()
    path = os.path.join(out_dir, 'mc_comparison.png')
    plt.savefig(path, dpi=120); plt.close()
    print(f'Saved: {path}')


def save_animation(data, out_dir):
    r = data['pf3d']
    tr = r['traj']
    n = len(tr)
    step = max(1, int(np.ceil((n - 1) / (N_ANIM - 1))))
    n_fr = int(np.ceil((n - 1) / step)) + 1
    fig = plt.figure(figsize=(11, 5))
    ax = fig.add_subplot(1, 2, 1, projection='3d')
    ax2 = fig.add_subplot(1, 2, 2)
    xs = np.linspace(40, 200, 161); zz = np.linspace(0, 8, 81)
    XZ, ZZ = np.meshgrid(xs, zz)
    C = concentration(np.stack([XZ, np.full_like(XZ, DEMO_SOURCE[1]), ZZ], axis=-1), DEMO_SOURCE)
    fr_t = np.array([f[0] for f in r['frames']])

    def update(i):
        k = min(i * step, n - 1)
        ax.clear(); ax2.clear()
        ax.plot(tr[:k + 1, 0], tr[:k + 1, 1], tr[:k + 1, 2], color='tab:green', lw=1)
        ax.scatter(*tr[k], color='k', s=20)
        ax.scatter(*DEMO_SOURCE, color='red', s=60)
        ax.set_xlim(40, 200); ax.set_ylim(60, 160); ax.set_zlim(0, 8)
        ax.view_init(elev=25, azim=-120 + 60 * i / max(n_fr - 1, 1))
        ax.set_title(f'PF-guided, t = {k * DT:.0f} s, z = {tr[k, 2]:.1f} m', fontsize=9)
        ax2.contourf(XZ, ZZ, np.minimum(C, 20), levels=20, cmap='YlOrRd')
        j = int(np.searchsorted(fr_t, k * DT, side='right')) - 1
        p = r['frames'][max(j, 0)][2]
        ax2.scatter(p[:, 0], p[:, 2], s=3, color='tab:blue', alpha=0.5)
        ax2.plot(tr[k, 0], tr[k, 2], 'ks', ms=5)
        ax2.plot(DEMO_SOURCE[0], DEMO_SOURCE[2], 'r*', ms=10)
        ax2.set_xlim(40, 200); ax2.set_ylim(0, 8)
        ax2.set_xlabel('x (m)'); ax2.set_ylabel('z (m)')
        ax2.set_title('xz: plume through y_s, particles (blue)', fontsize=9)
        return []

    ani = FuncAnimation(fig, update, frames=n_fr, interval=100, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer=PillowWriter(fps=10), dpi=70)
    plt.close()
    print(f'Saved: {path}  ({n_fr} frames)')


if __name__ == '__main__':
    print('Running S045 3D Chemical Plume Tracing simulation...')
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    data = run_simulation()
    plot_plume_slices(data, OUTPUT_DIR)
    plot_trajectory_3d(data, OUTPUT_DIR)
    plot_vertical_profiles(data, OUTPUT_DIR)
    plot_pf_snapshots(data, OUTPUT_DIR)
    plot_mc(data, OUTPUT_DIR)
    save_animation(data, OUTPUT_DIR)
    print('Done.')
