"""
S070 3D Upgrade — Swarm Weeding with Canopy and Sweep-Altitude Choice
=====================================================================
The 2D S070 base: three drones sweep 13.3 m sub-strips of a 40 x 40 m field (5 evenly
spaced passes, sensor radius 1.5 m, one Bernoulli(0.85) detection draw per pass encounter),
then the detected weeds are shared out (capacitated Hungarian + nearest-neighbour order)
and each is hovered over for 2 s; treatment needs the hover RMS within tolerance.  Its
altitude is implicit (the 2 m reference height of the sensor model).

The 3D variant makes altitude a planned state on both phases:
  * Phase 1 sweep altitude z: sensor radius r_s(z) = 1.5 z / 2 (fewer passes when higher),
    detection P(z) = min(1, 0.85 (2 / z)^0.5), safe speed v(z) = min(1, (z - 0.6) / 1.0) m/s,
    stereo depth noise sigma_z(z) = 0.05 (z / 2)^2 on the canopy height above each weed;
  * the crop canopy is a Gaussian random field (mean 0.6 m, std 0.15 m, length 4 m);
  * Phase 2: each drone transits at z, descends to z_treat = canopy estimate + 0.2 m,
    hovers 2 s under a 3D PID (or PI), and climbs back; treatment succeeds when the 3D RMS
    error to the true point 0.2 m above the canopy is <= 0.20 m and the drone never
    touches the canopy;
  * Hungarian assignment on 3D cost (card) vs the 2D cost of the base.
The 2D baseline is the base plan at its implicit 2 m altitude (same draws).  Every trial
uses common random numbers across altitudes, controllers and assignment costs; the first
draws of each trial are the 2D base's, so the z = 2 m detections reproduce the base.

Usage:
    conda activate drones
    python src/04_industrial_agriculture/3d/s070_3d_swarm_weeding.py
"""

import os
import sys
import math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from scipy.ndimage import gaussian_filter, map_coordinates
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, '..', '..', '..'))
sys.path.insert(0, os.path.join(SCRIPT_DIR, '..'))
import s070_swarm_weeding as base                  # noqa: E402  (assignment, NN order)

OUTPUT_DIR = os.path.join(PROJECT_ROOT, 'outputs', '04_industrial_agriculture', '3d',
                          's070_3d_swarm_weeding')

# ── Parameters ────────────────────────────────────────────────────────────────
FIELD, N_DRONES, N_WEEDS = base.FIELD, base.N_DRONES, base.N_WEEDS
STRIP_W = base.STRIP_W
V = base.V                          # 1 m/s (horizontal and vertical, added for vertical)
SIGMA_GPS = base.SIGMA_GPS          # 0.10 m, used on all three axes
T_TREAT, DT = base.T_TREAT, base.DT
KP, KI, KD = base.KP, base.KI, base.KD
START_SD = base.START_SD            # 0.3 m horizontal arrival offset per axis (base / card code)
START_SD_Z = 0.0                    # added: the vertical descent ends on its target (swept)
SDZ_SWEEP = [0.0, 0.1, 0.3]
SDZ_Z = [1.6, 2.0, 3.5]
SEED = base.SEED
N_TRIALS = base.N_TRIALS            # 200
H_BAR, SIG_H, ELL = 0.6, 0.15, 4.0  # card canopy field
GRID_N = 100                        # card: 100 x 100 grid
Z_REF, R0, P0, BETA = 2.0, 1.5, 0.85, 0.5   # card sensor model
SZ0 = 0.05                          # card stereo depth noise at z_ref
DH_SAFE = 1.0                       # card speed rule
DH_CLEAR = 0.20                     # card
EPS3 = 0.20                         # card 3D tolerance
Z_RANGE = (1.0, 3.5)                # card sweep-altitude search range
Z_BASIC = [1.0, 1.25, 1.5, 1.6, 1.75, 2.0, 2.5, 3.0, 3.5]
# pass-count steps: n(z) = ceil(L_strip / 2 r_s(z)) drops from k to k-1 at z_k = L_strip Z_REF / (2 R0 k)
Z_STEPS = [STRIP_W * Z_REF / (2 * R0 * k) for k in range(3, 10)]
Z_STEPS = [z for z in Z_STEPS if Z_RANGE[0] < z < Z_RANGE[1]]
Z_NEAR = sorted({math.floor(z * 100) / 100 for z in Z_STEPS} |          # just below: k passes
                {math.floor(z * 100) / 100 + 0.01 for z in Z_STEPS})    # just above: k - 1 passes
Z_NEAR = [round(z, 2) for z in Z_NEAR]
Z_GRID = sorted(set(Z_BASIC) | set(Z_NEAR))
Z_BASE = 2.0                        # the 2D base's implicit altitude (z_ref)
EXTRA = 10_000                      # second stream: default_rng(EXTRA + seed)
ANIM_STEP = 4.0
DRONE_COLS = ['tab:red', 'tab:orange', 'tab:purple']
GC = (np.arange(GRID_N) + 0.5) * FIELD / GRID_N       # canopy grid centres (0.2 ... 39.8 m)


# ── Models ────────────────────────────────────────────────────────────────────

def r_s(z):
    return R0 * z / Z_REF


def p_det(z, clip=True):
    p = P0 * (Z_REF / z) ** BETA
    return min(1.0, p) if clip else p


def v_sweep(z):
    return V * min(1.0, (z - H_BAR) / DH_SAFE)


def sigma_z(z):
    return SZ0 * (z / Z_REF) ** 2


def n_passes(z):
    return int(np.ceil(STRIP_W / (2 * r_s(z)) - 1e-9))


def pass_positions(i, z):
    n = n_passes(z)
    sp = STRIP_W / n
    return i * STRIP_W + sp / 2 + sp * np.arange(n)


def sweep_path(i, z):
    pts = []
    for k, y in enumerate(pass_positions(i, z)):
        xs = (0.0, FIELD) if k % 2 == 0 else (FIELD, 0.0)
        pts += [[xs[0], y], [xs[1], y]]
    return np.array(pts)


def canopy_field(rng):
    s = ELL / math.sqrt(2) / (FIELD / GRID_N)            # SE kernel -> Gaussian filter (cells)
    delta = np.zeros((GRID_N, GRID_N))
    delta[0, 0] = 1.0
    k_var = float(np.sum(gaussian_filter(delta, s, mode='wrap') ** 2))
    g = gaussian_filter(rng.standard_normal((GRID_N, GRID_N)), s, mode='wrap') / math.sqrt(k_var)
    return np.clip(H_BAR + SIG_H * g, 0.05, None)


def canopy_at(H, x, y):
    cx = (np.asarray(x) / (FIELD / GRID_N)) - 0.5
    cy = (np.asarray(y) / (FIELD / GRID_N)) - 0.5
    return map_coordinates(H, [np.atleast_1d(cy).ravel(), np.atleast_1d(cx).ravel()], order=1,
                           mode='nearest').reshape(np.shape(x))


def detect(weeds, eta, u, z, hw):
    det = np.zeros(len(weeds), dtype=bool)
    for i in range(N_DRONES):
        for j, y in enumerate(pass_positions(i, z)):
            enc = (np.abs(weeds[:, 1] - (y + SIGMA_GPS * eta[i, j])) <= r_s(z)) & (z > hw)  # card LOS
            det |= enc & (u[i, j] < p_det(z))
    return det


def pid_hover3d(z0, zn, kd, bias, hw, H, wxy, sd_z=None):
    """Card PID in 3D on the measured error to the target (velocity command saturated at V).
    Positions are relative to the target t_hat = (x_w, y_w, z_hat_treat). bias: (H,3) target minus
    true desired point; hw: true canopy + 0.2 at the weed. Returns 3D RMS to the true point and
    whether the drone touched the canopy."""
    pos = z0 * np.array([START_SD, START_SD, START_SD_Z if sd_z is None else sd_z])
    integ = np.zeros_like(pos)
    prev = -pos
    err2 = np.sum((pos + bias) ** 2, axis=1)
    z_tgt = (hw + bias[:, 2])                                  # absolute target altitude
    strike = (z_tgt + pos[:, 2]) < canopy_at(H, wxy[:, 0] + pos[:, 0], wxy[:, 1] + pos[:, 1])
    for k in range(zn.shape[1]):
        e = -(pos + SIGMA_GPS * zn[:, k])
        integ += e * DT
        u = KP * e + KI * integ + kd * (e - prev) / DT
        un = np.linalg.norm(u, axis=1, keepdims=True)
        u = np.where(un > V, u * V / np.maximum(un, 1e-12), u)
        assert np.all(np.linalg.norm(u, axis=1) <= V + 1e-9), 'hover speed above limit'
        pos = pos + u * DT
        assert np.all(np.isfinite(pos)) and np.all(np.abs(pos) < 2.0), 'hover diverged'
        prev = e
        err2 += np.sum((pos + bias) ** 2, axis=1)
        h_here = canopy_at(H, wxy[:, 0] + pos[:, 0], wxy[:, 1] + pos[:, 1])
        strike |= (z_tgt + pos[:, 2]) < h_here
    rms = np.sqrt(err2 / (zn.shape[1] + 1))
    return rms, strike


def routes(ends, P, assign):
    """Leg times for L-shaped flights: transit at the sweep altitude, vertical descent to the
    treatment altitude, 2 s hover, climb back. ends (3,3), P (M,3) treatment points."""
    dist = np.zeros(N_DRONES)
    hor = np.zeros(N_DRONES)
    for i, lst in enumerate(assign):
        cur = ends[i, :2]
        for j in lst:
            dh = np.linalg.norm(P[j, :2] - cur)
            dv = ends[i, 2] - P[j, 2]
            dist[i] += dh + 2 * abs(dv)
            hor[i] += dh
            cur = P[j, :2]
    times = dist / V + np.array([len(a) for a in assign]) * T_TREAT
    return dist, hor, times


# ── Simulation ────────────────────────────────────────────────────────────────

def run_trial(seed, keep=False):
    rng = np.random.default_rng(seed)                  # the 2D base's draws first
    weeds = rng.uniform(0, FIELD, size=(N_WEEDS, 2))
    nb = max(len(base.pass_positions(0, l)) for l in ('formula', 'card'))
    eta_b = rng.standard_normal((N_DRONES, nb, N_WEEDS))
    u_b = rng.random((N_DRONES, nb, N_WEEDS))
    r2 = np.random.default_rng(EXTRA + seed)
    nmax = n_passes(Z_RANGE[0])
    eta = np.concatenate([eta_b, r2.standard_normal((N_DRONES, nmax - nb, N_WEEDS))], axis=1)
    u = np.concatenate([u_b, r2.random((N_DRONES, nmax - nb, N_WEEDS))], axis=1)
    H = canopy_field(r2)
    hw = canopy_at(H, weeds[:, 0], weeds[:, 1])
    zweed = r2.uniform(0, 1, N_WEEDS) * hw              # weed top at or below the canopy (card)
    n_st = int(round(T_TREAT / DT))
    zs = r2.standard_normal(N_WEEDS)                    # stereo error per weed (scaled per altitude)
    z0 = r2.standard_normal((N_WEEDS, 3))
    zn = r2.standard_normal((N_WEEDS, n_st, 3))
    out = {'H': H, 'weeds': weeds, 'hw': hw, 'zweed': zweed, 'per_z': {}}
    for z in Z_GRID:
        det = detect(weeds, eta, u, z, hw)
        ys = np.concatenate([np.full(81, y) for i in range(N_DRONES) for y in pass_positions(i, z)])
        xs = np.tile(np.linspace(0, FIELD, 81), len(ys) // 81)
        sweep_clear = float(z - canopy_at(H, xs, ys).max())       # lowest clearance on the passes
        idx = np.where(det)[0]
        dz_est = sigma_z(z) * zs[idx]                   # canopy-height estimate error
        P = np.column_stack([weeds[idx], hw[idx] + DH_CLEAR + dz_est])   # targets (camera xy)
        ends = np.array([[*sweep_path(i, z)[-1], z] for i in range(N_DRONES)])
        a3 = base.hungarian_assign(ends, P)            # card: 3D Euclidean cost
        a2 = base.hungarian_assign(ends[:, :2], P[:, :2])   # 2D base cost
        nn3 = [base.nn_order(ends[i], P, a3[i]) for i in range(N_DRONES)]
        nn2 = [base.nn_order(ends[i, :2], P[:, :2], a2[i]) for i in range(N_DRONES)]
        for a in (nn3, nn2):
            assert sorted(j for lst in a for j in lst) == list(range(len(idx))), 'weed not covered once'
        d3, h3, t3 = routes(ends, P, nn3)
        d2, h2, t2 = routes(ends, P, nn2)
        t1 = max(float(np.sum(np.linalg.norm(np.diff(sweep_path(i, z), axis=0), axis=1)))
                 for i in range(N_DRONES)) / v_sweep(z)
        bias = np.column_stack([np.zeros((len(idx), 2)), dz_est])
        res = {'det': det, 'idx': idx, 'P': P, 'ends': ends, 'a3': nn3, 'a2': nn2,
               'dist3': float(d3.sum()), 'dist2': float(d2.sum()), 'mk3': float(t3.max()),
               'mk2': float(t2.max()), 'same': [sorted(x) for x in a3] == [sorted(x) for x in a2],
               't1': t1, 'dz_est': dz_est}
        res['sweep_clear'] = sweep_clear
        res['valid'] = sweep_clear >= DH_CLEAR
        for name, kd in (('pid', KD), ('pi', 0.0)):
            rms, strike = pid_hover3d(z0[idx], zn[idx], kd, bias, hw[idx] + DH_CLEAR, H, weeds[idx])
            ok = (rms <= EPS3) & ~strike & res['valid']      # no treatment if the sweep hit the crop
            res[name] = {'rms': rms, 'strike': strike, 'ok': ok}
        if z in SDZ_Z:                                       # vertical arrival-offset sensitivity
            for sdz in SDZ_SWEEP:
                rms, strike = pid_hover3d(z0[idx], zn[idx], 0.0, bias, hw[idx] + DH_CLEAR, H, weeds[idx], sdz)
                res[f'pi_sdz{sdz}'] = {'ok': (rms <= EPS3) & ~strike & res['valid'], 'strike': strike}
        out['per_z'][z] = res
    if not keep:
        for z in Z_GRID:
            for k in ('P', 'ends', 'a3', 'a2'):
                out['per_z'][z].pop(k)
        out.pop('H')
    return out


def run_simulation():
    return [run_trial(SEED + t, keep=(t == 0)) for t in range(N_TRIALS)]


def card_zstar():
    zz = np.round(np.arange(Z_RANGE[0], Z_RANGE[1] + 1e-9, 0.01), 4)
    rate_card = np.array([p_det(z) * v_sweep(z) for z in zz])               # W P / T, T ~ 1/v
    rate_pass = np.array([p_det(z) * v_sweep(z) / n_passes(z) for z in zz])  # T ~ n_passes / v
    return zz, rate_card, rate_pass


# ── Plotting ──────────────────────────────────────────────────────────────────

def _agg(trials, z, f):
    return np.array([f(t['per_z'][z]) for t in trials])


def plot_canopy_detection(trials, out_dir, z_show):
    t0 = trials[0]
    fig, axes = plt.subplots(1, 2, figsize=(16, 6.5))
    ax = axes[0]
    im = ax.imshow(t0['H'], origin='lower', extent=[0, FIELD, 0, FIELD], cmap='YlGn', vmin=0.3, vmax=0.9)
    fig.colorbar(im, ax=ax, label='canopy height h (m)')
    det = t0['per_z'][z_show]['det']
    ax.scatter(*t0['weeds'][~det].T, color='grey', edgecolor='k', s=40, label='missed weed')
    ax.scatter(*t0['weeds'][det].T, color='gold', edgecolor='k', s=60, label='detected weed')
    for i in range(N_DRONES):
        p = sweep_path(i, z_show)
        ax.plot(p[:, 0], p[:, 1], color=DRONE_COLS[i], lw=0.7)
    ax.set_title(f'Trial 0 canopy (GRF, mean {H_BAR} m, σ {SIG_H} m, ℓ {ELL} m) and sweeps at '
                 f'z = {z_show} m: {det.sum()}/{N_WEEDS} detected', fontsize=10)
    ax.legend(fontsize=8, loc='upper right')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax = axes[1]
    zz, rc, rp = card_zstar()
    ax.plot(zz, [p_det(z) for z in zz], color='tab:blue', lw=1.5, label='P_detect(z) (clipped at 1)')
    ax.plot(zz, [p_det(z, clip=False) for z in zz], color='tab:blue', lw=0.8, ls=':',
            label='card formula (exceeds 1 below 1.45 m)')
    ax.plot(zz, [v_sweep(z) for z in zz], color='tab:green', lw=1.2, label='v_sweep(z) (m/s)')
    ax.plot(zz, rc / rc.max(), color='k', lw=1.2, ls='--', label='card rate P·v (normalised)')
    ax.plot(zz, rp / rp.max(), color='tab:red', lw=1.2, ls='-.', label='rate P·v / passes (normalised)')
    ax2 = ax.twinx()
    ax2.plot(zz, [r_s(z) for z in zz], color='tab:orange', lw=1.2, label='r_s(z)')
    ax2.set_ylabel('sensor radius r_s (m)', color='tab:orange')
    ax.axvline(zz[np.argmax(rc)], color='k', lw=0.8, ls='--')
    ax.axvline(zz[np.argmax(rp)], color='tab:red', lw=0.8, ls='-.')
    ax.set_xlabel('sweep altitude z (m)')
    ax.set_ylabel('probability / speed / normalised rate')
    ax.set_ylim(0, 1.3)
    ax.legend(fontsize=8, loc='lower left')
    ax.set_title('Sensor model and the card\'s z* criterion', fontsize=10)
    ax.grid(alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'canopy_detection.png')
    plt.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def timelines(t0, z):
    """Piecewise-linear 3D waypoints with times for every drone of trial 0 at altitude z."""
    r = t0['per_z'][z]
    t1 = r['t1']
    tl = []
    for i in range(N_DRONES):
        p = sweep_path(i, z)
        pts = [(0.0, p[0, 0], p[0, 1], z)]
        t = 0.0
        for a, b in zip(p[:-1], p[1:]):
            t += np.linalg.norm(b - a) / v_sweep(z)
            pts.append((t, b[0], b[1], z))
        t = t1
        pts.append((t, p[-1, 0], p[-1, 1], z))
        cur = p[-1]
        for j in r['a3'][i]:
            tx, ty, tz = r['P'][j]
            t += np.linalg.norm(np.array([tx, ty]) - cur) / V
            pts.append((t, tx, ty, z))
            t += (z - tz) / V
            pts.append((t, tx, ty, tz))
            t += T_TREAT
            pts.append((t, tx, ty, tz))
            t += (z - tz) / V
            pts.append((t, tx, ty, z))
            cur = np.array([tx, ty])
        tl.append(np.array(pts))
    return tl


def plot_field_3d(trials, out_dir, z_show):
    t0 = trials[0]
    r = t0['per_z'][z_show]
    fig = plt.figure(figsize=(13, 9))
    ax = fig.add_subplot(111, projection='3d')
    gx, gy = np.meshgrid(GC[::4], GC[::4])
    ax.plot_surface(gx, gy, t0['H'][::4, ::4], color='green', alpha=0.15, linewidth=0)
    det = r['det']
    ax.scatter(*t0['weeds'][~det].T, t0['zweed'][~det], color='grey', s=25, label='missed weed')
    ax.scatter(*t0['weeds'][det].T, t0['zweed'][det], color='gold', edgecolor='k', s=35,
               label='detected weed')
    for i, tl in enumerate(timelines(t0, z_show)):
        ax.plot(tl[:, 1], tl[:, 2], tl[:, 3], color=DRONE_COLS[i], lw=0.8, label=f'drone {i}')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_zlabel('z (m)')
    ax.set_zlim(0, z_show + 0.5)
    ax.set_box_aspect((1, 1, 0.35))
    ax.view_init(elev=25, azim=-60)
    ax.set_title(f'S070 3D trial 0: sweeps at {z_show} m, Phase-2 transits and vertical descents '
                 'to 0.2 m above the canopy (green surface)')
    ax.legend(fontsize=8, loc='upper left')
    path = os.path.join(out_dir, 'field_3d.png')
    plt.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_altitude(trials, out_dir, z_show):
    t0 = trials[0]
    fig, ax = plt.subplots(figsize=(14, 4.5))
    for i, tl in enumerate(timelines(t0, z_show)):
        ax.plot(tl[:, 0], tl[:, 3], color=DRONE_COLS[i], lw=1.0, label=f'drone {i}')
    ax.axvline(t0['per_z'][z_show]['t1'], color='k', ls='--', lw=0.8, label='end of Phase 1')
    ax.axhline(H_BAR, color='green', lw=0.8, ls=':', label=f'mean canopy {H_BAR} m')
    ax.set_xlabel('time (s)')
    ax.set_ylabel('z (m)')
    ax.set_title(f'S070 3D altitude, trial 0, sweep at {z_show} m: Phase-1 cruise, then descent / '
                 f'2 s hover / climb per weed')
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'altitude_timeseries.png')
    plt.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_metrics(trials, out_dir, z_show):
    fig, axes = plt.subplots(1, 4, figsize=(22, 5))
    rp = np.concatenate([t['per_z'][z_show]['pid']['rms'] for t in trials])
    ri = np.concatenate([t['per_z'][z_show]['pi']['rms'] for t in trials])
    bins = np.linspace(0, max(rp.max(), ri.max()), 40)
    axes[0].hist(rp, bins=bins, alpha=0.7, color='tab:red', label=f'card PID, mean {rp.mean():.3f} m')
    axes[0].hist(ri, bins=bins, alpha=0.7, color='tab:blue', label=f'PI, mean {ri.mean():.3f} m')
    axes[0].axvline(EPS3, color='k', ls='--', label=f'ε = {EPS3} m')
    axes[0].set_xlabel('3D hover RMS over 2 s (m)')
    axes[0].set_title(f'(i) 3D hover RMS, z = {z_show} m', fontsize=10)
    axes[0].legend(fontsize=8)
    d3 = _agg(trials, z_show, lambda r: r['dist3'])
    d2 = _agg(trials, z_show, lambda r: r['dist2'])
    axes[1].bar([0, 1], [d2.mean(), d3.mean()], yerr=[d2.std(), d3.std()], color=['grey', 'tab:blue'],
                capsize=6)
    axes[1].set_xticks([0, 1])
    axes[1].set_xticklabels(['2D-cost Hungarian', '3D-cost Hungarian'])
    axes[1].set_ylabel('Phase-2 flight distance (m)')
    axes[1].set_title(f'(ii) distance, z = {z_show} m ({N_TRIALS} trials)', fontsize=10)
    for z, c in ((Z_GRID[0], 'tab:green'), (Z_BASE, 'tab:blue'), (Z_GRID[-1], 'tab:red')):
        e = np.concatenate([t['per_z'][z]['dz_est'] for t in trials])
        axes[2].hist(e, bins=np.linspace(-0.6, 0.6, 61), histtype='step', color=c, lw=1.5,
                     label=f'z = {z} m: σ = {e.std():.3f} m')
    axes[2].axvline(-DH_CLEAR, color='k', ls='--', lw=0.8, label='−0.2 m: target inside canopy')
    axes[2].set_xlabel('canopy-height estimate error (m)')
    axes[2].set_title('(iii) stereo depth error', fontsize=10)
    axes[2].legend(fontsize=8)
    det = _agg(trials, z_show, lambda r: len(r['idx'])).sum()
    ok = _agg(trials, z_show, lambda r: r['pid']['ok'].sum()).sum()
    st = _agg(trials, z_show, lambda r: r['pid']['strike'].sum()).sum()
    miss = N_TRIALS * N_WEEDS - det
    axes[3].pie([miss, det - ok - st, st, ok], colors=['grey', 'orange', 'red', 'green'],
                labels=['missed', 'detected, hover error', 'canopy contact', 'treated'],
                autopct='%1.1f%%')
    axes[3].set_title(f'(iv) outcomes, card PID, z = {z_show} m', fontsize=10)
    fig.suptitle('S070 3D metrics')
    plt.tight_layout()
    path = os.path.join(out_dir, 'metrics.png')
    plt.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_altitude_sweep(trials, out_dir):
    fig, axes = plt.subplots(1, 3, figsize=(19, 5))
    zg = np.array(Z_GRID)
    det = np.array([_agg(trials, z, lambda r: len(r['idx'])).mean() for z in zg])
    okp = np.array([_agg(trials, z, lambda r: r['pid']['ok'].sum()).mean() for z in zg])
    oki = np.array([_agg(trials, z, lambda r: r['pi']['ok'].sum()).mean() for z in zg])
    stp = np.array([_agg(trials, z, lambda r: r['pid']['strike'].sum()).mean() for z in zg])
    sti = np.array([_agg(trials, z, lambda r: r['pi']['strike'].sum()).mean() for z in zg])
    t1 = np.array([trials[0]['per_z'][z]['t1'] for z in zg])
    tm = np.array([(_agg(trials, z, lambda r: r['t1'] + r['mk3'])).mean() for z in zg])
    ax = axes[0]
    ax.plot(zg, det, 'o-', color='k', label='detected')
    ax.plot(zg, oki, 's-', color='tab:blue', label='treated, PI')
    ax.plot(zg, okp, '^-', color='tab:red', label='treated, card PID')
    ax.set_xlabel('sweep altitude z (m)')
    ax.set_ylabel(f'weeds per trial (of {N_WEEDS})')
    ax.set_title('Detection and treatment')
    ax.legend(fontsize=8)
    ax = axes[1]
    ax.plot(zg, t1, 'o-', color='tab:green', label='Phase 1 time')
    ax.plot(zg, tm, 's-', color='k', label='mission time (Phase 1 + Phase-2 makespan)')
    ax.set_xlabel('sweep altitude z (m)')
    ax.set_ylabel('time (s)')
    ax.set_title('Time')
    ax.legend(fontsize=8)
    ax = axes[2]
    for key, mk, col, lab in (('pi', 's-', 'tab:blue', 'PI'), ('pid', '^-', 'tab:red', 'card PID')):
        thr = [60 * _agg(trials, z, lambda r: r[key]['ok'].sum()) / _agg(trials, z, lambda r: r['t1'] + r['mk3'])
               for z in zg]
        ax.errorbar(zg, [v.mean() for v in thr], yerr=[v.std(ddof=1) / math.sqrt(v.size) for v in thr],
                    fmt=mk, color=col, capsize=3, label=f'treated per minute, {lab} (per-trial mean ± s.e.)')
    ax2 = ax.twinx()
    ax2.plot(zg, sti, 'x--', color='tab:blue', label='canopy contacts, PI')
    ax2.plot(zg, stp, 'x--', color='tab:red', label='canopy contacts, PID')
    ax2.set_ylabel('canopy contacts per trial')
    ax2.set_ylim(0, 5.2)
    ax.set_xlabel('sweep altitude z (m)')
    ax.set_ylabel('treated weeds per minute')
    ax.set_ylim(0, 5.2)                                # room for the legends above the curves
    zz, rc, rp = card_zstar()
    ax.axvline(zz[np.argmax(rc)], color='k', ls='--', lw=0.8)
    ax.set_title(f'Throughput and canopy contacts (dashed = card z* = {zz[np.argmax(rc)]:.2f} m)')
    ax.legend(fontsize=8, loc='upper left')
    ax2.legend(fontsize=8, loc='upper right')
    for a in axes:
        a.axvline(Z_BASE, color='grey', lw=0.6, ls=':')
        a.grid(alpha=0.3)
    fig.suptitle(f'S070 3D sweep-altitude study ({N_TRIALS} trials, common random numbers; '
                 f'dotted = 2D base altitude {Z_BASE} m)')
    plt.tight_layout()
    path = os.path.join(out_dir, 'altitude_sweep.png')
    plt.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def save_animation(trials, out_dir, z_show):
    t0 = trials[0]
    tls = timelines(t0, z_show)
    t_end = max(tl[-1, 0] for tl in tls)
    n_frames = int(np.ceil(t_end / ANIM_STEP)) + 1
    fig = plt.figure(figsize=(9, 7))
    ax = fig.add_subplot(111, projection='3d')
    gx, gy = np.meshgrid(GC[::5], GC[::5])
    det = t0['per_z'][z_show]['det']

    def pos(tl, t):
        return [np.interp(t, tl[:, 0], tl[:, c]) for c in (1, 2, 3)]

    def update(fi):
        t = min(fi * ANIM_STEP, t_end)
        ax.cla()
        ax.plot_wireframe(gx, gy, t0['H'][::5, ::5], color='green', lw=0.3, alpha=0.5)
        ax.scatter(*t0['weeds'][det].T, t0['zweed'][det], color='gold', s=15)
        ax.scatter(*t0['weeds'][~det].T, t0['zweed'][~det], color='grey', s=10)
        for i, tl in enumerate(tls):
            m = tl[:, 0] <= t
            seg = np.vstack([tl[m][:, 1:], pos(tl, t)])
            ax.plot(seg[:, 0], seg[:, 1], seg[:, 2], color=DRONE_COLS[i], lw=0.8)
            p = pos(tl, t)
            ax.scatter([p[0]], [p[1]], [p[2]], color=DRONE_COLS[i], s=40, edgecolor='k')
        ax.set_xlim(0, FIELD)
        ax.set_ylim(0, FIELD)
        ax.set_zlim(0, z_show + 0.5)
        ax.set_box_aspect((1, 1, 0.35))
        ax.view_init(elev=25, azim=-60)
        phase = 'Phase 1 sweep' if t < t0['per_z'][z_show]['t1'] else 'Phase 2 treatment'
        ax.set_title(f'S070 3D trial 0 — t = {t:.0f} s, {phase} (sweep altitude {z_show} m)')
        return []

    anim = animation.FuncAnimation(fig, update, frames=n_frames, interval=80, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    anim.save(path, writer='pillow', fps=10, dpi=70)
    plt.close(fig)
    print(f'Saved: {path}  ({n_frames} frames, every {ANIM_STEP:.0f} s, last at {t_end:.1f} s)')


# ── Report ────────────────────────────────────────────────────────────────────

def _ms(v, d=2):
    v = np.asarray(v, dtype=float)
    return f'{v.mean():.{d}f} ± {v.std():.{d}f}'


def report(trials):
    print('\n=== S070 3D Swarm Weeding — Results ===')
    zz, rc, rp = card_zstar()
    z_card, z_pass = float(zz[np.argmax(rc)]), float(zz[np.argmax(rp)])
    print(f'  Card P_detect formula exceeds 1 below z = {Z_REF * (P0) ** (1 / BETA):.3f} m '
          f'(P(1.0 m) = {p_det(1.0, clip=False):.3f}); clipped at 1')
    print(f'  Card z* = argmax P(z) v(z): {z_card:.2f} m; with the pass count (T ~ n_passes / v): '
          f'{z_pass:.2f} m')
    sd = np.concatenate([t['H'].ravel() for t in trials[:1]])
    print(f"  Canopy GRF trial 0: mean {sd.mean():.3f} m, std {sd.std():.3f} m, "
          f"range {sd.min():.3f}-{sd.max():.3f} m")
    hs = np.concatenate([t['hw'] for t in trials])
    print(f'  Canopy height at the weeds over all trials: mean {hs.mean():.3f} m, std {hs.std():.3f} m')
    db = _agg(trials, Z_BASE, lambda r: r['det'].sum())
    print(f'  2D-base reproduction at z = {Z_BASE} m: detected {_ms(db)} / {N_WEEDS} '
          f'(2D S070: 25.71 ± 1.94)')
    s3 = math.sqrt((2 * SIGMA_GPS ** 2 + sigma_z(z_card) ** 2) / (1 + 2 * KP / KD))
    p_card = p_det(z_card) * (1 - math.exp(-EPS3 ** 2 / (2 * s3 ** 2)))
    print(f'  Card prediction at z*: sigma_hover,3D = {s3:.4f} m, P_success = {p_card:.4f} '
          f'-> {N_WEEDS * p_card:.1f} treated')
    print(f'  Sweep altitude study ({N_TRIALS} trials, common random numbers):')
    print('    z (m) | passes | P_detect | v (m/s) | Phase 1 (s) | detected | sigma_z (m) | treated PID / PI | '
          'canopy contacts PID / PI | mission (s) | treated/min PID / PI (per-trial mean) | 3D vs 2D Hungarian')
    rows = {}
    for z in Z_GRID:
        det = _agg(trials, z, lambda r: len(r['idx']))
        okp = _agg(trials, z, lambda r: r['pid']['ok'].sum())
        oki = _agg(trials, z, lambda r: r['pi']['ok'].sum())
        stp = _agg(trials, z, lambda r: r['pid']['strike'].sum())
        sti = _agg(trials, z, lambda r: r['pi']['strike'].sum())
        mis = _agg(trials, z, lambda r: r['t1'] + r['mk3'])
        same = _agg(trials, z, lambda r: r['same'])
        d3 = _agg(trials, z, lambda r: r['dist3'])
        d2 = _agg(trials, z, lambda r: r['dist2'])
        rows[z] = dict(det=det, okp=okp, oki=oki, stp=stp, sti=sti, mis=mis)
        print(f"    {z:4.2f} | {n_passes(z)} | {p_det(z):.3f} | {v_sweep(z):.2f} | {trials[0]['per_z'][z]['t1']:7.1f} | "
              f"{_ms(det)} | {sigma_z(z):.4f} | {_ms(okp)} / {_ms(oki)} | {_ms(stp)} / {_ms(sti)} | "
              f"{_ms(mis, 1)} | {np.mean(60 * okp / mis):.3f} / {np.mean(60 * oki / mis):.3f} | "
              f"same assignment {int(same.sum())}/{N_TRIALS}, distance 3D {d3.mean():.1f} vs 2D "
              f"{d2.mean():.1f} m")
    best_ok = max(Z_GRID, key=lambda z: rows[z]['oki'].mean())
    print(f'  Most weeds treated (PI, mean per trial): {best_ok} m')
    se = lambda v: np.std(v, ddof=1) / math.sqrt(len(v))       # noqa: E731
    for name, key in (('PI', 'oki'), ('card PID', 'okp')):
        print(f'  Per-trial throughput (treated per minute of mission, {name}); head-to-head against the '
              f'2D base altitude {Z_BASE} m on the same trials: z -> mean ± s.e.; higher / lower / tied '
              f'trials; mean difference ± s.e.')
        b = 60 * rows[Z_BASE][key] / rows[Z_BASE]['mis']
        for z in Z_GRID:
            c = 60 * rows[z][key] / rows[z]['mis']
            line = f'    {z:4.2f} m: {c.mean():.3f} ± {se(c):.3f}'
            if z != Z_BASE:
                dd = c - b
                line += (f'; {int(np.sum(dd > 0))} / {int(np.sum(dd < 0))} / {int(np.sum(dd == 0))}; '
                         f'{dd.mean():+.3f} ± {se(dd):.3f} ({dd.mean() / se(dd):+.1f} s.e.)')
            print(line)
    print('  Pass-count steps (n drops from k to k-1 passes at z_k = 13.33 * 2 / (2 * 1.5 * k)): '
          + ', '.join(f'k = {k}: {STRIP_W * Z_REF / (2 * R0 * k):.4f} m' for k in range(3, 10)
                      if Z_RANGE[0] < STRIP_W * Z_REF / (2 * R0 * k) < Z_RANGE[1]))
    print('  Altitudes just below / just above each step: ' + ', '.join(f'{z:.2f} m ({n_passes(z)} passes)'
                                                                   for z in Z_NEAR))
    for name, key, st in (('PI', 'oki', 'sti'), ('card PID', 'okp', 'stp')):
        thr = {z: 60 * rows[z][key] / rows[z]['mis'] for z in Z_GRID}
        zb = max(Z_GRID, key=lambda z: thr[z].mean())
        dd = thr[zb] - thr[Z_BASE]
        dc = thr[zb] - thr[z_card]
        sc = _agg(trials, zb, lambda r: r['sweep_clear'])
        print(f'  Highest throughput ({name}) on the grid: {zb:.2f} m ({n_passes(zb)} passes), '
              f'{thr[zb].mean():.3f} ± {se(thr[zb]):.3f}/min; vs {Z_BASE} m: higher in {int(np.sum(dd > 0))}/'
              f'{N_TRIALS}, {dd.mean():+.3f} ± {se(dd):.3f} ({dd.mean() / se(dd):+.1f} s.e.); vs card z* '
              f'{z_card} m: higher in {int(np.sum(dc > 0))}/{N_TRIALS}, {dc.mean():+.3f} ± {se(dc):.3f} '
              f'({dc.mean() / se(dc):+.1f} s.e.); minimum sweep clearance {sc.min():.3f} m, canopy contacts '
              f'{rows[zb][st].mean():.2f} per trial')
    for zc in sorted({z_card, best_ok} - {Z_BASE}):
        b, c = rows[Z_BASE], rows[zc]
        print(f"  z = {zc} m vs 2D base altitude {Z_BASE} m (per trial, CRN): treated PI more in "
              f"{int(np.sum(c['oki'] > b['oki']))}, fewer in {int(np.sum(c['oki'] < b['oki']))}/{N_TRIALS} "
              f"(mean diff {np.mean(c['oki'] - b['oki']):+.2f}); detected more in "
              f"{int(np.sum(c['det'] > b['det']))}, fewer in {int(np.sum(c['det'] < b['det']))}; "
              f"mission time diff {np.mean(c['mis'] - b['mis']):+.1f} s")
    zr = np.concatenate([t['zweed'] / t['hw'] for t in trials])
    print(f'  Weed height below the canopy (card geometry): z_weed / h = {zr.mean():.3f} ± {zr.std():.3f}; '
          f'it is used only in the 3D plots - no result above depends on it (treatment targets the canopy '
          f'top above the weed)')
    rp_ = np.concatenate([t['per_z'][z_card]['pid']['rms'] for t in trials])
    ri_ = np.concatenate([t['per_z'][z_card]['pi']['rms'] for t in trials])
    print(f'  Hover at z = {z_card} m ({rp_.size} hovers): 3D RMS card PID {rp_.mean():.4f} m '
          f'({100 * np.mean(rp_ <= EPS3):.2f} % <= {EPS3} m), PI {ri_.mean():.4f} m '
          f'({100 * np.mean(ri_ <= EPS3):.2f} %)')
    print('  Sweep clearance above the highest canopy under the passes: z -> min / mean, trials '
          f'with clearance < {DH_CLEAR} m (mission invalid) / < 0 (sweep touches the crop)')
    for z in Z_GRID:
        sc = _agg(trials, z, lambda r: r['sweep_clear'])
        print(f'    {z:4.2f} m: {sc.min():+.3f} / {sc.mean():+.3f} m, {int(np.sum(sc < DH_CLEAR))} / '
              f'{int(np.sum(sc < 0))} of {N_TRIALS}')
    print('  Canopy contacts vs canopy-estimate error (PI): z -> contacts with error < -0.1 m / all')
    for z in Z_GRID:
        a = sum(int(np.sum(t['per_z'][z]['pi']['strike'] & (t['per_z'][z]['dz_est'] < -0.1)))
                for t in trials)
        b = sum(int(np.sum(t['per_z'][z]['pi']['strike'])) for t in trials)
        print(f'    {z:4.2f} m: {a} / {b}')
    print('  Vertical arrival-offset sensitivity (PI, sd per hover): z, sd -> treated per trial, contacts '
          'per trial')
    for z in SDZ_Z:
        for sdz in SDZ_SWEEP:
            ok = _agg(trials, z, lambda r: r[f'pi_sdz{sdz}']['ok'].sum())
            st = _agg(trials, z, lambda r: r[f'pi_sdz{sdz}']['strike'].sum())
            print(f'    z = {z} m, sd_z = {sdz} m: {_ms(ok)} treated, {_ms(st)} contacts')
    t0 = trials[0]['per_z'][z_card]
    print(f"  Trial 0 at z = {z_card} m: detected {len(t0['idx'])}/{N_WEEDS}, treated PID "
          f"{int(t0['pid']['ok'].sum())}, PI {int(t0['pi']['ok'].sum())}, Phase 1 {t0['t1']:.1f} s, "
          f"makespan {t0['mk3']:.1f} s")
    return z_card


if __name__ == '__main__':
    tr = run_simulation()
    zc = report(tr)
    od = os.path.normpath(OUTPUT_DIR)
    os.makedirs(od, exist_ok=True)
    plot_canopy_detection(tr, od, zc)
    plot_field_3d(tr, od, zc)
    plot_altitude(tr, od, zc)
    plot_metrics(tr, od, zc)
    plot_altitude_sweep(tr, od)
    save_animation(tr, od, zc)
