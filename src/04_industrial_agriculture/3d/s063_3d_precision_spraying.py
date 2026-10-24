"""
S063 3D Upgrade — Precision Per-Plant Spraying
==============================================
The S063 mission (20 plants in a 20 x 20 m field, nearest-neighbour tour from the depot,
1 m/s reference legs, settle test, 3 s spray window, PID with GPS noise, 3 m/s^2
acceleration limit, sinusoidal gust + turbulence, integrator reset per spray window) is
flown in 3D over plants with canopy heights h_i ~ TN(1.0, 0.3^2; 0.3, 2.5) m and crowns of
radius 0.5 m.  A vertical PID loop (gains 1.5 x the horizontal set) holds altitude against
a rotor-downwash disturbance -0.15 m/s^2 + N(0, 0.05^2).

Altitude strategies (same plants, heights, wind, GPS and altimeter noise per seed, CRN):
  fixed15   card: z = 1.5 m everywhere (S063 behaviour, the 2D baseline)
  fixedsafe added 2D baseline that clears every plant: z = max_i h_i + 0.5 m
  adaptive  card: z_cmd = h_i + 0.5 m per plant (prior height map); on each leg the reference is
            max(previous, next z_cmd): climb first, descend over the plant
  tf        card: lidar terrain following, z_ref = canopy surface below the drone + 0.5 m
            (lidar noise 0.05 m), continuous

Dose (card, with the fall-time units fixed): per spray sample the droplet falls the
standoff dz = z - h_i with initial downwash speed w_dw, t_fall = (sqrt(w_dw^2 + 2 g dz) - w_dw)/g,
and drifts with the wind velocity u = (gust acceleration)/c_w;
dose = exp(-d^2 / 2 sigma_spray^2) exp(-(dz - 0.5)^2 / 2 sigma_z^2), 0 inside the canopy.

Usage:
    conda activate drones
    python src/04_industrial_agriculture/3d/s063_3d_precision_spraying.py
"""

import os
import math
import importlib.util
import numpy as np
from scipy.stats import norm
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401

_HERE = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location('s063_2d', os.path.join(_HERE, '..', 's063_precision_spraying.py'))
m2d = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(m2d)

OUTPUT_DIR = os.path.normpath(os.path.join(
    _HERE, '..', '..', '..', 'outputs', '04_industrial_agriculture', '3d', 's063_3d_precision_spraying'))

# ── Parameters (card / S063) ──────────────────────────────────────────────────
FIELD, N_PLANTS, DEPOT = m2d.FIELD_SIZE, m2d.N_PLANTS, m2d.DEPOT
SIGMA_SPRAY, DT, T_SPRAY, V_TRANSIT = m2d.SIGMA_SPRAY, m2d.DT, m2d.T_SPRAY, m2d.V_TRANSIT
A_MAX, GPS_SIGMA = m2d.A_MAX, m2d.GPS_SIGMA
A_WIND, T_WIND, SIGMA_W = m2d.A_WIND, m2d.T_WIND, m2d.SIGMA_W_BASE
MU_H, SIGMA_H, H_MIN, H_MAX = 1.0, 0.3, 0.3, 2.5
SIGMA_LIDAR = 0.05
DZ_NOM = 0.5
Z_MIN, Z_MAX = 0.3, 6.0
ALPHA_Z = 1.5
SIGMA_Z = 0.2
F_DW, SIGMA_DW = 0.15, 0.05
Z_FIXED = 1.5
G = 9.81
GAINS = {'Tuned': m2d.PID_CONFIGS['Tuned'], 'Tight': m2d.PID_CONFIGS['Tight']}

# ── Parameters (added) ────────────────────────────────────────────────────────
R_CROWN = 0.5                 # m crown radius (canopy surface for lidar and contact checks)
C_W = 0.3                     # 1/s: wind velocity = gust acceleration / C_W (1 m/s amplitude)
W_DW = 2.0                    # m/s initial downward droplet speed from rotor downwash
C_W_SWEEP = [0.15, 0.3, 0.6]
W_DW_SWEEP = [0.0, 2.0, 5.0]
SETTLE_POS, SETTLE_VEL, SETTLE_Z, SETTLE_HOLD, SETTLE_MAX = 0.3, 0.1, 0.15, 0.2, 10.0
SURF_RES = 0.05               # m lidar surface grid
N_SEEDS = 20
SIGMA_H_SWEEP = [0.0, 0.15, 0.3, 0.45, 0.6]
N_SWEEP_SEEDS = 10
R_CROWN_SWEEP = [0.25, 0.5, 1.0]  # crown-radius sensitivity (added)
KDZ_PROBE = 2.5                  # vertical-damping probe: k_d,z x 2.5
BOUND_CONTACT_MAX = 1.0          # s, longest allowed continuous contact with the altitude bound (added)
T_MAX = 700.0
PLOT_SEED = 0
ANIM_STEP = 100

STRATS = ['fixed15', 'fixedsafe', 'adaptive', 'tf']
LABELS = {'fixed15': 'Fixed 1.5 m (card)', 'fixedsafe': 'Fixed, clears all plants',
          'adaptive': 'Per-plant adaptive', 'tf': 'Lidar terrain following'}
COLORS = {'fixed15': '#7F7F7F', 'fixedsafe': '#9467BD', 'adaptive': '#2CA02C', 'tf': '#1F77B4'}

PLANTS = m2d.make_plants()
TOUR = m2d.tsp_nearest_neighbour(PLANTS)


def heights(seed, sigma_h):
    """Truncated normal by inverse CDF of a per-seed uniform (same uniforms for every sigma_h)."""
    u = np.random.default_rng(3000 + seed).random(N_PLANTS)
    if sigma_h == 0:
        return np.full(N_PLANTS, MU_H)
    a, b = norm.cdf((H_MIN - MU_H) / sigma_h), norm.cdf((H_MAX - MU_H) / sigma_h)
    return MU_H + sigma_h * norm.ppf(a + u * (b - a))


def surface_grid(h, rc=R_CROWN):
    g = np.arange(-2.0, FIELD + 2.0 + 1e-9, SURF_RES)
    X, Y = np.meshgrid(g, g, indexing='ij')
    S = np.zeros_like(X)
    for (px, py), hh in zip(PLANTS, h):
        S = np.where(np.hypot(X - px, Y - py) <= rc, np.maximum(S, hh), S)
    return g[0], S


def run_mission(args):
    strategy, gname, sigma_h, seed, keep_log = args[:5]
    rc = args[5] if len(args) > 5 else R_CROWN                 # crown radius (sweep)
    kdz_scale = args[6] if len(args) > 6 else 1.0               # vertical damping probe
    gains = GAINS[gname]
    kp, ki, kd = gains['kp'], gains['ki'], gains['kd']
    kpz, kiz, kdz = ALPHA_Z * kp, ALPHA_Z * ki, kdz_scale * ALPHA_Z * kd
    h = heights(seed, sigma_h)
    n_max = int(T_MAX / DT)
    phases, turb, gps = m2d.noise(seed, SIGMA_W, n_max)
    rz = np.random.default_rng(5000 + seed)
    dw = (-F_DW + SIGMA_DW * rz.standard_normal(n_max)).tolist()
    alt_noise = (SIGMA_LIDAR * rz.standard_normal(n_max)).tolist()
    tx, ty = turb[:, 0].tolist(), turb[:, 1].tolist()
    gx, gy = gps[:, 0].tolist(), gps[:, 1].tolist()
    s0, S = surface_grid(h, rc)
    nS = S.shape[0]
    Sl = S.tolist()
    z_safe = float(h.max()) + DZ_NOM
    zc = {'fixed15': [Z_FIXED] * N_PLANTS, 'fixedsafe': [z_safe] * N_PLANTS,
          'adaptive': (h + DZ_NOM).tolist(), 'tf': (h + DZ_NOM).tolist()}[strategy]
    zc = [min(max(z, Z_MIN), Z_MAX) for z in zc]
    st = {'px': float(DEPOT[0]), 'py': float(DEPOT[1]),
          'pz': zc[TOUR[0]] if strategy != 'tf' else DZ_NOM,
          'vx': 0.0, 'vy': 0.0, 'vz': 0.0, 'ix': 0.0, 'iy': 0.0, 'iz': 0.0, 'k': 0,
          'nsat': 0, 'nclamp': 0, 'crun': 0, 'cmax': 0}
    X, Y, Z, CUR = [], [], [], []
    w_g = 2 * math.pi / T_WIND

    def surf(x, y):
        i = int(round((x - s0) / SURF_RES))
        j = int(round((y - s0) / SURF_RES))
        if 0 <= i < nS and 0 <= j < nS:
            return Sl[i][j]
        return 0.0

    def step(rx, ry, vrx, vry, zref, ph):
        k = st['k']
        t = k * DT
        px, py, pz = st['px'], st['py'], st['pz']
        ex, ey = rx - (px + gx[k]), ry - (py + gy[k])
        if strategy == 'tf':
            ez = DZ_NOM - (pz - surf(px, py) + alt_noise[k])       # lidar standoff
        else:
            ez = zref - (pz + alt_noise[k])
        st['ix'] += ex * DT
        st['iy'] += ey * DT
        st['iz'] += ez * DT
        ax = kp * ex + ki * st['ix'] + kd * (vrx - st['vx'])
        ay = kp * ey + ki * st['iy'] + kd * (vry - st['vy'])
        az = kpz * ez + kiz * st['iz'] + kdz * (0.0 - st['vz'])
        if abs(ax) > A_MAX or abs(ay) > A_MAX or abs(az) > A_MAX:
            st['nsat'] += 1
        ax = min(max(ax, -A_MAX), A_MAX)
        ay = min(max(ay, -A_MAX), A_MAX)
        az = min(max(az, -A_MAX), A_MAX)
        st['vx'] += (ax + A_WIND * math.sin(w_g * t + ph[0]) + tx[k]) * DT
        st['vy'] += (ay + A_WIND * math.sin(w_g * t + ph[1]) + ty[k]) * DT
        st['vz'] += (az + dw[k]) * DT
        st['px'] = px + st['vx'] * DT
        st['py'] = py + st['vy'] * DT
        nz = pz + st['vz'] * DT
        if nz < Z_MIN or nz > Z_MAX:                            # card altitude bounds
            st['nclamp'] += 1
            st['crun'] += 1
            nz = min(max(nz, Z_MIN), Z_MAX)
            st['vz'] = 0.0
        else:
            st['crun'] = 0
        st['cmax'] = max(st['cmax'], st['crun'])
        # a brief touch of the altitude bound is part of the model; resting on it is a landing/crash
        assert st['crun'] * DT <= BOUND_CONTACT_MAX, 'drone resting on the altitude bound (landed or stuck)'
        st['pz'] = nz
        X.append(st['px']); Y.append(st['py']); Z.append(nz); CUR.append(st['cur'])
        st['k'] = k + 1

    hover = []
    waits, timeouts = np.zeros(N_PLANTS), 0
    prev = DEPOT.copy()
    z_prev = st['pz']
    n_spray = int(round(T_SPRAY / DT))
    st['cur'] = -1
    climb_leg = np.zeros(N_PLANTS, dtype=bool)                  # leg to this plant climbs to a taller plant
    for iv, pi in enumerate(TOUR):
        st['cur'] = int(pi)
        climb_leg[pi] = zc[pi] > z_prev + 1e-12
        cx, cy = float(PLANTS[pi, 0]), float(PLANTS[pi, 1])
        ph = (float(phases[iv, 0]), float(phases[iv, 1]))
        z_t = zc[pi]
        leg = PLANTS[pi] - prev
        L = float(np.linalg.norm(leg))
        ux, uy = float(leg[0] / L), float(leg[1] / L)
        n_leg = int(np.ceil(L / (V_TRANSIT * DT)))
        for j in range(n_leg):
            s = min((j + 1) * V_TRANSIT * DT, L)
            mv = s < L
            step(prev[0] + s * ux, prev[1] + s * uy, V_TRANSIT * ux if mv else 0.0,
                 V_TRANSIT * uy if mv else 0.0, max(z_prev, z_t), ph)        # climb first, descend on arrival
        held = n_wait = 0
        n_wmax = int(round(SETTLE_MAX / DT))
        while n_wait < n_wmax:
            k = st['k']
            ezm = (DZ_NOM - (st['pz'] - surf(st['px'], st['py']) + alt_noise[k])) if strategy == 'tf' \
                else z_t - (st['pz'] + alt_noise[k])
            ok = (math.hypot(cx - st['px'] - gx[k], cy - st['py'] - gy[k]) < SETTLE_POS and
                  math.sqrt(st['vx'] ** 2 + st['vy'] ** 2 + st['vz'] ** 2) < SETTLE_VEL and abs(ezm) < SETTLE_Z)
            held = held + 1 if ok else 0
            if held * DT >= SETTLE_HOLD - 1e-9:
                break
            step(cx, cy, 0.0, 0.0, z_t, ph)
            n_wait += 1
        waits[iv] = n_wait * DT
        timeouts += int(n_wait >= n_wmax)
        st['ix'] = st['iy'] = st['iz'] = 0.0
        k0 = st['k']
        for j in range(n_spray):
            step(cx, cy, 0.0, 0.0, z_t, ph)
        hover.append((pi, k0, st['k'], ph))
        prev = PLANTS[pi].copy()
        z_prev = z_t
    X, Y, Z = np.array(X), np.array(Y), np.array(Z)
    # canopy contact: drone inside any crown cylinder below its top
    CUR = np.array(CUR)
    contact = np.zeros(N_PLANTS, dtype=bool)
    own = np.zeros(N_PLANTS, dtype=bool)          # contact with the plant currently being approached/sprayed
    n_contact = 0
    for j in range(N_PLANTS):
        m = (np.hypot(X - PLANTS[j, 0], Y - PLANTS[j, 1]) < rc) & (Z < h[j])
        contact[j] = m.any()
        own[j] = (m & (CUR == j)).any()
        n_contact += int(m.sum())
    sp = np.hypot(np.diff(np.concatenate([[DEPOT[0]], X])), np.diff(np.concatenate([[DEPOT[1]], Y])))
    out = {'h': h, 'hover': [], 'time': st['k'] * DT, 'timeouts': timeouts, 'wait': waits.mean(),
           'sat': st['nsat'] / st['k'], 'clamp': st['nclamp'], 'clamp_run': st['cmax'] * DT, 'contact_plants': int(contact.sum()),
           'contact_own': int(own.sum()), 'contact_other': int((contact & ~own).sum()),
           'contact_own_climb': int((own & climb_leg).sum()), 'contact_own_descent': int((own & ~climb_leg).sum()),
           'contact_steps': n_contact, 'z_safe': z_safe, 'max_step': float(sp.max()),
           'climb': float(np.sum(np.abs(np.diff(Z)))), 'zmin': float(Z.min()), 'zmax': float(Z.max())}
    for (pi, a, b, ph) in hover:
        out['hover'].append({'plant': pi, 't': np.arange(a, b) * DT, 'x': X[a:b], 'y': Y[a:b], 'z': Z[a:b],
                             'ph': ph})
    if keep_log:
        out['traj'] = np.column_stack([X, Y, Z])
    return out


def dose_eval(res, c_w=C_W, w_dw=W_DW, alt_factor=True):
    """Per-plant dose, impact offset, standoff error and 3D/horizontal hover RMS from logged samples."""
    h = res['h']
    w_g = 2 * math.pi / T_WIND
    D, dmean, sdev, rms3, rmsh, dzm = (np.zeros(N_PLANTS) for _ in range(6))
    for hv in res['hover']:
        i = hv['plant']
        dz = hv['z'] - h[i]
        inside = dz <= 0
        tf = np.where(inside, 0.0, (np.sqrt(w_dw ** 2 + 2 * G * np.maximum(dz, 0)) - w_dw) / G)
        ux = A_WIND * np.sin(w_g * hv['t'] + hv['ph'][0]) / c_w
        uy = A_WIND * np.sin(w_g * hv['t'] + hv['ph'][1]) / c_w
        ix, iy = hv['x'] + ux * tf, hv['y'] + uy * tf
        d = np.hypot(ix - PLANTS[i, 0], iy - PLANTS[i, 1])
        dose = np.exp(-d ** 2 / (2 * SIGMA_SPRAY ** 2))
        if alt_factor:
            dose = dose * np.exp(-(dz - DZ_NOM) ** 2 / (2 * SIGMA_Z ** 2))
        dose[inside] = 0.0
        D[i] = dose.mean()
        dmean[i] = d.mean()
        sdev[i] = np.mean(np.abs(dz - DZ_NOM))
        dzm[i] = dz.mean()
        e3 = (hv['x'] - PLANTS[i, 0]) ** 2 + (hv['y'] - PLANTS[i, 1]) ** 2
        rmsh[i] = np.sqrt(np.mean(e3))
        rms3[i] = np.sqrt(np.mean(e3 + (hv['z'] - h[i] - DZ_NOM) ** 2))
    return {'dose': D, 'ceff': float(D.mean()), 'drift': dmean, 'sdev': sdev, 'rms3': rms3, 'rmsh': rmsh, 'dz': dzm}


def run_simulation():
    jobs = [(s, g, SIGMA_H, seed, seed == PLOT_SEED and g == 'Tuned')
            for g in GAINS for s in STRATS for seed in range(N_SEEDS)]
    jobs += [(s, 'Tuned', sh, seed, False) for sh in SIGMA_H_SWEEP if sh != SIGMA_H
             for s in STRATS for seed in range(N_SWEEP_SEEDS)]
    jobs += [(s, g, SIGMA_H, seed, False, rc) for rc in R_CROWN_SWEEP if rc != R_CROWN
             for g in GAINS for s in ('fixed15', 'adaptive', 'tf') for seed in range(N_SWEEP_SEEDS)]
    jobs += [('adaptive', g, SIGMA_H, seed, False, R_CROWN, KDZ_PROBE) for g in GAINS for seed in range(N_SEEDS)]
    outs = [run_mission(j) for j in jobs]
    res = {}
    for j, o in zip(jobs, outs):
        res.setdefault(tuple(j[:3]) + tuple(j[5:]), []).append(o)
    return res


# ── Plots ─────────────────────────────────────────────────────────────────────

def _save(fig, out_dir, name):
    path = os.path.join(out_dir, name)
    fig.savefig(path, dpi=120, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_field_3d(res, out_dir):
    fig = plt.figure(figsize=(15, 6.5))
    for c, s in enumerate(['adaptive', 'fixed15']):
        r = res[(s, 'Tuned', SIGMA_H)][PLOT_SEED]
        ax = fig.add_subplot(1, 2, c + 1, projection='3d')
        tr = r['traj']
        ax.plot(tr[::5, 0], tr[::5, 1], tr[::5, 2], color='red', lw=0.6, label='drone')
        h = r['h']
        for i in range(N_PLANTS):
            ax.plot([PLANTS[i, 0]] * 2, [PLANTS[i, 1]] * 2, [0, h[i]], color='green', lw=3)
            zh = res[(s, 'Tuned', SIGMA_H)][PLOT_SEED]['hover'][TOUR.index(i)]['z'].mean()
            ax.plot([PLANTS[i, 0]] * 2, [PLANTS[i, 1]] * 2, [h[i], zh], 'k:', lw=0.8)
        ax.scatter(PLANTS[:, 0], PLANTS[:, 1], h, color='green', s=30, label='canopy top h_i')
        ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)'); ax.set_zlabel('z (m)')
        ax.set_zlim(0, 3)
        ax.view_init(elev=25, azim=-60)
        ax.set_title(f'{LABELS[s]}, Tuned PID, seed {PLOT_SEED}\ndotted = standoff during spraying', fontsize=10)
        ax.legend(fontsize=8)
    fig.suptitle('S063 3D field overview (green stems = canopy height)')
    _save(fig, out_dir, 'field_3d.png')


def plot_altitude(res, out_dir):
    fig, axes = plt.subplots(len(STRATS), 1, figsize=(14, 11), sharex=True)
    for ax, s in zip(axes, STRATS):
        r = res[(s, 'Tuned', SIGMA_H)][PLOT_SEED]
        tr = r['traj']
        t = (np.arange(len(tr)) + 1) * DT
        ax.plot(t, tr[:, 2], color=COLORS[s], lw=0.8, label='z(t)')
        for hv in r['hover']:
            i = hv['plant']
            ax.fill_between(hv['t'], r['h'][i] + DZ_NOM - SIGMA_Z, r['h'][i] + DZ_NOM + SIGMA_Z, color='green', alpha=0.25)
            ax.plot(hv['t'], np.full_like(hv['t'], r['h'][i]), color='saddlebrown', lw=2)
        ax.set_ylabel('z (m)')
        ax.set_title(f'{LABELS[s]}: brown = canopy top of the plant being sprayed, green = h_i + 0.5 ± σ_z', fontsize=9)
        ax.legend(fontsize=7, loc='upper right')
    axes[-1].set_xlabel('mission time (s)')
    fig.suptitle(f'S063 3D Altitude time series, Tuned PID, seed {PLOT_SEED}')
    plt.tight_layout()
    _save(fig, out_dir, 'altitude_timeseries.png')


def plot_standoff(res, out_dir):
    fig, ax = plt.subplots(figsize=(14, 4.8))
    w = 0.2
    x = np.arange(N_PLANTS)
    for j, s in enumerate(STRATS):
        sd = np.mean([dose_eval(r)['sdev'][TOUR] for r in res[(s, 'Tuned', SIGMA_H)]], axis=0)
        ax.bar(x + (j - 1.5) * w, 100 * sd, w, color=COLORS[s], label=LABELS[s])
    ax.set_xticks(x, [f'{n + 1}' for n in range(N_PLANTS)])
    ax.set_xlabel('visit order')
    ax.set_ylabel('mean |δz − 0.5 m| during spraying (cm)')
    ax.set_yscale('log')
    ax.legend(fontsize=8)
    ax.set_title(f'S063 3D Standoff deviation per plant, mean over {N_SEEDS} seeds (Tuned PID)', fontsize=10)
    _save(fig, out_dir, 'standoff_deviation.png')


def plot_impacts(res, out_dir):
    fig, axes = plt.subplots(1, 2, figsize=(14, 6.6))
    w_g = 2 * math.pi / T_WIND
    for ax, s in zip(axes, ['adaptive', 'fixed15']):
        r = res[(s, 'Tuned', SIGMA_H)][PLOT_SEED]
        h = r['h']
        for hv in r['hover']:
            i = hv['plant']
            dz = hv['z'] - h[i]
            tf = np.where(dz <= 0, 0.0, (np.sqrt(W_DW ** 2 + 2 * G * np.maximum(dz, 0)) - W_DW) / G)
            ix = hv['x'] + A_WIND * np.sin(w_g * hv['t'] + hv['ph'][0]) / C_W * tf
            iy = hv['y'] + A_WIND * np.sin(w_g * hv['t'] + hv['ph'][1]) / C_W * tf
            ax.scatter(ix[::5], iy[::5], s=2, color='tab:blue', alpha=0.5)
            ax.add_patch(plt.Circle(PLANTS[i], SIGMA_SPRAY, fill=False, color='green', lw=0.8))
        ax.scatter(PLANTS[:, 0], PLANTS[:, 1], color='green', s=25, zorder=5, label='plant centre')
        dv = dose_eval(r)
        ax.set_title(f"{LABELS[s]}: C_eff = {dv['ceff']:.3f}, mean drift-offset d = {100 * dv['drift'].mean():.1f} cm",
                     fontsize=10)
        ax.set_aspect('equal'); ax.set_xlim(-0.5, 20.5); ax.set_ylim(-0.5, 20.5)
        ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')
        ax.legend(fontsize=8)
    fig.suptitle(f'S063 3D Droplet impact points (every 5th spray sample), Tuned PID, seed {PLOT_SEED}; circles: σ_spray')
    _save(fig, out_dir, 'droplet_impacts.png')


def plot_sweep(res, out_dir):
    fig, ax = plt.subplots(figsize=(8, 5))
    for s in STRATS:
        m, sd = [], []
        for sh in SIGMA_H_SWEEP:
            rs = res[(s, 'Tuned', sh)][:N_SWEEP_SEEDS]
            c = [dose_eval(r)['ceff'] for r in rs]
            m.append(np.mean(c)); sd.append(np.std(c))
        ax.errorbar(SIGMA_H_SWEEP, m, yerr=sd, marker='o', capsize=3, color=COLORS[s], label=LABELS[s])
    ax.set_xlabel('canopy height std σ_h (m)')
    ax.set_ylabel(f'C_eff 3D (mean ± std, {N_SWEEP_SEEDS} seeds)')
    ax.set_ylim(0, 1)
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    ax.set_title('S063 3D Coverage vs canopy-height variability (Tuned PID)', fontsize=10)
    _save(fig, out_dir, 'ceff_vs_sigma_h.png')


def save_animation(res, out_dir):
    r = res[('tf', 'Tuned', SIGMA_H)][PLOT_SEED]
    tr = r['traj']
    n = len(tr)
    nf = int(np.ceil((n - 1) / ANIM_STEP)) + 1
    idx = [min(i * ANIM_STEP, n - 1) for i in range(nf)]
    h = r['h']
    fig = plt.figure(figsize=(6.4, 5.6))
    ax = fig.add_subplot(111, projection='3d')

    def update(f):
        k = idx[f]
        ax.cla()
        for i in range(N_PLANTS):
            ax.plot([PLANTS[i, 0]] * 2, [PLANTS[i, 1]] * 2, [0, h[i]], color='green', lw=3)
        ax.plot(tr[:k + 1:10, 0], tr[:k + 1:10, 1], tr[:k + 1:10, 2], color='red', lw=0.7)
        ax.scatter(*tr[k], color='red', s=25)
        ax.set_xlim(0, 20); ax.set_ylim(0, 20); ax.set_zlim(0, 3)
        ax.view_init(elev=25, azim=-60)
        ax.set_title(f'S063 3D lidar terrain following, t = {(k + 1) * DT:.0f} s, z = {tr[k, 2]:.2f} m', fontsize=9)
        return []

    anim = animation.FuncAnimation(fig, update, frames=nf, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    anim.save(path, writer='pillow', fps=8, dpi=65)
    plt.close(fig)
    from PIL import Image
    with Image.open(path) as gif:
        nfile = gif.n_frames
    print(f'Saved: {path} ({nf} frames rendered, {nfile} in the file, every {ANIM_STEP} steps = {ANIM_STEP * DT:.0f} s)')


# ── Report ────────────────────────────────────────────────────────────────────

def wins(a, b):
    return int(np.sum(a > b + 1e-12)), int(np.sum(np.abs(a - b) <= 1e-12)), int(np.sum(a < b - 1e-12))


def report(res):
    print('\n=== S063 3D Precision Per-Plant Spraying — Results ===')
    print(f'  NN tour {m2d.path_length(PLANTS, TOUR):.2f} m (S063)')
    h0 = heights(0, SIGMA_H)
    hs = np.concatenate([heights(s, SIGMA_H) for s in range(N_SEEDS)])
    print(f'  Heights ({N_SEEDS} seeds x {N_PLANTS}): mean {hs.mean():.3f} m, std {hs.std():.3f} m, '
          f'min {hs.min():.3f}, max {hs.max():.3f}; above 1.5 m: {int(np.sum(hs > Z_FIXED))} plants '
          f'({100 * np.mean(hs > Z_FIXED):.1f} %); within 0.2 m below 1.5 m: {int(np.sum((hs <= Z_FIXED) & (hs > Z_FIXED - 0.2)))}')
    print(f'  Seed 0 heights {np.round(h0, 2).tolist()}')
    for dz in (0.25, 0.5, 1.0, 1.5):
        print(f'  Fall time at standoff {dz:.2f} m: w_dw 0 -> {math.sqrt(2 * dz / G):.3f} s, w_dw 2 m/s -> '
              f'{(math.sqrt(4 + 2 * G * dz) - 2) / G:.3f} s, w_dw 5 m/s -> {(math.sqrt(25 + 2 * G * dz) - 5) / G:.3f} s')
    print(f'  Card formula sqrt(2 dz / (g + w_dw)) mixes m/s^2 and m/s (dimension error).')
    for g in GAINS:
        kp, ki, kd = (ALPHA_Z * GAINS[g][k] for k in ('kp', 'ki', 'kd'))
        poles = np.roots([1, kd, kp, ki])
        cp = poles[np.argmax(np.abs(poles.imag))]
        print(f'  Vertical loop {g} x{ALPHA_Z} ({kp:.2f}, {ki:.2f}, {kd:.3f}): poles ' +
              ', '.join(f'{p.real:.3f}{p.imag:+.3f}j' for p in sorted(poles, key=lambda p: (p.real, p.imag))) +
              f'; damping of the complex pair {-cp.real / abs(cp):.3f}')
    for g in GAINS:
        print(f'\n  --- {g} PID (horizontal {GAINS[g]}, vertical x{ALPHA_Z}), sigma_h = {SIGMA_H}, {N_SEEDS} seeds ---')
        ce = {}
        for s in STRATS:
            rs = res[(s, g, SIGMA_H)]
            ev = [dose_eval(r) for r in rs]
            c = np.array([e['ceff'] for e in ev])
            ce[s] = c
            a = lambda k: np.mean([e[k].mean() for e in ev])
            print(f"    {LABELS[s]:26s} C_eff {c.mean():.3f} +- {c.std():.3f} | hover RMS 3D {100 * a('rms3'):.1f} cm, "
                  f"horiz {100 * a('rmsh'):.1f} cm | standoff dev {100 * a('sdev'):.1f} cm | drift offset d "
                  f"{100 * a('drift'):.1f} cm | plants with dose > 0.9 {100 * np.mean([np.mean(e['dose'] > 0.9) for e in ev]):.1f} % | "
                  f"canopy contact: plants {np.mean([r['contact_plants'] for r in rs]):.2f}/mission "
                  f"(target plant {np.mean([r['contact_own'] for r in rs]):.2f} = on a climb leg "
                  f"{np.mean([r['contact_own_climb'] for r in rs]):.2f} + on a descent/level leg "
                  f"{np.mean([r['contact_own_descent'] for r in rs]):.2f}, other plants "
                  f"{np.mean([r['contact_other'] for r in rs]):.2f}) "
                  f"(missions with any {sum(r['contact_plants'] > 0 for r in rs)}/{N_SEEDS}), "
                  f"plants sprayed from inside the crown {np.mean([np.sum(e['dz'] <= 0) for e in ev]):.2f} | "
                  f"time {np.mean([r['time'] for r in rs]):.1f} s, time-outs {np.mean([r['timeouts'] for r in rs]):.1f}/20, "
                  f"saturated {100 * np.mean([r['sat'] for r in rs]):.2f} %, altitude clamps {np.mean([r['clamp'] for r in rs]):.1f} (longest continuous contact {max(r['clamp_run'] for r in rs):.2f} s), "
                  f"vertical travel {np.mean([r['climb'] for r in rs]):.1f} m, z range {min(r['zmin'] for r in rs):.2f}-"
                  f"{max(r['zmax'] for r in rs):.2f} m, max step {max(r['max_step'] for r in rs):.4f} m")
        for a_, b_ in [('adaptive', 'fixed15'), ('adaptive', 'fixedsafe'), ('adaptive', 'tf'), ('tf', 'fixed15')]:
            w = wins(ce[a_], ce[b_])
            print(f'    {LABELS[a_]} vs {LABELS[b_]}: higher C_eff in {w[0]} / equal {w[1]} / lower {w[2]} seeds; '
                  f'mean difference {ce[a_].mean() - ce[b_].mean():+.3f}')
    print(f'\n  --- sensitivity of mean C_eff to the added drift parameters (Tuned, sigma_h = {SIGMA_H}, {N_SEEDS} seeds) ---')
    for cw in C_W_SWEEP:
        for wd in W_DW_SWEEP:
            vals = {s: np.mean([dose_eval(r, cw, wd)['ceff'] for r in res[(s, 'Tuned', SIGMA_H)]]) for s in STRATS}
            drift = {s: np.mean([dose_eval(r, cw, wd)['drift'].mean() for r in res[(s, 'Tuned', SIGMA_H)]]) for s in STRATS}
            order = sorted(STRATS, key=lambda s: -vals[s])
            print(f'    c_w {cw:.2f} 1/s (wind amplitude {A_WIND / cw:.1f} m/s), w_dw {wd:.0f} m/s: ' +
                  ', '.join(f'{s} {vals[s]:.3f} (d {100 * drift[s]:.1f} cm)' for s in STRATS) + f' | ranking {" > ".join(order)}')
    print(f'\n  --- crown-radius sweep (sigma_h = {SIGMA_H}, {N_SWEEP_SEEDS} seeds; C_eff mean, crown contacts per mission) ---')
    for rc in R_CROWN_SWEEP:
        for g in GAINS:
            out = {}
            for s in ('fixed15', 'adaptive', 'tf'):
                key = (s, g, SIGMA_H) if rc == R_CROWN else (s, g, SIGMA_H, rc)
                rs = res[key][:N_SWEEP_SEEDS]
                out[s] = (np.array([dose_eval(r)['ceff'] for r in rs]), np.mean([r['contact_plants'] for r in rs]))
            print(f'    R_crown {rc:.2f} m, {g}: ' + ', '.join(
                f'{s} {out[s][0].mean():.3f} ({out[s][1]:.2f} contacts)' for s in out) +
                f"; tf > fixed15 in {wins(out['tf'][0], out['fixed15'][0])[0]}/{N_SWEEP_SEEDS}, "
                f"adaptive > tf in {wins(out['adaptive'][0], out['tf'][0])[0]}/{N_SWEEP_SEEDS}")
    print(f'\n  --- vertical-damping probe: adaptive with k_d,z x {KDZ_PROBE} (sigma_h = {SIGMA_H}, {N_SEEDS} seeds) ---')
    for g in GAINS:
        kp, ki, kd = (ALPHA_Z * GAINS[g][k] for k in ('kp', 'ki', 'kd'))
        for sc, key in ((1.0, ('adaptive', g, SIGMA_H)), (KDZ_PROBE, ('adaptive', g, SIGMA_H, R_CROWN, KDZ_PROBE))):
            poles = np.roots([1, sc * kd, kp, ki])
            cp = poles[np.argmax(np.abs(poles.imag))]
            rs = res[key]
            print(f'    {g}, k_d,z x {sc:g} (damping {-cp.real / abs(cp):.3f}): target-plant contacts '
                  f"{np.mean([r['contact_own'] for r in rs]):.2f}/mission (climb leg {np.mean([r['contact_own_climb'] for r in rs]):.2f}, "
                  f"descent/level leg {np.mean([r['contact_own_descent'] for r in rs]):.2f}), other plants "
                  f"{np.mean([r['contact_other'] for r in rs]):.2f}, z min {min(r['zmin'] for r in rs):.2f} m, "
                  f"C_eff {np.mean([dose_eval(r)['ceff'] for r in rs]):.3f}")
    print(f'\n  --- probe: dose without the standoff factor (drift and hover error only), sigma_h = {SIGMA_H}, {N_SEEDS} seeds ---')
    for g in GAINS:
        vals = {s: np.array([dose_eval(r, alt_factor=False)['ceff'] for r in res[(s, g, SIGMA_H)]]) for s in STRATS}
        print(f'    {g}: ' + ', '.join(f'{s} {vals[s].mean():.3f} +- {vals[s].std():.3f}' for s in STRATS) +
              f"; adaptive vs fixed15 higher/equal/lower {wins(vals['adaptive'], vals['fixed15'])}, "
              f"adaptive vs fixedsafe {wins(vals['adaptive'], vals['fixedsafe'])}")
    print(f'\n  --- C_eff vs sigma_h (Tuned, {N_SWEEP_SEEDS} seeds) ---')
    for sh in SIGMA_H_SWEEP:
        parts = []
        for s in STRATS:
            rs = res[(s, 'Tuned', sh)][:N_SWEEP_SEEDS]
            c = [dose_eval(r)['ceff'] for r in rs]
            parts.append(f"{s} {np.mean(c):.3f} +- {np.std(c):.3f} (contact plants {np.mean([r['contact_plants'] for r in rs]):.2f})")
        print(f'    sigma_h {sh:.2f}: ' + ' | '.join(parts))


if __name__ == '__main__':
    import sys
    import time
    t0 = time.time()
    res = run_simulation()
    report(res)
    out = OUTPUT_DIR
    os.makedirs(out, exist_ok=True)
    plot_field_3d(res, out)
    plot_altitude(res, out)
    plot_standoff(res, out)
    plot_impacts(res, out)
    plot_sweep(res, out)
    save_animation(res, out)
    print(f'[runtime {time.time() - t0:.0f} s]', file=sys.stderr)
