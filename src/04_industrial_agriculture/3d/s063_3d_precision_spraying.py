"""
S063 3D Upgrade — Precision Per-Plant Spraying
==============================================
The S063 mission (20 plants in a 20 x 20 m field, nearest-neighbour tour from the depot,
1 m/s reference legs, settle test, 3 s spray window, PID with GPS noise, 3 m/s^2
acceleration limit, sinusoidal gust + turbulence, integrator reset per spray window) is
flown in 3D over plants with canopy heights h_i ~ TN(1.0, 0.3^2; 0.3, 2.5) m and crowns of
radius 0.5 m.  A vertical PID loop (gains 1.5 x the horizontal set) holds altitude against
a rotor-downwash disturbance -0.15 m/s^2 + N(0, 0.05^2).  The card's altitude bounds
z in [0.3, 6] m are enforced through the controller: an acceleration-limited braking barrier
caps the vertical speed towards a bound so that full braking stops inside it (no state clamp,
no velocity reset); asserts check z >= 0.3 m, z > 0 and thrust acceleration <= A_MAX.
A mission with any crown contact is scored as a collision (safe C_eff 0).

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
SIGMA_H_LOW = 0.15               # low-spread case checked per seed and per clearance (added)
R_CROWN_SWEEP = [0.25, 0.5, 1.0]  # crown-radius sensitivity (added)
KDZ_PROBE = 2.5                  # vertical-damping probe: k_d,z x 2.5
# Altitude bounds [Z_MIN, Z_MAX] are enforced through the controller (added): a braking barrier limits the
# vertical speed towards a bound to what full braking can stop within the remaining distance.
DW_WORST = F_DW + 4 * SIGMA_DW   # m/s^2 worst-case downward downwash assumed by the barrier (4 sigma)
A_BRAKE = A_MAX - DW_WORST - 0.05  # m/s^2 braking deceleration the barrier plans with (0.05 margin)
Z_BUF = 0.005                    # m: the barrier stops this far inside the bound
BARRIER = True                   # set False only to show that the altitude asserts can fail
# Longest single braking manoeuvre: stopping from the fastest descent possible inside the altitude envelope
# (full thrust down plus worst downwash from Z_MAX to Z_MIN) at A_BRAKE (printed for reference).
V_ENVELOPE = math.sqrt(2 * (A_MAX + DW_WORST) * (Z_MAX - Z_MIN))
T_BARRIER_MAX = V_ENVELOPE / A_BRAKE
# Broken-loop check (added): the barrier is a safety override that acts only during transients (braking a
# descent, unwinding the integrator).  If it overrides the vertical command for more than half of the
# mission, the vertical loop is not flying the aircraft any more and the run is stopped.
BARRIER_SHARE_MAX = 0.5
# Clearance sweep (added): drone radius r_d widens the crown cylinder, vertical margin m_z raises its top.
CLEARANCES = [(0.0, 0.0), (0.1, 0.05), (0.2, 0.1)]
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
          'nsat': 0, 'nbar': 0, 'ninf': 0, 'brun': 0, 'bmax': 0}
    X, Y, Z, CUR, VZ, DWL = [], [], [], [], [], []
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
        active = False
        if BARRIER:                                             # acceleration-limited braking barrier
            vz = st['vz']
            for d, sgn, dw_bad in ((pz - Z_MIN - Z_BUF, -1.0, -DW_WORST),          # floor
                                   (Z_MAX - Z_BUF - pz, 1.0, -F_DW + 4 * SIGMA_DW)):  # ceiling
                d = max(d, 0.0)
                # exact discrete stopping speed: after this step's velocity v, braking by q = A_BRAKE DT per
                # step covers DT * sum_k max(v - k q, 0); v_ok is the largest v with that distance <= d
                q = A_BRAKE * DT
                n = int((-1.0 + math.sqrt(1.0 + 8.0 * d / (DT * q))) / 2.0)
                v_ok = (d / DT + q * n * (n + 1) / 2.0) / (n + 1)
                if sgn < 0:                                     # need vz_next >= -v_ok
                    a_req = (-v_ok - vz) / DT - dw_bad
                    if az < a_req:
                        st['nbar'] += 1
                        active = True
                        st['ninf'] += int(a_req > A_MAX)
                        az = min(a_req, A_MAX)
                else:                                           # need vz_next <= v_ok
                    a_req = (v_ok - vz) / DT - dw_bad
                    if az > a_req:
                        st['nbar'] += 1
                        active = True
                        st['ninf'] += int(a_req < -A_MAX)
                        az = max(a_req, -A_MAX)
        st['brun'] = st['brun'] + 1 if active else 0
        st['bmax'] = max(st['bmax'], st['brun'])
        st['vx'] += (ax + A_WIND * math.sin(w_g * t + ph[0]) + tx[k]) * DT
        st['vy'] += (ay + A_WIND * math.sin(w_g * t + ph[1]) + ty[k]) * DT
        st['vz'] += (az + dw[k]) * DT
        st['px'] = px + st['vx'] * DT
        st['py'] = py + st['vy'] * DT
        nz = pz + st['vz'] * DT
        assert nz > 0.0, 'drone hit the ground'
        assert Z_MIN <= nz <= Z_MAX, 'altitude bound violated'
        st['pz'] = nz
        X.append(st['px']); Y.append(st['py']); Z.append(nz); CUR.append(st['cur'])
        VZ.append(st['vz']); DWL.append(dw[k])
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
    assert st['nbar'] / st['k'] <= BARRIER_SHARE_MAX, 'barrier flies the drone: vertical loop broken'
    # vertical acceleration from the logged trajectory: thrust part within A_MAX (no velocity resets)
    acc = np.diff(np.concatenate([[0.0], VZ])) / DT - np.array(DWL)
    assert np.max(np.abs(acc)) <= A_MAX + 1e-6, 'vertical thrust acceleration above A_MAX'
    # canopy contact: drone inside any crown cylinder below its top
    CUR = np.array(CUR)
    contact = np.zeros(N_PLANTS, dtype=bool)
    own = np.zeros(N_PLANTS, dtype=bool)          # contact with the plant currently being approached/sprayed
    n_contact = 0
    any_step = np.zeros(len(X), dtype=bool)
    dist = np.hypot(X[:, None] - PLANTS[None, :, 0], Y[:, None] - PLANTS[None, :, 1])
    for j in range(N_PLANTS):
        m = (dist[:, j] < rc) & (Z < h[j])                     # point drone, zero clearance (main rule)
        contact[j] = m.any()
        own[j] = (m & (CUR == j)).any()
        n_contact += int(m.sum())
        any_step |= m
    contact_cl = tuple(int(np.sum(np.any((dist < rc + rd) & (Z[:, None] < h[None, :] + mz), axis=0)))
                       for rd, mz in CLEARANCES)
    sp = np.hypot(np.diff(np.concatenate([[DEPOT[0]], X])), np.diff(np.concatenate([[DEPOT[1]], Y])))
    out = {'h': h, 'hover': [], 'time': st['k'] * DT, 'timeouts': timeouts, 'wait': waits.mean(),
           'sat': st['nsat'] / st['k'], 'barrier': st['nbar'], 'barrier_infeasible': st['ninf'], 'barrier_run': st['bmax'] * DT, 'barrier_share': st['nbar'] / st['k'],
           'contact_cl': contact_cl, 'contact_plants': int(contact.sum()),
           'contact_own': int(own.sum()), 'contact_other': int((contact & ~own).sum()),
           'contact_own_climb': int((own & climb_leg).sum()), 'contact_own_descent': int((own & ~climb_leg).sum()),
           'contact_steps': n_contact, 'z_safe': z_safe, 'max_step': float(sp.max()),
           'climb': float(np.sum(np.abs(np.diff(Z)))), 'zmin': float(Z.min()), 'zmax': float(Z.max())}
    for (pi, a, b, ph) in hover:
        out['hover'].append({'plant': pi, 't': np.arange(a, b) * DT, 'x': X[a:b], 'y': Y[a:b], 'z': Z[a:b],
                             'ph': ph})
    if keep_log:
        out['traj'] = np.column_stack([X, Y, Z])
        out['contact_step'] = any_step
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
    jobs += [(s, 'Tight', SIGMA_H_LOW, seed, False) for s in STRATS for seed in range(N_SWEEP_SEEDS)]
    jobs += [(s, 'Tuned', sh, seed, False) for sh in SIGMA_H_SWEEP if sh != SIGMA_H
             for s in STRATS for seed in range(N_SWEEP_SEEDS)]
    jobs += [(s, g, SIGMA_H, seed, False, rc) for rc in R_CROWN_SWEEP if rc != R_CROWN
             for g in GAINS for s in STRATS for seed in range(N_SWEEP_SEEDS)]
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


_TH = np.linspace(0, 2 * np.pi, 25)


def _crowns(ax, h, rc=R_CROWN):
    for i in range(N_PLANTS):
        ax.plot(PLANTS[i, 0] + rc * np.cos(_TH), PLANTS[i, 1] + rc * np.sin(_TH), np.full_like(_TH, h[i]),
                color='darkgreen', lw=0.8)


def _collision_text(r):
    c = dose_eval(r)['ceff']
    n = r['contact_plants']
    return (f"crown contacts {n} -> collision, safe C_eff 0.000" if n > 0 else 'no crown contact, safe C_eff = C_eff'), c


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
        _crowns(ax, h)
        cs = r['contact_step']
        if cs.any():
            ax.scatter(tr[cs, 0], tr[cs, 1], tr[cs, 2], color='black', marker='x', s=12, label='inside a crown')
        ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)'); ax.set_zlabel('z (m)')
        ax.set_zlim(0, 3)
        ax.view_init(elev=25, azim=-60)
        txt, ce = _collision_text(r)
        ax.set_title(f'{LABELS[s]}, Tuned PID, seed {PLOT_SEED}: C_eff {ce:.3f}; {txt}\n'
                     f'dark-green rings = crowns (r = {R_CROWN} m), dotted = standoff during spraying', fontsize=9)
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
        txt, _ = _collision_text(r)
        ax.set_title(f"{LABELS[s]}: C_eff = {dv['ceff']:.3f}, mean drift-offset d = {100 * dv['drift'].mean():.1f} cm\n{txt}",
                     fontsize=10)
        ax.set_aspect('equal'); ax.set_xlim(-0.5, 20.5); ax.set_ylim(-0.5, 20.5)
        ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')
        ax.legend(fontsize=8)
    fig.suptitle(f'S063 3D Droplet impact points (every 5th spray sample), Tuned PID, seed {PLOT_SEED}; circles: σ_spray')
    _save(fig, out_dir, 'droplet_impacts.png')


def plot_sweep(res, out_dir):
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    for s in STRATS:
        m, sd, sm = [], [], []
        for sh in SIGMA_H_SWEEP:
            rs = res[(s, 'Tuned', sh)][:N_SWEEP_SEEDS]
            c = np.array([dose_eval(r)['ceff'] for r in rs])
            free = np.array([r['contact_plants'] == 0 for r in rs])
            m.append(c.mean()); sd.append(c.std()); sm.append((c * free).mean())
        axes[0].errorbar(SIGMA_H_SWEEP, m, yerr=sd, marker='o', capsize=3, color=COLORS[s], label=LABELS[s])
        axes[1].plot(SIGMA_H_SWEEP, sm, marker='o', color=COLORS[s], label=LABELS[s])
    axes[0].set_ylabel(f'C_eff 3D, dose only (mean ± std, {N_SWEEP_SEEDS} seeds)')
    axes[1].set_ylabel('safe C_eff 3D (collision missions scored 0)')
    for ax in axes:
        ax.set_xlabel('canopy height std σ_h (m)')
        ax.set_ylim(0, 1)
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
    fig.suptitle('S063 3D Coverage vs canopy-height variability (Tuned PID)', fontsize=11)
    _save(fig, out_dir, 'ceff_vs_sigma_h.png')


def save_animation(res, out_dir):
    r = res[('tf', 'Tuned', SIGMA_H)][PLOT_SEED]
    tr = r['traj']
    n = len(tr)
    nf = int(np.ceil((n - 1) / ANIM_STEP)) + 1
    idx = [min(i * ANIM_STEP, n - 1) for i in range(nf)]
    h = r['h']
    cs = r['contact_step']
    # cumulative number of distinct crown entries (rising edges of the contact flag) up to each step
    entries = np.cumsum(np.concatenate([[cs[0]], cs[1:] & ~cs[:-1]]).astype(int))
    fig = plt.figure(figsize=(6.4, 5.6))
    ax = fig.add_subplot(111, projection='3d')

    def update(f):
        k = idx[f]
        ax.cla()
        for i in range(N_PLANTS):
            ax.plot([PLANTS[i, 0]] * 2, [PLANTS[i, 1]] * 2, [0, h[i]], color='green', lw=3)
        _crowns(ax, h)
        ax.plot(tr[:k + 1:10, 0], tr[:k + 1:10, 1], tr[:k + 1:10, 2], color='red', lw=0.7)
        sel = cs[:k + 1]
        if sel.any():
            ax.scatter(tr[:k + 1][sel, 0], tr[:k + 1][sel, 1], tr[:k + 1][sel, 2], color='black', marker='x', s=6)
        ax.scatter(*tr[k], color='red', s=25)
        ax.set_xlim(0, 20); ax.set_ylim(0, 20); ax.set_zlim(0, 3)
        ax.view_init(elev=25, azim=-60)
        ax.set_title(f'S063 3D lidar terrain following, t = {(k + 1) * DT:.0f} s, z = {tr[k, 2]:.2f} m\n'
                     f'crown entries so far {entries[k]} (black x = inside a crown)', fontsize=9)
        return []

    anim = animation.FuncAnimation(fig, update, frames=nf, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    anim.save(path, writer='pillow', fps=8, dpi=65)
    plt.close(fig)
    from PIL import Image
    with Image.open(path) as gif:
        nfile = gif.n_frames
    print(f'Saved: {path} ({nf} frames rendered, {nfile} in the file, every {ANIM_STEP} steps = {ANIM_STEP * DT:.0f} s; '
          f'crown entries in this mission {entries[-1]}, plants touched {r["contact_plants"]}, lowest altitude {tr[:, 2].min():.3f} m)')


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
    print(f'  Floor barrier: plans with {A_BRAKE:.2f} m/s^2 braking (A_MAX {A_MAX:.1f} - worst downwash {DW_WORST:.2f} - 0.05); '
          f'highest speed that full braking can stop from z = {Z_MIN:.1f} m before z = 0: '
          f'{math.sqrt(2 * (A_MAX - F_DW) * Z_MIN):.2f} m/s')
    print('  Scoring rule: a mission with any crown contact is a collision; its safe C_eff is 0. '
          'C_eff (dose only) is reported beside it. Main rule: point drone, zero clearance.')
    print(f'  Broken-loop check: the barrier may override the vertical command in at most {100 * BARRIER_SHARE_MAX:.0f} % '
          f'of the steps of a mission (assert). For reference, braking the fastest descent possible inside the envelope '
          f'({V_ENVELOPE:.2f} m/s) at {A_BRAKE:.2f} m/s^2 takes {T_BARRIER_MAX:.2f} s')

    def stats(rs):
        c = np.array([dose_eval(r)['ceff'] for r in rs])
        free = np.array([r['contact_plants'] == 0 for r in rs])
        return c, c * free, int(free.sum())

    for g in GAINS:
        print(f'\n  --- {g} PID (horizontal {GAINS[g]}, vertical x{ALPHA_Z}), sigma_h = {SIGMA_H}, {N_SEEDS} seeds ---')
        ce, sa = {}, {}
        for s in STRATS:
            rs = res[(s, g, SIGMA_H)]
            ev = [dose_eval(r) for r in rs]
            c, sf, nfree = stats(rs)
            ce[s], sa[s] = c, sf
            a = lambda k: np.mean([e[k].mean() for e in ev])
            print(f"    {LABELS[s]:26s} C_eff {c.mean():.3f} +- {c.std():.3f} | collision-free missions {nfree}/{len(rs)}, "
                  f"safe C_eff {sf.mean():.3f}, mean C_eff of the collision-free missions "
                  f"{(c[sf > 0].mean() if np.any(sf > 0) else float('nan')):.4f} | hover RMS 3D {100 * a('rms3'):.1f} cm, "
                  f"horiz {100 * a('rmsh'):.1f} cm | standoff dev {100 * a('sdev'):.1f} cm | drift offset d "
                  f"{100 * a('drift'):.1f} cm | plants with dose > 0.9 {100 * np.mean([np.mean(e['dose'] > 0.9) for e in ev]):.1f} % | "
                  f"canopy contact: plants {np.mean([r['contact_plants'] for r in rs]):.2f}/mission "
                  f"(target plant {np.mean([r['contact_own'] for r in rs]):.2f} = on a climb leg "
                  f"{np.mean([r['contact_own_climb'] for r in rs]):.2f} + on a descent/level leg "
                  f"{np.mean([r['contact_own_descent'] for r in rs]):.2f}, other plants "
                  f"{np.mean([r['contact_other'] for r in rs]):.2f}), "
                  f"plants sprayed from inside the crown {np.mean([np.sum(e['dz'] <= 0) for e in ev]):.2f} | "
                  f"time {np.mean([r['time'] for r in rs]):.1f} s, time-outs {np.mean([r['timeouts'] for r in rs]):.1f}/20, "
                  f"saturated {100 * np.mean([r['sat'] for r in rs]):.2f} %, floor-barrier steps {np.mean([r['barrier'] for r in rs]):.1f}/mission, "
                  f"vertical travel {np.mean([r['climb'] for r in rs]):.1f} m, z range {min(r['zmin'] for r in rs):.3f}-"
                  f"{max(r['zmax'] for r in rs):.2f} m, max step {max(r['max_step'] for r in rs):.4f} m")
        print(f'    clearance sweep (drone radius r_d, vertical margin m_z): collision-free missions and safe C_eff')
        for s in STRATS:
            rs = res[(s, g, SIGMA_H)]
            c = np.array([dose_eval(r)['ceff'] for r in rs])
            parts = []
            for i, (rd, mz) in enumerate(CLEARANCES):
                free = np.array([r['contact_cl'][i] == 0 for r in rs])
                parts.append(f'r_d {rd:.1f} m / m_z {mz:.2f} m: {int(free.sum())}/{len(rs)}, {np.mean(c * free):.3f}')
            print(f'      {LABELS[s]:26s} ' + ' | '.join(parts))
        for a_, b_ in [('adaptive', 'fixed15'), ('adaptive', 'fixedsafe'), ('adaptive', 'tf'), ('tf', 'fixed15'),
                       ('fixedsafe', 'fixed15')]:
            w, ws = wins(ce[a_], ce[b_]), wins(sa[a_], sa[b_])
            print(f'    {LABELS[a_]} vs {LABELS[b_]}: C_eff higher / equal / lower {w[0]} / {w[1]} / {w[2]}, mean difference '
                  f'{ce[a_].mean() - ce[b_].mean():+.3f}; safe C_eff {ws[0]} / {ws[1]} / {ws[2]}, '
                  f'mean difference {sa[a_].mean() - sa[b_].mean():+.3f}')
    print(f'\n  --- sensitivity of mean C_eff (dose only) to the added drift parameters (Tuned, sigma_h = {SIGMA_H}, {N_SEEDS} seeds) ---')
    for cw in C_W_SWEEP:
        for wd in W_DW_SWEEP:
            vals = {s: np.mean([dose_eval(r, cw, wd)['ceff'] for r in res[(s, 'Tuned', SIGMA_H)]]) for s in STRATS}
            drift = {s: np.mean([dose_eval(r, cw, wd)['drift'].mean() for r in res[(s, 'Tuned', SIGMA_H)]]) for s in STRATS}
            order = sorted(STRATS, key=lambda s: -vals[s])
            print(f'    c_w {cw:.2f} 1/s (wind amplitude {A_WIND / cw:.1f} m/s), w_dw {wd:.0f} m/s: ' +
                  ', '.join(f'{s} {vals[s]:.3f} (d {100 * drift[s]:.1f} cm)' for s in STRATS) + f' | ranking {" > ".join(order)}')
    print(f'\n  --- crown-radius sweep (sigma_h = {SIGMA_H}, {N_SWEEP_SEEDS} seeds; C_eff / collision-free missions / safe C_eff / contacts per mission) ---')
    for rc in R_CROWN_SWEEP:
        for g in GAINS:
            out = {}
            for s in STRATS:
                key = (s, g, SIGMA_H) if rc == R_CROWN else (s, g, SIGMA_H, rc)
                rs = res[key][:N_SWEEP_SEEDS]
                c, sf, nfree = stats(rs)
                out[s] = (c, sf, nfree, np.mean([r['contact_plants'] for r in rs]))
            print(f'    R_crown {rc:.2f} m, {g}: ' + ', '.join(
                f'{s} {out[s][0].mean():.3f} / {out[s][2]}/{N_SWEEP_SEEDS} / {out[s][1].mean():.3f} / {out[s][3]:.2f}' for s in out) +
                f"; C_eff: tf > fixed15 in {wins(out['tf'][0], out['fixed15'][0])[0]}/{N_SWEEP_SEEDS}, "
                f"adaptive > tf in {wins(out['adaptive'][0], out['tf'][0])[0]}/{N_SWEEP_SEEDS}; safe C_eff: tf > fixed15 in "
                f"{wins(out['tf'][1], out['fixed15'][1])[0]}/{N_SWEEP_SEEDS}, adaptive > tf in "
                f"{wins(out['adaptive'][1], out['tf'][1])[0]}/{N_SWEEP_SEEDS}, adaptive vs fixedsafe higher/equal/lower "
                f"{wins(out['adaptive'][1], out['fixedsafe'][1])}")
    print(f'\n  --- vertical-damping probe: adaptive with k_d,z x {KDZ_PROBE} (sigma_h = {SIGMA_H}, {N_SEEDS} seeds) ---')
    for g in GAINS:
        kp, ki, kd = (ALPHA_Z * GAINS[g][k] for k in ('kp', 'ki', 'kd'))
        for sc, key in ((1.0, ('adaptive', g, SIGMA_H)), (KDZ_PROBE, ('adaptive', g, SIGMA_H, R_CROWN, KDZ_PROBE))):
            poles = np.roots([1, sc * kd, kp, ki])
            cp = poles[np.argmax(np.abs(poles.imag))]
            rs = res[key]
            c, sf, nfree = stats(rs)
            print(f'    {g}, k_d,z x {sc:g} (damping {-cp.real / abs(cp):.3f}): target-plant contacts '
                  f"{np.mean([r['contact_own'] for r in rs]):.2f}/mission (climb leg {np.mean([r['contact_own_climb'] for r in rs]):.2f}, "
                  f"descent/level leg {np.mean([r['contact_own_descent'] for r in rs]):.2f}), other plants "
                  f"{np.mean([r['contact_other'] for r in rs]):.2f}, z min {min(r['zmin'] for r in rs):.2f} m, "
                  f"C_eff {c.mean():.3f}, collision-free {nfree}/{len(rs)}, safe C_eff {sf.mean():.3f}")
    print(f'\n  --- probe: dose without the standoff factor (drift and hover error only), sigma_h = {SIGMA_H}, {N_SEEDS} seeds ---')
    for g in GAINS:
        vals = {s: np.array([dose_eval(r, alt_factor=False)['ceff'] for r in res[(s, g, SIGMA_H)]]) for s in STRATS}
        print(f'    {g}: ' + ', '.join(f'{s} {vals[s].mean():.3f} +- {vals[s].std():.3f}' for s in STRATS) +
              f"; adaptive vs fixed15 higher/equal/lower {wins(vals['adaptive'], vals['fixed15'])}, "
              f"adaptive vs fixedsafe {wins(vals['adaptive'], vals['fixedsafe'])}")
    print(f'\n  --- C_eff vs sigma_h (Tuned, {N_SWEEP_SEEDS} seeds; C_eff, collision-free missions, safe C_eff) ---')
    for sh in SIGMA_H_SWEEP:
        parts = []
        for s in STRATS:
            rs = res[(s, 'Tuned', sh)][:N_SWEEP_SEEDS]
            c, sf, nfree = stats(rs)
            parts.append(f"{s} {c.mean():.3f} +- {c.std():.3f}, {nfree}/{N_SWEEP_SEEDS}, {sf.mean():.3f}")
        print(f'    sigma_h {sh:.2f}: ' + ' | '.join(parts))
    print(f'\n  --- low height spread sigma_h = {SIGMA_H_LOW}, {N_SWEEP_SEEDS} seeds: per seed and per clearance '
          f'(collision-free missions, safe C_eff; per-seed safe C_eff higher/equal/lower) ---')
    for g in GAINS:
        rsd = {s: res[(s, g, SIGMA_H_LOW)][:N_SWEEP_SEEDS] for s in STRATS}
        cd = {s: np.array([dose_eval(r)['ceff'] for r in rsd[s]]) for s in STRATS}
        for i, (rd, mz) in enumerate(CLEARANCES):
            sf = {s: cd[s] * np.array([r['contact_cl'][i] == 0 for r in rsd[s]]) for s in STRATS}
            print(f'    {g}, r_d {rd:.1f} m / m_z {mz:.2f} m: ' + ', '.join(
                f"{s} {int(np.sum([r['contact_cl'][i] == 0 for r in rsd[s]]))}/{N_SWEEP_SEEDS} {sf[s].mean():.3f}"
                for s in STRATS) +
                f"; adaptive vs fixed15 {wins(sf['adaptive'], sf['fixed15'])}, adaptive vs fixedsafe "
                f"{wins(sf['adaptive'], sf['fixedsafe'])}, fixed15 vs fixedsafe {wins(sf['fixed15'], sf['fixedsafe'])}")
    print('\n  --- floor barrier over EVERY job group (asserts: z >= Z_MIN, z > 0, thrust acceleration <= A_MAX) ---')
    nb = 0
    for key in sorted(res, key=str):
        rs = res[key]
        if max(r['barrier'] for r in rs) == 0:
            continue
        nb += 1
        print(f"    {str(key):42s} barrier active {np.mean([r['barrier'] for r in rs]):.1f} steps/mission in "
              f"{sum(r['barrier'] > 0 for r in rs)}/{len(rs)} missions, infeasible {sum(r['barrier_infeasible'] for r in rs)}, "
              f"lowest altitude {min(r['zmin'] for r in rs):.4f} m, longest continuous stretch "
              f"{max(r['barrier_run'] for r in rs):.2f} s, largest share of steps {100 * max(r['barrier_share'] for r in rs):.2f} %")
    print('\n  --- seed-0 missions shown in the figures (Tuned, sigma_h 0.3) ---')
    for s in STRATS:
        r = res[(s, 'Tuned', SIGMA_H)][PLOT_SEED]
        txt, ce = _collision_text(r)
        print(f'    {LABELS[s]:26s} C_eff {ce:.3f}, mean drift offset {100 * dose_eval(r)["drift"].mean():.1f} cm; {txt}')
    print(f'    job groups with any barrier activity: {nb} of {len(res)}; longest stretch overall '
          f"{max(r['barrier_run'] for rs in res.values() for r in rs):.2f} s; largest share of steps "
          f"{100 * max(r['barrier_share'] for rs in res.values() for r in rs):.2f} % vs the {100 * BARRIER_SHARE_MAX:.0f} % assert")

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
