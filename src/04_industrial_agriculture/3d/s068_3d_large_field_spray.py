"""
S068 3D Upgrade — Large-Scale Field Spraying over a DEM
=======================================================
The 2D S068 base flies four drones over a flat 200 x 100 m field (boustrophedon runs,
1.9 m run spacing, 3 m/s, 600 s batteries, 120 s swaps at a station at (0, 50)), with an
event-based flight model and an exact battery-return trigger; its best partition
("B", contiguous run blocks balanced by a dynamic programme) finishes in 1144.1 s.

The 3D variant lays the card's DEM under the same mission,
    z_t(x, y) = 1.5 sin(2 pi x / 80) cos(2 pi y / 120),
and makes altitude a controlled state:
  * the spray cone (half-angle 27.5 deg) gives a swath 2 (z - z_t) tan(27.5 deg), so the
    nozzle height above ground decides whether neighbouring runs still touch;
  * drones either hold a fixed altitude (the 2D plan) or follow the terrain at 2.0 m AGL
    with the card's second-order altitude loop (optionally with velocity feed-forward);
  * odd drones may fly 0.4 m higher (the card's altitude stagger);
  * flight time is the 3D path length / 3 m/s and the battery trigger uses the exact
    terrain-following path home (the card's straight 3D distance is reported too);
  * the card's rotor-wash interference index is evaluated for adjacent drones, together
    with a corrected index that uses the vertical separation between the drones.
The flight plan, partitions and battery logic are the 2D base's; everything here is
deterministic (no random inputs), so there is no Monte Carlo.

Usage:
    conda activate drones
    python src/04_industrial_agriculture/3d/s068_3d_large_field_spray.py
"""

import os
import sys
import math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.colors import ListedColormap
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, '..', '..', '..'))
sys.path.insert(0, os.path.join(SCRIPT_DIR, '..'))
import s068_large_field_spray as base               # noqa: E402  (2D base: run grid, DP partition)

OUTPUT_DIR = os.path.join(PROJECT_ROOT, 'outputs', '04_industrial_agriculture', '3d',
                          's068_3d_large_field_spray')

# ── Parameters ────────────────────────────────────────────────────────────────
FIELD_W, FIELD_D = base.FIELD_W, base.FIELD_D
N_DRONES = base.N_DRONES
RUN_SPACING = base.RUN_SPACING            # 1.9 m
V = base.SPEED                            # 3 m/s (3D airspeed along the path)
T_BAT, T_CHARGE = base.T_BAT, base.T_CHARGE
STATION = base.STATION                    # (0, 50)
CELL = base.CELL
A_DEM = 1.5                               # card
LAM_X, LAM_Y = 80.0, 120.0                # card
H_AGL = 2.0                               # card
DH = 0.4                                  # card stagger
TAN_C = math.tan(math.radians(27.5))      # card cone half-angle -> swath 2.083 m at 2 m
K_Z, K_ZD = 4.0, 2.0                      # card
EPS_Z = 0.1                               # card tolerance
V0, SIG_W, SIG_Z, PHI_MAX = 8.0, 0.6, 1.5, 0.15   # card wash model
AGL_BAND = (1.5, 5.0)                     # card altitude range, read as height above ground
DT = 0.2                                  # card timestep (sampling of the altitude loop)
SUB = 10                                  # added: integration sub-steps per DT
DS = 1.0                                  # added: path-length sampling (m)
PAD_DY = 3.0                              # added: one landing pad per drone, 3 m apart
Z_FIXED = 2.0                             # 2D plan: constant altitude above the datum
Z_FIXED_HI = 3.5                          # added: 2D plan raised so that AGL >= 2 m everywhere
ANIM_DT = 20.0
COLORS = ['tab:red', 'tab:orange', 'tab:blue', 'tab:purple']

MX, MY = int(FIELD_W / CELL), int(FIELD_D / CELL)
XC = (np.arange(MX) + 0.5) * CELL
YC = (np.arange(MY) + 0.5) * CELL

CONFIGS = {  # key: (label, DEM amplitude, altitude mode, stagger, feed-forward)
    'flat': ('Flat field, z = 2 m (2D base)', 0.0, 'fixed', 0.0, False),
    'fixed': ('DEM, fixed z = 2 m (2D plan)', A_DEM, 'fixed', 0.0, False),
    'fixed35': ('DEM, fixed z = 3.5 m (2D plan, AGL >= 2 m)', A_DEM, 'fixed35', 0.0, False),
    'tf': ('DEM, terrain following', A_DEM, 'tf', 0.0, False),
    'tf_stag': ('DEM, TF + 0.4 m stagger (card)', A_DEM, 'tf', DH, False),
    'tf_stag_ff': ('DEM, TF + stagger + feed-forward', A_DEM, 'tf', DH, True),
}


# ── Terrain ───────────────────────────────────────────────────────────────────

def terrain(x, y, A=A_DEM):
    return A * np.sin(2 * np.pi * np.asarray(x) / LAM_X) * np.cos(2 * np.pi * np.asarray(y) / LAM_Y)


def pads(n, dy=PAD_DY):
    if n == 1:
        return [STATION.copy()]
    return [STATION + np.array([0.0, dy * (i - (n - 1) / 2)]) for i in range(n)]


# ── Event-based flight (2D base logic, 3D path times) ─────────────────────────

class Flight:
    def __init__(self, A, mode, offset):
        self.A, self.mode, self.offset = A, mode, offset

    def z_cmd(self, x, y):
        if self.mode == 'fixed':
            return np.full(np.shape(x), Z_FIXED)
        if self.mode == 'fixed35':
            return np.full(np.shape(x), Z_FIXED_HI)
        return terrain(x, y, self.A) + H_AGL + self.offset

    def path_time(self, p0, p1):
        L = float(np.hypot(*(p1 - p0)))
        if L < 1e-12:
            return 0.0
        if self.mode != 'tf' or self.A == 0.0:
            return L / V
        n = int(math.ceil(L / DS)) + 1
        s = np.linspace(0.0, 1.0, n)
        x = p0[0] + s * (p1[0] - p0[0])
        y = p0[1] + s * (p1[1] - p0[1])
        z = self.z_cmd(x, y)
        return float(np.sum(np.sqrt((L / (n - 1)) ** 2 + np.diff(z) ** 2))) / V

    def straight3d_time(self, p0, p1):
        z0, z1 = float(self.z_cmd(p0[0], p0[1])), float(self.z_cmd(p1[0], p1[1]))
        return math.sqrt(float(np.sum((p1 - p0) ** 2)) + (z1 - z0) ** 2) / V


def simulate_drone(xs, pad, fl):
    pos, t, used = pad.copy(), 0.0, 0.0
    segs, trips = [], 0
    acc = {'spray': 0.0, 'turn': 0.0, 'ferry': 0.0, 'charge': 0.0}
    shortfall = []

    def home(p):
        return fl.path_time(p, pad)

    def fly(p_to, kind):
        nonlocal pos, t, used
        dur = fl.path_time(pos, p_to)
        if dur <= 0:
            return
        segs.append((pos.copy(), p_to.copy(), t, t + dur, kind))
        t += dur
        used += dur
        acc[kind] += dur
        pos = p_to.copy()
        assert T_BAT - used - home(pos) >= -1e-6, 'battery would not reach the pad'
        assert used <= T_BAT + 1e-6, 'flying on an empty battery'

    def swap():
        nonlocal t, used, trips
        fly(pad, 'ferry')
        segs.append((pad.copy(), pad.copy(), t, t + T_CHARGE, 'charge'))
        t += T_CHARGE
        acc['charge'] += T_CHARGE
        used = 0.0
        trips += 1

    def leg(p_to, kind):
        while True:
            need = fl.path_time(pos, p_to) + home(p_to)
            if used + need <= T_BAT + 1e-9:
                fly(p_to, kind)
                return
            p0 = pos.copy()
            L = float(np.hypot(*(p_to - p0)))
            u = (p_to - p0) / L
            lo, hi = 0.0, L
            if used + home(p0) >= T_BAT - 1e-9:
                hi = 0.0
            for _ in range(50):
                mid = 0.5 * (lo + hi)
                if used + fl.path_time(p0, p0 + mid * u) + home(p0 + mid * u) <= T_BAT:
                    lo = mid
                else:
                    hi = mid
            stop = p0 + lo * u
            shortfall.append(home(stop) - fl.straight3d_time(stop, pad))
            fly(stop, kind)
            swap()
            assert 2 * home(stop) < T_BAT, 'resume point unreachable on a full battery'
            if kind != 'ferry':
                fly(stop, 'ferry')

    for r, x in enumerate(xs):
        ya, yb = (0.0, FIELD_D) if r % 2 == 0 else (FIELD_D, 0.0)
        leg(np.array([x, ya]), 'ferry' if r == 0 else 'turn')
        leg(np.array([x, yb]), 'spray')
    t_cover = t
    acc_cover = dict(acc)
    fly(pad, 'ferry')
    return dict(segs=segs, trips=trips, t_cover=t_cover, t_home=t, acc=acc, acc_cover=acc_cover,
                shortfall=shortfall, pad=pad)


# ── Altitude loop ─────────────────────────────────────────────────────────────

def timeline(d, fl, t_end):
    """Horizontal position, segment kind and altitude command on the DT grid."""
    t = np.arange(0.0, t_end + 1e-9, DT)
    t0s = np.array([s[2] for s in d['segs']])
    idx = np.clip(np.searchsorted(t0s, t, side='right') - 1, 0, len(d['segs']) - 1)
    px, py = np.empty(t.size), np.empty(t.size)
    kind = np.empty(t.size, dtype=object)
    for k, (p0, p1, a, b, kd) in enumerate(d['segs']):
        m = idx == k
        f = np.clip((t[m] - a) / (b - a), 0, 1) if b > a else np.zeros(m.sum())
        px[m] = p0[0] + f * (p1[0] - p0[0])
        py[m] = p0[1] + f * (p1[1] - p0[1])
        kind[m] = kd
    done = t > d['t_home']
    px[done], py[done] = d['pad'][0], d['pad'][1]
    kind[done] = 'done'
    return t, px, py, kind, fl.z_cmd(px, py)


def altitude_loop(zc, ff):
    """Card loop z'' = K_z (z_c - z) - K_zd z'  (ff: K_zd (z_c' - z')), semi-implicit Euler."""
    zc_dot = np.gradient(zc, DT)
    z = np.empty_like(zc)
    zz, zd = float(zc[0]), (float(zc_dot[0]) if ff else 0.0)
    z[0] = zz
    h = DT / SUB
    for k in range(1, zc.size):
        c0, c1 = zc[k - 1], zc[k]
        d0, d1 = (zc_dot[k - 1], zc_dot[k]) if ff else (0.0, 0.0)
        for s in range(SUB):
            f = (s + 1) / SUB
            acc = K_Z * (c0 + f * (c1 - c0) - zz) + K_ZD * (d0 + f * (d1 - d0) - zd)
            zd += acc * h
            zz += zd * h
        z[k] = zz
    return z


# ── Coverage ─────────────────────────────────────────────────────────────────

def coverage(drones, tls, A):
    t_first = np.full((MX, MY), np.inf)
    count = np.zeros((MX, MY), dtype=int)
    for d, tl in zip(drones, tls):
        t, px, py, kind, zc, z = tl
        for p0, p1, t0, t1, kd in d['segs']:
            if kd != 'spray':
                continue
            x = p0[0]
            ylo, yhi = min(p0[1], p1[1]), max(p0[1], p1[1])
            yy = np.linspace(ylo, yhi, max(2, int((yhi - ylo) / 0.25) + 1))
            tt = t0 + np.abs(yy - p0[1]) / abs(p1[1] - p0[1]) * (t1 - t0) if yhi > ylo else \
                np.full(yy.size, t0)
            agl = np.interp(tt, t, z) - terrain(x, yy, A)
            assert agl.min() > 0.3, 'nozzle within 0.3 m of the ground'
            r = agl * TAN_C
            rmax = r.max()
            ix = np.where(np.abs(XC - x) <= rmax + 1e-9)[0]
            iy = np.where((YC >= ylo - rmax) & (YC <= yhi + rmax))[0]
            yproj = np.clip(YC[iy], ylo, yhi)
            r_at = np.interp(yproj, yy, r)
            dist = np.hypot(XC[ix][:, None] - x, YC[iy][None, :] - yproj[None, :])
            hit = dist <= r_at[None, :] + 1e-9
            t_hit = np.interp(yproj, yy, tt)[None, :] * np.ones_like(dist)
            sub = t_first[np.ix_(ix, iy)]
            t_first[np.ix_(ix, iy)] = np.where(hit, np.minimum(sub, t_hit), sub)
            count[np.ix_(ix, iy)] += hit
    return t_first, count


# ── Strategy run ──────────────────────────────────────────────────────────────

def run(blocks, cfg, pad_dy=PAD_DY, offset=None, A=None):
    label, A_c, mode, stag, ff = CONFIGS[cfg] if isinstance(cfg, str) else cfg
    A = A_c if A is None else A
    stag = stag if offset is None else offset
    pp = pads(len(blocks), pad_dy)
    fls = [Flight(A, mode, stag * (i % 2)) for i in range(len(blocks))]
    drones = [simulate_drone(xs, pp[i], fls[i]) for i, xs in enumerate(blocks)]
    t_end = max(d['t_home'] for d in drones)
    tls = []
    for d, fl in zip(drones, fls):
        t, px, py, kind, zc = timeline(d, fl, t_end)
        z = zc.copy() if mode != 'tf' else altitude_loop(zc, ff)
        tls.append((t, px, py, kind, zc, z))
    t_first, count = coverage(drones, tls, A)
    res = dict(drones=drones, tls=tls, blocks=blocks, A=A, mode=mode, t_first=t_first,
               count=count, coverage=float(np.mean(np.isfinite(t_first))),
               t_mission=float(np.max(t_first)) if np.all(np.isfinite(t_first)) else np.nan,
               t_last_home=t_end, trips=[d['trips'] for d in drones],
               overspray=float(np.mean(count >= 2)))
    res.update(altitude_metrics(tls, A))
    assert res['air_agl_min'] > 0.3, 'drone within 0.3 m of the ground while airborne'
    res.update(wash_metrics(tls, drones, A))
    res['shortfall'] = max([s for d in drones for s in d['shortfall']] or [0.0])
    return res


def altitude_metrics(tls, A):
    err, agl, sw_air, err_late = [], [], [], []
    for t, px, py, kind, zc, z in tls:
        m = kind == 'spray'
        err.append(np.abs(z - zc)[m])
        since = np.zeros(t.size)                       # time since the current segment kind began
        for k in range(1, t.size):
            since[k] = since[k - 1] + DT if kind[k] == kind[k - 1] else 0.0
        err_late.append(np.abs(z - zc)[m & (since > 5.0)])
        a = z - terrain(px, py, A)
        agl.append(a[m])
        sw_air.append(a[(kind != 'done') & (kind != 'charge')])
    err, agl, air = np.concatenate(err), np.concatenate(agl), np.concatenate(sw_air)
    return dict(eps_max=float(err.max()), eps_frac=float(np.mean(err > EPS_Z)),
                eps_max_late=float(np.concatenate(err_late).max()),
                eps_mean=float(err.mean()), agl_min=float(agl.min()), agl_max=float(agl.max()),
                swath_min=float(2 * agl.min() * TAN_C), swath_max=float(2 * agl.max() * TAN_C),
                band_frac=float(np.mean((agl < AGL_BAND[0]) | (agl > AGL_BAND[1]))),
                air_agl_min=float(air.min()))


def wash_metrics(tls, drones, A):
    n = len(tls)
    out = dict(phi_card=0.0, phi_card_spray=0.0, phi_corr=0.0, phi_corr_spray=0.0,
               phi_card_frac=0.0, min_sep=np.inf, min_hsep_spray=np.inf, phi_series={})
    if n < 2:
        return out
    P = [np.stack([tl[1], tl[2], tl[5]]) for tl in tls]
    air = [np.isin(tl[3], ('spray', 'turn', 'ferry')) for tl in tls]
    spr = [tl[3] == 'spray' for tl in tls]
    above = 0
    tot = 0
    for i in range(n - 1):
        j = i + 1
        rho2 = (P[i][0] - P[j][0]) ** 2 + (P[i][1] - P[j][1]) ** 2
        both = air[i] & air[j]
        hi = P[i][2] - terrain(P[j][0], P[j][1], A)          # card: height of i over the ground at j
        hj = P[j][2] - terrain(P[i][0], P[i][1], A)
        phi_c = np.where(both, np.exp(-rho2 / (2 * SIG_W ** 2)) *
                         (np.exp(-hi ** 2 / (2 * SIG_Z ** 2)) + np.exp(-hj ** 2 / (2 * SIG_Z ** 2))), 0.0)
        dz = P[i][2] - P[j][2]
        phi_r = np.where(both, 2 * np.exp(-rho2 / (2 * SIG_W ** 2)) *
                         np.exp(-dz ** 2 / (2 * SIG_Z ** 2)), 0.0)
        sp = both & (spr[i] | spr[j])
        out['phi_card'] = max(out['phi_card'], float(phi_c.max()))
        out['phi_corr'] = max(out['phi_corr'], float(phi_r.max()))
        if sp.any():
            out['phi_card_spray'] = max(out['phi_card_spray'], float(phi_c[sp].max()))
            out['phi_corr_spray'] = max(out['phi_corr_spray'], float(phi_r[sp].max()))
            out['min_hsep_spray'] = min(out['min_hsep_spray'], float(np.sqrt(rho2[sp].min())))
        above += int(np.sum(phi_c > PHI_MAX))
        tot += int(np.sum(both))
        out['phi_series'][(i, j)] = (phi_c, phi_r)
    t = tls[0][0]
    out['close_time'] = 0.0
    for i in range(n):
        for j in range(i + 1, n):
            both = air[i] & air[j]
            if both.any():
                d3 = np.where(both, np.sqrt(((P[i] - P[j]) ** 2).sum(axis=0)), np.inf)
                hs = np.where(both, np.hypot(P[i][0] - P[j][0], P[i][1] - P[j][1]), np.inf)
                out['close_time'] += float(np.sum(hs < 2.0)) * DT
                k = int(np.argmin(d3))
                if d3[k] < out['min_sep']:
                    out['min_sep'] = float(d3[k])
                    out['min_sep_event'] = (i, j, float(t[k]), float(P[i][0][k]), float(P[i][1][k]),
                                            tls[i][3][k], tls[j][3][k], float(abs(P[i][2][k] - P[j][2][k])))
    out['phi_card_frac'] = above / max(tot, 1)
    return out


def run_simulation():
    xs = base.global_runs()
    n = len(xs)
    edges = [int(round(n * k / N_DRONES)) for k in range(N_DRONES + 1)]
    equal = [xs[edges[k]:edges[k + 1]] for k in range(N_DRONES)]
    bal, cuts, _ = base.balanced_blocks(xs, N_DRONES)        # 2D-optimised partition
    part = {'E': equal, 'B': bal}
    out = {'xs': xs, 'cuts': cuts, 'main': {}, 'sweep': {}}
    # reproduction of the 2D base: flat, shared station point, swath radius exactly 1.0 m
    global TAN_C
    tan_saved, TAN_C = TAN_C, 1.0 / H_AGL
    out['repro'] = {p: run(part[p], 'flat', pad_dy=0.0) for p in ('E', 'B')}
    TAN_C = tan_saved
    for cfg in CONFIGS:
        for p in ('E', 'B'):
            out['main'][(cfg, p)] = run(part[p], cfg)
    sw = out['sweep']
    sw['stagger'] = {dh: run(bal, 'tf', offset=dh) for dh in (0.0, 0.4, 1.0, 2.0)}
    sw['amp'] = {a: {c: run(bal, c, A=a) for c in ('fixed', 'tf', 'tf_stag_ff')}
                 for a in (0.5, 1.0, 1.5)}
    sw['pad'] = {pd: run(bal, 'tf_stag', pad_dy=pd) for pd in (0.0, 1.0, 3.0, 6.0)}
    return out


# ── Plotting ──────────────────────────────────────────────────────────────────

def plot_trajectory_3d(out, out_dir):
    r = out['main'][('tf_stag', 'B')]
    fig = plt.figure(figsize=(15, 7.5))
    ax = fig.add_subplot(111, projection='3d')
    gx, gy = np.meshgrid(np.linspace(0, FIELD_W, 81), np.linspace(0, FIELD_D, 41))
    ax.plot_surface(gx, gy, terrain(gx, gy), color='grey', alpha=0.25, linewidth=0)
    for i, (t, px, py, kind, zc, z) in enumerate(r['tls']):
        m = kind != 'done'
        ax.plot(px[m][::3], py[m][::3], z[m][::3], color=COLORS[i], lw=0.6,
                label=f"drone {i} ({'+0.4 m' if i % 2 else 'nominal'})")
    for p in pads(N_DRONES):
        ax.scatter(p[0], p[1], terrain(p[0], p[1]), marker='*', s=150, color='gold', edgecolor='k')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_zlabel('z (m)')
    ax.set_box_aspect((4, 2, 0.8))
    ax.view_init(elev=28, azim=-65)
    ax.set_title('S068 3D — terrain-following paths over the DEM, balanced partition, 0.4 m stagger '
                 '(gold = pads)')
    ax.legend(fontsize=8, loc='upper left')
    path = os.path.join(out_dir, 'trajectory_3d.png')
    plt.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_altitude(out, out_dir):
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    rc, rf = out['main'][('tf_stag', 'B')], out['main'][('tf_stag_ff', 'B')]
    for i, (t, px, py, kind, zc, z) in enumerate(rc['tls']):
        m = kind != 'done'
        axes[0].plot(t[m], z[m], color=COLORS[i], lw=0.8, label=f'drone {i} z')
        axes[0].plot(t[m], zc[m], color='k', lw=0.4, ls=':')
        e = np.where(kind == 'spray', z - zc, np.nan)
        axes[1].plot(t, e, color=COLORS[i], lw=0.6)
        ef = np.where(rf['tls'][i][3] == 'spray', rf['tls'][i][5] - rf['tls'][i][4], np.nan)
        axes[2].plot(rf['tls'][i][0], ef, color=COLORS[i], lw=0.6)
    axes[0].set_ylabel('z (m)')
    axes[0].set_title('Altitude of the four drones (dotted = command z_t + 2 m (+0.4 m odd drones))')
    axes[0].legend(fontsize=8, ncol=4)
    for ax, ttl in ((axes[1], f"card loop: max |ε_z| = {rc['eps_max']:.3f} m while spraying"),
                    (axes[2], f"with velocity feed-forward: max |ε_z| = {rf['eps_max']:.3f} m")):
        ax.axhspan(-EPS_Z, EPS_Z, color='green', alpha=0.1)
        ax.axhline(EPS_Z, color='green', lw=0.8, ls='--')
        ax.axhline(-EPS_Z, color='green', lw=0.8, ls='--')
        ax.set_ylabel('ε_z = z − z_cmd (m)')
        ax.set_title(ttl + ' (green band = ±0.1 m tolerance; spraying only)')
        ax.set_ylim(-0.25, 0.25)
    axes[2].set_xlabel('time (s)')
    for a in axes:
        a.grid(alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'altitude.png')
    plt.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_wash(out, out_dir):
    fig, axes = plt.subplots(2, 1, figsize=(14, 7.5), sharex=True)
    for ax, key, ttl in ((axes[0], 'tf', 'no stagger'), (axes[1], 'tf_stag', '0.4 m stagger')):
        r = out['main'][(key, 'B')]
        t = r['tls'][0][0]
        for (i, j), (pc, pr) in r['phi_series'].items():
            ax.plot(t, pc, lw=0.8, label=f'Φ card, pair ({i},{j})')
            ax.plot(t, pr, lw=0.8, ls='--', label=f'Φ vertical-separation, pair ({i},{j})')
        ax.axhline(PHI_MAX, color='k', ls='--', lw=1, label='Φ_max = 0.15')
        ax.set_ylim(0, 2.05)
        ax.set_ylabel('Φ')
        ax.set_title(f"{ttl}: max Φ card {r['phi_card']:.3f}, while spraying {r['phi_card_spray']:.2e}; "
                     f"min 3D separation {r['min_sep']:.2f} m", fontsize=10)
        ax.grid(alpha=0.3)
    axes[0].legend(fontsize=7, ncol=3)
    axes[1].set_xlabel('time (s)')
    fig.suptitle('S068 3D — wash interference index for adjacent drones (balanced partition)')
    plt.tight_layout()
    path = os.path.join(out_dir, 'wash_interference.png')
    plt.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_coverage(out, out_dir):
    cmap = ListedColormap(['#ffffff', '#a1d99b', '#31a354', '#006d2c'])
    fig, axes = plt.subplots(2, 2, figsize=(17, 9.5))
    for ax, key in zip(axes.flat, ('flat', 'fixed', 'tf', 'tf_stag')):
        r = out['main'][(key, 'B')]
        ax.imshow(np.minimum(r['count'], 3).T, origin='lower', extent=[0, FIELD_W, 0, FIELD_D],
                  cmap=cmap, vmin=-0.5, vmax=3.5, aspect='equal', interpolation='nearest')
        if r['A'] > 0:
            gx, gy = np.meshgrid(XC, YC)
            c = ax.contour(gx, gy, terrain(gx, gy, r['A']), levels=[-1, 0, 1], colors='k',
                           linewidths=0.5)
            ax.clabel(c, fontsize=6, fmt='%.0f m')
        for b in r['blocks'][1:]:
            ax.axvline(b[0] - RUN_SPACING / 2, color='k', ls='--', lw=1)
        ax.scatter(*STATION, marker='*', s=200, color='gold', edgecolor='k', zorder=5)
        ax.set_title(f"{CONFIGS[key][0]}: coverage {100 * r['coverage']:.2f} %, sprayed ≥ 2× "
                     f"{100 * r['overspray']:.1f} %, swath {r['swath_min']:.2f}–{r['swath_max']:.2f} m",
                     fontsize=10)
        ax.set_xlabel('x (m)')
        ax.set_ylabel('y (m)')
    fig.suptitle('S068 3D — spray count per 0.5 m cell (white = missed, light/mid/dark green = 1/2/3+ '
                 'passes); contours = terrain height')
    plt.tight_layout()
    path = os.path.join(out_dir, 'coverage_maps.png')
    plt.savefig(path, dpi=120, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_progress_breakdown(out, out_dir):
    fig, axes = plt.subplots(1, 2, figsize=(16, 5.5))
    ax = axes[0]
    for key, c, ls in (('flat', 'k', '--'), ('fixed', '#D62728', '-'), ('tf', '#1F77B4', '-'),
                       ('tf_stag', '#2CA02C', ':')):
        r = out['main'][(key, 'B')]
        tf = np.sort(r['t_first'].ravel())
        tg = np.arange(0, 1300, DT)
        ax.plot(tg, 100 * np.searchsorted(tf, tg, side='right') / tf.size, color=c, ls=ls, lw=1.4,
                label=f"{CONFIGS[key][0]}: {100 * r['coverage']:.2f} %"
                      + (f", {r['t_mission']:.0f} s" if np.isfinite(r['t_mission']) else ''))
    ax.set_xlabel('time (s)')
    ax.set_ylabel('coverage C(t) (%)')
    ax.set_title('Coverage progress, balanced partition')
    ax.legend(fontsize=8, loc='lower right')
    ax.grid(alpha=0.3)
    ax = axes[1]
    kinds = [('spray', 'Spraying', '#31a354'), ('turn', 'Turns', '#fdae6b'),
             ('ferry', 'Ferry', '#6baed6'), ('charge', 'Battery swaps', '#bdbdbd')]
    groups = [('flat', 'B'), ('tf_stag', 'B')]
    xpos = 0
    ticks, labs = [], []
    for g, (key, p) in enumerate(groups):
        r = out['main'][(key, p)]
        for i, d in enumerate(r['drones']):
            bottom = 0.0
            for k, lab, col in kinds:
                v = d['acc_cover'][k]
                ax.bar(xpos, v, bottom=bottom, color=col, edgecolor='k', lw=0.4,
                       label=lab if (g == 0 and i == 0) else None)
                bottom += v
            ticks.append(xpos)
            labs.append(f"{'2D' if key == 'flat' else '3D'} D{i}")
            xpos += 1
        ax.hlines(r['t_mission'], xpos - 4.4, xpos - 0.6, color='crimson', ls='--', lw=1.2)
        xpos += 0.8
    ax.set_xticks(ticks)
    ax.set_xticklabels(labs, fontsize=8)
    ax.set_ylabel('time until the last spray (s)')
    ax.set_title('Per-drone time breakdown, flat 2D vs 3D TF + stagger (red dashed = T_mission)')
    ax.legend(fontsize=8, loc='lower right')
    plt.tight_layout()
    path = os.path.join(out_dir, 'progress_breakdown.png')
    plt.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def save_animation(out, out_dir):
    r = out['main'][('tf_stag', 'B')]
    t = r['tls'][0][0]
    step = int(round(ANIM_DT / DT))
    n = int(np.ceil(r['t_mission'] / DT)) + 1
    n = min(n, t.size)
    n_frames = int(np.ceil((n - 1) / step)) + 1
    fig = plt.figure(figsize=(10, 6))
    ax = fig.add_subplot(111, projection='3d')
    gx, gy = np.meshgrid(np.linspace(0, FIELD_W, 41), np.linspace(0, FIELD_D, 21))
    gz = terrain(gx, gy)

    def update(fi):
        k = min(fi * step, n - 1)
        ax.cla()
        ax.plot_wireframe(gx, gy, gz, color='grey', lw=0.3, alpha=0.5)
        for i, (tt, px, py, kind, zc, z) in enumerate(r['tls']):
            k0 = max(0, k - int(60 / DT))
            ax.plot(px[k0:k + 1], py[k0:k + 1], z[k0:k + 1], color=COLORS[i], lw=1.2)
            ax.scatter([px[k]], [py[k]], [z[k]], color=COLORS[i], s=30, edgecolor='k',
                       alpha=0.4 if kind[k] == 'charge' else 1.0)
        ax.set_xlim(0, FIELD_W)
        ax.set_ylim(0, FIELD_D)
        ax.set_zlim(-2, 5)
        ax.set_box_aspect((4, 2, 0.8))
        ax.view_init(elev=30, azim=-60)
        cov = np.mean(r['t_first'] <= t[k])
        ax.set_title(f'S068 3D — t = {t[k]:.0f} s, coverage {100 * cov:.1f} % '
                     f'(60 s tails; faded = battery swap)', fontsize=10)
        return []

    anim = animation.FuncAnimation(fig, update, frames=n_frames, interval=80, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    anim.save(path, writer='pillow', fps=10, dpi=70)
    plt.close(fig)
    print(f'Saved: {path}  ({n_frames} frames, every {ANIM_DT:.0f} s, last at {t[n - 1]:.1f} s)')


# ── Report ────────────────────────────────────────────────────────────────────

def _line(r):
    tm = f"{r['t_mission']:.1f} s" if np.isfinite(r['t_mission']) else 'never'
    return (f"T_mission {tm}, all home {r['t_last_home']:.1f} s, swaps {r['trips']}, coverage "
            f"{100 * r['coverage']:.3f} %, sprayed >= 2x {100 * r['overspray']:.2f} %")


def report(out):
    print('\n=== S068 3D Large-Scale Field Spraying over a DEM — Results ===')
    xs = out['xs']
    gx = 2 * np.pi * A_DEM / LAM_X
    gy = 2 * np.pi * A_DEM / LAM_Y
    W = 2 * np.pi * V / LAM_Y
    zeta = K_ZD / (2 * math.sqrt(K_Z))
    H = K_Z / complex(K_Z - W ** 2, K_ZD * W)
    cuts = [(int(a), int(b)) for a, b in out['cuts']]
    print(f'  Run grid {len(xs)} runs (2D base); B partition run blocks {cuts} (2D-optimised)')
    print(f'  DEM: max slope dz/dx {gx:.3f}, dz/dy {gy:.3f}; along-run terrain frequency '
          f'{W:.3f} rad/s; altitude loop omega_n {math.sqrt(K_Z):.1f} rad/s, zeta {zeta:.2f}')
    print(f'  Card loop without feed-forward: steady sinusoidal tracking error on a 1.5 m run = '
          f'{A_DEM * abs(1 - H):.3f} m (> eps_z {EPS_Z} m); with velocity feed-forward '
          f'{A_DEM * abs(1 - K_Z / complex(K_Z - W ** 2, K_ZD * W) * complex(1, K_ZD * W / K_Z)):.4f} m')
    print(f'  Swath at 2.0 / 2.4 m AGL: {2 * 2.0 * TAN_C:.3f} / {2 * 2.4 * TAN_C:.3f} m; '
          f'run spacing {RUN_SPACING} m -> runs touch while AGL >= {RUN_SPACING / (2 * TAN_C):.3f} m')
    print(f'  Card claim "+-0.3 m AGL -> +-15 % swath": +-0.3 / 2.0 = {100 * 0.3 / 2.0:.1f} %')
    print(f'  Card wash index for two co-located drones at 2.0 / 2.4 m AGL: '
          f'{math.exp(-2.0 ** 2 / (2 * SIG_Z ** 2)) + math.exp(-2.4 ** 2 / (2 * SIG_Z ** 2)):.3f} '
          f'(both at 2.0 m: {2 * math.exp(-2.0 ** 2 / (2 * SIG_Z ** 2)):.3f}); vertical-separation '
          f'factor of a 0.4 m stagger exp(-0.4^2 / 2 sigma_z^2) = {math.exp(-0.16 / (2 * SIG_Z ** 2)):.3f}')
    for p in ('E', 'B'):
        print(f"  2D-base reproduction ({p}, flat, one station point, swath radius 1.0 m): "
              f"{_line(out['repro'][p])} (2D S068: {'1175.9' if p == 'E' else '1144.1'} s)")
    for p in ('E', 'B'):
        print(f'  --- partition {p} (pads 3 m apart) ---')
        for cfg in CONFIGS:
            r = out['main'][(cfg, p)]
            print(f'    [{CONFIGS[cfg][0]}] {_line(r)}')
            print(f"        spraying: AGL {r['agl_min']:.3f}-{r['agl_max']:.3f} m, swath "
                  f"{r['swath_min']:.3f}-{r['swath_max']:.3f} m, outside the card band "
                  f"{AGL_BAND} {100 * r['band_frac']:.1f} %, |eps_z| max {r['eps_max']:.3f} m / mean "
                  f"{r['eps_mean']:.4f} m / > 0.1 m {100 * r['eps_frac']:.2f} %; max |eps_z| more "
                  f"than 5 s into a run {r['eps_max_late']:.3f} m")
            ev = r['min_sep_event']
            print(f"        closest approach: drones {ev[0]}-{ev[1]} at t = {ev[2]:.1f} s near "
                  f"({ev[3]:.1f}, {ev[4]:.1f}) m, both {ev[5]}/{ev[6]}, vertical gap {ev[7]:.2f} m; "
                  f"pair-time with horizontal separation < 2 m: {r['close_time']:.1f} s; "
                  f"last spray per drone " + ', '.join(f"{d['t_cover']:.1f}" for d in r['drones']) + ' s')
            print(f"        wash: max Phi card {r['phi_card']:.3f} (while spraying {r['phi_card_spray']:.2e}), "
                  f"samples > 0.15 {100 * r['phi_card_frac']:.2f} %; max Phi vertical-separation "
                  f"{r['phi_corr']:.3f} (spraying {r['phi_corr_spray']:.2e}); min 3D separation "
                  f"{r['min_sep']:.2f} m; min horizontal separation while spraying "
                  f"{r['min_hsep_spray']:.1f} m; card 3D-distance trigger shortfall {r['shortfall']:.3f} s")
    f, b = out['main'][('flat', 'B')], out['main'][('tf_stag', 'B')]
    t_ = out['main'][('tf', 'B')]
    print(f"  Overhead of terrain following (B): T_mission {b['t_mission'] - f['t_mission']:+.1f} s "
          f"({100 * (b['t_mission'] / f['t_mission'] - 1):+.2f} %) with stagger, "
          f"{t_['t_mission'] - f['t_mission']:+.1f} s without")
    sw = out['sweep']
    print('  Sweep: stagger dh (B, TF, card loop): dh -> max Phi card / vertical-sep, min 3D separation, '
          'coverage, >= 2x, T_mission')
    for dh, r in sw['stagger'].items():
        print(f"    {dh:.1f} m: {r['phi_card']:.3f} / {r['phi_corr']:.3f}, {r['min_sep']:.2f} m, "
              f"{100 * r['coverage']:.3f} %, {100 * r['overspray']:.2f} %, {r['t_mission']:.1f} s")
    print('  Sweep: DEM amplitude (B): A -> coverage fixed / TF / TF+stagger+FF; max |eps_z| TF (card loop) '
          '/ TF+FF; fixed-plan AGL range')
    for a, rr in sw['amp'].items():
        print(f"    {a:.1f} m: {100 * rr['fixed']['coverage']:.3f} / {100 * rr['tf']['coverage']:.3f} / "
              f"{100 * rr['tf_stag_ff']['coverage']:.3f} %; {rr['tf']['eps_max']:.3f} / "
              f"{rr['tf_stag_ff']['eps_max']:.4f} m; {rr['fixed']['agl_min']:.2f}-{rr['fixed']['agl_max']:.2f} m; "
              f"T_mission TF {rr['tf']['t_mission']:.1f} s")
    print('  Sweep: pad spacing (B, TF + stagger): spacing -> max Phi card / vertical-sep, min 3D separation')
    for pd, r in sw['pad'].items():
        print(f"    {pd:.1f} m: {r['phi_card']:.3f} / {r['phi_corr']:.3f}, {r['min_sep']:.2f} m, "
              f"T_mission {r['t_mission']:.1f} s")


if __name__ == '__main__':
    res = run_simulation()
    report(res)
    od = os.path.normpath(OUTPUT_DIR)
    os.makedirs(od, exist_ok=True)
    plot_trajectory_3d(res, od)
    plot_altitude(res, od)
    plot_wash(res, od)
    plot_coverage(res, od)
    plot_progress_breakdown(res, od)
    save_animation(res, od)
