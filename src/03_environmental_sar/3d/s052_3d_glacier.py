"""
S052 3D Glacier Area Monitoring
===============================
The S052 survey drone (25 mm lens, 17.3 x 13.0 mm sensor, 4608 x 3456 px, 80 % / 60 %
overlap, 10 m/s) now works over a 3D glacier: a 180 m ramp along the flow direction
plus six Gaussian mounds (synthetic DEM, z_base = 2800 m), with vertical ice cliffs
(20-40 m) at the terminus and along both lateral margins.

Strategies (card), all on the same DEM, cliffs and noise draws per seed:
  1. Flat-plane boustrophedon (S052 2D base): the 2D strips at one fixed altitude,
     max(DEM) + 80 m (the lowest fixed altitude that keeps 80 m clearance)
  2. Terrain-following nadir: z_cmd = z_dem + 80 m, first-order lag tau_z = 1 s,
     |z_dot| <= 3 m/s
  3. Full 3D: terrain-following nadir + oblique cliff legs (30 m stand-off, camera
     70 deg from nadir). Two leg designs: the card's single mid-height leg per face,
     and stacked legs that cover the full face height with 60 % overlap (added).
Probe: ideal terrain following (z = z_cmd exactly) isolates the effect of the lag.

Usage:
    conda activate drones
    python src/03_environmental_sar/3d/s052_3d_glacier.py
"""

import os
import math
import importlib.util
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401

SCRIPT_DIR   = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, '..', '..', '..'))
OUTPUT_DIR   = os.path.join(PROJECT_ROOT, 'outputs', '03_environmental_sar', '3d', 's052_3d_glacier')

_spec = importlib.util.spec_from_file_location('s052_2d', os.path.join(SCRIPT_DIR, '..', 's052_glacier.py'))
s2d = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(s2d)

# -- Parameters (card / 2D) ------------------------------------------------------------
H_AGL      = 80.0                 # m nominal clearance (card)
FOCAL      = s2d.FOCAL_LENGTH     # 25 mm
SENSOR_W, SENSOR_H = s2d.SENSOR_W, s2d.SENSOR_H
PIX        = s2d.PIXEL_PITCH      # mm
KW, KL     = s2d.KW, s2d.KL       # footprint per metre of range
P_OVER_F   = PIX / FOCAL          # rad per pixel
O_F, O_S   = s2d.FORWARD_OVERLAP, s2d.SIDE_OVERLAP
W_AREA, L_AREA = s2d.GLACIER_W, s2d.GLACIER_L     # 600 m (x, across strips), 800 m (y, along strips)
V          = s2d.DRONE_SPEED      # 10 m/s
GSD_LIMIT  = s2d.GSD_LIMIT_CM / 100.0             # m
MIN_VIEWS  = s2d.MIN_VIEWS        # 3
Z_BASE     = 2800.0               # m
DZ_RAMP    = 180.0                # m over the 800 m flow length
N_MOUNDS   = 6
TAU_Z      = 1.0                  # s
VZ_MAX     = 3.0                  # m/s
DT         = 0.1                  # s  (2D: 0.5 s; finer for the 1 s lag)
DEM_RES    = 5.0                  # m  volume grid (card delta)
COV_RES    = s2d.COV_RES          # 1 m coverage grid
D_STANDOFF = 30.0                 # m
PSI        = math.radians(70.0)   # camera pitch from nadir
H_CLIFF_RANGE = (20.0, 40.0)
RHO_ICE    = 917.0
Z_CARD_BOUNDS = (2780.0, 3060.0)

# -- Parameters (added) ----------------------------------------------------------------
MOUND_A_RANGE = (10.0, 35.0)      # m |amplitude|; 4 positive (ridges), 2 negative (hollows)
MOUND_SIG_RANGE = (30.0, 90.0)    # m
MOUND_SCALES = (0.0, 0.5, 1.0, 1.5, 2.0)   # amplitude multiplier sweep (1.0 = main)
CLIFFS = [  # name, start (x, y), end (x, y), outward normal
    ('terminus', (150.0, 0.0), (450.0, 0.0), (0.0, -1.0)),
    ('west margin', (0.0, 100.0), (0.0, 400.0), (-1.0, 0.0)),
    ('east margin', (W_AREA, 350.0), (W_AREA, 700.0), (1.0, 0.0)),
]
FACE_RES   = 1.0                  # m face sample spacing
THIN_TERM, THIN_HEAD = 3.0, 0.5   # m thinning between epochs at terminus / head (linear)
K_SFM      = 1.5                  # SfM vertical error = K_SFM x GSD (per 5 m cell, iid)
K_SFM_SWEEP = (0.5, 1.5, 3.0)
N_MC       = 10
VZ_SWEEP   = (4.5, 6.0, 10.0)     # m/s  vertical-speed limit sweep for TF (card 3 m/s)
N_SWEEP    = 5                    # seeds per mound-amplitude sweep point
MAIN_SEED  = 0
GIF_FRAMES = 110                   # target frame count (GIF <= 3 MB)

STRATS = ['flat', 'tf', 'full_card', 'full']
NADIR_MODES = ('flat', 'tf', 'nolimit', 'ideal')
LABELS = {'flat': 'Flat-plane boustrophedon (2D base)', 'tf': 'Terrain-following nadir',
          'full_card': 'Full 3D, card legs (1 per face)', 'full': 'Full 3D, stacked legs'}
COLORS = {'flat': '#888888', 'tf': '#1F77B4', 'full_card': '#FF7F0E', 'full': '#D62728',
          'ideal': '#2CA02C'}


# -- Terrain ---------------------------------------------------------------------------

def make_mounds(rng):
    signs = np.array([1, 1, 1, 1, -1, -1], dtype=float)
    out = []
    for k in range(N_MOUNDS):
        out.append((rng.uniform(60.0, W_AREA - 60.0), rng.uniform(80.0, L_AREA - 80.0),
                    signs[k] * rng.uniform(*MOUND_A_RANGE),
                    rng.uniform(*MOUND_SIG_RANGE), rng.uniform(*MOUND_SIG_RANGE)))
    return out


class Glacier:
    def __init__(self, seed, mound_scale=1.0):
        rng = np.random.default_rng([seed, 11])
        self.mounds = [(x, y, mound_scale * a, sx, sy) for x, y, a, sx, sy in make_mounds(rng)]
        self.h_cliff = rng.uniform(*H_CLIFF_RANGE, size=len(CLIFFS))

    def z(self, x, y):
        x = np.asarray(x, float); y = np.asarray(y, float)
        z = Z_BASE + DZ_RAMP * y / L_AREA
        for mx, my, a, sx, sy in self.mounds:
            z = z + a * np.exp(-(x - mx) ** 2 / (2 * sx ** 2) - (y - my) ** 2 / (2 * sy ** 2))
        return z

    def grid(self, res):
        gx, gy = s2d.grid_centres(res)
        return gx, gy, self.z(gx[None, :], gy[:, None])

    def faces(self):
        """Face samples: points (n,3), face id, outward normal per face, tangent per face."""
        out = []
        for fid, (name, p0, p1, nrm) in enumerate(CLIFFS):
            p0, p1, nrm = np.array(p0), np.array(p1), np.array(nrm)
            L = np.linalg.norm(p1 - p0)
            t = (p1 - p0) / L
            s = np.arange(FACE_RES / 2, L, FACE_RES)
            xy = p0[None, :] + s[:, None] * t[None, :]
            top = self.z(xy[:, 0], xy[:, 1])
            h = self.h_cliff[fid]
            dz = np.arange(FACE_RES / 2, h, FACE_RES)
            pts = np.array([[x, y, zt - d] for (x, y), zt in zip(xy, top) for d in dz])
            out.append({'name': name, 'p0': p0, 'p1': p1, 'n': np.append(nrm, 0.0),
                        't': np.append(t, 0.0), 'h': h, 's': s, 'top': top, 'pts': pts,
                        'shape': (len(s), len(dz))})
        return out


# -- Flight ----------------------------------------------------------------------------

def nadir_layout():
    strips, bxs = s2d.nominal_layout()
    wps, photos, sop = s2d.build_plan(strips, bxs, 'bous')
    return strips, bxs, wps, photos, sop


def sample_path(wps):
    """Horizontal positions every V*DT along the polyline (exact waypoints included)."""
    seg = np.linalg.norm(np.diff(wps, axis=0), axis=1)
    cum = np.concatenate([[0.0], np.cumsum(seg)])
    s = np.arange(0.0, cum[-1], V * DT)
    s = np.unique(np.concatenate([s, cum]))
    x = np.interp(s, cum, wps[:, 0])
    y = np.interp(s, cum, wps[:, 1])
    return s, x, y


def fly_nadir(gl, wps, mode, z_flat=None, vz_max=VZ_MAX):
    """3D nadir trajectory. mode: 'flat' (fixed z), 'tf' (lag law, |z_dot| <= VZ_MAX),
    'nolimit' (lag law without the vertical-speed limit, probe), 'ideal' (z = z_cmd, probe)."""
    s, x, y = sample_path(wps)
    zg = gl.z(x, y)
    zc = zg + H_AGL
    if mode == 'flat':
        z = np.full_like(x, z_flat)
    elif mode == 'ideal':
        z = zc.copy()
    else:
        z = np.empty_like(x)
        z[0] = zc[0]
        for i in range(1, len(x)):
            dt = (s[i] - s[i - 1]) / V
            rate = (zc[i - 1] - z[i - 1]) / TAU_Z
            if mode == 'tf':
                rate = min(max(rate, -vz_max), vz_max)
            z[i] = z[i - 1] + rate * dt
        if mode == 'tf':
            vz = np.diff(z) / np.maximum(np.diff(s) / V, 1e-12)
            assert np.all(np.abs(vz) <= vz_max + 1e-9), 'vertical speed limit exceeded'
    agl = z - zg
    assert agl.min() > 10.0, f'terrain clearance lost ({agl.min():.1f} m)'
    return {'s': s, 'x': x, 'y': y, 'z': z, 'zg': zg, 'zc': zc, 'agl': agl}


def photo_positions(traj, wps, photos, sop):
    """Exact distance triggers: path arc length of each photo, z interpolated."""
    seg = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(wps, axis=0), axis=1))])
    a = wps[2 * sop]
    s_ph = seg[2 * sop] + np.hypot(photos[:, 0] - a[:, 0], photos[:, 1] - a[:, 1])
    z = np.interp(s_ph, traj['s'], traj['z'])
    return np.column_stack([photos, z])


def path_metrics(xyz, t_extra=0.0):
    d = np.diff(xyz, axis=0)
    dist = float(np.linalg.norm(d, axis=1).sum())
    asc = float(np.clip(d[:, 2], 0, None).sum())
    return dist, asc


def oblique_legs(gl, face, design):
    """Oblique leg geometry for one face. design 'card' (1 leg, axis at mid-height) or 'stack'.

    Camera axis at PSI from nadir looks toward the face; with horizontal stand-off d_s the
    axis meets the face d_s / tan(PSI) below the camera, the slant range is d_s / sin(PSI)
    and the vertical field of view covers [d_s / tan(PSI + th_v), d_s / tan(PSI - th_v)]
    below the camera.
    """
    th_v = math.atan(SENSOR_H / 2 / FOCAL)
    d_hi = D_STANDOFF / math.tan(PSI + th_v)          # nearest-to-camera depth below camera
    d_lo = D_STANDOFF / math.tan(PSI - th_v)          # deepest
    ext = d_lo - d_hi
    h = face['h']
    if design == 'card':
        offs = [h / 2 - D_STANDOFF / math.tan(PSI)]   # camera height below the top, negative = above
    else:
        step = ext * (1 - O_S)
        n = 1 + max(0, math.ceil((h - ext) / step - 1e-9))
        offs = [-d_hi + j * step for j in range(n)]
        if -d_hi + (n - 1) * step + d_lo < h:      # guarantee the base is inside the last leg
            offs.append(h - d_lo)
    R = D_STANDOFF / math.sin(PSI)
    w_fp = R * KW
    b = w_fp * (1 - O_F)
    L = np.linalg.norm(face['p1'] - face['p0'])
    margin = w_fp / 2
    s = np.arange(-margin, L + margin + 1e-9, b)
    t2, n2 = face['t'][:2], face['n'][:2]
    legs = []
    for j, off in enumerate(offs):
        ss = s if j % 2 == 0 else s[::-1]
        xy_face = face['p0'][None, :] + ss[:, None] * t2[None, :]
        top = gl.z(np.clip(xy_face[:, 0], 0, W_AREA), np.clip(xy_face[:, 1], 0, L_AREA))
        xy = xy_face + D_STANDOFF * n2[None, :]
        z = top - off
        legs.append(np.column_stack([xy, z]))
    return legs, {'R': R, 'w_fp': w_fp, 'b': b, 'ext': ext, 'd_hi': d_hi, 'd_lo': d_lo,
                  'n_legs': len(offs)}


def leg_time(leg):
    """Time along a leg at <= V horizontally and <= VZ_MAX vertically."""
    d = np.diff(leg, axis=0)
    dh = np.linalg.norm(d[:, :2], axis=1)
    return float(np.sum(np.maximum(dh / V, np.abs(d[:, 2]) / VZ_MAX)))


def transit(a, b, z_safe):
    """Climb to z_safe, fly straight, descend (3D polyline, time at V / VZ_MAX)."""
    za = max(a[2], z_safe)
    pts = np.array([a, [a[0], a[1], za], [b[0], b[1], za], b])
    d = np.diff(pts, axis=0)
    t = abs(d[0, 2]) / VZ_MAX + np.linalg.norm(d[1, :2]) / V + abs(d[2, 2]) / VZ_MAX
    return pts, t


# -- Cameras and coverage --------------------------------------------------------------

def nadir_cams(xyz):
    n = len(xyz)
    return {'c': xyz, 'a': np.tile([0.0, 0.0, -1.0], (n, 1)), 'r': np.tile([1.0, 0.0, 0.0], (n, 1)),
            'u': np.tile([0.0, 1.0, 0.0], (n, 1))}


def oblique_cams(legs, face):
    c = np.vstack(legs)
    n = len(c)
    a = -math.sin(PSI) * face['n'] + math.cos(PSI) * np.array([0.0, 0.0, -1.0])
    r = face['t']
    u = np.cross(r, a)
    return {'c': c, 'a': np.tile(a, (n, 1)), 'r': np.tile(r, (n, 1)), 'u': np.tile(u, (n, 1))}


def face_views(face, cams):
    """Per face point: number of photos that see it (front side, in frame, GSD <= limit)."""
    pts, nrm = face['pts'], face['n']
    cnt = np.zeros(len(pts), dtype=int)
    gsd_best = np.full(len(pts), np.inf)
    n_front = 0
    for k in range(len(cams['c'])):
        v = pts - cams['c'][k]
        rng_ = np.linalg.norm(v, axis=1)
        front = (v @ nrm) < 0
        n_front += int(front.sum())
        depth = v @ cams['a'][k]
        ok = front & (depth > 0)
        xi = FOCAL * (v @ cams['r'][k]) / np.where(depth > 0, depth, 1)
        yi = FOCAL * (v @ cams['u'][k]) / np.where(depth > 0, depth, 1)
        ok &= (np.abs(xi) <= SENSOR_W / 2) & (np.abs(yi) <= SENSOR_H / 2)
        cos_inc = -(v @ nrm) / rng_
        gsd = rng_ * P_OVER_F / np.maximum(cos_inc, 1e-9)
        ok &= gsd <= GSD_LIMIT
        cnt += ok
        gsd_best = np.where(ok, np.minimum(gsd_best, gsd), gsd_best)
    return cnt, gsd_best, n_front


def coverage_count(gl, xyz, res=COV_RES):
    """Photos per ground cell; terrain-aware nadir pinhole with per-photo altitude (2D model)."""
    gx, gy, Z = gl.grid(res)
    cnt = np.zeros(Z.shape, dtype=np.int32)
    agl_sum = np.zeros(Z.shape)
    for px, py, pz in xyz:
        hw = (pz - Z.min()) * KW / 2
        hl = (pz - Z.min()) * KL / 2
        i0 = max(0, int((px - hw) / res)); i1 = min(gx.size, int((px + hw) / res) + 2)
        j0 = max(0, int((py - hl) / res)); j1 = min(gy.size, int((py + hl) / res) + 2)
        if i0 >= i1 or j0 >= j1:
            continue
        agl = pz - Z[j0:j1, i0:i1]
        inside = ((np.abs(gx[i0:i1][None, :] - px) <= agl * KW / 2)
                  & (np.abs(gy[j0:j1][:, None] - py) <= agl * KL / 2))
        cnt[j0:j1, i0:i1] += inside
        agl_sum[j0:j1, i0:i1] += np.where(inside, agl, 0.0)
    mean_agl = np.where(cnt > 0, agl_sum / np.maximum(cnt, 1), np.nan)
    return cnt, mean_agl


def overlap_maps(gl, strips, bxs, traj_strip_z, res=COV_RES):
    """Local side / forward overlap at every cell from the drone's altitude over that cell.

    traj_strip_z(i, y) gives the drone altitude on strip i at along-strip position y.
    Side overlap uses the lower of the two neighbouring strips' AGL (conservative).
    """
    gx, gy, Z = gl.grid(res)
    k = np.clip(np.searchsorted(strips, gx) - 1, 0, len(strips) - 2)
    near = np.argmin(np.abs(gx[:, None] - strips[None, :]), axis=1)
    zs = np.array([traj_strip_z(i, gy) for i in range(len(strips))])    # (n_strips, ny)
    agl_l = zs[k].T - Z
    agl_r = zs[k + 1].T - Z
    agl_n = zs[near].T - Z
    b_side = (strips[k + 1] - strips[k])[None, :]
    o_s = 1 - b_side / (np.minimum(agl_l, agl_r) * KW)
    o_f = 1 - bxs[near][None, :] / (agl_n * KL)
    return o_s, o_f


def strip_z_func(traj, strips, wps):
    """Altitude on each strip as a function of y (from the simulated trajectory)."""
    seg = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(wps, axis=0), axis=1))])
    funcs = []
    for i in range(len(strips)):
        s0, s1 = seg[2 * i], seg[2 * i + 1]
        m = (traj['s'] >= s0 - 1e-9) & (traj['s'] <= s1 + 1e-9)
        yy, zz = traj['y'][m], traj['z'][m]
        o = np.argsort(yy, kind='stable')
        funcs.append((yy[o], zz[o]))
    return lambda i, y: np.interp(y, funcs[i][0], funcs[i][1])


# -- Volume ----------------------------------------------------------------------------

def thinning(y):
    return THIN_TERM + (THIN_HEAD - THIN_TERM) * y / L_AREA


def volume_estimates(gl, cnt, mean_agl, rng, k_sfm=K_SFM):
    """V and dV from noisy reconstructed DEMs (5 m cells, error K_SFM x GSD, holes filled)."""
    gx, gy, Z2 = gl.grid(DEM_RES)
    Z1 = Z2 + thinning(gy)[:, None]                 # reference epoch (thicker)
    f = int(round(DEM_RES / COV_RES))
    c5 = cnt.reshape(cnt.shape[0] // f, f, cnt.shape[1] // f, f).min(axis=(1, 3))
    a5 = np.nanmean(mean_agl.reshape(cnt.shape[0] // f, f, cnt.shape[1] // f, f), axis=(1, 3))
    valid = c5 >= MIN_VIEWS
    sig = k_sfm * a5 * P_OVER_F
    sig = np.where(valid, sig, np.nan)
    e1 = rng.standard_normal(Z2.shape)
    e2 = rng.standard_normal(Z2.shape)
    Zh1 = Z1 + np.nan_to_num(sig) * e1
    Zh2 = Z2 + np.nan_to_num(sig) * e2
    if not valid.all():                              # holes: fill with the mean of valid cells
        Zh1 = np.where(valid, Zh1, np.nanmean(np.where(valid, Zh1, np.nan)))
        Zh2 = np.where(valid, Zh2, np.nanmean(np.where(valid, Zh2, np.nan)))
    a = DEM_RES ** 2
    V1, V2 = a * np.clip(Z1 - Z_BASE, 0, None).sum(), a * np.clip(Z2 - Z_BASE, 0, None).sum()
    V1h, V2h = a * np.clip(Zh1 - Z_BASE, 0, None).sum(), a * np.clip(Zh2 - Z_BASE, 0, None).sum()
    return {'V1': V1, 'V2': V2, 'dV': V2 - V1, 'V2h': V2h, 'dVh': V2h - V1h,
            'V_err': (V2h - V2) / V2, 'dV_err': (V2h - V1h - (V2 - V1)) / abs(V2 - V1),
            'rmse': float(np.sqrt(np.nanmean((np.where(valid, Zh2 - Z2, np.nan)) ** 2))),
            'holes': int((~valid).sum())}


# -- Simulation ------------------------------------------------------------------------

def survey(seed, mound_scale=1.0, full=True, vz_max=VZ_MAX, modes=None):
    gl = Glacier(seed, mound_scale)
    strips, bxs, wps, photos, sop = nadir_layout()
    gx, gy, Zc = gl.grid(COV_RES)
    z_max_dem = float(max(Zc.max(), gl.z(*sample_path(wps)[1:]).max()))
    z_flat = z_max_dem + H_AGL
    out = {'gl': gl, 'strips': strips, 'bxs': bxs, 'wps': wps, 'z_flat': z_flat,
           'z_dem_min': float(Zc.min()), 'z_dem_max': z_max_dem}
    for mode in (NADIR_MODES if modes is None else modes):
        tr = fly_nadir(gl, wps, mode, z_flat, vz_max)
        xyz = photo_positions(tr, wps, photos, sop)
        gz = gl.z(xyz[:, 0], xyz[:, 1])
        gsd = (xyz[:, 2] - gz) * P_OVER_F
        cnt, magl = coverage_count(gl, xyz)
        o_s, o_f = overlap_maps(gl, strips, bxs, strip_z_func(tr, strips, wps))
        dist, asc = path_metrics(np.column_stack([tr['x'], tr['y'], tr['z']]))
        out[mode] = {'traj': tr, 'xyz': xyz, 'gsd': gsd, 'cnt': cnt, 'mean_agl': magl,
                     'o_s': o_s, 'o_f': o_f, 'dist': dist, 'asc': asc, 'time': tr['s'][-1] / V,
                     'agl_trig': xyz[:, 2] - gz}
    if not full:
        return out
    faces = gl.faces()
    out['faces'] = faces
    cams_nadir = nadir_cams(out['tf']['xyz'])
    out['face_nadir'] = [face_views(f, cams_nadir) for f in faces]
    cams_flat = nadir_cams(out['flat']['xyz'])
    out['face_flat'] = [face_views(f, cams_flat) for f in faces]
    z_safe = z_flat
    for design, key in (('card', 'full_card'), ('stack', 'full')):
        legs_all, info_all, views = [], [], []
        t_leg, d_leg, a_leg = 0.0, 0.0, 0.0
        pos = np.array([out['tf']['traj']['x'][-1], out['tf']['traj']['y'][-1], out['tf']['traj']['z'][-1]])
        segs = []
        t_tr = 0.0
        for f in faces:
            legs, info = oblique_legs(gl, f, design)
            assert np.all(legs[0][:, 2] - (np.array([f['top'].min()]) - f['h']) > 0), 'leg below ground'
            pts, tt = transit(pos, legs[0][0], z_safe)
            segs.append(pts); t_tr += tt
            for j, lg in enumerate(legs):
                segs.append(lg)
                t_leg += leg_time(lg)
                if j + 1 < len(legs):
                    link = np.array([lg[-1], legs[j + 1][0]])
                    segs.append(link)
                    t_leg += abs(link[1, 2] - link[0, 2]) / VZ_MAX + \
                        np.linalg.norm(link[1, :2] - link[0, :2]) / V
            pos = legs[-1][-1]
            legs_all.append(legs); info_all.append(info)
            views.append(face_views(f, oblique_cams(legs, f)))
        poly = np.vstack(segs)
        d_ex, a_ex = path_metrics(poly)
        # ground clearance of the oblique legs over the proglacial ground in front of each face
        for f, legs in zip(faces, legs_all):
            ground = f['top'].min() - f['h']
            assert min(lg[:, 2].min() for lg in legs) > ground, 'oblique leg below the proglacial ground'
        n_obl = sum(len(np.vstack(lg)) for lg in legs_all)
        out[key] = {'legs': legs_all, 'info': info_all, 'views': views, 'poly': poly,
                    'dist': out['tf']['dist'] + d_ex, 'asc': out['tf']['asc'] + a_ex,
                    'time': out['tf']['time'] + t_tr + t_leg, 'extra_dist': d_ex,
                    't_transit': t_tr, 't_legs': t_leg, 'n_obl': n_obl}
    return out


def seed_metrics(sv, seed):
    """Scalar metrics per strategy for one survey (same noise draws for every strategy)."""
    m = {}
    for s_ in STRATS + ['nolimit', 'ideal']:
        nad = sv[s_] if s_ in NADIR_MODES else sv['tf']
        d = {}
        d['dist'] = nad['dist'] if s_ in NADIR_MODES else sv[s_]['dist']
        d['asc'] = nad['asc'] if s_ in NADIR_MODES else sv[s_]['asc']
        d['time'] = float(nad['time'] if s_ in NADIR_MODES else sv[s_]['time'])
        d['gsd_min'], d['gsd_max'] = float(nad['gsd'].min()), float(nad['gsd'].max())
        d['gsd_mean'] = float(nad['gsd'].mean())
        d['gsd_cv'] = float(nad['gsd'].std() / nad['gsd'].mean())
        d['gsd_exceed'] = int(np.sum(nad['gsd'] > GSD_LIMIT))
        d['agl_min'], d['agl_max'] = float(nad['agl_trig'].min()), float(nad['agl_trig'].max())
        d['traj_agl_min'] = float(nad['traj']['agl'].min())
        d['cov_min'], d['cov_mean'] = int(nad['cnt'].min()), float(nad['cnt'].mean())
        d['frac3'] = float(np.mean(nad['cnt'] >= MIN_VIEWS))
        d['compliant'] = float(np.mean((nad['o_s'] >= O_S - 1e-9) & (nad['o_f'] >= O_F - 1e-9)))
        d['fwd_ok'] = float(np.mean(nad['o_f'] >= O_F - 1e-9))
        d['side_ok'] = float(np.mean(nad['o_s'] >= O_S - 1e-9))
        d['fwd_min'], d['side_min'] = float(nad['o_f'].min()), float(nad['o_s'].min())
        d['severe'] = float(np.mean((nad['o_s'] < O_S - 0.05) | (nad['o_f'] < O_F - 0.05)))
        if 'faces' in sv and s_ in STRATS:
            if s_ in ('flat', 'tf'):
                views = sv['face_flat'] if s_ == 'flat' else sv['face_nadir']
            else:
                views = sv[s_]['views']
            npts = sum(len(f['pts']) for f in sv['faces'])
            d['face_cov'] = sum(int(np.sum(v[0] >= MIN_VIEWS)) for v in views) / npts
            d['face_cov_each'] = [float(np.mean(v[0] >= MIN_VIEWS)) for v in views]
            d['face_front'] = sum(v[2] for v in views)
        rng = np.random.default_rng([seed, 21])
        d['vol'] = volume_estimates(sv['gl'], nad['cnt'], nad['mean_agl'], rng)
        d['vol_k'] = {k: volume_estimates(sv['gl'], nad['cnt'], nad['mean_agl'],
                                          np.random.default_rng([seed, 21]), k)
                      for k in K_SFM_SWEEP}
        m[s_] = d
    return m


def melt_areas(sv, seed):
    """S052 NDWI melt area (0.5 m ortho raster, >= 3 photos) for flat vs terrain-following."""
    pools = s2d.make_pools(np.random.default_rng([seed, 2]))
    ndwi, truth = s2d.synthetic_ndwi(pools, np.random.default_rng([seed, 3]))
    a_true = float(sum(np.pi * p[2] * p[3] for p in pools))
    out = {'true': a_true}
    for mode in ('flat', 'tf'):
        a, _, _ = s2d.melt_area(ndwi, sv[mode]['cnt'])
        out[mode] = a
    return out


def run_simulation():
    main = survey(MAIN_SEED)
    mc = []
    for seed in range(N_MC):
        sv = main if seed == MAIN_SEED else survey(seed)
        m = seed_metrics(sv, seed)
        m['melt'] = melt_areas(sv, seed)
        m['z_flat'], m['relief'] = sv['z_flat'], sv['z_dem_max'] - sv['z_dem_min']
        m['z_range'] = {s_: (float(min(sv['tf']['traj']['z'].min(), min(lg[:, 2].min() for legs in sv[s_]['legs'] for lg in legs))),
                             float(max(sv['tf']['traj']['z'].max(), max(lg[:, 2].max() for legs in sv[s_]['legs'] for lg in legs))))
                        for s_ in ('full_card', 'full')}
        m['n_legs'] = {s_: [i['n_legs'] for i in sv[s_]['info']] for s_ in ('full_card', 'full')}
        m['h_cliff'] = sv['gl'].h_cliff.copy()
        mc.append(m)
        del sv
    sweep = {}
    for sc in MOUND_SCALES:
        rows = []
        for seed in range(N_SWEEP):
            sv = survey(seed, sc, full=False)
            rows.append({md: {'agl_min': float(sv[md]['agl_trig'].min()),
                              'traj_agl_min': float(sv[md]['traj']['agl'].min()),
                              'compliant': float(np.mean((sv[md]['o_s'] >= O_S - 1e-9)
                                                         & (sv[md]['o_f'] >= O_F - 1e-9))),
                              'severe': float(np.mean((sv[md]['o_s'] < O_S - 0.05)
                                                      | (sv[md]['o_f'] < O_F - 0.05))),
                              'agl_max': float(sv[md]['agl_trig'].max()),
                              'gsd_max': float(sv[md]['gsd'].max()),
                              'gsd_cv': float(sv[md]['gsd'].std() / sv[md]['gsd'].mean()),
                              'asc': sv[md]['asc'], 'frac3': float(np.mean(sv[md]['cnt'] >= MIN_VIEWS))}
                         for md in NADIR_MODES})
            rows[-1]['z_flat'] = sv['z_flat']
            del sv
        sweep[sc] = rows
    vz_sweep = {}
    for vz in VZ_SWEEP:
        rows = []
        for seed in range(N_SWEEP):
            sv = survey(seed, 1.0, full=False, vz_max=vz, modes=('tf',))
            rows.append({'severe': float(np.mean((sv['tf']['o_s'] < O_S - 0.05) | (sv['tf']['o_f'] < O_F - 0.05))),
                         'traj_agl_min': float(sv['tf']['traj']['agl'].min())})
            del sv
        vz_sweep[vz] = rows
    return {'main': main, 'mc': mc, 'sweep': sweep, 'vz_sweep': vz_sweep}


# -- Report ----------------------------------------------------------------------------

def _mc(res, s_, key):
    return np.array([m[s_][key] for m in res['mc']], dtype=float)


def report(res):
    mc, sv = res['mc'], res['main']
    fp_w, fp_l, gsd80 = s2d.compute_footprint(H_AGL)
    print('=== S052 3D Glacier Area Monitoring ===')
    print(f'Camera at {H_AGL:.0f} m AGL: GSD {100 * gsd80:.3f} cm/px (card table: ~4.8), footprint '
          f'{fp_w:.2f} x {fp_l:.2f} m; GSD reaches {100 * GSD_LIMIT:.0f} cm at {GSD_LIMIT / P_OVER_F:.0f} m range')
    print(f'  pixel pitch {1000 * PIX:.3f} um; the card\'s 4.8 cm/px would need {0.048 / P_OVER_F:.0f} m AGL')
    print(f'Nadir layout (2D S052 nominal): {len(sv["strips"])} strips, Bx = {sv["bxs"][0]:.2f} m, '
          f'By = {sv["strips"][1] - sv["strips"][0]:.3f} m, {len(sv["tf"]["xyz"])} photos; last strip at '
          f'x = {sv["strips"][-1]:.2f} m; 2D path length {s2d.path_length(sv["wps"]):.0f} m')
    th_v = math.degrees(math.atan(SENSOR_H / 2 / FOCAL))
    info = sv['full']['info'][0]
    R_card = D_STANDOFF / math.cos(PSI)
    print(f'Oblique geometry: slant range d_s/sin(psi) = {info["R"]:.2f} m (card d_s/cos(psi) = {R_card:.2f} m, '
          f'GSD {100 * R_card * P_OVER_F:.2f} cm; card table 8.4 cm); axis meets the face '
          f'{D_STANDOFF / math.tan(PSI):.2f} m below the camera; on-axis face GSD '
          f'{100 * info["R"] * P_OVER_F / math.cos(math.pi / 2 - PSI):.3f} cm')
    print(f'  vertical half-FOV {th_v:.2f} deg -> face seen from {info["d_hi"]:.2f} to {info["d_lo"]:.2f} m below '
          f'the camera ({info["ext"]:.2f} m per leg); along-leg footprint {info["w_fp"]:.2f} m, trigger every '
          f'{info["b"]:.2f} m; stacked-leg spacing {info["ext"] * (1 - O_S):.2f} m')

    print(f'\nMain seed {MAIN_SEED}: DEM {sv["z_dem_min"]:.1f}-{sv["z_dem_max"]:.1f} m (relief '
          f'{sv["z_dem_max"] - sv["z_dem_min"]:.1f} m); flat-plane altitude {sv["z_flat"]:.1f} m; cliffs '
          + ', '.join(f'{c[0]} {h:.1f} m' for c, h in zip(CLIFFS, sv['gl'].h_cliff)))
    m0 = mc[MAIN_SEED]
    keys = [('dist', 'distance (m)', '{:.0f}'), ('asc', 'total ascent (m)', '{:.0f}'),
            ('time', 'time (s)', '{:.0f}'), ('gsd_min', 'GSD min (cm)', 'c'), ('gsd_max', 'GSD max (cm)', 'c'),
            ('gsd_cv', 'GSD CV (%)', 'p'), ('gsd_exceed', 'triggers > 5 cm', '{:.0f}'),
            ('agl_min', 'trigger AGL min (m)', '{:.1f}'), ('agl_max', 'trigger AGL max (m)', '{:.1f}'),
            ('traj_agl_min', 'flight AGL min (m)', '{:.1f}'), ('cov_min', 'photos/cell min', '{:.0f}'),
            ('cov_mean', 'photos/cell mean', '{:.2f}'), ('frac3', 'area >= 3 photos (%)', 'p'),
            ('compliant', 'area meeting 80/60 (%)', 'p'), ('severe', 'area > 5 pp short (%)', 'p'),
            ('side_min', 'min side overlap (%)', 'p'), ('fwd_min', 'min fwd overlap (%)', 'p'),
            ('face_cov', 'cliff face covered (%)', 'p')]
    cols = STRATS + ['nolimit', 'ideal']
    print('  metric | ' + ' | '.join(cols))
    for k, lab, fmt in keys:
        row = []
        for c in cols:
            v = m0[c].get(k)
            if v is None:
                row.append('-')
            elif fmt == 'c':
                row.append(f'{100 * v:.3f}')
            elif fmt == 'p':
                row.append(f'{100 * v:.2f}')
            else:
                row.append(fmt.format(v))
        print(f'  {lab} | ' + ' | '.join(row))
    for c in STRATS:
        print(f'  {c}: face coverage per face ' + ', '.join(
            f'{CLIFFS[i][0]} {100 * v:.1f}%' for i, v in enumerate(m0[c]['face_cov_each'])))
    print(f'  nadir photos front-facing to a cliff (photo x face-point pairs): flat {m0["flat"]["face_front"]}, '
          f'tf {m0["tf"]["face_front"]} (all from the strip at x = {sv["strips"][-1]:.1f} m outside the east margin)')
    print(f'  oblique legs per face: card {m0["n_legs"]["full_card"]}, stacked {m0["n_legs"]["full"]}; '
          f'oblique photos card {sv["full_card"]["n_obl"]}, stacked {sv["full"]["n_obl"]}; '
          f'transit / leg time card {sv["full_card"]["t_transit"]:.0f} / {sv["full_card"]["t_legs"]:.0f} s, '
          f'stacked {sv["full"]["t_transit"]:.0f} / {sv["full"]["t_legs"]:.0f} s')
    zr = {'flat': (sv['z_flat'], sv['z_flat']),
          'tf': (float(sv['tf']['traj']['z'].min()), float(sv['tf']['traj']['z'].max()))}
    zr.update(m0['z_range'])
    print(f'  altitude range vs card bounds {Z_CARD_BOUNDS}: ' + ', '.join(
        f'{c} {a:.1f}-{b:.1f} m{"" if Z_CARD_BOUNDS[0] <= a and b <= Z_CARD_BOUNDS[1] else " (outside)"}'
        for c, (a, b) in zr.items()))
    for c in ('flat', 'tf'):
        v = m0[c]['vol']
        print(f'  volume ({c}): V_ref {v["V1"]:.4e} m3, V_cur {v["V2"]:.4e} m3, dV true {v["dV"]:.0f} m3 '
              f'(dM {RHO_ICE * v["dV"] / 1000:.0f} t); estimated dV {v["dVh"]:.0f} m3, error {100 * v["dV_err"]:+.4f}%, '
              f'V error {100 * v["V_err"]:+.6f}%, DEM RMSE {100 * v["rmse"]:.2f} cm, holes {v["holes"]}')

    print(f'\nMonte Carlo ({N_MC} seeds: mounds, cliff heights, SfM noise, NDWI pools; same draws for every strategy):')
    for k, lab, fmt in keys:
        parts = []
        for c in cols:
            if k not in mc[0][c]:
                continue
            v = _mc(res, c, k)
            sc = 100 if fmt in ('c', 'p') else 1
            parts.append(f'{c} {sc * v.mean():.2f} [{sc * v.min():.2f}, {sc * v.max():.2f}]')
        print(f'  {lab}: ' + '; '.join(parts))
    dd = _mc(res, 'tf', 'dist') / _mc(res, 'flat', 'dist') - 1
    print(f'  terrain-following distance overhead vs flat: mean {100 * dd.mean():+.2f}%, range '
          f'{100 * dd.min():+.2f} to {100 * dd.max():+.2f}%')
    for c in ('full_card', 'full'):
        dt_ = _mc(res, c, 'time') / _mc(res, 'tf', 'time') - 1
        print(f'  {c} time overhead vs terrain-following nadir: mean {100 * dt_.mean():+.1f}%, range '
              f'{100 * dt_.min():+.1f} to {100 * dt_.max():+.1f}%')
    sev_f, sev_t = _mc(res, 'flat', 'severe'), _mc(res, 'tf', 'severe')
    cv_f, cv_t = _mc(res, 'flat', 'gsd_cv'), _mc(res, 'tf', 'gsd_cv')
    print(f'  per seed: TF GSD CV lower than flat in {int(np.sum(cv_t < cv_f))}/{N_MC}; TF area > 5 pp short larger '
          f'than flat in {int(np.sum(sev_t > sev_f))}/{N_MC} (flat max {100 * sev_f.max():.2f}%)')
    gr = np.array([m['flat']['gsd_max'] / m['flat']['gsd_min'] for m in mc])
    print(f'  flat GSD max/min ratio: mean {gr.mean():.2f}, range {gr.min():.2f}-{gr.max():.2f}; '
          f'relief mean {np.mean([m["relief"] for m in mc]):.1f} m; flat altitude range '
          f'{min(m["z_flat"] for m in mc):.1f}-{max(m["z_flat"] for m in mc):.1f} m')
    for c in ('flat', 'tf'):
        e = np.array([m[c]['vol']['dV_err'] for m in mc])
        r = np.array([m[c]['vol']['rmse'] for m in mc])
        print(f'  dV error ({c}, K_SFM {K_SFM}): mean |e| {100 * np.abs(e).mean():.4f}%, max |e| '
              f'{100 * np.abs(e).max():.4f}%; DEM RMSE mean {100 * r.mean():.2f} cm')
    for k in K_SFM_SWEEP:
        parts = []
        for c in ('flat', 'tf'):
            e = np.array([m[c]['vol_k'][k]['dV_err'] for m in mc])
            parts.append(f'{c} max |dV error| {100 * np.abs(e).max():.4f}%')
        print(f'  K_SFM = {k}: ' + ', '.join(parts))
    for c in ('full_card', 'full'):
        fc = _mc(res, c, 'face_cov')
        print(f'  {c}: cliff face covered mean {100 * fc.mean():.2f}%, range {100 * fc.min():.2f}-{100 * fc.max():.2f}%; '
              f'legs per face ' + ', '.join(str(m['n_legs'][c]) for m in mc))
    print('  cliff heights per seed: ' + '; '.join(', '.join(f'{h:.0f}' for h in m['h_cliff']) for m in mc))
    mt = [(m['melt']['true'], m['melt']['flat'], m['melt']['tf']) for m in mc]
    ef = np.array([(f_ - t) / t for t, f_, _ in mt]); et = np.array([(g - t) / t for t, _, g in mt])
    print(f'  NDWI melt area error: flat {100 * ef.mean():+.3f}% [{100 * ef.min():+.3f}, {100 * ef.max():+.3f}], '
          f'tf {100 * et.mean():+.3f}% [{100 * et.min():+.3f}, {100 * et.max():+.3f}]; identical in '
          f'{sum(abs(a - b) < 1e-9 for _, a, b in mt)}/{N_MC} seeds')

    print(f'\nMound-amplitude sweep ({N_SWEEP} seeds each; 1.0 = main):')
    for sc in MOUND_SCALES:
        rows = res['sweep'][sc]
        parts = []
        for md in NADIR_MODES:
            a = np.array([r[md]['traj_agl_min'] for r in rows])
            amax = np.array([r[md]['agl_max'] for r in rows])
            sev = np.array([r[md]['severe'] for r in rows])
            gm = np.array([r[md]['gsd_max'] for r in rows])
            parts.append(f'{md}: AGL {a.min():.1f}-{amax.max():.1f} m, >5pp short {100 * sev.mean():.2f}%, '
                         f'GSD max {100 * gm.max():.2f} cm')
        print(f'  scale {sc:.1f}: ' + ' | '.join(parts))

    print(f'Vertical-speed-limit sweep (terrain following, mound scale 1.0, {N_SWEEP} seeds):')
    rows3 = res['sweep'][1.0]
    vals = [(VZ_MAX, [r['tf']['severe'] for r in rows3], [r['tf']['traj_agl_min'] for r in rows3])]
    vals += [(vz, [r['severe'] for r in res['vz_sweep'][vz]], [r['traj_agl_min'] for r in res['vz_sweep'][vz]])
             for vz in VZ_SWEEP]
    for vz, sev, amin in vals:
        print(f'  v_z,max {vz:4.1f} m/s: area > 5 pp short {100 * np.mean(sev):.2f}%, lowest flight AGL {min(amin):.1f} m')


# -- Plotting --------------------------------------------------------------------------

def _save(fig, out, name):
    p = os.path.join(out, name)
    fig.savefig(p, dpi=120, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {p}')


def plot_terrain_3d(res, out):
    sv = res['main']
    gl = sv['gl']
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d', computed_zorder=False)
    gx, gy, Z = gl.grid(20.0)
    X, Y = np.meshgrid(gx, gy)
    ax.plot_surface(X, Y, Z, cmap='Blues_r', alpha=0.45, linewidth=0, zorder=0)
    for f in sv['faces']:
        p0, p1 = f['p0'], f['p1']
        ax.plot([p0[0], p1[0]], [p0[1], p1[1]], [f['top'][0] - f['h'], f['top'][-1] - f['h']], color='k', lw=1)
        for k in (0, len(f['s']) - 1):
            xy = p0 + f['s'][k] * f['t'][:2]
            ax.plot([xy[0]] * 2, [xy[1]] * 2, [f['top'][k] - f['h'], f['top'][k]], color='k', lw=1)
    tr = sv['tf']['traj']
    ax.plot(tr['x'][::5], tr['y'][::5], tr['z'][::5], color='red', lw=0.7, zorder=3, label='terrain-following nadir path')
    ax.plot([0, W_AREA], [L_AREA / 2] * 2, [sv['z_flat']] * 2, color=COLORS['flat'], lw=2, ls='--',
            label=f'flat-plane altitude {sv["z_flat"]:.0f} m (2D base)')
    for legs in sv['full']['legs']:
        for lg in legs:
            ax.plot(lg[:, 0], lg[:, 1], lg[:, 2], color='orange', lw=1.5, zorder=4)
    ax.plot([], [], color='orange', label='oblique cliff legs (stacked design)')
    xyz, gsd = sv['tf']['xyz'][::4], sv['tf']['gsd'][::4]
    ok = gsd <= GSD_LIMIT
    ax.scatter(xyz[ok, 0], xyz[ok, 1], xyz[ok, 2], s=1, color='green', depthshade=False, zorder=5,
               label=f'triggers, GSD <= 5 cm ({int(np.sum(sv["tf"]["gsd"] <= GSD_LIMIT))} of {len(sv["tf"]["gsd"])})')
    ax.set_xlabel('x across (m)'); ax.set_ylabel('y along flow (m)'); ax.set_zlabel('z (m)')
    ax.set_box_aspect((600, 800, 300)); ax.view_init(elev=24, azim=-60)
    ax.legend(loc='upper left', fontsize=8)
    ax.set_title(f'S052 3D glacier (seed {MAIN_SEED}): DEM, terrain-following path, cliff faces (black) '
                 'and oblique legs', fontsize=10)
    _save(fig, out, 'terrain_trajectory_3d.png')


def plot_altitude_profile(res, out):
    sv = res['main']
    tr = sv['tf']['traj']
    seg = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(sv['wps'], axis=0), axis=1))])
    i_min = int(np.argmin(tr['agl']))
    strip = int(np.searchsorted(seg, tr['s'][i_min]) - 1) // 2
    s0, s1 = seg[2 * strip], seg[2 * strip + 1]
    print(f'  altitude profile: strip {strip + 1} at x = {sv["strips"][strip]:.1f} m has the lowest TF AGL '
          f'({tr["agl"][i_min]:.1f} m)')
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    for md, col, lab in (('tf', COLORS['tf'], 'terrain-following (lag, 3 m/s)'),
                         ('nolimit', 'purple', 'probe: lag only, no 3 m/s limit'),
                         ('ideal', COLORS['ideal'], 'probe: ideal (z = z_cmd)')):
        t = sv[md]['traj']
        m = (t['s'] >= s0) & (t['s'] <= s1)
        a1.plot(t['y'][m], t['z'][m], color=col, lw=1.2, label=lab)
        a2.plot(t['y'][m], t['agl'][m], color=col, lw=1.2)
    m = (tr['s'] >= s0) & (tr['s'] <= s1)
    a1.fill_between(tr['y'][m], Z_BASE - 20, tr['zg'][m], color='saddlebrown', alpha=0.4, label='DEM under the strip')
    a1.plot(tr['y'][m], tr['zc'][m], 'k--', lw=0.8, label='z_cmd = DEM + 80 m')
    a1.axhline(sv['z_flat'], color=COLORS['flat'], lw=1.5, label='flat plane')
    a2.plot(tr['y'][m], sv['flat']['traj']['agl'][m], color=COLORS['flat'], lw=1.2)
    a2.axhspan(H_AGL - 1, H_AGL + 1, color='green', alpha=0.15)
    a1.set_ylabel('z (m)'); a2.set_ylabel('AGL (m)'); a2.set_xlabel('y along strip (m)')
    a1.legend(fontsize=8, loc='upper left'); a1.grid(alpha=0.3); a2.grid(alpha=0.3)
    a1.set_title(f'Altitude along strip {strip + 1} (x = {sv["strips"][strip]:.1f} m, the strip with the '
                 f'lowest terrain-following AGL, {tr["agl"][i_min]:.1f} m)', fontsize=10)
    _save(fig, out, 'altitude_profile.png')


def plot_gsd_map(res, out):
    sv = res['main']
    fig, axes = plt.subplots(1, 2, figsize=(12, 7.5))
    vmax = 100 * max(sv['flat']['gsd'].max(), GSD_LIMIT)
    for ax, md in zip(axes, ('flat', 'tf')):
        xyz, g = sv[md]['xyz'], 100 * sv[md]['gsd']
        sc = ax.scatter(xyz[:, 0], xyz[:, 1], c=g, s=2, cmap='viridis', vmin=0, vmax=vmax)
        ax.set_xlim(-10, W_AREA + 15); ax.set_ylim(0, L_AREA); ax.set_aspect('equal'); ax.set_xlabel('x (m)')
        ax.set_title(f'{LABELS[md]}\nGSD {g.min():.2f}-{g.max():.2f} cm/px, CV {100 * g.std() / g.mean():.1f}%',
                     fontsize=10)
        fig.colorbar(sc, ax=ax, shrink=0.7, label='GSD at trigger (cm/px)')
    axes[0].set_ylabel('y (m)')
    fig.suptitle(f'S052 3D GSD at every trigger, seed {MAIN_SEED} (limit {100 * GSD_LIMIT:.0f} cm/px, '
                 f'nominal {100 * H_AGL * P_OVER_F:.2f} cm/px)')
    _save(fig, out, 'gsd_map.png')


def plot_coverage_heatmap(res, out):
    sv = res['main']
    fig, axes = plt.subplots(1, 3, figsize=(16, 7.5))
    for ax, md in zip(axes[:2], ('flat', 'tf')):
        im = ax.imshow(np.clip(sv[md]['cnt'], 0, 40), origin='lower', extent=[0, W_AREA, 0, L_AREA],
                       cmap='RdYlBu', vmin=0, vmax=40)
        ax.set_title(f'{LABELS[md]}: photos per 1 m cell\nmin {sv[md]["cnt"].min()}, mean {sv[md]["cnt"].mean():.1f}',
                     fontsize=10)
        fig.colorbar(im, ax=ax, shrink=0.7)
    short = 100 * np.minimum(sv['tf']['o_s'] - O_S, sv['tf']['o_f'] - O_F)
    im = axes[2].imshow(np.clip(short, -20, 5), origin='lower', extent=[0, W_AREA, 0, L_AREA], cmap='RdBu',
                        vmin=-20, vmax=20)
    fig.colorbar(im, ax=axes[2], shrink=0.7, label='min(side - 60, fwd - 80) (pp)')
    axes[2].set_title(f'Terrain-following: overlap margin\n{100 * np.mean(short < -5):.1f}% of the area > 5 pp short',
                      fontsize=10)
    for ax in axes:
        ax.set_xlabel('x (m)')
    axes[0].set_ylabel('y (m)')
    _save(fig, out, 'coverage_heatmap.png')


def plot_cliff_coverage(res, out):
    sv = res['main']
    fig, axes = plt.subplots(2, 3, figsize=(17, 8.5))
    fig.subplots_adjust(hspace=0.45, wspace=0.25)
    for r, key in enumerate(('full_card', 'full')):
        for c, f in enumerate(sv['faces']):
            cnt = sv[key]['views'][c][0].reshape(f['shape']).T
            ax = axes[r, c]
            im = ax.imshow(cnt, origin='upper', aspect='auto', cmap='viridis', vmin=0, vmax=12,
                           extent=[0, f['s'][-1] + FACE_RES / 2, f['h'], 0])
            ax.contour(np.arange(f['shape'][0]) * FACE_RES + FACE_RES / 2,
                       np.arange(f['shape'][1]) * FACE_RES + FACE_RES / 2, cnt, levels=[MIN_VIEWS - 0.5],
                       colors='red', linewidths=0.8)
            ax.set_title(f'{LABELS[key]}\n{f["name"]} (h = {f["h"]:.1f} m):\n'
                         f'{100 * np.mean(cnt >= MIN_VIEWS):.1f}% seen by >= 3 photos at <= 5 cm', fontsize=9)
            ax.set_xlabel('distance along face (m)')
            if c == 0:
                ax.set_ylabel('depth below cliff top (m)')
    fig.colorbar(im, ax=axes, shrink=0.8, label='qualifying photos per face point')
    fig.suptitle('S052 3D cliff-face coverage (red contour = 3 photos); nadir surveys cover 0% of every face')
    _save(fig, out, 'cliff_coverage.png')


def plot_volume(res, out):
    mc = res['mc']
    m0 = mc[MAIN_SEED]
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(12, 4.6))
    v = m0['tf']['vol']
    a1.bar(['reference epoch', 'current epoch'], [v['V1'] / 1e6, v['V2'] / 1e6], color=['#9ecae1', '#3182bd'])
    a1.set_ylabel('glacier volume above z_base (10^6 m^3)')
    a1.set_title(f'True volume, seed {MAIN_SEED}: dV = {v["dV"] / 1e3:.0f} x 10^3 m^3, '
                 f'dM = {RHO_ICE * v["dV"] / 1e6:.0f} x 10^3 t', fontsize=10)
    for c, x in (('flat', 0), ('tf', 1)):
        for k_i, k in enumerate(K_SFM_SWEEP):
            e = 100 * np.abs([m[c]['vol_k'][k]['dV_err'] for m in mc])
            a2.bar(x + (k_i - 1) * 0.25, e.mean(), 0.25, color=COLORS[c], alpha=0.4 + 0.2 * k_i,
                   label=f'{c}, K_SFM = {k}')
    a2.set_xticks([0, 1], ['flat plane', 'terrain-following'])
    a2.set_ylabel('mean |dV error| (%)')
    a2.set_title(f'Mass-balance error over {N_MC} seeds (SfM error = K_SFM x GSD)', fontsize=10)
    a2.legend(fontsize=7)
    _save(fig, out, 'volume_bars.png')


def plot_strategy_comparison(res, out):
    mc = res['mc']
    cols = STRATS
    panels = [('dist', 'flight distance (km)', 1e-3), ('time', 'mission time (min)', 1 / 60),
              ('gsd_cv', 'GSD CV at triggers (%)', 100), ('severe', 'area > 5 pp below 80/60 (%)', 100),
              ('face_cov', 'cliff face covered (%)', 100)]
    fig, axes = plt.subplots(1, len(panels), figsize=(20, 4.5))
    for ax, (k, lab, sc) in zip(axes, panels):
        for i, c in enumerate(cols):
            v = sc * _mc(res, c, k)
            ax.bar(i, v.mean(), color=COLORS[c], alpha=0.8)
            ax.scatter(np.full(len(v), i), v, color='k', s=8, zorder=3)
        ax.set_xticks(range(len(cols)), ['flat', 'TF', 'full\n(card)', 'full\n(stacked)'])
        ax.set_title(lab, fontsize=10); ax.grid(axis='y', alpha=0.3)
    fig.suptitle(f'S052 3D strategy comparison ({N_MC} seeds; bars = mean, dots = seeds)')
    _save(fig, out, 'strategy_comparison.png')


def plot_mound_sweep(res, out):
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(12, 4.5))
    cols = {'flat': COLORS['flat'], 'tf': COLORS['tf'], 'nolimit': 'purple', 'ideal': COLORS['ideal']}
    for md in NADIR_MODES:
        amin = [min(r[md]['traj_agl_min'] for r in res['sweep'][sc]) for sc in MOUND_SCALES]
        sev = [100 * np.mean([r[md]['severe'] for r in res['sweep'][sc]]) for sc in MOUND_SCALES]
        a1.plot(MOUND_SCALES, amin, 'o-', color=cols[md], label=md)
        a2.plot(MOUND_SCALES, sev, 'o-', color=cols[md], label=md)
    a1.axhline(H_AGL, color='k', ls=':', lw=1)
    a1.set_xlabel('mound amplitude scale (1.0 = main)'); a1.set_ylabel('lowest flight AGL over seeds (m)')
    a2.set_xlabel('mound amplitude scale (1.0 = main)'); a2.set_ylabel('area > 5 pp below 80/60 (%)')
    a1.legend(fontsize=8); a2.legend(fontsize=8); a1.grid(alpha=0.3); a2.grid(alpha=0.3)
    fig.suptitle(f'Sensitivity to the added mound amplitudes ({N_SWEEP} seeds each)')
    _save(fig, out, 'mound_sweep.png')


def save_animation(res, out):
    sv = res['main']
    gl = sv['gl']
    tr = sv['tf']['traj']
    nad = np.column_stack([tr['x'], tr['y'], tr['z']])
    extra = []
    for a, b in zip(sv['full']['poly'][:-1], sv['full']['poly'][1:]):
        n = max(1, int(np.ceil(np.linalg.norm(b - a) / (V * DT))))
        extra.append(a + (b - a) * (np.arange(n) / n)[:, None])
    extra.append(sv['full']['poly'][-1:])
    path = np.vstack([nad, np.vstack(extra)])
    agl = path[:, 2] - gl.z(np.clip(path[:, 0], 0, W_AREA), np.clip(path[:, 1], 0, L_AREA))
    n = len(path)
    step = int(np.ceil((n - 1) / (GIF_FRAMES - 1)))
    n_frames = int(np.ceil((n - 1) / step)) + 1
    idx = [min(f * step, n - 1) for f in range(n_frames)]
    fig = plt.figure(figsize=(11, 5))
    ax = fig.add_subplot(1, 2, 1, projection='3d')
    gx, gy, Z = gl.grid(25.0)
    X, Y = np.meshgrid(gx, gy)
    ax.plot_surface(X, Y, Z, cmap='Blues_r', alpha=0.4, lw=0)
    ax.set_box_aspect((600, 800, 300)); ax.view_init(elev=26, azim=-60)
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)'); ax.set_zlabel('z (m)')
    (line,) = ax.plot([], [], [], color='red', lw=0.6)
    (dot,) = ax.plot([], [], [], 'o', color='k', ms=4)
    ax2 = fig.add_subplot(1, 2, 2)
    ax2.set_xlim(0, n * DT / 60); ax2.set_ylim(0, max(130, agl.max() + 5))
    ax2.axhline(H_AGL, color='green', ls=':')
    ax2.set_xlabel('time (min, sample clock)'); ax2.set_ylabel('height above ground below drone (m)')
    ax2.grid(alpha=0.3)
    (aline,) = ax2.plot([], [], color=COLORS['tf'], lw=0.6)
    ttl = fig.suptitle('')
    tt = np.arange(n) * DT / 60

    def update(f):
        k = idx[f]
        line.set_data(path[:k + 1:5, 0], path[:k + 1:5, 1]); line.set_3d_properties(path[:k + 1:5, 2])
        dot.set_data([path[k, 0]], [path[k, 1]]); dot.set_3d_properties([path[k, 2]])
        aline.set_data(tt[:k + 1], agl[:k + 1])
        phase = 'nadir terrain-following' if k < len(nad) else 'cliff transit / oblique legs'
        ttl.set_text(f'S052 3D full survey (stacked legs), {phase}, AGL {agl[k]:.0f} m')
        return line, dot, aline, ttl

    anim = animation.FuncAnimation(fig, update, frames=n_frames, blit=False)
    p = os.path.join(out, 'animation.gif')
    anim.save(p, writer='pillow', fps=12, dpi=70)
    plt.close(fig)
    print(f'Saved: {p} ({n_frames} frames, every {step} path samples)')


if __name__ == '__main__':
    res = run_simulation()
    report(res)
    out = os.path.normpath(OUTPUT_DIR)
    os.makedirs(out, exist_ok=True)
    plot_terrain_3d(res, out)
    plot_altitude_profile(res, out)
    plot_gsd_map(res, out)
    plot_coverage_heatmap(res, out)
    plot_cliff_coverage(res, out)
    plot_volume(res, out)
    plot_strategy_comparison(res, out)
    plot_mound_sweep(res, out)
    save_animation(res, out)
