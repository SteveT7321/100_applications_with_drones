"""
S073 3D Solar Panel Thermal Inspection over Tilted Rows
=======================================================
The 2D S073 flies a nadir thermal camera at a fixed 5 m over a flat 80 x 60 m farm.
Here the 10 panel rows are tilted 25 deg (south-facing, y = north) with 2 m maintenance
gaps, so the panel surface rises from z = 0 to z = 1.865 m across every row, and the
flight altitude, cross-track offset and camera pointing are planned states.

Every frame (0.1 s) is evaluated in 3D: square 45 deg FOV around the camera boresight,
front-face visibility, occlusion by other rows, range-dependent motion blur
(b = |v x r| t_exp / (R^2 IFOV)), specular sun glare and cast shadows.

Plans (all at the card's 1.0 m/s and the same frame rate):
  2d_card    the 2D card lawnmower, 19 strips at a fixed 5 m (2D baseline)
  2d_row     the 2D row-centred plan, 10 strips at a fixed 5 m (2D baseline)
  card_low   the card's rule: lowest glare-free altitude h_edge + 1 m, lawnmower
  3d_row     one strip per row, altitude and offset planned to cover the tilted row
  3d_gimbal  one strip per row, tilt-normal gimbal, stand-off planned to cover the row

Studies: card sun (120 deg, 35 deg) Monte Carlo, planned-altitude sweep, margin sweep,
time-of-day shadow/glare series (latitude 35 N, equinox), and a solar-noon glare study
in which altitude moves the specular glare spot into the row gap.

Usage:
    conda activate drones
    python src/04_industrial_agriculture/3d/s073_3d_solar_thermal.py
"""

import math
import os
import sys
import time
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, '..', '..', '..'))
OUTPUT_DIR = os.path.join(PROJECT_ROOT, 'outputs', '04_industrial_agriculture', '3d',
                          's073_3d_solar_thermal')

# ── Parameters (card) ─────────────────────────────────────────────────────────
FARM_L, FARM_W = 80.0, 60.0      # m, x = east (along-track), y = north (cross-track)
N_ROWS, N_COLS = 10, 12
M_DEFECTIVE = 8
ALPHA_DEG = 25.0                 # panel tilt, south-facing
G_ROW = 2.0                      # m, maintenance gap between rows
FOV_DEG = 45.0
GAMMA_GLARE_DEG = 5.0
D_SAFE = 1.0                     # m above panel top edge
D_TRANSIT = 0.5                  # m extra clearance in transit
OVERLAP = 0.20
V_NOM = 1.0                      # m/s
SUN_AZ, SUN_EL = 120.0, 35.0     # deg (azimuth east of north, elevation)
T_NOMINAL = 35.0
DT_HOT_MIN, DT_HOT_MAX = 10.0, 20.0
DT_THRESH = 8.0
T_EXP = 0.033
BLUR_MAX = 2.5
DT = 0.1
# Kept from the 2D base
ALT_2D = 5.0                     # m, fixed altitude of the 2D plans
GSD_2D = 0.02                    # m/px at 5 m -> IFOV = 0.004 rad
GPS_ERROR = 0.1
SIGMA_HOT = 0.4
T_NOISE_STD = 0.5
DEFECT_SEED, NOISE_SEED = 42, 0

# ── Derived (card geometry made consistent with the 80 x 60 m farm) ───────────
ALPHA = np.deg2rad(ALPHA_DEG)
PITCH = FARM_W / N_ROWS                      # 6.0 m row pitch
DEPTH = PITCH - G_ROW                        # 4.0 m horizontal depth of a row
W_SLANT = DEPTH / np.cos(ALPHA)              # 4.414 m slant width
H_EDGE = DEPTH * np.tan(ALPHA)               # 1.865 m top-edge height
PANEL_L = FARM_L / N_COLS                    # 6.667 m
ROW_YB = PITCH * np.arange(N_ROWS) + G_ROW / 2   # bottom (south) edge y
ROW_YT = ROW_YB + DEPTH                          # top (north) edge y
ROW_YC = ROW_YB + DEPTH / 2
N_HAT = np.array([0.0, -np.sin(ALPHA), np.cos(ALPHA)])   # panel normal (up, south)
TAN_H = np.tan(np.deg2rad(FOV_DEG / 2))
HALF_FOV = np.deg2rad(FOV_DEG / 2)
GAMMA = np.deg2rad(GAMMA_GLARE_DEG)
IFOV = GSD_2D / ALT_2D
ATTEN = np.exp(-GPS_ERROR ** 2 / (2 * SIGMA_HOT ** 2))
H_SCAN_CARD = H_EDGE + D_SAFE                # 2.865 m
H_TRANSIT = H_EDGE + D_SAFE + D_TRANSIT      # 3.365 m
W_FOOT_2D = 2 * ALT_2D * TAN_H
STRIP_D_2D = W_FOOT_2D * (1 - OVERLAP)
PANEL_AREA = N_ROWS * FARM_L * W_SLANT

# ── Added parameters (see README Deviations) ──────────────────────────────────
PLAN_MARGIN = 0.2                # m, coverage margin beyond each row edge (2 x GPS error)
MARGIN_SWEEP = [0.0, 0.1, 0.2, 0.3, 0.5]
H_SWEEP = [4.5, 5.0, 5.5, 5.76, 6.0, 6.25, 6.5, 7.0, 8.0]
LAT_DEG, DECL_DEG = 35.0, 0.0    # site for the time-of-day study (09:00 ~ card sun)
DECL_SWEEP = [-23.44, 0.0, 23.44]
TOD_HOURS = np.round(np.arange(7.0, 17.0 + 1e-9, 0.25), 2)
NOON = 12.0
WINTER_HOUR = 9.0                # winter-solstice morning for the shadow probe
H_GRID = np.round(np.arange(4.0, 15.0 + 1e-9, 0.05), 3)   # glare-aware altitude search
H_MAX = 15.0
N_MC = 50
MC_SEED0 = 3000
COV_DS = 0.01                    # m, slant sampling of the coverage metric
ANIM_STEP = 90

PLANS = ['2d_card', '2d_row', 'card_low', '3d_row', '3d_gimbal']
LABELS = {'2d_card': '2D card lawnmower @5 m', '2d_row': '2D row-centred @5 m',
          'card_low': 'Card rule: lowest glare-free alt.', '3d_row': '3D row-aligned (planned alt.)',
          '3d_gimbal': '3D row-aligned, tilt-normal gimbal',
          '3d_row_glare': '3D row-aligned, glare-aware alt.'}
COLORS = {'2d_card': 'tab:grey', '2d_row': 'tab:olive', 'card_low': 'tab:purple',
          '3d_row': 'tab:red', '3d_gimbal': 'tab:orange', '3d_row_glare': 'tab:brown'}


# ── Sun geometry ──────────────────────────────────────────────────────────────
def sun_vec(az_deg, el_deg):
    """Unit vector of the sunlight travel direction (sun -> ground)."""
    az, el = np.deg2rad(az_deg), np.deg2rad(el_deg)
    return -np.array([np.sin(az) * np.cos(el), np.cos(az) * np.cos(el), np.sin(el)])


def spec_vec(s):
    return s - 2 * np.dot(s, N_HAT) * N_HAT


def solar_position(hour, lat_deg=LAT_DEG, decl_deg=DECL_DEG):
    lat, dec = np.deg2rad(lat_deg), np.deg2rad(decl_deg)
    H = np.deg2rad(15.0 * (hour - 12.0))
    el = np.arcsin(np.sin(lat) * np.sin(dec) + np.cos(lat) * np.cos(dec) * np.cos(H))
    az = np.arctan2(np.sin(H), np.cos(H) * np.sin(lat) - np.tan(dec) * np.cos(lat)) + np.pi
    return np.rad2deg(az) % 360.0, np.rad2deg(el)


def surface_z(y):
    y = np.asarray(y, dtype=float)
    z = np.zeros_like(y)
    for yb, yt in zip(ROW_YB, ROW_YT):
        m = (y >= yb) & (y <= yt)
        z[m] = (y[m] - yb) * np.tan(ALPHA)
    return z


# ── Camera ────────────────────────────────────────────────────────────────────
def camera_axes(mode):
    e1 = np.array([1.0, 0.0, 0.0])
    if mode == 'nadir':
        b = np.array([0.0, 0.0, -1.0])
    else:                                   # tilt-normal pointing: boresight = -n
        b = -N_HAT
    e2 = np.cross(b, e1)
    return b, e1, e2


def occluded_yz(Dy, Dz, Py, Pz, prow):
    """Segment D->P (cross-section) crosses another row's panel. Arrays broadcast."""
    occ = np.zeros(np.broadcast(Dy, Py).shape, dtype=bool)
    ey, ez = DEPTH, H_EDGE
    dy, dz = Py - Dy, Pz - Dz
    den = dy * ez - dz * ey
    safe = np.where(np.abs(den) < 1e-12, 1e-12, den)
    for k in range(N_ROWS):
        ay, az = ROW_YB[k] - Dy, 0.0 - Dz
        t = (ay * ez - az * ey) / safe
        u = (ay * dz - az * dy) / safe
        hit = (np.abs(den) > 1e-12) & (t > 1e-9) & (t < 1 - 1e-9) & (u >= 0) & (u <= 1)
        occ |= hit & (prow != k)
    return occ


def glare_spot(D, g):
    """First panel hit of the ray D + t g (g = -r_spec). Returns (hit, y, x, row)."""
    D = np.atleast_2d(D)
    gn = np.dot(g, N_HAT)
    best_t = np.full(len(D), np.inf)
    best_row = np.full(len(D), -1)
    if gn >= 0:
        return np.zeros(len(D), bool), np.full(len(D), np.nan), np.full(len(D), np.nan), best_row
    for k in range(N_ROWS):
        A = np.array([0.0, ROW_YB[k], 0.0])
        t = ((A - D) @ N_HAT) / gn
        y = D[:, 1] + t * g[1]
        x = D[:, 0] + t * g[0]
        ok = (t > 0) & (y >= ROW_YB[k]) & (y <= ROW_YT[k]) & (x >= 0) & (x <= FARM_L)
        upd = ok & (t < best_t)
        best_t[upd] = t[upd]
        best_row[upd] = k
    hit = best_row >= 0
    tt = np.where(hit, best_t, np.nan)
    return hit, D[:, 1] + tt * g[1], D[:, 0] + tt * g[0], best_row


def glare_in_fov(mode, g):
    b, e1, e2 = camera_axes(mode)
    d = g @ b
    if d <= 0:
        return False
    return (abs(np.arctan(g @ e1 / d)) <= HALF_FOV + GAMMA and
            abs(np.arctan(g @ e2 / d)) <= HALF_FOV + GAMMA)


def frame_glare(D, mode, sun):
    """Frame-wise glare: the specular sun reflection lands on a panel inside the FOV."""
    if sun is None:
        return np.zeros(len(np.atleast_2d(D)), bool)
    g = -spec_vec(sun_vec(*sun))
    if not glare_in_fov(mode, g):
        return np.zeros(len(np.atleast_2d(D)), bool)
    return glare_spot(D, g)[0]


def shadowed(P, prow, sun):
    """Point on a panel face is shaded (back-lit or a ray to the sun hits another row)."""
    s = sun_vec(*sun)
    if np.dot(s, N_HAT) >= 0:
        return np.ones(len(P), bool)
    w = -s                                   # toward the sun
    wn = np.dot(w, N_HAT)
    sh = np.zeros(len(P), bool)
    for k in range(N_ROWS):
        A = np.array([0.0, ROW_YB[k], 0.0])
        t = ((A - P) @ N_HAT) / wn
        y = P[:, 1] + t * w[1]
        x = P[:, 0] + t * w[0]
        sh |= (t > 1e-9) & (y >= ROW_YB[k]) & (y <= ROW_YT[k]) & (x >= 0) & (x <= FARM_L) & (prow != k)
    return sh


# ── Plans ─────────────────────────────────────────────────────────────────────
def h_star(margin=PLAN_MARGIN):
    return ((DEPTH + 2 * margin) / TAN_H + H_EDGE) / 2


def r_star(margin=PLAN_MARGIN):
    return (W_SLANT / 2 + margin) / TAN_H


def row_offset(H, margin=PLAN_MARGIN):
    """Feasible cross-track window [lo, hi] of the nadir drone relative to the row centre."""
    lo = DEPTH / 2 + margin - TAN_H * (H - H_EDGE)
    hi = -DEPTH / 2 - margin + TAN_H * H
    return lo, hi


def make_plan(key, H=None, margin=PLAN_MARGIN, dy=None):
    """Returns dict with strips [(y, z)], camera mode."""
    if key == '2d_card':
        n = int(np.ceil(FARM_W / STRIP_D_2D))
        strips = [((i + 0.5) * STRIP_D_2D, ALT_2D) for i in range(n)]
        mode = 'nadir'
    elif key == '2d_row':
        strips = [(y, ALT_2D) for y in ROW_YC]
        mode = 'nadir'
    elif key == 'card_low':
        H = H_SCAN_CARD if H is None else H
        d = 2 * TAN_H * (H - H_EDGE / 2) * (1 - OVERLAP)
        n = int(np.ceil(FARM_W / d))
        strips = [((i + 0.5) * d, H) for i in range(n)]
        mode = 'nadir'
    elif key in ('3d_row', '3d_row_glare'):
        H = h_star(margin) if H is None else H
        if dy is None:
            lo, hi = row_offset(H, margin)
            dy = 0.5 * (lo + hi)
        strips = [(y + dy, H) for y in ROW_YC]
        mode = 'nadir'
    elif key == '3d_gimbal':
        R = r_star(margin) if H is None else H
        strips = [(y + R * N_HAT[1], H_EDGE / 2 + R * N_HAT[2]) for y in ROW_YC]
        mode = 'normal'
    else:
        raise ValueError(key)
    return dict(key=key, strips=strips, mode=mode)


def waypoints(plan):
    wps = []
    st = plan['strips']
    for i, (y, z) in enumerate(st):
        xs, xe = (0.0, FARM_L) if i % 2 == 0 else (FARM_L, 0.0)
        wps += [(xs, y, z), (xe, y, z)]
        if i + 1 < len(st):
            y2, z2 = st[i + 1]
            if min(z, z2) < H_TRANSIT:      # climb - traverse - descend
                wps += [(xe, y, H_TRANSIT), (xe, y2, H_TRANSIT)]
    out = [wps[0]]
    for w in wps[1:]:
        if np.linalg.norm(np.subtract(w, out[-1])) > 1e-9:
            out.append(w)
    return np.array(out)


def fly3d(wps, v):
    seg = np.diff(wps, axis=0)
    L = np.linalg.norm(seg, axis=1)
    cum = np.concatenate([[0.0], np.cumsum(L)])
    total = cum[-1]
    s = np.arange(int(np.floor(total / (v * DT) + 1e-9)) + 1) * v * DT
    if s[-1] < total - 1e-9:
        s = np.append(s, total)
    idx = np.clip(np.searchsorted(cum, s, side='right') - 1, 0, len(seg) - 1)
    pos = wps[idx] + ((s - cum[idx]) / L[idx])[:, None] * seg[idx]
    vel = seg[idx] / L[idx][:, None] * v
    return pos, vel, idx, total


def check_path(pos, vel, v):
    step = np.linalg.norm(np.diff(pos, axis=0), axis=1)
    assert np.all(step <= v * DT + 1e-9), 'teleport / over-speed'
    clr = pos[:, 2] - surface_z(pos[:, 1])
    assert np.all(clr >= D_SAFE - 1e-9), f'clearance violated: {clr.min():.3f} m'
    assert np.all(pos[:, 0] >= -1e-9) and np.all(pos[:, 0] <= FARM_L + 1e-9), 'left the farm'
    return float(clr.min())


# ── Randomness (one stream per seed) ──────────────────────────────────────────
def draw_case(seed, model, defect_seed=None):
    rng = np.random.default_rng(seed if defect_seed is None else defect_seed)
    idx = rng.choice(N_ROWS * N_COLS, size=M_DEFECTIVE, replace=False)
    delta_t = rng.uniform(DT_HOT_MIN, DT_HOT_MAX, size=M_DEFECTIVE)
    off = rng.uniform(-1, 1, (M_DEFECTIVE, 2)) * \
        np.array([PANEL_L / 2 - SIGMA_HOT, W_SLANT / 2 - SIGMA_HOT])
    if model == 'centre':
        off = np.zeros_like(off)
    row, col = idx // N_COLS, idx % N_COLS
    x = (col + 0.5) * PANEL_L + off[:, 0]
    s = W_SLANT / 2 + off[:, 1]
    P = np.stack([x, ROW_YB[row] + s * np.cos(ALPHA), s * np.sin(ALPHA)], axis=1)
    return dict(idx=idx, row=row, col=col, dT=delta_t, P=P)


# ── One inspection ────────────────────────────────────────────────────────────
def inspect(plan, case, seed, v=V_NOM, sun=(SUN_AZ, SUN_EL), glare_rule='frame'):
    wps = waypoints(plan)
    pos, vel, segi, total = fly3d(wps, v)
    clr_min = check_path(pos, vel, v)
    F, M = len(pos), M_DEFECTIVE
    b, e1, e2 = camera_axes(plan['mode'])
    P = case['P']
    r = P[None, :, :] - pos[:, None, :]                     # F x M x 3
    d = r @ b
    in_fov = (d > 1e-9) & (np.abs(r @ e1) <= TAN_H * d) & (np.abs(r @ e2) <= TAN_H * d)
    front = (-r) @ N_HAT > 0
    cand = in_fov & front
    vis = cand.copy()
    fi, mi = np.nonzero(cand)
    if len(fi):
        occ = occluded_yz(pos[fi, 1], pos[fi, 2], P[mi, 1], P[mi, 2], case['row'][mi])
        vis[fi[occ], mi[occ]] = False
    R2 = np.sum(r ** 2, axis=2)
    omega = np.linalg.norm(np.cross(vel[:, None, :], r), axis=2) / R2
    blur = omega * T_EXP / IFOV
    blur_ok = blur < BLUR_MAX
    if glare_rule == 'frame':
        glare = np.repeat(frame_glare(pos, plan['mode'], sun)[:, None], M, axis=1)
    elif glare_rule == 'point':
        rs = spec_vec(sun_vec(*sun))
        ang = np.arccos(np.clip((-r) @ rs / np.sqrt(R2), -1, 1))
        glare = ang < GAMMA
    else:
        glare = np.zeros((F, M), bool)
    shade = np.repeat(shadowed(P, case['row'], sun)[None, :], F, axis=0)
    rng = np.random.default_rng(seed + 104729)
    t_meas = T_NOMINAL + case['dT'][None, :] * ATTEN + rng.normal(0, T_NOISE_STD, (F, M))
    valid = vis & blur_ok & ~glare & ~shade
    hit = valid & (t_meas > T_NOMINAL + DT_THRESH)
    det = hit.any(axis=0)
    t_det = np.where(det, hit.argmax(axis=0), -1)
    seen = vis.any(axis=0)
    return dict(pos=pos, vel=vel, det=det, t_det=t_det, det_rate=float(det.mean()),
                n_vis=vis.sum(axis=0), n_valid=valid.sum(axis=0),
                n_blur=(vis & ~blur_ok).sum(axis=0), n_glare=(vis & glare).sum(axis=0),
                miss_unseen=int((~det & ~seen).sum()),
                miss_blur=int((~det & seen & ((vis & ~glare & ~shade & blur_ok).sum(0) == 0)
                               & ((vis & ~blur_ok).sum(0) > 0)).sum()),
                miss_glare=int((~det & seen & ((vis & ~glare).sum(0) == 0)).sum()),
                shade=int(shade[0].sum()), glare_frames=float(glare[:, 0].mean()),
                t=total / v, L=total, clr_min=clr_min, n_frames=F)


# ── Geometric coverage of the panel surface (cross-section, along-track invariant) ──
def coverage(plan, v=V_NOM, sun=(SUN_AZ, SUN_EL), return_map=False):
    b, e1, e2 = camera_axes(plan['mode'])
    s = np.arange(COV_DS / 2, W_SLANT, COV_DS)
    Qy = (ROW_YB[:, None] + s[None, :] * np.cos(ALPHA)).ravel()
    Qz = np.tile(s * np.sin(ALPHA), N_ROWS)
    qrow = np.repeat(np.arange(N_ROWS), len(s))
    cov = np.zeros(len(Qy), bool)
    best_gsd = np.full(len(Qy), np.inf)
    rho_min = np.inf
    for (y, z) in plan['strips']:
        if frame_glare(np.array([[FARM_L / 2, y, z]]), plan['mode'], sun)[0]:
            continue
        ry, rz = Qy - y, Qz - z
        d = ry * b[1] + rz * b[2]
        a2 = ry * e2[1] + rz * e2[2]
        ok = (d > 1e-9) & (np.abs(a2) <= TAN_H * d)
        ok &= (-(ry * N_HAT[1] + rz * N_HAT[2])) > 0
        rho = np.hypot(ry, rz)
        idx = np.nonzero(ok)[0]
        if len(idx):
            occ = occluded_yz(np.full(len(idx), y), np.full(len(idx), z), Qy[idx], Qz[idx], qrow[idx])
            ok[idx[occ]] = False
        if ok.any():                       # closest visible point sets the abeam blur limit
            rho_min = min(rho_min, rho[ok].min())
        dx = TAN_H * np.maximum(d, 0)
        blur_best = v * rho / (dx ** 2 + rho ** 2) * T_EXP / IFOV
        ok &= blur_best < BLUR_MAX
        cos_inc = (-(ry * N_HAT[1] + rz * N_HAT[2])) / rho
        gsd = rho * IFOV / np.maximum(cos_inc, 1e-6)
        best_gsd = np.where(ok, np.minimum(best_gsd, gsd), best_gsd)
        cov |= ok
    res = dict(frac=float(cov.mean()), gsd_mean=float(best_gsd[cov].mean()) if cov.any() else np.nan,
               gsd_max=float(best_gsd[cov].max()) if cov.any() else np.nan,
               v_max=BLUR_MAX * IFOV * rho_min / T_EXP)
    if return_map:
        res.update(cov=cov.reshape(N_ROWS, len(s)), s=s)
    return res


def shadow_fraction(sun, nx=80, ns=45):
    xs = (np.arange(nx) + 0.5) * FARM_L / nx
    ss = (np.arange(ns) + 0.5) * W_SLANT / ns
    X, S, Rr = np.meshgrid(xs, ss, np.arange(N_ROWS), indexing='ij')
    P = np.stack([X.ravel(), ROW_YB[Rr.ravel()] + S.ravel() * np.cos(ALPHA),
                  S.ravel() * np.sin(ALPHA)], axis=1)
    return float(shadowed(P, Rr.ravel(), sun).mean())


def min_spec_angle(mode, sun, n=201):
    """Minimum angle between any FOV ray (reversed) and the specular direction."""
    b, e1, e2 = camera_axes(mode)
    a = np.tan(np.linspace(-HALF_FOV, HALF_FOV, n))
    A1, A2 = np.meshgrid(a, a)
    rays = b[None, :] + A1.ravel()[:, None] * e1 + A2.ravel()[:, None] * e2
    rays /= np.linalg.norm(rays, axis=1, keepdims=True)
    rs = spec_vec(sun_vec(*sun))
    return float(np.rad2deg(np.arccos(np.clip((-rays) @ rs, -1, 1)).min()))


def min_spec_angle_alt(H, sun, nx=41):
    """Over the panel points the 3d_row camera at altitude H sees from x = 40 m."""
    lo, hi = row_offset(H)
    D = np.array([FARM_L / 2, ROW_YC[5] + 0.5 * (lo + hi), H])
    s = np.arange(COV_DS / 2, W_SLANT, 0.02)
    rs = spec_vec(sun_vec(*sun))
    best = np.inf
    for k in (4, 5, 6):
        Py = ROW_YB[k] + s * np.cos(ALPHA)
        Pz = s * np.sin(ALPHA)
        for dx in np.linspace(-TAN_H * H, TAN_H * H, nx):
            r = np.stack([np.full_like(Py, dx), Py - D[1], Pz - D[2]], axis=1)
            d = -r[:, 2]
            ok = (np.abs(r[:, 0]) <= TAN_H * d) & (np.abs(r[:, 1]) <= TAN_H * d)
            if ok.any():
                u = -r[ok] / np.linalg.norm(r[ok], axis=1, keepdims=True)
                best = min(best, np.rad2deg(np.arccos(np.clip(u @ rs, -1, 1))).min())
    return best


def glare_aware_altitude(sun, margin=PLAN_MARGIN):
    """Lowest H >= H* (and an offset in the feasible window) whose glare spot misses
    every panel, for an interior row; None if no such H <= H_MAX."""
    g = -spec_vec(sun_vec(*sun))
    if not glare_in_fov('nadir', g):
        H = h_star(margin)
        lo, hi = row_offset(H, margin)
        return H, 0.5 * (lo + hi)
    for H in H_GRID:
        if H < h_star(margin) - 1e-9:
            continue
        lo, hi = row_offset(H, margin)
        offs = np.round(np.arange(lo, hi + 1e-9, 0.02), 3)
        offs = offs[np.argsort(np.abs(offs - 0.5 * (lo + hi)), kind='stable')]
        D = np.stack([np.full(len(offs), FARM_L / 2), ROW_YC[5] + offs, np.full(len(offs), H)], 1)
        hit = glare_spot(D, g)[0]
        if (~hit).any():
            return float(H), float(offs[np.argmax(~hit)])
    return None


# ── Simulation ────────────────────────────────────────────────────────────────
def run_mc(plan, model, sun=(SUN_AZ, SUN_EL), v=V_NOM, glare_rule='frame', n=N_MC):
    out = []
    for k in range(n):
        seed = MC_SEED0 + k
        r = inspect(plan, draw_case(seed, model), seed, v=v, sun=sun, glare_rule=glare_rule)
        out.append(dict(det=r['det_rate'], unseen=r['miss_unseen'], blur=r['miss_blur'],
                        glare=r['miss_glare'], t=r['t'], L=r['L'], clr=r['clr_min'],
                        gf=r['glare_frames'], shade=r['shade']))
    return out


def run_simulation():
    t0 = time.time()
    card_sun = (SUN_AZ, SUN_EL)
    plans = {k: make_plan(k) for k in PLANS}
    # Card rule: lowest glare-free altitude per strip (sweep z from h_edge + 1 m at 0.1 m)
    z_grid = np.round(np.arange(H_SCAN_CARD, ALT_2D + 1e-9, 0.1), 3)
    card_alt = []
    for (y, _) in plans['card_low']['strips']:
        D = np.stack([np.full(len(z_grid), FARM_L / 2), np.full(len(z_grid), y), z_grid], 1)
        gl = frame_glare(D, 'nadir', card_sun)
        card_alt.append(float(z_grid[np.argmax(~gl)]) if (~gl).any() else np.nan)
    # Nominal mission: card seeds, centre hot spots, 3d_row
    nominal = {}
    case_nom = draw_case(None, 'centre', defect_seed=DEFECT_SEED)
    for k in PLANS:
        nominal[k] = inspect(plans[k], case_nom, NOISE_SEED)
    geo = {k: coverage(plans[k], return_map=True) for k in PLANS}
    mc = {m: {k: run_mc(plans[k], m) for k in PLANS} for m in ('centre', 'anywhere')}
    v_low = 0.9 * geo['card_low']['v_max']
    mc_low_slow = run_mc(plans['card_low'], 'anywhere', v=v_low)
    geo_low_slow = coverage(plans['card_low'], v=v_low)
    # Planned-altitude sweep (3d_row, anywhere model)
    hsweep = {}
    for H in H_SWEEP:
        p = make_plan('3d_row', H=H)
        hsweep[H] = dict(geo=coverage(p), mc=run_mc(p, 'anywhere'))
    msweep = {}
    for m in MARGIN_SWEEP:
        p, q = make_plan('3d_row', margin=m), make_plan('3d_gimbal', margin=m)
        msweep[m] = dict(H=h_star(m), R=r_star(m), cov=coverage(p)['frac'],
                         cov_g=coverage(q)['frac'], z_g=q['strips'][0][1],
                         ga_noon=glare_aware_altitude(solar_position(NOON), margin=m))
    # Time of day (lat 35 N, equinox)
    tod = []
    for hr in TOD_HOURS:
        az, el = solar_position(hr)
        sun = (az, el)
        g = -spec_vec(sun_vec(*sun))
        ga = glare_aware_altitude(sun)
        tod.append(dict(h=hr, az=az, el=el, shadow=shadow_fraction(sun),
                        glare_row=bool(frame_glare(np.array([[40.0, plans['3d_row']['strips'][5][0],
                                                              plans['3d_row']['strips'][5][1]]]),
                                                   'nadir', sun)[0]),
                        glare_gimbal=bool(frame_glare(np.array([[40.0, plans['3d_gimbal']['strips'][5][0],
                                                                 plans['3d_gimbal']['strips'][5][1]]]),
                                                      'normal', sun)[0]),
                        glare_2drow=bool(frame_glare(np.array([[40.0, ROW_YC[5], ALT_2D]]), 'nadir', sun)[0]),
                        fov_nadir=glare_in_fov('nadir', g), fov_normal=glare_in_fov('normal', g),
                        ga=ga,
                        shadow_decl={dec: shadow_fraction(solar_position(hr, decl_deg=dec))
                                     if solar_position(hr, decl_deg=dec)[1] > 0 else 1.0
                                     for dec in DECL_SWEEP}))
    # Winter-solstice morning: shadows (altitude-independent) on the same seeds
    win_sun = solar_position(WINTER_HOUR, decl_deg=DECL_SWEEP[0])
    winter = {k: run_mc(plans[k], 'anywhere', sun=win_sun) for k in ('2d_card', '3d_row')}
    winter_shadow = shadow_fraction(win_sun)
    # Solar-noon glare study
    noon_sun = solar_position(NOON)
    ga_noon = glare_aware_altitude(noon_sun)
    noon_plans = dict(plans)
    noon_plans['3d_row_glare'] = make_plan('3d_row_glare', H=ga_noon[0], dy=ga_noon[1]) \
        if ga_noon is not None else None
    noon_keys = ['2d_card', '2d_row', '3d_row', '3d_row_glare', '3d_gimbal']
    noon = {}
    for rule in ('frame', 'point'):
        noon[rule] = {k: run_mc(noon_plans[k], 'anywhere', sun=noon_sun, glare_rule=rule)
                      for k in noon_keys if noon_plans[k] is not None}
    noon_geo = {k: coverage(noon_plans[k], sun=noon_sun) for k in noon_keys
                if noon_plans[k] is not None}
    noon_ga_decl = {dec: glare_aware_altitude(solar_position(NOON, decl_deg=dec))
                    for dec in DECL_SWEEP}
    noon_spot = []
    gnoon = -spec_vec(sun_vec(*noon_sun))
    for H in H_GRID:
        lo, hi = row_offset(H)
        D = np.array([[40.0, ROW_YC[5] + 0.5 * (lo + hi), H]])
        hit, ys, _, _ = glare_spot(D, gnoon)
        tg = H / (-gnoon[2])
        y_any = ys[0] if hit[0] else D[0, 1] + tg * gnoon[1]
        noon_spot.append((H, y_any - ROW_YC[5], bool(hit[0])))
    alt_band = {'card': [min_spec_angle_alt(H, card_sun) for H in H_SWEEP],
                'noon': [min_spec_angle_alt(H, noon_sun) for H in H_SWEEP]}
    plan_spot = None
    if ga_noon is not None:                      # where the reflection lands for the actual plan
        D = np.array([[40.0, ROW_YC[5] + ga_noon[1], ga_noon[0]]])
        hit, ys, _, _ = glare_spot(D, gnoon)
        y_any = ys[0] if hit[0] else D[0, 1] + ga_noon[0] / (-gnoon[2]) * gnoon[1]
        plan_spot = (ga_noon[0], float(y_any - ROW_YC[5]), bool(hit[0]))
    # card rule with the card's tilt-normal gimbal instead of the nadir camera
    r_low = (H_SCAN_CARD - H_EDGE / 2) / N_HAT[2]
    geo_low_gimbal = dict(coverage(make_plan('3d_gimbal', H=r_low)), R=r_low)
    # Ablation: keep the 2D altitude (5 m) and only move the strips cross-track
    offs = np.round(np.arange(-3.0, 3.0 + 1e-9, 0.02), 3)
    D5 = np.stack([np.full(len(offs), 40.0), ROW_YC[5] + offs, np.full(len(offs), ALT_2D)], 1)
    free5 = ~glare_spot(D5, gnoon)[0]
    best5 = None
    for o in offs[free5]:
        c = coverage(make_plan('3d_row', H=ALT_2D, dy=float(o)), sun=noon_sun)['frac']
        if best5 is None or c > best5[1] + 1e-12:
            best5 = (float(o), c)
    off5 = dict(n_free=int(free5.sum()), n=len(offs), best=best5,
                mc=run_mc(make_plan('3d_row', H=ALT_2D, dy=best5[0]), 'anywhere', sun=noon_sun)
                if best5 is not None else None)
    return dict(plans=plans, card_alt=card_alt, nominal=nominal, case_nom=case_nom, geo=geo,
                mc=mc, v_low=v_low, mc_low_slow=mc_low_slow, geo_low_slow=geo_low_slow,
                hsweep=hsweep, msweep=msweep, tod=tod, noon_sun=noon_sun, ga_noon=ga_noon,
                noon_plans=noon_plans, noon=noon, noon_geo=noon_geo, noon_ga_decl=noon_ga_decl,
                noon_spot=noon_spot, alt_band=alt_band, off5=off5, win_sun=win_sun, winter=winter,
                plan_spot=plan_spot, geo_low_gimbal=geo_low_gimbal,
                winter_shadow=winter_shadow, runtime=time.time() - t0)


# ── Plots ─────────────────────────────────────────────────────────────────────
def _row_polys():
    polys = []
    for k in range(N_ROWS):
        yb, yt = ROW_YB[k], ROW_YT[k]
        polys.append([(0, yb, 0), (FARM_L, yb, 0), (FARM_L, yt, H_EDGE), (0, yt, H_EDGE)])
    return polys


def plot_farm_3d(res, out_dir):
    fig = plt.figure(figsize=(12, 7.5))
    ax = fig.add_subplot(111, projection='3d')
    ax.add_collection3d(Poly3DCollection(_row_polys(), facecolor='#3b5b92', edgecolor='k',
                                         alpha=0.55, lw=0.3))
    for c in range(1, N_COLS):
        for k in range(N_ROWS):
            ax.plot([c * PANEL_L] * 2, [ROW_YB[k], ROW_YT[k]], [0, H_EDGE], color='k', lw=0.2)
    # shadow on the ground at 08:00 (illustration of the cast-shadow model)
    az, el = solar_position(8.0)
    s = sun_vec(az, el)
    for k in range(N_ROWS):
        tip = np.array([0, ROW_YT[k], H_EDGE]) + H_EDGE / (-s[2]) * s
        poly = [(0, ROW_YT[k], 0), (FARM_L, ROW_YT[k], 0),
                (FARM_L + tip[0], tip[1], 0), (tip[0], tip[1], 0)]
        ax.add_collection3d(Poly3DCollection([poly], facecolor='0.25', alpha=0.35, edgecolor='none'))
    for k in ('2d_card', '3d_row', '3d_gimbal'):
        p = res['nominal'][k]['pos']
        ax.plot(p[:, 0], p[:, 1], p[:, 2], color=COLORS[k], lw=0.9 if k != '2d_card' else 0.6,
                label=LABELS[k])
    sv = -sun_vec(SUN_AZ, SUN_EL)
    o = np.array([62.0, 20.0, 4.0])
    ax.quiver(*o, *(sv * 8), color='gold', lw=2, arrow_length_ratio=0.2)
    ax.text(*(o + sv * 9), 'sun (card, 120°/35°)', color='darkgoldenrod', fontsize=8)
    P = res['case_nom']['P']
    ax.scatter(P[:, 0], P[:, 1], P[:, 2], color='magenta', s=25, marker='x', label='hot spots (card seeds)')
    ax.set_xlim(0, FARM_L)
    ax.set_ylim(0, 62)
    ax.set_zlim(0, 8)
    ax.set_box_aspect((80, 62, 18))
    ax.set_xlabel('x east (m)')
    ax.set_ylabel('y north (m)')
    ax.set_zlabel('z (m)')
    ax.view_init(elev=28, azim=-115)
    ax.set_title('Tilted solar farm (25°, 2 m row gaps), ground shadows at 08:00 (grey), '
                 'flight paths', fontsize=10)
    ax.legend(loc='upper left', fontsize=8)
    path = os.path.join(out_dir, 'farm_3d.png')
    plt.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_cross_section(res, out_dir):
    fig, ax = plt.subplots(figsize=(12, 5))
    rows = [4, 5, 6]
    for k in rows:
        ax.fill([ROW_YB[k], ROW_YT[k], ROW_YT[k]], [0, H_EDGE, 0], color='#3b5b92', alpha=0.25)
        ax.plot([ROW_YB[k], ROW_YT[k]], [0, H_EDGE], color='#3b5b92', lw=3)
    for key, ls in (('2d_row', '-'), ('3d_row', '-'), ('3d_gimbal', '--')):
        p = res['plans'][key]
        y, z = p['strips'][5]
        b, e1, e2 = camera_axes(p['mode'])
        for sgn in (-1, 1):
            ray = b + sgn * TAN_H * e2
            # intersect with row 5 plane or ground
            t = ((np.array([0, ROW_YB[5], 0]) - np.array([0, y, z])) @ N_HAT) / (ray @ N_HAT)
            yy = y + t * ray[1]
            if yy < ROW_YB[5] or yy > ROW_YT[5]:
                t = z / -ray[2]
            ax.plot([y, y + t * ray[1]], [z, z + t * ray[2]], ls, color=COLORS[key], lw=1.2)
        ax.plot(y, z, 'o', color=COLORS[key], ms=9, label=f'{LABELS[key]} (y-off {y - ROW_YC[5]:+.2f} m, z {z:.2f} m)')
        cov = res['geo'][key]['cov'][5]
        s = res['geo'][key]['s']
        unc = s[~cov]
        if len(unc):
            ax.plot(ROW_YB[5] + unc * np.cos(ALPHA), unc * np.sin(ALPHA) - 0.15,
                    '-', color=COLORS[key], lw=4, label=f'not imaged by {LABELS[key]}')
    ax.axhline(H_EDGE + D_SAFE, color='k', ls=':', lw=0.8, label='h_edge + 1 m (card rule)')
    ax.set_xlim(ROW_YB[4] - 1, ROW_YT[6] + 1)
    ax.set_ylim(-0.4, 7.2)
    ax.set_aspect('equal')
    ax.set_xlabel('y north (m)')
    ax.set_ylabel('z (m)')
    ax.set_title('Cross-section of rows 5-7: FOV edge rays on row 6 (thick segment under the row: '
                 'panel surface not imaged)', fontsize=10)
    ax.legend(fontsize=8, loc='upper left')
    path = os.path.join(out_dir, 'cross_section.png')
    plt.savefig(path, dpi=140, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_altitude_profile(res, out_dir):
    fig, ax = plt.subplots(figsize=(12, 4.5))
    for k in ('2d_card', 'card_low', '3d_row', '3d_gimbal'):
        p = res['nominal'][k]['pos']
        sdist = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(p, axis=0), axis=1))])
        ax.plot(sdist, p[:, 2], color=COLORS[k], lw=0.9, label=LABELS[k])
        if k == 'card_low':
            ax.plot(sdist, surface_z(p[:, 1]) + D_SAFE, color='k', lw=0.4, alpha=0.6,
                    label='panel surface below + 1 m')
    ax.axhline(H_SCAN_CARD, color='green', ls='--', lw=1, label='glare-free lower bound (card rule)')
    ax.axhline(H_TRANSIT, color='tab:purple', ls=':', lw=1, label='h_transit')
    ax.set_xlabel('cumulative 3D path distance (m)')
    ax.set_ylabel('z (m)')
    ax.set_ylim(0, 7.5)
    ax.set_title('Altitude profiles (card sun, nominal mission)')
    ax.legend(fontsize=8, ncol=3, loc='upper right')
    path = os.path.join(out_dir, 'altitude_profile.png')
    plt.savefig(path, dpi=140, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_glare_band(res, out_dir):
    fig, axes = plt.subplots(1, 2, figsize=(13, 4.8))
    ax = axes[0]
    ax.plot(H_SWEEP, res['alt_band']['card'], 'o-', color='tab:red', label='card sun 120°/35° (09:00)')
    ax.plot(H_SWEEP, res['alt_band']['noon'], 's-', color='tab:orange',
            label=f'solar noon {res["noon_sun"][0]:.0f}°/{res["noon_sun"][1]:.0f}°')
    ax.axhline(GAMMA_GLARE_DEG, color='k', ls='--', label='glare half-angle 5°')
    ax.set_xlabel('altitude of the 3d_row nadir camera (m)')
    ax.set_ylabel('min angle (view, specular) over FOV (°)')
    ax.set_title('Specular separation vs altitude (nadir camera)')
    ax.set_ylim(0, max(res['alt_band']['card']) * 1.2)
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)
    ax = axes[1]
    H = np.array([a for a, _, _ in res['noon_spot']])
    off = np.array([b for _, b, _ in res['noon_spot']])
    hit = np.array([c for _, _, c in res['noon_spot']])
    for k in (4, 5, 6):
        ax.axhspan(ROW_YB[k] - ROW_YC[5], ROW_YT[k] - ROW_YC[5], color='#3b5b92', alpha=0.15)
    ax.plot(H[hit], off[hit], '.', color='gold', ms=3, label='glare spot on a panel (frame lost)')
    ax.plot(H[~hit], off[~hit], '.', color='k', ms=3, label='glare spot in a row gap (frame clean)')
    ax.axvline(h_star(), color='tab:red', ls='--', label=f'H* = {h_star():.2f} m (coverage)')
    if res['plan_spot'] is not None:
        hp, op, hitp = res['plan_spot']
        ax.plot(hp, op, '*', color='tab:brown', ms=14, mec='k',
                label=f'glare-aware plan: H = {hp:.2f} m, offset {res["ga_noon"][1]:+.2f} m '
                      f'({"on a panel" if hitp else "in a gap"})')
    ax.set_xlabel('altitude H (m); dots: offset at the window midpoint')
    ax.set_ylabel('glare spot y relative to row centre (m)')
    ax.set_title('Solar noon: where the sun reflection lands (blue bands = rows)')
    ax.legend(fontsize=7, loc='lower left')
    path = os.path.join(out_dir, 'glare_band.png')
    plt.tight_layout()
    plt.savefig(path, dpi=140, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_time_of_day(res, out_dir):
    tod = res['tod']
    h = np.array([t['h'] for t in tod])
    fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    ax = axes[0]
    names = {DECL_SWEEP[0]: 'winter solstice', 0.0: 'equinox', DECL_SWEEP[2]: 'summer solstice'}
    for dec, c in zip(DECL_SWEEP, ('tab:blue', 'k', 'tab:red')):
        sh = np.array([t['shadow_decl'][dec] for t in tod])
        ax.plot(h, 100 * sh, '-o', color=c, ms=3, label=f'panel area in shadow, {names[dec]} (%)')
    ax.plot(h, [t['el'] for t in tod], color='gold', lw=1.5, label='sun elevation, equinox (°)')
    ax.axhline(10, color='green', ls='--', lw=1, label='10 % shadow')
    ax.axvline(9.0, color='tab:red', ls=':', lw=1, label='09:00 equinox ≈ card sun')
    ax.set_ylim(0, 102)
    ax.set_ylabel('%  /  °')
    ax.legend(fontsize=7, loc='upper right', ncol=2)
    ax.set_title(f'Time of day at {LAT_DEG:.0f}° N (solar time); 100 % = sun below horizon or behind the panels')
    ax = axes[1]
    for key, off, lab in (('glare_2drow', 0.0, '2d_row @5 m'), ('glare_row', 0.1, f'3d_row @H*={h_star():.2f} m'),
                          ('glare_gimbal', 0.2, '3d_gimbal')):
        g = np.array([t[key] for t in tod])
        ax.plot(h, g + off, 's', ms=4, label=f'{lab}: frames lose glare (1) / clean (0)')
    ga = np.array([t['ga'][0] if t['ga'] is not None else np.nan for t in tod])
    ax2 = ax.twinx()
    ax2.plot(h, ga, 'o-', color='tab:brown', ms=3, label='glare-aware 3d_row altitude')
    ax2.set_ylabel('glare-aware altitude (m)', color='tab:brown')
    ax2.set_ylim(0, H_MAX + 1)
    ax.set_yticks([0, 1])
    ax.set_ylim(-0.3, 1.5)
    ax.set_xlabel('solar time (h)')
    ax.legend(fontsize=7, loc='upper left')
    ax2.legend(fontsize=7, loc='upper right')
    path = os.path.join(out_dir, 'time_of_day.png')
    plt.tight_layout()
    plt.savefig(path, dpi=140, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_coverage_map(res, out_dir):
    fig, axes = plt.subplots(1, 2, figsize=(13, 5.5), sharey=True)
    show = MC_SEED0                       # first MC seed on which 2d_row misses a hot spot
    for k in range(N_MC):
        if res['mc']['anywhere']['2d_row'][k]['det'] < 1:
            show = MC_SEED0 + k
            break
    rng_case = draw_case(show, 'anywhere')
    for ax, key in zip(axes, ('2d_row', '3d_row')):
        g = res['geo'][key]
        s = g['s']
        for k in range(N_ROWS):
            ys = ROW_YB[k] + s * np.cos(ALPHA)
            ax.add_patch(patches.Rectangle((0, ROW_YB[k]), FARM_L, DEPTH, fc='lightgrey', ec='grey', lw=0.4))
            unc = ~g['cov'][k]
            if unc.any():
                for yy in ys[unc]:
                    ax.plot([0, FARM_L], [yy, yy], color='orange', lw=0.6, alpha=0.5)
        r = inspect(res['plans'][key], rng_case, show)
        P = rng_case['P']
        ax.plot(P[r['det'], 0], P[r['det'], 1], 'o', color='red', ms=7, label='hot spot detected')
        ax.plot(P[~r['det'], 0], P[~r['det'], 1], 'X', color='blue', ms=9, label='hot spot missed')
        ax.plot(r['pos'][:, 0], r['pos'][:, 1], 'k-', lw=0.4)
        ax.set_xlim(-1, FARM_L + 1)
        ax.set_ylim(-1, 61)
        ax.set_aspect('equal')
        ax.set_xlabel('x (m)')
        ax.set_title(f'{LABELS[key]}: panel surface imaged {100 * g["frac"]:.1f} %\n'
                     f'(orange = not imaged), seed {show}: {int(r["det"].sum())}/8 detected',
                     fontsize=9)
        ax.legend(fontsize=7, loc='upper center', bbox_to_anchor=(0.5, -0.12), ncol=2)
    axes[0].set_ylabel('y (m)')
    path = os.path.join(out_dir, 'coverage_map.png')
    plt.tight_layout()
    plt.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_plan_comparison(res, out_dir):
    fig, axes = plt.subplots(1, 3, figsize=(16, 4.8))
    x = np.arange(len(PLANS))
    for j, m in enumerate(('centre', 'anywhere')):
        mean = [100 * np.mean([r['det'] for r in res['mc'][m][k]]) for k in PLANS]
        sd = [100 * np.std([r['det'] for r in res['mc'][m][k]]) for k in PLANS]
        bars = axes[0].bar(x + (j - 0.5) * 0.38, mean, 0.38, yerr=sd, capsize=3,
                           color=[COLORS[k] for k in PLANS], alpha=0.9 if j == 0 else 0.45,
                           edgecolor='k', label='hot spot at panel centre' if j == 0 else 'anywhere on panel')
        for bb, mv in zip(bars, mean):
            axes[0].text(bb.get_x() + bb.get_width() / 2, mv + 2, f'{mv:.1f}', ha='center', fontsize=7)
    axes[0].set_ylim(0, 115)
    axes[0].set_ylabel('detection rate (%)')
    axes[0].set_title(f'Card sun, {N_MC} seeds (CRN)')
    axes[0].legend(fontsize=7, loc='upper left')
    cov = [100 * res['geo'][k]['frac'] for k in PLANS]
    axes[1].bar(x, cov, color=[COLORS[k] for k in PLANS], edgecolor='k')
    for i, c in enumerate(cov):
        axes[1].text(i, c + 1.5, f'{c:.1f}', ha='center', fontsize=8)
    axes[1].set_ylim(0, 110)
    axes[1].set_ylabel('panel surface imaged unblurred (%)')
    axes[1].set_title('Geometric coverage of the tilted panels')
    t = [res['mc']['centre'][k][0]['t'] for k in PLANS]
    axes[2].bar(x, t, color=[COLORS[k] for k in PLANS], edgecolor='k')
    for i, tv in enumerate(t):
        axes[2].text(i, tv + 40, f'{tv:.0f}', ha='center', fontsize=8)
    axes[2].set_ylabel('coverage time at 1.0 m/s (s)')
    axes[2].set_title('Mission time')
    for ax in axes:
        ax.set_xticks(x)
        ax.set_xticklabels(['2D card\n@5 m', '2D row\n@5 m', 'card rule\n2.87 m', '3D row\nplanned', '3D gimbal\nplanned'],
                           fontsize=8)
    path = os.path.join(out_dir, 'plan_comparison.png')
    plt.tight_layout()
    plt.savefig(path, dpi=140, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_altitude_sweep(res, out_dir):
    hs = res['hsweep']
    H = np.array(H_SWEEP)
    det = np.array([np.mean([r['det'] for r in hs[h]['mc']]) for h in H_SWEEP])
    sd = np.array([np.std([r['det'] for r in hs[h]['mc']]) for h in H_SWEEP])
    cov = np.array([hs[h]['geo']['frac'] for h in H_SWEEP])
    gsd = np.array([hs[h]['geo']['gsd_max'] for h in H_SWEEP])
    fig, ax = plt.subplots(figsize=(9, 5))
    ax.errorbar(H, 100 * det, yerr=100 * sd, fmt='o-', color='tab:red', capsize=3,
                label='detection rate, anywhere model (MC)')
    ax.plot(H, 100 * cov, 's--', color='k', label='panel surface imaged (geometry)')
    ax.axvline(ALT_2D, color='tab:olive', ls=':', label='2D fixed altitude 5 m')
    ax.axvline(h_star(), color='tab:red', ls='--', lw=1, label=f'planned H* = {h_star():.2f} m')
    ax.set_xlabel('3d_row altitude H (m)')
    ax.set_ylabel('%')
    ax.set_ylim(40, 105)
    ax.grid(alpha=0.3)
    ax2 = ax.twinx()
    ax2.plot(H, 100 * gsd, '^-', color='tab:blue', ms=5, label='coarsest GSD on the panel (cm/px)')
    ax2.set_ylabel('GSD (cm/px)', color='tab:blue')
    ax.legend(fontsize=8, loc='lower right')
    ax2.legend(fontsize=8, loc='center right')
    ax.set_title(f'Planned altitude of the row-aligned plan ({N_MC} seeds, card sun)')
    path = os.path.join(out_dir, 'altitude_sweep.png')
    plt.savefig(path, dpi=140, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def save_animation(res, out_dir):
    nom = res['nominal']['3d_row']
    case = res['case_nom']
    pos = nom['pos']
    n = len(pos)
    n_frames = int(np.ceil((n - 1) / ANIM_STEP)) + 1
    fig = plt.figure(figsize=(8, 5.5))
    ax = fig.add_subplot(111, projection='3d')
    polys = []
    for k in range(N_ROWS):
        for c in range(N_COLS):
            x0, x1 = c * PANEL_L, (c + 1) * PANEL_L
            polys.append([(x0, ROW_YB[k], 0), (x1, ROW_YB[k], 0), (x1, ROW_YT[k], H_EDGE),
                          (x0, ROW_YT[k], H_EDGE)])
    coll = Poly3DCollection(polys, facecolor='#3b5b92', edgecolor='k', lw=0.15, alpha=0.6)
    ax.add_collection3d(coll)
    trail, = ax.plot([], [], [], color='tab:red', lw=1)
    dot, = ax.plot([], [], [], 'o', color='tab:red', mec='k', ms=6)
    ax.set_xlim(0, FARM_L)
    ax.set_ylim(0, 60)
    ax.set_zlim(0, 8)
    ax.set_box_aspect((80, 60, 16))
    ax.view_init(elev=30, azim=-110)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_zlabel('z (m)')
    base = np.array([[0.23, 0.36, 0.57, 0.6]] * len(polys))

    def update(f):
        k = min(f * ANIM_STEP, n - 1)
        trail.set_data(pos[:k + 1, 0], pos[:k + 1, 1])
        trail.set_3d_properties(pos[:k + 1, 2])
        dot.set_data([pos[k, 0]], [pos[k, 1]])
        dot.set_3d_properties([pos[k, 2]])
        fc = base.copy()
        nd = 0
        for j in range(M_DEFECTIVE):
            if 0 <= nom['t_det'][j] <= k:
                fc[case['idx'][j]] = [0.85, 0.1, 0.1, 0.95]
                nd += 1
        coll.set_facecolor(fc)
        ax.set_title(f'3D row-aligned plan, z = {pos[k, 2]:.2f} m | t = {k * DT:5.1f} s | '
                     f'hot panels detected {nd}/{M_DEFECTIVE}', fontsize=9)
        return [trail, dot, coll]

    anim = animation.FuncAnimation(fig, update, frames=n_frames, interval=100, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    anim.save(path, writer='pillow', fps=10, dpi=70)
    plt.close(fig)
    print(f'Saved: {path} ({n_frames} frames)')


# ── Report ────────────────────────────────────────────────────────────────────
def _stats(rs, key='det'):
    d = np.array([r[key] for r in rs])
    return d.mean(), d.std(), int((d == 1).sum())


def report(res):
    P = res['plans']
    print('\n=== S073 3D Solar Panel Thermal Inspection — Results ===')
    print(f'  Geometry: pitch {PITCH:.1f} m, row depth {DEPTH:.1f} m, gap {G_ROW:.1f} m, slant width '
          f'{W_SLANT:.3f} m, top edge {H_EDGE:.3f} m, panel area {PANEL_AREA:.1f} m^2')
    s, r = sun_vec(SUN_AZ, SUN_EL), spec_vec(sun_vec(SUN_AZ, SUN_EL))
    print(f'  Card sun az {SUN_AZ:.0f} el {SUN_EL:.0f}: s = ({s[0]:.4f}, {s[1]:.4f}, {s[2]:.4f}), '
          f'r_spec = ({r[0]:.4f}, {r[1]:.4f}, {r[2]:.4f}), specular elevation '
          f'{np.rad2deg(np.arcsin(r[2])):.1f} deg, angle(r, n) = {np.rad2deg(np.arccos(r @ N_HAT)):.1f} deg')
    print(f'  Min angle between any FOV ray and the specular direction: nadir '
          f'{min_spec_angle("nadir", (SUN_AZ, SUN_EL)):.1f} deg, tilt-normal '
          f'{min_spec_angle("normal", (SUN_AZ, SUN_EL)):.1f} deg (glare if < {GAMMA_GLARE_DEG:.0f})')
    print(f'  Same over the panel points the 3d_row camera sees, by altitude {H_SWEEP}: card sun '
          + ', '.join(f'{a:.1f}' for a in res['alt_band']['card']) + ' deg; noon '
          + ', '.join(f'{a:.1f}' for a in res['alt_band']['noon']) + ' deg')
    ca = np.array(res['card_alt'])
    print(f'  Card rule (lowest glare-free z >= h_edge + 1 = {H_SCAN_CARD:.3f} m, 0.1 m steps): '
          f'h_scan = {ca.min():.3f} .. {ca.max():.3f} m over {len(ca)} strips; h_transit = {H_TRANSIT:.3f} m')
    tip = H_EDGE / np.tan(np.deg2rad(SUN_EL))
    sxy = s[:2] / np.linalg.norm(s[:2])
    print(f'  Card-sun shadow: horizontal length h_edge/tan(el) = {tip:.3f} m, northward reach '
          f'{tip * sxy[1]:.3f} m < gap {G_ROW:.1f} m; shadowed panel area {100 * shadow_fraction((SUN_AZ, SUN_EL)):.2f} %'
          f' = {PANEL_AREA * shadow_fraction((SUN_AZ, SUN_EL)):.1f} m^2')
    print(f'  Planned 3d_row: H* = {h_star():.3f} m, offset {P["3d_row"]["strips"][0][0] - ROW_YC[0]:+.3f} m '
          f'north of the row centre (margin {PLAN_MARGIN} m); 3d_gimbal: R* = {r_star():.3f} m, '
          f'z = {P["3d_gimbal"]["strips"][0][1]:.3f} m, offset {P["3d_gimbal"]["strips"][0][0] - ROW_YC[0]:+.3f} m')
    print('  [Plans at the card sun]')
    print('    plan                                  strips  path(m)  time(s)  min clr(m)  imaged   GSD mean/max (cm)  v_max abeam (m/s)')
    for k in PLANS:
        g, nm = res['geo'][k], res['nominal'][k]
        print(f"    {LABELS[k]:<38} {len(P[k]['strips']):5d}  {nm['L']:7.1f}  {nm['t']:7.1f}  {nm['clr_min']:8.3f}"
              f"  {100 * g['frac']:6.2f}%   {100 * g['gsd_mean']:.2f} / {100 * g['gsd_max']:.2f}        {g['v_max']:.3f}")
    print('  [Nominal mission: card seeds (defects 42, noise 0), hot spots at panel centres]')
    print(f"    defective panels {sorted(res['case_nom']['idx'].tolist())}")
    for k in PLANS:
        nm = res['nominal'][k]
        print(f"    {LABELS[k]:<38} detected {int(nm['det'].sum())}/8, valid frames per hot spot "
              f"{nm['n_valid'].min()}..{nm['n_valid'].max()}, glare frames {100 * nm['glare_frames']:.1f}%, "
              f"shaded hot spots {nm['shade']}")
    for m in ('centre', 'anywhere'):
        print(f'  [Monte Carlo {N_MC} seeds, CRN, hot spot {m}, card sun, 1.0 m/s]')
        for k in PLANS:
            rs = res['mc'][m][k]
            mu, sd, n8 = _stats(rs)
            print(f"    {LABELS[k]:<38} detection {100 * mu:5.1f} ± {100 * sd:4.1f}%, seeds 8/8 {n8:2d}/{N_MC}, "
                  f"misses: never in view {sum(x['unseen'] for x in rs)}, all frames blurred "
                  f"{sum(x['blur'] for x in rs)}, glare {sum(x['glare'] for x in rs)}")
        for a, bkey in (('3d_row', '2d_row'), ('3d_row', '2d_card'), ('3d_gimbal', '3d_row'),
                        ('3d_gimbal', '2d_row')):
            da = np.array([x['det'] for x in res['mc'][m][a]])
            db = np.array([x['det'] for x in res['mc'][m][bkey]])
            print(f'    paired {a} vs {bkey}: {a} better {int((da > db).sum())}, worse {int((da < db).sum())}, '
                  f'ties {int((da == db).sum())}; mean difference {100 * (da - db).mean():+.1f} pts')
    t = {k: res['nominal'][k]['t'] for k in PLANS}
    print(f"  Time: 3d_row {t['3d_row']:.1f} s vs 2d_card {t['2d_card']:.1f} s: saves {t['2d_card'] - t['3d_row']:.1f} s "
          f"({100 * (1 - t['3d_row'] / t['2d_card']):.1f}%); vs 2d_row {t['2d_row']:.1f} s: "
          f"{t['3d_row'] - t['2d_row']:+.1f} s; 3d_gimbal {t['3d_gimbal']:.1f} s; card_low {t['card_low']:.1f} s "
          f"({t['card_low'] / t['2d_card']:.2f} x 2d_card)")
    gg = res['geo_low_gimbal']
    print(f"  Card rule with the tilt-normal gimbal at z = {H_SCAN_CARD:.3f} m (stand-off R = {gg['R']:.3f} m): "
          f"imaged unblurred {100 * gg['frac']:.2f}%, v_max abeam {gg['v_max']:.3f} m/s, blur abeam at 1 m/s "
          f"{V_NOM * T_EXP / (gg['R'] * IFOV):.2f} px (limit {BLUR_MAX})")
    if res['plan_spot'] is not None:
        print(f"  Glare-aware noon plan (H {res['plan_spot'][0]:.2f} m, offset {res['ga_noon'][1]:+.2f} m): reflection "
              f"{res['plan_spot'][1]:+.2f} m from the row centre, " + ('on a panel' if res['plan_spot'][2] else 'in a gap'))
    print(f'  Detection ceiling: the weakest hot spot reads ({DT_HOT_MIN} x {ATTEN:.4f} - {DT_THRESH}) / {T_NOISE_STD} = '
          f'{(DT_HOT_MIN * ATTEN - DT_THRESH) / T_NOISE_STD:.2f} sigma above the threshold in every frame, so an imaged, '
          f'unblurred hot spot is always detected')
    mu, sd, n8 = _stats(res['mc_low_slow'])
    print(f"  Card rule at its blur-limited speed 0.9 v_max = {res['v_low']:.3f} m/s: imaged "
          f"{100 * res['geo_low_slow']['frac']:.2f}%, detection (anywhere) {100 * mu:.1f} ± {100 * sd:.1f}%, "
          f"8/8 {n8}/{N_MC}, time {res['mc_low_slow'][0]['t']:.1f} s ({res['mc_low_slow'][0]['t'] / t['3d_row']:.1f} x 3d_row)")
    print('  [Planned-altitude sweep, 3d_row, anywhere model]')
    print('     H (m)   imaged    GSD max (cm)  detection mean ± std   8/8   never in view   all blurred')
    for H in H_SWEEP:
        hs = res['hsweep'][H]
        mu, sd, n8 = _stats(hs['mc'])
        print(f"    {H:5.2f}   {100 * hs['geo']['frac']:6.2f}%   {100 * hs['geo']['gsd_max']:6.2f}        "
              f"{100 * mu:5.1f} ± {100 * sd:4.1f}%      {n8:2d}/{N_MC}   {sum(x['unseen'] for x in hs['mc']):4d}"
              f"            {sum(x['blur'] for x in hs['mc']):4d}")
    gmax = max(res['hsweep'][H]['geo']['gsd_max'] for H in H_SWEEP)
    pf = (np.sqrt(2 * np.pi) * SIGMA_HOT / gmax * math.erf(gmax / (2 * np.sqrt(2) * SIGMA_HOT))) ** 2
    print(f'  Pixel-integration check (not modelled): a {100 * gmax:.2f} cm pixel centred on a sigma = '
          f'{SIGMA_HOT} m hot spot reads {100 * pf:.2f}% of its peak (loss {100 * (1 - pf):.2f}%)')
    print('  [Planning-margin sweep]')
    for m in MARGIN_SWEEP:
        ms = res['msweep'][m]
        print(f"    margin {m:.1f} m: H* = {ms['H']:.3f} m (imaged {100 * ms['cov']:.2f}%), "
              f"R* = {ms['R']:.3f} m, gimbal z = {ms['z_g']:.3f} m (imaged {100 * ms['cov_g']:.2f}%), "
              + (f"noon glare-aware H {ms['ga_noon'][0]:.2f} m" if ms['ga_noon'] is not None else 'no noon glare-aware H'))
    print(f'  [Time of day, lat {LAT_DEG:.0f} N, decl {DECL_DEG:.0f} deg]')
    print('    hour   az     el    shadow   specular in FOV (nadir/normal)   glare 2d_row/3d_row/gimbal   glare-aware H')
    for td in res['tod']:
        ga = f"{td['ga'][0]:.2f} m (off {td['ga'][1]:+.2f})" if td['ga'] is not None else 'none <= 15 m'
        print(f"    {td['h']:5.2f}  {td['az']:5.1f}  {td['el']:4.1f}  {100 * td['shadow']:6.2f}%   "
              f"{int(td['fov_nadir'])}/{int(td['fov_normal'])}                              "
              f"{int(td['glare_2drow'])}/{int(td['glare_row'])}/{int(td['glare_gimbal'])}                    {ga}")
    hrs = np.array([td['h'] for td in res['tod']])
    gl = np.array([td['glare_row'] for td in res['tod']])
    print(f'    3d_row at H* loses its frames to glare at {int(gl.sum())}/{len(gl)} times '
          f'({hrs[gl].min():.2f}..{hrs[gl].max():.2f} h); a glare-aware altitude <= {H_MAX:.0f} m exists at '
          f"{sum(td['ga'] is not None for td in res['tod'])}/{len(gl)} times")
    for dec in DECL_SWEEP:
        sh = np.array([td['shadow_decl'][dec] for td in res['tod']])
        ok = sh < 0.10
        print(f'    decl {dec:+6.2f}: shadowed panel area max {100 * sh.max():.2f}% '
              f'(at {hrs[np.argmax(sh)]:.2f} h), < 10% at {int(ok.sum())}/{len(ok)} times'
              + (f', from {hrs[ok].min():.2f} to {hrs[ok].max():.2f} h' if ok.any() else '')
              + '; values at ' + ', '.join(f'{h:.0f} h {100 * s:.1f}%' for h, s in zip(hrs, sh)
                                           if abs(h - round(h)) < 1e-9))
    ws = res['win_sun']
    print(f'  [Winter solstice {WINTER_HOUR:.0f}:00: sun az {ws[0]:.1f} el {ws[1]:.1f}, shadowed panel area '
          f"{100 * res['winter_shadow']:.2f}% = {PANEL_AREA * res['winter_shadow']:.1f} m^2, anywhere model]")
    for k, rs in res['winter'].items():
        mu, sd, n8 = _stats(rs)
        print(f"    {LABELS[k]:<38} detection {100 * mu:5.1f} ± {100 * sd:4.1f}%, 8/8 {n8:2d}/{N_MC}, "
              f"shaded hot spots {sum(x['shade'] for x in rs)}/{N_MC * M_DEFECTIVE}, "
              f"glare frames {100 * rs[0]['gf']:.1f}%")
    ns = res['noon_sun']
    print(f'  [Solar noon: sun az {ns[0]:.1f} el {ns[1]:.1f}, anywhere model, {N_MC} seeds]')
    print(f"    glare-aware 3d_row altitude {res['ga_noon'][0]:.2f} m, offset {res['ga_noon'][1]:+.2f} m"
          if res['ga_noon'] is not None else '    no glare-aware altitude <= 15 m')
    for dec, ga in res['noon_ga_decl'].items():
        az, el = solar_position(NOON, decl_deg=dec)
        print(f"    decl {dec:+6.2f}: noon el {el:.1f} deg, glare-aware H "
              + (f"{ga[0]:.2f} m (offset {ga[1]:+.2f} m)" if ga is not None else 'none <= 15 m'))
    for rule in ('frame', 'point'):
        print(f'    glare rule: {rule}-wise')
        for k, rs in res['noon'][rule].items():
            mu, sd, n8 = _stats(rs)
            g = res['noon_geo'][k]
            extra = (f"imaged glare-free {100 * g['frac']:6.2f}%, " if rule == 'frame' else '')
            print(f"      {LABELS[k]:<38} detection {100 * mu:5.1f} ± {100 * sd:4.1f}%, 8/8 {n8:2d}/{N_MC}, "
                  f"glare frames {100 * rs[0]['gf']:5.1f}%, {extra}time {rs[0]['t']:.1f} s, "
                  f"misses glare {sum(x['glare'] for x in rs)}")
    o5 = res['off5']
    if o5['best'] is not None:
        mu, sd, n8 = _stats(o5['mc'])
        print(f"    ablation, altitude fixed at {ALT_2D:.0f} m, offset only: {o5['n_free']}/{o5['n']} offsets in "
              f"[-3, 3] m are glare-free; best of them {o5['best'][0]:+.2f} m images {100 * o5['best'][1]:.2f}% "
              f"glare-free, detection {100 * mu:.1f} ± {100 * sd:.1f}%, 8/8 {n8}/{N_MC}")
    else:
        print(f"    ablation, altitude fixed at {ALT_2D:.0f} m, offset only: no glare-free offset in [-3, 3] m")
    if '3d_row_glare' in res['noon']['frame']:
        da = np.array([x['det'] for x in res['noon']['frame']['3d_row_glare']])
        for bkey in ('3d_row', '2d_row', '2d_card'):
            db = np.array([x['det'] for x in res['noon']['frame'][bkey]])
            print(f'    paired (frame rule) 3d_row_glare vs {bkey}: better {int((da > db).sum())}, worse '
                  f'{int((da < db).sum())}, ties {int((da == db).sum())}, mean diff {100 * (da - db).mean():+.1f} pts')
    print(f"  Simulation time {res['runtime']:.0f} s", file=sys.stderr)


if __name__ == '__main__':
    res = run_simulation()
    report(res)
    out = os.path.normpath(OUTPUT_DIR)
    os.makedirs(out, exist_ok=True)
    plot_farm_3d(res, out)
    plot_cross_section(res, out)
    plot_altitude_profile(res, out)
    plot_glare_band(res, out)
    plot_time_of_day(res, out)
    plot_coverage_map(res, out)
    plot_plan_comparison(res, out)
    plot_altitude_sweep(res, out)
    save_animation(res, out)
