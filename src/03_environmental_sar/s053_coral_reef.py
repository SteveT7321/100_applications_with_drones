"""
S053 Coral Reef 3D Reconstruction
=================================
A drone (or AUV) with a downward stereo camera (70 deg cone) photographs a 60 x 40 m
coral reef whose heightmap is a sum of 12 Gaussian bommies on a smooth substrate.
A reef surface cell counts as reconstructable when >= 2 images see it under the
three card conditions: incidence angle <= 45 deg, inside the camera cone, and a
clear line of sight (ray-heightmap occlusion test).

Three path families from the card are compared:
  1. Flat lawnmower at z = 4.0 m
  2. Altitude-adaptive lawnmower at 3.0 m standoff above the local reef height
  3. Rectangular spiral descent on three levels z = {5.0, 3.5, 2.0} m
Each family is evaluated (a) with the card's 60 % overlap spacing and no budget,
(b) truncated where the 500 m battery budget runs out, and (c) re-spaced so the
whole pattern fits in 500 m. A 10-reef Monte Carlo (same reef for every strategy)
gives mean +- std and win counts for (b) and (c).

Usage:
    conda activate drones
    python src/03_environmental_sar/s053_coral_reef.py
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.colors import ListedColormap
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401
from scipy.ndimage import map_coordinates, zoom

# ── Parameters (card) ─────────────────────────────────────────────────────────
REEF_LX      = 60.0     # m
REEF_LY      = 40.0     # m
GRID_SPACING = 0.5      # m
N_BOMMIES    = 12
THETA_MAX    = 45.0     # deg — max incidence angle
FOV_DEG      = 70.0     # deg — full cone angle
SENSOR_W_MM  = 6.3
FOCAL_MM     = 4.5
IMAGE_W_PX   = 4000
IMAGE_H_PX   = 3000
OVERLAP      = 0.60     # design image overlap
N_MIN_VIEWS  = 2
FLAT_Z       = 4.0      # m — flat lawnmower altitude
STANDOFF     = 3.0      # m — altitude-adaptive standoff
ALT_LEVELS   = [5.0, 3.5, 2.0]   # m — spiral levels
L_MAX        = 500.0    # m — path budget
CRUISE_SPEED = 1.5      # m/s
Z_MAX_REL    = 6.0      # m — max altitude above the reef crest
EPS_TARGET   = 0.008    # m — reconstruction error target
GSD_TARGET   = 0.005    # m/px
C_TARGET     = 0.90

# ── Parameters (added) ────────────────────────────────────────────────────────
ETA_CTRL     = 5        # control points per axis for the background term eta
REEF_MIN_H   = 0.05     # m — bommie height above substrate that counts as reef
RAY_STEP     = 0.25     # m — horizontal sampling step of the occlusion test
RAY_TOL      = 0.02     # m — tolerance of the occlusion test
MIN_CLEAR    = 0.5      # m — minimum clearance above the reef (asserted)
N_MC         = 10       # reef seeds
MAIN_SEED    = 42
ANIM_FRAMES  = 120

FOV_HALF = np.radians(FOV_DEG / 2)
COS_MAX = np.cos(np.radians(THETA_MAX))
STRATS = ['flat', 'adaptive', 'spiral', 'spiral1']
LABELS = {'flat': 'Flat lawnmower', 'adaptive': 'Altitude-adaptive', 'spiral': 'Spiral descent',
          'spiral1': 'Spiral, top level only'}
COLORS = {'flat': '#1F77B4', 'adaptive': '#FF7F0E', 'spiral': '#2CA02C', 'spiral1': '#9467BD'}
MODES = ['card', 'trunc', 'fit']
# cut at 500 m, the top-level-only spiral is identical to the full spiral (same first level)
MODE_STRATS = {'card': STRATS, 'trunc': ['flat', 'adaptive', 'spiral'], 'fit': STRATS}
MODE_LABELS = {'card': 'card spacing, no budget', 'trunc': 'card spacing, cut at 500 m',
               'fit': 're-spaced to fit 500 m'}

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), '..', '..',
                          'outputs', '03_environmental_sar', 's053_coral_reef')


# ── Reef model ────────────────────────────────────────────────────────────────

def generate_reef(seed):
    """Card heightmap: 12 Gaussian bommies, shifted by -0.5 max, plus eta, clipped."""
    rng = np.random.default_rng(seed)
    nx = int(REEF_LX / GRID_SPACING) + 1
    ny = int(REEF_LY / GRID_SPACING) + 1
    xs = np.linspace(0, REEF_LX, nx)
    ys = np.linspace(0, REEF_LY, ny)
    X, Y = np.meshgrid(xs, ys, indexing='ij')
    G = np.zeros((nx, ny))
    for _ in range(N_BOMMIES):
        cx, cy = rng.uniform(5, REEF_LX - 5), rng.uniform(5, REEF_LY - 5)
        amp = rng.uniform(0.3, 1.5)
        sx, sy = rng.uniform(1.5, 5.0, 2)
        G += amp * np.exp(-((X - cx) ** 2 / (2 * sx ** 2) + (Y - cy) ** 2 / (2 * sy ** 2)))
    ctrl = rng.uniform(-0.3, 0.0, (ETA_CTRL, ETA_CTRL))
    eta = zoom(ctrl, (nx / ETA_CTRL, ny / ETA_CTRL), order=3, mode='nearest')[:nx, :ny]
    eta = np.clip(eta, -0.3, 0.0)
    shift = -0.5 * G.max()
    H = np.clip(G + shift + eta, -3.0, 0.5)
    reef = G > REEF_MIN_H
    return X, Y, H, reef


def compute_normals(H):
    dhdx = np.gradient(H, GRID_SPACING, axis=0)
    dhdy = np.gradient(H, GRID_SPACING, axis=1)
    N = np.stack([-dhdx, -dhdy, np.ones_like(H)], axis=-1)
    return N / np.linalg.norm(N, axis=-1, keepdims=True)


def height_at(H, x, y):
    """Bilinear interpolation of the heightmap at metric coordinates."""
    return map_coordinates(H, [np.asarray(x) / GRID_SPACING, np.asarray(y) / GRID_SPACING],
                           order=1, mode='nearest')


class Reef:
    def __init__(self, seed):
        self.X, self.Y, self.H, self.reef = generate_reef(seed)
        self.N = compute_normals(self.H)
        self.h_max = float(self.H.max())
        self.pts = np.stack([self.X.ravel(), self.Y.ravel(), self.H.ravel()], axis=1)
        self.nrm = self.N.reshape(-1, 3)
        self.nx, self.ny = self.H.shape

    def visible(self, d):
        """Indices of grid cells visible from camera position d, and their incidence angles."""
        r_max = (d[2] - self.H.min()) * np.tan(FOV_HALF)
        i0 = max(0, int((d[0] - r_max) / GRID_SPACING)); i1 = min(self.nx, int((d[0] + r_max) / GRID_SPACING) + 2)
        j0 = max(0, int((d[1] - r_max) / GRID_SPACING)); j1 = min(self.ny, int((d[1] + r_max) / GRID_SPACING) + 2)
        ii, jj = np.meshgrid(np.arange(i0, i1), np.arange(j0, j1), indexing='ij')
        idx = (ii * self.ny + jj).ravel()
        p = self.pts[idx]
        v = d[None, :] - p
        dist = np.linalg.norm(v, axis=1)
        z_rel = d[2] - p[:, 2]
        lat = np.hypot(v[:, 0], v[:, 1])
        cos_t = np.sum(self.nrm[idx] * v, axis=1) / dist
        ok = (cos_t >= COS_MAX) & (lat <= z_rel * np.tan(FOV_HALF)) & (z_rel > 0.1)
        idx, p, v, lat, cos_t = idx[ok], p[ok], v[ok], lat[ok], cos_t[ok]
        # occlusion: sample the ray at horizontal steps from GRID_SPACING to lat
        n_s = int(np.ceil(r_max / RAY_STEP))
        s = GRID_SPACING + RAY_STEP * np.arange(n_s)                 # (S,)
        t = s[None, :] / np.maximum(lat[:, None], 1e-9)               # (N, S)
        valid = t < 1.0
        xr = p[:, 0:1] + t * v[:, 0:1]
        yr = p[:, 1:2] + t * v[:, 1:2]
        zr = p[:, 2:3] + t * v[:, 2:3]
        hr = height_at(self.H, xr.ravel(), yr.ravel()).reshape(xr.shape)
        blocked = np.any(valid & (hr > zr - RAY_TOL), axis=1)
        return idx[~blocked], np.degrees(np.arccos(np.clip(cos_t[~blocked], -1, 1)))


# ── Path generation ───────────────────────────────────────────────────────────

def lat_spacing(z_rel):
    return 2 * z_rel * np.tan(FOV_HALF) * (1 - OVERLAP)


def lawnmower_xy(n_strips):
    xs = np.linspace(0, REEF_LX, n_strips)
    wps = []
    for k, x in enumerate(xs):
        wps += [(x, 0.0), (x, REEF_LY)] if k % 2 == 0 else [(x, REEF_LY), (x, 0.0)]
    return np.array(wps)


def spiral_xy(step, inward=True):
    """Rectangular spiral over the reef box with ring spacing `step`."""
    pts = []
    k = 0
    while True:
        a = k * step
        x0, x1, y0, y1 = a, REEF_LX - a, a, REEF_LY - a
        if x1 - x0 < 0 or y1 - y0 < 0:
            break
        pts += [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
        if y1 - y0 <= step:
            break
        pts.append((x0, min(y0 + step, y1)))
        k += 1
    pts = np.array(pts)
    return pts if inward else pts[::-1]


def densify(wps_xy, z_of, ds_cam, ds_path=0.1):
    """Sample a 2D polyline densely (ds_path) for the flight path and every ds_cam for cameras.

    Returns the 3D path, the 3D camera positions and, for each camera, the index of the
    path sample at the same place (cameras lie on the path polyline).
    """
    path, cams, cam_idx = [wps_xy[0]], [wps_xy[0]], [0]
    base = 0
    for a, b in zip(wps_xy[:-1], wps_xy[1:]):
        L = np.linalg.norm(b - a)
        n_p = max(1, int(np.ceil(L / ds_path)))
        n_c = max(1, int(np.ceil(L / ds_cam)))
        path.extend(a + (b - a) * i / n_p for i in range(1, n_p + 1))
        for i in range(1, n_c + 1):
            cams.append(a + (b - a) * i / n_c)
            cam_idx.append(base + int(round(i / n_c * n_p)))
        base += n_p
    path, cams = np.array(path), np.array(cams)
    path = np.column_stack([path, z_of(path[:, 0], path[:, 1])])
    cams = np.column_stack([cams, z_of(cams[:, 0], cams[:, 1])])
    return path, cams, np.array(cam_idx)


def build_path(strategy, reef, scale=1.0, n_strips=None):
    """3D flight path (dense, 0.1 m) and camera positions for one strategy.

    scale multiplies the lateral spacing (1 = card 60 % overlap); for the lawnmowers
    n_strips overrides the strip count. Image spacing along track always uses the
    card overlap (Delta_lon = Delta_lat at scale 1).
    """
    h_max = reef.h_max
    segs = []          # (path, cams, cam_idx-in-this-path)
    if strategy in ('flat', 'adaptive'):
        z_rel = FLAT_Z - h_max if strategy == 'flat' else STANDOFF
        d_lat = lat_spacing(z_rel)
        n = n_strips if n_strips else int(np.ceil(REEF_LX / (d_lat * scale))) + 1
        xy = lawnmower_xy(n)
        if strategy == 'flat':
            zf = lambda x, y: np.full(np.shape(x), FLAT_Z)
        else:
            zf = lambda x, y: height_at(reef.H, x, y) + STANDOFF
        segs.append(densify(xy, zf, d_lat))
    else:
        n = None
        prev_end = None
        levels = ALT_LEVELS if strategy == 'spiral' else ALT_LEVELS[:1]
        for lvl, z in enumerate(levels):
            d_lat = lat_spacing(z - h_max)
            xy = spiral_xy(d_lat * scale, inward=(lvl % 2 == 0))
            zf = lambda x, y, z=z: np.full(np.shape(x), z)
            seg = densify(xy, zf, d_lat)
            if prev_end is not None:
                # straight transition from the previous level end to this level start (no images)
                p0 = seg[0][0]
                m = max(2, int(np.ceil(np.linalg.norm(p0 - prev_end) / 0.1)) + 1)
                tr = prev_end[None, :] + np.linspace(0, 1, m)[1:-1, None] * (p0 - prev_end)[None, :]
                segs.append((tr, np.empty((0, 3)), np.empty(0, dtype=int)))
            segs.append(seg)
            prev_end = seg[0][-1]
    path = np.vstack([sg[0] for sg in segs])
    cum = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))])
    cams, cam_d = [], []
    offset = 0
    for pth, c, ci in segs:
        if len(c):
            cams.append(c)
            cam_d.append(cum[offset + ci])
        offset += len(pth)
    cams = np.vstack(cams)
    cam_d = np.concatenate(cam_d)
    assert np.all(np.diff(cam_d) >= -1e-9), 'cameras out of order'
    step = np.linalg.norm(np.diff(path, axis=0), axis=1)
    assert step.max() < 0.5, 'path discontinuity (teleport)'
    clearance = path[:, 2] - height_at(reef.H, path[:, 0], path[:, 1])
    assert clearance.min() >= MIN_CLEAR, f'{strategy}: collision risk ({clearance.min():.2f} m)'
    assert path[:, 2].max() <= reef.h_max + Z_MAX_REL + 1e-9, 'above max operating altitude'
    return {'path': path, 'cum': cum, 'length': float(cum[-1]), 'cams': cams, 'cam_d': cam_d,
            'n_strips': n, 'scale': scale, 'clear_min': float(clearance.min())}


def fit_budget(strategy, reef):
    """Re-space a pattern so its full length is <= L_MAX (largest pattern that fits)."""
    if strategy in ('flat', 'adaptive'):
        best = None
        for n in range(2, 60):
            pl = build_path(strategy, reef, n_strips=n)
            if pl['length'] <= L_MAX + 1e-6:
                best = pl
            else:
                break
        return best
    # bisection on the common spacing scale (length falls as the scale grows)
    lo, hi = 1.0, 64.0
    best = build_path(strategy, reef, scale=hi)
    assert best['length'] <= L_MAX + 1e-6
    while hi - lo > 0.01:
        mid = 0.5 * (lo + hi)
        pl = build_path(strategy, reef, scale=mid)
        if pl['length'] <= L_MAX + 1e-6:
            hi, best = mid, pl
        else:
            lo = mid
    return best


# ── Evaluation ────────────────────────────────────────────────────────────────

def evaluate(reef, plan, d_max=np.inf, keep_angles=False, curve=False):
    n_cells = reef.pts.shape[0]
    views = np.zeros(n_cells, dtype=np.int32)
    best_zrel = np.full(n_cells, np.inf)
    angles = []
    reef_flat = reef.reef.ravel()
    n_surf = reef_flat.sum()
    curve_L, curve_C = [], []
    n_used = 0
    for cam, dcam in zip(plan['cams'], plan['cam_d']):
        if dcam > d_max + 1e-9:
            break
        idx, ang = reef.visible(cam)
        views[idx] += 1
        zr = cam[2] - reef.pts[idx, 2]
        best_zrel[idx] = np.minimum(best_zrel[idx], zr)
        n_used += 1
        if keep_angles:
            angles.append(ang)
        if curve:
            curve_L.append(dcam)
            curve_C.append(np.sum((views >= N_MIN_VIEWS) & reef_flat) / n_surf)
    cov_mask = (views >= N_MIN_VIEWS) & reef_flat
    seen = (views >= 1) & reef_flat
    gsd = best_zrel * SENSOR_W_MM / (FOCAL_MM * IMAGE_W_PX)          # m/px
    eps = gsd / (OVERLAP * np.sqrt(np.maximum(views, 1)))
    out = {
        'coverage': float(cov_mask.sum() / n_surf),
        'cov_all': float(np.mean(views >= N_MIN_VIEWS)),
        'gsd_mean': float(gsd[seen].mean()) if seen.any() else np.nan,
        'eps_mean': float(eps[cov_mask].mean()) if cov_mask.any() else np.nan,
        'n_images': n_used,
        'length': float(min(plan['length'], d_max)),
        'views': views.reshape(reef.H.shape),
        'gsd_cells': gsd[seen],
    }
    if keep_angles:
        out['angles'] = np.concatenate(angles) if angles else np.empty(0)
    if curve:
        out['curve'] = (np.array(curve_L), np.array(curve_C))
    return out


def run_simulation():
    reef = Reef(MAIN_SEED)
    plans = {}
    res = {}
    for s in STRATS:
        plans[(s, 'card')] = build_path(s, reef)
        plans[(s, 'fit')] = fit_budget(s, reef)
        res[(s, 'card')] = evaluate(reef, plans[(s, 'card')], keep_angles=True, curve=True)
        res[(s, 'trunc')] = evaluate(reef, plans[(s, 'card')], d_max=L_MAX)
        res[(s, 'fit')] = evaluate(reef, plans[(s, 'fit')], keep_angles=True, curve=True)
    # Monte Carlo over reef layouts (common reef for every strategy)
    mc = {(s, m): [] for m in ('trunc', 'fit') for s in MODE_STRATS[m]}
    for seed in range(N_MC):
        rf = Reef(1000 + seed)
        for s in STRATS:
            if s in MODE_STRATS['trunc']:
                mc[(s, 'trunc')].append(evaluate(rf, build_path(s, rf), d_max=L_MAX)['coverage'])
            mc[(s, 'fit')].append(evaluate(rf, fit_budget(s, rf))['coverage'])
    mc = {k: np.array(v) for k, v in mc.items()}
    return reef, plans, res, mc


# ── Plotting ──────────────────────────────────────────────────────────────────

def _save(fig, out_dir, name):
    path = os.path.join(out_dir, name)
    fig.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_reef_3d(reef, plans, out_dir):
    fig = plt.figure(figsize=(14, 6.5))
    for k, mode in enumerate(['card', 'fit']):
        ax = fig.add_subplot(1, 2, k + 1, projection='3d', computed_zorder=False)
        ax.plot_surface(reef.X, reef.Y, reef.H, cmap='terrain', linewidth=0, alpha=0.85,
                        rstride=2, cstride=2, zorder=0)
        p = plans[('spiral', mode)]['path']
        lvl_cols = ['purple', 'orange', 'red']
        for z, c in zip(ALT_LEVELS, lvl_cols):
            m = np.isclose(p[:, 2], z)
            pts = np.where(m, p[:, 0], np.nan), np.where(m, p[:, 1], np.nan)
            ax.plot(pts[0], pts[1], np.where(m, p[:, 2], np.nan), color=c, lw=0.8,
                    label=f'z = {z} m', zorder=3)
        cams = plans[('spiral', mode)]['cams']
        th = np.linspace(0, 2 * np.pi, 24)
        for cam in cams[:: max(1, len(cams) // 8)]:
            zr = cam[2] - height_at(reef.H, cam[0:1], cam[1:2])[0]
            r = zr * np.tan(FOV_HALF)
            for a in th[::6]:
                ax.plot([cam[0], cam[0] + r * np.cos(a)], [cam[1], cam[1] + r * np.sin(a)],
                        [cam[2], cam[2] - zr], color='grey', lw=0.5, zorder=2)
        ax.set_xlabel('x (m)')
        ax.set_ylabel('y (m)')
        ax.set_zlabel('z (m)')
        ax.set_zlim(-2, 6)
        ax.set_box_aspect((60, 40, 14))
        ax.view_init(elev=30, azim=-60)
        ax.set_title(f"Spiral descent, {MODE_LABELS[mode]}\n"
                     f"path {plans[('spiral', mode)]['length']:.0f} m, {len(cams)} images", fontsize=10)
        ax.legend(fontsize=7, loc='upper left')
    fig.suptitle('S053 Reef heightmap and spiral-descent path (grey = sample camera cones)')
    _save(fig, out_dir, 'reef_path_3d.png')


def plot_coverage_maps(reef, res, out_dir):
    fig, axes = plt.subplots(2, len(STRATS), figsize=(21, 8.5))
    cmap = ListedColormap(['#BBBBBB', '#9ECAE1', '#4292C6', '#2171B5', '#08306B'])
    for r, mode in enumerate(['card', 'fit']):
        for c, s in enumerate(STRATS):
            ax = axes[r, c]
            v = res[(s, mode)]['views']
            ax.imshow(np.clip(v, 0, 4).T, origin='lower', extent=[0, REEF_LX, 0, REEF_LY],
                      cmap=cmap, vmin=-0.5, vmax=4.5)
            unc = reef.reef & (v < N_MIN_VIEWS)
            ax.imshow(np.ma.masked_where(~unc.T, unc.T), origin='lower',
                      extent=[0, REEF_LX, 0, REEF_LY], cmap=ListedColormap(['red']), alpha=0.85)
            ax.contour(reef.X, reef.Y, reef.reef.astype(float), levels=[0.5], colors='k',
                       linewidths=0.5)
            rr = res[(s, mode)]
            ax.set_title(f"{LABELS[s]}, {MODE_LABELS[mode]}\nC = {100 * rr['coverage']:.1f}%, "
                         f"L = {rr['length']:.0f} m", fontsize=8.5)
            ax.set_aspect('equal')
    for ax in axes[1]:
        ax.set_xlabel('x (m)')
    for ax in axes[:, 0]:
        ax.set_ylabel('y (m)')
    fig.suptitle('S053 Views per cell (grey 0 → dark blue >= 4); red = reef cell with < 2 views; '
                 'black line = reef outline', fontsize=11)
    plt.tight_layout()
    _save(fig, out_dir, 'coverage_maps.png')


def plot_strategy_bars(res, mc, out_dir):
    fig, axes = plt.subplots(1, 4, figsize=(18, 4.5))
    w = 0.27
    x = np.arange(len(STRATS))
    for k, mode in enumerate(MODES):
        cov = [100 * res[(s, mode)]['coverage'] for s in STRATS]
        b = axes[0].bar(x + (k - 1) * w, cov, w, label=MODE_LABELS[mode],
                        color=['#444444', '#999999', '#DDDDDD'][k], edgecolor='k')
        for bar in b:
            axes[0].text(bar.get_x() + bar.get_width() / 2, bar.get_height(), f'{bar.get_height():.0f}',
                         ha='center', va='bottom', fontsize=7)
        L = [res[(s, mode)]['length'] for s in STRATS]
        axes[3].bar(x + (k - 1) * w, L, w, color=['#444444', '#999999', '#DDDDDD'][k], edgecolor='k')
        g = [1000 * res[(s, mode)]['gsd_mean'] for s in STRATS]
        axes[1].bar(x + (k - 1) * w, g, w, color=['#444444', '#999999', '#DDDDDD'][k], edgecolor='k')
        e = [1000 * res[(s, mode)]['eps_mean'] for s in STRATS]
        axes[2].bar(x + (k - 1) * w, e, w, color=['#444444', '#999999', '#DDDDDD'][k], edgecolor='k')
    axes[0].axhline(100 * C_TARGET, color='red', ls='--', lw=1, label='90 % target')
    axes[0].set_ylabel('coverage C (%)')
    axes[0].legend(fontsize=7, loc="upper center", bbox_to_anchor=(0.5, -0.36), ncol=2)
    axes[0].set_ylim(0, 112)
    axes[1].axhline(1000 * GSD_TARGET, color='red', ls='--', lw=1)
    axes[1].set_ylabel('mean best GSD (mm/px)')
    axes[2].axhline(1000 * EPS_TARGET, color='red', ls='--', lw=1)
    axes[2].set_ylabel('mean error bound ε (mm)')
    axes[3].axhline(L_MAX, color='red', ls='--', lw=1)
    axes[3].set_ylabel('path length (m)')
    axes[3].set_yscale('log')
    titles = ['Coverage (>= 2 valid views)', 'GSD (target 5 mm/px dashed)',
              'Reconstruction error (target 8 mm)', 'Path length (budget 500 m)']
    for ax, t in zip(axes, titles):
        ax.set_xticks(x)
        ax.set_xticklabels([LABELS[s] for s in STRATS], fontsize=8, rotation=20, ha="right")
        ax.set_title(t, fontsize=10)
    fig.suptitle(f'S053 Strategy comparison (reef seed {MAIN_SEED})')
    plt.tight_layout()
    _save(fig, out_dir, 'strategy_comparison.png')


def plot_mc(mc, out_dir):
    fig, ax = plt.subplots(figsize=(8, 4.5))
    data, labels = [], []
    cols = []
    for mode in ('trunc', 'fit'):
        for s in MODE_STRATS[mode]:
            data.append(100 * mc[(s, mode)])
            cols.append(s)
            labels.append(f"{LABELS[s]}\n{'cut at 500 m' if mode == 'trunc' else 'fit 500 m'}")
    bp = ax.boxplot(data, tick_labels=labels, patch_artist=True)
    for patch, s in zip(bp['boxes'], cols):
        patch.set_facecolor(COLORS[s])
    ax.axhline(100 * C_TARGET, color='red', ls='--', lw=1, label='90 % target')
    ax.set_ylabel('coverage C (%)')
    ax.tick_params(axis='x', labelsize=7)
    ax.legend(fontsize=8)
    ax.set_title(f'S053 Coverage within the 500 m budget, {N_MC} random reefs')
    _save(fig, out_dir, 'mc_coverage.png')


def plot_gsd_hist(res, out_dir):
    fig, ax = plt.subplots(figsize=(8, 4.5))
    bins = np.linspace(0, 1.6, 50)
    for s in STRATS:
        ax.hist(1000 * res[(s, 'card')]['gsd_cells'], bins=bins, histtype='step', lw=1.5,
                color=COLORS[s], label=f'{LABELS[s]} (card spacing)')
    ax.axvline(5, color='red', ls='--', label='5 mm/px target')
    ax.axvline(10, color='k', ls=':', label='10 mm/px degraded')
    ax.set_xlim(0, 11)
    ax.set_xlabel('best GSD per reef cell (mm/px)')
    ax.set_ylabel('cells')
    ax.legend(fontsize=8)
    ax.set_title('S053 GSD distribution (all strategies far below the 5 mm/px target)')
    _save(fig, out_dir, 'gsd_histogram.png')


def plot_coverage_curve(res, out_dir):
    fig, ax = plt.subplots(figsize=(8, 4.8))
    for s in STRATS:
        L, C = res[(s, 'card')]['curve']
        ax.plot(L, 100 * C, color=COLORS[s], label=f'{LABELS[s]} (card spacing)')
        L, C = res[(s, 'fit')]['curve']
        ax.plot(L, 100 * C, color=COLORS[s], ls='--', label=f'{LABELS[s]} (fit 500 m)')
    ax.axvline(L_MAX, color='red', ls=':', label='500 m budget')
    ax.axhline(100 * C_TARGET, color='k', ls=':', lw=1, label='90 % target')
    ax.set_xscale('log')
    ax.set_xlim(10, None)
    ax.set_xlabel('cumulative path length (m, log scale)')
    ax.set_ylabel('coverage C (%)')
    ax.legend(fontsize=7, ncol=2)
    ax.grid(alpha=0.3)
    ax.set_title('S053 Coverage vs path length')
    _save(fig, out_dir, 'coverage_vs_length.png')


def plot_incidence(res, out_dir):
    fig = plt.figure(figsize=(6, 6))
    ax = fig.add_subplot(111, projection='polar')
    ang = res[('spiral', 'card')]['angles']
    bins = np.radians(np.arange(0, 91, 3))
    h, _ = np.histogram(np.radians(ang), bins=bins)
    ax.bar(bins[:-1], h, width=np.radians(3), align='edge', color=COLORS['spiral'], edgecolor='k')
    ax.plot([np.radians(THETA_MAX)] * 2, [0, h.max() * 1.05], 'r--', lw=1.5, label='45° limit')
    ax.set_thetamin(0)
    ax.set_thetamax(90)
    ax.set_title(f'S053 Incidence angles of all valid views\n(spiral, card spacing; '
                 f'max {ang.max():.1f}°, {len(ang):,} views)', fontsize=10)
    ax.legend(loc='upper right', fontsize=8)
    _save(fig, out_dir, 'incidence_polar.png')


def save_animation(reef, plan, label, out_dir):
    path = plan['path']
    cams, cam_d = plan['cams'], plan['cam_d']
    n = len(path)
    step = int(np.ceil((n - 1) / ANIM_FRAMES))
    idx = [min(i * step, n - 1) for i in range(int(np.ceil((n - 1) / step)) + 1)]
    views = np.zeros(reef.pts.shape[0], dtype=np.int32)
    state = {'k': 0}
    cmap = ListedColormap(['#BBBBBB', '#4292C6', '#2CA02C'])
    fig, ax = plt.subplots(figsize=(7, 5.2))
    ax.contourf(reef.X, reef.Y, reef.H, levels=20, cmap='Greys', alpha=0.4)
    im = ax.imshow(np.zeros(reef.H.shape).T, origin='lower', extent=[0, REEF_LX, 0, REEF_LY],
                   cmap=cmap, vmin=-0.5, vmax=2.5, alpha=0.75)
    ax.contour(reef.X, reef.Y, reef.reef.astype(float), levels=[0.5], colors='k', linewidths=0.5)
    trail, = ax.plot([], [], '-', color='red', lw=0.7)
    dot, = ax.plot([], [], 'o', color='red', ms=7, mec='k')
    ax.set_xlim(0, REEF_LX)
    ax.set_ylim(0, REEF_LY)
    ax.set_aspect('equal')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    n_surf = reef.reef.sum()

    def update(f):
        k = idx[f]
        d_now = plan['cum'][k]
        while state['k'] < len(cams) and cam_d[state['k']] <= d_now + 1e-9:
            vi, _ = reef.visible(cams[state['k']])
            views[vi] += 1
            state['k'] += 1
        v = views.reshape(reef.H.shape)
        im.set_data(np.clip(v, 0, 2).T)
        trail.set_data(path[:k + 1, 0], path[:k + 1, 1])
        dot.set_data([path[k, 0]], [path[k, 1]])
        cov = np.sum((v >= N_MIN_VIEWS) & reef.reef) / n_surf
        ax.set_title(f'S053 {label}: z = {path[k, 2]:.1f} m, path {d_now:.0f} m\n'
                     f'grey 0 / blue 1 / green 2+ views; reef coverage {100 * cov:.1f}%', fontsize=10)
        return im, trail, dot

    anim = animation.FuncAnimation(fig, update, frames=len(idx), blit=False)
    out = os.path.join(out_dir, 'animation.gif')
    anim.save(out, writer='pillow', fps=12, dpi=80)
    plt.close(fig)
    from PIL import Image
    with Image.open(out) as gif:
        n_file = gif.n_frames      # Pillow merges identical consecutive frames
    print(f'Saved: {out} ({len(idx)} frames rendered, {n_file} in the file, '
          f'every {step} path samples of 0.1 m)')


if __name__ == '__main__':
    reef, plans, res, mc = run_simulation()
    gsd25 = 2.5 * SENSOR_W_MM / (FOCAL_MM * IMAGE_W_PX)
    print('\n=== S053 Coral Reef 3D Reconstruction — Results ===')
    print(f'  Reef seed {MAIN_SEED}: h in [{reef.H.min():.2f}, {reef.h_max:.2f}] m, '
          f'{reef.reef.sum()} reef cells of {reef.reef.size} ({100 * reef.reef.mean():.1f}%)')
    slope = np.degrees(np.arccos(reef.N[..., 2]))
    print(f'  Max surface slope {slope.max():.1f} deg; cells steeper than 45 deg: '
          f'{int((slope > 45).sum())}')
    print(f'  GSD at z_rel = 2.5 m: {1000 * gsd25:.3f} mm/px (card: 4 mm); required overlap '
          f'g/(eps sqrt(2)) = {gsd25 / (EPS_TARGET * np.sqrt(2)):.3f} (card: 0.354)')
    for s in STRATS:
        for mode in ('card', 'fit'):
            pl = plans[(s, mode)]
            extra = (f"strips {pl['n_strips']}" if pl['n_strips'] else f"spacing scale {pl['scale']:.2f}")
            print(f"  plan {LABELS[s]:18s} {mode:4s}: length {pl['length']:.1f} m, "
                  f"{len(pl['cams'])} images, {extra}, min clearance {pl['clear_min']:.2f} m, "
                  f"time {pl['length'] / CRUISE_SPEED:.0f} s")
    print('  Spiral level spacing (card, 60 % overlap at the crest): ' + ', '.join(
        f'z={z}: {lat_spacing(z - reef.h_max):.2f} m' for z in ALT_LEVELS))
    print(f"  Lawnmower spacing (card): flat {lat_spacing(FLAT_Z - reef.h_max):.2f} m, "
          f"adaptive {lat_spacing(STANDOFF):.2f} m")
    for mode in MODES:
        print(f'  [{MODE_LABELS[mode]}]')
        for s in MODE_STRATS[mode]:
            r = res[(s, mode)]
            print(f"    {LABELS[s]:18s} C = {100 * r['coverage']:.1f}% (all cells {100 * r['cov_all']:.1f}%), "
                  f"L = {r['length']:.0f} m, images {r['n_images']}, "
                  f"mean GSD {1000 * r['gsd_mean']:.3f} mm/px, mean eps {1000 * r['eps_mean']:.3f} mm")
    for s in STRATS:
        L, C = res[(s, 'card')]['curve']
        hit = np.where(C >= C_TARGET)[0]
        print(f"  {LABELS[s]} (card spacing) reaches 90 % at "
              f"{'%.0f m' % L[hit[0]] if len(hit) else 'never'}")
    ang = res[('spiral', 'card')]['angles']
    print(f'  Incidence angles (spiral, card spacing): max {ang.max():.1f} deg, '
          f'mean {ang.mean():.1f} deg, {len(ang)} valid views')
    print(f'  Monte Carlo ({N_MC} reefs, same reef for every strategy): coverage within 500 m')
    for mode in ('trunc', 'fit'):
        sl = MODE_STRATS[mode]
        arr = np.array([mc[(s, mode)] for s in sl])
        wins = np.argmax(arr, axis=0)
        ties = np.sum(np.sort(arr, axis=0)[-1] - np.sort(arr, axis=0)[-2] < 1e-12)
        for i, s in enumerate(sl):
            print(f"    {MODE_LABELS[mode]:26s} {LABELS[s]:18s} {100 * arr[i].mean():.1f} +- "
                  f"{100 * arr[i].std():.1f}% (min {100 * arr[i].min():.1f}, max {100 * arr[i].max():.1f}), "
                  f"best in {int(np.sum(wins == i))}/{N_MC}")
        print(f'    ({MODE_LABELS[mode]}: ties for best {int(ties)})')
    fit_flat_over_spiral = np.sum(mc[('flat', 'fit')] > mc[('spiral', 'fit')])
    print(f"  Fit-500 m: flat beats spiral in {int(fit_flat_over_spiral)}/{N_MC} reefs; "
          f"flat beats adaptive in {int(np.sum(mc[('flat', 'fit')] > mc[('adaptive', 'fit')]))}/{N_MC}; "
          f"top-level spiral beats flat in "
          f"{int(np.sum(mc[('spiral1', 'fit')] > mc[('flat', 'fit')]))}/{N_MC}")
    print(f"  Fit-500 m reaching the 90 % target: " + ', '.join(
        f"{LABELS[s]} {int(np.sum(mc[(s, 'fit')] >= C_TARGET))}/{N_MC}" for s in STRATS))

    out = os.path.normpath(OUTPUT_DIR)
    os.makedirs(out, exist_ok=True)
    plot_reef_3d(reef, plans, out)
    plot_coverage_maps(reef, res, out)
    plot_strategy_bars(res, mc, out)
    plot_mc(mc, out)
    plot_gsd_hist(res, out)
    plot_coverage_curve(res, out)
    plot_incidence(res, out)
    save_animation(reef, plans[('spiral1', 'fit')], 'Top-level spiral (fit 500 m)', out)
