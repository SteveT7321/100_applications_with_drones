"""
S052 Glacier Melt Area Measurement
==================================
A survey drone photographs an 800 x 600 m glacier tongue with a nadir RGB+NIR camera
(25 mm lens, 17.3 x 13.0 mm sensor, 4608 x 3456 px) to build an orthomosaic and
measure the melt-pool area from NDWI. The flight is planned for 80 m above the
glacier centreline with 80 % forward and 60 % side overlap. The 600 m survey width
includes the lateral-moraine flanks, which rise up to 37 m above the centreline, so
a constant-altitude flight loses overlap there.

Four plans are compared on the same terrain and the same melt-pool layout:
  1. Unidirectional lawnmower (nominal flat-glacier spacing)
  2. Boustrophedon lawnmower (nominal spacing)
  3. Adaptive strip spacing (card): a coarse laser-altimeter pass gives a terrain
     profile; strip spacing and trigger interval shrink where the ground is higher
  4. Uniform worst-case spacing (added baseline): the tightest spacing everywhere

Melt area is classified only where the ground point is seen by >= 3 photos
(the SfM requirement). A 20-seed Monte Carlo varies the pool layout, the NDWI
noise and the altimeter noise (common random numbers across plans).

Usage:
    conda activate drones
    python src/03_environmental_sar/s052_glacier.py
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Rectangle
from matplotlib.colors import ListedColormap
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401

# ── Parameters (card) ─────────────────────────────────────────────────────────
ALTITUDE        = 80.0      # m — survey altitude above the glacier centreline
FOCAL_LENGTH    = 25.0      # mm
SENSOR_W        = 17.3      # mm (across-track)
SENSOR_H        = 13.0      # mm (along-track)
IMAGE_W         = 4608      # px
IMAGE_H         = 3456      # px
FORWARD_OVERLAP = 0.80
SIDE_OVERLAP    = 0.60
GLACIER_L       = 800.0     # m — along the strips (y)
GLACIER_W       = 600.0     # m — across the strips (x)
DRONE_SPEED     = 10.0      # m/s
DT              = 0.5       # s
NDWI_THRESHOLD  = 0.2
MIN_VIEWS       = 3         # photos per ground point for SfM
GSD_LIMIT_CM    = 5.0       # cm/px requirement
NDWI_ICE        = (-0.3, 0.05)
NDWI_MELT       = (0.4, 0.08)

# ── Parameters (added) ────────────────────────────────────────────────────────
Z_FLY           = ALTITUDE  # m — flight altitude above the centreline datum z = 0
H_FLANK         = 35.0      # m — lateral-moraine rise at x = 0 and x = 600 m
UND_AMP         = 2.0       # m — surface undulation amplitude
COV_RES         = 1.0       # m — coverage-count grid
NDWI_RES        = 0.5       # m — NDWI raster (1200 x 1600 px)
N_POOLS_C       = 8         # supraglacial pools in the central glacier
N_POOLS_M       = 4         # ice-marginal ponds on the flanks
COARSE_Y        = [600.0, 400.0, 200.0]   # m — coarse-pass cross-glacier transects (flown in this order)
COARSE_DX       = 5.0       # m — laser-altimeter sample spacing
ALT_NOISE       = 0.5       # m — altimeter noise (1 sigma)
DEM_MARGIN      = 2.0       # m — safety margin added to the coarse terrain profile
N_MC            = 20        # Monte Carlo seeds
MAIN_SEED       = 0
ANIM_FRAMES     = 150

PIXEL_PITCH = SENSOR_W / IMAGE_W            # mm
KW = SENSOR_W / FOCAL_LENGTH                # footprint width per metre of AGL
KL = SENSOR_H / FOCAL_LENGTH                # footprint length per metre of AGL

STRATS = ['uni', 'bous', 'adapt', 'worst']
LABELS = {'uni': 'Unidirectional', 'bous': 'Boustrophedon',
          'adapt': 'Adaptive spacing', 'worst': 'Uniform worst-case'}
COLORS = {'uni': '#888888', 'bous': '#1F77B4', 'adapt': '#2CA02C', 'worst': '#9467BD'}

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), '..', '..',
                          'outputs', '03_environmental_sar', 's052_glacier')


# ── Camera and terrain ────────────────────────────────────────────────────────

def compute_footprint(agl):
    """Footprint width (across), length (along) in m and GSD in m/px at a given AGL."""
    return agl * KW, agl * KL, agl * PIXEL_PITCH / FOCAL_LENGTH


def terrain(x, y):
    """Glacier surface elevation (m) relative to the centreline datum."""
    flank = H_FLANK * ((x - GLACIER_W / 2) / (GLACIER_W / 2)) ** 4
    und = UND_AMP * np.sin(2 * np.pi * y / 400.0) * np.cos(2 * np.pi * x / 300.0)
    return flank + und


# ── Flight plans ──────────────────────────────────────────────────────────────

def coarse_profile(rng):
    """Coarse pass: laser-altimeter transects across the glacier at Z_FLY.

    Returns sample x positions, the planning profile z_plan(x) (max over transects
    plus margin) and the coarse-pass path (list of 2D points).
    """
    xs = np.arange(0.0, GLACIER_W + 1e-9, COARSE_DX)
    meas = np.array([terrain(xs, y) + rng.normal(0.0, ALT_NOISE, xs.size) for y in COARSE_Y])
    z_plan = meas.max(axis=0) + DEM_MARGIN
    path = []
    for i, y in enumerate(COARSE_Y):
        seg = [(GLACIER_W, y), (0.0, y)] if i % 2 == 0 else [(0.0, y), (GLACIER_W, y)]
        path.extend(seg)
    return xs, z_plan, path


def band_max(xs, z_plan, lo, hi):
    m = (xs >= lo) & (xs <= hi)
    return float(z_plan[m].max()) if m.any() else float(z_plan.max())


def nominal_layout():
    """Card layout: flat-glacier spacing, strips at By/2 + i*By."""
    fp_w, fp_l, _ = compute_footprint(ALTITUDE)
    bx = fp_l * (1 - FORWARD_OVERLAP)
    by = fp_w * (1 - SIDE_OVERLAP)
    n = int(np.ceil(GLACIER_W / by))
    xs = by / 2 + by * np.arange(n)
    return xs, np.full(n, bx)


def worst_layout(z_top):
    agl = Z_FLY - z_top
    fp_w, fp_l, _ = compute_footprint(agl)
    bx = fp_l * (1 - FORWARD_OVERLAP)
    by = fp_w * (1 - SIDE_OVERLAP)
    n = int(np.ceil(GLACIER_W / by))
    return by / 2 + by * np.arange(n), np.full(n, bx)


def adaptive_layout(xs_c, z_plan):
    """Strip spacing and trigger interval from the local (coarse) terrain height."""
    by_flat = compute_footprint(ALTITUDE)[0] * (1 - SIDE_OVERLAP)

    def spacing(x_lo):
        z = band_max(xs_c, z_plan, x_lo, x_lo + by_flat)
        return compute_footprint(Z_FLY - z)[0] * (1 - SIDE_OVERLAP)

    strips = [spacing(0.0) / 2]
    while strips[-1] + spacing(strips[-1]) / 2 < GLACIER_W:
        strips.append(strips[-1] + spacing(strips[-1]))
    strips = np.array(strips)
    bxs = np.array([compute_footprint(Z_FLY - band_max(xs_c, z_plan, x - by_flat, x + by_flat))[1]
                    * (1 - FORWARD_OVERLAP) for x in strips])
    return strips, bxs


def build_plan(strips, bxs, pattern):
    """Waypoints (2D) and photo positions for a strip layout.

    pattern 'uni': every strip south->north, repositioning leg back to y = 0.
    pattern 'bous': strips alternate direction. Photos are taken only on strips,
    at along-strip distances 0, Bx, 2Bx, ... (exact distance trigger).
    """
    wps, photos, strip_of_photo = [], [], []
    for i, (x, bx) in enumerate(zip(strips, bxs)):
        up = pattern == 'uni' or i % 2 == 0
        y0, y1 = (0.0, GLACIER_L) if up else (GLACIER_L, 0.0)
        wps.extend([(x, y0), (x, y1)])
        d = np.arange(0.0, GLACIER_L + 1e-9, bx)
        ys = y0 + d if up else y0 - d
        photos.extend((x, y) for y in ys)
        strip_of_photo.extend([i] * len(ys))
    return np.array(wps), np.array(photos), np.array(strip_of_photo)


def path_length(wps):
    return float(np.sum(np.linalg.norm(np.diff(wps, axis=0), axis=1)))


def fly(wps, v=DRONE_SPEED, dt=DT):
    """Point-mass constant-speed flight through the waypoints (no overshoot)."""
    pos = [wps[0].astype(float)]
    p = wps[0].astype(float)
    for wp in wps[1:]:
        while True:
            d = wp - p
            dist = np.linalg.norm(d)
            if dist <= v * dt:
                p = wp.astype(float)
                pos.append(p.copy())
                break
            p = p + d / dist * v * dt
            pos.append(p.copy())
    pos = np.array(pos)
    steps = np.linalg.norm(np.diff(pos, axis=0), axis=1)
    assert np.all(steps <= v * dt + 1e-9), 'speed limit exceeded'
    return pos


# ── Coverage, overlap and NDWI ────────────────────────────────────────────────

def grid_centres(res):
    gx = np.arange(res / 2, GLACIER_W, res)
    gy = np.arange(res / 2, GLACIER_L, res)
    return gx, gy


def coverage_count(photos, res=COV_RES):
    """Number of photos whose pinhole footprint contains each ground cell.

    Ground point q is in photo k iff |dx| <= (Z - z_q) * Sw / 2f and
    |dy| <= (Z - z_q) * Sh / 2f (nadir pinhole camera, terrain-aware).
    """
    gx, gy = grid_centres(res)
    Z = terrain(gx[None, :], gy[:, None])          # (ny, nx)
    cnt = np.zeros(Z.shape, dtype=np.int32)
    hw_max = (Z_FLY - Z.min()) * KW / 2
    hl_max = (Z_FLY - Z.min()) * KL / 2
    for px, py in photos:
        i0 = max(0, int((px - hw_max) / res)); i1 = min(gx.size, int((px + hw_max) / res) + 2)
        j0 = max(0, int((py - hl_max) / res)); j1 = min(gy.size, int((py + hl_max) / res) + 2)
        if i0 >= i1 or j0 >= j1:
            continue
        agl = Z_FLY - Z[j0:j1, i0:i1]
        inside = ((np.abs(gx[i0:i1][None, :] - px) <= agl * KW / 2)
                  & (np.abs(gy[j0:j1][:, None] - py) <= agl * KL / 2))
        cnt[j0:j1, i0:i1] += inside
    return cnt


def overlap_maps(strips, bxs, res=COV_RES):
    """Local side and forward overlap at every cell from the true terrain height."""
    gx, gy = grid_centres(res)
    Z = terrain(gx[None, :], gy[:, None])
    fp_w = (Z_FLY - Z) * KW
    fp_l = (Z_FLY - Z) * KL
    k = np.clip(np.searchsorted(strips, gx) - 1, 0, len(strips) - 2)
    b_side = (strips[k + 1] - strips[k])[None, :]
    near = np.argmin(np.abs(gx[:, None] - strips[None, :]), axis=1)
    b_fwd = bxs[near][None, :]
    return 1 - b_side / fp_w, 1 - b_fwd / fp_l


def make_pools(rng):
    """Elliptical melt pools (non-overlapping): central supraglacial + marginal."""
    pools = []
    def place(n, xlo, xhi, alo, ahi):
        placed = 0
        while placed < n:
            a, b = rng.uniform(alo, ahi, 2)
            cx = rng.uniform(xlo, xhi)
            cy = rng.uniform(60.0, GLACIER_L - 60.0)
            ang = rng.uniform(0, np.pi)
            if all(np.hypot(cx - q[0], cy - q[1]) > max(a, b) + max(q[2], q[3]) + 5 for q in pools):
                pools.append((cx, cy, a, b, ang))
                placed += 1
    place(N_POOLS_C, 180.0, 420.0, 10.0, 35.0)
    for side in range(N_POOLS_M):
        lo, hi = (12.0, 30.0) if side % 2 == 0 else (GLACIER_W - 30.0, GLACIER_W - 12.0)
        place(1, lo, hi, 8.0, 12.0)
    return pools


def synthetic_ndwi(pools, rng):
    gx, gy = grid_centres(NDWI_RES)
    ndwi = rng.normal(*NDWI_ICE, size=(gy.size, gx.size))
    truth = np.zeros(ndwi.shape, dtype=bool)
    for cx, cy, a, b, ang in pools:
        r = max(a, b)
        i0, i1 = int((cx - r) / NDWI_RES), int((cx + r) / NDWI_RES) + 2
        j0, j1 = int((cy - r) / NDWI_RES), int((cy + r) / NDWI_RES) + 2
        i0, j0 = max(i0, 0), max(j0, 0)
        X, Y = np.meshgrid(gx[i0:i1] - cx, gy[j0:j1] - cy)
        u = X * np.cos(ang) + Y * np.sin(ang)
        w = -X * np.sin(ang) + Y * np.cos(ang)
        truth[j0:j1, i0:i1] |= (u / a) ** 2 + (w / b) ** 2 <= 1.0
    ndwi[truth] = rng.normal(*NDWI_MELT, size=int(truth.sum()))
    return ndwi, truth


def melt_area(ndwi, cov):
    """Melt area (m^2) from NDWI, counted only where the cell has >= MIN_VIEWS photos."""
    up = int(round(COV_RES / NDWI_RES))
    recon = np.repeat(np.repeat(cov >= MIN_VIEWS, up, axis=0), up, axis=1)
    mask = (ndwi > NDWI_THRESHOLD) & recon
    return float(mask.sum() * NDWI_RES ** 2), mask, recon


# ── Simulation ────────────────────────────────────────────────────────────────

def plan_all(rng_alt):
    xs_c, z_plan, coarse = coarse_profile(rng_alt)
    layouts = {'uni': nominal_layout(), 'bous': nominal_layout(),
               'adapt': adaptive_layout(xs_c, z_plan), 'worst': worst_layout(z_plan.max())}
    plans = {}
    for s in STRATS:
        strips, bxs = layouts[s]
        wps, photos, sop = build_plan(strips, bxs, 'uni' if s == 'uni' else 'bous')
        if s in ('adapt', 'worst'):
            full = np.vstack([np.array(coarse), wps])
            d_coarse = path_length(np.array(coarse)) + float(np.linalg.norm(wps[0] - coarse[-1]))
        else:
            full = wps
            d_coarse = 0.0
        plans[s] = {'strips': strips, 'bxs': bxs, 'wps': full, 'photos': photos,
                    'sop': sop, 'dist': path_length(full), 'd_coarse': d_coarse}
    return plans, (xs_c, z_plan)


def evaluate(plan, cov):
    o_s, o_f = overlap_maps(plan['strips'], plan['bxs'])
    gx, gy = grid_centres(COV_RES)
    Z = terrain(gx[None, :], gy[:, None])
    gsd = (Z_FLY - Z) * PIXEL_PITCH / FOCAL_LENGTH * 100.0
    return {
        'n_strips': len(plan['strips']), 'n_photos': len(plan['photos']),
        'dist': plan['dist'], 'time': plan['dist'] / DRONE_SPEED, 'd_coarse': plan['d_coarse'],
        'cov_min': int(cov.min()), 'cov_mean': float(cov.mean()),
        'frac3': float(np.mean(cov >= MIN_VIEWS)),
        'side_min': float(o_s.min()), 'fwd_min': float(o_f.min()),
        'compliant': float(np.mean((o_s >= SIDE_OVERLAP - 1e-9) & (o_f >= FORWARD_OVERLAP - 1e-9))),
        'severe': float(np.mean((o_s < SIDE_OVERLAP - 0.05) | (o_f < FORWARD_OVERLAP - 0.05))),
        'gsd_max': float(gsd.max()), 'gsd_min': float(gsd.min()),
    }


def run_simulation():
    plans, dem = plan_all(np.random.default_rng([MAIN_SEED, 1]))
    covs = {s: coverage_count(plans[s]['photos']) for s in STRATS}
    metrics = {s: evaluate(plans[s], covs[s]) for s in STRATS}

    pools = make_pools(np.random.default_rng([MAIN_SEED, 2]))
    ndwi, truth = synthetic_ndwi(pools, np.random.default_rng([MAIN_SEED, 3]))
    a_true = float(sum(np.pi * p[2] * p[3] for p in pools))
    for s in STRATS:
        a, mask, recon = melt_area(ndwi, covs[s])
        metrics[s]['a_est'] = a
        metrics[s]['a_err'] = (a - a_true) / a_true
        metrics[s]['a_lost'] = float((truth & ~recon).sum() * NDWI_RES ** 2)
    a_all = float((ndwi > NDWI_THRESHOLD).sum() * NDWI_RES ** 2)

    # Monte Carlo over pool layout, NDWI noise and altimeter noise (same draws for all plans)
    mc = {s: {'err': [], 'dist': [], 'frac3': [], 'lost': []} for s in STRATS}
    for seed in range(N_MC):
        pl, _ = plan_all(np.random.default_rng([seed, 1]))
        pp = make_pools(np.random.default_rng([seed, 2]))
        nd, tr = synthetic_ndwi(pp, np.random.default_rng([seed, 3]))
        at = float(sum(np.pi * p[2] * p[3] for p in pp))
        for s in STRATS:
            cv = covs[s] if s in ('uni', 'bous') else coverage_count(pl[s]['photos'])
            a, _, rc = melt_area(nd, cv)
            mc[s]['err'].append((a - at) / at)
            mc[s]['lost'].append(float((tr & ~rc).sum() * NDWI_RES ** 2))
            mc[s]['dist'].append(pl[s]['dist'])
            mc[s]['frac3'].append(float(np.mean(cv >= MIN_VIEWS)))
    for s in STRATS:
        for k in mc[s]:
            mc[s][k] = np.array(mc[s][k])

    traj = fly(plans['adapt']['wps'])
    gx, gy = grid_centres(COV_RES)
    agl_traj = Z_FLY - terrain(traj[:, 0], traj[:, 1])
    assert agl_traj.min() > 30.0, 'drone too close to terrain'
    return {'plans': plans, 'covs': covs, 'metrics': metrics, 'dem': dem, 'pools': pools,
            'ndwi': ndwi, 'truth': truth, 'a_true': a_true, 'a_all': a_all, 'mc': mc,
            'traj': traj, 'agl_traj_min': float(agl_traj.min())}


# ── Plotting ──────────────────────────────────────────────────────────────────

def _save(fig, out_dir, name):
    path = os.path.join(out_dir, name)
    fig.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_coverage_path(res, out_dir):
    fig, axes = plt.subplots(1, 2, figsize=(13, 8))
    gx, gy = grid_centres(4.0)
    Z = terrain(gx[None, :], gy[:, None])
    for ax, s in zip(axes, ['bous', 'adapt']):
        p = res['plans'][s]
        ax.imshow(Z, origin='lower', extent=[0, GLACIER_W, 0, GLACIER_L], cmap='Greys', alpha=0.45)
        ax.plot(p['wps'][:, 0], p['wps'][:, 1], '-', color='grey', lw=0.8, label='flight path')
        ax.plot(p['photos'][::3, 0], p['photos'][::3, 1], '.', color='tab:blue', ms=1.5,
                label='photo triggers (every 3rd)')
        agl = Z_FLY - terrain(p['photos'][:, 0], p['photos'][:, 1])
        for k in range(0, len(p['photos']), max(1, len(p['photos']) // 25)):
            px, py = p['photos'][k]
            w, l = agl[k] * KW, agl[k] * KL
            ax.add_patch(Rectangle((px - w / 2, py - l / 2), w, l, fill=False,
                                   ec='tab:orange', lw=0.7))
        for i in range(0, len(p['strips']), 4):
            ax.annotate(f'{i + 1}', (p['strips'][i], GLACIER_L + 8), ha='center', fontsize=7)
            ax.annotate('', xy=(p['strips'][i], 420 if i % 2 == 0 else 380),
                        xytext=(p['strips'][i], 380 if i % 2 == 0 else 420),
                        arrowprops=dict(arrowstyle='->', color='red', lw=1))
        if s == 'adapt':
            ax.plot(p['wps'][:6, 0], p['wps'][:6, 1], '-', color='magenta', lw=1.5,
                    label='coarse altimeter pass')
        ax.set_xlim(-30, GLACIER_W + 30)
        ax.set_ylim(-30, GLACIER_L + 30)
        ax.set_aspect('equal')
        ax.set_xlabel('x across glacier (m)')
        ax.set_title(f"{LABELS[s]}: {len(p['strips'])} strips, {len(p['photos'])} photos, "
                     f"{p['dist'] / 1000:.1f} km", fontsize=10)
        ax.legend(loc='lower center', fontsize=7)
    axes[0].set_ylabel('y along glacier (m)')
    fig.suptitle('S052 Coverage paths (orange = sample footprints, grey shading = terrain height, '
                 'strip numbers on top)', fontsize=11)
    _save(fig, out_dir, 'coverage_path.png')


def plot_coverage_heatmap(res, out_dir):
    fig, axes = plt.subplots(1, 4, figsize=(17, 6))
    for ax, s in zip(axes, STRATS):
        c = res['covs'][s]
        m = res['metrics'][s]
        im = ax.imshow(np.clip(c, 0, 15), origin='lower', extent=[0, GLACIER_W, 0, GLACIER_L],
                       cmap='RdYlBu', vmin=0, vmax=15)
        gx, gy = grid_centres(COV_RES)
        jj, ii = np.where(c < MIN_VIEWS)
        ax.plot(gx[ii], gy[jj], 's', color='magenta', ms=1.2, mew=0)
        ax.set_title(f"{LABELS[s]}\nmin {m['cov_min']}, mean {m['cov_mean']:.1f}, "
                     f">=3: {100 * m['frac3']:.2f}%", fontsize=10)
        ax.set_xlabel('x (m)')
    axes[0].set_ylabel('y (m)')
    fig.colorbar(im, ax=axes, shrink=0.8, label='photos per ground cell (clipped at 15)')
    fig.suptitle('S052 Coverage count (magenta = cells with fewer than 3 photos)', fontsize=12)
    _save(fig, out_dir, 'coverage_heatmap.png')


def plot_ndwi(res, out_dir):
    fig, ax = plt.subplots(figsize=(7, 8.5))
    nd = res['ndwi'][::2, ::2]
    im = ax.imshow(nd, origin='lower', extent=[0, GLACIER_W, 0, GLACIER_L], cmap='RdBu',
                   vmin=-0.6, vmax=0.6)
    fig.colorbar(im, ax=ax, shrink=0.8, label='NDWI')
    ax.set_title(f"S052 Synthetic NDWI mosaic ({NDWI_RES} m raster)\n"
                 f"blue = melt water (NDWI > {NDWI_THRESHOLD}), red = clean ice")
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    _save(fig, out_dir, 'ndwi_mosaic.png')


def plot_melt_classification(res, out_dir):
    fig, axes = plt.subplots(1, 2, figsize=(13, 8))
    for ax, s in zip(axes, ['bous', 'adapt']):
        a, mask, recon = melt_area(res['ndwi'], res['covs'][s])
        ax.imshow(res['ndwi'][::2, ::2], origin='lower', extent=[0, GLACIER_W, 0, GLACIER_L],
                  cmap='Greys_r', vmin=-0.6, vmax=0.6, alpha=0.6)
        ax.imshow(np.ma.masked_where(~mask[::2, ::2], mask[::2, ::2]), origin='lower',
                  extent=[0, GLACIER_W, 0, GLACIER_L], cmap=ListedColormap(['tab:blue']))
        missed = res['truth'] & ~recon
        ax.imshow(np.ma.masked_where(~missed[::2, ::2], missed[::2, ::2]), origin='lower',
                  extent=[0, GLACIER_W, 0, GLACIER_L], cmap=ListedColormap(['red']))
        gx, gy = grid_centres(COV_RES)
        jj, ii = np.where(res['covs'][s] < MIN_VIEWS)
        ax.plot(gx[ii], gy[jj], 's', color='magenta', ms=1.2, mew=0)
        m = res['metrics'][s]
        ax.set_title(f"{LABELS[s]}: A_melt = {a:,.0f} m² "
                     f"({100 * a / (GLACIER_L * GLACIER_W):.2f}% of glacier)\n"
                     f"truth {res['a_true']:,.0f} m², error {100 * m['a_err']:+.2f}%; "
                     f"melt in <3-photo zone {m['a_lost']:,.0f} m²", fontsize=10)
        ax.set_xlabel('x (m)')
    axes[0].set_ylabel('y (m)')
    fig.suptitle('S052 Melt classification (blue = classified melt, red = melt missed '
                 'because <3 photos, magenta = <3-photo cells)', fontsize=11)
    _save(fig, out_dir, 'melt_classification.png')


def plot_strategy_comparison(res, out_dir):
    m = res['metrics']
    mc = res['mc']
    fig, axes = plt.subplots(1, 4, figsize=(18, 4.8))
    names = [LABELS[s] for s in STRATS]
    cols = [COLORS[s] for s in STRATS]
    d = [m[s]['dist'] / 1000 for s in STRATS]
    b = axes[0].bar(names, d, color=cols, edgecolor='k')
    for bar, s in zip(b, STRATS):
        rat = m[s]['dist'] / m['uni']['dist']
        axes[0].text(bar.get_x() + bar.get_width() / 2, bar.get_height(),
                     f"{bar.get_height():.1f} km\n{m[s]['time'] / 60:.0f} min\n{rat:.2f} x uni",
                     ha='center', va='bottom', fontsize=8)
    axes[0].set_ylabel('flight distance (km)')
    axes[0].set_ylim(0, max(d) * 1.35)
    axes[0].set_title('Distance / time (ratio to unidirectional)')
    v = [100 * m[s]['compliant'] for s in STRATS]
    b = axes[1].bar(names, v, color=cols, edgecolor='k')
    for bar in b:
        axes[1].text(bar.get_x() + bar.get_width() / 2, bar.get_height(), f'{bar.get_height():.1f}%',
                     ha='center', va='bottom', fontsize=9)
    sev = [100 * m[s]['severe'] for s in STRATS]
    axes[1].bar(names, sev, color='red', alpha=0.5, width=0.4, label='area > 5 pp short of target')
    axes[1].legend(fontsize=8, loc='upper left')
    axes[1].set_ylim(0, 110)
    axes[1].set_title('Area meeting 80 % / 60 % overlap (%)')
    v = [100 * m[s]['frac3'] for s in STRATS]
    b = axes[2].bar(names, v, color=cols, edgecolor='k')
    for bar in b:
        axes[2].text(bar.get_x() + bar.get_width() / 2, bar.get_height(), f'{bar.get_height():.2f}%',
                     ha='center', va='bottom', fontsize=9)
    axes[2].set_ylim(95, 100.6)
    axes[2].set_title('Area seen by >= 3 photos')
    errs = [100 * mc[s]['err'] for s in STRATS]
    axes[3].boxplot(errs, tick_labels=names)
    axes[3].axhline(0, color='k', lw=0.8)
    axes[3].set_ylabel('melt-area error (%)')
    axes[3].set_title(f'Melt-area error, {N_MC} seeds')
    for ax in axes:
        ax.tick_params(axis='x', labelsize=8, rotation=15)
    fig.suptitle('S052 Strategy comparison', fontsize=12)
    plt.tight_layout()
    _save(fig, out_dir, 'strategy_comparison.png')


def plot_gsd_altitude(res, out_dir):
    h = np.linspace(40, 160, 200)
    gsd = h * PIXEL_PITCH / FOCAL_LENGTH * 100
    h_lim = GSD_LIMIT_CM / 100 * FOCAL_LENGTH / PIXEL_PITCH
    fig, ax = plt.subplots(figsize=(7, 4.5))
    ax.plot(h, gsd, color='tab:blue', label='GSD = h p / f')
    ax.axhline(GSD_LIMIT_CM, color='red', ls='--', label=f'{GSD_LIMIT_CM:.0f} cm limit (reached at h = {h_lim:.0f} m)')
    g80 = ALTITUDE * PIXEL_PITCH / FOCAL_LENGTH * 100
    ax.plot(ALTITUDE, g80, 'o', color='tab:green', ms=9, label=f'operating point h = 80 m: {g80:.2f} cm')
    agl_min = Z_FLY - terrain(np.array([0.0]), np.array([0.0]))[0]
    ax.axvspan(agl_min, ALTITUDE, color='grey', alpha=0.2, label='AGL range over the flanks')
    ax.set_xlabel('altitude above ground h (m)')
    ax.set_ylabel('GSD (cm/px)')
    ax.set_ylim(0, 6)
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)
    ax.set_title('S052 GSD vs altitude (25 mm lens, 3.75 µm pixel)')
    _save(fig, out_dir, 'gsd_altitude.png')


def plot_terrain_3d(res, out_dir):
    fig = plt.figure(figsize=(11, 7))
    ax = fig.add_subplot(111, projection='3d', computed_zorder=False)
    gx, gy = grid_centres(10.0)
    X, Y = np.meshgrid(gx, gy)
    Z = terrain(X, Y)
    ax.plot_surface(X, Y, Z, cmap='Blues_r', alpha=0.6, linewidth=0, zorder=0)
    w = res['plans']['adapt']['wps']
    ax.plot(w[:6, 0], w[:6, 1], np.full(6, Z_FLY), color='magenta', lw=1.2, zorder=3,
            label='coarse altimeter pass (flight)')
    w = w[6:]
    ax.plot(w[:, 0], w[:, 1], np.full(len(w), Z_FLY), color=COLORS['adapt'], lw=0.7,
            label=f"{LABELS['adapt']} ({len(res['plans']['adapt']['strips'])} strips, "
                  "closer over the flanks)", zorder=3)
    xs_c, z_plan = res['dem']
    for y in COARSE_Y:
        ax.plot(xs_c, np.full_like(xs_c, y), terrain(xs_c, y), color='magenta', lw=1.5, zorder=2)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_zlabel('z (m)')
    ax.set_zlim(-5, 90)
    ax.set_box_aspect((600, 800, 250))
    ax.view_init(elev=22, azim=-70)
    ax.legend(loc='upper left', fontsize=8)
    ax.set_title(f'S052 Glacier valley and adaptive flight lines at z = {Z_FLY:.0f} m\n'
                 '(magenta on the ground = altimeter profiles measured by the coarse pass)')
    _save(fig, out_dir, 'terrain_3d.png')


def save_animation(res, out_dir):
    p = res['plans']['adapt']
    traj = res['traj']
    res_a = 4.0
    gx, gy = grid_centres(res_a)
    Z = terrain(gx[None, :], gy[:, None])
    cum = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(traj, axis=0), axis=1))])
    # photo along-path distances: map each photo to the distance at which it is taken
    w = p['wps']
    seg = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(w, axis=0), axis=1))])
    ph_d = []
    n_coarse = 2 * len(COARSE_Y)
    for (px, py), si in zip(p['photos'], p['sop']):
        a, b = w[n_coarse + 2 * si], w[n_coarse + 2 * si + 1]
        ph_d.append(seg[n_coarse + 2 * si] + np.hypot(px - a[0], py - a[1]))
        del b
    ph_d = np.array(ph_d)
    n = len(traj)
    step = int(np.ceil((n - 1) / ANIM_FRAMES))
    idx = [min(i * step, n - 1) for i in range(int(np.ceil((n - 1) / step)) + 1)]
    cnt = np.zeros(Z.shape, dtype=np.int32)
    state = {'done': 0}
    fig, ax = plt.subplots(figsize=(6, 7.2))
    im = ax.imshow(cnt, origin='lower', extent=[0, GLACIER_W, 0, GLACIER_L], cmap='RdYlBu',
                   vmin=0, vmax=15)
    fig.colorbar(im, ax=ax, shrink=0.8, label='photos per cell')
    line, = ax.plot([], [], '-', color='k', lw=0.6)
    dot, = ax.plot([], [], 'o', color='red', ms=7, mec='k')
    ax.set_xlim(0, GLACIER_W)
    ax.set_ylim(0, GLACIER_L)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')

    def update(f):
        k = idx[f]
        new = np.where((ph_d <= cum[k] + 1e-9))[0]
        for j in new[state['done']:]:
            px, py = p['photos'][j]
            agl = Z_FLY - Z
            inside = ((np.abs(gx[None, :] - px) <= agl * KW / 2)
                      & (np.abs(gy[:, None] - py) <= agl * KL / 2))
            cnt[inside] += 1
        state['done'] = len(new)
        im.set_data(np.clip(cnt, 0, 15))
        line.set_data(traj[:k + 1, 0], traj[:k + 1, 1])
        dot.set_data([traj[k, 0]], [traj[k, 1]])
        ax.set_title(f'S052 Adaptive survey, t = {k * DT / 60:.1f} min\n'
                     f'{state["done"]} photos, distance {cum[k] / 1000:.1f} km', fontsize=10)
        return im, line, dot

    anim = animation.FuncAnimation(fig, update, frames=len(idx), blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    anim.save(path, writer='pillow', fps=15, dpi=80)
    plt.close(fig)
    print(f'Saved: {path} ({len(idx)} frames, every {step} steps of {DT} s)')


if __name__ == '__main__':
    res = run_simulation()
    m = res['metrics']
    fp_w, fp_l, gsd = compute_footprint(ALTITUDE)
    bx, by = fp_l * (1 - FORWARD_OVERLAP), fp_w * (1 - SIDE_OVERLAP)
    n_s, n_p = int(np.ceil(GLACIER_W / by)), int(np.ceil(GLACIER_L / bx))
    d_b = n_s * GLACIER_L + (n_s - 1) * by
    d_u = n_s * GLACIER_L + (n_s - 1) * (GLACIER_L + by)
    print('\n=== S052 Glacier Melt Area — Results ===')
    print(f'  Pixel pitch: {PIXEL_PITCH * 1000:.3f} um; GSD at {ALTITUDE:.0f} m: {gsd * 100:.3f} cm/px '
          f'(GSD = 5 cm reached at h = {0.05 * FOCAL_LENGTH / PIXEL_PITCH:.0f} m; '
          f'GSD = 4.8 cm at h = {0.048 * FOCAL_LENGTH / PIXEL_PITCH:.0f} m)')
    print(f'  Footprint at {ALTITUDE:.0f} m: {fp_w:.2f} x {fp_l:.2f} m; Bx = {bx:.2f} m, By = {by:.3f} m')
    print(f'  Card formulas: N_strips = ceil({GLACIER_W:.0f}/{by:.3f}) = {n_s}, N_photos = {n_p}, '
          f'N_total = {n_s * n_p}')
    print(f'  Card formulas: D_bous = {d_b:.0f} m ({d_b / DRONE_SPEED / 60:.1f} min), '
          f'D_uni = {d_u:.0f} m, saving (N-1)L = {(n_s - 1) * GLACIER_L:.0f} m')
    print(f'  Mean coverage 1/((1-Of)(1-Os)) = {1 / ((1 - FORWARD_OVERLAP) * (1 - SIDE_OVERLAP)):.1f}; '
          f'card first expression 1/(Of*Os) = {1 / (FORWARD_OVERLAP * SIDE_OVERLAP):.2f}')
    zt = terrain(np.array([0.0, 300.0]), np.array([0.0, 0.0]))
    print(f'  Terrain: centreline z = {zt[1]:.1f} m, flank edge z = {zt[0]:.1f} m '
          f'-> AGL {Z_FLY - zt[0]:.0f}..{Z_FLY - zt[1]:.0f} m; min AGL along adaptive flight '
          f'{res["agl_traj_min"]:.1f} m')
    xs_c, z_plan = res['dem']
    gxx, gyy = grid_centres(COV_RES)
    z_true_max = terrain(gxx[None, :], gyy[:, None]).max()
    print(f'  Coarse-pass planning profile max = {z_plan.max():.2f} m; true terrain max over the '
          f'glacier {z_true_max:.2f} m; coarse transects at y = {COARSE_Y} m')
    for s in STRATS:
        r = m[s]
        print(f'  [{LABELS[s]}]')
        print(f"    strips {r['n_strips']}, photos {r['n_photos']}, distance {r['dist']:.0f} m "
              f"(coarse pass {r['d_coarse']:.0f} m), time {r['time'] / 60:.1f} min")
        print(f"    coverage count min {r['cov_min']}, mean {r['cov_mean']:.2f}, "
              f">= 3 photos {100 * r['frac3']:.2f}% of area")
        print(f"    min side overlap {100 * r['side_min']:.1f}%, min forward overlap "
              f"{100 * r['fwd_min']:.1f}%, area meeting 60/80 overlap {100 * r['compliant']:.1f}%, "
              f"area > 5 pp short {100 * r['severe']:.1f}%")
        print(f"    GSD {r['gsd_min']:.2f}..{r['gsd_max']:.2f} cm/px")
        print(f"    melt area {r['a_est']:.0f} m^2 (error {100 * r['a_err']:+.2f}%), "
              f"melt in <3-photo zone {r['a_lost']:.0f} m^2")
    p_strip = res['plans']['adapt']['strips']
    print(f"  Adaptive spacing range: {np.diff(p_strip).min():.2f}..{np.diff(p_strip).max():.2f} m; "
          f"trigger interval {res['plans']['adapt']['bxs'].min():.2f}..{res['plans']['adapt']['bxs'].max():.2f} m")
    o_s, o_f = overlap_maps(p_strip, res['plans']['adapt']['bxs'])
    gxx, gyy = grid_centres(COV_RES)
    bad = (o_s < SIDE_OVERLAP - 1e-9) | (o_f < FORWARD_OVERLAP - 1e-9)
    jb, ib = np.where(bad)
    jm, im_ = np.unravel_index(np.argmin(o_s), o_s.shape)
    print(f"  Adaptive non-compliant cells: x {gxx[ib].min():.1f}..{gxx[ib].max():.1f} m, "
          f"y {gyy[jb].min():.1f}..{gyy[jb].max():.1f} m; min side overlap at "
          f"({gxx[im_]:.1f}, {gyy[jm]:.1f}) m")
    print(f"  Worst-case spacing {np.diff(res['plans']['worst']['strips'])[0]:.2f} m, "
          f"trigger {res['plans']['worst']['bxs'][0]:.2f} m")
    print(f"  Melt truth (analytic ellipses) {res['a_true']:.0f} m^2 = "
          f"{100 * res['a_true'] / (GLACIER_L * GLACIER_W):.2f}% of glacier; "
          f"NDWI over the whole raster (no coverage mask) {res['a_all']:.0f} m^2 "
          f"({100 * (res['a_all'] - res['a_true']) / res['a_true']:+.2f}%)")
    p_miss = 0.5 * (1 + __import__('math').erf((NDWI_THRESHOLD - NDWI_MELT[0]) / (NDWI_MELT[1] * np.sqrt(2))))
    print(f'  Expected melt-pixel miss rate P(N(0.4,0.08) <= 0.2) = {100 * p_miss:.2f}%')
    print(f'  Monte Carlo ({N_MC} seeds): melt-area error mean +- std')
    for s in STRATS:
        e = res['mc'][s]['err']
        print(f"    {LABELS[s]:20s} {100 * e.mean():+.2f} +- {100 * e.std():.2f}%  "
              f"(min {100 * e.min():+.2f}, max {100 * e.max():+.2f}), "
              f"melt in <3-photo zone {res['mc'][s]['lost'].mean():.1f} m^2 mean "
              f"({int(np.sum(res['mc'][s]['lost'] > 0))} seeds > 0), "
              f"area >= 3 photos {100 * res['mc'][s]['frac3'].mean():.2f}%, "
              f"distance {res['mc'][s]['dist'].mean():.0f} +- {res['mc'][s]['dist'].std():.0f} m")
    ea = np.abs(res['mc']['adapt']['err'])
    eb = np.abs(res['mc']['bous']['err'])
    print(f"    |error| adaptive vs boustrophedon: adaptive smaller in {int(np.sum(ea < eb - 1e-12))}, "
          f"equal in {int(np.sum(np.abs(ea - eb) <= 1e-12))}, larger in {int(np.sum(ea > eb + 1e-12))} "
          f"of {N_MC} seeds")
    eu = np.abs(res['mc']['uni']['err'])
    ew = np.abs(res['mc']['worst']['err'])
    print(f"    identical results: uni = bous in {int(np.sum(np.abs(eu - eb) <= 1e-12))}/{N_MC}, "
          f"adaptive = worst-case in {int(np.sum(np.abs(ea - ew) <= 1e-12))}/{N_MC} seeds")

    out = os.path.normpath(OUTPUT_DIR)
    os.makedirs(out, exist_ok=True)
    plot_coverage_path(res, out)
    plot_coverage_heatmap(res, out)
    plot_ndwi(res, out)
    plot_melt_classification(res, out)
    plot_strategy_comparison(res, out)
    plot_gsd_altitude(res, out)
    plot_terrain_3d(res, out)
    save_animation(res, out)
