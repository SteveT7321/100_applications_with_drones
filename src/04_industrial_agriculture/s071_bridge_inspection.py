"""
S071 Bridge Underside Structural Inspection
===========================================
A drone flies boustrophedon passes 0.5 m below a 50 x 10 m bridge deck (z = 8 m),
holding its upward clearance with a PD controller fed by a noisy upward rangefinder.
An upward camera (45 deg half-angle -> 1 m wide square footprint) looks for 30 seeded
cracks. The script reports coverage, crack detection rate and safety-ceiling violations
for the card's plan, then asks the questions the card's targets raise:

  1. Coverage vs. strip overlap when the horizontal position has a 0.1 m error
     (card GPS_ERROR), with common random numbers over a Monte Carlo of seeds, and
     how that result depends on the error's correlation time.
  2. Safety margin: how much rangefinder noise the 0.2 m ceiling margin tolerates,
     with and without the card's emergency-descent override (plus two probes:
     a damped override and a rangefinder-triggered override).

Usage:
    conda activate drones
    python src/04_industrial_agriculture/s071_bridge_inspection.py
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.colors import ListedColormap

# ── Parameters (card) ─────────────────────────────────────────────────────────
BRIDGE_LEN   = 50.0     # m  deck length (x)
BRIDGE_WID   = 10.0     # m  deck width (y)
Z_DECK       = 8.0      # m  deck underside height
D_TARGET     = 0.5      # m  inspection standoff below deck
Z_NOM        = Z_DECK - D_TARGET      # 7.5 m
Z_MAX        = Z_DECK - 0.2           # 7.8 m safety ceiling
SIGMA_RANGE  = 0.02     # m  rangefinder noise std
V_CRUISE     = 1.0      # m/s
FOV_HALF_DEG = 45.0     # deg
OVERLAP      = 0.10     # strip overlap fraction
P_DETECT     = 0.90     # per-timestep detection probability inside footprint
N_CRACKS     = 30
DT           = 0.05     # s
KP, KD       = 16.0, 8.0   # s^-2, s^-1  (omega_n = 4 rad/s, critically damped)
K_EMG        = 30.0     # s^-2 emergency descent gain
GRID_RES     = 0.1      # m  coverage grid cell
CRACK_SEED   = 7        # card seed for the nominal crack layout
NOISE_SEED   = 42       # card seed for the nominal noise stream

# ── Added parameters (not in card; see README Deviations) ─────────────────────
SIGMA_NAV    = 0.10     # m  horizontal position error std (card's GPS_ERROR value)
TAU_NAV      = 20.0     # s  correlation time of the horizontal error (Gauss-Markov)
N_MC         = 50       # Monte Carlo seeds for the overlap study
MC_SEED0     = 1000     # first Monte Carlo seed
OVERLAPS     = [0.0, 0.1, 0.2, 0.3]
SIGMA_SWEEP  = [0.02, 0.10, 0.20, 0.30, 0.35, 0.40, 0.45]   # m rangefinder noise stress test
N_SIGMA_MC   = 20       # seeds per sigma value
# Ceiling-protection variants of the stress test:
#   card   - card override  u = -K_emg (z - z_max) when the true z > z_max
#   off    - PD only
#   damped - card override plus the PD damping term -K_d z_dot (probe)
#   noisy  - card override triggered and driven by the rangefinder height (probe)
OVERRIDE_MODES = ['card', 'off', 'damped', 'noisy']
TAU_SWEEP    = [2.0, 20.0, 100.0]   # s nav-error correlation times (sensitivity)
TAU_OVERLAPS = [0.1, 0.2]
N_TAU_MC     = 20       # seeds per (tau, overlap)
K_SIGMA_STEP = 6.0      # asserted bound on a nav-error increment, in increment std devs
ANIM_STEP    = 100      # simulation steps per GIF frame

# ── Derived geometry ──────────────────────────────────────────────────────────
ALPHA   = np.radians(FOV_HALF_DEG)
H_STRIP = D_TARGET * np.tan(ALPHA)            # 0.5 m
W_STRIP = 2.0 * H_STRIP                       # 1.0 m

COV_CMAP = ListedColormap(['white', '#7fc97f'])   # unseen / seen

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '04_industrial_agriculture', 's071_bridge_inspection',
)


# ── Planning ──────────────────────────────────────────────────────────────────
def pass_centres_card(overlap=OVERLAP):
    """Card formula: N = ceil(W / dy), y_k = dy/2 + (k-1) dy."""
    dy = W_STRIP * (1.0 - overlap)
    n = int(np.ceil(BRIDGE_WID / dy - 1e-9))
    return dy / 2.0 + np.arange(n) * dy


def pass_centres(overlap=OVERLAP):
    """Edge-aligned plan: first pass at y = h, spacing dy, last pass clamped to W - h.

    N = ceil((W - w) / dy) + 1 is the smallest pass count whose footprints tile [0, W]
    with every adjacent pair overlapping by at least overlap * w.
    """
    dy = W_STRIP * (1.0 - overlap)
    n = int(np.ceil((BRIDGE_WID - W_STRIP) / dy - 1e-9)) + 1
    y = H_STRIP + np.arange(n) * dy
    return np.minimum(y, BRIDGE_WID - H_STRIP)


def build_waypoints(ys):
    wps = []
    for k, yc in enumerate(ys):
        if k % 2 == 0:
            wps += [(0.0, yc), (BRIDGE_LEN, yc)]
        else:
            wps += [(BRIDGE_LEN, yc), (0.0, yc)]
    return np.array(wps)


def polyline_length(wps):
    return float(np.sum(np.linalg.norm(np.diff(wps, axis=0), axis=1)))


def sample_path(wps):
    """Constant-speed planned xy trajectory sampled every DT; also pass-end step indices."""
    seg = np.diff(wps, axis=0)
    seg_len = np.linalg.norm(seg, axis=1)
    cum = np.concatenate([[0.0], np.cumsum(seg_len)])
    n_steps = int(np.ceil(cum[-1] / (V_CRUISE * DT) - 1e-9))
    s = np.minimum(np.arange(n_steps + 1) * V_CRUISE * DT, cum[-1])
    idx = np.clip(np.searchsorted(cum, s, side='right') - 1, 0, len(seg) - 1)
    frac = (s - cum[idx]) / seg_len[idx]
    xy = wps[idx] + frac[:, None] * seg[idx]
    # pass k ends at waypoint 2k+1 (arc-length cum[2k+1])
    pass_end = [int(np.searchsorted(s, cum[2 * k + 1] - 1e-9)) for k in range(len(wps) // 2)]
    return xy, pass_end


# ── Stochastic inputs (one stream per seed -> common random numbers) ──────────
def seed_cracks(n, seed):
    rng = np.random.default_rng(seed)
    x = rng.uniform(1.0, BRIDGE_LEN - 1.0, n)
    y = rng.uniform(0.5, BRIDGE_WID - 0.5, n)
    r = rng.uniform(0.05, 0.20, n)
    return np.stack([x, y, r], axis=1)


def draw_noise(seed, n_steps):
    """Pre-draw every random number a run can use, so all strategies share them."""
    rng = np.random.default_rng(seed)
    range_noise = rng.normal(0.0, 1.0, n_steps)          # scaled by sigma_range later
    nav_xi = rng.normal(0.0, 1.0, (n_steps, 2))
    nav_e0 = rng.normal(0.0, SIGMA_NAV, 2)
    u_det = rng.random((n_steps, N_CRACKS))
    return dict(range=range_noise, nav_xi=nav_xi, nav_e0=nav_e0, u_det=u_det)


def nav_error(noise, n, sigma=SIGMA_NAV, tau=TAU_NAV):
    """First-order Gauss-Markov horizontal error, stationary std sigma, corr. time tau."""
    phi = np.exp(-DT / tau)
    q = sigma * np.sqrt(1.0 - phi ** 2)
    e = np.empty((n, 2))
    e[0] = noise['nav_e0'] * (sigma / SIGMA_NAV)
    xi = noise['nav_xi']
    for k in range(1, n):
        e[k] = phi * e[k - 1] + q * xi[k]
    return e


# ── Vertical (inverse surface-following) controller ───────────────────────────
def fly_height(n, range_noise, sigma_range=SIGMA_RANGE, mode='card'):
    """PD on the noisy upward range, double-integrator z. Returns z, vz, flags.
    mode selects the ceiling protection (see OVERRIDE_MODES)."""
    z = np.empty(n)
    vz = np.empty(n)
    viol = np.zeros(n, dtype=bool)
    z[0], vz[0] = Z_NOM, 0.0
    crashed_at = -1
    for k in range(1, n):
        zk, vk = z[k - 1], vz[k - 1]
        d_meas = (Z_DECK - zk) + sigma_range * range_noise[k - 1]
        ez = d_meas - D_TARGET
        uz = KP * ez + KD * (-vk)                 # de_z/dt = -z_dot
        if mode in ('card', 'damped') and zk > Z_MAX:
            uz = -K_EMG * (zk - Z_MAX)
            if mode == 'damped':
                uz -= KD * vk
        elif mode == 'noisy':
            z_meas = Z_DECK - d_meas
            if z_meas > Z_MAX:
                uz = -K_EMG * (z_meas - Z_MAX)
        vk = vk + uz * DT
        zk = zk + vk * DT
        z[k], vz[k] = zk, vk
        viol[k] = zk > Z_MAX
        if zk >= Z_DECK:                          # deck strike: the flight ends here
            crashed_at = k
            z, vz, viol = z[:k + 1], vz[:k + 1], viol[:k + 1]
            break
    return z, vz, viol, crashed_at


# ── Sensing ───────────────────────────────────────────────────────────────────
def coverage_grid(xy_true, upto=None):
    nx = int(round(BRIDGE_LEN / GRID_RES))
    ny = int(round(BRIDGE_WID / GRID_RES))
    grid = np.zeros((nx, ny), dtype=bool)
    pts = xy_true if upto is None else xy_true[:upto]
    # cell centre c_j = (j + 0.5) res is covered if |x - c_j| <= h
    j0 = np.maximum(np.ceil((pts[:, 0] - H_STRIP) / GRID_RES - 0.5 - 1e-9), 0).astype(int)
    j1 = np.minimum(np.floor((pts[:, 0] + H_STRIP) / GRID_RES - 0.5 + 1e-9), nx - 1).astype(int)
    k0 = np.maximum(np.ceil((pts[:, 1] - H_STRIP) / GRID_RES - 0.5 - 1e-9), 0).astype(int)
    k1 = np.minimum(np.floor((pts[:, 1] + H_STRIP) / GRID_RES - 0.5 + 1e-9), ny - 1).astype(int)
    for a, b, c, d in zip(j0, j1, k0, k1):
        if a <= b and c <= d:
            grid[a:b + 1, c:d + 1] = True
    return grid


def detect_cracks(xy_true, nav_err, cracks, u_det):
    """First Bernoulli success while the crack centre is in the square footprint."""
    n = len(xy_true)
    in_fp = ((np.abs(xy_true[:, None, 0] - cracks[None, :, 0]) <= H_STRIP) &
             (np.abs(xy_true[:, None, 1] - cracks[None, :, 1]) <= H_STRIP))
    hit = in_fp & (u_det[:n] < P_DETECT)
    detected = hit.any(axis=0)
    t_det = np.where(detected, hit.argmax(axis=0), -1)
    # The report logs the crack at (estimated drone position + camera offset);
    # estimated = true - nav error, so the logged position is off by -nav_err.
    est = np.full((len(cracks), 2), np.nan)
    loc_err = np.full(len(cracks), np.nan)
    for i in np.where(detected)[0]:
        est[i] = cracks[i, :2] - nav_err[t_det[i]]
        loc_err[i] = np.linalg.norm(nav_err[t_det[i]])
    frames_in = in_fp.sum(axis=0)
    return detected, t_det, est, loc_err, frames_in


# ── One mission ───────────────────────────────────────────────────────────────
def run_mission(overlap, cracks, noise, with_nav_error=True, sigma_range=SIGMA_RANGE,
                mode='card', ys=None, tau=TAU_NAV):
    if ys is None:
        ys = pass_centres(overlap)
    wps = build_waypoints(ys)
    xy_plan, pass_end = sample_path(wps)
    n = len(xy_plan)
    e = nav_error(noise, n, tau=tau) if with_nav_error else np.zeros((n, 2))
    xy_true = xy_plan + e
    z, vz, viol, crashed = fly_height(n, noise['range'], sigma_range, mode)
    assert crashed < 0, 'drone struck the deck in a nominal mission'
    # Physical invariants (X4)
    step_plan = np.linalg.norm(np.diff(xy_plan, axis=0), axis=1)
    assert step_plan.max() <= V_CRUISE * DT + 1e-9, 'planned speed above V_CRUISE'
    # The true path adds the drifting position error to the plan; its step-to-step
    # increments raise the instantaneous true speed above V_CRUISE (bounded, asserted).
    # The drift model's increments are white, so they add step-to-step jitter to the true
    # speed. Assert that each true step is the planned step plus at most K_SIGMA_STEP
    # standard deviations of the 2D drift increment (no teleporting).
    v_true = np.linalg.norm(np.diff(xy_true, axis=0), axis=1) / DT
    q = SIGMA_NAV * np.sqrt(1.0 - np.exp(-2 * DT / tau)) if with_nav_error else 0.0
    step_bound = V_CRUISE * DT + K_SIGMA_STEP * np.sqrt(2) * q
    assert v_true.max() * DT <= step_bound + 1e-12, 'true step above the model bound'
    assert np.all(z < Z_DECK), 'drone above deck underside'
    assert np.all(np.abs(xy_true[:, 1] - xy_plan[:, 1]) < 10 * SIGMA_NAV + 1e-9)
    grid = coverage_grid(xy_true)
    detected, t_det, est, loc_err, frames_in = detect_cracks(xy_true, e, cracks,
                                                             noise['u_det'])
    cum_cov = [coverage_grid(xy_true, upto=pe + 1).mean() for pe in pass_end]
    # unseen cells inside the border band (within h of any deck edge)
    cx = (np.arange(grid.shape[0]) + 0.5) * GRID_RES
    cy = (np.arange(grid.shape[1]) + 0.5) * GRID_RES
    border = ((np.minimum(cx, BRIDGE_LEN - cx) < H_STRIP)[:, None] |
              (np.minimum(cy, BRIDGE_WID - cy) < H_STRIP)[None, :])
    unseen = ~grid
    border_share = unseen[border].sum() / max(unseen.sum(), 1)
    return dict(overlap=overlap, ys=ys, wps=wps, xy_plan=xy_plan, xy_true=xy_true,
                nav_err=e, z=z, vz=vz, viol=viol, grid=grid, coverage=grid.mean(),
                detected=detected, t_det=t_det, crack_est=est, loc_err=loc_err,
                frames_in=frames_in, det_rate=detected.mean(), n_viol=int(viol.sum()),
                path_len=polyline_length(wps), t_flight=(n - 1) * DT,
                pass_end=pass_end, cum_cov=np.array(cum_cov), cracks=cracks,
                border_share=border_share, border_frac=border.mean(),
                v_true_max=v_true.max(), v_true_mean=v_true.mean(),
                v_true_p99=np.percentile(v_true, 99), v_true_bound=step_bound / DT)


def run_simulation():
    max_steps = 20000
    # Nominal mission: card seeds, overlap 0.10, edge-aligned plan
    cracks0 = seed_cracks(N_CRACKS, CRACK_SEED)
    noise0 = draw_noise(NOISE_SEED, max_steps)
    nominal = run_mission(OVERLAP, cracks0, noise0)
    ideal = run_mission(OVERLAP, cracks0, noise0, with_nav_error=False)
    card_ys = pass_centres_card(OVERLAP)
    card_ideal = run_mission(OVERLAP, cracks0, noise0, with_nav_error=False, ys=card_ys)

    # Overlap study: Monte Carlo with common random numbers (same cracks/noise per seed)
    mc = {o: dict(cov=[], det=[], loc=[], t=[], border=[]) for o in OVERLAPS}
    for s in range(N_MC):
        seed = MC_SEED0 + s
        cracks = seed_cracks(N_CRACKS, seed)
        noise = draw_noise(seed, max_steps)
        for o in OVERLAPS:
            r = run_mission(o, cracks, noise)
            mc[o]['cov'].append(r['coverage'])
            mc[o]['det'].append(r['det_rate'])
            mc[o]['loc'].append(np.nanmean(r['loc_err']))
            mc[o]['t'].append(r['t_flight'])
            mc[o]['border'].append(r['border_share'])
    for o in OVERLAPS:
        for k in mc[o]:
            mc[o][k] = np.array(mc[o][k])

    # Sensitivity of the overlap result to the nav-error correlation time (CRN per seed)
    tau_mc = {}
    for tau in TAU_SWEEP:
        for o in TAU_OVERLAPS:
            cov = []
            for s in range(N_TAU_MC):
                seed = MC_SEED0 + s
                r = run_mission(o, seed_cracks(N_CRACKS, seed), draw_noise(seed, max_steps),
                                tau=tau)
                cov.append(r['coverage'])
            tau_mc[(tau, o)] = np.array(cov)

    # Safety stress test: rangefinder noise sweep, ceiling-protection variants (CRN per seed)
    n_nom = len(nominal['z'])
    range_noise = [draw_noise(MC_SEED0 + s, max_steps)['range'] for s in range(N_SIGMA_MC)]
    sweep = {}
    for sig in SIGMA_SWEEP:
        for mode in OVERRIDE_MODES:
            nv, crash, zmax = [], 0, []
            for s in range(N_SIGMA_MC):
                z, _, viol, c = fly_height(n_nom, range_noise[s], sig, mode)
                nv.append(int(viol.sum()))
                zmax.append(z.max())
                crash += int(c >= 0)
            sweep[(sig, mode)] = dict(n_viol=np.array(nv), crashes=crash, zmax=np.array(zmax))
    return dict(nominal=nominal, ideal=ideal, card_ideal=card_ideal, mc=mc, sweep=sweep,
                tau_mc=tau_mc)


# ── Plots ─────────────────────────────────────────────────────────────────────
def plot_flight_path_3d(nom, out_dir):
    fig = plt.figure(figsize=(12, 6))
    ax = fig.add_subplot(111, projection='3d')
    deck = [[(0, 0, Z_DECK), (BRIDGE_LEN, 0, Z_DECK),
             (BRIDGE_LEN, BRIDGE_WID, Z_DECK), (0, BRIDGE_WID, Z_DECK)]]
    ax.add_collection3d(Poly3DCollection(deck, facecolor='grey', alpha=0.25, edgecolor='k'))
    for yc in nom['ys']:
        for yb in (yc - H_STRIP, yc + H_STRIP):
            ax.plot([0, BRIDGE_LEN], [yb, yb], [Z_DECK, Z_DECK], color='tab:blue',
                    lw=0.4, alpha=0.4)
    xy, z = nom['xy_true'], nom['z']
    ax.plot(xy[:, 0], xy[:, 1], z, color='red', lw=0.8, label='Drone path (true)')
    c, d = nom['cracks'], nom['detected']
    ax.scatter(c[d, 0], c[d, 1], np.full(d.sum(), Z_DECK), marker='*', s=70, c='green',
               label=f'Detected crack ({d.sum()})')
    if (~d).any():
        ax.scatter(c[~d, 0], c[~d, 1], np.full((~d).sum(), Z_DECK), marker='o', s=40,
                   c='grey', edgecolors='k', label=f'Missed crack ({(~d).sum()})')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_zlabel('z (m)')
    ax.set_zlim(7.0, 8.2)
    ax.set_zticks([7.0, 7.5, 8.0])
    ax.set_yticks([0, 5, 10])
    ax.set_box_aspect((5, 1, 0.6))
    ax.view_init(elev=-18, azim=-60)
    ax.set_title('S071 Bridge underside inspection: 3D flight path below the deck')
    ax.legend(loc='upper left', fontsize=8)
    path = os.path.join(out_dir, 'flight_path_3d.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_coverage_map(nom, out_dir):
    fig, ax = plt.subplots(figsize=(12, 3.4))
    g = nom['grid'].T.astype(float)
    ax.imshow(g, origin='lower', extent=[0, BRIDGE_LEN, 0, BRIDGE_WID], cmap=COV_CMAP,
              vmin=0, vmax=1, aspect='equal', interpolation='nearest')
    xy = nom['xy_true']
    ax.plot(xy[:, 0], xy[:, 1], color='k', lw=0.3, alpha=0.5)
    c, d = nom['cracks'], nom['detected']
    ax.plot(c[d, 0], c[d, 1], 'x', color='red', ms=7, mew=1.8, label='Detected crack')
    if (~d).any():
        ax.plot(c[~d, 0], c[~d, 1], 'o', mfc='none', mec='k', ms=8, label='Missed crack')
    gaps = np.argwhere(~nom['grid'])
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title(f"Deck coverage C = {100 * nom['coverage']:.2f}% "
                 f"({len(gaps)} of {nom['grid'].size} cells unseen, white); "
                 f"nav error sigma = {SIGMA_NAV} m, overlap {int(100 * OVERLAP)}%")
    ax.legend(loc='upper right', fontsize=8, framealpha=0.9)
    path = os.path.join(out_dir, 'coverage_map.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_height_series(nom, out_dir):
    t = np.arange(len(nom['z'])) * DT
    z = nom['z']
    fig, axes = plt.subplots(2, 1, figsize=(11, 6), sharex=True)
    ax = axes[0]
    ax.plot(t, z, color='red', lw=0.6, label='z(t)')
    ax.axhline(Z_NOM, color='tab:blue', ls='--', label='z_nom = 7.5 m')
    ax.axhline(Z_MAX, color='red', ls='--', label='z_max = 7.8 m')
    ax.axhline(Z_DECK, color='grey', lw=2, label='deck 8.0 m')
    if nom['viol'].any():
        ax.plot(t[nom['viol']], z[nom['viol']], 'r.', ms=3, label='violation')
    ax.set_ylim(7.3, 8.1)
    ax.set_ylabel('z (m)')
    ax.set_title(f"Height hold under the deck: {nom['n_viol']} safety violations, "
                 f"min clearance {Z_DECK - z.max():.3f} m")
    ax.legend(loc='lower right', fontsize=8, ncol=4)
    ax = axes[1]
    ax.fill_between(t, Z_NOM - SIGMA_RANGE, Z_NOM + SIGMA_RANGE, color='tab:blue',
                    alpha=0.15, label=r'$z_{nom} \pm \sigma_{range}$')
    ax.plot(t, z, color='red', lw=0.5)
    ax.axhline(Z_NOM, color='tab:blue', ls='--')
    ax.set_ylim(Z_NOM - 0.035, Z_NOM + 0.035)
    ax.set_xlabel('time (s)')
    ax.set_ylabel('z (m), zoomed')
    ax.set_title(f"Zoom: std of z = {1000 * z.std():.1f} mm "
                 f"(rangefinder noise std {1000 * SIGMA_RANGE:.0f} mm)")
    ax.legend(loc='lower right', fontsize=8)
    plt.tight_layout()
    path = os.path.join(out_dir, 'height_timeseries.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_strip_coverage(nom, out_dir):
    cc = 100 * nom['cum_cov']
    inc = np.diff(np.concatenate([[0], cc]))
    k = np.arange(1, len(cc) + 1)
    fig, ax = plt.subplots(figsize=(9, 4.5))
    ax.bar(k, cc, color='green', alpha=0.75, label='cumulative coverage')
    for ki, c, d in zip(k, cc, inc):
        ax.text(ki, c + 1, f'+{d:.1f}', ha='center', fontsize=7)
    ax.axhline(99, color='red', ls='--', lw=1, label='card target 99%')
    ax.set_xticks(k)
    ax.set_xlabel('pass number')
    ax.set_ylabel('cumulative coverage (%)')
    ax.set_ylim(0, 115)
    ax.set_title(f"Coverage after each pass ({len(cc)} passes; labels = increment in pp)")
    ax.legend(loc='upper left', fontsize=8)
    path = os.path.join(out_dir, 'strip_coverage.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_overlap_study(mc, out_dir):
    fig, axes = plt.subplots(1, 3, figsize=(14, 4.5))
    labels = [f'{int(100 * o)}%' for o in OVERLAPS]
    cols = ['tab:orange', 'tab:green', 'tab:purple', 'tab:brown']
    for ax, key, ylab, scale in [(axes[0], 'cov', 'coverage (%)', 100),
                                 (axes[1], 'det', 'crack detection rate (%)', 100)]:
        data = [scale * mc[o][key] for o in OVERLAPS]
        bp = ax.boxplot(data, tick_labels=labels, patch_artist=True, widths=0.5)
        for p, c in zip(bp['boxes'], cols):
            p.set_facecolor(c)
            p.set_alpha(0.5)
        ax.axhline(99 if key == 'cov' else 90, color='red', ls='--', lw=1,
                   label='card target')
        ax.set_xlabel('strip overlap')
        ax.set_ylabel(ylab)
        ax.legend(fontsize=8, loc='lower right')
        ax.grid(alpha=0.3)
    axes[0].set_title(f'Coverage, {N_MC} seeds (CRN)')
    axes[1].set_title('Detection rate (30 cracks / seed)')
    ax = axes[2]
    t = [mc[o]['t'][0] for o in OVERLAPS]
    ax.bar(labels, t, color=cols, alpha=0.7)
    for i, v in enumerate(t):
        ax.text(i, v + 5, f'{v:.0f} s', ha='center', fontsize=8)
    ax.set_xlabel('strip overlap')
    ax.set_ylabel('flight time (s)')
    ax.set_title('Flight time (deterministic)')
    ax.set_ylim(0, max(t) * 1.15)
    plt.suptitle(f'Overlap trade-off with horizontal nav error sigma = {SIGMA_NAV} m, '
                 f'tau = {TAU_NAV:.0f} s')
    plt.tight_layout()
    path = os.path.join(out_dir, 'overlap_study.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_safety_sweep(sweep, out_dir):
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.5))
    sig = np.array(SIGMA_SWEEP)
    for mode, c, lab in [('card', 'red', 'card override (true z)'),
                         ('off', 'tab:blue', 'PD only'),
                         ('damped', 'tab:orange', 'card override + damping (probe)'),
                         ('noisy', 'tab:purple', 'card override, rangefinder trigger (probe)')]:
        mv = [sweep[(s, mode)]['n_viol'].mean() for s in SIGMA_SWEEP]
        zm = [sweep[(s, mode)]['zmax'].mean() for s in SIGMA_SWEEP]
        axes[0].plot(sig, mv, 'o-', color=c, label=lab)
        axes[1].plot(sig, zm, 'o-', color=c, label=lab)
    axes[0].set_yscale('symlog', linthresh=1)
    axes[0].set_xlabel('rangefinder noise std (m)')
    axes[0].set_ylabel('violation steps per mission (mean)')
    axes[0].set_title(f'Safety violations vs noise ({N_SIGMA_MC} seeds, CRN)')
    axes[1].axhline(Z_MAX, color='red', ls='--', lw=1, label='z_max')
    axes[1].axhline(Z_DECK, color='grey', lw=2, label='deck')
    axes[1].set_xlabel('rangefinder noise std (m)')
    axes[1].set_ylabel('max z in mission (m, mean)')
    axes[1].set_title('Highest point reached')
    for ax in axes:
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
    plt.tight_layout()
    path = os.path.join(out_dir, 'safety_sweep.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_tau_sensitivity(tau_mc, out_dir):
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.5))
    for o, c in zip(TAU_OVERLAPS, ['tab:green', 'tab:purple']):
        m = [100 * tau_mc[(t, o)].mean() for t in TAU_SWEEP]
        sd = [100 * tau_mc[(t, o)].std() for t in TAU_SWEEP]
        k = [int((tau_mc[(t, o)] >= 0.99).sum()) for t in TAU_SWEEP]
        axes[0].errorbar(TAU_SWEEP, m, yerr=sd, fmt='o-', color=c, capsize=4,
                         label=f'overlap {int(100 * o)}%')
        axes[1].plot(TAU_SWEEP, k, 'o-', color=c, label=f'overlap {int(100 * o)}%')
    axes[0].axhline(99, color='red', ls='--', lw=1, label='card target 99%')
    axes[0].set_ylabel('coverage (%), mean ± std')
    axes[1].set_ylabel(f'seeds with coverage >= 99% (of {N_TAU_MC})')
    axes[1].set_ylim(0, N_TAU_MC + 1)
    for ax in axes:
        ax.set_xscale('log')
        ax.set_xticks(TAU_SWEEP)
        ax.set_xticklabels([f'{t:g}' for t in TAU_SWEEP])
        ax.set_xlabel('nav-error correlation time tau (s)')
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
    axes[0].set_title(f'Coverage vs tau (sigma = {SIGMA_NAV} m, {N_TAU_MC} seeds, CRN)')
    axes[1].set_title('How often the card target is met')
    plt.tight_layout()
    path = os.path.join(out_dir, 'tau_sensitivity.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def save_animation(nom, out_dir):
    xy = nom['xy_true']
    n = len(xy)
    n_frames = int(np.ceil((n - 1) / ANIM_STEP)) + 1
    fig, ax = plt.subplots(figsize=(10, 2.9))
    nx, ny = nom['grid'].shape
    img = ax.imshow(np.zeros((ny, nx)), origin='lower', extent=[0, BRIDGE_LEN, 0, BRIDGE_WID],
                    cmap=COV_CMAP, vmin=0, vmax=1, aspect='equal',
                    interpolation='nearest')
    trail, = ax.plot([], [], color='k', lw=0.4, alpha=0.6)
    drone, = ax.plot([], [], 'o', color='red', ms=6)
    fp = plt.Rectangle((0, 0), W_STRIP, W_STRIP, fill=False, ec='red', lw=1)
    ax.add_patch(fp)
    c = nom['cracks']
    ax.plot(c[:, 0], c[:, 1], '.', color='grey', ms=4)
    det_pts, = ax.plot([], [], '*', color='green', ms=9)
    flash, = ax.plot([], [], 'o', mfc='none', mec='gold', ms=16, mew=2)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    # incremental coverage for speed
    grid = np.zeros((nx, ny), dtype=bool)
    state = {'last': 0}

    def update(f):
        k = min(f * ANIM_STEP, n - 1)
        lo = state['last']
        g = coverage_grid(xy[lo:k + 1])
        grid[:] |= g
        state['last'] = k
        img.set_data(grid.T.astype(float))
        trail.set_data(xy[:k + 1, 0], xy[:k + 1, 1])
        drone.set_data([xy[k, 0]], [xy[k, 1]])
        fp.set_xy((xy[k, 0] - H_STRIP, xy[k, 1] - H_STRIP))
        dmask = (nom['t_det'] >= 0) & (nom['t_det'] <= k)
        det_pts.set_data(c[dmask, 0], c[dmask, 1])
        new = (nom['t_det'] > k - ANIM_STEP) & (nom['t_det'] <= k) & (nom['t_det'] >= 0)
        flash.set_data(c[new, 0], c[new, 1])
        ax.set_title(f't = {k * DT:5.1f} s | coverage {100 * grid.mean():5.1f}% | '
                     f'cracks detected {int(dmask.sum())}/{N_CRACKS}', fontsize=10)
        return img, trail, drone, fp, det_pts, flash

    anim = animation.FuncAnimation(fig, update, frames=n_frames, interval=100, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    anim.save(path, writer='pillow', fps=10, dpi=80)
    plt.close(fig)
    print(f'Saved: {path} ({n_frames} frames)')


if __name__ == '__main__':
    res = run_simulation()
    nom, ideal, card = res['nominal'], res['ideal'], res['card_ideal']
    mc, sweep = res['mc'], res['sweep']

    print('\n=== S071 Bridge Underside Inspection — Results ===')
    print(f'  Footprint half-width h = {H_STRIP:.3f} m, strip width {W_STRIP:.3f} m, '
          f'step dy = {W_STRIP * (1 - OVERLAP):.3f} m')
    print(f"  Card plan:  {len(card['ys'])} passes at y = "
          + ', '.join(f'{y:.2f}' for y in card['ys']))
    print(f"    last pass {card['ys'][-1] - BRIDGE_WID:.2f} m outside the deck edge; "
          f"path {card['path_len']:.1f} m; coverage without nav error "
          f"{100 * card['coverage']:.2f}%")
    print(f"  Edge-aligned plan: {len(nom['ys'])} passes at y = "
          + ', '.join(f'{y:.2f}' for y in nom['ys']))
    print(f"    path {nom['path_len']:.1f} m; coverage without nav error "
          f"{100 * ideal['coverage']:.2f}%; detection without nav error "
          f"{100 * ideal['det_rate']:.1f}%")
    print(f"    min frames a crack spends in the footprint (no nav error): "
          f"{ideal['frames_in'].min()} -> P(miss | covered) <= "
          f"{(1 - P_DETECT) ** ideal['frames_in'].min():.1e}")
    print('  [Nominal mission: overlap 10%, nav error sigma 0.1 m, card seeds]')
    print(f"    Coverage:             {100 * nom['coverage']:.2f}% "
          f"({int((~nom['grid']).sum())} of {nom['grid'].size} cells unseen)")
    print(f"    Unseen cells in the {H_STRIP:.1f} m border band: "
          f"{100 * nom['border_share']:.1f}% (band = {100 * nom['border_frac']:.1f}% of deck)")
    print(f"    Crack detection:      {int(nom['detected'].sum())}/{N_CRACKS} "
          f"= {100 * nom['det_rate']:.1f}%")
    missed = np.where(~nom['detected'])[0]
    for i in missed:
        c = nom['cracks'][i]
        print(f'      missed crack {i} at ({c[0]:.2f}, {c[1]:.2f}) m, '
              f"frames in footprint {nom['frames_in'][i]}")
    print(f"    Crack log position error: mean {1000 * np.nanmean(nom['loc_err']):.0f} mm, "
          f"max {1000 * np.nanmax(nom['loc_err']):.0f} mm")
    print(f"    Safety violations:    {nom['n_viol']}")
    print(f"    Max z: {nom['z'].max():.4f} m (min clearance {Z_DECK - nom['z'].max():.4f} m); "
          f"std z {1000 * nom['z'].std():.2f} mm; max |z - z_nom| "
          f"{1000 * np.abs(nom['z'] - Z_NOM).max():.1f} mm")
    print(f"    Flight time:          {nom['t_flight']:.1f} s")
    print(f"    Path length:          {nom['path_len']:.1f} m")
    print(f"    Max |nav error|:      {1000 * np.linalg.norm(nom['nav_err'], axis=1).max():.0f} mm")
    print(f"    True horizontal speed (plan + nav error): mean {nom['v_true_mean']:.3f} m/s, "
          f"99th percentile {nom['v_true_p99']:.2f} m/s, max {nom['v_true_max']:.2f} m/s "
          f"(asserted <= {nom['v_true_bound']:.2f} m/s = cruise + {K_SIGMA_STEP:.0f} std of "
          f"the drift increment; commanded speed {V_CRUISE:.1f} m/s)")
    print(f'  [Overlap study: {N_MC} seeds, CRN; nav error sigma {SIGMA_NAV} m, '
          f'tau {TAU_NAV:.0f} s]')
    print('    overlap passes time(s)  coverage mean±std (min)   detect mean±std   '
          'cov>=99%  det=100%  loc err(mm)')
    for o in OVERLAPS:
        m = mc[o]
        print(f"    {100 * o:5.0f}%  {len(pass_centres(o)):5d}  {m['t'][0]:7.1f}  "
              f"{100 * m['cov'].mean():6.2f} ± {100 * m['cov'].std():4.2f} "
              f"({100 * m['cov'].min():6.2f})   "
              f"{100 * m['det'].mean():5.1f} ± {100 * m['det'].std():4.1f}   "
              f"{int((m['cov'] >= 0.99).sum()):3d}/{N_MC}   "
              f"{int((m['det'] == 1.0).sum()):3d}/{N_MC}   {1000 * m['loc'].mean():5.0f}")
    print('    Share of unseen cells in the border band (mean): ' + ', '.join(
        f"{100 * o:.0f}% -> {100 * mc[o]['border'].mean():.0f}%" for o in OVERLAPS))
    print('    Paired wins on coverage (row overlap beats 10%, CRN):')
    for o in OVERLAPS:
        if o == OVERLAP:
            continue
        d = mc[o]['cov'] - mc[OVERLAP]['cov']
        print(f"      {100 * o:3.0f}% vs 10%: wins {int((d > 0).sum())}, "
              f"ties {int((d == 0).sum())}, losses {int((d < 0).sum())}; "
              f"mean diff {100 * d.mean():+.2f} pp")
    tm = res['tau_mc']
    print(f'  [Nav-error correlation time sensitivity: {N_TAU_MC} seeds, CRN, sigma {SIGMA_NAV} m]')
    print('    tau(s)  overlap  coverage mean±std   seeds >= 99%')
    for t in TAU_SWEEP:
        for o in TAU_OVERLAPS:
            c = tm[(t, o)]
            print(f'    {t:6.0f}  {100 * o:5.0f}%   {100 * c.mean():6.2f} ± {100 * c.std():4.2f}'
                  f'      {int((c >= 0.99).sum()):2d}/{N_TAU_MC}')
    print(f'  [Safety stress test: {N_SIGMA_MC} seeds per sigma, CRN, '
          f'{nom["t_flight"]:.0f} s mission]')
    print('    sigma(m)  mode     viol steps mean   max z mean   deck strikes   '
          'paired vs PD only (more / fewer / equal violations)')
    for s in SIGMA_SWEEP:
        for mode in OVERRIDE_MODES:
            r = sweep[(s, mode)]
            line = (f"    {s:6.2f}    {mode:<7}  {r['n_viol'].mean():9.1f}        "
                    f"{r['zmax'].mean():.4f}       {r['crashes']:2d}/{N_SIGMA_MC}")
            if mode != 'off':
                d = r['n_viol'] - sweep[(s, 'off')]['n_viol']
                line += (f"          {int((d > 0).sum()):2d} / {int((d < 0).sum()):2d} / "
                         f"{int((d == 0).sum()):2d}")
            print(line)
    z_eq = (K_EMG * Z_MAX - KP * Z_NOM) / (K_EMG - KP)
    print(f'  Override vs PD restoring acceleration: equal at z = {z_eq:.3f} m '
          f'(above the deck), so the override is weaker for all z < {Z_DECK} m '
          f'(at z = 7.9 m: override {-K_EMG * (7.9 - Z_MAX):.1f} m/s^2, '
          f'PD {KP * (Z_NOM - 7.9):.1f} m/s^2, noise-free, vz = 0)')

    out = os.path.normpath(OUTPUT_DIR)
    os.makedirs(out, exist_ok=True)
    plot_flight_path_3d(nom, out)
    plot_coverage_map(nom, out)
    plot_height_series(nom, out)
    plot_strip_coverage(nom, out)
    plot_overlap_study(mc, out)
    plot_safety_sweep(sweep, out)
    plot_tau_sensitivity(res['tau_mc'], out)
    save_animation(nom, out)
