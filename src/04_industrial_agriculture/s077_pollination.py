"""
S077 Precision Pollination Flight Pattern
=========================================
A drone visits F = 25 flowers placed at random in a 10 x 10 x 2 m orchard volume
(z in [1, 3] m) in nearest-neighbour TSP order. For every flower it climbs vertically to
the 3.5 m transit altitude, flies over, descends onto a 30 deg approach line that ends
0.8 m below-and-behind the flower, approaches it, docks for 1.5 s, and departs upward.
Two approach references are compared:
  1. Minimum-snap degree-7 polynomial (arrives at rest with zero acceleration and jerk)
  2. Straight line at constant 0.5 m/s (the reference stops abruptly at the flower)
The drone is a 3D point mass with a first-order velocity response; it tracks the reference
with velocity (and optionally acceleration) feedforward plus a P loop on GPS positions
(sigma = 0.1 m per axis). Docking error = mean true distance to the flower during the dwell;
transfer probability P_t = exp(-e_dock / sigma_dock). Both methods share the flower layout
and the GPS-noise samples (common random numbers).

Usage:
    conda activate drones
    python src/04_industrial_agriculture/s077_pollination.py
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401

# ── Parameters (card) ─────────────────────────────────────────────────────────
GPS_ERROR   = 0.1        # m   per-axis GPS noise sigma
F_FLOWERS   = 25
ORCHARD_X   = 10.0       # m
ORCHARD_Y   = 10.0       # m
Z_MIN, Z_MAX = 1.0, 3.0  # m   flower heights
D_DOCK      = 0.1        # m   docking radius
SIGMA_DOCK  = D_DOCK     # m   transfer-probability scale
T_DWELL     = 1.5        # s
D_STANDOFF  = 0.8        # m
ELEV_ANGLE  = 30.0       # deg approach elevation above horizontal
V_APPROACH  = 0.5        # m/s approach entry speed
V_TRANSIT   = 1.5        # m/s transit speed
Z_TRANSIT   = 3.5        # m
DT          = 0.05       # s
SEED        = 42

# ── Added parameters ──────────────────────────────────────────────────────────
TAU_V       = 0.3        # s   first-order velocity response time constant
KP          = 2.0        # 1/s position-error gain of the tracking loop
V_MAX       = 3.0        # m/s commanded-speed limit
T_HOVER_VIA = 1.0        # s   minimum hover above the approach point before the descent
N_MC        = 100        # Monte Carlo seeds (layout + GPS noise)
SIGMA_SWEEP = [0.0, 0.05, 0.1, 0.2]   # m GPS-noise sweep
N_SWEEP     = 40         # seeds per sweep point
ANIM_STEP   = 40         # simulation steps per GIF frame (2 s)

T_SEG = max(D_STANDOFF / V_APPROACH, 0.5)          # s approach duration (card rule) = 1.6 s
N_SEG = int(round(T_SEG / DT))
N_DWELL = int(round(T_DWELL / DT))
PEAK_FACTOR = 35.0 / 16.0                          # peak/mean speed of a rest-to-rest septic
Z_START_MIN = Z_MIN - D_STANDOFF * np.sin(np.deg2rad(ELEV_ANGLE))
N_DESC_MAX = int(np.ceil(PEAK_FACTOR * (Z_TRANSIT - Z_START_MIN) / V_TRANSIT / DT))
N_PAD_MIN = int(round(T_HOVER_VIA / DT))
N_WIN = N_PAD_MIN + N_DESC_MAX + N_SEG + N_DWELL   # steps in one docking window
I_APP = N_PAD_MIN + N_DESC_MAX                     # first approach step in a window
I_DWELL = I_APP + N_SEG                            # first dwell step

START_POS = np.array([0.0, 0.0, Z_TRANSIT])
METHODS = ['min_snap', 'straight']
LABELS = {'min_snap': 'Minimum-snap approach', 'straight': 'Straight-line approach'}
COLORS = {'min_snap': '#D62728', 'straight': '#1F77B4'}
FLOWER_COLOR = '#2CA02C'

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), '..', '..',
                          'outputs', '04_industrial_agriculture', 's077_pollination')


# ── Planning ──────────────────────────────────────────────────────────────────

def make_flowers(rng):
    return np.column_stack([rng.uniform(0, ORCHARD_X, F_FLOWERS),
                            rng.uniform(0, ORCHARD_Y, F_FLOWERS),
                            rng.uniform(Z_MIN, Z_MAX, F_FLOWERS)])


def tsp_nearest_neighbour(points, start):
    left, tour, cur = list(range(len(points))), [], start
    while left:
        j = left[int(np.argmin([np.linalg.norm(points[i] - cur) for i in left]))]
        tour.append(j)
        cur = points[j]
        left.remove(j)
    return tour


def approach_direction(flower, from_pos):
    """Unit travel direction of the approach: 30 deg above horizontal, bearing toward the flower."""
    d = flower[:2] - from_pos[:2]
    phi = np.arctan2(d[1], d[0])
    e = np.deg2rad(ELEV_ANGLE)
    return np.array([np.cos(e) * np.cos(phi), np.cos(e) * np.sin(phi), np.sin(e)])


def boundary_matrix(T):
    """Rows p, v, a, j at t = 0 and t = T for a degree-7 polynomial (card's A_bc)."""
    A = np.zeros((8, 8))
    for r in range(4):
        A[r, r] = np.prod(np.arange(1, r + 1))
        for i in range(r, 8):
            A[4 + r, i] = np.prod(np.arange(i - r + 1, i + 1)) * T ** (i - r)
    return A


def snap_poly(p0, p1, v0, v1, T):
    """Per-axis coefficients (3 x 8): given end positions/velocities, zero acc/jerk at both ends."""
    z = np.zeros(3)
    B = np.stack([p0, v0, z, z, p1, v1, z, z])
    return np.linalg.solve(boundary_matrix(T), B).T


def snap_cost(c, T):
    """J = integral of the squared 4th derivative, summed over axes (card's Q matrix)."""
    Q = np.zeros((8, 8))
    for i in range(4, 8):
        for j in range(4, 8):
            k = i + j - 7
            Q[i, j] = (np.prod(np.arange(i - 3, i + 1)) * np.prod(np.arange(j - 3, j + 1))
                       * T ** k / k)
    return float(sum(ci @ Q @ ci for ci in c))


def eval_poly(c, t, d=0):
    """d-th derivative at times t; returns (len(t), 3)."""
    out = np.zeros((len(t), 3))
    for i in range(d, 8):
        out += np.outer(t ** (i - d), c[:, i] * np.prod(np.arange(i - d + 1, i + 1)))
    return out


def poly_leg(p0, p1, v0, v1, n):
    """n steps of a degree-7 leg (positions, velocities, accelerations at t = DT..n DT)."""
    c = snap_poly(p0, p1, v0, v1, n * DT)
    t = np.arange(1, n + 1) * DT
    return eval_poly(c, t), eval_poly(c, t, 1), eval_poly(c, t, 2), c


def leg_steps(dist, v_peak):
    return max(1, int(np.ceil(PEAK_FACTOR * dist / v_peak / DT)))


def plan_tour(flowers):
    """Visit order, approach directions, approach-start and via points."""
    tour = tsp_nearest_neighbour(flowers, START_POS)
    cur, legs = START_POS, []
    for k in tour:
        f = flowers[k]
        d_hat = approach_direction(f, cur)
        p_start = f - D_STANDOFF * d_hat
        legs.append(dict(k=k, flower=f, d_hat=d_hat, p_start=p_start,
                         via=np.array([p_start[0], p_start[1], Z_TRANSIT]), prev=cur.copy()))
        cur = f
    return tour, legs


def window_reference(leg, method):
    """Reference (P, V, A) for one docking window: hover at via, descend onto the approach
    line (arriving with velocity V_APPROACH * d_hat), approach, dwell. Fixed length N_WIN."""
    f, d_hat, p_start, via = leg['flower'], leg['d_hat'], leg['p_start'], leg['via']
    n_desc = leg_steps(np.linalg.norm(p_start - via), V_TRANSIT)
    assert n_desc <= N_DESC_MAX
    n_pad = N_WIN - N_DWELL - N_SEG - n_desc
    P = np.repeat(via[None], n_pad, 0); V = np.zeros((n_pad, 3)); A = np.zeros((n_pad, 3))
    pd, vd, ad, _ = poly_leg(via, p_start, np.zeros(3), V_APPROACH * d_hat, n_desc)
    t = np.arange(1, N_SEG + 1) * DT
    if method == 'min_snap':
        c = snap_poly(p_start, f, V_APPROACH * d_hat, np.zeros(3), T_SEG)
        pa, va, aa = eval_poly(c, t), eval_poly(c, t, 1), eval_poly(c, t, 2)
        J = snap_cost(c, T_SEG)
    else:
        pa = p_start + np.outer(t, V_APPROACH * d_hat)
        va = np.repeat((V_APPROACH * d_hat)[None], N_SEG, 0)
        aa = np.zeros((N_SEG, 3))
        va[-1] = 0.0                                   # reference stops at the flower
        J = np.nan
    assert np.allclose(pa[-1], f), 'approach must end at the flower'
    P = np.vstack([P, pd, pa, np.repeat(f[None], N_DWELL, 0)])
    V = np.vstack([V, vd, va, np.zeros((N_DWELL, 3))])
    A = np.vstack([A, ad, aa, np.zeros((N_DWELL, 3))])
    return P, V, A, J


# ── Simulation ────────────────────────────────────────────────────────────────

def fly_batch(P, V, A, noise, ff_acc):
    """Track N references at once. P, V, A, noise: (N, L, 3). Starts at rest on P[:, 0]."""
    N, L, _ = P.shape
    p = np.zeros((N, L, 3)); v = np.zeros((N, L, 3))
    p[:, 0] = P[:, 0]
    a = 1.0 - np.exp(-DT / TAU_V)
    for k in range(L - 1):
        v_cmd = V[:, k + 1] + KP * (P[:, k + 1] - (p[:, k] + noise[:, k]))
        if ff_acc:
            v_cmd = v_cmd + TAU_V * A[:, k + 1]
        sp = np.linalg.norm(v_cmd, axis=1, keepdims=True)
        v_cmd = np.where(sp > V_MAX, v_cmd * V_MAX / np.maximum(sp, 1e-12), v_cmd)
        v[:, k + 1] = v[:, k] + a * (v_cmd - v[:, k])
        p[:, k + 1] = p[:, k] + v[:, k + 1] * DT
    assert np.all(p[..., 2] >= 0.0), 'drone below ground'
    assert np.all(np.linalg.norm(v, axis=2) <= V_MAX + 1e-9), 'speed above V_MAX'
    assert np.all(np.linalg.norm(np.diff(p, axis=1), axis=2) <= V_MAX * DT + 1e-9), 'teleport'
    return p, v


def evaluate(seeds, sigma=GPS_ERROR, ff_acc=True, keep=False):
    """Simulate every docking window of every seed for both methods (same noise)."""
    refs = {m: [] for m in METHODS}
    noises, layouts, Js = [], [], []
    for s in seeds:
        rng = np.random.default_rng(s)
        flowers = make_flowers(rng)
        tour, legs = plan_tour(flowers)
        layouts.append((flowers, tour, legs))
        noises.append(sigma * rng.standard_normal((F_FLOWERS, N_WIN, 3)))
        for m in METHODS:
            for lg in legs:
                P, V, A, J = window_reference(lg, m)
                refs[m].append((P, V, A))
                if m == 'min_snap':
                    Js.append(J)
    noise = np.concatenate(noises)
    out = {}
    for m in METHODS:
        P = np.stack([r[0] for r in refs[m]]); V = np.stack([r[1] for r in refs[m]])
        A = np.stack([r[2] for r in refs[m]])
        p, v = fly_batch(P, V, A, noise, ff_acc)
        fl = P[:, -1]
        err = p[:, I_DWELL:] - fl[:, None]
        e_dock = np.mean(np.linalg.norm(err, axis=2), axis=1).reshape(len(seeds), F_FLOWERS)
        d_hat = np.stack([lg['d_hat'] for lay in layouts for lg in lay[2]])
        over = np.maximum(0.0, np.max(np.einsum('nlk,nk->nl', err, d_hat), axis=1))
        P_t = np.exp(-e_dock / SIGMA_DOCK)
        out[m] = dict(e_dock=e_dock, P_t=P_t, E_N=P_t.sum(axis=1),
                      overshoot=over.reshape(len(seeds), F_FLOWERS),
                      v_arrive=np.linalg.norm(v[:, I_DWELL - 1], axis=1)
                      .reshape(len(seeds), F_FLOWERS),
                      peak_ref=float(np.max(np.linalg.norm(V[:, I_APP:I_DWELL], axis=2))))
        if keep:
            out[m].update(p=p, v=v, P=P)
    out['layouts'] = layouts
    out['snap_J'] = np.array(Js)
    return out


def mission_trajectory(win_p, legs):
    """Chain the flown docking windows with kinematic climb / transit legs (for plots, GIF)."""
    segs, cur, t_total = [START_POS[None]], START_POS.copy(), 0.0
    dwell_end = []
    for w, lg in zip(win_p, legs):
        if cur[2] < Z_TRANSIT - 1e-9:
            top = np.array([cur[0], cur[1], Z_TRANSIT])
            n = leg_steps(Z_TRANSIT - cur[2], V_TRANSIT)
            segs.append(poly_leg(cur, top, np.zeros(3), np.zeros(3), n)[0])
            cur = top
        n = leg_steps(np.linalg.norm(lg['via'] - cur), V_TRANSIT)
        segs.append(poly_leg(cur, lg['via'], np.zeros(3), np.zeros(3), n)[0])
        segs.append(w[1:])
        dwell_end.append(sum(len(s) for s in segs))
        cur = w[-1].copy()
    traj = np.vstack(segs)
    steps = np.linalg.norm(np.diff(traj, axis=0), axis=1)
    assert np.all(steps <= V_MAX * DT + 1e-9), 'teleport in mission trajectory'
    assert np.all(traj[:, 2] >= 0.0)
    return traj, np.array(dwell_end)


def run_simulation():
    main = evaluate([SEED], keep=True)
    noiseless = evaluate([SEED], sigma=0.0, keep=True)
    noiseless_noff = evaluate([SEED], sigma=0.0, ff_acc=False, keep=True)
    mc = evaluate(range(N_MC))
    mc_noff = evaluate(range(N_MC), ff_acc=False)
    sweep = {m: [] for m in METHODS}
    for sg in SIGMA_SWEEP:
        r = evaluate(range(N_SWEEP), sigma=sg)
        for m in METHODS:
            sweep[m].append(r[m]['E_N'])
    sweep = {m: np.array(v) for m, v in sweep.items()}
    flowers, tour, legs = main['layouts'][0]
    traj = {}
    for m in METHODS:
        traj[m] = mission_trajectory(main[m]['p'], legs)
    return main, noiseless, noiseless_noff, mc, mc_noff, sweep, traj


# ── Plotting ──────────────────────────────────────────────────────────────────

def _save(fig, out_dir, name, dpi=150):
    path = os.path.join(out_dir, name)
    fig.savefig(path, dpi=dpi, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_tour_3d(main, traj, out_dir):
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    fl = main['layouts'][0][0]
    xx, yy = np.meshgrid([0, ORCHARD_X], [0, ORCHARD_Y])
    ax.plot_surface(xx, yy, np.full_like(xx, Z_TRANSIT), color='grey', alpha=0.12)
    for m, ls in (('min_snap', '-'), ('straight', '--')):
        p = traj[m][0]
        ax.plot(p[:, 0], p[:, 1], p[:, 2], ls, color=COLORS[m], lw=0.9, label=LABELS[m])
    ax.scatter(fl[:, 0], fl[:, 1], fl[:, 2], color=FLOWER_COLOR, s=40, edgecolor='k',
               depthshade=False, label='Flowers')
    ax.scatter(*START_POS, marker='*', color='k', s=150, label='Start')
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)'); ax.set_zlabel('z (m)')
    ax.set_zlim(0, 4)
    ax.set_title(f'S077 Pollination Tour (seed {SEED}); grey plane = transit altitude '
                 f'{Z_TRANSIT} m')
    ax.legend(fontsize=8, loc='upper left')
    ax.view_init(elev=25, azim=-60)
    _save(fig, out_dir, 'tour_3d.png')


def plot_tsp_map(main, out_dir):
    flowers, tour, legs = main['layouts'][0]
    fig, ax = plt.subplots(figsize=(7.5, 7))
    pts = np.vstack([START_POS, flowers[tour]])
    for a, b in zip(pts[:-1], pts[1:]):
        ax.annotate('', xy=b[:2], xytext=a[:2],
                    arrowprops=dict(arrowstyle='->', color='grey', lw=1.0))
    Pt = np.empty(F_FLOWERS)
    Pt[tour] = main['min_snap']['P_t'][0]
    sc = ax.scatter(flowers[:, 0], flowers[:, 1], c=Pt, cmap='RdYlGn', vmin=0, vmax=1, s=120,
                    edgecolor='k', zorder=3)
    for order, k in enumerate(tour):
        ax.annotate(str(order + 1), flowers[k, :2], xytext=(5, 5), textcoords='offset points',
                    fontsize=7)
    for lg in legs:
        ax.plot([lg['p_start'][0], lg['flower'][0]], [lg['p_start'][1], lg['flower'][1]],
                color=COLORS['min_snap'], lw=2)
    ax.plot(*START_POS[:2], '*', color='k', ms=15)
    fig.colorbar(sc, ax=ax, label='$P_t$ (minimum-snap)')
    ax.set_xlim(-1, ORCHARD_X + 0.5); ax.set_ylim(-1, ORCHARD_Y + 0.5)
    ax.set_aspect('equal')
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')
    tl = np.linalg.norm(np.diff(pts, axis=0), axis=1).sum()
    ax.set_title(f'S077 Nearest-Neighbour Tour ({tl:.1f} m start-to-last flower)\n'
                 f'numbers = visit order, red ticks = 0.8 m approach lines', fontsize=10)
    _save(fig, out_dir, 'tsp_map.png')


def plot_dock_error_bars(main, out_dir):
    fig, ax = plt.subplots(figsize=(12, 4.5))
    x = np.arange(1, F_FLOWERS + 1)
    for j, m in enumerate(METHODS):
        e = main[m]['e_dock'][0]
        ax.bar(x + (j - 0.5) * 0.4, e, 0.4, color=COLORS[m],
               label=f'{LABELS[m]} (mean {e.mean():.4f} m)')
    ax.axhline(D_DOCK, color='k', ls='--', lw=1, label=f'$d_{{dock}}$ = {D_DOCK} m')
    ax.set_xlabel('Visit number'); ax.set_ylabel('Docking error $e_{dock}$ (m)')
    ax.set_xticks(x)
    ax.set_title(f'S077 Docking Error per Flower (seed {SEED}, GPS σ = {GPS_ERROR} m)')
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3, axis='y')
    _save(fig, out_dir, 'dock_error_per_flower.png')


def plot_transfer_prob(main, out_dir):
    fig, ax = plt.subplots(figsize=(10, 4.5))
    x = np.arange(1, F_FLOWERS + 1)
    for m in METHODS:
        ax.plot(x, main[m]['P_t'][0], 'o-', color=COLORS[m],
                label=f"{LABELS[m]}: E[N_poll] = {main[m]['E_N'][0]:.2f}")
    ax.set_xlabel('Visit number'); ax.set_ylabel('Transfer probability $P_t$')
    ax.set_ylim(0, 1)
    ax.set_title(f'S077 Pollen Transfer Probability per Visit (seed {SEED})')
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)
    _save(fig, out_dir, 'transfer_prob.png')


def plot_dock_error_hist(mc, out_dir):
    fig, ax = plt.subplots(figsize=(8, 4.5))
    bins = np.linspace(0, 0.15, 61)
    for m in METHODS:
        e = mc[m]['e_dock'].ravel()
        ax.hist(e, bins=bins, color=COLORS[m], alpha=0.55,
                label=f'{LABELS[m]} (median {np.median(e):.4f} m)')
    ax.axvline(D_DOCK, color='k', ls='--', lw=1, label=f'$d_{{dock}}$ = {D_DOCK} m')
    ax.set_xlabel('Docking error (m)'); ax.set_ylabel('Number of dockings')
    ax.set_title(f'S077 Docking Error Distribution ({N_MC} seeds × {F_FLOWERS} flowers)')
    ax.legend(fontsize=8)
    _save(fig, out_dir, 'dock_error_hist.png')


def plot_docking_transient(main, noiseless, noiseless_noff, out_dir, visit=0):
    fig, axes = plt.subplots(1, 3, figsize=(17, 4.6), sharey=True)
    cases = ((noiseless_noff, 'GPS noise off, no acceleration feedforward'),
             (noiseless, 'GPS noise off, with acceleration feedforward'),
             (main, f'GPS σ = {GPS_ERROR} m, with acceleration feedforward'))
    t = (np.arange(N_WIN) - I_APP) * DT
    sl = slice(I_APP - 10, N_WIN)
    for ax, (res, ttl) in zip(axes, cases):
        for m in METHODS:
            err = np.linalg.norm(res[m]['p'][visit] - res[m]['P'][visit, -1], axis=1)
            ax.plot(t[sl], err[sl], color=COLORS[m], label=LABELS[m])
        ax.axvspan(T_SEG, T_SEG + T_DWELL, color='green', alpha=0.08, label='dwell')
        ax.axhline(D_DOCK, color='k', ls='--', lw=1)
        ax.set_yscale('log')
        ax.set_ylim(1e-3, 1.5)
        ax.set_xlabel('Time since approach start (s)')
        ax.set_title(ttl, fontsize=10)
        ax.grid(alpha=0.3, which='both')
    axes[0].set_ylabel('Distance to flower (m, log)')
    axes[0].legend(fontsize=8)
    fig.suptitle(f'S077 Docking Transient, visit 1 (dashed = $d_{{dock}}$)')
    plt.tight_layout()
    _save(fig, out_dir, 'docking_transient.png')


def plot_monte_carlo(mc, mc_noff, sweep, out_dir):
    fig, axes = plt.subplots(1, 3, figsize=(18, 5))
    for ax, res, ttl in ((axes[0], mc, 'with acceleration feedforward'),
                         (axes[1], mc_noff, 'without acceleration feedforward')):
        es, el = res['min_snap']['E_N'], res['straight']['E_N']
        ax.scatter(el, es, s=18, color='k', alpha=0.7)
        lo, hi = min(es.min(), el.min()) - 0.2, max(es.max(), el.max()) + 0.2
        ax.plot([lo, hi], [lo, hi], 'k--', lw=1, label='equal')
        ax.set_xlabel('E[N_poll], straight-line'); ax.set_ylabel('E[N_poll], minimum-snap')
        ax.set_title(f'{ttl}: min-snap wins {int(np.sum(es > el))}/{N_MC} paired seeds',
                     fontsize=10)
        ax.legend(fontsize=8); ax.grid(alpha=0.3)
    for m in METHODS:
        axes[2].errorbar(SIGMA_SWEEP, sweep[m].mean(1), yerr=sweep[m].std(1), fmt='o-',
                         capsize=4, color=COLORS[m], label=LABELS[m])
    axes[2].set_xlabel('GPS noise σ per axis (m)')
    axes[2].set_ylabel(f'E[N_poll] (of {F_FLOWERS})')
    axes[2].set_title(f'GPS-noise sweep, with feedforward ({N_SWEEP} seeds, mean ± std)',
                      fontsize=10)
    axes[2].axvline(GPS_ERROR, color='grey', ls=':')
    axes[2].legend(fontsize=8); axes[2].grid(alpha=0.3)
    fig.suptitle('S077 Monte Carlo Comparison (same layouts and GPS noise for both methods)')
    plt.tight_layout()
    _save(fig, out_dir, 'monte_carlo.png')


def plot_summary(mc, mc_noff, out_dir):
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(12, 4.8))
    groups = (('with FF', mc), ('without FF', mc_noff))
    x = np.arange(2)
    for j, m in enumerate(METHODS):
        for ax, key, scale in ((a1, 'E_N', 1), (a2, 'rate', 100)):
            vals = [(r[m]['E_N'] / (F_FLOWERS if key == 'rate' else 1)) * scale
                    for _, r in groups]
            mu = [v.mean() for v in vals]; sd = [v.std() for v in vals]
            bars = ax.bar(x + (j - 0.5) * 0.38, mu, 0.38, yerr=sd, capsize=5, color=COLORS[m],
                          edgecolor='k', label=LABELS[m])
            for b, mm, ss in zip(bars, mu, sd):
                ax.text(b.get_x() + b.get_width() / 2, mm / 2, f'{mm:.2f}\n± {ss:.2f}',
                        ha='center', fontsize=8,
                        bbox=dict(boxstyle='round', fc='white', alpha=0.85))
    for ax, ttl in ((a1, f'E[N_poll] (of {F_FLOWERS})'), (a2, 'Transfer success rate (%)')):
        ax.set_xticks(x)
        ax.set_xticklabels(['with acceleration\nfeedforward', 'without acceleration\nfeedforward'])
        ax.set_title(ttl)
        ax.grid(alpha=0.3, axis='y')
        ax.legend(fontsize=8, loc='lower right')
    fig.suptitle(f'S077 Summary over {N_MC} seeds (mean ± std), GPS σ = {GPS_ERROR} m')
    plt.tight_layout()
    _save(fig, out_dir, 'summary_bar.png')


def save_animation(main, traj, out_dir):
    fig = plt.figure(figsize=(8, 6.5))
    ax = fig.add_subplot(111, projection='3d')
    flowers, tour, _ = main['layouts'][0]
    n = min(len(traj[m][0]) for m in METHODS)
    n_frames = int(np.ceil((n - 1) / ANIM_STEP)) + 1
    ax.set_xlim(0, ORCHARD_X); ax.set_ylim(0, ORCHARD_Y); ax.set_zlim(0, 4)
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)'); ax.set_zlabel('z (m)')
    ax.view_init(elev=25, azim=-60)
    fl_sc = ax.scatter(flowers[:, 0], flowers[:, 1], flowers[:, 2], color='lightgrey', s=40,
                       edgecolor='k', depthshade=False)
    trails = {m: ax.plot([], [], [], color=COLORS[m], lw=1, label=LABELS[m])[0] for m in METHODS}
    dots = {m: ax.plot([], [], [], 'o', color=COLORS[m], mec='k')[0] for m in METHODS}
    ax.legend(fontsize=8, loc='upper left')
    title = ax.set_title('')

    def update(f):
        i = min(f * ANIM_STEP, n - 1)
        txt = []
        done = 0
        for m in METHODS:
            p, dwell_end = traj[m]
            trails[m].set_data(p[:i + 1, 0], p[:i + 1, 1])
            trails[m].set_3d_properties(p[:i + 1, 2])
            dots[m].set_data([p[i, 0]], [p[i, 1]])
            dots[m].set_3d_properties([p[i, 2]])
            done = int(np.sum(dwell_end <= i + 1))
            txt.append(f"{'snap' if m == 'min_snap' else 'line'} "
                       f"{main[m]['P_t'][0][:done].sum():.2f}")
        cols = np.array(['lightgrey'] * F_FLOWERS, dtype=object)
        cols[tour[:done]] = FLOWER_COLOR
        fl_sc.set_facecolor(list(cols))
        title.set_text(f'S077 t = {i * DT:.0f} s, visited {done}/{F_FLOWERS}\n'
                       f'running E[N_poll]: ' + ', '.join(txt))
        return []

    anim = animation.FuncAnimation(fig, update, frames=n_frames, interval=80, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    anim.save(path, writer='pillow', fps=12, dpi=80)
    plt.close(fig)
    print(f'Saved: {path} ({n_frames} frames)')


# ── Main ──────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    main, noiseless, noiseless_noff, mc, mc_noff, sweep, traj = run_simulation()
    print('=== S077 Precision Pollination — Results ===')
    d = approach_direction(np.array([1.0, 0.0, 0.0]), np.zeros(3))
    card_d = np.array([0.5, 0.0, np.sqrt(3) / 2])
    print(f'  Approach elevation: implemented {np.degrees(np.arcsin(d[2])):.1f} deg; card vector '
          f'(0.5 cos, 0.5 sin, 0.866) gives {np.degrees(np.arcsin(card_d[2])):.1f} deg')
    print(f'  Approach duration {T_SEG:.2f} s for both methods; docking window {N_WIN * DT:.2f} s')
    flowers, tour, legs = main['layouts'][0]
    pts = np.vstack([START_POS, flowers[tour]])
    print(f'  Seed {SEED}: NN tour length {np.linalg.norm(np.diff(pts, axis=0), axis=1).sum():.2f} m; '
          f'mission {(len(traj["min_snap"][0]) - 1) * DT:.2f} s (min-snap) / '
          f'{(len(traj["straight"][0]) - 1) * DT:.2f} s (straight)')
    zmin = min(traj[m][0][:, 2].min() for m in METHODS)
    print(f'  Seed {SEED}: lowest flown altitude {zmin:.2f} m (descent curls under the approach '
          f'point to enter the 30 deg line); lowest approach point '
          f'{min(lg["p_start"][2] for lg in legs):.2f} m')
    print(f'  Approach reference peak speed: min-snap {main["min_snap"]["peak_ref"]:.3f} m/s, '
          f'straight {main["straight"]["peak_ref"]:.3f} m/s; mean min-snap snap cost '
          f'{np.mean(main["snap_J"]):.2f} m^2/s^7')
    for res, tag in ((noiseless_noff, 'noise off, no FF'), (noiseless, 'noise off, FF'),
                     (main, f'GPS sigma {GPS_ERROR} m, FF')):
        print(f'  --- seed {SEED}, {tag} ---')
        for m in METHODS:
            o = res[m]
            print(f'  [{LABELS[m]}] E[N_poll] {o["E_N"][0]:.2f}/{F_FLOWERS}, mean e_dock '
                  f'{o["e_dock"].mean():.4f} m, mean overshoot {o["overshoot"].mean():.4f} m, '
                  f'max overshoot {o["overshoot"].max():.4f} m, speed at dwell start '
                  f'{o["v_arrive"].mean():.3f} m/s')
    for res, tag in ((mc, 'with acceleration feedforward'), (mc_noff, 'without feedforward')):
        print(f'  --- Monte Carlo, {N_MC} seeds, GPS sigma {GPS_ERROR} m, {tag} ---')
        for m in METHODS:
            o = res[m]
            e = o['e_dock'].ravel()
            print(f'  [{LABELS[m]}] E[N_poll] {o["E_N"].mean():.2f} ± {o["E_N"].std():.2f} '
                  f'(rate {100 * o["E_N"].mean() / F_FLOWERS:.1f}%), mean e_dock {e.mean():.4f} m '
                  f'(median {np.median(e):.4f}), mean overshoot {o["overshoot"].mean():.4f} m, '
                  f'e_dock < d_dock in {100 * np.mean(e < D_DOCK):.1f}%')
        diff = res['min_snap']['E_N'] - res['straight']['E_N']
        print(f'  Paired difference (snap - straight): {diff.mean():+.3f} ± {diff.std():.3f}; '
              f'min-snap wins {int(np.sum(diff > 0))}/{N_MC}, ties {int(np.sum(diff == 0))}')
    print(f'  Card error model (e_dock = mean |eta|): E|eta| = '
          f'{GPS_ERROR * 2 * np.sqrt(2 / np.pi):.4f} m -> P_t = '
          f'{np.exp(-GPS_ERROR * 2 * np.sqrt(2 / np.pi) / SIGMA_DOCK):.3f} for both methods')
    print(f'  GPS-noise sweep with FF ({N_SWEEP} seeds, E[N_poll] mean ± std):')
    for i, sg in enumerate(SIGMA_SWEEP):
        s_, l_ = sweep['min_snap'][i], sweep['straight'][i]
        print(f'    sigma {sg:.2f} m: min-snap {s_.mean():.2f} ± {s_.std():.2f}, straight '
              f'{l_.mean():.2f} ± {l_.std():.2f}, min-snap wins {int(np.sum(s_ > l_))}/{N_SWEEP}')

    out = os.path.normpath(OUTPUT_DIR)
    os.makedirs(out, exist_ok=True)
    plot_tour_3d(main, traj, out)
    plot_tsp_map(main, out)
    plot_dock_error_bars(main, out)
    plot_transfer_prob(main, out)
    plot_dock_error_hist(mc, out)
    plot_docking_transient(main, noiseless, noiseless_noff, out)
    plot_monte_carlo(mc, mc_noff, sweep, out)
    plot_summary(mc, mc_noff, out)
    save_animation(main, traj, out)
