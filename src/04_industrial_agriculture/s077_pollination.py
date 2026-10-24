"""
S077 Precision Pollination Flight Pattern
=========================================
A drone visits F = 25 flowers placed at random in a 10 x 10 x 2 m orchard volume
(z in [1, 3] m) in nearest-neighbour TSP order. For every flower it climbs vertically to
the 3.5 m transit altitude, flies over, descends onto a 30 deg approach line that ends
0.8 m below-and-behind the flower, approaches it, docks for 1.5 s, and departs upward.
Three approach references are compared:
  1. Minimum-snap degree-7 polynomial (arrives at rest with zero acceleration and jerk)
  2. Straight line at constant 0.5 m/s, stopping abruptly at the flower (no feedforward
     of the stop)
  3. The same straight line with the stop fed forward as a one-step deceleration
The drone is a 3D point mass with a first-order velocity response and an acceleration
limit; it tracks the reference with velocity (and optionally acceleration) feedforward
plus a P loop on GPS positions (sigma = 0.1 m per axis). Docking error = mean true distance to the flower during the dwell;
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
A_MAX       = 2.0        # m/s^2 acceleration limit of the airframe (main configuration)
A_SWEEP     = [float('inf'), 4.0, 2.0, 1.5]   # m/s^2 acceleration-limit sweep
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
METHODS = ['min_snap', 'straight', 'straight_ff']
LABELS = {'min_snap': 'Minimum-snap approach', 'straight': 'Straight line (no FF at the stop)',
          'straight_ff': 'Straight line + stop-impulse FF'}
COLORS = {'min_snap': '#D62728', 'straight': '#1F77B4', 'straight_ff': '#17BECF'}
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
        if method == 'straight_ff':
            aa[-1] = -V_APPROACH * d_hat / DT          # the stop as a one-step deceleration
        J = np.nan
    assert np.allclose(pa[-1], f), 'approach must end at the flower'
    P = np.vstack([P, pd, pa, np.repeat(f[None], N_DWELL, 0)])
    V = np.vstack([V, vd, va, np.zeros((N_DWELL, 3))])
    A = np.vstack([A, ad, aa, np.zeros((N_DWELL, 3))])
    return P, V, A, J


# ── Simulation ────────────────────────────────────────────────────────────────

def fly_batch(P, V, A, noise, ff_acc, a_max):
    """Track N references at once. P, V, A, noise: (N, L, 3). Starts at rest on P[:, 0].
    The velocity follows the command with time constant TAU_V, with the acceleration
    magnitude limited to a_max (inf = no limit)."""
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
        dv = a * (v_cmd - v[:, k])
        if np.isfinite(a_max):
            n_dv = np.linalg.norm(dv, axis=1, keepdims=True)
            dv = np.where(n_dv > a_max * DT, dv * a_max * DT / np.maximum(n_dv, 1e-12), dv)
        v[:, k + 1] = v[:, k] + dv
        p[:, k + 1] = p[:, k] + v[:, k + 1] * DT
    assert np.all(p[..., 2] >= 0.0), 'drone below ground'
    assert np.all(np.linalg.norm(v, axis=2) <= V_MAX + 1e-9), 'speed above V_MAX'
    assert np.all(np.linalg.norm(np.diff(v, axis=1), axis=2) <= a_max * DT + 1e-9), 'accel limit'
    assert np.all(np.linalg.norm(np.diff(p, axis=1), axis=2) <= V_MAX * DT + 1e-9), 'teleport'
    return p, v


def evaluate(seeds, sigma=GPS_ERROR, ff_acc=True, a_max=A_MAX, keep=False):
    """Simulate every docking window of every seed for all methods (same noise)."""
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
    half = N_DWELL // 2
    for m in METHODS:
        P = np.stack([r[0] for r in refs[m]]); V = np.stack([r[1] for r in refs[m]])
        A = np.stack([r[2] for r in refs[m]])
        p, v = fly_batch(P, V, A, noise, ff_acc, a_max)
        fl = P[:, -1]
        err = p[:, I_DWELL:] - fl[:, None]
        dist = np.linalg.norm(err, axis=2)
        e_dock = np.mean(dist, axis=1).reshape(len(seeds), F_FLOWERS)
        d_hat = np.stack([lg['d_hat'] for lay in layouts for lg in lay[2]])
        over = np.maximum(0.0, np.max(np.einsum('nlk,nk->nl', err, d_hat), axis=1))
        P_t = np.exp(-e_dock / SIGMA_DOCK)
        out[m] = dict(e_dock=e_dock, P_t=P_t, E_N=P_t.sum(axis=1),
                      overshoot=over.reshape(len(seeds), F_FLOWERS),
                      e_first=float(dist[:, :half].mean()), e_second=float(dist[:, half:].mean()),
                      v_arrive=np.linalg.norm(v[:, I_DWELL - 1], axis=1)
                      .reshape(len(seeds), F_FLOWERS),
                      peak_ref=float(np.max(np.linalg.norm(V[:, I_APP:I_DWELL], axis=2))),
                      peak_ref_acc=float(np.max(np.linalg.norm(A[:, I_APP:I_DWELL], axis=2))),
                      peak_acc=float(np.max(np.linalg.norm(np.diff(v[:, I_APP - 1:], axis=1),
                                                           axis=2)) / DT))
        if keep:
            out[m].update(p=p, v=v, P=P)
    out['layouts'] = layouts
    out['snap_J'] = np.array(Js)
    return out


def mission_trajectory(win_p, legs):
    """Chain the flown docking windows with kinematic climb / transit legs (for plots, GIF)."""
    segs, cur = [START_POS[None]], START_POS.copy()
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


# Configurations: (acceleration feedforward, acceleration limit)
CONFIGS = {'main': (True, A_MAX), 'nolim': (True, np.inf), 'noff_lim': (False, A_MAX),
           'noff': (False, np.inf)}
CONFIG_LABELS = {'main': f'FF, |a| ≤ {A_MAX:g} m/s²', 'nolim': 'FF, no accel. limit',
                 'noff_lim': f'no FF, |a| ≤ {A_MAX:g} m/s²', 'noff': 'no FF, no accel. limit'}


def run_simulation():
    single = {c: evaluate([SEED], ff_acc=ff, a_max=am, keep=True) for c, (ff, am) in CONFIGS.items()}
    quiet = {c: evaluate([SEED], sigma=0.0, ff_acc=ff, a_max=am, keep=True)
             for c, (ff, am) in CONFIGS.items()}
    mc = {c: evaluate(range(N_MC), ff_acc=ff, a_max=am) for c, (ff, am) in CONFIGS.items()}
    sweep = {m: [] for m in METHODS}
    for sg in SIGMA_SWEEP:
        r = evaluate(range(N_SWEEP), sigma=sg)
        for m in METHODS:
            sweep[m].append(r[m]['E_N'])
    sweep = {m: np.array(v) for m, v in sweep.items()}
    asweep = {m: [] for m in METHODS}
    for am in A_SWEEP:
        r = evaluate(range(N_SWEEP), a_max=am)
        for m in METHODS:
            asweep[m].append(r[m]['E_N'])
    asweep = {m: np.array(v) for m, v in asweep.items()}
    noise_sweep_noff = {m: [] for m in METHODS}
    for sg in SIGMA_SWEEP:
        r = evaluate(range(N_SWEEP), sigma=sg, ff_acc=False, a_max=np.inf)
        for m in METHODS:
            noise_sweep_noff[m].append(r[m]['E_N'])
    noise_sweep_noff = {m: np.array(v) for m, v in noise_sweep_noff.items()}
    flowers, tour, legs = single['main']['layouts'][0]
    traj = {m: mission_trajectory(single['main'][m]['p'], legs) for m in METHODS}
    return single, quiet, mc, sweep, asweep, noise_sweep_noff, traj


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
    for m, ls in (('min_snap', '-'), ('straight', '--'), ('straight_ff', ':')):
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
    fig, ax = plt.subplots(figsize=(13, 4.5))
    x = np.arange(1, F_FLOWERS + 1)
    w = 0.27
    for j, m in enumerate(METHODS):
        e = main[m]['e_dock'][0]
        ax.bar(x + (j - 1) * w, e, w, color=COLORS[m],
               label=f'{LABELS[m]} (mean {e.mean():.4f} m)')
    ax.axhline(D_DOCK, color='k', ls='--', lw=1, label=f'$d_{{dock}}$ = {D_DOCK} m')
    ax.set_xlabel('Visit number'); ax.set_ylabel('Docking error $e_{dock}$ (m)')
    ax.set_xticks(x)
    ax.set_title(f'S077 Docking Error per Flower (seed {SEED}, GPS σ = {GPS_ERROR} m, '
                 f'{CONFIG_LABELS["main"]})')
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
    ax.set_title(f'S077 Pollen Transfer Probability per Visit (seed {SEED}, '
                 f'{CONFIG_LABELS["main"]})')
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)
    _save(fig, out_dir, 'transfer_prob.png')


def plot_dock_error_hist(mc, out_dir):
    fig, ax = plt.subplots(figsize=(8, 4.5))
    bins = np.linspace(0, 0.15, 61)
    for m in METHODS:
        e = mc[m]['e_dock'].ravel()
        ax.hist(e, bins=bins, color=COLORS[m], alpha=0.5,
                label=f'{LABELS[m]} (median {np.median(e):.4f} m)')
    ax.axvline(D_DOCK, color='k', ls='--', lw=1, label=f'$d_{{dock}}$ = {D_DOCK} m')
    ax.set_xlabel('Docking error (m)'); ax.set_ylabel('Number of dockings')
    ax.set_title(f'S077 Docking Error Distribution ({N_MC} seeds × {F_FLOWERS} flowers, '
                 f'{CONFIG_LABELS["main"]})', fontsize=10)
    ax.legend(fontsize=8)
    _save(fig, out_dir, 'dock_error_hist.png')


def plot_docking_transient(single, quiet, out_dir, visit=0):
    fig, axes = plt.subplots(2, 2, figsize=(14, 9), sharey=True, sharex=True)
    cases = ((quiet['noff'], 'GPS noise off, ' + CONFIG_LABELS['noff']),
             (quiet['nolim'], 'GPS noise off, ' + CONFIG_LABELS['nolim']),
             (quiet['main'], 'GPS noise off, ' + CONFIG_LABELS['main']),
             (single['main'], f'GPS σ = {GPS_ERROR} m, ' + CONFIG_LABELS['main']))
    t = (np.arange(N_WIN) - I_APP) * DT
    sl = slice(I_APP - 10, N_WIN)
    for ax, (res, ttl) in zip(axes.ravel(), cases):
        for m in METHODS:
            err = np.linalg.norm(res[m]['p'][visit] - res[m]['P'][visit, -1], axis=1)
            ax.plot(t[sl], err[sl], color=COLORS[m], label=LABELS[m],
                    ls='--' if m == 'straight_ff' else '-')
        ax.axvspan(T_SEG, T_SEG + T_DWELL, color='green', alpha=0.08, label='dwell')
        ax.axhline(D_DOCK, color='k', ls='--', lw=1)
        ax.set_yscale('log')
        ax.set_ylim(1e-3, 1.5)
        ax.set_title(ttl, fontsize=10)
        ax.grid(alpha=0.3, which='both')
    for ax in axes[1]:
        ax.set_xlabel('Time since approach start (s)')
    for ax in axes[:, 0]:
        ax.set_ylabel('Distance to flower (m, log)')
    axes[0, 0].legend(fontsize=8)
    fig.suptitle(f'S077 Docking Transient, visit 1 (dashed black = $d_{{dock}}$)')
    plt.tight_layout()
    _save(fig, out_dir, 'docking_transient.png')


def plot_monte_carlo(mc, sweep, asweep, out_dir):
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    for ax, key in ((axes[0, 0], 'main'), (axes[0, 1], 'nolim')):
        es, el = mc[key]['min_snap']['E_N'], mc[key]['straight_ff']['E_N']
        ax.scatter(el, es, s=18, color='k', alpha=0.7)
        lo, hi = min(es.min(), el.min()) - 0.2, max(es.max(), el.max()) + 0.2
        ax.plot([lo, hi], [lo, hi], 'k--', lw=1, label='equal')
        ax.set_xlabel('E[N_poll], straight line + stop impulse FF')
        ax.set_ylabel('E[N_poll], minimum-snap')
        ax.set_title(f'{CONFIG_LABELS[key]}: min-snap wins {int(np.sum(es > el))}/{N_MC}',
                     fontsize=10)
        ax.legend(fontsize=8); ax.grid(alpha=0.3)
    ax = axes[1, 0]
    xs = np.arange(len(A_SWEEP))
    for m in METHODS:
        ax.errorbar(xs, asweep[m].mean(1), yerr=asweep[m].std(1), fmt='o-', capsize=4,
                    color=COLORS[m], label=LABELS[m])
    ax.set_xticks(xs)
    ax.set_xticklabels(['none' if not np.isfinite(a) else f'{a:g}' for a in A_SWEEP])
    ax.set_xlabel('Acceleration limit (m/s²)'); ax.set_ylabel(f'E[N_poll] (of {F_FLOWERS})')
    ax.set_title(f'Acceleration-limit sweep, with FF, σ = {GPS_ERROR} m ({N_SWEEP} seeds)',
                 fontsize=10)
    ax.legend(fontsize=8); ax.grid(alpha=0.3)
    ax = axes[1, 1]
    for m in METHODS:
        ax.errorbar(SIGMA_SWEEP, sweep[m].mean(1), yerr=sweep[m].std(1), fmt='o-', capsize=4,
                    color=COLORS[m], label=LABELS[m])
    ax.set_xlabel('GPS noise σ per axis (m)'); ax.set_ylabel(f'E[N_poll] (of {F_FLOWERS})')
    ax.set_title(f'GPS-noise sweep, {CONFIG_LABELS["main"]} ({N_SWEEP} seeds)', fontsize=10)
    ax.axvline(GPS_ERROR, color='grey', ls=':')
    ax.legend(fontsize=8); ax.grid(alpha=0.3)
    fig.suptitle('S077 Monte Carlo Comparison (same layouts and GPS noise for every method)')
    plt.tight_layout()
    _save(fig, out_dir, 'monte_carlo.png')


def plot_summary(mc, out_dir):
    fig, ax = plt.subplots(figsize=(13, 5))
    keys = list(CONFIGS)
    x = np.arange(len(keys))
    w = 0.27
    for j, m in enumerate(METHODS):
        mu = [mc[k][m]['E_N'].mean() for k in keys]
        sd = [mc[k][m]['E_N'].std() for k in keys]
        bars = ax.bar(x + (j - 1) * w, mu, w, yerr=sd, capsize=4, color=COLORS[m],
                      edgecolor='k', label=LABELS[m])
        for b, mm in zip(bars, mu):
            ax.text(b.get_x() + b.get_width() / 2, mm / 2, f'{mm:.2f}', ha='center',
                    fontsize=8, rotation=90,
                    bbox=dict(boxstyle='round', fc='white', alpha=0.85))
    ax.set_xticks(x)
    ax.set_xticklabels([CONFIG_LABELS[k] for k in keys])
    ax.set_ylabel(f'E[N_poll] (of {F_FLOWERS}); rate = value / {F_FLOWERS}')
    ax.grid(alpha=0.3, axis='y')
    ax.legend(fontsize=8, loc='lower right')
    ax.set_title(f'S077 Summary over {N_MC} seeds (mean ± std), GPS σ = {GPS_ERROR} m; '
                 f'without FF the two straight-line variants are identical')
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
    ax.legend(fontsize=7, loc='upper left')
    title = ax.set_title('')
    short = {'min_snap': 'snap', 'straight': 'line', 'straight_ff': 'line+FF'}

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
            txt.append(f"{short[m]} {main[m]['P_t'][0][:done].sum():.2f}")
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

def _mc_lines(res, tag):
    print(f'  --- Monte Carlo, {N_MC} seeds, GPS sigma {GPS_ERROR} m, {tag} ---')
    for m in METHODS:
        o = res[m]
        e = o['e_dock'].ravel()
        print(f'  [{LABELS[m]}] E[N_poll] {o["E_N"].mean():.2f} ± {o["E_N"].std():.2f} '
              f'(rate {100 * o["E_N"].mean() / F_FLOWERS:.1f}%), mean e_dock {e.mean():.4f} m '
              f'(median {np.median(e):.4f}), mean overshoot {o["overshoot"].mean():.4f} m, '
              f'e_dock < d_dock in {100 * np.mean(e < D_DOCK):.1f}%')
    for other in ('straight', 'straight_ff'):
        diff = res['min_snap']['E_N'] - res[other]['E_N']
        print(f'  Paired min-snap - {other}: {diff.mean():+.3f} ± {diff.std():.3f}; min-snap wins '
              f'{int(np.sum(diff > 0))}/{N_MC}, ties {int(np.sum(diff == 0))}')


if __name__ == '__main__':
    single, quiet, mc, sweep, asweep, noise_noff, traj = run_simulation()
    main = single['main']
    print('=== S077 Precision Pollination — Results ===')
    d = approach_direction(np.array([1.0, 0.0, 0.0]), np.zeros(3))
    card_d = np.array([0.5, 0.0, np.sqrt(3) / 2])
    print(f'  Approach elevation: implemented {np.degrees(np.arcsin(d[2])):.1f} deg; card vector '
          f'(0.5 cos, 0.5 sin, 0.866) gives {np.degrees(np.arcsin(card_d[2])):.1f} deg')
    print(f'  Approach duration {T_SEG:.2f} s for every method; docking window {N_WIN * DT:.2f} s')
    flowers, tour, legs = main['layouts'][0]
    pts = np.vstack([START_POS, flowers[tour]])
    print(f'  Seed {SEED}: NN tour length {np.linalg.norm(np.diff(pts, axis=0), axis=1).sum():.2f} m; '
          f'mission ' + ' / '.join(f'{(len(traj[m][0]) - 1) * DT:.2f} s' for m in METHODS) +
          ' (min-snap / straight / straight+FF)')
    zmin = min(traj[m][0][:, 2].min() for m in METHODS)
    print(f'  Seed {SEED}: lowest flown altitude {zmin:.2f} m (descent curls under the approach '
          f'point to enter the 30 deg line); lowest approach point '
          f'{min(lg["p_start"][2] for lg in legs):.2f} m')
    print(f'  Approach reference peak speed: min-snap {main["min_snap"]["peak_ref"]:.3f} m/s, '
          f'straight {main["straight"]["peak_ref"]:.3f} m/s; peak reference acceleration: '
          f'min-snap {main["min_snap"]["peak_ref_acc"]:.2f} m/s^2, straight+FF stop impulse '
          f'{main["straight_ff"]["peak_ref_acc"]:.2f} m/s^2; mean min-snap snap cost '
          f'{np.mean(main["snap_J"]):.2f} m^2/s^7')
    for key in CONFIGS:
        for res, tag in ((quiet[key], 'noise off'), (single[key], f'GPS sigma {GPS_ERROR} m')):
            print(f'  --- seed {SEED}, {tag}, {CONFIG_LABELS[key]} ---')
            for m in METHODS:
                o = res[m]
                print(f'  [{LABELS[m]}] E[N_poll] {o["E_N"][0]:.2f}/{F_FLOWERS}, mean e_dock '
                      f'{o["e_dock"].mean():.4f} m (1st half of dwell {o["e_first"]:.4f}, 2nd half '
                      f'{o["e_second"]:.4f}), mean overshoot {o["overshoot"].mean():.4f} m, '
                      f'speed at dwell start {o["v_arrive"].mean():.3f} m/s, peak flown accel '
                      f'{o["peak_acc"]:.2f} m/s^2')
    for key in CONFIGS:
        _mc_lines(mc[key], CONFIG_LABELS[key])
    print(f'  Card error model (e_dock = mean |eta|): E|eta| = '
          f'{GPS_ERROR * 2 * np.sqrt(2 / np.pi):.4f} m -> P_t = '
          f'{np.exp(-GPS_ERROR * 2 * np.sqrt(2 / np.pi) / SIGMA_DOCK):.3f} for every method')
    for title_, sw, xs, fmt in ((f'GPS-noise sweep, {CONFIG_LABELS["main"]}', sweep, SIGMA_SWEEP,
                                 'sigma {:.2f} m'),
                                (f'Acceleration-limit sweep, FF, sigma {GPS_ERROR} m', asweep,
                                 A_SWEEP, 'a_max {:g} m/s^2'),
                                (f'GPS-noise sweep, {CONFIG_LABELS["noff"]}', noise_noff,
                                 SIGMA_SWEEP, 'sigma {:.2f} m')):
        print(f'  {title_} ({N_SWEEP} seeds, E[N_poll] mean ± std):')
        for i, xv in enumerate(xs):
            s_ = sw['min_snap'][i]
            parts = [f'{LABELS[m]} {sw[m][i].mean():.2f} ± {sw[m][i].std():.2f}' for m in METHODS]
            wins = [f'vs {m} {int(np.sum(s_ > sw[m][i]))}/{N_SWEEP}' for m in METHODS[1:]]
            print('    ' + fmt.format(xv) + ': ' + ', '.join(parts) + '; min-snap wins ' +
                  ', '.join(wins))

    out = os.path.normpath(OUTPUT_DIR)
    os.makedirs(out, exist_ok=True)
    plot_tour_3d(main, traj, out)
    plot_tsp_map(main, out)
    plot_dock_error_bars(main, out)
    plot_transfer_prob(main, out)
    plot_dock_error_hist(mc['main'], out)
    plot_docking_transient(single, quiet, out)
    plot_monte_carlo(mc, sweep, asweep, out)
    plot_summary(mc, out)
    save_animation(main, traj, out)
