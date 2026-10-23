"""
S046 3D Trilateration with Altitude
===================================
Five drones hover at known 3D anchor positions around a survivor's beacon
inside a 300 x 300 x 50 m disaster area.  Compared with the 2D-folder S046
(four drones at hand-picked tiers), this variant

  1. chooses the five drone positions (azimuth, horizontal radius, altitude)
     by minimising the 3D range GDOP with L-BFGS-B (altitude bounds, minimum
     altitude spread and inter-drone separation as penalties),
  2. adds TDOA (time-difference-of-arrival) localisation: a two-stage linear
     solver (unknowns x, y, z, rho_1) and a weighted Gauss-Newton refinement,
     with the correlated TDOA noise of a shared reference drone,
  3. tracks HDOP and VDOP separately and sweeps the allowed altitude band
     Delta z and the true beacon altitude z*.

Every estimator sees the same per-drone noise draws in each trial: the range
measurement is rho_i + e_i and the TDOA measurement is the difference of the
same arrival-time errors, (rho_i + e_i) - (rho_1 + e_1).

The range LLS / range Gauss-Newton / DOP helpers are imported from the 2D
script (src/03_environmental_sar/s046_trilateration.py) so the fixes made
there (LLS sign, GN Jacobian sign, coplanar height recovery) carry over.

Usage:
    conda activate drones
    python src/03_environmental_sar/3d/s046_3d_trilateration.py
"""

import os
import importlib.util
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from scipy.optimize import minimize

# -- Paths --------------------------------------------------------------------
SCRIPT_DIR   = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, '..', '..', '..'))
OUTPUT_DIR   = os.path.join(PROJECT_ROOT, 'outputs', '03_environmental_sar', '3d',
                            's046_3d_trilateration')

_spec = importlib.util.spec_from_file_location(
    's046_2d', os.path.join(SCRIPT_DIR, '..', 's046_trilateration.py'))
s2d = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(s2d)

# -- Parameters (card values unless marked "added") ----------------------------
N_DRONES     = 5
AREA_XY      = 150.0     # m  - search area is [-150, 150]^2 x [0, 50] m
AREA_Z       = 50.0
SIGMA_R      = 0.5       # m  - baseline range noise std dev
Z_MIN, Z_MAX = 5.0, 45.0 # m  - drone altitude bounds
DZ_MIN       = 20.0      # m  - minimum altitude spread of the optimised layout
R_MIN, R_MAX = 30.0, 120.0   # m - horizontal radius bounds (from the area centre)
D_SEP        = 15.0      # m  - inter-drone separation
EPSILON_TOL  = s2d.EPSILON_TOL   # 1e-4 m  (same as 2D)
K_MAX        = s2d.K_MAX         # 100     (same as 2D)
N_TRIALS     = 500
SIGMA_LEVELS = s2d.SIGMA_LEVELS  # 0.1, 0.25 ... 3.0 m (13 levels, same as 2D)
X_NOMINAL    = np.array([30.0, -20.0, 2.0])  # added: beacon used for the layout design (2D value)
Z_SWEEP      = np.arange(0.0, 10.01, 1.0)    # beacon altitude sweep z* (m)
DZ_SWEEP     = np.arange(0.0, 40.01, 5.0)    # added: altitude-band widths for the DOP sweep
N_STARTS     = 16        # added: L-BFGS-B multi-starts per optimisation
PEN_W        = 1e4       # added: quadratic penalty weight (separation / spread)
DIVERGE_M    = 20.0      # added: a 3D error above this counts as a gross error (2D value)
R_RING       = 80.0      # added: coplanar baseline = 5 drones on an 80 m ring at z = 5 m
SEED_OPT     = 0
SEED_MC      = 42        # same noise stream for every layout / level / z*
SEED_UNIQ    = 7
SEED_ANIM    = 99

# Coplanar baseline: the 2D S046 coplanar ring extended to five drones
ANCHORS_FLAT = np.array([[R_RING * np.cos(a), R_RING * np.sin(a), Z_MIN]
                         for a in np.arange(N_DRONES) * 2 * np.pi / N_DRONES])
ANCHORS_2D   = s2d.ANCHORS_GOOD   # 2D S046 well-spread 4-drone layout (reference)

EST_NAMES  = ['Range LLS', 'Range GN', 'TDOA LLS', 'TDOA GN']
EST_COLORS = ['tab:blue', 'navy', 'tab:orange', 'darkred']


# -- Geometry / DOP -------------------------------------------------------------

def unit_vectors(anchors, x):
    d = x - anchors
    return d / np.linalg.norm(d, axis=1, keepdims=True)


def _dop_from_cov(cov):
    return (float(np.sqrt(np.trace(cov))), float(np.sqrt(cov[0, 0] + cov[1, 1])),
            float(np.sqrt(cov[2, 2])))


def range_cov(anchors, x):
    """Range CRLB covariance / sigma_r^2 = (H^T H)^-1 (inf if singular)."""
    H = unit_vectors(anchors, x)
    M = H.T @ H
    if np.linalg.cond(M) > 1e12:
        return np.full((3, 3), np.inf)
    return np.linalg.inv(M)


def tdoa_Q(n):
    """TDOA noise covariance / sigma_r^2 for differences against drone 1."""
    return np.eye(n - 1) + np.ones((n - 1, n - 1))


def tdoa_cov(anchors, x, card_formula=False):
    """
    TDOA CRLB covariance / sigma_r^2.  Correct form: (Ht^T Q^-1 Ht)^-1 with the
    correlated noise Q = I + 11^T.  card_formula=True uses the card's
    independent-noise form 2 (Ht^T Ht)^-1.
    """
    U  = unit_vectors(anchors, x)
    Ht = U[1:] - U[0]
    M  = 0.5 * Ht.T @ Ht if card_formula else Ht.T @ np.linalg.solve(tdoa_Q(len(anchors)), Ht)
    if np.linalg.cond(M) > 1e12:
        return np.full((3, 3), np.inf)
    return np.linalg.inv(M)


def range_dop(anchors, x):
    return _dop_from_cov(range_cov(anchors, x))


def tdoa_dop(anchors, x, card_formula=False):
    return _dop_from_cov(tdoa_cov(anchors, x, card_formula))


# -- Layout optimisation --------------------------------------------------------

def layout_from_vars(v):
    v = v.reshape(-1, 3)
    return np.c_[v[:, 1] * np.cos(v[:, 0]), v[:, 1] * np.sin(v[:, 0]), v[:, 2]]


def constraint_violation(P, dz_min):
    """Separation shortfall^2 + spread shortfall^2 (0 when feasible)."""
    pen = 0.0
    for i in range(len(P)):
        for j in range(i + 1, len(P)):
            pen += max(0.0, D_SEP - np.linalg.norm(P[i] - P[j])) ** 2
    pen += max(0.0, dz_min - (P[:, 2].max() - P[:, 2].min())) ** 2
    return pen


def objective(v, x, dz_min, kind):
    P = layout_from_vars(v)
    g = (range_dop if kind == 'range' else tdoa_dop)(P, x)[0]
    if not np.isfinite(g):
        g = 1e3
    return g + PEN_W * constraint_violation(P, dz_min)


def optimise_layout(x, dz_min, z_hi, kind='range', record=False):
    """Multi-start L-BFGS-B over (phi_i, R_i, z_i); z bounds [Z_MIN, z_hi]."""
    rng = np.random.default_rng(SEED_OPT)
    bounds = [(-2 * np.pi, 2 * np.pi), (R_MIN, R_MAX), (Z_MIN, z_hi)] * N_DRONES
    best, best_v0 = None, None
    for _ in range(N_STARTS):
        v0 = np.c_[rng.uniform(-np.pi, np.pi, N_DRONES), rng.uniform(R_MIN, R_MAX, N_DRONES),
                   rng.uniform(Z_MIN, z_hi, N_DRONES)].ravel()
        r = minimize(objective, v0, args=(x, dz_min, kind), method='L-BFGS-B', bounds=bounds)
        if best is None or r.fun < best.fun - 1e-12:
            best, best_v0 = r, v0
    hist = None
    if record:   # rerun the winning start with a callback to log the objective
        def _log(vk):
            Pk = layout_from_vars(vk)
            hist.append((objective(vk, x, dz_min, kind), range_dop(Pk, x)[0],
                         np.sqrt(constraint_violation(Pk, dz_min))))
        hist = []
        _log(best_v0)
        minimize(objective, best_v0, args=(x, dz_min, kind), method='L-BFGS-B', bounds=bounds,
                 callback=_log)
    P = layout_from_vars(best.x)
    # invariants: bounds and constraints hold (penalty leaves < 5 cm violation)
    assert np.all(P[:, 2] >= Z_MIN - 1e-9) and np.all(P[:, 2] <= z_hi + 1e-9)
    assert np.sqrt(constraint_violation(P, dz_min)) < 0.05, 'layout infeasible'
    return P, best, hist


# -- Estimators -------------------------------------------------------------------

def solve_tdoa_lls(anchors, d):
    """
    Two-stage TDOA linear solve (card Stage 1/2): unknowns [x, y, z, rho_1],
    rows  2 (p_i - p_1)^T x + 2 d_i1 rho_1 = |p_i|^2 - |p_1|^2 - d_i1^2.
    The card's W_d is a scaled identity, so the WLS equals ordinary LS.
    For coplanar anchors the z column is zero; z is then recovered from
    rho_1 with the root below the drone plane (as in the 2D range LLS).
    """
    p1 = anchors[0]
    C = np.c_[2.0 * (anchors[1:] - p1), 2.0 * d]
    g = np.sum(anchors[1:] ** 2, axis=1) - p1 @ p1 - d ** 2
    sol, _, rank, _ = np.linalg.lstsq(C, g, rcond=None)
    x = sol[:3].copy()
    if np.ptp(anchors[:, 2]) < 1e-9:
        dz2 = sol[3] ** 2 - np.sum((p1[:2] - x[:2]) ** 2)
        x[2] = p1[2] - np.sqrt(max(dz2, 0.0))
    return x


def solve_tdoa_gn(anchors, d, x_init):
    """Weighted Gauss-Newton on h_i(x) = rho_i - rho_1 with W = Q^-1."""
    W = np.linalg.inv(tdoa_Q(len(anchors)))
    x = x_init.copy()
    trace = []
    for _ in range(K_MAX):
        rho = np.linalg.norm(x - anchors, axis=1)
        U = (x - anchors) / np.maximum(rho[:, None], 1e-12)
        J = U[1:] - U[0]
        A = J.T @ W @ J
        if abs(np.linalg.det(A)) < 1e-10:
            break
        delta = np.linalg.solve(A, J.T @ W @ (d - (rho[1:] - rho[0])))
        x = x + delta
        trace.append(x.copy())
        if np.linalg.norm(delta) < EPSILON_TOL:
            break
    return x, trace


def estimate_all(anchors, x_true, e):
    """Run the four estimators on one shared per-drone noise draw e."""
    rho = np.linalg.norm(anchors - x_true, axis=1)
    r = rho + e
    d = (rho[1:] + e[1:]) - (rho[0] + e[0])
    x_rl = s2d.solve_lls(anchors, r)
    x_rg, _ = s2d.solve_gauss_newton(anchors, r, x_rl)
    x_tl = solve_tdoa_lls(anchors, d)
    x_tg, _ = solve_tdoa_gn(anchors, d, x_tl)
    return np.array([x_rl, x_rg, x_tl, x_tg]) - x_true


def monte_carlo(anchors, x_true, sigma):
    """Error array (N_TRIALS, 4 estimators, 3)."""
    rng = np.random.default_rng(SEED_MC)
    E = rng.normal(0.0, 1.0, size=(N_TRIALS, len(anchors))) * sigma
    return np.array([estimate_all(anchors, x_true, e) for e in E])


def rmse(err):
    return float(np.sqrt(np.mean(np.sum(err ** 2, axis=-1))))


def uniqueness_check(anchors, x_true, n_starts=200):
    """
    Noise-free range GN (pseudo-inverse steps) from random starts in the
    search volume.  A start ends on an exact solution if the residual RMS is
    < 1e-6 m.  Returns (number of exact endings, distinct exact solutions,
    number of starts ending elsewhere = local minimum or not converged).
    """
    rng = np.random.default_rng(SEED_UNIQ)
    r = np.linalg.norm(anchors - x_true, axis=1)
    sols, n_exact = [], 0
    for _ in range(n_starts):
        xh = np.r_[rng.uniform(-AREA_XY, AREA_XY, 2), rng.uniform(0, AREA_Z)]
        for _ in range(K_MAX):
            rho = np.linalg.norm(xh - anchors, axis=1)
            H = (xh - anchors) / np.maximum(rho[:, None], 1e-12)
            delta = np.linalg.pinv(H) @ (r - rho)
            xh = xh + delta
            if np.linalg.norm(delta) < EPSILON_TOL:
                break
        res = np.sqrt(np.mean((r - np.linalg.norm(xh - anchors, axis=1)) ** 2))
        if res < 1e-6:
            n_exact += 1
            if not any(np.linalg.norm(xh - q) < 1e-3 for q in sols):
                sols.append(xh.copy())
    return n_exact, sols, n_starts - n_exact


# -- Simulation -----------------------------------------------------------------

def run_simulation():
    out = {}
    # 1) optimised layout (card constraints) and the no-spread-constraint check
    P_opt, res_opt, hist = optimise_layout(X_NOMINAL, DZ_MIN, Z_MAX, 'range', record=True)
    P_nodz, _, _ = optimise_layout(X_NOMINAL, 0.0, Z_MAX, 'range')
    P_tdoa, _, _ = optimise_layout(X_NOMINAL, DZ_MIN, Z_MAX, 'tdoa')
    out.update(P_opt=P_opt, hist=np.array(hist), P_nodz=P_nodz, P_tdoa=P_tdoa)

    # 2) DOP vs allowed altitude band [Z_MIN, Z_MIN + dz] (re-optimised per dz)
    band = {'range': [], 'tdoa': []}
    for dz in DZ_SWEEP:
        for kind in ('range', 'tdoa'):
            P, _, _ = optimise_layout(X_NOMINAL, 0.0, Z_MIN + dz, kind)
            band[kind].append((range_dop(P, X_NOMINAL), tdoa_dop(P, X_NOMINAL), np.ptp(P[:, 2])))
    out['band'] = band
    # fixed-plane check: all five drones at z = 45 m (coplanar but high above the beacon)
    rng = np.random.default_rng(SEED_OPT)
    bounds45 = [(-2 * np.pi, 2 * np.pi), (R_MIN, R_MAX), (Z_MAX, Z_MAX)] * N_DRONES
    best = None
    for _ in range(N_STARTS):
        v0 = np.c_[rng.uniform(-np.pi, np.pi, N_DRONES), rng.uniform(R_MIN, R_MAX, N_DRONES),
                   np.full(N_DRONES, Z_MAX)].ravel()
        r = minimize(objective, v0, args=(X_NOMINAL, 0.0, 'range'), method='L-BFGS-B',
                     bounds=bounds45)
        if best is None or r.fun < best.fun - 1e-12:
            best = r
    P45 = layout_from_vars(best.x)
    assert np.sqrt(constraint_violation(P45, 0.0)) < 0.05
    out['P45'] = P45

    # 3) uniqueness: 3 vs 4 anchors (first drones of the optimised layout)
    out['uniq3'] = uniqueness_check(P_opt[:3], X_NOMINAL)
    out['uniq4'] = uniqueness_check(P_opt[:4], X_NOMINAL)

    # 4) Monte Carlo at baseline sigma: range-optimised, TDOA-optimised, coplanar ring
    layouts = (('opt', P_opt), ('tdoa', P_tdoa), ('flat', ANCHORS_FLAT))
    out['mc'] = {name: monte_carlo(P, X_NOMINAL, SIGMA_R) for name, P in layouts}

    # 5) noise sweep: range-optimised (card) and TDOA-optimised layouts
    out['sweep'], out['sweep_gross'] = {}, {}
    for name, P in layouts[:2]:
        Es = [monte_carlo(P, X_NOMINAL, s) for s in SIGMA_LEVELS]
        out['sweep'][name] = np.array([[rmse(E[:, k]) for k in range(4)] for E in Es])
        out['sweep_gross'][name] = np.array(
            [[int(np.sum(np.linalg.norm(E[:, k], axis=1) > DIVERGE_M)) for k in range(4)]
             for E in Es])

    # 6) beacon altitude sweep (layouts fixed, designed for z* = 2 m)
    zs = {}
    for name, P in layouts:
        rows = []
        for z in Z_SWEEP:
            xt = np.array([X_NOMINAL[0], X_NOMINAL[1], z])
            E = monte_carlo(P, xt, SIGMA_R)
            n3 = np.linalg.norm(E, axis=2)
            rows.append(dict(rdop=range_dop(P, xt), tdop=tdoa_dop(P, xt),
                             rmse=[rmse(E[:, k]) for k in range(4)],
                             med=[float(np.median(n3[:, k])) for k in range(4)],
                             gross=[int(np.sum(n3[:, k] > DIVERGE_M)) for k in range(4)]))
        zs[name] = rows
    out['zsweep'] = zs

    # 7) GDOP map of the optimised layout over the area at z = 2 m (point-design check)
    g = np.linspace(-AREA_XY, AREA_XY, 61)
    out['gmap'] = np.array([[range_dop(P_opt, np.array([xx, yy, X_NOMINAL[2]]))[0]
                             for xx in g] for yy in g])
    return out


# -- Plots -------------------------------------------------------------------------

def _save(fig, name):
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    path = os.path.join(OUTPUT_DIR, name)
    fig.savefig(path, dpi=130)
    plt.close(fig)
    print(f'Saved: {path}')


def _ellipsoid(ax, centre, cov, nsig=3.0, color='tab:blue'):
    w, V = np.linalg.eigh(cov)
    u, v = np.meshgrid(np.linspace(0, 2 * np.pi, 30), np.linspace(0, np.pi, 16))
    sph = np.stack([np.cos(u) * np.sin(v), np.sin(u) * np.sin(v), np.cos(v)], -1)
    pts = sph @ (V * (nsig * np.sqrt(w))).T + centre
    ax.plot_surface(pts[..., 0], pts[..., 1], pts[..., 2], color=color, alpha=0.3, lw=0)


def plot_anchor_geometry(data):
    """3D anchor layouts (top) and 3-sigma range-GN error ellipsoids at true scale (bottom)."""
    fig = plt.figure(figsize=(18, 10))
    layouts = [('Range-GDOP-optimised layout (card)', data['P_opt']),
               ('TDOA-GDOP-optimised layout', data['P_tdoa']),
               ('Coplanar baseline (80 m ring, z = 5 m)', ANCHORS_FLAT)]
    for col, (title, P) in enumerate(layouts):
        g, h, v = range_dop(P, X_NOMINAL)
        gt = tdoa_dop(P, X_NOMINAL)[0]
        ax = fig.add_subplot(2, 3, col + 1, projection='3d')
        ax.scatter(P[:, 0], P[:, 1], P[:, 2], s=90, c='red', marker='^', label='Drone')
        ax.scatter(*X_NOMINAL, s=200, c='green', marker='*', label='Beacon')
        for i, p in enumerate(P):
            ax.plot(*zip(p, X_NOMINAL), 'k--', lw=0.8, alpha=0.5)
            ax.text(p[0], p[1], p[2] + 2, f'D{i+1}', fontsize=8, color='darkred')
        ax.set_xlim(-AREA_XY, AREA_XY); ax.set_ylim(-AREA_XY, AREA_XY); ax.set_zlim(0, AREA_Z)
        ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
        ax.set_title(f'{title}\nrange GDOP {g:.2f}  HDOP {h:.2f}  VDOP {v:.2f} | '
                     f'TDOA GDOP {gt:.2f}', fontsize=9)
        ax.legend(fontsize=8, loc='upper left')
        ax2 = fig.add_subplot(2, 3, col + 4, projection='3d')
        cov = SIGMA_R ** 2 * range_cov(P, X_NOMINAL)
        _ellipsoid(ax2, X_NOMINAL, cov, color='tab:blue')
        ax2.scatter(*X_NOMINAL, s=80, c='green', marker='*')
        L = 18.0
        ax2.set_xlim(X_NOMINAL[0] - L, X_NOMINAL[0] + L)
        ax2.set_ylim(X_NOMINAL[1] - L, X_NOMINAL[1] + L)
        ax2.set_zlim(X_NOMINAL[2] - L, X_NOMINAL[2] + L)
        ax2.set_xlabel('X (m)'); ax2.set_ylabel('Y (m)'); ax2.set_zlabel('Z (m)')
        sd = np.sqrt(np.diag(cov))
        ax2.set_title(f'3σ range error ellipsoid (σ_r = {SIGMA_R} m, true scale ±{L:.0f} m)\n'
                      f'1σ x/y/z = {sd[0]:.2f} / {sd[1]:.2f} / {sd[2]:.2f} m', fontsize=9)
    fig.suptitle('S046 3D - Anchor Geometry and Error Ellipsoids', fontsize=12, fontweight='bold')
    fig.tight_layout()
    _save(fig, 'anchor_geometry_3d.png')


def plot_dop_vs_spread(data):
    """VDOP / HDOP vs width of the allowed altitude band [5, 5 + dz] m."""
    fig, axes = plt.subplots(1, 2, figsize=(13, 4.8))
    for ax, kind, title in zip(axes, ('range', 'tdoa'),
                               ('Layout optimised for range GDOP',
                                'Layout optimised for TDOA GDOP')):
        rows = data['band'][kind]
        for j, (lab, ls, cv, ch) in enumerate((('range', '-', 'red', 'blue'),
                                               ('TDOA', '--', 'orange', 'deepskyblue'))):
            ax.plot(DZ_SWEEP, [r[j][2] for r in rows], color=cv, ls=ls, marker='o', ms=4,
                    label=f'VDOP ({lab})')
            ax.plot(DZ_SWEEP, [r[j][1] for r in rows], color=ch, ls=ls, marker='s', ms=4,
                    label=f'HDOP ({lab})')
        fr, ft = range_dop(ANCHORS_FLAT, X_NOMINAL), tdoa_dop(ANCHORS_FLAT, X_NOMINAL)
        ax.axhline(fr[2], color='gray', ls=':', lw=1.2,
                   label=f'coplanar 80 m ring: VDOP range {fr[2]:.1f} (TDOA {ft[2]:.0f})')
        ax.set_yscale('log')
        ax.set_xlabel('Allowed altitude band width Δz (drones in [5, 5 + Δz] m)')
        ax.set_ylabel('DOP (log scale)')
        ax.set_title(title, fontsize=10); ax.grid(True, alpha=0.3); ax.legend(fontsize=7)
    fig.suptitle('S046 3D - HDOP / VDOP vs altitude band (re-optimised at each Δz, beacon z* = 2 m)',
                 fontsize=11, fontweight='bold')
    fig.tight_layout()
    _save(fig, 'dop_vs_spread.png')


def _hyperboloid_sheet(pa, pb, dd, x_ref, nt=40, nth=48):
    """Sheet of |x - pa| - |x - pb| = dd, parametrised up to 1.6x the beacon's t."""
    c = 0.5 * (pa + pb); f = 0.5 * np.linalg.norm(pa - pb)
    e = (pa - pb) / (2 * f)
    A = 0.5 * abs(dd); B = np.sqrt(f ** 2 - A ** 2)
    tmp = np.array([1.0, 0, 0]) if abs(e[0]) < 0.9 else np.array([0, 1.0, 0])
    e2 = np.cross(e, tmp); e2 /= np.linalg.norm(e2); e3 = np.cross(e, e2)
    rel = x_ref - c
    rperp = np.linalg.norm(rel - (rel @ e) * e)
    t_star = np.arcsinh(rperp / B)
    t, th = np.meshgrid(np.linspace(0, 1.6 * max(t_star, 0.5), nt), np.linspace(0, 2 * np.pi, nth))
    u = -np.sign(dd) * A * np.cosh(t)
    rr = B * np.sinh(t)
    pts = (c[None, None] + u[..., None] * e + (rr * np.cos(th))[..., None] * e2
           + (rr * np.sin(th))[..., None] * e3)
    out = (np.abs(pts[..., 0]) > AREA_XY) | (np.abs(pts[..., 1]) > AREA_XY) | \
          (pts[..., 2] < 0) | (pts[..., 2] > AREA_Z)
    pts[out] = np.nan
    return pts


def plot_tdoa_hyperboloids(data):
    """Three TDOA hyperboloid sheets (drones 2-4 vs reference drone 1, N = 4)."""
    P = data['P_opt'][:4]
    rho = np.linalg.norm(P - X_NOMINAL, axis=1)
    fig = plt.figure(figsize=(9, 7))
    ax = fig.add_subplot(111, projection='3d')
    cols = ['tab:orange', 'tab:purple', 'tab:cyan']
    for k, i in enumerate(range(1, 4)):
        S = _hyperboloid_sheet(P[i], P[0], rho[i] - rho[0], X_NOMINAL)
        F = S[np.isfinite(S[..., 0])]
        resid = (np.linalg.norm(F - P[i], axis=1) - np.linalg.norm(F - P[0], axis=1)
                 - (rho[i] - rho[0]))
        assert np.abs(resid).max() < 1e-6, 'hyperboloid parametrisation wrong'
        print(f'Hyperboloid D{i+1}-D1: max |TDOA residual| on sheet {np.abs(resid).max():.1e} m, '
              f'closest grid point to x* {np.linalg.norm(F - X_NOMINAL, axis=1).min():.2f} m')
        ax.plot_surface(S[..., 0], S[..., 1], S[..., 2], color=cols[k], alpha=0.35, lw=0)
        ax.plot([], [], color=cols[k], lw=6, alpha=0.5,
                label=f'd_{i+1}1 = {rho[i]-rho[0]:+.1f} m (D{i+1} vs D1)')
    ax.scatter(P[:, 0], P[:, 1], P[:, 2], s=90, c='red', marker='^', label='Drone')
    for i, p in enumerate(P):
        ax.text(p[0], p[1], p[2] + 2, f'D{i+1}', fontsize=9, color='darkred')
    ax.scatter(*X_NOMINAL, s=250, c='green', marker='*', label='Beacon x*', depthshade=False)
    ax.set_xlim(-AREA_XY, AREA_XY); ax.set_ylim(-AREA_XY, AREA_XY); ax.set_zlim(0, AREA_Z)
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
    ax.legend(fontsize=8, loc='upper left')
    ax.set_title('S046 3D - TDOA hyperboloid sheets (noise-free, N = 4, reference D1)\n'
                 'all three pass through the beacon; clipped to the search volume', fontsize=10)
    fig.tight_layout()
    _save(fig, 'tdoa_hyperboloids.png')


LAYOUT_LABELS = {'opt': 'range-optimised (card)', 'tdoa': 'TDOA-optimised',
                 'flat': 'coplanar 80 m ring'}


def plot_rmse_vs_noise(data):
    fig, axes = plt.subplots(1, 2, figsize=(14, 5.2), sharey=True)
    sl = SIGMA_LEVELS
    for ax, name, P in zip(axes, ('opt', 'tdoa'), (data['P_opt'], data['P_tdoa'])):
        g_r = range_dop(P, X_NOMINAL)[0]
        g_t = tdoa_dop(P, X_NOMINAL)[0]
        ax.fill_between(sl, 1e-3, sl * g_r, color='tab:blue', alpha=0.12,
                        label=f'below range CRLB σ_r·{g_r:.3f}')
        ax.fill_between(sl, 1e-3, sl * g_t, color='tab:orange', alpha=0.10,
                        label=f'below TDOA CRLB σ_r·{g_t:.3f}')
        marks = ['o', 's', 'o', 's']; lss = ['-', '--', '-', '--']
        for k in range(4):
            ax.plot(sl, data['sweep'][name][:, k], color=EST_COLORS[k], ls=lss[k],
                    marker=marks[k], ms=4, label=EST_NAMES[k])
        ax.set_xscale('log'); ax.set_yscale('log'); ax.set_ylim(0.08, 40)
        ax.set_xlabel('Range noise σ_r (m, log)'); ax.set_ylabel('3D RMSE (m, log)')
        ax.set_title(f'{LAYOUT_LABELS[name]} layout', fontsize=10)
        ax.grid(True, which='both', alpha=0.3); ax.legend(fontsize=8)
    fig.suptitle(f'S046 3D - RMSE vs noise ({N_TRIALS} trials / level, same noise for all estimators)',
                 fontsize=11, fontweight='bold')
    fig.tight_layout()
    _save(fig, 'rmse_vs_noise.png')


def plot_beacon_altitude_sweep(data):
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.8))
    lay_col = {'opt': 'tab:blue', 'tdoa': 'tab:orange', 'flat': 'dimgray'}
    for name in ('opt', 'tdoa', 'flat'):
        lab = LAYOUT_LABELS[name]; c = lay_col[name]
        rows = data['zsweep'][name]
        for k, ls, mk in ((1, '-', 'o'), (3, '--', 's')):
            axes[0].plot(Z_SWEEP, [r['med'][k] for r in rows], color=c, ls=ls, marker=mk, ms=4,
                         label=f'{EST_NAMES[k]} - {lab}')
        axes[1].plot(Z_SWEEP, [r['rdop'][2] for r in rows], color=c, ls='-', marker='o', ms=4,
                     label=f'range - {lab}')
        axes[1].plot(Z_SWEEP, [r['tdop'][2] for r in rows], color=c, ls='--', marker='s', ms=4,
                     label=f'TDOA - {lab}')
        axes[2].plot(Z_SWEEP, [r['rdop'][1] for r in rows], color=c, ls='-', marker='o', ms=4,
                     label=f'range - {lab}')
        axes[2].plot(Z_SWEEP, [r['tdop'][1] for r in rows], color=c, ls='--', marker='s', ms=4,
                     label=f'TDOA - {lab}')
    axes[0].set_ylabel('median 3D error (m, log)')
    axes[0].set_title(f'Median 3D error, σ_r = {SIGMA_R} m ({N_TRIALS} trials)\n'
                      'solid = Range GN, dashed = TDOA GN', fontsize=10)
    axes[1].set_title('VDOP (solid range, dashed TDOA)\nring at z* = 5 m: singular, not drawn',
                      fontsize=10)
    axes[2].set_title('HDOP (solid range, dashed TDOA)', fontsize=10)
    for ax in axes:
        ax.set_yscale('log'); ax.set_xlabel('True beacon altitude z* (m)')
        ax.grid(True, which='both', alpha=0.3); ax.legend(fontsize=6.5)
    fig.suptitle('S046 3D - Beacon altitude sweep (layouts designed for z* = 2 m)',
                 fontsize=11, fontweight='bold')
    fig.tight_layout()
    _save(fig, 'beacon_altitude_sweep.png')


def plot_gdop_convergence(data):
    fig, ax = plt.subplots(figsize=(8, 4.5))
    h = data['hist']
    it = np.arange(len(h))
    ax.plot(it, h[:, 1], 'k-o', ms=3, label='range GDOP')
    ax.axhline(np.sqrt(9 / N_DRONES), color='r', ls='--',
               label=f'lower bound √(9/N) = {np.sqrt(9 / N_DRONES):.4f}')
    ax.set_xlabel('L-BFGS-B iteration (winning start)')
    ax.set_ylabel('range GDOP')
    ax2 = ax.twinx()
    ax2.semilogy(it, np.maximum(h[:, 2], 1e-6), 'b:s', ms=3,
                 label='constraint violation (m, floored at 1e-6)')
    ax2.set_ylabel('√(separation² + spread² shortfall) (m, log)', color='b')
    ax.set_title('S046 3D - GDOP optimisation convergence', fontsize=11, fontweight='bold')
    ax.grid(True, alpha=0.3)
    l1, b1 = ax.get_legend_handles_labels(); l2, b2 = ax2.get_legend_handles_labels()
    ax.legend(l1 + l2, b1 + b2, fontsize=8)
    fig.tight_layout()
    _save(fig, 'gdop_convergence.png')


def plot_error_histogram(data):
    fig, axes = plt.subplots(2, 2, figsize=(13, 8.5))
    for row, (name, P) in enumerate((('opt', data['P_opt']), ('tdoa', data['P_tdoa']))):
        E = data['mc'][name]
        cr = SIGMA_R ** 2 * range_cov(P, X_NOMINAL)
        ct = SIGMA_R ** 2 * tdoa_cov(P, X_NOMINAL)
        comps = [(np.linalg.norm(E[:, :, :2], axis=2), 'horizontal error √(Δx²+Δy²) (m)',
                  np.sqrt(cr[0, 0] + cr[1, 1]), np.sqrt(ct[0, 0] + ct[1, 1])),
                 (np.abs(E[:, :, 2]), 'vertical error |Δz| (m)', np.sqrt(cr[2, 2]),
                  np.sqrt(ct[2, 2]))]
        for ax, (vals, xl, b_r, b_t) in zip(axes[row], comps):
            bins = np.linspace(0, np.percentile(vals, 99.5), 40)
            for k in range(4):
                ax.hist(vals[:, k], bins=bins, histtype='step', lw=1.6, color=EST_COLORS[k],
                        label=f'{EST_NAMES[k]} (RMS {np.sqrt(np.mean(vals[:, k] ** 2)):.3f} m)')
            ax.axvline(b_r, color='navy', ls=':', label=f'range CRLB RMS {b_r:.3f} m')
            ax.axvline(b_t, color='darkred', ls=':', label=f'TDOA CRLB RMS {b_t:.3f} m')
            ax.set_xlabel(xl); ax.set_ylabel('count'); ax.grid(True, alpha=0.3)
            ax.legend(fontsize=7)
            ax.set_title(f'{LAYOUT_LABELS[name]} layout', fontsize=10)
    fig.suptitle(f'S046 3D - Error components, σ_r = {SIGMA_R} m '
                 f'({N_TRIALS} trials, same noise for all estimators)', fontsize=11,
                 fontweight='bold')
    fig.tight_layout()
    _save(fig, 'error_histogram.png')


def save_animation(data):
    """Range GN and TDOA GN iterates from the same poor start, rotating 3D view."""
    P = data['P_opt']
    rng = np.random.default_rng(SEED_ANIM)
    e = rng.normal(0.0, SIGMA_R, N_DRONES)
    x0 = X_NOMINAL + rng.normal(0.0, 40.0, 3)
    rho = np.linalg.norm(P - X_NOMINAL, axis=1)
    r = rho + e; d = (rho[1:] + e[1:]) - (rho[0] + e[0])
    xr = [x0.copy()]
    x = x0.copy()
    for _ in range(K_MAX):
        rr = np.linalg.norm(x - P, axis=1); H = (x - P) / rr[:, None]
        delta = np.linalg.solve(H.T @ H, H.T @ (r - rr)); x = x + delta; xr.append(x.copy())
        if np.linalg.norm(delta) < EPSILON_TOL:
            break
    xt_end, tr = solve_tdoa_gn(P, d, x0)
    xt = [x0.copy()] + tr
    xr, xt = np.array(xr), np.array(xt)
    print(f'Animation: range GN {len(xr) - 1} iterations (final error '
          f'{np.linalg.norm(xr[-1] - X_NOMINAL):.3f} m), TDOA GN {len(xt) - 1} iterations '
          f'(final error {np.linalg.norm(xt[-1] - X_NOMINAL):.3f} m)')
    n_frames = 36
    fig = plt.figure(figsize=(8, 6.5))
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(P[:, 0], P[:, 1], P[:, 2], s=70, c='red', marker='^', label='Drone')
    ax.scatter(*X_NOMINAL, s=200, c='green', marker='*', label='Beacon')
    (lr,) = ax.plot([], [], [], 'o-', color='navy', ms=4, label='Range GN iterates')
    (lt,) = ax.plot([], [], [], 's-', color='darkred', ms=4, label='TDOA GN iterates')
    allp = np.vstack([P, xr, xt])
    ax.set_xlim(allp[:, 0].min() - 10, allp[:, 0].max() + 10)
    ax.set_ylim(allp[:, 1].min() - 10, allp[:, 1].max() + 10)
    ax.set_zlim(min(0, allp[:, 2].min() - 5), allp[:, 2].max() + 5)
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
    ax.legend(fontsize=8, loc='upper left')
    ttl = ax.set_title('')

    def update(f):
        kr = min(f, len(xr) - 1); kt = min(f, len(xt) - 1)
        lr.set_data(xr[:kr + 1, 0], xr[:kr + 1, 1]); lr.set_3d_properties(xr[:kr + 1, 2])
        lt.set_data(xt[:kt + 1, 0], xt[:kt + 1, 1]); lt.set_3d_properties(xt[:kt + 1, 2])
        ax.view_init(elev=22, azim=-60 + 5 * f)
        ttl.set_text(f'S046 3D - GN iteration {f}: |err| range {np.linalg.norm(xr[kr]-X_NOMINAL):.2f} m'
                     f', TDOA {np.linalg.norm(xt[kt]-X_NOMINAL):.2f} m')
        return lr, lt, ttl

    ani = animation.FuncAnimation(fig, update, frames=n_frames, blit=False)
    path = os.path.join(OUTPUT_DIR, 'animation.gif')
    ani.save(path, writer='pillow', fps=4, dpi=80)
    plt.close(fig)
    print(f'Saved: {path} ({n_frames} frames)')


# -- Report ---------------------------------------------------------------------------

def _fmt_dop(t):
    return '  '.join('inf' if not np.isfinite(v) else f'{v:.3f}' for v in t)


def report(data):
    P = data['P_opt']
    print('\n-- Optimised layout (range GDOP, card constraints) --')
    print(' drone |      x |      y |     z |  R_xy | elevation seen from beacon')
    for i, p in enumerate(P):
        rel = p - X_NOMINAL
        el = np.degrees(np.arctan2(rel[2], np.linalg.norm(rel[:2])))
        print(f'  D{i+1}   | {p[0]:6.1f} | {p[1]:6.1f} | {p[2]:5.1f} | {np.linalg.norm(p[:2]):5.1f}'
              f' | {el:5.1f} deg')
    seps = [np.linalg.norm(P[i] - P[j]) for i in range(5) for j in range(i + 1, 5)]
    print(f'  altitude spread {np.ptp(P[:, 2]):.2f} m, min separation {min(seps):.2f} m, '
          f'L-BFGS-B iterations {len(data["hist"]) - 1}, start GDOP {data["hist"][0, 1]:.3f} '
          f'(violation {data["hist"][0, 2]:.1f} m), final objective {data["hist"][-1, 0]:.6f}')
    print(f'  lower bound sqrt(9/N) = {np.sqrt(9 / N_DRONES):.6f}')
    print('\n-- DOP table at x* = (30, -20, 2) [GDOP HDOP VDOP] --')
    rows = [('optimised (range)', P), ('optimised, no spread constraint', data['P_nodz']),
            ('optimised for TDOA', data['P_tdoa']), ('all 5 drones at z = 45 m', data['P45']),
            ('coplanar 80 m ring z = 5 m', ANCHORS_FLAT), ('2D S046 well-spread (N=4)', ANCHORS_2D)]
    for name, A in rows:
        print(f'  {name:32s} spread {np.ptp(A[:, 2]):5.1f} m | range {_fmt_dop(range_dop(A, X_NOMINAL))}'
              f' | TDOA {_fmt_dop(tdoa_dop(A, X_NOMINAL))}'
              f' | TDOA card-formula {_fmt_dop(tdoa_dop(A, X_NOMINAL, True))}')
    print('\n-- Altitude band sweep: drones confined to [5, 5 + dz] m, re-optimised --')
    print('   dz | range-opt: range G/H/V        TDOA G/H/V        spread | tdoa-opt: range G/H/V'
          '        TDOA G/H/V        spread')
    for i, dz in enumerate(DZ_SWEEP):
        a = data['band']['range'][i]; b = data['band']['tdoa'][i]
        print(f'  {dz:4.0f} | {_fmt_dop(a[0])} | {_fmt_dop(a[1])} | {a[2]:5.1f} | '
              f'{_fmt_dop(b[0])} | {_fmt_dop(b[1])} | {b[2]:5.1f}')
    print('\n-- TDOA-optimised layout (TDOA GDOP, card constraints) --')
    for i, p in enumerate(data['P_tdoa']):
        rel = p - X_NOMINAL
        el = np.degrees(np.arctan2(rel[2], np.linalg.norm(rel[:2])))
        print(f'  D{i+1}   | {p[0]:6.1f} | {p[1]:6.1f} | {p[2]:5.1f} | '
              f'{np.linalg.norm(p[:2]):5.1f} | {el:5.1f} deg')
    print('\n-- Uniqueness (noise-free range GN, 200 random starts in the volume) --')
    for lab, (n_ex, sols, n_other) in (('N=3 (D1-D3)', data['uniq3']), ('N=4 (D1-D4)', data['uniq4'])):
        sol_txt = ', '.join('(' + ', '.join(f'{c:.2f}' for c in q) + ')' for q in sols)
        print(f'  {lab}: {n_ex} starts end on an exact solution, {len(sols)} distinct: {sol_txt};'
              f' {n_other} end in a local minimum / not converged')
    layouts = (('opt', P), ('tdoa', data['P_tdoa']), ('flat', ANCHORS_FLAT))
    for name, A in layouts:
        E = data['mc'][name]
        cr = SIGMA_R * np.array(range_dop(A, X_NOMINAL)); ct = SIGMA_R * np.array(tdoa_dop(A, X_NOMINAL))
        print(f'\n-- sigma_r = {SIGMA_R} m, {LAYOUT_LABELS[name]}, {N_TRIALS} trials --')
        print(f'  CRLB 3D/horiz/vert: range {_fmt_dop(cr)} m | TDOA {_fmt_dop(ct)} m')
        for k in range(4):
            e = E[:, k]; n3 = np.linalg.norm(e, axis=1)
            ok = n3 <= DIVERGE_M
            print(f'  {EST_NAMES[k]:9s}: RMSE {rmse(e):9.3f} m | median {np.median(n3):.3f} m | '
                  f'horiz RMS {np.sqrt(np.mean(np.sum(e[:, :2]**2, 1))):8.3f} | vert RMS '
                  f'{np.sqrt(np.mean(e[:, 2]**2)):9.3f} | bias z {e[:, 2].mean():+.3f} m | '
                  f'gross {np.sum(~ok)} | RMSE w/o gross {rmse(e[ok]):.3f} m')
        n3 = np.linalg.norm(E, axis=2)
        print(f'  per-trial 3D error: Range GN < TDOA GN in {np.sum(n3[:, 1] < n3[:, 3])}/{N_TRIALS};'
              f' Range GN < Range LLS in {np.sum(n3[:, 1] < n3[:, 0])}/{N_TRIALS};'
              f' TDOA GN < TDOA LLS in {np.sum(n3[:, 3] < n3[:, 2])}/{N_TRIALS}')
    n_o = np.linalg.norm(data['mc']['opt'], axis=2); n_t = np.linalg.norm(data['mc']['tdoa'], axis=2)
    print(f'\n  same noise, TDOA GN: TDOA-optimised layout better than range-optimised in '
          f'{np.sum(n_t[:, 3] < n_o[:, 3])}/{N_TRIALS}; Range GN: in {np.sum(n_t[:, 1] < n_o[:, 1])}/{N_TRIALS}')
    for name, A in layouts[:2]:
        gr = range_dop(A, X_NOMINAL)[0]; gt = tdoa_dop(A, X_NOMINAL)[0]
        print(f'\n-- Noise sweep, {LAYOUT_LABELS[name]} layout: 3D RMSE (m) [gross errors > 20 m] --')
        print(' sigma | Range LLS | Range GN | CRLB rng | TDOA LLS | TDOA GN | CRLB tdoa | gross RL/RG/TL/TG')
        for i, s in enumerate(SIGMA_LEVELS):
            w = data['sweep'][name][i]; gcount = data['sweep_gross'][name][i]
            print(f' {s:5.2f} | {w[0]:9.3f} | {w[1]:8.3f} | {s*gr:8.3f} | {w[2]:8.3f} | {w[3]:7.3f} | '
                  f'{s*gt:9.3f} | {"/".join(map(str, gcount))}')
    print('\n-- Beacon altitude sweep (sigma 0.5 m): median 3D error RL/RG/TL/TG (m); VDOP range/TDOA --')
    for name in ('opt', 'tdoa', 'flat'):
        print(f'  layout: {LAYOUT_LABELS[name]}')
        for z, r in zip(Z_SWEEP, data['zsweep'][name]):
            med = '/'.join(f'{m:.3f}' for m in r['med'])
            print(f'   z*={z:4.1f} | median {med} | gross {"/".join(map(str, r["gross"]))} | '
                  f'VDOP {_fmt_dop([r["rdop"][2]])}/{_fmt_dop([r["tdop"][2]])} | '
                  f'HDOP {_fmt_dop([r["rdop"][1]])}/{_fmt_dop([r["tdop"][1]])}')
    g = data['gmap']
    print(f'\nGDOP map of the optimised layout (z = 2 m, 300 x 300 m, 61x61): min {g.min():.3f}, '
          f'median {np.median(g):.3f}, max {g.max():.3f}; cells with GDOP < 2: {np.mean(g < 2)*100:.1f}%')


if __name__ == '__main__':
    print('Running S046 3D Trilateration with Altitude...')
    data = run_simulation()
    report(data)
    plot_anchor_geometry(data)
    plot_dop_vs_spread(data)
    plot_tdoa_hyperboloids(data)
    plot_rmse_vs_noise(data)
    plot_beacon_altitude_sweep(data)
    plot_gdop_convergence(data)
    plot_error_histogram(data)
    save_animation(data)
    print('\nDone. Outputs in:', OUTPUT_DIR)
