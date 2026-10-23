"""
S009 3D Upgrade — Differential Game (Lion & Man in 3D)
======================================================
Part A — HJI in 3D relative coordinates s = p_E - p_P ∈ [-10, 10]³ (no walls)
  Stationary HJI for the time-to-capture value:
      min_{u_P} max_{u_E} ∇V·(v_E u_E - v_P u_P) + 1 = 0
      ⇒  (v_P - v_E)|∇V| = 1,  V = 0 on |s| <= r_c,
      analytic V = (|s| - r_c)/(v_P - v_E).
  Same corrected scheme as the fixed 2D S009: pseudo-time iteration with a
  monotone Godunov upwind gradient, starting from V = 0,
      V <- V + τ (1 - (v_P - v_E)|∇V|_God),  τ = 0.9·Δx/(3(v_P - v_E)).
  - v_P = 5 > v_E = 3 on the card's 30³ grid: converges to the analytic V.
  - v_P = v_E = 4: V grows by τ every iteration → V = ∞ (no capture in open
    space).
  The z = 0 plane of the 3D solution is compared with a 2D solve on the same
  30 × 30 x-y nodes and with the analytic value; both players then play the
  asymmetric game by the interpolated 3D ∇V from three 3D starts.

Part B — equal-speed Lion & Man in the bounded cube [-5, 5]³ (v = 4 m/s,
  capture radius 0.15 m), walls enforced.  Lion: radius strategy (stay on the
  segment centre→man) | pure pursuit.  Man: flee (open-space HJI control,
  sliding along walls) | circle (horizontal 4.5 m circle, the 2D strategy) |
  sphere (tangent to a 4.5 m sphere, away from the lion — uses all 3 axes).
  Two starts: planar (z = 0, identical to 2D S009) and a 3D offset start.
  For every capture the script reports where the man was (interior / face /
  edge / corner of the cube) and how fast he was actually moving just before
  capture, so captures caused by a man pinned against walls are visible.

Usage:
    conda activate drones
    python src/01_pursuit_evasion/3d/s009_3d_differential_game.py
"""

import sys
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from matplotlib.animation import FuncAnimation, PillowWriter
from scipy.interpolate import RegularGridInterpolator

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..'))

# ── Parameters ───────────────────────────────────────────────
ARENA      = 5.0          # m — cube [-ARENA, ARENA]³
S_MAX      = 2 * ARENA    # m — relative-state grid [-S_MAX, S_MAX]^d
N_GRID     = 30           # card: 30 per axis (3D) — also used for the 2D comparison
CFL        = 0.9
MAX_ITER   = 500          # card
CONV_TOL   = 1e-6         # s
TAU_EQUAL  = 0.05         # s — pseudo-time step for the equal-speed divergence demo
ITER_EQUAL = 200
CAPTURE_R  = 0.15         # m
DT_SIM     = 1 / 48
MAX_TIME_OPEN  = 20.0
MAX_TIME_ARENA = 150.0
V_P_ASYM, V_E_ASYM = 5.0, 3.0
V_EQUAL    = 4.0
ISO_LEVELS = [1.0, 2.5, 4.0]   # s — isosurfaces drawn (radius 2.15, 5.15, 8.15 m)
CIRCLE_R   = 4.5          # m — man's circle / sphere radius
CIRCLE_GAIN = 2.0         # 1/m — radial correction gain
PIN_SPEED  = 1.0          # m/s — "pinned" if the man's actual speed is below this
S0         = 6.0          # m — open-space start separation
OPEN_STARTS = [('x-axis', 0.0, 0.0), ('az 22.5°, el 20°', 22.5, 20.0),
               ('cube diagonal', 45.0, np.degrees(np.arctan(1 / np.sqrt(2))))]
ARENA_STARTS = {'planar': (np.array([-3.0, 0.0, 0.0]), np.array([3.0, 0.0, 0.0])),
                'offset': (np.array([-3.0, -1.0, -2.0]), np.array([2.0, 2.0, 2.0]))}
CENTRE = np.zeros(3)

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), '..', '..', '..',
                          'outputs', '01_pursuit_evasion', '3d', 's009_3d_differential_game')


# ── Part A: HJI solver ───────────────────────────────────────

def godunov_grad_norm(V, dx):
    """Upwind (Godunov) |∇V| for a value function increasing outward."""
    d = V.ndim
    Vp = np.pad(V, 1, mode='edge')
    sq = np.zeros_like(V)
    for ax in range(d):
        sl_c = [slice(1, -1)] * d
        sl_m = list(sl_c); sl_m[ax] = slice(0, -2)
        sl_p = list(sl_c); sl_p[ax] = slice(2, None)
        c = Vp[tuple(sl_c)]
        dm = (c - Vp[tuple(sl_m)]) / dx
        dp = (Vp[tuple(sl_p)] - c) / dx
        sq += np.maximum(np.maximum(dm, -dp), 0.0) ** 2
    return np.sqrt(sq)


def solve_hji(v_p, v_e, n, dim, max_iter=MAX_ITER, tau=None):
    """Pseudo-time iteration on an n^dim grid over [-S_MAX, S_MAX]^dim.
    Nodes with |s| < r_c + Δx are fixed to the exact value (sub-grid capture set).
    Returns dict(g, V, delta, vmax, iters, converged, tau)."""
    g = np.linspace(-S_MAX, S_MAX, n)
    dx = g[1] - g[0]
    mesh = np.meshgrid(*([g] * dim), indexing='ij')
    R = np.sqrt(sum(m ** 2 for m in mesh))
    dv = v_p - v_e
    if tau is None:
        tau = CFL * dx / (dim * dv)
    fixed = R < CAPTURE_R + dx
    V = np.zeros_like(R)
    if dv > 0:
        V[fixed] = np.maximum(R[fixed] - CAPTURE_R, 0.0) / dv
    deltas, vmax, converged = [], [], False
    for it in range(max_iter):
        V_new = V + tau * (1.0 - dv * godunov_grad_norm(V, dx))
        V_new[fixed] = V[fixed]
        delta = np.max(np.abs(V_new - V))
        V = V_new
        deltas.append(delta); vmax.append(V.max())
        if dv > 0 and delta < CONV_TOL:
            converged = True
            break
    return dict(g=g, V=V, delta=np.array(deltas), vmax=np.array(vmax), iters=it + 1,
                converged=converged, tau=tau, R=R)


def analytic_value(R, v_p=V_P_ASYM, v_e=V_E_ASYM):
    return np.maximum(R - CAPTURE_R, 0.0) / (v_p - v_e)


def gradient_interpolators(g, V):
    grads = np.gradient(V, g[1] - g[0])
    return [RegularGridInterpolator((g, g, g), gk, bounds_error=False, fill_value=None)
            for gk in grads]


def unit(v):
    n = np.linalg.norm(v)
    return v / n if n > 1e-12 else np.zeros_like(v)


def direction(az_deg, el_deg):
    a, e = np.radians(az_deg), np.radians(el_deg)
    return np.array([np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)])


def run_open_game(e_dir, grads):
    """Asymmetric speeds, no walls, both players steer by the numerical ∇V(s)."""
    L = np.zeros(3); M = S0 * e_dir
    p_l, e_l = [L.copy()], [M.copy()]
    captured, cap_time = False, None
    for k in range(int(round(MAX_TIME_OPEN / DT_SIM))):
        s = M - L
        u = unit(np.array([gi([s])[0] for gi in grads]))
        L = L + V_P_ASYM * DT_SIM * u
        M = M + V_E_ASYM * DT_SIM * u
        p_l.append(L.copy()); e_l.append(M.copy())
        if np.linalg.norm(M - L) < CAPTURE_R:
            captured, cap_time = True, (k + 1) * DT_SIM
            break
    p, e = np.array(p_l), np.array(e_l)
    return dict(p=p, e=e, d=np.linalg.norm(e - p, axis=1), t=np.arange(len(p)) * DT_SIM,
                captured=captured, cap_time=cap_time, e0=e_dir * S0)


def isosurface_radii(g, V, level, n_az=36, n_el=19):
    """Radius at which the trilinear-interpolated V reaches `level` along each
    direction (bisection; V increases outward)."""
    interp = RegularGridInterpolator((g, g, g), V, bounds_error=False, fill_value=None)
    az = np.linspace(0, 2 * np.pi, n_az)
    el = np.linspace(-np.pi / 2, np.pi / 2, n_el)
    AZ, EL = np.meshgrid(az, el, indexing='ij')
    dirs = np.stack([np.cos(EL) * np.cos(AZ), np.cos(EL) * np.sin(AZ), np.sin(EL)], -1)
    lo = np.zeros(AZ.shape); hi = np.full(AZ.shape, S_MAX * 0.999)
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        val = interp(dirs * mid[..., None])
        below = val < level
        lo = np.where(below, mid, lo); hi = np.where(below, hi, mid)
    return AZ, EL, dirs, 0.5 * (lo + hi)


# ── Part B: bounded-cube Lion & Man ──────────────────────────

def _clip_out(pos, d):
    d = d.copy()
    for k in range(3):
        if (pos[k] >= ARENA - 1e-9 and d[k] > 0) or (pos[k] <= -ARENA + 1e-9 and d[k] < 0):
            d[k] = 0.0
    return d


def wall_slide(pos, d):
    """Remove components that push through a wall.  If nothing is left (motion
    straight into a wall/edge), slide sideways: try ẑ×d, x̂×d, ŷ×d in turn
    (ẑ×d is the 2D counter-clockwise turn).  In a corner this can still
    leave zero velocity — the man is then pinned."""
    out = _clip_out(pos, d)
    if np.linalg.norm(out) < 1e-9:
        for axis in (np.array([0, 0, 1.0]), np.array([1.0, 0, 0]), np.array([0, 1.0, 0])):
            out = _clip_out(pos, np.cross(axis, d))
            if np.linalg.norm(out) > 1e-9:
                break
    return unit(out)


def lion_radius(L, M, v, dt):
    """Move to the point of segment [centre, M] closest to the man that is
    reachable within v·dt; if the segment is out of reach, move toward it."""
    c = CENTRE
    seg = M - c
    a = seg @ seg
    step = v * dt
    if a < 1e-12:
        return L + step * unit(M - L)
    lam0 = np.clip((L - c) @ seg / a, 0.0, 1.0)
    foot = c + lam0 * seg
    if np.linalg.norm(L - foot) > step:
        return L + step * unit(foot - L)
    b = 2 * seg @ (c - L)
    cc = (c - L) @ (c - L) - step ** 2
    lam_hi = min((-b + np.sqrt(max(b * b - 4 * a * cc, 0.0))) / (2 * a), 1.0)
    return c + lam_hi * seg


def lion_pure(L, M, v, dt):
    return L + v * dt * unit(M - L)


def man_flee(L, M, v, dt):
    return np.clip(M + v * dt * wall_slide(M, unit(M - L)), -ARENA, ARENA)


def man_circle(L, M, v, dt):
    """2D strategy: counter-clockwise on a horizontal 4.5 m circle, z unchanged."""
    r = M - CENTRE; r[2] = 0.0
    rho = np.linalg.norm(r)
    r_hat = r / rho if rho > 1e-9 else np.array([1.0, 0, 0])
    t_hat = np.array([-r_hat[1], r_hat[0], 0.0])
    d = wall_slide(M, unit(t_hat + CIRCLE_GAIN * (CIRCLE_R - rho) * r_hat))
    return np.clip(M + v * dt * d, -ARENA, ARENA)


def man_sphere(L, M, v, dt):
    """3D strategy: run tangent to a 4.5 m sphere about the centre, in the
    tangent direction that points most directly away from the lion."""
    r = M - CENTRE
    rho = np.linalg.norm(r)
    r_hat = r / rho if rho > 1e-9 else np.array([1.0, 0, 0])
    away = M - L
    tan = away - (away @ r_hat) * r_hat
    if np.linalg.norm(tan) < 1e-6:
        tan = np.cross(r_hat, [0, 0, 1.0])
        if np.linalg.norm(tan) < 1e-6:
            tan = np.cross(r_hat, [1.0, 0, 0])
    d = wall_slide(M, unit(unit(tan) + CIRCLE_GAIN * (CIRCLE_R - rho) * r_hat))
    return np.clip(M + v * dt * d, -ARENA, ARENA)


LIONS = {'radius': lion_radius, 'pure': lion_pure}
MEN = {'flee': man_flee, 'circle': man_circle, 'sphere': man_sphere}
LION_LABEL = {'radius': 'Lion radius', 'pure': 'Lion pure pursuit'}
MAN_LABEL = {'flee': 'Man flee', 'circle': 'Man circle (2D)', 'sphere': 'Man sphere (3D)'}


def run_simulation(lion, man, start):
    L, M = (x.astype(float).copy() for x in ARENA_STARTS[start])
    p_l, e_l = [L.copy()], [M.copy()]
    captured, cap_time = False, None
    for k in range(int(round(MAX_TIME_ARENA / DT_SIM))):
        L_new = LIONS[lion](L, M, V_EQUAL, DT_SIM)
        M = MEN[man](L, M, V_EQUAL, DT_SIM)
        L = L_new
        p_l.append(L.copy()); e_l.append(M.copy())
        if np.linalg.norm(M - L) < CAPTURE_R:
            captured, cap_time = True, (k + 1) * DT_SIM
            break
    p, e = np.array(p_l), np.array(e_l)
    return dict(p=p, e=e, d=np.linalg.norm(e - p, axis=1), t=np.arange(len(p)) * DT_SIM,
                captured=captured, cap_time=cap_time, lion=lion, man=man, start=start)


def wall_contacts(pos):
    return int(np.sum(np.abs(pos) >= ARENA - 1e-6))


def capture_analysis(r):
    """Where the man ended (0 interior, 1 face, 2 edge, 3 corner), his mean
    actual speed over the last 0.5 s, and the time he spent pinned before the
    end (speed < PIN_SPEED)."""
    e = r['e']
    sp = np.linalg.norm(np.diff(e, axis=0), axis=1) / DT_SIM
    k = max(1, int(round(0.5 / DT_SIM)))
    pinned = sp < PIN_SPEED
    run = 0
    for flag in pinned[::-1]:
        if not flag:
            break
        run += 1
    return dict(where=wall_contacts(e[-1]), speed_end=sp[-k:].mean(),
                pinned_time=run * DT_SIM, max_abs=np.abs(np.vstack([r['p'], e])).max())


# ── Plots ────────────────────────────────────────────────────

WHERE = {0: 'in the interior', 1: 'on a face', 2: 'on an edge', 3: 'in a corner'}


def z0_plane(sol3):
    """3D V on the middle plane.  N is even, so there is no node at s_z = 0; the
    two nearest planes s_z = ±Δx/2 hold identical values by symmetry, and their
    average (= linear interpolation to s_z = 0) is that plane."""
    V = sol3['V']; n = V.shape[2]
    return 0.5 * (V[:, :, n // 2 - 1] + V[:, :, n // 2])


def mid_plane_analytic(g):
    """Analytic V on the plane s_z = Δx/2 (where the 3D middle-plane nodes are)."""
    X, Y = np.meshgrid(g, g, indexing='ij')
    h = 0.5 * (g[1] - g[0])
    return analytic_value(np.sqrt(X ** 2 + Y ** 2 + h ** 2))


def plot_value_slices(sol3, out_dir):
    g, V = sol3['g'], sol3['V']
    n = len(g)
    Vz0 = z0_plane(sol3)
    Vy0 = 0.5 * (V[:, n // 2 - 1, :] + V[:, n // 2, :])
    err = Vz0 - mid_plane_analytic(g)
    fig, axes = plt.subplots(1, 3, figsize=(17, 5))
    h = 0.5 * (g[1] - g[0])
    for ax, data, lab in [(axes[0], Vz0, ('s_x (m)', 's_y (m)', f'V on s_z = ±{h:.3f} m')),
                          (axes[1], Vy0, ('s_x (m)', 's_z (m)', f'V on s_y = ±{h:.3f} m'))]:
        ct = ax.contourf(g, g, data.T, levels=30, cmap='viridis')
        plt.colorbar(ct, ax=ax, label='V = time to capture (s)')
        ax.contour(g, g, data.T, levels=ISO_LEVELS, colors='white', linewidths=1.0)
        ax.set_aspect('equal'); ax.set_xlabel(lab[0]); ax.set_ylabel(lab[1])
        ax.set_title(f'{lab[2]} (3D {n}³ grid, v_P=5, v_E=3)\nwhite: V = '
                     + ', '.join(f'{l:g}' for l in ISO_LEVELS) + ' s', fontsize=9)
    ct = axes[2].contourf(g, g, err.T, levels=30, cmap='magma')
    plt.colorbar(ct, ax=axes[2], label='V − V_analytic (s)')
    axes[2].set_aspect('equal'); axes[2].set_xlabel('s_x (m)'); axes[2].set_ylabel('s_y (m)')
    axes[2].set_title(f'Error on s_z = ±{h:.3f} m vs (|s| − r_c)/(v_P − v_E)\n'
                      f'max {np.abs(err).max():.3f} s',
                      fontsize=9)
    fig.suptitle('S009 3D — HJI value function slices (relative coordinates, no walls)',
                 fontsize=11)
    plt.tight_layout()
    path = os.path.join(out_dir, 'value_function_slices.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_isosurfaces(sol3, out_dir):
    fig = plt.figure(figsize=(15, 6.5))
    ax = fig.add_subplot(1, 2, 1, projection='3d')
    ax2 = fig.add_subplot(1, 2, 2)
    cols = ['tab:red', 'tab:orange', 'tab:blue']
    for lev, c in zip(ISO_LEVELS, cols):
        AZ, EL, dirs, rad = isosurface_radii(sol3['g'], sol3['V'], lev)
        r_an = lev * (V_P_ASYM - V_E_ASYM) + CAPTURE_R
        P = dirs * rad[..., None]
        keep = ~((AZ > 0) & (AZ < np.pi / 2))                 # cut away one quadrant
        Px = np.where(keep, P[..., 0], np.nan)
        ax.plot_surface(Px, P[..., 1], P[..., 2], color=c, alpha=0.35, linewidth=0)
        # radius error vs direction: max over elevation, plotted against azimuth
        ax2.plot(np.degrees(EL[0]), ((rad - r_an) / r_an * 100).max(axis=0), '-', color=c,
                 label=f'V = {lev:g} s (analytic r = {r_an:.2f} m): max over azimuth')
        ax2.plot(np.degrees(EL[0]), ((rad - r_an) / r_an * 100).min(axis=0), '--', color=c)
    ax.set_xlabel('s_x'); ax.set_ylabel('s_y'); ax.set_zlabel('s_z')
    ax.set_box_aspect((1, 1, 1))
    ax.set_title('Isosurfaces V = ' + ', '.join(f'{l:g}' for l in ISO_LEVELS) +
                 ' s (one quadrant cut away)', fontsize=10)
    ax2.axhline(0, color='grey')
    ax2.set_xlabel('Elevation of direction (deg)')
    ax2.set_ylabel('Isosurface radius error (%)')
    ax2.set_title('Radius error vs analytic sphere (solid = max, dashed = min over azimuth)',
                  fontsize=10)
    ax2.legend(fontsize=7); ax2.grid(True, alpha=0.3)
    fig.suptitle('S009 3D — level sets of the numerical V are near-spheres', fontsize=11)
    plt.tight_layout()
    path = os.path.join(out_dir, 'isosurfaces.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_convergence(sol3, sol2, sol_eq, out_dir):
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(14, 4.8))
    a1.semilogy(sol3['delta'], color='darkorange', label=f'v_P=5, v_E=3, 3D {N_GRID}³')
    a1.semilogy(sol2['delta'], color='steelblue', label=f'v_P=5, v_E=3, 2D {N_GRID}²')
    a1.semilogy(sol_eq['delta'], color='crimson', label=f'v_P=v_E=4, 3D (τ={TAU_EQUAL})')
    a1.axhline(CONV_TOL, color='grey', linestyle='--', label=f'tolerance {CONV_TOL:g} s')
    a1.set_xlabel('Iteration'); a1.set_ylabel('max |ΔV| (s)')
    a1.set_title('max |ΔV| per iteration', fontsize=10)
    a1.legend(fontsize=8); a1.grid(True, which='both', alpha=0.3)
    a2.plot(sol3['vmax'], color='darkorange', label='v_P=5, v_E=3, 3D')
    a2.plot(sol2['vmax'], color='steelblue', label='v_P=5, v_E=3, 2D')
    a2.plot(sol_eq['vmax'], color='crimson', label='v_P=v_E=4, 3D (grows forever)')
    a2.set_xlabel('Iteration'); a2.set_ylabel('max V on grid (s)')
    a2.set_title('Largest value on the grid', fontsize=10)
    a2.legend(fontsize=8); a2.grid(True, alpha=0.3)
    fig.suptitle('S009 3D — HJI pseudo-time iteration (Godunov upwind)', fontsize=11)
    plt.tight_layout()
    path = os.path.join(out_dir, 'convergence.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def plot_z0_vs_2d(sol3, sol2, out_dir):
    g = sol3['g']
    X, Y = np.meshgrid(g, g, indexing='ij')
    h = 0.5 * (g[1] - g[0])
    V3 = z0_plane(sol3); V2 = sol2['V']
    fig, axes = plt.subplots(1, 3, figsize=(17, 5))
    for ax, data, t, cm in [(axes[0], V3 - mid_plane_analytic(g),
                             f'3D middle plane (s_z = ±{h:.3f} m) − analytic there', 'magma'),
                            (axes[1], V2 - analytic_value(np.hypot(X, Y)),
                             f'2D {N_GRID}² solution − analytic', 'magma')]:
        vm = np.abs(data).max()
        ct = ax.contourf(g, g, data.T, levels=30, cmap=cm)
        plt.colorbar(ct, ax=ax, label='s')
        ax.set_aspect('equal'); ax.set_xlabel('s_x (m)'); ax.set_ylabel('s_y (m)')
        ax.set_title(f'{t}\nmax |·| = {vm:.3f} s, mean = {np.abs(data).mean():.3f} s', fontsize=9)
    j = N_GRID // 2
    ax = axes[2]
    ax.plot(g, V3[:, j], 'o-', color='darkorange', ms=3, label='3D middle plane')
    ax.plot(g, V2[:, j], 's--', color='steelblue', ms=3, label=f'2D {N_GRID}²')
    ax.plot(g, analytic_value(np.hypot(g, g[j])), 'k-', lw=1, label='analytic (2D row)')
    d = V3 - V2
    ax.set_xlabel(f's_x (m), row s_y = {g[j]:.3f} m'); ax.set_ylabel('V (s)')
    ax.set_title(f'One grid row: 3D − 2D = {d.min():.4f} … {d.max():.4f} s everywhere',
                 fontsize=9)
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)
    fig.suptitle('S009 3D — middle plane of the 3D solution vs a 2D solve on the same x-y '
                 'nodes (v_P=5, v_E=3)', fontsize=11)
    plt.tight_layout()
    path = os.path.join(out_dir, 'z0_vs_2d.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_trajectory_sim(open_runs, out_dir):
    fig = plt.figure(figsize=(15, 6))
    ax = fig.add_subplot(1, 2, 1, projection='3d')
    ax2 = fig.add_subplot(1, 2, 2)
    styles = ['-', '--', ':']
    for (lab, r), ls in zip(open_runs, styles):
        p, e = r['p'], r['e']
        ax.plot(p[:, 0], p[:, 1], p[:, 2], color='red', linestyle=ls, linewidth=2,
                label=f'Pursuer, {lab}')
        ax.plot(e[:, 0], e[:, 1], e[:, 2], color='royalblue', linestyle=ls, linewidth=2,
                label=f'Evader, {lab}')
        ax.scatter(*e[0], color='blue', s=30)
        if r['captured']:
            ax.scatter(*p[-1], color='black', marker='X', s=60)
        ax2.plot(r['t'], r['d'], color='black', linestyle=ls, linewidth=2,
                 label=f'{lab}: captured {r["cap_time"]:.3f} s')
    ax.scatter(0, 0, 0, color='red', s=40)
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
    ax.set_title('Open space, both players steer by the numerical 3D ∇V', fontsize=10)
    ax.legend(fontsize=7, loc='upper left')
    t = open_runs[0][1]['t']
    ax2.plot(t, S0 - (V_P_ASYM - V_E_ASYM) * t, 'g--', alpha=0.7, label='6 − 2t (analytic)')
    ax2.axhline(CAPTURE_R, color='red', linestyle=':', label='capture radius')
    ax2.set_xlabel('Time (s)'); ax2.set_ylabel('Distance (m)')
    ax2.set_title('Separation vs time', fontsize=10)
    ax2.legend(fontsize=8); ax2.grid(True, alpha=0.3)
    fig.suptitle('S009 3D — asymmetric game (v_P=5, v_E=3) played from the 3D HJI solution',
                 fontsize=11)
    plt.tight_layout()
    path = os.path.join(out_dir, 'trajectory_sim.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def draw_cube(ax):
    for s1 in (-ARENA, ARENA):
        for s2 in (-ARENA, ARENA):
            kw = dict(color='grey', linewidth=0.6, alpha=0.6)
            ax.plot([-ARENA, ARENA], [s1, s1], [s2, s2], **kw)
            ax.plot([s1, s1], [-ARENA, ARENA], [s2, s2], **kw)
            ax.plot([s1, s1], [s2, s2], [-ARENA, ARENA], **kw)


def plot_arena(arena_res, out_dir):
    keys = [k for k in arena_res if k[2] == 'offset']
    fig = plt.figure(figsize=(20, 11))
    for i, key in enumerate(keys):
        r = arena_res[key]
        ax = fig.add_subplot(2, 4, [1, 2, 3, 5, 6, 7][i], projection='3d')
        draw_cube(ax)
        p, e = r['p'], r['e']
        ax.plot(p[:, 0], p[:, 1], p[:, 2], 'r-', linewidth=1.0, label='Lion')
        ax.plot(e[:, 0], e[:, 1], e[:, 2], color='royalblue', linewidth=1.0, label='Man')
        ax.scatter(*p[0], color='red', s=30); ax.scatter(*e[0], color='blue', s=30)
        ca = capture_analysis(r)
        if r['captured']:
            ax.scatter(*p[-1], color='black', marker='X', s=60)
            st = f'captured {r["cap_time"]:.2f} s, man {WHERE[ca["where"]]}'
        else:
            st = f'no capture in {MAX_TIME_ARENA:.0f} s'
        ax.set_xlim(-ARENA, ARENA); ax.set_ylim(-ARENA, ARENA); ax.set_zlim(-ARENA, ARENA)
        ax.set_title(f'{LION_LABEL[key[0]]} vs {MAN_LABEL[key[1]]}\n{st}', fontsize=9)
        ax.legend(fontsize=7, loc='upper left')
    ax = fig.add_subplot(1, 4, 4)
    for key, c in zip(keys, ['crimson', 'darkorange', 'purple', 'teal', 'olive', 'black']):
        r = arena_res[key]
        ax.semilogy(r['t'], r['d'], color=c, linewidth=1.3, label=f'{key[0]} vs {key[1]}')
    ax.axhline(CAPTURE_R, color='grey', linestyle=':', label='capture radius')
    ax.set_xlabel('Time (s)'); ax.set_ylabel('Lion–man distance (m, log)')
    ax.set_title('Distance vs time (offset start)', fontsize=10)
    ax.legend(fontsize=8); ax.grid(True, which='both', alpha=0.3)
    fig.suptitle(f'S009 3D — equal-speed Lion & Man in the ±{ARENA:.0f} m cube '
                 f'(v = {V_EQUAL:.0f} m/s, r_c = {CAPTURE_R} m), offset start '
                 f'L{ARENA_STARTS["offset"][0].tolist()} M{ARENA_STARTS["offset"][1].tolist()}',
                 fontsize=11)
    plt.tight_layout()
    path = os.path.join(out_dir, 'arena_lion_man.png')
    plt.savefig(path, dpi=130)
    plt.close()
    print(f'Saved: {path}')


def save_animation(cases, out_dir):
    T_ANIM = 12.0
    n_max = int(round(T_ANIM / DT_SIM)) + 1
    step = 5
    n_frames = (n_max - 1) // step + 2
    fig = plt.figure(figsize=(16, 6))
    arts = []
    for k, (lab, r) in enumerate(cases):
        ax = fig.add_subplot(1, len(cases), k + 1, projection='3d')
        draw_cube(ax)
        ax.set_xlim(-ARENA, ARENA); ax.set_ylim(-ARENA, ARENA); ax.set_zlim(-ARENA, ARENA)
        ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
        lp, = ax.plot([], [], [], 'r-', linewidth=1.3)
        le, = ax.plot([], [], [], color='royalblue', linewidth=1.3)
        dp, = ax.plot([], [], [], 'r^', markersize=8)
        de, = ax.plot([], [], [], 'bs', markersize=7)
        ti = ax.set_title(lab, fontsize=9)
        arts.append((lab, r, lp, le, dp, de, ti))

    def update(i):
        out = []
        for lab, r, lp, le, dp, de, ti in arts:
            n = len(r['p']); si = min(i * step, n - 1, n_max - 1)
            p, e = r['p'], r['e']
            lp.set_data(p[:si + 1, 0], p[:si + 1, 1]); lp.set_3d_properties(p[:si + 1, 2])
            le.set_data(e[:si + 1, 0], e[:si + 1, 1]); le.set_3d_properties(e[:si + 1, 2])
            dp.set_data([p[si, 0]], [p[si, 1]]); dp.set_3d_properties([p[si, 2]])
            de.set_data([e[si, 0]], [e[si, 1]]); de.set_3d_properties([e[si, 2]])
            if r['captured'] and si == n - 1:
                ti.set_text(f'{lab}\ncaptured {r["cap_time"]:.2f} s')
            else:
                ti.set_text(f'{lab}\nt={r["t"][si]:.2f} s  dist={r["d"][si]:.2f} m')
            out += [lp, le, dp, de, ti]
        return out

    fig.suptitle(f'S009 3D — Lion & Man in the cube, offset start (first {T_ANIM:.0f} s; '
                 'red ▲ lion, blue ■ man)', fontsize=10)
    ani = FuncAnimation(fig, update, frames=n_frames, interval=60, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer=PillowWriter(fps=16), dpi=75)
    plt.close()
    print(f'Saved: {path}  ({n_frames} frames, every {step} steps)')


# ── Main ─────────────────────────────────────────────────────

def _summary(name, sol, dim):
    err = np.abs(sol['V'] - analytic_value(sol['R']))
    print(f'  {name}: {"converged" if sol["converged"] else "NOT converged"} after '
          f'{sol["iters"]} iterations (final max|ΔV| = {sol["delta"][-1]:.2e} s), '
          f'dx = {sol["g"][1] - sol["g"][0]:.4f} m, τ = {sol["tau"]:.4f} s')
    print(f'    max |V - V_analytic| = {err.max():.4f} s, mean = {err.mean():.4f} s, max V = '
          f'{sol["V"].max():.3f} s (analytic at grid corner {analytic_value(sol["R"].max()):.3f} s)')


if __name__ == '__main__':
    out_dir = os.path.normpath(OUTPUT_DIR)
    os.makedirs(out_dir, exist_ok=True)

    print('Part A — HJI in relative coordinates (no walls)')
    sol3 = solve_hji(V_P_ASYM, V_E_ASYM, N_GRID, 3)
    _summary(f'3D {N_GRID}^3', sol3, 3)
    sol2 = solve_hji(V_P_ASYM, V_E_ASYM, N_GRID, 2)
    _summary(f'2D {N_GRID}^2 (comparison)', sol2, 2)
    sol_eq = solve_hji(V_EQUAL, V_EQUAL, N_GRID, 3, max_iter=ITER_EQUAL, tau=TAU_EQUAL)
    print(f'  equal speeds 3D: after {sol_eq["iters"]} iterations max|ΔV| min '
          f'{sol_eq["delta"].min():.4f}, max {sol_eq["delta"].max():.4f} s (= τ every '
          f'iteration); max V = {sol_eq["vmax"][-1]:.2f} s = {sol_eq["iters"]} × τ → V = ∞')

    g = sol3['g']
    n = N_GRID
    X, Y = np.meshgrid(g, g, indexing='ij')
    Van2 = analytic_value(np.hypot(X, Y))
    Van3 = mid_plane_analytic(g)
    V3z = z0_plane(sol3)
    d3 = V3z - sol2['V']
    print(f'  middle planes s_z = ±{g[n // 2]:.4f} m identical: '
          f'{np.array_equal(sol3["V"][:, :, n // 2 - 1], sol3["V"][:, :, n // 2])}')
    print(f'  middle plane: |3D - analytic(s_z = {g[n // 2]:.4f})| max {np.abs(V3z - Van3).max():.4f} s, '
          f'mean {np.abs(V3z - Van3).mean():.4f} s; |2D - analytic| max '
          f'{np.abs(sol2["V"] - Van2).max():.4f} s, mean {np.abs(sol2["V"] - Van2).mean():.4f} s; '
          f'3D - 2D: min {d3.min():.4f} s, max {d3.max():.4f} s')
    fixed3 = sol3['R'][:, :, n // 2] < CAPTURE_R + (g[1] - g[0])
    print(f'  fixed capture-set nodes on the middle plane: {int(fixed3.sum())}; their 3D value '
          f'{np.unique(np.round(sol3["V"][:, :, n // 2][fixed3], 4))} s vs 2D value '
          f'{np.unique(np.round(sol2["V"][fixed3], 4))} s')

    X3, Y3, Z3 = np.meshgrid(g, g, g, indexing='ij')
    gx, gy, gz = np.gradient(sol3['V'], g[1] - g[0])
    R3 = sol3['R']; mask = R3 > 1.0
    cosang = (gx * X3 + gy * Y3 + gz * Z3)[mask] / (np.sqrt(gx ** 2 + gy ** 2 + gz ** 2)[mask]
                                                    * R3[mask])
    ang = np.degrees(np.arccos(np.clip(cosang, -1, 1)))
    print(f'  angle between numerical 3D ∇V and s/|s| (|s| > 1 m): mean {ang.mean():.2f}°, '
          f'max {ang.max():.2f}°')
    for lev in ISO_LEVELS:
        _, EL, _, rad = isosurface_radii(g, sol3['V'], lev)
        r_an = lev * (V_P_ASYM - V_E_ASYM) + CAPTURE_R
        print(f'  isosurface V = {lev:g} s: radius {rad.min():.3f}–{rad.max():.3f} m '
              f'(analytic {r_an:.3f} m; error {((rad - r_an) / r_an * 100).min():+.1f}% to '
              f'{((rad - r_an) / r_an * 100).max():+.1f}%)')

    grads = gradient_interpolators(g, sol3['V'])
    t_an = (S0 - CAPTURE_R) / (V_P_ASYM - V_E_ASYM)
    open_runs = []
    for lab, az, el in OPEN_STARTS:
        u0 = direction(az, el)
        r = run_open_game(u0, grads)
        open_runs.append((lab, r))
        de = r['e'] - r['e0']
        perp = np.linalg.norm(de - np.outer(de @ u0, u0), axis=1)
        head = np.degrees(np.arccos(np.clip(unit(de[-1]) @ u0, -1, 1)))
        print(f'  open-space 3D ∇V game, evader start {lab} {np.round(r["e0"], 3)}: '
              f'captured={r["captured"]} @ {r["cap_time"]:.3f} s (analytic {t_an:.3f} s) | '
              f'evader max deviation from the straight escape line {perp.max():.3f} m, '
              f'net heading error {head:.2f}°')

    print(f'\nPart B — equal-speed Lion & Man in the ±{ARENA:.0f} m cube (v = {V_EQUAL} m/s, '
          f'r_c = {CAPTURE_R} m, max {MAX_TIME_ARENA:.0f} s)')
    arena_res = {}
    for start in ARENA_STARTS:
        L0, M0 = ARENA_STARTS[start]
        print(f'  start {start}: lion {L0.tolist()}, man {M0.tolist()}')
        for lion in LIONS:
            for man in MEN:
                r = run_simulation(lion, man, start)
                arena_res[(lion, man, start)] = r
                ca = capture_analysis(r)
                if r['captured']:
                    st = (f'captured @ {r["cap_time"]:.3f} s, man {WHERE[ca["where"]]} '
                          f'{np.round(r["e"][-1], 2)}, his speed over the last 0.5 s '
                          f'{ca["speed_end"]:.2f} m/s, pinned (<{PIN_SPEED:g} m/s) for the last '
                          f'{ca["pinned_time"]:.2f} s')
                else:
                    last10 = r['d'][r['t'] >= r['t'][-1] - 10]
                    st = (f'no capture; distance in the last 10 s {last10.min():.3f}–'
                          f'{last10.max():.3f} m')
                zr = r['e'][:, 2]
                print(f'    lion={lion:<6} man={man:<6}: {st} | man z range '
                      f'{zr.min():.2f}–{zr.max():.2f} m | max |coord| {ca["max_abs"]:.3f}')

    plot_value_slices(sol3, out_dir)
    plot_isosurfaces(sol3, out_dir)
    plot_convergence(sol3, sol2, sol_eq, out_dir)
    plot_z0_vs_2d(sol3, sol2, out_dir)
    plot_trajectory_sim(open_runs, out_dir)
    plot_arena(arena_res, out_dir)
    save_animation([('Radius lion vs sphere man', arena_res[('radius', 'sphere', 'offset')]),
                    ('Pure-pursuit lion vs sphere man', arena_res[('pure', 'sphere', 'offset')]),
                    ('Pure-pursuit lion vs flee man', arena_res[('pure', 'flee', 'offset')])],
                   out_dir)
    print('Done.')
