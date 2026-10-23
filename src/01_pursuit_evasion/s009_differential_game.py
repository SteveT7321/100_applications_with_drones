"""
S009 Differential Game 1v1 — Lion & Man (HJI Numerical Solution)
=================================================================
Part A — HJI in relative coordinates s = p_E - p_P (no walls)
  Stationary HJI for the time-to-capture value V(s):
      min_{u_P} max_{u_E} ∇V·(v_E u_E - v_P u_P) + 1 = 0
      ⇒  (v_P - v_E)|∇V| = 1,   V = 0 on |s| <= r_c
  Both optimal controls point along ∇V/|∇V| (pursuer toward the evader,
  evader straight away).  Analytic solution: V = (|s| - r_c)/(v_P - v_E).

  Solved by pseudo-time iteration with a monotone Godunov upwind gradient:
      V <- V + τ (1 - (v_P - v_E) |∇V|_God)
  starting from V = 0 and growing outward from the capture set.
  - v_P > v_E: converges to the analytic V (2-D 50×50 and 3-D 30³ grids).
  - v_P = v_E: |∇V| drops out, V grows by τ every iteration → V = ∞
    (no finite capture time in open space).
  The asymmetric game (v_P = 5, v_E = 3) is then played with both players
  steering by the interpolated numerical ∇V, from an on-axis start and two
  off-axis starts (OFF_AXIS_DEG) where the grid gradient is least accurate.

Part B — Equal-speed Lion & Man in the bounded arena [-5, 5]² (v = 4 m/s,
  capture radius 0.15 m).  The relative-state HJI ignores walls; the full
  bounded game has a 4-D state and is not solved on a grid here (runtime).
  Instead four strategy pairs are simulated with the walls enforced:
    Lion: radius strategy (stay on the segment centre→man, as close to the
          man as possible) | pure pursuit (= open-space HJI control)
    Man:  circle (run tangentially on a 4.5 m circle about the centre) |
          flee (straight away from the lion = open-space HJI control,
          sliding along walls)

Usage:
    conda activate drones
    python src/01_pursuit_evasion/s009_differential_game.py
"""

import sys
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.interpolate import RegularGridInterpolator

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

# ── Parameters ───────────────────────────────────────────────
ARENA      = 5.0     # m — arena is [-ARENA, ARENA]²
S_MAX      = 2 * ARENA   # m — relative-state grid covers [-S_MAX, S_MAX]^d
N_GRID_2D  = 50
N_GRID_3D  = 30
CFL        = 0.9     # τ = CFL · dx / (d · (v_P - v_E))
MAX_ITER   = 2000
CONV_TOL   = 1e-6    # s — max |ΔV| per iteration
TAU_EQUAL  = 0.05    # s — pseudo-time step used to show divergence at v_P = v_E
ITER_EQUAL = 200
CAPTURE_R  = 0.15    # m
DT_SIM     = 1 / 48
MAX_TIME_OPEN  = 20.0   # s
MAX_TIME_ARENA = 150.0  # s
OFF_AXIS_DEG = [22.5, 45.0]   # deg — extra open-space starts off the grid axes

V_P_ASYM, V_E_ASYM = 5.0, 3.0
V_EQUAL = 4.0

INIT_PURSUER = np.array([-3.0, 0.0])
INIT_EVADER  = np.array([3.0, 0.0])
ARENA_CENTRE = np.array([0.0, 0.0])
CIRCLE_R     = 4.5   # m — circling man's target radius
CIRCLE_GAIN  = 2.0   # 1/m — radial correction gain of the circling man

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '01_pursuit_evasion', 's009_differential_game',
)


# ── Part A: HJI solver ───────────────────────────────────────

def godunov_grad_norm(V, dx):
    """Upwind (Godunov) |∇V| for an increasing-outward value function."""
    d = V.ndim
    Vp = np.pad(V, 1, mode='edge')
    sq = np.zeros_like(V)
    for ax in range(d):
        sl_c = [slice(1, -1)] * d
        sl_m = list(sl_c); sl_m[ax] = slice(0, -2)
        sl_p = list(sl_c); sl_p[ax] = slice(2, None)
        c = Vp[tuple(sl_c)]
        dm = (c - Vp[tuple(sl_m)]) / dx      # backward difference
        dp = (Vp[tuple(sl_p)] - c) / dx      # forward difference
        sq += np.maximum(np.maximum(dm, -dp), 0.0) ** 2
    return np.sqrt(sq)


def solve_hji(v_p, v_e, n, dim, max_iter=MAX_ITER, tau=None):
    """
    Pseudo-time iteration of the stationary HJI on an n^dim grid over
    [-S_MAX, S_MAX]^dim.  Nodes with |s| < r_c + dx are fixed to the exact
    value max(|s| - r_c, 0)/(v_P - v_E) (or 0 for v_P = v_E) to represent the
    sub-grid capture circle.

    Returns axes, V, history of max|ΔV|, history of max V, iterations, converged
    """
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

    deltas, vmax = [], []
    converged = False
    for it in range(max_iter):
        gn = godunov_grad_norm(V, dx)
        V_new = V + tau * (1.0 - dv * gn)
        V_new[fixed] = V[fixed]
        delta = np.max(np.abs(V_new - V))
        V = V_new
        deltas.append(delta)
        vmax.append(V.max())
        if dv > 0 and delta < CONV_TOL:
            converged = True
            break
    return g, V, np.array(deltas), np.array(vmax), it + 1, converged


def analytic_value(R, v_p, v_e):
    return np.maximum(R - CAPTURE_R, 0.0) / (v_p - v_e)


def gradient_interpolators(g, V):
    dx = g[1] - g[0]
    gx, gy = np.gradient(V, dx)
    ix = RegularGridInterpolator((g, g), gx, bounds_error=False, fill_value=None)
    iy = RegularGridInterpolator((g, g), gy, bounds_error=False, fill_value=None)
    return ix, iy


# ── Strategies ───────────────────────────────────────────────

def unit(v):
    n = np.linalg.norm(v)
    return v / n if n > 1e-12 else np.zeros_like(v)


def _clip_out(pos, d):
    d = d.copy()
    for k in range(2):
        if pos[k] >= ARENA - 1e-9 and d[k] > 0:
            d[k] = 0.0
        if pos[k] <= -ARENA + 1e-9 and d[k] < 0:
            d[k] = 0.0
    return d


def wall_slide(pos, d):
    """
    Remove velocity components that push through an arena wall.  If the
    desired direction is exactly into the wall (nothing left), slide along
    the wall instead (counter-clockwise turn of d); in a corner this can
    still leave zero velocity.
    """
    out = _clip_out(pos, d)
    if np.linalg.norm(out) < 1e-9:
        out = _clip_out(pos, np.array([-d[1], d[0]]))
    return unit(out)


def lion_radius(L, M, v, dt):
    """
    Radius strategy: move to the point of segment [c, M] that is closest to
    the man and reachable within v·dt; if the segment is out of reach, move
    toward its nearest point.  Returns the new lion position.
    """
    c = ARENA_CENTRE
    seg = M - c
    seg_len2 = seg @ seg
    step = v * dt
    if seg_len2 < 1e-12:
        return L + step * unit(M - L)
    lam0 = np.clip((L - c) @ seg / seg_len2, 0.0, 1.0)
    foot = c + lam0 * seg
    dist_foot = np.linalg.norm(L - foot)
    if dist_foot > step:
        return L + step * unit(foot - L)
    # points c + λ seg with |c + λ seg - L| <= step: solve quadratic in λ
    a = seg_len2
    b = 2 * seg @ (c - L)
    cc = (c - L) @ (c - L) - step ** 2
    disc = max(b * b - 4 * a * cc, 0.0)
    lam_hi = min((-b + np.sqrt(disc)) / (2 * a), 1.0)
    return c + lam_hi * seg


def lion_pure(L, M, v, dt):
    return L + v * dt * unit(M - L)


def man_circle(L, M, v, dt):
    r = M - ARENA_CENTRE
    rho = np.linalg.norm(r)
    r_hat = unit(r) if rho > 1e-9 else np.array([1.0, 0.0])
    t_hat = np.array([-r_hat[1], r_hat[0]])           # counter-clockwise
    d = unit(t_hat + CIRCLE_GAIN * (CIRCLE_R - rho) * r_hat)
    d = wall_slide(M, d)
    return np.clip(M + v * dt * d, -ARENA, ARENA)


def man_flee(L, M, v, dt):
    d = wall_slide(M, unit(M - L))
    return np.clip(M + v * dt * d, -ARENA, ARENA)


LIONS = {'radius': lion_radius, 'pure': lion_pure}
MEN = {'circle': man_circle, 'flee': man_flee}
LION_LABEL = {'radius': 'Lion: radius strategy', 'pure': 'Lion: pure pursuit'}
MAN_LABEL = {'circle': 'Man: circle (4.5 m)', 'flee': 'Man: flee (HJI open-space)'}


# ── Simulation ───────────────────────────────────────────────

def run_simulation(mode, lion='radius', man='circle', grad_interp=None, init_evader=None):
    """
    mode = 'open'  : asymmetric speeds (V_P_ASYM, V_E_ASYM), no walls, both
                     players steer by the interpolated numerical ∇V(s).
    mode = 'arena' : equal speeds V_EQUAL in [-ARENA, ARENA]², strategies
                     LIONS[lion] vs MEN[man].
    init_evader: evader start (default INIT_EVADER).
    Returns dict(p, e, d, t, captured, cap_time).
    """
    M0 = INIT_EVADER if init_evader is None else init_evader
    L, M = INIT_PURSUER.astype(float).copy(), np.asarray(M0, dtype=float).copy()
    p_list, e_list = [L.copy()], [M.copy()]
    if mode == 'open':
        ix, iy = grad_interp
        max_t = MAX_TIME_OPEN
    else:
        max_t = MAX_TIME_ARENA
    captured, cap_time = False, None
    for k in range(int(round(max_t / DT_SIM))):
        if mode == 'open':
            s = M - L
            g = np.array([ix([s])[0], iy([s])[0]])
            u = unit(g)
            L = L + V_P_ASYM * DT_SIM * u
            M = M + V_E_ASYM * DT_SIM * u
        else:
            L_new = LIONS[lion](L, M, V_EQUAL, DT_SIM)
            M = MEN[man](L, M, V_EQUAL, DT_SIM)
            L = L_new
        p_list.append(L.copy())
        e_list.append(M.copy())
        if np.linalg.norm(M - L) < CAPTURE_R:
            captured, cap_time = True, (k + 1) * DT_SIM
            break
    p, e = np.array(p_list), np.array(e_list)
    return dict(p=p, e=e, d=np.linalg.norm(e - p, axis=1),
                t=np.arange(len(p)) * DT_SIM, captured=captured, cap_time=cap_time)


# ── Plots ────────────────────────────────────────────────────

def plot_value_functions(sol2, sol3, out_dir):
    g2, V2 = sol2['g'], sol2['V']
    X, Y = np.meshgrid(g2, g2, indexing='ij')
    R2 = np.sqrt(X ** 2 + Y ** 2)
    err2 = np.abs(V2 - analytic_value(R2, V_P_ASYM, V_E_ASYM))

    fig, axes = plt.subplots(1, 3, figsize=(18, 5.2))
    ax = axes[0]
    ct = ax.contourf(g2, g2, V2.T, levels=30, cmap='viridis')
    plt.colorbar(ct, ax=ax, label='V = time to capture (s)')
    ax.contour(g2, g2, V2.T, levels=np.arange(0.5, 8, 0.5), colors='white',
               linewidths=0.5, alpha=0.6)
    th = np.linspace(0, 2 * np.pi, 100)
    ax.plot(CAPTURE_R * np.cos(th), CAPTURE_R * np.sin(th), 'r-', linewidth=1.5)
    ax.set_aspect('equal')
    ax.set_xlabel('Δx = x_E − x_P (m)'); ax.set_ylabel('Δy = y_E − y_P (m)')
    ax.set_title(f'Numerical V(s), 2-D {N_GRID_2D}×{N_GRID_2D}\n'
                 f'v_P={V_P_ASYM:.0f}, v_E={V_E_ASYM:.0f} m/s', fontsize=10)

    ax = axes[1]
    ct = ax.contourf(g2, g2, err2.T, levels=30, cmap='magma')
    plt.colorbar(ct, ax=ax, label='|V − V_analytic| (s)')
    ax.set_aspect('equal')
    ax.set_xlabel('Δx (m)'); ax.set_ylabel('Δy (m)')
    ax.set_title('Error vs analytic (|s| − r_c)/(v_P − v_E)\n'
                 f'max {err2.max():.3f} s', fontsize=10)

    ax = axes[2]
    g3, V3 = sol3['g'], sol3['V']
    X3, Y3, Z3 = np.meshgrid(g3, g3, g3, indexing='ij')
    R3 = np.sqrt(X3 ** 2 + Y3 ** 2 + Z3 ** 2)
    r_line = np.linspace(0, S_MAX * np.sqrt(3), 200)
    ax.plot(r_line, analytic_value(r_line, V_P_ASYM, V_E_ASYM), 'k-', linewidth=2,
            label='Analytic (|s| − r_c)/(v_P − v_E)')
    ax.scatter(R2.ravel(), V2.ravel(), s=3, color='steelblue', alpha=0.4,
               label=f'2-D grid {N_GRID_2D}²')
    ax.scatter(R3.ravel(), V3.ravel(), s=2, color='darkorange', alpha=0.2,
               label=f'3-D grid {N_GRID_3D}³')
    ax.set_xlabel('|s| (m)'); ax.set_ylabel('V (s)')
    ax.set_title('Value vs distance: every grid node', fontsize=10)
    ax.legend(fontsize=8, markerscale=4)
    ax.grid(True, alpha=0.3)

    fig.suptitle('S009 — HJI time-to-capture value function (relative coordinates, '
                 'no walls)', fontsize=11)
    plt.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'value_functions.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_convergence(sol2, sol3, sol_eq, out_dir):
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 4.8))
    ax1.semilogy(sol2['delta'], color='steelblue', label=f'v_P=5, v_E=3, 2-D {N_GRID_2D}²')
    ax1.semilogy(sol3['delta'], color='darkorange', label=f'v_P=5, v_E=3, 3-D {N_GRID_3D}³')
    ax1.semilogy(sol_eq['delta'], color='crimson', label=f'v_P=v_E=4, 2-D (τ={TAU_EQUAL})')
    ax1.axhline(CONV_TOL, color='grey', linestyle='--', label=f'tolerance {CONV_TOL:g} s')
    ax1.set_xlabel('Iteration'); ax1.set_ylabel('max |ΔV| per iteration (s)')
    ax1.set_title('Convergence of the pseudo-time iteration', fontsize=10)
    ax1.legend(fontsize=8); ax1.grid(True, which='both', alpha=0.3)

    ax2.plot(sol2['vmax'], color='steelblue', label='v_P=5, v_E=3, 2-D')
    ax2.plot(sol3['vmax'], color='darkorange', label='v_P=5, v_E=3, 3-D')
    ax2.plot(sol_eq['vmax'], color='crimson', label='v_P=v_E=4, 2-D (grows forever)')
    ax2.set_xlabel('Iteration'); ax2.set_ylabel('max V on grid (s)')
    ax2.set_title('Largest value on the grid', fontsize=10)
    ax2.legend(fontsize=8); ax2.grid(True, alpha=0.3)
    fig.suptitle('S009 — HJI value iteration', fontsize=11)
    plt.tight_layout()
    path = os.path.join(out_dir, 'convergence.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def plot_optimal_trajectories(open_runs, out_dir):
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5.5))
    styles = ['-', '--', ':']
    for (lab, r), ls in zip(open_runs, styles):
        p, e = r['p'], r['e']
        ax1.plot(p[:, 0], p[:, 1], color='red', linestyle=ls, linewidth=2,
                 label=f'Pursuer, {lab}')
        ax1.plot(e[:, 0], e[:, 1], color='royalblue', linestyle=ls, linewidth=2,
                 label=f'Evader, {lab}')
        ax1.scatter(*p[0], color='red', s=50); ax1.scatter(*e[0], color='blue', s=50)
        if r['captured']:
            ax1.scatter(*p[-1], color='black', marker='X', s=80)
        ax2.plot(r['t'], r['d'], color='black', linestyle=ls, linewidth=2,
                 label=f'{lab}: captured {r["cap_time"]:.3f} s')
    ax1.add_patch(plt.Rectangle((-ARENA, -ARENA), 2 * ARENA, 2 * ARENA, fill=False,
                                edgecolor='grey', linestyle='--',
                                label='±5 m arena (not enforced here)'))
    ax1.set_aspect('equal'); ax1.grid(True, alpha=0.3)
    ax1.set_xlabel('X (m)'); ax1.set_ylabel('Y (m)')
    ax1.set_title('Open space, both players steer by numerical ∇V (X = capture)', fontsize=10)
    ax1.legend(fontsize=7, loc='upper left')

    t_line = open_runs[0][1]['t']
    ax2.plot(t_line, np.linalg.norm(INIT_EVADER - INIT_PURSUER)
             - (V_P_ASYM - V_E_ASYM) * t_line, 'g--', alpha=0.7,
             label='|s₀| − (v_P − v_E) t')
    ax2.axhline(CAPTURE_R, color='red', linestyle=':', label='capture radius')
    ax2.set_xlabel('Time (s)'); ax2.set_ylabel('Distance (m)')
    ax2.set_title('Separation vs time', fontsize=10)
    ax2.legend(fontsize=8); ax2.grid(True, alpha=0.3)
    fig.suptitle('S009 — Asymmetric game (v_P=5, v_E=3) played from the HJI solution',
                 fontsize=11)
    plt.tight_layout()
    path = os.path.join(out_dir, 'optimal_trajectories.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def plot_arena(arena_res, out_dir):
    fig = plt.figure(figsize=(16, 9))
    keys = list(arena_res.keys())
    for i, key in enumerate(keys):
        r = arena_res[key]
        ax = fig.add_subplot(2, 3, [1, 2, 4, 5][i])
        p, e = r['p'], r['e']
        ax.plot(p[:, 0], p[:, 1], 'r-', linewidth=1.2, label='Lion')
        ax.plot(e[:, 0], e[:, 1], color='royalblue', linewidth=1.2, label='Man')
        ax.scatter(*p[0], color='red', s=40); ax.scatter(*e[0], color='blue', s=40)
        if r['captured']:
            ax.scatter(*p[-1], color='black', marker='X', s=80)
            st = f'captured {r["cap_time"]:.2f} s'
        else:
            st = f'no capture in {MAX_TIME_ARENA:.0f} s, final gap {r["d"][-1]:.2f} m'
        ax.add_patch(plt.Rectangle((-ARENA, -ARENA), 2 * ARENA, 2 * ARENA, fill=False,
                                   edgecolor='grey'))
        ax.set_xlim(-ARENA - 0.5, ARENA + 0.5); ax.set_ylim(-ARENA - 0.5, ARENA + 0.5)
        ax.set_aspect('equal'); ax.grid(True, alpha=0.3)
        ax.set_title(f'{LION_LABEL[key[0]]} vs {MAN_LABEL[key[1]]}\n{st}', fontsize=9)
        ax.legend(fontsize=7, loc='lower left')
    ax = fig.add_subplot(1, 3, 3)
    for key, c in zip(keys, ['crimson', 'darkorange', 'purple', 'teal']):
        r = arena_res[key]
        ax.semilogy(r['t'], r['d'], color=c, linewidth=1.4,
                    label=f'{key[0]} vs {key[1]}')
    ax.axhline(CAPTURE_R, color='black', linestyle=':', label='capture radius')
    ax.set_xlabel('Time (s)'); ax.set_ylabel('Lion–man distance (m, log)')
    ax.set_title('Distance vs time', fontsize=10)
    ax.legend(fontsize=8); ax.grid(True, which='both', alpha=0.3)
    fig.suptitle(f'S009 — Equal-speed Lion & Man in the ±{ARENA:.0f} m arena '
                 f'(v = {V_EQUAL:.0f} m/s, capture radius {CAPTURE_R} m)', fontsize=11)
    plt.tight_layout()
    path = os.path.join(out_dir, 'arena_lion_man.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def save_animation(open_res, arena_res, out_dir):
    import matplotlib.animation as animation
    cases = [('Open space, v_P=5 > v_E=3 (∇V vs ∇V)', open_res),
             ('Arena, equal speed: radius vs circle', arena_res[('radius', 'circle')]),
             ('Arena, equal speed: pure pursuit vs circle', arena_res[('pure', 'circle')])]
    T_ANIM = 12.0
    n_max = int(round(T_ANIM / DT_SIM)) + 1
    step = int(np.ceil(n_max / 140))
    n_frames = (n_max - 1) // step + 2

    fig, axes = plt.subplots(1, 3, figsize=(17, 5.5))
    arts = []
    for ax, (lab, r) in zip(axes, cases):
        if r is open_res:
            pts = np.vstack([r['p'], r['e']])
            lo, hi = pts.min(axis=0) - 1, pts.max(axis=0) + 1
            ax.set_xlim(lo[0], hi[0]); ax.set_ylim(min(lo[1], -3), max(hi[1], 3))
        else:
            ax.set_xlim(-ARENA - 0.5, ARENA + 0.5); ax.set_ylim(-ARENA - 0.5, ARENA + 0.5)
            ax.add_patch(plt.Rectangle((-ARENA, -ARENA), 2 * ARENA, 2 * ARENA,
                                       fill=False, edgecolor='grey'))
        ax.set_aspect('equal'); ax.grid(True, alpha=0.3)
        lp, = ax.plot([], [], 'r-', linewidth=1.4)
        le, = ax.plot([], [], color='royalblue', linewidth=1.4)
        dp, = ax.plot([], [], 'r^', markersize=8)
        de, = ax.plot([], [], 'bs', markersize=8)
        ti = ax.set_title(lab, fontsize=9)
        arts.append((lab, r, lp, le, dp, de, ti))

    def update(i):
        out = []
        for lab, r, lp, le, dp, de, ti in arts:
            n = len(r['p'])
            si = min(i * step, n - 1, n_max - 1)
            lp.set_data(r['p'][:si + 1, 0], r['p'][:si + 1, 1])
            le.set_data(r['e'][:si + 1, 0], r['e'][:si + 1, 1])
            dp.set_data([r['p'][si, 0]], [r['p'][si, 1]])
            de.set_data([r['e'][si, 0]], [r['e'][si, 1]])
            if r['captured'] and si == n - 1:
                ti.set_text(f'{lab}\ncaptured {r["cap_time"]:.2f} s')
            else:
                ti.set_text(f'{lab}\nt={r["t"][si]:.2f} s  dist={r["d"][si]:.2f} m')
            out += [lp, le, dp, de, ti]
        return out

    ani = animation.FuncAnimation(fig, update, frames=n_frames, interval=60, blit=False)
    fig.suptitle(f'S009 Differential Game (first {T_ANIM:.0f} s; red ▲ pursuer/lion, '
                 'blue ■ evader/man)', fontsize=10)
    plt.tight_layout()
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer='pillow', fps=16, dpi=80)
    plt.close()
    print(f'Saved: {path}  ({n_frames} frames, every {step} steps)')


# ── Main ─────────────────────────────────────────────────────

def _summary(name, g, V, delta, it, conv, dim):
    mesh = np.meshgrid(*([g] * dim), indexing='ij')
    R = np.sqrt(sum(m ** 2 for m in mesh))
    err = np.abs(V - analytic_value(R, V_P_ASYM, V_E_ASYM))
    print(f'  {name}: {"converged" if conv else "NOT converged"} after {it} iterations '
          f'(final max|ΔV| = {delta[-1]:.2e} s), dx = {g[1]-g[0]:.4f} m')
    print(f'    max |V - V_analytic| = {err.max():.4f} s, mean = {err.mean():.4f} s, '
          f'max V = {V.max():.3f} s (analytic at corner {analytic_value(R.max(), V_P_ASYM, V_E_ASYM):.3f} s)')


if __name__ == '__main__':
    print('Part A — HJI in relative coordinates (no walls)')
    g2, V2, d2, m2, it2, c2 = solve_hji(V_P_ASYM, V_E_ASYM, N_GRID_2D, 2)
    sol2 = dict(g=g2, V=V2, delta=d2, vmax=m2)
    _summary(f'2-D {N_GRID_2D}x{N_GRID_2D}', g2, V2, d2, it2, c2, 2)
    g3, V3, d3, m3, it3, c3 = solve_hji(V_P_ASYM, V_E_ASYM, N_GRID_3D, 3)
    sol3 = dict(g=g3, V=V3, delta=d3, vmax=m3)
    _summary(f'3-D {N_GRID_3D}^3', g3, V3, d3, it3, c3, 3)

    ge, Ve, de, me, ite, ce = solve_hji(V_EQUAL, V_EQUAL, N_GRID_2D, 2,
                                        max_iter=ITER_EQUAL, tau=TAU_EQUAL)
    sol_eq = dict(g=ge, V=Ve, delta=de, vmax=me)
    print(f'  equal speeds 2-D: after {ite} iterations max|ΔV| = {de[-1]:.4f} s '
          f'(= τ every iteration: min {de.min():.4f}, max {de.max():.4f}), '
          f'max V = {me[-1]:.2f} s = {ite} × τ → V grows without bound (no capture)')

    # direction of numerical ∇V vs the analytic direction s/|s|
    X, Y = np.meshgrid(g2, g2, indexing='ij')
    gx, gy = np.gradient(V2, g2[1] - g2[0])
    R2 = np.sqrt(X ** 2 + Y ** 2)
    mask = R2 > 1.0
    cosang = (gx * X + gy * Y)[mask] / (np.hypot(gx, gy)[mask] * R2[mask])
    ang = np.degrees(np.arccos(np.clip(cosang, -1, 1)))
    print(f'  angle between numerical ∇V and s/|s| (|s| > 1 m): mean {ang.mean():.2f}°, '
          f'max {ang.max():.2f}°')

    # error near the axes (N even: no node lies exactly on an axis)
    err2 = np.abs(V2 - analytic_value(R2, V_P_ASYM, V_E_ASYM))
    near = np.min(np.abs(g2))
    row = (np.isclose(np.abs(X), near) | np.isclose(np.abs(Y), near)) & (R2 > 1.0)
    axis_ang = np.degrees(np.arctan2(np.minimum(np.abs(X), np.abs(Y)),
                                     np.maximum(np.abs(X), np.abs(Y))))
    within5 = (axis_ang <= 5.0) & (R2 > 1.0)
    diag = (np.abs(axis_ang - 45.0) <= 5.0) & (R2 > 1.0)
    print(f'  V error on the grid rows nearest the axes (|x| or |y| = {near:.4f} m, '
          f'|s| > 1 m): max {err2[row].max():.4f} s, mean {err2[row].mean():.4f} s')
    print(f'  V error within 5° of an axis: mean {err2[within5].mean():.4f} s '
          f'({within5.sum()} nodes); within 5° of a diagonal: mean '
          f'{err2[diag].mean():.4f} s ({diag.sum()} nodes)')

    grads = gradient_interpolators(g2, V2)
    s0 = np.linalg.norm(INIT_EVADER - INIT_PURSUER)
    t_an = (s0 - CAPTURE_R) / (V_P_ASYM - V_E_ASYM)
    open_runs = []
    for deg in [0.0] + OFF_AXIS_DEG:
        th = np.radians(deg)
        e0 = INIT_PURSUER + s0 * np.array([np.cos(th), np.sin(th)])
        r = run_simulation('open', grad_interp=grads, init_evader=e0)
        open_runs.append((f'start at {deg:g}°', r))
        u0 = (e0 - INIT_PURSUER) / s0
        de = r['e'] - e0
        perp = np.abs(u0[0] * de[:, 1] - u0[1] * de[:, 0])   # distance from the start line
        head = np.degrees(np.arccos(np.clip(
            (r['e'][-1] - e0) @ u0 / np.linalg.norm(r['e'][-1] - e0), -1, 1)))
        print(f'  open-space game from ∇V, evader start at {deg:g}° ({e0[0]:.3f}, {e0[1]:.3f}): '
              f'captured={r["captured"]} @ {r["cap_time"]:.3f} s (analytic {t_an:.3f} s) | '
              f'evader max deviation from the straight escape line {perp.max():.3f} m, '
              f'net heading error {head:.2f}° | evader final ({r["e"][-1, 0]:.2f}, '
              f'{r["e"][-1, 1]:.2f})')
    open_res = open_runs[0][1]

    print(f'\nPart B — equal-speed Lion & Man in the ±{ARENA:.0f} m arena '
          f'(v = {V_EQUAL} m/s, r_c = {CAPTURE_R} m, max {MAX_TIME_ARENA:.0f} s)')
    arena_res = {}
    for lion in ('radius', 'pure'):
        for man in ('circle', 'flee'):
            r = run_simulation('arena', lion, man)
            arena_res[(lion, man)] = r
            inside = (np.abs(r['p']).max() <= ARENA + 1e-9 and
                      np.abs(r['e']).max() <= ARENA + 1e-9)
            last10 = r['d'][r['t'] >= r['t'][-1] - 10.0]
            if r['captured']:
                st = f'captured @ {r["cap_time"]:.3f} s'
            else:
                st = (f'no capture in {MAX_TIME_ARENA:.0f} s; distance in last 10 s: '
                      f'min {last10.min():.3f} m, max {last10.max():.3f} m')
            print(f'  lion={lion:<6} man={man:<6}: {st} | min distance '
                  f'{r["d"].min():.3f} m | both inside arena: {inside}')

    out_dir = os.path.normpath(OUTPUT_DIR)
    plot_value_functions(sol2, sol3, out_dir)
    plot_convergence(sol2, sol3, sol_eq, out_dir)
    plot_optimal_trajectories(open_runs, out_dir)
    plot_arena(arena_res, out_dir)
    save_animation(open_res, arena_res, out_dir)
