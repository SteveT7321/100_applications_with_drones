"""
S009 Differential Game 1v1 — Lion & Man (HJI Numerical Solution)
=================================================================
Equal-speed pursuer and evader in a 2-D bounded arena.
Solves the Hamilton-Jacobi-Isaacs (HJI) equation via value iteration to
find the value function V(s) where s = p_E - p_P (relative state).

When v_P == v_E the Hamiltonian H = (v_E - v_P)|∇V| - 1 = -1 everywhere,
so V diverges — confirming capture is impossible.  We also compare with an
asymmetric case (v_P = 5, v_E = 3) where capture is guaranteed.

Outputs:
  - 2D value function contour (equal + asymmetric speeds)
  - Convergence curves
  - Optimal trajectories extracted from ∇V
  - Capture time contour (asymmetric case)

Usage:
    conda activate drones
    python src/pursuit/s009_differential_game.py
"""

import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from src.base.drone_base import DroneBase

# ── Parameters ───────────────────────────────────────────────
ARENA      = 5.0     # ±ARENA m square
N_GRID     = 60      # grid resolution per axis
MAX_ITER   = 500
DT_VI      = 0.02    # value-iteration step
CONV_TOL   = 1e-4
CAPTURE_R  = 0.15    # m
DT_SIM     = 1 / 48
MAX_TIME   = 20.0

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '01_pursuit_evasion', 's009_differential_game',
)


# ── HJI Value Iteration ──────────────────────────────────────

def solve_hji(v_p, v_e, n=N_GRID, arena=ARENA, max_iter=MAX_ITER):
    """
    Solve HJI in relative-state space s = p_E - p_P (2-D, z fixed).

    H(s, ∇V) = (v_E - v_P)|∇V| - 1
    For v_P > v_E:  H < 0 when |∇V| < 1/(v_P-v_E) → V converges.
    For v_P == v_E: H = -1 everywhere → V grows without bound (no capture).

    Terminal: V=0 inside capture_radius ball.
    """
    x = np.linspace(-2 * arena, 2 * arena, n)
    y = np.linspace(-2 * arena, 2 * arena, n)
    X, Y = np.meshgrid(x, y, indexing='ij')
    R = np.sqrt(X ** 2 + Y ** 2)

    # Initial value: straight-line distance / (v_P - v_E)
    dv = max(v_p - v_e, 1e-6)
    V  = R / dv

    conv_hist = []
    dx = x[1] - x[0]

    for it in range(max_iter):
        dVdx = np.gradient(V, dx, axis=0)
        dVdy = np.gradient(V, dx, axis=1)
        grad_norm = np.sqrt(dVdx ** 2 + dVdy ** 2) + 1e-8

        H = (v_e - v_p) * grad_norm - 1.0
        V_new = V - DT_VI * H
        V_new = np.maximum(V_new, 0.0)
        V_new[R < CAPTURE_R] = 0.0

        delta = np.max(np.abs(V_new - V))
        conv_hist.append(delta)
        V = V_new

        if delta < CONV_TOL:
            print(f'  [v_P={v_p}, v_E={v_e}] Converged at iter {it+1}')
            break
    else:
        print(f'  [v_P={v_p}, v_E={v_e}] Max iter reached (delta={conv_hist[-1]:.2e})')

    return x, y, V, conv_hist


# ── Trajectory simulation using ∇V ──────────────────────────

def simulate_from_value(x_grid, V, v_p, v_e,
                        init_pursuer=None, init_evader=None):
    """
    Extract optimal pursuer/evader strategies from ∇V(s).
    Pursuer moves along +∇V (in world coords, this reduces the relative state);
    evader moves along +∇V to maximise V.
    For v_P > v_E: net ds/dt = (v_E-v_P)*∇V/|∇V| < 0 → capture guaranteed.
    For v_P = v_E: net ds/dt = 0 → no capture.

    For the asymmetric case we fall back to pure pursuit for the pursuer
    (which is the theoretical optimal when v_P > v_E) to avoid noise from an
    unconverged value function.
    """
    from scipy.interpolate import RegularGridInterpolator

    # ∇V on grid
    dx = x_grid[1] - x_grid[0]
    dVdx = np.gradient(V, dx, axis=0)
    dVdy = np.gradient(V, dx, axis=1)

    interp_x = RegularGridInterpolator(
        (x_grid, x_grid), dVdx, method='linear',
        bounds_error=False, fill_value=0.0)
    interp_y = RegularGridInterpolator(
        (x_grid, x_grid), dVdy, method='linear',
        bounds_error=False, fill_value=0.0)

    if init_pursuer is None:
        init_pursuer = np.array([-3.0,  0.0, 2.0])
    if init_evader is None:
        init_evader  = np.array([ 3.0,  0.0, 2.0])

    pursuer = DroneBase(init_pursuer.copy(), max_speed=v_p, dt=DT_SIM)
    evader  = DroneBase(init_evader.copy(),  max_speed=v_e, dt=DT_SIM)

    # Use longer time limit; asymmetric case converges in finite time
    sim_max_time = 40.0 if v_p > v_e else MAX_TIME
    p_traj = [pursuer.pos.copy()]
    e_traj = [evader.pos.copy()]
    max_steps = int(sim_max_time / DT_SIM)
    captured = False; cap_time = None

    use_pure_pursuit = (v_p > v_e)   # more reliable when HJI not fully converged

    for step in range(1, max_steps + 1):
        t = step * DT_SIM
        s = evader.pos[:2] - pursuer.pos[:2]
        s_clamped = np.clip(s, x_grid[0] + 1e-6, x_grid[-1] - 1e-6)

        gx = float(interp_x([[s_clamped[0], s_clamped[1]]]).flat[0])
        gy = float(interp_y([[s_clamped[0], s_clamped[1]]]).flat[0])
        grad = np.array([gx, gy, 0.0])
        gnorm = np.linalg.norm(grad)

        if gnorm < 1e-8:
            p_dir = np.zeros(3)
            e_dir = np.zeros(3)
        else:
            if use_pure_pursuit:
                # Pure pursuit (optimal when v_P > v_E): head straight at evader
                diff = evader.pos - pursuer.pos
                dn   = np.linalg.norm(diff)
                p_dir = diff / dn if dn > 1e-8 else np.zeros(3)
            else:
                p_dir = grad / gnorm   # ∇V strategy (equal speeds)
            e_dir = grad / gnorm       # evader always maximises V

        pursuer.step(p_dir * v_p)
        evader.step(e_dir * v_e)

        p_traj.append(pursuer.pos.copy())
        e_traj.append(evader.pos.copy())

        dist = np.linalg.norm(pursuer.pos - evader.pos)
        if dist < CAPTURE_R:
            captured = True; cap_time = t
            break

    return np.array(p_traj), np.array(e_traj), captured, cap_time


# ── Plots ────────────────────────────────────────────────────

def plot_value_functions(cases, out_dir):
    """Contour plots of the value functions for equal and asymmetric speeds."""
    fig, axes = plt.subplots(1, len(cases), figsize=(6 * len(cases), 5))
    if len(cases) == 1:
        axes = [axes]

    for ax, (label, xg, V, _) in zip(axes, cases):
        # Clip very large V for display
        V_disp = np.minimum(V, np.percentile(V, 95))
        ct = ax.contourf(xg, xg, V_disp.T, levels=30, cmap='viridis')
        plt.colorbar(ct, ax=ax, label='V (capture time estimate, s)')
        ax.contour(xg, xg, V_disp.T, levels=10, colors='white',
                   linewidths=0.5, alpha=0.5)
        # Capture radius circle
        theta = np.linspace(0, 2 * np.pi, 100)
        ax.plot(CAPTURE_R * np.cos(theta), CAPTURE_R * np.sin(theta),
                'w--', linewidth=1.5, label=f'Capture r={CAPTURE_R}m')
        ax.set_xlabel('Δx = x_E - x_P (m)')
        ax.set_ylabel('Δy = y_E - y_P (m)')
        ax.set_title(f'{label}\nValue Function V(s)', fontsize=10)
        ax.set_aspect('equal')
        ax.legend(fontsize=8, loc='upper right')

    fig.suptitle('S009 Differential Game — HJI Value Function', fontsize=11)
    plt.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'value_functions.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_convergence(cases, out_dir):
    fig, ax = plt.subplots(figsize=(10, 5))
    colors = ['steelblue', 'mediumseagreen', 'darkorange']
    for (label, _, _, conv_hist), color in zip(cases, colors):
        ax.semilogy(conv_hist, color=color, linewidth=1.8, label=label)
    ax.axhline(CONV_TOL, color='red', linestyle='--', linewidth=1.0,
               label=f'Convergence tol {CONV_TOL}')
    ax.set_xlabel('Iteration')
    ax.set_ylabel('Max |ΔV| (log scale)')
    ax.set_title('S009 Differential Game — HJI Value Iteration Convergence')
    ax.legend(fontsize=9)
    ax.grid(True, which='both', alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'convergence.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def plot_optimal_trajectories(traj_cases, out_dir):
    """Show trajectories extracted from value function for each speed pair."""
    fig, axes = plt.subplots(1, len(traj_cases), figsize=(6 * len(traj_cases), 5))
    if len(traj_cases) == 1:
        axes = [axes]

    for ax, (label, p_traj, e_traj, captured, cap_t) in zip(axes, traj_cases):
        ax.plot(p_traj[:, 0], p_traj[:, 1], color='red',
                linewidth=2.0, label='Pursuer')
        ax.plot(e_traj[:, 0], e_traj[:, 1], color='royalblue',
                linewidth=2.0, label='Evader')
        ax.scatter(*p_traj[0, :2], color='red',  s=80, zorder=5)
        ax.scatter(*e_traj[0, :2], color='blue', s=80, zorder=5)
        if captured:
            ax.scatter(*p_traj[-1, :2], color='black', s=120, marker='X', zorder=6,
                       label=f'Captured {cap_t:.2f}s')
            status = f'Captured {cap_t:.2f} s'
        else:
            dist = np.linalg.norm(p_traj[-1, :2] - e_traj[-1, :2])
            status = f'Equal speed — gap={dist:.2f}m  (no capture)'

        # Draw arena boundary
        arena_box = plt.Rectangle((-ARENA, -ARENA), 2 * ARENA, 2 * ARENA,
                                  fill=False, edgecolor='grey',
                                  linestyle='--', linewidth=1.0)
        ax.add_patch(arena_box)

        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
        ax.set_title(f'{label}\n{status}', fontsize=9)
        ax.legend(fontsize=8, loc='upper left')

    fig.suptitle('S009 Differential Game — Optimal Trajectories from ∇V', fontsize=11)
    plt.tight_layout()
    path = os.path.join(out_dir, 'optimal_trajectories.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def save_animation(traj_cases, out_dir):
    """Side-by-side animation: equal speeds vs asymmetric speeds."""
    import matplotlib.animation as animation

    max_len = max(len(tc[1]) for tc in traj_cases)
    step = 3
    fig, axes = plt.subplots(1, len(traj_cases), figsize=(12, 5))
    if len(traj_cases) == 1:
        axes = [axes]

    lines_p, lines_e, dots_p, dots_e, titles = [], [], [], [], []
    for ax, (label, p_traj, e_traj, captured, cap_t) in zip(axes, traj_cases):
        all_pts = np.vstack([p_traj, e_traj])
        lo = all_pts.min(axis=0)[:2] - 1.0
        hi = all_pts.max(axis=0)[:2] + 1.0
        ax.set_xlim(lo[0], hi[0]); ax.set_ylim(lo[1], hi[1])
        ax.set_aspect('equal'); ax.grid(True, alpha=0.3)
        ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')

        rect = plt.Rectangle((-ARENA, -ARENA), 2*ARENA, 2*ARENA,
                              fill=False, edgecolor='grey',
                              linestyle='--', linewidth=1.0)
        ax.add_patch(rect)
        ax.scatter(*p_traj[0, :2], color='red',  s=60, zorder=5)
        ax.scatter(*e_traj[0, :2], color='blue', s=60, zorder=5)

        lp, = ax.plot([], [], color='red',       linewidth=1.8, alpha=0.8)
        le, = ax.plot([], [], color='royalblue', linewidth=1.8, alpha=0.8)
        dp, = ax.plot([], [], 'r^', markersize=9, zorder=6)
        de, = ax.plot([], [], 'bs', markersize=9, zorder=6)
        ti  = ax.set_title(label, fontsize=9)
        lines_p.append(lp); lines_e.append(le)
        dots_p.append(dp); dots_e.append(de); titles.append(ti)

    n_frames = max_len // step

    def update(i):
        artists = []
        for idx, (label, p_traj, e_traj, captured, cap_t) in enumerate(traj_cases):
            si = min(i * step, len(p_traj) - 1)
            done = (i * step >= len(p_traj) - 1)
            t = si * DT_SIM
            dist = np.linalg.norm(p_traj[si, :2] - e_traj[si, :2])
            lines_p[idx].set_data(p_traj[:si+1, 0], p_traj[:si+1, 1])
            lines_e[idx].set_data(e_traj[:si+1, 0], e_traj[:si+1, 1])
            dots_p[idx].set_data([float(p_traj[si, 0])], [float(p_traj[si, 1])])
            dots_e[idx].set_data([float(e_traj[si, 0])], [float(e_traj[si, 1])])
            if done:
                st = f'✓ Captured {cap_t:.2f}s [DONE]' if captured else f'gap={dist:.2f}m [DONE]'
                titles[idx].set_text(f'{label}  t={t:.1f}s\n{st}')
            else:
                titles[idx].set_text(f'{label}  t={t:.1f}s  dist={dist:.2f}m')
            artists += [lines_p[idx], lines_e[idx], dots_p[idx], dots_e[idx], titles[idx]]
        return artists

    ani = animation.FuncAnimation(fig, update, frames=n_frames,
                                  interval=60, blit=True)
    fig.suptitle('S009 Differential Game — Optimal Trajectories\n'
                 '(red=pursuer, blue=evader, dashed box=arena)', fontsize=10)
    plt.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer='pillow', fps=16, dpi=100)
    plt.close()
    print(f'Saved: {path}')


# ── Main ─────────────────────────────────────────────────────

if __name__ == '__main__':
    speed_pairs = [
        ('Equal speeds (v_P=v_E=4)',      4.0, 4.0),
        ('Asymmetric (v_P=5, v_E=3)',     5.0, 3.0),
    ]

    hji_cases  = []
    traj_cases = []

    for label, v_p, v_e in speed_pairs:
        print(f'Solving HJI: {label}')
        xg, yg, V, conv = solve_hji(v_p, v_e)
        hji_cases.append((label, xg, V, conv))

        print(f'  Simulating optimal trajectory...')
        p_traj, e_traj, captured, cap_t = simulate_from_value(xg, V, v_p, v_e)
        traj_cases.append((label, p_traj, e_traj, captured, cap_t))
        status = f'captured @ {cap_t:.2f}s' if captured else (
            f'no capture, final gap = '
            f'{np.linalg.norm(p_traj[-1,:2]-e_traj[-1,:2]):.2f}m')
        print(f'  Trajectory: {status}')

    out_dir = os.path.normpath(OUTPUT_DIR)
    plot_value_functions(hji_cases, out_dir)
    plot_convergence(hji_cases, out_dir)
    plot_optimal_trajectories(traj_cases, out_dir)
    save_animation(traj_cases, out_dir)
