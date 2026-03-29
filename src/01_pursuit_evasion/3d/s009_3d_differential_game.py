"""
S009 3D Upgrade — Differential Game (HJI in 3D)
================================================
Solves the Hamilton-Jacobi-Isaacs equation on a 30×30×30 grid of
relative state s = p_E - p_P ∈ [-10,10]³.

Equal speeds (v_P = v_E = 4): capture time diverges, H = -1 everywhere.
Gradient-policy trajectory simulation shows how agents move under
optimal play in bounded 3D arena.

Usage:
    conda activate drones
    python src/01_pursuit_evasion/3d/s009_3d_differential_game.py
"""

import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from matplotlib.animation import FuncAnimation, PillowWriter
from scipy.interpolate import RegularGridInterpolator

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..'))
from src.base.drone_base import DroneBase

# ── Parameters ───────────────────────────────────────────────
N          = 25       # grid resolution per axis (25³ = 15 625 cells, fast)
S_MAX      = 10.0
V_P        = 4.0
V_E        = 4.0
DT_VI      = 0.005
MAX_ITER   = 300
CONV_TOL   = 1e-4
CAPTURE_R  = 0.15
DT_SIM     = 1 / 48
MAX_SIM    = 20.0
ARENA      = 5.0      # bounded arena ±5 m

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..', '..',
    'outputs', '01_pursuit_evasion', '3d', 's009_3d_differential_game',
)
os.makedirs(OUTPUT_DIR, exist_ok=True)


# ── 3D HJI Solver ────────────────────────────────────────────

def solve_hji_3d(n=N, s_max=S_MAX, v_p=V_P, v_e=V_E,
                 max_iter=MAX_ITER, dt_vi=DT_VI):
    sx = np.linspace(-s_max, s_max, n)
    sy = np.linspace(-s_max, s_max, n)
    sz = np.linspace(-s_max, s_max, n)
    SX, SY, SZ = np.meshgrid(sx, sy, sz, indexing='ij')
    S_norm = np.sqrt(SX ** 2 + SY ** 2 + SZ ** 2)

    # Initial guess: straight-line distance
    V = S_norm.copy()
    V[S_norm < CAPTURE_R] = 0.0

    conv_hist = []

    for iteration in range(max_iter):
        dVdx = np.gradient(V, sx, axis=0)
        dVdy = np.gradient(V, sy, axis=1)
        dVdz = np.gradient(V, sz, axis=2)
        grad_norm = np.sqrt(dVdx ** 2 + dVdy ** 2 + dVdz ** 2) + 1e-8

        # Equal speeds: H = (v_E - v_P)*|∇V| - 1 = -1
        H = (v_e - v_p) * grad_norm - 1.0
        V_new = V - dt_vi * H
        V_new[S_norm < CAPTURE_R] = 0.0
        V_new = np.maximum(V_new, 0.0)

        delta = np.max(np.abs(V_new - V))
        conv_hist.append(delta)
        V = V_new

        if delta < CONV_TOL:
            print(f'  3D HJI converged at iteration {iteration + 1}  (delta={delta:.2e})')
            break
    else:
        print(f'  3D HJI max iter reached  (delta={conv_hist[-1]:.2e})')

    return sx, sy, sz, V, np.array(conv_hist)


def solve_hji_2d(n=N, s_max=S_MAX, v_p=V_P, v_e=V_E,
                 max_iter=MAX_ITER, dt_vi=DT_VI):
    """Recompute 2D HJI on N×N grid for comparison."""
    sx = np.linspace(-s_max, s_max, n)
    sy = np.linspace(-s_max, s_max, n)
    SX, SY = np.meshgrid(sx, sy, indexing='ij')
    S_norm = np.sqrt(SX ** 2 + SY ** 2)
    V = S_norm.copy()
    V[S_norm < CAPTURE_R] = 0.0

    for _ in range(max_iter):
        dVdx = np.gradient(V, sx, axis=0)
        dVdy = np.gradient(V, sy, axis=1)
        gn = np.sqrt(dVdx ** 2 + dVdy ** 2) + 1e-8
        H = (v_e - v_p) * gn - 1.0
        V_new = V - dt_vi * H
        V_new[S_norm < CAPTURE_R] = 0.0
        V_new = np.maximum(V_new, 0.0)
        delta = np.max(np.abs(V_new - V))
        V = V_new
        if delta < CONV_TOL:
            break

    return sx, sy, V


# ── Trajectory simulation using ∇V ──────────────────────────

def simulate_from_3d_value(sx, sy, sz, V,
                            init_pursuer=None, init_evader=None,
                            v_p=V_P, v_e=V_E, dt=DT_SIM, max_time=MAX_SIM):
    """Simulate pursuer/evader using gradient of 3D value function."""
    dVdx = np.gradient(V, sx, axis=0)
    dVdy = np.gradient(V, sy, axis=1)
    dVdz = np.gradient(V, sz, axis=2)

    interp_x = RegularGridInterpolator((sx, sy, sz), dVdx,
                                        method='linear', bounds_error=False, fill_value=0.0)
    interp_y = RegularGridInterpolator((sx, sy, sz), dVdy,
                                        method='linear', bounds_error=False, fill_value=0.0)
    interp_z = RegularGridInterpolator((sx, sy, sz), dVdz,
                                        method='linear', bounds_error=False, fill_value=0.0)

    if init_pursuer is None:
        init_pursuer = np.array([-3.0, 0.0, 0.0])
    if init_evader is None:
        init_evader  = np.array([ 3.0, 0.0, 0.0])

    pursuer = DroneBase(init_pursuer.copy(), max_speed=v_p, dt=dt)
    evader  = DroneBase(init_evader.copy(),  max_speed=v_e, dt=dt)

    p_traj = [pursuer.pos.copy()]
    e_traj = [evader.pos.copy()]
    captured = False
    cap_time = None
    max_steps = int(max_time / dt)

    for step in range(1, max_steps + 1):
        t = step * dt
        s = evader.pos - pursuer.pos
        s_clamped = np.clip(s, -S_MAX + 0.1, S_MAX - 0.1)
        pt = s_clamped.reshape(1, 3)

        gx = float(interp_x(pt).flat[0])
        gy = float(interp_y(pt).flat[0])
        gz = float(interp_z(pt).flat[0])
        grad = np.array([gx, gy, gz])
        gnorm = np.linalg.norm(grad)

        if gnorm < 1e-8:
            p_dir = np.zeros(3)
            e_dir = np.zeros(3)
        else:
            grad_n = grad / gnorm
            # Pursuer: move toward +∇V (reduce relative state toward 0)
            p_dir = grad_n
            # Evader: move along +∇V (maximize capture time)
            e_dir = grad_n

        # Clip to arena
        new_p = np.clip(pursuer.pos + p_dir * v_p * dt, -ARENA, ARENA)
        new_e = np.clip(evader.pos  + e_dir * v_e * dt, -ARENA, ARENA)
        pursuer.pos = new_p
        evader.pos  = new_e
        pursuer.trajectory.append(pursuer.pos.copy())
        evader.trajectory.append(evader.pos.copy())

        p_traj.append(pursuer.pos.copy())
        e_traj.append(evader.pos.copy())

        if np.linalg.norm(pursuer.pos - evader.pos) < CAPTURE_R:
            captured = True
            cap_time = t
            break

    return np.array(p_traj), np.array(e_traj), captured, cap_time


# ── Plots ────────────────────────────────────────────────────

def plot_value_slices(sx, sy, sz, V, out_dir):
    """3 subplots: z=0 slice, y=0 slice, x=0 slice of V (contourf)."""
    mid_x = len(sx) // 2
    mid_y = len(sy) // 2
    mid_z = len(sz) // 2
    V_clip = np.minimum(V, np.percentile(V, 90))

    fig, axes = plt.subplots(1, 3, figsize=(15, 5))

    # z=0 slice (x-y plane)
    ct = axes[0].contourf(sx, sy, V_clip[:, :, mid_z].T, levels=25, cmap='viridis')
    plt.colorbar(ct, ax=axes[0])
    axes[0].set_xlabel('sx (m)'); axes[0].set_ylabel('sy (m)')
    axes[0].set_title('V(sx, sy, sz=0)\nz=0 slice', fontsize=10)
    axes[0].set_aspect('equal')

    # y=0 slice (x-z plane)
    ct2 = axes[1].contourf(sx, sz, V_clip[:, mid_y, :].T, levels=25, cmap='viridis')
    plt.colorbar(ct2, ax=axes[1])
    axes[1].set_xlabel('sx (m)'); axes[1].set_ylabel('sz (m)')
    axes[1].set_title('V(sx, sy=0, sz)\ny=0 slice', fontsize=10)
    axes[1].set_aspect('equal')

    # x=0 slice (y-z plane)
    ct3 = axes[2].contourf(sy, sz, V_clip[mid_x, :, :].T, levels=25, cmap='viridis')
    plt.colorbar(ct3, ax=axes[2])
    axes[2].set_xlabel('sy (m)'); axes[2].set_ylabel('sz (m)')
    axes[2].set_title('V(sx=0, sy, sz)\nx=0 slice', fontsize=10)
    axes[2].set_aspect('equal')

    fig.suptitle('S009 3D Differential Game — Value Function Slices', fontsize=11)
    plt.tight_layout()
    path = os.path.join(out_dir, 'value_function_slices.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_isosurface_concept(sx, sy, sz, V, out_dir):
    """Show z=0 slice with level lines at V=5,10,20,30."""
    mid_z = len(sz) // 2
    V_z0 = V[:, :, mid_z]

    fig, ax = plt.subplots(figsize=(7, 6))
    ct = ax.contourf(sx, sy, V_z0.T, levels=30, cmap='plasma')
    plt.colorbar(ct, ax=ax, label='V (capture time estimate, s)')
    cs = ax.contour(sx, sy, V_z0.T, levels=[5, 10, 20, 30],
                    colors='white', linewidths=1.5)
    ax.clabel(cs, inline=True, fontsize=9, fmt='V=%.0f')
    theta = np.linspace(0, 2 * np.pi, 100)
    ax.plot(CAPTURE_R * np.cos(theta), CAPTURE_R * np.sin(theta),
            'w--', linewidth=1.5, label=f'Capture r={CAPTURE_R}m')
    ax.set_xlabel('sx (m)'); ax.set_ylabel('sy (m)')
    ax.set_title('S009 3D — Isosurface Concept: z=0 Slice\nLevel lines at V=5,10,20,30',
                 fontsize=10)
    ax.set_aspect('equal')
    ax.legend(fontsize=8)
    plt.tight_layout()
    path = os.path.join(out_dir, 'isosurface_concept.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_trajectory_sim(traj_cases, out_dir):
    """3 trajectories on 3D axes."""
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    colors_p = ['red', 'darkred', 'firebrick']
    colors_e = ['royalblue', 'steelblue', 'navy']

    for i, (label, p_traj, e_traj, captured, cap_t) in enumerate(traj_cases):
        ax.plot(p_traj[:, 0], p_traj[:, 1], p_traj[:, 2],
                color=colors_p[i], linewidth=1.5, label=f'Pursuer {i+1}', alpha=0.8)
        ax.plot(e_traj[:, 0], e_traj[:, 1], e_traj[:, 2],
                color=colors_e[i], linewidth=1.5, linestyle='--',
                label=f'Evader {i+1}', alpha=0.8)
        ax.scatter(*p_traj[0], color=colors_p[i], s=50, zorder=5)
        ax.scatter(*e_traj[0], color=colors_e[i], s=50, zorder=5)
        if captured:
            ax.scatter(*p_traj[-1], color='black', s=80, marker='X', zorder=6)

    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
    ax.set_title('S009 3D — Gradient Policy Trajectories\n(3 starting configs)', fontsize=10)
    ax.legend(fontsize=7, loc='upper left')
    plt.tight_layout()
    path = os.path.join(out_dir, 'trajectory_sim.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_convergence(conv_hist, out_dir):
    """Max|V_new - V| vs iteration."""
    fig, ax = plt.subplots(figsize=(10, 5))
    ax.semilogy(conv_hist, color='firebrick', linewidth=2.0, label='3D HJI')
    ax.axhline(CONV_TOL, color='red', linestyle='--', linewidth=1.0,
               label=f'Tol {CONV_TOL}')
    ax.set_xlabel('Iteration')
    ax.set_ylabel('Max |ΔV| (log scale)')
    ax.set_title('S009 3D — HJI Value Iteration Convergence (Equal Speeds)', fontsize=10)
    ax.legend(fontsize=9)
    ax.grid(True, which='both', alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'convergence.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def plot_z0_vs_2d(sx, sy, sz, V3d, out_dir):
    """Compare z=0 slice of 3D result vs 2D result on same grid."""
    mid_z = len(sz) // 2
    V3d_z0 = V3d[:, :, mid_z]

    print('  Solving 2D HJI for comparison...')
    sx2d, sy2d, V2d = solve_hji_2d(n=N)

    V_clip_3d = np.minimum(V3d_z0, np.percentile(V3d_z0, 90))
    V_clip_2d = np.minimum(V2d,     np.percentile(V2d,     90))

    fig, axes = plt.subplots(1, 3, figsize=(16, 5))

    ct1 = axes[0].contourf(sx, sy, V_clip_3d.T, levels=25, cmap='viridis')
    plt.colorbar(ct1, ax=axes[0])
    axes[0].set_title('3D V — z=0 slice', fontsize=10)
    axes[0].set_xlabel('sx'); axes[0].set_ylabel('sy')
    axes[0].set_aspect('equal')

    ct2 = axes[1].contourf(sx2d, sy2d, V_clip_2d.T, levels=25, cmap='viridis')
    plt.colorbar(ct2, ax=axes[1])
    axes[1].set_title('2D V (recomputed on same N×N grid)', fontsize=10)
    axes[1].set_xlabel('sx'); axes[1].set_ylabel('sy')
    axes[1].set_aspect('equal')

    # Difference (interpolate to same grid)
    diff = V_clip_3d - V_clip_2d
    vmax = np.abs(diff).max()
    ct3 = axes[2].contourf(sx, sy, diff.T, levels=25, cmap='RdBu_r',
                            vmin=-vmax, vmax=vmax)
    plt.colorbar(ct3, ax=axes[2])
    axes[2].set_title('Difference: 3D[z=0] - 2D', fontsize=10)
    axes[2].set_xlabel('sx'); axes[2].set_ylabel('sy')
    axes[2].set_aspect('equal')

    fig.suptitle('S009 3D vs 2D Value Function Comparison', fontsize=11)
    plt.tight_layout()
    path = os.path.join(out_dir, 'z0_vs_2d.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def save_animation(p_traj, e_traj, out_dir):
    """Animate one trajectory in 3D."""
    n = len(p_traj)
    step = 3

    fig = plt.figure(figsize=(8, 7))
    ax = fig.add_subplot(111, projection='3d')

    all_pts = np.vstack([p_traj, e_traj])
    margin = 0.5
    ax.set_xlim(all_pts[:, 0].min() - margin, all_pts[:, 0].max() + margin)
    ax.set_ylim(all_pts[:, 1].min() - margin, all_pts[:, 1].max() + margin)
    ax.set_zlim(all_pts[:, 2].min() - margin, all_pts[:, 2].max() + margin)
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')

    line_p, = ax.plot([], [], [], color='red', linewidth=1.8, label='Pursuer')
    line_e, = ax.plot([], [], [], color='royalblue', linewidth=1.8,
                      linestyle='--', label='Evader')
    dot_p,  = ax.plot([], [], [], 'r^', markersize=9, zorder=6)
    dot_e,  = ax.plot([], [], [], 'bs', markersize=9, zorder=6)
    ax.legend(fontsize=8)
    title = ax.set_title('S009 3D — Gradient Policy Trajectory', fontsize=10)

    def update(i):
        si = min(i * step, n - 1)
        line_p.set_data(p_traj[:si+1, 0], p_traj[:si+1, 1])
        line_p.set_3d_properties(p_traj[:si+1, 2])
        line_e.set_data(e_traj[:si+1, 0], e_traj[:si+1, 1])
        line_e.set_3d_properties(e_traj[:si+1, 2])
        dot_p.set_data([p_traj[si, 0]], [p_traj[si, 1]])
        dot_p.set_3d_properties([p_traj[si, 2]])
        dot_e.set_data([e_traj[si, 0]], [e_traj[si, 1]])
        dot_e.set_3d_properties([e_traj[si, 2]])
        t_now = si * DT_SIM
        title.set_text(f'S009 3D — Gradient Policy  t={t_now:.2f}s')
        return [line_p, line_e, dot_p, dot_e, title]

    n_frames = n // step
    ani = FuncAnimation(fig, update, frames=n_frames, interval=60, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer=PillowWriter(fps=16), dpi=100)
    plt.close()
    print(f'Saved: {path}')


# ── Main ─────────────────────────────────────────────────────

if __name__ == '__main__':
    out_dir = os.path.normpath(OUTPUT_DIR)

    print('Solving 3D HJI (equal speeds v_P=v_E=4)...')
    sx, sy, sz, V, conv_hist = solve_hji_3d()

    # 3 trajectory simulations from different starting points
    start_configs = [
        (np.array([-3.0,  0.0,  0.0]), np.array([ 3.0,  0.0,  0.0])),
        (np.array([ 0.0, -3.0,  1.0]), np.array([ 0.0,  3.0, -1.0])),
        (np.array([-2.0,  2.0, -1.0]), np.array([ 2.0, -2.0,  1.0])),
    ]

    traj_cases = []
    for i, (ip, ie) in enumerate(start_configs):
        print(f'  Simulating trajectory {i+1}...')
        p_traj, e_traj, captured, cap_t = simulate_from_3d_value(
            sx, sy, sz, V, init_pursuer=ip, init_evader=ie)
        status = f'captured @ {cap_t:.2f}s' if captured else (
            f'gap={np.linalg.norm(p_traj[-1]-e_traj[-1]):.2f}m')
        print(f'    Config {i+1}: {status}')
        traj_cases.append((f'Config {i+1}', p_traj, e_traj, captured, cap_t))

    print('Generating plots...')
    plot_value_slices(sx, sy, sz, V, out_dir)
    plot_isosurface_concept(sx, sy, sz, V, out_dir)
    plot_trajectory_sim(traj_cases, out_dir)
    plot_convergence(conv_hist, out_dir)
    plot_z0_vs_2d(sx, sy, sz, V, out_dir)
    save_animation(traj_cases[0][1], traj_cases[0][2], out_dir)
    print('Done.')
