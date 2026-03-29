"""
S006 3D Upgrade — Energy Race
Usage:
    conda activate drones
    python src/01_pursuit_evasion/3d/s006_3d_energy_race.py
"""
import sys, os, numpy as np, matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from matplotlib.animation import FuncAnimation, PillowWriter
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..'))
from src.base.drone_base import DroneBase

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), '..', '..', '..',
                          'outputs', '01_pursuit_evasion', '3d', 's006_3d_energy_race')
os.makedirs(OUTPUT_DIR, exist_ok=True)

# ── Power model ──────────────────────────────────────────────
P_HOVER  = 10.0    # W
K_H      = 0.4     # W·s²/m²  horizontal drag
K_V      = 1.2     # W·s²/m²  climb penalty (only vz > 0)
BATTERY  = 80.0    # J

def power(vel):
    vxy = np.linalg.norm(vel[:2])
    vz  = vel[2]
    climb_term = K_V * vz**2 if vz > 0 else 0.0
    return P_HOVER + K_H * vxy**2 + climb_term

# ── Parameters ───────────────────────────────────────────────
V_PURSUER = 5.0       # m/s
V_EVADER  = 3.0       # m/s
CAPTURE_R = 0.15      # m
DT        = 1 / 48
MAX_TIME  = 30.0

INIT_PURSUER = np.array([-4.0, 0.0, 2.0])
INIT_EVADER  = np.array([ 4.0, 0.0, 2.0])

BETAS_DEG = [0, 15, 30]   # evader climb angles
PURSUER_STRATEGIES = ['level', 'direct_3d', 'energy_opt']
STRAT_LABELS = ['Level (z=2m)', 'Direct 3D', 'Energy-Opt']


# ── Simulation ───────────────────────────────────────────────

def run_simulation(strat, beta_deg):
    beta = np.radians(beta_deg)
    v_evader_dir = np.array([np.cos(beta), 0.0, np.sin(beta)])
    v_evader_cmd = v_evader_dir * V_EVADER

    pursuer = DroneBase(INIT_PURSUER.copy(), max_speed=V_PURSUER, dt=DT)
    evader  = DroneBase(INIT_EVADER.copy(),  max_speed=V_EVADER,  dt=DT)

    p_energy = BATTERY
    e_energy = BATTERY
    max_steps = int(MAX_TIME / DT)

    p_traj_list  = [pursuer.pos.copy()]
    e_traj_list  = [evader.pos.copy()]
    p_energy_log = [p_energy]
    e_energy_log = [e_energy]
    time_log     = [0.0]

    captured = False
    cap_time = None
    p_dead   = False

    for step in range(1, max_steps + 1):
        t = step * DT

        diff = evader.pos - pursuer.pos
        dist = np.linalg.norm(diff)
        if dist < CAPTURE_R:
            captured = True
            cap_time = t
            break

        if p_energy <= 0.0:
            p_dead = True
            break

        # ── Pursuer strategy ──
        if strat == 'level':
            # Stay at z=2, pursue in XY only
            target = evader.pos.copy()
            target[2] = 2.0
            cmd = target - pursuer.pos
            cmd[2] = 2.0 - pursuer.pos[2]  # also correct z
            n = np.linalg.norm(cmd)
            if n > 1e-8:
                cmd = cmd / n * V_PURSUER
            else:
                cmd = np.zeros(3)

        elif strat == 'direct_3d':
            n = dist
            if n > 1e-8:
                cmd = diff / n * V_PURSUER
            else:
                cmd = np.zeros(3)

        else:  # energy_opt
            # If climbing cost > 30% of remaining battery, stay level
            delta_z = evader.pos[2] - pursuer.pos[2]
            if delta_z > 0:
                T_climb = max(0.1, abs(delta_z) / V_PURSUER)
                vz_needed = delta_z / T_climb
                climb_cost = (K_V * vz_needed**2 + P_HOVER) * T_climb
                if climb_cost > 0.3 * p_energy:
                    # Hold z, intercept horizontally
                    target = evader.pos.copy()
                    target[2] = pursuer.pos[2]
                    cmd = target - pursuer.pos
                    n = np.linalg.norm(cmd)
                    cmd = cmd / n * V_PURSUER if n > 1e-8 else np.zeros(3)
                else:
                    n = dist
                    cmd = diff / n * V_PURSUER if n > 1e-8 else np.zeros(3)
            else:
                n = dist
                cmd = diff / n * V_PURSUER if n > 1e-8 else np.zeros(3)

        pursuer.step(cmd)
        p_power = power(pursuer.vel)
        p_energy = max(0.0, p_energy - p_power * DT)

        # Evader flies at beta angle (may climb)
        evader.step(v_evader_cmd)
        e_power = power(evader.vel)
        e_energy = max(0.0, e_energy - e_power * DT)

        p_traj_list.append(pursuer.pos.copy())
        e_traj_list.append(evader.pos.copy())
        p_energy_log.append(p_energy)
        e_energy_log.append(e_energy)
        time_log.append(t)

    return {
        'p_traj':   np.array(p_traj_list),
        'e_traj':   np.array(e_traj_list),
        'p_energy': np.array(p_energy_log),
        'e_energy': np.array(e_energy_log),
        'times':    np.array(time_log),
        'captured': captured,
        'cap_time': cap_time,
        'p_dead':   p_dead,
        'strat':    strat,
        'beta_deg': beta_deg,
    }


# ── Plots ─────────────────────────────────────────────────────

def plot_trajectories_3d(all_res, out_dir):
    """3x3 grid: strategy (rows) × beta (cols)."""
    fig = plt.figure(figsize=(18, 15))
    for si, strat in enumerate(PURSUER_STRATEGIES):
        for bi, beta in enumerate(BETAS_DEG):
            r = all_res[si][bi]
            ax = fig.add_subplot(3, 3, si * 3 + bi + 1, projection='3d')
            ax.view_init(elev=20, azim=-55)
            ax.plot(r['p_traj'][:, 0], r['p_traj'][:, 1], r['p_traj'][:, 2],
                    color='red', linewidth=1.5, label='Pursuer')
            ax.plot(r['e_traj'][:, 0], r['e_traj'][:, 1], r['e_traj'][:, 2],
                    color='royalblue', linewidth=1.5, label='Evader')
            ax.scatter(*r['p_traj'][0], color='red',  s=40, marker='o')
            ax.scatter(*r['e_traj'][0], color='blue', s=40, marker='o')
            if r['captured']:
                ax.scatter(*r['p_traj'][-1], color='black', s=80, marker='X')
                status = f'Cap {r["cap_time"]:.1f}s'
            elif r['p_dead']:
                status = 'P-dead'
            else:
                status = 'Timeout'
            ax.set_title(f'{STRAT_LABELS[si]}\nβ={beta}°  {status}', fontsize=8)
            ax.set_xlabel('X', fontsize=7); ax.set_ylabel('Y', fontsize=7)
            ax.set_zlabel('Z', fontsize=7)
            ax.tick_params(labelsize=6)

    fig.suptitle('S006 3D Energy Race — 3x3 Trajectory Grid\n'
                 '(rows=strategy, cols=evader climb angle β)', fontsize=12)
    plt.tight_layout()
    path = os.path.join(out_dir, 'trajectories_3d.png')
    plt.savefig(path, dpi=130, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_energy_curves(all_res, out_dir):
    """Pursuer energy vs time for 3 strategies at beta=15°."""
    bi = 1  # beta=15
    colors = ['steelblue', 'darkorange', 'mediumseagreen']
    fig, ax = plt.subplots(figsize=(10, 5))
    for si, strat in enumerate(PURSUER_STRATEGIES):
        r = all_res[si][bi]
        ax.plot(r['times'], r['p_energy'], color=colors[si], linewidth=2.0,
                label=f'{STRAT_LABELS[si]} (pursuer)')
        # Evader energy dashed
        ax.plot(r['times'], r['e_energy'], color=colors[si], linewidth=1.2,
                linestyle='--', alpha=0.6, label=f'{STRAT_LABELS[si]} (evader)')
        if r['captured']:
            ax.axvline(r['cap_time'], color=colors[si], linestyle=':', linewidth=1.5,
                       alpha=0.8, label=f'Cap {r["cap_time"]:.1f}s')

    ax.axhline(0, color='black', linestyle=':', linewidth=0.8)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Energy (J)')
    ax.set_title('S006 3D Energy Race — Pursuer/Evader Energy at β=15°\n'
                 '(solid=pursuer, dashed=evader)', fontsize=10)
    ax.legend(fontsize=7, loc='upper right', ncol=2)
    ax.grid(True, alpha=0.3)
    ax.set_ylim(-5, BATTERY + 5)
    plt.tight_layout()
    path = os.path.join(out_dir, 'energy_curves.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def plot_feasibility_grid(out_dir):
    """Heatmap: capture success for direct_3d on (init_dist, delta_z) grid."""
    dists  = np.linspace(2, 8, 14)
    dz_arr = np.linspace(0, 3, 12)

    grid = np.zeros((len(dz_arr), len(dists)), dtype=float)

    for i, dz in enumerate(dz_arr):
        for j, d in enumerate(dists):
            # pursuer at (-d/2, 0, 2), evader at (d/2, 0, 2+dz)
            p0 = np.array([-d / 2, 0.0, 2.0])
            e0 = np.array([ d / 2, 0.0, 2.0 + dz])
            p = DroneBase(p0.copy(), max_speed=V_PURSUER, dt=DT)
            e = DroneBase(e0.copy(), max_speed=V_EVADER,  dt=DT)
            p_en = BATTERY
            beta = np.radians(15)
            v_e = np.array([np.cos(beta), 0.0, np.sin(beta)]) * V_EVADER
            captured_local = False
            for _ in range(int(MAX_TIME / DT)):
                diff = e.pos - p.pos
                dist_cur = np.linalg.norm(diff)
                if dist_cur < CAPTURE_R:
                    captured_local = True
                    break
                if p_en <= 0:
                    break
                cmd = diff / (dist_cur + 1e-8) * V_PURSUER
                p.step(cmd)
                p_en -= power(p.vel) * DT
                p_en = max(0.0, p_en)
                e.step(v_e)
            grid[i, j] = 1.0 if captured_local else 0.0

    fig, ax = plt.subplots(figsize=(9, 6))
    im = ax.imshow(grid, origin='lower', aspect='auto', cmap='RdYlGn',
                   extent=[dists[0], dists[-1], dz_arr[0], dz_arr[-1]],
                   vmin=0, vmax=1)
    plt.colorbar(im, ax=ax, label='Capture success (1=yes, 0=no)')
    ax.set_xlabel('Initial 3D distance (m)')
    ax.set_ylabel('Δz evader above pursuer (m)')
    ax.set_title('S006 3D Energy Race — Feasibility Grid\n'
                 '(direct_3d strategy, evader β=15°)', fontsize=10)
    plt.tight_layout()
    path = os.path.join(out_dir, 'feasibility_grid.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def save_animation(all_res, out_dir):
    """Animate direct_3d vs evader at β=30°."""
    r = all_res[1][2]  # direct_3d, beta=30
    p_traj = r['p_traj']
    e_traj = r['e_traj']
    p_en   = r['p_energy']

    step_skip = 3
    n_frames = len(p_traj) // step_skip

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    all_pts = np.vstack([p_traj, e_traj])
    lo = all_pts.min(axis=0) - 1.5
    hi = all_pts.max(axis=0) + 1.5
    ax.set_xlim(lo[0], hi[0])
    ax.set_ylim(lo[1], hi[1])
    ax.set_zlim(max(0, lo[2]), hi[2])
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
    ax.view_init(elev=25, azim=-50)

    p_line, = ax.plot([], [], [], 'r-', linewidth=1.8, label='Pursuer (direct 3D)')
    e_line, = ax.plot([], [], [], 'b-', linewidth=1.8, label=f'Evader (β=30°)')
    p_dot,  = ax.plot([], [], [], 'r^', markersize=10, zorder=8)
    e_dot,  = ax.plot([], [], [], 'bs', markersize=10, zorder=8)
    ax.legend(fontsize=9, loc='upper left')
    title = ax.set_title('')

    def update(i):
        si = min(i * step_skip, len(p_traj) - 1)
        p_line.set_data(p_traj[:si+1, 0], p_traj[:si+1, 1])
        p_line.set_3d_properties(p_traj[:si+1, 2])
        e_line.set_data(e_traj[:si+1, 0], e_traj[:si+1, 1])
        e_line.set_3d_properties(e_traj[:si+1, 2])
        p_dot.set_data([p_traj[si, 0]], [p_traj[si, 1]])
        p_dot.set_3d_properties([p_traj[si, 2]])
        e_dot.set_data([e_traj[si, 0]], [e_traj[si, 1]])
        e_dot.set_3d_properties([e_traj[si, 2]])
        t  = si * DT
        en = p_en[si] if si < len(p_en) else 0.0
        title.set_text(f'S006 3D Direct Pursuit  t={t:.2f}s  P-energy={en:.1f}J')
        return p_line, e_line, p_dot, e_dot, title

    ani = FuncAnimation(fig, update, frames=n_frames, interval=70, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer=PillowWriter(fps=14), dpi=90)
    plt.close()
    print(f'Saved: {path}')


# ── Main ─────────────────────────────────────────────────────

if __name__ == '__main__':
    # Build 3x3 result matrix
    all_res = []
    for strat in PURSUER_STRATEGIES:
        row = []
        for beta in BETAS_DEG:
            r = run_simulation(strat, beta)
            status = f'cap {r["cap_time"]:.2f}s' if r['captured'] else \
                     ('P-dead' if r['p_dead'] else 'timeout')
            print(f'[{strat:<12} β={beta:2d}°]  {status}')
            row.append(r)
        all_res.append(row)

    out_dir = os.path.normpath(OUTPUT_DIR)
    plot_trajectories_3d(all_res, out_dir)
    plot_energy_curves(all_res, out_dir)
    plot_feasibility_grid(out_dir)
    save_animation(all_res, out_dir)
    print('S006 3D done.')
