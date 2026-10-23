"""
S006 3D Upgrade — Energy Race
==============================
Pursuer and evader carry 80 J batteries. Power depends on horizontal speed and
on the climb rate only (descent is free):

    P(v) = P_hover + k_h * v_xy^2 + k_v * max(0, v_z)^2

The evader starts 6 m ahead (and dz = 0 / 1 / 2 m higher) and flies straight
at 3 m/s with climb angle beta = 0 / 15 / 30 deg. A drone whose battery is
empty stops (the pursuer run ends; the evader holds position).

Pursuer strategies:
  level      — pure pursuit in the horizontal plane, altitude held at 2 m
               (the 2D baseline), at 3 / 4 / 5 m/s
  direct_3d  — pure pursuit in 3D, at 3 / 4 / 5 m/s
  energy_opt — re-planned every step: the constant-velocity intercept of the
               evader's current straight-line motion that minimises the energy
               P(v) * T, with |v| <= 5 m/s. Constant velocity is optimal for a
               given T because the power is convex in v.

Usage:
    conda activate drones
    python src/01_pursuit_evasion/3d/s006_3d_energy_race.py
"""
import sys
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from matplotlib.animation import FuncAnimation, PillowWriter

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..'))

OUTPUT_DIR = os.path.normpath(os.path.join(
    os.path.dirname(__file__), '..', '..', '..',
    'outputs', '01_pursuit_evasion', '3d', 's006_3d_energy_race'))

# ── Power model ──────────────────────────────────────────────
P_HOVER = 10.0    # W
K_H     = 0.4     # W s^2/m^2, horizontal
K_V     = 1.2     # W s^2/m^2, climb only (v_z > 0)
BATTERY = 80.0    # J


def power(vel):
    """Power for velocity vel: only the vertical component counts as climb."""
    vz = vel[2]
    return P_HOVER + K_H * (vel[0]**2 + vel[1]**2) + (K_V * vz**2 if vz > 0 else 0.0)


# ── Parameters ───────────────────────────────────────────────
V_OPTIONS  = [3.0, 4.0, 5.0]   # pursuer speed options (card)
V_MAX      = 5.0               # energy_opt speed limit
V_EVADER   = 3.0
CAPTURE_R  = 0.15
DT         = 1 / 48
MAX_TIME   = 30.0
Z_LEVEL    = 2.0

R0         = 6.0                     # initial horizontal distance (card)
INIT_PURSUER = np.array([0.0, 0.0, Z_LEVEL])
BETAS_DEG  = [0, 15, 30]
DZS        = [0.0, 1.0, 2.0]

# feasibility grid and climb-angle sweep
GRID_R0    = np.arange(2.0, 12.01, 1.0)
GRID_DZ    = np.arange(0.0, 4.01, 0.5)
GRID_BETA  = 15
SWEEP_BETA = np.arange(0, 46, 5)
N_T_CAND   = 400                     # intercept-time candidates per re-plan


def init_evader(dz):
    return np.array([R0, 0.0, Z_LEVEL + dz])


# ── Energy-optimal intercept ─────────────────────────────────

def energy_opt_velocity(p, e, v_e, e_left, v_max=V_MAX):
    """Constant-velocity intercept minimising P(v) * T, |v| <= v_max, T within
    the remaining battery. Returns (velocity, planned T, planned energy)."""
    D = e - p
    a = v_e @ v_e - v_max**2
    b = 2.0 * D @ v_e
    c = D @ D
    if a >= 0:                                    # evader not slower: chase directly
        return v_max * D / (np.linalg.norm(D) + 1e-12), np.inf, np.inf
    t_min = (-b - np.sqrt(b * b - 4 * a * c)) / (2 * a)   # fastest intercept at v_max
    t_max = max(e_left / P_HOVER, t_min)                    # cannot fly longer than this
    T = t_min * np.geomspace(1.0, max(t_max / t_min, 1.0 + 1e-9), N_T_CAND)
    V = D[None, :] / T[:, None] + v_e[None, :]
    vz = V[:, 2]
    P = P_HOVER + K_H * (V[:, 0]**2 + V[:, 1]**2) + K_V * np.where(vz > 0, vz, 0.0)**2
    E = P * T
    k = int(np.argmin(E))
    v = V[k]
    s = np.linalg.norm(v)
    if s > v_max:
        v = v * v_max / s
    return v, T[k], E[k]


# ── Simulation ───────────────────────────────────────────────

def run_simulation(strat, beta_deg, dz, v_p=V_MAX, r0=R0):
    beta = np.radians(beta_deg)
    v_e_cmd = V_EVADER * np.array([np.cos(beta), 0.0, np.sin(beta)])
    p = INIT_PURSUER.copy()
    e = np.array([r0, 0.0, Z_LEVEL + dz])
    p_en, e_en = BATTERY, BATTERY
    p_traj, e_traj, p_log, e_log, spd = [p.copy()], [e.copy()], [p_en], [e_en], []
    outcome, t_end = 'timeout', MAX_TIME
    e_dead = False
    plan0 = None

    for step in range(int(MAX_TIME / DT)):
        t = step * DT
        D = e - p
        dist = np.linalg.norm(D)
        if dist < CAPTURE_R:
            outcome, t_end = 'captured', t
            break
        if p_en <= 0.0:
            outcome, t_end = 'pursuer_battery', t
            break

        v_e = np.zeros(3) if e_dead else v_e_cmd
        if strat == 'level':
            d = np.array([D[0], D[1], Z_LEVEL - p[2]])
            v = v_p * d / (np.linalg.norm(d) + 1e-12)
        elif strat == 'direct_3d':
            v = v_p * D / (dist + 1e-12)
        else:
            v, T_plan, E_plan = energy_opt_velocity(p, e, v_e, p_en)
            if plan0 is None:
                plan0 = (np.linalg.norm(v), v[2], T_plan, E_plan)
        spd.append(np.linalg.norm(v))

        p = p + v * DT
        p_en = max(0.0, p_en - power(v) * DT)
        if not e_dead:
            e = e + v_e * DT
            e_en = max(0.0, e_en - power(v_e) * DT)
            e_dead = e_en <= 0.0
        p_traj.append(p.copy()); e_traj.append(e.copy())
        p_log.append(p_en); e_log.append(e_en)

    return dict(p=np.array(p_traj), e=np.array(e_traj), p_en=np.array(p_log),
                e_en=np.array(e_log), outcome=outcome, t=t_end, strat=strat,
                beta=beta_deg, dz=dz, v_p=v_p, plan0=plan0,
                used=BATTERY - p_log[-1], mean_speed=float(np.mean(spd)) if spd else 0.0,
                gap=float(np.linalg.norm(e_traj[-1] - p_traj[-1])))


def label(r):
    if r['strat'] == 'energy_opt':
        return 'energy-opt'
    return f'{r["strat"].replace("_3d", " 3D")} {r["v_p"]:.0f} m/s'


def status(r):
    if r['outcome'] == 'captured':
        return f'captured {r["t"]:.2f} s, {r["used"]:.1f} J used'
    if r['outcome'] == 'pursuer_battery':
        return f'battery out {r["t"]:.2f} s, gap {r["gap"]:.2f} m'
    return f'timeout, gap {r["gap"]:.2f} m'


# ── Plots ─────────────────────────────────────────────────────

def plot_trajectories_3d(res):
    fig = plt.figure(figsize=(18, 15))
    show = [('level', 5.0, 'gray'), ('direct_3d', 5.0, 'red'), ('energy_opt', 5.0, 'darkorange')]
    for bi, beta in enumerate(BETAS_DEG):
        for di, dz in enumerate(DZS):
            ax = fig.add_subplot(3, 3, bi * 3 + di + 1, projection='3d')
            ax.view_init(elev=18, azim=-60)
            ev = res[('direct_3d', 5.0, beta, dz)]['e']
            ax.plot(*ev.T, color='blue', linewidth=1.6, label='Evader (direct 3D run)')
            ax.scatter(*ev[0], color='blue', s=30)
            for s, v, c in show:
                r = res[(s, v, beta, dz)]
                ax.plot(*r['p'].T, color=c, linewidth=1.4, label=f'{label(r)}: {status(r)}')
                ax.scatter(*r['p'][-1], color=c, s=40, marker='X')
            ax.scatter(*INIT_PURSUER, color='red', s=30)
            ax.set_xlabel('X', fontsize=7); ax.set_ylabel('Y', fontsize=7); ax.set_zlabel('Z', fontsize=7)
            ax.tick_params(labelsize=6)
            ax.set_ylim(-1, 1)
            ax.set_title(f'β = {beta}°, Δz = {dz:.0f} m', fontsize=10)
            ax.legend(fontsize=6, loc='upper left')
    fig.suptitle('S006 3D Energy Race — pursuers at 5 m/s (rows: evader climb angle β, '
                 'cols: initial altitude gap Δz)', fontsize=12)
    path = os.path.join(OUTPUT_DIR, 'trajectories_3d.png')
    plt.savefig(path, dpi=100, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_energy_curves(res, beta=15, dz=1.0):
    fig, axes = plt.subplots(1, 2, figsize=(15, 5), sharey=True)
    cols = {('direct_3d', 3.0): 'pink', ('direct_3d', 4.0): 'salmon', ('direct_3d', 5.0): 'red',
            ('level', 5.0): 'gray', ('energy_opt', 5.0): 'darkorange'}
    for ax, (b, z) in zip(axes, [(0, 0.0), (beta, dz)]):
        for (s, v), c in cols.items():
            r = res[(s, v, b, z)]
            t = np.arange(len(r['p_en'])) * DT
            ax.plot(t, r['p_en'], color=c, linewidth=2, label=f'{label(r)}: {status(r)}')
        r = res[('direct_3d', 5.0, b, z)]
        ax.plot(np.arange(len(r['e_en'])) * DT, r['e_en'], color='blue', linestyle='--',
                label='evader (direct 3D 5 m/s run)')
        ax.axhline(0, color='black', linewidth=0.8)
        ax.set_xlabel('Time (s)')
        ax.set_title(f'β = {b}°, Δz = {z:.0f} m')
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=7, loc='upper right')
    axes[0].set_ylabel('Battery energy (J)')
    fig.suptitle('S006 3D — Pursuer battery vs time (runs end at capture or empty battery)',
                 fontsize=12)
    plt.tight_layout()
    path = os.path.join(OUTPUT_DIR, 'energy_curves.png')
    plt.savefig(path, dpi=120)
    plt.close()
    print(f'Saved: {path}')


def plot_feasibility_grid(grid):
    fig, axes = plt.subplots(1, 2, figsize=(15, 5.5), sharey=True)
    for ax, s in zip(axes, ['direct_3d', 'energy_opt']):
        g = grid[s]
        im = ax.imshow(g, origin='lower', aspect='auto', cmap='RdYlGn', vmin=0, vmax=BATTERY,
                       extent=[GRID_R0[0] - 0.5, GRID_R0[-1] + 0.5,
                               GRID_DZ[0] - 0.25, GRID_DZ[-1] + 0.25])
        for i, dz in enumerate(GRID_DZ):
            for j, r0 in enumerate(GRID_R0):
                v = g[i, j]
                ax.text(r0, dz, 'x' if np.isnan(v) else f'{v:.0f}', ha='center', va='center',
                        fontsize=6)
        ax.set_xlabel('Initial horizontal distance r0 (m)')
        ax.set_title('Direct 3D, 5 m/s' if s == 'direct_3d' else 'Energy-opt (≤ 5 m/s)')
    axes[0].set_ylabel('Initial altitude gap Δz, evader above (m)')
    fig.colorbar(im, ax=axes, label='Pursuer energy left at capture (J)')
    fig.suptitle(f'S006 3D — Feasibility grid, evader climb angle β = {GRID_BETA}°: '
                 f'pursuer energy left at capture (J), x = no capture', fontsize=12)
    path = os.path.join(OUTPUT_DIR, 'feasibility_grid.png')
    plt.savefig(path, dpi=120, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_climb_angle_sweep(sweep, v_free):
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    for s, c in (('direct_3d', 'red'), ('energy_opt', 'darkorange')):
        used = [r['used'] if r['outcome'] == 'captured' else np.nan for r in sweep[s]]
        tc = [r['t'] if r['outcome'] == 'captured' else np.nan for r in sweep[s]]
        name = 'direct 3D 5 m/s' if s == 'direct_3d' else 'energy-opt'
        axes[0].plot(SWEEP_BETA, used, 'o-', color=c, label=name)
        axes[1].plot(SWEEP_BETA, tc, 'o-', color=c, label=name)
        fail = [b for b, r in zip(SWEEP_BETA, sweep[s]) if r['outcome'] != 'captured']
        if fail:
            axes[0].plot(fail, [BATTERY] * len(fail), 'x', color=c, markersize=10,
                         label=f'{name}: no capture')
    axes[0].axhline(BATTERY, color='black', linestyle=':', label='battery 80 J')
    axes[0].set_ylabel('Pursuer energy used to capture (J)')
    axes[1].set_ylabel('Capture time (s)')
    sp = [r['plan0'][0] for r in sweep['energy_opt']]
    ax2 = axes[1].twinx()
    ax2.plot(SWEEP_BETA, sp, 's--', color='purple', label='energy-opt planned speed at t = 0')
    ax2.plot(SWEEP_BETA, v_free, 'd:', color='purple', alpha=0.6,
             label='optimal speed without the 5 m/s limit')
    ax2.set_ylabel('Planned speed (m/s)', color='purple')
    ax2.set_ylim(0, max(v_free) * 1.15)
    for ax in axes:
        ax.set_xlabel('Evader climb angle β (°)')
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8, loc='upper left')
    ax2.legend(fontsize=8, loc='lower left')
    fig.suptitle('S006 3D — Evader climb-angle sweep (Δz = 0, r0 = 6 m)', fontsize=12)
    plt.tight_layout()
    path = os.path.join(OUTPUT_DIR, 'climb_angle_sweep.png')
    plt.savefig(path, dpi=120)
    plt.close()
    print(f'Saved: {path}')


def save_animation(res, beta=30, dz=2.0):
    runs = [res[('direct_3d', 5.0, beta, dz)], res[('energy_opt', 5.0, beta, dz)]]
    step_skip = 3
    n_steps = max(len(r['p']) for r in runs)
    frames = list(range(0, n_steps, step_skip))
    if frames[-1] != n_steps - 1:
        frames.append(n_steps - 1)
    pts = np.vstack([np.vstack([r['p'], r['e']]) for r in runs])
    lo, hi = pts.min(axis=0) - 0.5, pts.max(axis=0) + 0.5

    fig = plt.figure(figsize=(13, 5.5))
    panels = []
    for i, r in enumerate(runs):
        ax = fig.add_subplot(1, 2, i + 1, projection='3d')
        ax.view_init(elev=18, azim=-60)
        ax.set_xlim(lo[0], hi[0]); ax.set_ylim(-1.5, 1.5); ax.set_zlim(0, hi[2])
        ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
        pl, = ax.plot([], [], [], 'r-', linewidth=1.6)
        el, = ax.plot([], [], [], 'b-', linewidth=1.6)
        pd, = ax.plot([], [], [], 'r^', markersize=8)
        ed, = ax.plot([], [], [], 'bs', markersize=8)
        txt = ax.text2D(0.02, 0.92, '', transform=ax.transAxes, fontsize=9)
        ax.set_title(f'{label(r)}: {status(r)}', fontsize=10)
        panels.append((r, pl, el, pd, ed, txt))
    title = fig.suptitle('', fontsize=11)

    def update(i):
        k = frames[i]
        for r, pl, el, pd, ed, txt in panels:
            j = min(k, len(r['p']) - 1)
            P, E = r['p'][:j + 1], r['e'][:j + 1]
            pl.set_data(P[:, 0], P[:, 1]); pl.set_3d_properties(P[:, 2])
            el.set_data(E[:, 0], E[:, 1]); el.set_3d_properties(E[:, 2])
            pd.set_data([P[-1, 0]], [P[-1, 1]]); pd.set_3d_properties([P[-1, 2]])
            ed.set_data([E[-1, 0]], [E[-1, 1]]); ed.set_3d_properties([E[-1, 2]])
            txt.set_text(f'pursuer battery {r["p_en"][j]:.1f} J')
        title.set_text(f'S006 3D — evader β = {beta}°, Δz = {dz:.0f} m   t = {k * DT:.2f} s')
        return []

    ani = FuncAnimation(fig, update, frames=len(frames), interval=60, blit=False)
    path = os.path.join(OUTPUT_DIR, 'animation.gif')
    ani.save(path, writer=PillowWriter(fps=16), dpi=80)
    plt.close()
    print(f'Saved: {path}')


# ── Main ─────────────────────────────────────────────────────

def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    print('Power check: level 5 m/s = %.1f W, level 4 m/s = %.1f W, level 3 m/s = %.1f W, '
          'evader 3 m/s at beta 0/15/30 = %.2f / %.2f / %.2f W'
          % (power(np.array([5.0, 0, 0])), power(np.array([4.0, 0, 0])),
             power(np.array([3.0, 0, 0])),
             *[power(V_EVADER * np.array([np.cos(np.radians(b)), 0, np.sin(np.radians(b))]))
               for b in BETAS_DEG]))
    print(f'Battery life at those powers: {BATTERY / power(np.array([5.0, 0, 0])):.2f} / '
          f'{BATTERY / power(np.array([4.0, 0, 0])):.2f} / {BATTERY / power(np.array([3.0, 0, 0])):.2f} s')

    res = {}
    print(f'\nMain grid: r0 = {R0} m, pursuer (0, 0, 2), evader ({R0}, 0, 2 + dz)')
    for beta in BETAS_DEG:
        for dz in DZS:
            print(f'  beta = {beta:2d} deg, dz = {dz:.0f} m:')
            for s in ('level', 'direct_3d'):
                for v in V_OPTIONS:
                    r = run_simulation(s, beta, dz, v)
                    res[(s, v, beta, dz)] = r
                    print(f'    {label(r):>17}: {status(r)}')
            r = run_simulation('energy_opt', beta, dz)
            res[('energy_opt', 5.0, beta, dz)] = r
            sp0, vz0, T0, E0 = r['plan0']
            print(f'    {label(r):>17}: {status(r)}; plan at t=0: speed {sp0:.2f} m/s, '
                  f'v_z {vz0:+.2f} m/s, T {T0:.2f} s, E {E0:.1f} J; mean speed {r["mean_speed"]:.2f} m/s')
            d5, eo = res[('direct_3d', 5.0, beta, dz)], r
            if d5['outcome'] == 'captured' and eo['outcome'] == 'captured':
                print(f'    energy-opt vs direct 3D 5 m/s: {eo["used"] - d5["used"]:+.2f} J, '
                      f'{eo["t"] - d5["t"]:+.2f} s')

    print(f'\nFeasibility grid (beta = {GRID_BETA} deg): r0 {GRID_R0[0]:.0f}..{GRID_R0[-1]:.0f} m, '
          f'dz {GRID_DZ[0]:.1f}..{GRID_DZ[-1]:.1f} m')
    grid = {}
    for s in ('direct_3d', 'energy_opt'):
        g = np.full((len(GRID_DZ), len(GRID_R0)), np.nan)
        for i, dz in enumerate(GRID_DZ):
            for j, r0 in enumerate(GRID_R0):
                r = run_simulation(s, GRID_BETA, dz, V_MAX, r0)
                if r['outcome'] == 'captured':
                    g[i, j] = BATTERY - r['used']
        grid[s] = g
        print(f'  {s:>10}: {int(np.sum(~np.isnan(g)))}/{g.size} cells captured; largest r0 '
              f'captured at dz = 0: {GRID_R0[~np.isnan(g[0])].max():.0f} m, at dz = 4: '
              + (f'{GRID_R0[~np.isnan(g[-1])].max():.0f} m' if (~np.isnan(g[-1])).any() else 'none'))
    both = ~np.isnan(grid['direct_3d']) & ~np.isnan(grid['energy_opt'])
    diff = grid['energy_opt'][both] - grid['direct_3d'][both]
    only_eo = int(np.sum(np.isnan(grid['direct_3d']) & ~np.isnan(grid['energy_opt'])))
    print(f'  cells captured by both: {int(both.sum())}; energy-opt leaves {diff.mean():+.2f} J '
          f'more on average (min {diff.min():+.2f}, max {diff.max():+.2f}); captured only by '
          f'energy-opt: {only_eo}')

    print('\nClimb-angle sweep (dz = 0, r0 = 6 m):')
    sweep = {'direct_3d': [], 'energy_opt': []}
    v_free = []
    for b in SWEEP_BETA:
        v_e0 = V_EVADER * np.array([np.cos(np.radians(b)), 0.0, np.sin(np.radians(b))])
        v_free.append(float(np.linalg.norm(
            energy_opt_velocity(INIT_PURSUER, init_evader(0.0), v_e0, BATTERY, v_max=100.0)[0])))
        row = []
        for s in ('direct_3d', 'energy_opt'):
            r = run_simulation(s, int(b), 0.0)
            sweep[s].append(r)
            row.append(f'{s} {status(r)}')
        print(f'  beta {b:2d} deg: ' + '; '.join(row) +
              f'; energy-opt planned speed {sweep["energy_opt"][-1]["plan0"][0]:.2f} m/s '
              f'(without the 5 m/s limit: {v_free[-1]:.2f} m/s)')

    print('\nGenerating plots...')
    plot_trajectories_3d(res)
    plot_energy_curves(res)
    plot_feasibility_grid(grid)
    plot_climb_angle_sweep(sweep, v_free)
    print('Generating animation...')
    save_animation(res)
    print('S006 3D done.')


if __name__ == '__main__':
    main()
