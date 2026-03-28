"""
S006 Energy Race
=================
Pursuer and evader both carry limited batteries.  Power consumption scales as
P = k * v².  The pursuer must capture the evader before either battery runs dry.

Setup:
  - Pursuer starts at (0, 0, 2), evader starts at (6, 0, 2) — 6 m apart.
  - Evader flies in the +x direction at constant 3 m/s (battery limited).
  - Pursuer uses pure pursuit at one of three fixed speeds: 3, 4, 5 m/s.

Battery model:
  E0 = 80 J, k = 0.5 W·s²/m²  →  dE/dt = -k·v²

Results:
  v=3 m/s  →  pursuer == evader speed, gap never closes  →  BATTERY OUT
  v=4 m/s  →  T_cap ≈ 6 s < T_max = 10 s              →  captured
  v=5 m/s  →  T_cap ≈ 3 s < T_max = 6.4 s             →  captured

Usage:
    conda activate drones
    python src/01_pursuit_evasion/s006_energy_race.py
"""

import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from src.base.drone_base import DroneBase

# ── Scenario parameters ─────────────────────────────────────
K_ENERGY    = 0.5       # W·s²/m²  (power = k * v²)
E0          = 80.0      # J  — both pursuer and evader start with this
V_EVADER    = 3.0       # m/s constant straight flight
R0          = 6.0       # m  initial distance
CAPTURE_R   = 0.15      # m
DT          = 1 / 48    # s
MAX_TIME    = 30.0      # s

PURSUER_SPEEDS = [3.0, 4.0, 5.0]   # m/s options to compare

INIT_PURSUER = np.array([0.0, 0.0, 2.0])
INIT_EVADER  = np.array([R0,  0.0, 2.0])
EVADER_DIR   = np.array([1.0, 0.0, 0.0])   # constant +x

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '01_pursuit_evasion', 's006_energy_race',
)


# ── Analytical helpers ───────────────────────────────────────

def max_flight_time(v, e0=E0, k=K_ENERGY):
    """Maximum hover/flight time before battery depletes at constant speed v."""
    return e0 / (k * v ** 2)


def analytical_cap_time(r0, v_p, v_e=V_EVADER):
    """Approximate capture time for head-on pure pursuit (upper bound)."""
    if v_p <= v_e:
        return np.inf
    return r0 / (v_p - v_e)


def print_feasibility():
    print("\nFeasibility check (analytical):")
    print(f"{'Speed':>8}  {'T_cap (s)':>10}  {'T_max (s)':>10}  {'Result':>12}")
    for v_p in PURSUER_SPEEDS:
        t_cap = analytical_cap_time(R0, v_p)
        t_max = max_flight_time(v_p)
        ok    = t_cap <= t_max
        print(f"{v_p:>7.1f}  {t_cap:>10.2f}  {t_max:>10.2f}  "
              f"{'OK' if ok else 'BATTERY OUT':>12}")
    print()


# ── Simulation ──────────────────────────────────────────────

def run_simulation(v_pursuer):
    """
    Pure pursuit at constant speed v_pursuer vs straight-flying evader at V_EVADER.
    Both have finite battery E0.  Simulation ends on capture, pursuer battery empty,
    or timeout.

    Returns:
        p_traj      (N, 3)
        e_traj      (N, 3)
        e_energy    (N,)   evader energy vs step
        p_energy    (N,)   pursuer energy vs step
        times       (N,)
        captured    bool
        cap_time    float or None
        p_out       bool   pursuer battery ran out
        e_out       bool   evader battery ran out
    """
    pursuer = DroneBase(INIT_PURSUER.copy(), max_speed=v_pursuer, dt=DT)
    evader  = DroneBase(INIT_EVADER.copy(),  max_speed=V_EVADER,  dt=DT)

    p_energy = E0
    e_energy = E0
    max_steps = int(MAX_TIME / DT)

    p_traj_list = [pursuer.pos.copy()]
    e_traj_list = [evader.pos.copy()]
    p_energy_log = [p_energy]
    e_energy_log = [e_energy]
    time_log = [0.0]

    captured   = False
    cap_time   = None
    p_out      = False
    e_out      = False

    for step in range(1, max_steps + 1):
        t = step * DT

        # ── Pursuer pure pursuit ──
        diff = evader.pos - pursuer.pos
        dist = np.linalg.norm(diff)

        if dist < CAPTURE_R:
            captured = True
            cap_time = t
            break

        cmd_dir = diff / dist if dist > 1e-8 else diff
        pursuer.step(cmd_dir * v_pursuer)
        p_energy -= K_ENERGY * v_pursuer ** 2 * DT

        # ── Evader straight flight ──
        evader.step(EVADER_DIR * V_EVADER)
        e_energy -= K_ENERGY * V_EVADER ** 2 * DT

        p_energy = max(p_energy, 0.0)
        e_energy = max(e_energy, 0.0)

        p_traj_list.append(pursuer.pos.copy())
        e_traj_list.append(evader.pos.copy())
        p_energy_log.append(p_energy)
        e_energy_log.append(e_energy)
        time_log.append(t)

        if p_energy <= 0.0 and not captured:
            p_out = True
            break
        if e_energy <= 0.0 and not captured:
            e_out = True
            # Evader has stopped — pursuer can still close in; continue
            # (evader holds position from now on)

    return (
        np.array(p_traj_list),
        np.array(e_traj_list),
        np.array(p_energy_log),
        np.array(e_energy_log),
        np.array(time_log),
        captured,
        cap_time,
        p_out,
        e_out,
    )


# ── Plots ────────────────────────────────────────────────────

def plot_trajectories_3d(results, out_dir):
    """Top-down XY view + 3D inset for all three speed cases."""
    colors = ['steelblue', 'darkorange', 'mediumseagreen']
    labels = [f'v={v} m/s' for v in PURSUER_SPEEDS]

    fig = plt.figure(figsize=(16, 5))

    for idx, (res, color, label) in enumerate(zip(results, colors, labels)):
        p_traj, e_traj, p_en, e_en, times, captured, cap_t, p_out, e_out = res

        ax = fig.add_subplot(1, 3, idx + 1, projection='3d')
        ax.view_init(elev=25, azim=-60)

        ax.plot(p_traj[:, 0], p_traj[:, 1], p_traj[:, 2],
                color='red', linewidth=1.8, label='Pursuer')
        ax.plot(e_traj[:, 0], e_traj[:, 1], e_traj[:, 2],
                color='royalblue', linewidth=1.8, label='Evader')

        ax.scatter(*p_traj[0], color='red',  s=50, marker='o')
        ax.scatter(*e_traj[0], color='blue', s=50, marker='o')

        if captured:
            ax.scatter(*p_traj[-1], color='black', s=100, marker='X',
                       label=f'Captured {cap_t:.2f}s')
            status = f'Captured {cap_t:.2f} s'
        elif p_out:
            ax.scatter(*p_traj[-1], color='gray', s=80, marker='X')
            status = 'Battery out'
        else:
            status = 'Timeout'

        ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
        ax.set_title(f'{label}\n{status}', fontsize=9)
        ax.legend(fontsize=7, loc='upper left')

    fig.suptitle('S006 Energy Race — 3D Trajectories (red=Pursuer, blue=Evader)',
                 fontsize=11)
    plt.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'trajectories_3d.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_energy_vs_time(results, out_dir):
    """Energy remaining vs time for pursuer (solid) and evader (dashed)."""
    colors = ['steelblue', 'darkorange', 'mediumseagreen']
    labels = [f'v={v} m/s' for v in PURSUER_SPEEDS]

    fig, axes = plt.subplots(1, 3, figsize=(15, 4), sharey=True)

    for ax, res, color, label in zip(axes, results, colors, labels):
        p_traj, e_traj, p_en, e_en, times, captured, cap_t, p_out, e_out = res

        ax.plot(times, p_en, color=color,   linewidth=2.0, label='Pursuer energy')
        ax.plot(times, e_en, color='royalblue', linewidth=1.5, linestyle='--',
                label='Evader energy')

        # Shade "dead" zone
        ax.axhline(0, color='black', linestyle=':', linewidth=0.8, alpha=0.5)

        if captured:
            ax.axvline(cap_t, color='black', linestyle='--', linewidth=1.2,
                       label=f'Captured {cap_t:.2f}s')
            ax.fill_betweenx([0, E0], cap_t, times[-1],
                             color='lightgreen', alpha=0.15, label='After capture')

        t_max_p = max_flight_time(PURSUER_SPEEDS[labels.index(label)])
        ax.axvline(t_max_p, color=color, linestyle=':', linewidth=1.2, alpha=0.7,
                   label=f'T_max_p={t_max_p:.1f}s')

        ax.set_xlabel('Time (s)')
        if ax == axes[0]:
            ax.set_ylabel('Energy (J)')
        ax.set_title(f'{label}', fontsize=10)
        ax.set_ylim(-2, E0 + 5)
        ax.legend(fontsize=7, loc='upper right')
        ax.grid(True, alpha=0.3)

    fig.suptitle('S006 Energy Race — Energy Remaining vs Time', fontsize=11)
    plt.tight_layout()
    path = os.path.join(out_dir, 'energy_vs_time.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_capture_bar(results, out_dir):
    """Bar chart: capture time per speed (infeasible shown in red with label)."""
    labels = [f'v={v} m/s' for v in PURSUER_SPEEDS]
    cap_times = []
    bar_colors = []
    annotations = []

    for res, v in zip(results, PURSUER_SPEEDS):
        _, _, _, _, times, captured, cap_t, p_out, _ = res
        if captured:
            cap_times.append(cap_t)
            bar_colors.append('mediumseagreen')
            annotations.append(f'{cap_t:.2f} s')
        else:
            # Show T_max as the bar height, but mark as failed
            cap_times.append(max_flight_time(v))
            bar_colors.append('tomato')
            if p_out:
                annotations.append('Battery out')
            else:
                annotations.append('Timeout')

    fig, ax = plt.subplots(figsize=(8, 5))
    bars = ax.bar(labels, cap_times, color=bar_colors, edgecolor='black',
                  width=0.4)

    for bar, ann, color in zip(bars, annotations, bar_colors):
        y = bar.get_height()
        ax.text(bar.get_x() + bar.get_width() / 2, y + 0.2,
                ann, ha='center', va='bottom', fontsize=10,
                color='black' if color != 'tomato' else 'darkred', fontweight='bold')

    # Analytical T_cap lines
    for v in PURSUER_SPEEDS:
        t_cap = analytical_cap_time(R0, v)
        if not np.isinf(t_cap):
            ax.axhline(t_cap, color='grey', linestyle='--', linewidth=0.8, alpha=0.5)

    ax.set_ylabel('Time (s)')
    ax.set_title('S006 Energy Race — Capture Time by Pursuer Speed\n'
                 '(green = captured | red = battery exhausted before capture)',
                 fontsize=10)
    ax.set_ylim(0, max(cap_times) * 1.25)
    ax.grid(axis='y', alpha=0.3)

    # Legend patches
    from matplotlib.patches import Patch
    legend_elements = [
        Patch(facecolor='mediumseagreen', edgecolor='black', label='Captured'),
        Patch(facecolor='tomato', edgecolor='black', label='Battery out / Timeout'),
    ]
    ax.legend(handles=legend_elements, fontsize=9)

    plt.tight_layout()
    path = os.path.join(out_dir, 'capture_bar.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def plot_feasibility_boundary(out_dir):
    """
    Feasibility boundary: for each initial distance r0, the minimum pursuer speed
    required such that T_cap(v_p) <= T_max(v_p).

    Condition: r0 / (v_p - v_e) = E0 / (k * v_p^2)
    → k * v_p^2 * r0 = E0 * (v_p - v_e)
    → k * r0 * v_p^2 - E0 * v_p + E0 * v_e = 0
    → quadratic in v_p.
    """
    r0_range = np.linspace(0.5, 30, 300)
    v_min_boundary = []

    for r0 in r0_range:
        # k*r0 * v^2 - E0*v + E0*v_e = 0
        a = K_ENERGY * r0
        b = -E0
        c = E0 * V_EVADER
        disc = b ** 2 - 4 * a * c
        if disc < 0:
            v_min_boundary.append(np.nan)
        else:
            v_sol = (-b + np.sqrt(disc)) / (2 * a)
            v_min_boundary.append(v_sol)

    v_min_boundary = np.array(v_min_boundary)

    fig, ax = plt.subplots(figsize=(9, 5))

    ax.plot(r0_range, v_min_boundary, color='steelblue', linewidth=2.0,
            label='Min feasible pursuer speed')
    ax.axhline(V_EVADER, color='royalblue', linestyle='--', linewidth=1.2,
               label=f'Evader speed ({V_EVADER} m/s)')

    # Shade feasible region
    ax.fill_between(r0_range, v_min_boundary, np.nanmax(v_min_boundary) * 1.1,
                    alpha=0.12, color='mediumseagreen', label='Feasible (capture)')
    ax.fill_between(r0_range, V_EVADER, v_min_boundary,
                    alpha=0.12, color='tomato', label='Infeasible (battery out)')

    # Mark our scenario points
    for v_p, col in zip(PURSUER_SPEEDS, ['steelblue', 'darkorange', 'mediumseagreen']):
        t_cap = analytical_cap_time(R0, v_p)
        t_max = max_flight_time(v_p)
        feasible = t_cap <= t_max
        marker = 'o' if feasible else 'X'
        ax.scatter(R0, v_p, color=col, s=120, marker=marker, zorder=5,
                   label=f'v={v_p} m/s ({"OK" if feasible else "FAIL"})')

    ax.set_xlabel('Initial Distance r₀ (m)')
    ax.set_ylabel('Pursuer Speed (m/s)')
    ax.set_title('S006 Energy Race — Feasibility Boundary\n'
                 'Minimum pursuer speed to capture before battery runs out',
                 fontsize=10)
    ax.set_xlim(0.5, 30)
    ax.set_ylim(V_EVADER * 0.9, max(np.nanmax(v_min_boundary), max(PURSUER_SPEEDS)) * 1.15)
    ax.legend(fontsize=8, loc='upper left')
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    path = os.path.join(out_dir, 'feasibility_boundary.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def save_animation(results, out_dir):
    """XY-plane animation of all three speed cases side by side.

    Duration follows the longest simulation (v=3 battery-out case).
    Shorter simulations freeze at their last frame once they end.
    """
    import matplotlib.animation as animation

    colors = ['steelblue', 'darkorange', 'mediumseagreen']
    labels = [f'v={v} m/s' for v in PURSUER_SPEEDS]

    # Drive animation by the longest trajectory
    max_len = max(len(res[0]) for res in results)
    step = 3

    fig, axes = plt.subplots(1, 3, figsize=(15, 5))

    lines_p, lines_e, dots_p, dots_e, titles = [], [], [], [], []
    for ax, color, label, res in zip(axes, colors, labels, results):
        p_traj, e_traj = res[0], res[1]
        all_pts = np.vstack([p_traj, e_traj])
        lo = all_pts.min(axis=0)[:2] - 1.0
        hi = all_pts.max(axis=0)[:2] + 1.0

        ax.set_xlim(lo[0], hi[0]); ax.set_ylim(lo[1], hi[1])
        ax.set_aspect('equal'); ax.grid(True, alpha=0.3)
        ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
        ax.scatter(*p_traj[0, :2], color='red',  s=60, zorder=5)
        ax.scatter(*e_traj[0, :2], color='blue', s=60, zorder=5)

        lp, = ax.plot([], [], color='red',       linewidth=1.8, alpha=0.8)
        le, = ax.plot([], [], color='royalblue',  linewidth=1.8, alpha=0.8)
        dp, = ax.plot([], [], 'r^', markersize=9, zorder=6)
        de, = ax.plot([], [], 'bs', markersize=9, zorder=6)
        ti  = ax.set_title(label, fontsize=9)

        lines_p.append(lp); lines_e.append(le)
        dots_p.append(dp);  dots_e.append(de)
        titles.append(ti)

    n_frames = max_len // step

    def update(i):
        artists = []
        for idx, res in enumerate(results):
            p_traj, e_traj, p_en, e_en, times, captured, cap_t, p_out, _ = res
            # Clamp to last step when this simulation has already ended
            si = min(i * step, len(p_traj) - 1)
            done = (i * step >= len(p_traj) - 1)

            px = p_traj[:si+1, 0]; py = p_traj[:si+1, 1]
            ex = e_traj[:si+1, 0]; ey = e_traj[:si+1, 1]
            lines_p[idx].set_data(px, py)
            lines_e[idx].set_data(ex, ey)
            dots_p[idx].set_data([float(px[-1])], [float(py[-1])])
            dots_e[idx].set_data([float(ex[-1])], [float(ey[-1])])

            t = times[si]
            en = p_en[si]
            v_p = PURSUER_SPEEDS[idx]
            t_max = max_flight_time(v_p)
            pct = 100.0 * en / E0

            if done:
                if captured:
                    status = f'✓ Captured {cap_t:.2f}s  [DONE]'
                else:
                    status = f'✗ Battery out {t:.2f}s  [DONE]'
                titles[idx].set_text(
                    f'v={v_p} m/s  t={t:.1f}s\n{status}'
                )
            else:
                titles[idx].set_text(
                    f'v={v_p} m/s  t={t:.1f}s\n'
                    f'Energy: {en:.1f}J ({pct:.0f}%)  T_max={t_max:.1f}s'
                )
            artists += [lines_p[idx], lines_e[idx], dots_p[idx], dots_e[idx], titles[idx]]
        return artists

    ani = animation.FuncAnimation(fig, update, frames=n_frames,
                                  interval=60, blit=True)
    fig.suptitle('S006 Energy Race — Pure Pursuit at Three Speeds', fontsize=11)
    plt.tight_layout()

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer='pillow', fps=16, dpi=100)
    plt.close()
    print(f'Saved: {path}')


# ── Main ────────────────────────────────────────────────────

if __name__ == '__main__':
    print_feasibility()

    results = []
    for v_p in PURSUER_SPEEDS:
        res = run_simulation(v_p)
        p_traj, e_traj, p_en, e_en, times, captured, cap_t, p_out, e_out = res
        if captured:
            status = f'captured @ {cap_t:.2f} s  (energy left: {p_en[-1]:.1f} J)'
        elif p_out:
            status = (f'pursuer battery out @ {times[-1]:.2f} s  '
                      f'(dist = {np.linalg.norm(p_traj[-1]-e_traj[-1]):.2f} m)')
        else:
            status = 'timeout'
        print(f'v={v_p} m/s  →  {status}')
        results.append(res)

    out_dir = os.path.normpath(OUTPUT_DIR)
    plot_trajectories_3d(results, out_dir)
    plot_energy_vs_time(results, out_dir)
    plot_capture_bar(results, out_dir)
    plot_feasibility_boundary(out_dir)
    save_animation(results, out_dir)
