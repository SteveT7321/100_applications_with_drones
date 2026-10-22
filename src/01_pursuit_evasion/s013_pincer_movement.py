"""
S013 Pincer Movement
=====================
Two pursuers approach the evader from opposite sides (pincer slots at
phi = 0 deg and 180 deg around the evader, card) with a shrinking offset
R(t) = max(R0 - V_SHRINK*t, R_MIN).  Once R reaches R_MIN the two pursuers
close the pincer by flying straight at the evader (terminal phase).

Evader (card): flies perpendicular to the direction from the pursuers'
centroid,  v_E = V_E * Rot90 * (p_E - p_bar)/|p_E - p_bar|.
A second evader ("flee"): flies straight away from the centroid.
Both evaders react to the pursuers' current positions every step.

Comparison: coordinated pincer vs two independent pure-pursuit drones,
same speeds and start positions, against both evaders.

Usage:
    conda activate drones
    python src/01_pursuit_evasion/s013_pincer_movement.py
"""

import sys, os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from src.base.drone_base import DroneBase

# ── Parameters (card values) ─────────────────────────────────
V_PURSUER  = 5.0     # m/s
V_EVADER   = 3.5     # m/s
R0         = 3.0     # m   initial pincer offset radius
R_MIN      = 0.5     # m   minimum offset; then terminal phase (direct closing)
V_SHRINK   = 0.3     # m/s offset shrink rate
PHI        = (0.0, np.pi)   # pincer slot angles for P1, P2
CAPTURE_R  = 0.15    # m
DT         = 1 / 48
MAX_TIME   = 20.0

INIT_P1    = np.array([-4.0, -2.0, 2.0])
INIT_P2    = np.array([-4.0,  2.0, 2.0])
INIT_E     = np.array([ 2.0,  0.0, 2.0])

STRATEGIES = ['pincer', 'independent']
SWEEP_SHRINK = [0.3, 0.6, 1.0, 2.0]   # m/s pincer shrink-rate sensitivity
EVADERS    = ['perpendicular', 'flee']

P_COLORS   = ['red', 'darkorange']

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '01_pursuit_evasion', 's013_pincer_movement',
)


# ── Helpers ───────────────────────────────────────────────────

def pincer_angle(p1, p2, pe):
    d1 = p1 - pe; d2 = p2 - pe
    n1 = np.linalg.norm(d1); n2 = np.linalg.norm(d2)
    if n1 < 1e-8 or n2 < 1e-8:
        return 0.0
    cos_a = np.dot(d1, d2) / (n1 * n2)
    return np.degrees(np.arccos(np.clip(cos_a, -1, 1)))


def evader_velocity(p1, p2, pe, mode, prev_dir):
    """Card evader: perpendicular (+90 deg) to the centroid->evader direction.
    'flee': straight away from the centroid.  If the centroid coincides with
    the evader the previous direction is kept."""
    c = 0.5 * (p1 + p2)
    d = (pe - c)[:2]
    n = np.linalg.norm(d)
    if n < 1e-6:
        u = prev_dir
    else:
        d = d / n
        u = np.array([-d[1], d[0]]) if mode == 'perpendicular' else d
    return V_EVADER * np.array([u[0], u[1], 0.0]), u


# ── Simulation ────────────────────────────────────────────────

def run_simulation(strategy='pincer', evader_mode='perpendicular', v_shrink=V_SHRINK):
    p1 = DroneBase(INIT_P1.copy(), max_speed=V_PURSUER, dt=DT)
    p2 = DroneBase(INIT_P2.copy(), max_speed=V_PURSUER, dt=DT)
    ev = DroneBase(INIT_E.copy(),  max_speed=V_EVADER,  dt=DT)

    p1_traj = [p1.pos.copy()]; p2_traj = [p2.pos.copy()]
    e_traj  = [ev.pos.copy()]
    angle_log = [pincer_angle(p1.pos, p2.pos, ev.pos)]
    dist_log  = [[np.linalg.norm(p1.pos - ev.pos)], [np.linalg.norm(p2.pos - ev.pos)]]
    time_log  = [0.0]
    prev_dir  = np.array([1.0, 0.0])

    captured = False; cap_time = None; cap_who = None

    for step in range(1, int(round(MAX_TIME / DT)) + 1):
        t = step * DT

        if strategy == 'pincer' and R0 - v_shrink * t > R_MIN:
            R_cur = max(R0 - v_shrink * t, R_MIN)
            tgts = [ev.pos + R_cur * np.array([np.cos(ph), np.sin(ph), 0.0]) for ph in PHI]
        else:   # independent pure pursuit, or pincer terminal phase
            tgts = [ev.pos, ev.pos]

        for pursuer, tgt in zip((p1, p2), tgts):
            d = tgt - pursuer.pos
            n = np.linalg.norm(d)
            # saturated go-to-target (same controller for both strategies):
            # stop on the target if reachable this step -> no slot chattering
            pursuer.step(d / n * min(V_PURSUER, n / DT) if n > 1e-8 else np.zeros(3))

        e_vel, prev_dir = evader_velocity(p1.pos, p2.pos, ev.pos, evader_mode, prev_dir)
        ev.step(e_vel)

        for i, pursuer in enumerate((p1, p2)):
            if np.linalg.norm(pursuer.pos - ev.pos) < CAPTURE_R:
                captured = True; cap_time = t; cap_who = i
                break

        p1_traj.append(p1.pos.copy()); p2_traj.append(p2.pos.copy())
        e_traj.append(ev.pos.copy())
        angle_log.append(pincer_angle(p1.pos, p2.pos, ev.pos))
        dist_log[0].append(np.linalg.norm(p1.pos - ev.pos))
        dist_log[1].append(np.linalg.norm(p2.pos - ev.pos))
        time_log.append(t)

        if captured:
            break

    angle_log = np.array(angle_log); time_log = np.array(time_log)
    above = np.where(angle_log >= 170.0)[0]
    return dict(
        strategy=strategy, evader=evader_mode,
        p1=np.array(p1_traj), p2=np.array(p2_traj), e=np.array(e_traj),
        angle=angle_log, dist=[np.array(d) for d in dist_log], times=time_log,
        captured=captured, cap_time=cap_time, cap_who=cap_who,
        t170=float(time_log[above[0]]) if len(above) else None,
    )


# ── Plots ─────────────────────────────────────────────────────

def _key(r):
    return f"{r['strategy']} vs {r['evader']} evader"


def plot_trajectories_2d(results, out_dir):
    fig, axes = plt.subplots(2, 2, figsize=(13, 11))
    for ax, r in zip(axes.flat, results):
        ax.plot(r['p1'][:, 0], r['p1'][:, 1], color=P_COLORS[0], linewidth=1.6, label='Pursuer 1')
        ax.plot(r['p2'][:, 0], r['p2'][:, 1], color=P_COLORS[1], linewidth=1.6, label='Pursuer 2')
        ax.plot(r['e'][:, 0], r['e'][:, 1], color='royalblue', linewidth=1.8, label='Evader')
        for tr, c in ((r['p1'], P_COLORS[0]), (r['p2'], P_COLORS[1]), (r['e'], 'blue')):
            ax.scatter(*tr[0, :2], color=c, s=60, zorder=5)
        if r['captured']:
            tr = r['p1'] if r['cap_who'] == 0 else r['p2']
            ax.scatter(*tr[-1, :2], color='black', s=140, marker='X', zorder=7,
                       label=f"Captured by P{r['cap_who']+1} @ {r['cap_time']:.2f}s")
            status = f"Captured {r['cap_time']:.2f}s"
        else:
            status = 'Timeout'
        ax.set_aspect('equal', adjustable='datalim'); ax.grid(True, alpha=0.3)
        ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
        ax.set_title(f'{_key(r)}\n{status}', fontsize=9)
        ax.legend(fontsize=7, loc='best')
    fig.suptitle('S013 Pincer Movement — Top-Down Trajectories', fontsize=11)
    plt.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'trajectories_2d.png')
    plt.savefig(path, dpi=130, bbox_inches='tight'); plt.close(); print(f'Saved: {path}')


def plot_pincer_angle(results, out_dir):
    styles = {('pincer', 'perpendicular'): ('steelblue', '-'),
              ('independent', 'perpendicular'): ('darkorange', '-'),
              ('pincer', 'flee'): ('steelblue', '--'),
              ('independent', 'flee'): ('darkorange', '--')}
    fig, ax = plt.subplots(figsize=(11, 5))
    for r in results:
        c, ls = styles[(r['strategy'], r['evader'])]
        ax.plot(r['times'], r['angle'], color=c, linestyle=ls, linewidth=1.6, label=_key(r))
        if r['captured']:
            ax.scatter(r['cap_time'], r['angle'][-1], color=c, marker='X', s=60, zorder=5)
    ax.axhline(180, color='grey', linestyle=':', linewidth=1.0, label='Ideal pincer (180°)')
    t_term = (R0 - R_MIN) / V_SHRINK
    ax.axvline(t_term, color='green', linestyle=':', linewidth=1.0,
               label=f'Pincer R reaches R_min ({t_term:.2f}s)')
    ax.set_xlabel('Time (s)'); ax.set_ylabel('Pincer Angle (°)')
    ax.set_title('S013 Pincer Movement — Pincer Angle vs Time (X = capture)')
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'pincer_angle.png')
    plt.savefig(path, dpi=150); plt.close(); print(f'Saved: {path}')


def plot_distances(results, out_dir):
    fig, axes = plt.subplots(2, 2, figsize=(13, 8), sharey=True)
    for ax, r in zip(axes.flat, results):
        ax.plot(r['times'], r['dist'][0], color=P_COLORS[0], linewidth=1.6, label='Pursuer 1')
        ax.plot(r['times'], r['dist'][1], color=P_COLORS[1], linewidth=1.6, label='Pursuer 2')
        ax.axhline(CAPTURE_R, color='red', linestyle='--', linewidth=0.8,
                   label=f'Capture r={CAPTURE_R}m')
        if r['captured']:
            ax.axvline(r['cap_time'], color='black', linestyle='--', linewidth=1.0,
                       label=f"Captured {r['cap_time']:.2f}s")
        ax.set_xlabel('Time (s)'); ax.set_ylabel('Distance to Evader (m)')
        ax.set_title(_key(r), fontsize=9)
        ax.legend(fontsize=7); ax.grid(True, alpha=0.3)
    fig.suptitle('S013 Pincer Movement — Pursuer Distances to Evader', fontsize=11)
    plt.tight_layout()
    path = os.path.join(out_dir, 'distances.png')
    plt.savefig(path, dpi=150); plt.close(); print(f'Saved: {path}')


def save_animation(results, out_dir):
    """Side-by-side animation of pincer vs independent against the card's
    perpendicular evader."""
    import matplotlib.animation as animation
    sel = [r for r in results if r['evader'] == 'perpendicular']
    max_len = max(len(r['e']) for r in sel)
    step = max(1, int(np.ceil(max_len / 140)))
    fig, axes = plt.subplots(1, 2, figsize=(13, 6))
    store = []
    for ax, r in zip(axes, sel):
        pts = np.vstack([r['p1'], r['p2'], r['e']])
        lo = pts.min(axis=0)[:2] - 1.0; hi = pts.max(axis=0)[:2] + 1.0
        span = max(hi - lo) / 2; c0 = (hi + lo) / 2
        ax.set_xlim(c0[0] - span, c0[0] + span); ax.set_ylim(c0[1] - span, c0[1] + span)
        ax.set_aspect('equal'); ax.grid(True, alpha=0.3)
        ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
        lines = [ax.plot([], [], color=c, linewidth=1.6)[0]
                 for c in (P_COLORS[0], P_COLORS[1], 'royalblue')]
        dots = [ax.plot([], [], mk, color=c, markersize=9, zorder=6)[0]
                for mk, c in (('^', P_COLORS[0]), ('^', P_COLORS[1]), ('s', 'blue'))]
        ti = ax.set_title('', fontsize=9)
        store.append((r, lines, dots, ti))
    n_frames = (max_len - 1) // step + 1

    def update(i):
        arts = []
        for r, lines, dots, ti in store:
            si = min(i * step, len(r['e']) - 1)
            for l, d, tr in zip(lines, dots, (r['p1'], r['p2'], r['e'])):
                l.set_data(tr[:si+1, 0], tr[:si+1, 1])
                d.set_data([float(tr[si, 0])], [float(tr[si, 1])])
            done = si >= len(r['e']) - 1
            if done:
                st = f"Captured {r['cap_time']:.2f}s" if r['captured'] else 'Timeout'
            else:
                st = f"angle={r['angle'][si]:.0f}°"
            ti.set_text(f"{_key(r)}\nt={r['times'][si]:.2f}s  {st}")
            arts += lines + dots + [ti]
        return arts

    ani = animation.FuncAnimation(fig, update, frames=n_frames, interval=60, blit=False)
    fig.suptitle('S013 Pincer Movement (card perpendicular evader)', fontsize=10)
    plt.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer='pillow', fps=16, dpi=80); plt.close()
    print(f'Saved: {path}  ({n_frames} frames, every {step} steps)')


if __name__ == '__main__':
    print(f'Pincer offset reaches R_min at t = (R0-R_min)/V_shrink = '
          f'{(R0 - R_MIN) / V_SHRINK:.2f} s (then terminal direct closing)')
    results = []
    for ev_mode in EVADERS:
        for strat in STRATEGIES:
            r = run_simulation(strat, ev_mode)
            status = (f"captured by P{r['cap_who']+1} @ {r['cap_time']:.2f}s"
                      if r['captured'] else 'timeout')
            t170 = f"{r['t170']:.2f}s" if r['t170'] is not None else 'never'
            print(f"[{strat:<11} vs {ev_mode:<13} evader]  {status:<26} "
                  f"initial angle {r['angle'][0]:.1f} deg, max {r['angle'].max():.1f} deg, "
                  f"first >=170 deg at {t170}, angle at end {r['angle'][-1]:.1f} deg, "
                  f"evader travelled {np.sum(np.linalg.norm(np.diff(r['e'], axis=0), axis=1)):.2f} m")
            if strat == 'pincer':
                t_term = (R0 - R_MIN) / V_SHRINK
                pre = r['times'] < t_term
                dmin = np.minimum(r['dist'][0], r['dist'][1])[pre]
                k = int(np.argmin(dmin))
                k_hold = (r['times'] >= r['t170']) & pre
                print(f"    before terminal phase: closest pursuer approach {dmin[k]:.2f} m "
                      f"at t={r['times'][pre][k]:.2f}s; pincer angle after first >=170 deg: "
                      f"{r['angle'][k_hold].min():.1f}-{r['angle'][k_hold].max():.1f} deg")
            results.append(r)

    print('\nPincer shrink-rate sensitivity (capture time; terminal phase starts at '
          '(R0-R_min)/v_shrink):')
    for vs in SWEEP_SHRINK:
        row = []
        for ev_mode in EVADERS:
            r = run_simulation('pincer', ev_mode, v_shrink=vs)
            row.append(f"{ev_mode}: " + (f"{r['cap_time']:.2f}s" if r['captured'] else 'timeout')
                       + f" (max angle {r['angle'].max():.1f} deg)")
        print(f'  v_shrink={vs:.1f} m/s  terminal at {(R0 - R_MIN) / vs:.2f}s  ' + '  '.join(row))

    out_dir = os.path.normpath(OUTPUT_DIR)
    plot_trajectories_2d(results, out_dir)
    plot_pincer_angle(results, out_dir)
    plot_distances(results, out_dir)
    save_animation(results, out_dir)
