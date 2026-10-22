"""
S013 Pincer Movement
=====================
Two pursuers approach the evader from opposite sides (pincer slots at
phi = 0 deg and 180 deg around the evader, card) with a shrinking offset
R(t) = max(R0 - V_SHRINK*t, R_MIN).  Once R reaches R_MIN the two pursuers
close the pincer by flying straight at the evader (terminal phase).

Evaders (all recompute their heading from the pursuers' current positions
every step):
  perpendicular (card): v_E = V_E * Rot(+-90) * (p_E - p_bar)/|p_E - p_bar|,
      the +-90 side chosen closest to the previous heading (card: +90 only).
      When the evader sits inside the pincer (centroid closer than
      INSIDE_FRAC * half the pursuer spacing) that direction is undefined, so
      it escapes perpendicular to the pursuer axis P1->P2 instead (the open
      side of the pincer), choosing the side closer to its previous heading.
  flee: away from the pursuers, inverse-distance-squared repulsion
      sum (p_E - p_i)/|p_E - p_i|^2; when the two terms nearly cancel
      (inside a pincer) it uses the same axis-perpendicular fallback.
  jink: the flee heading plus a random offset in [-JINK_MAX, +JINK_MAX]
      redrawn every T_JINK s (seeded; same sequence for both strategies).

Comparison: coordinated pincer vs two independent pure-pursuit drones,
same speeds, start positions and controller, against the three evaders at
the card's V_E = 3.5 m/s and in an evader-speed sweep up to V_E > V_P.

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
EVADERS    = ['perpendicular', 'flee', 'jink']
SWEEP_VE   = [3.5, 4.5, 5.0, 5.5]     # m/s evader-speed sweep (V_P = 5)
INSIDE_FRAC = 0.1    # evader "inside the pincer" threshold (fraction of half-spacing)
T_JINK     = 1.0     # s   jink evader: heading offset redrawn every T_JINK
JINK_MAX   = np.radians(60.0)
JINK_SEED  = 13

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


def _axis_perp(p1, p2, prev_dir):
    """Unit vector perpendicular to the pursuer axis, on the side closer to
    the previous heading (tie -> +90 deg rotation of P1->P2)."""
    ax = (p2 - p1)[:2]
    n = np.linalg.norm(ax)
    if n < 1e-9:
        return prev_dir
    ax = ax / n
    u = np.array([-ax[1], ax[0]])
    return -u if u @ prev_dir < 0 else u


def evader_velocity(p1, p2, pe, mode, prev_dir, v_e, jink_offset=0.0):
    """Returns (velocity, unit heading, used_fallback)."""
    half_spacing = 0.5 * np.linalg.norm((p2 - p1)[:2])
    fallback = False
    if mode == 'perpendicular':
        d = (pe - 0.5 * (p1 + p2))[:2]
        n = np.linalg.norm(d)
        if n < INSIDE_FRAC * half_spacing or n < 1e-9:
            u = _axis_perp(p1, p2, prev_dir); fallback = True
        else:
            d = d / n
            u = np.array([-d[1], d[0]])          # card: +90 deg rotation ...
            if u @ prev_dir < 0:                 # ... but keep the side closer to
                u = -u                           # the previous heading (no flip-flop)
    else:   # 'flee' or 'jink'
        terms = []
        for p in (p1, p2):
            r = (pe - p)[:2]
            terms.append(r / max(r @ r, 1e-9))
        rep_v = terms[0] + terms[1]
        biggest = max(np.linalg.norm(terms[0]), np.linalg.norm(terms[1]))
        if np.linalg.norm(rep_v) < INSIDE_FRAC * biggest:
            u = _axis_perp(p1, p2, prev_dir); fallback = True
        else:
            u = rep_v / np.linalg.norm(rep_v)
        if mode == 'jink' and jink_offset != 0.0:
            c, s_ = np.cos(jink_offset), np.sin(jink_offset)
            u = np.array([c * u[0] - s_ * u[1], s_ * u[0] + c * u[1]])
    return v_e * np.array([u[0], u[1], 0.0]), u, fallback


# ── Simulation ────────────────────────────────────────────────

def run_simulation(strategy='pincer', evader_mode='perpendicular', v_shrink=V_SHRINK,
                   v_e=V_EVADER, seed=JINK_SEED):
    rng = np.random.default_rng(seed)        # jink offsets (only used by 'jink')
    jink_steps = int(round(T_JINK / DT))
    jink_offset = 0.0
    p1 = DroneBase(INIT_P1.copy(), max_speed=V_PURSUER, dt=DT)
    p2 = DroneBase(INIT_P2.copy(), max_speed=V_PURSUER, dt=DT)
    ev = DroneBase(INIT_E.copy(),  max_speed=v_e,  dt=DT)

    p1_traj = [p1.pos.copy()]; p2_traj = [p2.pos.copy()]
    e_traj  = [ev.pos.copy()]
    angle_log = [pincer_angle(p1.pos, p2.pos, ev.pos)]
    dist_log  = [[np.linalg.norm(p1.pos - ev.pos)], [np.linalg.norm(p2.pos - ev.pos)]]
    time_log  = [0.0]
    prev_dir  = np.array([1.0, 0.0])
    n_fallback = 0

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

        if evader_mode == 'jink' and (step - 1) % jink_steps == 0:
            jink_offset = rng.uniform(-JINK_MAX, JINK_MAX)
        e_vel, prev_dir, fb = evader_velocity(p1.pos, p2.pos, ev.pos, evader_mode,
                                              prev_dir, v_e, jink_offset)
        n_fallback += int(fb)
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
        n_fallback=n_fallback, n_steps=len(time_log) - 1, v_e=v_e,
        t170=float(time_log[above[0]]) if len(above) else None,
    )


# ── Plots ─────────────────────────────────────────────────────

def _key(r):
    return f"{r['strategy']} vs {r['evader']} evader"


def plot_trajectories_2d(results, out_dir):
    fig, axes = plt.subplots(3, 2, figsize=(13, 16))
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
              ('independent', 'flee'): ('darkorange', '--'),
              ('pincer', 'jink'): ('steelblue', ':'),
              ('independent', 'jink'): ('darkorange', ':')}
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
    fig, axes = plt.subplots(3, 2, figsize=(13, 11), sharey=True)
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


def plot_speed_sweep(rows, out_dir):
    fig, axes = plt.subplots(1, len(EVADERS), figsize=(15, 4.6), sharey=True)
    for ax, ev_mode in zip(axes, EVADERS):
        for strat, c, mk in (('pincer', 'steelblue', 'o'), ('independent', 'darkorange', 's')):
            rr = [r for r in rows if r['evader'] == ev_mode and r['strategy'] == strat]
            v = [r['v_e'] for r in rr]
            ct = [r['cap_time'] if r['captured'] else np.nan for r in rr]
            ax.plot(v, ct, marker=mk, color=c, label=strat)
            for r in rr:
                if not r['captured']:
                    ax.scatter(r['v_e'], MAX_TIME, marker='x', color=c, s=90, zorder=5)
        ax.axvline(V_PURSUER, color='grey', linestyle=':', label=f'V_P = {V_PURSUER} m/s')
        ax.set_title(f'{ev_mode} evader', fontsize=10)
        ax.set_xlabel('Evader speed V_E (m/s)'); ax.grid(True, alpha=0.3)
        ax.set_ylim(0, MAX_TIME + 1)
    axes[0].set_ylabel(f'Capture time (s)   [x = no capture by {MAX_TIME:.0f} s]')
    axes[0].legend(fontsize=8)
    fig.suptitle('S013 Pincer Movement — capture time vs evader speed', fontsize=11)
    plt.tight_layout()
    path = os.path.join(out_dir, 'speed_sweep.png')
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


def _status(r):
    return (f"captured by P{r['cap_who']+1} @ {r['cap_time']:.2f}s" if r['captured']
            else f'no capture by {MAX_TIME:.0f}s')


if __name__ == '__main__':
    print(f'Pincer offset reaches R_min at t = (R0-R_min)/V_shrink = '
          f'{(R0 - R_MIN) / V_SHRINK:.2f} s (then terminal direct closing)')
    print(f'Main runs: V_E = {V_EVADER} m/s, V_P = {V_PURSUER} m/s')
    results = []
    for ev_mode in EVADERS:
        for strat in STRATEGIES:
            r = run_simulation(strat, ev_mode)
            t170 = f"{r['t170']:.2f}s" if r['t170'] is not None else 'never'
            print(f"[{strat:<11} vs {ev_mode:<13} evader]  {_status(r):<26} "
                  f"initial angle {r['angle'][0]:.1f} deg, max {r['angle'].max():.1f} deg, "
                  f"first >=170 deg at {t170}, "
                  f"evader travelled {np.sum(np.linalg.norm(np.diff(r['e'], axis=0), axis=1)):.2f} m, "
                  f"net {np.linalg.norm(r['e'][-1] - r['e'][0]):.2f} m, "
                  f"inside-pincer fallback {r['n_fallback']}/{r['n_steps']} steps")
            if strat == 'pincer' and r['t170'] is not None:
                t_term = (R0 - R_MIN) / V_SHRINK
                pre = r['times'] < t_term
                k_hold = (r['times'] >= r['t170']) & pre
                if k_hold.any():
                    print(f"    pincer angle between first >=170 deg and terminal phase: "
                          f"{r['angle'][k_hold].min():.1f}-{r['angle'][k_hold].max():.1f} deg")
            results.append(r)

    print('\nPincer shrink-rate sensitivity at V_E = 3.5 m/s (terminal phase at '
          '(R0-R_min)/v_shrink):')
    for vs in SWEEP_SHRINK:
        row = []
        for ev_mode in EVADERS:
            r = run_simulation('pincer', ev_mode, v_shrink=vs)
            row.append(f"{ev_mode}: " + (f"{r['cap_time']:.2f}s" if r['captured'] else 'none'))
        print(f'  v_shrink={vs:.1f} m/s  terminal at {(R0 - R_MIN) / vs:.2f}s  ' + '  '.join(row))

    print(f'\nEvader-speed sweep (V_P = {V_PURSUER} m/s, card shrink {V_SHRINK} m/s):')
    sweep_rows = []
    for ev_mode in EVADERS:
        for v in SWEEP_VE:
            line = []
            for strat in STRATEGIES:
                r = run_simulation(strat, ev_mode, v_e=v)
                sweep_rows.append(r)
                dmin = float(np.min(np.minimum(r['dist'][0], r['dist'][1])))
                line.append(f"{strat}: " + (f"{r['cap_time']:.2f}s" if r['captured'] else
                                            f"none (closest {dmin:.2f} m)")
                            + f" [net {np.linalg.norm(r['e'][-1] - r['e'][0]):.1f} m, "
                            f"fallback {r['n_fallback']}/{r['n_steps']}]")
            print(f'  {ev_mode:<13} V_E={v:.1f}  ' + '   '.join(line))

    out_dir = os.path.normpath(OUTPUT_DIR)
    plot_trajectories_2d(results, out_dir)
    plot_pincer_angle(results, out_dir)
    plot_distances(results, out_dir)
    plot_speed_sweep(sweep_rows, out_dir)
    save_animation(results, out_dir)
