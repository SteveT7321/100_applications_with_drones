"""
S011 Swarm Encirclement
========================
3 pursuer drones simultaneously encircle 1 evader, synchronously shrinking
the encirclement radius.  The evader commits to the widest gap between
adjacent pursuers and tries to break out through it.

Strategy (scenario card):
  Pursuer i targets  p_E + R(t)·[cos(2πi/N + ψ_E), sin(2πi/N + ψ_E), 0]
  R(t) = max(R0 − (R0 − R_min)·t / T_conv, R_min),  R_min = r_capture = 0.3 m
  ψ_E = evader heading (direction of its last velocity), so pursuer 0 is
  always placed directly ahead of the evader ("heading-aligned ring").

  For comparison a "fixed-orientation ring" (ψ_E ≡ 0) is also simulated.

Evader:
  Steers toward the mid-angle of the widest angular gap between adjacent
  pursuers, but COMMITS to that gap (identified by its pair of bounding
  pursuers) and only switches when another gap is wider by GAP_HYST.

Breakout:
  The evader is outside the pursuers' convex hull iff the largest angular gap
  seen from the evader exceeds 180°.  The script logs the max gap and whether
  this ever happens, and sweeps the evader speed against the card's
  theoretical breakout threshold  v_E > v_P·sin(π/N).

Usage:
    conda activate drones
    python src/01_pursuit_evasion/s011_swarm_encirclement.py
"""

import sys, os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from src.base.drone_base import DroneBase

# ── Parameters ───────────────────────────────────────────────
N_PURSUERS  = 3
R_INIT      = 4.0    # m   initial encirclement radius R0
T_CONVERGE  = 15.0   # s   time for R to reach R_MIN
R_MIN       = 0.3    # m   ring floor (= r_capture, card)
CAPTURE_R   = 0.3    # m   capture distance (card r_capture)
V_PURSUER   = 4.0    # m/s
V_EVADER    = 3.0    # m/s (main run)
GAP_HYST    = np.radians(20.0)  # evader switches gap only if another is wider by this
DT          = 1 / 48
MAX_TIME    = 20.0
SWEEP_VE    = [2.0, 2.5, 3.0, 3.5, 4.0, 4.5]   # m/s evader-speed sweep

INIT_EVADER = np.array([0.0, 0.0, 2.0])

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '01_pursuit_evasion', 's011_swarm_encirclement',
)


def make_pursuer_inits(n=N_PURSUERS, r=R_INIT, z=2.0):
    inits = []
    for i in range(n):
        a = 2 * np.pi * i / n
        inits.append(INIT_EVADER + np.array([r * np.cos(a), r * np.sin(a), z - INIT_EVADER[2]]))
    return inits


def encirclement_radius(t, r0=R_INIT, r_min=R_MIN, t_conv=T_CONVERGE):
    return max(r0 - (r0 - r_min) * t / t_conv, r_min)


def encirclement_targets(pos_e, t, psi_e, n=N_PURSUERS):
    R = encirclement_radius(t)
    targets = []
    for i in range(n):
        a = 2 * np.pi * i / n + psi_e
        targets.append(pos_e + np.array([R * np.cos(a), R * np.sin(a), 0.0]))
    return targets


def angular_gaps(pursuer_positions, pos_e):
    """Return list of (gap, start_angle, idx_a, idx_b) for consecutive pursuers
    sorted counter-clockwise around the evader."""
    ang = [(np.arctan2(*(p[:2] - pos_e[:2])[::-1]), i)
           for i, p in enumerate(pursuer_positions)]
    ang.sort()
    out = []
    for k in range(len(ang)):
        a0, i0 = ang[k]
        a1, i1 = ang[(k + 1) % len(ang)]
        if k == len(ang) - 1:
            a1 += 2 * np.pi
        out.append((a1 - a0, a0, i0, i1))
    return out


def evader_breakout(pursuer_positions, pos_e, committed, v_e):
    """Commit to the widest gap (pair of bounding pursuers) with hysteresis.
    Returns (velocity, committed_pair, max_gap)."""
    gaps = angular_gaps(pursuer_positions, pos_e)
    widest = max(gaps, key=lambda g: g[0])
    current = next((g for g in gaps if (g[2], g[3]) == committed), None)
    if current is None or widest[0] > current[0] + GAP_HYST:
        current = widest
    mid = current[1] + current[0] / 2
    vel = v_e * np.array([np.cos(mid), np.sin(mid), 0.0])
    return vel, (current[2], current[3]), widest[0]


def pursuer_cmd(pos, tgt):
    """Saturated go-to-target: reach the target this step if possible,
    otherwise fly at V_PURSUER straight toward it (no chattering)."""
    d = tgt - pos
    n = np.linalg.norm(d)
    if n < 1e-9:
        return np.zeros(3)
    return d / n * min(V_PURSUER, n / DT)


# ── Simulation ────────────────────────────────────────────────

def run_simulation(v_e=V_EVADER, heading_aligned=True):
    inits    = make_pursuer_inits()
    pursuers = [DroneBase(p.copy(), max_speed=V_PURSUER, dt=DT) for p in inits]
    evader   = DroneBase(INIT_EVADER.copy(), max_speed=v_e, dt=DT)

    p_trajs  = [[p.pos.copy()] for p in pursuers]
    e_traj   = [evader.pos.copy()]
    r_log    = [R_INIT]
    dist_log = [[np.linalg.norm(p.pos - evader.pos)] for p in pursuers]
    gap_log  = [max(g[0] for g in angular_gaps([p.pos for p in pursuers], evader.pos))]
    time_log = [0.0]

    captured = False; cap_time = None; cap_who = None
    committed = None
    n_switch = 0
    psi_e = 0.0
    max_steps = int(round(MAX_TIME / DT))

    for step in range(1, max_steps + 1):
        t = step * DT

        targets = encirclement_targets(evader.pos, t, psi_e if heading_aligned else 0.0)
        for pursuer, tgt in zip(pursuers, targets):
            pursuer.step(pursuer_cmd(pursuer.pos, tgt))

        e_vel, new_pair, _ = evader_breakout([p.pos for p in pursuers], evader.pos,
                                             committed, v_e)
        if committed is not None and new_pair != committed:
            n_switch += 1
        committed = new_pair
        evader.step(e_vel)
        psi_e = np.arctan2(e_vel[1], e_vel[0])

        r_log.append(encirclement_radius(t))
        time_log.append(t)
        for i, p in enumerate(pursuers):
            p_trajs[i].append(p.pos.copy())
            dist_log[i].append(np.linalg.norm(p.pos - evader.pos))
        e_traj.append(evader.pos.copy())
        gap_log.append(max(g[0] for g in angular_gaps([p.pos for p in pursuers], evader.pos)))

        for i, p in enumerate(pursuers):
            if np.linalg.norm(p.pos - evader.pos) <= CAPTURE_R:
                captured = True; cap_time = t; cap_who = i
                break
        if captured:
            break

    e_traj = np.array(e_traj)
    gap_log = np.degrees(np.array(gap_log))
    return dict(
        p_trajs=[np.array(t) for t in p_trajs], e_traj=e_traj,
        r_log=np.array(r_log), dist_log=[np.array(d) for d in dist_log],
        gap_log=gap_log, times=np.array(time_log),
        captured=captured, cap_time=cap_time, cap_who=cap_who,
        n_switch=n_switch,
        path_len=float(np.sum(np.linalg.norm(np.diff(e_traj, axis=0), axis=1))),
        net_disp=float(np.linalg.norm(e_traj[-1] - e_traj[0])),
        breakout=bool(np.any(gap_log > 180.0)),
        breakout_time=(float(np.array(time_log)[np.argmax(gap_log > 180.0)])
                       if np.any(gap_log > 180.0) else None),
    )


def run_sweep():
    rows = []
    for mode in (True, False):
        for v in SWEEP_VE:
            r = run_simulation(v_e=v, heading_aligned=mode)
            rows.append(dict(mode='heading-aligned' if mode else 'fixed', v_e=v,
                             captured=r['captured'], cap_time=r['cap_time'],
                             max_gap=float(r['gap_log'].max()),
                             breakout=r['breakout'], breakout_time=r['breakout_time'],
                             n_switch=r['n_switch'], net_disp=r['net_disp']))
    return rows


# ── Plots ─────────────────────────────────────────────────────

P_COLORS = ['red', 'darkorange', 'firebrick']


def plot_trajectories_2d(res, out_dir, fname, label):
    p_trajs, e_traj = res['p_trajs'], res['e_traj']
    fig, ax = plt.subplots(figsize=(8, 8))
    for i, (traj, color) in enumerate(zip(p_trajs, P_COLORS)):
        ax.plot(traj[:, 0], traj[:, 1], color=color, linewidth=1.4,
                label=f'Pursuer {i+1}')
        ax.scatter(*traj[0, :2], color=color, s=80, zorder=5)
    ax.plot(e_traj[:, 0], e_traj[:, 1], color='royalblue', linewidth=2.0, label='Evader')
    ax.scatter(*e_traj[0, :2], color='blue', s=80, zorder=5)

    t_end = res['times'][-1]
    for t_snap in np.round(np.array([0.0, 1/3, 2/3]) * t_end, 1):
        R = encirclement_radius(t_snap)
        theta = np.linspace(0, 2*np.pi, 100)
        idx = min(int(round(t_snap / DT)), len(e_traj)-1)
        cx, cy = e_traj[idx, 0], e_traj[idx, 1]
        ax.plot(cx + R*np.cos(theta), cy + R*np.sin(theta),
                color='grey', linestyle='--', linewidth=0.8, alpha=0.6)
        ax.text(cx + R + 0.1, cy, f't={t_snap:.1f}s R={R:.1f}m', fontsize=7, color='grey')

    if res['captured']:
        ax.scatter(*p_trajs[res['cap_who']][-1, :2], color='black', s=140, marker='X',
                   zorder=7, label=f"Captured by P{res['cap_who']+1} @ {res['cap_time']:.2f}s")
    ax.set_aspect('equal'); ax.grid(True, alpha=0.3)
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
    out = f"Captured @ {res['cap_time']:.2f}s" if res['captured'] else 'Timeout'
    ax.set_title(f'S011 Swarm Encirclement — {label}, V_E={V_EVADER} m/s\n({out}, '
                 f"max gap {res['gap_log'].max():.0f}°)", fontsize=10)
    ax.legend(fontsize=8, loc='best')
    plt.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, fname)
    plt.savefig(path, dpi=150); plt.close(); print(f'Saved: {path}')


def plot_radius_and_distances(res_list, labels, out_dir):
    fig, axes = plt.subplots(3, len(res_list), figsize=(13, 9), sharey='row')
    for col, (res, lab) in enumerate(zip(res_list, labels)):
        ax1, ax2, ax3 = axes[:, col]
        times = res['times']; cap_time = res['cap_time']
        ax1.plot(times, res['r_log'], color='steelblue', linewidth=2.0, label='Encirclement R(t)')
        ax1.axhline(CAPTURE_R, color='red', linestyle='--', linewidth=1,
                    label=f'Capture radius {CAPTURE_R} m')
        ax2.axhline(CAPTURE_R, color='red', linestyle='--', linewidth=1)
        for ax in (ax1, ax2, ax3):
            if cap_time:
                ax.axvline(cap_time, color='black', linestyle='--', linewidth=1.2,
                           label=f'Captured {cap_time:.2f}s' if ax is ax1 else None)
            ax.grid(True, alpha=0.3)
        ax1.set_title(lab, fontsize=10)
        ax1.legend(fontsize=8)
        for i, (dist, color) in enumerate(zip(res['dist_log'], P_COLORS)):
            ax2.plot(times, dist, color=color, linewidth=1.5, label=f'Pursuer {i+1}')
        ax2.legend(fontsize=8)
        ax3.plot(times, res['gap_log'], color='royalblue', linewidth=1.5,
                 label='Largest angular gap seen by evader')
        ax3.axhline(180, color='grey', linestyle='--', linewidth=1,
                    label='180 deg = evader outside pursuer triangle')
        ax3.set_ylim(0, 360)
        ax3.set_xlabel('Time (s)')
        ax3.legend(fontsize=8)
    axes[0, 0].set_ylabel('Radius (m)')
    axes[1, 0].set_ylabel('Distance to Evader (m)')
    axes[2, 0].set_ylabel('Max gap (deg)')
    fig.suptitle(f'S011 Swarm Encirclement — Radius, Distances, Max Gap (V_E={V_EVADER} m/s)')
    plt.tight_layout()
    path = os.path.join(out_dir, 'radius_and_distances.png')
    plt.savefig(path, dpi=150); plt.close(); print(f'Saved: {path}')


def plot_speed_sweep(rows, out_dir):
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4.8))
    thresh = V_PURSUER * np.sin(np.pi / N_PURSUERS)
    for mode, color, mk in (('heading-aligned', 'red', 'o'), ('fixed', 'dimgray', 's')):
        rr = [r for r in rows if r['mode'] == mode]
        v = [r['v_e'] for r in rr]
        ax1.plot(v, [r['max_gap'] for r in rr], marker=mk, color=color, label=mode)
        ct = [r['cap_time'] if r['captured'] else np.nan for r in rr]
        ax2.plot(v, ct, marker=mk, color=color, label=mode)
        for r in rr:
            if not r['captured']:
                ax2.scatter(r['v_e'], MAX_TIME, marker='x', color=color, s=80, zorder=5)
    for ax in (ax1, ax2):
        ax.axvline(thresh, color='green', linestyle=':', linewidth=1.2,
                   label=f'card threshold v_P·sin(π/N)={thresh:.2f}')
        ax.set_xlabel('Evader speed V_E (m/s)'); ax.grid(True, alpha=0.3)
    ax1.axhline(180, color='grey', linestyle='--', linewidth=1, label='180° (hull broken)')
    ax1.set_ylabel('Max angular gap over run (deg)')
    ax1.set_title('Largest gap the evader ever saw')
    ax2.set_ylabel('Capture time (s)  [x = timeout at 20 s]')
    ax2.set_ylim(0, MAX_TIME + 1)
    ax2.set_title('Capture time vs evader speed')
    ax1.legend(fontsize=8); ax2.legend(fontsize=8)
    plt.tight_layout()
    path = os.path.join(out_dir, 'speed_sweep.png')
    plt.savefig(path, dpi=150); plt.close(); print(f'Saved: {path}')


def save_animation(res_list, labels, out_dir):
    """Side-by-side top-down animation of the two ring modes (same V_E)."""
    import matplotlib.animation as animation
    step = 5
    n_steps = max(len(r['e_traj']) for r in res_list)
    n_frames = (n_steps - 1) // step + 1
    fig, axes = plt.subplots(1, len(res_list), figsize=(12, 6.2))
    artists = []
    circ_th = np.linspace(0, 2*np.pi, 80)
    for ax, res, lab in zip(axes, res_list, labels):
        allxy = np.vstack([res['e_traj'][:, :2]] + [p[:, :2] for p in res['p_trajs']])
        lo = allxy.min(axis=0) - 1.0; hi = allxy.max(axis=0) + 1.0
        span = max(hi - lo) / 2; c = (hi + lo) / 2
        ax.set_xlim(c[0] - span, c[0] + span); ax.set_ylim(c[1] - span, c[1] + span)
        ax.set_aspect('equal'); ax.grid(True, alpha=0.3)
        ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
        lines_p = [ax.plot([], [], color=cc, linewidth=1.3, alpha=0.8)[0] for cc in P_COLORS]
        dots_p  = [ax.plot([], [], '^', color=cc, markersize=9, zorder=6)[0] for cc in P_COLORS]
        line_e, = ax.plot([], [], color='royalblue', linewidth=1.6, alpha=0.8)
        dot_e,  = ax.plot([], [], 'bs', markersize=9, zorder=6)
        circ,   = ax.plot([], [], 'k--', linewidth=1.0, alpha=0.4)
        ttl = ax.set_title('', fontsize=9)
        artists.append((res, lab, lines_p, dots_p, line_e, dot_e, circ, ttl))

    def update(i):
        out = []
        for res, lab, lines_p, dots_p, line_e, dot_e, circ, ttl in artists:
            e_traj, p_trajs, times = res['e_traj'], res['p_trajs'], res['times']
            si = min(i * step, len(e_traj) - 1)
            t = times[si]; R = encirclement_radius(t)
            ex, ey = e_traj[si, 0], e_traj[si, 1]
            circ.set_data(ex + R*np.cos(circ_th), ey + R*np.sin(circ_th))
            line_e.set_data(e_traj[:si+1, 0], e_traj[:si+1, 1])
            dot_e.set_data([float(ex)], [float(ey)])
            for j, (lp, dp) in enumerate(zip(lines_p, dots_p)):
                lp.set_data(p_trajs[j][:si+1, 0], p_trajs[j][:si+1, 1])
                dp.set_data([float(p_trajs[j][si, 0])], [float(p_trajs[j][si, 1])])
            done = si >= len(e_traj) - 1
            if done:
                st = f"Captured {res['cap_time']:.2f}s" if res['captured'] else 'Timeout'
            else:
                st = f'R={R:.2f}m'
            ttl.set_text(f'{lab}\nt={t:.2f}s  {st}')
            out += lines_p + dots_p + [line_e, dot_e, circ, ttl]
        return out

    fig.suptitle(f'S011 Swarm Encirclement  (V_E={V_EVADER} m/s, V_P={V_PURSUER} m/s)')
    plt.tight_layout()
    ani = animation.FuncAnimation(fig, update, frames=n_frames, interval=60, blit=False)
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer='pillow', fps=16, dpi=80); plt.close()
    print(f'Saved: {path}  ({n_frames} frames)')


def summarize(name, r):
    out = f"captured by P{r['cap_who']+1} @ {r['cap_time']:.2f}s" if r['captured'] else 'timeout'
    bt = f"{r['breakout_time']:.2f}s" if r['breakout'] else 'never'
    print(f'[{name}] {out}; max gap {r["gap_log"].max():.1f} deg; '
          f'gap >180 deg (hull broken): {bt}; gap switches {r["n_switch"]}; '
          f'evader path {r["path_len"]:.2f} m, net displacement {r["net_disp"]:.2f} m')
    for tq in (0.0, 1.0, 2.0, 5.0, 10.0, 14.0):
        if tq > r['times'][-1]:
            continue
        k = int(round(tq / DT))
        print(f'    t={tq:5.2f}s  R={r["r_log"][k]:.2f} m  max gap={r["gap_log"][k]:.1f} deg')
    print(f'    final  t={r["times"][-1]:.2f}s  R={r["r_log"][-1]:.2f} m  '
          f'max gap={r["gap_log"][-1]:.1f} deg')
    if r['captured']:
        e = r['e_traj']; hd = e[-1, :2] - e[-2, :2]
        rel = r['p_trajs'][r['cap_who']][-1, :2] - e[-1, :2]
        brg = np.degrees(np.arctan2(hd[0]*rel[1] - hd[1]*rel[0], hd @ rel))
        print(f'    capturing pursuer bearing relative to evader heading: {brg:.0f} deg '
              f'(0 = dead ahead, +-180 = behind)')


if __name__ == '__main__':
    out_dir = os.path.normpath(OUTPUT_DIR)
    print(f'Card breakout threshold v_P*sin(pi/N) = {V_PURSUER*np.sin(np.pi/N_PURSUERS):.3f} m/s')
    res_h = run_simulation(V_EVADER, heading_aligned=True)
    res_f = run_simulation(V_EVADER, heading_aligned=False)
    summarize('heading-aligned ring (card, psi_E)', res_h)
    summarize('fixed-orientation ring (psi_E=0)', res_f)

    rows = run_sweep()
    print('\nEvader-speed sweep:')
    print(f'{"mode":16s} {"V_E":>5s} {"outcome":>16s} {"max gap":>8s} {"hull broken":>12s} '
          f'{"switches":>8s} {"net disp":>8s}')
    for r in rows:
        oc = f"capture {r['cap_time']:.2f}s" if r['captured'] else 'timeout'
        bt = f"{r['breakout_time']:.2f}s" if r['breakout'] else 'never'
        print(f'{r["mode"]:16s} {r["v_e"]:5.1f} {oc:>16s} {r["max_gap"]:7.1f}° {bt:>12s} '
              f'{r["n_switch"]:8d} {r["net_disp"]:7.2f}m')

    plot_trajectories_2d(res_h, out_dir, 'trajectories_2d.png', 'heading-aligned ring')
    plot_trajectories_2d(res_f, out_dir, 'trajectories_2d_fixed.png', 'fixed-orientation ring')
    plot_radius_and_distances([res_h, res_f], ['heading-aligned ring (card, psi_E)', 'fixed-orientation ring (psi_E=0)'], out_dir)
    plot_speed_sweep(rows, out_dir)
    save_animation([res_h, res_f], ['heading-aligned ring (card)', 'fixed-orientation ring'], out_dir)
