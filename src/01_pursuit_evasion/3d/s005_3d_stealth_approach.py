"""
S005 3D Upgrade — Stealth Approach
===================================
The evader patrols +x at 3 m/s with a 3D forward detection cone (half-angle
60 deg, range 5 m) along its velocity. The first detection alerts it (latched):
it then flees directly away from the pursuer in 3D at 3 m/s. The pursuer flies
5.5 m/s while undetected and the evader is not alerted, 4.5 m/s otherwise
(same rule for every strategy).

As in the fixed 2D S005, the pursuer starts AHEAD of the evader at (6, 2, 2),
outside range but near the cone axis, so a straight approach must cross the
cone. The card's start (-4, 2, 2) is already in the rear blind spot and is run
as a check.

Strategies:
  direct        — always fly straight at the evader
  horiz_flank   — 2D stealth: aim 2 m behind the evader, headings searched in
                  the horizontal plane only (z stays at 2 m)
  optimal_3d    — same aim, but headings searched over the full 3D sphere
  high_approach — 3D search, aim 2 m behind and 2.5 m above the evader
  low_approach  — 3D search, aim 2 m behind and 1.5 m below (z >= 0.3 m)
Every stealth strategy aims at the evader itself once it is within
|offset| + 0.5 m, and falls back to direct pursuit once the evader is alerted.

A Monte Carlo over 40 co-altitude pursuer starts ahead of the evader (same
starts for every strategy) checks that the ranking does not hinge on one start.

Usage:
    conda activate drones
    python src/01_pursuit_evasion/3d/s005_3d_stealth_approach.py
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
    'outputs', '01_pursuit_evasion', '3d', 's005_3d_stealth_approach'))

# ── Parameters ──────────────────────────────────────────────
CONE_HALF_ANGLE_DEG = 60.0
DETECT_RANGE        = 5.0       # m
PURSUER_SPEED_CLEAR = 5.5       # m/s, undetected and evader not alerted
PURSUER_SPEED_DET   = 4.5       # m/s, detected / evader alerted
EVADER_SPEED        = 3.0       # m/s, patrol and alerted
CAPTURE_R           = 0.15      # m
DT                  = 1 / 48
MAX_TIME            = 20.0
Z_MIN               = 0.3
Z_MAX               = 10.0
REAR_OFFSET         = 2.0       # m behind the evader
H_CLIMB             = 2.5       # m above the evader (high approach)
H_DIVE              = -1.5      # m below the evader (low approach)
HEADING_STEP_DEG    = 5.0       # heading search resolution (azimuth and elevation)

INIT_PURSUER      = np.array([6.0, 2.0, 2.0])
CARD_INIT_PURSUER = np.array([-4.0, 2.0, 2.0])
INIT_EVADER       = np.array([0.0, 0.0, 2.0])
EVADER_HEADING    = np.array([1.0, 0.0, 0.0])

N_MC    = 40
MC_SEED = 5

STRATEGIES = ['direct', 'horiz_flank', 'optimal_3d', 'high_approach', 'low_approach']
STRAT_LABELS = {'direct': 'Direct', 'horiz_flank': 'Horiz flank (2D)',
                'optimal_3d': 'Optimal 3D (3D search)', 'high_approach': 'High approach',
                'low_approach': 'Low approach'}
STRAT_COLORS = {'direct': 'gray', 'horiz_flank': 'steelblue', 'optimal_3d': 'mediumpurple',
                'high_approach': 'darkorange', 'low_approach': 'seagreen'}

COS_CONE = np.cos(np.radians(CONE_HALF_ANGLE_DEG))


def _heading_sets():
    az = np.radians(np.arange(0.0, 360.0, HEADING_STEP_DEG))
    horiz = np.column_stack([np.cos(az), np.sin(az), np.zeros_like(az)])
    dirs = [np.array([[0.0, 0.0, 1.0], [0.0, 0.0, -1.0]])]
    for el in np.radians(np.arange(-90 + HEADING_STEP_DEG, 90, HEADING_STEP_DEG)):
        dirs.append(np.column_stack([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az),
                                     np.full_like(az, np.sin(el))]))
    return horiz, np.vstack(dirs)


HEADINGS_2D, HEADINGS_3D = _heading_sets()


# ── Helpers ──────────────────────────────────────────────────

def cone_angle(p_pursuer, p_evader, v_hat):
    d = p_pursuer - p_evader
    dist = np.linalg.norm(d)
    cos_a = np.dot(d, v_hat) / (dist + 1e-12)
    return dist, cos_a


def is_detected(p_pursuer, p_evader, v_hat):
    dist, cos_a = cone_angle(p_pursuer, p_evader, v_hat)
    return dist <= DETECT_RANGE and cos_a > COS_CONE


def stealth_heading(p, e, v_hat, desired, speed, dirs):
    """Heading closest to `desired` whose predicted next position (evader
    advanced one step on its patrol) is outside the cone and inside the
    altitude bounds. Falls back to `desired` if none is safe."""
    e_next = e + v_hat * EVADER_SPEED * DT
    cand = p + dirs * speed * DT
    d = cand - e_next
    dist = np.linalg.norm(d, axis=1)
    det = (dist <= DETECT_RANGE) & (d @ v_hat > COS_CONE * dist)
    ok = ~det & (cand[:, 2] >= Z_MIN) & (cand[:, 2] <= Z_MAX)
    if not ok.any():
        return desired
    score = np.where(ok, dirs @ desired, -np.inf)
    return dirs[int(np.argmax(score))]


def aim_offset(strategy):
    up = {'high_approach': H_CLIMB, 'low_approach': H_DIVE}.get(strategy, 0.0)
    return np.array([-REAR_OFFSET, 0.0, up])   # relative to the evader (heading +x)


# ── Simulation ───────────────────────────────────────────────

def run_simulation(strategy, init_pursuer=INIT_PURSUER):
    p = np.array(init_pursuer, dtype=float)
    e = INIT_EVADER.copy()
    v_hat = EVADER_HEADING.copy()
    alerted, alarm_t = False, None
    captured, cap_t = False, None
    p_traj, e_traj, det_log, alpha_log = [p.copy()], [e.copy()], [], []
    offs = aim_offset(strategy)

    for step in range(int(MAX_TIME / DT)):
        t = step * DT
        dist, cos_a = cone_angle(p, e, v_hat)
        alpha_log.append(np.degrees(np.arccos(np.clip(cos_a, -1, 1))))
        det_now = (not alerted) and is_detected(p, e, v_hat)
        det_log.append(int(det_now))
        if det_now:
            alerted, alarm_t = True, t
        if dist < CAPTURE_R:
            captured, cap_t = True, t
            break

        speed = PURSUER_SPEED_DET if alerted else PURSUER_SPEED_CLEAR
        to_e = e - p
        if strategy == 'direct' or alerted:
            cmd = to_e / (np.linalg.norm(to_e) + 1e-12)
        else:
            if dist > np.linalg.norm(offs) + 0.5:
                target = e + offs
                target[2] = np.clip(target[2], Z_MIN, Z_MAX)
            else:
                target = e
            desired = target - p
            desired /= np.linalg.norm(desired) + 1e-12
            if strategy == 'horiz_flank':
                desired = np.array([desired[0], desired[1], 0.0])
                desired /= np.linalg.norm(desired) + 1e-12
                dirs = HEADINGS_2D
            else:
                dirs = HEADINGS_3D
            cmd = stealth_heading(p, e, v_hat, desired, speed, dirs)
        p = p + cmd * speed * DT
        p[2] = np.clip(p[2], Z_MIN, Z_MAX)

        # Evader
        if alerted:
            flee = e - p
            v_hat = flee / (np.linalg.norm(flee) + 1e-12)
        e_new = e + v_hat * EVADER_SPEED * DT
        if not (Z_MIN <= e_new[2] <= Z_MAX):           # slide along floor / ceiling
            vh = np.array([v_hat[0], v_hat[1], 0.0])
            vh /= np.linalg.norm(vh) + 1e-12
            e_new = e + vh * EVADER_SPEED * DT
        e = e_new
        p_traj.append(p.copy())
        e_traj.append(e.copy())

    p_traj, e_traj = np.array(p_traj), np.array(e_traj)
    return dict(p=p_traj, e=e_traj, captured=captured, t=cap_t if captured else MAX_TIME,
                alerted=alerted, alarm_t=alarm_t, det=np.array(det_log),
                alpha=np.array(alpha_log), strategy=strategy,
                Lp=float(np.sum(np.linalg.norm(np.diff(p_traj, axis=0), axis=1))),
                min_alpha_in_range=_min_alpha_in_range(p_traj, e_traj, alarm_t))


def _min_alpha_in_range(p_traj, e_traj, alarm_t):
    """Smallest cone angle while within detection range and before any alarm."""
    n = len(p_traj) if alarm_t is None else int(round(alarm_t / DT)) + 1
    d = p_traj[:n] - e_traj[:n]
    dist = np.linalg.norm(d, axis=1)
    inr = dist <= DETECT_RANGE
    if not inr.any():
        return None
    cos_a = d[inr] @ EVADER_HEADING / dist[inr]
    return float(np.degrees(np.arccos(np.clip(cos_a, -1, 1))).min())


def sample_starts(n, seed):
    """Co-altitude starts ahead of the evader: range U[5.5, 7] m, bearing from
    the evader's heading U[-45, 45] deg (inside the cone's angular sector)."""
    rng = np.random.default_rng(seed)
    r = rng.uniform(5.5, 7.0, n)
    b = np.radians(rng.uniform(-45.0, 45.0, n))
    return [INIT_EVADER + np.array([ri * np.cos(bi), ri * np.sin(bi), 0.0])
            for ri, bi in zip(r, b)]


# ── Cone drawing ─────────────────────────────────────────────

def cone_wireframe(apex, axis, half_deg, length=DETECT_RANGE, n=24):
    axis = axis / (np.linalg.norm(axis) + 1e-12)
    a = np.array([0.0, 0.0, 1.0]) if abs(axis[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    u = np.cross(axis, a); u /= np.linalg.norm(u)
    w = np.cross(axis, u)
    th = np.linspace(0, 2 * np.pi, n)
    # slant length = range (detection is range-limited, not height-limited)
    h = length * np.cos(np.radians(half_deg))
    r = length * np.sin(np.radians(half_deg))
    rim = apex + h * axis + r * (np.outer(np.cos(th), u) + np.outer(np.sin(th), w))
    X = np.column_stack([np.full(n, apex[0]), rim[:, 0]])
    Y = np.column_stack([np.full(n, apex[1]), rim[:, 1]])
    Z = np.column_stack([np.full(n, apex[2]), rim[:, 2]])
    return X, Y, Z


def _status(r):
    s = f'Captured {r["t"]:.2f} s' if r['captured'] else 'Timeout'
    a = f'alarm at {r["alarm_t"]:.2f} s' if r['alerted'] else 'no alarm'
    return f'{s}, {a}'


# ── Plots ────────────────────────────────────────────────────

def plot_trajectories_3d(results):
    fig = plt.figure(figsize=(20, 11))
    for i, s in enumerate(STRATEGIES):
        r = results[s]
        ax = fig.add_subplot(2, 3, i + 1, projection='3d')
        ax.view_init(elev=25, azim=-60)
        ax.plot(*r['p'].T, color='red', linewidth=1.8, label='Pursuer')
        ax.plot(*r['e'].T, color='blue', linewidth=1.8, label='Evader')
        ax.scatter(*r['p'][0], color='red', s=40)
        ax.scatter(*r['e'][0], color='blue', s=40)
        ax.scatter(*r['p'][-1], color='black', s=90, marker='X')
        X, Y, Z = cone_wireframe(r['e'][0], EVADER_HEADING, CONE_HALF_ANGLE_DEG)
        ax.plot_wireframe(X, Y, Z, color='goldenrod', alpha=0.35, linewidth=0.6)
        if r['alerted']:
            k = int(round(r['alarm_t'] / DT))
            ax.scatter(*r['p'][k], color='orange', s=80, marker='*', label='alarm')
        pts = np.vstack([r['p'], r['e']])
        ax.set_xlim(min(-3, pts[:, 0].min() - 0.5), max(8, pts[:, 0].max() + 0.5))
        ax.set_ylim(min(-4, pts[:, 1].min() - 0.5), max(4, pts[:, 1].max() + 0.5))
        ax.set_zlim(0, 6)
        ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
        ax.set_title(f'{STRAT_LABELS[s]}\n{_status(r)}, path {r["Lp"]:.2f} m', fontsize=10)
        ax.legend(fontsize=7, loc='upper left')
    fig.suptitle('S005 3D Stealth Approach — trajectories from (6, 2, 2)  '
                 '(gold: detection cone at t = 0)', fontsize=13)
    path = os.path.join(OUTPUT_DIR, 'trajectories_3d.png')
    plt.savefig(path, dpi=100, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_detection_status(results):
    fig, axes = plt.subplots(len(STRATEGIES), 1, figsize=(11, 9), sharex=True)
    for ax, s in zip(axes, STRATEGIES):
        r = results[s]
        t = np.arange(len(r['det'])) * DT
        alerted = np.zeros(len(t))
        if r['alerted']:
            alerted[t >= r['alarm_t'] - 1e-9] = 1
        ax.fill_between(t, alerted, step='post', color=STRAT_COLORS[s], alpha=0.35,
                        label='evader alerted (latched)')
        ax.step(t, r['det'], where='post', color='red', label='detected this step')
        if r['captured']:
            ax.axvline(r['t'], color='black', linestyle='--', linewidth=1.0)
        ax.set_ylim(-0.1, 1.3)
        ax.set_ylabel('0 / 1')
        ax.set_title(f'{STRAT_LABELS[s]} — {_status(r)}', fontsize=9)
        ax.legend(fontsize=7, loc='center right')
        ax.grid(True, alpha=0.3)
    axes[-1].set_xlabel('Time (s)')
    fig.suptitle('S005 3D — Detection and alert status (start (6, 2, 2))', fontsize=11)
    plt.tight_layout()
    path = os.path.join(OUTPUT_DIR, 'detection_status.png')
    plt.savefig(path, dpi=110)
    plt.close()
    print(f'Saved: {path}')


def plot_approach_angle(results):
    fig, ax = plt.subplots(figsize=(11, 5))
    for s in STRATEGIES:
        r = results[s]
        n = len(r['alpha'])
        d = np.linalg.norm(r['p'][:n] - r['e'][:n], axis=1)
        a = np.where(d <= DETECT_RANGE, r['alpha'], np.nan)   # only inside range matters
        ax.plot(np.arange(n) * DT, a, color=STRAT_COLORS[s], linewidth=1.6, label=STRAT_LABELS[s])
    ax.axhline(CONE_HALF_ANGLE_DEG, color='goldenrod', linestyle='--',
               label='cone boundary 60°')
    ax.axhspan(0, CONE_HALF_ANGLE_DEG, color='gold', alpha=0.12)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Cone angle α (°), only while within 5 m')
    ax.set_ylim(0, 185)
    ax.set_title('S005 3D — Angle between the evader heading and the pursuer '
                 '(below 60° inside 5 m = detected)')
    ax.legend(fontsize=8, loc='lower right')
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    path = os.path.join(OUTPUT_DIR, 'approach_angle.png')
    plt.savefig(path, dpi=120)
    plt.close()
    print(f'Saved: {path}')


def plot_capture_times(results, mc):
    fig, axes = plt.subplots(1, 2, figsize=(15, 5))
    labels = [STRAT_LABELS[s].replace(' (', '\n(') for s in STRATEGIES]
    ax = axes[0]
    vals = [results[s]['t'] for s in STRATEGIES]
    bars = ax.bar(labels, vals, color=[STRAT_COLORS[s] for s in STRATEGIES], edgecolor='black')
    for b, s in zip(bars, STRATEGIES):
        r = results[s]
        ax.text(b.get_x() + b.get_width() / 2, b.get_height() + 0.05,
                f'{r["t"]:.2f} s\n' + ('alarm' if r['alerted'] else 'no alarm'),
                ha='center', va='bottom', fontsize=9)
    ax.set_ylim(0, max(vals) * 1.35)
    ax.set_ylabel('Capture time (s)')
    ax.set_title('Nominal start (6, 2, 2)')
    ax.grid(True, axis='y', alpha=0.3)
    ax.tick_params(axis='x', labelsize=8)

    ax = axes[1]
    data = [[x['t'] for x in mc[s]] for s in STRATEGIES]
    ax.boxplot(data, showmeans=True)
    ax.set_xticks(range(1, len(STRATEGIES) + 1))
    ax.set_xticklabels([lab + f'\nalarm {sum(x["alerted"] for x in mc[s])}/{N_MC}'
                        for lab, s in zip(labels, STRATEGIES)], fontsize=8)
    ax.set_ylabel('Capture time (s)')
    ax.set_title(f'Monte Carlo: {N_MC} co-altitude starts ahead of the evader (same for all)')
    ax.grid(True, axis='y', alpha=0.3)
    fig.suptitle('S005 3D — Capture time by strategy', fontsize=12)
    plt.tight_layout()
    path = os.path.join(OUTPUT_DIR, 'capture_times.png')
    plt.savefig(path, dpi=120)
    plt.close()
    print(f'Saved: {path}')


def save_animation(results):
    """Horizontal flank vs high approach side by side, cone moving with the evader."""
    runs = [results['horiz_flank'], results['high_approach']]
    step_skip = 2
    n_steps = max(len(r['p']) for r in runs)
    frames = list(range(0, n_steps, step_skip))
    if frames[-1] != n_steps - 1:
        frames.append(n_steps - 1)

    fig = plt.figure(figsize=(13, 5.5))
    panels = []
    for i, r in enumerate(runs):
        ax = fig.add_subplot(1, 2, i + 1, projection='3d')
        ax.view_init(elev=25, azim=-60)
        ax.set_xlim(-3, 8); ax.set_ylim(-4, 4); ax.set_zlim(0, 6)
        ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
        ax.set_title(f'{STRAT_LABELS[r["strategy"]]}: {_status(r)}', fontsize=10)
        pl, = ax.plot([], [], [], 'r-', linewidth=1.6)
        el, = ax.plot([], [], [], 'b-', linewidth=1.6)
        pd, = ax.plot([], [], [], 'r^', markersize=8)
        ed, = ax.plot([], [], [], 'bs', markersize=8)
        panels.append([ax, r, pl, el, pd, ed, None])
    title = fig.suptitle('', fontsize=11)

    def update(i):
        k = frames[i]
        for pan in panels:
            ax, r, pl, el, pd, ed, wire = pan
            j = min(k, len(r['p']) - 1)
            P, E = r['p'][:j + 1], r['e'][:j + 1]
            pl.set_data(P[:, 0], P[:, 1]); pl.set_3d_properties(P[:, 2])
            el.set_data(E[:, 0], E[:, 1]); el.set_3d_properties(E[:, 2])
            pd.set_data([P[-1, 0]], [P[-1, 1]]); pd.set_3d_properties([P[-1, 2]])
            ed.set_data([E[-1, 0]], [E[-1, 1]]); ed.set_3d_properties([E[-1, 2]])
            if wire is not None:
                wire.remove()
            X, Y, Z = cone_wireframe(E[-1], EVADER_HEADING, CONE_HALF_ANGLE_DEG)
            pan[6] = ax.plot_wireframe(X, Y, Z, color='goldenrod', alpha=0.6, linewidth=0.7)
        title.set_text(f'S005 3D — pursuer red, evader blue, gold = detection cone   t = {k * DT:.2f} s')
        return []

    ani = FuncAnimation(fig, update, frames=len(frames), interval=60, blit=False)
    path = os.path.join(OUTPUT_DIR, 'animation.gif')
    ani.save(path, writer=PillowWriter(fps=16), dpi=80)
    plt.close()
    print(f'Saved: {path}')


# ── Main ─────────────────────────────────────────────────────

def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    print('Nominal start (6, 2, 2), evader (0, 0, 2) heading +x:')
    results = {}
    for s in STRATEGIES:
        r = run_simulation(s)
        results[s] = r
        ma = r['min_alpha_in_range']
        print(f'  [{s:>13}] {_status(r)}; in-cone steps {int(r["det"].sum())}; '
              f'pursuer path {r["Lp"]:.2f} m; pursuer z [{r["p"][:, 2].min():.2f}, '
              f'{r["p"][:, 2].max():.2f}] m; min cone angle within range before alarm '
              f'{"n/a" if ma is None else f"{ma:.1f} deg"}; capture point '
              f'({r["p"][-1, 0]:.2f}, {r["p"][-1, 1]:.2f}, {r["p"][-1, 2]:.2f})')
        if r['alerted']:
            k = int(round(r['alarm_t'] / DT))
            print(f'  {"":>15} distance at alarm {np.linalg.norm(r["p"][k] - r["e"][k]):.2f} m; '
                  f'evader z range after alarm [{r["e"][k:, 2].min():.2f}, {r["e"][k:, 2].max():.2f}] m')

    print('\nCard start (-4, 2, 2) (already in the rear blind spot):')
    for s in STRATEGIES:
        r = run_simulation(s, CARD_INIT_PURSUER)
        print(f'  [{s:>13}] {_status(r)}; pursuer path {r["Lp"]:.2f} m')

    print(f'\nMonte Carlo: {N_MC} co-altitude starts, range U[5.5, 7] m, bearing U[-45, 45] deg '
          f'(seed {MC_SEED})')
    starts = sample_starts(N_MC, MC_SEED)
    mc = {s: [run_simulation(s, st) for st in starts] for s in STRATEGIES}
    for s in STRATEGIES:
        rs = mc[s]
        tc = np.array([x['t'] for x in rs])
        ncap = sum(x['captured'] for x in rs)
        nal = sum(x['alerted'] for x in rs)
        print(f'  [{s:>13}] captured {ncap}/{N_MC}; alarm {nal}/{N_MC}; capture time mean '
              f'{tc.mean():.2f} s, median {np.median(tc):.2f} s, max {tc.max():.2f} s; '
              f'pursuer path mean {np.mean([x["Lp"] for x in rs]):.2f} m')
    for s in ['optimal_3d', 'high_approach', 'low_approach']:
        d = np.array([b['t'] - a['t'] for a, b in zip(mc['horiz_flank'], mc[s])])
        print(f'  paired {s} - horiz_flank: mean {d.mean():+.2f} s; faster in '
              f'{(d < -1e-9).sum()}, equal in {(np.abs(d) <= 1e-9).sum()}, slower in {(d > 1e-9).sum()}')

    print('\nGenerating plots...')
    plot_trajectories_3d(results)
    plot_detection_status(results)
    plot_approach_angle(results)
    plot_capture_times(results, mc)
    print('Generating animation...')
    save_animation(results)
    print('S005 3D done.')


if __name__ == '__main__':
    main()
