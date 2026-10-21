"""
S031 Path De-confliction
========================
Eight delivery drones fly straight-line routes across a 500 x 500 m urban airspace. They
launch from four edge hubs (two drones per hub, second wave 3 s later) toward random
delivery points on the opposite side (rng seed 7), so routes cross near the centre at
nearly the same time and genuine conflicts occur.

Strategies compared (plus an unresolved baseline):
  0. none      - no de-confliction (reference: how many conflicts the traffic contains)
  1. speed     - priority-based speed adjustment (lower-priority drone slows while the
                 conflict is predicted, resumes v once it clears)
  2. waypoint  - lateral avoidance waypoint inserted for the lower-priority drone
  3. altitude  - each drone pre-assigned its own altitude layer (dz >= d_sep)

Separation metrics only count airborne drones (launched and not yet arrived), so
co-location on the ground at a hub is never counted as a loss of separation.

Usage:
    conda activate drones
    python src/02_logistics_delivery/s031_path_deconfliction.py
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from itertools import combinations
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

# ── Parameters ─────────────────────────────────────────────────────────────────
N_DRONES    = 8          # number of drones
V_DRONE     = 12.0       # m/s cruise speed (nominal)
V_MIN       = 4.0        # m/s minimum speed for strategy 1
V_STEP      = 0.25       # m/s resolution of the speed scan (strategy 1)
D_SEP       = 5.0        # m minimum separation threshold
D_MARGIN    = 2.0        # m extra clearance above D_SEP
T_HORIZON   = 30.0       # s conflict look-ahead window
DT          = 0.1        # s simulation timestep
T_MAX       = 200.0      # s simulation cap
Z_NOM       = 20.0       # m nominal cruise / hand-over altitude
DZ_LAYER    = D_SEP + D_MARGIN   # m altitude layer spacing (strategy 3) = 7 m >= d_sep
Z_BASE      = Z_NOM      # m lowest layer (strategy 3) -> layers 20, 27, ..., 69 m
ARENA       = 500.0      # m side of delivery area
HUB_INSET   = 20.0       # m distance of hubs / goal line from the arena edge
GOAL_SPREAD = 40.0       # m goals uniform within +/- this along the opposite edge
T_STAGGER   = 3.0        # s launch delay of the second wave at each hub
SEED        = 7

STRATEGIES = ['none', 'speed', 'waypoint', 'altitude']
LABELS = {'none': 'No de-confliction', 'speed': 'Speed adjustment',
          'waypoint': 'Waypoint insertion', 'altitude': 'Altitude layers'}
STRAT_COLORS = {'none': 'tab:gray', 'speed': 'tab:blue',
                'waypoint': 'tab:orange', 'altitude': 'tab:green'}

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '02_logistics_delivery', 's031_path_deconfliction',
)


# ── Traffic ────────────────────────────────────────────────────────────────────

def make_traffic(seed=SEED):
    """Origins (4 edge hubs, 2 drones each), random goals on the opposite edge, launch times."""
    rng = np.random.default_rng(seed)
    lo, hi, mid = HUB_INSET, ARENA - HUB_INSET, ARENA / 2
    hubs = np.array([[lo, mid], [hi, mid], [mid, lo], [mid, hi]])        # W, E, S, N
    tangents = np.array([[0, 1], [0, 1], [1, 0], [1, 0]], dtype=float)
    opposite = [1, 0, 3, 2]
    origins = np.zeros((N_DRONES, 3))
    goals = np.zeros((N_DRONES, 3))
    launch = np.zeros(N_DRONES)
    offsets = rng.uniform(-GOAL_SPREAD, GOAL_SPREAD, N_DRONES)
    for k in range(N_DRONES):
        h = k % 4
        g = opposite[h]
        origins[k] = [*hubs[h], Z_NOM]
        goals[k] = [*(hubs[g] + offsets[k] * tangents[g]), Z_NOM]
        launch[k] = 0.0 if k < 4 else T_STAGGER
    return origins, goals, launch


# ── Geometry helpers ───────────────────────────────────────────────────────────

def cpa(pos_i, vel_i, pos_j, vel_j):
    """Return (t_cpa, d_cpa) for two drones with constant velocities (t clamped >= 0)."""
    dp = pos_i - pos_j
    dv = vel_i - vel_j
    dv2 = dv @ dv
    if dv2 < 1e-12:
        return 0.0, float(np.linalg.norm(dp))
    t = max(-(dp @ dv) / dv2, 0.0)
    return t, float(np.linalg.norm(dp + dv * t))


def detect_conflicts(positions, velocities, active, t_horizon, d_sep):
    """List of (i, j, t_cpa, d_cpa) for airborne pairs with a predicted conflict."""
    idx = np.flatnonzero(active)
    out = []
    for a, b in combinations(idx, 2):
        t_c, d_c = cpa(positions[a], velocities[a], positions[b], velocities[b])
        if d_c < d_sep and 0.0 < t_c <= t_horizon:
            out.append((int(a), int(b), t_c, d_c))
    return out


def resolve_speed(pos_y, unit_y, pos_o, vel_o, v_now):
    """Highest speed in [V_MIN, v_now] (descending scan) whose CPA clears d_sep + margin.

    d_CPA(v') is not monotonic in v' (slowing the drone that passes first can make the
    encounter worse before it gets better), so a scan is used instead of bisection.
    Returns V_MIN if no speed in the range clears the conflict.
    """
    for v in np.arange(v_now, V_MIN - 1e-9, -V_STEP):
        t_c, d_c = cpa(pos_y, v * unit_y, pos_o, vel_o)
        if d_c >= D_SEP + D_MARGIN or t_c > T_HORIZON or t_c <= 0.0:
            return float(v)
    return V_MIN


def avoidance_waypoint(pos_y, vel_y, pos_o, vel_o):
    """Lateral avoidance waypoint for yielding drone y: CPA midpoint + rho * n (horizontal)."""
    t_c, _ = cpa(pos_y, vel_y, pos_o, vel_o)
    py, po = pos_y + vel_y * t_c, pos_o + vel_o * t_c
    mid = 0.5 * (py + po)
    dv = vel_y - vel_o
    n = np.array([-dv[1], dv[0], 0.0])
    if np.linalg.norm(n) < 1e-9:
        n = np.array([1.0, 0.0, 0.0])
    n /= np.linalg.norm(n)
    rel = (py - po) * np.array([1.0, 1.0, 0.0])
    if rel @ n < 0:                       # push y further to the side it already passes on
        n = -n
    wp = mid + (D_SEP + D_MARGIN) * n
    wp[2] = pos_y[2]
    return wp


def assign_altitude_layers(n, z_base, dz):
    """Unique altitude layer per drone: z_k = z_base + k * dz."""
    return z_base + dz * np.arange(n)


# ── Simulation ─────────────────────────────────────────────────────────────────

def run_simulation(strategy):
    """
    Run one strategy ('none' | 'speed' | 'waypoint' | 'altitude').

    Returns a dict with trajectories, per-step airborne masks, speeds, separation
    history, conflict/LoS events and per-drone metrics.
    """
    origins, goals, launch = make_traffic(SEED)
    nominal_len = np.linalg.norm(goals - origins, axis=1)
    nominal_arrival = launch + nominal_len / V_DRONE

    # Waypoint queues (list of 3D points still to be visited)
    if strategy == 'altitude':
        layers = assign_altitude_layers(N_DRONES, Z_BASE, DZ_LAYER)
        queues = []
        for k in range(N_DRONES):
            q = []
            if layers[k] > Z_NOM + 1e-9:
                q.append(np.array([origins[k, 0], origins[k, 1], layers[k]]))
                q.append(np.array([goals[k, 0], goals[k, 1], layers[k]]))
            q.append(goals[k].copy())
            queues.append(q)
    else:
        layers = np.full(N_DRONES, Z_NOM)
        queues = [[goals[k].copy()] for k in range(N_DRONES)]
    has_avoid_wp = np.zeros(N_DRONES, dtype=bool)

    pos = origins.copy()
    arrived = np.zeros(N_DRONES, dtype=bool)
    arrival = np.full(N_DRONES, np.nan)
    path_len = np.zeros(N_DRONES)
    n_wp_inserted = 0

    hist_pos, hist_active, hist_speed, hist_sep, times = [], [], [], [], []
    # An "event" is a distinct drone pair: counted the first time it is flagged, so a
    # conflict that stays predicted over many timesteps (or flickers) counts once.
    flagged_seen, los_seen = set(), set()
    conflict_events, los_events = [], []          # (t, i, j, x, y)
    t = 0.0

    while not arrived.all() and t < T_MAX:
        active = (launch <= t + 1e-9) & ~arrived
        targets = np.array([queues[k][0] if queues[k] else goals[k] for k in range(N_DRONES)])
        d = targets - pos
        dist = np.linalg.norm(d, axis=1)
        unit = np.where(dist[:, None] > 1e-9, d / np.maximum(dist, 1e-9)[:, None], 0.0)
        speed = np.where(active, V_DRONE, 0.0)
        vel = speed[:, None] * unit

        # Conflict detection (all strategies; resolution only for speed / waypoint)
        conflicts = detect_conflicts(pos, vel, active, T_HORIZON, D_SEP)
        flagged = {(i, j) for i, j, _, _ in conflicts}
        for (i, j) in sorted(flagged - flagged_seen):
            m = 0.5 * (pos[i] + pos[j])
            conflict_events.append((t, i, j, m[0], m[1]))
        flagged_seen |= flagged

        if strategy == 'speed':
            # lower index = higher priority -> the larger index yields
            for i, j, _, _ in sorted(conflicts, key=lambda c: (c[1], c[0])):
                y, o = j, i
                v_new = resolve_speed(pos[y], unit[y], pos[o], vel[o], speed[y])
                if v_new < speed[y]:
                    speed[y] = v_new
                    vel[y] = v_new * unit[y]
        elif strategy == 'waypoint':
            for i, j, _, _ in sorted(conflicts, key=lambda c: (c[1], c[0])):
                y, o = j, i
                if has_avoid_wp[y]:
                    continue
                wp = avoidance_waypoint(pos[y], vel[y], pos[o], vel[o])
                queues[y].insert(0, wp)
                has_avoid_wp[y] = True
                n_wp_inserted += 1
                dy = wp - pos[y]
                unit[y] = dy / max(np.linalg.norm(dy), 1e-9)
                vel[y] = speed[y] * unit[y]

        # Separation bookkeeping (airborne pairs only)
        idx = np.flatnonzero(active)
        min_sep = np.nan
        los_now = set()
        for a, b in combinations(idx, 2):
            s = float(np.linalg.norm(pos[a] - pos[b]))
            min_sep = s if np.isnan(min_sep) else min(min_sep, s)
            if s < D_SEP:
                los_now.add((int(a), int(b)))
        for (a, b) in sorted(los_now - los_seen):
            m = 0.5 * (pos[a] + pos[b])
            los_events.append((t, a, b, m[0], m[1]))
        los_seen |= los_now

        times.append(t)
        hist_pos.append(pos.copy())
        hist_active.append(active.copy())
        hist_speed.append(speed.copy())
        hist_sep.append(min_sep)

        # Advance with an exact distance budget (carry-over through waypoints)
        for k in np.flatnonzero(active):
            budget = speed[k] * DT
            used = 0.0
            while budget > 1e-12:
                tgt = queues[k][0]
                seg = float(np.linalg.norm(tgt - pos[k]))
                if seg <= budget:
                    pos[k] = tgt.copy()
                    budget -= seg
                    used += seg
                    path_len[k] += seg
                    queues[k].pop(0)
                    if not queues[k]:
                        arrived[k] = True
                        arrival[k] = t + used / speed[k]
                        break
                    if strategy == 'waypoint' and has_avoid_wp[k] and len(queues[k]) == 1:
                        has_avoid_wp[k] = False      # avoidance waypoint reached
                else:
                    pos[k] += budget * (tgt - pos[k]) / seg
                    path_len[k] += budget
                    used += budget
                    budget = 0.0
        t = round(t + DT, 10)

    # final frame
    times.append(t)
    hist_pos.append(pos.copy())
    hist_active.append(np.zeros(N_DRONES, dtype=bool))
    hist_speed.append(np.zeros(N_DRONES))
    hist_sep.append(np.nan)

    delays = np.round(arrival - nominal_arrival, 6) + 0.0     # strip float noise (-0.0)
    added = np.round(path_len - nominal_len, 6) + 0.0
    return {
        'strategy': strategy, 'origins': origins, 'goals': goals, 'launch': launch,
        'layers': layers, 'times': np.array(times), 'pos': np.array(hist_pos),
        'active': np.array(hist_active), 'speed': np.array(hist_speed),
        'sep': np.array(hist_sep), 'conflict_events': conflict_events,
        'los_events': los_events, 'arrival': arrival, 'delays': delays,
        'added_dist': added, 'nominal_len': nominal_len,
        'n_wp_inserted': n_wp_inserted, 'all_arrived': bool(arrived.all()),
    }


def summarise(res):
    los_pairs = sorted({(i + 1, j + 1) for _, i, j, _, _ in res['los_events']})
    return {
        'strategy': res['strategy'],
        'n_conflicts': len(res['conflict_events']),
        'n_los': len(res['los_events']),
        'los_pairs': los_pairs,
        'min_sep': float(np.nanmin(res['sep'])),
        'added_dist': float(np.sum(res['added_dist'])),
        'total_delay': float(np.sum(res['delays'])),
        'max_delay': float(np.max(res['delays'])),
        'n_wp': res['n_wp_inserted'],
        'min_speed': float(np.min(np.where(res['active'], res['speed'], np.inf))),
    }


# ── Plots ──────────────────────────────────────────────────────────────────────

DRONE_COLORS = plt.cm.tab10(np.linspace(0, 0.9, N_DRONES))


def plot_trajectories_2d(results, out_dir):
    """Top-down flown paths per strategy; red circles = loss-of-separation locations."""
    fig, axes = plt.subplots(1, 4, figsize=(22, 6))
    fig.suptitle('Path De-confliction - Top-Down Flown Paths '
                 '(red circle = loss of separation < d_sep)', fontsize=13)
    for ax, res in zip(axes, results):
        P, A = res['pos'], res['active']
        for k in range(N_DRONES):
            m = A[:, k]
            ax.plot([res['origins'][k, 0], res['goals'][k, 0]],
                    [res['origins'][k, 1], res['goals'][k, 1]],
                    color='lightgrey', lw=0.8, ls='--', zorder=1)
            ax.plot(P[m, k, 0], P[m, k, 1], color=DRONE_COLORS[k], lw=1.4, zorder=2,
                    label=f'D{k+1}')
            ax.plot(res['origins'][k, 0], res['origins'][k, 1], 'ks', ms=6, zorder=3)
            ax.plot(res['goals'][k, 0], res['goals'][k, 1], '*', color=DRONE_COLORS[k],
                    ms=10, zorder=3)
        for (_, i, j, x, y) in res['los_events']:
            ax.add_patch(plt.Circle((x, y), 12, fill=False, color='red', lw=2, zorder=4))
        s = summarise(res)
        ax.set_title(f"{LABELS[res['strategy']]}\nconflicts={s['n_conflicts']}, "
                     f"LoS={s['n_los']}", fontsize=11)
        ax.set_xlim(0, ARENA); ax.set_ylim(0, ARENA); ax.set_aspect('equal')
        ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
    axes[0].legend(loc='upper left', fontsize=7, ncol=2)
    plt.tight_layout()
    path = os.path.join(out_dir, 'trajectories_2d.png')
    plt.savefig(path, dpi=130, bbox_inches='tight'); plt.close()
    print(f'Saved: {path}')


def plot_3d_altitude_layers(results, out_dir):
    """3D flown paths for the altitude-layer strategy."""
    res = results[STRATEGIES.index('altitude')]
    P, A = res['pos'], res['active']
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    for k in range(N_DRONES):
        m = A[:, k]
        ax.plot(P[m, k, 0], P[m, k, 1], P[m, k, 2], color=DRONE_COLORS[k], lw=1.6,
                label=f"D{k+1} layer {res['layers'][k]:.0f} m")
        ax.scatter(*res['origins'][k], c='k', marker='s', s=25)
        ax.scatter(*res['goals'][k], c=[DRONE_COLORS[k]], marker='*', s=80)
    ax.set_xlim(0, ARENA); ax.set_ylim(0, ARENA); ax.set_zlim(0, 75)
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
    ax.set_title(f'Altitude Layers (dz = {DZ_LAYER:.0f} m) - 3D Flown Paths', fontsize=12)
    ax.legend(loc='upper left', fontsize=7, ncol=2)
    plt.tight_layout()
    path = os.path.join(out_dir, 'trajectories_3d_altitude.png')
    plt.savefig(path, dpi=130, bbox_inches='tight'); plt.close()
    print(f'Saved: {path}')


def plot_separation_timeseries(results, out_dir):
    """Minimum airborne pairwise separation vs time."""
    fig, ax = plt.subplots(figsize=(12, 5))
    for res, ls in zip(results, [':', '-', '--', '-.']):
        ax.plot(res['times'], res['sep'], lw=1.5, ls=ls,
                color=STRAT_COLORS[res['strategy']], label=LABELS[res['strategy']])
    ax.axhline(D_SEP, color='red', ls=':', lw=2, label=f'd_sep = {D_SEP} m')
    ax.set_xlabel('Time (s)'); ax.set_ylabel('Min airborne pairwise separation (m)')
    ax.set_title('Minimum Pairwise Separation vs Time (airborne drones only)')
    ax.set_ylim(0, 60); ax.set_xlim(left=0)
    ax.legend(); ax.grid(alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'separation_timeseries.png')
    plt.savefig(path, dpi=130); plt.close()
    print(f'Saved: {path}')


def plot_strategy_comparison(summaries, out_dir):
    """Bars: conflict events, LoS events, added distance, max delay."""
    names = [LABELS[s['strategy']].replace(' ', '\n', 1) for s in summaries]
    cols = [STRAT_COLORS[s['strategy']] for s in summaries]
    panels = [('n_conflicts', 'Predicted conflict events', '{:.0f}'),
              ('n_los', 'Loss-of-separation events (< d_sep)', '{:.0f}'),
              ('added_dist', 'Total added flight distance (m)', '{:.1f}'),
              ('max_delay', 'Max arrival delay (s)', '{:.2f}')]
    fig, axes = plt.subplots(1, 4, figsize=(20, 5))
    fig.suptitle('Strategy Comparison', fontsize=13)
    x = np.arange(len(summaries))
    for ax, (key, title, fmt) in zip(axes, panels):
        vals = [s[key] for s in summaries]
        ax.bar(x, vals, color=cols, width=0.6)
        ax.set_xticks(x); ax.set_xticklabels(names, fontsize=9)
        ax.set_title(title, fontsize=11)
        top = max(max(vals), 1e-6)
        for xi, v in zip(x, vals):
            ax.text(xi, v + 0.02 * top, fmt.format(v), ha='center', va='bottom', fontsize=9)
        ax.set_ylim(0, top * 1.18)
    plt.tight_layout()
    path = os.path.join(out_dir, 'strategy_comparison.png')
    plt.savefig(path, dpi=130); plt.close()
    print(f'Saved: {path}')


def plot_speed_profiles(results, out_dir):
    """Commanded speed of each drone under the speed-adjustment strategy."""
    res = results[STRATEGIES.index('speed')]
    fig, ax = plt.subplots(figsize=(12, 4.5))
    for k in range(N_DRONES):
        sp = np.where(res['active'][:, k], res['speed'][:, k], np.nan)
        ax.plot(res['times'], sp, color=DRONE_COLORS[k], lw=1.5, label=f'D{k+1}')
    ax.axhline(V_MIN, color='k', ls=':', lw=1, label=f'v_min = {V_MIN} m/s')
    ax.set_xlabel('Time (s)'); ax.set_ylabel('Speed (m/s)')
    ax.set_title('Speed Adjustment Strategy - Commanded Speed per Drone')
    ax.set_ylim(0, V_DRONE + 1); ax.legend(fontsize=8, ncol=5); ax.grid(alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'speed_profiles.png')
    plt.savefig(path, dpi=130); plt.close()
    print(f'Saved: {path}')


def plot_altitude_timeseries(results, out_dir):
    """Altitude vs time for the altitude-layer strategy (airborne only)."""
    res = results[STRATEGIES.index('altitude')]
    fig, ax = plt.subplots(figsize=(12, 5))
    for k in range(N_DRONES):
        z = np.where(res['active'][:, k], res['pos'][:, k, 2], np.nan)
        ax.plot(res['times'], z, color=DRONE_COLORS[k], lw=1.5,
                label=f"D{k+1} ({res['layers'][k]:.0f} m)")
    ax.axhline(Z_NOM, color='black', ls=':', lw=1.2, label=f'z_nom = {Z_NOM} m')
    ax.set_xlabel('Time (s)'); ax.set_ylabel('Altitude (m)')
    ax.set_title('Altitude Layers - climb at hub, cruise on layer, descend at goal')
    ax.legend(fontsize=8, ncol=3); ax.set_xlim(left=0); ax.grid(alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'altitude_timeseries.png')
    plt.savefig(path, dpi=130); plt.close()
    print(f'Saved: {path}')


def save_animation(results, out_dir):
    """Animated top-down view of all four runs (<= ~150 frames)."""
    import matplotlib.animation as animation
    n_steps = max(len(r['times']) for r in results)
    stride = max(1, int(np.ceil(n_steps / 150)))
    frames = list(range(0, n_steps, stride))

    fig, axes = plt.subplots(1, 4, figsize=(18, 5))
    artists = []
    for ax, res in zip(axes, results):
        ax.set_xlim(0, ARENA); ax.set_ylim(0, ARENA); ax.set_aspect('equal')
        ax.set_title(LABELS[res['strategy']], fontsize=10)
        ax.set_xticks([]); ax.set_yticks([])
        for k in range(N_DRONES):
            ax.plot([res['origins'][k, 0], res['goals'][k, 0]],
                    [res['origins'][k, 1], res['goals'][k, 1]],
                    color='lightgrey', lw=0.6, ls='--')
            ax.plot(res['goals'][k, 0], res['goals'][k, 1], '*', color=DRONE_COLORS[k], ms=7)
        dots = [ax.plot([], [], 'o', color=DRONE_COLORS[k], ms=5)[0] for k in range(N_DRONES)]
        trails = [ax.plot([], [], '-', color=DRONE_COLORS[k], lw=1)[0] for k in range(N_DRONES)]
        ring = ax.plot([], [], 'o', mfc='none', mec='red', ms=14, mew=2)[0]
        artists.append((dots, trails, ring))
    ttl = fig.suptitle('')

    def update(fi):
        f_glob = frames[fi]
        ttl.set_text(f'Path De-confliction   t = {f_glob * DT:.1f} s')
        out = [ttl]
        for res, (dots, trails, ring) in zip(results, artists):
            f = min(f_glob, len(res['times']) - 1)
            P, A = res['pos'], res['active']
            for k in range(N_DRONES):
                m = A[:f + 1, k]
                tr = P[:f + 1, k][m]
                trails[k].set_data(tr[:, 0], tr[:, 1])
                if A[f, k]:
                    dots[k].set_data([P[f, k, 0]], [P[f, k, 1]])
                else:
                    dots[k].set_data([], [])
            rx, ry = [], []
            idx = np.flatnonzero(A[f])
            for a, b in combinations(idx, 2):
                if np.linalg.norm(P[f, a] - P[f, b]) < D_SEP:
                    rx.append(0.5 * (P[f, a, 0] + P[f, b, 0]))
                    ry.append(0.5 * (P[f, a, 1] + P[f, b, 1]))
            ring.set_data(rx, ry)
            out += dots + trails + [ring]
        return out

    ani = animation.FuncAnimation(fig, update, frames=len(frames), interval=50, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer='pillow', fps=20, dpi=80)
    plt.close()
    print(f'Saved: {path}  ({len(frames)} frames)')


# ── Main ───────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    out_dir = os.path.normpath(OUTPUT_DIR)
    os.makedirs(out_dir, exist_ok=True)

    origins, goals, launch = make_traffic(SEED)
    print('Traffic (seed 7):')
    for k in range(N_DRONES):
        print(f'  D{k+1}: ({origins[k,0]:5.1f},{origins[k,1]:5.1f}) -> '
              f'({goals[k,0]:5.1f},{goals[k,1]:5.1f})  launch {launch[k]:.1f} s  '
              f'L = {np.linalg.norm(goals[k]-origins[k]):.1f} m')

    results, summaries = [], []
    for strat in STRATEGIES:
        res = run_simulation(strat)
        s = summarise(res)
        results.append(res); summaries.append(s)
        print(f'\nStrategy: {strat}')
        print(f'  All drones arrived       : {res["all_arrived"]}')
        print(f'  Predicted conflict events: {s["n_conflicts"]}')
        print(f'  LoS events (< {D_SEP} m)    : {s["n_los"]}  pairs {s["los_pairs"]}')
        print(f'  Min airborne separation  : {s["min_sep"]:.2f} m')
        print(f'  Total added flight dist  : {s["added_dist"]:.1f} m')
        print(f'  Total / max arrival delay: {s["total_delay"]:.2f} s / {s["max_delay"]:.2f} s')
        if strat == 'speed':
            print(f'  Lowest commanded speed   : {s["min_speed"]:.2f} m/s')
        if strat == 'waypoint':
            print(f'  Avoidance waypoints      : {s["n_wp"]}')
        if strat == 'none':
            for (tt, i, j, x, y) in res['conflict_events']:
                print(f'    conflict D{i+1}-D{j+1} first flagged t={tt:.1f} s')

    plot_trajectories_2d(results, out_dir)
    plot_3d_altitude_layers(results, out_dir)
    plot_separation_timeseries(results, out_dir)
    plot_strategy_comparison(summaries, out_dir)
    plot_speed_profiles(results, out_dir)
    plot_altitude_timeseries(results, out_dir)
    save_animation(results, out_dir)

    print('\n=== Summary ===')
    for s in summaries:
        print(f"  {s['strategy']:9s}: conflicts={s['n_conflicts']:2d}  LoS={s['n_los']:2d}  "
              f"min_sep={s['min_sep']:6.2f} m  added_dist={s['added_dist']:6.1f} m  "
              f"total_delay={s['total_delay']:6.2f} s  max_delay={s['max_delay']:5.2f} s")
