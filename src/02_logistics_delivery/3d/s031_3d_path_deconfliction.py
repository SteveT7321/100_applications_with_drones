"""
S031 3D Upgrade - Path De-confliction
=====================================
Eight delivery drones fly straight 3-D routes in a 500 x 500 m urban airspace
column z in [15, 60] m.  As in the fixed 2D S031 they launch from four edge
hubs at 20 m (two drones per hub, second wave 3 s later) toward delivery points
on the opposite edge (horizontal layout identical to 2D seed 7), but every goal
now has its own altitude drawn from U[15, 50] m, so each nominal path is a true
3-D line segment.

Separation is a cylinder (card): a loss of separation (LoS) is
d_h < r_sep = 5 m AND d_v < h_sep = 3 m.  Conflict detection is an exact 4-D
test: for constant velocities it intersects the time interval in which
d_h(t) < r with the interval in which |dz(t)| < h, inside (0, T_horizon].

Strategies (plus an unresolved baseline):
  0. none       - no de-confliction (how much conflict the traffic contains)
  1. speed      - lower-priority drone slows (descending speed scan) while the
                  enlarged cylinder (r+margin, h+margin) is predicted to be hit
  2. horizontal - lateral avoidance waypoint (horizontal normal), 2D baseline
  3. layers     - each drone pre-assigned its own altitude layer, dz = 5 m
  4. vertical   - NEW: lower-priority drone inserts a climb/descend waypoint so
                  the predicted vertical separation becomes h_sep + margin

The seed-7 instance is shown in detail; a 100-seed Monte Carlo (same seeds,
same traffic for every strategy) on the 3-D traffic and on the same traffic
flattened to z = 20 m (the 2D situation) gives spread and win counts.

Outputs (outputs/02_logistics_delivery/3d/s031_3d_path_deconfliction/):
  trajectories_3d.png, topdown_2d.png, altitude_timeseries.png,
  separation_timeseries.png, strategy_comparison.png, monte_carlo.png,
  animation.gif

Usage:
    conda activate drones
    python src/02_logistics_delivery/3d/s031_3d_path_deconfliction.py
"""

import matplotlib
matplotlib.use('Agg')

import os
from itertools import combinations
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from matplotlib.animation import FuncAnimation, PillowWriter

OUTPUT_DIR = os.path.normpath(os.path.join(
    os.path.dirname(__file__), '..', '..', '..',
    'outputs', '02_logistics_delivery', '3d', 's031_3d_path_deconfliction',
))

# ── Parameters (card unless noted) ────────────────────────────────
N_DRONES    = 8
V_DRONE     = 12.0        # m/s cruise speed (3-D speed along the path)
V_MIN       = 4.0         # m/s minimum speed, strategy 1
V_STEP      = 0.25        # m/s speed-scan resolution, strategy 1 (as 2D fix)
V_Z_MAX     = 5.0         # m/s climb / descent rate limit (added)
R_SEP       = 5.0         # m horizontal separation radius
H_SEP       = 3.0         # m vertical separation half-height
D_MARGIN    = 2.0         # m clearance margin
T_HORIZON   = 30.0        # s look-ahead window
DT          = 0.1         # s time step
T_MAX       = 200.0       # s simulation cap
Z_MIN, Z_MAX = 15.0, 60.0 # m airspace column
Z_NOM       = 20.0        # m hub (depot) altitude
GOAL_Z_LO, GOAL_Z_HI = 15.0, 50.0   # m goal altitude range
Z_BASE      = 20.0        # m lowest layer, strategy 3
DZ_LAYER    = 5.0         # m layer spacing, strategy 3 -> 20 ... 55 m
ARENA       = 500.0       # m
HUB_INSET   = 20.0        # m (2D fix)
GOAL_SPREAD = 40.0        # m (2D fix)
T_STAGGER   = 3.0         # s second-wave launch delay (2D fix)
SEED        = 7           # showcase instance (same x-y traffic as 2D)
MC_SEEDS    = 100         # Monte Carlo traffic seeds 0 .. 99
HCLOSE      = 2 * R_SEP   # m "horizontally close" band for the d_v plot

STRATEGIES = ['none', 'speed', 'horizontal', 'layers', 'vertical']
LABELS = {'none': 'No de-confliction', 'speed': 'S1 Speed adjustment',
          'horizontal': 'S2 Horizontal waypoint', 'layers': 'S3 Altitude layers',
          'vertical': 'S4 Vertical climb-to-avoid'}
SHORT = {'none': 'None', 'speed': 'S1\nspeed', 'horizontal': 'S2\nhorizontal',
         'layers': 'S3\nlayers', 'vertical': 'S4\nvertical'}
STRAT_COLORS = {'none': 'tab:gray', 'speed': 'tab:blue', 'horizontal': 'tab:orange',
                'layers': 'tab:green', 'vertical': 'tab:purple'}
DRONE_COLORS = plt.cm.tab10(np.linspace(0, 0.9, N_DRONES))
PAIRS = np.array(list(combinations(range(N_DRONES), 2)))
PI, PJ = PAIRS[:, 0], PAIRS[:, 1]
EPS = 1e-12


# ── Traffic ───────────────────────────────────────────────────────

def make_traffic(seed, flat=False):
    """4 edge hubs at 20 m, goals on the opposite edge; goal z ~ U[15, 50] m.

    The x-y offsets are drawn first with the same generator call as the 2D
    script, so seed 7 has exactly the 2D horizontal layout.  flat=True puts
    every goal at 20 m (the 2D situation) with identical x-y.
    """
    rng = np.random.default_rng(seed)
    lo, hi, mid = HUB_INSET, ARENA - HUB_INSET, ARENA / 2
    hubs = np.array([[lo, mid], [hi, mid], [mid, lo], [mid, hi]])
    tangents = np.array([[0, 1], [0, 1], [1, 0], [1, 0]], dtype=float)
    opposite = [1, 0, 3, 2]
    offsets = rng.uniform(-GOAL_SPREAD, GOAL_SPREAD, N_DRONES)
    goal_z = rng.uniform(GOAL_Z_LO, GOAL_Z_HI, N_DRONES)
    if flat:
        goal_z = np.full(N_DRONES, Z_NOM)
    origins = np.zeros((N_DRONES, 3))
    goals = np.zeros((N_DRONES, 3))
    launch = np.zeros(N_DRONES)
    for k in range(N_DRONES):
        h = k % 4
        origins[k] = [*hubs[h], Z_NOM]
        goals[k] = [*(hubs[opposite[h]] + offsets[k] * tangents[opposite[h]]), goal_z[k]]
        launch[k] = 0.0 if k < 4 else T_STAGGER
    return origins, goals, launch


# ── 4-D cylinder conflict test ────────────────────────────────────

def cylinder_conflict(dp, dv, r, h, t_hor):
    """Exact cylinder test for linear relative motion dp + dv t (arrays (P, 3)).

    Returns (flag, t_c): flag where d_h < r and |dz| < h overlap inside
    (0, t_hor]; t_c is the horizontal-CPA time clamped into the overlap
    interval.  A pair already inside the cylinder and moving apart
    horizontally gets t_c = 0 and is not flagged (same rule as the 2D t_CPA > 0).
    """
    dp = np.atleast_2d(dp); dv = np.atleast_2d(dv)
    a = dv[:, 0] ** 2 + dv[:, 1] ** 2
    b = 2 * (dp[:, 0] * dv[:, 0] + dp[:, 1] * dv[:, 1])
    c = dp[:, 0] ** 2 + dp[:, 1] ** 2 - r * r
    still = a < EPS
    disc = b * b - 4 * a * np.where(still, 1.0, c)
    sq = np.sqrt(np.maximum(disc, 0.0))
    a_s = np.where(still, 1.0, a)
    t1 = np.where(still, np.where(c < 0, -np.inf, np.inf), (-b - sq) / (2 * a_s))
    t2 = np.where(still, np.where(c < 0, np.inf, -np.inf), (-b + sq) / (2 * a_s))
    t1 = np.where(~still & (disc < 0), np.inf, t1)
    t2 = np.where(~still & (disc < 0), -np.inf, t2)
    vz, pz = dv[:, 2], dp[:, 2]
    flatv = np.abs(vz) < EPS
    vz_s = np.where(flatv, 1.0, vz)
    ta = (-h - pz) / vz_s
    tb = (h - pz) / vz_s
    za = np.where(flatv, np.where(np.abs(pz) < h, -np.inf, np.inf), np.minimum(ta, tb))
    zb = np.where(flatv, np.where(np.abs(pz) < h, np.inf, -np.inf), np.maximum(ta, tb))
    lo = np.maximum(np.maximum(t1, za), 0.0)
    hi = np.minimum(np.minimum(t2, zb), t_hor)
    t_h = np.where(still, 0.0, -b / (2 * a_s))
    t_c = np.clip(t_h, lo, np.maximum(hi, lo))
    flag = (hi > lo) & (t_c > 0.0)
    return flag, t_c


def seg_speed(unit, v_cmd):
    """Speed along a segment: commanded speed, capped by the climb-rate limit."""
    uz = abs(unit[2])
    return v_cmd if uz * v_cmd <= V_Z_MAX else V_Z_MAX / uz


# ── Resolution rules ──────────────────────────────────────────────

def resolve_speed(p_y, u_y, p_o, v_o, v_now):
    """Highest speed in [V_MIN, v_now] (descending scan) clearing the enlarged cylinder."""
    for v in np.arange(v_now, V_MIN - 1e-9, -V_STEP):
        vel = seg_speed(u_y, v) * u_y
        f, _ = cylinder_conflict(p_y - p_o, vel - v_o, R_SEP + D_MARGIN,
                                 H_SEP + D_MARGIN, T_HORIZON)
        if not f[0]:
            return float(v)
    return V_MIN


def horizontal_waypoint(p_y, v_y, p_o, v_o, t_c):
    """Card S2: CPA midpoint + (r_sep + margin) * horizontal normal of dv.

    z is the yielding drone's own predicted altitude at t_c (deviation: the
    card holds the midpoint z, which drags the yielder toward the intruder).
    """
    py, po = p_y + v_y * t_c, p_o + v_o * t_c
    mid = 0.5 * (py + po)
    dv = v_y - v_o
    n = np.array([-dv[1], dv[0], 0.0])
    if np.linalg.norm(n) < 1e-9:
        n = np.array([1.0, 0.0, 0.0])
    n /= np.linalg.norm(n)
    rel = (py - po) * np.array([1.0, 1.0, 0.0])
    if rel @ n < 0:
        n = -n
    wp = mid + (R_SEP + D_MARGIN) * n
    wp[2] = py[2]
    return wp


def vertical_waypoint(p_y, v_y, p_o, v_o, t_c):
    """Card S4: w = p_y(t_c) + [0, 0, dz_avoid], dz_avoid = s (h_sep + m - |dz|).

    s = sign(dz) (move away from the intruder; s = +1 when dz = 0).  If the
    waypoint would leave [Z_MIN, Z_MAX] the drone passes on the other side
    (offset -s (h_sep + m + |dz|)).
    """
    py, po = p_y + v_y * t_c, p_o + v_o * t_c
    dz = py[2] - po[2]
    s = 1.0 if dz >= 0 else -1.0
    dz_avoid = s * (H_SEP + D_MARGIN - abs(dz))
    wz = py[2] + dz_avoid
    if not (Z_MIN <= wz <= Z_MAX):
        wz = py[2] - s * (H_SEP + D_MARGIN + abs(dz))
    assert Z_MIN <= wz <= Z_MAX
    return np.array([py[0], py[1], wz]), wz - py[2]


# ── Simulation ────────────────────────────────────────────────────

def run_simulation(strategy, seed=SEED, flat=False, record=True):
    """Simulate one strategy on one traffic instance; returns a result dict."""
    origins, goals, launch = make_traffic(seed, flat)
    nominal_len = np.linalg.norm(goals - origins, axis=1)
    u_nom = (goals - origins) / nominal_len[:, None]
    assert np.all(np.abs(u_nom[:, 2]) * V_DRONE <= V_Z_MAX)   # nominal paths unaffected
    nominal_arrival = launch + nominal_len / V_DRONE

    if strategy == 'layers':
        layers = Z_BASE + DZ_LAYER * np.arange(N_DRONES)
        queues = []
        for k in range(N_DRONES):
            q = []
            if abs(layers[k] - origins[k, 2]) > 1e-9:
                q.append(np.array([origins[k, 0], origins[k, 1], layers[k]]))
            q.append(np.array([goals[k, 0], goals[k, 1], layers[k]]))
            if abs(layers[k] - goals[k, 2]) > 1e-9:
                q.append(goals[k].copy())
            queues.append(q)
    else:
        layers = None
        queues = [[goals[k].copy()] for k in range(N_DRONES)]
    has_wp = np.zeros(N_DRONES, dtype=bool)

    pos = origins.copy()
    arrived = np.zeros(N_DRONES, dtype=bool)
    arrival = np.full(N_DRONES, np.nan)
    path_len = np.zeros(N_DRONES)
    wp_offsets, wp_log = [], []
    flagged_seen, los_seen = set(), set()
    conflict_events, los_events = [], []
    H = {'t': [], 'pos': [], 'active': [], 'speed': [], 'sigma': [], 'dh': [], 'dv': []}
    min_sigma = np.inf
    t = 0.0

    while not arrived.all() and t < T_MAX:
        active = (launch <= t + 1e-9) & ~arrived
        tgt = np.array([queues[k][0] if queues[k] else goals[k] for k in range(N_DRONES)])
        d = tgt - pos
        dist = np.linalg.norm(d, axis=1)
        unit = np.where(dist[:, None] > 1e-9, d / np.maximum(dist, 1e-9)[:, None], 0.0)
        vcmd = np.where(active, V_DRONE, 0.0)
        vel = np.array([seg_speed(unit[k], vcmd[k]) * unit[k] for k in range(N_DRONES)])

        # 4-D conflict detection at nominal commanded speed
        both = active[PI] & active[PJ]
        flag, tcs = cylinder_conflict(pos[PI] - pos[PJ], vel[PI] - vel[PJ],
                                      R_SEP, H_SEP, T_HORIZON)
        flag &= both
        conflicts = [(int(PI[p]), int(PJ[p]), float(tcs[p])) for p in np.flatnonzero(flag)]
        fl = {(i, j) for i, j, _ in conflicts}
        for (i, j) in sorted(fl - flagged_seen):
            conflict_events.append((t, i, j))
        flagged_seen |= fl

        order = sorted(conflicts, key=lambda c: (c[1], c[0]))   # larger index yields
        if strategy == 'speed':
            for i, j, _ in order:
                y, o = j, i
                v_new = resolve_speed(pos[y], unit[y], pos[o], vel[o], vcmd[y])
                if v_new < vcmd[y]:
                    vcmd[y] = v_new
                    vel[y] = seg_speed(unit[y], v_new) * unit[y]
        elif strategy in ('horizontal', 'vertical'):
            for i, j, tc in order:
                y, o = j, i
                if has_wp[y]:
                    continue
                if strategy == 'horizontal':
                    wp = horizontal_waypoint(pos[y], vel[y], pos[o], vel[o], tc)
                    off = 0.0
                else:
                    wp, off = vertical_waypoint(pos[y], vel[y], pos[o], vel[o], tc)
                wp_offsets.append(off)
                wp_log.append((t, y, o, t + tc, wp.copy(),
                               float(np.linalg.norm(wp - pos[y]) / seg_speed(
                                   (wp - pos[y]) / max(np.linalg.norm(wp - pos[y]), 1e-9), vcmd[y]))))
                queues[y].insert(0, wp)
                has_wp[y] = True
                dy = wp - pos[y]
                unit[y] = dy / max(np.linalg.norm(dy), 1e-9)
                vel[y] = seg_speed(unit[y], vcmd[y]) * unit[y]

        # Separation bookkeeping (airborne pairs only)
        dpair = pos[PI] - pos[PJ]
        dh = np.hypot(dpair[:, 0], dpair[:, 1])
        dvv = np.abs(dpair[:, 2])
        sigma = np.maximum(dh / R_SEP, dvv / H_SEP)
        if both.any():
            min_sigma = min(min_sigma, float(sigma[both].min()))
        los = both & (dh < R_SEP) & (dvv < H_SEP)
        lnow = {(int(PI[p]), int(PJ[p])) for p in np.flatnonzero(los)}
        for (a, b) in sorted(lnow - los_seen):
            los_events.append((t, a, b, *(0.5 * (pos[a] + pos[b]))))
        los_seen |= lnow
        if record:
            H['t'].append(t); H['pos'].append(pos.copy()); H['active'].append(active.copy())
            H['speed'].append(np.linalg.norm(vel, axis=1))
            H['sigma'].append(sigma[both].min() if both.any() else np.nan)
            H['dh'].append(dh[both].min() if both.any() else np.nan)
            close = both & (dh < HCLOSE)
            H['dv'].append(dvv[close].min() if close.any() else np.nan)

        # Advance with an exact time budget (carry-over through waypoints)
        for k in np.flatnonzero(active):
            tau = DT
            while tau > 1e-12:
                tg = queues[k][0]
                dd = tg - pos[k]
                seg = float(np.linalg.norm(dd))
                if seg < 1e-12:
                    queues[k].pop(0)
                else:
                    u = dd / seg
                    sp = seg_speed(u, vcmd[k])
                    if seg / sp <= tau:
                        pos[k] = tg.copy(); tau -= seg / sp; path_len[k] += seg
                        queues[k].pop(0)
                    else:
                        pos[k] = pos[k] + sp * tau * u; path_len[k] += sp * tau; tau = 0.0
                if not queues[k]:
                    arrived[k] = True
                    arrival[k] = t + (DT - tau)
                    break
                if has_wp[k] and len(queues[k]) == 1:
                    has_wp[k] = False                 # avoidance waypoint reached
            assert Z_MIN - 1e-9 <= pos[k, 2] <= Z_MAX + 1e-9, 'left the airspace column'
        t = round(t + DT, 10)

    assert arrived.all(), 'not all drones arrived'
    if record:
        H['t'].append(t); H['pos'].append(pos.copy())
        H['active'].append(np.zeros(N_DRONES, dtype=bool)); H['speed'].append(np.zeros(N_DRONES))
        for key in ('sigma', 'dh', 'dv'):
            H[key].append(np.nan)
    delays = np.round(arrival - nominal_arrival, 6) + 0.0
    added = np.round(path_len - nominal_len, 6) + 0.0
    assert np.all(added >= -1e-6) and np.all(delays >= -DT)
    res = {'strategy': strategy, 'seed': seed, 'flat': flat, 'origins': origins,
           'goals': goals, 'launch': launch, 'layers': layers,
           'conflict_events': conflict_events, 'los_events': los_events,
           'delays': delays, 'added': added, 'min_sigma': min_sigma,
           'wp_offsets': wp_offsets, 'wp_log': wp_log}
    if record:
        res.update({k: np.array(v) for k, v in H.items()})
        res['times'] = res.pop('t')
    return res


def summarise(r):
    return {'n_conf': len(r['conflict_events']), 'n_los': len(r['los_events']),
            'los_pairs': sorted({(i + 1, j + 1) for _, i, j, *_ in r['los_events']}),
            'added': float(r['added'].sum()), 'max_delay': float(r['delays'].max()),
            'total_delay': float(r['delays'].sum()), 'min_sigma': r['min_sigma']}


def run_monte_carlo():
    """All strategies on MC_SEEDS traffic seeds, 3-D and flattened traffic."""
    out = {}
    for flat in (False, True):
        M = {s: {'los': [], 'added': [], 'max_delay': [], 'n_conf': []} for s in STRATEGIES}
        for seed in range(MC_SEEDS):
            for s in STRATEGIES:
                sm = summarise(run_simulation(s, seed, flat, record=False))
                M[s]['los'].append(sm['n_los']); M[s]['added'].append(sm['added'])
                M[s]['max_delay'].append(sm['max_delay']); M[s]['n_conf'].append(sm['n_conf'])
        out[flat] = {s: {k: np.array(v) for k, v in M[s].items()} for s in STRATEGIES}
    return out


# ── Plots ─────────────────────────────────────────────────────────

def _los_marks(ax, r, three_d=True):
    for (_, i, j, x, y, z) in r['los_events']:
        if three_d:
            ax.scatter([x], [y], [z], s=120, facecolors='none', edgecolors='red', linewidths=2)
        else:
            ax.add_patch(plt.Circle((x, y), 12, fill=False, color='red', lw=2, zorder=4))


def plot_trajectories_3d(results, out_dir):
    """3-D flown paths for the four strategies, colour = altitude."""
    fig = plt.figure(figsize=(16, 13))
    norm = plt.Normalize(Z_MIN, Z_MAX)
    sc = None
    for n, r in enumerate(results[1:]):
        ax = fig.add_subplot(2, 2, n + 1, projection='3d')
        P, A = r['pos'], r['active']
        for k in range(N_DRONES):
            m = A[:, k]
            sc = ax.scatter(P[m, k, 0], P[m, k, 1], P[m, k, 2], c=P[m, k, 2], cmap='viridis',
                            norm=norm, s=2)
            ax.scatter(*r['origins'][k], c='k', marker='s', s=20)
            ax.scatter(*r['goals'][k], c='green', marker='*', s=60)
        _los_marks(ax, r)
        s = summarise(r)
        ax.set_title(f"{LABELS[r['strategy']]}\nLoS pairs = {s['n_los']}, "
                     f"added = {s['added']:.1f} m, max delay = {s['max_delay']:.2f} s", fontsize=11)
        ax.set_xlim(0, ARENA); ax.set_ylim(0, ARENA); ax.set_zlim(Z_MIN, Z_MAX)
        ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
        ax.view_init(elev=28, azim=-60)
    fig.colorbar(sc, ax=fig.axes, shrink=0.5, label='altitude z (m)')
    fig.suptitle(f'S031 3D - flown paths, seed {SEED} (black square = hub, green star = goal, '
                 'red ring = loss of separation)', fontsize=13)
    path = os.path.join(out_dir, 'trajectories_3d.png')
    plt.savefig(path, dpi=110); plt.close()
    print(f'Saved: {path}')


def plot_topdown(results, out_dir):
    """Top-down footprints of all five runs (compare with 2D S031)."""
    fig, axes = plt.subplots(1, 5, figsize=(25, 5.6))
    for ax, r in zip(axes, results):
        P, A = r['pos'], r['active']
        for k in range(N_DRONES):
            m = A[:, k]
            ax.plot([r['origins'][k, 0], r['goals'][k, 0]], [r['origins'][k, 1], r['goals'][k, 1]],
                    color='lightgrey', lw=0.8, ls='--', zorder=1)
            ax.plot(P[m, k, 0], P[m, k, 1], color=DRONE_COLORS[k], lw=1.3, label=f'D{k+1}')
            ax.plot(*r['origins'][k, :2], 'ks', ms=5)
            ax.plot(*r['goals'][k, :2], '*', color=DRONE_COLORS[k], ms=9)
        _los_marks(ax, r, three_d=False)
        s = summarise(r)
        ax.set_title(f"{LABELS[r['strategy']]}\nconflict pairs={s['n_conf']}, LoS pairs={s['n_los']}",
                     fontsize=10)
        ax.set_xlim(0, ARENA); ax.set_ylim(0, ARENA); ax.set_aspect('equal')
        ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
    axes[0].legend(fontsize=7, ncol=2, loc='upper left')
    fig.suptitle(f'Top-down footprints, 3-D traffic seed {SEED} (red circle = cylinder LoS)', fontsize=13)
    plt.tight_layout()
    path = os.path.join(out_dir, 'topdown_2d.png')
    plt.savefig(path, dpi=110); plt.close()
    print(f'Saved: {path}')


def plot_altitude_timeseries(results, out_dir):
    fig, axes = plt.subplots(5, 1, figsize=(12, 15), sharex=True)
    for ax, r in zip(axes, results):
        for k in range(N_DRONES):
            z = np.where(r['active'][:, k], r['pos'][:, k, 2], np.nan)
            ax.plot(r['times'], z, color=DRONE_COLORS[k], lw=1.3, label=f'D{k+1}')
        for (tt, i, j, *_rest) in r['los_events']:
            ax.axvline(tt, color='red', lw=0.8, ls=':')
        ax.set_ylabel('z (m)'); ax.set_ylim(Z_MIN - 2, Z_MAX + 2)
        ax.set_title(LABELS[r['strategy']], fontsize=10); ax.grid(alpha=0.3)
    axes[0].legend(fontsize=7, ncol=8, loc='upper right')
    axes[-1].set_xlabel('Time (s)')
    fig.suptitle(f'Altitude z_i(t), seed {SEED} (red dotted = first LoS of a pair)', fontsize=12)
    plt.tight_layout()
    path = os.path.join(out_dir, 'altitude_timeseries.png')
    plt.savefig(path, dpi=110); plt.close()
    print(f'Saved: {path}')


def plot_separation_timeseries(results, out_dir):
    fig, axes = plt.subplots(3, 1, figsize=(12, 11), sharex=True)
    lss = [':', '-', '--', '-.', '-']
    for r, ls in zip(results, lss):
        c = STRAT_COLORS[r['strategy']]
        axes[0].plot(r['times'], r['sigma'], ls=ls, color=c, lw=1.4, label=LABELS[r['strategy']])
        axes[1].plot(r['times'], r['dh'], ls=ls, color=c, lw=1.4)
        axes[2].plot(r['times'], r['dv'], ls=ls, color=c, lw=1.4, marker='.', ms=2)
    axes[0].axhline(1.0, color='red', ls='--', lw=1.5, label='LoS boundary (sigma = 1)')
    axes[0].set_ylabel('min sigma = max(d_h/r, d_v/h)'); axes[0].set_ylim(0, 6)
    axes[1].axhline(R_SEP, color='red', ls='--', lw=1.5, label=f'r_sep = {R_SEP} m')
    axes[1].set_ylabel('min d_h (m)'); axes[1].set_ylim(0, 60)
    axes[2].axhline(H_SEP, color='red', ls='--', lw=1.5, label=f'h_sep = {H_SEP} m')
    axes[2].set_ylabel(f'min d_v of pairs\nwith d_h < {HCLOSE:.0f} m'); axes[2].set_ylim(0, 25)
    for ax in axes:
        ax.legend(fontsize=8, loc='upper right'); ax.grid(alpha=0.3)
    axes[2].set_xlabel('Time (s)')
    fig.suptitle(f'Separation time series, seed {SEED} (airborne pairs only)', fontsize=12)
    plt.tight_layout()
    path = os.path.join(out_dir, 'separation_timeseries.png')
    plt.savefig(path, dpi=110); plt.close()
    print(f'Saved: {path}')


def plot_strategy_comparison(summaries, out_dir):
    names = [SHORT[s] for s in STRATEGIES]
    cols = [STRAT_COLORS[s] for s in STRATEGIES]
    panels = [('n_los', 'LoS pairs (cylinder)', '{:.0f}'),
              ('added', 'Total added 3-D distance (m)', '{:.2f}'),
              ('max_delay', 'Max arrival delay (s)', '{:.2f}'),
              ('total_delay', 'Total arrival delay (s)', '{:.2f}')]
    fig, axes = plt.subplots(1, 4, figsize=(22, 5))
    x = np.arange(len(STRATEGIES))
    for ax, (key, title, fmt) in zip(axes, panels):
        vals = [summaries[s][key] for s in STRATEGIES]
        ax.bar(x, vals, color=cols, width=0.6)
        top = max(max(vals), 1e-6)
        for xi, v in zip(x, vals):
            ax.text(xi, v + 0.02 * top, fmt.format(v), ha='center', va='bottom', fontsize=9)
        ax.set_ylim(0, top * 1.2); ax.set_xticks(x); ax.set_xticklabels(names, fontsize=8)
        ax.set_title(title, fontsize=11)
    fig.suptitle(f'Strategy comparison, seed {SEED} (3-D traffic)', fontsize=13)
    plt.tight_layout()
    path = os.path.join(out_dir, 'strategy_comparison.png')
    plt.savefig(path, dpi=110); plt.close()
    print(f'Saved: {path}')


def plot_monte_carlo(mc, out_dir):
    fig, axes = plt.subplots(1, 3, figsize=(20, 5.5))
    x = np.arange(len(STRATEGIES))
    w = 0.38
    for off, flat, hatch in ((-w / 2, True, '//'), (w / 2, False, '')):
        tot = [mc[flat][s]['los'].sum() for s in STRATEGIES]
        axes[0].bar(x + off, tot, w, color=[STRAT_COLORS[s] for s in STRATEGIES], hatch=hatch,
                    edgecolor='k', label='flat traffic (all goals 20 m)' if flat else '3-D traffic')
        for xi, v in zip(x + off, tot):
            axes[0].text(xi, v + 1, f'{v}', ha='center', fontsize=8)
    axes[0].set_title(f'Total LoS pairs over {MC_SEEDS} seeds'); axes[0].legend(fontsize=8)
    for ax, key, title in ((axes[1], 'added', 'Added 3-D distance per seed (m)'),
                           (axes[2], 'max_delay', 'Max arrival delay per seed (s)')):
        data = [mc[False][s][key] for s in STRATEGIES[1:]]
        bp = ax.boxplot(data, patch_artist=True, widths=0.6)
        for patch, s in zip(bp['boxes'], STRATEGIES[1:]):
            patch.set_facecolor(STRAT_COLORS[s])
        ax.set_xticks(np.arange(1, 5)); ax.set_xticklabels([SHORT[s] for s in STRATEGIES[1:]], fontsize=9)
        ax.set_title(title + ', 3-D traffic'); ax.grid(alpha=0.3, axis='y')
    axes[1].set_yscale('symlog', linthresh=1.0); axes[1].set_ylim(bottom=0)
    axes[0].set_xticks(x); axes[0].set_xticklabels([SHORT[s] for s in STRATEGIES], fontsize=9)
    fig.suptitle(f'Monte Carlo over {MC_SEEDS} traffic seeds (same traffic for every strategy)',
                 fontsize=13)
    plt.tight_layout()
    path = os.path.join(out_dir, 'monte_carlo.png')
    plt.savefig(path, dpi=110); plt.close()
    print(f'Saved: {path}')


def save_animation(results, out_dir, step=4):
    """3-D animation of the four strategies side by side (seed 7)."""
    runs = results[1:]
    n = max(len(r['times']) for r in runs)
    n_frames = int(np.ceil((n - 1) / step)) + 1
    fig = plt.figure(figsize=(14, 11))
    axes, arts = [], []
    for q, r in enumerate(runs):
        ax = fig.add_subplot(2, 2, q + 1, projection='3d')
        ax.set_xlim(0, ARENA); ax.set_ylim(0, ARENA); ax.set_zlim(Z_MIN, Z_MAX)
        ax.set_title(LABELS[r['strategy']], fontsize=10); ax.view_init(elev=28, azim=-60)
        ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
        for k in range(N_DRONES):
            ax.scatter(*r['goals'][k], c='green', marker='*', s=40)
        trails = [ax.plot([], [], [], '-', color=DRONE_COLORS[k], lw=1)[0] for k in range(N_DRONES)]
        dots = [ax.plot([], [], [], 'o', color=DRONE_COLORS[k], ms=4)[0] for k in range(N_DRONES)]
        axes.append(ax); arts.append((trails, dots))
    ttl = fig.suptitle('')

    def update(fi):
        g = min(fi * step, n - 1)
        ttl.set_text(f'S031 3D path de-confliction, seed {SEED}   t = {g * DT:.1f} s')
        for r, (trails, dots) in zip(runs, arts):
            f = min(g, len(r['times']) - 1)
            P, A = r['pos'], r['active']
            for k in range(N_DRONES):
                tr = P[:f + 1, k][A[:f + 1, k]]
                trails[k].set_data(tr[:, 0], tr[:, 1]); trails[k].set_3d_properties(tr[:, 2])
                if A[f, k]:
                    dots[k].set_data([P[f, k, 0]], [P[f, k, 1]]); dots[k].set_3d_properties([P[f, k, 2]])
                else:
                    dots[k].set_data([], []); dots[k].set_3d_properties([])
        return []

    ani = FuncAnimation(fig, update, frames=n_frames, interval=100, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer=PillowWriter(fps=10), dpi=60)
    plt.close()
    print(f'Saved: {path}  ({n_frames} frames)')


# ── Main ──────────────────────────────────────────────────────────

def _stats(a):
    return f'mean {a.mean():.2f}, median {np.median(a):.2f}, max {a.max():.2f}'


if __name__ == '__main__':
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    o, g, l = make_traffic(SEED)
    print(f'Traffic seed {SEED} (3-D goal altitudes; the flat variant has the same x-y, all goals at 20 m):')
    for k in range(N_DRONES):
        print(f'  D{k+1}: ({o[k,0]:5.1f},{o[k,1]:5.1f},{o[k,2]:4.1f}) -> '
              f'({g[k,0]:5.1f},{g[k,1]:5.1f},{g[k,2]:4.1f})  launch {l[k]:.1f} s  '
              f'L3D = {np.linalg.norm(g[k]-o[k]):.1f} m')

    results, summaries = [], {}
    flat_summ = {}
    for s in STRATEGIES:
        r = run_simulation(s, SEED, flat=False)
        results.append(r); summaries[s] = summarise(r)
        flat_summ[s] = summarise(run_simulation(s, SEED, flat=True, record=False))

    for label, SM in (('FLAT (2D situation)', flat_summ), ('3-D', summaries)):
        print(f'\n=== Seed {SEED}, {label} traffic ===')
        for s in STRATEGIES:
            m = SM[s]
            print(f"  {s:10s}: conflict pairs={m['n_conf']:2d}  LoS pairs={m['n_los']:2d} {m['los_pairs']}  "
                  f"min sigma={m['min_sigma']:.2f}  added={m['added']:7.2f} m  "
                  f"max delay={m['max_delay']:5.2f} s  total delay={m['total_delay']:6.2f} s")
    print('  avoidance waypoints (t inserted, yielder, intruder, predicted conflict time, waypoint, reached):')
    for r in results:
        for (ti, y, o, tcabs, wp, tt) in r['wp_log']:
            print(f"    {r['strategy']:10s}: t={ti:.1f} s D{y+1} yields to D{o+1}, predicted t_c={tcabs:.2f} s, "
                  f"w=({wp[0]:.2f}, {wp[1]:.2f}, {wp[2]:.2f}) m, reached t={ti + tt:.2f} s")
    rv = results[STRATEGIES.index('vertical')]
    print('  vertical waypoint offsets (m): ' + ', '.join(f'{v:+.2f}' for v in rv['wp_offsets']))
    print('  LoS pairs at their deepest point (min sigma):')
    for r in results:
        for (a, b) in sorted({(i, j) for _, i, j, *_x in r['los_events']}):
            mm = r['active'][:, a] & r['active'][:, b]
            dd = r['pos'][mm, a] - r['pos'][mm, b]
            dh = np.hypot(dd[:, 0], dd[:, 1]); dv = np.abs(dd[:, 2])
            sg = np.maximum(dh / R_SEP, dv / H_SEP); q = int(np.argmin(sg))
            print(f"    {r['strategy']:10s}: D{a+1}-D{b+1} t={r['times'][mm][q]:.1f} s  sigma={sg[q]:.2f}  "
                  f"d_h={dh[q]:.2f} m  d_v={dv[q]:.2f} m")
    print('  per-drone added distance (m):')
    for r in results[1:]:
        print(f"    {r['strategy']:10s}: " + ' '.join(f'D{k+1}={v:.2f}' for k, v in enumerate(r['added'])))
    print('  per-drone delay (s):')
    for r in results[1:]:
        print(f"    {r['strategy']:10s}: " + ' '.join(f'D{k+1}={v:.2f}' for k, v in enumerate(r['delays'])))
    for (tt, i, j) in results[0]['conflict_events']:
        print(f'    baseline conflict D{i+1}-D{j+1} first flagged t={tt:.1f} s')
    r0 = results[0]
    print('  flat-baseline LoS pairs in the 3-D baseline (at the closest horizontal approach):')
    for (i, j) in flat_summ['none']['los_pairs']:
        a, b = i - 1, j - 1
        m = r0['active'][:, a] & r0['active'][:, b]
        dd = r0['pos'][m, a] - r0['pos'][m, b]
        dh = np.hypot(dd[:, 0], dd[:, 1])
        q = int(np.argmin(dh))
        verdict = ('LoS' if (dh[q] < R_SEP and abs(dd[q, 2]) < H_SEP)
                   else 'separated vertically' if dh[q] < R_SEP else 'separated horizontally')
        print(f'    D{i}-D{j}: t={r0["times"][m][q]:.1f} s  d_h={dh[q]:.2f} m  '
              f'd_v={abs(dd[q, 2]):.2f} m  -> {verdict}')

    mc = run_monte_carlo()
    print(f'\n=== Monte Carlo, seeds 0-{MC_SEEDS-1} (same traffic for every strategy) ===')
    for flat in (True, False):
        print(f"--- {'FLAT traffic (all goals 20 m)' if flat else '3-D traffic'} ---")
        for s in STRATEGIES:
            M = mc[flat][s]
            print(f"  {s:10s}: LoS pairs total {M['los'].sum():3d}, seeds with LoS {int((M['los'] > 0).sum()):3d}/{MC_SEEDS}, "
                  f"max per seed {M['los'].max()} | added m: {_stats(M['added'])} | "
                  f"max delay s: {_stats(M['max_delay'])}")
    b3, bf = mc[False]['none']['los'], mc[True]['none']['los']
    print(f'  baseline LoS pairs 3-D vs flat: 3-D fewer on {int((b3 < bf).sum())} seeds, '
          f'equal on {int((b3 == bf).sum())}, more on {int((b3 > bf).sum())}; '
          f'totals {b3.sum()} vs {bf.sum()}')
    M3 = mc[False]
    def cmp(a, b, key, tol=1e-6):
        x, y = M3[a][key], M3[b][key]
        return int((x < y - tol).sum()), int((np.abs(x - y) <= tol).sum()), int((x > y + tol).sum())
    for a, b, key in (('vertical', 'horizontal', 'added'), ('vertical', 'horizontal', 'los'),
                      ('vertical', 'layers', 'added'), ('vertical', 'speed', 'max_delay'),
                      ('vertical', 'speed', 'los'), ('vertical', 'layers', 'max_delay')):
        lo, eq, hi = cmp(a, b, key)
        print(f'  {a} vs {b} on {key}: lower on {lo}, equal on {eq}, higher on {hi} seeds')
    safe = {s: M3[s]['los'] == 0 for s in STRATEGIES[1:]}
    both = safe['vertical'] & safe['horizontal']
    av, ah = M3['vertical']['added'][both], M3['horizontal']['added'][both]
    print(f'  seeds where both S2 and S4 have 0 LoS: {int(both.sum())}; on those S4 adds less distance on '
          f'{int((av < ah - 1e-6).sum())}, equal on {int((np.abs(av - ah) <= 1e-6).sum())}, '
          f'more on {int((av > ah + 1e-6).sum())}; median added S4 {np.median(av):.2f} m vs S2 {np.median(ah):.2f} m')
    nz = M3['none']['los'] > 0
    print(f'  seeds whose 3-D baseline has >= 1 LoS: {int(nz.sum())}; resolved to 0 LoS by '
          + ', '.join(f"{s} {int((M3[s]['los'][nz] == 0).sum())}" for s in STRATEGIES[1:]))

    plot_trajectories_3d(results, OUTPUT_DIR)
    plot_topdown(results, OUTPUT_DIR)
    plot_altitude_timeseries(results, OUTPUT_DIR)
    plot_separation_timeseries(results, OUTPUT_DIR)
    plot_strategy_comparison(summaries, OUTPUT_DIR)
    plot_monte_carlo(mc, OUTPUT_DIR)
    save_animation(results, OUTPUT_DIR)
