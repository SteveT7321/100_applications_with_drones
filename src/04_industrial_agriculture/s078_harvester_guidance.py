"""
S078 Harvester Cooperative Guidance
===================================
A harvester crosses a 200 x 50 m field along y = 25 m at 1.5 m/s. Fifteen unknown
obstacles (r = 1 m) are scattered over the field. A scout drone (3 m/s) flies ahead of the
harvester (at most L_lead = 45 m) and scans a corridor around the planned track; every
detection (sensor radius 2 m) reaches the harvester after a 2 s radio latency, and the
harvester replans with A* on a 0.5 m grid (obstacles inflated by 2.5 m), then smooths the
grid path by line-of-sight shortcutting.
Compared: the card's 3-pass lawnmower over the 10 m corridor vs a 1-pass centreline scan,
plus an oracle (all obstacles known at start), and sweeps over L_lead, latency and sensor
range. Each configuration is a 100-seed (sweeps: 30-seed) Monte Carlo with common random
numbers (same obstacle layout and GPS-noise samples for every configuration of a seed).

Usage:
    conda activate drones
    python src/04_industrial_agriculture/s078_harvester_guidance.py
"""

import os
import heapq
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Circle, Rectangle

# ── Parameters (card) ─────────────────────────────────────────────────────────
FIELD_X     = 200.0      # m
FIELD_Y     = 50.0       # m
V_HARV      = 1.5        # m/s
V_DRONE     = 3.0        # m/s
L_LEAD      = 45.0       # m   maximum lead of the scout ahead of the harvester
CORRIDOR_W  = 10.0       # m
W_SCAN      = 3.3        # m   strip spacing of the 3-pass pattern
R_SENSOR    = 2.0        # m
R_OBS       = 1.0        # m
R_HARV      = 1.5        # m   harvester half-width
R_INFLATE   = R_OBS + R_HARV   # 2.5 m
T_LATENCY   = 2.0        # s
GPS_ERROR   = 0.1        # m
GRID_RES    = 0.5        # m
DT          = 0.1        # s
T_REPLAN    = 5.0        # s   timely-detection margin
N_OBS       = 15
OBS_LO, OBS_HI = np.array([20.0, 5.0]), np.array([FIELD_X - 20.0, FIELD_Y - 5.0])
Y_TRACK     = FIELD_Y / 2.0
SEED        = 42

# ── Added parameters ──────────────────────────────────────────────────────────
BLOCK_LEN   = 10.0       # m   along-track length of one lawnmower block (= card's 10 m window)
GRID_MARGIN = GRID_RES / 2   # m  extra inflation of grid nodes (discretisation margin)
LOS_MARGIN  = 0.05       # m   clearance margin of the smoothed path
N_MC        = 100        # seeds for the main comparison
N_SWEEP     = 25         # seeds per sweep point
LEAD_SWEEP    = [15.0, 30.0, 45.0, 60.0, 75.0]
LATENCY_SWEEP = [0.0, 1.0, 2.0, 4.0, 8.0]
SENSOR_SWEEP  = [1.5, 2.0, 2.5, 3.0, 4.0]
MAX_STEPS   = 6000       # simulation cap (600 s)
ANIM_STEP   = 25         # simulation steps per GIF frame (2.5 s)

PATTERNS = {'3-pass': [-W_SCAN, 0.0, W_SCAN], '1-pass': [0.0]}
LABELS = {'3-pass': '3-pass lawnmower (card)', '1-pass': '1-pass centreline',
          'oracle': 'Oracle (all obstacles known)'}
COLORS = {'3-pass': '#9467BD', '1-pass': '#1F77B4', 'oracle': '#7F7F7F'}
HARV_COLOR, DRONE_COLOR, OBS_COLOR = '#D62728', '#1F77B4', '#7F7F7F'

NXN, NYN = int(round(FIELD_X / GRID_RES)) + 1, int(round(FIELD_Y / GRID_RES)) + 1
GOAL = np.array([FIELD_X, Y_TRACK])

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), '..', '..',
                          'outputs', '04_industrial_agriculture', 's078_harvester_guidance')


# ── Geometry helpers ──────────────────────────────────────────────────────────

def seg_point_dist(a, b, c):
    """Distance from points c (K, 2) to segment a-b."""
    ab = b - a
    L2 = float(ab @ ab)
    if L2 < 1e-12:
        return np.linalg.norm(c - a, axis=1)
    t = np.clip((c - a) @ ab / L2, 0.0, 1.0)
    return np.linalg.norm(a + t[:, None] * ab - c, axis=1)


def path_obstacle_dist(path, c):
    """Min distance from one point c to a polyline."""
    return min(float(seg_point_dist(a, b, c[None])[0]) for a, b in zip(path[:-1], path[1:]))


# ── Planner ───────────────────────────────────────────────────────────────────

def blocked_grid(known):
    xs = np.arange(NXN) * GRID_RES
    ys = np.arange(NYN) * GRID_RES
    X, Y = np.meshgrid(xs, ys, indexing='ij')
    B = np.zeros((NXN, NYN), dtype=bool)
    for c in known:
        B |= (X - c[0]) ** 2 + (Y - c[1]) ** 2 <= (R_INFLATE + GRID_MARGIN) ** 2
    return B


def clearance_grid(known):
    """Distance from every lattice node to the nearest known obstacle centre."""
    xs = np.arange(NXN) * GRID_RES
    ys = np.arange(NYN) * GRID_RES
    X, Y = np.meshgrid(xs, ys, indexing='ij')
    C = np.full((NXN, NYN), np.inf)
    for c in known:
        C = np.minimum(C, np.hypot(X - c[0], Y - c[1]))
    return C


def astar(B, start, goal, C=None):
    """8-connected A* on lattice nodes. From a blocked node (harvester already inside an
    inflated disc) the only allowed moves are those that increase the clearance C."""
    s = (int(round(start[0] / GRID_RES)), int(round(start[1] / GRID_RES)))
    s = (min(max(s[0], 0), NXN - 1), min(max(s[1], 0), NYN - 1))
    g_node = (int(round(goal[0] / GRID_RES)), int(round(goal[1] / GRID_RES)))
    sq2 = np.sqrt(2.0)

    def h(n):
        dx, dy = abs(n[0] - g_node[0]), abs(n[1] - g_node[1])
        return GRID_RES * (max(dx, dy) + (sq2 - 1) * min(dx, dy))

    g = {s: 0.0}
    parent = {s: None}
    heap = [(h(s), 0.0, s)]
    closed = set()
    moves = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]
    while heap:
        _, gc, n = heapq.heappop(heap)
        if n in closed:
            continue
        if n == g_node:
            break
        closed.add(n)
        for dx, dy in moves:
            m = (n[0] + dx, n[1] + dy)
            if not (0 <= m[0] < NXN and 0 <= m[1] < NYN) or m in closed:
                continue
            if B[m] and (not B[n] or C[m] <= C[n]):
                continue
            gm = gc + GRID_RES * (sq2 if dx and dy else 1.0)
            if gm < g.get(m, np.inf) - 1e-12:
                g[m] = gm
                parent[m] = n
                heapq.heappush(heap, (gm + h(m), -gm, m))
    if g_node not in parent:
        return None
    out, n = [], g_node
    while n is not None:
        out.append(np.array(n, dtype=float) * GRID_RES)
        n = parent[n]
    return out[::-1]


def line_of_sight(a, b, known):
    if len(known) == 0:
        return True
    K = np.asarray(known)
    r = R_INFLATE + LOS_MARGIN
    da = np.linalg.norm(K - a, axis=1)
    d = seg_point_dist(a, b, K)
    # Inside a disc (late detection): allowed only if the segment moves straight away from it
    escaping = (da < r) & (d >= da - 1e-9)
    return bool(np.all((d >= r) | escaping))


def plan(pos, known):
    """A* grid path from pos to the exit, then greedy line-of-sight shortcutting."""
    if len(known) == 0 or line_of_sight(pos, GOAL, known):
        return [pos.copy(), GOAL.copy()]
    nodes = astar(blocked_grid(known), pos, GOAL, clearance_grid(known))
    if nodes is None:
        return [pos.copy(), GOAL.copy()]
    pts = [pos.copy()] + nodes[1:]
    out, i = [pts[0]], 0
    while i < len(pts) - 1:
        j = i + 1
        while j + 1 < len(pts) and line_of_sight(pts[i], pts[j + 1], known):
            j += 1
        out.append(pts[j])
        i = j
    return out


# ── Scout route ───────────────────────────────────────────────────────────────

def scout_route(offsets):
    """Block-wise boustrophedon along the corridor; an odd strip count ends each block at its far end."""
    assert len(offsets) % 2 == 1
    wps = [np.array([0.0, Y_TRACK + offsets[0]])]
    n_blocks = int(np.ceil(FIELD_X / BLOCK_LEN))
    for k in range(n_blocks):
        x0, x1 = k * BLOCK_LEN, min((k + 1) * BLOCK_LEN, FIELD_X)
        order = offsets if k % 2 == 0 else offsets[::-1]
        for i, off in enumerate(order):
            y = Y_TRACK + off
            xa, xb = (x0, x1) if i % 2 == 0 else (x1, x0)
            if np.linalg.norm(wps[-1] - [xa, y]) > 1e-9:
                wps.append(np.array([xa, y]))
            wps.append(np.array([xb, y]))
    return np.array(wps)


# ── Simulation ────────────────────────────────────────────────────────────────

def make_world(seed):
    rng = np.random.default_rng(seed)
    obs = rng.uniform(OBS_LO, OBS_HI, size=(N_OBS, 2))
    noise = GPS_ERROR * rng.standard_normal((MAX_STEPS, 2))
    return obs, noise


def _polyline(points):
    """Cumulative arc length of a polyline (for np.interp position look-up)."""
    P = np.asarray(points, dtype=float)
    return P, np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(P, axis=0).T))])


def simulate(seed, pattern='1-pass', l_lead=L_LEAD, t_lat=T_LATENCY, r_sensor=R_SENSOR,
             oracle=False, keep=False):
    obs, noise = make_world(seed)
    ox, oy = obs[:, 0], obs[:, 1]
    known = [o for o in obs] if oracle else []
    harv = np.array([0.0, Y_TRACK])
    path = plan(harv, known)
    P_h, cum_h = _polyline(path)
    s_h = 0.0                                   # arc length driven along the current path
    if not oracle:
        R_d, cum_d = _polyline(scout_route(PATTERNS[pattern]))
        s_d, drone = 0.0, R_d[0].copy()
    else:
        drone = None
    route_done = oracle
    started = oracle
    t = 0.0
    r_det = r_sensor - GPS_ERROR
    detected = np.zeros(N_OBS, dtype=bool)
    t_detect = np.full(N_OBS, np.nan)
    relays = []                                 # (t_relay, j)
    relayed = np.zeros(N_OBS, dtype=bool)
    ttc = np.full(N_OBS, np.nan)
    collided = np.zeros(N_OBS, dtype=bool)
    min_clear = np.full(N_OBS, np.inf)
    driven, n_replan, t_start = 0.0, 0, 0.0 if oracle else np.nan
    log_h, log_d, log_t, replans = [harv.copy()], [drone.copy() if drone is not None else None], [0.0], []
    lead_log = []
    threat = np.abs(oy - Y_TRACK) < R_INFLATE

    for k in range(MAX_STEPS):
        # --- scout drone (moves along its route, never more than l_lead ahead in x) ---
        if not route_done:
            s_new = min(s_d + V_DRONE * DT, cum_d[-1])
            new = np.array([np.interp(s_new, cum_d, R_d[:, 0]), np.interp(s_new, cum_d, R_d[:, 1])])
            if new[0] <= harv[0] + l_lead + 1e-9 or new[0] <= drone[0]:
                assert np.hypot(*(new - drone)) <= V_DRONE * DT + 1e-9, 'drone teleport'
                drone, s_d = new, s_new
                route_done = s_d >= cum_d[-1] - 1e-9
            mx, my = drone + noise[k]
            hit = (~detected) & (np.hypot(ox - mx, oy - my) <= r_det)
            if hit.any():
                for j in np.where(hit)[0]:
                    detected[j] = True
                    t_detect[j] = t
                    relays.append((t + t_lat, int(j)))
            if not started and (drone[0] >= l_lead - V_DRONE * DT or route_done):
                started, t_start = True, t
        # --- relayed detections reach the harvester ---
        if relays and min(r[0] for r in relays) <= t + 1e-9:
            due = [r for r in relays if r[0] <= t + 1e-9]
            relays = [r for r in relays if r[0] > t + 1e-9]
            replan = False
            ahead = [harv] + [p for p, c in zip(P_h, cum_h) if c > s_h]
            for _, j in due:
                relayed[j] = True
                known.append(obs[j])
                ttc[j] = (obs[j, 0] - R_INFLATE - harv[0]) / V_HARV
                if path_obstacle_dist(ahead, obs[j]) < R_INFLATE + LOS_MARGIN:
                    replan = True
            if replan:
                path = plan(harv, known)
                P_h, cum_h = _polyline(path)
                s_h = 0.0
                n_replan += 1
                replans.append((t, harv.copy()))
        # --- harvester (constant speed along its current path) ---
        if started:
            prev = harv
            s_h = min(s_h + V_HARV * DT, cum_h[-1])
            harv = np.array([np.interp(s_h, cum_h, P_h[:, 0]), np.interp(s_h, cum_h, P_h[:, 1])])
            step = np.hypot(*(harv - prev))
            driven += step
            assert step <= V_HARV * DT + 1e-9, 'harvester teleport'
            assert 0.0 <= harv[1] <= FIELD_Y, 'harvester left the field'
            dist = np.hypot(ox - harv[0], oy - harv[1])
            np.minimum(min_clear, dist, out=min_clear)
            collided |= dist < R_INFLATE
            if not oracle and not route_done:
                lead_log.append((t, drone[0] - harv[0]))
        t += DT
        if keep:
            log_h.append(harv.copy()); log_d.append(drone.copy() if drone is not None else None)
            log_t.append(t)
        if started and s_h >= cum_h[-1] - 1e-9:
            break
    assert s_h >= cum_h[-1] - 1e-9, 'harvester did not reach the exit'

    cls = np.full(N_OBS, 'missed', dtype=object)
    cls[relayed & (ttc > T_REPLAN)] = 'timely'
    cls[relayed & (ttc > 0) & (ttc <= T_REPLAN)] = 'marginal'
    cls[relayed & (ttc <= 0)] = 'late'
    detected_never_relayed = detected & ~relayed       # relay still pending at mission end
    cls[detected_never_relayed] = 'late'
    lead = np.array(lead_log) if lead_log else np.zeros((0, 2))
    res = dict(seed=seed, pattern='oracle' if oracle else pattern, obs=obs, threat=threat,
               cls=cls, collided=collided, n_coll=int(collided.sum()),
               n_coll_threat=int((collided & threat).sum()), n_det=int(detected.sum()),
               n_replan=n_replan, extra=driven - FIELD_X, t_start=t_start,
               t_mission=t, min_clear=min_clear,
               min_lead=float(lead[:, 1].min()) if len(lead) else np.nan,
               t_detect=t_detect, ttc=ttc)
    if keep:
        res.update(h=np.array(log_h), d=(np.array(log_d) if not oracle else None),
                   t=np.array(log_t), replans=replans, lead=lead, known=np.array(known),
                   path=np.array(path))
    return res


def run_simulation():
    main = {p: simulate(SEED, p, keep=True) for p in PATTERNS}
    main['oracle'] = simulate(SEED, oracle=True, keep=True)
    mc = {p: [simulate(s, p) for s in range(N_MC)] for p in PATTERNS}
    mc['oracle'] = [simulate(s, oracle=True) for s in range(N_MC)]
    # Sweep points equal to the card's value reuse the main Monte Carlo runs (same seeds = CRN)
    sweeps = {
        'lead': {p: [mc[p][:N_SWEEP] if L == L_LEAD else
                     [simulate(s, p, l_lead=L) for s in range(N_SWEEP)] for L in LEAD_SWEEP]
                 for p in PATTERNS},
        'latency': [mc['1-pass'][:N_SWEEP] if tl == T_LATENCY else
                    [simulate(s, '1-pass', t_lat=tl) for s in range(N_SWEEP)]
                    for tl in LATENCY_SWEEP],
        'sensor': [mc['1-pass'][:N_SWEEP] if r == R_SENSOR else
                   [simulate(s, '1-pass', r_sensor=r) for s in range(N_SWEEP)]
                   for r in SENSOR_SWEEP],
    }
    return main, mc, sweeps


# ── Summaries ─────────────────────────────────────────────────────────────────

def summarize(runs):
    th = np.concatenate([r['threat'] for r in runs])
    cl = np.concatenate([r['cls'] for r in runs])
    coll = np.array([r['n_coll'] for r in runs])
    ex = np.array([r['extra'] for r in runs])
    return dict(n_threat=int(th.sum()),
                frac={c: float(np.mean(cl[th] == c)) if th.sum() else np.nan
                      for c in ('timely', 'marginal', 'late', 'missed')},
                coll_mean=coll.mean(), coll_std=coll.std(), runs_with_coll=int(np.sum(coll > 0)),
                coll_total=int(coll.sum()),
                coll_threat=int(sum(r['n_coll_threat'] for r in runs)),
                extra_mean=ex.mean(), extra_std=ex.std(),
                det_mean=float(np.mean([r['n_det'] for r in runs])),
                replan_mean=float(np.mean([r['n_replan'] for r in runs])),
                t_start=float(np.mean([r['t_start'] for r in runs])),
                t_mission=float(np.mean([r['t_mission'] for r in runs])),
                min_lead=float(np.mean([r['min_lead'] for r in runs])),
                all_cls={c: int(np.sum(cl == c)) for c in ('timely', 'marginal', 'late')})


# ── Plotting ──────────────────────────────────────────────────────────────────

def _save(fig, out_dir, name, dpi=140):
    path = os.path.join(out_dir, name)
    fig.savefig(path, dpi=dpi, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def _field(ax, r, show_drone=True):
    ax.add_patch(Rectangle((0, 0), FIELD_X, FIELD_Y, fc='#F5F0DC', ec='k'))
    ax.add_patch(Rectangle((0, Y_TRACK - CORRIDOR_W / 2), FIELD_X, CORRIDOR_W, fc='none',
                           ec='k', ls=':', lw=0.8))
    for j, o in enumerate(r['obs']):
        det = r['cls'][j] != 'missed' or r['pattern'] == 'oracle'
        ax.add_patch(Circle(o, R_OBS, color='darkorange' if det else OBS_COLOR, zorder=3))
        if r['collided'][j]:
            ax.plot(*o, 'kx', ms=10, mew=2, zorder=4)
    if show_drone and r.get('d') is not None:
        ax.plot(r['d'][:, 0], r['d'][:, 1], color=DRONE_COLOR, lw=0.5, alpha=0.7, label='scout')
    ax.plot(r['h'][:, 0], r['h'][:, 1], color=HARV_COLOR, lw=1.8, label='harvester')
    for _, p in r['replans']:
        ax.plot(*p, '^', color=HARV_COLOR, mec='k', ms=7, zorder=5)
    ax.set_xlim(-2, FIELD_X + 2); ax.set_ylim(-2, FIELD_Y + 2)
    ax.set_aspect('equal')


def plot_field_overview(main, out_dir):
    fig, axes = plt.subplots(3, 1, figsize=(14, 12))
    for ax, p in zip(axes, ['3-pass', '1-pass', 'oracle']):
        r = main[p]
        _field(ax, r)
        ax.set_title(f"{LABELS[p]} (seed {SEED}): {r['n_coll']} collision(s), "
                     f"{r['n_replan']} replan(s), extra path {r['extra']:.2f} m, "
                     f"mission {r['t_mission']:.1f} s", fontsize=10)
        ax.set_ylabel('y (m)')
    axes[-1].set_xlabel('x (m)')
    axes[0].legend(loc='upper right', fontsize=8)
    fig.suptitle('S078 Field Overview: orange = detected/known obstacle, grey = never detected, '
                 'black × = collision, ▲ = replan, dotted = 10 m corridor', fontsize=11)
    plt.tight_layout()
    _save(fig, out_dir, 'field_overview.png')


def plot_lead_timeline(main, out_dir):
    fig, ax = plt.subplots(figsize=(10, 4.5))
    for p in PATTERNS:
        lead = main[p]['lead']
        ax.plot(lead[:, 0], lead[:, 1], color=COLORS[p], label=f'{LABELS[p]}')
        ax.axvline(main[p]['t_start'], color=COLORS[p], ls=':', lw=1)
    ax.axhline(0, color='k', lw=0.8)
    ax.axhline(L_LEAD, color='grey', ls='--', lw=1, label=f'$L_{{lead}}$ = {L_LEAD:.0f} m cap')
    ax.set_xlabel('Time (s); dotted = harvester start')
    ax.set_ylabel('Scout x − harvester x (m)')
    ax.set_title(f'S078 Scout Lead over Time (seed {SEED}); below 0 = harvester ahead of scout')
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)
    _save(fig, out_dir, 'lead_timeline.png')


def plot_detection_classes(summ, out_dir):
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(13, 4.8))
    keys = ['3-pass', '1-pass']
    cols = {'timely': 'green', 'marginal': 'gold', 'late': 'red', 'missed': 'grey'}
    bottom = np.zeros(2)
    for c in ('timely', 'marginal', 'late', 'missed'):
        v = np.array([100 * summ[k]['frac'][c] for k in keys])
        a1.bar([LABELS[k] for k in keys], v, bottom=bottom, color=cols[c], edgecolor='k',
               label=c)
        for i, (b, vv) in enumerate(zip(bottom, v)):
            if vv > 4:
                a1.text(i, b + vv / 2, f'{vv:.1f}%', ha='center', va='center', fontsize=9)
        bottom += v
    a1.set_ylabel('% of on-track obstacles')
    a1.set_title(f"On-track obstacles (|y − 25| < {R_INFLATE} m), {N_MC} seeds", fontsize=10)
    a1.legend(fontsize=8, loc='lower right')
    keys3 = ['oracle', '1-pass', '3-pass']
    a2.bar([LABELS[k] for k in keys3], [summ[k]['coll_mean'] for k in keys3],
           yerr=[summ[k]['coll_std'] for k in keys3], capsize=5,
           color=[COLORS[k] for k in keys3], edgecolor='k')
    for i, k in enumerate(keys3):
        a2.text(i, summ[k]['coll_mean'] + summ[k]['coll_std'] + 0.02,
                f"{summ[k]['coll_mean']:.2f}\n({summ[k]['runs_with_coll']}/{N_MC} runs)",
                ha='center', fontsize=9)
    a2.set_ylabel('Collisions per run (mean ± std)')
    a2.set_ylim(0, 1.25 * max(summ[k]['coll_mean'] + summ[k]['coll_std'] for k in keys3) + 0.2)
    a2.set_title('Collisions (harvester within 2.5 m of an obstacle centre)', fontsize=10)
    a2.tick_params(axis='x', labelsize=8)
    fig.suptitle('S078 Detection Timing and Safety')
    plt.tight_layout()
    _save(fig, out_dir, 'detection_classes.png')


def plot_extra_path(mc, out_dir):
    fig, ax = plt.subplots(figsize=(9, 4.8))
    keys = ['oracle', '1-pass', '3-pass']
    data = [[r['extra'] for r in mc[k]] for k in keys]
    bp = ax.boxplot(data, tick_labels=[LABELS[k] for k in keys], patch_artist=True, showmeans=True)
    for patch, k in zip(bp['boxes'], keys):
        patch.set_facecolor(COLORS[k]); patch.set_alpha(0.6)
    ax.set_ylabel('Extra path vs 200 m straight line (m)')
    ax.set_title(f'S078 Detour Cost per Run ({N_MC} seeds; triangle = mean)')
    ax.grid(alpha=0.3, axis='y')
    _save(fig, out_dir, 'extra_path.png')


def plot_occupancy(main, out_dir):
    r = main['1-pass']
    B = blocked_grid(r['known'])
    fig, ax = plt.subplots(figsize=(14, 4.2))
    ax.imshow(B.T, origin='lower', extent=[0, FIELD_X, 0, FIELD_Y], cmap='Greys', alpha=0.6,
              aspect='equal')
    for j, o in enumerate(r['obs']):
        known = r['cls'][j] != 'missed'
        ax.plot(*o, '+' if known else 'x', color='c' if known else 'grey', ms=9, mew=2)
    ax.plot(r['h'][:, 0], r['h'][:, 1], color=HARV_COLOR, lw=1.5)
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')
    ax.set_title(f'S078 Final Known Occupancy Grid, 1-pass (seed {SEED}): grey = node blocked '
                 f'(inflation {R_INFLATE} m + {GRID_MARGIN} m), cyan + = known, grey × = never '
                 f'detected', fontsize=10)
    _save(fig, out_dir, 'occupancy_grid.png')


def plot_sweeps(sweeps, out_dir):
    fig, axes = plt.subplots(1, 3, figsize=(18, 4.8))
    ax = axes[0]
    for p in PATTERNS:
        s = [summarize(rs) for rs in sweeps['lead'][p]]
        ax.plot(LEAD_SWEEP, [x['coll_mean'] for x in s], 'o-', color=COLORS[p],
                label=f'{LABELS[p]}: collisions/run')
        ax.plot(LEAD_SWEEP, [x['t_start'] / 60 for x in s], 's--', color=COLORS[p], alpha=0.6,
                label=f'{LABELS[p]}: start delay (min)')
    ax.set_xlabel('$L_{lead}$ (m)')
    ax.set_title('Lead distance', fontsize=10)
    ax.legend(fontsize=7); ax.grid(alpha=0.3)
    ax = axes[1]
    s = [summarize(rs) for rs in sweeps['latency']]
    ax.plot(LATENCY_SWEEP, [x['extra_mean'] for x in s], 'o-', color=COLORS['1-pass'],
            label='extra path (m, left)')
    ax.set_ylabel('Extra path (m)')
    ax.set_ylim(0, 0.5)
    axr = ax.twinx()
    axr.plot(LATENCY_SWEEP, [100 * x['frac']['timely'] for x in s], 's--', color='green',
             label='timely on-track detections (%, right)')
    axr.plot(LATENCY_SWEEP, [x['coll_mean'] * 100 for x in s], '^:', color=HARV_COLOR,
             label='collisions per 100 runs (right)')
    axr.set_ylim(0, 105)
    ax.set_xlabel('$T_{latency}$ (s)')
    ax.set_title('Latency, 1-pass', fontsize=10)
    h1, l1 = ax.get_legend_handles_labels(); h2, l2 = axr.get_legend_handles_labels()
    ax.legend(h1 + h2, l1 + l2, fontsize=7, loc='center right'); ax.grid(alpha=0.3)
    ax = axes[2]
    s = [summarize(rs) for rs in sweeps['sensor']]
    ax.plot(SENSOR_SWEEP, [x['coll_mean'] for x in s], 'o-', color=HARV_COLOR,
            label='collisions/run')
    ax.plot(SENSOR_SWEEP, [x['frac']['missed'] for x in s], 's--', color='grey',
            label='missed on-track fraction')
    ax.axvline(R_INFLATE + GPS_ERROR, color='k', ls=':', lw=1)
    ax.set_xlabel('$r_{sensor}$ (m); dotted = 2.6 m')
    ax.set_title('Sensor range, 1-pass', fontsize=10)
    ax.legend(fontsize=7); ax.grid(alpha=0.3)
    fig.suptitle(f'S078 Parameter Sweeps ({N_SWEEP} seeds per point, common random numbers)')
    plt.tight_layout()
    _save(fig, out_dir, 'sweeps.png')


def save_animation(main, out_dir):
    keys = ['3-pass', '1-pass']
    fig, axes = plt.subplots(2, 1, figsize=(12, 7))
    n = max(len(main[k]['t']) for k in keys)
    n_frames = int(np.ceil((n - 1) / ANIM_STEP)) + 1
    art = {}
    for ax, k in zip(axes, keys):
        r = main[k]
        ax.add_patch(Rectangle((0, 0), FIELD_X, FIELD_Y, fc='#F5F0DC', ec='k'))
        circ = [ax.add_patch(Circle(o, R_OBS, color=OBS_COLOR, zorder=3)) for o in r['obs']]
        hl, = ax.plot([], [], color=HARV_COLOR, lw=1.8)
        dl, = ax.plot([], [], color=DRONE_COLOR, lw=0.5, alpha=0.7)
        hd, = ax.plot([], [], 's', color=HARV_COLOR, mec='k', ms=8)
        dd, = ax.plot([], [], '^', color=DRONE_COLOR, mec='k', ms=8)
        txt = ax.text(2, FIELD_Y - 4, '', fontsize=9)
        ax.set_xlim(-2, FIELD_X + 2); ax.set_ylim(-2, FIELD_Y + 2); ax.set_aspect('equal')
        ax.set_title(LABELS[k], fontsize=10)
        art[k] = (circ, hl, dl, hd, dd, txt)

    def update(f):
        for k in keys:
            r = main[k]
            i = min(f * ANIM_STEP, len(r['t']) - 1)
            circ, hl, dl, hd, dd, txt = art[k]
            tt = r['t'][i]
            for j, c in enumerate(circ):
                relay_t = r['t_detect'][j] + T_LATENCY
                if r['collided'][j] and np.min(np.linalg.norm(r['h'][:i + 1] - r['obs'][j],
                                                              axis=1)) < R_INFLATE:
                    c.set_color('black')
                elif not np.isnan(relay_t) and tt >= relay_t:
                    c.set_color('darkorange')
            hl.set_data(r['h'][:i + 1, 0], r['h'][:i + 1, 1])
            dl.set_data(r['d'][:i + 1, 0], r['d'][:i + 1, 1])
            hd.set_data([r['h'][i, 0]], [r['h'][i, 1]])
            dd.set_data([r['d'][i, 0]], [r['d'][i, 1]])
            nrep = sum(1 for tr, _ in r['replans'] if tr <= tt)
            txt.set_text(f't = {tt:.0f} s, replans {nrep}')
        fig.suptitle(f'S078 seed {SEED}: red = harvester, blue = scout, orange = relayed '
                     f'obstacle, black = collision', fontsize=10)
        return []

    anim = animation.FuncAnimation(fig, update, frames=n_frames, interval=80, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    anim.save(path, writer='pillow', fps=12, dpi=70)
    plt.close(fig)
    print(f'Saved: {path} ({n_frames} frames)')


# ── Main ──────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    main, mc, sweeps = run_simulation()
    summ = {k: summarize(v) for k, v in mc.items()}
    print('=== S078 Harvester Cooperative Guidance — Results ===')
    for p, offs in PATTERNS.items():
        route = scout_route(offs)
        L = np.linalg.norm(np.diff(route, axis=0), axis=1).sum()
        print(f'  {LABELS[p]}: route {L:.1f} m for {FIELD_X:.0f} m of corridor -> max advance '
              f'speed {V_DRONE * FIELD_X / L:.3f} m/s (harvester {V_HARV} m/s); covered half-width '
              f'{max(offs) + R_SENSOR - GPS_ERROR:.2f} m')
    print(f'  Area check: scout sweep rate 2 (r_sensor - sigma) v_drone = '
          f'{2 * (R_SENSOR - GPS_ERROR) * V_DRONE:.1f} m^2/s vs corridor demand '
          f'{CORRIDOR_W * V_HARV:.1f} m^2/s')
    for p in ['3-pass', '1-pass', 'oracle']:
        r = main[p]
        print(f'  Seed {SEED} [{LABELS[p]}]: detected {r["n_det"]}/{N_OBS}, on-track obstacles '
              f'{int(r["threat"].sum())}, collisions {r["n_coll"]}, replans {r["n_replan"]}, '
              f'extra path {r["extra"]:.2f} m, start {r["t_start"]:.1f} s, mission '
              f'{r["t_mission"]:.1f} s, min lead {r["min_lead"]:.1f} m')
    print(f'  --- Monte Carlo, {N_MC} seeds (same layouts / noise for every configuration) ---')
    for k in ['3-pass', '1-pass', 'oracle']:
        s = summ[k]
        print(f'  [{LABELS[k]}] collisions/run {s["coll_mean"]:.2f} ± {s["coll_std"]:.2f} '
              f'({s["runs_with_coll"]}/{N_MC} runs, {s["coll_total"]} total, '
              f'{s["coll_threat"]} on-track); extra path {s["extra_mean"]:.2f} ± '
              f'{s["extra_std"]:.2f} m; detected {s["det_mean"]:.2f}/{N_OBS}; replans '
              f'{s["replan_mean"]:.2f}; start {s["t_start"]:.1f} s; mission {s["t_mission"]:.1f} s; '
              f'min lead {s["min_lead"]:.1f} m')
        if k != 'oracle':
            print(f'      on-track obstacles {s["n_threat"]}: ' + ', '.join(
                f'{c} {100 * v:.1f}%' for c, v in s['frac'].items())
                + f"; all relayed: timely {s['all_cls']['timely']}, marginal "
                  f"{s['all_cls']['marginal']}, late {s['all_cls']['late']}")
    c3 = np.array([r['n_coll'] for r in mc['3-pass']])
    c1 = np.array([r['n_coll'] for r in mc['1-pass']])
    print(f'  Paired collisions 1-pass vs 3-pass: 1-pass fewer in {int(np.sum(c1 < c3))}, '
          f'equal in {int(np.sum(c1 == c3))}, more in {int(np.sum(c1 > c3))} of {N_MC} seeds')
    e1 = np.array([r['extra'] for r in mc['1-pass']])
    eo = np.array([r['extra'] for r in mc['oracle']])
    print(f'  Extra path 1-pass minus oracle: {np.mean(e1 - eo):.2f} ± {np.std(e1 - eo):.2f} m')
    print('  Lead sweep (L: collisions/run, start delay s, min lead m):')
    for p in PATTERNS:
        for L, rs in zip(LEAD_SWEEP, sweeps['lead'][p]):
            s = summarize(rs)
            print(f'    {p} L={L:.0f}: {s["coll_mean"]:.2f}, {s["t_start"]:.1f}, '
                  f'{s["min_lead"]:.1f}, timely {100 * s["frac"]["timely"]:.1f}%')
    print('  Latency sweep, 1-pass (T: extra path m, collisions/run, timely %, late %):')
    base = None
    for tl, rs in zip(LATENCY_SWEEP, sweeps['latency']):
        s = summarize(rs)
        ex = np.array([r['extra'] for r in rs])
        base = ex if base is None else base
        print(f'    {tl:.0f} s: {s["extra_mean"]:.3f} ± {s["extra_std"]:.3f} (paired vs 0 s '
              f'{np.mean(ex - base):+.3f}), {s["coll_mean"]:.2f}, '
              f'{100 * s["frac"]["timely"]:.1f}%, {100 * s["frac"]["late"]:.1f}%')
    print('  Sensor sweep, 1-pass (r: collisions/run, missed on-track %):')
    for rr, rs in zip(SENSOR_SWEEP, sweeps['sensor']):
        s = summarize(rs)
        print(f'    {rr:.1f} m: {s["coll_mean"]:.2f} ({s["runs_with_coll"]}/{N_SWEEP} runs), '
              f'missed {100 * s["frac"]["missed"]:.1f}% of {s["n_threat"]}')

    out = os.path.normpath(OUTPUT_DIR)
    os.makedirs(out, exist_ok=True)
    plot_field_overview(main, out)
    plot_lead_timeline(main, out)
    plot_detection_classes(summ, out)
    plot_extra_path(mc, out)
    plot_occupancy(main, out)
    plot_sweeps(sweeps, out)
    save_animation(main, out)
