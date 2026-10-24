"""
S078 3D Harvester Guidance on Sloped Terrain with a Crop Canopy
===============================================================
The 2D S078 runs on a flat field with the scout at an implicit fixed height and a 2D
detection disc. Here the 200 x 50 m field has a sinusoidal DEM and a crop canopy
(0.3-1.2 m) and the obstacles are 0.5 m above the ground, so the scout's detection is a
3D sphere (r = 2.5 m) and its flight altitude decides what it can see: the higher above
the obstacle tops it flies, the smaller its horizontal detection radius
r_xy = sqrt(r^2 - dz^2). The harvester's speed follows the slope along its heading and its
A* edge costs are terrain arc lengths.

Scout altitude plans (same 1-pass centreline route as the 2D base's safer pattern,
3.5 m/s, same obstacle layouts and GPS noise for every plan):
  fixed_msl  the 2D base at one altitude: the lowest constant MSL altitude that clears
             every canopy top by h_ACL (2D baseline)
  fixed_low  the same with the low clearance c_min (2D baseline, added)
  acl_card   card: constant 2.0 m above the local canopy (terrain + crop + h_ACL)
  acl_low    added: the lowest safe clearance above the canopy (c_min = 0.5 m)
  oracle     all obstacles known at the start (reference)
Monte Carlo over obstacle layouts with common random numbers, a clearance sweep, and a
DEM-amplitude sweep.

Usage:
    conda activate drones
    python src/04_industrial_agriculture/3d/s078_3d_harvester_guidance.py
"""

import os
import heapq
from functools import lru_cache
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Circle, Rectangle
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, '..', '..', '..'))
OUTPUT_DIR = os.path.join(PROJECT_ROOT, 'outputs', '04_industrial_agriculture', '3d',
                          's078_3d_harvester_guidance')

# ── Parameters (card) ─────────────────────────────────────────────────────────
FIELD_X, FIELD_Y = 200.0, 50.0
A_DEM, L_DEM_X, L_DEM_Y = 2.0, 200.0, 100.0
H_ACL = 2.0
V0_HARV = 1.5
PHI_REF = np.deg2rad(3.0)
K_SLOPE = 5.0
ETA_MIN, ETA_MAX = 0.4, 1.3
V_DRONE = 3.5
VZ_MAX = 1.5
KP_Z = 2.0
R_SENSOR = 2.5
H_OBS = 0.5
L_LEAD = 45.0
R_INFLATE = 2.5
GRID_RES = 0.5
T_LATENCY = 2.0
GPS_ERROR = 0.1
N_OBS = 15
DT = 0.1
Z_BOUNDS = (0.5, 8.0)
# Kept from the 2D base
R_OBS, R_HARV = 1.0, 1.5
Y_TRACK = FIELD_Y / 2
OBS_LO, OBS_HI = np.array([20.0, 5.0]), np.array([FIELD_X - 20.0, FIELD_Y - 5.0])
T_REPLAN = 5.0
GRID_MARGIN = GRID_RES / 2
LOS_MARGIN = 0.05
MAX_STEPS = 6000
SEED = 42

# ── Added parameters (see README Deviations) ──────────────────────────────────
C_MIN = 0.5                      # m, lowest safe clearance above the canopy (acl_low)
C_SWEEP = [0.3, 0.5, 1.0, 1.5, 2.0, 2.5]
A_SWEEP = [0.0, 1.0, 2.0, 3.0, 4.0]
N_MC = 40
N_SWEEP = 15
ANIM_STEP = 25

PLANS = ['fixed_msl', 'fixed_low', 'acl_card', 'acl_low']
LABELS = {'fixed_msl': 'Fixed MSL, 2.0 m over top (2D)', 'fixed_low': f'Fixed MSL, {C_MIN} m over top (2D)',
          'acl_card': 'ACL 2.0 m (card)',
          'acl_low': f'Lowest safe clearance ({C_MIN} m)', 'oracle': 'Oracle (all known)'}
COLORS = {'fixed_msl': 'tab:grey', 'fixed_low': 'tab:olive', 'acl_card': 'tab:purple', 'acl_low': 'tab:blue', 'oracle': 'k'}

NXN, NYN = int(round(FIELD_X / GRID_RES)) + 1, int(round(FIELD_Y / GRID_RES)) + 1
GOAL = np.array([FIELD_X, Y_TRACK])
GX = np.arange(NXN) * GRID_RES
GY = np.arange(NYN) * GRID_RES


# ── Terrain, crop ─────────────────────────────────────────────────────────────
def terrain(x, y, a=A_DEM):
    """Card DEM lifted by A_dem so that its minimum is 0 m (the altitude bounds are MSL)."""
    return a + a * (np.sin(2 * np.pi * x / L_DEM_X) + 0.5 * np.sin(2 * np.pi * y / L_DEM_Y))


def grad(x, y, a=A_DEM):
    return (a * 2 * np.pi / L_DEM_X * np.cos(2 * np.pi * x / L_DEM_X),
            a * 0.5 * 2 * np.pi / L_DEM_Y * np.cos(2 * np.pi * y / L_DEM_Y))


def crop(x, y):
    return 0.3 + 0.9 * np.exp(-(x - 100.0) ** 2 / (2 * 60.0 ** 2) - (y - 25.0) ** 2 / (2 * 15.0 ** 2))


def eta_slope(phi):
    """phi: signed slope along the heading (positive uphill)."""
    if phi > PHI_REF:
        return max(ETA_MIN, 1.0 - K_SLOPE * (phi - PHI_REF))
    if phi < -PHI_REF:
        return min(ETA_MAX, 1.0 - K_SLOPE * (phi + PHI_REF))
    return 1.0


def harv_speed(p, u, a):
    gx, gy = grad(p[0], p[1], a)
    phi = np.arctan(gx * u[0] + gy * u[1])
    return V0_HARV * np.cos(phi) * eta_slope(phi), phi


@lru_cache(maxsize=None)
def z_fixed(a=A_DEM, margin=H_ACL):
    """Lowest constant MSL altitude that clears the highest canopy top by margin."""
    X, Y = np.meshgrid(np.linspace(0, FIELD_X, 801), np.linspace(0, FIELD_Y, 201))
    return float((terrain(X, Y, a) + crop(X, Y)).max() + margin)


def z_command(plan, x, y, a, c_min=C_MIN):
    if plan == 'fixed_msl':
        return z_fixed(a)
    if plan == 'fixed_low':
        return z_fixed(a, c_min)
    top = terrain(x, y, a) + crop(x, y)
    return top + (H_ACL if plan == 'acl_card' else c_min)


# ── Planner (2D base A*, terrain arc-length edge costs) ───────────────────────
def seg_point_dist(a, b, c):
    ab = b - a
    L2 = float(ab @ ab)
    if L2 < 1e-12:
        return np.linalg.norm(c - a, axis=1)
    t = np.clip((c - a) @ ab / L2, 0.0, 1.0)
    return np.linalg.norm(a + t[:, None] * ab - c, axis=1)


def path_obstacle_dist(path, c):
    return min(float(seg_point_dist(a, b, c[None])[0]) for a, b in zip(path[:-1], path[1:]))


def blocked_and_clear(known):
    X, Y = np.meshgrid(GX, GY, indexing='ij')
    C = np.full((NXN, NYN), np.inf)
    for c in known:
        C = np.minimum(C, np.hypot(X - c[0], Y - c[1]))
    return C <= R_INFLATE + GRID_MARGIN, C


def sec_grid(a):
    X, Y = np.meshgrid(GX, GY, indexing='ij')
    gx, gy = grad(X, Y, a)
    return np.sqrt(1 + gx ** 2 + gy ** 2)          # 1 / cos(phi)


def astar(B, C, S, start, goal):
    s = (min(max(int(round(start[0] / GRID_RES)), 0), NXN - 1),
         min(max(int(round(start[1] / GRID_RES)), 0), NYN - 1))
    g_node = (int(round(goal[0] / GRID_RES)), int(round(goal[1] / GRID_RES)))
    sq2 = np.sqrt(2.0)

    def h(n):
        dx, dy = abs(n[0] - g_node[0]), abs(n[1] - g_node[1])
        return GRID_RES * (max(dx, dy) + (sq2 - 1) * min(dx, dy))   # admissible (sec >= 1)

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
            gm = gc + GRID_RES * (sq2 if dx and dy else 1.0) * S[m]
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
    escaping = (da < r) & (d >= da - 1e-9)
    return bool(np.all((d >= r) | escaping))


def plan_path(pos, known, S):
    if len(known) == 0 or line_of_sight(pos, GOAL, known):
        return [pos.copy(), GOAL.copy()]
    B, C = blocked_and_clear(known)
    nodes = astar(B, C, S, pos, GOAL)
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


def _polyline(points):
    P = np.asarray(points, dtype=float)
    return P, np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(P, axis=0).T))])


# ── Simulation ────────────────────────────────────────────────────────────────
def make_world(seed):
    rng = np.random.default_rng(seed)
    obs = rng.uniform(OBS_LO, OBS_HI, size=(N_OBS, 2))
    noise = GPS_ERROR * rng.standard_normal((MAX_STEPS, 2))
    return obs, noise


def simulate(seed, plan='acl_card', a=A_DEM, c_min=C_MIN, oracle=False, keep=False, S=None):
    obs, noise = make_world(seed)
    ox, oy = obs[:, 0], obs[:, 1]
    oz = terrain(ox, oy, a) + H_OBS
    S = sec_grid(a) if S is None else S
    known = [o for o in obs] if oracle else []
    harv = np.array([0.0, Y_TRACK])
    path = plan_path(harv, known, S)
    P_h, cum_h = _polyline(path)
    s_h = 0.0
    drone = np.array([0.0, Y_TRACK])
    dz = z_command(plan, drone[0], drone[1], a, c_min)
    vz = 0.0
    route_done = oracle
    started = oracle
    t, t_start = 0.0, (0.0 if oracle else np.nan)
    r_det = R_SENSOR - GPS_ERROR
    detected = np.zeros(N_OBS, bool)
    relays, relayed = [], np.zeros(N_OBS, bool)
    ttc = np.full(N_OBS, np.nan)
    collided = np.zeros(N_OBS, bool)
    driven, driven3, n_replan = 0.0, 0.0, 0
    canopy_clear, dz_obs_det, vzmax = np.inf, [], 0.0
    log = dict(t=[0.0], h=[harv.copy()], d=[np.r_[drone, dz]], v=[0.0])
    threat = np.abs(oy - Y_TRACK) < R_INFLATE
    min_lead = np.inf
    zmax_seen = -np.inf
    for k in range(MAX_STEPS):
        if not route_done:
            nx = min(drone[0] + V_DRONE * DT, FIELD_X)
            if nx <= harv[0] + L_LEAD + 1e-9:
                drone = np.array([nx, Y_TRACK])
            route_done = drone[0] >= FIELD_X - 1e-9
            # altitude: feed-forward of the command rate + P correction, rate-limited
            zc = z_command(plan, drone[0], drone[1], a, c_min)
            zc_next = z_command(plan, min(drone[0] + V_DRONE * DT, FIELD_X), drone[1], a, c_min)
            vz = float(np.clip((zc_next - zc) / DT + KP_Z * (zc - dz), -VZ_MAX, VZ_MAX))
            dz += vz * DT
            vzmax = max(vzmax, abs(vz))
            ground = float(terrain(drone[0], drone[1], a))
            assert dz > ground, 'scout below ground'
            assert dz >= Z_BOUNDS[0] - 1e-9, 'below the lower altitude bound'
            zmax_seen = max(zmax_seen, dz)
            canopy_clear = min(canopy_clear, dz - ground - float(crop(drone[0], drone[1])))
            mx, my = drone + noise[k]
            d3 = np.sqrt((ox - mx) ** 2 + (oy - my) ** 2 + (oz - dz) ** 2)
            hit = (~detected) & (d3 <= r_det)
            for j in np.flatnonzero(hit):
                detected[j] = True
                dz_obs_det.append(dz - oz[j])
                relays.append((t + T_LATENCY, int(j)))
            if not started and (drone[0] >= L_LEAD - V_DRONE * DT or route_done):
                started, t_start = True, t
        if relays and min(r[0] for r in relays) <= t + 1e-9:
            due = [r for r in relays if r[0] <= t + 1e-9]
            relays = [r for r in relays if r[0] > t + 1e-9]
            replan = False
            ahead = [harv] + [p for p, c in zip(P_h, cum_h) if c > s_h]
            for _, j in due:
                relayed[j] = True
                known.append(obs[j])
                ttc[j] = (obs[j, 0] - R_INFLATE - harv[0]) / V0_HARV
                if path_obstacle_dist(ahead, obs[j]) < R_INFLATE + LOS_MARGIN:
                    replan = True
            if replan:
                path = plan_path(harv, known, S)
                P_h, cum_h = _polyline(path)
                s_h = 0.0
                n_replan += 1
        v = 0.0
        if started:
            prev = harv
            j = min(int(np.searchsorted(cum_h, s_h, side='right')) - 1, len(P_h) - 2)
            seg = P_h[j + 1] - P_h[j]
            u = seg / max(np.hypot(*seg), 1e-12)
            v, _ = harv_speed(harv, u, a)
            s_h = min(s_h + v * DT, cum_h[-1])
            harv = np.array([np.interp(s_h, cum_h, P_h[:, 0]), np.interp(s_h, cum_h, P_h[:, 1])])
            step = np.hypot(*(harv - prev))
            assert step <= ETA_MAX * V0_HARV * DT + 1e-9, 'harvester teleport'
            assert 0.0 <= harv[1] <= FIELD_Y, 'harvester left the field'
            driven += step
            driven3 += np.hypot(step, terrain(harv[0], harv[1], a) - terrain(prev[0], prev[1], a))
            collided |= np.hypot(ox - harv[0], oy - harv[1]) < R_INFLATE
            if not route_done:
                min_lead = min(min_lead, drone[0] - harv[0])
        t += DT
        if keep:
            log['t'].append(t)
            log['h'].append(harv.copy())
            log['d'].append(np.r_[drone, dz])
            log['v'].append(v)
        if started and s_h >= cum_h[-1] - 1e-9:
            break
    assert s_h >= cum_h[-1] - 1e-9, 'harvester did not reach the exit'
    cls = np.full(N_OBS, 'missed', dtype=object)
    cls[relayed & (ttc > T_REPLAN)] = 'timely'
    cls[relayed & (ttc > 0) & (ttc <= T_REPLAN)] = 'marginal'
    cls[relayed & (ttc <= 0)] = 'late'
    cls[detected & ~relayed] = 'late'
    res = dict(cls=cls, threat=threat, n_coll=int(collided.sum()), collided=collided,
               n_det=int(detected.sum()), n_replan=n_replan, t_mission=t, t_start=t_start,
               extra=driven - FIELD_X, ground3=driven3, canopy_clear=canopy_clear,
               dz_det=dz_obs_det, vzmax=vzmax, min_lead=min_lead, obs=obs, oz=oz, zmax=zmax_seen)
    if keep:
        res.update({k: np.array(v) for k, v in log.items()})
        res['path'] = np.array(path)
    return res


def summarize(runs):
    th = np.concatenate([r['threat'] for r in runs])
    cl = np.concatenate([r['cls'] for r in runs])
    coll = np.array([r['n_coll'] for r in runs])
    return dict(n_threat=int(th.sum()),
                frac={c: float(np.mean(cl[th] == c)) for c in ('timely', 'marginal', 'late', 'missed')},
                coll=coll, coll_mean=coll.mean(), coll_std=coll.std(), runs_coll=int((coll > 0).sum()),
                det=float(np.mean([r['n_det'] for r in runs])),
                t_mission=np.array([r['t_mission'] for r in runs]),
                extra=np.array([r['extra'] for r in runs]),
                canopy=float(min(r['canopy_clear'] for r in runs)),
                t_start=float(np.mean([r['t_start'] for r in runs])))


def run_simulation():
    S = sec_grid(A_DEM)
    main = {p: simulate(SEED, p, keep=True, S=S) for p in PLANS}
    main['oracle'] = simulate(SEED, oracle=True, keep=True, S=S)
    mc = {p: [simulate(s, p, S=S) for s in range(N_MC)] for p in PLANS}
    mc['oracle'] = [simulate(s, oracle=True, S=S) for s in range(N_MC)]
    csweep = {c: (mc['acl_low'][:N_SWEEP] if c == C_MIN else
                  [simulate(s, 'acl_low', c_min=c, S=S) for s in range(N_SWEEP)]) for c in C_SWEEP}
    asweep = {}
    for a in A_SWEEP:
        Sa = S if a == A_DEM else sec_grid(a)
        asweep[a] = {p: (mc[p][:N_SWEEP] if a == A_DEM else
                         [simulate(s, p, a=a, S=Sa) for s in range(N_SWEEP)]) for p in PLANS}
        asweep[a]['oracle'] = (mc['oracle'][:N_SWEEP] if a == A_DEM else
                               [simulate(s, oracle=True, a=a, S=Sa) for s in range(N_SWEEP)])
    return main, mc, csweep, asweep


# ── Plots ─────────────────────────────────────────────────────────────────────
def _save(fig, out_dir, name, dpi=130):
    path = os.path.join(out_dir, name)
    fig.savefig(path, dpi=dpi, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_field_3d(main, out_dir):
    fig = plt.figure(figsize=(14, 6.5))
    ax = fig.add_subplot(111, projection='3d')
    X, Y = np.meshgrid(np.linspace(0, FIELD_X, 81), np.linspace(0, FIELD_Y, 21))
    Z = terrain(X, Y) + crop(X, Y)
    ax.plot_surface(X, Y, Z, facecolors=plt.cm.YlGn((crop(X, Y) - 0.3) / 0.9), alpha=0.5, lw=0)
    ax.plot_wireframe(X, Y, terrain(X, Y), color='0.5', lw=0.3, rstride=2, cstride=4)
    for p in PLANS:
        d = main[p]['d']
        ax.plot(d[:, 0], d[:, 1], d[:, 2], color=COLORS[p], lw=1.4, label=LABELS[p])
    r = main['acl_low']
    h = r['h']
    ax.plot(h[:, 0], h[:, 1], terrain(h[:, 0], h[:, 1]), color='tab:red', lw=2, label='harvester (acl_low run)')
    det = r['cls'] != 'missed'
    ax.scatter(r['obs'][:, 0], r['obs'][:, 1], r['oz'], c=np.where(det, 'darkorange', '0.4'), s=30,
               depthshade=False, label='obstacles (orange = detected in acl_low run)')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_zlabel('z (m)')
    ax.set_box_aspect((200, 50, 25))
    ax.view_init(elev=25, azim=-70)
    ax.legend(fontsize=8, loc='upper left')
    ax.set_title(f'Terrain (wireframe), canopy top (green = taller crop) and scout altitudes, seed {SEED}')
    _save(fig, out_dir, 'field_3d.png')


def plot_altitude(main, out_dir):
    fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
    for p in PLANS:
        d = main[p]['d']
        x = d[:, 0]
        axes[0].plot(x, d[:, 2], color=COLORS[p], label=LABELS[p])
        axes[2].plot(x, d[:, 2] - terrain(x, d[:, 1]) - crop(x, d[:, 1]), color=COLORS[p])
        axes[3].plot(x, np.sqrt(np.maximum((R_SENSOR - GPS_ERROR) ** 2 - (d[:, 2] - terrain(x, d[:, 1]) - H_OBS) ** 2, 0)),
                     color=COLORS[p])
    xx = np.linspace(0, FIELD_X, 401)
    axes[1].plot(xx, terrain(xx, Y_TRACK), 'k-', label='terrain under the track')
    axes[1].plot(xx, terrain(xx, Y_TRACK) + crop(xx, Y_TRACK), 'g-', label='canopy top')
    axes[0].set_ylabel('scout z MSL (m)')
    axes[1].set_ylabel('z (m)')
    axes[2].set_ylabel('clearance above canopy (m)')
    axes[3].set_ylabel('horizontal detection radius (m)')
    axes[3].axhline(R_INFLATE, color='k', ls='--', lw=1, label='on-track band half-width 2.5 m')
    axes[3].set_xlabel('scout x (m)')
    axes[0].legend(fontsize=8)
    axes[1].legend(fontsize=8)
    axes[3].legend(fontsize=8)
    for ax in axes:
        ax.grid(alpha=0.3)
    fig.suptitle(f'Scout altitude plans along the field (seed {SEED})')
    plt.tight_layout()
    _save(fig, out_dir, 'altitude_profiles.png')


def plot_detection(summ, csweep_s, out_dir):
    fig, axes = plt.subplots(1, 3, figsize=(16, 4.8))
    keys = PLANS
    cols = {'timely': 'green', 'marginal': 'gold', 'late': 'red', 'missed': 'grey'}
    bottom = np.zeros(len(keys))
    for c in ('timely', 'marginal', 'late', 'missed'):
        v = np.array([100 * summ[k]['frac'][c] for k in keys])
        axes[0].bar(range(len(keys)), v, bottom=bottom, color=cols[c], edgecolor='k', label=c)
        for i, (b, vv) in enumerate(zip(bottom, v)):
            if vv > 4:
                axes[0].text(i, b + vv / 2, f'{vv:.1f}%', ha='center', va='center', fontsize=8)
        bottom += v
    axes[0].set_xticks(range(len(keys)))
    axes[0].set_xticklabels(['fixed MSL\n+2.0 m', f'fixed MSL\n+{C_MIN} m', 'ACL 2.0 m\n(card)', f'ACL\n{C_MIN} m'], fontsize=8)
    axes[0].set_ylabel('% of on-track obstacles')
    axes[0].set_title(f'On-track obstacles, {N_MC} seeds')
    axes[0].legend(fontsize=7, loc='lower right')
    k3 = ['oracle'] + PLANS
    axes[1].bar(range(5), [summ[k]['coll_mean'] for k in k3], yerr=[summ[k]['coll_std'] for k in k3],
                capsize=4, color=[COLORS[k] for k in k3], edgecolor='k')
    for i, k in enumerate(k3):
        axes[1].text(i, summ[k]['coll_mean'] + summ[k]['coll_std'] + 0.05, f"{summ[k]['coll_mean']:.2f}",
                     ha='center', fontsize=8)
    axes[1].set_xticks(range(5))
    axes[1].set_xticklabels(['oracle', 'fixed +2.0', f'fixed +{C_MIN}', 'ACL 2.0', f'ACL {C_MIN}'], fontsize=8)
    axes[1].set_ylabel('collisions per run (mean ± std)')
    axes[1].set_title('Collisions')
    cs = sorted(csweep_s)
    axes[2].plot(cs, [csweep_s[c]['coll_mean'] for c in cs], 'o-', color='tab:blue', label='collisions / run')
    axes[2].plot(cs, [csweep_s[c]['frac']['missed'] for c in cs], 's--', color='grey', label='missed on-track fraction')
    axes[2].plot(cs, [csweep_s[c]['canopy'] for c in cs], '^:', color='green', label='min clearance above canopy (m)')
    axes[2].axhline(0, color='k', lw=0.6)
    axes[2].set_xlabel('commanded clearance above canopy (m)')
    axes[2].set_title(f'Clearance sweep ({N_SWEEP} seeds)')
    axes[2].legend(fontsize=7)
    axes[2].grid(alpha=0.3)
    plt.tight_layout()
    _save(fig, out_dir, 'detection_comparison.png')


def plot_speed(main, out_dir):
    r = main['acl_low']
    h, t, v = r['h'], r['t'], r['v']
    fig, ax = plt.subplots(figsize=(11, 4.2))
    m = v > 0
    ax.plot(t[m], v[m], color='tab:red', label='harvester speed')
    ax.axhline(V0_HARV, color='k', ls=':', lw=1, label='v0 = 1.5 m/s (flat)')
    ax2 = ax.twinx()
    gx, gy = grad(h[:, 0], h[:, 1])
    dh = np.diff(h, axis=0, prepend=h[:1])
    u = dh / np.maximum(np.hypot(dh[:, 0], dh[:, 1]), 1e-12)[:, None]
    ax2.plot(t[m], np.rad2deg(np.arctan(gx * u[:, 0] + gy * u[:, 1]))[m], color='tab:brown', lw=0.8,
             label='slope along heading (deg)')
    ax.set_xlabel('t (s)')
    ax.set_ylabel('speed (m/s)')
    ax2.set_ylabel('slope (deg)')
    ax.legend(fontsize=8, loc='upper left')
    ax2.legend(fontsize=8, loc='upper right')
    ax.set_title(f'Harvester speed on the slope (seed {SEED}, acl_low run)')
    _save(fig, out_dir, 'speed_profile.png')


def plot_dem_sweep(asweep_s, out_dir):
    fig, axes = plt.subplots(1, 2, figsize=(13, 4.6))
    for p in PLANS + ['oracle']:
        axes[0].plot(A_SWEEP, [asweep_s[a][p]['coll_mean'] for a in A_SWEEP], 'o-', color=COLORS[p], label=LABELS[p])
        axes[1].plot(A_SWEEP, [asweep_s[a][p]['t_mission'].mean() for a in A_SWEEP], 'o-', color=COLORS[p])
    axes[0].set_xlabel('DEM amplitude A_dem (m)')
    axes[0].set_ylabel('collisions per run')
    axes[0].legend(fontsize=8)
    axes[1].set_xlabel('DEM amplitude A_dem (m)')
    axes[1].set_ylabel('mission time (s)')
    for ax in axes:
        ax.grid(alpha=0.3)
    fig.suptitle(f'DEM amplitude sweep ({N_SWEEP} seeds, same layouts)')
    plt.tight_layout()
    _save(fig, out_dir, 'dem_sweep.png')


def save_animation(main, out_dir):
    r = main['acl_low']
    rc = main['acl_card']
    n = min(len(r['t']), len(rc['t']))
    frames = int(np.ceil((n - 1) / ANIM_STEP)) + 1
    fig = plt.figure(figsize=(9, 5))
    ax = fig.add_subplot(111, projection='3d')
    X, Y = np.meshgrid(np.linspace(0, FIELD_X, 41), np.linspace(0, FIELD_Y, 11))
    ax.plot_wireframe(X, Y, terrain(X, Y) + crop(X, Y), color='green', lw=0.3)
    ax.scatter(r['obs'][:, 0], r['obs'][:, 1], r['oz'], c='0.3', s=15, depthshade=False)
    hl, = ax.plot([], [], [], color='tab:red', lw=2, label='harvester')
    dl, = ax.plot([], [], [], color='tab:blue', lw=1, label=LABELS['acl_low'])
    cl, = ax.plot([], [], [], color='tab:purple', lw=1, label=LABELS['acl_card'])
    ax.set_xlim(0, FIELD_X)
    ax.set_ylim(0, FIELD_Y)
    ax.set_zlim(0, 8)
    ax.set_box_aspect((200, 50, 25))
    ax.legend(fontsize=7, loc='upper left')

    def update(f):
        k = min(f * ANIM_STEP, n - 1)
        h = r['h'][:k + 1]
        hl.set_data(h[:, 0], h[:, 1])
        hl.set_3d_properties(terrain(h[:, 0], h[:, 1]))
        for line, rr in ((dl, r), (cl, rc)):
            d = rr['d'][:k + 1]
            line.set_data(d[:, 0], d[:, 1])
            line.set_3d_properties(d[:, 2])
        ax.set_title(f't = {k * DT:5.1f} s (seed {SEED})', fontsize=9)
        return hl, dl, cl

    anim = animation.FuncAnimation(fig, update, frames=frames, interval=80, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    anim.save(path, writer='pillow', fps=12, dpi=65)
    plt.close(fig)
    print(f'Saved: {path} ({frames} frames)')


# ── Report ────────────────────────────────────────────────────────────────────
def report(main, mc, csweep, asweep):
    X, Y = np.meshgrid(np.linspace(0, FIELD_X, 801), np.linspace(0, FIELD_Y, 201))
    gx, gy = grad(X, Y)
    print('\n=== S078 3D Harvester Guidance — Results ===')
    print(f'  DEM: terrain {terrain(X, Y).min():.2f} .. {terrain(X, Y).max():.2f} m (card DEM + A_dem), '
          f'max slope {np.rad2deg(np.arctan(np.hypot(gx, gy).max())):.2f} deg; crop {crop(X, Y).min():.2f} .. '
          f'{crop(X, Y).max():.2f} m')
    print(f'  Fixed MSL altitude: highest canopy top + {H_ACL} m = {z_fixed():.2f} m; + {C_MIN} m = {z_fixed(A_DEM, C_MIN):.2f} m')
    rd = R_SENSOR - GPS_ERROR
    for c in (0.3, 0.5, 1.0, H_ACL):
        for hc in (0.3, 1.2):
            dzz = hc + c - H_OBS
            print(f'    clearance {c:.1f} m over {hc:.1f} m crop: dz to obstacle top {dzz:.2f} m, horizontal '
                  f'detection radius {np.sqrt(max(rd ** 2 - dzz ** 2, 0)):.2f} m')
    print(f'  Harvester speed factor: eta at the max slope uphill/downhill '
          f'{eta_slope(np.arctan(np.hypot(gx, gy).max())):.3f} / {eta_slope(-np.arctan(np.hypot(gx, gy).max())):.3f}')
    print(f'  [Seed {SEED}]')
    for p in PLANS + ['oracle']:
        r = main[p]
        print(f"    {LABELS[p]:<34} collisions {r['n_coll']}, detected {r['n_det']}/{N_OBS}, replans {r['n_replan']}, "
              f"mission {r['t_mission']:.1f} s, extra path {r['extra']:.2f} m"
              + (f", min canopy clearance {r['canopy_clear']:.2f} m, max |v_z| {r['vzmax']:.2f} m/s"
                 if p != 'oracle' else ''))
    summ = {k: summarize(v) for k, v in mc.items()}
    print(f'  [Monte Carlo {N_MC} seeds, common random numbers, {summ["acl_card"]["n_threat"]} on-track obstacles]')
    for k in PLANS + ['oracle']:
        s = summ[k]
        f = s['frac']
        print(f"    {LABELS[k]:<34} collisions/run {s['coll_mean']:.2f} ± {s['coll_std']:.2f} ({s['runs_coll']}/{N_MC} runs); "
              + (f"on-track timely/marginal/late/missed {100 * f['timely']:.1f}/{100 * f['marginal']:.1f}/"
                 f"{100 * f['late']:.1f}/{100 * f['missed']:.1f}%; " if k != 'oracle' else '')
              + f"detected {s['det']:.2f}/{N_OBS}; mission {s['t_mission'].mean():.1f} ± {s['t_mission'].std():.1f} s; "
              f"extra path {s['extra'].mean():.2f} m" + (f"; min canopy clearance {s['canopy']:.2f} m" if k != 'oracle' else ''))
    for a, b in (('acl_low', 'acl_card'), ('acl_low', 'fixed_low'), ('acl_card', 'fixed_msl'), ('fixed_low', 'fixed_msl')):
        ca, cb = summ[a]['coll'], summ[b]['coll']
        print(f'    paired collisions {a} vs {b}: fewer {int((ca < cb).sum())}, equal {int((ca == cb).sum())}, '
              f'more {int((ca > cb).sum())}; mean difference {np.mean(ca - cb):+.2f}')
    dz = np.concatenate([np.array(r['dz_det']) for r in mc['acl_low']])
    print(f'    acl_low detections: drone height above obstacle top {dz.min():.2f} .. {dz.max():.2f} m')
    flat = summarize(asweep[0.0]['oracle'])
    print(f'  Oracle mission time: flat (A_dem = 0) {flat["t_mission"].mean():.1f} s, card DEM '
          f'{summ["oracle"]["t_mission"].mean():.1f} s ({summ["oracle"]["t_mission"].mean() - flat["t_mission"].mean():+.1f} s)')
    print(f'  [Clearance sweep, acl_low, {N_SWEEP} seeds]')
    for c in C_SWEEP:
        s = summarize(csweep[c])
        print(f"    clearance {c:.1f} m: collisions/run {s['coll_mean']:.2f} ({s['runs_coll']}/{N_SWEEP}), missed on-track "
              f"{100 * s['frac']['missed']:.1f}%, timely {100 * s['frac']['timely']:.1f}%, min canopy clearance "
              f"{s['canopy']:.2f} m")
    print(f'  [DEM amplitude sweep, {N_SWEEP} seeds]')
    for a in A_SWEEP:
        line = ', '.join(f"{p} {summarize(asweep[a][p])['coll_mean']:.2f} coll / "
                         f"{100 * summarize(asweep[a][p])['frac']['missed']:.0f}% missed / "
                         f"{summarize(asweep[a][p])['t_mission'].mean():.1f} s" for p in PLANS)
        so = summarize(asweep[a]['oracle'])
        print(f'    A_dem {a:.0f} m (fixed MSL {z_fixed(a):.2f} / {z_fixed(a, C_MIN):.2f} m'
              + (f', above the {Z_BOUNDS[1]:.0f} m ceiling' if z_fixed(a) > Z_BOUNDS[1] else '')
              + f', max slope {np.rad2deg(np.arctan(np.hypot(*grad(X, Y, a)).max())):.2f} deg): {line}; '
              f'oracle {so["t_mission"].mean():.1f} s')
    return summ


if __name__ == '__main__':
    main, mc, csweep, asweep = run_simulation()
    summ = report(main, mc, csweep, asweep)
    out = os.path.normpath(OUTPUT_DIR)
    os.makedirs(out, exist_ok=True)
    plot_field_3d(main, out)
    plot_altitude(main, out)
    plot_detection(summ, {c: summarize(v) for c, v in csweep.items()}, out)
    plot_speed(main, out)
    plot_dem_sweep({a: {p: summarize(v) for p, v in d.items()} for a, d in asweep.items()}, out)
    save_animation(main, out)
