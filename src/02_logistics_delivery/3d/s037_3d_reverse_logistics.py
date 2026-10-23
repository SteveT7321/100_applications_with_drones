"""
S037 3D Upgrade — Reverse Logistics
===================================
Five drones collect returned parcels from 12 customers and bring them back to
the depot at (250, 250, 2) m (CVRPTW).  The 3D card adds:

  * a fixed transit altitude z_cruise = 12 m, with depot climb / descent;
  * customer ground elevation h_i ~ U[0, 8] m and a descend-hover-ascend
    attachment excursion to z_hover,i = h_i + 1.2 m at every stop;
  * an attachment dwell tau_i = tau_0 + kappa q_i;
  * three cylindrical buildings, taller than z_cruise, that every transit
    leg must clear horizontally by r_c + r_safety (single tangent-waypoint
    detour per obstacle);
  * a 3D energy proxy E_k = horizontal + gamma * vertical, range E_k <= R_max.

Three planners run the same three-stage pipeline (Clarke-Wright -> 2-opt ->
Or-opt) on the same instances; every result is EVALUATED with the true 3D
model (detoured legs, vertical excursions, 3D timing):

  P1  2D planner   -- the S037 2D cost (straight horizontal legs, no vertical
                      motion, no dwell, obstacles ignored)
  P2  3D penalty   -- card's optimisation model: straight legs with the
                      obstacle-penetration penalty nu * W, plus vertical cost
                      and 3D timing; detours are inserted afterwards
  P3  3D detour    -- leg lengths and times use the confirmed detour paths
                      (planning model = evaluation model)

The showcase instance (seed 42) is plotted and animated; the same pipeline is
also run on 30 further instances (seeds 0..29) with identical random draws
for every planner.  On seed 42 all three planners return the same routes, so
the route figures add a contrast instance chosen by a fixed rule: the study
seed with the median P1 - P3 gap among the seeds where the two differ.

Outputs (outputs/02_logistics_delivery/3d/s037_3d_reverse_logistics/):
  routes_3d.png, top_down_map.png, altitude_profiles.png, cost_breakdown.png,
  stage_comparison.png, time_windows.png, animation.gif

Usage:
    conda activate drones
    python src/02_logistics_delivery/3d/s037_3d_reverse_logistics.py
"""

import matplotlib
matplotlib.use('Agg')

import os
import math
import itertools
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.lines import Line2D
import matplotlib.patches as mpatches

OUTPUT_DIR = os.path.normpath(os.path.join(
    os.path.dirname(__file__), '..', '..', '..',
    'outputs', '02_logistics_delivery', '3d', 's037_3d_reverse_logistics',
))

# ── Parameters (card) ──────────────────────────────────────────────────────────
N_DRONES    = 5
N_CUSTOMERS = 12
Q_MAX       = 3.0          # kg payload capacity (hard)
R_MAX       = 2500.0       # m-equivalent energy range per drone
V_C         = 8.0          # m/s horizontal cruise speed
V_Z         = 2.0          # m/s vertical speed
Z_CRUISE    = 12.0         # m transit altitude
Z_DEPOT     = 2.0          # m depot altitude
H_MAX       = 8.0          # m customer ground elevation h_i ~ U[0, H_MAX]
Z_OFFSET    = 1.2          # m winch clearance above the ground
TAU_0       = 8.0          # s base attachment time
KAPPA       = 3.0          # s/kg attachment time per kg
K_OBS       = 3            # cylindrical obstacles
R_OBS_RANGE = (15.0, 25.0) # m obstacle radius
R_SAFETY    = 5.0          # m safety buffer
GAMMA       = 1.4          # vertical energy penalty
LAMBDA_TW   = 50.0         # cost per second of lateness
MU_RANGE    = 1000.0       # cost per m-eq of range excess
NU_OBS      = 500.0        # cost per m of buffer penetration depth
DEPOT_XY    = np.array([250.0, 250.0])
ARENA       = 500.0

# ── Added parameters (not on the card) ────────────────────────────────────────
H_OBS_RANGE   = (20.0, 40.0)  # m obstacle height (card: unspecified; > z_cruise)
OBS_MARGIN    = 5.0           # m min gap from any buffer edge to a customer,
                              #   the depot or another buffer (instance generator)
DETOUR_STEP   = 0.25          # m outward step when pushing a detour waypoint
VIS_SIDES     = 16            # polygon sides for the visibility-graph fallback
SHOWCASE_SEED = 42
N_INSTANCES   = 30            # study seeds 0..29 (as in the 2D script)
DT            = 0.1           # s trajectory sample step
GIF_FRAMES    = 150

DZ_DEPOT = Z_CRUISE - Z_DEPOT          # 10 m
PLANNERS = ['P1', 'P2', 'P3']
PLANNER_NAMES = {'P1': '2D planner', 'P2': '3D penalty', 'P3': '3D detour'}
PLANNER_MODE = {'P1': '2d', 'P2': 'pen', 'P3': 'det'}
STAGES = ['cw', 'two', 'fin']
STAGE_NAMES = {'cw': 'Clarke-Wright', 'two': '+ 2-opt', 'fin': '+ Or-opt'}
DRONE_COLORS = ['#e6194b', '#3cb44b', '#4363d8', '#f58231', '#911eb4']
EPS = 1e-9


# ── Instance ───────────────────────────────────────────────────────────────────
class Instance:
    pass


def seg_dist(c, A, B):
    """Distance from point c to segment A-B (all 2D)."""
    AB = B - A
    den = float(AB @ AB)
    s = 0.0 if den == 0.0 else min(1.0, max(0.0, float((c - A) @ AB) / den))
    return float(np.linalg.norm(A + s * AB - c)), s


def make_instance(seed):
    # Customer draws: identical stream and order to the 2D S037 script
    rng = np.random.default_rng(seed)
    pos, q, e, l = [], [], [], []
    for _ in range(N_CUSTOMERS):
        pos.append(rng.uniform(50, 450, size=2))
        q.append(float(rng.uniform(0.3, 1.2)))
        e.append(float(rng.uniform(0, 60)))
        l.append(float(rng.uniform(90, 180)))
    inst = Instance()
    inst.seed = seed
    inst.pos = np.array(pos)
    inst.q = np.array(q)
    inst.e = np.array(e)
    inst.l = np.array(l)
    # 3D additions: separate stream so the 2D customers stay unchanged
    rng3 = np.random.default_rng([seed, 37])
    inst.h = rng3.uniform(0.0, H_MAX, N_CUSTOMERS)
    inst.zh = inst.h + Z_OFFSET
    inst.dz = Z_CRUISE - inst.zh
    inst.tau = TAU_0 + KAPPA * inst.q
    inst.xy = np.vstack([inst.pos, DEPOT_XY])          # node M = depot
    obs = []
    for _ in range(100000):
        cx, cy = rng3.uniform(50, 450, size=2)
        r = float(rng3.uniform(*R_OBS_RANGE))
        H = float(rng3.uniform(*H_OBS_RANGE))
        c = np.array([cx, cy])
        R = r + R_SAFETY
        if np.min(np.linalg.norm(inst.xy - c, axis=1)) < R + OBS_MARGIN:
            continue
        if any(np.linalg.norm(c - o[0]) < R + o[1] + R_SAFETY + OBS_MARGIN for o in obs):
            continue
        obs.append((c, r, H))
        if len(obs) == K_OBS:
            break
    assert len(obs) == K_OBS
    inst.obs = obs
    precompute_legs(inst)
    return inst


def _clear(A, B, obs):
    return all(seg_dist(c, A, B)[0] >= r + R_SAFETY - 1e-9 for (c, r, H) in obs)


def _path_len(path):
    return sum(float(np.linalg.norm(path[i + 1] - path[i])) for i in range(len(path) - 1))


def leg_path(A, B, obs, depth=0):
    """Card rule: horizontal path A -> B that keeps r + r_safety from every
    obstacle, with one waypoint per blocking obstacle at c + R' n (n
    perpendicular to the leg; R' starts at r + r_safety and is pushed outward
    in DETOUR_STEP increments until both sub-legs clear that obstacle); the
    side with the shorter total path wins; sub-legs are checked recursively.
    Returns None when neither side works (waypoint inside another buffer)."""
    viol = []
    for k, (c, r, H) in enumerate(obs):
        d, s = seg_dist(c, A, B)
        if d < r + R_SAFETY - 1e-9:
            viol.append((s, k))
    if not viol:
        return [A, B]
    if depth >= 6:
        return None
    _, k = min(viol)
    c, r, H = obs[k]
    R = r + R_SAFETY
    u = (B - A) / np.linalg.norm(B - A)
    n = np.array([-u[1], u[0]])
    best, best_len = None, np.inf
    for sign in (1.0, -1.0):
        Rp = R
        found = False
        while Rp < 4 * R:
            qpt = c + sign * Rp * n
            if (seg_dist(c, A, qpt)[0] >= R - 1e-9 and
                    seg_dist(c, qpt, B)[0] >= R - 1e-9):
                found = True
                break
            Rp += DETOUR_STEP
        if not found:
            continue
        if any(np.linalg.norm(qpt - c2) < r2 + R_SAFETY - 1e-9 for (c2, r2, _) in obs):
            continue
        p1 = leg_path(A, qpt, obs, depth + 1)
        p2 = leg_path(qpt, B, obs, depth + 1)
        if p1 is None or p2 is None:
            continue
        path = p1[:-1] + p2
        L = _path_len(path)
        if L < best_len - 1e-12:
            best, best_len = path, L
    return best


def visibility_path(A, B, obs):
    """Fallback: shortest path over a visibility graph whose extra nodes are
    the vertices of a VIS_SIDES-gon circumscribing each buffer circle."""
    nodes = [A, B]
    for (c, r, H) in obs:
        Rv = (r + R_SAFETY) / math.cos(math.pi / VIS_SIDES) + 1e-6
        for m in range(VIS_SIDES):
            th = 2 * math.pi * m / VIS_SIDES
            nodes.append(c + Rv * np.array([math.cos(th), math.sin(th)]))
    nn = len(nodes)
    dist = [np.inf] * nn
    prev = [-1] * nn
    done = [False] * nn
    dist[0] = 0.0
    for _ in range(nn):
        u = min((i for i in range(nn) if not done[i]), key=lambda i: dist[i])
        if dist[u] == np.inf or u == 1:
            break
        done[u] = True
        for v in range(nn):
            if done[v] or v == u:
                continue
            if _clear(nodes[u], nodes[v], obs):
                d = dist[u] + float(np.linalg.norm(nodes[v] - nodes[u]))
                if d < dist[v] - 1e-12:
                    dist[v], prev[v] = d, u
    assert dist[1] < np.inf, 'no obstacle-free path'
    path, v = [], 1
    while v != -1:
        path.append(nodes[v])
        v = prev[v]
    return path[::-1]


def penetration(path, obs):
    """Card W: sum over legs/obstacles of max(0, r + r_safety - d_seg)."""
    W = 0.0
    for i in range(len(path) - 1):
        for (c, r, H) in obs:
            W += max(0.0, r + R_SAFETY - seg_dist(c, path[i], path[i + 1])[0])
    return W


def precompute_legs(inst):
    n = N_CUSTOMERS + 1
    inst.L_st = np.zeros((n, n))
    inst.W_st = np.zeros((n, n))
    inst.L_det = np.zeros((n, n))
    inst.paths = {}
    inst.n_fallback = 0
    for i in range(n):
        inst.paths[(i, i)] = [inst.xy[i], inst.xy[i]]
    for i, j in itertools.combinations(range(n), 2):
        A, B = inst.xy[i], inst.xy[j]
        inst.L_st[i, j] = inst.L_st[j, i] = float(np.linalg.norm(B - A))
        inst.W_st[i, j] = inst.W_st[j, i] = penetration([A, B], inst.obs)
        p = leg_path(A, B, inst.obs)
        if p is None:
            p = visibility_path(A, B, inst.obs)
            inst.n_fallback += 1
        assert penetration(p, inst.obs) <= 1e-6
        L = _path_len(p)
        inst.L_det[i, j] = inst.L_det[j, i] = L
        inst.paths[(i, j)] = p
        inst.paths[(j, i)] = p[::-1]


# ── Route evaluation ───────────────────────────────────────────────────────────
def route_eval(seq, inst, mode):
    """mode '2d': S037 2D model (straight horizontal legs, no vertical, no dwell)
       mode 'pen': 3D model on straight legs + nu * W   (card penalty planner)
       mode 'det': 3D model on detoured legs            (true 3D evaluation)"""
    D = N_CUSTOMERS
    out = dict(hor=0.0, vert=0.0, late=0.0, W=0.0, E=0.0, cdist=0.0, cost=0.0,
               arr=[], t_end=0.0, t_hor=0.0, t_vert=0.0, t_dwell=0.0, t_wait=0.0,
               n_late=0, n_detour=0)
    if not seq:
        return out
    if mode == '2d':
        t, prev = 0.0, D
        for i in seq:
            out['hor'] += inst.L_st[prev, i]
            t = max(inst.e[i], t + inst.L_st[prev, i] / V_C)
            out['arr'].append(t)
            out['late'] += max(0.0, t - inst.l[i])
            prev = i
        out['hor'] += inst.L_st[prev, D]
        out['t_end'] = t + inst.L_st[prev, D] / V_C
        out['cdist'] = out['hor']
        out['E'] = out['hor']
        out['cost'] = (out['hor'] + LAMBDA_TW * out['late'] +
                       MU_RANGE * max(0.0, out['hor'] - R_MAX))
        return out
    L = inst.L_st if mode == 'pen' else inst.L_det
    t = DZ_DEPOT / V_Z
    out['t_vert'] += DZ_DEPOT / V_Z
    prev = D
    vert = 2.0 * DZ_DEPOT
    for i in seq:
        out['hor'] += L[prev, i]
        out['W'] += inst.W_st[prev, i] if mode == 'pen' else 0.0
        out['n_detour'] += int(len(inst.paths[(prev, i)]) > 2)
        t += L[prev, i] / V_C + inst.dz[i] / V_Z          # transit + descent
        out['t_hor'] += L[prev, i] / V_C
        out['t_vert'] += 2.0 * inst.dz[i] / V_Z
        a = t
        out['arr'].append(a)
        lt = max(0.0, a - inst.l[i])
        out['late'] += lt
        out['n_late'] += int(lt > 1e-9)
        out['t_wait'] += max(0.0, inst.e[i] - a)
        out['t_dwell'] += inst.tau[i]
        t = max(a, inst.e[i]) + inst.tau[i] + inst.dz[i] / V_Z   # attach + ascent
        vert += 2.0 * inst.dz[i]
        prev = i
    out['hor'] += L[prev, D]
    out['W'] += inst.W_st[prev, D] if mode == 'pen' else 0.0
    out['n_detour'] += int(len(inst.paths[(prev, D)]) > 2)
    out['t_hor'] += L[prev, D] / V_C
    out['t_vert'] += DZ_DEPOT / V_Z
    out['t_end'] = t + L[prev, D] / V_C + DZ_DEPOT / V_Z
    out['vert'] = vert
    out['cdist'] = out['hor'] + vert
    out['E'] = out['hor'] + GAMMA * vert
    out['cost'] = (out['cdist'] + LAMBDA_TW * out['late'] +
                   MU_RANGE * max(0.0, out['E'] - R_MAX) + NU_OBS * out['W'])
    return out


def route_cost(seq, inst, mode):
    return route_eval(seq, inst, mode)['cost']


def savings_cost(seq, inst, mode):
    """Distance-type route cost used for Clarke-Wright savings (no lateness)."""
    r = route_eval(seq, inst, mode)
    return r['cdist'] + NU_OBS * r['W']


def payload(seq, inst):
    return float(sum(inst.q[i] for i in seq))


# ── Clarke-Wright / 2-opt / Or-opt (S037 logic, cost model as argument) ───────
def clarke_wright(inst, mode):
    M = N_CUSTOMERS
    single = [savings_cost([i], inst, mode) for i in range(M)]
    S = np.zeros((M, M))
    for i, j in itertools.combinations(range(M), 2):
        s = single[i] + single[j] - savings_cost([i, j], inst, mode)
        S[i, j] = S[j, i] = s
    routes = [[i] for i in range(M)]
    pairs = sorted(itertools.combinations(range(M), 2), key=lambda p: -S[p[0], p[1]])
    for (i, j) in pairs:
        if len(routes) <= N_DRONES:
            break
        for a, b in ((i, j), (j, i)):
            ra = next((r for r in routes if r[-1] == a), None)
            rb = next((r for r in routes if r[0] == b), None)
            if ra is None or rb is None or ra is rb:
                continue
            merged = ra + rb
            if payload(merged, inst) <= Q_MAX:
                routes.remove(ra)
                routes.remove(rb)
                routes.append(merged)
                break
    if len(routes) > N_DRONES:
        raise RuntimeError(f'Clarke-Wright left {len(routes)} routes')
    while len(routes) < N_DRONES:
        routes.append([])
    return routes


def two_opt(route, inst, mode):
    best = route[:]
    improved = True
    while improved:
        improved = False
        for j in range(0, len(best) - 1):
            for k in range(j + 1, len(best)):
                cand = best[:j] + best[j:k + 1][::-1] + best[k + 1:]
                if route_cost(cand, inst, mode) < route_cost(best, inst, mode) - 1e-6:
                    best = cand
                    improved = True
    return best


def or_opt_relocate(routes, inst, mode):
    routes = [r[:] for r in routes]
    improved = True
    while improved:
        improved = False
        for k_src in range(len(routes)):
            for pos_src in range(len(routes[k_src])):
                cid = routes[k_src][pos_src]
                for k_dst in range(len(routes)):
                    if k_dst == k_src:
                        continue
                    if payload(routes[k_dst], inst) + inst.q[cid] > Q_MAX:
                        continue
                    for pos_dst in range(len(routes[k_dst]) + 1):
                        src_new = routes[k_src][:pos_src] + routes[k_src][pos_src + 1:]
                        dst_new = routes[k_dst][:pos_dst] + [cid] + routes[k_dst][pos_dst:]
                        delta = (route_cost(src_new, inst, mode) + route_cost(dst_new, inst, mode)
                                 - route_cost(routes[k_src], inst, mode)
                                 - route_cost(routes[k_dst], inst, mode))
                        if delta < -1e-6:
                            routes[k_src] = src_new
                            routes[k_dst] = dst_new
                            improved = True
                            break
                    if improved:
                        break
                if improved:
                    break
    return routes


def solve(inst, mode):
    r_cw = clarke_wright(inst, mode)
    r_two = [two_opt(r, inst, mode) for r in r_cw]
    r_fin = or_opt_relocate(r_two, inst, mode)
    for rs in (r_cw, r_two, r_fin):
        assert sorted(c for r in rs for c in r) == list(range(N_CUSTOMERS))
        assert all(payload(r, inst) <= Q_MAX + 1e-9 for r in rs)
    return {'cw': r_cw, 'two': r_two, 'fin': r_fin}


def metrics(routes, inst, mode='det'):
    ev = [route_eval(r, inst, mode) for r in routes]
    return dict(
        hor=sum(e['hor'] for e in ev), vert=sum(e['vert'] for e in ev),
        late=sum(e['late'] for e in ev), n_late=sum(e['n_late'] for e in ev),
        cost=sum(e['cost'] for e in ev), E_max=max(e['E'] for e in ev),
        makespan=max(e['t_end'] for e in ev), n_detour=sum(e['n_detour'] for e in ev),
        W=sum(e['W'] for e in ev), ev=ev)


# ── Trajectory (true 3D, detoured) ─────────────────────────────────────────────
def build_trajectory(route, inst):
    """Knot list (t, x, y, z) for one drone flying `route` in true 3D."""
    D = N_CUSTOMERS
    kt = [0.0]
    kp = [np.array([DEPOT_XY[0], DEPOT_XY[1], Z_DEPOT])]
    events = []        # (arrival time, customer)

    def add(dt, p):
        if dt > 1e-12:
            kt.append(kt[-1] + dt)
            kp.append(np.asarray(p, float))

    if not route:
        return np.array(kt), np.array(kp), events
    add(DZ_DEPOT / V_Z, [DEPOT_XY[0], DEPOT_XY[1], Z_CRUISE])
    prev = D
    for i in route + [D]:
        path = inst.paths[(prev, i)]
        for a, b in zip(path[:-1], path[1:]):
            add(float(np.linalg.norm(b - a)) / V_C, [b[0], b[1], Z_CRUISE])
        if i == D:
            add(DZ_DEPOT / V_Z, [DEPOT_XY[0], DEPOT_XY[1], Z_DEPOT])
            break
        x, y = inst.xy[i]
        add(inst.dz[i] / V_Z, [x, y, inst.zh[i]])
        events.append((kt[-1], i))
        add(max(0.0, inst.e[i] - kt[-1]), [x, y, inst.zh[i]])
        add(inst.tau[i], [x, y, inst.zh[i]])
        add(inst.dz[i] / V_Z, [x, y, Z_CRUISE])
        prev = i
    return np.array(kt), np.array(kp), events


def sample_trajectories(routes, inst):
    knots = [build_trajectory(r, inst) for r in routes]
    T = max(k[0][-1] for k in knots)
    ts = np.arange(0.0, T + DT / 2, DT)
    trajs = []
    for kt, kp, _ in knots:
        if len(kt) == 1:
            trajs.append(np.tile(kp[0], (len(ts), 1)))
            continue
        trajs.append(np.column_stack([np.interp(ts, kt, kp[:, d]) for d in range(3)]))
    return ts, trajs, knots


def check_physics(routes, inst):
    """Assert invariants on the sampled 3D trajectories."""
    ts, trajs, knots = sample_trajectories(routes, inst)
    for k, (route, tr) in enumerate(zip(routes, trajs)):
        ev = route_eval(route, inst, 'det')
        arr = [a for a, _ in knots[k][2]]
        assert np.allclose(arr, ev['arr']), 'trajectory/arrival mismatch'
        if route:
            assert abs(knots[k][0][-1] - ev['t_end']) < 1e-6
            assert ev['E'] <= R_MAX + 1e-9, 'route exceeds battery range'
        assert payload(route, inst) <= Q_MAX + 1e-9
        assert tr[:, 2].min() >= Z_OFFSET - 1e-9           # never below winch clearance over flat ground
        assert tr[:, 2].max() <= Z_CRUISE + 1e-9
        v_xy = np.linalg.norm(np.diff(tr[:, :2], axis=0), axis=1) / DT
        v_z = np.abs(np.diff(tr[:, 2])) / DT
        assert v_xy.max() <= V_C + 1e-6 and v_z.max() <= V_Z + 1e-6, 'speed limit'
        # ground clearance at every customer column, obstacle clearance below H
        for i in range(N_CUSTOMERS):
            near = np.linalg.norm(tr[:, :2] - inst.xy[i], axis=1) < 1e-6
            if near.any():
                assert tr[near, 2].min() >= inst.h[i] + Z_OFFSET - 1e-9
        for (c, r, H) in inst.obs:
            d = np.linalg.norm(tr[:, :2] - c, axis=1)
            below = tr[:, 2] < H
            assert (d[below] >= r + R_SAFETY - 1e-6).all(), 'obstacle buffer entered'
    return ts, trajs, knots


# ── Simulation ─────────────────────────────────────────────────────────────────
def run_simulation():
    show = make_instance(SHOWCASE_SEED)
    show_sol = {p: solve(show, PLANNER_MODE[p]) for p in PLANNERS}
    for p in PLANNERS:
        check_physics(show_sol[p]['fin'], show)
    study = []
    for seed in range(N_INSTANCES):
        inst = make_instance(seed)
        sol = {p: solve(inst, PLANNER_MODE[p]) for p in PLANNERS}
        for p in PLANNERS:
            check_physics(sol[p]['fin'], inst)
        study.append(dict(seed=seed, inst=inst, sol=sol,
                          m={p: {s: metrics(sol[p][s], inst) for s in STAGES}
                             for p in PLANNERS}))
    return show, show_sol, study


# ── Plots ──────────────────────────────────────────────────────────────────────
def _draw_cylinder(ax, c, r, H, alpha=0.25, n=30):
    th = np.linspace(0, 2 * np.pi, n)
    xs, ys = c[0] + r * np.cos(th), c[1] + r * np.sin(th)
    ax.plot_surface(np.vstack([xs, xs]), np.vstack([ys, ys]),
                    np.vstack([np.zeros(n), np.full(n, H)]),
                    color='grey', alpha=alpha, linewidth=0, shade=False)
    ax.plot(xs, ys, np.full(n, H), color='grey', lw=0.6)


def _setup_3d(ax, zmax):
    ax.set_xlim(0, ARENA)
    ax.set_ylim(0, ARENA)
    ax.set_zlim(0, zmax)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_zlabel('z (m)')
    ax.set_box_aspect((1, 1, 0.45))


def plot_routes_3d(panels, out_dir):
    """panels: list of (instance, solutions, planner)."""
    fig = plt.figure(figsize=(20, 7.5))
    for col, (inst, sols, p) in enumerate(panels):
        ax = fig.add_subplot(1, len(panels), col + 1, projection='3d')
        zmax = max(o[2] for o in inst.obs) + 2
        _setup_3d(ax, zmax)
        for (c, r, H) in inst.obs:
            _draw_cylinder(ax, c, r, H)
        _, trajs, _ = sample_trajectories(sols[p]['fin'], inst)
        for k, tr in enumerate(trajs):
            if sols[p]['fin'][k]:
                ax.plot(tr[:, 0], tr[:, 1], tr[:, 2], color=DRONE_COLORS[k], lw=1.4,
                        label=f'Drone {k+1}')
        sc = ax.scatter(inst.pos[:, 0], inst.pos[:, 1], inst.zh, c=inst.h, cmap='viridis',
                        vmin=0, vmax=H_MAX, s=30 + 50 * inst.q, edgecolors='k', depthshade=False)
        ax.scatter(*DEPOT_XY, Z_DEPOT, marker='s', color='black', s=80)
        m = metrics(sols[p]['fin'], inst)
        ax.set_title(f'seed {inst.seed}: {PLANNER_NAMES[p]} ({p}) flown in 3D\n'
                     f'J3D = {m["cost"]:.1f}, late {m["n_late"]}/{N_CUSTOMERS}, '
                     f'{m["n_detour"]} detoured legs', fontsize=10)
        ax.view_init(elev=28, azim=-60)
        ax.legend(fontsize=7, loc='upper left')
    fig.colorbar(sc, ax=fig.axes, shrink=0.5, label='ground elevation h_i (m)')
    fig.suptitle('S037 3D Reverse Logistics — final routes (grey columns = buildings '
                 'taller than z_cruise = 12 m)')
    path = os.path.join(out_dir, 'routes_3d.png')
    plt.savefig(path, dpi=120, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_top_down(inst, sols, out_dir):
    fig, axes = plt.subplots(1, 2, figsize=(15, 7.5))
    for ax, p in zip(axes, ['P1', 'P3']):
        ax.set_xlim(0, ARENA)
        ax.set_ylim(0, ARENA)
        ax.set_aspect('equal')
        for (c, r, H) in inst.obs:
            ax.add_patch(plt.Circle(c, r, color='grey', alpha=0.5))
            ax.add_patch(plt.Circle(c, r + R_SAFETY, fill=False, ls='--', color='grey'))
            ax.text(c[0], c[1], f'H={H:.0f} m', ha='center', va='center', fontsize=7)
        routes = sols[p]['fin']
        n_wp = 0
        for k, route in enumerate(routes):
            if not route:
                continue
            nodes = [N_CUSTOMERS] + route + [N_CUSTOMERS]
            for a, b in zip(nodes[:-1], nodes[1:]):
                path = np.array(inst.paths[(a, b)])
                ax.plot(path[:, 0], path[:, 1], '-', color=DRONE_COLORS[k], lw=1.6)
                if len(path) > 2:
                    ax.plot(path[1:-1, 0], path[1:-1, 1], 'x', color='k', ms=8, mew=2)
                    n_wp += len(path) - 2
                if p == 'P1':
                    st = inst.xy[[a, b]]
                    ax.plot(st[:, 0], st[:, 1], ':', color=DRONE_COLORS[k], lw=0.8)
            for idx, cid in enumerate(route):
                ax.text(inst.pos[cid, 0] + 6, inst.pos[cid, 1] + 6, f'C{cid}.{idx+1}',
                        fontsize=7, color=DRONE_COLORS[k])
            ax.plot([], [], '-', color=DRONE_COLORS[k], label=f'Drone {k+1}')
        sc = ax.scatter(inst.pos[:, 0], inst.pos[:, 1], c=inst.h, cmap='viridis', vmin=0,
                        vmax=H_MAX, s=30 + 80 * inst.q, edgecolors='k', zorder=5)
        ax.plot(*DEPOT_XY, 's', color='black', ms=12, zorder=6, label='Depot')
        ax.plot([], [], 'x', color='k', mew=2, label=f'Detour waypoint ({n_wp})')
        if p == 'P1':
            ax.plot([], [], ':', color='grey', label='Planned straight leg')
        m = metrics(routes, inst)
        ax.set_title(f'{PLANNER_NAMES[p]} ({p}): horizontal {m["hor"]:.1f} m, '
                     f'{m["n_detour"]} detoured legs')
        ax.set_xlabel('x (m)')
        ax.set_ylabel('y (m)')
        ax.legend(fontsize=7, loc='upper center', bbox_to_anchor=(0.5, -0.1), ncol=4)
        ax.grid(alpha=0.2)
    fig.colorbar(sc, ax=axes, shrink=0.7, label='ground elevation h_i (m); marker size = parcel weight')
    fig.suptitle(f'Top-down routes, seed {inst.seed} (labels C<id>.<stop order>)')
    path = os.path.join(out_dir, 'top_down_map.png')
    plt.savefig(path, dpi=130, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_altitude_profiles(inst, routes, out_dir):
    _, _, knots = sample_trajectories(routes, inst)
    fig, axes = plt.subplots(N_DRONES, 1, figsize=(11, 10), sharex=True)
    for k, ax in enumerate(axes):
        kt, kp, events = knots[k]
        ax.plot(kt, kp[:, 2], '-', color=DRONE_COLORS[k], lw=1.5)
        ax.axhline(Z_CRUISE, color='k', ls='--', lw=0.8)
        for a, cid in events:
            late = a > inst.l[cid] + 1e-9
            ax.plot(a, inst.zh[cid], 'x' if late else 'o', color='red' if late else 'green',
                    ms=8 if late else 5, mew=2)
            ax.text(a, inst.zh[cid] - 1.8, f'C{cid}', fontsize=7, ha='center')
        ax.set_ylim(0, 14)
        ax.set_ylabel(f'D{k+1} z (m)')
        ax.grid(alpha=0.3)
    axes[-1].set_xlabel('time (s)')
    axes[0].legend(handles=[Line2D([0], [0], color='k', ls='--', label='z_cruise = 12 m'),
                            Line2D([0], [0], marker='o', color='green', ls='', label='on-time arrival'),
                            Line2D([0], [0], marker='x', color='red', ls='', mew=2, label='late arrival')],
                   fontsize=7, loc='upper right', ncol=3)
    fig.suptitle(f'Altitude profiles, 3D detour planner (P3), seed {inst.seed}')
    plt.tight_layout()
    path = os.path.join(out_dir, 'altitude_profiles.png')
    plt.savefig(path, dpi=130)
    plt.close()
    print(f'Saved: {path}')


def plot_cost_breakdown(inst, routes, out_dir):
    ev = [route_eval(r, inst, 'det') for r in routes]
    x = np.arange(N_DRONES)
    fig, axes = plt.subplots(1, 2, figsize=(13, 5))
    hor = np.array([e['hor'] for e in ev])
    vert = np.array([e['vert'] for e in ev])
    axes[0].bar(x, hor, color='steelblue', label='horizontal transit')
    axes[0].bar(x, vert, bottom=hor, color='darkorange', label='vertical (excursions + depot)')
    for xi, a, b in zip(x, hor, vert):
        axes[0].text(xi, a + b + 8, f'{a:.0f}+{b:.0f}', ha='center', fontsize=8)
    axes[0].set_ylabel('path length (m)')
    axes[0].set_title('Distance per drone')
    parts = [('t_hor', 'horizontal flight', 'steelblue'), ('t_vert', 'vertical flight', 'darkorange'),
             ('t_dwell', 'attachment dwell', 'seagreen'), ('t_wait', 'waiting for e_i', 'lightgrey')]
    bottom = np.zeros(N_DRONES)
    for key, lab, col in parts:
        v = np.array([e[key] for e in ev])
        axes[1].bar(x, v, bottom=bottom, color=col, label=lab)
        bottom += v
    for xi, b in zip(x, bottom):
        axes[1].text(xi, b + 2, f'{b:.1f} s', ha='center', fontsize=8)
    axes[1].set_ylabel('time (s)')
    axes[1].set_title('Mission time per drone')
    for ax in axes:
        ax.set_xticks(x)
        ax.set_xticklabels([f'D{k+1}' for k in range(N_DRONES)])
        ax.legend(fontsize=8)
        ax.grid(axis='y', alpha=0.3)
    fig.suptitle(f'Attachment excursion cost breakdown, P3 final routes, seed {inst.seed}')
    plt.tight_layout()
    path = os.path.join(out_dir, 'cost_breakdown.png')
    plt.savefig(path, dpi=130)
    plt.close()
    print(f'Saved: {path}')


def plot_stage_comparison(inst, sols, study, out_dir):
    fig, axes = plt.subplots(1, 3, figsize=(17, 5))
    x = np.arange(3)
    w = 0.26
    cols = {'P1': '#888888', 'P2': '#f58231', 'P3': '#4363d8'}
    for j, p in enumerate(PLANNERS):
        vals = [metrics(sols[p][s], inst)['cost'] for s in STAGES]
        axes[0].bar(x + (j - 1) * w, vals, w, color=cols[p], label=f'{p} {PLANNER_NAMES[p]}')
        for xi, v in zip(x + (j - 1) * w, vals):
            axes[0].text(xi, v + 15, f'{v:.0f}', ha='center', fontsize=7)
        mean = [np.mean([s_['m'][p][s]['cost'] for s_ in study]) for s in STAGES]
        axes[1].bar(x + (j - 1) * w, mean, w, color=cols[p], label=f'{p} {PLANNER_NAMES[p]}')
        for xi, v in zip(x + (j - 1) * w, mean):
            axes[1].text(xi, v + 15, f'{v:.0f}', ha='center', fontsize=7)
    for ax, t in ((axes[0], f'Showcase seed {inst.seed}'),
                  (axes[1], f'Mean over {len(study)} instances')):
        ax.set_xticks(x)
        ax.set_xticklabels([STAGE_NAMES[s] for s in STAGES])
        ax.set_ylabel('true 3D cost J3D')
        ax.set_title(t)
        ax.legend(fontsize=8, loc='lower right')
        ax.grid(axis='y', alpha=0.3)
    seeds = np.array([s['seed'] for s in study])
    j3 = np.array([s['m']['P3']['fin']['cost'] for s in study])
    d1 = np.array([s['m']['P1']['fin']['cost'] for s in study]) - j3
    d2 = np.array([s['m']['P2']['fin']['cost'] for s in study]) - j3
    axes[2].bar(seeds - 0.2, d1, 0.4, color=cols['P1'], label='P1 − P3')
    axes[2].bar(seeds + 0.2, d2, 0.4, color=cols['P2'], label='P2 − P3')
    axes[2].axhline(0, color='k', lw=0.8)
    axes[2].set_yscale('symlog', linthresh=10)
    axes[2].set_xlabel('instance seed')
    axes[2].set_ylabel('final J3D difference (symlog)')
    axes[2].set_title('Final J3D relative to P3 (positive = P3 better)')
    axes[2].legend(fontsize=8)
    axes[2].grid(axis='y', alpha=0.3)
    fig.suptitle('Three planners x three stages, all evaluated in the true 3D model')
    plt.tight_layout()
    path = os.path.join(out_dir, 'stage_comparison.png')
    plt.savefig(path, dpi=130)
    plt.close()
    print(f'Saved: {path}')


def elevation_slack(study, p):
    """(h_i, l_i - arrival) for every pick-up of planner p's final routes."""
    hs, slack = [], []
    for s in study:
        ins = s['inst']
        for r in s['sol'][p]['fin']:
            ev = route_eval(r, ins, 'det')
            for j, cid in enumerate(r):
                hs.append(ins.h[cid])
                slack.append(ins.l[cid] - ev['arr'][j])
    return np.array(hs), np.array(slack)


def plot_time_windows(inst, sols, study, out_dir):
    fig, axes = plt.subplots(1, 2, figsize=(15, 6.5), gridspec_kw=dict(width_ratios=[1.2, 1]))
    ax = axes[0]
    arr1 = {}
    for r in sols['P1']['fin']:
        ev = route_eval(r, inst, 'det')
        for j, cid in enumerate(r):
            arr1[cid] = ev['arr'][j]
    routes = sols['P3']['fin']
    ylab, y = [], 0
    for k, route in enumerate(routes):
        ev = route_eval(route, inst, 'det')
        for j, cid in enumerate(route):
            a = ev['arr'][j]
            ok = a <= inst.l[cid] + 1e-9
            ok1 = arr1[cid] <= inst.l[cid] + 1e-9
            ax.barh(y, inst.l[cid] - inst.e[cid], left=inst.e[cid], height=0.6,
                    color='#cccccc', alpha=0.6, edgecolor='grey')
            ax.plot(a, y, 'D', color='green' if ok else 'red', ms=7, zorder=5)
            ax.plot(arr1[cid], y, 'o', mfc='none', mec='green' if ok1 else 'red',
                    ms=9, mew=1.5, zorder=6)
            ylab.append(f'C{cid} (P3 D{k+1}, h={inst.h[cid]:.1f})')
            y += 1
    ax.set_yticks(range(len(ylab)))
    ax.set_yticklabels(ylab, fontsize=7)
    ax.set_xlabel('time (s)')
    ax.set_xlim(0, 260)          # room on the right for the legend
    ax.set_title(f'Arrival at hover altitude vs window [e_i, l_i], seed {inst.seed}')
    ax.legend(handles=[mpatches.Patch(color='#cccccc', label='window [e_i, l_i]'),
                       Line2D([0], [0], marker='D', color='grey', ls='', label='P3 arrival'),
                       Line2D([0], [0], marker='o', mfc='none', mec='grey', ls='',
                              label='P1 arrival (2D plan flown in 3D)'),
                       Line2D([0], [0], color='green', lw=4, label='green = on time'),
                       Line2D([0], [0], color='red', lw=4, label='red = late')],
                  fontsize=7, loc='lower right')
    ax.grid(axis='x', alpha=0.3)
    ax = axes[1]
    for p, mk, sz in (('P1', 'o', 14), ('P3', 'D', 10)):
        hs, slack = elevation_slack(study, p)
        late = slack < -1e-9
        ax.scatter(hs[~late], slack[~late], s=sz, marker=mk, facecolors='none' if p == 'P1' else 'green',
                   edgecolors='green', alpha=0.5, label=f'{p} on time')
        ax.scatter(hs[late], slack[late], s=3 * sz, marker=mk, facecolors='none' if p == 'P1' else 'red',
                   edgecolors='red', label=f'{p} late ({int(late.sum())})')
    ax.axhline(0, color='k', lw=0.8)
    ax.set_xlabel('customer ground elevation h_i (m)')
    ax.set_ylabel('window slack l_i − arrival (s)')
    ax.set_title(f'All pick-ups of the final routes, {len(study)} study instances')
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'time_windows.png')
    plt.savefig(path, dpi=130)
    plt.close()
    print(f'Saved: {path}')


def save_animation(inst, routes, out_dir):
    ts, trajs, knots = sample_trajectories(routes, inst)
    n = len(ts)
    step = max(1, math.ceil((n - 1) / (GIF_FRAMES - 1)))
    n_frames = math.ceil((n - 1) / step) + 1
    idx = [min(f * step, n - 1) for f in range(n_frames)]
    events = [knots[k][2] for k in range(N_DRONES)]
    fig = plt.figure(figsize=(8, 7))
    ax = fig.add_subplot(111, projection='3d')
    zmax = max(o[2] for o in inst.obs) + 2
    _setup_3d(ax, zmax)
    for (c, r, H) in inst.obs:
        _draw_cylinder(ax, c, r, H, alpha=0.2)
    ax.scatter(*DEPOT_XY, Z_DEPOT, marker='s', color='black', s=60)
    cust = ax.scatter(inst.pos[:, 0], inst.pos[:, 1], inst.zh, s=30 + 50 * inst.q,
                      color='lightblue', edgecolors='grey', depthshade=False)
    dots = [ax.plot([], [], [], 'o', color=DRONE_COLORS[k], ms=7)[0] for k in range(N_DRONES)]
    trails = [ax.plot([], [], [], '-', color=DRONE_COLORS[k], lw=1.1, alpha=0.6)[0]
              for k in range(N_DRONES)]
    texts = [ax.text2D(0.02, 0.95 - 0.04 * k, '', transform=ax.transAxes, fontsize=8,
                       color=DRONE_COLORS[k]) for k in range(N_DRONES)]
    title = ax.set_title('')
    ax.view_init(elev=30, azim=-60)

    def update(f):
        i = idx[f]
        t = ts[i]
        colors = ['lightblue'] * N_CUSTOMERS
        for k in range(N_DRONES):
            tr = trajs[k]
            dots[k].set_data([tr[i, 0]], [tr[i, 1]])
            dots[k].set_3d_properties([tr[i, 2]])
            lo = max(0, i - 400)
            trails[k].set_data(tr[lo:i + 1, 0], tr[lo:i + 1, 1])
            trails[k].set_3d_properties(tr[lo:i + 1, 2])
            pay = 0.0
            for a, cid in events[k]:
                if t >= a + max(0.0, inst.e[cid] - a) + inst.tau[cid] - 1e-9:
                    pay += inst.q[cid]
                    colors[cid] = DRONE_COLORS[k]
            texts[k].set_text(f'D{k+1}: {pay:.2f} kg, z={tr[i, 2]:.1f} m' if routes[k] else '')
        cust.set_facecolors(colors)
        title.set_text(f'S037 3D Reverse Logistics (P3), t = {t:.1f} s')
        return dots + trails + texts

    anim = FuncAnimation(fig, update, frames=n_frames, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    anim.save(path, writer=PillowWriter(fps=15), dpi=80)
    plt.close()
    print(f'Saved: {path} ({n_frames} frames, step {step} samples = {step*DT:.1f} s)')


# ── Main ───────────────────────────────────────────────────────────────────────
def _fmt_routes(rs):
    return '[' + ', '.join('[' + ','.join(str(c) for c in r) + ']' for r in rs) + ']'


def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    show, sols, study = run_simulation()
    I = show
    print(f'=== S037 3D Reverse Logistics — showcase instance (seed {SHOWCASE_SEED}) ===')
    print(f'Total parcel weight {I.q.sum():.2f} kg (fleet capacity {N_DRONES*Q_MAX:.1f} kg)')
    print(f'Ground elevation h: {I.h.min():.2f}-{I.h.max():.2f} m, '
          f'excursion depth dz = z_cruise - h - 1.2: {I.dz.min():.2f}-{I.dz.max():.2f} m')
    for k, (c, r, H) in enumerate(I.obs):
        print(f'  Obstacle {k}: centre ({c[0]:.1f}, {c[1]:.1f}), r = {r:.1f} m, H = {H:.1f} m')
    n_pairs = (N_CUSTOMERS + 1) * N_CUSTOMERS // 2
    n_block = sum(1 for i, j in itertools.combinations(range(N_CUSTOMERS + 1), 2)
                  if len(I.paths[(i, j)]) > 2)
    print(f'Node pairs whose straight leg violates a buffer: {n_block}/{n_pairs} '
          f'(visibility-graph fallback used for {I.n_fallback})')
    st_block = [sum(1 for i, j in itertools.combinations(range(N_CUSTOMERS + 1), 2)
                    if len(s['inst'].paths[(i, j)]) > 2) for s in study]
    st_fb = [s['inst'].n_fallback for s in study]
    print(f'Study: blocked node pairs per instance mean {np.mean(st_block):.1f}/{n_pairs} '
          f'(range {min(st_block)}-{max(st_block)}); card single-waypoint rule failed and the '
          f'visibility-graph fallback was used for {sum(st_fb)} of {N_INSTANCES*n_pairs} pairs')
    print(f'\n{"Planner":<14}{"Stage":<15}{"Hor(m)":>9}{"Vert(m)":>9}{"Late(s)":>9}'
          f'{"#late":>6}{"Detours":>8}{"Makespan":>9}{"J3D":>9}{"J_own":>10}')
    for p in PLANNERS:
        for s in STAGES:
            m = metrics(sols[p][s], I)
            own = sum(route_cost(r, I, PLANNER_MODE[p]) for r in sols[p][s])
            print(f'{p+" "+PLANNER_NAMES[p]:<14}{STAGE_NAMES[s]:<15}{m["hor"]:>9.1f}{m["vert"]:>9.1f}'
                  f'{m["late"]:>9.1f}{m["n_late"]:>6d}{m["n_detour"]:>8d}{m["makespan"]:>9.1f}'
                  f'{m["cost"]:>9.1f}{own:>10.1f}')
    for p in PLANNERS:
        for s in STAGES:
            print(f'  {p} {STAGE_NAMES[s]:<14} routes: {_fmt_routes(sols[p][s])}')
    m2 = metrics(sols['P2']['fin'], I, 'pen')
    print(f'P2 final: planned straight-leg penetration W = {m2["W"]:.2f} m')
    print(f'P1 final makespan: 2D model {metrics(sols["P1"]["fin"], I, "2d")["makespan"]:.1f} s, '
          f'true 3D {metrics(sols["P1"]["fin"], I)["makespan"]:.1f} s')

    print('\n=== P3 final routes, per drone (true 3D) ===')
    print(f'{"Drone":<6}{"Stops":>6}{"Hor(m)":>9}{"Vert(m)":>9}{"E(m-eq)":>9}{"Load(kg)":>9}'
          f'{"t_hor":>7}{"t_vert":>7}{"t_dwell":>8}{"t_wait":>7}{"Return(s)":>10}{"Late(s)":>8}')
    for k, r in enumerate(sols['P3']['fin']):
        e = route_eval(r, I, 'det')
        print(f'D{k+1:<5}{len(r):>6}{e["hor"]:>9.1f}{e["vert"]:>9.1f}{e["E"]:>9.1f}'
              f'{payload(r, I):>9.2f}{e["t_hor"]:>7.1f}{e["t_vert"]:>7.1f}{e["t_dwell"]:>8.1f}'
              f'{e["t_wait"]:>7.1f}{e["t_end"]:>10.1f}{e["late"]:>8.1f}')
    print('\n=== Customers (P3 final vs P1 final, arrival at hover altitude) ===')
    print(f'{"Cust":<5}{"h(m)":>6}{"dz(m)":>7}{"q(kg)":>7}{"tau(s)":>7}{"e":>7}{"l":>7}'
          f'{"arr P3":>8}{"arr P1":>8}{"start P1 2D-model":>18}')
    arr = {}
    for p in ('P1', 'P3'):
        for r in sols[p]['fin']:
            e3 = route_eval(r, I, 'det')['arr']
            e2 = route_eval(r, I, '2d')['arr']
            for j, c in enumerate(r):
                arr[(p, c)] = (e3[j], e2[j])
    for c in range(N_CUSTOMERS):
        print(f'C{c:<4}{I.h[c]:>6.2f}{I.dz[c]:>7.2f}{I.q[c]:>7.2f}{I.tau[c]:>7.2f}{I.e[c]:>7.1f}'
              f'{I.l[c]:>7.1f}{arr[("P3", c)][0]:>8.1f}{arr[("P1", c)][0]:>8.1f}{arr[("P1", c)][1]:>18.1f}')

    # ── Study ────────────────────────────────────────────────────────────
    print(f'\n=== Study: {N_INSTANCES} instances (seeds 0..{N_INSTANCES-1}), '
          'same draws for every planner, final J3D ===')
    print('seed   J3D_P1   J3D_P2   J3D_P3  late_P1 late_P2 late_P3  det_P1 det_P2 det_P3')
    J = {p: np.array([s['m'][p]['fin']['cost'] for s in study]) for p in PLANNERS}
    LT = {p: np.array([s['m'][p]['fin']['late'] for s in study]) for p in PLANNERS}
    NL = {p: np.array([s['m'][p]['fin']['n_late'] for s in study]) for p in PLANNERS}
    ND = {p: np.array([s['m'][p]['fin']['n_detour'] for s in study]) for p in PLANNERS}
    for i, s in enumerate(study):
        print(f'{s["seed"]:>4} {J["P1"][i]:>8.1f} {J["P2"][i]:>8.1f} {J["P3"][i]:>8.1f}'
              f' {LT["P1"][i]:>7.1f} {LT["P2"][i]:>7.1f} {LT["P3"][i]:>7.1f}'
              f' {ND["P1"][i]:>7d} {ND["P2"][i]:>6d} {ND["P3"][i]:>6d}')
    for p in PLANNERS:
        print(f'{p} {PLANNER_NAMES[p]:<11}: J3D mean {J[p].mean():.1f} +- {J[p].std():.1f} (std), '
              f'median {np.median(J[p]):.1f}, range {J[p].min():.1f}-{J[p].max():.1f}; '
              f'instances with lateness {int((LT[p] > 1e-9).sum())}/{N_INSTANCES}, '
              f'late pick-ups {int(NL[p].sum())}/{N_INSTANCES*N_CUSTOMERS}, '
              f'total lateness {LT[p].sum():.1f} s; detoured legs mean {ND[p].mean():.2f}')
    for a, b in (('P3', 'P1'), ('P3', 'P2'), ('P2', 'P1')):
        d = J[b] - J[a]
        print(f'{a} vs {b}: {a} lower in {int((d > 1e-6).sum())}, higher in '
              f'{int((d < -1e-6).sum())}, tie in {int((np.abs(d) <= 1e-6).sum())}; '
              f'mean {b}-{a} = {d.mean():.1f}, median {np.median(d):.1f}, '
              f'max {d.max():.1f}, min {d.min():.1f}')
    # stage contributions for P3 (card's three-stage structure)
    for p in PLANNERS:
        cw = np.array([s['m'][p]['cw']['cost'] for s in study])
        tw = np.array([s['m'][p]['two']['cost'] for s in study])
        fn = J[p]
        print(f'{p} stages (J3D): CW {cw.mean():.1f} -> 2-opt {tw.mean():.1f} -> Or-opt {fn.mean():.1f}; '
              f'2-opt lowers J3D in {int((tw < cw - 1e-6).sum())}/{N_INSTANCES}, raises in '
              f'{int((tw > cw + 1e-6).sum())}; Or-opt lowers in {int((fn < tw - 1e-6).sum())}/{N_INSTANCES}, '
              f'raises in {int((fn > tw + 1e-6).sum())}')
    # card claim: 3D savings re-rank merges -> check against 2D CW
    same_cw_nodet = 0
    same_cw_p3 = 0
    for s in study + [dict(inst=show, sol=sols)]:
        ins = s['inst']
        r2d = s['sol']['P1']['cw']
        # 3D cost without obstacles: straight legs, vertical excursions, nu = 0
        saved_W = ins.W_st.copy()
        ins.W_st[:] = 0.0
        r3_flat = clarke_wright(ins, 'pen')
        ins.W_st[:] = saved_W
        same_cw_nodet += int(r3_flat == r2d)
        same_cw_p3 += int(s['sol']['P3']['cw'] == r2d)
    print(f'Clarke-Wright with 3D vertical cost but no obstacles gives the 2D CW routes in '
          f'{same_cw_nodet}/{N_INSTANCES+1} instances; with detoured legs (P3) in {same_cw_p3}/{N_INSTANCES+1}')
    # lateness of the 2D plans: how much comes from vertical motion + dwell alone
    late_st = sum(metrics(s['sol']['P1']['fin'], s['inst'], 'pen')['late'] for s in study)
    late_2d = sum(metrics(s['sol']['P1']['fin'], s['inst'], '2d')['late'] for s in study)
    print(f'P1 final lateness, study total: 2D model {late_2d:.1f} s; 3D timing on straight legs '
          f'(vertical motion + dwell, no detours) {late_st:.1f} s; true 3D with detours '
          f'{LT["P1"].sum():.1f} s')
    # elevation vs lateness (card claim)
    for p in ('P1', 'P3'):
        hs, slack = elevation_slack(study, p)
        lat = -slack
        lo, hi = hs < H_MAX / 2, hs >= H_MAX / 2
        print(f'{p} final pick-ups: h < 4 m: {int(lo.sum())} pick-ups, {int((lat[lo] > 1e-9).sum())} late; '
              f'h >= 4 m: {int(hi.sum())} pick-ups, {int((lat[hi] > 1e-9).sum())} late; '
              f'corr(h, arrival - l) = {np.corrcoef(hs, lat)[0, 1]:.3f}')
    print(f'Excursion time 2 dz / v_z per stop: {2*(Z_CRUISE-Z_OFFSET-H_MAX)/V_Z:.1f} s (h = 8 m) '
          f'to {2*(Z_CRUISE-Z_OFFSET)/V_Z:.1f} s (h = 0 m)')
    allE = [s['m'][p]['fin']['E_max'] for s in study for p in PLANNERS]
    allQ = [payload(r, s['inst']) for s in study for p in PLANNERS for r in s['sol'][p]['fin']]
    allE += [metrics(sols[p]['fin'], I)['E_max'] for p in PLANNERS]
    allQ += [payload(r, I) for p in PLANNERS for r in sols[p]['fin']]
    print(f'Physical checks passed: max route energy {max(allE):.1f} m-eq (R_max {R_MAX:.0f}), '
          f'max load {max(allQ):.2f} kg (Q_max {Q_MAX}), all trajectories clear obstacle buffers, '
          f'speeds <= {V_C}/{V_Z} m/s')

    # Contrast instance for the route figures: the study seed whose P1 - P3 gap
    # is the (lower) median among the seeds where the two planners differ.
    d = J['P1'] - J['P3']
    differ = [i for i in range(N_INSTANCES) if abs(d[i]) > 1e-6]
    order = sorted(differ, key=lambda i: (d[i], i))
    ci = order[(len(order) - 1) // 2]
    C = study[ci]
    print(f'\n=== Contrast instance for the figures: seed {C["seed"]} (median P1-P3 gap '
          f'{d[ci]:.1f} among {len(differ)} seeds where P1 and P3 differ) ===')
    for p in PLANNERS:
        m = C['m'][p]['fin']
        print(f'  {p} final: J3D {m["cost"]:.1f}, horizontal {m["hor"]:.1f} m, vertical {m["vert"]:.1f} m, '
              f'lateness {m["late"]:.1f} s ({m["n_late"]} late), detoured legs {m["n_detour"]}, '
              f'makespan {m["makespan"]:.1f} s, routes {_fmt_routes(C["sol"][p]["fin"])}')
    for k, (c, r, H) in enumerate(C['inst'].obs):
        print(f'  Obstacle {k}: centre ({c[0]:.1f}, {c[1]:.1f}), r = {r:.1f} m, H = {H:.1f} m')

    plot_routes_3d([(I, sols, 'P3'), (C['inst'], C['sol'], 'P1'), (C['inst'], C['sol'], 'P3')],
                   OUTPUT_DIR)
    plot_top_down(C['inst'], C['sol'], OUTPUT_DIR)
    plot_altitude_profiles(I, sols['P3']['fin'], OUTPUT_DIR)
    plot_cost_breakdown(I, sols['P3']['fin'], OUTPUT_DIR)
    plot_stage_comparison(I, sols, study, OUTPUT_DIR)
    plot_time_windows(C['inst'], C['sol'], study, OUTPUT_DIR)
    save_animation(I, sols['P3']['fin'], OUTPUT_DIR)


if __name__ == '__main__':
    main()
