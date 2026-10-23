"""
S033 3D Upgrade — Online Order Insertion
=========================================
K = 4 drones fly pre-planned routes (12 waypoints each, then back to the depot).
Each drone owns an altitude corridor z_k in {3, 5, 7, 9} m and its planned
waypoints lie in that corridor.  Between t = 10 s and 80 s, 8 online orders
arrive at full 3-D delivery points q = (x, y, z_new) with z_new in [1.5, 10] m.

Dispatcher (card):
  * 3-D cheapest insertion  Delta3D = |w_i - q| + |q - w_i+1| - |w_i - w_i+1|
  * corridor penalty         phi_k = gamma * max(0, |z_new - z_k| - dz_tol)
  * corridor-return waypoint (x_new, y_new, z_k) after q when |z_new - z_k| > dz_tol
  * feasibility: battery, deadline, and a corridor-conflict pre-screen of the
    candidate's projected trajectory against the other drones over
    [t_now, t_arrive + 30 s]  (|dz| < h_sep and horizontal distance < r_sep)
  * if no drone is feasible the order is queued on the drone that finishes its
    route earliest (served late), or rejected if no drone has the battery.

Strategies (same instances for all):
  cheapest_3d       - card proposal: min Delta3D + phi over the feasible drones
  nearest_3d        - feasible drone closest (3-D) to q
  corridor_pref     - feasible drone whose corridor z_k is closest to z_new
  least_loaded      - feasible drone with the fewest remaining waypoints
  cheapest_planar   - 2-D baseline: ranks slots and drones by the x-y detour only
                      (no phi); feasibility is still checked in 3-D
  cheapest_3d_nochk - cheapest_3d without the corridor-conflict pre-screen
                      (ablation: what does the pre-screen buy?)

Each run lasts until every drone is back at the depot.  Losses of separation
(LoS) are measured in the simulation at every 0.1 s step.  The terminal area
(horizontal distance < R_TERM from the depot) is reported separately because
all four drones take off from and land at the same depot.

Outputs (outputs/02_logistics_delivery/3d/s033_3d_online_order/):
  trajectory_3d.png, altitude_profiles.png, detour_comparison.png,
  vertical_breakdown.png, load_balance.png, animation.gif

Usage:
    conda activate drones
    python src/02_logistics_delivery/3d/s033_3d_online_order.py
"""

import matplotlib
matplotlib.use('Agg')

import os
import math
from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.lines as mlines
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from matplotlib.animation import FuncAnimation, PillowWriter

OUTPUT_DIR = os.path.normpath(os.path.join(
    os.path.dirname(__file__), '..', '..', '..',
    'outputs', '02_logistics_delivery', '3d', 's033_3d_online_order',
))

# ── Parameters (card) ─────────────────────────────────────────────
K              = 4
V_DRONE        = 8.0          # m/s
B_FULL         = 10_000.0     # m full-battery range
CORRIDORS      = np.array([3.0, 5.0, 7.0, 9.0])   # m, z_k of D1..D4
Z_NEW_LO       = 1.5          # m delivery altitude range
Z_NEW_HI       = 10.0
DZ_TOL         = 1.5          # m corridor tolerance
GAMMA          = 1.2          # corridor penalty multiplier
H_SEP          = 1.5          # m vertical separation
R_SEP          = 20.0         # m horizontal conflict radius
CHECK_PAD      = 30.0         # s conflict-check horizon beyond t_arrive
N_WP_INIT      = 12
N_INSERTIONS   = 8
T_ORDER_LO     = 10.0
T_ORDER_HI     = 80.0
DEADLINE_SLACK = 30.0
DT             = 0.1
ARENA_LO       = -500.0
ARENA_HI       = 500.0
DEPOT          = np.array([0.0, 0.0, 5.0])
# ── Added (not on the card) ───────────────────────────────────────
B_INIT_LO      = 0.75         # initial battery ~ B_FULL * U(0.75, 1.0)   (2D S033)
T_CAP          = 3000.0       # s safety cap
R_TERM         = 20.0         # m depot pad radius (= r_sep); LoS there reported apart
SEED0          = 0
N_SEEDS        = 100          # Monte-Carlo: first 100 battery-feasible seeds from SEED0 (0-115, 16 skipped)

DRONE_COLORS = ['tab:red', 'tab:blue', 'tab:green', 'tab:orange']
STRATEGIES = ['cheapest_3d', 'nearest_3d', 'corridor_pref', 'least_loaded',
              'cheapest_planar', 'cheapest_3d_nochk']
STRAT_LABEL = {'cheapest_3d': '3D cheapest', 'nearest_3d': 'Nearest 3D',
               'corridor_pref': 'Corridor-pref.', 'least_loaded': 'Least loaded',
               'cheapest_planar': 'Planar cheapest', 'cheapest_3d_nochk': '3D cheapest\nno conflict chk'}
STRAT_COLORS = ['#2ecc71', '#3498db', '#9b59b6', '#e67e22', '#7f8c8d', '#c0392b']

RET_ID = -2          # route id of a synthetic corridor-return waypoint
PLAN_ID = -1         # pre-planned customer waypoint
TRANSIT_ID = -3      # vertical climb / descent point (0, 0, z_k) above the depot
DEPOT_ID = -4        # final depot landing
NON_STOP = (RET_ID, TRANSIT_ID)


# ── Geometry helpers ──────────────────────────────────────────────

def route_length(pts) -> float:
    P = np.asarray(pts)
    return float(np.sum(np.linalg.norm(np.diff(P, axis=0), axis=1))) if len(P) > 1 else 0.0


def ins_delta(a, q, b) -> float:
    return float(np.linalg.norm(a - q) + np.linalg.norm(q - b) - np.linalg.norm(b - a))


def ins_delta_xy(a, q, b) -> float:
    return ins_delta(a[:2], q[:2], b[:2])


def ins_delta_z(a, q, b) -> float:
    """Card's vertical detour term |z_i - z_q| + |z_q - z_i+1| - |z_i - z_i+1|."""
    return abs(a[2] - q[2]) + abs(q[2] - b[2]) - abs(a[2] - b[2])


def point_along(pts, s):
    for i in range(len(pts) - 1):
        seg = float(np.linalg.norm(pts[i + 1] - pts[i]))
        if s <= seg:
            return pts[i] + (pts[i + 1] - pts[i]) * (s / max(seg, 1e-12))
        s -= seg
    return pts[-1].copy()


def project(pts, speed, trel):
    """Positions at relative times trel along polyline pts flown at constant speed."""
    P = np.asarray(pts)
    if len(P) == 1:
        return np.repeat(P, len(trel), axis=0)
    cum = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))])
    s = np.minimum(speed * trel, cum[-1])
    return np.column_stack([np.interp(s, cum, P[:, j]) for j in range(3)])


def conflict_mask(A, B):
    """LoS mask between two trajectories outside the terminal area."""
    dz = np.abs(A[:, 2] - B[:, 2])
    dh = np.linalg.norm(A[:, :2] - B[:, :2], axis=1)
    term = (np.linalg.norm(A[:, :2] - DEPOT[:2], axis=1) < R_TERM) | \
           (np.linalg.norm(B[:, :2] - DEPOT[:2], axis=1) < R_TERM)
    los = (dz < H_SEP) & (dh < R_SEP)
    return los & ~term, los & term


# ── Data classes ──────────────────────────────────────────────────

@dataclass
class Order:
    oid: int
    loc: np.ndarray
    t_arrive: float
    t_deadline: float


@dataclass
class Drone:
    id: int
    zc: float
    pos: np.ndarray
    route: List[np.ndarray]
    route_ids: List[int]
    battery_rem: float
    speed: float = V_DRONE
    initial_route: List[np.ndarray] = field(default_factory=list)
    path: List[np.ndarray] = field(default_factory=list)
    flown: float = 0.0
    climb: float = 0.0           # total |dz| flown

    def full_pts(self):
        return [self.pos] + self.route

    def remaining_length(self):
        return route_length(self.full_pts())

    def needs_ret(self, q):
        return abs(q[2] - self.zc) > DZ_TOL

    def spliced(self, q, idx):
        """Full point list after inserting q (and its corridor-return waypoint) at slot idx."""
        pts = self.full_pts()
        new = pts[:idx + 1] + [q]
        if self.needs_ret(q):
            new.append(np.array([q[0], q[1], self.zc]))
        return new + pts[idx + 1:]

    def slot_allowed(self, idx):
        """Segment pts[idx] -> pts[idx+1] may take an insertion unless it is a q -> return
        leg or the vertical descent onto the depot pad."""
        end = self.route_ids[idx]
        start = self.route_ids[idx - 1] if idx > 0 else None
        return end != RET_ID and not (end == DEPOT_ID and start == TRANSIT_ID)

    def last_slot(self):
        ok = [i for i in range(len(self.route)) if self.slot_allowed(i)]
        return ok[-1] if ok else None

    def candidate_slots(self, q, t_now, t_deadline):
        """Battery- and deadline-feasible slots (idx, Delta3D, Delta_xy, true added length, t_arr)."""
        pts = self.full_pts()
        n_seg = len(pts) - 1
        L = self.remaining_length()
        out, prefix = [], 0.0
        ret = np.array([q[0], q[1], self.zc]) if self.needs_ret(q) else None
        for idx in range(n_seg):
            if idx > 0:
                prefix += float(np.linalg.norm(pts[idx] - pts[idx - 1]))
            if not self.slot_allowed(idx):
                continue
            a, b = pts[idx], pts[idx + 1]
            d3 = ins_delta(a, q, b)
            if ret is None:
                dtrue = d3
            else:
                dtrue = float(np.linalg.norm(a - q) + np.linalg.norm(q - ret)
                              + np.linalg.norm(ret - b) - np.linalg.norm(b - a))
            t_arr = t_now + (prefix + float(np.linalg.norm(a - q))) / self.speed
            if L + dtrue <= self.battery_rem and t_arr <= t_deadline:
                out.append((idx, d3, ins_delta_xy(a, q, b), dtrue, t_arr))
        return out

    def insert(self, q, idx, oid):
        self.route.insert(idx, q.copy())
        self.route_ids.insert(idx, oid)
        if self.needs_ret(q):
            self.route.insert(idx + 1, np.array([q[0], q[1], self.zc]))
            self.route_ids.insert(idx + 1, RET_ID)

    def step(self, dt, t, deliveries):
        budget = self.speed * dt
        while budget > 1e-12 and self.route:
            tgt = self.route[0]
            d = math.dist(tgt, self.pos)
            if d <= budget:
                self.climb += abs(tgt[2] - self.pos[2])
                self.pos = tgt.copy()
                budget -= d
                self.flown += d
                self.battery_rem -= d
                oid = self.route_ids.pop(0)
                self.route.pop(0)
                if oid >= 0:
                    deliveries[oid] = t + (self.speed * dt - budget) / self.speed
            else:
                new = self.pos + (tgt - self.pos) / d * budget
                self.climb += abs(new[2] - self.pos[2])
                self.pos = new
                self.flown += budget
                self.battery_rem -= budget
                budget = 0.0
        assert self.battery_rem >= -1e-6, 'flying on an empty battery'
        assert self.pos[2] > 0.0, 'underground'


# ── Instance ──────────────────────────────────────────────────────

def make_instance(seed):
    """Same draw order as 2D S033 (identical x-y layout, batteries and arrival times).
    Planned waypoints are moved into the drone's corridor; the 2D altitude draw
    U(2, 8) of an order is mapped affinely onto z_new ~ U(1.5, 10)."""
    rng = np.random.default_rng(seed)

    def rand_wp():
        xy = rng.uniform(ARENA_LO, ARENA_HI, 2)
        return np.array([xy[0], xy[1], rng.uniform(2.0, 8.0)])

    fleet = []
    for k in range(K):
        wps = []
        for _ in range(N_WP_INIT):
            w = rand_wp()
            w[2] = CORRIDORS[k]
            wps.append(w)
        fleet.append((wps, B_FULL * rng.uniform(B_INIT_LO, 1.0)))
    arrivals = np.sort(rng.uniform(T_ORDER_LO, T_ORDER_HI, N_INSERTIONS))
    orders = []
    for n, t_a in enumerate(arrivals):
        loc = rand_wp()
        loc[2] = Z_NEW_LO + (loc[2] - 2.0) / 6.0 * (Z_NEW_HI - Z_NEW_LO)
        eta = min(np.linalg.norm(point_along(planned_route(k, w), V_DRONE * t_a) - loc)
                  for k, (w, _) in enumerate(fleet)) / V_DRONE
        orders.append(Order(n, loc, float(t_a), float(t_a + eta + DEADLINE_SLACK)))
    return fleet, orders


def planned_feasible(seed):
    fleet, _ = make_instance(seed)
    return all(route_length(planned_route(k, w)) <= b for k, (w, b) in enumerate(fleet))


def valid_seeds(n):
    """First n seeds (from SEED0) whose pre-planned routes fit every drone's battery.
    About 1 seed in 7 draws a battery U(0.75, 1) x 10 km shorter than a ~7 km plan."""
    seeds, skipped, s = [], [], SEED0
    while len(seeds) < n:
        (seeds if planned_feasible(s) else skipped).append(s)
        s += 1
    return seeds, skipped


def planned_route(k, wps):
    """Full planned point list: depot, vertical climb to the corridor above the pad,
    the 12 corridor waypoints, back above the pad, vertical descent."""
    above = np.array([DEPOT[0], DEPOT[1], CORRIDORS[k]])
    t = [above] if abs(CORRIDORS[k] - DEPOT[2]) > 1e-9 else []   # D2 cruises at depot height
    return [p.copy() for p in [DEPOT] + t + list(wps) + t + [DEPOT]]


def planned_ids(k):
    """Route ids matching planned_route(k)[1:]."""
    has_transit = abs(CORRIDORS[k] - DEPOT[2]) > 1e-9
    t = [TRANSIT_ID] if has_transit else []
    return t + [PLAN_ID] * N_WP_INIT + t + [DEPOT_ID]


def build_drones(fleet):
    out = []
    for k, (wps, bat) in enumerate(fleet):
        pts = planned_route(k, wps)
        d = Drone(id=k, zc=float(CORRIDORS[k]), pos=DEPOT.copy(),
                  route=[p.copy() for p in pts[1:]], route_ids=planned_ids(k),
                  battery_rem=bat)
        assert len(d.route) == len(d.route_ids)
        d.initial_route = [p.copy() for p in pts]
        out.append(d)
    return out


# ── Dispatcher ────────────────────────────────────────────────────

def phi(d, q):
    return GAMMA * max(0.0, abs(q[2] - d.zc) - DZ_TOL)


def conflict_free(drones, d, q, idx, t_arr, t_now, others_cache):
    """Pre-screen the candidate's projected trajectory over [t_now, t_arr + CHECK_PAD]."""
    horizon = t_arr + CHECK_PAD - t_now
    trel = np.arange(0.0, horizon + 1e-9, DT)
    key = len(trel)
    if key not in others_cache:
        others_cache[key] = {o.id: project(o.full_pts(), o.speed, trel) for o in drones}
    A = project(d.spliced(q, idx), d.speed, trel)
    for o in drones:
        if o.id == d.id:
            continue
        en_route, _ = conflict_mask(A, others_cache[key][o.id])
        if en_route.any():
            return False
    return True


def dispatch(strategy, drones, order, t_now, stats):
    """Return (drone, slot) or None. For each drone the slot is its cheapest feasible one
    (by the strategy's own cost metric); the strategy then picks the drone."""
    q = order.loc
    planar = strategy == 'cheapest_planar'
    check = strategy != 'cheapest_3d_nochk'
    cache = {}
    feas = []
    for d in drones:
        slots = d.candidate_slots(q, t_now, order.t_deadline)
        slots.sort(key=lambda s: (s[2] if planar else s[1], s[0]))
        chosen = None
        for s in slots:
            if not check or conflict_free(drones, d, q, s[0], s[4], t_now, cache):
                chosen = s
                break
            stats['slot_conflicts'] += 1
        if slots and chosen is None:
            stats['drone_conflict_rejections'] += 1
        if chosen is not None:
            feas.append((d, chosen))
    if not feas:
        return None
    if strategy in ('cheapest_3d', 'cheapest_3d_nochk'):
        key = lambda f: (f[1][1] + phi(f[0], q), f[0].id)
    elif strategy == 'cheapest_planar':
        key = lambda f: (f[1][2], f[0].id)
    elif strategy == 'nearest_3d':
        key = lambda f: (float(np.linalg.norm(f[0].pos - q)), f[0].id)
    elif strategy == 'corridor_pref':
        key = lambda f: (abs(f[0].zc - q[2]), f[1][1] + phi(f[0], q), f[0].id)
    else:
        key = lambda f: (len([i for i in f[0].route_ids if i not in NON_STOP]), f[0].id)
    return min(feas, key=key)


def queue_fallback(drones, order):
    """No feasible drone: append before the depot return of the earliest-finishing drone
    that has the battery (served late)."""
    q = order.loc
    cands = []
    for d in drones:
        idx = d.last_slot() if d.route else None
        if idx is None:
            continue
        pts = d.full_pts()
        a, b = pts[idx], pts[idx + 1]
        added = route_length(d.spliced(q, idx)) - d.remaining_length()
        if d.remaining_length() + added <= d.battery_rem:
            t_arr = 0.0
            cands.append((d.remaining_length() / d.speed, d.id, d,
                          (idx, ins_delta(a, q, b), ins_delta_xy(a, q, b), added, t_arr)))
    if not cands:
        return None
    _, _, d, slot = min(cands, key=lambda c: (c[0], c[1]))
    return d, slot


# ── Simulation ────────────────────────────────────────────────────

def run_simulation(strategy='cheapest_3d', seed=SEED0, record=False):
    fleet, orders = make_instance(seed)
    drones = build_drones(fleet)
    planned_len = sum(route_length(d.initial_route) for d in drones)
    planned_climb = sum(float(np.sum(np.abs(np.diff(np.array(d.initial_route)[:, 2]))))
                        for d in drones)
    stats = {'slot_conflicts': 0, 'drone_conflict_rejections': 0}
    pending = sorted(orders, key=lambda o: o.t_arrive)
    assign, events, rejected, deliveries = {}, [], [], {}
    mon_pos, mon_fly = [], []
    t, times = 0.0, []
    while (any(d.route for d in drones) or pending) and t < T_CAP:
        while pending and pending[0].t_arrive <= t + 1e-9:
            o = pending.pop(0)
            res = dispatch(strategy, drones, o, t, stats)
            mode = 'online'
            if res is None:
                res = queue_fallback(drones, o)
                mode = 'queued'
            if res is None:
                rejected.append(o.oid)
                continue
            d, (idx, d3, dxy, dtrue, t_arr) = res
            t_end = t_arr + CHECK_PAD if mode == 'online' else None
            a, b = d.full_pts()[idx], d.full_pts()[idx + 1]
            L0 = d.remaining_length()
            d.insert(o.loc, idx, o.oid)
            assert abs(d.remaining_length() - L0 - dtrue) < 1e-6
            assert d.remaining_length() <= d.battery_rem + 1e-6
            assign[o.oid] = (d.id, mode, dtrue)
            events.append(dict(t=t, t_end=t_end, oid=o.oid, drone=d.id, mode=mode, d3=d3, dxy=dxy,
                               dz=ins_delta_z(a, o.loc, b), dtrue=dtrue, phi=phi(d, o.loc),
                               ret=d.needs_ret(o.loc)))
        if record:
            times.append(t)
            for d in drones:
                d.path.append(d.pos.copy())
        for d in drones:
            d.step(DT, t, deliveries)
        t = round(t + DT, 10)
        mon_pos.append([d.pos for d in drones])           # state after each step
        mon_fly.append([bool(d.route) for d in drones])
    # ── separation / corridor monitoring over every 0.1 s step (vectorised) ──
    P = np.array(mon_pos)                                  # (n, K, 3)
    fly = np.array(mon_fly)                                # (n, K)
    interm = np.linalg.norm(P[:, :, :2] - DEPOT[:2], axis=2) < R_TERM
    i0, i1 = np.triu_indices(K, 1)
    dz = np.abs(P[:, i0, 2] - P[:, i1, 2])
    dh = np.linalg.norm(P[:, i0, :2] - P[:, i1, :2], axis=2)
    los = (dz < H_SEP) & (dh < R_SEP)
    tm = interm[:, i0] | interm[:, i1]
    en, te = los & ~tm, los & tm
    rise = lambda m: int(np.sum(m[0])) + int(np.sum(m[1:] & ~m[:-1]))
    los_steps, term_steps = int(en.sum()), int(te.sum())
    los_events, term_events = rise(en), rise(te)
    exc = np.abs(P[:, :, 2] - CORRIDORS[None, :])
    los_log = []           # (t_start, duration, drone a, drone b, |z - z_k| of a and b at start)
    for p in range(len(i0)):
        m_ = en[:, p]
        starts = np.where(m_ & ~np.concatenate([[False], m_[:-1]]))[0]
        for s0 in starts:
            s1 = s0
            while s1 + 1 < len(m_) and m_[s1 + 1]:
                s1 += 1
            los_log.append((round((s0 + 1) * DT, 1), round((s1 - s0 + 1) * DT, 1), int(i0[p]),
                            int(i1[p]), float(exc[s0, i0[p]]), float(exc[s0, i1[p]])))
    active = fly & ~interm
    excursion_steps = int(np.sum((exc > DZ_TOL) & active))
    max_excursion = float(np.max(np.where(active, exc, 0.0)))
    if record:
        times.append(t)
        for d in drones:
            d.path.append(d.pos.copy())

    flown = sum(d.flown for d in drones)
    sum_true = sum(v[2] for v in assign.values())
    assert abs((flown - planned_len) - sum_true) < 1e-4, 'added flown != inserted length'
    on_time = sum(1 for o in orders if o.oid in deliveries and deliveries[o.oid] <= o.t_deadline)
    late = sum(1 for o in orders if o.oid in deliveries and deliveries[o.oid] > o.t_deadline)
    return {
        'strategy': strategy, 'seed': seed, 'drones': drones, 'orders': orders,
        'assign': assign, 'events': events, 'deliveries': deliveries,
        'n_online': sum(1 for v in assign.values() if v[1] == 'online'),
        'n_queued': sum(1 for v in assign.values() if v[1] == 'queued'),
        'n_rejected': len(rejected), 'on_time': on_time, 'late': late,
        'added_flown': flown - planned_len,
        'added_climb': sum(d.climb for d in drones) - planned_climb,
        'detour_xy': float(sum(e['dxy'] for e in events)),
        'detour_3d': float(sum(e['d3'] for e in events)),
        'cost_card': float(sum(e['d3'] + e['phi'] for e in events)),
        'n_ret': sum(1 for e in events if e['ret']),
        'los_events': los_events, 'los_s': los_steps * DT, 'los_log': los_log,
        'term_events': term_events, 'term_s': term_steps * DT,
        'slot_conflicts': stats['slot_conflicts'],
        'drone_conflict_rejections': stats['drone_conflict_rejections'],
        'excursion_s': excursion_steps * DT, 'max_excursion': max_excursion,
        'makespan': t, 'min_battery': min(d.battery_rem for d in drones),
        'per_drone': [sum(1 for v in assign.values() if v[0] == k) for k in range(K)],
        'choice': {oid: v[0] for oid, v in assign.items()},
        'times': np.array(times) if record else None,
    }


# ── Plots ─────────────────────────────────────────────────────────

def plot_trajectory_3d(res, out_dir):
    fig = plt.figure(figsize=(12, 9))
    ax = fig.add_subplot(111, projection='3d')
    for d in res['drones']:
        c = DRONE_COLORS[d.id]
        ir = np.array(d.initial_route); P = np.array(d.path)
        ax.plot(ir[:, 0], ir[:, 1], ir[:, 2], '--', color=c, lw=0.9, alpha=0.5)
        ax.plot(P[:, 0], P[:, 1], P[:, 2], '-', color=c, lw=1.5,
                label=f'D{d.id + 1} (corridor {d.zc:.0f} m)')
    ax.scatter(*DEPOT, color='black', marker='s', s=80, label='Depot (0, 0, 5)')
    for o in res['orders']:
        ok = o.oid in res['deliveries'] and res['deliveries'][o.oid] <= o.t_deadline
        ax.scatter(*o.loc, marker='*', s=220, color='green' if ok else 'red',
                   edgecolors='k', depthshade=False)
        did = res['assign'][o.oid][0] if o.oid in res['assign'] else None
        ax.text(o.loc[0], o.loc[1], o.loc[2] + 0.4,
                f"O{o.oid + 1} z={o.loc[2]:.1f}" + (f"->D{did + 1}" if did is not None else ''),
                fontsize=7)
    h, _ = ax.get_legend_handles_labels()
    h += [mlines.Line2D([], [], ls='--', color='grey', label='Planned route'),
          mlines.Line2D([], [], marker='*', ls='', color='green', mec='k', ms=12, label='Order on time'),
          mlines.Line2D([], [], marker='*', ls='', color='red', mec='k', ms=12, label='Order late')]
    ax.legend(handles=h, fontsize=8, loc='upper left')
    ax.set_xlim(ARENA_LO, ARENA_HI); ax.set_ylim(ARENA_LO, ARENA_HI); ax.set_zlim(0, 11)
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)'); ax.set_zlabel('z (m)', labelpad=6)
    ax.view_init(elev=22, azim=-60)
    ax.set_box_aspect(None, zoom=0.88)
    ax.set_title('S033 3D - 3D cheapest insertion, seed 0\n'
                 '(dashed = planned, solid = flown; z axis stretched: corridors 3/5/7/9 m)')
    path = os.path.join(out_dir, 'trajectory_3d.png')
    plt.savefig(path, dpi=130, bbox_inches='tight'); plt.close()
    print(f'Saved: {path}')


def plot_altitude_profiles(res, out_dir):
    t = res['times']
    fig, axes = plt.subplots(2, 1, figsize=(13, 8), sharey=True)
    for ax, xmax in zip(axes, [t[-1], 300.0]):
        for d in res['drones']:
            P = np.array(d.path)
            ax.plot(t, P[:, 2], color=DRONE_COLORS[d.id], lw=1.3, label=f'D{d.id + 1}')
            ax.axhspan(d.zc - DZ_TOL, d.zc + DZ_TOL, color=DRONE_COLORS[d.id], alpha=0.06)
        for e in res['events']:
            o = res['orders'][e['oid']]
            td = res['deliveries'].get(e['oid'])
            if td is not None:
                ok = td <= o.t_deadline
                ax.plot(td, o.loc[2], '*', ms=12, color='green' if ok else 'red', mec='k', zorder=5)
                if td <= xmax:
                    ax.annotate(f"O{e['oid'] + 1}", (td, o.loc[2]), xytext=(4, 4),
                                textcoords='offset points', fontsize=7)
        ax.set_xlim(0, xmax); ax.grid(alpha=0.3)
        ax.set_ylabel('z (m)')
    axes[1].set_xlabel('Time (s)')
    axes[0].set_title('Whole mission (shaded bands = corridor z_k +/- 1.5 m tolerance; '
                      'stars = order delivered, green on time / red late)')
    axes[1].set_title('Zoom on the first 300 s')
    axes[0].legend(fontsize=8, ncol=4, loc='upper right')
    fig.suptitle('S033 3D Altitude profiles - 3D cheapest insertion, seed 0', fontsize=12)
    fig.tight_layout()
    path = os.path.join(out_dir, 'altitude_profiles.png')
    plt.savefig(path, dpi=130); plt.close()
    print(f'Saved: {path}')


def plot_detour_comparison(seed0, mc, out_dir):
    x = np.arange(len(STRATEGIES))
    labels = [STRAT_LABEL[s] for s in STRATEGIES]
    panels = [('added_flown', 'Added 3D flight distance (m)', '{:.0f}'),
              ('on_time', f'Orders on time (of {N_INSERTIONS})', '{:.2f}')]
    fig, axes = plt.subplots(1, 3, figsize=(18, 5.5))
    w = 0.38
    ax = axes[2]
    tot = [sum(r['los_events'] for r in mc[s]) for s in STRATEGIES]
    nrun = [sum(1 for r in mc[s] if r['los_events'] > 0) for s in STRATEGIES]
    bars = ax.bar(x, tot, 0.6, color=STRAT_COLORS)
    ax.bar_label(bars, labels=[f'{t} ({n} runs)' for t, n in zip(tot, nrun)], fontsize=7, padding=2)
    ax.set_ylim(0, max(tot) * 1.25 + 1)
    ax.set_xticks(x); ax.set_xticklabels(labels, fontsize=7.5); ax.grid(axis='y', alpha=0.3)
    s0 = [seed0[s]['los_events'] for s in STRATEGIES]
    s0txt = (f'{s0[0]} for every strategy' if len(set(s0)) == 1
             else ', '.join(str(v) for v in s0))
    ax.set_title(f'En-route losses of separation, total over {N_SEEDS} seeds\n(seed 0: {s0txt})',
                 fontsize=10)
    for ax, (key, title, fmt) in zip(axes[:2], panels):
        v0 = [seed0[s][key] for s in STRATEGIES]
        vm = [np.mean([r[key] for r in mc[s]]) for s in STRATEGIES]
        sd = [np.std([r[key] for r in mc[s]]) for s in STRATEGIES]
        b1 = ax.bar(x - w / 2, v0, w, color=STRAT_COLORS, alpha=0.5, label='seed 0')
        b2 = ax.bar(x + w / 2, vm, w, yerr=sd, capsize=3, color=STRAT_COLORS,
                    label=f'mean +/- std, {N_SEEDS} seeds')
        ax.bar_label(b1, labels=[fmt.format(v) for v in v0], fontsize=7, padding=2)
        ax.bar_label(b2, labels=[fmt.format(v) for v in vm], fontsize=7, padding=2)
        ax.set_xticks(x); ax.set_xticklabels(labels, fontsize=7.5)
        ax.set_title(title, fontsize=10); ax.grid(axis='y', alpha=0.3)
    axes[0].legend(fontsize=8)
    fig.suptitle('S033 3D Strategy comparison (light = seed 0, dark = Monte-Carlo mean +/- std)',
                 fontsize=12)
    fig.tight_layout()
    path = os.path.join(out_dir, 'detour_comparison.png')
    plt.savefig(path, dpi=130); plt.close()
    print(f'Saved: {path}')


def plot_vertical_breakdown(res, mc, out_dir):
    ev = sorted(res['events'], key=lambda e: e['oid'])
    fig, axes = plt.subplots(1, 2, figsize=(16, 5.5), gridspec_kw={'width_ratios': [3, 2]})
    ax = axes[0]
    xs = np.arange(len(ev))
    hxy = np.array([e['dxy'] for e in ev])
    v3 = np.array([e['d3'] - e['dxy'] for e in ev])
    rt = np.array([e['dtrue'] - e['d3'] for e in ev])
    ax.bar(xs, hxy, color='#95a5a6', label='horizontal detour (x-y)')
    ax.bar(xs, v3, bottom=hxy, color='#2980b9', label='3D excess of the q insertion')
    ax.bar(xs, rt, bottom=hxy + v3, color='#e74c3c', label='corridor-return leg')
    for i, e in enumerate(ev):
        ax.text(i, e['dtrue'] + 8, f"{e['dtrue']:.1f} m\nvert {e['dtrue'] - e['dxy']:.1f}",
                ha='center', fontsize=7)
    ax.set_xticks(xs)
    ax.set_xticklabels([f"O{e['oid'] + 1}->D{e['drone'] + 1}\n{e['mode']}" for e in ev], fontsize=8)
    ax.set_ylabel('Added distance (m)'); ax.grid(axis='y', alpha=0.3)
    ax.set_ylim(0, max(e['dtrue'] for e in ev) * 1.15)
    ax.legend(fontsize=8, loc='upper center')
    ax.set_title('Per insertion, 3D cheapest, seed 0 (labels: total, and vertical part = total - x-y detour)')
    ax = axes[1]
    for j, s in enumerate(STRATEGIES):
        share = np.array([(r['added_flown'] - r['detour_xy']) / r['added_flown'] * 100
                          for r in mc[s] if r['added_flown'] > 0])
        ax.boxplot(share, positions=[j], widths=0.6, patch_artist=True,
                   boxprops=dict(facecolor=STRAT_COLORS[j], alpha=0.6),
                   medianprops=dict(color='k'))
    ax.set_xticks(range(len(STRATEGIES)))
    ax.set_xticklabels([STRAT_LABEL[s] for s in STRATEGIES], fontsize=7.5)
    ax.set_ylabel('Vertical share of added distance (%)'); ax.grid(axis='y', alpha=0.3)
    ax.set_title(f'(added 3D - x-y detour) / added 3D, {N_SEEDS} seeds')
    fig.suptitle('S033 3D Vertical detour breakdown', fontsize=12)
    fig.tight_layout()
    path = os.path.join(out_dir, 'vertical_breakdown.png')
    plt.savefig(path, dpi=130); plt.close()
    print(f'Saved: {path}')


def plot_load_balance(mc, out_dir):
    fig, axes = plt.subplots(1, len(STRATEGIES), figsize=(18, 4), sharey=True)
    for ax, s in zip(axes, STRATEGIES):
        counts = np.sum([r['per_drone'] for r in mc[s]], axis=0)
        bars = ax.bar(range(K), counts, color=DRONE_COLORS)
        ax.bar_label(bars, fontsize=8)
        ax.set_xticks(range(K))
        ax.set_xticklabels([f'D{i + 1}\n{CORRIDORS[i]:.0f} m' for i in range(K)], fontsize=8)
        ax.set_title(STRAT_LABEL[s].replace('\n', ' '), fontsize=9); ax.grid(axis='y', alpha=0.3)
    axes[0].set_ylabel(f'Orders assigned (sum over {N_SEEDS} seeds)')
    fig.suptitle('S033 3D Load balance per drone / corridor', fontsize=12)
    fig.tight_layout()
    path = os.path.join(out_dir, 'load_balance.png')
    plt.savefig(path, dpi=130); plt.close()
    print(f'Saved: {path}')


def save_animation(res, out_dir, max_frames=150):
    t = res['times']
    n = len(t)
    step = math.ceil((n - 1) / (max_frames - 1))
    n_frames = math.ceil((n - 1) / step) + 1
    idx_of = lambda f: min(f * step, n - 1)
    paths = [np.array(d.path) for d in res['drones']]
    fig = plt.figure(figsize=(9, 7.5))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim(ARENA_LO, ARENA_HI); ax.set_ylim(ARENA_LO, ARENA_HI); ax.set_zlim(0, 11)
    ax.set_xlabel('x (m)', fontsize=8); ax.set_ylabel('y (m)', fontsize=8); ax.set_zlabel('z (m)', fontsize=8)
    ax.view_init(elev=22, azim=-60)
    ax.scatter(*DEPOT, color='black', marker='s', s=50)
    for d in res['drones']:
        ir = np.array(d.initial_route)
        ax.plot(ir[:, 0], ir[:, 1], ir[:, 2], '--', color=DRONE_COLORS[d.id], lw=0.5, alpha=0.35)
    marks = [ax.plot([], [], [], '*', ms=12, color='purple', mec='k')[0] for _ in res['orders']]
    dots = [ax.plot([], [], [], 'o', color=DRONE_COLORS[k], ms=7)[0] for k in range(K)]
    trails = [ax.plot([], [], [], '-', color=DRONE_COLORS[k], lw=1.2)[0] for k in range(K)]
    title = ax.set_title('')

    def update(f):
        i = idx_of(f)
        for k in range(K):
            p = paths[k][:i + 1]
            trails[k].set_data(p[:, 0], p[:, 1]); trails[k].set_3d_properties(p[:, 2])
            dots[k].set_data([p[-1, 0]], [p[-1, 1]]); dots[k].set_3d_properties([p[-1, 2]])
        for o, m in zip(res['orders'], marks):
            if o.t_arrive <= t[i]:
                done = o.oid in res['deliveries'] and res['deliveries'][o.oid] <= t[i]
                m.set_data([o.loc[0]], [o.loc[1]]); m.set_3d_properties([o.loc[2]])
                m.set_color('lightgrey' if done else 'purple')
        title.set_text(f'S033 3D - 3D cheapest insertion, seed 0, t = {t[i]:.0f} s')
        return dots + trails + marks + [title]

    ani = FuncAnimation(fig, update, frames=n_frames, interval=60, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer=PillowWriter(fps=15), dpi=75)
    plt.close()
    print(f'Saved: {path}  ({n_frames} frames)')


# ── Main ──────────────────────────────────────────────────────────

def _summ(rs, key):
    v = np.array([r[key] for r in rs], float)
    return v.mean(), v.std(), v.min(), v.max()


if __name__ == '__main__':
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    assert planned_feasible(SEED0)
    seed0 = {s: run_simulation(s, SEED0, record=(s == 'cheapest_3d')) for s in STRATEGIES}
    fleet, orders = make_instance(SEED0)
    print('Seed-0 instance:')
    for k, (wps, bat) in enumerate(fleet):
        print(f'  D{k + 1}: corridor {CORRIDORS[k]:.0f} m, planned route '
              f'{route_length(planned_route(k, wps)):7.1f} m, battery {bat:7.1f} m')
    for o in orders:
        print(f'  O{o.oid + 1}: q=({o.loc[0]:7.1f}, {o.loc[1]:7.1f}, {o.loc[2]:5.2f})  '
              f't_arrive={o.t_arrive:5.1f} s  deadline={o.t_deadline:6.1f} s')

    print('\n=== Seed 0 ===')
    for s in STRATEGIES:
        r = seed0[s]
        print(f"  {s:18s} added_3d={r['added_flown']:7.1f} m  xy_detour={r['detour_xy']:7.1f} m  "
              f"added_climb={r['added_climb']:5.1f} m  online={r['n_online']} queued={r['n_queued']} "
              f"rej={r['n_rejected']}  on_time={r['on_time']}/{N_INSERTIONS}  "
              f"LoS={r['los_events']} ({r['los_s']:.1f} s)  term_LoS={r['term_events']}  "
              f"conflict_slots={r['slot_conflicts']}  makespan={r['makespan']:6.1f} s  "
              f"min_bat={r['min_battery']:6.1f} m  per_drone={r['per_drone']}")
    print('\n  Seed-0 insertions (3D cheapest):')
    for e in sorted(seed0['cheapest_3d']['events'], key=lambda e: e['oid']):
        print(f"    O{e['oid'] + 1} -> D{e['drone'] + 1} {e['mode']:6s} xy={e['dxy']:7.2f}  "
              f"3d={e['d3']:7.2f}  card_dz={e['dz']:5.2f}  phi={e['phi']:4.2f}  "
              f"true={e['dtrue']:7.2f}  ret={e['ret']}")

    seeds, skipped = valid_seeds(N_SEEDS)
    print(f'\nMonte-Carlo seeds: first {N_SEEDS} valid seeds = {seeds[0]}..{seeds[-1]}; '
          f'skipped {len(skipped)} seeds whose planned route exceeds a drone battery: {skipped}')
    mc = {s: [run_simulation(s, sd) for sd in seeds] for s in STRATEGIES}
    print(f'\n=== Monte Carlo, {N_SEEDS} seeds (mean +/- std [min, max]) ===')
    for s in STRATEGIES:
        rs = mc[s]
        a = _summ(rs, 'added_flown'); c = _summ(rs, 'added_climb')
        print(f"  {s:18s} added_3d={a[0]:7.1f} +/- {a[1]:6.1f} [{a[2]:6.1f}, {a[3]:6.1f}] m  "
              f"added_climb={c[0]:5.1f} m  on_time={_summ(rs, 'on_time')[0]:.2f}  "
              f"late={_summ(rs, 'late')[0]:.2f}  queued={_summ(rs, 'n_queued')[0]:.2f}  "
              f"rej={_summ(rs, 'n_rejected')[0]:.2f}  "
              f"LoS={sum(r['los_events'] for r in rs)} events in "
              f"{sum(1 for r in rs if r['los_events'] > 0)} runs ({sum(r['los_s'] for r in rs):.1f} s)  "
              f"term_LoS={sum(r['term_events'] for r in rs)}  "
              f"conflict_slots={sum(r['slot_conflicts'] for r in rs)}  "
              f"drone_conflict_rej={sum(r['drone_conflict_rejections'] for r in rs)}  "
              f"ret_wps={_summ(rs, 'n_ret')[0]:.2f}  excursion={_summ(rs, 'excursion_s')[0]:.1f} s  "
              f"makespan={_summ(rs, 'makespan')[0]:6.1f} s  min_bat={min(r['min_battery'] for r in rs):.1f} m")
    vshare = np.array([(r['added_flown'] - r['detour_xy']) / r['added_flown'] * 100
                       for r in mc['cheapest_3d'] if r['added_flown'] > 0])
    print(f"  3D cheapest: vertical share of added distance median {np.median(vshare):.2f} %, "
          f"max {vshare.max():.2f} %")

    print('\n  Paired comparison of added 3D distance (3D cheapest vs X): wins / ties / losses')
    base = np.array([r['added_flown'] for r in mc['cheapest_3d']])
    for s in STRATEGIES[1:]:
        o = np.array([r['added_flown'] for r in mc[s]])
        print(f"    vs {s:18s} {int(np.sum(base < o - 1e-6)):3d} / {int(np.sum(np.abs(base - o) <= 1e-6)):3d} / "
              f"{int(np.sum(base > o + 1e-6)):3d}   mean diff {np.mean(o - base):+7.1f} m")
    n_ord = sum(len(r['choice']) for r in mc['cheapest_3d'])
    diff_drone = sum(1 for r3, rp in zip(mc['cheapest_3d'], mc['cheapest_planar'])
                     for oid in r3['choice'] if rp['choice'].get(oid) != r3['choice'][oid])
    diff_run = sum(1 for r3, rp in zip(mc['cheapest_3d'], mc['cheapest_planar'])
                   if abs(r3['added_flown'] - rp['added_flown']) > 1e-6)
    print(f"  3D vs planar cheapest: different drone for {diff_drone}/{n_ord} orders; "
          f"different outcome in {diff_run}/{N_SEEDS} runs")
    diff_chk = sum(1 for r3, rn in zip(mc['cheapest_3d'], mc['cheapest_3d_nochk'])
                   if abs(r3['added_flown'] - rn['added_flown']) > 1e-6)
    print(f"  conflict pre-screen changed the outcome in {diff_chk}/{N_SEEDS} runs")

    print('\n  En-route losses of separation (all strategies, all MC runs):')
    print('    a "drift" LoS has both drones within dz_tol of their corridor (no return waypoint);')
    print('    "in window" = inside the [t_now, t_arrive + 30 s] pre-screen of an online insertion on one of the pair')
    for s in STRATEGIES:
        for r in mc[s]:
            for (ts, dur, a, b, ea, eb) in r['los_log']:
                kind = 'drift' if max(ea, eb) <= DZ_TOL else 'excursion'
                inwin = any(e['t_end'] is not None and e['drone'] in (a, b) and e['t'] <= ts <= e['t_end']
                            for e in r['events'])
                print(f"    {s:18s} seed {r['seed']:3d}  D{a + 1}-D{b + 1}  t={ts:6.1f} s for {dur:3.1f} s  "
                      f"|z-z_k| = {ea:4.2f} / {eb:4.2f} m  {kind:9s} {'in window' if inwin else 'outside window'}")

    plot_trajectory_3d(seed0['cheapest_3d'], OUTPUT_DIR)
    plot_altitude_profiles(seed0['cheapest_3d'], OUTPUT_DIR)
    plot_detour_comparison(seed0, mc, OUTPUT_DIR)
    plot_vertical_breakdown(seed0['cheapest_3d'], mc, OUTPUT_DIR)
    plot_load_balance(mc, OUTPUT_DIR)
    save_animation(seed0['cheapest_3d'], OUTPUT_DIR)
    print('Done.')
