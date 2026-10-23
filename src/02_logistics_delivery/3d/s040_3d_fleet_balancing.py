"""
S040 3D Upgrade — Fleet Load Balancing
======================================
8 delivery drones work from one ground depot at (300, 300, 0) m with 2
charging pads over a 600 x 600 x 30 m urban volume. 40 orders arrive at
uniformly random times in [0, 320] s; the horizon is 400 s. The horizontal
order stream (x, y, arrival time) is drawn exactly as in the fixed 2D S040, so
the 2D and 3D runs see the same orders; each order also gets a delivery tier
z_o in {5, 15, 25} m (equal probability, separate seeded generator).

Drones fly straight 3D legs at 8 m/s (3D speed) through a private FIFO queue
and return to the depot (descending to z = 0) only to recharge.

3D energy per metre flown (card, with the card's missing alpha restored, see
README):  ds = k (d_3D + gamma max(dz, 0) - eta max(-dz, 0)),
k = alpha (m_body + m_p) g / E_cap.

Dispatch strategies (all flown and scored in the same 3D world; drone-level
safety rules always use the true 3D energy model)
  round_robin   : order j -> drone j mod 8.
  least_queue   : fewest unfinished orders, ties -> lowest index.
  li_blind      : S040 load-index balancer that plans in 2D: horizontal
                  marginal distance, horizontal battery-feasibility check, no
                  altitude terms (what the 2D dispatcher would do here).
  li_3d_nozone  : same balancer with 3D distances and the 3D energy model in
                  the feasibility check, but phi_z = 0 and no swap veto
                  (ablation: isolates the zone terms).
  li_3d         : card's proposed 3D load-index balancer: 3D distances, 3D
                  feasibility, phi_z altitude-mismatch term against the
                  drone's most recent delivery altitude, swap veto 15 m.
  zone_affinity : card's strategy 4: like li_3d, but every drone has a fixed
                  preferred tier (D0 5 m, D1 15 m, D2 25 m, D3 5 m, ...) that
                  replaces the most-recent-delivery altitude.

No order is ever dropped (per-step conservation check, as in 2D). Invariants
asserted every step: SoC never below 0, 0 <= z <= 30 m, x, y inside the
volume, pads hold at most 2 drones and only drones at the depot charge. At the
end of each run: SoC bookkeeping (start + charged - used = final) and, on the
recorded runs, no step longer than v dt.

Usage:
    conda activate drones
    python src/02_logistics_delivery/3d/s040_3d_fleet_balancing.py
"""

import importlib.util
import math
import os
from collections import deque
from dataclasses import dataclass, field
from typing import Optional

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.animation import FuncAnimation, PillowWriter
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

# ── Parameters (card unless marked) ───────────────────────────────────────────
N_DRONES      = 8
N_PADS        = 2
M_ORDERS      = 40
T_SIM         = 400.0        # s
T_ARRIVE_MAX  = 0.8 * T_SIM  # s, orders arrive uniformly in [0, 320] s (as 2D)
DT            = 0.2          # s
REBAL_PERIOD  = 20.0         # s
S_MAX_SWAPS   = 2
T_PROMISE     = 120.0        # s on-time promise (added in 2D, kept)

V_DRONE       = 8.0          # m/s, 3D speed
M_BODY        = 1.5          # kg
M_PAYLOAD     = 0.3          # kg
G             = 9.81
E_CAP         = 1.0
ALPHA         = 1.5e-5       # 1/m (card 1.5e-4 gives 377 m, not ~3560 m; fixed as in 2D)
GAMMA_CLIMB   = 0.5          # climb penalty factor
ETA_REGEN     = 0.2          # descent regeneration factor
P_CHG         = 0.20         # E_cap/s
BETA_SAFE     = 0.05
K_DIS         = ALPHA * (M_BODY + M_PAYLOAD) * G / E_CAP    # SoC per metre
D_RANGE       = 1.0 / K_DIS                                 # 3775 m
ALPHA_STRESS  = 6.0e-5       # stress test as in 2D (D_range 944 m)
K_DIS_STRESS  = ALPHA_STRESS * (M_BODY + M_PAYLOAD) * G / E_CAP

PHI_W, PHI_F, PHI_S, PHI_Z = 0.4, 0.25, 0.2, 0.15
F_NORM        = 1800.0       # m
Z_MAX_TIER    = 25.0         # m
DZ_VETO       = 15.0         # m
TIERS         = np.array([5.0, 15.0, 25.0])
Z_CEIL        = 30.0         # m, top of the operational volume

DEPOT = np.array([300.0, 300.0, 0.0])
GRID  = (0.0, 600.0)
CORNERS_TOP = [np.array([x, y, TIERS[-1]]) for x in GRID for y in GRID]

SHOW_SEED   = 42
STUDY_SEEDS = list(range(20))
TIER_SEED_OFFSET = 1000      # tiers drawn from default_rng(seed + 1000)
PHI_Z_SWEEP = [0.0, 0.025, 0.05, 0.1, 0.15, 0.3]
GIF_STEP    = 20             # simulation steps per GIF frame

STRATEGIES = ['round_robin', 'least_queue', 'li_blind', 'li_3d_nozone', 'li_3d',
              'zone_affinity']
LABELS = {'round_robin': 'Round-Robin', 'least_queue': 'Least-Queue',
          'li_blind': 'Load-Index (2D-planned)', 'li_3d_nozone': 'Load-Index 3D, phi_z=0',
          'li_3d': '3D Load-Index (proposed)', 'zone_affinity': 'Zone-Affinity'}
SHORT = {'round_robin': 'RR', 'least_queue': 'LQ', 'li_blind': 'LI-2D',
         'li_3d_nozone': 'LI-3D\nphi_z=0', 'li_3d': 'LI-3D\n(proposed)',
         'zone_affinity': 'Zone-\nAffinity'}
COLORS = {'round_robin': '#e74c3c', 'least_queue': '#3498db', 'li_blind': '#95a5a6',
          'li_3d_nozone': '#f39c12', 'li_3d': '#2ecc71', 'zone_affinity': '#8e44ad'}
STATE_COLORS = {'flying': '#2ecc71', 'returning': '#e67e22', 'charging': '#3498db',
                'waiting': '#9b59b6', 'idle': '#ecf0f1'}
DRONE_COLORS = plt.cm.tab10(np.arange(N_DRONES))
TIER_COLORS = {5.0: '#1f77b4', 15.0: '#ff7f0e', 25.0: '#d62728'}

OUTPUT_DIR = os.path.normpath(os.path.join(
    os.path.dirname(__file__), '..', '..', '..',
    'outputs', '02_logistics_delivery', '3d', 's040_3d_fleet_balancing'))
SRC_2D = os.path.normpath(os.path.join(os.path.dirname(__file__), '..',
                                       's040_fleet_load_balancing.py'))


# ── Geometry and energy ───────────────────────────────────────────────────────
def d3(a, b):
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def d2(a, b):
    return math.hypot(b[0] - a[0], b[1] - a[1])


def plan_cost_3d(a, b, k):
    """Card's planning cost of a leg: cruise on the 3D length plus the climb
    penalty; regeneration is not credited (conservative, as the card's
    feasibility formula)."""
    return k * (d3(a, b) + GAMMA_CLIMB * max(b[2] - a[2], 0.0))


def plan_cost_2d(a, b, k):
    return k * d2(a, b)


def route_cost(points, cost):
    return sum(cost(points[j], points[j + 1]) for j in range(len(points) - 1))


# ── Data classes ──────────────────────────────────────────────────────────────
@dataclass
class Order:
    idx: int
    location: np.ndarray        # (x, y, z_o)
    t_arrive: float
    t_deliver: Optional[float] = None
    drone: Optional[int] = None


@dataclass
class Drone:
    idx: int
    pos: np.ndarray
    k: float
    soc: float = 1.0
    queue: deque = field(default_factory=deque)
    current: Optional[Order] = None
    state: str = 'idle'
    deliveries: int = 0
    flight_dist: float = 0.0     # cumulative 3D path length f_i^3D
    returns: int = 0
    z_last: Optional[float] = None   # altitude of most recent delivery
    e_cruise: float = 0.0
    e_climb: float = 0.0
    e_regen: float = 0.0
    e_charged: float = 0.0
    climb_m: float = 0.0
    zone_switches: int = 0

    def n_unfinished(self):
        return len(self.queue) + (1 if self.current is not None else 0)

    def load_index(self):
        return (PHI_W * len(self.queue) + PHI_F * self.flight_dist / F_NORM
                + PHI_S * (1.0 - self.soc))

    def last_point(self):
        if self.queue:
            return self.queue[-1].location
        if self.current is not None:
            return self.current.location
        return self.pos

    def route_points(self, extra=None):
        pts = [self.pos]
        if self.current is not None:
            pts.append(self.current.location)
        pts += [o.location for o in self.queue]
        if extra is not None:
            pts.append(extra.location)
        pts.append(DEPOT)
        return pts

    def available(self):
        return self.state in ('idle', 'flying')

    # true 3D model (drone-level safety rules)
    def cost(self, a, b):
        return plan_cost_3d(a, b, self.k)

    def s_min_3d(self):
        return (self.k * d3(self.pos, DEPOT)
                + GAMMA_CLIMB * self.k * max(DEPOT[2] - self.pos[2], 0.0) + BETA_SAFE)


# ── Dispatcher configurations ─────────────────────────────────────────────────
def make_config(strategy, phi_z=PHI_Z):
    if strategy in ('round_robin', 'least_queue'):
        return dict(kind=strategy)
    if strategy == 'li_blind':
        return dict(kind='li', geom='2d', phi_z=0.0, veto=False, fixed_zone=False)
    if strategy == 'li_3d_nozone':
        return dict(kind='li', geom='3d', phi_z=0.0, veto=False, fixed_zone=False)
    if strategy == 'li_3d':
        return dict(kind='li', geom='3d', phi_z=phi_z, veto=True, fixed_zone=False)
    if strategy == 'zone_affinity':
        return dict(kind='li', geom='3d', phi_z=phi_z, veto=True, fixed_zone=True)
    raise ValueError(strategy)


def zone_of(d, cfg):
    if cfg['fixed_zone']:
        return float(TIERS[d.idx % len(TIERS)])
    return d.z_last                     # None before the first delivery


def can_accept(d, order, cfg):
    if cfg['geom'] == '3d':
        cost = lambda a, b: plan_cost_3d(a, b, d.k)
    else:
        cost = lambda a, b: plan_cost_2d(a, b, d.k)
    return d.soc - route_cost(d.route_points(order), cost) >= BETA_SAFE


def marginal_load(d, order, cfg):
    dist = (d3 if cfg['geom'] == '3d' else d2)(d.last_point(), order.location)
    dl = PHI_W * 1.0 + PHI_F * dist / F_NORM
    zz = zone_of(d, cfg)
    if cfg['phi_z'] > 0 and zz is not None:
        dl += cfg['phi_z'] * abs(order.location[2] - zz) / Z_MAX_TIER
    return dl


def assign_load_index(drones, order, cfg):
    best_k, best = None, np.inf
    for d in drones:
        if not d.available() or not can_accept(d, order, cfg):
            continue
        score = d.load_index() + marginal_load(d, order, cfg)
        if score < best - 1e-12:
            best, best_k = score, d.idx
    return best_k


def fleet_imbalance(drones):
    loads = [d.load_index() for d in drones]
    mean = float(np.mean(loads))
    return 0.0 if mean == 0 else (max(loads) - min(loads)) / mean


def rebalance(drones, cfg):
    """2D S040 rule plus the card's altitude veto. Returns (accepted swaps,
    veto rejections, other rejections)."""
    swaps = vetoed = rejected = 0
    for _ in range(S_MAX_SWAPS):
        donors = [d for d in drones if d.queue]
        takers = [d for d in drones if d.available()]
        if not donors or not takers:
            break
        most = max(donors, key=lambda d: (d.load_index(), -d.idx))
        least = min(takers, key=lambda d: (d.load_index(), d.idx))
        if most.idx == least.idx:
            break
        cand = most.queue[-1]
        zz = zone_of(least, cfg)
        if cfg['veto'] and zz is not None and abs(cand.location[2] - zz) > DZ_VETO:
            vetoed += 1
            break
        before = fleet_imbalance(drones)
        most.queue.pop()
        ok = can_accept(least, cand, cfg)
        least.queue.append(cand)
        if not ok or fleet_imbalance(drones) >= before:
            least.queue.pop()
            most.queue.append(cand)
            rejected += 1
            break
        cand.drone = least.idx
        swaps += 1
    return swaps, vetoed, rejected


def jain_fairness(counts):
    counts = np.asarray(counts, dtype=float)
    if counts.sum() == 0:
        return 1.0
    return counts.sum() ** 2 / (len(counts) * (counts ** 2).sum())


def check_conservation(orders, stream, backlog, drones):
    places = ([id(o) for o in stream] + [id(o) for o in backlog]
              + [id(o) for d in drones for o in d.queue]
              + [id(d.current) for d in drones if d.current is not None]
              + [id(o) for o in orders if o.t_deliver is not None])
    assert len(places) == len(orders) and len(set(places)) == len(orders), \
        'order conservation violated'


def make_orders(seed):
    """Same draw order as 2D S040 for (x, y, t); tiers from a separate rng."""
    rng = np.random.default_rng(seed)
    xy_t = [(rng.uniform(*GRID, size=2), float(rng.uniform(0, T_ARRIVE_MAX)))
            for _ in range(M_ORDERS)]
    tiers = np.random.default_rng(seed + TIER_SEED_OFFSET).choice(TIERS, size=M_ORDERS)
    orders = [Order(idx=j, location=np.array([xy[0], xy[1], tiers[j]]), t_arrive=t)
              for j, (xy, t) in enumerate(xy_t)]
    orders.sort(key=lambda o: (o.t_arrive, o.idx))
    return orders


# ── Simulation ────────────────────────────────────────────────────────────────
def simulate(strategy, seed, record=False, k_dis=K_DIS, phi_z=PHI_Z, rebal=True):
    cfg = make_config(strategy, phi_z)
    orders = make_orders(seed)
    stream = deque(orders)
    drones = [Drone(idx=i, pos=DEPOT.copy(), k=k_dis) for i in range(N_DRONES)]
    pads = [None] * N_PADS
    wait_q = deque()
    backlog = deque()
    rr = 0
    next_rebal = REBAL_PERIOD
    log_t, log_imb, log_cum, log_loads, log_states = [], [], [], [], []
    rebal_log, pos_hist, soc_hist = [], [], []
    min_soc = 1.0
    aborts = 0
    is_li = cfg['kind'] == 'li'

    def give(k, order):
        order.drone = k
        drones[k].queue.append(order)

    def go_home(d):
        if is_li:
            if d.current is not None:
                backlog.appendleft(d.current)
            backlog.extend(d.queue)
            d.queue.clear()
        elif d.current is not None:
            d.queue.appendleft(d.current)
        d.current = None
        d.state = 'returning'
        d.returns += 1

    n_steps = int(round(T_SIM / DT))
    for step in range(n_steps + 1):
        t = step * DT
        while stream and stream[0].t_arrive <= t:
            o = stream.popleft()
            if cfg['kind'] == 'round_robin':
                give(rr % N_DRONES, o)
                rr += 1
            elif cfg['kind'] == 'least_queue':
                give(min(range(N_DRONES), key=lambda i: (drones[i].n_unfinished(), i)), o)
            else:
                backlog.append(o)
        if is_li:
            for _ in range(len(backlog)):
                o = backlog.popleft()
                k = assign_load_index(drones, o, cfg)
                if k is None:
                    backlog.append(o)
                else:
                    give(k, o)
            if rebal and t >= next_rebal - 1e-9:
                sw, ve, rj = rebalance(drones, cfg)
                rebal_log.append((t, sw, ve, rj))
                next_rebal += REBAL_PERIOD

        for d in drones:
            if d.state == 'idle' and not d.queue and d.soc < 1.0 - 1e-9:
                worst = max(d.cost(d.pos, c) + d.cost(c, DEPOT) for c in CORNERS_TOP)
                if d.soc - worst < BETA_SAFE:
                    go_home(d)
            if d.state == 'idle' and d.queue:
                nxt = d.queue[0]
                need = d.cost(d.pos, nxt.location) + d.cost(nxt.location, DEPOT)
                if d.soc - need < BETA_SAFE:
                    go_home(d)
                else:
                    d.current = d.queue.popleft()
                    d.state = 'flying'
            if d.state in ('flying', 'returning'):
                tgt = d.current.location if d.state == 'flying' else DEPOT
                vec = tgt - d.pos
                dist = d3(d.pos, tgt)
                step_len = min(V_DRONE * DT, dist)
                if dist > 1e-9:
                    mv = vec / dist * step_len
                    d.pos = d.pos + mv
                    dz = float(mv[2])
                else:
                    dz = 0.0
                ec = d.k * step_len
                el = d.k * GAMMA_CLIMB * max(dz, 0.0)
                er = d.k * ETA_REGEN * max(-dz, 0.0)
                d.soc -= ec + el - er
                assert d.soc >= 0.0, f'drone {d.idx} flew on an empty battery'
                d.e_cruise += ec
                d.e_climb += el
                d.e_regen += er
                d.climb_m += max(dz, 0.0)
                d.flight_dist += step_len
                if d.state == 'flying':
                    if dist <= V_DRONE * DT:
                        d.pos = d.current.location.copy()
                        d.current.t_deliver = t
                        d.deliveries += 1
                        zo = float(d.current.location[2])
                        if d.z_last is not None and zo != d.z_last:
                            d.zone_switches += 1
                        d.z_last = zo
                        d.current = None
                        d.state = 'idle'
                    elif d.soc <= d.s_min_3d():
                        aborts += 1          # card's in-flight return rule
                        go_home(d)
                elif dist <= V_DRONE * DT:
                    d.pos = DEPOT.copy()
                    free = next((p for p, v in enumerate(pads) if v is None), None)
                    if free is None:
                        wait_q.append(d.idx)
                        d.state = 'waiting'
                    else:
                        pads[free] = d.idx
                        d.state = 'charging'
        for p in range(N_PADS):
            if pads[p] is None:
                continue
            d = drones[pads[p]]
            assert d3(d.pos, DEPOT) < 1e-9, 'charging away from the depot'
            gain = min(1.0 - d.soc, P_CHG * DT)
            d.soc += gain
            d.e_charged += gain
            if d.soc >= 1.0 - 1e-12:
                d.state = 'idle'
                pads[p] = None
                if wait_q:
                    pads[p] = wait_q.popleft()
                    drones[pads[p]].state = 'charging'

        # invariants
        check_conservation(orders, stream, backlog, drones)
        assert sum(1 for d in drones if d.state == 'charging') <= N_PADS
        for d in drones:
            assert -1e-9 <= d.pos[2] <= Z_CEIL + 1e-9, 'altitude outside the volume'
            assert GRID[0] - 1e-9 <= d.pos[0] <= GRID[1] + 1e-9
            assert GRID[0] - 1e-9 <= d.pos[1] <= GRID[1] + 1e-9
            min_soc = min(min_soc, d.soc)
        log_t.append(t)
        log_imb.append(fleet_imbalance(drones))
        log_cum.append(sum(d.deliveries for d in drones))
        if record:
            log_loads.append([d.load_index() for d in drones])
            log_states.append([d.state for d in drones])
            pos_hist.append(np.array([d.pos.copy() for d in drones]))
            soc_hist.append([d.soc for d in drones])

    for d in drones:   # SoC bookkeeping: start + charged - used = final
        used = d.e_cruise + d.e_climb - d.e_regen
        assert abs(1.0 + d.e_charged - used - d.soc) < 1e-9, 'SoC bookkeeping'
    if record:
        sp = np.diff(np.array(pos_hist), axis=0)
        assert np.all(np.linalg.norm(sp, axis=2) <= V_DRONE * DT + 1e-9), 'teleport'

    on_time = [o for o in orders if o.t_deliver is not None
               and o.t_deliver - o.t_arrive <= T_PROMISE]
    late = [o for o in orders if o.t_deliver is not None
            and o.t_deliver - o.t_arrive > T_PROMISE]
    svc = [o.t_deliver - o.t_arrive for o in orders if o.t_deliver is not None]
    counts = [d.deliveries for d in drones]
    res = dict(strategy=strategy, seed=seed, orders=orders,
               delivered=len(on_time) + len(late), on_time=len(on_time), late=len(late),
               open=sum(1 for d in drones if d.current is not None)
               + sum(len(d.queue) for d in drones) + len(backlog),
               in_backlog=len(backlog),
               mean_service=float(np.mean(svc)) if svc else np.nan,
               p95_service=float(np.percentile(svc, 95)) if svc else np.nan,
               max_service=float(np.max(svc)) if svc else np.nan,
               service=np.array(svc), per_drone=counts, jain=jain_fairness(counts),
               mean_imbalance=float(np.mean(log_imb)),
               flight_km=sum(d.flight_dist for d in drones) / 1000.0,
               energy=sum(d.e_cruise + d.e_climb - d.e_regen for d in drones),
               e_cruise=[d.e_cruise for d in drones], e_climb=[d.e_climb for d in drones],
               e_regen=[d.e_regen for d in drones],
               climb_m=sum(d.climb_m for d in drones),
               zone_switches=sum(d.zone_switches for d in drones),
               returns=sum(d.returns for d in drones), aborts=aborts, min_soc=min_soc,
               rebal_log=rebal_log, swaps=sum(r[1] for r in rebal_log),
               vetoes=sum(r[2] for r in rebal_log), rejects=sum(r[3] for r in rebal_log),
               times=np.array(log_t), imbalance=np.array(log_imb),
               cumulative=np.array(log_cum))
    if record:
        res.update(loads=np.array(log_loads), states=log_states,
                   pos=np.array(pos_hist), soc=np.array(soc_hist))
    return res


def load_2d_module():
    spec = importlib.util.spec_from_file_location('s040_2d', SRC_2D)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def run_simulation():
    show = {s: simulate(s, SHOW_SEED, record=True) for s in STRATEGIES}
    study = {s: [simulate(s, sd) for sd in STUDY_SEEDS] for s in STRATEGIES}
    stress = {s: [simulate(s, sd, k_dis=K_DIS_STRESS) for sd in STUDY_SEEDS]
              for s in STRATEGIES}
    sweep = {pz: [simulate('li_3d', sd, phi_z=pz) for sd in STUDY_SEEDS]
             for pz in PHI_Z_SWEEP}
    noswap = [simulate('li_3d_nozone', sd, rebal=False) for sd in STUDY_SEEDS]
    m2d = load_2d_module()
    ref2d = {s: [m2d.simulate(s, sd) for sd in STUDY_SEEDS] for s in m2d.STRATEGIES}
    return show, study, stress, sweep, noswap, ref2d


# ── Plots ─────────────────────────────────────────────────────────────────────
def _save(fig, out_dir, name, **kw):
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, name)
    fig.savefig(path, dpi=130, **kw)
    plt.close(fig)
    print(f'Saved: {path}')


def _mean(runs, key):
    return float(np.mean([r[key] for r in runs]))


def _draw_scene(ax, r):
    locs = np.array([o.location for o in r['orders']])
    for z, c in TIER_COLORS.items():
        m = locs[:, 2] == z
        ax.scatter(locs[m, 0], locs[m, 1], locs[m, 2], color=c, s=28, depthshade=False,
                   edgecolors='black', linewidths=0.3, label=f'order tier {z:.0f} m')
    ax.scatter(*DEPOT, color='black', marker='s', s=80, label='depot (z = 0)')
    ax.set_xlim(0, 600); ax.set_ylim(0, 600); ax.set_zlim(0, Z_CEIL)
    ax.set_xlabel('x (m)', fontsize=8); ax.set_ylabel('y (m)', fontsize=8)
    ax.set_zlabel('z (m)', fontsize=8)
    ax.tick_params(labelsize=7)


def plot_trajectories_3d(show, out_dir):
    fig = plt.figure(figsize=(15, 6.5))
    for j, s in enumerate(['li_blind', 'li_3d']):
        r = show[s]
        ax = fig.add_subplot(1, 2, j + 1, projection='3d')
        _draw_scene(ax, r)
        for i in range(N_DRONES):
            p = r['pos'][:, i, :]
            ax.plot(p[:, 0], p[:, 1], p[:, 2], color=DRONE_COLORS[i], lw=0.9,
                    label=f'D{i}')
        ax.set_title(f'{LABELS[s]} (seed {SHOW_SEED})\nfleet 3D path {r["flight_km"]:.2f} km, '
                     f'climb {r["climb_m"]:.0f} m, zone switches {r["zone_switches"]}',
                     fontsize=10)
        ax.view_init(elev=28, azim=-60)
        if j == 1:
            ax.legend(fontsize=7, loc='upper left', bbox_to_anchor=(1.02, 1.0))
    fig.suptitle('S040 3D: drone paths in the 600 x 600 x 30 m volume', fontsize=12)
    plt.tight_layout()
    _save(fig, out_dir, 'trajectories_3d.png', bbox_inches='tight')


def plot_altitude_timeseries(show, out_dir):
    ss = ['li_blind', 'li_3d', 'zone_affinity']
    fig, axes = plt.subplots(len(ss), 1, figsize=(12, 9), sharex=True)
    for ax, s in zip(axes, ss):
        r = show[s]
        for i in range(N_DRONES):
            ax.plot(r['times'], r['pos'][:, i, 2] + 0.25 * (i - 3.5), color=DRONE_COLORS[i],
                    lw=0.9, label=f'D{i}')
        for z in TIERS:
            ax.axhline(z, color='gray', ls=':', lw=0.7)
        ax.set_ylabel('z (m)')
        ax.set_ylim(-2, 29)
        ax.set_title(f'{LABELS[s]}: zone switches {r["zone_switches"]}, '
                     f'total climb {r["climb_m"]:.0f} m', fontsize=10)
        ax.grid(alpha=0.25)
    axes[0].legend(ncol=8, fontsize=7, loc='upper right')
    axes[-1].set_xlabel('Time (s)')
    axes[-1].set_xlim(0, T_SIM)
    fig.suptitle(f'Altitude time series z_i(t) (seed {SHOW_SEED}; lines offset by '
                 '0.25 m per drone for legibility; dotted = tiers)', fontsize=11)
    plt.tight_layout()
    _save(fig, out_dir, 'altitude_timeseries.png')


def plot_imbalance_timeseries(show, out_dir):
    fig, ax = plt.subplots(figsize=(11, 5))
    for s in ['round_robin', 'least_queue', 'li_blind', 'li_3d']:
        r = show[s]
        ax.plot(r['times'], r['imbalance'], color=COLORS[s], lw=1.1,
                label=f'{LABELS[s]} (mean {r["mean_imbalance"]:.3f})')
    for (t_r, *_rest) in show['li_3d']['rebal_log']:
        ax.axvspan(t_r - 1.0, t_r + 1.0, color='#2ecc71', alpha=0.15, lw=0)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel(r'$\Lambda(t) = (L_{max}-L_{min})/\bar L$')
    ax.set_title(f'Fleet load imbalance (seed {SHOW_SEED}; green bands = rebalancing epochs)')
    ax.set_xlim(0, T_SIM)
    ax.set_ylim(bottom=0)
    ax.legend(fontsize=9)
    ax.grid(alpha=0.3)
    _save(fig, out_dir, 'imbalance_timeseries.png')


def plot_jain_fairness(study, ref2d, out_dir):
    fig, ax = plt.subplots(figsize=(12, 4.8))
    names, vals, errs, cols = [], [], [], []
    for s in ['round_robin', 'least_queue', 'load_index']:
        v = [r['jain'] for r in ref2d[s]]
        names.append({'round_robin': 'RR', 'least_queue': 'LQ',
                      'load_index': 'LI'}[s] + '\n(2D S040)')
        vals.append(np.mean(v)); errs.append(np.std(v)); cols.append('#dddddd')
    for s in STRATEGIES:
        v = [r['jain'] for r in study[s]]
        names.append(SHORT[s] + '\n(3D)')
        vals.append(np.mean(v)); errs.append(np.std(v)); cols.append(COLORS[s])
    x = np.arange(len(names))
    bars = ax.bar(x, vals, yerr=errs, capsize=4, color=cols, edgecolor='black')
    for b, v in zip(bars, vals):
        ax.text(b.get_x() + b.get_width() / 2, 1.05, f'{v:.3f}', ha='center', fontsize=8)
    ax.axvline(2.5, color='black', lw=0.8)
    ax.axhline(1.0, color='gray', ls='--', lw=0.8)
    ax.set_xticks(x)
    ax.set_xticklabels(names, fontsize=8)
    ax.set_ylim(0, 1.12)
    ax.set_ylabel(r"Jain's index of per-drone deliveries")
    ax.set_title(f"Jain's fairness, mean ± std over {len(STUDY_SEEDS)} order streams "
                 '(left: 2D S040 on the same x, y, t; right: 3D)')
    ax.grid(axis='y', alpha=0.3)
    _save(fig, out_dir, 'jain_fairness.png', bbox_inches='tight')


def plot_energy_breakdown(show, study, out_dir):
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    r = show['li_3d']
    x = np.arange(N_DRONES)
    ec, el, er = (np.array(r[k]) * 100 for k in ('e_cruise', 'e_climb', 'e_regen'))
    axes[0].bar(x, ec, color='#3498db', edgecolor='black', label='cruise (3D length)')
    axes[0].bar(x, el, bottom=ec, color='#e74c3c', edgecolor='black', label='climb penalty')
    axes[0].bar(x, -er, color='#2ecc71', edgecolor='black', label='descent regeneration')
    axes[0].set_xticks(x)
    axes[0].set_xticklabels([f'D{i}' for i in range(N_DRONES)])
    axes[0].set_ylabel('Energy (% of one battery)')
    axes[0].set_title(f'{LABELS["li_3d"]}, per drone (seed {SHOW_SEED})')
    axes[0].axhline(0, color='black', lw=0.8)
    axes[0].legend(fontsize=8)
    xs = np.arange(len(STRATEGIES))
    mc = [np.mean([sum(rr['e_cruise']) for rr in study[s]]) * 100 for s in STRATEGIES]
    ml = [np.mean([sum(rr['e_climb']) for rr in study[s]]) * 100 for s in STRATEGIES]
    mr = [np.mean([sum(rr['e_regen']) for rr in study[s]]) * 100 for s in STRATEGIES]
    axes[1].bar(xs, mc, color='#3498db', edgecolor='black', label='cruise')
    axes[1].bar(xs, ml, bottom=mc, color='#e74c3c', edgecolor='black', label='climb penalty')
    axes[1].bar(xs, -np.array(mr), color='#2ecc71', edgecolor='black', label='regeneration')
    for j in range(len(STRATEGIES)):
        axes[1].text(xs[j], mc[j] + ml[j] + 3, f'net {mc[j] + ml[j] - mr[j]:.0f}\n'
                     f'climb {ml[j]:.1f}\nregen {mr[j]:.1f}', ha='center', fontsize=7)
    axes[1].set_xticks(xs)
    axes[1].set_xticklabels([SHORT[s] for s in STRATEGIES], fontsize=8)
    axes[1].axhline(0, color='black', lw=0.8)
    axes[1].set_ylabel('Fleet energy (% of one battery)')
    axes[1].set_ylim(top=max(mc) + max(ml) + 45)
    axes[1].set_title(f'Fleet total, mean over {len(STUDY_SEEDS)} streams (nominal battery)')
    for ax in axes:
        ax.grid(axis='y', alpha=0.3)
    plt.tight_layout()
    _save(fig, out_dir, 'energy_breakdown.png')


def plot_cost_comparison(study, stress, out_dir):
    ss = STRATEGIES
    metrics = [('mean_service', 'Mean service time (s)'), ('energy', 'Fleet energy (batteries)'),
               ('delivered', 'Delivered by 400 s'), ('mean_imbalance', r'Mean $\Lambda$')]
    fig, axes = plt.subplots(2, 4, figsize=(17, 7.5))
    for row, (runs, name) in enumerate([(study, 'nominal battery'),
                                        (stress, 'stress test (D_range 944 m)')]):
        for col, (key, lab) in enumerate(metrics):
            ax = axes[row, col]
            data = [[r[key] for r in runs[s]] for s in ss]
            bp = ax.boxplot(data, showmeans=True, patch_artist=True, widths=0.6)
            for patch, s in zip(bp['boxes'], ss):
                patch.set_facecolor(COLORS[s]); patch.set_alpha(0.6)
            ax.set_xticks(range(1, len(ss) + 1))
            ax.set_xticklabels([SHORT[s] for s in ss], fontsize=7)
            ax.set_title(f'{lab}\n{name}', fontsize=9)
            ax.grid(axis='y', alpha=0.3)
    fig.suptitle(f'Altitude-blind vs 3D-aware dispatching, {len(STUDY_SEEDS)} order streams '
                 '(same orders for every strategy; triangle = mean)', fontsize=12)
    plt.tight_layout()
    _save(fig, out_dir, 'cost_3d_vs_2d.png')


def plot_phi_z_sweep(sweep, out_dir):
    fig, axes = plt.subplots(1, 4, figsize=(17, 4.3))
    pz = np.array(PHI_Z_SWEEP)
    for ax, (key, lab) in zip(axes, [('zone_switches', 'Zone switches per run'),
                                     ('climb_m', 'Fleet climb (m)'),
                                     ('energy', 'Fleet energy (batteries)'),
                                     ('mean_service', 'Mean service time (s)')]):
        m = np.array([_mean(sweep[p], key) for p in PHI_Z_SWEEP])
        sd = np.array([np.std([r[key] for r in sweep[p]]) for p in PHI_Z_SWEEP])
        ax.errorbar(pz, m, yerr=sd, fmt='o-', color='#2ecc71', capsize=4)
        ax.axvline(PHI_Z, color='gray', ls='--', lw=0.8)
        ax.set_xlabel(r'$\phi_z$')
        ax.set_title(lab, fontsize=10)
        ax.grid(alpha=0.3)
    fig.suptitle(f'3D Load-Index: effect of the altitude-mismatch weight phi_z (mean ± std, '
                 f'{len(STUDY_SEEDS)} streams; dashed = card value 0.15)', fontsize=11)
    plt.tight_layout()
    _save(fig, out_dir, 'phi_z_sweep.png')


def plot_veto_log(show, study, out_dir):
    fig, axes = plt.subplots(1, 2, figsize=(14, 4.5), gridspec_kw={'width_ratios': [2.2, 1]})
    r = show['li_3d']
    t_r = np.array([v[0] for v in r['rebal_log']])
    sw = np.array([v[1] for v in r['rebal_log']])
    ve = np.array([v[2] for v in r['rebal_log']])
    rj = np.array([v[3] for v in r['rebal_log']])
    axes[0].bar(t_r, sw, width=5, color='#2ecc71', label='swap accepted')
    axes[0].bar(t_r, ve, width=5, bottom=sw, color='#e74c3c', label='altitude veto')
    axes[0].bar(t_r, rj, width=5, bottom=sw + ve, color='#bbbbbb',
                label='rejected (imbalance / battery)')
    axes[0].set_xlabel('Rebalancing epoch time (s)')
    axes[0].set_ylabel('Count')
    axes[0].set_xlim(0, T_SIM)
    axes[0].set_ylim(0, S_MAX_SWAPS + 0.6)
    axes[0].set_title(f'{LABELS["li_3d"]}, seed {SHOW_SEED}: per-epoch swap outcome')
    axes[0].legend(fontsize=8)
    ss = ['li_blind', 'li_3d_nozone', 'li_3d', 'zone_affinity']
    x = np.arange(len(ss))
    a = [_mean(study[s], 'swaps') for s in ss]
    v = [_mean(study[s], 'vetoes') for s in ss]
    j = [_mean(study[s], 'rejects') for s in ss]
    axes[1].bar(x - 0.27, a, 0.27, color='#2ecc71', edgecolor='black', label='accepted')
    axes[1].bar(x, v, 0.27, color='#e74c3c', edgecolor='black', label='vetoed')
    axes[1].bar(x + 0.27, j, 0.27, color='#bbbbbb', edgecolor='black', label='rejected')
    axes[1].set_xticks(x)
    axes[1].set_xticklabels([SHORT[s] for s in ss], fontsize=8)
    axes[1].set_ylabel('Per run (mean)')
    axes[1].set_title(f'Mean over {len(STUDY_SEEDS)} streams')
    axes[1].legend(fontsize=8)
    for ax in axes:
        ax.grid(axis='y', alpha=0.3)
    plt.tight_layout()
    _save(fig, out_dir, 'veto_log.png')


def plot_load_heatmap(show, out_dir):
    ss = ['round_robin', 'li_3d']
    fig, axes = plt.subplots(1, 2, figsize=(13, 4), sharey=True)
    vmax = max(float(show[s]['loads'].max()) for s in ss)
    for ax, s in zip(axes, ss):
        im = ax.imshow(show[s]['loads'].T, aspect='auto', origin='lower',
                       extent=[0, T_SIM, -0.5, N_DRONES - 0.5], cmap='YlOrRd', vmin=0, vmax=vmax,
                       interpolation='nearest')
        ax.set_title(LABELS[s])
        ax.set_xlabel('Time (s)')
        ax.set_yticks(range(N_DRONES))
        ax.set_yticklabels([f'D{i}' for i in range(N_DRONES)], fontsize=7)
    axes[0].set_ylabel('Drone')
    fig.colorbar(im, ax=axes.tolist(), label='Load index $L_i(t)$', shrink=0.8)
    fig.suptitle(f'Load index heatmap (seed {SHOW_SEED})', fontsize=12)
    _save(fig, out_dir, 'load_heatmap.png', bbox_inches='tight')


def gif_frame_indices(n, step=GIF_STEP):
    n_frames = math.ceil((n - 1) / step) + 1
    return [min(f * step, n - 1) for f in range(n_frames)]


def save_animation(show, out_dir):
    r = show['li_3d']
    idx = gif_frame_indices(len(r['times']))
    fig = plt.figure(figsize=(9, 7))
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(*DEPOT, color='black', marker='s', s=80)
    ax.set_xlim(0, 600); ax.set_ylim(0, 600); ax.set_zlim(0, Z_CEIL)
    ax.set_xlabel('x (m)', fontsize=8); ax.set_ylabel('y (m)', fontsize=8)
    ax.set_zlabel('z (m)', fontsize=8)
    ax.view_init(elev=28, azim=-60)
    locs = np.array([o.location for o in r['orders']])
    t_arr = np.array([o.t_arrive for o in r['orders']])
    t_del = np.array([o.t_deliver if o.t_deliver is not None else np.inf for o in r['orders']])
    open_sc = ax.scatter([], [], [], color='#e74c3c', s=30, depthshade=False)
    done_sc = ax.scatter([], [], [], color='#aaaaaa', s=14, depthshade=False)
    trails = [ax.plot([], [], [], color=DRONE_COLORS[i], lw=0.9)[0] for i in range(N_DRONES)]
    dots = [ax.plot([], [], [], 'o', color=DRONE_COLORS[i], ms=6)[0] for i in range(N_DRONES)]
    title = ax.set_title('')

    def _set3(sc, pts):
        if len(pts):
            sc._offsets3d = (pts[:, 0], pts[:, 1], pts[:, 2])
        else:
            sc._offsets3d = ([], [], [])

    def update(f):
        i = idx[f]
        t = r['times'][i]
        _set3(open_sc, locs[(t_arr <= t) & (t_del > t)])
        _set3(done_sc, locs[t_del <= t])
        lo = max(0, i - 150)
        for k in range(N_DRONES):
            p = r['pos'][lo:i + 1, k, :]
            trails[k].set_data(p[:, 0], p[:, 1]); trails[k].set_3d_properties(p[:, 2])
            dots[k].set_data([p[-1, 0]], [p[-1, 1]]); dots[k].set_3d_properties([p[-1, 2]])
        title.set_text(f'S040 3D — {LABELS["li_3d"]}, t = {t:.0f} s, delivered '
                       f'{int(r["cumulative"][i])}/{M_ORDERS}\nred = open order, '
                       'grey = delivered, trail = last 30 s')
        return trails + dots + [open_sc, done_sc, title]

    ani = FuncAnimation(fig, update, frames=len(idx), blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer=PillowWriter(fps=10), dpi=72)
    plt.close(fig)
    print(f'Saved: {path} ({len(idx)} frames)')


# ── Reporting ─────────────────────────────────────────────────────────────────
def print_runs(title, runs):
    print(f'\n--- {title} ---')
    for s in STRATEGIES:
        R = runs[s]
        print(f'{LABELS[s]:<26} delivered {_mean(R, "delivered"):.2f} (on time '
              f'{_mean(R, "on_time"):.2f}, late {_mean(R, "late"):.2f}, open '
              f'{_mean(R, "open"):.2f}) | service mean {_mean(R, "mean_service"):.1f} s '
              f'(sd {np.std([r["mean_service"] for r in R]):.1f}), p95 '
              f'{_mean(R, "p95_service"):.1f} s | energy {_mean(R, "energy"):.3f} '
              f'(sd {np.std([r["energy"] for r in R]):.3f}) | flight '
              f'{_mean(R, "flight_km"):.2f} km | climb {_mean(R, "climb_m"):.0f} m | zone '
              f'switches {_mean(R, "zone_switches"):.1f} (climb energy '
              f'{np.mean([sum(r["e_climb"]) for r in R]):.3f}, regen '
              f'{np.mean([sum(r["e_regen"]) for r in R]):.3f}) | Jain {_mean(R, "jain"):.3f} | '
              f'mean Lambda {_mean(R, "mean_imbalance"):.3f} | returns '
              f'{_mean(R, "returns"):.2f} (in-flight aborts {_mean(R, "aborts"):.2f}) | swaps {_mean(R, "swaps"):.2f}, vetoes '
              f'{_mean(R, "vetoes"):.2f} | min SoC {min(r["min_soc"] for r in R):.3f}')
    for key, name in [('mean_service', 'mean service time'), ('energy', 'fleet energy')]:
        best = {s: 0 for s in STRATEGIES}
        for i in range(len(STUDY_SEEDS)):
            best[min(STRATEGIES, key=lambda s: (runs[s][i][key], STRATEGIES.index(s)))] += 1
        print(f'Lowest {name} per stream: ' + ', '.join(f'{SHORT[s].replace(chr(10), " ")} '
                                                         f'{best[s]}' for s in STRATEGIES))
    for a, b in [('li_3d_nozone', 'li_blind'), ('li_3d', 'li_3d_nozone'),
                 ('zone_affinity', 'li_3d')]:
        parts = []
        for key in ('mean_service', 'energy', 'delivered'):
            diff = np.array([runs[a][i][key] - runs[b][i][key] for i in range(len(STUDY_SEEDS))])
            lower = int(np.sum(diff < -1e-9)); higher = int(np.sum(diff > 1e-9))
            parts.append(f'{key} diff mean {diff.mean():+.3f} (lower on {lower}, higher on '
                         f'{higher}, equal on {len(diff) - lower - higher})')
        print(f'Paired {SHORT[a].replace(chr(10), " ")} - {SHORT[b].replace(chr(10), " ")}: '
              + '; '.join(parts))


if __name__ == '__main__':
    show, study, stress, sweep, noswap, ref2d = run_simulation()
    print('=== S040 3D Fleet Load Balancing ===')
    print(f'k = alpha (m_body+m_p) g = {K_DIS:.4e} SoC/m, D_range = {D_RANGE:.0f} m; '
          f'climb +{GAMMA_CLIMB} k per m up, regen -{ETA_REGEN} k per m down; stress k = '
          f'{K_DIS_STRESS:.4e} SoC/m (D_range {1 / K_DIS_STRESS:.0f} m)')
    tiers42 = [o.location[2] for o in show['li_3d']['orders']]
    print(f'Seed {SHOW_SEED} tiers: 5 m x{tiers42.count(5.0)}, 15 m x{tiers42.count(15.0)}, '
          f'25 m x{tiers42.count(25.0)}')
    print(f'\n--- Showcase order stream (seed {SHOW_SEED}) ---')
    for s in STRATEGIES:
        r = show[s]
        print(f'{LABELS[s]:<26} delivered {r["delivered"]:>2}/{M_ORDERS} (on time '
              f'{r["on_time"]}, late {r["late"]}, open {r["open"]}) | service mean '
              f'{r["mean_service"]:.1f} s, p95 {r["p95_service"]:.1f} s, max '
              f'{r["max_service"]:.1f} s | energy {r["energy"]:.3f} (cruise '
              f'{sum(r["e_cruise"]):.3f}, climb {sum(r["e_climb"]):.3f}, regen '
              f'{sum(r["e_regen"]):.3f}) | flight {r["flight_km"]:.2f} km | climb '
              f'{r["climb_m"]:.0f} m | zone switches {r["zone_switches"]} | Jain '
              f'{r["jain"]:.3f} | mean Lambda {r["mean_imbalance"]:.3f} | returns '
              f'{r["returns"]} (in-flight aborts {r["aborts"]}) | swaps {r["swaps"]}, vetoes {r["vetoes"]}, rejects '
              f'{r["rejects"]} | min SoC {r["min_soc"]:.3f}')
        print(f'{"":<26} per-drone deliveries {r["per_drone"]}')
    print_runs(f'Study: {len(STUDY_SEEDS)} order streams (seeds {STUDY_SEEDS[0]}..'
               f'{STUDY_SEEDS[-1]}), nominal battery', study)
    print_runs(f'Stress test (alpha = {ALPHA_STRESS:.1e}), same streams', stress)
    print('\n--- phi_z sweep (3D Load-Index with veto, nominal battery, 20 streams) ---')
    for pz in PHI_Z_SWEEP:
        R = sweep[pz]
        print(f'phi_z = {pz:<5} zone switches {_mean(R, "zone_switches"):.1f} | climb '
              f'{_mean(R, "climb_m"):.0f} m | energy {_mean(R, "energy"):.3f} | flight '
              f'{_mean(R, "flight_km"):.2f} km | service {_mean(R, "mean_service"):.1f} s | '
              f'delivered {_mean(R, "delivered"):.2f} | vetoes {_mean(R, "vetoes"):.2f}')
    R = noswap
    print(f'No rebalancing (phi_z = 0, no veto, no swaps): zone switches '
          f'{_mean(R, "zone_switches"):.1f} | climb {_mean(R, "climb_m"):.0f} m | energy '
          f'{_mean(R, "energy"):.3f} | flight {_mean(R, "flight_km"):.2f} km | service '
          f'{_mean(R, "mean_service"):.1f} s | delivered {_mean(R, "delivered"):.2f}')
    # paired per-stream comparison: no rebalancing vs every other variant
    print('Paired no rebalancing vs variant, per stream (better / equal / worse; '
          'service and energy lower = better, delivered higher = better):')
    others = [(SHORT[s].replace(chr(10), ' '), study[s]) for s in STRATEGIES] + \
        [(f'sweep phi_z={pz}', sweep[pz]) for pz in PHI_Z_SWEEP]
    for name, Rb in others:
        parts = []
        for key, sign in (('mean_service', 1), ('energy', 1), ('delivered', -1)):
            diff = np.array([sign * (noswap[i][key] - Rb[i][key])
                             for i in range(len(STUDY_SEEDS))])
            better = int(np.sum(diff < -1e-9)); worse = int(np.sum(diff > 1e-9))
            parts.append(f'{key} {better}/{len(diff) - better - worse}/{worse}')
        print(f'  vs {name:<16} ' + ' | '.join(parts))
    print('\n--- 2D S040 reference on the same x, y, t streams (flat world, 2D weights) ---')
    for s, R in ref2d.items():
        print(f'{s:<12} delivered {_mean(R, "delivered"):.2f} | service mean '
              f'{_mean(R, "mean_service"):.1f} s | Jain {_mean(R, "jain"):.3f} | flight '
              f'{_mean(R, "flight_km"):.2f} km')

    out_dir = OUTPUT_DIR
    os.makedirs(out_dir, exist_ok=True)
    print()
    plot_trajectories_3d(show, out_dir)
    plot_altitude_timeseries(show, out_dir)
    plot_imbalance_timeseries(show, out_dir)
    plot_jain_fairness(study, ref2d, out_dir)
    plot_energy_breakdown(show, study, out_dir)
    plot_cost_comparison(study, stress, out_dir)
    plot_phi_z_sweep(sweep, out_dir)
    plot_veto_log(show, study, out_dir)
    plot_load_heatmap(show, out_dir)
    save_animation(show, out_dir)
    print('Done.')
