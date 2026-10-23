"""
S030 3D Upgrade - Multi-Depot Delivery
======================================
4 depots on three altitude tiers (D1 0 m, D2 30 m rooftop, D3 15 m, D4 0 m),
10 drones (fleet [3, 2, 3, 2]), 20 ground customers at z = 1 m.

Altitude-zone routing (card): every horizontal leg of a drone from depot m is
flown at that fleet's cruise layer z_m^cruise; altitude changes happen
vertically at the nodes (descend to 1 m at each customer, climb back, final
climb/descent to the landing depot).  A leg a -> b of fleet m therefore has

    length = |z_c - z_a| + d_xy(a, b) + |z_b - z_c|
    climb  = max(z_c - z_a, 0) + max(z_b - z_c, 0)
    cost   = length + ALPHA_E * M_DRONE * G * climb    (m-equivalent)

Climbs are penalised, descents are free (card).  Ground-tier fleets D1 and D4
share tier 0 m and are put on separate layers 2.5 m / 7.5 m (card: +-2.5 m
offset, shifted up so that no layer is underground).

Constraints enforced for every plan (reused from the fixed 2D S030):
  * one sortie per drone, so depot m flies <= F_m sorties;
  * per sortie: payload <= Q, 3D path length (incl. vertical legs) <= R_max;
  * per depot: dispatched cargo <= inventory I_m.

Partition strategies (as in 2D S030, now on the 3D cost):
  1. Nearest-Depot     : each customer to the depot with the cheapest 3D
                         out-and-back cost
  2. Balanced K-Means  : depot-anchored K-Means on 3D positions with a
                         ceil(N/D) cluster cap
  3. AO                : single-customer moves on 3D cost + balance penalty

Every strategy is planned twice - with the flat 2D S030 cost ("flat plan")
and with the 3D cost ("3D plan") - and both plans are flown and scored in 3D.
That is the 2D-vs-3D comparison the card asks for.  A 200-layout Monte-Carlo
(customer seeds 0-199, same layout for every strategy and both planners) checks
whether the showcase result (card seed 7) is typical.  A time-stepped flight
(0.5 s) checks the card's claim that tiered layers give vertical separation
between fleets without collision logic, against a single shared layer.

Outputs (outputs/02_logistics_delivery/3d/s030_3d_multi_depot/):
  trajectory_3d.png, altitude_timeseries.png, strategy_comparison.png,
  flat_vs_3d.png, ao_convergence.png, per_drone_summary.png, animation.gif

Usage:
    conda activate drones
    python src/02_logistics_delivery/3d/s030_3d_multi_depot.py
"""

import matplotlib
matplotlib.use('Agg')

import os
import numpy as np
from scipy.spatial.distance import cdist
from scipy.optimize import linear_sum_assignment
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from matplotlib.lines import Line2D
from matplotlib.animation import FuncAnimation, PillowWriter

OUT_DIR = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', '..', '..',
    'outputs', '02_logistics_delivery', '3d', 's030_3d_multi_depot'))
os.makedirs(OUT_DIR, exist_ok=True)

# ─── Constants (card; 2D S030 values where the 3D card says "as in S030") ─────
N_DEPOTS    = 4
N_CUSTOMERS = 20
N_DRONES    = 10
V_DRONE     = 14.0     # m/s horizontal cruise speed
V_Z         = 3.0      # m/s vertical speed (added: card has instantaneous dz)
Q_MAX       = 3.0      # kg payload per sortie
R_MAX       = 3500.0   # m max 3D path length per sortie
SVC_TIME    = 8.0      # s service time per stop (from 2D S030)
LAMBDA      = 50.0     # balance penalty weight
EPS_AO      = 1.0      # AO convergence threshold
MAX_AO_ITER = 50
KM_ITER     = 100
SEED        = 7        # card customer seed (showcase)
MC_SEEDS    = list(range(200))  # Monte-Carlo customer layouts
M_DRONE     = 1.5      # kg
G           = 9.81     # m/s^2
ALPHA_E     = 0.05     # m/J energy-to-distance factor
DZ_SEP      = 5.0      # m minimum vertical separation between fleets
R_SEP_H     = 20.0     # m horizontal radius of a loss-of-separation (added)
CUST_Z      = 1.0      # m customer delivery altitude
Z_FLAT      = 15.0     # m single shared layer for the separation counterfactual
DT_SIM      = 0.5      # s flight sampling step

DEPOTS_XY  = np.array([[150., 150.], [150., 1050.], [1050., 150.],
                       [1050., 1050.]])
DEPOT_Z    = np.array([0.0, 30.0, 15.0, 0.0])
CRUISE_Z   = np.array([2.5, 30.0, 15.0, 7.5])   # D1/D4 share tier 0: 2.5/7.5
FLEET_SIZE = np.array([3, 2, 3, 2])
INVENTORY  = np.array([25., 18., 25., 20.])
assert FLEET_SIZE.sum() == N_DRONES
assert np.all(CRUISE_Z >= CUST_Z) and np.all(DEPOT_Z >= 0)
for _a in range(N_DEPOTS):
    for _b in range(_a + 1, N_DEPOTS):
        assert abs(CRUISE_Z[_a] - CRUISE_Z[_b]) >= DZ_SEP - 1e-9

DEPOT_COLORS = ["#e6194b", "#3cb44b", "#4363d8", "#f58231"]
DEPOT_LABELS = ["D1 SW", "D2 NW", "D3 SE", "D4 NE"]
STRATS = ["Nearest-Depot", "Balanced K-Means", "AO"]
MODES  = ["2d", "3d"]
MODE_NAME = {"2d": "flat plan (2D S030 cost)", "3d": "3D plan (3D cost)"}
MGA = ALPHA_E * M_DRONE * G          # m-equivalent per metre of climb


# ─── Instance and cost matrices ───────────────────────────────────────────────

class Instance:
    """One customer layout with 2D and per-fleet 3D leg matrices."""

    def __init__(self, seed):
        rng = np.random.default_rng(seed)
        self.seed = seed
        self.cust_xy = rng.uniform(100, 1100, (N_CUSTOMERS, 2))
        self.demand = rng.uniform(0.3, 1.8, N_CUSTOMERS)
        self.xy = np.vstack([DEPOTS_XY, self.cust_xy])
        self.z = np.concatenate([DEPOT_Z, np.full(N_CUSTOMERS, CUST_Z)])
        n = len(self.xy)
        dxy = cdist(self.xy, self.xy)
        self.D2 = dxy
        self.L3 = np.zeros((N_DEPOTS, n, n))
        self.CL = np.zeros((N_DEPOTS, n, n))
        for m in range(N_DEPOTS):
            zc = CRUISE_Z[m]
            za = self.z[:, None]
            zb = self.z[None, :]
            self.L3[m] = np.abs(zc - za) + dxy + np.abs(zb - zc)
            self.CL[m] = np.maximum(zc - za, 0) + np.maximum(zb - zc, 0)
            np.fill_diagonal(self.L3[m], 0.0)
            np.fill_diagonal(self.CL[m], 0.0)
        self.C3 = self.L3 + MGA * self.CL
        self.n_bar = N_CUSTOMERS / N_DEPOTS
        self.infeasible = {"2d": {"partitions": 0, "fleet": 0, "inventory": 0,
                                  "range": 0},
                           "3d": {"partitions": 0, "fleet": 0, "inventory": 0,
                                  "range": 0}}

    def cost_mat(self, mode, m):
        return self.D2 if mode == "2d" else self.C3[m]

    def len_mat(self, mode, m):
        return self.D2 if mode == "2d" else self.L3[m]

    def load_of(self, route):
        return float(sum(self.demand[c - N_DEPOTS] for c in route
                         if c >= N_DEPOTS))


def path_sum(M, route):
    return float(sum(M[route[i], route[i + 1]] for i in range(len(route) - 1)))


def two_opt(C, route):
    """2-opt on the customer segment (depot legs stay at the ends).

    The 3D leg matrix is asymmetric only on depot legs; customer-customer legs
    are symmetric, so the reversal gain formula is exact."""
    route = list(route)
    improved = True
    while improved:
        improved = False
        for i in range(1, len(route) - 2):
            for j in range(i + 1, len(route) - 1):
                gain = (C[route[i-1], route[i]] + C[route[j], route[j+1]]
                        - C[route[i-1], route[j]] - C[route[i], route[j+1]])
                if gain > 1e-6:
                    route[i:j+1] = route[i:j+1][::-1]
                    improved = True
    return route


def nn_order(C, depot, nodes):
    route, rem, cur = [depot], list(nodes), depot
    while rem:
        nxt = min(rem, key=lambda c: (C[cur, c], c))
        route.append(nxt); rem.remove(nxt); cur = nxt
    return two_opt(C, route + [depot])


def build_routes_for_depot(inst, mode, m, cust_nodes):
    """Pack depot m's customers into <= FLEET_SIZE[m] feasible sorties
    (nearest-neighbour, then first-fit-decreasing fallback; as in 2D S030)."""
    if not cust_nodes:
        return [], True, "-"
    C, L = inst.cost_mat(mode, m), inst.len_mat(mode, m)
    if inst.load_of(cust_nodes) > INVENTORY[m] + 1e-9:
        return [], False, "inventory"
    sorties, rem = [], list(cust_nodes)
    while rem:
        route, load, d, cur = [m], 0.0, 0.0, m
        while True:
            cand = [c for c in rem
                    if load + inst.demand[c - N_DEPOTS] <= Q_MAX + 1e-9
                    and d + L[cur, c] + L[c, m] <= R_MAX]
            if not cand:
                break
            c = min(cand, key=lambda c: (C[cur, c], c))
            route.append(c); rem.remove(c)
            load += inst.demand[c - N_DEPOTS]; d += L[cur, c]; cur = c
        if len(route) == 1:                 # nothing fits: unreachable
            return [], False, "range"
        sorties.append(two_opt(C, route + [m]))
    if len(sorties) <= FLEET_SIZE[m]:
        return sorties, True, "NN"
    bins = [[] for _ in range(FLEET_SIZE[m])]
    for c in sorted(cust_nodes, key=lambda c: (-inst.demand[c - N_DEPOTS], c)):
        for b in bins:
            if inst.load_of(b) + inst.demand[c - N_DEPOTS] <= Q_MAX + 1e-9:
                b.append(c)
                break
        else:
            return [], False, "fleet"
    sorties = [nn_order(C, m, b) for b in bins if b]
    if all(path_sum(L, s) <= R_MAX for s in sorties):
        return sorties, True, "FFD"
    return [], False, "range"


def all_routes(inst, mode, partition):
    routes, ok, how = [], True, []
    for m in range(N_DEPOTS):
        nodes = [c + N_DEPOTS for c in range(N_CUSTOMERS) if partition[c] == m]
        r, f, h = build_routes_for_depot(inst, mode, m, nodes)
        routes.append(r)
        ok &= f
        how.append(h)
    return routes, ok, how


def balance_penalty(inst, partition):
    n = np.bincount(partition, minlength=N_DEPOTS)
    return LAMBDA * float(np.sum((n - inst.n_bar) ** 2))


def total_cost(inst, mode, partition):
    """Home-return cost (mode's metric) + balance penalty; inf if infeasible."""
    routes, ok, how = all_routes(inst, mode, partition)
    if not ok:
        inst.infeasible[mode]["partitions"] += 1
        for h in how:
            if h in inst.infeasible[mode]:
                inst.infeasible[mode][h] += 1
        return np.inf
    return sum(path_sum(inst.cost_mat(mode, m), r)
               for m, rr in enumerate(routes) for r in rr) + \
        balance_penalty(inst, partition)


# ─── Partition strategies ─────────────────────────────────────────────────────

def nearest_depot_partition(inst, mode):
    if mode == "2d":
        return np.argmin(inst.D2[N_DEPOTS:, :N_DEPOTS], axis=1)
    c = np.arange(N_DEPOTS, N_DEPOTS + N_CUSTOMERS)
    ob = np.stack([inst.C3[m][m, c] + inst.C3[m][c, m]
                   for m in range(N_DEPOTS)], axis=1)
    return np.argmin(ob, axis=1)


def balanced_kmeans_partition(inst, mode):
    """Depot-anchored K-Means (2D or 3D positions) with ceil(N/D) cap."""
    cap = int(np.ceil(inst.n_bar))
    if mode == "2d":
        pts = inst.cust_xy
        anchors = DEPOTS_XY.astype(float)
    else:
        pts = np.column_stack([inst.cust_xy, np.full(N_CUSTOMERS, CUST_Z)])
        anchors = np.column_stack([DEPOTS_XY, DEPOT_Z]).astype(float)
    centroids = anchors.copy()
    part = None
    for it in range(KM_ITER):
        d = cdist(pts, centroids)
        srt = np.sort(d, axis=1)
        regret = srt[:, 1] - srt[:, 0]
        new = -np.ones(N_CUSTOMERS, dtype=int)
        count = np.zeros(N_DEPOTS, dtype=int)
        for c in sorted(range(N_CUSTOMERS), key=lambda c: (-regret[c], c)):
            for m in np.argsort(d[c], kind="stable"):
                if count[m] < cap:
                    new[c] = m; count[m] += 1
                    break
        if part is not None and np.array_equal(new, part):
            break
        part = new
        for m in range(N_DEPOTS):
            members = pts[part == m]
            if len(members):
                centroids[m] = 0.5 * anchors[m] + 0.5 * members.mean(axis=0)
    return part, it + 1


def alternating_optimisation(inst, mode):
    partition = nearest_depot_partition(inst, mode).copy()
    history = []
    prev = np.inf
    for _ in range(MAX_AO_ITER):
        c_val = total_cost(inst, mode, partition)
        history.append(c_val)
        if abs(prev - c_val) < EPS_AO:
            break
        prev = c_val
        for c in range(N_CUSTOMERS):
            m_cur, best_m, best_c = partition[c], partition[c], c_val
            for m2 in range(N_DEPOTS):
                if m2 == m_cur:
                    continue
                partition[c] = m2
                c2 = total_cost(inst, mode, partition)
                if c2 < best_c:
                    best_c, best_m = c2, m2
            partition[c] = best_m
            c_val = best_c
    return partition, history


def open_route_return(inst, mode, routes_by_depot):
    """Assign every sortie's landing depot (docking slots = fleet size) to
    maximise the total saving in the mode's cost, subject to the mode's
    range check.  Landing at home is always allowed."""
    sorties = [(m, r) for m, rr in enumerate(routes_by_depot) for r in rr]
    slots = [m for m in range(N_DEPOTS) for _ in range(FLEET_SIZE[m])]
    big = 1e9
    cost = np.full((len(sorties), len(slots)), big)
    for i, (m, r) in enumerate(sorties):
        C, L = inst.cost_mat(mode, m), inst.len_mat(mode, m)
        last, home_len = r[-2], path_sum(L, r)
        for j, m2 in enumerate(slots):
            if home_len - L[last, m] + L[last, m2] <= R_MAX + 1e-9:
                cost[i, j] = -(C[last, m] - C[last, m2])
    rows, cols = linear_sum_assignment(cost)
    assert np.all(cost[rows, cols] < big)
    out = [[] for _ in range(N_DEPOTS)]
    for i, j in zip(rows, cols):
        m, r = sorties[i]
        out[m].append(r[:-1] + [slots[j]])
    return out


# ─── Planning and 3D evaluation ───────────────────────────────────────────────

def plan(inst, mode, strat):
    info = {}
    if strat == "Nearest-Depot":
        part = nearest_depot_partition(inst, mode)
    elif strat == "Balanced K-Means":
        part, info["km_iter"] = balanced_kmeans_partition(inst, mode)
    else:
        part, info["history"] = alternating_optimisation(inst, mode)
    routes_home, ok, how = all_routes(inst, mode, part)
    if not ok:
        return dict(part=part, feasible=False, how=how, **info)
    routes = open_route_return(inst, mode, routes_home)
    return dict(part=part, feasible=True, how=how, routes_home=routes_home,
                routes=routes, **info)


def evaluate_3d(inst, p):
    """Score a plan (planned in either mode) as flown in 3D tiers."""
    if not p["feasible"]:
        return None
    n = np.bincount(p["part"], minlength=N_DEPOTS)
    pen = balance_penalty(inst, p["part"])
    home_cost = sum(path_sum(inst.C3[m], r)
                    for m, rr in enumerate(p["routes_home"]) for r in rr)
    drones = []
    for m, rr in enumerate(p["routes"]):
        for k, r in enumerate(rr):
            length = path_sum(inst.L3[m], r)
            climb = path_sum(inst.CL[m], r)
            hlen = path_sum(inst.D2, r)
            home = r[:-1] + [m]
            assert length <= R_MAX + 1e-6, "3D range exceeded"
            assert inst.load_of(r) <= Q_MAX + 1e-9, "payload exceeded"
            drones.append(dict(
                name=f"D{m+1}-#{k+1}", origin=m, ret=r[-1], route=r,
                stops=len(r) - 2, length=length, hlen=hlen,
                vlen=length - hlen, climb=climb,
                e_climb=M_DRONE * G * climb,
                cost=path_sum(inst.C3[m], r),
                saving=path_sum(inst.C3[m], home) - path_sum(inst.C3[m], r),
                load=inst.load_of(r)))
    sorties = [len(rr) for rr in p["routes"]]
    cargo = [sum(inst.load_of(r) for r in rr) for rr in p["routes"]]
    assert all(s <= f for s, f in zip(sorties, FLEET_SIZE)), "fleet exceeded"
    assert all(c <= i + 1e-9 for c, i in zip(cargo, INVENTORY)), "inventory"
    served = sorted(c for d in drones for c in d["route"][1:-1])
    assert served == list(range(N_DEPOTS, N_DEPOTS + N_CUSTOMERS))
    landed = np.bincount([d["ret"] for d in drones], minlength=N_DEPOTS)
    assert np.all(landed <= FLEET_SIZE), "docking slots exceeded"
    cost = sum(d["cost"] for d in drones)
    return dict(n=n, penalty=pen, home_cost=home_cost, cost=cost,
                objective=home_cost + pen, saving=home_cost - cost,
                length=sum(d["length"] for d in drones),
                hlen=sum(d["hlen"] for d in drones),
                vlen=sum(d["vlen"] for d in drones),
                e_climb=sum(d["e_climb"] for d in drones),
                drones=drones, sorties=sorties, cargo=cargo,
                max_load=max(d["load"] for d in drones),
                max_len=max(d["length"] for d in drones))


def eval_2d(inst, p):
    """Flat S030 metric of a 2D-planned plan (regression check vs 2D README)."""
    dist = sum(path_sum(inst.D2, r) for rr in p["routes"] for r in rr)
    return dist


# ─── Time-stepped flight (altitude profiles, separation, animation) ──────────

def drone_events(inst, route, m, zc):
    """Segments (t0, t1, p0, p1, kind) of one sortie at cruise layer zc."""
    ev, t = [], 0.0

    def add(p0, p1, speed, kind):
        nonlocal t
        dur = float(np.linalg.norm(p1 - p0)) / speed
        if dur > 0:
            ev.append((t, t + dur, p0, p1, kind))
            t += dur

    for a, b in zip(route[:-1], route[1:]):
        pa = np.array([*inst.xy[a], inst.z[a]])
        pb = np.array([*inst.xy[b], inst.z[b]])
        up = np.array([*inst.xy[a], zc])
        dn = np.array([*inst.xy[b], zc])
        add(pa, up, V_Z, "vert")
        add(up, dn, V_DRONE, "cruise")
        add(dn, pb, V_Z, "vert")
        if b >= N_DEPOTS:
            ev.append((t, t + SVC_TIME, pb, pb, "service"))
            t += SVC_TIME
    return ev


def sample_flight(inst, ev3d, cruise_z):
    """Sample every drone on a common DT_SIM grid.

    Returns times, P (T, K, 3), kind codes (T, K): 0 cruise, 1 vertical,
    2 service, 3 docked (finished)."""
    evs = [drone_events(inst, d["route"], d["origin"], cruise_z[d["origin"]])
           for d in ev3d["drones"]]
    t_end = max(e[-1][1] for e in evs)
    n = int(np.ceil(t_end / DT_SIM)) + 1
    times = np.arange(n) * DT_SIM
    K = len(evs)
    P = np.zeros((n, K, 3))
    kind = np.full((n, K), 3, dtype=int)
    code = {"cruise": 0, "vert": 1, "service": 2}
    for k, ev in enumerate(evs):
        j = 0
        for i, t in enumerate(times):
            while j < len(ev) and t > ev[j][1]:
                j += 1
            if j >= len(ev):
                P[i, k] = ev[-1][3]
                continue
            t0, t1, p0, p1, kd = ev[j]
            s = (t - t0) / (t1 - t0) if t1 > t0 else 1.0
            P[i, k] = p0 + s * (p1 - p0)
            kind[i, k] = code[kd]
    # physical invariants: nothing underground, no teleporting
    assert P[:, :, 2].min() >= -1e-9
    step = np.linalg.norm(np.diff(P, axis=0), axis=2)
    assert step.max() <= max(V_DRONE, V_Z) * DT_SIM * (1 + 1e-6) + 1e-9
    return times, P, kind, t_end


def separation_stats(ev3d, times, P, kind, check_layers):
    """Loss-of-separation between drones of different fleets:
    horizontal < R_SEP_H and vertical < DZ_SEP, both airborne."""
    origins = [d["origin"] for d in ev3d["drones"]]
    K = len(origins)
    los_pairs, los_steps, los_vert, los_svc = 0, 0, 0, 0
    min_d3 = np.inf
    for a in range(K):
        for b in range(a + 1, K):
            if origins[a] == origins[b]:
                continue
            air = (kind[:, a] != 3) & (kind[:, b] != 3)
            h = np.linalg.norm(P[:, a, :2] - P[:, b, :2], axis=1)
            v = np.abs(P[:, a, 2] - P[:, b, 2])
            if check_layers:
                both_cruise = air & (kind[:, a] == 0) & (kind[:, b] == 0)
                assert np.all(v[both_cruise] >= DZ_SEP - 1e-9)
            d3 = np.sqrt(h ** 2 + v ** 2)[air]
            if len(d3):
                min_d3 = min(min_d3, float(d3.min()))
            los = air & (h < R_SEP_H) & (v < DZ_SEP)
            if los.any():
                los_pairs += 1
                los_steps += int(los.sum())
                vert = (kind[:, a] == 1) | (kind[:, b] == 1)
                svc = (kind[:, a] == 2) | (kind[:, b] == 2)
                los_vert += int((los & vert).sum())
                los_svc += int((los & ~vert & svc).sum())
                if check_layers:      # cruise-cruise LoS is impossible
                    assert not np.any(los & ~vert & ~svc)
    return dict(pairs=los_pairs, steps=los_steps, vert_steps=los_vert,
                svc_steps=los_svc, min_d3=min_d3)


# ─── Simulation driver ────────────────────────────────────────────────────────

def run_instance(seed):
    inst = Instance(seed)
    plans = {(md, s): plan(inst, md, s) for md in MODES for s in STRATS}
    evals = {k: evaluate_3d(inst, p) for k, p in plans.items()}
    return inst, plans, evals


def run_simulation():
    """Showcase (card seed 7) plus Monte-Carlo over customer layouts."""
    inst, plans, evals = run_instance(SEED)
    flights = {}
    for md in MODES:
        ev = evals[(md, "AO")]
        times, P, kind, t_end = sample_flight(inst, ev, CRUISE_Z)
        sep = separation_stats(ev, times, P, kind, check_layers=True)
        tf, Pf, kf, _ = sample_flight(inst, ev, np.full(N_DEPOTS, Z_FLAT))
        sep_flat = separation_stats(ev, tf, Pf, kf, check_layers=False)
        flights[md] = dict(times=times, P=P, kind=kind, t_end=t_end,
                           sep=sep, sep_flat=sep_flat)
    mc = []
    for s in MC_SEEDS:
        mi, mp, me = run_instance(s)
        # does the 3D cost change the Nearest-Depot start that AO begins from?
        row = {"seed": s,
               "nd_changed": bool(np.any(nearest_depot_partition(mi, "2d")
                                         != nearest_depot_partition(mi, "3d")))}
        for st in STRATS:
            f, t = me[("2d", st)], me[("3d", st)]
            row[st] = dict(
                flat=None if f is None else f["cost"],
                d3=None if t is None else t["cost"],
                flat_obj=None if f is None else f["objective"],
                d3_obj=None if t is None else t["objective"],
                why=[h for md in MODES for h in mp[(md, st)]["how"]
                     if not mp[(md, st)]["feasible"]
                     and h in ("fleet", "inventory", "range")],
                moved=int(np.sum(mp[("2d", st)]["part"]
                                 != mp[("3d", st)]["part"])),
                d2_part=np.bincount(mp[("2d", st)]["part"],
                                    minlength=N_DEPOTS),
                d3_part=np.bincount(mp[("3d", st)]["part"],
                                    minlength=N_DEPOTS))
        t = me[("3d", "AO")]
        if t is not None:
            tm, Pm, km, _ = sample_flight(mi, t, CRUISE_Z)
            row["sep"] = separation_stats(t, tm, Pm, km, True)
            tf, Pf, kf, _ = sample_flight(mi, t, np.full(N_DEPOTS, Z_FLAT))
            row["sep_flat"] = separation_stats(t, tf, Pf, kf, False)
        mc.append(row)
    return dict(inst=inst, plans=plans, evals=evals, flights=flights, mc=mc)


# ─── Plotting ─────────────────────────────────────────────────────────────────

def _save(fig, name):
    fig.savefig(os.path.join(OUT_DIR, name), dpi=120)
    plt.close(fig)
    print(f"  Saved: {name}")


def plot_trajectory_3d(res):
    inst, fl = res["inst"], res["flights"]["3d"]
    ev = res["evals"][("3d", "AO")]
    fig = plt.figure(figsize=(11, 8.5))
    ax = fig.add_subplot(111, projection="3d")
    P = fl["P"]
    for k, d in enumerate(ev["drones"]):
        col = DEPOT_COLORS[d["origin"]]
        # split off the final return leg (after the last service)
        last_c = d["route"][-2]
        idx = np.where(np.all(np.isclose(P[:, k, :2], inst.xy[last_c]), axis=1)
                       & (fl["kind"][:, k] == 2))[0]
        cut = idx[-1] if len(idx) else len(P) - 1
        ax.plot(P[:cut + 1, k, 0], P[:cut + 1, k, 1], P[:cut + 1, k, 2],
                color=col, lw=1.4)
        ls = "--" if d["ret"] != d["origin"] else "-"
        ax.plot(P[cut:, k, 0], P[cut:, k, 1], P[cut:, k, 2], color=col, lw=1.4,
                ls=ls)
    part = res["plans"][("3d", "AO")]["part"]
    for c in range(N_CUSTOMERS):
        ax.scatter(*inst.cust_xy[c], CUST_Z, color=DEPOT_COLORS[part[c]],
                   s=30, edgecolors="k", linewidths=0.4)
    for m in range(N_DEPOTS):
        ax.scatter(*DEPOTS_XY[m], DEPOT_Z[m], marker="^", s=200,
                   color=DEPOT_COLORS[m], edgecolors="k", linewidths=1.0)
        ax.text(DEPOTS_XY[m, 0], DEPOTS_XY[m, 1], DEPOT_Z[m] + 3,
                f"{DEPOT_LABELS[m]} z={DEPOT_Z[m]:.0f} m\ncruise "
                f"{CRUISE_Z[m]:.1f} m", fontsize=7, color=DEPOT_COLORS[m])
    ax.set_xlim(0, 1200); ax.set_ylim(0, 1200); ax.set_zlim(0, 35)
    ax.set_box_aspect((1, 1, 0.45))
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)"); ax.set_zlabel("z (m)")
    ax.view_init(elev=22, azim=-60)
    handles = [mpatches.Patch(color=DEPOT_COLORS[m],
                              label=f"{DEPOT_LABELS[m]} fleet")
               for m in range(N_DEPOTS)]
    handles += [Line2D([0], [0], color="gray", ls="--",
                       label="Open return (other depot)")]
    ax.legend(handles=handles, loc="upper left", fontsize=8)
    ax.set_title(f"S030 3D - AO 3D plan, seed {SEED}: "
                 f"{ev['cost']:.0f} m-eq cost, "
                 f"{ev['length']:.0f} m flown", fontsize=11)
    plt.tight_layout()
    _save(fig, "trajectory_3d.png")


def plot_altitude_timeseries(res):
    fl, ev = res["flights"]["3d"], res["evals"][("3d", "AO")]
    times, P, kind = fl["times"], fl["P"], fl["kind"]
    K = P.shape[1]
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True,
                             gridspec_kw=dict(height_ratios=[3, 1.4]))
    for k, d in enumerate(ev["drones"]):
        z = P[:, k, 2].copy()
        z[kind[:, k] == 3] = np.nan
        axes[0].plot(times, z, color=DEPOT_COLORS[d["origin"]], lw=1.2,
                     alpha=0.85, label=f"{d['name']}->D{d['ret']+1}")
    for m in range(N_DEPOTS):
        axes[0].axhline(CRUISE_Z[m], color=DEPOT_COLORS[m], ls=":", lw=0.8)
    axes[0].axhline(CUST_Z, color="k", ls="--", lw=0.6)
    axes[0].set_ylabel("z (m)")
    axes[0].set_title(f"S030 3D - altitude per drone (AO 3D plan, seed {SEED}); "
                      "dotted = fleet cruise layers, dashed = delivery 1 m")
    axes[0].legend(fontsize=7, ncol=5, loc="upper right")
    axes[0].grid(True, ls="--", alpha=0.4)
    origins = [d["origin"] for d in ev["drones"]]
    for lbl, (Pq, kq, col) in [("tiered layers", (P, kind, "#4363d8")),
                               (f"single layer z={Z_FLAT:.0f} m",
                                (None, None, "#888888"))]:
        if Pq is None:
            tf, Pq, kq, _ = sample_flight(res["inst"], ev,
                                          np.full(N_DEPOTS, Z_FLAT))
            tq = tf
        else:
            tq = times
        md = np.full(len(tq), np.nan)
        for a in range(K):
            for b in range(a + 1, K):
                if origins[a] == origins[b]:
                    continue
                air = (kq[:, a] != 3) & (kq[:, b] != 3)
                d3 = np.linalg.norm(Pq[:, a] - Pq[:, b], axis=1)
                d3[~air] = np.nan
                md = np.fmin(md, d3)
        axes[1].plot(tq, md, color=col, lw=1.2, label=lbl)
    axes[1].axhline(R_SEP_H, color="r", ls="--", lw=0.8,
                    label=f"{R_SEP_H:.0f} m")
    axes[1].set_yscale("log")
    axes[1].set_ylabel("min inter-fleet\n3D distance (m)")
    axes[1].set_xlabel("time (s)")
    axes[1].legend(fontsize=8)
    axes[1].grid(True, ls="--", alpha=0.4)
    plt.tight_layout()
    _save(fig, "altitude_timeseries.png")


def plot_strategy_comparison(res):
    ev = res["evals"]
    x = np.arange(len(STRATS))
    fig, axes = plt.subplots(1, 3, figsize=(16, 5))
    cols = ["#4363d8", "#3cb44b", "#e6194b"]
    vals = [ev[("3d", s)]["cost"] for s in STRATS]
    bars = axes[0].bar(x, vals, 0.55, color=cols, edgecolor="k")
    for b, v in zip(bars, vals):
        axes[0].text(b.get_x() + b.get_width() / 2, v, f"{v:.0f}",
                     ha="center", va="bottom", fontsize=9)
    axes[0].set_title("3D energy-weighted cost, open return (m-eq)",
                      fontsize=10)
    hl = np.array([ev[("3d", s)]["hlen"] for s in STRATS])
    vl = np.array([ev[("3d", s)]["vlen"] for s in STRATS])
    ce = np.array([MGA * ev[("3d", s)]["e_climb"] / (M_DRONE * G)
                   for s in STRATS])
    axes[1].bar(x, hl, 0.55, color="#9999cc", edgecolor="k",
                label="horizontal length")
    axes[1].bar(x, vl, 0.55, bottom=hl, color="#66aa66", edgecolor="k",
                label="vertical length")
    axes[1].bar(x, ce, 0.55, bottom=hl + vl, color="#dd8844", edgecolor="k",
                label="climb penalty a_E m g dz")
    axes[1].set_ylim(0.9 * hl.min(), 1.03 * (hl + vl + ce).max())
    axes[1].set_title("Cost breakdown (m-eq; y-axis truncated)", fontsize=10)
    axes[1].legend(fontsize=8, loc="lower right")
    bottom = np.zeros(len(STRATS))
    for m in range(N_DEPOTS):
        v = np.array([ev[("3d", s)]["n"][m] for s in STRATS])
        axes[2].bar(x, v, 0.55, bottom=bottom, color=DEPOT_COLORS[m],
                    edgecolor="k", label=DEPOT_LABELS[m])
        for xi, (b, vv) in enumerate(zip(bottom, v)):
            axes[2].text(xi, b + vv / 2, str(vv), ha="center", va="center",
                         fontsize=8, color="white")
        bottom += v
    axes[2].set_title("Deliveries per depot (ideal 5 each)", fontsize=10)
    axes[2].set_ylim(0, 23.5)
    axes[2].legend(fontsize=8, loc="upper center", ncol=4)
    for ax in axes:
        ax.set_xticks(x); ax.set_xticklabels(STRATS)
        ax.grid(axis="y", ls="--", alpha=0.5)
    plt.suptitle(f"S030 3D - strategy comparison, 3D plans, seed {SEED}")
    plt.tight_layout()
    _save(fig, "strategy_comparison.png")


def plot_flat_vs_3d(res):
    ev, mc = res["evals"], res["mc"]
    fig, axes = plt.subplots(1, 3, figsize=(17, 5.2))
    x = np.arange(len(STRATS))
    w = 0.38
    for i, (md, col) in enumerate([("2d", "#aaaaaa"), ("3d", "#4363d8")]):
        vals = [ev[(md, s)]["cost"] for s in STRATS]
        bars = axes[0].bar(x + (i - 0.5) * w, vals, w, color=col,
                           edgecolor="k", label=MODE_NAME[md])
        for b, v in zip(bars, vals):
            axes[0].text(b.get_x() + b.get_width() / 2, v, f"{v:.0f}",
                         ha="center", va="bottom", fontsize=8)
    lo = min(ev[(md, s)]["cost"] for md in MODES for s in STRATS)
    axes[0].set_ylim(0.9 * lo, None)
    axes[0].set_title(f"Seed {SEED}: 3D cost when flown in tiers (m-eq)",
                      fontsize=10)
    axes[0].legend(fontsize=8)
    # deliveries per depot, flat vs 3D, AO
    for i, md in enumerate(MODES):
        bottom = 0
        for m in range(N_DEPOTS):
            v = ev[(md, "AO")]["n"][m]
            axes[1].bar(i, v, 0.5, bottom=bottom, color=DEPOT_COLORS[m],
                        edgecolor="k",
                        label=DEPOT_LABELS[m] if i == 0 else None)
            axes[1].text(i, bottom + v / 2, str(v), ha="center", va="center",
                         color="white", fontsize=9)
            bottom += v
    axes[1].set_xticks([0, 1])
    axes[1].set_xticklabels(["AO flat plan", "AO 3D plan"])
    axes[1].set_title(f"Seed {SEED}: AO deliveries per depot", fontsize=10)
    axes[1].set_ylim(0, 23.5)
    axes[1].legend(fontsize=8, loc="upper center", ncol=4)
    # Monte-Carlo saving distribution
    data, labels = [], []
    for s in STRATS:
        sv = [r[s]["flat"] - r[s]["d3"] for r in mc
              if r[s]["flat"] is not None and r[s]["d3"] is not None]
        data.append(sv)
        labels.append(f"{s}\n(n={len(sv)})")
    axes[2].boxplot(data, showmeans=True)
    axes[2].set_xticks(range(1, len(STRATS) + 1))
    axes[2].set_xticklabels(labels, fontsize=8)
    axes[2].axhline(0, color="k", lw=0.8)
    axes[2].set_ylabel("flat cost - 3D cost (m-eq), >0 = 3D plan better")
    axes[2].set_title(f"Monte-Carlo, {len(mc)} layouts", fontsize=10)
    for ax in axes:
        ax.grid(axis="y", ls="--", alpha=0.5)
    axes[0].set_xticks(x); axes[0].set_xticklabels(STRATS, fontsize=9)
    plt.suptitle("S030 3D - flat S030 plan vs 3D-aware plan, both flown in 3D")
    plt.tight_layout()
    _save(fig, "flat_vs_3d.png")


def plot_ao_convergence(res):
    h = res["plans"][("3d", "AO")]["history"]
    h2 = res["plans"][("2d", "AO")]["history"]
    fig, ax = plt.subplots(figsize=(7, 4))
    ax.plot(range(1, len(h) + 1), h, "o-", color="#e6194b",
            label="3D objective (3D plan)")
    ax.plot(range(1, len(h2) + 1), h2, "s--", color="#888888",
            label="2D objective (flat plan, S030)")
    ax.set_xticks(range(1, max(len(h), len(h2)) + 1))
    ax.set_xlabel("AO iteration (cost at the start of the sweep)")
    ax.set_ylabel("home-return cost + balance penalty\n(3D: m-eq, 2D: m)")
    ax.set_title(f"S030 3D - AO convergence, seed {SEED}")
    ax.legend(fontsize=8)
    ax.grid(True, ls="--", alpha=0.5)
    plt.tight_layout()
    _save(fig, "ao_convergence.png")


def plot_per_drone_summary(res):
    ev = res["evals"][("3d", "AO")]
    rows = ev["drones"]
    fig, axes = plt.subplots(1, 2, figsize=(14, 5.5),
                             gridspec_kw=dict(width_ratios=[1, 1.25]))
    x = np.arange(len(rows))
    cols = [DEPOT_COLORS[d["origin"]] for d in rows]
    axes[0].bar(x, [d["hlen"] for d in rows], color=cols, edgecolor="k",
                label="horizontal")
    axes[0].bar(x, [d["vlen"] for d in rows], bottom=[d["hlen"] for d in rows],
                color="white", edgecolor="k", hatch="//", label="vertical")
    axes[0].axhline(R_MAX, color="red", ls="--", label=f"R_max {R_MAX:.0f} m")
    axes[0].set_xticks(x)
    axes[0].set_xticklabels([f"{d['name']}->D{d['ret']+1}" for d in rows],
                            rotation=45, ha="right", fontsize=8)
    axes[0].set_ylabel("3D sortie length (m)")
    axes[0].legend(fontsize=8)
    axes[0].grid(axis="y", ls="--", alpha=0.5)
    axes[1].axis("off")
    hdr = ["Drone", "Origin (z)", "Return", "Stops", "3D len m", "vert m",
           "Climb J", "Cost m-eq", "Load kg"]
    cell = [[d["name"], f"D{d['origin']+1} ({CRUISE_Z[d['origin']]:g} m)",
             f"D{d['ret']+1}", str(d["stops"]), f"{d['length']:.1f}",
             f"{d['vlen']:.1f}", f"{d['e_climb']:.0f}", f"{d['cost']:.1f}",
             f"{d['load']:.2f}"] for d in rows]
    tb = axes[1].table(cellText=cell, colLabels=hdr, loc="center",
                       cellLoc="center")
    tb.auto_set_font_size(False); tb.set_fontsize(8); tb.scale(1, 1.35)
    for i, d in enumerate(rows):
        tb[i + 1, 0].set_facecolor(DEPOT_COLORS[d["origin"]])
        tb[i + 1, 0].get_text().set_color("white")
    plt.suptitle(f"S030 3D - per-drone summary, AO 3D plan, seed {SEED} "
                 "(cruise layer in brackets)")
    plt.tight_layout()
    _save(fig, "per_drone_summary.png")


# ─── Animation ────────────────────────────────────────────────────────────────

def save_animation(res, max_frames=120, fps=10):
    """3D GIF of the AO 3D-plan sorties flown simultaneously."""
    inst, fl = res["inst"], res["flights"]["3d"]
    ev = res["evals"][("3d", "AO")]
    times, P, kind = fl["times"], fl["P"], fl["kind"]
    n = len(times)
    step = max(1, int(np.ceil((n - 1) / (max_frames - 1))))
    n_frames = int(np.ceil((n - 1) / step)) + 1
    served_t = {}
    for k, d in enumerate(ev["drones"]):
        for c in d["route"][1:-1]:
            hit = np.where((kind[:, k] == 2) &
                           np.all(np.isclose(P[:, k, :2], inst.xy[c]),
                                  axis=1))[0]
            served_t[c] = times[hit[0]] if len(hit) else np.inf
    fig = plt.figure(figsize=(8, 7))
    ax = fig.add_subplot(111, projection="3d")
    ax.set_xlim(0, 1200); ax.set_ylim(0, 1200); ax.set_zlim(0, 35)
    ax.set_box_aspect((1, 1, 0.45))
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)"); ax.set_zlabel("z (m)")
    ax.view_init(elev=22, azim=-60)
    for m in range(N_DEPOTS):
        ax.scatter(*DEPOTS_XY[m], DEPOT_Z[m], marker="^", s=160,
                   color=DEPOT_COLORS[m], edgecolors="k")
    cust = ax.scatter(inst.cust_xy[:, 0], inst.cust_xy[:, 1],
                      np.full(N_CUSTOMERS, CUST_Z), s=30, color="#bbbbbb",
                      edgecolors="k", depthshade=False)
    origins = [d["origin"] for d in ev["drones"]]
    dots = [ax.plot([], [], [], "o", color=DEPOT_COLORS[o], ms=6,
                    markeredgecolor="k")[0] for o in origins]
    trails = [ax.plot([], [], [], "-", color=DEPOT_COLORS[o], lw=1.0,
                      alpha=0.6)[0] for o in origins]
    ax.legend(handles=[mpatches.Patch(color=DEPOT_COLORS[m],
                                      label=f"{DEPOT_LABELS[m]} "
                                            f"(cruise {CRUISE_Z[m]:g} m)")
                       for m in range(N_DEPOTS)] +
              [Line2D([], [], marker="o", ls="", color="gold",
                      label="Customer served")],
              fontsize=7, loc="upper left")
    ttl = ax.set_title("")

    def update(f):
        i = min(f * step, n - 1)
        t = times[i]
        fc = ["gold" if served_t.get(c + N_DEPOTS, np.inf) <= t
              else "#bbbbbb" for c in range(N_CUSTOMERS)]
        cust.set_facecolor(fc)
        for k in range(len(origins)):
            dots[k].set_data([P[i, k, 0]], [P[i, k, 1]])
            dots[k].set_3d_properties([P[i, k, 2]])
            lo = max(0, i - 40)
            trails[k].set_data(P[lo:i + 1, k, 0], P[lo:i + 1, k, 1])
            trails[k].set_3d_properties(P[lo:i + 1, k, 2])
        ttl.set_text(f"S030 3D Multi-Depot (AO 3D plan) - t = {t:.0f} s")
        return dots + trails + [cust, ttl]

    anim = FuncAnimation(fig, update, frames=n_frames, blit=False)
    anim.save(os.path.join(OUT_DIR, "animation.gif"),
              writer=PillowWriter(fps=fps))
    plt.close(fig)
    print(f"  Saved: animation.gif ({n_frames} frames, {fps} fps, step "
          f"{step} x {DT_SIM} s, {fl['t_end']:.1f} s of flight)")


# ─── Reporting ────────────────────────────────────────────────────────────────

def _fmt_part_change(inst, pa, pb):
    out = []
    for c in range(N_CUSTOMERS):
        if pa[c] != pb[c]:
            out.append(f"c{c+1}:D{pa[c]+1}->D{pb[c]+1}")
    return ", ".join(out) if out else "none"


def report(res):
    inst, plans, evals, mc = res["inst"], res["plans"], res["evals"], res["mc"]
    print(f"  Fleet/depot {FLEET_SIZE.tolist()}, inventory "
          f"{INVENTORY.tolist()} kg; depot z {DEPOT_Z.tolist()} m; cruise "
          f"layers {CRUISE_Z.tolist()} m")
    print(f"  Total demand {inst.demand.sum():.2f} kg; climb penalty "
          f"a_E*m*g = {MGA:.4f} m-eq per m of climb")
    print("  Extra 3D cost of one more stop (descend + re-climb) per fleet: "
          + ", ".join(f"D{m+1} {2*(CRUISE_Z[m]-CUST_Z) + MGA*(CRUISE_Z[m]-CUST_Z):.1f}"
                      for m in range(N_DEPOTS)) + " m-eq")
    print("\n[Regression] flat plans scored with the 2D S030 metric "
          "(open-return distance, m):")
    for s in STRATS:
        p = plans[("2d", s)]
        print(f"  {s:<17} {eval_2d(inst, p):8.1f}  deliveries/depot "
              f"{np.bincount(p['part'], minlength=N_DEPOTS).tolist()}")
    for md in MODES:
        print(f"\n=== {MODE_NAME[md]}, flown and scored in 3D (seed {SEED}) ===")
        for s in STRATS:
            p, e = plans[(md, s)], evals[(md, s)]
            print(f"--- {s} ---")
            if "km_iter" in p:
                print(f"  K-Means iterations: {p['km_iter']}")
            if "history" in p:
                print(f"  AO iterations: {len(p['history'])}, objective "
                      + " -> ".join(f"{h:.1f}" for h in p["history"]))
                inf = inst.infeasible[md]
                print(f"  AO candidates rejected as infeasible ({md} model): "
                      f"{inf['partitions']} (fleet {inf['fleet']}, inventory "
                      f"{inf['inventory']}, range {inf['range']})")
            print(f"  Deliveries/depot {e['n'].tolist()} (penalty "
                  f"{e['penalty']:.1f}); sorties/depot {e['sorties']} "
                  f"(packing {p['how']})")
            print(f"  Cargo/depot {[round(float(c), 2) for c in e['cargo']]} kg;"
                  f" max sortie load {e['max_load']:.2f} kg; max 3D sortie "
                  f"length {e['max_len']:.1f} m")
            print(f"  3D cost home-return {e['home_cost']:.1f}, open-return "
                  f"{e['cost']:.1f} m-eq (open-route saving {e['saving']:.1f}); "
                  f"objective {e['objective']:.1f}")
            print(f"  3D length {e['length']:.1f} m (horizontal {e['hlen']:.1f}"
                  f", vertical {e['vlen']:.1f}); climb energy "
                  f"{e['e_climb']:.0f} J")
    print("\n=== Flat plan vs 3D plan, both flown in 3D (seed 7) ===")
    for s in STRATS:
        f, t = evals[("2d", s)], evals[("3d", s)]
        print(f"  {s:<17} flat {f['cost']:8.1f}  3D {t['cost']:8.1f}  "
              f"saving {f['cost'] - t['cost']:+7.1f} m-eq "
              f"({(f['cost'] - t['cost']) / f['cost'] * 100:+.2f}%); "
              f"objective flat {f['objective']:.1f} vs 3D {t['objective']:.1f}"
              f"; moved: {_fmt_part_change(inst, plans[('2d', s)]['part'], plans[('3d', s)]['part'])}")
    for md in MODES:
        rets = [(d["name"], d["ret"]) for d in evals[(md, "AO")]["drones"]
                if d["ret"] != d["origin"]]
        print(f"  AO {md} open returns: "
              + (", ".join(f"{n}->D{r+1}" for n, r in rets) if rets
                 else "none"))
    print("\nPER-DRONE SUMMARY (AO, 3D plan):")
    for d in evals[("3d", "AO")]["drones"]:
        print(f"  {d['name']:<6} origin D{d['origin']+1} (cruise "
              f"{CRUISE_Z[d['origin']]:4.1f} m) return D{d['ret']+1} stops "
              f"{d['stops']} 3D len {d['length']:7.1f} m (vert "
              f"{d['vlen']:5.1f}) climb {d['e_climb']:5.0f} J cost "
              f"{d['cost']:7.1f} load {d['load']:.2f} kg")
    for md in MODES:
        fl = res["flights"][md]
        print(f"\nSeparation, AO {md} plan (dt {DT_SIM} s, LoS = horizontal "
              f"< {R_SEP_H:.0f} m and vertical < {DZ_SEP:.0f} m):")
        for lbl, sp in [("tiered layers", fl["sep"]),
                        (f"single layer {Z_FLAT:.0f} m", fl["sep_flat"])]:
            print(f"  {lbl:<18} LoS pairs {sp['pairs']}, LoS samples "
                  f"{sp['steps']} (vertical transition {sp['vert_steps']}, "
                  f"delivery hover {sp['svc_steps']}), min inter-fleet 3D "
                  f"distance "
                  f"{sp['min_d3']:.1f} m")
        print(f"  makespan {fl['t_end']:.1f} s")
    print(f"\n=== Monte-Carlo: {len(mc)} customer layouts (seeds "
          f"{MC_SEEDS[0]}-{MC_SEEDS[-1]}), same layout for every strategy ===")
    for s in STRATS:
        both = [r for r in mc if r[s]["flat"] is not None
                and r[s]["d3"] is not None]
        nf = sum(r[s]["flat"] is None for r in mc)
        n3 = sum(r[s]["d3"] is None for r in mc)
        sv = np.array([r[s]["flat"] - r[s]["d3"] for r in both])
        so = np.array([r[s]["flat_obj"] - r[s]["d3_obj"] for r in both])
        rel = np.array([(r[s]["flat"] - r[s]["d3"]) / r[s]["flat"] * 100
                        for r in both])
        mv = np.array([r[s]["moved"] for r in both])
        d2share = np.mean([r[s]["d2_part"][1] for r in both])
        d3share = np.mean([r[s]["d3_part"][1] for r in both])
        why = [h for r in mc for h in r[s]["why"]]
        print(f"  {s:<17} infeasible flat {nf}, 3D {n3}; compared {len(both)}"
              f"; depot-level failure reasons (both planners): fleet "
              f"{why.count('fleet')}, inventory {why.count('inventory')}, "
              f"range {why.count('range')}")
        print(f"    open-return 3D cost saving of 3D plan: mean {sv.mean():+.1f}"
              f" +- {sv.std():.1f} m-eq (median {np.median(sv):+.1f}, min "
              f"{sv.min():+.1f}, max {sv.max():+.1f}); mean {rel.mean():+.2f}%")
        print(f"    wins/ties/losses (|diff| <= 0.5 m-eq tie): "
              f"{int((sv > 0.5).sum())}/{int((np.abs(sv) <= 0.5).sum())}/"
              f"{int((sv < -0.5).sum())}; objective (home cost + penalty) "
              f"wins/ties/losses {int((so > 0.5).sum())}/"
              f"{int((np.abs(so) <= 0.5).sum())}/{int((so < -0.5).sum())}")
        nz = np.abs(sv)[sv != 0.0]
        print(f"    exact wins/ties/losses (tie = diff exactly 0): "
              f"{int((sv > 0).sum())}/{int((sv == 0).sum())}/"
              f"{int((sv < 0).sum())}; nonzero diffs inside the 0.5 m-eq tie "
              f"band {int(((sv != 0) & (np.abs(sv) <= 0.5)).sum())}"
              + (f" (smallest |diff| {nz.min():.3f} m-eq)" if len(nz) else ""))
        if s == "AO":
            ndc = np.array([r["nd_changed"] for r in both])
            for lbl, msk in [("losses (diff < -0.5)", sv < -0.5),
                             ("objective losses", so < -0.5),
                             ("wins (diff > 0.5)", sv > 0.5)]:
                print(f"    AO {lbl}: {int(msk.sum())}, of which the 3D cost "
                      f"changed the Nearest-Depot start in "
                      f"{int((msk & ndc).sum())}")
        print(f"    customers reassigned: mean {mv.mean():.2f}, max {mv.max()},"
              f" layouts with any change {int((mv > 0).sum())}; mean D2 "
              f"(rooftop) deliveries flat {d2share:.2f} -> 3D {d3share:.2f}")
    sp = [r for r in mc if "sep" in r]
    tl = np.array([r["sep"]["pairs"] for r in sp])
    fl_ = np.array([r["sep_flat"]["pairs"] for r in sp])
    tv = np.array([r["sep"]["vert_steps"] for r in sp])
    ts = np.array([r["sep"]["steps"] for r in sp])
    tsv = np.array([r["sep"]["svc_steps"] for r in sp])
    fs = np.array([r["sep_flat"]["steps"] for r in sp])
    print(f"  Separation (AO 3D plans, {len(sp)} layouts): LoS pairs tiered "
          f"mean {tl.mean():.2f} (layouts with any {int((tl > 0).sum())}), "
          f"single layer mean {fl_.mean():.2f} (layouts with any "
          f"{int((fl_ > 0).sum())}); LoS samples tiered {int(ts.sum())} "
          f"(vertical transition {int(tv.sum())}, delivery hover "
          f"{int(tsv.sum())}), single layer {int(fs.sum())}; tiered fewer/"
          f"equal/more than single layer: {int((tl < fl_).sum())}/"
          f"{int((tl == fl_).sum())}/{int((tl > fl_).sum())}")


# ─── Main ─────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    print("=" * 70)
    print("S030 3D Upgrade - Multi-Depot Delivery (altitude tiers)")
    print("=" * 70)
    res = run_simulation()
    report(res)
    print("\nGenerating plots...")
    plot_trajectory_3d(res)
    plot_altitude_timeseries(res)
    plot_strategy_comparison(res)
    plot_flat_vs_3d(res)
    plot_ao_convergence(res)
    plot_per_drone_summary(res)
    save_animation(res)
    print(f"\nAll outputs saved to: {OUT_DIR}")
