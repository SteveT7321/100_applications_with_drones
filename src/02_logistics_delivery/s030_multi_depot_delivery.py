"""
S030 Multi-Depot Delivery
=========================
4 depots, 10 drones (fleet [3, 2, 3, 2]), 20 customers.

Constraints enforced for every strategy:
  * one sortie per drone (card formulation: one route per drone), so a depot
    can fly at most F_m sorties;
  * per sortie: payload <= Q, length (with home return) <= R_max;
  * per depot: dispatched cargo <= inventory I_m.
A depot's customer set is packed into sorties by nearest-neighbour; if that
needs more than F_m sorties, a first-fit-decreasing bin packing into F_m
sorties is tried.  A partition that still does not fit is infeasible.

Partition strategies:
  1. Nearest-Depot      : each customer to its closest depot
  2. Balanced K-Means   : depot-anchored K-Means (centroid = 0.5 depot +
                          0.5 cluster mean, as in the card) whose assignment
                          step caps every cluster at ceil(N/D) customers
                          (regret order) - targets the card's balance term
  3. Alternating Opt.   : start from Nearest-Depot, move single customers to
                          the depot that lowers distance + balance penalty
                          (routes rebuilt + 2-opt), until convergence

After routing, every sortie's landing depot is chosen by an optimal
assignment to docking slots (slots = fleet size) that maximises the total
return saving subject to the range limit (open-route return).

Usage:
    conda activate drones
    python src/02_logistics_delivery/s030_multi_depot_delivery.py
"""

import os
import numpy as np
from scipy.spatial.distance import cdist
from scipy.optimize import linear_sum_assignment
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.lines import Line2D
from matplotlib.animation import FuncAnimation

OUT_DIR = os.path.normpath(os.path.join(
    os.path.dirname(__file__), "..", "..",
    "outputs", "02_logistics_delivery", "s030_multi_depot_delivery"))
os.makedirs(OUT_DIR, exist_ok=True)

# ─── Constants ────────────────────────────────────────────────────────────────
N_DEPOTS    = 4
N_CUSTOMERS = 20
N_DRONES    = 10
V_DRONE     = 14.0   # m/s
Q_MAX       = 3.0    # kg payload per sortie
R_MAX       = 3500.0 # m max range per sortie
SVC_TIME    = 8.0    # s service time per stop
LAMBDA      = 50.0   # balance penalty weight
EPS_AO      = 1.0    # convergence threshold
MAX_AO_ITER = 50
KM_ITER     = 100
SEED        = 7

DEPOT_COLORS = ["#e6194b", "#3cb44b", "#4363d8", "#f58231"]
DEPOT_LABELS = ["D1 SW", "D2 NW", "D3 SE", "D4 NE"]
STRATS = ["Nearest-Depot", "Balanced K-Means", "AO"]

DEPOTS     = np.array([[150., 150.], [150., 1050.], [1050., 150.],
                       [1050., 1050.]])
FLEET_SIZE = np.array([3, 2, 3, 2])
INVENTORY  = np.array([25., 18., 25., 20.])
assert FLEET_SIZE.sum() == N_DRONES

_rng      = np.random.default_rng(SEED)
CUSTOMERS = _rng.uniform(100, 1100, (N_CUSTOMERS, 2))
DEMANDS   = _rng.uniform(0.3, 1.8, N_CUSTOMERS)

ALL_POS = np.vstack([DEPOTS, CUSTOMERS])      # nodes: depots 0..3, then customers
DIST_M  = cdist(ALL_POS, ALL_POS)
N_BAR   = N_CUSTOMERS / N_DEPOTS
INFEASIBLE = {"partitions": 0, "inventory": 0, "fleet": 0, "range": 0}
# partitions = AO candidates rejected; the others count depot-level reasons


def route_dist(route):
    return sum(DIST_M[route[i], route[i + 1]] for i in range(len(route) - 1))


def load_of(route):
    return sum(DEMANDS[n - N_DEPOTS] for n in route if n >= N_DEPOTS)


def two_opt(route):
    route = list(route)
    improved = True
    while improved:
        improved = False
        for i in range(1, len(route) - 2):
            for j in range(i + 1, len(route) - 1):
                gain = (DIST_M[route[i-1], route[i]] + DIST_M[route[j], route[j+1]]
                        - DIST_M[route[i-1], route[j]] - DIST_M[route[i], route[j+1]])
                if gain > 1e-6:
                    route[i:j+1] = route[i:j+1][::-1]
                    improved = True
    return route


def nn_order(depot, nodes):
    route, rem, cur = [depot], list(nodes), depot
    while rem:
        nxt = min(rem, key=lambda c: (DIST_M[cur, c], c))
        route.append(nxt); rem.remove(nxt); cur = nxt
    return two_opt(route + [depot])


def build_routes_for_depot(m, cust_nodes):
    """Pack depot m's customers into <= FLEET_SIZE[m] feasible sorties.

    Returns (sorties, feasible flag, packing method or failure reason)."""
    if not cust_nodes:
        return [], True, "-"
    if load_of(cust_nodes) > INVENTORY[m] + 1e-9:
        return [], False, "inventory"
    # (a) nearest-neighbour sorties
    sorties, rem = [], list(cust_nodes)
    while rem:
        route, load, d, cur = [m], 0.0, 0.0, m
        while True:
            cand = [c for c in rem
                    if load + DEMANDS[c - N_DEPOTS] <= Q_MAX + 1e-9
                    and d + DIST_M[cur, c] + DIST_M[c, m] <= R_MAX]
            if not cand:
                break
            c = min(cand, key=lambda c: (DIST_M[cur, c], c))
            route.append(c); rem.remove(c)
            load += DEMANDS[c - N_DEPOTS]; d += DIST_M[cur, c]; cur = c
        sorties.append(two_opt(route + [m]))
    if len(sorties) <= FLEET_SIZE[m]:
        return sorties, True, "NN"
    # (b) first-fit-decreasing bin packing into F_m sorties
    bins = [[] for _ in range(FLEET_SIZE[m])]
    for c in sorted(cust_nodes, key=lambda c: (-DEMANDS[c - N_DEPOTS], c)):
        for b in bins:
            if load_of(b) + DEMANDS[c - N_DEPOTS] <= Q_MAX + 1e-9:
                b.append(c)
                break
        else:
            return [], False, "fleet"
    sorties = [nn_order(m, b) for b in bins if b]
    if all(route_dist(s) <= R_MAX for s in sorties):
        return sorties, True, "FFD"
    return [], False, "range"


def all_routes(partition):
    routes, ok, how = [], True, []
    for m in range(N_DEPOTS):
        nodes = [c + N_DEPOTS for c in range(N_CUSTOMERS) if partition[c] == m]
        r, f, h = build_routes_for_depot(m, nodes)
        routes.append(r)
        ok &= f
        how.append(h)
    return routes, ok, how


def balance_penalty(partition):
    n = np.bincount(partition, minlength=N_DEPOTS)
    return LAMBDA * float(np.sum((n - N_BAR) ** 2))


def total_cost(partition):
    """Home-return distance + balance penalty (inf if infeasible)."""
    routes, ok, how = all_routes(partition)
    if not ok:
        INFEASIBLE["partitions"] += 1
        for h in how:
            if h in INFEASIBLE:
                INFEASIBLE[h] += 1
        return np.inf
    return sum(route_dist(r) for rr in routes for r in rr) + \
        balance_penalty(partition)


# ─── Strategies ───────────────────────────────────────────────────────────────

def nearest_depot_partition():
    return np.argmin(cdist(CUSTOMERS, DEPOTS), axis=1)


def balanced_kmeans_partition():
    """Depot-anchored K-Means with a size cap of ceil(N/D) per cluster."""
    cap = int(np.ceil(N_BAR))
    centroids = DEPOTS.astype(float).copy()
    part = None
    for it in range(KM_ITER):
        d = cdist(CUSTOMERS, centroids)
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
            members = CUSTOMERS[part == m]
            if len(members):
                centroids[m] = 0.5 * DEPOTS[m] + 0.5 * members.mean(axis=0)
    return part, it + 1


def alternating_optimisation():
    partition = nearest_depot_partition().copy()
    history = []
    prev = np.inf
    for _ in range(MAX_AO_ITER):
        c_val = total_cost(partition)
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
                c2 = total_cost(partition)
                if c2 < best_c:
                    best_c, best_m = c2, m2
            partition[c] = best_m
            c_val = best_c
    return partition, history


def open_route_return(routes_by_depot):
    """Choose every sortie's landing depot to maximise the total saving.

    Optimal assignment of sorties to docking slots (FLEET_SIZE[m] slots at
    depot m) with scipy's linear_sum_assignment; a landing depot is allowed
    only if the sortie's total length stays within R_MAX.  Landing at home
    is always feasible, so the total saving is never negative."""
    sorties = [(m, r) for m, rr in enumerate(routes_by_depot) for r in rr]
    slots = [m for m in range(N_DEPOTS) for _ in range(FLEET_SIZE[m])]
    big = 1e9
    cost = np.full((len(sorties), len(slots)), big)
    for i, (m, r) in enumerate(sorties):
        last, home_len = r[-2], route_dist(r)
        for j, m2 in enumerate(slots):
            if home_len - DIST_M[last, m] + DIST_M[last, m2] <= R_MAX + 1e-9:
                cost[i, j] = -(DIST_M[last, m] - DIST_M[last, m2])
    rows, cols = linear_sum_assignment(cost)
    assert np.all(cost[rows, cols] < big)
    out = [[] for _ in range(N_DEPOTS)]
    saving = 0.0
    for i, j in zip(rows, cols):
        m, r = sorties[i]
        saving -= cost[i, j]
        out[m].append(r[:-1] + [slots[j]])
    return out, saving


def run_simulation():
    """Partition, route and evaluate the three strategies."""
    res = {}
    hist = None
    km_iter = None
    for s in STRATS:
        if s == "Nearest-Depot":
            part = nearest_depot_partition()
        elif s == "Balanced K-Means":
            part, km_iter = balanced_kmeans_partition()
        else:
            part, hist = alternating_optimisation()
        routes_home, ok, how = all_routes(part)
        routes, saving = open_route_return(routes_home)
        home_d = sum(route_dist(r) for rr in routes_home for r in rr)
        dist = sum(route_dist(r) for rr in routes for r in rr)
        n = np.bincount(part, minlength=N_DEPOTS)
        drones = []
        for m, rr in enumerate(routes):
            for k, r in enumerate(rr):
                stops = [x for x in r if x >= N_DEPOTS]
                drones.append(dict(
                    name=f"D{m+1}-#{k+1}", origin=m, ret=r[-1], route=r,
                    stops=len(stops), length=route_dist(r), load=load_of(r),
                    t_end=route_dist(r) / V_DRONE + SVC_TIME * len(stops)))
        res[s] = dict(
            part=part, routes=routes, feasible=ok, how=how, dist_home=home_d,
            dist=dist, saving=saving, n=n, bal_std=float(np.std(n)),
            penalty=balance_penalty(part),
            objective=home_d + balance_penalty(part),
            sorties=[len(rr) for rr in routes],
            cargo=[sum(load_of(r) for r in rr) for rr in routes],
            drones=drones, makespan=max(d["t_end"] for d in drones),
            max_load=max(d["load"] for d in drones),
            max_len=max(route_dist(r) for rr in routes_home for r in rr))
    return res, hist, km_iter


# ─── Plotting ─────────────────────────────────────────────────────────────────

def plot_route_map(r, title, filename):
    fig, ax = plt.subplots(figsize=(11.5, 9))
    ax.set_xlim(0, 1200); ax.set_ylim(0, 1200); ax.set_aspect("equal")
    ax.set_facecolor("#f8f8f8"); ax.grid(True, ls="--", alpha=0.4)
    for c in range(N_CUSTOMERS):
        ax.scatter(*CUSTOMERS[c], color=DEPOT_COLORS[r["part"][c]], s=60,
                   zorder=3, edgecolors="k", linewidths=0.5)
        ax.text(CUSTOMERS[c, 0] + 10, CUSTOMERS[c, 1] + 10, str(c + 1),
                fontsize=7, color="dimgray")
    for m, rr in enumerate(r["routes"]):
        for route in rr:
            xs, ys = ALL_POS[route, 0], ALL_POS[route, 1]
            ax.plot(xs[:-1], ys[:-1], color=DEPOT_COLORS[m], lw=1.4, alpha=0.8)
            ax.plot(xs[-2:], ys[-2:], color=DEPOT_COLORS[m], lw=1.4,
                    ls="--" if route[-1] != m else "-", alpha=0.8)
            ax.annotate("", xy=ALL_POS[route[1]], xytext=ALL_POS[route[0]],
                        arrowprops=dict(arrowstyle="->",
                                        color=DEPOT_COLORS[m], lw=0.8))
    for m in range(N_DEPOTS):
        ax.scatter(*DEPOTS[m], marker="^", s=250, color=DEPOT_COLORS[m],
                   zorder=5, edgecolors="k", linewidths=1.2)
        ax.text(DEPOTS[m, 0] + 20, DEPOTS[m, 1] + 20,
                f"{DEPOT_LABELS[m]} ({r['sorties'][m]}/{FLEET_SIZE[m]} drones)",
                fontsize=9, fontweight="bold", color=DEPOT_COLORS[m])
    handles = [mpatches.Patch(color=DEPOT_COLORS[m], label=DEPOT_LABELS[m])
               for m in range(N_DEPOTS)]
    handles += [Line2D([0], [0], color="gray", ls="-", label="Home return"),
                Line2D([0], [0], color="gray", ls="--",
                       label="Open return (other depot)")]
    ax.legend(handles=handles, loc="upper left", bbox_to_anchor=(1.01, 1.0),
              fontsize=8)
    ax.set_title(f"{title}\n{r['dist']:.0f} m, deliveries/depot "
                 f"{r['n'].tolist()}", fontsize=12)
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
    plt.tight_layout()
    _save(fig, filename)


def plot_strategy_comparison(res):
    x = np.arange(len(STRATS))
    fig, axes = plt.subplots(1, 3, figsize=(16, 5))
    cols = ["#4363d8", "#3cb44b", "#e6194b"]
    for ax, key, ttl in [(axes[0], "dist", "Total fleet distance, open return (m)"),
                         (axes[1], "objective",
                          "AO objective: home-return dist + balance penalty")]:
        vals = [res[s][key] for s in STRATS]
        bars = ax.bar(x, vals, 0.55, color=cols, edgecolor="k")
        for b, v in zip(bars, vals):
            ax.text(b.get_x() + b.get_width() / 2, v, f"{v:.0f}", ha="center",
                    va="bottom", fontsize=9)
        ax.set_title(ttl, fontsize=10); ax.set_xticks(x)
        ax.set_xticklabels(STRATS); ax.grid(axis="y", ls="--", alpha=0.5)
    bottom = np.zeros(len(STRATS))
    for m in range(N_DEPOTS):
        v = np.array([res[s]["n"][m] for s in STRATS])
        axes[2].bar(x, v, 0.55, bottom=bottom, color=DEPOT_COLORS[m],
                    edgecolor="k", label=DEPOT_LABELS[m])
        for xi, (b, vv) in enumerate(zip(bottom, v)):
            axes[2].text(xi, b + vv / 2, str(vv), ha="center", va="center",
                         fontsize=8, color="white")
        bottom += v
    axes[2].set_title("Deliveries per depot (ideal 5 each)", fontsize=10)
    axes[2].set_xticks(x); axes[2].set_xticklabels(STRATS)
    axes[2].legend(fontsize=8, loc="lower right")
    plt.suptitle("S030 — Strategy comparison")
    plt.tight_layout()
    _save(fig, "strategy_comparison.png")


def plot_ao_convergence(history):
    fig, ax = plt.subplots(figsize=(7, 4))
    ax.plot(range(1, len(history) + 1), history, "o-", color="#e6194b")
    ax.set_xlabel("AO iteration (cost evaluated at the start of the sweep)")
    ax.set_ylabel("Home-return distance + balance penalty")
    ax.set_title("S030 — Alternating optimisation convergence")
    ax.grid(True, ls="--", alpha=0.5)
    plt.tight_layout()
    _save(fig, "ao_convergence.png")


def plot_per_drone_summary(r, name):
    rows = r["drones"]
    x = np.arange(len(rows))
    cols = [DEPOT_COLORS[d["origin"]] for d in rows]
    labels = [f"{d['name']}→D{d['ret']+1}" for d in rows]
    fig, axes = plt.subplots(1, 2, figsize=(13, 5))
    axes[0].bar(x, [d["length"] for d in rows], color=cols, edgecolor="k")
    axes[0].axhline(R_MAX, color="red", ls="--", label=f"R_max {R_MAX:.0f} m")
    axes[0].set_ylabel("Sortie length (m)")
    axes[1].bar(x, [d["load"] for d in rows], color=cols, edgecolor="k")
    axes[1].axhline(Q_MAX, color="red", ls="--", label=f"Q {Q_MAX} kg")
    axes[1].set_ylabel("Payload (kg)")
    for ax in axes:
        ax.set_xticks(x); ax.set_xticklabels(labels, rotation=45, ha="right",
                                             fontsize=8)
        ax.legend(fontsize=8); ax.grid(axis="y", ls="--", alpha=0.5)
    plt.suptitle(f"S030 — Per-drone sorties, {name} (label: drone→return depot)")
    plt.tight_layout()
    _save(fig, "per_drone_metrics_ao.png")


def _save(fig, name):
    fig.savefig(os.path.join(OUT_DIR, name), dpi=120)
    plt.close(fig)
    print(f"  Saved: {name}")


# ─── Animation ────────────────────────────────────────────────────────────────

def save_animation(r, name, n_frames=130, fps=10):
    """Top-down GIF of all sorties flown simultaneously (<= 150 frames)."""
    tracks = []                    # per drone: list of (t0, t1, p0, p1, cust)
    for d in r["drones"]:
        t, ev = 0.0, []
        route = d["route"]
        for a, b in zip(route[:-1], route[1:]):
            t1 = t + DIST_M[a, b] / V_DRONE
            ev.append((t, t1, ALL_POS[a], ALL_POS[b], None))
            t = t1
            if b >= N_DEPOTS:
                ev.append((t, t + SVC_TIME, ALL_POS[b], ALL_POS[b], b))
                t += SVC_TIME
        tracks.append((d["origin"], ev))
    t_end = max(ev[-1][1] for _, ev in tracks)
    times = np.linspace(0, t_end, n_frames)

    def pos(ev, t):
        for t0, t1, p0, p1, _ in ev:
            if t0 <= t <= t1:
                return p0 + ((t - t0) / (t1 - t0) if t1 > t0 else 1) * (p1 - p0)
        return ev[-1][3]

    P = np.array([[pos(ev, t) for _, ev in tracks] for t in times])
    served_t = {}
    for _, ev in tracks:
        for t0, _, _, _, c in ev:
            if c is not None:
                served_t[c] = t0
    fig, ax = plt.subplots(figsize=(7.5, 7.5))
    ax.set_xlim(0, 1200); ax.set_ylim(0, 1200); ax.set_aspect("equal")
    ax.grid(True, ls="--", alpha=0.3)
    for m in range(N_DEPOTS):
        ax.scatter(*DEPOTS[m], marker="^", s=250, color=DEPOT_COLORS[m],
                   edgecolors="k", zorder=5)
        ax.text(DEPOTS[m, 0] + 20, DEPOTS[m, 1] + 20, DEPOT_LABELS[m],
                color=DEPOT_COLORS[m], fontweight="bold")
    cust = [ax.scatter(*CUSTOMERS[c], s=70, color="#bbbbbb",
                       edgecolors="k", zorder=4) for c in range(N_CUSTOMERS)]
    dots = [ax.plot([], [], "o", color=DEPOT_COLORS[o], ms=7,
                    markeredgecolor="k", zorder=6)[0] for o, _ in tracks]
    trails = [ax.plot([], [], "-", color=DEPOT_COLORS[o], lw=1.2,
                      alpha=0.5)[0] for o, _ in tracks]
    ttl = ax.set_title("")
    ax.legend(handles=[mpatches.Patch(color=DEPOT_COLORS[m],
                                      label=f"{DEPOT_LABELS[m]} drones")
                       for m in range(N_DEPOTS)] + [
        Line2D([], [], marker="o", ls="", color="limegreen",
               label="Customer served")], fontsize=8, loc="lower right")

    def update(f):
        t = times[f]
        for c in range(N_CUSTOMERS):
            if served_t.get(c + N_DEPOTS, np.inf) <= t:
                cust[c].set_facecolor("limegreen")
        for i in range(len(tracks)):
            dots[i].set_data([P[f, i, 0]], [P[f, i, 1]])
            lo = max(0, f - 6)
            trails[i].set_data(P[lo:f + 1, i, 0], P[lo:f + 1, i, 1])
        ttl.set_text(f"S030 Multi-Depot Delivery ({name}) — t = {t:.0f} s")
        return dots + trails + cust + [ttl]

    anim = FuncAnimation(fig, update, frames=n_frames, blit=False)
    anim.save(os.path.join(OUT_DIR, "animation.gif"), writer="pillow", fps=fps)
    plt.close(fig)
    print(f"  Saved: animation.gif ({n_frames} frames, {fps} fps, "
          f"{t_end:.0f} s of flight)")


# ─── Main ─────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    print("=" * 66)
    print("S030  Multi-Depot Delivery Simulation")
    print("=" * 66)
    print(f"  Fleet/depot {FLEET_SIZE.tolist()}, inventory {INVENTORY.tolist()} kg,"
          f" fleet capacity F*Q {(FLEET_SIZE * Q_MAX).tolist()} kg")
    print(f"  Total demand {DEMANDS.sum():.2f} kg")
    res, hist, km_iter = run_simulation()
    nd = res["Nearest-Depot"]
    for s in STRATS:
        r = res[s]
        moved = [c + 1 for c in range(N_CUSTOMERS) if r["part"][c] != nd["part"][c]]
        print(f"\n--- {s} ---")
        if s == "Balanced K-Means":
            print(f"  K-Means iterations: {km_iter}")
        if s == "AO":
            print(f"  AO iterations: {len(hist)}, objective "
                  + " -> ".join(f"{h:.1f}" for h in hist))
            print(f"  Candidate partitions rejected as infeasible: "
                  f"{INFEASIBLE['partitions']} (depot-level reasons: fleet "
                  f"{INFEASIBLE['fleet']}, inventory {INFEASIBLE['inventory']},"
                  f" range {INFEASIBLE['range']})")
        print(f"  Feasible (fleet, payload, range, inventory): {r['feasible']}")
        print(f"  Customers assigned differently from Nearest-Depot: "
              f"{moved if moved else 'none'}")
        print(f"  Deliveries/depot {r['n'].tolist()} (std {r['bal_std']:.2f}, "
              f"penalty {r['penalty']:.1f})")
        print(f"  Sorties/depot {r['sorties']} vs fleet {FLEET_SIZE.tolist()} "
              f"(packing {r['how']}); cargo/depot "
              f"{[round(float(x), 2) for x in r['cargo']]} kg vs "
              f"inventory {INVENTORY.tolist()}")
        print(f"  Max sortie load {r['max_load']:.2f} kg, max sortie length "
              f"(home return) {r['max_len']:.1f} m")
        print(f"  Distance home-return {r['dist_home']:.1f} m, open-route "
              f"{r['dist']:.1f} m (saving {r['saving']:.1f} m)")
        print(f"  Objective (home dist + penalty) {r['objective']:.1f}; "
              f"makespan {r['makespan']:.1f} s")
    ao = res["AO"]
    print("\nPER-DRONE SUMMARY (AO):")
    for d in ao["drones"]:
        print(f"  {d['name']:<7} origin D{d['origin']+1} return D{d['ret']+1} "
              f"stops {d['stops']} length {d['length']:7.1f} m load "
              f"{d['load']:.2f} kg")
    for s in STRATS[1:]:
        print(f"{s} vs Nearest-Depot distance (open return): "
              f"{(nd['dist'] - res[s]['dist']) / nd['dist'] * 100:+.1f}% saving")
    print("\nGenerating plots...")
    plot_route_map(nd, "Nearest-Depot", "route_map_nearest_depot.png")
    plot_route_map(res["Balanced K-Means"], "Balanced K-Means",
                   "route_map_kmeans.png")
    plot_route_map(ao, "Alternating Optimisation", "route_map_ao.png")
    plot_strategy_comparison(res)
    plot_ao_convergence(hist)
    plot_per_drone_summary(ao, "AO")
    save_animation(ao, "AO")
