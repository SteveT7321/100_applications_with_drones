"""
S029 Urban Logistics Scheduling
Multi-depot multi-drone VRP with time windows.
Compares: Greedy Nearest-Neighbour vs Clarke-Wright Savings.
"""

import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from scipy.spatial.distance import cdist
from itertools import combinations

# ─── Output directory ────────────────────────────────────────────────────────
OUT_DIR = os.path.join(
    os.path.dirname(__file__),
    "..", "..",
    "outputs", "02_logistics_delivery", "s029_urban_logistics_scheduling"
)
OUT_DIR = os.path.normpath(OUT_DIR)
os.makedirs(OUT_DIR, exist_ok=True)

# ─── Constants ────────────────────────────────────────────────────────────────
N_DEPOTS    = 3
N_CUSTOMERS = 12
N_DRONES    = 6          # 2 per depot
V_DRONE     = 15.0       # m/s
Q_MAX       = 3.0        # kg capacity per drone per sortie
R_MAX       = 4000.0     # m max range per sortie
SVC_TIME    = 10.0       # s service time per customer
ALPHA       = 0.7        # tardiness vs distance weight in objective

DEPOT_POS = np.array([
    [100., 100.],
    [500., 900.],
    [900., 200.],
])

rng = np.random.default_rng(42)
CUST_POS = rng.uniform(50, 950, (N_CUSTOMERS, 2))
EARLY    = rng.uniform(0, 120, N_CUSTOMERS)
LATE     = EARLY + rng.uniform(60, 180, N_CUSTOMERS)
DEMAND   = rng.uniform(0.3, 1.5, N_CUSTOMERS)
WEIGHT   = rng.uniform(0.5, 2.0, N_CUSTOMERS)   # importance weight for TWT

# ─── Utility helpers ──────────────────────────────────────────────────────────

def dist(a, b):
    return np.linalg.norm(np.asarray(a) - np.asarray(b))


def route_length(route, depot_pos, cust_pos):
    """Total flight distance: depot → c0 → c1 → … → depot."""
    if not route:
        return 0.0
    pts = [depot_pos] + [cust_pos[c] for c in route] + [depot_pos]
    return sum(dist(pts[i], pts[i + 1]) for i in range(len(pts) - 1))


def simulate_route(route, depot_pos, cust_pos, early, late, weight, v, svc_time):
    """
    Execute one drone route and return:
      delivery_times : dict {customer_idx: service_start_time}
      total_dist     : float
      tardiness      : float (weighted)
      timeline       : list of (t_depart_leg, t_arrive_leg, customer_idx_or_None)
    """
    pos = depot_pos.copy()
    t = 0.0
    delivery_times = {}
    tardiness = 0.0
    total_dist = 0.0
    timeline = []

    for c in route:
        d = dist(pos, cust_pos[c])
        total_dist += d
        travel_time = d / v
        arrive = t + travel_time
        start_svc = max(arrive, early[c])
        tardi = max(0.0, start_svc - late[c]) * weight[c]
        tardiness += tardi
        delivery_times[c] = start_svc
        timeline.append((t, arrive, start_svc, c))
        pos = cust_pos[c].copy()
        t = start_svc + svc_time

    # return to depot
    d_back = dist(pos, depot_pos)
    total_dist += d_back
    return delivery_times, total_dist, tardiness, timeline


def evaluate_solution(drone_routes, depot_of_drone, cust_pos, early, late, weight, v, svc_time):
    """
    Evaluate a full set of drone routes.
    Returns aggregated metrics dict and per-drone detail list.
    """
    total_dist = 0.0
    total_twt  = 0.0
    all_delivery = {}
    details = []

    for k, route in enumerate(drone_routes):
        dep_pos = DEPOT_POS[depot_of_drone[k]]
        dt_map, rd, twt, timeline = simulate_route(
            route, dep_pos, cust_pos, early, late, weight, v, svc_time
        )
        all_delivery.update(dt_map)
        total_dist += rd
        total_twt  += twt
        details.append(dict(route=route, depot=depot_of_drone[k],
                            dist=rd, twt=twt, timeline=timeline))

    obj = ALPHA * total_twt + (1 - ALPHA) * total_dist
    return dict(total_dist=total_dist, total_twt=total_twt, objective=obj,
                n_routes=sum(1 for r in drone_routes if r),
                deliveries=all_delivery), details


# ─── Strategy 1: Greedy Nearest-Neighbour ────────────────────────────────────

def greedy_nn(depot_pos, cust_pos, demands, q_max, r_max, n_drones):
    """
    Assign each customer to nearest depot, then build nearest-neighbour tours
    per drone (round-robin within depot).
    Returns drone_routes list (length n_drones) and depot_of_drone list.
    """
    depot_dist = cdist(cust_pos, depot_pos)
    nearest_depot = np.argmin(depot_dist, axis=1)

    # Bucket customers per depot
    depot_customers = {d: [] for d in range(len(depot_pos))}
    for c in range(len(cust_pos)):
        depot_customers[nearest_depot[c]].append(c)

    drone_routes = [[] for _ in range(n_drones)]
    depot_of_drone = [k // 2 for k in range(n_drones)]

    for d in range(len(depot_pos)):
        custs = depot_customers[d]
        drones_here = [k for k in range(n_drones) if depot_of_drone[k] == d]
        if not custs or not drones_here:
            continue

        # Build one NN tour per drone (capacity-constrained round-robin load)
        unvisited = set(custs)
        drone_idx = 0
        while unvisited:
            k = drones_here[drone_idx % len(drones_here)]
            # Start from depot, greedily pick nearest unvisited within capacity/range
            pos = depot_pos[d].copy()
            load = 0.0
            rdist = 0.0
            route = drone_routes[k]
            advanced = False
            while unvisited:
                # Filter feasible (capacity and range: must still be able to return)
                candidates = []
                for c in unvisited:
                    new_load = load + demands[c]
                    if new_load > q_max:
                        continue
                    leg = dist(pos, cust_pos[c])
                    back = dist(cust_pos[c], depot_pos[d])
                    if rdist + leg + back > r_max:
                        continue
                    candidates.append((leg, c))
                if not candidates:
                    break
                candidates.sort()
                _, best = candidates[0]
                leg = dist(pos, cust_pos[best])
                rdist += leg
                load += demands[best]
                route.append(best)
                pos = cust_pos[best].copy()
                unvisited.discard(best)
                advanced = True
            drone_idx += 1
            if not advanced:
                # Unfeasible customer — force assign to avoid infinite loop
                if unvisited:
                    forced = next(iter(unvisited))
                    drone_routes[drones_here[drone_idx % len(drones_here)]].append(forced)
                    unvisited.discard(forced)
                    drone_idx += 1

    return drone_routes, depot_of_drone


# ─── Strategy 2: Clarke-Wright Savings ───────────────────────────────────────

def clarke_wright(depot_pos, cust_pos, demands, q_max, r_max, n_drones):
    """
    Clarke-Wright savings algorithm (multi-depot variant).
    Returns drone_routes and depot_of_drone.
    """
    depot_dist_mat = cdist(cust_pos, depot_pos)
    nearest_depot  = np.argmin(depot_dist_mat, axis=1)

    # Initial solution: one route per customer
    # route_id -> (list_of_customers, depot_index)
    routes = {c: [c] for c in range(len(cust_pos))}
    route_depot = {c: int(nearest_depot[c]) for c in range(len(cust_pos))}

    # Interior end tracking: can only merge at route ends
    def route_ends(r):
        return r[0], r[-1]

    # Build savings list (only within same depot)
    savings = []
    cust_list = list(range(len(cust_pos)))
    for i, j in combinations(cust_list, 2):
        if nearest_depot[i] != nearest_depot[j]:
            continue
        d = int(nearest_depot[i])
        s = depot_dist_mat[i, d] + depot_dist_mat[j, d] - dist(cust_pos[i], cust_pos[j])
        savings.append((s, i, j))
    savings.sort(reverse=True, key=lambda x: x[0])

    # Merge
    for s, i, j in savings:
        # Find routes containing i and j
        ri_key = next((k for k, v in routes.items() if i in v), None)
        rj_key = next((k for k, v in routes.items() if j in v), None)
        if ri_key is None or rj_key is None or ri_key == rj_key:
            continue
        if route_depot[ri_key] != route_depot[rj_key]:
            continue

        ri = routes[ri_key]
        rj = routes[rj_key]
        d = route_depot[ri_key]

        # i must be at an end of ri, j must be at an end of rj
        ri_ends = route_ends(ri)
        rj_ends = route_ends(rj)
        if i not in ri_ends or j not in rj_ends:
            continue

        # Form merged route (orient so i is last in ri, j is first in rj)
        if ri[-1] != i:
            ri = ri[::-1]
        if rj[0] != j:
            rj = rj[::-1]
        merged = ri + rj

        # Feasibility
        total_demand = sum(demands[c] for c in merged)
        depot_pos_d  = depot_pos[d]
        total_dist_m = route_length(merged, depot_pos_d, cust_pos)
        if total_demand > q_max or total_dist_m > r_max:
            continue

        # Merge
        routes[ri_key] = merged
        del routes[rj_key]
        route_depot[ri_key] = d
        if rj_key in route_depot:
            del route_depot[rj_key]

    # Assign merged routes to drones (round-robin per depot)
    depot_of_drone = [k // 2 for k in range(n_drones)]
    drone_routes   = [[] for _ in range(n_drones)]

    depot_drone_idx = {d: [k for k in range(n_drones) if depot_of_drone[k] == d]
                       for d in range(len(depot_pos))}
    depot_assigned  = {d: 0 for d in range(len(depot_pos))}

    for rk, route in routes.items():
        d = route_depot[rk]
        drones_here = depot_drone_idx[d]
        if not drones_here:
            continue
        drone_k = drones_here[depot_assigned[d] % len(drones_here)]
        drone_routes[drone_k].extend(route)
        depot_assigned[d] += 1

    return drone_routes, depot_of_drone


# ─── Plotting helpers ─────────────────────────────────────────────────────────

COLORS = plt.cm.tab10(np.linspace(0, 1, N_DRONES))

def plot_route_map(drone_routes, depot_of_drone, cust_pos, depot_pos,
                   title, filename):
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_xlim(0, 1000)
    ax.set_ylim(0, 1000)
    ax.set_aspect("equal")
    ax.set_title(title, fontsize=13)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")

    # Customer circles
    ax.scatter(cust_pos[:, 0], cust_pos[:, 1],
               s=80, c="steelblue", zorder=4, label="Customer")
    for c in range(len(cust_pos)):
        ax.text(cust_pos[c, 0] + 12, cust_pos[c, 1] + 12,
                f"C{c}", fontsize=7, color="navy")

    # Depot triangles
    for d, dp in enumerate(depot_pos):
        ax.plot(dp[0], dp[1], marker="^", markersize=14,
                color="crimson", zorder=5)
        ax.text(dp[0] + 15, dp[1] + 15, f"D{d}", fontsize=9,
                color="crimson", fontweight="bold")

    # Routes
    patch_handles = []
    for k, route in enumerate(drone_routes):
        if not route:
            continue
        col = COLORS[k]
        dep = depot_pos[depot_of_drone[k]]
        pts = [dep] + [cust_pos[c] for c in route] + [dep]
        xs  = [p[0] for p in pts]
        ys  = [p[1] for p in pts]
        ax.plot(xs, ys, "-o", color=col, lw=1.5, markersize=5, zorder=3)
        patch_handles.append(mpatches.Patch(color=col, label=f"Drone {k} (depot {depot_of_drone[k]})"))

    ax.legend(handles=patch_handles, loc="lower right", fontsize=8)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    path = os.path.join(OUT_DIR, filename)
    fig.savefig(path, dpi=120)
    plt.close(fig)
    print(f"  Saved: {path}")


def plot_gantt(details, title, filename):
    """Gantt chart: each drone's timeline."""
    fig, ax = plt.subplots(figsize=(12, 5))
    ax.set_title(title, fontsize=13)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Drone")

    max_t = 0
    for k, d in enumerate(details):
        y = k
        t_cur = 0.0
        dep_pos = DEPOT_POS[d["depot"]]
        pos = dep_pos.copy()
        for (t_dep, t_arr, t_svc, c) in d["timeline"]:
            # Travel bar
            ax.barh(y, t_arr - t_dep, left=t_dep, height=0.4,
                    color="steelblue", edgecolor="white", linewidth=0.5)
            # Wait bar (if waiting for time window)
            if t_svc > t_arr:
                ax.barh(y, t_svc - t_arr, left=t_arr, height=0.4,
                        color="gold", edgecolor="white", linewidth=0.5)
            # Service bar
            ax.barh(y, SVC_TIME, left=t_svc, height=0.4,
                    color="forestgreen", edgecolor="white", linewidth=0.5)
            # Time-window marker
            ax.plot([EARLY[c], EARLY[c]], [y - 0.3, y + 0.3], "k--", lw=0.8)
            ax.plot([LATE[c],  LATE[c]],  [y - 0.3, y + 0.3], "r--", lw=0.8)
            max_t = max(max_t, t_svc + SVC_TIME)

    ax.set_yticks(range(len(details)))
    ax.set_yticklabels([f"Drone {k}" for k in range(len(details))])
    ax.set_xlim(0, max_t * 1.05 + 20)
    ax.grid(True, axis="x", alpha=0.3)

    legend_patches = [
        mpatches.Patch(color="steelblue",   label="Travel"),
        mpatches.Patch(color="gold",        label="Wait (time window)"),
        mpatches.Patch(color="forestgreen", label="Service"),
    ]
    ax.legend(handles=legend_patches, loc="upper right", fontsize=8)
    fig.tight_layout()
    path = os.path.join(OUT_DIR, filename)
    fig.savefig(path, dpi=120)
    plt.close(fig)
    print(f"  Saved: {path}")


def plot_comparison(results_dict, filename):
    """Bar chart comparing metrics across strategies."""
    strategies = list(results_dict.keys())
    twt_vals   = [results_dict[s]["total_twt"]  for s in strategies]
    dist_vals  = [results_dict[s]["total_dist"] for s in strategies]
    obj_vals   = [results_dict[s]["objective"]  for s in strategies]

    x = np.arange(len(strategies))
    width = 0.25

    fig, axes = plt.subplots(1, 3, figsize=(13, 5))
    fig.suptitle("Strategy Comparison — S029 Urban Logistics Scheduling", fontsize=13)

    for ax, vals, ylabel, title, color in zip(
        axes,
        [twt_vals, dist_vals, obj_vals],
        ["Total Weighted Tardiness (s)", "Total Flight Distance (m)", "Composite Objective"],
        ["Weighted Tardiness", "Total Distance", "Objective (α=0.7)"],
        ["salmon", "steelblue", "mediumpurple"]
    ):
        bars = ax.bar(x, vals, color=color, edgecolor="black", linewidth=0.8)
        ax.set_xticks(x)
        ax.set_xticklabels(strategies, fontsize=10)
        ax.set_ylabel(ylabel, fontsize=9)
        ax.set_title(title, fontsize=11)
        ax.grid(True, axis="y", alpha=0.3)
        for bar, v in zip(bars, vals):
            ax.text(bar.get_x() + bar.get_width() / 2,
                    bar.get_height() * 1.01, f"{v:.1f}",
                    ha="center", va="bottom", fontsize=9)

    fig.tight_layout()
    path = os.path.join(OUT_DIR, filename)
    fig.savefig(path, dpi=120)
    plt.close(fig)
    print(f"  Saved: {path}")


def plot_delivery_table(metrics, strategy_name, filename):
    """Table of per-customer delivery time, window, tardiness."""
    deliveries = metrics["deliveries"]
    rows = []
    for c in range(N_CUSTOMERS):
        svc_t = deliveries.get(c, float("nan"))
        tardi = max(0.0, svc_t - LATE[c]) * WEIGHT[c] if not np.isnan(svc_t) else float("nan")
        rows.append([f"C{c}", f"{EARLY[c]:.1f}", f"{LATE[c]:.1f}",
                     f"{svc_t:.1f}", f"{tardi:.2f}"])

    fig, ax = plt.subplots(figsize=(9, 4.5))
    ax.axis("off")
    ax.set_title(f"Per-Customer Delivery Details — {strategy_name}", fontsize=12)
    cols = ["Customer", "Earliest (s)", "Latest (s)", "Delivery (s)", "Weighted Tardiness"]
    table = ax.table(cellText=rows, colLabels=cols, loc="center", cellLoc="center")
    table.auto_set_font_size(False)
    table.set_fontsize(9)
    table.scale(1.1, 1.4)
    fig.tight_layout()
    path = os.path.join(OUT_DIR, filename)
    fig.savefig(path, dpi=120)
    plt.close(fig)
    print(f"  Saved: {path}")


# ─── Main ─────────────────────────────────────────────────────────────────────

def main():
    print("=" * 60)
    print("S029  Urban Logistics Scheduling")
    print("=" * 60)

    # ── Strategy 1: Greedy NN ──────────────────────────────────────
    print("\n[1/2] Greedy Nearest-Neighbour ...")
    nn_routes, nn_depot_of_drone = greedy_nn(
        DEPOT_POS, CUST_POS, DEMAND, Q_MAX, R_MAX, N_DRONES
    )
    nn_metrics, nn_details = evaluate_solution(
        nn_routes, nn_depot_of_drone,
        CUST_POS, EARLY, LATE, WEIGHT, V_DRONE, SVC_TIME
    )

    # ── Strategy 2: Clarke-Wright Savings ─────────────────────────
    print("[2/2] Clarke-Wright Savings ...")
    cw_routes, cw_depot_of_drone = clarke_wright(
        DEPOT_POS, CUST_POS, DEMAND, Q_MAX, R_MAX, N_DRONES
    )
    cw_metrics, cw_details = evaluate_solution(
        cw_routes, cw_depot_of_drone,
        CUST_POS, EARLY, LATE, WEIGHT, V_DRONE, SVC_TIME
    )

    # ── Plots ──────────────────────────────────────────────────────
    print("\nGenerating plots ...")

    plot_route_map(nn_routes, nn_depot_of_drone, CUST_POS, DEPOT_POS,
                   "Route Map — Greedy Nearest-Neighbour",
                   "route_map_greedy_nn.png")

    plot_route_map(cw_routes, cw_depot_of_drone, CUST_POS, DEPOT_POS,
                   "Route Map — Clarke-Wright Savings",
                   "route_map_clarke_wright.png")

    plot_gantt(nn_details,
               "Drone Timeline — Greedy Nearest-Neighbour",
               "gantt_greedy_nn.png")

    plot_gantt(cw_details,
               "Drone Timeline — Clarke-Wright Savings",
               "gantt_clarke_wright.png")

    all_results = {
        "Greedy NN":     nn_metrics,
        "Clarke-Wright": cw_metrics,
    }
    plot_comparison(all_results, "comparison_metrics.png")

    plot_delivery_table(cw_metrics, "Clarke-Wright Savings",
                        "delivery_table_clarke_wright.png")

    # ── Console summary ────────────────────────────────────────────
    print("\n" + "=" * 60)
    print("SIMULATION RESULTS SUMMARY")
    print("=" * 60)
    for name, m in all_results.items():
        print(f"\n  Strategy : {name}")
        print(f"    Active routes        : {m['n_routes']}")
        print(f"    Total flight distance: {m['total_dist']:.1f} m")
        print(f"    Total weighted TWT   : {m['total_twt']:.2f} s")
        print(f"    Composite objective  : {m['objective']:.2f}")
        customers_served = len(m["deliveries"])
        print(f"    Customers served     : {customers_served} / {N_CUSTOMERS}")

    best = min(all_results, key=lambda k: all_results[k]["objective"])
    print(f"\n  Best strategy: {best}")
    dist_improvement = (
        (nn_metrics["total_dist"] - cw_metrics["total_dist"])
        / nn_metrics["total_dist"] * 100
        if nn_metrics["total_dist"] > 0 else 0
    )
    print(f"  Clarke-Wright distance improvement over Greedy NN: "
          f"{dist_improvement:.1f}%")
    print("\n  Per-drone route summary (Clarke-Wright):")
    for k, (route, det) in enumerate(zip(cw_routes, cw_details)):
        print(f"    Drone {k} (depot {cw_depot_of_drone[k]}): "
              f"route {route}  dist={det['dist']:.1f} m  TWT={det['twt']:.2f} s")
    print("\n  Output files saved to:")
    print(f"  {OUT_DIR}")
    print("=" * 60)


if __name__ == "__main__":
    main()
