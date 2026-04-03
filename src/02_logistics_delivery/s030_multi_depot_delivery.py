"""
S030 Multi-Depot Delivery
=========================
4 depots, 10 drones, 20 customers.
Compares three partition strategies:
  1. Nearest-Depot
  2. K-Means (depot-seeded)
  3. Alternating Optimisation (AO) with 2-opt + boundary reassignment

Open-route return: each drone returns to the nearest depot after its last delivery.
"""

import os
import numpy as np
from scipy.spatial.distance import cdist
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.lines import Line2D
from matplotlib.animation import FuncAnimation

# ─── Output directory ────────────────────────────────────────────────────────
OUT_DIR = "outputs/02_logistics_delivery/s030_multi_depot_delivery"
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
EPS_AO      = 1.0    # m convergence threshold
MAX_AO_ITER = 50

DEPOT_COLORS  = ["#e6194b", "#3cb44b", "#4363d8", "#f58231"]
DEPOT_LABELS  = ["D1 SW", "D2 NW", "D3 SE", "D4 NE"]

DEPOTS      = np.array([[150., 150.], [150., 1050.], [1050., 150.], [1050., 1050.]])
FLEET_SIZE  = np.array([3, 2, 3, 2])
INVENTORY   = np.array([25., 18., 25., 20.])

rng       = np.random.default_rng(7)
CUSTOMERS = rng.uniform(100, 1100, (N_CUSTOMERS, 2))
DEMANDS   = rng.uniform(0.3, 1.8, N_CUSTOMERS)

ALL_POS = np.vstack([DEPOTS, CUSTOMERS])   # shape (D+N, 2)
DIST_M  = cdist(ALL_POS, ALL_POS)          # full distance matrix

# ─── Helpers ──────────────────────────────────────────────────────────────────

def route_dist(route):
    """Total Euclidean distance of a node-index sequence."""
    return sum(DIST_M[route[i], route[i+1]] for i in range(len(route) - 1))


def two_opt(route):
    """2-opt improvement; route includes depot at start and end."""
    route = list(route)
    improved = True
    while improved:
        improved = False
        for i in range(1, len(route) - 2):
            for j in range(i + 1, len(route) - 1):
                gain = (DIST_M[route[i-1], route[i]]
                        + DIST_M[route[j],   route[j+1]]
                        - DIST_M[route[i-1], route[j]]
                        - DIST_M[route[i],   route[j+1]])
                if gain > 1e-6:
                    route[i:j+1] = route[i:j+1][::-1]
                    improved = True
    return route


def nearest_neighbour_route(depot_node, cust_nodes):
    """Build a nearest-neighbour route from depot through cust_nodes, back to depot."""
    if not cust_nodes:
        return [depot_node, depot_node]
    remaining = list(cust_nodes)
    route = [depot_node]
    pos_now = depot_node
    while remaining:
        dists = [DIST_M[pos_now, c] for c in remaining]
        nearest = remaining[int(np.argmin(dists))]
        route.append(nearest)
        pos_now = nearest
        remaining.remove(nearest)
    route.append(depot_node)
    return route


def build_routes_for_depot(depot_idx, cust_idxs, n_drones):
    """
    Build capacity/range-feasible sub-routes for one depot using
    nearest-neighbour then 2-opt.  Returns list of route lists.
    """
    D = N_DEPOTS
    depot_node = depot_idx
    if not cust_idxs:
        return []

    sub_routes = []
    remaining = list(cust_idxs)  # global node indices (D .. D+N-1)

    while remaining:
        route = [depot_node]
        load = 0.0
        dist = 0.0
        pos_now = depot_node

        while True:
            best_c   = None
            best_leg = np.inf
            for c in remaining:
                leg        = DIST_M[pos_now, c]
                ret_leg    = DIST_M[c, depot_node]
                demand_c   = DEMANDS[c - D]
                if (load + demand_c <= Q_MAX
                        and dist + leg + ret_leg <= R_MAX):
                    if leg < best_leg:
                        best_leg = leg
                        best_c   = c
            if best_c is None:
                break
            route.append(best_c)
            load += DEMANDS[best_c - D]
            dist += DIST_M[pos_now, best_c]
            pos_now = best_c
            remaining.remove(best_c)

        route.append(depot_node)
        sub_routes.append(two_opt(route))

    return sub_routes


def total_cost(partition, lam=LAMBDA):
    """Compute total distance + balance penalty for a given partition."""
    total = 0.0
    for m in range(N_DEPOTS):
        cust_idxs = [c + N_DEPOTS for c in range(N_CUSTOMERS) if partition[c] == m]
        routes = build_routes_for_depot(m, cust_idxs, FLEET_SIZE[m])
        for r in routes:
            total += route_dist(r)
    n_per = np.bincount(partition, minlength=N_DEPOTS).astype(float)
    n_bar = N_CUSTOMERS / N_DEPOTS
    penalty = lam * np.sum((n_per - n_bar) ** 2)
    return total + penalty


def all_routes(partition):
    routes_all = []
    for m in range(N_DEPOTS):
        cust_idxs = [c + N_DEPOTS for c in range(N_CUSTOMERS) if partition[c] == m]
        routes_all.append(build_routes_for_depot(m, cust_idxs, FLEET_SIZE[m]))
    return routes_all

# ─── Strategy 1: Nearest-Depot ────────────────────────────────────────────────

def nearest_depot_partition():
    dists = cdist(CUSTOMERS, DEPOTS)
    return np.argmin(dists, axis=1)

# ─── Strategy 2: K-Means (depot-seeded) ──────────────────────────────────────

def kmeans_partition(max_iter=100):
    """K-means with depot positions as initial centroids."""
    centroids = DEPOTS.copy().astype(float)
    partition = np.zeros(N_CUSTOMERS, dtype=int)
    for _ in range(max_iter):
        dists = cdist(CUSTOMERS, centroids)
        new_part = np.argmin(dists, axis=1)
        if np.all(new_part == partition):
            break
        partition = new_part
        for m in range(N_DEPOTS):
            members = CUSTOMERS[partition == m]
            if len(members) > 0:
                # Centroid attracted 50/50 toward depot and cluster mean
                centroids[m] = 0.5 * DEPOTS[m] + 0.5 * members.mean(axis=0)
    return partition

# ─── Strategy 3: Alternating Optimisation ─────────────────────────────────────

def alternating_optimisation():
    partition = nearest_depot_partition().copy()
    prev_cost = np.inf
    history   = []

    for iteration in range(MAX_AO_ITER):
        c_val = total_cost(partition)
        history.append(c_val)
        if abs(prev_cost - c_val) < EPS_AO:
            break
        prev_cost = c_val

        # Boundary reassignment
        for c in range(N_CUSTOMERS):
            m_cur  = partition[c]
            best_m = m_cur
            best_c = c_val
            for m2 in range(N_DEPOTS):
                if m2 == m_cur:
                    continue
                partition[c] = m2
                c2 = total_cost(partition)
                if c2 < best_c:
                    best_c = c2
                    best_m = m2
            partition[c] = best_m
            c_val = best_c

    return partition, history

# ─── Open-route return ────────────────────────────────────────────────────────

def open_route_return(routes_by_depot):
    """
    For each sub-route, replace the closing depot node with the
    nearest depot, and record the saving vs forced home-base return.
    Returns (updated_routes_by_depot, total_saving_m).
    """
    avail = np.array(FLEET_SIZE, dtype=float)   # parking slots proxy
    total_saving = 0.0
    updated = []
    for m, depot_routes in enumerate(routes_by_depot):
        depot_updated = []
        for route in depot_routes:
            if len(route) < 2:
                depot_updated.append(route)
                continue
            last_cust = route[-2]   # last delivery node (before home depot)
            home_dist = DIST_M[last_cust, m]
            # Find nearest depot with remaining capacity
            d_to_depots = DIST_M[last_cust, :N_DEPOTS].copy()
            d_to_depots[avail <= 0] = np.inf
            best_depot = int(np.argmin(d_to_depots))
            saving = home_dist - d_to_depots[best_depot]
            total_saving += max(saving, 0.0)
            avail[best_depot] -= 1
            new_route = route[:-1] + [best_depot]
            depot_updated.append(new_route)
        updated.append(depot_updated)
    return updated, total_saving

# ─── Compute metrics ─────────────────────────────────────────────────────────

def fleet_total_distance(routes_by_depot):
    return sum(route_dist(r) for depot_routes in routes_by_depot for r in depot_routes)


def deliveries_per_depot(partition):
    return np.bincount(partition, minlength=N_DEPOTS)


def per_drone_summary(routes_by_depot, partition):
    rows = []
    D = N_DEPOTS
    for m, depot_routes in enumerate(routes_by_depot):
        for k, route in enumerate(depot_routes):
            rd  = route_dist(route)
            load = sum(DEMANDS[n - D] for n in route if n >= D)
            ret_depot = route[-1]
            rows.append({
                "drone":       f"D{m+1}-#{k+1}",
                "origin":      m,
                "return":      ret_depot,
                "n_stops":     len([n for n in route if n >= D]),
                "route_m":     rd,
                "payload_kg":  load,
                "range_used":  rd,
            })
    return rows

# ─── Plotting ─────────────────────────────────────────────────────────────────

def plot_route_map(partition, routes_by_depot, title, filename):
    fig, ax = plt.subplots(figsize=(9, 9))
    ax.set_xlim(0, 1200)
    ax.set_ylim(0, 1200)
    ax.set_aspect("equal")
    ax.set_facecolor("#f8f8f8")
    ax.grid(True, linestyle="--", alpha=0.4)

    # Customer markers
    for c in range(N_CUSTOMERS):
        m = partition[c]
        ax.scatter(*CUSTOMERS[c], color=DEPOT_COLORS[m], s=60, zorder=3,
                   edgecolors="k", linewidths=0.5)
        ax.text(CUSTOMERS[c, 0] + 10, CUSTOMERS[c, 1] + 10,
                str(c + 1), fontsize=6, color="dimgray")

    # Routes
    for m, depot_routes in enumerate(routes_by_depot):
        for route in depot_routes:
            xs = [ALL_POS[n, 0] for n in route]
            ys = [ALL_POS[n, 1] for n in route]
            # Dashed line if open-route return (last node != origin depot)
            ls = "--" if route[-1] != m else "-"
            ax.plot(xs, ys, color=DEPOT_COLORS[m], lw=1.3, ls=ls, alpha=0.75)
            # Arrow on first leg
            if len(route) >= 2:
                dx = ALL_POS[route[1], 0] - ALL_POS[route[0], 0]
                dy = ALL_POS[route[1], 1] - ALL_POS[route[0], 1]
                ax.annotate("", xy=(ALL_POS[route[1], 0], ALL_POS[route[1], 1]),
                            xytext=(ALL_POS[route[0], 0], ALL_POS[route[0], 1]),
                            arrowprops=dict(arrowstyle="->", color=DEPOT_COLORS[m],
                                            lw=0.8))

    # Depot markers
    for m in range(N_DEPOTS):
        ax.scatter(*DEPOTS[m], marker="^", s=250, color=DEPOT_COLORS[m],
                   zorder=5, edgecolors="k", linewidths=1.2)
        ax.text(DEPOTS[m, 0] + 20, DEPOTS[m, 1] + 20, DEPOT_LABELS[m],
                fontsize=9, fontweight="bold", color=DEPOT_COLORS[m])

    # Legend
    handles = [mpatches.Patch(color=DEPOT_COLORS[m], label=DEPOT_LABELS[m])
               for m in range(N_DEPOTS)]
    handles += [Line2D([0], [0], color="gray", ls="-",  label="Home return"),
                Line2D([0], [0], color="gray", ls="--", label="Open return")]
    ax.legend(handles=handles, loc="lower right", fontsize=8)
    ax.set_title(title, fontsize=13, fontweight="bold")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    plt.tight_layout()
    fpath = os.path.join(OUT_DIR, filename)
    plt.savefig(fpath, dpi=120)
    plt.close()
    print(f"  Saved: {fpath}")


def plot_strategy_comparison(distances, balances):
    """Bar charts: total distance and deliveries-per-depot for each strategy."""
    strategies = ["Nearest\nDepot", "K-Means", "AO"]
    x = np.arange(len(strategies))
    width = 0.55

    fig, axes = plt.subplots(1, 2, figsize=(12, 5))

    # Total distance
    ax = axes[0]
    bars = ax.bar(x, distances, width, color=["#4363d8", "#3cb44b", "#e6194b"],
                  edgecolor="k", linewidth=0.8)
    ax.set_xticks(x)
    ax.set_xticklabels(strategies)
    ax.set_ylabel("Total fleet distance (m)")
    ax.set_title("Total Fleet Flight Distance by Strategy")
    for bar, val in zip(bars, distances):
        ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 20,
                f"{val:.0f} m", ha="center", va="bottom", fontsize=9)
    ax.set_ylim(0, max(distances) * 1.15)
    ax.grid(axis="y", linestyle="--", alpha=0.5)

    # Deliveries per depot (stacked bars)
    ax2 = axes[1]
    balances_arr = np.array(balances)   # shape (3, 4)
    bottom = np.zeros(len(strategies))
    for m in range(N_DEPOTS):
        ax2.bar(x, balances_arr[:, m], width, bottom=bottom,
                label=DEPOT_LABELS[m], color=DEPOT_COLORS[m],
                edgecolor="k", linewidth=0.5)
        bottom += balances_arr[:, m]
    ax2.axhline(N_CUSTOMERS, color="k", ls="--", lw=0.8, label="Total (20)")
    ax2.set_xticks(x)
    ax2.set_xticklabels(strategies)
    ax2.set_ylabel("Number of deliveries")
    ax2.set_title("Deliveries per Depot by Strategy")
    ax2.legend(fontsize=8, loc="lower right")
    ax2.set_ylim(0, N_CUSTOMERS * 1.25)
    ax2.grid(axis="y", linestyle="--", alpha=0.5)

    plt.tight_layout()
    fpath = os.path.join(OUT_DIR, "strategy_comparison.png")
    plt.savefig(fpath, dpi=120)
    plt.close()
    print(f"  Saved: {fpath}")


def plot_ao_convergence(history):
    fig, ax = plt.subplots(figsize=(7, 4))
    ax.plot(range(1, len(history) + 1), history, "o-", color="#e6194b",
            lw=2, ms=5)
    ax.set_xlabel("AO Iteration")
    ax.set_ylabel("Objective (dist + balance penalty, m)")
    ax.set_title("Alternating Optimisation Convergence")
    ax.grid(True, linestyle="--", alpha=0.5)
    plt.tight_layout()
    fpath = os.path.join(OUT_DIR, "ao_convergence.png")
    plt.savefig(fpath, dpi=120)
    plt.close()
    print(f"  Saved: {fpath}")


def plot_per_drone_summary(summary_rows, strategy_name, filename):
    labels   = [r["drone"]   for r in summary_rows]
    ranges   = [r["range_used"] for r in summary_rows]
    payloads = [r["payload_kg"] for r in summary_rows]
    colors   = [DEPOT_COLORS[r["origin"]] for r in summary_rows]

    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    x = np.arange(len(labels))
    width = 0.6

    ax = axes[0]
    bars = ax.bar(x, ranges, width, color=colors, edgecolor="k", linewidth=0.6)
    ax.axhline(R_MAX, color="red", ls="--", lw=1, label=f"R_max={R_MAX:.0f} m")
    ax.set_xticks(x); ax.set_xticklabels(labels, rotation=45, ha="right", fontsize=8)
    ax.set_ylabel("Range used (m)")
    ax.set_title(f"{strategy_name}: Range Used per Drone")
    ax.legend(fontsize=8)
    ax.grid(axis="y", linestyle="--", alpha=0.5)

    ax2 = axes[1]
    ax2.bar(x, payloads, width, color=colors, edgecolor="k", linewidth=0.6)
    ax2.axhline(Q_MAX, color="red", ls="--", lw=1, label=f"Q_max={Q_MAX:.1f} kg")
    ax2.set_xticks(x); ax2.set_xticklabels(labels, rotation=45, ha="right", fontsize=8)
    ax2.set_ylabel("Payload (kg)")
    ax2.set_title(f"{strategy_name}: Payload per Sortie")
    ax2.legend(fontsize=8)
    ax2.grid(axis="y", linestyle="--", alpha=0.5)

    # Depot colour legend
    handles = [mpatches.Patch(color=DEPOT_COLORS[m], label=DEPOT_LABELS[m])
               for m in range(N_DEPOTS)]
    axes[0].legend(handles=handles + [Line2D([0],[0], color="red", ls="--",
                   label=f"R_max")], fontsize=8)

    plt.tight_layout()
    fpath = os.path.join(OUT_DIR, filename)
    plt.savefig(fpath, dpi=120)
    plt.close()
    print(f"  Saved: {fpath}")

# ─── Animation ────────────────────────────────────────────────────────────────

def _build_drone_trajectories(routes_by_depot):
    """
    Convert routes (node-index sequences) into per-drone (x, y) position arrays
    sampled at uniform time steps (FPS=20, speed=V_DRONE m/s).

    Returns:
        trajectories : list of np.ndarray shape (T_drone, 2) — one per drone
        depot_origin : list of int — origin depot index per drone
        cust_delivery_frame : dict mapping customer node index -> (drone_idx, frame_idx)
    """
    FPS = 20
    dt  = 1.0 / FPS                 # seconds per frame

    trajectories    = []
    depot_origin    = []
    cust_del_frame  = {}            # cust_node -> (drone_idx, global frame when delivered)

    drone_idx = 0
    for m, depot_routes in enumerate(routes_by_depot):
        for route in depot_routes:
            # Build list of (x, y) waypoints for this drone
            waypoints = [ALL_POS[n] for n in route]

            # Simulate movement at V_DRONE m/s
            positions = []
            delivery_events = []   # (node, frame_at_delivery) in local frames

            # Service time at each customer stop
            for wi in range(len(waypoints) - 1):
                p0 = waypoints[wi]
                p1 = waypoints[wi + 1]
                seg_dist = np.linalg.norm(p1 - p0)
                n_fly    = max(1, int(round(seg_dist / (V_DRONE * dt))))
                for f in range(n_fly):
                    alpha = f / n_fly
                    positions.append(p0 + alpha * (p1 - p0))

                # Arrived at waypoint wi+1
                arr_frame_local = len(positions)
                node_at_p1 = route[wi + 1]
                if node_at_p1 >= N_DEPOTS:          # it's a customer
                    svc_frames = max(1, int(round(SVC_TIME * FPS)))
                    for _ in range(svc_frames):
                        positions.append(p1.copy())
                    delivery_events.append((node_at_p1, arr_frame_local))
                else:
                    positions.append(p1.copy())     # depot arrival

            traj = np.array(positions)
            trajectories.append(traj)
            depot_origin.append(m)

            for node, local_f in delivery_events:
                cust_del_frame[node] = (drone_idx, local_f)

            drone_idx += 1

    return trajectories, depot_origin, cust_del_frame


def save_animation(routes_by_depot, partition, gif_path, fps=20):
    """
    Build and save a 2-D top-down animated GIF showing all drones flying
    the AO routes simultaneously.

    Parameters
    ----------
    routes_by_depot : list[list[list[int]]]  — output of all_routes() / open_route_return()
    partition       : np.ndarray of shape (N_CUSTOMERS,)  — depot assignment
    gif_path        : str  — full path for the output GIF
    fps             : int  — frames per second (default 20)
    """
    TRAIL_LEN = 20

    print("  Building drone trajectories for animation…")
    trajectories, depot_origin, cust_del_frame = _build_drone_trajectories(routes_by_depot)

    # Pad all trajectories to the same length (drones wait at last position)
    T_max = max(len(t) for t in trajectories)
    padded = []
    for t in trajectories:
        if len(t) < T_max:
            pad = np.tile(t[-1], (T_max - len(t), 1))
            t = np.vstack([t, pad])
        padded.append(t)
    trajectories = padded

    n_drones = len(trajectories)

    # Pre-compute per-customer delivery frame (global)
    delivered_at = {}   # cust_node -> global frame
    for node, (di, lf) in cust_del_frame.items():
        delivered_at[node] = lf    # local frame == global frame since all start at 0

    # ── Set up figure ──────────────────────────────────────────────────────
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_xlim(0, 1200)
    ax.set_ylim(0, 1200)
    ax.set_aspect("equal")
    ax.set_facecolor("#1a1a2e")
    ax.grid(True, linestyle="--", alpha=0.2, color="white")
    ax.tick_params(colors="white")
    for spine in ax.spines.values():
        spine.set_edgecolor("white")
    ax.set_xlabel("x (m)", color="white")
    ax.set_ylabel("y (m)", color="white")

    # ── Static: depot triangles ────────────────────────────────────────────
    for m in range(N_DEPOTS):
        ax.scatter(*DEPOTS[m], marker="^", s=300,
                   color=DEPOT_COLORS[m], zorder=6,
                   edgecolors="white", linewidths=1.2)
        ax.text(DEPOTS[m, 0] + 25, DEPOTS[m, 1] + 25,
                DEPOT_LABELS[m], color=DEPOT_COLORS[m],
                fontsize=8, fontweight="bold", zorder=7)

    # ── Static: customer circles (will be updated each frame) ─────────────
    cust_scatters = []
    for c in range(N_CUSTOMERS):
        sc = ax.scatter(*CUSTOMERS[c], s=80, color="#cccccc",
                        edgecolors="white", linewidths=0.6, zorder=4)
        ax.text(CUSTOMERS[c, 0] + 12, CUSTOMERS[c, 1] + 12,
                str(c + 1), color="white", fontsize=5, zorder=5)
        cust_scatters.append(sc)

    # ── Dynamic: drone dots ────────────────────────────────────────────────
    drone_dots = []
    for di in range(n_drones):
        m = depot_origin[di]
        dot, = ax.plot([], [], "o", color=DEPOT_COLORS[m],
                       ms=6, zorder=8,
                       markeredgecolor="white", markeredgewidth=0.5)
        drone_dots.append(dot)

    # ── Dynamic: trail lines ───────────────────────────────────────────────
    trail_lines = []
    for di in range(n_drones):
        m = depot_origin[di]
        line, = ax.plot([], [], "-", color=DEPOT_COLORS[m],
                        lw=1.0, alpha=0.5, zorder=3)
        trail_lines.append(line)

    # ── Title / frame counter ──────────────────────────────────────────────
    title_text = ax.set_title("S030 Multi-Depot Delivery (AO) — frame 0",
                              color="white", fontsize=11, fontweight="bold")

    # ── Legend ─────────────────────────────────────────────────────────────
    legend_handles = []
    for m in range(N_DEPOTS):
        legend_handles.append(
            mpatches.Patch(color=DEPOT_COLORS[m], label=DEPOT_LABELS[m]))
    legend_handles.append(
        plt.scatter([], [], s=80, color="#cccccc",
                    edgecolors="white", linewidths=0.6, label="Customer (undelivered)"))
    legend_handles.append(
        plt.scatter([], [], s=80, color="#00cc44",
                    edgecolors="white", linewidths=0.6, label="Customer (delivered)"))
    ax.legend(handles=legend_handles, loc="lower right",
              fontsize=7, facecolor="#1a1a2e", labelcolor="white",
              edgecolor="white")

    fig.patch.set_facecolor("#1a1a2e")

    # ── Init function ──────────────────────────────────────────────────────
    def init():
        for dot in drone_dots:
            dot.set_data([], [])
        for line in trail_lines:
            line.set_data([], [])
        for sc in cust_scatters:
            sc.set_facecolor("#cccccc")
        return drone_dots + trail_lines + cust_scatters + [title_text]

    # ── Update function ────────────────────────────────────────────────────
    def update(frame):
        artists = []

        # Update customer colours
        for c in range(N_CUSTOMERS):
            node = c + N_DEPOTS
            if node in delivered_at and frame >= delivered_at[node]:
                cust_scatters[c].set_facecolor("#00cc44")
            else:
                cust_scatters[c].set_facecolor("#cccccc")
            artists.append(cust_scatters[c])

        # Update drones
        for di in range(n_drones):
            traj = trajectories[di]
            f = min(frame, len(traj) - 1)
            x, y = traj[f]
            drone_dots[di].set_data([x], [y])
            artists.append(drone_dots[di])

            # Trail
            start = max(0, f - TRAIL_LEN)
            tx = traj[start:f+1, 0]
            ty = traj[start:f+1, 1]
            trail_lines[di].set_data(tx, ty)
            artists.append(trail_lines[di])

        title_text.set_text(
            f"S030 Multi-Depot Delivery (AO) — t = {frame / fps:.1f} s")
        artists.append(title_text)
        return artists

    # ── Build and save ─────────────────────────────────────────────────────
    print(f"  Rendering {T_max} frames at {fps} fps…")
    anim = FuncAnimation(fig, update, frames=T_max,
                         init_func=init, blit=True, interval=1000 // fps)

    os.makedirs(os.path.dirname(gif_path), exist_ok=True)
    anim.save(gif_path, writer="pillow", fps=fps)
    plt.close(fig)
    print(f"  Saved: {gif_path}")


# ─── Main ─────────────────────────────────────────────────────────────────────

def main():
    print("=" * 60)
    print("S030  Multi-Depot Delivery Simulation")
    print("=" * 60)
    print(f"  Depots    : {N_DEPOTS}  |  Customers : {N_CUSTOMERS}")
    print(f"  Drones    : {N_DRONES}  |  Fleet/depot: {FLEET_SIZE.tolist()}")
    print(f"  Payload   : {Q_MAX} kg  |  Range     : {R_MAX} m")
    print(f"  Inventory : {INVENTORY.tolist()} kg")
    print()

    # ── Strategy 1: Nearest-Depot ──────────────────────────────────────────
    print("─ Strategy 1: Nearest-Depot Partition ─")
    part_nd  = nearest_depot_partition()
    rts_nd   = all_routes(part_nd)
    rts_nd, saving_nd = open_route_return(rts_nd)
    dist_nd  = fleet_total_distance(rts_nd)
    bal_nd   = deliveries_per_depot(part_nd)
    summary_nd = per_drone_summary(rts_nd, part_nd)
    print(f"  Total distance  : {dist_nd:.1f} m")
    print(f"  Deliveries/depot: {bal_nd.tolist()}")
    print(f"  Open-route saving: {saving_nd:.1f} m")

    # ── Strategy 2: K-Means ───────────────────────────────────────────────
    print("\n─ Strategy 2: K-Means Partition ─")
    part_km  = kmeans_partition()
    rts_km   = all_routes(part_km)
    rts_km, saving_km = open_route_return(rts_km)
    dist_km  = fleet_total_distance(rts_km)
    bal_km   = deliveries_per_depot(part_km)
    summary_km = per_drone_summary(rts_km, part_km)
    print(f"  Total distance  : {dist_km:.1f} m")
    print(f"  Deliveries/depot: {bal_km.tolist()}")
    print(f"  Open-route saving: {saving_km:.1f} m")

    # ── Strategy 3: Alternating Optimisation ──────────────────────────────
    print("\n─ Strategy 3: Alternating Optimisation ─")
    part_ao, ao_history = alternating_optimisation()
    rts_ao   = all_routes(part_ao)
    rts_ao, saving_ao = open_route_return(rts_ao)
    dist_ao  = fleet_total_distance(rts_ao)
    bal_ao   = deliveries_per_depot(part_ao)
    summary_ao = per_drone_summary(rts_ao, part_ao)
    print(f"  AO iterations   : {len(ao_history)}")
    print(f"  Total distance  : {dist_ao:.1f} m")
    print(f"  Deliveries/depot: {bal_ao.tolist()}")
    print(f"  Open-route saving: {saving_ao:.1f} m")

    # ── Summary table ─────────────────────────────────────────────────────
    print()
    print("=" * 60)
    print("STRATEGY COMPARISON")
    print(f"  {'Strategy':<22}  {'Total dist (m)':>14}  {'Balance (std)':>13}  {'OR saving (m)':>13}")
    def std_balance(bal):
        return np.std(bal)
    for name, dist, bal, saving in [
        ("Nearest-Depot",  dist_nd, bal_nd, saving_nd),
        ("K-Means",        dist_km, bal_km, saving_km),
        ("AO",             dist_ao, bal_ao, saving_ao),
    ]:
        print(f"  {name:<22}  {dist:>14.1f}  {std_balance(bal):>13.2f}  {saving:>13.1f}")

    print()
    print("PER-DRONE SUMMARY (AO strategy):")
    print(f"  {'Drone':<8} {'Origin':>6} {'Return':>6} {'Stops':>5} {'Route(m)':>9} {'Load(kg)':>8}")
    for r in summary_ao:
        print(f"  {r['drone']:<8} {r['origin']:>6} {r['return']:>6} {r['n_stops']:>5}"
              f" {r['route_m']:>9.1f} {r['payload_kg']:>8.2f}")

    improvement = (dist_nd - dist_ao) / dist_nd * 100 if dist_nd > 0 else 0
    print()
    print(f"AO vs Nearest-Depot improvement: {improvement:.1f}%")
    print(f"Total open-route return saving (AO): {saving_ao:.1f} m")
    print("=" * 60)

    # ── Plots ──────────────────────────────────────────────────────────────
    print("\nGenerating plots…")
    plot_route_map(part_nd, rts_nd,
                   "Nearest-Depot Strategy – Routes & Assignments",
                   "route_map_nearest_depot.png")
    plot_route_map(part_km, rts_km,
                   "K-Means Strategy – Routes & Assignments",
                   "route_map_kmeans.png")
    plot_route_map(part_ao, rts_ao,
                   "Alternating Optimisation – Routes & Assignments",
                   "route_map_ao.png")

    plot_strategy_comparison(
        [dist_nd, dist_km, dist_ao],
        [bal_nd.tolist(), bal_km.tolist(), bal_ao.tolist()]
    )
    plot_ao_convergence(ao_history)
    plot_per_drone_summary(summary_ao, "AO", "per_drone_metrics_ao.png")

    # ── Animation ──────────────────────────────────────────────────────────
    print("\nGenerating animation GIF…")
    gif_path = os.path.join(OUT_DIR, "animation.gif")
    save_animation(rts_ao, part_ao, gif_path, fps=20)

    print("\nAll outputs saved to:", OUT_DIR)


if __name__ == "__main__":
    main()
