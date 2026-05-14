"""
S029 3D Upgrade — Urban Logistics Scheduling
=============================================
Extends the original S029 2D multi-depot VRP by adding altitude dimension to urban
logistics. Different delivery zone types enforce distinct operating altitudes:
  - Residential: 30 m
  - Commercial:  60 m
  - Rooftop:    100 m

Drones must climb/descend to the correct altitude for each zone. Energy cost is
3D Euclidean distance plus an altitude-change penalty (gamma * |dz|). Clarke-Wright
savings are recomputed using this 3D energy metric. Results compare 2D (flat) vs
3D altitude-aware routing in terms of energy cost and total weighted tardiness.
Outputs include a 3D route map, altitude profiles, energy/tardiness comparison
charts, and an animated GIF of all drones flying simultaneously.

Usage:
    conda activate drones
    python src/02_logistics_delivery/3d/s029_3d_urban_logistics.py
"""

import matplotlib
matplotlib.use('Agg')

import sys
import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from itertools import combinations
from matplotlib.animation import FuncAnimation, PillowWriter

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..'))

# ── Parameters ─────────────────────────────────────────────────
N_DEPOTS    = 3
N_CUSTOMERS = 12
N_DRONES    = 6           # 2 per depot
V_DRONE     = 15.0        # m/s cruising speed (3D)
Q_MAX       = 3.0         # kg payload capacity per drone
E_MAX       = 5000.0      # m-equivalent energy budget per sortie
SVC_TIME    = 10.0        # s service time per stop
ALPHA       = 0.7         # tardiness vs energy weight
DT          = 0.5         # s simulation timestep
GAMMA       = 1.5         # altitude penalty factor

DEPOT_ALT   = 30.0        # m — all depots at this altitude
ZONE_ALTS   = {0: 30.0, 1: 60.0, 2: 100.0}   # residential / commercial / rooftop
ZONE_NAMES  = {0: 'Residential (30 m)', 1: 'Commercial (60 m)', 2: 'Rooftop (100 m)'}
ZONE_COLORS = {0: 'cyan', 1: 'orange', 2: 'magenta'}

# Fixed depot ground positions (x, y) — altitude = DEPOT_ALT
DEPOT_XY = np.array([
    [100., 100.],
    [500., 900.],
    [900., 200.],
])

RNG = np.random.default_rng(42)   # fixed seed for reproducibility

OUTPUT_DIR = os.path.normpath(os.path.join(
    os.path.dirname(__file__), '..', '..', '..',
    'outputs', '02_logistics_delivery', '3d', 's029_3d_urban_logistics',
))

# Drone colour palette (6 drones)
DRONE_COLORS = ['#e6194b', '#3cb44b', '#4363d8', '#f58231', '#911eb4', '#42d4f4']


# ── Helpers ─────────────────────────────────────────────────────

def build_network():
    """Build 3D positions for depots and customers."""
    depots = np.column_stack([DEPOT_XY, np.full(N_DEPOTS, DEPOT_ALT)])   # (3, 3)

    cust_xy = RNG.uniform(50, 950, (N_CUSTOMERS, 2))
    zone_types = RNG.integers(0, 3, N_CUSTOMERS)                          # 0/1/2
    cust_z = np.array([ZONE_ALTS[z] for z in zone_types])
    customers = np.column_stack([cust_xy, cust_z])                        # (12, 3)

    # Time windows
    early = RNG.uniform(0, 120, N_CUSTOMERS)
    late  = early + RNG.uniform(60, 180, N_CUSTOMERS)
    demands = RNG.uniform(0.3, 1.5, N_CUSTOMERS)

    return depots, customers, zone_types, early, late, demands


def energy_3d(pi, pj):
    """3D energy cost: Euclidean distance + gamma * altitude change."""
    dist = np.linalg.norm(pj - pi)
    dz   = abs(pj[2] - pi[2])
    return dist + GAMMA * dz


def route_energy(route, depot, customers):
    """Total energy for a route starting and ending at depot."""
    pts = [depot] + [customers[c] for c in route] + [depot]
    return sum(energy_3d(pts[k], pts[k+1]) for k in range(len(pts)-1))


def route_energy_2d(route, depot_2d, customers_2d):
    """2D (flat) route energy: plain Euclidean distance at z=DEPOT_ALT."""
    pts = [depot_2d] + [customers_2d[c] for c in route] + [depot_2d]
    return sum(np.linalg.norm(pts[k+1][:2] - pts[k][:2]) for k in range(len(pts)-1))


# ── Clarke-Wright Savings ────────────────────────────────────────

def clarke_wright_3d(depots, customers, demands):
    """3D Clarke-Wright savings; returns list-of-routes (customer indices) per drone."""
    # Each customer gets nearest depot
    nearest_depot = np.argmin(
        np.array([[np.linalg.norm(customers[c, :2] - depots[d, :2])
                   for d in range(N_DEPOTS)]
                  for c in range(N_CUSTOMERS)]),
        axis=1
    )

    routes = [[c] for c in range(N_CUSTOMERS)]
    # Map route id -> depot index (use id of route list object)
    rdepot_map = {id(r): int(nearest_depot[r[0]]) for r in routes}

    # Compute savings
    savings = []
    for i, j in combinations(range(N_CUSTOMERS), 2):
        di = rdepot_map[id(next(r for r in routes if i in r))]
        dj = rdepot_map[id(next(r for r in routes if j in r))]
        if di != dj:
            continue
        d = di
        s = (energy_3d(depots[d], customers[i]) +
             energy_3d(depots[d], customers[j]) -
             energy_3d(customers[i], customers[j]))
        savings.append((s, i, j))
    savings.sort(reverse=True)

    # Greedy merge
    for s, i, j in savings:
        ri = next((r for r in routes if i in r), None)
        rj = next((r for r in routes if j in r), None)
        if ri is None or rj is None or ri is rj:
            continue
        # Must be at ends of their routes
        if ri[-1] != i and ri[0] != i:
            continue
        if rj[0] != j and rj[-1] != j:
            continue
        merged = ri + rj
        total_demand = sum(demands[c] for c in merged)
        d_idx = rdepot_map[id(ri)]
        total_energy = route_energy(merged, depots[d_idx], customers)
        if total_demand <= Q_MAX and total_energy <= E_MAX:
            rdepot_map[id(ri)] = d_idx  # keep depot for merged route
            ri.extend(rj)
            del rdepot_map[id(rj)]
            routes.remove(rj)

    # Build per-customer depot array (for assign_routes_to_drones compatibility)
    route_depot_list = []
    for r in routes:
        route_depot_list.append(rdepot_map[id(r)])

    return routes, route_depot_list


def clarke_wright_2d(depots, customers, demands):
    """2D (flat) Clarke-Wright — ignores altitude entirely."""
    cust_flat = customers.copy(); cust_flat[:, 2] = DEPOT_ALT
    dep_flat  = depots.copy();    dep_flat[:, 2]  = DEPOT_ALT

    nearest_depot = np.argmin(
        np.array([[np.linalg.norm(cust_flat[c, :2] - dep_flat[d, :2])
                   for d in range(N_DEPOTS)]
                  for c in range(N_CUSTOMERS)]),
        axis=1
    )

    routes = [[c] for c in range(N_CUSTOMERS)]
    # Map route id -> depot index
    rdepot_map = {id(r): int(nearest_depot[r[0]]) for r in routes}

    savings = []
    for i, j in combinations(range(N_CUSTOMERS), 2):
        di = rdepot_map[id(next(r for r in routes if i in r))]
        dj = rdepot_map[id(next(r for r in routes if j in r))]
        if di != dj:
            continue
        d = di
        s = (np.linalg.norm(cust_flat[i, :2] - dep_flat[d, :2]) +
             np.linalg.norm(cust_flat[j, :2] - dep_flat[d, :2]) -
             np.linalg.norm(cust_flat[i, :2] - cust_flat[j, :2]))
        savings.append((s, i, j))
    savings.sort(reverse=True)

    for s, i, j in savings:
        ri = next((r for r in routes if i in r), None)
        rj = next((r for r in routes if j in r), None)
        if ri is None or rj is None or ri is rj:
            continue
        if ri[-1] != i and ri[0] != i:
            continue
        if rj[0] != j and rj[-1] != j:
            continue
        merged = ri + rj
        total_demand = sum(demands[c] for c in merged)
        d_idx = rdepot_map[id(ri)]
        total_energy = route_energy_2d(merged, dep_flat[d_idx], cust_flat)
        if total_demand <= Q_MAX and total_energy <= E_MAX:
            rdepot_map[id(ri)] = d_idx
            ri.extend(rj)
            del rdepot_map[id(rj)]
            routes.remove(rj)

    route_depot_list = [rdepot_map[id(r)] for r in routes]
    return routes, route_depot_list


def assign_routes_to_drones(routes, route_depot):
    """Distribute routes across N_DRONES (2 per depot); balance load."""
    drone_routes = [[] for _ in range(N_DRONES)]  # drone k gets a list of routes
    depot_queue  = {d: [2*d, 2*d+1] for d in range(N_DEPOTS)}  # 2 drones per depot
    depot_load   = {d: [0, 0] for d in range(N_DEPOTS)}         # route count per drone

    for route, depot_idx in zip(routes, route_depot):
        drones = depot_queue[depot_idx]
        loads  = depot_load[depot_idx]
        # Pick the less loaded drone
        pick = 0 if loads[0] <= loads[1] else 1
        drone_idx = drones[pick]
        drone_routes[drone_idx].append(route)
        loads[pick] += len(route)

    # Flatten: each drone executes its routes sequentially
    flat = []
    for k in range(N_DRONES):
        seq = []
        for r in drone_routes[k]:
            seq.extend(r)
        flat.append(seq)
    return flat, [k // 2 for k in range(N_DRONES)]  # drone k belongs to depot k//2


# ── Simulation ──────────────────────────────────────────────────

def simulate_routes_3d(drone_seqs, drone_depot_map, depots, customers, early, late):
    """
    Fly each drone along its assigned sequence in 3D.
    Returns per-drone history (list of 3D positions) and delivery_times dict.
    """
    history        = {k: [] for k in range(N_DRONES)}
    delivery_times = {}

    for k, seq in enumerate(drone_seqs):
        depot_idx = drone_depot_map[k]
        pos = depots[depot_idx].copy()
        t   = 0.0
        history[k].append(pos.copy())

        for c in seq:
            target = customers[c].copy()
            leg_3d = np.linalg.norm(target - pos)
            if leg_3d < 1e-6:
                continue
            leg_time   = leg_3d / V_DRONE
            arrive     = t + leg_time
            start_svc  = max(arrive, early[c])
            delivery_times[c] = start_svc

            # Log 3D trajectory for this leg
            n_steps = max(int(leg_time / DT), 1)
            for step in range(n_steps + 1):
                frac = min(step * DT / leg_time, 1.0)
                history[k].append(pos + frac * (target - pos))

            pos = target.copy()
            t   = start_svc + SVC_TIME

        # Return to depot
        ret = depots[depot_idx].copy()
        ret_dist = np.linalg.norm(ret - pos)
        if ret_dist > 1e-6:
            n_steps = max(int(ret_dist / V_DRONE / DT), 1)
            for step in range(n_steps + 1):
                frac = min(step * DT / (ret_dist / V_DRONE), 1.0)
                history[k].append(pos + frac * (ret - pos))
        history[k].append(ret.copy())

    # Convert to arrays
    history = {k: np.array(v) for k, v in history.items()}
    return history, delivery_times


def compute_metrics(drone_seqs, drone_depot_map, depots, customers,
                    early, late, demands, delivery_times):
    """Compute total energy, weighted tardiness, and per-drone energy."""
    total_energy = 0.0
    per_drone_energy = []

    for k, seq in enumerate(drone_seqs):
        d  = drone_depot_map[k]
        pts = [depots[d]] + [customers[c] for c in seq] + [depots[d]]
        e   = sum(energy_3d(pts[i], pts[i+1]) for i in range(len(pts)-1))
        total_energy += e
        per_drone_energy.append(e)

    twt = sum(max(0.0, delivery_times.get(c, 0.0) - late[c])
              for c in range(N_CUSTOMERS))

    return total_energy, twt, per_drone_energy


# ── Simulation Runner ───────────────────────────────────────────

def run_simulation():
    depots, customers, zone_types, early, late, demands = build_network()

    # --- 3D Clarke-Wright ---
    routes_3d, rdepot_3d = clarke_wright_3d(depots, customers, demands)
    seqs_3d, ddmap_3d    = assign_routes_to_drones(routes_3d, rdepot_3d)
    hist_3d, dtimes_3d   = simulate_routes_3d(seqs_3d, ddmap_3d, depots, customers, early, late)
    energy_3d_val, twt_3d, drone_energy_3d = compute_metrics(
        seqs_3d, ddmap_3d, depots, customers, early, late, demands, dtimes_3d)

    # --- 2D Clarke-Wright (flat baseline) ---
    routes_2d, rdepot_2d = clarke_wright_2d(depots, customers, demands)
    seqs_2d, ddmap_2d    = assign_routes_to_drones(routes_2d, rdepot_2d)
    # Simulate 2D routes using flat positions
    cust_flat = customers.copy(); cust_flat[:, 2] = DEPOT_ALT
    dep_flat  = depots.copy();    dep_flat[:, 2]  = DEPOT_ALT
    hist_2d, dtimes_2d   = simulate_routes_3d(seqs_2d, ddmap_2d, dep_flat, cust_flat, early, late)
    energy_2d_val, twt_2d, drone_energy_2d = compute_metrics(
        seqs_2d, ddmap_2d, depots, customers, early, late, demands, dtimes_2d)

    return (depots, customers, zone_types, early, late, demands,
            seqs_3d, ddmap_3d, hist_3d, energy_3d_val, twt_3d, drone_energy_3d,
            seqs_2d, ddmap_2d, hist_2d, energy_2d_val, twt_2d, drone_energy_2d)


# ── Plots ───────────────────────────────────────────────────────

def plot_3d_routes(depots, customers, zone_types, seqs_3d, ddmap_3d, hist_3d, out_dir):
    """3D route map with depots, zone-coloured customers, and drone trails."""
    fig = plt.figure(figsize=(14, 10))
    ax  = fig.add_subplot(111, projection='3d')

    # Depots — black squares
    for d in range(N_DEPOTS):
        ax.scatter(depots[d, 0], depots[d, 1], depots[d, 2],
                   color='black', marker='s', s=120, zorder=6)
        ax.text(depots[d, 0], depots[d, 1], depots[d, 2] + 3,
                f'D{d+1}', fontsize=9, color='black', fontweight='bold')

    # Customers — coloured by zone
    for c in range(N_CUSTOMERS):
        zt = zone_types[c]
        ax.scatter(customers[c, 0], customers[c, 1], customers[c, 2],
                   color=ZONE_COLORS[zt], marker='o', s=80, edgecolors='black',
                   linewidth=0.6, zorder=5)
        ax.text(customers[c, 0] + 8, customers[c, 1], customers[c, 2],
                str(c+1), fontsize=7, color='dimgray')

    # Drone trajectories
    for k in range(N_DRONES):
        if len(hist_3d[k]) < 2:
            continue
        traj = hist_3d[k]
        ax.plot(traj[:, 0], traj[:, 1], traj[:, 2],
                color=DRONE_COLORS[k], linewidth=1.4, alpha=0.85, label=f'Drone {k+1}')

    # Zone altitude planes (semi-transparent)
    xg, yg = np.meshgrid([50, 950], [50, 950])
    for alt, col, alpha in [(30, 'cyan', 0.06), (60, 'orange', 0.06), (100, 'magenta', 0.06)]:
        ax.plot_surface(xg, yg, np.full_like(xg, alt, dtype=float),
                        color=col, alpha=alpha)

    # Legend for zone types
    from matplotlib.lines import Line2D
    legend_elements = [
        Line2D([0], [0], marker='s', color='w', markerfacecolor='black', markersize=10, label='Depot'),
        Line2D([0], [0], marker='o', color='w', markerfacecolor='cyan',    markersize=8,  label='Residential 30 m'),
        Line2D([0], [0], marker='o', color='w', markerfacecolor='orange',  markersize=8,  label='Commercial 60 m'),
        Line2D([0], [0], marker='o', color='w', markerfacecolor='magenta', markersize=8,  label='Rooftop 100 m'),
    ]
    for k in range(N_DRONES):
        legend_elements.append(
            Line2D([0], [0], color=DRONE_COLORS[k], linewidth=2, label=f'Drone {k+1}'))

    ax.legend(handles=legend_elements, loc='upper left', fontsize=7, ncol=2)
    ax.set_xlabel('X (m)', fontsize=9)
    ax.set_ylabel('Y (m)', fontsize=9)
    ax.set_zlabel('Z (m)', fontsize=9)
    ax.set_xlim(0, 1000); ax.set_ylim(0, 1000); ax.set_zlim(0, 120)
    ax.set_title('S029 3D Urban Logistics — 3D Route Map', fontsize=12, fontweight='bold')

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'routes_3d.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_altitude_profiles(hist_3d, seqs_3d, customers, zone_types, out_dir):
    """Per-drone altitude vs time profile showing zone-altitude step changes."""
    fig, axes = plt.subplots(2, 3, figsize=(15, 8), sharex=False)
    axes = axes.flatten()

    for k in range(N_DRONES):
        ax = axes[k]
        traj = hist_3d[k]
        if len(traj) < 2:
            ax.set_visible(False)
            continue

        t_arr = np.arange(len(traj)) * DT
        ax.plot(t_arr, traj[:, 2], color=DRONE_COLORS[k], linewidth=1.8, label=f'Drone {k+1}')

        # Mark zone altitudes
        for alt, col, lbl in [(30, 'cyan', 'Residential'),
                               (60, 'orange', 'Commercial'),
                               (100, 'magenta', 'Rooftop')]:
            ax.axhline(alt, color=col, linestyle='--', linewidth=0.8, alpha=0.7, label=lbl)

        # Mark customer stops
        cust_t = 0.0
        pos = None
        for c in seqs_3d[k]:
            # approximate time of arrival
            pass  # trajectory already encodes altitude

        ax.set_ylim(0, 115)
        ax.set_ylabel('Altitude (m)', fontsize=8)
        ax.set_xlabel('Time (s)', fontsize=8)
        ax.set_title(f'Drone {k+1}  ({len(seqs_3d[k])} stops)', fontsize=9)
        ax.legend(fontsize=6, loc='upper right')
        ax.grid(True, alpha=0.3)

    fig.suptitle('S029 3D Urban Logistics — Altitude Profiles', fontsize=13, fontweight='bold')
    plt.tight_layout()

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'altitude_profiles.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_energy_comparison(energy_2d, energy_3d_v, drone_energy_2d, drone_energy_3d,
                           twt_2d, twt_3d, out_dir):
    """Side-by-side bar charts: total energy and tardiness for 2D vs 3D routing."""
    fig, axes = plt.subplots(1, 3, figsize=(15, 5))

    # Total energy
    ax = axes[0]
    bars = ax.bar(['2D Flat\nRouting', '3D Altitude-Aware\nRouting'],
                  [energy_2d, energy_3d_v],
                  color=['steelblue', 'coral'], edgecolor='black', linewidth=0.8)
    for bar, val in zip(bars, [energy_2d, energy_3d_v]):
        ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 20,
                f'{val:.0f} m', ha='center', va='bottom', fontsize=10)
    ax.set_ylabel('Total Energy Cost (m-equiv)', fontsize=9)
    ax.set_title('Total Fleet Energy Cost', fontsize=11, fontweight='bold')
    ax.grid(True, axis='y', alpha=0.3)
    ax.set_ylim(0, max(energy_2d, energy_3d_v) * 1.2)

    # Tardiness
    ax = axes[1]
    bars = ax.bar(['2D Flat\nRouting', '3D Altitude-Aware\nRouting'],
                  [twt_2d, twt_3d],
                  color=['steelblue', 'coral'], edgecolor='black', linewidth=0.8)
    for bar, val in zip(bars, [twt_2d, twt_3d]):
        ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.5,
                f'{val:.1f} s', ha='center', va='bottom', fontsize=10)
    ax.set_ylabel('Total Weighted Tardiness (s)', fontsize=9)
    ax.set_title('Total Weighted Tardiness', fontsize=11, fontweight='bold')
    ax.grid(True, axis='y', alpha=0.3)
    ax.set_ylim(0, max(twt_2d, twt_3d, 1.0) * 1.3)

    # Per-drone energy
    ax = axes[2]
    x  = np.arange(N_DRONES)
    w  = 0.35
    ax.bar(x - w/2, drone_energy_2d, w, color='steelblue', edgecolor='black',
           linewidth=0.6, label='2D Flat')
    ax.bar(x + w/2, drone_energy_3d, w, color='coral', edgecolor='black',
           linewidth=0.6, label='3D Alt-Aware')
    ax.set_xticks(x)
    ax.set_xticklabels([f'D{k+1}' for k in range(N_DRONES)], fontsize=8)
    ax.set_ylabel('Energy Cost (m-equiv)', fontsize=9)
    ax.set_title('Per-Drone Energy Comparison', fontsize=11, fontweight='bold')
    ax.legend(fontsize=8)
    ax.grid(True, axis='y', alpha=0.3)

    fig.suptitle('S029 3D Urban Logistics — 2D vs 3D Routing Comparison', fontsize=13,
                 fontweight='bold')
    plt.tight_layout()

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'energy_comparison.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def save_animation(depots, customers, zone_types, hist_3d, seqs_3d, out_dir):
    """Animate all drones flying simultaneously in 3D."""
    # Determine common time grid using step-skipping for animation speed
    step_skip = 4
    max_frames = max(len(hist_3d[k]) for k in range(N_DRONES))
    n_frames = (max_frames + step_skip - 1) // step_skip

    fig = plt.figure(figsize=(12, 9))
    ax  = fig.add_subplot(111, projection='3d')

    # Static: depots
    for d in range(N_DEPOTS):
        ax.scatter(depots[d, 0], depots[d, 1], depots[d, 2],
                   color='black', marker='s', s=120, zorder=6)

    # Static: customers (zone-coloured)
    for c in range(N_CUSTOMERS):
        zt = zone_types[c]
        ax.scatter(customers[c, 0], customers[c, 1], customers[c, 2],
                   color=ZONE_COLORS[zt], marker='o', s=60,
                   edgecolors='black', linewidth=0.5, alpha=0.7, zorder=5)

    ax.set_xlim(0, 1000); ax.set_ylim(0, 1000); ax.set_zlim(0, 120)
    ax.set_xlabel('X (m)', fontsize=8)
    ax.set_ylabel('Y (m)', fontsize=8)
    ax.set_zlabel('Z (m)', fontsize=8)

    # Precompute subsampled trajectories
    trajs = {}
    for k in range(N_DRONES):
        t = hist_3d[k]
        trajs[k] = t[::step_skip] if len(t) > 0 else t

    # Create dynamic objects per drone
    trails = {}
    dots   = {}
    for k in range(N_DRONES):
        trail, = ax.plot([], [], [], color=DRONE_COLORS[k], linewidth=1.2, alpha=0.7)
        dot    = ax.scatter([], [], [], color=DRONE_COLORS[k], s=80, marker='^', zorder=8)
        trails[k] = trail
        dots[k]   = dot

    title_obj = ax.set_title('', fontsize=10)

    def update(frame):
        artists = []
        for k in range(N_DRONES):
            t = trajs[k]
            if len(t) == 0:
                continue
            idx = min(frame, len(t) - 1)
            # Trail: all points up to idx
            trail_pts = t[:idx+1]
            trails[k].set_data(trail_pts[:, 0], trail_pts[:, 1])
            trails[k].set_3d_properties(trail_pts[:, 2])
            # Current dot
            dots[k]._offsets3d = (
                [float(t[idx, 0])],
                [float(t[idx, 1])],
                [float(t[idx, 2])],
            )
            artists.extend([trails[k], dots[k]])
        t_sim = frame * step_skip * DT
        title_obj.set_text(f'S029 3D Urban Logistics — t = {t_sim:.1f} s')
        artists.append(title_obj)
        return artists

    ani = FuncAnimation(fig, update, frames=n_frames, interval=50, blit=False)

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer=PillowWriter(fps=20), dpi=100)
    plt.close()
    print(f'Saved: {path}')


# ── Main ────────────────────────────────────────────────────────

if __name__ == '__main__':
    print('Running S029 3D Urban Logistics Scheduling...')
    data = run_simulation()
    (depots, customers, zone_types, early, late, demands,
     seqs_3d, ddmap_3d, hist_3d, energy_3d_val, twt_3d, drone_energy_3d,
     seqs_2d, ddmap_2d, hist_2d, energy_2d_val, twt_2d, drone_energy_2d) = data

    # Print key metrics
    print('\n--- Key Metrics ---')
    print(f'Customers served: {N_CUSTOMERS}')
    print(f'Zone distribution: '
          f'Residential={sum(zone_types==0)}, '
          f'Commercial={sum(zone_types==1)}, '
          f'Rooftop={sum(zone_types==2)}')
    print(f'\n2D Flat Routing:')
    print(f'  Total energy cost : {energy_2d_val:.1f} m-equiv')
    print(f'  Total wtd tardiness: {twt_2d:.1f} s')
    print(f'\n3D Altitude-Aware Routing:')
    print(f'  Total energy cost : {energy_3d_val:.1f} m-equiv')
    print(f'  Total wtd tardiness: {twt_3d:.1f} s')
    print(f'\nAltitude penalty overhead: '
          f'{(energy_3d_val - energy_2d_val) / energy_2d_val * 100:.1f}%')
    print(f'Tardiness change: '
          f'{twt_3d - twt_2d:+.1f} s  '
          f'({(twt_3d - twt_2d) / max(twt_2d, 1e-6) * 100:+.1f}%)')

    for k in range(N_DRONES):
        print(f'  Drone {k+1}: {len(seqs_3d[k])} stops, '
              f'energy = {drone_energy_3d[k]:.1f} m-equiv')

    out_dir = OUTPUT_DIR
    print(f'\nSaving outputs to: {out_dir}')

    plot_3d_routes(depots, customers, zone_types, seqs_3d, ddmap_3d, hist_3d, out_dir)
    plot_altitude_profiles(hist_3d, seqs_3d, customers, zone_types, out_dir)
    plot_energy_comparison(energy_2d_val, energy_3d_val, drone_energy_2d, drone_energy_3d,
                           twt_2d, twt_3d, out_dir)
    print('Saving animation (all drones simultaneously)...')
    save_animation(depots, customers, zone_types, hist_3d, seqs_3d, out_dir)
    print('\nDone. All outputs saved to:', out_dir)
