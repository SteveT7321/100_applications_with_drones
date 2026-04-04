"""
S038 Disaster Relief Drop
=========================
A heterogeneous fleet of 5 drones delivers relief supplies from a forward operating
base (FOB) to 8 disaster sites spread over a 1000x1000 m area. Sites have different
priorities (critical / urgent / standard), demand weights, and soft deadlines after which
delivery value decays exponentially. Three dispatch strategies are compared:
  1. Greedy priority-first with nearest-neighbour packing + 2-opt improvement
  2. Regret-based insertion
  3. Iterated local search (ILS) with 2-opt / Or-opt improvements

Usage:
    conda activate drones
    python src/02_logistics_delivery/s038_disaster_relief_drop.py
"""

import sys, os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.animation as animation
from copy import deepcopy
from dataclasses import dataclass, field
from typing import List, Optional

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

# ── Parameters ─────────────────────────────────────────────────────────────────
K_DRONES     = 5
M_SITES      = 8
T_MAX        = 600.0   # s — mission horizon
TAU_DROP     = 8.0     # s — hover + release at each site
TAU_TURN     = 30.0    # s — battery swap turnaround at FOB
LAMBDA_DECAY = 0.01    # s⁻¹ — value decay rate after soft deadline
MU_PENALTY   = 5.0     # unserved demand penalty multiplier
PI_MAX       = 3       # maximum priority level
DT           = 0.1     # s — simulation timestep

FOB = np.array([500.0, 100.0])  # 2D (x, y)

# Heterogeneous fleet: [payload_kg, range_m, speed_m/s]
FLEET = np.array([
    [5.0, 2000.0, 12.0],   # drone 0 — heavy-lift, slow
    [5.0, 2000.0, 12.0],   # drone 1
    [2.5, 2800.0, 16.0],   # drone 2 — mid-range
    [2.5, 2800.0, 16.0],   # drone 3
    [1.0, 3500.0, 20.0],   # drone 4 — fast scout, light payload
])

# Relief sites: [x, y, demand_kg, priority, soft_deadline_s]
SITES = np.array([
    [150., 800., 3.0, 1, 120.],   # s0 critical
    [820., 750., 2.0, 1, 150.],   # s1 critical
    [300., 600., 1.5, 2, 200.],   # s2 urgent
    [700., 500., 1.0, 2, 240.],   # s3 urgent
    [200., 350., 2.5, 1, 100.],   # s4 critical — nearest critical
    [600., 300., 1.0, 3, 300.],   # s5 standard
    [850., 200., 0.8, 3, 350.],   # s6 standard
    [450., 700., 1.5, 2, 180.],   # s7 urgent
])

DRONE_COLORS = ['tab:blue', 'tab:orange', 'tab:green', 'tab:red', 'tab:purple']
PRIORITY_COLORS = {1: 'red', 2: 'darkorange', 3: 'royalblue'}

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '02_logistics_delivery', 's038_disaster_relief_drop',
)

RNG = np.random.default_rng(0)


# ── Helpers ────────────────────────────────────────────────────────────────────

def delivery_value(priority: int, t_arrive: float, soft_deadline: float) -> float:
    """Priority-weighted time-decayed delivery value (0..1 range)."""
    w = (PI_MAX + 1 - priority) / PI_MAX
    decay = np.exp(-LAMBDA_DECAY * max(0.0, t_arrive - soft_deadline))
    return w * decay


def sortie_range(site_seq: List[int], sites: np.ndarray, fob: np.ndarray) -> float:
    """Total round-trip distance for a sortie visiting sites in given order."""
    pts = [fob] + [sites[i, :2] for i in site_seq] + [fob]
    return sum(np.linalg.norm(np.array(pts[j+1]) - np.array(pts[j]))
               for j in range(len(pts)-1))


def sortie_arrivals(site_seq: List[int], sites: np.ndarray, fob: np.ndarray,
                    depart_time: float, speed: float, tau_drop: float):
    """Compute arrival time at each site in the sortie; return (arrivals, return_time)."""
    t = depart_time
    pos = fob.copy()
    arrivals = []
    for idx in site_seq:
        target = sites[idx, :2]
        t += np.linalg.norm(target - pos) / speed
        arrivals.append(t)
        t += tau_drop
        pos = target.copy()
    return_t = t + np.linalg.norm(fob - pos) / speed
    return arrivals, return_t


def compute_total_value(sorties, sites: np.ndarray, t_max: float):
    """Sum delivery values for all served sites within T_max minus unserved penalty."""
    total = 0.0
    served_ids = set()
    for s in sorties:
        for site_id, t_arr in zip(s['sites'], s['arrivals']):
            if t_arr <= t_max:
                total += delivery_value(
                    int(sites[site_id, 3]), t_arr, sites[site_id, 4]
                ) * sites[site_id, 2]
                served_ids.add(site_id)
    for i in range(len(sites)):
        if i not in served_ids:
            w = (PI_MAX + 1 - int(sites[i, 3])) / PI_MAX
            total -= MU_PENALTY * w * sites[i, 2]
    return total, served_ids


def two_opt_sortie(site_seq: List[int], sites: np.ndarray, fob: np.ndarray) -> List[int]:
    """Apply 2-opt improvement to a single sortie's site ordering."""
    best = site_seq[:]
    improved = True
    while improved:
        improved = False
        for a in range(len(best) - 1):
            for b in range(a + 1, len(best)):
                candidate = best[:a] + best[a:b+1][::-1] + best[b+1:]
                if sortie_range(candidate, sites, fob) < sortie_range(best, sites, fob):
                    best = candidate
                    improved = True
    return best


# ── Strategy 1: Greedy Priority-First ─────────────────────────────────────────

def greedy_dispatch(sites: np.ndarray, fob: np.ndarray):
    """Greedy priority-first dispatch with nearest-neighbour packing + 2-opt."""
    t_available = np.zeros(K_DRONES)
    served = set()
    sorties = []

    def score(site_idx, dk):
        dist = np.linalg.norm(sites[site_idx, :2] - fob)
        priority = int(sites[site_idx, 3])
        w = (PI_MAX + 1 - priority) / PI_MAX
        return w / (dist / FLEET[dk, 2] + 1e-6)

    while True:
        dk = int(np.argmin(t_available))
        if t_available[dk] >= T_MAX:
            break
        unserved = [i for i in range(M_SITES) if i not in served]
        if not unserved:
            break

        ranked = sorted(unserved, key=lambda i: -score(i, dk))
        seed = ranked[0]

        sortie_sites = [seed]
        total_demand = sites[seed, 2]
        pos = sites[seed, :2].copy()

        remaining = [i for i in ranked[1:] if i != seed]
        for candidate in sorted(remaining, key=lambda i: np.linalg.norm(sites[i, :2] - pos)):
            if total_demand + sites[candidate, 2] > FLEET[dk, 0]:
                continue
            trial = sortie_sites + [candidate]
            if sortie_range(trial, sites, fob) > FLEET[dk, 1]:
                continue
            sortie_sites = trial
            total_demand += sites[candidate, 2]
            pos = sites[candidate, :2].copy()

        # 2-opt improve
        if len(sortie_sites) > 2:
            sortie_sites = two_opt_sortie(sortie_sites, sites, fob)

        arrivals, return_t = sortie_arrivals(
            sortie_sites, sites, fob, t_available[dk], FLEET[dk, 2], TAU_DROP)

        sorties.append({
            'drone': dk, 'sites': sortie_sites,
            'depart': t_available[dk], 'arrivals': arrivals, 'return': return_t,
        })
        served.update(sortie_sites)
        t_available[dk] = return_t + TAU_TURN

    return sorties


# ── Strategy 2: Regret-Based Insertion ────────────────────────────────────────

def insertion_cost(site_idx, route, sites, fob, t_start, speed):
    """Compute best insertion cost for site_idx into route."""
    best_delta = float('inf')
    best_pos = 1
    pts = [fob] + [sites[i, :2] for i in route] + [fob]
    base_len = sum(np.linalg.norm(pts[j+1] - pts[j]) for j in range(len(pts)-1))
    for pos in range(1, len(pts)):
        new_route = route[:pos-1] + [site_idx] + route[pos-1:]
        new_len = sortie_range(new_route, sites, fob)
        delta = new_len - base_len
        if delta < best_delta:
            best_delta = delta
            best_pos = pos - 1
    return best_delta, best_pos


def regret_dispatch(sites: np.ndarray, fob: np.ndarray):
    """Regret-based insertion: assign site with highest regret to best drone/position."""
    # Initialise one (empty) sortie per drone
    t_available = np.zeros(K_DRONES)
    # Maintain current routes as lists; we'll rebuild sorties at end
    routes = [[] for _ in range(K_DRONES)]  # site sequences per drone (current sortie)
    demands = [0.0] * K_DRONES
    unassigned = list(range(M_SITES))
    served = set()

    while unassigned:
        regrets = []
        for site_idx in unassigned:
            costs = []
            for dk in range(K_DRONES):
                # Check payload feasibility
                if demands[dk] + sites[site_idx, 2] > FLEET[dk, 0]:
                    costs.append(float('inf'))
                    continue
                trial = routes[dk] + [site_idx]
                if sortie_range(trial, sites, fob) > FLEET[dk, 1]:
                    costs.append(float('inf'))
                    continue
                delta, _ = insertion_cost(site_idx, routes[dk], sites, fob,
                                          t_available[dk], FLEET[dk, 2])
                costs.append(delta)
            costs_sorted = sorted(c for c in costs if c < float('inf'))
            if len(costs_sorted) == 0:
                # No feasible insertion — skip this site
                regret_val = -1
            elif len(costs_sorted) == 1:
                regret_val = costs_sorted[0]  # only one option
            else:
                regret_val = costs_sorted[1] - costs_sorted[0]
            regrets.append((regret_val, site_idx))

        # Pick site with highest regret (ties broken arbitrarily)
        regrets.sort(key=lambda x: -x[0])
        best_regret, chosen_site = regrets[0]

        if best_regret < 0:
            # No feasible assignment; skip
            unassigned.remove(chosen_site)
            continue

        # Find best drone for chosen_site
        best_dk, best_delta = -1, float('inf')
        for dk in range(K_DRONES):
            if demands[dk] + sites[chosen_site, 2] > FLEET[dk, 0]:
                continue
            trial = routes[dk] + [chosen_site]
            if sortie_range(trial, sites, fob) > FLEET[dk, 1]:
                continue
            delta, _ = insertion_cost(chosen_site, routes[dk], sites, fob,
                                      t_available[dk], FLEET[dk, 2])
            if delta < best_delta:
                best_delta = delta
                best_dk = dk

        if best_dk == -1:
            unassigned.remove(chosen_site)
            continue

        routes[best_dk].append(chosen_site)
        demands[best_dk] += sites[chosen_site, 2]
        unassigned.remove(chosen_site)
        served.add(chosen_site)

    # Build sorties from routes (one sortie per drone, 2-opt improved)
    sorties = []
    for dk in range(K_DRONES):
        if not routes[dk]:
            continue
        route = two_opt_sortie(routes[dk], sites, fob)
        arrivals, return_t = sortie_arrivals(
            route, sites, fob, t_available[dk], FLEET[dk, 2], TAU_DROP)
        sorties.append({
            'drone': dk, 'sites': route,
            'depart': t_available[dk], 'arrivals': arrivals, 'return': return_t,
        })

    return sorties


# ── Strategy 3: Iterated Local Search (ILS) ───────────────────────────────────

def ils_dispatch(sites: np.ndarray, fob: np.ndarray, n_restarts: int = 8):
    """ILS: random initial assignment + 2-opt + Or-opt, keep best over restarts."""
    best_sorties = None
    best_value = -float('inf')

    for restart in range(n_restarts):
        # Random assignment: shuffle sites and assign round-robin checking feasibility
        perm = RNG.permutation(M_SITES).tolist()
        t_available = np.zeros(K_DRONES)
        routes = [[] for _ in range(K_DRONES)]
        demands = [0.0] * K_DRONES

        for site_idx in perm:
            # Try drones in random order
            order = RNG.permutation(K_DRONES).tolist()
            for dk in order:
                if demands[dk] + sites[site_idx, 2] > FLEET[dk, 0]:
                    continue
                trial = routes[dk] + [site_idx]
                if sortie_range(trial, sites, fob) > FLEET[dk, 1]:
                    continue
                routes[dk].append(site_idx)
                demands[dk] += sites[site_idx, 2]
                break

        # 2-opt on each route
        for dk in range(K_DRONES):
            if len(routes[dk]) > 2:
                routes[dk] = two_opt_sortie(routes[dk], sites, fob)

        # Or-opt: try relocating each site to a better drone
        improved = True
        iter_count = 0
        while improved and iter_count < 20:
            improved = False
            iter_count += 1
            for dk_src in range(K_DRONES):
                for site_pos, site_idx in enumerate(routes[dk_src][:]):
                    for dk_dst in range(K_DRONES):
                        if dk_dst == dk_src:
                            continue
                        # Check feasibility after removal from src and insertion at dst
                        new_src = [s for s in routes[dk_src] if s != site_idx]
                        demand_src_new = demands[dk_src] - sites[site_idx, 2]
                        demand_dst_new = demands[dk_dst] + sites[site_idx, 2]
                        if demand_dst_new > FLEET[dk_dst, 0]:
                            continue
                        new_dst = routes[dk_dst] + [site_idx]
                        if sortie_range(new_dst, sites, fob) > FLEET[dk_dst, 1]:
                            continue

                        # Compute value change
                        arr_old_src, ret_old_src = sortie_arrivals(
                            routes[dk_src], sites, fob, t_available[dk_src], FLEET[dk_src, 2], TAU_DROP)
                        arr_new_src, ret_new_src = sortie_arrivals(
                            new_src, sites, fob, t_available[dk_src], FLEET[dk_src, 2], TAU_DROP) if new_src else ([], t_available[dk_src])
                        arr_old_dst, ret_old_dst = sortie_arrivals(
                            routes[dk_dst], sites, fob, t_available[dk_dst], FLEET[dk_dst, 2], TAU_DROP) if routes[dk_dst] else ([], t_available[dk_dst])
                        arr_new_dst, ret_new_dst = sortie_arrivals(
                            new_dst, sites, fob, t_available[dk_dst], FLEET[dk_dst, 2], TAU_DROP)

                        # Value of site at old location vs new
                        old_pos_in_src = routes[dk_src].index(site_idx)
                        t_old = arr_old_src[old_pos_in_src]
                        t_new = arr_new_dst[-1]  # appended at end
                        v_old = delivery_value(int(sites[site_idx, 3]), t_old, sites[site_idx, 4]) * sites[site_idx, 2]
                        v_new = delivery_value(int(sites[site_idx, 3]), t_new, sites[site_idx, 4]) * sites[site_idx, 2]

                        if v_new > v_old:
                            routes[dk_src] = new_src
                            routes[dk_dst] = new_dst
                            demands[dk_src] = demand_src_new
                            demands[dk_dst] = demand_dst_new
                            improved = True
                            break
                    if improved:
                        break
                if improved:
                    break

        # Build sorties
        sorties = []
        for dk in range(K_DRONES):
            if not routes[dk]:
                continue
            route = two_opt_sortie(routes[dk], sites, fob)
            arrivals, return_t = sortie_arrivals(
                route, sites, fob, t_available[dk], FLEET[dk, 2], TAU_DROP)
            sorties.append({
                'drone': dk, 'sites': route,
                'depart': t_available[dk], 'arrivals': arrivals, 'return': return_t,
            })

        val, _ = compute_total_value(sorties, sites, T_MAX)
        if val > best_value:
            best_value = val
            best_sorties = deepcopy(sorties)

    return best_sorties


# ── Simulation ──────────────────────────────────────────────────────────────────

def run_simulation():
    """Run all three strategies and return results."""
    sorties_greedy  = greedy_dispatch(SITES, FOB)
    sorties_regret  = regret_dispatch(SITES, FOB)
    sorties_ils     = ils_dispatch(SITES, FOB, n_restarts=10)

    val_g, served_g = compute_total_value(sorties_greedy, SITES, T_MAX)
    val_r, served_r = compute_total_value(sorties_regret, SITES, T_MAX)
    val_i, served_i = compute_total_value(sorties_ils,    SITES, T_MAX)

    return (sorties_greedy, sorties_regret, sorties_ils,
            val_g, val_r, val_i,
            served_g, served_r, served_i)


# ── Build cumulative value timeline ────────────────────────────────────────────

def build_value_timeline(sorties, sites, t_max, dt=1.0):
    """Return (times, cumulative_value) arrays for a strategy."""
    events = []
    for s in sorties:
        for site_id, t_arr in zip(s['sites'], s['arrivals']):
            if t_arr <= t_max:
                v = delivery_value(int(sites[site_id, 3]), t_arr, sites[site_id, 4]) * sites[site_id, 2]
                events.append((t_arr, v))
    events.sort()
    times = [0.0]
    cum   = [0.0]
    running = 0.0
    for t, v in events:
        running += v
        times.append(t)
        cum.append(running)
    times.append(t_max)
    cum.append(running)
    return np.array(times), np.array(cum)


# ── Plots ───────────────────────────────────────────────────────────────────────

def plot_mission_map(sorties_greedy, sorties_regret, sorties_ils, out_dir):
    """2D mission map: FOB, sites by priority, sortie routes for each strategy."""
    fig, axes = plt.subplots(1, 3, figsize=(18, 6))
    strategy_names = ['Greedy Priority-First', 'Regret-Based Insertion', 'ILS']
    all_sorties = [sorties_greedy, sorties_regret, sorties_ils]

    for ax, name, sorties in zip(axes, strategy_names, all_sorties):
        # Sites
        for i, (x, y, dem, pri, sd) in enumerate(SITES):
            col = PRIORITY_COLORS[int(pri)]
            ax.scatter(x, y, c=col, s=120, zorder=5, edgecolors='black', linewidths=0.5)
            ax.annotate(f'S{i}\n{dem}kg', (x, y), textcoords='offset points',
                        xytext=(6, 4), fontsize=7)

        # FOB
        ax.scatter(*FOB, marker='*', s=300, c='black', zorder=6, label='FOB')

        # Routes
        _, served_ids = compute_total_value(sorties, SITES, T_MAX)
        for s in sorties:
            dk = s['drone']
            col = DRONE_COLORS[dk % len(DRONE_COLORS)]
            route = [FOB] + [SITES[i, :2] for i in s['sites']] + [FOB]
            xs = [p[0] for p in route]
            ys = [p[1] for p in route]
            ax.plot(xs, ys, color=col, alpha=0.7, linewidth=1.5,
                    label=f'Drone {dk}')

        ax.set_xlim(-50, 1050); ax.set_ylim(-50, 1050)
        ax.set_title(name, fontsize=11, fontweight='bold')
        ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)

    # Legend patches
    legend_patches = [
        mpatches.Patch(color='red', label='Critical (P1)'),
        mpatches.Patch(color='darkorange', label='Urgent (P2)'),
        mpatches.Patch(color='royalblue', label='Standard (P3)'),
    ]
    axes[0].legend(handles=legend_patches, loc='lower left', fontsize=8)

    fig.suptitle('S038 Disaster Relief Drop — Mission Routes', fontsize=13, fontweight='bold')
    plt.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'mission_map.png')
    plt.savefig(path, dpi=150); plt.close()
    print(f'Saved: {path}')


def plot_gantt(sorties_greedy, sorties_regret, sorties_ils, out_dir):
    """Gantt chart: per-drone timeline for each strategy."""
    fig, axes = plt.subplots(3, 1, figsize=(14, 10))
    strategy_names = ['Greedy', 'Regret-Based', 'ILS']
    all_sorties = [sorties_greedy, sorties_regret, sorties_ils]

    for ax, name, sorties in zip(axes, strategy_names, all_sorties):
        for s in sorties:
            dk = s['drone']
            col = DRONE_COLORS[dk % len(DRONE_COLORS)]
            # Flight bar
            ax.barh(dk, s['return'] - s['depart'], left=s['depart'],
                    color=col, alpha=0.5, edgecolor='gray', height=0.6)
            # Drop markers
            for site_id, t_arr in zip(s['sites'], s['arrivals']):
                pri = int(SITES[site_id, 3])
                pcol = PRIORITY_COLORS[pri]
                ax.scatter(t_arr, dk, color=pcol, s=50, zorder=5)
            # Battery swap bar
            if s['return'] + TAU_TURN <= T_MAX:
                ax.barh(dk, TAU_TURN, left=s['return'],
                        color='gray', alpha=0.3, height=0.6, hatch='//')

        ax.set_xlim(0, T_MAX)
        ax.set_yticks(range(K_DRONES))
        ax.set_yticklabels([f'Drone {k}' for k in range(K_DRONES)])
        ax.set_xlabel('Time (s)')
        ax.set_title(f'{name}', fontsize=10, fontweight='bold')
        ax.axvline(T_MAX, color='black', linestyle='--', linewidth=1.2, alpha=0.6, label='T_max')
        ax.grid(True, axis='x', alpha=0.3)

    # Priority legend
    legend_patches = [
        mpatches.Patch(color='red', label='Critical drop'),
        mpatches.Patch(color='darkorange', label='Urgent drop'),
        mpatches.Patch(color='royalblue', label='Standard drop'),
        mpatches.Patch(color='gray', alpha=0.4, hatch='//', label='Battery swap'),
    ]
    axes[0].legend(handles=legend_patches, loc='upper right', fontsize=8)

    fig.suptitle('S038 Disaster Relief Drop — Gantt Chart', fontsize=13, fontweight='bold')
    plt.tight_layout()
    path = os.path.join(out_dir, 'gantt_chart.png')
    plt.savefig(path, dpi=150); plt.close()
    print(f'Saved: {path}')


def plot_value_timeline(sorties_greedy, sorties_regret, sorties_ils,
                        val_g, val_r, val_i, out_dir):
    """Cumulative delivery value vs time for each strategy."""
    fig, ax = plt.subplots(figsize=(10, 5))

    t_g, v_g = build_value_timeline(sorties_greedy, SITES, T_MAX)
    t_r, v_r = build_value_timeline(sorties_regret, SITES, T_MAX)
    t_i, v_i = build_value_timeline(sorties_ils, SITES, T_MAX)

    ax.step(t_g, v_g, where='post', color='tab:blue',   lw=2, label=f'Greedy    J={val_g:.2f}')
    ax.step(t_r, v_r, where='post', color='tab:orange', lw=2, label=f'Regret    J={val_r:.2f}')
    ax.step(t_i, v_i, where='post', color='tab:green',  lw=2, label=f'ILS       J={val_i:.2f}')

    # Theoretical max (all sites served instantly at t=0)
    theo_max = sum(
        delivery_value(int(SITES[i, 3]), 0.0, SITES[i, 4]) * SITES[i, 2]
        for i in range(M_SITES)
    )
    ax.axhline(theo_max, color='gray', linestyle='--', lw=1.2, label=f'Theoretical max={theo_max:.2f}')

    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Cumulative Delivery Value J')
    ax.set_title('S038 — Cumulative Delivery Value vs Time')
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'value_timeline.png')
    plt.savefig(path, dpi=150); plt.close()
    print(f'Saved: {path}')


def plot_coverage_bar(sorties_greedy, sorties_regret, sorties_ils, out_dir):
    """Coverage bar chart: fraction of demand served per priority tier per strategy."""
    fig, ax = plt.subplots(figsize=(9, 5))

    strategies = ['Greedy', 'Regret', 'ILS']
    all_sorties_list = [sorties_greedy, sorties_regret, sorties_ils]
    priority_labels = ['Critical (P1)', 'Urgent (P2)', 'Standard (P3)']
    pri_colors_list = ['red', 'darkorange', 'royalblue']
    x = np.arange(len(strategies))
    width = 0.25

    for p_idx, (plabel, pcol) in enumerate(zip(priority_labels, pri_colors_list)):
        pri = p_idx + 1
        fracs = []
        for sorties in all_sorties_list:
            _, served_ids = compute_total_value(sorties, SITES, T_MAX)
            total_dem = sum(SITES[i, 2] for i in range(M_SITES) if int(SITES[i, 3]) == pri)
            served_dem = sum(SITES[i, 2] for i in served_ids if int(SITES[i, 3]) == pri)
            fracs.append(served_dem / total_dem if total_dem > 0 else 0.0)
        ax.bar(x + (p_idx - 1) * width, fracs, width, label=plabel,
               color=pcol, alpha=0.75, edgecolor='black')

    ax.set_xticks(x)
    ax.set_xticklabels(strategies)
    ax.set_ylim(0, 1.15)
    ax.set_ylabel('Fraction of Demand Served')
    ax.set_title('S038 — Demand Coverage by Priority Tier')
    ax.legend(fontsize=9)
    ax.axhline(1.0, color='gray', linestyle='--', lw=1)
    ax.grid(True, axis='y', alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'coverage_bar.png')
    plt.savefig(path, dpi=150); plt.close()
    print(f'Saved: {path}')


def plot_lateness_histogram(sorties_greedy, sorties_regret, sorties_ils, out_dir):
    """Arrival lateness histogram grouped by strategy."""
    fig, axes = plt.subplots(1, 3, figsize=(14, 4), sharey=True)
    strategy_names = ['Greedy', 'Regret-Based', 'ILS']
    all_sorties = [sorties_greedy, sorties_regret, sorties_ils]
    colors = ['tab:blue', 'tab:orange', 'tab:green']

    for ax, name, sorties, col in zip(axes, strategy_names, all_sorties, colors):
        latenesses = []
        for s in sorties:
            for site_id, t_arr in zip(s['sites'], s['arrivals']):
                if t_arr <= T_MAX:
                    soft_d = SITES[site_id, 4]
                    latenesses.append(max(0.0, t_arr - soft_d))
        if latenesses:
            ax.hist(latenesses, bins=10, color=col, alpha=0.75, edgecolor='black')
        ax.set_xlabel('Lateness (s)')
        ax.set_ylabel('Count')
        ax.set_title(name, fontsize=10)
        ax.grid(True, alpha=0.3)

    fig.suptitle('S038 — Arrival Lateness (max(0, t_arrive - d_soft))', fontsize=12)
    plt.tight_layout()
    path = os.path.join(out_dir, 'lateness_histogram.png')
    plt.savefig(path, dpi=150); plt.close()
    print(f'Saved: {path}')


def save_animation(sorties_ils, out_dir):
    """Animate drones flying sorties (ILS strategy) — 2D top-down view."""
    # Build per-drone trajectory (t, x, y) and delivery events
    T_ANIM = T_MAX
    t_samples = np.arange(0, T_ANIM + DT, DT)

    drone_traj = {dk: [] for dk in range(K_DRONES)}  # list of (t, x, y) per drone

    for s in sorties_ils:
        dk = s['drone']
        speed = FLEET[dk, 2]
        waypoints = [FOB] + [SITES[i, :2] for i in s['sites']] + [FOB]
        t = s['depart']
        for wp_idx in range(len(waypoints) - 1):
            p0 = waypoints[wp_idx]
            p1 = waypoints[wp_idx + 1]
            dist = np.linalg.norm(p1 - p0)
            t_leg = dist / speed
            n_steps = max(1, int(t_leg / DT))
            for step in range(n_steps + 1):
                frac = step / max(1, n_steps)
                pos = p0 + frac * (p1 - p0)
                drone_traj[dk].append((t + frac * t_leg, pos[0], pos[1]))
            t += t_leg
            if wp_idx < len(waypoints) - 2:
                t += TAU_DROP

    # Delivery events: {site_id: t_arrive}
    delivery_events = {}
    for s in sorties_ils:
        for site_id, t_arr in zip(s['sites'], s['arrivals']):
            if t_arr <= T_MAX:
                delivery_events[site_id] = t_arr

    # Animation frames
    STEP = 4
    frame_times = t_samples[::STEP]

    def get_drone_pos(dk, t):
        traj = drone_traj[dk]
        if not traj:
            return FOB[0], FOB[1]
        times_arr = [pt[0] for pt in traj]
        if t <= times_arr[0]:
            return traj[0][1], traj[0][2]
        if t >= times_arr[-1]:
            return traj[-1][1], traj[-1][2]
        idx = np.searchsorted(times_arr, t) - 1
        t0, x0, y0 = traj[idx]
        t1, x1, y1 = traj[min(idx + 1, len(traj) - 1)]
        frac = (t - t0) / (t1 - t0 + 1e-9)
        return x0 + frac * (x1 - x0), y0 + frac * (y1 - y0)

    fig, ax = plt.subplots(figsize=(7, 7))
    ax.set_xlim(-50, 1050); ax.set_ylim(-50, 1050)
    ax.set_aspect('equal')
    ax.set_title('S038 Disaster Relief Drop — ILS Animation', fontsize=11)
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')

    # Static: FOB
    ax.scatter(*FOB, marker='*', s=300, c='black', zorder=6)
    ax.annotate('FOB', FOB, textcoords='offset points', xytext=(6, 4), fontsize=9)

    # Relief site markers (hollow = unserved)
    site_scatters = []
    for i, (x, y, dem, pri, sd) in enumerate(SITES):
        col = PRIORITY_COLORS[int(pri)]
        sc = ax.scatter(x, y, c='white', edgecolors=col, s=120, linewidths=2, zorder=5)
        site_scatters.append(sc)
        ax.annotate(f'S{i}', (x, y), textcoords='offset points',
                    xytext=(6, 4), fontsize=7)

    # Drone markers
    drone_scatters = []
    for dk in range(K_DRONES):
        sc = ax.scatter(*FOB, c=DRONE_COLORS[dk % len(DRONE_COLORS)], s=80, zorder=7,
                        marker='D')
        drone_scatters.append(sc)

    time_text = ax.text(10, 1020, '', fontsize=9)

    def update(frame_idx):
        t = frame_times[frame_idx]
        time_text.set_text(f't = {t:.1f} s')

        # Update drone positions
        for dk in range(K_DRONES):
            x, y = get_drone_pos(dk, t)
            drone_scatters[dk].set_offsets([[x, y]])

        # Update site fill (delivered = filled)
        for i in range(M_SITES):
            col = PRIORITY_COLORS[int(SITES[i, 3])]
            if i in delivery_events and t >= delivery_events[i]:
                site_scatters[i].set_facecolor(col)
            else:
                site_scatters[i].set_facecolor('white')

        return drone_scatters + site_scatters + [time_text]

    ani = animation.FuncAnimation(
        fig, update, frames=len(frame_times), interval=50, blit=True
    )
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer='pillow', fps=20, dpi=100)
    plt.close()
    print(f'Saved: {path}')


# ── Main ────────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    print('Running S038 Disaster Relief Drop simulation...')
    (sorties_greedy, sorties_regret, sorties_ils,
     val_g, val_r, val_i,
     served_g, served_r, served_i) = run_simulation()

    print(f'\n=== Results ===')
    print(f'Greedy   — served {len(served_g)}/8 sites, J = {val_g:.3f}')
    print(f'Regret   — served {len(served_r)}/8 sites, J = {val_r:.3f}')
    print(f'ILS      — served {len(served_i)}/8 sites, J = {val_i:.3f}')

    # Per-strategy lateness
    for name, sorties in [('Greedy', sorties_greedy), ('Regret', sorties_regret), ('ILS', sorties_ils)]:
        lates = [max(0.0, t_arr - SITES[sid, 4])
                 for s in sorties
                 for sid, t_arr in zip(s['sites'], s['arrivals'])
                 if t_arr <= T_MAX]
        mean_late = np.mean(lates) if lates else 0.0
        print(f'{name:8s} mean lateness = {mean_late:.1f} s')

    # Critical sites coverage
    for name, served_ids in [('Greedy', served_g), ('Regret', served_r), ('ILS', served_i)]:
        n_crit = sum(1 for i in served_ids if int(SITES[i, 3]) == 1)
        n_crit_total = sum(1 for i in range(M_SITES) if int(SITES[i, 3]) == 1)
        print(f'{name:8s} critical sites served = {n_crit}/{n_crit_total}')

    out_dir = os.path.normpath(OUTPUT_DIR)
    plot_mission_map(sorties_greedy, sorties_regret, sorties_ils, out_dir)
    plot_gantt(sorties_greedy, sorties_regret, sorties_ils, out_dir)
    plot_value_timeline(sorties_greedy, sorties_regret, sorties_ils,
                        val_g, val_r, val_i, out_dir)
    plot_coverage_bar(sorties_greedy, sorties_regret, sorties_ils, out_dir)
    plot_lateness_histogram(sorties_greedy, sorties_regret, sorties_ils, out_dir)
    save_animation(sorties_ils, out_dir)
