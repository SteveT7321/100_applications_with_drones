"""
S037 Reverse Logistics
======================
A fleet of 5 drones collects returned parcels from 12 customer locations
scattered across a 500×500 m arena (customers in [50, 450]^2) and consolidates
them at a central depot. The problem is a Capacitated Vehicle Routing Problem
with Time Windows (CVRPTW), solved in three stages: (1) Clarke-Wright savings
initialisation, (2) 2-opt intra-route improvement, and (3) Or-opt inter-route
relocation of single customers. Capacity Q_max is a hard constraint; lateness
and range excess (R_max) are soft penalties in the cost J.

The showcase instance (seed 42) is plotted and animated; the same three-stage
pipeline is also run on N_INSTANCES further seeded instances to check whether
the per-stage improvements generalise.

Usage:
    conda activate drones
    python src/02_logistics_delivery/s037_reverse_logistics.py
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.animation as animation
import itertools
from dataclasses import dataclass
from typing import List

# ── Parameters ─────────────────────────────────────────────────────────────────
N_DRONES    = 5          # fleet size
N_CUSTOMERS = 12         # return-parcel pick-up locations
Q_MAX       = 3.0        # kg  — drone payload capacity
R_MAX       = 2000.0     # m   — max route length per battery charge
V_CRUISE    = 8.0        # m/s — cruise speed
LAMBDA_TW   = 50.0       # penalty per second of lateness
MU_RANGE    = 1000.0     # penalty per metre of range excess
DT          = 0.1        # s   — simulation timestep
ARENA_SIZE  = 500.0      # m
DEPOT       = np.array([250.0, 250.0])
SHOWCASE_SEED = 42       # instance plotted / animated
N_INSTANCES   = 30       # extra instances (seeds 0 .. 29) for the generalisation study
GIF_FRAMES    = 150

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '02_logistics_delivery', 's037_reverse_logistics',
)


# ── Data classes ───────────────────────────────────────────────────────────────
@dataclass
class Customer:
    id:     int
    pos:    np.ndarray   # (2,) metres
    weight: float        # kg
    e:      float        # earliest pick-up time (s)
    l:      float        # latest pick-up time (s)


# ── Helpers ────────────────────────────────────────────────────────────────────
def route_length(seq: List[int], customers: List[Customer]) -> float:
    """Total round-trip distance (depot → stops → depot)."""
    if not seq:
        return 0.0
    pts = [DEPOT] + [customers[i].pos for i in seq] + [DEPOT]
    return float(sum(np.linalg.norm(pts[j+1] - pts[j]) for j in range(len(pts)-1)))


def compute_arrivals(seq: List[int], customers: List[Customer]) -> List[float]:
    """Propagate arrival times along route, waiting at early windows."""
    arrivals, t, prev = [], 0.0, DEPOT
    for cid in seq:
        travel = np.linalg.norm(customers[cid].pos - prev) / V_CRUISE
        t = max(customers[cid].e, t + travel)
        arrivals.append(t)
        prev = customers[cid].pos
    return arrivals


def total_violation(seq: List[int], customers: List[Customer]) -> float:
    """Sum of lateness (seconds past l_i) across all stops."""
    arrivals = compute_arrivals(seq, customers)
    return float(sum(max(0.0, arrivals[j] - customers[seq[j]].l)
                     for j in range(len(seq))))


def route_cost(seq: List[int], customers: List[Customer]) -> float:
    """Combined cost: distance + lambda * lateness + mu * range excess."""
    L = route_length(seq, customers)
    V = total_violation(seq, customers)
    return L + LAMBDA_TW * V + MU_RANGE * max(0.0, L - R_MAX)


def payload(seq: List[int], customers: List[Customer]) -> float:
    return sum(customers[i].weight for i in seq)


# ── Clarke-Wright savings ──────────────────────────────────────────────────────
def clarke_wright(customers: List[Customer]) -> List[List[int]]:
    M = len(customers)
    # savings matrix
    S = np.zeros((M, M))
    for i, j in itertools.combinations(range(M), 2):
        s = (np.linalg.norm(customers[i].pos - DEPOT) +
             np.linalg.norm(customers[j].pos - DEPOT) -
             np.linalg.norm(customers[i].pos - customers[j].pos))
        S[i, j] = S[j, i] = s

    routes: List[List[int]] = [[i] for i in range(M)]
    pairs = sorted(itertools.combinations(range(M), 2),
                   key=lambda p: -S[p[0], p[1]])

    for (i, j) in pairs:
        if len(routes) <= N_DRONES:
            break
        # i at the tail of one route and j at the head of another, or vice versa
        for a, b in ((i, j), (j, i)):
            ra = next((r for r in routes if r[-1] == a), None)
            rb = next((r for r in routes if r[0] == b), None)
            if ra is None or rb is None or ra is rb:
                continue
            merged = ra + rb
            if payload(merged, customers) <= Q_MAX:
                routes.remove(ra)
                routes.remove(rb)
                routes.append(merged)
                break

    if len(routes) > N_DRONES:
        # never drop customers silently
        raise RuntimeError(f'Clarke-Wright left {len(routes)} routes for {N_DRONES} drones')
    while len(routes) < N_DRONES:
        routes.append([])
    return routes


# ── 2-opt intra-route improvement ─────────────────────────────────────────────
def two_opt(route: List[int], customers: List[Customer]) -> List[int]:
    """Reverse every sub-sequence best[j..k] (0 <= j < k < n), including ones
    that start at the first stop, and keep improving reversals."""
    best = route[:]
    improved = True
    while improved:
        improved = False
        for j in range(0, len(best) - 1):
            for k in range(j + 1, len(best)):
                candidate = best[:j] + best[j:k+1][::-1] + best[k+1:]
                if route_cost(candidate, customers) < route_cost(best, customers) - 1e-6:
                    best = candidate
                    improved = True
    return best


# ── Or-opt inter-route relocation ─────────────────────────────────────────────
def or_opt_relocate(routes: List[List[int]], customers: List[Customer]) -> List[List[int]]:
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
                    if payload(routes[k_dst], customers) + customers[cid].weight > Q_MAX:
                        continue
                    for pos_dst in range(len(routes[k_dst]) + 1):
                        src_new = routes[k_src][:pos_src] + routes[k_src][pos_src+1:]
                        dst_new = routes[k_dst][:pos_dst] + [cid] + routes[k_dst][pos_dst:]
                        delta = (route_cost(src_new, customers) +
                                 route_cost(dst_new, customers) -
                                 route_cost(routes[k_src], customers) -
                                 route_cost(routes[k_dst], customers))
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


# ── Simulation ─────────────────────────────────────────────────────────────────
def make_instance(seed: int) -> List[Customer]:
    rng = np.random.default_rng(seed)
    return [
        Customer(
            id=i,
            pos=rng.uniform(50, 450, size=2),
            weight=float(rng.uniform(0.3, 1.2)),
            e=float(rng.uniform(0, 60)),
            l=float(rng.uniform(90, 180)),
        )
        for i in range(N_CUSTOMERS)
    ]


def solve(customers):
    routes_cw = clarke_wright(customers)                          # Phase 1
    routes_2opt = [two_opt(r, customers) for r in routes_cw]      # Phase 2
    routes_final = or_opt_relocate(routes_2opt, customers)        # Phase 3
    served = sorted(c for r in routes_final for c in r)
    assert served == list(range(len(customers))), 'customer lost or duplicated'
    assert all(payload(r, customers) <= Q_MAX + 1e-9 for r in routes_final)
    return routes_cw, routes_2opt, routes_final


def run_simulation():
    customers = make_instance(SHOWCASE_SEED)
    routes_cw, routes_2opt, routes_final = solve(customers)

    # Generalisation study over further seeded instances
    study = []
    for seed in range(N_INSTANCES):
        cs = make_instance(seed)
        r_cw, r_2o, r_fin = solve(cs)
        study.append(dict(seed=seed,
                          cw=compute_stage_metrics(r_cw, cs),
                          two=compute_stage_metrics(r_2o, cs),
                          fin=compute_stage_metrics(r_fin, cs)))

    return customers, routes_cw, routes_2opt, routes_final, study


def compute_stage_metrics(routes, customers):
    """Return (total_dist, total_lateness, total_cost) for a set of routes."""
    dist = sum(route_length(r, customers) for r in routes)
    late = sum(total_violation(r, customers) for r in routes)
    cost = sum(route_cost(r, customers) for r in routes)
    return dist, late, cost


# ── Drone animation trajectories ───────────────────────────────────────────────
def build_trajectories(routes, customers):
    """Return list of (N,2) position arrays, one per drone, at DT steps."""
    trajs = []
    for route in routes:
        if not route:
            trajs.append(np.array([[DEPOT[0], DEPOT[1]]]))
            continue
        # knots: (time, position); a drone that arrives before e_i hovers there
        knots_t, knots_p = [0.0], [DEPOT]
        t, prev = 0.0, DEPOT
        for cid in route:
            c = customers[cid]
            t += np.linalg.norm(c.pos - prev) / V_CRUISE
            knots_t.append(t)
            knots_p.append(c.pos)
            if t < c.e:                      # wait for the window to open
                t = c.e
                knots_t.append(t)
                knots_p.append(c.pos)
            prev = c.pos
        t += np.linalg.norm(DEPOT - prev) / V_CRUISE
        knots_t.append(t)
        knots_p.append(DEPOT)

        ts = np.arange(0.0, t + DT / 2, DT)     # sample k is time k*DT
        pos_x = np.interp(ts, knots_t, [p[0] for p in knots_p])
        pos_y = np.interp(ts, knots_t, [p[1] for p in knots_p])
        trajs.append(np.column_stack([pos_x, pos_y]))
    return trajs


# ── Plots ───────────────────────────────────────────────────────────────────────
DRONE_COLORS = ['#e6194b', '#3cb44b', '#4363d8', '#f58231', '#911eb4']


def plot_route_map(customers, routes_final, out_dir):
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_xlim(0, ARENA_SIZE)
    ax.set_ylim(0, ARENA_SIZE)
    ax.set_aspect('equal')

    # Time-window tightness heatmap (background circles)
    for c in customers:
        width = (c.l - c.e)  # window width, 30 s (tight) … 180 s (wide)
        norm = max(0, min(1, (width - 30) / 150))
        color = plt.cm.RdYlGn(norm)
        ax.add_patch(plt.Circle(c.pos, 18, color=color, alpha=0.25, zorder=1))

    # Depot
    ax.plot(*DEPOT, 's', color='black', ms=14, zorder=5, label='Depot')

    # Routes
    for k, route in enumerate(routes_final):
        if not route:
            continue
        col = DRONE_COLORS[k % len(DRONE_COLORS)]
        pts = [DEPOT] + [customers[i].pos for i in route] + [DEPOT]
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        ax.plot(xs, ys, '-', color=col, lw=1.8, alpha=0.8, zorder=2)
        ax.plot(xs, ys, 'o', color=col, ms=4, zorder=3)
        # customer circles sized by weight
        for idx, cid in enumerate(route):
            c = customers[cid]
            sz = 40 + c.weight * 60
            ax.scatter(*c.pos, s=sz, color=col, edgecolors='black',
                       linewidths=0.7, zorder=4)
            ax.text(c.pos[0]+5, c.pos[1]+5, f'{idx+1}', fontsize=7,
                    color=col, zorder=6)
        ax.plot([], [], '-o', color=col, label=f'Drone {k+1}')

    # Legend for time window heatmap (use proxy artists, not add_patch)
    tight_proxy = mpatches.Rectangle((0, 0), 1, 1, color=plt.cm.RdYlGn(0.0),
                                     alpha=0.6, label='Tight TW')
    wide_proxy  = mpatches.Rectangle((0, 0), 1, 1, color=plt.cm.RdYlGn(1.0),
                                     alpha=0.6, label='Wide TW')

    ax.set_title('S037 Reverse Logistics — Final Routes')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    # Collect all handles/labels and add TW proxies
    handles, labels = ax.get_legend_handles_labels()
    handles += [tight_proxy, wide_proxy]
    labels  += ['Tight TW', 'Wide TW']
    ax.legend(handles, labels, loc='upper right', fontsize=7)
    ax.grid(True, alpha=0.2)

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'route_map.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def plot_cost_comparison(customers, routes_cw, routes_2opt, routes_final, study, out_dir):
    """Left: stage metrics on the showcase instance. Right: mean over the study."""
    stages = ['Clarke-Wright', '+ 2-opt', '+ Or-opt']
    show = [
        compute_stage_metrics(routes_cw, customers),
        compute_stage_metrics(routes_2opt, customers),
        compute_stage_metrics(routes_final, customers),
    ]
    mean = [tuple(np.mean([s[key][i] for s in study]) for i in range(3))
            for key in ('cw', 'two', 'fin')]

    x = np.arange(3)
    w = 0.25
    fig, axes = plt.subplots(1, 2, figsize=(13, 5))
    for ax, metrics, title in (
            (axes[0], show, f'Showcase instance (seed {SHOWCASE_SEED})'),
            (axes[1], mean, f'Mean over {len(study)} instances (seeds 0..{len(study)-1})')):
        dists = [m[0] for m in metrics]
        lates = [m[1] * LAMBDA_TW for m in metrics]  # convert to cost units
        costs = [m[2] for m in metrics]
        ax.bar(x - w, dists,  w, label='Distance (m)',     color='steelblue')
        ax.bar(x,     lates,  w, label='λ·Lateness (cost)', color='tomato')
        ax.bar(x + w, costs,  w, label='Total cost J',      color='seagreen')
        for xs, vals in ((x - w, dists), (x, lates), (x + w, costs)):
            for xi, v in zip(xs, vals):
                ax.text(xi, v + 15, f'{v:.0f}', ha='center', fontsize=8)
        ax.set_xticks(x)
        ax.set_xticklabels(stages)
        ax.set_ylim(0, 2500)
        ax.set_title(title)
        ax.set_ylabel('Cost')
        ax.legend(loc='upper center', ncol=3, fontsize=8)
        ax.grid(axis='y', alpha=0.3)
    fig.suptitle('Cost Across Optimisation Stages', fontweight='bold')
    plt.tight_layout()

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'cost_comparison.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def plot_payload_trace(customers, routes_final, out_dir):
    fig, ax = plt.subplots(figsize=(9, 4))
    for k, route in enumerate(routes_final):
        if not route:
            continue
        weights = [customers[i].weight for i in route]
        bottoms = np.cumsum([0] + weights[:-1])
        for j, (w, b) in enumerate(zip(weights, bottoms)):
            ax.bar(k, w, bottom=b, color=DRONE_COLORS[k],
                   edgecolor='black', linewidth=0.5)
            ax.text(k, b + w / 2, f'C{route[j]}', ha='center', va='center',
                    fontsize=7, color='white')
        ax.text(k, sum(weights) + 0.05, f'{sum(weights):.2f} kg', ha='center',
                fontsize=8)
    ax.axhline(Q_MAX, color='red', ls='--', lw=1.5, label=f'Q_max = {Q_MAX} kg')
    ax.set_xticks(range(N_DRONES))
    ax.set_xticklabels([f'Drone {k+1}' for k in range(N_DRONES)])
    ax.set_ylim(0, Q_MAX + 0.5)
    ax.set_ylabel('Cumulative payload (kg)')
    ax.set_title('Payload Trace per Drone (segments = pick-ups in route order)')
    ax.legend(fontsize=8)
    ax.grid(axis='y', alpha=0.3)

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'payload_trace.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def plot_instance_study(study, out_dir):
    """Per-instance cost reduction contributed by 2-opt and by Or-opt."""
    seeds = [s['seed'] for s in study]
    d2 = [(s['cw'][2] - s['two'][2]) / s['cw'][2] * 100 for s in study]
    d3 = [(s['two'][2] - s['fin'][2]) / s['cw'][2] * 100 for s in study]
    x = np.arange(len(seeds))
    fig, ax = plt.subplots(figsize=(11, 4.5))
    ax.bar(x - 0.2, d2, 0.4, color='#4363d8', label='2-opt step (CW → 2-opt)')
    ax.bar(x + 0.2, d3, 0.4, color='#f58231', label='Or-opt step (2-opt → Or-opt)')
    ax.set_xticks(x)
    ax.set_xticklabels([str(s) for s in seeds], fontsize=7)
    ax.set_xlabel('Instance seed')
    ax.set_ylabel('Reduction of J (% of Clarke-Wright J)')
    ax.set_title(f'Contribution of each improvement step over {len(seeds)} seeded instances')
    ax.legend(fontsize=9)
    ax.grid(axis='y', alpha=0.3)
    plt.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'instance_study.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def plot_time_windows(customers, routes_final, out_dir):
    """Horizontal bar per customer: [e_i, l_i] window + actual arrival marker."""
    fig, ax = plt.subplots(figsize=(10, 6))
    y_labels, y_pos = [], []
    for k, route in enumerate(routes_final):
        if not route:
            continue
        arrivals = compute_arrivals(route, customers)
        for j, cid in enumerate(route):
            c = customers[cid]
            y = len(y_labels)
            y_labels.append(f'C{cid}  (D{k+1})')
            y_pos.append(y)
            on_time = arrivals[j] <= c.l
            bar_color = '#88cc88' if on_time else '#cc4444'
            ax.barh(y, c.l - c.e, left=c.e, height=0.6,
                    color=bar_color, alpha=0.4, edgecolor='grey')
            ax.plot(arrivals[j], y, 'D',
                    color='green' if on_time else 'red', ms=7, zorder=5)

    ax.set_yticks(y_pos)
    ax.set_yticklabels(y_labels, fontsize=8)
    ax.set_xlabel('Time (s)')
    ax.set_title('Arrival Time vs Time Window')
    on_patch  = mpatches.Patch(color='#88cc88', alpha=0.6, label='On time')
    lat_patch = mpatches.Patch(color='#cc4444', alpha=0.6, label='Late')
    arr_marker = plt.Line2D([0], [0], marker='D', color='w',
                             markerfacecolor='grey', ms=7, label='Arrival')
    ax.legend(handles=[on_patch, lat_patch, arr_marker], fontsize=8)
    ax.grid(axis='x', alpha=0.3)
    plt.tight_layout()

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'time_windows.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')




def save_animation(customers, routes_final, out_dir):
    trajs = build_trajectories(routes_final, customers)
    max_len = max(len(t) for t in trajs)
    # pad shorter trajectories by repeating the last point (drone parked at depot)
    trajs_padded = [
        np.vstack([t, np.tile(t[-1], (max_len - len(t), 1))]) if len(t) < max_len else t
        for t in trajs
    ]
    arrivals_all = [compute_arrivals(r, customers) if r else [] for r in routes_final]
    steps = np.unique(np.linspace(0, max_len - 1, GIF_FRAMES).round().astype(int))

    fig, ax = plt.subplots(figsize=(7, 7))
    ax.set_xlim(0, ARENA_SIZE)
    ax.set_ylim(0, ARENA_SIZE)
    ax.set_aspect('equal')
    title = ax.set_title('')

    ax.plot(*DEPOT, 's', color='black', ms=12, zorder=5)
    cust_sc = ax.scatter(
        [c.pos[0] for c in customers],
        [c.pos[1] for c in customers],
        s=[50 + c.weight * 60 for c in customers],
        c='lightblue', edgecolors='grey', linewidths=0.8, zorder=3
    )
    drone_dots = [ax.plot([], [], 'o', color=DRONE_COLORS[k], ms=10, zorder=6)[0]
                  for k in range(N_DRONES)]
    drone_trails = [ax.plot([], [], '-', color=DRONE_COLORS[k], lw=1.2, alpha=0.5)[0]
                    for k in range(N_DRONES)]
    payload_texts = [ax.text(0, 0, '', fontsize=7, color=DRONE_COLORS[k],
                             ha='center', va='bottom', zorder=7)
                     for k in range(N_DRONES)]

    def animate(frame):
        step = steps[frame]
        t_now = step * DT
        cust_colors = ['lightblue'] * N_CUSTOMERS
        for k, (route, traj) in enumerate(zip(routes_final, trajs_padded)):
            pos = traj[step]
            trail = traj[max(0, step - 300):step + 1]
            drone_dots[k].set_data([pos[0]], [pos[1]])
            drone_trails[k].set_data(trail[:, 0], trail[:, 1])
            pay = sum(customers[cid].weight
                      for cid, ta in zip(route, arrivals_all[k]) if t_now >= ta)
            payload_texts[k].set_position((pos[0], pos[1] + 12))
            payload_texts[k].set_text(f'{pay:.1f}kg' if route else '')
            # picked-up customers take the colour of their drone
            for cid, ta in zip(route, arrivals_all[k]):
                if t_now >= ta:
                    cust_colors[cid] = DRONE_COLORS[k]
        cust_sc.set_facecolors(cust_colors)
        title.set_text(f'S037 Reverse Logistics — t = {t_now:.0f} s')
        return drone_dots + drone_trails + payload_texts + [cust_sc, title]

    ani = animation.FuncAnimation(fig, animate, frames=len(steps), blit=False)
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer='pillow', fps=20, dpi=90)
    plt.close()
    print(f'Saved: {path} ({len(steps)} frames)')


# ── Main ────────────────────────────────────────────────────────────────────────
if __name__ == '__main__':
    customers, routes_cw, routes_2opt, routes_final, study = run_simulation()

    m_cw    = compute_stage_metrics(routes_cw, customers)
    m_2opt  = compute_stage_metrics(routes_2opt, customers)
    m_final = compute_stage_metrics(routes_final, customers)

    print(f'\n=== S037 Reverse Logistics — showcase instance (seed {SHOWCASE_SEED}) ===')
    print(f'Total parcel weight: {sum(c.weight for c in customers):.2f} kg '
          f'(fleet capacity {N_DRONES * Q_MAX:.1f} kg)')
    print(f'{"Stage":<22} {"Distance(m)":>12} {"Lateness(s)":>12} {"Total Cost":>12}')
    print('-' * 62)
    for label, m in [('Clarke-Wright', m_cw), ('After 2-opt', m_2opt), ('After Or-opt', m_final)]:
        print(f'{label:<22} {m[0]:>12.1f} {m[1]:>12.1f} {m[2]:>12.1f}')

    for label, rs in [('Clarke-Wright', routes_cw), ('After 2-opt', routes_2opt),
                      ('After Or-opt', routes_final)]:
        print(f'  {label:<14} routes: {rs}')

    print('\n=== Fleet Utilisation (final routes) ===')
    print(f'{"Drone":<8} {"Stops":>6} {"Dist(m)":>10} {"Payload(kg)":>12} '
          f'{"Lateness(s)":>12} {"Return(s)":>10}')
    print('-' * 62)
    for k, route in enumerate(routes_final):
        d = route_length(route, customers)
        p = payload(route, customers)
        v = total_violation(route, customers)
        if route:
            arr = compute_arrivals(route, customers)
            t_ret = arr[-1] + np.linalg.norm(DEPOT - customers[route[-1]].pos) / V_CRUISE
        else:
            t_ret = 0.0
        print(f'  D{k+1:<5} {len(route):>6} {d:>10.1f} {p:>12.2f} {v:>12.1f} {t_ret:>10.1f}')

    dist_imp = (m_cw[0] - m_final[0]) / m_cw[0] * 100
    cost_imp = (m_cw[2] - m_final[2]) / m_cw[2] * 100
    print(f'\nDistance improvement vs CW: {dist_imp:.1f}%')
    print(f'Total cost improvement vs CW: {cost_imp:.1f}%')

    on_time = late_count = 0
    for route in routes_final:
        arrivals = compute_arrivals(route, customers)
        for j, cid in enumerate(route):
            if arrivals[j] <= customers[cid].l:
                on_time += 1
            else:
                late_count += 1
    print(f'Customers on-time: {on_time}/{N_CUSTOMERS}, late: {late_count}/{N_CUSTOMERS}')

    # ── Generalisation study ────────────────────────────────────────────────
    J_cw  = np.array([s['cw'][2] for s in study])
    J_2o  = np.array([s['two'][2] for s in study])
    J_fin = np.array([s['fin'][2] for s in study])
    late_cw = np.array([s['cw'][1] for s in study])
    late_fin = np.array([s['fin'][1] for s in study])
    red2 = (J_cw - J_2o) / J_cw * 100
    red3 = (J_2o - J_fin) / J_cw * 100
    print(f'\n=== Generalisation study: {N_INSTANCES} instances (seeds 0..{N_INSTANCES-1}) ===')
    print('seed   J_CW    J_2opt  J_Or-opt  2opt%  Or-opt%')
    for s, a, b, c, p2, p3 in zip(range(N_INSTANCES), J_cw, J_2o, J_fin, red2, red3):
        print(f'{s:>4} {a:>7.1f} {b:>8.1f} {c:>8.1f} {p2:>6.2f} {p3:>7.2f}')
    print(f'Mean J  CW / 2-opt / Or-opt          : {J_cw.mean():.1f} / {J_2o.mean():.1f} / {J_fin.mean():.1f}')
    print(f'2-opt step: improves J in {int((red2 > 1e-6).sum())}/{N_INSTANCES} instances, '
          f'mean reduction {red2.mean():.2f}% of CW J (max {red2.max():.2f}%)')
    print(f'Or-opt step: improves J in {int((red3 > 1e-6).sum())}/{N_INSTANCES} instances, '
          f'mean reduction {red3.mean():.2f}% of CW J (max {red3.max():.2f}%)')
    print(f'Or-opt step larger than 2-opt step in {int((red3 > red2 + 1e-9).sum())}/{N_INSTANCES} instances, '
          f'2-opt larger in {int((red2 > red3 + 1e-9).sum())}, tie in '
          f'{int((np.abs(red2 - red3) <= 1e-9).sum())}')
    print(f'Instances with lateness > 0: CW {int((late_cw > 1e-6).sum())}/{N_INSTANCES}, '
          f'final {int((late_fin > 1e-6).sum())}/{N_INSTANCES}')

    out_dir = os.path.normpath(OUTPUT_DIR)
    plot_route_map(customers, routes_final, out_dir)
    plot_cost_comparison(customers, routes_cw, routes_2opt, routes_final, study, out_dir)
    plot_payload_trace(customers, routes_final, out_dir)
    plot_time_windows(customers, routes_final, out_dir)
    plot_instance_study(study, out_dir)
    save_animation(customers, routes_final, out_dir)
