"""
S033 Online Order Insertion
============================
A fleet of K=4 drones is executing a pre-planned delivery schedule when new
customer orders arrive mid-mission. The dispatcher evaluates every feasible
insertion position across all drones and splices each new order into the
globally cheapest slot, subject to battery-range and deadline constraints.
Four strategies are compared: Cheapest Insertion, Nearest Drone, Least Loaded,
and Random Feasible. The simulation advances drones along their routes step-by-
step and dispatches orders as they arrive.

Usage:
    conda activate drones
    python src/02_logistics_delivery/s033_online_order_insertion.py
"""

import sys, os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.lines as mlines
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

# ── Parameters ─────────────────────────────────
K            = 4          # number of drones
V_DRONE      = 8.0        # m/s cruise speed
B_FULL       = 10_000.0   # m  full-battery range equivalent
DT           = 0.1        # s  simulation timestep
N_WP_INIT    = 12         # pre-planned waypoints per drone at t=0
N_INSERTIONS = 8          # online orders arriving mid-mission
T_ORDER_LO   = 10.0       # s  earliest order arrival
T_ORDER_HI   = 80.0       # s  latest order arrival
DEADLINE_SLACK = 30.0     # s  added on top of nearest-drone ETA for deadline
ARENA_LO     = -500.0     # m
ARENA_HI     =  500.0     # m
ALT_LO       = 2.0        # m
ALT_HI       = 8.0        # m
DEPOT        = np.array([0.0, 0.0, 2.0])

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '02_logistics_delivery', 's033_online_order_insertion',
)

RNG = np.random.default_rng(0)   # fixed seed for reproducibility

DRONE_COLORS = ['tab:red', 'tab:blue', 'tab:green', 'tab:orange']
STRATEGIES   = ['cheapest_insertion', 'nearest_drone', 'least_loaded', 'random_feasible']


# ── Helpers ────────────────────────────────────

def rand_wp():
    """Random 3-D waypoint inside arena."""
    xy = RNG.uniform(ARENA_LO, ARENA_HI, 2)
    z  = RNG.uniform(ALT_LO,   ALT_HI)
    return np.array([xy[0], xy[1], z])


def route_length(pts: List[np.ndarray]) -> float:
    """Sum of Euclidean segment lengths."""
    return sum(np.linalg.norm(pts[i+1] - pts[i]) for i in range(len(pts)-1))


def insertion_delta(a: np.ndarray, q: np.ndarray, b: np.ndarray) -> float:
    """Extra distance introduced by inserting q between a and b."""
    return (np.linalg.norm(a - q) + np.linalg.norm(q - b)
            - np.linalg.norm(b - a))


# ── Data classes ───────────────────────────────

@dataclass
class Order:
    loc: np.ndarray
    t_arrive: float
    t_deadline: float


@dataclass
class Drone:
    id: int
    pos: np.ndarray
    route: List[np.ndarray]          # remaining waypoints (index 0 = next)
    battery_rem: float               # metres of flight remaining
    speed: float = V_DRONE
    # tracking
    path_x: List[float] = field(default_factory=list)
    path_y: List[float] = field(default_factory=list)
    initial_route: List[np.ndarray] = field(default_factory=list)
    inserted_orders: int = 0         # count of online orders assigned

    def full_pts(self) -> List[np.ndarray]:
        """Current position prepended to remaining route."""
        return [self.pos] + self.route

    def remaining_length(self) -> float:
        return route_length(self.full_pts())

    def best_insertion(self, q: np.ndarray, t_now: float, t_deadline: float
                       ) -> Optional[Tuple[int, float]]:
        """Return (insert_idx, delta) for cheapest feasible slot, or None."""
        pts = self.full_pts()
        n_seg = len(pts) - 1
        if n_seg < 1:
            return None
        best_idx, best_delta = None, float('inf')
        for idx in range(n_seg):
            delta = insertion_delta(pts[idx], q, pts[idx + 1])
            new_len = self.remaining_length() + delta
            # distance to q via this insertion prefix
            dist_to_q = route_length(pts[:idx+1]) + np.linalg.norm(pts[idx] - q)
            t_arr = t_now + dist_to_q / self.speed
            if new_len <= self.battery_rem and t_arr <= t_deadline:
                if delta < best_delta:
                    best_delta = delta
                    best_idx   = idx
        if best_idx is None:
            return None
        return best_idx, best_delta

    def insert_order(self, q: np.ndarray, insert_idx: int) -> float:
        """Splice q into route; return detour cost."""
        pts = self.full_pts()
        delta = insertion_delta(pts[insert_idx], q, pts[insert_idx + 1])
        self.route.insert(insert_idx, q)
        self.battery_rem -= delta
        self.inserted_orders += 1
        return delta

    def step(self, dt: float):
        """Advance drone toward next waypoint; record path."""
        self.path_x.append(self.pos[0])
        self.path_y.append(self.pos[1])
        if not self.route:
            return
        target = self.route[0]
        diff   = target - self.pos
        dist   = np.linalg.norm(diff)
        move   = self.speed * dt
        if dist <= move:
            self.pos = target.copy()
            self.battery_rem = max(0.0, self.battery_rem - dist)
            self.route.pop(0)
        else:
            self.pos = self.pos + diff / dist * move
            self.battery_rem = max(0.0, self.battery_rem - move)

    @property
    def route_done(self) -> bool:
        return len(self.route) == 0


# ── Fleet & order initialisation ───────────────

def initialise_fleet() -> List[Drone]:
    drones = []
    for k in range(K):
        wps = [rand_wp() for _ in range(N_WP_INIT)]
        wps.append(DEPOT.copy())
        bat = B_FULL * RNG.uniform(0.75, 1.0)  # slight variation
        d = Drone(id=k, pos=DEPOT.copy(), route=wps,
                  battery_rem=bat, speed=V_DRONE)
        d.initial_route = [DEPOT.copy()] + [w.copy() for w in wps]
        drones.append(d)
    return drones


def generate_orders() -> List[Order]:
    orders = []
    arrivals = np.sort(RNG.uniform(T_ORDER_LO, T_ORDER_HI, N_INSERTIONS))
    for t_a in arrivals:
        loc = rand_wp()
        # deadline = now + nearest-drone flying time + slack
        t_dl = t_a + DEADLINE_SLACK + RNG.uniform(0, 20)
        orders.append(Order(loc=loc, t_arrive=t_a, t_deadline=t_dl))
    return orders


# ── Dispatch strategies ────────────────────────

def dispatch_cheapest(drones: List[Drone], order: Order, t_now: float
                      ) -> Optional[Tuple[int, int, float]]:
    """Global cheapest-insertion across all feasible drones."""
    best = None
    for d in drones:
        res = d.best_insertion(order.loc, t_now, order.t_deadline)
        if res is None:
            continue
        idx, delta = res
        if best is None or delta < best[2]:
            best = (d.id, idx, delta)
    return best


def dispatch_nearest(drones: List[Drone], order: Order, t_now: float
                     ) -> Optional[Tuple[int, int, float]]:
    """Assign to drone whose current position is closest to the new order."""
    feasible = []
    for d in drones:
        res = d.best_insertion(order.loc, t_now, order.t_deadline)
        if res is None:
            continue
        dist_cur = np.linalg.norm(d.pos - order.loc)
        feasible.append((dist_cur, d.id, res[0], res[1]))
    if not feasible:
        return None
    feasible.sort(key=lambda x: x[0])
    _, did, idx, delta = feasible[0]
    return (did, idx, delta)


def dispatch_least_loaded(drones: List[Drone], order: Order, t_now: float
                          ) -> Optional[Tuple[int, int, float]]:
    """Assign to feasible drone with fewest remaining waypoints."""
    feasible = []
    for d in drones:
        res = d.best_insertion(order.loc, t_now, order.t_deadline)
        if res is None:
            continue
        feasible.append((len(d.route), d.id, res[0], res[1]))
    if not feasible:
        return None
    feasible.sort(key=lambda x: x[0])
    _, did, idx, delta = feasible[0]
    return (did, idx, delta)


def dispatch_random(drones: List[Drone], order: Order, t_now: float
                    ) -> Optional[Tuple[int, int, float]]:
    """Uniform random draw from feasible drones."""
    feasible = []
    for d in drones:
        res = d.best_insertion(order.loc, t_now, order.t_deadline)
        if res is None:
            continue
        feasible.append((d.id, res[0], res[1]))
    if not feasible:
        return None
    choice = feasible[RNG.integers(len(feasible))]
    return choice


DISPATCH_FNS = {
    'cheapest_insertion': dispatch_cheapest,
    'nearest_drone':      dispatch_nearest,
    'least_loaded':       dispatch_least_loaded,
    'random_feasible':    dispatch_random,
}


# ── Simulation ─────────────────────────────────

def run_simulation(strategy: str = 'cheapest_insertion'):
    """Run full simulation for one strategy. Return metrics + per-drone data."""
    # Re-seed to same state so all strategies get the same drones & orders
    RNG.bit_generator.state = np.random.default_rng(0).bit_generator.state

    drones = initialise_fleet()
    orders = generate_orders()
    order_queue = sorted(orders, key=lambda o: o.t_arrive)

    metrics = {
        'total_detour': 0.0,
        'feasible':     0,
        'queued':       0,
        'strategy':     strategy,
        'order_events': [],   # (t, drone_id, delta) for timeline plot
    }
    per_drone_inserted = [0] * K

    t = 0.0
    T_MAX = T_ORDER_HI + 150.0   # run until all routes are done
    dispatch_fn = DISPATCH_FNS[strategy]

    while (any(not d.route_done for d in drones) or order_queue) and t <= T_MAX:
        # Dispatch newly arrived orders
        due = [o for o in order_queue if o.t_arrive <= t]
        for order in due:
            order_queue.remove(order)
            result = dispatch_fn(drones, order, t)
            if result is not None:
                did, idx, delta = result
                drone = next(d for d in drones if d.id == did)
                drone.insert_order(order.loc, idx)
                metrics['total_detour']  += delta
                metrics['feasible']      += 1
                per_drone_inserted[did]  += 1
                metrics['order_events'].append((t, did, delta))
            else:
                metrics['queued'] += 1

        # Advance drones
        for drone in drones:
            drone.step(DT)

        t += DT

    metrics['per_drone_inserted'] = per_drone_inserted
    metrics['drones'] = drones
    metrics['orders'] = orders
    return metrics


def run_all_strategies():
    """Run simulation for all four strategies; return dict keyed by strategy."""
    results = {}
    for s in STRATEGIES:
        results[s] = run_simulation(s)
    return results


# ── Plots ──────────────────────────────────────

def plot_route_map(metrics, out_dir):
    """2-D top-down: initial routes (dashed) vs updated routes (solid)."""
    fig, ax = plt.subplots(figsize=(9, 9))
    ax.set_facecolor('#f7f7f7')

    drones = metrics['drones']
    orders = metrics['orders']

    for d in drones:
        c = DRONE_COLORS[d.id]
        lbl = f'Drone {d.id+1}'
        # Initial planned route
        ir = d.initial_route
        ax.plot([p[0] for p in ir], [p[1] for p in ir],
                '--', color=c, linewidth=1.0, alpha=0.5)
        # Actual path flown
        ax.plot(d.path_x, d.path_y, '-', color=c, linewidth=1.8,
                label=lbl)
        # Start position = depot
        ax.plot(ir[0][0], ir[0][1], 's', color=c, markersize=7)

    # Depot
    ax.plot(DEPOT[0], DEPOT[1], 'k*', markersize=14, label='Depot', zorder=5)

    # New order locations
    for i, o in enumerate(orders):
        ax.plot(o.loc[0], o.loc[1], 'p', color='purple',
                markersize=9, zorder=4)
    ax.plot([], [], 'p', color='purple', markersize=9, label='Online Orders')

    # Legend helpers
    dash_patch = mlines.Line2D([], [], linestyle='--', color='grey', alpha=0.6,
                               label='Planned route (dashed)')

    ax.set_xlim(ARENA_LO - 50, ARENA_HI + 50)
    ax.set_ylim(ARENA_LO - 50, ARENA_HI + 50)
    ax.set_xlabel('x (m)', fontsize=11)
    ax.set_ylabel('y (m)', fontsize=11)
    ax.set_title('S033 Online Order Insertion — Route Map\n'
                 '(dashed = planned, solid = actual)', fontsize=12)
    handles, labels = ax.get_legend_handles_labels()
    handles.append(dash_patch)
    ax.legend(handles=handles, loc='upper right', fontsize=9, framealpha=0.85)
    ax.grid(True, linestyle=':', alpha=0.5)

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'route_map.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def plot_battery_timeline(metrics, out_dir):
    """Drone battery consumption over time with order-arrival markers."""
    fig, ax = plt.subplots(figsize=(10, 5))

    drones = metrics['drones']
    # Reconstruct battery-over-time from route lengths (approx)
    # We plot remaining_length proxy: just mark events
    events = metrics['order_events']

    for ev in events:
        t_ev, did, delta = ev
        ax.axvline(t_ev, color=DRONE_COLORS[did], linestyle=':', alpha=0.7)
        ax.text(t_ev + 0.5, 0.92 - did * 0.07,
                f'D{did+1} +{delta:.0f}m',
                color=DRONE_COLORS[did], fontsize=7.5,
                transform=ax.get_xaxis_transform())

    # Show order arrival times
    orders = metrics['orders']
    for o in orders:
        ax.axvspan(o.t_arrive, o.t_deadline, alpha=0.04, color='grey')

    for i, (t_ev, did, delta) in enumerate(events):
        ax.annotate('', xy=(t_ev, 0.1 + did * 0.2),
                    xytext=(t_ev - 2, 0.1 + did * 0.2),
                    xycoords=('data', 'axes fraction'),
                    textcoords=('data', 'axes fraction'),
                    arrowprops=dict(arrowstyle='->', color=DRONE_COLORS[did],
                                   lw=1.5))

    ax.set_xlim(0, T_ORDER_HI + 10)
    ax.set_ylim(0, 1)
    ax.set_xlabel('Simulation time (s)', fontsize=11)
    ax.set_ylabel('Relative timeline', fontsize=11)
    ax.set_title('S033 Order Arrival Timeline\n'
                 '(vertical lines = order assigned to drone; '
                 'grey bands = arrival→deadline window)', fontsize=11)

    handles = [mpatches.Patch(color=DRONE_COLORS[i], label=f'Drone {i+1}')
               for i in range(K)]
    ax.legend(handles=handles, loc='upper left', fontsize=9)
    ax.grid(True, axis='x', linestyle=':', alpha=0.5)

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'order_timeline.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def plot_strategy_comparison(all_results, out_dir):
    """Bar chart: total detour cost per strategy + feasibility rate."""
    labels    = [s.replace('_', '\n') for s in STRATEGIES]
    detours   = [all_results[s]['total_detour'] for s in STRATEGIES]
    feas_rate = [all_results[s]['feasible'] / N_INSERTIONS * 100
                 for s in STRATEGIES]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 5))

    colors = ['#2ecc71', '#3498db', '#e67e22', '#9b59b6']

    bars = ax1.bar(labels, detours, color=colors, edgecolor='white', linewidth=1.2)
    ax1.bar_label(bars, fmt='%.0f m', padding=3, fontsize=9)
    ax1.set_ylabel('Total Detour Distance (m)', fontsize=11)
    ax1.set_title('Detour Cost by Strategy', fontsize=12)
    ax1.set_ylim(0, max(detours) * 1.25)
    ax1.grid(True, axis='y', linestyle=':', alpha=0.5)

    bars2 = ax2.bar(labels, feas_rate, color=colors, edgecolor='white', linewidth=1.2)
    ax2.bar_label(bars2, fmt='%.0f%%', padding=3, fontsize=9)
    ax2.set_ylabel('Feasibility Rate (%)', fontsize=11)
    ax2.set_title('Orders Served On-Time', fontsize=12)
    ax2.set_ylim(0, 115)
    ax2.axhline(100, color='grey', linestyle='--', linewidth=1)
    ax2.grid(True, axis='y', linestyle=':', alpha=0.5)

    fig.suptitle('S033 Strategy Comparison', fontsize=13, fontweight='bold')
    plt.tight_layout()

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'strategy_comparison.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def plot_load_balance(all_results, out_dir):
    """Per-drone insertion count histogram for each strategy."""
    fig, axes = plt.subplots(1, len(STRATEGIES), figsize=(13, 4), sharey=True)

    for ax, s in zip(axes, STRATEGIES):
        counts = all_results[s]['per_drone_inserted']
        bars = ax.bar(range(K), counts,
                      color=DRONE_COLORS[:K], edgecolor='white', linewidth=1.2)
        ax.bar_label(bars, padding=2, fontsize=9)
        ax.set_xticks(range(K))
        ax.set_xticklabels([f'D{i+1}' for i in range(K)], fontsize=9)
        ax.set_title(s.replace('_', ' ').title(), fontsize=10)
        ax.set_ylim(0, N_INSERTIONS + 1)
        ax.grid(True, axis='y', linestyle=':', alpha=0.5)

    axes[0].set_ylabel('Inserted Orders', fontsize=11)
    fig.suptitle('S033 Load Balance per Strategy', fontsize=13, fontweight='bold')
    plt.tight_layout()

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'load_balance.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def save_animation(metrics, out_dir):
    """Top-down animation of drone movement + order insertion events."""
    import matplotlib.animation as animation

    drones = metrics['drones']
    orders = metrics['orders']

    # ---- Re-simulate, storing positions every step ----
    RNG.bit_generator.state = np.random.default_rng(0).bit_generator.state
    sim_drones = initialise_fleet()
    sim_orders = generate_orders()
    order_queue = sorted(sim_orders, key=lambda o: o.t_arrive)
    dispatch_fn = DISPATCH_FNS['cheapest_insertion']

    pos_history  = {k: [] for k in range(K)}
    event_frames = []   # (frame_idx, drone_id, order_loc)

    t = 0.0
    frame = 0
    T_MAX = T_ORDER_HI + 80.0
    STEP  = 4   # record every 4 DT steps => fewer frames

    while (any(not d.route_done for d in sim_drones) or order_queue) and t <= T_MAX:
        due = [o for o in order_queue if o.t_arrive <= t]
        for order in due:
            order_queue.remove(order)
            result = dispatch_fn(sim_drones, order, t)
            if result is not None:
                did, idx, delta = result
                drone = next(d for d in sim_drones if d.id == did)
                drone.insert_order(order.loc, idx)
                event_frames.append((frame, did, order.loc.copy()))

        for d in sim_drones:
            d.step(DT)

        if frame % STEP == 0:
            for d in sim_drones:
                pos_history[d.id].append(d.pos[:2].copy())

        t    += DT
        frame += 1

    n_frames = min(len(pos_history[0]), 200)   # cap animation length
    step_ani = max(1, len(pos_history[0]) // n_frames)

    fig, ax = plt.subplots(figsize=(7, 7))
    ax.set_xlim(ARENA_LO - 50, ARENA_HI + 50)
    ax.set_ylim(ARENA_LO - 50, ARENA_HI + 50)
    ax.set_facecolor('#f0f0f0')
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')
    ax.set_title('S033 Online Order Insertion — Cheapest Insertion')

    # Depot
    ax.plot(DEPOT[0], DEPOT[1], 'k*', markersize=12, zorder=5)

    # Order locations
    for o in sim_orders:
        ax.plot(o.loc[0], o.loc[1], 'p', color='purple', markersize=8, alpha=0.6, zorder=4)

    dots   = [ax.plot([], [], 'o', color=DRONE_COLORS[k], markersize=8, zorder=6)[0]
              for k in range(K)]
    trails = [ax.plot([], [], '-', color=DRONE_COLORS[k], linewidth=1.2, alpha=0.6)[0]
              for k in range(K)]
    time_txt = ax.text(0.02, 0.97, '', transform=ax.transAxes,
                       fontsize=10, verticalalignment='top')

    def init():
        for dot, trail in zip(dots, trails):
            dot.set_data([], [])
            trail.set_data([], [])
        time_txt.set_text('')
        return dots + trails + [time_txt]

    def update(fi):
        real_fi = fi * step_ani
        for k in range(K):
            hist = pos_history[k]
            idx  = min(real_fi, len(hist) - 1)
            px   = [h[0] for h in hist[:idx+1]]
            py   = [h[1] for h in hist[:idx+1]]
            dots[k].set_data([px[-1]] if px else [], [py[-1]] if py else [])
            trail_start = max(0, idx - 60)
            trails[k].set_data(px[trail_start:], py[trail_start:])
        sim_t = real_fi * STEP * DT
        time_txt.set_text(f't = {sim_t:.1f} s')
        return dots + trails + [time_txt]

    ani = animation.FuncAnimation(fig, update, frames=n_frames,
                                  init_func=init, blit=True, interval=50)
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer='pillow', fps=20, dpi=100)
    plt.close()
    print(f'Saved: {path}')


# ── Main ───────────────────────────────────────

if __name__ == '__main__':
    print('Running all strategies...')
    all_results = run_all_strategies()

    print('\n=== S033 Online Order Insertion — Results ===')
    for s in STRATEGIES:
        m = all_results[s]
        feas = m['feasible']
        queued = m['queued']
        rate = feas / N_INSERTIONS * 100
        print(f"  {s:25s}  detour={m['total_detour']:7.1f} m  "
              f"served={feas}/{N_INSERTIONS}  queued={queued}  "
              f"feasibility={rate:.0f}%")

    ci = all_results['cheapest_insertion']
    nd = all_results['nearest_drone']
    saving = nd['total_detour'] - ci['total_detour']
    print(f"\n  Cheapest Insertion saves {saving:.1f} m vs Nearest Drone")
    print(f"  Per-drone load (cheapest): {ci['per_drone_inserted']}")

    out_dir = os.path.normpath(OUTPUT_DIR)
    plot_route_map(ci, out_dir)
    plot_battery_timeline(ci, out_dir)
    plot_strategy_comparison(all_results, out_dir)
    plot_load_balance(all_results, out_dir)
    save_animation(ci, out_dir)
