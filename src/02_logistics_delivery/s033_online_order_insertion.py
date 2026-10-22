"""
S033 Online Order Insertion
============================
A fleet of K=4 drones is executing pre-planned delivery routes (12 waypoints each,
then back to the depot) when 8 new customer orders arrive between t = 10 s and 80 s.
Each order has a deadline = arrival time + nearest-drone ETA + 30 s. The dispatcher
splices every order into one drone's remaining route, subject to battery range and
deadline feasibility. If no drone can meet the deadline, the order is queued: it is
appended to the drone that finishes its route earliest (served late) if that drone has
the battery for it, otherwise it is rejected.

Strategies compared (same fleet and orders for every strategy):
  cheapest_insertion - global minimum detour over all feasible drones / positions
  nearest_drone      - feasible drone whose current position is closest to the order
  least_loaded       - feasible drone with the fewest remaining waypoints
  random_feasible    - uniform random feasible drone

The simulation runs until every drone is back at the depot. Results are reported for
the seed-0 instance (plots) and averaged over N_SEEDS instances.

Usage:
    conda activate drones
    python src/02_logistics_delivery/s033_online_order_insertion.py
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.lines as mlines
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

# ── Parameters ─────────────────────────────────
K              = 4          # number of drones
V_DRONE        = 8.0        # m/s cruise speed
B_FULL         = 10_000.0   # m  full-battery range equivalent
B_INIT_LO      = 0.75       # initial battery fraction ~ U(B_INIT_LO, 1.0)
DT             = 0.1        # s  simulation timestep
T_CAP          = 3000.0     # s  safety cap (all runs finish well before this)
N_WP_INIT      = 12         # pre-planned waypoints per drone at t=0
N_INSERTIONS   = 8          # online orders arriving mid-mission
T_ORDER_LO     = 10.0       # s  earliest order arrival
T_ORDER_HI     = 80.0       # s  latest order arrival
DEADLINE_SLACK = 30.0       # s  deadline = t_arrive + nearest-drone ETA + slack
ARENA_LO       = -500.0     # m
ARENA_HI       =  500.0     # m
ALT_LO         = 2.0        # m
ALT_HI         = 8.0        # m
DEPOT          = np.array([0.0, 0.0, 2.0])
SEED0          = 0          # instance used for the plots
N_SEEDS        = 20         # Monte-Carlo instances (seeds 0 ... 19)

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '02_logistics_delivery', 's033_online_order_insertion',
)

DRONE_COLORS = ['tab:red', 'tab:blue', 'tab:green', 'tab:orange']
STRATEGIES   = ['cheapest_insertion', 'nearest_drone', 'least_loaded', 'random_feasible']
STRAT_COLORS = ['#2ecc71', '#3498db', '#e67e22', '#9b59b6']


# ── Helpers ────────────────────────────────────

def rand_wp(rng):
    """Random 3-D waypoint inside the arena."""
    xy = rng.uniform(ARENA_LO, ARENA_HI, 2)
    return np.array([xy[0], xy[1], rng.uniform(ALT_LO, ALT_HI)])


def route_length(pts: List[np.ndarray]) -> float:
    return float(sum(np.linalg.norm(pts[i + 1] - pts[i]) for i in range(len(pts) - 1)))


def insertion_delta(a, q, b) -> float:
    """Extra distance introduced by inserting q between a and b."""
    return float(np.linalg.norm(a - q) + np.linalg.norm(q - b) - np.linalg.norm(b - a))


def point_along(pts: List[np.ndarray], s: float) -> np.ndarray:
    """Position after flying distance s along polyline pts (clamped at the end)."""
    for i in range(len(pts) - 1):
        seg = float(np.linalg.norm(pts[i + 1] - pts[i]))
        if s <= seg:
            return pts[i] + (pts[i + 1] - pts[i]) * (s / max(seg, 1e-12))
        s -= seg
    return pts[-1].copy()


# ── Data classes ───────────────────────────────

@dataclass
class Order:
    oid: int
    loc: np.ndarray
    t_arrive: float
    t_deadline: float


@dataclass
class Drone:
    id: int
    pos: np.ndarray
    route: List[np.ndarray]          # remaining waypoints (index 0 = next)
    route_ids: List[int]             # order id per waypoint (-1 = pre-planned / depot)
    battery_rem: float               # metres of flight remaining
    speed: float = V_DRONE
    initial_route: List[np.ndarray] = field(default_factory=list)
    path: List[np.ndarray] = field(default_factory=list)
    battery_log: List[float] = field(default_factory=list)
    flown: float = 0.0

    def full_pts(self):
        return [self.pos] + self.route

    def remaining_length(self) -> float:
        return route_length(self.full_pts())

    def best_insertion(self, q, t_now, t_deadline) -> Optional[Tuple[int, float]]:
        """Cheapest (insert_idx, delta) that is battery- and deadline-feasible, or None.

        Insertion never goes after the final depot return (last waypoint)."""
        pts = self.full_pts()
        n_seg = len(pts) - 1
        if n_seg < 1:
            return None
        L = self.remaining_length()
        best_idx, best_delta = None, np.inf
        prefix = 0.0
        for idx in range(n_seg):
            if idx > 0:
                prefix += float(np.linalg.norm(pts[idx] - pts[idx - 1]))
            delta = insertion_delta(pts[idx], q, pts[idx + 1])
            t_arr = t_now + (prefix + float(np.linalg.norm(pts[idx] - q))) / self.speed
            if L + delta <= self.battery_rem and t_arr <= t_deadline and delta < best_delta:
                best_idx, best_delta = idx, delta
        return None if best_idx is None else (best_idx, best_delta)

    def insert_order(self, q, insert_idx, oid) -> float:
        """Splice q into the route. Battery is NOT pre-debited: it is consumed while flying."""
        pts = self.full_pts()
        delta = insertion_delta(pts[insert_idx], q, pts[insert_idx + 1])
        self.route.insert(insert_idx, q.copy())
        self.route_ids.insert(insert_idx, oid)
        return delta

    def step(self, dt, t, deliveries):
        """Advance toward the route with an exact distance budget; log deliveries."""
        budget = self.speed * dt
        while budget > 1e-12 and self.route:
            tgt = self.route[0]
            d = float(np.linalg.norm(tgt - self.pos))
            if d <= budget:
                self.pos = tgt.copy()
                budget -= d
                self.flown += d
                self.battery_rem -= d
                oid = self.route_ids.pop(0)
                self.route.pop(0)
                if oid >= 0:
                    deliveries[oid] = t + (self.speed * dt - budget) / self.speed
            else:
                self.pos = self.pos + (tgt - self.pos) / d * budget
                self.flown += budget
                self.battery_rem -= budget
                budget = 0.0

    @property
    def route_done(self) -> bool:
        return len(self.route) == 0


# ── Instance generation ────────────────────────

def make_instance(seed):
    """Fleet + online orders. Deadlines use the nearest drone on the *planned* routes
    at the arrival time, so they are identical for every strategy."""
    rng = np.random.default_rng(seed)
    fleet = []
    for k in range(K):
        wps = [rand_wp(rng) for _ in range(N_WP_INIT)] + [DEPOT.copy()]
        bat = B_FULL * rng.uniform(B_INIT_LO, 1.0)
        fleet.append((wps, bat))
    arrivals = np.sort(rng.uniform(T_ORDER_LO, T_ORDER_HI, N_INSERTIONS))
    orders = []
    for n, t_a in enumerate(arrivals):
        loc = rand_wp(rng)
        eta = min(np.linalg.norm(point_along([DEPOT] + wps, V_DRONE * t_a) - loc)
                  for wps, _ in fleet) / V_DRONE
        orders.append(Order(n, loc, float(t_a), float(t_a + eta + DEADLINE_SLACK)))
    return fleet, orders


def build_drones(fleet):
    drones = []
    for k, (wps, bat) in enumerate(fleet):
        d = Drone(id=k, pos=DEPOT.copy(), route=[w.copy() for w in wps],
                  route_ids=[-1] * len(wps), battery_rem=bat)
        d.initial_route = [DEPOT.copy()] + [w.copy() for w in wps]
        drones.append(d)
    return drones


# ── Dispatch strategies ────────────────────────

def _feasible(drones, order, t_now):
    out = []
    for d in drones:
        res = d.best_insertion(order.loc, t_now, order.t_deadline)
        if res is not None:
            out.append((d, res[0], res[1]))
    return out


def dispatch(strategy, drones, order, t_now, rng):
    feas = _feasible(drones, order, t_now)
    if not feas:
        return None
    if strategy == 'cheapest_insertion':
        key = lambda f: f[2]
    elif strategy == 'nearest_drone':
        key = lambda f: float(np.linalg.norm(f[0].pos - order.loc))
    elif strategy == 'least_loaded':
        key = lambda f: (len(f[0].route), f[0].id)
    else:
        return feas[int(rng.integers(len(feas)))]
    return min(feas, key=key)


def queue_fallback(drones, order):
    """Infeasible order: append before the depot return of the drone that finishes its
    route earliest among those with enough battery. Returns (drone, idx, delta) or None."""
    cands = []
    for d in drones:
        if not d.route:
            continue
        idx = len(d.route) - 1                        # slot before the final depot return
        pts = d.full_pts()
        delta = insertion_delta(pts[idx], order.loc, pts[idx + 1])
        if d.remaining_length() + delta <= d.battery_rem:
            cands.append((d.remaining_length() / d.speed, d.id, d, idx, delta))
    if not cands:
        return None
    _, _, d, idx, delta = min(cands, key=lambda c: (c[0], c[1]))
    return d, idx, delta


# ── Simulation ─────────────────────────────────

def run_simulation(strategy='cheapest_insertion', seed=SEED0, record=False):
    """Run one strategy on one instance until all drones are back at the depot."""
    fleet, orders = make_instance(seed)
    drones = build_drones(fleet)
    rng_pick = np.random.default_rng(seed + 10_000)   # random_feasible choices
    planned_len = sum(route_length(d.initial_route) for d in drones)

    pending = sorted(orders, key=lambda o: o.t_arrive)
    assign = {}                  # oid -> (drone id, mode 'online' | 'queued', delta)
    rejected = []
    deliveries = {}
    events = []                  # (t, drone id, delta, mode)
    t = 0.0
    times = []
    while (any(not d.route_done for d in drones) or pending) and t < T_CAP:
        while pending and pending[0].t_arrive <= t + 1e-9:
            o = pending.pop(0)
            res = dispatch(strategy, drones, o, t, rng_pick)
            mode = 'online'
            if res is None:
                res = queue_fallback(drones, o)
                mode = 'queued'
            if res is None:
                rejected.append(o.oid)
                continue
            d, idx, _ = res
            delta = d.insert_order(o.loc, idx, o.oid)
            assign[o.oid] = (d.id, mode, delta)
            events.append((t, d.id, delta, mode))
        if record:
            times.append(t)
            for d in drones:
                d.path.append(d.pos.copy())
                d.battery_log.append(d.battery_rem)
        for d in drones:
            d.step(DT, t, deliveries)
        t = round(t + DT, 10)
    if record:
        times.append(t)
        for d in drones:
            d.path.append(d.pos.copy())
            d.battery_log.append(d.battery_rem)

    on_time = sum(1 for o in orders if o.oid in deliveries and deliveries[o.oid] <= o.t_deadline)
    late = sum(1 for o in orders if o.oid in deliveries and deliveries[o.oid] > o.t_deadline)
    lateness = [deliveries[o.oid] - o.t_deadline for o in orders
                if o.oid in deliveries and deliveries[o.oid] > o.t_deadline]
    flown = sum(d.flown for d in drones)
    per_drone = [sum(1 for v in assign.values() if v[0] == k) for k in range(K)]
    return {
        'strategy': strategy, 'seed': seed, 'drones': drones, 'orders': orders,
        'assign': assign, 'deliveries': deliveries, 'events': events,
        'n_online': sum(1 for v in assign.values() if v[1] == 'online'),
        'n_queued': sum(1 for v in assign.values() if v[1] == 'queued'),
        'n_rejected': len(rejected), 'on_time': on_time, 'late': late,
        'mean_lateness': float(np.mean(lateness)) if lateness else 0.0,
        'detour_planned': float(sum(v[2] for v in assign.values())),
        'added_flown': flown - planned_len, 'makespan': t,
        'min_battery': min(d.battery_rem for d in drones),
        'per_drone': per_drone, 'times': np.array(times) if record else None,
    }


# ── Plots ──────────────────────────────────────

def plot_route_map(res, out_dir):
    """Planned routes (dashed) vs flown routes (solid), cheapest insertion, seed 0."""
    fig, ax = plt.subplots(figsize=(9, 9))
    for d in res['drones']:
        c = DRONE_COLORS[d.id]
        ir = np.array(d.initial_route)
        P = np.array(d.path)
        ax.plot(ir[:, 0], ir[:, 1], '--', color=c, lw=1.0, alpha=0.5)
        ax.plot(P[:, 0], P[:, 1], '-', color=c, lw=1.6, label=f'Drone {d.id + 1}')
    ax.plot(DEPOT[0], DEPOT[1], 'k*', ms=14, label='Depot', zorder=5)
    for o in res['orders']:
        did, mode = res['assign'].get(o.oid, (None, 'rejected'))[:2]
        ok = o.oid in res['deliveries'] and res['deliveries'][o.oid] <= o.t_deadline
        ax.plot(o.loc[0], o.loc[1], '*', ms=15, zorder=6,
                color='green' if ok else 'red', mec='k')
        ax.annotate(f'O{o.oid + 1}' + (f'->D{did + 1}' if did is not None else ''),
                    (o.loc[0], o.loc[1]), xytext=(6, 6), textcoords='offset points', fontsize=8)
    handles, _ = ax.get_legend_handles_labels()
    handles += [mlines.Line2D([], [], ls='--', color='grey', label='Planned route'),
                mlines.Line2D([], [], marker='*', ls='', color='green', mec='k', ms=12,
                              label='Order delivered on time'),
                mlines.Line2D([], [], marker='*', ls='', color='red', mec='k', ms=12,
                              label='Order late / rejected')]
    ax.legend(handles=handles, loc='upper left', bbox_to_anchor=(1.01, 1.0), fontsize=8)
    ax.set_xlim(ARENA_LO - 50, ARENA_HI + 50); ax.set_ylim(ARENA_LO - 50, ARENA_HI + 50)
    ax.set_aspect('equal'); ax.grid(True, ls=':', alpha=0.5)
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')
    ax.set_title('S033 Route Map - Cheapest Insertion, seed 0\n'
                 '(dashed = planned, solid = flown to completion)')
    path = os.path.join(out_dir, 'route_map.png')
    plt.savefig(path, dpi=130, bbox_inches='tight'); plt.close()
    print(f'Saved: {path}')


def plot_battery_timeline(res, out_dir):
    """Battery remaining vs time per drone with order arrival / assignment markers."""
    fig, axes = plt.subplots(1, 2, figsize=(15, 5), gridspec_kw={'width_ratios': [2, 1]})
    t = res['times']
    for ax, xmax in zip(axes, [t[-1], T_ORDER_HI + 70]):
        for d in res['drones']:
            ax.plot(t, d.battery_log, color=DRONE_COLORS[d.id], lw=1.6, label=f'Drone {d.id + 1}')
        for (te, did, delta, mode) in res['events']:
            ax.axvline(te, color=DRONE_COLORS[did], ls=':' if mode == 'online' else '--', lw=1)
        ax.set_xlim(0, xmax); ax.grid(alpha=0.3)
        ax.set_xlabel('Time (s)'); ax.set_ylabel('Battery remaining (m of range)')
    for o in res['orders']:
        axes[1].axvspan(o.t_arrive, o.t_deadline, color='grey', alpha=0.05)
    for (te, did, delta, mode) in res['events']:
        axes[1].text(te, axes[1].get_ylim()[1], f' D{did + 1} +{delta:.0f} m', rotation=90,
                     va='top', fontsize=7, color=DRONE_COLORS[did])
    axes[0].set_title('Battery over the whole mission (dotted = order assigned online)')
    axes[1].set_title('Zoom on the order window (grey = arrival-to-deadline)')
    axes[0].legend(fontsize=8)
    fig.suptitle('S033 Order timeline and battery - Cheapest Insertion, seed 0', fontsize=12)
    fig.tight_layout()
    path = os.path.join(out_dir, 'order_timeline.png')
    plt.savefig(path, dpi=130); plt.close()
    print(f'Saved: {path}')


def plot_strategy_comparison(seed0, mc, out_dir):
    """Seed-0 and Monte-Carlo detour / on-time rate per strategy."""
    labels = [s.replace('_', '\n') for s in STRATEGIES]
    x = np.arange(len(STRATEGIES))
    fig, axes = plt.subplots(1, 3, figsize=(16, 5))
    panels = [
        ('detour_planned', 'Insertion detour (m)', '{:.0f}'),
        ('on_time', f'Orders delivered on time (of {N_INSERTIONS})', '{:.2f}'),
        ('added_flown', 'Added flown distance vs plan (m)', '{:.0f}'),
    ]
    w = 0.38
    for ax, (key, title, fmt) in zip(axes, panels):
        v0 = [seed0[s][key] for s in STRATEGIES]
        vm = [np.mean([r[key] for r in mc[s]]) for s in STRATEGIES]
        sd = [np.std([r[key] for r in mc[s]]) for s in STRATEGIES]
        b1 = ax.bar(x - w / 2, v0, w, color=STRAT_COLORS, alpha=0.55, label='seed 0')
        b2 = ax.bar(x + w / 2, vm, w, yerr=sd, capsize=3, color=STRAT_COLORS,
                    label=f'mean of {N_SEEDS} seeds')
        ax.bar_label(b1, labels=[fmt.format(v) for v in v0], fontsize=7, padding=2)
        ax.bar_label(b2, labels=[fmt.format(v) for v in vm], fontsize=7, padding=2)
        ax.set_xticks(x); ax.set_xticklabels(labels, fontsize=8)
        ax.set_title(title, fontsize=10); ax.grid(axis='y', alpha=0.3)
    axes[0].legend(fontsize=8)
    fig.suptitle('S033 Strategy comparison (light = seed 0, dark = Monte-Carlo mean +/- std)',
                 fontsize=12)
    fig.tight_layout()
    path = os.path.join(out_dir, 'strategy_comparison.png')
    plt.savefig(path, dpi=130); plt.close()
    print(f'Saved: {path}')


def plot_load_balance(mc, out_dir):
    """Inserted orders per drone summed over all Monte-Carlo instances."""
    fig, axes = plt.subplots(1, len(STRATEGIES), figsize=(14, 4), sharey=True)
    for ax, s in zip(axes, STRATEGIES):
        counts = np.sum([r['per_drone'] for r in mc[s]], axis=0)
        bars = ax.bar(range(K), counts, color=DRONE_COLORS)
        ax.bar_label(bars, fontsize=9)
        ax.set_xticks(range(K)); ax.set_xticklabels([f'D{i + 1}' for i in range(K)])
        ax.set_title(s.replace('_', ' ').title(), fontsize=10); ax.grid(axis='y', alpha=0.3)
    axes[0].set_ylabel(f'Orders assigned (sum over {N_SEEDS} seeds)')
    fig.suptitle('S033 Load balance per strategy', fontsize=12)
    fig.tight_layout()
    path = os.path.join(out_dir, 'load_balance.png')
    plt.savefig(path, dpi=130); plt.close()
    print(f'Saved: {path}')


def save_animation(res, out_dir):
    """Top-down animation of the cheapest-insertion run (seed 0), <= 150 frames."""
    import matplotlib.animation as animation
    t = res['times']
    n = len(t)
    frames = list(np.linspace(0, n - 1, 150).astype(int))
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.set_xlim(ARENA_LO - 50, ARENA_HI + 50); ax.set_ylim(ARENA_LO - 50, ARENA_HI + 50)
    ax.set_aspect('equal'); ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')
    ax.plot(DEPOT[0], DEPOT[1], 'k*', ms=12, zorder=5)
    for d in res['drones']:
        ir = np.array(d.initial_route)
        ax.plot(ir[:, 0], ir[:, 1], '--', color=DRONE_COLORS[d.id], lw=0.6, alpha=0.4)
    order_marks = [ax.plot([], [], '*', ms=13, color='purple', mec='k', zorder=6)[0]
                   for _ in res['orders']]
    dots = [ax.plot([], [], 'o', color=DRONE_COLORS[k], ms=8, zorder=7)[0] for k in range(K)]
    trails = [ax.plot([], [], '-', color=DRONE_COLORS[k], lw=1.3)[0] for k in range(K)]
    txt = ax.text(0.02, 0.97, '', transform=ax.transAxes, va='top', fontsize=10)
    ax.set_title('S033 Online Order Insertion - Cheapest Insertion (seed 0)')
    paths = [np.array(d.path) for d in res['drones']]

    def update(fi):
        f = frames[fi]
        for k in range(K):
            dots[k].set_data([paths[k][f, 0]], [paths[k][f, 1]])
            trails[k].set_data(paths[k][:f + 1, 0], paths[k][:f + 1, 1])
        for o, m in zip(res['orders'], order_marks):
            if o.t_arrive <= t[f]:
                done = o.oid in res['deliveries'] and res['deliveries'][o.oid] <= t[f]
                m.set_data([o.loc[0]], [o.loc[1]])
                m.set_color('lightgrey' if done else 'purple')
        txt.set_text(f't = {t[f]:.0f} s')
        return dots + trails + order_marks + [txt]

    ani = animation.FuncAnimation(fig, update, frames=len(frames), interval=50, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer='pillow', fps=20, dpi=80)
    plt.close()
    print(f'Saved: {path}  ({len(frames)} frames)')


# ── Main ───────────────────────────────────────

if __name__ == '__main__':
    out_dir = os.path.normpath(OUTPUT_DIR)
    os.makedirs(out_dir, exist_ok=True)

    seed0 = {s: run_simulation(s, SEED0, record=(s == 'cheapest_insertion')) for s in STRATEGIES}
    fleet, orders = make_instance(SEED0)
    print('Seed-0 instance:')
    for k, (wps, bat) in enumerate(fleet):
        print(f'  D{k + 1}: planned route {route_length([DEPOT] + wps):7.1f} m, '
              f'battery {bat:7.1f} m')
    for o in orders:
        print(f'  O{o.oid + 1}: t_arrive={o.t_arrive:5.1f} s  deadline={o.t_deadline:6.1f} s')

    print('\n=== Seed 0 ===')
    for s in STRATEGIES:
        r = seed0[s]
        print(f"  {s:20s} detour={r['detour_planned']:7.1f} m  added_flown={r['added_flown']:7.1f} m  "
              f"online={r['n_online']} queued={r['n_queued']} rejected={r['n_rejected']}  "
              f"on_time={r['on_time']}/{N_INSERTIONS} late={r['late']}  "
              f"makespan={r['makespan']:6.1f} s  min_battery={r['min_battery']:7.1f} m  "
              f"per_drone={r['per_drone']}")

    mc = {s: [run_simulation(s, SEED0 + i) for i in range(N_SEEDS)] for s in STRATEGIES}
    print(f'\n=== Mean over {N_SEEDS} seeds ===')
    for s in STRATEGIES:
        rs = mc[s]
        f = lambda k: (np.mean([r[k] for r in rs]), np.std([r[k] for r in rs]))
        print(f"  {s:20s} detour={f('detour_planned')[0]:7.1f} +/- {f('detour_planned')[1]:6.1f} m  "
              f"added_flown={f('added_flown')[0]:7.1f} m  on_time={f('on_time')[0]:.2f}/{N_INSERTIONS}  "
              f"queued={f('n_queued')[0]:.2f}  rejected={f('n_rejected')[0]:.2f}  "
              f"late={f('late')[0]:.2f}  makespan={f('makespan')[0]:6.1f} s")
    ci = np.array([r['detour_planned'] for r in mc['cheapest_insertion']])
    for s in STRATEGIES[1:]:
        o = np.array([r['detour_planned'] for r in mc[s]])
        print(f'  cheapest detour <= {s} in {int(np.sum(ci <= o + 1e-9))}/{N_SEEDS} seeds')

    plot_route_map(seed0['cheapest_insertion'], out_dir)
    plot_battery_timeline(seed0['cheapest_insertion'], out_dir)
    plot_strategy_comparison(seed0, mc, out_dir)
    plot_load_balance(mc, out_dir)
    save_animation(seed0['cheapest_insertion'], out_dir)
