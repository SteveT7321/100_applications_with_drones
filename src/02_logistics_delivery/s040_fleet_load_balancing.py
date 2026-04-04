"""
S040 Fleet Load Balancing
=========================
Simulates a fleet of 8 delivery drones operating from a single depot over a
600x600 m urban grid. A stream of 40 delivery orders arrives dynamically over
a 400 s horizon. Three dispatch strategies are compared: Round-Robin, Least-
Queue, and a composite Load-Index Balancer with periodic queue-swap rebalancing.
Key metrics include fleet imbalance (Lambda), Jain's fairness index, and total
completed deliveries. Outputs include imbalance time series, fairness bar chart,
per-drone delivery histogram, cumulative deliveries, Gantt chart, load heatmap,
and a rebalancing events scatter plot.

Usage:
    conda activate drones
    python src/02_logistics_delivery/s040_fleet_load_balancing.py
"""

import sys, os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from collections import deque
from dataclasses import dataclass, field
from typing import List, Optional

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

# ── Parameters ─────────────────────────────────────────────────────────────────
N_DRONES      = 8
N_PADS        = 2
M_ORDERS      = 40
T_SIM         = 400.0        # s
DT            = 0.2          # s simulation timestep
REBAL_PERIOD  = 20.0         # s rebalancing epoch interval
S_MAX_SWAPS   = 2            # max queue swaps per rebalancing epoch

V_DRONE       = 8.0          # m/s cruise speed
M_BODY        = 1.5          # kg
M_PAYLOAD     = 0.3          # kg
G             = 9.81         # m/s²
E_CAP         = 1.0          # normalised battery capacity
ALPHA         = 1.5e-4       # discharge coefficient (1/m)
P_CHG         = 0.20         # charge rate (E_cap/s)
BETA_SAFE     = 0.05         # safety SoC reserve
D_RANGE       = E_CAP / (ALPHA * (M_BODY + M_PAYLOAD) * G)  # ~3560 m
F_NORM        = D_RANGE * 0.5  # normalisation for flight distance

# Load-index weights (sum = 1)
PHI_W = 0.5    # queue depth weight
PHI_F = 0.3    # cumulative distance weight
PHI_S = 0.2    # battery depletion weight

DEPOT = np.array([300.0, 300.0, 2.0])
GRID  = (0.0, 600.0)

STRATEGIES = ['round_robin', 'least_queue', 'load_index']

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '02_logistics_delivery', 's040_fleet_load_balancing',
)

RNG = np.random.default_rng(0)   # fixed seed for reproducibility


# ── Data Classes ───────────────────────────────────────────────────────────────
@dataclass
class Order:
    idx: int
    location: np.ndarray
    t_arrive: float


@dataclass
class Drone:
    idx: int
    pos: np.ndarray
    soc: float = 1.0
    queue: deque = field(default_factory=deque)
    deliveries: int = 0
    flight_dist: float = 0.0
    state: str = 'idle'   # idle | flying | returning | charging | waiting
    target: Optional[np.ndarray] = None
    current_order: Optional[Order] = None

    def load_index(self) -> float:
        return (PHI_W * len(self.queue)
                + PHI_F * (self.flight_dist / F_NORM)
                + PHI_S * (1.0 - self.soc))

    def remaining_queue_distance(self) -> float:
        pts = [self.pos if self.target is None else self.target]
        for o in self.queue:
            pts.append(o.location)
        pts.append(DEPOT)
        return sum(np.linalg.norm(pts[k+1] - pts[k]) for k in range(len(pts)-1))

    def can_accept(self, order: Order) -> bool:
        if self.queue:
            last = self.queue[-1].location
        elif self.target is not None:
            last = self.target
        else:
            last = self.pos
        extra = np.linalg.norm(order.location - last)
        projected_dist = self.remaining_queue_distance() + extra
        projected_soc  = self.soc - projected_dist * ALPHA * (M_BODY + M_PAYLOAD) * G
        return projected_soc >= BETA_SAFE


# ── Helpers ────────────────────────────────────────────────────────────────────
def marginal_load(drone: Drone, order: Order) -> float:
    last = (drone.queue[-1].location if drone.queue else
            (drone.target if drone.target is not None else drone.pos))
    extra_dist = np.linalg.norm(order.location - last)
    return PHI_W * 1.0 + PHI_F * extra_dist / F_NORM


def assign_load_index(drones: List[Drone], order: Order) -> Optional[int]:
    best_k, best_score = None, float('inf')
    for drone in drones:
        if drone.state in ('returning', 'charging', 'waiting'):
            continue
        if not drone.can_accept(order):
            continue
        score = drone.load_index() + marginal_load(drone, order)
        if score < best_score:
            best_score = score
            best_k = drone.idx
    return best_k


def rebalance(drones: List[Drone]) -> int:
    swaps = 0
    for _ in range(S_MAX_SWAPS):
        active = [d for d in drones if d.queue]
        if len(active) < 2:
            break
        most  = max(active, key=lambda d: d.load_index())
        least = min(drones,  key=lambda d: d.load_index())
        if most.idx == least.idx:
            break
        candidate = most.queue[-1]
        imbal_before = (max(d.load_index() for d in drones)
                        - min(d.load_index() for d in drones))
        most.queue.pop()
        least.queue.append(candidate)
        imbal_after = (max(d.load_index() for d in drones)
                       - min(d.load_index() for d in drones))
        if imbal_after >= imbal_before or not least.can_accept(candidate):
            least.queue.pop()
            most.queue.append(candidate)
            break
        swaps += 1
    return swaps


def fleet_imbalance(drones: List[Drone]) -> float:
    loads = [d.load_index() for d in drones]
    mean  = np.mean(loads)
    if mean == 0:
        return 0.0
    return (max(loads) - min(loads)) / mean


def jain_fairness(drones: List[Drone]) -> float:
    counts = np.array([d.deliveries for d in drones], dtype=float)
    if counts.sum() == 0:
        return 1.0
    return counts.sum()**2 / (N_DRONES * (counts**2).sum())


# ── Simulation ─────────────────────────────────────────────────────────────────
def run_simulation(strategy: str = 'load_index', seed: int = 42) -> dict:
    rng    = np.random.default_rng(seed)
    drones = [Drone(idx=i, pos=DEPOT.copy()) for i in range(N_DRONES)]
    pads        = [None] * N_PADS
    charge_rem  = [0.0]  * N_PADS
    wait_queue  = deque()

    # Pre-generate order stream
    orders = [
        Order(idx=j,
              location=np.array([rng.uniform(*GRID), rng.uniform(*GRID), 2.0]),
              t_arrive=rng.uniform(0, T_SIM * 0.8))
        for j in range(M_ORDERS)
    ]
    orders.sort(key=lambda o: o.t_arrive)
    order_stream = deque(orders)

    imbalance_log   = []   # (t, Lambda)
    rebal_log       = []   # (t, n_swaps)
    cumulative_log  = []   # (t, total_deliveries)
    state_log       = {i: [] for i in range(N_DRONES)}  # drone_idx -> list of (t, state)
    load_snapshots  = []   # (t, [L_0..L_7])

    t          = 0.0
    next_rebal = REBAL_PERIOD
    rr_counter = 0   # round-robin pointer

    while t <= T_SIM:
        # --- Inject arriving orders ---
        while order_stream and order_stream[0].t_arrive <= t:
            order = order_stream.popleft()
            if strategy == 'load_index':
                k = assign_load_index(drones, order)
            elif strategy == 'least_queue':
                k = min(range(N_DRONES), key=lambda i: len(drones[i].queue))
            elif strategy == 'round_robin':
                k = rr_counter % N_DRONES
                rr_counter += 1
            else:
                k = assign_load_index(drones, order)
            if k is not None:
                drones[k].queue.append(order)

        # --- Rebalancing epoch (load_index only) ---
        if t >= next_rebal and strategy == 'load_index':
            swaps = rebalance(drones)
            rebal_log.append((t, swaps))
            next_rebal += REBAL_PERIOD

        # --- Step each drone ---
        for d in drones:
            if d.state == 'idle' and d.queue:
                d.current_order = d.queue.popleft()
                d.target = d.current_order.location.copy()
                d.state  = 'flying'

            if d.state == 'flying' and d.target is not None:
                to_target = d.target - d.pos
                dist_rem  = np.linalg.norm(to_target)
                step_dist = V_DRONE * DT
                if dist_rem <= step_dist:
                    d.pos = d.target.copy()
                    d.soc -= ALPHA * (M_BODY + M_PAYLOAD) * G * dist_rem
                    d.soc  = max(d.soc, 0.0)
                    d.flight_dist += dist_rem
                    d.deliveries  += 1
                    d.current_order = None
                    d.target = None
                    dist_home = np.linalg.norm(d.pos - DEPOT)
                    if d.soc <= ALPHA * (M_BODY + M_PAYLOAD) * G * dist_home + BETA_SAFE + 0.1:
                        d.state  = 'returning'
                        d.target = DEPOT.copy()
                    else:
                        d.state = 'idle'
                else:
                    direction = to_target / dist_rem
                    d.pos     += direction * step_dist
                    d.soc     -= ALPHA * (M_BODY + M_PAYLOAD) * G * step_dist
                    d.soc      = max(d.soc, 0.0)
                    d.flight_dist += step_dist
                    dist_home = np.linalg.norm(d.pos - DEPOT)
                    if d.soc <= ALPHA * (M_BODY + M_PAYLOAD) * G * dist_home + BETA_SAFE:
                        d.state  = 'returning'
                        d.target = DEPOT.copy()

            elif d.state == 'returning':
                to_base   = DEPOT - d.pos
                dist_rem  = np.linalg.norm(to_base)
                step_dist = V_DRONE * DT
                if dist_rem <= step_dist:
                    d.pos  = DEPOT.copy()
                    d.soc -= ALPHA * (M_BODY + M_PAYLOAD) * G * dist_rem
                    d.soc  = max(d.soc, 0.0)
                    d.flight_dist += dist_rem
                    free = next((k for k, v in enumerate(pads) if v is None), None)
                    if free is not None:
                        pads[free] = d.idx
                        charge_rem[free] = (1.0 - d.soc) / P_CHG
                        d.state = 'charging'
                    else:
                        wait_queue.append(d.idx)
                        d.state = 'waiting'
                else:
                    d.pos     += (to_base / dist_rem) * step_dist
                    d.soc     -= ALPHA * (M_BODY + M_PAYLOAD) * G * step_dist
                    d.soc      = max(d.soc, 0.0)
                    d.flight_dist += step_dist

        # --- Advance charging pads ---
        for k in range(N_PADS):
            if pads[k] is not None:
                charge_rem[k] -= DT
                d = drones[pads[k]]
                d.soc = min(1.0, d.soc + P_CHG * DT)
                if charge_rem[k] <= 0:
                    d.soc   = 1.0
                    d.state = 'idle'
                    pads[k] = None
                    if wait_queue:
                        nxt = wait_queue.popleft()
                        pads[k] = nxt
                        nd = drones[nxt]
                        charge_rem[k] = (1.0 - nd.soc) / P_CHG
                        nd.state = 'charging'

        # --- Log ---
        imbalance_log.append((t, fleet_imbalance(drones)))
        cumulative_log.append((t, sum(d.deliveries for d in drones)))
        load_snapshots.append([d.load_index() for d in drones])
        for d in drones:
            state_log[d.idx].append(d.state)

        t = round(t + DT, 6)

    times = np.array([v[0] for v in imbalance_log])

    return {
        'strategy':          strategy,
        'total_deliveries':  sum(d.deliveries for d in drones),
        'jain_fairness':     jain_fairness(drones),
        'mean_imbalance':    float(np.mean([v[1] for v in imbalance_log])),
        'per_drone_deliveries': [d.deliveries for d in drones],
        'imbalance_log':     imbalance_log,
        'rebal_log':         rebal_log,
        'cumulative_log':    cumulative_log,
        'load_snapshots':    np.array(load_snapshots),  # shape (T_steps, N_DRONES)
        'state_log':         state_log,
        'times':             times,
    }


def run_all_simulations():
    results = {}
    for strat in STRATEGIES:
        print(f'Running strategy: {strat} ...')
        results[strat] = run_simulation(strategy=strat, seed=42)
    return results


# ── Plots ───────────────────────────────────────────────────────────────────────
COLORS = {
    'round_robin': '#e74c3c',
    'least_queue': '#3498db',
    'load_index':  '#2ecc71',
}
LABELS = {
    'round_robin': 'Round-Robin',
    'least_queue': 'Least-Queue',
    'load_index':  'Load-Index Balancer',
}


def plot_imbalance_timeseries(results: dict, out_dir: str):
    fig, ax = plt.subplots(figsize=(10, 5))
    for strat, res in results.items():
        times = res['times']
        vals  = [v[1] for v in res['imbalance_log']]
        ax.plot(times, vals, color=COLORS[strat], alpha=0.7,
                linewidth=1.0, label=LABELS[strat])

    # Shade rebalancing epochs for load_index
    for (t_r, _) in results['load_index']['rebal_log']:
        ax.axvspan(t_r, t_r + REBAL_PERIOD, alpha=0.07, color='#2ecc71')

    ax.set_xlabel('Time (s)')
    ax.set_ylabel(r'Fleet Imbalance $\Lambda(t)$')
    ax.set_title('Fleet Load Imbalance Over Time')
    ax.legend()
    ax.set_xlim(0, T_SIM)
    ax.set_ylim(bottom=0)
    ax.grid(alpha=0.3)
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'imbalance_timeseries.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def plot_jain_fairness(results: dict, out_dir: str):
    fig, ax = plt.subplots(figsize=(6, 4))
    strats = list(results.keys())
    vals   = [results[s]['jain_fairness'] for s in strats]
    colors = [COLORS[s] for s in strats]
    labels = [LABELS[s] for s in strats]
    bars   = ax.bar(labels, vals, color=colors, edgecolor='black', linewidth=0.5)
    for bar, v in zip(bars, vals):
        ax.text(bar.get_x() + bar.get_width() / 2, v + 0.005,
                f'{v:.3f}', ha='center', va='bottom', fontsize=9)
    ax.axhline(1.0, color='gray', linestyle='--', linewidth=0.8, label='Perfect fairness')
    ax.set_ylabel("Jain's Fairness Index $\\mathcal{J}$")
    ax.set_title("Jain's Fairness Index by Strategy")
    ax.set_ylim(0, 1.15)
    ax.legend()
    ax.grid(axis='y', alpha=0.3)
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'jain_fairness.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_per_drone_deliveries(results: dict, out_dir: str):
    fig, ax = plt.subplots(figsize=(10, 5))
    n_strats  = len(STRATEGIES)
    width     = 0.25
    x         = np.arange(N_DRONES)
    for idx, strat in enumerate(STRATEGIES):
        counts = results[strat]['per_drone_deliveries']
        offset = (idx - 1) * width
        ax.bar(x + offset, counts, width=width, color=COLORS[strat],
               label=LABELS[strat], edgecolor='black', linewidth=0.4, alpha=0.85)
    ax.set_xlabel('Drone Index')
    ax.set_ylabel('Completed Deliveries')
    ax.set_title('Per-Drone Delivery Count by Strategy')
    ax.set_xticks(x)
    ax.set_xticklabels([f'D{i}' for i in range(N_DRONES)])
    ax.legend()
    ax.grid(axis='y', alpha=0.3)
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'per_drone_deliveries.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_cumulative_deliveries(results: dict, out_dir: str):
    fig, ax = plt.subplots(figsize=(10, 5))
    for strat, res in results.items():
        times = [v[0] for v in res['cumulative_log']]
        vals  = [v[1] for v in res['cumulative_log']]
        ax.plot(times, vals, color=COLORS[strat], linewidth=1.8,
                label=LABELS[strat])
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Cumulative Completed Deliveries')
    ax.set_title('Cumulative Deliveries Over Time')
    ax.set_xlim(0, T_SIM)
    ax.set_ylim(bottom=0)
    ax.legend()
    ax.grid(alpha=0.3)
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'cumulative_deliveries.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def plot_gantt_chart(results: dict, out_dir: str):
    """Fleet state Gantt chart for the Load-Index Balancer."""
    res = results['load_index']
    state_log = res['state_log']
    times     = res['times']

    STATE_COLORS = {
        'flying':    '#2ecc71',
        'returning': '#e67e22',
        'charging':  '#3498db',
        'waiting':   '#9b59b6',
        'idle':      '#ecf0f1',
    }
    state_list = list(STATE_COLORS.keys())

    fig, ax = plt.subplots(figsize=(12, 5))
    dt = DT
    for drone_idx in range(N_DRONES):
        states_ts = state_log[drone_idx]
        for t_idx, state in enumerate(states_ts):
            t = times[t_idx]
            color = STATE_COLORS.get(state, '#bdc3c7')
            ax.barh(drone_idx, dt, left=t, height=0.6,
                    color=color, edgecolor='none')

    ax.set_yticks(range(N_DRONES))
    ax.set_yticklabels([f'Drone {i}' for i in range(N_DRONES)])
    ax.set_xlabel('Time (s)')
    ax.set_title('Fleet State Gantt Chart — Load-Index Balancer')
    ax.set_xlim(0, T_SIM)
    patches = [mpatches.Patch(color=c, label=s.capitalize())
               for s, c in STATE_COLORS.items()]
    ax.legend(handles=patches, loc='upper right', fontsize=8, ncol=2)
    ax.grid(axis='x', alpha=0.3)
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'gantt_chart.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_load_heatmap(results: dict, out_dir: str):
    """Load index heatmap (drones x time) for all strategies."""
    fig, axes = plt.subplots(1, 3, figsize=(15, 4), sharey=True)
    for ax, strat in zip(axes, STRATEGIES):
        snapshots = results[strat]['load_snapshots']  # (T_steps, N_DRONES)
        times     = results[strat]['times']
        # Downsample for display
        step = max(1, len(times) // 300)
        snap_ds = snapshots[::step].T  # (N_DRONES, T_display)
        t_ds    = times[::step]
        im = ax.imshow(snap_ds, aspect='auto', origin='lower',
                       extent=[0, T_SIM, -0.5, N_DRONES - 0.5],
                       cmap='YlOrRd', vmin=0, vmax=2.5)
        ax.set_title(LABELS[strat])
        ax.set_xlabel('Time (s)')
        ax.set_yticks(range(N_DRONES))
        ax.set_yticklabels([f'D{i}' for i in range(N_DRONES)], fontsize=7)
    axes[0].set_ylabel('Drone Index')
    plt.colorbar(im, ax=axes.tolist(), label='Load Index $L_i(t)$', shrink=0.8)
    fig.suptitle('Load Index Heatmap (Drones × Time)', fontsize=12)
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'load_heatmap.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_rebalancing_events(results: dict, out_dir: str):
    """Scatter plot of swap counts per epoch vs time for Load-Index Balancer."""
    res       = results['load_index']
    rebal_log = res['rebal_log']
    imb_log   = res['imbalance_log']

    if not rebal_log:
        print('No rebalancing events to plot.')
        return

    t_rebal   = np.array([v[0] for v in rebal_log])
    n_swaps   = np.array([v[1] for v in rebal_log])

    # Compute imbalance just before and after each rebalancing epoch
    times_arr = np.array([v[0] for v in imb_log])
    imb_arr   = np.array([v[1] for v in imb_log])

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)

    # Top: imbalance curve
    ax1.plot(times_arr, imb_arr, color='#2ecc71', linewidth=1.2)
    for t_r in t_rebal:
        ax1.axvline(t_r, color='#e74c3c', linewidth=0.6, alpha=0.5)
    ax1.set_ylabel(r'$\Lambda(t)$')
    ax1.set_title('Load-Index Balancer: Imbalance & Rebalancing Events')
    ax1.set_ylim(bottom=0)
    ax1.grid(alpha=0.3)

    # Bottom: swap counts
    ax2.scatter(t_rebal, n_swaps, c='#e74c3c', s=40, zorder=3, label='Swaps executed')
    ax2.bar(t_rebal, n_swaps, width=3, color='#e74c3c', alpha=0.4)
    for x, y in zip(t_rebal, n_swaps):
        if y > 0:
            ax2.annotate(str(int(y)), (x, y), textcoords='offset points',
                         xytext=(3, 3), fontsize=7)
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Queue Swaps')
    ax2.set_ylim(-0.3, S_MAX_SWAPS + 0.5)
    ax2.set_xlim(0, T_SIM)
    ax2.legend()
    ax2.grid(alpha=0.3)

    plt.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'rebalancing_events.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def save_animation(results: dict, out_dir: str):
    """Animate drone positions and delivery counts over time (Load-Index Balancer).

    We re-run the simulation once more but capture drone positions at each step.
    To keep the GIF small, frames are decimated.
    """
    import matplotlib.animation as animation

    # Re-run to collect position history
    rng_anim = np.random.default_rng(42)
    orders_anim = [
        Order(idx=j,
              location=np.array([rng_anim.uniform(*GRID), rng_anim.uniform(*GRID), 2.0]),
              t_arrive=rng_anim.uniform(0, T_SIM * 0.8))
        for j in range(M_ORDERS)
    ]
    orders_anim.sort(key=lambda o: o.t_arrive)

    drones_a   = [Drone(idx=i, pos=DEPOT.copy()) for i in range(N_DRONES)]
    pads_a     = [None] * N_PADS
    crm_a      = [0.0]  * N_PADS
    wq_a       = deque()
    ostream    = deque(orders_anim)

    pos_history  = []   # list of (N_DRONES, 2) arrays
    soc_history  = []
    state_history= []
    t = 0.0
    next_rebal_a = REBAL_PERIOD
    rr_a = 0

    while t <= T_SIM:
        while ostream and ostream[0].t_arrive <= t:
            order = ostream.popleft()
            k = assign_load_index(drones_a, order)
            if k is not None:
                drones_a[k].queue.append(order)

        if t >= next_rebal_a:
            rebalance(drones_a)
            next_rebal_a += REBAL_PERIOD

        for d in drones_a:
            if d.state == 'idle' and d.queue:
                d.current_order = d.queue.popleft()
                d.target = d.current_order.location.copy()
                d.state  = 'flying'
            if d.state == 'flying' and d.target is not None:
                to_t = d.target - d.pos
                dr   = np.linalg.norm(to_t)
                sd   = V_DRONE * DT
                if dr <= sd:
                    d.pos = d.target.copy()
                    d.soc -= ALPHA * (M_BODY + M_PAYLOAD) * G * dr
                    d.soc  = max(d.soc, 0.0)
                    d.flight_dist += dr
                    d.deliveries  += 1
                    d.current_order = None; d.target = None
                    dh = np.linalg.norm(d.pos - DEPOT)
                    if d.soc <= ALPHA * (M_BODY + M_PAYLOAD) * G * dh + BETA_SAFE + 0.1:
                        d.state = 'returning'; d.target = DEPOT.copy()
                    else:
                        d.state = 'idle'
                else:
                    d.pos += (to_t / dr) * sd
                    d.soc -= ALPHA * (M_BODY + M_PAYLOAD) * G * sd
                    d.soc  = max(d.soc, 0.0)
                    d.flight_dist += sd
                    dh = np.linalg.norm(d.pos - DEPOT)
                    if d.soc <= ALPHA * (M_BODY + M_PAYLOAD) * G * dh + BETA_SAFE:
                        d.state = 'returning'; d.target = DEPOT.copy()
            elif d.state == 'returning':
                to_b = DEPOT - d.pos
                dr   = np.linalg.norm(to_b)
                sd   = V_DRONE * DT
                if dr <= sd:
                    d.pos = DEPOT.copy()
                    d.soc -= ALPHA * (M_BODY + M_PAYLOAD) * G * dr
                    d.soc  = max(d.soc, 0.0)
                    d.flight_dist += dr
                    free = next((kk for kk, v in enumerate(pads_a) if v is None), None)
                    if free is not None:
                        pads_a[free] = d.idx; crm_a[free] = (1.0 - d.soc) / P_CHG
                        d.state = 'charging'
                    else:
                        wq_a.append(d.idx); d.state = 'waiting'
                else:
                    d.pos += (to_b / dr) * sd
                    d.soc -= ALPHA * (M_BODY + M_PAYLOAD) * G * sd
                    d.soc  = max(d.soc, 0.0)
                    d.flight_dist += sd

        for kk in range(N_PADS):
            if pads_a[kk] is not None:
                crm_a[kk] -= DT
                d = drones_a[pads_a[kk]]
                d.soc = min(1.0, d.soc + P_CHG * DT)
                if crm_a[kk] <= 0:
                    d.soc = 1.0; d.state = 'idle'; pads_a[kk] = None
                    if wq_a:
                        nxt = wq_a.popleft(); pads_a[kk] = nxt
                        nd = drones_a[nxt]; crm_a[kk] = (1.0 - nd.soc) / P_CHG
                        nd.state = 'charging'

        pos_history.append(np.array([d.pos[:2].copy() for d in drones_a]))
        soc_history.append([d.soc for d in drones_a])
        state_history.append([d.state for d in drones_a])
        t = round(t + DT, 6)

    # Decimate frames
    STEP = 10
    frames_pos   = pos_history[::STEP]
    frames_soc   = soc_history[::STEP]
    frames_state = state_history[::STEP]
    n_frames = len(frames_pos)

    STATE_COLORS_ANIM = {
        'flying':    '#2ecc71',
        'returning': '#e67e22',
        'charging':  '#3498db',
        'waiting':   '#9b59b6',
        'idle':      '#95a5a6',
    }

    fig, ax = plt.subplots(figsize=(7, 7))
    ax.set_xlim(-20, 620)
    ax.set_ylim(-20, 620)
    ax.set_aspect('equal')
    ax.set_title('Fleet Load Balancing — Load-Index Strategy')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')

    # Depot
    ax.plot(300, 300, 's', markersize=12, color='#34495e', zorder=5, label='Depot')

    drone_dots  = [ax.plot([], [], 'o', markersize=8, zorder=4)[0]
                   for _ in range(N_DRONES)]
    drone_texts = [ax.text(0, 0, '', fontsize=6, ha='center', va='bottom', zorder=5)
                   for _ in range(N_DRONES)]
    time_text   = ax.text(0.02, 0.97, '', transform=ax.transAxes,
                          fontsize=9, va='top')

    patches_legend = [mpatches.Patch(color=c, label=s.capitalize())
                      for s, c in STATE_COLORS_ANIM.items()]
    ax.legend(handles=patches_legend, loc='lower right', fontsize=7)

    def init():
        for dot in drone_dots:
            dot.set_data([], [])
        for txt in drone_texts:
            txt.set_text('')
        time_text.set_text('')
        return drone_dots + drone_texts + [time_text]

    def update(frame_idx):
        pos   = frames_pos[frame_idx]
        soc   = frames_soc[frame_idx]
        state = frames_state[frame_idx]
        t_cur = frame_idx * STEP * DT
        for i in range(N_DRONES):
            color = STATE_COLORS_ANIM.get(state[i], '#bdc3c7')
            drone_dots[i].set_data([pos[i, 0]], [pos[i, 1]])
            drone_dots[i].set_color(color)
            drone_texts[i].set_position((pos[i, 0], pos[i, 1] + 10))
            drone_texts[i].set_text(f'D{i}\n{soc[i]:.2f}')
        time_text.set_text(f't = {t_cur:.0f} s')
        return drone_dots + drone_texts + [time_text]

    ani = animation.FuncAnimation(fig, update, frames=n_frames,
                                  init_func=init, blit=True, interval=50)
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer='pillow', fps=20, dpi=100)
    plt.close()
    print(f'Saved: {path}')


# ── Main ────────────────────────────────────────────────────────────────────────
if __name__ == '__main__':
    results = run_all_simulations()

    print('\n=== S040 Fleet Load Balancing — Results ===')
    for strat in STRATEGIES:
        r = results[strat]
        print(f'\n  Strategy: {LABELS[strat]}')
        print(f'    Total deliveries  : {r["total_deliveries"]}')
        print(f'    Jain fairness     : {r["jain_fairness"]:.4f}')
        print(f'    Mean imbalance    : {r["mean_imbalance"]:.4f}')
        print(f'    Per-drone counts  : {r["per_drone_deliveries"]}')

    out_dir = os.path.normpath(OUTPUT_DIR)
    plot_imbalance_timeseries(results, out_dir)
    plot_jain_fairness(results, out_dir)
    plot_per_drone_deliveries(results, out_dir)
    plot_cumulative_deliveries(results, out_dir)
    plot_gantt_chart(results, out_dir)
    plot_load_heatmap(results, out_dir)
    plot_rebalancing_events(results, out_dir)
    save_animation(results, out_dir)
