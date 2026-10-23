"""
S040 Fleet Load Balancing
=========================
8 delivery drones work from a single depot (2 charging pads) over a 600 x 600 m
grid (2-D model). 40 orders arrive at uniformly random times in [0, 320] s at
uniformly random locations; the horizon is 400 s. As in the card, each drone
works through a private FIFO queue, flying directly from one delivery point to
the next, and returns to the depot only to recharge.

Dispatch strategies
  round_robin : order j goes to drone j mod 8, regardless of drone state.
  least_queue : order goes to the drone with the fewest unfinished orders
                (queued + in progress); ties -> lowest index; ignores battery,
                charging state and distance.
  load_index  : card's balancer. Only drones that are not returning/charging/
                waiting and pass the battery-feasibility check are eligible;
                the order goes to argmin L_i + dL_i with
                L_i = 0.5 w_i + 0.3 f_i/F_norm + 0.2 (1 - s_i). Orders with no
                eligible drone wait in a central backlog and are retried every
                step; every 20 s up to 2 queued orders are moved from the most-
                to the least-loaded eligible drone if that lowers the imbalance.

No order is ever dropped: when a drone must return to recharge, its current
order goes back to the front of its own queue (round_robin / least_queue) or
back to the central backlog (load_index, which re-dispatches it). A per-step
check asserts that every order is in exactly one place (not yet arrived,
backlog, a drone queue, in flight, delivered). At 400 s each order is
delivered on time (<= 120 s after arrival), delivered late, in flight,
queued on a drone, or in the backlog.

Battery: Delta s = alpha (m_body + m_p) g d, alpha = 1.5e-5 1/m ->
D_range = 1 / (1.5e-5 * 1.8 * 9.81) = 3775 m.

Usage:
    conda activate drones
    python src/02_logistics_delivery/s040_fleet_load_balancing.py
"""

import os
from collections import deque
from dataclasses import dataclass, field
from typing import Optional

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.animation as animation

# ── Parameters ─────────────────────────────────────────────────────────────────
N_DRONES      = 8
N_PADS        = 2
M_ORDERS      = 40
T_SIM         = 400.0        # s
T_ARRIVE_MAX  = 0.8 * T_SIM  # s  orders arrive uniformly in [0, 320] s
DT            = 0.2          # s simulation timestep
REBAL_PERIOD  = 20.0         # s rebalancing epoch interval
S_MAX_SWAPS   = 2            # max queue swaps per rebalancing epoch
T_PROMISE     = 120.0        # s delivery promise (arrival -> drop) for "on time"

V_DRONE       = 8.0          # m/s cruise speed
M_BODY        = 1.5          # kg
M_PAYLOAD     = 0.3          # kg
G             = 9.81         # m/s²
E_CAP         = 1.0          # normalised battery capacity
ALPHA         = 1.5e-5       # discharge coefficient (1/m)
P_CHG         = 0.20         # charge rate (E_cap/s)
BETA_SAFE     = 0.05         # safety SoC reserve
K_DIS         = ALPHA * (M_BODY + M_PAYLOAD) * G / E_CAP   # SoC per metre
D_RANGE       = 1.0 / K_DIS  # 3775 m full-battery range
# Low-battery stress test: 4x the discharge coefficient (D_range = 944 m), so
# drones must recharge within the horizon and the return / requeue logic is used
ALPHA_STRESS  = 6.0e-5
K_DIS_STRESS  = ALPHA_STRESS * (M_BODY + M_PAYLOAD) * G / E_CAP
F_NORM        = D_RANGE * 0.5

PHI_W, PHI_F, PHI_S = 0.5, 0.3, 0.2   # load-index weights (sum = 1)

DEPOT = np.array([300.0, 300.0])
GRID  = (0.0, 600.0)
CORNERS = [np.array([x, y]) for x in GRID for y in GRID]
MAX_CORNER_DEPOT = max(float(np.hypot(*(c - DEPOT))) for c in CORNERS)   # 424 m

SHOW_SEED   = 42                     # order stream used for the plots
STUDY_SEEDS = list(range(20))        # further seeded order streams
GIF_FRAMES  = 150

STRATEGIES = ['round_robin', 'least_queue', 'load_index']
COLORS = {'round_robin': '#e74c3c', 'least_queue': '#3498db', 'load_index': '#2ecc71'}
LABELS = {'round_robin': 'Round-Robin', 'least_queue': 'Least-Queue',
          'load_index': 'Load-Index Balancer'}
STATE_COLORS = {'flying': '#2ecc71', 'returning': '#e67e22', 'charging': '#3498db',
                'waiting': '#9b59b6', 'idle': '#ecf0f1'}

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '02_logistics_delivery', 's040_fleet_load_balancing',
)


# ── Data classes ──────────────────────────────────────────────────────────────
@dataclass
class Order:
    idx: int
    location: np.ndarray
    t_arrive: float
    t_deliver: Optional[float] = None
    drone: Optional[int] = None


@dataclass
class Drone:
    idx: int
    pos: np.ndarray
    soc: float = 1.0
    queue: deque = field(default_factory=deque)
    current: Optional[Order] = None
    state: str = 'idle'          # idle | flying | returning | charging | waiting
    deliveries: int = 0
    flight_dist: float = 0.0
    returns: int = 0
    k: float = K_DIS              # SoC consumed per metre

    def n_unfinished(self):
        return len(self.queue) + (1 if self.current is not None else 0)

    def load_index(self):
        return (PHI_W * len(self.queue) + PHI_F * self.flight_dist / F_NORM
                + PHI_S * (1.0 - self.soc))

    def last_point(self):
        if self.queue:
            return self.queue[-1].location
        if self.current is not None:
            return self.current.location
        return self.pos

    def remaining_distance(self):
        """Distance to finish current + queued orders and fly home."""
        pts = [self.pos]
        if self.current is not None:
            pts.append(self.current.location)
        pts += [o.location for o in self.queue]
        pts.append(DEPOT)
        return float(sum(np.hypot(*(pts[k + 1] - pts[k])) for k in range(len(pts) - 1)))

    def can_accept(self, order):
        extra = (float(np.hypot(*(order.location - self.last_point())))
                 + float(np.hypot(*(DEPOT - order.location)))
                 - float(np.hypot(*(DEPOT - self.last_point()))))
        return self.soc - (self.remaining_distance() + extra) * self.k >= BETA_SAFE

    def available(self):
        return self.state in ('idle', 'flying')


# ── Strategy helpers ──────────────────────────────────────────────────────────
def marginal_load(drone, order):
    extra = float(np.hypot(*(order.location - drone.last_point())))
    return PHI_W * 1.0 + PHI_F * extra / F_NORM


def assign_load_index(drones, order):
    best_k, best = None, np.inf
    for d in drones:
        if not d.available() or not d.can_accept(order):
            continue
        score = d.load_index() + marginal_load(d, order)
        if score < best - 1e-12:
            best, best_k = score, d.idx
    return best_k


def fleet_imbalance(drones):
    loads = [d.load_index() for d in drones]
    mean = float(np.mean(loads))
    return 0.0 if mean == 0 else (max(loads) - min(loads)) / mean


def rebalance(drones):
    """Move the last queued order of the most-loaded drone to the least-loaded
    eligible drone while that lowers the imbalance (max S_MAX_SWAPS)."""
    swaps = 0
    for _ in range(S_MAX_SWAPS):
        donors = [d for d in drones if d.queue]
        takers = [d for d in drones if d.available()]
        if not donors or not takers:
            break
        most = max(donors, key=lambda d: (d.load_index(), -d.idx))
        least = min(takers, key=lambda d: (d.load_index(), d.idx))
        if most.idx == least.idx:
            break
        cand = most.queue[-1]
        before = fleet_imbalance(drones)
        most.queue.pop()
        ok = least.can_accept(cand)
        least.queue.append(cand)
        if not ok or fleet_imbalance(drones) >= before:
            least.queue.pop()
            most.queue.append(cand)
            break
        cand.drone = least.idx
        swaps += 1
    return swaps


def jain_fairness(counts):
    counts = np.asarray(counts, dtype=float)
    if counts.sum() == 0:
        return 1.0
    return counts.sum() ** 2 / (len(counts) * (counts ** 2).sum())


def check_conservation(orders, stream, backlog, drones):
    """Every order is in exactly one place: not yet arrived (stream), central
    backlog, a drone queue, in flight (a drone's current order) or delivered."""
    places = ([id(o) for o in stream] + [id(o) for o in backlog]
              + [id(o) for d in drones for o in d.queue]
              + [id(d.current) for d in drones if d.current is not None]
              + [id(o) for o in orders if o.t_deliver is not None])
    assert len(places) == len(orders) and len(set(places)) == len(orders), \
        'order conservation violated'
    assert not any(d.current is not None and d.current.t_deliver is not None for d in drones)


def make_orders(seed):
    rng = np.random.default_rng(seed)
    orders = [Order(idx=j, location=rng.uniform(*GRID, size=2),
                    t_arrive=float(rng.uniform(0, T_ARRIVE_MAX)))
              for j in range(M_ORDERS)]
    orders.sort(key=lambda o: (o.t_arrive, o.idx))
    return orders


# ── Simulation ────────────────────────────────────────────────────────────────
def simulate(strategy, seed, record=False, k_dis=K_DIS):
    orders = make_orders(seed)
    stream = deque(orders)
    drones = [Drone(idx=i, pos=DEPOT.copy(), k=k_dis) for i in range(N_DRONES)]
    pads = [None] * N_PADS
    wait_q = deque()
    backlog = deque()            # load_index only: orders waiting for an eligible drone
    rr = 0
    next_rebal = REBAL_PERIOD
    log_t, log_imb, log_cum, log_loads, log_states = [], [], [], [], []
    rebal_log, pos_hist, soc_hist = [], [], []

    def give(k, order):
        order.drone = k
        drones[k].queue.append(order)

    def release(d):
        """Drone has to go home: put its unfinished orders back."""
        if strategy == 'load_index':
            if d.current is not None:
                backlog.appendleft(d.current)
            backlog.extend(d.queue)
            d.queue.clear()
        elif d.current is not None:
            d.queue.appendleft(d.current)
        d.current = None

    def go_home(d):
        release(d)
        d.state = 'returning'
        d.returns += 1

    n_steps = int(round(T_SIM / DT))
    for step in range(n_steps + 1):
        t = step * DT
        # 1) arrivals
        while stream and stream[0].t_arrive <= t:
            o = stream.popleft()
            if strategy == 'round_robin':
                give(rr % N_DRONES, o)
                rr += 1
            elif strategy == 'least_queue':
                give(min(range(N_DRONES), key=lambda i: (drones[i].n_unfinished(), i)), o)
            else:
                backlog.append(o)
        # 2) load_index dispatch from backlog (FIFO)
        if strategy == 'load_index':
            for _ in range(len(backlog)):
                o = backlog.popleft()
                k = assign_load_index(drones, o)
                if k is None:
                    backlog.append(o)
                else:
                    give(k, o)
            if t >= next_rebal - 1e-9:
                rebal_log.append((t, rebalance(drones), fleet_imbalance(drones)))
                next_rebal += REBAL_PERIOD

        # 3) drones
        for d in drones:
            if d.state == 'idle' and not d.queue and d.soc < 1.0 - 1e-9:
                # opportunistic recharge: idle drone that could no longer fly a
                # worst-case order (farthest corner, then home) goes to charge
                worst = (max(np.hypot(*(c - d.pos)) for c in CORNERS) + MAX_CORNER_DEPOT)
                if d.soc - worst * d.k < BETA_SAFE:
                    go_home(d)
            if d.state == 'idle' and d.queue:
                nxt = d.queue[0]
                need = (np.hypot(*(nxt.location - d.pos)) + np.hypot(*(DEPOT - nxt.location))) * d.k
                if d.soc - need < BETA_SAFE:
                    go_home(d)                       # recharge before this leg
                else:
                    d.current = d.queue.popleft()
                    d.state = 'flying'
            if d.state in ('flying', 'returning'):
                tgt = d.current.location if d.state == 'flying' else DEPOT
                vec = tgt - d.pos
                dist = float(np.hypot(*vec))
                step_len = min(V_DRONE * DT, dist)
                if dist > 1e-9:
                    d.pos = d.pos + vec / dist * step_len
                d.soc = max(0.0, d.soc - d.k * step_len)
                d.flight_dist += step_len
                if d.state == 'flying':
                    if dist <= V_DRONE * DT:
                        d.current.t_deliver = t
                        d.deliveries += 1
                        d.current = None
                        d.state = 'idle'
                    elif d.soc <= d.k * np.hypot(*(d.pos - DEPOT)) + BETA_SAFE:
                        go_home(d)                   # card's in-flight return rule
                elif dist <= V_DRONE * DT:
                    d.pos = DEPOT.copy()
                    free = next((p for p, v in enumerate(pads) if v is None), None)
                    if free is None:
                        wait_q.append(d.idx)
                        d.state = 'waiting'
                    else:
                        pads[free] = d.idx
                        d.state = 'charging'
        # 4) pads
        for p in range(N_PADS):
            if pads[p] is None:
                continue
            d = drones[pads[p]]
            d.soc = min(1.0, d.soc + P_CHG * DT)
            if d.soc >= 1.0 - 1e-12:
                d.state = 'idle'
                pads[p] = None
                if wait_q:
                    pads[p] = wait_q.popleft()
                    drones[pads[p]].state = 'charging'

        check_conservation(orders, stream, backlog, drones)
        log_t.append(t)
        log_imb.append(fleet_imbalance(drones))
        log_cum.append(sum(d.deliveries for d in drones))
        if record:
            log_loads.append([d.load_index() for d in drones])
            log_states.append([d.state for d in drones])
            pos_hist.append(np.array([d.pos.copy() for d in drones]))
            soc_hist.append([d.soc for d in drones])

    # end state (conservation was checked every step)
    on_time = [o for o in orders if o.t_deliver is not None
               and o.t_deliver - o.t_arrive <= T_PROMISE]
    late = [o for o in orders if o.t_deliver is not None
            and o.t_deliver - o.t_arrive > T_PROMISE]
    in_flight = [d.current for d in drones if d.current is not None]
    in_queue = [o for d in drones for o in d.queue]
    in_backlog = list(backlog)
    svc = [o.t_deliver - o.t_arrive for o in orders if o.t_deliver is not None]
    counts = [d.deliveries for d in drones]
    res = dict(strategy=strategy, seed=seed, orders=orders,
               delivered=len(on_time) + len(late), on_time=len(on_time), late=len(late),
               in_flight=len(in_flight), in_queue=len(in_queue), in_backlog=len(in_backlog),
               mean_service=float(np.mean(svc)) if svc else np.nan,
               p95_service=float(np.percentile(svc, 95)) if svc else np.nan,
               max_service=float(np.max(svc)) if svc else np.nan,
               service=np.array(svc),
               per_drone=counts, jain=jain_fairness(counts),
               mean_imbalance=float(np.mean(log_imb)),
               flight_km=sum(d.flight_dist for d in drones) / 1000.0,
               max_drone_km=max(d.flight_dist for d in drones) / 1000.0,
               min_soc=None, returns=sum(d.returns for d in drones),
               rebal_log=rebal_log, swaps=sum(r[1] for r in rebal_log),
               times=np.array(log_t), imbalance=np.array(log_imb),
               cumulative=np.array(log_cum))
    if record:
        res.update(loads=np.array(log_loads), states=log_states,
                   pos=np.array(pos_hist), soc=np.array(soc_hist))
        res['min_soc'] = float(np.min(res['soc']))
    return res


def run_simulation():
    show = {s: simulate(s, SHOW_SEED, record=True) for s in STRATEGIES}
    study = {s: [simulate(s, seed) for seed in STUDY_SEEDS] for s in STRATEGIES}
    stress = {s: [simulate(s, seed, k_dis=K_DIS_STRESS) for seed in STUDY_SEEDS]
              for s in STRATEGIES}
    return show, study, stress


# ── Plots ─────────────────────────────────────────────────────────────────────
def _save(fig, out_dir, name, **kw):
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, name)
    fig.savefig(path, dpi=150, **kw)
    plt.close(fig)
    print(f'Saved: {path}')


def plot_imbalance_timeseries(show, out_dir):
    fig, ax = plt.subplots(figsize=(10, 5))
    for s in STRATEGIES:
        r = show[s]
        ax.plot(r['times'], r['imbalance'], color=COLORS[s], lw=1.1,
                label=f'{LABELS[s]} (mean {r["mean_imbalance"]:.3f})')
    for (t_r, _, _) in show['load_index']['rebal_log']:
        ax.axvline(t_r, color='#2ecc71', alpha=0.25, lw=0.8)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel(r'Fleet imbalance $\Lambda(t) = (L_{max}-L_{min})/\bar L$')
    ax.set_title(f'Fleet Load Imbalance Over Time (seed {SHOW_SEED}; '
                 'green lines = rebalancing epochs)')
    ax.legend()
    ax.set_xlim(0, T_SIM)
    ax.set_ylim(bottom=0)
    ax.grid(alpha=0.3)
    _save(fig, out_dir, 'imbalance_timeseries.png')


def plot_jain_fairness(show, study, out_dir):
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.5))
    x = np.arange(len(STRATEGIES))
    vals = [show[s]['jain'] for s in STRATEGIES]
    bars = axes[0].bar(x, vals, color=[COLORS[s] for s in STRATEGIES], edgecolor='black')
    for b, v in zip(bars, vals):
        axes[0].text(b.get_x() + b.get_width() / 2, v + 0.01, f'{v:.3f}', ha='center')
    axes[0].set_title(f"Jain's fairness of per-drone deliveries (seed {SHOW_SEED})")
    m = [np.mean([r['jain'] for r in study[s]]) for s in STRATEGIES]
    sd = [np.std([r['jain'] for r in study[s]]) for s in STRATEGIES]
    bars = axes[1].bar(x, m, yerr=sd, capsize=5, color=[COLORS[s] for s in STRATEGIES],
                       edgecolor='black')
    for b, v, e in zip(bars, m, sd):
        axes[1].text(b.get_x() + b.get_width() / 2 + 0.2, v + 0.01, f'{v:.3f}', ha='left')
    axes[1].set_title(f"Mean ± std over {len(STUDY_SEEDS)} order streams")
    for ax in axes:
        ax.axhline(1.0, color='gray', ls='--', lw=0.8)
        ax.set_xticks(x)
        ax.set_xticklabels([LABELS[s] for s in STRATEGIES])
        ax.set_ylim(0, 1.15)
        ax.set_ylabel(r"Jain's index $\mathcal{J}$")
        ax.grid(axis='y', alpha=0.3)
    plt.tight_layout()
    _save(fig, out_dir, 'jain_fairness.png')


def plot_per_drone_deliveries(show, out_dir):
    fig, ax = plt.subplots(figsize=(10, 5))
    x = np.arange(N_DRONES)
    for j, s in enumerate(STRATEGIES):
        ax.bar(x + (j - 1) * 0.25, show[s]['per_drone'], 0.25, color=COLORS[s],
               edgecolor='black', lw=0.4, label=f'{LABELS[s]} (total {show[s]["delivered"]})')
    ax.set_xlabel('Drone')
    ax.set_ylabel('Completed deliveries')
    ax.set_title(f'Per-Drone Delivery Count (seed {SHOW_SEED})')
    ax.set_xticks(x)
    ax.set_xticklabels([f'D{i}' for i in range(N_DRONES)])
    ax.legend()
    ax.grid(axis='y', alpha=0.3)
    _save(fig, out_dir, 'per_drone_deliveries.png', bbox_inches='tight')


def plot_cumulative_deliveries(show, out_dir):
    fig, ax = plt.subplots(figsize=(10, 5))
    arr = np.sort([o.t_arrive for o in show['round_robin']['orders']])
    ax.step(arr, np.arange(1, len(arr) + 1), where='post', color='black', ls=':',
            lw=1.2, label='Orders arrived')
    for s in STRATEGIES:
        r = show[s]
        ax.plot(r['times'], r['cumulative'], color=COLORS[s], lw=1.8,
                label=f'{LABELS[s]} ({r["delivered"]} delivered, mean service '
                      f'{r["mean_service"]:.1f} s)')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Cumulative count')
    ax.set_title(f'Orders Arrived vs Delivered (seed {SHOW_SEED})')
    ax.set_xlim(0, T_SIM)
    ax.set_ylim(0, M_ORDERS + 2)
    ax.legend(loc='upper left', fontsize=9)
    ax.grid(alpha=0.3)
    _save(fig, out_dir, 'cumulative_deliveries.png')


def plot_gantt_chart(show, out_dir):
    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    for ax, s in zip(axes, STRATEGIES):
        r = show[s]
        st = np.array(r['states'])
        for i in range(N_DRONES):
            col = st[:, i]
            start = 0
            for k in range(1, len(col) + 1):
                if k == len(col) or col[k] != col[start]:
                    ax.barh(i, (k - start) * DT, left=start * DT, height=0.6,
                            color=STATE_COLORS[col[start]], edgecolor='none')
                    start = k
        ax.set_yticks(range(N_DRONES))
        ax.set_yticklabels([f'D{i}' for i in range(N_DRONES)], fontsize=8)
        ax.set_title(f'{LABELS[s]}: busy (flying) share '
                     f'{np.mean(st == "flying") * 100:.0f} %, returns {r["returns"]}',
                     fontsize=10)
        ax.set_xlim(0, T_SIM)
        ax.grid(axis='x', alpha=0.3)
    axes[-1].set_xlabel('Time (s)')
    fig.legend(handles=[mpatches.Patch(color=c, label=k.capitalize())
                        for k, c in STATE_COLORS.items()], loc='lower center', ncol=5)
    fig.suptitle(f'Fleet State Gantt Chart (seed {SHOW_SEED})', fontsize=12)
    plt.tight_layout(rect=[0, 0.04, 1, 0.97])
    _save(fig, out_dir, 'gantt_chart.png')


def plot_load_heatmap(show, out_dir):
    fig, axes = plt.subplots(1, 3, figsize=(15, 4), sharey=True)
    vmax = max(float(show[s]['loads'].max()) for s in STRATEGIES)
    for ax, s in zip(axes, STRATEGIES):
        im = ax.imshow(show[s]['loads'].T, aspect='auto', origin='lower',
                       extent=[0, T_SIM, -0.5, N_DRONES - 0.5], cmap='YlOrRd',
                       vmin=0, vmax=vmax)
        ax.set_title(LABELS[s])
        ax.set_xlabel('Time (s)')
        ax.set_yticks(range(N_DRONES))
        ax.set_yticklabels([f'D{i}' for i in range(N_DRONES)], fontsize=7)
    axes[0].set_ylabel('Drone')
    fig.colorbar(im, ax=axes.tolist(), label='Load index $L_i(t)$', shrink=0.8)
    fig.suptitle(f'Load Index Heatmap (seed {SHOW_SEED})', fontsize=12)
    _save(fig, out_dir, 'load_heatmap.png', bbox_inches='tight')


def plot_rebalancing_events(show, out_dir):
    r = show['load_index']
    t_r = np.array([v[0] for v in r['rebal_log']])
    n_sw = np.array([v[1] for v in r['rebal_log']])
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    ax1.plot(r['times'], r['imbalance'], color='#2ecc71', lw=1.2)
    for tt in t_r:
        ax1.axvline(tt, color='#e74c3c', lw=0.6, alpha=0.5)
    ax1.set_ylabel(r'$\Lambda(t)$')
    ax1.set_title(f'Load-Index Balancer: imbalance and queue swaps (seed {SHOW_SEED}; '
                  f'{int(n_sw.sum())} swaps in {len(t_r)} epochs)')
    ax1.set_ylim(bottom=0)
    ax1.grid(alpha=0.3)
    ax2.bar(t_r, n_sw, width=3, color='#e74c3c', alpha=0.6)
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Queue swaps')
    ax2.set_ylim(0, S_MAX_SWAPS + 0.5)
    ax2.set_xlim(0, T_SIM)
    ax2.grid(alpha=0.3)
    plt.tight_layout()
    _save(fig, out_dir, 'rebalancing_events.png')


def plot_service_time(show, study, out_dir):
    fig, axes = plt.subplots(1, 2, figsize=(13, 4.8))
    axes[0].boxplot([show[s]['service'] for s in STRATEGIES], showmeans=True)
    axes[0].axhline(T_PROMISE, color='red', ls='--', lw=1, label=f'{T_PROMISE:.0f} s promise')
    axes[0].set_xticks(range(1, len(STRATEGIES) + 1))
    axes[0].set_xticklabels([LABELS[s] for s in STRATEGIES])
    axes[0].set_ylabel('Service time: arrival → delivery (s)')
    axes[0].set_title(f'Service time of delivered orders (seed {SHOW_SEED})')
    axes[0].legend()
    for j, s in enumerate(STRATEGIES):
        v = [r['mean_service'] for r in study[s]]
        axes[1].plot(STUDY_SEEDS, v, 'o-', color=COLORS[s], ms=4,
                     label=f'{LABELS[s]} (mean {np.mean(v):.1f} s)')
    axes[1].set_xlabel('Order-stream seed')
    axes[1].set_ylabel('Mean service time (s)')
    axes[1].set_title(f'Mean service time on {len(STUDY_SEEDS)} order streams')
    axes[1].legend(fontsize=8)
    for ax in axes:
        ax.grid(alpha=0.3)
    plt.tight_layout()
    _save(fig, out_dir, 'service_time.png')


def plot_stress_test(stress, out_dir):
    """Order outcome and service time with the low-battery configuration."""
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.6))
    x = np.arange(len(STRATEGIES))
    on = [np.mean([r['on_time'] for r in stress[s]]) for s in STRATEGIES]
    lt = [np.mean([r['late'] for r in stress[s]]) for s in STRATEGIES]
    op = [np.mean([r['in_flight'] + r['in_queue'] + r['in_backlog'] for r in stress[s]])
          for s in STRATEGIES]
    axes[0].bar(x, on, color='#2ecc71', edgecolor='black', label='On time (≤ 120 s)')
    axes[0].bar(x, lt, bottom=on, color='#f39c12', edgecolor='black', label='Late')
    axes[0].bar(x, op, bottom=np.array(on) + np.array(lt), color='#bbbbbb',
                edgecolor='black', label='Still open at 400 s')
    axes[0].set_ylim(0, M_ORDERS + 6)
    axes[0].set_ylabel('Orders (mean over streams)')
    axes[0].set_title('Order outcome')
    axes[0].legend(fontsize=8, loc='upper center', ncol=3)
    sv = [np.mean([r['mean_service'] for r in stress[s]]) for s in STRATEGIES]
    p95 = [np.mean([r['p95_service'] for r in stress[s]]) for s in STRATEGIES]
    axes[1].bar(x - 0.2, sv, 0.4, color=[COLORS[s] for s in STRATEGIES], edgecolor='black',
                label='mean')
    axes[1].bar(x + 0.2, p95, 0.4, color=[COLORS[s] for s in STRATEGIES], edgecolor='black',
                alpha=0.5, hatch='//', label='95th percentile')
    axes[1].axhline(T_PROMISE, color='red', ls='--', lw=1)
    axes[1].set_ylabel('Service time of delivered orders (s)')
    axes[1].set_title('Service time')
    axes[1].legend(fontsize=8)
    rt = [np.mean([r['returns'] for r in stress[s]]) for s in STRATEGIES]
    axes[2].bar(x, rt, color=[COLORS[s] for s in STRATEGIES], edgecolor='black')
    axes[2].set_ylabel('Returns to depot for charging')
    axes[2].set_title('Recharge returns per run')
    for ax in axes:
        ax.set_xticks(x)
        ax.set_xticklabels([LABELS[s] for s in STRATEGIES], fontsize=9)
        ax.grid(axis='y', alpha=0.3)
    fig.suptitle(f'Low-battery stress test (alpha = {ALPHA_STRESS:.0e} 1/m, D_range = '
                 f'{1 / K_DIS_STRESS:.0f} m; {len(STUDY_SEEDS)} order streams)', fontsize=12)
    plt.tight_layout()
    _save(fig, out_dir, 'stress_test.png')


def save_animation(show, out_dir):
    fig, axes = plt.subplots(1, 3, figsize=(15, 5.3))
    n = len(show['load_index']['times'])
    frames = np.linspace(0, n - 1, GIF_FRAMES).round().astype(int)
    art = []
    for ax, s in zip(axes, STRATEGIES):
        r = show[s]
        ax.set_xlim(-20, 620)
        ax.set_ylim(-20, 620)
        ax.set_aspect('equal')
        ax.set_title(LABELS[s], fontsize=10)
        ax.plot(*DEPOT, 's', ms=10, color='#34495e', zorder=5)
        locs = np.array([o.location for o in r['orders']])
        sc = ax.scatter(locs[:, 0], locs[:, 1], s=14, c='white', edgecolors='none')
        dots = ax.scatter(r['pos'][0][:, 0], r['pos'][0][:, 1], s=40, zorder=6)
        txt = ax.text(0.02, 0.97, '', transform=ax.transAxes, fontsize=8, va='top')
        art.append((r, sc, dots, txt))
    clock = fig.suptitle('')

    def update(fi):
        i = frames[fi]
        t = i * DT
        out = []
        for r, sc, dots, txt in art:
            dots.set_offsets(r['pos'][i])
            dots.set_color([STATE_COLORS[st] if st != 'idle' else '#7f8c8d'
                            for st in r['states'][i]])
            fc = []
            for o in r['orders']:
                if o.t_arrive > t:
                    fc.append((1, 1, 1, 0))
                elif o.t_deliver is not None and o.t_deliver <= t:
                    fc.append((0.6, 0.6, 0.6, 0.5))
                else:
                    fc.append((0.9, 0.1, 0.1, 1))
            sc.set_facecolors(fc)
            txt.set_text(f'delivered {int(r["cumulative"][i])}/{M_ORDERS}')
            out += [sc, dots, txt]
        clock.set_text(f'S040 Fleet Load Balancing — t = {t:.0f} s (seed {SHOW_SEED}; '
                       'red = open order, grey = delivered; drone colour = state)')
        return out + [clock]

    ani = animation.FuncAnimation(fig, update, frames=len(frames), blit=False)
    plt.tight_layout()
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer='pillow', fps=15, dpi=72)
    plt.close(fig)
    print(f'Saved: {path}')




# ── Main ──────────────────────────────────────────────────────────────────────
def print_summary(title, runs):
    print(f'\n--- {title} ---')
    for s in STRATEGIES:
        R = runs[s]
        print(f'{LABELS[s]:<20} delivered {np.mean([r["delivered"] for r in R]):.2f} '
              f'(on time {np.mean([r["on_time"] for r in R]):.2f}, late '
              f'{np.mean([r["late"] for r in R]):.2f}), open at end: in flight '
              f'{np.mean([r["in_flight"] for r in R]):.2f}, queued on a drone '
              f'{np.mean([r["in_queue"] for r in R]):.2f}, in backlog '
              f'{np.mean([r["in_backlog"] for r in R]):.2f} | service mean '
              f'{np.mean([r["mean_service"] for r in R]):.1f} s, p95 '
              f'{np.mean([r["p95_service"] for r in R]):.1f} s | Jain '
              f'{np.mean([r["jain"] for r in R]):.3f} | mean Lambda '
              f'{np.mean([r["mean_imbalance"] for r in R]):.3f} | flight '
              f'{np.mean([r["flight_km"] for r in R]):.2f} km | returns '
              f'{np.mean([r["returns"] for r in R]):.2f}')
    best = {s: 0 for s in STRATEGIES}
    for i in range(len(STUDY_SEEDS)):
        best[min(STRATEGIES, key=lambda s: (runs[s][i]['mean_service'], s))] += 1
    print('Lowest mean service time per stream: ' +
          ', '.join(f'{LABELS[s]} {best[s]}' for s in STRATEGIES))
    print('Streams with at least one recharge return: ' +
          ', '.join(f'{LABELS[s]} {sum(1 for r in runs[s] if r["returns"] > 0)}/{len(runs[s])}'
                    for s in STRATEGIES))


if __name__ == '__main__':
    show, study, stress = run_simulation()
    print('=== S040 Fleet Load Balancing ===')
    print(f'k = alpha (m_body+m_p) g = {K_DIS:.3e} SoC/m, D_range = {D_RANGE:.0f} m, '
          f'F_norm = {F_NORM:.0f} m; stress test k = {K_DIS_STRESS:.3e} SoC/m, '
          f'D_range = {1 / K_DIS_STRESS:.0f} m')
    print(f'\n--- Showcase order stream (seed {SHOW_SEED}) ---')
    for s in STRATEGIES:
        r = show[s]
        print(f'{LABELS[s]:<20} delivered {r["delivered"]:>2}/{M_ORDERS} '
              f'(on time {r["on_time"]}, late {r["late"]}), open at end: in flight '
              f'{r["in_flight"]}, queued on a drone {r["in_queue"]}, in backlog '
              f'{r["in_backlog"]} | service mean '
              f'{r["mean_service"]:.1f} s, p95 {r["p95_service"]:.1f} s, max '
              f'{r["max_service"]:.1f} s | Jain {r["jain"]:.3f} | mean Lambda '
              f'{r["mean_imbalance"]:.3f} | flight {r["flight_km"]:.2f} km (max drone '
              f'{r["max_drone_km"]:.2f} km) | returns {r["returns"]} | min SoC '
              f'{r["min_soc"]:.3f} | swaps {r["swaps"]}')
        print(f'{"":<20} per-drone deliveries {r["per_drone"]}')
    print_summary(f'Study: {len(STUDY_SEEDS)} order streams (seeds {STUDY_SEEDS[0]}..'
                  f'{STUDY_SEEDS[-1]}), nominal battery', study)
    print_summary(f'Low-battery stress test (alpha = {ALPHA_STRESS:.1e}), same '
                  f'{len(STUDY_SEEDS)} order streams', stress)

    out_dir = os.path.normpath(OUTPUT_DIR)
    plot_imbalance_timeseries(show, out_dir)
    plot_jain_fairness(show, study, out_dir)
    plot_per_drone_deliveries(show, out_dir)
    plot_cumulative_deliveries(show, out_dir)
    plot_gantt_chart(show, out_dir)
    plot_load_heatmap(show, out_dir)
    plot_rebalancing_events(show, out_dir)
    plot_service_time(show, study, out_dir)
    plot_stress_test(stress, out_dir)
    save_animation(show, out_dir)
