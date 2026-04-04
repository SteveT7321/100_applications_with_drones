"""
S032 Charging Queue
===================
Simulates a fleet of 6 delivery drones competing for 2 shared charging pads at a base
station. Drones fly to random delivery waypoints, monitor battery state-of-charge (SoC),
and return to base when low on charge. A FIFO queue manages access to charging pads.
Three charge-management policies are compared — threshold, greedy, and predictive — and
the optimal return threshold β* that maximises total deliveries within a 300-second
horizon is identified.

Usage:
    conda activate drones
    python src/02_logistics_delivery/s032_charging_queue.py
"""

import sys
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from collections import deque

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

# ── Parameters ─────────────────────────────────
N_DRONES      = 6
N_PADS        = 2
T_SIM         = 300.0          # s  simulation horizon
DT            = 0.5            # s  timestep
V_DRONE       = 8.0            # m/s cruise speed
E_CAP         = 1.0            # normalised battery capacity
P_CHG         = 0.25           # charge rate (fraction of E_cap per second)
ALPHA         = 1.8e-4         # efficiency constant (1/m)
M_BODY        = 1.5            # kg
M_PAYLOAD     = 0.5            # kg
G             = 9.81           # m/s²
BETA_SAFE     = 0.05           # emergency reserve
BASE          = np.array([0.0, 0.0, 2.0])
GRID_HALF     = 30.0           # m, delivery area half-width
BETA_SWEEP    = np.arange(0.10, 0.55, 0.05)   # threshold candidates

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '02_logistics_delivery', 's032_charging_queue',
)

RNG = np.random.default_rng(0)   # fixed seed for reproducibility


# ── Helpers ────────────────────────────────────
def discharge_per_metre(m_payload=M_PAYLOAD):
    """Energy fraction consumed per metre of flight."""
    return ALPHA * (M_BODY + m_payload) * G / E_CAP


def random_waypoint(rng):
    x = rng.uniform(-GRID_HALF, GRID_HALF)
    y = rng.uniform(-GRID_HALF, GRID_HALF)
    return np.array([x, y, 2.0])


def min_soc_to_return(pos):
    """Minimum SoC required to fly from pos back to BASE plus safety reserve."""
    dist = np.linalg.norm(pos - BASE)
    return discharge_per_metre() * dist + BETA_SAFE


# ── Simulation ─────────────────────────────────
def run_simulation(beta_threshold, policy='threshold', seed=42, record=False):
    """
    Run the charging-queue simulation for one policy configuration.

    Parameters
    ----------
    beta_threshold : float  – SoC level that triggers return (threshold policy)
    policy         : str    – 'threshold' | 'greedy' | 'predictive'
    seed           : int    – RNG seed
    record         : bool   – whether to record per-step state for plotting

    Returns
    -------
    total_deliveries : int
    drones_data      : list of per-drone dicts  (only populated when record=True)
    queue_ts         : array of (time, queue_length)  (only when record=True)
    """
    rng = np.random.default_rng(seed)

    # Drone state
    soc      = np.ones(N_DRONES)
    pos      = np.tile(BASE, (N_DRONES, 1)).astype(float)
    state    = ['idle'] * N_DRONES   # idle | flying | returning | charging | waiting
    target   = [None]  * N_DRONES
    deliveries = np.zeros(N_DRONES, dtype=int)
    fly_time   = np.zeros(N_DRONES)   # cumulative airborne seconds

    # Recording
    soc_history   = [[] for _ in range(N_DRONES)]    # list of (t, soc)
    state_history = [[] for _ in range(N_DRONES)]    # list of (t, state)
    queue_ts      = []                                # (t, queue_length)

    # Charging infrastructure
    pads             = [None] * N_PADS     # drone index on each pad
    charge_remaining = [0.0] * N_PADS     # seconds of charge left on each pad
    queue            = deque()             # FIFO waiting list (drone indices)

    # Mission queue (pre-generate; replenished after each delivery)
    mission_queue = deque(random_waypoint(rng) for _ in range(100))

    t = 0.0
    steps = int(T_SIM / DT)

    for _ in range(steps):
        # ─── Assign missions to idle drones ───────────────────────────────
        for i in range(N_DRONES):
            if state[i] == 'idle' and mission_queue:
                target[i] = mission_queue.popleft()
                state[i]  = 'flying'

        # ─── Step each drone ──────────────────────────────────────────────
        for i in range(N_DRONES):
            if state[i] == 'flying':
                dist_step = V_DRONE * DT
                to_tgt    = target[i] - pos[i]
                dist_rem  = np.linalg.norm(to_tgt)

                # Should we abort and return?
                should_return = False
                if policy == 'threshold':
                    should_return = soc[i] <= beta_threshold
                elif policy == 'greedy':
                    should_return = False   # always finish unless forced
                elif policy == 'predictive':
                    should_return = soc[i] <= min_soc_to_return(pos[i])

                # Safety override (always)
                if soc[i] <= min_soc_to_return(pos[i]):
                    should_return = True

                if should_return:
                    state[i]  = 'returning'
                    target[i] = BASE.copy()
                    fly_time[i] += DT
                    continue

                if dist_rem <= dist_step:
                    # Arrived at waypoint
                    soc[i]   -= discharge_per_metre() * dist_rem
                    soc[i]    = max(soc[i], 0.0)
                    pos[i]    = target[i].copy()
                    deliveries[i] += 1
                    mission_queue.append(random_waypoint(rng))
                    state[i]  = 'returning'
                    target[i] = BASE.copy()
                else:
                    pos[i]   += (to_tgt / dist_rem) * dist_step
                    soc[i]   -= discharge_per_metre() * dist_step
                fly_time[i] += DT

            elif state[i] == 'returning':
                to_base   = BASE - pos[i]
                dist_rem  = np.linalg.norm(to_base)
                dist_step = V_DRONE * DT

                if dist_rem <= dist_step:
                    soc[i]  -= discharge_per_metre() * dist_rem
                    soc[i]   = max(soc[i], 0.0)
                    pos[i]   = BASE.copy()
                    # Try to acquire a free pad
                    free = next((k for k, v in enumerate(pads) if v is None), None)
                    if free is not None:
                        pads[free]             = i
                        charge_remaining[free] = (1.0 - soc[i]) / P_CHG
                        state[i]               = 'charging'
                    else:
                        queue.append(i)
                        state[i] = 'waiting'
                else:
                    pos[i]  += (to_base / dist_rem) * dist_step
                    soc[i]  -= discharge_per_metre() * dist_step
                fly_time[i] += DT

        # ─── Advance charging pads ────────────────────────────────────────
        for k in range(N_PADS):
            if pads[k] is not None:
                charge_remaining[k] -= DT
                i = pads[k]
                soc[i] = min(1.0, soc[i] + P_CHG * DT)
                if charge_remaining[k] <= 0:
                    soc[i]   = 1.0
                    state[i] = 'idle'
                    pads[k]  = None
                    if queue:
                        nxt = queue.popleft()
                        pads[k]             = nxt
                        charge_remaining[k] = (1.0 - soc[nxt]) / P_CHG
                        state[nxt]          = 'charging'

        t += DT

        # ─── Recording ────────────────────────────────────────────────────
        if record:
            for i in range(N_DRONES):
                soc_history[i].append((t, soc[i]))
                state_history[i].append((t, state[i]))
            queue_ts.append((t, len(queue)))

    # Fleet utilisation
    mean_util = np.mean(fly_time / T_SIM)

    drone_data = None
    if record:
        drone_data = [
            {'soc': soc_history[i], 'state': state_history[i], 'deliveries': int(deliveries[i])}
            for i in range(N_DRONES)
        ]

    return int(deliveries.sum()), mean_util, drone_data, queue_ts


# ── Plots ──────────────────────────────────────
def plot_threshold_sweep(betas, deliveries_list, beta_star, out_dir):
    """Bar/line chart of total deliveries vs return threshold β."""
    fig, ax = plt.subplots(figsize=(8, 4))
    colors = ['#e05c5c' if b == beta_star else '#4c9be8' for b in betas]
    ax.bar(betas, deliveries_list, width=0.035, color=colors, edgecolor='white', linewidth=0.5)
    ax.axvline(beta_star, color='#e05c5c', linestyle='--', linewidth=1.4,
               label=f'β* = {beta_star:.2f}  (D={max(deliveries_list)})')
    ax.set_xlabel('Return Threshold β')
    ax.set_ylabel('Total Deliveries  $D_{total}$')
    ax.set_title('S032 – Charging Queue: Threshold Sweep')
    ax.legend()
    ax.set_xticks(betas)
    ax.set_xticklabels([f'{b:.2f}' for b in betas], rotation=45)
    ax.yaxis.grid(True, linestyle=':', alpha=0.6)
    ax.set_axisbelow(True)
    fig.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'threshold_sweep.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def plot_policy_comparison(policy_results, out_dir):
    """Bar chart comparing threshold-optimal, greedy, and predictive policies."""
    labels  = [r['label']       for r in policy_results]
    totals  = [r['deliveries']  for r in policy_results]
    utils   = [r['utilisation'] for r in policy_results]
    colors  = ['#4c9be8', '#f4a55b', '#56c478']

    fig, axes = plt.subplots(1, 2, figsize=(10, 4))

    ax = axes[0]
    bars = ax.bar(labels, totals, color=colors, edgecolor='white')
    ax.bar_label(bars, fmt='%d', padding=3, fontsize=11)
    ax.set_ylabel('Total Deliveries')
    ax.set_title('Total Deliveries by Policy')
    ax.yaxis.grid(True, linestyle=':', alpha=0.6)
    ax.set_axisbelow(True)

    ax = axes[1]
    bars2 = ax.bar(labels, [u * 100 for u in utils], color=colors, edgecolor='white')
    ax.bar_label(bars2, fmt='%.1f%%', padding=3, fontsize=10)
    ax.set_ylabel('Mean Fleet Utilisation (%)')
    ax.set_title('Fleet Utilisation by Policy')
    ax.yaxis.grid(True, linestyle=':', alpha=0.6)
    ax.set_axisbelow(True)
    ax.set_ylim(0, 100)

    fig.suptitle('S032 – Policy Comparison', fontsize=13)
    fig.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'policy_comparison.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def plot_soc_traces(drone_data, out_dir):
    """SoC over time for all 6 drones (threshold-optimal run)."""
    cmap   = plt.get_cmap('tab10')
    fig, ax = plt.subplots(figsize=(12, 4))
    for i, dd in enumerate(drone_data):
        ts   = [p[0] for p in dd['soc']]
        socs = [p[1] for p in dd['soc']]
        ax.plot(ts, socs, color=cmap(i), linewidth=0.8, label=f'Drone {i}')
    ax.axhline(BETA_SAFE, color='red', linestyle='--', linewidth=0.9, label=f'β_safe={BETA_SAFE}')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('State of Charge')
    ax.set_title('S032 – SoC Traces (Optimal Threshold Policy)')
    ax.legend(ncol=4, fontsize=8, loc='lower right')
    ax.set_ylim(-0.05, 1.05)
    ax.yaxis.grid(True, linestyle=':', alpha=0.6)
    ax.set_axisbelow(True)
    fig.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'soc_traces.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def plot_queue_length(queue_ts, out_dir):
    """Queue length (waiting drones) over time."""
    ts  = [p[0] for p in queue_ts]
    qls = [p[1] for p in queue_ts]
    fig, ax = plt.subplots(figsize=(10, 3))
    ax.fill_between(ts, qls, step='post', alpha=0.35, color='#e05c5c')
    ax.step(ts, qls, where='post', color='#e05c5c', linewidth=1.2)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Drones Waiting in Queue')
    ax.set_title('S032 – Charging-Pad Queue Length Over Time')
    ax.set_ylim(-0.2, max(qls) + 1)
    ax.yaxis.grid(True, linestyle=':', alpha=0.6)
    ax.set_axisbelow(True)
    fig.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'queue_length.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def plot_state_gantt(drone_data, out_dir):
    """Gantt-style chart showing each drone's state over time."""
    STATE_COLORS = {
        'idle':      '#9ecae1',
        'flying':    '#4daf4a',
        'returning': '#ff7f00',
        'charging':  '#e31a1c',
        'waiting':   '#984ea3',
    }
    fig, ax = plt.subplots(figsize=(13, 4))
    for i, dd in enumerate(drone_data):
        states = dd['state']
        if not states:
            continue
        # Build run-length segments
        prev_t, prev_s = states[0]
        for t, s in states[1:]:
            if s != prev_s:
                rect_width = t - prev_t
                color = STATE_COLORS.get(prev_s, '#cccccc')
                ax.barh(i, rect_width, left=prev_t, height=0.6,
                        color=color, edgecolor='none')
                prev_t, prev_s = t, s
        # Last segment
        ax.barh(i, T_SIM - prev_t, left=prev_t, height=0.6,
                color=STATE_COLORS.get(prev_s, '#cccccc'), edgecolor='none')

    ax.set_yticks(range(N_DRONES))
    ax.set_yticklabels([f'Drone {i}' for i in range(N_DRONES)])
    ax.set_xlabel('Time (s)')
    ax.set_title('S032 – Drone State Timeline (Optimal Threshold Policy)')
    # Legend
    patches = [mpatches.Patch(color=v, label=k) for k, v in STATE_COLORS.items()]
    ax.legend(handles=patches, ncol=5, fontsize=8, loc='upper right')
    ax.set_xlim(0, T_SIM)
    fig.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'state_gantt.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def save_animation(drone_data, queue_ts, beta_star, out_dir):
    """Animate SoC bars and queue length over time."""
    import matplotlib.animation as animation

    ts     = [p[0] for p in drone_data[0]['soc']]
    soc_arr = np.array([[p[1] for p in dd['soc']] for dd in drone_data])  # (N, steps)
    qls    = np.array([p[1] for p in queue_ts])
    cmap   = plt.get_cmap('tab10')

    STEP   = 4    # frame decimation
    frames = range(0, len(ts), STEP)

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4))

    # SoC bars
    bars = ax1.bar(range(N_DRONES),
                   soc_arr[:, 0],
                   color=[cmap(i) for i in range(N_DRONES)],
                   edgecolor='white')
    ax1.axhline(beta_star, color='black', linestyle='--', linewidth=1, label=f'β*={beta_star:.2f}')
    ax1.axhline(BETA_SAFE,  color='red',   linestyle=':',  linewidth=1, label=f'β_safe={BETA_SAFE}')
    ax1.set_xlim(-0.5, N_DRONES - 0.5)
    ax1.set_ylim(0, 1.05)
    ax1.set_xticks(range(N_DRONES))
    ax1.set_xticklabels([f'D{i}' for i in range(N_DRONES)])
    ax1.set_ylabel('SoC')
    ax1.set_title('Battery Level')
    ax1.legend(fontsize=7)
    time_txt = ax1.text(0.02, 0.97, 't = 0.0 s', transform=ax1.transAxes,
                        va='top', fontsize=9)

    # Queue line
    line_q, = ax2.plot([], [], color='#e05c5c', linewidth=1.4)
    ax2.set_xlim(0, T_SIM)
    ax2.set_ylim(-0.2, max(qls.max(), 1) + 1)
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Drones in Queue')
    ax2.set_title('Charging Queue Length')
    ax2.yaxis.grid(True, linestyle=':', alpha=0.6)
    ax2.set_axisbelow(True)

    def update(f):
        for i, bar in enumerate(bars):
            bar.set_height(soc_arr[i, f])
        time_txt.set_text(f't = {ts[f]:.1f} s')
        line_q.set_data(ts[:f+1], qls[:f+1])
        return list(bars) + [time_txt, line_q]

    ani = animation.FuncAnimation(fig, update, frames=frames,
                                  interval=50, blit=True)
    fig.suptitle('S032 – Charging Queue (Optimal Threshold Policy)', fontsize=11)
    fig.tight_layout()

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer='pillow', fps=20, dpi=100)
    plt.close()
    print(f'Saved: {path}')


if __name__ == '__main__':
    out_dir = os.path.normpath(OUTPUT_DIR)

    # ── Threshold sweep ───────────────────────────────────────────────────
    print('Running threshold sweep ...')
    sweep_deliveries = []
    sweep_utils      = []
    for beta in BETA_SWEEP:
        d, u, _, _ = run_simulation(beta, policy='threshold', seed=42)
        sweep_deliveries.append(d)
        sweep_utils.append(u)

    beta_star = BETA_SWEEP[int(np.argmax(sweep_deliveries))]
    d_star    = max(sweep_deliveries)

    print(f'\n── Threshold Sweep Results ──────────────────')
    for b, d in zip(BETA_SWEEP, sweep_deliveries):
        marker = ' ← optimal' if b == beta_star else ''
        print(f'  β = {b:.2f}  →  {d:3d} deliveries{marker}')
    print(f'  β* = {beta_star:.2f}  (D_total = {d_star})')

    # ── Policy comparison ─────────────────────────────────────────────────
    print('\nRunning policy comparison ...')
    d_greedy,    u_greedy,    _, _ = run_simulation(beta_star, policy='greedy',     seed=42)
    d_predictive, u_predictive, _, _ = run_simulation(beta_star, policy='predictive', seed=42)

    policy_results = [
        {'label': f'Threshold\n(β*={beta_star:.2f})', 'deliveries': d_star,       'utilisation': sweep_utils[int(np.argmax(sweep_deliveries))]},
        {'label': 'Greedy',                            'deliveries': d_greedy,     'utilisation': u_greedy},
        {'label': 'Predictive',                        'deliveries': d_predictive, 'utilisation': u_predictive},
    ]

    print('\n── Policy Comparison ────────────────────────')
    for p in policy_results:
        print(f"  {p['label'].replace(chr(10), ' '):<30}  deliveries={p['deliveries']:3d}  "
              f"utilisation={p['utilisation']*100:.1f}%")

    # ── Detailed run for recording ────────────────────────────────────────
    print('\nRunning detailed recording for plots ...')
    _, _, drone_data, queue_ts = run_simulation(beta_star, policy='threshold',
                                                seed=42, record=True)

    # ── Plots ─────────────────────────────────────────────────────────────
    plot_threshold_sweep(BETA_SWEEP, sweep_deliveries, beta_star, out_dir)
    plot_policy_comparison(policy_results, out_dir)
    plot_soc_traces(drone_data, out_dir)
    plot_queue_length(queue_ts, out_dir)
    plot_state_gantt(drone_data, out_dir)
    save_animation(drone_data, queue_ts, beta_star, out_dir)

    print('\n── Summary ──────────────────────────────────')
    print(f'  Optimal threshold  β*          = {beta_star:.2f}')
    print(f'  Deliveries (threshold β*)      = {d_star}')
    print(f'  Deliveries (greedy)            = {d_greedy}')
    print(f'  Deliveries (predictive)        = {d_predictive}')
    print(f'  Mean fleet utilisation (β*)    = {sweep_utils[int(np.argmax(sweep_deliveries))]*100:.1f}%')
    print(f'  Mean fleet utilisation (greedy)= {u_greedy*100:.1f}%')
