"""
S032 Charging Queue
===================
A fleet of 6 delivery drones shares 2 charging pads at a base station. Drones fly
multi-drop sorties: after each delivery they continue directly to the next delivery
point from the shared mission queue and only return to base when their charge policy
tells them to. At base they wait in a FIFO queue for a free pad and charge to full.

Charge policies compared (same random delivery streams for every policy):
  threshold  - abort the current delivery and return as soon as SoC <= beta (beta swept)
  greedy     - always finish the current delivery; return after it if SoC <= beta (beta swept)
  predictive - return (aborting if necessary) as soon as SoC <= s_min(i), the energy
               needed to fly home plus the reserve beta_safe
A drone whose SoC reaches 0 before it is back at base is stranded (out of service, its
current delivery is lost).

Results are averaged over N_SEEDS Monte-Carlo delivery streams.

Usage:
    conda activate drones
    python src/02_logistics_delivery/s032_charging_queue.py
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from collections import deque

# ── Parameters ─────────────────────────────────
N_DRONES      = 6
N_PADS        = 2
T_SIM         = 300.0          # s  simulation horizon
DT            = 0.5            # s  timestep
V_DRONE       = 8.0            # m/s cruise speed
E_CAP         = 1.0            # normalised battery capacity
P_CHG         = 0.04           # charge rate (fraction of E_cap per second) -> 25 s from empty
ALPHA         = 1.8e-4         # efficiency constant (1/m)
M_BODY        = 1.5            # kg
M_PAYLOAD     = 0.5            # kg
G             = 9.81           # m/s^2
BETA_SAFE     = 0.05           # emergency reserve (predictive policy)
BASE          = np.array([0.0, 0.0, 2.0])
GRID_HALF     = 30.0           # m, delivery area half-width
BETA_SWEEP    = np.round(np.arange(0.10, 0.55, 0.05), 2)   # threshold candidates
SEED0         = 42             # first Monte-Carlo seed
N_SEEDS       = 20             # Monte-Carlo delivery streams (seeds 42 ... 61)

POLICY_COLORS = {'threshold': '#4c9be8', 'greedy': '#f4a55b', 'predictive': '#56c478'}

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '02_logistics_delivery', 's032_charging_queue',
)


# ── Helpers ────────────────────────────────────
def discharge_per_metre(m_payload=M_PAYLOAD):
    """Energy fraction consumed per metre of flight: alpha (m_body + m_p) g / E_cap."""
    return ALPHA * (M_BODY + m_payload) * G / E_CAP


def random_waypoint(rng):
    x = rng.uniform(-GRID_HALF, GRID_HALF)
    y = rng.uniform(-GRID_HALF, GRID_HALF)
    return np.array([x, y, 2.0])


def min_soc_to_return(pos):
    """Minimum SoC required to fly from pos back to BASE plus safety reserve."""
    return discharge_per_metre() * np.linalg.norm(pos - BASE) + BETA_SAFE


# ── Simulation ─────────────────────────────────
def run_simulation(beta_threshold, policy='threshold', seed=SEED0, record=False):
    """
    Run the charging-queue simulation for one policy.

    Returns dict: deliveries, utilisation, stranded, aborted, charge_visits,
    wait_time (total s spent queueing), and (if record) soc/state histories, queue_ts.
    """
    rng = np.random.default_rng(seed)
    dpm = discharge_per_metre()

    soc        = np.ones(N_DRONES)
    pos        = np.tile(BASE, (N_DRONES, 1)).astype(float)
    state      = ['idle'] * N_DRONES   # idle | flying | returning | charging | waiting | stranded
    target     = [None] * N_DRONES
    deliveries = np.zeros(N_DRONES, dtype=int)
    fly_time   = np.zeros(N_DRONES)
    stranded = aborted = charge_visits = 0
    wait_time = 0.0

    soc_hist   = [[] for _ in range(N_DRONES)]
    state_hist = [[] for _ in range(N_DRONES)]
    queue_ts   = []

    pads             = [None] * N_PADS
    charge_remaining = [0.0] * N_PADS
    queue            = deque()
    mission_queue    = deque(random_waypoint(rng) for _ in range(200))

    def start_return(i):
        state[i] = 'returning'
        target[i] = BASE.copy()

    def dock(i):
        nonlocal charge_visits
        charge_visits += 1
        free = next((k for k, v in enumerate(pads) if v is None), None)
        if free is not None:
            pads[free] = i
            charge_remaining[free] = (1.0 - soc[i]) * E_CAP / P_CHG
            state[i] = 'charging'
        else:
            queue.append(i)
            state[i] = 'waiting'

    def fly_leg(i):
        """Move drone i one step toward target; returns True when the target is reached."""
        nonlocal stranded
        to_tgt = target[i] - pos[i]
        d_rem = float(np.linalg.norm(to_tgt))
        step = V_DRONE * DT
        d = min(step, d_rem)
        if soc[i] - dpm * d < 0.0:                    # battery empties on this step
            pos[i] += to_tgt / max(d_rem, 1e-9) * (soc[i] / dpm)
            soc[i] = 0.0
            if state[i] == 'flying':
                mission_queue.appendleft(target[i])   # parcel lost; order re-queued
            state[i] = 'stranded'
            stranded += 1
            return False
        soc[i] -= dpm * d
        if d_rem <= step:
            pos[i] = target[i].copy()
            return True
        pos[i] += to_tgt / d_rem * step
        return False

    for _ in range(int(round(T_SIM / DT))):
        t_now = _ * DT
        # Assign missions to idle (fully charged) drones at base
        for i in range(N_DRONES):
            if state[i] == 'idle':
                target[i] = mission_queue.popleft()
                state[i] = 'flying'

        for i in range(N_DRONES):
            if state[i] == 'flying':
                # Mid-leg return decision
                if policy == 'threshold' and soc[i] <= beta_threshold:
                    mission_queue.appendleft(target[i]); aborted += 1
                    start_return(i)
                elif policy == 'predictive' and soc[i] <= min_soc_to_return(pos[i]):
                    mission_queue.appendleft(target[i]); aborted += 1
                    start_return(i)
            if state[i] == 'flying':
                fly_time[i] += DT
                if fly_leg(i):
                    deliveries[i] += 1
                    mission_queue.append(random_waypoint(rng))
                    # Decision point after a completed delivery
                    go_home = soc[i] <= beta_threshold if policy in ('threshold', 'greedy') \
                        else soc[i] <= min_soc_to_return(pos[i])
                    if go_home:
                        start_return(i)
                    else:
                        target[i] = mission_queue.popleft()
            elif state[i] == 'returning':
                fly_time[i] += DT
                if fly_leg(i):
                    dock(i)
            elif state[i] == 'waiting':
                wait_time += DT

        for k in range(N_PADS):
            if pads[k] is not None:
                i = pads[k]
                charge_remaining[k] -= DT
                soc[i] = min(1.0, soc[i] + P_CHG * DT / E_CAP)
                if charge_remaining[k] <= 1e-9:
                    soc[i] = 1.0
                    state[i] = 'idle'
                    pads[k] = None
                    if queue:
                        nxt = queue.popleft()
                        pads[k] = nxt
                        charge_remaining[k] = (1.0 - soc[nxt]) * E_CAP / P_CHG
                        state[nxt] = 'charging'

        if record:
            for i in range(N_DRONES):
                soc_hist[i].append((t_now + DT, soc[i]))
                state_hist[i].append((t_now, state[i]))
            queue_ts.append((t_now + DT, len(queue)))

    out = {
        'deliveries': int(deliveries.sum()),
        'utilisation': float(np.mean(fly_time / T_SIM)),
        'stranded': stranded, 'aborted': aborted,
        'charge_visits': charge_visits, 'wait_time': wait_time,
    }
    if record:
        out['drone_data'] = [{'soc': soc_hist[i], 'state': state_hist[i],
                              'deliveries': int(deliveries[i])} for i in range(N_DRONES)]
        out['queue_ts'] = queue_ts
    return out


def monte_carlo(beta, policy):
    """Average metrics over N_SEEDS delivery streams (same streams for every policy)."""
    runs = [run_simulation(beta, policy, seed=SEED0 + s) for s in range(N_SEEDS)]
    keys = ['deliveries', 'utilisation', 'stranded', 'aborted', 'charge_visits', 'wait_time']
    res = {k: float(np.mean([r[k] for r in runs])) for k in keys}
    res['deliveries_std'] = float(np.std([r['deliveries'] for r in runs]))
    return res


# ── Plots ──────────────────────────────────────
def plot_threshold_sweep(sweep, pred, best, out_dir):
    """Mean total deliveries vs beta for threshold and greedy policies; predictive as a line."""
    fig, axes = plt.subplots(1, 2, figsize=(13, 4.5))
    ax = axes[0]
    for pol in ('threshold', 'greedy'):
        m = [sweep[pol][b]['deliveries'] for b in BETA_SWEEP]
        s = [sweep[pol][b]['deliveries_std'] for b in BETA_SWEEP]
        ax.errorbar(BETA_SWEEP, m, yerr=s, marker='o', capsize=3,
                    color=POLICY_COLORS[pol], label=f'{pol} (best beta={best[pol]:.2f})')
    ax.axhline(pred['deliveries'], color=POLICY_COLORS['predictive'], ls='--',
               label=f"predictive (no beta) = {pred['deliveries']:.1f}")
    ax.set_xlabel('Return threshold beta'); ax.set_ylabel(f'Mean total deliveries ({N_SEEDS} seeds)')
    ax.set_title('Deliveries vs return threshold'); ax.set_xticks(BETA_SWEEP)
    ax.legend(fontsize=8); ax.grid(alpha=0.3)
    ax = axes[1]
    for pol in ('threshold', 'greedy'):
        ax.plot(BETA_SWEEP, [sweep[pol][b]['stranded'] for b in BETA_SWEEP], marker='s',
                color=POLICY_COLORS[pol], label=pol)
    ax.axhline(pred['stranded'], color=POLICY_COLORS['predictive'], ls='--', label='predictive')
    ax.set_xlabel('Return threshold beta'); ax.set_ylabel('Mean stranded drones per run')
    ax.set_title('Stranded drones (battery empty before reaching base)')
    ax.set_xticks(BETA_SWEEP); ax.legend(fontsize=8); ax.grid(alpha=0.3)
    fig.suptitle('S032 - Threshold sweep', fontsize=12)
    fig.tight_layout()
    path = os.path.join(out_dir, 'threshold_sweep.png')
    plt.savefig(path, dpi=130); plt.close()
    print(f'Saved: {path}')


def plot_policy_comparison(policy_results, out_dir):
    """Bars: deliveries, utilisation, stranded, queue waiting time per policy."""
    labels = [r['label'] for r in policy_results]
    cols = [POLICY_COLORS[r['policy']] for r in policy_results]
    panels = [('deliveries', 'Mean total deliveries', '{:.1f}'),
              ('utilisation', 'Mean fleet utilisation (%)', '{:.1f}'),
              ('stranded', 'Mean stranded drones', '{:.2f}'),
              ('wait_time', 'Mean total queue waiting (drone-s)', '{:.0f}')]
    fig, axes = plt.subplots(1, 4, figsize=(22, 4.8))
    for ax, (key, title, fmt) in zip(axes, panels):
        vals = [r['mc'][key] * (100 if key == 'utilisation' else 1) for r in policy_results]
        bars = ax.bar(range(len(vals)), vals, color=cols,
                      hatch=[('//' if 'safe' in lb else '') for lb in labels], edgecolor='white')
        ax.set_xticks(range(len(vals))); ax.set_xticklabels(labels, fontsize=8)
        ax.bar_label(bars, labels=[fmt.format(v) for v in vals], padding=2, fontsize=9)
        if key == 'deliveries':
            ax.errorbar(range(len(vals)), vals,
                        yerr=[r['mc']['deliveries_std'] for r in policy_results],
                        fmt='none', ecolor='k', capsize=4)
        ax.set_title(title, fontsize=10); ax.grid(axis='y', alpha=0.3)
        ax.set_ylim(0, max(max(vals), 1e-6) * 1.2)
    fig.suptitle(f'S032 - Policy comparison (mean of {N_SEEDS} seeds)', fontsize=12)
    fig.tight_layout()
    path = os.path.join(out_dir, 'policy_comparison.png')
    plt.savefig(path, dpi=130); plt.close()
    print(f'Saved: {path}')


def plot_soc_traces(rec, out_dir):
    """SoC over time for all drones, one panel per policy (seed 42)."""
    cmap = plt.get_cmap('tab10')
    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    for ax, (label, pol, r) in zip(axes, rec):
        for i, dd in enumerate(r['drone_data']):
            ts = [p[0] for p in dd['soc']]; ss = [p[1] for p in dd['soc']]
            ax.plot(ts, ss, color=cmap(i), lw=0.9, label=f'D{i}')
        ax.axhline(BETA_SAFE, color='red', ls=':', lw=0.9, label='beta_safe')
        ax.set_ylabel('SoC'); ax.set_ylim(-0.05, 1.05); ax.grid(alpha=0.3)
        ax.set_title(f"{label}: {r['deliveries']} deliveries, {r['stranded']} stranded", fontsize=10)
    axes[0].legend(ncol=7, fontsize=7, loc='lower right')
    axes[-1].set_xlabel('Time (s)')
    fig.suptitle('S032 - SoC traces (seed 42)', fontsize=12)
    fig.tight_layout()
    path = os.path.join(out_dir, 'soc_traces.png')
    plt.savefig(path, dpi=130); plt.close()
    print(f'Saved: {path}')


def plot_queue_length(rec, out_dir):
    """Number of drones waiting for a pad vs time, per policy (seed 42)."""
    fig, ax = plt.subplots(figsize=(11, 3.5))
    for label, pol, r in rec:
        ts = [p[0] for p in r['queue_ts']]; q = [p[1] for p in r['queue_ts']]
        ax.step(ts, q, where='post', color=POLICY_COLORS[pol], lw=1.3, label=label)
    ax.set_xlabel('Time (s)'); ax.set_ylabel('Drones waiting')
    ax.set_title('S032 - Charging-pad queue length (seed 42)')
    ax.legend(fontsize=8); ax.grid(alpha=0.3)
    fig.tight_layout()
    path = os.path.join(out_dir, 'queue_length.png')
    plt.savefig(path, dpi=130); plt.close()
    print(f'Saved: {path}')


STATE_COLORS = {'idle': '#9ecae1', 'flying': '#4daf4a', 'returning': '#ff7f00',
                'charging': '#e31a1c', 'waiting': '#984ea3', 'stranded': '#000000'}


def plot_state_gantt(rec, out_dir):
    """Gantt chart of each drone's state, one panel per policy (seed 42)."""
    fig, axes = plt.subplots(3, 1, figsize=(13, 8), sharex=True)
    for ax, (label, pol, r) in zip(axes, rec):
        for i, dd in enumerate(r['drone_data']):
            st = dd['state']
            t0, s0 = st[0]
            for t, s in st[1:] + [(T_SIM, None)]:
                if s != s0:
                    ax.barh(i, t - t0, left=t0, height=0.6, color=STATE_COLORS[s0])
                    t0, s0 = t, s
        ax.set_yticks(range(N_DRONES)); ax.set_yticklabels([f'D{i}' for i in range(N_DRONES)])
        ax.set_title(label, fontsize=10); ax.set_xlim(0, T_SIM)
    patches = [mpatches.Patch(color=v, label=k) for k, v in STATE_COLORS.items()]
    axes[0].legend(handles=patches, ncol=6, fontsize=8, loc='upper right',
                   bbox_to_anchor=(1.0, 1.35))
    axes[-1].set_xlabel('Time (s)')
    fig.suptitle('S032 - Drone state timeline (seed 42)', fontsize=12, y=0.995)
    fig.tight_layout()
    path = os.path.join(out_dir, 'state_gantt.png')
    plt.savefig(path, dpi=130); plt.close()
    print(f'Saved: {path}')


def save_animation(rec, out_dir):
    """Animated SoC bars (coloured by state) for the three policies, 150 frames."""
    import matplotlib.animation as animation
    n = len(rec[0][2]['queue_ts'])
    frames = list(range(0, n, 4))
    fig, axes = plt.subplots(1, 3, figsize=(13, 4))
    bar_sets, txts = [], []
    for ax, (label, pol, r) in zip(axes, rec):
        bars = ax.bar(range(N_DRONES), np.ones(N_DRONES), color='#4daf4a')
        ax.axhline(BETA_SAFE, color='red', ls=':', lw=1)
        ax.set_ylim(0, 1.05); ax.set_xticks(range(N_DRONES))
        ax.set_xticklabels([f'D{i}' for i in range(N_DRONES)])
        ax.set_title(label.replace('\n', ' '), fontsize=9); ax.set_ylabel('SoC')
        bar_sets.append(bars)
        txts.append(ax.text(0.02, 0.97, '', transform=ax.transAxes, va='top', fontsize=8))
    patches = [mpatches.Patch(color=v, label=k) for k, v in STATE_COLORS.items()]
    fig.legend(handles=patches, ncol=6, fontsize=8, loc='lower center')
    ttl = fig.suptitle('')

    def update(fi):
        f = frames[fi]
        ttl.set_text(f'S032 Charging Queue - t = {(f + 1) * DT:.1f} s')
        for bars, txt, (label, pol, r) in zip(bar_sets, txts, rec):
            for i, b in enumerate(bars):
                b.set_height(r['drone_data'][i]['soc'][f][1])
                b.set_color(STATE_COLORS[r['drone_data'][i]['state'][f][1]])
            txt.set_text(f"queue = {r['queue_ts'][f][1]}")
        return [b for bs in bar_sets for b in bs] + txts + [ttl]

    fig.tight_layout(rect=(0, 0.08, 1, 0.94))
    ani = animation.FuncAnimation(fig, update, frames=len(frames), interval=50, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer='pillow', fps=20, dpi=80)
    plt.close()
    print(f'Saved: {path}  ({len(frames)} frames)')


if __name__ == '__main__':
    out_dir = os.path.normpath(OUTPUT_DIR)
    os.makedirs(out_dir, exist_ok=True)
    dpm = discharge_per_metre()
    print(f'Discharge per metre = {dpm:.5f} E_cap/m  -> full-charge range = {1/dpm:.0f} m '
          f'({1/dpm/V_DRONE:.1f} s of flight); full charge from empty = {E_CAP/P_CHG:.0f} s')

    print(f'\nThreshold sweep ({N_SEEDS} seeds each) ...')
    sweep = {'threshold': {}, 'greedy': {}}
    for pol in ('threshold', 'greedy'):
        for b in BETA_SWEEP:
            sweep[pol][b] = monte_carlo(b, pol)
    pred = monte_carlo(0.0, 'predictive')

    best, best_safe = {}, {}
    for pol in ('threshold', 'greedy'):
        print(f'\n  {pol}:')
        for b in BETA_SWEEP:
            r = sweep[pol][b]
            print(f"    beta={b:.2f}  deliveries={r['deliveries']:6.1f} +/- {r['deliveries_std']:4.1f}"
                  f"  stranded={r['stranded']:.2f}  aborted={r['aborted']:5.2f}"
                  f"  charge visits={r['charge_visits']:5.2f}  wait={r['wait_time']:6.1f} s")
        best[pol] = float(BETA_SWEEP[int(np.argmax([sweep[pol][b]['deliveries'] for b in BETA_SWEEP]))])
        safe = [b for b in BETA_SWEEP if sweep[pol][b]['stranded'] == 0.0]
        best_safe[pol] = float(max(safe, key=lambda b: sweep[pol][b]['deliveries']))
        print(f'    beta* (max deliveries)            = {best[pol]:.2f}')
        print(f'    beta*_safe (max deliveries, 0 stranded in all seeds) = {best_safe[pol]:.2f}')

    policy_results = [
        {'label': f"Threshold\n(beta*={best['threshold']:.2f})", 'policy': 'threshold',
         'beta': best['threshold'], 'mc': sweep['threshold'][best['threshold']]},
        {'label': f"Threshold safe\n(beta={best_safe['threshold']:.2f})", 'policy': 'threshold',
         'beta': best_safe['threshold'], 'mc': sweep['threshold'][best_safe['threshold']]},
        {'label': f"Greedy\n(beta*={best['greedy']:.2f})", 'policy': 'greedy',
         'beta': best['greedy'], 'mc': sweep['greedy'][best['greedy']]},
        {'label': f"Greedy safe\n(beta={best_safe['greedy']:.2f})", 'policy': 'greedy',
         'beta': best_safe['greedy'], 'mc': sweep['greedy'][best_safe['greedy']]},
        {'label': 'Predictive', 'policy': 'predictive', 'beta': 0.0, 'mc': pred},
    ]
    print('\nPolicy comparison (mean of seeds):')
    for p in policy_results:
        m = p['mc']
        print(f"  {p['label'].replace(chr(10), ' '):<24} deliveries={m['deliveries']:6.1f} +/- "
              f"{m['deliveries_std']:4.1f}  util={m['utilisation']*100:5.1f}%  "
              f"stranded={m['stranded']:.2f}  aborted={m['aborted']:5.2f}  "
              f"charge visits={m['charge_visits']:5.2f}  queue wait={m['wait_time']:6.1f} drone-s")

    rec = []
    print('\nRecorded runs (seed 42):')
    for p in (policy_results[0], policy_results[2], policy_results[4]):
        r = run_simulation(p['beta'], p['policy'], seed=SEED0, record=True)
        rec.append((p['label'], p['policy'], r))
        print(f"  {p['label'].replace(chr(10), ' '):<24} deliveries={r['deliveries']}  "
              f"stranded={r['stranded']}  max queue={max(q for _, q in r['queue_ts'])}")

    plot_threshold_sweep(sweep, pred, best, out_dir)
    plot_policy_comparison(policy_results, out_dir)
    plot_soc_traces(rec, out_dir)
    plot_queue_length(rec, out_dir)
    plot_state_gantt(rec, out_dir)
    save_animation(rec, out_dir)
