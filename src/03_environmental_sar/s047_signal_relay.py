"""
S047 Base Station Signal Relay Enhancement
===========================================
Three relay drones are positioned to form a communication chain between a ground
base station and a rescue team deep in a mountain valley where direct line-of-sight
is blocked by terrain. The goal is to find relay positions that maximise the minimum
(bottleneck) single-hop SNR across all four links in the chain. Three strategies are
compared: uniform spacing, greedy hop-by-hop (each relay maximises its own hop SNR
given the previous node, under a fixed progress requirement), and gradient ascent on
the soft-min SNR.

Usage:
    conda activate drones
    python src/03_environmental_sar/s047_signal_relay.py
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.animation as animation

# ── Parameters ────────────────────────────────────────────────────────────────
N_RELAYS     = 3          # number of relay drones
F_C          = 2.4e9      # Hz  — carrier frequency (2.4 GHz ISM band)
C_LIGHT      = 3e8        # m/s — speed of light
D_0          = 1.0        # m   — reference distance
N_LOSS       = 2.8        # path loss exponent (mixed terrain / vegetation)
P_TX         = 30.0       # dBm — transmit power (all nodes)
N_FLOOR      = -95.0      # dBm — receiver noise floor
SNR_MIN      = 10.0       # dB  — minimum viable SNR
D_MAX        = 600.0      # m   — maximum comm range per hop
DELTA_SHADOW = 15.0       # dB  — terrain shadowing penalty
ALPHA_SOFT   = 0.5        # soft-min temperature
ETA          = 5.0        # m   — gradient ascent step size
N_ITER       = 2000       # gradient ascent iterations
RECORD_EVERY = 20         # store relay positions every N ascent steps (animation)
GREEDY_GRID  = 1.0        # m   — candidate-grid resolution of the greedy search

# Fixed endpoints
BS   = np.array([100.0, 500.0])   # base station
TEAM = np.array([900.0, 500.0])   # rescue team in valley

# Shadow zones: (centre_x, centre_y, half_width, half_height)
SHADOW_ZONES = [
    (350.0, 480.0,  80.0, 120.0),   # ridge 1
    (500.0, 520.0,  70.0, 100.0),   # valley wall centre
    (650.0, 460.0,  90.0, 130.0),   # ridge 2
]

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '03_environmental_sar', 's047_signal_relay',
)

# ── Helpers ───────────────────────────────────────────────────────────────────

def L0_db(d0=D_0, fc=F_C):
    """Free-space path loss at reference distance d0."""
    return 20.0 * np.log10(4.0 * np.pi * d0 * fc / C_LIGHT)


L0 = L0_db()


def in_shadow(midpoint, zones=SHADOW_ZONES):
    """Return True if midpoint lies inside any shadow zone rectangle."""
    mx, my = midpoint
    for (cx, cy, hw, hh) in zones:
        if abs(mx - cx) <= hw and abs(my - cy) <= hh:
            return True
    return False


def path_loss_db(d, midpoint):
    """Log-distance path loss (dB) with terrain shadowing penalty."""
    d = max(float(d), 1e-3)
    pl = L0 + 10.0 * N_LOSS * np.log10(d / D_0)
    if in_shadow(midpoint):
        pl += DELTA_SHADOW
    return pl


def snr_link(p_a, p_b):
    """SNR (dB) for a single hop from p_a to p_b."""
    d = float(np.linalg.norm(p_b - p_a))
    mid = 0.5 * (p_a + p_b)
    return P_TX - path_loss_db(d, mid) - N_FLOOR


def chain_snrs(positions):
    """Compute per-link SNR for a chain of node positions (shape: N×2)."""
    return np.array([snr_link(positions[k], positions[k + 1])
                     for k in range(len(positions) - 1)])


def soft_min(snrs, alpha=ALPHA_SOFT):
    """Smooth approximation of min(snrs) using log-sum-exp."""
    snrs = np.asarray(snrs, dtype=float)
    return -(1.0 / alpha) * np.log(np.sum(np.exp(-alpha * snrs)))


def uniform_spacing():
    """Place relays at equal fractional distances along BS→TEAM line."""
    return np.array([
        BS + (i / (N_RELAYS + 1)) * (TEAM - BS)
        for i in range(1, N_RELAYS + 1)
    ], dtype=float)


# ── Strategies ────────────────────────────────────────────────────────────────

def enforce_range(relays):
    """
    Project relay positions so that every hop of BS-R1-R2-R3-TEAM is at most
    D_MAX: a forward pass clips each relay to D_MAX from its predecessor, a
    backward pass clips each relay to D_MAX from its successor (this also
    covers the last hop R3 -> TEAM).
    """
    nodes = np.vstack([BS, relays, TEAM])
    for i in range(1, N_RELAYS + 1):                 # forward pass
        v = nodes[i] - nodes[i - 1]; d = float(np.linalg.norm(v))
        if d > D_MAX:
            nodes[i] = nodes[i - 1] + v / d * D_MAX
    for i in range(N_RELAYS, 0, -1):                 # backward pass
        v = nodes[i] - nodes[i + 1]; d = float(np.linalg.norm(v))
        if d > D_MAX:
            nodes[i] = nodes[i + 1] + v / d * D_MAX
    return nodes[1:N_RELAYS + 1].copy()


def _snr_vec(p_a, cand):
    """Vectorised SNR (dB) from point p_a to every candidate point cand (M x 2)."""
    d = np.maximum(np.linalg.norm(cand - p_a, axis=1), 1e-3)
    mid = 0.5 * (cand + p_a)
    shadow = np.zeros(len(cand), dtype=bool)
    for (cx, cy, hw, hh) in SHADOW_ZONES:
        shadow |= (np.abs(mid[:, 0] - cx) <= hw) & (np.abs(mid[:, 1] - cy) <= hh)
    pl = L0 + 10.0 * N_LOSS * np.log10(d / D_0) + DELTA_SHADOW * shadow
    return P_TX - pl - N_FLOOR


def greedy_hop_by_hop():
    """
    Greedy hop-by-hop placement: relays are placed one at a time from the BS
    side.  Relay i is put at the grid point that maximises the SNR of its own
    hop (previous node -> relay i), ignoring all downstream hops, subject to
      * a progress requirement |p_i - TEAM| <= |TEAM - BS| (1 - i / (N+1))
        (without it the current-hop SNR is maximised by not moving at all), and
      * the range limit |p_i - p_{i-1}| <= D_MAX.
    The last hop R3 -> TEAM is never optimised; it is whatever results.
    """
    xs = np.arange(0.0, 1000.0 + GREEDY_GRID / 2, GREEDY_GRID)
    gx, gy = np.meshgrid(xs, xs)
    cand = np.column_stack([gx.ravel(), gy.ravel()])
    d_total = float(np.linalg.norm(TEAM - BS))
    prev = BS.copy()
    relays = []
    for i in range(1, N_RELAYS + 1):
        r_max = d_total * (1.0 - i / (N_RELAYS + 1))
        ok = ((np.linalg.norm(cand - TEAM, axis=1) <= r_max + 1e-9) &
              (np.linalg.norm(cand - prev, axis=1) <= D_MAX))
        c = cand[ok]
        snr = _snr_vec(prev, c)
        best = c[int(np.argmax(snr))]
        relays.append(best)
        prev = best
    return np.array(relays)


def gradient_ascent():
    """
    Gradient ascent on the soft-min SNR from the uniform layout (central
    differences, eps = 1 m), step ETA, N_ITER iterations, range projection and
    operating-area clipping after every step.
    """
    relays = uniform_spacing()
    history = []
    relay_traj = [relays.copy()]
    for it in range(N_ITER):
        snrs = chain_snrs(np.vstack([BS, relays, TEAM]))
        history.append(float(np.min(snrs)))
        eps = 1.0
        grad = np.zeros_like(relays)
        for i in range(N_RELAYS):
            for j in range(2):
                r_fwd = relays.copy(); r_fwd[i, j] += eps
                r_bwd = relays.copy(); r_bwd[i, j] -= eps
                J_fwd = soft_min(chain_snrs(np.vstack([BS, r_fwd, TEAM])))
                J_bwd = soft_min(chain_snrs(np.vstack([BS, r_bwd, TEAM])))
                grad[i, j] = (J_fwd - J_bwd) / (2.0 * eps)
        relays = np.clip(enforce_range(relays + ETA * grad), 0.0, 1000.0)
        if (it + 1) % RECORD_EVERY == 0:
            relay_traj.append(relays.copy())
    history.append(float(np.min(chain_snrs(np.vstack([BS, relays, TEAM])))))
    return relays, np.array(relay_traj), np.array(history)


# ── Simulation ────────────────────────────────────────────────────────────────

def run_simulation():
    """
    Run all three relay-placement strategies.
    Returns (results_dict, relay_trajectory of the gradient ascent, history),
    where history[k] is the bottleneck SNR after k ascent steps.
    """
    uniform = uniform_spacing()
    greedy = greedy_hop_by_hop()
    opt_relays, relay_traj, history = gradient_ascent()
    results = {}
    for name, pos in (('uniform', uniform), ('greedy', greedy), ('optimised', opt_relays)):
        chain = np.vstack([BS, pos, TEAM])
        results[name] = {
            'positions': pos,
            'snrs': chain_snrs(chain),
            'hops': np.linalg.norm(np.diff(chain, axis=0), axis=1),
        }
    return results, relay_traj, history


# ── Plot helpers ──────────────────────────────────────────────────────────────

STRATEGY_COLORS = {
    'uniform':   '#2196F3',   # blue
    'greedy':    '#FF9800',   # orange
    'optimised': '#4CAF50',   # green
}
STRATEGY_LABELS = {
    'uniform':   'Uniform Spacing',
    'greedy':    'Greedy Hop-by-Hop',
    'optimised': 'Gradient Ascent (Opt)',
}


def _draw_terrain(ax):
    """Draw shadow zones and fixed endpoints on a 2D axes."""
    for (cx, cy, hw, hh) in SHADOW_ZONES:
        rect = mpatches.Rectangle(
            (cx - hw, cy - hh), 2 * hw, 2 * hh,
            linewidth=1, edgecolor='#555', facecolor='#aaa', alpha=0.55,
            zorder=1, label='_nolegend_'
        )
        ax.add_patch(rect)
        ax.text(cx, cy, 'Shadow\nZone', ha='center', va='center',
                fontsize=7, color='#333', zorder=2)

    ax.plot(*BS,   's', color='black',  markersize=10, zorder=5, label='Base Station')
    ax.plot(*TEAM, '^', color='green',  markersize=10, zorder=5, label='Rescue Team')


LABEL_DY = {'uniform': 15, 'greedy': 15, 'optimised': -35}   # SNR label offsets (m)


def plot_terrain_map(results, out_dir):
    """2D terrain map showing relay positions and link lines for each strategy."""
    fig, ax = plt.subplots(figsize=(10, 7))
    _draw_terrain(ax)

    for strategy, data in results.items():
        color = STRATEGY_COLORS[strategy]
        label = STRATEGY_LABELS[strategy]
        relays = data['positions']
        snrs   = data['snrs']
        chain  = np.vstack([BS, relays, TEAM])

        # Draw link lines
        for k in range(len(chain) - 1):
            ax.plot([chain[k, 0], chain[k + 1, 0]],
                    [chain[k, 1], chain[k + 1, 1]],
                    color=color, linewidth=1.8, alpha=0.7, zorder=3)
            # Annotate with SNR
            mid_x = 0.5 * (chain[k, 0] + chain[k + 1, 0])
            mid_y = 0.5 * (chain[k, 1] + chain[k + 1, 1]) + LABEL_DY[strategy]
            ax.text(mid_x, mid_y, f'{snrs[k]:.1f} dB',
                    fontsize=7, color=color, ha='center', zorder=6,
                    bbox=dict(facecolor='white', alpha=0.5, edgecolor='none', pad=1))

        # Draw relay markers
        ax.scatter(relays[:, 0], relays[:, 1],
                   color=color, s=80, zorder=5, marker='o',
                   edgecolors='white', linewidths=0.8, label=label)
        for i, (rx, ry) in enumerate(relays):
            ax.text(rx + 8, ry + (12 if LABEL_DY[strategy] > 0 else -28), f'R{i+1}',
                    fontsize=7, color=color, zorder=6)

    ax.set_xlim(-30, 1030)
    ax.set_ylim(250, 750)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('Relay Drone Positions — Terrain Map')
    ax.legend(loc='upper right', fontsize=8)
    ax.set_aspect('equal')
    ax.grid(True, linestyle='--', alpha=0.3)

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'terrain_map.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_snr_bar_chart(results, out_dir):
    """Grouped bar chart of per-link SNR for each strategy."""
    n_links = 4
    x = np.arange(n_links)
    width = 0.25
    offsets = [-width, 0, width]

    fig, ax = plt.subplots(figsize=(9, 5))
    for idx, (strategy, data) in enumerate(results.items()):
        snrs  = data['snrs']
        color = STRATEGY_COLORS[strategy]
        bars  = ax.bar(x + offsets[idx], snrs, width,
                       label=STRATEGY_LABELS[strategy],
                       color=color, alpha=0.85, edgecolor='white', linewidth=0.6)
        # Highlight bottleneck link with red outline
        for bot_idx in np.flatnonzero(snrs <= snrs.min() + 1e-6):   # all tied minima
            bars[bot_idx].set_edgecolor('red')
            bars[bot_idx].set_linewidth(2.0)

    ax.axhline(SNR_MIN, color='red', linewidth=1.5, linestyle='--',
               label=f'SNR_min = {SNR_MIN} dB')
    handles = [mpatches.Patch(color=STRATEGY_COLORS[s_], label=STRATEGY_LABELS[s_])
               for s_ in results]
    handles.append(mpatches.Patch(facecolor='white', edgecolor='red', linewidth=2,
                                  label='bottleneck link'))
    handles.append(plt.Line2D([], [], color='red', ls='--', label=f'SNR_min = {SNR_MIN} dB'))
    ax.set_xticks(x)
    ax.set_xticklabels([f'Link {k} (hop {k}→{k+1})' for k in range(n_links)])
    ax.set_ylabel('SNR (dB)')
    ax.set_title('Per-Link SNR — Strategy Comparison')
    ax.legend(handles=handles, fontsize=8, loc='upper center', ncol=3,
              bbox_to_anchor=(0.5, -0.12))
    ax.grid(True, axis='y', linestyle='--', alpha=0.3)
    ax.set_ylim(bottom=min(0, ax.get_ylim()[0]) - 2)

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'snr_bar_chart.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_convergence(history, out_dir):
    """Bottleneck SNR vs gradient ascent iteration."""
    fig, ax = plt.subplots(figsize=(8, 4))
    ax.plot(np.arange(len(history)), history, color='#4CAF50', linewidth=1.5)
    ax.axhline(SNR_MIN, color='red', linewidth=1.2, linestyle='--',
               label=f'SNR_min = {SNR_MIN} dB')
    ax.axhline(history[-1], color='gray', linewidth=0.8, linestyle=':',
               label=f'Final = {history[-1]:.2f} dB')
    ax.set_xlabel('Gradient Ascent Iteration')
    ax.set_ylabel('Bottleneck SNR (dB)')
    ax.set_title('Optimisation Convergence — Bottleneck SNR')
    ax.legend(fontsize=8)
    ax.grid(True, linestyle='--', alpha=0.3)

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'convergence.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_bottleneck_comparison(results, out_dir):
    """Horizontal bar chart of bottleneck SNR for each strategy."""
    strategies = list(results.keys())
    bottlenecks = [float(np.min(results[s]['snrs'])) for s in strategies]
    colors = [STRATEGY_COLORS[s] for s in strategies]
    labels = [STRATEGY_LABELS[s] for s in strategies]

    fig, ax = plt.subplots(figsize=(8, 3.5))
    bars = ax.barh(labels, bottlenecks, color=colors, alpha=0.85,
                   edgecolor='white', linewidth=0.8)
    ax.axvline(SNR_MIN, color='red', linewidth=1.5, linestyle='--',
               label=f'SNR_min = {SNR_MIN} dB')
    for bar, val in zip(bars, bottlenecks):
        ax.text(val + 0.3, bar.get_y() + bar.get_height() / 2,
                f'{val:.2f} dB', va='center', fontsize=9)
    ax.set_xlim(0, max(bottlenecks) * 1.15)
    ax.set_xlabel('Bottleneck SNR (dB)')
    ax.set_title('Bottleneck SNR — Strategy Summary')
    ax.legend(fontsize=8)
    ax.grid(True, axis='x', linestyle='--', alpha=0.3)

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'bottleneck_comparison.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def save_animation(relay_traj, history, out_dir):
    """
    Animate how relay positions migrate across the terrain during gradient ascent.
    relay_traj shape: (frames, 3, 2)
    """
    n_frames = len(relay_traj)
    colors_relay = ['#E91E63', '#9C27B0', '#F44336']  # R1, R2, R3

    fig, ax = plt.subplots(figsize=(9, 6))
    _draw_terrain(ax)

    # Trail lines for each relay
    trail_lines = [ax.plot([], [], '-', color=colors_relay[i], alpha=0.35,
                           linewidth=0.9)[0] for i in range(N_RELAYS)]
    # Relay markers
    relay_markers = [ax.plot([], [], 'o', color=colors_relay[i],
                             markersize=9, zorder=7,
                             label=f'R{i+1}')[0] for i in range(N_RELAYS)]
    # Chain link lines
    link_lines = [ax.plot([], [], '-', color='#555', linewidth=1.0,
                          alpha=0.6)[0] for _ in range(4)]

    ax.set_xlim(-30, 1030)
    ax.set_ylim(250, 750)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_aspect('equal')
    ax.grid(True, linestyle='--', alpha=0.3)
    ax.legend(loc='upper right', fontsize=8)

    iter_text = ax.text(0.02, 0.97, '', transform=ax.transAxes,
                        fontsize=9, va='top',
                        bbox=dict(facecolor='white', alpha=0.6, edgecolor='none'))

    def update(frame):
        relays = relay_traj[frame]
        chain  = np.vstack([BS, relays, TEAM])

        for i in range(N_RELAYS):
            past_x = relay_traj[:frame + 1, i, 0]
            past_y = relay_traj[:frame + 1, i, 1]
            trail_lines[i].set_data(past_x, past_y)
            relay_markers[i].set_data([relays[i, 0]], [relays[i, 1]])

        for k in range(4):
            link_lines[k].set_data(
                [chain[k, 0], chain[k + 1, 0]],
                [chain[k, 1], chain[k + 1, 1]],
            )

        iter_text.set_text(f'Iteration: {frame * RECORD_EVERY}  bottleneck {history[frame * RECORD_EVERY]:.2f} dB')
        return trail_lines + relay_markers + link_lines + [iter_text]

    ani = animation.FuncAnimation(
        fig, update, frames=n_frames, interval=80, blit=True
    )

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer='pillow', fps=12, dpi=100)
    plt.close()
    print(f'Saved: {path}')


# ── Main ──────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    results, relay_traj, history = run_simulation()

    print('\n=== S047 Signal Relay Enhancement — Results ===')
    for strategy, data in results.items():
        snrs = data['snrs']
        chain = np.vstack([BS, data['positions'], TEAM])
        print(f'  [{STRATEGY_LABELS[strategy]}]')
        print('    Relays: ' + ', '.join(f'R{i+1}=({p[0]:.1f}, {p[1]:.1f})'
                                       for i, p in enumerate(data['positions'])))
        for k, s_k in enumerate(snrs):
            viable = 'OK' if s_k >= SNR_MIN else 'FAIL'
            sh = 'shadowed' if in_shadow(0.5 * (chain[k] + chain[k + 1])) else 'clear'
            print(f'    Link {k}: {s_k:6.2f} dB  d = {data["hops"][k]:6.1f} m  {sh:8s} [{viable}]')
        print(f'    Bottleneck SNR: {np.min(snrs):.2f} dB   max hop {data["hops"].max():.1f} m '
              f'(limit {D_MAX:.0f} m)')
        print()

    print(f'  Gradient ascent: {N_ITER} iterations')
    print(f'  Initial bottleneck SNR: {history[0]:.2f} dB')
    print(f'  Best bottleneck SNR:    {history.max():.2f} dB (after {int(np.argmax(history))} steps)')
    print(f'  Final bottleneck SNR:   {history[-1]:.2f} dB')
    print(f'  Improvement (final - initial): {history[-1] - history[0]:.2f} dB')
    print(f'  Animation frames: {len(relay_traj)}')

    out = os.path.normpath(OUTPUT_DIR)
    plot_terrain_map(results, out)
    plot_snr_bar_chart(results, out)
    plot_convergence(history, out)
    plot_bottleneck_comparison(results, out)
    save_animation(relay_traj, history, out)
