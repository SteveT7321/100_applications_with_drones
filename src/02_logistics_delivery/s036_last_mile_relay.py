"""
S036 Last-Mile Relay
====================
Simulates a multi-drone relay chain delivering a parcel from a source depot to
a destination whose straight-line distance (120 m) exceeds any single drone's
effective range (31.5 m). Relay stations are placed optimally (equal-spacing on
the straight route) and via a greedy furthest-reachable strategy on a bent
corridor route forced by airspace restrictions. The simulation tracks the parcel
position over time, battery consumption per drone, and delivery-time breakdown.

Usage:
    conda activate drones
    python src/02_logistics_delivery/s036_last_mile_relay.py
"""

import sys, os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.gridspec as gridspec

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

# ── Parameters ─────────────────────────────────────────────────────────────────
TOTAL_DISTANCE = 120.0      # m   straight-line source-to-destination
DRONE_RANGE    = 35.0       # m   max range per full charge
RANGE_RESERVE  = 0.10       # 10% battery reserve
CRUISE_SPEED   = 5.0        # m/s
HANDOFF_DWELL  = 3.0        # s   parcel transfer + fresh drone spin-up
DT             = 0.05       # s   simulation timestep

R_EFF = DRONE_RANGE * (1.0 - RANGE_RESERVE)   # 31.5 m

SOURCE = np.array([0.0,   0.0])
DEST   = np.array([120.0, 0.0])

# Bent-corridor via-points (airspace restrictions)
VIA_POINTS = [
    np.array([30.0,  20.0]),   # divert north
    np.array([60.0,  10.0]),
    np.array([90.0, -15.0]),   # divert south
]

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '02_logistics_delivery', 's036_last_mile_relay',
)

RNG = np.random.default_rng(0)   # fixed seed for reproducibility


# ── Helpers ───────────────────────────────────────────────────────────────────
def greedy_segment(waypoints, r_eff):
    """Greedy furthest-reachable segmentation of an ordered waypoint list.

    Returns list of (start_idx, end_idx) tuples.
    """
    legs = []
    start_idx = 0
    while start_idx < len(waypoints) - 1:
        reachable = [
            i for i in range(start_idx + 1, len(waypoints))
            if np.linalg.norm(waypoints[i] - waypoints[start_idx]) <= r_eff
        ]
        if not reachable:
            raise ValueError(f"No reachable waypoint from index {start_idx}")
        end_idx = reachable[-1]
        legs.append((start_idx, end_idx))
        start_idx = end_idx
    return legs


def leg_distances(waypoints, legs):
    return [np.linalg.norm(waypoints[ei] - waypoints[si]) for si, ei in legs]


def delivery_time(legs, waypoints, speed, dwell):
    total = sum(
        np.linalg.norm(waypoints[ei] - waypoints[si]) / speed
        for si, ei in legs
    )
    total += (len(legs) - 1) * dwell
    return total


def simulate_parcel(waypoints, legs, speed, dwell, dt):
    """Simulate parcel movement through the relay chain.

    Returns arrays: times, positions (Nx2), leg indices.
    """
    times, positions, leg_ids = [], [], []
    t = 0.0
    for leg_num, (si, ei) in enumerate(legs):
        p_start = waypoints[si].astype(float)
        p_end   = waypoints[ei].astype(float)
        diff    = p_end - p_start
        dist    = np.linalg.norm(diff)
        direction = diff / dist
        n_steps = int(dist / speed / dt)
        for step in range(n_steps + 1):
            elapsed = step * dt
            pos = p_start + direction * speed * elapsed
            times.append(t + elapsed)
            positions.append(pos.copy())
            leg_ids.append(leg_num)
        # arrive at relay station
        t += dist / speed
        times.append(t)
        positions.append(p_end.copy())
        leg_ids.append(leg_num)
        # handoff dwell (not for final leg)
        if leg_num < len(legs) - 1:
            t += dwell
    return np.array(times), np.array(positions), np.array(leg_ids)


# ── Simulation ────────────────────────────────────────────────────────────────
def run_simulation():
    # ── Straight-line optimal relay ──
    straight_wps = [SOURCE]
    K_star = int(np.ceil(TOTAL_DISTANCE / R_EFF))   # 4 legs
    for k in range(1, K_star):
        frac = k / K_star
        straight_wps.append(SOURCE + frac * (DEST - SOURCE))
    straight_wps.append(DEST)
    straight_wps = np.array(straight_wps)

    straight_legs = list(zip(range(K_star), range(1, K_star + 1)))
    straight_dists = leg_distances(straight_wps, straight_legs)
    straight_batt  = [d / DRONE_RANGE for d in straight_dists]   # B_k = d_k / R
    straight_T     = delivery_time(straight_legs, straight_wps, CRUISE_SPEED, HANDOFF_DWELL)

    # ── Bent corridor: dense candidate waypoints along the via-point path ──
    # Build corridor waypoints: SOURCE → via-points → DEST
    corridor_raw = [SOURCE] + VIA_POINTS + [DEST]
    # Interpolate densely (every ~5 m) so greedy can snap to optimal cuts
    DENSE_SPACING = 5.0
    dense_wps = [SOURCE]
    for a, b in zip(corridor_raw[:-1], corridor_raw[1:]):
        seg_len = np.linalg.norm(b - a)
        n_pts   = max(2, int(np.ceil(seg_len / DENSE_SPACING)))
        for frac in np.linspace(0, 1, n_pts)[1:]:
            dense_wps.append(a + frac * (b - a))
    dense_wps = np.array(dense_wps)

    bent_legs  = greedy_segment(dense_wps, R_EFF)
    bent_dists = leg_distances(dense_wps, bent_legs)
    bent_batt  = [d / DRONE_RANGE for d in bent_dists]
    bent_T     = delivery_time(bent_legs, dense_wps, CRUISE_SPEED, HANDOFF_DWELL)

    # ── Straight naive equal-spacing (4 legs) as baseline for bent route ──
    # Equal-spacing ignores the via-points; show it would violate range
    naive_dists_bent = []
    naive_relay_x = [0, 30, 60, 90, 120]
    for i in range(len(naive_relay_x) - 1):
        naive_dists_bent.append(naive_relay_x[i+1] - naive_relay_x[i])   # x-only; actual must account for y-detour

    # ── Simulate straight-route parcel ──
    s_times, s_pos, s_leg_ids = simulate_parcel(
        straight_wps, straight_legs, CRUISE_SPEED, HANDOFF_DWELL, DT
    )

    # ── Handoff event times ──
    handoff_times = []
    t_acc = 0.0
    for k, (si, ei) in enumerate(straight_legs[:-1]):
        t_acc += straight_dists[k] / CRUISE_SPEED
        handoff_times.append(t_acc)
        t_acc += HANDOFF_DWELL

    return dict(
        straight_wps=straight_wps,
        straight_legs=straight_legs,
        straight_dists=straight_dists,
        straight_batt=straight_batt,
        straight_T=straight_T,
        dense_wps=dense_wps,
        bent_legs=bent_legs,
        bent_dists=bent_dists,
        bent_batt=bent_batt,
        bent_T=bent_T,
        s_times=s_times,
        s_pos=s_pos,
        s_leg_ids=s_leg_ids,
        handoff_times=handoff_times,
        K_star=K_star,
    )


# ── Plots ─────────────────────────────────────────────────────────────────────
LEG_COLORS = ['#E63946', '#2A9D8F', '#E9C46A', '#F4A261', '#264653',
              '#8338EC', '#3A86FF']


def plot_route_map(data, out_dir):
    """2-D top-down view: straight route and bent corridor route."""
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))

    # ─ Left: straight optimal relay ─
    ax = axes[0]
    wps  = data['straight_wps']
    legs = data['straight_legs']
    dists = data['straight_dists']

    for k, (si, ei) in enumerate(legs):
        color = LEG_COLORS[k % len(LEG_COLORS)]
        ax.plot([wps[si][0], wps[ei][0]], [wps[si][1], wps[ei][1]],
                color=color, lw=3, zorder=2)
        mid = (wps[si] + wps[ei]) / 2
        ax.annotate(f'{dists[k]:.1f} m\n({dists[k]/DRONE_RANGE*100:.0f}%)',
                    xy=mid, fontsize=8, ha='center', va='bottom',
                    color=color, fontweight='bold')

    # Relay stations
    for k, wp in enumerate(wps[1:-1], 1):
        ax.scatter(*wp, s=150, color='orange', zorder=4,
                   edgecolors='k', linewidths=1)
        ax.annotate(f'W{k}', xy=wp, xytext=(0, 8),
                    textcoords='offset points', ha='center', fontsize=9)

    ax.scatter(*wps[0], s=200, marker='s', color='green', zorder=5,
               edgecolors='k', linewidths=1.2, label='Source')
    ax.scatter(*wps[-1], s=200, marker='*', color='red', zorder=5,
               edgecolors='k', linewidths=1.2, label='Destination')

    ax.set_xlim(-10, 130)
    ax.set_ylim(-30, 30)
    ax.set_title(f'Straight Route  (K={data["K_star"]} legs, T={data["straight_T"]:.1f} s)',
                 fontsize=11)
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')
    ax.legend(fontsize=9); ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')

    # ─ Right: bent corridor greedy relay ─
    ax2 = axes[1]
    bwps  = data['dense_wps']
    blegs = data['bent_legs']
    bdists = data['bent_dists']

    # Draw corridor path
    ax2.plot(bwps[:, 0], bwps[:, 1], color='lightgray', lw=1.5,
             zorder=1, label='Corridor')

    for k, (si, ei) in enumerate(blegs):
        color = LEG_COLORS[k % len(LEG_COLORS)]
        ax2.plot([bwps[si][0], bwps[ei][0]], [bwps[si][1], bwps[ei][1]],
                 color=color, lw=3, zorder=2)
        mid = (bwps[si] + bwps[ei]) / 2
        ax2.annotate(f'{bdists[k]:.1f} m', xy=mid, fontsize=8,
                     ha='center', va='top', color=color, fontweight='bold')

    # Relay stations at leg cut-points
    relay_indices = sorted({ei for si, ei in blegs[:-1]})
    for idx in relay_indices:
        ax2.scatter(*bwps[idx], s=150, color='orange', zorder=4,
                    edgecolors='k', linewidths=1)

    ax2.scatter(*SOURCE, s=200, marker='s', color='green', zorder=5,
                edgecolors='k', linewidths=1.2, label='Source')
    ax2.scatter(*DEST, s=200, marker='*', color='red', zorder=5,
                edgecolors='k', linewidths=1.2, label='Destination')

    # Via-points
    for vp in VIA_POINTS:
        ax2.scatter(*vp, s=80, marker='^', color='purple', zorder=3,
                    edgecolors='k', linewidths=0.8)

    ax2.set_xlim(-10, 130)
    ylo = min(bwps[:, 1].min(), -5) - 10
    yhi = max(bwps[:, 1].max(), 5) + 10
    ax2.set_ylim(ylo, yhi)
    ax2.set_title(f'Bent Corridor  (K={len(blegs)} legs, T={data["bent_T"]:.1f} s)',
                  fontsize=11)
    ax2.set_xlabel('x (m)'); ax2.set_ylabel('y (m)')
    via_patch = mpatches.Patch(color='purple', label='Airspace restriction')
    relay_patch = mpatches.Patch(color='orange', label='Relay station')
    ax2.legend(handles=[
        mpatches.Patch(color='green', label='Source'),
        mpatches.Patch(color='red', label='Destination'),
        via_patch, relay_patch
    ], fontsize=9)
    ax2.grid(True, alpha=0.3)
    ax2.set_aspect('equal')

    fig.suptitle('S036 Last-Mile Relay — Route Maps', fontsize=13, fontweight='bold')
    plt.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'route_map.png')
    plt.savefig(path, dpi=150); plt.close()
    print(f'Saved: {path}')


def plot_parcel_position(data, out_dir):
    """Parcel x-coordinate vs time with handoff markers."""
    fig, ax = plt.subplots(figsize=(10, 4))

    times    = data['s_times']
    pos      = data['s_pos']
    leg_ids  = data['s_leg_ids']
    handoffs = data['handoff_times']

    # Color by leg
    for k in range(data['K_star']):
        mask = leg_ids == k
        ax.plot(times[mask], pos[mask, 0],
                color=LEG_COLORS[k % len(LEG_COLORS)],
                lw=2, label=f'Leg {k+1}')

    for i, ht in enumerate(handoffs):
        ax.axvline(ht, color='gray', lw=1.2, ls='--')
        ax.annotate(f'Handoff {i+1}\n(+{HANDOFF_DWELL:.0f} s dwell)',
                    xy=(ht, 5), fontsize=8, color='gray', ha='center')

    ax.set_xlabel('Time (s)', fontsize=11)
    ax.set_ylabel('Parcel x-position (m)', fontsize=11)
    ax.set_title('Parcel Position vs Time — Straight Route', fontsize=12)
    ax.set_xlim(0, data['straight_T'] + 1)
    ax.set_ylim(-5, 130)
    ax.legend(fontsize=9, loc='upper left')
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'parcel_position.png')
    plt.savefig(path, dpi=150); plt.close()
    print(f'Saved: {path}')


def plot_battery_and_timing(data, out_dir):
    """Battery consumption bars and delivery-time breakdown side by side."""
    fig, axes = plt.subplots(1, 2, figsize=(13, 5))

    # ─ Left: battery bar chart ─
    ax = axes[0]
    K = data['K_star']
    batteries = data['straight_batt']
    x = np.arange(1, K + 1)
    bars = ax.bar(x, [b * 100 for b in batteries],
                  color=[LEG_COLORS[k % len(LEG_COLORS)] for k in range(K)],
                  edgecolor='k', linewidth=0.8, width=0.6)

    ax.axhline(90, color='crimson', lw=1.5, ls='--', label='Reserve limit (90%)')
    ax.axhline(100, color='black', lw=1, ls=':')
    for bar, b in zip(bars, batteries):
        ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.5,
                f'{b*100:.1f}%', ha='center', va='bottom', fontsize=9)

    ax.set_xticks(x)
    ax.set_xticklabels([f'Drone {k}' for k in x])
    ax.set_ylim(0, 115)
    ax.set_ylabel('Battery consumed (%)', fontsize=11)
    ax.set_title('Battery Consumption per Drone\n(Straight Route)', fontsize=11)
    ax.legend(fontsize=9)
    ax.grid(axis='y', alpha=0.3)

    # ─ Right: delivery time stacked bar ─
    ax2 = axes[1]
    flight_times = [d / CRUISE_SPEED for d in data['straight_dists']]
    dwell_times  = [HANDOFF_DWELL] * (K - 1) + [0.0]   # last leg no dwell

    bottoms_f = [sum(flight_times[:k]) + sum(dwell_times[:k]) for k in range(K)]
    bottoms_d = [b + ft for b, ft in zip(bottoms_f, flight_times)]

    for k in range(K):
        # flight segment
        ax2.barh(0, flight_times[k], left=bottoms_f[k],
                 color=LEG_COLORS[k % len(LEG_COLORS)],
                 edgecolor='k', linewidth=0.6, height=0.5,
                 label=f'Leg {k+1} flight ({flight_times[k]:.1f} s)')
        # dwell segment
        if dwell_times[k] > 0:
            ax2.barh(0, dwell_times[k], left=bottoms_d[k],
                     color='lightgray', edgecolor='k', linewidth=0.6,
                     height=0.5)
            ax2.text(bottoms_d[k] + dwell_times[k] / 2, 0,
                     f'+{dwell_times[k]:.0f}s', ha='center', va='center',
                     fontsize=8, color='#444')

    total_T = data['straight_T']
    ax2.set_xlim(0, total_T + 2)
    ax2.set_yticks([])
    ax2.set_xlabel('Time (s)', fontsize=11)
    ax2.set_title(f'Delivery Time Breakdown\nTotal = {total_T:.1f} s', fontsize=11)
    ax2.legend(fontsize=8, loc='lower right')
    ax2.grid(axis='x', alpha=0.3)

    plt.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'battery_and_timing.png')
    plt.savefig(path, dpi=150); plt.close()
    print(f'Saved: {path}')


def plot_comparison_table(data, out_dir):
    """Bar chart comparing straight optimal vs bent greedy relay."""
    fig, axes = plt.subplots(1, 3, figsize=(12, 5))

    categories = ['Straight (optimal)', 'Bent corridor (greedy)']
    colors = ['#2A9D8F', '#E63946']

    # Number of legs
    n_legs = [data['K_star'], len(data['bent_legs'])]
    axes[0].bar(categories, n_legs, color=colors, edgecolor='k', width=0.5)
    for i, v in enumerate(n_legs):
        axes[0].text(i, v + 0.05, str(v), ha='center', fontsize=11, fontweight='bold')
    axes[0].set_title('Number of Legs K', fontsize=11)
    axes[0].set_ylabel('Legs')
    axes[0].set_ylim(0, max(n_legs) + 1)
    axes[0].grid(axis='y', alpha=0.3)

    # Bottleneck leg
    bottleneck = [max(data['straight_dists']), max(data['bent_dists'])]
    axes[1].bar(categories, bottleneck, color=colors, edgecolor='k', width=0.5)
    for i, v in enumerate(bottleneck):
        axes[1].text(i, v + 0.3, f'{v:.2f} m', ha='center', fontsize=10, fontweight='bold')
    axes[1].axhline(R_EFF, color='crimson', ls='--', lw=1.5, label=f'R_eff = {R_EFF:.1f} m')
    axes[1].set_title('Bottleneck Leg Length', fontsize=11)
    axes[1].set_ylabel('Length (m)')
    axes[1].set_ylim(0, R_EFF + 8)
    axes[1].legend(fontsize=9)
    axes[1].grid(axis='y', alpha=0.3)

    # Total delivery time
    total_times = [data['straight_T'], data['bent_T']]
    axes[2].bar(categories, total_times, color=colors, edgecolor='k', width=0.5)
    for i, v in enumerate(total_times):
        axes[2].text(i, v + 0.3, f'{v:.1f} s', ha='center', fontsize=10, fontweight='bold')
    axes[2].set_title('Total Delivery Time', fontsize=11)
    axes[2].set_ylabel('Time (s)')
    axes[2].set_ylim(0, max(total_times) + 5)
    axes[2].grid(axis='y', alpha=0.3)

    fig.suptitle('S036 Last-Mile Relay — Strategy Comparison', fontsize=13, fontweight='bold')
    plt.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'comparison.png')
    plt.savefig(path, dpi=150); plt.close()
    print(f'Saved: {path}')


def save_animation(data, out_dir):
    import matplotlib.animation as animation

    times   = data['s_times']
    pos     = data['s_pos']
    leg_ids = data['s_leg_ids']
    wps     = data['straight_wps']
    legs    = data['straight_legs']

    # Decimate for animation speed
    step = 4
    t_dec   = times[::step]
    pos_dec = pos[::step]
    lid_dec = leg_ids[::step]

    fig, ax = plt.subplots(figsize=(9, 4))

    # Static elements
    for k, (si, ei) in enumerate(legs):
        ax.plot([wps[si][0], wps[ei][0]], [wps[si][1], wps[ei][1]],
                color=LEG_COLORS[k % len(LEG_COLORS)], lw=2.5, alpha=0.4)
    for wp in wps[1:-1]:
        ax.scatter(*wp, s=120, color='orange', zorder=4, edgecolors='k')
    ax.scatter(*wps[0],  s=180, marker='s', color='green',  zorder=5, edgecolors='k')
    ax.scatter(*wps[-1], s=180, marker='*', color='red',    zorder=5, edgecolors='k')

    drone_dot, = ax.plot([], [], 'o', ms=10, color='#264653', zorder=6)
    trail_line, = ax.plot([], [], '-', lw=1.5, color='#264653', alpha=0.4, zorder=5)
    time_text = ax.text(0.02, 0.92, '', transform=ax.transAxes, fontsize=10)
    leg_text  = ax.text(0.02, 0.82, '', transform=ax.transAxes, fontsize=10, color='navy')

    ax.set_xlim(-8, 128)
    ax.set_ylim(-12, 12)
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')
    ax.set_title('S036 Last-Mile Relay — Parcel in Transit', fontsize=12)
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')

    trail_len = 40   # frames to keep in trail

    def init():
        drone_dot.set_data([], [])
        trail_line.set_data([], [])
        time_text.set_text('')
        leg_text.set_text('')
        return drone_dot, trail_line, time_text, leg_text

    def update(frame):
        x, y = pos_dec[frame]
        drone_dot.set_data([x], [y])
        start = max(0, frame - trail_len)
        trail_line.set_data(pos_dec[start:frame+1, 0], pos_dec[start:frame+1, 1])
        time_text.set_text(f't = {t_dec[frame]:.1f} s')
        leg_text.set_text(f'Leg {lid_dec[frame]+1} / {data["K_star"]}')
        return drone_dot, trail_line, time_text, leg_text

    ani = animation.FuncAnimation(
        fig, update, frames=len(t_dec),
        init_func=init, blit=True, interval=50
    )

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer='pillow', fps=20, dpi=100)
    plt.close()
    print(f'Saved: {path}')


if __name__ == '__main__':
    data = run_simulation()

    # ── Key metrics ──────────────────────────────────────────────────────────
    print('=' * 55)
    print('S036 Last-Mile Relay — Simulation Results')
    print('=' * 55)
    print(f'  Straight route:')
    print(f'    Number of legs K*          : {data["K_star"]}')
    print(f'    Leg distances              : {[f"{d:.2f}" for d in data["straight_dists"]]} m')
    print(f'    Bottleneck leg             : {max(data["straight_dists"]):.2f} m')
    print(f'    Battery per drone          : {[f"{b*100:.1f}%" for b in data["straight_batt"]]}')
    print(f'    Total delivery time        : {data["straight_T"]:.2f} s')
    print()
    print(f'  Bent corridor (greedy segmentation):')
    print(f'    Number of legs             : {len(data["bent_legs"])}')
    print(f'    Leg distances              : {[f"{d:.2f}" for d in data["bent_dists"]]} m')
    print(f'    Bottleneck leg             : {max(data["bent_dists"]):.2f} m')
    print(f'    Battery per drone          : {[f"{b*100:.1f}%" for b in data["bent_batt"]]}')
    print(f'    Total delivery time        : {data["bent_T"]:.2f} s')
    print('=' * 55)

    out_dir = os.path.normpath(OUTPUT_DIR)
    plot_route_map(data, out_dir)
    plot_parcel_position(data, out_dir)
    plot_battery_and_timing(data, out_dir)
    plot_comparison_table(data, out_dir)
    save_animation(data, out_dir)
