"""
S036 Last-Mile Relay
====================
A parcel is delivered from a source depot to a destination 120 m away, which
exceeds any single drone's effective range (R_eff = 0.9 x 35 m = 31.5 m), by a
chain of relay drones that hand the parcel over at relay stations.

Two routes are studied (2-D top-down model, altitude not simulated):

* Straight route (120 m): equal spacing with K* = ceil(L / R_eff) legs.
* Bent corridor (source -> 3 forced via-points -> destination, 140.3 m): the
  drones must stay on the corridor polyline, so every leg length is measured
  as arc length ALONG the polyline (not as the straight chord between cut
  points, which would cut the restricted corners). Three relay plans:
    - naive equal-x spacing (relays at x = 30/60/90 m, as on the straight route)
    - greedy furthest-reachable segmentation (each leg as long as R_eff allows)
    - equal arc-length spacing with K = ceil(L_arc / R_eff) legs
      (minimises the bottleneck leg for that K)

Delivery time  T = sum(d_k) / v + (K - 1) * tau_h.
Battery used   B_k = d_k / R (must stay <= 0.90).

Usage:
    conda activate drones
    python src/02_logistics_delivery/s036_last_mile_relay.py
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# ── Parameters ─────────────────────────────────────────────────────────────────
DRONE_RANGE    = 35.0       # m   max range per full charge (R)
RANGE_RESERVE  = 0.10       # 10% battery reserve
CRUISE_SPEED   = 5.0        # m/s
HANDOFF_DWELL  = 3.0        # s   parcel transfer + fresh drone spin-up
DT             = 0.05       # s   parcel-trace sampling step

R_EFF = DRONE_RANGE * (1.0 - RANGE_RESERVE)   # 31.5 m

SOURCE = np.array([0.0,   0.0])
DEST   = np.array([120.0, 0.0])

# Bent-corridor via-points (airspace restrictions)
VIA_POINTS = [
    np.array([30.0,  20.0]),   # divert north
    np.array([60.0,  10.0]),
    np.array([90.0, -15.0]),   # divert south
]
NAIVE_RELAY_X = [30.0, 60.0, 90.0]   # equal-x relay positions copied from straight route

GIF_FRAMES = 140

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '02_logistics_delivery', 's036_last_mile_relay',
)


# ── Polyline helpers ──────────────────────────────────────────────────────────
class Polyline:
    """Piecewise-linear route with arc-length parametrisation."""

    def __init__(self, vertices):
        self.v = np.asarray(vertices, dtype=float)
        seg = np.linalg.norm(np.diff(self.v, axis=0), axis=1)
        self.cum = np.concatenate([[0.0], np.cumsum(seg)])
        self.length = float(self.cum[-1])

    def point(self, s):
        """Point at arc length s (clipped to [0, length])."""
        s = float(np.clip(s, 0.0, self.length))
        i = int(np.searchsorted(self.cum, s, side='right') - 1)
        i = min(i, len(self.v) - 2)
        seg_len = self.cum[i + 1] - self.cum[i]
        frac = 0.0 if seg_len == 0 else (s - self.cum[i]) / seg_len
        return self.v[i] + frac * (self.v[i + 1] - self.v[i])

    def arc_of_x(self, x):
        """Arc length of the first polyline point with abscissa x (x monotone)."""
        for i in range(len(self.v) - 1):
            x0, x1 = self.v[i, 0], self.v[i + 1, 0]
            if x0 <= x <= x1 and x1 > x0:
                frac = (x - x0) / (x1 - x0)
                return float(self.cum[i] + frac * (self.cum[i + 1] - self.cum[i]))
        raise ValueError(f'x = {x} not on polyline')

    def sub_path(self, s0, s1):
        """Vertices of the polyline between arc lengths s0 and s1."""
        inner = [self.v[i] for i in range(len(self.v)) if s0 < self.cum[i] < s1]
        return np.array([self.point(s0)] + inner + [self.point(s1)])


# ── Segmentation strategies (return cut arc lengths [0, s1, ..., L]) ──────────
def greedy_segment(route, r_eff):
    """Greedy furthest-reachable segmentation along the route's arc length.

    Every leg is extended until its along-route length reaches r_eff (the
    candidate relay locations are the continuum of corridor points).
    """
    cuts = [0.0]
    while route.length - cuts[-1] > r_eff + 1e-9:
        cuts.append(cuts[-1] + r_eff)
    cuts.append(route.length)
    return cuts


def equal_arc_segment(route, r_eff):
    """K = ceil(L / r_eff) legs of equal along-route length (min bottleneck)."""
    K = int(np.ceil(route.length / r_eff - 1e-12))
    return [route.length * k / K for k in range(K + 1)]


def naive_x_segment(route, relay_x):
    """Relays at fixed x positions, ignoring the corridor's detours."""
    return [0.0] + [route.arc_of_x(x) for x in relay_x] + [route.length]


def evaluate_plan(name, route, cuts):
    dists = list(np.diff(cuts))
    K = len(dists)
    batt = [d / DRONE_RANGE for d in dists]
    T = sum(dists) / CRUISE_SPEED + (K - 1) * HANDOFF_DWELL
    feasible = all(d <= R_EFF + 1e-9 for d in dists)
    return dict(name=name, route=route, cuts=cuts, dists=dists, K=K,
                batt=batt, T=T, feasible=feasible,
                bottleneck=max(dists))


def parcel_state(plan, t):
    """Parcel arc length, xy position and leg index at time t."""
    s_acc, t_acc = 0.0, 0.0
    for k, d in enumerate(plan['dists']):
        t_fly = d / CRUISE_SPEED
        if t <= t_acc + t_fly:
            s = s_acc + CRUISE_SPEED * max(t - t_acc, 0.0)
            return s, plan['route'].point(s), k
        t_acc += t_fly
        s_acc += d
        if k < plan['K'] - 1:
            if t <= t_acc + HANDOFF_DWELL:
                return s_acc, plan['route'].point(s_acc), k   # dwelling at relay
            t_acc += HANDOFF_DWELL
    return plan['route'].length, plan['route'].point(plan['route'].length), plan['K'] - 1


def handoff_times(plan):
    """Arrival times at each intermediate relay station."""
    out, t = [], 0.0
    for d in plan['dists'][:-1]:
        t += d / CRUISE_SPEED
        out.append(t)
        t += HANDOFF_DWELL
    return out


# ── Simulation ────────────────────────────────────────────────────────────────
def run_simulation():
    straight = Polyline([SOURCE, DEST])
    bent = Polyline([SOURCE] + VIA_POINTS + [DEST])

    plans = {
        'straight': evaluate_plan('Straight, equal spacing', straight,
                                  equal_arc_segment(straight, R_EFF)),
        'naive':    evaluate_plan('Bent, naive equal-x', bent,
                                  naive_x_segment(bent, NAIVE_RELAY_X)),
        'greedy':   evaluate_plan('Bent, greedy furthest-reachable', bent,
                                  greedy_segment(bent, R_EFF)),
        'equal':    evaluate_plan('Bent, equal arc-length', bent,
                                  equal_arc_segment(bent, R_EFF)),
    }

    # Parcel traces (sampled every DT) for the feasible plans
    for key in ('straight', 'greedy', 'equal'):
        p = plans[key]
        ts = np.arange(0.0, p['T'] + DT / 2, DT)
        states = [parcel_state(p, t) for t in ts]
        p['trace_t'] = ts
        p['trace_s'] = np.array([st[0] for st in states])
        p['trace_xy'] = np.array([st[1] for st in states])
        p['trace_leg'] = np.array([st[2] for st in states])
        p['handoffs'] = handoff_times(p)

    return dict(plans=plans, straight=straight, bent=bent,
                direct_feasible=straight.length <= R_EFF)


# ── Plots ─────────────────────────────────────────────────────────────────────
LEG_COLORS = ['#E63946', '#2A9D8F', '#E9C46A', '#F4A261', '#264653',
              '#8338EC', '#3A86FF']
PLAN_COLORS = {'straight': '#2A9D8F', 'naive': '#999999',
               'greedy': '#E63946', 'equal': '#3A86FF'}


def _draw_plan(ax, plan, title):
    route = plan['route']
    ax.plot(route.v[:, 0], route.v[:, 1], color='lightgray', lw=6, zorder=1,
            solid_capstyle='round')
    for k in range(plan['K']):
        sub = route.sub_path(plan['cuts'][k], plan['cuts'][k + 1])
        color = LEG_COLORS[k % len(LEG_COLORS)]
        ax.plot(sub[:, 0], sub[:, 1], color=color, lw=3, zorder=2)
        mid = route.point(0.5 * (plan['cuts'][k] + plan['cuts'][k + 1]))
        ok = plan['dists'][k] <= R_EFF + 1e-9
        ax.annotate(f'{plan["dists"][k]:.1f} m\n{plan["batt"][k]*100:.0f}%'
                    + ('' if ok else ' (!)'),
                    xy=mid, xytext=(0, 6), textcoords='offset points',
                    fontsize=7.5, ha='center', va='bottom', color=color,
                    fontweight='bold')
    for s in plan['cuts'][1:-1]:
        ax.scatter(*route.point(s), s=110, color='orange', zorder=4,
                   edgecolors='k', linewidths=1)
    for vp in VIA_POINTS if route.v.shape[0] > 2 else []:
        ax.scatter(*vp, s=60, marker='^', color='purple', zorder=5,
                   edgecolors='k', linewidths=0.8)
    ax.scatter(*SOURCE, s=160, marker='s', color='green', zorder=6,
               edgecolors='k', linewidths=1.2)
    ax.scatter(*DEST, s=220, marker='*', color='red', zorder=6,
               edgecolors='k', linewidths=1.2)
    ax.set_xlim(-10, 130)
    ax.set_ylim(-28, 32)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    status = 'feasible' if plan['feasible'] else 'INFEASIBLE'
    ax.set_title(f'{title}\nK={plan["K"]}, bottleneck {plan["bottleneck"]:.2f} m, '
                 f'T={plan["T"]:.2f} s ({status})', fontsize=10)


def plot_route_map(data, out_dir):
    """2x2 top-down maps: straight route + the three bent-corridor plans."""
    P = data['plans']
    fig, axes = plt.subplots(2, 2, figsize=(14, 9))
    _draw_plan(axes[0, 0], P['straight'], 'Straight route, equal spacing')
    _draw_plan(axes[0, 1], P['naive'], 'Bent corridor, naive equal-x relays')
    _draw_plan(axes[1, 0], P['greedy'], 'Bent corridor, greedy furthest-reachable')
    _draw_plan(axes[1, 1], P['equal'], 'Bent corridor, equal arc-length')
    handles = [
        plt.Line2D([], [], marker='s', color='green', ls='', ms=10, label='Source'),
        plt.Line2D([], [], marker='*', color='red', ls='', ms=12, label='Destination'),
        plt.Line2D([], [], marker='o', color='orange', ls='', ms=10, label='Relay station'),
        plt.Line2D([], [], marker='^', color='purple', ls='', ms=8, label='Forced via-point'),
        plt.Line2D([], [], color='lightgray', lw=6, label='Corridor'),
    ]
    fig.legend(handles=handles, loc='lower center', ncol=5, fontsize=9)
    fig.suptitle('S036 Last-Mile Relay — Route Maps (leg length along corridor, '
                 f'battery % of R = {DRONE_RANGE:.0f} m; (!) = leg > R_eff)',
                 fontsize=12, fontweight='bold')
    plt.tight_layout(rect=[0, 0.05, 1, 0.96])
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'route_map.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def plot_parcel_position(data, out_dir):
    """Parcel along-route distance vs time for the three feasible plans."""
    P = data['plans']
    fig, ax = plt.subplots(figsize=(10, 4.5))
    for key in ('straight', 'greedy', 'equal'):
        p = P[key]
        ax.plot(p['trace_t'], p['trace_s'], color=PLAN_COLORS[key], lw=2,
                label=f'{p["name"]} (T = {p["T"]:.2f} s)')
        for ht in p['handoffs']:
            ax.axvline(ht, color=PLAN_COLORS[key], lw=0.8, ls=':', alpha=0.7)
    ax.set_xlabel('Time (s)', fontsize=11)
    ax.set_ylabel('Parcel distance along route (m)', fontsize=11)
    ax.set_title('Parcel Progress vs Time (flat steps = 3 s handoff dwell; '
                 'dotted lines = relay arrivals)', fontsize=11)
    ax.legend(fontsize=9, loc='upper left')
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'parcel_position.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def plot_battery_and_timing(data, out_dir):
    """Battery consumed per drone and delivery-time breakdown per plan."""
    P = data['plans']
    keys = ('straight', 'naive', 'greedy', 'equal')
    fig, axes = plt.subplots(1, 2, figsize=(14, 5.5))

    ax = axes[0]
    width = 0.2
    max_k = max(P[k]['K'] for k in keys)
    for j, key in enumerate(keys):
        p = P[key]
        xs = np.arange(1, p['K'] + 1) + (j - 1.5) * width
        ax.bar(xs, [b * 100 for b in p['batt']], width=width,
               color=PLAN_COLORS[key], edgecolor='k', linewidth=0.6,
               label=p['name'])
        for x, b in zip(xs, p['batt']):
            ax.text(x, b * 100 + 1, f'{b*100:.0f}', ha='center', fontsize=6.5)
    ax.axhline(90, color='crimson', lw=1.5, ls='--', label='Reserve limit (90%)')
    ax.axhline(100, color='black', lw=1, ls=':', label='Empty (100%)')
    ax.set_xticks(np.arange(1, max_k + 1))
    ax.set_xticklabels([f'Drone {k}' for k in range(1, max_k + 1)])
    ax.set_ylim(0, 150)
    ax.set_ylabel('Battery consumed (% of R)', fontsize=11)
    ax.set_title('Battery Consumption per Relay Drone', fontsize=11)
    ax.legend(fontsize=8, loc='upper center', ncol=2)
    ax.grid(axis='y', alpha=0.3)

    ax2 = axes[1]
    for row, key in enumerate(keys):
        p = P[key]
        left = 0.0
        for k, d in enumerate(p['dists']):
            ft = d / CRUISE_SPEED
            ax2.barh(row, ft, left=left, height=0.55,
                     color=LEG_COLORS[k % len(LEG_COLORS)], edgecolor='k',
                     linewidth=0.5)
            left += ft
            if k < p['K'] - 1:
                ax2.barh(row, HANDOFF_DWELL, left=left, height=0.55,
                         color='lightgray', edgecolor='k', linewidth=0.5)
                left += HANDOFF_DWELL
        tag = '' if p['feasible'] else '  (infeasible)'
        ax2.text(left + 0.4, row, f'{p["T"]:.2f} s{tag}', va='center', fontsize=9)
    ax2.set_yticks(range(len(keys)))
    ax2.set_yticklabels([P[k]['name'] for k in keys], fontsize=9)
    ax2.invert_yaxis()
    ax2.set_xlim(0, 58)
    ax2.set_xlabel('Time (s)', fontsize=11)
    ax2.set_title('Delivery Time Breakdown (coloured = leg flight, grey = 3 s handoff)',
                  fontsize=11)
    ax2.grid(axis='x', alpha=0.3)

    plt.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'battery_and_timing.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def plot_comparison_table(data, out_dir):
    """Bar chart: legs, bottleneck leg and delivery time for every plan."""
    P = data['plans']
    keys = ('straight', 'naive', 'greedy', 'equal')
    labels = ['Straight\nequal', 'Bent\nnaive equal-x', 'Bent\ngreedy', 'Bent\nequal-arc']
    colors = [PLAN_COLORS[k] for k in keys]
    fig, axes = plt.subplots(1, 3, figsize=(14, 5))

    n_legs = [P[k]['K'] for k in keys]
    axes[0].bar(labels, n_legs, color=colors, edgecolor='k', width=0.55)
    for i, v in enumerate(n_legs):
        axes[0].text(i, v + 0.05, str(v), ha='center', fontsize=11, fontweight='bold')
    axes[0].set_title('Number of Legs K', fontsize=11)
    axes[0].set_ylim(0, max(n_legs) + 1)
    axes[0].grid(axis='y', alpha=0.3)

    bott = [P[k]['bottleneck'] for k in keys]
    axes[1].bar(labels, bott, color=colors, edgecolor='k', width=0.55)
    for i, v in enumerate(bott):
        axes[1].text(i, v + 0.4, f'{v:.2f} m', ha='center', fontsize=10, fontweight='bold')
    axes[1].axhline(R_EFF, color='crimson', ls='--', lw=1.5, label=f'R_eff = {R_EFF:.1f} m')
    axes[1].axhline(DRONE_RANGE, color='black', ls=':', lw=1.2, label=f'R = {DRONE_RANGE:.0f} m')
    axes[1].set_title('Bottleneck Leg (along corridor)', fontsize=11)
    axes[1].set_ylabel('Length (m)')
    axes[1].set_ylim(0, 45)
    axes[1].legend(fontsize=9, loc='lower right')
    axes[1].grid(axis='y', alpha=0.3)

    tt = [P[k]['T'] for k in keys]
    axes[2].bar(labels, tt, color=colors, edgecolor='k', width=0.55)
    for i, v in enumerate(tt):
        axes[2].text(i, v + 0.4, f'{v:.2f} s', ha='center', fontsize=10, fontweight='bold')
    axes[2].set_title('Total Delivery Time', fontsize=11)
    axes[2].set_ylabel('Time (s)')
    axes[2].set_ylim(0, max(tt) + 6)
    axes[2].grid(axis='y', alpha=0.3)
    for ax in axes:
        ax.tick_params(axis='x', labelsize=8.5)

    fig.suptitle('S036 Last-Mile Relay — Plan Comparison (naive equal-x is infeasible)',
                 fontsize=13, fontweight='bold')
    plt.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'comparison.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def save_animation(data, out_dir):
    """Parcel in transit on the straight route and on the bent corridor
    (equal arc-length plan), on a common clock."""
    P = data['plans']
    shown = [('straight', 'Straight route, equal spacing'),
             ('equal', 'Bent corridor, equal arc-length')]
    T_end = max(P[k]['T'] for k, _ in shown)
    frame_t = np.linspace(0.0, T_end, GIF_FRAMES)

    fig, axes = plt.subplots(2, 1, figsize=(9, 6.5))
    artists = []
    for ax, (key, title) in zip(axes, shown):
        p = P[key]
        route = p['route']
        ax.plot(route.v[:, 0], route.v[:, 1], color='lightgray', lw=6, zorder=1)
        for k in range(p['K']):
            sub = route.sub_path(p['cuts'][k], p['cuts'][k + 1])
            ax.plot(sub[:, 0], sub[:, 1], color=LEG_COLORS[k % len(LEG_COLORS)],
                    lw=2.5, alpha=0.5, zorder=2)
        for s in p['cuts'][1:-1]:
            ax.scatter(*route.point(s), s=100, color='orange', zorder=4, edgecolors='k')
        ax.scatter(*SOURCE, s=150, marker='s', color='green', zorder=5, edgecolors='k')
        ax.scatter(*DEST, s=200, marker='*', color='red', zorder=5, edgecolors='k')
        dot, = ax.plot([], [], 'o', ms=10, color='#264653', zorder=6)
        txt = ax.text(0.01, 0.06, '', transform=ax.transAxes, fontsize=9, color='navy')
        ax.set_xlim(-8, 128)
        ax.set_ylim(-25, 28)
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.set_title(f'{title} (K={p["K"]}, T={p["T"]:.2f} s)', fontsize=10)
        artists.append((key, dot, txt))
    clock = fig.suptitle('', fontsize=12, fontweight='bold')

    def update(i):
        t = frame_t[i]
        out = []
        for key, dot, txt in artists:
            p = P[key]
            s, xy, leg = parcel_state(p, t)
            dot.set_data([xy[0]], [xy[1]])
            if t >= p['T'] - 1e-9:
                txt.set_text('Delivered')
            else:
                txt.set_text(f'Leg {leg + 1}/{p["K"]}   s = {s:.1f} m')
            out += [dot, txt]
        clock.set_text(f'S036 Last-Mile Relay — t = {t:.1f} s')
        return out + [clock]

    ani = animation.FuncAnimation(fig, update, frames=GIF_FRAMES, blit=False)
    plt.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer='pillow', fps=20, dpi=90)
    plt.close()
    print(f'Saved: {path}')


if __name__ == '__main__':
    data = run_simulation()
    P = data['plans']

    print('=' * 64)
    print('S036 Last-Mile Relay — Simulation Results')
    print('=' * 64)
    print(f'  R = {DRONE_RANGE:.1f} m, R_eff = {R_EFF:.2f} m, v = {CRUISE_SPEED:.1f} m/s, '
          f'tau_h = {HANDOFF_DWELL:.1f} s')
    print(f'  Straight route length       : {data["straight"].length:.2f} m')
    print(f'  Bent corridor length        : {data["bent"].length:.2f} m '
          f'(segments {[f"{d:.2f}" for d in np.diff(data["bent"].cum)]})')
    print(f'  Direct single-drone flight  : '
          f'{"feasible" if data["direct_feasible"] else "infeasible"} '
          f'({data["straight"].length:.1f} m > R_eff {R_EFF:.1f} m)')
    for key in ('straight', 'naive', 'greedy', 'equal'):
        p = P[key]
        print()
        print(f'  {p["name"]}:')
        print(f'    Feasible (all legs <= R_eff): {p["feasible"]}')
        print(f'    Number of legs K            : {p["K"]}')
        print(f'    Relay arc positions (m)     : {[f"{s:.2f}" for s in p["cuts"][1:-1]]}')
        print(f'    Leg lengths along route (m) : {[f"{d:.2f}" for d in p["dists"]]}')
        print(f'    Sum of legs (m)             : {sum(p["dists"]):.2f}')
        print(f'    Bottleneck leg (m)          : {p["bottleneck"]:.2f}')
        print(f'    Battery per drone           : {[f"{b*100:.1f}%" for b in p["batt"]]}')
        print(f'    Total delivery time (s)     : {p["T"]:.2f}')
    print('=' * 64)

    out_dir = os.path.normpath(OUTPUT_DIR)
    plot_route_map(data, out_dir)
    plot_parcel_position(data, out_dir)
    plot_battery_and_timing(data, out_dir)
    plot_comparison_table(data, out_dir)
    save_animation(data, out_dir)
