"""
S068 Large-Scale Farmland Cooperative Spraying
==============================================
Four drones spray a 200 x 100 m field with boustrophedon runs (2.0 m swath, 1.9 m run
spacing) at 3 m/s. A battery lasts 600 s of flight; a swap at the single station
(0, 50) takes 120 s. A drone returns when the time already flown plus the time to fly
home reaches the battery limit (exact trigger, solved along the current leg), then
flies back to the exact point where it stopped and carries on.

Strategies:
  S  single drone, whole field (card baseline)
  E  four equal 50 m strips on one global run grid (card's partition)
  B  four contiguous run blocks chosen to minimise the slowest drone's finish time
     (dynamic programme over run cut points; added)
  Ec the card's own per-strip run grid (27 runs per strip, last run clamped), for reference

The flight is computed event by event (exact segment times); the card's 0.2 s timestep
is only used to sample the animation and the coverage curve.

Usage:
    conda activate drones
    python src/04_industrial_agriculture/s068_large_field_spray.py
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# ── Parameters (card) ─────────────────────────────────────────────────────────
FIELD_W     = 200.0
FIELD_D     = 100.0
N_DRONES    = 4
SPRAY_WIDTH = 2.0
OVERLAP     = 0.1
RUN_SPACING = SPRAY_WIDTH - OVERLAP        # 1.9 m
SPEED       = 3.0
T_BAT       = 600.0
T_CHARGE    = 120.0
STATION     = np.array([0.0, 50.0])
CELL        = 0.5
DT          = 0.2
T_CUTOFF    = 10_000.0
STRIP_W     = FIELD_W / N_DRONES           # 50 m

# ── Parameters (added) ───────────────────────────────────────────────────────
ANIM_DT     = 20.0      # s between GIF frames
COLORS      = ['tab:red', 'tab:orange', 'tab:blue', 'tab:purple']
STRAT_COLORS = {'S': '#7F7F7F', 'E': '#1F77B4', 'B': '#2CA02C', 'Ec': '#9467BD'}
STRAT_LABELS = {'S': 'Single drone', 'E': 'Equal 50 m strips (4 drones)',
                'B': 'Balanced partition (4 drones)', 'Ec': 'Equal strips, card run grid'}

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), '..', '..', 'outputs',
                          '04_industrial_agriculture', 's068_large_field_spray')

MX, MY = int(FIELD_W / CELL), int(FIELD_D / CELL)
XC = (np.arange(MX) + 0.5) * CELL
YC = (np.arange(MY) + 0.5) * CELL


# ── Run grids ────────────────────────────────────────────────────────────────

def global_runs():
    """Run x-positions over the whole field: 0.95, 2.85, ... clamped to W - w/2."""
    n = int(np.ceil(FIELD_W / RUN_SPACING))
    return np.minimum((np.arange(n) + 0.5) * RUN_SPACING, FIELD_W - SPRAY_WIDTH / 2)


def card_strip_runs(i):
    """Card's grid: 27 runs per 50 m strip, last run clamped to the strip edge - w/2."""
    n = int(np.ceil(STRIP_W / RUN_SPACING))
    x0 = i * STRIP_W
    return np.minimum(x0 + (np.arange(n) + 0.5) * RUN_SPACING, x0 + STRIP_W - SPRAY_WIDTH / 2)


# ── Event-based drone flight ────────────────────────────────────────────────

def simulate_drone(xs):
    """Fly boustrophedon runs at x positions xs (first run upward). Returns a dict.

    Segments: (p0, p1, t0, t1, kind) with kind in {'spray', 'turn', 'ferry'}.
    """
    pos, t, used = STATION.copy(), 0.0, 0.0
    segs, trips, min_reserve = [], 0, T_BAT
    acc = {'spray': 0.0, 'turn': 0.0, 'ferry': 0.0, 'charge': 0.0}

    def home(p):
        return np.linalg.norm(p - STATION) / SPEED

    def fly(p_to, kind):
        nonlocal pos, t, used, min_reserve
        dur = np.linalg.norm(p_to - pos) / SPEED
        if dur <= 0:
            return
        segs.append((pos.copy(), p_to.copy(), t, t + dur, kind))
        t += dur
        used += dur
        acc[kind] += dur
        pos = p_to.copy()
        reserve = T_BAT - used - home(pos)
        min_reserve = min(min_reserve, T_BAT - used)
        assert reserve >= -1e-6, 'battery would not reach the station'
        assert used <= T_BAT + 1e-6, 'flying on an empty battery'

    def swap():
        nonlocal t, used, trips
        fly(STATION, 'ferry')
        assert T_BAT - used >= -1e-6
        segs.append((STATION.copy(), STATION.copy(), t, t + T_CHARGE, 'charge'))
        t += T_CHARGE
        acc['charge'] += T_CHARGE
        used = 0.0
        trips += 1

    def leg(p_to, kind):
        """Fly to p_to, swapping batteries on the way with the card's exact trigger."""
        while True:
            need = np.linalg.norm(p_to - pos) / SPEED + home(p_to)
            if used + need <= T_BAT + 1e-9:
                fly(p_to, kind)
                return
            # abort point: used + s/v + home(p(s)) = T_BAT, monotone in s -> bisection
            p0 = pos.copy()
            L = np.linalg.norm(p_to - p0)
            u = (p_to - p0) / L
            lo, hi = 0.0, L
            if used + home(p0) >= T_BAT - 1e-9:
                hi = 0.0
            for _ in range(60):
                mid = 0.5 * (lo + hi)
                if used + mid / SPEED + home(p0 + mid * u) <= T_BAT:
                    lo = mid
                else:
                    hi = mid
            stop = p0 + lo * u
            fly(stop, kind)
            swap()
            assert home(stop) * 2 < T_BAT, 'resume point unreachable on a full battery'
            if kind != 'ferry':
                fly(stop, 'ferry')          # back to the exact stopping point, nozzle off

    for r, x in enumerate(xs):
        ya, yb = (0.0, FIELD_D) if r % 2 == 0 else (FIELD_D, 0.0)
        start = np.array([x, ya])
        leg(start, 'ferry' if r == 0 else 'turn')
        leg(np.array([x, yb]), 'spray')
    t_cover = t
    acc_cover = dict(acc)
    fly(STATION, 'ferry')                   # final flight home (reserve asserted in fly)
    return dict(segs=segs, trips=trips, t_cover=t_cover, t_home=t, acc=acc,
                acc_cover=acc_cover, min_reserve=min_reserve)


# ── Coverage ─────────────────────────────────────────────────────────────────

def coverage_times(drones):
    """First-spray time and spray count for every cell (nozzle disk radius w/2)."""
    t_first = np.full((MX, MY), np.inf)
    count = np.zeros((MX, MY), dtype=int)
    R = SPRAY_WIDTH / 2
    for d in drones:
        for p0, p1, t0, t1, kind in d['segs']:
            if kind != 'spray':
                continue
            x = p0[0]
            ylo, yhi = min(p0[1], p1[1]), max(p0[1], p1[1])
            ix = np.where(np.abs(XC - x) <= R + 1e-9)[0]
            iy = np.where((YC >= ylo - R) & (YC <= yhi + R))[0]
            dx = XC[ix][:, None] - x
            yproj = np.clip(YC[iy][None, :], ylo, yhi)
            dist = np.hypot(dx, YC[iy][None, :] - yproj)
            hit = dist <= R + 1e-9
            tt = t0 + np.abs(yproj - p0[1]) / SPEED * np.ones_like(dx)
            sub_t = t_first[np.ix_(ix, iy)]
            t_first[np.ix_(ix, iy)] = np.where(hit, np.minimum(sub_t, tt), sub_t)
            count[np.ix_(ix, iy)] += hit
    return t_first, count


def run_strategy(blocks):
    drones = [simulate_drone(xs) for xs in blocks]
    t_first, count = coverage_times(drones)
    return dict(drones=drones, blocks=blocks, t_first=t_first, count=count,
                coverage=float(np.mean(np.isfinite(t_first))),
                t_mission=float(np.max(t_first)) if np.all(np.isfinite(t_first)) else np.nan,
                t_last_home=max(d['t_home'] for d in drones),
                trips=[d['trips'] for d in drones],
                overspray=float(np.mean(count >= 2)))


def balanced_blocks(xs, n):
    """Contiguous run blocks minimising max finish time (DP over cut points)."""
    m = len(xs)
    cost = np.full((m + 1, m + 1), np.inf)
    lo, hi = m // (2 * n), 2 * m // n          # block sizes considered (13..53 runs)
    for a in range(m):
        for b in range(a + lo, min(a + hi, m) + 1):
            cost[a, b] = simulate_drone(xs[a:b])['t_cover']
    best = np.full((n + 1, m + 1), np.inf)
    arg = np.zeros((n + 1, m + 1), dtype=int)
    best[0, 0] = 0.0
    for k in range(1, n + 1):
        for b in range(1, m + 1):
            vals = np.maximum(best[k - 1, :b], cost[:b, b])
            a = int(np.argmin(vals))
            best[k, b], arg[k, b] = vals[a], a
    cuts, b = [], m
    for k in range(n, 0, -1):
        a = arg[k, b]
        cuts.append((a, b))
        b = a
    cuts = cuts[::-1]
    return [xs[a:b] for a, b in cuts], cuts, best[n, m]


def run_simulation():
    xs = global_runs()
    n = len(xs)
    edges = [int(round(n * k / N_DRONES)) for k in range(N_DRONES + 1)]
    equal = [xs[edges[k]:edges[k + 1]] for k in range(N_DRONES)]
    bal, cuts, opt = balanced_blocks(xs, N_DRONES)
    res = {'S': run_strategy([xs]), 'E': run_strategy(equal), 'B': run_strategy(bal),
           'Ec': run_strategy([card_strip_runs(i) for i in range(N_DRONES)])}
    res['B']['dp_opt'] = opt
    res['B']['cuts'] = cuts
    res['E']['cuts'] = list(zip(edges[:-1], edges[1:]))
    return xs, res


# ── Plotting ──────────────────────────────────────────────────────────────────

def _draw_paths(ax, r):
    for i, d in enumerate(r['drones']):
        col = COLORS[i % len(COLORS)]
        for p0, p1, t0, t1, kind in d['segs']:
            if kind == 'charge':
                continue
            ax.plot([p0[0], p1[0]], [p0[1], p1[1]], color=col,
                    lw=0.5 if kind != 'ferry' else 0.8,
                    ls='-' if kind != 'ferry' else '--', alpha=0.8)
    for b in r['blocks'][1:]:
        edge = b[0] - RUN_SPACING / 2
        ax.axvline(edge, color='k', ls='--', lw=1)
    ax.scatter(*STATION, marker='*', s=300, color='gold', edgecolor='k', zorder=5)


def plot_coverage_map(res, out_dir):
    fig, axes = plt.subplots(1, 2, figsize=(18, 5.5))
    from matplotlib.colors import ListedColormap
    cmap = ListedColormap(['#f2f2f2', '#a1d99b', '#31a354', '#006d2c'])
    for ax, s in zip(axes, ['E', 'B']):
        r = res[s]
        ax.imshow(np.minimum(r['count'], 3).T, origin='lower', extent=[0, FIELD_W, 0, FIELD_D],
                  cmap=cmap, vmin=-0.5, vmax=3.5, aspect='equal')
        _draw_paths(ax, r)
        widths = [b[-1] - b[0] + RUN_SPACING for b in r['blocks']]
        ax.set_title(f"{STRAT_LABELS[s]}: block widths "
                     f"{', '.join(f'{w:.1f}' for w in widths)} m\n"
                     f"T_mission {r['t_mission']:.0f} s, swaps {r['trips']}, "
                     f"coverage {100 * r['coverage']:.2f}%, sprayed twice or more "
                     f"{100 * r['overspray']:.1f}%", fontsize=10)
        ax.set_xlabel('x (m)')
        ax.set_ylabel('y (m)')
    fig.suptitle('S068 Coverage (light/mid/dark green = sprayed 1/2/3+ times), spray runs '
                 '(solid), ferry legs (dashed), block boundaries (black dashed), station '
                 '(gold star)', fontsize=11)
    plt.tight_layout()
    path = os.path.join(out_dir, 'coverage_map.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def coverage_curve(r, t_grid):
    tf = np.sort(r['t_first'].ravel())
    return np.searchsorted(tf, t_grid, side='right') / tf.size


def plot_coverage_progress(res, out_dir):
    fig, ax = plt.subplots(figsize=(10, 5.5))
    t_end = res['S']['t_mission'] * 1.03
    t_grid = np.arange(0, t_end, DT)
    for s in ['S', 'Ec', 'E', 'B']:
        r = res[s]
        ax.plot(t_grid, 100 * coverage_curve(r, t_grid), color=STRAT_COLORS[s],
                lw=1.8 if s != 'Ec' else 1.0, ls='-' if s != 'Ec' else ':',
                label=f"{STRAT_LABELS[s]}: {r['t_mission']:.0f} s")
    ax.axhline(100, color='gray', ls='--', lw=0.8)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Field coverage C(t) (%)')
    ax.set_ylim(0, 105)
    ax.set_title('S068 Coverage progress (flat steps = battery swaps)')
    ax.legend()
    ax.grid(alpha=0.3)
    path = os.path.join(out_dir, 'coverage_progress.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_time_breakdown(res, out_dir):
    fig, axes = plt.subplots(1, 3, figsize=(17, 5), gridspec_kw={'width_ratios': [1, 4, 4]})
    kinds = [('spray', 'Spraying', '#31a354'), ('turn', 'Run-to-run turns', '#fdae6b'),
             ('ferry', 'Ferry to/from station', '#6baed6'), ('charge', 'Battery swaps', '#bdbdbd')]
    for ax, s in zip(axes, ['S', 'E', 'B']):
        r = res[s]
        n = len(r['drones'])
        bottom = np.zeros(n)
        for k, lab, col in kinds:
            # time up to the last spray (t_cover): drop the final flight home
            vals = np.array([d['acc_cover'][k] for d in r['drones']])
            ax.bar(np.arange(n), vals, bottom=bottom, color=col, edgecolor='k', lw=0.4,
                   label=lab)
            bottom += vals
        for i, d in enumerate(r['drones']):
            ax.text(i, bottom[i] + 30, f"{d['trips']} swap{'' if d['trips'] == 1 else 's'}",
                    ha='center', fontsize=9)
        ax.axhline(r['t_mission'], color='crimson', ls='--', lw=1.2,
                   label=f"T_mission = {r['t_mission']:.0f} s")
        ax.set_xticks(np.arange(n))
        ax.set_xticklabels([f'Drone {i}' for i in range(n)])
        ax.set_title(STRAT_LABELS[s], fontsize=10)
        ax.set_ylabel('Time until its last spray (s)')
        ax.set_ylim(0, res['S']['t_mission'] * 1.1 if s == 'S' else res['E']['t_mission'] * 1.2)
        ax.legend(fontsize=7, loc='lower right')
    fig.suptitle('S068 Per-drone time breakdown (until each drone sprays its last cell)')
    plt.tight_layout()
    path = os.path.join(out_dir, 'time_breakdown.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_battery_trips(res, out_dir):
    fig, ax = plt.subplots(figsize=(9, 4.5))
    labels, vals, cols = [], [], []
    for s in ['S', 'Ec', 'E', 'B']:
        for i, n in enumerate(res[s]['trips']):
            labels.append(f'{s}\nD{i}')
            vals.append(n)
            cols.append(STRAT_COLORS[s])
    bars = ax.bar(np.arange(len(vals)), vals, color=cols, edgecolor='k')
    for b, v in zip(bars, vals):
        ax.text(b.get_x() + b.get_width() / 2, v + 0.1, str(v), ha='center')
    ax.set_xticks(np.arange(len(vals)))
    ax.set_xticklabels(labels, fontsize=8)
    ax.set_ylabel('Battery swaps')
    tot = {s: sum(res[s]['trips']) for s in res}
    ax.set_title('S068 Battery swaps per drone (totals: ' +
                 ', '.join(f'{s} {tot[s]}' for s in ['S', 'Ec', 'E', 'B']) + ')')
    ax.grid(alpha=0.3, axis='y')
    path = os.path.join(out_dir, 'battery_trips.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def drone_position(d, t):
    for p0, p1, t0, t1, kind in d['segs']:
        if t0 <= t <= t1:
            f = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)
            return p0 + f * (p1 - p0), kind
    return d['segs'][-1][1], 'done'


def save_animation(res, out_dir):
    t_end = max(res['E']['t_mission'], res['B']['t_mission'])
    step = int(round(ANIM_DT / DT))
    n_samples = int(np.ceil(t_end / DT)) + 1
    n_frames = int(np.ceil((n_samples - 1) / step)) + 1
    fig, axes = plt.subplots(2, 1, figsize=(9, 9))
    ims, dots = [], []
    for ax, s in zip(axes, ['E', 'B']):
        im = ax.imshow(np.zeros((MY, MX)), origin='lower', extent=[0, FIELD_W, 0, FIELD_D],
                       cmap='Greens', vmin=0, vmax=1.6, aspect='equal')
        for b in res[s]['blocks'][1:]:
            ax.axvline(b[0] - RUN_SPACING / 2, color='k', ls='--', lw=1)
        ax.scatter(*STATION, marker='*', s=250, color='gold', edgecolor='k', zorder=5)
        dd = [ax.plot([], [], 'o', color=COLORS[i], mec='k', ms=8, zorder=6)[0]
              for i in range(N_DRONES)]
        ax.set_xlabel('x (m)')
        ax.set_ylabel('y (m)')
        ims.append(im)
        dots.append(dd)

    def update(fi):
        k = min(fi * step, n_samples - 1)
        t = k * DT
        for ax, s, im, dd in zip(axes, ['E', 'B'], ims, dots):
            r = res[s]
            im.set_data((r['t_first'] <= t).T.astype(float))
            for i, d in enumerate(r['drones']):
                p, kind = drone_position(d, t)
                dd[i].set_data([p[0]], [p[1]])
                dd[i].set_alpha(0.4 if kind == 'charge' else 1.0)
            cov = np.mean(r['t_first'] <= t)
            ax.set_title(f"{STRAT_LABELS[s]} — t = {t:.0f} s, coverage {100 * cov:.1f}%",
                         fontsize=10)
        return ims

    anim = animation.FuncAnimation(fig, update, frames=n_frames, interval=50, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    anim.save(path, writer='pillow', fps=12, dpi=60)
    plt.close(fig)
    print(f'Saved: {path} ({n_frames} frames, every {ANIM_DT:.0f} s, last frame at '
          f'{(n_samples - 1) * DT:.1f} s)')


# ── Main ─────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    xs, res = run_simulation()
    print('=== S068 Large-Scale Farmland Cooperative Spraying ===')
    print(f'Global run grid: {len(xs)} runs, x = {xs[0]:.2f} ... {xs[-1]:.2f} m '
          f'(last run clamped from {(len(xs) - 0.5) * RUN_SPACING:.2f} m)')
    print(f'Card per-strip grid: {len(card_strip_runs(0))} runs per strip, '
          f'{N_DRONES * len(card_strip_runs(0))} in total')
    print(f'Nominal swath overlap between neighbouring runs: {OVERLAP / RUN_SPACING * 100:.2f} % '
          f'of the field; on the 0.5 m grid a cell counts as sprayed when its centre is '
          f'within {SPRAY_WIDTH / 2} m of the nozzle (inclusive)')
    work = len(xs) * FIELD_D / SPEED + (len(xs) - 1) * RUN_SPACING / SPEED
    print(f'Pure run + turn time for the whole field: {work:.1f} s '
          f'(ideal 4-drone share {work / N_DRONES:.1f} s)')
    for s in ['S', 'Ec', 'E', 'B']:
        r = res[s]
        print(f'[{s}] {STRAT_LABELS[s]}')
        if 'cuts' in r:
            print('    run blocks: ' + ', '.join(f'[{a},{b}) {b - a} runs '
                                                f'x {xs[a] - RUN_SPACING / 2:.1f}-'
                                                f'{min(xs[b - 1] + RUN_SPACING / 2, FIELD_W):.1f} m'
                                                for a, b in r['cuts']))
        print(f"    T_mission (last cell sprayed) {r['t_mission']:.1f} s, all drones home "
              f"{r['t_last_home']:.1f} s, coverage {100 * r['coverage']:.3f} %, "
              f"cells sprayed >= 2 times {100 * r['overspray']:.2f} %")
        print(f"    swaps per drone {r['trips']} (total {sum(r['trips'])})")
        for i, d in enumerate(r['drones']):
            a = d['acc']
            print(f"    drone {i}: last spray {d['t_cover']:.1f} s, home {d['t_home']:.1f} s, "
                  f"spray {a['spray']:.1f} s, turns {a['turn']:.1f} s, ferry {a['ferry']:.1f} s, "
                  f"swaps {d['trips']} ({a['charge']:.0f} s), min battery left "
                  f"{round(d['min_reserve'], 3) + 0.0:.3f} s")
        ev = sorted([(t0, 1) for d in r['drones'] for _, _, t0, _, k in d['segs']
                     if k == 'charge'] +
                    [(t1, -1) for d in r['drones'] for _, _, _, t1, k in d['segs']
                     if k == 'charge'])
        occ = np.cumsum([e for _, e in ev]) if ev else [0]
        print(f"    swap times: " + ', '.join(f"{t0:.0f}-{t1:.0f} s" for d in r['drones']
                                            for _, _, t0, t1, k in d['segs'] if k == 'charge')
              + f"; max drones swapping at once {int(max(occ))}")
        assert r['t_mission'] < T_CUTOFF
    print(f"DP optimum max finish time: {res['B']['dp_opt']:.1f} s")
    for s in ['Ec', 'E', 'B']:
        print(f"Speed-up vs single drone [{s}]: {res['S']['t_mission'] / res[s]['t_mission']:.3f}x "
              f"(ideal {N_DRONES}x)")
    print(f"Balanced vs equal: {res['E']['t_mission'] - res['B']['t_mission']:.1f} s faster "
          f"({100 * (1 - res['B']['t_mission'] / res['E']['t_mission']):.1f} %), swaps "
          f"{sum(res['E']['trips'])} -> {sum(res['B']['trips'])}")
    fin_e = [d['t_cover'] for d in res['E']['drones']]
    fin_b = [d['t_cover'] for d in res['B']['drones']]
    print(f'Finish-time spread: equal {max(fin_e) - min(fin_e):.1f} s, '
          f'balanced {max(fin_b) - min(fin_b):.1f} s')
    for s in ['E', 'B']:
        ac = [d['acc'] for d in res[s]['drones']]
        print(f"[{s}] overhead incl. the flight home: ferry {sum(a['ferry'] for a in ac):.1f} s, "
              f"swaps {sum(a['charge'] for a in ac):.1f} s (every drone swaps "
              f"{sorted(set(res[s]['trips']))} time(s)); bottleneck drone ferry "
              f"{ac[int(np.argmax([d['t_cover'] for d in res[s]['drones']]))]['ferry']:.1f} s")
    print(f'Mean finish time of E {np.mean(fin_e):.1f} s -> perfect rebalancing could save at most '
          f'about {max(fin_e) - np.mean(fin_e):.1f} s; one run + turn takes '
          f'{FIELD_D / SPEED + RUN_SPACING / SPEED:.2f} s, so blocks can only shift in steps of '
          f'that size')

    out = os.path.normpath(OUTPUT_DIR)
    os.makedirs(out, exist_ok=True)
    plot_coverage_map(res, out)
    plot_coverage_progress(res, out)
    plot_time_breakdown(res, out)
    plot_battery_trips(res, out)
    save_animation(res, out)
