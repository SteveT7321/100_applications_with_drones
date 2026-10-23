"""
S049 Dynamic Zone Search
========================
Four drones search a 400x400 m disaster area for 5 survivors using Voronoi zone
partitioning.  A prior probability map (Gaussian mixture) guides the search
order inside each zone.  All strategies share the same battery model: a drone
whose state of charge (SoC) drops below 30 % flies back to its home pad, swaps
its battery (T_SWAP) and rejoins; a drone that finds a survivor hovers for
T_CONFIRM to confirm and then rejoins.

Strategies compared:
  (1) Static Voronoi    - initial partition fixed; a drone that is away leaves
                          its zone unattended, an idle drone stays idle.
  (2) Dynamic, equal    - every time the set of available drones changes (or a
                          drone exhausts its zone) all unscanned cells are
                          re-partitioned by a standard Voronoi on the current
                          positions of the available drones.
  (3) Dynamic, weighted - same events, SoC-weighted Voronoi (|x - p_k| / SoC_k).

Results are reported for one showcase survivor placement (figures, animation)
and as a Monte Carlo over N_MC survivor placements.

Usage:
    conda activate drones
    python src/03_environmental_sar/s049_dynamic_zone.py
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from scipy.spatial import KDTree

# ── Parameters ─────────────────────────────────────────────────────────────
K_DRONES    = 4          # number of drones
AREA_SIZE   = 400.0      # m — square search area side length
GRID_RES    = 5.0        # m — cell resolution (delta)
SPEED       = 6.0        # m/s — drone cruise speed
SENSOR_R    = 8.0        # m — sensor footprint radius
D_MAX       = 2000.0     # m — range on a full battery
SOC_THRESH  = 0.30       # SoC below which a drone returns for a battery swap
T_SWAP      = 60.0       # s — battery swap time at the home pad
T_CONFIRM   = 10.0       # s — hover time to confirm a detected survivor
HOVER_POWER_RATIO = 1.0  # hover power / cruise power (multirotor: about equal)
TOP_M       = 20         # next target = nearest of the TOP_M highest-prior unscanned zone cells
N_SURVIVORS = 5          # number of survivors
DT          = 0.5        # s — simulation timestep
T_MAX       = 3600.0     # s — safety cutoff
N_MC        = 20         # Monte Carlo survivor placements (seeds 0..N_MC-1)
SHOW_SEED   = 0          # survivor placement used for the figures / animation
FRAME_EVERY = 4          # store an animation frame every 4 steps (2 s)
ANIM_MAX_FRAMES = 150

Nx = int(AREA_SIZE / GRID_RES)   # 80
Ny = int(AREA_SIZE / GRID_RES)   # 80

STARTS = np.array([[20.0, 20.0], [380.0, 20.0], [20.0, 380.0], [380.0, 380.0]])
STRATEGIES = ['static', 'equal', 'weighted']
DRONE_COLORS = ['#E63946', '#2196F3', '#4CAF50', '#FF9800']  # red, blue, green, orange
STYLE = {
    'static':   ('--', '#888888', 'Static Voronoi'),
    'equal':    ('-.', '#2196F3', 'Dynamic equal-weight Voronoi'),
    'weighted': ('-',  '#E63946', 'Dynamic battery-weighted Voronoi'),
}

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '03_environmental_sar', 's049_dynamic_zone',
)


# ── Helpers ────────────────────────────────────────────────────────────────

def build_prior_map():
    """Build a 3-component Gaussian mixture prior probability map (Nx x Ny)."""
    xi = np.arange(Nx) * GRID_RES + GRID_RES / 2.0
    yi = np.arange(Ny) * GRID_RES + GRID_RES / 2.0
    xv, yv = np.meshgrid(xi, yi, indexing='ij')
    coords = np.stack([xv, yv], axis=-1)   # (Nx, Ny, 2)
    components = [
        (0.5, np.array([100., 300.]), 60.),
        (0.3, np.array([280., 150.]), 50.),
        (0.2, np.array([340., 320.]), 40.),
    ]
    P = np.zeros((Nx, Ny))
    for alpha, mu, sigma in components:
        diff = coords - mu
        P += alpha * np.exp(-0.5 * np.sum(diff ** 2, axis=-1) / sigma ** 2)
    P /= P.sum()
    return P


def build_cell_centres():
    """Return (Nx*Ny, 2) array of cell centre coordinates (same order as P.ravel())."""
    xi = np.arange(Nx) * GRID_RES + GRID_RES / 2.0
    yi = np.arange(Ny) * GRID_RES + GRID_RES / 2.0
    xv, yv = np.meshgrid(xi, yi, indexing='ij')
    return np.stack([xv.ravel(), yv.ravel()], axis=1)


def weighted_voronoi_assignment(positions, weights, candidate_centres):
    """Assign each candidate cell to the generator with minimum |x - p_k| / w_k."""
    d = np.linalg.norm(candidate_centres[:, None, :] - positions[None, :, :], axis=2)
    return np.argmin(d / (weights[None, :] + 1e-9), axis=1)


def jain_fairness(areas):
    """Jain fairness index of per-drone remaining areas."""
    areas = np.array(areas, dtype=float)
    if len(areas) == 0 or areas.sum() == 0:
        return 1.0
    return areas.sum() ** 2 / (len(areas) * (areas ** 2).sum())


def place_survivors(cell_centres, P_flat, rng):
    """Sample survivor cells from the prior map distribution (no repeats)."""
    idx = rng.choice(len(cell_centres), size=N_SURVIVORS, replace=False, p=P_flat)
    return cell_centres[idx].copy()


def cells_to_image(values_flat):
    """(Nx*Ny,) cell values -> (Ny, Nx) image for imshow(origin='lower')."""
    return values_flat.reshape(Nx, Ny).T


# ── Drone Agent ────────────────────────────────────────────────────────────

class DroneAgent:
    """Drone state. mode: 'search' | 'idle' | 'confirm' | 'return' | 'swap'."""

    def __init__(self, idx, home):
        self.idx = idx
        self.home = home.astype(float).copy()
        self.pos = home.astype(float).copy()
        self.soc = 1.0
        self.dist_pack = 0.0      # distance flown on the current battery pack
        self.dist_total = 0.0
        self.mode = 'search'
        self.timer = 0.0
        self.target = None        # index of the cell currently flown to
        self.n_swaps = 0
        self.history = [self.pos.copy()]

    @property
    def available(self):
        """True if the drone takes part in the zone partition."""
        return self.mode in ('search', 'idle')

    def hover(self):
        """Hover for one step: drains the battery like HOVER_POWER_RATIO x
        cruise power, i.e. HOVER_POWER_RATIO * SPEED * DT of range."""
        self.dist_pack += HOVER_POWER_RATIO * SPEED * DT
        self.soc = max(0.0, 1.0 - self.dist_pack / D_MAX)

    def fly_towards(self, point):
        """Move up to SPEED*DT towards point, drain the battery by the distance
        flown (SoC = 1 - d / D_MAX); return the remaining distance."""
        diff = point - self.pos
        d = float(np.linalg.norm(diff))
        move = min(SPEED * DT, d)
        if d > 1e-9:
            self.pos = self.pos + diff / d * move
        self.dist_pack += move
        self.dist_total += move
        self.soc = max(0.0, 1.0 - self.dist_pack / D_MAX)
        return d - move


def choose_target(drone, zone_owner, scanned, cell_centres, P_flat):
    """Nearest cell among the TOP_M highest-prior unscanned cells of the drone's zone."""
    cand = np.flatnonzero((zone_owner == drone.idx) & ~scanned)
    if len(cand) == 0:
        return None
    if len(cand) > TOP_M:
        cand = cand[np.argpartition(-P_flat[cand], TOP_M - 1)[:TOP_M]]
    d = np.linalg.norm(cell_centres[cand] - drone.pos, axis=1)
    return int(cand[np.argmin(d)])


# ── Simulation ─────────────────────────────────────────────────────────────

def run_simulation(strategy='weighted', survivor_seed=SHOW_SEED, record=True):
    """
    Run the zone search for one strategy and one survivor placement.

    Events: a drone leaves the search (survivor found -> confirm hover;
    SoC < SOC_THRESH -> return home), rejoins (confirmation done; battery
    swapped) or exhausts its zone.  Dynamic strategies re-partition all
    unscanned cells among the available drones once per time step in which
    any event occurred; the static strategy never re-partitions.
    The sensor is processed only while a drone is in 'search' mode.
    """
    P = build_prior_map()
    P_flat = P.ravel()
    cell_centres = build_cell_centres()
    cell_tree = KDTree(cell_centres)

    survivors = place_survivors(cell_centres, P_flat, np.random.default_rng(survivor_seed))
    detect_times = [None] * N_SURVIVORS
    finder = [None] * N_SURVIVORS

    drones = [DroneAgent(k, STARTS[k]) for k in range(K_DRONES)]
    scanned = np.zeros(len(cell_centres), dtype=bool)
    for d in drones:
        scanned[cell_tree.query_ball_point(d.pos, SENSOR_R)] = True

    def partition(avail, weighted):
        owner = np.full(len(cell_centres), -1, dtype=np.int16)
        unsc = np.flatnonzero(~scanned)
        if len(avail) == 0 or len(unsc) == 0:
            return owner
        pos = np.array([d.pos for d in avail])
        w = (np.array([max(d.soc, 1e-3) for d in avail]) if weighted
             else np.ones(len(avail)))
        a = weighted_voronoi_assignment(pos, w, cell_centres[unsc])
        owner[unsc] = np.array([d.idx for d in avail])[a]
        return owner

    def fairness():
        avail = [d.idx for d in drones if d.available]
        if not avail:
            return np.nan
        rem = np.bincount(zone_owner[zone_owner >= 0], minlength=K_DRONES)
        return jain_fairness([rem[k] for k in avail])

    # initial partition: standard Voronoi on the start positions (all strategies)
    zone_owner = partition(drones, weighted=False)
    for d in drones:
        d.target = choose_target(d, zone_owner, scanned, cell_centres, P_flat)

    event_log = []        # (t, reason, drone idx)
    repartitions = []     # (t, J_before, J_after, SoC of available drones)
    zone_snapshots = [(0.0, zone_owner.copy(), scanned.copy())]
    t_log, coverage_log, fairness_log, n_avail_log = [], [], [], []
    frames = []

    t = 0.0
    step = 0
    while (t < T_MAX - 1e-9 and any(x is None for x in detect_times)
           and not scanned.all()):
        events = []
        for d in drones:
            if d.mode == 'search':
                if d.soc < SOC_THRESH:
                    d.mode = 'return'; d.target = None
                    events.append(('battery_out', d.idx))
                elif d.target is None or scanned[d.target]:
                    d.target = choose_target(d, zone_owner, scanned, cell_centres, P_flat)
                    if d.target is None:
                        d.mode = 'idle'
                        events.append(('zone_done', d.idx))
                if d.mode == 'search':
                    d.fly_towards(cell_centres[d.target])
                    scanned[cell_tree.query_ball_point(d.pos, SENSOR_R)] = True
                    for i, s in enumerate(survivors):
                        if detect_times[i] is None and np.linalg.norm(d.pos - s) <= SENSOR_R:
                            detect_times[i] = t + DT
                            finder[i] = d.idx
                            if d.mode == 'search':
                                d.mode = 'confirm'; d.timer = T_CONFIRM
                                events.append(('survivor', d.idx))
            elif d.mode == 'confirm':
                d.hover()
                d.timer -= DT
                if d.timer <= 1e-9:
                    d.mode = 'search'; d.target = None
                    events.append(('confirm_done', d.idx))
            elif d.mode == 'return':
                if d.fly_towards(d.home) <= 1e-9:
                    d.mode = 'swap'; d.timer = T_SWAP
            elif d.mode == 'swap':
                d.timer -= DT
                if d.timer <= 1e-9:
                    d.soc = 1.0; d.dist_pack = 0.0; d.n_swaps += 1
                    d.mode = 'search'; d.target = None
                    events.append(('swap_done', d.idx))
            d.history.append(d.pos.copy())

        t += DT
        step += 1
        zone_owner[scanned] = -1
        for reason, idx in events:
            event_log.append((t, reason, idx))
        if (strategy != 'static' and events and not scanned.all()
                and any(d.available for d in drones)):
            j_before = fairness()
            avail = [d for d in drones if d.available]
            zone_owner = partition(avail, weighted=(strategy == 'weighted'))
            for d in avail:
                d.target = choose_target(d, zone_owner, scanned, cell_centres, P_flat)
                d.mode = 'search' if d.target is not None else 'idle'
            repartitions.append((t, j_before, fairness(), [round(d.soc, 3) for d in avail]))
            if record:
                zone_snapshots.append((t, zone_owner.copy(), scanned.copy()))

        if record:
            t_log.append(t)
            coverage_log.append(scanned.mean())
            fairness_log.append(fairness())
            n_avail_log.append(sum(d.available for d in drones))
            if step % FRAME_EVERY == 0:
                frames.append((t, np.array([d.pos for d in drones]), zone_owner.copy(),
                               scanned.copy(), [x is not None for x in detect_times]))

    found = [x for x in detect_times if x is not None]
    return {
        'strategy': strategy,
        'seed': survivor_seed,
        'drones': drones,
        'survivors': survivors,
        'detect_times': detect_times,
        'finder': finder,
        't_end': t,
        'coverage_log': np.array(coverage_log),
        'fairness_log': np.array(fairness_log),
        'n_avail_log': np.array(n_avail_log),
        't_log': np.array(t_log),
        'event_log': event_log,
        'repartitions': repartitions,
        'zone_snapshots': zone_snapshots,
        'frames': frames,
        'scanned_final': scanned.copy(),
        'cell_centres': cell_centres,
        'P': P,
        'n_found': len(found),
        'time_all_found': max(found) if len(found) == N_SURVIVORS else np.nan,
        'mean_detect_time': float(np.mean(found)) if found else np.nan,
        'final_coverage': float(scanned.mean()),
        'mean_fairness': float(np.nanmean(fairness_log)) if fairness_log else np.nan,
        'n_swaps': sum(d.n_swaps for d in drones),
        'dist_total': sum(d.dist_total for d in drones),
    }


def run_monte_carlo():
    """Run every strategy on survivor placements 0..N_MC-1 (no recording)."""
    mc = {s: {'t_all': [], 't_mean': [], 'n_found': [], 'n_repart': [], 'swaps': []}
          for s in STRATEGIES}
    for seed in range(N_MC):
        for s in STRATEGIES:
            r = run_simulation(s, seed, record=False)
            mc[s]['t_all'].append(r['time_all_found'])
            mc[s]['t_mean'].append(r['mean_detect_time'])
            mc[s]['n_found'].append(r['n_found'])
            mc[s]['n_repart'].append(len(r['repartitions']))
            mc[s]['swaps'].append(r['n_swaps'])
    return {s: {k: np.array(v, dtype=float) for k, v in d.items()} for s, d in mc.items()}


# ── Plots ──────────────────────────────────────────────────────────────────

def _zone_rgba(zone_owner, alpha):
    img = np.zeros((Ny, Nx, 4))
    owner_img = cells_to_image(zone_owner)
    for k in range(K_DRONES):
        img[owner_img == k] = mcolors.to_rgba(DRONE_COLORS[k], alpha=alpha)
    return img


def _scanned_rgba(scanned):
    img = np.zeros((Ny, Nx, 4))
    img[cells_to_image(scanned)] = (0.55, 0.55, 0.55, 0.6)
    return img


def _save(fig, out_dir, name):
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, name)
    fig.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_search_area_map(result, out_dir):
    """Prior probability heatmap with initial Voronoi zones, survivors and drone starts."""
    fig, ax = plt.subplots(figsize=(8, 8))
    im = ax.imshow(result['P'].T, origin='lower', extent=[0, AREA_SIZE, 0, AREA_SIZE],
                   cmap='YlOrRd', alpha=0.55, aspect='equal')
    plt.colorbar(im, ax=ax, label='Prior probability per cell')
    ax.imshow(_zone_rgba(result['zone_snapshots'][0][1], 0.18), origin='lower',
              extent=[0, AREA_SIZE, 0, AREA_SIZE], aspect='equal')
    for k in range(K_DRONES):
        ax.plot(*STARTS[k], '^', color=DRONE_COLORS[k], markersize=12, zorder=5,
                label=f'Drone {k+1} start / home pad')
    sv = result['survivors']
    ax.scatter(sv[:, 0], sv[:, 1], s=200, marker='*', color='gold', edgecolors='black',
               zorder=6, linewidths=1.0, label=f'Survivors (placement seed {result["seed"]})')
    ax.set_xlim(0, AREA_SIZE); ax.set_ylim(0, AREA_SIZE)
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
    ax.set_title('S049: Search Area — Prior Map & Initial Voronoi Zones')
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.08), ncol=2, fontsize=8)
    _save(fig, out_dir, 'search_area_map.png')


def plot_reallocation_snapshots(result, out_dir):
    """Zones (coloured) and scanned cells (grey) at t = 0 and the 1st / 2nd re-partition."""
    snaps = result['zone_snapshots'][:3]
    fig, axes = plt.subplots(1, len(snaps), figsize=(5 * len(snaps), 5),
                             constrained_layout=True)
    axes = np.atleast_1d(axes)
    reasons = {}
    for (t, reason, idx) in result['event_log']:
        reasons.setdefault(t, []).append(f'{reason} D{idx+1}')
    for i, (ax, (t_snap, owner, sc)) in enumerate(zip(axes, snaps)):
        ax.imshow(_zone_rgba(owner, 0.55), origin='lower',
                  extent=[0, AREA_SIZE, 0, AREA_SIZE], aspect='equal')
        ax.imshow(_scanned_rgba(sc), origin='lower',
                  extent=[0, AREA_SIZE, 0, AREA_SIZE], aspect='equal')
        sv = result['survivors']
        ax.scatter(sv[:, 0], sv[:, 1], s=150, marker='*', color='gold',
                   edgecolors='black', zorder=5, linewidths=0.8)
        ax.set_xlim(0, AREA_SIZE); ax.set_ylim(0, AREA_SIZE)
        ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)' if i == 0 else '')
        if i == 0:
            ax.set_title('t = 0 s (initial Voronoi)')
        else:
            ax.set_title(f'Re-partition {i} at t = {t_snap:.1f} s\n'
                         f'({", ".join(reasons.get(t_snap, []))})', fontsize=10)
    fig.suptitle(f'S049: Zone re-partitions — {STYLE[result["strategy"]][2]} '
                 '(colour = zone owner, grey = scanned)', fontsize=12)
    _save(fig, out_dir, 'reallocation_snapshots.png')


def plot_trajectories(result, out_dir):
    """Trajectories of all drones; survivors (gold = found, grey = missed)."""
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.imshow(result['P'].T, origin='lower', extent=[0, AREA_SIZE, 0, AREA_SIZE],
              cmap='Blues', alpha=0.3, aspect='equal')
    for d in result['drones']:
        h = np.array(d.history)
        ax.plot(h[:, 0], h[:, 1], '-', color=DRONE_COLORS[d.idx], linewidth=0.7,
                alpha=0.8, label=f'Drone {d.idx+1} (battery swaps: {d.n_swaps})')
        ax.plot(*h[0], '^', color=DRONE_COLORS[d.idx], markersize=10, zorder=5)
        ax.plot(*h[-1], 's', color=DRONE_COLORS[d.idx], markersize=8, zorder=5)
    for i, s in enumerate(result['survivors']):
        found = result['detect_times'][i] is not None
        ax.scatter(*s, s=200, marker='*', color='gold' if found else 'gray',
                   edgecolors='black', zorder=7, linewidths=1.0)
        if found:
            ax.add_patch(plt.Circle(s, SENSOR_R * 1.5, fill=False, edgecolor='gold',
                                    linewidth=1.5, zorder=6))
            ax.annotate(f'{result["detect_times"][i]:.0f} s', s, xytext=(6, 6),
                        textcoords='offset points', fontsize=8)
    ax.set_xlim(0, AREA_SIZE); ax.set_ylim(0, AREA_SIZE)
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
    ax.set_title(f'S049: Drone Trajectories — {STYLE[result["strategy"]][2]}\n'
                 '(triangle = start/home, square = end, circle = survivor found)')
    ax.legend(loc='upper right', fontsize=8)
    _save(fig, out_dir, 'trajectories.png')


def plot_coverage_comparison(results_dict, out_dir):
    """Coverage and number of available drones vs time for all strategies."""
    fig, (ax, ax2) = plt.subplots(2, 1, figsize=(10, 7), sharex=True,
                                  gridspec_kw={'height_ratios': [3, 1]})
    for strat, res in results_dict.items():
        ls, color, label = STYLE[strat]
        ax.plot(res['t_log'], res['coverage_log'] * 100.0, linestyle=ls, color=color,
                linewidth=1.8, label=label)
        for tf in [x for x in res['detect_times'] if x is not None]:
            ax.plot(tf, np.interp(tf, res['t_log'], res['coverage_log'] * 100), 'o',
                    color=color, ms=5)
        ax2.step(res['t_log'], res['n_avail_log'], where='post', linestyle=ls,
                 color=color, linewidth=1.2)
    ax.set_ylabel('Coverage (%)')
    ax.set_title('S049: Area Coverage vs Time (dots = survivor detections; '
                 f'placement seed {SHOW_SEED})')
    ax.legend(loc='lower right', fontsize=9); ax.grid(True, alpha=0.3)
    ax.set_ylim(0, 102)
    ax2.set_xlabel('Time (s)'); ax2.set_ylabel('Drones\nsearching')
    ax2.set_ylim(-0.2, K_DRONES + 0.2); ax2.grid(True, alpha=0.3)
    ax2.set_xlim(left=0)
    _save(fig, out_dir, 'coverage_comparison.png')


def plot_survivor_detection_timeline(results_dict, out_dir):
    """Grouped bar chart of detection times per survivor per strategy (showcase run)."""
    fig, ax = plt.subplots(figsize=(10, 5))
    x = np.arange(N_SURVIVORS)
    width = 0.25
    for si, strat in enumerate(results_dict):
        times = [t if t is not None else 0 for t in results_dict[strat]['detect_times']]
        ax.bar(x + (si - 1) * width, times, width, label=STYLE[strat][2],
               color=STYLE[strat][1], alpha=0.85, edgecolor='black', linewidth=0.5)
    ax.set_xlabel('Survivor index'); ax.set_ylabel('Detection time (s)')
    ax.set_title(f'S049: Survivor detection times (placement seed {SHOW_SEED})')
    ax.set_xticks(x); ax.set_xticklabels([f'S{i+1}' for i in range(N_SURVIVORS)])
    ax.legend(fontsize=9); ax.grid(True, axis='y', alpha=0.3)
    _save(fig, out_dir, 'detection_timeline.png')


def plot_jain_fairness(results_dict, out_dir):
    """Jain fairness index of remaining load among available drones vs time."""
    fig, ax = plt.subplots(figsize=(10, 5))
    for strat, res in results_dict.items():
        ls, color, label = STYLE[strat]
        ax.plot(res['t_log'], res['fairness_log'], linestyle=ls, color=color,
                linewidth=1.5, label=label)
    ax.axhline(1.0, color='black', linestyle=':', alpha=0.4, label='Perfect fairness')
    ax.set_xlabel('Time (s)'); ax.set_ylabel('Jain fairness index J(t)')
    ax.set_title('S049: Load balance of remaining zone area among searching drones')
    ax.legend(loc='lower left', fontsize=9); ax.grid(True, alpha=0.3)
    ax.set_xlim(left=0); ax.set_ylim(0, 1.05)
    _save(fig, out_dir, 'jain_fairness.png')


def plot_summary_table(results_dict, mc, out_dir):
    """Monte Carlo summary (N_MC survivor placements) as a figure."""
    col_labels = ['Strategy', 'All 5 found\n(runs)', 'Time to find all\nmean ± std (s)',
                  'Mean detection\ntime, mean (s)', 'Re-partitions\nmean', 'Battery swaps\nmean']
    rows = []
    for s in STRATEGIES:
        m = mc[s]
        ok = ~np.isnan(m['t_all'])
        rows.append([
            STYLE[s][2], f'{int(ok.sum())}/{N_MC}',
            f'{np.nanmean(m["t_all"]):.1f} ± {np.nanstd(m["t_all"]):.1f}',
            f'{np.nanmean(m["t_mean"]):.1f}', f'{m["n_repart"].mean():.1f}',
            f'{m["swaps"].mean():.2f}',
        ])
    fig, ax = plt.subplots(figsize=(13, 2.8))
    ax.axis('off')
    tbl = ax.table(cellText=rows, colLabels=col_labels, cellLoc='center', loc='center',
                   colWidths=[0.26, 0.11, 0.17, 0.15, 0.13, 0.13])
    tbl.auto_set_font_size(False); tbl.set_fontsize(10); tbl.scale(1.1, 2.4)
    ax.set_title(f'S049: Monte Carlo summary over {N_MC} survivor placements', fontsize=13, pad=20)
    _save(fig, out_dir, 'summary_table.png')


def save_animation(result, out_dir):
    """Top-down animation: drones move, cells fill, zones update, survivors turn green."""
    import matplotlib.animation as animation

    frames = result['frames']
    stride = int(np.ceil(len(frames) / ANIM_MAX_FRAMES))
    frames = frames[::stride]
    survivors = result['survivors']

    fig, ax = plt.subplots(figsize=(7, 7))
    ax.set_xlim(0, AREA_SIZE); ax.set_ylim(0, AREA_SIZE); ax.set_aspect('equal')
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
    title = ax.set_title('')
    ax.imshow(result['P'].T, origin='lower', extent=[0, AREA_SIZE, 0, AREA_SIZE],
              cmap='YlOrRd', alpha=0.25, aspect='equal')
    zone_im = ax.imshow(np.zeros((Ny, Nx, 4)), origin='lower',
                        extent=[0, AREA_SIZE, 0, AREA_SIZE], aspect='equal')
    scan_im = ax.imshow(np.zeros((Ny, Nx, 4)), origin='lower',
                        extent=[0, AREA_SIZE, 0, AREA_SIZE], aspect='equal')
    markers = [ax.plot([], [], 'o', color=DRONE_COLORS[k], markersize=9, zorder=10,
                       markeredgecolor='black')[0] for k in range(K_DRONES)]
    surv = [ax.plot(s[0], s[1], '*', color='gold', markersize=14, markeredgecolor='black',
                    zorder=11)[0] for s in survivors]

    def update(i):
        t, pos, owner, sc, found = frames[i]
        zone_im.set_data(_zone_rgba(owner, 0.25))
        scan_im.set_data(_scanned_rgba(sc))
        for k in range(K_DRONES):
            markers[k].set_data([pos[k, 0]], [pos[k, 1]])
        for j, sm in enumerate(surv):
            sm.set_color('limegreen' if found[j] else 'gold')
            sm.set_markeredgecolor('black')
        title.set_text(f'S049: {STYLE[result["strategy"]][2]}  t = {t:.0f} s  '
                       f'coverage {sc.mean()*100:.0f}%  found {sum(found)}/{N_SURVIVORS}')
        return [zone_im, scan_im] + markers + surv

    ani = animation.FuncAnimation(fig, update, frames=len(frames), blit=False)
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer='pillow', fps=10, dpi=80)
    plt.close(fig)
    print(f'Saved: {path} ({len(frames)} frames, {FRAME_EVERY * stride * DT:.0f} s per frame)')


# ── Main ───────────────────────────────────────────────────────────────────

def _fmt(x):
    return 'not found' if x is None else f'{x:.1f}'


if __name__ == '__main__':
    out_dir = os.path.normpath(OUTPUT_DIR)

    results_dict = {}
    for s in STRATEGIES:
        print(f'Running {STYLE[s][2]} (survivor seed {SHOW_SEED})...')
        results_dict[s] = run_simulation(s, SHOW_SEED)

    print(f'\n=== Showcase run (survivor placement seed {SHOW_SEED}) ===')
    print('Survivors: ' + ', '.join(f'({p[0]:.1f}, {p[1]:.1f})'
                                    for p in results_dict['static']['survivors']))
    for strat, res in results_dict.items():
        print(f'\n[{strat.upper()}]')
        print(f'  Survivors found:       {res["n_found"]}/{N_SURVIVORS}')
        print('  Detection times (s):   ' + ', '.join(_fmt(x) for x in res['detect_times']))
        print('  Found by drone:        ' + ', '.join(
            '-' if f is None else f'D{f+1}' for f in res['finder']))
        print(f'  Time to find all:      {res["time_all_found"]:.1f} s')
        print(f'  Mean detection time:   {res["mean_detect_time"]:.1f} s')
        print(f'  Coverage at end:       {res["final_coverage"]*100:.1f}% (t = {res["t_end"]:.1f} s)')
        print(f'  Events:                ' + ', '.join(
            f'{r}:{sum(1 for e in res["event_log"] if e[1] == r)}'
            for r in ('survivor', 'confirm_done', 'battery_out', 'swap_done', 'zone_done')))
        print(f'  Re-partitions:         {len(res["repartitions"])}')
        print(f'  Battery swaps:         {res["n_swaps"]}  | total distance '
              f'{res["dist_total"]:.0f} m')
        print(f'  Mean Jain fairness:    {res["mean_fairness"]:.4f}')
        for (t, jb, ja, socs) in res['repartitions'][:3]:
            print(f'    re-partition t={t:.1f} s: J {jb:.3f} -> {ja:.3f}, SoC of available {socs}')

    print(f'\nRunning Monte Carlo over {N_MC} survivor placements...')
    mc = run_monte_carlo()
    print(f'\n=== Monte Carlo ({N_MC} survivor placements, seeds 0..{N_MC-1}) ===')
    for s in STRATEGIES:
        m = mc[s]
        ok = ~np.isnan(m['t_all'])
        print(f'  {STYLE[s][2]:34s}: all found {int(ok.sum())}/{N_MC} | t_all mean '
              f'{np.nanmean(m["t_all"]):.1f} s (std {np.nanstd(m["t_all"]):.1f}, '
              f'median {np.nanmedian(m["t_all"]):.1f}) | mean detection '
              f'{np.nanmean(m["t_mean"]):.1f} s | re-partitions {m["n_repart"].mean():.1f} | '
              f'swaps {m["swaps"].mean():.2f}')
    for a, b in (('equal', 'static'), ('weighted', 'static'), ('weighted', 'equal')):
        diff = mc[a]['t_all'] - mc[b]['t_all']
        print(f'  t_all {a} - {b}: mean {np.nanmean(diff):+.1f} s, '
              f'{a} faster in {int(np.sum(diff < 0))}/{N_MC}, slower in '
              f'{int(np.sum(diff > 0))}/{N_MC}, tie {int(np.sum(diff == 0))}/{N_MC}')

    print('\nGenerating plots...')
    plot_search_area_map(results_dict['weighted'], out_dir)
    plot_reallocation_snapshots(results_dict['weighted'], out_dir)
    plot_trajectories(results_dict['weighted'], out_dir)
    plot_coverage_comparison(results_dict, out_dir)
    plot_survivor_detection_timeline(results_dict, out_dir)
    plot_jain_fairness(results_dict, out_dir)
    plot_summary_table(results_dict, mc, out_dir)
    save_animation(results_dict['weighted'], out_dir)
    print('\nAll outputs saved to:', out_dir)
