"""
S051 Post-Disaster Communication Network Restoration
=====================================================
An earthquake has knocked out cellular towers across a 2000 x 2000 m city. Five
drones hover at 50 m as airborne cell towers (coverage radius 300 m) and must stay
connected to the charge station through a relay graph (radio range 600 m). Three
placement strategies are compared: greedy max-coverage (no connectivity), greedy
followed by connectivity repair, and simulated annealing with a connectivity penalty.

Usage:
    conda activate drones
    python src/03_environmental_sar/s051_post_disaster_comm.py
"""

import sys
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Circle
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401
from scipy.spatial import distance_matrix

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

# ── Parameters ────────────────────────────────────────────────────────────────
K_DRONES    = 5          # number of relay drones
AREA_SIZE   = 2000.0     # m   — side length of square area
GRID_N      = 40         # grid resolution per axis (50 m cells)
R_COV       = 300.0      # m   — ground-projected coverage radius per drone
R_COMM      = 600.0      # m   — inter-drone / drone-base radio range
Z_HOVER     = 50.0       # m   — hover altitude
V_DRONE     = 15.0       # m/s — transit speed
T_MAX       = 1800.0     # s   — battery budget
T_HOVER     = 1500.0     # s   — intended hover duration
D_MAX       = V_DRONE * (T_MAX - T_HOVER) / 2.0   # m — max reachable distance (2250 m)

# SA cost is in persons, E = -C + ALPHA * n_bad (card). With ~18.7 k people in the city,
# ALPHA = 1e6 makes one disconnected (or unreachable) drone cost ~50x the whole city.
ALPHA       = 1e6        # persons per disconnected / unreachable drone (card)
T0_SA       = 500.0      # initial SA temperature (persons, card)
T_END_SA    = 0.25       # final SA temperature (persons)
N_SA_STEPS  = 80_000     # total SA iterations
DELTA_MAX   = 200.0      # m   — max perturbation at T0
DELTA_MIN   = 10.0       # m   — perturbation floor at low temperature
SA_SEED     = 0
SENS_STEPS  = 20_000     # SA iterations per sensitivity run
EPS_REPAIR  = 1.0        # m   — margin inside R_COMM when repairing a link
DC_TOL      = 0.02       # max coverage loss per repair move (fraction of total population)
DC_TOL_SWEEP = [0.0, 0.005, 0.01, 0.015, 0.02, 0.05]

BASE_POS    = np.array([1000.0, 100.0])   # charge station

# Population clusters: (centre_x, centre_y, peak density persons/km^2, sigma m).
# Cell population = density x cell area (50 m x 50 m = 0.0025 km^2).
CELL_AREA_KM2 = (AREA_SIZE / GRID_N / 1000.0) ** 2
BACKGROUND_KM2 = 200.0   # background density ~ U(0, 200) persons/km^2
CLUSTERS = [
    (400.0, 1600.0, 8000.0, 350.0),
    (1200.0, 1200.0, 12000.0, 400.0),
    (1700.0, 700.0, 6000.0, 250.0),
]
CLUSTER_NAMES = ['Residential NW', 'Downtown', 'Industrial E']

STRATEGIES = ['greedy', 'repair', 'sa']
STRATEGY_LABELS = {
    'greedy': 'Greedy (no connectivity)',
    'repair': 'Greedy + Repair',
    'sa':     'Simulated Annealing',
}
STRATEGY_COLORS = {'greedy': '#888888', 'repair': '#E87722', 'sa': '#1F77B4'}

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '03_environmental_sar', 's051_post_disaster_comm',
)


# ── Model ─────────────────────────────────────────────────────────────────────

def build_population_map(grid_n=GRID_N, area_size=AREA_SIZE, seed=42):
    """Synthetic population density: three urban clusters plus uniform background."""
    rng = np.random.default_rng(seed)
    cell = area_size / grid_n
    xs = np.linspace(cell / 2, area_size - cell / 2, grid_n)
    cx, cy = np.meshgrid(xs, xs)
    centroids = np.stack([cx.ravel(), cy.ravel()], axis=1)     # (N, 2), row-major in y

    density = np.zeros(grid_n * grid_n)                         # persons / km^2
    for mx, my, peak, sigma in CLUSTERS:
        d2 = (centroids[:, 0] - mx) ** 2 + (centroids[:, 1] - my) ** 2
        density += peak * np.exp(-d2 / (2 * sigma ** 2))
    density += rng.uniform(0, BACKGROUND_KM2, size=density.shape)
    cell_area = (area_size / grid_n / 1000.0) ** 2
    return centroids, density * cell_area                      # persons per cell


def covered_mask(positions, centroids, r_cov=R_COV):
    """Boolean mask of cells within r_cov of at least one drone."""
    return np.any(distance_matrix(centroids, positions) <= r_cov, axis=1)


def coverage(positions, centroids, pop, r_cov=R_COV):
    return float(np.sum(pop[covered_mask(positions, centroids, r_cov)]))


def connected_mask(positions, base_pos=BASE_POS, r_comm=R_COMM):
    """BFS from the charge station; True for drones that can relay back to base."""
    nodes = np.vstack([base_pos, positions])
    adj = distance_matrix(nodes, nodes) <= r_comm
    seen = np.zeros(len(nodes), dtype=bool)
    seen[0] = True
    frontier = [0]
    while frontier:
        i = frontier.pop()
        nbr = np.where(adj[i] & ~seen)[0]
        seen[nbr] = True
        frontier.extend(nbr.tolist())
    return seen[1:]


def comm_edges(positions, base_pos=BASE_POS, r_comm=R_COMM):
    """List of (i, j) node-index pairs within radio range; node 0 is the base."""
    nodes = np.vstack([base_pos, positions])
    d = distance_matrix(nodes, nodes)
    n = len(nodes)
    return [(i, j) for i in range(n) for j in range(i + 1, n) if d[i, j] <= r_comm]


def graph_components(positions, base_pos=BASE_POS, r_comm=R_COMM):
    """Connected-component label for every node (node 0 = base, label 0 = base's component)."""
    nodes = np.vstack([base_pos, positions])
    adj = distance_matrix(nodes, nodes) <= r_comm
    labels = -np.ones(len(nodes), dtype=int)
    n_comp = 0
    for start in range(len(nodes)):
        if labels[start] >= 0:
            continue
        labels[start] = n_comp
        frontier = [start]
        while frontier:
            i = frontier.pop()
            nbr = np.where(adj[i] & (labels < 0))[0]
            labels[nbr] = n_comp
            frontier.extend(nbr.tolist())
        n_comp += 1
    return labels


def unreachable_mask(positions, base_pos=BASE_POS):
    return np.linalg.norm(positions - base_pos, axis=1) > D_MAX


# ── Strategies ────────────────────────────────────────────────────────────────

def greedy_placement(centroids, pop, k_drones=K_DRONES, r_cov=R_COV):
    """Sequential max-marginal-coverage placement on cell centroids."""
    cover = (distance_matrix(centroids, centroids) <= r_cov).astype(float)
    remaining = pop.copy()
    positions = []
    for _ in range(k_drones):
        gains = cover @ remaining
        idx = int(np.argmax(gains))
        positions.append(centroids[idx].copy())
        remaining[cover[idx] > 0] = 0.0
    return np.array(positions)


def connectivity_repair(positions, centroids, pop, dc_tol=DC_TOL,
                        base_pos=BASE_POS, r_comm=R_COMM, eps=EPS_REPAIR):
    """Pull disconnected drones toward the connected component, subject to a coverage tolerance.

    Each pass ranks all (disconnected drone, connected anchor) pairs by distance (base
    included as an anchor). The candidate move puts the drone r_comm - eps from the anchor
    along the line between them (minimum displacement that restores the link). A move is
    accepted only if the coverage it loses is at most dc_tol (fraction of total population),
    as in the card's step 4; otherwise the next-nearest pair is tried. If no pair passes the
    test, repair stops and the remaining drones stay disconnected.

    Returns the repaired positions, the accepted moves (k, old, new, loss) and the number
    of candidate moves rejected by the tolerance test.
    """
    pos = positions.copy()
    total = pop.sum()
    moves, n_rejected = [], 0
    for _ in range(2 * len(pos)):
        conn = connected_mask(pos, base_pos, r_comm)
        if conn.all():
            break
        anchors = np.vstack([base_pos, pos[conn]])
        disc = np.where(~conn)[0]
        d = distance_matrix(pos[disc], anchors)
        cov_now = coverage(pos, centroids, pop)
        accepted = False
        for flat in np.argsort(d, axis=None, kind='stable'):
            i, j = np.unravel_index(flat, d.shape)
            k, anchor = disc[i], anchors[j]
            direction = (pos[k] - anchor) / d[i, j]
            trial = pos.copy()
            trial[k] = anchor + (r_comm - eps) * direction
            loss = (cov_now - coverage(trial, centroids, pop)) / total
            if loss <= dc_tol:
                moves.append((int(k), pos[k].copy(), trial[k].copy(), float(loss)))
                pos = trial
                accepted = True
                break
            n_rejected += 1
        if not accepted:
            break
    return pos, moves, n_rejected


def simulated_annealing(centroids, pop, k_drones=K_DRONES, r_cov=R_COV,
                        n_steps=N_SA_STEPS, seed=SA_SEED, n_frames=0):
    """SA over continuous hover positions with a hard connectivity/range penalty.

    E = -C + ALPHA * n_bad in persons; history stores E, coverage fraction and n_bad.
    """
    rng = np.random.default_rng(seed)
    total = pop.sum()
    beta = (T_END_SA / T0_SA) ** (1.0 / n_steps)

    def energy(cover_cols, pos):
        cov = pop[cover_cols.any(axis=1)].sum()                  # persons
        n_bad = int(np.sum(~connected_mask(pos) | unreachable_mask(pos)))
        return -cov + ALPHA * n_bad, cov / total, n_bad

    pos = rng.uniform(0, AREA_SIZE, size=(k_drones, 2))
    cols = np.stack([np.linalg.norm(centroids - p, axis=1) <= r_cov for p in pos], axis=1)
    E, cov, pen = energy(cols, pos)
    best_E, best_pos = E, pos.copy()

    hist_E = np.empty(n_steps)
    hist_cov = np.empty(n_steps)
    hist_pen = np.empty(n_steps)
    frame_every = max(1, n_steps // n_frames) if n_frames else 0
    frames = []
    T = T0_SA

    for step in range(n_steps):
        k = rng.integers(k_drones)
        delta = max(DELTA_MAX * T / T0_SA, DELTA_MIN)
        new_p = np.clip(pos[k] + rng.uniform(-delta, delta, size=2), 0.0, AREA_SIZE)
        new_pos = pos.copy()
        new_pos[k] = new_p
        new_cols = cols.copy()
        new_cols[:, k] = np.linalg.norm(centroids - new_p, axis=1) <= r_cov
        E_new, cov_new, pen_new = energy(new_cols, new_pos)
        dE = E_new - E
        if dE <= 0 or rng.random() < np.exp(-dE / T):
            pos, cols, E, cov, pen = new_pos, new_cols, E_new, cov_new, pen_new
            if E < best_E:
                best_E, best_pos = E, pos.copy()
        hist_E[step], hist_cov[step], hist_pen[step] = E, cov, pen
        if frame_every and step % frame_every == 0:
            frames.append((step, pos.copy(), cov, pen))
        T *= beta

    if n_frames:
        best_cov = coverage(best_pos, centroids, pop, r_cov) / total
        frames.append((n_steps, best_pos.copy(), best_cov, 0))
    history = {'E': hist_E, 'cov': hist_cov, 'n_bad': hist_pen}
    return best_pos, history, frames


# ── Simulation ────────────────────────────────────────────────────────────────

def summarise(positions, centroids, pop):
    conn = connected_mask(positions)
    dist_base = np.linalg.norm(positions - BASE_POS, axis=1)
    return {
        'pos': positions,
        'cov_frac': coverage(positions, centroids, pop) / pop.sum(),
        'connected': conn,
        'n_connected': int(conn.sum()),
        'max_base_dist': float(dist_base.max()),
        'round_trip_s': float(2 * dist_base.max() / V_DRONE),
    }


def run_simulation():
    centroids, pop = build_population_map()

    greedy_pos = greedy_placement(centroids, pop)
    repair_pos, repair_moves, n_rej = connectivity_repair(greedy_pos, centroids, pop)
    sa_pos, sa_hist, sa_frames = simulated_annealing(centroids, pop, n_frames=100)

    results = {
        'greedy': summarise(greedy_pos, centroids, pop),
        'repair': summarise(repair_pos, centroids, pop),
        'sa':     summarise(sa_pos, centroids, pop),
    }
    results['repair']['moves'] = repair_moves
    results['repair']['n_rejected'] = n_rej

    # Effect of the repair coverage tolerance DC_TOL
    tol_sweep = []
    for tol in DC_TOL_SWEEP:
        p_tol, m_tol, rej_tol = connectivity_repair(greedy_pos, centroids, pop, dc_tol=tol)
        s_tol = summarise(p_tol, centroids, pop)
        tol_sweep.append((tol, s_tol['cov_frac'], s_tol['n_connected'], len(m_tol), rej_tol))
    results['repair']['tol_sweep'] = tol_sweep

    # Sensitivity of the SA solution to coverage radius and fleet size
    r_values = np.array([250.0, 300.0, 350.0, 400.0, 450.0])
    k_values = np.arange(1, 9)
    sens_r = np.array([
        coverage(simulated_annealing(centroids, pop, r_cov=r, n_steps=SENS_STEPS)[0],
                 centroids, pop, r) / pop.sum()
        for r in r_values])
    sens_k = np.array([
        coverage(simulated_annealing(centroids, pop, k_drones=int(k), n_steps=SENS_STEPS)[0],
                 centroids, pop) / pop.sum()
        for k in k_values])
    sensitivity = {'r': r_values, 'cov_r': sens_r, 'k': k_values, 'cov_k': sens_k}

    return centroids, pop, results, sa_hist, sa_frames, sensitivity


# ── Plotting ──────────────────────────────────────────────────────────────────

def _pop_image(pop):
    return pop.reshape(GRID_N, GRID_N)


def _draw_network(ax, positions, color, show_cov=True):
    conn = connected_mask(positions)
    nodes = np.vstack([BASE_POS, positions])
    for i, j in comm_edges(positions):
        ax.plot(*zip(nodes[i], nodes[j]), '--', color='k', lw=1.0, alpha=0.7, zorder=3)
    for k, p in enumerate(positions):
        c = color if conn[k] else 'red'
        if show_cov:
            ax.add_patch(Circle(p, R_COV, fill=False, ec=c, lw=1.5, zorder=4))
        ax.plot(*p, 'o' if conn[k] else 'X', color=c, ms=9, mec='k', zorder=5)
        ax.annotate(f'D{k + 1}', p, xytext=(6, 6), textcoords='offset points',
                    fontsize=8, zorder=6)
    ax.plot(*BASE_POS, '*', color='k', ms=16, zorder=6)


def plot_population_map(centroids, pop, out_dir):
    fig, ax = plt.subplots(figsize=(7, 6))
    im = ax.imshow(_pop_image(pop) / CELL_AREA_KM2, origin='lower',
                   extent=[0, AREA_SIZE, 0, AREA_SIZE], cmap='YlOrRd')
    fig.colorbar(im, ax=ax, label='Population density (persons/km²)')
    for (mx, my, peak, _), name in zip(CLUSTERS, CLUSTER_NAMES):
        ax.annotate(f'{name}\npeak {peak:,.0f}/km²', (mx, my), ha='center', fontsize=9,
                    bbox=dict(boxstyle='round', fc='white', alpha=0.8))
    ax.plot(*BASE_POS, '*', color='k', ms=16, label='Charge station')
    ax.add_patch(Circle(BASE_POS, R_COMM, fill=False, ls=':', ec='k',
                        label=f'Base radio range {R_COMM:.0f} m'))
    ax.set_xlim(0, AREA_SIZE)
    ax.set_ylim(0, AREA_SIZE)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title(f'S051 Population Density — {pop.sum():,.0f} persons in '
                 f'{AREA_SIZE ** 2 / 1e6:.0f} km²')
    ax.legend(loc='lower left', fontsize=8)
    path = os.path.join(out_dir, 'population_map.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_coverage_comparison(centroids, pop, results, out_dir):
    fig, axes = plt.subplots(1, 3, figsize=(18, 6))
    for ax, s in zip(axes, STRATEGIES):
        r = results[s]
        mask = covered_mask(r['pos'], centroids)
        img = np.where(mask, pop, np.nan).reshape(GRID_N, GRID_N)
        ax.imshow(_pop_image(pop), origin='lower', extent=[0, AREA_SIZE, 0, AREA_SIZE],
                  cmap='Greys', alpha=0.35)
        ax.imshow(img, origin='lower', extent=[0, AREA_SIZE, 0, AREA_SIZE], cmap='YlOrRd')
        _draw_network(ax, r['pos'], STRATEGY_COLORS[s])
        status = 'connected' if r['n_connected'] == K_DRONES else \
            f"{K_DRONES - r['n_connected']} drone(s) DISCONNECTED"
        ax.set_title(f"{STRATEGY_LABELS[s]}\ncoverage {100 * r['cov_frac']:.1f}% — {status}",
                     fontsize=11)
        ax.set_xlim(0, AREA_SIZE)
        ax.set_ylim(0, AREA_SIZE)
        ax.set_xlabel('x (m)')
        ax.set_aspect('equal')
    axes[0].set_ylabel('y (m)')
    for k, old, new, _ in results['repair'].get('moves', []):
        axes[1].annotate('', xy=new, xytext=old,
                         arrowprops=dict(arrowstyle='->', color='purple', lw=1.5))
    fig.suptitle('S051 Coverage Comparison (coloured cells = served population; '
                 'dashed = radio links; red X = disconnected; '
                 'purple arrows = repair moves)', fontsize=12, y=1.03)
    plt.tight_layout()
    path = os.path.join(out_dir, 'coverage_comparison.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_relay_graph(centroids, pop, results, out_dir):
    """Relay graph per strategy: connected components coloured, drones cut off from base in red."""
    comp_colors = ['#2CA02C', '#9467BD', '#17BECF', '#BCBD22', '#8C564B', '#E377C2']
    fig, axes = plt.subplots(1, 3, figsize=(18, 6))
    n_max = 1
    for ax, s in zip(axes, STRATEGIES):
        pos = results[s]['pos']
        labels = graph_components(pos)
        n_max = max(n_max, labels.max() + 1)
        nodes = np.vstack([BASE_POS, pos])
        ax.imshow(_pop_image(pop), origin='lower', extent=[0, AREA_SIZE, 0, AREA_SIZE],
                  cmap='Greys', alpha=0.35)
        for i, j in comm_edges(pos):
            ax.plot(*zip(nodes[i], nodes[j]), '-', color=comp_colors[labels[i]], lw=2.5,
                    zorder=3)
        for n, p in enumerate(nodes):
            col = comp_colors[labels[n]]
            if n == 0:
                ax.plot(*p, '*', color=col, ms=22, mec='k', zorder=5)
                ax.annotate('Base', p, xytext=(8, -12), textcoords='offset points', fontsize=9)
                continue
            cut = labels[n] != labels[0]
            ax.plot(*p, 'o', color=col, ms=13, mec='red' if cut else 'k',
                    mew=3.0 if cut else 1.0, zorder=5)
            ax.annotate(f'D{n}', p, xytext=(8, 6), textcoords='offset points', fontsize=9,
                        color='red' if cut else 'k', zorder=6)
        n_comp = labels.max() + 1
        n_cut = int(np.sum(labels[1:] != labels[0]))
        ax.set_title(f"{STRATEGY_LABELS[s]}\n{n_comp} component(s), "
                     f"{n_cut} drone(s) cut off from base", fontsize=11)
        ax.set_xlim(0, AREA_SIZE)
        ax.set_ylim(0, AREA_SIZE)
        ax.set_xlabel('x (m)')
        ax.set_aspect('equal')
    axes[0].set_ylabel('y (m)')
    from matplotlib.lines import Line2D
    handles = [Line2D([], [], color=comp_colors[0], marker='o', lw=2.5, mec='k',
                      label='component containing the base')]
    handles += [Line2D([], [], color=comp_colors[c], marker='o', lw=2.5, mec='k',
                       label=f'other component {c}') for c in range(1, n_max)]
    handles.append(Line2D([], [], ls='', marker='o', color='w', mec='red', mew=3.0, ms=11,
                          label='drone cut off from base'))
    fig.legend(handles=handles, loc='lower center', ncol=len(handles), fontsize=9,
               bbox_to_anchor=(0.5, -0.06))
    fig.suptitle(f'S051 Relay Graph (edges = links within r_comm = {R_COMM:.0f} m)',
                 fontsize=12, y=1.02)
    plt.tight_layout()
    path = os.path.join(out_dir, 'relay_graph.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_network_3d(centroids, pop, results, out_dir):
    """3D view of the SA relay network hovering over the population surface."""
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d', computed_zorder=False)
    xs = np.linspace(0, AREA_SIZE, GRID_N)
    X, Y = np.meshgrid(xs, xs)
    P = _pop_image(pop)
    colors = plt.cm.YlOrRd(P / P.max())
    ax.plot_surface(X, Y, np.zeros_like(X), facecolors=colors, rstride=1, cstride=1,
                    shade=False, alpha=0.8, linewidth=0, zorder=0)

    pos = results['sa']['pos']
    nodes3 = np.vstack([np.append(BASE_POS, 0.0),
                        np.column_stack([pos, np.full(len(pos), Z_HOVER)])])
    for i, j in comm_edges(pos):
        ax.plot(*zip(nodes3[i], nodes3[j]), 'k--', lw=1.2, zorder=3)
    theta = np.linspace(0, 2 * np.pi, 60)
    for k, p in enumerate(pos):
        ax.plot([p[0], p[0]], [p[1], p[1]], [0, Z_HOVER], ':', color='gray', zorder=2)
        ax.plot(p[0] + R_COV * np.cos(theta), p[1] + R_COV * np.sin(theta),
                np.zeros_like(theta), color=STRATEGY_COLORS['sa'], lw=1.5, zorder=1)
        ax.scatter(*p, Z_HOVER, color=STRATEGY_COLORS['sa'], s=60, edgecolor='k',
                   depthshade=False, zorder=4)
        ax.text(p[0], p[1], Z_HOVER + 8, f'D{k + 1}', fontsize=9, zorder=5)
    ax.scatter(*BASE_POS, 0, marker='*', color='k', s=250, depthshade=False, zorder=4)
    ax.text(BASE_POS[0], BASE_POS[1], 12, 'Base', fontsize=9)
    ax.set_xlim(0, AREA_SIZE)
    ax.set_ylim(0, AREA_SIZE)
    ax.set_zlim(0, 120)
    ax.set_box_aspect((1, 1, 0.35))
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_zlabel('z (m)')
    ax.view_init(elev=28, azim=-60)
    ax.set_title(f"S051 SA Relay Network at z = {Z_HOVER:.0f} m — "
                 f"coverage {100 * results['sa']['cov_frac']:.1f}%")
    path = os.path.join(out_dir, 'network_3d.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_coverage_bar(results, out_dir):
    fig, ax = plt.subplots(figsize=(8, 5))
    vals = [100 * results[s]['cov_frac'] for s in STRATEGIES]
    bars = ax.bar([STRATEGY_LABELS[s] for s in STRATEGIES], vals,
                  color=[STRATEGY_COLORS[s] for s in STRATEGIES], edgecolor='k')
    for b, s, v in zip(bars, STRATEGIES, vals):
        ok = results[s]['n_connected'] == K_DRONES
        b.set_hatch('' if ok else '//')
        ax.text(b.get_x() + b.get_width() / 2, v / 2,
                f"{v:.1f}%\n{'connected' if ok else 'disconnected'}",
                ha='center', va='center', fontsize=10,
                bbox=dict(boxstyle='round', fc='white', alpha=0.85))
    best_feasible = max(v for s, v in zip(STRATEGIES, vals)
                        if results[s]['n_connected'] == K_DRONES)
    opt_ub = vals[0] / (1 - 1 / np.e)
    ax.axhline(opt_ub, color='k', ls='--', lw=1,
               label=rf'Upper bound on OPT (discrete sites, no connectivity): '
                     rf'greedy$/(1-1/e)$ = {opt_ub:.1f}%')
    ax.axhline(best_feasible, color=STRATEGY_COLORS['sa'], ls=':', lw=1,
               label=f'Best connected solution = {best_feasible:.1f}%')
    ax.set_ylabel('Covered population (%)')
    ax.set_ylim(0, 125)
    ax.set_title('S051 Coverage Fraction by Strategy (hatched = violates connectivity)')
    ax.legend(fontsize=8, loc='upper center')
    path = os.path.join(out_dir, 'coverage_bar.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_sa_convergence(history, out_dir):
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
    it = np.arange(1, len(history['E']) + 1)
    ax1.plot(it, history['E'], color=STRATEGY_COLORS['sa'], lw=0.8, label='Cost E')
    ax1.set_yscale('symlog', linthresh=1e2)
    ax1.set_ylabel('E = −C + α·n_bad (persons)')
    ax1.legend()
    ax1.grid(alpha=0.3)
    ax2.plot(it, 100 * history['cov'], color='tab:green', lw=0.8, label='Coverage (%)')
    ax2b = ax2.twinx()
    ax2b.plot(it, history['n_bad'], color='red', lw=0.8, label='n_bad (disconnected or out-of-range drones)')
    ax2b.set_ylabel('n_bad (drones)', color='red')
    feasible = np.where(history['n_bad'] == 0)[0]
    if len(feasible):
        bad = np.where(history['n_bad'] > 0)[0]
        first_stable = int(bad[-1] + 1) if len(bad) else 0
        ax2.axvline(first_stable, color='k', ls='--', lw=1)
        ax2.text(first_stable, 0.05, f'  connected from iter {first_stable}', fontsize=9,
                 transform=ax2.get_xaxis_transform())
    ax2.set_xscale('log')
    ax2.set_xlabel('SA iteration (log scale)')
    ax2.set_ylabel('Coverage (%)', color='tab:green')
    ax2.grid(alpha=0.3)
    fig.suptitle('S051 Simulated Annealing Convergence')
    plt.tight_layout()
    path = os.path.join(out_dir, 'sa_convergence.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_sensitivity(sens, out_dir):
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4.5))
    ax1.plot(sens['r'], 100 * sens['cov_r'], 'o-', color=STRATEGY_COLORS['sa'])
    ax1.set_xlabel('Coverage radius r_cov (m)')
    ax1.set_ylabel('Covered population (%)')
    ax1.set_title(f'Coverage vs r_cov (K = {K_DRONES})')
    ax1.grid(alpha=0.3)
    ax2.plot(sens['k'], 100 * sens['cov_k'], 's-', color=STRATEGY_COLORS['sa'])
    ax2.set_xlabel('Number of drones K')
    ax2.set_ylabel('Covered population (%)')
    ax2.set_title(f'Coverage vs K (r_cov = {R_COV:.0f} m)')
    ax2.grid(alpha=0.3)
    fig.suptitle(f'S051 Sensitivity of the connected SA solution ({SENS_STEPS:,} iterations each)')
    plt.tight_layout()
    path = os.path.join(out_dir, 'sensitivity.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def save_animation(frames, centroids, pop, out_dir):
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.imshow(_pop_image(pop), origin='lower', extent=[0, AREA_SIZE, 0, AREA_SIZE],
              cmap='YlOrRd', alpha=0.85)
    ax.plot(*BASE_POS, '*', color='k', ms=16)
    ax.set_xlim(0, AREA_SIZE)
    ax.set_ylim(0, AREA_SIZE)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    artists = []

    def update(i):
        for a in artists:
            a.remove()
        artists.clear()
        step, pos, cov, pen = frames[i]
        conn = connected_mask(pos)
        nodes = np.vstack([BASE_POS, pos])
        for a, b in comm_edges(pos):
            artists.extend(ax.plot(*zip(nodes[a], nodes[b]), 'k--', lw=1))
        for k, p in enumerate(pos):
            c = STRATEGY_COLORS['sa'] if conn[k] else 'red'
            circ = Circle(p, R_COV, fill=True, fc=c, alpha=0.18, ec=c, lw=1.5)
            ax.add_patch(circ)
            artists.append(circ)
            artists.extend(ax.plot(*p, 'o' if conn[k] else 'X', color=c, ms=9, mec='k'))
        state = 'connected' if conn.all() else f'{int((~conn).sum())} disconnected'
        ax.set_title(f'S051 SA progress — iter {step:,}\n'
                     f'coverage {100 * cov:.1f}%, {state}')
        return artists

    anim = animation.FuncAnimation(fig, update, frames=len(frames), interval=80, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    anim.save(path, writer='pillow', fps=12)
    plt.close(fig)
    print(f'Saved: {path}')


if __name__ == '__main__':
    centroids, pop, results, sa_hist, sa_frames, sens = run_simulation()
    total = pop.sum()

    print('\n=== S051 Post-Disaster Comm Network — Results ===')
    print(f'  Total population: {total:,.0f} persons in {AREA_SIZE ** 2 / 1e6:.0f} km² '
          f'(mean {total / (AREA_SIZE ** 2 / 1e6):,.0f}/km², '
          f'max cell {pop.max() / CELL_AREA_KM2:,.0f}/km² = {pop.max():.1f} persons/cell)')
    print(f'  Penalty ALPHA = {ALPHA:,.0f} persons = {ALPHA / total:.1f} x total population')
    print(f'  D_max (range constraint): {D_MAX:.0f} m')
    beta = (T_END_SA / T0_SA) ** (1.0 / N_SA_STEPS)
    n_cold_card = int(np.ceil(np.log(1.0 / T0_SA) / np.log(0.995)))
    print(f'  SA cooling: beta = {beta:.6f}; card beta = 0.995 takes T below 1 person '
          f'after {n_cold_card} steps')
    print(f'  SA step size at T_end without floor: {DELTA_MAX * T_END_SA / T0_SA:.2f} m '
          f'(floor DELTA_MIN = {DELTA_MIN:.0f} m)')
    for s in STRATEGIES:
        r = results[s]
        print(f'  [{STRATEGY_LABELS[s]}]')
        print(f"    Coverage:          {100 * r['cov_frac']:.2f}%")
        print(f"    Connected drones:  {r['n_connected']}/{K_DRONES}")
        print(f"    Graph components:  {graph_components(r['pos']).max() + 1}")
        print(f"    Max dist to base:  {r['max_base_dist']:.0f} m "
              f"(round trip {r['round_trip_s']:.0f} s)")
        print('    Positions:', ', '.join(f'({p[0]:.0f},{p[1]:.0f})' for p in r['pos']))
    g_dist = np.linalg.norm(results['greedy']['pos'] - BASE_POS, axis=1)
    print(f'  Greedy drone nearest to base: D{int(np.argmin(g_dist)) + 1} '
          f'at {g_dist.min():.0f} m')
    rep = results['repair']
    print(f'  Repair (DC_tol = {100 * DC_TOL:.1f}% = {DC_TOL * total:.0f} persons): '
          f"{len(rep['moves'])} moves accepted, {rep['n_rejected']} candidates rejected")
    for k, old, new, loss in rep['moves']:
        print(f'    D{k + 1}: ({old[0]:.0f},{old[1]:.0f}) -> ({new[0]:.0f},{new[1]:.0f}), '
              f'moved {np.linalg.norm(new - old):.0f} m, '
              f'{np.linalg.norm(new - BASE_POS):.0f} m from base, '
              f'coverage loss {100 * loss:+.2f} pp ({loss * total:+.0f} persons)')
    print(f"  Repair coverage loss vs greedy: "
          f"{100 * (results['greedy']['cov_frac'] - rep['cov_frac']):.2f} pp")
    print('  DC_tol sweep (tol -> coverage, connected, moves, rejected):')
    for tol, cf, nc, nm, nr in rep['tol_sweep']:
        print(f'    {100 * tol:4.1f}% -> {100 * cf:.2f}%, {nc}/{K_DRONES}, {nm}, {nr}')
    ratio = results['sa']['cov_frac'] / results['greedy']['cov_frac']
    print(f'  SA / unconstrained greedy coverage ratio: {ratio:.3f}')
    opt_ub = results['greedy']['cov_frac'] / (1 - 1 / np.e)
    print(f'  Upper bound on discrete unconstrained OPT, greedy/(1-1/e): {100 * opt_ub:.1f}%')
    bad = np.where(sa_hist['n_bad'] > 0)[0]
    first_ok = int(bad[-1] + 1) if len(bad) else 0
    print(f'  SA connected from iteration: {first_ok} '
          f"(coverage then {100 * sa_hist['cov'][first_ok]:.1f}%)")
    print('  Sensitivity r_cov:', ', '.join(f'{r:.0f}m→{100 * c:.1f}%'
                                           for r, c in zip(sens['r'], sens['cov_r'])))
    print('  Sensitivity K:    ', ', '.join(f'{k}→{100 * c:.1f}%'
                                           for k, c in zip(sens['k'], sens['cov_k'])))
    print('  Gain per added drone (pp):', ', '.join(
        f'{k - 1}→{k}: {100 * d:+.1f}' for k, d in zip(sens['k'][1:], np.diff(sens['cov_k']))))

    out = os.path.normpath(OUTPUT_DIR)
    os.makedirs(out, exist_ok=True)
    plot_population_map(centroids, pop, out)
    plot_coverage_comparison(centroids, pop, results, out)
    plot_relay_graph(centroids, pop, results, out)
    plot_network_3d(centroids, pop, results, out)
    plot_coverage_bar(results, out)
    plot_sa_convergence(sa_hist, out)
    plot_sensitivity(sens, out)
    save_animation(sa_frames, centroids, pop, out)
