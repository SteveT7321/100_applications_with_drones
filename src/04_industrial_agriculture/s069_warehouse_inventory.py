"""
S069 Automated Warehouse Inventory
==================================
One drone scans 60 shelf faces (20 rack columns x 3 tiers) in a 30 x 20 x 6 m
GPS-denied warehouse. Paths are planned with 26-connected A* on a 0.5 m lattice; the
pairwise cost matrix comes from Dijkstra (same optimal lengths) and every flown leg is
re-planned with A* and checked against it. The script compares

  * visiting orders: greedy nearest neighbour (card) vs nearest neighbour + 2-opt;
  * scan policies: one 1 s dwell per face (card) vs re-dwelling until the read succeeds
    (capped), under the card's hover-velocity noise and smaller noise levels;

and checks the card's speed-accuracy optimum v* numerically.

Usage:
    conda activate drones
    python src/04_industrial_agriculture/s069_warehouse_inventory.py
"""

import os
import heapq
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import dijkstra
from scipy.optimize import brentq

# ── Parameters (card) ─────────────────────────────────────────────────────────
WH          = np.array([30.0, 20.0, 6.0])
RES         = 0.5
N_COLS      = 20
TIER_Z      = [1.0, 3.0, 5.0]
D_SCAN      = 0.5        # m stand-off from the shelf face
T_DWELL     = 1.0        # s per scan attempt
V_CRUISE    = 1.0        # m/s
V_THRESH    = 0.2        # m/s
K_SCAN      = 3.0
POS_ERROR   = 0.1        # m (card: GPS_ERROR) indoor position noise std per axis
DT_CTRL     = 0.1        # s (card code divides by 0.1)
SIGMA_V     = POS_ERROR / DT_CTRL      # 1.0 m/s, card formula
SHELF_INFL  = 0.2        # m nodes within 0.2 m of a rack are occupied
AISLE_W     = 1.5        # m
ETA_REQ     = 0.95

# ── Parameters (added) ───────────────────────────────────────────────────────
RACK_X      = (5.0, 25.0)              # racks run along x; 5 columns of 4 m each
RACK_DEPTH  = 1.0                      # m
RACK_Y0     = [4.0 + 2.5 * k for k in range(4)]   # rack y-starts (1.5 m aisles)
DEPOT       = np.array([2.0, 10.0, 1.0])
N_MC        = 500
SEED        = 7
K_MAX       = 30                       # max dwell attempts per face (retry policy)
SIGMA_SWEEP = [1.0, 0.5, 0.2, 0.1, 0.05]
ANIM_STEP_S = 2.0                      # s of mission time per GIF frame

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), '..', '..', 'outputs',
                          '04_industrial_agriculture', 's069_warehouse_inventory')

NX, NY, NZ = (WH / RES).astype(int) + 1        # 61 x 41 x 13 lattice nodes
OFFS = np.array([(dx, dy, dz) for dx in (-1, 0, 1) for dy in (-1, 0, 1) for dz in (-1, 0, 1)
                 if (dx, dy, dz) != (0, 0, 0)])


# ── Map ──────────────────────────────────────────────────────────────────────

def rack_boxes():
    return [((RACK_X[0], y0, 0.0), (RACK_X[1], y0 + RACK_DEPTH, WH[2])) for y0 in RACK_Y0]


def dist_to_racks(p):
    """Euclidean distance from points p (M,3) to the nearest rack box."""
    d = np.full(len(p), np.inf)
    for lo, hi in rack_boxes():
        lo, hi = np.array(lo), np.array(hi)
        q = np.clip(p, lo, hi)
        d = np.minimum(d, np.linalg.norm(p - q, axis=1))
    return d


def build_grid():
    ix, iy, iz = np.meshgrid(np.arange(NX), np.arange(NY), np.arange(NZ), indexing='ij')
    pts = np.stack([ix, iy, iz], -1).reshape(-1, 3) * RES
    occ = dist_to_racks(pts) <= SHELF_INFL + 1e-9
    occ = occ.reshape(NX, NY, NZ)
    occ[0, :, :] = occ[-1, :, :] = True
    occ[:, 0, :] = occ[:, -1, :] = True
    occ[:, :, 0] = occ[:, :, -1] = True
    return occ


def targets():
    """60 scan waypoints: +y face of each rack, column centres, three tiers."""
    out, meta = [], []
    for r, y0 in enumerate(RACK_Y0):
        for c in range(N_COLS // len(RACK_Y0)):
            x = RACK_X[0] + 4.0 * c + 2.0
            for t, z in enumerate(TIER_Z):
                out.append([x, y0 + RACK_DEPTH + D_SCAN, z])
                meta.append((r, c, t))
    return np.array(out), meta


def node_id(p):
    i = np.round(np.asarray(p) / RES).astype(int)
    return int(np.ravel_multi_index(tuple(i), (NX, NY, NZ)))


def node_pos(n):
    return np.array(np.unravel_index(n, (NX, NY, NZ))) * RES


def build_graph(occ):
    free = ~occ
    idx = np.arange(NX * NY * NZ).reshape(NX, NY, NZ)
    rows, cols, w = [], [], []
    for o in OFFS:
        if tuple(o) <= (0, 0, 0):
            continue                       # each undirected edge once
        sl_a = tuple(slice(max(0, -d), NX_ - max(0, d)) for d, NX_ in zip(o, (NX, NY, NZ)))
        sl_b = tuple(slice(max(0, d), NX_ - max(0, -d)) for d, NX_ in zip(o, (NX, NY, NZ)))
        ok = free[sl_a] & free[sl_b]
        rows.append(idx[sl_a][ok])
        cols.append(idx[sl_b][ok])
        w.append(np.full(ok.sum(), RES * np.linalg.norm(o)))
    rows, cols, w = map(np.concatenate, (rows, cols, w))
    n = NX * NY * NZ
    return coo_matrix((w, (rows, cols)), shape=(n, n)).tocsr()


def astar(occ, start, goal):
    """26-connected A* with Euclidean heuristic; returns (node path, length)."""
    s, g = node_id(start), node_id(goal)
    gp = node_pos(g)
    best = {s: 0.0}
    parent = {s: -1}
    closed = set()
    heap = [(np.linalg.norm(node_pos(s) - gp), 0.0, s)]
    steps = RES * np.linalg.norm(OFFS, axis=1)
    while heap:
        f, gc, n = heapq.heappop(heap)
        if n == g:
            path = [n]
            while parent[path[-1]] != -1:
                path.append(parent[path[-1]])
            return path[::-1], gc
        if n in closed:
            continue
        closed.add(n)
        i = np.array(np.unravel_index(n, (NX, NY, NZ)))
        nb = i + OFFS
        ok = np.all((nb >= 0) & (nb < (NX, NY, NZ)), axis=1)
        for (a, b, c), st in zip(nb[ok], steps[ok]):
            if occ[a, b, c]:
                continue
            m = (a * NY + b) * NZ + c
            g2 = gc + st
            if g2 < best.get(m, np.inf) - 1e-12:
                best[m] = g2
                parent[m] = n
                h = np.linalg.norm(np.array([a, b, c]) * RES - gp)
                heapq.heappush(heap, (g2 + h, g2, m))
    return [], np.inf


# ── Tours ────────────────────────────────────────────────────────────────────

def nn_tour(D):
    """Greedy nearest neighbour from the depot (index 0); open path."""
    n = len(D)
    tour, left = [0], set(range(1, n))
    while left:
        last = tour[-1]
        nxt = min(left, key=lambda j: (D[last, j], j))
        tour.append(nxt)
        left.remove(nxt)
    return tour


def path_cost(D, tour):
    return float(sum(D[a, b] for a, b in zip(tour[:-1], tour[1:])))


def two_opt(D, tour):
    """2-opt on an open path with a fixed start (depot) and a free end."""
    t = list(tour)
    improved = True
    while improved:
        improved = False
        for i in range(1, len(t) - 1):
            for j in range(i + 1, len(t)):
                a, b = t[i - 1], t[i]
                c = t[j]
                d = t[j + 1] if j + 1 < len(t) else None
                old = D[a, b] + (D[c, d] if d is not None else 0.0)
                new = D[a, c] + (D[b, d] if d is not None else 0.0)
                if new < old - 1e-9:
                    t[i:j + 1] = t[i:j + 1][::-1]
                    improved = True
    return t


# ── Scan model ───────────────────────────────────────────────────────────────

def p_scan(v):
    v = np.asarray(v, dtype=float)
    return np.where(v <= V_THRESH, 1.0 - np.exp(-K_SCAN * (V_THRESH - v) / V_THRESH), 0.0)


def card_v_star():
    return V_THRESH / (1.0 + K_SCAN)


def true_v_star():
    """argmax of v * P_scan(v): solve exp(k(1-u)) = 1 + k u for u = v / v_thresh."""
    u = brentq(lambda u: np.exp(K_SCAN * (1 - u)) - 1 - K_SCAN * u, 1e-6, 1 - 1e-9)
    return u * V_THRESH


def scan_policy(succ, k_max):
    """succ: (M, N, K) bool attempt outcomes. Returns attempts used and success per face."""
    first = np.argmax(succ[..., :k_max], axis=2)
    ok = np.take_along_axis(succ[..., :k_max], first[..., None], axis=2)[..., 0]
    attempts = np.where(ok, first + 1, k_max)
    return attempts, ok


# ── Simulation ───────────────────────────────────────────────────────────────

def run_simulation():
    occ = build_grid()
    tg, meta = targets()
    nodes = np.vstack([DEPOT, tg])
    assert not any(occ[tuple(np.round(p / RES).astype(int))] for p in nodes), 'waypoint occupied'
    G = build_graph(occ)
    src = [node_id(p) for p in nodes]
    dist, pred = dijkstra(G, directed=False, indices=src, return_predecessors=True)
    D = dist[:, src]
    assert np.all(np.isfinite(D))

    tours = {'Index': list(range(len(nodes))), 'NN': nn_tour(D)}
    tours['NN+2opt'] = two_opt(D, tours['NN'])
    legs = {}
    for name, tour in tours.items():
        lp = []
        for a, b in zip(tour[:-1], tour[1:]):
            path, L = astar(occ, nodes[a], nodes[b])
            assert abs(L - D[a, b]) < 1e-9, 'A* and Dijkstra disagree'
            pts = np.array([node_pos(n) for n in path])
            assert not np.any(occ[tuple(np.round(pts / RES).astype(int).T)]), 'path in obstacle'
            lp.append((pts, L))
        back, Lb = astar(occ, nodes[tour[-1]], nodes[0])
        legs[name] = dict(legs=lp, length=sum(L for _, L in lp), return_len=Lb,
                          return_pts=np.array([node_pos(n) for n in back]))

    # Random scan outcomes, shared by every policy (common random numbers)
    rng = np.random.default_rng(SEED)
    z_v = np.abs(rng.standard_normal((N_MC, len(tg), K_MAX)))   # |N(0,1)| per attempt
    u = rng.random((N_MC, len(tg), K_MAX))
    pos_noise = POS_ERROR * rng.standard_normal((N_MC, len(tg), 3))
    hover = tg[None] + pos_noise
    clearance = dist_to_racks(hover.reshape(-1, 3))
    assert clearance.min() > 0, 'hovering drone touches a rack'

    policy = {}
    for sv in SIGMA_SWEEP:
        succ = u < p_scan(sv * z_v)
        for name, k in [('single', 1), ('retry', K_MAX)]:
            att, ok = scan_policy(succ, k)
            eta = ok.mean(axis=1)
            policy[(sv, name)] = dict(attempts=att, ok=ok, eta=eta,
                                      dwell=att.sum(axis=1) * T_DWELL,
                                      p_attempt=float(succ[..., 0].mean()))
    return dict(occ=occ, tg=tg, meta=meta, nodes=nodes, D=D, tours=tours, legs=legs,
                policy=policy, clearance=clearance)


# ── Plotting ──────────────────────────────────────────────────────────────────

def _draw_racks(ax):
    for lo, hi in rack_boxes():
        ax.bar3d(lo[0], lo[1], lo[2], hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2],
                 color='grey', alpha=0.25, shade=False, edgecolor='0.5', linewidth=0.3)


def _draw_shell(ax):
    X, Y, Z = WH
    for z in (0, Z):
        ax.plot([0, X, X, 0, 0], [0, 0, Y, Y, 0], [z] * 5, color='0.6', lw=0.6)
    for x, y in [(0, 0), (X, 0), (X, Y), (0, Y)]:
        ax.plot([x, x], [y, y], [0, Z], color='0.6', lw=0.6)


def plot_flight_path(res, out_dir, name='NN+2opt'):
    fig = plt.figure(figsize=(16, 7))
    ok = res['policy'][(SIGMA_V, 'retry')]['ok'][0]
    for k, (elev, azim, title) in enumerate([(25, -60, 'Isometric'), (90, -90, 'Top-down')]):
        ax = fig.add_subplot(1, 2, k + 1, projection='3d')
        _draw_shell(ax)
        _draw_racks(ax)
        for pts, _ in res['legs'][name]['legs']:
            ax.plot(*pts.T, color='red', lw=1.0)
        rp = res['legs'][name]['return_pts']
        ax.plot(*rp.T, color='red', lw=0.8, ls=':')
        ax.scatter(*res['tg'].T, color=np.where(ok, 'green', 'red'), s=25, depthshade=False,
                   edgecolor='k', linewidth=0.3)
        ax.scatter(*DEPOT, color='blue', marker='s', s=80, depthshade=False)
        ax.set_xlim(0, WH[0])
        ax.set_ylim(0, WH[1])
        ax.set_zlim(0, WH[2])
        ax.set_box_aspect(tuple(WH))
        ax.view_init(elev=elev, azim=azim)
        ax.set_xlabel('x (m)')
        ax.set_ylabel('y (m)')
        if elev == 90:
            ax.set_zticks([])
        else:
            ax.set_zlabel('z (m)')
        ax.set_title(title)
    L = res['legs'][name]['length']
    fig.suptitle(f'S069 A* route ({name}, {L:.1f} m; dotted = return to depot); green = scanned '
                 f'faces, blue square = depot, grey = racks', fontsize=12)
    plt.tight_layout()
    path = os.path.join(out_dir, 'flight_path_3d.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_scan_success(res, out_dir):
    meta = np.array(res['meta'])
    fig, axes = plt.subplots(1, 2, figsize=(14, 4.8))
    for ax, name in zip(axes, ['single', 'retry']):
        p = res['policy'][(SIGMA_V, name)]
        rates = [p['ok'][:, meta[:, 2] == t].mean() for t in range(3)]
        bars = ax.bar([f'z = {z:.0f} m' for z in TIER_Z], [100 * r for r in rates],
                      color=['green' if r >= ETA_REQ else 'red' for r in rates], edgecolor='k')
        for b, r in zip(bars, rates):
            ax.text(b.get_x() + b.get_width() / 2, 100 * r + 1, f'{100 * r:.1f}%', ha='center')
        ax.axhline(100 * ETA_REQ, color='k', ls='--', lw=1, label='95 % requirement')
        ax.set_ylim(0, 110)
        ax.set_ylabel(f'Scan success over {N_MC} missions (%)')
        lab = 'one 1 s dwell per face (card)' if name == 'single' else \
            f're-dwell until read, max {K_MAX} attempts'
        ax.set_title(f'{lab}\nsigma_v = {SIGMA_V:.1f} m/s (card)')
        ax.legend()
    fig.suptitle('S069 Scan success per shelf tier (green = meets 95 %, red = fails)')
    plt.tight_layout()
    path = os.path.join(out_dir, 'scan_success.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_speed_tradeoff(out_dir):
    v = np.linspace(0, V_THRESH, 400)
    P = p_scan(v)
    rate = v * P / V_THRESH
    fig, ax = plt.subplots(figsize=(9, 5))
    ax.plot(v, P, 'b-', lw=2, label='P_scan(v)')
    ax.plot(v, rate / rate.max(), 'k--', lw=1.5, label='v P_scan(v) (normalised)')
    ax.axvline(true_v_star(), color='green', lw=1.5,
               label=f'true argmax of v P_scan = {true_v_star():.4f} m/s')
    ax.axvline(card_v_star(), color='red', ls=':', lw=1.5,
               label=f'card v* = v_th/(1+k) = {card_v_star():.4f} m/s')
    ax.axhline(ETA_REQ, color='gray', ls='--', lw=1)
    ax.fill_between(v, 0, 1.05, where=P >= ETA_REQ, color='green', alpha=0.2,
                    label='P_scan >= 95 % (only v = 0)')
    ax.text(0.002, ETA_REQ + 0.01, f'95 % (P_scan(0) = {p_scan(0.0):.4f})', fontsize=9)
    ax.set_xlabel('Speed during the scan v (m/s)')
    ax.set_ylabel('Probability / normalised rate')
    ax.set_ylim(0, 1.05)
    ax.set_title('S069 Speed-accuracy trade-off')
    ax.legend(fontsize=8, loc='center left')
    ax.grid(alpha=0.3)
    path = os.path.join(out_dir, 'speed_tradeoff.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_time_breakdown(res, out_dir):
    fig, ax = plt.subplots(figsize=(11, 5))
    labels, tr, dw, err = [], [], [], []
    for name in ['Index', 'NN', 'NN+2opt']:
        for pol in ['single', 'retry']:
            p = res['policy'][(SIGMA_V, pol)]
            labels.append(f'{name}\n{pol}')
            tr.append(res['legs'][name]['length'] / V_CRUISE)
            dw.append(p['dwell'].mean())
            err.append(p['dwell'].std())
    x = np.arange(len(labels))
    ax.bar(x, tr, color='#6baed6', edgecolor='k', label='Transit (A* length / 1 m/s)')
    ax.bar(x, dw, bottom=tr, yerr=err, color='#fd8d3c', edgecolor='k', capsize=4,
           label=f'Dwell (mean ± std over {N_MC} missions)')
    for i in range(len(x)):
        ax.text(x[i], tr[i] + dw[i] + err[i] + 10, f'{tr[i] + dw[i]:.0f} s', ha='center')
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.set_ylabel('Mission time to last face (s)')
    ax.set_title(f'S069 Mission time breakdown (sigma_v = {SIGMA_V:.1f} m/s, card)')
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.15), ncol=2)
    ax.set_ylim(0, max(t + d + e for t, d, e in zip(tr, dw, err)) * 1.12)
    path = os.path.join(out_dir, 'time_breakdown.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_sigma_sweep(res, out_dir):
    fig, axes = plt.subplots(1, 2, figsize=(13, 4.8))
    s = np.array(SIGMA_SWEEP)
    for name, mk in [('single', 'o-'), ('retry', 's-')]:
        eta = [res['policy'][(sv, name)]['eta'].mean() for sv in s]
        dw = [res['policy'][(sv, name)]['dwell'].mean() for sv in s]
        axes[0].plot(s, 100 * np.array(eta), mk, label=name)
        axes[1].plot(s, dw, mk, label=name)
    axes[0].plot(s, [100 * res['policy'][(sv, 'single')]['p_attempt'] for sv in s], 'k:',
                 label='per-attempt success')
    axes[0].axhline(100 * ETA_REQ, color='gray', ls='--')
    axes[0].set_ylabel('Mean scan success (%)')
    axes[1].set_ylabel('Mean total dwell time (s)')
    for ax in axes:
        ax.set_xscale('log')
        ax.set_xlabel('Hover velocity noise sigma_v (m/s)')
        ax.grid(alpha=0.3, which='both')
        ax.legend()
    fig.suptitle(f'S069 Scan policies vs hover velocity noise (card sigma_v = {SIGMA_V:.1f} m/s; '
                 f'retry cap {K_MAX})')
    plt.tight_layout()
    path = os.path.join(out_dir, 'sigma_sweep.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def save_animation(res, out_dir, name='NN+2opt'):
    """Mission 0 of the retry policy at the card noise level."""
    p = res['policy'][(SIGMA_V, 'retry')]
    att, ok = p['attempts'][0], p['ok'][0]
    tour = res['tours'][name]
    # timeline: (t_start, t_end, kind, data)
    events, t = [], 0.0
    for k, (pts, L) in enumerate(res['legs'][name]['legs']):
        events.append((t, t + L / V_CRUISE, 'fly', pts))
        t += L / V_CRUISE
        face = tour[k + 1] - 1
        events.append((t, t + att[face] * T_DWELL, 'dwell', face))
        t += att[face] * T_DWELL
    t_end = t
    n_samples = int(np.ceil(t_end / ANIM_STEP_S)) + 1
    times = np.minimum(np.arange(n_samples) * ANIM_STEP_S, t_end)

    def state(tq):
        done = {}
        pos = DEPOT
        for t0, t1, kind, data in events:
            if kind == 'dwell' and t1 <= tq:
                done[data] = ok[data]
            if t0 <= tq <= t1:
                if kind == 'fly':
                    seg = np.linalg.norm(np.diff(data, axis=0), axis=1)
                    s = (tq - t0) * V_CRUISE
                    cs = np.concatenate([[0], np.cumsum(seg)])
                    i = min(np.searchsorted(cs, s, side='right') - 1, len(seg) - 1)
                    f = 0 if seg[i] == 0 else (s - cs[i]) / seg[i]
                    pos = data[i] + f * (data[i + 1] - data[i])
                else:
                    pos = res['tg'][data]
        return pos, done

    fig = plt.figure(figsize=(9, 6.5))
    ax = fig.add_subplot(111, projection='3d')

    def update(fi):
        ax.cla()
        tq = times[fi]
        pos, done = state(tq)
        _draw_shell(ax)
        _draw_racks(ax)
        cols = ['white'] * len(res['tg'])
        for f, s in done.items():
            cols[f] = 'green' if s else 'red'
        ax.scatter(*res['tg'].T, c=cols, s=22, edgecolor='k', linewidth=0.4, depthshade=False)
        ax.scatter(*DEPOT, color='blue', marker='s', s=50, depthshade=False)
        ax.scatter(*pos, color='red', s=70, depthshade=False, edgecolor='k')
        ax.set_xlim(0, WH[0])
        ax.set_ylim(0, WH[1])
        ax.set_zlim(0, WH[2])
        ax.set_box_aspect(tuple(WH))
        ax.view_init(elev=30, azim=-65)
        n_ok = sum(1 for s in done.values() if s)
        ax.set_title(f'S069 t = {tq:.0f} s / {t_end:.0f} s — faces done {len(done)}/60, '
                     f'scanned {n_ok}', fontsize=10)
        return []

    anim = animation.FuncAnimation(fig, update, frames=n_samples, interval=80, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    anim.save(path, writer='pillow', fps=12, dpi=65)
    plt.close(fig)
    print(f'Saved: {path} ({n_samples} frames, mission 0 = {t_end:.1f} s)')


# ── Main ─────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    res = run_simulation()
    occ = res['occ']
    print('=== S069 Automated Warehouse Inventory ===')
    print(f'Lattice {NX} x {NY} x {NZ} nodes, {occ.sum()} occupied ({100 * occ.mean():.1f} %)')
    print(f'Card hover velocity noise sigma_v = POS_ERROR / DT_CTRL = {SIGMA_V:.2f} m/s; '
          f'P_scan(0) = {float(p_scan(0.0)):.4f}')
    print(f'Speed-accuracy optimum: card v* = {card_v_star():.4f} m/s, '
          f'true argmax v*P_scan(v) = {true_v_star():.4f} m/s; rate ratio card/true = '
          f'{card_v_star() * float(p_scan(card_v_star())) / (true_v_star() * float(p_scan(true_v_star()))):.3f}')
    for name in ['Index', 'NN', 'NN+2opt']:
        L = res['legs'][name]
        legs = [l for _, l in L['legs']]
        print(f'[{name}] path {L["length"]:.2f} m, transit {L["length"] / V_CRUISE:.1f} s, '
              f'mean leg {np.mean(legs):.2f} m ({np.mean(legs) / V_CRUISE:.2f} s), '
              f'longest leg {max(legs):.2f} m, return to depot {L["return_len"]:.2f} m')
    gain = 1 - res['legs']['NN+2opt']['length'] / res['legs']['NN']['length']
    print(f'2-opt shortens the NN route by {100 * gain:.1f} %; NN is '
          f'{100 * (1 - res["legs"]["NN"]["length"] / res["legs"]["Index"]["length"]):.1f} % '
          f'shorter than the index order')
    print(f'Hover clearance to racks over {res["clearance"].size} noisy hovers: '
          f'min {res["clearance"].min():.3f} m (nominal {D_SCAN} m)')
    for sv in SIGMA_SWEEP:
        for pol in ['single', 'retry']:
            p = res['policy'][(sv, pol)]
            t_tr = res['legs']['NN+2opt']['length'] / V_CRUISE
            print(f'sigma_v {sv:.2f} m/s [{pol:6s}] per-attempt {100 * p["p_attempt"]:.2f} %, '
                  f'success {100 * p["eta"].mean():.2f} ± {100 * p["eta"].std():.2f} % '
                  f'(missions >= 95 %: {100 * np.mean(p["eta"] >= ETA_REQ):.1f} %), '
                  f'missed {np.mean(60 - p["ok"].sum(axis=1)):.2f}, attempts/face '
                  f'{p["attempts"].mean():.2f}, dwell {p["dwell"].mean():.1f} s, '
                  f'mission (NN+2opt) {t_tr + p["dwell"].mean():.1f} s')
    p = res['policy'][(SIGMA_V, 'retry')]
    print(f'Mission 0 (retry, card sigma): {int(p["ok"][0].sum())}/60 scanned, '
          f'dwell {p["dwell"][0]:.0f} s')
    meta = np.array(res['meta'])
    for pol in ['single', 'retry']:
        pp = res['policy'][(SIGMA_V, pol)]
        print(f'Per-tier success [{pol}]: ' + ', '.join(
            f'z={TIER_Z[t]:.0f} m {100 * pp["ok"][:, meta[:, 2] == t].mean():.2f} %'
            for t in range(3)))

    out = os.path.normpath(OUTPUT_DIR)
    os.makedirs(out, exist_ok=True)
    plot_flight_path(res, out)
    plot_scan_success(res, out)
    plot_speed_tradeoff(out)
    plot_time_breakdown(res, out)
    plot_sigma_sweep(res, out)
    save_animation(res, out)
