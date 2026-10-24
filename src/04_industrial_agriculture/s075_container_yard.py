"""
S075 Port Container Yard Inventory
==================================
A drone reads the barcode of each of 50 stacked containers (1-3 tiers) in a 100 x 80 m
yard. Scan points sit 0.8 m in front of each container's low-y face; they are visited in
a nearest-neighbour order. Between consecutive scan points the path is planned in 3D with
RRT* (first solution, as in the card; sampling a box around the segment) or with A* on
a 0.5 m grid; both paths are then
shortcut-pruned and corner-rounded with cubic Bezier curves, keeping every scan point on
the path. At each scan point the drone stabilises (GPS error) and dwells 1.5 s.

RRT* is randomised, so the comparison is repeated over several seeds; the same seed drives
the shortcut smoother of both planners and the GPS stabilisation draws (common random
numbers).

Usage:
    conda activate drones
    python src/04_industrial_agriculture/s075_container_yard.py
"""

import os
import heapq
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# ── Yard layout (card) ────────────────────────────────────────────────────────
YARD_X, YARD_Y, YARD_Z = 100.0, 80.0, 10.0
CONTAINER_L, CONTAINER_W, CONTAINER_H = 6.0, 2.5, 2.5
ROW_Y_STARTS = [5.0, 15.0, 27.0, 39.0, 51.0, 63.0]
ROW_N_CONT = [9, 9, 8, 8, 8, 8]
X_SPACING = CONTAINER_L + 4.0
X_ORIGIN = 3.0
LAYOUT_SEED = 42

# ── Drone / planning (card) ───────────────────────────────────────────────────
R_DRONE = 0.25
V_SCAN = 1.0
T_DWELL = 1.5
D_STAB = 0.05
K_STAB = 0.5
GPS_SIGMA = 0.1
Z_CRUISE = 5.0
D_SCAN = 0.8
ETA_RRT = 2.0
N_ITER = 5000
P_GOAL_BIAS = 0.05
GAMMA_RRT = 12.0
GRID_RES = 0.5
N_SHORTCUT = 200
BEZIER_ALPHA = 0.3
THETA_MAX = np.radians(150.0)        # interior angle below which a corner is rounded

# ── Added parameters (see README Deviations) ──────────────────────────────────
SEEDS = [0, 1, 2, 3, 4]
SAMPLE_MARGINS = [8.0, 16.0, None]   # m, RRT* sampling box margin (None = whole yard)
CHECK_STEP = R_DRONE / 2             # m, collision sampling along segments (card delta)
BEZIER_PTS = 8
ANIM_DT = 4.0                        # s of mission per GIF frame

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '04_industrial_agriculture', 's075_container_yard',
)


# ── Layout ────────────────────────────────────────────────────────────────────
def build_containers():
    rng = np.random.default_rng(LAYOUT_SEED)
    boxes = []
    for y0, n in zip(ROW_Y_STARTS, ROW_N_CONT):
        for col in range(n):
            stack = int(rng.integers(1, 4))
            x_lo = X_ORIGIN + col * X_SPACING
            boxes.append([x_lo, y0, 0.0, x_lo + CONTAINER_L, y0 + CONTAINER_W,
                          stack * CONTAINER_H, stack])
    return np.array(boxes)


def scan_waypoints(boxes):
    """Front (low-y) face, 0.8 m out, mid-length, at mid-stack height (capped at 5 m)."""
    return np.stack([(boxes[:, 0] + boxes[:, 3]) / 2, boxes[:, 1] - D_SCAN,
                     np.minimum(boxes[:, 5] / 2, Z_CRUISE)], axis=1)


def nn_order(start, wps):
    remaining = list(range(len(wps)))
    order, cur = [], start
    while remaining:
        d = [np.linalg.norm(wps[i] - cur) for i in remaining]
        b = remaining[int(np.argmin(d))]
        order.append(b)
        cur = wps[b]
        remaining.remove(b)
    return order


class World:
    """Collision checking against containers inflated by the drone radius, the ground
    (z >= r_d) and the workspace bounds. Counts checked points (deterministic effort)."""

    def __init__(self, boxes):
        self.lo = boxes[:, :3] - R_DRONE
        self.hi = boxes[:, 3:6] + R_DRONE
        self.lo[:, 2] = -1.0
        self.wlo = np.array([0.0, 0.0, R_DRONE])
        self.whi = np.array([YARD_X, YARD_Y, YARD_Z - R_DRONE])
        self.n_checked = 0

    def points_free(self, P):
        P = np.atleast_2d(P)
        self.n_checked += len(P)
        inb = np.all((P >= self.wlo) & (P <= self.whi), axis=1)
        hit = np.all((P[:, None, :] > self.lo[None]) & (P[:, None, :] < self.hi[None]), axis=2)
        return inb & ~hit.any(axis=1)

    def seg_free(self, a, b):
        n = max(2, int(np.ceil(np.linalg.norm(b - a) / CHECK_STEP)) + 1)
        t = np.linspace(0, 1, n)[:, None]
        return bool(self.points_free(a + t * (b - a)).all())

    def path_free(self, path):
        return all(self.seg_free(path[i], path[i + 1]) for i in range(len(path) - 1))


# ── RRT* (card: first solution returned) ──────────────────────────────────────
def rrt_star(start, goal, world, rng):
    """Sample first in the start/goal bounding box grown by SAMPLE_MARGINS[0]; if no
    solution after N_ITER iterations, restart with the next (larger) margin."""
    total = 0
    for margin in SAMPLE_MARGINS:
        path, it = _rrt_star(start, goal, world, rng, margin)
        total += it
        if path is not None:
            return path, total
    raise RuntimeError('RRT* found no path')


def _rrt_star(start, goal, world, rng, margin):
    nodes = np.empty((N_ITER + 2, 3))
    parent = np.full(N_ITER + 2, -1)
    cost = np.zeros(N_ITER + 2)
    nodes[0] = start
    n = 1
    if margin is None:
        lo, hi = world.wlo, world.whi
    else:
        lo = np.maximum(np.minimum(start, goal) - margin, world.wlo)
        hi = np.minimum(np.maximum(start, goal) + margin, world.whi)
        lo[2], hi[2] = world.wlo[2], world.whi[2]
    for it in range(1, N_ITER + 1):
        q = goal.copy() if rng.random() < P_GOAL_BIAS else rng.uniform(lo, hi)
        d2 = ((nodes[:n] - q) ** 2).sum(1)
        i_near = int(np.argmin(d2))
        diff = q - nodes[i_near]
        d = np.sqrt(d2[i_near])
        if d < 1e-9:
            continue
        x_new = nodes[i_near] + min(ETA_RRT, d) * diff / d
        if not world.points_free(x_new)[0] or not world.seg_free(nodes[i_near], x_new):
            continue
        r = min(GAMMA_RRT * (np.log(n) / n) ** (1 / 3), ETA_RRT) if n > 1 else ETA_RRT
        dn = np.sqrt(((nodes[:n] - x_new) ** 2).sum(1))
        near = np.where(dn <= r)[0]
        best_p, best_c = i_near, cost[i_near] + np.linalg.norm(x_new - nodes[i_near])
        for j in near[np.argsort(cost[near] + dn[near])]:
            c = cost[j] + dn[j]
            if c >= best_c:
                break
            if world.seg_free(nodes[j], x_new):
                best_p, best_c = j, c
                break
        nodes[n], parent[n], cost[n] = x_new, best_p, best_c
        new = n
        n += 1
        for j in near:
            c = best_c + dn[j]
            if c < cost[j] and world.seg_free(x_new, nodes[j]):
                parent[j], cost[j] = new, c
        if np.linalg.norm(x_new - goal) < ETA_RRT and world.seg_free(x_new, goal):
            nodes[n], parent[n], cost[n] = goal, new, best_c + np.linalg.norm(goal - x_new)
            path, k = [], n
            while k != -1:
                path.append(nodes[k].copy())
                k = parent[k]
            return path[::-1], it
    return None, N_ITER


# ── A* on a 3D grid (26-connected) ────────────────────────────────────────────
class GridPlanner:
    def __init__(self, boxes):
        self.nx, self.ny, self.nz = int(YARD_X / GRID_RES), int(YARD_Y / GRID_RES), int(YARD_Z / GRID_RES)
        occ = np.zeros((self.nx, self.ny, self.nz), dtype=bool)
        inf = int(np.ceil(R_DRONE / GRID_RES))
        for b in boxes:
            occ[max(0, int(b[0] / GRID_RES) - inf):min(self.nx, int(b[3] / GRID_RES) + inf + 1),
                max(0, int(b[1] / GRID_RES) - inf):min(self.ny, int(b[4] / GRID_RES) + inf + 1),
                0:min(self.nz, int(b[5] / GRID_RES) + inf + 1)] = True
        occ[:, :, 0] = True                         # ground cell layer (z < 0.5 m)
        self.occ = occ
        self.deltas = [(dx, dy, dz) for dx in (-1, 0, 1) for dy in (-1, 0, 1)
                       for dz in (-1, 0, 1) if (dx, dy, dz) != (0, 0, 0)]
        self.n_expanded = 0

    def cell(self, p):
        return tuple(int(np.clip(p[i] / GRID_RES, 0, s - 1))
                     for i, s in enumerate((self.nx, self.ny, self.nz)))

    def centre(self, c):
        return (np.array(c) + 0.5) * GRID_RES

    def plan(self, start, goal):
        s, g = self.cell(start), self.cell(goal)
        gx, gy, gz = g
        h = lambda c: GRID_RES * np.sqrt((c[0] - gx) ** 2 + (c[1] - gy) ** 2 + (c[2] - gz) ** 2)
        openh = [(h(s), 0.0, s)]
        came = {s: None}
        gsc = {s: 0.0}
        closed = set()
        while openh:
            f, gc, cur = heapq.heappop(openh)
            if cur in closed:
                continue
            closed.add(cur)
            self.n_expanded += 1
            if cur == g:
                break
            for d in self.deltas:
                nb = (cur[0] + d[0], cur[1] + d[1], cur[2] + d[2])
                if not (0 <= nb[0] < self.nx and 0 <= nb[1] < self.ny and 0 <= nb[2] < self.nz):
                    continue
                if self.occ[nb] and nb != g:
                    continue
                ng = gc + GRID_RES * np.sqrt(d[0] ** 2 + d[1] ** 2 + d[2] ** 2)
                if ng < gsc.get(nb, np.inf):
                    gsc[nb] = ng
                    came[nb] = cur
                    heapq.heappush(openh, (ng + h(nb), ng, nb))
        if g not in closed:
            raise RuntimeError('A* found no path')
        cells, c = [], g
        while c is not None:
            cells.append(c)
            c = came[c]
        cells = cells[::-1]
        pts = [self.centre(c) for c in cells[1:-1]]
        return [start.copy()] + pts + [goal.copy()]


# ── Smoothing (per segment, scan points stay on the path) ─────────────────────
def shortcut(path, world, rng):
    path = [p.copy() for p in path]
    for _ in range(N_SHORTCUT):
        if len(path) < 3:
            break
        i = int(rng.integers(0, len(path) - 2))
        j = int(rng.integers(i + 2, len(path)))
        if world.seg_free(path[i], path[j]):
            path = path[:i + 1] + path[j:]
    return path


def bezier_round(path, world):
    """Round each interior corner whose interior angle is below THETA_MAX with a cubic
    Bezier from P0 = w - alpha (w - w_prev) to P3 = w + alpha (w_next - w). A rounded
    corner is kept only if the curve is collision-free."""
    if len(path) < 3:
        return path
    out = [path[0]]
    for i in range(1, len(path) - 1):
        a, w, b = path[i - 1], path[i], path[i + 1]
        vi, vo = w - a, b - w
        li, lo_ = np.linalg.norm(vi), np.linalg.norm(vo)
        if li < 1e-9 or lo_ < 1e-9:
            out.append(w)
            continue
        turn = np.arccos(np.clip(vi @ vo / (li * lo_), -1, 1))
        if np.pi - turn >= THETA_MAX:
            out.append(w)
            continue
        p0 = w - BEZIER_ALPHA * vi
        p3 = w + BEZIER_ALPHA * vo
        p1 = p0 + 2 / 3 * (w - p0)
        p2 = p3 + 2 / 3 * (w - p3)
        t = np.linspace(0, 1, BEZIER_PTS)[:, None]
        curve = (1 - t) ** 3 * p0 + 3 * (1 - t) ** 2 * t * p1 + 3 * (1 - t) * t ** 2 * p2 + t ** 3 * p3
        full = [out[-1]] + list(curve) + [b]
        if world.path_free(full):
            out.extend(list(curve))
        else:
            out.append(w)
    out.append(path[-1])
    return out


def path_length(path):
    P = np.asarray(path)
    return float(np.linalg.norm(np.diff(P, axis=0), axis=1).sum())


def cta(path):
    P = np.asarray(path)
    v = np.diff(P, axis=0)
    L = np.linalg.norm(v, axis=1)
    v, L = v[L > 1e-9], L[L > 1e-9]
    c = np.einsum('ij,ij->i', v[:-1], v[1:]) / (L[:-1] * L[1:])
    return float(np.degrees(np.arccos(np.clip(c, -1, 1))).sum())


# ── Mission ───────────────────────────────────────────────────────────────────
def plan_mission(planner, seq, world, rng, grid=None):
    full, seg_starts, iters = [seq[0]], [0], []
    raw_len = 0.0
    for a, b in zip(seq[:-1], seq[1:]):
        if planner == 'rrt':
            seg, it = rrt_star(a, b, world, rng)
            iters.append(it)
        else:
            seg = grid.plan(a, b)
        raw_len += path_length(seg)
        seg = shortcut(seg, world, rng)
        seg = bezier_round(seg, world)
        full.extend(seg[1:])
        seg_starts.append(len(full) - 1)
    return full, seg_starts, raw_len, iters


def stabilisation(rng, n):
    eps = np.linalg.norm(rng.normal(0, GPS_SIGMA, (n, 3)), axis=1)
    return eps / (K_STAB * V_SCAN)


def run_simulation():
    boxes = build_containers()
    wps = scan_waypoints(boxes)
    start = np.array([0.0, 0.0, Z_CRUISE])
    order = nn_order(start, wps)
    seq = [start] + [wps[i] for i in order]
    world = World(boxes)
    assert world.points_free(np.array(seq)).all(), 'a scan point is inside an obstacle'
    grid = GridPlanner(boxes)
    res = {'rrt': [], 'astar': []}
    for s in SEEDS:
        for planner in ('rrt', 'astar'):
            rng = np.random.default_rng(s)
            world.n_checked = 0
            grid.n_expanded = 0
            path, idx, raw, iters = plan_mission(planner, seq, world, rng, grid)
            checks = world.n_checked
            P = np.array(path)
            assert world.path_free(path), 'smoothed path collides'
            assert all(np.allclose(P[i], seq[k]) for k, i in enumerate(idx)), 'scan point lost'
            t_stab = stabilisation(np.random.default_rng(1000 + s), len(wps))
            L = path_length(path)
            res[planner].append(dict(path=P, idx=idx, L=L, raw=raw, cta=cta(path),
                                     t_travel=L / V_SCAN, t_stab=t_stab,
                                     t_scan=float((t_stab + T_DWELL).sum()),
                                     t_mission=L / V_SCAN + float((t_stab + T_DWELL).sum()),
                                     checks=checks, expanded=grid.n_expanded,
                                     iters=iters, zmax=P[:, 2].max(), zmin=P[:, 2].min()))
    straight = path_length(seq)
    return dict(boxes=boxes, wps=wps, order=order, seq=np.array(seq), res=res,
                straight=straight)


# ── Plots ─────────────────────────────────────────────────────────────────────
def _box_faces(b):
    x0, y0, z0, x1, y1, z1 = b[:6]
    v = np.array([[x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0],
                  [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1]])
    return [v[[0, 1, 2, 3]], v[[4, 5, 6, 7]], v[[0, 1, 5, 4]], v[[2, 3, 7, 6]],
            v[[1, 2, 6, 5]], v[[0, 3, 7, 4]]]


def plot_yard_3d(sim, out_dir):
    fig = plt.figure(figsize=(13, 9))
    ax = fig.add_subplot(111, projection='3d')
    shades = {1: '0.8', 2: '0.6', 3: '0.4'}
    for b in sim['boxes']:
        ax.add_collection3d(Poly3DCollection(_box_faces(b), facecolor=shades[int(b[6])],
                                             edgecolor='k', linewidths=0.2, alpha=0.6))
    r0, a0 = sim['res']['rrt'][0], sim['res']['astar'][0]
    ax.plot(*r0['path'].T, color='red', lw=1.2, label=f"RRT* ({r0['L']:.1f} m)")
    ax.plot(*a0['path'].T, color='tab:blue', lw=1.0, label=f"A* ({a0['L']:.1f} m)")
    w = sim['wps']
    ax.scatter(*w.T, marker='x', c='green', s=25, label='scan points')
    ax.scatter(*sim['seq'][0], c='k', s=50, label='start')
    ax.plot([0, YARD_X, YARD_X, 0, 0], [0, 0, YARD_Y, YARD_Y, 0], [0] * 5, 'k-', lw=0.8)
    ax.set_xlim(0, YARD_X)
    ax.set_ylim(0, YARD_Y)
    ax.set_zlim(0, YARD_Z)
    ax.set_box_aspect((100, 80, 25))
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_zlabel('z (m)')
    ax.view_init(elev=35, azim=-110)
    ax.set_title(f'S075 container yard: 50 containers (grey shade = 1/2/3 tiers), seed {SEEDS[0]}')
    ax.legend(loc='upper left', fontsize=8)
    path = os.path.join(out_dir, 'yard_3d.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def densify(P, step):
    """Points every `step` metres along the polyline (for colour-by-altitude plots)."""
    out = [P[:1]]
    for a, b in zip(P[:-1], P[1:]):
        n = max(1, int(np.ceil(np.linalg.norm(b - a) / step)))
        out.append(a + np.linspace(0, 1, n + 1)[1:, None] * (b - a))
    return np.vstack(out)


def plot_projections(sim, out_dir):
    fig, axes = plt.subplots(2, 2, figsize=(15, 10), gridspec_kw=dict(height_ratios=[3, 1.3]))
    for j, (key, name) in enumerate([('rrt', 'RRT*'), ('astar', 'A*')]):
        r = sim['res'][key][0]
        P = densify(r['path'], 0.2)
        ax = axes[0, j]
        for b in sim['boxes']:
            ax.add_patch(plt.Rectangle((b[0], b[1]), b[3] - b[0], b[4] - b[1],
                                       color=str(1.0 - 0.2 * b[6]), ec='k', lw=0.3))
        sc = ax.scatter(P[:, 0], P[:, 1], c=P[:, 2], cmap='viridis', s=2, vmin=0, vmax=YARD_Z)
        ax.plot(P[:, 0], P[:, 1], color='k', lw=0.3, alpha=0.5)
        ax.plot(sim['wps'][:, 0], sim['wps'][:, 1], 'x', color='green', ms=5)
        ax.set_xlim(0, YARD_X)
        ax.set_ylim(0, YARD_Y)
        ax.set_aspect('equal')
        ax.set_title(f"{name}: top view, L = {r['L']:.1f} m, CTA = {r['cta']:.0f} deg")
        ax.set_xlabel('x (m)')
        ax.set_ylabel('y (m)')
        plt.colorbar(sc, ax=ax, label='z (m)', shrink=0.8)
        ax = axes[1, j]
        for b in sim['boxes']:
            ax.add_patch(plt.Rectangle((b[0], 0), b[3] - b[0], b[5], color='0.85', ec='0.6', lw=0.3))
        ax.scatter(P[:, 0], P[:, 2], c=P[:, 2], cmap='viridis', s=2, vmin=0, vmax=YARD_Z)
        ax.set_xlim(0, YARD_X)
        ax.set_ylim(0, YARD_Z)
        ax.set_xlabel('x (m)')
        ax.set_ylabel('z (m)')
        ax.set_title(f'{name}: front view (x-z), stacks projected in grey')
    plt.tight_layout()
    path = os.path.join(out_dir, 'path_projections.png')
    plt.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_metrics(sim, out_dir):
    res = sim['res']
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.8))
    for ax, key, lab in [(axes[0], 'L', 'path length (m)'), (axes[1], 'cta', 'cumulative turning angle (deg)'),
                         (axes[2], 't_mission', 'mission time (s)')]:
        vals = [np.array([r[key] for r in res[p]]) for p in ('rrt', 'astar')]
        ax.bar([0, 1], [v.mean() for v in vals], yerr=[v.std() for v in vals], capsize=5,
               color=['red', 'tab:blue'], edgecolor='k')
        for i, v in enumerate(vals):
            ax.scatter(np.full(len(v), i) + np.linspace(-0.15, 0.15, len(v)), v, c='k', s=10, zorder=3)
            ax.text(i, v.mean() * 1.04, f'{v.mean():.1f}', ha='center', fontsize=9)
        ax.set_xticks([0, 1])
        ax.set_xticklabels(['RRT*', 'A*'])
        ax.set_ylabel(lab)
        ax.set_ylim(0, max(v.max() for v in vals) * 1.15)
    ch = np.mean([r['checks'] for r in res['rrt']])
    ex = np.mean([r['expanded'] for r in res['astar']])
    axes[0].set_title(f'Path length ({len(SEEDS)} seeds; dots = seeds)')
    axes[1].set_title('Smoothness (lower = smoother)')
    axes[2].set_title('Mission time incl. 50 x (stabilisation + dwell)')
    fig.suptitle(f'Planning effort: RRT* {ch / 1e6:.2f} M collision-checked points per mission; '
                 f'A* {ex / 1e3:.0f} k node expansions per mission', fontsize=10)
    plt.tight_layout()
    path = os.path.join(out_dir, 'planner_metrics.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def save_animation(sim, out_dir):
    r = sim['res']['rrt'][0]
    P, idx = r['path'], r['idx']
    # time-parameterise: fly at V_SCAN; at scan point k wait t_stab[k] + T_DWELL
    seg = np.linalg.norm(np.diff(P, axis=0), axis=1)
    stops = {i: r['t_stab'][k - 1] + T_DWELL for k, i in enumerate(idx) if k > 0}
    t_arr = np.zeros(len(P))           # arrival time at node i
    t_dep = np.zeros(len(P))           # departure time from node i
    dwell_iv = []
    for i in range(len(P)):
        if i > 0:
            t_arr[i] = t_dep[i - 1] + seg[i - 1] / V_SCAN
        t_dep[i] = t_arr[i] + stops.get(i, 0.0)
        if i in stops:
            dwell_iv.append((t_arr[i], t_dep[i], i))
    T = t_dep[-1]
    n_frames = int(np.ceil(T / ANIM_DT)) + 1
    times = np.minimum(np.arange(n_frames) * ANIM_DT, T)

    def pos_at(t):
        k = int(np.searchsorted(t_dep, t, side='right'))
        if k >= len(P):
            return P[-1]
        if t <= t_arr[k] and k > 0:
            f = (t - t_dep[k - 1]) / max(t_arr[k] - t_dep[k - 1], 1e-9)
            return P[k - 1] + np.clip(f, 0, 1) * (P[k] - P[k - 1])
        return P[k]

    fig, ax = plt.subplots(figsize=(8, 6.6))
    for b in sim['boxes']:
        ax.add_patch(plt.Rectangle((b[0], b[1]), b[3] - b[0], b[4] - b[1],
                                   color=str(1.0 - 0.2 * b[6]), ec='k', lw=0.3))
    ax.plot(P[:, 0], P[:, 1], color='red', lw=0.3, alpha=0.3)
    sp, = ax.plot([], [], 'o', color='green', ms=5)
    flash, = ax.plot([], [], 'o', mfc='none', mec='lime', ms=16, mew=2.5)
    trail, = ax.plot([], [], '-', color='red', lw=1.5, alpha=0.6)
    dot, = ax.plot([], [], 'o', color='red', mec='k', ms=8)
    ax.set_xlim(0, YARD_X)
    ax.set_ylim(0, YARD_Y)
    ax.set_aspect('equal')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    pos = np.array([pos_at(t) for t in times])

    def update(f):
        t = times[f]
        p = pos[f]
        h = pos[max(0, f - 29):f + 1]
        trail.set_data(h[:, 0], h[:, 1])
        dot.set_data([p[0]], [p[1]])
        done = [i for (a, b, i) in dwell_iv if b <= t]
        cur = [i for (a, b, i) in dwell_iv if a <= t < b]
        sp.set_data(P[done, 0] if done else [], P[done, 1] if done else [])
        flash.set_data(P[cur, 0] if cur else [], P[cur, 1] if cur else [])
        ax.set_title(f't = {t:5.0f} s | z = {p[2]:.1f} m | scanned {len(done)}/50', fontsize=10)
        return trail, dot, sp, flash

    anim = animation.FuncAnimation(fig, update, frames=n_frames, interval=100, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    anim.save(path, writer='pillow', fps=10, dpi=70)
    plt.close(fig)
    print(f'Saved: {path} ({n_frames} frames; mission {T:.1f} s)')


if __name__ == '__main__':
    sim = run_simulation()
    res = sim['res']
    b = sim['boxes']
    print('\n=== S075 Port Container Yard Inventory — Results ===')
    print(f'  Containers: {len(b)}; stacks 1/2/3 tiers: '
          f'{int((b[:, 6] == 1).sum())}/{int((b[:, 6] == 2).sum())}/{int((b[:, 6] == 3).sum())}; '
          f'scan points {len(sim["wps"])}')
    gaps = np.diff(sorted(set(ROW_Y_STARTS)))
    print(f'  Aisles between rows (y): ' + ', '.join(f'{g - CONTAINER_W:.1f}' for g in gaps)
          + ' m; gaps between containers (x): ' f'{X_SPACING - CONTAINER_L:.1f} m')
    print(f'  Straight-line tour through the scan points (ignoring obstacles): {sim["straight"]:.1f} m')
    print(f'  Seeds {SEEDS}')
    print(f"  {'Metric':<36}{'RRT* mean ± std':>22}{'A* mean ± std':>22}")
    for key, lab, fmt in [('raw', 'Raw path length before smoothing (m)', '.1f'),
                          ('L', 'Path length (m)', '.1f'), ('cta', 'Cumulative turning angle (deg)', '.0f'),
                          ('t_travel', 'Travel time (s)', '.1f'), ('t_scan', 'Stabilisation + dwell (s)', '.1f'),
                          ('t_mission', 'Total mission time (s)', '.1f'), ('zmax', 'Highest point (m)', '.2f')]:
        a = np.array([r[key] for r in res['rrt']])
        c = np.array([r[key] for r in res['astar']])
        print(f"  {lab:<36}{format(a.mean(), fmt):>12} ± {format(a.std(), fmt):<7}"
              f"{format(c.mean(), fmt):>12} ± {format(c.std(), fmt):<7}")
    print('  Containers scanned: RRT* ' + '/'.join(str(len(r['idx']) - 1) for r in res['rrt'])
          + ', A* ' + '/'.join(str(len(r['idx']) - 1) for r in res['astar']))
    it = np.concatenate([r['iters'] for r in res['rrt']])
    print(f'  RRT* segments needing a larger sampling box: {int((it > N_ITER).sum())} of {len(it)}')
    print(f'  RRT* iterations per segment: mean {it.mean():.0f}, max {it.max()}; '
          f'collision-checked points per mission: mean {np.mean([r["checks"] for r in res["rrt"]]) / 1e6:.2f} M')
    print(f'  A* node expansions per mission: {np.mean([r["expanded"] for r in res["astar"]]) / 1e3:.0f} k; '
          f'collision-checked points (smoothing only): '
          f'{np.mean([r["checks"] for r in res["astar"]]) / 1e6:.2f} M')
    r0, a0 = res['rrt'][0], res['astar'][0]
    print(f"  Seed {SEEDS[0]} (figures, animation): RRT* L {r0['L']:.1f} m, CTA {r0['cta']:.0f} deg, "
          f"mission {r0['t_mission']:.1f} s; A* L {a0['L']:.1f} m, CTA {a0['cta']:.0f} deg, "
          f"mission {a0['t_mission']:.1f} s")
    ts = res['rrt'][0]['t_stab']
    print(f'  Stabilisation per stop (seed {SEEDS[0]}): mean {ts.mean():.2f} s, max {ts.max():.2f} s')
    for key, lab in [('L', 'path length'), ('cta', 'turning angle'), ('t_mission', 'mission time')]:
        a = np.array([r[key] for r in res['rrt']])
        c = np.array([r[key] for r in res['astar']])
        print(f'  Paired {lab}: RRT* lower in {int((a < c).sum())}/{len(SEEDS)} seeds')
    out = os.path.normpath(OUTPUT_DIR)
    os.makedirs(out, exist_ok=True)
    plot_yard_3d(sim, out)
    plot_projections(sim, out)
    plot_metrics(sim, out)
    save_animation(sim, out)
