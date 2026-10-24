"""
S054 Minefield Detection
========================
A drone with a downward ground-penetrating radar (GPR, effective radius 4 m) flies at
3 m over a 200 x 200 m suspected minefield and builds a risk map. Mines are fixed point
targets with unknown positions; the GPR also raises false alarms. Every time the risk map
changes, the least-risk escape path back to the safe-zone entry (100, 0) m is replanned
on the 8-connected 1 m grid with cell cost 1 + 50 R^2.

Three survey strategies (card) are compared on the same mines and the same false-alarm
draws (common random numbers):
  1. Fixed boustrophedon: 25 strips 8 m apart, ignores detections
  2. Risk-aware boustrophedon: same strips, local least-risk detour around danger cells
  3. Adaptive frontier: flies to the nearest unscanned, non-danger cell (least-risk path
     when the straight line crosses risk); after 95 % coverage it revisits unresolved
     alarms once to reclassify them
Every mission stops at 95 % coverage completeness (card) and returns to the entry along
the escape path. A 15-field Monte Carlo (N_mine ~ U{8, 20}) reports mean +- std.

Usage:
    conda activate drones
    python src/03_environmental_sar/s054_minefield.py
"""

import os
import heapq
from concurrent.futures import ProcessPoolExecutor
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.colors import LinearSegmentedColormap
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra

# ── Parameters (card) ─────────────────────────────────────────────────────────
FIELD_SIZE    = 200
CELL_SIZE     = 1.0
N_CELLS       = FIELD_SIZE
R_GPR         = 4.0         # m
R_INFLUENCE   = 6.0         # m
DRONE_SPEED   = 3.0         # m/s
ALTITUDE      = 3.0         # m (fixed)
DT            = 0.2         # s
THETA_ALARM   = 0.50
THETA_CONFIRM = 0.95
P_FA          = 0.05        # false-alarm probability per pass (per 8 x 8 m block, see README)
R_SAFE        = 0.10
R_CAUTION     = 0.40
ALPHA_RISK    = 50.0
STRIP_WIDTH   = 2 * R_GPR
SAFE_ZONE_Y   = 10.0
ENTRY_POINT   = np.array([100.0, 0.0])
C_TARGET      = 0.95
SCAN_PD       = 0.9         # cumulative P_d for a cell to count as scanned
N_MINE_DEFAULT = 15

# ── Parameters (added) ────────────────────────────────────────────────────────
R_GPR_MAX     = R_GPR       # m — GPR returns nothing beyond the footprint radius
FA_BLOCK      = 8.0         # m — false-alarm block size
LOOKAHEAD     = 15.0        # m — risk-aware look-ahead along the strip
REJOIN_MARGIN = 2.0         # m — rejoin the strip this far past the danger stretch
J_MAX_FACTOR  = 3.0         # J_max = 3 x risk-free cost from the farthest corner
EVAL_EVERY    = 5.0         # s — escape-path risk sampling interval
REWEIGHT_EVERY = 10.0       # s — refresh risk weights of active sources
N_MC          = 15
MC_WORKERS    = 4             # worker processes for the Monte Carlo
SEAM_WIDTHS   = [6.8, 6.4]    # m — narrower fixed-strip widths tested against the seams
MAIN_SEED     = 42
ANIM_FRAMES   = 90

STRATS = ['fixed', 'risk', 'frontier']
LABELS = {'fixed': 'Fixed boustrophedon', 'risk': 'Risk-aware boustrophedon',
          'frontier': 'Adaptive frontier'}
COLORS = {'fixed': '#7F7F7F', 'risk': '#1F77B4', 'frontier': '#2CA02C'}

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), '..', '..',
                          'outputs', '03_environmental_sar', 's054_minefield')

XC = (np.arange(N_CELLS) + 0.5) * CELL_SIZE          # cell centres (both axes)
J_MAX = J_MAX_FACTOR * np.hypot(FIELD_SIZE / 2, FIELD_SIZE)   # escape-cost abort threshold


# ── Sensor model ──────────────────────────────────────────────────────────────

def gpr_detect_prob(r):
    """Single-step GPR detection probability at horizontal range r (0 beyond R_GPR_MAX)."""
    r = np.asarray(r, dtype=float)
    p = 1.0 - np.exp(-R_GPR ** 2 / (2.0 * np.maximum(r, 1e-3) ** 2))
    return np.where(r <= R_GPR_MAX, p, 0.0)


def untruncated_pass_pd(lateral):
    """Cumulative P_d of one straight pass at a lateral offset with the card's untruncated model."""
    s = np.arange(-300.0, 300.0, DRONE_SPEED * DT)
    r = np.hypot(lateral, s)
    p = 1.0 - np.exp(-R_GPR ** 2 / (2.0 * r ** 2))
    return 1.0 - np.prod(1.0 - p)


# ── Grid graph for the escape / detour searches ───────────────────────────────

def build_graph_index():
    rows, cols, dist = [], [], []
    idx = np.arange(N_CELLS * N_CELLS).reshape(N_CELLS, N_CELLS)    # idx[row=y, col=x]
    for dr in (-1, 0, 1):
        for dc in (-1, 0, 1):
            if dr == 0 and dc == 0:
                continue
            r0, r1 = max(0, -dr), N_CELLS - max(0, dr)
            c0, c1 = max(0, -dc), N_CELLS - max(0, dc)
            a = idx[r0:r1, c0:c1].ravel()
            b = idx[r0 + dr:r1 + dr, c0 + dc:c1 + dc].ravel()
            rows.append(a)
            cols.append(b)
            dist.append(np.full(a.size, np.hypot(dr, dc)))
    return np.concatenate(rows), np.concatenate(cols), np.concatenate(dist)


G_FROM, G_TO, G_DIST = build_graph_index()


def _pattern(rows, cols):
    g = csr_matrix((np.arange(1, rows.size + 1, dtype=float), (rows, cols)),
                   shape=(N_CELLS ** 2,) * 2)
    return g, (g.data - 1).astype(int)


G_REV, PERM_REV = _pattern(G_TO, G_FROM)       # edge b -> a carries dist * w(b)
G_FWD, PERM_FWD = _pattern(G_FROM, G_TO)       # edge a -> b carries dist * w(b)


def cell_of(pos):
    c = int(np.clip(pos[0] // CELL_SIZE, 0, N_CELLS - 1))
    r = int(np.clip(pos[1] // CELL_SIZE, 0, N_CELLS - 1))
    return r * N_CELLS + c


def cell_xy(k):
    return np.array([XC[k % N_CELLS], XC[k // N_CELLS]])


def node_cost(risk):
    return 1.0 + ALPHA_RISK * risk.ravel() ** 2


def escape_tree(risk):
    """Least-risk cost-to-go to the entry cell for every cell (Dijkstra from the goal on the
    reversed graph); identical optimal costs to A* with an admissible heuristic."""
    w = node_cost(risk)
    # forward move a -> b costs dist * w(b); reversed edge b -> a carries the same weight
    g = G_REV.copy()
    g.data = (G_DIST * w[G_TO])[PERM_REV]
    goal = cell_of(ENTRY_POINT)
    d, pred = dijkstra(g, directed=True, indices=goal, return_predecessors=True)
    return d, pred


def follow_tree(pred, start, goal):
    path = [start]
    k = start
    while k != goal and pred[k] >= 0:
        k = pred[k]
        path.append(k)
    return path


def forward_path(risk, start, goal):
    """Least-risk path start -> goal (forward Dijkstra from start)."""
    w = node_cost(risk)
    g = G_FWD.copy()
    g.data = (G_DIST * w[G_TO])[PERM_FWD]
    d, pred = dijkstra(g, directed=True, indices=start, return_predecessors=True)
    path = [goal]
    k = goal
    while k != start and pred[k] >= 0:
        k = pred[k]
        path.append(k)
    return path[::-1], d[goal]


def astar_cost(risk, start, goal):
    """Plain A* (8-connected, Euclidean heuristic) used once to check the Dijkstra costs."""
    w = node_cost(risk)
    gx, gy = goal % N_CELLS, goal // N_CELLS
    h = lambda k: np.hypot(k % N_CELLS - gx, k // N_CELLS - gy)
    best = {start: 0.0}
    heap = [(h(start), 0.0, start)]
    closed = set()
    while heap:
        f, g, k = heapq.heappop(heap)
        if k in closed:
            continue
        if k == goal:
            return g
        closed.add(k)
        r, c = divmod(k, N_CELLS)
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                if dr == 0 and dc == 0:
                    continue
                rr, cc = r + dr, c + dc
                if 0 <= rr < N_CELLS and 0 <= cc < N_CELLS:
                    nb = rr * N_CELLS + cc
                    g2 = g + np.hypot(dr, dc) * w[nb]
                    if g2 < best.get(nb, np.inf):
                        best[nb] = g2
                        heapq.heappush(heap, (g2 + h(nb), g2, nb))
    return np.inf


# ── Field / world ─────────────────────────────────────────────────────────────

def make_field(seed, n_mines=None):
    rng = np.random.default_rng(seed)
    if n_mines is None:
        n_mines = int(rng.integers(8, 21))
    mines = np.column_stack([rng.uniform(5, FIELD_SIZE - 5, n_mines),
                             rng.uniform(15, FIELD_SIZE - 5, n_mines)])
    nb = int(FIELD_SIZE / FA_BLOCK)
    fa_u = rng.random((nb * nb, 64))                      # per-block, per-pass draws (CRN)
    fa_xy = rng.random((nb * nb, 64, 2)) * FA_BLOCK
    return {'mines': mines, 'fa_u': fa_u, 'fa_xy': fa_xy, 'nb': nb}


class Mission:
    def __init__(self, field, strategy, record=False, strip_width=STRIP_WIDTH):
        self.strip_width = strip_width
        self.f = field
        self.strategy = strategy
        self.mines = field['mines']
        self.cum_pd = np.zeros(len(self.mines))
        self.min_r = np.full(len(self.mines), np.inf)   # closest horizontal approach
        # survey track crossed the mine's row (|dy| <= one step) within one strip width on
        # its west / east side: a missed mine passed on both sides lies between two flown
        # strips (seam); otherwise it lies outside the flown strips (left by the 95 % stop)
        self.pass_w = np.zeros(len(self.mines), dtype=bool)
        self.pass_e = np.zeros(len(self.mines), dtype=bool)
        self.surveying = True
        self.alarmed = np.zeros(len(self.mines), dtype=bool)
        self.confirmed = np.zeros(len(self.mines), dtype=bool)
        self.cov = np.zeros((N_CELLS, N_CELLS))            # [y, x]
        self.n_scanned = 0
        nb = field['nb']
        bc = (np.arange(nb) + 0.5) * FA_BLOCK
        self.block_xy = np.array([(x, y) for y in bc for x in bc])
        self.in_block = np.zeros(nb * nb, dtype=bool)
        self.block_pass = np.zeros(nb * nb, dtype=int)
        self.fa_xy = np.empty((0, 2))
        self.fa_cum = np.empty(0)
        self.fa_active = np.empty(0, dtype=bool)
        self.fa_left = np.empty(0, dtype=bool)
        self.risk = np.zeros((N_CELLS, N_CELLS))
        self.esc_d, self.esc_pred = escape_tree(self.risk)
        self.n_risk_updates = 0
        self.n_tree_builds = 0
        self.n_detours = 0
        self.n_aborts = 0
        self.t = 0.0
        self.pos = ENTRY_POINT.copy()
        self.dist = 0.0
        self.danger_dist = 0.0
        self.esc_log = []    # (t, mean R, max R, cost ratio)
        self.cov_log = []
        self.event_t = []
        self.record = record
        self.traj = [self.pos.copy()]
        self.frames = []
        self.last_reweight = 0.0
        self.weights_at_update = {}
        self.ev_at_frame = 0
        self.abort = False
        self.tree_dirty = False

    # -- sensing ------------------------------------------------------------
    def sense(self):
        px, py = self.pos
        changed = False
        i0, i1 = max(0, int(px - R_GPR_MAX) - 1), min(N_CELLS, int(px + R_GPR_MAX) + 2)
        j0, j1 = max(0, int(py - R_GPR_MAX) - 1), min(N_CELLS, int(py + R_GPR_MAX) + 2)
        r = np.hypot(XC[i0:i1][None, :] - px, XC[j0:j1][:, None] - py)
        win = self.cov[j0:j1, i0:i1]
        before = win > SCAN_PD
        win[:] = 1.0 - (1.0 - win) * (1.0 - gpr_detect_prob(r))
        self.n_scanned += int(np.sum((win > SCAN_PD) & ~before))
        # real mines
        rm = np.hypot(self.mines[:, 0] - px, self.mines[:, 1] - py)
        np.minimum(self.min_r, rm, out=self.min_r)
        if self.surveying:
            dx = self.mines[:, 0] - px
            row = np.abs(self.mines[:, 1] - py) <= DRONE_SPEED * DT + 1e-9
            self.pass_w |= row & (dx >= 0) & (dx <= self.strip_width)
            self.pass_e |= row & (dx <= 0) & (-dx <= self.strip_width)
        pd = gpr_detect_prob(rm)
        self.cum_pd = 1.0 - (1.0 - self.cum_pd) * (1.0 - pd)
        new_al = (~self.alarmed) & (pd > THETA_ALARM)
        new_cf = (~self.confirmed) & (self.cum_pd > THETA_CONFIRM)
        if new_al.any() or new_cf.any():
            changed = True
        self.alarmed |= new_al
        self.confirmed |= new_cf
        # false alarms: one Bernoulli(P_FA) draw each time the footprint enters a block centre
        nb = self.f['nb']
        bi = np.clip(np.arange(int(px // FA_BLOCK) - 1, int(px // FA_BLOCK) + 2), 0, nb - 1)
        bj = np.clip(np.arange(int(py // FA_BLOCK) - 1, int(py // FA_BLOCK) + 2), 0, nb - 1)
        near = np.unique((bj[:, None] * nb + bi[None, :]).ravel())
        db = np.hypot(self.block_xy[near, 0] - px, self.block_xy[near, 1] - py)
        inside = np.zeros(nb * nb, dtype=bool)
        inside[near] = db <= R_GPR_MAX
        entering = np.where(inside & ~self.in_block)[0]
        for b in entering:
            k = self.block_pass[b]
            self.block_pass[b] += 1
            if self.f['fa_u'][b, k % 64] < P_FA:
                xy = self.block_xy[b] - FA_BLOCK / 2 + self.f['fa_xy'][b, k % 64]
                self.fa_xy = np.vstack([self.fa_xy, xy])
                self.fa_cum = np.append(self.fa_cum, 0.0)
                self.fa_active = np.append(self.fa_active, True)
                self.fa_left = np.append(self.fa_left, False)
                changed = True
        self.in_block = inside
        # false-alarm bookkeeping: cumulative P_d (for the risk weight) and reclassification
        if self.fa_active.any():
            d = np.hypot(self.fa_xy[:, 0] - px, self.fa_xy[:, 1] - py)
            p = gpr_detect_prob(d)
            act = self.fa_active
            self.fa_cum[act] = 1.0 - (1.0 - self.fa_cum[act]) * (1.0 - p[act])
            self.fa_left |= act & (d > R_GPR_MAX)
            clear = act & self.fa_left & (p > THETA_ALARM)   # second overpass: no return
            if clear.any():
                self.fa_active[clear] = False
                changed = True
        if not changed and self.t - self.last_reweight >= REWEIGHT_EVERY:
            w = self.source_weights()
            if any(abs(w.get(k, 0) - v) > 0.01 for k, v in w.items()) or \
                    len(w) != len(self.weights_at_update):
                changed = True
            self.last_reweight = self.t
        if changed:
            self.update_risk()
        return changed

    def source_weights(self):
        w = {('m', k): float(self.cum_pd[k]) for k in np.where(self.alarmed | self.confirmed)[0]}
        for i in np.where(self.fa_active)[0]:
            w[('f', int(i))] = float(self.fa_cum[i])
        return w

    def source_xy(self, key):
        return self.mines[key[1]] if key[0] == 'm' else self.fa_xy[key[1]]

    def update_risk(self):
        w = self.source_weights()
        R = np.zeros((N_CELLS, N_CELLS))
        span = int(3 * R_INFLUENCE) + 1
        for key, wk in w.items():
            mx, my = self.source_xy(key)
            i0, i1 = max(0, int(mx) - span), min(N_CELLS, int(mx) + span + 1)
            j0, j1 = max(0, int(my) - span), min(N_CELLS, int(my) + span + 1)
            d2 = (XC[i0:i1][None, :] - mx) ** 2 + (XC[j0:j1][:, None] - my) ** 2
            R[j0:j1, i0:i1] += wk * np.exp(-d2 / (2 * R_INFLUENCE ** 2))
        self.risk = np.clip(R, 0.0, 1.0)
        self.tree_dirty = True
        self.weights_at_update = w
        self.n_risk_updates += 1
        self.event_t.append(self.t)

    # -- motion -------------------------------------------------------------
    def ensure_tree(self):
        if self.tree_dirty:
            self.esc_d, self.esc_pred = escape_tree(self.risk)
            self.n_tree_builds += 1
            self.tree_dirty = False

    def risk_at(self, p):
        return self.risk[cell_of(p) // N_CELLS, cell_of(p) % N_CELLS]

    def risk_along(self, pts):
        c = np.clip((pts[:, 0] // CELL_SIZE).astype(int), 0, N_CELLS - 1)
        r = np.clip((pts[:, 1] // CELL_SIZE).astype(int), 0, N_CELLS - 1)
        return self.risk[r, c]

    def step_towards(self, target):
        d = target - self.pos
        dist = np.linalg.norm(d)
        stp = DRONE_SPEED * DT
        if dist <= stp:
            new = target.astype(float).copy()
            arrived = True
        else:
            new = self.pos + d / dist * stp
            arrived = False
        moved = np.linalg.norm(new - self.pos)
        assert moved <= stp + 1e-9, 'speed limit exceeded'
        assert -1e-9 <= new[0] <= FIELD_SIZE + 1e-9 and -1e-9 <= new[1] <= FIELD_SIZE + 1e-9
        if self.risk_at(self.pos) >= R_CAUTION:
            self.danger_dist += moved
        self.pos = new
        self.dist += moved
        self.t += DT
        self.sense()
        self.log()
        return arrived

    def log(self):
        if self.record:
            self.traj.append(self.pos.copy())
        if len(self.esc_log) == 0 or self.t - self.esc_log[-1][0] >= EVAL_EVERY - 1e-9:
            self.ensure_tree()
            start = cell_of(self.pos)
            path = follow_tree(self.esc_pred, start, cell_of(ENTRY_POINT))
            rv = self.risk.ravel()[path]
            ratio = self.esc_d[start] / J_MAX
            if ratio >= 1.0:
                self.n_aborts += 1
                self.abort = True
            self.esc_log.append((self.t, float(rv.mean()), float(rv.max()), float(ratio)))
            self.cov_log.append((self.t, self.n_scanned / N_CELLS ** 2))
            if self.record:
                new_events = len(self.event_t) != self.ev_at_frame or not self.frames
                self.ev_at_frame = len(self.event_t)
                self.frames.append((self.t, self.pos.copy(), len(self.traj) - 1,
                                    np.array([cell_xy(k) for k in path]),
                                    self.risk.copy() if new_events else None))

    def coverage(self):
        return self.n_scanned / N_CELLS ** 2

    def fly_cells(self, cells, stop_at_target=True):
        for k in cells[1:]:
            tgt = cell_xy(k)
            while not self.step_towards(tgt):
                if stop_at_target and (self.coverage() >= C_TARGET or self.abort):
                    return

    def return_home(self):
        self.ensure_tree()
        path = follow_tree(self.esc_pred, cell_of(self.pos), cell_of(ENTRY_POINT))
        for k in path[1:]:
            tgt = cell_xy(k)
            while not self.step_towards(tgt):
                pass
        while not self.step_towards(ENTRY_POINT):
            pass

    # -- strategies ---------------------------------------------------------
    def waypoints(self):
        wps = []
        w = self.strip_width
        n_strips = int(np.ceil(FIELD_SIZE / w))
        for n in range(n_strips):
            x = min(w / 2 + n * w, FIELD_SIZE - w / 2)
            ys = (0.0, FIELD_SIZE) if n % 2 == 0 else (FIELD_SIZE, 0.0)
            wps += [np.array([x, ys[0]]), np.array([x, ys[1]])]
        return wps

    def run_boustrophedon(self, risk_aware):
        wps = self.waypoints()
        i = 0
        while i < len(wps) and self.coverage() < C_TARGET and not self.abort:
            tgt = wps[i]
            if risk_aware and self.danger_ahead(tgt):
                # a strip end inside a danger zone is pulled back along its strip to the
                # first safe point (the strip is shortened, not skipped)
                if self.risk_at(tgt) >= R_CAUTION:
                    other = wps[i + 1] if i % 2 == 0 else wps[i - 1]
                    u = (other - tgt) / np.linalg.norm(other - tgt)
                    s = np.arange(1.0, FIELD_SIZE, 1.0)
                    pts = tgt[None, :] + s[:, None] * u[None, :]
                    ok = np.where(self.risk_along(pts) < R_CAUTION)[0]
                    if len(ok):
                        wps[i] = pts[ok[0]]
                    tgt = wps[i]
                rejoin = self.rejoin_point(tgt)
                path, _ = forward_path(self.risk, cell_of(self.pos), cell_of(rejoin))
                self.n_detours += 1
                if len(path) > 1:
                    self.fly_cells(path)
                elif self.step_towards(tgt):
                    i += 1
                continue
            if self.step_towards(tgt):
                i += 1

    def danger_ahead(self, tgt):
        d = tgt - self.pos
        L = np.linalg.norm(d)
        if L < 1e-9:
            return False
        s = np.arange(0.0, min(L, LOOKAHEAD), 1.0)
        pts = self.pos[None, :] + s[:, None] * d[None, :] / L
        return bool(np.any(self.risk_along(pts) >= R_CAUTION))

    def rejoin_point(self, tgt):
        d = tgt - self.pos
        L = np.linalg.norm(d)
        s = np.arange(0.0, L, 1.0)
        pts = self.pos[None, :] + s[:, None] * d[None, :] / max(L, 1e-9)
        bad = np.where(self.risk_along(pts) >= R_CAUTION)[0]
        if not len(bad):
            return tgt
        k = bad[-1] + int(REJOIN_MARGIN) + 1
        return pts[k] if k < len(pts) else tgt

    def run_frontier(self):
        xs, ys = np.meshgrid(XC, XC)
        while self.coverage() < C_TARGET and not self.abort:
            cand = (self.cov <= SCAN_PD) & (self.risk < R_CAUTION)
            if not cand.any():
                break
            # sweep-line ordering: only the westernmost 8 m band that still has targets
            col0 = int(np.argmax(cand.any(axis=0)))
            band = int(XC[col0] // STRIP_WIDTH)
            in_band = (XC // STRIP_WIDTH).astype(int) == band
            cand &= in_band[None, :]
            d2 = (xs - self.pos[0]) ** 2 + (ys - self.pos[1]) ** 2
            d2 = np.where(cand, d2, np.inf)
            # prefer frontier cells outside the current footprint (they are not about to be
            # scanned anyway), so the drone keeps sweeping instead of crawling sideways
            far = np.where(d2 >= R_GPR ** 2, d2, np.inf)
            if np.isfinite(far).any():
                d2 = far
            j, i = np.unravel_index(int(np.argmin(d2)), d2.shape)
            xc = (band + 0.5) * STRIP_WIDTH
            ic = int(xc // CELL_SIZE)
            goals = [np.array([XC[i], XC[j]])]
            if ic != i and self.risk[j, ic] < R_CAUTION:
                # fly the band centreline first so the 8 m footprint spans the band;
                # go sideways to the cell only if it is still unscanned on arrival
                goals.insert(0, np.array([XC[ic], XC[j]]))
            t0 = self.t
            for tgt in goals:
                if self.cov[j, i] > SCAN_PD or self.coverage() >= C_TARGET or self.abort:
                    break
                if self.line_has_risk(tgt):
                    path, _ = forward_path(self.risk, cell_of(self.pos), cell_of(tgt))
                    self.n_detours += 1
                    for k in path[1:]:
                        c = cell_xy(k)
                        while not self.step_towards(c):
                            pass
                        if self.cov[j, i] > SCAN_PD or self.coverage() >= C_TARGET:
                            break
                while self.cov[j, i] <= SCAN_PD and self.coverage() < C_TARGET:
                    if self.step_towards(tgt):
                        break
            if self.t == t0:            # guarantee progress (hover one step over the target)
                self.step_towards(goals[-1])
        # reclassification visits: unresolved false alarms and alarmed-but-unconfirmed mines
        targets = [xy for xy in self.fa_xy[self.fa_active]]
        targets += [self.mines[k] for k in np.where(self.alarmed & ~self.confirmed)[0]]
        if self.abort:
            targets = []
        self.n_reclass_visits = len(targets)
        for tgt in sorted(targets, key=lambda p: (p[0], p[1])):
            path, _ = forward_path(self.risk, cell_of(self.pos), cell_of(tgt))
            self.fly_cells(path, stop_at_target=False)
            while not self.step_towards(np.asarray(tgt, dtype=float)):
                pass
            # leave the footprint so a later overpass counts as a second pass
        return

    def line_has_risk(self, tgt):
        d = tgt - self.pos
        L = np.linalg.norm(d)
        s = np.arange(0.0, L + 1e-9, 1.0)
        pts = self.pos[None, :] + s[:, None] * d[None, :] / max(L, 1e-9)
        return bool(np.any(self.risk_along(pts) >= R_CAUTION))

    def run(self):
        self.sense()
        self.log()
        self.n_reclass_visits = 0
        if self.strategy == 'fixed':
            self.run_boustrophedon(False)
        elif self.strategy == 'risk':
            self.run_boustrophedon(True)
        else:
            self.run_frontier()
        self.t_survey = self.t
        self.cov_survey = self.coverage()
        self.surveying = False
        self.return_home()
        el = np.array(self.esc_log)
        detected = self.alarmed | self.confirmed
        between = ~detected & self.pass_w & self.pass_e
        return {
            'time': self.t, 't_survey': self.t_survey, 'dist': self.dist,
            'coverage': self.cov_survey, 'n_mines': len(self.mines),
            'detected': int(detected.sum()), 'confirmed': int(self.confirmed.sum()),
            'missed': int((~detected).sum()),
            'miss_seam': int(between.sum()),
            'miss_outside': int(np.sum(~detected & ~between)),
            'miss_seam_r': [float(v) for v in self.min_r[between]],
            'miss_outside_r': [float(v) for v in self.min_r[~detected & ~between]],
            'fa_raised': len(self.fa_xy), 'fa_unresolved': int(self.fa_active.sum()),
            'esc_mean': float(el[:, 1].mean()), 'esc_mean_max': float(el[:, 1].max()),
            'esc_max_max': float(el[:, 2].max()),
            'frac_esc_unsafe': float(np.mean(el[:, 1] >= R_SAFE)),
            'frac_esc_danger_cell': float(np.mean(el[:, 2] >= R_CAUTION)),
            'danger_dist': self.danger_dist, 'risk_updates': self.n_risk_updates,
            'detours': self.n_detours, 'aborts': self.n_aborts,
            'reclass_visits': self.n_reclass_visits,
        }


def run_simulation():
    field = make_field(MAIN_SEED, n_mines=N_MINE_DEFAULT)
    main = {}
    missions = {}
    for s in STRATS:
        m = Mission(field, s, record=True)
        main[s] = m.run()
        missions[s] = m
    # check: Dijkstra escape cost equals A* cost on the final risk map of the risk-aware run
    mr = missions['risk']
    start = cell_of(np.array([150.0, 150.0]))
    d_dij = float(mr.esc_d[start])
    d_ast = float(astar_cost(mr.risk, start, cell_of(ENTRY_POINT)))
    # Monte Carlo fields run in worker processes; results are collected in seed order, so the
    # output does not depend on the number of workers (each field is fully seeded)
    mc = {s: [] for s in STRATS}
    with ProcessPoolExecutor(max_workers=MC_WORKERS) as ex:
        for res in ex.map(mc_field, range(N_MC)):
            for key, r in res:
                mc.setdefault(key, []).append(r)
    return field, main, missions, mc, (d_dij, d_ast)


def mc_field(seed):
    """All Monte Carlo missions on one field (same field and false-alarm draws for each)."""
    fld = make_field(seed)
    out = [(s, Mission(fld, s).run()) for s in STRATS]
    out += [(('fixed', w), Mission(fld, 'fixed', strip_width=w).run()) for w in SEAM_WIDTHS]
    return out


# ── Plotting ──────────────────────────────────────────────────────────────────

RISK_CMAP = LinearSegmentedColormap.from_list('risk', ['#FFFFFF', '#FFE08A', '#F28E2B', '#B2182B'])


def _save(fig, out_dir, name):
    path = os.path.join(out_dir, name)
    fig.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def _mines(ax, m):
    det = m.alarmed | m.confirmed
    ax.plot(*m.mines[m.confirmed].T, 'o', mfc='none', mec='red', ms=9, mew=2, label='confirmed mine')
    al = m.alarmed & ~m.confirmed
    if al.any():
        ax.plot(*m.mines[al].T, '^', color='orange', ms=8, label='alarmed (unconfirmed)')
    if (~det).any():
        ax.plot(*m.mines[~det].T, 'x', color='k', ms=8, mew=2, label='missed mine')
    fa = m.fa_xy
    act = m.fa_active
    if act.any():
        ax.plot(*fa[act].T, '+', color='gold', ms=9, mew=2, label='false alarm (unresolved)')
    if (~act).any() and len(fa):
        ax.plot(*fa[~act].T, '+', color='grey', ms=7, mew=1, label='false alarm (cleared)')


def plot_risk_maps(missions, out_dir):
    fig, axes = plt.subplots(1, 3, figsize=(19, 6.6))
    for ax, s in zip(axes, STRATS):
        m = missions[s]
        im = ax.imshow(m.risk, origin='lower', extent=[0, FIELD_SIZE, 0, FIELD_SIZE], cmap=RISK_CMAP,
                       vmin=0, vmax=1)
        tr = np.array(m.traj)
        ax.plot(tr[:, 0], tr[:, 1], '-', color='tab:blue', lw=0.5, alpha=0.8, label='trajectory')
        _mines(ax, m)
        ax.axhline(SAFE_ZONE_Y, color='green', ls='--', lw=1.2, label='safe zone boundary')
        ax.plot(*ENTRY_POINT, 's', color='green', ms=9)
        ax.set_xlim(0, FIELD_SIZE)
        ax.set_ylim(0, FIELD_SIZE)
        ax.set_aspect('equal')
        ax.set_title(f'{LABELS[s]}\nfinal risk map, t = {m.t:.0f} s', fontsize=10)
        ax.set_xlabel('x (m)')
    axes[0].set_ylabel('y (m)')
    axes[0].legend(fontsize=7, loc='upper left')
    fig.colorbar(im, ax=axes, shrink=0.8, label='risk R')
    fig.suptitle('S054 Risk maps at mission end (seed 42, 15 mines)', fontsize=12)
    _save(fig, out_dir, 'risk_maps.png')


def plot_coverage_map(missions, out_dir):
    fig, axes = plt.subplots(1, 3, figsize=(19, 6.6))
    cm = LinearSegmentedColormap.from_list('cov', ['#FFFFFF', '#1B7837'])
    for ax, s in zip(axes, STRATS):
        m = missions[s]
        im = ax.imshow(m.cov, origin='lower', extent=[0, FIELD_SIZE, 0, FIELD_SIZE], cmap=cm,
                       vmin=0, vmax=SCAN_PD)
        un = m.cov <= SCAN_PD
        ax.imshow(np.ma.masked_where(~un, un), origin='lower', extent=[0, FIELD_SIZE, 0, FIELD_SIZE],
                  cmap=LinearSegmentedColormap.from_list('r', ['red', 'red']), alpha=0.7)
        ax.plot(*m.mines.T, 'x', color='k', ms=6)
        ax.set_aspect('equal')
        ax.set_title(f'{LABELS[s]}\nC = {100 * m.cov_survey:.2f}% (red = P_d^cum <= 0.9)', fontsize=10)
        ax.set_xlabel('x (m)')
    axes[0].set_ylabel('y (m)')
    fig.colorbar(im, ax=axes, shrink=0.8, label='cumulative detection probability P_d^cum (saturated at 0.9)')
    fig.suptitle('S054 Coverage completeness maps (black x = mine)', fontsize=12)
    _save(fig, out_dir, 'coverage_map.png')


def plot_escape_path(missions, out_dir):
    m = missions['risk']
    fig, ax = plt.subplots(figsize=(7.5, 7))
    ax.imshow(m.risk, origin='lower', extent=[0, FIELD_SIZE, 0, FIELD_SIZE], cmap=RISK_CMAP, vmin=0, vmax=1)
    _mines(ax, m)
    # escape path from the position where the survey ends (95 % coverage or end of pattern)
    t_s = m.t_survey
    k = int(round(t_s / DT))
    p0 = m.traj[min(k, len(m.traj) - 1)]
    path = follow_tree(m.esc_pred, cell_of(p0), cell_of(ENTRY_POINT))
    xy = np.array([cell_xy(c) for c in path])
    rv = m.risk.ravel()[path]
    sc = ax.scatter(xy[:, 0], xy[:, 1], c=rv, cmap='viridis', s=6, vmin=0, vmax=max(0.05, rv.max()),
                    zorder=4)
    fig.colorbar(sc, ax=ax, shrink=0.8, label='risk along escape path')
    ax.plot(*p0, 'o', color='tab:blue', ms=9, mec='k', label='position when the survey ends')
    ax.plot(*ENTRY_POINT, 's', color='green', ms=9, label='entry / exit')
    ax.axhline(SAFE_ZONE_Y, color='green', ls='--', lw=1.2)
    ax.set_xlim(0, FIELD_SIZE)
    ax.set_ylim(0, FIELD_SIZE)
    ax.set_aspect('equal')
    ax.legend(fontsize=7, loc='upper left')
    ax.set_title(f'S054 Escape path (risk-aware run): {len(path)} cells, '
                 f'mean R {rv.mean():.3f}, max R {rv.max():.3f}', fontsize=10)
    _save(fig, out_dir, 'escape_path.png')


def plot_time_series(missions, out_dir):
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7.5), sharex=True)
    for s in STRATS:
        m = missions[s]
        cl = np.array(m.cov_log)
        ax1.plot(cl[:, 0], 100 * cl[:, 1], color=COLORS[s], label=LABELS[s])
        el = np.array(m.esc_log)
        ax2.plot(el[:, 0], el[:, 1], color=COLORS[s], lw=1, label=f'{LABELS[s]} (mean R on path)')
        ax2.plot(el[:, 0], el[:, 2], color=COLORS[s], lw=0.6, ls=':', label=f'{LABELS[s]} (max R on path)')
    for t in missions['frontier'].event_t:
        ax1.plot([t, t], [0, 4], color=COLORS['frontier'], lw=0.6)
    ax1.axhline(100 * C_TARGET, color='k', ls='--', lw=1, label='95 % target')
    ax1.set_ylabel('coverage completeness C (%)')
    ax1.legend(fontsize=8)
    ax1.grid(alpha=0.3)
    ax1.set_title('Coverage vs time (green ticks at bottom = frontier risk-map updates)', fontsize=10)
    ax2.axhline(R_SAFE, color='red', ls='--', lw=1, label='R_safe = 0.1')
    ax2.axhline(R_CAUTION, color='darkred', ls='--', lw=1, label='R_caution = 0.4')
    ax2.set_ylabel('risk on escape path')
    ax2.set_xlabel('time (s)')
    ax2.legend(fontsize=6, ncol=2)
    ax2.grid(alpha=0.3)
    ax2.set_title('Escape-path risk (sampled every 5 s)', fontsize=10)
    fig.suptitle('S054 Coverage and escape-path risk vs time (seed 42)')
    plt.tight_layout()
    _save(fig, out_dir, 'time_series.png')


def plot_detection_summary(missions, out_dir):
    m = missions['risk']
    fig, ax = plt.subplots(figsize=(9, 4.2))
    order = np.argsort(m.mines[:, 0])
    cols = ['red' if m.confirmed[k] else ('orange' if m.alarmed[k] else 'black') for k in order]
    ax.bar(np.arange(len(order)), m.cum_pd[order], color=cols, edgecolor='k')
    ax.axhline(THETA_CONFIRM, color='red', ls='--', lw=1, label='confirm 0.95')
    ax.set_xticks(np.arange(len(order)))
    ax.set_xticklabels([f'({m.mines[k, 0]:.0f},{m.mines[k, 1]:.0f})' for k in order], rotation=60, fontsize=7)
    ax.set_ylabel('P_d^cum at mission end')
    ax.set_ylim(0, 1.1)
    ax.legend(fontsize=8)
    ax.set_title(f'S054 Per-mine detection (risk-aware run): red = confirmed, orange = alarmed, '
                 f'black = missed; false alarms raised {len(m.fa_xy)}, '
                 f'unresolved {int(m.fa_active.sum())}', fontsize=9)
    _save(fig, out_dir, 'detection_summary.png')


def plot_mc(mc, out_dir):
    keys = [('time', 'mission time (s)'), ('danger_dist', 'distance over danger cells (m)'),
            ('missed', 'mines missed'), ('fa_unresolved', 'unresolved false alarms')]
    fig, axes = plt.subplots(1, 4, figsize=(18, 4.5))
    for ax, (k, lab) in zip(axes, keys):
        data = [[r[k] for r in mc[s]] for s in STRATS]
        bp = ax.boxplot(data, tick_labels=[LABELS[s].replace(' ', '\n', 1) for s in STRATS],
                        patch_artist=True)
        for patch, s in zip(bp['boxes'], STRATS):
            patch.set_facecolor(COLORS[s])
        ax.set_title(lab, fontsize=10)
        ax.tick_params(axis='x', labelsize=8)
    fig.suptitle(f'S054 Monte Carlo over {N_MC} fields (N_mine ~ U{{8, 20}}, same field and '
                 'false-alarm draws for every strategy)')
    plt.tight_layout()
    _save(fig, out_dir, 'mc_comparison.png')


def plot_risk_3d(missions, out_dir):
    m = missions['risk']
    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(111, projection='3d', computed_zorder=False)
    X, Y = np.meshgrid(XC[::2], XC[::2])
    ax.plot_surface(X, Y, m.risk[::2, ::2], cmap=RISK_CMAP, vmin=0, vmax=1, linewidth=0, alpha=0.9,
                    zorder=0)
    tr = np.array(m.traj)[::5]
    ax.plot(tr[:, 0], tr[:, 1], np.full(len(tr), ALTITUDE), color='tab:blue', lw=0.5, zorder=3,
            label=f'drone at z = {ALTITUDE:.0f} m')
    ax.scatter(m.mines[:, 0], m.mines[:, 1], np.zeros(len(m.mines)), color='red', s=25, zorder=4,
               label='mines')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_zlabel('risk R / altitude (m)')
    ax.set_zlim(0, 3.5)
    ax.view_init(elev=35, azim=-65)
    ax.legend(fontsize=8)
    ax.set_title('S054 Risk surface and risk-aware trajectory (3D)')
    _save(fig, out_dir, 'risk_3d.png')


def save_animation(missions, out_dir):
    m = missions['risk']
    tr = np.array(m.traj)
    frames = m.frames
    n = len(frames)
    step = int(np.ceil((n - 1) / ANIM_FRAMES))
    idx = [min(i * step, n - 1) for i in range(int(np.ceil((n - 1) / step)) + 1)]
    # risk map at each frame: last stored snapshot at or before the frame
    snaps = []
    last = np.zeros((N_CELLS, N_CELLS))
    for f in frames:
        if f[4] is not None:
            last = f[4]
        snaps.append(last)
    fig, ax = plt.subplots(figsize=(5.6, 5.6))
    im = ax.imshow(snaps[0], origin='lower', extent=[0, FIELD_SIZE, 0, FIELD_SIZE], cmap=RISK_CMAP,
                   vmin=0, vmax=1)
    trail, = ax.plot([], [], '-', color='tab:blue', lw=0.5)
    esc, = ax.plot([], [], '-', color='k', lw=1.2)
    dot, = ax.plot([], [], 'o', color='tab:blue', ms=7, mec='k')
    conf, = ax.plot([], [], 'o', mfc='none', mec='red', ms=9, mew=2)
    ax.axhline(SAFE_ZONE_Y, color='green', ls='--', lw=1)
    ax.set_xlim(0, FIELD_SIZE)
    ax.set_ylim(0, FIELD_SIZE)
    ax.set_aspect('equal')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    t_conf = {}

    def update(fi):
        t, pos, k, path, _ = frames[idx[fi]]
        im.set_data(snaps[idx[fi]])
        trail.set_data(tr[:k + 1, 0], tr[:k + 1, 1])
        esc.set_data(path[:, 0], path[:, 1])
        dot.set_data([pos[0]], [pos[1]])
        # mines are shown once the risk map contains them (risk > 0.5 at the mine)
        sn = snaps[idx[fi]]
        shown = [mm for mm in m.mines if sn[int(mm[1]), int(mm[0])] > 0.5]
        if shown:
            conf.set_data(np.array(shown)[:, 0], np.array(shown)[:, 1])
        ax.set_title(f'S054 Risk-aware sweep, t = {t:.0f} s\n'
                     f'black = current escape path; red circles = detected mines', fontsize=10)
        return im, trail, esc, dot, conf

    del t_conf
    anim = animation.FuncAnimation(fig, update, frames=len(idx), blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    anim.save(path, writer='pillow', fps=8, dpi=72)
    plt.close(fig)
    from PIL import Image
    with Image.open(path) as gif:
        n_file = gif.n_frames      # Pillow merges identical consecutive frames
    print(f'Saved: {path} ({len(idx)} frames rendered, {n_file} in the file, '
          f'every {step} log samples of {EVAL_EVERY:.0f} s)')


if __name__ == '__main__':
    field, main, missions, mc, (d_dij, d_ast) = run_simulation()
    print('\n=== S054 Minefield Detection — Results ===')
    print(f'  Untruncated card sensor: one pass at 20 m lateral offset gives P_d^cum = '
          f'{untruncated_pass_pd(20.0):.3f}; at 50 m {untruncated_pass_pd(50.0):.3f} '
          f'(sensor truncated at r = {R_GPR_MAX:.0f} m)')
    print(f'  Single-step P_d: r = 4 m {float(gpr_detect_prob(4.0)):.3f}, r = 2 m '
          f'{float(gpr_detect_prob(2.0)):.3f}; alarm radius (P_d > 0.5) '
          f'{R_GPR / np.sqrt(2 * np.log(2)):.2f} m')
    print(f'  Dijkstra vs A* escape cost from (150,150) on the final risk map: '
          f'{d_dij:.4f} vs {d_ast:.4f}')
    n_strips = int(np.ceil(FIELD_SIZE / STRIP_WIDTH))
    print(f'  Card pattern: {n_strips} strips, {n_strips * FIELD_SIZE + (n_strips - 1) * STRIP_WIDTH:.0f} m, '
          f'{(n_strips * FIELD_SIZE + (n_strips - 1) * STRIP_WIDTH) / DRONE_SPEED:.0f} s')
    print(f'  Showcase field: seed {MAIN_SEED}, {N_MINE_DEFAULT} mines')
    hdr = ('time', 't_survey', 'dist', 'coverage', 'detected', 'confirmed', 'missed', 'fa_raised',
           'fa_unresolved', 'esc_mean', 'esc_mean_max', 'esc_max_max', 'frac_esc_unsafe',
           'frac_esc_danger_cell', 'danger_dist', 'risk_updates', 'detours', 'aborts', 'reclass_visits')
    for s in STRATS:
        r = main[s]
        print(f'  [{LABELS[s]}]')
        print(f"    mission time {r['time']:.0f} s (survey ends at {r['t_survey']:.0f} s, then return), "
              f"distance {r['dist']:.0f} m, coverage {100 * r['coverage']:.2f}%")
        print(f"    mines {r['n_mines']}: detected {r['detected']}, confirmed {r['confirmed']}, "
              f"missed {r['missed']}; false alarms raised {r['fa_raised']}, unresolved {r['fa_unresolved']}")
        print(f"    escape path: mean R {r['esc_mean']:.4f} (max of means {r['esc_mean_max']:.3f}), "
              f"max cell R {r['esc_max_max']:.3f}; samples with mean R >= 0.1: "
              f"{100 * r['frac_esc_unsafe']:.1f}%, with a cell R >= 0.4: {100 * r['frac_esc_danger_cell']:.1f}%")
        print(f"    distance over danger cells {r['danger_dist']:.1f} m; risk-map updates "
              f"{r['risk_updates']}, detour plans {r['detours']}, aborts {r['aborts']}, "
              f"reclassification visits {r['reclass_visits']}")
    mf = missions['fixed']
    det = mf.alarmed | mf.confirmed
    strips = STRIP_WIDTH / 2 + STRIP_WIDTH * np.arange(int(np.ceil(FIELD_SIZE / STRIP_WIDTH)))
    for k in np.where(~det)[0]:
        x, y = mf.mines[k]
        lat = np.min(np.abs(strips - x))
        print(f'  Fixed run, missed mine at ({x:.1f}, {y:.1f}): {lat:.2f} m from the nearest strip '
              f'centreline, P_d^cum {mf.cum_pd[k]:.3f}')
    print(f'  Strips flown by the fixed run end at x <= {mf.traj[int(round(mf.t_survey / DT))][0]:.0f} m')
    print(f'  Monte Carlo ({N_MC} fields, N_mine ~ U{{8,20}}): mean +- std')
    for s in STRATS:
        a = {k: np.array([r[k] for r in mc[s]], dtype=float) for k in hdr + ('n_mines',)}
        print(f"    {LABELS[s]:25s} time {a['time'].mean():.0f} +- {a['time'].std():.0f} s, "
              f"coverage {100 * a['coverage'].mean():.2f}%, missed {a['missed'].mean():.2f} +- "
              f"{a['missed'].std():.2f} (of {a['n_mines'].mean():.1f}), "
              f"FA unresolved {a['fa_unresolved'].mean():.1f} +- {a['fa_unresolved'].std():.1f} "
              f"(raised {a['fa_raised'].mean():.1f}), danger distance {a['danger_dist'].mean():.1f} +- "
              f"{a['danger_dist'].std():.1f} m, unsafe-escape samples {100 * a['frac_esc_unsafe'].mean():.1f}%, "
              f"aborts {int(a['aborts'].sum())}")
    tt = np.array([[r['time'] for r in mc[s]] for s in STRATS])
    dd = np.array([[r['danger_dist'] for r in mc[s]] for s in STRATS])
    print(f"  Fastest mission: " + ', '.join(f"{LABELS[s]} {int(np.sum(np.argmin(tt, 0) == i))}"
                                            for i, s in enumerate(STRATS)) + f" of {N_MC}")
    print(f"  Risk-aware flies less over danger than fixed in {int(np.sum(dd[1] < dd[0]))}/{N_MC} fields, "
          f"frontier less than fixed in {int(np.sum(dd[2] < dd[0]))}/{N_MC}")
    print(f"  Risk-aware slower than fixed in {int(np.sum(tt[1] > tt[0]))}/{N_MC}; mean extra "
          f"{(tt[1] - tt[0]).mean():.0f} s; frontier / fixed time {tt[2].mean() / tt[0].mean():.1f}x")

    def paired(key, a, b):
        x = np.array([r[key] for r in mc[a]], dtype=float)
        y = np.array([r[key] for r in mc[b]], dtype=float)
        return int(np.sum(x < y)), int(np.sum(x == y)), int(np.sum(x > y))

    for key, lab in (('missed', 'mines missed'), ('fa_unresolved', 'unresolved false alarms')):
        for a in ('risk', 'frontier'):
            f, e, m_ = paired(key, a, 'fixed')
            print(f"  Paired {lab}: {LABELS[a]} vs fixed -> fewer {f}, equal {e}, more {m_} of {N_MC}")
    for w in SEAM_WIDTHS:
        runs = mc[('fixed', w)]
        a = {k: np.array([r[k] for r in runs], dtype=float) for k in ('time', 'missed', 'coverage', 'n_mines')}
        miss8 = np.array([r['missed'] for r in mc['fixed']], dtype=float)
        print(f"  Fixed boustrophedon with {w:.1f} m strips: time {a['time'].mean():.0f} +- {a['time'].std():.0f} s, "
              f"coverage {100 * a['coverage'].mean():.2f}%, missed {a['missed'].mean():.2f} +- {a['missed'].std():.2f} "
              f"(8 m strips: {miss8.mean():.2f}); fewer than 8 m in {int(np.sum(a['missed'] < miss8))}, "
              f"equal {int(np.sum(a['missed'] == miss8))}, more {int(np.sum(a['missed'] > miss8))} of {N_MC}")
    for key, lab in (('fixed', '8.0'),) + tuple((('fixed', w), f'{w:.1f}') for w in SEAM_WIDTHS):
        runs = mc[key]
        tot = sum(r['missed'] for r in runs)
        seam = sum(r['miss_seam'] for r in runs)
        outside = sum(r['miss_outside'] for r in runs)
        rs = [v for r in runs for v in r['miss_seam_r']]
        ro = [v for r in runs for v in r['miss_outside_r']]
        rng_s = f" (closest approach {min(rs):.3f}-{max(rs):.3f} m)" if rs else ''
        rng_o = f" (closest approach {min(ro):.3f}-{max(ro):.3f} m)" if ro else ''
        print(f"  Missed mines with {lab} m strips over {N_MC} fields: {tot}; between two flown "
              f"strips (seam) {seam}{rng_s}; outside the flown strips (left by the 95 % stop) "
              f"{outside}{rng_o}")

    out = os.path.normpath(OUTPUT_DIR)
    os.makedirs(out, exist_ok=True)
    plot_risk_maps(missions, out)
    plot_coverage_map(missions, out)
    plot_escape_path(missions, out)
    plot_time_series(missions, out)
    plot_detection_summary(missions, out)
    plot_mc(mc, out)
    plot_risk_3d(missions, out)
    save_animation(missions, out)
