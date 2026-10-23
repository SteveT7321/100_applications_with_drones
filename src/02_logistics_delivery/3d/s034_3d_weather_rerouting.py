"""
S034 3D Upgrade — Weather Rerouting
===================================
A delivery drone flies from (0, 0, 31) m to (90, 90, 31) m through a
100 x 100 x 60 m airspace.  The planner graph is a 2 m voxel grid,
26-connected, with node altitudes 5 ... 55 m (the card's floor and ceiling).
Eight axis-aligned ellipsoidal convective cells (semi-axes a, b ~ U(8, 20) m,
c ~ U(10, 30) m) appear and expire during the flight.  Three altitude bands
carry different mean winds (2 / 6 / 3 m/s below 20 m / 20-40 m / above 40 m).

Card edge cost (D* Lite, backward search from the goal):
    c(u,v,t) = L + beta W(z_u) max(e.w_hat, 0) L + alpha L sum_k w_k 1[v in H_k(t)]
               + gamma |z_v - z_u|
Ground speed along an edge (card): v_eff = v_max - W(z_u) e.w_hat, where w_hat
is the direction the wind comes FROM.  Edges with v_eff < V_EFF_MIN are not
flyable (the card's formula goes negative there because W_mid > v_max).

Variants (same weather, same wind, same seeds):
  blind - layer-locked at the 31 m cruise layer, no weather information
  flat  - layer-locked at 31 m (the 2D "flat cruise" of S034), D* Lite
          replanning on weather events: horizontal detours only
  3d    - full 3D D* Lite replanning, free altitude 5-55 m

Two wind worlds:
  calm  - no wind: altitude changes can only come from weather avoidance
  shear - the card's three wind bands

Replanning triggers (card): newly active cell on the remaining path, expiring
cell within D_NEAR of it, and crossing a wind-band boundary.

Outputs (outputs/02_logistics_delivery/3d/s034_3d_weather_rerouting/):
  trajectory_3d.png, altitude_wind_layers.png, replan_timeline.png,
  cost_breakdown.png, animation.gif

Usage:
    conda activate drones
    python src/02_logistics_delivery/3d/s034_3d_weather_rerouting.py
"""

import matplotlib
matplotlib.use('Agg')

import os
import heapq
import math
from dataclasses import dataclass
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from matplotlib.animation import FuncAnimation, PillowWriter

OUTPUT_DIR = os.path.normpath(os.path.join(
    os.path.dirname(__file__), '..', '..', '..',
    'outputs', '02_logistics_delivery', '3d', 's034_3d_weather_rerouting',
))

# ── Parameters (card unless noted) ─────────────────────────────────
GRID       = 2.0              # m voxel side (xy and z)
NX, NY     = 50, 50           # x, y nodes at 0 ... 98 m
Z_MIN, Z_MAX = 5.0, 55.0      # m altitude floor / ceiling
NZ         = int(round((Z_MAX - Z_MIN) / GRID)) + 1   # 26 layers (card said 30: fixed)
V_MAX      = 5.0              # m/s airspeed
V_EFF_MIN  = 0.5              # m/s (added) slowest flyable ground speed on an edge
DT         = 0.05             # s
ALPHA      = 10.0             # hazard inflation
BETA       = 0.5              # wind-shear penalty weight
GAMMA      = 1.5              # vertical penalty per m of climb / descent
Z_MARGIN   = 3.0              # m vertical bypass margin
N_HAZARDS  = 8
AB_RANGE   = (8.0, 20.0)      # m horizontal semi-axes
C_RANGE    = (10.0, 30.0)     # m vertical semi-axis
TAU_RANGE  = (30.0, 120.0)    # s lifetime
W_RANGE    = (0.3, 1.0)       # severity
T_ON_RANGE = (-60.0, 20.0)    # s activation (added, as in the 2D fix)
ROUTE_FRAC = (0.15, 0.85)     # cell centre along the straight route (added)
LAT_OFFSET = 10.0             # m +/- lateral offset of the centre (added)
Z_OFFSET   = 8.0              # m +/- centre altitude around the cruise layer (added)
D_NEAR     = 6.0              # m expiry trigger margin (as in the 2D fix)
W_BANDS    = {'calm': (0.0, 0.0, 0.0), 'shear': (2.0, 6.0, 3.0)}   # low / mid / high
BAND_EDGES = (20.0, 40.0)     # m band boundaries
PSI_W      = math.radians(105.0)   # wind comes from 105 deg (added; route is 45 deg)
W_HAT      = np.array([math.cos(PSI_W), math.sin(PSI_W), 0.0])
Z_CRUISE   = 31.0             # m start / goal / flat-cruise altitude (added)
P_START    = np.array([0.0, 0.0, Z_CRUISE])
P_GOAL     = np.array([90.0, 90.0, Z_CRUISE])
T_MAX      = 300.0            # s simulation cap
N_SEEDS    = 12               # Monte-Carlo weather layouts (seeds 0 ... N_SEEDS-1)

WORLDS   = ['calm', 'shear']
VARIANTS = ['blind', 'flat', '3d']
VAR_LABEL = {'blind': 'Blind, flat 31 m', 'flat': 'Flat-cruise D* Lite (2D)',
             '3d': '3D D* Lite'}
VAR_COLOR = {'blind': 'tab:gray', 'flat': 'tab:purple', '3d': 'tab:blue'}


# ── Geometry helpers ───────────────────────────────────────────────
def k_of_z(z):
    return int(round((z - Z_MIN) / GRID))


def node_xyz(v):
    return np.array([v[0] * GRID, v[1] * GRID, Z_MIN + v[2] * GRID])


def band_of_z(z):
    return 0 if z < BAND_EDGES[0] else (1 if z < BAND_EDGES[1] else 2)


def wind_of_z(z, world):
    return W_BANDS[world][band_of_z(z)]


OFFS_3D = [(di, dj, dk) for di in (-1, 0, 1) for dj in (-1, 0, 1) for dk in (-1, 0, 1)
           if (di, dj, dk) != (0, 0, 0)]
OFFS_2D = [o for o in OFFS_3D if o[2] == 0]


def _edge_static(o, k, world, gamma):
    """Static part of the card edge cost (distance + shear + vertical); inf if unflyable."""
    d = np.array(o, dtype=float) * GRID
    L = float(np.linalg.norm(d))
    e = d / L
    W = wind_of_z(Z_MIN + k * GRID, world)
    head = float(e @ W_HAT)
    if V_MAX - W * head < V_EFF_MIN:
        return math.inf
    return L + BETA * W * max(head, 0.0) * L + gamma * abs(d[2])


GAMMA_SENS = 0.0              # (added) vertical penalty of the calm-world sensitivity run
STATIC = {(w, g): [[_edge_static(o, k, w, g) for o in OFFS_3D] for k in range(NZ)]
          for w in WORLDS for g in (GAMMA, GAMMA_SENS)}
OFF_LEN = [GRID * math.sqrt(o[0] ** 2 + o[1] ** 2 + o[2] ** 2) for o in OFFS_3D]
OFF_INDEX = {o: i for i, o in enumerate(OFFS_3D)}


def v_eff_edge(u, v, world):
    d = node_xyz(v) - node_xyz(u)
    e = d / np.linalg.norm(d)
    return V_MAX - wind_of_z(node_xyz(u)[2], world) * float(e @ W_HAT)


# ── Hazards ────────────────────────────────────────────────────────
@dataclass
class Cell:
    center: np.ndarray
    axes: np.ndarray        # (a, b, c)
    severity: float
    t_on: float
    t_off: float


def is_active(cell, t):
    return cell.t_on <= t < cell.t_off


def generate_hazards(seed):
    rng = np.random.default_rng(seed)
    d = (P_GOAL - P_START)[:2]
    u = d / np.linalg.norm(d)
    n = np.array([-u[1], u[0]])
    cells = []
    for _ in range(N_HAZARDS):
        s = rng.uniform(*ROUTE_FRAC)
        lat = rng.uniform(-LAT_OFFSET, LAT_OFFSET)
        xy = P_START[:2] + s * d + lat * n
        z = Z_CRUISE + rng.uniform(-Z_OFFSET, Z_OFFSET)
        a, b = rng.uniform(*AB_RANGE), rng.uniform(*AB_RANGE)
        c = rng.uniform(*C_RANGE)
        w = rng.uniform(*W_RANGE)
        t_on = rng.uniform(*T_ON_RANGE)
        tau = rng.uniform(*TAU_RANGE)
        cells.append(Cell(np.array([xy[0], xy[1], z]), np.array([a, b, c]), w, t_on, t_on + tau))
    return cells


_I, _J, _K = np.meshgrid(np.arange(NX), np.arange(NY), np.arange(NZ), indexing='ij')
_PX, _PY, _PZ = _I * GRID, _J * GRID, Z_MIN + _K * GRID


def cell_voxels(cell, margin=0.0):
    ax = cell.axes + margin
    m = (((_PX - cell.center[0]) / ax[0]) ** 2 + ((_PY - cell.center[1]) / ax[1]) ** 2
         + ((_PZ - cell.center[2]) / ax[2]) ** 2) <= 1.0
    return {(int(i), int(j), int(k)) for i, j, k in zip(_I[m], _J[m], _K[m])}


def inside(cell, p, margin=0.0):
    q = (p - cell.center) / (cell.axes + margin)
    return float(q @ q) <= 1.0


def hazard_weights(cells, active_idx, layer=None):
    hw = {}
    for kk in sorted(active_idx):
        for v in cells[kk].vox:
            if layer is None or v[2] == layer:
                hw[v] = hw.get(v, 0.0) + cells[kk].severity
    return hw


# ── D* Lite ────────────────────────────────────────────────────────
class DStarLite:
    """Backward D* Lite on the voxel graph.  `layer` locks the search to one z layer."""

    def __init__(self, start, goal, hw, world, layer=None, gamma=GAMMA):
        self.start, self.goal, self.km = start, goal, 0.0
        self.hw, self.world, self.layer, self.gamma = hw, world, layer, gamma
        self.offs = [OFF_INDEX[o] for o in (OFFS_2D if layer is not None else OFFS_3D)]
        ks = range(NZ) if layer is None else [layer]
        self.w_lb = min(wind_of_z(Z_MIN + k * GRID, world) for k in ks)
        self.static = STATIC[(world, gamma)]
        self.g, self.rhs = {}, {goal: 0.0}
        self.U = [(self._key(goal), goal)]
        self.expanded = 0

    def nbrs(self, s):
        i, j, k = s
        for oi in self.offs:
            di, dj, dk = OFFS_3D[oi]
            a, b, c = i + di, j + dj, k + dk
            if 0 <= a < NX and 0 <= b < NY and 0 <= c < NZ:
                yield oi, (a, b, c)

    def cost(self, s, oi, nb):
        return self.static[s[2]][oi] + OFF_LEN[oi] * ALPHA * self.hw.get(nb, 0.0)

    def _h(self, a, b):
        """Consistent lower bound on the cost a -> b (card heuristic, made admissible)."""
        dx, dy, dz = (b[0] - a[0]) * GRID, (b[1] - a[1]) * GRID, (b[2] - a[2]) * GRID
        head = dx * W_HAT[0] + dy * W_HAT[1]
        return math.sqrt(dx * dx + dy * dy + dz * dz) + self.gamma * abs(dz) + BETA * self.w_lb * max(head, 0.0)

    def _key(self, s):
        m = min(self.g.get(s, math.inf), self.rhs.get(s, math.inf))
        return (round(m + self._h(self.start, s) + self.km, 6), round(m, 6))

    def update_vertex(self, s):
        if s != self.goal:
            # inlined nbrs() + cost(): this loop is the hot spot of the script
            best = math.inf
            g, hw = self.g, self.hw
            i, j, k = s
            st = self.static[k]
            for oi in self.offs:
                di, dj, dk = OFFS_3D[oi]
                a, b, c = i + di, j + dj, k + dk
                if 0 <= a < NX and 0 <= b < NY and 0 <= c < NZ:
                    nb = (a, b, c)
                    gv = g.get(nb)
                    if gv is None or gv == math.inf:
                        continue
                    w = hw.get(nb)
                    # same operation order as cost(s, oi, nb) + g(nb)
                    val = (st[oi] + OFF_LEN[oi] * ALPHA * w if w else st[oi] + 0.0) + gv
                    if val < best:
                        best = val
            self.rhs[s] = best
        if self.g.get(s, math.inf) != self.rhs.get(s, math.inf):
            heapq.heappush(self.U, (self._key(s), s))

    def compute_shortest_path(self):
        n0 = self.expanded
        while self.U:
            k_old, u = self.U[0]
            if not (k_old < self._key(self.start)
                    or self.rhs.get(self.start, math.inf) != self.g.get(self.start, math.inf)):
                break
            heapq.heappop(self.U)
            g_u, rhs_u = self.g.get(u, math.inf), self.rhs.get(u, math.inf)
            if g_u == rhs_u:
                continue
            k_new = self._key(u)
            if k_old < k_new:
                heapq.heappush(self.U, (k_new, u))
                continue
            self.expanded += 1
            if g_u > rhs_u:
                # over-consistent: g(u) drops, so rhs(s) = min(rhs(s), c(s,u) + g(u)) for
                # every predecessor s (optimised D* Lite, Koenig & Likhachev 2002, Fig. 4)
                self.g[u] = rhs_u
                w = self.hw.get(u)
                i, j, k = u
                for oi in self.offs:
                    di, dj, dk = OFFS_3D[oi]
                    a, b, c = i - di, j - dj, k - dk          # s = u - offset, edge s -> u
                    if 0 <= a < NX and 0 <= b < NY and 0 <= c < NZ:
                        s = (a, b, c)
                        if s == self.goal:
                            continue
                        st = self.static[c][oi]
                        val = (st + OFF_LEN[oi] * ALPHA * w if w else st + 0.0) + rhs_u
                        if val < self.rhs.get(s, math.inf):
                            self.rhs[s] = val
                            if self.g.get(s, math.inf) != val:
                                heapq.heappush(self.U, (self._key(s), s))
            else:
                self.g[u] = math.inf
                self.update_vertex(u)
                for _, nb in self.nbrs(u):
                    self.update_vertex(nb)
        return self.expanded - n0

    def update_costs(self, new_hw):
        changed = {v for v in set(self.hw) | set(new_hw)
                   if self.hw.get(v, 0.0) != new_hw.get(v, 0.0)}
        self.hw = new_hw
        touched = set()
        for v in changed:
            for _, nb in self.nbrs(v):
                touched.add(nb)
        for s in sorted(touched):
            self.update_vertex(s)
        return len(changed)

    def move_start(self, s):
        if s != self.start:
            self.km += self._h(self.start, s)
            self.start = s

    def extract_path(self):
        path, s = [self.start], self.start
        seen = {s}
        for _ in range(NX * NY * NZ):
            if s == self.goal:
                break
            best, best_nb = math.inf, None
            for oi, nb in self.nbrs(s):
                val = self.cost(s, oi, nb) + self.g.get(nb, math.inf)
                if val < best:
                    best, best_nb = val, nb
            assert best_nb is not None and best < math.inf and best_nb not in seen, 'path extraction failed'
            path.append(best_nb)
            seen.add(best_nb)
            s = best_nb
        return path

    def g_start(self):
        return self.g.get(self.start, math.inf)


# ── Simulation ─────────────────────────────────────────────────────
START_V = (0, 0, k_of_z(Z_CRUISE))
GOAL_V = (int(round(P_GOAL[0] / GRID)), int(round(P_GOAL[1] / GRID)), k_of_z(Z_CRUISE))
J_BASE = float(np.linalg.norm(P_GOAL - P_START))


def make_cells(seed):
    cells = generate_hazards(seed)
    for c in cells:
        c.vox = cell_voxels(c)
    return cells


def _plen(path):
    return float(sum(OFF_LEN[OFF_INDEX[(b[0] - a[0], b[1] - a[1], b[2] - a[2])]]
                     for a, b in zip(path[:-1], path[1:])))


def classify_pass(path, cell):
    """How a planned path passes a cell.

    around  - never enters the cell's xy footprint
    through - a path node lies inside the cell
    over / under       - crosses the footprint above / below the cell after climbing / diving
                         (footprint altitude differs from the plan's start altitude)
    level-over / level-under - crosses the footprint above / below the cell at the plan's
                         start altitude, i.e. without any altitude change
    """
    P = np.array([node_xyz(v) for v in path])
    hx = (((P[:, 0] - cell.center[0]) / cell.axes[0]) ** 2
          + ((P[:, 1] - cell.center[1]) / cell.axes[1]) ** 2)
    foot = hx <= 1.0
    if not foot.any():
        return 'around'
    if any(v in cell.vox for v in path):
        return 'through'
    side = 'over' if float(np.mean(P[foot, 2])) > cell.center[2] else 'under'
    if np.all(np.abs(P[foot, 2] - P[0, 2]) < 1e-9):
        return 'level-' + side
    return side


def bypass_heuristic(z_drone, cell):
    """Card vertical-bypass check; the margin and the floor are part of the feasibility test."""
    dz_above = cell.center[2] + cell.axes[2] - z_drone
    dz_below = z_drone - (cell.center[2] - cell.axes[2])
    z_up = z_drone + dz_above + Z_MARGIN
    z_dn = z_drone - dz_below - Z_MARGIN
    opts = []
    if z_up <= Z_MAX:
        opts.append((abs(dz_above), 'over', z_up))
    if z_dn >= Z_MIN:
        opts.append((abs(dz_below), 'under', z_dn))
    if not opts:
        return 'none', None
    _, kind, zb = min(opts)
    return kind, zb


def path_cost(path, planner):
    c = 0.0
    for a, b in zip(path[:-1], path[1:]):
        c += planner.cost(a, OFF_INDEX[(b[0] - a[0], b[1] - a[1], b[2] - a[2])], b)
    return c


def _check_path(path, planner):
    """The extracted path must realise the D* Lite cost-to-go g(start)."""
    assert abs(path_cost(path, planner) - planner.g_start()) < 1e-6, 'path cost != g(start)'


def _search_cost(a, b, hw, world, gamma):
    d = DStarLite(a, b, hw, world, None, gamma)
    d.compute_shortest_path()
    return d.g_start()


def run_simulation(seed=0, world='shear', variant='3d', analyse=False, gamma=GAMMA):
    """Fly one variant through one weather layout in one wind world."""
    cells = make_cells(seed)
    layer = None if variant == '3d' else START_V[2]
    act = {k for k, c in enumerate(cells) if is_active(c, 0.0)}
    hw0 = {} if variant == 'blind' else hazard_weights(cells, act, layer)
    planner = DStarLite(START_V, GOAL_V, hw0, world, layer, gamma)
    init_exp = planner.compute_shortest_path()
    path = planner.extract_path()
    _check_path(path, planner)
    planned = [(0.0, 'initial', path)]
    waypoints = path[1:]
    last = START_V
    pos = node_xyz(START_V)
    t = 0.0
    traj, times = [pos.copy()], [0.0]
    replans, decisions = [], []
    acc = dict(L=0.0, Lxy=0.0, dz=0.0, shear=0.0, vert=0.0, hazc=0.0)
    hazard_time = sev_exp = 0.0
    prev_active = act
    band_prev = band_of_z(pos[2])
    pending_layer = False
    min_veff = math.inf

    while t < T_MAX and waypoints:
        active = {k for k, c in enumerate(cells) if is_active(c, t)}
        trig = None
        if variant != 'blind':
            if active != prev_active:
                planner.update_costs(hazard_weights(cells, active, layer))
                rem_set = set(waypoints)
                hit_new = [k for k in sorted(active - prev_active) if cells[k].vox & rem_set]
                rem_xyz = [node_xyz(v) for v in waypoints]
                near_exp = [k for k in sorted(prev_active - active)
                            if any(inside(cells[k], p, D_NEAR) for p in rem_xyz)]
                if hit_new:
                    trig = ('activation', hit_new)
                elif near_exp:
                    trig = ('expiry', near_exp)
            if trig is None and pending_layer:
                trig = ('layer', [])
        prev_active = active
        pending_layer = False

        if trig is not None:
            s_plan = waypoints[0]           # commit to the edge being flown
            old_rem = waypoints[:]
            old_cost = path_cost(old_rem, planner)
            planner.move_start(s_plan)
            n_exp = planner.compute_shortest_path()
            new_path = planner.extract_path()
            _check_path(new_path, planner)
            rec = dict(t=t, kind=trig[0], cells=trig[1], exp=n_exp,
                       changed=new_path != old_rem, d_cost=planner.g_start() - old_cost,
                       d_len=_plen(new_path) - _plen(old_rem))
            if analyse:
                fresh = DStarLite(s_plan, GOAL_V, planner.hw, world, layer, gamma)
                rec['scratch'] = fresh.compute_shortest_path()
                rec['g_err'] = abs(fresh.g_start() - planner.g_start())
            if trig[0] == 'activation' and variant == '3d' and analyse:
                for k in trig[1]:
                    c = cells[k]
                    heur, zb = bypass_heuristic(pos[2], c)
                    # forced-horizontal alternative: the cell's whole xy footprint column is hazardous
                    hw_f = dict(planner.hw)
                    cols = {(v[0], v[1]) for v in c.vox}
                    for (i, j) in sorted(cols):
                        for kz in range(NZ):
                            if (i, j, kz) not in c.vox:
                                hw_f[(i, j, kz)] = hw_f.get((i, j, kz), 0.0) + c.severity
                    forced = DStarLite(s_plan, GOAL_V, hw_f, world, None, gamma)
                    forced.compute_shortest_path()
                    fp = forced.extract_path()
                    # vertical alternative: best path through the card's bypass point above / below
                    # the cell centre (s_plan -> B -> goal, same cost map)
                    cost_v = math.inf
                    if zb is not None:
                        kb = math.ceil((zb - Z_MIN) / GRID - 1e-9) if heur == 'over' \
                            else math.floor((zb - Z_MIN) / GRID + 1e-9)
                        B = (int(np.clip(round(c.center[0] / GRID), 0, NX - 1)),
                             int(np.clip(round(c.center[1] / GRID), 0, NY - 1)), int(kb))
                        cost_v = (_search_cost(s_plan, B, planner.hw, world, gamma)
                                  + _search_cost(B, GOAL_V, planner.hw, world, gamma))
                    decisions.append(dict(t=t, seed=seed, cell=k, choice=classify_pass(new_path, c),
                                          heur=heur, z_bypass=zb, z_drone=float(pos[2]),
                                          cost=planner.g_start(), cost_h=path_cost(fp, planner),
                                          choice_h=classify_pass(fp, c), cost_v=cost_v))
            replans.append(rec)
            waypoints = new_path[:]          # still flying to s_plan = new_path[0]
            planned.append((t, trig[0], new_path))

        # exposure at the true position
        ins = [cells[k] for k in sorted(active) if inside(cells[k], pos)]
        if ins:
            hazard_time += DT
            sev_exp += DT * sum(c.severity for c in ins)

        # move along grid edges for one time step at the edge ground speed
        tleft = DT
        while tleft > 1e-12 and waypoints:
            v = waypoints[0]
            pv = node_xyz(v)
            d_edge = pv - node_xyz(last)
            e = d_edge / np.linalg.norm(d_edge)
            ve = v_eff_edge(last, v, world)
            assert ve >= V_EFF_MIN - 1e-9, 'unflyable edge flown'
            min_veff = min(min_veff, ve)
            d = float(np.linalg.norm(pv - pos))
            step = min(d, ve * tleft)
            W = wind_of_z(node_xyz(last)[2], world)
            hsum = sum(cells[k].severity for k in sorted(active) if v in cells[k].vox)
            acc['L'] += step
            acc['Lxy'] += step * math.hypot(e[0], e[1])
            acc['dz'] += step * abs(e[2])
            acc['shear'] += BETA * W * max(float(e @ W_HAT), 0.0) * step
            acc['vert'] += gamma * abs(e[2]) * step
            acc['hazc'] += ALPHA * hsum * step
            if d <= ve * tleft:
                pos = pv.copy()
                last = v
                waypoints.pop(0)
                tleft -= d / ve
            else:
                pos = pos + e * step
                tleft = 0.0
        assert Z_MIN - 1e-9 <= pos[2] <= Z_MAX + 1e-9, 'altitude limit violated'
        t = round(t + DT, 10)
        traj.append(pos.copy())
        times.append(t)
        b = band_of_z(pos[2])
        if variant == '3d' and world == 'shear' and b != band_prev:
            pending_layer = True
        band_prev = b

    traj = np.array(traj)
    arrived = (not waypoints) and bool(np.allclose(pos, P_GOAL))
    flown = acc['L'] + acc['shear'] + acc['vert'] + acc['hazc']
    return dict(seed=seed, world=world, variant=variant, cells=cells, traj=traj,
                times=np.array(times), replans=replans, decisions=decisions, planned=planned,
                init_exp=init_exp, arrived=arrived, T=t, L=acc['L'],
                J_dxy=acc['Lxy'] - J_BASE, J_dz=acc['dz'], J_shear=acc['shear'],
                J_vert=acc['vert'], J_hazc=acc['hazc'], J_hazard=hazard_time, sev=sev_exp,
                flown=flown, min_veff=min_veff, zmin=float(traj[:, 2].min()),
                zmax=float(traj[:, 2].max()), gamma=gamma)


# ── Plots ──────────────────────────────────────────────────────────
KIND_COLOR = {'activation': 'red', 'expiry': 'green', 'layer': 'black'}
BAND_COLOR = ['#deebf7', '#6baed6', '#c6dbef']


def _ellipsoid(ax, cell, color, alpha=0.35, lw=0.5):
    u = np.linspace(0, 2 * np.pi, 16)
    v = np.linspace(0, np.pi, 10)
    x = cell.center[0] + cell.axes[0] * np.outer(np.cos(u), np.sin(v))
    y = cell.center[1] + cell.axes[1] * np.outer(np.sin(u), np.sin(v))
    z = cell.center[2] + cell.axes[2] * np.outer(np.ones_like(u), np.cos(v))
    z = np.clip(z, 0.0, 60.0)
    ax.plot_wireframe(x, y, z, color=color, alpha=alpha, linewidth=lw)


def _footprint(ax, cell, color):
    u = np.linspace(0, 2 * np.pi, 60)
    ax.plot(cell.center[0] + cell.axes[0] * np.cos(u), cell.center[1] + cell.axes[1] * np.sin(u),
            color=color, lw=1.0, alpha=0.8)


def _flight_cells(r):
    return [c for c in r['cells'] if c.t_on < r['T'] and c.t_off > 0.0]


def _draw_pair(ax3, ax2, runs, title):
    """runs: list of (label, color, run).  The first run is coloured by altitude."""
    ref = runs[0][2]
    for c in _flight_cells(ref):
        col = 'orange' if c.t_on <= 0.0 else 'red'
        _ellipsoid(ax3, c, col)
        _footprint(ax2, c, col)
    sc = None
    for idx, (lab, col, r) in enumerate(runs):
        tr = r['traj']
        if idx == 0:
            sc = ax3.scatter(tr[::4, 0], tr[::4, 1], tr[::4, 2], c=tr[::4, 2], cmap='viridis',
                             vmin=Z_MIN, vmax=Z_MAX, s=6, label=lab)
            ax2.scatter(tr[::4, 0], tr[::4, 1], c=tr[::4, 2], cmap='viridis', vmin=Z_MIN, vmax=Z_MAX, s=5)
        else:
            ax3.plot(tr[:, 0], tr[:, 1], tr[:, 2], color=col, lw=1.6, label=lab)
            ax2.plot(tr[:, 0], tr[:, 1], color=col, lw=1.6, label=lab)
        for rp in r['replans']:
            if rp['kind'] == 'layer':
                continue
            p = tr[int(round(rp['t'] / DT))]
            ax3.scatter(*p, color=KIND_COLOR[rp['kind']], marker='x', s=40)
            ax2.scatter(p[0], p[1], color=KIND_COLOR[rp['kind']], marker='x', s=40, zorder=5)
    for ax in (ax3, ax2):
        ax.scatter(*P_START[:2 if ax is ax2 else 3], c='green', s=60, marker='^')
        ax.scatter(*P_GOAL[:2 if ax is ax2 else 3], c='black', s=100, marker='*')
    ax3.set_xlim(0, 100); ax3.set_ylim(0, 100); ax3.set_zlim(0, 60)
    ax3.set_xlabel('X (m)'); ax3.set_ylabel('Y (m)'); ax3.set_zlabel('Z (m)')
    ax3.view_init(elev=22, azim=-60)
    ax3.set_title(title, fontsize=10)
    ax3.legend(loc='upper left', fontsize=7)
    ax2.set_xlim(0, 100); ax2.set_ylim(0, 100); ax2.set_aspect('equal')
    ax2.set_xlabel('X (m)'); ax2.set_ylabel('Y (m)'); ax2.grid(alpha=0.3)
    ax2.set_title('Top-down (orange = active at take-off, red = appears in flight, x = replan)',
                  fontsize=9)
    ax2.legend(loc='lower right', fontsize=7)
    return sc


def plot_trajectory_3d(show, out_dir):
    fig = plt.figure(figsize=(15, 13))
    s = show['seed']
    rows = [
        ([('3D D* Lite (colour = altitude)', None, show['shear_3d']),
          (VAR_LABEL['flat'], VAR_COLOR['flat'], show['shear_flat'])],
         f'Card wind shear, seed {s}'),
        ([('3D D* Lite, gamma = 0 (colour = altitude)', None, show['calm_3d_g0']),
          ('3D D* Lite, gamma = 1.5', 'tab:blue', show['calm_3d'])],
         f"Calm air, seed {show['calm_seed']}: vertical penalty sensitivity"),
    ]
    sc = None
    for i, (runs, title) in enumerate(rows):
        ax3 = fig.add_axes([0.0, 0.52 - 0.47 * i, 0.5, 0.42], projection='3d')
        ax2 = fig.add_axes([0.55, 0.53 - 0.47 * i, 0.33, 0.38])
        sc = _draw_pair(ax3, ax2, runs, title)
    cax = fig.add_axes([0.92, 0.3, 0.015, 0.4])
    cb = fig.colorbar(sc, cax=cax)
    cb.set_label('Altitude (m)')
    fig.suptitle('S034 3D Weather Rerouting: trajectories', fontsize=13)
    path = os.path.join(out_dir, 'trajectory_3d.png')
    plt.savefig(path, dpi=110)
    plt.close()
    print(f'Saved: {path}')


def _bands(ax, world):
    lims = [(Z_MIN, BAND_EDGES[0]), (BAND_EDGES[0], BAND_EDGES[1]), (BAND_EDGES[1], Z_MAX)]
    for b, (lo, hi) in enumerate(lims):
        W = W_BANDS[world][b]
        ax.axhspan(lo, hi, color=BAND_COLOR[b] if world == 'shear' else '#f0f0f0', alpha=0.7, lw=0)
        ax.text(0.995, (lo + hi) / 2, f'W = {W:.0f} m/s', transform=ax.get_yaxis_transform(),
                ha='right', va='center', fontsize=8, color='k')


def _events(ax, r):
    for rp in r['replans']:
        ax.axvline(rp['t'], color=KIND_COLOR[rp['kind']], lw=0.9, ls='--', alpha=0.8)


def plot_altitude_wind(show, out_dir):
    fig, axes = plt.subplots(2, 1, figsize=(12, 8))
    ax = axes[0]
    _bands(ax, 'shear')
    for key, lab, col in [('shear_flat', VAR_LABEL['flat'], VAR_COLOR['flat']),
                          ('shear_3d', VAR_LABEL['3d'], VAR_COLOR['3d'])]:
        r = show[key]
        ax.plot(r['times'], r['traj'][:, 2], color=col, lw=1.8, label=f"{lab} (T = {r['T']:.1f} s)")
    _events(ax, show['shear_3d'])
    ax.set_ylim(0, 60); ax.set_ylabel('Altitude (m)')
    ax.set_title(f"Card wind shear (seed {show['seed']}): altitude vs time over the wind bands; "
                 'dashed = 3D replans (red activation, green expiry, black band crossing)', fontsize=9)
    ax.legend(loc='upper center', fontsize=8)
    ax = axes[1]
    _bands(ax, 'calm')
    for key, lab, col in [('calm_3d', '3D D* Lite, gamma = 1.5', 'tab:blue'),
                          ('calm_3d_g0', '3D D* Lite, gamma = 0', 'tab:cyan')]:
        r = show[key]
        ax.plot(r['times'], r['traj'][:, 2], color=col, lw=1.8, label=lab)
    _events(ax, show['calm_3d_g0'])
    ax.set_ylim(0, 60); ax.set_xlabel('Time (s)'); ax.set_ylabel('Altitude (m)')
    ax.set_title(f"Calm air (seed {show['calm_seed']}): altitude vs time; dashed = replans of the "
                 'gamma = 0 run', fontsize=9)
    ax.legend(loc='upper center', fontsize=8)
    fig.tight_layout()
    path = os.path.join(out_dir, 'altitude_wind_layers.png')
    plt.savefig(path, dpi=120)
    plt.close()
    print(f'Saved: {path}')


def plot_replan_timeline(show, mc, out_dir):
    fig, axes = plt.subplots(2, 1, figsize=(12, 8))
    r = show['shear_3d']
    tt = r['times']
    n_act = [sum(1 for c in r['cells'] if is_active(c, x)) for x in tt]
    ax = axes[0]
    ax.step(tt, n_act, where='post', color='darkorange')
    ax.fill_between(tt, n_act, step='post', color='orange', alpha=0.3)
    for rp in r['replans']:
        ax.axvline(rp['t'], color=KIND_COLOR[rp['kind']], lw=1.2)
        ax.text(rp['t'] + 0.3, 7.6, f"{rp['kind']}\ninc {rp['exp']} / scratch {rp['scratch']}",
                fontsize=7, va='top', color=KIND_COLOR[rp['kind']])
    ax.set_ylim(0, 8); ax.set_xlabel('Time (s)'); ax.set_ylabel('Active cells')
    ax.set_title(f"3D D* Lite replans, card wind shear, seed {show['seed']} "
                 '(node expansions: incremental / from scratch)', fontsize=10)
    ax.grid(alpha=0.3)
    ax = axes[1]
    groups = []
    for world in WORLDS:
        for kind in ('activation', 'expiry', 'layer'):
            rr = [rp for run in mc[(world, '3d', GAMMA)] for rp in run['replans'] if rp['kind'] == kind]
            if rr:
                groups.append((f'{world}\n{kind} (n={len(rr)})', np.mean([x['exp'] for x in rr]),
                               np.mean([x['scratch'] for x in rr])))
    x = np.arange(len(groups))
    b1 = ax.bar(x - 0.2, [g[1] for g in groups], 0.4, color='tab:blue', label='incremental D* Lite')
    b2 = ax.bar(x + 0.2, [g[2] for g in groups], 0.4, color='tab:gray', label='from scratch')
    ax.bar_label(b1, fmt='%.0f', fontsize=8); ax.bar_label(b2, fmt='%.0f', fontsize=8)
    ax.set_xticks(x); ax.set_xticklabels([g[0] for g in groups], fontsize=8)
    ax.set_ylabel('Mean node expansions per replan')
    ax.set_title(f'3D D* Lite replans over {N_SEEDS} layouts (gamma = 1.5)', fontsize=10)
    ax.legend(fontsize=8); ax.grid(axis='y', alpha=0.3)
    fig.tight_layout()
    path = os.path.join(out_dir, 'replan_timeline.png')
    plt.savefig(path, dpi=120)
    plt.close()
    print(f'Saved: {path}')


def plot_cost_breakdown(mc, out_dir):
    fig, axes = plt.subplots(1, 3, figsize=(17, 5.5))
    cfgs = [('calm', 'flat', GAMMA), ('calm', '3d', GAMMA), ('shear', 'flat', GAMMA), ('shear', '3d', GAMMA)]
    labels = ['calm\nflat', 'calm\n3D', 'shear\nflat', 'shear\n3D']
    comps = [('L', 'distance', 'tab:gray'), ('J_shear', 'wind shear', 'tab:cyan'),
             ('J_vert', 'vertical (gamma |dz|)', 'tab:green'), ('J_hazc', 'hazard inflation', 'tab:red')]
    ax = axes[0]
    bottom = np.zeros(len(cfgs))
    for key, lab, col in comps:
        vals = np.array([np.mean([r[key] for r in mc[c]]) for c in cfgs])
        ax.bar(labels, vals, bottom=bottom, color=col, label=lab)
        bottom += vals
    for i, c in enumerate(cfgs):
        f = [r['flown'] for r in mc[c]]
        ax.errorbar(i, np.mean(f), yerr=np.std(f), color='k', capsize=4)
        ax.text(i, np.mean(f) + np.std(f) + 5, f'{np.mean(f):.1f}', ha='center', fontsize=8)
    ax.set_ylabel('Flown composite cost (card c summed along the flight)')
    ax.set_title(f'Mean flown cost by term ({N_SEEDS} layouts, error bar = 1 std)', fontsize=10)
    ax.legend(fontsize=8); ax.grid(axis='y', alpha=0.3)
    ax = axes[1]
    keys = [('J_dxy', 'J_detour xy'), ('J_dz', 'J_detour z'), ('J_shear', 'J_shear')]
    x = np.arange(len(keys))
    w = 0.2
    for i, (c, lab) in enumerate(zip(cfgs, labels)):
        m = [np.mean([r[k] for r in mc[c]]) for k, _ in keys]
        sd = [np.std([r[k] for r in mc[c]]) for k, _ in keys]
        ax.bar(x + (i - 1.5) * w, m, w, yerr=sd, capsize=3, label=lab.replace('\n', ' '),
               color=['#bcbddc', '#9ecae1', '#756bb1', '#3182bd'][i])
    ax.set_xticks(x); ax.set_xticklabels([k[1] for k in keys])
    ax.set_ylabel('m'); ax.grid(axis='y', alpha=0.3)
    ax.set_title(f'Card metrics (J_base = {J_BASE:.2f} m for all), mean +/- std', fontsize=10)
    ax.legend(fontsize=8)
    ax = axes[2]
    for world, mk in (('calm', 'o'), ('shear', 's')):
        a = [r['flown'] for r in mc[(world, 'flat', GAMMA)]]
        b = [r['flown'] for r in mc[(world, '3d', GAMMA)]]
        ax.scatter(a, b, marker=mk, label=world, alpha=0.8)
    lo = min(min(r['flown'] for r in mc[c]) for c in cfgs) - 10
    hi = max(max(r['flown'] for r in mc[c]) for c in cfgs) + 10
    ax.plot([lo, hi], [lo, hi], 'k:', lw=1)
    ax.set_xlabel('Flat-cruise flown cost'); ax.set_ylabel('3D flown cost')
    ax.set_title('Per layout (same weather): below the diagonal = 3D cheaper', fontsize=10)
    ax.legend(fontsize=8); ax.grid(alpha=0.3)
    fig.tight_layout()
    path = os.path.join(out_dir, 'cost_breakdown.png')
    plt.savefig(path, dpi=120)
    plt.close()
    print(f'Saved: {path}')


def save_animation(show, out_dir, n_target=100):
    r3, rf = show['shear_3d'], show['shear_flat']
    n = max(len(r3['times']), len(rf['times']))
    step = int(math.ceil((n - 1) / (n_target - 1)))
    n_frames = int(math.ceil((n - 1) / step)) + 1
    fig = plt.figure(figsize=(8, 7))
    ax = fig.add_subplot(111, projection='3d')
    plan_t = [p[0] for p in r3['planned']]

    def update(f):
        i = min(f * step, n - 1)
        t = i * DT
        ax.cla()
        for c in r3['cells']:
            if is_active(c, t):
                _ellipsoid(ax, c, 'orange', alpha=0.45)
        i3 = min(i, len(r3['traj']) - 1)
        tr = r3['traj'][:i3 + 1]
        ax.scatter(tr[::4, 0], tr[::4, 1], tr[::4, 2], c=tr[::4, 2], cmap='viridis',
                   vmin=Z_MIN, vmax=Z_MAX, s=5)
        # current 3D plan: blue = initial plan, red = replanned
        j = max(q for q in range(len(plan_t)) if plan_t[q] <= t + 1e-9)
        kind, pth = r3['planned'][j][1], r3['planned'][j][2]
        if i3 < len(r3['traj']) - 1:
            P = np.array([node_xyz(v) for v in pth])
            ax.plot(P[:, 0], P[:, 1], P[:, 2], color='blue' if kind == 'initial' else 'red',
                    lw=1.0, ls='--')
        ax.scatter(*r3['traj'][i3], color='tab:blue', s=40)
        iff = min(i, len(rf['traj']) - 1)
        ax.plot(rf['traj'][:iff + 1, 0], rf['traj'][:iff + 1, 1], rf['traj'][:iff + 1, 2],
                color=VAR_COLOR['flat'], lw=1.4)
        ax.scatter(*rf['traj'][iff], color=VAR_COLOR['flat'], s=40)
        ax.scatter(*P_GOAL, c='black', marker='*', s=90)
        ax.set_xlim(0, 100); ax.set_ylim(0, 100); ax.set_zlim(0, 60)
        ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
        ax.view_init(elev=22, azim=-60)
        ax.set_title(f"S034 3D, card wind shear, seed {show['seed']}, t = {t:5.1f} s\n"
                     '3D drone (trail coloured by altitude; dashed plan: blue initial, red replanned) '
                     'vs flat cruise (purple)', fontsize=8)
        return []

    ani = FuncAnimation(fig, update, frames=n_frames, interval=100, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer=PillowWriter(fps=10), dpi=75)
    plt.close()
    print(f'Saved: {path}  ({n_frames} frames, every {step} steps)')


# ── Entry point ────────────────────────────────────────────────────
CONFIGS = [('calm', 'blind', GAMMA), ('calm', 'flat', GAMMA), ('calm', '3d', GAMMA),
           ('calm', '3d', GAMMA_SENS),
           ('shear', 'blind', GAMMA), ('shear', 'flat', GAMMA), ('shear', '3d', GAMMA)]


def _cfg_name(c):
    return f"{c[0]:5s} {c[1]:5s}" + (f" gamma={c[2]:.1f}" if c[1] == '3d' else ' ' * 10)


CHOICES = ['around', 'over', 'under', 'level-over', 'level-under', 'through']


def _sgn(x):
    return f'{round(x, 6) + 0.0:+7.2f}'


def _ms(x):
    return f'{np.mean(x):7.2f} +/- {np.std(x):6.2f}'


def main():
    out_dir = OUTPUT_DIR
    os.makedirs(out_dir, exist_ok=True)
    print(f'Grid {NX} x {NY} x {NZ} nodes, z = {Z_MIN:.0f} ... {Z_MAX:.0f} m; start '
          f'({P_START[0]:.0f}, {P_START[1]:.0f}, {P_START[2]:.0f}) m, goal '
          f'({P_GOAL[0]:.0f}, {P_GOAL[1]:.0f}, {P_GOAL[2]:.0f}) m; J_base = {J_BASE:.2f} m')
    print('Ground speed and cost per metre on a straight flat route (heading 45 deg) by band:')
    e45 = np.array([1.0, 1.0, 0.0]) / math.sqrt(2.0)
    head = float(e45 @ W_HAT)
    for b, nm in enumerate(['low (<20 m)', 'mid (20-40 m)', 'high (>40 m)']):
        W = W_BANDS['shear'][b]
        print(f'  {nm:14s} W = {W:.0f} m/s  e.w = {head:.3f}  v_eff = {V_MAX - W * head:.2f} m/s  '
              f'cost/m = {1 + BETA * W * max(head, 0):.2f}')
    bad = [(k, OFFS_3D[o]) for k in range(NZ) for o in range(26) if STATIC[('shear', GAMMA)][k][o] == math.inf]
    print(f'  unflyable (v_eff < {V_EFF_MIN} m/s) edges in the shear world: {len(bad)} of {NZ * 26} '
          f'(layer, direction) pairs; bands {sorted({band_of_z(Z_MIN + k * GRID) for k, _ in bad})} '
          f'(0 low, 1 mid, 2 high); directions {sorted({o for _, o in bad})}')

    print('\nNo-weather plans (static costs only):')
    for world in WORLDS:
        for var, layer in (('flat', START_V[2]), ('3d', None)):
            d = DStarLite(START_V, GOAL_V, {}, world, layer)
            d.compute_shortest_path()
            p = d.extract_path()
            zs = [node_xyz(v)[2] for v in p]
            print(f'  {world:5s} {var:4s} cost = {d.g_start():7.2f}  length = {_plen(p):6.2f} m  '
                  f'z = {min(zs):.0f} ... {max(zs):.0f} m')

    mc = {c: [run_simulation(s, c[0], c[1], analyse=(c[1] == '3d'), gamma=c[2])
              for s in range(N_SEEDS)] for c in CONFIGS}
    for c in CONFIGS:
        for r in mc[c]:
            assert r['arrived'], f'{c} seed {r["seed"]} did not arrive'

    print(f'\nMonte Carlo over {N_SEEDS} weather layouts (seeds 0 ... {N_SEEDS - 1}), mean +/- std:')
    for c in CONFIGS:
        rs = mc[c]
        kinds = [rp['kind'] for r in rs for rp in r['replans']]
        print(f"  {_cfg_name(c)} flown {_ms([r['flown'] for r in rs])}  T {_ms([r['T'] for r in rs])} s  "
              f"J_hazard {_ms([r['J_hazard'] for r in rs])} s (>0 in {sum(r['J_hazard'] > 0 for r in rs)})  "
              f"z {min(r['zmin'] for r in rs):.0f}-{max(r['zmax'] for r in rs):.0f} m  "
              f"left 31 m in {sum(r['zmin'] < Z_CRUISE or r['zmax'] > Z_CRUISE for r in rs)}  "
              f"replans act/exp/layer {kinds.count('activation')}/{kinds.count('expiry')}/{kinds.count('layer')}")
        print(f"      J_detour_xy {_ms([r['J_dxy'] for r in rs])} m  J_detour_z {_ms([r['J_dz'] for r in rs])} m  "
              f"J_shear {_ms([r['J_shear'] for r in rs])}  gamma|dz| {_ms([r['J_vert'] for r in rs])}  "
              f"hazard inflation {_ms([r['J_hazc'] for r in rs])}  sev-exposure {_ms([r['sev'] for r in rs])} s")
    print(f"  slowest ground speed flown: {min(r['min_veff'] for c in CONFIGS for r in mc[c]):.2f} m/s")

    print('\n3D vs flat cruise on the same layouts (flown cost; tie = |diff| < 1e-6):')
    for world in WORLDS:
        a = np.array([r['flown'] for r in mc[(world, 'flat', GAMMA)]])
        b = np.array([r['flown'] for r in mc[(world, '3d', GAMMA)]])
        d = b - a
        ha = np.array([r['J_hazard'] for r in mc[(world, 'flat', GAMMA)]])
        hb = np.array([r['J_hazard'] for r in mc[(world, '3d', GAMMA)]])
        print(f'  {world:5s} 3D cheaper {int(np.sum(d < -1e-6))}, tie {int(np.sum(abs(d) <= 1e-6))}, '
              f'dearer {int(np.sum(d > 1e-6))};  diff (3D - flat) mean {np.mean(d):+.2f}, '
              f'min {np.min(d):+.2f}, median {np.median(d):+.2f}, max {np.max(d):+.2f};  '
              f'relative mean {100 * np.mean(d / a):+.1f} %;  J_hazard lower/equal/higher '
              f'{int(np.sum(hb < ha - 1e-9))}/{int(np.sum(abs(hb - ha) <= 1e-9))}/{int(np.sum(hb > ha + 1e-9))}')
    a = np.array([r['flown'] for r in mc[('calm', '3d', GAMMA)]])
    b = np.array([r['flown'] for r in mc[('calm', '3d', GAMMA_SENS)]])
    print(f'  calm gamma=0 vs gamma=1.5 (each scored with its own gamma): flown diff mean {np.mean(b - a):+.2f}; '
          f"length gamma=0 {_ms([r['L'] for r in mc[('calm', '3d', GAMMA_SENS)]])} m vs gamma=1.5 "
          f"{_ms([r['L'] for r in mc[('calm', '3d', GAMMA)]])} m")

    print('\nReplanning (all D* Lite variants):')
    for c in CONFIGS:
        if c[1] == 'blind':
            continue
        for kind in ('activation', 'expiry', 'layer'):
            rr = [rp for r in mc[c] for rp in r['replans'] if rp['kind'] == kind]
            if not rr:
                continue
            sc = [rp['scratch'] for rp in rr if 'scratch' in rp]
            print(f"  {_cfg_name(c)} {kind:10s} n={len(rr):3d}  path changed {sum(rp['changed'] for rp in rr):3d}  "
                  f"mean expansions {np.mean([rp['exp'] for rp in rr]):7.1f}"
                  + (f" (from scratch {np.mean(sc):7.1f})" if sc else '')
                  + f"  remaining-cost change {np.mean([rp['d_cost'] for rp in rr]):+7.2f}"
                  f"  length change {np.mean([rp['d_len'] for rp in rr]):+6.2f} m")
    errs = [rp['g_err'] for c in CONFIGS for r in mc[c] for rp in r['replans'] if 'g_err' in rp]
    print(f'  max |g_incremental - g_scratch| at the replan start: {max(errs):.1e}')

    print('\nBypass decision log (3D D* Lite, every activation replan, per cell on the path):')
    print('  world  gamma seed     t   cell  z_drone  D*-choice     card-heuristic  z_bypass   cost    '
          'forced-horizontal(+diff, choice)    via-bypass-point(+diff)')
    for c in [('calm', '3d', GAMMA), ('calm', '3d', GAMMA_SENS), ('shear', '3d', GAMMA)]:
        for r in mc[c]:
            for d in r['decisions']:
                zb = f"{d['z_bypass']:5.1f}" if d['z_bypass'] is not None else '   - '
                cv = (f"{d['cost_v']:7.2f} ({_sgn(d['cost_v'] - d['cost'])})"
                      if d['cost_v'] < math.inf else '      -          ')
                print(f"  {c[0]:5s}  {c[2]:.1f}  {d['seed']:3d}  {d['t']:6.2f}  {d['cell']:3d}  "
                      f"{d['z_drone']:6.1f}   {d['choice']:11s}   {d['heur']:6s}        {zb}   "
                      f"{d['cost']:7.2f}  {d['cost_h']:7.2f} ({_sgn(d['cost_h'] - d['cost'])}, {d['choice_h']:11s})"
                      f"   {cv}")
        ds = [d for r in mc[c] for d in r['decisions']]
        ch = [d['choice'] for d in ds]
        agree = sum(1 for d in ds if d['choice'] in ('over', 'under') and d['choice'] == d['heur'])
        nvert = sum(1 for x in ch if x in ('over', 'under'))
        nv = [d['cost_v'] - d['cost'] for d in ds if d['cost_v'] < math.inf]
        print(f"  -> {c[0]} gamma={c[2]:.1f}: {len(ds)} decisions: "
              + ', '.join(f"{k} {ch.count(k)}" for k in CHOICES)
              + f"; card heuristic proposed a bypass in {sum(1 for d in ds if d['heur'] != 'none')}, "
              f"matched {agree} of {nvert} climb/dive choices; via-bypass-point extra cost "
              + (f"{min(nv):+.2f} ... {max(nv):+.2f}" if nv else 'n/a'))

    print('\nPer layout (flown cost / J_hazard s / z range m):')
    for world in WORLDS:
        for s in range(N_SEEDS):
            f_, t_ = mc[(world, 'flat', GAMMA)][s], mc[(world, '3d', GAMMA)][s]
            line = (f"  {world:5s} seed {s:2d}: flat {f_['flown']:7.2f} / {f_['J_hazard']:4.2f}   "
                    f"3D {t_['flown']:7.2f} / {t_['J_hazard']:4.2f} / {t_['zmin']:.0f}-{t_['zmax']:.0f}")
            if world == 'calm':
                g0 = mc[('calm', '3d', GAMMA_SENS)][s]
                line += f"   3D gamma=0 {g0['flown']:7.2f} / {g0['J_hazard']:4.2f} / {g0['zmin']:.0f}-{g0['zmax']:.0f}"
            print(line)

    # Showcase layout (rule): the first seed whose 3D shear flight has an activation replan
    shear3 = mc[('shear', '3d', GAMMA)]
    seed = next((r['seed'] for r in shear3 if any(rp['kind'] == 'activation' for rp in r['replans'])), 0)
    # calm-air showcase (rule): the first seed whose gamma = 0 flight leaves the 31 m layer
    calm0 = mc[('calm', '3d', GAMMA_SENS)]
    cseed = next((r['seed'] for r in calm0 if r['zmin'] < Z_CRUISE or r['zmax'] > Z_CRUISE), 0)
    show = {'seed': seed, 'calm_seed': cseed,
            'shear_3d': shear3[seed], 'shear_flat': mc[('shear', 'flat', GAMMA)][seed],
            'calm_3d': mc[('calm', '3d', GAMMA)][cseed], 'calm_3d_g0': calm0[cseed]}
    print(f'\nShowcase layouts: seed {seed} for the shear world (first seed with an activation replan '
          f'in the 3D flight), seed {cseed} for calm air (first seed whose gamma = 0 flight leaves 31 m)')
    for sd in sorted({seed, cseed}):
        print(f'  seed {sd} cells:')
        for k, cell in enumerate(make_cells(sd)):
            print(f'    cell {k}: centre ({cell.center[0]:5.1f}, {cell.center[1]:5.1f}, {cell.center[2]:4.1f}) '
                  f'semi-axes ({cell.axes[0]:4.1f}, {cell.axes[1]:4.1f}, {cell.axes[2]:4.1f}) m  '
                  f'w = {cell.severity:.2f}  active {cell.t_on:6.1f} ... {cell.t_off:6.1f} s')
    for key in ('shear_flat', 'shear_3d', 'calm_3d', 'calm_3d_g0'):
        r = show[key]
        rp = ', '.join(f"{x['t']:.2f} s {x['kind']} (inc {x['exp']}"
                       + (f", scratch {x['scratch']})" if 'scratch' in x else ')') for x in r['replans']) or 'none'
        print(f"  {key:10s} flown {r['flown']:7.2f}  L {r['L']:6.2f} m  T {r['T']:6.2f} s  "
              f"J_dxy {r['J_dxy']:6.2f}  J_dz {r['J_dz']:5.2f}  J_shear {r['J_shear']:6.2f}  "
              f"J_hazard {r['J_hazard']:.2f} s  z {r['zmin']:.0f}-{r['zmax']:.0f} m  replans: {rp}")

    plot_trajectory_3d(show, out_dir)
    plot_altitude_wind(show, out_dir)
    plot_replan_timeline(show, mc, out_dir)
    plot_cost_breakdown(mc, out_dir)
    save_animation(show, out_dir)


if __name__ == '__main__':
    main()
