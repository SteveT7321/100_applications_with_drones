"""
S051 3D Post-Disaster Comm Network
==================================
Five relay drones act as airborne cell towers over the S051 earthquake city
(2000 x 2000 m, 40 x 40 population grid, charge station at (1000, 100, 0) m).
Compared with the 2D S051 (all drones at 50 m, 300 m ground disc, 600 m radio
range, no obstacles) every drone now has its own altitude z_k in [20, 120] m:

  * ground coverage is a slant-range link budget (d_3D <= d_cov = 450 m), so the
    ground footprint shrinks as the drone climbs;
  * collapsed-building debris columns (h >= 10 m, height map 0-30 m) block the
    drone-to-survivor ray, so climbing removes debris shadow;
  * relay links are 3D (d <= d_relay = 900 m) and must clear the same columns;
  * hover power grows with altitude, P_hover(z) = P_0 (1 + kappa z).

Strategies (card), all on the same debris maps and SA random numbers:
  1. Flat 3D baseline  - the 2D S051 SA solution lifted to 50 m (card)
  2. Fixed-z SA (2D baseline, added) - S051 SA re-optimised in the 3D model with
                         every drone held at 50 m
  3. Altitude SA       - S051 SA with z_k added to the state
  4. Vertical daisy-chain - M relay-only drones stacked above the highest debris,
                         K - M drones by greedy (x, y, z) coverage

Usage:
    conda activate drones
    python src/03_environmental_sar/3d/s051_3d_post_disaster_comm.py
"""

import os
import math
import time
import importlib.util
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401
from scipy.ndimage import gaussian_filter

SCRIPT_DIR   = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, '..', '..', '..'))
OUTPUT_DIR   = os.path.join(PROJECT_ROOT, 'outputs', '03_environmental_sar', '3d',
                            's051_3d_post_disaster_comm')

_spec = importlib.util.spec_from_file_location(
    's051_2d', os.path.join(SCRIPT_DIR, '..', 's051_post_disaster_comm.py'))
s2d = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(s2d)

# -- Parameters --------------------------------------------------------------------
K_DRONES   = s2d.K_DRONES        # 5
AREA       = s2d.AREA_SIZE       # 2000 m
GRID_N     = s2d.GRID_N          # 40 (50 m cells)
CELL       = AREA / GRID_N
N_CELLS    = GRID_N * GRID_N
Z_MIN, Z_MAX = 20.0, 120.0       # m  altitude bounds (card)
Z_FLAT     = s2d.Z_HOVER         # 50 m  2D hover altitude

# Link budget (card values; see the d_cov / d_relay note in the README)
F_C        = 900e6               # Hz
C_LIGHT    = 299_792_458.0
P_TX_W     = 1.0                 # W
G_DBI      = 3.0                 # dBi, each antenna
P_MIN_DBM_CARD = -100.0          # card sensitivity
D_COV      = 450.0               # m  max coverage slant range (card "~450 m")
D_RELAY    = 900.0               # m  max relay range (card "~900 m")

# Debris (card: h in [0, 30] m, dense if h >= 10 m)
H_DEBRIS_MAX = 30.0
H_DENSE    = 10.0
N_RAY      = 50                  # LOS samples per ray (card)
DEBRIS_CORR_CELLS = 1.5          # added: spatial correlation of the collapse field (cells)
DEBRIS_SEED_BASE = 1000          # added
DEBRIS_SCALES = (0.0, 0.5, 1.0, 1.5)   # added: debris-height multiplier sweep (1.0 = main)

# Energy (card)
P_HOVER0   = 80.0                # W
KAPPA      = 4e-5                # 1/m
P_TRANSIT  = 80.0                # W  added: card gives none; = P_hover0
V_DRONE    = s2d.V_DRONE         # 15 m/s
E_MAX      = 144_000.0           # J  (= 80 W x 1800 s, S051 battery)
T_HOVER_MIN = s2d.T_HOVER        # 1500 s

# SA (2D S051 schedule; card alpha, T0, iterations, perturbation sizes)
ALPHA      = s2d.ALPHA           # 1e6 persons per bad drone
T0_SA      = s2d.T0_SA           # 500
T_END_SA   = s2d.T_END_SA        # 0.25 (2D fix of the card's beta = 0.995)
N_SA_STEPS = s2d.N_SA_STEPS      # 80 000
D_XY_MAX   = 200.0               # m  card delta_xy (scaled with T as in 2D)
D_Z_MAX    = 20.0                # m  card delta_z  (scaled with T)
LAT        = 25.0                # m  added: SA x, y lattice (half a cell)
D_Z_MIN    = 2.0                 # m  added: floor on the z step (2D uses a 10 m xy floor)
N_LAT      = int(round(AREA / LAT)) + 1        # 81 lattice points per axis
N_SEEDS    = 6                   # Monte Carlo debris maps / SA seeds
SA_SEED_BASE = 0
SWEEP_STEPS = 20_000             # SA iterations per sweep run (both sides equal)
Z_FIX_SWEEP = (20.0, 35.0, 50.0, 80.0, 120.0)
Z_LEVELS   = (20.0, 40.0, 60.0, 80.0, 100.0, 120.0)   # greedy / daisy-chain altitude levels
SHOW_SEED  = 0
N_WORKERS  = min(6, os.cpu_count() or 1)                  # process pool over seeds (results collected in seed order)
GIF_FRAMES_TARGET = 60
Z_FIX_HIGH = (150.0, 200.0, 250.0, 300.0)   # probe: uniform altitude above the 120 m ceiling
Z_MAX_PROBE = 300.0              # probe: altitude SA with the ceiling lifted
CORR_SWEEP = (0.5, 3.0)          # debris correlation-length sweep (cells), main = 1.5
N_LINK_PROBE = 2000              # random drone pairs per altitude for the relay-LOS probe

BASE3 = np.array([1000.0, 100.0, 0.0])
TAU_GROUND = np.arange(N_RAY) / N_RAY            # 0 .. 49/50, excludes the survivor end
TAU_LINK   = np.arange(1, N_RAY) / N_RAY         # interior samples of a relay link

STRATS = ['flat', 'fixed', 'alt', 'daisy']
LABELS = {'flat': 'Flat 3D (2D S051 SA lifted to 50 m)',
          'fixed': 'Fixed-z SA at 50 m (2D baseline)',
          'alt': 'Altitude SA',
          'daisy': 'Vertical daisy-chain'}
COLORS = {'flat': '#888888', 'fixed': '#E87722', 'alt': '#1F77B4', 'daisy': '#9467BD'}


# -- Model -----------------------------------------------------------------------------

def card_link_ranges():
    """Card formula d = lambda/(4 pi) sqrt(Ptx Gt Gr / Pmin) and the thresholds for 450/900 m."""
    lam = C_LIGHT / F_C
    g = 10 ** (G_DBI / 10)
    pmin_w = 10 ** ((P_MIN_DBM_CARD - 30) / 10)
    d_card = lam / (4 * np.pi) * np.sqrt(P_TX_W * g * g / pmin_w)

    def p_at(d):
        return 10 * np.log10(P_TX_W * g * g * (lam / (4 * np.pi * d)) ** 2) + 30
    return d_card, p_at(D_COV), p_at(D_RELAY)


def build_debris(density, seed, scale, corr=None):
    """Collapsed-building heights: building stock sqrt(density/max) x a correlated collapse field."""
    rng = np.random.default_rng(DEBRIS_SEED_BASE + seed)
    corr = DEBRIS_CORR_CELLS if corr is None else corr
    noise = gaussian_filter(rng.standard_normal((GRID_N, GRID_N)), corr,
                            mode='reflect').ravel()
    q = (np.argsort(np.argsort(noise, kind='stable'), kind='stable') + 0.5) / N_CELLS
    stock = np.sqrt(density / density.max())
    h = scale * H_DEBRIS_MAX * stock * q
    assert h.min() >= 0.0 and h.max() <= scale * H_DEBRIS_MAX + 1e-9
    return h


def cell_of(x, y):
    i = np.clip((np.asarray(x) // CELL).astype(int), 0, GRID_N - 1)
    j = np.clip((np.asarray(y) // CELL).astype(int), 0, GRID_N - 1)
    return j * GRID_N + i


class Geometry:
    """Debris-independent ray geometry: which grid cells each drone-to-cell ray crosses.

    SA positions live on a 25 m (x, y) lattice. For every lattice point and every ground
    cell within the largest possible footprint (horizontal range sqrt(d_cov^2 - z_min^2)),
    the cells under the N_RAY ray samples are stored once; the survivor's own cell is
    replaced by a sentinel (no column), so it never blocks its own ray.
    """

    def __init__(self, centroids):
        self.centroids = centroids
        g = np.arange(N_LAT) * LAT
        lx, ly = np.meshgrid(g, g)
        self.lat = np.column_stack([lx.ravel(), ly.ravel()])           # index iy*N_LAT + ix
        self.lat_cell = cell_of(self.lat[:, 0], self.lat[:, 1])
        n = len(self.lat)
        d2 = ((self.lat[:, None, :] - centroids[None, :, :]) ** 2).sum(-1)
        mask = d2 <= D_COV ** 2 - Z_MIN ** 2
        m = int(mask.sum(1).max())
        a_idx, u_idx = np.nonzero(mask)                      # grouped by a, u ascending
        slot = np.arange(len(a_idx)) - np.searchsorted(a_idx, a_idx)
        self.tidx = np.full((n, m), N_CELLS, dtype=np.int32)
        self.trho2 = np.full((n, m), np.inf)
        self.tidx[a_idx, slot] = u_idx
        self.trho2[a_idx, slot] = d2[a_idx, u_idx]
        self.a_idx, self.slot = a_idx, slot
        self.ray_cells = np.empty((len(a_idx), N_RAY), dtype=np.int16)
        for s0 in range(0, len(a_idx), 200_000):
            sl = slice(s0, s0 + 200_000)
            self.ray_cells[sl] = ray_cells(self.lat[a_idx[sl]], centroids[u_idx[sl]], u_idx[sl])


def ray_cells(P, C, u):
    """Cells under the N_RAY samples (tau = 0 .. 49/50) of the rays P[i] -> C[i]; own cell -> sentinel."""
    pts = P[:, None, :] * (1 - TAU_GROUND)[None, :, None] + C[:, None, :] * TAU_GROUND[None, :, None]
    cells = cell_of(pts[..., 0], pts[..., 1])
    return np.where(cells == u[:, None], N_CELLS, cells)


INV_1MT = 1.0 / (1.0 - TAU_GROUND)


class World:
    """Population, debris and the drone-to-cell clearance table for one debris map."""

    def __init__(self, geom, pop, h):
        self.geom, self.centroids, self.pop, self.h = geom, geom.centroids, pop, h
        self.lat, self.lat_cell = geom.lat, geom.lat_cell
        self.tidx, self.trho2 = geom.tidx, geom.trho2
        self.h_block = np.where(h >= H_DENSE, h, 0.0)
        self.hb_ext = np.append(self.h_block, 0.0)
        self.pop_ext = np.append(pop, 0.0)
        base_cell = int(cell_of(BASE3[0], BASE3[1]))
        assert self.h_block[base_cell] == 0.0, 'charge station inside a debris column'
        # The ray sample at tau sits at height (1 - tau) z and its horizontal track does
        # not depend on z, so the ray clears a column of height h iff z > h / (1 - tau).
        zc = np.empty(len(geom.a_idx))
        for s0 in range(0, len(zc), 200_000):
            sl = slice(s0, s0 + 200_000)
            zc[sl] = (self.hb_ext[geom.ray_cells[sl]] * INV_1MT[None, :]).max(axis=1)
        self.tzc = np.full(self.tidx.shape, np.inf)
        self.tzc[geom.a_idx, geom.slot] = zc

    def zclear_direct(self, p, u):
        """Same clearance altitude for an arbitrary (continuous) drone position p, by ray marching."""
        P = np.broadcast_to(p, (len(u), 2))
        cells = ray_cells(P, self.centroids[u], u)
        return (self.hb_ext[cells] * INV_1MT[None, :]).max(axis=1)

    def cover_idx(self, p, z):
        m = (self.trho2[p] + z * z <= D_COV ** 2) & (z > self.tzc[p])
        return self.tidx[p][m]

    def los_link(self, a, b):
        """Clear LOS between 3D points a (n,3) and b (n,3) at the interior samples."""
        pts = a[:, None, :] * (1 - TAU_LINK)[None, :, None] + b[:, None, :] * TAU_LINK[None, :, None]
        hb = self.h_block[cell_of(pts[..., 0], pts[..., 1])]
        return np.all(pts[..., 2] > hb, axis=1)


def energy_ok(xyz):
    L = np.sqrt((xyz[:, 0] - BASE3[0]) ** 2 + (xyz[:, 1] - BASE3[1]) ** 2 + xyz[:, 2] ** 2)
    t_max = (E_MAX - 2 * L * P_TRANSIT / V_DRONE) / (P_HOVER0 * (1 + KAPPA * xyz[:, 2]))
    return t_max >= T_HOVER_MIN


def link_matrix(world, xyz):
    nodes = np.vstack([BASE3, xyz])
    n = len(nodes)
    ii, jj = np.triu_indices(n, 1)
    d = np.linalg.norm(nodes[ii] - nodes[jj], axis=1)
    ok = (d <= D_RELAY) & world.los_link(nodes[ii], nodes[jj])
    A = np.zeros((n, n), dtype=bool)
    A[ii, jj] = ok
    A[jj, ii] = ok
    return A


def reach_from_base(A):
    r = np.zeros(len(A), dtype=bool)
    r[0] = True
    for _ in range(len(A)):
        r = r | (A[r].any(axis=0))
    return r


def evaluate(world, xyz, relay_only=(), z_max=Z_MAX):
    """Independent evaluation by direct ray marching (no lookup table)."""
    xyz = np.asarray(xyz, float)
    assert np.all((xyz[:, 2] >= Z_MIN - 1e-9) & (xyz[:, 2] <= z_max + 1e-9)), 'altitude bound'
    assert np.all(xyz[:, :2] >= 0) and np.all(xyz[:, :2] <= AREA), 'outside area'
    assert np.all(xyz[:, 2] > world.h[cell_of(xyz[:, 0], xyz[:, 1])]), 'drone inside debris'
    covered = np.zeros(N_CELLS, dtype=bool)
    in_range = np.zeros(N_CELLS, dtype=bool)
    clear_cnt = np.zeros(N_CELLS, dtype=int)
    u = np.arange(N_CELLS)
    for k, p in enumerate(xyz):
        zc = world.zclear_direct(p[:2], u)
        rho2 = ((world.centroids - p[:2]) ** 2).sum(1)
        rng_ok = rho2 + p[2] ** 2 <= D_COV ** 2
        los = p[2] > zc
        clear_cnt += los
        if k in relay_only:
            continue
        in_range |= rng_ok
        covered |= rng_ok & los
    A = link_matrix(world, xyz)
    conn = reach_from_base(A)[1:]
    e_ok = energy_ok(xyz)
    total = world.pop.sum()
    return {'xyz': xyz.copy(), 'cov': world.pop[covered].sum() / total,
            'in_range': world.pop[in_range].sum() / total,
            'shadowed': world.pop[in_range & ~covered].sum() / total,
            'covered_mask': covered, 'clear_cnt': clear_cnt, 'A': A,
            'n_conn': int(conn.sum()), 'conn': conn, 'n_energy_bad': int((~e_ok).sum()),
            'feasible': bool(conn.all() and e_ok.all()), 'relay_only': tuple(relay_only)}


# -- Strategies -------------------------------------------------------------------------

def _energy_ok1(x, y, z):
    L = math.sqrt((x - BASE3[0]) ** 2 + (y - BASE3[1]) ** 2 + z * z)
    return (E_MAX - 2 * L * P_TRANSIT / V_DRONE) / (P_HOVER0 * (1 + KAPPA * z)) >= T_HOVER_MIN


def _links_of(world, nodes, k):
    """Bitmask of nodes linked to node k (3D range and debris LOS at the interior samples).

    Fast version of link_matrix() for one node, used inside the SA loop.
    """
    a = nodes[k]
    dv = nodes - a
    d = np.sqrt((dv * dv).sum(1))
    X = a[0] + TAU_LINK[None, :] * dv[:, 0:1]
    Y = a[1] + TAU_LINK[None, :] * dv[:, 1:2]
    Z = a[2] + TAU_LINK[None, :] * dv[:, 2:3]
    ix = np.minimum((X * (1.0 / CELL)).astype(np.intp), GRID_N - 1)
    iy = np.minimum((Y * (1.0 / CELL)).astype(np.intp), GRID_N - 1)
    ok = (d <= D_RELAY) & (Z > world.h_block[iy * GRID_N + ix]).all(axis=1)
    ok[k] = False
    r = 0
    for i in range(len(ok)):
        if ok[i]:
            r |= 1 << i
    return r


def _n_reached(adj):
    r = 1
    while True:
        nr = r
        for i, a in enumerate(adj):
            if (r >> i) & 1:
                nr |= a
        if nr == r:
            return bin(r).count('1')
        r = nr


def simulated_annealing(world, seed, n_steps, free_z, z_fix=Z_FLAT, n_frames=0, z_max=Z_MAX):
    """S051 SA on a 25 m (x, y) lattice; z continuous (free_z) or held at z_fix.

    E = -C + ALPHA * (n_disconnected + n_energy_infeasible), C in persons. Both variants
    draw the same random numbers every step (drone index, 3 uniforms, acceptance draw);
    the fixed-z run ignores the z component. Proposals that put a drone inside a debris
    column (z <= h) are rejected.
    """
    rng = np.random.default_rng(SA_SEED_BASE + seed)
    total = world.pop.sum()
    pop_ext, h, lat, lat_cell = world.pop_ext, world.h, world.lat, world.lat_cell
    beta = (T_END_SA / T0_SA) ** (1.0 / n_steps)
    open_lat = np.where(world.h_block[lat_cell] == 0.0)[0]
    p = [int(v) for v in rng.choice(open_lat, size=K_DRONES, replace=False)]
    z = [Z_FLAT if free_z else float(z_fix)] * K_DRONES
    nodes = np.vstack([BASE3, np.column_stack([lat[p], z])])
    cur = [world.cover_idx(p[k], z[k]) for k in range(K_DRONES)]
    cnt = np.zeros(N_CELLS + 1, dtype=np.int64)
    for c in cur:
        cnt[c] += 1
    cov = float(pop_ext[:N_CELLS][cnt[:N_CELLS] > 0].sum())
    adj = [0] * (K_DRONES + 1)
    for k in range(K_DRONES + 1):
        adj[k] = _links_of(world, nodes, k)
    e_ok = [_energy_ok1(*nodes[k + 1]) for k in range(K_DRONES)]
    n_disc = K_DRONES + 1 - _n_reached(adj)
    n_en = e_ok.count(False)
    E = -cov + ALPHA * (n_disc + n_en)
    best = (E, list(p), list(z))
    hist = {key: np.empty(n_steps) for key in ('E', 'cov', 'n_disc', 'n_en', 'zmean')}
    frame_every = max(1, n_steps // n_frames) if n_frames else 0
    frames = []
    T = T0_SA
    for step in range(n_steps):
        k = int(rng.integers(K_DRONES))
        uu = rng.uniform(-1.0, 1.0, size=3)
        r_acc = rng.random()
        dxy = max(D_XY_MAX * T / T0_SA, LAT)
        dz = max(D_Z_MAX * T / T0_SA, D_Z_MIN)
        ix, iy = p[k] % N_LAT, p[k] // N_LAT
        ix2 = min(max(ix + math.floor(uu[0] * dxy / LAT + 0.5), 0), N_LAT - 1)
        iy2 = min(max(iy + math.floor(uu[1] * dxy / LAT + 0.5), 0), N_LAT - 1)
        p2 = iy2 * N_LAT + ix2
        z2 = min(max(z[k] + uu[2] * dz, Z_MIN), z_max) if free_z else z[k]
        if z2 > h[lat_cell[p2]]:
            old, new = cur[k], world.cover_idx(p2, z2)
            cnt[old] -= 1
            lost = pop_ext[old[cnt[old] == 0]].sum()
            gained = pop_ext[new[cnt[new] == 0]].sum()
            cov2 = cov - lost + gained
            nodes2 = nodes.copy()
            nodes2[k + 1] = (lat[p2, 0], lat[p2, 1], z2)
            lk = _links_of(world, nodes2, k + 1)
            bit = 1 << (k + 1)
            adj2 = [(a | bit) if (lk >> i) & 1 else (a & ~bit) for i, a in enumerate(adj)]
            adj2[k + 1] = lk
            e2 = _energy_ok1(*nodes2[k + 1])
            nd2 = K_DRONES + 1 - _n_reached(adj2)
            ne2 = n_en - (0 if e_ok[k] else 1) + (0 if e2 else 1)
            E2 = -cov2 + ALPHA * (nd2 + ne2)
            dE = E2 - E
            if dE <= 0 or r_acc < math.exp(-dE / T):
                cnt[new] += 1
                cur[k], p[k], z[k] = new, p2, z2
                nodes, adj, cov, n_disc, n_en, E = nodes2, adj2, cov2, nd2, ne2, E2
                e_ok[k] = e2
                if E < best[0]:
                    best = (E, list(p), list(z))
            else:
                cnt[old] += 1
        hist['E'][step], hist['cov'][step] = E, cov / total
        hist['n_disc'][step], hist['n_en'][step] = n_disc, n_en
        hist['zmean'][step] = sum(z) / K_DRONES
        if frame_every and step % frame_every == 0:
            frames.append((step, nodes[1:].copy(), cov / total, n_disc + n_en))
        T *= beta
    assert np.all(cnt >= 0) and np.all(cnt <= K_DRONES)
    xyz_best = np.column_stack([lat[best[1]], best[2]])
    if n_frames:
        frames.append((n_steps, xyz_best, -best[0] / total if best[0] < 0 else 0.0, 0))
    return xyz_best, -best[0] / total, hist, frames


def greedy_xyz(world, n):
    """Greedy max-marginal coverage over (cell centroid, altitude level) candidates.

    Returns the n picks in order; the first n - M picks are the greedy solution for
    n - M drones. Sums use numpy reductions (no BLAS) so the result does not depend on
    the thread count.
    """
    ix = np.rint(world.centroids[:, 0] / LAT).astype(int)
    iy = np.rint(world.centroids[:, 1] / LAT).astype(int)
    lat_idx = iy * N_LAT + ix
    cand, cover = [], []
    for zl in Z_LEVELS:
        for c, pl in enumerate(lat_idx):
            if zl <= world.h[c]:
                continue
            row = np.zeros(N_CELLS + 1, dtype=bool)
            row[world.cover_idx(pl, zl)] = True
            cand.append((world.lat[pl][0], world.lat[pl][1], zl))
            cover.append(row[:N_CELLS])
    cover = np.array(cover)
    remaining = world.pop.copy()
    out = []
    for _ in range(n):
        gains = np.where(cover, remaining[None, :], 0.0).sum(axis=1)
        i = int(np.argmax(gains))                      # first index on ties
        out.append(cand[i])
        remaining[cover[i]] = 0.0
    return np.array(out).reshape(-1, 3)


def daisy_chain(world):
    """Card heuristic: M relay-only drones stacked above the highest debris cell,
    K - M drones by greedy (x, y, z) coverage; M* = best coverage among connected layouts."""
    c_star = int(np.argmax(world.h))
    xs, ys = world.centroids[c_star]
    h_max = float(world.h[c_star])
    picks = greedy_xyz(world, K_DRONES)
    rows = []
    for M in range(K_DRONES):
        free = picks[:K_DRONES - M]
        if M:
            dz = (Z_MAX - h_max) / M
            stack = np.array([[xs, ys, h_max + m * dz] for m in range(1, M + 1)])
            xyz = np.vstack([stack, free])
        else:
            xyz = free
        rows.append((M, evaluate(world, xyz, relay_only=tuple(range(M)))))
    feas = [(M, ev) for M, ev in rows if ev['feasible']]
    pool = feas if feas else rows
    M_best, ev_best = max(pool, key=lambda t: (t[1]['cov'], -t[0]))
    return M_best, ev_best, rows, (float(xs), float(ys), h_max)


def n_blocked_links(world, xyz):
    """Pairs within d_relay whose LOS is blocked by debris (base included)."""
    nodes = np.vstack([BASE3, xyz])
    ii, jj = np.triu_indices(len(nodes), 1)
    d = np.linalg.norm(nodes[ii] - nodes[jj], axis=1)
    los = world.los_link(nodes[ii], nodes[jj])
    return int(np.sum((d <= D_RELAY) & ~los))


def connected_from(hist):
    bad = np.where(hist['n_disc'] + hist['n_en'] > 0)[0]
    return int(bad[-1] + 1) if len(bad) else 0


# -- Simulation -------------------------------------------------------------------------

_G = {}


def _init_worker(flat_xyz=None):
    centroids, pop = s2d.build_population_map()
    _G['centroids'], _G['pop'] = centroids, pop
    _G['geom'] = Geometry(centroids)
    _G['flat_xyz'] = flat_xyz


def _slim(ev):
    return {k: v for k, v in ev.items() if k not in ('A', 'clear_cnt')}


def _run_seed(seed):
    """All strategies and sweeps for one debris map / SA seed (common random numbers)."""
    pop, geom = _G['pop'], _G['geom']
    density = pop / s2d.CELL_AREA_KM2
    worlds = {sc: World(geom, pop, build_debris(density, seed, sc)) for sc in DEBRIS_SCALES}
    w = worlds[1.0]
    out = {'seed': seed, 'h': w.h.copy()}
    out['flat'] = evaluate(w, _G['flat_xyz'])
    nf = GIF_FRAMES_TARGET if seed == SHOW_SEED else 0
    xf, cf, hf, _ = simulated_annealing(w, seed, N_SA_STEPS, free_z=False)
    xa, ca, ha, fr = simulated_annealing(w, seed, N_SA_STEPS, free_z=True, n_frames=nf)
    out['fixed'], out['alt'] = evaluate(w, xf), evaluate(w, xa)
    # incremental SA coverage must equal direct ray marching (catches table / drift bugs)
    assert abs(out['fixed']['cov'] - cf) < 1e-9 and abs(out['alt']['cov'] - ca) < 1e-9
    out['conn_iter'] = {'fixed': connected_from(hf), 'alt': connected_from(ha)}
    out['pen'] = {'fixed': (hf['n_disc'] + hf['n_en']).astype(np.int8),
                  'alt': (ha['n_disc'] + ha['n_en']).astype(np.int8)}
    M_best, ev_d, rows, info = daisy_chain(w)
    ev_d['M'] = M_best
    out['daisy'], out['daisy_info'] = ev_d, info
    out['daisy_rows'] = [(M, _slim(ev)) for M, ev in rows]
    out['blocked'] = {s: n_blocked_links(w, out[s]['xyz']) for s in ('flat', 'fixed', 'alt')}
    if seed == SHOW_SEED:
        out['hist_f'], out['hist_a'], out['frames'] = hf, ha, fr
    out['dsweep'] = {}
    for sc in DEBRIS_SCALES:
        ws = worlds[sc]
        x1, _, _, _ = simulated_annealing(ws, seed, SWEEP_STEPS, free_z=False)
        x2, _, _, _ = simulated_annealing(ws, seed, SWEEP_STEPS, free_z=True)
        out['dsweep'][sc] = {'fixed': _slim(evaluate(ws, x1)), 'alt': _slim(evaluate(ws, x2)),
                             'dense_frac': float((ws.h >= H_DENSE).mean()),
                             'pop_dense': float(pop[ws.h >= H_DENSE].sum() / pop.sum())}
    out['zsweep'] = {}
    for zf in Z_FIX_SWEEP:
        if zf == Z_FLAT:
            out['zsweep'][zf] = out['dsweep'][1.0]['fixed']
            continue
        x1, _, _, _ = simulated_annealing(w, seed, SWEEP_STEPS, free_z=False, z_fix=zf)
        out['zsweep'][zf] = _slim(evaluate(w, x1))
    out['blocked_z20'] = n_blocked_links(w, out['zsweep'][20.0]['xyz'])
    # probes above the card's 120 m ceiling (fixed uniform altitude and altitude SA)
    out['zhigh'] = {}
    for zf in Z_FIX_HIGH:
        x1, _, _, _ = simulated_annealing(w, seed, SWEEP_STEPS, free_z=False, z_fix=zf)
        out['zhigh'][zf] = _slim(evaluate(w, x1, z_max=zf))
    x1, _, _, _ = simulated_annealing(w, seed, SWEEP_STEPS, free_z=True, z_max=Z_MAX_PROBE)
    out['alt_high'] = _slim(evaluate(w, x1, z_max=Z_MAX_PROBE))
    # debris correlation-length sweep
    out['corr'] = {}
    for cr in CORR_SWEEP:
        wc = World(geom, pop, build_debris(density, seed, 1.0, corr=cr))
        x1, _, _, _ = simulated_annealing(wc, seed, SWEEP_STEPS, free_z=False)
        x2, _, _, _ = simulated_annealing(wc, seed, SWEEP_STEPS, free_z=True)
        out['corr'][cr] = {'fixed': _slim(evaluate(wc, x1)), 'alt': _slim(evaluate(wc, x2)),
                           'dense_frac': float((wc.h >= H_DENSE).mean())}
    # relay-LOS probe: random drone pairs within d_relay at a common altitude
    rng = np.random.default_rng(5000 + seed)
    out['link_probe'] = {}
    for zp in (Z_MIN, Z_FLAT):
        a = np.column_stack([rng.uniform(0, AREA, (N_LINK_PROBE, 2)), np.full(N_LINK_PROBE, zp)])
        ang = rng.uniform(0, 2 * np.pi, N_LINK_PROBE)
        r = rng.uniform(50.0, D_RELAY, N_LINK_PROBE)
        b = a + np.column_stack([r * np.cos(ang), r * np.sin(ang), np.zeros(N_LINK_PROBE)])
        ok = np.all((b[:, :2] >= 0) & (b[:, :2] <= AREA), axis=1)
        ok &= (a[:, 2] > w.h[cell_of(a[:, 0], a[:, 1])]) & (b[:, 2] > w.h[cell_of(b[:, 0], b[:, 1])])
        blocked = ~w.los_link(a[ok], b[ok])
        out['link_probe'][zp] = (int(ok.sum()), int(blocked.sum()))
    return out


def run_simulation(n_workers=None):
    t0 = time.time()
    n_workers = N_WORKERS if n_workers is None else n_workers
    centroids, pop = s2d.build_population_map()
    total = pop.sum()
    # Card baseline: 2D S051 SA solution (300 m disc, 600 m range), lifted to 50 m
    sa2d_pos, _, _ = s2d.simulated_annealing(centroids, pop)
    sa2d_cov2d = s2d.coverage(sa2d_pos, centroids, pop) / total
    flat_xyz = np.column_stack([sa2d_pos, np.full(K_DRONES, Z_FLAT)])
    seeds = list(range(N_SEEDS))
    if n_workers > 1:
        import multiprocessing as mp
        with mp.get_context('spawn').Pool(n_workers, initializer=_init_worker,
                                          initargs=(flat_xyz,)) as pool:
            per_seed = pool.map(_run_seed, seeds, chunksize=1)   # returned in seed order
        _init_worker(flat_xyz)
    else:
        _init_worker(flat_xyz)
        per_seed = [_run_seed(sd) for sd in seeds]
    w_show = World(_G['geom'], pop, build_debris(pop / s2d.CELL_AREA_KM2, SHOW_SEED, 1.0))
    return {'centroids': centroids, 'pop': pop, 'total': total, 'sa2d_pos': sa2d_pos,
            'sa2d_cov2d': sa2d_cov2d, 'flat_xyz': flat_xyz, 'per_seed': per_seed,
            'world': w_show, 'runtime_sim': time.time() - t0}


# -- Report ----------------------------------------------------------------------------

def _pct(v):
    return 100.0 * np.asarray(v, float)


def report(res):
    P = res['per_seed']
    total = res['total']
    d_card, p450, p900 = card_link_ranges()
    print('=== S051 3D Post-Disaster Comm Network ===')
    print(f'Population: {total:,.0f} persons (2D S051 map); {K_DRONES} drones; base '
          f'({BASE3[0]:.0f}, {BASE3[1]:.0f}, {BASE3[2]:.0f}) m')
    print(f'Link budget: card formula with P_min = {P_MIN_DBM_CARD:.0f} dBm gives d = {d_card / 1000:.1f} km; '
          f'received power at {D_COV:.0f} m / {D_RELAY:.0f} m = {p450:.1f} / {p900:.1f} dBm '
          f'(thresholds used for the card ranges)')
    for zz in (Z_MIN, Z_FLAT, Z_MAX, Z_MAX_PROBE):
        print(f'  ground footprint radius at z = {zz:.0f} m: {math.sqrt(D_COV ** 2 - zz ** 2):.1f} m')
    corners = np.array([[x, y] for x in (0.0, AREA) for y in (0.0, AREA)])
    far = float(np.max(np.linalg.norm(corners - BASE3[:2], axis=1)))
    for zz in (Z_MIN, Z_MAX, Z_MAX_PROBE):
        lmax = (E_MAX - T_HOVER_MIN * P_HOVER0 * (1 + KAPPA * zz)) * V_DRONE / (2 * P_TRANSIT)
        print(f'  energy: max 3D transit distance at z = {zz:.0f} m: {lmax:.0f} m; '
              f'farthest corner at that altitude: {math.hypot(far, zz):.0f} m')
    print(f'  hover power at 20 / 120 m: {P_HOVER0 * (1 + KAPPA * Z_MIN):.3f} / '
          f'{P_HOVER0 * (1 + KAPPA * Z_MAX):.3f} W')
    beta = (T_END_SA / T0_SA) ** (1.0 / N_SA_STEPS)
    print(f'SA: {N_SA_STEPS} iterations, beta = {beta:.6f}, xy lattice {LAT:.0f} m, '
          f'delta_xy {D_XY_MAX:.0f}->{LAT:.0f} m, delta_z {D_Z_MAX:.0f}->{D_Z_MIN:.0f} m; '
          f'{N_SEEDS} seeds (debris map + SA random numbers per seed)')
    print(f'2D S051 SA solution: coverage in the 2D model (300 m disc) = {100 * res["sa2d_cov2d"]:.2f}%; '
          'positions ' + ', '.join(f'({p[0]:.0f},{p[1]:.0f})' for p in res['sa2d_pos']))

    print('\nDebris maps (scale 1.0):')
    for o in P:
        h = o['h']
        print(f'  seed {o["seed"]}: dense cells {100 * np.mean(h >= H_DENSE):.1f}%, population in dense '
              f'cells {100 * res["pop"][h >= H_DENSE].sum() / total:.1f}%, max height {h.max():.1f} m')

    print('\nCoverage per seed (% of population; * = disconnected or energy-infeasible):')
    print('  seed |  flat  | fixed50 |  alt   | daisy (M*) | alt-fixed | alt-flat')
    for o in P:
        cells = []
        for s_ in STRATS:
            mark = '' if o[s_]['feasible'] else '*'
            cells.append(f'{100 * o[s_]["cov"]:.2f}{mark}')
        print(f'  {o["seed"]:4d} | {cells[0]:>6} | {cells[1]:>7} | {cells[2]:>6} | '
              f'{cells[3]:>6} ({o["daisy"]["M"]}) | {100 * (o["alt"]["cov"] - o["fixed"]["cov"]):+.2f} pp | '
              f'{100 * (o["alt"]["cov"] - o["flat"]["cov"]):+.2f} pp')
    for s_ in STRATS:
        v = _pct([o[s_]['cov'] for o in P])
        nf = sum(o[s_]['feasible'] for o in P)
        fv = np.array([100 * o[s_]['cov'] for o in P if o[s_]['feasible']])
        print(f'  {LABELS[s_]}: mean {v.mean():.2f}%, std {v.std():.2f}, range {v.min():.2f}-{v.max():.2f}%, '
              f'feasible {nf}/{len(P)} (mean over feasible seeds {fv.mean():.2f}%)')
    d_af = _pct([o['alt']['cov'] - o['fixed']['cov'] for o in P])
    print(f'  altitude SA - fixed-z SA (50 m): mean {d_af.mean():+.2f} pp, range {d_af.min():+.2f} to '
          f'{d_af.max():+.2f} pp; alt better in {int(np.sum(d_af > 0))}/{len(P)}, '
          f'tie {int(np.sum(d_af == 0))}, worse {int(np.sum(d_af < 0))}')
    d_ff = _pct([o['fixed']['cov'] - o['flat']['cov'] for o in P])
    print(f'  fixed-z SA - flat (2D solution lifted): mean {d_ff.mean():+.2f} pp, range '
          f'{d_ff.min():+.2f} to {d_ff.max():+.2f} pp')
    fe = [o for o in P if o['daisy']['feasible']]
    d_da = _pct([o['alt']['cov'] - o['daisy']['cov'] for o in fe])
    print(f'  altitude SA - daisy-chain (feasible seeds only, {len(fe)}): mean {d_da.mean():+.2f} pp, '
          f'range {d_da.min():+.2f} to {d_da.max():+.2f} pp; alt better in {int(np.sum(d_da > 0))}/{len(fe)}')

    print('\nShadow decomposition (mean over seeds, % of population):')
    for s_ in ('flat', 'fixed', 'alt'):
        ir = _pct([o[s_]['in_range'] for o in P]).mean()
        cv = _pct([o[s_]['cov'] for o in P]).mean()
        sh = _pct([o[s_]['shadowed'] for o in P]).mean()
        print(f'  {LABELS[s_]}: within slant range {ir:.2f}%, covered {cv:.2f}%, '
              f'in range but debris-shadowed {sh:.2f}%')

    o_s = P[SHOW_SEED]
    print(f'  seed {SHOW_SEED}: mean fraction of the 5 drones with clear LOS to a cell (range ignored): '
          f'flat {np.mean(o_s["flat"]["clear_cnt"]) / K_DRONES:.2f}, fixed {np.mean(o_s["fixed"]["clear_cnt"]) / K_DRONES:.2f}, '
          f'alt {np.mean(o_s["alt"]["clear_cnt"]) / K_DRONES:.2f}')

    print('\nAltitudes chosen by altitude SA:')
    o0 = P[SHOW_SEED]
    for k, p in enumerate(o0['alt']['xyz']):
        print(f'  seed {SHOW_SEED} D{k + 1}: ({p[0]:.0f}, {p[1]:.0f}, {p[2]:.1f}) m')
    zall = np.concatenate([o['alt']['xyz'][:, 2] for o in P])
    print(f'  all seeds: mean z {zall.mean():.1f} m, min {zall.min():.1f} m, '
          f'{int(np.sum(zall >= Z_MAX - 5))}/{len(zall)} drones within 5 m of the {Z_MAX:.0f} m ceiling')
    print('  SA constraint-free (connected, energy-feasible) from iteration, per seed: fixed '
          + ', '.join(str(o['conn_iter']['fixed']) for o in P) + '; alt '
          + ', '.join(str(o['conn_iter']['alt']) for o in P))
    for s_ in ('fixed', 'alt'):
        v80 = _pct([o[s_]['cov'] for o in P])
        v20 = _pct([o['dsweep'][1.0][s_]['cov'] for o in P])
        dd = v80 - v20
        print(f'  SA optimiser noise, {s_}: {N_SA_STEPS} it. minus {SWEEP_STEPS} it. (same seed): mean '
              f'{dd.mean():+.2f} pp, range {dd.min():+.2f} to {dd.max():+.2f} pp')

    print(f'\nUniform-altitude sweep (fixed-z SA, {SWEEP_STEPS} iterations, same seeds):')
    zs_all = list(Z_FIX_SWEEP) + list(Z_FIX_HIGH)
    for zf in zs_all:
        src = 'zsweep' if zf in Z_FIX_SWEEP else 'zhigh'
        v = _pct([o[src][zf]['cov'] for o in P])
        nf = sum(o[src][zf]['feasible'] for o in P)
        print(f'  z = {zf:5.0f} m: mean {v.mean():.2f}%, range {v.min():.2f}-{v.max():.2f}%, '
              f'feasible {nf}/{len(P)}')
    va = _pct([o['dsweep'][1.0]['alt']['cov'] for o in P])
    v120 = _pct([o['zsweep'][Z_MAX]['cov'] for o in P])
    print(f'  altitude SA (z <= {Z_MAX:.0f} m, {SWEEP_STEPS} it.): mean {va.mean():.2f}%; '
          f'minus uniform {Z_MAX:.0f} m: mean {np.mean(va - v120):+.2f} pp, range '
          f'{np.min(va - v120):+.2f} to {np.max(va - v120):+.2f} pp, better in '
          f'{int(np.sum(va > v120))}/{len(P)}, tie {int(np.sum(va == v120))}')
    vh = _pct([o['alt_high']['cov'] for o in P])
    zh = np.concatenate([o['alt_high']['xyz'][:, 2] for o in P])
    best_u = np.array([max(_pct(o['zhigh'][zf]['cov']) if o['zhigh'][zf]['feasible'] else -1
                           for zf in Z_FIX_HIGH) for o in P])
    print(f'  probe, ceiling lifted to {Z_MAX_PROBE:.0f} m: altitude SA mean {vh.mean():.2f}% '
          f'(range {vh.min():.2f}-{vh.max():.2f}%), feasible {sum(o["alt_high"]["feasible"] for o in P)}/'
          f'{len(P)}; drone z mean {zh.mean():.1f} m, range {zh.min():.1f}-{zh.max():.1f} m; '
          f'gain over altitude SA capped at {Z_MAX:.0f} m: mean {np.mean(vh - va):+.2f} pp')
    spread = np.array([np.ptp(o['alt_high']['xyz'][:, 2]) for o in P])
    print(f'  probe: per-seed spread (max - min) of the 5 drone altitudes: '
          + ', '.join(f'{v:.1f}' for v in spread) + ' m')
    print(f'  probe: lifted-ceiling altitude SA minus best feasible uniform altitude in '
          f'{{150..300}} m: mean {np.mean(vh - best_u):+.2f} pp, better in {int(np.sum(vh > best_u))}/{len(P)}')

    print(f'\nDebris-height scale sweep ({SWEEP_STEPS} iterations; scale 0 = no debris):')
    for sc in DEBRIS_SCALES:
        vf = _pct([o['dsweep'][sc]['fixed']['cov'] for o in P])
        va_ = _pct([o['dsweep'][sc]['alt']['cov'] for o in P])
        zm = np.array([o['dsweep'][sc]['alt']['xyz'][:, 2].mean() for o in P])
        df = np.mean([o['dsweep'][sc]['dense_frac'] for o in P])
        dd = va_ - vf
        print(f'  scale {sc:.1f}: dense cells {100 * df:.1f}%, fixed-50 {vf.mean():.2f}%, alt {va_.mean():.2f}%, '
              f'alt-fixed mean {dd.mean():+.2f} pp (range {dd.min():+.2f} to {dd.max():+.2f}, better '
              f'{int(np.sum(dd > 0))}/{len(P)}), alt mean z {zm.mean():.1f} m (range {zm.min():.1f}-{zm.max():.1f})')
    print(f'Debris correlation-length sweep (scale 1.0, {SWEEP_STEPS} iterations; main = {DEBRIS_CORR_CELLS} cells):')
    for cr in CORR_SWEEP:
        vf = _pct([o['corr'][cr]['fixed']['cov'] for o in P])
        va_ = _pct([o['corr'][cr]['alt']['cov'] for o in P])
        zm = np.array([o['corr'][cr]['alt']['xyz'][:, 2].mean() for o in P])
        dd = va_ - vf
        print(f'  corr {cr:.1f} cells: dense {100 * np.mean([o["corr"][cr]["dense_frac"] for o in P]):.1f}%, '
              f'fixed-50 {vf.mean():.2f}%, alt {va_.mean():.2f}%, alt-fixed mean {dd.mean():+.2f} pp '
              f'(better {int(np.sum(dd > 0))}/{len(P)}), alt mean z {zm.mean():.1f} m')

    print('\nVertical daisy-chain:')
    for o in P:
        xs, ys, hm = o['daisy_info']
        rows = ', '.join(f'M={M}: {100 * ev["cov"]:.2f}%{"" if ev["feasible"] else "*"}'
                         for M, ev in o['daisy_rows'])
        print(f'  seed {o["seed"]}: stack at ({xs:.0f}, {ys:.0f}) over h = {hm:.1f} m; {rows}; M* = {o["daisy"]["M"]}')
    zg = np.concatenate([o['daisy_rows'][0][1]['xyz'][:, 2] for o in P])
    print(f'  greedy (M = 0) altitude levels chosen: ' + ', '.join(f'{v:.0f}' for v in zg))
    nb = {s_: sum(o['blocked'][s_] for o in P) for s_ in ('flat', 'fixed', 'alt')}
    print(f'  debris-blocked links within {D_RELAY:.0f} m in the final layouts (all seeds): '
          f'flat {nb["flat"]}, fixed {nb["fixed"]}, alt {nb["alt"]}, uniform-20 m SA '
          f'{sum(o["blocked_z20"] for o in P)}')
    for zp in (Z_MIN, Z_FLAT):
        n_ok = sum(o['link_probe'][zp][0] for o in P)
        n_bl = sum(o['link_probe'][zp][1] for o in P)
        print(f'  random drone pairs at z = {zp:.0f} m, 50-{D_RELAY:.0f} m apart: {n_bl}/{n_ok} '
              f'blocked by debris ({100 * n_bl / n_ok:.2f}%)')


# -- Plotting --------------------------------------------------------------------------

def _img(v):
    return np.asarray(v).reshape(GRID_N, GRID_N)


def _pillars(ax, h, alpha=0.25):
    dense = np.where(h >= H_DENSE)[0]
    xs = (dense % GRID_N) * CELL
    ys = (dense // GRID_N) * CELL
    ax.bar3d(xs, ys, np.zeros(len(dense)), CELL, CELL, h[dense], color='gray', alpha=alpha,
             shade=True, linewidth=0, zorder=1)


def _draw_links3d(ax, A, xyz, color='k'):
    nodes = np.vstack([BASE3, xyz])
    ii, jj = np.nonzero(np.triu(A))
    for i, j in zip(ii, jj):
        ax.plot(*zip(nodes[i], nodes[j]), '--', color=color, lw=1.0, zorder=4)


def plot_network_3d(res, out):
    w, o = res['world'], res['per_seed'][SHOW_SEED]
    fig = plt.figure(figsize=(11, 8.5))
    ax = fig.add_subplot(111, projection='3d', computed_zorder=False)
    _pillars(ax, w.h)
    xf, xa = o['fixed']['xyz'], o['alt']['xyz']
    _draw_links3d(ax, link_matrix(w, xf), xf, COLORS['fixed'])
    _draw_links3d(ax, link_matrix(w, xa), xa, 'k')
    ax.scatter(xf[:, 0], xf[:, 1], xf[:, 2], color=COLORS['fixed'], s=60, edgecolor='k',
               depthshade=False, label=f'fixed-z SA, z = 50 m ({100 * o["fixed"]["cov"]:.1f}%)', zorder=5)
    sc = ax.scatter(xa[:, 0], xa[:, 1], xa[:, 2], c=xa[:, 2], cmap='viridis', vmin=Z_MIN, vmax=Z_MAX,
                    s=110, edgecolor='k', depthshade=False,
                    label=f'altitude SA ({100 * o["alt"]["cov"]:.1f}%)', zorder=6)
    for k, p in enumerate(xa):
        ax.plot([p[0]] * 2, [p[1]] * 2, [0, p[2]], ':', color='gray', lw=0.8)
        ax.text(p[0], p[1], p[2] + 6, f'D{k + 1}', fontsize=8)
    ax.scatter(*BASE3, marker='*', color='k', s=250, depthshade=False, label='charge station')
    fig.colorbar(sc, ax=ax, shrink=0.55, pad=0.08, label='drone altitude z (m)')
    ax.set_xlim(0, AREA); ax.set_ylim(0, AREA); ax.set_zlim(0, Z_MAX + 10)
    ax.set_box_aspect((1, 1, 0.4))
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)'); ax.set_zlabel('z (m)')
    ax.view_init(elev=24, azim=-58)
    ax.legend(loc='upper left', fontsize=8)
    ax.set_title(f'S051 3D - relay network, seed {SHOW_SEED} (grey pillars = dense debris h >= 10 m; '
                 'dashed = relay links)', fontsize=10)
    p = os.path.join(out, 'network_3d.png')
    plt.savefig(p, dpi=130, bbox_inches='tight'); plt.close(fig)
    print(f'Saved: {p}')


def plot_coverage_maps(res, out):
    w, o = res['world'], res['per_seed'][SHOW_SEED]
    fig, axes = plt.subplots(1, 4, figsize=(22, 5.8))
    dense = _img(w.h >= H_DENSE).astype(float)
    for ax, s_ in zip(axes, STRATS):
        ev = o[s_]
        ax.imshow(_img(res['pop']), origin='lower', extent=[0, AREA, 0, AREA], cmap='Greys', alpha=0.35)
        ax.imshow(np.where(_img(ev['covered_mask']), _img(res['pop']), np.nan), origin='lower',
                  extent=[0, AREA, 0, AREA], cmap='YlOrRd')
        ax.contourf(np.linspace(CELL / 2, AREA - CELL / 2, GRID_N), np.linspace(CELL / 2, AREA - CELL / 2, GRID_N),
                    dense, levels=[0.5, 1.5], colors='none', hatches=['////'])
        nodes = np.vstack([BASE3, ev['xyz']])
        ii, jj = np.nonzero(np.triu(ev['A']))
        for a, b in zip(ii, jj):
            ax.plot(*zip(nodes[a, :2], nodes[b, :2]), 'k--', lw=1)
        for k, p in enumerate(ev['xyz']):
            relay = k in ev['relay_only']
            ax.plot(p[0], p[1], 's' if relay else 'o', color=COLORS[s_] if ev['conn'][k] else 'red',
                    ms=9, mec='k')
            ax.annotate(f'{p[2]:.0f} m', p[:2], xytext=(6, 5), textcoords='offset points', fontsize=8)
        ax.plot(*BASE3[:2], '*', color='k', ms=15)
        extra = f' (M* = {ev["M"]})' if s_ == 'daisy' else ''
        ax.set_title(f'{LABELS[s_]}{extra}\ncoverage {100 * ev["cov"]:.1f}%, '
                     f'{"connected" if ev["feasible"] else "NOT connected"}', fontsize=10)
        ax.set_xlim(0, AREA); ax.set_ylim(0, AREA); ax.set_aspect('equal'); ax.set_xlabel('x (m)')
    axes[0].set_ylabel('y (m)')
    fig.suptitle(f'S051 3D coverage, seed {SHOW_SEED}: coloured = served population; hatched = dense debris; '
                 'labels = drone altitude; squares = relay-only drones; red = disconnected', fontsize=11)
    plt.tight_layout()
    p = os.path.join(out, 'coverage_maps.png')
    plt.savefig(p, dpi=120, bbox_inches='tight'); plt.close(fig)
    print(f'Saved: {p}')


def plot_los_shadow(res, out):
    w, o = res['world'], res['per_seed'][SHOW_SEED]
    fig, axes = plt.subplots(1, 3, figsize=(17, 5.4))
    im = axes[0].imshow(_img(w.h), origin='lower', extent=[0, AREA, 0, AREA], cmap='Greys', vmin=0, vmax=H_DEBRIS_MAX)
    fig.colorbar(im, ax=axes[0], label='debris height h (m)')
    axes[0].contour(np.linspace(CELL / 2, AREA - CELL / 2, GRID_N), np.linspace(CELL / 2, AREA - CELL / 2, GRID_N),
                    _img(w.h), levels=[H_DENSE], colors='red', linewidths=0.8)
    axes[0].set_title(f'Debris height map\n(red contour = {H_DENSE:.0f} m dense threshold)', fontsize=10)
    for ax, s_ in zip(axes[1:], ('fixed', 'alt')):
        ev = o[s_]
        im = ax.imshow(_img(ev['clear_cnt'] / K_DRONES), origin='lower', extent=[0, AREA, 0, AREA],
                       cmap='magma', vmin=0, vmax=1)
        ax.plot(ev['xyz'][:, 0], ev['xyz'][:, 1], 'o', color='cyan', mec='k', ms=8)
        ax.set_title(f'{LABELS[s_]}:\nfraction of drones with clear LOS '
                     f'(range ignored; mean over cells {np.mean(ev["clear_cnt"]) / K_DRONES:.2f})', fontsize=10)
        fig.colorbar(im, ax=ax, label='fraction of the 5 drones')
    for ax in axes:
        ax.set_xlabel('x (m)'); ax.set_aspect('equal')
    axes[0].set_ylabel('y (m)')
    plt.tight_layout()
    p = os.path.join(out, 'los_shadow_map.png')
    plt.savefig(p, dpi=120, bbox_inches='tight'); plt.close(fig)
    print(f'Saved: {p}')


def plot_altitudes(res, out):
    P = res['per_seed']
    o = P[SHOW_SEED]
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 4.8))
    k = np.arange(K_DRONES)
    ax1.bar(k - 0.2, o['fixed']['xyz'][:, 2], 0.4, color=COLORS['fixed'], label='fixed-z SA (50 m)')
    ax1.bar(k + 0.2, o['alt']['xyz'][:, 2], 0.4, color=COLORS['alt'], label=f'altitude SA (ceiling {Z_MAX:.0f} m)')
    ax1.bar(k + 0.2, o['alt_high']['xyz'][:, 2] - o['alt']['xyz'][:, 2], 0.4, bottom=o['alt']['xyz'][:, 2],
            color='none', edgecolor=COLORS['alt'], hatch='//', label=f'probe: ceiling lifted to {Z_MAX_PROBE:.0f} m')
    ax1.axhline(Z_MAX, color='k', ls=':', lw=1)
    ax1.set_xticks(k, [f'D{i + 1}' for i in k])
    ax1.set_ylabel('altitude z (m)')
    ax1.set_title(f'Per-drone altitude, seed {SHOW_SEED} (drones are matched by index only)')
    ax1.legend(fontsize=8)
    for sc in DEBRIS_SCALES:
        zm = [p_['dsweep'][sc]['alt']['xyz'][:, 2].mean() for p_ in P]
        ax2.plot([sc] * len(zm), zm, 'o', color=COLORS['alt'], alpha=0.5)
    ax2.plot(DEBRIS_SCALES, [np.mean([p_['dsweep'][sc]['alt']['xyz'][:, 2].mean() for p_ in P])
                             for sc in DEBRIS_SCALES], '-', color=COLORS['alt'], lw=2, label='mean over seeds')
    ax2.axhline(Z_MAX, color='k', ls=':', lw=1, label=f'ceiling {Z_MAX:.0f} m')
    ax2.axhline(Z_FLAT, color=COLORS['fixed'], ls='--', lw=1, label='2D altitude 50 m')
    ax2.set_xlabel('debris height scale (0 = no debris, 1 = main)')
    ax2.set_ylabel('mean drone altitude chosen by altitude SA (m)')
    ax2.set_title(f'Ablation: altitude chosen vs debris height ({SWEEP_STEPS:,} it.)')
    ax2.legend(fontsize=8); ax2.grid(alpha=0.3)
    plt.tight_layout()
    p = os.path.join(out, 'altitude_bars.png')
    plt.savefig(p, dpi=130, bbox_inches='tight'); plt.close(fig)
    print(f'Saved: {p}')


def plot_coverage_bar(res, out):
    P = res['per_seed']
    fig, ax = plt.subplots(figsize=(9, 5))
    for i, s_ in enumerate(STRATS):
        v = _pct([o[s_]['cov'] for o in P])
        feas = np.array([o[s_]['feasible'] for o in P])
        ax.bar(i, v.mean(), color=COLORS[s_], edgecolor='k', alpha=0.8)
        jit = np.linspace(-0.25, 0.25, len(v))
        ax.scatter(i + jit[feas], v[feas], color='k', s=18, zorder=3)
        ax.scatter(i + jit[~feas], v[~feas], color='red', marker='x', s=40, zorder=3)
        ax.text(i, 5, f'mean {v.mean():.1f}%\n(all {len(v)} seeds)', ha='center', fontsize=9,
                bbox=dict(boxstyle='round', fc='white', alpha=0.85))
    ax.axhline(100 * res['sa2d_cov2d'], color='k', ls=':', lw=1,
               label=f'2D S051 SA in the 2D model: {100 * res["sa2d_cov2d"]:.1f}%')
    ax.set_xticks(range(len(STRATS)), [LABELS[s_].replace(' (', '\n(') for s_ in STRATS], fontsize=9)
    ax.set_ylabel('covered population (%)'); ax.set_ylim(0, 100)
    ax.set_title(f'S051 3D coverage by strategy - bars = mean, dots = {len(P)} seeds '
                 '(red x = not connected)')
    ax.legend(fontsize=8, loc='upper left'); ax.grid(axis='y', alpha=0.3)
    p = os.path.join(out, 'coverage_bar.png')
    plt.savefig(p, dpi=130, bbox_inches='tight'); plt.close(fig)
    print(f'Saved: {p}')


def plot_altitude_sweep(res, out):
    P = res['per_seed']
    zs = list(Z_FIX_SWEEP) + list(Z_FIX_HIGH)
    fig, ax = plt.subplots(figsize=(9, 5))
    for o in P:
        v = [100 * (o['zsweep'][z]['cov'] if z in Z_FIX_SWEEP else o['zhigh'][z]['cov']) for z in zs]
        ax.plot(zs, v, '-', color=COLORS['fixed'], alpha=0.25)
    mean = [np.mean([100 * (o['zsweep'][z]['cov'] if z in Z_FIX_SWEEP else o['zhigh'][z]['cov']) for o in P])
            for z in zs]
    ax.plot(zs, mean, 'o-', color=COLORS['fixed'], lw=2, label='fixed-z SA, uniform altitude (mean; thin = seeds)')
    va = np.mean(_pct([o['dsweep'][1.0]['alt']['cov'] for o in P]))
    vh = np.mean(_pct([o['alt_high']['cov'] for o in P]))
    ax.axhline(va, color=COLORS['alt'], ls='--', label=f'altitude SA, z <= {Z_MAX:.0f} m: {va:.1f}%')
    ax.axhline(vh, color=COLORS['alt'], ls=':', label=f'altitude SA, z <= {Z_MAX_PROBE:.0f} m (probe): {vh:.1f}%')
    ax.axvspan(Z_MAX, Z_MAX_PROBE + 10, color='gray', alpha=0.12, label=f'above the card ceiling {Z_MAX:.0f} m')
    ax.set_xlabel('uniform hover altitude z (m)'); ax.set_ylabel('covered population (%)')
    ax.set_title(f'Coverage vs uniform altitude ({SWEEP_STEPS:,} SA iterations, {len(P)} seeds)')
    ax.legend(fontsize=8); ax.grid(alpha=0.3)
    p = os.path.join(out, 'altitude_sweep.png')
    plt.savefig(p, dpi=130, bbox_inches='tight'); plt.close(fig)
    print(f'Saved: {p}')


def plot_debris_sweep(res, out):
    P = res['per_seed']
    fig, ax = plt.subplots(figsize=(8.5, 5))
    for s_, key in (('fixed', 'fixed'), ('alt', 'alt')):
        m = [np.mean(_pct([o['dsweep'][sc][key]['cov'] for o in P])) for sc in DEBRIS_SCALES]
        lo = [np.min(_pct([o['dsweep'][sc][key]['cov'] for o in P])) for sc in DEBRIS_SCALES]
        hi = [np.max(_pct([o['dsweep'][sc][key]['cov'] for o in P])) for sc in DEBRIS_SCALES]
        ax.plot(DEBRIS_SCALES, m, 'o-', color=COLORS[s_], label=LABELS[s_])
        ax.fill_between(DEBRIS_SCALES, lo, hi, color=COLORS[s_], alpha=0.15)
    ax.set_xlabel('debris height scale (0 = no debris, 1.0 = main, heights 0-30 m)')
    ax.set_ylabel('covered population (%)')
    ax.set_title(f'Debris ablation ({SWEEP_STEPS:,} SA iterations; band = seed range)')
    ax.legend(fontsize=9); ax.grid(alpha=0.3)
    p = os.path.join(out, 'debris_sweep.png')
    plt.savefig(p, dpi=130, bbox_inches='tight'); plt.close(fig)
    print(f'Saved: {p}')


def plot_sa_convergence(res, out):
    o = res['per_seed'][SHOW_SEED]
    fig, axes = plt.subplots(3, 1, figsize=(9, 9), sharex=True)
    it = np.arange(1, N_SA_STEPS + 1)
    for s_, h in (('fixed', o['hist_f']), ('alt', o['hist_a'])):
        axes[0].plot(it, h['E'], color=COLORS[s_], lw=0.8, label=LABELS[s_])
        axes[1].plot(it, 100 * h['cov'], color=COLORS[s_], lw=0.8)
    for j, oo in enumerate(res['per_seed']):
        for s_, ls in (('fixed', '-'), ('alt', '--')):
            axes[2].plot(it, oo['pen'][s_] + 0.04 * j, color=COLORS[s_], lw=0.8, ls=ls,
                         label=f'{s_} (all seeds, offset 0.04 per seed)' if j == 0 else None)
    ax2b = axes[1].twinx()
    ax2b.plot(it, o['hist_a']['zmean'], color='tab:green', lw=0.8)
    ax2b.set_ylabel('altitude SA: mean z (m)', color='tab:green')
    axes[0].set_yscale('symlog', linthresh=1e2)
    axes[0].set_ylabel('E (persons)'); axes[0].legend(fontsize=8)
    axes[1].set_ylabel('coverage (%)')
    axes[2].set_ylabel('disconnected + energy-\ninfeasible drones'); axes[2].legend(fontsize=7)
    axes[2].set_xscale('log'); axes[2].set_xlabel('SA iteration (log scale)')
    for a in axes:
        a.grid(alpha=0.3)
    fig.suptitle(f'S051 3D SA convergence (top two panels: seed {SHOW_SEED}; bottom: penalties, all seeds)')
    plt.tight_layout()
    p = os.path.join(out, 'sa_convergence.png')
    plt.savefig(p, dpi=130, bbox_inches='tight'); plt.close(fig)
    print(f'Saved: {p}')


def save_animation(res, out):
    w, o = res['world'], res['per_seed'][SHOW_SEED]
    frames = o['frames']
    fig = plt.figure(figsize=(8, 6.5))
    ax = fig.add_subplot(111, projection='3d', computed_zorder=False)
    _pillars(ax, w.h, alpha=0.18)
    ax.scatter(*BASE3, marker='*', color='k', s=200, depthshade=False)
    ax.set_xlim(0, AREA); ax.set_ylim(0, AREA); ax.set_zlim(0, Z_MAX + 10)
    ax.set_box_aspect((1, 1, 0.4)); ax.view_init(elev=24, azim=-58)
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)'); ax.set_zlabel('z (m)')
    artists = []

    def update(i):
        for a in artists:
            a.remove()
        artists.clear()
        step, xyz, cov, nbad = frames[i]
        A = link_matrix(w, xyz)
        conn = reach_from_base(A)[1:]
        nodes = np.vstack([BASE3, xyz])
        ii, jj = np.nonzero(np.triu(A))
        for a, b in zip(ii, jj):
            artists.extend(ax.plot(*zip(nodes[a], nodes[b]), 'k--', lw=0.8))
        cols = [COLORS['alt'] if c else 'red' for c in conn]
        artists.append(ax.scatter(xyz[:, 0], xyz[:, 1], xyz[:, 2], color=cols, s=70, edgecolor='k',
                                  depthshade=False))
        for p in xyz:
            artists.extend(ax.plot([p[0]] * 2, [p[1]] * 2, [0, p[2]], ':', color='gray', lw=0.8))
        label = 'best' if i == len(frames) - 1 else f'iter {step:,}'
        ax.set_title(f'S051 3D altitude SA ({label}) - coverage {100 * cov:.1f}%, '
                     f'mean z {xyz[:, 2].mean():.0f} m, {nbad} penalised', fontsize=10)
        return artists

    anim = animation.FuncAnimation(fig, update, frames=len(frames), interval=100, blit=False)
    p = os.path.join(out, 'animation.gif')
    anim.save(p, writer='pillow', fps=8, dpi=80)
    plt.close(fig)
    print(f'Saved: {p} ({len(frames)} frames: SA states every {frames[1][0]} iterations, then the final best layout)')


if __name__ == '__main__':
    res = run_simulation()
    report(res)
    out = os.path.normpath(OUTPUT_DIR)
    os.makedirs(out, exist_ok=True)
    plot_network_3d(res, out)
    plot_coverage_maps(res, out)
    plot_los_shadow(res, out)
    plot_altitudes(res, out)
    plot_coverage_bar(res, out)
    plot_altitude_sweep(res, out)
    plot_debris_sweep(res, out)
    plot_sa_convergence(res, out)
    save_animation(res, out)
