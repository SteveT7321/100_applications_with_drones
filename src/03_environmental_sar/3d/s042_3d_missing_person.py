"""
S042 3D Upgrade — Missing Person Search
=======================================
The S042 search (Gaussian + diffusion prior, sigma0 = 19.7 m, shared Bayesian
belief map, binary hit/miss observation after a 1 s dwell, Pd = 0.90,
Pfa = 0.02, 5 m/s kinematic legs, 1800 s cutoff) is moved onto a synthetic
mountain DEM (4 sin*cos octaves, 1.25 - 38.75 m).  Every observation now uses
the 3D sensor geometry of the card:
  * a downward thermal cone of half-angle 11.3 deg: a ground cell is inside the
    cone when its nadir angle psi = atan(horizontal distance / (z_d - z_dem))
    is <= 11.3 deg (footprint 2.0 m on level ground at 10 m AGL);
  * terrain occlusion: the line of sight from the drone to the cell is sampled
    at 20 points; a sample more than 0.1 m below the DEM blocks the cell;
  * only visible in-cone cells get the Pd / (1 - Pd) likelihood, every other
    cell gets Pfa / (1 - Pfa) (no information).
Drones fly either terrain-following legs (z = z_dem + 10 m sampled every 1 m,
time = 3D length / v) or at one fixed altitude above every ridge.

Strategies (same person per trial, same sensor stream, same starts):
  1. greedy      terrain-aware greedy: highest-belief cell with slope <= 1,
                 terrain-following at 10 m AGL                       (card 1)
  2. lm_flat     boustrophedon of the S042 +/-2 sigma0 box at a fixed altitude
                 that clears every ridge (48.75 m MSL)                (card 2)
  3. valley      greedy on the valley-biased prior P0 exp(-z/15 m)   (card 3)
  4. lm_terrain  the same boustrophedon, terrain-following at 10 m AGL (control:
                 separates "lawnmower vs greedy" from "flat vs terrain-following")
Two truth models: the person is drawn from P0 (the card) or from the
valley-biased prior (the valley hypothesis is true).

Usage:
    conda activate drones
    python src/03_environmental_sar/3d/s042_3d_missing_person.py
"""

import os
import importlib.util
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401
from matplotlib.animation import FuncAnimation, PillowWriter

_HERE = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location(
    's042_2d', os.path.join(_HERE, '..', 's042_missing_person.py'))
m2d = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(m2d)

OUTPUT_DIR = os.path.normpath(os.path.join(
    _HERE, '..', '..', '..', 'outputs', '03_environmental_sar', '3d', 's042_3d_missing_person'))

# ── Parameters ──────────────────────────────────────────────────────────────
N_DRONES      = 3
AREA_SIZE     = m2d.AREA_SIZE        # 200 m
GRID_RES      = m2d.GRID_RES         # 0.5 m
GRID_N        = m2d.GRID_N           # 400
CELL_C        = m2d.CELL_C
H_AGL         = 10.0                 # m
PHI_S         = np.deg2rad(11.3)     # thermal cone half-angle
TAN_PHI       = np.tan(PHI_S)
R_S_NOM       = H_AGL * TAN_PHI      # 2.0 m on level ground
DRONE_SPEED   = m2d.DRONE_SPEED      # 5 m/s (3D path speed)
TAU_DWELL     = m2d.TAU_DWELL        # 1 s
P_DETECT      = m2d.P_DETECT         # 0.90
P_FALSE_ALARM = m2d.P_FALSE_ALARM    # 0.02
LKP           = m2d.LKP              # (100, 120)
SIGMA0        = m2d.SIGMA0           # 19.70 m
START_XY      = m2d.START_POS        # south edge
MAX_SIM_TIME  = m2d.MAX_SIM_TIME     # 1800 s
LM_SPACING    = m2d.LM_SPACING       # 2.5 m
LM_HALF_WIDTH = m2d.LM_HALF_WIDTH    # 2 sigma0
DEM_OFFSET    = 20.0                 # m — base elevation (added: card's sum ranges +-18.75 m)
DEM_A         = 10.0                 # m — first-octave amplitude
DEM_L1        = 80.0                 # m — base wavelength
DEM_K         = 4                    # octaves (L_k = L1 / 2^(k-1), A_k = A / 2^(k-1))
DEM_SEED      = 7                    # phases of octaves 2..4
TAU_VALLEY    = 15.0                 # m
N_RAY         = 20                   # LOS samples
OCC_TOL       = 0.1                  # m
DS_TRANSIT    = 1.0                  # m — terrain-following altitude sample interval
SLOPE_MAX     = 1.0                  # |grad z| <= 1 (45 deg) for greedy targets
N_MC          = 50                   # trials per strategy and truth model
N_FRAMES      = 120
OCC_CONES     = (11.3, 30.0, 45.0)   # deg — occlusion sensitivity check (card cone + two wider ones)
OCC_POSE_EVERY = 40                 # every 40th lawnmower waypoint is a check pose
ENT_EVERY     = 5.0                  # s

STRATS = ['greedy', 'lm_flat', 'valley', 'lm_terrain']
LABELS = {'greedy': 'Terrain-aware greedy', 'lm_flat': 'Flat-altitude lawnmower',
          'valley': 'Valley-prior greedy', 'lm_terrain': 'Terrain-following lawnmower'}
COLORS = {'greedy': 'tab:blue', 'lm_flat': 'tab:orange', 'valley': 'tab:green',
          'lm_terrain': 'tab:purple'}
TRUTHS = ['P0', 'valley']
DCOL = ['cyan', 'lime', 'magenta']


# ── Terrain ─────────────────────────────────────────────────────────────────

def _phases():
    """Octave 1 phases put the LKP at the octave-1 valley floor (sin = -1, cos = +1);
    octaves 2..4 get seeded random phases."""
    rng = np.random.default_rng(DEM_SEED)
    ph = rng.uniform(0, 2 * np.pi, size=(DEM_K, 2))
    ph[0, 0] = -np.pi / 2 - 2 * np.pi * LKP[0] / DEM_L1
    ph[0, 1] = -2 * np.pi * LKP[1] / DEM_L1
    return ph


PHASES = _phases()


def dem(x, y):
    z = np.full(np.broadcast(x, y).shape, DEM_OFFSET)
    for k in range(DEM_K):
        L = DEM_L1 / 2 ** k
        z = z + DEM_A / 2 ** k * np.sin(2 * np.pi * x / L + PHASES[k, 0]) * np.cos(2 * np.pi * y / L + PHASES[k, 1])
    return np.clip(z, 0.0, 40.0)


def dem_grad(x, y):
    gx = np.zeros(np.broadcast(x, y).shape); gy = np.zeros_like(gx)
    for k in range(DEM_K):
        L = DEM_L1 / 2 ** k
        a, w = DEM_A / 2 ** k, 2 * np.pi / L
        sx, cx = np.sin(w * x + PHASES[k, 0]), np.cos(w * x + PHASES[k, 0])
        sy, cy = np.sin(w * y + PHASES[k, 1]), np.cos(w * y + PHASES[k, 1])
        gx = gx + a * w * cx * cy
        gy = gy - a * w * sx * sy
    return gx, gy


XX, YY = np.meshgrid(CELL_C, CELL_C)          # [iy, ix], as in S042
ZDEM = dem(XX, YY)
GX, GY = dem_grad(XX, YY)
SLOPE = np.hypot(GX, GY)
SAFE = SLOPE <= SLOPE_MAX
Z_FLAT = float(ZDEM.max()) + H_AGL            # fixed altitude clearing every ridge


def build_priors():
    p0 = m2d.build_prior()
    pv = p0 * np.exp(-ZDEM / TAU_VALLEY)
    return p0, pv / pv.sum()


# ── 3D sensor ───────────────────────────────────────────────────────────────

def visible_cone(p, tan_phi=TAN_PHI):
    """Index window and mask of the cells that are inside the cone AND not occluded,
    plus the mask of in-cone cells that are occluded."""
    rmax = max(p[2] - 0.0, 0.0) * tan_phi
    ix0 = max(int((p[0] - rmax) / GRID_RES) - 1, 0); ix1 = min(int((p[0] + rmax) / GRID_RES) + 2, GRID_N)
    iy0 = max(int((p[1] - rmax) / GRID_RES) - 1, 0); iy1 = min(int((p[1] + rmax) / GRID_RES) + 2, GRID_N)
    win = (slice(iy0, iy1), slice(ix0, ix1))
    X, Y, Z = XX[win], YY[win], ZDEM[win]
    h = p[2] - Z
    dh = np.hypot(X - p[0], Y - p[1])
    cone = (h > 0) & (dh <= h * tan_phi)
    occ = np.zeros_like(cone)
    ci = np.nonzero(cone)
    if len(ci[0]):
        t = (np.arange(1, N_RAY + 1) / (N_RAY + 1))[None, :]
        xs = p[0] + t * (X[ci][:, None] - p[0])
        ys = p[1] + t * (Y[ci][:, None] - p[1])
        zl = p[2] + t * (Z[ci][:, None] - p[2])
        blocked = np.any(dem(xs, ys) > zl + OCC_TOL, axis=1)
        occ[ci] = blocked
    return win, cone & ~occ, cone & occ


def bayes_update(B, win, vis, hit):
    ratio = P_DETECT / P_FALSE_ALARM if hit else (1.0 - P_DETECT) / (1.0 - P_FALSE_ALARM)
    sub = B[win]
    sub[vis] *= ratio


# ── Paths ───────────────────────────────────────────────────────────────────

def terrain_leg(p_from, xy_to):
    """Terrain-following polyline: z = dem + h_AGL sampled every DS_TRANSIT m."""
    d = np.linalg.norm(xy_to - p_from[:2])
    n = max(int(np.ceil(d / DS_TRANSIT)), 1)
    s = np.linspace(0, 1, n + 1)[:, None]
    xy = p_from[:2] + s * (xy_to - p_from[:2])
    z = dem(xy[:, 0], xy[:, 1]) + H_AGL
    pts = np.column_stack([xy, z])
    pts[0] = p_from
    return pts


def flat_leg(p_from, xy_to):
    """Climb vertically to Z_FLAT (only on the first leg), then fly level."""
    pts = [p_from]
    if abs(p_from[2] - Z_FLAT) > 1e-9:
        pts.append(np.array([p_from[0], p_from[1], Z_FLAT]))
    pts.append(np.array([xy_to[0], xy_to[1], Z_FLAT]))
    return np.array(pts)


class Drone3D:
    def __init__(self, idx, p0, flat):
        self.idx, self.flat = idx, flat
        self.legs = []            # (t0, cumulative times, points)
        self.target = p0.copy()
        self.t_obs = 0.0
        self.lm_cursor = 0
        self.length = 0.0

    def assign(self, xy, t_now):
        pts = flat_leg(self.target, xy) if self.flat else terrain_leg(self.target, xy)
        seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        self.length += seg.sum()
        times = t_now + np.concatenate([[0.0], np.cumsum(seg)]) / DRONE_SPEED
        self.legs.append((t_now, times, pts))
        self.target = pts[-1].copy()
        self.t_obs = times[-1] + TAU_DWELL

    def pos_at(self, t):
        for t0, times, pts in reversed(self.legs):
            if t >= t0:
                if t >= times[-1]:
                    return pts[-1].copy()
                return np.array([np.interp(t, times, pts[:, j]) for j in range(3)])
        return self.legs[0][2][0].copy()


def lawnmower_seqs():
    return m2d.build_lawnmower_sequences()


LM_SEQS = lawnmower_seqs()


def greedy_next(B, others):
    """S042 greedy restricted to slope-safe cells."""
    Bs = np.where(SAFE, B, -1.0)
    return m2d.greedy_next(Bs, others)


def next_waypoint(strategy, B, drone, drones):
    others = [d.target[:2] for d in drones if d is not drone]
    if strategy in ('greedy', 'valley'):
        return greedy_next(B, others)
    seq = LM_SEQS[drone.idx]
    wp = seq[drone.lm_cursor % len(seq)].copy()
    drone.lm_cursor += 1
    return wp


# ── Mission ─────────────────────────────────────────────────────────────────

def simulate(strategy, seed, truth='P0', record=False, person_absent=False, frame_times=None):
    rng_p = np.random.default_rng([seed, 0, TRUTHS.index(truth)])
    rng_s = np.random.default_rng([seed, 1])
    p0, pv = build_priors()
    k = rng_p.choice(GRID_N * GRID_N, p=(p0 if truth == 'P0' else pv).ravel())
    iy, ix = divmod(int(k), GRID_N)
    person = np.array([CELL_C[ix], CELL_C[iy]])
    B = (pv if strategy == 'valley' else p0).copy()
    prior_eval = p0 if truth == 'P0' else pv
    flat = strategy == 'lm_flat'
    drones = [Drone3D(i, np.array([*START_XY[i], dem(*START_XY[i]) + H_AGL]), flat)
              for i in range(N_DRONES)]
    for d in drones:
        d.assign(next_waypoint(strategy, B, d, drones), 0.0)
    Q = np.ones_like(B)
    occ_count = np.zeros_like(B) if record else None
    ent_t, ent_h, pdet = [0.0], [m2d.belief_entropy(B)], [0.0]
    next_ent = ENT_EVERY
    snap_times = [0.0, 60.0, 120.0]
    snaps, frames = [], []
    frame_times = [] if frame_times is None else list(frame_times)
    f_idx = 0
    n_obs = n_occ_obs = n_person_occ = 0
    vis_cells = []
    det_time = None

    def capture(tc):
        return (tc, B / B.sum(), np.array([dd.pos_at(tc) for dd in drones]),
                None if occ_count is None else occ_count.copy())

    while True:
        d = min(drones, key=lambda dd: (dd.t_obs, dd.idx))
        t = d.t_obs
        if t > MAX_SIM_TIME:
            break
        if record:
            while snap_times and snap_times[0] < t:
                snaps.append(capture(snap_times.pop(0)))
            while next_ent < t:
                ent_t.append(next_ent); ent_h.append(m2d.belief_entropy(B))
                pdet.append(1.0 - float((prior_eval * Q).sum()))
                next_ent += ENT_EVERY
        while f_idx < len(frame_times) and frame_times[f_idx] < t:
            tf = frame_times[f_idx]
            frames.append((tf, (B / B.sum()).reshape(GRID_N // 4, 4, GRID_N // 4, 4).sum(axis=(1, 3)),
                           np.array([dd.pos_at(tf) for dd in drones]), m2d.belief_entropy(B)))
            f_idx += 1
        p = d.target
        assert p[2] - dem(p[0], p[1]) >= H_AGL - 1e-6, 'observation pose below h_AGL'
        win, vis, occ = visible_cone(p)
        n_obs += 1
        n_occ_obs += int(occ.any())
        vis_cells.append(int(vis.sum()))
        rows, cols = win
        pi, pj = int(person[1] / GRID_RES), int(person[0] / GRID_RES)
        in_win = rows.start <= pi < rows.stop and cols.start <= pj < cols.stop
        in_fp = (not person_absent) and in_win and bool(vis[pi - rows.start, pj - cols.start])
        if (not person_absent) and in_win and bool(occ[pi - rows.start, pj - cols.start]):
            n_person_occ += 1
        hit = rng_s.random() < (P_DETECT if in_fp else P_FALSE_ALARM)
        bayes_update(B, win, vis, hit)
        if record:
            Q[win][vis] *= (1.0 - P_DETECT)
            occ_count[win][occ] += 1
        if hit and in_fp:
            det_time = t
            break
        d.assign(next_waypoint(strategy, B, d, drones), t)

    t_end = det_time if det_time is not None else MAX_SIM_TIME
    if record:
        ent_t.append(t_end); ent_h.append(m2d.belief_entropy(B))
        pdet.append(1.0 - float((prior_eval * Q).sum()))
        snaps.append(capture(t_end))
    while f_idx < len(frame_times):
        tf = frame_times[f_idx]
        frames.append((min(tf, t_end), (B / B.sum()).reshape(GRID_N // 4, 4, GRID_N // 4, 4).sum(axis=(1, 3)),
                       np.array([dd.pos_at(min(tf, t_end)) for dd in drones]), m2d.belief_entropy(B)))
        f_idx += 1
    return dict(det_time=det_time, person=person, n_obs=n_obs, n_occ_obs=n_occ_obs,
                n_person_occ=n_person_occ, mean_vis=float(np.mean(vis_cells)) * GRID_RES ** 2,
                ent_t=np.array(ent_t), ent_h=np.array(ent_h), pdet=np.array(pdet),
                snaps=snaps, frames=frames, drones=drones, t_end=t_end, occ_count=occ_count,
                path_len=sum(dd.length for dd in drones), steep=not SAFE[pi, pj])


def check_clearance(drones):
    """Minimum height above the DEM along every flown leg (sampled every 0.25 m)."""
    worst = np.inf
    for d in drones:
        for _, times, pts in d.legs:
            for a, b in zip(pts[:-1], pts[1:]):
                n = max(int(np.ceil(np.linalg.norm(b[:2] - a[:2]) / 0.25)), 1)
                s = np.linspace(0, 1, n + 1)[:, None]
                q = a + s * (b - a)
                worst = min(worst, float(np.min(q[:, 2] - dem(q[:, 0], q[:, 1]))))
    return worst


def run_simulation():
    p0, pv = build_priors()
    print(f'Prior sigma0 = sqrt(10^2 + 2*0.02*7200) = {SIGMA0:.2f} m (card 17.2 m corrected)')
    print(f'DEM range {ZDEM.min():.2f} .. {ZDEM.max():.2f} m, LKP elevation {dem(*LKP):.2f} m, '
          f'mean {ZDEM.mean():.2f} m')
    print(f'Footprint on level ground at {H_AGL:.0f} m AGL: r_s = {R_S_NOM:.3f} m')
    print(f'Flat-lawnmower altitude Z_FLAT = max DEM + h_AGL = {Z_FLAT:.2f} m MSL '
          f'(card 20 m MSL would be {ZDEM.max() - 20:.1f} m below the highest ridge)')
    X, Y = XX, YY
    box = (np.abs(X - LKP[0]) <= LM_HALF_WIDTH) & (np.abs(Y - LKP[1]) <= LM_HALF_WIDTH)
    zb = ZDEM[box]
    print(f'Lawnmower box DEM range {zb.min():.2f} .. {zb.max():.2f} m -> flat-lawnmower AGL '
          f'{Z_FLAT - zb.max():.2f} .. {Z_FLAT - zb.min():.2f} m, level-ground footprint '
          f'{(Z_FLAT - zb.max()) * TAN_PHI:.2f} .. {(Z_FLAT - zb.min()) * TAN_PHI:.2f} m')
    print(f'Slope-safe cells (|grad z| <= 1): {100 * SAFE.mean():.1f} % of the area, prior mass on '
          f'steep cells P0 {p0[~SAFE].sum():.3f}, valley prior {pv[~SAFE].sum():.3f}')
    print(f'Prior mass inside the lawnmower box: P0 {p0[box].sum():.3f}, valley prior {pv[box].sum():.3f}')
    print(f'Prior entropy H0: P0 {m2d.belief_entropy(p0):.3f} nats, valley {m2d.belief_entropy(pv):.3f} nats')
    smax = float(SLOPE.max())
    print(f'Steepest DEM slope |grad z| = {smax:.2f} ({np.degrees(np.arctan(smax)):.1f} deg); a line of '
          f'sight inside the 11.3 deg cone is >= {90 - np.degrees(PHI_S):.1f} deg above horizontal')
    print('--- Occlusion check: share of in-cone cells hidden by terrain, poses on the lawnmower lattice '
          f'(every {OCC_POSE_EVERY}th waypoint) ---')
    poses = np.vstack(LM_SEQS)[::OCC_POSE_EVERY]
    occ_check, occ_grid = {}, {}
    for deg in OCC_CONES:
        tp = np.tan(np.deg2rad(deg))
        for mode in ('terrain', 'flat'):
            n_c = n_o = 0
            grid = np.zeros((GRID_N, GRID_N))
            for xy in poses:
                z = Z_FLAT if mode == 'flat' else dem(*xy) + H_AGL
                win, vis, occ = visible_cone(np.array([xy[0], xy[1], z]), tp)
                n_c += int(vis.sum() + occ.sum()); n_o += int(occ.sum())
                grid[win][occ] += 1
            occ_check[(deg, mode)] = 100 * n_o / max(n_c, 1)
            occ_grid[(deg, mode)] = grid
            print(f'cone half-angle {deg:4.1f} deg, {mode:7s} altitude: {len(poses)} poses, '
                  f'{n_o}/{n_c} in-cone cells occluded ({occ_check[(deg, mode)]:.2f} %)')

    print('--- Person-absent runs, seed 0, full 1800 s ---')
    absent = {}
    for s in STRATS:
        r = simulate(s, 0, record=True, person_absent=True)
        absent[s] = r
        hs = {tt: np.interp(tt, r['ent_t'], r['ent_h']) for tt in (300, 900, 1800)}
        ps = {tt: np.interp(tt, r['ent_t'], r['pdet']) for tt in (300, 900, 1800)}
        zs = np.concatenate([pts[:, 2] for d in r['drones'] for _, _, pts in d.legs])
        print(f'[{s:>10s}] {r["n_obs"]} obs, path {r["path_len"]:.0f} m, mean visible footprint '
              f'{r["mean_vis"]:.1f} m^2, obs with an occluded in-cone cell {r["n_occ_obs"]} '
              f'({100 * r["n_occ_obs"] / r["n_obs"]:.1f} %), cells ever occluded '
              f'{int((r["occ_count"] > 0).sum())}, min clearance {check_clearance(r["drones"]):.2f} m, '
              f'altitude {zs.min():.1f}..{zs.max():.1f} m')
        print(f'[{s:>10s}] H(300/900/1800) = {hs[300]:.3f} / {hs[900]:.3f} / {hs[1800]:.3f} nats; '
              f'E[P_det] (P0 truth) 300/900/1800 s = {ps[300]:.3f} / {ps[900]:.3f} / {ps[1800]:.3f}')

    print('--- Seed-0 demo runs (truth P0) ---')
    demo = {}
    for s in STRATS:
        demo[s] = simulate(s, 0, record=True)
    # animation frames for the greedy demo, spread over its own mission
    demo['greedy'] = simulate('greedy', 0, record=True,
                              frame_times=np.linspace(0.0, demo['greedy']['t_end'], N_FRAMES))
    for s in STRATS:
        r = demo[s]
        st = f'{r["det_time"]:.1f} s' if r['det_time'] is not None else 'NOT FOUND'
        print(f'[{s:>10s}] person at ({r["person"][0]:.2f}, {r["person"][1]:.2f}), '
              f'z {dem(*r["person"]):.2f} m, detection {st}, {r["n_obs"]} observations')

    print(f'--- Monte Carlo: {N_MC} trials per strategy and truth model (same persons and sensor '
          f'stream across strategies) ---')
    mc = {tr: {} for tr in TRUTHS}
    for tr in TRUTHS:
        for s in STRATS:
            times, steep, pocc = [], [], 0
            for seed in range(N_MC):
                r = simulate(s, seed, truth=tr)
                times.append(r['det_time'] if r['det_time'] is not None else np.nan)
                steep.append(r['steep'])
                pocc += r['n_person_occ']
            times = np.array(times)
            mc[tr][s] = dict(times=times, steep=np.array(steep), pocc=pocc)
            f = np.isfinite(times)
            cens = np.where(f, times, MAX_SIM_TIME)
            print(f'[truth {tr:>6s}] [{s:>10s}] found {f.sum()}/{N_MC}, median {np.median(cens):.1f} s, '
                  f'IQR [{np.percentile(cens, 25):.1f}, {np.percentile(cens, 75):.1f}] s, '
                  f'mean {cens.mean():.1f} s (not-found = 1800 s), by 300 s {np.sum(times <= 300)}, '
                  f'steep-cell persons found {np.sum(f & np.array(steep))}/{int(np.sum(steep))}, '
                  f'person hidden by terrain inside a cone {pocc} times')
        for a, b in [('lm_flat', 'greedy'), ('lm_flat', 'lm_terrain'), ('valley', 'greedy'),
                     ('lm_terrain', 'greedy')]:
            ta = np.where(np.isfinite(mc[tr][a]['times']), mc[tr][a]['times'], MAX_SIM_TIME)
            tb = np.where(np.isfinite(mc[tr][b]['times']), mc[tr][b]['times'], MAX_SIM_TIME)
            print(f'[truth {tr:>6s}] {a} faster than {b}: {int(np.sum(ta < tb))}/{N_MC}, '
                  f'slower {int(np.sum(ta > tb))}/{N_MC}, tie {int(np.sum(ta == tb))}/{N_MC}')
    return dict(absent=absent, demo=demo, mc=mc, occ_check=occ_check, occ_grid=occ_grid,
                n_occ_poses=len(poses))


# ── Plots ───────────────────────────────────────────────────────────────────

def plot_dem_prior(res, out_dir):
    p0, pv = build_priors()
    fig = plt.figure(figsize=(15, 6.5))
    ax = fig.add_subplot(1, 2, 1, projection='3d')
    s = 4
    Xs, Ys, Zs = XX[::s, ::s], YY[::s, ::s], ZDEM[::s, ::s]
    pn = p0[::s, ::s] / p0.max()
    fc = plt.cm.terrain(0.15 + 0.7 * (Zs - Zs.min()) / (Zs.max() - Zs.min()))
    hot = plt.cm.hot_r(pn)
    fc = np.where(pn[..., None] > 0.05, 0.4 * fc + 0.6 * hot, fc)
    ax.plot_surface(Xs, Ys, Zs, facecolors=fc, rstride=1, cstride=1, linewidth=0, shade=False)
    ax.scatter(*LKP, dem(*LKP) + 1, color='k', marker='x', s=80)
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)'); ax.set_zlabel('z (m)')
    ax.set_title('DEM (terrain colours) with the P0 prior as a heat layer; x = LKP')
    ax.view_init(elev=40, azim=-70)
    ax2 = fig.add_subplot(1, 2, 2)
    im = ax2.imshow(ZDEM, origin='lower', extent=[0, AREA_SIZE, 0, AREA_SIZE], cmap='terrain')
    plt.colorbar(im, ax=ax2, fraction=0.046, label='elevation (m)')
    ax2.contourf(CELL_C, CELL_C, (~SAFE).astype(float), levels=[0.5, 1.5], colors='none', hatches=['////'])
    ax2.contour(CELL_C, CELL_C, p0 / p0.max(), levels=[0.1, 0.5, 0.9], colors='k', linewidths=1)
    ax2.contour(CELL_C, CELL_C, pv / pv.max(), levels=[0.1, 0.5, 0.9], colors='crimson',
                linewidths=1, linestyles='--')
    ax2.add_patch(plt.Rectangle(LKP - LM_HALF_WIDTH, 2 * LM_HALF_WIDTH, 2 * LM_HALF_WIDTH,
                                fill=False, ec='tab:orange', lw=1.5))
    ax2.plot(*LKP, 'kx', ms=10, mew=2)
    ax2.plot(START_XY[:, 0], START_XY[:, 1], '^', color='tab:purple', ms=9)
    ax2.legend(handles=[Line2D([], [], color='k', label='P0 (10/50/90 % of peak)'),
                        Line2D([], [], color='crimson', ls='--', label='valley prior'),
                        Line2D([], [], color='tab:orange', label='lawnmower box (±2σ0)'),
                        Line2D([], [], color='tab:purple', marker='^', ls='', label='drone starts'),
                        plt.Rectangle((0, 0), 1, 1, fc='none', hatch='////', label='slope > 1 (no greedy targets)')],
               fontsize=8, loc='upper right')
    ax2.set_xlabel('x (m)'); ax2.set_ylabel('y (m)')
    ax2.set_title('Top view: priors, steep cells, lawnmower box')
    plt.tight_layout()
    path = os.path.join(out_dir, 'dem_prior.png')
    plt.savefig(path, dpi=120); plt.close()
    print(f'Saved: {path}')


def _leg_points(d, t_max):
    pts = []
    for _, times, p in d.legs:
        keep = times <= t_max
        pts.append(p[keep])
    return np.vstack(pts) if pts else np.zeros((0, 3))


def plot_trajectories_3d(res, out_dir, t_show=300.0):
    fig = plt.figure(figsize=(15, 12))
    s = 5
    Xs, Ys, Zs = XX[::s, ::s], YY[::s, ::s], ZDEM[::s, ::s]
    for i, st in enumerate(STRATS):
        ax = fig.add_subplot(2, 2, i + 1, projection='3d')
        ax.plot_surface(Xs, Ys, Zs, cmap='terrain', alpha=0.35, linewidth=0, rstride=1, cstride=1)
        for d in res['absent'][st]['drones']:
            p = _leg_points(d, t_show)
            ax.plot(p[:, 0], p[:, 1], p[:, 2], color=DCOL[d.idx], lw=1.0)
        ax.scatter(*LKP, dem(*LKP), color='k', marker='x', s=60)
        ax.set_xlim(0, 200); ax.set_ylim(0, 200); ax.set_zlim(0, 55)
        ax.set_xlabel('x'); ax.set_ylabel('y'); ax.set_zlabel('z (m)')
        ax.view_init(elev=30, azim=-75)
        ax.set_title(f'{LABELS[st]} (first {t_show:.0f} s, person absent)', fontsize=10)
    plt.tight_layout()
    path = os.path.join(out_dir, 'trajectories_3d.png')
    plt.savefig(path, dpi=110); plt.close()
    print(f'Saved: {path}')


def plot_altitude(res, out_dir, t_show=600.0):
    fig, axes = plt.subplots(len(STRATS), 1, figsize=(11, 10), sharex=True)
    tt = np.linspace(0, t_show, 1201)
    for ax, st in zip(axes, STRATS):
        for d in res['absent'][st]['drones']:
            P = np.array([d.pos_at(t) for t in tt])
            ax.plot(tt, P[:, 2], color=DCOL[d.idx], lw=1.0, label=f'drone {d.idx + 1} z')
            ax.plot(tt, dem(P[:, 0], P[:, 1]), color=DCOL[d.idx], lw=0.6, ls=':')
        ax.set_ylabel('z (m)')
        ax.set_title(LABELS[st] + ' (solid: drone, dotted: terrain below it)', fontsize=9)
        ax.grid(True, alpha=0.3)
    axes[0].legend(fontsize=7, ncol=3)
    axes[-1].set_xlabel('Time (s)')
    plt.tight_layout()
    path = os.path.join(out_dir, 'altitude_timeseries.png')
    plt.savefig(path, dpi=120); plt.close()
    print(f'Saved: {path}')


def plot_belief_snapshots(res, out_dir):
    rows = ['greedy', 'lm_flat']
    vmax = m2d.build_prior().max()
    fig, axes = plt.subplots(2, 4, figsize=(18, 9.5))
    for ri, st in enumerate(rows):
        r = res['demo'][st]
        for ci, (tc, bel, poses, occ) in enumerate(r['snaps'][:4]):
            ax = axes[ri, ci]
            ax.imshow(bel, origin='lower', extent=[0, AREA_SIZE, 0, AREA_SIZE], cmap='hot_r',
                      vmin=0, vmax=vmax, interpolation='nearest')
            ax.contour(CELL_C, CELL_C, ZDEM, levels=np.arange(5, 40, 5), colors='grey',
                       linewidths=0.3)
            if occ is not None and occ.any():
                oy, ox = np.nonzero(occ > 0)
                ax.plot(CELL_C[ox], CELL_C[oy], 's', color='deepskyblue', ms=1.2, alpha=0.7)
            for k, p in enumerate(poses):
                ax.plot(p[0], p[1], '^', color=DCOL[k], ms=7, mec='k', mew=0.5)
            ax.plot(*r['person'], '*', color='green', ms=13, mec='k')
            ax.plot(*LKP, 'kx', ms=8, mew=2)
            end = ' (detection)' if ci == 3 and r['det_time'] is not None else (' (end)' if ci == 3 else '')
            ax.set_title(f'{LABELS[st]} — t = {tc:.0f} s{end}', fontsize=9)
            ax.set_xlim(40, 160); ax.set_ylim(0, 180)
    fig.suptitle('Belief, seed 0 (blue squares = cells that were inside a cone but hidden by terrain so far; '
                 'grey = 5 m contours; green star = person)', fontsize=11)
    plt.tight_layout()
    path = os.path.join(out_dir, 'belief_snapshots.png')
    plt.savefig(path, dpi=100); plt.close()
    print(f'Saved: {path}')


def plot_occlusion_map(res, out_dir):
    fig, axes = plt.subplots(2, len(OCC_CONES), figsize=(16, 10))
    for r_i, mode in enumerate(('terrain', 'flat')):
        for c_i, deg in enumerate(OCC_CONES):
            ax = axes[r_i, c_i]
            oc = res['occ_grid'][(deg, mode)]
            ax.imshow(ZDEM, origin='lower', extent=[0, AREA_SIZE, 0, AREA_SIZE], cmap='Greys', alpha=0.5)
            if oc.any():
                im = ax.imshow(np.ma.masked_where(oc == 0, oc), origin='lower',
                               extent=[0, AREA_SIZE, 0, AREA_SIZE], cmap='plasma', interpolation='nearest')
                plt.colorbar(im, ax=ax, fraction=0.046, label='check poses hiding the cell')
            ax.add_patch(plt.Rectangle(LKP - LM_HALF_WIDTH, 2 * LM_HALF_WIDTH, 2 * LM_HALF_WIDTH,
                                       fill=False, ec='tab:orange', lw=1))
            ax.set_title(f'{deg:.1f} deg cone, {"10 m AGL" if mode == "terrain" else f"{Z_FLAT:.1f} m MSL"}: '
                         f'{res["occ_check"][(deg, mode)]:.2f} % of in-cone cells hidden', fontsize=9)
            ax.set_xlim(40, 160); ax.set_ylim(60, 180)
    fig.suptitle(f'Terrain occlusion at {res["n_occ_poses"]} poses on the lawnmower lattice: '
                 'the card cone (11.3 deg) is never occluded, wider cones are')
    plt.tight_layout()
    path = os.path.join(out_dir, 'occlusion_map.png')
    plt.savefig(path, dpi=110); plt.close()
    print(f'Saved: {path}')


def plot_entropy(res, out_dir):
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(14, 4.5))
    for st in STRATS:
        r = res['absent'][st]
        a1.plot(r['ent_t'], r['ent_h'], color=COLORS[st], lw=1.6, label=LABELS[st])
        a2.plot(r['ent_t'], r['pdet'], color=COLORS[st], lw=1.6, label=LABELS[st])
    a1.set_xlabel('Time (s)'); a1.set_ylabel('H(B) (nats)'); a1.set_title('Belief entropy, person absent, seed 0')
    a2.set_xlabel('Time (s)'); a2.set_ylabel('E[P_det] under P0')
    a2.axhline(0.95, color='grey', ls='--', lw=1)
    a2.set_title('Model-expected detection probability along the same paths')
    for a in (a1, a2):
        a.grid(True, alpha=0.3); a.legend(fontsize=8)
    plt.tight_layout()
    path = os.path.join(out_dir, 'entropy_reduction.png')
    plt.savefig(path, dpi=130); plt.close()
    print(f'Saved: {path}')


def plot_mc_boxplot(res, out_dir):
    fig, axes = plt.subplots(1, 2, figsize=(15, 5.5), sharey=True)
    for ax, tr in zip(axes, TRUTHS):
        data = [np.where(np.isfinite(res['mc'][tr][s]['times']), res['mc'][tr][s]['times'], MAX_SIM_TIME)
                for s in STRATS]
        bp = ax.boxplot(data, tick_labels=[LABELS[s].replace(' ', '\n', 1) for s in STRATS], patch_artist=True)
        for patch, s in zip(bp['boxes'], STRATS):
            patch.set_facecolor(COLORS[s]); patch.set_alpha(0.6)
        for i, s in enumerate(STRATS):
            d = data[i]
            ax.text(i + 1, 1860, f'med {np.median(d):.0f} s\n{np.isfinite(res["mc"][tr][s]["times"]).sum()}/{N_MC} found',
                    ha='center', va='bottom', fontsize=8)
        ax.axhline(MAX_SIM_TIME, color='red', ls='--', lw=1)
        ax.set_ylim(0, 2100)
        ax.set_title(f'Person drawn from {"P0 (card)" if tr == "P0" else "the valley-biased prior"}')
    axes[0].set_ylabel('Detection time (s), not found = 1800 s')
    fig.suptitle(f'Detection time, {N_MC} trials per strategy (same persons for every strategy)')
    plt.tight_layout()
    path = os.path.join(out_dir, 'detection_boxplot.png')
    plt.savefig(path, dpi=130); plt.close()
    print(f'Saved: {path}')


def save_animation(res, out_dir):
    r = res['demo']['greedy']
    frames = r['frames']
    s = 4
    win = (slice(0, None, s), slice(0, None, s))
    Xs, Ys, Zs = XX[win], YY[win], ZDEM[win]
    keep = (Xs[0] >= 40) & (Xs[0] <= 160)
    rows = (Ys[:, 0] >= 0) & (Ys[:, 0] <= 180)
    Xs, Ys, Zs = Xs[np.ix_(rows, keep)], Ys[np.ix_(rows, keep)], Zs[np.ix_(rows, keep)]
    ref = frames[0][1].max()
    fig = plt.figure(figsize=(8, 6.5))
    ax = fig.add_subplot(111, projection='3d')
    hist = np.array([[d.pos_at(f[0]) for d in r['drones']] for f in frames])

    def update(i):
        ax.clear()
        tf, bel, poses, h = frames[i]
        bel_up = np.kron(bel, np.ones((4, 4)))[win][np.ix_(rows, keep)]
        fc = plt.cm.terrain(0.15 + 0.7 * (Zs - ZDEM.min()) / (ZDEM.max() - ZDEM.min()))
        bn = np.clip(bel_up / ref, 0, 1)
        fc = np.where(bn[..., None] > 0.05, 0.3 * fc + 0.7 * plt.cm.hot_r(bn), fc)
        ax.plot_surface(Xs, Ys, Zs, facecolors=fc, rstride=1, cstride=1, linewidth=0, shade=False, alpha=0.6)
        for k in range(N_DRONES):
            tr_ = hist[max(0, i - 15):i + 1, k]
            ax.plot(tr_[:, 0], tr_[:, 1], tr_[:, 2], color=DCOL[k], lw=1.5)
            ax.scatter(*poses[k], color=DCOL[k], s=70, edgecolors='k', depthshade=False)
        ax.scatter(*r['person'], dem(*r['person']), color='green', marker='*', s=150, depthshade=False)
        ax.set_xlim(40, 160); ax.set_ylim(0, 180); ax.set_zlim(0, 50)
        ax.view_init(elev=35, azim=-75 + 90 * i / max(len(frames) - 1, 1))
        done = r['det_time'] is not None and tf >= r['det_time']
        ax.set_title(f'Terrain-aware greedy, seed 0: t = {tf:.0f} s, H = {h:.2f} nats' + ('  FOUND' if done else ''),
                     fontsize=10)
        return []

    ani = FuncAnimation(fig, update, frames=len(frames), interval=100, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer=PillowWriter(fps=10), dpi=70)
    plt.close()
    print(f'Saved: {path}  ({len(frames)} frames)')


if __name__ == '__main__':
    print('Running S042 3D Missing Person Search simulation...')
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    data = run_simulation()
    plot_dem_prior(data, OUTPUT_DIR)
    plot_trajectories_3d(data, OUTPUT_DIR)
    plot_altitude(data, OUTPUT_DIR)
    plot_belief_snapshots(data, OUTPUT_DIR)
    plot_occlusion_map(data, OUTPUT_DIR)
    plot_entropy(data, OUTPUT_DIR)
    plot_mc_boxplot(data, OUTPUT_DIR)
    save_animation(data, OUTPUT_DIR)
    print('Done.')
