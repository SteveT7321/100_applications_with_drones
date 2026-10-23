"""
S041 3D Upgrade — Wildfire Boundary Scan
========================================
The S041 fire (elliptical Huygens spread on a 0.5 m grid, wind toward NE,
sigma_spot = 0.01 m, drones start with an all-unknown map) is placed on the
card's ridge terrain, and every burning cell now carries the card's smoke /
heat column.  Three IR drones track the boundary with the S041 guidance law
(boundary tangent + signed stand-off correction, coverage-gap and lost-contact
replanning, front search) and a separate altitude channel:

    z_cmd = max(z_terrain + z_scan, z_floor),  z_dot = (z_cmd - z)/tau_z,
    |z_dot| <= 2 m/s
    z_floor = z_terrain + 15 exp(-d_b^2 / (2 * 5^2)) + h_flame + z_safety

(d_b = distance to the nearest burning cell).  The IR footprint is the 15 deg
cone, r_foot = z_AGL tan(15 deg), and the IR noise variance grows as
25 K^2 (z_AGL / 10 m)^2.  The stand-off from the front equals the current
footprint radius (S041: the drone settles r_s outside the front).

Four configurations fly over the SAME fire realisation (the fire does not
depend on the drones; it is simulated once per seed) with the same sensor
noise stream:
  A  2D-style  : ideal 10 m AGL hold, smoke ignored (S041 behaviour, card cone)
  B  AGL only  : first-order terrain following to 10 m AGL, smoke ignored
  C  AGL+smoke : card controller, z_cmd also clears the smoke floor
  D  control   : same lagged controller with a fixed 22.5 m AGL target (the highest
                 possible smoke floor), smoke ignored — separates "flying higher"
                 from "following the smoke floor"
Metrics (all configs): Hausdorff d_H and average Hausdorff, coverage
freshness, share of drone-steps inside the smoke column (below plume top) and
inside the 2 m safety margin, footprint radius, AGL, IR misclassification
rate.  A multi-seed Monte Carlo and a fleet-size sweep answer the card's
key question.

Usage:
    conda activate drones
    python src/03_environmental_sar/3d/s041_3d_wildfire.py
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
from scipy.spatial import cKDTree
from scipy.ndimage import distance_transform_edt

# ── Reuse the fixed 2D S041 fire model and helpers ─────────────────────────
_HERE = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location(
    's041_2d', os.path.join(_HERE, '..', 's041_wildfire_boundary.py'))
m2d = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(m2d)

OUTPUT_DIR = os.path.normpath(os.path.join(
    _HERE, '..', '..', '..', 'outputs', '03_environmental_sar', '3d', 's041_3d_wildfire'))

# ── Parameters (card) ──────────────────────────────────────────────────────
N_DRONES     = 3
ARENA_SIZE   = m2d.ARENA_SIZE          # 200 m
GRID_RES     = m2d.GRID_RES            # 0.5 m
N_CELLS      = m2d.N_CELLS
CELL_C       = m2d.CELL_C
A_RIDGE      = 8.0       # m — terrain relief amplitude
SIGMA_X      = 40.0      # m — ridge width
L_Y          = 200.0     # m — ridge length scale
X_C, Y_C     = 100.0, 100.0   # m — ridge centre (added: card leaves x_c, y_c open)
Z_SCAN       = 10.0      # m — nominal scan AGL
ALPHA_S      = np.deg2rad(15.0)   # IR cone half-angle
R_SENSOR_0   = 25.0      # K^2 — IR noise variance at z_scan
V_CRUISE     = 2.5       # m/s
TAU_Z        = 1.0       # s — altitude time constant
ZDOT_MAX     = 2.0       # m/s — vertical rate limit
H0_FLAME     = 3.0       # m — calm-wind flame height
K_WIND       = 0.5       # m/(m/s)
WIND_SPEED   = 5.0       # m/s
H_FLAME      = H0_FLAME + K_WIND * WIND_SPEED   # 5.5 m (constant wind -> constant)
H_RISE       = 15.0      # m — plume rise height
SIGMA_PLUME  = 5.0       # m — plume dispersion width
PLUME_CUTOFF = 4.0 * SIGMA_PLUME   # m — burning cells farther than this are ignored (term < 0.005 m)
Z_SAFETY     = 2.0       # m
Z_MIN, Z_MAX = 0.5, 40.0 # m — absolute altitude bounds
K_PHI        = m2d.K_PHI           # 0.8 1/s
D_REPLAN     = m2d.D_REPLAN        # 3.0 m
DT_GAP       = m2d.DT_GAP          # 30 s
GAP_HOLD     = m2d.GAP_HOLD        # 10 s
R0_SPREAD    = m2d.R0_SPREAD       # 0.04 m/s
SIGMA_SPOT   = m2d.SIGMA_SPOT      # 0.01 m/step (S041 fix)
IGN_CENTRE   = m2d.IGN_CENTRE
R_INIT       = m2d.R_INIT          # 20 m
DT           = m2d.DT              # 0.5 s
T_SIM        = m2d.T_SIM           # 300 s
T_FIRE, T_AMBIENT, T_THRESH = m2d.T_FIRE, m2d.T_AMBIENT, m2d.T_THRESH
R_FOOT_NOM   = Z_SCAN * np.tan(ALPHA_S)    # 2.68 m
START_OFFSET = R_FOOT_NOM                 # drones start one nominal footprint outside the perimeter
SEED         = 0
N_MC         = 12                  # Monte-Carlo seeds 0..11 (fire + sensor noise)
FLEET_SIZES  = [1, 2, 3, 4, 5]
N_FLEET_SEEDS = 4                  # seeds 0..3 for the fleet-size sweep
ANIM_EVERY   = 10                  # steps per animation frame
Z_HIGH       = H_RISE + H_FLAME + Z_SAFETY   # 22.5 m AGL — highest possible smoke floor (config D)

CONFIGS = ['A', 'B', 'C', 'D']
CFG_LABEL = {'A': 'A 2D-style: ideal 10 m AGL, smoke ignored',
             'B': 'B AGL-following only (tau_z lag), smoke ignored',
             'C': 'C AGL + smoke floor (card)',
             'D': 'D fixed 22.5 m AGL target (control)'}
CFG_COLOR = {'A': '#7f7f7f', 'B': '#1f77b4', 'C': '#d62728', 'D': '#ff7f0e'}
DRONE_COLORS = ['#1f77b4', '#9467bd', '#17becf', '#2ca02c', '#8c564b']
UNKNOWN, UNBURNED, BURNING = m2d.UNKNOWN, m2d.UNBURNED, m2d.BURNING
MODES = ['follow', 'transit', 'search']   # guidance mode at the start of each step
N_STEPS = int(round(T_SIM / DT))


# ── Terrain / smoke ────────────────────────────────────────────────────────

def z_terrain(x, y):
    """Card ridge: A exp(-(x-xc)^2 / (2 sx^2)) cos(pi (y-yc)/Ly)."""
    return A_RIDGE * np.exp(-(x - X_C) ** 2 / (2 * SIGMA_X ** 2)) * np.cos(np.pi * (y - Y_C) / L_Y)


def nearest_burning_dist(F, x, y):
    """Distance from (x, y) to the nearest burning cell centre (inf beyond PLUME_CUTOFF)."""
    i0 = max(int((x - PLUME_CUTOFF) / GRID_RES), 0)
    i1 = min(int((x + PLUME_CUTOFF) / GRID_RES) + 2, N_CELLS)
    j0 = max(int((y - PLUME_CUTOFF) / GRID_RES), 0)
    j1 = min(int((y + PLUME_CUTOFF) / GRID_RES) + 2, N_CELLS)
    bi, bj = np.nonzero(F[i0:i1, j0:j1])
    if len(bi) == 0:
        return np.inf
    d2 = (CELL_C[i0 + bi] - x) ** 2 + (CELL_C[j0 + bj] - y) ** 2
    d = np.sqrt(d2.min())
    return d if d <= PLUME_CUTOFF else np.inf


def plume_height(d):
    return H_RISE * np.exp(-d ** 2 / (2 * SIGMA_PLUME ** 2)) if np.isfinite(d) else 0.0


def smoke_levels(F, x, y):
    """(terrain, plume top = terrain + plume + h_flame, z_floor = top + z_safety) — card formula."""
    zt = z_terrain(x, y)
    top = zt + plume_height(nearest_burning_dist(F, x, y)) + H_FLAME
    return zt, top, top + Z_SAFETY


def z_command(cfg, F, p):
    """Altitude command: A/B terrain + z_scan, C card max(terrain + z_scan, z_floor), D terrain + 22.5 m."""
    zt, _, zf = smoke_levels(F, p[0], p[1])
    if cfg == 'C':
        return max(zt + Z_SCAN, zf)
    if cfg == 'D':
        return zt + Z_HIGH
    return zt + Z_SCAN


# ── Fire (independent of the drones: simulated once per seed) ──────────────

def simulate_fire(seed):
    """Run the S041 fire alone; return ignition times and the true boundary per step."""
    rng = np.random.default_rng(seed)
    kernel = m2d.make_spread_kernel(R0_SPREAD)
    F = m2d.init_grid()
    T_ign = np.where(F, 0.0, np.inf)
    W = np.zeros(F.shape)
    txy, trees = [], []
    for step in range(N_STEPS):
        F = m2d.spread_fire(F, T_ign, W, (step + 1) * DT, kernel, rng, SIGMA_SPOT)
        ti, tj = np.nonzero(m2d.boundary_mask_true(F))
        xy = np.stack([CELL_C[ti], CELL_C[tj]], axis=1)
        txy.append(xy)
        trees.append(cKDTree(xy))
    return dict(T_ign=T_ign.copy(), txy=txy, trees=trees, seed=seed)


def fire_at(fire, step):
    """Burn state seen by the drones at step (fire advanced to t + DT, as in S041)."""
    return fire['T_ign'] <= (step + 1) * DT + 1e-9


# ── Sensor / guidance (S041 functions generalised to a variable footprint) ─

def _window(pos, r):
    i0 = max(int((pos[0] - r) / GRID_RES) - 1, 0)
    i1 = min(int((pos[0] + r) / GRID_RES) + 2, N_CELLS)
    j0 = max(int((pos[1] - r) / GRID_RES) - 1, 0)
    j1 = min(int((pos[1] + r) / GRID_RES) + 2, N_CELLS)
    X, Y = np.meshgrid(CELL_C[i0:i1], CELL_C[j0:j1], indexing='ij')
    return i0, i1, j0, j1, np.hypot(X - pos[0], Y - pos[1]) <= r


def ir_observe(F, E, last_scan, pos, r_foot, var, t, rng):
    """Noisy IR reading of every cell in the cone footprint; returns (#cells, #misclassified)."""
    i0, i1, j0, j1, fp = _window(pos, r_foot)
    truth = F[i0:i1, j0:j1]
    T_true = np.where(truth, T_FIRE, T_AMBIENT)
    z = T_true + rng.normal(0.0, np.sqrt(var), size=T_true.shape)
    obs = np.where(z >= T_THRESH, BURNING, UNBURNED)
    E[i0:i1, j0:j1][fp] = obs[fp]
    last_scan[i0:i1, j0:j1][fp] = t
    wrong = (obs == BURNING) != truth
    return int(fp.sum()), int((wrong & fp).sum())


def footprint_all_burning(E, pos, r_foot):
    i0, i1, j0, j1, fp = _window(pos, r_foot)
    return bool(np.all(E[i0:i1, j0:j1][fp] == BURNING))


def gradient_step(pos, bxy, bij, E, standoff):
    """S041 law v = v_cruise t_hat - K_phi phi n_hat, phi = s d - standoff (clipped to v_cruise)."""
    d = np.hypot(bxy[:, 0] - pos[0], bxy[:, 1] - pos[1])
    k = np.argmin(d)
    rel = pos - bxy[k]
    n_hat = m2d.outward_normal(E, bij[0][k], bij[1][k], rel)
    side = 1.0 if np.dot(rel, n_hat) >= 0 else -1.0
    t_hat = np.array([n_hat[1], -n_hat[0]])
    vel = V_CRUISE * t_hat - K_PHI * (side * d[k] - standoff) * n_hat
    sp = np.linalg.norm(vel)
    if sp > V_CRUISE:
        vel *= V_CRUISE / sp
    return vel, d[k]


def select_replan_target(bxy, ages, other_targets):
    return m2d.select_replan_target(bxy, ages, other_targets)


# ── Mission ─────────────────────────────────────────────────────────────────

def simulate(fire, cfg, n_drones=N_DRONES, record=False):
    """One mission over a pre-computed fire. cfg in {'A','B','C'}."""
    rng = np.random.default_rng([fire['seed'], 1])     # sensor noise stream (same for every cfg)
    E = np.full((N_CELLS, N_CELLS), UNKNOWN, dtype=np.int8)
    last_scan = np.full((N_CELLS, N_CELLS), -np.inf)
    angles = np.linspace(0, 2.0 * np.pi, n_drones, endpoint=False)
    pos = np.array([IGN_CENTRE + (R_INIT + START_OFFSET) * np.array([np.cos(a), np.sin(a)])
                    for a in angles])
    F0 = fire_at(fire, -1)
    z = np.zeros(n_drones)
    for k in range(n_drones):
        z[k] = z_command(cfg, F0, pos[k])         # arrive at the commanded altitude
    targets = [None] * n_drones
    search_dir = [None] * n_drones
    follow_since = np.zeros(n_drones)

    t_axis = np.arange(N_STEPS) * DT
    dH = np.zeros(N_STEPS); dAvg = np.zeros(N_STEPS); fresh = np.zeros(N_STEPS)
    traj = np.zeros((n_drones, N_STEPS, 3))
    terr = np.zeros((n_drones, N_STEPS)); top = np.zeros((n_drones, N_STEPS))
    floor = np.zeros((n_drones, N_STEPS)); rfoot = np.zeros((n_drones, N_STEPS))
    modes = np.zeros((n_drones, N_STEPS), dtype=int)
    events = []
    n_obs = n_wrong = 0
    anim = []

    for step in range(N_STEPS):
        t = t_axis[step]
        F = fire_at(fire, step)
        r_now = np.zeros(n_drones)
        for k in range(n_drones):
            zt = z_terrain(*pos[k])
            agl = z[k] - zt
            assert agl > 0.0, 'drone underground'
            r_now[k] = agl * np.tan(ALPHA_S)
            var = R_SENSOR_0 * (agl / Z_SCAN) ** 2
            a, b = ir_observe(F, E, last_scan, pos[k], r_now[k], var, t, rng)
            n_obs += a; n_wrong += b
        b_est = m2d.boundary_mask_est(E)
        bi, bj = np.nonzero(b_est)
        bxy = np.stack([CELL_C[bi], CELL_C[bj]], axis=1)
        ages = t - last_scan[bi, bj]
        for k in range(n_drones):
            p_old = pos[k].copy()
            # altitude command at the current position (card: z_cmd(x_k, y_k))
            z_cmd = z_command(cfg, F, pos[k])
            mode = ('search' if search_dir[k] is not None else
                    'transit' if targets[k] is not None or len(bxy) == 0 else 'follow')
            # horizontal guidance (S041), stand-off = current footprint radius
            rf = r_now[k]
            if search_dir[k] is not None:
                d_near = (np.min(np.hypot(bxy[:, 0] - pos[k, 0], bxy[:, 1] - pos[k, 1]))
                          if len(bxy) else np.inf)
                if d_near <= rf + D_REPLAN or not footprint_all_burning(E, pos[k], rf):
                    search_dir[k] = None
                    follow_since[k] = t
                else:
                    pos[k] += V_CRUISE * DT * search_dir[k]
            elif targets[k] is not None:
                diff = targets[k] - pos[k]
                dist = np.linalg.norm(diff)
                if dist <= V_CRUISE * DT:
                    pos[k] = targets[k]
                    targets[k] = None
                    follow_since[k] = t
                else:
                    pos[k] += V_CRUISE * DT * diff / dist
            elif len(bxy) == 0:
                u = IGN_CENTRE - pos[k]
                pos[k] += V_CRUISE * DT * u / (np.linalg.norm(u) + 1e-9)
            else:
                vel, d_fire = gradient_step(pos[k], bxy, (bi, bj), E, rf)
                pos[k] += vel * DT
                if d_fire > rf + D_REPLAN and footprint_all_burning(E, pos[k], rf):
                    bu, bv = np.nonzero(E == BURNING)
                    c = np.array([CELL_C[bu].mean(), CELL_C[bv].mean()])
                    u = pos[k] - c
                    search_dir[k] = u / (np.linalg.norm(u) + 1e-9)
                    events.append((t, k, 'search'))
                elif d_fire > rf + D_REPLAN:
                    targets[k] = select_replan_target(bxy, ages, targets[:k] + targets[k + 1:])
                    events.append((t, k, 'lost'))
                elif np.any(ages > DT_GAP) and t - follow_since[k] >= GAP_HOLD:
                    targets[k] = select_replan_target(bxy, ages, targets[:k] + targets[k + 1:])
                    events.append((t, k, 'gap'))
            pos[k] = np.clip(pos[k], 0.0, ARENA_SIZE)
            assert np.linalg.norm(pos[k] - p_old) <= V_CRUISE * DT + 1e-9, 'horizontal speed limit'
            # altitude channel
            if cfg == 'A':
                z_new = z_terrain(*pos[k]) + Z_SCAN            # ideal AGL hold (S041 fixed height)
            else:
                zdot = np.clip((z_cmd - z[k]) / TAU_Z, -ZDOT_MAX, ZDOT_MAX)
                z_new = z[k] + zdot * DT
                assert abs(z_new - z[k]) <= ZDOT_MAX * DT + 1e-9
            z[k] = np.clip(z_new, Z_MIN, Z_MAX)
            # bookkeeping at the new position (same fire state)
            zt2, top2, zf2 = smoke_levels(F, *pos[k])
            assert z[k] > zt2, 'drone underground'
            traj[k, step] = (pos[k, 0], pos[k, 1], z[k])
            modes[k, step] = MODES.index(mode)
            terr[k, step], top[k, step], floor[k, step] = zt2, top2, zf2
            rfoot[k, step] = (z[k] - zt2) * np.tan(ALPHA_S)
        tree = fire['trees'][step]
        if len(bxy) and len(fire['txy'][step]):
            d_te, _ = cKDTree(bxy).query(fire['txy'][step])
            d_et, _ = tree.query(bxy)
            dH[step] = 0.5 * (d_te.max() + d_et.max())
            dAvg[step] = 0.5 * (d_te.mean() + d_et.mean())
        else:
            dH[step] = dAvg[step] = np.nan
        fresh[step] = np.mean(ages <= DT_GAP) if len(ages) else np.nan
        if record and (step % ANIM_EVERY == 0 or step == N_STEPS - 1):
            anim.append((step, bxy.copy(), traj[:, step].copy()))

    agl = traj[:, :, 2] - terr
    late = t_axis >= DT_GAP
    smoke_mask = traj[:, :, 2] < top
    smoke_by_mode = {m: int(np.sum(smoke_mask & (modes == i))) for i, m in enumerate(MODES)}
    steps_by_mode = {m: int(np.sum(modes == i)) for i, m in enumerate(MODES)}
    cnt = {e: sum(1 for ev in events if ev[2] == e) for e in ('lost', 'gap', 'search')}
    return dict(cfg=cfg, n=n_drones, t_axis=t_axis, dH=dH, dAvg=dAvg, fresh=fresh,
                traj=traj, terr=terr, top=top, floor=floor, rfoot=rfoot, agl=agl,
                events=events, anim=anim, seed=fire['seed'], modes=modes,
                smoke_by_mode=smoke_by_mode, steps_by_mode=steps_by_mode,
                mean_dH=np.nanmean(dH), mean_dAvg=np.nanmean(dAvg), final_dAvg=dAvg[-1],
                mean_fresh=100 * np.nanmean(fresh[late]),
                in_smoke=100 * np.mean(traj[:, :, 2] < top),
                in_margin=100 * np.mean(traj[:, :, 2] < floor),
                max_depth=float(np.max(top - traj[:, :, 2])),
                mean_rfoot=float(rfoot.mean()), mean_agl=float(agl.mean()),
                max_agl=float(agl.max()), min_agl=float(agl.min()),
                rms_agl_dev=float(np.sqrt(np.mean((agl - Z_SCAN) ** 2))),
                misclass=100 * n_wrong / max(n_obs, 1), n_obs=n_obs, n_wrong=n_wrong,
                n_lost=cnt['lost'], n_gap=cnt['gap'], n_search=cnt['search'])


def floor_fixed_point():
    """Stand-off s = r_foot = z_AGL tan(a) with z_AGL = plume(s) + h_flame + z_safety (front-side
    equilibrium of config C, straight front, nearest burning cell at distance s)."""
    s = R_FOOT_NOM
    for _ in range(200):
        s = (plume_height(s) + H_FLAME + Z_SAFETY) * np.tan(ALPHA_S)
    return s, s / np.tan(ALPHA_S)


def run_simulation():
    print('=== Card arithmetic ===')
    print(f'h_flame = {H0_FLAME} + {K_WIND} * {WIND_SPEED} = {H_FLAME:.2f} m')
    print(f'Nominal footprint r_foot(10 m) = {R_FOOT_NOM:.3f} m; r_foot(20 m) = {20 * np.tan(ALPHA_S):.3f} m')
    print(f'gamma_max = atan({ZDOT_MAX}/{V_CRUISE}) = {np.degrees(np.arctan(ZDOT_MAX / V_CRUISE)):.1f} deg')
    X, Y = np.meshgrid(CELL_C, CELL_C, indexing='ij')
    ZT = z_terrain(X, Y)
    print(f'Terrain range over the arena: {ZT.min():.2f} .. {ZT.max():.2f} m')
    d_free = np.sqrt(-2 * SIGMA_PLUME ** 2 * np.log((Z_SCAN - H_FLAME - Z_SAFETY) / H_RISE))
    print(f'Smoke floor binds (z_floor > z_t + z_scan) within {d_free:.2f} m of a burning cell')
    print(f'Floor directly above a burning cell: {H_RISE + H_FLAME + Z_SAFETY:.1f} m AGL')
    s_eq, z_eq = floor_fixed_point()
    print(f'Config C straight-front equilibrium: stand-off = r_foot = {s_eq:.2f} m at {z_eq:.2f} m AGL')
    for zz in (Z_SCAN, z_eq, H_RISE + H_FLAME + Z_SAFETY):
        sig = np.sqrt(R_SENSOR_0) * zz / Z_SCAN
        print(f'  IR noise at {zz:5.2f} m AGL: sigma = {sig:5.2f} K, threshold margin '
              f'{(T_THRESH - T_AMBIENT) / sig:5.1f} sigma')

    fires = {}
    mc = {c: [] for c in CONFIGS}
    main = {}
    print(f'=== Monte Carlo: {N_MC} seeds x {len(CONFIGS)} configs, N = {N_DRONES} '
          f'(same fire + sensor stream per seed) ===')
    for seed in range(N_MC):
        fires[seed] = simulate_fire(seed)
        for c in CONFIGS:
            r = simulate(fires[seed], c, record=(seed == SEED))
            if seed == SEED:
                main[c] = r
            mc[c].append(r)
            print(f'seed {seed} {c}: mean d_H {r["mean_dH"]:.2f} m, mean avg-H {r["mean_dAvg"]:.3f} m, '
                  f'freshness(t>=30) {r["mean_fresh"]:.1f} %, in smoke {r["in_smoke"]:.1f} %, '
                  f'in margin {r["in_margin"]:.1f} %, mean r_foot {r["mean_rfoot"]:.2f} m, '
                  f'mean AGL {r["mean_agl"]:.2f} m, misclass {r["misclass"]:.4f} %, '
                  f'replans lost/gap {r["n_lost"]}/{r["n_gap"]}, searches {r["n_search"]}')
    print('=== Monte-Carlo summary (mean +- std over seeds; [min, max]) ===')
    keys = ['mean_dH', 'mean_dAvg', 'mean_fresh', 'in_smoke', 'in_margin', 'max_depth',
            'mean_rfoot', 'mean_agl', 'min_agl', 'max_agl', 'rms_agl_dev', 'misclass']
    summ = {}
    for c in CONFIGS:
        summ[c] = {}
        for kk in keys:
            v = np.array([r[kk] for r in mc[c]])
            summ[c][kk] = (v.mean(), v.std(), v.min(), v.max())
            print(f'{c} {kk:12s}: {v.mean():8.3f} +- {v.std():7.3f}  [{v.min():8.3f}, {v.max():8.3f}]')
    print('=== Head-to-head on identical fires (seeds won, lower is better unless noted) ===')
    for a, b in [('C', 'A'), ('C', 'B'), ('B', 'A'), ('D', 'C')]:
        for kk, better in [('mean_dAvg', 'low'), ('mean_dH', 'low'), ('mean_fresh', 'high')]:
            va = np.array([r[kk] for r in mc[a]]); vb = np.array([r[kk] for r in mc[b]])
            wins = int(np.sum(va < vb)) if better == 'low' else int(np.sum(va > vb))
            print(f'{a} better than {b} on {kk}: {wins}/{N_MC} seeds '
                  f'(mean diff {np.mean(va - vb):+.3f})')
    print('=== Where config C enters the smoke column (all seeds, drone-steps) ===')
    for mname in MODES:
        n_sm = sum(r['smoke_by_mode'][mname] for r in mc['C'])
        n_st = sum(r['steps_by_mode'][mname] for r in mc['C'])
        print(f'C {mname:8s}: {n_st} drone-steps ({100 * n_st / (N_MC * N_DRONES * N_STEPS):.1f} %), '
              f'in smoke {n_sm} ({100 * n_sm / max(n_st, 1):.1f} % of this mode)')
    nm = mc['C'][0]
    print(f'Seed 0 config C: final avg-H {nm["final_dAvg"]:.3f} m; deepest smoke penetration '
          f'{nm["max_depth"]:.2f} m (negative = always above plume top)')

    print(f'=== Fleet-size sweep (configs A and C, seeds 0..{N_FLEET_SEEDS - 1}) ===')
    fleet = {c: {} for c in ('A', 'C')}
    for n in FLEET_SIZES:
        for c in ('A', 'C'):
            rs = [mc[c][s] if n == N_DRONES else simulate(fires[s], c, n_drones=n)
                  for s in range(N_FLEET_SEEDS)]
            fleet[c][n] = dict(dAvg=np.mean([r['mean_dAvg'] for r in rs]),
                               dH=np.mean([r['mean_dH'] for r in rs]),
                               fresh=np.mean([r['mean_fresh'] for r in rs]),
                               smoke=np.mean([r['in_smoke'] for r in rs]),
                               each=[float(r['mean_dAvg']) for r in rs])
            f = fleet[c][n]
            each = ', '.join(f'{v:.3f}' for v in f['each'])
            print(f'N = {n} {c}: mean avg-H {f["dAvg"]:.3f} m (per seed {each}), mean d_H {f["dH"]:.2f} m, '
                  f'freshness(t>=30) {f["fresh"]:.1f} %, in smoke {f["smoke"]:.1f} %')
    return dict(main=main, mc=mc, summ=summ, fleet=fleet, fire0=fires[SEED])


# ── Plots ───────────────────────────────────────────────────────────────────
ZOOM = (60.0, 140.0)


def _terrain_mesh(step=2.0):
    g = np.arange(ZOOM[0], ZOOM[1] + 1e-9, step)
    X, Y = np.meshgrid(g, g, indexing='ij')
    return g, X, Y, z_terrain(X, Y)


def _floor_grid(F, X, Y, ZT):
    """Card smoke floor on a mesh (EDT distance to the nearest burning cell)."""
    dist = distance_transform_edt(~F) * GRID_RES
    ii = np.clip((X / GRID_RES).astype(int), 0, N_CELLS - 1)
    jj = np.clip((Y / GRID_RES).astype(int), 0, N_CELLS - 1)
    d = dist[ii, jj]
    d = np.where(d <= PLUME_CUTOFF, d, np.inf)
    plume = np.where(np.isfinite(d), H_RISE * np.exp(-np.where(np.isfinite(d), d, 0) ** 2 /
                                                      (2 * SIGMA_PLUME ** 2)), 0.0)
    return ZT + plume + H_FLAME + Z_SAFETY, plume


def _burn_on_mesh(F, X, Y):
    ii = np.clip((X / GRID_RES).astype(int), 0, N_CELLS - 1)
    jj = np.clip((Y / GRID_RES).astype(int), 0, N_CELLS - 1)
    return F[ii, jj]


def _draw_scene(ax, F, X, Y, ZT, show_floor=True):
    burn = _burn_on_mesh(F, X, Y)
    fc = np.where(burn[..., None], np.array([1.0, 0.55, 0.26, 0.9]), np.array([0.6, 0.6, 0.6, 0.5]))
    ax.plot_surface(X, Y, ZT, facecolors=fc, rstride=1, cstride=1, linewidth=0, shade=False)
    if show_floor:
        ZF, plume = _floor_grid(F, X, Y, ZT)
        mask = plume > (Z_SCAN - H_FLAME - Z_SAFETY)       # where the floor rises above z_t + z_scan
        ZF = np.where(mask, ZF, np.nan)
        ax.plot_surface(X, Y, ZF, color='#555555', alpha=0.18, linewidth=0, shade=False)


def plot_trajectory_3d(res, out_dir):
    r = res['main']['C']
    F = fire_at(res['fire0'], N_STEPS - 1)
    g, X, Y, ZT = _terrain_mesh()
    fig = plt.figure(figsize=(11, 8.5))
    ax = fig.add_subplot(111, projection='3d')
    _draw_scene(ax, F, X, Y, ZT)
    t = r['t_axis']
    for k in range(r['n']):
        tr = r['traj'][k]
        sc = ax.scatter(tr[:, 0], tr[:, 1], tr[:, 2], c=t, cmap='coolwarm', s=2, depthshade=False)
        ax.scatter(*tr[0], marker='s', s=50, color=DRONE_COLORS[k], edgecolors='k')
        ax.scatter(*tr[-1], marker='*', s=120, color=DRONE_COLORS[k], edgecolors='k')
    fig.colorbar(sc, ax=ax, shrink=0.6, label='time (s)')
    ax.set_xlim(*ZOOM); ax.set_ylim(*ZOOM); ax.set_zlim(0, 32)
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)'); ax.set_zlabel('z (m)')
    ax.view_init(elev=28, azim=-60)
    ax.legend(handles=[Line2D([], [], color='#999999', lw=6, label='Terrain (ridge)'),
                       Line2D([], [], color='#ff8c42', lw=6, label='Burned area t = 300 s'),
                       Line2D([], [], color='#555555', lw=6, alpha=0.4,
                              label='Smoke floor (where above 10 m AGL)'),
                       Line2D([], [], marker='s', color='k', ls='', label='start'),
                       Line2D([], [], marker='*', color='k', ls='', ms=10, label='end')],
              loc='upper left', fontsize=8)
    ax.set_title('S041 3D — config C (AGL + smoke floor), seed 0: paths coloured by time')
    plt.tight_layout()
    path = os.path.join(out_dir, 'trajectory_3d.png')
    plt.savefig(path, dpi=130); plt.close()
    print(f'Saved: {path}')


def plot_altitude_timeseries(res, out_dir):
    r = res['main']['C']
    t = r['t_axis']
    fig, axes = plt.subplots(r['n'], 1, figsize=(11, 8), sharex=True)
    for k, ax in enumerate(axes):
        ax.fill_between(t, r['terr'][k], r['floor'][k], color='#bbbbbb', alpha=0.6,
                        label='keep-out zone (terrain -> smoke floor)')
        ax.plot(t, r['terr'][k], color='#6b4f2a', lw=1.0, label='terrain $z_{terrain}(x_k,y_k)$')
        ax.plot(t, r['top'][k], color='#444444', lw=0.8, ls=':', label='plume top')
        ax.plot(t, r['floor'][k], color='k', lw=0.8, ls='--', label='smoke floor $z_{floor}$')
        ax.plot(t, r['terr'][k] + Z_SCAN, color='#2ca02c', lw=0.8, label='terrain + $z_{scan}$')
        ax.plot(t, r['traj'][k, :, 2], color=DRONE_COLORS[k], lw=1.4, label=f'drone {k + 1} $z_k$')
        ax.set_ylabel('z (m)')
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=7, ncol=3, loc='lower right')
    axes[-1].set_xlabel('Time (s)')
    fig.suptitle('Config C, seed 0: drone altitude vs terrain and smoke floor at the drone position')
    plt.tight_layout()
    path = os.path.join(out_dir, 'altitude_timeseries.png')
    plt.savefig(path, dpi=130); plt.close()
    print(f'Saved: {path}')


def plot_agl_footprint(res, out_dir):
    t = res['main']['C']['t_axis']
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    for c in ('B', 'C', 'D'):
        r = res['main'][c]
        for k in range(r['n']):
            a1.plot(t, r['agl'][k] - Z_SCAN, color=CFG_COLOR[c], lw=0.9, alpha=0.8,
                    label=CFG_LABEL[c] if k == 0 else None)
        a2.plot(t, r['rfoot'].mean(axis=0), color=CFG_COLOR[c], lw=1.4, label=CFG_LABEL[c])
    a1.axhline(0, color='k', lw=0.6)
    a1.set_ylabel('$z_{AGL} - z_{scan}$ (m)')
    a1.set_title('AGL deviation from 10 m, all drones (seed 0)')
    a1.legend(fontsize=8); a1.grid(True, alpha=0.3)
    a2.axhline(R_FOOT_NOM, color='#2ca02c', ls='--', lw=1, label=f'nominal {R_FOOT_NOM:.2f} m')
    a2.set_ylabel(r'$\bar r_{foot}$ (m)')
    a2.set_xlabel('Time (s)')
    a2.set_title('Mean IR footprint radius')
    a2.legend(fontsize=8); a2.grid(True, alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'agl_footprint.png')
    plt.savefig(path, dpi=130); plt.close()
    print(f'Saved: {path}')


def plot_tracking_comparison(res, out_dir):
    t = res['main']['A']['t_axis']
    fig, axes = plt.subplots(2, 2, figsize=(13, 8))
    for c in CONFIGS:
        r = res['main'][c]
        axes[0, 0].plot(t, r['dAvg'], color=CFG_COLOR[c], lw=1.1, label=CFG_LABEL[c])
        axes[0, 1].plot(t, r['dH'], color=CFG_COLOR[c], lw=1.1, label=CFG_LABEL[c])
        axes[1, 0].plot(t, 100 * r['fresh'], color=CFG_COLOR[c], lw=1.1, label=CFG_LABEL[c])
    axes[0, 0].set_ylabel('average Hausdorff (m)'); axes[0, 0].set_title('Average Hausdorff, seed 0')
    axes[0, 1].set_ylabel('$d_H$ (m)'); axes[0, 1].set_title('Max-based $d_H$, seed 0')
    axes[1, 0].set_ylabel('freshness (%)'); axes[1, 0].set_title(r'Coverage freshness $\eta(t)$, seed 0')
    axes[1, 0].axhline(90, color='k', ls='--', lw=0.8)
    for ax in (axes[0, 0], axes[0, 1], axes[1, 0]):
        ax.set_xlabel('Time (s)'); ax.grid(True, alpha=0.3); ax.legend(fontsize=7)
    ax = axes[1, 1]
    s = res['summ']
    metrics = [('mean_dAvg', 'mean avg-H (m)'), ('mean_dH', 'mean $d_H$ (m)'),
               ('in_smoke', 'in smoke (%)/10')]
    xs = np.arange(len(metrics))
    for i, c in enumerate(CONFIGS):
        vals = [s[c][m][0] / (10 if m == 'in_smoke' else 1) for m, _ in metrics]
        errs = [s[c][m][1] / (10 if m == 'in_smoke' else 1) for m, _ in metrics]
        ax.bar(xs + (i - 1.5) * 0.2, vals, 0.2, yerr=errs, color=CFG_COLOR[c], label=c, capsize=3)
    ax.set_xticks(xs, [lab for _, lab in metrics])
    ax.set_title(f'Monte Carlo, {N_MC} seeds (mean +- std)')
    ax.legend(fontsize=8); ax.grid(True, axis='y', alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'tracking_comparison.png')
    plt.savefig(path, dpi=130); plt.close()
    print(f'Saved: {path}')


def plot_fleet_sweep(res, out_dir):
    fl = res['fleet']
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(11, 4))
    for c in ('A', 'C'):
        a1.plot(FLEET_SIZES, [fl[c][n]['dAvg'] for n in FLEET_SIZES], 'o-', color=CFG_COLOR[c], label=CFG_LABEL[c])
        a2.plot(FLEET_SIZES, [fl[c][n]['fresh'] for n in FLEET_SIZES], 'o-', color=CFG_COLOR[c], label=CFG_LABEL[c])
    a1.set_xlabel('Fleet size N'); a1.set_ylabel('mean average Hausdorff (m)')
    a2.set_xlabel('Fleet size N'); a2.set_ylabel('mean freshness, t >= 30 s (%)')
    for a in (a1, a2):
        a.grid(True, alpha=0.3); a.legend(fontsize=7); a.set_xticks(FLEET_SIZES)
    fig.suptitle(f'Fleet-size sweep (mean of seeds 0..{N_FLEET_SEEDS - 1}, same fires)')
    plt.tight_layout()
    path = os.path.join(out_dir, 'fleet_sweep.png')
    plt.savefig(path, dpi=130); plt.close()
    print(f'Saved: {path}')


def save_animation(res, out_dir):
    r = res['main']['C']
    frames = r['anim']
    g, X, Y, ZT = _terrain_mesh(step=2.5)
    fig = plt.figure(figsize=(7.5, 6.5))
    ax = fig.add_subplot(111, projection='3d')

    def update(i):
        ax.clear()
        step, bxy, p = frames[i]
        F = fire_at(res['fire0'], step)
        _draw_scene(ax, F, X, Y, ZT)
        if len(bxy):
            ax.scatter(bxy[:, 0], bxy[:, 1], z_terrain(bxy[:, 0], bxy[:, 1]) + 0.3, s=0.4,
                       color='#d62728')
        for k in range(r['n']):
            tr = r['traj'][k, max(0, step - 60):step + 1]
            ax.plot(tr[:, 0], tr[:, 1], tr[:, 2], color=DRONE_COLORS[k], lw=1.2)
            ax.scatter(*p[k], color=DRONE_COLORS[k], s=30, edgecolors='k')
            ax.plot([p[k, 0]] * 2, [p[k, 1]] * 2, [z_terrain(*p[k][:2]), p[k, 2]],
                    color=DRONE_COLORS[k], lw=0.5, ls=':')
        ax.set_xlim(*ZOOM); ax.set_ylim(*ZOOM); ax.set_zlim(0, 32)
        ax.set_xlabel('x'); ax.set_ylabel('y'); ax.set_zlabel('z')
        ax.view_init(elev=30, azim=-60 + 120 * i / max(len(frames) - 1, 1))
        ax.set_title(f'S041 3D config C   t = {step * DT:.0f} s', fontsize=10)
        return []

    ani = FuncAnimation(fig, update, frames=len(frames), interval=100, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer=PillowWriter(fps=10), dpi=70)
    plt.close()
    print(f'Saved: {path}  ({len(frames)} frames)')


if __name__ == '__main__':
    print('Running S041 3D Wildfire Boundary Scan simulation...')
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    data = run_simulation()
    plot_trajectory_3d(data, OUTPUT_DIR)
    plot_altitude_timeseries(data, OUTPUT_DIR)
    plot_agl_footprint(data, OUTPUT_DIR)
    plot_tracking_comparison(data, OUTPUT_DIR)
    plot_fleet_sweep(data, OUTPUT_DIR)
    save_animation(data, OUTPUT_DIR)
    print('Done.')
