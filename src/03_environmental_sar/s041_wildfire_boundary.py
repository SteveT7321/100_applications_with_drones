"""
S041 Wildfire Boundary Scan
============================
A fleet of 3 IR-equipped drones tracks the expanding boundary of an active
wildfire in a 200x200 m arena. The fire spreads with an elliptical
(Huygens / Rothermel-style) rate of spread R(theta) = R0 (1-e)/(1 - e cos theta)
driven by a wind blowing toward the NE (45 deg). Spread is implemented as a
cellular automaton with fractional (sub-cell) burn progress on a 16-direction
stencil, so slow spread rates still move the front.

The drones start with NO map of the fire (every cell "unknown"). They build a
shared three-state occupancy grid (unknown / unburned / burning) from noisy IR
readings, patrol the boundary with a gradient-following guidance law and
replan when contact is lost or a boundary cell has not been scanned for 30 s.
Metrics: Hausdorff distance between the true and estimated boundary, coverage
freshness, and replanning events by trigger type. A small sweep over the
head-fire rate R0 shows how tracking error grows with spread rate.

Usage:
    conda activate drones
    python src/03_environmental_sar/s041_wildfire_boundary.py
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.lines import Line2D
from scipy.spatial import cKDTree

# ── Parameters ─────────────────────────────────────────────────────────────
N_DRONES       = 3
ARENA_SIZE     = 200.0     # m — square arena side length
GRID_RES       = 0.5       # m — occupancy grid cell size
SCAN_HEIGHT    = 10.0      # m — drone altitude (fixed, informational)
SENSOR_RADIUS  = 2.0       # m — IR footprint radius on ground
R_SENSOR_VAR   = 25.0      # K^2 — IR sensor noise variance
T_FIRE         = 800.0     # K — temperature of burning cells
T_AMBIENT      = 300.0     # K — background temperature
T_THRESH       = 550.0     # K — burn/no-burn classification threshold
V_CRUISE       = 2.5       # m/s — drone cruise speed
K_PHI          = 0.8       # s^-1 — stand-off correction gain
D_REPLAN       = 3.0       # m — lost-contact replanning distance
DT_GAP         = 30.0      # s — max allowed scan gap per boundary cell
R0_SPREAD      = 0.04      # m/s — head-fire rate of spread
ECC            = 0.6       # ellipse eccentricity e_w  (R_head/R_back = 4)
WIND_DIR_DEG   = 45.0      # deg — direction the wind blows TOWARD (CCW from +x)
SIGMA_SPOT     = 0.01      # m — spotting noise std per timestep (card: 0.05, see README)
SIGMA_SPOT_CARD = 0.05     # m — card value, used only in the spread diagnostic
GAP_HOLD       = 10.0      # s — after reaching a replan target a drone follows the front this long before another gap replan
NORMAL_WIN     = 6         # cells — half-window for the local outward-normal estimate (3 m)
IGN_CENTRE     = np.array([100.0, 100.0])  # m — ignition point
R_INIT         = 20.0      # m — initial fire radius
DT             = 0.5       # s — simulation timestep
T_SIM          = 300.0     # s — total simulation duration
START_OFFSET   = 1.0       # m — drones start this far outside the reported perimeter
RESERVE_RADIUS = 10.0      # m — replan targets closer than this to another drone's target are skipped
SWEEP_R0       = [0.02, 0.04, 0.08, 0.16]  # m/s — head-fire rates for the sweep
SEED           = 0
ANIM_EVERY     = 4         # record an animation frame every 4 steps (150 frames)

N_CELLS = int(ARENA_SIZE / GRID_RES)  # 400
UNKNOWN, UNBURNED, BURNING = -1, 0, 1

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '03_environmental_sar', 's041_wildfire_boundary',
)

# 16-direction stencil (all offsets with |di|,|dj| <= 2 and gcd = 1)
STENCIL = [(1, 0), (0, 1), (-1, 0), (0, -1),
           (1, 1), (1, -1), (-1, 1), (-1, -1),
           (2, 1), (1, 2), (-1, 2), (-2, 1),
           (-2, -1), (-1, -2), (1, -2), (2, -1)]

CELL_C = np.arange(N_CELLS) * GRID_RES + GRID_RES / 2.0   # cell-centre coords


# ── Fire model ─────────────────────────────────────────────────────────────

def rate_of_spread(theta, r0, ecc):
    """Elliptical rate of spread: R0 at the head (theta=0), R0(1-e)/(1+e) at the back."""
    return r0 * (1.0 - ecc) / (1.0 - ecc * np.cos(theta))


def init_grid():
    """Binary burn grid: disc of radius R_INIT centred at IGN_CENTRE."""
    X, Y = np.meshgrid(CELL_C, CELL_C, indexing='ij')
    return np.hypot(X - IGN_CENTRE[0], Y - IGN_CENTRE[1]) <= R_INIT


def make_spread_kernel(r0):
    """Rate of spread R(theta_k) and length L_k for every stencil offset."""
    wind = np.deg2rad(WIND_DIR_DEG)
    rates, lens = [], []
    for di, dj in STENCIL:
        theta = np.arctan2(dj, di) - wind
        rates.append(rate_of_spread(theta, r0, ECC))
        lens.append(np.hypot(di, dj) * GRID_RES)
    return np.array(rates), np.array(lens)


def spread_fire(F, T_ign, noise_acc, t, kernel, rng, sigma=SIGMA_SPOT):
    """
    Advance the fire to time t (Huygens principle on a 16-direction stencil).
    A burning cell n ignited at T_ign(n) has pushed its front a distance
    R(theta_nc) (t - T_ign(n)) toward neighbour c.  Cell c also carries the
    accumulated spotting perturbation W_c = sum eps_i, eps_i ~ N(0, sigma_spot^2)
    drawn once per step while c is adjacent to the fire (zero-mean, unclamped,
    so the mean spread rate is exactly R(theta)).  c ignites when
        max_n [R(theta_nc) (t - T_ign(n)) - L_nc] + W_c >= 0.
    Sub-cell progress is therefore carried in continuous time, so slow fronts
    advance correctly.  Burned cells never revert.
    """
    rates, lens = kernel
    P = 2
    Tp = np.pad(T_ign, P, constant_values=np.inf)
    reach = np.full(F.shape, -np.inf)
    for (di, dj), r, L in zip(STENCIL, rates, lens):
        src_T = Tp[P - di:P - di + N_CELLS, P - dj:P - dj + N_CELLS]   # source = target - offset
        np.maximum(reach, r * (t - src_T) - L, out=reach)
    active = np.isfinite(reach) & (~F)
    idx = np.nonzero(active)
    noise_acc[idx] += rng.normal(0.0, sigma, size=len(idx[0]))
    ignite = active & (reach + noise_acc >= 0.0)
    F[ignite] = True
    T_ign[ignite] = t
    return F


def boundary_mask_true(F):
    """Burning cells 4-adjacent to an unburned cell."""
    Fp = np.pad(F, 1, constant_values=False)
    nb_unburned = np.zeros_like(F)
    for di, dj in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
        nb_unburned |= ~Fp[1 + di:N_CELLS + 1 + di, 1 + dj:N_CELLS + 1 + dj]
    return F & nb_unburned


def boundary_mask_est(E):
    """Estimated boundary: cells observed burning that are 4-adjacent to a cell observed unburned."""
    Ep = np.pad(E, 1, constant_values=UNKNOWN)
    nb_unburned = np.zeros(E.shape, dtype=bool)
    for di, dj in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
        nb_unburned |= Ep[1 + di:N_CELLS + 1 + di, 1 + dj:N_CELLS + 1 + dj] == UNBURNED
    return (E == BURNING) & nb_unburned


# ── Sensor / guidance ──────────────────────────────────────────────────────

def ir_observe(F_true, E, last_scan, pos, t, rng):
    """Noisy IR reading of every cell in the footprint; overwrite the shared estimate."""
    i0 = max(int((pos[0] - SENSOR_RADIUS) / GRID_RES) - 1, 0)
    i1 = min(int((pos[0] + SENSOR_RADIUS) / GRID_RES) + 2, N_CELLS)
    j0 = max(int((pos[1] - SENSOR_RADIUS) / GRID_RES) - 1, 0)
    j1 = min(int((pos[1] + SENSOR_RADIUS) / GRID_RES) + 2, N_CELLS)
    X, Y = np.meshgrid(CELL_C[i0:i1], CELL_C[j0:j1], indexing='ij')
    fp = np.hypot(X - pos[0], Y - pos[1]) <= SENSOR_RADIUS
    T_true = np.where(F_true[i0:i1, j0:j1], T_FIRE, T_AMBIENT)
    z = T_true + rng.normal(0.0, np.sqrt(R_SENSOR_VAR), size=T_true.shape)
    obs = np.where(z >= T_THRESH, BURNING, UNBURNED)
    E_win = E[i0:i1, j0:j1]
    E_win[fp] = obs[fp]
    ls_win = last_scan[i0:i1, j0:j1]
    ls_win[fp] = t


def outward_normal(E, ci, cj, fallback):
    """Local outward boundary normal: from the centroid of observed-burning cells to the
    centroid of observed-unburned cells within NORMAL_WIN cells of boundary cell (ci, cj)."""
    i0, i1 = max(ci - NORMAL_WIN, 0), min(ci + NORMAL_WIN + 1, N_CELLS)
    j0, j1 = max(cj - NORMAL_WIN, 0), min(cj + NORMAL_WIN + 1, N_CELLS)
    win = E[i0:i1, j0:j1]
    bi, bj = np.nonzero(win == BURNING)
    ui, uj = np.nonzero(win == UNBURNED)
    if len(bi) == 0 or len(ui) == 0:
        n = fallback
    else:
        n = np.array([ui.mean() - bi.mean(), uj.mean() - bj.mean()])
    nn = np.linalg.norm(n)
    return n / nn if nn > 1e-9 else np.array([1.0, 0.0])


def gradient_step(pos, bxy, bij, E):
    """
    Boundary gradient-following velocity  v = v_cruise t_hat - K_phi phi n_hat
    (clipped to v_cruise).  n_hat is the local outward normal of the estimated
    boundary, phi = s d - r_s with s = +1 when the drone is on the unburned side
    and -1 when it is over the burned side, so the drone always settles r_s
    OUTSIDE the front.  t_hat = clockwise tangent.
    """
    if len(bxy) == 0:
        return np.zeros(2), np.inf
    d = np.hypot(bxy[:, 0] - pos[0], bxy[:, 1] - pos[1])
    k = np.argmin(d)
    d_fire = d[k]
    rel = pos - bxy[k]
    n_hat = outward_normal(E, bij[0][k], bij[1][k], rel)
    side = 1.0 if np.dot(rel, n_hat) >= 0 else -1.0
    t_hat = np.array([n_hat[1], -n_hat[0]])       # clockwise tangent
    phi = side * d_fire - SENSOR_RADIUS
    vel = V_CRUISE * t_hat - K_PHI * phi * n_hat
    sp = np.linalg.norm(vel)
    if sp > V_CRUISE:
        vel *= V_CRUISE / sp
    return vel, d_fire


def footprint_all_burning(E, pos):
    """True if every observed cell under the drone's footprint is classified burning."""
    i0 = max(int((pos[0] - SENSOR_RADIUS) / GRID_RES) - 1, 0)
    i1 = min(int((pos[0] + SENSOR_RADIUS) / GRID_RES) + 2, N_CELLS)
    j0 = max(int((pos[1] - SENSOR_RADIUS) / GRID_RES) - 1, 0)
    j1 = min(int((pos[1] + SENSOR_RADIUS) / GRID_RES) + 2, N_CELLS)
    X, Y = np.meshgrid(CELL_C[i0:i1], CELL_C[j0:j1], indexing='ij')
    win = E[i0:i1, j0:j1][np.hypot(X - pos[0], Y - pos[1]) <= SENSOR_RADIUS]
    return bool(np.all(win == BURNING))


def select_replan_target(bxy, ages, other_targets):
    """Oldest-timestamp boundary cell, skipping cells near other drones' active targets."""
    if len(bxy) == 0:
        return None
    ok = np.ones(len(bxy), dtype=bool)
    for tgt in other_targets:
        if tgt is not None:
            ok &= np.hypot(bxy[:, 0] - tgt[0], bxy[:, 1] - tgt[1]) > RESERVE_RADIUS
    if not ok.any():
        ok[:] = True
    cand = np.nonzero(ok)[0]
    return bxy[cand[np.argmax(ages[cand])]].copy()


def hausdorff_pair(true_xy, est_xy):
    """Return (card d_H = mean of the two directed max-min distances,
               average Hausdorff = mean of the two directed mean-min distances)."""
    if len(true_xy) == 0 or len(est_xy) == 0:
        return np.nan, np.nan
    d_te, _ = cKDTree(est_xy).query(true_xy)
    d_et, _ = cKDTree(true_xy).query(est_xy)
    return 0.5 * (d_te.max() + d_et.max()), 0.5 * (d_te.mean() + d_et.mean())


# ── Simulation ──────────────────────────────────────────────────────────────

def simulate(r0, seed, record=False):
    """One full mission with head-fire rate r0."""
    rng = np.random.default_rng(seed)
    kernel = make_spread_kernel(r0)
    F_true = init_grid()
    T_ign = np.where(F_true, 0.0, np.inf)       # initial disc ignites at t = 0
    noise_acc = np.zeros((N_CELLS, N_CELLS))
    E = np.full((N_CELLS, N_CELLS), UNKNOWN, dtype=np.int8)
    last_scan = np.full((N_CELLS, N_CELLS), -np.inf)

    # Drones start evenly spaced just outside the reported initial perimeter
    angles = np.linspace(0, 2.0 * np.pi, N_DRONES, endpoint=False)
    pos = np.array([IGN_CENTRE + (R_INIT + START_OFFSET) * np.array([np.cos(a), np.sin(a)])
                    for a in angles])
    targets = [None] * N_DRONES
    search_dir = [None] * N_DRONES
    follow_since = np.zeros(N_DRONES)          # time the drone (re)started gradient-following

    n_steps = int(round(T_SIM / DT))
    t_axis = np.arange(n_steps) * DT
    dH = np.zeros(n_steps)
    dAvg = np.zeros(n_steps)
    fresh = np.zeros(n_steps)
    trajs = np.zeros((N_DRONES, n_steps, 2))
    events = []           # (t, drone, type, x, y)
    snap_steps = {0, 200, 400, n_steps - 1}
    snaps, anim = {}, []

    for step in range(n_steps):
        t = t_axis[step]
        # 1. fire spreads
        F_true = spread_fire(F_true, T_ign, noise_acc, t + DT, kernel, rng)
        # 2. observations
        for k in range(N_DRONES):
            ir_observe(F_true, E, last_scan, pos[k], t, rng)
        b_est = boundary_mask_est(E)
        bi, bj = np.nonzero(b_est)
        bxy = np.stack([CELL_C[bi], CELL_C[bj]], axis=1)
        ages = t - last_scan[bi, bj]           # finite: every estimated cell was observed
        # 3. guidance
        for k in range(N_DRONES):
            if search_dir[k] is not None:
                # Front re-acquisition: fly outward until the front is back in reach
                d_near = (np.min(np.hypot(bxy[:, 0] - pos[k, 0], bxy[:, 1] - pos[k, 1]))
                          if len(bxy) else np.inf)
                if d_near <= SENSOR_RADIUS + D_REPLAN or not footprint_all_burning(E, pos[k]):
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
            else:
                vel, d_fire = gradient_step(pos[k], bxy, (bi, bj), E)
                pos[k] += vel * DT
                if d_fire > SENSOR_RADIUS + D_REPLAN and footprint_all_burning(E, pos[k]):
                    # front has outrun the estimate here: search outward from the fire centre
                    bu, bv = np.nonzero(E == BURNING)
                    c = np.array([CELL_C[bu].mean(), CELL_C[bv].mean()])
                    u = pos[k] - c
                    search_dir[k] = u / (np.linalg.norm(u) + 1e-9)
                    events.append((t, k, 'search', pos[k, 0], pos[k, 1]))
                elif d_fire > SENSOR_RADIUS + D_REPLAN and len(bxy) > 0:
                    targets[k] = select_replan_target(
                        bxy, ages, targets[:k] + targets[k + 1:])
                    events.append((t, k, 'lost', pos[k, 0], pos[k, 1]))
                elif (len(ages) > 0 and np.any(ages > DT_GAP)
                      and t - follow_since[k] >= GAP_HOLD):
                    targets[k] = select_replan_target(
                        bxy, ages, targets[:k] + targets[k + 1:])
                    events.append((t, k, 'gap', pos[k, 0], pos[k, 1]))
            pos[k] = np.clip(pos[k], 0.0, ARENA_SIZE)
            trajs[k, step] = pos[k]
        # 4. metrics
        b_true = boundary_mask_true(F_true)
        ti, tj = np.nonzero(b_true)
        txy = np.stack([CELL_C[ti], CELL_C[tj]], axis=1)
        dH[step], dAvg[step] = hausdorff_pair(txy, bxy)
        fresh[step] = np.mean(ages <= DT_GAP) if len(ages) else np.nan
        if record:
            if step in snap_steps:
                snaps[step] = dict(F=F_true.copy(), bxy=bxy.copy(),
                                   pos=pos.copy(), t=t)
            if step % ANIM_EVERY == 0 or step == n_steps - 1:
                anim.append((F_true.copy(), bxy.copy(), pos.copy(), t))

    return dict(r0=r0, F_final=F_true, dH=dH, dAvg=dAvg, fresh=fresh,
                trajs=trajs, events=events, t_axis=t_axis, snaps=snaps,
                anim=anim, area0=init_grid().sum() * GRID_RES ** 2,
                area=F_true.sum() * GRID_RES ** 2)


def front_advance(F):
    """Max advance of the burned region beyond R_INIT along head / flank / back directions."""
    w = np.deg2rad(WIND_DIR_DEG)
    X, Y = np.meshgrid(CELL_C, CELL_C, indexing='ij')
    ang = np.arctan2(Y - IGN_CENTRE[1], X - IGN_CENTRE[0])
    rad = np.hypot(X - IGN_CENTRE[0], Y - IGN_CENTRE[1])
    out = []
    for d in (w, w + np.pi / 2, w + np.pi):
        sel = F & (np.abs(np.angle(np.exp(1j * (ang - d)))) < np.deg2rad(3))
        out.append(rad[sel].max() - R_INIT)
    return out


def fire_only_advance(sigma, seed):
    """Spread the fire alone for T_SIM (no drones) to check the spread model."""
    rng = np.random.default_rng(seed)
    kernel = make_spread_kernel(R0_SPREAD)
    F = init_grid()
    T_ign = np.where(F, 0.0, np.inf)
    W = np.zeros(F.shape)
    for step in range(int(round(T_SIM / DT))):
        F = spread_fire(F, T_ign, W, (step + 1) * DT, kernel, rng, sigma)
    return front_advance(F), F.sum() * GRID_RES ** 2


def summarise(res):
    cnt = {k: sum(1 for e in res['events'] if e[2] == k) for k in ('lost', 'gap', 'search')}
    return dict(mean_dH=np.nanmean(res['dH']), final_dH=res['dH'][-1],
                mean_dAvg=np.nanmean(res['dAvg']), final_dAvg=res['dAvg'][-1],
                mean_fresh=100 * np.nanmean(res['fresh']),
                min_fresh=100 * np.nanmin(res['fresh'][res['t_axis'] >= DT_GAP]),
                n_lost=cnt['lost'], n_gap=cnt['gap'], n_search=cnt['search'])


def run_simulation():
    """Main mission (R0 = 0.04 m/s, animated) plus a sweep over head-fire rate."""
    print('=== Spread-model check (fire only, R0 = %.2f m/s, %.0f s) ===' % (R0_SPREAD, T_SIM))
    print(f"Model R(theta)*T     head/flank/back: {R0_SPREAD * T_SIM:.1f} / "
          f"{rate_of_spread(np.pi / 2, R0_SPREAD, ECC) * T_SIM:.1f} / "
          f"{rate_of_spread(np.pi, R0_SPREAD, ECC) * T_SIM:.1f} m")
    for sig in (0.0, SIGMA_SPOT, SIGMA_SPOT_CARD):
        (h, f, b), a = fire_only_advance(sig, SEED)
        print(f"sigma_spot = {sig:.2f} m: advance head/flank/back {h:.1f} / {f:.1f} / {b:.1f} m, "
              f"area {a:.0f} m^2")

    main = simulate(R0_SPREAD, SEED, record=True)
    s = summarise(main)
    head, flank, back = front_advance(main['F_final'])
    print('=== Main run (R0 = %.2f m/s, sigma_spot = %.2f m, seed %d) ===' % (R0_SPREAD, SIGMA_SPOT, SEED))
    print(f"Fire area                 : {main['area0']:.0f} -> {main['area']:.0f} m^2 "
          f"(+{100 * (main['area'] / main['area0'] - 1):.1f} %)")
    print(f"Front advance head/flank/back: {head:.1f} / {flank:.1f} / {back:.1f} m")
    print(f"Hausdorff d_H (card, max-based): mean {s['mean_dH']:.2f} m, final {s['final_dH']:.2f} m")
    print(f"Average Hausdorff (mean-based) : mean {s['mean_dAvg']:.2f} m, final {s['final_dAvg']:.2f} m")
    print(f"Coverage freshness        : mean {s['mean_fresh']:.1f} %, min (t>=30 s) {s['min_fresh']:.1f} %")
    print(f"Replans: lost-contact {s['n_lost']}, coverage-gap {s['n_gap']}, total {s['n_lost'] + s['n_gap']}; "
          f"front re-acquisition searches {s['n_search']}")
    first_gap = [e[0] for e in main['events'] if e[2] == 'gap']
    print(f"First coverage-gap replan at t = {first_gap[0]:.1f} s" if first_gap
          else "No coverage-gap replans")

    sweep = []
    print('=== Spread-rate sweep (seed %d) ===' % SEED)
    for r0 in SWEEP_R0:
        res = main if r0 == R0_SPREAD else simulate(r0, SEED)
        ss = summarise(res)
        ss['r0'] = r0
        ss['area'] = res['area']
        sweep.append(ss)
        print(f"R0 = {r0:.2f} m/s: mean d_H {ss['mean_dH']:.2f} m, final d_H {ss['final_dH']:.2f} m, "
              f"mean avg-H {ss['mean_dAvg']:.2f} m, freshness {ss['mean_fresh']:.1f} %, "
              f"replans lost/gap {ss['n_lost']}/{ss['n_gap']}, searches {ss['n_search']}, final area {ss['area']:.0f} m^2")
    main['summary'] = s
    main['sweep'] = sweep
    return main


# ── Plots ───────────────────────────────────────────────────────────────────
DRONE_COLORS = ['#1f77b4', '#9467bd', '#17becf']
FIRE_CMAP = matplotlib.colors.ListedColormap(['none', '#ff8c42'])
EXTENT = [0, ARENA_SIZE, 0, ARENA_SIZE]


def _draw_map(ax, F, bxy, pos, xlim=(65, 140)):
    ax.imshow(F.T.astype(float), origin='lower', extent=EXTENT, cmap=FIRE_CMAP,
              vmin=0, vmax=1, alpha=0.6, interpolation='nearest')
    ax.contour(CELL_C, CELL_C, F.T.astype(float), levels=[0.5],
               colors=['#8b2500'], linewidths=0.8)
    if len(bxy):
        ax.plot(bxy[:, 0], bxy[:, 1], '.', color='#d62728', ms=1.5)
    for k in range(N_DRONES):
        ax.add_patch(plt.Circle(pos[k], SENSOR_RADIUS, color=DRONE_COLORS[k],
                                fill=False, lw=1.2))
        ax.plot(*pos[k], 'o', color=DRONE_COLORS[k], ms=5, zorder=5)
    w = np.deg2rad(WIND_DIR_DEG)
    ax.annotate('', xy=(xlim[0] + 5 + 8 * np.cos(w), xlim[0] + 5 + 8 * np.sin(w)),
                xytext=(xlim[0] + 5, xlim[0] + 5),
                arrowprops=dict(arrowstyle='->', color='navy', lw=1.5))
    ax.text(xlim[0] + 3, xlim[0] + 1.5, 'wind', fontsize=7, color='navy')
    ax.set_xlim(*xlim)
    ax.set_ylim(*xlim)
    ax.set_aspect('equal')


def plot_fire_evolution(snaps, out_dir):
    keys = sorted(snaps)
    fig, axes = plt.subplots(1, 4, figsize=(20, 5.4))
    for ax, k in zip(axes, keys):
        s = snaps[k]
        _draw_map(ax, s['F'], s['bxy'], s['pos'])
        ax.set_title(f"t = {s['t']:.1f} s", fontsize=11)
        ax.set_xlabel('x (m)')
    axes[0].set_ylabel('y (m)')
    handles = [mpatches.Patch(color='#ff8c42', alpha=0.6, label='True burn region'),
               Line2D([], [], color='#8b2500', lw=0.8, label='True boundary'),
               Line2D([], [], color='#d62728', marker='.', ls='', label='Estimated boundary cells')]
    handles += [Line2D([], [], color=DRONE_COLORS[k], marker='o', ls='',
                       label=f'Drone {k + 1} + IR footprint') for k in range(N_DRONES)]
    fig.legend(handles=handles, loc='lower center', ncol=6, fontsize=9)
    fig.suptitle('S041 Wildfire Boundary Scan — Fire Evolution (R0 = 0.04 m/s, wind toward NE)',
                 fontsize=13, fontweight='bold')
    plt.tight_layout(rect=[0, 0.07, 1, 0.95])
    path = os.path.join(out_dir, 'fire_evolution.png')
    plt.savefig(path, dpi=130)
    plt.close()
    print(f'Saved: {path}')


def plot_hausdorff(res, out_dir):
    t = res['t_axis']
    fig, ax = plt.subplots(figsize=(10, 4.2))
    ax.plot(t, res['dH'], color='#264653', lw=1.3, label='$d_H$ (card, max-based)')
    ax.plot(t, res['dAvg'], color='#e76f51', lw=1.3, label='Average Hausdorff (mean-based)')
    for typ, mk in [('lost', 'v'), ('gap', '^'), ('search', 'D')]:
        for k in range(N_DRONES):
            ev = [e[0] for e in res['events'] if e[1] == k and e[2] == typ]
            if ev:
                ax.scatter(ev, np.interp(ev, t, res['dH']), marker=mk, s=40,
                           color=DRONE_COLORS[k], edgecolors='k', lw=0.4, zorder=5,
                           label=f'Drone {k + 1} {typ} replan ({len(ev)})')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Distance (m)')
    ax.set_title('Boundary tracking error (v lost-contact replan, ^ coverage-gap replan, D front search)')
    ax.legend(fontsize=8, ncol=2)
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'hausdorff_distance.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def plot_coverage_freshness(res, out_dir):
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(res['t_axis'], res['fresh'] * 100.0, color='#2a9d8f', lw=1.5,
            label='Coverage freshness $\\eta(t)$')
    ax.axhline(90.0, color='#e76f51', lw=1.2, ls='--', label='90% target')
    ax.axvline(DT_GAP, color='grey', lw=0.8, ls=':', label='t = 30 s')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Fresh boundary fraction (%)')
    ax.set_ylim(-5, 105)
    ax.set_title('Coverage freshness — estimated boundary cells scanned within the last 30 s')
    ax.legend(fontsize=10)
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'coverage_freshness.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def plot_trajectory_overlay(res, out_dir):
    fig, ax = plt.subplots(figsize=(8, 8))
    F = res['F_final']
    ax.imshow(F.T.astype(float), origin='lower', extent=EXTENT, cmap=FIRE_CMAP,
              vmin=0, vmax=1, alpha=0.35, interpolation='nearest')
    ax.contour(CELL_C, CELL_C, init_grid().T.astype(float), levels=[0.5],
               colors=['k'], linewidths=0.8, linestyles='--')
    ax.contour(CELL_C, CELL_C, F.T.astype(float), levels=[0.5],
               colors=['#8b2500'], linewidths=1.0)
    t = res['t_axis']
    for k in range(N_DRONES):
        tr = res['trajs'][k]
        sc = ax.scatter(tr[:, 0], tr[:, 1], c=t, cmap='coolwarm', s=1.5)
        ax.plot(*tr[0], 's', color=DRONE_COLORS[k], ms=8, mec='k', label=f'Drone {k + 1} start')
        ax.plot(*tr[-1], '*', color=DRONE_COLORS[k], ms=12, mec='k')
        for typ, mk in [('lost', 'v'), ('gap', '^'), ('search', 'D')]:
            ev = np.array([(e[3], e[4]) for e in res['events'] if e[1] == k and e[2] == typ])
            if len(ev):
                ax.scatter(ev[:, 0], ev[:, 1], marker=mk, s=45, color=DRONE_COLORS[k],
                           edgecolors='k', lw=0.5, zorder=6)
    plt.colorbar(sc, ax=ax, fraction=0.046, label='time (s)')
    ax.plot([], [], 'k--', lw=0.8, label='Perimeter t = 0')
    ax.plot([], [], color='#8b2500', lw=1.0, label='Perimeter t = 300 s')
    ax.plot([], [], 'v', color='grey', mec='k', label='Lost-contact replan')
    ax.plot([], [], '^', color='grey', mec='k', label='Coverage-gap replan')
    ax.plot([], [], 'D', color='grey', mec='k', label='Front re-acquisition search')
    ax.set_xlim(65, 140)
    ax.set_ylim(65, 140)
    ax.set_aspect('equal')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title('Drone trajectories (colour = time, blue early -> red late)')
    ax.legend(fontsize=8, loc='lower right')
    plt.tight_layout()
    path = os.path.join(out_dir, 'trajectory_overlay.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def plot_spread_sweep(sweep, out_dir):
    r0 = [s['r0'] for s in sweep]
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(11, 4))
    a1.plot(r0, [s['mean_dH'] for s in sweep], 'o-', label='mean $d_H$ (max-based)')
    a1.plot(r0, [s['final_dH'] for s in sweep], 's--', label='final $d_H$')
    a1.plot(r0, [s['mean_dAvg'] for s in sweep], 'd-', label='mean average-Hausdorff')
    a1.set_xlabel('Head-fire rate $R_0$ (m/s)')
    a1.set_ylabel('Tracking error (m)')
    a1.set_xscale('log', base=2)
    a1.set_xticks(r0, [f'{v:.2f}' for v in r0])
    a1.legend(fontsize=8)
    a1.grid(True, alpha=0.3)
    a2.plot(r0, [s['mean_fresh'] for s in sweep], 'o-', color='#2a9d8f')
    a2.axhline(90, color='#e76f51', ls='--', lw=1)
    a2.set_xscale('log', base=2)
    a2.set_xticks(r0, [f'{v:.2f}' for v in r0])
    a2.set_xlabel('Head-fire rate $R_0$ (m/s)')
    a2.set_ylabel('Mean coverage freshness (%)')
    a2.set_ylim(0, 105)
    a2.grid(True, alpha=0.3)
    fig.suptitle('Tracking performance vs fire spread rate (3 drones, seed 0)')
    plt.tight_layout()
    path = os.path.join(out_dir, 'spread_rate_sweep.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f'Saved: {path}')


def save_animation(res, out_dir):
    """Replay the recorded history of the main run (same fire the drones tracked)."""
    import matplotlib.animation as animation
    frames = res['anim']
    fig, ax = plt.subplots(figsize=(6, 6))

    def update(i):
        ax.clear()
        F, bxy, pos, t = frames[i]
        _draw_map(ax, F, bxy, pos)
        ax.set_title(f'S041 Wildfire Boundary Scan   t = {t:.0f} s', fontsize=11)
        ax.set_xlabel('x (m)')
        ax.set_ylabel('y (m)')
        return []

    ani = animation.FuncAnimation(fig, update, frames=len(frames), interval=80, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer='pillow', fps=12, dpi=80)
    plt.close()
    print(f'Saved: {path}  ({len(frames)} frames)')


# ── Main ────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    print('Running S041 Wildfire Boundary Scan simulation...')
    out_dir = os.path.normpath(OUTPUT_DIR)
    os.makedirs(out_dir, exist_ok=True)
    data = run_simulation()
    plot_fire_evolution(data['snaps'], out_dir)
    plot_hausdorff(data, out_dir)
    plot_coverage_freshness(data, out_dir)
    plot_trajectory_overlay(data, out_dir)
    plot_spread_sweep(data['sweep'], out_dir)
    save_animation(data, out_dir)
    print('Done.')
