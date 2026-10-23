"""
S036 3D Upgrade — Last-Mile Relay
=================================
A parcel travels 120 m from a source depot D0 = (0, 0, 2) to a destination
DK = (120, 0, 8) over terrain: a 0 -> 6 m ramp along x plus a 5 m Gaussian
hill at the centre (60, 0).  No single drone can fly the route
(R_eff = 0.9 x 35 = 31.5 m), so a chain of relay drones carries it leg by leg
and hands it over at relay stations.

Flight model (card):
  * a leg from station w_{k-1} to w_k flies the straight 3-D line between the
    two rendezvous points, raised to the terrain-clearance floor
    h(x, y) + h_clr wherever the terrain would intrude:
        z_fly(s) = max(h(p(s)) + h_clr, (1 - s) z_{k-1} + s z_k)
  * the DEM is the analytic terrain sampled on a 1 m grid and bilinearly
    interpolated; legs are sampled every <= 0.25 m along x;
  * battery (flown path, cumulative climb):  B_k = (L_path + a_climb * climb) / R
    The card's chord formula  B_k = (d_3D + a_climb * max(0, dz)) / R  is also
    printed, to show how much it underestimates;
  * speed: 5 m/s along the 3-D path, slowed so that |v_z| <= 2 m/s.

Station altitude models:
  * flat     : intermediate rendezvous points at one fixed altitude Z_FLAT,
               the lowest altitude that clears terrain + h_clr everywhere on
               the route (terminals stay at terrain + h_clr);
  * terrain  : every rendezvous point at terrain + h_clr (card).

Relay plans (all use the same flight and battery model):
  1. 2D plan, flat altitude       -- the 2D S036 relays (x = 30/60/90 m)
  2. 2D plan, terrain-following   -- same relays, terrain-following stations
  3. Flat, minimax                -- relay x positions that minimise the worst
                                     leg battery, smallest feasible K
  4. Terrain greedy (card)        -- furthest-reachable segmentation over
                                     1 m (DEM resolution) via-points
  5. Terrain minimax              -- as 3, with terrain-following stations
  Minimax uses exact dynamic programming on a 0.25 m grid of relay x.

Handoff (card): the outgoing drone waits on a pad on the ground, 2 m to the
side of the station, and launches early enough to be aligned at the
rendezvous point when the incoming drone arrives.  It closes the gap with a
proportional law (K = 1.5, |v_z| <= 2 m/s, |v| <= 5 m/s); capture is declared
when |z_out - z_k| < 0.5 m and |p_in - p_out| < 0.5 m.  The pad climb (and
the incoming drone's landing on the pad of the next station) is reported as
station overhead, and a second feasibility check counts it in each drone's
battery.

A terrain sweep (hill peak 0-10 m, width sigma 5-30 m) repeats the minimum-K
analysis to show how far the results depend on the chosen hill.

Outputs (outputs/02_logistics_delivery/3d/s036_3d_last_mile_relay/):
  trajectory_3d.png, altitude_profile.png, battery_usage.png,
  delivery_time_breakdown.png, handoff_detail.png, terrain_sweep.png,
  animation.gif

Usage:
    conda activate drones
    python src/02_logistics_delivery/3d/s036_3d_last_mile_relay.py
"""

import matplotlib
matplotlib.use('Agg')

import os
import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.lines import Line2D

OUTPUT_DIR = os.path.normpath(os.path.join(
    os.path.dirname(__file__), '..', '..', '..',
    'outputs', '02_logistics_delivery', '3d', 's036_3d_last_mile_relay',
))

# ── Parameters (card) ─────────────────────────────────────────────────────────
L_2D          = 120.0      # m   ground distance D0 -> DK (along y = 0)
RAMP_RISE     = 6.0        # m   terrain rise from D0 to DK (card dH = 6 m)
HILL_PEAK     = 5.0        # m   Gaussian hill amplitude at the centre
H_CLR         = 2.0        # m   terrain clearance
DRONE_RANGE   = 35.0       # m   R
RESERVE       = 0.10       # battery reserve
B_LIMIT       = 1.0 - RESERVE              # 0.90
R_EFF         = DRONE_RANGE * B_LIMIT      # 31.5 m
ALPHA_CLIMB   = 0.30       # climb surcharge (m-equivalent per m climbed)
V_CRUISE      = 5.0        # m/s along the 3-D path
VZ_MAX        = 2.0        # m/s
K_Z           = 1.5        # 1/s altitude alignment gain
R_HANDOFF     = 0.5        # m   3-D capture radius
DELTA_Z       = 0.5        # m   altitude match tolerance
TAU_H         = 3.0        # s   handoff dwell
Z_BOUNDS      = (0.5, 20.0)
DEM_RES       = 1.0        # m
DT            = 0.05       # s

# ── Added parameters (not on the card) ────────────────────────────────────────
HILL_SIGMA    = 15.0       # m   Gaussian hill width (card gives none)
HILL_CENTRE   = (60.0, 0.0)
K_XY          = 1.5        # 1/s horizontal alignment gain (same as K_z)
PAD_OFFSET    = 2.0        # m   standby pad, lateral (+y) offset from station
LEAD_MARGIN   = 1.0        # s   outgoing drone is aligned this long before arrival
GRID_STEP     = 0.25       # m   relay x grid for the minimax DP
SAMPLE_STEP   = 0.25       # m   max x step when sampling a leg
PLAN2D_X      = [30.0, 60.0, 90.0]     # 2D S036 straight-route relays
SWEEP_PEAKS   = [0.0, 2.5, 5.0, 7.5, 10.0]
SWEEP_SIGMAS  = [5.0, 10.0, 15.0, 20.0, 30.0]
SWEEP_STEP    = 0.5        # m   relay grid for the sweep (coarser)
GIF_STEP      = 8          # simulation steps per GIF frame

DEM_X = np.arange(-10.0, 130.0 + DEM_RES / 2, DEM_RES)
DEM_Y = np.arange(-20.0, 20.0 + DEM_RES / 2, DEM_RES)


# ── Terrain (analytic -> 1 m DEM -> bilinear) ────────────────────────────────
def terrain_analytic(x, y, peak=HILL_PEAK, sigma=HILL_SIGMA):
    x = np.asarray(x, float)
    y = np.asarray(y, float)
    ramp = RAMP_RISE * np.clip(x, 0.0, L_2D) / L_2D
    hill = peak * np.exp(-((x - HILL_CENTRE[0]) ** 2 + (y - HILL_CENTRE[1]) ** 2)
                         / (2.0 * sigma ** 2))
    return ramp + hill


class Terrain:
    def __init__(self, peak=HILL_PEAK, sigma=HILL_SIGMA):
        self.peak, self.sigma = peak, sigma
        XX, YY = np.meshgrid(DEM_X, DEM_Y, indexing='ij')
        self.grid = terrain_analytic(XX, YY, peak, sigma)
        xs = np.arange(0.0, L_2D + 1e-9, 0.05)
        self._z_flat = float(np.max(self.h(xs, 0.0 * xs)) + H_CLR)

    def h(self, x, y):
        x = np.clip(np.asarray(x, float), DEM_X[0], DEM_X[-1] - 1e-9)
        y = np.clip(np.asarray(y, float), DEM_Y[0], DEM_Y[-1] - 1e-9)
        fx = (x - DEM_X[0]) / DEM_RES
        fy = (y - DEM_Y[0]) / DEM_RES
        i = np.floor(fx).astype(int)
        j = np.floor(fy).astype(int)
        tx, ty = fx - i, fy - j
        g = self.grid
        return ((1 - tx) * (1 - ty) * g[i, j] + tx * (1 - ty) * g[i + 1, j]
                + (1 - tx) * ty * g[i, j + 1] + tx * ty * g[i + 1, j + 1])

    def z_flat(self):
        return self._z_flat


# ── Leg geometry, battery and time ───────────────────────────────────────────
def leg_profile(ter, xa, za, xb, zb):
    """Flown profile (x, z) of a leg along y = 0 between rendezvous points."""
    n = max(2, int(math.ceil(abs(xb - xa) / SAMPLE_STEP)) + 1)
    s = np.linspace(0.0, 1.0, n)
    x = xa + s * (xb - xa)
    z_lin = (1 - s) * za + s * zb
    z_safe = ter.h(x, 0.0 * x) + H_CLR
    z = np.maximum(z_safe, z_lin)
    return x, z


def leg_metrics(ter, xa, za, xb, zb):
    x, z = leg_profile(ter, xa, za, xb, zb)
    dx, dz = np.diff(x), np.diff(z)
    seg = np.hypot(dx, dz)
    path = float(seg.sum())
    climb = float(np.clip(dz, 0, None).sum())
    t = float(np.maximum(seg / V_CRUISE, np.abs(dz) / VZ_MAX).sum())
    chord = math.hypot(xb - xa, zb - za)
    return dict(path=path, climb=climb, time=t, chord=chord,
                B=(path + ALPHA_CLIMB * climb) / DRONE_RANGE,
                B_card=(chord + ALPHA_CLIMB * max(0.0, zb - za)) / DRONE_RANGE,
                x=x, z=z)


def station_z(ter, x, mode, terminal):
    if mode == 'terrain' or terminal:
        return float(ter.h(x, 0.0) + H_CLR)
    return ter.z_flat()


def leg_matrix(ter, xs, mode):
    """Battery B[i, j] for legs xs[i] -> xs[j] (inf if obviously too long)."""
    n = len(xs)
    zs = np.array([station_z(ter, xs[i], mode, i in (0, n - 1)) for i in range(n)])
    B = np.full((n, n), np.inf)
    max_dx = R_EFF + 1e-9   # B >= dx / R, so longer legs can never be feasible
    for i in range(n - 1):
        j_hi = int(np.searchsorted(xs, xs[i] + max_dx, side='right'))
        for j in range(i + 1, min(j_hi, n)):
            B[i, j] = leg_metrics(ter, xs[i], zs[i], xs[j], zs[j])['B']
    return B, zs


def minimax_dp(B, k_max=30, extra_up=None, extra_down=None):
    """Exact min-K, min-bottleneck relay placement on a grid.

    Returns (K, bottleneck, idx list).  Secondary objective: among plans with
    the optimal bottleneck, minimise the sum of B.
    """
    n = B.shape[0]
    C = B.copy()
    if extra_up is not None:
        C = C + extra_up[:, None] + extra_down[None, :]
    best = None
    for K in range(1, k_max + 1):
        # pass 1: bottleneck
        dp = np.full(n, np.inf)
        dp[0] = 0.0
        for _ in range(K):
            dp = np.min(np.maximum(dp[:, None], C), axis=0)
        bott = dp[n - 1]
        if bott <= B_LIMIT + 1e-12:
            best = (K, bott)
            break
    if best is None:
        return None
    K, bott = best
    allowed = np.where(C <= bott + 1e-12, B, np.inf)
    # pass 2: minimise sum of B among bottleneck-optimal plans (backtrack)
    cost = np.full((K + 1, n), np.inf)
    arg = np.zeros((K + 1, n), dtype=int)
    cost[0, 0] = 0.0
    for k in range(1, K + 1):
        tot = cost[k - 1][:, None] + allowed
        arg[k] = np.argmin(tot, axis=0)
        cost[k] = tot[arg[k], np.arange(n)]
    idx = [n - 1]
    for k in range(K, 0, -1):
        idx.append(int(arg[k, idx[-1]]))
    return K, float(bott), idx[::-1]


def greedy_card(ter, mode='terrain'):
    """Card pseudocode: furthest reachable 1 m via-point with B <= 0.9."""
    vx = np.arange(0.0, L_2D + 1e-9, DEM_RES)
    n = len(vx)
    zs = [station_z(ter, vx[i], mode, i in (0, n - 1)) for i in range(n)]
    cuts, i = [0], 0
    while i < n - 1:
        reach = [j for j in range(i + 1, n)
                 if leg_metrics(ter, vx[i], zs[i], vx[j], zs[j])['B'] <= B_LIMIT + 1e-12]
        if not reach:
            raise RuntimeError('infeasible route')
        i = reach[-1]
        cuts.append(i)
    return [float(vx[c]) for c in cuts]


# ── Handoff pre-positioning ──────────────────────────────────────────────────
def prepos_traj(ter, x_st, z_st):
    """Outgoing drone: pad at (x_st, PAD_OFFSET, ground) -> rendezvous point.

    Integrated until aligned: |z - z_st| < DELTA_Z and 3-D distance < R_HANDOFF/2.
    Returns positions (n, 3) and the alignment time.
    """
    p = np.array([x_st, PAD_OFFSET, float(ter.h(x_st, PAD_OFFSET))])
    tgt = np.array([x_st, 0.0, z_st])
    P = [p.copy()]
    for _ in range(4000):
        e = tgt - p
        if abs(e[2]) < DELTA_Z and np.linalg.norm(e) < R_HANDOFF / 2:
            break
        v = np.array([K_XY * e[0], K_XY * e[1], K_Z * e[2]])
        v[2] = np.clip(v[2], -VZ_MAX, VZ_MAX)
        sp = np.linalg.norm(v)
        if sp > V_CRUISE:
            v *= V_CRUISE / sp
        p = p + v * DT
        assert p[2] >= ter.h(p[0], p[1]) - 1e-9, 'pre-positioning below ground'
        P.append(p.copy())
    else:
        raise RuntimeError('alignment did not converge')
    P = np.array(P)
    return P, (len(P) - 1) * DT


def prepos_overhead(ter, x_st, z_st):
    """Battery of the pad climb (with surcharge) and of the landing (no surcharge)."""
    P, t = prepos_traj(ter, x_st, z_st)
    # include the last few cm flown during the hold (the drone ends exactly
    # at the rendezvous point, see simulate_timeline)
    P = np.vstack([P, [x_st, 0.0, z_st]])
    d = np.diff(P, axis=0)
    path = float(np.linalg.norm(d, axis=1).sum())
    climb = float(np.clip(d[:, 2], 0, None).sum())
    gap = z_st - P[0, 2]                 # pad ground -> rendezvous height
    up = (path + ALPHA_CLIMB * climb) / DRONE_RANGE
    down = path / DRONE_RANGE            # reverse path back to the pad
    return up, down, t, path, gap


# ── Plan evaluation and time simulation ──────────────────────────────────────
def evaluate_plan(ter, name, key, xs_st, mode):
    K = len(xs_st) - 1
    zs = [station_z(ter, x, mode, i in (0, K)) for i, x in enumerate(xs_st)]
    legs = [leg_metrics(ter, xs_st[k], zs[k], xs_st[k + 1], zs[k + 1]) for k in range(K)]
    ovh = [prepos_overhead(ter, x, z) for x, z in zip(xs_st, zs)]
    B = [lg['B'] for lg in legs]
    B_tot = [ovh[k][0] + legs[k]['B'] + ovh[k + 1][1] for k in range(K)]
    for lg in legs:
        assert np.all(lg['z'] >= ter.h(lg['x'], 0 * lg['x']) + H_CLR - 1e-9)
        assert np.all((lg['z'] >= Z_BOUNDS[0]) & (lg['z'] <= Z_BOUNDS[1]))
    return dict(name=name, key=key, mode=mode, xs=list(xs_st), zs=zs, K=K, legs=legs,
                B=B, B_card=[lg['B_card'] for lg in legs], B_tot=B_tot,
                ovh=ovh, feasible=max(B) <= B_LIMIT + 1e-12,
                feasible_tot=max(B_tot) <= B_LIMIT + 1e-12,
                path=sum(lg['path'] for lg in legs),
                climb=sum(lg['climb'] for lg in legs),
                t_flight=sum(lg['time'] for lg in legs),
                T=sum(lg['time'] for lg in legs) + (K - 1) * TAU_H)


def simulate_timeline(ter, plan):
    """Time-stepped parcel delivery with handoff capture (dt = 0.05 s)."""
    assert max(plan['B_tot']) <= 1.0, 'a drone would run its battery empty'
    K = plan['K']
    t = 0.0
    T, Ppar, LEG = [], [], []
    handoffs = []
    for k in range(K):
        lg = plan['legs'][k]
        # parcel along the leg: time stamps of the sampled profile
        seg = np.hypot(np.diff(lg['x']), np.diff(lg['z']))
        tseg = np.maximum(seg / V_CRUISE, np.abs(np.diff(lg['z'])) / VZ_MAX)
        tcum = np.concatenate([[0.0], np.cumsum(tseg)])
        n_steps = int(math.floor(tcum[-1] / DT + 1e-9))
        tt = np.arange(n_steps + 1) * DT
        px = np.interp(tt, tcum, lg['x'])
        pz = np.interp(tt, tcum, lg['z'])
        # final step lands exactly on the rendezvous point
        if tt[-1] < tcum[-1] - 1e-12:
            tt = np.append(tt, tcum[-1])
            px = np.append(px, lg['x'][-1])
            pz = np.append(pz, lg['z'][-1])
        p_in = np.column_stack([px, 0 * px, pz])
        t_arr = t + tcum[-1]
        t_leg = t + tt
        T.extend(t_leg)
        Ppar.extend(p_in)
        LEG.extend([k] * len(tt))
        t = t_arr
        if k == K - 1:
            break
        # handoff at station k+1: outgoing launched so it is aligned
        # LEAD_MARGIN s before the incoming drone arrives
        x_st, z_st = plan['xs'][k + 1], plan['zs'][k + 1]
        P_out, t_al = prepos_traj(ter, x_st, z_st)
        t_launch = t_arr - LEAD_MARGIN - t_al
        # relative states over the last part of the approach (for the plot)
        rel_t, rel = [], []
        t_cap = None
        for ii, ti in enumerate(t_leg):
            j = int(round((ti - t_launch) / DT))
            if j < 0:
                po = P_out[0]
            else:
                po = P_out[min(j, len(P_out) - 1)]
            if j >= len(P_out):     # aligned: hold with the same P law
                po = np.array([x_st, 0.0, z_st]) + (P_out[-1] - np.array([x_st, 0.0, z_st])) \
                    * math.exp(-K_Z * (j - len(P_out) + 1) * DT)
            pi = p_in[ii]
            rel_t.append(ti - t_arr)
            rel.append(pi - po)
            if t_cap is None and abs(po[2] - z_st) < DELTA_Z \
                    and np.linalg.norm(pi - po) < R_HANDOFF:
                t_cap = ti
        assert t_cap is not None, 'handoff capture not reached'
        # dwell (parcel held at the rendezvous point)
        tw = np.arange(1, int(round(TAU_H / DT)) + 1) * DT
        pts = np.repeat(np.array([[x_st, 0.0, z_st]]), len(tw), axis=0)
        T.extend(t + tw)
        Ppar.extend(pts)
        LEG.extend([k] * len(tw))
        # outgoing residual offset after the dwell (P law keeps converging)
        resid = np.linalg.norm(P_out[-1] - np.array([x_st, 0.0, z_st])) * \
            math.exp(-K_Z * (t_arr + TAU_H - (t_launch + t_al)))
        assert resid < 0.01, 'outgoing drone not at the rendezvous point'
        handoffs.append(dict(station=k + 1, x=x_st, z=z_st, t_arr=t_arr,
                             t_cap=t_cap, t_launch=t_launch, t_align=t_al,
                             climb=z_st - P_out[0, 2], P_out=P_out,
                             rel_t=np.array(rel_t), rel=np.array(rel),
                             resid=resid))
        t = t_arr + TAU_H
    plan['T'] = t
    plan['trace_t'] = np.array(T)
    plan['trace_p'] = np.array(Ppar)
    plan['trace_leg'] = np.array(LEG)
    plan['handoffs'] = handoffs
    return plan


# ── Simulation ───────────────────────────────────────────────────────────────
def min_k_study(ter, step):
    xs = np.round(np.arange(0.0, L_2D + 1e-9, step), 6)
    out = {}
    for mode in ('flat', 'terrain'):
        B, zs = leg_matrix(ter, xs, mode)
        up = np.zeros(len(xs))
        dn = np.zeros(len(xs))
        for i, x in enumerate(xs):
            u, d, *_ = prepos_overhead(ter, x, zs[i])
            up[i], dn[i] = u, d
        res = minimax_dp(B)
        res_tot = minimax_dp(B, extra_up=up, extra_down=dn)
        out[mode] = dict(leg=res, tot=res_tot, xs=xs)
    return out


def equal_spacing_min_k(ter, mode):
    for K in range(1, 10):
        xs = [L_2D * k / K for k in range(K + 1)]
        K_ = len(xs) - 1
        zs = [station_z(ter, x, mode, i in (0, K_)) for i, x in enumerate(xs)]
        Bmax = max(leg_metrics(ter, xs[k], zs[k], xs[k + 1], zs[k + 1])['B']
                   for k in range(K_))
        if Bmax <= B_LIMIT + 1e-12:
            return K, Bmax
    return None, None


def run_simulation():
    ter = Terrain()
    z_flat = ter.z_flat()
    study = min_k_study(ter, GRID_STEP)

    plans = {}
    plans['2d_flat'] = evaluate_plan(ter, '2D plan, flat altitude', '2d_flat',
                                     [0.0] + PLAN2D_X + [L_2D], 'flat')
    plans['2d_terr'] = evaluate_plan(ter, '2D plan, terrain-following', '2d_terr',
                                     [0.0] + PLAN2D_X + [L_2D], 'terrain')
    K, b, idx = study['flat']['leg']
    plans['flat_mm'] = evaluate_plan(ter, f'Flat altitude, minimax (K={K})', 'flat_mm',
                                     [float(study['flat']['xs'][i]) for i in idx], 'flat')
    plans['terr_greedy'] = evaluate_plan(ter, 'Terrain-following, card greedy', 'terr_greedy',
                                         greedy_card(ter), 'terrain')
    K, b, idx = study['terrain']['leg']
    plans['terr_mm'] = evaluate_plan(ter, f'Terrain-following, minimax (K={K})', 'terr_mm',
                                     [float(study['terrain']['xs'][i]) for i in idx], 'terrain')
    for mode, key in (('flat', 'flat_tot'), ('terrain', 'terr_tot')):
        K, b, idx = study[mode]['tot']
        label = 'Flat altitude' if mode == 'flat' else 'Terrain-following'
        plans[key] = evaluate_plan(ter, f'{label}, minimax incl. overhead (K={K})', key,
                                   [float(study[mode]['xs'][i]) for i in idx], mode)
    for p in plans.values():
        p['flown'] = max(p['B_tot']) <= 1.0
        if p['flown']:
            T_formula = p['T']
            simulate_timeline(ter, p)
            assert abs(p['T'] - T_formula) < 1e-9

    eq = {mode: equal_spacing_min_k(ter, mode) for mode in ('flat', 'terrain')}

    # 2D model (no altitude), as in 2D S036
    two_d = dict(K=4, B=30.0 / DRONE_RANGE,
                 T=L_2D / V_CRUISE + 3 * TAU_H)

    # terrain sweep: minimum K (leg-only and incl. overhead)
    sweep = {}
    for pk in SWEEP_PEAKS:
        for sg in SWEEP_SIGMAS:
            t2 = Terrain(pk, sg)
            st = min_k_study(t2, SWEEP_STEP)
            eqt = equal_spacing_min_k(t2, 'terrain')
            eqf = equal_spacing_min_k(t2, 'flat')
            def k_of(r):
                return math.inf if r is None else r[0]   # inf = no K <= 30 works
            sweep[(pk, sg)] = dict(
                flat=k_of(st['flat']['leg']), terr=k_of(st['terrain']['leg']),
                flat_tot=k_of(st['flat']['tot']), terr_tot=k_of(st['terrain']['tot']),
                flat_b=st['flat']['leg'][1], terr_b=st['terrain']['leg'][1],
                eq_terr=math.inf if eqt[0] is None else eqt[0],
                eq_flat=math.inf if eqf[0] is None else eqf[0], z_flat=t2.z_flat())
    return dict(ter=ter, z_flat=z_flat, plans=plans, eq=eq, two_d=two_d,
                sweep=sweep)


# ── Plots ────────────────────────────────────────────────────────────────────
LEG_COLORS = ['#E63946', '#2A9D8F', '#E9C46A', '#8338EC', '#3A86FF', '#F4A261']
PLAN_ORDER = ['2d_flat', '2d_terr', 'flat_mm', 'terr_greedy', 'terr_mm',
              'flat_tot', 'terr_tot']
PLAN_COLORS = {'2d_flat': '#999999', '2d_terr': '#555555', 'flat_mm': '#F4A261',
               'terr_greedy': '#E9C46A', 'terr_mm': '#2A9D8F',
               'flat_tot': '#E76F51', 'terr_tot': '#264653'}
SHORT = {'2d_flat': '2D plan\nflat', '2d_terr': '2D plan\nterrain',
         'flat_mm': 'Flat\nminimax', 'terr_greedy': 'Terrain\ngreedy',
         'terr_mm': 'Terrain\nminimax', 'flat_tot': 'Flat\nminimax+ovh',
         'terr_tot': 'Terrain\nminimax+ovh'}


def _terrain_surface(ax, ter, alpha=0.35):
    xs = np.arange(-5, 126, 2.5)
    ys = np.arange(-15, 15.1, 2.5)
    XX, YY = np.meshgrid(xs, ys)
    ax.plot_surface(XX, YY, ter.h(XX, YY), color='grey', alpha=alpha,
                    linewidth=0, antialiased=True, shade=True)


def _draw_plan_3d(ax, ter, p):
    for k, lg in enumerate(p['legs']):
        ax.plot(lg['x'], 0 * lg['x'], lg['z'], color=LEG_COLORS[k % 6], lw=2.5)
    for i, (x, z) in enumerate(zip(p['xs'], p['zs'])):
        if 0 < i < p['K']:
            ax.scatter([x], [0], [z], s=70, color='orange', edgecolors='k', depthshade=False)
            ax.plot([x, x], [0, 0], [float(ter.h(x, 0)), z], color='orange', lw=1, ls=':')
            ax.scatter([x], [PAD_OFFSET], [float(ter.h(x, PAD_OFFSET))], s=18,
                       marker='s', color='k', depthshade=False)
    ax.scatter([p['xs'][0]], [0], [p['zs'][0]], s=90, marker='s', color='green',
               edgecolors='k', depthshade=False)
    ax.scatter([p['xs'][-1]], [0], [p['zs'][-1]], s=160, marker='*', color='red',
               edgecolors='k', depthshade=False)


def plot_trajectory_3d(data, out_dir):
    ter, P = data['ter'], data['plans']
    fig = plt.figure(figsize=(15, 10))
    for n, key in enumerate(('flat_mm', 'terr_mm', 'flat_tot', 'terr_tot')):
        ax = fig.add_subplot(2, 2, n + 1, projection='3d')
        _terrain_surface(ax, ter)
        p = P[key]
        _draw_plan_3d(ax, ter, p)
        ax.set_xlim(-5, 125); ax.set_ylim(-15, 15); ax.set_zlim(0, 14)
        ax.set_box_aspect((4, 1, 0.9))
        ax.view_init(elev=22, azim=-70)
        ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)'); ax.set_zlabel('z (m)')
        ax.set_title(f'{p["name"]}\nmax leg B = {max(p["B"])*100:.1f} %, max per-drone '
                     f'incl. pad = {max(p["B_tot"])*100:.1f} %, T = {p["T"]:.2f} s',
                     fontsize=10)
    handles = [Line2D([], [], marker='s', color='green', ls='', ms=9, label='Source D0'),
               Line2D([], [], marker='*', color='red', ls='', ms=12, label='Destination DK'),
               Line2D([], [], marker='o', color='orange', ls='', ms=9, label='Relay rendezvous'),
               Line2D([], [], marker='s', color='k', ls='', ms=5, label='Standby pad'),
               Line2D([], [], color='grey', lw=6, alpha=0.5, label='Terrain')]
    fig.legend(handles=handles, loc='lower center', ncol=5, fontsize=9)
    fig.suptitle('S036 3D Last-Mile Relay: flat-altitude (left) vs terrain-following (right) '
                 'stations\ntop: card accounting (leg only), bottom: pad climb and landing '
                 'counted', fontsize=12, fontweight='bold')
    plt.tight_layout(rect=[0, 0.04, 1, 0.94])
    path = os.path.join(out_dir, 'trajectory_3d.png')
    plt.savefig(path, dpi=130)
    plt.close()
    print(f'Saved: {path}')


def plot_altitude_profile(data, out_dir):
    ter, P = data['ter'], data['plans']
    keys = ('2d_flat', '2d_terr', 'flat_mm', 'terr_mm', 'flat_tot', 'terr_tot')
    fig, axes = plt.subplots(3, 2, figsize=(17, 11), sharex=True, sharey=True)
    axes = axes.ravel()
    xs = np.linspace(0, L_2D, 1201)
    hz = ter.h(xs, 0 * xs)
    for ax, key in zip(axes, keys):
        p = P[key]
        ax.fill_between(xs, 0, hz, color='grey', alpha=0.5, label='Terrain')
        ax.fill_between(xs, hz, hz + H_CLR, color='grey', alpha=0.18,
                        label=f'Clearance band ({H_CLR:.0f} m)')
        for k, lg in enumerate(p['legs']):
            ax.plot(lg['x'], lg['z'], color=LEG_COLORS[k % 6], lw=2.2,
                    label=f'Leg {k+1}: B = {p["B"][k]*100:.1f} %')
        for i, (x, z) in enumerate(zip(p['xs'], p['zs'])):
            ax.plot([x - 2, x + 2], [z, z], color='k', lw=2)
            if 0 < i < p['K']:
                ax.plot([x, x], [float(ter.h(x, 0)), z], color='orange', lw=1.2, ls=':')
        ok = 'leg B ok' if p['feasible'] else 'leg B > 90 %'
        ok += ', incl. pad ok' if p['feasible_tot'] else ', incl. pad > 90 %'
        ax.set_title(f'{p["name"]} ({ok})\nrelays at x = '
                     + ', '.join(f'{x:.2f}' for x in p['xs'][1:-1]) + ' m', fontsize=9.5)
        ax.set_ylabel('z (m)')
        ax.set_ylim(0, 13)
        ax.grid(alpha=0.3)
        ax.legend(fontsize=7, loc='upper left', ncol=4)
    for ax in axes[-2:]:
        ax.set_xlabel('Horizontal distance along route x (m)')
    fig.suptitle('Altitude profiles (black ticks = rendezvous altitudes, '
                 'dotted = station above its ground point)', fontsize=12,
                 fontweight='bold')
    plt.tight_layout(rect=[0, 0, 1, 0.97])
    path = os.path.join(out_dir, 'altitude_profile.png')
    plt.savefig(path, dpi=130)
    plt.close()
    print(f'Saved: {path}')


def plot_battery_usage(data, out_dir):
    P = data['plans']
    fig, axes = plt.subplots(1, 2, figsize=(16, 6.8), sharey=True)
    for ax, field, title in ((axes[0], 'B', 'Leg battery $B_k$ (card definition, flown path)'),
                             (axes[1], 'B_tot', 'Per-drone battery incl. pad climb + landing')):
        width = 0.12
        max_k = max(P[k]['K'] for k in PLAN_ORDER)
        for j, key in enumerate(PLAN_ORDER):
            p = P[key]
            xs = np.arange(1, p['K'] + 1) + (j - 3) * width
            ax.bar(xs, [b * 100 for b in p[field]], width=width, color=PLAN_COLORS[key],
                   edgecolor='k', linewidth=0.5, label=p['name'])
        ax.axhline(90, color='crimson', ls='--', lw=1.5, label='Reserve limit 90 %')
        ax.axhline(30 / DRONE_RANGE * 100, color='navy', ls=':', lw=1.2,
                   label='2D model (30 m legs) 85.7 %')
        ax.set_xticks(np.arange(1, max_k + 1))
        ax.set_xticklabels([f'Drone {k}' for k in range(1, max_k + 1)])
        ax.set_ylim(0, 135)
        ax.set_title(title, fontsize=11)
        ax.grid(axis='y', alpha=0.3)
    axes[0].set_ylabel('Battery used (% of R)')
    h, lab = axes[1].get_legend_handles_labels()
    fig.legend(h, lab, fontsize=8.5, loc='lower center', ncol=5)
    fig.suptitle('Battery usage per relay drone', fontsize=12, fontweight='bold')
    plt.tight_layout(rect=[0, 0.1, 1, 0.95])
    path = os.path.join(out_dir, 'battery_usage.png')
    plt.savefig(path, dpi=130)
    plt.close()
    print(f'Saved: {path}')


def plot_delivery_time(data, out_dir):
    P = data['plans']
    fig, ax = plt.subplots(figsize=(12, 5.5))
    rows = list(PLAN_ORDER)
    for r, key in enumerate(rows):
        p = P[key]
        left = 0.0
        for k, lg in enumerate(p['legs']):
            ax.barh(r, lg['time'], left=left, height=0.55, color=LEG_COLORS[k % 6],
                    edgecolor='k', lw=0.5)
            left += lg['time']
            if k < p['K'] - 1:
                ax.barh(r, TAU_H, left=left, height=0.55, color='lightgray',
                        edgecolor='k', lw=0.5)
                left += TAU_H
        tag = '' if p['feasible'] else '  (leg B > 90 %)'
        if not p['flown']:
            tag += '  (not flyable: a drone > 100 % incl. pad)'
        ax.text(left + 0.3, r, f'{p["T"]:.2f} s{tag}', va='center', fontsize=9)
    ax.axvline(data['two_d']['T'], color='navy', ls=':', lw=1.2,
               label=f'2D model, K=4: {data["two_d"]["T"]:.2f} s')
    ax.set_yticks(range(len(rows)))
    ax.set_yticklabels([P[k]['name'] for k in rows], fontsize=9)
    ax.invert_yaxis()
    ax.set_xlim(0, 72)
    ax.set_xlabel('Time (s)')
    ax.set_title('Delivery time breakdown (coloured = leg flight along the 3D path, '
                 'grey = 3 s handoff dwell)', fontsize=11)
    ax.legend(fontsize=9, loc='lower right')
    ax.grid(axis='x', alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'delivery_time_breakdown.png')
    plt.savefig(path, dpi=130)
    plt.close()
    print(f'Saved: {path}')


def plot_handoff_detail(data, out_dir):
    P = data['plans']
    fig, axes = plt.subplots(3, 2, figsize=(14, 9), sharex=True)
    for col, key in enumerate(('terr_tot', 'flat_tot')):
        p = P[key]
        for h_i, h in enumerate(p['handoffs']):
            m = h['rel_t'] >= -(h['t_align'] + LEAD_MARGIN + 1.5)
            for r in range(3):
                axes[r, col].plot(h['rel_t'][m], h['rel'][m, r], color=LEG_COLORS[h_i % 6],
                                  lw=1.8, label=f'Station {h["station"]} '
                                  f'(x = {h["x"]:.1f} m, pad-to-rendezvous {h["climb"]:.2f} m)')
            for r in range(3):
                axes[r, col].axvline(h['t_cap'] - h['t_arr'], color=LEG_COLORS[h_i % 6],
                                     ls=':', lw=1)
                axes[r, col].axvline(h['t_launch'] - h['t_arr'], color=LEG_COLORS[h_i % 6],
                                     ls='--', lw=0.8, alpha=0.6)
        for r, lab in enumerate(['$\\Delta x$ (m)', '$\\Delta y$ (m)', '$\\Delta z$ (m)']):
            axes[r, col].set_ylabel(lab)
            axes[r, col].grid(alpha=0.3)
            axes[r, col].axhline(0, color='k', lw=0.6)
        axes[2, col].axhspan(-DELTA_Z, DELTA_Z, color='green', alpha=0.12)
        axes[0, col].set_title(f'{p["name"]}\nincoming minus outgoing; dashed = '
                               'outgoing launch, dotted = capture', fontsize=10)
        axes[0, col].legend(fontsize=7.5, loc='upper left')
        axes[2, col].set_xlabel('Time relative to incoming arrival (s)')
        axes[0, col].set_ylim(-30, 3)
    fig.suptitle('Handoff alignment: the outgoing drone climbs from its pad and is aligned '
                 'before the incoming drone arrives (green band = $\\delta_z$)',
                 fontsize=12, fontweight='bold')
    plt.tight_layout(rect=[0, 0, 1, 0.95])
    path = os.path.join(out_dir, 'handoff_detail.png')
    plt.savefig(path, dpi=130)
    plt.close()
    print(f'Saved: {path}')


def plot_terrain_sweep(data, out_dir):
    S = data['sweep']
    fig, axes = plt.subplots(2, 3, figsize=(15, 9))
    panels = [('eq_flat', 'Flat, equal spacing (leg B)'),
              ('flat', 'Flat, minimax (leg B)'),
              ('flat_tot', 'Flat, minimax incl. pad'),
              ('eq_terr', 'Terrain-following, equal spacing (leg B)'),
              ('terr', 'Terrain-following, minimax (leg B)'),
              ('terr_tot', 'Terrain-following, minimax incl. pad')]
    allv = [S[k][f] for k in S for f, _ in panels if np.isfinite(S[k][f])]
    vmin, vmax = min(allv), max(allv)
    for ax, (field, title) in zip(axes.ravel(), panels):
        M = np.array([[S[(pk, sg)][field] for sg in SWEEP_SIGMAS] for pk in SWEEP_PEAKS],
                     dtype=float)
        Mi = np.where(np.isfinite(M), M, np.nan)
        cmap = plt.get_cmap('viridis').copy()
        cmap.set_bad('lightgrey')
        ax.imshow(Mi, origin='lower', cmap=cmap, vmin=vmin, vmax=vmax, aspect='auto')
        for a in range(len(SWEEP_PEAKS)):
            for b in range(len(SWEEP_SIGMAS)):
                lab = f'{int(M[a, b])}' if np.isfinite(M[a, b]) else 'none'
                ax.text(b, a, lab, ha='center', va='center',
                        color='w' if M[a, b] < (vmin + vmax) / 2 else 'k', fontsize=11,
                        fontweight='bold')
        ax.set_xticks(range(len(SWEEP_SIGMAS)))
        ax.set_xticklabels([f'{s:g}' for s in SWEEP_SIGMAS])
        ax.set_yticks(range(len(SWEEP_PEAKS)))
        ax.set_yticklabels([f'{p:g}' for p in SWEEP_PEAKS])
        ax.set_xlabel('Hill width $\\sigma$ (m)')
        ax.set_title(title, fontsize=10)
        i5, j15 = SWEEP_PEAKS.index(HILL_PEAK), SWEEP_SIGMAS.index(HILL_SIGMA)
        ax.add_patch(plt.Rectangle((j15 - 0.5, i5 - 0.5), 1, 1, fill=False,
                                   ec='red', lw=2))
    for ax in axes[:, 0]:
        ax.set_ylabel('Hill peak (m)')
    fig.suptitle('Terrain sweep: smallest feasible number of legs K (red box = '
                 'main instance; none = no K <= 30 works; 2D model: K = 4 everywhere)',
                 fontsize=12,
                 fontweight='bold')
    plt.tight_layout(rect=[0, 0, 1, 0.93])
    path = os.path.join(out_dir, 'terrain_sweep.png')
    plt.savefig(path, dpi=130)
    plt.close()
    print(f'Saved: {path}')


def save_animation(data, out_dir):
    ter, p = data['ter'], data['plans']['terr_tot']
    t, P, leg = p['trace_t'], p['trace_p'], p['trace_leg']
    n = len(t)
    n_frames = math.ceil((n - 1) / GIF_STEP) + 1
    idx = [min(f * GIF_STEP, n - 1) for f in range(n_frames)]
    data['gif_frames'] = n_frames
    fig = plt.figure(figsize=(11, 5.5))
    ax = fig.add_subplot(111, projection='3d')
    _terrain_surface(ax, ter, alpha=0.3)
    for k, lg in enumerate(p['legs']):
        ax.plot(lg['x'], 0 * lg['x'], lg['z'], color=LEG_COLORS[k % 6], lw=1.2, alpha=0.5)
    for i, (x, z) in enumerate(zip(p['xs'], p['zs'])):
        if 0 < i < p['K']:
            ax.scatter([x], [0], [z], s=50, color='orange', edgecolors='k', depthshade=False)
    ax.scatter([p['xs'][0]], [0], [p['zs'][0]], s=80, marker='s', color='green',
               edgecolors='k', depthshade=False)
    ax.scatter([p['xs'][-1]], [0], [p['zs'][-1]], s=140, marker='*', color='red',
               edgecolors='k', depthshade=False)
    ax.set_xlim(-5, 125); ax.set_ylim(-15, 15); ax.set_zlim(0, 14)
    ax.set_box_aspect((4, 1, 0.9), zoom=1.1)
    ax.view_init(elev=22, azim=-70)
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)'); ax.set_zlabel('z (m)')
    parcel, = ax.plot([], [], [], 'o', color='#264653', ms=9)
    outgoing, = ax.plot([], [], [], '^', color='purple', ms=7)
    flash, = ax.plot([], [], [], 'o', color='yellow', ms=22, alpha=0.6, mec='orange')
    title = ax.set_title('')

    def outgoing_pos(ti):
        for h in p['handoffs']:
            if h['t_launch'] <= ti <= h['t_cap']:
                j = min(int(round((ti - h['t_launch']) / DT)), len(h['P_out']) - 1)
                return h['P_out'][j]
        return None

    def update(f):
        i = idx[f]
        ti = t[i]
        parcel.set_data([P[i, 0]], [P[i, 1]]); parcel.set_3d_properties([P[i, 2]])
        po = outgoing_pos(ti)
        if po is None:
            outgoing.set_data([], []); outgoing.set_3d_properties([])
        else:
            outgoing.set_data([po[0]], [po[1]]); outgoing.set_3d_properties([po[2]])
        fl = [h for h in p['handoffs'] if h['t_cap'] <= ti <= h['t_cap'] + 1.0]
        if fl:
            flash.set_data([fl[0]['x']], [0]); flash.set_3d_properties([fl[0]['z']])
        else:
            flash.set_data([], []); flash.set_3d_properties([])
        status = 'delivered' if i == n - 1 else f'leg {leg[i] + 1}/{p["K"]}'
        title.set_text(f'S036 3D Last-Mile Relay (terrain-following, pad counted, K={p["K"]})  '
                       f't = {ti:5.2f} s  {status}  z = {P[i, 2]:.2f} m')
        return [parcel, outgoing, flash, title]

    ani = FuncAnimation(fig, update, frames=n_frames, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer=PillowWriter(fps=20), dpi=80)
    plt.close()
    print(f'Saved: {path}  ({n_frames} frames)')


# ── Main ─────────────────────────────────────────────────────────────────────
def _fmt(v, f='{:.2f}'):
    return '[' + ', '.join(f.format(x) for x in v) + ']'


def print_results(data):
    ter, P = data['ter'], data['plans']
    print('=' * 72)
    print('S036 3D Last-Mile Relay — Simulation Results')
    print('=' * 72)
    print(f'  R = {DRONE_RANGE:.1f} m, R_eff = {R_EFF:.2f} m, alpha_climb = {ALPHA_CLIMB}, '
          f'v = {V_CRUISE} m/s, vz_max = {VZ_MAX} m/s, tau_h = {TAU_H} s')
    xs = np.linspace(0, L_2D, 2401)
    hz = ter.h(xs, 0 * xs)
    print(f'  Terrain on route: h(0) = {hz[0]:.4f} m, h(120) = {hz[-1]:.4f} m, '
          f'max {hz.max():.3f} m at x = {xs[hz.argmax()]:.2f} m')
    print(f'  D0 = (0, 0, {P["terr_mm"]["zs"][0]:.3f}), DK = (120, 0, '
          f'{P["terr_mm"]["zs"][-1]:.3f}); straight 3D distance '
          f'{math.hypot(L_2D, P["terr_mm"]["zs"][-1] - P["terr_mm"]["zs"][0]):.3f} m')
    print(f'  Flat cruise altitude Z_FLAT = {data["z_flat"]:.3f} m')
    td = data['two_d']
    print(f'  2D model (no altitude): K = {td["K"]}, B = {td["B"]*100:.1f} % per leg, '
          f'T = {td["T"]:.2f} s')
    for mode in ('flat', 'terrain'):
        K, b = data['eq'][mode]
        print(f'  Equal spacing, {mode:7s}: smallest feasible K = {K} (max B {b*100:.1f} %)')
    for key in PLAN_ORDER:
        p = P[key]
        print()
        print(f'  {p["name"]}:')
        print(f'    relay x (m)            : {_fmt(p["xs"][1:-1])}')
        print(f'    rendezvous z (m)       : {_fmt(p["zs"])}')
        print(f'    legs K                 : {p["K"]}')
        print(f'    leg path (m)           : {_fmt([lg["path"] for lg in p["legs"]])}')
        print(f'    leg chord 3D (m)       : {_fmt([lg["chord"] for lg in p["legs"]])}')
        print(f'    leg climb (m)          : {_fmt([lg["climb"] for lg in p["legs"]])}')
        print(f'    B_k flown (%)          : {_fmt([b*100 for b in p["B"]], "{:.1f}")}  '
              f'max {max(p["B"])*100:.1f}')
        print(f'    B_k card chord (%)     : {_fmt([b*100 for b in p["B_card"]], "{:.1f}")}  '
              f'max {max(p["B_card"])*100:.1f}')
        print(f'    B_k incl. overhead (%) : {_fmt([b*100 for b in p["B_tot"]], "{:.1f}")}  '
              f'max {max(p["B_tot"])*100:.1f}')
        print(f'    pad-to-rendezvous gap  : {_fmt([o[4] for o in p["ovh"]])} m, '
              f'align time {_fmt([o[2] for o in p["ovh"]])} s')
        print(f'    feasible (leg / incl.) : {p["feasible"]} / {p["feasible_tot"]}')
        print(f'    total path {p["path"]:.2f} m, total climb {p["climb"]:.2f} m, '
              f'flight time {p["t_flight"]:.2f} s')
        print(f'    delivery time T        : {p["T"]:.2f} s')
        if p['flown']:
            caps = [h['t_cap'] - h['t_arr'] for h in p['handoffs']]
            print(f'    time-stepped handoffs  : capture rel. arrival {_fmt(caps)} s, '
                  f'outgoing launch {_fmt([h["t_launch"] - h["t_arr"] for h in p["handoffs"]])} s, '
                  f'max residual after dwell '
                  f'{max([h["resid"] for h in p["handoffs"]] + [0.0])*1000:.3f} mm')
        else:
            print('    time-stepped handoffs  : not flown (a drone would exceed 100 % '
                  'incl. pad climb and landing)')
    print()
    print('  Terrain sweep (peak m, sigma m): min K  eq-terrain / mm-terrain / mm-flat / '
          'mm-terrain+ovh / mm-flat+ovh   [mm bottleneck terrain / flat %]')
    for pk in SWEEP_PEAKS:
        for sg in SWEEP_SIGMAS:
            s = data['sweep'][(pk, sg)]
            print(f'    ({pk:4.1f}, {sg:4.1f}): {s["eq_terr"]} / {s["terr"]} / {s["flat"]} / '
                  f'{s["terr_tot"]} / {s["flat_tot"]}   [{s["terr_b"]*100:.1f} / '
                  f'{s["flat_b"]*100:.1f}]  z_flat {s["z_flat"]:.2f}')
    n = len(SWEEP_PEAKS) * len(SWEEP_SIGMAS)
    S = data['sweep'].values()
    print(f'  Sweep counts ({n} terrains): mm-terrain K < mm-flat K in '
          f'{sum(s["terr"] < s["flat"] for s in S)}, equal in '
          f'{sum(s["terr"] == s["flat"] for s in S)}, greater in '
          f'{sum(s["terr"] > s["flat"] for s in S)}')
    print(f'                      incl. overhead: terrain < flat in '
          f'{sum(s["terr_tot"] < s["flat_tot"] for s in S)}, equal in '
          f'{sum(s["terr_tot"] == s["flat_tot"] for s in S)}, greater in '
          f'{sum(s["terr_tot"] > s["flat_tot"] for s in S)}')
    print(f'                      equal spacing: terrain K < flat K in '
          f'{sum(s["eq_terr"] < s["eq_flat"] for s in S)}, equal in '
          f'{sum(s["eq_terr"] == s["eq_flat"] for s in S)}, greater in '
          f'{sum(s["eq_terr"] > s["eq_flat"] for s in S)}')
    print(f'                      equal-spacing terrain K > 4 in '
          f'{sum(s["eq_terr"] > 4 for s in S)}; mm-terrain K > 4 in '
          f'{sum(s["terr"] > 4 for s in S)}')
    print('=' * 72)


if __name__ == '__main__':
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    data = run_simulation()
    print_results(data)
    plot_trajectory_3d(data, OUTPUT_DIR)
    plot_altitude_profile(data, OUTPUT_DIR)
    plot_battery_usage(data, OUTPUT_DIR)
    plot_delivery_time(data, OUTPUT_DIR)
    plot_handoff_detail(data, OUTPUT_DIR)
    plot_terrain_sweep(data, OUTPUT_DIR)
    save_animation(data, OUTPUT_DIR)
