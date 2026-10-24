"""
S100 Ultimate Grand Challenge — Multi-Domain Integration
========================================================
Five drones share a 100 x 100 x 30 m arena and fly four sequential mission phases under
one mission sequencer, with a 300 s total budget:

  Phase 1  Aerial ball catching   Drone A   predictive catch point (S094 code, imported);
                                            proportional navigation (PN) as the comparison
  Phase 2  Logistics delivery     B, C      nearest-neighbour TSP split, capacity 1,
                                            battery trigger before every trip
  Phase 3  Search and rescue      D         Bayesian belief maps for two survivors,
                                            greedy max-belief frontier, battery abort
  Phase 4  Grand formation        all five  Hungarian assignment + PID + ORCA-lite repulsion

Every drone draws energy from the same battery model in every phase and must never fly on
an empty battery; drones talk over an 80 m multi-hop broadcast network and the sequencer
(on Drone E) only learns of an event when the sender is connected to it.

Monte Carlo over N_SEEDS missions with common random numbers: the throws and mocap noise
(Phase 1), the survivor positions and the sensor draws (Phase 3) depend only on the seed,
never on the strategy or on the batch, so every variant flies the same missions.

Usage:
    conda activate drones
    python src/05_special_entertainment/s100_grand_challenge.py
"""

import os
import importlib.util
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.colors import LogNorm
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401
from scipy.optimize import linear_sum_assignment
from scipy.special import xlogy
from scipy.stats import binomtest, chi2_contingency

HERE = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location('s094_ball_catching',
                                               os.path.join(HERE, 's094_ball_catching.py'))
m94 = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(m94)

OUT_DIR = os.environ.get('S100_OUT_DIR') or os.path.normpath(
    os.path.join(HERE, '..', '..', 'outputs', '05_special_entertainment', 's100_grand_challenge'))

# ── Mission (card) ────────────────────────────────────────────────────────────
ARENA = np.array([100.0, 100.0, 30.0])
T_TOTAL = 300.0
T_DEADLINE = np.array([60.0, 160.0, 240.0, 300.0])   # absolute phase deadlines (card FSM)
WEIGHTS = np.array([2.0, 3.0, 3.0, 2.0])
OT_PENALTY = 0.05            # points per second over T_TOTAL
T_HARD = 330.0               # card code: the mission loop stops at T_TOTAL + 30 s
DT = 0.1                     # s, mission tick (10 Hz broadcast)
R_COMM = 80.0                # m
NAMES = ['A', 'B', 'C', 'D', 'E']
START = np.array([[50.0, 90.0, 2.0],    # A catcher
                  [45.0, 92.0, 2.0],    # B courier (its pad)
                  [55.0, 92.0, 2.0],    # C courier (its pad)
                  [50.0, 88.0, 2.0],    # D scout
                  [50.0, 86.0, 15.0]])  # E formation lead, holding
DECK_Z = 2.0                 # the launch deck is 2 m high (card start altitudes)
K_TICKS = int(round(T_HARD / DT))

# ── Phase 1: aerial ball catching (card values = S094 values) ─────────────────
CATCH_CENTRE = np.array([80.0, 80.0, 0.0])
N_BALLS = 3
T_RELOAD = 5.0
N_BALLS_SIM = 5              # balls simulated per mission (for the N_balls sweep)
DT1 = m94.DT                 # 0.002 s
SUB = int(round(DT / DT1))   # 50 sub-steps per tick
V1, A1 = m94.V_MAX, m94.A_MAX   # 8 m/s, 10 m/s^2
SETTLE_TOL = 1e-3            # m and m/s: catcher on station
P1_STRATS = ['pcp', 'pn']
P1_LABEL = {'pcp': 'Predictive catch point', 'pn': 'Proportional navigation'}

# ── Phase 2: logistics (card) ─────────────────────────────────────────────────
V_COURIER = 5.0
C_FLY = 0.004                # battery fraction per metre at cruise
C_HOVER = 0.003              # battery fraction per second hovering
TAU_DELIVER = 3.0
E_RESERVE = 0.15
BASE_POS = np.array([50.0, 95.0, 2.0])
PADS = START[1:3].copy()     # each courier loads and lands on its own pad
DELIVERY_WPS = np.array([[20.0, 70.0, 5.0], [75.0, 60.0, 5.0], [40.0, 40.0, 5.0]])
T_LOAD = 3.0                 # s, reload on the pad (added; = tau_deliver)

# ── Phase 3: SAR (card) ───────────────────────────────────────────────────────
SAR_HI = 50.0
GRID_RES = 0.5
GRID_N = 100
SCOUT_H = 8.0
SENSOR_R = 3.0
P_DET, P_FA = 0.85, 0.01
SIGMA0 = 8.0
LKP = np.array([[15.0, 20.0], [35.0, 35.0]])
DET_THRESH = 0.60
V_SCOUT = 4.0
SENSOR_ALT_TOL = 0.5         # m, the sensor reports only near the scan altitude (added)
XC = (np.arange(GRID_N) + 0.5) * GRID_RES

# ── Phase 4: formation (card, except the show altitude) ───────────────────────
Z_SHOW = 25.0                # card: 50 m, above the 30 m arena ceiling
STAR_R = 8.0
V_FORM = 6.0
KP, KI, KD = 1.2, 0.05, 0.4
K_REP, R_SAFE = 2.0, 1.2
EPS_FORM = 0.15
STAR_CENTRE = np.array([50.0, 50.0, Z_SHOW])
_ang = 2 * np.pi * np.arange(5) / 5 - np.pi / 2
STAR = np.stack([STAR_CENTRE[0] + STAR_R * np.cos(_ang), STAR_CENTRE[1] + STAR_R * np.sin(_ang),
                 np.full(5, Z_SHOW)], axis=1)

# ── Fleet model for the 10 Hz phases (added) ──────────────────────────────────
A_FLEET = 10.0               # m/s^2, every drone (= the S094 catcher limit)
A_BRAKE = 5.0                # m/s^2, braking curve of the go-to controller
V_PHYS = 8.0                 # m/s, no drone ever exceeds this
ARRIVE_TOL = 1e-3            # m and m/s
FAILSAFE_MARGIN = 0.03       # battery fraction kept for braking + a vertical landing

# ── Monte Carlo ───────────────────────────────────────────────────────────────
N_SEEDS = 200
KAPPAS = [1.0, 1.25, 1.5, 2.0, 3.0, 5.0]   # battery capacity multiplier sweep
C_FLY_SWEEP = [0.004, 0.0045, 0.005, 0.0055, 0.006, 0.008]
RELOAD_SWEEP = [2.0, 5.0, 10.0, 15.0, 20.0, 25.0]
SEED_P1, SEED_P3 = 1001, 3001
GIF_EVERY = 15               # ticks per GIF frame (1.5 s)

COL = {'A': '#d62728', 'B': '#ff7f0e', 'C': '#9467bd', 'D': '#8c564b', 'E': '#e6ab02'}
BALL_C, TARGET_C, GREY = '#1f77b4', '#2ca02c', '#7f7f7f'

# mode codes for the 10 Hz loop
PARKED, GOTO, PID, TRACK = 0, 1, 2, 3


def _norm(a):
    return np.linalg.norm(a, axis=-1)


# ═════════════════════════════════════════════════════════════════════════════
# Phase 1 — aerial ball catching (S094 model, imported unchanged)
# ═════════════════════════════════════════════════════════════════════════════
def make_p1_throws(seeds, ball):
    """S094 throws, one independent generator per (seed, ball): common random numbers that
    do not depend on the batch or the strategy. Mirrors m94.make_throws."""
    th = [m94.sample_throw(np.random.default_rng([SEED_P1, int(s), ball])) for s in seeds]
    p0 = np.array([t[0] for t in th])
    vb0 = np.array([t[1] for t in th])
    pd0 = np.array([t[2] for t in th])
    v0 = np.array([t[4] for t in th])
    n_meas = m94.N_TRUTH // m94.MEAS_EVERY + 2
    noise = np.array([np.random.default_rng(int(t[3])).standard_normal((n_meas, 3)) for t in th])
    x = np.concatenate([p0, vb0], axis=1)
    truth = np.empty((len(th), m94.N_TRUTH, 6))
    truth[:, 0] = x
    for i in range(1, m94.N_TRUTH):
        x = m94.rk4(x, m94.DT)
        truth[:, i] = x
    below = (truth[:, :, 2] < m94.Z_MIN_BALL) & (truth[:, :, 5] < 0)
    assert below.any(axis=1).all()
    n_end = np.argmax(below, axis=1)
    return dict(p0=p0, vb0=vb0, pd0=pd0, v0=v0, noise=noise, truth=truth, n_end=n_end)


def _p1_step(p, v, q):
    """One 2 ms step of the S094 go-to tracker (braking curve), S094 integration."""
    u = m94.altitude_guard(p, v, m94.goto_accel(p, v, q))
    v_new = m94.sat(v + u * DT1, V1)
    return p + v_new * DT1, v_new


def p1_reposition(p, v, q, max_steps=12000):
    """Fly the catcher from (p, v) to hover station q at 2 ms and settle (|e|, |v| <= 1 mm).
    Returns positions per step (frozen after settling) and the settle step."""
    B = len(p)
    p, v = p.copy(), v.copy()
    settled = np.zeros(B, bool)
    n_settle = np.zeros(B, int)
    hist = [p.copy()]
    for i in range(max_steps):
        ok = (_norm(q - p) <= SETTLE_TOL) & (_norm(v) <= SETTLE_TOL) & ~settled
        n_settle[ok] = i
        settled |= ok
        v[settled] = 0.0      # final command |dv| <= 1 mm/s in one step = 0.5 m/s^2
        if settled.all():
            break
        p_new, v_new = _p1_step(p, v, q)
        a_app = _norm(v_new - v) / DT1
        assert np.all(a_app <= A1 * (1 + 1e-6)) and np.all(_norm(v_new) <= V1 * (1 + 1e-9))
        p = np.where(settled[:, None], p, p_new)
        v = np.where(settled[:, None], v, v_new)
        assert np.all(p[:, 2] >= m94.Z_LO - 1e-3)
        hist.append(p.copy())
    assert settled.all(), 'catcher did not settle on station'
    return dict(hist=np.array(hist), n_settle=n_settle, p_final=p.copy())


def run_phase1(strategy, seeds):
    """Simulate N_BALLS_SIM sequential throws per mission for one guidance law. The throws
    are the same for every law; the catcher's repositioning starts from where the previous
    throw left it."""
    S = len(seeds)
    p = np.tile(START[0], (S, 1))
    v = np.zeros((S, 3))
    n_now = np.zeros(S, int)
    reload_steps = int(round(T_RELOAD / DT1))
    balls = []
    for b in range(N_BALLS_SIM):
        th = make_p1_throws(seeds, b)
        rep = p1_reposition(p, v, CATCH_CENTRE + th['pd0'])
        settle = rep['n_settle']
        wait = np.maximum(reload_steps - settle, 0) if b > 0 else np.zeros(S, int)
        launch = n_now + settle + wait
        assert np.all(_norm(rep['p_final'] - CATCH_CENTRE - th['pd0']) <= SETTLE_TOL)
        th['pd0'] = rep['p_final'] - CATCH_CENTRE      # exact station, within 1 mm
        est = m94.run_ekf(th, m94.SIGMA_Z)
        out = m94.simulate(strategy, th, est, record=True)
        pdrec = out['rec']['pd'] + CATCH_CENTRE
        n_end = th['n_end']
        idx = np.arange(S)
        p = pdrec[n_end, idx].copy()
        v = (pdrec[n_end, idx] - pdrec[n_end - 1, idx]) / DT1
        balls.append(dict(th=th, caught=out['caught'], t_catch=out['t_catch'],
                          rep_hist=rep['hist'].astype(np.float64), settle=settle, wait=wait,
                          launch=launch, n_start=n_now.copy(), p_settled=rep['p_final'],
                          pdrec=pdrec, n_end=n_end))
        n_now = launch + n_end
    return balls


def p1_cut_index(B, s):
    """Local throw step after which the catch is complete (or the ball is lost)."""
    if B['caught'][s]:
        n = min(int(B['t_catch'][s] / DT1 + 1e-9), int(B['n_end'][s]) - 1)
        return n + 1
    return int(B['n_end'][s])


def p1_timeline(balls, n_balls, t_reload):
    """Phase 1 end tick per mission from the simulated pieces, for any N_balls / T_reload
    (the throws themselves do not depend on either; the catcher hovers while waiting)."""
    S = len(balls[0]['caught'])
    reload_steps = int(round(t_reload / DT1))
    end_step = np.zeros(S, int)
    success = np.zeros(S, bool)
    first = np.full(S, -1)
    for s in range(S):
        n_start = 0
        for b in range(n_balls):
            B = balls[b]
            st = int(B['settle'][s])
            launch = n_start + (st if b == 0 else max(st, reload_steps))
            if B['caught'][s]:
                end_step[s] = launch + p1_cut_index(B, s)
                success[s], first[s] = True, b
                break
            n_start = launch + int(B['n_end'][s])
            end_step[s] = n_start
    tick = -(-end_step // SUB)
    return tick, success, first


def assemble_phase1(balls):
    """Stitch the catcher's 2 ms path for the mission (N_BALLS, T_RELOAD), up to the first
    tick at or after the catch / the loss of the last ball, then brake to a hold."""
    S = len(balls[0]['caught'])
    tick, success, first = p1_timeline(balls, N_BALLS, T_RELOAD)
    assert np.all(tick * DT <= T_DEADLINE[0]), 'Phase 1 deadline reached (not modelled)'
    tracks = []
    cut_states = np.zeros((S, 2, 3))
    cut_step = np.zeros(S, int)
    b_last = np.where(first >= 0, first, N_BALLS - 1)
    for s in range(S):
        segs = []
        for b in range(b_last[s] + 1):
            B = balls[b]
            segs.append(B['rep_hist'][:B['settle'][s], s])
            w = int(B['launch'][s] - B['n_start'][s] - B['settle'][s])
            if w > 0:
                segs.append(np.repeat(B['p_settled'][s][None], w, axis=0))
            c = int(B['n_end'][s]) if b < b_last[s] else p1_cut_index(B, s)
            segs.append(B['pdrec'][:c, s])
            if b == b_last[s]:
                cut_states[s, 0] = B['pdrec'][c, s]
                cut_states[s, 1] = (B['pdrec'][c, s] - B['pdrec'][c - 1, s]) / DT1
                cut_step[s] = int(B['launch'][s]) + c
        tracks.append(np.concatenate(segs + [cut_states[s, 0][None]], axis=0))
    # after the cut the catcher brakes to a hold (S094 tracker, altitude guard) and settles;
    # it is handed to the 10 Hz loop at the first tick at or after it has settled
    assert np.all(tick * SUB >= cut_step)
    p, v = cut_states[:, 0].copy(), cut_states[:, 1].copy()
    q = p.copy()
    bp, bv = [], []
    settled = np.zeros(S, bool)
    n_stop = np.zeros(S, int)
    for i in range(int(round(3.0 / DT1))):
        ok = (_norm(v) <= SETTLE_TOL) & ~settled
        n_stop[ok] = i
        settled |= ok
        v[settled] = 0.0
        if settled.all():
            break
        p_new, v_new = _p1_step(p, v, q)
        assert np.all(_norm(v_new - v) / DT1 <= A1 * (1 + 1e-6))
        p = np.where(settled[:, None], p, p_new)
        v = np.where(settled[:, None], v, v_new)
        assert np.all(p[:, 2] >= m94.Z_LO - 1e-3)
        bp.append(p.copy())
    assert settled.all()
    bp = np.array(bp)
    k_track = np.maximum(tick, -(-(cut_step + n_stop) // SUB))
    K1 = int(k_track.max())
    track_ticks = np.zeros((S, K1 + 1, 3))
    e_ticks = np.zeros((S, K1 + 1))
    p1_len = np.zeros(S)
    for s in range(S):
        m = int(k_track[s] * SUB - cut_step[s])
        ns = int(n_stop[s])
        ext = bp[:min(m, ns), s]
        if m > ns:
            ext = np.concatenate([ext, np.repeat((bp[ns - 1, s] if ns > 0 else cut_states[s, 0])[None],
                                                 m - ns, axis=0)], axis=0)
        tr = np.concatenate([tracks[s], ext], axis=0)
        assert len(tr) == k_track[s] * SUB + 1
        spd = _norm(np.diff(tr, axis=0)) / DT1
        assert np.all(spd <= V1 * (1 + 1e-9))
        rate = np.maximum(C_HOVER, C_FLY * spd)            # battery model (all phases)
        cum = np.concatenate([[0.0], np.cumsum(rate * DT1)])
        tt = tr[::SUB]
        track_ticks[s, :k_track[s] + 1] = tt
        track_ticks[s, k_track[s] + 1:] = tt[-1]
        e_ticks[s, :k_track[s] + 1] = cum[::SUB]
        e_ticks[s, k_track[s] + 1:] = cum[-1]
        p1_len[s] = float(np.sum(spd) * DT1)
        tracks[s] = tr
    t_event = np.where(success, 0.0, np.nan)
    for s in range(S):
        if success[s]:
            B = balls[first[s]]
            t_event[s] = float(B['launch'][s]) * DT1 + float(B['t_catch'][s])
    return dict(S=S, k1_end=tick, k_track=k_track, success=success, first=first,
                track=track_ticks, e_used=e_ticks, t_event=t_event, path_len=p1_len,
                tracks_2ms=tracks)


# ═════════════════════════════════════════════════════════════════════════════
# Phase 2 — courier plans
# ═════════════════════════════════════════════════════════════════════════════
def _d_trip(c, w):
    return float(np.linalg.norm(DELIVERY_WPS[w] - PADS[c]))


def nn_plan():
    """Card heuristic: repeatedly give the remaining waypoint nearest to a courier to that
    courier. Couriers carry one package and return to their pad between trips, so a
    courier's position at every decision is its pad."""
    remaining = [0, 1, 2]
    plan = ([], [])
    while remaining:
        _, w, c = min((_d_trip(c, w), w, c) for w in remaining for c in range(2))
        plan[c].append(w)
        remaining.remove(w)
    return plan


def plan_makespan(plan):
    """Analytic courier time: round trips at cruise speed + dwell + reloads (no ramps)."""
    t = []
    for c in range(2):
        tc = sum(2 * _d_trip(c, w) / V_COURIER + TAU_DELIVER for w in plan[c])
        t.append(tc + T_LOAD * max(len(plan[c]) - 1, 0))
    return max(t), sum(2 * _d_trip(c, w) for c in range(2) for w in plan[c])


def exhaustive_plan():
    """All 8 waypoint-to-courier splits (nearest trip first); minimum makespan, then
    minimum total distance."""
    best = None
    for mask in range(8):
        plan = tuple(sorted([w for w in range(3) if ((mask >> w) & 1) == c],
                            key=lambda w, c=c: _d_trip(c, w)) for c in range(2))
        mk, tot = plan_makespan(plan)
        if best is None or (mk, tot) < best[0]:
            best = ((mk, tot), plan)
    return best[1]


# ═════════════════════════════════════════════════════════════════════════════
# Phase 3 — belief helpers
# ═════════════════════════════════════════════════════════════════════════════
def sar_prior(lkp):
    d2 = (XC[:, None] - lkp[0])**2 + (XC[None, :] - lkp[1])**2
    p = np.exp(-d2 / (2 * SIGMA0**2))
    return p / p.sum()


PRIORS = np.stack([sar_prior(LKP[0]), sar_prior(LKP[1])])      # (2, 100, 100), [ix, iy]


def survivors_for(seed):
    rng = np.random.default_rng([SEED_P3, int(seed)])
    pos = np.zeros((2, 2))
    for s in range(2):
        k = rng.choice(GRID_N * GRID_N, p=PRIORS[s].ravel())
        pos[s] = [XC[k // GRID_N], XC[k % GRID_N]]
    return pos


def sensor_draws(seed):
    return np.random.default_rng([SEED_P3 + 1, int(seed)]).random((K_TICKS + 1, 2))


def box3(b):
    """3 x 3 patch sums over the last two axes."""
    c = np.cumsum(np.cumsum(b, axis=-1), axis=-2)
    c = np.pad(c, [(0, 0)] * (b.ndim - 2) + [(1, 0), (1, 0)])
    return c[..., 3:, 3:] - c[..., :-3, 3:] - c[..., 3:, :-3] + c[..., :-3, :-3]


# The belief maps are stored unnormalised on a grid padded by PADC zero cells per side,
# with one running sum Z per map. An observation only changes the 13 x 13 cells around the
# scout (footprint radius 3 m = 6 cells), and the card's normalised update is recovered
# exactly as B / Z (the likelihood outside the footprint is the same for every cell).
PADC = 12
NPAD = GRID_N + 2 * PADC
XP = (np.arange(NPAD) - PADC + 0.5) * GRID_RES
WIN = np.arange(-6, 7)


def padded_priors(S):
    B = np.zeros((S, 2, NPAD, NPAD))
    B[:, :, PADC:PADC + GRID_N, PADC:PADC + GRID_N] = PRIORS
    return B


def p3_observe(Braw, Z, ix, dxy, r):
    """Multiply the likelihood ratio r (n, 2) into the footprint cells of missions ix."""
    ci = np.floor(dxy / GRID_RES).astype(int)
    ok = np.all((ci >= -6) & (ci <= GRID_N + 5), axis=1)       # footprint touches the zone
    ix, dxy, r, ci = ix[ok], dxy[ok], r[ok], ci[ok]
    if len(ix) == 0:
        return
    rows = ci[:, 0, None] + PADC + WIN
    cols = ci[:, 1, None] + PADC + WIN
    fp = (((XP[rows] - dxy[:, 0, None])**2)[:, :, None]
          + ((XP[cols] - dxy[:, 1, None])**2)[:, None, :]) <= SENSOR_R**2
    I = ix[:, None, None, None]
    Sv = np.arange(2)[None, :, None, None]
    Rr, Cc = rows[:, None, :, None], cols[:, None, None, :]
    W = Braw[I, Sv, Rr, Cc]
    Wn = W * (1.0 + fp[:, None] * (r[:, :, None, None] - 1.0))
    Z[ix] += (Wn - W).sum(axis=(2, 3))
    Braw[I, Sv, Rr, Cc] = Wn


def _inner(Braw, ix):
    """Unpadded maps of missions ix (a view-based gather that copies only what is used)."""
    V = Braw[:, :, PADC:PADC + GRID_N, PADC:PADC + GRID_N]
    return V[ix]


def p3_target(Braw, Z, alpha, ix, dxy):
    """Card policy: the cell with the highest combined belief sum_s alpha_s B_s, ties
    (within 1e-9 relative) broken by proximity to the scout."""
    n = len(ix)
    V = Braw[:, :, PADC:PADC + GRID_N, PADC:PADC + GRID_N]
    w = alpha[ix] / Z[ix]
    comb = np.empty((n, GRID_N, GRID_N))
    for a in range(n):                     # per mission: two views, no large temporaries
        np.multiply(V[ix[a], 0], w[a, 0], out=comb[a])
        comb[a] += V[ix[a], 1] * w[a, 1]
    comb = comb.reshape(n, -1)
    k = np.argmax(comb, axis=1)
    m = comb[np.arange(n), k]
    cand = comb >= (m * (1 - 1e-9))[:, None]
    multi = np.where(cand.sum(axis=1) > 1)[0]
    if len(multi):
        d2 = ((XC[None, :, None] - dxy[multi, 0, None, None])**2
              + (XC[None, None, :] - dxy[multi, 1, None, None])**2).reshape(len(multi), -1)
        k[multi] = np.argmin(np.where(cand[multi], d2, np.inf), axis=1)
    return np.stack([XC[k // GRID_N], XC[k % GRID_N]], axis=1)


def unpad(Braw, Z):
    return Braw[..., PADC:PADC + GRID_N, PADC:PADC + GRID_N] / Z[..., None, None]


# ═════════════════════════════════════════════════════════════════════════════
# 10 Hz motion
# ═════════════════════════════════════════════════════════════════════════════
def goto_step(p, v, q, vmax):
    """Braking-curve go-to with an acceleration limit (exact arrival, no snapping)."""
    e = q - p
    D = _norm(e)
    dh = e / np.maximum(D, 1e-12)[..., None]
    speed = np.minimum(np.minimum(vmax, np.sqrt(2 * A_BRAKE * D)), D / DT)
    dv = ground_barrier(p, dh * speed[..., None]) - v
    n = _norm(dv)
    lim = A_FLEET * DT
    dv = dv * np.where(n > lim, lim / np.maximum(n, 1e-12), 1.0)[..., None]
    v_new = v + dv
    return p + v_new * DT, v_new


def ground_barrier(p, v_cmd):
    """Never command a descent faster than the braking curve to the ground allows."""
    vz_min = -np.sqrt(2 * A_BRAKE * np.maximum(p[..., 2], 0.0))
    out = v_cmd.copy()
    out[..., 2] = np.maximum(v_cmd[..., 2], vz_min)
    return out


def rate_limit(v, v_cmd):
    dv = v_cmd - v
    n = _norm(dv)
    lim = A_FLEET * DT
    return v + dv * np.where(n > lim, lim / np.maximum(n, 1e-12), 1.0)[..., None]


def orca_lite(pos):
    diff = pos[:, :, None, :] - pos[:, None, :, :]
    d = _norm(diff)
    idx = np.arange(5)
    d[:, idx, idx] = np.inf
    act = d < R_SAFE
    mag = np.where(act, K_REP * (1.0 / d - 1.0 / R_SAFE) / d**2, 0.0)
    return np.sum((mag / d)[..., None] * diff, axis=2), act


def formation_step(pos, vel, goal, integ, e_prev, mp, orca=True, anti_windup=True, ki=KI):
    """Card PID (velocity command) + ORCA-lite repulsion, |v| <= V_FORM, |a| <= A_FLEET.
    anti_windup: conditional integration (S085) - the integral is frozen while the PID
    command exceeds V_FORM."""
    err = goal - pos
    integ_new = integ + err * DT
    u = KP * err + ki * integ_new + KD * (err - e_prev) / DT
    if anti_windup:
        sat = _norm(u) > V_FORM
        integ_new = np.where(sat[..., None], integ, integ_new)
        u = KP * err + ki * integ_new + KD * (err - e_prev) / DT
    integ = np.where(mp[..., None], integ_new, integ)
    e_prev = np.where(mp[..., None], err, e_prev)
    close = np.zeros(pos.shape[:2] + (pos.shape[1],), bool)
    if orca:
        F, close = orca_lite(pos)
        u = u + F
    n = _norm(u)
    u = u * np.where(n > V_FORM, V_FORM / np.maximum(n, 1e-12), 1.0)[..., None]
    v_p = rate_limit(vel, ground_barrier(pos, u))
    return pos + v_p * DT, v_p, integ, e_prev, close


def run_formation(p_start, v_start, goal, n_max, orca=True, anti_windup=True, ki=KI):
    """Phase 4 on its own, from the mission's Phase 4 start states (all five drones get
    the assignment at once; battery not modelled - used for the kappa = 5 ablations, where
    no battery rule binds). Returns convergence tick, min separation, ORCA ticks, path."""
    S = len(p_start)
    pos, vel = p_start.copy(), v_start.copy()
    integ = np.zeros_like(pos)
    e_prev = goal - pos
    mp = np.ones((S, 5), bool)
    k_conv = np.full(S, -1)
    minsep = np.full(S, np.inf)
    orca_ticks = np.zeros(S, int)
    path = np.zeros(S)
    peak_err = np.zeros(S)
    iu = np.triu_indices(5, 1)
    done = np.zeros(S, bool)
    for k in range(1, int(n_max.max()) + 1):
        act = ~done & (k <= n_max)
        if not act.any():
            break
        p_p, v_p, integ_n, e_prev_n, close = formation_step(pos, vel, goal, integ, e_prev, mp,
                                                            orca, anti_windup, ki)
        a_ = act[:, None, None]
        assert np.all(_norm(v_p[act]) <= V_FORM + 1e-9)
        assert np.all(_norm(v_p - vel)[act] / DT <= A_FLEET * (1 + 1e-6))
        assert np.all(p_p[act][..., 2] >= -1e-9)
        path += np.where(act, _norm(v_p).sum(axis=1) * DT, 0.0)
        orca_ticks += act & np.any(close, axis=(1, 2))
        pos, vel = np.where(a_, p_p, pos), np.where(a_, v_p, vel)
        integ, e_prev = np.where(a_, integ_n, integ), np.where(a_, e_prev_n, e_prev)
        d = _norm(pos[:, :, None] - pos[:, None])[:, iu[0], iu[1]].min(axis=1)
        minsep = np.where(act, np.minimum(minsep, d), minsep)
        err = _norm(goal - pos).max(axis=1)
        peak_err = np.where(act, np.maximum(peak_err, (pos[..., 2] - Z_SHOW).max(axis=1)), peak_err)
        conv = act & (err <= EPS_FORM)
        k_conv[conv] = k
        done |= conv
    return dict(k_conv=k_conv, minsep=minsep, orca_ticks=orca_ticks, path=path,
                z_over=peak_err)


def connectivity(pos):
    d = _norm(pos[:, :, None, :] - pos[:, None, :, :])
    adj = d <= R_COMM
    conn = np.zeros(pos.shape[:2], bool)
    conn[:, 4] = True                         # the sequencer runs on Drone E
    for _ in range(4):
        conn = conn | np.any(adj & conn[:, None, :], axis=2)
    return conn, d


# ═════════════════════════════════════════════════════════════════════════════
# Mission loop (vectorised over missions)
# ═════════════════════════════════════════════════════════════════════════════
def run_mission(p1, seeds, kappa=1.0, plan=None, c_fly=C_FLY, orca=True,
                assignment='hungarian', anti_windup=True, policy='greedy', failsafe_on=True,
                log=False, phase2_only=False):
    S = len(seeds)
    plan = plan if plan is not None else nn_plan()
    cfly, chov = c_fly / kappa, C_HOVER / kappa
    pos = np.repeat(START[None], S, axis=0)
    vel = np.zeros((S, 5, 3))
    E = np.ones((S, 5))
    air = np.zeros((S, 5), bool)
    air[:, 4] = True
    mode = np.full((S, 5), PARKED)
    mode[:, 4] = GOTO
    target = pos.copy()
    vmax = np.zeros((S, 5))
    vmax[:, 4] = V_FORM
    phase = np.full(S, 2 if phase2_only else 1)
    t_end = np.full((S, 4), np.nan)
    if not phase2_only:
        air[:, 0] = True
        mode[:, 0] = TRACK
        k1_end = p1['k1_end']
        k_track = p1['k_track']
        pos[:, 0] = p1['track'][:, 0]
    else:
        k1_end = np.zeros(S, int)
        k_track = np.zeros(S, int)
        t_end[:, 0] = 0.0
    # Phase 2
    cst = np.where(phase[:, None] == 2, 1, 0) * np.ones((S, 2), int)
    ptr = np.zeros((S, 2), int)
    timer = np.zeros((S, 2), int)
    delivered = np.zeros((S, 3), bool)
    t_deliver = np.full((S, 3), np.nan)
    aborted = np.zeros((S, 2), bool)
    deliv_p2 = np.zeros(S, int)
    # Phase 3
    surv = np.array([survivors_for(s) for s in seeds]) if not phase2_only else np.zeros((S, 2, 2))
    U = np.array([sensor_draws(s) for s in seeds]) if not phase2_only else None
    Braw = padded_priors(S) if not phase2_only else None
    Zb = np.ones((S, 2))
    focus = np.zeros(S, int)          # sequential policy: survivor searched first
    p3_switch = np.zeros(S, int)      # greedy target moved to the other survivor's prior
    side = np.full(S, -1)
    p3_path = np.zeros(S)
    found = np.zeros((S, 2), bool)
    how = np.zeros((S, 2), int)
    loc_est = np.full((S, 2, 2), np.nan)
    t_found = np.full((S, 2), np.nan)
    dst = np.zeros(S, int)            # 0 parked, 1 search, 2 rally, 3 formation, 4 failsafe
    k3_start = np.zeros(S, int)
    p3_status = np.zeros(S, int)      # 0 -, 1 both found, 2 battery abort, 3 deadline
    p3_pending = np.zeros(S, bool)
    p3_obs = np.zeros(S, int)
    p3_hits = np.zeros(S, int)
    p3_false = np.zeros(S, int)
    E_D_search = np.full(S, np.nan)
    # Phase 4
    assign = np.full((S, 5), -1)
    have = np.zeros((S, 5), bool)
    pend = np.zeros((S, 5), bool)
    integ = np.zeros((S, 5, 3))
    e_prev = np.zeros((S, 5, 3))
    goal = np.zeros((S, 5, 3))
    failsafe = np.zeros((S, 5), bool)
    land = np.zeros((S, 5), int)      # 0 -, 1 braking to a hold, 2 descending to the ground
    t_fs = np.full((S, 5), np.nan)
    z_over = np.zeros(S)
    p4_start_pos = np.full((S, 5, 3), np.nan)
    p4_start_vel = np.full((S, 5, 3), np.nan)
    p4_path = np.zeros((S, 5))
    orca_steps = np.zeros(S, int)
    # bookkeeping
    lk = pos.copy()
    minsep = np.full((S, 4), np.inf)
    disc_ticks = np.zeros((S, 5), int)
    p3_delay = np.zeros(S, int)
    v_peak = np.zeros((S, 5))
    a_peak = np.zeros((S, 5))
    iu = np.triu_indices(5, 1)
    if log:
        pos_log = np.zeros((S, K_TICKS + 1, 5, 3), np.float32)
        E_log = np.zeros((S, K_TICKS + 1, 5), np.float32)
        ph_log = np.zeros((S, K_TICKS + 1), np.int8)
        H_log = np.full((S, K_TICKS + 1), np.nan, np.float32)
        Dtgt_log = np.full((S, K_TICKS + 1, 2), np.nan, np.float32)
        pos_log[:, 0], E_log[:, 0], ph_log[:, 0] = pos, E, phase
    k_last = np.full(S, K_TICKS)
    for k in range(1, K_TICKS + 1):
        t = k * DT
        act = phase < 5
        if not act.any():
            break
        # ── motion ────────────────────────────────────────────────────────────
        p_g, v_g = goto_step(pos, vel, target, vmax)
        new_p, new_v = pos.copy(), vel.copy()
        mg = act[:, None] & air & (mode == GOTO)
        new_p[mg], new_v[mg] = p_g[mg], v_g[mg]
        mp = act[:, None] & (mode == PID)
        if mp.any():
            p_p, v_p, integ, e_prev, close = formation_step(pos, vel, goal, integ, e_prev, mp,
                                                            orca, anti_windup)
            orca_steps += np.any(close & mp[:, :, None], axis=(1, 2))
            new_v[mp] = v_p[mp]
            new_p[mp] = p_p[mp]
            p4_path += np.where(mp, _norm(v_p) * DT, 0.0)
            z_over = np.maximum(z_over, np.where(mp, p_p[..., 2] - Z_SHOW, -np.inf).max(axis=1))
        on_track = act & (k <= k_track) & (not phase2_only)
        if on_track.any():
            new_p[on_track, 0] = p1['track'][on_track, k]
            new_v[on_track, 0] = (p1['track'][on_track, k] - p1['track'][on_track, k - 1]) / DT
        # invariants (X4)
        moved = act[:, None] & air & (mode != TRACK)
        spd = _norm(new_v)
        acc = _norm(new_v - vel) / DT
        assert np.all(spd[moved] <= V_PHYS + 1e-9), 'speed limit'
        assert np.all(acc[moved] <= A_FLEET * (1 + 1e-6)), 'acceleration limit'
        assert np.all(new_p[..., 2] >= -1e-9) and np.all(new_p[..., 2] <= ARENA[2] + 1e-9)
        assert np.all(new_p[..., :2] >= -1e-9) and np.all(new_p[..., :2] <= ARENA[0] + 1e-9)
        v_peak = np.maximum(v_peak, np.where(act[:, None] & air, spd, 0.0))
        p3_path += np.where(act & (dst == 1), spd[:, 3] * DT, 0.0)
        a_peak = np.maximum(a_peak, np.where(moved, acc, 0.0))
        # battery
        rate = np.maximum(chov, cfly * spd)
        drain = act[:, None] & air & (mode != TRACK)
        E = E - np.where(drain, rate * DT, 0.0)
        if on_track.any():
            E[on_track, 0] = 1.0 - p1['e_used'][on_track, k] / kappa
        assert np.all(E >= -1e-12), 'flying on an empty battery'
        pos, vel = new_p, new_v
        # Phase 1 handover: the catcher has stopped (2 ms brake); it lands below its hold
        hand = act & (k == k_track) & (not phase2_only)
        if hand.any():
            vel[hand, 0] = 0.0
            mode[hand, 0] = GOTO
            target[hand, 0] = pos[hand, 0] * [1.0, 1.0, 0.0]
            vmax[hand, 0] = V1
            land[hand, 0] = 2
        # ── Phase 3 sensing ───────────────────────────────────────────────────
        if not phase2_only:
            sens = act & (dst == 1) & (np.abs(pos[:, 3, 2] - SCOUT_H) <= SENSOR_ALT_TOL)
            if sens.any():
                ix = np.where(sens)[0]
                dxy = pos[ix, 3, :2]
                in_true = _norm(dxy[:, None, :] - surv[ix]) <= SENSOR_R
                j = k - k3_start[ix] - 1
                u_ = U[ix, j]
                hit = u_ < np.where(in_true, P_DET, P_FA)
                r = np.where(hit, P_DET / P_FA, (1 - P_DET) / (1 - P_FA))
                r = np.where(found[ix], 1.0, r)
                p3_observe(Braw, Zb, ix, dxy, r)
                p3_obs[ix] += 1
                p3_hits[ix] += np.sum(hit & ~found[ix], axis=1)
                p3_false[ix] += np.sum(hit & ~in_true & ~found[ix], axis=1)
                new_hit = hit & in_true & ~found[ix]
                # patch rule: a 3 x 3 patch >= 0.6 needs some cell >= 0.6 / 9
                V = Braw[:, :, PADC:PADC + GRID_N, PADC:PADC + GRID_N]
                cmax = np.max(V, axis=(2, 3))[ix] / Zb[ix]
                cand = (cmax >= DET_THRESH / 9) & ~found[ix] & ~new_hit
                for jj, s in enumerate(ix):
                    for v_ in range(2):
                        if new_hit[jj, v_]:
                            found[s, v_], how[s, v_], t_found[s, v_] = True, 1, t
                            loc_est[s, v_] = dxy[jj]
                        elif cand[jj, v_]:
                            bx = box3(V[s, v_] / Zb[s, v_])
                            if bx.max() >= DET_THRESH:
                                found[s, v_], how[s, v_], t_found[s, v_] = True, 2, t
                                a, b2 = divmod(int(bx.argmax()), GRID_N - 2)
                                loc_est[s, v_] = [XC[a + 1], XC[b2 + 1]]
                if k % 50 == 0:                   # exact renormalisation (no drift)
                    for s in ix:
                        Braw[s] /= Zb[s][:, None, None]
                    Zb[ix] = 1.0
                if log and k % 10 == 0:
                    comb = (_inner(Braw, ix) / Zb[ix][:, :, None, None]
                            * (~found[ix])[:, :, None, None]).sum(axis=1)
                    tot = comb.sum(axis=(1, 2))
                    pz = comb / np.maximum(tot, 1e-300)[:, None, None]
                    H = -xlogy(pz, pz).sum(axis=(1, 2))
                    H_log[ix, k] = np.where(tot > 0, H, 0.0)
        # ── role state machines ───────────────────────────────────────────────
        arrived = (_norm(target - pos) <= ARRIVE_TOL) & (_norm(vel) <= ARRIVE_TOL) & (mode == GOTO)
        # landing sequence (catcher after Phase 1, battery failsafe): hold, then descend
        m_ = act[:, None] & (land == 2) & arrived
        air[m_] = False
        mode[m_] = PARKED
        vel[m_] = 0.0
        land[m_] = 0
        m_ = act[:, None] & (land == 1) & arrived
        target[m_, 2] = 0.0
        land[m_] = 2
        # couriers
        for c in range(2):
            d = c + 1
            n_trips = len(plan[c])
            m1 = act & (cst[:, c] == 1)
            if m1.any():
                more = ptr[:, c] < n_trips
                w = np.array([plan[c][min(int(x), n_trips - 1)] for x in ptr[:, c]]) if n_trips else np.zeros(S, int)
                d_out = _norm(DELIVERY_WPS[w] - pos[:, d])
                need = cfly * 2 * d_out + chov * TAU_DELIVER
                go = m1 & more & (E[:, d] - need >= E_RESERVE)
                stop = m1 & ~go
                aborted[stop & more, c] = True
                cst[stop, c] = 6
                cst[go, c] = 2
                air[go, d] = True
                mode[go, d] = GOTO
                target[go, d] = DELIVERY_WPS[w[go]]
                vmax[go, d] = V_COURIER
            m2 = act & (cst[:, c] == 2) & arrived[:, d]
            cst[m2, c] = 3
            timer[m2, c] = int(round(TAU_DELIVER / DT))
            m3 = act & (cst[:, c] == 3)
            timer[m3, c] -= 1
            m3d = m3 & (timer[:, c] <= 0)
            if m3d.any():
                for s in np.where(m3d)[0]:
                    w = plan[c][ptr[s, c]]
                    delivered[s, w] = True
                    t_deliver[s, w] = t
                ptr[m3d, c] += 1
                cst[m3d, c] = 4
                target[m3d, d] = PADS[c]
                arrived[m3d, d] = False          # new target: arrival must be re-detected
            m4 = act & (cst[:, c] == 4) & arrived[:, d]
            if m4.any():
                air[m4, d] = False
                mode[m4, d] = PARKED
                vel[m4, d] = 0.0
                again = m4 & (ptr[:, c] < n_trips) & (phase == 2)
                cst[again, c] = 5
                timer[again, c] = int(round(T_LOAD / DT))
                cst[m4 & ~again, c] = 6
            m5 = act & (cst[:, c] == 5)
            timer[m5, c] -= 1
            cst[m5 & (timer[:, c] <= 0), c] = 1
        # scout
        if not phase2_only:
            ms = act & (dst == 1)
            if ms.any():
                ix = np.where(ms)[0]
                done_both = found[ix].all(axis=1)
                d_far = _norm(pos[ix, 3][:, None, :] - STAR[None]).max(axis=1)
                # card trigger with "home" = the farthest star vertex, then down to the ground
                low = E[ix, 3] - cfly * (d_far + Z_SHOW) < E_RESERVE
                late = np.full(len(ix), t >= T_DEADLINE[2])
                stop = done_both | low | late
                st = ix[stop]
                p3_status[st] = np.where(done_both[stop], 1, np.where(low[stop], 2, 3))
                p3_pending[st] = True
                dst[st] = 2
                E_D_search[st] = E[st, 3]
                target[st, 3] = STAR_CENTRE
                go = ix[~stop]
                if len(go):
                    alpha = (~found).astype(float)
                    if policy == 'sequential':      # only the focus survivor until found
                        one = alpha[go, focus[go]] > 0
                        a2 = alpha[go].copy()
                        a2[one, 1 - focus[go][one]] = 0.0
                        alpha[go] = a2
                    txy = p3_target(Braw, Zb, alpha, go, pos[go, 3, :2])
                    target[go, 3, :2] = txy
                    target[go, 3, 2] = SCOUT_H
                    sd = np.argmin(_norm(txy[:, None, :] - LKP[None]), axis=1)
                    both = ~found[go].any(axis=1)
                    p3_switch[go] += both & (side[go] >= 0) & (sd != side[go])
                    side[go] = sd
                    if log:
                        Dtgt_log[go, k] = txy
        # battery failsafe: land vertically while a landing is still affordable
        fs = (act[:, None] & air & ~failsafe & (mode != TRACK) & failsafe_on
              & (E < cfly * pos[..., 2] + FAILSAFE_MARGIN))
        if fs.any():
            failsafe |= fs
            t_fs[fs] = t
            mode[fs] = GOTO
            target[fs] = pos[fs]
            vmax[fs] = 2.0
            land[fs] = 1
            have[fs] = False
            for s, d in zip(*np.where(fs)):
                if d in (1, 2):
                    cst[s, d - 1] = 6
                if d == 3 and dst[s] == 1:
                    p3_status[s], p3_pending[s] = 2, True
                if d == 3:
                    dst[s] = 4
        # ── communication ─────────────────────────────────────────────────────
        conn, dmat = connectivity(pos)
        lk = np.where(conn[..., None], pos, lk)
        disc_ticks += (act[:, None] & ~conn)
        pair_air = (air[:, :, None] | air[:, None, :])[:, iu[0], iu[1]]
        dpair = np.where(pair_air, dmat[:, iu[0], iu[1]], np.inf).min(axis=1)
        for p_ in range(4):
            mm = act & (phase == p_ + 1)
            minsep[mm, p_] = np.minimum(minsep[mm, p_], dpair[mm])
        # ── sequencer ─────────────────────────────────────────────────────────
        # Phase 1 -> 2
        if not phase2_only:
            m = act & (phase == 1) & (k == k1_end)
            phase[m] = 2
            t_end[m, 0] = t
            cst[m] = 1
        # Phase 2 -> 3
        m = act & (phase == 2)
        done2 = m & np.all(cst == 6, axis=1) & conn[:, 1] & conn[:, 2]
        dl2 = m & ~done2 & (t >= T_DEADLINE[1])
        for mm in (done2, dl2):
            if mm.any():
                deliv_p2[mm] = delivered[mm].sum(axis=1)
                t_end[mm, 1] = t
                phase[mm] = 3 if not phase2_only else 5
        if dl2.any():
            for c in range(2):
                fly = dl2 & np.isin(cst[:, c], [2, 3])
                cst[fly, c] = 4
                target[fly, c + 1] = PADS[c]
                cst[dl2 & np.isin(cst[:, c], [1, 5]), c] = 6
        if phase2_only:
            k_last[(done2 | dl2)] = k
            if log:
                pos_log[:, k], E_log[:, k], ph_log[:, k] = pos, E, phase
            continue
        st3 = done2 | dl2
        if st3.any():
            dst[st3] = 1
            k3_start[st3] = k
            air[st3, 3] = True
            mode[st3, 3] = GOTO
            vmax[st3, 3] = V_SCOUT
            ix3 = np.where(st3)[0]
            focus[ix3] = np.argmin(_norm(pos[ix3, 3, None, :2] - LKP[None]), axis=1)
            alpha = np.ones((S, 2))
            if policy == 'sequential':
                alpha[ix3, 1 - focus[ix3]] = 0.0
            target[ix3, 3, :2] = p3_target(Braw, Zb, alpha, ix3, pos[ix3, 3, :2])
            target[ix3, 3, 2] = SCOUT_H
        # Phase 3 -> 4
        m = act & (phase == 3)
        ev = m & p3_pending & conn[:, 3]
        dl3 = m & ~ev & (t >= T_DEADLINE[2])
        p3_delay += (m & p3_pending & ~conn[:, 3])
        st4 = ev | dl3
        if st4.any():
            t_end[st4, 2] = t
            phase[st4] = 4
            for s in np.where(st4)[0]:
                if assignment == 'hungarian':
                    cost = _norm(lk[s][:, None, :] - STAR[None])
                    r_, c_ = linear_sum_assignment(cost)
                    assign[s, r_] = c_
                else:
                    assign[s] = np.arange(5)
            goal[st4] = STAR[assign[st4]]
            pend[st4] = ~failsafe[st4]
        # assignment broadcast reaches connected drones
        rec = act[:, None] & pend & conn
        if rec.any():
            pend &= ~rec
            have |= rec
            mode[rec] = PID
            air[rec] = True
            land[rec] = 0
            integ[rec] = 0.0
            e_prev[rec] = goal[rec] - pos[rec]
            p4_start_pos[rec] = pos[rec]
            p4_start_vel[rec] = vel[rec]
            dst[rec[:, 3]] = 3
        # Phase 4 -> done
        m = act & (phase == 4)
        err4 = _norm(goal - pos).max(axis=1)
        conv = m & have.all(axis=1) & (err4 <= EPS_FORM)
        dl4 = m & ~conv & (t >= T_HARD - 1e-9)
        for mm in (conv, dl4):
            t_end[mm, 3] = t
            phase[mm] = 5
            k_last[mm] = k
        if log:
            pos_log[:, k], E_log[:, k], ph_log[:, k] = pos, E, phase
    assert np.all(phase == 5), 'mission loop did not terminate'
    t4 = t_end[:, 3]
    s1 = (p1['success'] & (t_end[:, 0] <= T_DEADLINE[0] + 1e-9)) if not phase2_only else np.zeros(S, bool)
    sc2 = np.where(t_end[:, 1] <= T_DEADLINE[1] + 1e-9, deliv_p2, 0)
    s3 = found.all(axis=1) & (p3_status == 1) & (t_end[:, 2] <= T_DEADLINE[2] + 1e-9)
    s4 = (t4 <= T_DEADLINE[3] + 1e-9) & ~np.isnan(t4)
    conv_ok = np.zeros(S, bool)
    if not phase2_only:
        conv_ok = (_norm(goal - pos).max(axis=1) <= EPS_FORM)
        s4 = s4 & conv_ok
    score = (WEIGHTS[0] * s1 + sc2 * (WEIGHTS[1] / 3.0) + WEIGHTS[2] * s3 + WEIGHTS[3] * s4
             - OT_PENALTY * np.maximum(0.0, np.nan_to_num(t4, nan=T_HARD) - T_TOTAL))
    out = dict(seeds=np.array(seeds), t_end=t_end, s1=s1, sc2=sc2, s3=s3, s4=s4, conv_ok=conv_ok,
               score=score, delivered=delivered, t_deliver=t_deliver, aborted=aborted,
               deliv_p2=deliv_p2, found=found, how=how, loc_est=loc_est, t_found=t_found,
               surv=surv, p3_status=p3_status, p3_obs=p3_obs, p3_hits=p3_hits,
               p3_false=p3_false, E_D_search=E_D_search, assign=assign, goal=goal,
               failsafe=failsafe, t_fs=t_fs, z_over=z_over, E_final=E, minsep=minsep, disc_ticks=disc_ticks,
               p3_delay=p3_delay, orca_steps=orca_steps, p4_path=p4_path,
               p4_start=p4_start_pos, p4_vel=p4_start_vel, v_peak=v_peak, a_peak=a_peak, k_last=k_last,
               k3_start=k3_start, kappa=kappa, p3_switch=p3_switch, p3_path=p3_path,
               belief=(unpad(Braw, Zb) if Braw is not None else None))
    if log:
        out.update(pos_log=pos_log, E_log=E_log, ph_log=ph_log, H_log=H_log, Dtgt_log=Dtgt_log)
    return out


# ═════════════════════════════════════════════════════════════════════════════
# Experiment
# ═════════════════════════════════════════════════════════════════════════════
def run_simulation():
    seeds = list(range(N_SEEDS))
    R = dict(seeds=np.array(seeds))
    R['balls'], R['p1'] = {}, {}
    for st in P1_STRATS:
        R['balls'][st] = run_phase1(st, seeds)
        R['p1'][st] = assemble_phase1(R['balls'][st])
    # Phase 1 sweeps: exact re-timing of the same simulated throws
    R['nb_sweep'] = {st: [p1_timeline(R['balls'][st], n, T_RELOAD)[1]
                          for n in range(1, N_BALLS_SIM + 1)] for st in P1_STRATS}
    R['reload_sweep'] = {st: [p1_timeline(R['balls'][st], N_BALLS, tr)[0]
                              for tr in RELOAD_SWEEP] for st in P1_STRATS}
    # Phase 2 on its own (deterministic): card heuristic vs exhaustive split, c_fly sweep
    plans = {'nn': nn_plan(), 'exh': exhaustive_plan()}
    R['plans'] = plans
    R['p2'] = {nm: run_mission(None, [0], plan=pl, phase2_only=True, log=True)
               for nm, pl in plans.items()}
    R['p2_cfly'] = {nm: [run_mission(None, [0], plan=pl, c_fly=c, phase2_only=True)
                         for c in C_FLY_SWEEP] for nm, pl in plans.items()}
    # full missions, all on the same seeds (common random numbers)
    pc, pn = R['p1']['pcp'], R['p1']['pn']
    M = {}
    M['k1'] = run_mission(pc, seeds, kappa=1.0, log=True)
    M['pn1'] = run_mission(pn, seeds, kappa=1.0)
    M['exh1'] = run_mission(pc, seeds, kappa=1.0, plan=plans['exh'])
    M['nofs1'] = run_mission(pc, seeds, kappa=1.0, failsafe_on=False)
    for kap in KAPPAS[1:]:
        M[f'k{kap:g}'] = run_mission(pc, seeds, kappa=kap, log=(kap == KAPPAS[-1]))
    M['pn5'] = run_mission(pn, seeds, kappa=KAPPAS[-1])
    M['seq5'] = run_mission(pc, seeds, kappa=KAPPAS[-1], policy='sequential')
    R['M'] = M
    # Phase 4 ablations from the kappa = 5 Phase 4 start states
    b = M['k5']
    n_max = np.round((T_HARD - b['t_end'][:, 2]) / DT).astype(int)
    goal_h = b['goal']
    goal_i = np.repeat(STAR[None], len(seeds), axis=0)
    p0, v0 = b['p4_start'], b['p4_vel']
    assert np.all(np.isfinite(p0)) and np.all(np.isfinite(v0))
    R['F'] = {
        'main': run_formation(p0, v0, goal_h, n_max),
        'noorca': run_formation(p0, v0, goal_h, n_max, orca=False),
        'ident': run_formation(p0, v0, goal_i, n_max),
        'ident_noorca': run_formation(p0, v0, goal_i, n_max, orca=False),
        'noaw': run_formation(p0, v0, goal_h, n_max, anti_windup=False),
        'ki0': run_formation(p0, v0, goal_h, n_max, ki=0.0),
    }
    R['F_nmax'] = n_max
    R['rep'] = pick_representative(R)
    return R


def pick_representative(R):
    """First seed with a perfect score at kappa = 5 whose kappa = 1 score is the most
    common kappa = 1 score."""
    s1 = np.round(R['M']['k1']['score'], 6)
    vals, cnt = np.unique(s1, return_counts=True)
    mode = vals[np.argmax(cnt)]
    ok = np.where((np.abs(R['M']['k5']['score'] - 10.0) < 1e-9) & (s1 == mode))[0]
    return int(ok[0]) if len(ok) else 0


# ═════════════════════════════════════════════════════════════════════════════
# Report
# ═════════════════════════════════════════════════════════════════════════════
def _ms(x):
    x = np.asarray(x, float)
    return f'{x.mean():.2f} +- {x.std(ddof=1):.2f}'


def _rate(b):
    b = np.asarray(b, bool)
    lo, hi = m94.wilson(b.sum(), len(b))
    return f'{b.sum():3d}/{len(b)} = {100 * b.mean():5.1f}% [{100 * lo:4.1f}, {100 * hi:5.1f}]'


def _paired(a, b, name_a, name_b):
    w, l = int(np.sum(a > b + 1e-9)), int(np.sum(b > a + 1e-9))
    p = binomtest(w, w + l, 0.5).pvalue if w + l else 1.0
    return (f'{name_a} vs {name_b}: {name_a} better {w}, ties {len(a) - w - l}, '
            f'{name_b} better {l} (sign test p = {p:.2e})')


def mission_line(name, r):
    S = len(r['score'])
    perfect = np.abs(r['score'] - 10.0) < 1e-9
    within = r['t_end'][:, 3] <= T_TOTAL + 1e-9
    return (f'  {name:<32} P1 {100 * r["s1"].mean():5.1f}%  P2 3/3 {100 * np.mean(r["sc2"] == 3):5.1f}%  '
            f'P3 {100 * r["s3"].mean():5.1f}%  P4 {100 * r["s4"].mean():5.1f}%  '
            f'end<=300s {within.sum():3d}/{S}  perfect {perfect.sum():3d}/{S}  '
            f'score {_ms(r["score"])}')


MISSION_LABELS = [('k1', 'PCP, battery x1 (card)'), ('pn1', 'PN, battery x1'),
                  ('exh1', 'PCP, x1, exhaustive Phase 2'), ('nofs1', 'PCP, x1, no landing reserve')]
MISSION_LABELS += [(f'k{k:g}', f'PCP, battery x{k:g}') for k in KAPPAS[1:]]
MISSION_LABELS += [('pn5', 'PN, battery x5'), ('seq5', 'PCP, x5, sequential SAR')]
F_LABELS = [('main', 'Hungarian + ORCA + anti-windup'), ('noorca', 'no ORCA'),
            ('ident', 'identity assignment'), ('ident_noorca', 'identity, no ORCA'),
            ('noaw', 'card PID, no anti-windup'), ('ki0', 'PD only (K_i = 0)')]


def report(R):
    S = len(R['seeds'])
    M = R['M']
    print('=' * 100)
    print('S100 Ultimate Grand Challenge — Multi-Domain Integration')
    print('=' * 100)
    print(f'Missions: {S} seeds (common random numbers), mission tick {DT} s, Phase 1 sub-step '
          f'{DT1} s, horizon {T_HARD:.0f} s')
    print(f'Battery model: rate = max(c_hover, c_fly*|v|) while airborne, c_fly = {C_FLY}/m, '
          f'c_hover = {C_HOVER}/s (range {1 / C_FLY:.0f} m at cruise, hover endurance '
          f'{1 / C_HOVER:.0f} s); capacity multiplier kappa divides both')

    # ── Phase 1 ──
    print('\n--- Phase 1: aerial ball catching (S094 model, catch-zone centre '
          f'({CATCH_CENTRE[0]:.0f}, {CATCH_CENTRE[1]:.0f}) m) ---')
    for st in P1_STRATS:
        c = np.array([b['caught'] for b in R['balls'][st]])
        print(f'  {P1_LABEL[st]:<24} per-throw catch rate, {N_BALLS_SIM} x {S} throws: '
              f'{_rate(c.ravel())}; by throw index: '
              + ', '.join(f'#{i + 1} {100 * c[i].mean():.1f}%' for i in range(N_BALLS_SIM)))
        tab = np.stack([c.sum(axis=1), (~c).sum(axis=1)], axis=1)
        print(f'    same rate for every throw index? chi-square p = {chi2_contingency(tab)[1]:.3f}')
    ca = np.array([b['caught'] for b in R['balls']['pcp']]).ravel()
    cb = np.array([b['caught'] for b in R['balls']['pn']]).ravel()
    w, l = int(np.sum(ca & ~cb)), int(np.sum(~ca & cb))
    print(f'  Paired throws: PCP only {w}, both or neither {len(ca) - w - l}, PN only {l} '
          f'(McNemar p = {binomtest(w, w + l, 0.5).pvalue:.2e})')
    for st in P1_STRATS:
        p1 = R['p1'][st]
        p = np.array([b['caught'] for b in R['balls'][st]])[:N_BALLS].mean()
        print(f'  {P1_LABEL[st]:<24} Phase 1 success (N_balls = {N_BALLS}): {_rate(p1["success"])}; '
              f'1-(1-p)^{N_BALLS} with p = {p:.3f} (throws 1-{N_BALLS}): '
              f'{100 * (1 - (1 - p)**N_BALLS):.1f}%')
        fb = p1['first'][p1['success']]
        print(f'    first catch on ball 1/2/3: {np.sum(fb == 0)}/{np.sum(fb == 1)}/{np.sum(fb == 2)}; '
              f'Phase 1 end {_ms(p1["k1_end"] * DT)} s (min {p1["k1_end"].min() * DT:.1f}, '
              f'max {p1["k1_end"].max() * DT:.1f}); catcher path {_ms(p1["path_len"])} m; '
              f'catcher battery used {_ms(100 * p1["e_used"][np.arange(S), p1["k_track"]])} %')
        print('    success vs N_balls: ' + ', '.join(
            f'{n + 1}: {100 * R["nb_sweep"][st][n].mean():.1f}%' for n in range(N_BALLS_SIM)))
        print('    T_reload sweep (N_balls = 3): ' + '; '.join(
            f'{tr:g} s: max end {R["reload_sweep"][st][i].max() * DT:.1f} s, '
            f'> 60 s in {int(np.sum(R["reload_sweep"][st][i] * DT > T_DEADLINE[0]))}'
            for i, tr in enumerate(RELOAD_SWEEP)))

    # ── Phase 2 ──
    print('\n--- Phase 2: logistics (1 package per trip, round trips from each courier pad) ---')
    for nm, lab in [('nn', 'Card NN heuristic'), ('exh', 'Exhaustive min-makespan')]:
        pl = R['plans'][nm]
        mk, tot = plan_makespan(pl)
        r = R['p2'][nm]
        k = int(r['k_last'][0])
        P = r['pos_log'][0, :k + 1, 1:3].astype(float)
        plen = np.sum(_norm(np.diff(P, axis=0)), axis=0)
        print(f'  {lab:<24} B -> {"+".join(f"WP{w + 1}" for w in pl[0])}, '
              f'C -> {"+".join(f"WP{w + 1}" for w in pl[1])}; planned makespan {mk:.1f} s, '
              f'planned distance {tot:.1f} m')
        print(f'    simulated: duration {r["t_end"][0, 1]:.1f} s, deliveries {r["deliv_p2"][0]}/3, '
              f'flown B {plen[0]:.1f} m / C {plen[1]:.1f} m, battery left B '
              f'{100 * r["E_final"][0, 1]:.1f}% / C {100 * r["E_final"][0, 2]:.1f}%')
    rn_, re_ = R['p2']['nn'], R['p2']['exh']
    print(f'  Exhaustive minus NN: planned distance {plan_makespan(R["plans"]["exh"])[1] - plan_makespan(R["plans"]["nn"])[1]:+.1f} m, '
          f'duration {re_["t_end"][0, 1] - rn_["t_end"][0, 1]:+.1f} s, battery left B '
          f'{100 * (re_["E_final"][0, 1] - rn_["E_final"][0, 1]):+.1f} percentage points')
    print('  c_fly sweep (Phase 2 alone): c_fly -> deliveries, aborted trips B/C, battery left B/C')
    for nm in ['nn', 'exh']:
        print(f'    {nm:>3}: ' + '; '.join(
            f'{c}: {r["deliv_p2"][0]}/3, {int(r["aborted"][0, 0])}/{int(r["aborted"][0, 1])}, '
            f'{100 * r["E_final"][0, 1]:.1f}%/{100 * r["E_final"][0, 2]:.1f}%'
            for c, r in zip(C_FLY_SWEEP, R['p2_cfly'][nm])))

    # ── Missions ──
    print('\n--- Full missions: per-phase success, 300 s budget, score (mean +- std) ---')
    for key, lab in MISSION_LABELS:
        print(mission_line(lab, M[key]))
    print('  Paired mission scores (same seeds):')
    for a, b_, na, nb in [('k1', 'pn1', 'PCP', 'PN'), ('k5', 'pn5', 'PCP(x5)', 'PN(x5)'),
                          ('exh1', 'k1', 'exhaustive', 'NN'), ('k5', 'k1', 'x5', 'x1'),
                          ('seq5', 'k5', 'sequential', 'greedy')]:
        print('    ' + _paired(M[a]['score'], M[b_]['score'], na, nb))
    for key in ['k1', 'pn1', 'k5', 'pn5']:
        vals, cnt = np.unique(np.round(M[key]['score'], 3), return_counts=True)
        print(f'  Score distribution {key}: ' + ', '.join(f'{v:g} x{c}' for v, c in zip(vals, cnt)))
    for key in ['k1', 'k5', 'seq5', 'pn5']:
        r = M[key]
        te = r['t_end']
        print(f'\n  [{key}] phase end times mean / min / max (s): ' + '; '.join(
            f'P{p + 1} {np.mean(te[:, p]):.1f} / {np.min(te[:, p]):.1f} / {np.max(te[:, p]):.1f}'
            for p in range(4)))
        dur = np.diff(np.concatenate([np.zeros((S, 1)), te], axis=1), axis=1)
        print(f'  [{key}] phase durations mean / max (s): ' + '; '.join(
            f'P{p + 1} {dur[:, p].mean():.1f} / {dur[:, p].max():.1f}' for p in range(4))
            + '; every phase within its nominal window (60/100/80/60 s): '
            f'{int(np.sum(np.all(dur <= np.array([60, 100, 80, 60]) + 1e-9, axis=1)))}/{S}')
        st_ = r['p3_status']
        print(f'  [{key}] Phase 3 ends: both found {np.sum(st_ == 1)}, battery abort {np.sum(st_ == 2)}, '
              f'deadline {np.sum(st_ == 3)}; survivors found 0/1/2: '
              + '/'.join(str(int(np.sum(r['found'].sum(1) == n))) for n in range(3)))
        hw = r['how'][r['found']]
        err = _norm(r['loc_est'] - r['surv'])[r['found']]
        print(f'  [{key}] located by confirmed hit {np.sum(hw == 1)}, by 3x3 patch >= {DET_THRESH} '
              f'{np.sum(hw == 2)}; location error mean {err.mean():.2f} m, max {err.max():.2f} m, '
              f'> r_s: {int(np.sum(err > SENSOR_R))}')
        print(f'  [{key}] scout: search path {_ms(r["p3_path"])} m, observations {_ms(r["p3_obs"])}, '
              f'false alarms {_ms(r["p3_false"])}, target switches between the two priors while '
              f'both unfound: median {np.median(r["p3_switch"]):.0f}, max {r["p3_switch"].max()}; '
              f'battery at end of search {_ms(100 * r["E_D_search"])} %')
        cv = r['conv_ok']
        print(f'  [{key}] Phase 4 converged {int(cv.sum())}/{S}'
              + (f', duration {_ms((te[:, 3] - te[:, 2])[cv])} s' if cv.any() else '')
              + f'; failsafe landings A-E {r["failsafe"].sum(0).tolist()}; '
              f'ORCA active in {int(np.sum(r["orca_steps"] > 0))} missions')
        print(f'  [{key}] min separation (pairs with one drone airborne) per phase: '
              + ', '.join(f'P{p + 1} {r["minsep"][:, p].min():.2f} m' for p in range(4)))
        print(f'  [{key}] final battery min A-E: ' + ', '.join(f'{100 * x:.1f}%' for x in r['E_final'].min(0))
              + f'; peak speed {r["v_peak"].max():.2f} m/s; peak accel (10 Hz loop) '
              f'{r["a_peak"].max():.2f} m/s^2')
        print(f'  [{key}] comm: missions with a drone out of range of E: '
              + ', '.join(f'{NAMES[d]} {int(np.sum(r["disc_ticks"][:, d] > 0))}' for d in range(5))
              + f'; Phase 3 report delayed in {int(np.sum(r["p3_delay"] > 0))} missions, max '
              f'{r["p3_delay"].max() * DT:.1f} s')
    r1, rn = M['k1'], M['nofs1']
    need = C_FLY * Z_SHOW
    print(f'\n  Landing-reserve probe (x1): Phase 4 converged with failsafe {int(r1["conv_ok"].sum())}/{S}, '
          f'without {int(rn["conv_ok"].sum())}/{S}; without it, final battery of B min '
          f'{100 * rn["E_final"][:, 1].min():.1f}% mean {100 * rn["E_final"][:, 1].mean():.1f}% vs '
          f'{100 * need:.1f}% needed to land from {Z_SHOW:.0f} m; drones unable to land afterwards '
          f'{int(np.sum(rn["E_final"] < need))} in {int(np.sum(np.any(rn["E_final"] < need, axis=1)))} missions')
    first = np.argmin(np.where(np.isnan(r1['t_fs']), np.inf, r1['t_fs']), axis=1)
    t_first = np.nanmin(r1['t_fs'], axis=1)
    print(f'  x1 first battery failsafe: ' + ', '.join(
        f'{NAMES[d]} {int(np.sum(first == d))}' for d in range(5))
        + f' (at {_ms(t_first)} s, {_ms(t_first - r1["t_end"][:, 2])} s after Phase 4 starts)'
        + f'; courier B battery after Phase 2: NN {100 * R["p2"]["nn"]["E_final"][0, 1]:.1f}%, '
        f'exhaustive {100 * R["p2"]["exh"]["E_final"][0, 1]:.1f}%; x1 Phase 4 converged NN '
        f'{int(r1["conv_ok"].sum())}, exhaustive {int(M["exh1"]["conv_ok"].sum())}')
    print(f'  Card numbers: an 80 s search at {V_SCOUT:.0f} m/s costs {C_FLY * V_SCOUT * 80:.2f} of a battery; '
          f'arena diagonal {np.hypot(ARENA[0], ARENA[1]):.0f} m vs r_comm {R_COMM:.0f} m')
    print('  Battery capacity sweep (PCP): ' + '; '.join(
        f'x{k:g}: P3 {100 * M[f"k{k:g}"]["s3"].mean():.1f}%, P4 {100 * M[f"k{k:g}"]["s4"].mean():.1f}%, '
        f'perfect {int(np.sum(np.abs(M[f"k{k:g}"]["score"] - 10) < 1e-9))}, '
        f'score {M[f"k{k:g}"]["score"].mean():.2f}' for k in KAPPAS))

    # ── Phase 4 ablations ──
    F = R['F']
    print('\n--- Phase 4 ablations (standalone formation from the x5 Phase 4 start states) ---')
    r5 = M['k5']
    t_mis = r5['t_end'][:, 3] - r5['t_end'][:, 2]
    for key, lab in F_LABELS:
        f = F[key]
        ok = f['k_conv'] > 0
        tc = f['k_conv'][ok] * DT
        print(f'  {lab:<32} converged {ok.sum():3d}/{S}, time {tc.mean():.1f} +- {tc.std(ddof=1):.1f} s '
              f'(max {tc.max():.1f}), min separation {f["minsep"].min():.2f} m '
              f'(missions < r_safe: {int(np.sum(f["minsep"] < R_SAFE))}), ORCA ticks '
              f'{int(f["orca_ticks"].sum())}, path {f["path"].mean():.1f} m, max height above the '
              f'show plane {f["z_over"].mean():.3f} m (mean of per-mission maxima)')
    a_, b_ = F['ident']['minsep'], F['ident_noorca']['minsep']
    close = b_ < R_SAFE
    print(f'  ORCA effect with identity assignment, {int(close.sum())} missions where a pair comes within '
          f'r_safe without ORCA: ORCA raises the min separation in {int(np.sum(a_[close] > b_[close] + 1e-9))}, '
          f'lowers it in {int(np.sum(a_[close] < b_[close] - 1e-9))}; mean {b_[close].mean():.2f} -> '
          f'{a_[close].mean():.2f} m; pairs closer than 0.5 m: {int(np.sum(b_ < 0.5))} -> {int(np.sum(a_ < 0.5))}')
    ok = F['main']['k_conv'] > 0
    dd = np.abs(F['main']['k_conv'][ok] * DT - t_mis[ok])
    print(f'  Standalone vs in-mission Phase 4 duration (x5): mean |difference| {dd.mean():.2f} s, '
          f'max {dd.max():.2f} s')
    rep = R['rep']
    st_name = {1: 'both found', 2: 'battery abort', 3: 'deadline'}
    print(f'\nRepresentative mission: seed {rep}: first ball caught at {R["p1"]["pcp"]["t_event"][rep]:.2f} s; '
          f'Phase 3 end x1 {st_name[int(M["k1"]["p3_status"][rep])]}, x5 {st_name[int(M["k5"]["p3_status"][rep])]}')
    print(f'Representative mission: seed {rep} (x1 score {M["k1"]["score"][rep]:.2f}, '
          f'x5 score {M["k5"]["score"][rep]:.2f}); x1 phase ends {np.round(M["k1"]["t_end"][rep], 1).tolist()} s, '
          f'x5 phase ends {np.round(M["k5"]["t_end"][rep], 1).tolist()} s')
    print('Invariants asserted every step: |v| <= limits, |a| <= 10 m/s^2, 0 <= z <= 30 m, inside the '
          'arena, battery >= 0; Phase 1 limits asserted inside the S094 code')


# ═════════════════════════════════════════════════════════════════════════════
# Plots
# ═════════════════════════════════════════════════════════════════════════════
PH_COL = ['#d62728', '#ff7f0e', '#8c564b', '#e6ab02']


def ball_ticks(R, st, s, n_ticks):
    """World ball position at each mission tick for mission s (NaN when no ball flies)."""
    out = np.full((n_ticks + 1, 3), np.nan)
    p1 = R['p1'][st]
    b_last = p1['first'][s] if p1['first'][s] >= 0 else N_BALLS - 1
    for b in range(b_last + 1):
        B = R['balls'][st][b]
        c = p1_cut_index(B, s) if b == b_last else int(B['n_end'][s])
        L = int(B['launch'][s])
        for k in range(-(-L // SUB), (L + c) // SUB + 1):
            if k <= n_ticks:
                out[k] = B['th']['truth'][s, k * SUB - L, :3] + CATCH_CENTRE
    return out


def plot_phase1(R, out_dir):
    s = R['rep']
    st = 'pcp'
    p1 = R['p1'][st]
    fig = plt.figure(figsize=(17, 5.2))
    ax = fig.add_subplot(131, projection='3d')
    tr = p1['tracks_2ms'][s][::10]
    ax.plot(tr[:, 0], tr[:, 1], tr[:, 2], color=COL['A'], lw=1.6, label='Catcher A (net path)')
    b_last = p1['first'][s] if p1['first'][s] >= 0 else N_BALLS - 1
    for b in range(b_last + 1):
        B = R['balls'][st][b]
        c = p1_cut_index(B, s) if b == b_last else int(B['n_end'][s])
        tb = B['th']['truth'][s, :c + 1, :3] + CATCH_CENTRE
        ax.plot(tb[:, 0], tb[:, 1], tb[:, 2], color=BALL_C, lw=1.4, label='Ball' if b == 0 else None)
        ax.scatter(*tb[0], color='k', marker='^', s=40, label='Launcher' if b == 0 else None)
        if B['caught'][s] and b == b_last:
            ax.scatter(*tb[-1], color=TARGET_C, marker='*', s=220, label=f'Catch at {p1["t_event"][s]:.2f} s')
    ax.scatter(*START[0], color=COL['A'], s=40, marker='s', label='A start (deck)')
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)'); ax.set_zlabel('z (m)')
    ax.set_title(f'Mission {s}: transit and first throw (PCP)')
    ax.legend(fontsize=7, loc='upper left')
    ax.view_init(elev=24, azim=-120)

    ax = fig.add_subplot(132)
    S = len(R['seeds'])
    rates, errs = [], []
    for st_ in P1_STRATS:
        c = np.array([b['caught'] for b in R['balls'][st_]]).ravel()
        lo, hi = m94.wilson(c.sum(), len(c))
        rates.append(100 * c.mean())
        errs.append([100 * (c.mean() - lo), 100 * (hi - c.mean())])
    x = np.arange(2)
    ax.bar(x, rates, yerr=np.array(errs).T, color=[COL['A'], '#ff9896'], capsize=6, edgecolor='k')
    ax.scatter(x, [93.5, 6.7], marker='_', s=900, color='k', zorder=5, label='S094 (5 seeds x 300 throws)')
    for xi, r in zip(x, rates):
        ax.text(xi, r + 4, f'{r:.1f}%', ha='center')
    ax.set_xticks(x)
    ax.set_xticklabels([P1_LABEL[s_] for s_ in P1_STRATS])
    ax.set_ylabel('Per-throw catch rate (%)')
    ax.set_ylim(0, 110)
    ax.set_title(f'Catch rate, {N_BALLS_SIM} x {S} common throws (95% Wilson CI)')
    ax.legend(fontsize=8, loc='center right')
    ax.grid(True, axis='y', alpha=0.3)

    ax = fig.add_subplot(133)
    n = np.arange(1, N_BALLS_SIM + 1)
    for st_, col in zip(P1_STRATS, [COL['A'], '#ff9896']):
        obs = [100 * v.mean() for v in R['nb_sweep'][st_]]
        p = np.array([b['caught'] for b in R['balls'][st_]]).mean()
        ax.plot(n, obs, 'o-', color=col, label=f'{P1_LABEL[st_]} (simulated)')
        ax.plot(n, 100 * (1 - (1 - p)**n), ':', color=col, label=f'1-(1-p)^n, p = {p:.3f}')
    ax.axvline(N_BALLS, color=GREY, ls='--', lw=1)
    ax.text(N_BALLS + 0.05, 50, f'card: {N_BALLS} balls', color=GREY)
    ax.set_xlabel('Balls the launcher may throw'); ax.set_ylabel('Phase 1 success (%)')
    ax.set_title('Phase 1 success vs number of balls')
    ax.set_ylim(0, 105); ax.legend(fontsize=7); ax.grid(True, alpha=0.3)
    fig.suptitle('S100 Phase 1 — Aerial Ball Catching', fontweight='bold')
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, 'phase1_catching.png'), dpi=110)
    plt.close(fig)


def plot_phase2(R, out_dir):
    fig, axes = plt.subplots(1, 3, figsize=(17, 5.2))
    ax = axes[0]
    for nm, ls in [('nn', '-'), ('exh', '--')]:
        r = R['p2'][nm]
        k = int(r['k_last'][0])
        for d, c in [(1, 'B'), (2, 'C')]:
            P = r['pos_log'][0, :k + 1, d]
            ax.plot(P[:, 0], P[:, 1], ls, color=COL[c], lw=1.6,
                    label=f'{c} ({"card NN" if nm == "nn" else "exhaustive"})')
    for i, w in enumerate(DELIVERY_WPS):
        ax.scatter(w[0], w[1], marker='D', s=110, color=TARGET_C, zorder=5)
        ax.annotate(f'WP{i + 1}', w[:2] + 1.5)
    for c, pad in zip('BC', PADS):
        ax.scatter(pad[0], pad[1], marker='s', s=70, color=COL[c], edgecolor='k', zorder=6)
    ax.set_xlim(10, 85); ax.set_ylim(30, 100); ax.set_aspect('equal')
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')
    ax.set_title('Courier routes (one package per trip)')
    ax.legend(fontsize=7, loc='lower right'); ax.grid(True, alpha=0.3)

    ax = axes[1]
    for nm, ls in [('nn', '-'), ('exh', '--')]:
        r = R['p2'][nm]
        k = int(r['k_last'][0])
        t = np.arange(k + 1) * DT
        for d, c in [(1, 'B'), (2, 'C')]:
            ax.plot(t, 100 * r['E_log'][0, :k + 1, d], ls, color=COL[c],
                    label=f'{c} ({"card NN" if nm == "nn" else "exhaustive"})')
    ax.axhline(100 * E_RESERVE, color='r', ls=':', label='Reserve 15%')
    ax.set_xlabel('Time since Phase 2 start (s)'); ax.set_ylabel('Battery (%)')
    ax.set_title('Courier battery, Phase 2 alone'); ax.legend(fontsize=7); ax.grid(True, alpha=0.3)

    ax = axes[2]
    for nm, mk, col in [('nn', 'o-', 'k'), ('exh', 's--', GREY)]:
        dl = [r['deliv_p2'][0] for r in R['p2_cfly'][nm]]
        ax.plot(C_FLY_SWEEP, dl, mk, color=col, label='card NN' if nm == 'nn' else 'exhaustive')
    ax.axvline(C_FLY, color='r', ls=':', lw=1)
    ax.text(C_FLY, 0.3, ' card c_fly', color='r')
    ax.set_xlabel('c_fly (battery fraction per metre)'); ax.set_ylabel('Packages delivered')
    ax.set_ylim(0, 3.3); ax.set_title('Battery trigger: deliveries vs c_fly')
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)
    fig.suptitle('S100 Phase 2 — Logistics (deterministic: same in every mission)', fontweight='bold')
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, 'phase2_logistics.png'), dpi=110)
    plt.close(fig)


def plot_phase3(R, out_dir):
    s = R['rep']
    r = R['M']['k5']
    fig, axes = plt.subplots(2, 2, figsize=(13, 11))
    k3, k4 = int(round(r['t_end'][s, 1] / DT)), int(round(r['t_end'][s, 2] / DT))
    path = r['pos_log'][s, k3:k4 + 1, 3].astype(float)
    for v in range(2):
        ax = axes[0, v]
        bm = r['belief'][s, v]
        im = ax.imshow(np.maximum(bm, 1e-9).T, origin='lower', extent=[0, SAR_HI, 0, SAR_HI],
                       cmap='hot_r', interpolation='nearest',
                       norm=LogNorm(vmin=1e-7, vmax=max(bm.max(), 1e-6)))
        fig.colorbar(im, ax=ax, fraction=0.046, label='Belief per cell (log scale)')
        ax.plot(path[:, 0], path[:, 1], color=COL['D'], lw=0.9, label='Scout D path')
        ax.scatter(*LKP[v], marker='x', s=90, color='c', lw=2, label='Last known position')
        ax.scatter(*r['surv'][s, v], marker='o', s=90, facecolor='none', edgecolor=TARGET_C, lw=2,
                   label='True survivor')
        if r['found'][s, v]:
            ax.scatter(*r['loc_est'][s, v], marker='+', s=120, color='b', lw=2,
                       label=f'Located at {r["t_found"][s, v]:.1f} s')
        ax.set_xlim(0, SAR_HI); ax.set_ylim(0, SAR_HI)
        ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')
        ax.set_title(f'Survivor {v + 1}: final belief (mission {s}, battery x5)')
        ax.legend(fontsize=7, loc='upper left')
    ax = axes[1, 0]
    for key, col, lab, ls, lw in [('k5', COL['D'], 'battery x5', '-', 1.5),
                                  ('k1', 'k', 'battery x1 (card): search aborted for battery', '--', 2.5)]:
        rr = R['M'][key]
        H = rr['H_log'][s].astype(float)
        k0 = int(round(rr['t_end'][s, 1] / DT))
        ok = np.isfinite(H)
        ax.plot(np.arange(len(H))[ok] * DT - k0 * DT, H[ok], ls, color=col, lw=lw, label=lab)
        for v in range(2):
            if rr['found'][s, v]:
                ax.axvline(rr['t_found'][s, v] - k0 * DT, color=col, ls=':', lw=1)
    ax.set_xlabel('Time since Phase 3 start (s)'); ax.set_ylabel('Entropy of unfound-survivor belief (nats)')
    ax.set_title(f'Mission {s}: search entropy (dotted: detections; drops to 0 when both found)')
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)
    ax = axes[1, 1]
    labs = [f'x{k:g}' for k in KAPPAS]
    st = np.array([[np.sum(R['M'][f'k{k:g}']['p3_status'] == c) for c in (1, 2, 3)] for k in KAPPAS])
    x = np.arange(len(KAPPAS))
    bot = np.zeros(len(KAPPAS))
    for c, lab, col in zip(range(3), ['Both found', 'Battery abort', 'Deadline 240 s'],
                           [TARGET_C, '#d62728', GREY]):
        ax.bar(x, st[:, c], bottom=bot, color=col, edgecolor='k', label=lab)
        bot += st[:, c]
    ax.set_xticks(x); ax.set_xticklabels(labs)
    ax.set_xlabel('Battery capacity multiplier'); ax.set_ylabel('Missions')
    ax.set_title('How Phase 3 ends (PCP missions, same seeds)')
    ax.legend(fontsize=8); ax.grid(True, axis='y', alpha=0.3)
    fig.suptitle('S100 Phase 3 — Bayesian Search and Rescue', fontweight='bold')
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, 'phase3_sar.png'), dpi=100)
    plt.close(fig)


def plot_phase4(R, out_dir):
    s = R['rep']
    r = R['M']['k5']
    k4, k5 = int(round(r['t_end'][s, 2] / DT)), int(round(r['t_end'][s, 3] / DT))
    P = r['pos_log'][s, k4:k5 + 1].astype(float)
    fig = plt.figure(figsize=(18, 5.4))
    ax = fig.add_subplot(131, projection='3d')
    for d, nm in enumerate(NAMES):
        ax.plot(P[:, d, 0], P[:, d, 1], P[:, d, 2], color=COL[nm], lw=1.5, label=f'Drone {nm}')
        ax.scatter(*P[0, d], color=COL[nm], s=30)
    ax.scatter(STAR[:, 0], STAR[:, 1], STAR[:, 2], marker='*', s=200, color=TARGET_C, label='Star vertices')
    for d in range(5):
        g = r['goal'][s, d]
        ax.plot([P[0, d, 0], g[0]], [P[0, d, 1], g[1]], [P[0, d, 2], g[2]], ':', color=GREY, lw=0.8)
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)'); ax.set_zlabel('z (m)')
    ax.set_title(f'Mission {s} (x5): Hungarian assignment (dotted) and flight')
    ax.legend(fontsize=7, loc='upper left')
    ax = fig.add_subplot(132)
    t = np.arange(len(P)) * DT
    err = _norm(P - r['goal'][s][None])
    for d, nm in enumerate(NAMES):
        ax.semilogy(t, np.maximum(err[:, d], 1e-3), color=COL[nm], label=f'Drone {nm}')
    ax.axhline(EPS_FORM, color='k', ls='--', label=f'eps = {EPS_FORM} m')
    ax.set_xlabel('Time since Phase 4 start (s)'); ax.set_ylabel('Distance to assigned vertex (m)')
    ax.set_title(f'Convergence in {t[-1]:.1f} s (all within eps)')
    ax.legend(fontsize=7); ax.grid(True, which='both', alpha=0.3)
    ax = fig.add_subplot(133)
    F = R['F']
    S = len(R['seeds'])
    keys = [k for k, _ in F_LABELS]
    names = ['Hungarian\n+ORCA+AW', 'no ORCA', 'identity', 'identity\nno ORCA', 'card PID\n(no AW)', 'PD only']
    tc = [np.mean(F[k]['k_conv'][F[k]['k_conv'] > 0]) * DT for k in keys]
    conv = [int(np.sum(F[k]['k_conv'] > 0)) for k in keys]
    x = np.arange(len(keys))
    ax.bar(x, tc, color=['#2ca02c', '#98df8a', '#ff7f0e', '#ffbb78', '#d62728', '#9467bd'], edgecolor='k')
    for xi, (tt, cv) in enumerate(zip(tc, conv)):
        ax.text(xi, tt + 2, f'{cv}/{S}', ha='center', fontsize=8)
    ax.set_xticks(x); ax.set_xticklabels(names, fontsize=8)
    ax.set_ylabel('Mean convergence time (s), converged missions')
    ax2 = ax.twinx()
    ax2.plot(x, [F[k]['minsep'].min() for k in keys], 'kD', label='Min separation (all missions)')
    ax2.axhline(R_SAFE, color='k', ls=':', lw=1)
    ax2.set_ylabel('Min separation (m)')
    ax2.set_ylim(0, 6)
    ax2.legend(fontsize=7, loc='upper left')
    ax.set_title('Phase 4 ablations (x5 start states; label: converged)')
    fig.suptitle('S100 Phase 4 — Grand Formation (Hungarian + PID + ORCA-lite)', fontweight='bold')
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, 'phase4_formation.png'), dpi=110)
    plt.close(fig)


def plot_dashboard(R, out_dir):
    s = R['rep']
    M = R['M']
    S = len(R['seeds'])
    fig, axes = plt.subplots(2, 2, figsize=(15, 10))
    ax = axes[0, 0]
    rows = [('k1', 'x1 (card)'), ('k5', 'x5')]
    for i, (key, lab) in enumerate(rows):
        te = M[key]['t_end'][s]
        t0 = 0.0
        for p in range(4):
            y = i * 5 + p
            ok = [M[key]['s1'], M[key]['sc2'] == 3, M[key]['s3'], M[key]['s4']][p][s]
            ax.barh(y, te[p] - t0, left=t0, color=PH_COL[p], edgecolor='k', alpha=0.85 if ok else 0.35,
                    hatch=None if ok else '//')
            ax.text(te[p] + 2, y, ('ok' if ok else 'FAILED') + f' {te[p]:.1f} s', va='center', fontsize=8)
            t0 = te[p]
    for p, tdl in enumerate(T_DEADLINE):
        ax.axvline(tdl, color=PH_COL[p], ls=':', lw=1)
    ax.axvline(T_TOTAL, color='k', ls='--', lw=1.5)
    ax.set_yticks([0, 1, 2, 3, 5, 6, 7, 8])
    ax.set_yticklabels([f'{lab} P{p + 1}' for _, lab in rows for p in range(4)])
    ax.invert_yaxis()
    ax.set_xlabel('Mission time (s)'); ax.set_xlim(0, 360)
    ax.set_title(f'Mission {s}: phase timeline (dotted: deadlines, dashed: 300 s)')
    ax = axes[0, 1]
    cfg = [('k1', 'PCP x1'), ('pn1', 'PN x1'), ('k1.25', 'PCP x1.25'), ('k1.5', 'PCP x1.5'),
           ('k2', 'PCP x2'), ('k5', 'PCP x5'), ('pn5', 'PN x5')]
    w = 0.11
    for j, (key, lab) in enumerate(cfg):
        r = M[key]
        v = [r['s1'].mean(), np.mean(r['sc2'] == 3), r['s3'].mean(), r['s4'].mean()]
        ax.bar(np.arange(4) + (j - 3) * w, 100 * np.array(v), w, label=lab)
    ax.set_xticks(range(4)); ax.set_xticklabels(['P1 catch', 'P2 3/3', 'P3 both', 'P4 formed'])
    ax.set_ylabel('Missions (%)'); ax.set_title(f'Per-phase success ({S} common missions)')
    ax.legend(fontsize=7, ncol=2); ax.grid(True, axis='y', alpha=0.3)
    ax = axes[1, 0]
    bins = np.arange(-0.25, 10.75, 0.5)
    for key, lab, col in [('k1', 'PCP x1', 'k'), ('pn1', 'PN x1', '#ff9896'),
                          ('k5', 'PCP x5', COL['A']), ('pn5', 'PN x5', '#9467bd')]:
        ax.hist(M[key]['score'], bins=bins, histtype='step', lw=2, color=col,
                label=f'{lab}: {M[key]["score"].mean():.2f}')
    ax.set_xlabel('Mission score (max 10)'); ax.set_ylabel('Missions')
    ax.set_title('Score distribution (legend: mean)'); ax.legend(fontsize=8); ax.grid(True, alpha=0.3)
    ax = axes[1, 1]
    sc = np.array([M[f'k{k:g}']['score'].mean() for k in KAPPAS])
    ci = np.array([1.96 * M[f'k{k:g}']['score'].std(ddof=1) / np.sqrt(S) for k in KAPPAS])
    ax.errorbar(KAPPAS, sc, yerr=ci, fmt='ko-', capsize=4, label='Mean score (95% CI)')
    ax.set_xlabel('Battery capacity multiplier kappa'); ax.set_ylabel('Mean score')
    ax.set_ylim(0, 10.5)
    ax2 = ax.twinx()
    ax2.plot(KAPPAS, [100 * M[f'k{k:g}']['s3'].mean() for k in KAPPAS], 's--', color=COL['D'], label='P3 success')
    ax2.plot(KAPPAS, [100 * M[f'k{k:g}']['s4'].mean() for k in KAPPAS], '^--', color=COL['E'], label='P4 success')
    ax2.set_ylabel('Phase success (%)'); ax2.set_ylim(0, 105)
    h1, l1 = ax.get_legend_handles_labels()
    h2, l2 = ax2.get_legend_handles_labels()
    ax.legend(h1 + h2, l1 + l2, fontsize=8, loc='lower right')
    ax.set_title('Battery capacity sweep (PCP, same missions)'); ax.grid(True, alpha=0.3)
    fig.suptitle('S100 Ultimate Grand Challenge — Mission Dashboard', fontweight='bold')
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, 'mission_dashboard.png'), dpi=100)
    plt.close(fig)


def _arena(ax):
    ax.set_xlim(0, ARENA[0]); ax.set_ylim(0, ARENA[1]); ax.set_zlim(0, ARENA[2])
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)'); ax.set_zlabel('z (m)')
    sq = np.array([[0, 0], [SAR_HI, 0], [SAR_HI, SAR_HI], [0, SAR_HI], [0, 0]])
    ax.plot(sq[:, 0], sq[:, 1], 0, color=GREY, lw=1)
    th = np.linspace(0, 2 * np.pi, 60)
    ax.plot(CATCH_CENTRE[0] + 8 * np.cos(th), CATCH_CENTRE[1] + 8 * np.sin(th), 0, color=GREY, lw=1)
    ax.scatter(DELIVERY_WPS[:, 0], DELIVERY_WPS[:, 1], DELIVERY_WPS[:, 2], marker='D', color=TARGET_C, s=40)
    ax.scatter(STAR[:, 0], STAR[:, 1], STAR[:, 2], marker='*', color=TARGET_C, s=90)


def plot_mission_3d(R, out_dir):
    s = R['rep']
    fig = plt.figure(figsize=(15, 6.5))
    for i, (key, lab) in enumerate([('k1', 'battery x1 (card)'), ('k5', 'battery x5')]):
        r = R['M'][key]
        k = int(r['k_last'][s])
        P = r['pos_log'][s, :k + 1].astype(float)
        ax = fig.add_subplot(1, 2, i + 1, projection='3d')
        _arena(ax)
        for d, nm in enumerate(NAMES):
            ax.plot(P[:, d, 0], P[:, d, 1], P[:, d, 2], color=COL[nm], lw=1.1, label=f'Drone {nm}')
        ax.set_title(f'Mission {s}, {lab}: score {r["score"][s]:.2f}, ends at {r["t_end"][s, 3]:.1f} s')
        ax.legend(fontsize=7, loc='upper left')
        ax.view_init(elev=28, azim=-125)
    fig.suptitle('S100 — whole mission (grey: SAR zone and launcher circle; green: delivery '
                 'waypoints and star vertices)', fontweight='bold')
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, 'mission_3d.png'), dpi=100)
    plt.close(fig)


def save_animation(R, out_dir):
    s = R['rep']
    runs = [('k1', 'battery x1 (card)'), ('k5', 'battery x5')]
    K = max(int(R['M'][k]['k_last'][s]) for k, _ in runs)
    ball = ball_ticks(R, 'pcp', s, K)
    n_frames = int(np.ceil(K / GIF_EVERY)) + 1
    fig = plt.figure(figsize=(11, 5.2))
    axes, dots, trails, balls, titles = [], [], [], [], []
    for i, (key, lab) in enumerate(runs):
        ax = fig.add_subplot(1, 2, i + 1, projection='3d')
        _arena(ax)
        ax.view_init(elev=28, azim=-125)
        dd = [ax.plot([], [], [], 'o', color=COL[nm], ms=6, label=nm)[0] for nm in NAMES]
        tt = [ax.plot([], [], [], '-', color=COL[nm], lw=1)[0] for nm in NAMES]
        bb, = ax.plot([], [], [], 'o', color=BALL_C, ms=4)
        if i == 0:
            ax.legend(fontsize=7, loc='upper left', ncol=5)
        titles.append(ax.set_title('', fontsize=9))
        axes.append(ax); dots.append(dd); trails.append(tt); balls.append(bb)
    ph_name = {1: 'Phase 1 catching', 2: 'Phase 2 logistics', 3: 'Phase 3 SAR', 4: 'Phase 4 formation',
               5: 'done'}

    def update(i):
        k = min(i * GIF_EVERY, K)
        for j, (key, lab) in enumerate(runs):
            r = R['M'][key]
            kk = min(k, int(r['k_last'][s]))
            P = r['pos_log'][s]
            for d in range(5):
                dots[j][d].set_data_3d([P[kk, d, 0]], [P[kk, d, 1]], [P[kk, d, 2]])
                lo = max(0, kk - 100)
                trails[j][d].set_data_3d(P[lo:kk + 1, d, 0], P[lo:kk + 1, d, 1], P[lo:kk + 1, d, 2])
            if np.all(np.isfinite(ball[kk])):
                balls[j].set_data_3d([ball[kk, 0]], [ball[kk, 1]], [ball[kk, 2]])
            else:
                balls[j].set_data_3d([], [], [])
            ph = int(r['ph_log'][s, kk])
            titles[j].set_text(f'{lab}\nt = {kk * DT:5.1f} s  {ph_name[ph]}')
        return []

    ani = animation.FuncAnimation(fig, update, frames=n_frames, interval=100, blit=False)
    ani.save(os.path.join(out_dir, 'animation.gif'), writer='pillow', fps=10, dpi=62)
    plt.close(fig)
    print(f'GIF: {n_frames} frames (every {GIF_EVERY * DT:.1f} s of mission time, up to t = {K * DT:.1f} s)')


if __name__ == '__main__':
    os.makedirs(OUT_DIR, exist_ok=True)
    R = run_simulation()
    report(R)
    plot_phase1(R, OUT_DIR)
    plot_phase2(R, OUT_DIR)
    plot_phase3(R, OUT_DIR)
    plot_phase4(R, OUT_DIR)
    plot_dashboard(R, OUT_DIR)
    plot_mission_3d(R, OUT_DIR)
    save_animation(R, OUT_DIR)
