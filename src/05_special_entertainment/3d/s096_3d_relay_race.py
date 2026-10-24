"""
S096 3D Upgrade — Drone Relay Race
==================================
The S096 relay (three point-mass drones, 3 x 100 m legs, baton passes when two drones stay
within 0.3 m and 0.5 m/s for 1 s; 6-state constant-velocity KF on 5 cm position fixes; launch
at the last moment a rest -> v_in minimum-snap intercept peaks at a_L = 0.8 a_max; 10 Hz
replanning; formation PD; 50 m exchange zone) is moved onto the 3D card course:

  course  (0,0,10) -> (50,0,6) -> RP1 (100,0,8) -> (150,0,16) -> RP2 (200,0,14) -> (250,0,12)
          -> finish (300,0,10); the baton holder tracks the waypoints with the card's 3D velocity
          director (switch radius 3 m) and keeps following the course past a relay point, so
          the RP1 exchange happens on the 0.16-slope climb towards (150,0,16);
  waits   B waits at (100,0,12) (4 m above RP1), C at (200,0,10) (4 m below RP2) (card).

Outgoing-drone planners (same seeds -> same disturbances and sensor noise, CRN):
  base    S096 planner unchanged: launch when the x-gap <= v_x T / 2 with T = 1.875 v_x / a_L,
          constant-velocity prediction of the KF state                    (2D baseline, X5)
  flatz   2D planner with a z-stabiliser: horizontal launch rule, predicted rendezvous altitude
          held at the current altitude estimate with v_z* = 0  (the card's "flat-z" predictor)
  cv3d    card: the launch gap and speed are measured along the 3D KF velocity, and the
          rendezvous state is the 3D constant-velocity prediction (uses v_z)
  course  added: the KF speed is propagated along the known course polyline, so the
          predicted rendezvous follows the slope change at the relay point

2D baselines on a flat course (same seeds): the S096 planner at a fixed race altitude z_f,
swept over 4-18 m, with the card's waiting stations (B 12 m, C 10 m) and the S096 cruise law.

Experiments: Monte Carlo at the card speeds (12/14/16), a speed grid (v_A, v_B; v_C = 20) per
planner and for the flat baseline, waiting-altitude sweeps for B and C (the planned altitude
decision), the flat-altitude sweep, and launch-acceleration / zone-length sensitivity.

Usage:
    conda activate drones
    python src/05_special_entertainment/3d/s096_3d_relay_race.py
"""

import os
import math
import importlib.util
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401

_HERE = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location(
    's096_2d', os.path.join(_HERE, '..', 's096_relay_race.py'))
m2d = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(m2d)

OUTPUT_DIR = os.path.normpath(os.path.join(
    _HERE, '..', '..', '..', 'outputs', '05_special_entertainment', '3d', 's096_3d_relay_race'))

# ── Parameters (card / S096) ──────────────────────────────────────────────────
D_HANDOFF, V_TOL, T_HANDOFF = m2d.D_HANDOFF, m2d.V_TOL, m2d.T_HANDOFF   # 0.3 m, 0.5 m/s, 1 s
A_MAX, V_MAX = m2d.A_MAX, m2d.V_CRUISE_MAX                              # 5 m/s^2, 20 m/s
DT = m2d.DT                                                              # 0.02 s
V_CRUISE = [12.0, 14.0, 16.0]
KP_F, KD_F, K_CRUISE, K_ALT = m2d.KP_F, m2d.KD_F, m2d.K_CRUISE, m2d.K_ALT
A_LAUNCH, REPLAN_DT, L_ZONE = m2d.A_LAUNCH, m2d.REPLAN_DT, m2d.L_ZONE  # 4 m/s^2, 0.1 s, 50 m
SNAP_PEAK = m2d.SNAP_PEAK                                                # 15/8
GRID_V, A_SWEEP, L_SWEEP = m2d.GRID_V, m2d.A_SWEEP, m2d.L_SWEEP
T_MAX = m2d.T_MAX
RELAY_XS = [0.0, 100.0, 200.0, 300.0]

COURSE_WP = np.array([[0.0, 0, 10], [50, 0, 6], [100, 0, 8], [150, 0, 16], [200, 0, 14],
                      [250, 0, 12], [300, 0, 10]])                       # card waypoints
CHECKPOINTS = COURSE_WP[[1, 3, 5]]
RP_Z = [8.0, 14.0]
WAITS_CARD = (12.0, 10.0)        # card: B 4 m above RP1, C 4 m below RP2
Z_MIN, Z_MAX = 2.0, 20.0
R_SWITCH = 3.0
K_BOUND = 10.0
BOUND_BAND = 0.5

# ── Parameters (added) ────────────────────────────────────────────────────────
N_MC = 30
N_SWEEP = 10                     # seeds for the altitude sweeps
ERR_AT = (5.0, 4.0, 3.0, 2.0, 1.0)  # s before t*: read-out of the planned-rendezvous z error
WAIT_SWEEP = [4.0, 6.0, 8.0, 10.0, 12.0, 14.0, 16.0, 18.0]
ZF_SWEEP = [4.0, 6.0, 8.0, 10.0, 12.0, 14.0, 16.0, 18.0]
Z_FLAT_REF = 10.0                # flat baseline reference altitude (S096)
VARIANTS = {'base': ('x', 'cv'), 'flatz': ('path', 'flatz'),
            'cv3d': ('path', 'cv'), 'course': ('path', 'course')}
ABLATIONS = {'zfix': ('path', 'zfix'), 'vzfix': ('path', 'vzfix'),
             'zvzfix': ('path', 'zvzfix'), 'xyfix': ('path', 'xyfix'),
             'flatz_rp2': ('path', 'flatz_rp2')}          # added probes
LABELS = {'base': 'S096 planner (x rule, CV)', 'flatz': 'flat-z predictor',
          'cv3d': '3D CV predictor (card)', 'course': 'course-aware predictor',
          'flat': f'flat course z = {Z_FLAT_REF:.0f} m (S096)'}
VCOL = {'base': '#7f7f7f', 'flatz': '#1f77b4', 'cv3d': '#ff7f0e', 'course': '#d62728',
        'flat': 'k'}
NAMES = ['A', 'B', 'C']
DRONE_COLORS = ['red', 'blue', 'green']


# ── Course polyline ───────────────────────────────────────────────────────────

class Course:
    """Waypoint polyline, monotone in x, extended level for 3 km past the finish (only reached
    by drones that are past the finish line, e.g. in unfinished races of the speed grid)."""

    def __init__(self, wps):
        self.W = np.vstack([wps, wps[-1] + np.array([3000.0, 0.0, 0.0])])
        d = np.diff(self.W, axis=0)
        self.seg_len = np.linalg.norm(d, axis=1)
        self.u = d / self.seg_len[:, None]
        self.cum = np.concatenate([[0.0], np.cumsum(self.seg_len)])
        self.n = len(self.seg_len)

    def s_of_x(self, x):
        i = int(np.clip(np.searchsorted(self.W[:, 0], x, 'right') - 1, 0, self.n - 1))
        return self.cum[i] + (x - self.W[i, 0]) / self.u[i, 0]

    def at(self, s):
        s = np.atleast_1d(s)
        i = np.clip(np.searchsorted(self.cum, s, 'right') - 1, 0, self.n - 1)
        return self.W[i] + (s - self.cum[i])[:, None] * self.u[i], self.u[i]

    def z_at_x(self, x):
        return float(np.interp(x, self.W[:, 0], self.W[:, 2]))


COURSE_3D = Course(COURSE_WP)


def flat_course(zf):
    w = COURSE_WP.copy()
    w[:, 2] = zf
    return Course(w)


def leg_lengths(course):
    return [course.cum[2 * k + 2] - course.cum[2 * k] for k in range(3)]


# ── Prediction and planning ───────────────────────────────────────────────────

def predict(kind, xkf, taus, course):
    """Predicted incoming position and velocity after each lookahead in taus."""
    p, v = xkf[:3], xkf[3:]
    taus = np.atleast_1d(taus)
    if kind == 'cv':
        return p[None, :] + taus[:, None] * v[None, :], np.broadcast_to(v, (len(taus), 3))
    if kind == 'flatz':
        vh = np.array([v[0], v[1], 0.0])
        pf = p[None, :] + taus[:, None] * vh[None, :]
        return pf, np.broadcast_to(vh, (len(taus), 3))
    # ablations (added): which part of the course-aware state fixes the CV planner?
    if kind in ('zfix', 'vzfix', 'zvzfix', 'xyfix'):
        pc, vc = predict('course', xkf, taus, course)
        pv, vv = predict('cv', xkf, taus, course)
        pv, vv = pv.copy(), np.array(vv).copy()
        if kind in ('zfix', 'zvzfix'):                  # course altitude
            pv[:, 2] = pc[:, 2]
        if kind in ('vzfix', 'zvzfix'):                 # course v_z
            vv[:, 2] = vc[:, 2]
        if kind == 'xyfix':                             # course horizontal state, CV vertical
            pv[:, :2], vv[:, :2] = pc[:, :2], np.array(vc)[:, :2]
        return pv, vv
    if kind == 'flatz_rp2':                             # CV at RP1, flat-z only at RP2
        return predict('cv' if p[0] < COURSE_WP[3, 0] else 'flatz', xkf, taus, course)
    sp = float(np.linalg.norm(v))                       # course-aware
    pf, tan = course.at(course.s_of_x(p[0]) + sp * taus)
    pf = pf.copy()
    pf[:, 1] += p[1]                                    # keep the lateral offset
    return pf, sp * tan


def plan_peak(po, vo, pf, vf, T):
    """Peak |a| of the rest-state (a = j = 0) degree-7 min-snap from (po, vo) to (pf, vf)."""
    bnd = np.stack([po, vo * T, pf, vf * T], axis=-1)            # (3, 4)
    acc = (m2d._B2[:, m2d._COLS] @ bnd.T) / T ** 2               # (41, 3)
    return float(np.linalg.norm(acc, axis=1).max())


def bound_term(z):
    """Soft altitude bound (card, sign corrected: pushes back inside the band)."""
    if z < Z_MIN + BOUND_BAND:
        return K_BOUND * (Z_MIN + BOUND_BAND - z)
    if z > Z_MAX - BOUND_BAND:
        return K_BOUND * (Z_MAX - BOUND_BAND - z)
    return 0.0


# ── Race simulation ───────────────────────────────────────────────────────────

def run_simulation(variant='course', v_cruise=V_CRUISE, seed=0, z_flat=None, waits=WAITS_CARD,
                   a_launch=A_LAUNCH, record=False):
    """One race. z_flat=None -> 3D card course; a number -> flat course at that altitude
    with the S096 cruise law (2D baseline). waits = waiting altitudes of B and C."""
    rule, pk = {**VARIANTS, **ABLATIONS}[variant]
    flat = z_flat is not None
    course = flat_course(z_flat) if flat else COURSE_3D
    st = [course.W[0].copy(), np.array([100.0, 0.0, waits[0]]),
          np.array([200.0, 0.0, waits[1]])]
    rng_dist = np.random.default_rng([seed, 1])        # same streams as S096 (CRN)
    rng_obs = np.random.default_rng([seed, 2])
    p = np.array([s.copy() for s in st])
    v = np.zeros((3, 3))
    holder = 0
    widx = [1, None, None]
    z_done = [None, None, None]
    kf = m2d.KalmanFilter6D(p[0], v[0])
    mode, plan, window = 'wait', None, 0.0
    handoffs = []
    hist = dict(t=[], p=[], v=[], holder=[]) if record else None
    finish = None
    n_bound = 0
    u_max_int, n_sat, n_int = 0.0, 0, 0
    leg_t = 0.0
    for k in range(int(round(T_MAX / DT))):
        t = k * DT
        z_noise = rng_obs.normal(0.0, m2d.SIGMA_OBS, 3)
        d_noise = rng_dist.normal(0.0, m2d.SIGMA_D, (3, 3))
        a = np.zeros((3, 3))
        out = holder + 1 if holder < 2 else None
        for i in range(3):
            if i == holder:
                if flat:
                    vd = np.array([v_cruise[i], -K_ALT * p[i, 1], -K_ALT * (p[i, 2] - z_flat)])
                else:
                    while (widx[i] < len(course.W) - 1 and
                           np.linalg.norm(course.W[widx[i]] - p[i]) < R_SWITCH):
                        widx[i] += 1
                    d = course.W[widx[i]] - p[i]
                    vd = v_cruise[i] * d / np.linalg.norm(d)
                a[i] = K_CRUISE * (vd - v[i])
            elif i < holder:                    # finished: brake, hold altitude
                zh = z_flat if flat else z_done[i]
                a[i] = K_CRUISE * (np.array([0.0, -K_ALT * p[i, 1], -K_ALT * (p[i, 2] - zh)])
                                   - v[i])
            elif i != out:
                a[i] = m2d.hold_cmd(p[i], v[i], st[i])
        if out is not None:
            kf.step(p[holder] + z_noise)
            xin = kf.x
            po, vo = p[out], v[out]
            if mode == 'wait':
                a[out] = m2d.hold_cmd(po, vo, st[out])
                if rule == 'x':
                    v_in, gap = xin[3], po[0] - xin[0]
                else:
                    pf0, vf0 = predict(pk, xin, 0.0, course)
                    v_in = float(np.linalg.norm(vf0[0]))
                    gap = float((po - pf0[0]) @ vf0[0]) / max(v_in, 1e-9)
                # armed only while the incoming drone moves forward (S096: v_x > 1 m/s); the
                # 3D projection alone would launch on the noisy first KF velocity estimates
                if v_in > 1.0 and xin[3] > 1.0:
                    tau = SNAP_PEAK * v_in / a_launch
                    if gap <= v_in * tau / 2:
                        pf, vf = predict(pk, xin, tau, course)
                        plan = dict(t0=t, T=tau, t_star=t + tau, last=t, tau=tau,
                                    c=m2d.snap_coeffs(po, vo, pf[0], vf[0], tau),
                                    launch_gap=float(np.linalg.norm(xin[:3] - po)),
                                    launch_peak=plan_peak(po, vo, pf[0], vf[0], tau),
                                    launch_delay=t - leg_t, pf=pf[0].copy(), vf=vf[0].copy(),
                                    dz_wait=float(po[2] - p[holder, 2]),
                                    targets=[(tau, pf[0].copy())])
                        mode = 'intercept'
            if mode == 'intercept':
                rem = plan['t_star'] - t
                if rem > 0.3 and t - plan['last'] >= REPLAN_DT - 1e-9:
                    _, _, a0, j0 = m2d.snap_eval(plan['c'], t - plan['t0'], plan['T'])
                    pf, vf = predict(pk, xin, rem, course)
                    plan.update(t0=t, T=rem, last=t,
                                c=m2d.snap_coeffs(po, vo, pf[0], vf[0], rem, a0, j0))
                    plan['targets'].append((rem, pf[0].copy()))
                if rem <= 0.0:
                    mode = 'hold'
                    plan['p_true'] = p[holder].copy()            # incoming position at t*
                    plan['pred_err'] = plan['pf'] - p[holder]    # launch prediction error
                    plan['vpred_err'] = plan['vf'] - v[holder]
                else:
                    pr, vr, ar, _ = m2d.snap_eval(plan['c'], t - plan['t0'], plan['T'])
                    a[out] = KP_F * (pr - po) + KD_F * (vr - vo) + ar
                    un = float(np.linalg.norm(a[out]))
                    u_max_int = max(u_max_int, un)
                    n_sat += un > A_MAX
                    n_int += 1
            if mode == 'hold':
                a[out] = KP_F * (xin[:3] - po) + KD_F * (xin[3:] - vo)
        for i in range(3):
            bz = bound_term(p[i, 2])
            if bz != 0.0:
                a[i, 2] += bz
                n_bound += 1
        for i in range(3):
            p_old = p[i].copy()
            p[i], v[i] = m2d.integrate(p[i], v[i], m2d.limit(a[i]) + d_noise[i])
            assert np.linalg.norm(v[i]) <= V_MAX + 1e-9, 'speed limit exceeded'
            assert np.linalg.norm(p[i] - p_old) <= V_MAX * DT + 1e-9, 'teleport'
            assert p[i, 2] > 0.0, 'below ground'
        if record:
            hist['t'].append(t + DT); hist['p'].append(p.copy()); hist['v'].append(v.copy())
            hist['holder'].append(holder)
        if out is not None:
            dp = np.linalg.norm(p[holder] - p[out])
            dv = np.linalg.norm(v[holder] - v[out])
            window = window + DT if (dp <= D_HANDOFF and dv <= V_TOL) else 0.0
            if window >= T_HANDOFF - 1e-9:
                pl = plan or {}
                if 'pf' in pl and 'pred_err' not in pl:
                    # pass completed before t*: extrapolate the true incoming state to t*
                    dt_rem = pl['t_star'] - (t + DT)
                    pl['p_true'] = p[holder] + dt_rem * v[holder]
                    pl['pred_err'] = pl['pf'] - pl['p_true']
                    pl['vpred_err'] = pl['vf'] - v[holder]
                # z error of the planned rendezvous point (all plans target the same t*) as a
                # function of the time left when it was planned
                zerr_at = np.full(len(ERR_AT), np.nan)
                if 'targets' in pl:
                    rems = np.array([r_ for r_, _ in pl['targets']])
                    for q, ra in enumerate(ERR_AT):
                        kq = int(np.argmin(np.abs(rems - ra)))
                        if abs(rems[kq] - ra) < 0.06:
                            zerr_at[q] = pl['targets'][kq][1][2] - pl['p_true'][2]
                handoffs.append(dict(
                    t=t + DT, x=float(p[holder, 0]), z=float(p[holder, 2]), dp=dp, dv=dv,
                    zone_ok=p[holder, 0] <= RELAY_XS[out] + L_ZONE,
                    launch_gap=pl.get('launch_gap', np.nan),
                    launch_peak=pl.get('launch_peak', np.nan),
                    launch_delay=pl.get('launch_delay', np.nan),
                    tau=pl.get('tau', np.nan), dz_wait=pl.get('dz_wait', np.nan),
                    pred_err=pl.get('pred_err', np.full(3, np.nan)),
                    vpred_err=pl.get('vpred_err', np.full(3, np.nan)),
                    late=(t + DT) - pl.get('t_star', np.nan), zerr_at=zerr_at,
                    u_max=u_max_int, sat_frac=n_sat / max(n_int, 1)))
                u_max_int, n_sat, n_int = 0.0, 0, 0
                z_done[holder] = float(p[holder, 2])
                holder = out
                window = 0.0
                leg_t = t + DT
                widx[holder] = int(np.searchsorted(course.W[:, 0], p[holder, 0], 'right'))
                mode, plan = 'wait', None
                if holder < 2:
                    kf = m2d.KalmanFilter6D(p[holder] + z_noise, v[holder])
        if holder == 2 and p[2, 0] >= RELAY_XS[3]:
            finish = t + DT
            break
    res = dict(finish=finish, handoffs=handoffs, n_bound=n_bound,
               zone_ok=len(handoffs) == 2 and all(h['zone_ok'] for h in handoffs))
    res['success'] = finish is not None and len(handoffs) == 2
    if record:
        for key in hist:
            hist[key] = np.array(hist[key])
        res['hist'] = hist
    return res


# ── Experiments ───────────────────────────────────────────────────────────────

def summarise(rows):
    n = len(rows)
    fin = np.array([r['finish'] if r['finish'] is not None else np.nan for r in rows])
    keys = ('x', 'dp', 'dv', 'launch_peak', 'u_max', 'sat_frac', 'late', 't', 'launch_gap')
    H = {kk: np.full((n, 2), np.nan) for kk in keys}
    PE = np.full((n, 2, 3), np.nan)
    ZE = np.full((n, 2, len(ERR_AT)), np.nan)
    VE = np.full((n, 2, 3), np.nan)
    for i, r in enumerate(rows):
        for j, h in enumerate(r['handoffs']):
            for kk in keys:
                H[kk][i, j] = h[kk]
            H['x'][i, j] -= RELAY_XS[j + 1]
            PE[i, j], VE[i, j] = h['pred_err'], h['vpred_err']
            ZE[i, j] = h['zerr_at']
    return dict(finish=fin, zone_ok=np.array([r['zone_ok'] for r in rows]),
                success=np.array([r['success'] for r in rows]), H=H, PE=PE, VE=VE, ZE=ZE,
                n_bound=sum(r['n_bound'] for r in rows), rows=rows)


def monte_carlo(variant, v_cruise=V_CRUISE, n=N_MC, **kw):
    return summarise([run_simulation(variant, v_cruise, seed=s, **kw) for s in range(n)])


def speed_grid(variant, z_flat=None):
    T = np.full((len(GRID_V), len(GRID_V)), np.nan)
    X = np.full_like(T, np.nan)
    for i, va in enumerate(GRID_V):
        for j, vb in enumerate(GRID_V):
            r = run_simulation(variant, [va, vb, V_MAX], seed=0, z_flat=z_flat)
            if r['success']:
                T[i, j] = r['finish']
                X[i, j] = max(h['x'] - RELAY_XS[k + 1] for k, h in enumerate(r['handoffs']))
    return T, X


def best_feasible(T, X, L=L_ZONE):
    feas = np.where(np.isfinite(T) & (np.nan_to_num(X, nan=np.inf) <= L), T, np.inf)
    if not np.isfinite(feas).any():
        return None
    i, j = np.unravel_index(np.argmin(feas), feas.shape)     # first minimum: deterministic
    return GRID_V[i], GRID_V[j], T[i, j]


def mc_line(tag, m):
    f = m['finish']
    ok = np.isfinite(f)
    s = (f'  {tag:34s} finished {ok.sum()}/{len(f)}, both in zone {m["zone_ok"].sum()}/{len(f)}')
    if ok.any():
        s += f', race {np.nanmean(f):.2f} ± {np.nanstd(f):.2f} s'
    for j in range(2):
        x = m['H']['x'][:, j]
        if np.isfinite(x).any():
            s += (f'; RP{j + 1} +{np.nanmean(x):.1f} ± {np.nanstd(x):.1f} m, late '
                  f'{np.nanmean(m["H"]["late"][:, j]):+.2f} s')
        else:
            s += f'; RP{j + 1} never'
    print(s)


def paired(tag, ma, mb, name_a, name_b):
    """Seed-by-seed comparison (CRN): race time and pass position."""
    fa = np.where(np.isfinite(ma['finish']), ma['finish'], np.inf)
    fb = np.where(np.isfinite(mb['finish']), mb['finish'], np.inf)
    diff = ma['finish'] - mb['finish']
    s = (f'  {tag}: {name_a} faster in {(fa < fb).sum()}/{len(fa)}, slower in '
         f'{(fa > fb).sum()}, tie {(fa == fb).sum()}; race time diff {name_a} - {name_b} '
         f'{np.nanmean(diff):+.3f} s (min {np.nanmin(diff):+.3f}, max {np.nanmax(diff):+.3f})')
    for j in range(2):
        dx = ma['H']['x'][:, j] - mb['H']['x'][:, j]
        s += (f'; RP{j + 1} pass {np.nanmean(dx):+.1f} m (closer to RP in '
              f'{int(np.sum(dx < 0))}/{len(dx)})')
    print(s)


# ── Plots ─────────────────────────────────────────────────────────────────────

def plot_trajectory_3d(r3, rf, out_dir):
    fig = plt.figure(figsize=(15, 7))
    ax = fig.add_subplot(111, projection='3d')
    for res, ls, tag, lw, al in [(r3, '-', 'course-aware, 3D course', 1.8, 1.0),
                                 (rf, '--', f'flat course z = {Z_FLAT_REF:.0f} m', 1.0, 0.55)]:
        P = res['hist']['p']
        H = res['hist']['holder']
        for i in range(3):
            m = H == i
            ax.plot(P[m, i, 0], P[m, i, 1], P[m, i, 2], ls, color=DRONE_COLORS[i], lw=lw,
                    alpha=al, label=f'Drone {NAMES[i]} with baton ({tag})')
        for h in res['handoffs']:
            ax.scatter([h['x']], [0], [h['z']], c='k', s=40, marker='o')
    ax.plot(COURSE_WP[:, 0], COURSE_WP[:, 1], COURSE_WP[:, 2], ':', color='grey', lw=1,
            label='course polyline')
    ax.scatter(CHECKPOINTS[:, 0], CHECKPOINTS[:, 1], CHECKPOINTS[:, 2], marker='D', c='orange',
               s=60, label='altitude checkpoints')
    for xr, zr in zip(RELAY_XS[1:3], RP_Z):
        ax.scatter([xr], [0], [zr], c='orange', s=160, alpha=0.5)
        ax.plot([xr, xr + L_ZONE], [0, 0], [Z_MIN] * 2, color='orange', lw=4, alpha=0.5)
    for zw, xr in zip(WAITS_CARD, RELAY_XS[1:3]):
        ax.scatter([xr], [0], [zw], marker='s', c='purple', s=40)
    ax.scatter([0], [0], [10], c='black', s=80, marker='^', label='Start')
    ax.scatter([300], [0], [10], c='gold', s=160, marker='*', label='Finish')
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)'); ax.set_zlabel('z (m)')
    ax.set_ylim(-3, 3); ax.set_zlim(Z_MIN, 18)
    ax.set_title('S096 3D relay, seed 0, 12/14/16 m/s: solid = 3D course (course-aware '
                 'planner), dashed = flat 2D baseline\nblack dots = baton passes, purple '
                 'squares = waiting stations, orange bars (at z = 2 m) = 50 m exchange zones')
    ax.legend(fontsize=7, ncol=2, loc='upper left')
    ax.view_init(elev=18, azim=-70)
    path = os.path.join(out_dir, 'trajectory_3d.png')
    fig.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_altitude(rec, out_dir):
    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    for var, ls in [('course', '-'), ('flatz', '--'), ('base', ':')]:
        h = rec[var]['hist']
        for i in range(3):
            axes[i].plot(h['t'], h['p'][:, i, 2], ls, color=VCOL[var], lw=1.4,
                         label=LABELS[var])
        for hh in rec[var]['handoffs']:
            for ax in axes:
                ax.axvline(hh['t'], color=VCOL[var], ls=ls, lw=0.8)
    for i, ax in enumerate(axes):
        for zc in CHECKPOINTS[:, 2]:
            ax.axhline(zc, color='orange', ls='--', lw=0.7)
        ax.set_ylabel(f'z of drone {NAMES[i]} (m)')
        ax.set_ylim(4, 18)
    axes[0].legend(fontsize=8, loc='upper right')
    axes[-1].set_xlabel('Time (s)')
    fig.suptitle('Altitude of each drone (seed 0, 12/14/16 m/s); orange dashed = checkpoint '
                 'altitudes 6 / 16 / 12 m; vertical lines = baton passes')
    fig.tight_layout()
    path = os.path.join(out_dir, 'altitude.png')
    fig.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_handoff(rec, relay, out_dir):
    fig, axes = plt.subplots(1, 3, figsize=(18, 4.8))
    for var, ls in [('flat', '-'), ('base', ':'), ('flatz', '--'), ('cv3d', '-.'),
                    ('course', '-')]:
        res = rec[var]
        h = res['hist']
        inc, out = relay, relay + 1
        t_end = res['handoffs'][relay]['t'] if len(res['handoffs']) > relay else h['t'][-1]
        t_beg = res['handoffs'][relay - 1]['t'] if relay > 0 else 0.0
        m = (h['t'] >= t_beg) & (h['t'] <= t_end + 1.0)
        dp = np.linalg.norm(h['p'][m, inc] - h['p'][m, out], axis=1)
        dv = np.linalg.norm(h['v'][m, inc] - h['v'][m, out], axis=1)
        dz = h['p'][m, out, 2] - h['p'][m, inc, 2]
        lw = 2.2 if var == 'course' else 1.2
        axes[0].semilogy(h['t'][m], dp, ls, color=VCOL[var], lw=lw, label=LABELS[var])
        axes[1].plot(h['t'][m], dv, ls, color=VCOL[var], lw=lw, label=LABELS[var])
        axes[2].plot(h['t'][m], dz, ls, color=VCOL[var], lw=lw, label=LABELS[var])
        if len(res['handoffs']) > relay:
            for ax in axes:
                ax.axvline(t_end, color=VCOL[var], ls=ls, lw=0.8)
    axes[0].axhline(D_HANDOFF, color='green', ls=':', lw=1.5, label='$d_{handoff}$ 0.3 m')
    axes[1].axhline(V_TOL, color='green', ls=':', lw=1.5, label='$v_{tol}$ 0.5 m/s')
    axes[1].set_ylim(0, 8)
    axes[0].set_ylabel('3D separation (m, log)')
    axes[1].set_ylabel('3D velocity mismatch (m/s)')
    axes[2].set_ylabel('$z_{out} - z_{in}$ (m)')
    for ax, ti in zip(axes, ['separation', 'velocity mismatch', 'altitude offset']):
        ax.set_xlabel('Time (s)'); ax.set_title(f'RP{relay + 1}: {ti}')
        ax.legend(fontsize=7)
    fig.suptitle(f'S096 3D handoff detail at RP{relay + 1} (seed 0, 12/14/16 m/s; vertical '
                 f'lines = baton passes)')
    fig.tight_layout()
    path = os.path.join(out_dir, f'handoff_rp{relay + 1}.png')
    fig.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_monte_carlo(mc, out_dir):
    fig, axes = plt.subplots(1, 3, figsize=(18, 5))
    keys = ['flat', 'base', 'flatz', 'cv3d', 'course']
    ax = axes[0]
    data = [mc[k]['finish'][np.isfinite(mc[k]['finish'])] for k in keys]
    bp = ax.boxplot(data, tick_labels=[LABELS[k].replace(' (', '\n(').replace('predictor',
                                                                               '\npredictor')
                                       for k in keys], patch_artist=True)
    for patch, k in zip(bp['boxes'], keys):
        patch.set_facecolor(VCOL[k]); patch.set_alpha(0.6)
    ax.set_ylabel('Race time (s)')
    ax.set_title(f'Race time, 12/14/16 m/s, {N_MC} seeds')
    ax.tick_params(axis='x', labelsize=7)
    hi = 130.0                                    # last bin collects every pass beyond it
    for j, ax in enumerate(axes[1:]):
        n_over = 0
        for k in keys:
            x = mc[k]['H']['x'][:, j]
            x = x[np.isfinite(x)]
            n_over += int(np.sum(x >= hi - 1))
            ax.hist(np.minimum(x, hi - 0.5), bins=np.arange(20, hi + 1e-9, 1.0), histtype='step',
                    lw=1.6, color=VCOL[k], label=LABELS[k])
        ax.axvline(L_ZONE, color='orange', lw=2, label='zone end 50 m')
        ax.set_xlabel(f'RP{j + 1} pass position past the relay point (m); last bin holds '
                      f'{n_over} passes >= {hi - 1:.0f} m')
        ax.set_ylabel('Seeds')
        ax.set_title(f'RP{j + 1} baton-pass position ({N_MC} seeds)')
        ax.legend(fontsize=7)
    fig.tight_layout()
    path = os.path.join(out_dir, 'monte_carlo.png')
    fig.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_altitude_sweeps(ws, zs, out_dir):
    fig, axes = plt.subplots(1, 3, figsize=(18, 5))
    for j, ax in enumerate(axes[:2]):
        for var, mk in [('course', 'o-'), ('base', 's--')]:
            pk = [np.nanmean(ws[(var, j, z)]['H']['launch_peak'][:, j]) for z in WAIT_SWEEP]
            xx = [np.nanmean(ws[(var, j, z)]['H']['x'][:, j]) for z in WAIT_SWEEP]
            ax.plot(WAIT_SWEEP, pk, mk, color=VCOL[var], label=f'{LABELS[var]}: planned peak')
            ax2 = ax.twinx() if var == 'course' else ax2
            ax2.plot(WAIT_SWEEP, xx, mk, color=VCOL[var], alpha=0.45, mfc='none',
                     label=f'{LABELS[var]}: pass position')
        ax.axvline(RP_Z[j], color='orange', ls=':', label=f'RP{j + 1} altitude')
        ax.axvline(WAITS_CARD[j], color='purple', ls=':', label='card waiting altitude')
        ax.axhline(A_MAX, color='k', lw=0.7, ls='--')
        ax.set_xlabel(f'Waiting altitude of drone {NAMES[j + 1]} (m)')
        ax.set_ylabel('Planned intercept peak |a| (m/s$^2$)')
        ax2.set_ylabel('Pass position past RP (m), faded')
        ax.set_title(f'RP{j + 1}: waiting altitude sweep ({N_SWEEP} seeds)')
        h1, l1 = ax.get_legend_handles_labels()
        h2, l2 = ax2.get_legend_handles_labels()
        ax.legend(h1 + h2, l1 + l2, fontsize=7, loc='upper center')
    ax = axes[2]
    ax.plot(ZF_SWEEP, [np.nanmean(zs[z]['finish']) for z in ZF_SWEEP], 'o-', color='k',
            label='flat course (S096 planner)')
    for var in ('course', 'base'):
        ax.axhline(np.nanmean(ws[(var, 0, WAITS_CARD[0])]['finish']), color=VCOL[var],
                   ls='--', label=f'3D course, {LABELS[var]}')
    ax.set_xlabel('Fixed race altitude $z_f$ of the flat course (m)')
    ax.set_ylabel(f'Mean race time (s), {N_SWEEP} seeds')
    ax.set_title('2D baseline: flat-course altitude sweep (waits B 12 m, C 10 m)')
    ax.legend(fontsize=8)
    fig.tight_layout()
    path = os.path.join(out_dir, 'altitude_sweeps.png')
    fig.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_speed(grids, out_dir):
    fig, axes = plt.subplots(1, len(grids), figsize=(6 * len(grids), 5.5))
    vmin = np.nanmin([np.nanmin(T) for T, _ in grids.values()])
    vmax = 32.0
    for ax, (name, (T, X)) in zip(axes, grids.items()):
        Z = np.isfinite(T) & (np.nan_to_num(X, nan=np.inf) <= L_ZONE)
        im = ax.imshow(np.where(Z, T, np.nan), origin='lower', cmap='viridis_r', vmin=vmin,
                       vmax=vmax, extent=[GRID_V[0] - 1, GRID_V[-1] + 1, GRID_V[0] - 1,
                                          GRID_V[-1] + 1])
        vb, va = np.meshgrid(GRID_V, GRID_V)
        bad = np.isfinite(T) & ~Z
        ax.scatter(vb[bad], va[bad], marker='x', c='grey', s=25, label='pass outside zone')
        ax.scatter(vb[~np.isfinite(T)], va[~np.isfinite(T)], marker='.', c='k', s=25,
                   label='not finished')
        b = best_feasible(T, X)
        if b:
            ax.scatter([b[1]], [b[0]], marker='*', c='red', s=250,
                       label=f'fastest feasible {b[2]:.2f} s')
        ax.set_xlabel('$v_B$ (m/s)'); ax.set_ylabel('$v_A$ (m/s)')
        ax.set_title(name, fontsize=10)
        ax.legend(fontsize=7, loc='upper center', bbox_to_anchor=(0.5, -0.12), ncol=2)
    fig.colorbar(im, ax=axes, label='Race time (s), seed 0, $v_C$ = 20 m/s')
    path = os.path.join(out_dir, 'speed_optimisation.png')
    fig.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def save_animation(res, out_dir, step=10):
    h = res['hist']
    n = len(h['t'])
    idx = m2d.gif_indices(n, step)
    fig = plt.figure(figsize=(12, 7.5))
    ax3 = fig.add_axes([-0.05, 0.33, 1.1, 0.67], projection='3d')
    ax3.set_box_aspect((3.0, 0.6, 1.0))
    axz = fig.add_axes([0.07, 0.07, 0.9, 0.27])
    ax3.plot(COURSE_WP[:, 0], COURSE_WP[:, 1], COURSE_WP[:, 2], ':', color='grey')
    ax3.scatter(CHECKPOINTS[:, 0], CHECKPOINTS[:, 1], CHECKPOINTS[:, 2], marker='D', c='orange')
    ax3.set_xlim(0, 320); ax3.set_ylim(-5, 5); ax3.set_zlim(4, 18)
    ax3.set_xlabel('x (m)'); ax3.set_ylabel('y (m)'); ax3.set_zlabel('z (m)')
    ax3.view_init(elev=20, azim=-75)
    fig.suptitle('S096 3D relay race, course-aware planner (seed 0, 12/14/16 m/s)', y=0.97)
    axz.plot(COURSE_WP[:, 0], COURSE_WP[:, 2], ':', color='grey', label='course')
    axz.scatter(CHECKPOINTS[:, 0], CHECKPOINTS[:, 2], marker='D', c='orange',
                label='checkpoints')
    for xr in RELAY_XS[1:3]:
        axz.axvspan(xr, xr + L_ZONE, color='orange', alpha=0.12)
    axz.set_xlim(-5, 320); axz.set_ylim(4, 18)
    axz.set_xlabel('x (m)'); axz.set_ylabel('z (m)')
    dots = [ax3.plot([], [], [], 'o', color=c, ms=7)[0] for c in DRONE_COLORS]
    trails = [ax3.plot([], [], [], '-', color=c, lw=1)[0] for c in DRONE_COLORS]
    zl = [axz.plot([], [], '-', color=c, lw=1.2, label=f'Drone {nm}')[0]
          for c, nm in zip(DRONE_COLORS, NAMES)]
    zd = [axz.plot([], [], 'o', color=c, ms=6)[0] for c in DRONE_COLORS]
    baton, = axz.plot([], [], 'D', color='white', mec='k', ms=8, label='baton')
    axz.legend(fontsize=7, loc='lower right', ncol=3)
    txt = axz.text(0.01, 0.88, '', transform=axz.transAxes, fontsize=9, family='monospace')

    def update(fi):
        k = idx[fi]
        for i in range(3):
            P = h['p'][:k + 1, i]
            dots[i].set_data([P[-1, 0]], [P[-1, 1]]); dots[i].set_3d_properties([P[-1, 2]])
            trails[i].set_data(P[:, 0], P[:, 1]); trails[i].set_3d_properties(P[:, 2])
            zl[i].set_data(P[:, 0], P[:, 2])
            zd[i].set_data([P[-1, 0]], [P[-1, 2]])
        b = int(h['holder'][k])
        baton.set_data([h['p'][k, b, 0]], [h['p'][k, b, 2] + 1.0])
        z = h['p'][k, :, 2]
        txt.set_text(f't = {h["t"][k]:5.2f} s  baton: {NAMES[b]}  z A/B/C = '
                     f'{z[0]:4.1f}/{z[1]:4.1f}/{z[2]:4.1f} m')
        return dots + trails + zl + zd + [baton, txt]

    anim = animation.FuncAnimation(fig, update, frames=len(idx), interval=50, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    anim.save(path, writer='pillow', fps=20, dpi=65)
    plt.close(fig)
    print(f'Saved: {path}  ({len(idx)} frames, step {step}, {n} samples)')


# ── Main ──────────────────────────────────────────────────────────────────────

def describe(tag, r):
    race = '-' if r['finish'] is None else f'{r["finish"]:.2f}'
    print(f'  [{tag}] finished {r["finish"] is not None}, race {race} s, both in zone '
          f'{r["zone_ok"]}')
    for j, h in enumerate(r['handoffs']):
        pe = h['pred_err']
        print(f'    RP{j + 1}: pass t {h["t"]:.2f} s at x {h["x"]:.1f} (+{h["x"] - RELAY_XS[j + 1]:.1f} m)'
              f', z {h["z"]:.2f} m; |dp| {h["dp"]:.3f} m, |dv| {h["dv"]:.3f} m/s; launch gap '
              f'{h["launch_gap"]:.1f} m, outgoing minus incoming altitude at launch '
              f'{h["dz_wait"]:+.2f} m, planned peak '
              f'{h["launch_peak"]:.2f} m/s^2, max commanded {h["u_max"]:.2f} m/s^2 '
              f'(saturated {100 * h["sat_frac"]:.0f} % of intercept steps); launch prediction '
              f'error at t* x {pe[0]:+.2f} z {pe[2]:+.2f} m, vz {h["vpred_err"][2]:+.2f} m/s; '
              f'pass {h["late"]:+.2f} s after t*')


if __name__ == '__main__':
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    print('=== S096 3D Drone Relay Race ===')
    L3 = leg_lengths(COURSE_3D)
    print('3D leg lengths (card waypoints): ' + ', '.join(f'leg {k + 1} {L:.2f} m'
                                                       for k, L in enumerate(L3))
          + f'; total {sum(L3):.2f} m (+{100 * (sum(L3) / 300 - 1):.2f} % over 300 m)')
    for k in range(COURSE_3D.n - 1):
        u = COURSE_3D.u[k]
        print(f'  segment {k + 1}: slope dz/dx {u[2] / u[0]:+.3f}, v_z at 12/14/16 m/s '
              + '/'.join(f'{vv * u[2]:+.2f}' for vv in V_CRUISE) + ' m/s')
    print(f'Card example: tau 3 s x delta v_z 0.5 m/s = {3 * 0.5:.1f} m vertical miss')

    # reproduction check: flat course, S096 stations -> must match S096 exactly
    rb = run_simulation('base', V_CRUISE, seed=0, z_flat=10.0, waits=(12.0, 12.0))
    r2 = m2d.run_simulation('kf', V_CRUISE, seed=0)
    same = (rb['finish'] == r2['finish'] and
            all(a['x'] == b['x'] for a, b in zip(rb['handoffs'], r2['handoffs'])))
    print(f'Reproduction: flat z 10 m, S096 stations, S096 planner -> race {rb["finish"]:.2f} s '
          f'(S096 script {r2["finish"]:.2f} s); identical: {same}')
    assert same, 'flat baseline does not reproduce S096'

    print('\nSeed 0, card speeds 12/14/16 m/s:')
    rec = {'flat': run_simulation('base', seed=0, z_flat=Z_FLAT_REF, record=True)}
    describe(LABELS['flat'], rec['flat'])
    for var in VARIANTS:
        rec[var] = run_simulation(var, seed=0, record=True)
        describe(LABELS[var] + ', 3D course', rec[var])
    for j in range(2):
        hs = rec['course']['hist']
        m = hs['holder'] == j + 1
        print(f'  Drone {NAMES[j + 1]} with baton: z range {hs["p"][m, j + 1, 2].min():.2f}-'
              f'{hs["p"][m, j + 1, 2].max():.2f} m')
    P = rec['course']['hist']['p']
    Hh = rec['course']['hist']['holder']
    exc = []
    for j in range(3):
        zmax = P[Hh == j, j, 2].max()
        zref = RP_Z[j] if j < 2 else 10.0
        exc.append(zmax - zref)
    print('  Max altitude of each leg minus the altitude where it ends (RP1 8, RP2 14, finish '
          '10 m): ' + ', '.join(f'leg {j + 1} {e:+.2f} m' for j, e in enumerate(exc)))

    print(f'\nMonte Carlo ({N_MC} seeds, card speeds; same disturbance and sensor noise):')
    mc = {'flat': monte_carlo('base', z_flat=Z_FLAT_REF)}
    for var in VARIANTS:
        mc[var] = monte_carlo(var)
    for k, m in mc.items():
        mc_line(LABELS[k] + ('' if k == 'flat' else ', 3D'), m)
    print('  second pass beyond the 300 m finish (race then ends at the pass, leg 3 never '
          'flown): ' + ', '.join(f'{k} {int(np.sum(m["H"]["x"][:, 1] + RELAY_XS[2] > RELAY_XS[3]))}'
                                 f'/{N_MC}' for k, m in mc.items()))
    print('  passes at >= 129 m past the relay point (collected in the last histogram bin): '
          + ', '.join(f'RP{j + 1} {sum(int(np.sum(m["H"]["x"][:, j] >= 129.0)) for m in mc.values())}'
                      for j in range(2)))
    for k in VARIANTS:
        m = mc[k]
        for j in range(2):
            print(f'    {k:7s} RP{j + 1}: |dp| max {np.nanmax(m["H"]["dp"][:, j]):.3f} m, |dv| max '
                  f'{np.nanmax(m["H"]["dv"][:, j]):.3f} m/s, planned peak '
                  f'{np.nanmean(m["H"]["launch_peak"][:, j]):.3f} m/s^2, max commanded '
                  f'{np.nanmean(m["H"]["u_max"][:, j]):.2f} m/s^2, saturated '
                  f'{100 * np.nanmean(m["H"]["sat_frac"][:, j]):.1f} % of intercept steps, '
                  f'launch prediction error z {np.nanmean(m["PE"][:, j, 2]):+.2f} ± '
                  f'{np.nanstd(m["PE"][:, j, 2]):.2f} m, v_z {np.nanmean(m["VE"][:, j, 2]):+.2f}'
                  f' m/s, |3D| {np.nanmean(np.linalg.norm(m["PE"][:, j], axis=1)):.2f} m')
    print('  z error of the planned rendezvous point vs the time left when it was planned '
          '(mean over seeds; all replans target the same t*):')
    for k in VARIANTS:
        for j in range(2):
            print(f'    {k:7s} RP{j + 1}: ' + ', '.join(
                f'{ra:.0f} s before t*: {np.nanmean(mc[k]["ZE"][:, j, q]):+.2f} m'
                for q, ra in enumerate(ERR_AT) if np.isfinite(mc[k]['ZE'][:, j, q]).any()))
    print(f'  altitude-bound term active in {sum(m["n_bound"] for m in mc.values())} drone-steps')
    paired('3D course-aware vs flat 2D baseline', mc['course'], mc['flat'], 'course', 'flat')
    paired('3D course-aware vs S096 planner on 3D', mc['course'], mc['base'], 'course', 'base')
    paired('3D CV (card) vs flat-z predictor', mc['cv3d'], mc['flatz'], 'cv3d', 'flatz')
    paired('course-aware vs 3D CV (card)', mc['course'], mc['cv3d'], 'course', 'cv3d')
    for k in VARIANTS:
        ratio = np.nanmean(mc[k]['finish']) / np.nanmean(mc['flat']['finish'])
        print(f'  {k:7s}: mean race time / flat baseline = {ratio:.4f} '
              f'({100 * (ratio - 1):+.2f} %; card target <= +10 %)')
    print(f'\nAblations ({N_MC} seeds, card speeds, same seeds):')
    mc_abl = {}
    for k, lab in (('cv3d', '3D CV (card)'), ('zfix', 'CV, course altitude, CV v_z'),
                   ('vzfix', 'CV altitude, course v_z'),
                   ('zvzfix', 'CV horizontal, course z and v_z'),
                   ('xyfix', 'course horizontal, CV z and v_z'), ('course', 'course-aware (both)'),
                   ('flatz', 'flat-z at both relays'),
                   ('flatz_rp2', 'CV at RP1, flat-z only at RP2')):
        m = mc[k] if k in mc else monte_carlo(k)
        mc_abl[k] = m
        H = m['H']
        print(f'  {lab:32s} both in zone {m["zone_ok"].sum()}/{N_MC}, race '
              f'{np.nanmean(m["finish"]):.2f} s; RP1 +{np.nanmean(H["x"][:, 0]):.1f} m, late '
              f'{np.nanmean(H["late"][:, 0]):+.2f} s, in zone {int(np.sum(H["x"][:, 0] <= L_ZONE))}/'
              f'{N_MC}; RP2 +{np.nanmean(H["x"][:, 1]):.1f} m, late {np.nanmean(H["late"][:, 1]):+.2f} s, '
              f'in zone {int(np.sum(H["x"][:, 1] <= L_ZONE))}/{N_MC}, RP2 launch gap '
              f'{np.nanmean(H["launch_gap"][:, 1]):.1f} m; second pass past the finish '
              f'{int(np.sum(H["x"][:, 1] + RELAY_XS[2] > RELAY_XS[3]))}/{N_MC}')
    x_cv = mc_abl['cv3d']['H']['x'][:, 0]
    gain_course = np.mean(x_cv - mc_abl['course']['H']['x'][:, 0])
    print('  RP1 pass, paired against 3D CV (card):')
    for k in ('zfix', 'vzfix', 'zvzfix', 'xyfix', 'course'):
        d = x_cv - mc_abl[k]['H']['x'][:, 0]
        print(f'    {k:7s}: {np.mean(d):+.2f} m closer to RP1 on average, closer in '
              f'{int(np.sum(d > 0))}/{N_MC} seeds; share of the course-aware gain '
              f'{100 * np.mean(d) / gain_course:.0f} %')

    print(f'\nLaunch-acceleration sensitivity ({N_SWEEP} seeds, card speeds; zone L = 50 m):')
    for al in A_SWEEP:
        for k in ['flat', 'base', 'flatz', 'cv3d', 'course']:
            m = (mc[k] if al == A_LAUNCH else
                 monte_carlo('base' if k == 'flat' else k, n=N_SWEEP, a_launch=al,
                             z_flat=Z_FLAT_REF if k == 'flat' else None))
            sl = slice(0, N_SWEEP)
            print(f'  a_L {al:.0f}: {k:7s} race {np.nanmean(m["finish"][sl]):.2f} s, RP1 '
                  f'+{np.nanmean(m["H"]["x"][sl, 0]):.1f} m, RP2 +{np.nanmean(m["H"]["x"][sl, 1]):.1f}'
                  f' m, both in zone {int(m["zone_ok"][sl].sum())}/{N_SWEEP}, '
                  + ', '.join(f'in L={L:.0f}: {int(np.sum(np.nanmax(np.nan_to_num(m["H"]["x"][sl], nan=np.inf), axis=1) <= L))}'
                              for L in L_SWEEP))

    print(f'\nWaiting-altitude sweep ({N_SWEEP} seeds, card speeds; the other drone at its card '
          f'altitude):')
    ws = {}
    for var in ('course', 'base'):
        for j in range(2):
            for zw in WAIT_SWEEP:
                waits = list(WAITS_CARD)
                waits[j] = zw
                m = monte_carlo(var, n=N_SWEEP, waits=tuple(waits))
                ws[(var, j, zw)] = m
                print(f'  {var:6s} {NAMES[j + 1]} waits at {zw:4.1f} m (RP{j + 1} {RP_Z[j]:.0f} m): '
                      f'race {np.nanmean(m["finish"]):.3f} s, RP{j + 1} pass '
                      f'+{np.nanmean(m["H"]["x"][:, j]):.2f} m, planned peak '
                      f'{np.nanmean(m["H"]["launch_peak"][:, j]):.3f} m/s^2, max commanded '
                      f'{np.nanmean(m["H"]["u_max"][:, j]):.2f} m/s^2, saturated '
                      f'{100 * np.nanmean(m["H"]["sat_frac"][:, j]):.1f} %, |dv| '
                      f'{np.nanmean(m["H"]["dv"][:, j]):.3f} m/s, in zone '
                      f'{int(m["zone_ok"].sum())}/{N_SWEEP}')
            best = min(WAIT_SWEEP, key=lambda z: (np.nanmean(ws[(var, j, z)]['H']['launch_peak'][:, j]), z))
            fast = min(WAIT_SWEEP, key=lambda z: (np.nanmean(ws[(var, j, z)]['finish']), z))
            card = ws[(var, j, WAITS_CARD[j])]
            al = ws[(var, j, RP_Z[j])]
            print(f'    -> {var} RP{j + 1}: lowest planned peak at {best:.0f} m, fastest race at '
                  f'{fast:.0f} m; RP altitude vs card wait: race '
                  f'{np.nanmean(al["finish"] - card["finish"]):+.3f} s, pass '
                  f'{np.nanmean(al["H"]["x"][:, j] - card["H"]["x"][:, j]):+.2f} m, planned peak '
                  f'{np.nanmean(al["H"]["launch_peak"][:, j] - card["H"]["launch_peak"][:, j]):+.2f}'
                  f' m/s^2 (faster in {int(np.sum(al["finish"] < card["finish"]))}/{N_SWEEP})')

    print(f'\nFlat 2D baseline: race-altitude sweep ({N_SWEEP} seeds, waits B 12 m, C 10 m):')
    zs = {}
    for zf in ZF_SWEEP:
        zs[zf] = monte_carlo('base', n=N_SWEEP, z_flat=zf)
        m = zs[zf]
        print(f'  z_f {zf:4.1f} m: race {np.nanmean(m["finish"]):.3f} ± {np.nanstd(m["finish"]):.3f} s, '
              f'RP1 +{np.nanmean(m["H"]["x"][:, 0]):.2f} m, RP2 +{np.nanmean(m["H"]["x"][:, 1]):.2f} m, '
              f'planned peaks {np.nanmean(m["H"]["launch_peak"][:, 0]):.2f} / '
              f'{np.nanmean(m["H"]["launch_peak"][:, 1]):.2f} m/s^2, in zone {int(m["zone_ok"].sum())}/{N_SWEEP}')
    zbest = min(ZF_SWEEP, key=lambda z: (np.nanmean(zs[z]['finish']), z))
    c3 = ws[('course', 0, WAITS_CARD[0])]
    print(f'  best flat altitude {zbest:.0f} m: {np.nanmean(zs[zbest]["finish"]):.3f} s; 3D course-aware '
          f'(same {N_SWEEP} seeds) {np.nanmean(c3["finish"]):.3f} s -> 3D minus best flat '
          f'{np.nanmean(c3["finish"] - zs[zbest]["finish"]):+.3f} s; spread over z_f '
          f'{max(np.nanmean(zs[z]["finish"]) for z in ZF_SWEEP) - min(np.nanmean(zs[z]["finish"]) for z in ZF_SWEEP):.3f} s')

    print('\nSpeed grid (seed 0, v_C = 20 m/s, zone 50 m):')
    grids = {f'flat z = {Z_FLAT_REF:.0f} m, S096 planner': speed_grid('base', z_flat=Z_FLAT_REF),
             '3D course, S096 planner': speed_grid('base'),
             '3D course, course-aware planner': speed_grid('course')}
    opt = {}
    for name, (T, X) in grids.items():
        Zg = np.isfinite(T) & (np.nan_to_num(X, nan=np.inf) <= L_ZONE)
        b = best_feasible(T, X)
        opt[name] = b
        print(f'  {name}: finished {np.isfinite(T).sum()}/{T.size}, feasible {Zg.sum()}/{T.size}, '
              f'fastest feasible ' + (f'({b[0]:.0f}, {b[1]:.0f}) -> {b[2]:.2f} s' if b else 'none')
              + '; zone sensitivity: ' + ', '.join(
                  f'L {L:.0f}: ' + (f'({bb[0]:.0f}, {bb[1]:.0f}) {bb[2]:.2f} s' if bb else 'none')
                  for L in L_SWEEP for bb in [best_feasible(T, X, L)]))
    names = list(grids)
    print('\nMonte Carlo at each optimum (30 seeds):')
    mopt = {}
    for name, var, zf in [(names[0], 'base', Z_FLAT_REF), (names[1], 'base', None),
                          (names[2], 'course', None)]:
        b = opt[name]
        mopt[name] = monte_carlo(var, [b[0], b[1], V_MAX], z_flat=zf)
        mc_line(f'{name} @ ({b[0]:.0f}, {b[1]:.0f}, 20)', mopt[name])
    paired('optimised 3D course-aware vs optimised flat', mopt[names[2]], mopt[names[0]],
           '3D', 'flat')
    paired('optimised 3D course-aware vs optimised 3D S096 planner', mopt[names[2]],
           mopt[names[1]], 'course', 'base')
    print('  Per-leg times (seed 0; start -> pass 1 -> pass 2 -> finish):')
    leg_times = {}
    for tag, var, vc, zf in [('flat, card speeds', 'base', V_CRUISE, Z_FLAT_REF),
                             ('3D fixed speed (course-aware)', 'course', V_CRUISE, None),
                             ('3D optimised (course-aware)', 'course',
                              [opt[names[2]][0], opt[names[2]][1], V_MAX], None),
                             ('flat optimised', 'base',
                              [opt[names[0]][0], opt[names[0]][1], V_MAX], Z_FLAT_REF)]:
        r = run_simulation(var, vc, seed=0, z_flat=zf)
        ts = [0.0] + [h['t'] for h in r['handoffs']] + [r['finish']]
        leg_times[tag] = np.diff(ts)
        print(f'    {tag}: ' + ', '.join(f'{x:.2f}' for x in leg_times[tag])
              + f' s; total {r["finish"]:.2f} s')

    print()
    plot_trajectory_3d(rec['course'], rec['flat'], OUTPUT_DIR)
    plot_altitude(rec, OUTPUT_DIR)
    plot_handoff(rec, 0, OUTPUT_DIR)
    plot_handoff(rec, 1, OUTPUT_DIR)
    plot_monte_carlo(mc, OUTPUT_DIR)
    plot_altitude_sweeps(ws, zs, OUTPUT_DIR)
    plot_speed(grids, OUTPUT_DIR)
    save_animation(rec['course'], OUTPUT_DIR)
