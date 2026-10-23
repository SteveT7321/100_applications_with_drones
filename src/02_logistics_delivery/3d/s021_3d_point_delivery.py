"""
S021 3D Upgrade — Point Delivery
================================
The 2D S021 starts at a fixed 2.0 m cruise layer and descends vertically.  The
3D variant flies a full four-phase mission from ground to ground:

  TAKEOFF  vertical bang-bang climb, |u_z| <= A_Z_MAX, from z = 0 to z_cruise
  CRUISE   level flight at z_cruise along the depot->goal bearing; horizontal
           acceleration bounded by A_MAX, ground speed capped by the wind
           triangle (airspeed <= V_MAX) in the altitude-dependent wind
  DESCENT  Pontryagin minimum-time 3D transfer to the pad (|u| <= A_MAX, soft
           landing), started at the cruise instant that minimises total time
  LANDED   touchdown check (radius, height, speed)

The cruise altitude z_cruise is chosen on the card's 56-point grid (0.5-6.0 m)
to minimise total mission time.  The same optimisation is repeated for a long
hop (10x the card distance, same bearing), for the reverse (headwind) bearing
and for a sweep of distances, so the altitude trade-off the card describes is
actually exercised.  Baselines: the 2D 2.0 m layer flown through the same
pipeline, and a decoupled 2D-style descent (stop above the pad, then drop).

The model is deterministic (no random numbers), so no Monte-Carlo is needed.

Outputs (outputs/02_logistics_delivery/3d/s021_3d_point_delivery/):
  trajectory_3d.png, telemetry.png, altitude_sweep.png, descent_profile.png,
  wind_field.png, energy_breakdown.png, animation.gif

Usage:
    conda activate drones
    python src/02_logistics_delivery/3d/s021_3d_point_delivery.py
"""

import matplotlib
matplotlib.use('Agg')

import math
import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from matplotlib.animation import FuncAnimation, PillowWriter
from scipy.optimize import least_squares, minimize, minimize_scalar

OUTPUT_DIR = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', '..', '..',
    'outputs', '02_logistics_delivery', '3d', 's021_3d_point_delivery'))
os.makedirs(OUTPUT_DIR, exist_ok=True)

# ── Card constants ────────────────────────────────────────────────────────────
V_MAX      = 5.0      # m/s   max airspeed (card: "max horizontal speed")
A_Z_MAX    = 2.0      # m/s^2 takeoff vertical acceleration bound
A_MAX      = 3.0      # m/s^2 3D acceleration bound (descent; also cruise, added)
Z_MIN      = 0.5      # m     lowest allowed cruise layer
Z_MAX      = 8.0      # m     altitude ceiling
Z_GRID     = np.linspace(0.5, 6.0, 56)   # card search grid (0.1 m step)
R_LAND     = 0.20     # m
V_LAND_MAX = 0.3      # m/s
Z_TOUCH    = 0.05     # m
DT         = 1.0 / 48.0
P_HOVER    = 30.0     # W
K_DRAG_0   = 0.08     # kg/m
ALPHA_Z    = 0.05     # 1/m
V_WIND_0   = 1.5      # m/s at Z_REF, wind blows along +x
Z_REF      = 2.0      # m
MASS       = 0.5      # kg
G          = 9.81     # m/s^2

DEPOT = np.array([-4.0, 0.0, 0.0])
GOAL  = np.array([3.5, 2.5, 0.0])

# ── Added constants (documented in the README) ───────────────────────────────
Z_LAYER_2D   = 2.0     # m   the 2D S021 cruise layer (baseline)
LONG_FACTOR  = 10.0    # long-hop distance = 10 x card distance, same bearing
DIST_FACTORS = [1.0, 2.0, 3.0, 5.0, 7.0, 10.0, 20.0]
Z_SWEEP_COARSE = np.linspace(0.5, 6.0, 12)   # 0.5 m grid for the distance sweep
S_FINE_MAX   = 30.0    # m  descent-start grid: 21 points within 30 m of the pad
N_S_COARSE   = 6       #    plus 6 points spread over the whole leg
M_SHOOT      = 1000    # samples used to check a descent's airspeed / plot it
N_SUB        = 4       # midpoint substeps per 48 Hz step for the energy integral
EPS_V        = 1e-6    # m/s tolerance on the airspeed cap

D_XY_CARD = float(np.linalg.norm(GOAL[:2] - DEPOT[:2]))
LONG_GOAL = np.array([DEPOT[0] + LONG_FACTOR * (GOAL[0] - DEPOT[0]),
                      DEPOT[1] + LONG_FACTOR * (GOAL[1] - DEPOT[1]), 0.0])


# ── Wind, drag, wind triangle ────────────────────────────────────────────────
def wind_speed(z):
    return V_WIND_0 * z / Z_REF


def wind_vec(z):
    return np.array([wind_speed(z), 0.0, 0.0])


def k_drag(z):
    return K_DRAG_0 * (1.0 + ALPHA_Z * z)


def ground_speed_cap(z, dhat):
    """Max ground speed along unit dhat (xy) with airspeed <= V_MAX (wind triangle)."""
    w = wind_vec(z)[:2]
    w_al = float(w @ dhat)
    w_perp = float(np.linalg.norm(w - w_al * dhat))
    assert w_perp < V_MAX, "crosswind exceeds airspeed"
    v = w_al + np.sqrt(V_MAX ** 2 - w_perp ** 2)
    assert v > 0.0, "headwind exceeds airspeed"
    return v


def card_literal_vgnd(z):
    """The card's formula sqrt(V^2 - w^2): direction-blind (kept only for comparison)."""
    return np.sqrt(max(V_MAX ** 2 - wind_speed(z) ** 2, 0.1))


# ── Pontryagin descent: planar (along-track a, altitude z) shooting ──────────
# lambda_v(t) = c - b t,  u(t) = -A_MAX lambda_v / |lambda_v|.  Unknowns:
# phi (direction of c, |c| = 1), b (2), T.  Four terminal conditions:
# a(T) = s, z(T) = 0, v_a(T) = 0, v_z(T) = 0.  The problem is planar because the
# initial velocity and the remaining displacement both lie in the vertical
# plane through the bearing, so the cross-track co-state is zero.
def _controls(x, t):
    phi, b1, b2 = x[0], x[1], x[2]
    lam = np.stack([np.cos(phi) - b1 * t, np.sin(phi) - b2 * t], axis=-1)
    nrm = np.maximum(np.linalg.norm(lam, axis=-1), 1e-12)
    return -A_MAX * lam / nrm[..., None]


def _state_at(x, z0, v0, t):
    """Closed-form state under the linear-tangent law at times t (array).

    With lambda(t) = c - b t = sigma e + c_perp, sigma = c.e - |b| t, the
    integrals of u = -A lambda/|lambda| are elementary (sqrt and asinh), so
    the shooting residual is exact and smooth (no integration error).
    """
    t = np.atleast_1d(np.asarray(t, dtype=float))
    phi, b1, b2 = x[0], x[1], x[2]
    c = np.array([np.cos(phi), np.sin(phi)])
    b = np.array([b1, b2])
    p0 = np.array([0.0, z0])
    vv0 = np.array([v0, 0.0])
    B = float(np.linalg.norm(b))
    if B < 1e-12:
        u = -A_MAX * c
        v = vv0 + np.outer(t, u)
        p = p0 + np.outer(t, vv0) + 0.5 * np.outer(t ** 2, u)
        return p, v
    e = b / B
    cpar = float(c @ e)
    cperp = c - cpar * e
    q = float(np.linalg.norm(cperp))
    chat = cperp / q if q > 1e-15 else np.zeros(2)

    def FG(sig):
        R = np.sqrt(sig ** 2 + q ** 2)
        ash = q * np.arcsinh(sig / q) if q > 1e-15 else np.zeros_like(sig)
        F = np.outer(R, e) + np.outer(ash, chat)
        G = np.outer(0.5 * (sig * R + q * ash), e) + np.outer(sig * ash - q * R, chat)
        return F, G

    sig = cpar - B * t
    F, Gs = FG(sig)
    F0, G0 = FG(np.array([cpar]))
    v = vv0 + (A_MAX / B) * (F - F0)
    p = p0 + np.outer(t, vv0) + (A_MAX / B) * (-(Gs - G0) / B - np.outer(t, F0[0]))
    return p, v


def _integrate(x, z0, v0, m=M_SHOOT):
    t = np.linspace(0.0, x[3], m + 1)
    p, v = _state_at(x, z0, v0, t)
    return t, p, v


def _resid(x, z0, v0, s):
    """Terminal residual; scalar re-implementation of _state_at at t = T (speed)."""
    phi, b1, b2, T = float(x[0]), float(x[1]), float(x[2]), float(x[3])
    c1, c2 = math.cos(phi), math.sin(phi)
    B = math.hypot(b1, b2)
    if B < 1e-12:
        u1, u2 = -A_MAX * c1, -A_MAX * c2
        return np.array([v0 * T + 0.5 * u1 * T * T - s, z0 + 0.5 * u2 * T * T,
                         v0 + u1 * T, u2 * T])
    e1, e2 = b1 / B, b2 / B
    cpar = c1 * e1 + c2 * e2
    p1, p2 = c1 - cpar * e1, c2 - cpar * e2
    q = math.hypot(p1, p2)
    h1, h2 = (p1 / q, p2 / q) if q > 1e-15 else (0.0, 0.0)

    def fg(sig):
        R = math.sqrt(sig * sig + q * q)
        ash = q * math.asinh(sig / q) if q > 1e-15 else 0.0
        ga = 0.5 * (sig * R + q * ash)
        gb = sig * ash - q * R
        return (R * e1 + ash * h1, R * e2 + ash * h2, ga * e1 + gb * h1, ga * e2 + gb * h2)

    F01, F02, G01, G02 = fg(cpar)
    F1, F2, G1, G2 = fg(cpar - B * T)
    k = A_MAX / B
    va, vz = v0 + k * (F1 - F01), k * (F2 - F02)
    pa = v0 * T + k * (-(G1 - G01) / B - T * F01)
    pz = z0 + k * (-(G2 - G02) / B - T * F02)
    return np.array([pa - s, pz, va, vz])


_LS = dict(bounds=([-10.0, -60.0, -60.0, 1e-3], [10.0, 60.0, 60.0, 80.0]),
           xtol=1e-13, ftol=1e-13, gtol=1e-13, max_nfev=400)
SOLVE_STATS = {"warm": 0, "multistart": 0}


def solve_descent(z0, v0, s, x_warm=None):
    """Minimum-time transfer from (a=0, z0, v_a=v0, v_z=0) to (s, 0, 0, 0)."""
    if x_warm is not None:
        r = least_squares(_resid, x_warm, args=(z0, v0, s), method='lm',
                          xtol=1e-13, ftol=1e-13, gtol=1e-13, max_nfev=400)
        if np.max(np.abs(r.fun)) < 1e-8 and 1e-3 < r.x[3] < 80.0:
            SOLVE_STATS["warm"] += 1
            return r.x
    SOLVE_STATS["multistart"] += 1
    t_lb = 2.0 * np.sqrt(z0 / A_MAX) + abs(v0) / A_MAX + 0.1
    best = None
    for phi in np.linspace(-np.pi, np.pi, 12, endpoint=False):
        for bs in (0.3, 1.0, 3.0):
            for tg in (t_lb, 2.0 * t_lb + 2.0 * np.sqrt(abs(s) / A_MAX)):
                x0 = np.array([phi, bs * np.cos(phi), bs * np.sin(phi), tg])
                r = least_squares(_resid, x0, args=(z0, v0, s), **_LS)
                if np.max(np.abs(r.fun)) < 1e-8 and (best is None or r.x[3] < best[3] - 1e-9):
                    best = r.x
    assert best is not None, f"shooting failed z0={z0} v0={v0} s={s}"
    return best


def descent_states(x, z0, v0, m=M_SHOOT):
    return _integrate(x, z0, v0, m)


# ── Mission plan: takeoff + cruise (accelerate / hold) + Pontryagin descent ──
class Mission:
    """Geometry of one delivery: depot -> goal, both on the ground."""

    def __init__(self, depot, goal, name):
        self.depot, self.goal, self.name = depot.copy(), goal.copy(), name
        dxy = goal[:2] - depot[:2]
        self.d = float(np.linalg.norm(dxy))
        self.dhat = dxy / self.d


def cruise_state(tau, vcap):
    """Along-track distance and speed tau seconds into cruise (accelerate, then hold)."""
    t_acc = vcap / A_MAX
    if tau <= t_acc:
        return 0.5 * A_MAX * tau ** 2, A_MAX * tau
    return 0.5 * A_MAX * t_acc ** 2 + vcap * (tau - t_acc), vcap


def tau_at_distance(x, vcap):
    s_acc = vcap ** 2 / (2.0 * A_MAX)
    if x <= s_acc:
        return np.sqrt(2.0 * x / A_MAX)
    return vcap / A_MAX + (x - s_acc) / vcap


def descent_max_airspeed(x, z0, v0, mission, m=M_SHOOT):
    _, p, v = descent_states(x, z0, v0, m)
    v3 = np.column_stack([v[:, 0:1] * mission.dhat[None, :], v[:, 1]])
    w = np.column_stack([wind_speed(p[:, 1]), np.zeros(len(p)), np.zeros(len(p))])
    return float(np.max(np.linalg.norm(v3 - w, axis=1)))


def plan_mission(mission, z, x_init=None):
    """Optimise the descent-start time tau for cruise layer z.  Returns a dict."""
    vcap = ground_speed_cap(z, mission.dhat)
    t_to = 2.0 * np.sqrt(z / A_Z_MAX)
    tau_max = tau_at_distance(mission.d, vcap)
    cache = []        # (tau, x)

    def evaluate(tau):
        a0, v0 = cruise_state(tau, vcap)
        s = mission.d - a0
        xw = x_init
        if cache:
            xw = min(cache, key=lambda c: abs(c[0] - tau))[1]
        x = solve_descent(z, v0, s, xw)
        cache.append((tau, x))
        vmax_air = descent_max_airspeed(x, z, v0, mission)
        return t_to + tau + x[3], vmax_air, x

    # descent-start candidates by remaining distance s: fine near the pad, coarse beyond
    s_cand = np.concatenate([np.linspace(0.0, min(mission.d, S_FINE_MAX), 21),
                             np.linspace(0.0, mission.d, N_S_COARSE)])
    taus = np.unique(np.round([min(tau_at_distance(mission.d - s, vcap), tau_max)
                               for s in s_cand], 12))
    evals = [evaluate(t) for t in taus]
    feas = [e[1] <= V_MAX + EPS_V for e in evals]
    if not any(feas):
        # no descent start on the grid keeps airspeed <= V_MAX: layer infeasible
        return dict(z=z, vcap=vcap, t_to=t_to, T=np.inf, feasible=False,
                    mission=mission, x_tau0=cache[0][1])
    i_best = min((i for i in range(len(taus)) if feas[i]), key=lambda i: evals[i][0])
    best = (taus[i_best], *evals[i_best])
    lo = taus[max(i_best - 1, 0)]
    hi = taus[min(i_best + 1, len(taus) - 1)]

    def obj(tau):
        T, va, _ = evaluate(tau)
        return T + 50.0 * max(va - V_MAX, 0.0)

    if hi > lo:
        r = minimize_scalar(obj, bounds=(lo, hi), method='bounded',
                            options={'xatol': 1e-5})
        T, va, x = evaluate(r.x)
        if va > V_MAX + EPS_V:      # pull back to the feasibility boundary
            a, b = taus[i_best], r.x          # a feasible, b infeasible
            for _ in range(30):
                mid = 0.5 * (a + b)
                if evaluate(mid)[1] <= V_MAX + EPS_V:
                    a = mid
                else:
                    b = mid
            T, va, x = evaluate(a)
            r_x = a
        else:
            r_x = r.x
        if va <= V_MAX + EPS_V and T < best[1] - 1e-12:
            best = (r_x, T, va, x)
    tau, T, va, x = best
    a0, v0 = cruise_state(tau, vcap)
    return dict(z=z, vcap=vcap, t_to=t_to, tau=tau, t_desc=x[3], T=T,
                x=x, a0=a0, v0=v0, vair_desc=va, mission=mission,
                x_tau0=cache[0][1], feasible=True)


def decoupled_time(mission, z):
    """2D-style: climb, level cruise ending at rest above the pad, vertical drop."""
    vcap = ground_speed_cap(z, mission.dhat)
    t_to = 2.0 * np.sqrt(z / A_Z_MAX)
    d = mission.d
    if d >= vcap ** 2 / A_MAX:
        t_h = d / vcap + vcap / A_MAX
    else:
        t_h = 2.0 * np.sqrt(d / A_MAX)
    t_v = 2.0 * np.sqrt(z / A_MAX)
    return t_to + t_h + t_v, t_to, t_h, t_v


def card_literal_time(mission, z):
    """Card's own T_total with its sqrt(V^2 - w^2) ground speed and a vertical descent."""
    return (2.0 * np.sqrt(z / A_Z_MAX) + mission.d / card_literal_vgnd(z)
            + 2.0 * np.sqrt(z / A_MAX))


# ── 48 Hz simulation of a planned mission ────────────────────────────────────
def mission_state(plan, t):
    """Exact position, velocity, commanded acceleration and phase at time t.

    Every phase is a closed-form double-integrator solution (bang-bang climb,
    accelerate/hold cruise, linear-tangent descent), so the logged states have
    no integration error."""
    m = plan["mission"]
    z, t_to, tau, vcap = plan["z"], plan["t_to"], plan["tau"], plan["vcap"]
    if t < t_to:
        half = 0.5 * t_to
        if t < half:
            zz, vz, uz = 0.5 * A_Z_MAX * t ** 2, A_Z_MAX * t, A_Z_MAX
        else:
            r = t - half
            zz = 0.5 * z + A_Z_MAX * half * r - 0.5 * A_Z_MAX * r ** 2
            vz, uz = A_Z_MAX * (half - r), -A_Z_MAX
        return (np.array([m.depot[0], m.depot[1], zz]), np.array([0.0, 0.0, vz]),
                np.array([0.0, 0.0, uz]), "TAKEOFF")
    if t < t_to + tau:
        a, sp = cruise_state(t - t_to, vcap)
        ua = A_MAX if (t - t_to) < vcap / A_MAX else 0.0
        xy = m.depot[:2] + a * m.dhat
        return (np.array([xy[0], xy[1], z]), np.array([sp * m.dhat[0], sp * m.dhat[1], 0.0]),
                np.array([ua * m.dhat[0], ua * m.dhat[1], 0.0]), "CRUISE")
    td = min(t - t_to - tau, plan["t_desc"])
    p, v = _state_at(plan["x"], z, plan["v0"], np.array([td]))
    u = _controls(plan["x"], np.array([td]))[0]
    xy = m.depot[:2] + (plan["a0"] + p[0, 0]) * m.dhat
    return (np.array([xy[0], xy[1], p[0, 1]]),
            np.array([v[0, 0] * m.dhat[0], v[0, 0] * m.dhat[1], v[0, 1]]),
            np.array([u[0] * m.dhat[0], u[0] * m.dhat[1], u[1]]), "DESCENT")


def _power(pos, vel):
    air = float(np.linalg.norm(vel - wind_vec(pos[2])))
    p_drag = k_drag(pos[2]) * air ** 3
    p_climb = MASS * G * max(vel[2], 0.0)
    return P_HOVER + p_drag + p_climb, p_drag, p_climb, air


def simulate(plan):
    """Log the mission at 48 Hz and integrate power with N_SUB midpoints per step."""
    m = plan["mission"]
    T = plan["T"]
    t_to, tau, vcap = plan["t_to"], plan["tau"], plan["vcap"]
    breaks = sorted({0.5 * t_to, t_to, t_to + min(tau, vcap / A_MAX), t_to + tau})
    n_steps = int(np.ceil(T / DT - 1e-9))
    grid = [min(k * DT, T) for k in range(n_steps + 1)]
    log = dict(t=[], p=[], v=[], phase=[], pw=[])
    energy = {"TAKEOFF": 0.0, "CRUISE": 0.0, "DESCENT": 0.0}
    e_comp = {"hover": 0.0, "drag": 0.0, "climb": 0.0}
    max_air, max_u, min_z, max_z = 0.0, 0.0, 0.0, 0.0
    for k, tk in enumerate(grid):
        p, v, _, ph = mission_state(plan, tk)
        log["t"].append(tk)
        log["p"].append(p)
        log["v"].append(v)
        log["phase"].append("LANDED" if k == n_steps else ph)
        log["pw"].append(_power(p, v)[0])
        min_z, max_z = min(min_z, p[2]), max(max_z, p[2])
        if k == n_steps:
            break
        t1 = grid[k + 1]
        cuts = [tk] + [b for b in breaks if tk < b < t1] + [t1]
        for a, b in zip(cuts[:-1], cuts[1:]):
            hs = (b - a) / N_SUB
            for j in range(N_SUB):
                pm, vm, um, phm = mission_state(plan, a + (j + 0.5) * hs)
                pw, p_drag, p_climb, air = _power(pm, vm)
                energy[phm] += pw * hs
                e_comp["hover"] += P_HOVER * hs
                e_comp["drag"] += p_drag * hs
                e_comp["climb"] += p_climb * hs
                max_air = max(max_air, air)
                lim = A_Z_MAX if phm == "TAKEOFF" else A_MAX
                assert np.linalg.norm(um) <= lim + 1e-9, "acceleration bound violated"
                max_u = max(max_u, float(np.linalg.norm(um)))
                min_z, max_z = min(min_z, pm[2]), max(max_z, pm[2])
    for key in ("t", "p", "v", "pw"):
        log[key] = np.array(log[key])
    # physical invariants
    assert min_z > -1e-6, f"underground: {min_z}"
    assert max_z <= Z_MAX, "above ceiling"
    assert max_air <= V_MAX + 1e-4, f"airspeed {max_air} > V_MAX"
    assert np.all(np.diff(log["t"]) > 0)
    pos, vel = log["p"][-1], log["v"][-1]
    land_err = float(np.linalg.norm(pos - m.goal))
    touch_v = float(np.linalg.norm(vel))
    ok = land_err < R_LAND and pos[2] <= Z_TOUCH and touch_v < V_LAND_MAX
    assert ok, "landing condition violated"
    log.update(energy=energy, e_comp=e_comp, E=sum(energy.values()),
               land_err=land_err, touch_v=touch_v, max_air=max_air, ok=ok,
               max_z=max_z, min_z=min_z)
    return log


# ── Direct-method cross-check of the shooting solution ───────────────────────
def direct_check(z0, v0, s, n_seg=30):
    """Piecewise-constant controls + SLSQP; returns the minimum T found."""
    def unpack(y):
        return y[:n_seg], y[n_seg:2 * n_seg], y[-1]

    def terminal(y):
        ua, uz, T = unpack(y)
        h = T / n_seg
        va, vz = v0 + np.cumsum(ua) * h, np.cumsum(uz) * h
        va_prev = np.concatenate([[v0], va[:-1]])
        vz_prev = np.concatenate([[0.0], vz[:-1]])
        a = np.sum(va_prev * h + 0.5 * ua * h * h)
        zz = z0 + np.sum(vz_prev * h + 0.5 * uz * h * h)
        return np.array([a - s, zz, va[-1], vz[-1]])

    def bound(y):
        ua, uz, _ = unpack(y)
        return A_MAX ** 2 - ua ** 2 - uz ** 2

    T0 = 2.0 * np.sqrt((abs(s) + z0) / A_MAX) + abs(v0) / A_MAX
    y0 = np.concatenate([np.zeros(n_seg), np.zeros(n_seg), [T0]])
    r = minimize(lambda y: y[-1], y0, method='SLSQP',
                 constraints=[{'type': 'eq', 'fun': terminal},
                              {'type': 'ineq', 'fun': bound}],
                 bounds=[(-A_MAX, A_MAX)] * (2 * n_seg) + [(0.05, 60.0)],
                 options={'maxiter': 500, 'ftol': 1e-12})
    return float(r.x[-1]), bool(r.success), float(np.max(np.abs(terminal(r.x))))


# ── Main computation ─────────────────────────────────────────────────────────
def sweep(mission, z_grid):
    """Plans for every layer; each layer's shooting is warm-started from the previous one."""
    plans, x_prev = [], None
    for z in z_grid:
        plans.append(plan_mission(mission, z, x_prev))
        x_prev = plans[-1]["x_tau0"]
    return plans


def run_simulation():
    card = Mission(DEPOT, GOAL, "card hop")
    long = Mission(DEPOT, LONG_GOAL, "long hop")
    print(f"Card hop: depot {DEPOT.tolist()} -> goal {GOAL.tolist()}, d_xy = {card.d:.3f} m")
    print(f"Long hop: depot {DEPOT.tolist()} -> goal {LONG_GOAL.tolist()}, d_xy = {long.d:.3f} m")
    print(f"Bearing unit vector {np.round(card.dhat, 4).tolist()}, wind along +x "
          f"(tailwind component {card.dhat[0]:.4f} of w)")
    print()
    print("Wind triangle ground-speed cap v_gnd(z) along the bearing:")
    for z in (0.5, 2.0, 4.0, 6.0):
        print(f"  z = {z:.1f} m: w = {wind_speed(z):.3f} m/s, tailwind leg {ground_speed_cap(z, card.dhat):.3f} m/s, "
              f"headwind leg {ground_speed_cap(z, -card.dhat):.3f} m/s, card formula {card_literal_vgnd(z):.3f} m/s")
    print()
    # Card's cruise command v_cmd = V_MAX*dhat - w*x_hat read as a ground velocity:
    # the airspeed it needs is |v_cmd - w*x_hat| = |V_MAX*dhat - 2w*x_hat|.
    print("Airspeed needed by the card's literal cruise command V_MAX*dhat - w*x_hat (ground velocity):")
    for z in (0.5, 2.0, 4.0, 6.0):
        w2 = 2.0 * wind_vec(z)[:2]
        print(f"  z = {z:.1f} m: tailwind leg {np.linalg.norm(V_MAX * card.dhat - w2):.3f} m/s, "
              f"headwind leg {np.linalg.norm(-V_MAX * card.dhat - w2):.3f} m/s (V_MAX = {V_MAX:.1f})")
    print()

    res = {"card": card, "long": long}
    for key, mis in (("card", card), ("long", long)):
        plans = sweep(mis, Z_GRID)
        Ts = np.array([p["T"] for p in plans])
        i_opt = int(np.argmin(Ts))
        res[key + "_plans"] = plans
        res[key + "_iopt"] = i_opt
        res[key + "_dec"] = np.array([decoupled_time(mis, z)[0] for z in Z_GRID])
        res[key + "_lit"] = np.array([card_literal_time(mis, z) for z in Z_GRID])
        sims = [simulate(p) for p in plans]
        res[key + "_E"] = np.array([s["E"] for s in sims])
        res[key + "_sims"] = sims
        i2 = int(np.argmin(np.abs(Z_GRID - Z_LAYER_2D)))
        res[key + "_i2"] = i2
        po, p2 = plans[i_opt], plans[i2]
        so, s2 = sims[i_opt], sims[i2]
        iE = int(np.argmin(res[key + "_E"]))
        res[key + "_iE"] = iE
        print(f"=== {mis.name} (d_xy = {mis.d:.3f} m) ===")
        print(f"  time-optimal z_cruise* = {po['z']:.1f} m  (grid {Z_GRID[0]:.1f}-{Z_GRID[-1]:.1f} m)")
        for lab, p, s in (("z*", po, so), ("2D layer 2.0 m", p2, s2)):
            print(f"  [{lab}] T_total = {p['T']:.3f} s = takeoff {p['t_to']:.3f} + cruise {p['tau']:.3f} "
                  f"+ descent {p['t_desc']:.3f}; v_gnd cap {p['vcap']:.3f} m/s, descent starts "
                  f"{mis.d - p['a0']:.3f} m before the pad at {p['v0']:.3f} m/s")
            print(f"      energy {s['E']:.2f} J (takeoff {s['energy']['TAKEOFF']:.2f}, cruise "
                  f"{s['energy']['CRUISE']:.2f}, descent {s['energy']['DESCENT']:.2f}; hover "
                  f"{s['e_comp']['hover']:.2f}, drag {s['e_comp']['drag']:.2f}, climb {s['e_comp']['climb']:.2f})")
            print(f"      max airspeed {s['max_air']:.3f} m/s, landing error {s['land_err']*1000:.3f} mm, "
                  f"touchdown speed {s['touch_v']*1000:.3f} mm/s, landed {s['ok']}")
        dec_o = decoupled_time(mis, po['z'])
        print(f"  z* saves {p2['T'] - po['T']:.3f} s ({100*(p2['T']-po['T'])/p2['T']:.1f} %) vs the 2.0 m layer; "
              f"energy {s2['E'] - so['E']:+.2f} J for the 2.0 m layer")
        print(f"  decoupled 2D-style descent at z*: T = {dec_o[0]:.3f} s (takeoff {dec_o[1]:.3f} + level "
              f"{dec_o[2]:.3f} + vertical {dec_o[3]:.3f}); Pontryagin saves {dec_o[0] - po['T']:.3f} s")
        print(f"  decoupled optimum over grid: z = {Z_GRID[np.argmin(res[key + '_dec'])]:.1f} m, "
              f"T = {res[key + '_dec'].min():.3f} s")
        print(f"  card-literal formula optimum: z = {Z_GRID[np.argmin(res[key + '_lit'])]:.1f} m "
              f"(T_card = {res[key + '_lit'].min():.3f} s)")
        print(f"  energy-optimal z = {Z_GRID[iE]:.1f} m (E = {res[key + '_E'][iE]:.2f} J); "
              f"energy at time-optimal z = {so['E']:.2f} J")
        print(f"  T_total at grid ends: z=0.5 {plans[0]['T']:.3f} s, z=6.0 {plans[-1]['T']:.3f} s; "
              f"max over grid {Ts.max():.3f} s at z = {Z_GRID[np.argmax(Ts)]:.1f} m")
        n_cruise = sum(p['tau'] > 1e-9 for p in plans)
        print(f"  layers with a non-zero cruise phase: {n_cruise}/{len(plans)}")
        dT = np.diff(Ts)
        trend = ("increasing" if np.all(dT > 0) else "decreasing" if np.all(dT < 0)
                 else "non-monotone")
        print(f"  T_total(z) over the grid is {trend}")
        sel = [0.5, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
        print("  z (m):    " + "  ".join(f"{z:6.1f}" for z in sel))
        print("  T (s):    " + "  ".join(f"{Ts[int(np.argmin(np.abs(Z_GRID - z)))]:6.3f}" for z in sel))
        print("  E (J):    " + "  ".join(f"{res[key + '_E'][int(np.argmin(np.abs(Z_GRID - z)))]:6.1f}" for z in sel))
        print("  T_dec (s):" + "  ".join(f"{res[key + '_dec'][int(np.argmin(np.abs(Z_GRID - z)))]:6.3f}" for z in sel))
        print()

    # Shooting vs direct-method cross-check
    print("Direct-method cross-check (30 piecewise-constant segments, SLSQP):")
    chk = []
    for key in ("card", "long"):
        p = res[key + "_plans"][res[key + "_iopt"]]
        s_rem = res[key].d - p["a0"]
        Td, succ, err = direct_check(p["z"], p["v0"], s_rem)
        chk.append((key, p["t_desc"], Td))
        print(f"  {key}: shooting T3 = {p['t_desc']:.4f} s, direct T3 = {Td:.4f} s "
              f"(success {succ}, terminal residual {err:.1e})")
    t_rest = 2.0 * np.sqrt(np.hypot(card.d, Z_GRID[0]) / A_MAX)
    print(f"  card hop from rest at z = 0.5 m: straight-line bang-bang 2*sqrt(D/A) = {t_rest:.4f} s")
    print()

    # Distance sweep (tailwind bearing and reverse/headwind bearing)
    print("Distance sweep (0.5 m altitude grid):")
    print("  factor  d_xy(m)  z*_tail  T*_tail  T2.0_tail  z*_head  T*_head  T2.0_head")
    dsw = []
    for f in DIST_FACTORS:
        gt = np.array([DEPOT[0] + f * (GOAL[0] - DEPOT[0]), DEPOT[1] + f * (GOAL[1] - DEPOT[1]), 0.0])
        m_t = Mission(DEPOT, gt, f"tail x{f}")
        m_h = Mission(gt, DEPOT, f"head x{f}")
        row = [f, m_t.d]
        for mis in (m_t, m_h):
            Ts = np.array([p["T"] for p in sweep(mis, Z_SWEEP_COARSE)])
            i = int(np.argmin(Ts))
            T2 = Ts[int(np.argmin(np.abs(Z_SWEEP_COARSE - Z_LAYER_2D)))]
            row += [Z_SWEEP_COARSE[i], Ts[i], T2, Ts]
        dsw.append(row)
        bad = [",".join(f"{z:.1f}" for z, T in zip(Z_SWEEP_COARSE, row[k]) if not np.isfinite(T))
               or "none" for k in (5, 9)]
        print(f"  {f:5.1f}  {row[1]:8.2f}  {row[2]:7.1f}  {row[3]:7.3f}  {row[4]:9.3f}  "
              f"{row[6]:7.1f}  {row[7]:7.3f}  {row[8]:9.3f}   saving vs 2.0 m: tail "
              f"{100 * (row[4] - row[3]) / row[4]:4.1f} %, head {100 * (row[8] - row[7]) / row[8]:4.1f} %;"
              f" infeasible layers: tail {bad[0]}, head {bad[1]}")
    res["dsweep"] = dsw
    interior = [(r[0], d) for r in dsw for d, zz in (("tail", r[2]), ("head", r[6]))
                if Z_SWEEP_COARSE[0] < zz < Z_SWEEP_COARSE[-1]]
    print(f"  interior optima (not at 0.5 or 6.0 m): {interior if interior else 'none'}")
    for r in dsw:
        if r[0] in [f for f, _ in interior]:
            print(f"  factor {r[0]:.1f} tailwind T(z), z = 0.5..6.0 m: "
                  + " ".join(f"{T:.3f}" for T in r[5]))
    print()
    print(f"Shooting solves: {SOLVE_STATS['warm']} warm-started, {SOLVE_STATS['multistart']} multistart")
    return res


# ── Plots ────────────────────────────────────────────────────────────────────
PHASE_COL = {"TAKEOFF": "orange", "CRUISE": "red", "DESCENT": "magenta", "LANDED": "magenta"}


def _phase_segments(log):
    ph = log["phase"]
    out = []
    for name in ("TAKEOFF", "CRUISE", "DESCENT"):
        idx = [i for i in range(len(ph)) if ph[i] == name or (name == "DESCENT" and ph[i] == "LANDED")]
        if len(idx) == 0:
            continue
        i0, i1 = idx[0], idx[-1]
        out.append((name, max(i0 - 1, 0), i1))
    return out


def plot_trajectory_3d(res):
    fig = plt.figure(figsize=(15, 6.5))
    for k, key in enumerate(("card", "long")):
        ax = fig.add_subplot(1, 2, k + 1, projection='3d')
        mis = res[key]
        so = res[key + "_sims"][res[key + "_iopt"]]
        s2 = res[key + "_sims"][res[key + "_i2"]]
        po = res[key + "_plans"][res[key + "_iopt"]]
        p = s2["p"]
        ax.plot(p[:, 0], p[:, 1], p[:, 2], color='grey', ls='--', lw=1.2,
                label=f"2.0 m layer (2D altitude), T = {res[key + '_plans'][res[key + '_i2']]['T']:.2f} s")
        for name, i0, i1 in _phase_segments(so):
            q = so["p"][i0:i1 + 1]
            ax.plot(q[:, 0], q[:, 1], q[:, 2], color=PHASE_COL[name], lw=2.5,
                    label=f"{name.lower()} (z* = {po['z']:.1f} m)")
        ax.scatter(*mis.depot, color='royalblue', marker='s', s=90, label='Depot')
        ax.scatter(*mis.goal, color='limegreen', marker='*', s=260, label='Landing pad')
        th = np.linspace(0, 2 * np.pi, 60)
        ax.plot(mis.goal[0] + R_LAND * np.cos(th), mis.goal[1] + R_LAND * np.sin(th),
                np.zeros_like(th), color='limegreen', lw=1)
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.set_zlim(0, max(2.5, po['z'] + 0.5))
        ax.set_title(f"{mis.name}: d_xy = {mis.d:.1f} m, z* = {po['z']:.1f} m, "
                     f"T = {po['T']:.2f} s, E = {so['E']:.0f} J", fontsize=10)
        ax.legend(fontsize=7, loc='upper left')
    fig.suptitle("S021 3D Point Delivery — time-optimal missions (wind +x, tailwind leg)")
    fig.tight_layout()
    fig.savefig(os.path.join(OUTPUT_DIR, 'trajectory_3d.png'), dpi=130)
    plt.close(fig)


def plot_telemetry(res):
    fig, axes = plt.subplots(3, 2, figsize=(13, 10), sharex='col')
    for c, key in enumerate(("card", "long")):
        for lab, i, ls in (("z*", res[key + "_iopt"], '-'), ("2.0 m layer", res[key + "_i2"], '--')):
            s = res[key + "_sims"][i]
            p = res[key + "_plans"][i]
            t = s["t"]
            vh = np.linalg.norm(s["v"][:, :2], axis=1)
            air = np.linalg.norm(s["v"] - np.column_stack([wind_speed(s["p"][:, 2]),
                                                           np.zeros(len(t)), np.zeros(len(t))]), axis=1)
            axes[0, c].plot(t, s["p"][:, 2], ls=ls, color='tab:blue', label=f"{lab} ({p['z']:.1f} m)")
            axes[1, c].plot(t, vh, ls=ls, color='tab:red', label=f"ground speed, horiz. ({lab})")
            axes[1, c].plot(t, s["v"][:, 2], ls=ls, color='tab:green', label=f"vertical speed ({lab})")
            axes[1, c].plot(t, air, ls=ls, color='tab:purple', lw=0.9, label=f"airspeed ({lab})")
            axes[2, c].plot(t, s["pw"], ls=ls, color='darkorchid', label=f"{lab}")
            for tb in (p["t_to"], p["t_to"] + p["tau"]):
                for r in range(3):
                    axes[r, c].axvline(tb, color='grey', ls=ls, lw=0.6)
        axes[1, c].axhline(V_MAX, color='k', ls=':', lw=0.8, label='V_MAX airspeed')
        axes[2, c].axhline(P_HOVER, color='k', ls=':', lw=0.8, label='P_hover')
        axes[0, c].set_title(f"{res[key].name} (grey lines: takeoff end, descent start)")
        axes[0, c].set_ylabel('Altitude (m)')
        axes[1, c].set_ylabel('Speed (m/s)')
        axes[2, c].set_ylabel('Power (W)')
        axes[2, c].set_xlabel('Time (s)')
        for r in range(3):
            axes[r, c].grid(alpha=0.3)
            axes[r, c].legend(fontsize=6.5, loc='upper right')
    fig.suptitle("S021 3D Point Delivery — telemetry: time-optimal layer vs 2.0 m layer")
    fig.tight_layout()
    fig.savefig(os.path.join(OUTPUT_DIR, 'telemetry.png'), dpi=130)
    plt.close(fig)


def plot_altitude_sweep(res):
    fig, axes = plt.subplots(2, 2, figsize=(13, 9.5))
    for c, key in enumerate(("card", "long")):
        ax = axes[0, c]
        plans = res[key + "_plans"]
        T = np.array([p["T"] for p in plans])
        ax.plot(Z_GRID, T, 'k-', lw=2, label='T_total (Pontryagin descent)')
        ax.plot(Z_GRID, [p["t_to"] for p in plans], color='orange', label='takeoff')
        ax.plot(Z_GRID, [p["tau"] for p in plans], color='red', label='cruise')
        ax.plot(Z_GRID, [p["t_desc"] for p in plans], color='magenta', label='descent')
        ax.plot(Z_GRID, res[key + "_dec"], 'b--', label='decoupled 2D-style descent')
        ax.plot(Z_GRID, res[key + "_lit"], color='grey', ls=':', label="card formula sqrt(V²-w²)")
        i = res[key + "_iopt"]
        ax.plot(Z_GRID[i], T[i], 'g*', ms=16, label=f"z* = {Z_GRID[i]:.1f} m, {T[i]:.2f} s")
        ax.axvline(Z_LAYER_2D, color='grey', lw=0.6)
        ax.set_xlabel('cruise altitude z (m)')
        ax.set_ylabel('time (s)')
        ax.set_title(f"{res[key].name}: mission time vs cruise altitude")
        ax.grid(alpha=0.3)
        ax.legend(fontsize=7)
    ax = axes[1, 0]
    dsw = res["dsweep"]
    f = [r[1] for r in dsw]
    ax.semilogx(f, [r[2] for r in dsw], 'o-', color='tab:red', label='tailwind bearing (depot→goal)')
    ax.semilogx(f, [r[6] for r in dsw], 's--', color='tab:blue', label='headwind bearing (goal→depot)')
    ax.set_xlabel('delivery distance d_xy (m)')
    ax.set_ylabel('time-optimal z* (m)')
    ax.set_ylim(0, 6.5)
    ax.axhline(Z_LAYER_2D, color='grey', lw=0.6)
    ax2 = ax.twinx()
    ax2.semilogx(f, [100 * (r[4] - r[3]) / r[4] for r in dsw], 'o:', color='tab:red', alpha=0.6)
    ax2.semilogx(f, [100 * (r[8] - r[7]) / r[8] for r in dsw], 's:', color='tab:blue', alpha=0.6)
    ax2.set_ylabel('time saved vs 2.0 m layer (%)  [dotted]')
    ax.set_title('distance sweep: optimal layer and saving')
    ax.legend(fontsize=7, loc='center left')
    ax.grid(alpha=0.3)
    ax = axes[1, 1]
    for key, col in (("card", 'tab:green'), ("long", 'tab:purple')):
        E = res[key + "_E"]
        ax.plot(Z_GRID, E, color=col, label=f"{res[key].name}")
        iE = res[key + "_iE"]
        ax.plot(Z_GRID[iE], E[iE], '*', color=col, ms=14,
                label=f"min E at z = {Z_GRID[iE]:.1f} m ({E[iE]:.0f} J)")
    ax.set_xlabel('cruise altitude z (m)')
    ax.set_ylabel('mission energy (J)')
    ax.set_title('energy of the time-optimal plan at each layer')
    ax.grid(alpha=0.3)
    ax.legend(fontsize=7)
    fig.suptitle('S021 3D Point Delivery — cruise-altitude trade-off')
    fig.tight_layout()
    fig.savefig(os.path.join(OUTPUT_DIR, 'altitude_sweep.png'), dpi=130)
    plt.close(fig)


def plot_descent_profile(res):
    fig, axes = plt.subplots(1, 3, figsize=(16, 5))
    for c, key in enumerate(("card", "long")):
        ax = axes[c]
        mis = res[key]
        po = res[key + "_plans"][res[key + "_iopt"]]
        t, p, v = descent_states(po["x"], po["z"], po["v0"])
        a_start = po["a0"]
        ax.plot(a_start + p[:, 0], p[:, 1], color='magenta', lw=2, label=f"Pontryagin descent ({po['t_desc']:.2f} s)")
        ax.plot([0, a_start], [po["z"], po["z"]], color='red', lw=2, label='cruise')
        ax.plot([0, 0], [0, po["z"]], color='orange', lw=2, label='takeoff')
        ax.plot([0, mis.d, mis.d], [po["z"], po["z"], 0], 'b--', lw=1, label='decoupled: stop above pad, drop')
        u = _controls(po["x"], t[::100])
        ax.quiver(a_start + p[::100, 0], p[::100, 1], u[:, 0], u[:, 1], color='k', width=0.004,
                  scale=40, label='thrust direction u*(t)')
        ax.plot(mis.d, 0, 'g*', ms=14)
        ax.set_xlabel('along-track distance (m)')
        ax.set_ylabel('z (m)')
        ax.set_title(f"{mis.name}, z* = {po['z']:.1f} m (side view)")
        ax.grid(alpha=0.3)
        ax.legend(fontsize=7)
    ax = axes[2]
    for key, col in (("card", 'tab:green'), ("long", 'tab:purple')):
        po = res[key + "_plans"][res[key + "_iopt"]]
        t, _, _ = descent_states(po["x"], po["z"], po["v0"])
        u = _controls(po["x"], t)
        x = po["x"]
        lam = np.hypot(np.cos(x[0]) - x[1] * t, np.sin(x[0]) - x[2] * t)
        ok = lam > 1e-3 * lam.max()       # drop the sample where the co-state crosses zero
        ang = np.degrees(np.unwrap(np.arctan2(u[ok, 1], u[ok, 0])))
        ax.plot(t[ok], ang, color=col, label=f"{res[key].name}")
    ax.set_xlabel('time since descent start (s)')
    ax.set_ylabel('thrust angle atan2(u_z, u_along) (deg)')
    ax.set_title('optimal thrust direction (linear co-state law)')
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    fig.suptitle('S021 3D Point Delivery — Pontryagin minimum-time descent')
    fig.tight_layout()
    fig.savefig(os.path.join(OUTPUT_DIR, 'descent_profile.png'), dpi=130)
    plt.close(fig)


def plot_wind_field(res):
    zs = sorted({Z_GRID[res["card_iopt"]], Z_LAYER_2D, Z_GRID[res["long_iopt"]]})
    fig, axes = plt.subplots(1, len(zs), figsize=(5.2 * len(zs), 4.8))
    axes = np.atleast_1d(axes)
    xs, ys = np.meshgrid(np.linspace(-4, 3.5, 9), np.linspace(-1.5, 4.5, 7))
    for ax, z in zip(axes, zs):
        U = np.zeros_like(xs)
        V = np.zeros_like(xs)
        S = np.zeros_like(xs)
        for i in range(xs.shape[0]):
            for j in range(xs.shape[1]):
                d = GOAL[:2] - np.array([xs[i, j], ys[i, j]])
                n = np.linalg.norm(d)
                if n < 1e-6:
                    continue
                dh = d / n
                vg = ground_speed_cap(z, dh)
                U[i, j], V[i, j], S[i, j] = vg * dh[0], vg * dh[1], vg
        q = ax.quiver(xs, ys, U, V, S, cmap='viridis', clim=(0, 10), scale=90)
        ax.quiver([-3.5], [4.0], [wind_speed(z)], [0.0], color='k', scale=90)
        ax.text(-3.5, 4.25, f"wind {wind_speed(z):.2f} m/s", fontsize=8)
        ax.plot(*DEPOT[:2], 'bs', ms=8)
        ax.plot(*GOAL[:2], 'g*', ms=14)
        ax.set_title(f"z = {z:.1f} m: max ground speed toward pad")
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_aspect('equal')
        fig.colorbar(q, ax=ax, label='ground speed (m/s)', shrink=0.8)
    fig.suptitle('S021 3D Point Delivery — wind-triangle ground speed (airspeed 5 m/s)')
    fig.tight_layout()
    fig.savefig(os.path.join(OUTPUT_DIR, 'wind_field.png'), dpi=130)
    plt.close(fig)


def plot_energy_breakdown(res):
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    for ax, key in zip(axes, ("card", "long")):
        idx = sorted({0, res[key + "_i2"], len(Z_GRID) - 1, res[key + "_iopt"]})
        labels = []
        bottom = np.zeros(len(idx))
        for ph, col in (("TAKEOFF", 'orange'), ("CRUISE", 'red'), ("DESCENT", 'magenta')):
            vals = np.array([res[key + "_sims"][i]["energy"][ph] for i in idx])
            ax.bar(range(len(idx)), vals, bottom=bottom, color=col, label=ph.lower())
            bottom += vals
        for i in idx:
            tag = " (z*)" if i == res[key + "_iopt"] else ""
            labels.append(f"z = {Z_GRID[i]:.1f} m{tag}")
        for k, b in enumerate(bottom):
            ax.text(k, b, f"{b:.0f} J", ha='center', va='bottom', fontsize=8)
        ax.set_xticks(range(len(idx)))
        ax.set_xticklabels(labels, fontsize=8)
        ax.set_ylabel('energy (J)')
        ax.set_title(f"{res[key].name}: energy per phase")
        ax.legend(fontsize=8)
        ax.grid(alpha=0.3, axis='y')
    fig.suptitle('S021 3D Point Delivery — energy breakdown (time-optimal plan per layer)')
    fig.tight_layout()
    fig.savefig(os.path.join(OUTPUT_DIR, 'energy_breakdown.png'), dpi=130)
    plt.close(fig)


def save_animation(res, step=4):
    logs = [res[k + "_sims"][res[k + "_iopt"]] for k in ("card", "long")]
    n = max(len(s["t"]) for s in logs)
    n_frames = int(np.ceil((n - 1) / step)) + 1
    fig = plt.figure(figsize=(12, 5.5))
    axes, trails, dots, titles = [], [], [], []
    for k, (key, s) in enumerate(zip(("card", "long"), logs)):
        ax = fig.add_subplot(1, 2, k + 1, projection='3d')
        mis = res[key]
        ax.scatter(*mis.depot, color='royalblue', marker='s', s=60)
        ax.scatter(*mis.goal, color='limegreen', marker='*', s=200)
        p = s["p"]
        ax.set_xlim(p[:, 0].min() - 0.5, p[:, 0].max() + 0.5)
        ax.set_ylim(p[:, 1].min() - 0.5, p[:, 1].max() + 0.5)
        ax.set_zlim(0, max(2.5, p[:, 2].max() + 0.5))
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        tr = ax.scatter([], [], [], c=[], cmap='plasma', vmin=0, vmax=6.0, s=6)
        dot, = ax.plot([], [], [], 'o', color='red', ms=8)
        axes.append(ax)
        trails.append(tr)
        dots.append(dot)
        titles.append(ax.set_title(''))
    fig.colorbar(trails[1], ax=axes, label='altitude (m)', shrink=0.6)

    def update(fr):
        i = min(fr * step, n - 1)
        for k, (key, s) in enumerate(zip(("card", "long"), logs)):
            j = min(i, len(s["t"]) - 1)
            p = s["p"][:j + 1]
            trails[k]._offsets3d = (p[:, 0], p[:, 1], p[:, 2])
            trails[k].set_array(p[:, 2])
            dots[k].set_data([p[-1, 0]], [p[-1, 1]])
            dots[k].set_3d_properties([p[-1, 2]])
            titles[k].set_text(f"{res[key].name}: t = {s['t'][j]:.2f} s, {s['phase'][j].lower()}")
        return trails + dots + titles

    anim = FuncAnimation(fig, update, frames=n_frames, interval=1000 / 15, blit=False)
    anim.save(os.path.join(OUTPUT_DIR, 'animation.gif'), writer=PillowWriter(fps=15), dpi=80)
    plt.close(fig)
    print(f"Animation: {n_frames} frames (every {step} steps of {n} logged samples), 15 fps")


if __name__ == '__main__':
    res = run_simulation()
    plot_trajectory_3d(res)
    plot_telemetry(res)
    plot_altitude_sweep(res)
    plot_descent_profile(res)
    plot_wind_field(res)
    plot_energy_breakdown(res)
    save_animation(res)
    print("Outputs written to outputs/02_logistics_delivery/3d/s021_3d_point_delivery/")
