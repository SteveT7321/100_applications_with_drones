"""
S099 Cross-Obstacle Relay Pass
==============================
Two drones carry a virtual baton through a random 5-gate course with pillar obstacles. Drone A
starts at (0, 0, 1) m and threads gates 1-3; near the gate-3 exit it hands the baton to drone B
mid-air (|dp| <= 0.10 m, |dv| <= 0.15 m/s); B threads gates 4-5 to the finish.

Planning: RRT* (card gains) between consecutive mandatory waypoints (gate entry / centre / exit
points along the gate normal), greedy shortcutting, then a degree-7 polynomial per segment
(position, velocity given; acceleration and jerk zero at the knots), time-scaled to respect
v_max = 3 m/s and 80 % of a_max. Tracking: point-mass PD + feedforward, |a| <= 5 m/s^2, speed
<= 3 m/s, Gaussian acceleration disturbance.

Compared on the same courses and the same disturbance draws (CRN):
  1. Single drone       - A flies all five gates (reference time).
  2. Relay, matched     - B launches from a staging mark T_lead = 2.5 s before the handoff with a
                          min-snap intercept to A's planned state (card), then flies the rest.
  3. Relay, hover       - A stops at the handoff point, B (hovering there) takes over from rest.
  4. Relay, matched, RRT- same as 2 but the planner does not rewire (plain RRT).

Usage:
    conda activate drones
    python src/05_special_entertainment/s099_obstacle_relay.py
"""

import os
import math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401

# ── Parameters (card) ─────────────────────────────────────────────────────────
R_GATE      = 0.4
N_GATES     = 5
N_ITER      = 3000          # RRT* iterations per course (split over the free segments)
P_BIAS      = 0.2
STEP        = 0.20
GAMMA       = 2.5
LAMBDA      = 50.0
EPS_P       = 0.10
EPS_V       = 0.15
V_MAX       = 3.0
A_MAX       = 5.0
T_LEAD      = 2.5

# ── Parameters (added) ────────────────────────────────────────────────────────
START       = np.array([0.0, 0.0, 1.0])
FINISH      = np.array([14.0, 0.0, 2.0])
BOUNDS      = np.array([[-1.0, 15.0], [-5.0, 5.0], [0.3, 5.0]])
D_APPROACH  = 0.6           # m  entry / exit points along the gate normal
N_PILLARS   = 6
R_PILLAR    = 0.35
CLEAR       = 0.30          # m  planning clearance from pillars
FRAME_W     = 0.30          # m  gate frame width outside the opening (obstacle)
V_CRUISE    = 2.0           # m/s knot speed for the polynomial
A_PLAN      = 0.8 * A_MAX   # planned peak acceleration (tracking margin)
KP, KD      = 8.0, 4.0      # tracking gains (as in S096)
SIGMA_D     = 0.3           # m/s^2 acceleration disturbance
DT          = 0.02
T_AFTER_G3  = 1.0           # s  handoff this long after the planned gate-3 crossing
T_HO_HOLD   = 0.2           # s  handoff conditions must hold continuously this long
N_COURSES   = 20
DEMO_SEED   = 3
STRATS      = ['single', 'matched', 'hover', 'matched_rrt']
LABELS      = {'single': 'Single drone (no relay)', 'matched': 'Relay, velocity-matched',
               'hover': 'Relay, hover handoff', 'matched_rrt': 'Relay, matched, plain RRT'}
SCOL        = {'single': '#7f7f7f', 'matched': '#d62728', 'hover': '#1f77b4',
               'matched_rrt': '#ff7f0e'}

OUTPUT_DIR = os.path.normpath(os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '05_special_entertainment', 's099_obstacle_relay'))


# ── Course ────────────────────────────────────────────────────────────────────

def make_course(seed):
    rng = np.random.default_rng(seed)
    while True:
        x = np.sort(rng.uniform(2.0, 12.0, N_GATES))
        if np.diff(x).min() >= 1.5:
            break
    g = np.column_stack([x, rng.uniform(-3, 3, N_GATES), rng.uniform(1, 4, N_GATES)])
    normals = []
    for _ in range(N_GATES):
        while True:
            n = rng.standard_normal(3)
            n /= np.linalg.norm(n)
            if abs(n[2]) > 0.5:          # card: |z-component| > 0.5 (sign chosen below)
                break
        normals.append(n)
    normals = np.array(normals)
    pts = np.vstack([START, g, FINISH])
    for k in range(N_GATES):              # fly through along the course direction
        d = pts[k + 2] - pts[k]
        if normals[k] @ d < 0:
            normals[k] = -normals[k]
    entry = g - D_APPROACH * normals
    exit_ = g + D_APPROACH * normals
    keep = np.vstack([START, FINISH, g, entry, exit_])
    pillars = []
    while len(pillars) < N_PILLARS:
        c = np.array([rng.uniform(1, 13), rng.uniform(-3.5, 3.5)])
        if np.min(np.linalg.norm(keep[:, :2] - c, axis=1)) > R_PILLAR + CLEAR + 0.5:
            pillars.append(c)
    return dict(g=g, n=normals, entry=entry, exit=exit_, pillars=np.array(pillars))


def gate_basis(n):
    a = np.array([1.0, 0, 0]) if abs(n[0]) < 0.9 else np.array([0, 1.0, 0])
    u = np.cross(n, a); u /= np.linalg.norm(u)
    return u, np.cross(n, u)


def seg_free(p, q, course, margin=CLEAR):
    """Collision check of segment p-q against pillars (+margin), gate frames and bounds."""
    L = np.linalg.norm(q - p)
    n = max(2, int(L / 0.05) + 1)
    s = np.linspace(0, 1, n)[:, None]
    pts = p + s * (q - p)
    if np.any(pts[:, 2] < BOUNDS[2, 0]) or np.any(pts[:, 2] > BOUNDS[2, 1]):
        return False
    d = np.linalg.norm(pts[:, None, :2] - course['pillars'][None, :, :2], axis=2)
    if np.any(d < R_PILLAR + margin):
        return False
    for g, nn in zip(course['g'], course['n']):
        a, b = (p - g) @ nn, (q - g) @ nn
        if a * b < 0:
            c = p + (a / (a - b)) * (q - p)
            r = np.linalg.norm(c - g)
            if R_GATE - 0.1 < r < R_GATE + FRAME_W:
                return False
    return True


# ── RRT* ──────────────────────────────────────────────────────────────────────

def rrt_star(p, q, course, n_iter, rng, rewire=True):
    """Grow a tree from p toward q; returns (path list, nodes array)."""
    nodes = [p.copy()]
    parent = [-1]
    cost = [0.0]
    goal_idx = None
    lo, hi = BOUNDS[:, 0], BOUNDS[:, 1]
    for _ in range(n_iter):
        if rng.random() < P_BIAS:
            qr = q + rng.standard_normal(3) * 0.3
        else:
            qr = lo + rng.random(3) * (hi - lo)
        N = np.array(nodes)
        d = np.linalg.norm(N - qr, axis=1)
        i_near = int(np.argmin(d))
        step = qr - N[i_near]
        L = np.linalg.norm(step)
        if L < 1e-9:
            continue
        qn = N[i_near] + step * min(1.0, STEP / L)
        if not seg_free(N[i_near], qn, course):
            continue
        n = len(nodes)
        best, cbest = i_near, cost[i_near] + np.linalg.norm(qn - N[i_near])
        near = []
        if rewire:
            r = min(GAMMA * (math.log(n + 1) / (n + 1)) ** (1 / 3), 1.5)
            near = np.where(np.linalg.norm(N - qn, axis=1) <= r)[0]
            for j in near:
                c = cost[j] + np.linalg.norm(qn - N[j])
                if c < cbest - 1e-12 and seg_free(N[j], qn, course):
                    best, cbest = j, c
        nodes.append(qn); parent.append(best); cost.append(cbest)
        if rewire:
            for j in near:
                c = cbest + np.linalg.norm(N[j] - qn)
                if c < cost[j] - 1e-12 and seg_free(qn, N[j], course):
                    parent[j] = n
                    cost[j] = c
        if np.linalg.norm(qn - q) <= STEP and seg_free(qn, q, course):
            c = cbest + np.linalg.norm(q - qn)
            if goal_idx is None or c < cost[goal_idx]:
                if goal_idx is None:
                    nodes.append(q.copy()); parent.append(n); cost.append(c)
                    goal_idx = len(nodes) - 1
                else:
                    parent[goal_idx] = n
                    cost[goal_idx] = c
    if goal_idx is None:
        return None, np.array(nodes)
    path, i = [], goal_idx
    seen = set()
    while i != -1 and i not in seen:
        seen.add(i)
        path.append(nodes[i]); i = parent[i]
    return path[::-1], np.array(nodes)


def shortcut(path, course):
    out, i = [path[0]], 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and not seg_free(path[i], path[j], course):
            j -= 1
        out.append(path[j]); i = j
    return out


def plan_course(course, seed, rewire=True):
    """Waypoints through all gates; returns waypoints, flags for gate centres, tree nodes."""
    rng = np.random.default_rng(seed)
    mand = [START]
    for k in range(N_GATES):
        mand += [course['entry'][k], course['g'][k], course['exit'][k]]
    mand.append(FINISH)
    wps, is_gate, trees, raw_len = [START.copy()], [-1], [], 0.0
    free_pairs = [(0, 1)] + [(3 * k + 3, 3 * k + 4) for k in range(N_GATES)]
    per = N_ITER // len(free_pairs)
    i = 0
    while i < len(mand) - 1:
        a, b = mand[i], mand[i + 1]
        if (i, i + 1) in free_pairs:
            path, nodes = rrt_star(a, b, course, per, rng, rewire)
            trees.append(nodes)
            if path is None:
                return None
            raw_len += sum(np.linalg.norm(np.diff(np.array(path), axis=0), axis=1))
            path = shortcut(path, course)
            for pnt in path[1:]:
                wps.append(pnt); is_gate.append(-1)
        else:                                   # entry -> centre -> exit along the normal
            wps.append(b.copy())
            raw_len += np.linalg.norm(b - a)
            k = (i - 1) // 3
            is_gate.append(k if np.allclose(b, course['g'][k]) else -1)
        i += 1
    return dict(wps=np.array(wps), gate_idx=np.array(is_gate), trees=trees, raw_len=raw_len)


# ── Polynomial trajectory ─────────────────────────────────────────────────────

def _row(s, d):
    r = np.zeros(8)
    for k in range(d, 8):
        r[k] = math.prod(range(k - d + 1, k + 1)) * s ** (k - d)
    return r


_MINV = np.linalg.inv(np.array([_row(0, 0), _row(0, 1), _row(0, 2), _row(0, 3),
                                _row(1, 0), _row(1, 1), _row(1, 2), _row(1, 3)]))
_FAC = np.array([[math.prod(range(k - d + 1, k + 1)) if k >= d else 0.0 for k in range(8)]
                 for d in range(3)])
_POW = np.array([[max(k - d, 0) for k in range(8)] for d in range(3)])


def seg_coeffs(p0, v0, p1, v1, T, a0=None, j0=None):
    b = np.zeros((3, 8))
    b[:, 0], b[:, 1], b[:, 4], b[:, 5] = p0, v0 * T, p1, v1 * T
    if a0 is not None:
        b[:, 2], b[:, 3] = a0 * T ** 2, j0 * T ** 3
    return b @ _MINV.T


class Traj:
    """Piecewise degree-7 trajectory; eval(t) -> p, v, a (clamped at the ends)."""

    def __init__(self, segs, t0=0.0):
        self.segs = segs          # list of (coeffs 3x8, T)
        self.t0 = t0
        self.tk = t0 + np.concatenate([[0.0], np.cumsum([T for _, T in segs])])

    @property
    def t_end(self):
        return self.tk[-1]

    def eval(self, t):
        if t <= self.tk[0]:
            c, T = self.segs[0]; s = 0.0
        elif t >= self.tk[-1]:
            c, T = self.segs[-1]; s = 1.0
        else:
            i = int(np.searchsorted(self.tk, t, side='right') - 1)
            c, T = self.segs[i]; s = (t - self.tk[i]) / T
        rows = _FAC * s ** _POW
        o = rows @ c.T
        return o[0], o[1] / T, o[2] / T ** 2

    def sample(self, dt=0.02):
        ts = np.arange(self.tk[0], self.tk[-1] + 1e-9, dt)
        P = np.array([self.eval(t)[0] for t in ts])
        V = np.array([self.eval(t)[1] for t in ts])
        A = np.array([self.eval(t)[2] for t in ts])
        return ts, P, V, A


def knot_velocities(wps, gate_idx, course, v_end=0.0, v_start=0.0):
    n = len(wps)
    V = np.zeros_like(wps)
    for i in range(1, n - 1):
        k = gate_idx[i]
        if k >= 0:
            d = course['n'][k]
        elif i + 1 < n and gate_idx[i + 1] >= 0:
            d = course['n'][gate_idx[i + 1]]          # entry point: along the normal
        elif gate_idx[i - 1] >= 0:
            d = course['n'][gate_idx[i - 1]]          # exit point: along the normal
        else:
            d = wps[i + 1] - wps[i - 1]
            d = d / np.linalg.norm(d)
        V[i] = V_CRUISE * d
    if v_start is not None and np.ndim(v_start):
        V[0] = v_start
    return V


def build_traj(wps, V, t0=0.0, a0=None, j0=None, course=None):
    """Polynomial through the waypoints. If a segment bulges below 0.3 m or into a pillar
    (+0.1 m), the knot speeds at both of its ends are reduced by 30 % and all are rebuilt."""
    V = V.copy()
    for _ in range(12):
        tr = _build(wps, V, t0, a0, j0)
        if course is None:
            return tr
        bad = []
        for i, (c, T) in enumerate(tr.segs):
            s = np.linspace(0, 1, 41)
            P = np.array([(_FAC[0] * x ** _POW[0]) @ c.T for x in s])
            d = np.linalg.norm(P[:, None, :2] - course['pillars'][None], axis=2)
            if P[:, 2].min() < BOUNDS[2, 0] or d.min() < R_PILLAR + 0.1:
                bad.append(i)
        if not bad:
            return tr
        for i in bad:
            V[i] *= 0.7
            V[i + 1] *= 0.7
    return tr


def _build(wps, V, t0=0.0, a0=None, j0=None):
    segs = []
    for i in range(len(wps) - 1):
        L = np.linalg.norm(wps[i + 1] - wps[i])
        T = max(L / V_CRUISE, 0.3)
        for _ in range(60):
            c = seg_coeffs(wps[i], V[i], wps[i + 1], V[i + 1], T,
                           a0 if i == 0 else None, j0 if i == 0 else None)
            s = np.linspace(0, 1, 41)
            vv = np.array([(_FAC[1] * x ** _POW[1]) @ c.T for x in s]) / T
            aa = np.array([(_FAC[2] * x ** _POW[2]) @ c.T for x in s]) / T ** 2
            if np.linalg.norm(vv, axis=1).max() <= V_MAX and \
                    np.linalg.norm(aa, axis=1).max() <= A_PLAN:
                break
            T *= 1.08
        segs.append((c, T))
    return Traj(segs, t0)


def gate_times(traj, course):
    """Planned crossing time of every gate plane (first crossing near the gate)."""
    ts, P, _, _ = traj.sample(0.01)
    out = []
    for g, n in zip(course['g'], course['n']):
        s = (P - g) @ n
        idx = np.where((s[:-1] < 0) & (s[1:] >= 0) &
                       (np.linalg.norm(P[:-1] - g, axis=1) < 1.5))[0]
        out.append(ts[idx[0]] if len(idx) else np.nan)
    return np.array(out)


# ── Mission simulation ────────────────────────────────────────────────────────

def track(p, v, ref, noise):
    pr, vr, ar = ref
    a = KP * (pr - p) + KD * (vr - v) + ar
    n = np.linalg.norm(a)
    if n > A_MAX:
        a *= A_MAX / n
    v = v + (a + noise) * DT
    s = np.linalg.norm(v)
    if s > V_MAX:
        v *= V_MAX / s
    return p + v * DT, v


def run_simulation(course, plan, strategy, seed):
    """Fly one mission. Returns metrics and logs."""
    rng = np.random.default_rng(seed)
    wps, gidx = plan['wps'], plan['gate_idx']
    full = build_traj(wps, knot_velocities(wps, gidx, course), course=course)
    tg = gate_times(full, course)
    t_h = tg[2] + T_AFTER_G3
    n_max = int((full.t_end + 25.0) / DT)
    noise = rng.normal(0, SIGMA_D, (n_max, 2, 3))
    pA, vA = START.copy(), np.zeros(3)
    if strategy == 'single':
        trajA, trajB, pB = full, None, None
    else:
        pH, vH, _ = full.eval(t_h)
        if strategy in ('matched', 'matched_rrt'):
            # min-snap from rest at the staging mark to A's planned handoff state (card)
            # symmetric profile: start v T/2 behind the handoff point (kept 0.8-4.5 m high)
            stage = pH - vH * T_LEAD / 2
            stage[2] = min(max(stage[2], 0.8), 4.5)
            cB = seg_coeffs(stage, np.zeros(3), pH, vH, T_LEAD)
            launch = Traj([(cB, T_LEAD)], t0=t_h - T_LEAD)
            trajA = full
            pB = stage.copy()
            trajB = (launch, full)
        else:  # hover handoff: A stops at pH; B waits there and restarts from rest
            k_split = int(np.argmin(np.abs(full.tk - t_h)))
            k_split = max(k_split, int(np.where(gidx == 2)[0][0]) + 2)
            pH = wps[k_split]
            Va = knot_velocities(wps[:k_split + 1], gidx[:k_split + 1], course)
            Va[-1] = 0.0
            trajA = build_traj(wps[:k_split + 1], Va, course=course)
            Vb = knot_velocities(wps[k_split:], gidx[k_split:], course)
            Vb[0] = 0.0
            trajB = ('hover', wps[k_split:], Vb)
            pB = pH.copy()
    vB = np.zeros(3)
    holder, window, t_pass, ho = 'A', 0.0, None, None
    trajB_active = None
    log = dict(t=[], pA=[], pB=[], holder=[])
    t_finish = None
    for k in range(n_max):
        t = k * DT
        # drone A
        if holder == 'A' or strategy == 'single':
            refA = trajA.eval(t)
        else:
            refA = (pA_hold, np.zeros(3), np.zeros(3))
        pA, vA = track(pA, vA, refA, noise[k, 0])
        # drone B
        if strategy != 'single':
            if strategy in ('matched', 'matched_rrt'):
                launch, rest = trajB
                if t < launch.t0:
                    refB = (launch.eval(launch.t0)[0], np.zeros(3), np.zeros(3))
                elif t < t_h:
                    refB = launch.eval(t)
                else:
                    refB = rest.eval(t)
            else:
                if trajB_active is None:
                    refB = (pH, np.zeros(3), np.zeros(3))
                else:
                    refB = trajB_active.eval(t)
            pB, vB = track(pB, vB, refB, noise[k, 1])
        assert pA[2] > 0 and (pB is None or pB[2] > 0), (strategy, t, pA, pB, holder)
        # handoff check
        if strategy != 'single' and holder == 'A':
            dp, dv = np.linalg.norm(pA - pB), np.linalg.norm(vA - vB)
            if strategy in ('matched', 'matched_rrt') and t < t_h - 0.5:
                ok = False
            else:
                ok = dp <= EPS_P and dv <= EPS_V
            window = window + DT if ok else 0.0
            if window >= T_HO_HOLD - 1e-9:
                holder, t_pass = 'B', t + DT
                ho = dict(t=t_pass, dp=dp, dv=dv, p=pB.copy())
                pA_hold = pA.copy()
                if strategy == 'hover':
                    _, wl, Vb = trajB
                    trajB_active = build_traj(wl, Vb, t0=t + DT, course=course)
            elif strategy in ('matched', 'matched_rrt') and t > t_h + 1.0:
                break                                    # handoff failed
            elif strategy == 'hover' and t > trajA.t_end + 10.0:
                break
        log['t'].append(t + DT); log['pA'].append(pA.copy())
        log['pB'].append(pB.copy() if pB is not None else pA.copy()); log['holder'].append(holder)
        carrier = pA if (holder == 'A') else pB
        if np.linalg.norm(carrier - FINISH) <= 0.3 and \
                (strategy == 'single' or holder == 'B'):
            t_finish = t + DT
            break
    for key in ('t', 'pA', 'pB'):
        log[key] = np.array(log[key])
    log['holder'] = np.array(log['holder'])
    # gate passage of the baton carrier (A for gates 1-3, B for 4-5; A for all if single)
    miss, crossed = np.full(N_GATES, np.nan), np.zeros(N_GATES, bool)
    for gk in range(N_GATES):
        P = log['pA'] if (strategy == 'single' or gk < 3) else log['pB']
        g, n = course['g'][gk], course['n'][gk]
        s = (P - g) @ n
        idx = np.where((s[:-1] < 0) & (s[1:] >= 0))[0]
        best = np.inf
        for i in idx:
            c = P[i] + (s[i] / (s[i] - s[i + 1])) * (P[i + 1] - P[i])
            best = min(best, np.linalg.norm(c - g))
        if np.isfinite(best):
            miss[gk], crossed[gk] = best, True
    clear = min(np.linalg.norm(log['pA'][:, None, :2] - course['pillars'][None], axis=2).min(),
                np.linalg.norm(log['pB'][:, None, :2] - course['pillars'][None], axis=2).min()
                if strategy != 'single' else np.inf) - R_PILLAR
    pen = np.nansum(np.maximum(0.0, np.nan_to_num(miss, nan=10.0) - R_GATE) ** 2)
    res = dict(t_finish=t_finish, handoff=ho, miss=miss, crossed=crossed,
               gates_ok=int(np.sum(crossed & (miss <= R_GATE))), clearance=clear,
               J=(t_finish if t_finish else np.nan) + LAMBDA * pen, log=log, plan_t=full.t_end,
               t_h=t_h, tg=tg)
    res['success'] = (t_finish is not None and res['gates_ok'] == N_GATES and clear > 0
                      and (strategy == 'single' or ho is not None))
    res['len_A'] = float(np.sum(np.linalg.norm(np.diff(log['pA'], axis=0), axis=1)))
    res['len_B'] = float(np.sum(np.linalg.norm(np.diff(log['pB'], axis=0), axis=1))) \
        if strategy != 'single' else 0.0
    return res


def run_course(seed, record=False):
    course = make_course(seed)
    plans = {'rrtstar': plan_course(course, seed, rewire=True),
             'rrt': plan_course(course, seed, rewire=False)}
    out = {}
    for s in STRATS:
        plan = plans['rrt' if s == 'matched_rrt' else 'rrtstar']
        if plan is None:
            out[s] = None
            continue
        out[s] = run_simulation(course, plan, s, seed=10_000 + seed)
        if not record:
            out[s].pop('log')
    return course, plans, out


# ── Plots ─────────────────────────────────────────────────────────────────────

def draw_gate(ax, g, n, color='grey', lw=2.0):
    u, w = gate_basis(n)
    a = np.linspace(0, 2 * np.pi, 60)
    ring = g + R_GATE * (np.outer(np.cos(a), u) + np.outer(np.sin(a), w))
    ax.plot(ring[:, 0], ring[:, 1], ring[:, 2], color=color, lw=lw)


def draw_pillars(ax, pillars, top=5.0):
    a = np.linspace(0, 2 * np.pi, 24)
    for c in pillars:
        for z in (0.0, top):
            ax.plot(c[0] + R_PILLAR * np.cos(a), c[1] + R_PILLAR * np.sin(a), z, color='grey',
                    lw=0.8)
        for ang in a[::4]:
            ax.plot([c[0] + R_PILLAR * np.cos(ang)] * 2, [c[1] + R_PILLAR * np.sin(ang)] * 2,
                    [0, top], color='grey', lw=0.5, alpha=0.6)


def plot_course(course, plans, res, out_dir):
    r = res['matched']
    log = r['log']
    fig = plt.figure(figsize=(18, 8))
    for i, (elev, azim, ttl) in enumerate([(25, -60, 'isometric'), (89, -90, 'top-down')]):
        ax = fig.add_subplot(1, 2, i + 1, projection='3d')
        for tr in plans['rrtstar']['trees']:
            ax.scatter(tr[:, 0], tr[:, 1], tr[:, 2], s=1, c='lightgrey', alpha=0.4)
        draw_pillars(ax, course['pillars'])
        for k in range(N_GATES):
            ok = r['crossed'][k] and r['miss'][k] <= R_GATE
            draw_gate(ax, course['g'][k], course['n'][k], 'green' if ok else 'grey')
            ax.text(*(course['g'][k] + np.array([0, 0, 0.6])), f'G{k + 1}', fontsize=9)
        mA = log['holder'] == 'A'
        ax.plot(log['pA'][:, 0], log['pA'][:, 1], log['pA'][:, 2], 'r-', lw=1.6,
                label='Drone A')
        ax.plot(log['pB'][:, 0], log['pB'][:, 1], log['pB'][:, 2], 'b-', lw=1.6,
                label='Drone B')
        if r['handoff']:
            ax.scatter(*r['handoff']['p'], marker='*', s=300, c='gold', edgecolors='k',
                       label='Handoff', zorder=10)
        ax.scatter(*START, marker='^', s=80, c='k', label='Start (A)')
        ax.scatter(*log['pB'][0], marker='s', s=60, c='b', label='Staging (B)')
        ax.scatter(*FINISH, marker='*', s=200, c='green', label='Finish')
        ax.set_xlim(-1, 15); ax.set_ylim(-5, 5); ax.set_zlim(0, 5)
        ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)'); ax.set_zlabel('z (m)')
        ax.view_init(elev=elev, azim=azim)
        ax.set_title(f'Course seed {DEMO_SEED}, velocity-matched relay ({ttl}); '
                     'grey dots = RRT* trees, rings = gates (green = passed)', fontsize=9)
        if i == 0:
            ax.legend(fontsize=8, loc='upper left')
    fig.tight_layout()
    path = os.path.join(out_dir, 'course_overview.png')
    fig.savefig(path, dpi=120, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_gate_miss(res, out_dir):
    fig, ax = plt.subplots(figsize=(9, 4.8))
    w = 0.2
    for j, s in enumerate(STRATS):
        m = res[s]['miss']
        cols = [SCOL[s] if (np.isfinite(x) and x <= R_GATE) else 'red' for x in m]
        ax.bar(np.arange(N_GATES) + (j - 1.5) * w, np.nan_to_num(m, nan=0.0), w, color=cols,
               edgecolor='k', lw=0.5, label=LABELS[s])
    ax.axhline(R_GATE, color='k', ls='--', label=f'$r_{{gate}}$ = {R_GATE} m')
    ax.set_xticks(range(N_GATES)); ax.set_xticklabels([f'Gate {k + 1}' for k in range(N_GATES)])
    ax.set_ylabel('Distance from gate centre at plane crossing (m)')
    ax.set_title(f'Gate miss distance, course seed {DEMO_SEED} (red bar = outside the gate)')
    ax.legend(fontsize=8)
    fig.tight_layout()
    path = os.path.join(out_dir, 'gate_miss.png')
    fig.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_monte_carlo(mc, out_dir):
    fig, axes = plt.subplots(1, 3, figsize=(18, 5))
    ax = axes[0]
    data = [mc[s]['t'][np.isfinite(mc[s]['t'])] for s in STRATS]
    bp = ax.boxplot(data, tick_labels=[LABELS[s].replace(', ', ',\n') for s in STRATS],
                    patch_artist=True)
    for p, s in zip(bp['boxes'], STRATS):
        p.set_facecolor(SCOL[s])
    ax.set_ylabel('Mission time (s)'); ax.set_title(f'Mission time over {N_COURSES} courses')
    ax = axes[1]
    for s in STRATS:
        x = mc[s]['miss'].ravel()
        ax.hist(x[np.isfinite(x)], bins=np.linspace(0, 0.4, 41), histtype='step', lw=1.6,
                color=SCOL[s], label=LABELS[s])
    ax.axvline(R_GATE, color='k', ls='--')
    ax.set_xlabel('Gate miss distance (m)'); ax.set_ylabel('Gate passages')
    ax.set_title('Gate miss distance, all gates and courses'); ax.legend(fontsize=8)
    ax = axes[2]
    ax.scatter(mc['plan_len_rrt'], mc['plan_len_star'], c='k', s=20)
    lim = [min(mc['plan_len_star'].min(), mc['plan_len_rrt'].min()) - 1,
           max(mc['plan_len_star'].max(), mc['plan_len_rrt'].max()) + 1]
    ax.plot(lim, lim, 'k:', lw=1)
    ax.set_xlabel('Plain RRT raw path length (m)'); ax.set_ylabel('RRT* raw path length (m)')
    ax.set_title('Raw tree path length per course (below the diagonal = RRT* shorter)')
    fig.tight_layout()
    path = os.path.join(out_dir, 'monte_carlo.png')
    fig.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def gif_indices(n, step):
    return [min(i * step, n - 1) for i in range(math.ceil((n - 1) / step) + 1)]


def save_animation(course, res, out_dir, step=3):
    r = res['matched']
    log = r['log']
    n = len(log['t'])
    idx = gif_indices(n, step)
    fig = plt.figure(figsize=(9, 6.5))
    ax = fig.add_subplot(111, projection='3d')
    draw_pillars(ax, course['pillars'])
    rings = []
    for k in range(N_GATES):
        u, w = gate_basis(course['n'][k])
        a = np.linspace(0, 2 * np.pi, 40)
        ring = course['g'][k] + R_GATE * (np.outer(np.cos(a), u) + np.outer(np.sin(a), w))
        rings.append(ax.plot(ring[:, 0], ring[:, 1], ring[:, 2], color='grey', lw=2)[0])
    pass_t = []
    for k in range(N_GATES):
        P = log['pA'] if k < 3 else log['pB']
        s = (P - course['g'][k]) @ course['n'][k]
        ii = np.where((s[:-1] < 0) & (s[1:] >= 0))[0]
        ok = r['crossed'][k] and r['miss'][k] <= R_GATE
        pass_t.append(log['t'][ii[0]] if (len(ii) and ok) else np.inf)
    ax.scatter(*FINISH, marker='*', s=150, c='green')
    dA, = ax.plot([], [], [], 'o', color='red', ms=7)
    dB, = ax.plot([], [], [], 'o', color='blue', ms=7)
    tA, = ax.plot([], [], [], '-', color='red', lw=1)
    tB, = ax.plot([], [], [], '-', color='blue', lw=1)
    baton, = ax.plot([], [], [], 'D', ms=8, mec='k')
    ax.set_xlim(-1, 15); ax.set_ylim(-5, 5); ax.set_zlim(0, 5)
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)'); ax.set_zlabel('z (m)')
    ax.view_init(elev=25, azim=-60)

    def update(fi):
        k = idx[fi]
        t = log['t'][k]
        for j in range(N_GATES):
            rings[j].set_color('green' if t >= pass_t[j] else 'grey')
        for d, tr, P in ((dA, tA, log['pA']), (dB, tB, log['pB'])):
            d.set_data([P[k, 0]], [P[k, 1]]); d.set_3d_properties([P[k, 2]])
            lo = max(0, k - 80)
            tr.set_data(P[lo:k + 1, 0], P[lo:k + 1, 1]); tr.set_3d_properties(P[lo:k + 1, 2])
        h = log['holder'][k]
        P = log['pA'] if h == 'A' else log['pB']
        baton.set_data([P[k, 0]], [P[k, 1]]); baton.set_3d_properties([P[k, 2] + 0.35])
        baton.set_color('red' if h == 'A' else 'blue')
        ax.set_title(f'S099 obstacle relay (seed {DEMO_SEED})  t = {t:5.2f} s  baton: drone {h}')
        return [dA, dB, tA, tB, baton] + rings

    anim = animation.FuncAnimation(fig, update, frames=len(idx), interval=33, blit=False)
    path = os.path.join(out_dir, 'gate_relay_animation.gif')
    anim.save(path, writer='pillow', fps=30, dpi=65)
    plt.close(fig)
    print(f'Saved: {path}  ({len(idx)} frames, step {step}, {n} samples)')


# ── Main ──────────────────────────────────────────────────────────────────────

def describe(s, r):
    ho = r['handoff']
    hs = (f'handoff at {ho["t"]:.2f} s, |dp| {ho["dp"]:.3f} m, |dv| {ho["dv"]:.3f} m/s'
          if ho else 'no handoff')
    t = f'{r["t_finish"]:.2f} s' if r['t_finish'] else 'not finished'
    print(f'  {LABELS[s]:28s} time {t}, gates {r["gates_ok"]}/5, max miss '
          f'{np.nanmax(r["miss"]):.3f} m, J {r["J"]:.2f}, {hs}, path A {r["len_A"]:.1f} m, '
          f'path B {r["len_B"]:.1f} m, pillar clearance {r["clearance"]:.2f} m, '
          f'success {r["success"]}')


if __name__ == '__main__':
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    print('=== S099 Cross-Obstacle Relay Pass ===')
    print(f'Card T_total = path_a.duration + path_b.duration counts the overlap of the two '
          f'flights twice; the mission time used here is the time the baton reaches the finish.')
    print(f'Matched launch: rest -> v in T_lead = {T_LEAD} s needs a peak of 1.875 v / T_lead = '
          f'{1.875 * V_CRUISE / T_LEAD:.2f} m/s^2 at {V_CRUISE} m/s')

    course, plans, res = run_course(DEMO_SEED, record=True)
    print(f'\nDemo course (seed {DEMO_SEED}): gates at ' + ', '.join(
        f'({g[0]:.1f}, {g[1]:.1f}, {g[2]:.1f})' for g in course['g']))
    for key in ('rrtstar', 'rrt'):
        p = plans[key]
        print(f'  {key:7s}: {sum(len(t) for t in p["trees"])} tree nodes, raw path '
              f'{p["raw_len"]:.2f} m, {len(p["wps"])} waypoints after shortcutting')
    print(f'  Planned gate-3 crossing {res["matched"]["tg"][2]:.2f} s, handoff target '
          f'{res["matched"]["t_h"]:.2f} s, planned single-drone time '
          f'{res["single"]["plan_t"]:.2f} s')
    for s in STRATS:
        describe(s, res[s])

    print(f'\nMonte Carlo ({N_COURSES} random courses, same disturbance draws per course):')
    mc = {s: dict(t=[], miss=[], ok=[], ho_dp=[], ho_dv=[], J=[]) for s in STRATS}
    mc['plan_len_star'], mc['plan_len_rrt'] = [], []
    for c in range(N_COURSES):
        _, pl, rr = run_course(100 + c)
        mc['plan_len_star'].append(pl['rrtstar']['raw_len'] if pl['rrtstar'] else np.nan)
        mc['plan_len_rrt'].append(pl['rrt']['raw_len'] if pl['rrt'] else np.nan)
        for s in STRATS:
            r = rr[s]
            mc[s]['t'].append(r['t_finish'] if (r and r['t_finish']) else np.nan)
            mc[s]['miss'].append(r['miss'] if r else np.full(N_GATES, np.nan))
            mc[s]['ok'].append(bool(r and r['success']))
            mc[s]['J'].append(r['J'] if r else np.nan)
            ho = r['handoff'] if r else None
            mc[s]['ho_dp'].append(ho['dp'] if ho else np.nan)
            mc[s]['ho_dv'].append(ho['dv'] if ho else np.nan)
    for key in ('plan_len_star', 'plan_len_rrt'):
        mc[key] = np.array(mc[key])
    for s in STRATS:
        for key in mc[s]:
            mc[s][key] = np.array(mc[s][key])
        m = mc[s]
        line = (f'  {LABELS[s]:28s} success {m["ok"].sum()}/{N_COURSES}, time '
                f'{np.nanmean(m["t"]):.2f} ± {np.nanstd(m["t"]):.2f} s, gates passed '
                f'{int(np.sum(m["miss"] <= R_GATE))}/{N_GATES * N_COURSES}, max miss '
                f'{np.nanmax(m["miss"]):.3f} m, mean miss {np.nanmean(m["miss"]):.3f} m')
        if s != 'single':
            line += (f', handoff |dp| {np.nanmean(m["ho_dp"]):.3f} m, |dv| '
                     f'{np.nanmean(m["ho_dv"]):.3f} m/s')
        print(line)
    d_mh = mc['hover']['t'] - mc['matched']['t']
    d_ms = mc['matched']['t'] - mc['single']['t']
    print(f'  Hover - matched time: {np.nanmean(d_mh):.2f} ± {np.nanstd(d_mh):.2f} s '
          f'(matched faster in {int(np.sum(d_mh > 0))}/{N_COURSES})')
    print(f'  Matched - single time: {np.nanmean(d_ms):+.2f} ± {np.nanstd(d_ms):.2f} s')
    d_rr = mc['matched_rrt']['t'] - mc['matched']['t']
    print(f'  Plain RRT - RRT* (matched relay) time: {np.nanmean(d_rr):+.2f} ± '
          f'{np.nanstd(d_rr):.2f} s (RRT* faster in {int(np.sum(d_rr > 0))}/{N_COURSES}); raw '
          f'path RRT* {np.nanmean(mc["plan_len_star"]):.2f} m vs RRT '
          f'{np.nanmean(mc["plan_len_rrt"]):.2f} m (RRT* shorter in '
          f'{int(np.sum(mc["plan_len_star"] < mc["plan_len_rrt"]))}/{N_COURSES})')

    print()
    plot_course(course, plans, res, OUTPUT_DIR)
    plot_gate_miss(res, OUTPUT_DIR)
    plot_monte_carlo(mc, OUTPUT_DIR)
    save_animation(course, res, OUTPUT_DIR)
