"""
S090 Racing Drone Optimal Path
==============================
A racing drone plans a degree-7 minimum-snap trajectory through 6 gates, each with a required
passage velocity (5 m/s along the gate heading). Three time allocations are compared:

  1. Fixed time        - segment durations proportional to distance, total 10 s (baseline).
  2. Redistributed     - projected gradient descent on the snap cost J with the total kept at 10 s.
  3. J + lambda T      - gradient descent on J + lambda * T_total (lambda = 0.5).

The planned trajectories pass every gate exactly by construction, so the comparison is made by
FLYING them: each plan is tracked by the 6-DOF rigid-body model (src/base/rigid_body_drone.py,
fpv_5inch preset) with a flatness feed-forward cascade (position / velocity / attitude / rate).
The flown gate miss distance, heading error, shaft energy and rotor activity test whether lower
snap really means an easier, cheaper flight. The point-mass reference itself is the baseline:
an ideal point mass would fly every plan with zero miss. A 20-layout robustness study (gates
jittered by up to 0.5 m, seeds 0-19) checks that the conclusions are not specific to one course.

Usage:
    conda activate drones
    python src/05_special_entertainment/s090_racing_optimal.py
"""

import os
import sys
from math import factorial
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'base'))
from rigid_body_drone import (RigidBodyDrone, make_params, default_gains,  # noqa: E402
                              quat_to_rot, tilt_angle, G)

# ── Parameters (card) ─────────────────────────────────────────────────────────
N_GATES     = 6
R_GATE      = 0.3            # m
R_TOL       = 0.2            # m
V_GATE      = 5.0            # m/s
V_MAX       = 12.0           # m/s
A_MAX       = 2 * G          # m/s^2  thrust-acceleration limit
T_TOTAL     = 10.0           # s
T_MIN       = 0.2            # s
ETA         = 1e-3           # gradient step size (card)
N_ITER      = 200
LAMBDA      = 0.5
DT_EVAL     = 0.02           # s
COS_HEADING = np.cos(np.radians(20.0))
GATE_POS = np.array([
    [5.0, 0.0, 1.5],
    [8.0, 4.0, 2.5],
    [5.0, 8.0, 1.8],
    [0.0, 6.0, 2.2],
    [-3.0, 2.0, 1.5],
    [0.0, 0.0, 1.0],
])
_raw = np.array([
    [0.5, 1.0, 0.1],
    [-0.5, 1.0, 0.2],
    [-1.0, -0.2, 0.0],
    [-0.8, -0.8, -0.1],
    [0.5, -0.8, 0.0],
    [1.0, 0.3, 0.0],
])
GATE_HEAD = _raw / np.linalg.norm(_raw, axis=1, keepdims=True)

# ── Parameters (added) ────────────────────────────────────────────────────────
DT_INNER    = 0.002          # s  rigid-body step (500 Hz)
T_PAD       = 1.0            # s  flown after the last gate (reference holds final velocity)
FD_EPS      = 1e-6           # s  central-difference step for dJ/dT
R_SEARCH    = 1.0            # m  a gate-plane crossing counts only this close to the centre
ARMIJO      = 1e-4           # sufficient-decrease constant of the backtracking line search
TRUST       = 0.2            # max relative change of any segment time per iteration
N_LAYOUTS   = 20             # robustness study
JITTER      = 0.5            # m  max gate displacement per axis in the robustness study
MAX_TILT    = np.radians(89.0)

STRATS = ['fixed', 'redist', 'joint']
ROB_FLOWN = ['fixed', 'redist']   # robustness flights (the J + lambda T laps are 4-5x longer)
LABEL = {'fixed': 'Fixed time', 'redist': 'Redistributed (T = 10 s)',
         'joint': r'J + $\lambda$T'}
LABEL_TXT = {'fixed': 'Fixed time', 'redist': 'Redistributed', 'joint': 'J + lambda*T'}
COLOR = {'fixed': 'tab:red', 'redist': 'tab:blue', 'joint': 'tab:purple'}

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), '..', '..',
                          'outputs', '05_special_entertainment', 's090_racing_optimal')


# ── Minimum-snap QP ───────────────────────────────────────────────────────────
_J = np.arange(8)
_FAC = np.array([[factorial(j) / factorial(j - r) if j >= r else 0.0 for j in range(8)]
                 for r in range(5)])                     # d^r/dt^r coefficient factors
_C4 = _FAC[4]
_EXP = _J[:, None] + _J[None, :] - 7
_QMASK = (_J[:, None] >= 4) & (_J[None, :] >= 4)
_QC = np.where(_QMASK, np.outer(_C4, _C4) / np.where(_QMASK, _EXP, 1), 0.0)


def _drow(t, r):
    """Row vector of d^r/dt^r [1, t, ..., t^7]."""
    return _FAC[r] * np.where(_J >= r, t ** np.maximum(_J - r, 0), 0.0)


def snap_Q(T):
    return _QC * np.where(_QMASK, T ** np.where(_QMASK, _EXP, 0), 0.0)


def plan(T, gates=GATE_POS, heads=GATE_HEAD):
    """Min-snap through the gates: position and velocity (V_GATE * heading) fixed at every gate,
    acceleration and jerk free but continuous (C3); start 2 m before gate 1 at V_GATE along its
    heading with zero acceleration and jerk. Returns coeffs (M, 8, 3) and J."""
    M = len(T)
    start = gates[0] - 2.0 * heads[0]
    wp = np.vstack([start, gates])
    vel = V_GATE * np.vstack([heads[0], heads])
    n = 8 * M
    H = np.zeros((n, n))
    for k in range(M):
        H[8 * k:8 * k + 8, 8 * k:8 * k + 8] = snap_Q(T[k])
    rows, rhs = [], []

    def add(parts, val):
        row = np.zeros(n)
        for k, v in parts:
            row[8 * k:8 * k + 8] += v
        rows.append(row)
        rhs.append(val)

    for k in range(M):
        add([(k, _drow(0.0, 0))], wp[k])
        add([(k, _drow(T[k], 0))], wp[k + 1])
        add([(k, _drow(0.0, 1))], vel[k])
        add([(k, _drow(T[k], 1))], vel[k + 1])
    for k in range(M - 1):
        for r in (2, 3):
            add([(k, _drow(T[k], r)), (k + 1, -_drow(0.0, r))], np.zeros(3))
    add([(0, _drow(0.0, 2))], np.zeros(3))
    add([(0, _drow(0.0, 3))], np.zeros(3))
    A = np.array(rows)
    b = np.array(rhs)
    m = A.shape[0]
    K = np.block([[2 * H, A.T], [A, np.zeros((m, m))]])
    c = np.linalg.solve(K, np.vstack([np.zeros((n, 3)), b]))[:n]
    J = float(sum(c[:, a] @ H @ c[:, a] for a in range(3)))
    return c.reshape(M, 8, 3), J


def eval_traj(coeffs, T, t, r=0):
    t = np.atleast_1d(t)
    cum = np.concatenate([[0.0], np.cumsum(T)])
    seg = np.clip(np.searchsorted(cum, t, side='right') - 1, 0, len(T) - 1)
    tau = t - cum[seg]
    out = np.zeros((len(t), 3))
    for j in range(r, 8):
        out += _FAC[r, j] * (tau ** (j - r))[:, None] * coeffs[seg, j]
    return out


def grad_J(T, **kw):
    g = np.zeros(len(T))
    for i in range(len(T)):
        e = np.zeros(len(T))
        e[i] = FD_EPS
        g[i] = (plan(T + e, **kw)[1] - plan(T - e, **kw)[1]) / (2 * FD_EPS)
    return g


def dominant_grad(T, coeffs):
    """The card's 'dominant' term c^T dQ/dT c (explicit T dependence only)."""
    g = np.zeros(len(T))
    for i, Ti in enumerate(T):
        dQ = np.zeros((8, 8))
        for j in range(4, 8):
            for k in range(4, 8):
                dQ[j, k] = (factorial(j) / factorial(j - 4) * factorial(k) / factorial(k - 4)
                            * Ti ** (j + k - 8))
        g[i] = sum(coeffs[i, :, a] @ dQ @ coeffs[i, :, a] for a in range(3))
    return g


def optimise(T0, mode, eta=ETA, n_iter=N_ITER, lam=LAMBDA, **kw):
    """Card's update T <- T - eta * grad (projected for 'redist'), with T >= T_MIN and, for
    'redist', renormalisation to the fixed total. The card's fixed eta overshoots by orders of
    magnitude (|dJ/dT| ~ 1e6), so each step starts at eta, is shrunk until no segment changes by
    more than TRUST x its duration, and is then halved until the Armijo sufficient-decrease
    condition f(T_new) <= f(T) - ARMIJO * step * |g|^2 holds."""
    T = T0.copy()
    total = T0.sum()

    def obj(Tx):
        J = plan(Tx, **kw)[1]
        return J + (lam * Tx.sum() if mode == 'joint' else 0.0), J

    f, J = obj(T)
    hist = [(0, J, T.sum(), f)]
    n_back = 0
    for it in range(1, n_iter + 1):
        g = grad_J(T, **kw)
        if mode == 'redist':
            g = g - g.mean()
        else:
            g = g + lam
        step = min(eta, TRUST * np.min(T / np.maximum(np.abs(g), 1e-12)))
        while True:
            Tn = np.maximum(T - step * g, T_MIN)
            if mode == 'redist':
                Tn = Tn / Tn.sum() * total
            fn, Jn = obj(Tn)
            if fn <= f - ARMIJO * step * float(g @ g) or step < 1e-14:
                break
            step *= 0.5
            n_back += 1
        T, f, J = Tn, fn, Jn
        hist.append((it, J, T.sum(), f))
    return T, np.array(hist), n_back


# ── Flight with the rigid-body model ──────────────────────────────────────────
def reference(coeffs, T, dt):
    t_end = T.sum()
    t = np.arange(0.0, t_end + T_PAD + dt / 2, dt)
    tq = np.minimum(t, t_end)
    p = eval_traj(coeffs, T, tq, 0)
    v = eval_traj(coeffs, T, tq, 1)
    a = eval_traj(coeffs, T, tq, 2)
    j = eval_traj(coeffs, T, tq, 3)
    after = t > t_end                         # coast at the final gate velocity
    p[after] = p[after] + v[after] * (t[after] - t_end)[:, None]
    a[after] = 0.0
    j[after] = 0.0
    return dict(t=t, p=p, v=v, a=a, j=j, T=t_end)


def fly(refs):
    """Fly the references (one per batch row) with the flatness feed-forward cascade."""
    B = len(refs)
    L = max(len(r['t']) for r in refs)
    pad = {k: np.stack([np.concatenate([r[k], np.repeat(r[k][-1:], L - len(r[k]), 0)])
                        for r in refs], axis=1) for k in ('p', 'v', 'a', 'j')}
    params = make_params('fpv_5inch')
    gains = default_gains('fpv_5inch', max_tilt=MAX_TILT)
    start = pad['p'][0]
    d = RigidBodyDrone(start, preset='fpv_5inch', params=params, gains=gains, dt=DT_INNER,
                       ground_z=0.0)
    d.set_state(vel=pad['v'][0])
    zero = np.zeros(B)
    for k in range(L - 1):
        a, jk = pad['a'][k], pad['j'][k]
        f = a + np.array([0.0, 0.0, G])
        nf = np.linalg.norm(f, axis=1, keepdims=True)
        z = f / nf
        zdot = (jk - np.sum(jk * z, axis=1, keepdims=True) * z) / nf
        R = quat_to_rot(d.quat)
        rate_ff = np.column_stack([-np.sum(R[:, :, 1] * zdot, axis=1),
                                   np.sum(R[:, :, 0] * zdot, axis=1), zero])
        d.step_position(pad['p'][k], v_ff=pad['v'][k], a_ff=a, yaw=0.0, rate_ff=rate_ff)
    lg = d.get_log()
    lg['p_ref'] = pad['p']
    lg['w_max'] = params.w_max
    lg['n_sat'] = d.n_sat
    return lg


def gate_metrics(lg, b, gates, heads, T_end):
    t, P, V = lg['t'], lg['pos'][:, b], lg['vel'][:, b]
    assert np.all(np.isfinite(P)) and np.all(P[:, 2] > 0.0), 'NaN / ground contact'
    miss, cosh = np.full(N_GATES, np.nan), np.full(N_GATES, np.nan)
    i0 = 0
    for k in range(N_GATES):
        # forward crossings of the gate plane within R_SEARCH of the centre (a slow plan may
        # pass beside a gate before flying through it): first one inside the opening, else
        # the closest one
        side = (P[i0:] - gates[k]) @ heads[k]
        cands = []
        for j in np.where((side[:-1] < 0) & (side[1:] >= 0))[0]:
            a = -side[j] / (side[j + 1] - side[j])
            pc = P[i0 + j] + a * (P[i0 + j + 1] - P[i0 + j])
            d = np.linalg.norm(pc - gates[k])
            if d <= R_SEARCH:
                cands.append((d, j, a))
        if not cands:
            miss[k] = np.linalg.norm(P[i0:] - gates[k], axis=1).min()
            continue
        inside = [c for c in cands if c[0] <= R_GATE]
        d, j, a = inside[0] if inside else min(cands)
        vc = V[i0 + j] + a * (V[i0 + j + 1] - V[i0 + j])
        miss[k] = d
        cosh[k] = vc @ heads[k] / np.linalg.norm(vc)
        i0 = i0 + j + 1
    sel = t <= T_end + 1e-9
    r = lg['rotor'][sel, b]
    rdot = np.diff(r, axis=0) / DT_INNER
    e = np.linalg.norm(P[sel] - lg['p_ref'][sel, b], axis=1)
    dt = np.diff(t[sel])
    pw = lg['power'][sel, b]
    return dict(miss=miss, cos=cosh, hits=int(np.sum((miss <= R_TOL) & (cosh >= COS_HEADING))),
                energy=float(np.sum(0.5 * (pw[1:] + pw[:-1]) * dt)),
                rotor_rate_rms=float(np.sqrt(np.mean(rdot ** 2))),
                rotor_max=float(r.max() / lg['w_max']),
                rate_max=float(np.linalg.norm(lg['omega'][sel, b], axis=1).max()),
                tilt_max=float(np.degrees(tilt_angle(lg['quat'][sel, b]).max())),
                track_rms=float(np.sqrt(np.mean(e ** 2))), track_max=float(e.max()))


def plan_stats(coeffs, T):
    t = np.arange(0.0, T.sum(), 0.002)
    v = eval_traj(coeffs, T, t, 1)
    a = eval_traj(coeffs, T, t, 2)
    u = np.linalg.norm(a + np.array([0, 0, G]), axis=1)
    p = eval_traj(coeffs, T, t, 0)
    cum = np.cumsum(T)
    pg = eval_traj(coeffs, T, cum - 1e-12, 0)
    return dict(v_max=float(np.linalg.norm(v, axis=1).max()), u_max=float(u.max()),
                z_min=float(p[:, 2].min()),
                plan_miss=float(np.max(np.linalg.norm(pg - GATE_POS_CUR[0], axis=1))))


GATE_POS_CUR = [GATE_POS]


def run_simulation():
    seg_len = np.linalg.norm(np.diff(np.vstack([GATE_POS[0] - 2 * GATE_HEAD[0], GATE_POS]),
                                     axis=0), axis=1)
    T_fixed = seg_len / seg_len.sum() * T_TOTAL
    plans = {'fixed': (T_fixed, np.array([[0, plan(T_fixed)[1], T_TOTAL, plan(T_fixed)[1]]]), 0)}
    plans['redist'] = optimise(T_fixed, 'redist')
    plans['joint'] = optimise(T_fixed, 'joint')
    coeffs = {s: plan(plans[s][0])[0] for s in STRATS}
    J = {s: plan(plans[s][0])[1] for s in STRATS}
    refs = {s: reference(coeffs[s], plans[s][0], DT_INNER) for s in STRATS}
    lg = fly([refs[s] for s in STRATS])
    met = {s: gate_metrics(lg, i, GATE_POS, GATE_HEAD, plans[s][0].sum())
           for i, s in enumerate(STRATS)}
    stats = {s: plan_stats(coeffs[s], plans[s][0]) for s in STRATS}
    g_true = grad_J(T_fixed)
    g_dom = dominant_grad(T_fixed, coeffs['fixed'])
    # robustness: jittered layouts, same three allocation rules, flown in one batch
    rob = {s: [] for s in STRATS}
    rob_refs, rob_meta = [], []
    for seed in range(N_LAYOUTS):
        rng = np.random.default_rng(seed)
        gp = GATE_POS + rng.uniform(-JITTER, JITTER, GATE_POS.shape)
        gp[:, 2] = np.maximum(gp[:, 2], 0.8)
        sl = np.linalg.norm(np.diff(np.vstack([gp[0] - 2 * GATE_HEAD[0], gp]), axis=0), axis=1)
        Tf = sl / sl.sum() * T_TOTAL
        for s in STRATS:
            Ts = Tf if s == 'fixed' else optimise(Tf, s, gates=gp)[0]
            c, Js = plan(Ts, gates=gp)
            rob[s].append(dict(J=Js, T=Ts.sum()))
            if s in ROB_FLOWN:
                rob_refs.append(reference(c, Ts, DT_INNER))
                rob_meta.append((s, seed, gp, Ts.sum()))
    lr = fly(rob_refs)
    for i, (s, seed, gp, Te) in enumerate(rob_meta):
        rob[s][seed].update(gate_metrics(lr, i, gp, GATE_HEAD, Te))
    return dict(plans=plans, coeffs=coeffs, J=J, refs=refs, lg=lg, met=met, stats=stats,
                g_true=g_true, g_dom=g_dom, rob=rob, T_fixed=T_fixed)


# ── Plots ─────────────────────────────────────────────────────────────────────
def _gates(ax):
    ax.scatter(GATE_POS[:, 0], GATE_POS[:, 1], GATE_POS[:, 2], color='green', marker='D', s=50,
               label='gates')
    for k, (g, h) in enumerate(zip(GATE_POS, GATE_HEAD)):
        ax.quiver(*g, *h, length=1.2, color='green', lw=1.5, arrow_length_ratio=0.3)
        ax.text(*(g + [0, 0, 0.4]), f'G{k + 1}', color='darkgreen', fontsize=8)
    st = GATE_POS[0] - 2 * GATE_HEAD[0]
    ax.scatter(*st, color='k', marker='^', s=60, label='start')


def plot_trajectory_3d(R, out_dir):
    fig = plt.figure(figsize=(11, 8))
    ax = fig.add_subplot(111, projection='3d')
    _gates(ax)
    lg = R['lg']
    for i, s in enumerate(STRATS):
        r = R['refs'][s]
        sel = r['t'] <= r['T'] + 1e-9
        ax.plot(*r['p'][sel].T, ':', color=COLOR[s], lw=1.0)
        P = lg['pos'][:len(r['t']), i][sel]
        ax.plot(*P.T, color=COLOR[s], lw=1.6,
                label=f"{LABEL[s]}: T = {r['T']:.2f} s, J = {R['J'][s]:.0f}")
    lo, hi = GATE_POS.min(0) - 2, GATE_POS.max(0) + 2
    for x in (lo[0], hi[0]):
        for y in (lo[1], hi[1]):
            ax.plot([x, x], [y, y], [0, hi[2] + 1], color='lightgrey', lw=0.6)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_zlabel('z (m)')
    ax.set_zlim(0, 6)
    ax.set_title('S090 minimum-snap racing line: plan (dotted) and rigid-body flight (solid)')
    ax.legend(fontsize=8, loc='upper left')
    ax.view_init(elev=35, azim=-60)
    path = os.path.join(out_dir, 'trajectory_3d.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_time_allocation(R, out_dir):
    fig, ax = plt.subplots(figsize=(10, 4.5))
    x = np.arange(N_GATES)
    for i, s in enumerate(STRATS):
        T = R['plans'][s][0]
        ax.bar(x + (i - 1) * 0.27, T, 0.27, color=COLOR[s],
               label=f'{LABEL[s]} (total {T.sum():.2f} s)')
    ax.set_xticks(x)
    ax.set_xticklabels([f'seg {k + 1}\n-> G{k + 1}' for k in range(N_GATES)])
    ax.set_ylabel('segment duration (s)')
    ax.set_title('Segment time allocation')
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3, axis='y')
    path = os.path.join(out_dir, 'time_allocation.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_gate_miss(R, out_dir):
    fig, axes = plt.subplots(1, 2, figsize=(13, 4.5))
    x = np.arange(N_GATES)
    for i, s in enumerate(STRATS):
        m = R['met'][s]
        axes[0].bar(x + (i - 1) * 0.27, 100 * m['miss'], 0.27, color=COLOR[s], label=LABEL[s])
        axes[1].bar(x + (i - 1) * 0.27, np.degrees(np.arccos(np.clip(m['cos'], -1, 1))), 0.27,
                    color=COLOR[s], label=LABEL[s])
    axes[0].axhline(100 * R_TOL, color='red', ls='--', lw=1, label=r'$r_{tol}$ = 0.2 m')
    axes[0].axhline(100 * R_GATE, color='grey', ls=':', lw=1, label=r'$r_{gate}$ = 0.3 m')
    axes[0].set_ylabel('flown miss distance (cm)')
    axes[0].set_title('Rigid-body gate miss (planned miss = 0 for all)')
    axes[1].axhline(20, color='red', ls='--', lw=1, label='20 deg limit')
    axes[1].set_ylabel('heading error at crossing (deg)')
    axes[1].set_title('Flown velocity direction vs gate heading')
    for ax in axes:
        ax.set_xticks(x)
        ax.set_xticklabels([f'G{k + 1}' for k in range(N_GATES)])
        ax.legend(fontsize=8)
        ax.grid(alpha=0.3, axis='y')
    plt.tight_layout()
    path = os.path.join(out_dir, 'gate_miss.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_convergence(R, out_dir):
    fig, axes = plt.subplots(1, 2, figsize=(13, 4.5))
    Jf = R['J']['fixed']
    for s in ('redist', 'joint'):
        h = R['plans'][s][1]
        axes[0].semilogy(h[:, 0], h[:, 1], color=COLOR[s], lw=1.5, label=LABEL[s])
        axes[1].plot(h[:, 0], h[:, 2], color=COLOR[s], lw=1.5, label=LABEL[s])
    axes[0].axhline(Jf, color=COLOR['fixed'], ls='--', lw=1.2, label='fixed-time baseline')
    axes[0].set_ylabel('snap cost J')
    axes[0].set_title('Snap cost vs iteration')
    axes[1].axhline(T_TOTAL, color=COLOR['fixed'], ls='--', lw=1.2)
    axes[1].set_ylabel('total lap time (s)')
    axes[1].set_title('Total time vs iteration')
    for ax in axes:
        ax.set_xlabel('gradient-descent iteration')
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
    plt.tight_layout()
    path = os.path.join(out_dir, 'snap_convergence.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_speed_profile(R, out_dir):
    fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    lg = R['lg']
    for i, s in enumerate(STRATS):
        r = R['refs'][s]
        sel = r['t'] <= r['T'] + 1e-9
        axes[0].plot(r['t'][sel], np.linalg.norm(r['v'][sel], axis=1), color=COLOR[s], lw=1.4,
                     label=LABEL[s])
        u = np.linalg.norm(r['a'][sel] + np.array([0, 0, G]), axis=1)
        axes[1].plot(r['t'][sel], u / G, color=COLOR[s], lw=1.2, label=LABEL[s])
        for tg in np.cumsum(R['plans'][s][0]):
            axes[0].plot(tg, 0.3, '|', color=COLOR[s], ms=14, mew=2)
    axes[0].axhline(V_MAX, color='grey', ls=':', lw=1, label=r'$v_{max}$ = 12 m/s')
    axes[0].set_ylabel('planned speed (m/s)')
    axes[0].set_title('Planned speed (ticks = gate passage times)')
    axes[1].axhline(A_MAX / G, color='grey', ls=':', lw=1, label=r'$a_{max}$ = 2g')
    axes[1].set_ylabel('planned thrust acceleration (g)')
    axes[1].set_xlabel('time (s)')
    for ax in axes:
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
    plt.tight_layout()
    path = os.path.join(out_dir, 'speed_profile.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_summary(R, out_dir):
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.5))
    x = np.arange(len(STRATS))
    Jf = R['J']['fixed']
    vals = [R['J'][s] for s in STRATS]
    axes[0].bar(x, vals, color=[COLOR[s] for s in STRATS])
    for i, v in enumerate(vals):
        axes[0].text(i, v, f'{v:.0f}\n({100 * (v / Jf - 1):+.1f} %)', ha='center', va='bottom',
                     fontsize=8)
    axes[0].set_ylabel('snap cost J')
    axes[0].set_title('Snap cost (main course)')
    rob = R['rob']
    rel_J = [[r['J'] / f['J'] for r, f in zip(rob[s], rob['fixed'])] for s in STRATS]
    rel_E = [[r['energy'] / f['energy'] for r, f in zip(rob[s], rob['fixed'])]
             for s in ROB_FLOWN]
    axes[1].boxplot(rel_J, patch_artist=True)
    axes[1].set_yscale('log')
    axes[1].set_ylabel('J / J(fixed)')
    axes[1].set_title(f'Snap cost over {N_LAYOUTS} jittered layouts')
    axes[2].boxplot(rel_E, patch_artist=True)
    axes[2].set_ylabel('flown shaft energy / fixed')
    axes[2].set_title('Rigid-body shaft energy per lap')
    for ax, keys in ((axes[1], STRATS), (axes[2], ROB_FLOWN)):
        ax.set_xticks(range(1, len(keys) + 1))
        ax.set_xticklabels([LABEL_TXT[s] for s in keys], fontsize=8)
        ax.grid(alpha=0.3, axis='y')
        ax.axhline(1.0, color='grey', lw=0.8)
    axes[0].set_xticks(x)
    axes[0].set_xticklabels([LABEL_TXT[s] for s in STRATS], fontsize=8)
    plt.tight_layout()
    path = os.path.join(out_dir, 'snap_summary.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def save_animation(R, out_dir, step=25):
    """Fixed-time (red) and redistributed (blue) plans flown simultaneously (rigid body)."""
    lg = R['lg']
    show = ['fixed', 'redist']
    n = int(round(max(R['refs'][s]['T'] for s in show) / DT_INNER))
    idx = np.unique(np.minimum(np.arange(0, n + step, step), n))
    fig = plt.figure(figsize=(8, 7))
    ax = fig.add_subplot(111, projection='3d')
    _gates(ax)
    ax.set_xlim(-5, 10)
    ax.set_ylim(-3, 10)
    ax.set_zlim(0, 5)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_zlabel('z (m)')
    ax.view_init(elev=35, azim=-60)
    trails, dots = {}, {}
    for s in show:
        trails[s], = ax.plot([], [], [], color=COLOR[s], lw=1.5, label=LABEL[s])
        dots[s], = ax.plot([], [], [], 'o', color=COLOR[s], ms=7)
    ax.legend(fontsize=8, loc='upper left')

    def update(f):
        i = idx[f]
        for s in show:
            b = STRATS.index(s)
            j = min(i, int(round(R['refs'][s]['T'] / DT_INNER)))
            P = lg['pos'][:j + 1, b]
            trails[s].set_data(P[:, 0], P[:, 1])
            trails[s].set_3d_properties(P[:, 2])
            dots[s].set_data([P[-1, 0]], [P[-1, 1]])
            dots[s].set_3d_properties([P[-1, 2]])
        ax.set_title(f'S090 rigid-body flight  t = {lg["t"][i]:.2f} s  (frame {f + 1}/{len(idx)})')
        return list(trails.values()) + list(dots.values())

    anim = animation.FuncAnimation(fig, update, frames=len(idx), interval=50, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    anim.save(path, writer='pillow', fps=20, dpi=80)
    plt.close(fig)
    print(f'Saved: {path} ({len(idx)} frames)')


# ── Main ──────────────────────────────────────────────────────────────────────
if __name__ == '__main__':
    R = run_simulation()
    out = os.path.normpath(OUTPUT_DIR)
    os.makedirs(out, exist_ok=True)
    print('\n=== S090 Racing Drone Optimal Path - Results ===')
    pf = make_params('fpv_5inch')
    print(f'  Rigid body: fpv_5inch preset, m = {pf.mass} kg, T/W = {pf.t2w:.2f}, '
          f'{1 / DT_INNER:.0f} Hz')
    Jf = R['J']['fixed']
    print('  Initial (fixed) segment times (s): ' + ', '.join(f'{x:.3f}' for x in R['T_fixed']))
    print('  dJ/dT at the fixed allocation, exact (central difference): '
          + ', '.join(f'{x:.1f}' for x in R['g_true']))
    print("  card's 'dominant' term c^T dQ/dT c:                        "
          + ', '.join(f'{x:.1f}' for x in R['g_dom']))
    gt, gd = R['g_true'], R['g_dom']
    sig = np.abs(gd) > 1e-6 * np.abs(gd).max()
    print(f"  explicit term has the opposite sign to the exact gradient on "
          f"{int(np.sum(sig & (np.sign(gd) != np.sign(gt))))} of {len(gt)} segments; on "
          f"{int(np.sum(~sig))} segment(s) it is ~0 ({', '.join(f'{x:.1e}' for x in gd[~sig])}: "
          f"snap vanishes at the free end)")
    print(f'  card step eta = {ETA}: first step on the projected gradient changes T by up to '
          f"{ETA * np.abs(R['g_true'] - R['g_true'].mean()).max():.3f} s")
    for s in STRATS:
        T, hist, nb = R['plans'][s]
        st, m = R['stats'][s], R['met'][s]
        print(f"  [{LABEL_TXT[s]}] T = {T.sum():.3f} s, J = {R['J'][s]:.1f} "
              f"({100 * (R['J'][s] / Jf - 1):+.1f} % vs fixed), backtracking halvings {nb}")
        print('      segment times: ' + ', '.join(f'{x:.3f}' for x in T))
        print(f"      plan: max speed {st['v_max']:.2f} m/s (limit {V_MAX}), max thrust acc "
              f"{st['u_max'] / G:.2f} g (limit 2 g), min z {st['z_min']:.2f} m, planned gate "
              f"miss {st['plan_miss']:.1e} m")
        print(f"      flown: gates passed {m['hits']}/6, miss (cm) "
              + ', '.join(f'{100 * x:.1f}' for x in m['miss'])
              + ', heading error (deg) '
              + ', '.join(f'{np.degrees(np.arccos(np.clip(c, -1, 1))):.1f}' for c in m['cos']))
        print(f"      flown: tracking RMS {100 * m['track_rms']:.1f} cm / max "
              f"{100 * m['track_max']:.1f} cm, shaft energy {m['energy']:.1f} J, rotor-speed "
              f"rate RMS {m['rotor_rate_rms']:.0f} rad/s^2, max rotor {100 * m['rotor_max']:.1f} %"
              f", max |omega| {m['rate_max']:.2f} rad/s, max tilt {m['tilt_max']:.1f} deg")
    hj = R['plans']['joint'][1]
    print(f"  J + lambda*T: objective {hj[0, 3]:.1f} -> {hj[-1, 3]:.1f}; lambda*T share at the "
          f"end {LAMBDA * hj[-1, 2]:.2f} of {hj[-1, 3]:.1f}; still decreasing by "
          f"{hj[-2, 3] - hj[-1, 3]:.3f} per iteration at iteration {N_ITER}")
    # how large must lambda be before J + lambda*T prefers a SHORTER lap? -> lambda > -dJ/dT
    for s in ('fixed', 'redist'):
        Ts = R['plans'][s][0]
        dJ = (plan(Ts * 1.001)[1] - plan(Ts * 0.999)[1]) / (0.002 * Ts.sum())
        print(f"  [{LABEL_TXT[s]}] dJ/dT_total along uniform scaling = {dJ:.0f} per s; "
              f"J + lambda*T shortens this lap only if lambda > {-dJ:.0f} "
              f"(card lambda = {LAMBDA})")
    rob = R['rob']
    print(f'\n  Robustness: {N_LAYOUTS} layouts, gates jittered by U(-{JITTER}, {JITTER}) m:')
    rj = np.array([r['J'] / f['J'] for r, f in zip(rob['joint'], rob['fixed'])])
    tj = np.array([r['T'] for r in rob['joint']])
    print(f"  [{LABEL_TXT['joint']}] J/J_fixed {rj.mean():.4f} +- {rj.std():.4f}; T "
          f"{tj.mean():.1f} +- {tj.std():.1f} s (planned only, not flown)")
    for s in ROB_FLOWN:
        rel = np.array([r['J'] / f['J'] for r, f in zip(rob[s], rob['fixed'])])
        relE = np.array([r['energy'] / f['energy'] for r, f in zip(rob[s], rob['fixed'])])
        tt = np.array([r['T'] for r in rob[s]])
        hits = np.array([r['hits'] for r in rob[s]])
        miss = np.concatenate([r['miss'] for r in rob[s]])
        rr = np.array([r['rotor_rate_rms'] for r in rob[s]])
        print(f"  [{LABEL_TXT[s]}] J/J_fixed {rel.mean():.3f} +- {rel.std():.3f} (lower on "
              f"{np.sum(rel < 1 - 1e-9)}/{N_LAYOUTS}); T {tt.mean():.2f} +- {tt.std():.2f} s; "
              f"flown gates {hits.mean():.2f}/6 (all 6 on {np.sum(hits == 6)}); miss mean "
              f"{100 * miss.mean():.1f} cm, max {100 * miss.max():.1f} cm; energy/fixed "
              f"{relE.mean():.3f} +- {relE.std():.3f} (lower on {np.sum(relE < 1)}); rotor-rate "
              f"RMS {rr.mean():.0f} rad/s^2; mean J reduction {100 * (1 - rel.mean()):.1f} %, "
              f"mean energy saving {100 * (1 - relE.mean()):.1f} %")
    # does snap predict the flown effort? rank correlation over all flights
    Js = np.array([r['J'] for s in ROB_FLOWN for r in rob[s]])
    Es = np.array([r['energy'] / r['T'] for s in ROB_FLOWN for r in rob[s]])
    Rr = np.array([r['rotor_rate_rms'] for s in ROB_FLOWN for r in rob[s]])
    Ms = np.array([r['track_rms'] for s in ROB_FLOWN for r in rob[s]])

    def spearman(a, b):
        ra, rb = np.argsort(np.argsort(a)), np.argsort(np.argsort(b))
        return np.corrcoef(ra, rb)[0, 1]
    print(f'  Spearman rank correlation of J with: mean shaft power {spearman(Js, Es):+.2f}, '
          f'rotor-rate RMS {spearman(Js, Rr):+.2f}, tracking RMS {spearman(Js, Ms):+.2f} '
          f'({len(Js)} flights, pooled; J differs ~{np.median(Js[:N_LAYOUTS]) / np.median(Js[N_LAYOUTS:]):.0f}x '
          f'between the two groups)')
    # a pure group split with no within-group relation: J ranks by group, effort random
    rng = np.random.default_rng(7)
    null = np.array([spearman(Js, np.concatenate([rng.permutation(N_LAYOUTS) + N_LAYOUTS,
                                                  rng.permutation(N_LAYOUTS)]))
                     for _ in range(2000)])
    print(f'  pooled-correlation baseline from the group split alone: {null.mean():+.2f} '
          f'(95th percentile {np.percentile(null, 95):+.2f}, 2000 permutations)')
    for gi, s in enumerate(ROB_FLOWN):
        sl = slice(gi * N_LAYOUTS, (gi + 1) * N_LAYOUTS)
        print(f'  within {LABEL_TXT[s]} ({N_LAYOUTS} layouts): Spearman of J with mean shaft '
              f'power {spearman(Js[sl], Es[sl]):+.2f}, rotor-rate RMS '
              f'{spearman(Js[sl], Rr[sl]):+.2f}, tracking RMS {spearman(Js[sl], Ms[sl]):+.2f}')
    print(f"  Mixer desaturation steps: main {R['lg']['n_sat']}")

    plot_trajectory_3d(R, out)
    plot_time_allocation(R, out)
    plot_gate_miss(R, out)
    plot_convergence(R, out)
    plot_speed_profile(R, out)
    plot_summary(R, out)
    save_animation(R, out)
