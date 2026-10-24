"""
S096 Drone Relay Race
=====================
Three drones run a 300 m relay at 10 m altitude (legs of 100 m). Drone A starts at the origin
with the baton; drones B and C wait at the relay points (100, 0, 12) and (200, 0, 12). A baton
passes when the two drones stay within 0.3 m and 0.5 m/s of each other for 1 s continuously.

Strategies for the outgoing drone (same drones, disturbances and sensor noise; CRN):
  1. Naive (card)   - reacts only when the incoming drone is within 5 m: it then matches the
                      incoming velocity at full acceleration and closes the remaining gap with
                      the formation PD (current KF estimate, no prediction).
  2. KF + intercept - tracks the incoming drone with a 6-state constant-velocity Kalman filter,
                      launches at the last moment a minimum-snap intercept (degree-7 polynomial)
                      to the predicted rendezvous state stays within 80 % of a_max, replans it
                      at 10 Hz, then holds formation with the same PD controller.

A grid search over the cruise speeds of A and B (C has no handoff after it and flies at
v_max) gives the fastest race whose handoffs both finish inside a 50 m exchange zone.

Physics: point-mass drones, |a| <= 5 m/s^2, speed governor at 20 m/s, Gaussian acceleration
disturbance 0.1 m/s^2, Euler integration at 50 Hz.

Usage:
    conda activate drones
    python src/05_special_entertainment/s096_relay_race.py
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
L_LEG        = 100.0
Z_RACE       = 10.0
RELAY_XS     = [0.0, 100.0, 200.0, 300.0]
D_HANDOFF    = 0.3       # m
V_TOL        = 0.5       # m/s
T_HANDOFF    = 1.0       # s
D_DETECT     = 5.0       # m   naive detection range
A_MAX        = 5.0       # m/s^2
V_CRUISE_MIN = 5.0       # m/s
V_CRUISE_MAX = 20.0      # m/s
SIGMA_D      = 0.1       # m/s^2 disturbance
DT           = 0.02      # s
SIGMA_A_KF   = 0.5       # m/s^2 KF process noise
SIGMA_OBS    = 0.05      # m   position measurement noise
TAU_MIN, TAU_MAX, DTAU = 0.5, 10.0, 0.1
V_CRUISE     = [12.0, 14.0, 16.0]
T_MAX        = 120.0
KP_F, KD_F   = 8.0, 4.0  # card tracking gains (min-snap following and formation hold)
K_CRUISE     = 5.0       # card cruise velocity gain
K_ALT        = 2.0       # card altitude-hold gain

# ── Parameters (added) ────────────────────────────────────────────────────────
A_LAUNCH     = 0.8 * A_MAX   # launch when the gentlest feasible intercept needs this peak accel
REPLAN_DT    = 0.1           # s  intercept replanning period
L_ZONE       = 50.0          # m  handoff must finish within this distance past the relay point
N_MC         = 30            # Monte Carlo seeds per configuration
GRID_V       = np.arange(8.0, 20.0 + 1e-9, 2.0)   # speed grid for A and B
DETECT_SWEEP = [5.0, 10.0, 15.0, 17.5, 20.0, 30.0]  # naive detection ranges
RP_TRIGGER   = 15.0          # m  card: KF approach trigger before the relay point
A_SWEEP      = [3.0, 4.0, 5.0]          # launch peak-acceleration sensitivity
L_SWEEP      = [40.0, 50.0, 55.0, 60.0]  # exchange-zone length sensitivity
STATIONS     = [np.array([0.0, 0.0, Z_RACE]), np.array([100.0, 0.0, Z_RACE + 2.0]),
                np.array([200.0, 0.0, Z_RACE + 2.0])]
NAMES        = ['A', 'B', 'C']
DRONE_COLORS = ['red', 'blue', 'green']
STRAT_LABELS = {'naive': 'Naive (detect 5 m)', 'kf': 'KF + min-snap intercept',
                'kf15': 'KF, card 15 m trigger'}

OUTPUT_DIR = os.path.normpath(os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '05_special_entertainment', 's096_relay_race'))


# ── Kalman filter ─────────────────────────────────────────────────────────────

class KalmanFilter6D:
    """Constant-velocity KF on [p, v] with position measurements (card model)."""

    def __init__(self, p0, v0, dt=DT):
        self.F = np.eye(6)
        self.F[:3, 3:] = dt * np.eye(3)
        q = SIGMA_A_KF ** 2
        self.Q = q * np.block([[dt ** 4 / 4 * np.eye(3), dt ** 3 / 2 * np.eye(3)],
                               [dt ** 3 / 2 * np.eye(3), dt ** 2 * np.eye(3)]])
        self.R = SIGMA_OBS ** 2 * np.eye(3)
        self.x = np.concatenate([p0, v0])
        self.P = np.diag([SIGMA_OBS ** 2] * 3 + [1.0] * 3)

    def step(self, z):
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        S = self.P[:3, :3] + self.R
        K = self.P[:, :3] @ np.linalg.inv(S)
        self.x = self.x + K @ (z - self.x[:3])
        self.P = self.P - K @ self.P[:3, :]


# ── Minimum-snap (degree 7, rest-to-rest in acceleration and jerk) ───────────

def _deriv_row(s, d):
    row = np.zeros(8)
    for k in range(d, 8):
        row[k] = math.prod(range(k - d + 1, k + 1)) * s ** (k - d)
    return row


_M = np.array([_deriv_row(0, 0), _deriv_row(0, 1), _deriv_row(0, 2), _deriv_row(0, 3),
               _deriv_row(1, 0), _deriv_row(1, 1), _deriv_row(1, 2), _deriv_row(1, 3)])
_MINV = np.linalg.inv(_M)
_S_CHECK = np.linspace(0.0, 1.0, 41)
_B1 = np.array([_deriv_row(s, 1) for s in _S_CHECK]) @ _MINV   # dp/ds from boundary vector
_B2 = np.array([_deriv_row(s, 2) for s in _S_CHECK]) @ _MINV   # d2p/ds2
# Only p0, v0*T, pf, vf*T are non-zero in the boundary vector (a = j = 0 at both ends).
_COLS = [0, 1, 4, 5]
# Rest -> v with a = j = 0 at both ends and displacement v T / 2: the velocity follows the
# quintic smoothstep 10 s^3 - 15 s^4 + 6 s^5, whose peak slope is 15/8, so peak accel = 1.875 v/T.
SNAP_PEAK = 15.0 / 8.0


def snap_coeffs(p0, v0, pf, vf, T, a0=0.0, j0=0.0):
    """Coefficients (3 x 8) in normalised time s = t/T for each axis.

    End state: a = j = 0 (card). Start: a = j = 0 for the first plan (card); a replan starts
    from the previous reference acceleration and jerk so the command stays continuous."""
    b = np.zeros((3, 8))
    b[:, 0], b[:, 1], b[:, 2], b[:, 3] = p0, v0 * T, a0 * T ** 2, j0 * T ** 3
    b[:, 4], b[:, 5] = pf, vf * T
    return b @ _MINV.T


_FAC = np.array([[math.prod(range(k - d + 1, k + 1)) if k >= d else 0.0 for k in range(8)]
                 for d in range(4)])
_POW = np.array([[max(k - d, 0) for k in range(8)] for d in range(4)])


def snap_eval(c, t, T):
    """Position, velocity, acceleration and jerk of the plan at local time t."""
    s = min(max(t / T, 0.0), 1.0)
    rows = _FAC * s ** _POW                     # (4, 8) derivative rows at s
    out = rows @ c.T                             # (4, 3)
    return out[0], out[1] / T, out[2] / T ** 2, out[3] / T ** 3


def intercept_options(p_out, v_out, x_in, taus):
    """Peak |a| and min x-velocity of the min-snap to the CV-predicted incoming state."""
    p_in, v_in = x_in[:3], x_in[3:]
    pf = p_in[None, :] + taus[:, None] * v_in[None, :]          # (n, 3)
    bnd = np.stack([np.broadcast_to(p_out, pf.shape), v_out[None, :] * taus[:, None],
                    pf, v_in[None, :] * taus[:, None]], axis=-1)  # (n, 3, 4)
    acc = np.einsum('sk,nak->nsa', _B2[:, _COLS], bnd) / taus[:, None, None] ** 2
    vx = np.einsum('sk,nk->ns', _B1[:, _COLS], bnd[:, 0, :]) / taus[:, None]
    return np.linalg.norm(acc, axis=2).max(axis=1), vx.min(axis=1)


# ── Race simulation ───────────────────────────────────────────────────────────

def limit(a):
    """Clip the commanded acceleration to |a| <= A_MAX."""
    n = np.linalg.norm(a)
    return a * (A_MAX / n) if n > A_MAX else a


def integrate(p, v, a):
    """Euler step with a speed governor: the speed never exceeds V_CRUISE_MAX."""
    v_new = v + a * DT
    sp = np.linalg.norm(v_new)
    if sp > V_CRUISE_MAX:
        v_new = v_new * (V_CRUISE_MAX / sp)
    return p + v_new * DT, v_new


def cruise_cmd(p, v, v_cruise):
    vd = np.array([v_cruise, -K_ALT * p[1], -K_ALT * (p[2] - Z_RACE)])
    return K_CRUISE * (vd - v)


def hold_cmd(p, v, target):
    return 2.0 * (target - p) - 2.0 * v


def run_simulation(strategy, v_cruise=V_CRUISE, seed=0, d_detect=D_DETECT, record=False,
                   a_launch=A_LAUNCH):
    """Simulate one race. Returns a result dict (and logs when record=True).

    strategy 'kf15' is the KF strategy with the card's approach-zone trigger: the outgoing
    drone may only launch once the incoming drone is within RP_TRIGGER of the relay point."""
    trigger = RP_TRIGGER if strategy == 'kf15' else None
    if strategy == 'kf15':
        strategy = 'kf'
    rng_dist = np.random.default_rng([seed, 1])     # drone disturbances
    rng_obs = np.random.default_rng([seed, 2])      # KF measurements
    p = np.array([s.copy() for s in STATIONS])
    v = np.zeros((3, 3))
    holder = 0
    done = [False, False, False]                     # holder finished its handoff
    kf = KalmanFilter6D(p[0], v[0])
    mode = 'wait'                                    # outgoing: wait / intercept / hold
    plan = None
    window = 0.0
    handoffs = []
    hist = dict(t=[], p=[], v=[], holder=[], mode=[]) if record else None
    finish = None
    n_steps = int(round(T_MAX / DT))
    for k in range(n_steps):
        t = k * DT
        z_noise = rng_obs.normal(0.0, SIGMA_OBS, 3)      # drawn every step (CRN)
        d_noise = rng_dist.normal(0.0, SIGMA_D, (3, 3))
        a = np.zeros((3, 3))
        out = holder + 1 if holder < 2 else None
        for i in range(3):
            if i == holder:
                a[i] = cruise_cmd(p[i], v[i], v_cruise[i])
            elif i < holder:                         # finished: brake and hold altitude
                a[i] = K_CRUISE * (np.array([0.0, -K_ALT * p[i, 1], -K_ALT * (p[i, 2] - Z_RACE)])
                                   - v[i])
            elif i != out:
                a[i] = hold_cmd(p[i], v[i], STATIONS[i])
        if out is not None:
            kf.step(p[holder] + z_noise)
            xin = kf.x
            po, vo = p[out], v[out]
            if mode == 'wait':
                a[out] = hold_cmd(po, vo, STATIONS[out])
                if strategy == 'kf':
                    # Latest launch (= earliest rendezvous) for a rest -> v_in min-snap whose
                    # peak acceleration is A_LAUNCH: it lasts T = 15/8 v / A_LAUNCH and covers
                    # v T / 2, so launch when the predicted along-track gap is v T / 2.
                    v_in = xin[3]
                    armed = trigger is None or RELAY_XS[out] - xin[0] <= trigger
                    if v_in > 1.0 and armed:
                        tau = SNAP_PEAK * v_in / a_launch
                        if po[0] - xin[0] <= v_in * tau / 2:
                            peak, vxmin = intercept_options(po, vo, xin, np.array([tau]))
                            plan = dict(t0=t, T=tau, t_star=t + tau, last=t,
                                        c=snap_coeffs(po, vo, xin[:3] + tau * xin[3:],
                                                      xin[3:], tau),
                                        launch_gap=float(np.linalg.norm(xin[:3] - po)),
                                        launch_peak=float(peak[0]), tau=tau,
                                        launch_vxmin=float(vxmin[0]))
                            mode = 'intercept'
                elif np.linalg.norm(xin[:3] - po) <= d_detect:
                    plan = dict(launch_gap=float(np.linalg.norm(xin[:3] - po)), t0=t)
                    mode = 'sprint'
            if mode == 'sprint':
                # naive: match the incoming velocity at full acceleration (card cruise gain),
                # then close the position gap with the formation PD
                a[out] = K_CRUISE * (xin[3:] - vo)
                if np.linalg.norm(xin[3:] - vo) <= V_TOL:
                    mode = 'hold'
            if mode == 'intercept':
                rem = plan['t_star'] - t
                if rem > 0.3 and t - plan['last'] >= REPLAN_DT - 1e-9:
                    _, _, a0, j0 = snap_eval(plan['c'], t - plan['t0'], plan['T'])
                    plan.update(t0=t, T=rem, last=t,
                                c=snap_coeffs(po, vo, xin[:3] + rem * xin[3:], xin[3:], rem,
                                              a0, j0))
                if rem <= 0.0:
                    mode = 'hold'
                else:
                    pr, vr, ar, _ = snap_eval(plan['c'], t - plan['t0'], plan['T'])
                    a[out] = KP_F * (pr - po) + KD_F * (vr - vo) + ar
            if mode == 'hold':
                a[out] = KP_F * (xin[:3] - po) + KD_F * (xin[3:] - vo)
        # dynamics
        for i in range(3):
            p_old = p[i].copy()
            p[i], v[i] = integrate(p[i], v[i], limit(a[i]) + d_noise[i])
            assert np.linalg.norm(v[i]) <= V_CRUISE_MAX + 1e-9, 'speed limit exceeded'
            assert np.linalg.norm(p[i] - p_old) <= V_CRUISE_MAX * DT + 1e-9, 'teleport'
            assert p[i, 2] > 0.0, 'below ground'
        if record:
            hist['t'].append(t + DT); hist['p'].append(p.copy()); hist['v'].append(v.copy())
            hist['holder'].append(holder); hist['mode'].append(mode)
        # handoff check on true states
        if out is not None:
            dp = np.linalg.norm(p[holder] - p[out])
            dv = np.linalg.norm(v[holder] - v[out])
            window = window + DT if (dp <= D_HANDOFF and dv <= V_TOL) else 0.0
            if window >= T_HANDOFF - 1e-9:
                x_h = float(p[holder, 0])
                handoffs.append(dict(t=t + DT, x=x_h, dp=dp, dv=dv,
                                     zone_ok=x_h <= RELAY_XS[out] + L_ZONE,
                                     launch_gap=(plan or {}).get('launch_gap', np.nan),
                                     launch_peak=(plan or {}).get('launch_peak', np.nan)))
                done[holder] = True
                holder = out
                window = 0.0
                mode = 'wait'
                plan = None
                if holder < 2:
                    kf = KalmanFilter6D(p[holder] + z_noise, v[holder])
        if holder == 2 and p[2, 0] >= RELAY_XS[3]:
            finish = t + DT
            break
    res = dict(finish=finish, handoffs=handoffs, n_handoffs=len(handoffs),
               zone_ok=len(handoffs) == 2 and all(h['zone_ok'] for h in handoffs))
    res['success'] = finish is not None and len(handoffs) == 2
    if record:
        for key in ('t', 'p', 'v', 'holder'):
            hist[key] = np.array(hist[key])
        res['hist'] = hist
    return res


# ── Experiments ───────────────────────────────────────────────────────────────

def monte_carlo(strategy, v_cruise, n=N_MC, d_detect=D_DETECT):
    rows = [run_simulation(strategy, v_cruise, seed=s, d_detect=d_detect) for s in range(n)]
    return mc_from_rows(rows)


def mc_from_rows(rows):
    n = len(rows)
    fin = np.array([r['finish'] if r['finish'] is not None else np.nan for r in rows])
    hx = np.full((n, 2), np.nan)
    ht = np.full((n, 2), np.nan)
    for i, r in enumerate(rows):
        for j, h in enumerate(r['handoffs']):
            hx[i, j] = h['x'] - RELAY_XS[j + 1]
            ht[i, j] = h['t']
    return dict(finish=fin, success=np.array([r['success'] for r in rows]),
                zone_ok=np.array([r['zone_ok'] for r in rows]), hx=hx, ht=ht, rows=rows)


def speed_grid(a_launch=A_LAUNCH):
    """Seed-0 grid over (v_A, v_B) with v_C = V_CRUISE_MAX for the KF strategy.

    Returns race times T, the zone flag Z for L_ZONE, and the farther of the two pass
    offsets past their relay points (X) so other zone lengths can be checked afterwards."""
    T = np.full((len(GRID_V), len(GRID_V)), np.nan)
    X = np.full_like(T, np.nan)
    for i, va in enumerate(GRID_V):
        for j, vb in enumerate(GRID_V):
            r = run_simulation('kf', [va, vb, V_CRUISE_MAX], seed=0, a_launch=a_launch)
            if r['success']:
                T[i, j] = r['finish']
                X[i, j] = max(h['x'] - RELAY_XS[k + 1] for k, h in enumerate(r['handoffs']))
    Z = np.isfinite(T) & (np.nan_to_num(X, nan=np.inf) <= L_ZONE)
    return T, Z, X


def best_feasible(T, X, L):
    feas = np.where(np.isfinite(T) & (np.nan_to_num(X, nan=np.inf) <= L), T, np.inf)
    if not np.isfinite(feas).any():
        return None
    i, j = np.unravel_index(np.argmin(feas), feas.shape)
    return GRID_V[i], GRID_V[j], T[i, j]


# ── Plots ─────────────────────────────────────────────────────────────────────

def plot_race_3d(rk, rn, out_dir):
    fig = plt.figure(figsize=(16, 7))
    ax = fig.add_subplot(111, projection='3d')
    for res, ls, tag, lw in [(rk, '-', 'KF', 1.6), (rn, '--', 'Naive', 1.0)]:
        P = res['hist']['p']
        for i in range(3):
            ax.plot(P[:, i, 0], P[:, i, 1], P[:, i, 2], ls, color=DRONE_COLORS[i], lw=lw,
                    alpha=1.0 if tag == 'KF' else 0.6, label=f'Drone {NAMES[i]} ({tag})')
        for h in res['handoffs']:
            ax.scatter([h['x']], [0], [Z_RACE], marker='o' if tag == 'KF' else 's', s=50,
                       c='k' if tag == 'KF' else 'grey')
    for xrp in RELAY_XS[1:3]:
        ax.scatter([xrp], [0.0], [Z_RACE], c='orange', s=120)
        ax.plot([xrp, xrp + L_ZONE], [0, 0], [Z_RACE - 0.6] * 2, color='orange', lw=4, alpha=0.5)
    ax.scatter([0], [0], [Z_RACE], c='black', s=80, marker='^', label='Start')
    ax.scatter([300], [0], [Z_RACE], c='gold', s=150, marker='*', label='Finish')
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)'); ax.set_zlabel('z (m)')
    ax.set_ylim(-3, 3); ax.set_zlim(8, 13)
    ax.set_title('S096 relay race, seed 0, speeds 12/14/16 m/s (solid = KF, dashed = naive; '
                 'dots = baton passes; orange bar = 50 m exchange zone)')
    ax.legend(fontsize=7, ncol=2, loc='upper left')
    ax.view_init(elev=22, azim=-80)
    path = os.path.join(out_dir, 'race_3d_trajectories.png')
    fig.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_handoff_detail(rk, rn, relay, out_dir):
    fig, axes = plt.subplots(1, 3, figsize=(18, 4.8))
    for res, ls, tag in [(rk, '-', 'KF + intercept'), (rn, '--', 'Naive')]:
        h = res['hist']
        inc, out = relay, relay + 1
        if len(res['handoffs']) > relay:
            t_end = res['handoffs'][relay]['t']
        else:
            t_end = h['t'][-1]
        t_beg = (res['handoffs'][relay - 1]['t'] if relay > 0 and
                 len(res['handoffs']) >= relay else 0.0)
        m = (h['t'] >= t_beg) & (h['t'] <= t_end + 1.0)
        dp = np.linalg.norm(h['p'][m, inc] - h['p'][m, out], axis=1)
        dv = np.linalg.norm(h['v'][m, inc] - h['v'][m, out], axis=1)
        col = 'red' if tag.startswith('KF') else 'blue'
        axes[0].semilogy(h['t'][m], dp, ls, color=col, label=tag)
        axes[1].plot(h['t'][m], dv, ls, color=col, label=tag)
        axes[2].plot(h['t'][m], h['v'][m, out, 0], ls, color=col, label=f'{tag}: outgoing')
        axes[2].plot(h['t'][m], h['v'][m, inc, 0], ':', color=col, lw=1,
                     label=f'{tag}: incoming')
        if len(res['handoffs']) > relay:
            for ax in axes:
                ax.axvline(t_end, color=col, ls=ls, lw=0.8)
    axes[0].axhline(D_HANDOFF, color='k', ls=':', label=f'$d_{{handoff}}$ = {D_HANDOFF} m')
    axes[0].set_ylabel('Separation (m, log)')
    axes[1].axhline(V_TOL, color='k', ls=':', label=f'$v_{{tol}}$ = {V_TOL} m/s')
    axes[1].set_ylabel('Velocity mismatch (m/s)')
    axes[1].set_ylim(0, 16)
    axes[2].set_ylabel('$v_x$ (m/s)')
    for ax, ti in zip(axes, ['Separation', 'Velocity mismatch', 'Forward speed']):
        ax.set_xlabel('Time (s)'); ax.set_title(f'RP{relay + 1}: {ti}')
        ax.legend(fontsize=7)
    fig.suptitle(f'S096 handoff detail at RP{relay + 1} (seed 0, vertical lines = baton passes)')
    fig.tight_layout()
    path = os.path.join(out_dir, f'handoff_rp{relay + 1}_detail.png')
    fig.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_monte_carlo(mc, sweep, out_dir):
    fig, axes = plt.subplots(1, 3, figsize=(18, 5))
    ax = axes[0]
    keys = [('naive', 'card'), ('kf15', 'card'), ('kf', 'card'), ('kf', 'opt')]
    labels = ['Naive\n12/14/16', 'KF, 15 m trigger\n12/14/16', 'KF\n12/14/16', 'KF\noptimised']
    data = [mc[k]['finish'][np.isfinite(mc[k]['finish'])] for k in keys]
    bp = ax.boxplot(data, tick_labels=labels, patch_artist=True)
    for patch, c in zip(bp['boxes'], ['#1f77b4', '#9467bd', '#d62728', '#ff9896']):
        patch.set_facecolor(c)
    ax.set_ylabel('Race time (s)')
    ax.set_title(f'Race time over {N_MC} seeds (finished races)')
    ax = axes[1]
    for k, c, lab in [(('naive', 'card'), '#1f77b4', 'Naive'), (('kf', 'card'), '#d62728', 'KF')]:
        for j, mk in enumerate(['o', 's']):
            ax.hist(mc[k]['hx'][:, j][np.isfinite(mc[k]['hx'][:, j])], bins=np.arange(0, 120, 2.5),
                    histtype='step', lw=1.6, color=c, ls='-' if j == 0 else '--',
                    label=f'{lab} RP{j + 1}')
    ax.axvline(L_ZONE, color='orange', lw=2, label=f'exchange zone {L_ZONE:.0f} m')
    ax.set_xlabel('Baton-pass position past the relay point (m)')
    ax.set_ylabel('Seeds'); ax.set_title('Where the handoff completes (12/14/16 m/s)')
    ax.legend(fontsize=8)
    ax = axes[2]
    ax.plot(DETECT_SWEEP, [np.nanmean(sweep[d]['finish']) for d in DETECT_SWEEP], 'o--',
            color='#1f77b4', label='Naive, mean race time')
    ax.axhline(np.nanmean(mc[('kf', 'card')]['finish']), color='#d62728',
               label='KF + intercept, mean race time')
    ax2 = ax.twinx()
    ax2.plot(DETECT_SWEEP, [100 * sweep[d]['zone_ok'].mean() for d in DETECT_SWEEP], 's:',
             color='grey', label='Naive, % inside zone')
    ax2.set_ylabel('% of seeds with both passes inside the zone', color='grey')
    ax2.set_ylim(-5, 105)
    ax.set_xlabel('Naive detection range $d_{detect}$ (m)')
    ax.set_ylabel('Race time (s)')
    ax.set_title('Naive baseline vs detection range (12/14/16 m/s)')
    h1, l1 = ax.get_legend_handles_labels()
    h2, l2 = ax2.get_legend_handles_labels()
    ax.legend(h1 + h2, l1 + l2, fontsize=8, loc='center right')
    fig.tight_layout()
    path = os.path.join(out_dir, 'monte_carlo.png')
    fig.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_speed_grid(T, Z, best, out_dir):
    fig, ax = plt.subplots(figsize=(8, 6.5))
    show = np.where(Z, T, np.nan)
    im = ax.imshow(show, origin='lower', cmap='viridis_r',
                   extent=[GRID_V[0] - 0.5, GRID_V[-1] + 0.5, GRID_V[0] - 0.5, GRID_V[-1] + 0.5])
    bad = np.isfinite(T) & ~Z
    vb, va = np.meshgrid(GRID_V, GRID_V)
    ax.scatter(vb[bad], va[bad], marker='x', c='grey', s=25, label='finished, pass outside zone')
    ax.scatter(vb[~np.isfinite(T)], va[~np.isfinite(T)], marker='.', c='k', s=25,
               label='race not finished')
    ax.scatter([best[1]], [best[0]], marker='*', c='red', s=250, label='fastest feasible')
    ax.scatter([V_CRUISE[1]], [V_CRUISE[0]], marker='o', facecolors='none', edgecolors='w',
               s=120, label='card speeds (12, 14)')
    plt.colorbar(im, ax=ax, label='Race time (s), seed 0')
    ax.set_xlabel('$v_B$ (m/s)'); ax.set_ylabel('$v_A$ (m/s)')
    ax.set_title(f'Speed optimisation (KF strategy, $v_C$ = {V_CRUISE_MAX:.0f} m/s)')
    ax.legend(fontsize=7, loc='upper left')
    path = os.path.join(out_dir, 'speed_optimisation.png')
    fig.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_sensitivity(grids, out_dir):
    """Fastest feasible race time against zone length for each launch acceleration."""
    fig, ax = plt.subplots(figsize=(8, 5))
    Ls = np.arange(30.0, 100.1, 2.5)
    for al, c in zip(A_SWEEP, ['#1f77b4', '#d62728', '#2ca02c']):
        Tg, _, Xg = grids[al]
        ts = [best_feasible(Tg, Xg, L) for L in Ls]
        ax.step(Ls, [b[2] if b else np.nan for b in ts], where='post', color=c,
                label=f'a_launch = {al:.0f} m/s$^2$')
    ax.axvline(L_ZONE, color='orange', ls='--', label=f'baseline zone {L_ZONE:.0f} m')
    ax.set_xlabel('Exchange-zone length L (m)'); ax.set_ylabel('Fastest feasible race time (s)')
    ax.set_title('Speed optimum vs zone length and launch acceleration (KF, seed 0, 2 m/s grid)')
    ax.legend(fontsize=8)
    path = os.path.join(out_dir, 'zone_sensitivity.png')
    fig.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def gif_indices(n, step):
    return [min(i * step, n - 1) for i in range(math.ceil((n - 1) / step) + 1)]


def save_animation(rk, out_dir, step=10):
    h = rk['hist']
    n = len(h['t'])
    idx = gif_indices(n, step)
    fig, (ax, axz) = plt.subplots(2, 1, figsize=(12, 6), gridspec_kw=dict(height_ratios=[2, 1]))
    for xrp, lbl in [(RELAY_XS[1], 'RP1'), (RELAY_XS[2], 'RP2')]:
        ax.axvspan(xrp, xrp + L_ZONE, color='orange', alpha=0.15)
        ax.text(xrp + 2, 4.0, lbl + ' zone', fontsize=8, color='darkorange')
    ax.axvline(0, color='k'); ax.axvline(300, color='gold', lw=2)
    ax.set_xlim(-10, 320); ax.set_ylim(-5, 5)
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')
    dots = [ax.plot([], [], 'o', color=c, ms=8, label=f'Drone {nm}')[0]
            for c, nm in zip(DRONE_COLORS, NAMES)]
    baton, = ax.plot([], [], 'D', color='white', mec='k', ms=7, label='Baton')
    ax.legend(fontsize=8, loc='lower left', ncol=4)
    txt = ax.text(0.01, 0.92, '', transform=ax.transAxes, fontsize=9, family='monospace')
    axz.set_xlim(-10, 320); axz.set_ylim(9, 13)
    axz.set_xlabel('x (m)'); axz.set_ylabel('z (m)')
    lines = [axz.plot([], [], '-', color=c, lw=1.2)[0] for c in DRONE_COLORS]
    for xrp in RELAY_XS[1:3]:
        axz.axvspan(xrp, xrp + L_ZONE, color='orange', alpha=0.15)

    def update(fi):
        k = idx[fi]
        for i in range(3):
            dots[i].set_data([h['p'][k, i, 0]], [h['p'][k, i, 1]])
            lines[i].set_data(h['p'][:k + 1, i, 0], h['p'][:k + 1, i, 2])
        b = int(h['holder'][k])
        baton.set_data([h['p'][k, b, 0]], [h['p'][k, b, 1] + 1.2])
        sp = np.linalg.norm(h['v'][k], axis=1)
        txt.set_text(f't = {h["t"][k]:5.2f} s   baton: {NAMES[b]}   speeds A/B/C = '
                     f'{sp[0]:4.1f}/{sp[1]:4.1f}/{sp[2]:4.1f} m/s')
        ax.set_title('S096 relay race — KF + min-snap intercept (seed 0, 12/14/16 m/s)')
        return dots + lines + [baton, txt]

    anim = animation.FuncAnimation(fig, update, frames=len(idx), interval=50, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    anim.save(path, writer='pillow', fps=20, dpi=70)
    plt.close(fig)
    print(f'Saved: {path}  ({len(idx)} frames, step {step}, {n} samples)')


# ── Main ──────────────────────────────────────────────────────────────────────

def describe(tag, r):
    print(f'  [{tag}] finished: {r["finish"] is not None}, race time '
          f'{r["finish"] if r["finish"] is None else round(r["finish"], 2)} s, '
          f'both passes inside zone: {r["zone_ok"]}')
    for j, h in enumerate(r['handoffs']):
        print(f'    RP{j + 1}: pass at t = {h["t"]:.2f} s, x = {h["x"]:.1f} m '
              f'({h["x"] - RELAY_XS[j + 1]:+.1f} m past RP), |dp| = {h["dp"]:.3f} m, '
              f'|dv| = {h["dv"]:.3f} m/s, gap at launch/detection = {h["launch_gap"]:.1f} m'
              + (f', launch peak accel {h["launch_peak"]:.2f} m/s^2'
                 if np.isfinite(h['launch_peak']) else ''))


def mc_line(tag, m):
    f = m['finish']
    ok = np.isfinite(f)
    s = f'  {tag:30s} finished {ok.sum()}/{len(f)}, both passes in zone {m["zone_ok"].sum()}/{len(f)}'
    if ok.any():
        s += (f', race time {np.nanmean(f):.2f} ± {np.nanstd(f):.2f} s '
              f'(min {np.nanmin(f):.2f}, max {np.nanmax(f):.2f})')
    for j in range(2):
        x = m['hx'][:, j]
        n_ok = np.isfinite(x).sum()
        s += (f', RP{j + 1} pass +{np.nanmean(x):.1f} ± {np.nanstd(x):.1f} m ({n_ok} seeds)'
              if n_ok else f', RP{j + 1} never passed')
    print(s)


if __name__ == '__main__':
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    print('=== S096 Drone Relay Race ===')
    # card-code check: re-planning every step and evaluating the new plan at t_local = 0
    c = snap_coeffs(np.array([100.0, 0, 12]), np.zeros(3), np.array([95.0, 0, 10]),
                    np.array([12.0, 0, 0]), 0.5)
    pr, vr, ar, _ = snap_eval(c, 0.0, 0.5)
    a0 = KP_F * (pr - np.array([100.0, 0, 12])) + KD_F * (vr - np.zeros(3)) + ar
    print(f'Card code check: command at t_local = 0 of a fresh plan = {np.linalg.norm(a0):.2e} m/s^2 '
          f'(boundary conditions force p_ref = p, v_ref = v, a_ref = 0)')
    for vin in (12.0, 14.0, 16.0, 20.0):
        T = SNAP_PEAK * vin / A_LAUNCH
        print(f'Min-snap rest -> {vin:.0f} m/s at peak {A_LAUNCH:.1f} m/s^2: {T:.2f} s, '
              f'rendezvous {vin * T / 2:.1f} m past the relay point')
    print('Pass position ~ rendezvous point = SNAP_PEAK v^2 / (2 a_launch); largest incoming '
          'speed whose pass fits a zone L: v_lim = sqrt(2 a_launch L / SNAP_PEAK):')
    for al in A_SWEEP:
        print(f'  a_launch {al:.0f} m/s^2: ' + ', '.join(
            f'L {L:.0f} m -> v_lim {math.sqrt(2 * al * L / SNAP_PEAK):.2f} m/s' for L in L_SWEEP))

    rk = run_simulation('kf', V_CRUISE, seed=0, record=True)
    rn = run_simulation('naive', V_CRUISE, seed=0, record=True)
    print('\nSeed 0, card speeds 12/14/16 m/s:')
    describe('KF + intercept', rk)
    describe('Naive', rn)
    r15 = run_simulation('kf15', V_CRUISE, seed=0)
    describe(f'KF, card {RP_TRIGGER:.0f} m trigger', r15)
    for j, h in enumerate(rk['handoffs']):
        v_in = V_CRUISE[j]
        print(f'  KF RP{j + 1}: predicted pass offset SNAP_PEAK v^2/(2 a) = '
              f'{SNAP_PEAK * v_in ** 2 / (2 * A_LAUNCH):.1f} m, simulated '
              f'{h["x"] - RELAY_XS[j + 1]:.1f} m')

    print('\nSpeed grid (KF, seed 0, v_C = 20 m/s):')
    grids = {al: speed_grid(al) for al in A_SWEEP}
    T, Z, X = grids[A_LAUNCH]
    feas = np.where(Z, T, np.inf)
    i, j = np.unravel_index(np.argmin(feas), feas.shape)
    best = (GRID_V[i], GRID_V[j])
    print(f'  finished {np.isfinite(T).sum()}/{T.size}, finished with both passes in zone '
          f'{Z.sum()}/{T.size}')
    print(f'  fastest feasible: v_A = {best[0]:.0f}, v_B = {best[1]:.0f}, v_C = '
          f'{V_CRUISE_MAX:.0f} m/s -> {T[i, j]:.2f} s')
    k12, k14 = int(np.where(GRID_V == 12)[0][0]), int(np.where(GRID_V == 14)[0][0])
    print(f'  (12, 14, 20): {T[k12, k14]:.2f} s, zone ok {Z[k12, k14]}')
    fin = np.where(np.isfinite(T), T, np.inf)
    ii, jj = np.unravel_index(np.argmin(fin), fin.shape)
    print(f'  fastest ignoring the zone: v_A = {GRID_V[ii]:.0f}, v_B = {GRID_V[jj]:.0f} -> '
          f'{T[ii, jj]:.2f} s')
    print('  max feasible v_B for each v_A: ' + ', '.join(
        f'{GRID_V[a]:.0f}->{GRID_V[np.where(Z[a])[0]].max():.0f}' if Z[a].any() else
        f'{GRID_V[a]:.0f}->none' for a in range(len(GRID_V))))
    print('  Sensitivity of the fastest feasible (v_A, v_B) -> time to a_launch and L_ZONE:')
    for al in A_SWEEP:
        Tg, _, Xg = grids[al]
        cells = []
        for L in L_SWEEP:
            b = best_feasible(Tg, Xg, L)
            cells.append(f'L {L:.0f}: ' + (f'({b[0]:.0f}, {b[1]:.0f}) {b[2]:.2f} s' if b
                                           else 'none'))
        print(f'    a_launch {al:.0f} m/s^2: ' + '; '.join(cells))

    v_opt = [best[0], best[1], V_CRUISE_MAX]
    print(f'\nMonte Carlo ({N_MC} seeds; same disturbance and sensor noise for every strategy):')
    mc = {('naive', 'card'): monte_carlo('naive', V_CRUISE),
          ('kf', 'card'): monte_carlo('kf', V_CRUISE),
          ('naive', 'opt'): monte_carlo('naive', v_opt),
          ('kf', 'opt'): monte_carlo('kf', v_opt)}
    mc[('kf15', 'card')] = monte_carlo('kf15', V_CRUISE)
    for (s, sp), m in mc.items():
        mc_line(f'{STRAT_LABELS[s]} @ {"12/14/16" if sp == "card" else "opt"}', m)
    f15, fk = mc[('kf15', 'card')]['finish'], mc[('kf', 'card')]['finish']
    print(f'  Card 15 m trigger vs free launch (KF, 12/14/16): mean race time difference '
          f'{np.nanmean(f15 - fk):+.2f} s; launch gaps (seed 0) '
          + ', '.join(f'{h["launch_gap"]:.1f} m' for h in r15['handoffs']))
    for sp in ('card', 'opt'):
        fk, fn = mc[('kf', sp)]['finish'], mc[('naive', sp)]['finish']
        fk2, fn2 = np.where(np.isfinite(fk), fk, np.inf), np.where(np.isfinite(fn), fn, np.inf)
        zk, zn = mc[('kf', sp)]['zone_ok'], mc[('naive', sp)]['zone_ok']
        print(f'  [{sp}] KF faster than naive in {(fk2 < fn2).sum()}/{N_MC} seeds; '
              + (f'mean saving {np.nanmean(fn - fk):.2f} s over races both finished; '
                 if np.isfinite(fn - fk).any() else 'naive never finished; ')
              + f'zone: KF-only {int((zk & ~zn).sum())}, '
              f'naive-only {int((~zk & zn).sum())}, both {int((zk & zn).sum())}')
    dps = [h['dp'] for r in mc[('kf', 'card')]['rows'] for h in r['handoffs']]
    dvs = [h['dv'] for r in mc[('kf', 'card')]['rows'] for h in r['handoffs']]
    print(f'  KF @ 12/14/16 at baton pass: |dp| max {max(dps):.3f} m, |dv| max {max(dvs):.3f} m/s')

    print('\nNaive detection-range sweep (12/14/16 m/s, same seeds):')
    sweep = {}
    for d in DETECT_SWEEP:
        sweep[d] = mc[('naive', 'card')] if d == D_DETECT else monte_carlo('naive', V_CRUISE,
                                                                         d_detect=d)
        mc_line(f'naive d_detect = {d:g} m', sweep[d])
    print('  Both passes inside a zone of L (seeds out of 30) for naive d_detect / KF:')
    for L in L_SWEEP:
        cnt = [int(np.sum(np.nanmax(np.nan_to_num(sweep[d]['hx'], nan=np.inf), axis=1) <= L))
               for d in DETECT_SWEEP]
        kfc = int(np.sum(np.nanmax(np.nan_to_num(mc[('kf', 'card')]['hx'], nan=np.inf), axis=1)
                         <= L))
        print(f'    L {L:.0f} m: ' + ', '.join(f'd {d:g} -> {c}' for d, c in zip(DETECT_SWEEP, cnt))
              + f'; KF -> {kfc}')
    beyond = int(np.sum(sweep[D_DETECT]['hx'][:, 1] + RELAY_XS[2] > RELAY_XS[3]))
    print(f'  Naive d_detect = {D_DETECT:.0f} m: second pass beyond the 300 m finish in {beyond}/'
          f'{N_MC} seeds (the race then ends at the pass, so leg 3 is never flown)')

    print()
    plot_race_3d(rk, rn, OUTPUT_DIR)
    plot_handoff_detail(rk, rn, 0, OUTPUT_DIR)
    plot_handoff_detail(rk, rn, 1, OUTPUT_DIR)
    plot_monte_carlo(mc, sweep, OUTPUT_DIR)
    plot_speed_grid(T, Z, best, OUTPUT_DIR)
    plot_sensitivity(grids, OUTPUT_DIR)
    save_animation(rk, OUTPUT_DIR)
