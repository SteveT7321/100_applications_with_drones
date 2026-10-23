"""
S032 3D Upgrade - Charging Queue
================================
Six delivery drones share a charging hub with L = 3 stacked pad levels
(z = 1.5, 3.0, 4.5 m, one pad per level).  Drones cruise at z = 5 m, deliver to
3-D waypoints in [-30, 30]^2 x [1, 6] m and fly multi-drop sorties (as in the
fixed 2D S032): after each delivery they go on to the next one and return only
when their charge rule says so.

Flight protocol (card lanes): every leg is
    vertical to cruise altitude -> horizontal at cruise -> vertical to target z.
A returning drone flies to the hub hold point (0, 0, 5), is assigned a pad, flies
2 m to the top of that pad's vertical shaft and descends to the pad.  After a full
charge it climbs the shaft back to cruise; the shaft (and pad) is released only
when the drone is back at cruise, so one shaft never holds a climbing and a
descending drone at the same time.

Energy (card): dE = alpha (m_b + m_p) g d_xy + gamma (m_b + m_p) g |dz|, gamma = 2 alpha.

Return rules compared (same delivery streams for every rule, 20 seeds):
  flat_thr   - flat-world threshold: abort and return when SoC <= beta
  thr3d      - 3D-aware threshold: return when SoC - E_vert,return(i) <= beta
  pred_flat  - flat predictive (added baseline): SoC <= alpha' d_xy(i, hub) + beta_safe
  pred3d     - predictive 3D (card): SoC <= min_l s_min^3D(i, l)
Pad dispatch rules compared:
  fifo_rand  - one shared FIFO queue, altitude-blind: a free pad is picked at random
  fifo_near  - one shared FIFO queue, the free pad with the shortest descent is taken (added)
  alt_prio   - card: per-level queues, level l* = argmin_l t_queue(l) + |z_c - z_l| / v_z

Usage:
    conda activate drones
    python src/02_logistics_delivery/3d/s032_3d_charging_queue.py
"""

import matplotlib
matplotlib.use('Agg')

import os
import math
from collections import deque
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.lines import Line2D

OUTPUT_DIR = os.path.normpath(os.path.join(
    os.path.dirname(__file__), '..', '..', '..',
    'outputs', '02_logistics_delivery', '3d', 's032_3d_charging_queue',
))

# ── Parameters ────────────────────────────────────────────────────
N_DRONES   = 6
PAD_Z      = np.array([1.5, 3.0, 4.5])   # m, card pad levels (one pad each)
N_LEVELS   = len(PAD_Z)
Z_CRUISE   = 5.0            # m
Z_MIN, Z_MAX = 0.5, 6.0     # m, card altitude bounds
WP_Z_LO, WP_Z_HI = 1.0, 6.0 # m, card delivery volume z range
GRID_HALF  = 30.0           # m
T_SIM      = 300.0          # s
DT         = 0.5            # s
V_H        = 8.0            # m/s horizontal cruise speed
V_Z        = 2.0            # m/s max vertical speed
E_CAP      = 1.0
P_CHG      = 0.04           # E_cap/s (2D-fixed value; card 0.25)
ALPHA      = 1.8e-4         # 1/m horizontal discharge constant
GAMMA      = 3.6e-4         # 1/m vertical discharge constant
M_BODY     = 1.5            # kg
M_PAYLOAD  = 0.5            # kg
G          = 9.81           # m/s^2
BETA_SAFE  = 0.05
BETA_SWEEP = np.round(np.arange(0.10, 0.55, 0.05), 2)
HOLD       = np.array([0.0, 0.0, Z_CRUISE])   # hub hold point (card p_hold)
R_SHAFT    = 2.0            # m, horizontal offset of each pad shaft from the hold point
SHAFT_ANG  = np.deg2rad([90.0, 210.0, 330.0])
SHAFT_XY   = np.column_stack([R_SHAFT * np.cos(SHAFT_ANG), R_SHAFT * np.sin(SHAFT_ANG)])
T_CHG_PRIOR = 0.8 / P_CHG   # s, prior for the mean charge time before any docking
SEED0      = 42
N_SEEDS    = 20
N_WAYPOINTS = 800           # pre-drawn delivery points per seed (asserted sufficient)
EPS        = 1e-9

A_H = ALPHA * (M_BODY + M_PAYLOAD) * G / E_CAP   # SoC per horizontal metre
A_V = GAMMA * (M_BODY + M_PAYLOAD) * G / E_CAP   # SoC per vertical metre

LEVEL_COLORS = ['#1b9e77', '#d95f02', '#7570b3']
DRONE_COLORS = ['#e6194b', '#3cb44b', '#4363d8', '#f58231', '#911eb4', '#42d4f4']
STATE_LIST = ['flying', 'returning', 'waiting', 'docking', 'charging', 'ascending', 'stranded']
TRANSIT = ('flying', 'returning', 'docking', 'ascending')


# ── Geometry and energy helpers ───────────────────────────────────
def leg_path(p, q):
    """Card lane protocol p -> q: vertical to cruise, horizontal, vertical to q's z."""
    pts = []
    cur = np.array(p, dtype=float)
    if abs(cur[2] - Z_CRUISE) > EPS:
        cur = np.array([cur[0], cur[1], Z_CRUISE]); pts.append(cur)
    if np.hypot(q[0] - cur[0], q[1] - cur[1]) > EPS:
        cur = np.array([q[0], q[1], Z_CRUISE]); pts.append(cur)
    if abs(q[2] - cur[2]) > EPS:
        pts.append(np.array(q, dtype=float))
    return pts


def path_energy(p, pts, gamma_on=True):
    """SoC needed to fly the polyline p -> pts (horizontal + vertical terms)."""
    e, cur = 0.0, np.array(p, dtype=float)
    for q in pts:
        e += A_H * np.hypot(q[0] - cur[0], q[1] - cur[1])
        if gamma_on:
            e += A_V * abs(q[2] - cur[2])
        cur = q
    return e


def shaft_top(l):
    return np.array([SHAFT_XY[l, 0], SHAFT_XY[l, 1], Z_CRUISE])


def pad_pos(l):
    return np.array([SHAFT_XY[l, 0], SHAFT_XY[l, 1], PAD_Z[l]])


def s_min_3d(p, l, gamma_on=True):
    """Card s_min^3D(i, l): transit to the hold point, 2 m to shaft l, descent, + beta_safe."""
    e = path_energy(p, leg_path(p, HOLD), gamma_on)
    e += A_H * R_SHAFT + (A_V * abs(Z_CRUISE - PAD_Z[l]) if gamma_on else 0.0)
    return e + BETA_SAFE


def e_vert_return(p):
    """Vertical part of the cheapest 3-D return: |z - z_c| + min_l |z_c - z_l| (gamma term)."""
    return A_V * (abs(p[2] - Z_CRUISE) + np.min(np.abs(Z_CRUISE - PAD_Z)))


def draw_waypoints(seed):
    rng = np.random.default_rng(seed)
    xy = rng.uniform(-GRID_HALF, GRID_HALF, (N_WAYPOINTS, 2))
    z = rng.uniform(WP_Z_LO, WP_Z_HI, N_WAYPOINTS)
    return np.column_stack([xy, z])


# ── Simulation ────────────────────────────────────────────────────
def run_simulation(rule='thr3d', beta=0.15, queue_rule='fifo_rand', seed=SEED0,
                   gamma_on=True, record=False):
    """
    One 300 s run.  rule in {flat_thr, thr3d, pred_flat, pred3d};
    queue_rule in {fifo_rand, fifo_near, alt_prio}; gamma_on=False switches the
    vertical energy term off in both physics and planning (counterfactual).
    """
    wps = draw_waypoints(seed)
    rng_q = np.random.default_rng(seed + 10_000)    # only used by fifo_rand
    next_wp = 0
    requeue = deque()                                # aborted / lost parcels, served first

    def pop_mission():
        nonlocal next_wp
        if requeue:
            return requeue.popleft()
        assert next_wp < N_WAYPOINTS, 'waypoint pool exhausted'
        q = wps[next_wp].copy(); next_wp += 1
        return q

    soc = np.ones(N_DRONES)
    pos = np.tile(HOLD, (N_DRONES, 1)).astype(float)
    state = ['flying'] * N_DRONES
    path = [[] for _ in range(N_DRONES)]
    goal = [None] * N_DRONES             # current delivery point
    level = [-1] * N_DRONES              # assigned pad level
    for i in range(N_DRONES):            # all six launch from the hold point at t = 0
        goal[i] = pop_mission(); path[i] = leg_path(pos[i], goal[i])

    shaft_busy = [None] * N_LEVELS       # drone occupying shaft/pad l (docking, charging, ascending)
    charge_rem = [0.0] * N_LEVELS
    shared_q = deque()
    level_q = [deque() for _ in range(N_LEVELS)]
    arrive_deficits = []

    deliveries = np.zeros(N_DRONES, dtype=int)
    fly_time = np.zeros(N_DRONES)
    stranded = aborted = 0
    wait_time = 0.0
    visits = np.zeros(N_LEVELS, dtype=int)
    pad_busy_time = np.zeros(N_LEVELS)
    e_h_used = e_v_used = 0.0
    idle_pad_while_waiting = 0.0         # pad-seconds a pad sat free while some drone waited

    hist = {'pos': [pos.copy()], 'soc': [soc.copy()], 'state': [list(state)],
            'nq': [], 'occ': [], 'events': []} if record else None

    def soc_floor(i):
        p = pos[i]
        if rule == 'flat_thr':
            return soc[i] <= beta
        if rule == 'thr3d':
            return soc[i] - (e_vert_return(p) if gamma_on else 0.0) <= beta
        if rule == 'pred_flat':
            return soc[i] <= A_H * np.hypot(p[0], p[1]) + BETA_SAFE
        if rule == 'pred3d':
            return soc[i] <= min(s_min_3d(p, l, gamma_on) for l in range(N_LEVELS))
        raise ValueError(rule)

    def advance(i):
        """Move drone i along its path for DT; returns True when the path is finished."""
        nonlocal stranded, e_h_used, e_v_used
        t_left = DT
        while path[i] and t_left > EPS:
            q = path[i][0]
            d = q - pos[i]
            dxy, dz = float(np.hypot(d[0], d[1])), abs(float(d[2]))
            assert dxy < 1e-6 or dz < 1e-6, 'segments are purely horizontal or vertical'
            if dxy + dz < EPS:
                pos[i] = q.copy(); path[i].pop(0); continue
            horiz = dxy >= dz
            seg = dxy if horiz else dz
            t_need = seg / (V_H if horiz else V_Z)
            frac = min(1.0, t_left / t_need)
            rate = (A_H * dxy) if horiz else (A_V * dz if gamma_on else 0.0)
            cost = rate * frac
            if cost > soc[i] + 1e-12:                       # battery empties on this segment
                f0 = soc[i] / rate
                pos[i] = pos[i] + d * f0
                if horiz: e_h_used += soc[i]
                else: e_v_used += soc[i]
                soc[i] = 0.0
                if state[i] == 'flying':
                    requeue.appendleft(goal[i])            # parcel lost, order re-queued
                state[i] = 'stranded'; path[i] = []
                stranded += 1
                return False
            soc[i] -= cost
            if horiz: e_h_used += cost
            else: e_v_used += cost
            t_left -= t_need * frac
            if frac >= 1.0:
                pos[i] = q.copy(); path[i].pop(0)
            else:
                pos[i] = pos[i] + d * frac
        return not path[i]

    def t_chg_mean():
        return (np.mean(arrive_deficits) / P_CHG) if arrive_deficits else T_CHG_PRIOR

    def start_docking(i, l):
        assert shaft_busy[l] is None
        shaft_busy[l] = i; level[i] = l; state[i] = 'docking'
        path[i] = [shaft_top(l), pad_pos(l)]

    def arrive_hold(i):
        free = [l for l in range(N_LEVELS) if shaft_busy[l] is None]
        if queue_rule == 'alt_prio':
            tq = t_chg_mean()
            cost = [(len(level_q[l]) + (shaft_busy[l] is not None)) * tq
                    + abs(Z_CRUISE - PAD_Z[l]) / V_Z for l in range(N_LEVELS)]
            l_star = int(np.argmin(cost))
            if shaft_busy[l_star] is None and not level_q[l_star]:
                start_docking(i, l_star)
            else:
                level_q[l_star].append(i); state[i] = 'waiting'; level[i] = l_star
        else:
            if free and not shared_q:
                if queue_rule == 'fifo_rand':
                    l = int(free[rng_q.integers(len(free))])
                else:   # fifo_near: shortest descent
                    l = max(free, key=lambda k: PAD_Z[k])
                start_docking(i, l)
            else:
                shared_q.append(i); state[i] = 'waiting'

    def release_shaft(l):
        shaft_busy[l] = None
        if queue_rule == 'alt_prio':
            if level_q[l]:
                start_docking(level_q[l].popleft(), l)
        elif shared_q:
            start_docking(shared_q.popleft(), l)

    n_steps = int(round(T_SIM / DT))
    for step in range(n_steps):
        t_now = step * DT
        # mid-leg return decision (flying drones only)
        for i in range(N_DRONES):
            if state[i] == 'flying' and soc_floor(i):
                requeue.appendleft(goal[i]); aborted += 1
                state[i] = 'returning'; path[i] = leg_path(pos[i], HOLD)

        for i in range(N_DRONES):
            st = state[i]
            if st in TRANSIT:
                fly_time[i] += DT
                done = advance(i)
                if state[i] == 'stranded':
                    if st in ('docking', 'ascending'):     # crashed inside a shaft: free it
                        l = level[i]; level[i] = -1
                        release_shaft(l)
                    continue
                if not done:
                    continue
                if st == 'flying':
                    deliveries[i] += 1
                    if soc_floor(i):
                        state[i] = 'returning'; path[i] = leg_path(pos[i], HOLD)
                    else:
                        goal[i] = pop_mission(); path[i] = leg_path(pos[i], goal[i])
                elif st == 'returning':
                    arrive_hold(i)
                elif st == 'docking':
                    l = level[i]
                    assert np.allclose(pos[i], pad_pos(l))
                    state[i] = 'charging'; visits[l] += 1
                    arrive_deficits.append(1.0 - soc[i])
                    charge_rem[l] = (1.0 - soc[i]) * E_CAP / P_CHG
                    if record:
                        hist['events'].append((t_now + DT, i, l, float(soc[i])))
                elif st == 'ascending':
                    l = level[i]; level[i] = -1
                    goal[i] = pop_mission()
                    state[i] = 'flying'; path[i] = leg_path(pos[i], goal[i])
                    release_shaft(l)
            elif st == 'waiting':
                wait_time += DT

        for l in range(N_LEVELS):
            i = shaft_busy[l]
            if i is not None and state[i] == 'charging':
                pad_busy_time[l] += DT
                charge_rem[l] -= DT
                soc[i] = min(1.0, soc[i] + P_CHG * DT / E_CAP)
                if charge_rem[l] <= 1e-9:
                    soc[i] = 1.0
                    state[i] = 'ascending'; path[i] = [shaft_top(l)]

        n_wait = sum(1 for s in state if s == 'waiting')
        n_free = sum(1 for l in range(N_LEVELS) if shaft_busy[l] is None)
        if n_wait:
            idle_pad_while_waiting += DT * n_free

        # invariants
        assert np.all(soc >= -1e-12) and np.all(soc <= 1.0 + 1e-12)
        for i in range(N_DRONES):
            assert Z_MIN - 1e-9 <= pos[i, 2] <= Z_MAX + 1e-9, 'altitude out of bounds'
            if state[i] == 'charging':
                assert shaft_busy[level[i]] == i and np.allclose(pos[i], pad_pos(level[i]))
        occupants = [i for i in shaft_busy if i is not None]
        assert len(occupants) == len(set(occupants))
        for l in range(N_LEVELS):        # a shaft never holds two drones
            inside = [i for i in range(N_DRONES) if state[i] in ('docking', 'charging', 'ascending')
                      and level[i] == l]
            assert len(inside) <= 1

        if record:
            hist['pos'].append(pos.copy()); hist['soc'].append(soc.copy())
            hist['state'].append(list(state))
            if queue_rule == 'alt_prio':
                hist['nq'].append([len(q) for q in level_q] + [0])
            else:
                hist['nq'].append([0, 0, 0, len(shared_q)])
            hist['occ'].append([shaft_busy[l] is not None for l in range(N_LEVELS)])

    out = {'deliveries': int(deliveries.sum()),
           'utilisation': float(np.mean(fly_time / T_SIM)),
           'stranded': stranded, 'aborted': aborted,
           'charge_visits': int(visits.sum()), 'visits': visits.copy(),
           'wait_time': wait_time, 'pad_busy': pad_busy_time / T_SIM,
           'vert_share': e_v_used / max(e_h_used + e_v_used, EPS),
           'idle_pad_wait': idle_pad_while_waiting,
           'energy_used': e_h_used + e_v_used}
    if record:
        for k in ('pos', 'soc', 'nq', 'occ'):
            hist[k] = np.array(hist[k])
        out['hist'] = hist
        out['per_drone'] = deliveries.copy()
    return out


KEYS = ['deliveries', 'utilisation', 'stranded', 'aborted', 'charge_visits',
        'wait_time', 'vert_share', 'idle_pad_wait', 'energy_used']


def monte_carlo(rule, beta=0.0, queue_rule='fifo_rand', gamma_on=True):
    """Same N_SEEDS delivery streams for every configuration."""
    runs = [run_simulation(rule, beta, queue_rule, SEED0 + s, gamma_on) for s in range(N_SEEDS)]
    res = {k: float(np.mean([r[k] for r in runs])) for k in KEYS}
    res['deliveries_std'] = float(np.std([r['deliveries'] for r in runs]))
    res['per_seed'] = np.array([r['deliveries'] for r in runs])
    res['stranded_seeds'] = int(sum(r['stranded'] > 0 for r in runs))
    res['visits'] = np.mean([r['visits'] for r in runs], axis=0)
    res['pad_busy'] = np.mean([r['pad_busy'] for r in runs], axis=0)
    return res


def paired(a, b):
    d = a['per_seed'] - b['per_seed']
    return float(d.mean()), float(d.std()), int((d > 0).sum()), int((d == 0).sum()), int((d < 0).sum())


# ── Plots ─────────────────────────────────────────────────────────
def _draw_hub(ax, half=3.2, alpha=0.18):
    for l in range(N_LEVELS):
        xx, yy = np.meshgrid([-half, half], [-half, half])
        ax.plot_surface(xx, yy, np.full_like(xx, PAD_Z[l]), color=LEVEL_COLORS[l], alpha=alpha,
                        shade=False)
        ax.scatter(*pad_pos(l), color=LEVEL_COLORS[l], marker='s', s=50, edgecolors='k')
    ax.scatter(*HOLD, color='k', marker='*', s=80)


def plot_trajectories_3d(rec, out_dir):
    h = rec['hist']
    n_show = int(round(120.0 / DT)) + 1
    fig = plt.figure(figsize=(16, 7.5))
    ax = fig.add_subplot(1, 2, 1, projection='3d')
    for i in range(N_DRONES):
        p = h['pos'][:n_show, i]
        ax.plot(p[:, 0], p[:, 1], p[:, 2], color=DRONE_COLORS[i], lw=0.9)
    _draw_hub(ax)
    ax.set_xlim(-GRID_HALF, GRID_HALF); ax.set_ylim(-GRID_HALF, GRID_HALF); ax.set_zlim(0, 6.5)
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
    ax.set_title('All drones, first 120 s (lane protocol: vertical / cruise / vertical)', fontsize=10)
    ax = fig.add_subplot(1, 2, 2, projection='3d')
    hub_states = ('returning', 'waiting', 'docking', 'charging', 'ascending')
    for i in range(N_DRONES):
        p = h['pos'][:, i]
        st = np.array([s[i] in hub_states for s in h['state']])
        m = st & (np.abs(p[:, 0]) <= 4.5) & (np.abs(p[:, 1]) <= 4.5)
        pp = np.where(m[:, None], p, np.nan)
        ax.plot(pp[:, 0], pp[:, 1], pp[:, 2], color=DRONE_COLORS[i], lw=1.2)
    _draw_hub(ax, half=3.2, alpha=0.25)
    for l in range(N_LEVELS):
        ax.plot([SHAFT_XY[l, 0]] * 2, [SHAFT_XY[l, 1]] * 2, [PAD_Z[l], Z_CRUISE],
                color=LEVEL_COLORS[l], ls=':', lw=1.5)
    ax.set_xlim(-4.5, 4.5); ax.set_ylim(-4.5, 4.5); ax.set_zlim(0, 6.5)
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
    ax.set_title('Hub zoom, whole run, hub phases only (return, hold, shaft descent, climb)', fontsize=10)
    handles = [Line2D([0], [0], color=DRONE_COLORS[i], lw=2, label=f'D{i}') for i in range(N_DRONES)]
    handles += [mpatches.Patch(color=LEVEL_COLORS[l], label=f'pad level {PAD_Z[l]:.1f} m')
                for l in range(N_LEVELS)]
    fig.legend(handles=handles, loc='lower center', ncol=9, fontsize=8)
    fig.suptitle(f"S032 3D - trajectories ({rec['label']}, seed {SEED0})", fontsize=12)
    plt.tight_layout(rect=[0, 0.06, 1, 0.95])
    path = os.path.join(out_dir, 'trajectories_3d.png')
    plt.savefig(path, dpi=130); plt.close()
    print(f'Saved: {path}')


def plot_altitude_soc(rec, out_dir):
    h = rec['hist']
    t = np.arange(h['pos'].shape[0]) * DT
    fig, axes = plt.subplots(2, 1, figsize=(13, 8.5), sharex=True)
    ax = axes[0]
    for i in range(N_DRONES):
        ax.plot(t, h['pos'][:, i, 2], color=DRONE_COLORS[i], lw=0.9, label=f'D{i}')
    for l in range(N_LEVELS):
        ax.axhline(PAD_Z[l], color=LEVEL_COLORS[l], ls='--', lw=1, label=f'pad {PAD_Z[l]:.1f} m')
    ax.axhline(Z_CRUISE, color='k', ls=':', lw=1, label='cruise 5.0 m')
    ax.set_ylabel('Altitude z (m)'); ax.set_ylim(0, 6.5); ax.grid(alpha=0.3)
    ax.set_title('Altitude time series: descents to delivery points and to the assigned pad level')
    ax.legend(ncol=10, fontsize=7, loc='lower right')
    ax = axes[1]
    for i in range(N_DRONES):
        ax.plot(t, h['soc'][:, i], color=DRONE_COLORS[i], lw=0.9)
    for (te, i, l, s) in h['events']:
        ax.plot(te, s, marker='v', color=LEVEL_COLORS[l], mec='k', ms=7)
    ax.axhline(BETA_SAFE, color='red', ls=':', lw=0.9)
    ax.set_ylabel('SoC'); ax.set_xlabel('Time (s)'); ax.set_ylim(-0.03, 1.05); ax.grid(alpha=0.3)
    ax.set_title('SoC traces; triangles = docking, coloured by pad level (green 1.5, orange 3.0, '
                 'purple 4.5 m)')
    fig.suptitle(f"S032 3D - {rec['label']}, seed {SEED0}: {rec['deliveries']} deliveries, "
                 f"{rec['stranded']} stranded", fontsize=12)
    plt.tight_layout()
    path = os.path.join(out_dir, 'altitude_soc.png')
    plt.savefig(path, dpi=130); plt.close()
    print(f'Saved: {path}')


def plot_threshold_sweep(sweep, preds, best_safe, out_dir):
    fig, axes = plt.subplots(1, 2, figsize=(13, 4.8))
    cols = {'flat_thr': '#4c9be8', 'thr3d': '#e8554c'}
    names = {'flat_thr': 'flat-world threshold', 'thr3d': '3D-aware threshold'}
    ax = axes[0]
    for rule in ('flat_thr', 'thr3d'):
        m = [sweep[rule][b]['deliveries'] for b in BETA_SWEEP]
        s = [sweep[rule][b]['deliveries_std'] for b in BETA_SWEEP]
        ax.errorbar(BETA_SWEEP, m, yerr=s, marker='o', capsize=3, color=cols[rule],
                    label=f'{names[rule]} (best safe beta={best_safe[rule]:.2f})')
    for key, ls, c in (('pred_flat', ':', '#7f7f7f'), ('pred3d', '--', '#2ca02c')):
        ax.axhline(preds[key]['deliveries'], color=c, ls=ls,
                   label=f"{key} (no beta) = {preds[key]['deliveries']:.1f}")
    ax.set_xlabel('beta'); ax.set_ylabel(f'Mean total deliveries ({N_SEEDS} seeds)')
    ax.set_xticks(BETA_SWEEP); ax.grid(alpha=0.3); ax.legend(fontsize=8)
    ax.set_title('Deliveries vs return threshold (shared FIFO, random free pad)')
    ax = axes[1]
    for rule in ('flat_thr', 'thr3d'):
        ax.plot(BETA_SWEEP, [sweep[rule][b]['stranded'] for b in BETA_SWEEP], marker='s',
                color=cols[rule], label=names[rule])
    for key, ls, c in (('pred_flat', ':', '#7f7f7f'), ('pred3d', '--', '#2ca02c')):
        ax.axhline(preds[key]['stranded'], color=c, ls=ls, label=key)
    ax.set_xlabel('beta'); ax.set_ylabel('Mean stranded drones per run')
    ax.set_xticks(BETA_SWEEP); ax.grid(alpha=0.3); ax.legend(fontsize=8)
    ax.set_title('Stranded drones (battery empty before reaching the pad)')
    fig.suptitle('S032 3D - threshold sweep: flat-world vs 3D-aware', fontsize=12)
    plt.tight_layout()
    path = os.path.join(out_dir, 'threshold_sweep.png')
    plt.savefig(path, dpi=130); plt.close()
    print(f'Saved: {path}')


def plot_policy_comparison(table, out_dir):
    labels = [r[0] for r in table]
    cols = ['#4c9be8', '#e8554c', '#9467bd', '#7f7f7f', '#2ca02c', '#bcbd22']
    panels = [('deliveries', 'Mean total deliveries', '{:.1f}', 1),
              ('utilisation', 'Fleet utilisation (%)', '{:.1f}', 100),
              ('stranded', 'Mean stranded drones', '{:.2f}', 1),
              ('wait_time', 'Queue waiting (drone-s)', '{:.0f}', 1)]
    fig, axes = plt.subplots(1, 4, figsize=(22, 5.2))
    for ax, (key, title, fmt, sc) in zip(axes, panels):
        vals = [r[1][key] * sc for r in table]
        ax.bar(range(len(vals)), vals, color=cols[:len(vals)], edgecolor='white')
        err = [r[1]['deliveries_std'] for r in table] if key == 'deliveries' else [0.0] * len(vals)
        if key == 'deliveries':
            ax.errorbar(range(len(vals)), vals, yerr=err, fmt='none', ecolor='k', capsize=4)
        top = max(max(vals), 1e-6)
        for x, (v, e) in enumerate(zip(vals, err)):
            ax.text(x, v + e + 0.015 * top, fmt.format(v), ha='center', va='bottom', fontsize=8)
        ax.set_xticks(range(len(vals))); ax.set_xticklabels(labels, fontsize=7)
        ax.set_title(title, fontsize=10); ax.grid(axis='y', alpha=0.3)
        ax.set_ylim(0, max(max(vals), 1e-6) * 1.2)
    fig.suptitle(f'S032 3D - policy comparison (mean of {N_SEEDS} seeds, same delivery streams)',
                 fontsize=12)
    plt.tight_layout()
    path = os.path.join(out_dir, 'policy_comparison.png')
    plt.savefig(path, dpi=130); plt.close()
    print(f'Saved: {path}')


def plot_level_queues(rec_fifo, rec_alt, out_dir):
    fig, axes = plt.subplots(2, 1, figsize=(13, 7), sharex=True)
    for ax, rec in zip(axes, (rec_fifo, rec_alt)):
        h = rec['hist']
        t = (np.arange(h['nq'].shape[0]) + 1) * DT
        if rec['queue_rule'] == 'alt_prio':
            for l in range(N_LEVELS):
                ax.step(t, h['nq'][:, l] + 0.04 * l, where='post', color=LEVEL_COLORS[l], lw=1.3,
                        label=f'n_l queue, level {PAD_Z[l]:.1f} m')
        else:
            ax.step(t, h['nq'][:, 3], where='post', color='k', lw=1.3, label='shared FIFO queue')
        for l in range(N_LEVELS):
            occ = h['occ'][:, l].astype(float)
            ax.fill_between(t, -0.6 - 0.35 * l, -0.6 - 0.35 * l + 0.28 * occ, step='post',
                            color=LEVEL_COLORS[l], alpha=0.8)
        ax.axhline(-0.05, color='grey', lw=0.5)
        ax.set_ylabel('Drones waiting'); ax.grid(alpha=0.3); ax.set_ylim(-1.7, 4.5)
        ax.set_title(f"{rec['label']}: {rec['deliveries']} deliveries; bands below 0 = shaft/pad "
                     'occupied (green 1.5, orange 3.0, purple 4.5 m)', fontsize=10)
        ax.legend(fontsize=8, loc='upper right')
    axes[-1].set_xlabel('Time (s)')
    fig.suptitle(f'S032 3D - per-level queue length n_l(t) and pad occupancy (seed {SEED0})',
                 fontsize=12)
    plt.tight_layout()
    path = os.path.join(out_dir, 'level_queues.png')
    plt.savefig(path, dpi=130); plt.close()
    print(f'Saved: {path}')


def save_animation(rec, out_dir, step=4):
    h = rec['hist']
    n = h['pos'].shape[0]
    n_frames = math.ceil((n - 1) / step) + 1
    idx = [min(f * step, n - 1) for f in range(n_frames)]
    st_marker = {'flying': '^', 'returning': 'v', 'waiting': 'o', 'docking': 'v',
                 'charging': 's', 'ascending': '^', 'stranded': 'x'}
    fig = plt.figure(figsize=(9, 7.5))
    ax = fig.add_subplot(111, projection='3d')
    _draw_hub(ax, half=3.2, alpha=0.2)
    ax.set_xlim(-GRID_HALF, GRID_HALF); ax.set_ylim(-GRID_HALF, GRID_HALF); ax.set_zlim(0, 6.5)
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
    trails = [ax.plot([], [], [], color=DRONE_COLORS[i], lw=0.9)[0] for i in range(N_DRONES)]
    dots = [ax.plot([], [], [], '^', color=DRONE_COLORS[i], ms=7, mec='k')[0] for i in range(N_DRONES)]
    title = ax.set_title('')

    def update(f):
        k = idx[f]
        k0 = max(0, k - 30)
        for i in range(N_DRONES):
            p = h['pos'][k0:k + 1, i]
            trails[i].set_data(p[:, 0], p[:, 1]); trails[i].set_3d_properties(p[:, 2])
            dots[i].set_data([p[-1, 0]], [p[-1, 1]]); dots[i].set_3d_properties([p[-1, 2]])
            dots[i].set_marker(st_marker[h['state'][k][i]])
        socs = ' '.join(f'{s:.2f}' for s in h['soc'][k])
        title.set_text(f"S032 3D - {rec['label']}  t = {k * DT:.1f} s\nSoC: {socs}")
        return trails + dots + [title]

    ani = FuncAnimation(fig, update, frames=n_frames, interval=60, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer=PillowWriter(fps=15), dpi=72)
    plt.close()
    print(f'Saved: {path}  ({n_frames} frames)')


# ── Main ──────────────────────────────────────────────────────────
def main():
    out_dir = OUTPUT_DIR
    os.makedirs(out_dir, exist_ok=True)
    print(f'Horizontal discharge = {A_H:.5f} SoC/m, vertical = {A_V:.5f} SoC/m '
          f'(gamma/alpha = {GAMMA / ALPHA:.1f})')
    far = np.array([GRID_HALF, GRID_HALF, WP_Z_LO])
    print(f'Worst-case return from ({far[0]:.0f}, {far[1]:.0f}, {far[2]:.0f}): '
          f'flat s_min = {A_H * np.hypot(far[0], far[1]) + BETA_SAFE:.4f}, '
          f's_min^3D(level 4.5) = {s_min_3d(far, 2):.4f}, s_min^3D(level 1.5) = {s_min_3d(far, 0):.4f}')
    print(f'  vertical part of that return: {e_vert_return(far):.4f} (best level), '
          f'{A_V * (abs(far[2] - Z_CRUISE) + abs(Z_CRUISE - PAD_Z[0])):.4f} (level 1.5)')
    print(f'Pad energy supply = {N_LEVELS} x {P_CHG} x {T_SIM:.0f} s = {N_LEVELS * P_CHG * T_SIM:.0f} E_cap')

    print(f'\nThreshold sweep ({N_SEEDS} seeds, shared FIFO / random free pad):')
    sweep = {'flat_thr': {}, 'thr3d': {}}
    best, best_safe = {}, {}
    for rule in ('flat_thr', 'thr3d'):
        print(f'  {rule}:')
        for b in BETA_SWEEP:
            r = monte_carlo(rule, b)
            sweep[rule][b] = r
            print(f"    beta={b:.2f}  deliveries={r['deliveries']:6.1f} +/- {r['deliveries_std']:4.1f}"
                  f"  stranded={r['stranded']:.2f} ({r['stranded_seeds']:2d} seeds)"
                  f"  aborted={r['aborted']:5.2f}  visits={r['charge_visits']:5.2f}"
                  f"  wait={r['wait_time']:6.1f} s")
        best[rule] = float(BETA_SWEEP[int(np.argmax([sweep[rule][b]['deliveries'] for b in BETA_SWEEP]))])
        safe = [b for b in BETA_SWEEP if sweep[rule][b]['stranded'] == 0.0]
        best_safe[rule] = float(max(safe, key=lambda b: sweep[rule][b]['deliveries']))
        print(f'    beta* = {best[rule]:.2f}; beta*_safe (0 stranded in all seeds) = {best_safe[rule]:.2f}')

    preds = {'pred_flat': monte_carlo('pred_flat'), 'pred3d': monte_carlo('pred3d')}
    bs = best_safe['thr3d']
    q_near = monte_carlo('thr3d', bs, 'fifo_near')
    q_alt = monte_carlo('thr3d', bs, 'alt_prio')
    p_near = monte_carlo('pred3d', 0.0, 'fifo_near')
    p_alt = monte_carlo('pred3d', 0.0, 'alt_prio')
    nog = monte_carlo('pred3d', 0.0, 'fifo_rand', gamma_on=False)

    rows = [
        (f"Flat thr\nbeta={best_safe['flat_thr']:.2f}", sweep['flat_thr'][best_safe['flat_thr']]),
        (f"3D thr\nbeta={bs:.2f}", sweep['thr3d'][bs]),
        (f"Alt-priority\n(3D thr {bs:.2f})", q_alt),
        ('Flat\npredictive', preds['pred_flat']),
        ('Predictive 3D', preds['pred3d']),
        ('Pred 3D +\nalt-priority', p_alt),
    ]
    print('\nPolicy comparison (mean of seeds; Flat thr and 3D thr at their best stranding-free beta):')
    for lab, m in rows:
        print(f"  {lab.replace(chr(10), ' '):<28} deliveries={m['deliveries']:6.1f} +/- "
              f"{m['deliveries_std']:4.1f}  util={m['utilisation'] * 100:5.1f}%  "
              f"stranded={m['stranded']:.2f} ({m['stranded_seeds']} seeds)  aborted={m['aborted']:5.2f}  "
              f"visits={m['charge_visits']:5.2f}  wait={m['wait_time']:6.1f} drone-s  "
              f"vertical energy share={m['vert_share'] * 100:4.1f}%")
    ft2d = sweep['flat_thr'][0.15]
    print(f"\n2D-tuned flat threshold beta = 0.15 flown in 3D: deliveries={ft2d['deliveries']:.1f}, "
          f"stranded={ft2d['stranded']:.2f} per run ({ft2d['stranded_seeds']} of {N_SEEDS} seeds)")
    print(f"Vertical energy off (pred3d, gamma = 0 in physics and planning): deliveries="
          f"{nog['deliveries']:.1f} +/- {nog['deliveries_std']:.1f}, stranded={nog['stranded']:.2f}")

    print('\nPad dispatch comparison (same return rule, same seeds):')
    for name, base, near, alt in ((f'3D thr beta={bs:.2f}', sweep['thr3d'][bs], q_near, q_alt),
                                  ('Predictive 3D', preds['pred3d'], p_near, p_alt)):
        for qn, m in (('fifo_rand', base), ('fifo_near', near), ('alt_prio', alt)):
            v = m['visits']; pb = m['pad_busy']
            print(f"  {name:<16} {qn:<9} deliveries={m['deliveries']:6.1f} +/- {m['deliveries_std']:4.1f}  "
                  f"wait={m['wait_time']:6.1f} drone-s  idle pad while waiting={m['idle_pad_wait']:5.1f} pad-s  "
                  f"visits per level (1.5/3.0/4.5 m)={v[0]:.2f}/{v[1]:.2f}/{v[2]:.2f}  "
                  f"pad busy={pb[0] * 100:.0f}/{pb[1] * 100:.0f}/{pb[2] * 100:.0f}%")

    print('\nPad capacity check (pad busy = charging; drone time = '
          f'{N_DRONES} drones x {T_SIM:.0f} s = {N_DRONES * T_SIM:.0f} drone-s):')
    for name, m in ((f'3D thr beta={bs:.2f} FIFO', sweep['thr3d'][bs]),
                    ('Predictive 3D FIFO', preds['pred3d'])):
        pb = m['pad_busy']; drone_s = N_DRONES * T_SIM
        print(f"  {name:<24} pads busy {pb.min() * 100:.0f}-{pb.max() * 100:.0f}% "
              f"(mean {pb.mean() * 100:.1f}%, idle {100 - pb.mean() * 100:.1f}%)  "
              f"drone time: flying {m['utilisation'] * 100:.1f}%, charging "
              f"{pb.sum() * T_SIM / drone_s * 100:.1f}%, queue wait {m['wait_time']:.1f} drone-s "
              f"({m['wait_time'] / drone_s * 100:.1f}%)")

    print('\nPaired differences in deliveries (mean, std, wins/ties/losses over seeds):')
    pairs = [('3D thr (best safe) - flat thr (best safe)', sweep['thr3d'][bs],
              sweep['flat_thr'][best_safe['flat_thr']]),
             ('3D thr 0.15 - flat thr 0.15', sweep['thr3d'][0.15], sweep['flat_thr'][0.15]),
             ('Predictive 3D - flat predictive', preds['pred3d'], preds['pred_flat']),
             ('Predictive 3D - 3D thr (best safe)', preds['pred3d'], sweep['thr3d'][bs]),
             ('alt_prio - fifo_rand (3D thr)', q_alt, sweep['thr3d'][bs]),
             ('fifo_near - fifo_rand (3D thr)', q_near, sweep['thr3d'][bs]),
             ('alt_prio - fifo_near (3D thr)', q_alt, q_near),
             ('alt_prio - fifo_rand (pred 3D)', p_alt, preds['pred3d']),
             ('gamma off - gamma on (pred 3D)', nog, preds['pred3d'])]
    for name, a, b in pairs:
        m, s, w, t, l = paired(a, b)
        print(f'  {name:<44} {m:+6.1f} +/- {s:4.1f}   {w:2d}/{t:2d}/{l:2d}')

    print(f'\nRecorded runs (seed {SEED0}):')
    recs = []
    for lab, rule, beta, qr in ((f'3D thr beta={bs:.2f}, FIFO', 'thr3d', bs, 'fifo_rand'),
                                (f'3D thr beta={bs:.2f}, alt-priority', 'thr3d', bs, 'alt_prio')):
        r = run_simulation(rule, beta, qr, SEED0, record=True)
        r['label'] = lab; r['queue_rule'] = qr
        recs.append(r)
        nq = r['hist']['nq']
        print(f"  {lab:<32} deliveries={r['deliveries']}  stranded={r['stranded']}  "
              f"dockings per level={[int(v) for v in r['visits']]}  max waiting={int(nq.sum(axis=1).max())}  "
              f"z range=[{r['hist']['pos'][:, :, 2].min():.2f}, {r['hist']['pos'][:, :, 2].max():.2f}] m")
        if qr == 'alt_prio':                 # pad free while a drone waits for another level
            occ, nqa = r['hist']['occ'], r['hist']['nq']
            tt = (np.arange(nqa.shape[0]) + 1) * DT   # same time base as plot_level_queues
            for lf in range(N_LEVELS):
                for lw in range(N_LEVELS):
                    if lf == lw:
                        continue
                    mask = ~occ[:, lf] & (nqa[:, lw] > 0)
                    edges = np.flatnonzero(np.diff(np.r_[0, mask.astype(int), 0]))
                    for s0, s1 in zip(edges[::2], edges[1::2]):
                        print(f"    pad {PAD_Z[lf]:.1f} m free while a drone waits for {PAD_Z[lw]:.1f} m: "
                              f"t = {tt[s0]:.1f}-{tt[s1 - 1]:.1f} s")

    print()
    plot_trajectories_3d(recs[0], out_dir)
    plot_altitude_soc(recs[0], out_dir)
    plot_threshold_sweep(sweep, preds, best_safe, out_dir)
    plot_policy_comparison(rows, out_dir)
    plot_level_queues(recs[0], recs[1], out_dir)
    save_animation(recs[0], out_dir)
    print('Done.')


if __name__ == '__main__':
    main()
