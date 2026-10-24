"""
S089 Large-Scale Swarm Collision Avoidance Test
===============================================
Fifty drones start on a sphere of radius 20 m and fly to their antipodal points, so every
straight path passes through the centre at about the same time. Three strategies are
compared on the same start positions (common random numbers):

    1. No avoidance (straight to goal)
    2. Potential-field repulsion
    3. ORCA (RVO2-3D half-planes + incremental 3D linear program)

All strategies share the same goal-seeking preferred velocity, speed cap and acceleration
limit. A Monte Carlo over random sphere placements gives mean +- std and per-seed counts.

Usage:
    conda activate drones
    python src/05_special_entertainment/s089_collision_avoidance.py
"""

import os
import math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401

# ── Parameters ────────────────────────────────────────────────────────────────
N_DRONES         = 50
SPHERE_RADIUS    = 20.0       # m
DRONE_RADIUS     = 0.25       # m body radius
COLLISION_RADIUS = 2 * DRONE_RADIUS   # 0.5 m pair threshold
FORMATION_DIST   = 2.0        # m minimum separation between start points (and goals)
V_PREF           = 2.0        # m/s preferred cruise speed
V_MAX            = 3.0        # m/s speed cap
A_MAX            = 5.0        # m/s^2 acceleration limit (all strategies)
TAU              = 3.0        # s ORCA time horizon
DT               = 0.05       # s
T_MAX            = 60.0       # s mission limit
N_STEPS          = int(round(T_MAX / DT))
D_GOAL           = 0.5        # m arrival radius
T_ARRIVE         = 1.0        # s preferred speed tapers as d / T_ARRIVE inside 2 m of the goal
NB_RADIUS        = 10.0       # m neighbour radius (ORCA and potential field)
D_REP            = 5.0        # m potential-field activation radius
K_REP            = 5.0        # potential-field repulsion gain (m^3/s^2)
K_ATT            = 1.0        # 1/s potential-field attraction gain (velocity tracking)
SEED_MAIN        = 42
N_MC             = 20
MC_SEEDS         = [SEED_MAIN + k for k in range(N_MC)]

STRATEGIES = ['orca', 'potential_field', 'none']
LABEL = {'orca': 'ORCA', 'potential_field': 'Potential field', 'none': 'No avoidance'}
COLOR = {'orca': '#1F77B4', 'potential_field': '#FF9800', 'none': '#D62728'}

OUTPUT_DIR = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', '..',
    'outputs', '05_special_entertainment', 's089_collision_avoidance'))


# ── Initial positions ─────────────────────────────────────────────────────────

def init_sphere_positions(seed, n=N_DRONES, radius=SPHERE_RADIUS, d_min=FORMATION_DIST):
    """Uniform points on the sphere (Gaussian normalisation) with rejection of any point
    closer than d_min to an accepted point or to an accepted point's antipode."""
    rng = np.random.default_rng(seed)
    pts = []
    n_rej = 0
    while len(pts) < n:
        p = rng.standard_normal(3)
        p = radius * p / np.linalg.norm(p)
        if pts:
            P = np.array(pts)
            if (np.min(np.linalg.norm(P - p, axis=1)) < d_min or
                    np.min(np.linalg.norm(P + p, axis=1)) < d_min):
                n_rej += 1
                continue
        pts.append(p)
    pos = np.array(pts)
    return pos, -pos, n_rej


# ── 3D ORCA (RVO2-3D half-planes and incremental linear programs) ────────────
# Half-plane {v : n . (v - p) >= 0}. Small vectors are plain Python tuples for speed.

LP_EPS = 1e-9


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _mul(s, a):
    return (s * a[0], s * a[1], s * a[2])


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _norm(a):
    return math.sqrt(_dot(a, a))


def _lp1(planes, plane_no, lp, ld, radius, opt, dir_opt):
    dotp = _dot(lp, ld)
    disc = dotp * dotp + radius * radius - _dot(lp, lp)
    if disc < 0.0:
        return None
    sq = math.sqrt(disc)
    t_left, t_right = -dotp - sq, -dotp + sq
    for i in range(plane_no):
        pp, pn = planes[i]
        num = _dot(_sub(pp, lp), pn)
        den = _dot(ld, pn)
        if den * den <= LP_EPS:
            if num > 0.0:
                return None
            continue
        t = num / den
        if den >= 0.0:
            t_left = max(t_left, t)
        else:
            t_right = min(t_right, t)
        if t_left > t_right:
            return None
    if dir_opt:
        t = t_right if _dot(opt, ld) > 0.0 else t_left
    else:
        t = min(max(_dot(ld, _sub(opt, lp)), t_left), t_right)
    return _add(lp, _mul(t, ld))


def _lp2(planes, plane_no, radius, opt, dir_opt):
    pp, pn = planes[plane_no]
    pdist = _dot(pp, pn)
    if pdist * pdist > radius * radius:
        return None
    prad2 = radius * radius - pdist * pdist
    centre = _mul(pdist, pn)
    if dir_opt:
        po = _sub(opt, _mul(_dot(opt, pn), pn))
        l2 = _dot(po, po)
        result = centre if l2 <= LP_EPS else _add(centre, _mul(math.sqrt(prad2 / l2), po))
    else:
        result = _add(opt, _mul(_dot(_sub(pp, opt), pn), pn))
        if _dot(result, result) > radius * radius:
            pr = _sub(result, centre)
            result = _add(centre, _mul(math.sqrt(prad2 / _dot(pr, pr)), pr))
    for i in range(plane_no):
        qp, qn = planes[i]
        if _dot(qn, _sub(qp, result)) > 0.0:
            cr = _cross(qn, pn)
            cl2 = _dot(cr, cr)
            if cl2 <= LP_EPS:
                return None
            ld = _mul(1.0 / math.sqrt(cl2), cr)
            ln = _cross(ld, pn)
            lp = _add(pp, _mul(_dot(_sub(qp, pp), qn) / _dot(ln, qn), ln))
            result = _lp1(planes, i, lp, ld, radius, opt, dir_opt)
            if result is None:
                return None
    return result


def _lp3(planes, radius, opt, dir_opt):
    if dir_opt:
        result = _mul(radius, opt)
    elif _dot(opt, opt) > radius * radius:
        result = _mul(radius / _norm(opt), opt)
    else:
        result = opt
    for i in range(len(planes)):
        pp, pn = planes[i]
        if _dot(pn, _sub(pp, result)) > 0.0:
            new = _lp2(planes, i, radius, opt, dir_opt)
            if new is None:
                return i, result
            result = new
    return len(planes), result


def _lp4(planes, begin, radius, result):
    distance = 0.0
    for i in range(begin, len(planes)):
        pp, pn = planes[i]
        if _dot(pn, _sub(pp, result)) > distance:
            proj = []
            for j in range(i):
                qp, qn = planes[j]
                cr = _cross(qn, pn)
                if _dot(cr, cr) <= LP_EPS:
                    if _dot(pn, qn) > 0.0:
                        continue
                    point = _mul(0.5, _add(pp, qp))
                else:
                    ln = _cross(cr, pn)
                    point = _add(pp, _mul(_dot(_sub(qp, pp), qn) / _dot(ln, qn), ln))
                nd = _sub(qn, pn)
                proj.append((point, _mul(1.0 / _norm(nd), nd)))
            fail, new = _lp3(proj, radius, pn, True)
            if fail >= len(proj):
                result = new
            distance = _dot(pn, _sub(pp, result))
    return result


def orca_planes(pos, vel, radius, t_h, r_sense, dt):
    """Vectorised RVO2-3D ORCA half-planes for every ordered neighbour pair (i, j)."""
    rel_p = pos[None, :, :] - pos[:, None, :]          # [i, j] = p_j - p_i
    d2_all = np.einsum('ijk,ijk->ij', rel_p, rel_p)
    mask = d2_all <= r_sense ** 2
    np.fill_diagonal(mask, False)
    ii, jj = np.nonzero(mask)
    P = rel_p[ii, jj]
    V = vel[ii] - vel[jj]
    d2 = d2_all[ii, jj]
    R = 2.0 * radius
    R2 = R * R
    normal = np.zeros_like(P)
    u = np.zeros_like(P)
    sep = d2 > R2
    with np.errstate(divide='ignore', invalid='ignore'):
        w = V - P / t_h
        wl2 = np.einsum('ij,ij->i', w, w)
        dp = np.einsum('ij,ij->i', w, P)
        cut = sep & (dp < 0) & (dp * dp > R2 * wl2)
        wl = np.sqrt(wl2)
        c_idx = np.nonzero(cut)[0]
        normal[c_idx] = w[c_idx] / wl[c_idx, None]
        u[c_idx] = (R / t_h - wl[c_idx])[:, None] * normal[c_idx]
        leg = sep & ~cut
        l_idx = np.nonzero(leg)[0]
        if len(l_idx):
            Pl, Vl, dl = P[l_idx], V[l_idx], d2[l_idx]
            b = np.einsum('ij,ij->i', Pl, Vl)
            cr = np.cross(Pl, Vl)
            c = np.einsum('ij,ij->i', Vl, Vl) - np.einsum('ij,ij->i', cr, cr) / (dl - R2)
            t = (b + np.sqrt(np.maximum(b * b - dl * c, 0.0))) / dl
            w2 = Vl - t[:, None] * Pl
            wl_2 = np.linalg.norm(w2, axis=1)
            good = wl_2 > 1e-12
            nrm = np.where(good[:, None], w2 / np.where(good, wl_2, 1.0)[:, None],
                           -Pl / np.sqrt(dl)[:, None])
            normal[l_idx] = nrm
            u[l_idx] = (R * t - wl_2)[:, None] * nrm
        o_idx = np.nonzero(~sep)[0]
        if len(o_idx):
            w3 = V[o_idx] - P[o_idx] / dt
            wl3 = np.linalg.norm(w3, axis=1)
            good = wl3 > 1e-12
            nrm = np.where(good[:, None], w3 / np.where(good, wl3, 1.0)[:, None],
                           np.array([1.0, 0.0, 0.0]))
            normal[o_idx] = nrm
            u[o_idx] = (R / dt - wl3)[:, None] * nrm
    point = vel[ii] + 0.5 * u
    return ii, d2, point, normal


def orca_step(pos, vel, v_pref, radius=DRONE_RADIUS, t_h=TAU, r_sense=NB_RADIUS,
              v_max=V_MAX, dt=DT):
    """New ORCA velocities; returns (v_new, n_agents_constrained, n_agents_infeasible)."""
    sp = np.linalg.norm(v_pref, axis=1, keepdims=True)
    v0 = np.where(sp > v_max, v_pref * v_max / np.maximum(sp, 1e-12), v_pref)
    v_new = v0.copy()
    ii, d2, point, normal = orca_planes(pos, vel, radius, t_h, r_sense, dt)
    if len(ii) == 0:
        return v_new, 0, 0
    viol = np.einsum('ij,ij->i', normal, point - v0[ii]) > 0.0
    keep = np.einsum('ij,ij->i', normal, point) >= -v_max     # drop planes inactive on the speed ball
    n_con = n_inf = 0
    for i in np.unique(ii[viol]):
        sel = np.nonzero((ii == i) & keep)[0]
        sel = sel[np.argsort(d2[sel], kind='stable')]
        planes = [(tuple(point[k].tolist()), tuple(normal[k].tolist())) for k in sel]
        opt = tuple(v_pref[i].tolist())
        fail, res = _lp3(planes, v_max, opt, False)
        if fail < len(planes):
            res = _lp4(planes, fail, v_max, res)
            n_inf += 1
        v_new[i] = res
        n_con += 1
    return v_new, n_con, n_inf


# ── Potential field ───────────────────────────────────────────────────────────

def pf_acceleration(pos, vel, v_goal):
    """a = K_ATT (v_goal - v) + sum_j k_rep (1/d - 1/d_rep) / d^2 * unit(p_i - p_j)."""
    diff = pos[:, None, :] - pos[None, :, :]                # p_i - p_j
    d = np.linalg.norm(diff, axis=2)
    np.fill_diagonal(d, np.inf)
    act = (d < D_REP) & (d < NB_RADIUS)
    with np.errstate(divide='ignore', invalid='ignore'):
        mag = np.where(act, K_REP * (1.0 / d - 1.0 / D_REP) / d ** 2, 0.0)
        f_rep = np.sum(np.where(act[:, :, None], mag[:, :, None] * diff / d[:, :, None], 0.0),
                       axis=1)
    return K_ATT * (v_goal - vel) + f_rep


# ── Simulation ────────────────────────────────────────────────────────────────

def goal_velocity(pos, goals):
    to_goal = goals - pos
    dist = np.linalg.norm(to_goal, axis=1, keepdims=True)
    speed = np.minimum(V_PREF, dist / T_ARRIVE)
    return np.where(dist > 1e-9, to_goal / np.maximum(dist, 1e-9) * speed, 0.0)


def simulate(strategy, pos0, goals, keep_history=False):
    n = len(pos0)
    pos = pos0.copy()
    vel = np.zeros((n, 3))
    arrival = np.full(n, np.nan)
    first_contact = np.full(n, np.nan)
    hist = [pos.copy()] if keep_history else None
    min_sep_t, n_arr_t, pairs_t = [], [], []
    acc_sum = 0.0
    n_active = 0
    n_infeasible = 0
    n_acc_sat = 0
    iu = np.triu_indices(n, 1)
    k_end = N_STEPS
    for k in range(N_STEPS + 1):
        t = k * DT
        d = np.linalg.norm(pos[:, None] - pos[None], axis=2)
        dp = d[iu]
        coll = dp < COLLISION_RADIUS
        pairs_t.append(int(coll.sum()))
        min_sep_t.append(float(dp.min()))
        if coll.any():
            inv = np.unique(np.concatenate([iu[0][coll], iu[1][coll]]))
            first_contact[inv] = np.where(np.isnan(first_contact[inv]), t, first_contact[inv])
        dist_goal = np.linalg.norm(goals - pos, axis=1)
        newly = np.isnan(arrival) & (dist_goal <= D_GOAL)
        arrival[newly] = t
        n_arr_t.append(int(np.sum(~np.isnan(arrival))))
        if not np.isnan(arrival).any() or k == N_STEPS:
            k_end = k
            break
        v_goal = goal_velocity(pos, goals)
        if strategy == 'orca':
            v_cmd, nc, ni = orca_step(pos, vel, v_goal)
            n_active += nc
            n_infeasible += ni
        elif strategy == 'potential_field':
            v_cmd = vel + pf_acceleration(pos, vel, v_goal) * DT
        else:
            v_cmd = v_goal
        dv = v_cmd - vel
        dvn = np.linalg.norm(dv, axis=1, keepdims=True)
        lim = A_MAX * DT
        n_acc_sat += int(np.sum(dvn > lim + 1e-12))
        dv = np.where(dvn > lim, dv * lim / np.maximum(dvn, 1e-12), dv)
        vel = vel + dv
        sp = np.linalg.norm(vel, axis=1, keepdims=True)
        vel = np.where(sp > V_MAX, vel * V_MAX / np.maximum(sp, 1e-12), vel)
        assert np.all(np.linalg.norm(vel, axis=1) <= V_MAX + 1e-9), 'speed cap violated'
        assert np.all(np.linalg.norm(dv, axis=1) <= lim + 1e-9), 'acceleration limit violated'
        acc_sum += float(np.sum(np.linalg.norm(dv, axis=1)) / DT)
        pos = pos + vel * DT
        if keep_history:
            hist.append(pos.copy())
    arrived = ~np.isnan(arrival)
    clean = arrived & (np.isnan(first_contact) | (first_contact > arrival))
    res = {
        'pairs_t': np.array(pairs_t), 'min_sep_t': np.array(min_sep_t),
        'n_arr_t': np.array(n_arr_t), 'arrival': arrival, 'first_contact': first_contact,
        'collisions': int(np.sum(pairs_t)),
        'coll_steps': int(np.sum(np.array(pairs_t) > 0)),
        'drones_in_contact': int(np.sum(~np.isnan(first_contact))),
        'min_sep': float(np.min(min_sep_t)),
        'pct_arrived': 100.0 * arrived.mean(),
        'pct_clean': 100.0 * clean.mean(),
        'mission_time': float(np.nanmax(arrival)) if arrived.all() else T_MAX,
        'mean_t_goal': float(np.nanmean(arrival)) if arrived.any() else float('nan'),
        'mean_accel': acc_sum / (n * max(k_end, 1)),
        'orca_active_rate': n_active / (n * max(k_end, 1)),
        'orca_infeasible': n_infeasible,
        'acc_sat_rate': n_acc_sat / (n * max(k_end, 1)),
        'steps': k_end,
    }
    if keep_history:
        res['history'] = np.array(hist)
    return res


def run_simulation():
    main = {}
    mc = {s: [] for s in STRATEGIES}
    placement = {}
    for seed in MC_SEEDS:
        pos0, goals, n_rej = init_sphere_positions(seed)
        placement[seed] = (pos0, goals, n_rej)
        for s in STRATEGIES:
            r = simulate(s, pos0, goals, keep_history=(seed == SEED_MAIN))
            if seed == SEED_MAIN:
                main[s] = r
            mc[s].append({k: v for k, v in r.items()
                          if k not in ('history', 'pairs_t', 'min_sep_t', 'n_arr_t',
                                       'arrival', 'first_contact')})
    return main, mc, placement


# ── Plotting ──────────────────────────────────────────────────────────────────

def plot_trajectories_3d(main, goals, out_dir):
    fig = plt.figure(figsize=(18, 6.2))
    cmap = plt.cm.tab20
    for p_i, s in enumerate(['orca', 'potential_field', 'none']):
        ax = fig.add_subplot(1, 3, p_i + 1, projection='3d')
        h = main[s]['history']
        for i in range(N_DRONES):
            ax.plot(h[:, i, 0], h[:, i, 1], h[:, i, 2], color=cmap(i % 20), lw=0.6, alpha=0.6)
        ax.scatter(*h[0].T, color='0.4', s=10, depthshade=False, label='start')
        ax.scatter(*goals.T, color='green', marker='*', s=30, depthshade=False, label='goal')
        fc = main[s]['first_contact']
        idx = np.nonzero(~np.isnan(fc))[0]
        if len(idx):
            kk = np.round(fc[idx] / DT).astype(int)
            pts = h[np.minimum(kk, len(h) - 1), idx]
            ax.scatter(*pts.T, color='red', marker='x', s=30, depthshade=False,
                       label='first contact (< 0.5 m)')
        r = main[s]
        ax.set_title(f"{LABEL[s]}: {r['collisions']} collision pair-steps,\n"
                     f"min sep {r['min_sep']:.3f} m, {r['pct_arrived']:.0f}% arrived",
                     fontsize=10)
        ax.set_xlim(-22, 22); ax.set_ylim(-22, 22); ax.set_zlim(-22, 22)
        ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)'); ax.set_zlabel('z (m)')
        ax.legend(fontsize=7, loc='upper left')
    fig.suptitle(f'S089 Trajectories of {N_DRONES} drones to antipodal goals (seed {SEED_MAIN})',
                 y=1.06)
    plt.tight_layout()
    path = os.path.join(out_dir, 'trajectories_3d.png')
    plt.savefig(path, dpi=120, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_metrics_comparison(mc, out_dir):
    panels = [('collisions', 'Collision pair-steps', False),
              ('min_sep', 'Minimum separation (m)', True),
              ('pct_arrived', 'Drones arrived (%)', False),
              ('pct_clean', 'Arrived with no contact (%)', False),
              ('mission_time', 'Mission time (s)\n(60 s = not all arrived)', False),
              ('mean_accel', 'Mean |acceleration| (m/s²)', False)]
    fig, axes = plt.subplots(1, len(panels), figsize=(22, 4.8))
    for ax, (key, title, thr) in zip(axes, panels):
        for x, s in enumerate(STRATEGIES):
            vals = np.array([m[key] for m in mc[s]], dtype=float)
            ax.bar(x, vals.mean(), yerr=vals.std(), color=COLOR[s], edgecolor='k',
                   alpha=0.75, capsize=4)
            ax.scatter(x + np.linspace(-0.2, 0.2, len(vals)), vals, color='k', s=8, zorder=3)
        if thr:
            ax.axhline(COLLISION_RADIUS, color='red', ls='--', lw=1.2, label='0.5 m threshold')
            ax.set_yscale('log')
            ax.legend(fontsize=8)
        if key == 'collisions':
            ax.set_yscale('symlog', linthresh=1)
        ax.set_xticks(range(len(STRATEGIES)))
        ax.set_xticklabels([LABEL[s] for s in STRATEGIES], fontsize=9)
        ax.set_title(title, fontsize=10)
        ax.grid(alpha=0.3, axis='y')
    fig.suptitle(f'S089 Monte Carlo over {N_MC} random sphere placements '
                 '(bar = mean, error bar = std, dots = seeds)')
    plt.tight_layout()
    path = os.path.join(out_dir, 'metrics_comparison.png')
    plt.savefig(path, dpi=120, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_separation_histogram(main, out_dir):
    fig, axes = plt.subplots(1, 3, figsize=(16, 4.6))
    for ax, s in zip(axes, STRATEGIES):
        ms = main[s]['min_sep_t']
        idx = np.unique(np.linspace(0, len(ms) - 1, 100).round().astype(int))
        ax.hist(ms[idx], bins=np.linspace(0, max(4.0, ms[idx].max()), 41), color=COLOR[s],
                edgecolor='k', alpha=0.8)
        ax.axvline(COLLISION_RADIUS, color='red', ls='--', lw=2, label='0.5 m threshold')
        ax.set_title(f'{LABEL[s]}: swarm minimum separation\n'
                     f'({len(idx)} evenly spaced timesteps of {len(ms)})', fontsize=10)
        ax.set_xlabel('minimum pairwise separation (m)')
        ax.set_ylabel('timesteps')
        ax.legend(fontsize=8)
    fig.suptitle(f'S089 Distribution of the swarm minimum separation (seed {SEED_MAIN})')
    plt.tight_layout()
    path = os.path.join(out_dir, 'separation_histogram.png')
    plt.savefig(path, dpi=120, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_timeline(main, out_dir):
    fig, axes = plt.subplots(1, 3, figsize=(17, 4.6))
    for s in STRATEGIES:
        r = main[s]
        t = np.arange(len(r['min_sep_t'])) * DT
        axes[0].plot(t, r['min_sep_t'], color=COLOR[s], label=LABEL[s])
        axes[1].plot(t, r['pairs_t'], color=COLOR[s], label=LABEL[s])
        axes[2].plot(t, r['n_arr_t'], color=COLOR[s], label=LABEL[s])
    axes[0].axhline(COLLISION_RADIUS, color='red', ls='--', lw=1.2)
    axes[0].set_yscale('log')
    axes[0].set_title('Swarm minimum separation (log scale)')
    axes[0].set_ylabel('m')
    axes[1].set_title('Colliding pairs per timestep')
    axes[2].set_title('Drones arrived')
    axes[2].axhline(N_DRONES, color='k', ls=':', lw=1)
    for ax in axes:
        ax.set_xlabel('time (s)')
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
    fig.suptitle(f'S089 Time histories (seed {SEED_MAIN})')
    plt.tight_layout()
    path = os.path.join(out_dir, 'timeline.png')
    plt.savefig(path, dpi=120, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def save_animation(main, goals, out_dir, step=4):
    hs = {s: main[s]['history'] for s in ('orca', 'none')}
    n = max(len(h) for h in hs.values())
    n_frames = math.ceil((n - 1) / step) + 1
    fig, axes = plt.subplots(1, 2, figsize=(11, 5.6))
    scat, txt = {}, {}
    for ax, s in zip(axes, hs):
        ax.set_xlim(-23, 23); ax.set_ylim(-23, 23)
        ax.set_aspect('equal')
        ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')
        ax.scatter(goals[:, 0], goals[:, 1], color='green', marker='*', s=25, alpha=0.5)
        scat[s] = ax.scatter(hs[s][0, :, 0], hs[s][0, :, 1], c=np.arange(N_DRONES) % 20,
                             cmap='tab20', s=18, zorder=3, edgecolor='k', linewidths=0.3)
        ax.set_title(f'{LABEL[s]} (XY projection)')
        txt[s] = ax.text(0.02, 0.97, '', transform=ax.transAxes, va='top', fontsize=9)

    def update(f):
        k = f * step
        for s, h in hs.items():
            kk = min(k, len(h) - 1)
            scat[s].set_offsets(h[kk, :, :2])
            pairs = main[s]['pairs_t'][kk]
            cum = int(main[s]['pairs_t'][:kk + 1].sum())
            txt[s].set_text(f't = {kk * DT:4.1f} s\ncolliding pairs now: {pairs}\n'
                            f'collision pair-steps so far: {cum}\n'
                            f"arrived: {main[s]['n_arr_t'][kk]}/{N_DRONES}")
        return list(scat.values()) + list(txt.values())

    fig.suptitle('S089 Antipodal swap, green stars = goals')
    anim = animation.FuncAnimation(fig, update, frames=n_frames, interval=50, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    anim.save(path, writer='pillow', fps=20, dpi=72)
    plt.close(fig)
    print(f'Saved: {path} ({n_frames} frames)')


# ── Main ──────────────────────────────────────────────────────────────────────

def _ms(vals, fmt='{:.2f}'):
    v = np.asarray(vals, dtype=float)
    return f'{fmt.format(v.mean())} ± {fmt.format(v.std())}'


if __name__ == '__main__':
    main, mc, placement = run_simulation()
    pos0, goals, n_rej = placement[SEED_MAIN]
    d0 = np.linalg.norm(pos0[:, None] - pos0[None], axis=2)
    np.fill_diagonal(d0, np.inf)
    print('\n=== S089 Large-Scale Swarm Collision Avoidance — Results ===')
    print(f'  N = {N_DRONES}, sphere R = {SPHERE_RADIUS} m, dt = {DT} s, T_max = {T_MAX} s')
    print(f'  Seed {SEED_MAIN}: min start separation {d0.min():.3f} m '
          f'({n_rej} candidate points rejected for being < {FORMATION_DIST} m)')
    print(f'  Straight-line time to goal at v_pref: {2 * SPHERE_RADIUS / V_PREF:.1f} s; '
          f'all straight paths pass the centre at t ≈ {SPHERE_RADIUS / V_PREF:.1f} s')
    print(f'\n  Main seed {SEED_MAIN}:')
    for s in STRATEGIES:
        r = main[s]
        print(f'  [{LABEL[s]}]')
        print(f"    Collision pair-steps     : {r['collisions']} "
              f"(timesteps with any collision: {r['coll_steps']})")
        print(f"    Drones ever in contact   : {r['drones_in_contact']}/{N_DRONES}")
        print(f"    Min separation           : {r['min_sep']:.5f} m")
        print(f"    Arrived                  : {r['pct_arrived']:.1f}% "
              f"(without any prior contact: {r['pct_clean']:.1f}%)")
        print(f"    Mission time             : {r['mission_time']:.2f} s "
              f"(mean time-to-goal {r['mean_t_goal']:.2f} s)")
        print(f"    Mean |accel|             : {r['mean_accel']:.3f} m/s²")
        print(f"    Acceleration-limited rate: {100 * r['acc_sat_rate']:.1f}% of agent-steps")
        if s == 'orca':
            print(f"    ORCA active rate         : {100 * r['orca_active_rate']:.1f}% of "
                  f"agent-steps (infeasible LP agent-steps: {r['orca_infeasible']})")
    print(f'\n  Monte Carlo over {N_MC} placements (seeds {MC_SEEDS[0]}–{MC_SEEDS[-1]}), '
          'mean ± std:')
    for s in STRATEGIES:
        m = mc[s]
        col = [x['collisions'] for x in m]
        print(f'  [{LABEL[s]}]')
        print(f'    Collision pair-steps     : {_ms(col, "{:.1f}")}  '
              f'(seeds with 0: {sum(c == 0 for c in col)}/{N_MC}, max {max(col)})')
        print(f'    Drones ever in contact   : '
              f'{_ms([x["drones_in_contact"] for x in m], "{:.1f}")}')
        print(f'    Min separation           : {_ms([x["min_sep"] for x in m], "{:.5f}")} m '
              f'(worst {min(x["min_sep"] for x in m):.5f} m)')
        print(f'    Arrived                  : {_ms([x["pct_arrived"] for x in m], "{:.1f}")} %'
              f', without prior contact {_ms([x["pct_clean"] for x in m], "{:.1f}")} %')
        print(f'    Mission time             : {_ms([x["mission_time"] for x in m])} s'
              f' (seeds with all arrived: {sum(x["pct_arrived"] == 100 for x in m)}/{N_MC})')
        print(f'    Mean time-to-goal        : {_ms([x["mean_t_goal"] for x in m])} s')
        print(f'    Mean |accel|             : {_ms([x["mean_accel"] for x in m], "{:.3f}")} m/s²')
        if s == 'orca':
            print(f'    ORCA active rate         : '
                  f'{_ms([100 * x["orca_active_rate"] for x in m], "{:.1f}")} % of agent-steps'
                  f', infeasible LP agent-steps {_ms([x["orca_infeasible"] for x in m], "{:.1f}")}')
    o = [x['collisions'] for x in mc['orca']]
    p = [x['collisions'] for x in mc['potential_field']]
    z = [x['collisions'] for x in mc['none']]
    print('\n  Paired per-seed comparison (collision pair-steps):')
    print(f'    ORCA < potential field in {sum(a < b for a, b in zip(o, p))}/{N_MC} seeds, '
          f'equal in {sum(a == b for a, b in zip(o, p))}')
    print(f'    ORCA < no avoidance in {sum(a < b for a, b in zip(o, z))}/{N_MC} seeds')
    print(f'    Potential field < no avoidance in {sum(a < b for a, b in zip(p, z))}/{N_MC} seeds')
    mo = [x['mission_time'] for x in mc['orca']]
    mp = [x['mission_time'] for x in mc['potential_field']]
    print(f'    ORCA mission time < potential field in '
          f'{sum(a < b for a, b in zip(mo, mp))}/{N_MC} seeds')

    os.makedirs(OUTPUT_DIR, exist_ok=True)
    plot_trajectories_3d(main, goals, OUTPUT_DIR)
    plot_metrics_comparison(mc, OUTPUT_DIR)
    plot_separation_histogram(main, OUTPUT_DIR)
    plot_timeline(main, OUTPUT_DIR)
    save_animation(main, goals, OUTPUT_DIR)
