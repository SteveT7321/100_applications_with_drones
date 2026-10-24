"""
S088 Formation Geometric Shape Morphing
=======================================
Twenty drones morph through four 3D shapes (sphere -> cube -> helix -> star), 10 s per
transition. At each transition every drone is assigned a keypoint of the next shape and
tracks a linearly interpolated target. A 3D ORCA layer (RVO2-3D half-planes + incremental
LP) keeps the drones apart. Three conditions are compared on the same random numbers:

    1. ORCA + Hungarian assignment (proposed)
    2. Hungarian assignment, no ORCA
    3. Random assignment + ORCA

A 20-seed Monte Carlo (common random numbers) gives mean +- std and per-seed counts.

Usage:
    conda activate drones
    python src/05_special_entertainment/s088_formation_morphing.py
"""

import os
import math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401
from scipy.optimize import linear_sum_assignment

# ── Parameters ────────────────────────────────────────────────────────────────
N_DRONES       = 20
T_MORPH        = 10.0     # s   morph duration per transition
DT             = 0.05     # s   simulation step
N_STEPS_MORPH  = int(round(T_MORPH / DT))   # 200
V_MAX          = 3.0      # m/s speed limit
A_MAX          = 5.0      # m/s^2 acceleration limit (added, same for all conditions)
R_COLL         = 0.5      # m   physical collision radius (pair threshold 2 r = 1.0 m)
R_SENSE        = 4.0      # m   ORCA sensing radius
T_HORIZON      = 2.0      # s   ORCA time horizon
K_P            = 1.0      # 1/s position feedback on the interpolated target (added)
D_FORM         = 2.0      # m   nominal formation distance (reference line only)
R_SHAPE        = 4.0      # m   sphere radius
A_CUBE         = 3.0      # m   cube half-side
R_HELIX        = 3.0      # m   helix radius
PITCH_HELIX    = 0.6      # m   helix rise per drone
R_OUT_STAR     = 4.5      # m
R_IN_STAR      = 1.8      # m
Z_STAR         = (-1.2, 1.2)
INIT_NOISE     = 0.05     # m   std of initial position error around the sphere keypoints
Z_CENTRE       = 8.0      # m   altitude of the formation centre (all shapes above ground)
EPS_TARGET     = 0.3      # m   card target for end-of-morph formation error
N_MC           = 20       # Monte Carlo seeds
SEEDS          = list(range(N_MC))
MAIN_SEED      = SEEDS[0]
SHAPE_SEQUENCE = ['sphere', 'cube', 'helix', 'star']

CONDITIONS = ['orca_hung', 'no_orca', 'orca_rand', 'rand_no_orca']
COND_LABEL = {'orca_hung':    'ORCA + Hungarian',
              'no_orca':      'Hungarian, no ORCA',
              'orca_rand':    'ORCA + random assignment',
              'rand_no_orca': 'Random assignment, no ORCA (added control)'}
COND_KW = {'orca_hung':    dict(use_orca=True,  use_hungarian=True),
           'no_orca':      dict(use_orca=False, use_hungarian=True),
           'orca_rand':    dict(use_orca=True,  use_hungarian=False),
           'rand_no_orca': dict(use_orca=False, use_hungarian=False)}
COND_COLOR = {'orca_hung': 'tab:blue', 'no_orca': 'tab:red', 'orca_rand': 'tab:orange',
              'rand_no_orca': 'tab:purple'}
COND_STYLE = {'orca_hung': '-', 'no_orca': '--', 'orca_rand': '-', 'rand_no_orca': '-'}
COND_SHORT = {'orca_hung': 'ORCA +\nHungarian', 'no_orca': 'Hungarian,\nno ORCA',
              'orca_rand': 'ORCA +\nrandom', 'rand_no_orca': 'random,\nno ORCA'}

OUTPUT_DIR = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', '..',
    'outputs', '05_special_entertainment', 's088_formation_morphing'))


# ── Shapes ────────────────────────────────────────────────────────────────────

def make_sphere(n=N_DRONES, radius=R_SHAPE):
    """Fibonacci sphere (card formula)."""
    k = np.arange(n, dtype=float)
    golden = (1.0 + math.sqrt(5.0)) / 2.0
    theta = np.arccos(1.0 - 2.0 * k / (n - 1))
    phi = 2.0 * np.pi * k / golden
    return radius * np.column_stack([np.sin(theta) * np.cos(phi),
                                     np.sin(theta) * np.sin(phi),
                                     np.cos(theta)])


def make_cube(a=A_CUBE):
    """8 corners + 12 edge midpoints = 20 points on the cube surface (all six faces)."""
    pts = [[sx, sy, sz] for sx in (-a, a) for sy in (-a, a) for sz in (-a, a)]
    for axis in range(3):
        for s1 in (-a, a):
            for s2 in (-a, a):
                p = [0.0, 0.0, 0.0]
                others = [ax for ax in range(3) if ax != axis]
                p[others[0]], p[others[1]] = s1, s2
                pts.append(p)
    return np.array(pts, dtype=float)


def make_helix(n=N_DRONES, radius=R_HELIX, pitch=PITCH_HELIX):
    k = np.arange(n, dtype=float)
    return np.column_stack([radius * np.cos(2 * np.pi * k / n),
                            radius * np.sin(2 * np.pi * k / n),
                            pitch * k - pitch * (n - 1) / 2.0])


def make_star(n=N_DRONES, r_out=R_OUT_STAR, r_in=R_IN_STAR, z_layers=Z_STAR):
    """Five-pointed star outline (10 vertices, alternating outer/inner) on two layers."""
    per = n // 2
    pts = []
    for z in z_layers:
        for k in range(per):
            r = r_out if k % 2 == 0 else r_in
            ang = 2.0 * np.pi * k / per
            pts.append([r * np.cos(ang), r * np.sin(ang), z])
    return np.array(pts, dtype=float)


_CENTRE = np.array([0.0, 0.0, Z_CENTRE])
SHAPES = {'sphere': make_sphere() + _CENTRE, 'cube': make_cube() + _CENTRE,
          'helix': make_helix() + _CENTRE, 'star': make_star() + _CENTRE}


def nn_stats(pts):
    d = np.linalg.norm(pts[:, None] - pts[None], axis=2)
    np.fill_diagonal(d, np.inf)
    nn = d.min(axis=1)
    return nn.mean(), nn.min()


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
    n = len(pos)
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


def orca_step(pos, vel, v_pref, radius=R_COLL, t_h=T_HORIZON, r_sense=R_SENSE,
              v_max=V_MAX, dt=DT):
    """New ORCA velocities; returns (v_new, n_agents_constrained, n_agents_infeasible)."""
    n = len(pos)
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


# ── Simulation ────────────────────────────────────────────────────────────────

def simulate(seed, use_orca=True, use_hungarian=True):
    rng_init = np.random.default_rng([seed, 0])
    rng_assign = np.random.default_rng([seed, 1])
    pos = SHAPES['sphere'] + rng_init.normal(0.0, INIT_NOISE, (N_DRONES, 3))
    vel = np.zeros((N_DRONES, 3))
    n_tot = 3 * N_STEPS_MORPH
    log = {k: np.zeros((n_tot + 1, N_DRONES, 3)) for k in ('pos', 'vel', 'des')}
    log['pos'][0], log['des'][0] = pos, pos
    min_sep = np.zeros(n_tot + 1)
    mean_nn = np.zeros(n_tot + 1)
    n_pairs = np.zeros(n_tot + 1, dtype=int)
    eps_f = np.zeros(n_tot + 1)
    n_con = np.zeros(n_tot + 1, dtype=int)
    n_inf = np.zeros(n_tot + 1, dtype=int)
    travel_assigned = []
    eps_end = []
    acc_sat = 0

    def record(k, p, des):
        d = np.linalg.norm(p[:, None] - p[None], axis=2)
        np.fill_diagonal(d, np.inf)
        nn = d.min(axis=1)
        min_sep[k], mean_nn[k] = nn.min(), nn.mean()
        n_pairs[k] = int(np.sum(np.triu(d < 2.0 * R_COLL, 1)))
        eps_f[k] = np.mean(np.linalg.norm(p - des, axis=1))

    record(0, pos, pos)
    k = 0
    for s in range(3):
        tgt = SHAPES[SHAPE_SEQUENCE[s + 1]]
        if use_hungarian:
            C = np.linalg.norm(pos[:, None] - tgt[None], axis=2)
            row, col = linear_sum_assignment(C)
            p_b = tgt[col[np.argsort(row)]]
        else:
            p_b = tgt[rng_assign.permutation(N_DRONES)]
        p_a = pos.copy()
        travel_assigned.append(float(np.linalg.norm(p_b - p_a, axis=1).sum()))
        v_nom = (p_b - p_a) / T_MORPH
        for step in range(N_STEPS_MORPH):
            tau = step / N_STEPS_MORPH
            des_now = (1.0 - tau) * p_a + tau * p_b
            v_pref = v_nom + K_P * (des_now - pos)
            if use_orca:
                v_cmd, nc, ni = orca_step(pos, vel, v_pref)
            else:
                sp = np.linalg.norm(v_pref, axis=1, keepdims=True)
                v_cmd = np.where(sp > V_MAX, v_pref * V_MAX / np.maximum(sp, 1e-12), v_pref)
                nc = ni = 0
            dv = v_cmd - vel
            dvn = np.linalg.norm(dv, axis=1, keepdims=True)
            lim = A_MAX * DT
            acc_sat += int(np.sum(dvn > lim))
            dv = np.where(dvn > lim, dv * lim / np.maximum(dvn, 1e-12), dv)
            vel = vel + dv
            assert np.all(np.linalg.norm(vel, axis=1) <= V_MAX + 1e-9), 'speed limit violated'
            assert np.all(np.linalg.norm(dv, axis=1) <= lim + 1e-9), 'acceleration limit violated'
            pos = pos + vel * DT
            assert np.all(pos[:, 2] >= 0.0), 'drone below ground'
            k += 1
            tau1 = (step + 1) / N_STEPS_MORPH
            des1 = (1.0 - tau1) * p_a + tau1 * p_b
            log['pos'][k], log['vel'][k], log['des'][k] = pos, vel, des1
            n_con[k], n_inf[k] = nc, ni
            record(k, pos, des1)
        eps_end.append(eps_f[k])
    dvel = np.diff(log['vel'], axis=0)
    smooth = float(np.sqrt(np.mean(np.sum(dvel ** 2, axis=2))))
    speed = np.linalg.norm(log['vel'], axis=2)
    path_len = float(np.linalg.norm(np.diff(log['pos'], axis=0), axis=2).sum())
    return {
        'pos': log['pos'], 'vel': log['vel'], 'des': log['des'],
        'min_sep': min_sep, 'mean_nn': mean_nn, 'n_pairs': n_pairs, 'eps_f': eps_f,
        'n_con': n_con, 'n_inf': n_inf,
        'coll_steps': int(np.sum(n_pairs > 0)), 'coll_pair_steps': int(n_pairs.sum()),
        'min_sep_all': float(min_sep.min()), 'eps_end': np.array(eps_end),
        'smooth': smooth, 'max_speed': float(speed.max()),
        'min_z': float(log['pos'][:, :, 2].min()), 'max_z': float(log['pos'][:, :, 2].max()),
        'travel_assigned': np.array(travel_assigned), 'path_len': path_len,
        'con_steps': int(n_con.sum()), 'inf_steps': int(n_inf.sum()), 'acc_sat': acc_sat,
    }


def run_simulation():
    mc = {c: [] for c in CONDITIONS}
    main = {}
    for seed in SEEDS:
        for c in CONDITIONS:
            r = simulate(seed, **COND_KW[c])
            if seed == MAIN_SEED:
                main[c] = r
            mc[c].append({k: r[k] for k in ('coll_steps', 'coll_pair_steps', 'min_sep_all',
                                            'eps_end', 'smooth', 'max_speed', 'min_z',
                                            'travel_assigned', 'path_len', 'con_steps',
                                            'inf_steps', 'acc_sat')})
    return main, mc


# ── Plotting ──────────────────────────────────────────────────────────────────

def _shade(ax):
    for s, col in enumerate(['#e8f4f8', '#f8f4e8', '#f4f8e8']):
        ax.axvspan(s * T_MORPH, (s + 1) * T_MORPH, color=col, alpha=0.6, zorder=0)
    for s in (1, 2):
        ax.axvline(s * T_MORPH, color='gray', lw=0.8, ls='--')


def _outline_edges(name):
    """Index pairs drawn as grey guide lines (shape outline only, not trajectories)."""
    p = SHAPES[name]
    if name == 'helix':
        return [(k, k + 1) for k in range(N_DRONES - 1)]
    if name == 'star':
        per = N_DRONES // 2
        e = [(l * per + k, l * per + (k + 1) % per) for l in range(2) for k in range(per)]
        return e + [(k, k + per) for k in range(0, per, 2)]
    d = np.linalg.norm(p[:, None] - p[None], axis=2)
    lim = {'cube': A_CUBE + 1e-6, 'sphere': 3.0}[name]
    return [(a, b) for a in range(N_DRONES) for b in range(a + 1, N_DRONES) if d[a, b] <= lim]


def plot_shapes(out_dir):
    fig = plt.figure(figsize=(16, 4.4))
    for i, name in enumerate(SHAPE_SEQUENCE):
        ax = fig.add_subplot(1, 4, i + 1, projection='3d')
        p = SHAPES[name]
        mnn, minn = nn_stats(p)
        for a, b in _outline_edges(name):
            ax.plot(*zip(p[a], p[b]), color='0.5', lw=0.8)
        ax.scatter(p[:, 0], p[:, 1], p[:, 2], color='green', s=40, depthshade=False)
        ax.set_xlim(-5, 5); ax.set_ylim(-5, 5); ax.set_zlim(Z_CENTRE - 6, Z_CENTRE + 6)
        ax.set_box_aspect((10, 10, 12))
        ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)'); ax.set_zlabel('z (m)')
        ax.set_title(f'{name}: NN mean {mnn:.2f} m, min {minn:.2f} m', fontsize=10)
    fig.suptitle('S088 Target keypoint sets (N = 20 each, green = targets, grey = outline guide)')
    plt.tight_layout()
    path = os.path.join(out_dir, 'shapes.png')
    plt.savefig(path, dpi=120, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_3d_trajectories(main, out_dir):
    r = main['orca_hung']
    pos = r['pos']
    fig = plt.figure(figsize=(11, 8))
    ax = fig.add_subplot(111, projection='3d')
    cols = plt.cm.tab20(np.linspace(0, 1, N_DRONES))
    for i in range(N_DRONES):
        ax.plot(pos[:, i, 0], pos[:, i, 1], pos[:, i, 2], lw=0.7, alpha=0.5, color=cols[i])
    snaps = [0, N_STEPS_MORPH, 2 * N_STEPS_MORPH, 3 * N_STEPS_MORPH]
    for k, name, mk, c in zip(snaps, SHAPE_SEQUENCE, ['o', 's', '^', '*'],
                              ['tab:blue', 'tab:green', 'tab:orange', 'tab:purple']):
        ax.scatter(pos[k, :, 0], pos[k, :, 1], pos[k, :, 2], s=55 if mk != '*' else 110,
                   marker=mk, color=c, edgecolor='k', depthshade=False,
                   label=f'{name} at t = {k * DT:.0f} s')
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)'); ax.set_zlabel('z (m)')
    ax.set_title(f'S088 Trajectories, ORCA + Hungarian (seed {MAIN_SEED})')
    ax.legend(fontsize=8, loc='upper left')
    path = os.path.join(out_dir, '3d_trajectories.png')
    plt.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_metrics_panel(main, out_dir):
    t = np.arange(3 * N_STEPS_MORPH + 1) * DT
    fig, axes = plt.subplots(2, 2, figsize=(14, 9))
    for ax in axes.ravel():
        _shade(ax)
    for c in CONDITIONS:
        r = main[c]
        kw = dict(color=COND_COLOR[c], lw=2.2 if c == 'orca_hung' else 1.4,
                  ls=COND_STYLE[c], label=COND_LABEL[c])
        axes[0, 0].plot(t, r['eps_f'], **kw)
        axes[0, 1].plot(t, r['min_sep'], **kw)
        axes[1, 0].plot(t, np.cumsum(r['n_pairs'] > 0), **kw)
        axes[1, 1].plot(t, r['mean_nn'], **kw)
    axes[0, 0].axhline(EPS_TARGET, color='k', ls=':', lw=1.2, label=f'{EPS_TARGET} m target')
    axes[0, 0].set_title(r'Formation error $\varepsilon_f(t)$')
    axes[0, 0].set_ylabel('m')
    axes[0, 1].axhline(2 * R_COLL, color='red', ls=':', lw=1.5,
                       label=f'collision threshold {2 * R_COLL:.1f} m')
    axes[0, 1].set_title('Minimum inter-drone separation')
    axes[0, 1].set_ylabel('m')
    axes[1, 0].set_title('Cumulative timesteps with a collision (any pair < 1.0 m)')
    axes[1, 0].set_ylabel('steps')
    axes[1, 1].axhline(D_FORM, color='green', ls=':', lw=1.2, label=f'd_form = {D_FORM} m')
    axes[1, 1].set_title('Mean nearest-neighbour distance')
    axes[1, 1].set_ylabel('m')
    handles, labels = axes[0, 0].get_legend_handles_labels()
    for ax in axes.ravel():
        ax.set_xlabel('time (s)')
        h, l = ax.get_legend_handles_labels()
        ref = [(hh, ll) for hh, ll in zip(h, l) if ll not in COND_LABEL.values()]
        if ref:
            ax.legend(*zip(*ref), fontsize=8, loc='upper right')
        ax.grid(alpha=0.3)
        y0, y1 = ax.get_ylim()
        for s, lab in enumerate(['sphere→cube', 'cube→helix', 'helix→star']):
            ax.text((s + 0.5) * T_MORPH, y0 + 0.02 * (y1 - y0), lab, ha='center',
                    va='bottom', fontsize=8, style='italic', color='dimgray')
    cond = [(hh, ll) for hh, ll in zip(handles, labels) if ll in COND_LABEL.values()]
    fig.legend(*zip(*cond), loc='lower center', ncol=4, fontsize=9,
               bbox_to_anchor=(0.5, -0.03))
    fig.suptitle(f'S088 Formation morphing metrics (seed {MAIN_SEED}; '
                 'blue solid lies under red dashed where identical)', fontsize=13)
    plt.tight_layout()
    path = os.path.join(out_dir, 'metrics_panel.png')
    plt.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_monte_carlo(mc, out_dir):
    fig, axes = plt.subplots(1, 4, figsize=(20, 5.2))
    panels = [('coll_steps', 'Timesteps with a collision', None),
              ('min_sep_all', 'Minimum separation (m)', 2 * R_COLL),
              ('eps_end_max', 'Worst end-of-morph formation error (m)', EPS_TARGET),
              ('smooth', r'Smoothness $\mathcal{S}$ (m/s per step)', None)]
    for ax, (key, title, ref) in zip(axes, panels):
        data = []
        for c in CONDITIONS:
            if key == 'eps_end_max':
                data.append([m['eps_end'].max() for m in mc[c]])
            else:
                data.append([m[key] for m in mc[c]])
        for x, (c, d) in enumerate(zip(CONDITIONS, data)):
            jitter = np.linspace(-0.15, 0.15, len(d))
            ax.scatter(np.full(len(d), x) + jitter, d, color=COND_COLOR[c], s=18, alpha=0.8)
            ax.plot([x - 0.25, x + 0.25], [np.mean(d)] * 2, color='k', lw=2)
        if ref is not None:
            ax.axhline(ref, color='red', ls=':', lw=1.3)
        ax.set_xticks(range(len(CONDITIONS)))
        ax.set_xticklabels([COND_SHORT[c] for c in CONDITIONS], fontsize=9)
        ax.set_title(title, fontsize=10)
        ax.grid(alpha=0.3)
    fig.suptitle(f'S088 Monte Carlo over {N_MC} seeds (dots = seeds, bar = mean; '
                 'red dotted = threshold/target)')
    plt.tight_layout()
    path = os.path.join(out_dir, 'monte_carlo.png')
    plt.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def save_animation(main, out_dir, step=3):
    r = main['orca_hung']
    pos = r['pos']
    n = len(pos)
    n_frames = math.ceil((n - 1) / step) + 1
    cols = plt.cm.tab20(np.linspace(0, 1, N_DRONES))
    fig = plt.figure(figsize=(7, 6.5))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim(-5, 5); ax.set_ylim(-5, 5); ax.set_zlim(Z_CENTRE - 6, Z_CENTRE + 6)
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)'); ax.set_zlabel('z (m)')
    dots = ax.scatter(pos[0, :, 0], pos[0, :, 1], pos[0, :, 2], c=cols, s=40,
                      depthshade=False)
    trail_len = int(2.0 / DT)
    trails = [ax.plot([], [], [], '-', color=cols[i], lw=0.8, alpha=0.5)[0]
              for i in range(N_DRONES)]
    title = ax.set_title('')
    names = ['sphere→cube', 'cube→helix', 'helix→star']

    def update(f):
        k = min(f * step, n - 1)
        dots._offsets3d = (pos[k, :, 0], pos[k, :, 1], pos[k, :, 2])
        s0 = max(0, k - trail_len)
        for i in range(N_DRONES):
            trails[i].set_data(pos[s0:k + 1, i, 0], pos[s0:k + 1, i, 1])
            trails[i].set_3d_properties(pos[s0:k + 1, i, 2])
        seg = min(max(k - 1, 0) // N_STEPS_MORPH, 2)
        title.set_text(f'S088 ORCA + Hungarian  t = {k * DT:4.1f} s  {names[seg]}\n'
                       f'min separation {r["min_sep"][k]:.2f} m')
        return [dots, title] + trails

    anim = animation.FuncAnimation(fig, update, frames=n_frames, interval=50, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    anim.save(path, writer='pillow', fps=20, dpi=72)
    plt.close(fig)
    print(f'Saved: {path} ({n_frames} frames)')


# ── Main ──────────────────────────────────────────────────────────────────────

def _ms(vals, fmt='{:.3f}'):
    vals = np.asarray(vals, dtype=float)
    return f'{fmt.format(vals.mean())} ± {fmt.format(vals.std())}'


if __name__ == '__main__':
    main, mc = run_simulation()
    print('\n=== S088 Formation Morphing — Results ===')
    for name in SHAPE_SEQUENCE:
        mnn, minn = nn_stats(SHAPES[name])
        print(f'  Shape {name:6s}: mean NN spacing {mnn:.3f} m, min NN spacing {minn:.3f} m')
    print(f'\n  Main seed {MAIN_SEED}:')
    for c in CONDITIONS:
        r = main[c]
        print(f'  [{COND_LABEL[c]}]')
        print(f"    Timesteps with a collision : {r['coll_steps']} / {3 * N_STEPS_MORPH}"
              f"  (pair-steps {r['coll_pair_steps']})")
        print(f"    Min separation ever        : {r['min_sep_all']:.4f} m")
        print('    End-of-morph formation err : ' +
              ', '.join(f'{e:.3f}' for e in r['eps_end']) + ' m')
        print(f"    Max eps_f during mission   : {r['eps_f'].max():.3f} m")
        print(f"    Smoothness S               : {r['smooth']:.4f} m/s per step")
        print(f"    Max speed                  : {r['max_speed']:.3f} m/s")
        print(f"    Altitude range             : {r['min_z']:.2f}–{r['max_z']:.2f} m")
        print('    Assigned straight-line travel per transition: ' +
              ', '.join(f'{d:.1f}' for d in r['travel_assigned']) + ' m')
        print(f"    Flown path length (sum)    : {r['path_len']:.1f} m")
        print(f"    Agent-steps with active ORCA constraint: {r['con_steps']} "
              f"(infeasible LP: {r['inf_steps']})")
        print(f"    Agent-steps at the acceleration limit: {r['acc_sat']}")
    print(f'\n  Monte Carlo, {N_MC} seeds (mean ± std):')
    for c in CONDITIONS:
        m = mc[c]
        cs = [x['coll_steps'] for x in m]
        ms = [x['min_sep_all'] for x in m]
        ee = [x['eps_end'].max() for x in m]
        print(f'  [{COND_LABEL[c]}]')
        print(f'    Collision timesteps        : {_ms(cs, "{:.1f}")}, '
              f'seeds with >= 1 collision: {sum(v > 0 for v in cs)}/{N_MC}, max {max(cs)}')
        print(f'    Collision pair-steps       : '
              f'{_ms([x["coll_pair_steps"] for x in m], "{:.1f}")}')
        print(f'    Min separation             : {_ms(ms, "{:.4f}")} m (worst {min(ms):.4f} m)')
        print(f'    Worst end-of-morph eps_f   : {_ms(ee)} m (max {max(ee):.3f} m), '
              f'seeds meeting < {EPS_TARGET} m: {sum(v < EPS_TARGET for v in ee)}/{N_MC}')
        print(f'    Smoothness S               : '
              f'{_ms([x["smooth"] for x in m], "{:.4f}")} m/s per step')
        print(f'    Assigned travel (3 morphs) : '
              f'{_ms([x["travel_assigned"].sum() for x in m], "{:.1f}")} m')
        print(f'    Flown path length          : {_ms([x["path_len"] for x in m], "{:.1f}")} m')
        print(f'    ORCA-active agent-steps    : {_ms([x["con_steps"] for x in m], "{:.1f}")}'
              f', infeasible LP agent-steps: {_ms([x["inf_steps"] for x in m], "{:.1f}")}')
        print(f'    Max speed over all seeds   : {max(x["max_speed"] for x in m):.3f} m/s, '
              f'lowest altitude {min(x["min_z"] for x in m):.2f} m')
        print(f'    Acceleration-limited agent-steps: '
              f'{_ms([x["acc_sat"] for x in m], "{:.1f}")}')
    # Paired comparisons (common random numbers)
    h = [x['coll_steps'] for x in mc['orca_hung']]
    rnd = [x['coll_steps'] for x in mc['orca_rand']]
    no = [x['coll_steps'] for x in mc['no_orca']]
    print('\n  Paired per-seed comparison of collision timesteps:')
    print(f'    ORCA+Hungarian < no ORCA in {sum(a < b for a, b in zip(h, no))}/{N_MC} seeds, '
          f'equal in {sum(a == b for a, b in zip(h, no))}')
    print(f'    ORCA+Hungarian < ORCA+random in {sum(a < b for a, b in zip(h, rnd))}/{N_MC} '
          f'seeds, equal in {sum(a == b for a, b in zip(h, rnd))}')
    rn = [x['coll_steps'] for x in mc['rand_no_orca']]
    print(f'    ORCA+random < random without ORCA in '
          f'{sum(a < b for a, b in zip(rnd, rn))}/{N_MC} seeds, '
          f'equal in {sum(a == b for a, b in zip(rnd, rn))}')
    eh = [x['eps_end'].max() for x in mc['orca_hung']]
    er = [x['eps_end'].max() for x in mc['orca_rand']]
    print(f'    Worst end-of-morph error: Hungarian lower than random in '
          f'{sum(a < b for a, b in zip(eh, er))}/{N_MC} seeds')

    os.makedirs(OUTPUT_DIR, exist_ok=True)
    plot_shapes(OUTPUT_DIR)
    plot_3d_trajectories(main, OUTPUT_DIR)
    plot_metrics_panel(main, OUTPUT_DIR)
    plot_monte_carlo(mc, OUTPUT_DIR)
    save_animation(main, OUTPUT_DIR)
