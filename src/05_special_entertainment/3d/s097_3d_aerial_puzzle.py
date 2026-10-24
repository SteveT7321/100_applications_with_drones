"""
S097 3D Upgrade — Aerial Puzzle Assembly
========================================
The S097 assembly (six 0.5 kg drones, triangular pieces of side sqrt(3) m tiling a hexagon of
slot radius 1 m at z = 10 m; error consensus over a ring graph with the card gains; impedance
fitting with neighbour coupling; OU wind, RTK-like position bias, 1 mm relative sensing; edge
contact springs) is given the 3D card's two additions:

  * piece attitude: each piece hangs on a 2-axis gimbal under its drone, so roll, pitch and yaw
    of the piece are controlled (kinematic alignment K_omega = 2, then the card's rotational
    impedance K_r = 3, B_r = 1.5, I_eff = 0.2); targets yaw 2 pi k / 6 and the card's tilt
    theta_k* = 5 deg sin(2 pi k / 6);
  * layered approach: drone k climbs to z_asm + dz_k and runs the consensus phase there, then
    inserts vertically into the slot plane during the impedance fit.

Mission sequences
  card        consensus with the piece attitude frozen at its random start heading (the card's
              "random heading offsets"); at the 50 mm switch each piece rotates in place with
              translation frozen until its attitude error is <= 2 deg; then 6-DOF impedance fit
  concurrent  the attitude is aligned from t = 0 while the drones climb and converge (S097 base
              yaw behaviour); impedance fit directly at the switch

Variants (same wind, bias, noise and start jitter per seed; CRN)
  flat_conc   dz = 0, concurrent      (2D baseline: S097 at a fixed altitude)
  flat_card   dz = 0, card sequence
  cos_card    card stagger dz_k = 0.25 cos(2 pi k / 6), card sequence
  alt_card    alternating stagger dz_k = 0.25 (-1)^k, card sequence
  alt_conc    alternating stagger, concurrent

Collision diagnostic (added): each piece is a thin panel (thickness 20 mm). Two pieces interfere
when their vertical extents overlap and their footprints overlap (separating-axis penetration
depth). Overlap between two pieces that are both seated (within 50 mm of the slot and within
2 deg of the slot attitude) is the edge contact that S097 models with springs; any other overlap
is reported as a collision. Collisions are a diagnostic only: no impact forces are modelled.

Usage:
    conda activate drones
    python src/05_special_entertainment/3d/s097_3d_aerial_puzzle.py
"""

import os
import math
import importlib.util
from multiprocessing import Pool
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

_HERE = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location(
    's097_2d', os.path.join(_HERE, '..', 's097_aerial_puzzle.py'))
m2d = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(m2d)

OUTPUT_DIR = os.path.normpath(os.path.join(
    _HERE, '..', '..', '..', 'outputs', '05_special_entertainment', '3d', 's097_3d_aerial_puzzle'))

# ── Parameters (card / S097) ──────────────────────────────────────────────────
N = m2d.N_DRONES
M_DRONE, DT = m2d.M_DRONE, m2d.DT
R_HEX, Z_ASM, R_STAGE, Z_STAGE = m2d.R_HEX, m2d.Z_ASM, m2d.R_STAGE, m2d.Z_STAGE
EPSILON, ALPHA_XY = m2d.EPSILON, m2d.ALPHA          # 0.05, 0.10
ALPHA_Z = 0.15                                      # card: faster vertical settling
K_IMP, B_IMP, K_REL = m2d.K_IMP, m2d.B_IMP, m2d.K_REL
K_C, B_C = m2d.K_C, m2d.B_C
DELTA_SWITCH, DELTA_FIT, D_MIN = m2d.DELTA_SWITCH, m2d.DELTA_FIT, m2d.D_MIN
K_REP, JITTER = m2d.K_REP, m2d.JITTER
KP_TRACK, KD_TRACK, KP_ASC, KD_ASC = m2d.KP_TRACK, m2d.KD_TRACK, m2d.KP_ASC, m2d.KD_ASC
A_MAX, SIDE = m2d.A_MAX, m2d.SIDE
T_HOLD, T_LIMIT = m2d.T_HOLD, m2d.T_LIMIT
WIND_COMMON, WIND_INDIV, TAU_WIND = m2d.WIND_COMMON, m2d.WIND_INDIV, m2d.TAU_WIND
SIGMA_BIAS, TAU_BIAS, SIGMA_REL = m2d.SIGMA_BIAS, m2d.TAU_BIAS, m2d.SIGMA_REL
ANG, EDGE_N = m2d.ANG, m2d.EDGE_N
DELTA_Z = 0.25                     # m  card approach stagger
THETA_TILT = math.radians(5.0)     # card inward tilt
K_OMEGA = 2.0                      # rad/s per rad, card alignment law
DELTA_ATT = math.radians(2.0)      # card alignment switch
DELTA_ATT_FINAL = math.radians(1.0)
K_R, B_R, I_EFF = 3.0, 1.5, 0.2    # card rotational impedance

# ── Parameters (added) ────────────────────────────────────────────────────────
T_PANEL = 0.02                     # m  panel thickness for the collision check
N_MC = 20
N_SWEEP = 10
N_WORKERS = 4
DZ_SWEEP = [0.0, 0.05, 0.10, 0.15, 0.25, 0.50]
HEAD_SWEEP = [0.0, 0.25, 0.5, 1.0, 2.0]   # x the S097 start jitter (+-0.3 rad)
SIDE_SWEEP = [0.7, 1.2, 1.5, 1.55, 1.6, 1.65, 1.7, SIDE]   # card piece (0.7 m) ... S097 tiling
ATT_SWITCH_SWEEP = [2.0, 0.5, 0.2]         # deg, in-place alignment switch probe
ZASM_SWEEP = [8.0, 10.0, 12.0, 15.0]
VARIANTS = {'flat_conc': (0.0, 'alt', 'conc'), 'flat_card': (0.0, 'alt', 'card'),
            'cos_card': (DELTA_Z, 'cos', 'card'), 'alt_card': (DELTA_Z, 'alt', 'card'),
            'alt_conc': (DELTA_Z, 'alt', 'conc')}
LABELS = {'flat_conc': 'flat, concurrent alignment (2D baseline)',
          'flat_card': 'flat, card sequence', 'cos_card': 'card cos stagger, card sequence',
          'alt_card': 'alternating stagger, card sequence',
          'alt_conc': 'alternating stagger, concurrent'}
VCOL = {'flat_conc': 'k', 'flat_card': '#1f77b4', 'cos_card': '#ff7f0e', 'alt_card': '#d62728',
        'alt_conc': '#2ca02c'}
DRONE_COLORS = m2d.DRONE_COLORS
PAIRS = np.array([(i, j) for i in range(N) for j in range(i + 1, N)])
ADJ = np.array([(j - i) % N in (1, N - 1) for i, j in PAIRS])
LOCAL_V = SIDE / math.sqrt(3) * np.column_stack(
    [np.cos(np.pi + 2 * np.pi * np.arange(3) / 3), np.sin(np.pi + 2 * np.pi * np.arange(3) / 3),
     np.zeros(3)])                 # body frame: x outward, apex towards the centre


def stagger(dz, pattern):
    if pattern == 'cos':
        return dz * np.cos(ANG)
    return dz * (-1.0) ** np.arange(N)


def rot_z(a):
    c, s = np.cos(a), np.sin(a)
    R = np.zeros((len(a), 3, 3))
    R[:, 0, 0], R[:, 0, 1], R[:, 1, 0], R[:, 1, 1], R[:, 2, 2] = c, -s, s, c, 1.0
    return R


def rot_y(a):
    c, s = np.cos(a), np.sin(a)
    R = np.zeros((len(a), 3, 3))
    R[:, 0, 0], R[:, 0, 2], R[:, 2, 0], R[:, 2, 2], R[:, 1, 1] = c, s, -s, c, 1.0
    return R


R_STAR = rot_z(ANG) @ rot_y(THETA_TILT * np.sin(ANG))          # card R_z(psi) R_y(theta)


def rotvec(R):
    """Axis-angle vectors of a stack of rotation matrices (angles < pi)."""
    c = np.clip((np.trace(R, axis1=1, axis2=2) - 1.0) / 2.0, -1.0, 1.0)
    th = np.arccos(c)
    w = np.stack([R[:, 2, 1] - R[:, 1, 2], R[:, 0, 2] - R[:, 2, 0], R[:, 1, 0] - R[:, 0, 1]], 1)
    s = np.sin(th)
    f = np.where(s > 1e-9, th / (2 * np.maximum(s, 1e-12)), 0.5)
    return w * f[:, None]


def expmap(v):
    """Rotation matrices exp([v]x) for a stack of rotation vectors (Rodrigues)."""
    th = np.linalg.norm(v, axis=1)
    k = v / np.maximum(th, 1e-12)[:, None]
    K = np.zeros((len(v), 3, 3))
    K[:, 0, 1], K[:, 0, 2], K[:, 1, 0] = -k[:, 2], k[:, 1], k[:, 2]
    K[:, 1, 2], K[:, 2, 0], K[:, 2, 1] = -k[:, 0], -k[:, 1], k[:, 0]
    s, c = np.sin(th)[:, None, None], np.cos(th)[:, None, None]
    return np.eye(3)[None] + s * K + (1 - c) * (K @ K)


def local_vertices(side):
    return side / math.sqrt(3) * np.column_stack(
        [np.cos(np.pi + 2 * np.pi * np.arange(3) / 3), np.sin(np.pi + 2 * np.pi * np.arange(3) / 3),
         np.zeros(3)])


def piece_vertices(pos, R, lv=LOCAL_V):
    return pos[:, None, :] + np.einsum('kij,vj->kvi', R, lv)            # (N, 3, 3)


def interference(V):
    """Penetration depth of every piece pair (footprint SAT x vertical-extent overlap)."""
    A, B = V[PAIRS[:, 0], :, :2], V[PAIRS[:, 1], :, :2]                 # (P, 3, 2)
    edges = np.concatenate([np.roll(A, -1, 1) - A, np.roll(B, -1, 1) - B], 1)   # (P, 6, 2)
    ax = np.stack([-edges[..., 1], edges[..., 0]], -1)
    ax /= np.linalg.norm(ax, axis=-1, keepdims=True)
    pa = np.einsum('pak,pvk->pav', ax, A)
    pb = np.einsum('pak,pvk->pav', ax, B)
    ov = np.minimum(pa.max(-1), pb.max(-1)) - np.maximum(pa.min(-1), pb.min(-1))
    pen = np.maximum(ov.min(1), 0.0)
    za, zb = V[PAIRS[:, 0], :, 2], V[PAIRS[:, 1], :, 2]
    zov = (np.minimum(za.max(1), zb.max(1)) - np.maximum(za.min(1), zb.min(1))) + T_PANEL
    return np.where(zov > 0, pen, 0.0)


def rel_err(pos, targets, rel_noise):
    """(L e)_k from neighbour-relative measurements (S097) for a given target formation."""
    out = np.zeros((N, 3))
    for d in (-1, 1):
        rel = pos - np.roll(pos, d, axis=0) + rel_noise * (1 if d == 1 else -1)
        out += rel - (targets - np.roll(targets, d, axis=0))
    return out


# ── Simulation ────────────────────────────────────────────────────────────────

def run_simulation(variant='alt_card', seed=0, dz=None, pattern=None, seq=None, head_scale=1.0,
                   z_asm=Z_ASM, alpha_z=ALPHA_Z, side=SIDE, att_switch=DELTA_ATT, record=False):
    dz0, pat0, seq0 = VARIANTS[variant]
    dz = dz0 if dz is None else dz
    pattern = pat0 if pattern is None else pattern
    seq = seq0 if seq is None else seq
    targets = np.column_stack([R_HEX * np.cos(ANG), R_HEX * np.sin(ANG), np.full(N, z_asm)])
    p_app = targets + np.column_stack([np.zeros((N, 2)), stagger(dz, pattern)])
    alpha = np.array([ALPHA_XY, ALPHA_XY, alpha_z])
    rng = np.random.default_rng(seed)                    # S097 draw order (CRN with S097)
    ang0 = ANG + rng.uniform(-JITTER, JITTER, N)
    n_steps = int(round((T_LIMIT + T_HOLD + 30.0) / DT))
    n2d = int(round((T_LIMIT + T_HOLD) / DT))
    w_c_draw = rng.standard_normal((n2d, 3))
    w_i_draw = rng.standard_normal((n2d, N, 3))
    b_draw = rng.standard_normal((n2d, N, 3))
    r_draw = rng.standard_normal((n2d, N, 3))
    bias = SIGMA_BIAS * rng.standard_normal((N, 3))
    rng2 = np.random.default_rng([seed, 97])             # extra steps beyond the S097 horizon
    extra = rng2.standard_normal((n_steps - n2d, 1 + 3 * N, 3))
    pos = np.column_stack([R_STAGE * np.cos(ang0), R_STAGE * np.sin(ang0), np.full(N, Z_STAGE)])
    vel = np.zeros((N, 3))
    Rm = rot_z(ANG + head_scale * (ang0 - ANG))          # random heading offsets (S097 jitter)
    om = np.zeros((N, 3))
    w_c, w_i = np.zeros(3), np.zeros((N, 3))
    phase, t_asc, t_conv = 0, None, None
    stage = np.zeros(N, int)                              # after the switch: 0 align, 1 fit
    t_fit_all, t_seated, t_end = None, None, None
    col_max, col_steps, col_adj_max, seat_pen_max = 0.0, 0, 0.0, 0.0
    col_phase, col_is_adj = np.nan, np.nan
    any_max = 0.0                                          # overlap of any kind (no exclusion)
    lv = local_vertices(side)
    clear = (SIDE - side) / math.sqrt(3)                   # edge clearance at the slots (0 for S097)
    hold = dict(err=[], gap=[], att=[], fc=[], e6=[])
    min_sep = np.inf
    log = dict(t=[], pos=[], R=[], att=[], err=[], col=[], phase=[]) if record else None
    for k in range(n_steps):
        t = k * DT
        if k < n2d:
            wc_d, wi_d, b_d, r_d = w_c_draw[k], w_i_draw[k], b_draw[k], r_draw[k]
        else:
            x = extra[k - n2d]
            wc_d, wi_d, b_d, r_d = x[0], x[1:1 + N], x[1 + N:1 + 2 * N], x[1 + 2 * N:]
        e_true = pos - targets
        meas = pos + bias
        rel_noise = SIGMA_REL * r_d
        if phase == 0 and np.all(pos[:, 2] >= p_app[:, 2] - 0.05):
            phase, t_asc = 1, t
        if phase == 1 and np.linalg.norm(meas - p_app, axis=1).max() <= DELTA_SWITCH:
            phase, t_conv = 2, t
            stage[:] = 0 if seq == 'card' else 1
        rv = rotvec(R_STAR @ np.transpose(Rm, (0, 2, 1)))   # world-frame attitude error
        att = np.linalg.norm(rv, axis=1)
        if phase == 0:
            p_des = np.column_stack([R_STAGE * np.cos(ang0), R_STAGE * np.sin(ang0), p_app[:, 2]])
            acc = KP_ASC * (p_des - meas) - KD_ASC * vel
        elif phase == 1:
            upd = -EPSILON * rel_err(pos, p_app, rel_noise) - alpha[None, :] * (meas - p_app)
            acc = KP_TRACK * upd - KD_TRACK * vel
        else:
            stage = np.where((stage == 0) & (att <= att_switch), 1, stage)
            tgt = np.where(stage[:, None] == 1, targets, p_app)
            F = K_IMP * (tgt - meas) - B_IMP * vel - K_REL * rel_err(pos, tgt, rel_noise)
            acc = F / M_DRONE
        # attitude: kinematic alignment (card) before the fit, rotational impedance in the fit
        align = np.zeros(N, bool) if phase < 2 else stage == 0
        if seq == 'conc' and phase < 2:
            align[:] = True
        fit = (phase == 2) & (stage == 1)
        om = np.where(align[:, None], K_OMEGA * rv, om)
        om = np.where(fit[:, None], om + DT * (K_R * rv - B_R * om) / I_EFF, om)
        om = np.where((~align & ~fit)[:, None], 0.0, om)
        Rm = expmap(om * DT) @ Rm
        # soft repulsion (S097 / card), acceleration limit
        diff = pos[:, None, :] - pos[None, :, :]
        dist = np.linalg.norm(diff, axis=2) + np.eye(N) * 1e9
        close = dist < 2 * D_MIN
        rep = np.where(close[:, :, None], K_REP * (2 * D_MIN - dist)[:, :, None]
                       / dist[:, :, None] ** 2 * diff, 0.0).sum(axis=1)
        acc = acc + rep
        nrm = np.linalg.norm(acc, axis=1, keepdims=True)
        acc = np.where(nrm > A_MAX, acc * A_MAX / np.maximum(nrm, 1e-12), acc)
        # S097 edge contact (seated pieces)
        gap = np.einsum('ij,ij->i', EDGE_N, np.roll(e_true, -1, 0) - e_true)
        gap_rate = np.einsum('ij,ij->i', EDGE_N, np.roll(vel, -1, 0) - vel)
        err_n = np.linalg.norm(e_true, axis=1)
        seated_pos = err_n <= DELTA_SWITCH
        seated = seated_pos & np.roll(seated_pos, -1)
        pen_e = np.where(seated, np.maximum(-(gap + clear), 0.0), 0.0)
        fmag = np.where(pen_e > 0, np.maximum(K_C * pen_e - B_C * gap_rate, 0.0), 0.0)
        fc = -fmag[:, None] * EDGE_N + np.roll(fmag[:, None] * EDGE_N, 1, axis=0)
        # collision diagnostic
        V = piece_vertices(pos, Rm, lv)
        pen = interference(V)
        any_max = max(any_max, float(pen.max()))
        ok = seated_pos & (att <= DELTA_ATT)
        both = ok[PAIRS[:, 0]] & ok[PAIRS[:, 1]]
        col = np.where(both, 0.0, pen)
        seat_pen_max = max(seat_pen_max, float(np.where(both, pen, 0.0).max()))
        if col.max() > 0:
            col_steps += 1
            if col.max() > col_max:                   # mission phase of the deepest collision
                ip = int(np.argmax(col))
                col_phase = phase if phase < 2 else (
                    2 if min(stage[PAIRS[ip, 0]], stage[PAIRS[ip, 1]]) == 0 else 3)
                col_is_adj = float(ADJ[ip])
            col_max = max(col_max, float(col.max()))
            col_adj_max = max(col_adj_max, float(col[ADJ].max()))
        # disturbances and integration
        w_c = m2d.ou_step(w_c, WIND_COMMON, TAU_WIND, wc_d)
        w_i = m2d.ou_step(w_i, WIND_INDIV, TAU_WIND, wi_d)
        bias = m2d.ou_step(bias, SIGMA_BIAS, TAU_BIAS, b_d)
        a_tot = acc + fc / M_DRONE + w_c[None, :] + w_i
        vel = vel + a_tot * DT
        pos = pos + vel * DT
        assert np.all(pos[:, 2] > 0.0), 'below ground'
        assert np.all(np.linalg.norm(vel, axis=1) < 10.0), 'speed'
        assert np.all(np.abs(np.linalg.det(Rm) - 1.0) < 1e-6), 'attitude not a rotation'
        min_sep = min(min_sep, float(dist[np.triu_indices(N, 1)].min()))
        if record:
            log['t'].append(t + DT); log['pos'].append(pos.copy()); log['R'].append(Rm.copy())
            log['att'].append(att.copy()); log['err'].append(np.linalg.norm(pos - targets, axis=1))
            log['col'].append(col.max()); log['phase'].append(phase + (stage.min() if phase == 2
                                                                        else 0))
        if phase == 2:
            errs = np.linalg.norm(pos - targets, axis=1)
            if t_seated is None and errs.max() <= DELTA_SWITCH and att.max() <= DELTA_ATT_FINAL:
                t_seated = t + DT
            if t_fit_all is None and stage.min() == 1:
                t_fit_all = t + DT
            if t_fit_all is not None:
                hold['err'].append(errs.max()); hold['gap'].append(np.abs(gap).max())
                hold['att'].append(att.max()); hold['fc'].append(fmag.max())
                hold['e6'].append(np.sqrt(errs ** 2 + (K_R / K_IMP) * att ** 2).max())
                if t + DT - t_fit_all >= T_HOLD - 1e-9:
                    t_end = t + DT
                    break
        if phase < 2 and t >= T_LIMIT:
            break
    res = dict(t_asc=t_asc, t_conv=t_conv, t_fit_all=t_fit_all, t_seated=t_seated, t_end=t_end,
               converged=t_end is not None, col_max=col_max, col_time=col_steps * DT,
               col_adj_max=col_adj_max, seat_pen_max=seat_pen_max, min_sep=min_sep, any_max=any_max,
               col_phase=col_phase, col_is_adj=col_is_adj)
    if hold['err']:
        m = np.arange(len(hold['err'])) * DT >= 2.0           # S097: score the last 8 s
        for key in hold:
            res[f'hold_{key}'] = float(np.array(hold[key])[m].max())
        res['final_att'] = float(hold['att'][-1])
    if record:
        for key in log:
            log[key] = np.array(log[key])
        log['targets'], log['p_app'] = targets, p_app
        res['log'] = log
    return res


KEYS = ['t_asc', 't_conv', 't_fit_all', 't_seated', 't_end', 'col_max', 'col_time', 'col_adj_max',
        'seat_pen_max', 'any_max', 'min_sep', 'hold_err', 'hold_gap', 'hold_att', 'hold_fc', 'hold_e6',
        'final_att', 'col_phase', 'col_is_adj']
PHASE_NAMES = ['ascent', 'consensus approach', 'in-place alignment', 'fit / insertion']


def _job(args):
    variant, seed, kw = args
    r = run_simulation(variant, seed=seed, **kw)
    return {k: (np.nan if r.get(k) is None else float(r[k])) for k in KEYS}


def batch(pool, jobs):
    """Run (variant, seed, kwargs) jobs; results come back in job order (deterministic)."""
    return pool.map(_job, jobs, chunksize=1)


def collect(rows):
    return {k: np.array([r[k] for r in rows]) for k in KEYS}


# ── Plots ─────────────────────────────────────────────────────────────────────

def plot_3d(rec, out_dir):
    fig = plt.figure(figsize=(17, 7))
    for i, var in enumerate(['flat_card', 'alt_card']):
        ax = fig.add_subplot(1, 2, i + 1, projection='3d')
        lg = rec[var]['log']
        P = lg['pos']
        for k in range(N):
            ax.plot(P[:, k, 0], P[:, k, 1], P[:, k, 2], color=DRONE_COLORS[k], lw=1.2,
                    label=f'Drone {k}')
            ax.scatter(*P[0, k], color=DRONE_COLORS[k], s=30, facecolors='none')
            ax.scatter(*lg['targets'][k], color=DRONE_COLORS[k], marker='*', s=90)
        kc = rec[var]['k_switch']
        V = piece_vertices(P[kc], lg['R'][kc])
        ax.add_collection3d(Poly3DCollection(list(V), facecolors=DRONE_COLORS, alpha=0.35,
                                             edgecolor='k', lw=0.5))
        ax.set_xlim(-5.5, 5.5); ax.set_ylim(-5.5, 5.5); ax.set_zlim(7.5, 10.6)
        ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
        r = rec[var]
        ax.set_title(f'{LABELS[var]} (seed 0)\npieces drawn at the 50 mm switch, before '
                     f'alignment; max collision depth {1000 * r["col_max"]:.0f} mm', fontsize=10)
        ax.view_init(elev=22, azim=-60)
        if i == 0:
            ax.legend(fontsize=7, loc='upper left')
    fig.tight_layout()
    path = os.path.join(out_dir, 'trajectory_3d.png')
    fig.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_timeseries(rec, out_dir):
    fig, axes = plt.subplots(2, 2, figsize=(15, 9))
    for var in ['flat_conc', 'flat_card', 'alt_card']:
        lg = rec[var]['log']
        t = lg['t']
        axes[0, 1].plot(t, np.degrees(lg['att'].max(1)), color=VCOL[var], label=LABELS[var])
        axes[1, 0].semilogy(t, np.maximum(lg['err'].max(1), 1e-4), color=VCOL[var],
                            label=LABELS[var])
        axes[1, 1].plot(t, 1000 * lg['col'], color=VCOL[var], label=LABELS[var])
    lg = rec['alt_card']['log']
    for k in range(N):
        axes[0, 0].plot(lg['t'], lg['pos'][:, k, 2], color=DRONE_COLORS[k], label=f'drone {k}')
    lgf = rec['flat_card']['log']
    axes[0, 0].plot(lgf['t'], lgf['pos'][:, 0, 2], 'k--', lw=1, label='flat (drone 0)')
    axes[0, 0].set_ylabel('z (m)'); axes[0, 0].set_title('Altitude, alternating stagger, card '
                                                          'sequence (dashed: flat)')
    axes[0, 1].axhline(2.0, color='grey', ls=':', label='2 deg (align switch)')
    axes[0, 1].axhline(1.0, color='grey', ls='--', label='1 deg (success)')
    axes[0, 1].set_ylabel('max attitude error (deg)'); axes[0, 1].set_title('Piece attitude error')
    axes[1, 0].axhline(DELTA_SWITCH, color='grey', ls=':', label='50 mm switch')
    axes[1, 0].axhline(DELTA_FIT, color='grey', ls='--', label='5 mm')
    axes[1, 0].set_ylabel('max 3D slot error (m, log)'); axes[1, 0].set_title('Slot error')
    axes[1, 1].set_ylabel('collision depth (mm)')
    axes[1, 1].set_title('Piece collisions (overlap of unseated pieces)')
    for ax in axes.flat:
        ax.set_xlabel('Time (s)'); ax.legend(fontsize=7)
    for ax in (axes[0, 1], axes[1, 0], axes[1, 1]):
        ax.set_xlim(10, None)
    fig.suptitle('S097 3D, seed 0: altitude, attitude, slot error and collisions')
    fig.tight_layout()
    path = os.path.join(out_dir, 'time_series.png')
    fig.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_sweeps(mc, dzs, hs, out_dir):
    fig, axes = plt.subplots(1, 3, figsize=(18, 5))
    ax = axes[0]
    keys = list(VARIANTS)
    data = [1000 * mc[k]['col_max'] for k in keys]
    bp = ax.boxplot(data, tick_labels=keys, patch_artist=True)
    for patch, k in zip(bp['boxes'], keys):
        patch.set_facecolor(VCOL[k]); patch.set_alpha(0.5)
    ax.set_ylabel('max collision depth (mm)')
    ax.set_title(f'Collision depth, {N_MC} seeds')
    ax.tick_params(axis='x', labelsize=7)
    ax = axes[1]
    for pat, mk in [('alt', 'o-'), ('cos', 's--')]:
        ax.plot(DZ_SWEEP, [1000 * np.mean(dzs[(pat, d)]['col_max']) for d in DZ_SWEEP], mk,
                label=f'{pat} stagger: mean max collision depth')
    ax.axvline(DELTA_Z, color='grey', ls=':', label='card dz = 0.25 m')
    ax.set_xlabel('stagger amplitude dz (m)'); ax.set_ylabel('collision depth (mm)')
    ax.set_title(f'Stagger sweep, card sequence ({N_SWEEP} seeds)')
    ax2 = ax.twinx()
    for pat, mk in [('alt', 'o:'), ('cos', 's:')]:
        ax2.plot(DZ_SWEEP, [np.mean(dzs[(pat, d)]['t_seated']) for d in DZ_SWEEP], mk,
                 color='grey', alpha=0.6, label=f'{pat}: time to seated (s)')
    ax2.set_ylabel('time to all seated (s), grey')
    h1, l1 = ax.get_legend_handles_labels()
    h2, l2 = ax2.get_legend_handles_labels()
    ax.legend(h1 + h2, l1 + l2, fontsize=7)
    ax = axes[2]
    hsj = [h * JITTER for h in HEAD_SWEEP]
    for var in ['flat_card', 'alt_card', 'flat_conc']:
        ax.plot(np.degrees(hsj), [1000 * np.mean(hs[(var, h)]['col_max']) for h in HEAD_SWEEP],
                'o-', color=VCOL[var], label=LABELS[var])
    ax.set_xlabel('max start heading offset (deg)'); ax.set_ylabel('mean max collision depth (mm)')
    ax.set_title(f'Heading-offset sweep ({N_SWEEP} seeds)')
    ax.legend(fontsize=7)
    fig.tight_layout()
    path = os.path.join(out_dir, 'collision_sweeps.png')
    fig.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def save_animation(res, out_dir, step=12):
    lg = res['log']
    n = len(lg['t'])
    k0 = max(0, res['k_switch'] - 150)              # start 3 s before the switch
    idx = [min(k0 + i * step, n - 1) for i in range(math.ceil((n - 1 - k0) / step) + 1)]
    fig = plt.figure(figsize=(9, 7))
    ax = fig.add_subplot(111, projection='3d')

    def update(fi):
        k = idx[fi]
        ax.cla()
        V = piece_vertices(lg['pos'][k], lg['R'][k])
        ax.add_collection3d(Poly3DCollection(list(V), facecolors=DRONE_COLORS, alpha=0.45,
                                             edgecolor='k', lw=0.6))
        ax.scatter(lg['pos'][k, :, 0], lg['pos'][k, :, 1], lg['pos'][k, :, 2], c=DRONE_COLORS,
                   s=25)
        hx = np.vstack([lg['targets'], lg['targets'][0]])
        ax.plot(hx[:, 0], hx[:, 1], hx[:, 2], 'k--', lw=0.8)
        ax.set_xlim(-2.2, 2.2); ax.set_ylim(-2.2, 2.2); ax.set_zlim(9.4, 10.6)
        ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
        ax.view_init(elev=18, azim=-60)
        ax.set_title(f'S097 3D, alternating stagger, card sequence (seed 0)\n'
                     f't = {lg["t"][k]:5.2f} s, max attitude error '
                     f'{np.degrees(lg["att"][k].max()):5.2f} deg, max slot error '
                     f'{1000 * lg["err"][k].max():6.1f} mm', fontsize=9)
        return []

    anim = animation.FuncAnimation(fig, update, frames=len(idx), interval=60, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    anim.save(path, writer='pillow', fps=16, dpi=65)
    plt.close(fig)
    print(f'Saved: {path}  ({len(idx)} frames, step {step}, samples {k0}-{n - 1})')


# ── Main ──────────────────────────────────────────────────────────────────────

def line(tag, m):
    def ms(a, f=1.0, d=2):
        return f'{np.nanmean(a) * f:.{d}f} ± {np.nanstd(a) * f:.{d}f}'
    print(f'  {tag:44s} done {int(np.isfinite(m["t_end"]).sum())}/{len(m["t_end"])}; switch '
          f'{ms(m["t_conv"])} s, all fitting {ms(m["t_fit_all"])} s, all seated (50 mm, 1 deg) '
          f'{ms(m["t_seated"])} s; collision depth {ms(m["col_max"], 1000, 1)} mm (max '
          f'{1000 * np.nanmax(m["col_max"]):.1f}, seeds > 1 mm: '
          f'{int(np.sum(m["col_max"] > 1e-3))}), collision time {ms(m["col_time"])} s; '
          f'seated overlap max {1000 * np.nanmax(m["seat_pen_max"]):.1f} mm; any overlap (no seated '
          f'exclusion) {ms(m["any_max"], 1000, 1)} mm, free of any overlap '
          f'{int(np.sum(m["any_max"] == 0))}/{len(m["any_max"])}; hold: slot '
          f'{ms(m["hold_err"], 1000)} mm, edge gap {ms(m["hold_gap"], 1000)} mm, attitude '
          f'{ms(m["hold_att"], 180 / np.pi, 3)} deg, eps_6DOF {ms(m["hold_e6"], 1000)} mm, '
          f'contact force max {np.nanmax(m["hold_fc"]):.3f} N; min separation '
          f'{np.nanmin(m["min_sep"]):.3f} m')


def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    print('=== S097 3D Aerial Puzzle Assembly ===')
    wr = math.sqrt(K_R / I_EFF)
    zr = B_R / (2 * math.sqrt(K_R * I_EFF))
    print(f'Rotational impedance: omega_n = {wr:.3f} rad/s, zeta = {zr:.3f} (card says 1.0); '
          f'translational omega_n = {math.sqrt(K_IMP / M_DRONE):.3f} rad/s, zeta = '
          f'{B_IMP / (2 * math.sqrt(K_IMP * M_DRONE)):.3f}')
    for pat in ('cos', 'alt'):
        s = stagger(DELTA_Z, pat)
        print(f'Stagger {pat}: dz_k = ' + ', '.join(f'{x:+.3f}' for x in s)
              + ' m; neighbour separations ' + ', '.join(
                  f'{abs(s[k] - s[(k + 1) % N]):.3f}' for k in range(N)) + ' m')
    tilt = np.degrees(THETA_TILT * np.sin(ANG))
    V0 = piece_vertices(np.column_stack([R_HEX * np.cos(ANG), R_HEX * np.sin(ANG),
                                         np.full(N, Z_ASM)]), R_STAR)
    ext = V0[:, :, 2].max(1) - V0[:, :, 2].min(1) + T_PANEL
    print('Target tilt per piece (deg): ' + ', '.join(f'{x:+.2f}' for x in tilt)
          + '; vertical extent incl. 20 mm panel: ' + ', '.join(f'{x:.3f}' for x in ext) + ' m')
    print(f'eps_6DOF weight: card (K_t/K_r) = {K_IMP / K_R:.3f} 1/m^2 (not a length); used '
          f'K_r/K_t = {K_R / K_IMP:.3f} m^2/rad^2 -> 1 deg = '
          f'{1000 * math.sqrt(K_R / K_IMP) * math.radians(1):.2f} mm')
    print(f'Tiled pieces at their slots: max overlap {1000 * interference(V0).max():.2f} mm '
          f'(adjacent pieces share edges)')

    # reproduction: flat, concurrent, alpha_z = alpha -> S097 consensus strategy (translation)
    r2 = m2d.run_simulation('consensus', seed=0)
    r3 = run_simulation('flat_conc', seed=0, alpha_z=ALPHA_XY)
    print(f'Reproduction (flat, concurrent, alpha_z = {ALPHA_XY}): switch {r3["t_conv"]:.2f} s '
          f'(S097 {r2["t_conv"]:.2f} s), worst slot error over the hold '
          f'{1000 * r3["hold_err"]:.2f} mm (S097 {1000 * r2["hold_err_max"]:.2f} mm)')
    assert abs(r3['t_conv'] - r2['t_conv']) < 1e-9 and abs(r3['hold_err'] - r2['hold_err_max']) < 1e-12

    rec = {}
    print('\nSeed 0:')
    for var in VARIANTS:
        rec[var] = run_simulation(var, seed=0, record=True)
        rec[var]['k_switch'] = int(round(rec[var]['t_conv'] / DT))
        line(LABELS[var], collect([{k: (np.nan if rec[var].get(k) is None else
                                         float(rec[var][k])) for k in KEYS}]))

    with Pool(N_WORKERS) as pool:
        print(f'\nMonte Carlo ({N_MC} seeds, same wind / bias / noise / jitter per seed):')
        jobs = [(v, s, {}) for v in VARIANTS for s in range(N_MC)]
        out = batch(pool, jobs)
        mc = {v: collect(out[i * N_MC:(i + 1) * N_MC]) for i, v in enumerate(VARIANTS)}
        for v in VARIANTS:
            line(LABELS[v], mc[v])
        b = mc['flat_conc']
        for v in ['flat_card', 'cos_card', 'alt_card', 'alt_conc']:
            m = mc[v]
            d = m['t_seated'] - b['t_seated']
            print(f'  {v} vs flat_conc (per seed): seated later in {int(np.sum(d > 0))}/{N_MC} '
                  f'by {np.mean(d):+.2f} s (min {np.min(d):+.2f}, max {np.max(d):+.2f}); '
                  f'collision-free seeds {int(np.sum(m["col_max"] == 0))}/{N_MC} vs '
                  f'{int(np.sum(b["col_max"] == 0))}/{N_MC}; hold slot error diff '
                  f'{1000 * np.mean(m["hold_err"] - b["hold_err"]):+.2f} mm, edge gap diff '
                  f'{1000 * np.mean(m["hold_gap"] - b["hold_gap"]):+.2f} mm')
        for v in VARIANTS:
            m = mc[v]
            hit = np.isfinite(m['col_phase'])
            if hit.any():
                print(f'  {v}: deepest collision per seed in phase ' + ', '.join(
                    f'{PHASE_NAMES[q]} {int(np.sum(m["col_phase"][hit] == q))}' for q in range(4))
                      + f'; between adjacent pieces in {int(np.sum(m["col_is_adj"][hit] == 1))}/'
                      f'{int(hit.sum())}, non-adjacent in '
                      f'{int(np.sum(m["col_is_adj"][hit] == 0))}/{int(hit.sum())}')
        a, c = mc['alt_card'], mc['flat_card']
        print(f'  alt_card vs flat_card: collision depth smaller in '
              f'{int(np.sum(a["col_max"] < c["col_max"]))}/{N_MC}, seated '
              f'{np.mean(a["t_seated"] - c["t_seated"]):+.2f} s')

        print(f'\nStagger sweep (card sequence, {N_SWEEP} seeds):')
        jobs = [('alt_card', s, dict(dz=d, pattern=p)) for p in ('alt', 'cos') for d in DZ_SWEEP
                for s in range(N_SWEEP)]
        out = batch(pool, jobs)
        dzs = {}
        i = 0
        for p in ('alt', 'cos'):
            for d in DZ_SWEEP:
                dzs[(p, d)] = collect(out[i:i + N_SWEEP]); i += N_SWEEP
                m = dzs[(p, d)]
                print(f'  {p} dz {d:.2f} m: collision depth mean {1000 * np.mean(m["col_max"]):.1f}'
                      f' mm (max {1000 * np.max(m["col_max"]):.1f}, adjacent pairs max '
                      f'{1000 * np.max(m["col_adj_max"]):.1f}), collision-free '
                      f'{int(np.sum(m["col_max"] == 0))}/{N_SWEEP}, all seated '
                      f'{np.mean(m["t_seated"]):.2f} s, all fitting {np.mean(m["t_fit_all"]):.2f} s,'
                      f' hold slot {1000 * np.mean(m["hold_err"]):.2f} mm, gap '
                      f'{1000 * np.mean(m["hold_gap"]):.2f} mm')

        print(f'\nHeading-offset sweep ({N_SWEEP} seeds; offsets = scale x S097 start jitter '
              f'+-{JITTER} rad):')
        hv = ['flat_card', 'alt_card', 'flat_conc']
        jobs = [(v, s, dict(head_scale=h)) for v in hv for h in HEAD_SWEEP for s in range(N_SWEEP)]
        out = batch(pool, jobs)
        hs = {}
        i = 0
        for v in hv:
            for h in HEAD_SWEEP:
                hs[(v, h)] = collect(out[i:i + N_SWEEP]); i += N_SWEEP
                m = hs[(v, h)]
                print(f'  {v:9s} max offset {math.degrees(h * JITTER):5.1f} deg: collision depth '
                      f'mean {1000 * np.mean(m["col_max"]):.1f} mm (max '
                      f'{1000 * np.max(m["col_max"]):.1f}), collision-free '
                      f'{int(np.sum(m["col_max"] == 0))}/{N_SWEEP}, all seated '
                      f'{np.mean(m["t_seated"]):.2f} s')

        print(f'\nPiece-size sweep ({N_SWEEP} seeds; card half-side 0.35 m = side 0.7 m, S097 '
              f'tiling side {SIDE:.4f} m; edge clearance at the slots = (S097 side - side) / sqrt 3):')
        jobs = [(v, s, dict(side=sd)) for sd in SIDE_SWEEP for v in VARIANTS for s in range(N_SWEEP)]
        out = batch(pool, jobs)
        i = 0
        for sd in SIDE_SWEEP:
            for v in VARIANTS:
                m = collect(out[i:i + N_SWEEP]); i += N_SWEEP
                print(f'  side {sd:.4f} m (clearance {1000 * (SIDE - sd) / math.sqrt(3):6.1f} mm) '
                      f'{v:9s}: collision depth mean {1000 * np.mean(m["col_max"]):.1f} mm (max '
                      f'{1000 * np.max(m["col_max"]):.1f}), collision-free {int(np.sum(m["col_max"] == 0))}'
                      f'/{N_SWEEP}; any overlap max {1000 * np.max(m["any_max"]):.1f} mm, free of any '
                      f'overlap {int(np.sum(m["any_max"] == 0))}/{N_SWEEP}; all seated '
                      f'{np.mean(m["t_seated"]):.2f} s')

        print(f'\nAlignment-switch probe (alt_card, {N_SWEEP} seeds; card switch 2 deg):')
        jobs = [('alt_card', s, dict(att_switch=math.radians(a))) for a in ATT_SWITCH_SWEEP
                for s in range(N_SWEEP)]
        out = batch(pool, jobs)
        for i, a in enumerate(ATT_SWITCH_SWEEP):
            m = collect(out[i * N_SWEEP:(i + 1) * N_SWEEP])
            ph = m['col_phase'][np.isfinite(m['col_phase'])]
            print(f'  switch {a:.1f} deg: collision depth mean {1000 * np.mean(m["col_max"]):.1f} mm (max '
                  f'{1000 * np.max(m["col_max"]):.1f}), collision-free {int(np.sum(m["col_max"] == 0))}/'
                  f'{N_SWEEP}, deepest collision in fit / insertion in {int(np.sum(ph == 3))}, all seated '
                  f'{np.mean(m["t_seated"]):.2f} s')

        print(f'\nCard cos stagger with the concurrent sequence ({N_SWEEP} seeds):')
        m = collect(batch(pool, [('alt_conc', s, dict(pattern='cos')) for s in range(N_SWEEP)]))
        print(f'  cos_conc: collision depth mean {1000 * np.mean(m["col_max"]):.2f} mm (max '
              f'{1000 * np.max(m["col_max"]):.1f}), collision-free {int(np.sum(m["col_max"] == 0))}/'
              f'{N_SWEEP}, all seated {np.mean(m["t_seated"]):.2f} s')

        print(f'\n2D baseline at other fixed assembly altitudes (flat, concurrent, {N_SWEEP} '
              f'seeds; staging altitude {Z_STAGE} m):')
        jobs = [('flat_conc', s, dict(z_asm=z)) for z in ZASM_SWEEP for s in range(N_SWEEP)]
        out = batch(pool, jobs)
        for i, z in enumerate(ZASM_SWEEP):
            m = collect(out[i * N_SWEEP:(i + 1) * N_SWEEP])
            print(f'  z_asm {z:4.1f} m: ascent {np.mean(m["t_asc"]):.2f} s, switch '
                  f'{np.mean(m["t_conv"]):.2f} s, all seated {np.mean(m["t_seated"]):.2f} s, '
                  f'collision-free {int(np.sum(m["col_max"] == 0))}/{N_SWEEP}, hold slot '
                  f'{1000 * np.mean(m["hold_err"]):.2f} mm, gap {1000 * np.mean(m["hold_gap"]):.2f} mm')

    print()
    plot_3d(rec, OUTPUT_DIR)
    plot_timeseries(rec, OUTPUT_DIR)
    plot_sweeps(mc, dzs, hs, OUTPUT_DIR)
    save_animation(rec['alt_card'], OUTPUT_DIR)


if __name__ == '__main__':
    main()
