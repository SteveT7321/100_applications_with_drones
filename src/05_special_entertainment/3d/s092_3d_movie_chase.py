"""
S092 3D Upgrade — Movie Chase Scene
===================================
The S092 chase (car on x = 8 t, y = 15 sin(2 pi x / 80); 20 Hz position fixes, sigma 0.3 m;
10-step MPC, dt 0.2 s, Q/R/P_f = 10/0.1/20 plus the S092 velocity-reference weight Q_v = 10;
pillar half-planes imposed at every 0.05 s of the horizon; soft constraints with slack;
pillars relocated 0.5 m from the camera path) is moved into a 3D obstacle world:

  * the car climbs a 2 m logistic ramp (x = 60 ... 100 m) -> its altitude is measured by a
    barometer (sigma 0.1 m) and tracked by a 6-state KF;
  * the five pillars have a finite height (card: 4 m; swept 4 / 6 / 8 m) so the drone may fly
    OVER them (z >= H + 0.5 m) instead of around them;
  * a bridge deck (slab x 140-160 m, |y| <= 20 m, z 3-6 m) spans the road: the car drives
    under it, a drone at the nominal altitude would fly into it, and the drone has to choose
    underfly (z <= 2.5 m, keeps the shot) or overfly (z >= 6.5 m, the deck hides the car);
  * the desired offset is 8 m behind and 8 tan 30 = 4.62 m above the car (the card's 5 m
    puts the camera at 32.0 deg, outside its own +-30 deg gimbal envelope);
  * the MPC adds a linearised gimbal-pitch-rate penalty (w_phi = 2) and separate vertical
    limits (|v_z| <= 4 m/s, |a_z| <= 3 m/s^2).

Variants (same measurement noise per seed):
  2d          S092 controller at a fixed altitude 4.62 m: pillars treated as infinitely tall
              (always go around), no knowledge of the deck, horizontal KF          (2D baseline, X5)
  3d_card     6-state KF, pillar mode (around / over) chosen by the MPC, deck mode by the
              card rule (smaller |dz| from the current altitude)                    (card)
  3d_under    as 3d_card, deck always underflown (keep the shot)                    (added)
  3d_around   3D, but pillars always avoided horizontally (no overfly)             (added ablation)
  3d_nogimbal 3d_under without the gimbal-rate penalty (w_phi = 0)                 (added ablation)
  3d_nozpred  3d_under with the car altitude held at its current estimate over the horizon
              (no vertical prediction)                                              (added ablation)
  3d_oracle   3d_under with the true future reference                              (added reference)

Usage:
    conda activate drones
    python src/05_special_entertainment/3d/s092_3d_movie_chase.py
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
    's092_2d', os.path.join(_HERE, '..', 's092_movie_chase.py'))
m2d = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(m2d)

OUTPUT_DIR = os.path.normpath(os.path.join(
    _HERE, '..', '..', '..', 'outputs', '05_special_entertainment', '3d', 's092_3d_movie_chase'))

# ── Parameters (card / S092) ──────────────────────────────────────────────────
CAR_VX      = m2d.CAR_VX              # 8 m/s along x
T_TOTAL     = m2d.T_TOTAL             # 37.5 s
DT_SIM      = m2d.DT_SIM              # 0.05 s
DT_MPC      = m2d.DT_MPC              # 0.2 s
N_HOR       = m2d.N_HOR               # 10
SUB         = m2d.SUB                 # 4
N_C         = m2d.N_C                 # 40 constraint instants per horizon
TAU_C       = m2d.TAU_C
GP, GV, GS  = m2d.GP, m2d.GV, m2d.GS
W_POS       = m2d.W_POS
Q_MPC, R_MPC, Q_VEL = m2d.Q_MPC, m2d.R_MPC, m2d.Q_VEL
W_SLACK_Q, W_SLACK_L = m2d.W_SLACK_Q, m2d.W_SLACK_L
OBS_RANGE   = m2d.OBS_RANGE           # 40 m
H_CAR       = m2d.H_CAR               # 1.5 m
FOV_DEG     = m2d.FOV_DEG             # 60 deg
BAND        = m2d.BAND_FIXED          # 10-20 % (S092 corrected band)
BAND_CARD   = m2d.BAND_CARD           # 20-40 % (card)
TOL_BALL    = m2d.TOL_BALL            # 1 m
PILLAR_RADIUS = m2d.PILLAR_RADIUS     # 0.6 m
R_SAFE      = m2d.R_SAFE              # 1.5 m
PILLARS     = m2d.PILLARS             # relocated (0.5 m from the camera path)
SIGMA_XY    = m2d.SIGMA_MEAS          # 0.3 m
SIGMA_Z     = 0.1                     # m (barometer, card)
Q_NOISE     = m2d.Q_NOISE             # 1.0 m^2/s^3
T_PRED      = m2d.T_PRED              # 2 s
H_RAMP, W_RAMP = 2.0, 3.0             # m (card)
S1, S2      = 60.0, 100.0             # m ramp centres (added: card gives none)
H_PIL_CARD  = 4.0                     # m (card)
Z_CLEAR     = 0.5                     # m vertical clearance (card)
DECK_X      = (140.0, 160.0)          # m (card)
DECK_Y      = (-20.0, 20.0)           # m (card)
DECK_Z      = (3.0, 6.0)              # m underside 3 m (card z_deck), 3 m thick (card "3 m tall")
Z_FLOOR, Z_CEIL = 1.0, 12.0           # m (card)
PHI_MAX     = np.radians(30.0)        # gimbal envelope
PHI_RATE_MAX = np.radians(30.0)       # rad/s
W_PHI       = 2.0
V_XY, A_XY  = 12.0, 6.0               # per axis (card / S092)
V_Z, A_Z    = 4.0, 3.0                # card
D_BACK      = 8.0                     # m behind (card / S092)
DZ_CARD     = 5.0                     # m above (card)
DZ_OFF      = D_BACK * np.tan(PHI_MAX)  # 4.62 m (card's own envelope rule)

# ── Parameters (added) ────────────────────────────────────────────────────────
N_MC        = 10                      # seeds (as S092)
SEED        = 9200                    # noise of seed s from default_rng(SEED + s)
H_PIL_SWEEP = [4.0, 6.0, 8.0]
OVER_ROWS_R = R_SAFE + 1.0            # m: horizon instants this close (horizontally) get the overfly constraint
DECK_MARGIN = R_SAFE                  # m: horizon instants within the deck x-span +- this get the deck constraint
W_PHI_SWEEP = [0.0, 2.0, 200.0, 2000.0]  # gimbal-rate weight probe (card value 2)
N_WORKERS   = 4
CLEAR_SUB   = m2d.CLEAR_SUB           # 10 fine points per 0.05 s
LOS_SAMPLES = 40
MAIN_SEED   = 0
ANIM_STEP   = 4                       # 0.2 s per frame

VARIANTS = ['2d', '3d_card', '3d_under', '3d_around', '3d_nogimbal', '3d_nozpred', '3d_oracle']
LAB = {'2d': '2D baseline: fixed z = 4.62 m', '3d_card': '3D, card deck rule',
       '3d_under': '3D, underfly deck', '3d_around': '3D, no pillar overfly',
       '3d_nogimbal': '3D underfly, w_phi = 0', '3d_nozpred': '3D underfly, no z prediction',
       '3d_oracle': '3D underfly, oracle reference'}
COL = {'2d': 'grey', '3d_card': 'darkorange', '3d_under': 'red', '3d_around': 'tab:brown',
       '3d_nogimbal': 'tab:pink', '3d_nozpred': 'tab:cyan', '3d_oracle': 'black'}


# ── World ─────────────────────────────────────────────────────────────────────

def sigmoid(u):
    return 1.0 / (1.0 + np.exp(-u))


def car_state(t):
    """3D car position, velocity and heading (x = 8 t, so the arc parameter s = x)."""
    p2, v2, psi = m2d.car_state(t)
    x = p2[..., 0]
    z = H_RAMP * (sigmoid((x - S1) / W_RAMP) - sigmoid((x - S2) / W_RAMP))
    dzdx = H_RAMP / W_RAMP * (sigmoid((x - S1) / W_RAMP) * (1 - sigmoid((x - S1) / W_RAMP))
                              - sigmoid((x - S2) / W_RAMP) * (1 - sigmoid((x - S2) / W_RAMP)))
    p = np.stack([x, p2[..., 1], z], -1)
    v = np.stack([v2[..., 0], v2[..., 1], dzdx * v2[..., 0]], -1)
    return p, v, psi


def offset_xy(c_xy, psi):
    return np.stack([c_xy[..., 0] - D_BACK * np.cos(psi), c_xy[..., 1] - D_BACK * np.sin(psi)], -1)


def desired(t):
    p, _, psi = car_state(t)
    return np.concatenate([offset_xy(p[..., :2], psi), (p[..., 2] + DZ_OFF)[..., None]], -1)


def in_deck(P, margin=0.0):
    return ((P[..., 0] >= DECK_X[0] - margin) & (P[..., 0] <= DECK_X[1] + margin) &
            (P[..., 1] >= DECK_Y[0] - margin) & (P[..., 1] <= DECK_Y[1] + margin) &
            (P[..., 2] >= DECK_Z[0] - margin) & (P[..., 2] <= DECK_Z[1] + margin))


def box_distance(P):
    lo = np.array([DECK_X[0], DECK_Y[0], DECK_Z[0]])
    hi = np.array([DECK_X[1], DECK_Y[1], DECK_Z[1]])
    d = np.maximum(np.maximum(lo - P, P - hi), 0.0)
    return np.linalg.norm(d, axis=-1)


def pillar_distance(P, h_pil):
    """3D distance from points P (..., 3) to the nearest pillar (capped cylinder) surface."""
    dxy = np.linalg.norm(P[..., None, :2] - PILLARS, axis=-1) - PILLAR_RADIUS     # (..., 5)
    dz = P[..., None, 2] - h_pil
    out = np.where(dz <= 0, np.maximum(dxy, 0.0),
                   np.where(dxy <= 0, dz, np.hypot(np.maximum(dxy, 0), dz)))
    return out.min(axis=-1)


def los_visible(P, C):
    """Line of sight from drone P to the car's mid-height C not blocked by the deck or a pillar."""
    s = np.linspace(0.0, 1.0, LOS_SAMPLES)[:, None]
    pts = P[None, :] + s * (C - P)[None, :]
    if np.any(in_deck(pts)):
        return False
    dxy = np.linalg.norm(pts[:, None, :2] - PILLARS[None], axis=2)
    return not np.any((dxy < PILLAR_RADIUS) & (pts[:, None, 2] < CURRENT_H_PIL[0]))


CURRENT_H_PIL = [H_PIL_CARD]          # set per run (module-level so los_visible stays simple)


# ── Kalman filter (6 states; 2D baseline uses only x, y) ─────────────────────

def kf_matrices():
    F = np.eye(6)
    F[:3, 3:] = DT_SIM * np.eye(3)
    q3, q2, q1 = DT_SIM ** 3 / 3, DT_SIM ** 2 / 2, DT_SIM
    Q = Q_NOISE * np.block([[q3 * np.eye(3), q2 * np.eye(3)], [q2 * np.eye(3), q1 * np.eye(3)]])
    H = np.hstack([np.eye(3), np.zeros((3, 3))])
    R = np.diag([SIGMA_XY ** 2, SIGMA_XY ** 2, SIGMA_Z ** 2])
    return F, Q, H, R


F_KF, Q_KF, H_KF, R_KF = kf_matrices()


# ── MPC ───────────────────────────────────────────────────────────────────────

def build_qp(p0, v0, ref_full, car_ref, cfg, modes, h_pil, p_lin):
    """Condensed QP in u (3 x N, axis-major) + one slack per soft constraint group.
    modes: dict pillar_index -> 'around' | 'over', plus 'deck' -> None | 'under' | 'over'."""
    ref = ref_full[1:]
    v_ref = np.diff(ref_full, axis=0) / DT_MPC
    free = p0[None, :] + m2d._IDX[:, None] * DT_MPC * v0[None, :]
    E = ref - free
    free_c = p0[None, :] + TAU_C[:, None] * v0[None, :]
    nU = 3 * N_HOR
    groups = []                                        # (rows (n, nU), rhs (n,)) as A u - s <= b
    for k, mode in modes.items():
        if k == 'deck':
            if mode is None:
                continue
            rows = np.where((p_lin[:, 0] >= DECK_X[0] - DECK_MARGIN) & (p_lin[:, 0] <= DECK_X[1] + DECK_MARGIN))[0]
            if len(rows) == 0:
                continue
            A = np.zeros((len(rows), nU))
            if mode == 'under':                         # z(tau) <= z_under
                A[:, 2 * N_HOR:] = GS[rows]
                b = (DECK_Z[0] - Z_CLEAR) - free_c[rows, 2]
            else:                                       # z(tau) >= z_over
                A[:, 2 * N_HOR:] = -GS[rows]
                b = free_c[rows, 2] - (DECK_Z[1] + Z_CLEAR)
            groups.append((A, b))
            continue
        o = PILLARS[k]
        if mode == 'around':                            # n.(p_xy - o) >= R_SAFE at every instant
            d = p_lin[:, :2] - o
            n = d / np.linalg.norm(d, axis=1, keepdims=True)
            A = np.zeros((N_C, nU))
            A[:, :N_HOR] = -n[:, 0:1] * GS
            A[:, N_HOR:2 * N_HOR] = -n[:, 1:2] * GS
            b = np.sum((free_c[:, :2] - o) * n, axis=1) - R_SAFE
            groups.append((A, b))
        else:                                           # z(tau) >= H + clearance near the pillar
            rows = np.where(np.linalg.norm(p_lin[:, :2] - o, axis=1) < OVER_ROWS_R)[0]
            if len(rows) == 0:
                continue
            A = np.zeros((len(rows), nU))
            A[:, 2 * N_HOR:] = -GS[rows]
            b = free_c[rows, 2] - (h_pil + Z_CLEAR)
            groups.append((A, b))
    K = len(groups)
    nz = nU + K
    H = np.zeros((nz, nz))
    g = np.zeros(nz)
    G2 = GP.T @ (W_POS[:, None] * GP) + R_MPC * np.eye(N_HOR)
    for a in range(3):
        sl = slice(a * N_HOR, (a + 1) * N_HOR)
        H[sl, sl] = 2 * G2 + 2 * Q_VEL * GV.T @ GV
        g[sl] = -2 * GP.T @ (W_POS * E[:, a]) - 2 * Q_VEL * GV.T @ (v_ref[:, a] - v0[a])
    if cfg['w_phi'] > 0:
        # Linearised gimbal pitch phi = atan2(z_c - z, d_xy) along the reference; rate penalty
        c = car_ref[1:]
        dvec = ref[:, :2] - c[:, :2]
        dxy = np.linalg.norm(dvec, axis=1)
        dzc = c[:, 2] - ref[:, 2]
        den = dxy ** 2 + dzc ** 2
        J = np.zeros((N_HOR, nU))                      # d phi_i / d u
        jx = -dzc / den * dvec[:, 0] / dxy
        jy = -dzc / den * dvec[:, 1] / dxy
        jz = -dxy / den
        phi_ref = np.arctan2(dzc, dxy)
        for a, ja in enumerate((jx, jy, jz)):
            J[:, a * N_HOR:(a + 1) * N_HOR] = ja[:, None] * GP
        phi0 = np.arctan2(car_ref[0, 2] - p0[2], np.linalg.norm(p0[:2] - car_ref[0, :2]))
        lin0 = phi_ref + jx * (free[:, 0] - ref[:, 0]) + jy * (free[:, 1] - ref[:, 1]) + \
            jz * (free[:, 2] - ref[:, 2])                 # phi_i at u = 0 (linearised)
        D = np.zeros((N_HOR, nU))
        cvec = np.zeros(N_HOR)
        D[0] = J[0]
        cvec[0] = lin0[0] - phi0
        D[1:] = J[1:] - J[:-1]
        cvec[1:] = lin0[1:] - lin0[:-1]
        wq = cfg['w_phi'] / DT_MPC ** 2
        H[:nU, :nU] += 2 * wq * D.T @ D
        g[:nU] += 2 * wq * D.T @ cvec
    for k in range(K):
        H[nU + k, nU + k] = 2 * W_SLACK_Q
        g[nU + k] = W_SLACK_L
    rows, rhs = [], []
    for a, (vm, am) in enumerate(((V_XY, A_XY), (V_XY, A_XY), (V_Z, A_Z))):
        for sgn in (1.0, -1.0):
            C = np.zeros((N_HOR, nz))
            C[:, a * N_HOR:(a + 1) * N_HOR] = sgn * GV
            rows.append(C)
            rhs.append(vm - sgn * np.full(N_HOR, v0[a]))
            C = np.zeros((N_HOR, nz))
            C[:, a * N_HOR:(a + 1) * N_HOR] = sgn * np.eye(N_HOR)
            rows.append(C)
            rhs.append(np.full(N_HOR, am))
    # altitude floor / ceiling at the knots (hard)
    for sgn, lim in ((1.0, Z_CEIL), (-1.0, -Z_FLOOR)):
        C = np.zeros((N_HOR, nz))
        C[:, 2 * N_HOR:3 * N_HOR] = sgn * GP
        rows.append(C)
        rhs.append(lim - sgn * free[:, 2])
    for k, (A, b) in enumerate(groups):
        C = np.zeros((len(b), nz))
        C[:, :nU] = A
        C[:, nU + k] = -1.0
        rows.append(C)
        rhs.append(b)
        C = np.zeros((1, nz))
        C[0, nU + k] = -1.0
        rows.append(C)
        rhs.append(np.zeros(1))
    return H, g, np.vstack(rows), np.concatenate(rhs), nU, K


def solve_mpc(p0, v0, ref_full, car_ref, cfg, h_pil, p_lin, deck_mode):
    """Enumerate the around/over choice of every nearby pillar (deck mode fixed by the caller);
    keep the combination with the lowest QP objective (first one on ties)."""
    near = [k for k, o in enumerate(PILLARS) if np.linalg.norm(p0[:2] - o) < OBS_RANGE]
    choice = [k for k in near if cfg['overfly'] and
              np.min(np.linalg.norm(p_lin[:, :2] - PILLARS[k], axis=1)) < OVER_ROWS_R + R_SAFE]
    best = None
    n_combo = 2 ** len(choice)
    for code in range(n_combo):
        modes = {k: 'around' for k in near}
        for b, k in enumerate(choice):
            modes[k] = 'over' if (code >> b) & 1 else 'around'
        modes['deck'] = deck_mode
        H, g, G, h, nU, K = build_qp(p0, v0, ref_full, car_ref, cfg, modes, h_pil, p_lin)
        z, ok, _ = m2d.qp_ipm(H, g, G, h)
        obj = 0.5 * z @ H @ z + g @ z
        if best is None or obj < best[0] - 1e-9:
            best = (obj, z, ok, K, modes, nU)
    obj, z, ok, K, modes, nU = best
    u = z[:nU].reshape(3, N_HOR)
    pred = p0[None, :] + TAU_C[:, None] * v0[None, :] + GS @ u.T
    slack = bool(K and z[nU:].max() > 1e-6)
    n_over = sum(1 for k, m in modes.items() if k != 'deck' and m == 'over')
    lim = np.array([A_XY, A_XY, A_Z])
    return np.clip(u[:, 0], -lim, lim), pred, ok, slack, n_over


# ── Simulation ────────────────────────────────────────────────────────────────

CFG = {
    '2d':          dict(dim=2, overfly=False, deck='none', w_phi=0.0, pred='kf'),
    '3d_card':     dict(dim=3, overfly=True, deck='card', w_phi=W_PHI, pred='kf'),
    '3d_under':    dict(dim=3, overfly=True, deck='under', w_phi=W_PHI, pred='kf'),
    '3d_around':   dict(dim=3, overfly=False, deck='under', w_phi=W_PHI, pred='kf'),
    '3d_nogimbal': dict(dim=3, overfly=True, deck='under', w_phi=0.0, pred='kf'),
    '3d_nozpred':  dict(dim=3, overfly=True, deck='under', w_phi=W_PHI, pred='kf_noz'),
    '3d_oracle':   dict(dim=3, overfly=True, deck='under', w_phi=W_PHI, pred='oracle'),
}


def simulate(variant, seed, h_pil=H_PIL_CARD, keep=False, w_phi=None):
    cfg = dict(CFG[variant])
    if w_phi is not None:
        cfg['w_phi'] = w_phi
    CURRENT_H_PIL[0] = h_pil
    rng = np.random.default_rng(SEED + seed)
    n_steps = int(round(T_TOTAL / DT_SIM))
    t_all = np.arange(n_steps + 1) * DT_SIM
    noise = rng.standard_normal((n_steps + 1, 3)) * np.array([SIGMA_XY, SIGMA_XY, SIGMA_Z])
    pc, vc, psi = car_state(t_all)
    pd = desired(t_all)
    z_fix = DZ_OFF                                       # 2D baseline altitude (car at z = 0 in 2D)
    p = pd[0].copy() if cfg['dim'] == 3 else np.array([pd[0, 0], pd[0, 1], z_fix])
    v = np.zeros(3)
    u = np.zeros(3)
    x_kf, P_kf = None, np.eye(6)
    pred_prev = None
    deck_mode = None
    log = {k: [] for k in ('p', 'v', 'u')}
    cnt = dict(solves=0, fail=0, slack=0, over=0, deck_over=0, deck_under=0)
    for k in range(n_steps + 1):
        t = t_all[k]
        zm = pc[k] + noise[k]
        if x_kf is None:
            x_kf = np.array([zm[0], zm[1], zm[2], CAR_VX, 0.0, 0.0])
        else:
            x_kf = F_KF @ x_kf
            P_kf = F_KF @ P_kf @ F_KF.T + Q_KF
        Kk = P_kf @ H_KF.T @ np.linalg.inv(H_KF @ P_kf @ H_KF.T + R_KF)
        x_kf = x_kf + Kk @ (zm - H_KF @ x_kf)
        P_kf = (np.eye(6) - Kk @ H_KF) @ P_kf
        if k % SUB == 0 and k < n_steps:
            tau = np.arange(N_HOR + 1) * DT_MPC
            if cfg['pred'] == 'oracle':
                car_ref, _, psi_r = car_state(t + tau)
            else:
                car_ref = x_kf[None, :3] + tau[:, None] * x_kf[None, 3:]
                if cfg['pred'] == 'kf_noz':
                    car_ref[:, 2] = x_kf[2]
                psi_r = np.full(N_HOR + 1, math.atan2(x_kf[4], x_kf[3]))
            ref = np.concatenate([offset_xy(car_ref[:, :2], psi_r),
                                  (car_ref[:, 2] + DZ_OFF)[:, None]], 1)
            if cfg['dim'] == 2:
                ref[:, 2] = z_fix
                car_ref = car_ref.copy()
                car_ref[:, 2] = 0.0
            if pred_prev is None:
                p_lin = np.column_stack([np.interp(TAU_C, tau, ref[:, a]) for a in range(3)])
            else:
                p_lin = np.vstack([pred_prev[SUB:], np.repeat(pred_prev[-1:], SUB, 0)])
            # deck mode, chosen once per planning cycle when the deck is within the horizon
            dm = None
            if cfg['deck'] != 'none' and np.any((p_lin[:, 0] >= DECK_X[0] - DECK_MARGIN) &
                                                (p_lin[:, 0] <= DECK_X[1] + DECK_MARGIN)):
                if cfg['deck'] == 'under':
                    dm = 'under'
                else:                                    # card rule: smaller |dz| from the current altitude
                    du = abs(p[2] - (DECK_Z[0] - Z_CLEAR))
                    do = abs(p[2] - (DECK_Z[1] + Z_CLEAR))
                    dm = 'under' if du < do else 'over'
                cnt['deck_' + dm] += 1
            u, pred_prev, ok, slack, n_over = solve_mpc(p, v, ref, car_ref, cfg, h_pil, p_lin, dm)
            if cfg['dim'] == 2:
                u[2] = 0.0
            cnt['solves'] += 1
            cnt['fail'] += int(not ok)
            cnt['slack'] += int(slack)
            cnt['over'] += int(n_over > 0)
        log['p'].append(p.copy())
        log['v'].append(v.copy())
        log['u'].append(u.copy())
        if k == n_steps:
            break
        p = p + v * DT_SIM + 0.5 * u * DT_SIM ** 2
        v = v + u * DT_SIM
        assert np.all(np.abs(u[:2]) <= A_XY + 1e-9) and abs(u[2]) <= A_Z + 1e-9
        assert p[2] > 0.0
    for key in log:
        log[key] = np.array(log[key])
    P, V, U = log['p'], log['v'], log['u']
    assert np.abs(V[:, :2]).max() <= V_XY + 0.05 and np.abs(V[:, 2]).max() <= V_Z + 0.05
    # fine path (constant acceleration within each 0.05 s step)
    ss = np.linspace(0.0, DT_SIM, CLEAR_SUB, endpoint=False)
    Pf = (P[:-1, None, :] + V[:-1, None, :] * ss[None, :, None]
          + 0.5 * U[:-1, None, :] * ss[None, :, None] ** 2).reshape(-1, 3)
    Pf = np.vstack([Pf, P[-1:]])
    dxy_f = np.linalg.norm(Pf[:, None, :2] - PILLARS[None], axis=2)
    pillar_in = int(np.sum(np.any((dxy_f < PILLAR_RADIUS) & (Pf[:, None, 2] < h_pil), axis=1)))
    deck_in = int(np.sum(in_deck(Pf)))
    if cfg['dim'] == 3:
        assert pillar_in == 0, f'{variant}: drone inside a pillar'
        assert deck_in == 0, f'{variant}: drone inside the bridge deck'
    clear3d = np.minimum(pillar_distance(Pf, h_pil), box_distance(Pf))
    # horizontal pillar clearance while below the top, vertical clearance above it when overhead
    below = Pf[:, None, 2] < h_pil
    hclear = np.where(below, dxy_f, np.inf).min()
    over = dxy_f < R_SAFE
    vclear = np.where(over & ~below, Pf[:, None, 2] - h_pil, np.inf).min()
    err = np.linalg.norm(P - pd, axis=1)
    dvec = P - pc
    dist = np.linalg.norm(dvec - np.array([0, 0, H_CAR / 2]), axis=1)     # to the car's centre
    phi = np.arctan2(pc[:, 2] + H_CAR / 2 - P[:, 2], np.linalg.norm(dvec[:, :2], axis=1))
    phi_rate = np.abs(np.diff(phi)) / DT_SIM
    vis = np.array([los_visible(P[i], pc[i] + np.array([0, 0, H_CAR / 2])) for i in range(len(P))])
    f_h = 2 * np.arctan(H_CAR / (2 * dist)) * np.cos(phi) / np.radians(FOV_DEG)
    f_ok = vis & (f_h >= BAND[0]) & (f_h <= BAND[1])
    bridge = (pd[:, 0] >= DECK_X[0] - DECK_MARGIN) & (pd[:, 0] <= DECK_X[1] + DECK_MARGIN)
    ramp = (pc[:, 0] >= S1 - 15) & (pc[:, 0] <= S2 + 15)
    steady = t_all >= m2d.T_SKIP
    res = {
        'rms': float(np.sqrt(np.mean(err ** 2))),
        'rms_steady': float(np.sqrt(np.mean(err[steady] ** 2))),
        'rms_nobridge': float(np.sqrt(np.mean(err[steady & ~bridge] ** 2))),
        'rms_xy_nb': float(np.sqrt(np.mean(np.sum((P - pd)[steady & ~bridge, :2] ** 2, axis=1)))),
        'rms_z_nb': float(np.sqrt(np.mean((P - pd)[steady & ~bridge, 2] ** 2))),
        'pillar_hmin': float(np.linalg.norm(Pf[:, None, :2] - PILLARS[None], axis=2).min()),
        'in_ball': float(np.mean(err <= TOL_BALL)),
        'z_rms_ramp': float(np.sqrt(np.mean((P[ramp, 2] - pd[ramp, 2]) ** 2))),
        'frame': float(np.mean(f_ok)), 'frame_card': float(np.mean(vis & (f_h >= BAND_CARD[0]) & (f_h <= BAND_CARD[1]))),
        'visible': float(np.mean(vis)), 'vis_bridge': float(np.mean(vis[bridge])),
        'gimbal_in': float(np.mean(np.abs(phi) <= PHI_MAX + 1e-9)),
        'rate_ok': float(np.mean(phi_rate <= PHI_RATE_MAX)),
        'rate_max': float(np.degrees(phi_rate.max())),
        'rate_rms_s': float(np.degrees(np.sqrt(np.mean(phi_rate[steady[1:]] ** 2)))),
        'rate_max_s': float(np.degrees(phi_rate[steady[1:]].max())),
        'clear3d': float(clear3d.min()), 'hclear': float(hclear), 'vclear': float(vclear),
        'pillar_in': pillar_in, 'deck_in': deck_in,
        'max_speed': float(np.linalg.norm(V, axis=1).max()), 'max_vxy_inf': float(np.abs(V[:, :2]).max()),
        'max_vz': float(np.abs(V[:, 2]).max()), 'zmin': float(P[:, 2].min()), 'zmax': float(P[:, 2].max()),
        **{f'n_{k2}': v2 for k2, v2 in cnt.items()},
    }
    if keep:
        res.update({'t': t_all, 'P': P, 'pd': pd, 'pc': pc, 'phi': phi, 'f_h': f_h, 'vis': vis,
                    'err': err, 'clear_t': clear3d[::CLEAR_SUB][:len(P)]})
    return res


def _job(args):
    return simulate(*args)


def run_simulation():
    from concurrent.futures import ProcessPoolExecutor
    jobs = [(v, s, H_PIL_CARD, s == MAIN_SEED) for v in VARIANTS for s in range(N_MC)]
    sweep_vars = ['2d', '3d_under', '3d_around']
    jobs += [(v, s, h, False) for h in H_PIL_SWEEP[1:] for v in sweep_vars for s in range(N_MC)]
    wjobs = [('3d_under', s, H_PIL_CARD, False, w) for w in W_PHI_SWEEP for s in range(N_MC)]
    with ProcessPoolExecutor(max_workers=min(N_WORKERS, os.cpu_count() or 1)) as ex:
        out = list(ex.map(_job, jobs + wjobs, chunksize=1))
    wout = out[len(jobs):]
    out = out[:len(jobs)]
    wsweep = {w: wout[i * N_MC:(i + 1) * N_MC] for i, w in enumerate(W_PHI_SWEEP)}
    mc = {v: [] for v in VARIANTS}
    main = {}
    sweep = {}
    for (v, s, h, keep), r in zip(jobs, out):
        if h == H_PIL_CARD:
            mc[v].append(r)
            if keep:
                main[v] = r
        else:
            sweep.setdefault((h, v), []).append(r)
    for v in sweep_vars:
        sweep[(H_PIL_CARD, v)] = mc[v]
    return main, mc, sweep, wsweep, len(jobs) + len(wjobs)


# ── Plotting ──────────────────────────────────────────────────────────────────

def _cyl(ax, c, r, h):
    th = np.linspace(0, 2 * np.pi, 24)
    TH, ZZ = np.meshgrid(th, np.array([0.0, h]))
    ax.plot_surface(c[0] + r * np.cos(TH), c[1] + r * np.sin(TH), ZZ, color='grey', alpha=0.6, lw=0)


def _deck(ax):
    x0, x1 = DECK_X
    y0, y1 = DECK_Y
    z0, z1 = DECK_Z
    for zz in (z0, z1):
        X, Y = np.meshgrid([x0, x1], [y0, y1])
        ax.plot_surface(X, Y, np.full_like(X, zz), color='grey', alpha=0.25, lw=0)
    for yy in (y0, y1):
        X, Z = np.meshgrid([x0, x1], [z0, z1])
        ax.plot_surface(X, np.full_like(X, yy), Z, color='grey', alpha=0.25, lw=0)


def plot_trajectory_3d(main, out_dir):
    fig = plt.figure(figsize=(15, 7))
    ax = fig.add_subplot(111, projection='3d')
    r = main['3d_under']
    ax.plot(*r['pc'].T, color='blue', lw=2, label='car (with ramp)')
    ax.plot(*r['pd'].T, 'g--', lw=1.1, label='desired offset (8 m behind, 4.62 m above)')
    for v, ls in (('3d_under', '-'), ('3d_card', '-'), ('2d', ':')):
        P = main[v]['P']
        ax.plot(*P.T, color=COL[v], lw=1.4 if v != '2d' else 1.1, ls=ls, label=LAB[v])
    for c in PILLARS:
        _cyl(ax, c, PILLAR_RADIUS, H_PIL_CARD)
    _deck(ax)
    ax.plot([], [], color='grey', lw=6, label='pillars (4 m) and bridge deck (z 3-6 m)')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_zlabel('z (m)')
    ax.set_zlim(0, 10)
    ax.set_box_aspect((6, 1.6, 1.1))
    ax.view_init(elev=18, azim=-62)
    ax.legend(loc='upper left', fontsize=8)
    ax.set_title(f'S092 3D chase (seed {MAIN_SEED})')
    path = os.path.join(out_dir, 'trajectory_3d.png')
    plt.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def bridge_span(r):
    m = (r['pd'][:, 0] >= DECK_X[0] - DECK_MARGIN) & (r['pd'][:, 0] <= DECK_X[1] + DECK_MARGIN)
    t = r['t'][m]
    return t.min(), t.max()


def plot_altitude(main, out_dir):
    fig, ax = plt.subplots(figsize=(13, 5))
    r = main['3d_under']
    t = r['t']
    b0, b1 = bridge_span(r)
    ax.axvspan(b0, b1, color='grey', alpha=0.2, label='drone set-point under the deck span')
    ax.axhspan(DECK_Z[0], DECK_Z[1], xmin=0, xmax=1, color='none')
    ax.plot(t, r['pc'][:, 2], 'b-', lw=1.5, label='car z (ramp)')
    ax.plot(t, r['pd'][:, 2], 'g--', lw=1, label='desired z (car + 4.62 m)')
    for v in ('3d_under', '3d_card', '3d_nozpred', '2d'):
        ax.plot(t, main[v]['P'][:, 2], color=COL[v], lw=1.1, label=LAB[v])
    ax.fill_between([b0, b1], DECK_Z[0], DECK_Z[1], color='k', alpha=0.25, label='deck slab z 3-6 m')
    ax.axhline(Z_FLOOR, color='k', ls=':', lw=0.8)
    ax.axhline(H_PIL_CARD, color='grey', ls='-.', lw=0.8, label='pillar top 4 m')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Altitude (m)')
    ax.set_ylim(0, 9)
    ax.legend(fontsize=7, ncol=3, loc='upper left')
    ax.grid(alpha=0.3)
    ax.set_title(f'Altitude (seed {MAIN_SEED})')
    plt.tight_layout()
    path = os.path.join(out_dir, 'altitude.png')
    plt.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_gimbal_framing(main, out_dir):
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(13, 7.5), sharex=True)
    for v in ('3d_under', '3d_card', '3d_nogimbal', '2d'):
        r = main[v]
        a1.plot(r['t'], np.degrees(r['phi']), color=COL[v], lw=1, label=LAB[v])
        fh = np.where(r['vis'], 100 * r['f_h'], np.nan)
        a2.plot(r['t'], fh, color=COL[v], lw=1, label=f"{LAB[v]} (in band & visible {100 * r['frame']:.1f}%)")
    b0, b1 = bridge_span(main['3d_under'])
    for ax in (a1, a2):
        ax.axvspan(b0, b1, color='grey', alpha=0.2)
        ax.grid(alpha=0.3)
    a1.axhline(-30, color='k', ls='--', lw=0.8)
    a1.axhline(30, color='k', ls='--', lw=0.8)
    a1.set_ylabel('Gimbal pitch to car (deg)')
    a1.legend(fontsize=7, ncol=2)
    a2.axhspan(100 * BAND[0], 100 * BAND[1], color='green', alpha=0.12)
    a2.set_ylabel('Car height in frame (%)\n(gap = car hidden)')
    a2.set_xlabel('Time (s)')
    a2.legend(fontsize=7, ncol=2)
    a1.set_title(f'Gimbal pitch and framing (seed {MAIN_SEED}); grey = deck span, green = 10-20 % band')
    plt.tight_layout()
    path = os.path.join(out_dir, 'gimbal_framing.png')
    plt.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_clearance(main, out_dir):
    fig, ax = plt.subplots(figsize=(13, 4.5))
    for v in ('3d_under', '3d_card', '3d_around', '2d'):
        r = main[v]
        ax.plot(r['t'], r['clear_t'], color=COL[v], lw=1, label=f"{LAB[v]} (min {r['clear3d']:.2f} m)")
    ax.axhline(Z_CLEAR, color='r', ls=':', lw=1, label='vertical clearance 0.5 m')
    ax.axhline(R_SAFE - PILLAR_RADIUS, color='r', ls='--', lw=1, label='r_safe from the pillar axis (0.9 m to the surface)')
    ax.set_yscale('symlog', linthresh=0.5)
    ax.set_ylim(0, 60)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('3D distance to nearest obstacle surface (m)')
    ax.legend(fontsize=7)
    ax.grid(alpha=0.3)
    ax.set_title(f'Obstacle clearance (seed {MAIN_SEED}); 0 = inside an obstacle')
    plt.tight_layout()
    path = os.path.join(out_dir, 'clearance.png')
    plt.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_monte_carlo(mc, sweep, out_dir):
    fig, axes = plt.subplots(1, 4, figsize=(19, 4.8))
    keys = VARIANTS
    short = ['2D', 'card\nrule', 'under', 'no\noverfly', 'w_phi\n= 0', 'no z\npred', 'oracle']
    for ax, (key, lab, sc) in zip(axes[:3], (('rms_xy_nb', 'Horizontal RMS error, t >= 3 s, outside deck span (m)', 1),
                                             ('frame', 'Framed & visible, 10-20 % band (%)', 100),
                                             ('z_rms_ramp', 'Altitude RMS error on the ramp (m)', 1))):
        ax.boxplot([[sc * r[key] for r in mc[v]] for v in keys], showmeans=True)
        ax.set_xticks(range(1, len(keys) + 1), short, fontsize=8)
        ax.set_ylabel(lab)
        ax.grid(alpha=0.3)
    ax = axes[3]
    for v, mk in (('2d', 'o'), ('3d_under', 's'), ('3d_around', '^')):
        ax.plot(H_PIL_SWEEP, [np.mean([r['rms_xy_nb'] for r in sweep[(h, v)]]) for h in H_PIL_SWEEP],
                marker=mk, color=COL[v], label=LAB[v])
    ax.set_xlabel('Pillar height (m)')
    ax.set_ylabel('Horizontal RMS error, t >= 3 s, outside deck span (m)')
    ax.set_title('Pillar-height sweep')
    ax.legend(fontsize=7)
    ax.grid(alpha=0.3)
    fig.suptitle(f'S092 3D Monte Carlo, {N_MC} seeds (same measurement noise for every variant)')
    plt.tight_layout()
    path = os.path.join(out_dir, 'monte_carlo.png')
    plt.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def save_animation(main, out_dir):
    r = main['3d_under']
    P, pc, pd, t = r['P'], r['pc'], r['pd'], r['t']
    n = len(t)
    idx = [min(i * ANIM_STEP, n - 1) for i in range(int(np.ceil((n - 1) / ANIM_STEP)) + 1)]
    fig = plt.figure(figsize=(8, 7.5))
    ax = fig.add_subplot(211, projection='3d')
    ax2 = fig.add_subplot(212)
    ax2.plot(pc[:, 0], pc[:, 2], 'b-', lw=1, alpha=0.4)
    ax2.plot(P[:, 0], P[:, 2], 'r-', lw=0.8, alpha=0.4)
    ax2.fill_between(DECK_X, DECK_Z[0], DECK_Z[1], color='grey', alpha=0.5)
    for c in PILLARS:
        ax2.fill_between([c[0] - PILLAR_RADIUS, c[0] + PILLAR_RADIUS], 0, H_PIL_CARD, color='grey', alpha=0.5)
    car2, = ax2.plot([], [], 'bs', ms=6)
    dr2, = ax2.plot([], [], 'r^', ms=7)
    ax2.set_xlim(0, 300)
    ax2.set_ylim(0, 9)
    ax2.set_xlabel('x (m)')
    ax2.set_ylabel('z (m)')
    ax2.set_title('Side elevation')

    def update(fi):
        i = idx[fi]
        ax.cla()
        lo, hi = max(0, pc[i, 0] - 30), pc[i, 0] + 20
        m = (pc[:, 0] >= lo) & (pc[:, 0] <= hi)
        ax.plot(*pc[m].T, color='blue', lw=1, alpha=0.5)
        ax.plot(*P[max(0, i - 60):i + 1].T, color='red', lw=1)
        ax.scatter(*pc[i], color='blue', s=40)
        ax.scatter(*P[i], color='red', marker='^', s=50)
        ax.scatter(*pd[i], color='green', marker='x', s=40)
        ax.plot([P[i, 0], pc[i, 0]], [P[i, 1], pc[i, 1]], [P[i, 2], pc[i, 2] + H_CAR / 2],
                color='k' if r['vis'][i] else 'r', ls=':', lw=0.8)
        for c in PILLARS:
            if lo - 2 <= c[0] <= hi + 2:
                _cyl(ax, c, PILLAR_RADIUS, H_PIL_CARD)
        if DECK_X[1] >= lo and DECK_X[0] <= hi:
            _deck(ax)
        ax.set_xlim(lo, hi)
        ax.set_ylim(-20, 20)
        ax.set_zlim(0, 9)
        ax.set_box_aspect((2.0, 1.6, 0.6))
        ax.view_init(elev=22, azim=-70 + 0.3 * fi)
        ax.set_title(f't = {t[i]:.1f} s  pitch {np.degrees(r["phi"][i]):+.0f} deg  '
                     f'{"car visible" if r["vis"][i] else "CAR HIDDEN"}', fontsize=9)
        car2.set_data([pc[i, 0]], [pc[i, 2]])
        dr2.set_data([P[i, 0]], [P[i, 2]])
        return car2, dr2

    ani = animation.FuncAnimation(fig, update, frames=len(idx), blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer='pillow', fps=10, dpi=60)
    plt.close(fig)
    print(f'Saved: {path} ({len(idx)} frames)')


# ── Main ──────────────────────────────────────────────────────────────────────

def ms(vals, f='{:.3f}', sc=1.0):
    a = sc * np.asarray(vals, float)
    return f'{f.format(a.mean())} ± {f.format(a.std())}'


if __name__ == '__main__':
    main, mc, sweep, wsweep, n_jobs = run_simulation()
    print('=== S092 3D Movie Chase ===')
    print(f'  Runs: {n_jobs} ({len(VARIANTS)} variants x {N_MC} seeds + pillar-height and w_phi sweeps), '
          f'process pool of {N_WORKERS}, results in job order; noise of seed s from default_rng({SEED} + s)')
    print(f'  Card offset 8 m behind / {DZ_CARD:.0f} m above -> gimbal pitch {np.degrees(np.arctan2(DZ_CARD, D_BACK)):.1f} deg '
          f'(envelope +-30 deg); envelope-compliant height 8 tan 30 = {DZ_OFF:.3f} m; nominal pitch to the car '
          f'centre ({H_CAR / 2:.2f} m) {np.degrees(np.arctan2(DZ_OFF - H_CAR / 2, D_BACK)):.1f} deg, nominal framing '
          f'{100 * 2 * np.arctan(H_CAR / (2 * np.hypot(D_BACK, DZ_OFF - H_CAR / 2))) * np.cos(np.arctan2(DZ_OFF - H_CAR / 2, D_BACK)) / np.radians(FOV_DEG):.1f} %')
    tt = np.linspace(0, T_TOTAL, 20001)
    pcz = car_state(tt)[0][:, 2]
    print(f'  Car ramp: z max {pcz.max():.3f} m; 5-95 % rise over {2 * np.log(19) * W_RAMP:.2f} m of road')
    print(f'  Pillars (relocated as in S092) at x = ' + ', '.join(f'{c[0]:.1f}' for c in PILLARS) +
          f' m; pillar inside the deck span: {int(np.sum((PILLARS[:, 0] >= DECK_X[0]) & (PILLARS[:, 0] <= DECK_X[1])))}')
    print(f'  Deck: x {DECK_X}, z {DECK_Z} m; nominal set-point altitude there {DZ_OFF:.2f} m is inside the slab; '
          f'underfly z <= {DECK_Z[0] - Z_CLEAR:.1f} m (|dz| {DZ_OFF - (DECK_Z[0] - Z_CLEAR):.2f} m), '
          f'overfly z >= {DECK_Z[1] + Z_CLEAR:.1f} m (|dz| {DECK_Z[1] + Z_CLEAR - DZ_OFF:.2f} m)')
    hdr = (f"  {'Variant':<34}{'RMS (m)':>16}{'RMS t>=3 s, no deck (m)':>26}{'within 1 m':>13}{'framed+visible':>16}"
           f"{'visible (deck span)':>21}{'gimbal in ±30°':>16}{'rate <= 30°/s':>15}{'ramp z RMS':>16}")
    print(hdr)
    for v in VARIANTS:
        R = mc[v]
        g = lambda k: [r[k] for r in R]
        print(f"  {LAB[v]:<34}{ms(g('rms')):>16}{ms(g('rms_nobridge')):>26}{ms(g('in_ball'), '{:.1f}', 100):>13}"
              f"{ms(g('frame'), '{:.1f}', 100):>16}{np.mean(g('visible')) * 100:>9.1f} ({np.mean(g('vis_bridge')) * 100:5.1f})"
              f"{ms(g('gimbal_in'), '{:.1f}', 100):>16}{ms(g('rate_ok'), '{:.1f}', 100):>15}{ms(g('z_rms_ramp')):>16}")
    print('  Error split, t >= 3 s outside the deck span (mean over seeds): horizontal / vertical RMS; '
          'closest horizontal approach to any pillar axis (min over seeds)')
    for v in VARIANTS:
        R = mc[v]
        print(f"    {LAB[v]:<34} {ms([r['rms_xy_nb'] for r in R])} m / {ms([r['rms_z_nb'] for r in R])} m; "
              f"{min(r['pillar_hmin'] for r in R):.3f} m")
    print('  Safety and limits (over all seeds): min 3D clearance to any obstacle surface, min horizontal pillar '
          'clearance below the top (from the axis), min height above a pillar top when within r_safe, '
          'fine-path points inside a pillar / the deck, max |v_xy|_inf, max |v_z|, z range, max pitch rate')
    for v in VARIANTS:
        R = mc[v]
        print(f"    {LAB[v]:<34} {min(r['clear3d'] for r in R):.3f} m, {min(r['hclear'] for r in R):.3f} m, "
              f"{min(r['vclear'] for r in R):.3f} m, {sum(r['pillar_in'] for r in R)} / {sum(r['deck_in'] for r in R)}, "
              f"{max(r['max_vxy_inf'] for r in R):.2f} m/s, {max(r['max_vz'] for r in R):.2f} m/s, "
              f"{min(r['zmin'] for r in R):.2f}-{max(r['zmax'] for r in R):.2f} m, {max(r['rate_max'] for r in R):.1f} deg/s")
    print('  MPC: solves, not converged, slack-active, pillar-overfly chosen, deck under / over cycles (sum over seeds)')
    for v in VARIANTS:
        R = mc[v]
        s = lambda k: sum(r[k] for r in R)
        print(f"    {LAB[v]:<34} {s('n_solves')}, {s('n_fail')}, {s('n_slack')}, {s('n_over')}, "
              f"{s('n_deck_under')} / {s('n_deck_over')}")
    print('  Paired per seed against 3D underfly (same noise):')
    base = mc['3d_under']
    for v in VARIANTS:
        if v == '3d_under':
            continue
        R = mc[v]
        dr = np.array([a['rms_nobridge'] - b['rms_nobridge'] for a, b in zip(base, R)])
        dxy = np.array([a['rms_xy_nb'] - b['rms_xy_nb'] for a, b in zip(base, R)])
        df = np.array([a['frame'] - b['frame'] for a, b in zip(base, R)])
        dz = np.array([a['z_rms_ramp'] - b['z_rms_ramp'] for a, b in zip(base, R)])
        print(f"    vs {LAB[v]:<32} lower RMS (no deck) in {int(np.sum(dr < 0))}/{N_MC} (mean diff {dr.mean():+.3f} m); "
              f"lower horizontal RMS in {int(np.sum(dxy < 0))}/{N_MC} (mean diff {dxy.mean():+.3f} m); "
              f"higher framing in {int(np.sum(df > 0))}/{N_MC} (mean diff {100 * df.mean():+.1f} pp); "
              f"lower ramp z RMS in {int(np.sum(dz < 0))}/{N_MC} (mean diff {dz.mean():+.3f} m)")
    print('  Gimbal pitch rate after 3 s (mean over seeds): RMS / max (deg/s); whole-run max')
    for v in VARIANTS:
        R = mc[v]
        print(f"    {LAB[v]:<34} {np.mean([r['rate_rms_s'] for r in R]):.2f} / {np.mean([r['rate_max_s'] for r in R]):.2f}; "
              f"{max(r['rate_max'] for r in R):.1f}")
    print('  w_phi sweep (3D underfly): pitch-rate RMS after 3 s / RMS t >= 3 s outside deck / framed+visible')
    for w in W_PHI_SWEEP:
        R = wsweep[w]
        print(f"    w_phi = {w:6.0f}: {np.mean([r['rate_rms_s'] for r in R]):.3f} deg/s / "
              f"{np.mean([r['rms_nobridge'] for r in R]):.3f} m / {100 * np.mean([r['frame'] for r in R]):.1f}%")
    print('  Pillar-height sweep (mean over seeds): RMS t >= 3 s outside the deck span / within 1 m / '
          'pillar entries / overfly cycles')
    for h in H_PIL_SWEEP:
        print(f'    H = {h:.0f} m: ' + '; '.join(
            f"{LAB[v]} {np.mean([r['rms_nobridge'] for r in sweep[(h, v)]]):.3f} m (xy {np.mean([r['rms_xy_nb'] for r in sweep[(h, v)]]):.3f}) / "
            f"{100 * np.mean([r['in_ball'] for r in sweep[(h, v)]]):.1f}% / "
            f"{sum(r['pillar_in'] for r in sweep[(h, v)])} / {sum(r['n_over'] for r in sweep[(h, v)])}"
            for v in ('2d', '3d_under', '3d_around')))
        a = [r['rms_xy_nb'] for r in sweep[(h, '3d_under')]]
        b = [r['rms_xy_nb'] for r in sweep[(h, '3d_around')]]
        c = [r['rms_xy_nb'] for r in sweep[(h, '2d')]]
        print(f"      horizontal RMS: overfly vs no overfly lower in {int(np.sum(np.array(a) < np.array(b)))}/{N_MC} "
              f"(mean diff {np.mean(a) - np.mean(b):+.3f} m); 3D underfly vs 2D: {np.mean(a) - np.mean(c):+.3f} m")
    r0 = main['3d_under']
    print(f"  Seed 0, 3D underfly: RMS {r0['rms']:.3f} m, framed {100 * r0['frame']:.1f}%, min clearance "
          f"{r0['clear3d']:.3f} m; card rule: framed {100 * main['3d_card']['frame']:.1f}%, visible in deck span "
          f"{100 * main['3d_card']['vis_bridge']:.1f}%; 2D: deck points {main['2d']['deck_in']}")

    os.makedirs(OUTPUT_DIR, exist_ok=True)
    plot_trajectory_3d(main, OUTPUT_DIR)
    plot_altitude(main, OUTPUT_DIR)
    plot_gimbal_framing(main, OUTPUT_DIR)
    plot_clearance(main, OUTPUT_DIR)
    plot_monte_carlo(mc, sweep, OUTPUT_DIR)
    save_animation(main, OUTPUT_DIR)
