"""
S081 3D Upgrade — Selfie Follow Mode
====================================
The S081 follow mission (figure-8, R = 8 m, 1.5 m/s, 10 Hz GPS, orbit PD with the
2D script's 6 m/s^2 / 10 m/s horizontal limits, camera yaw pointed at the estimate)
is moved onto a height-varying path: the person's altitude follows
    z_tgt(t) = h_base + dh sin(2 pi t / T_elev) + sum_i dh_jump Lambda((t - t_i) / (T_jump/2))
with h_base = 0.5 m, dh = 1.5 m, T_elev = 20 s and eight 0.6 m / 0.8 s jumps.  GPS now
reports z too (sigma_z = 0.5 m, sigma_xy = 0.3 m).  The drone's altitude becomes a
controlled state: z_d = clip(z_hat + 2.5 m, 1.5, 8) tracked by a vertical PD with its
own gains and limits, and a gimbal tilt (first-order lag, tau = 0.1 s) points the
camera at the estimate.  Framing is scored on both image axes (|u| <= 0.1 and
|v| <= 0.1) for the TRUE person.

Strategies (same GPS noise per seed for every strategy — common random numbers):
  sph    3D KF + spherical standoff (r_xy = sqrt(5^2 - 2.5^2)) + altitude tracking  (card 1)
  cyl    3D KF + cylindrical standoff (r_xy = 5 m) + altitude tracking               (card 2)
  raw    raw 3D GPS + spherical standoff + altitude tracking                         (card 3)
  fix    2D baseline: same 3D KF, same r_xy and gimbal, but z held at 3.0 m
         (= h_base + dz_clear) — altitude is NOT a controlled state                  (added, X5)
  base   S081 exactly as built in 2D: horizontal KF, z = 4 m, r = 5 m, gimbal aimed at
         the flat-ground height h_base (the 2D script has no z estimate)             (added, X5)
  oracle true 3D state, spherical standoff + altitude tracking (controller floor)    (added)

Usage:
    conda activate drones
    python src/05_special_entertainment/3d/s081_3d_selfie_follow.py
"""

import os
import importlib.util
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401

_HERE = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location(
    's081_2d', os.path.join(_HERE, '..', 's081_selfie_follow.py'))
m2d = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(m2d)

OUTPUT_DIR = os.path.normpath(os.path.join(
    _HERE, '..', '..', '..', 'outputs', '05_special_entertainment', '3d', 's081_3d_selfie_follow'))

# ── Parameters (card) ─────────────────────────────────────────────────────────
R_FIGURE8   = m2d.R_FIGURE8          # 8 m
V_TARGET    = m2d.V_TARGET           # 1.5 m/s (arc-length parameterisation of the 2D script)
H_BASE      = 0.5                    # m
DH          = 1.5                    # m
T_ELEV      = 20.0                   # s
DH_JUMP     = 0.6                    # m
T_JUMP      = 0.8                    # s
SIG_XY      = 0.3                    # m
SIG_Z       = 0.5                    # m
GPS_FREQ    = 10.0                   # Hz
R_3D        = 5.0                    # m
DZ_CLEAR    = 2.5                    # m
R_XY        = float(np.sqrt(R_3D ** 2 - DZ_CLEAR ** 2))   # 4.330 m
BEARING_PHI = np.radians(135.0)
Z_MIN, Z_MAX = 1.5, 8.0              # m
FOV_H       = np.radians(60.0)
FOV_V       = np.radians(45.0)
FRAME_TOL   = 0.10
Q_XY, Q_Z   = 0.5, 0.8               # m/s^2
KP, KD      = 1.5, 0.8
KPZ, KDZ    = 1.0, 0.5
C_DRAG      = 0.3                    # 1/s
TAU_G       = 0.1                    # s
DT          = 0.05                   # s
T_TOTAL     = 60.0                   # s
N_STEPS     = int(round(T_TOTAL / DT))
GPS_EVERY   = int(round(1.0 / (GPS_FREQ * DT)))     # 2 steps per fix
N_FIX       = (N_STEPS - 1) // GPS_EVERY + 1

# ── Parameters (added) ────────────────────────────────────────────────────────
A_MAX       = m2d.A_MAX              # 6 m/s^2 horizontal limit (2D script)
V_MAX       = m2d.V_MAX              # 10 m/s horizontal limit (2D script)
A_Z_MAX     = 4.0                    # m/s^2 vertical acceleration limit
V_Z_MAX     = 3.0                    # m/s vertical speed limit
JUMP_TIMES  = 3.75 + 7.5 * np.arange(8)   # s — "occasionally jumps": every 7.5 s
Z_FIX       = H_BASE + DZ_CLEAR      # 3.0 m — 2D-baseline altitude (geometry exact at mean height)
Z_FIX_SWEEP = [2.0, 3.0, 4.0, 5.0]
BASE_ALT, BASE_R = 4.0, 5.0          # S081 2D values
N_MC        = 100
SEED        = 2026                   # noise of seed s drawn from default_rng(SEED + s)
A_Z_SWEEP   = [2.0, 4.0, 8.0, np.inf]
SIG_Z_SWEEP = [0.1, 0.25, 0.5, 1.0]
R3_SWEEP    = [4.0, 5.0, 6.0, 7.0]     # range probe: dz = r_3D / 2 keeps the 30 deg look-down
ANIM_STEP   = 6                      # 0.3 s per frame
DEPRESS_NOM = np.degrees(np.arcsin(DZ_CLEAR / R_3D))  # 30 deg design look-down angle

STRATS = ['sph', 'cyl', 'raw', 'fix', 'base', 'oracle']
LABELS = {'sph': '3D KF + Spherical PD', 'cyl': '3D KF + Cylinder PD',
          'raw': 'Raw 3D GPS + Spherical PD', 'fix': f'2D baseline: fixed z = {Z_FIX:.1f} m',
          'base': 'S081 as-is (z = 4 m, flat tilt)', 'oracle': 'Oracle (true state)'}
COLORS = {'sph': 'red', 'cyl': 'darkorange', 'raw': 'gold', 'fix': 'purple',
          'base': 'saddlebrown', 'oracle': 'grey'}
CONFIG = {
    'sph':    dict(est='kf', r_xy=R_XY, alt='track', tilt='est'),
    'cyl':    dict(est='kf', r_xy=R_3D, alt='track', tilt='est'),
    'raw':    dict(est='raw', r_xy=R_XY, alt='track', tilt='est'),
    'fix':    dict(est='kf', r_xy=R_XY, alt='fixed', z_fix=Z_FIX, tilt='est'),
    'base':   dict(est='kf', r_xy=BASE_R, alt='fixed', z_fix=BASE_ALT, tilt='flat'),
    'oracle': dict(est='oracle', r_xy=R_XY, alt='track', tilt='est'),
}


# ── Person trajectory ─────────────────────────────────────────────────────────

def target_altitude(t, jumps=True):
    t = np.atleast_1d(t)
    z = H_BASE + DH * np.sin(2 * np.pi * t / T_ELEV)
    vz = DH * 2 * np.pi / T_ELEV * np.cos(2 * np.pi * t / T_ELEV)
    if jumps:
        half = T_JUMP / 2
        for tj in JUMP_TIMES:
            s = (t - tj) / half
            z = z + DH_JUMP * np.maximum(0.0, 1 - np.abs(s))
            vz = vz + np.where(np.abs(s) < 1, -np.sign(s) * DH_JUMP / half, 0.0)
    return z, vz


def person(jumps=True):
    t = np.arange(N_STEPS) * DT
    pxy, vxy, _ = m2d.figure8(t)
    z, vz = target_altitude(t, jumps)
    return t, np.column_stack([pxy, z]), np.column_stack([vxy, vz])


def draw_noise(seeds):
    """Standard-normal GPS noise (S, N_FIX, 3); seed s uses default_rng(SEED + s)."""
    return np.stack([np.random.default_rng(SEED + s).standard_normal((N_FIX, 3)) for s in seeds])


def kf_matrices(sig_xy, sig_z):
    F = np.eye(6)
    F[:3, 3:] = DT * np.eye(3)
    H = np.hstack([np.eye(3), np.zeros((3, 3))])
    q = np.array([Q_XY, Q_XY, Q_Z]) ** 2
    Q = np.diag(np.concatenate([q * DT ** 3 / 3, q * DT]))
    R = np.diag([sig_xy ** 2, sig_xy ** 2, sig_z ** 2])
    return F, H, Q, R


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


# ── Simulation (vectorised over seeds) ────────────────────────────────────────

def simulate(cfg, noise, sig_z=SIG_Z, jumps=True, a_z_max=A_Z_MAX, tau_g=TAU_G, log=False):
    S = noise.shape[0]
    t_all, p_tgt, v_tgt = person(jumps)
    F, H, Q, Rg = kf_matrices(SIG_XY, sig_z)
    sig = np.array([SIG_XY, SIG_XY, sig_z])
    est, r_xy, alt, tilt = cfg['est'], cfg['r_xy'], cfg['alt'], cfg['tilt']
    dz_c, r3 = cfg.get('dz', DZ_CLEAR), cfg.get('r3', R_3D)

    x_kf = np.tile(np.concatenate([p_tgt[0], v_tgt[0]]), (S, 1))
    P = np.eye(6)
    pos_est = np.tile(p_tgt[0], (S, 1))
    vel_est = np.tile(v_tgt[0], (S, 1))
    z_prev = None

    def desired(pe, ve):
        ang = np.arctan2(ve[:, 1], ve[:, 0]) + BEARING_PHI
        xy = pe[:, :2] + r_xy * np.column_stack([np.cos(ang), np.sin(ang)])
        if alt == 'track':
            zr = pe[:, 2] + dz_c
            z = np.clip(zr, Z_MIN, Z_MAX)
        else:
            zr = z = np.full(S, cfg['z_fix'])
        return np.column_stack([xy, z]), zr

    p_d, _ = desired(pos_est, vel_est)
    p_drone = p_d.copy()
    v_drone = np.zeros((S, 3))
    p_d_prev = p_d.copy()
    phi = None
    alpha = 1.0 - np.exp(-DT / tau_g) if tau_g > 0 else 1.0

    keys = ['u', 'v', 'e3', 'ez', 'dep', 'zerr', 'xyerr']
    L = {k: np.zeros((N_STEPS, S)) for k in keys}
    if log:
        L['drone'] = np.zeros((N_STEPS, S, 3))
        L['phi'] = np.zeros((N_STEPS, S))
        L['phi_cmd'] = np.zeros((N_STEPS, S))
        L['zd'] = np.zeros((N_STEPS, S))
        L['yaw'] = np.zeros((N_STEPS, S))
        L['est'] = np.zeros((N_STEPS, S, 3))
    n_clip = n_guard = n_sat_xy = n_sat_z = 0
    az_peak = vz_peak = axy_peak = vxy_peak = 0.0
    min_dist = np.inf

    for k in range(N_STEPS):
        t = k * DT
        got_fix = (k % GPS_EVERY == 0)
        if got_fix:
            zm = p_tgt[k] + sig * noise[:, k // GPS_EVERY]
        if est == 'kf':
            if k > 0:
                x_kf = x_kf @ F.T
                P = F @ P @ F.T + Q
            if got_fix:
                K = P @ H.T @ np.linalg.inv(H @ P @ H.T + Rg)
                x_kf = x_kf + (zm - x_kf[:, :3]) @ K.T
                P = (np.eye(6) - K @ H) @ P
            pos_est, vel_est = x_kf[:, :3], x_kf[:, 3:]
        elif est == 'raw':
            if got_fix:
                if z_prev is not None:
                    vel_est = (zm - z_prev) / (GPS_EVERY * DT)
                pos_est = zm.copy()
                z_prev = zm.copy()
        else:
            pos_est, vel_est = np.tile(p_tgt[k], (S, 1)), np.tile(v_tgt[k], (S, 1))

        p_d, z_raw = desired(pos_est, vel_est)
        n_clip += int(np.sum(z_raw != p_d[:, 2]))
        pd_dot = (p_d - p_d_prev) / DT
        e = p_d - p_drone
        edot = pd_dot - v_drone
        # Horizontal channel: exactly the 2D script's PD + limits
        a_xy = KP * e[:, :2] + KD * edot[:, :2]
        an = np.linalg.norm(a_xy, axis=1, keepdims=True)
        n_sat_xy += int(np.sum(an > A_MAX))
        a_xy = np.where(an > A_MAX, a_xy / np.maximum(an, 1e-12) * A_MAX, a_xy)
        v_xy = v_drone[:, :2] + (a_xy - C_DRAG * v_drone[:, :2]) * DT
        vn = np.linalg.norm(v_xy, axis=1, keepdims=True)
        v_xy = np.where(vn > V_MAX, v_xy / np.maximum(vn, 1e-12) * V_MAX, v_xy)
        # Vertical channel: decoupled PD + braking envelope to the altitude bounds
        if alt == 'track':
            a_z = KPZ * e[:, 2] + KDZ * edot[:, 2]
            vz = v_drone[:, 2]
            room_dn = np.maximum(p_drone[:, 2] - Z_MIN, 0.0)
            room_up = np.maximum(Z_MAX - p_drone[:, 2], 0.0)
            env = (lambda r: np.sqrt(a_z_max * r)) if np.isfinite(a_z_max) else (lambda r: r / DT)
            v_lo = -np.minimum(env(room_dn), room_dn / DT)
            v_hi = np.minimum(env(room_up), room_up / DT)
            a_lo = (v_lo - vz) / DT + C_DRAG * vz
            a_hi = (v_hi - vz) / DT + C_DRAG * vz
            guard = (a_z < a_lo) | (a_z > a_hi)
            n_guard += int(np.sum(guard))
            a_z = np.clip(a_z, a_lo, a_hi)
            n_sat_z += int(np.sum(np.abs(a_z) > a_z_max))
            a_z = np.clip(a_z, -a_z_max, a_z_max)
            vz_new = np.clip(vz + (a_z - C_DRAG * vz) * DT, -V_Z_MAX, V_Z_MAX)
        else:
            a_z = np.zeros(S)
            vz_new = np.zeros(S)
        v_new = np.column_stack([v_xy, vz_new])
        p_new = p_drone + v_new * DT
        # Physical invariants
        assert np.all(np.linalg.norm(a_xy, axis=1) <= A_MAX + 1e-9)
        assert np.all(np.abs(a_z) <= a_z_max + 1e-9)
        assert np.all(np.linalg.norm(v_xy, axis=1) <= V_MAX + 1e-9)
        assert np.all(np.abs(vz_new) <= V_Z_MAX + 1e-9)
        assert np.all(p_new[:, 2] >= Z_MIN - 1e-9) and np.all(p_new[:, 2] <= Z_MAX + 1e-9)
        assert np.all(np.linalg.norm(p_new - p_drone, axis=1)
                      <= np.hypot(V_MAX, V_Z_MAX) * DT + 1e-9)
        az_peak = max(az_peak, float(np.abs(a_z).max()))
        vz_peak = max(vz_peak, float(np.abs(vz_new).max()))
        axy_peak = max(axy_peak, float(np.linalg.norm(a_xy, axis=1).max()))
        vxy_peak = max(vxy_peak, float(np.linalg.norm(v_xy, axis=1).max()))
        p_drone, v_drone, p_d_prev = p_new, v_new, p_d

        # Camera: yaw at the estimate (instant, as in 2D), tilt with first-order lag
        rel = pos_est - p_drone
        yaw = np.arctan2(rel[:, 1], rel[:, 0])
        z_aim = pos_est[:, 2] if tilt == 'est' else np.full(S, H_BASE)
        phi_cmd = np.arctan2(z_aim - p_drone[:, 2], np.hypot(rel[:, 0], rel[:, 1]))
        phi = phi_cmd.copy() if phi is None else phi + alpha * (phi_cmd - phi)
        cy, sy, cp, sp = np.cos(yaw), np.sin(yaw), np.cos(phi), np.sin(phi)
        look = np.column_stack([cp * cy, cp * sy, sp])
        right = np.column_stack([sy, -cy, np.zeros(S)])
        up = np.column_stack([-sp * cy, -sp * sy, cp])
        d = p_tgt[k] - p_drone
        dl = np.sum(d * look, axis=1)
        L['u'][k] = np.arctan2(np.sum(d * right, axis=1), dl) / (FOV_H / 2)
        L['v'][k] = np.arctan2(np.sum(d * up, axis=1), dl) / (FOV_V / 2)
        dist = np.linalg.norm(d, axis=1)
        dh = np.hypot(d[:, 0], d[:, 1])
        min_dist = min(min_dist, float(dist.min()))
        L['e3'][k] = dist - r3
        L['ez'][k] = p_drone[:, 2] - p_tgt[k, 2] - dz_c
        L['dep'][k] = np.degrees(np.arctan2(-d[:, 2], dh)) - DEPRESS_NOM
        L['zerr'][k] = pos_est[:, 2] - p_tgt[k, 2]
        L['xyerr'][k] = np.hypot(*(pos_est[:, :2] - p_tgt[k, :2]).T)
        if log:
            L['drone'][k], L['phi'][k], L['phi_cmd'][k], L['zd'][k] = p_drone, phi, phi_cmd, p_d[:, 2]
            L['yaw'][k] = yaw
            L['est'][k] = pos_est

    inu = np.abs(L['u']) <= FRAME_TOL
    inv = np.abs(L['v']) <= FRAME_TOL
    res = {
        'F2': np.mean(inu & inv, axis=0), 'Fu': np.mean(inu, axis=0), 'Fv': np.mean(inv, axis=0),
        'rms3': np.sqrt(np.mean(L['e3'] ** 2, axis=0)), 'max3': np.max(np.abs(L['e3']), axis=0),
        'mean3': np.mean(L['e3'], axis=0),
        'rmsz': np.sqrt(np.mean(L['ez'] ** 2, axis=0)),
        'rmsdep': np.sqrt(np.mean(L['dep'] ** 2, axis=0)),
        'zrmse': np.sqrt(np.mean(L['zerr'] ** 2, axis=0)),
        'xyrmse': np.sqrt(np.mean(L['xyerr'] ** 2, axis=0)),
        'out': np.mean((np.abs(L['u']) > 1) | (np.abs(L['v']) > 1), axis=0),
        'clip': n_clip / (N_STEPS * S), 'guard': n_guard / (N_STEPS * S),
        'sat_xy': n_sat_xy / (N_STEPS * S), 'sat_z': n_sat_z / (N_STEPS * S),
        'az_peak': az_peak, 'vz_peak': vz_peak, 'axy_peak': axy_peak, 'vxy_peak': vxy_peak,
        'min_dist': min_dist,
    }
    if log:
        res.update({'t': t_all, 'p_tgt': p_tgt, **L})
    return res


def run_simulation():
    noise = draw_noise(range(N_MC))
    results = {s: simulate(CONFIG[s], noise, log=True) for s in STRATS}
    zsweep = {z: simulate(dict(CONFIG['fix'], z_fix=z), noise) for z in Z_FIX_SWEEP}
    azsweep = {a: simulate(CONFIG['sph'], noise, a_z_max=a) for a in A_Z_SWEEP}
    sigsweep = {sz: {s: simulate(CONFIG[s], noise, sig_z=sz) for s in ('sph', 'fix')}
                for sz in SIG_Z_SWEEP}
    nojump = {s: simulate(CONFIG[s], noise, jumps=False) for s in ('sph', 'fix', 'oracle')}
    nolag = {s: simulate(CONFIG[s], noise, tau_g=0.0) for s in ('sph', 'fix', 'oracle')}
    r3sweep = {r3: simulate(dict(CONFIG['sph'], r3=r3, dz=r3 / 2, r_xy=r3 * np.sqrt(3) / 2), noise)
               for r3 in R3_SWEEP}
    # sph and fix differ only in the altitude channel: identical horizontal paths
    assert np.array_equal(results['sph']['drone'][:, :, :2], results['fix']['drone'][:, :, :2])
    # Order check: the CRN noise of seed s must not depend on the batch
    sub = simulate(CONFIG['sph'], draw_noise([7, 3]))
    assert np.array_equal(sub['F2'], results['sph']['F2'][[7, 3]])
    return results, zsweep, azsweep, sigsweep, nojump, nolag, r3sweep


# ── Plotting ──────────────────────────────────────────────────────────────────

def jump_bands(ax):
    for tj in JUMP_TIMES:
        ax.axvspan(tj - T_JUMP / 2, tj + T_JUMP / 2, color='grey', alpha=0.25, lw=0)


def plot_trajectory_3d(results, out_dir, seed=0):
    fig = plt.figure(figsize=(11, 8))
    ax = fig.add_subplot(111, projection='3d')
    p = results['sph']['p_tgt']
    ax.plot(p[:, 0], p[:, 1], p[:, 2], 'b-', lw=2, label='Person (height-varying figure-8)')
    jm = np.zeros(len(p), bool)
    for tj in JUMP_TIMES:
        jm |= np.abs(results['sph']['t'] - tj) < T_JUMP / 2
    ax.scatter(p[jm, 0], p[jm, 1], p[jm, 2], color='grey', s=6, label='Jumps')
    for s, ls, lw in [('sph', '-', 1.4), ('cyl', '--', 1.0), ('fix', ':', 1.2)]:
        d = results[s]['drone'][:, seed]
        ax.plot(d[:, 0], d[:, 1], d[:, 2], color=COLORS[s], ls=ls, lw=lw, label=f'Drone, {LABELS[s]}')
    k = 400
    d = results['sph']['drone'][k, seed]
    ax.plot([d[0], p[k, 0]], [d[1], p[k, 1]], [d[2], p[k, 2]], 'k-', lw=0.8)
    ax.text(*((d + p[k]) / 2), f' r_3D = {R_3D:.0f} m', fontsize=8)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_zlabel('z (m)')
    ax.set_box_aspect((1.6, 1.2, 0.6))
    ax.view_init(elev=22, azim=-60)
    ax.set_title(f'S081 3D Selfie Follow — trajectories (seed {seed})')
    ax.legend(loc='upper left', fontsize=8)
    path = os.path.join(out_dir, 'trajectory_3d.png')
    plt.savefig(path, dpi=140, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_altitude(results, out_dir, seed=0):
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    t = results['sph']['t']
    zt = results['sph']['p_tgt'][:, 2]
    a1.plot(t, zt, 'b-', lw=1.5, label='Person $z_{tgt}$')
    a1.plot(t, zt + DZ_CLEAR, 'b:', lw=0.8, label='$z_{tgt} + 2.5$ m (ideal)')
    for s in ['sph', 'cyl', 'raw', 'fix', 'base']:
        a1.plot(t, results[s]['drone'][:, seed, 2], color=COLORS[s], lw=0.9, label=LABELS[s])
    a1.axhline(Z_MIN, color='k', ls='--', lw=0.7, label='$z_{min}$ = 1.5 m')
    jump_bands(a1)
    a1.set_ylabel('Altitude (m)')
    a1.legend(fontsize=7, ncol=3)
    a1.grid(alpha=0.3)
    a1.set_title(f'Altitude (seed {seed}); grey bands = jumps')
    for s in ['sph', 'raw', 'fix', 'base']:
        a2.plot(t, results[s]['ez'][:, seed], color=COLORS[s], lw=0.8,
                label=f"{LABELS[s]} (RMS {results[s]['rmsz'][seed]:.2f} m)")
    jump_bands(a2)
    a2.axhline(0, color='k', lw=0.6)
    a2.set_ylabel(r'$z_{drone} - z_{tgt} - \Delta z_{clear}$ (m)')
    a2.set_xlabel('Time (s)')
    a2.legend(fontsize=7, ncol=2)
    a2.grid(alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'altitude_timeseries.png')
    plt.savefig(path, dpi=140, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_gimbal(results, out_dir, seed=0):
    fig, ax = plt.subplots(figsize=(12, 4.5))
    r = results['sph']
    t = r['t']
    d = r['drone'][:, seed] - r['p_tgt']
    ideal = np.degrees(np.arctan2(-d[:, 2], np.hypot(d[:, 0], d[:, 1])))
    ax.plot(t, np.degrees(r['phi_cmd'][:, seed]), color='orange', lw=0.8, label=r'$\phi_{cmd}$ (from estimate)')
    ax.plot(t, np.degrees(r['phi'][:, seed]), 'r-', lw=1.2, label=r'$\phi_{gimbal}$ (lag $\tau$ = 0.1 s)')
    ax.plot(t, ideal, 'b--', lw=0.9, label='Pitch to the true person')
    jump_bands(ax)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Gimbal pitch (deg, negative = down)')
    ax.set_title(f'Gimbal pitch, {LABELS["sph"]} (seed {seed})')
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'gimbal_pitch.png')
    plt.savefig(path, dpi=140, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_framing(results, out_dir, seed=0):
    fig, axes = plt.subplots(2, 2, figsize=(13, 7), sharex=True, sharey=True)
    t = results['sph']['t']
    for row, s in enumerate(['sph', 'fix']):
        for col, key in enumerate(['u', 'v']):
            ax = axes[row, col]
            ax.plot(t, results[s][key][:, seed], color=COLORS[s], lw=0.7)
            ax.axhspan(-FRAME_TOL, FRAME_TOL, color='green', alpha=0.15)
            jump_bands(ax)
            ax.set_ylim(-0.6, 0.6)
            ax.grid(alpha=0.3)
            fr = np.mean(np.abs(results[s][key][:, seed]) <= FRAME_TOL)
            ax.set_title(f'{LABELS[s]}: {key} (in tolerance {100 * fr:.1f}% of time)', fontsize=9)
    axes[1, 0].set_xlabel('Time (s)')
    axes[1, 1].set_xlabel('Time (s)')
    axes[0, 0].set_ylabel('offset / half-FOV')
    axes[1, 0].set_ylabel('offset / half-FOV')
    fig.suptitle(f'2-axis framing (seed {seed}); green = ±10% band, grey = jumps')
    plt.tight_layout()
    path = os.path.join(out_dir, 'framing_uv.png')
    plt.savefig(path, dpi=140, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_scores(results, out_dir):
    fig, (a1, a2, a3) = plt.subplots(1, 3, figsize=(16, 5))
    est = ['sph', 'cyl', 'raw', 'fix', 'base']
    x = np.arange(len(est))
    w = 0.27
    for j, (key, lab) in enumerate([('Fu', '$F_u$ (1-axis, as S081)'), ('Fv', '$F_v$'),
                                    ('F2', '$F_{2D}$ (both axes)')]):
        m = [100 * results[s][key].mean() for s in est]
        sd = [100 * results[s][key].std() for s in est]
        a1.bar(x + (j - 1) * w, m, w, yerr=sd, capsize=2, label=lab)
    a1.set_xticks(x, ['3D KF\n+ spherical', '3D KF\n+ cylinder', 'raw GPS\n+ spherical',
                      '2D baseline\nz = 3 m', 'S081 as-is\nz = 4 m'], fontsize=8)
    a1.set_ylim(0, 118)
    a1.set_ylabel('Framing score (%)')
    a1.set_title(f'Framing, mean ± std over {N_MC} seeds')
    a1.legend(fontsize=8, ncol=3, loc='upper center')
    a1.grid(alpha=0.3, axis='y')
    a2.boxplot([results[s]['rms3'] for s in STRATS], showmeans=True)
    a2.set_xticks(range(1, len(STRATS) + 1), [s for s in STRATS])
    a2.set_ylabel('RMS $e_{3D}$ (m)')
    a2.set_title('RMS 3D orbit-distance error')
    a2.grid(alpha=0.3)
    a3.boxplot([results[s]['rmsdep'] for s in STRATS], showmeans=True)
    a3.set_xticks(range(1, len(STRATS) + 1), [s for s in STRATS])
    a3.set_ylabel(f'RMS look-down angle error vs {DEPRESS_NOM:.0f}° (deg)')
    a3.set_title('Shot-angle consistency')
    a3.grid(alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'scores.png')
    plt.savefig(path, dpi=140, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_orbit_error(results, out_dir, seed=0):
    fig, ax = plt.subplots(figsize=(12, 4.5))
    t = results['sph']['t']
    for s in STRATS:
        ax.plot(t, results[s]['e3'][:, seed], color=COLORS[s], lw=0.8,
                ls='--' if s == 'oracle' else '-',
                label=f"{LABELS[s]} (RMS {results[s]['rms3'][seed]:.2f} m)")
    ax.axhline(0, color='k', lw=0.6)
    jump_bands(ax)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel(r'$e_{3D} = \|p_{drone} - p_{tgt}\| - r_{3D}$ (m)')
    ax.set_title(f'3D orbit-distance error (seed {seed})')
    ax.legend(fontsize=7, ncol=3)
    ax.grid(alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'orbit_error.png')
    plt.savefig(path, dpi=140, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_sensitivity(zsweep, sigsweep, r3sweep, out_dir):
    fig, (a1, a2, a3) = plt.subplots(1, 3, figsize=(18, 4.8))
    zs = Z_FIX_SWEEP
    a1.plot(zs, [100 * zsweep[z]['F2'].mean() for z in zs], 'o-', color='purple', label='$F_{2D}$ fixed z')
    a1b = a1.twinx()
    a1b.plot(zs, [zsweep[z]['rms3'].mean() for z in zs], 's--', color='purple', label='RMS $e_{3D}$')
    a1.set_xlabel('2D-baseline altitude (m)')
    a1.set_ylabel('$F_{2D}$ (%)  (circles)')
    a1b.set_ylabel('RMS $e_{3D}$ (m)  (squares)')
    a1.set_title('Fixed-altitude baseline vs its altitude')
    a1.grid(alpha=0.3)
    for s, mk in (('sph', 'o-'), ('fix', 's-')):
        a2.plot(SIG_Z_SWEEP, [100 * sigsweep[sz][s]['F2'].mean() for sz in SIG_Z_SWEEP], marker=mk[0],
                color=COLORS[s], label=f'{LABELS[s]}: $F_{{2D}}$')
        a2.plot(SIG_Z_SWEEP, [100 * sigsweep[sz][s]['Fv'].mean() for sz in SIG_Z_SWEEP], marker=mk[0],
                color=COLORS[s], alpha=0.4, ls=':', label=f'{LABELS[s]}: $F_v$')
    a2.set_xlabel(r'GPS vertical noise $\sigma_z$ (m)')
    a2.set_ylabel('Framing (%)')
    a2.set_title('Framing vs vertical GPS noise')
    a2.legend(fontsize=7)
    a2.grid(alpha=0.3)
    for key, lab, mk in (('F2', '$F_{2D}$', 'o'), ('Fu', '$F_u$', 's'), ('Fv', '$F_v$', '^')):
        a3.plot(R3_SWEEP, [100 * r3sweep[r]['F2' if key == 'F2' else key].mean() for r in R3_SWEEP],
                marker=mk, label=lab)
    a3.set_xlabel(r'Standoff range $r_{3D}$ (m), $\Delta z_{clear} = r_{3D}/2$')
    a3.set_ylabel('Framing (%)')
    a3.set_title('Range probe: 3D KF + spherical PD')
    a3.legend(fontsize=8)
    a3.grid(alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'sensitivity.png')
    plt.savefig(path, dpi=140, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def anim_indices(n, step):
    n_frames = int(np.ceil((n - 1) / step)) + 1
    return [min(i * step, n - 1) for i in range(n_frames)]


def save_animation(results, out_dir, seed=0):
    r = results['sph']
    idx = anim_indices(N_STEPS, ANIM_STEP)
    fig = plt.figure(figsize=(7.5, 6))
    ax = fig.add_subplot(111, projection='3d')
    p = r['p_tgt']
    dr = r['drone'][:, seed]
    ax.plot(p[:, 0], p[:, 1], p[:, 2], 'b-', lw=0.6, alpha=0.3)
    ax.set_xlim(-14, 14)
    ax.set_ylim(-10, 10)
    ax.set_zlim(-1.5, 8)
    ax.set_box_aspect((1.4, 1.0, 0.55))
    ax.view_init(elev=20, azim=-60)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_zlabel('z (m)')
    tgt, = ax.plot([], [], [], 'bo', ms=8, label='Person')
    drn, = ax.plot([], [], [], 'r^', ms=9, label='Drone (3D KF + spherical)')
    trail, = ax.plot([], [], [], 'r-', lw=0.8, alpha=0.6)
    look, = ax.plot([], [], [], 'k-', lw=0.8, label='Camera axis (5 m)')
    ax.legend(loc='upper left', fontsize=8)

    def update(i):
        k = idx[i]
        tgt.set_data([p[k, 0]], [p[k, 1]])
        tgt.set_3d_properties([p[k, 2]])
        drn.set_data([dr[k, 0]], [dr[k, 1]])
        drn.set_3d_properties([dr[k, 2]])
        k0 = max(0, k - 60)
        trail.set_data(dr[k0:k + 1, 0], dr[k0:k + 1, 1])
        trail.set_3d_properties(dr[k0:k + 1, 2])
        ph, yaw = r['phi'][k, seed], r['yaw'][k, seed]
        L5 = 5.0
        end = dr[k] + L5 * np.array([np.cos(ph) * np.cos(yaw), np.cos(ph) * np.sin(yaw), np.sin(ph)])
        look.set_data([dr[k, 0], end[0]], [dr[k, 1], end[1]])
        look.set_3d_properties([dr[k, 2], end[2]])
        inf = abs(r['u'][k, seed]) <= FRAME_TOL and abs(r['v'][k, seed]) <= FRAME_TOL
        ax.set_title(f't = {r["t"][k]:.1f} s  z_person = {p[k, 2]:.2f} m  z_drone = {dr[k, 2]:.2f} m\n'
                     f'u = {r["u"][k, seed]:+.2f}  v = {r["v"][k, seed]:+.2f}  '
                     f'[{"IN FRAME" if inf else "off-centre"}]', fontsize=9)
        return tgt, drn, trail, look

    ani = animation.FuncAnimation(fig, update, frames=len(idx), blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer='pillow', fps=10, dpi=72)
    plt.close(fig)
    print(f'Saved: {path} ({len(idx)} frames)')


# ── Main ──────────────────────────────────────────────────────────────────────

def ms(a, scale=1.0, f='.1f'):
    return f'{scale * a.mean():{f}} ± {scale * a.std():{f}}'


if __name__ == '__main__':
    results, zsweep, azsweep, sigsweep, nojump, nolag, r3sweep = run_simulation()
    _, pt, vt = person()
    print('=== S081 3D Selfie Follow ===')
    print(f'  Person z range {pt[:, 2].min():.2f} .. {pt[:, 2].max():.2f} m, peak |vz| '
          f'{np.abs(vt[:, 2]).max():.2f} m/s; {len(JUMP_TIMES)} jumps at t = '
          f'{JUMP_TIMES[0]:.2f} + 7.5 k s')
    print(f'  r_xy = sqrt({R_3D:.0f}^2 - {DZ_CLEAR}^2) = {R_XY:.3f} m; design look-down angle '
          f'{DEPRESS_NOM:.1f} deg; cylinder geometric offset sqrt({R_3D:.0f}^2 + {DZ_CLEAR}^2) - '
          f'{R_3D:.0f} = {np.hypot(R_3D, DZ_CLEAR) - R_3D:.3f} m')
    print(f'  Framing tolerance: 0.1 x 30 deg = 3.0 deg (u), 0.1 x 22.5 deg = 2.25 deg (v); '
          f'at 5 m range the v tolerance is {R_3D * np.tan(np.radians(2.25)):.3f} m')
    print(f'  Monte Carlo: {N_MC} seeds, noise of seed s from default_rng({SEED} + s), same for '
          f'every strategy')
    hdr = (f"  {'Strategy':<34}{'F_2D (%)':>14}{'F_u (%)':>14}{'F_v (%)':>14}{'RMS e_3D (m)':>16}"
           f"{'Max e_3D (m)':>15}{'RMS dz err (m)':>16}{'RMS look-down err (deg)':>25}")
    print(hdr)
    for s in STRATS:
        r = results[s]
        print(f"  {LABELS[s]:<34}{ms(r['F2'], 100):>14}{ms(r['Fu'], 100):>14}{ms(r['Fv'], 100):>14}"
              f"{ms(r['rms3'], f='.3f'):>16}{ms(r['max3'], f='.2f'):>15}{ms(r['rmsz'], f='.3f'):>16}"
              f"{ms(r['rmsdep'], f='.2f'):>25}")
    print('  Estimates: xy RMSE / z RMSE (m), mean e_3D (m), person out of image (%)')
    for s in STRATS:
        r = results[s]
        print(f"    {LABELS[s]:<34} {r['xyrmse'].mean():.3f} / {r['zrmse'].mean():.3f}   "
              f"{r['mean3'].mean():+.3f}   {100 * r['out'].mean():.2f}")
    # X5 check: the 'base' strategy reproduces the 2D script's KF estimate and drone path
    noise = draw_noise(range(N_MC))
    r2d = m2d.simulate('kf', noise[:, :, :2], log=True)
    de = np.abs(results['base']['est'][:, :, :2] - r2d['est']).max()
    dd = np.abs(results['base']['drone'][:, :, :2] - r2d['drone']).max()
    print(f'  S081 2D script (KF) on the same xy noise vs "S081 as-is": max |estimate diff| {de:.1e} m, '
          f'max |drone xy diff| {dd:.1e} m; sph and fix horizontal paths identical (asserted)')
    assert de < 1e-6 and dd < 1e-6
    print('  Seed 0 (figures):')
    for s in STRATS:
        r = results[s]
        print(f"    {LABELS[s]:<34} F_2D {100 * r['F2'][0]:5.1f}%  F_u {100 * r['Fu'][0]:5.1f}%  "
              f"F_v {100 * r['Fv'][0]:5.1f}%  RMS e_3D {r['rms3'][0]:.3f} m  RMS dz {r['rmsz'][0]:.3f} m")
    print('  Per-seed comparison against the 3D nominal (sph):')
    for b in ['cyl', 'raw', 'fix', 'base']:
        a, c = results['sph'], results[b]
        w2 = int(np.sum(a['F2'] > c['F2'])); t2 = int(np.sum(a['F2'] == c['F2']))
        wr = int(np.sum(a['rms3'] < c['rms3']))
        wd = int(np.sum(a['rmsdep'] < c['rmsdep']))
        print(f"    sph vs {b:<5}: higher F_2D in {w2}/{N_MC} (ties {t2}), dF_2D = "
              f"{100 * (a['F2'] - c['F2']).mean():+.1f} pp; lower RMS e_3D in {wr}/{N_MC} "
              f"(diff {(a['rms3'] - c['rms3']).mean():+.3f} m); lower look-down error in {wd}/{N_MC}")
    print(f"  F_2D / F_u (cost of the vertical requirement): " + ', '.join(
        f"{s} {results[s]['F2'].mean() / results[s]['Fu'].mean():.3f}" for s in STRATS if s != 'oracle'))
    print('  Physical limits (all seeds): peak |a_xy| / |a_z| (m/s^2), peak |v_xy| / |v_z| (m/s), '
          'time at a_xy / a_z limit, z_d clipped, floor/ceiling guard active, min 3D distance')
    for s in STRATS:
        r = results[s]
        print(f"    {LABELS[s]:<34} {r['axy_peak']:.2f} / {r['az_peak']:.2f}, "
              f"{r['vxy_peak']:.2f} / {r['vz_peak']:.2f}, {100 * r['sat_xy']:.2f}% / "
              f"{100 * r['sat_z']:.2f}%, {100 * r['clip']:.2f}%, {100 * r['guard']:.2f}%, "
              f"{r['min_dist']:.2f} m")
    print(f'  2D-baseline altitude sweep (fixed z, r_xy = {R_XY:.2f} m, 3D KF tilt):')
    for z in Z_FIX_SWEEP:
        r = zsweep[z]
        print(f"    z = {z:.1f} m: F_2D {ms(r['F2'], 100)}%, F_u {100 * r['Fu'].mean():.1f}%, "
              f"F_v {100 * r['Fv'].mean():.1f}%, RMS e_3D {r['rms3'].mean():.3f} m (mean {r['mean3'].mean():+.3f}), RMS look-down err "
              f"{r['rmsdep'].mean():.2f} deg, min 3D dist {r['min_dist']:.2f} m; sph higher F_2D in "
              f"{int(np.sum(results['sph']['F2'] > r['F2']))}/{N_MC}")
    print('  Vertical acceleration-limit sweep (sph):')
    for a in A_Z_SWEEP:
        r = azsweep[a]
        lab = f'{a:.0f} m/s^2' if np.isfinite(a) else 'unlimited'
        print(f"    a_z_max = {lab:>10}: F_2D {100 * r['F2'].mean():.1f}%, F_v {100 * r['Fv'].mean():.1f}%, "
              f"RMS e_3D {r['rms3'].mean():.3f} m, RMS dz {r['rmsz'].mean():.3f} m, at limit "
              f"{100 * r['sat_z']:.2f}%")
    print('  Vertical GPS-noise sweep (sph vs fix, F_2D / F_v / RMS e_3D):')
    for sz in SIG_Z_SWEEP:
        a, c = sigsweep[sz]['sph'], sigsweep[sz]['fix']
        print(f"    sigma_z = {sz:.2f} m: sph {100 * a['F2'].mean():.1f}% / {100 * a['Fv'].mean():.1f}% / "
              f"{a['rms3'].mean():.3f} m;  fix {100 * c['F2'].mean():.1f}% / {100 * c['Fv'].mean():.1f}% / "
              f"{c['rms3'].mean():.3f} m;  dF_2D (sph - fix) {100 * (a['F2'] - c['F2']).mean():+.1f} pp, "
              f"sph higher in {int(np.sum(a['F2'] > c['F2']))}/{N_MC}")
    print('  Ablation — no jumps (F_2D / F_v / RMS e_3D / RMS dz):')
    for s in ('sph', 'fix', 'oracle'):
        a, b = results[s], nojump[s]
        print(f"    {LABELS[s]:<34} with jumps {100 * a['F2'].mean():.1f}% / {100 * a['Fv'].mean():.1f}% / "
              f"{a['rms3'].mean():.3f} / {a['rmsz'].mean():.3f};  without {100 * b['F2'].mean():.1f}% / "
              f"{100 * b['Fv'].mean():.1f}% / {b['rms3'].mean():.3f} / {b['rmsz'].mean():.3f}")
    for s in ('sph', 'fix'):
        d = nojump[s]['F2'] - results[s]['F2']
        print(f"    {LABELS[s]}: removing the jumps changes F_2D by {100 * d.mean():+.1f} pp "
              f"(higher without jumps in {int(np.sum(d > 0))}/{N_MC})")
    print('  Ablation — gimbal lag removed (tau = 0), F_v:')
    for s in ('sph', 'fix', 'oracle'):
        print(f"    {LABELS[s]:<34} tau = 0.1 s {100 * results[s]['Fv'].mean():.1f}%  ->  tau = 0 "
              f"{100 * nolag[s]['Fv'].mean():.1f}%  (F_2D {100 * results[s]['F2'].mean():.1f}% -> "
              f"{100 * nolag[s]['F2'].mean():.1f}%)")
    for name, R in (('tau = 0.1 s', results), ('tau = 0', nolag)):
        g = R['fix']['Fv'] - R['sph']['Fv']
        print(f"    F_v gap fix - sph with {name}: {100 * g.mean():+.1f} pp (fix higher in "
              f"{int(np.sum(g > 0))}/{N_MC})")
    print('  Range probe (sph, dz = r_3D/2, r_xy = r_3D cos 30 deg):')
    for r3 in R3_SWEEP:
        r = r3sweep[r3]
        print(f"    r_3D = {r3:.0f} m: F_2D {100 * r['F2'].mean():.1f}%, F_u {100 * r['Fu'].mean():.1f}%, "
              f"F_v {100 * r['Fv'].mean():.1f}%, RMS e_3D {r['rms3'].mean():.3f} m")

    os.makedirs(OUTPUT_DIR, exist_ok=True)
    plot_trajectory_3d(results, OUTPUT_DIR)
    plot_altitude(results, OUTPUT_DIR)
    plot_gimbal(results, OUTPUT_DIR)
    plot_framing(results, OUTPUT_DIR)
    plot_scores(results, OUTPUT_DIR)
    plot_orbit_error(results, OUTPUT_DIR)
    plot_sensitivity(zsweep, sigsweep, r3sweep, OUTPUT_DIR)
    save_animation(results, OUTPUT_DIR)
