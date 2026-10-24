"""
S086 3D Upgrade — Multi-Angle Cinematography
============================================
The S086 three-drone filming formation (subject on a figure-8, a = 10 m, b = 6 m, RMS speed
1.5 m/s; shared constant-velocity KF on 5 Hz fixes, sigma = 0.3 m; proportional velocity
command Kp = 2, |u| <= 4 m/s; formation rotated with the subject's heading, drones 120 deg apart)
is lifted onto a sphere: drone k sits at radius r = 6 m from the subject at its own elevation
theta_k (20, 45, 70 deg), so altitude becomes a planned state of the formation.  Each camera has
a 2-DOF gimbal (pitch + yaw, first-order lag 0.1 s) aimed at the KF estimate, and an elevation
repulsion law keeps pairs of cameras at least psi_min = 30 deg apart as seen from the subject.

Formations (same observation noise per seed for every formation — common random numbers):
  hemi        sphere r = 6 m, theta = 20/45/70 deg, heading-locked, 6-state KF        (card)
  flat        S086 as built: all drones at z = 4 m, horizontal radius 6 m             (2D baseline, X5)
  hemi_world  hemisphere, world-fixed azimuths (S086's added baseline)                (added)
  flat_world  flat, world-fixed azimuths                                              (added)
  hemi_oracle hemisphere, true subject state (controller-only error)                  (added)
  stress      theta = 60/70/80 deg, card repulsion  /  stress_norep: repulsion off /
  stress_sym  symmetric repulsion (both drones of a pair move apart)                  (added probes)

Usage:
    conda activate drones
    python src/05_special_entertainment/3d/s086_3d_multi_angle_cinema.py
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
    's086_2d', os.path.join(_HERE, '..', 's086_multi_angle_cinema.py'))
m2d = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(m2d)

OUTPUT_DIR = os.path.normpath(os.path.join(
    _HERE, '..', '..', '..', 'outputs', '05_special_entertainment', '3d', 's086_3d_multi_angle_cinema'))

# ── Parameters (card; S086 values reused) ─────────────────────────────────────
N_DRONES   = 3
R_SPHERE   = 6.0                         # m
THETA_CARD = np.radians([20.0, 45.0, 70.0])
FLY_ALT    = m2d.FLY_ALT                 # 4 m (2D baseline)
R_FLAT     = m2d.ORBIT_RADIUS            # 6 m horizontal (2D baseline)
Z_MIN, Z_MAX = 0.5, 8.0
PSI_MIN    = np.radians(30.0)
K_REP      = 0.5
TH_CLAMP   = np.radians([10.0, 80.0])
D_MIN      = m2d.D_MIN                   # 3 m
PHASE_TOL  = np.radians(5.0)
ELEV_TOL   = np.radians(5.0)
FRAME_THR  = 0.15
TAU_G      = 0.1                         # s
FOV_HALF   = np.radians(30.0)            # card 3D (S086 used atan(2/6) = 18.4 deg)
SIGMA_A    = m2d.SIGMA_A                 # 0.8 m/s^2
SIGMA_OBS  = m2d.SIGMA_OBS               # 0.3 m
KP, V_MAX  = m2d.KP, m2d.V_MAX           # 2.0 1/s, 4 m/s
DT, T_SIM  = m2d.DT, m2d.T_SIM           # 0.05 s, 60 s
N_STEPS    = m2d.N_STEPS
OBS_EVERY  = m2d.OBS_EVERY               # 4 steps (0.2 s)
N_OBS      = N_STEPS // OBS_EVERY + 1

# ── Parameters (added) ────────────────────────────────────────────────────────
N_MC       = 100
SEED       = 8600                        # noise of seed s from default_rng(SEED + s)
T_LATE     = m2d.T_LATE                  # 5 s: metrics after the KF start-up
THETA_STRESS = np.radians([60.0, 70.0, 80.0])
TH2_SWEEP  = [50.0, 60.0, 70.0, 80.0]    # deg, elevation of the high drone
COV_EVERY  = 10                          # steps between hemisphere-coverage evaluations
N_DIR      = 2000                        # Fibonacci directions on the upper hemisphere
ANIM_STEP  = 10                          # 0.5 s per frame (GIF < 3 MB)

THETA_FLAT = np.arctan2(FLY_ALT, R_FLAT)             # 33.7 deg
R_FLAT_3D  = np.hypot(FLY_ALT, R_FLAT)               # 7.21 m
D_COLORS   = ['tab:red', 'tab:orange', 'tab:purple']
PAIRS      = [(0, 1), (0, 2), (1, 2)]

CONFIGS = {
    'hemi':        dict(geom='sphere', theta=THETA_CARD, lock=True, est='kf', rep='card'),
    'flat':        dict(geom='flat', lock=True, est='kf', rep='none'),
    'flat_r6':     dict(geom='flat', lock=True, est='kf', rep='none', r_h=R_SPHERE * np.cos(np.arctan2(FLY_ALT, R_FLAT)),
                        alt=R_SPHERE * np.sin(np.arctan2(FLY_ALT, R_FLAT))),
    'hemi_world':  dict(geom='sphere', theta=THETA_CARD, lock=False, est='kf', rep='card'),
    'flat_world':  dict(geom='flat', lock=False, est='kf', rep='none'),
    'hemi_oracle': dict(geom='sphere', theta=THETA_CARD, lock=True, est='oracle', rep='card'),
    'stress':      dict(geom='sphere', theta=THETA_STRESS, lock=True, est='kf', rep='card'),
    'stress_norep': dict(geom='sphere', theta=THETA_STRESS, lock=True, est='kf', rep='none'),
    'stress_sym':  dict(geom='sphere', theta=THETA_STRESS, lock=True, est='kf', rep='sym'),
}
LABELS = {'hemi': 'Hemisphere 20/45/70°, heading-locked', 'flat': '2D baseline: flat z = 4 m',
          'flat_r6': 'Flat at 6 m range (z = 3.33 m)', 'hemi_world': 'Hemisphere, world-fixed', 'flat_world': 'Flat, world-fixed',
          'hemi_oracle': 'Hemisphere, oracle state', 'stress': 'Stress 60/70/80°, card repulsion',
          'stress_norep': 'Stress, no repulsion', 'stress_sym': 'Stress, symmetric repulsion'}
MAIN = ['hemi', 'flat', 'hemi_world', 'flat_world', 'hemi_oracle']


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def subject(t):
    pxy, vxy = m2d.subject(t)                 # corrected omega (RMS 1.5 m/s)
    n = len(pxy)
    return np.column_stack([pxy, np.zeros(n)]), np.column_stack([vxy, np.zeros(n)])


def kf_matrices():
    F = np.eye(6)
    F[:3, 3:] = DT * np.eye(3)
    H = np.hstack([np.eye(3), np.zeros((3, 3))])
    d4, d3, d2 = DT ** 4 / 4, DT ** 3 / 2, DT ** 2
    Q = SIGMA_A ** 2 * np.block([[d4 * np.eye(3), d3 * np.eye(3)], [d3 * np.eye(3), d2 * np.eye(3)]])
    return F, H, Q, SIGMA_OBS ** 2 * np.eye(3)


def draw_noise(seeds):
    return np.stack([np.random.default_rng(SEED + s).standard_normal((N_OBS, 3)) for s in seeds])


def hemisphere_dirs(n=N_DIR):
    i = np.arange(n) + 0.5
    z = i / n                                  # uniform in z on the upper hemisphere (equal area)
    ph = np.pi * (1 + 5 ** 0.5) * i
    rr = np.sqrt(1 - z ** 2)
    return np.column_stack([rr * np.cos(ph), rr * np.sin(ph), z])


DIRS = hemisphere_dirs()


def static_coverage(theta):
    az = 2 * np.pi * np.arange(N_DRONES) / N_DRONES
    u = np.column_stack([np.cos(theta) * np.cos(az), np.cos(theta) * np.sin(az), np.sin(theta)])
    ang = np.degrees(np.arccos(np.clip(DIRS @ u.T, -1, 1))).min(axis=1)
    return ang.mean(), ang.max()


def ideal_geometry(theta):
    az = 2 * np.pi * np.arange(N_DRONES) / N_DRONES
    u = np.column_stack([np.cos(theta) * np.cos(az), np.cos(theta) * np.sin(az), np.sin(theta)])
    out = []
    for i, j in PAIRS:
        out.append((np.degrees(np.arccos(np.clip(u[i] @ u[j], -1, 1))),
                    R_SPHERE * np.linalg.norm(u[i] - u[j])))
    return out


def card_distance(ti, tj):
    """Card formula for the ideal 3D distance of two drones 120 deg apart in azimuth."""
    return R_SPHERE * np.sqrt(np.cos(ti) ** 2 + np.cos(tj) ** 2
                              - 2 * np.cos(ti) * np.cos(tj) * np.cos(2 * np.pi / 3)
                              + (np.sin(ti) - np.sin(tj)) ** 2)


# ── Simulation ────────────────────────────────────────────────────────────────

def simulate(cfg, noise, log=False):
    S = noise.shape[0]
    t_all = np.arange(N_STEPS) * DT
    q, qv = subject(t_all)
    F, H, Q, R = kf_matrices()
    x = np.zeros((S, 6))
    x[:, :3] = q[0]                            # card: velocity starts at 0
    P = np.eye(6)
    offs = 2 * np.pi * np.arange(N_DRONES) / N_DRONES
    sphere = cfg['geom'] == 'sphere'
    r_flat, z_flat = cfg.get('r_h', R_FLAT), cfg.get('alt', FLY_ALT)
    theta_nom = cfg['theta'] if sphere else np.full(N_DRONES, np.arctan2(z_flat, r_flat))

    def setpoints(pe, base, theta):
        ph = base[:, None] + offs[None, :]
        if sphere:
            rh, dz = R_SPHERE * np.cos(theta), R_SPHERE * np.sin(theta)       # (S, N)
        else:
            rh, dz = np.full((S, N_DRONES), r_flat), None
        sp = np.zeros((S, N_DRONES, 3))
        sp[:, :, 0] = pe[:, None, 0] + rh * np.cos(ph)
        sp[:, :, 1] = pe[:, None, 1] + rh * np.sin(ph)
        sp[:, :, 2] = pe[:, None, 2] + dz if sphere else z_flat
        sp[:, :, 2] = np.clip(sp[:, :, 2], Z_MIN, Z_MAX)
        return sp, ph

    pos_est, vel_est = np.tile(q[0], (S, 1)), np.zeros((S, 3))
    theta = np.tile(theta_nom, (S, 1))
    drones, _ = setpoints(pos_est, np.zeros(S), theta)
    alpha_g = beta_g = None
    a_g = 1.0 - np.exp(-DT / TAU_G)

    L = {k: np.zeros((N_STEPS, S, N_DRONES)) for k in ('phase', 'elev', 'point', 'img', 'pitch')}
    L['psi'] = np.zeros((N_STEPS, S, 3))
    L['dist'] = np.zeros((N_STEPS, S, 3))
    L['est'] = np.zeros((N_STEPS, S))
    L['rep'] = np.zeros((N_STEPS, S), dtype=bool)
    cov_mean, cov_max = [], []
    if log:
        L['hist'] = np.zeros((N_STEPS, S, N_DRONES, 3))
        L['alpha'] = np.zeros((N_STEPS, S, N_DRONES))
        L['beta'] = np.zeros((N_STEPS, S, N_DRONES))
        L['elev_deg'] = np.zeros((N_STEPS, S, N_DRONES))
    for k in range(N_STEPS):
        obs = (k % OBS_EVERY == OBS_EVERY - 1)
        if cfg['est'] == 'kf':
            x = x @ F.T
            P = F @ P @ F.T + Q
            if obs:
                z = q[k] + SIGMA_OBS * noise[:, k // OBS_EVERY]
                K = P @ H.T @ np.linalg.inv(H @ P @ H.T + R)
                x = x + (z - x[:, :3]) @ K.T
                P = (np.eye(6) - K @ H) @ P
            pos_est, vel_est = x[:, :3], x[:, 3:]
        else:
            pos_est, vel_est = np.tile(q[k], (S, 1)), np.tile(qv[k], (S, 1))

        # Occlusion repulsion on the target elevations (from the current drone positions)
        theta = np.tile(theta_nom, (S, 1))
        if sphere and cfg['rep'] != 'none':
            rel = drones - pos_est[:, None, :]
            un = rel / np.linalg.norm(rel, axis=2, keepdims=True)
            act = np.zeros(S, dtype=bool)
            for i, j in PAIRS:
                psi = np.arccos(np.clip(np.sum(un[:, i] * un[:, j], axis=1), -1, 1))
                viol = psi < PSI_MIN
                act |= viol
                d = K_REP * (PSI_MIN - psi) * np.sign(theta_nom[j] - theta_nom[i])
                theta[:, j] = np.where(viol, theta[:, j] + d, theta[:, j])
                if cfg['rep'] == 'sym':
                    theta[:, i] = np.where(viol, theta[:, i] - d, theta[:, i])
            theta = np.clip(theta, *TH_CLAMP)
            L['rep'][k] = act

        base = np.arctan2(vel_est[:, 1], vel_est[:, 0]) if cfg['lock'] else np.zeros(S)
        sp, ph_des = setpoints(pos_est, base, theta)
        u = KP * (sp - drones)
        n = np.linalg.norm(u, axis=2, keepdims=True)
        u = np.where(n > V_MAX, u / np.maximum(n, 1e-12) * V_MAX, u)
        new = drones + u * DT
        assert np.all(np.linalg.norm(u, axis=2) <= V_MAX + 1e-9)
        assert np.all(new[:, :, 2] >= Z_MIN - 1e-9) and np.all(new[:, :, 2] <= Z_MAX + 1e-9)
        drones = new

        # Gimbal: commanded yaw / pitch at the estimate, first-order servo lag
        dl = pos_est[:, None, :] - drones
        b_cmd = np.arctan2(dl[..., 1], dl[..., 0])
        a_cmd = np.arctan2(dl[..., 2], np.hypot(dl[..., 0], dl[..., 1]))   # negative = down
        if alpha_g is None:
            alpha_g, beta_g = a_cmd.copy(), b_cmd.copy()
        else:
            alpha_g = alpha_g + a_g * (a_cmd - alpha_g)
            beta_g = wrap(beta_g + a_g * wrap(b_cmd - beta_g))
        c = np.stack([np.cos(alpha_g) * np.cos(beta_g), np.cos(alpha_g) * np.sin(beta_g),
                      np.sin(alpha_g)], axis=-1)
        dtrue = q[k][None, None, :] - drones
        dn = np.linalg.norm(dtrue, axis=2)
        eps = np.arccos(np.clip(np.sum(c * dtrue, axis=2) / dn, -1, 1))
        L['point'][k] = eps
        L['pitch'][k] = alpha_g
        L['img'][k] = np.tan(np.minimum(eps, np.radians(89.0))) / np.tan(FOV_HALF)

        # Geometry relative to the true subject
        rt = drones - q[k][None, None, :]
        ut = rt / np.linalg.norm(rt, axis=2, keepdims=True)
        L['elev'][k] = np.abs(np.arcsin(ut[..., 2]) - theta_nom[None, :])
        rel_e = drones[:, :, :2] - pos_est[:, None, :2]
        L['phase'][k] = np.abs(wrap(np.arctan2(rel_e[..., 1], rel_e[..., 0]) - ph_des))
        for j, (a, b) in enumerate(PAIRS):
            L['psi'][k, :, j] = np.arccos(np.clip(np.sum(ut[:, a] * ut[:, b], axis=1), -1, 1))
            L['dist'][k, :, j] = np.linalg.norm(drones[:, a] - drones[:, b], axis=1)
        L['est'][k] = np.linalg.norm(pos_est - q[k], axis=1)
        if k % COV_EVERY == 0:
            ang = np.arccos(np.clip(np.einsum('snc,dc->snd', ut, DIRS), -1, 1)).min(axis=1)  # (S, D)
            cov_mean.append(ang.mean(axis=1))
            cov_max.append(ang.max(axis=1))
        if log:
            L['hist'][k], L['alpha'][k], L['beta'][k] = drones, alpha_g, beta_g
            L['elev_deg'][k] = np.degrees(np.arcsin(ut[..., 2]))

    late = t_all >= T_LATE
    deg = np.degrees
    res = {
        'phase_mean': deg(L['phase'][late].mean(axis=0)),           # (S, N) after 5 s
        'phase_ok': 100 * np.mean(L['phase'][late] <= PHASE_TOL, axis=(0, 2)),
        'elev_mean': deg(L['elev'][late].mean(axis=0)),             # (S, N)
        'elev_ok': 100 * np.mean(L['elev'][late] <= ELEV_TOL, axis=(0, 2)),
        'point_mean': deg(L['point'][late].mean(axis=(0, 2))),
        'pitch_mean': deg(L['pitch'][late].mean(axis=0)),           # (S, N) gimbal pitch after 5 s
        'frame': np.mean(L['img'] <= FRAME_THR, axis=(0, 2)),
        'frame_s086fov': np.mean(L['img'] * np.tan(FOV_HALF) / (m2d.FOV_HALF_WIDTH / m2d.ORBIT_RADIUS)
                                 <= FRAME_THR, axis=(0, 2)),
        'psi_min': deg(L['psi'].min(axis=(0, 2))),
        'psi_min_late': deg(L['psi'][late].min(axis=(0, 2))),
        'psi_viol': 100 * np.mean(L['psi'][late] < PSI_MIN, axis=(0, 2)),
        'psi_pair_mean': deg(L['psi'][late].mean(axis=0)),          # (S, 3)
        'min_dist': L['dist'].min(axis=(0, 2)),
        'min_dist_late': L['dist'][late].min(axis=(0, 2)),
        'dist_pair_mean': L['dist'][late].mean(axis=0),
        'rep_frac': 100 * L['rep'].mean(axis=0),
        'kf_rmse': np.sqrt(np.mean(L['est'] ** 2, axis=0)),
        'cov_mean': deg(np.mean(cov_mean, axis=0)), 'cov_max': deg(np.mean(cov_max, axis=0)),
    }
    if log:
        res.update({'t': t_all, 'q': q, **{k: L[k] for k in ('hist', 'alpha', 'beta', 'elev_deg',
                                                               'psi', 'dist', 'phase')}})
    return res


def run_simulation():
    noise = draw_noise(range(N_MC))
    res = {k: simulate(c, noise, log=(k in ('hemi', 'flat'))) for k, c in CONFIGS.items()}
    res = dict(res)
    sweep = {}
    for th2 in TH2_SWEEP:
        th = np.radians([20.0, 45.0, th2])
        sweep[th2] = simulate(dict(CONFIGS['hemi'], theta=th), noise)
    sweep_or = {th2: simulate(dict(CONFIGS['hemi_oracle'], theta=np.radians([20.0, 45.0, th2])),
                              noise[:1]) for th2 in TH2_SWEEP}
    res['stress_diff'] = {k: float(np.abs(res['stress'][k] - res['stress_norep'][k]).max())
                          for k in ('phase_mean', 'elev_mean', 'frame', 'psi_pair_mean', 'psi_min_late', 'min_dist')}
    sub = simulate(CONFIGS['hemi'], draw_noise([9, 4]))
    assert np.array_equal(sub['frame'], res['hemi']['frame'][[9, 4]])
    return res, sweep, sweep_or


# ── Plotting ──────────────────────────────────────────────────────────────────

def plot_hemisphere(res, out_dir, seed=0):
    fig = plt.figure(figsize=(14, 6.5))
    for i, key in enumerate(['hemi', 'flat']):
        r = res[key]
        ax = fig.add_subplot(1, 2, i + 1, projection='3d')
        q = r['q']
        ax.plot(q[:, 0], q[:, 1], q[:, 2], color='blue', lw=1.5, alpha=0.6, label='Subject path')
        ax.scatter(*q[-1], color='blue', s=80, label='Subject (final)')
        for k in range(N_DRONES):
            h = r['hist'][:, seed, k]
            el = r['elev_deg'][-1, seed, k]
            ax.plot(h[:, 0], h[:, 1], h[:, 2], color=D_COLORS[k], lw=0.9, alpha=0.75,
                    label=f'Drone {k}')
            ax.scatter(*h[-1], color=D_COLORS[k], s=80, marker='^')
            ax.plot([h[-1, 0], q[-1, 0]], [h[-1, 1], q[-1, 1]], [h[-1, 2], 0], ':', color=D_COLORS[k])
            ax.text(*h[-1], f' {el:.0f}°', fontsize=8, color=D_COLORS[k])
        if key == 'hemi':
            u = np.linspace(0, 2 * np.pi, 60)
            for th in np.radians([20, 45, 70]):
                ax.plot(q[-1, 0] + R_SPHERE * np.cos(th) * np.cos(u),
                        q[-1, 1] + R_SPHERE * np.cos(th) * np.sin(u),
                        np.full_like(u, R_SPHERE * np.sin(th)), color='grey', ls='--', lw=0.6)
        ax.set_xlim(-17, 17)
        ax.set_ylim(-13, 13)
        ax.set_zlim(0, 8)
        ax.set_box_aspect((1.6, 1.25, 0.6))
        ax.set_xlabel('x (m)')
        ax.set_ylabel('y (m)')
        ax.set_zlabel('z (m)')
        ax.view_init(elev=24, azim=-60)
        ax.set_title(f'{LABELS[key]} (seed {seed})', fontsize=10)
        ax.legend(fontsize=7, loc='upper left')
    fig.suptitle('S086 3D: hemisphere formation vs the flat S086 formation; final elevations annotated, '
                 'dashed = elevation circles on the r = 6 m sphere')
    plt.tight_layout()
    path = os.path.join(out_dir, 'hemisphere.png')
    plt.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_altitude_elevation(res, out_dir, seed=0):
    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    r = res['hemi']
    t = r['t']
    for k in range(N_DRONES):
        axes[0].plot(t, r['hist'][:, seed, k, 2], color=D_COLORS[k], lw=1, label=f'Drone {k}')
        th = np.degrees(THETA_CARD[k])
        axes[1].plot(t, r['elev_deg'][:, seed, k], color=D_COLORS[k], lw=1, label=f'Drone {k} (target {th:.0f}°)')
        axes[1].axhspan(th - 5, th + 5, color=D_COLORS[k], alpha=0.12)
        axes[1].axhline(th, color=D_COLORS[k], ls='--', lw=0.7)
        axes[2].plot(t, np.degrees(r['phase'][:, seed, k]), color=D_COLORS[k], lw=0.9,
                     label=f'Drone {k} (horizontal radius {R_SPHERE * np.cos(THETA_CARD[k]):.2f} m)')
    axes[0].axhline(FLY_ALT, color='grey', ls=':', label='S086 flat altitude 4 m')
    axes[0].set_ylabel('Altitude (m)')
    axes[0].legend(fontsize=8, ncol=4)
    axes[1].set_ylabel('Elevation seen from subject (deg)')
    axes[1].legend(fontsize=8, ncol=3)
    axes[2].axhline(5, color='k', ls='--', lw=0.8, label='5° tolerance')
    axes[2].set_ylabel('Azimuth phase error (deg)')
    axes[2].set_ylim(0, 60)
    axes[2].set_xlabel('Time (s)')
    axes[2].legend(fontsize=8)
    for ax in axes:
        ax.grid(alpha=0.3)
    axes[0].set_title(f'Hemisphere formation, heading-locked KF (seed {seed}); bands = ±5° elevation tolerance')
    plt.tight_layout()
    path = os.path.join(out_dir, 'altitude_elevation.png')
    plt.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_gimbal(res, out_dir, seed=0):
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    r = res['hemi']
    t = r['t']
    for k in range(N_DRONES):
        a1.plot(t, np.degrees(r['alpha'][:, seed, k]), color=D_COLORS[k], lw=1,
                label=f'Drone {k} pitch (target elev. {np.degrees(THETA_CARD[k]):.0f}°)')
        a2.plot(t, np.degrees(np.unwrap(r['beta'][:, seed, k])), color=D_COLORS[k], lw=1,
                label=f'Drone {k} yaw')
    a1.set_ylabel('Gimbal pitch (deg, negative = down)')
    a2.set_ylabel('Gimbal yaw (deg, unwrapped)')
    a2.set_xlabel('Time (s)')
    a1.legend(fontsize=8)
    a2.legend(fontsize=8)
    a1.grid(alpha=0.3)
    a2.grid(alpha=0.3)
    a1.set_title(f'2-DOF gimbal angles, hemisphere formation (seed {seed})')
    plt.tight_layout()
    path = os.path.join(out_dir, 'gimbal_angles.png')
    plt.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_occlusion_distance(res, out_dir, seed=0):
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(12, 7.5), sharex=True)
    t = res['hemi']['t']
    for j, (a, b) in enumerate(PAIRS):
        a1.plot(t, np.degrees(res['hemi']['psi'][:, seed, j]), color=D_COLORS[j], lw=1,
                label=f'Hemisphere pair ({a},{b})')
        a1.plot(t, np.degrees(res['flat']['psi'][:, seed, j]), color=D_COLORS[j], lw=0.8, ls=':',
                label=f'Flat pair ({a},{b})')
        a2.plot(t, res['hemi']['dist'][:, seed, j], color=D_COLORS[j], lw=1, label=f'Hemisphere ({a},{b})')
        a2.plot(t, res['flat']['dist'][:, seed, j], color=D_COLORS[j], lw=0.8, ls=':', label=f'Flat ({a},{b})')
    a1.axhline(np.degrees(PSI_MIN), color='r', ls='--', label='psi_min 30°')
    a1.set_ylabel('Angular separation seen from subject (deg)')
    a1.legend(fontsize=7, ncol=3)
    a2.axhline(D_MIN, color='r', ls='--', label='d_min 3 m')
    a2.set_ylabel('Inter-drone distance (m)')
    a2.set_xlabel('Time (s)')
    a2.legend(fontsize=7, ncol=4)
    for ax in (a1, a2):
        ax.grid(alpha=0.3)
    a1.set_title(f'Occlusion separation and 3D distances (seed {seed})')
    plt.tight_layout()
    path = os.path.join(out_dir, 'occlusion_distance.png')
    plt.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_monte_carlo(res, sweep, out_dir):
    fig, axes = plt.subplots(1, 4, figsize=(19, 4.8))
    keys = MAIN
    short = ['hemi', 'flat', 'hemi\nworld', 'flat\nworld', 'hemi\noracle']
    items = [(lambda r: r['phase_mean'].mean(axis=1), 'Mean phase error after 5 s (deg)'),
             (lambda r: r['elev_mean'].mean(axis=1), 'Mean elevation error after 5 s (deg)'),
             (lambda r: r['frame'], 'Fleet framing score'),
             (lambda r: r['cov_mean'], 'Mean hemisphere coverage gap (deg)')]
    for ax, (fn, lab) in zip(axes[:4], items):
        ax.boxplot([fn(res[k]) for k in keys], showmeans=True)
        ax.set_xticks(range(1, len(keys) + 1), short, fontsize=8)
        ax.set_ylabel(lab)
        ax.grid(alpha=0.3)
    axes[2].axhline(0.90, color='g', ls='--', lw=1)
    fig.suptitle(f'S086 3D Monte Carlo, {N_MC} seeds (same observation noise for every formation)')
    plt.tight_layout()
    path = os.path.join(out_dir, 'monte_carlo.png')
    plt.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_sweep(sweep, sweep_or, out_dir):
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(13, 4.6))
    th = TH2_SWEEP
    ph = np.array([sweep[x]['phase_mean'][:, 2].mean() for x in th])
    po = np.array([sweep_or[x]['phase_mean'][0, 2] for x in th])
    a1.plot(th, ph, 'o-', color='tab:purple', label='High drone, KF')
    a1.plot(th, po, 's--', color='grey', label='High drone, oracle state')
    a1.set_xlabel('Elevation of the high drone (deg)')
    a1.set_ylabel('Mean azimuth phase error after 5 s (deg)')
    a1.set_title('Phase error grows as the horizontal radius shrinks')
    a1.legend(fontsize=8)
    a1.grid(alpha=0.3)
    a2.plot(th, [sweep[x]['cov_mean'].mean() for x in th], 'o-', label='Mean coverage gap')
    a2.plot(th, [sweep[x]['psi_min_late'].mean() for x in th], 's-', label='Min pairwise separation after 5 s')
    a2.axhline(np.degrees(PSI_MIN), color='r', ls='--', lw=0.8, label='psi_min 30°')
    a2.set_xlabel('Elevation of the high drone (deg)')
    a2.set_ylabel('deg')
    a2.set_title('View geometry vs the high drone elevation')
    a2.legend(fontsize=8)
    a2.grid(alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'elevation_sweep.png')
    plt.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def save_animation(res, out_dir, seed=0):
    r = res['hemi']
    q, hist = r['q'], r['hist'][:, seed]
    n_frames = int(np.ceil((N_STEPS - 1) / ANIM_STEP)) + 1
    idx = [min(i * ANIM_STEP, N_STEPS - 1) for i in range(n_frames)]
    fig = plt.figure(figsize=(6.5, 6))
    ax = fig.add_subplot(111, projection='3d')

    def update(fi):
        i = idx[fi]
        ax.cla()
        ax.plot(q[:i + 1, 0], q[:i + 1, 1], 0 * q[:i + 1, 0], color='blue', lw=1.2, alpha=0.4)
        ax.scatter(q[i, 0], q[i, 1], 0, color='blue', s=90)
        for k in range(N_DRONES):
            h = hist[max(0, i - 100):i + 1, k]
            ax.plot(h[:, 0], h[:, 1], h[:, 2], color=D_COLORS[k], lw=1, alpha=0.5)
            ax.scatter(*hist[i, k], color=D_COLORS[k], s=80, marker='^')
            ax.plot([hist[i, k, 0], q[i, 0]], [hist[i, k, 1], q[i, 1]], [hist[i, k, 2], 0],
                    color=D_COLORS[k], ls=':', lw=0.9)
            ax.text(*hist[i, k], f' {hist[i, k, 2]:.1f} m', fontsize=7, color=D_COLORS[k])
        ax.set_xlim(-17, 17)
        ax.set_ylim(-13, 13)
        ax.set_zlim(0, 8)
        ax.set_xlabel('x (m)')
        ax.set_ylabel('y (m)')
        ax.set_zlabel('z (m)')
        ax.set_title(f'S086 3D hemisphere formation  t = {r["t"][i]:.1f} s', fontsize=10)
        ax.view_init(elev=25, azim=-60 + fi * 0.8)

    ani = animation.FuncAnimation(fig, update, frames=len(idx), blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer='pillow', fps=10, dpi=64)
    plt.close(fig)
    print(f'Saved: {path} ({len(idx)} frames)')


# ── Main ──────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    res, sweep, sweep_or = run_simulation()
    print('=== S086 3D Multi-Angle Cinematography ===')
    print(f'  Subject: corrected omega {m2d.FIG8_OMEGA:.4f} rad/s (RMS 1.5 m/s), z = 0; '
          f'{N_MC} seeds, noise of seed s from default_rng({SEED} + s)')
    print(f'  Ideal hemisphere (20/45/70 deg, r = {R_SPHERE:.0f} m): altitudes ' +
          ', '.join(f'{R_SPHERE * np.sin(t):.2f}' for t in THETA_CARD) + ' m, horizontal radii ' +
          ', '.join(f'{R_SPHERE * np.cos(t):.2f}' for t in THETA_CARD) + ' m')
    for (i, j), (psi, d) in zip(PAIRS, ideal_geometry(THETA_CARD)):
        print(f'    pair ({i},{j}): ideal separation {psi:.1f} deg, distance {d:.2f} m '
              f'(card formula {card_distance(THETA_CARD[i], THETA_CARD[j]):.2f} m)')
    for (i, j), (psi, d) in zip(PAIRS, ideal_geometry(np.full(3, THETA_FLAT))):
        print(f'    flat pair ({i},{j}): elevation {np.degrees(THETA_FLAT):.1f} deg, range {R_FLAT_3D:.2f} m, '
              f'ideal separation {psi:.1f} deg, distance {d:.2f} m')
    for (i, j), (psi, d) in zip(PAIRS, ideal_geometry(THETA_STRESS)):
        print(f'    stress pair ({i},{j}): ideal separation {psi:.1f} deg, distance {d:.2f} m')
    print(f"  {'Formation':<38}{'phase err (deg)':>16}{'within 5° (%)':>15}{'elev err (deg)':>16}"
          f"{'within ±5° (%)':>16}{'point err (deg)':>16}{'framing':>15}{'min psi (deg)':>15}"
          f"{'min dist (m)':>14}{'coverage gap mean / max (deg)':>31}")
    for key in CONFIGS:
        r = res[key]
        print(f"  {LABELS[key]:<38}{r['phase_mean'].mean():>9.2f} ± {r['phase_mean'].mean(axis=1).std():.2f}"
              f"{r['phase_ok'].mean():>15.1f}{r['elev_mean'].mean():>9.2f} ± {r['elev_mean'].mean(axis=1).std():.2f}"
              f"{r['elev_ok'].mean():>16.1f}{r['point_mean'].mean():>16.2f}"
              f"{r['frame'].mean():>9.3f} ± {r['frame'].std():.3f}{r['psi_min'].min():>15.1f}"
              f"{r['min_dist'].min():>14.2f}{r['cov_mean'].mean():>17.1f} / {r['cov_max'].mean():.1f}")
    print('  After 5 s (KF start-up excluded): min pairwise separation / min distance over all seeds')
    for key in CONFIGS:
        r = res[key]
        print(f"    {LABELS[key]:<38} {r['psi_min_late'].min():.1f} deg / {r['min_dist_late'].min():.2f} m")
    print('  Stress: max |card repulsion - no repulsion| over seeds: ' +
          ', '.join(f'{k} {v:.2e}' for k, v in res['stress_diff'].items()))
    print('  Static coverage of ideal formations (120 deg azimuth spacing): mean / worst angular gap (deg)')
    for name, th in (('flat 33.7/33.7/33.7', np.full(3, THETA_FLAT)), ('hemisphere 20/45/70', THETA_CARD),
                     ('equal 45/45/45', np.radians([45.0] * 3))):
        g = static_coverage(th)
        print(f'    {name:<22} {g[0]:.1f} / {g[1]:.1f}')
    best = min(((static_coverage(np.radians([e] * 3))[0], e) for e in range(0, 91)), key=lambda x: (x[0], x[1]))
    best3 = min(((static_coverage(np.radians([a, b, c]))[0], (a, b, c)) for a in range(0, 91, 5)
                 for b in range(0, 91, 5) for c in range(0, 91, 5)), key=lambda x: (round(x[0], 9), x[1]))
    print(f'    best equal elevation (1 deg grid): {best[1]} deg, mean gap {best[0]:.1f}; best of a 5 deg '
          f'grid over three elevations: {best3[1]} deg, mean gap {best3[0]:.1f}')
    print('  Per drone (after 5 s, mean over seeds): phase error / elevation error / mean gimbal pitch (deg)')
    for key in MAIN:
        r = res[key]
        print(f"    {LABELS[key]:<38}" + '; '.join(
            f"d{k}: {r['phase_mean'][:, k].mean():.2f} / {r['elev_mean'][:, k].mean():.2f} / "
            f"{r['pitch_mean'][:, k].mean():+.1f}" for k in range(N_DRONES)))
    print('  Pairwise separation psi after 5 s (mean over seeds, deg) and mean distance (m):')
    for key in ('hemi', 'flat', 'stress', 'stress_norep', 'stress_sym'):
        r = res[key]
        print(f"    {LABELS[key]:<38}" + '; '.join(
            f"({a},{b}) {r['psi_pair_mean'][:, j].mean():.1f}° / {r['dist_pair_mean'][:, j].mean():.2f} m"
            for j, (a, b) in enumerate(PAIRS)) +
            f"; time below 30°: {r['psi_viol'].mean():.2f}%; repulsion active {r['rep_frac'].mean():.2f}% of steps")
    H_, F_ = res['hemi'], res['flat']
    R6 = res['flat_r6']
    print(f"  Range probe: flat at 6 m range framing {R6['frame'].mean():.3f} (flat 7.21 m: {F_['frame'].mean():.3f}, "
          f"hemisphere 6 m: {H_['frame'].mean():.3f}); hemisphere higher framing than flat-at-6-m in "
          f"{int(np.sum(H_['frame'] > R6['frame']))}/{N_MC} (ties {int(np.sum(H_['frame'] == R6['frame']))})")
    print('  Hemisphere vs flat (2D baseline), same noise:')
    print(f"    lower mean phase error in {int(np.sum(H_['phase_mean'].mean(1) < F_['phase_mean'].mean(1)))}/{N_MC}; "
          f"higher framing in {int(np.sum(H_['frame'] > F_['frame']))}/{N_MC} (ties {int(np.sum(H_['frame'] == F_['frame']))}), "
          f"mean diff {H_['frame'].mean() - F_['frame'].mean():+.3f}; "
          f"smaller mean coverage gap in {int(np.sum(H_['cov_mean'] < F_['cov_mean']))}/{N_MC} "
          f"(diff {H_['cov_mean'].mean() - F_['cov_mean'].mean():+.1f} deg); smaller worst coverage gap in "
          f"{int(np.sum(H_['cov_max'] < F_['cov_max']))}/{N_MC} (diff {H_['cov_max'].mean() - F_['cov_max'].mean():+.1f} deg); "
          f"larger min separation in {int(np.sum(H_['psi_min'] > F_['psi_min']))}/{N_MC}")
    print(f"    lower mean elevation error in {int(np.sum(H_['elev_mean'].mean(1) < F_['elev_mean'].mean(1)))}/{N_MC}; "
          f"larger closest-pair distance in {int(np.sum(H_['min_dist'] > F_['min_dist']))}/{N_MC} (whole run) and "
          f"{int(np.sum(H_['min_dist_late'] > F_['min_dist_late']))}/{N_MC} (after 5 s); larger min separation after 5 s in "
          f"{int(np.sum(H_['psi_min_late'] > F_['psi_min_late']))}/{N_MC}; lower pointing error in "
          f"{int(np.sum(H_['point_mean'] < F_['point_mean']))}/{N_MC}")
    print(f"  Card distance target >= 5 m: hemisphere min over seeds {H_['min_dist'].min():.2f} m (whole run) -> "
          f"{'met' if H_['min_dist'].min() >= 5 else 'NOT met'}; after 5 s {H_['min_dist_late'].min():.2f} m -> "
          f"{'met' if H_['min_dist_late'].min() >= 5 else 'NOT met'}; seeds with whole-run min >= 5 m: "
          f"{int(np.sum(H_['min_dist'] >= 5))}/{N_MC}")
    for a, b in (('hemi_world', 'flat_world'),):
        A, B = res[a], res[b]
        print(f"    world-fixed: hemisphere lower phase error in {int(np.sum(A['phase_mean'].mean(1) < B['phase_mean'].mean(1)))}/{N_MC} "
              f"({A['phase_mean'].mean():.2f} vs {B['phase_mean'].mean():.2f} deg); higher framing in "
              f"{int(np.sum(A['frame'] > B['frame']))}/{N_MC} ({A['frame'].mean():.3f} vs {B['frame'].mean():.3f})")
    print(f"  Framing with the S086 field of view (half-width 2 m at 6 m, {np.degrees(np.arctan(m2d.FOV_HALF_WIDTH / m2d.ORBIT_RADIUS)):.1f} deg): "
          + ', '.join(f"{k} {res[k]['frame_s086fov'].mean():.3f}" for k in MAIN))
    print(f"  KF position RMSE: " + ', '.join(f"{k} {res[k]['kf_rmse'].mean():.3f} m" for k in MAIN if k != 'hemi_oracle'))
    print('  High-drone elevation sweep (theta = 20/45/x deg, heading-locked KF; oracle on seed 0):')
    for th2 in TH2_SWEEP:
        r, ro = sweep[th2], sweep_or[th2]
        rh = R_SPHERE * np.cos(np.radians(th2))
        print(f"    x = {th2:.0f} deg: horizontal radius {rh:.2f} m; high-drone phase error {r['phase_mean'][:, 2].mean():.2f} deg "
              f"(oracle {ro['phase_mean'][0, 2]:.2f}), elevation error {r['elev_mean'][:, 2].mean():.2f} deg; framing "
              f"{r['frame'].mean():.3f}; min psi after 5 s {r['psi_min_late'].mean():.1f} deg; coverage gap mean / max "
              f"{r['cov_mean'].mean():.1f} / {r['cov_max'].mean():.1f} deg; min distance {r['min_dist'].min():.2f} m")

    os.makedirs(OUTPUT_DIR, exist_ok=True)
    plot_hemisphere(res, OUTPUT_DIR)
    plot_altitude_elevation(res, OUTPUT_DIR)
    plot_gimbal(res, OUTPUT_DIR)
    plot_occlusion_distance(res, OUTPUT_DIR)
    plot_monte_carlo(res, sweep, OUTPUT_DIR)
    plot_sweep(sweep, sweep_or, OUTPUT_DIR)
    save_animation(res, OUTPUT_DIR)
