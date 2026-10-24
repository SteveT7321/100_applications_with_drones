"""
S091 Aerial Acrobatic Maneuvers
===============================
A 0.5 kg quadrotor flies a timed aerobatic sequence at 10 m: backflip (360 deg pitch), barrel
roll (360 deg roll), Immelmann (180 deg pitch-up half loop + 180 deg roll) and a 2 s inverted
hold, with 2 s stabilisation pauses. Three attitude controllers track the same SO(3) reference
with the same position controller and the same rotor allocation:

  1. Euler-angle PID   - ZYX roll/pitch/yaw errors fed to a PID (small-angle torque law).
  2. Quaternion PD     - error quaternion (shortest-path sign), PD + SO(3) feed-forward.
  3. Geometric SO(3)   - Lee-Leok-McClamroch (2010) e_R / e_Omega controller with feed-forward.

The plant is the 6-DOF rigid-body model (src/base/rigid_body_drone.py): quaternion attitude,
first-order motor lag, rotor-speed limits and airmode desaturation. Mass, inertia and arm are the
card's; the rotor constants are the fpv_5inch preset because the card's k_f with
omega_max = 2000 rad/s cannot lift 0.5 kg (checked below). A quadrotor cannot hover inverted, so
the 2 s inverted hold is flown as a ballistic "hang": a short 3 g pop-up gives the drone the
vertical speed that brings it back to 10 m at the end of the inverted manoeuvre.

The nominal run is deterministic; a 10-seed robustness study adds gyro noise (0.02 rad/s) and
wind gusts (sigma 2 m/s), with the same noise and gusts for every controller.

Usage:
    conda activate drones
    python src/05_special_entertainment/s091_acrobatics.py
"""

import os
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'base'))
from rigid_body_drone import (RigidBodyDrone, make_params, quat_mul, quat_conj,  # noqa: E402
                              quat_to_rot, rot_to_quat, G)

# ── Parameters (card) ─────────────────────────────────────────────────────────
MASS        = 0.5
INERTIA     = np.array([4.9e-3, 4.9e-3, 8.8e-3])
ARM         = 0.17
KF_CARD     = 3.16e-10       # N s^2 (card)
OMEGA_MAX_CARD = 2000.0      # rad/s (card)
K_R         = np.array([6.0, 6.0, 3.0])
K_W         = np.array([0.5, 0.5, 0.3])
K_X         = 4.0
K_V         = 3.0
DT          = 0.005
Z_HOME      = 10.0
HOME        = np.array([0.0, 0.0, Z_HOME])
ER_TARGET   = 0.15           # rad   RMS attitude error target
W_BOUND     = 12.0           # rad/s peak angular-rate bound
DELTA_OK    = 5.0            # deg   completion threshold
POS_OK      = 0.5            # m     position recovery tolerance
T_PAUSE     = 2.0            # s     stabilisation pause after each manoeuvre

# ── Parameters (added) ────────────────────────────────────────────────────────
K_I_EULER   = np.array([1.0, 1.0, 0.5])   # N m / (rad s), Euler PID integral gain
I_MAX_EULER = 0.5                         # rad s, integral clamp
A_POP       = 3 * G          # m/s^2 pop-up acceleration before every manoeuvre
N_ROB       = 10             # robustness seeds
GYRO_STD    = 0.02           # rad/s
GUST_STD    = 2.0            # m/s
GUST_TAU    = 1.0            # s
A_Z_MIN     = 0.3 * G        # m/s^2 minimum upward command in upright holds (no flip-over)
TILT_POS    = np.radians(60.0)  # max tilt commanded by the position loop in upright holds

# (name, duration, kind, body axis, angle)
T_INV = (0.5, 2.0, 1.5)      # flip to inverted, hold, recover


def pop_speed(t_ball):
    """Launch speed of a symmetric zero-thrust arc lasting t_ball: v_up = g t_ball / 2.
    The pop-up accelerates from rest at Z_HOME at A_POP to v_up; after the arc the drone falls
    at v_up through the launch height and brakes at A_POP to rest exactly at Z_HOME."""
    return G * t_ball / 2.0


T_BALLS = {'pop_1': 1.5, 'pop_2': 1.5, 'pop_3': 3.0, 'pop_4': sum(T_INV)}
SCHED = [
    ('hover_init', 2.0, 'hold', None, 0.0),
    ('pop_1', pop_speed(T_BALLS['pop_1']) / A_POP, 'pop', None, 0.0),
    ('backflip', 1.5, 'rot', 1, -2 * np.pi),
    ('stabilise_1', 2.0, 'hold', None, 0.0),
    ('pop_2', pop_speed(T_BALLS['pop_2']) / A_POP, 'pop', None, 0.0),
    ('barrel_roll', 1.5, 'rot', 0, 2 * np.pi),
    ('stabilise_2', 2.0, 'hold', None, 0.0),
    ('pop_3', pop_speed(T_BALLS['pop_3']) / A_POP, 'pop', None, 0.0),
    ('immelmann_half', 1.5, 'rot', 1, -np.pi),
    ('immelmann_roll', 1.5, 'rot', 0, np.pi),
    ('stabilise_3', 2.0, 'home', None, 0.0),
    ('pop_4', pop_speed(T_BALLS['pop_4']) / A_POP, 'pop', None, 0.0),
    ('inverted_flip', T_INV[0], 'rot', 0, np.pi),
    ('inverted_hold', T_INV[1], 'hold', None, 0.0),
    ('recover', T_INV[2], 'rot', 0, np.pi),
    ('final_hover', 4.0, 'hold', None, 0.0),   # card 2.0 s; recovery is still scored at 2 s
]
MANOEUVRES = [  # (label, segments flown, following pause)
    ('Backflip', ['backflip'], 'stabilise_1'),
    ('Barrel roll', ['barrel_roll'], 'stabilise_2'),
    ('Immelmann', ['immelmann_half', 'immelmann_roll'], 'stabilise_3'),
    ('Inverted hold', ['inverted_flip', 'inverted_hold', 'recover'], 'final_hover'),
]
CTRLS = ['euler', 'quat', 'geom']
LABEL = {'euler': 'Euler-angle PID', 'quat': 'Quaternion PD', 'geom': 'Geometric SO(3)'}
COLOR = {'euler': 'tab:blue', 'quat': 'tab:orange', 'geom': 'tab:red'}
M_MARK = {'Backflip': ('^', 'orange'), 'Barrel roll': ('s', 'purple'),
          'Immelmann': ('D', 'cyan'), 'Inverted hold': ('*', 'gold')}

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), '..', '..',
                          'outputs', '05_special_entertainment', 's091_acrobatics')


# ── SO(3) helpers ─────────────────────────────────────────────────────────────
def vee(S):
    return np.stack([S[..., 2, 1], S[..., 0, 2], S[..., 1, 0]], axis=-1)


def quat_axis_angle(axis, ang):
    return np.concatenate([[np.cos(ang / 2)], np.sin(ang / 2) * axis])


def geodesic_deg(Ra, Rb):
    tr = np.trace(np.swapaxes(Ra, -1, -2) @ Rb, axis1=-2, axis2=-1)
    return np.degrees(np.arccos(np.clip((tr - 1) / 2, -1.0, 1.0)))


def euler_zyx(R):
    roll = np.arctan2(R[..., 2, 1], R[..., 2, 2])
    pitch = np.arcsin(np.clip(-R[..., 2, 0], -1.0, 1.0))
    yaw = np.arctan2(R[..., 1, 0], R[..., 0, 0])
    return np.stack([roll, pitch, yaw], axis=-1)


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


# ── Reference ─────────────────────────────────────────────────────────────────
def build_reference():
    """Attitude (quaternion, body rate, body angular acceleration) and position reference."""
    t_seg = np.cumsum([0.0] + [s[1] for s in SCHED])
    n = int(round(t_seg[-1] / DT)) + 1
    t = np.arange(n) * DT
    q = np.zeros((n, 4))
    w = np.zeros((n, 3))
    wd = np.zeros((n, 3))
    p = np.tile(HOME, (n, 1))
    v = np.zeros((n, 3))
    a = np.zeros((n, 3))
    seg_id = np.zeros(n, int)
    q0 = np.array([1.0, 0, 0, 0])
    for k, (name, dur, kind, ax, ang) in enumerate(SCHED):
        sel = (t >= t_seg[k] - 1e-9) & (t < t_seg[k + 1] - 1e-9)
        if k == len(SCHED) - 1:
            sel = t >= t_seg[k] - 1e-9
        seg_id[sel] = k
        u = np.clip((t[sel] - t_seg[k]) / dur, 0, 1)
        s, sd, sdd = 3 * u ** 2 - 2 * u ** 3, 6 * u * (1 - u) / dur, (6 - 12 * u) / dur ** 2
        if kind in ('hold', 'pop'):
            q[sel] = q0
            continue
        if kind == 'rot':
            axis = np.eye(3)[ax]
            total = ang
        else:                       # geodesic back to identity
            qe = quat_mul(quat_conj(q0), np.array([1.0, 0, 0, 0]))
            qe = qe if qe[0] >= 0 else -qe
            sv = np.linalg.norm(qe[1:])
            total = 2 * np.arctan2(sv, qe[0])
            axis = qe[1:] / sv if sv > 1e-12 else np.array([0, 0, 1.0])
        dq = np.stack([quat_axis_angle(axis, s_ * total) for s_ in s])
        q[sel] = quat_mul(np.broadcast_to(q0, dq.shape), dq)
        w[sel] = (sd * total)[:, None] * axis
        wd[sel] = (sdd * total)[:, None] * axis
        q0 = quat_mul(q0, quat_axis_angle(axis, total))
        q0 = q0 / np.linalg.norm(q0)
    # pop-up (constant A_POP) followed by a zero-thrust ballistic arc over the manoeuvre
    for k, (name, dur, kind, _, _) in enumerate(SCHED):
        if kind != 'pop':
            continue
        v_up = pop_speed(T_BALLS[name])
        t_pop0, t_ball0 = t_seg[k], t_seg[k + 1]
        z_pop1 = Z_HOME + 0.5 * A_POP * dur ** 2
        pp = (t >= t_pop0) & (t < t_ball0)
        tau = t[pp] - t_pop0
        p[pp, 2] = Z_HOME + 0.5 * A_POP * tau ** 2
        v[pp, 2] = A_POP * tau
        a[pp, 2] = A_POP
        bb = (t >= t_ball0) & (t < t_ball0 + T_BALLS[name] - 1e-9)
        tau = t[bb] - t_ball0
        p[bb, 2] = z_pop1 + v_up * tau - 0.5 * G * tau ** 2
        v[bb, 2] = v_up - G * tau
        a[bb, 2] = -G
        t_b = t_ball0 + T_BALLS[name]
        z_b = z_pop1 + v_up * T_BALLS[name] - 0.5 * G * T_BALLS[name] ** 2
        v_b = v_up - G * T_BALLS[name]
        br = (t >= t_b - 1e-9) & (t < t_b - v_b / A_POP - 1e-9)
        tau = t[br] - t_b
        p[br, 2] = z_b + v_b * tau + 0.5 * A_POP * tau ** 2
        v[br, 2] = v_b + A_POP * tau
        a[br, 2] = A_POP
    return dict(t=t, q=q, R=quat_to_rot(q), w=w, wd=wd, p=p, v=v, a=a, seg=seg_id,
                t_seg=t_seg)


# ── Attitude controllers ──────────────────────────────────────────────────────
def attitude_torques(ctrl, R, quat, Om, Rd, Wd, Wdd, eta_d, eta_d_dot, e_int):
    """Body torques of the three controllers; ctrl selects one per row (0 Euler, 1 quat, 2 geom).
    Wd, Wdd: reference body rate / acceleration in the reference frame."""
    J = INERTIA
    B = len(ctrl)
    qd = rot_to_quat(Rd)
    # SO(3) feed-forward shared by the quaternion and geometric controllers
    RtRd = np.swapaxes(R, 1, 2) @ Rd
    w_dbody = RtRd @ Wd
    e_W = Om - w_dbody
    ff = np.cross(Om, J * Om) - J * (np.cross(Om, w_dbody) - RtRd @ Wdd)
    # 3. geometric (Lee et al. 2010)
    e_R = 0.5 * vee(np.swapaxes(Rd, 1, 2) @ R - RtRd)
    tau_g = -K_R * e_R - K_W * e_W + ff
    # 2. quaternion PD, shortest-path sign
    qe = quat_mul(quat_conj(qd), quat)
    sgn = np.where(qe[:, :1] >= 0, 1.0, -1.0)
    tau_q = -K_R * (2 * sgn * qe[:, 1:]) - K_W * e_W + ff
    # 1. Euler-angle PID (ZYX); Euler rates from the gyro through the singular kinematic map
    eta = euler_zyx(R)
    ph, th = eta[:, 0], eta[:, 1]
    cth = np.where(np.abs(np.cos(th)) < 1e-3, 1e-3 * np.sign(np.cos(th) + 1e-12), np.cos(th))
    eta_dot = np.column_stack([
        Om[:, 0] + np.sin(ph) * np.tan(th) * Om[:, 1] + np.cos(ph) * np.tan(th) * Om[:, 2],
        np.cos(ph) * Om[:, 1] - np.sin(ph) * Om[:, 2],
        (np.sin(ph) * Om[:, 1] + np.cos(ph) * Om[:, 2]) / cth])
    e_eta = wrap(eta - eta_d)
    e_int = np.clip(e_int + e_eta * DT, -I_MAX_EULER, I_MAX_EULER)
    tau_e = (-K_R * e_eta - K_W * (eta_dot - eta_d_dot) - K_I_EULER * e_int
             + np.cross(Om, J * Om))
    tau = np.where((ctrl == 0)[:, None], tau_e,
                   np.where((ctrl == 1)[:, None], tau_q, tau_g))
    assert tau.shape == (B, 3)
    return tau, e_int


def recovery_test(angles_deg=(90.0, 135.0, 170.0, 179.0), t_end=3.0):
    """Recovery from a large initial attitude error to level (desired = identity, thrust = mg
    projected on body z). Returns time to stay within 5 deg and the peak body rate."""
    axis = np.array([1.0, 1.0, 0.3]) / np.linalg.norm([1.0, 1.0, 0.3])
    cases = [(c, a) for c in CTRLS for a in angles_deg]
    B = len(cases)
    ctrl = np.array([CTRLS.index(c) for c, _ in cases])
    params = make_params('fpv_5inch', mass=MASS, inertia=INERTIA, arm=ARM)
    drone = RigidBodyDrone(np.tile([0.0, 0.0, 100.0], (B, 1)), preset='fpv_5inch',
                           params=params, dt=DT, record=True)
    drone.set_state(quat=np.stack([quat_axis_angle(axis, np.radians(a)) for _, a in cases]))
    I3 = np.broadcast_to(np.eye(3), (B, 3, 3))
    e_int = np.zeros((B, 3))
    zero = np.zeros(3)
    n = int(round(t_end / DT))
    for _ in range(n):
        R = quat_to_rot(drone.quat)
        f = np.clip(MASS * G * R[:, 2, 2], 0.0, params.thrust_max)
        tau, e_int = attitude_torques(ctrl, R, drone.quat, drone.omega, I3, zero, zero,
                                      np.zeros((B, 3)), zero, e_int)
        drone.step_wrench(f, tau)
    lg = drone.get_log()
    ang = geodesic_deg(I3[None], quat_to_rot(lg['quat']))           # (T, B)
    out = {}
    for b, (c, a) in enumerate(cases):
        bad = np.where(ang[:, b] > DELTA_OK)[0]
        t_rec = lg['t'][bad[-1] + 1] if len(bad) and bad[-1] + 1 < len(lg['t']) else (
            0.0 if not len(bad) else np.nan)
        out[(c, a)] = dict(t_rec=float(t_rec), final=float(ang[-1, b]),
                           W_peak=float(np.linalg.norm(lg['omega'][:, b], axis=1).max()))
    return out


# ── Simulation ────────────────────────────────────────────────────────────────
def run_simulation():
    ref = build_reference()
    n = len(ref['t'])
    cases = [(c, None) for c in CTRLS] + [(c, s) for s in range(N_ROB) for c in CTRLS]
    B = len(cases)
    ctrl = np.array([CTRLS.index(c) for c, _ in cases])
    params = make_params('fpv_5inch', mass=MASS, inertia=INERTIA, arm=ARM)
    # common random numbers: gyro noise and gusts per seed, shared by the three controllers
    gyro = np.zeros((B, n, 3))
    gust = np.zeros((B, n, 3))
    for b, (c, s) in enumerate(cases):
        if s is None:
            continue
        rng = np.random.default_rng(1000 + s)
        gyro[b] = GYRO_STD * rng.standard_normal((n, 3))
        al = np.exp(-DT / GUST_TAU)
        xi = rng.standard_normal((n, 3))
        g = GUST_STD * xi[0]
        for k in range(n):
            gust[b, k] = g
            g = al * g + GUST_STD * np.sqrt(1 - al * al) * xi[k]
    wind_k = [0]
    drone = RigidBodyDrone(np.tile(HOME, (B, 1)), preset='fpv_5inch', params=params, dt=DT,
                           wind=lambda t, pos, h: gust[:, wind_k[0]], record=True,
                           ground_z=0.0)
    # upright hold / pause segments: the desired attitude comes from the position controller
    kinds = np.array([SCHED[i][2] for i in ref['seg']])
    pos_att = np.isin(kinds, ['hold', 'home', 'pop']) & (ref['R'][:, 2, 2] > 0.5)
    yaw_ref = np.arctan2(ref['R'][:, 1, 0], ref['R'][:, 0, 0])
    Rd_log = np.zeros((n, B, 3, 3))
    e_int = np.zeros((B, 3))
    eta_ref = np.unwrap(euler_zyx(ref['R']), axis=0)
    eta_ref_dot = np.gradient(eta_ref, DT, axis=0)
    thrust_cmd = np.zeros((n, B))
    for k in range(n - 1):
        wind_k[0] = k
        R = quat_to_rot(drone.quat)
        Om = drone.omega + gyro[:, k]
        Wd, Wdd = ref['w'][k], ref['wd'][k]
        # position PD (card) and thrust projected on the current body z axis
        ex = drone.pos - ref['p'][k]
        ev = drone.vel - ref['v'][k]
        a_des = -K_X * ex - K_V * ev + G * np.array([0, 0, 1.0]) + ref['a'][k]
        if pos_att[k]:
            # upright holds: keep the commanded thrust pointing up, tilt <= TILT_POS
            az = np.maximum(a_des[:, 2], A_Z_MIN)
            ah = a_des[:, :2]
            nh = np.linalg.norm(ah, axis=1)
            lim = az * np.tan(TILT_POS)
            ah = ah * np.where(nh > lim, lim / np.maximum(nh, 1e-12), 1.0)[:, None]
            a_des = np.column_stack([ah, az])
        f = np.clip(MASS * np.sum(a_des * R[:, :, 2], axis=1), 0.0, params.thrust_max)
        if pos_att[k]:
            b3 = a_des / np.linalg.norm(a_des, axis=1, keepdims=True)
            b1c = np.array([np.cos(yaw_ref[k]), np.sin(yaw_ref[k]), 0.0])
            b2 = np.cross(b3, b1c)
            b2 /= np.linalg.norm(b2, axis=1, keepdims=True)
            Rd = np.stack([np.cross(b2, b3), b2, b3], axis=-1)
        else:
            Rd = np.broadcast_to(ref['R'][k], (B, 3, 3))
        Rd_log[k] = Rd
        eta_d = euler_zyx(Rd) if pos_att[k] else np.broadcast_to(eta_ref[k], (B, 3))
        eta_d_dot = eta_ref_dot[k] * (0.0 if pos_att[k] else 1.0)
        tau, e_int = attitude_torques(ctrl, R, drone.quat, Om, Rd, Wd, Wdd, eta_d, eta_d_dot,
                                      e_int)
        thrust_cmd[k] = f
        drone.step_wrench(f, tau)
        assert np.all(np.isfinite(drone.pos))
    thrust_cmd[-1] = thrust_cmd[-2]
    Rd_log[-1] = Rd_log[-2]
    lg = drone.get_log()
    lg['Rd'] = Rd_log
    assert np.all(lg['pos'][:, :, 2] >= 0.0), 'below ground'
    assert np.all(lg['rotor'] <= params.w_max + 1e-9)
    return ref, lg, cases, params, thrust_cmd, drone.n_sat


def metrics(ref, lg, b):
    t = ref['t']
    R = quat_to_rot(lg['quat'][:, b])
    Rd = lg['Rd'][:, b]
    e_R = 0.5 * vee(np.swapaxes(Rd, 1, 2) @ R - np.swapaxes(R, 1, 2) @ Rd)
    eR = np.linalg.norm(e_R, axis=1)
    ang = geodesic_deg(Rd, R)
    W = np.linalg.norm(lg['omega'][:, b], axis=1)
    names = [s[0] for s in SCHED]
    ground = np.where(lg['pos'][:, b, 2] <= 1e-9)[0]
    t_crash = float(t[ground[0]]) if len(ground) else np.inf
    out = dict(eR=eR, ang=ang, W=W, man={}, t_crash=t_crash)
    for label, segs, pause in MANOEUVRES:
        k0, k1 = names.index(segs[0]), names.index(segs[-1])
        t0, t1 = ref['t_seg'][k0], ref['t_seg'][k1 + 1]
        if t0 >= t_crash:
            out['man'][label] = None          # crashed before this manoeuvre
            continue
        sel = (t >= t0 - 1e-9) & (t <= t1 + 1e-9)
        i1 = int(round(t1 / DT)) - 1          # last sample of the manoeuvre
        ip = int(round((ref['t_seg'][names.index(pause)] + T_PAUSE) / DT))
        ie = min(int(round(ref['t_seg'][names.index(pause) + 1] / DT)), len(t) - 1)
        out['man'][label] = dict(
            eR_rms=float(np.sqrt(np.mean(eR[sel] ** 2))),
            ang_rms=float(np.sqrt(np.mean(ang[sel] ** 2))),
            ang_max=float(ang[sel].max()), delta=float(ang[i1]),
            W_peak=float(W[sel].max()),
            pos_err=float(np.linalg.norm(lg['pos'][ip, b] - HOME)),
            pos_err_end=float(np.linalg.norm(lg['pos'][ie, b] - HOME)),
            dz_max=float(np.max(np.abs(lg['pos'][sel, b, 2] - Z_HOME))),
            dz_ref=float(np.max(np.abs(lg['pos'][sel, b, 2] - ref['p'][sel, 2]))))
    alive = t < t_crash
    out['eR_rms'] = float(np.sqrt(np.mean(eR[alive] ** 2)))
    out['ang_rms'] = float(np.sqrt(np.mean(ang[alive] ** 2)))
    out['W_peak'] = float(W[alive].max())
    out['T_peak'] = float(lg['thrust'][:, b].max())
    out['z_min'] = float(lg['pos'][:, b, 2].min())
    out['z_max'] = float(lg['pos'][:, b, 2].max())
    ih = names.index('inverted_hold')
    hs = (t >= ref['t_seg'][ih]) & (t < ref['t_seg'][ih + 1])
    out['T_hold'] = float(lg['thrust'][hs, b].mean())
    out['success'] = (not len(ground)) and all(m['delta'] <= DELTA_OK and m['pos_err'] <= POS_OK
                                               for m in out['man'].values())
    return out


def mget(r, label, key, crashed=np.nan):
    """Manoeuvre metric, or `crashed` if the drone hit the ground before the manoeuvre."""
    m = r['man'][label]
    return crashed if m is None else m[key]


# ── Plots ─────────────────────────────────────────────────────────────────────
def plot_trajectory_3d(ref, lg, M, out_dir):
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    names = [s[0] for s in SCHED]
    for b, c in enumerate(CTRLS):
        P = lg['pos'][ref['t'] <= M[c]['t_crash'], b]
        crash = ', crashed' if np.isfinite(M[c]['t_crash']) else ''
        ax.plot(P[:, 0], P[:, 1], P[:, 2], '--' if c == 'euler' else '-', color=COLOR[c],
                lw=1.3, label=f"{LABEL[c]} (RMS angle error {M[c]['ang_rms']:.1f} deg{crash})")
        if crash:
            ax.scatter(*P[-1], marker='x', color=COLOR[c], s=80)
    for label, segs, _ in MANOEUVRES:
        i = int(round(ref['t_seg'][names.index(segs[0])] / DT))
        mk, col = M_MARK[label]
        P = lg['pos'][i, CTRLS.index('geom')]
        ax.scatter(*P, marker=mk, color=col, s=90, edgecolor='k', label=f'start of {label}')
    ax.scatter(*HOME, marker='^', color='green', s=100, label='home')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_zlabel('z (m)')
    ax.set_title('S091 acrobatic sequence: position of the three controllers')
    ax.legend(fontsize=7, loc='upper left')
    ax.view_init(elev=20, azim=-55)
    path = os.path.join(out_dir, 'trajectory_3d.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_metrics_panel(ref, lg, M, out_dir):
    t = ref['t']
    fig, axes = plt.subplots(2, 3, figsize=(18, 9))
    for b, c in enumerate(CTRLS):
        ls = '--' if c == 'euler' else '-'
        a = t <= M[c]['t_crash']                 # curves end at ground contact
        lab = LABEL[c] + (f" (crash {M[c]['t_crash']:.2f} s)"
                          if np.isfinite(M[c]['t_crash']) else '')
        axes[0, 0].plot(t[a], M[c]['ang'][a], ls, color=COLOR[c], lw=1.0, label=lab)
        axes[0, 1].plot(t[a], M[c]['W'][a], ls, color=COLOR[c], lw=1.0, label=lab)
        axes[1, 0].plot(t[a], lg['thrust'][a, b] / (MASS * G), ls, color=COLOR[c], lw=1.0,
                        label=lab)
        axes[1, 1].plot(t[a], lg['pos'][a, b, 2] - Z_HOME, ls, color=COLOR[c], lw=1.0,
                        label=lab)
    eul = np.degrees(np.unwrap(euler_zyx(quat_to_rot(lg['quat'][:, CTRLS.index('geom')])),
                               axis=0))
    axes[0, 2].plot(t, eul[:, 0], color='tab:red', lw=1.0, label='roll (geometric)')
    axes[0, 2].plot(t, eul[:, 1], color='tab:green', lw=1.0, label='pitch (geometric)')
    axes[0, 2].plot(t, eul[:, 2], color='grey', lw=1.0, label='yaw (geometric)')
    axes[0, 0].axhline(np.degrees(ER_TARGET), color='k', ls=':', lw=1,
                       label=f'{np.degrees(ER_TARGET):.1f} deg (= 0.15 rad)')
    axes[0, 0].set_ylabel('geodesic attitude error (deg)')
    axes[0, 0].set_yscale('symlog', linthresh=1.0)
    axes[0, 0].set_ylim(0, 200)
    axes[0, 1].axhline(W_BOUND, color='k', ls=':', lw=1, label='12 rad/s bound')
    axes[0, 1].set_ylabel(r'$|\Omega|$ (rad/s)')
    axes[0, 2].set_ylabel('unwrapped ZYX Euler angle (deg)')
    axes[1, 0].axhline(1.0, color='k', ls=':', lw=1, label='hover thrust mg')
    axes[1, 0].set_ylabel('total thrust / mg')
    z_ref = ref['p'][:, 2] - Z_HOME
    axes[1, 1].plot(t, z_ref, ':', color='k', lw=1.0, label='reference (pop-up + ballistic)')
    axes[1, 1].axhspan(-POS_OK, POS_OK, color='green', alpha=0.15, label=r'$\pm$0.5 m')
    axes[1, 1].set_ylabel(r'$z - z_0$ (m)')
    for ax in axes.flat[:5]:
        ax.set_xlabel('time (s)')
        ax.grid(alpha=0.3)
        ax.legend(fontsize=7)
        for ts in ref['t_seg'][1:-1]:
            ax.axvline(ts, color='grey', lw=0.3)
    x = np.arange(len(MANOEUVRES))
    for i, c in enumerate(CTRLS):
        d = [mget(M[c], m[0], 'delta', 180.0) for m in MANOEUVRES]
        axes[1, 2].bar(x + (i - 1) * 0.27, d, 0.27, color=COLOR[c], label=LABEL[c])
        for j, m in enumerate(MANOEUVRES):
            if M[c]['man'][m[0]] is None:
                axes[1, 2].text(x[j] + (i - 1) * 0.27, 200, 'crashed', rotation=90,
                                ha='center', va='bottom', fontsize=7)
    axes[1, 2].axhline(DELTA_OK, color='k', ls='--', lw=1, label='5 deg threshold')
    axes[1, 2].set_yscale('log')
    axes[1, 2].set_xticks(x)
    axes[1, 2].set_xticklabels([m[0] for m in MANOEUVRES], fontsize=8)
    axes[1, 2].set_ylim(0.005, 1000)
    axes[1, 2].set_ylabel(r'completion error $\delta_k$ (deg, log; crashed = 180)')
    axes[1, 2].legend(fontsize=7)
    axes[1, 2].grid(alpha=0.3, axis='y')
    axes[0, 0].set_title('Attitude error (log above 1 deg)')
    axes[0, 1].set_title('Body angular rate')
    axes[0, 2].set_title('Attitude history')
    axes[1, 0].set_title('Total rotor thrust')
    axes[1, 1].set_title('Altitude deviation')
    axes[1, 2].set_title('Error at the end of each manoeuvre')
    plt.tight_layout()
    path = os.path.join(out_dir, 'metrics_panel.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_robustness(R_rob, out_dir):
    fig, axes = plt.subplots(1, 2, figsize=(13, 4.5))
    for i, c in enumerate(CTRLS):
        d = np.array([[mget(r, m[0], 'delta', 180.0) for m in MANOEUVRES] for r in R_rob[c]])
        x = np.arange(len(MANOEUVRES)) + (i - 1) * 0.27
        axes[0].boxplot(np.maximum(d, 1e-3), positions=x, widths=0.22, patch_artist=True,
                        boxprops=dict(facecolor=COLOR[c], alpha=0.7))
        crit = [
            sum(np.isinf(r['t_crash']) and all(mget(r, m[0], 'delta', 180.0) <= DELTA_OK
                                               for m in MANOEUVRES) for r in R_rob[c]),
            sum(np.isinf(r['t_crash']) and all(mget(r, m[0], 'pos_err', 99.0) <= POS_OK
                                               for m in MANOEUVRES) for r in R_rob[c]),
            sum(np.isinf(r['t_crash']) and all(mget(r, m[0], 'pos_err', 99.0) <= POS_OK
                                               for m in MANOEUVRES[:-1])
                and mget(r, 'Inverted hold', 'pos_err_end', 99.0) <= POS_OK for r in R_rob[c])]
        for j, (v, hat_) in enumerate(zip(crit, (None, '//', '..'))):
            xb = i + (j - 1) * 0.27
            axes[1].bar(xb, v, 0.27, color=COLOR[c], hatch=hat_, edgecolor='k')
            axes[1].text(xb, v + 0.1, f'{v}', ha='center', fontsize=8)
    axes[0].axhline(DELTA_OK, color='k', ls='--', lw=1)
    axes[0].set_yscale('log')
    axes[0].set_xticks(range(len(MANOEUVRES)))
    axes[0].set_xticklabels([m[0] for m in MANOEUVRES])
    axes[0].set_ylabel(r'$\delta_k$ (deg, log; crashed = 180)')
    axes[0].set_title(f'Completion error over {N_ROB} noisy runs (blue/orange/red = '
                      'Euler/quaternion/geometric)', fontsize=9)
    axes[1].set_xticks(range(3))
    axes[1].set_xticklabels([LABEL[c] for c in CTRLS])
    axes[1].set_ylim(0, N_ROB + 1.5)
    axes[1].set_ylabel(f'runs out of {N_ROB}')
    from matplotlib.patches import Patch
    axes[1].legend(handles=[Patch(facecolor='w', edgecolor='k', label='all delta <= 5 deg'),
                            Patch(facecolor='w', edgecolor='k', hatch='//',
                                  label='all recoveries <= 0.5 m at 2 s'),
                            Patch(facecolor='w', edgecolor='k', hatch='..',
                                  label='same, inverted hang scored at 4 s')],
                   fontsize=7, loc='upper left')
    axes[1].set_title(f'Gyro noise {GYRO_STD} rad/s + gusts {GUST_STD} m/s')
    for ax in axes:
        ax.grid(alpha=0.3, axis='y')
    plt.tight_layout()
    path = os.path.join(out_dir, 'robustness.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_recovery(rec, out_dir):
    angles = sorted({a for _, a in rec})
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.2))
    x = np.arange(len(angles))
    for i, c in enumerate(CTRLS):
        tr = [rec[(c, a)]['t_rec'] for a in angles]
        hv = [3.2 if not np.isfinite(v) else v for v in tr]
        axes[0].bar(x + (i - 1) * 0.27, hv, 0.27, color=COLOR[c], label=LABEL[c])
        for j, v in enumerate(tr):
            if not np.isfinite(v):
                axes[0].text(x[j] + (i - 1) * 0.27, 3.25, 'no', ha='center', fontsize=7)
        axes[1].bar(x + (i - 1) * 0.27, [rec[(c, a)]['W_peak'] for a in angles], 0.27,
                    color=COLOR[c], label=LABEL[c])
    axes[0].set_ylabel('time to stay within 5 deg (s); "no" = not within 3 s')
    axes[1].axhline(W_BOUND, color='k', ls=':', lw=1)
    axes[1].set_ylabel(r'peak $|\Omega|$ during recovery (rad/s)')
    for ax in axes:
        ax.set_xticks(x)
        ax.set_xticklabels([f'{a:.0f} deg' for a in angles])
        ax.set_xlabel('initial attitude error')
        ax.grid(alpha=0.3, axis='y')
        ax.legend(fontsize=8)
    axes[0].set_title('Recovery from a large attitude error to level')
    axes[1].set_title('Peak body rate (dotted = 12 rad/s bound)')
    plt.tight_layout()
    path = os.path.join(out_dir, 'recovery_test.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def save_animation(ref, lg, out_dir, step=20, trail=300):
    """Geometric controller: body axes (x red, y green, z blue), fading trail."""
    b = CTRLS.index('geom')
    P = lg['pos'][:, b]
    R = quat_to_rot(lg['quat'][:, b])
    n = len(ref['t']) - 1
    idx = np.unique(np.minimum(np.arange(0, n + step, step), n))
    names = [s[0] for s in SCHED]
    fig = plt.figure(figsize=(7.5, 7))
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(*HOME, marker='^', color='green', s=80)
    lo, hi = P.min(0) - 1.5, P.max(0) + 1.5
    ax.set_xlim(min(lo[0], -2), max(hi[0], 2))
    ax.set_ylim(min(lo[1], -2), max(hi[1], 2))
    ax.set_zlim(lo[2], hi[2])
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_zlabel('z (m)')
    tr, = ax.plot([], [], [], color='grey', lw=1.0)
    axl = [ax.plot([], [], [], color=col, lw=2.5)[0] for col in ('red', 'green', 'blue')]

    def update(f):
        i = idx[f]
        j0 = max(0, i - trail)
        tr.set_data(P[j0:i + 1, 0], P[j0:i + 1, 1])
        tr.set_3d_properties(P[j0:i + 1, 2])
        for k, a in enumerate(axl):
            e = np.vstack([P[i], P[i] + 1.5 * R[i][:, k]])
            a.set_data(e[:, 0], e[:, 1])
            a.set_3d_properties(e[:, 2])
        ax.set_title(f"S091 geometric SO(3)  t = {ref['t'][i]:.2f} s\n"
                     f"{names[ref['seg'][i]]}")
        return [tr] + axl

    anim = animation.FuncAnimation(fig, update, frames=len(idx), interval=80, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    anim.save(path, writer='pillow', fps=12)
    plt.close(fig)
    print(f'Saved: {path} ({len(idx)} frames)')


# ── Main ──────────────────────────────────────────────────────────────────────
if __name__ == '__main__':
    ref, lg, cases, params, thrust_cmd, n_sat = run_simulation()
    out = os.path.normpath(OUTPUT_DIR)
    os.makedirs(out, exist_ok=True)
    M = {c: metrics(ref, lg, i) for i, c in enumerate(CTRLS)}
    R_rob = {c: [metrics(ref, lg, b) for b, (cc, s) in enumerate(cases)
                 if cc == c and s is not None] for c in CTRLS}
    print('\n=== S091 Aerial Acrobatic Maneuvers - Results ===')
    print(f'  Card rotor check: k_f = {KF_CARD:.2e} N s^2 at omega_max = {OMEGA_MAX_CARD:.0f} '
          f'rad/s -> max total thrust {4 * KF_CARD * OMEGA_MAX_CARD ** 2:.2e} N vs weight '
          f'{MASS * G:.3f} N (ratio {4 * KF_CARD * OMEGA_MAX_CARD ** 2 / (MASS * G):.1e})')
    print(f'  Plant: m = {MASS} kg, I = {INERTIA.tolist()} kg m^2, arm {ARM} m, fpv_5inch rotors '
          f'(k_f = {params.kf:.3e}, omega_max = {params.w_max:.0f} rad/s), T/W = {params.t2w:.2f}'
          f', motor tau {params.tau_motor * 1000:.0f} ms, dt = {DT} s')
    print(f'  Inverted hold: free-fall drop over {T_INV[1]:.1f} s = '
          f'{0.5 * G * T_INV[1] ** 2:.1f} m; minimum possible altitude band for a '
          f'{T_INV[1]:.1f} s zero-thrust arc = g T^2 / 8 = {G * T_INV[1] ** 2 / 8:.2f} m')
    for (label, _, _), pk in zip(MANOEUVRES, ['pop_1', 'pop_2', 'pop_3', 'pop_4']):
        tb, vu = T_BALLS[pk], pop_speed(T_BALLS[pk])
        print(f'  Pop-up before {label}: {A_POP / G:.0f} g for {vu / A_POP:.3f} s -> v_up = '
              f'{vu:.2f} m/s; ballistic reference over {tb:.1f} s, apex '
              f'{0.5 * vu ** 2 / A_POP + vu ** 2 / (2 * G):.2f} m above home, back at home '
              f'altitude with {vu - G * tb:.2f} m/s')
    wr = np.linalg.norm(ref['w'], axis=1)
    print(f"  Reference peak body rate {wr.max():.2f} rad/s; sequence length {ref['t'][-1]:.2f} s")
    for c in CTRLS:
        m = M[c]
        if np.isfinite(m['t_crash']):
            print(f"  [{LABEL[c]}] before the crash: RMS |e_R| {m['eR_rms']:.3f} rad, RMS "
                  f"geodesic error {m['ang_rms']:.2f} deg, peak |Omega| {m['W_peak']:.2f} rad/s; "
                  f"all targets met: {m['success']}")
        else:
            print(f"  [{LABEL[c]}] RMS |e_R| {m['eR_rms']:.3f} rad, RMS geodesic error "
                  f"{m['ang_rms']:.2f} deg, peak |Omega| {m['W_peak']:.2f} rad/s, peak thrust "
                  f"{m['T_peak'] / (MASS * G):.2f} mg, mean thrust in inverted hold "
                  f"{m['T_hold'] / (MASS * G):.3f} mg, z range [{m['z_min']:.2f}, "
                  f"{m['z_max']:.2f}] m, all targets met: {m['success']}")
        if np.isfinite(m['t_crash']):
            print(f"      crashed (ground contact) at t = {m['t_crash']:.2f} s")
        for label, _, _ in MANOEUVRES:
            mm = m['man'][label]
            if mm is None:
                print(f'      {label:13s}: not flown (crashed earlier)')
                continue
            print(f"      {label:13s}: delta {mm['delta']:8.3f} deg, RMS |e_R| "
                  f"{mm['eR_rms']:.3f} rad, RMS / max angle error {mm['ang_rms']:.2f} / "
                  f"{mm['ang_max']:.2f} deg, peak |Omega| {mm['W_peak']:.2f} rad/s, position "
                  f"error after 2 s pause {mm['pos_err']:.3f} m, max |z - z0| {mm['dz_max']:.2f} m, "
                  f"max |z - z_ref| {mm['dz_ref']:.2f} m")
    print(f'  Mixer desaturation steps (all {len(cases)} runs): {n_sat}')
    print(f'\n  Robustness ({N_ROB} seeds, gyro noise {GYRO_STD} rad/s, gusts {GUST_STD} m/s, '
          f'same noise for every controller):')
    for c in CTRLS:
        rr = R_rob[c]
        crashes = sum(np.isfinite(r['t_crash']) for r in rr)
        if crashes == len(rr):
            tc = [r['t_crash'] for r in rr]
            print(f"  [{LABEL[c]}] all targets met {sum(r['success'] for r in rr)}/{N_ROB}; "
                  f"crashed in {crashes}/{N_ROB} runs, at t = {min(tc):.2f}-{max(tc):.2f} s")
            continue
        d = np.array([[mget(r, m[0], 'delta') for m in MANOEUVRES] for r in rr])
        pe = np.array([[mget(r, m[0], 'pos_err') for m in MANOEUVRES] for r in rr])
        print(f"  [{LABEL[c]}] all targets met {sum(r['success'] for r in rr)}/{N_ROB}; crashed "
              f"{crashes}/{N_ROB}; delta mean per manoeuvre (deg) "
              + ', '.join(f'{x:.3f}' for x in np.nanmean(d, 0))
              + f"; worst delta {np.nanmax(d):.3f} deg; worst recovery error {np.nanmax(pe):.3f} m"
              f"; RMS geodesic error {np.mean([r['ang_rms'] for r in rr]):.3f} +- "
              f"{np.std([r['ang_rms'] for r in rr]):.3f} deg")
    for c in ('quat', 'geom'):
        pe2 = [r['man']['Inverted hold']['pos_err'] for r in R_rob[c]]
        pe4 = [r['man']['Inverted hold']['pos_err_end'] for r in R_rob[c]]
        print(f"  [{LABEL[c]}] position error after the inverted hang: {np.mean(pe2):.2f} +- "
              f"{np.std(pe2):.2f} m at 2 s (<= 0.5 m on {sum(p <= POS_OK for p in pe2)}/{N_ROB}),"
              f" {np.mean(pe4):.2f} +- {np.std(pe4):.2f} m at 4 s (<= 0.5 m on "
              f"{sum(p <= POS_OK for p in pe4)}/{N_ROB}); nominal run "
              f"{M[c]['man']['Inverted hold']['pos_err']:.3f} / "
              f"{M[c]['man']['Inverted hold']['pos_err_end']:.3f} m")
    gq = sum(R_rob['geom'][s]['ang_rms'] < R_rob['quat'][s]['ang_rms'] for s in range(N_ROB))
    print(f'  Geometric has lower RMS geodesic error than quaternion PD on {gq}/{N_ROB} seeds')
    rec = recovery_test()
    print('\n  Large-error recovery (desired level, thrust mg projected on body z, 3 s):')
    for c in CTRLS:
        print(f'  [{LABEL[c]}] ' + '; '.join(
            f"{a:.0f} deg: " + (f"within 5 deg after {rec[(c, a)]['t_rec']:.2f} s"
                                if np.isfinite(rec[(c, a)]['t_rec'])
                                else f"not recovered (final {rec[(c, a)]['final']:.1f} deg)")
            + f", peak |Omega| {rec[(c, a)]['W_peak']:.1f} rad/s"
            for a in (90.0, 135.0, 170.0, 179.0)))

    plot_trajectory_3d(ref, lg, M, out)
    plot_metrics_panel(ref, lg, M, out)
    plot_robustness(R_rob, out)
    plot_recovery(rec, out)
    save_animation(ref, lg, out)
