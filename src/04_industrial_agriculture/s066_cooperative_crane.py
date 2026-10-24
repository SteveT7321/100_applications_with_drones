"""
S066 Cooperative Crane Simulation
=================================
Three quadrotors carry a 2 kg, 2 m rigid beam on 1 m cables from A = (0, 0, 2) to
B = (15, 0, 2) at 0.5 m/s. The beam is a 6-DOF rigid body (quaternion attitude), the
drones are point masses whose thrust vector follows the command through a 0.05 s
first-order lag, and each cable is a stiff tension-only spring-damper. Three
controllers are compared on the same plant and the same random wind (common random
numbers):

  D  Distributed tension allocation: beam PD -> desired wrench -> minimum-norm cable
     forces (pseudo-inverse of the 6x9 grasp matrix) plus an axial internal force that
     fans the outer cables out; each drone tracks its cable direction with a local PD
     and feeds its allocated cable force forward.
  C  Centralised PID mixer: one controller turns the beam error into a collective
     force and a pitch/yaw torque and splits them over the three thrusts with a fixed
     mixer; no tension allocation and no drone-level feedback.
  U  Uncoordinated: each drone flies its own slot on the reference path with a local
     PD and a static hover trim; it never looks at the beam.

A fourth configuration D0 runs D with the card's vertical-cable formation to show that
it breaks the 0.8 m separation limit by construction.

Usage:
    conda activate drones
    python src/04_industrial_agriculture/s066_cooperative_crane.py
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401

# ── Parameters (card) ─────────────────────────────────────────────────────────
N_DRONES   = 3
M_LOAD     = 2.0            # kg  beam mass
L_BEAM     = 2.0            # m   beam length
M_DRONE    = 0.5            # kg  per drone
L_CABLE    = 1.0            # m   cable rest length
V_CRUISE   = 0.5            # m/s
DT         = 0.01           # s   control period (card timestep)
TAU_THRUST = 0.05           # s   thrust/attitude first-order lag
T_MIN      = 0.5            # N   minimum cable tension
D_MIN      = 0.8            # m   hard separation limit
D_SAFE     = 1.2            # m   soft repulsion onset
ATT_LIM    = 5.0            # deg beam pitch limit
POS_TOL    = 0.1            # m   tracking tolerance
ARRIVE_TOL = 0.05           # m   arrival tolerance at B
GRAV       = 9.81
G_VEC      = np.array([0.0, 0.0, -GRAV])
I_AXIAL    = 1e-4           # kg m^2 regularised axial inertia (card epsilon)
I_PERP     = M_LOAD * L_BEAM ** 2 / 12.0     # 2/3 kg m^2
I_BODY     = np.array([I_AXIAL, I_PERP, I_PERP])
ANCHOR_S   = np.array([-0.75, 0.0, 0.75])    # m anchor offsets along the beam axis
KP_TRANS, KD_TRANS = 4.0, 3.0
KP_ROT, KD_ROT     = 8.0, 4.0
KP_DRONE, KD_DRONE = 12.0, 5.0
K_REP      = 0.8            # m/s^2 repulsion gain
P_START    = np.array([0.0, 0.0, 2.0])
P_END      = np.array([15.0, 0.0, 2.0])
PATH_LEN   = float(np.linalg.norm(P_END - P_START))
T_CRUISE   = PATH_LEN / V_CRUISE             # 30 s

# ── Parameters (added, see README deviations) ────────────────────────────────
N_SUB      = 10             # physics sub-steps per control step (1 ms)
DT_SUB     = DT / N_SUB
T_SIM      = 40.0           # s   fixed horizon: 30 s cruise + 10 s hold at B
K_CABLE    = 2000.0         # N/m cable stiffness (tension-only)
C_CABLE    = 10.0           # N s/m cable damping
F_MAX      = 20.0           # N   max thrust per drone (~4x drone weight)
F_Z_MIN    = 0.1            # N   thrust must point upward
C_DRAG_D   = 0.2            # N s/m linear air drag, drone
C_DRAG_B   = 0.6            # N s/m linear air drag, beam
WIND_SIGMA = np.array([0.5, 0.5, 0.15])       # m/s stationary std of OU wind
WIND_TAU   = 3.0            # s   OU correlation time
N_SEEDS    = 10
M_TOT      = M_LOAD + N_DRONES * M_DRONE
I_EFF_C    = I_PERP + 2 * M_DRONE * 0.75 ** 2   # pitch/yaw inertia used by the mixer

CONFIGS = ['D', 'C', 'U', 'D0', 'C0']
LABELS = {'D': 'Distributed allocation', 'C': 'Centralised mixer + formation hold',
          'U': 'Uncoordinated', 'D0': 'Distributed, card vertical cables',
          'C0': 'Centralised mixer, beam error only (card)'}
COLORS = {'D': '#1F77B4', 'C': '#E87722', 'U': '#9467BD', 'D0': '#8C564B', 'C0': '#7F7F7F'}
DRONE_COLORS = ['#D62728', '#17BECF', '#BCBD22']
FRAME_STEP = 25             # log samples per GIF frame (0.25 s)
DIV_NAMES = ['non-finite state', '|pitch| > 20 deg', 'position error > 1 m',
             'drone speed > 4 m/s', 'cable stretch > 4 %', 'drone height < 0.3 m']

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), '..', '..', 'outputs',
                          '04_industrial_agriculture', 's066_cooperative_crane')


# ── Geometry helpers ─────────────────────────────────────────────────────────

def separation_for_fan(phi):
    """Distance between an outer drone and the middle drone for outer-cable tilt phi."""
    dx = 0.75 + L_CABLE * np.sin(phi)
    dz = L_CABLE - L_CABLE * np.cos(phi)
    return np.hypot(dx, dz)


def solve_fan_angle(d_target=D_SAFE):
    lo, hi = 0.0, np.deg2rad(80.0)
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        if separation_for_fan(mid) < d_target:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def static_setup(phi):
    """Static equilibrium for outer-cable tilt phi: tensions, cable dirs, trims, slots."""
    tv = M_LOAD * GRAV / 3.0
    n0 = np.array([[-np.sin(phi), 0.0, np.cos(phi)],
                   [0.0, 0.0, 1.0],
                   [np.sin(phi), 0.0, np.cos(phi)]])
    T0 = np.array([tv / np.cos(phi), tv, tv / np.cos(phi)])
    anchors = np.column_stack([ANCHOR_S, np.zeros(3), np.zeros(3)])
    lengths = L_CABLE + T0 / K_CABLE
    slots = anchors + lengths[:, None] * n0               # drone offset from beam CoM
    trim = T0[:, None] * n0 + M_DRONE * GRAV * np.array([0.0, 0.0, 1.0])
    lam = tv * np.tan(phi)                                # axial internal force
    return dict(n0=n0, T0=T0, slots=slots, trim=trim, lam=lam)


def quat_to_R(q):
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    R = np.empty((len(q), 3, 3))
    R[:, 0, 0] = 1 - 2 * (y * y + z * z)
    R[:, 0, 1] = 2 * (x * y - w * z)
    R[:, 0, 2] = 2 * (x * z + w * y)
    R[:, 1, 0] = 2 * (x * y + w * z)
    R[:, 1, 1] = 1 - 2 * (x * x + z * z)
    R[:, 1, 2] = 2 * (y * z - w * x)
    R[:, 2, 0] = 2 * (x * z - w * y)
    R[:, 2, 1] = 2 * (y * z + w * x)
    R[:, 2, 2] = 1 - 2 * (x * x + y * y)
    return R


def quat_omega_dot(q, om):
    """q_dot = 0.5 q (x) (0, omega_body)."""
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    p, qq, r = om[:, 0], om[:, 1], om[:, 2]
    return 0.5 * np.stack([-x * p - y * qq - z * r,
                           w * p + y * r - z * qq,
                           w * qq - x * r + z * p,
                           w * r + x * qq - y * p], axis=1)


def skew(v):
    S = np.zeros(v.shape[:-1] + (3, 3))
    S[..., 0, 1], S[..., 0, 2] = -v[..., 2], v[..., 1]
    S[..., 1, 0], S[..., 1, 2] = v[..., 2], -v[..., 0]
    S[..., 2, 0], S[..., 2, 1] = -v[..., 1], v[..., 0]
    return S


def make_wind(seed, n_steps):
    """Ornstein-Uhlenbeck wind (one vector for the whole rig), sampled at DT."""
    if seed is None:
        return np.zeros((n_steps, 3))
    rng = np.random.default_rng(seed)
    a = np.exp(-DT / WIND_TAU)
    b = np.sqrt(1 - a * a)
    w = np.empty((n_steps, 3))
    w[0] = WIND_SIGMA * rng.standard_normal(3)
    for k in range(1, n_steps):
        w[k] = a * w[k - 1] + b * WIND_SIGMA * rng.standard_normal(3)
    return w


def reference(t):
    s = min(V_CRUISE * t, PATH_LEN)
    p = P_START + s * np.array([1.0, 0.0, 0.0])
    v = np.array([V_CRUISE, 0.0, 0.0]) if t < T_CRUISE else np.zeros(3)
    return p, v


# ── Simulation (batched over configurations and seeds) ──────────────────────

def simulate(cfgs, seeds, phi_fan):
    """cfgs[b] in CONFIGS, seeds[b] an int or None (calm). Returns a log dict."""
    B = len(cfgs)
    n_steps = int(round(T_SIM / DT))
    strat = np.array([{'D': 0, 'D0': 0, 'C': 1, 'U': 2, 'C0': 3}[c] for c in cfgs])
    may_diverge = strat == 3          # only the card's beam-error-only mixer may blow up
    setups = [static_setup(0.0 if c == 'D0' else phi_fan) for c in cfgs]
    lam = np.array([s['lam'] for s in setups])
    trim = np.stack([s['trim'] for s in setups])                  # (B,3,3)
    slots = np.stack([s['slots'] for s in setups])
    T0 = np.stack([s['T0'] for s in setups])
    trim_vert = trim[:, :, 2].copy()                              # vertical trim per drone
    wind = np.stack([make_wind(s, n_steps + 1) for s in seeds], axis=1)   # (n,B,3)

    pL = np.tile(P_START, (B, 1))
    vL = np.zeros((B, 3))
    q = np.tile([1.0, 0.0, 0.0, 0.0], (B, 1))
    om = np.zeros((B, 3))
    pd = pL[:, None, :] + slots
    vd = np.zeros((B, 3, 3))
    F_act = trim.copy()
    alpha_lag = 1.0 - np.exp(-DT_SUB / TAU_THRUST)
    e_x = np.array([1.0, 0.0, 0.0])
    pairs = [(0, 1), (0, 2), (1, 2)]

    n_log = n_steps + 1
    log = {k: np.zeros((n_log, B) + s) for k, s in [
        ('pL', (3,)), ('vL', (3,)), ('eb', (3,)), ('pd', (3, 3)), ('T', (3,)),
        ('F', (3,))]}
    log['T'][0] = T0
    log['F'][0] = np.linalg.norm(F_act, axis=2)
    n_tmin_alloc = np.zeros(B, dtype=int)
    alive = np.ones(B, dtype=bool)
    t_div = np.full(B, np.nan)
    div_why = np.zeros((B, len(DIV_NAMES)), dtype=bool)
    div_state = np.full((B, 5), np.nan)
    n_sat = np.zeros(B, dtype=int)

    def kinematics():
        R = quat_to_R(q)
        eb = R[:, :, 0]
        r = ANCHOR_S[None, :, None] * eb[:, None, :]
        anch = pL[:, None, :] + r
        ww = np.einsum('bij,bj->bi', R, om)
        adot = vL[:, None, :] + np.cross(ww[:, None, :], r)
        return R, eb, r, anch, ww, adot

    def record(k, T):
        R = quat_to_R(q)
        log['pL'][k], log['vL'][k], log['eb'][k] = pL, vL, R[:, :, 0]
        log['pd'][k], log['T'][k] = pd, T

    record(0, T0)
    for k in range(n_steps):
        t = k * DT
        p_ref, v_ref = reference(t)
        R, eb, r, anch, ww, adot = kinematics()

        # Beam-level PD (shared by D and C)
        a_des = KP_TRANS * (p_ref - pL) + KD_TRANS * (v_ref - vL)
        th_err = np.cross(eb, e_x)
        w_perp = ww - np.sum(ww * eb, axis=1, keepdims=True) * eb
        al_des = KP_ROT * th_err - KD_ROT * w_perp

        # Soft repulsion (all strategies)
        a_rep = np.zeros((B, 3, 3))
        for i, j in pairs:
            diff = pd[:, i] - pd[:, j]
            d = np.linalg.norm(diff, axis=1)
            mag = np.where(d < D_SAFE, K_REP * (D_SAFE - d) / D_SAFE, 0.0) / d
            a_rep[:, i] += mag[:, None] * diff
            a_rep[:, j] -= mag[:, None] * diff

        # D: wrench -> minimum-norm cable forces via the 6x9 grasp matrix
        Iw = np.einsum('bij,j,bkj->bik', R, I_BODY, R)
        W = np.concatenate([M_LOAD * (a_des - G_VEC),
                            np.einsum('bij,bj->bi', Iw, al_des)
                            + np.cross(ww, np.einsum('bij,bj->bi', Iw, ww))], axis=1)
        Gm = np.zeros((B, 6, 9))
        for i in range(3):
            Gm[:, :3, 3 * i:3 * i + 3] = np.eye(3)
            Gm[:, 3:, 3 * i:3 * i + 3] = skew(r[:, i])
        f = np.einsum('bij,bj->bi', np.linalg.pinv(Gm, rcond=1e-8), W).reshape(B, 3, 3)
        f[:, 0] -= lam[:, None] * eb
        f[:, 2] += lam[:, None] * eb
        fn = np.linalg.norm(f, axis=2)
        low = fn < T_MIN
        n_tmin_alloc += np.any(low, axis=1) & (strat == 0)
        f = np.where(low[..., None], f * (T_MIN / fn)[..., None], f)
        fn = np.maximum(fn, T_MIN)
        p_des = anch + L_CABLE * f / fn[..., None]
        u = KP_DRONE * (p_des - pd) + KD_DRONE * (adot - vd) + a_rep
        F_D = M_DRONE * (u - G_VEC) + f

        # C0 (card): collective force + fixed pitch/yaw mixer from the beam error only
        tau = I_EFF_C * al_des
        dF = np.cross(tau, eb) / (2 * 0.75)
        F_C0 = trim_vert[..., None] * np.array([0.0, 0.0, 1.0]) \
            + (M_TOT * a_des / 3.0)[:, None, :] + M_DRONE * a_rep
        F_C0[:, 0] += -lam[:, None] * eb - dF
        F_C0[:, 2] += lam[:, None] * eb + dF
        # C: same mixer plus a rigid-formation hold (each drone PD to its fixed slot in
        # the beam frame); still no tension allocation
        slot_w = np.einsum('bij,bkj->bki', R, slots)
        p_slot_b = pL[:, None, :] + slot_w
        v_slot_b = vL[:, None, :] + np.cross(ww[:, None, :], slot_w)
        F_C = F_C0 + M_DRONE * (KP_DRONE * (p_slot_b - pd) + KD_DRONE * (v_slot_b - vd))

        # U: own slot on the reference path, static trim, no beam feedback
        p_slot = p_ref[None, None, :] + slots
        F_U = trim + M_DRONE * (KP_DRONE * (p_slot - pd)
                                + KD_DRONE * (v_ref[None, None, :] - vd) + a_rep)

        sel = strat[:, None, None]
        F_cmd = np.where(sel == 0, F_D, np.where(sel == 1, F_C,
                                                  np.where(sel == 2, F_U, F_C0)))
        F_cmd[..., 2] = np.maximum(F_cmd[..., 2], F_Z_MIN)
        fmag = np.linalg.norm(F_cmd, axis=2)
        sat = fmag > F_MAX
        n_sat += np.any(sat, axis=1) & alive
        F_cmd = np.where(sat[..., None], F_cmd * (F_MAX / fmag)[..., None], F_cmd)

        w_now = wind[k]
        saved = [x.copy() for x in (pL, vL, q, om, pd, vd, F_act)]
        for _ in range(N_SUB):
            R, eb, r, anch, ww, adot = kinematics()
            cv = pd - anch
            ln = np.linalg.norm(cv, axis=2)
            n = cv / ln[..., None]
            ldot = np.sum(n * (vd - adot), axis=2)
            T = np.where(ln > L_CABLE,
                         np.maximum(0.0, K_CABLE * (ln - L_CABLE) + C_CABLE * ldot), 0.0)
            fc = T[..., None] * n
            F_beam = fc.sum(axis=1) + M_LOAD * G_VEC + C_DRAG_B * (w_now - vL)
            tau_w = np.cross(r, fc).sum(axis=1)
            a_d = (F_act - fc + M_DRONE * G_VEC
                   + C_DRAG_D * (w_now[:, None, :] - vd)) / M_DRONE
            F_act += (F_cmd - F_act) * alpha_lag
            vd += a_d * DT_SUB
            pd += vd * DT_SUB
            vL += F_beam / M_LOAD * DT_SUB
            pL += vL * DT_SUB
            tau_b = np.einsum('bji,bj->bi', R, tau_w)
            om += (tau_b - np.cross(om, I_BODY * om)) / I_BODY * DT_SUB
            q += quat_omega_dot(q, om) * DT_SUB
            q /= np.linalg.norm(q, axis=1, keepdims=True)

        # Divergence of C0: freeze the run at its last valid state and record the time
        R, eb, r, anch, ww, adot = kinematics()
        p_ref1 = reference(t + DT)[0]
        crit = np.stack([~np.isfinite(pL).all(axis=1),
                         np.abs(eb[:, 2]) > np.sin(np.radians(20.0)),
                         np.linalg.norm(pL - p_ref1, axis=1) > 1.0,
                         np.linalg.norm(vd, axis=2).max(axis=1) > 4.0,
                         np.linalg.norm(pd - anch, axis=2).max(axis=1) > 1.04 * L_CABLE,
                         pd[..., 2].min(axis=1) < 0.3], axis=1)
        div = crit.any(axis=1)
        new_dead = div & alive & may_diverge
        t_div[new_dead] = t + DT
        div_why[new_dead] = crit[new_dead]
        div_state[new_dead] = np.stack([np.degrees(np.arcsin(np.clip(eb[:, 2], -1, 1))),
                                        np.linalg.norm(pL - p_ref1, axis=1),
                                        np.linalg.norm(vd, axis=2).max(axis=1),
                                        np.linalg.norm(pd - anch, axis=2).max(axis=1),
                                        pd[..., 2].min(axis=1)], axis=1)[new_dead]
        alive &= ~new_dead
        dead = ~alive
        if dead.any():
            for x, x0 in zip((pL, vL, q, om, pd, vd, F_act), saved):
                x[dead] = x0[dead]
            T = np.where(dead[:, None], log['T'][k], T)
            R, eb, r, anch, ww, adot = kinematics()

        # Physical invariants (X4)
        assert np.all(np.isfinite(pd)) and np.all(np.isfinite(pL))
        assert np.all(pd[..., 2] > 0.0) and np.all(anch[..., 2] > 0.0), 'below ground'
        assert np.all(np.linalg.norm(pd - anch, axis=2) < 1.05 * L_CABLE), 'cable over-stretched'
        assert np.all(np.linalg.norm(F_act, axis=2) <= F_MAX + 1e-9), 'thrust above F_MAX'
        bad = np.linalg.norm(vd, axis=2).max(axis=1) >= 5.0
        assert not bad.any(), \
            f'implausible drone speed at t={t:.2f} s in {[cfgs[b] for b in np.where(bad)[0]]}'
        record(k + 1, T)
        log['F'][k + 1] = np.linalg.norm(F_act, axis=2)

    log['t'] = np.arange(n_log) * DT
    log['n_tmin_alloc'] = n_tmin_alloc
    log['n_sat'] = n_sat
    log['t_div'] = t_div
    log['div_why'] = div_why
    log['div_state'] = div_state
    log['wind'] = wind
    return log


def compute_metrics(log, b):
    t = log['t']
    pL, eb, pd, T = log['pL'][:, b], log['eb'][:, b], log['pd'][:, b], log['T'][:, b]
    ref = np.array([reference(tt)[0] for tt in t])
    err = np.linalg.norm(pL - ref, axis=1)
    pitch = np.degrees(np.arcsin(np.clip(eb[:, 2], -1, 1)))
    yaw = np.degrees(np.arctan2(eb[:, 1], eb[:, 0]))
    seps = np.stack([np.linalg.norm(pd[:, i] - pd[:, j], axis=1)
                     for i, j in [(0, 1), (0, 2), (1, 2)]], axis=1).min(axis=1)
    Tbar = T.mean(axis=1)
    bal = 1.0 - (np.abs(T[:, 0] - T[:, 1]) + np.abs(T[:, 0] - T[:, 2])
                 + np.abs(T[:, 1] - T[:, 2])) / (3.0 * np.maximum(Tbar, 1e-9))
    dB = np.linalg.norm(pL - P_END, axis=1)
    inside = dB < ARRIVE_TOL
    t_first = float(t[np.argmax(inside)]) if inside.any() else np.nan
    outside = np.where(~inside)[0]
    t_settle = np.nan if not inside[-1] else float(t[outside[-1] + 1]) if len(outside) else 0.0
    cruise = (t >= 5.0) & (t <= T_CRUISE)
    return dict(err=err, pitch=pitch, yaw=yaw, sep=seps, bal=bal,
                final_err=float(err[-1]), max_err=float(err.max()), rms_err=float(np.sqrt(np.mean(err ** 2))),
                max_err_cruise=float(err[cruise].max()),
                rms_att=float(np.sqrt(np.mean(pitch ** 2))),
                max_pitch=float(np.abs(pitch).max()), max_yaw=float(np.abs(yaw).max()),
                min_sep=float(seps.min()), t_below_dmin=float(np.sum(seps < D_MIN) * DT),
                mean_bal=float(bal.mean()), min_T=float(T.min()),
                max_T=float(T.max()), t_below_tmin=float(np.sum(T.min(axis=1) < T_MIN) * DT),
                t_first=t_first, t_settle=t_settle,
                impulse=float(np.sum(log['F'][:, b]) * DT),
                max_F=float(log['F'][:, b].max()),
                all_ok=bool(np.abs(pitch).max() <= ATT_LIM and seps.min() >= D_MIN
                            and err.max() < POS_TOL))


def run_simulation():
    phi = solve_fan_angle(D_SAFE)
    cfgs, seeds = [], []
    for c in CONFIGS:
        for s in [None] + list(range(N_SEEDS)):
            cfgs.append(c)
            seeds.append(s)
    log = simulate(cfgs, seeds, phi)
    metrics = {}
    for b, (c, s) in enumerate(zip(cfgs, seeds)):
        metrics[(c, s)] = compute_metrics(log, b)
    index = {(c, s): b for b, (c, s) in enumerate(zip(cfgs, seeds))}
    return phi, log, metrics, index


# ── Plotting ──────────────────────────────────────────────────────────────────

def _beam_ends(pL, eb):
    return pL - 0.5 * L_BEAM * eb, pL + 0.5 * L_BEAM * eb


def plot_formation_geometry(phi, out_dir):
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.5))
    for ax, ph, title in [(axes[0], 0.0, 'Card formation: cables vertical'),
                          (axes[1], phi, f'Fanned formation: outer cables tilted '
                                         f'{np.degrees(phi):.1f} deg')]:
        st = static_setup(ph)
        ax.plot([-1, 1], [0, 0], 'k-', lw=6, solid_capstyle='butt', label='Beam')
        for i in range(3):
            a = np.array([ANCHOR_S[i], 0.0])
            d = st['slots'][i, [0, 2]]
            ax.plot(*zip(a, d), '--', color='0.4', lw=1.2)
            ax.plot(*d, 'o', color=DRONE_COLORS[i], ms=14, mec='k')
            ax.text(d[0], d[1] + 0.12, f"D{i + 1}\nT={st['T0'][i]:.2f} N", ha='center',
                    fontsize=9)
        for i, j in [(0, 1), (1, 2)]:
            pi, pj = st['slots'][i, [0, 2]], st['slots'][j, [0, 2]]
            dist = np.linalg.norm(st['slots'][i] - st['slots'][j])
            ax.annotate('', xy=pj, xytext=pi,
                        arrowprops=dict(arrowstyle='<->', color='red' if dist < D_MIN else 'g'))
            mid = 0.5 * (pi + pj)
            ax.text(mid[0], mid[1] - 0.15, f'{dist:.3f} m', ha='center', fontsize=9,
                    color='red' if dist < D_MIN else 'g')
        ax.set_xlim(-2.0, 2.0)
        ax.set_ylim(-0.3, 1.6)
        ax.set_aspect('equal')
        ax.set_xlabel('x relative to beam CoM (m)')
        ax.set_ylabel('z relative to beam CoM (m)')
        ax.set_title(title)
        ax.grid(alpha=0.3)
    fig.suptitle(f'S066 Static formation (d_min = {D_MIN} m, d_safe = {D_SAFE} m; '
                 'red = separation below d_min)')
    plt.tight_layout()
    path = os.path.join(out_dir, 'formation_geometry.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_trajectory_3d(log, b, out_dir):
    pL, eb, pd = log['pL'][:, b], log['eb'][:, b], log['pd'][:, b]
    fig = plt.figure(figsize=(12, 6))
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(*pL.T, 'k-', lw=2, label='Beam CoM')
    for i in range(3):
        ax.plot(*pd[:, i].T, color=DRONE_COLORS[i], lw=1.0, alpha=0.8, label=f'Drone {i + 1}')
    for k in np.linspace(0, int(round(T_CRUISE / DT)), 5).astype(int):
        e0, e1 = _beam_ends(pL[k], eb[k])
        ax.plot(*zip(e0, e1), 'k-', lw=4)
        for i in range(3):
            a = pL[k] + ANCHOR_S[i] * eb[k]
            ax.plot(*zip(a, pd[k, i]), '--', color='0.4', lw=1)
            ax.scatter(*pd[k, i], color=DRONE_COLORS[i], s=25, depthshade=False)
        ax.text(pL[k, 0], pL[k, 1], pL[k, 2] - 0.5, f't={log["t"][k]:.1f}s', fontsize=8)
    ax.scatter(*P_START, color='k', s=80, marker='^', label='Start A')
    ax.scatter(*P_END, color='green', s=80, marker='s', label='Goal B')
    ax.set_xlim(-1, 16)
    ax.set_ylim(-2, 2)
    ax.set_zlim(0, 4)
    ax.set_box_aspect((17, 4, 4))
    ax.set_yticks([-2, 0, 2])
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)', labelpad=8)
    ax.set_zlabel('Z (m)')
    ax.view_init(elev=18, azim=-70)
    ax.set_title('S066 3D transport, distributed allocation (wind seed 0); '
                 'beam and cables at t = 0, 7.5, 15, 22.5, 30 s')
    ax.legend(fontsize=8, loc='upper left')
    path = os.path.join(out_dir, 'trajectory_3d.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_metrics_panel(log, metrics, index, seed, out_dir, fname, title):
    t = log['t']
    fig, axes = plt.subplots(2, 3, figsize=(16, 8.5))
    for c in ['D', 'C', 'U']:
        m = metrics[(c, seed)]
        b = index[(c, seed)]
        col = COLORS[c]
        axes[0, 0].plot(t, m['pitch'], color=col, lw=1, label=f'{LABELS[c]} pitch')
        axes[0, 0].plot(t, m['yaw'], color=col, lw=1, ls=':', label=f'{LABELS[c]} yaw')
        axes[0, 1].plot(t, m['err'], color=col, lw=1, label=LABELS[c])
        axes[0, 2].plot(t, m['sep'], color=col, lw=1, label=LABELS[c])
        axes[1, 1].plot(t, m['bal'], color=col, lw=1, label=LABELS[c])
        axes[1, 2].plot(t, np.linalg.norm(log['vL'][:, b], axis=1), color=col, lw=1,
                        label=LABELS[c])
    bD = index[('D', seed)]
    for i in range(3):
        axes[1, 0].plot(t, log['T'][:, bD, i], color=DRONE_COLORS[i], lw=1, label=f'T{i + 1}')
    for v in (ATT_LIM, -ATT_LIM):
        axes[0, 0].axhline(v, color='gray', ls='--', lw=1)
    axes[0, 0].set_title('Beam pitch (solid) and yaw (dotted), deg; dashed = ±5°')
    axes[0, 1].axhline(POS_TOL, color='gray', ls='--', lw=1, label='0.1 m tolerance')
    axes[0, 1].set_title('Beam CoM position error (m)')
    axes[0, 2].axhline(D_MIN, color='red', ls=':', lw=1.5, label=f'd_min = {D_MIN} m')
    axes[0, 2].axhline(D_SAFE, color='gray', ls='--', lw=1, label=f'd_safe = {D_SAFE} m')
    axes[0, 2].set_title('Minimum inter-drone separation (m)')
    axes[1, 0].axhline(T_MIN, color='gray', ls='--', lw=1, label='T_min')
    axes[1, 0].set_title('Cable tensions (N), distributed allocation')
    axes[1, 1].set_title('Tension balance index B(t)')
    axes[1, 1].set_ylim(0, 1.05)
    axes[1, 2].axhline(V_CRUISE, color='gray', ls=':', lw=1, label='v_ref = 0.5 m/s')
    axes[1, 2].set_title('Beam speed (m/s)')
    for ax in axes.ravel():
        ax.set_xlabel('Time (s)')
        ax.grid(alpha=0.3)
        ax.legend(fontsize=7)
    fig.suptitle(title, fontsize=13)
    plt.tight_layout()
    path = os.path.join(out_dir, fname)
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_monte_carlo(metrics, out_dir):
    keys = [('max_err', 'Max position error (m)', POS_TOL),
            ('max_pitch', 'Max |pitch| (deg)', ATT_LIM),
            ('max_yaw', 'Max |yaw| (deg)', None),
            ('min_sep', 'Min separation (m)', D_MIN),
            ('mean_bal', 'Mean balance index B', None),
            ('t_settle', 'Settle time at B (s)', None)]
    fig, axes = plt.subplots(2, 3, figsize=(15, 8))
    cs = ['D', 'C', 'U', 'D0']
    for ax, (k, lab, lim) in zip(axes.ravel(), keys):
        for j, c in enumerate(cs):
            vals = np.array([metrics[(c, s)][k] for s in range(N_SEEDS)])
            ax.scatter(np.full(N_SEEDS, j) + np.linspace(-0.15, 0.15, N_SEEDS), vals,
                       color=COLORS[c], s=18, zorder=3)
            ok = np.isfinite(vals)
            if ok.any():
                ax.errorbar(j + 0.3, vals[ok].mean(), yerr=vals[ok].std(), fmt='D',
                            color='k', ms=5, capsize=4)
        if lim is not None:
            ax.axhline(lim, color='red', ls=':', lw=1.5)
        ax.set_xticks(range(len(cs)))
        ax.set_xticklabels(['D', 'C', 'U', 'D0\n(vertical)'])
        ax.set_title(lab)
        ax.grid(alpha=0.3)
    fig.suptitle(f'S066 Monte Carlo over {N_SEEDS} wind seeds (dots = seeds, black = mean ± std, '
                 'red dotted = card limit; unsettled runs omitted)', fontsize=12)
    plt.tight_layout()
    path = os.path.join(out_dir, 'monte_carlo.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def save_animation(log, index, seed, out_dir):
    t = log['t']
    n = len(t)
    n_frames = int(np.ceil((n - 1) / FRAME_STEP)) + 1
    cs = ['D', 'C', 'U']
    fig = plt.figure(figsize=(13, 4.6))
    axes = [fig.add_subplot(1, 3, j + 1, projection='3d') for j in range(3)]
    fig.subplots_adjust(left=0.0, right=1.0, bottom=0.02, top=0.86, wspace=0.0)

    def update(fi):
        k = min(fi * FRAME_STEP, n - 1)
        for ax, c in zip(axes, cs):
            b = index[(c, seed)]
            ax.cla()
            pL, eb, pd = log['pL'][k, b], log['eb'][k, b], log['pd'][k, b]
            e0, e1 = _beam_ends(pL, eb)
            ax.plot(*zip(e0, e1), 'k-', lw=4)
            trail = log['pL'][:k + 1:5, b]
            ax.plot(*trail.T, 'k:', lw=1)
            ref = reference(t[k])[0]
            ax.scatter(*ref, color='green', marker='x', s=40)
            for i in range(3):
                a = pL + ANCHOR_S[i] * eb
                ax.plot(*zip(a, pd[i]), '--', color='0.4', lw=1)
                ax.scatter(*pd[i], color=DRONE_COLORS[i], s=40, depthshade=False)
            ax.set_xlim(pL[0] - 2, pL[0] + 2)
            ax.set_ylim(-2, 2)
            ax.set_zlim(1, 4)
            ax.set_box_aspect((1, 1, 0.75))
            ax.view_init(elev=15, azim=-60)
            ax.set_xlabel('X (m)', fontsize=8)
            ax.set_ylabel('Y (m)', fontsize=8)
            ax.tick_params(labelsize=7)
            err = np.linalg.norm(pL - ref)
            ax.set_title(f'{LABELS[c]}\nerror {err:.3f} m, pitch '
                         f'{np.degrees(np.arcsin(eb[2])):+.2f}°', fontsize=9)
        fig.suptitle(f'S066 cooperative crane, wind seed {seed} — frame {fi + 1}/{n_frames}, '
                     f't = {t[k]:.2f} s (green x = reference)', fontsize=10)
        return []

    anim = animation.FuncAnimation(fig, update, frames=n_frames, interval=50, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    anim.save(path, writer='pillow', fps=20, dpi=58)
    plt.close(fig)
    print(f'Saved: {path} ({n_frames} frames)')


def plot_c0_divergence(log, metrics, index, out_dir):
    """Card's beam-error-only mixer (C0) vs the mixer with formation hold (C), calm air."""
    t = log['t']
    bC0 = index[('C0', None)]
    td = log['t_div'][bC0]
    end = int(round(td / DT)) if np.isfinite(td) else len(t) - 1
    fig, axes = plt.subplots(1, 3, figsize=(16, 4.5))
    for c, k_end in [('C', min(len(t) - 1, int(round(6.0 / DT)))), ('C0', end)]:
        m = metrics[(c, None)]
        b = index[(c, None)]
        sl = slice(0, k_end + 1)
        axes[0].plot(t[sl], m['pitch'][sl], color=COLORS[c], label=LABELS[c])
        axes[1].plot(t[sl], m['err'][sl], color=COLORS[c], label=LABELS[c])
        rel = log['pd'][sl, b, 0, 0] - (log['pL'][sl, b, 0] + ANCHOR_S[0] * log['eb'][sl, b, 0])
        axes[2].plot(t[sl], rel, color=COLORS[c], label=LABELS[c])
    axes[0].axhline(ATT_LIM, color='gray', ls='--')
    axes[0].axhline(-ATT_LIM, color='gray', ls='--')
    axes[0].set_title('Beam pitch (deg)')
    axes[1].axhline(POS_TOL, color='gray', ls='--')
    axes[1].set_title('Beam position error (m)')
    axes[2].set_title('Drone 1 x-offset from its anchor (m)')
    for ax in axes:
        if np.isfinite(td):
            ax.axvline(td, color='red', ls=':', lw=1.5)
        ax.set_xlabel('Time (s)')
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
    fig.suptitle(f'S066 Card centralised mixer without drone feedback (C0) diverges at '
                 f't = {td:.2f} s in calm air (red dotted); first 6 s shown for C', fontsize=12)
    plt.tight_layout()
    path = os.path.join(out_dir, 'c0_divergence.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


# ── Main ─────────────────────────────────────────────────────────────────────

def _fmt(vals, digits=3):
    v = np.asarray(vals, dtype=float)
    ok = np.isfinite(v)
    if not ok.any():
        return 'n/a'
    return f'{v[ok].mean():.{digits}f} ± {v[ok].std():.{digits}f}'


if __name__ == '__main__':
    phi, log, metrics, index = run_simulation()
    st = static_setup(phi)
    st0 = static_setup(0.0)
    print('=== S066 Cooperative Crane ===')
    print(f'Card formation (vertical cables): drone separation '
          f'{separation_for_fan(0.0):.3f} m (d_min = {D_MIN} m)')
    print(f'Fan angle for separation = d_safe: {np.degrees(phi):.2f} deg; '
          f'static tensions {st["T0"][0]:.3f}/{st["T0"][1]:.3f}/{st["T0"][2]:.3f} N '
          f'(vertical: {st0["T0"][1]:.3f} N each); axial internal force {st["lam"]:.3f} N')
    print(f'Static thrust per drone: outer {np.linalg.norm(st["trim"][0]):.3f} N, '
          f'middle {np.linalg.norm(st["trim"][1]):.3f} N (F_max = {F_MAX} N)')
    bal0 = 1 - 2 * abs(st['T0'][0] - st['T0'][1]) / (3 * st['T0'].mean())
    print(f'Static balance index B: fanned {bal0:.4f}, vertical 1.0000')
    print(f'Mixer inertia I_eff = {I_EFF_C:.4f} kg m^2 (beam {I_PERP:.4f})')
    z_ideal = KD_TRANS / (2 * np.sqrt(KP_TRANS))
    wd = np.sqrt(KP_TRANS) * np.sqrt(1 - z_ideal ** 2)
    tpk = np.arctan2(wd, z_ideal * np.sqrt(KP_TRANS)) / wd
    e_pk = V_CRUISE / wd * np.exp(-z_ideal * np.sqrt(KP_TRANS) * tpk) * np.sin(wd * tpk)
    print(f'Ideal double integrator with the card PD and a 0.5 m/s step reference: '
          f'peak error {e_pk:.4f} m at t = {tpk:.3f} s')

    keys = ['final_err', 'max_err', 'max_err_cruise', 'rms_err', 'max_pitch', 'rms_att', 'max_yaw',
            'min_sep', 't_below_dmin', 'mean_bal', 'min_T', 'max_T', 't_below_tmin',
            't_first', 't_settle', 'impulse', 'max_F']
    MAIN = ['D', 'C', 'U', 'D0']
    print('\n-- Calm air (no wind) --')
    for c in MAIN:
        m = metrics[(c, None)]
        print(f'[{c}] {LABELS[c]}')
        print('    ' + ', '.join(f'{k}={m[k]:.4f}' for k in keys) + f', all_ok={m["all_ok"]}')
    print(f'\n-- Monte Carlo, {N_SEEDS} wind seeds (mean ± std) --')
    for c in MAIN:
        print(f'[{c}] {LABELS[c]}')
        for k in keys:
            vals = [metrics[(c, s)][k] for s in range(N_SEEDS)]
            n_nan = int(np.sum(~np.isfinite(vals)))
            extra = f'  (NaN in {n_nan} runs)' if n_nan else ''
            print(f'    {k:15s} {_fmt(vals, 4)}{extra}')
        n_ok = sum(metrics[(c, s)]['all_ok'] for s in range(N_SEEDS))
        n_p = sum(metrics[(c, s)]['max_pitch'] <= ATT_LIM for s in range(N_SEEDS))
        n_s = sum(metrics[(c, s)]['min_sep'] >= D_MIN for s in range(N_SEEDS))
        n_e = sum(metrics[(c, s)]['max_err'] < POS_TOL for s in range(N_SEEDS))
        print(f'    seeds meeting pitch <= 5 deg: {n_p}/{N_SEEDS}, separation >= 0.8 m: '
              f'{n_s}/{N_SEEDS}, error < 0.1 m: {n_e}/{N_SEEDS}, all three: {n_ok}/{N_SEEDS}')
    for c in MAIN:
        bs = [index[(c, s)] for s in [None] + list(range(N_SEEDS))]
        print(f'[{c}] thrust saturation steps {int(log["n_sat"][bs].sum())}, '
              f'allocation T_min clamp steps {int(log["n_tmin_alloc"][bs].sum())}')
    allm = [metrics[(c, s)] for c in MAIN for s in [None] + list(range(N_SEEDS))]
    print(f'All D/C/U/D0 runs: peak thrust {max(m["max_F"] for m in allm):.3f} N, '
          f'min cable tension {min(m["min_T"] for m in allm):.3f} N, static outer-cable '
          f'stretch {1000 * st["T0"][0] / K_CABLE:.2f} mm')
    print(f'Position stiffness: U drones on absolute path points '
          f'{N_DRONES * M_DRONE * KP_DRONE:.1f} N/m; beam loop of D '
          f'{M_LOAD * KP_TRANS:.1f} N/m')
    td = [log['t_div'][index[('C0', s)]] for s in [None] + list(range(N_SEEDS))]
    print(f'[C0] {LABELS["C0"]}: diverged in {int(np.sum(np.isfinite(td)))}/{N_SEEDS + 1} runs; '
          f'calm divergence t = {td[0]:.2f} s; wind seeds t = {_fmt(td[1:], 2)} s '
          f'(min {np.nanmin(td[1:]):.2f}, max {np.nanmax(td[1:]):.2f})')
    print('[C0] stop criteria: ' + '; '.join(DIV_NAMES))
    for s in [None] + list(range(N_SEEDS)):
        b = index[('C0', s)]
        why = [n for n, f in zip(DIV_NAMES, log['div_why'][b]) if f]
        p, e, v, c, z = log['div_state'][b]
        print(f'    {"calm" if s is None else f"seed {s}"}: stop at t = {log["t_div"][b]:.2f} s '
              f'by [{", ".join(why)}]; pitch {p:+.1f} deg, error {e:.3f} m, max drone speed '
              f'{v:.2f} m/s, max cable length {c:.4f} m, min drone height {z:.2f} m')

    print('\n-- Per-seed wins among D/C/U (lower is better except min_sep, mean_bal) --')
    higher = {'min_sep', 'mean_bal'}
    for k in ['max_err', 'max_err_cruise', 'rms_err', 'max_pitch', 'rms_att', 'max_yaw',
              'min_sep', 'mean_bal', 'impulse']:
        wins = {c: 0 for c in ['D', 'C', 'U']}
        for s in range(N_SEEDS):
            vals = {c: metrics[(c, s)][k] for c in wins}
            best = max(vals, key=vals.get) if k in higher else min(vals, key=vals.get)
            wins[best] += 1
        print(f'    {k:15s} ' + ', '.join(f'{c}: {w}' for c, w in wins.items()))
    print('\n-- Seed 0 per strategy --')
    for c in MAIN:
        m = metrics[(c, 0)]
        print(f'[{c}] max_err={m["max_err"]:.4f} m, max_err_cruise={m["max_err_cruise"]:.4f} m, '
              f'max_pitch={m["max_pitch"]:.3f} deg, max_yaw={m["max_yaw"]:.3f} deg, '
              f'min_sep={m["min_sep"]:.4f} m, mean_bal={m["mean_bal"]:.4f}')
    w = log['wind'][:, index[('D', 0)]]
    print(f'Wind seed 0: horizontal speed mean {np.linalg.norm(w[:, :2], axis=1).mean():.3f} '
          f'm/s, max {np.linalg.norm(w[:, :2], axis=1).max():.3f} m/s')

    out = os.path.normpath(OUTPUT_DIR)
    os.makedirs(out, exist_ok=True)
    plot_formation_geometry(phi, out)
    plot_trajectory_3d(log, index[('D', 0)], out)
    plot_metrics_panel(log, metrics, index, 0, out, 'metrics_panel.png',
                       'S066 Cooperative crane, wind seed 0')
    plot_metrics_panel(log, metrics, index, None, out, 'metrics_panel_calm.png',
                       'S066 Cooperative crane, calm air')
    plot_monte_carlo(metrics, out)
    plot_c0_divergence(log, metrics, index, out)
    save_animation(log, index, 0, out)
