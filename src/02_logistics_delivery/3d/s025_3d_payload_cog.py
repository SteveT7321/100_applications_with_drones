"""
S025 3D Payload CoG Offset
==========================
A 2.3 kg quadrotor (1.5 kg airframe + 0.8 kg payload) carries a payload whose
CoG sits at the full 3D offset dr = (+0.12, +0.08, -0.06) m from the airframe
centre (the payload hangs 6 cm below the motor plane).  It flies a 5-waypoint
mission at mixed altitudes (climb, level, descent legs).  Four attitude
controllers are compared on identical runs:

  1. No compensation  -- PD on the SO(3) attitude error (roll, pitch, yaw; yaw ref 0)
  2. Static FF trim   -- the same PD plus the constant hover trim -tau_hover
                         (equivalent to trimming the attitude reference by -tau/K_P)
  3. Full 3D PID      -- PD + integrators on roll, pitch and yaw, anti-windup clips
  4. PID, no yaw I    -- ablation: integrators on roll and pitch only, to test the
                         card's claim that a yaw integrator is required when dr_z != 0

The question the 3D card asks is whether the vertical offset dr_z amplifies yaw
coupling relative to the 2D (dr_z = 0) baseline.  Every controller is therefore
run with dr_z = 0 and dr_z = -0.06 m, and a dr_z sweep 0 .. -0.15 m is added.

Plant: 6-DOF rigid body about the system CoM, full inertia tensor (airframe
inertia + payload point mass, parallel-axis theorem, so dr_z creates products of
inertia I_xz, I_yz), Euler's equation with the gyroscopic term, full ZYX
Euler-angle kinematics matrix.  Two disturbance-torque models are run:

  'free' (primary)   -- free flight about the CoM: thrust acts at the geometric
                        centre, which is offset by -r_c from the CoM
                        (r_c = m_pay/m * dr), so tau = -r_c x (T e_z).  This has no
                        yaw component and does not depend on dr_z.
  'card' (secondary) -- the card's literal formula tau = dr x (m_pay R^T g), i.e.
                        the payload-weight moment about the geometric centre
                        (test-stand model, ignores the d'Alembert term).

Body frame: right-handed x forward, y left, z up; ZYX Euler angles.  With this
convention the payload side dips for tau_x = -m_pay g dr_y, tau_y = +m_pay g dr_x
(the card prints the 2D model's signs, +0.628 / -0.942; magnitudes are equal).

The run is deterministic (no random inputs), so no Monte Carlo is needed.

Outputs (saved to outputs/02_logistics_delivery/3d/s025_3d_payload_cog/):
  trajectory_3d.png, attitude_timeseries.png, attitude_error.png,
  yaw_comparison.png, thrust_saturation.png, animation.gif

Usage:
    conda activate drones
    python src/02_logistics_delivery/3d/s025_3d_payload_cog.py
"""

import matplotlib
matplotlib.use('Agg')

import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from matplotlib.animation import FuncAnimation, PillowWriter

# ── Output directory ──────────────────────────────────────────────────────────
SCRIPT_DIR   = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, '..', '..', '..'))
OUT_DIR      = os.path.join(PROJECT_ROOT, 'outputs', '02_logistics_delivery', '3d',
                            's025_3d_payload_cog')
os.makedirs(OUT_DIR, exist_ok=True)

# ── Physical parameters (card) ────────────────────────────────────────────────
G          = 9.81
M_AIR      = 1.50                     # kg airframe
M_PAY      = 0.80                     # kg payload
M          = M_AIR + M_PAY            # 2.30 kg
I_AIR      = np.diag([0.015, 0.015, 0.025])   # kg m^2, airframe about its centre
DR_CARD    = np.array([0.12, 0.08, -0.06])    # m, payload CoG offset (body)
MU         = M_AIR * M_PAY / M        # reduced mass for the parallel-axis term

# ── Control parameters (card) ─────────────────────────────────────────────────
DT         = 1.0 / 200.0              # 200 Hz
T_SIM      = 30.0                     # s, maximum simulation time
N_STEPS    = int(round(T_SIM / DT))
KP_POS, KD_POS = 1.5, 1.8
KP_ATT, KD_ATT, KI_ATT = 8.0, 3.0, 2.0
S_MAX      = np.array([0.5, 0.5, 0.3])   # rad s, anti-windup (roll, pitch, yaw)
MAX_TILT   = np.deg2rad(25.0)
T_SAT      = 1.3 * M * G              # N, thrust saturation (29.33 N)
TAU_MAX    = 5.0                      # N m, torque command clip (from 2D)
R_REACH    = 0.15                     # m
Z_BOUNDS   = (0.2, 6.0)               # m, card altitude bounds (checked, reported)

# ── Mission (added: the card gives 5 waypoints, z in [0.3, 4.0] m, no coordinates)
START      = np.array([0.0, 0.0, 1.0])
WAYPOINTS  = np.array([
    [4.0, 0.0, 4.0],   # WP1  climb  +3.0 m
    [4.0, 4.0, 4.0],   # WP2  level
    [0.0, 4.0, 1.5],   # WP3  descent -2.5 m
    [0.0, 0.0, 3.0],   # WP4  climb  +1.5 m
    [2.0, 2.0, 0.3],   # WP5  descent -2.7 m (delivery)
])

# ── Metrics ───────────────────────────────────────────────────────────────────
T_SETTLE   = 2.0                      # s, card success window starts here
THETA_OK   = 3.0                      # deg, card success threshold
STEADY_WIN = 2.0                      # s, last 2 s = steady-hover window
SWITCH_WIN = 2.0                      # s, window after a waypoint switch (reference step)
DRZ_SWEEP  = [0.0, -0.03, -0.06, -0.09, -0.12, -0.15]

CTRLS  = ['nocomp', 'ff', 'pid', 'pid_noyawI']
LABELS = {'nocomp': 'No compensation', 'ff': 'Static FF trim',
          'pid': 'Full 3D PID', 'pid_noyawI': 'PID, no yaw integrator'}
COLORS = {'nocomp': 'red', 'ff': 'orange', 'pid': 'green', 'pid_noyawI': 'purple'}
MODELS = ['free', 'card']
MODEL_LABEL = {'free': 'free-flight model', 'card': "card's literal model"}


# ── Rigid-body helpers ────────────────────────────────────────────────────────

def inertia_about_com(dr):
    """Airframe inertia + payload point mass, about the combined CoM."""
    return I_AIR + MU * (np.dot(dr, dr) * np.eye(3) - np.outer(dr, dr))


def euler_to_R(phi, theta, psi):
    """ZYX rotation matrix body -> world."""
    cf, sf = np.cos(phi), np.sin(phi)
    ct, st = np.cos(theta), np.sin(theta)
    cp, sp = np.cos(psi), np.sin(psi)
    return np.array([
        [cp * ct, cp * st * sf - sp * cf, cp * st * cf + sp * sf],
        [sp * ct, sp * st * sf + cp * cf, sp * st * cf - cp * sf],
        [-st,     ct * sf,                ct * cf]])


def euler_rate_matrix(phi, theta):
    """Full ZYX kinematics: eta_dot = W(phi, theta) omega."""
    sf, cf = np.sin(phi), np.cos(phi)
    tt, ct = np.tan(theta), np.cos(theta)
    return np.array([[1.0, sf * tt, cf * tt],
                     [0.0, cf, -sf],
                     [0.0, sf / ct, cf / ct]])


def vee(S):
    return np.array([S[2, 1], S[0, 2], S[1, 0]])


def hover_torque(dr):
    """Disturbance torque at hover (T = m g, level) -- identical in both models."""
    return np.array([-M_PAY * G * dr[1], M_PAY * G * dr[0], 0.0])


def disturbance_torque(model, dr, R, thrust):
    if model == 'free':
        r_c = (M_PAY / M) * dr
        return -np.cross(r_c, np.array([0.0, 0.0, thrust]))
    # card: payload weight moment about the geometric centre, body frame
    f_body = M_PAY * (R.T @ np.array([0.0, 0.0, -G]))
    return np.cross(dr, f_body)


# ── Simulation of one run ─────────────────────────────────────────────────────

def simulate(ctrl, model='free', dr=DR_CARD):
    I_cm = inertia_about_com(dr)
    I_inv = np.linalg.inv(I_cm)
    # Static FF trim: computed from the nominal card offset at hover.
    tau_ff = -hover_torque(DR_CARD) if ctrl == 'ff' else np.zeros(3)
    ki_mask = {'pid': np.array([1.0, 1.0, 1.0]),
               'pid_noyawI': np.array([1.0, 1.0, 0.0])}.get(ctrl, np.zeros(3))

    p = START.copy(); v = np.zeros(3)
    eta = np.zeros(3); omega = np.zeros(3); s = np.zeros(3)
    wp_idx = 0
    capture_t = []
    n = N_STEPS + 1
    log_p = np.zeros((n, 3)); log_eta = np.zeros((n, 3)); log_th = np.zeros(n)
    log_T = np.zeros(n); log_Traw = np.zeros(n); log_sat = np.zeros(n, bool)
    log_s = np.zeros((n, 3))
    log_p[0] = p
    for k in range(N_STEPS):
        target = WAYPOINTS[wp_idx]
        a_d = KP_POS * (target - p) - KD_POS * v
        b = a_d + np.array([0.0, 0.0, G])
        nb = np.linalg.norm(b)
        phi_d = np.clip(np.arcsin(np.clip(-b[1] / nb, -1, 1)), -MAX_TILT, MAX_TILT)
        theta_d = np.clip(np.arctan2(b[0], b[2]), -MAX_TILT, MAX_TILT)
        R_d = euler_to_R(phi_d, theta_d, 0.0)          # psi_ref = 0
        R = euler_to_R(*eta)

        T_raw = M * (G + a_d[2]) / (np.cos(eta[0]) * np.cos(eta[1]))
        T = float(np.clip(T_raw, 0.0, T_SAT))

        # SO(3) attitude error; eta_err = -e_R (reference minus actual)
        e_R = 0.5 * vee(R_d.T @ R - R.T @ R_d)
        eta_err = -e_R
        theta_err = np.arccos(np.clip((np.trace(R_d.T @ R) - 1.0) / 2.0, -1, 1))
        # log the state at step k with the reference active at step k
        log_eta[k] = eta; log_th[k] = theta_err
        log_T[k] = T; log_Traw[k] = T_raw; log_sat[k] = T_raw > T_SAT
        log_s[k] = s

        tau = KP_ATT * eta_err - KD_ATT * omega + tau_ff
        if ki_mask.any():
            s = np.clip(s + eta_err * DT, -S_MAX, S_MAX) * ki_mask
            tau = tau + KI_ATT * s
        tau = np.clip(tau, -TAU_MAX, TAU_MAX)

        tau_d = disturbance_torque(model, dr, R, T)
        omega_dot = I_inv @ (tau + tau_d - np.cross(omega, I_cm @ omega))
        omega = omega + omega_dot * DT
        eta = eta + euler_rate_matrix(eta[0], eta[1]) @ omega * DT
        R_new = euler_to_R(*eta)
        acc = (T / M) * R_new[:, 2] - np.array([0.0, 0.0, G])
        v = v + acc * DT
        p = p + v * DT
        log_p[k + 1] = p

        # Physical invariants
        assert np.all(np.isfinite(p)) and np.all(np.isfinite(eta)), 'non-finite state'
        assert p[2] > 0.0, f'below ground at t={(k + 1) * DT:.3f}s ({ctrl},{model})'
        assert abs(eta[1]) < np.deg2rad(80), 'pitch near gimbal lock'
        assert 0.0 <= T <= T_SAT + 1e-9

        # waypoint capture; after WP5 the drone keeps holding WP5 until T_SIM
        if len(capture_t) < len(WAYPOINTS) and np.linalg.norm(p - target) < R_REACH:
            capture_t.append((k + 1) * DT)
            if wp_idx < len(WAYPOINTS) - 1:
                wp_idx += 1
    # final sample (state only)
    log_eta[N_STEPS] = eta; log_th[N_STEPS] = log_th[N_STEPS - 1]
    log_T[N_STEPS] = log_T[N_STEPS - 1]; log_Traw[N_STEPS] = log_Traw[N_STEPS - 1]
    log_s[N_STEPS] = s
    return dict(ctrl=ctrl, model=model, drz=float(dr[2]),
                t=np.arange(n) * DT, p=log_p, eta=log_eta, theta=log_th,
                T=log_T, Traw=log_Traw, sat=log_sat, s=log_s,
                capture=capture_t,
                final_target=WAYPOINTS[wp_idx].copy())


def metrics(r):
    t = r['t']; th = np.rad2deg(r['theta']); psi = np.rad2deg(r['eta'][:, 2])
    after = t > T_SETTLE
    steady = t >= T_SIM - STEADY_WIN
    # 2 s after each waypoint switch (reference step), WP5 capture excluded
    near_switch = np.zeros_like(t, dtype=bool)
    for tc in r['capture'][:len(WAYPOINTS) - 1]:
        near_switch |= (t >= tc) & (t < tc + SWITCH_WIN)
    m = dict(
        n_wp=len(r['capture']),
        t_mission=r['capture'][-1] if len(r['capture']) == len(WAYPOINTS) else None,
        frac_ok=float(np.mean(th[after] < THETA_OK)),
        frac_ok_ex=float(np.mean(th[after & ~near_switch] < THETA_OK)),
        th_rms=float(np.sqrt(np.mean(th[after] ** 2))),
        th_peak=float(th[after].max()),
        th_steady=float(np.mean(th[steady])),
        yaw_peak=float(np.abs(psi).max()),
        yaw_rms=float(np.sqrt(np.mean(psi ** 2))),
        yaw_steady=float(np.mean(psi[steady])),
        n_sat=int(r['sat'].sum()),
        sat_first=float(t[r['sat']][0]) if r['sat'].any() else None,
        z_min=float(r['p'][:, 2].min()), z_max=float(r['p'][:, 2].max()),
        pos_err_steady=float(np.mean(np.linalg.norm(
            r['p'][steady] - r['final_target'], axis=1))),
    )
    return m


def run_simulation():
    """Run the 4 controllers x 2 models x dr_z in {0, -0.06} plus the dr_z sweep."""
    runs = {}
    for model in MODELS:
        for drz in sorted(set(DRZ_SWEEP)):
            dr = np.array([DR_CARD[0], DR_CARD[1], drz])
            for ctrl in CTRLS:
                r = simulate(ctrl, model, dr)
                r['m'] = metrics(r)
                runs[(model, round(drz, 3), ctrl)] = r
    return runs


# ── Reporting ─────────────────────────────────────────────────────────────────

def print_report(runs):
    I3 = inertia_about_com(DR_CARD); I2 = inertia_about_com(DR_CARD * [1, 1, 0])
    th = hover_torque(DR_CARD)
    print(f'Total mass {M:.2f} kg, payload {M_PAY:.2f} kg, dr = ({DR_CARD[0]:+.2f}, {DR_CARD[1]:+.2f}, {DR_CARD[2]:+.2f}) m')
    print(f'Hover disturbance torque (x-fwd/y-left/z-up body): tau_x={th[0]:+.3f} N m, '
          f'tau_y={th[1]:+.3f} N m, tau_z={th[2]:+.3f} N m')
    print(f'Static FF trim angles (-tau/K_P): roll={np.rad2deg(-th[0] / KP_ATT):+.2f} deg, '
          f'pitch={np.rad2deg(-th[1] / KP_ATT):+.2f} deg')
    print(f'Card-model pendulum stiffness m_pay g |dr_z| = {M_PAY * G * abs(DR_CARD[2]):.3f} '
          f'N m/rad (acts on roll/pitch, not yaw)')
    print(f'Thrust saturation T_sat = 1.3 m g = {T_SAT:.2f} N; mg/cos^2(25deg) = '
          f'{1 / np.cos(MAX_TILT) ** 2:.3f} mg')
    print('Inertia about CoM, dr_z = -0.06 m (kg m^2):')
    for row in I3:
        print('   [' + ', '.join(f'{x:+.5f}' for x in row) + ']')
    print(f'   products of inertia I_xz={I3[0, 2]:+.5f}, I_yz={I3[1, 2]:+.5f} '
          f'(dr_z = 0: {I2[0, 2]:+.5f}, {I2[1, 2]:+.5f})')
    print()

    for model in MODELS:
        print(f'=== Main comparison, {MODEL_LABEL[model]} ===')
        hdr = (f'{"controller":24s} {"dr_z":>6s} {"WPs":>4s} {"T_miss":>7s} '
               f'{"Th<3%":>6s} {"exSw%":>6s} {"ThRMS":>6s} {"ThPk":>6s} {"ThSS":>6s} '
               f'{"|psi|pk":>7s} {"psiRMS":>7s} {"psiSS":>7s} {"nSat":>5s} {"posSS":>6s} '
               f'{"zmin":>5s}')
        print(hdr)
        for ctrl in CTRLS:
            for drz in (0.0, -0.06):
                m = runs[(model, drz, ctrl)]['m']
                tm = f'{m["t_mission"]:.2f}' if m['t_mission'] is not None else 'timeout'
                print(f'{LABELS[ctrl]:24s} {drz:+6.2f} {m["n_wp"]:>2d}/5 {tm:>7s} '
                      f'{100 * m["frac_ok"]:5.1f}% {100 * m["frac_ok_ex"]:5.1f}% {m["th_rms"]:6.2f} {m["th_peak"]:6.2f} '
                      f'{m["th_steady"]:6.3f} {m["yaw_peak"]:7.3f} {m["yaw_rms"]:7.3f} '
                      f'{m["yaw_steady"]:+7.4f} {m["n_sat"]:5d} {m["pos_err_steady"]:6.3f} '
                      f'{m["z_min"]:5.2f}')
        print('  (angles in deg; Th = 3D attitude error, t > 2 s; exSw = Th<3% with the 2 s '
              'after each WP1-WP4 switch excluded;')
        print('   SS = mean of last 2 s; nSat = 5 ms steps with T_raw > T_sat)')
        print()
        print(f'  Waypoint capture times ({MODEL_LABEL[model]}, dr_z = -0.06):')
        for ctrl in CTRLS:
            r = runs[(model, -0.06, ctrl)]
            ct = ', '.join(f'{x:.2f}s' for x in r['capture']) if r['capture'] else 'none'
            print(f'    {LABELS[ctrl]:24s} {len(r["capture"])}/5: {ct}')
        print()
        print(f'  Yaw amplification by dr_z = -0.06 vs 0 ({MODEL_LABEL[model]}): '
              'peak|psi|(-0.06) / peak|psi|(0)')
        for ctrl in CTRLS:
            a = runs[(model, 0.0, ctrl)]['m']['yaw_peak']
            b = runs[(model, -0.06, ctrl)]['m']['yaw_peak']
            print(f'    {LABELS[ctrl]:24s} {a:.3f} -> {b:.3f} deg  (x{b / a:.2f})')
        print()
        # Yaw integrator ablation
        a = runs[(model, -0.06, 'pid')]['m']; b = runs[(model, -0.06, 'pid_noyawI')]['m']
        print(f'  Yaw-integrator ablation (dr_z=-0.06): full PID peak|psi|={a["yaw_peak"]:.3f}, '
              f'steady psi={a["yaw_steady"]:+.4f};  no yaw I peak|psi|={b["yaw_peak"]:.3f}, '
              f'steady psi={b["yaw_steady"]:+.4f} deg')
        print()

    print('=== dr_z sweep: peak |psi| (deg) / steady psi (deg) ===')
    for model in MODELS:
        for ctrl in ('ff', 'pid', 'pid_noyawI'):
            cells = '  '.join(
                f'{drz:+.2f}:{runs[(model, round(drz, 3), ctrl)]["m"]["yaw_peak"]:.3f}'
                f'/{runs[(model, round(drz, 3), ctrl)]["m"]["yaw_steady"]:+.4f}'
                for drz in DRZ_SWEEP)
            print(f'  {model:4s} {LABELS[ctrl]:24s} {cells}')
    print()
    print('=== dr_z sweep: waypoints reached / mission time ===')
    for model in MODELS:
        for ctrl in ('ff', 'pid', 'pid_noyawI'):
            cells = []
            for drz in DRZ_SWEEP:
                m = runs[(model, round(drz, 3), ctrl)]['m']
                tm = f'{m["t_mission"]:.2f}s' if m['t_mission'] is not None else 'timeout'
                cells.append(f'{drz:+.2f}:{m["n_wp"]}/5 {tm}')
            print(f'  {model:4s} {LABELS[ctrl]:24s} ' + '  '.join(cells))
    print()
    # Thrust-scaling residual of the static trim (free model)
    r = runs[('free', -0.06, 'ff')]
    ratio = r['T'] / (M * G)
    th = hover_torque(DR_CARD)
    print(f'FF trim residual (free model): T/mg range {ratio.min():.3f}..{ratio.max():.3f} '
          f'-> residual torque |(T/mg-1) tau_hover| up to '
          f'{np.abs(ratio - 1).max() * np.linalg.norm(th):.3f} N m')
    s_pid = runs[('free', -0.06, 'pid')]['s']
    print(f'PID integrator (free, dr_z=-0.06): steps at clip roll={int(np.sum(np.abs(s_pid[:, 0]) >= S_MAX[0] - 1e-12))}, '
          f'pitch={int(np.sum(np.abs(s_pid[:, 1]) >= S_MAX[1] - 1e-12))}, '
          f'yaw={int(np.sum(np.abs(s_pid[:, 2]) >= S_MAX[2] - 1e-12))}; '
          f'max integral torque K_I*s_max = {KI_ATT * S_MAX[0]:.2f} N m vs '
          f'|tau_y| up to {1.3 * abs(th[1]):.3f} N m at T_sat')
    for model in MODELS:
        cells = []
        for ctrl in CTRLS:
            for drz in (0.0, -0.06):
                rr = runs[(model, drz, ctrl)]
                ts = rr['t'][rr['sat']]
                win = f'{ts.min():.3f}-{ts.max():.3f}s' if ts.size else 'none'
                cells.append(f'{ctrl}({drz:+.2f}): {rr["m"]["n_sat"]} in {win}')
        print(f'Thrust saturation windows, {model}: ' + '; '.join(cells))
    zmins = [r['m']['z_min'] for r in runs.values()]
    zmaxs = [r['m']['z_max'] for r in runs.values()]
    print(f'Altitude over all {len(runs)} runs: z in [{min(zmins):.3f}, {max(zmaxs):.3f}] m '
          f'(card bounds {Z_BOUNDS})')


# ── Plots ─────────────────────────────────────────────────────────────────────

def plot_trajectories_3d(runs):
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    for ctrl in ('nocomp', 'ff', 'pid'):
        r = runs[('free', -0.06, ctrl)]
        p = r['p']
        ax.plot(p[:, 0], p[:, 1], p[:, 2], color=COLORS[ctrl], lw=1.6,
                label=f'{LABELS[ctrl]} ({r["m"]["n_wp"]}/5 WPs)')
    path = np.vstack([START, WAYPOINTS])
    ax.plot(path[:, 0], path[:, 1], path[:, 2], 'k--', lw=0.8, alpha=0.5, label='Ideal legs')
    ax.scatter(*START, color='royalblue', marker='s', s=80, label='Start')
    for i, wp in enumerate(WAYPOINTS):
        ax.scatter(*wp, color='gold', marker='*', s=200, edgecolor='k', linewidths=0.5,
                   label='Waypoints' if i == 0 else None)
        ax.text(wp[0] + 0.15, wp[1] + 0.15, wp[2] + 0.1, f'WP{i + 1}', fontsize=9)
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
    ax.set_title('S025 3D Payload CoG Offset — trajectories (free-flight model, '
                 'dr = (0.12, 0.08, -0.06) m)', fontsize=10)
    ax.legend(loc='upper left', fontsize=8)
    fig.tight_layout()
    out = os.path.join(OUT_DIR, 'trajectory_3d.png')
    fig.savefig(out, dpi=140); plt.close(fig)
    print(f'Saved: {os.path.relpath(out, PROJECT_ROOT)}')


def plot_attitude_timeseries(runs):
    fig, axes = plt.subplots(3, 1, figsize=(11, 10), sharex=True)
    names = ['Roll φ', 'Pitch θ', 'Yaw ψ']
    for ctrl in ('nocomp', 'ff', 'pid'):
        r = runs[('free', -0.06, ctrl)]
        for i in range(3):
            axes[i].plot(r['t'], np.rad2deg(r['eta'][:, i]), color=COLORS[ctrl], lw=1.1,
                         label=f'{LABELS[ctrl]}, dr_z=-0.06')
        r0 = runs[('free', 0.0, ctrl)]
        axes[2].plot(r0['t'], np.rad2deg(r0['eta'][:, 2]), color=COLORS[ctrl], lw=0.9,
                     ls=':', label=f'{LABELS[ctrl]}, dr_z=0 (2D offset)')
    for i in range(3):
        axes[i].set_ylabel(f'{names[i]} (deg)'); axes[i].grid(alpha=0.3)
    axes[0].legend(fontsize=8, loc='upper right')
    axes[2].legend(fontsize=7, loc='upper right', ncol=2)
    axes[2].set_xlabel('Time (s)')
    axes[0].set_title('S025 3D — Euler angles (free-flight model). Yaw panel: solid dr_z=-0.06 m, '
                      'dotted dr_z=0', fontsize=10)
    fig.tight_layout()
    out = os.path.join(OUT_DIR, 'attitude_timeseries.png')
    fig.savefig(out, dpi=140); plt.close(fig)
    print(f'Saved: {os.path.relpath(out, PROJECT_ROOT)}')


def plot_attitude_error(runs):
    fig, axes = plt.subplots(2, 1, figsize=(11, 8), sharex=True)
    for ax, model in zip(axes, MODELS):
        for ctrl in CTRLS:
            r = runs[(model, -0.06, ctrl)]
            ax.semilogy(r['t'], np.maximum(np.rad2deg(r['theta']), 1e-3), color=COLORS[ctrl],
                        lw=1.0, label=f'{LABELS[ctrl]} ({100 * r["m"]["frac_ok"]:.0f}% of t>2 s '
                                      f'below 3°)')
        ax.axhline(THETA_OK, color='k', ls='--', lw=1, label='3° threshold')
        ax.axvline(T_SETTLE, color='gray', ls=':', lw=1)
        ax.set_ylim(1e-3, 40)
        ax.set_ylabel('Θ_err (deg)'); ax.grid(alpha=0.3, which='both')
        ax.set_title(f'3D attitude error vs desired attitude, {MODEL_LABEL[model]}, dr_z = -0.06 m',
                     fontsize=10)
        ax.legend(fontsize=7, loc='lower right')
    axes[1].set_xlabel('Time (s)')
    fig.tight_layout()
    out = os.path.join(OUT_DIR, 'attitude_error.png')
    fig.savefig(out, dpi=140); plt.close(fig)
    print(f'Saved: {os.path.relpath(out, PROJECT_ROOT)}')


def plot_yaw_comparison(runs):
    fig, axes = plt.subplots(1, 3, figsize=(16, 5.2))
    groups = [('free', 0.0), ('free', -0.06), ('card', 0.0), ('card', -0.06)]
    glabels = ['free\ndr_z=0', 'free\ndr_z=-0.06', 'card\ndr_z=0', 'card\ndr_z=-0.06']
    x = np.arange(len(groups)); w = 0.2
    for key, ax, title in (('yaw_peak', axes[0], 'Peak |ψ| over the run (deg)'),
                           ('yaw_steady', axes[1], 'Final yaw ψ, mean of last 2 s (deg)')):
        for i, ctrl in enumerate(CTRLS):
            vals = [runs[(mo, dz, ctrl)]['m'][key] for mo, dz in groups]
            ax.bar(x + (i - 1.5) * w, vals, w, color=COLORS[ctrl], label=LABELS[ctrl])
        ax.set_xticks(x); ax.set_xticklabels(glabels, fontsize=8)
        ax.set_title(title, fontsize=10); ax.grid(axis='y', alpha=0.3)
        ax.axhline(0, color='k', lw=0.6)
    axes[0].legend(fontsize=7)
    ax = axes[2]
    for model, ls in (('free', '-'), ('card', '--')):
        for ctrl in ('ff', 'pid', 'pid_noyawI'):
            vals = [runs[(model, round(d, 3), ctrl)]['m']['yaw_peak'] for d in DRZ_SWEEP]
            ax.plot(DRZ_SWEEP, vals, ls=ls, marker='o', ms=4, color=COLORS[ctrl],
                    label=f'{LABELS[ctrl]}, {model}')
    ax.set_xlabel('dr_z (m)'); ax.set_ylabel('Peak |ψ| (deg)')
    ax.set_title('dr_z sweep: peak yaw (solid free-flight, dashed card model)', fontsize=10)
    ax.grid(alpha=0.3); ax.legend(fontsize=7)
    fig.suptitle('S025 3D — yaw coupling: does the vertical CoG offset dr_z amplify yaw?',
                 fontsize=11)
    fig.tight_layout()
    out = os.path.join(OUT_DIR, 'yaw_comparison.png')
    fig.savefig(out, dpi=140); plt.close(fig)
    print(f'Saved: {os.path.relpath(out, PROJECT_ROOT)}')


def plot_thrust_saturation(runs):
    fig, axes = plt.subplots(len(CTRLS), 1, figsize=(11, 9), sharex=True)
    for ax, ctrl in zip(axes, CTRLS):
        r = runs[('free', -0.06, ctrl)]; r0 = runs[('free', 0.0, ctrl)]
        ax.plot(r['t'], r['Traw'] / (M * G), color=COLORS[ctrl], lw=0.9,
                label='requested thrust T_raw / mg')
        idx = np.where(r['sat'])[0]
        if len(idx):
            ml, sl, bl = ax.stem(r['t'][idx], r['Traw'][idx] / (M * G), bottom=1.3)
            plt.setp(sl, color='k', lw=0.5); plt.setp(ml, color='k', ms=2); plt.setp(bl, visible=False)
        ax.axhline(1.3, color='k', ls='--', lw=1, label='T_sat = 1.3 mg')
        ax.set_ylabel('T / mg')
        ax.set_title(f'{LABELS[ctrl]}: {r["m"]["n_sat"]} saturated steps (dr_z=-0.06), '
                     f'{r0["m"]["n_sat"]} (dr_z=0)', fontsize=9)
        ax.grid(alpha=0.3); ax.legend(fontsize=7, loc='upper right')
    axes[-1].set_xlabel('Time (s)')
    fig.suptitle('S025 3D — thrust saturation events (free-flight model; stems = steps with '
                 'T_raw > T_sat)', fontsize=10)
    fig.tight_layout()
    out = os.path.join(OUT_DIR, 'thrust_saturation.png')
    fig.savefig(out, dpi=140); plt.close(fig)
    print(f'Saved: {os.path.relpath(out, PROJECT_ROOT)}')


ANIM_STEP = 50   # every 50th sample (0.25 s) -> ceil(6000/50) + 1 = 121 frames


def save_animation(runs):
    ctrls = ('nocomp', 'ff', 'pid')
    rs = [runs[('free', -0.06, c)] for c in ctrls]
    n = len(rs[0]['t'])
    n_frames = int(np.ceil((n - 1) / ANIM_STEP)) + 1
    fig = plt.figure(figsize=(8, 6.5))
    ax = fig.add_subplot(111, projection='3d')
    path = np.vstack([START, WAYPOINTS])
    ax.plot(path[:, 0], path[:, 1], path[:, 2], 'k--', lw=0.7, alpha=0.4)
    ax.scatter(*START, color='royalblue', marker='s', s=50)
    ax.scatter(WAYPOINTS[:, 0], WAYPOINTS[:, 1], WAYPOINTS[:, 2], color='gold', marker='*',
               s=140, edgecolor='k', linewidths=0.5)
    allp = np.vstack([r['p'] for r in rs])
    ax.set_xlim(allp[:, 0].min() - 0.5, allp[:, 0].max() + 0.5)
    ax.set_ylim(allp[:, 1].min() - 0.5, allp[:, 1].max() + 0.5)
    ax.set_zlim(0, allp[:, 2].max() + 0.5)
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
    lines, dots = [], []
    for c in ctrls:
        ln, = ax.plot([], [], [], color=COLORS[c], lw=1.4, label=LABELS[c])
        dt_, = ax.plot([], [], [], 'o', color=COLORS[c], ms=6)
        lines.append(ln); dots.append(dt_)
    ax.legend(loc='upper left', fontsize=8)
    title = ax.set_title('')

    def update(fi):
        k = min(fi * ANIM_STEP, n - 1)
        for r, ln, d in zip(rs, lines, dots):
            p = r['p']
            ln.set_data(p[:k + 1, 0], p[:k + 1, 1]); ln.set_3d_properties(p[:k + 1, 2])
            d.set_data([p[k, 0]], [p[k, 1]]); d.set_3d_properties([p[k, 2]])
        title.set_text(f'S025 3D Payload CoG (free-flight, dr_z=-0.06 m)  t = {k * DT:.2f} s')
        return lines + dots + [title]

    anim = FuncAnimation(fig, update, frames=n_frames, interval=50, blit=False)
    out = os.path.join(OUT_DIR, 'animation.gif')
    anim.save(out, writer=PillowWriter(fps=20), dpi=90)
    plt.close(fig)
    print(f'Saved: {os.path.relpath(out, PROJECT_ROOT)} ({n_frames} frames)')


if __name__ == '__main__':
    print('=' * 70)
    print('S025 3D Payload CoG Offset')
    print('=' * 70)
    runs = run_simulation()
    print_report(runs)
    print()
    plot_trajectories_3d(runs)
    plot_attitude_timeseries(runs)
    plot_attitude_error(runs)
    plot_yaw_comparison(runs)
    plot_thrust_saturation(runs)
    save_animation(runs)
    print('Done.')
