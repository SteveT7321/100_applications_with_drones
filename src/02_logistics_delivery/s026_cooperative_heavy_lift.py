"""
S026 Cooperative Heavy Lift
============================
Four quadrotors lift a 0.8 kg load through four cables (rest length 0.5 m),
carry it from (0, 0, 0) to (3, 2, 4) m and set it down at (3, 2, 0) m in 20 s.

Physical model (integrated at 1 kHz, semi-implicit Euler)
---------------------------------------------------------
* Drones: point masses, m_d p_i'' = u_i - m_d g z - T_i c_i
  (u_i = thrust vector, |u_i| <= T_MAX; c_i = unit vector attach -> drone).
* Cables: stiff spring-dampers that can only pull,
  T_i = max(0, k_c (L_i - l_c) + c_c dL_i/dt) when L_i > l_c, else 0 (slack).
* Load: non-rotating point mass, m_L q'' = sum_i T_i c_i - m_L g z + F_gust
  + ground contact (spring-damper floor at z = 0).

Controller (100 Hz, identical for both strategies)
--------------------------------------------------
* Desired force on the load: W = m_L (g z + a_ref) - F_gust
  (the gust is assumed measured, i.e. an ideal disturbance feed-forward).
* Tension allocation from the measured cable directions, A f = W
  (A = 3x4 matrix of cable unit vectors):
    1. Pseudo-inverse : f = A^T (A A^T)^-1 W        (minimum-norm, may be < 0)
    2. Anti-slack QP  : min ||f||^2  s.t.  A f = W,  f >= f_min  (SLSQP)
* Per-drone PD on its formation slot + tension feed-forward:
    u_i = m_d (a_ref + Kp (q_ref + d_i - p_i) + Kd (v_ref - v_i) + g z)
          + f_i c_i

A lateral gust on the load during cruise is the test that separates the two
allocations; a gust sweep shows where they are identical, where only the QP
keeps every cable taut, and where neither can.

Usage:
    conda activate drones
    python src/02_logistics_delivery/s026_cooperative_heavy_lift.py
"""

import os
import warnings
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401
from matplotlib.animation import FuncAnimation
from scipy.optimize import minimize, lsq_linear

# SLSQP clips trial points to the bounds; harmless, silence the warning
warnings.filterwarnings('ignore', message='Values in x were outside bounds')

# ── Output directory ──────────────────────────────────────────────────────────
OUTPUT_DIR = os.path.normpath(os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '02_logistics_delivery', 's026_cooperative_heavy_lift'
))
os.makedirs(OUTPUT_DIR, exist_ok=True)

# ── Physical constants ────────────────────────────────────────────────────────
G             = 9.81    # m/s²
N_DRONES      = 4
MASS_LOAD     = 0.8     # kg
MASS_DRONE    = 0.5     # kg  (a 27 g Crazyflie cannot pull a ~2.5 N cable)
T_MAX         = 15.0    # N   max thrust per drone (thrust/weight 3.06 bare)
CABLE_LENGTH  = 0.5     # m   cable rest length l_c
K_CABLE       = 1000.0  # N/m cable stiffness (2.5 mm stretch at 2.45 N)
C_CABLE       = 5.0     # N·s/m cable damping
ATTACH_R      = 0.15    # m   attachment radius on the load
FORMATION_H   = 0.40    # m   vertical cable projection h
FORMATION_S   = ATTACH_R + np.sqrt(CABLE_LENGTH**2 - FORMATION_H**2)  # 0.45 m
COS_A0        = FORMATION_H / CABLE_LENGTH                            # 0.8
SIN_A0        = np.sqrt(1.0 - COS_A0**2)                              # 0.6
F_MIN         = 0.05    # N   minimum tension (anti-slack)
KP            = 4.0     # 1/s²  PD position gain
KD            = 2.0     # 1/s   PD derivative gain
DT            = 0.01    # s   control period
N_SUB         = 10      # physics sub-steps per control step (1 kHz)
DT_PHYS       = DT / N_SUB
T_TOTAL       = 20.0    # s
K_GROUND      = 5000.0  # N/m floor stiffness (load)
C_GROUND      = 50.0    # N·s/m floor damping (also used as lateral friction)

# Gust on the load (+x), raised-cosine ramps
GUST_T0, GUST_T1, GUST_RAMP = 9.0, 11.0, 0.25   # s
GUST_MAIN     = 4.0     # N  gust used for the detailed plots
GUST_SWEEP    = [0.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0]   # N

# Analytic gust window for the symmetric hover geometry
F0_STATIC     = MASS_LOAD * G / (N_DRONES * COS_A0)          # 2.4525 N
GUST_PI_NEG   = 2 * SIN_A0 * F0_STATIC                       # PI f1 < 0
GUST_PI_SLACK = 2 * SIN_A0 * (F0_STATIC - F_MIN)             # PI f1 < f_min
GUST_QP_MAX   = 2 * SIN_A0 * (2 * F0_STATIC - 2 * F_MIN)     # QP infeasible

# Load cable attachment points (world-aligned, load does not rotate)
B_ATTACH = np.array([
    [+ATTACH_R, 0.0, 0.0],
    [-ATTACH_R, 0.0, 0.0],
    [0.0, +ATTACH_R, 0.0],
    [0.0, -ATTACH_R, 0.0],
])

# Drone formation offsets relative to the load centroid
D_OFFSETS = np.array([
    [+FORMATION_S, 0.0, FORMATION_H],
    [-FORMATION_S, 0.0, FORMATION_H],
    [0.0, +FORMATION_S, FORMATION_H],
    [0.0, -FORMATION_S, FORMATION_H],
])

DRONE_COLORS = ['red', 'orange', 'magenta', 'gold']
DRONE_LABELS = [f'Drone {i+1}' for i in range(N_DRONES)]
ZHAT = np.array([0.0, 0.0, 1.0])

# ── Trajectory waypoints (lift, cruise, descend) ─────────────────────────────
WAYPOINTS = [
    (0.0,  5.0,  np.array([0.0, 0.0, 0.0]),  np.array([0.0, 0.0, 4.0])),
    (5.0,  15.0, np.array([0.0, 0.0, 4.0]),  np.array([3.0, 2.0, 4.0])),
    (15.0, 20.0, np.array([3.0, 2.0, 4.0]),  np.array([3.0, 2.0, 0.0])),
]


def load_ref(t):
    """Reference load position, velocity, acceleration via quintic blend."""
    for (t0, t1, q0, q1) in WAYPOINTS:
        if t <= t1:
            tau = np.clip((t - t0) / (t1 - t0), 0.0, 1.0)
            dur = t1 - t0
            s   = 10*tau**3 - 15*tau**4 + 6*tau**5
            ds  = (30*tau**2 - 60*tau**3 + 30*tau**4) / dur
            dds = (60*tau - 180*tau**2 + 120*tau**3) / dur**2
            dq  = q1 - q0
            return q0 + s*dq, ds*dq, dds*dq
    return WAYPOINTS[-1][3].copy(), np.zeros(3), np.zeros(3)


def gust_force(t, amp):
    """Lateral (+x) gust force on the load with raised-cosine on/off ramps."""
    if amp == 0.0 or t <= GUST_T0 or t >= GUST_T1:
        return np.zeros(3)
    if t < GUST_T0 + GUST_RAMP:
        w = 0.5 * (1 - np.cos(np.pi * (t - GUST_T0) / GUST_RAMP))
    elif t > GUST_T1 - GUST_RAMP:
        w = 0.5 * (1 - np.cos(np.pi * (GUST_T1 - t) / GUST_RAMP))
    else:
        w = 1.0
    return np.array([amp * w, 0.0, 0.0])


# ── Tension allocation ────────────────────────────────────────────────────────

def cable_geometry(p, q):
    """Cable vectors attach -> drone: unit vectors (4,3) and lengths (4,)."""
    d = p - (q + B_ATTACH)
    L = np.linalg.norm(d, axis=1)
    return d / L[:, None], L


def solve_pseudoinverse(A, W):
    """Minimum-norm solution of the 3x4 force balance A f = W."""
    return A.T @ np.linalg.solve(A @ A.T, W)


def solve_qp(A, W, f_warm):
    """Anti-slack QP; returns (f, feasible). Falls back to bounded LSQ."""
    res = minimize(
        fun=lambda f: float(f @ f), x0=np.maximum(f_warm, F_MIN),
        jac=lambda f: 2.0 * f,
        constraints=[{'type': 'eq', 'fun': lambda f: A @ f - W,
                      'jac': lambda f: A}],
        bounds=[(F_MIN, None)] * N_DRONES, method='SLSQP',
        options={'ftol': 1e-12, 'maxiter': 200})
    if res.success and np.linalg.norm(A @ res.x - W) < 1e-6:
        return res.x, True
    # Infeasible: closest force that keeps every cable at >= f_min
    return lsq_linear(A, W, bounds=(F_MIN, np.inf)).x, False


# ── Simulation ────────────────────────────────────────────────────────────────

def run_simulation(use_qp, gust_amp):
    """Simulate the full mission for one allocation strategy and gust size."""
    label = 'QP' if use_qp else 'Pseudo-inverse'
    steps = int(round(T_TOTAL / DT)) + 1
    time  = np.arange(steps) * DT

    h = {k: np.zeros((steps, N_DRONES)) for k in
         ('f_alloc', 'T', 'L', 'thrust')}
    h.update({'p': np.zeros((steps, N_DRONES, 3)), 'q': np.zeros((steps, 3)),
              'q_ref': np.zeros((steps, 3))})
    qp_infeasible = 0
    n_sat = 0

    q_r0, _, _ = load_ref(0.0)
    q  = q_r0.copy()                         # load starts on the floor
    vq = np.zeros(3)
    p  = q + D_OFFSETS                       # cables exactly at rest length
    v  = np.zeros((N_DRONES, 3))
    f_warm = np.full(N_DRONES, F0_STATIC)

    for k in range(steps):
        t = time[k]
        q_r, v_r, a_r = load_ref(t)
        F_g = gust_force(t, gust_amp)
        c_hat, L = cable_geometry(p, q)

        # ── Allocation (same desired load force for both strategies) ──
        A = c_hat.T                                   # 3x4
        W = MASS_LOAD * (G * ZHAT + a_r) - F_g
        if use_qp:
            f, ok = solve_qp(A, W, f_warm)
            qp_infeasible += (not ok)
            f_warm = f
        else:
            f = solve_pseudoinverse(A, W)

        # ── Drone PD + tension feed-forward ──
        a_cmd = a_r + KP * (q_r + D_OFFSETS - p) + KD * (v_r - v)
        u = MASS_DRONE * (a_cmd + G * ZHAT) + f[:, None] * c_hat
        u_norm = np.linalg.norm(u, axis=1)
        sat = u_norm > T_MAX
        n_sat += int(sat.sum())
        u[sat] *= (T_MAX / u_norm[sat])[:, None]

        # ── Record state at the start of the control step ──
        h['q_ref'][k] = q_r
        h['q'][k] = q
        h['p'][k] = p
        h['f_alloc'][k] = f
        h['L'][k] = L
        h['thrust'][k] = np.minimum(u_norm, T_MAX)
        T_rec = None

        if k == steps - 1:
            T_rec = _cable_tension(p, v, q, vq)[0]
            h['T'][k] = T_rec
            break

        # ── Physics sub-steps (1 kHz) ──
        for _ in range(N_SUB):
            T, c_hat = _cable_tension(p, v, q, vq)
            if T_rec is None:
                T_rec = T
            F_cab = T[:, None] * c_hat
            acc_d = (u - F_cab) / MASS_DRONE - G * ZHAT
            F_load = F_cab.sum(axis=0) - MASS_LOAD * G * ZHAT + F_g
            if q[2] < 0.0:                       # floor contact
                F_load[2] += max(0.0, -K_GROUND * q[2] - C_GROUND * vq[2])
                F_load[:2] -= C_GROUND * vq[:2]
            v  = v + acc_d * DT_PHYS
            p  = p + v * DT_PHYS
            vq = vq + F_load / MASS_LOAD * DT_PHYS
            q  = q + vq * DT_PHYS
        h['T'][k] = T_rec

    airborne = h['q'][:, 2] > 0.01
    h.update(label=label, time=time, gust=gust_amp, airborne=airborne,
             qp_infeasible=qp_infeasible, n_sat=n_sat,
             pos_error=np.linalg.norm(h['q'] - h['q_ref'], axis=1))
    return h


def _cable_tension(p, v, q, vq):
    """Unilateral spring-damper cable tensions and unit vectors."""
    c_hat, L = cable_geometry(p, q)
    Ldot = np.sum((v - vq) * c_hat, axis=1)
    T = np.where(L > CABLE_LENGTH,
                 K_CABLE * (L - CABLE_LENGTH) + C_CABLE * Ldot, 0.0)
    return np.maximum(T, 0.0), c_hat


def summarize(d):
    """Scalar metrics of one run (airborne samples only where noted)."""
    air = d['airborne']
    fa, T = d['f_alloc'][air], d['T'][air]
    gap = d['p'][:, :, 2].mean(axis=1) - d['q'][:, 2]
    return {
        'alloc_viol': int(np.sum(fa.min(axis=1) < F_MIN)),
        'alloc_min': fa.min(), 'alloc_max': fa.max(),
        'phys_slack': int(np.sum(T.min(axis=1) <= 0.0)),
        'phys_min': T.min(), 'phys_max': T.max(),
        'T_median': np.median(T), 'cov': T.std() / T.mean(),
        'L_min': d['L'][air].min(), 'L_max': d['L'][air].max(),
        'gap_min': gap[air].min(), 'gap_max': gap[air].max(),
        'err_rms': np.sqrt(np.mean(d['pos_error'][air]**2)),
        'err_max': d['pos_error'][air].max(),
        'err_gust': d['pos_error'][(d['time'] >= GUST_T0)
                                   & (d['time'] <= GUST_T1 + 2.0)].max(),
        'thrust_max': d['thrust'].max(),
        'n_air': int(air.sum()),
    }


# ── Plots ─────────────────────────────────────────────────────────────────────

def _gust_window(ax):
    ax.axvspan(GUST_T0, GUST_T1, color='grey', alpha=0.15,
               label=f'Gust {GUST_MAIN:.0f} N (+x)')


def plot_trajectory_3d(qp, pi):
    fig = plt.figure(figsize=(12, 8))
    ax  = fig.add_subplot(111, projection='3d')
    qr = qp['q_ref']
    ax.plot(qr[:, 0], qr[:, 1], qr[:, 2], 'k:', lw=1.2, label='Load reference')
    for data, ls in [(qp, '-'), (pi, '--')]:
        q, p, lb = data['q'], data['p'], data['label']
        ax.plot(q[:, 0], q[:, 1], q[:, 2], color='blue', ls=ls, lw=2,
                label=f'Load ({lb})')
        for i in range(N_DRONES):
            ax.plot(p[:, i, 0], p[:, i, 1], p[:, i, 2], color=DRONE_COLORS[i],
                    ls=ls, lw=1.0, alpha=0.8,
                    label=DRONE_LABELS[i] if ls == '-' else None)
    for (_, _, q0, q1) in WAYPOINTS:
        ax.scatter(*q1, c='green', s=60)
    ax.scatter(*WAYPOINTS[0][2], c='green', s=60, label='Waypoints')
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
    ax.set_title(f'S026 Cooperative Heavy Lift — 3D trajectory, gust '
                 f'{GUST_MAIN:.0f} N\n(solid = QP, dashed = pseudo-inverse)')
    ax.legend(loc='upper left', fontsize=7, ncol=2)
    plt.tight_layout()
    _save(fig, 'trajectory_3d.png')


def plot_cable_tensions(qp, pi):
    fig, axes = plt.subplots(2, 2, figsize=(14, 8), sharex=True)
    for col, data in enumerate([qp, pi]):
        for row, (key, name) in enumerate([('f_alloc', 'Allocated f_i'),
                                           ('T', 'Actual cable tension T_i')]):
            ax = axes[row, col]
            for i in range(N_DRONES):
                ax.plot(data['time'], data[key][:, i], color=DRONE_COLORS[i],
                        label=DRONE_LABELS[i], lw=1.2)
            ax.axhline(F_MIN, color='black', ls='--', lw=1,
                       label=f'f_min = {F_MIN} N')
            ax.axhline(F0_STATIC, color='grey', ls=':', lw=1,
                       label=f'static share {F0_STATIC:.3f} N')
            _gust_window(ax)
            ax.set_title(f'{name} — {data["label"]}')
            ax.set_ylabel('Tension (N)'); ax.grid(True, alpha=0.3)
            ax.set_ylim(-2.5, 9)
    axes[0, 0].legend(fontsize=7, ncol=2)
    for ax in axes[1]:
        ax.set_xlabel('Time (s)')
    plt.suptitle('S026 — Allocated vs physical cable tensions')
    plt.tight_layout()
    _save(fig, 'cable_tensions.png')


def plot_slack_margin(qp, pi):
    fig, axes = plt.subplots(2, 1, figsize=(12, 6), sharex=True)
    for data, col in [(qp, 'steelblue'), (pi, 'darkorange')]:
        axes[0].plot(data['time'], data['f_alloc'].min(axis=1) - F_MIN,
                     color=col, label=data['label'], lw=1.5)
        axes[1].plot(data['time'], data['T'].min(axis=1) - F_MIN,
                     color=col, label=data['label'], lw=1.5)
    axes[0].set_title('Allocation slack margin  min_i f_i - f_min')
    axes[1].set_title('Physical slack margin  min_i T_i - f_min  '
                      '(load on the floor near t = 0 and t = 20 s)')
    for ax in axes:
        ax.axhline(0.0, color='red', ls='--', lw=1.2, label='Slack boundary')
        _gust_window(ax)
        ax.set_ylabel('Margin (N)'); ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8)
    axes[1].set_xlabel('Time (s)')
    plt.tight_layout()
    _save(fig, 'slack_margin.png')


def plot_position_error(qp, pi):
    fig, ax = plt.subplots(figsize=(12, 4))
    for data, col in [(qp, 'steelblue'), (pi, 'darkorange')]:
        ax.plot(data['time'], data['pos_error'], color=col,
                label=data['label'], lw=1.5)
    _gust_window(ax)
    for tb in (5.0, 15.0):
        ax.axvline(tb, color='k', ls=':', lw=0.8)
    ax.text(2.5, ax.get_ylim()[1]*0.9, 'lift', ha='center')
    ax.text(10, ax.get_ylim()[1]*0.9, 'cruise', ha='center')
    ax.text(17.5, ax.get_ylim()[1]*0.9, 'descend', ha='center')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('||q - q_ref|| (m)')
    ax.set_title('S026 — Load tracking error')
    ax.legend(); ax.grid(True, alpha=0.3)
    plt.tight_layout()
    _save(fig, 'position_error.png')


def plot_cable_geometry(qp, pi):
    fig, axes = plt.subplots(1, 2, figsize=(14, 4))
    for data, col in [(qp, 'steelblue'), (pi, 'darkorange')]:
        axes[0].plot(data['time'], data['L'].max(axis=1), color=col, lw=1.3,
                     label=f'{data["label"]} max_i L_i')
        axes[0].plot(data['time'], data['L'].min(axis=1), color=col, lw=1.0,
                     ls='--', label=f'{data["label"]} min_i L_i')
        gap = data['p'][:, :, 2].mean(axis=1) - data['q'][:, 2]
        axes[1].plot(data['time'], gap, color=col, lw=1.3, label=data['label'])
    axes[0].axhline(CABLE_LENGTH, color='k', ls=':', label='rest length l_c')
    axes[0].set_title('Attach-to-drone distance (< l_c means slack)')
    axes[0].set_ylabel('Distance (m)')
    axes[1].axhline(FORMATION_H, color='k', ls=':', label='h = 0.40 m')
    axes[1].set_title('Mean drone height above load')
    axes[1].set_ylabel('Height gap (m)')
    for ax in axes:
        _gust_window(ax)
        ax.set_xlabel('Time (s)'); ax.grid(True, alpha=0.3)
        ax.legend(fontsize=7)
    plt.tight_layout()
    _save(fig, 'cable_geometry.png')


def plot_tension_comparison(qp, pi):
    fig, axes = plt.subplots(1, 2, figsize=(12, 5), sharey=True)
    for ax, data in zip(axes, [qp, pi]):
        T = data['T'][data['airborne']]
        x = np.arange(N_DRONES)
        ax.bar(x, T.mean(axis=0), yerr=T.std(axis=0), color=DRONE_COLORS,
               capsize=5, alpha=0.8, label='Mean ± std (airborne)')
        ax.scatter(x, T.max(axis=0), marker='D', color='black', s=50,
                   zorder=5, label='Peak')
        ax.scatter(x, T.min(axis=0), marker='v', color='blue', s=50,
                   zorder=5, label='Minimum')
        ax.axhline(F_MIN, color='red', ls='--', lw=1, label=f'f_min={F_MIN} N')
        ax.set_xticks(x); ax.set_xticklabels(DRONE_LABELS)
        ax.set_title(f'Actual tension — {data["label"]}')
        ax.legend(fontsize=8); ax.grid(axis='y', alpha=0.3)
    axes[0].set_ylabel('Cable tension (N)')
    plt.suptitle(f'S026 — Physical tension distribution, gust {GUST_MAIN:.0f} N')
    plt.tight_layout()
    _save(fig, 'tension_comparison.png')


def plot_formation_snapshots(qp, pi):
    fig, axes = plt.subplots(2, 2, figsize=(11, 10))
    idx = [0, int(round(10.0 / DT))]
    for row, data in enumerate([qp, pi]):
        p, q, T = data['p'], data['q'], data['T']
        for ax, k in zip(axes[row], idx):
            ax.scatter(q[k, 0], q[k, 1], s=180, c='blue', marker='s',
                       label='Load', zorder=5)
            for i in range(N_DRONES):
                bp = q[k] + B_ATTACH[i]
                slack = T[k, i] <= 0.0
                ax.plot([bp[0], p[k, i, 0]], [bp[1], p[k, i, 1]],
                        color=DRONE_COLORS[i], lw=1.5,
                        ls=':' if slack else '-')
                ax.scatter(p[k, i, 0], p[k, i, 1], s=100, c=DRONE_COLORS[i],
                           label=f'{DRONE_LABELS[i]} T={T[k, i]:.2f} N',
                           zorder=6)
            ref = data['q_ref'][k]
            ax.scatter(ref[0], ref[1], marker='x', c='k', s=60,
                       label='Load reference')
            ax.set_xlim(q[k, 0] - 0.8, q[k, 0] + 0.8)
            ax.set_ylim(q[k, 1] - 0.8, q[k, 1] + 0.8)
            ax.set_aspect('equal'); ax.grid(True, alpha=0.3)
            ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
            ax.set_title(f'{data["label"]}, t = {data["time"][k]:.0f} s')
            ax.legend(fontsize=7, loc='lower right')
    plt.suptitle('S026 — Formation top-down (dotted cable = slack)')
    plt.tight_layout()
    _save(fig, 'formation_snapshots.png')


def plot_gust_sweep(sweep):
    amps = [r['gust'] for r in sweep['QP']]
    fig, axes = plt.subplots(1, 3, figsize=(16, 4.5))
    for name, col, mk in [('QP', 'steelblue', 'o'),
                          ('Pseudo-inverse', 'darkorange', 's')]:
        rs = sweep[name]
        axes[0].plot(amps, [r['phys_slack'] for r in rs], marker=mk,
                     color=col, label=name)
        axes[1].plot(amps, [r['L_min'] for r in rs], marker=mk,
                     color=col, label=name)
        axes[2].plot(amps, [r['err_gust'] for r in rs], marker=mk,
                     color=col, label=name)
    axes[0].set_ylabel('Airborne steps with a slack cable (T_i = 0)')
    axes[1].set_ylabel('Min attach-to-drone distance (m)')
    axes[1].axhline(CABLE_LENGTH, color='k', ls=':', lw=1,
                    label='rest length l_c')
    axes[2].set_ylabel('Max load error, t = 9–13 s (m)')
    for ax in axes:
        ax.axvline(GUST_PI_NEG, color='grey', ls=':', lw=1)
        ax.axvline(GUST_QP_MAX, color='grey', ls='--', lw=1)
        ax.set_xlabel('Gust amplitude (N)')
        ax.grid(True, alpha=0.3); ax.legend(fontsize=8)
    axes[0].set_title(f'Physical slack (dotted: PI f<0 at {GUST_PI_NEG:.2f} N,'
                      f'\ndashed: QP infeasible at {GUST_QP_MAX:.2f} N)',
                      fontsize=10)
    axes[1].set_title('How far a slack cable lets its drone sag in',
                      fontsize=10)
    axes[2].set_title('Worst load tracking error during/after the gust',
                      fontsize=10)
    plt.suptitle('S026 — Gust sweep (+x gust on the load, t = 9–11 s)')
    plt.tight_layout()
    _save(fig, 'gust_sweep.png')


def _save(fig, name):
    fp = os.path.join(OUTPUT_DIR, name)
    fig.savefig(fp, dpi=120)
    plt.close(fig)
    print(f'  Saved: {fp}')


# ── Animation ────────────────────────────────────────────────────────────────

def save_animation(qp, pi, n_frames=125, fps=10):
    """3D GIF of both runs side by side; slack cables drawn dotted."""
    steps = len(qp['time'])
    frame_idx = np.linspace(0, steps - 1, n_frames).astype(int)
    fig = plt.figure(figsize=(12, 5.5))
    objs = []
    for col, data in enumerate([qp, pi]):
        ax = fig.add_subplot(1, 2, col + 1, projection='3d')
        q, p = data['q'], data['p']
        ax.plot(*data['q_ref'].T, 'k:', lw=0.8, alpha=0.5)
        ax.set_xlim(-0.8, 3.8); ax.set_ylim(-0.8, 2.8); ax.set_zlim(0, 4.6)
        ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
        ax.set_title(data['label'])
        cables = [ax.plot([], [], [], color=DRONE_COLORS[i], lw=1.5)[0]
                  for i in range(N_DRONES)]
        dots = [ax.plot([], [], [], 'o', color=DRONE_COLORS[i], ms=6)[0]
                for i in range(N_DRONES)]
        load = ax.plot([], [], [], 's', color='blue', ms=9)[0]
        txt = ax.text2D(0.02, 0.95, '', transform=ax.transAxes, fontsize=9)
        objs.append((data, cables, dots, load, txt))
    fig.suptitle(f'S026 Cooperative Heavy Lift — gust {GUST_MAIN:.0f} N at '
                 f't = {GUST_T0:.0f}–{GUST_T1:.0f} s (dotted cable = slack)')

    def update(n):
        k = frame_idx[n]
        arts = []
        for data, cables, dots, load, txt in objs:
            q, p, T = data['q'][k], data['p'][k], data['T'][k]
            for i in range(N_DRONES):
                a = q + B_ATTACH[i]
                cables[i].set_data([a[0], p[i, 0]], [a[1], p[i, 1]])
                cables[i].set_3d_properties([a[2], p[i, 2]])
                cables[i].set_linestyle(':' if T[i] <= 0.0 else '-')
                dots[i].set_data([p[i, 0]], [p[i, 1]])
                dots[i].set_3d_properties([p[i, 2]])
            load.set_data([q[0]], [q[1]]); load.set_3d_properties([q[2]])
            gust = ' GUST' if GUST_T0 < data['time'][k] < GUST_T1 else ''
            txt.set_text(f't = {data["time"][k]:.1f} s{gust}')
            arts += cables + dots + [load, txt]
        return arts

    anim = FuncAnimation(fig, update, frames=n_frames, blit=False)
    fp = os.path.join(OUTPUT_DIR, 'animation.gif')
    anim.save(fp, writer='pillow', fps=fps)
    plt.close(fig)
    print(f'  Saved: {fp}  ({n_frames} frames, {fps} fps)')


# ── Metrics ───────────────────────────────────────────────────────────────────

def print_metrics(qp, pi, sweep):
    print('\n' + '=' * 64)
    print('S026 COOPERATIVE HEAVY LIFT - SIMULATION RESULTS')
    print('=' * 64)
    print(f'  Formation radius s = {FORMATION_S:.3f} m, cable angle '
          f'alpha0 = {np.degrees(np.arccos(COS_A0)):.2f} deg')
    print(f'  Static tension share m_L g/(4 cos a0) = {F0_STATIC:.4f} N')
    print(f'  Analytic gust thresholds: PI f1<0 at {GUST_PI_NEG:.3f} N, '
          f'PI f1<f_min at {GUST_PI_SLACK:.3f} N, '
          f'QP infeasible above {GUST_QP_MAX:.3f} N')
    for data in [qp, pi]:
        m = summarize(data)
        print(f'\n--- {data["label"]}, gust {data["gust"]:.1f} N ---')
        print(f'  Airborne samples:          {m["n_air"]} of {len(data["time"])}')
        print(f'  Allocation f<f_min steps:  {m["alloc_viol"]}  '
              f'(alloc range {m["alloc_min"]:.4f} .. {m["alloc_max"]:.4f} N)')
        print(f'  Physical slack steps:      {m["phys_slack"]}  '
              f'(T range {m["phys_min"]:.4f} .. {m["phys_max"]:.4f} N)')
        print(f'  Median actual tension:     {m["T_median"]:.4f} N')
        print(f'  Tension CoV (all cables):  {m["cov"]:.4f}')
        print(f'  Cable distance range:      {m["L_min"]:.4f} .. '
              f'{m["L_max"]:.4f} m')
        print(f'  Drone-load height gap:     {m["gap_min"]:.4f} .. '
              f'{m["gap_max"]:.4f} m')
        print(f'  Load error RMS / max:      {m["err_rms"]:.4f} / '
              f'{m["err_max"]:.4f} m  (max in t=9-13 s: '
              f'{m["err_gust"]:.4f} m)')
        print(f'  Max drone thrust:          {m["thrust_max"]:.3f} N '
              f'(limit {T_MAX} N, saturated drone-steps {data["n_sat"]})')
        if data['label'] == 'QP':
            print(f'  QP infeasible steps:       {data["qp_infeasible"]}')
        k20 = len(data['time']) - 1
        qf = data['q'][k20]
        print(f'  Final load position:       ({qf[0]:.3f}, {qf[1]:.3f}, '
              f'{qf[2]:.3f}) m')

    print('\n--- Gust sweep (airborne samples) ---')
    print('  gust | PI f<f_min | slack steps | min distance (m) |'
          ' max err 9-13 s (m) | QP infeas')
    print('   (N) |   steps    |   QP    PI  |   QP      PI     |'
          '   QP       PI      |  steps')
    for rq, rp in zip(sweep['QP'], sweep['Pseudo-inverse']):
        print(f'  {rq["gust"]:4.1f} | {rp["alloc_viol"]:6d}     |'
              f' {rq["phys_slack"]:4d}  {rp["phys_slack"]:4d}  |'
              f' {rq["L_min"]:6.4f}  {rp["L_min"]:6.4f}  |'
              f' {rq["err_gust"]:7.4f}  {rp["err_gust"]:7.4f}   |'
              f'  {rq["qp_infeasible"]:4d}')
    print('=' * 64)


# ── Entry point ───────────────────────────────────────────────────────────────

if __name__ == '__main__':
    print('Running S026 Cooperative Heavy Lift Simulation')
    sweep = {'QP': [], 'Pseudo-inverse': []}
    runs = {}
    for amp in GUST_SWEEP:
        for use_qp in (True, False):
            d = run_simulation(use_qp, amp)
            m = summarize(d)
            m['gust'] = amp
            m['qp_infeasible'] = d['qp_infeasible']
            sweep[d['label']].append(m)
            if amp == GUST_MAIN:
                runs[d['label']] = d
        print(f'  gust {amp:.1f} N done')
    qp_data, pi_data = runs['QP'], runs['Pseudo-inverse']

    print('\nGenerating plots...')
    plot_trajectory_3d(qp_data, pi_data)
    plot_cable_tensions(qp_data, pi_data)
    plot_slack_margin(qp_data, pi_data)
    plot_position_error(qp_data, pi_data)
    plot_cable_geometry(qp_data, pi_data)
    plot_tension_comparison(qp_data, pi_data)
    plot_formation_snapshots(qp_data, pi_data)
    plot_gust_sweep(sweep)

    print_metrics(qp_data, pi_data, sweep)

    print('\nGenerating animation...')
    save_animation(qp_data, pi_data)
