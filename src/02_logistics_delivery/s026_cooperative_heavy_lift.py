"""
S026 Cooperative Heavy Lift
============================
Four quadrotor drones cooperatively lift and transport a heavy load (0.8 kg)
via cable suspension.  The load mass exceeds the single-drone payload limit.

Two tension-allocation strategies are compared:
  1. Pseudo-inverse  — minimum L2-norm solution; may produce near-zero tensions
  2. Anti-slack QP   — SLSQP with f_i >= f_min constraint; guarantees cable taut

Simulation design
-----------------
At each timestep we:
  1. Advance the reference trajectory (quintic blend).
  2. Run a PD controller per drone that tracks the formation offset.
  3. Compute cable tensions from the current geometry (pseudo-inverse or QP).
  4. Propagate a separate load dynamics integrator driven by cable forces,
     representing the physical load response around the reference.

The load position *error* integrator captures how well the formation follows
the desired path.  Tensions are strictly bounded to physical limits.

Usage:
    conda activate drones
    python src/02_logistics_delivery/s026_cooperative_heavy_lift.py
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401
from scipy.optimize import minimize

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
MASS_DRONE    = 0.5     # kg  (effective drone mass including batteries/motors)
CABLE_LENGTH  = 0.5     # m
FORMATION_S   = 0.6     # m  horizontal separation
FORMATION_H   = 0.40    # m  vertical cable projection
F_MIN         = 0.05    # N  minimum tension (anti-slack)
KP            = 5.0     # PD proportional gain
KD            = 2.5     # PD derivative gain
V_MAX         = 3.0     # m/s velocity saturation
A_MAX         = 15.0    # m/s² acceleration saturation
DT            = 0.01    # s
T_TOTAL       = 20.0    # s

# Max physical tension per cable (safety clamp for numerics)
F_MAX_PHYS    = MASS_LOAD * G * 3.0   # 3× single-load weight

# Load cable attachment points in body frame
B_ATTACH = np.array([
    [+0.15,  0.00, 0.0],
    [-0.15,  0.00, 0.0],
    [ 0.00, +0.15, 0.0],
    [ 0.00, -0.15, 0.0],
])

# Drone formation offsets relative to load centroid
D_OFFSETS = np.array([
    [+FORMATION_S,  0.0,          FORMATION_H],
    [-FORMATION_S,  0.0,          FORMATION_H],
    [ 0.0,         +FORMATION_S,  FORMATION_H],
    [ 0.0,         -FORMATION_S,  FORMATION_H],
])

DRONE_COLORS = ['red', 'orange', 'magenta', 'gold']
DRONE_LABELS = [f'Drone {i+1}' for i in range(N_DRONES)]

# ── Trajectory waypoints ──────────────────────────────────────────────────────
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
            dt  = t1 - t0
            s   =   10*tau**3 -  15*tau**4 +  6*tau**5
            ds  =  (30*tau**2 -  60*tau**3 + 30*tau**4) / dt
            dds =  (60*tau    - 180*tau**2 + 120*tau**3) / (dt**2)
            dq  = q1 - q0
            return q0 + s*dq, ds*dq, dds*dq
    q_last = WAYPOINTS[-1][3]
    return q_last.copy(), np.zeros(3), np.zeros(3)


# ── Cable tension solvers ─────────────────────────────────────────────────────

def cable_unit_vecs(drone_pos, load_pos):
    """Compute cable unit vectors (attachment_world -> drone) and check validity."""
    c_hats = np.zeros((N_DRONES, 3))
    for i in range(N_DRONES):
        diff = drone_pos[i] - (load_pos + B_ATTACH[i])
        nrm  = np.linalg.norm(diff)
        c_hats[i] = diff / max(nrm, 1e-4)
    return c_hats


def build_A(c_hats):
    A = np.zeros((6, N_DRONES))
    for i in range(N_DRONES):
        A[:3, i] = c_hats[i]
        A[3:, i] = np.cross(B_ATTACH[i], c_hats[i])
    return A


B_RHS = np.array([0., 0., MASS_LOAD * G, 0., 0., 0.])


def solve_pseudoinverse(A):
    AAt = A @ A.T
    try:
        f = A.T @ np.linalg.solve(AAt, B_RHS)
    except np.linalg.LinAlgError:
        f = np.linalg.lstsq(A, B_RHS, rcond=None)[0]
    return f


def solve_qp(A, f_warm=None):
    N  = A.shape[1]
    f0 = f_warm if f_warm is not None else np.full(N, max(MASS_LOAD * G / N, F_MIN))
    f0 = np.maximum(f0, F_MIN)

    res = minimize(
        fun=lambda f: float(np.dot(f, f)),
        x0=f0,
        jac=lambda f: 2.0 * f,
        constraints=[{'type': 'eq',
                      'fun': lambda f: A @ f - B_RHS,
                      'jac': lambda f: A}],
        bounds=[(F_MIN, None)] * N,
        method='SLSQP',
        options={'ftol': 1e-10, 'maxiter': 500}
    )
    return res.x if res.success else np.maximum(solve_pseudoinverse(A), F_MIN)


# ── Simulation ────────────────────────────────────────────────────────────────

def run_simulation(use_qp: bool):
    label  = 'QP' if use_qp else 'PseudoInverse'
    steps  = int(round(T_TOTAL / DT)) + 1
    time   = np.linspace(0.0, T_TOTAL, steps)

    q_ref_h  = np.zeros((steps, 3))
    p_hist   = np.zeros((steps, N_DRONES, 3))
    pv_hist  = np.zeros((steps, N_DRONES, 3))
    f_hist   = np.zeros((steps, N_DRONES))
    slack_h  = np.zeros(steps)
    thrust_h = np.zeros((steps, N_DRONES))
    # Load position = reference + error from dynamics
    q_err_h  = np.zeros((steps, 3))   # load position error vs reference

    # ── Init ──────────────────────────────────────────────────────────────
    q_r0, _, _ = load_ref(0.0)
    q_ref_h[0] = q_r0

    # Drones start in nominal formation above ground load
    p  = np.array([q_r0 + D_OFFSETS[i] for i in range(N_DRONES)])
    pv = np.zeros((N_DRONES, 3))
    p_hist[0]  = p.copy()
    pv_hist[0] = pv.copy()

    # Load error dynamics: perturbation from reference
    eq  = np.zeros(3)   # load centroid error
    eqv = np.zeros(3)   # load centroid error velocity

    f_warm = None

    for k in range(steps - 1):
        t     = time[k]
        q_r, vr, ar = load_ref(t)
        # True load position = reference + error
        q_true = q_r + eq

        # ── Compute cable unit vectors and tensions ────────────────────
        c_hats = cable_unit_vecs(p, q_true)
        A      = build_A(c_hats)

        if use_qp:
            f      = solve_qp(A, f_warm)
            f_warm = f.copy()
        else:
            f = solve_pseudoinverse(A)

        # Physical clamp and positivity
        f = np.clip(f, 0.0, F_MAX_PHYS)

        # ── Load error dynamics ───────────────────────────────────────
        # Net cable force on load
        F_cable = np.sum(f[:, None] * c_hats, axis=0)
        # Residual acceleration = (cable - gravity)/m  — should be ~0 at eq
        q_acc_err = (F_cable - np.array([0., 0., MASS_LOAD * G])) / MASS_LOAD
        q_acc_err = np.clip(q_acc_err, -A_MAX, A_MAX)
        # Add damping proportional to error velocity to stabilise
        q_acc_err -= 2.0 * eqv

        eqv_new = eqv + q_acc_err * DT
        eqv_new = np.clip(eqv_new, -V_MAX, V_MAX)
        eq_new  = eq  + eqv * DT
        # Ground floor on absolute load position
        if (q_r[2] + eq_new[2]) < 0.0:
            eq_new[2]  = -q_r[2]
            eqv_new[2] = max(eqv_new[2], 0.0)
        eq, eqv = eq_new, eqv_new

        # ── Drone PD control ─────────────────────────────────────────
        p_new  = np.zeros_like(p)
        pv_new = np.zeros_like(pv)
        for i in range(N_DRONES):
            p_des = q_r + D_OFFSETS[i]
            v_des = vr
            a_cmd = KP * (p_des - p[i]) + KD * (v_des - pv[i])
            a_cmd = np.clip(a_cmd, -A_MAX, A_MAX)
            pv_i  = np.clip(pv[i] + a_cmd * DT, -V_MAX, V_MAX)
            pv_new[i] = pv_i
            p_new[i]  = p[i] + pv[i] * DT
            # Thrust command = drone weight + commanded acceleration force
            thrust_h[k, i] = MASS_DRONE * np.linalg.norm(
                a_cmd + np.array([0., 0., G])
            )

        p, pv = p_new, pv_new

        # ── Store ────────────────────────────────────────────────────
        q_ref_h[k+1]  = q_r
        p_hist[k+1]   = p.copy()
        pv_hist[k+1]  = pv.copy()
        f_hist[k+1]   = f
        q_err_h[k+1]  = eq
        slack_h[k+1]  = np.min(f) - F_MIN

    # Fill t=0 tensions
    c0 = cable_unit_vecs(p_hist[0], q_ref_h[0] + q_err_h[0])
    A0 = build_A(c0)
    f0 = solve_qp(A0) if use_qp else solve_pseudoinverse(A0)
    f_hist[0]  = np.clip(f0, 0.0, F_MAX_PHYS)
    slack_h[0] = np.min(f_hist[0]) - F_MIN

    pos_error = np.linalg.norm(q_err_h, axis=1)

    return {
        'label':     label,
        'time':      time,
        'q_ref':     q_ref_h,
        'q_err':     q_err_h,
        'p':         p_hist,
        'f':         f_hist,
        'slack':     slack_h,
        'pos_error': pos_error,
        'thrust':    thrust_h,
    }


# ── Plots ─────────────────────────────────────────────────────────────────────

def plot_trajectory_3d(qp, pi):
    fig = plt.figure(figsize=(12, 8))
    ax  = fig.add_subplot(111, projection='3d')
    for data, ls, alpha in [(qp, '-', 0.9), (pi, '--', 0.55)]:
        qr = data['q_ref']
        q  = qr + data['q_err']
        p  = data['p']
        lb = data['label']
        ax.plot(qr[:, 0], qr[:, 1], qr[:, 2],
                'b:', linewidth=1.2, alpha=0.3,
                label='Load reference' if ls == '-' else None)
        ax.plot(q[:, 0], q[:, 1], q[:, 2],
                'b', linestyle=ls, linewidth=2,
                label=f'Load ({lb})', alpha=alpha)
        for i in range(N_DRONES):
            ax.plot(p[:, i, 0], p[:, i, 1], p[:, i, 2],
                    color=DRONE_COLORS[i], linestyle=ls, linewidth=1.2,
                    label=f'{DRONE_LABELS[i]} ({lb})' if ls == '-' else None,
                    alpha=alpha * 0.75)
    for (_, _, q0, q1) in WAYPOINTS:
        ax.scatter(*q0, c='green',  s=70, zorder=5)
        ax.scatter(*q1, c='purple', s=70, zorder=5)
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
    ax.set_title('S026 Cooperative Heavy Lift — 3D Trajectory\n'
                 '(solid=QP  dashed=Pseudo-Inverse)')
    handles, labs = ax.get_legend_handles_labels()
    seen = {}
    for h, l in zip(handles, labs):
        if l and l not in seen:
            seen[l] = h
    ax.legend(list(seen.values()), list(seen.keys()),
              loc='upper left', fontsize=7, ncol=2)
    plt.tight_layout()
    fp = os.path.join(OUTPUT_DIR, 'trajectory_3d.png')
    plt.savefig(fp, dpi=120); plt.close()
    print(f'  Saved: {fp}')


def plot_cable_tensions(qp, pi):
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    for ax, data in zip(axes, [qp, pi]):
        for i in range(N_DRONES):
            ax.plot(data['time'], data['f'][:, i],
                    color=DRONE_COLORS[i], label=DRONE_LABELS[i], linewidth=1.4)
        ax.axhline(F_MIN, color='black', linestyle='--', linewidth=1,
                   label=f'f_min = {F_MIN} N')
        ax.set_ylabel('Tension (N)')
        ax.set_title(f'Cable Tensions — {data["label"]}')
        ax.legend(fontsize=8, ncol=5)
        ax.grid(True, alpha=0.3)
    axes[-1].set_xlabel('Time (s)')
    plt.tight_layout()
    fp = os.path.join(OUTPUT_DIR, 'cable_tensions.png')
    plt.savefig(fp, dpi=120); plt.close()
    print(f'  Saved: {fp}')


def plot_slack_margin(qp, pi):
    fig, ax = plt.subplots(figsize=(12, 4))
    for data, col in [(qp, 'steelblue'), (pi, 'darkorange')]:
        ax.plot(data['time'], data['slack'],
                color=col, label=data['label'], linewidth=1.5)
    ax.axhline(0.0, color='red', linestyle='--', linewidth=1.2,
               label='Slack boundary')
    ax.fill_between(qp['time'], 0, qp['slack'],
                    where=qp['slack'] >= 0, alpha=0.15, color='steelblue')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Slack margin (N)')
    ax.set_title('S026 — Cable Slack Margin  delta = min_i f_i - f_min')
    ax.legend(); ax.grid(True, alpha=0.3)
    plt.tight_layout()
    fp = os.path.join(OUTPUT_DIR, 'slack_margin.png')
    plt.savefig(fp, dpi=120); plt.close()
    print(f'  Saved: {fp}')


def plot_position_error(qp, pi):
    fig, ax = plt.subplots(figsize=(12, 4))
    for data, col in [(qp, 'steelblue'), (pi, 'darkorange')]:
        ax.plot(data['time'], data['pos_error'],
                color=col, label=data['label'], linewidth=1.5)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Load position error ||q - q_ref|| (m)')
    ax.set_title('S026 — Load Tracking Error')
    ax.legend(); ax.grid(True, alpha=0.3)
    plt.tight_layout()
    fp = os.path.join(OUTPUT_DIR, 'position_error.png')
    plt.savefig(fp, dpi=120); plt.close()
    print(f'  Saved: {fp}')


def plot_tension_comparison(qp, pi):
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    for ax, data in zip(axes, [qp, pi]):
        mf = data['f'].mean(axis=0)
        sf = data['f'].std(axis=0)
        xf = data['f'].max(axis=0)
        x  = np.arange(N_DRONES)
        ax.bar(x, mf, yerr=sf, color=DRONE_COLORS,
               capsize=5, alpha=0.8, label='Mean +/- std')
        ax.scatter(x, xf, marker='D', color='black', s=50, zorder=5, label='Peak')
        ax.axhline(F_MIN, color='red', linestyle='--', linewidth=1,
                   label=f'f_min={F_MIN} N')
        ax.set_xticks(x); ax.set_xticklabels(DRONE_LABELS)
        ax.set_ylabel('Cable Tension (N)')
        ax.set_title(f'Tension Distribution — {data["label"]}')
        ax.legend(fontsize=8); ax.grid(axis='y', alpha=0.3)
    plt.suptitle('S026 — Tension Allocation: QP vs Pseudo-Inverse')
    plt.tight_layout()
    fp = os.path.join(OUTPUT_DIR, 'tension_comparison.png')
    plt.savefig(fp, dpi=120); plt.close()
    print(f'  Saved: {fp}')


def plot_formation_snapshots(qp):
    p = qp['p']
    q = qp['q_ref'] + qp['q_err']
    fig, axes = plt.subplots(1, 2, figsize=(10, 5))
    titles  = ['t=0 s  (liftoff)', 't=10 s (cruise mid)']
    indices = [0, int(10.0 / DT)]
    for ax, idx, title in zip(axes, indices, titles):
        ax.scatter(q[idx, 0], q[idx, 1], s=180, c='blue',
                   marker='s', label='Load', zorder=5)
        for i in range(N_DRONES):
            bp = q[idx] + B_ATTACH[i]
            ax.scatter(bp[0], bp[1], s=40, c='cyan', marker='+', zorder=4)
            ax.scatter(p[idx, i, 0], p[idx, i, 1],
                       s=100, c=DRONE_COLORS[i],
                       label=DRONE_LABELS[i], zorder=6)
            ax.plot([bp[0], p[idx, i, 0]], [bp[1], p[idx, i, 1]],
                    color=DRONE_COLORS[i], linewidth=1.2, alpha=0.7)
        ax.set_aspect('equal')
        ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
        ax.set_title(title); ax.legend(fontsize=7); ax.grid(True, alpha=0.3)
    plt.suptitle('S026 — Formation Geometry Top-Down (QP)', fontsize=12)
    plt.tight_layout()
    fp = os.path.join(OUTPUT_DIR, 'formation_snapshots.png')
    plt.savefig(fp, dpi=120); plt.close()
    print(f'  Saved: {fp}')


# ── Metrics ───────────────────────────────────────────────────────────────────

def print_metrics(qp, pi):
    print('\n' + '='*60)
    print('S026 COOPERATIVE HEAVY LIFT - SIMULATION RESULTS')
    print('='*60)

    for data in [qp, pi]:
        f     = data['f']
        slack = data['slack']
        err   = data['pos_error']
        lb    = data['label']
        n_viol = int(np.sum(slack < 0))

        print(f'\n--- Strategy: {lb} ---')
        print(f'  Load mass:               {MASS_LOAD} kg')
        print(f'  Number of drones:        {N_DRONES}')
        print(f'  Min cable tension:       {f.min():.4f} N')
        print(f'  Max cable tension:       {f.max():.4f} N')
        print(f'  Mean tension per drone:  ' +
              ', '.join(f'{v:.3f}' for v in f.mean(axis=0)) + ' N')
        print(f'  Peak tension per drone:  ' +
              ', '.join(f'{v:.3f}' for v in f.max(axis=0)) + ' N')
        print(f'  Min slack margin:        {slack.min():.4f} N')
        print(f'  Slack violations:        {n_viol} steps '
              f'({100*n_viol/len(slack):.1f}%)')
        print(f'  Max load position error: {err.max():.4f} m')
        print(f'  RMS load position error: {np.sqrt(np.mean(err**2)):.4f} m')
        q_final = data['q_ref'][-1] + data['q_err'][-1]
        print(f'  Final load position:     '
              f'({q_final[0]:.3f}, {q_final[1]:.3f}, {q_final[2]:.3f}) m')

    print('\n--- QP vs PseudoInverse ---')
    fq, fp_ = qp['f'], pi['f']
    print(f'  QP   tension uniformity (CoV): {fq.std()/fq.mean():.4f}')
    print(f'  PI   tension uniformity (CoV): {fp_.std()/fp_.mean():.4f}')
    print(f'  QP   min slack:                {qp["slack"].min():.4f} N')
    print(f'  PI   min slack:                {pi["slack"].min():.4f} N')
    print(f'  QP   RMS position error:       '
          f'{np.sqrt(np.mean(qp["pos_error"]**2)):.4f} m')
    print(f'  PI   RMS position error:       '
          f'{np.sqrt(np.mean(pi["pos_error"]**2)):.4f} m')
    print('='*60)
    print(f'\nOutput directory: {OUTPUT_DIR}')


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    print('Running S026 Cooperative Heavy Lift Simulation')
    print(f'  Config: N_DRONES={N_DRONES}  MASS_LOAD={MASS_LOAD} kg  '
          f'T_TOTAL={T_TOTAL} s  DT={DT} s')

    print('\n[1/2] QP anti-slack tension allocation...')
    qp_data = run_simulation(use_qp=True)
    print('      Done.')

    print('[2/2] Pseudo-inverse tension allocation...')
    pi_data = run_simulation(use_qp=False)
    print('      Done.')

    print('\nGenerating plots...')
    plot_trajectory_3d(qp_data, pi_data)
    plot_cable_tensions(qp_data, pi_data)
    plot_slack_margin(qp_data, pi_data)
    plot_position_error(qp_data, pi_data)
    plot_tension_comparison(qp_data, pi_data)
    plot_formation_snapshots(qp_data)

    print_metrics(qp_data, pi_data)


if __name__ == '__main__':
    main()
