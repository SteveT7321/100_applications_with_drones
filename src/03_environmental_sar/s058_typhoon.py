"""
S058 Typhoon Eye Probing
=========================
A 1.5 kg research drone flies an inbound spiral from r = 190 m to the calm eye
(r = 15 m) of a Rankine-vortex typhoon (peak tangential wind 50 m/s at r_max = 150 m,
radial inflow 8 m/s). The wind acts on the drone through aerodynamic drag,
F = b (v_wind - v), and horizontal thrust is limited to |u| <= m g (45 deg tilt).
Three trajectory-tracking controllers are compared under the same turbulence
realisations (common random numbers):

  1. PD baseline (card gains, nominal feed-forward, no wind model)
  2. MPC with wind feed-forward (condensed QP, N_p = 20, thrust-disk constraint, ADMM)
  3. H-infinity state feedback (discrete game Riccati, gamma bisection, no wind model)

plus an MPC ablation without wind feed-forward and a wind-severity sweep
v_max = 20 ... 60 m/s.

Usage:
    conda activate drones
    python src/03_environmental_sar/s058_typhoon.py
"""

import os
for _v in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[_v] = '1'          # single-threaded BLAS: reproducible and fast on a shared machine
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Circle, Wedge
from mpl_toolkits.mplot3d import Axes3D              # noqa: F401
from scipy.linalg import expm, solve_discrete_are

# ── Parameters (card) ─────────────────────────────────────────────────────────
M_DRONE    = 1.5        # kg
G          = 9.81       # m/s^2
B_DRAG     = 0.4        # N s/m — linear aerodynamic drag coefficient
F_MAX_CARD = 30.0       # N — rated thrust (the 45 deg tilt limit is tighter)
U_MAX      = min(F_MAX_CARD, M_DRONE * G)   # N — horizontal thrust limit = m g
R_MAX      = 150.0      # m — max-wind radius
V_MAX      = 50.0       # m/s — peak tangential wind
V_INFLOW   = 8.0        # m/s — peak radial inflow
SIGMA_RAD  = 40.0       # m — inflow layer width
TURB_FRAC  = 0.10       # turbulence std = 10 % of local v_tan
R_START    = 190.0      # m
R_EYE      = 15.0       # m
V_INBOUND  = 0.8        # m/s
W_SPIRAL   = 0.12       # rad/s
DT         = 0.1        # s
N_PRED     = 20         # MPC horizon (2 s)
Q_MPC      = np.diag([10.0, 10.0, 1.0, 1.0])
R_MPC      = 0.1 * np.eye(2)
KP_PD, KD_PD = 4.0, 2.5  # PD gains (card code)

# ── Parameters (added) ────────────────────────────────────────────────────────
TAU_TURB   = 2.0        # s — correlation time of the band-limited (Ornstein-Uhlenbeck) gusts
E_FAIL     = R_EYE      # m — track lost if the error ever exceeds the eye radius
GAMMA_MARGIN = 1.10     # operate the H-inf controller at 1.1 x gamma*
N_SEEDS    = 12         # turbulence seeds, main comparison
SWEEP_VMAX = [20.0, 30.0, 40.0, 50.0, 60.0]
N_SEEDS_SWEEP = 5
PROBE_THRUST = 2.0      # thrust-limit multiplier for the saturation probe
SEED0      = 100
SHOW_SEED  = SEED0      # seed used for the figures and the animation
ADMM_RHO   = None       # set from the QP Hessian
ADMM_ITERS = 150
ADMM_TOL   = 1e-3       # N
ANIM_STEP  = 20         # simulation steps per GIF frame (2 s)

T_MISSION  = (R_START - R_EYE) / V_INBOUND           # 218.75 s
N_STEPS    = int(np.ceil(T_MISSION / DT - 1e-9))     # 2188
T_EYEWALL  = (R_START - R_MAX) / V_INBOUND           # 50 s

CTRL = ['pd', 'mpc', 'hinf']
ABLATIONS = ['mpc_nowind']
LABELS = {'pd': 'PD baseline', 'mpc': 'MPC + wind feed-forward', 'hinf': 'H-infinity',
          'mpc_nowind': 'MPC, no wind feed-forward'}
COLORS = {'pd': '#FF7F0E', 'mpc': '#2CA02C', 'hinf': '#9467BD', 'mpc_nowind': '#8C564B'}

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '03_environmental_sar', 's058_typhoon',
)


# ── Wind and reference ────────────────────────────────────────────────────────

def v_tan(r, v_max=V_MAX):
    r = np.asarray(r, float)
    return np.where(r <= R_MAX, v_max * r / R_MAX, v_max * R_MAX / np.maximum(r, 1e-9))


def v_rad(r):
    return -V_INFLOW * np.exp(-((np.asarray(r, float) - R_MAX) ** 2) / (2 * SIGMA_RAD ** 2))


def rankine_wind(p, v_max=V_MAX):
    """Wind velocity (m/s) at Cartesian position p (..., 2)."""
    p = np.asarray(p, float)
    r = np.linalg.norm(p, axis=-1)
    rs = np.maximum(r, 1e-9)[..., None]
    e_r = p / rs
    e_th = np.stack([-p[..., 1], p[..., 0]], axis=-1) / rs
    return v_tan(r, v_max)[..., None] * e_th + v_rad(r)[..., None] * e_r


def reference(t):
    """Position, velocity and acceleration of the inbound spiral at time t (scalar or array)."""
    t = np.asarray(t, float)
    r = np.maximum(R_EYE, R_START - V_INBOUND * t)
    rdot = np.where(R_START - V_INBOUND * t > R_EYE, -V_INBOUND, 0.0)
    th = W_SPIRAL * t
    e_r = np.stack([np.cos(th), np.sin(th)], axis=-1)
    e_th = np.stack([-np.sin(th), np.cos(th)], axis=-1)
    p = r[..., None] * e_r
    v = rdot[..., None] * e_r + (r * W_SPIRAL)[..., None] * e_th
    a = (-r * W_SPIRAL ** 2)[..., None] * e_r + (2 * rdot * W_SPIRAL)[..., None] * e_th
    return p, v, a


def nominal_ff(v_ref, a_ref):
    """Thrust that tracks the reference in still air: m a_ref + b v_ref."""
    return M_DRONE * a_ref + B_DRAG * v_ref


# ── Linear model ──────────────────────────────────────────────────────────────

def build_discrete_ss(dt=DT):
    """Exact ZOH discretisation of x = (p, v), m dv/dt = u - b v + m w."""
    A = np.block([[np.zeros((2, 2)), np.eye(2)],
                  [np.zeros((2, 2)), -(B_DRAG / M_DRONE) * np.eye(2)]])
    B = np.vstack([np.zeros((2, 2)), np.eye(2) / M_DRONE])
    E = np.vstack([np.zeros((2, 2)), np.eye(2)])
    n = 4
    Mbig = np.zeros((8, 8))
    Mbig[:4, :4] = A
    Mbig[:4, 4:6] = B
    Mbig[:4, 6:8] = E
    Me = expm(Mbig * dt)
    return A, B, E, Me[:n, :n], Me[:n, 4:6], Me[:n, 6:8]


A_C, B_C, E_C, AD, BD, ED = build_discrete_ss()
CZ = np.diag([np.sqrt(Q_MPC[0, 0]), np.sqrt(Q_MPC[1, 1]), 0.0, 0.0])   # card: position error


def lqr_gain(Q=Q_MPC, R=R_MPC):
    P = solve_discrete_are(AD, BD, Q, R)
    K = np.linalg.solve(R + BD.T @ P @ BD, BD.T @ P @ AD)
    return K, P


def hinf_riccati(gamma, Q=Q_MPC * np.diag([1, 1, 0, 0]), R=R_MPC, iters=20000, tol=1e-10):
    """Discrete full-information H-inf game Riccati. Returns (K_u, X) or None if infeasible."""
    Bt = np.hstack([BD, ED])
    Rg = np.block([[R, np.zeros((2, 2))], [np.zeros((2, 2)), -gamma ** 2 * np.eye(2)]])
    X = Q.copy()
    for _ in range(iters):
        S = Rg + Bt.T @ X @ Bt
        if np.min(np.linalg.eigvalsh(gamma ** 2 * np.eye(2) - ED.T @ X @ ED)) <= 0:
            return None
        Xn = Q + AD.T @ X @ AD - AD.T @ X @ Bt @ np.linalg.solve(S, Bt.T @ X @ AD)
        Xn = 0.5 * (Xn + Xn.T)
        if not np.all(np.isfinite(Xn)) or np.max(np.abs(Xn)) > 1e12:
            return None
        if np.max(np.abs(Xn - X)) < tol * max(1.0, np.max(np.abs(X))):
            X = Xn
            break
        X = Xn
    else:
        return None
    if np.min(np.linalg.eigvalsh(X)) < -1e-9:
        return None
    if np.min(np.linalg.eigvalsh(gamma ** 2 * np.eye(2) - ED.T @ X @ ED)) <= 0:
        return None
    S = Rg + Bt.T @ X @ Bt
    K = np.linalg.solve(S, Bt.T @ X @ AD)[:2]
    if np.max(np.abs(np.linalg.eigvals(AD - BD @ K))) >= 1.0:
        return None
    return K, X


def hinf_norm(K, n_freq=4000):
    """sup_w sigma_max of the closed-loop map w -> z = [C_z x; R^1/2 u] (discrete)."""
    Acl = AD - BD @ K
    Cz = np.vstack([CZ, np.sqrt(R_MPC[0, 0]) * (-K)])
    best = 0.0
    for om in np.linspace(0, np.pi, n_freq):
        z = np.exp(1j * om)
        T = Cz @ np.linalg.solve(z * np.eye(4) - Acl, ED)
        best = max(best, np.linalg.svd(T, compute_uv=False)[0])
    return best


def hinf_synthesis():
    K_lqr, _ = lqr_gain()
    hi = hinf_norm(K_lqr) * 1.5
    assert hinf_riccati(hi) is not None
    lo = 1e-3
    for _ in range(50):
        mid = np.sqrt(lo * hi)
        if hinf_riccati(mid) is None:
            lo = mid
        else:
            hi = mid
        if hi / lo < 1.0005:
            break
    gamma_star = hi
    gamma = GAMMA_MARGIN * gamma_star
    K, _ = hinf_riccati(gamma)
    return K, gamma_star, gamma


# ── MPC (condensed QP over the thrust disk, solved with ADMM) ─────────────────

class MPC:
    def __init__(self, mode='mpc', u_max=U_MAX):
        self.mode = mode
        self.u_max = u_max
        n, m, N = 4, 2, N_PRED
        _, P = lqr_gain()
        self.Phi = np.zeros((N * n, n))
        self.Gam = np.zeros((N * n, N * m))
        self.Psi = np.zeros((N * n, N * m))
        powers = [np.eye(n)]
        for j in range(N):
            powers.append(powers[-1] @ AD)
        for j in range(N):
            self.Phi[j * n:(j + 1) * n] = powers[j + 1]
            for i in range(j + 1):
                self.Gam[j * n:(j + 1) * n, i * m:(i + 1) * m] = powers[j - i] @ BD
                self.Psi[j * n:(j + 1) * n, i * m:(i + 1) * m] = powers[j - i] @ ED
        Qbar = np.kron(np.eye(N), Q_MPC)
        Qbar[-n:, -n:] = P
        self.Qbar = Qbar
        self.GQ = self.Gam.T @ Qbar
        self.GQPhi = self.GQ @ self.Phi
        self.GQPsi = self.GQ @ self.Psi
        self.Rbar = np.kron(np.eye(N), R_MPC)
        self.H = self.Gam.T @ Qbar @ self.Gam + self.Rbar
        ev = np.linalg.eigvalsh(self.H)
        self.rho = float(np.sqrt(ev[0] * ev[-1]))
        self.Hinv = np.linalg.inv(self.H)
        self.Minv = np.linalg.inv(self.H + self.rho * np.eye(N * m))
        self.U = np.zeros(N * m)
        self.Y = np.zeros(N * m)
        self.pred_pos = None
        self.n_admm = 0
        self.n_solves = 0

    def project(self, U):
        u = U.reshape(-1, 2)
        nrm = np.linalg.norm(u, axis=1)
        s = np.where(nrm > self.u_max, self.u_max / np.maximum(nrm, 1e-12), 1.0)
        return (u * s[:, None]).ravel()

    def step(self, x, t, v_max):
        N = N_PRED
        tj = t + DT * np.arange(N + 1)
        pr, vr, ar = reference(tj)
        Xr = np.hstack([pr[1:], vr[1:]]).ravel()
        if self.mode == 'mpc_nowind':
            W = np.zeros(2 * N)
        else:
            pp = np.vstack([x[:2], self.pred_pos[1:]]) if self.pred_pos is not None else \
                np.repeat(x[None, :2], N, axis=0)
            W = (B_DRAG / M_DRONE * rankine_wind(pp[:N], v_max)).ravel()
        # card cost: sum ||x - x_ref||_Q + ||u||_R (absolute thrust) + terminal DARE weight P
        g = self.GQPhi @ x + self.GQPsi @ W - self.GQ @ Xr
        U = -self.Hinv @ g
        self.n_solves += 1
        if np.any(np.linalg.norm(U.reshape(-1, 2), axis=1) > self.u_max):
            Z = self.project(np.concatenate([self.U[2:], self.U[-2:]]))
            Y = np.zeros_like(Z)
            for it in range(ADMM_ITERS):
                U = self.Minv @ (self.rho * (Z - Y) - g)
                Zn = self.project(U + Y)
                Y = Y + U - Zn
                r_p = np.max(np.abs(U - Zn))
                r_d = self.rho * np.max(np.abs(Zn - Z))
                Z = Zn
                if r_p < ADMM_TOL and r_d < ADMM_TOL:
                    break
            self.n_admm += 1
            U = Z
        self.U = U
        X = self.Phi @ x + self.Gam @ U + self.Psi @ W
        self.pred_pos = np.vstack([x[:2], X.reshape(N, 4)[:, :2]])[1:]
        self.pred_pos = np.vstack([self.pred_pos, self.pred_pos[-1:]])
        return U[:2].copy()


# ── Closed-loop simulation ────────────────────────────────────────────────────

def turbulence_noise(seed, n=N_STEPS):
    """Unit-variance Ornstein-Uhlenbeck sequence (n, 2), shared by every controller."""
    rng = np.random.default_rng(seed)
    phi = np.exp(-DT / TAU_TURB)
    xi = rng.standard_normal((n, 2))
    out = np.zeros((n, 2))
    s = rng.standard_normal(2)
    for k in range(n):
        out[k] = s
        s = phi * s + np.sqrt(1 - phi ** 2) * xi[k]
    return out


def simulate(ctrl, seed, v_max=V_MAX, K_hinf=None, record=False, u_max=U_MAX):
    noise = turbulence_noise(seed)
    p0, v0, _ = reference(0.0)
    x = np.concatenate([p0, v0])
    mpc = MPC(ctrl, u_max) if ctrl.startswith('mpc') else None
    err = np.zeros(N_STEPS)
    unorm = np.zeros(N_STEPS)
    traj = np.zeros((N_STEPS + 1, 2))
    traj[0] = x[:2]
    rec = {'u': np.zeros((N_STEPS, 2)), 'wind': np.zeros((N_STEPS, 2)),
           'ref': np.zeros((N_STEPS + 1, 2))} if record else None
    if record:
        rec['ref'][0] = p0
    for k in range(N_STEPS):
        t = k * DT
        pr, vr, ar = reference(t)
        xr = np.concatenate([pr, vr])
        r = np.linalg.norm(x[:2])
        v_w = rankine_wind(x[:2], v_max) + TURB_FRAC * float(v_tan(r, v_max)) * noise[k]
        if ctrl == 'pd':
            u = nominal_ff(vr, ar) + KP_PD * (pr - x[:2]) + KD_PD * (vr - x[2:])
        elif ctrl == 'hinf':
            u = nominal_ff(vr, ar) - K_hinf @ (x - xr)
        else:
            u = mpc.step(x, t, v_max)
        nu = np.linalg.norm(u)
        if nu > u_max:
            u = u * (u_max / nu)                       # thrust-cone saturation (all controllers)
        assert np.linalg.norm(u) <= u_max + 1e-9, 'thrust above the tilt limit'
        w = B_DRAG / M_DRONE * v_w                     # drag-induced disturbance acceleration
        x = AD @ x + BD @ u + ED @ w
        assert np.all(np.isfinite(x)) and np.linalg.norm(x[2:]) < 150.0, 'state blew up'
        pr1, _, _ = reference(t + DT)
        err[k] = np.linalg.norm(x[:2] - pr1)
        unorm[k] = np.linalg.norm(u)
        traj[k + 1] = x[:2]
        if record:
            rec['u'][k] = u
            rec['wind'][k] = v_w
            rec['ref'][k + 1] = pr1
    res = {
        'e_mean': err.mean(), 'e_peak': err.max(), 't_peak': (np.argmax(err) + 1) * DT,
        'effort': DT * unorm.sum(), 'sat_frac': np.mean(unorm >= u_max * (1 - 1e-6)),
        'success': bool(err.max() <= E_FAIL), 'r_final': float(np.linalg.norm(traj[-1])),
        'err': err, 'unorm': unorm, 'traj': traj,
    }
    if mpc is not None:
        res['admm_frac'] = mpc.n_admm / mpc.n_solves
    if record:
        res.update(rec)
    return res


def steady_thrust_along_reference(v_max=V_MAX, n=600):
    """|u| that keeps the drone exactly on the reference in the mean wind (no gusts)."""
    t = np.linspace(0, T_MISSION, n)
    p, v, a = reference(t)
    u = M_DRONE * a + B_DRAG * (v - rankine_wind(p, v_max))
    return t, np.linalg.norm(p, axis=1), np.linalg.norm(u, axis=1)


def run_simulation():
    K_hinf, gamma_star, gamma = hinf_synthesis()
    K_lqr, _ = lqr_gain()
    K_pd = np.hstack([KP_PD * np.eye(2), KD_PD * np.eye(2)])
    norms = {'pd': hinf_norm(K_pd), 'lqr': hinf_norm(K_lqr), 'hinf': hinf_norm(K_hinf)}
    seeds = [SEED0 + i for i in range(N_SEEDS)]
    main = {c: [simulate(c, s, K_hinf=K_hinf, record=(s == SHOW_SEED)) for s in seeds]
            for c in CTRL + ABLATIONS}
    sweep = {c: {} for c in CTRL}
    for vm in SWEEP_VMAX:
        for c in CTRL:
            if vm == V_MAX:
                sweep[c][vm] = main[c][:N_SEEDS_SWEEP]
            else:
                sweep[c][vm] = [simulate(c, SEED0 + i, v_max=vm, K_hinf=K_hinf)
                                for i in range(N_SEEDS_SWEEP)]
    # probe: same seeds with a larger thrust limit (no saturation at the eyewall)
    probe = {c: [simulate(c, s, K_hinf=K_hinf, u_max=PROBE_THRUST * U_MAX) for s in seeds]
             for c in CTRL}
    synth = {'K_hinf': K_hinf, 'gamma_star': gamma_star, 'gamma': gamma,
             'K_lqr': K_lqr, 'K_pd': K_pd, 'norms': norms}
    synth['probe'] = probe
    return main, sweep, synth


# ── Plots ─────────────────────────────────────────────────────────────────────

def _save(fig, out_dir, name):
    path = os.path.join(out_dir, name)
    fig.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def _speed_background(ax, lim=210, n=300, alpha=1.0):
    xs = np.linspace(-lim, lim, n)
    X, Y = np.meshgrid(xs, xs)
    S = np.linalg.norm(rankine_wind(np.stack([X, Y], axis=-1)), axis=-1)
    im = ax.pcolormesh(X, Y, S, cmap='Blues', shading='auto', alpha=alpha, vmin=0, vmax=V_MAX)
    return im


def plot_wind_field(out_dir):
    fig, ax = plt.subplots(figsize=(7.5, 6.5))
    im = _speed_background(ax)
    xs = np.linspace(-200, 200, 21)
    X, Y = np.meshgrid(xs, xs)
    W = rankine_wind(np.stack([X, Y], axis=-1))
    ax.quiver(X, Y, W[..., 0], W[..., 1], color='k', scale=900, width=0.003)
    ax.add_patch(Circle((0, 0), R_MAX, fill=False, ls='--', ec='grey', lw=1.5))
    ax.add_patch(Circle((0, 0), R_EYE, fc='green', alpha=0.6))
    ax.annotate('entry', xy=(R_START, 0), xytext=(R_START - 5, -60), color='red',
                arrowprops=dict(arrowstyle='->', color='red', lw=2), ha='center')
    ax.plot(R_START, 0, 'o', color='red', ms=8)
    fig.colorbar(im, ax=ax, label='wind speed (m/s)')
    ax.set_aspect('equal'); ax.set_xlim(-210, 210); ax.set_ylim(-210, 210)
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')
    ax.set_title('S058 Rankine vortex wind field\n(dashed: eyewall $r_{max}$ = 150 m; green: eye)')
    _save(fig, out_dir, 'wind_field.png')


def plot_wind_profile(out_dir):
    r = np.linspace(0, 200, 801)
    t, r_ref, u_req = steady_thrust_along_reference()
    fig, axs = plt.subplots(1, 2, figsize=(12, 4.5))
    ax = axs[0]
    ax.plot(r, v_tan(r), color='#1F77B4', lw=2, label=r'$v_{tan}(r)$')
    ax.plot(r, -v_rad(r), color='#D62728', lw=2, label=r'$|v_{rad}(r)|$')
    ax.fill_between(r, v_tan(r) * (1 - TURB_FRAC), v_tan(r) * (1 + TURB_FRAC), color='#1F77B4',
                    alpha=0.2, label=r'$v_{tan} \pm \sigma_w$ (10 %)')
    ax.plot(r, r * W_SPIRAL, 'k--', lw=1, label=r'reference tangential speed $r\,\omega$')
    ax.axvline(R_MAX, color='grey', ls=':')
    ax.set_xlabel('r (m)'); ax.set_ylabel('speed (m/s)')
    ax.set_title('Wind profile')
    ax.legend(fontsize=8)
    ax = axs[1]
    for vm, ls in zip(SWEEP_VMAX, [':', '-.', '--', '-', (0, (5, 1))]):
        _, rr, uu = steady_thrust_along_reference(vm)
        ax.plot(rr, uu, ls=ls, color='k', lw=1.8 if vm == V_MAX else 1.0,
                label=f'$v_{{max}}$ = {vm:.0f} m/s')
    ax.axhline(U_MAX, color='red', ls='--', label=f'thrust limit m g = {U_MAX:.2f} N')
    ax.axvline(R_MAX, color='grey', ls=':')
    ax.invert_xaxis()
    ax.set_xlabel('reference radius r (m)  (time runs left to right)')
    ax.set_ylabel('|u| needed in the mean wind (N)')
    ax.set_title('Thrust needed to stay exactly on the spiral (no gusts)')
    ax.legend(fontsize=8)
    fig.suptitle('S058 wind profile and thrust demand')
    _save(fig, out_dir, 'wind_profile.png')


def plot_trajectories(main, out_dir):
    fig, ax = plt.subplots(figsize=(8, 7))
    im = _speed_background(ax, alpha=0.6)
    ax.add_patch(Wedge((0, 0), R_MAX + 10, 0, 360, width=20, fc='grey', alpha=0.35))
    ax.add_patch(Circle((0, 0), R_EYE, fc='green', alpha=0.7))
    r0 = main['pd'][0]
    ax.plot(r0['ref'][:, 0], r0['ref'][:, 1], 'k--', lw=1.0, label='reference spiral')
    for c in CTRL:
        tr = main[c][0]['traj']
        ax.plot(tr[:, 0], tr[:, 1], color=COLORS[c], lw=1.4, label=LABELS[c])
    ax.plot(R_START, 0, 'o', color='red', ms=8, label='entry')
    fig.colorbar(im, ax=ax, label='wind speed (m/s)')
    ax.set_aspect('equal'); ax.set_xlim(-200, 200); ax.set_ylim(-200, 200)
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')
    ax.set_title(f'S058 inbound spiral, seed {SHOW_SEED} (grey annulus: eyewall)')
    ax.legend(loc='lower left', fontsize=8)
    _save(fig, out_dir, 'trajectories.png')


def plot_trajectories_3d(main, out_dir):
    fig = plt.figure(figsize=(9, 7.5))
    ax = fig.add_subplot(111, projection='3d')
    t = DT * np.arange(N_STEPS + 1)
    ref = main['pd'][0]['ref']
    ax.plot(ref[:, 0], ref[:, 1], t, 'k--', lw=0.8, label='reference')
    for c in CTRL:
        tr = main[c][0]['traj']
        ax.plot(tr[:, 0], tr[:, 1], t, color=COLORS[c], lw=1.2, label=LABELS[c])
    th = np.linspace(0, 2 * np.pi, 100)
    ax.plot(R_MAX * np.cos(th), R_MAX * np.sin(th), np.full_like(th, T_EYEWALL), color='grey',
            lw=1.5, label=f'eyewall r = {R_MAX:.0f} m at t = {T_EYEWALL:.0f} s')
    ax.plot(R_EYE * np.cos(th), R_EYE * np.sin(th), np.full_like(th, T_MISSION), color='green',
            lw=2, label='eye')
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)'); ax.set_zlabel('time (s)')
    ax.view_init(elev=24, azim=-55)
    ax.set_box_aspect(None, zoom=0.85)
    ax.set_title('S058 trajectories in space-time (horizontal plane vs time)')
    ax.legend(fontsize=8, loc='upper left')
    _save(fig, out_dir, 'trajectories_3d.png')


def plot_tracking_error(main, out_dir):
    fig, ax = plt.subplots(figsize=(11, 4.8))
    t = DT * np.arange(1, N_STEPS + 1)
    t_lo = (R_START - (R_MAX + SIGMA_RAD)) / V_INBOUND
    t_hi = (R_START - (R_MAX - SIGMA_RAD)) / V_INBOUND
    ax.axvspan(max(0, t_lo), t_hi, color='grey', alpha=0.15,
               label=rf'inflow layer $|r_{{ref}} - r_{{max}}| \leq \sigma_{{rad}}$')
    ax.axvline(T_EYEWALL, color='k', ls='--', lw=1, label=f'eyewall crossing t = {T_EYEWALL:.0f} s')
    for c in CTRL:
        ax.plot(t, main[c][0]['err'], color=COLORS[c], lw=1, label=LABELS[c])
    ax.set_yscale('log')
    ax.set_xlabel('time (s)'); ax.set_ylabel(r'$\|p - p_{ref}\|$ (m, log)')
    ax.set_title(f'S058 tracking error, seed {SHOW_SEED}')
    ax.legend(fontsize=8, ncol=2)
    ax.grid(alpha=0.3, which='both')
    _save(fig, out_dir, 'tracking_error.png')


def plot_control_inputs(main, out_dir):
    fig, axs = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    t = DT * np.arange(N_STEPS)
    for ax, c in zip(axs, CTRL):
        r = main[c][0]
        ax.plot(t, r['u'][:, 0], color='#1F77B4', lw=0.7, label='$u_x$')
        ax.plot(t, r['u'][:, 1], color='#D62728', lw=0.7, label='$u_y$')
        ax.plot(t, r['unorm'], color='k', lw=1.0, label=r'$\|u\|$')
        ax.axhline(U_MAX, color='k', ls='--', lw=0.8)
        ax.axvline(T_EYEWALL, color='grey', ls=':', lw=1)
        sat = r['unorm'] >= U_MAX * (1 - 1e-6)
        ax.plot(t[sat], np.full(sat.sum(), U_MAX + 1.0), '|', color=COLORS[c], ms=6)
        ax.set_ylabel('thrust (N)')
        ax.set_title(f"{LABELS[c]}: saturated {100 * r['sat_frac']:.1f}% of steps "
                     f"(ticks above the dashed limit m g)", fontsize=9)
        ax.set_ylim(-U_MAX - 2, U_MAX + 3)
    axs[0].legend(fontsize=8, ncol=3, loc='lower right')
    axs[-1].set_xlabel('time (s)')
    fig.suptitle(f'S058 control inputs, seed {SHOW_SEED}')
    _save(fig, out_dir, 'control_inputs.png')


def _stats(runs, key):
    a = np.array([r[key] for r in runs], float)
    return a.mean(), a.std()


def plot_comparison_bars(main, out_dir):
    ctrls = CTRL + ABLATIONS
    fig, axs = plt.subplots(1, 4, figsize=(16, 4.5))
    keys = [('e_mean', 'mean error (m)'), ('e_peak', 'peak error (m)'),
            ('effort', r'effort $J$ (N s)'), ('success', f'success rate (peak < {E_FAIL:.0f} m)')]
    x = np.arange(len(ctrls))
    for ax, (k, lab) in zip(axs, keys):
        m = [_stats(main[c], k)[0] for c in ctrls]
        s = [_stats(main[c], k)[1] for c in ctrls]
        ax.bar(x, m, yerr=None if k == 'success' else s, color=[COLORS[c] for c in ctrls],
               capsize=4, alpha=0.85)
        ax.set_xticks(x, [LABELS[c].replace(' + ', '\n+ ').replace(', ', ',\n') for c in ctrls],
                      fontsize=7)
        ax.set_title(lab)
        if k in ('e_mean', 'e_peak'):
            ax.set_yscale('log')
    fig.suptitle(f'S058 controller comparison, v_max = {V_MAX:.0f} m/s, {N_SEEDS} turbulence '
                 'seeds (mean ± std; last bar = MPC ablation)')
    _save(fig, out_dir, 'comparison_bars.png')


def plot_severity_sweep(sweep, out_dir):
    fig, axs = plt.subplots(1, 3, figsize=(15, 4.5))
    for c in CTRL:
        em = [_stats(sweep[c][vm], 'e_mean') for vm in SWEEP_VMAX]
        ep = [_stats(sweep[c][vm], 'e_peak') for vm in SWEEP_VMAX]
        sr = [_stats(sweep[c][vm], 'sat_frac')[0] for vm in SWEEP_VMAX]
        axs[0].errorbar(SWEEP_VMAX, [a for a, _ in em], [b for _, b in em], color=COLORS[c],
                        marker='o', capsize=3, label=LABELS[c])
        axs[1].errorbar(SWEEP_VMAX, [a for a, _ in ep], [b for _, b in ep], color=COLORS[c],
                        marker='o', capsize=3, label=LABELS[c])
        axs[2].plot(SWEEP_VMAX, [100 * s for s in sr], color=COLORS[c], marker='o', label=LABELS[c])
    axs[1].axhline(E_FAIL, color='red', ls='--', lw=1, label=f'track-loss threshold {E_FAIL:.0f} m')
    for ax, lab in zip(axs, ['mean error (m)', 'peak error (m)', 'saturated steps (%)']):
        ax.set_xlabel('$v_{max}$ (m/s)'); ax.set_ylabel(lab); ax.grid(alpha=0.3)
    axs[0].set_yscale('log'); axs[1].set_yscale('log')
    axs[0].legend(fontsize=8); axs[1].legend(fontsize=8)
    fig.suptitle(f'S058 wind-severity sweep ({N_SEEDS_SWEEP} seeds per point, mean ± std)')
    _save(fig, out_dir, 'severity_sweep.png')


def save_animation(main, out_dir):
    runs = {c: main[c][0] for c in CTRL}
    n = N_STEPS + 1
    n_frames = int(np.ceil((n - 1) / ANIM_STEP)) + 1
    idx = [min(f * ANIM_STEP, n - 1) for f in range(n_frames)]
    assert len(set(idx)) == n_frames
    fig, ax = plt.subplots(figsize=(6.5, 6.5))
    _speed_background(ax, n=100, alpha=0.5)
    ax.add_patch(Wedge((0, 0), R_MAX + 10, 0, 360, width=20, fc='grey', alpha=0.3))
    eye = Circle((0, 0), R_EYE, fc='lightgrey', ec='green', lw=1.5)
    ax.add_patch(eye)
    ref = runs['pd']['ref']
    ax.plot(ref[:, 0], ref[:, 1], 'k--', lw=0.6, alpha=0.6)
    trails = {c: ax.plot([], [], '-', color=COLORS[c], lw=1.2, label=LABELS[c])[0] for c in CTRL}
    dots = {c: ax.plot([], [], 'o', color=COLORS[c], ms=7, mec='k')[0] for c in CTRL}
    refdot, = ax.plot([], [], 'kx', ms=8)
    arrow = [None]
    ax.set_aspect('equal'); ax.set_xlim(-200, 200); ax.set_ylim(-200, 200)
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')
    ax.legend(loc='lower left', fontsize=7)
    title = ax.set_title('')

    def update(f):
        k = idx[f]
        for c in CTRL:
            tr = runs[c]['traj']
            trails[c].set_data(tr[max(0, k - 150):k + 1, 0], tr[max(0, k - 150):k + 1, 1])
            dots[c].set_data([tr[k, 0]], [tr[k, 1]])
        refdot.set_data([ref[k, 0]], [ref[k, 1]])
        if arrow[0] is not None:
            arrow[0].remove()
        p = runs['mpc']['traj'][k]
        wv = runs['mpc']['wind'][min(k, N_STEPS - 1)]
        arrow[0] = ax.annotate('', xy=p + 1.2 * wv, xytext=p,
                               arrowprops=dict(arrowstyle='->', color='navy', lw=1.5))
        done = k == n - 1 and runs['mpc']['success']
        eye.set_facecolor('limegreen' if done else 'lightgrey')
        r_ref = np.linalg.norm(ref[k])
        title.set_text(f'S058 typhoon eye probing  t = {k * DT:5.1f} s  r_ref = {r_ref:5.1f} m\n'
                       f'navy arrow: wind at the MPC drone (x1.2 s)')
        return list(trails.values()) + list(dots.values()) + [refdot, arrow[0], eye, title]

    anim = animation.FuncAnimation(fig, update, frames=n_frames, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    anim.save(path, writer='pillow', fps=10, dpi=65)
    plt.close(fig)
    print(f'Saved: {path} ({n_frames} frames, {ANIM_STEP} steps = {ANIM_STEP * DT:.1f} s each)')


# ── Main ──────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    out = os.path.normpath(OUTPUT_DIR)
    os.makedirs(out, exist_ok=True)
    main, sweep, synth = run_simulation()
    probe = synth['probe']

    print('\n=== S058 Typhoon Eye Probing — Results ===')
    print(f'  Mission: r {R_START:.0f} -> {R_EYE:.0f} m, T = {T_MISSION:.2f} s, '
          f'{N_STEPS} steps of {DT} s; eyewall crossing at t = {T_EYEWALL:.0f} s')
    print(f'  Thrust limit: min(F_max = {F_MAX_CARD:.0f} N, m g = {M_DRONE * G:.2f} N) = {U_MAX:.2f} N '
          f'-> max control acceleration {U_MAX / M_DRONE:.2f} m/s^2')
    rg = np.linspace(0.0, 200.0, 4001)
    wmag = np.linalg.norm(rankine_wind(np.column_stack([rg, np.zeros_like(rg)])), axis=1)
    peak_card_acc = float(wmag.max())
    entry_acc = float(np.linalg.norm(rankine_wind(np.array([R_START, 0.0]))))
    print(f'  Card dynamics check (m a = u + m f_wind, full wind incl. inflow): peak wind term '
          f'{peak_card_acc:.2f} m/s^2 at r = {rg[np.argmax(wmag)]:.1f} m '
          f'= {peak_card_acc / (U_MAX / M_DRONE):.2f} x the control authority; at entry '
          f'{entry_acc:.2f} m/s^2')
    _, rr, uu = steady_thrust_along_reference()
    print(f'  Drag model: thrust needed on the reference in mean wind: max {uu.max():.2f} N at '
          f'r = {rr[np.argmax(uu)]:.1f} m ({100 * uu.max() / U_MAX:.0f}% of m g); entry {uu[0]:.2f} N')
    for vm in SWEEP_VMAX:
        _, rr2, uu2 = steady_thrust_along_reference(vm)
        print(f'    v_max = {vm:.0f} m/s: max steady thrust {uu2.max():.2f} N '
              f'({100 * uu2.max() / U_MAX:.0f}% of m g)')
    print(f"  H-inf synthesis: gamma* = {synth['gamma_star']:.4f}, operating gamma = "
          f"{synth['gamma']:.4f}; |K_inf| pos gain {synth['K_hinf'][0, 0]:.2f} N/m, "
          f"vel gain {synth['K_hinf'][0, 2]:.2f} N s/m")
    print(f"  LQR (MPC unconstrained) gain: pos {synth['K_lqr'][0, 0]:.2f} N/m, "
          f"vel {synth['K_lqr'][0, 2]:.2f} N s/m; PD: {KP_PD} N/m, {KD_PD} N s/m")
    print('  Closed-loop H-inf norm w -> z = [C_z e; R^1/2 u]: ' +
          ', '.join(f'{k} {v:.4f}' for k, v in synth['norms'].items()))
    print(f'\n  --- v_max = {V_MAX:.0f} m/s, {N_SEEDS} turbulence seeds (CRN), mean +/- std ---')
    ctrls = CTRL + ABLATIONS
    for c in ctrls:
        rs = main[c]
        em, es = _stats(rs, 'e_mean'); pm, ps = _stats(rs, 'e_peak')
        jm, js = _stats(rs, 'effort'); sm, _ = _stats(rs, 'sat_frac')
        nsucc = sum(r['success'] for r in rs)
        rf = np.mean([r['r_final'] for r in rs])
        extra = f", ADMM active {100 * np.mean([r['admm_frac'] for r in rs]):.1f}% of steps" \
            if 'admm_frac' in rs[0] else ''
        print(f'  [{LABELS[c]}]')
        print(f'    mean error {em:.3f} +/- {es:.3f} m, peak {pm:.2f} +/- {ps:.2f} m, '
              f'effort {jm:.0f} +/- {js:.0f} N s')
        print(f'    saturated {100 * sm:.1f}% of steps{extra}; success {nsucc}/{len(rs)}; '
              f'final distance to eye centre {rf:.2f} m')
    em_all = {c: np.array([r['e_mean'] for r in main[c]]) for c in ctrls}
    ep_all = {c: np.array([r['e_peak'] for r in main[c]]) for c in ctrls}
    wins = {c: 0 for c in CTRL}
    for i in range(N_SEEDS):
        wins[min(CTRL, key=lambda c: em_all[c][i])] += 1
    print('  Per-seed wins on mean error (3 main controllers): ' +
          ', '.join(f'{LABELS[c]} {wins[c]}' for c in CTRL))
    print(f"  MPC beats H-inf on peak error in {int(np.sum(ep_all['mpc'] < ep_all['hinf']))}/{N_SEEDS} seeds; "
          f"H-inf beats PD on mean error in {int(np.sum(em_all['hinf'] < em_all['pd']))}/{N_SEEDS}")
    s0 = {c: main[c][0] for c in CTRL}
    print(f'  Seed {SHOW_SEED} peaks: ' + ', '.join(
        f"{LABELS[c]} {s0[c]['e_peak']:.2f} m at t = {s0[c]['t_peak']:.1f} s" for c in CTRL))
    print('  Per-seed peak error (m) PD / MPC / H-inf, saturated steps (MPC run):')
    for i in range(N_SEEDS):
        print(f"    seed {SEED0 + i}: {main['pd'][i]['e_peak']:.2f} / {main['mpc'][i]['e_peak']:.2f} / "
              f"{main['hinf'][i]['e_peak']:.2f}, saturated {100 * main['mpc'][i]['sat_frac']:.1f}%")
    print(f'\n  --- probe: thrust limit x{PROBE_THRUST:.0f} ({PROBE_THRUST * U_MAX:.2f} N), '
          f'same {N_SEEDS} seeds ---')
    for c in CTRL:
        rs = probe[c]
        em, es = _stats(rs, 'e_mean'); pm, ps = _stats(rs, 'e_peak'); sm, _ = _stats(rs, 'sat_frac')
        print(f'  [{LABELS[c]}] mean error {em:.3f} +/- {es:.3f} m, peak {pm:.2f} +/- {ps:.2f} m, '
              f'saturated {100 * sm:.1f}%, success {sum(r["success"] for r in rs)}/{len(rs)}')
    print(f'\n  --- severity sweep, {N_SEEDS_SWEEP} seeds per point ---')
    for vm in SWEEP_VMAX:
        parts = []
        for c in CTRL:
            em, _ = _stats(sweep[c][vm], 'e_mean'); pm, _ = _stats(sweep[c][vm], 'e_peak')
            sm, _ = _stats(sweep[c][vm], 'sat_frac')
            ns = sum(r['success'] for r in sweep[c][vm])
            parts.append(f'{c}: mean {em:.3f} / peak {pm:.2f} m, sat {100 * sm:.1f}%, '
                         f'success {ns}/{N_SEEDS_SWEEP}')
        print(f'  v_max {vm:.0f}: ' + ' | '.join(parts))
    print()

    plot_wind_field(out)
    plot_wind_profile(out)
    plot_trajectories(main, out)
    plot_trajectories_3d(main, out)
    plot_tracking_error(main, out)
    plot_control_inputs(main, out)
    plot_comparison_bars(main, out)
    plot_severity_sweep(sweep, out)
    save_animation(main, out)
