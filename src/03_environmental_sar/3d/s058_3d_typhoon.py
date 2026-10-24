"""
S058 3D Upgrade — Typhoon Eye Penetration
=========================================
The S058 spiral (r 190 -> 15 m, 0.8 m/s inward, 0.12 rad/s, 218.75 s) is flown as a
helical descent from 500 m to 50 m through a 3D Rankine vortex:
  * tangential and radial wind scaled by a vertical envelope Phi(z) (1 below the
    200 m boundary-layer top, Gaussian decay sigma_z = 300 m above it);
  * eyewall updraft w_max sin(pi z / 600 m) and eye subsidence;
  * 3D Ornstein-Uhlenbeck gusts with std 10 % of the local tangential wind.
The wind acts through drag, m a = u - m g e_z + b (v_wind + v_gust - v) (S058 fix).
Thrust: |u_xy| <= m g, u_z in [0.7, 1.3] m g (card's decoupled limits; the tilt is 45 deg at u_z = m g
and up to 55 deg at u_z = 0.7 m g). Ablation: tilt cone |u_xy| <= min(m g, u_z) for PD and H-inf.

Controllers (S058, extended to 6 states): PD with still-air + gravity feed-forward,
condensed MPC over 20 steps with wind feed-forward and the thrust constraints (ADMM),
H-infinity game-Riccati state feedback; MPC without wind feed-forward as an ablation.

Altitude plans (same turbulence seeds for every run, CRN):
  helix     card: linear descent 500 -> 50 m
  flat50    2D baseline (X5): the S058 spiral at a fixed 50 m (boundary layer)
  flat500   the spiral at a fixed 500 m (weaker wind, no sounding)
  ablations helix with Phi = 1 (no vertical decay) and helix without vertical wind

Usage:
    conda activate drones
    python src/03_environmental_sar/3d/s058_3d_typhoon.py
"""

import os
for _v in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[_v] = '1'          # single-threaded BLAS (as in S058)
import math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401
from scipy.linalg import expm, solve_discrete_are

_HERE = os.path.dirname(os.path.abspath(__file__))
OUTPUT_DIR = os.path.normpath(os.path.join(
    _HERE, '..', '..', '..', 'outputs', '03_environmental_sar', '3d', 's058_3d_typhoon'))

# ── Parameters (card / S058) ──────────────────────────────────────────────────
M_DRONE, G, B_DRAG = 1.5, 9.81, 0.4
F_MAX_CARD = 30.0
U_XY_MAX   = M_DRONE * G                      # card: m g tan 45 deg (decoupled from u_z)
DUZ        = 0.3 * M_DRONE * G
UZ_MIN, UZ_MAX = M_DRONE * G - DUZ, M_DRONE * G + DUZ
R_MAX, V_MAX, V_INFLOW, SIGMA_RAD = 150.0, 50.0, 8.0, 40.0
Z_BL, SIGMA_Z = 200.0, 300.0
W_MAX, W_SUB = 5.0, 2.0
H_DOM      = 600.0                            # domain top (card's "z_top = 600 m" in w)
TURB_FRAC  = 0.10
R_START, R_EYE = 190.0, 15.0
Z_TOP, Z_BOT = 500.0, 50.0                    # entry and floor altitude
V_INBOUND, W_SPIRAL = 0.8, 0.12
DT, N_PRED = 0.1, 20
Q_MPC = np.diag([10.0, 10.0, 10.0, 1.0, 1.0, 1.0])
R_MPC = np.diag([0.1, 0.1, 0.05])
KP_PD, KD_PD = 4.0, 2.5
WPT_DZ, WPT_TOL_Z, WPT_TOL_R = 10.0, 5.0, 20.0

# ── Parameters (added; S058 fixes) ────────────────────────────────────────────
TAU_TURB = 2.0
E_FAIL = R_EYE
GAMMA_MARGIN = 1.10
N_SEEDS, SEED0 = 12, 100
SHOW_SEED = SEED0
SWEEP_VMAX = [30.0, 40.0, 50.0, 60.0, 70.0]
N_SEEDS_SWEEP = 5
ADMM_ITERS, ADMM_TOL = 150, 1e-3
ANIM_STEP = 40

T_MISSION = (R_START - R_EYE) / V_INBOUND        # 218.75 s
N_STEPS = int(np.ceil(T_MISSION / DT - 1e-9))
V_DESCENT = (Z_TOP - Z_BOT) / T_MISSION
T_EYEWALL = (R_START - R_MAX) / V_INBOUND

CTRL = ['pd', 'mpc', 'hinf']
LABELS = {'pd': 'PD baseline', 'mpc': 'MPC + wind feed-forward', 'hinf': 'H-infinity',
          'mpc_nowind': 'MPC, no wind feed-forward', 'mpc_abscost': 'MPC, card cost on absolute u'}
COLORS = {'pd': '#FF7F0E', 'mpc': '#2CA02C', 'hinf': '#9467BD', 'mpc_nowind': '#8C564B', 'mpc_abscost': '#BCBD22'}
PROFILES = {'helix': 'Helix 500 -> 50 m (card)', 'flat50': 'Fixed 50 m (2D baseline)',
            'flat500': 'Fixed 500 m', 'helix_nodecay': 'Helix, Phi = 1 (ablation)',
            'helix_noupdraft': 'Helix, no vertical wind (ablation)'}


# ── Wind ─────────────────────────────────────────────────────────────────────

def phi_z(z, decay=True):
    z = np.asarray(z, float)
    if not decay:
        return np.ones_like(z)
    return np.where(z <= Z_BL, 1.0, np.exp(-(z - Z_BL) ** 2 / (2 * SIGMA_Z ** 2)))


def rankine(r, v_max=V_MAX):
    r = np.asarray(r, float)
    return np.where(r <= R_MAX, v_max * r / R_MAX, v_max * R_MAX / np.maximum(r, 1e-9))


def w_vert(r, z):
    r = np.asarray(r, float)
    s = np.sin(np.pi * np.clip(z, 0, H_DOM) / H_DOM)
    w = W_MAX * np.exp(-(r - R_MAX) ** 2 / (2 * SIGMA_RAD ** 2)) * s
    return w - np.where(r < R_EYE, W_SUB * (1 - r / R_EYE), 0.0) * s


def wind(p, v_max=V_MAX, decay=True, updraft=True):
    """3D mean wind velocity at positions p (..., 3)."""
    p = np.asarray(p, float)
    r = np.hypot(p[..., 0], p[..., 1])
    rs = np.maximum(r, 1e-9)
    ph = phi_z(p[..., 2], decay)
    vt = ph * rankine(r, v_max)
    vr = -ph * V_INFLOW * np.exp(-(r - R_MAX) ** 2 / (2 * SIGMA_RAD ** 2))
    out = np.zeros(p.shape)
    out[..., 0] = (vt * -p[..., 1] + vr * p[..., 0]) / rs
    out[..., 1] = (vt * p[..., 0] + vr * p[..., 1]) / rs
    if updraft:
        out[..., 2] = w_vert(r, p[..., 2])
    return out


# ── Reference ─────────────────────────────────────────────────────────────────

def z_profile(profile, t):
    t = np.asarray(t, float)
    if profile.startswith('helix'):
        return np.maximum(Z_BOT, Z_TOP - V_DESCENT * t), np.where(t < T_MISSION, -V_DESCENT, 0.0)
    z0 = 50.0 if profile == 'flat50' else 500.0
    return np.full(t.shape, z0), np.zeros(t.shape)


def reference(profile, t):
    t = np.asarray(t, float)
    r = np.maximum(R_EYE, R_START - V_INBOUND * t)
    rdot = np.where(R_START - V_INBOUND * t > R_EYE, -V_INBOUND, 0.0)
    th = W_SPIRAL * t
    c, s = np.cos(th), np.sin(th)
    z, zd = z_profile(profile, t)
    p = np.stack([r * c, r * s, z], axis=-1)
    v = np.stack([rdot * c - r * W_SPIRAL * s, rdot * s + r * W_SPIRAL * c, zd], axis=-1)
    ar = -r * W_SPIRAL ** 2
    at = 2 * rdot * W_SPIRAL
    a = np.stack([ar * c - at * s, ar * s + at * c, np.zeros(t.shape)], axis=-1)
    return p, v, a


def profile_wind_flags(profile):
    return (profile != 'helix_nodecay'), (profile != 'helix_noupdraft')


_REF = {}


def ref_table(profile):
    """Reference sampled at k * DT, k = 0 .. N_STEPS + N_PRED + 1 (cached; same values as reference())."""
    if profile not in _REF:
        p, v, a = reference(profile, np.arange(N_STEPS + N_PRED + 2) * DT)
        _REF[profile] = (p, v, a, np.hstack([p, v]))
    return _REF[profile]


def wind_point(x, y, z, v_max, decay, updraft):
    """Scalar version of wind() for the simulation loop (same formulas)."""
    r = math.hypot(x, y)
    rs = max(r, 1e-9)
    ph = 1.0 if (not decay or z <= Z_BL) else math.exp(-(z - Z_BL) ** 2 / (2 * SIGMA_Z ** 2))
    rk = v_max * r / R_MAX if r <= R_MAX else v_max * R_MAX / rs
    gr = math.exp(-(r - R_MAX) ** 2 / (2 * SIGMA_RAD ** 2))
    vt, vr = ph * rk, -ph * V_INFLOW * gr
    w = 0.0
    if updraft:
        s = math.sin(math.pi * min(max(z, 0.0), H_DOM) / H_DOM)
        w = W_MAX * gr * s - (W_SUB * (1 - r / R_EYE) * s if r < R_EYE else 0.0)
    return np.array([(vt * -y + vr * x) / rs, (vt * x + vr * y) / rs, w]), TURB_FRAC * ph * rk


# ── Linear model and controllers ─────────────────────────────────────────────

def build_discrete_ss(dt=DT):
    A = np.block([[np.zeros((3, 3)), np.eye(3)], [np.zeros((3, 3)), -(B_DRAG / M_DRONE) * np.eye(3)]])
    B = np.vstack([np.zeros((3, 3)), np.eye(3) / M_DRONE])
    E = np.vstack([np.zeros((3, 3)), np.eye(3)])
    Mb = np.zeros((12, 12))
    Mb[:6, :6] = A
    Mb[:6, 6:9] = B
    Mb[:6, 9:12] = E
    Me = expm(Mb * dt)
    return Me[:6, :6], Me[:6, 6:9], Me[:6, 9:12]


AD, BD, ED = build_discrete_ss()
QZ = np.diag([10.0, 10.0, 10.0, 0, 0, 0])
CZ = np.sqrt(QZ)


def lqr_gain():
    P = solve_discrete_are(AD, BD, Q_MPC, R_MPC)
    return np.linalg.solve(R_MPC + BD.T @ P @ BD, BD.T @ P @ AD), P


def hinf_riccati(gamma, iters=20000, tol=1e-10):
    Bt = np.hstack([BD, ED])
    Rg = np.block([[R_MPC, np.zeros((3, 3))], [np.zeros((3, 3)), -gamma ** 2 * np.eye(3)]])
    X = QZ.copy()
    for _ in range(iters):
        if np.min(np.linalg.eigvalsh(gamma ** 2 * np.eye(3) - ED.T @ X @ ED)) <= 0:
            return None
        S = Rg + Bt.T @ X @ Bt
        Xn = QZ + AD.T @ X @ AD - AD.T @ X @ Bt @ np.linalg.solve(S, Bt.T @ X @ AD)
        Xn = 0.5 * (Xn + Xn.T)
        if not np.all(np.isfinite(Xn)) or np.max(np.abs(Xn)) > 1e12:
            return None
        if np.max(np.abs(Xn - X)) < tol * max(1.0, np.max(np.abs(X))):
            X = Xn
            break
        X = Xn
    else:
        return None
    if np.min(np.linalg.eigvalsh(X)) < -1e-9 or \
            np.min(np.linalg.eigvalsh(gamma ** 2 * np.eye(3) - ED.T @ X @ ED)) <= 0:
        return None
    K = np.linalg.solve(Rg + Bt.T @ X @ Bt, Bt.T @ X @ AD)[:3]
    if np.max(np.abs(np.linalg.eigvals(AD - BD @ K))) >= 1.0:
        return None
    return K, X


def hinf_norm(K, n_freq=3000, rows=None):
    Acl = AD - BD @ K
    Cz = np.vstack([CZ, np.sqrt(R_MPC) @ (-K)])
    best = 0.0
    for om in np.linspace(0, np.pi, n_freq):
        T = Cz @ np.linalg.solve(np.exp(1j * om) * np.eye(6) - Acl, ED)
        if rows is not None:
            T = T[:, rows]
        best = max(best, np.linalg.svd(T, compute_uv=False)[0])
    return best


def hinf_synthesis():
    K_lqr, _ = lqr_gain()
    hi = hinf_norm(K_lqr) * 1.5
    assert hinf_riccati(hi) is not None
    lo = 1e-3
    for _ in range(60):
        mid = np.sqrt(lo * hi)
        if hinf_riccati(mid) is None:
            lo = mid
        else:
            hi = mid
        if hi / lo < 1.0005:
            break
    K, _ = hinf_riccati(GAMMA_MARGIN * hi)
    return K, hi, GAMMA_MARGIN * hi


U_HOVER = np.array([0.0, 0.0, M_DRONE * G])


def saturate(u, cone=False):
    """Card limits; cone=True also caps the tilt at 45 deg of the actual vertical thrust."""
    u = u.copy()
    u[2] = min(max(u[2], UZ_MIN), UZ_MAX)
    lim = min(U_XY_MAX, u[2]) if cone else U_XY_MAX
    n = np.hypot(u[0], u[1])
    if n > lim:
        u[:2] *= lim / n
    return u


class MPC:
    """Condensed QP: cost sum ||x - x_ref||_Q + ||u - u_hover||_R + terminal DARE weight,
    constraints |u_xy| <= m g and u_z in [0.7, 1.3] m g, solved with ADMM when active."""

    def __init__(self, mode, profile, v_max):
        self.mode, self.profile, self.v_max = mode, profile, v_max
        self.decay, self.updraft = profile_wind_flags(profile)
        n, m, N = 6, 3, N_PRED
        _, P = lqr_gain()
        pw = [np.eye(n)]
        for _ in range(N):
            pw.append(pw[-1] @ AD)
        self.Phi = np.zeros((N * n, n))
        self.Gam = np.zeros((N * n, N * m))
        self.Psi = np.zeros((N * n, N * m))
        for j in range(N):
            self.Phi[j * n:(j + 1) * n] = pw[j + 1]
            for i in range(j + 1):
                self.Gam[j * n:(j + 1) * n, i * m:(i + 1) * m] = pw[j - i] @ BD
                self.Psi[j * n:(j + 1) * n, i * m:(i + 1) * m] = pw[j - i] @ ED
        Qb = np.kron(np.eye(N), Q_MPC)
        Qb[-n:, -n:] = P
        self.GQ = self.Gam.T @ Qb
        self.GQPhi = self.GQ @ self.Phi
        self.GQPsi = self.GQ @ self.Psi
        Rb = np.kron(np.eye(N), R_MPC)
        # cost on u - u_hover; 'mpc_abscost' is the card's literal cost on the absolute thrust (probe)
        self.Rb_uh = np.zeros(N * m) if mode == 'mpc_abscost' else Rb @ np.tile(U_HOVER, N)
        H = self.Gam.T @ Qb @ self.Gam + Rb
        ev = np.linalg.eigvalsh(H)
        self.rho = float(np.sqrt(ev[0] * ev[-1]))
        self.Hinv = np.linalg.inv(H)
        self.Minv = np.linalg.inv(H + self.rho * np.eye(N * m))
        self.U = np.tile(U_HOVER, N)
        self.pred = None
        self.n_admm = self.n_solves = 0

    @staticmethod
    def project(U):
        u = U.reshape(-1, 3).copy()
        n = np.hypot(u[:, 0], u[:, 1])
        s = np.where(n > U_XY_MAX, U_XY_MAX / np.maximum(n, 1e-12), 1.0)
        u[:, :2] *= s[:, None]
        u[:, 2] = np.clip(u[:, 2], UZ_MIN, UZ_MAX)
        return u.ravel()

    def step(self, x, k):
        N = N_PRED
        Xr = ref_table(self.profile)[3][k + 1:k + N + 1].ravel()
        grav = np.tile([0.0, 0.0, -G], N)
        if self.mode == 'mpc_nowind':
            W = grav
        else:
            pp = np.vstack([x[None, :3], self.pred[1:]]) if self.pred is not None else np.repeat(x[None, :3], N, 0)
            W = (B_DRAG / M_DRONE * wind(pp[:N], self.v_max, self.decay, self.updraft)).ravel() + grav
        g = self.GQPhi @ x + self.GQPsi @ W - self.GQ @ Xr - self.Rb_uh
        U = -self.Hinv @ g
        self.n_solves += 1
        u3 = U.reshape(-1, 3)
        if np.any(np.hypot(u3[:, 0], u3[:, 1]) > U_XY_MAX) or np.any(u3[:, 2] < UZ_MIN) or np.any(u3[:, 2] > UZ_MAX):
            Z = self.project(np.concatenate([self.U[3:], self.U[-3:]]))
            Y = np.zeros_like(Z)
            for _ in range(ADMM_ITERS):
                U = self.Minv @ (self.rho * (Z - Y) - g)
                Zn = self.project(U + Y)
                Y = Y + U - Zn
                rp, rd = np.max(np.abs(U - Zn)), self.rho * np.max(np.abs(Zn - Z))
                Z = Zn
                if rp < ADMM_TOL and rd < ADMM_TOL:
                    break
            self.n_admm += 1
            U = Z
        self.U = U
        X = (self.Phi @ x + self.Gam @ U + self.Psi @ W).reshape(N, 6)
        self.pred = np.vstack([X[:, :3], X[-1:, :3]])
        return U[:3].copy()


# ── Simulation ────────────────────────────────────────────────────────────────

def turbulence_noise(seed, n=N_STEPS):
    rng = np.random.default_rng(seed)
    phi = np.exp(-DT / TAU_TURB)
    xi = rng.standard_normal((n, 3))
    out = np.zeros((n, 3))
    s = rng.standard_normal(3)
    for k in range(n):
        out[k] = s
        s = phi * s + np.sqrt(1 - phi ** 2) * xi[k]
    return out


def waypoints():
    zs = np.arange(Z_TOP, Z_BOT - 1e-9, -WPT_DZ)
    t = (Z_TOP - zs) / V_DESCENT
    r = np.maximum(R_EYE, R_START - V_INBOUND * t)
    return zs, r


WPT_Z, WPT_R = waypoints()


def simulate(ctrl, profile, seed, v_max=V_MAX, K_hinf=None, record=False):
    noise = turbulence_noise(seed)
    decay, updraft = profile_wind_flags(profile)
    P_r, V_r, A_r, X_r = ref_table(profile)
    x = X_r[0].copy()
    gvec = np.array([0.0, 0.0, -G])
    cone = ctrl.endswith('_cone')
    ctrl = ctrl.replace('_cone', '')
    mpc = MPC(ctrl, profile, v_max) if ctrl.startswith('mpc') else None
    tilt_max = 0.0
    err3, errxy, errz, sat, ezs = (np.zeros(N_STEPS) for _ in range(5))
    traj = np.zeros((N_STEPS + 1, 3))
    traj[0] = x[:3]
    U = np.zeros((N_STEPS, 3))
    for k in range(N_STEPS):
        pr, vr, ar, xr = P_r[k], V_r[k], A_r[k], X_r[k]
        v_mean, sig = wind_point(x[0], x[1], x[2], v_max, decay, updraft)
        v_w = v_mean + sig * noise[k]
        ff = M_DRONE * ar + B_DRAG * vr + U_HOVER
        if ctrl == 'pd':
            u = ff + KP_PD * (pr - x[:3]) + KD_PD * (vr - x[3:])
        elif ctrl == 'hinf':
            u = ff - K_hinf @ (x - xr)
        else:
            u = mpc.step(x, k)
        us = saturate(u, cone)
        lim = min(U_XY_MAX, us[2]) if cone else U_XY_MAX
        sat[k] = float(np.any(np.abs(us - u) > 1e-9) or np.hypot(us[0], us[1]) >= lim * (1 - 1e-6)
                       or us[2] <= UZ_MIN + 1e-9 or us[2] >= UZ_MAX - 1e-9)
        u = us
        assert np.hypot(u[0], u[1]) <= lim + 1e-9 and UZ_MIN - 1e-9 <= u[2] <= UZ_MAX + 1e-9
        tilt_max = max(tilt_max, math.degrees(math.atan2(math.hypot(u[0], u[1]), u[2])))
        w = B_DRAG / M_DRONE * v_w + gvec
        x = AD @ x + BD @ u + ED @ w
        assert np.all(np.isfinite(x)) and np.linalg.norm(x[3:]) < 150.0, 'state blew up'
        assert 0.0 < x[2] < H_DOM, 'drone left the modelled layer (sea surface to 600 m domain top)'
        d = x[:3] - P_r[k + 1]
        err3[k], errxy[k], errz[k], ezs[k] = np.linalg.norm(d), np.hypot(d[0], d[1]), abs(d[2]), d[2]
        traj[k + 1] = x[:3]
        U[k] = u
    rr = np.hypot(traj[:, 0], traj[:, 1])
    if profile.startswith('helix'):
        ok = [(np.any((np.abs(traj[:, 2] - zw) < WPT_TOL_Z) & (np.abs(rr - rw) < WPT_TOL_R)))
              for zw, rw in zip(WPT_Z, WPT_R)]
        compl = float(np.mean(ok))
    else:
        compl = float('nan')
    kz = int(np.argmax(errz))
    res = {'ez_signed': ezs.mean(), 'e3': err3.mean(), 'exy': errxy.mean(), 'ez': errz.mean(), 'peak': err3.max(),
           'peak_xy': errxy.max(), 'peak_z': errz.max(), 't_peak': (np.argmax(err3) + 1) * DT,
           't_peak_z': (kz + 1) * DT, 'r_peak_z': float(rr[kz + 1]), 'z_peak_z': float(traj[kz + 1, 2]),
           'effort': DT * np.linalg.norm(U, axis=1).sum(), 'sat': sat.mean(),
           'success': bool(err3.max() <= E_FAIL), 'compl': compl, 'z_min': float(traj[:, 2].min()),
           'umax': float(np.linalg.norm(U, axis=1).max()), 'tilt_max': tilt_max}
    if mpc is not None:
        res['admm'] = mpc.n_admm / mpc.n_solves
    if record:
        res.update({'traj': traj, 'err3': err3, 'errxy': errxy, 'errz': errz, 'U': U})
    return res


def steady_thrust(profile, v_max=V_MAX, n=800):
    """Thrust that keeps the drone on the reference in the mean wind (no gusts)."""
    t = np.linspace(0, T_MISSION, n)
    decay, updraft = profile_wind_flags(profile)
    p, v, a = reference(profile, t)
    u = M_DRONE * a + B_DRAG * (v - wind(p, v_max, decay, updraft)) + U_HOVER
    return t, np.hypot(u[:, 0], u[:, 1]), u[:, 2], p


def run_simulation():
    K_hinf, gstar, gamma = hinf_synthesis()
    seeds = [SEED0 + i for i in range(N_SEEDS)]
    main = {}
    for prof in PROFILES:
        ctrls = CTRL + (['mpc_nowind'] if prof in ('helix', 'flat50') else [])
        for c in ctrls:
            main[(prof, c)] = [simulate(c, prof, s, K_hinf=K_hinf,
                                        record=(s == SHOW_SEED and prof in ('helix', 'flat50')))
                               for s in seeds]
    main[('helix', 'mpc_abscost')] = [simulate('mpc_abscost', 'helix', s, K_hinf=K_hinf) for s in seeds]
    for prof in ('helix', 'flat50', 'helix_nodecay'):                # tilt-cone ablation
        for c in ('pd', 'hinf'):
            main[(prof, c + '_cone')] = [simulate(c + '_cone', prof, s, K_hinf=K_hinf) for s in seeds]
    sweep = {}
    for vm in SWEEP_VMAX:
        for prof in ('helix', 'flat50'):
            for c in CTRL:
                sweep[(vm, prof, c)] = (main[(prof, c)][:N_SEEDS_SWEEP] if vm == V_MAX else
                                        [simulate(c, prof, SEED0 + i, v_max=vm, K_hinf=K_hinf)
                                         for i in range(N_SEEDS_SWEEP)])
    for c in CTRL:                                                   # updraft probe at v_max = 40 m/s
        main[('helix', c, 40.0)] = sweep[(40.0, 'helix', c)]          # same runs as the sweep (reused)
        main[('flat50', c, 40.0)] = sweep[(40.0, 'flat50', c)]
        main[('helix_noupdraft', c, 40.0)] = [simulate(c, 'helix_noupdraft', SEED0 + i, v_max=40.0, K_hinf=K_hinf)
                                              for i in range(N_SEEDS_SWEEP)]
    return main, sweep, {'K': K_hinf, 'gstar': gstar, 'gamma': gamma}


# ── Plots ─────────────────────────────────────────────────────────────────────

def _save(fig, out_dir, name):
    path = os.path.join(out_dir, name)
    fig.savefig(path, dpi=120, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def _cyl(ax, r, z0, z1, color, alpha):
    th = np.linspace(0, 2 * np.pi, 40)
    for z in np.linspace(z0, z1, 6):
        ax.plot(r * np.cos(th), r * np.sin(th), np.full_like(th, z), color=color, alpha=alpha, lw=0.7)


def plot_helix_3d(main, out_dir):
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    t = np.arange(N_STEPS + 1) * DT
    pr, _, _ = reference('helix', t)
    ax.plot(pr[:, 0], pr[:, 1], pr[:, 2], 'k--', lw=1, label='reference helix')
    for c in CTRL:
        tr = main[('helix', c)][0]['traj']
        ax.plot(tr[:, 0], tr[:, 1], tr[:, 2], color=COLORS[c], lw=1.1, label=LABELS[c])
    _cyl(ax, R_EYE, 0, 500, 'green', 0.5)
    _cyl(ax, R_MAX, 0, 500, 'red', 0.25)
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)'); ax.set_zlabel('z (m)')
    ax.view_init(elev=22, azim=-55)
    ax.legend(fontsize=8)
    ax.set_title(f'S058 3D Helical descent, seed {SHOW_SEED} (green = eye r = 15 m, red = eyewall r = 150 m)', fontsize=10)
    _save(fig, out_dir, 'helix_3d.png')


def plot_top_down(main, out_dir):
    fig, axes = plt.subplots(1, 2, figsize=(14, 6.4))
    g = np.linspace(-210, 210, 211)
    X, Y = np.meshgrid(g, g)
    for ax, prof, zb in zip(axes, ['helix', 'flat50'], [50.0, 50.0]):
        P = np.stack([X, Y, np.full_like(X, zb)], axis=-1)
        sp = np.linalg.norm(wind(P)[..., :2], axis=-1)
        im = ax.pcolormesh(X, Y, sp, cmap='Blues', shading='auto', vmin=0, vmax=55)
        t = np.arange(N_STEPS + 1) * DT
        pr, _, _ = reference(prof, t)
        ax.plot(pr[:, 0], pr[:, 1], 'k--', lw=1, label='reference')
        for c in CTRL:
            tr = main[(prof, c)][0]['traj']
            ax.plot(tr[:, 0], tr[:, 1], color=COLORS[c], lw=1, label=LABELS[c])
        ax.set_aspect('equal')
        ax.set_title(f'{PROFILES[prof]}, seed {SHOW_SEED}\nbackground: horizontal wind speed at z = 50 m', fontsize=10)
        ax.legend(fontsize=7, loc='lower left')
    fig.colorbar(im, ax=axes, shrink=0.8, label='|v_xy| at 50 m (m/s)')
    _save(fig, out_dir, 'top_down.png')


def plot_altitude_errors(main, out_dir):
    fig, axes = plt.subplots(3, 2, figsize=(15, 10), sharex=True)
    t = (np.arange(N_STEPS) + 1) * DT
    for col, prof in enumerate(['helix', 'flat50']):
        ax = axes[0, col]
        pr, _, _ = reference(prof, np.arange(N_STEPS + 1) * DT)
        ax.plot(np.arange(N_STEPS + 1) * DT, pr[:, 2], 'k--', label='reference')
        for c in CTRL:
            ax.plot(np.arange(N_STEPS + 1) * DT, main[(prof, c)][0]['traj'][:, 2], color=COLORS[c], lw=1, label=LABELS[c])
        if prof == 'helix':
            for (z0, z1, col_), lab in zip([(300, 500, '#DDEEFF'), (150, 300, '#FFF2CC'), (50, 150, '#FDDDDD')],
                                            ['upper 500-300 m', 'eyewall 300-150 m', 'boundary layer 150-50 m']):
                ax.axhspan(z0, z1, color=col_, alpha=0.6, zorder=0)
                ax.text(200, (z0 + z1) / 2, lab, fontsize=7, ha='right')
        ax.set_ylabel('z (m)')
        ax.set_title(PROFILES[prof], fontsize=10)
        ax.legend(fontsize=7)
        for row, key, yl in [(1, 'errxy', 'horizontal error e_xy (m)'), (2, 'errz', 'vertical error e_z (m)')]:
            ax = axes[row, col]
            for c in CTRL:
                ax.semilogy(t, np.maximum(main[(prof, c)][0][key], 1e-4), color=COLORS[c], lw=0.9, label=LABELS[c])
            ax.axvspan((R_START - R_MAX - SIGMA_RAD) / V_INBOUND, (R_START - R_MAX + SIGMA_RAD) / V_INBOUND,
                       color='grey', alpha=0.15, label='reference in eyewall inflow/updraft band')
            ax.set_ylabel(yl)
            ax.set_ylim(1e-3, 100)
            ax.legend(fontsize=7)
    for ax in axes[-1]:
        ax.set_xlabel('time (s)')
    fig.suptitle(f'S058 3D Altitude and error decomposition, seed {SHOW_SEED}')
    plt.tight_layout()
    _save(fig, out_dir, 'altitude_errors.png')


def plot_wind(out_dir):
    fig = plt.figure(figsize=(17, 9))
    g = np.linspace(-200, 200, 17)
    X, Y = np.meshgrid(g, g)
    for i, z in enumerate([50.0, 200.0, 500.0]):
        ax = fig.add_subplot(2, 3, i + 1)
        W = wind(np.stack([X, Y, np.full_like(X, z)], axis=-1))
        sp = np.hypot(W[..., 0], W[..., 1])
        q = ax.quiver(X, Y, W[..., 0], W[..., 1], sp, cmap='viridis', clim=(0, 50))
        ax.set_aspect('equal')
        ax.set_title(f'horizontal wind at z = {z:.0f} m, Φ = {float(phi_z(z)):.3f}', fontsize=10)
    fig.colorbar(q, ax=fig.axes[:3], shrink=0.8, label='m/s')
    ax = fig.add_subplot(2, 3, 4)
    rg = np.linspace(0, 200, 201)
    zg = np.linspace(0, 600, 241)
    R, Z = np.meshgrid(rg, zg)
    im = ax.pcolormesh(R, Z, w_vert(R, Z), cmap='RdBu_r', shading='auto', vmin=-5, vmax=5)
    ax.plot(np.maximum(R_EYE, R_START - V_INBOUND * np.linspace(0, T_MISSION, 100)),
            np.maximum(Z_BOT, Z_TOP - V_DESCENT * np.linspace(0, T_MISSION, 100)), 'k--', label='helix (r, z)')
    fig.colorbar(im, ax=ax, label='w (m/s)')
    ax.set_xlabel('r (m)'); ax.set_ylabel('z (m)')
    ax.set_title('vertical wind w(r, z): eyewall updraft, eye subsidence', fontsize=10)
    ax.legend(fontsize=8)
    ax = fig.add_subplot(2, 3, 5)
    zz = np.linspace(0, 600, 301)
    ax.plot(phi_z(zz) * V_MAX, zz, label='v_tan(r_max, z)')
    ax.plot(np.abs(w_vert(R_MAX, zz)), zz, label='|w(r_max, z)|')
    ax.axhline(Z_BL, color='grey', ls=':', label='boundary-layer top')
    ax.set_xlabel('m/s'); ax.set_ylabel('z (m)')
    ax.legend(fontsize=8)
    ax.set_title('wind at the eyewall vs altitude', fontsize=10)
    ax = fig.add_subplot(2, 3, 6)
    for prof, ls in [('helix', '-'), ('flat50', '--'), ('flat500', ':'), ('helix_nodecay', '-.')]:
        t, uxy, uz, _ = steady_thrust(prof)
        ax.plot(t, uxy, ls, label=f'|u_xy|, {PROFILES[prof]}')
    ax.axhline(U_XY_MAX, color='red', lw=1, label='limit m g')
    ax.set_xlabel('time (s)'); ax.set_ylabel('steady thrust (N)')
    ax.legend(fontsize=7)
    ax.set_title('horizontal thrust to stay on the reference (mean wind)', fontsize=10)
    fig.suptitle('S058 3D wind field')
    _save(fig, out_dir, 'wind_field_3d.png')


def plot_sweep(sweep, out_dir):
    fig, axes = plt.subplots(1, 2, figsize=(14, 4.8))
    for c in CTRL:
        for prof, ls in [('helix', '-'), ('flat50', '--')]:
            pk = [np.mean([r['peak'] for r in sweep[(vm, prof, c)]]) for vm in SWEEP_VMAX]
            su = [np.sum([r['success'] for r in sweep[(vm, prof, c)]]) for vm in SWEEP_VMAX]
            axes[0].semilogy(SWEEP_VMAX, pk, ls + 'o', color=COLORS[c], label=f'{LABELS[c]}, {PROFILES[prof]}')
            axes[1].plot(SWEEP_VMAX, su, ls + 'o', color=COLORS[c], label=f'{LABELS[c]}, {PROFILES[prof]}')
    axes[0].axhline(E_FAIL, color='red', lw=1, label='track-loss threshold 15 m')
    axes[0].set_ylabel('mean peak 3D error (m)')
    axes[1].set_ylabel(f'successes out of {N_SEEDS_SWEEP}')
    for ax in axes:
        ax.set_xlabel('v_max (m/s)')
        ax.grid(alpha=0.3)
        ax.legend(fontsize=6)
    fig.suptitle('S058 3D Wind-severity sweep: helix vs fixed 50 m')
    _save(fig, out_dir, 'severity_sweep.png')


def save_animation(main, out_dir):
    trs = {c: main[('helix', c)][0]['traj'] for c in CTRL}
    succ = main[('helix', 'mpc')][0]['success']
    n = N_STEPS + 1
    nf = int(np.ceil((n - 1) / ANIM_STEP)) + 1
    idx = [min(i * ANIM_STEP, n - 1) for i in range(nf)]
    pr, _, _ = reference('helix', np.arange(n) * DT)
    fig = plt.figure(figsize=(6.5, 6))
    ax = fig.add_subplot(111, projection='3d')

    def update(f):
        k = idx[f]
        ax.cla()
        ax.plot(pr[:, 0], pr[:, 1], pr[:, 2], 'k--', lw=0.6)
        th = np.linspace(0, 2 * np.pi, 30)
        col = 'lime' if (k == n - 1 and succ) else 'green'
        ax.plot(R_EYE * np.cos(th), R_EYE * np.sin(th), np.full_like(th, Z_BOT), color=col, lw=2 if k == n - 1 else 1)
        for c in CTRL:
            tr = trs[c]
            ax.plot(tr[:k + 1, 0], tr[:k + 1, 1], tr[:k + 1, 2], color=COLORS[c], lw=0.9)
            ax.scatter(*tr[k], color=COLORS[c], s=25)
        p = trs['mpc'][k]
        w = wind(p)
        ax.quiver(p[0], p[1], p[2], w[0], w[1], w[2], length=1.5, color='navy')
        ax.set_xlim(-200, 200); ax.set_ylim(-200, 200); ax.set_zlim(0, 520)
        ax.view_init(elev=20, azim=-60 + 2 * f)
        ax.set_title(f'S058 3D t = {k * DT:.0f} s, MPC drone z = {p[2]:.0f} m, |wind| = {np.linalg.norm(w):.1f} m/s\n'
                     'orange PD, green MPC, purple H-inf; navy = wind at the MPC drone', fontsize=8)
        return []

    anim = animation.FuncAnimation(fig, update, frames=nf, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    anim.save(path, writer='pillow', fps=5, dpi=70)
    plt.close(fig)
    from PIL import Image
    with Image.open(path) as gif:
        nfile = gif.n_frames
    print(f'Saved: {path} ({nf} frames rendered, {nfile} in the file, {ANIM_STEP} steps = {ANIM_STEP * DT:.0f} s each)')


# ── Report ────────────────────────────────────────────────────────────────────

def report(main, sweep, synth):
    print('\n=== S058 3D Typhoon Eye Penetration — Results ===')
    print(f'  Mission {T_MISSION:.2f} s ({N_STEPS} steps), descent rate {V_DESCENT:.4f} m/s; eyewall crossing '
          f't = {T_EYEWALL:.1f} s at z_ref = {Z_TOP - V_DESCENT * T_EYEWALL:.1f} m, Phi = {float(phi_z(Z_TOP - V_DESCENT * T_EYEWALL)):.3f}')
    print(f'  Phi(500 m) = {float(phi_z(500.0)):.3f} (card: "approximately 70 %"), Phi(300 m) = {float(phi_z(300.0)):.3f}')
    print(f'  Limits: |u_xy| <= {U_XY_MAX:.2f} N, u_z in [{UZ_MIN:.2f}, {UZ_MAX:.2f}] N; largest possible |u| '
          f'{np.hypot(U_XY_MAX, UZ_MAX):.2f} N < F_max {F_MAX_CARD:.0f} N')
    print(f'  Peak eyewall updraft on the helix: {float(np.max(w_vert(*np.stack([np.maximum(R_EYE, R_START - V_INBOUND * np.linspace(0, T_MISSION, 2000)), np.maximum(Z_BOT, Z_TOP - V_DESCENT * np.linspace(0, T_MISSION, 2000))])))):.2f} m/s; '
          f'max over the domain {float(np.max(w_vert(R_MAX, np.linspace(0, 600, 601)))):.2f} m/s at z = 300 m')
    for prof in PROFILES:
        t, uxy, uz, p = steady_thrust(prof)
        i = int(np.argmax(uxy))
        print(f'  Steady thrust on {PROFILES[prof]:36s}: max |u_xy| {uxy.max():.2f} N ({100 * uxy.max() / U_XY_MAX:.0f} % of m g) '
              f'at t = {t[i]:.1f} s (r = {np.hypot(p[i, 0], p[i, 1]):.1f} m, z = {p[i, 2]:.0f} m); u_z range '
              f'{uz.min():.2f} - {uz.max():.2f} N')
    for vm in SWEEP_VMAX:
        print(f'    v_max {vm:.0f}: max steady |u_xy| helix {steady_thrust("helix", vm)[1].max():.2f} N, '
              f'fixed 50 m {steady_thrust("flat50", vm)[1].max():.2f} N')
    K = synth['K']
    print(f"  H-inf: gamma* = {synth['gstar']:.4f}, operating gamma = {synth['gamma']:.4f}; gains x/y pos "
          f"{K[0, 0]:.2f} N/m vel {K[0, 3]:.2f} N s/m, z pos {K[2, 2]:.2f} N/m vel {K[2, 5]:.2f} N s/m")
    Kl, _ = lqr_gain()
    print(f'  LQR (unconstrained MPC) gains: x/y {Kl[0, 0]:.2f} / {Kl[0, 3]:.2f}, z {Kl[2, 2]:.2f} / {Kl[2, 5]:.2f}')
    Kp = np.hstack([KP_PD * np.eye(3), KD_PD * np.eye(3)])
    print('  Closed-loop H-inf norm from w to z, all 3 inputs / horizontal inputs only / vertical input only: ' + '; '.join(
        f'{n} {hinf_norm(k):.4f} / {hinf_norm(k, rows=[0, 1]):.4f} / {hinf_norm(k, rows=[2]):.4f}'
        for n, k in [('PD', Kp), ('LQR', Kl), ('H-inf', K)]))
    for prof in PROFILES:
        print(f'\n  --- {PROFILES[prof]}, v_max {V_MAX:.0f} m/s, {N_SEEDS} seeds (CRN), mean +- std ---')
        for c in CTRL + ['mpc_nowind', 'mpc_abscost']:
            if (prof, c) not in main:
                continue
            rs = main[(prof, c)]
            a = {k: np.array([r[k] for r in rs], float) for k in
                 ('e3', 'exy', 'ez', 'peak', 'peak_xy', 'peak_z', 'effort', 'sat', 'success', 'compl', 'r_peak_z', 'z_peak_z', 't_peak')}
            extra = f", ADMM active {100 * np.mean([r['admm'] for r in rs]):.1f} %" if 'admm' in rs[0] else ''
            cmp_ = f", sounding completeness {100 * a['compl'].mean():.1f} % (min {100 * a['compl'].min():.1f} %)" \
                if prof.startswith('helix') else ''
            print(f"    {LABELS[c]:26s} e3 {a['e3'].mean():.3f} +- {a['e3'].std():.3f} m (e_xy {a['exy'].mean():.3f}, "
                  f"e_z {a['ez'].mean():.3f}), peak {a['peak'].mean():.2f} +- {a['peak'].std():.2f} m "
                  f"(xy {a['peak_xy'].mean():.2f}, z {a['peak_z'].mean():.2f}), effort {a['effort'].mean():.0f} N s, "
                  f"mean signed z error {np.mean([r['ez_signed'] for r in rs]):+.3f} m, saturated {100 * a['sat'].mean():.1f} %{extra}, success {int(a['success'].sum())}/{N_SEEDS}{cmp_}; "
                  f"peak e_z at r = {np.median(a['r_peak_z']):.0f} m (median), z = {np.median(a['z_peak_z']):.0f} m")
    print('\n  --- helix vs fixed 50 m (2D baseline), per seed ---')
    for c in CTRL + ['mpc_nowind']:
        h = main[('helix', c)]
        f = main[('flat50', c)]
        pk_h = np.array([r['peak'] for r in h]); pk_f = np.array([r['peak'] for r in f])
        exy_h = np.array([r['exy'] for r in h]); exy_f = np.array([r['exy'] for r in f])
        print(f"    {LABELS[c]:26s} lower peak error on the helix in {int(np.sum(pk_h < pk_f))}/{N_SEEDS} seeds; "
              f"lower mean horizontal error in {int(np.sum(exy_h < exy_f))}/{N_SEEDS}; mean peak ratio helix/flat "
              f"{pk_h.mean() / pk_f.mean():.2f}; success {sum(r['success'] for r in h)} vs {sum(r['success'] for r in f)}")
    print('\n  --- ablations on the helix (mean peak error, success, saturation) ---')
    for c in CTRL:
        parts = []
        for prof in ('helix', 'helix_nodecay', 'helix_noupdraft', 'flat50'):
            rs = main[(prof, c)]
            parts.append(f"{prof}: peak {np.mean([r['peak'] for r in rs]):.2f} m, e_z {np.mean([r['ez'] for r in rs]):.3f} m, "
                         f"success {sum(r['success'] for r in rs)}, sat {100 * np.mean([r['sat'] for r in rs]):.1f} %")
        print(f'    {LABELS[c]:26s} ' + ' | '.join(parts))
    print(f'\n  --- thrust geometry: tilt at |u_xy| = m g is {math.degrees(math.atan2(U_XY_MAX, M_DRONE * G)):.1f} deg '
          f'at u_z = m g and {math.degrees(math.atan2(U_XY_MAX, UZ_MIN)):.1f} deg at u_z = 0.7 m g; largest applied tilt '
          f"(card limits, helix / fixed 50 m, all controllers): "
          f"{max(r['tilt_max'] for c in CTRL for r in main[('helix', c)]):.1f} / "
          f"{max(r['tilt_max'] for c in CTRL for r in main[('flat50', c)]):.1f} deg")
    print('  --- tilt-cone ablation |u_xy| <= min(m g, u_z) (PD and H-inf; mean peak error, success, saturation) ---')
    for c in ('pd', 'hinf'):
        parts = []
        for prof in ('helix', 'flat50', 'helix_nodecay'):
            rs = main[(prof, c + '_cone')]
            parts.append(f"{prof}: peak {np.mean([r['peak'] for r in rs]):.2f} m (xy {np.mean([r['peak_xy'] for r in rs]):.2f}), "
                         f"success {sum(r['success'] for r in rs)}/{N_SEEDS}, sat {100 * np.mean([r['sat'] for r in rs]):.1f} %, "
                         f"max tilt {max(r['tilt_max'] for r in rs):.1f} deg")
        ph = np.array([r['peak'] for r in main[('helix', c + '_cone')]])
        pf = np.array([r['peak'] for r in main[('flat50', c + '_cone')]])
        print(f'    {LABELS[c]:26s} ' + ' | '.join(parts) + f'; helix lower peak than fixed 50 m in '
              f'{int(np.sum(ph < pf))}/{N_SEEDS}, mean peak ratio {ph.mean() / pf.mean():.2f}')
    print(f'  --- updraft probe at v_max = 40 m/s ({N_SEEDS_SWEEP} seeds): mean peak error / mean e_z / success / saturation ---')
    for c in CTRL:
        print(f'    {LABELS[c]:26s} ' + ' | '.join(
            f"{prof}: {np.mean([r['peak'] for r in main[(prof, c, 40.0)]]):.2f} m / "
            f"{np.mean([r['ez'] for r in main[(prof, c, 40.0)]]):.3f} m / {sum(r['success'] for r in main[(prof, c, 40.0)])} / "
            f"{100 * np.mean([r['sat'] for r in main[(prof, c, 40.0)]]):.1f} %"
            for prof in ('helix', 'helix_noupdraft', 'flat50')))
    print('\n  --- controller ranking on the helix (per-seed wins on mean 3D error) ---')
    E = np.array([[r['e3'] for r in main[('helix', c)]] for c in CTRL])
    print('    ' + ', '.join(f'{LABELS[c]} {int(np.sum(np.argmin(E, 0) == i))}' for i, c in enumerate(CTRL)))
    print(f"    MPC lower peak than H-inf in {int(np.sum(np.array([r['peak'] for r in main[('helix', 'mpc')]]) < np.array([r['peak'] for r in main[('helix', 'hinf')]])))}/{N_SEEDS}")
    print(f'\n  --- severity sweep, {N_SEEDS_SWEEP} seeds per point: mean e3 / mean peak / successes ---')
    for vm in SWEEP_VMAX:
        parts = []
        for prof in ('helix', 'flat50'):
            for c in CTRL:
                rs = sweep[(vm, prof, c)]
                parts.append(f"{prof}/{c} {np.mean([r['e3'] for r in rs]):.3f}/{np.mean([r['peak'] for r in rs]):.2f}/"
                             f"{sum(r['success'] for r in rs)}")
        print(f'    v_max {vm:.0f}: ' + ' | '.join(parts))


if __name__ == '__main__':
    import sys
    import time
    t0 = time.time()
    main, sweep, synth = run_simulation()
    report(main, sweep, synth)
    out = OUTPUT_DIR
    os.makedirs(out, exist_ok=True)
    plot_helix_3d(main, out)
    plot_top_down(main, out)
    plot_altitude_errors(main, out)
    plot_wind(out)
    plot_sweep(sweep, out)
    save_animation(main, out)
    print(f'[runtime {time.time() - t0:.0f} s]', file=sys.stderr)
