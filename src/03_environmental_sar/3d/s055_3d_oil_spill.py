"""
S055 3D Oil Spill Tracking
==========================
The S055 slick (advection-diffusion on a 4 x 4 km, 20 m grid; boundary C = 0.08; 36-landmark
Kalman filter with drift prediction; replanning every 30 s) is now observed by a nadir
fluorescence sensor whose footprint and signal depend on altitude, over a sea with a
spatially correlated significant-wave-height field. Oil thickness is H0 C/C0 sqrt(t0/(t+t0)).

Sensor (card): L = L0 (A_foot / pi z^2) alpha(h) cos(theta_w) exp(-mu z), alpha = 1 - exp(-kappa h),
averaged over the footprint disc r_foot = z tan(phi); the ground station inverts L to a
thickness and a concentration and thresholds C at 0.08 (boundary) or maps the thickness.

Strategies (card), same truth, waves and noise draws per trial:
  1. single     - both drones at 15 m patrol the boundary (2D S055 adaptive + prediction);
                  thickness map only from their boundary samples
  2. dual       - drone A at z_A = r_target / tan(phi) = 10 m patrols the boundary alone,
                  drone B at the card's z_B = argmax r_foot alpha exp(-mu z) (clipped to 40 m)
                  maps the interior with a drift-following lawnmower; no sea-state correction
  3. dual_corr  - as 2 with the sea-state correction <cos theta_w>, the card's wave-adaptive
                  z_A law and landmark confidence weights

Usage:
    conda activate drones
    python src/03_environmental_sar/3d/s055_3d_oil_spill.py
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
from scipy.ndimage import map_coordinates
from scipy.spatial import cKDTree

SCRIPT_DIR   = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, '..', '..', '..'))
OUTPUT_DIR   = os.path.join(PROJECT_ROOT, 'outputs', '03_environmental_sar', '3d', 's055_3d_oil_spill')

_spec = importlib.util.spec_from_file_location('s055_2d', os.path.join(SCRIPT_DIR, '..', 's055_oil_spill.py'))
s2d = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(s2d)

# -- Parameters (card) ----------------------------------------------------------------
PHI        = math.atan(5.0 / 10.0)   # sensor half-angle
R_TARGET   = 5.0                     # m
ZA_RANGE   = (8.0, 15.0)
ZB_RANGE   = (25.0, 40.0)
Z_2D       = s2d.ALTITUDE            # 15 m
MU_ATM     = 2e-4                    # 1/m
KAPPA      = 500.0                   # 1/m
H0         = 0.10e-3                 # m (0.10 mm)
EPS_TARGET = 0.02e-3                 # m (0.02 mm)
SIGMA_H    = 0.3                     # m  (std of Hs)
L_WAVE     = 500.0                   # m
LAMBDA_W   = 80.0                    # m
TAU_Z      = 2.0                     # s
DSIG_Q     = 0.3                     # m/s^0.5 confidence inflation
W_MIN      = 0.3
L0         = 1.0                     # normalised radiance
N_GRID_TH  = 20                      # coarse thickness grid (card 20 x 20)
WAVE_TAU   = 300.0                   # s  (sea state evolves on a 5 min timescale)
U          = s2d.U_CARD
T_MISSION, DT_SIM, T_REPLAN = s2d.T_MISSION, s2d.DT_SIM, s2d.T_REPLAN

# -- Parameters (added) ---------------------------------------------------------------
HS_MEAN    = 0.5                     # m  mean significant wave height (card gives the std only)
T0_SPREAD  = 1800.0                  # s  spreading time constant t0 (card: R0 / k_F, k_F not given)
SIGMA_L    = 0.05                    # relative radiometric noise per sample
N_FOOT     = 7                       # footprint sample points (centre + 6 at 0.7 r)
BOX        = 1000.0                  # m  side of the drift-following box mapped by drone B
WAVE_RES   = 100.0                   # m  wave-field grid
EPS_FROM   = 300.0                   # s  thickness RMSE averaged from this time on
ZB_SWEEP   = (15.0, 25.0, 30.0, 35.0, 60.0, 100.0)   # dual: drone B altitude (15 = ablation at the 2D altitude; 60, 100 beyond the card bound)
ZA_SWEEP   = (8.0, 15.0)                       # dual: drone A altitude
LAMBDA_PROBE = 8.0                   # m  steep-wave probe for the sea-state correction
NOISE_SWEEP = (0.02, 0.10)
N_TRIALS   = 5
MAIN_TRIAL = 0
ANIM_FRAMES = 90

STRATS = ['single', 'dual', 'dual_corr']
LABELS = {'single': 'Single altitude 15 m (2D baseline)', 'dual': 'Dual altitude, no sea-state correction',
          'dual_corr': 'Dual altitude + sea-state correction'}
COLORS = {'single': '#7F7F7F', 'dual': '#1F77B4', 'dual_corr': '#D62728'}
FOOT_OFF = np.vstack([[0.0, 0.0], 0.7 * np.column_stack([np.cos(np.arange(6) * np.pi / 3),
                                                        np.sin(np.arange(6) * np.pi / 3)])])


# -- Physics --------------------------------------------------------------------------

def r_foot(z):
    return z * math.tan(PHI)


def g_age(t):
    return math.sqrt(T0_SPREAD / (t + T0_SPREAD))


def radiance(z, h_mean_alpha, cos_t):
    """Card sensor model with a footprint-averaged fluorescence efficiency."""
    a_foot = math.pi * r_foot(z) ** 2
    return L0 * a_foot / (math.pi * z * z) * h_mean_alpha * cos_t * math.exp(-MU_ATM * z)


def invert(L, z_model, cw):
    """Thickness from radiance with the modelled altitude and wave factor."""
    a = L / (L0 * math.tan(PHI) ** 2 * math.exp(-MU_ATM * z_model) * cw)
    a = min(max(a, 0.0), 1 - 1e-12)
    return -math.log(1 - a) / KAPPA


def cos_mean(hs, lam=LAMBDA_W):
    s = np.arctan(np.pi * np.maximum(hs, 0.0) / lam)
    return np.exp(-s ** 2 / 2), s


class WaveField:
    """Hs(x, y, t): exponential covariance (L_WAVE), AR(1) in time (WAVE_TAU), mean HS_MEAN."""

    _Lc = None

    def __init__(self, rng, lam=LAMBDA_W):
        n = int(s2d.AREA_SIZE / WAVE_RES) + 1
        if WaveField._Lc is None:                    # shared Cholesky factor (deterministic)
            g = np.arange(n) * WAVE_RES
            X, Y = np.meshgrid(g, g, indexing='ij')
            P = np.column_stack([X.ravel(), Y.ravel()])
            d = np.sqrt(((P[:, None, :] - P[None, :, :]) ** 2).sum(-1))
            WaveField._Lc = np.linalg.cholesky(SIGMA_H ** 2 * np.exp(-d / L_WAVE) + 1e-9 * np.eye(len(P)))
        self.Lc = WaveField._Lc
        self.n = n
        self.rng = rng
        self.lam = lam
        self.F = self.Lc @ rng.standard_normal(n * n)
        self.t = 0.0

    def advance(self, t):
        while self.t + T_REPLAN <= t + 1e-9:
            a = math.exp(-T_REPLAN / WAVE_TAU)
            self.F = a * self.F + math.sqrt(1 - a * a) * (self.Lc @ self.rng.standard_normal(len(self.F)))
            self.t += T_REPLAN

    def hs(self, p):
        i = min(max(int(round(p[0] / WAVE_RES)), 0), self.n - 1)
        j = min(max(int(round(p[1] / WAVE_RES)), 0), self.n - 1)
        return max(HS_MEAN + self.F[i * self.n + j], 0.0)

    def grid(self):
        return np.maximum(HS_MEAN + self.F.reshape(self.n, self.n), 0.0)


# -- 3D drones and strategy -----------------------------------------------------------

class Drone3D(s2d.Drone):
    def __init__(self, pos, color, z0):
        super().__init__(pos, color)
        self.z = z0
        self.z_cmd = z0
        self.ztraj = [z0]

    def step(self, dt, z_lo, z_hi):
        moved = super().step(dt)
        self.z += (self.z_cmd - self.z) / TAU_Z * dt
        self.z = min(max(self.z, z_lo), z_hi)
        return moved


class Strategy3D(s2d.Strategy):
    def __init__(self, name, lm0, u, rng, waves, z_b=None, z_a=None, sigma_l=SIGMA_L):
        super().__init__('predict', lm0, u, 2 if name == 'single' else 1, rng)
        self.kind = name
        self.waves = waves
        self.sigma_l = sigma_l
        self.correct = name == 'dual_corr'
        if name == 'single':
            self.drones = [Drone3D(d.pos, d.color, Z_2D) for d in self.drones]
            self.ranges = [(Z_2D, Z_2D)] * 2
        else:
            za = z_a if z_a is not None else R_TARGET / math.tan(PHI)
            zb = z_b if z_b is not None else card_zb()
            self.drones = [Drone3D(self.drones[0].pos, 'orange', za),
                           Drone3D(s2d.X0.copy(), 'cyan', zb)]
            self.ranges = [(min(ZA_RANGE[0], za), max(ZA_RANGE[1], za)), (min(ZB_RANGE[0], zb), max(ZB_RANGE[1], zb))]
            self.zb = zb
            self.za_fixed = z_a
            self.box_path = lawnmower(2 * r_foot(zb))
        self.th_val = np.full((N_GRID_TH, N_GRID_TH), np.nan)   # thickness at measurement time
        self.th_t = np.full((N_GRID_TH, N_GRID_TH), np.nan)
        self.n_infl = 0
        self.n_samples = 0
        self.w_log = []
        self.th_log = {'t': [], 'eps': [], 'obs': [], 'bias': []}

    def replan(self, t):
        self.kf.predict_to(t)
        plan = self.kf.mu + self.u * T_REPLAN
        arcs = s2d.split_indices(self.kf.mu, self.n_drones, balance=True)
        self.arcs = arcs
        nrm = s2d.BoundaryKF(plan, np.zeros(2), 0).normals()
        boundary = self.drones if self.kind == 'single' else self.drones[:1]
        for d, a in zip(boundary, arcs):
            idx = [i % s2d.N_LANDMARKS for i in a]
            d.set_path(s2d.zigzag_path(plan[idx], nrm[idx]))
        if self.kind != 'single':
            B = self.drones[1]
            k_old, dir_old = B.k, B.dir
            B.path = self.box_path + s2d.X0 + self.u * (t + T_REPLAN / 2)
            if t == 0.0:
                B.set_path(B.path)
            else:
                B.k, B.dir = k_old, dir_old
            A = self.drones[0]
            if self.correct and self.za_fixed is None:
                cm, _ = cos_mean(self.waves.hs(A.pos), self.waves.lam)
                A.z_cmd = min(max(R_TARGET / math.tan(PHI) / math.sqrt(float(cm)), ZA_RANGE[0]), ZA_RANGE[1])

    def measure(self, d, C, t):
        """Footprint-averaged radiance -> thickness and concentration estimates."""
        rf = r_foot(d.z)
        pts = d.pos[None, :] + rf * FOOT_OFF
        c = map_coordinates(C, [pts[:, 0] / s2d.H_CELL - 0.5, pts[:, 1] / s2d.H_CELL - 0.5], order=1,
                            mode='nearest')
        h = H0 * g_age(t) * np.maximum(c, 0.0)
        alpha = float(np.mean(1 - np.exp(-KAPPA * h)))
        hs = self.waves.hs(d.pos)
        cm, sth = cos_mean(hs, self.waves.lam)
        cos_t = math.cos(self.rng.normal(0.0, float(sth)))
        L = radiance(d.z, alpha, cos_t) * (1 + self.sigma_l * self.rng.standard_normal())
        z_model = Z_2D if self.kind == 'single' else d.z
        h_hat = invert(L, z_model, float(cm) if self.correct else 1.0)
        return h_hat, h_hat / (H0 * g_age(t)), float(cm)

    def sense(self, d, C, t):
        h_hat, c_hat, cm = self.measure(d, C, t)
        self.n_samples += 1
        # thickness map (drift frame, coarse grid)
        q = d.pos - s2d.X0 - self.u * t
        i, j = int((q[0] + BOX / 2) // (BOX / N_GRID_TH)), int((q[1] + BOX / 2) // (BOX / N_GRID_TH))
        if 0 <= i < N_GRID_TH and 0 <= j < N_GRID_TH:
            self.th_val[i, j] = h_hat
            self.th_t[i, j] = t
        if self.kind != 'single' and d is self.drones[1]:
            return                                   # drone B maps thickness only
        reading = c_hat >= s2d.C_THR
        mu = self.kf.mu + self.kf.drift * (t - self.kf.t)
        dk = np.linalg.norm(mu - d.pos, axis=1)
        self.t_nom[dk <= s2d.R_ASSOC] = t
        if d.last_read is not None and reading != d.last_read:
            m = 0.5 * (d.pos + d.last_pos) + self.rng.normal(0, s2d.GPS_SIGMA, 2)
            self.kf.predict_to(t)
            dk = np.linalg.norm(self.kf.mu - m, axis=1)
            k = int(np.argmin(dk))
            if dk[k] <= s2d.R_ASSOC:
                if self.correct:
                    w = cm * math.exp(-MU_ATM * d.z) * (1.0 if r_foot(d.z) <= s2d.R_ASSOC else 0.0)
                    self.w_log.append(w)
                    if w < W_MIN:
                        self.kf.P[k] += DSIG_Q ** 2 * (1 - w) * np.eye(2)
                        self.n_infl += 1
                self.kf.update(k, m)
                self.t_last[k] = t
                self.n_meas += 1
        d.last_read = reading
        d.last_pos = d.pos.copy()

    def thickness_rmse(self, C, t):
        """RMSE of the coarse thickness map over the true interior (C >= 0.08), nearest fill."""
        cs = (np.arange(N_GRID_TH) + 0.5) * BOX / N_GRID_TH - BOX / 2
        X, Y = np.meshgrid(cs, cs, indexing='ij')
        wx, wy = X + s2d.X0[0] + self.u[0] * t, Y + s2d.X0[1] + self.u[1] * t
        c = map_coordinates(C, [wx.ravel() / s2d.H_CELL - 0.5, wy.ravel() / s2d.H_CELL - 0.5], order=1,
                            mode='nearest').reshape(X.shape)
        interior = c >= s2d.C_THR
        truth = H0 * g_age(t) * c
        obs = ~np.isnan(self.th_val)
        if not obs.any() or not interior.any():
            return np.nan, 0.0, np.nan
        est = self.th_val * np.sqrt((self.th_t + T0_SPREAD) / (t + T0_SPREAD))   # g(t) / g(t_meas)
        oi = np.argwhere(obs)
        tree = cKDTree(oi)
        _, nn = tree.query(np.argwhere(np.ones_like(obs)))
        filled = est[oi[nn, 0], oi[nn, 1]].reshape(obs.shape)
        err = (filled - truth)[interior]
        return float(np.sqrt(np.mean(err ** 2))), float(np.mean(obs[interior])), float(np.mean(err))


def card_zb():
    """Card: z_B = argmax r_foot(z) alpha exp(-mu z) -> 1/mu, clipped to the B range."""
    return float(min(max(1.0 / MU_ATM, ZB_RANGE[0]), ZB_RANGE[1]))


def lawnmower(spacing):
    """Boustrophedon over the drift-frame box (dense points every 2 m), relative to the centre."""
    xs = np.arange(-BOX / 2 + spacing / 2, BOX / 2, spacing)
    pts = []
    for i, x in enumerate(xs):
        y0, y1 = (-BOX / 2, BOX / 2) if i % 2 == 0 else (BOX / 2, -BOX / 2)
        pts += [(x, y0), (x, y1)]
    pts = np.array(pts)
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    cum = np.concatenate([[0.0], np.cumsum(seg)])
    s = np.arange(0.0, cum[-1] + 1e-9, 2.0)
    return np.column_stack([np.interp(s, cum, pts[:, 0]), np.interp(s, cum, pts[:, 1])])


# -- Simulation -----------------------------------------------------------------------

def run_trial(u, seed, configs, record=False, lam=LAMBDA_W):
    C = s2d.init_field()
    lm0 = s2d.resample_closed(s2d.truth_contour(C), s2d.N_LANDMARKS)
    strats = {}
    for key, (kind, kw) in configs.items():
        waves = WaveField(np.random.default_rng([seed, 1]), kw.get('lam', lam))
        strats[key] = Strategy3D(kind, lm0, u, np.random.default_rng([seed, 2]), waves,
                                 kw.get('z_b'), kw.get('z_a'), kw.get('sigma_l', SIGMA_L))
    n_steps = int(round(T_MISSION / DT_SIM))
    field_every = int(round(s2d.DT_FIELD / DT_SIM))
    replan_every = int(round(T_REPLAN / DT_SIM))
    eval_every = int(round(s2d.EVAL_EVERY / DT_SIM))
    frames, hs_snap = [], None
    for step in range(n_steps + 1):
        t = step * DT_SIM
        if step % replan_every == 0 and step < n_steps:
            for st in strats.values():
                st.waves.advance(t)
                st.replan(t)
        if step % eval_every == 0:
            tc = s2d.truth_contour(C)
            tree = cKDTree(s2d.resample_closed(tc, 2000))
            per_t = s2d.perimeter(tc)
            for st in strats.values():
                mu = st.kf.mu + st.kf.drift * (t - st.kf.t)
                e, _ = tree.query(mu)
                st.log['t'].append(t); st.log['eta'].append(st.eta(t))
                st.log['eta_nom'].append(st.eta(t, verified=False))
                st.log['err'].append(float(e.mean())); st.log['err_sd'].append(float(e.std()))
                st.log['trP'].append(float(np.trace(st.kf.P, axis1=1, axis2=2).mean()))
                st.log['per_est'].append(s2d.perimeter(mu)); st.log['per_true'].append(per_t)
                eps, obs, bias = st.thickness_rmse(C, t)
                st.th_log['t'].append(t); st.th_log['eps'].append(eps); st.th_log['obs'].append(obs)
                st.th_log['bias'].append(bias)
            if record:
                st = strats['dual_corr']
                mu = st.kf.mu + st.kf.drift * (t - st.kf.t)
                frames.append((t, C.copy(), tc, mu.copy(), [(d.pos.copy(), d.z) for d in st.drones],
                               st.log['eta'][-1], st.th_log['eps'][-1], st.waves.grid().copy()))
                if abs(t - 900.0) < 1e-9:
                    hs_snap = (st.waves.grid().copy(), tc.copy())
        if step == n_steps:
            break
        for st in strats.values():
            for d, (zlo, zhi) in zip(st.drones, st.ranges):
                d.dist_since += d.step(DT_SIM, zlo, zhi)
                assert zlo - 1e-9 <= d.z <= zhi + 1e-9, 'altitude outside its range'
                if record:
                    d.traj.append(d.pos.copy()); d.ztraj.append(d.z)
                if d.dist_since >= s2d.DELTA_S - 1e-9:
                    d.dist_since -= s2d.DELTA_S
                    st.sense(d, C, t + DT_SIM)
        if (step + 1) % field_every == 0:
            C = s2d.advance_field(C, u)
            assert C.min() > -1e-3, 'field went negative'
    return strats, frames, hs_snap, C


def summarise(st):
    s = s2d.summarise(st)
    t = np.array(st.th_log['t']); e = np.array(st.th_log['eps'], dtype=float)
    m = t >= EPS_FROM
    s['eps_mean'] = float(np.nanmean(e[m]))
    s['eps_final'] = float(e[-1])
    s['bias_mean'] = float(np.nanmean(np.array(st.th_log['bias'], dtype=float)[m]))
    s['eps_ok'] = float(np.mean(e[m] <= EPS_TARGET))
    s['obs_final'] = float(st.th_log['obs'][-1])
    s['n_infl'] = st.n_infl
    s['w_min'] = float(min(st.w_log)) if st.w_log else np.nan
    return s


def configs_main():
    cfg = {s_: (s_, {}) for s_ in STRATS}
    for zb in ZB_SWEEP:
        cfg[('zb', zb)] = ('dual', {'z_b': zb})
    for za in ZA_SWEEP:
        cfg[('za', za)] = ('dual', {'z_a': za})
    for sl in NOISE_SWEEP:
        cfg[('noise', sl)] = ('dual', {'sigma_l': sl})
    cfg[('steep', 'dual')] = ('dual', {'lam': LAMBDA_PROBE})
    cfg[('steep', 'dual_corr')] = ('dual_corr', {'lam': LAMBDA_PROBE})
    return cfg


def trial_wind(i):
    """Trial 0 = card wind; others = card speed in the 2D S055 Monte Carlo directions."""
    rng = np.random.default_rng(55)
    angles = rng.uniform(0, 2 * np.pi, s2d.N_TRIALS)
    if i == MAIN_TRIAL:
        return U.copy()
    return float(np.linalg.norm(U)) * np.array([np.cos(angles[i]), np.sin(angles[i])])


def run_one(i):
    strats, _, _, _ = run_trial(trial_wind(i), 100 + i, configs_main())
    return {k: summarise(v) for k, v in strats.items()}


def run_simulation():
    others = [i for i in range(N_TRIALS) if i != MAIN_TRIAL]
    n_workers = min(len(others), os.cpu_count() or 1)
    if n_workers > 1:
        import multiprocessing as mp
        pool = mp.get_context('spawn').Pool(n_workers)
        async_res = pool.map_async(run_one, others, chunksize=1)     # collected in trial order
    strats, frames, hs_snap, C = run_trial(trial_wind(MAIN_TRIAL), 100 + MAIN_TRIAL, configs_main(), record=True)
    main_sum = {k: summarise(v) for k, v in strats.items()}
    if n_workers > 1:
        other_res = async_res.get()
        pool.close(); pool.join()
    else:
        other_res = [run_one(i) for i in others]
    mc = []
    it = iter(other_res)
    for i in range(N_TRIALS):
        mc.append(main_sum if i == MAIN_TRIAL else next(it))
    return {'mc': mc, 'strats': strats, 'frames': frames, 'hs_snap': hs_snap, 'C_end': C,
            'winds': [trial_wind(i) for i in range(N_TRIALS)]}


# -- Report ---------------------------------------------------------------------------

def _mc(res, key, field):
    return np.array([m[key][field] for m in res['mc']], dtype=float)


def report(res):
    print('=== S055 3D Oil Spill Tracking ===')
    print(f'Sensor half-angle {math.degrees(PHI):.2f} deg; r_foot = z tan(phi) = z/2')
    for z in (8.0, 10.0, 15.0, 25.0, 40.0):
        a_term = math.pi * r_foot(z) ** 2 / (math.pi * z * z)
        print(f'  z = {z:4.0f} m: r_foot {r_foot(z):5.2f} m, A_foot/(pi z^2) = {a_term:.4f}, exp(-mu z) = '
              f'{math.exp(-MU_ATM * z):.5f}, radiance relative to 15 m {math.exp(-MU_ATM * (z - 15.0)):.5f}')
    for lam in (LAMBDA_W, LAMBDA_PROBE):
        for hs in (0.1, 0.3, 0.6):
            cm, st_ = cos_mean(hs, lam)
            print(f'  wave lambda {lam:.0f} m, Hs {hs:.1f} m: slope sigma {math.degrees(float(st_)):.3f} deg, '
                  f'<cos> {float(cm):.6f}, effective threshold raised by {100 * (1 / float(cm) - 1):.4f}%')
    hA, hB = 0.01e-3, 0.10e-3
    for h in (hA, hB):
        a = 1 - math.exp(-KAPPA * h)
        print(f'  h = {1e3 * h:.2f} mm: alpha {a:.5f}; ln(L_A/L_B) at z_A = 10, z_B = 40 m: '
              f'{math.log(radiance(10.0, a, 1.0) / radiance(40.0, a, 1.0)):.6f} (= -mu (z_A - z_B) = '
              f'{-MU_ATM * (10.0 - 40.0):.6f})')
    print(f'  card z_B = argmax z tan(phi) alpha exp(-mu z) = 1/mu = {1 / MU_ATM:.0f} m -> clipped to {card_zb():.0f} m')
    cm6, _ = cos_mean(0.6)
    print(f'  card z_A law at Hs = 0.6 m: {R_TARGET / math.tan(PHI) / math.sqrt(float(cm6)):.4f} m (nominal 10 m)')
    print(f'  thickness: H0 {1e3 * H0:.2f} mm, age factor at 1800 s {g_age(1800.0):.3f}; boundary thickness '
          f'C_thr H0 g = {1e3 * s2d.C_THR * H0:.4f} -> {1e3 * s2d.C_THR * H0 * g_age(1800.0):.4f} mm')
    sp = 2 * r_foot(card_zb())
    print(f'  drone B lawnmower at {card_zb():.0f} m: swath {sp:.0f} m, {len(np.arange(-BOX / 2 + sp / 2, BOX / 2, sp))} '
          f'strips over the {BOX:.0f} m box, {len(lawnmower(sp)) * 2.0 / 1000:.1f} km per pass '
          f'({len(lawnmower(sp)) * 2.0 / s2d.V_ARC:.0f} s at {s2d.V_ARC:.0f} m/s)')

    print(f'\nCard wind (trial {MAIN_TRIAL}):')
    cols = STRATS
    keys = [('eta_mean', 'mean eta (verified)', '{:.3f}'), ('err_mean', 'mean boundary error (m)', '{:.1f}'),
            ('err_final', 'final boundary error (m)', '{:.1f}'), ('n_meas', 'crossings', '{:.0f}'),
            ('eps_mean', 'mean thickness RMSE after 300 s (mm)', 'mm'), ('eps_final', 'final thickness RMSE (mm)', 'mm'),
            ('eps_ok', 'time with RMSE <= 0.02 mm (%)', 'p'), ('obs_final', 'interior cells observed at end (%)', 'p'),
            ('n_infl', 'confidence inflations', '{:.0f}'), ('w_min', 'min confidence weight', '{:.4f}')]
    m0 = res['mc'][MAIN_TRIAL]
    print('  metric | ' + ' | '.join(cols))
    for k, lab, fmt in keys:
        row = []
        for c in cols:
            v = m0[c][k]
            row.append('-' if (isinstance(v, float) and np.isnan(v)) else
                       f'{1e3 * v:.4f}' if fmt == 'mm' else f'{100 * v:.1f}' if fmt == 'p' else fmt.format(v))
        print(f'  {lab} | ' + ' | '.join(row))
    st = res['strats']['dual_corr']
    zA = np.array(st.drones[0].ztraj)
    print(f'  dual_corr drone A altitude range {zA.min():.4f}-{zA.max():.4f} m; drone B {min(st.drones[1].ztraj):.2f}-'
          f'{max(st.drones[1].ztraj):.2f} m; samples {st.n_samples}')

    print(f'\nMonte Carlo ({N_TRIALS} wind directions incl. the card wind; same truth, waves and noise per trial):')
    for k, lab, fmt in keys[:8]:
        parts = []
        for c in cols:
            v = _mc(res, c, k)
            sc = 1e3 if fmt == 'mm' else 100 if fmt == 'p' else 1
            parts.append(f'{c} {sc * v.mean():.4g} +- {sc * v.std():.2g}')
        print(f'  {lab}: ' + '; '.join(parts))
    for a, b, k, better in (('dual', 'single', 'eps_mean', 'lower'), ('dual', 'single', 'err_mean', 'lower'),
                            ('dual', 'single', 'eta_mean', 'higher'), ('dual_corr', 'dual', 'eps_mean', 'lower'),
                            ('dual_corr', 'dual', 'err_mean', 'lower')):
        va, vb = _mc(res, a, k), _mc(res, b, k)
        n_b = int(np.sum(va < vb)) if better == 'lower' else int(np.sum(va > vb))
        sc = 1e3 if k == 'eps_mean' else 1
        print(f'  {a} vs {b}, {k}: {better} in {n_b}/{N_TRIALS}, equal {int(np.sum(va == vb))}; mean diff '
              f'{sc * np.mean(va - vb):+.4g}{" mm" if k == "eps_mean" else ""}')
    print('Sweeps (dual strategy, Monte Carlo means):')
    for zb in sorted(ZB_SWEEP + (card_zb(),)):
        key = 'dual' if zb == card_zb() else ('zb', zb)
        print(f'  z_B = {zb:5.0f} m (swath {2 * r_foot(zb):.0f} m): thickness RMSE {1e3 * _mc(res, key, "eps_mean").mean():.4f} mm, '
              f'final {1e3 * _mc(res, key, "eps_final").mean():.4f} mm, interior observed {100 * _mc(res, key, "obs_final").mean():.1f}%, '
              f'boundary error {_mc(res, key, "err_mean").mean():.1f} m')
    for za in sorted(ZA_SWEEP + (R_TARGET / math.tan(PHI),)):
        key = 'dual' if abs(za - 10.0) < 1e-9 else ('za', za)
        print(f'  z_A = {za:5.1f} m: boundary error {_mc(res, key, "err_mean").mean():.2f} m, eta {_mc(res, key, "eta_mean").mean():.3f}, '
              f'crossings {_mc(res, key, "n_meas").mean():.1f}')
    for sl in sorted(NOISE_SWEEP + (SIGMA_L,)):
        key = 'dual' if sl == SIGMA_L else ('noise', sl)
        print(f'  radiometric noise {100 * sl:.0f}%: thickness RMSE {1e3 * _mc(res, key, "eps_mean").mean():.4f} mm, '
              f'boundary error {_mc(res, key, "err_mean").mean():.2f} m')
    for c in ('dual', 'dual_corr'):
        key = ('steep', c)
        print(f'  steep-wave probe (lambda {LAMBDA_PROBE:.0f} m), {c}: thickness RMSE {1e3 * _mc(res, key, "eps_mean").mean():.4f} mm, '
              f'boundary error {_mc(res, key, "err_mean").mean():.2f} m, eta {_mc(res, key, "eta_mean").mean():.3f}')
    for c in ('single', 'dual', ('steep', 'dual'), ('steep', 'dual_corr')):
        print(f'  mean signed thickness error (map - truth) after 300 s, {c}: '
              f'{1e3 * _mc(res, c, "bias_mean").mean():+.5f} mm')
    for k in ('eps_mean', 'err_mean'):
        a, b = _mc(res, ('steep', 'dual_corr'), k), _mc(res, ('steep', 'dual'), k)
        sc = 1e3 if k == 'eps_mean' else 1
        print(f'  steep-wave probe, corrected minus uncorrected {k}: per trial '
              + ', '.join(f'{sc * v:+.4f}' for v in a - b) + f'; lower in {int(np.sum(a < b))}/{N_TRIALS}')


# -- Plotting -------------------------------------------------------------------------

def _save(fig, out, name):
    p = os.path.join(out, name)
    fig.savefig(p, dpi=120, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {p}')


def plot_trajectories_3d(res, out):
    st = res['strats']['dual_corr']
    fig = plt.figure(figsize=(11, 8))
    ax = fig.add_subplot(111, projection='3d')
    tc = s2d.truth_contour(res['C_end'])
    cl = np.vstack([tc, tc[:1]])
    ax.plot(cl[:, 0], cl[:, 1], 0, color='red', lw=1.5, label='truth boundary at 1800 s (z = 0)')
    t0c = s2d.truth_contour(s2d.init_field())
    c0 = np.vstack([t0c, t0c[:1]])
    ax.plot(c0[:, 0], c0[:, 1], 0, color='red', lw=1.0, ls='--', label='truth boundary at 0 s')
    for d, lab in zip(st.drones, ('drone A (boundary)', 'drone B (thickness)')):
        tr = np.array(d.traj); z = np.array(d.ztraj)
        ax.plot(tr[::4, 0], tr[::4, 1], z[::4], color=d.color, lw=0.8, label=f'{lab}, z {z.min():.0f}-{z.max():.0f} m')
        for k in range(0, len(tr), 600):
            ax.plot([tr[k, 0]] * 2, [tr[k, 1]] * 2, [0, z[k]], ':', color=d.color, lw=0.6)
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)'); ax.set_zlabel('z (m)')
    ax.set_zlim(0, 45); ax.view_init(elev=28, azim=-60)
    ax.legend(fontsize=8, loc='upper left')
    ax.set_title('S055 3D dual-altitude patrol (card wind): A low on the boundary, B high over the interior', fontsize=10)
    _save(fig, out, 'trajectories_3d.png')


def plot_altitudes(res, out):
    fig, ax = plt.subplots(figsize=(10, 4.2))
    t = np.arange(len(res['strats']['dual_corr'].drones[0].ztraj)) * DT_SIM
    for key, ls in (('single', '-'), ('dual', '--'), ('dual_corr', '-')):
        st = res['strats'][key]
        for d, nm in zip(st.drones, ('A', 'B')):
            ax.plot(t, d.ztraj, ls, color=COLORS[key], lw=1, label=f'{key} drone {nm}')
    ax.axhspan(*ZA_RANGE, color='orange', alpha=0.1, label='z_A range')
    ax.axhspan(*ZB_RANGE, color='cyan', alpha=0.1, label='z_B range')
    ax.set_xlabel('time (s)'); ax.set_ylabel('altitude (m)'); ax.set_ylim(0, 45); ax.grid(alpha=0.3)
    ax.legend(fontsize=7, ncol=3)
    ax.set_title('Altitude time series (the wave-adaptive z_A law moves A by < 0.01 m)')
    _save(fig, out, 'altitude_timeseries.png')


def plot_wave_field(res, out):
    hs, tc = res['hs_snap']
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(13, 5.4))
    ext = [0, s2d.AREA_SIZE, 0, s2d.AREA_SIZE]
    im = a1.imshow(hs.T, origin='lower', extent=ext, cmap='Blues')
    fig.colorbar(im, ax=a1, label='Hs (m)')
    cm, _ = cos_mean(hs)
    im2 = a2.imshow(100 * (1 / cm - 1).T, origin='lower', extent=ext, cmap='magma')
    fig.colorbar(im2, ax=a2, label='effective threshold increase (%)')
    for a in (a1, a2):
        c = np.vstack([tc, tc[:1]])
        a.plot(c[:, 0], c[:, 1], 'r-', lw=1.2)
        a.set_xlabel('x (m)')
    a1.set_ylabel('y (m)')
    print(f'  wave field at 900 s: Hs {hs.min():.2f}-{hs.max():.2f} m, largest threshold increase 1/<cos> - 1 = '
          f'{100 * (1 / cm.min() - 1):.4f}%')
    a1.set_title('Hs at t = 900 s\n(red = truth boundary)', fontsize=10)
    a2.set_title(f'Wave correction 1/<cos> - 1\nmax {100 * (1 / cm.min() - 1):.4f}% (card expected > 20%)', fontsize=10)
    _save(fig, out, 'wave_field.png')


def plot_sensor_threshold(res, out):
    z = np.linspace(8, 40, 200)
    fig, ax = plt.subplots(figsize=(8.5, 4.6))
    for lam, ls in ((LAMBDA_W, '-'), (LAMBDA_PROBE, '--')):
        for hs, col in ((0.1, 'tab:green'), (0.3, 'tab:blue'), (0.6, 'tab:red')):
            cm, _ = cos_mean(hs, lam)
            ax.plot(z, 1 / (np.exp(-MU_ATM * (z - Z_2D)) * cm), ls, color=col,
                    label=f'Hs {hs} m, lambda {lam:.0f} m')
    ax.axhline(1.0, color='k', lw=0.8)
    ax.set_xlabel('altitude z (m)'); ax.set_ylabel('C_thr_eff / C_thr (2D, z = 15 m)')
    ax.set_title('Effective detection threshold relative to the 2D sensor')
    ax.legend(fontsize=7, ncol=2); ax.grid(alpha=0.3)
    _save(fig, out, 'sensor_threshold.png')


def plot_timeseries(res, out):
    fig, axes = plt.subplots(3, 1, figsize=(10, 10), sharex=True)
    for key in STRATS:
        st = res['strats'][key]
        axes[0].plot(st.log['t'], st.log['eta'], color=COLORS[key], label=LABELS[key])
        axes[1].plot(st.log['t'], st.log['err'], color=COLORS[key])
        axes[2].plot(st.th_log['t'], 1e3 * np.array(st.th_log['eps'], dtype=float), color=COLORS[key])
    axes[0].axhline(s2d.ETA_TARGET, color='k', ls='--', lw=1)
    axes[2].axhline(1e3 * EPS_TARGET, color='k', ls='--', lw=1, label='target 0.02 mm')
    axes[0].set_ylabel('boundary coverage eta (verified)'); axes[0].legend(fontsize=8)
    axes[1].set_ylabel('mean boundary error (m)'); axes[1].set_yscale('log')
    axes[2].set_ylabel('thickness RMSE (mm)'); axes[2].legend(fontsize=8); axes[2].set_xlabel('time (s)')
    for a in axes:
        a.grid(alpha=0.3)
        for tr in np.arange(0, T_MISSION + 1, T_REPLAN * 10):
            a.axvline(tr, color='gray', lw=0.3)
    fig.suptitle('S055 3D card wind: coverage, boundary error and thickness RMSE')
    _save(fig, out, 'timeseries.png')


def plot_inversion_scatter(res, out):
    st = res['strats']['dual_corr']
    C, t = res['C_end'], T_MISSION
    cs = (np.arange(N_GRID_TH) + 0.5) * BOX / N_GRID_TH - BOX / 2
    X, Y = np.meshgrid(cs, cs, indexing='ij')
    wx, wy = X + s2d.X0[0] + st.u[0] * t, Y + s2d.X0[1] + st.u[1] * t
    c = map_coordinates(C, [wx.ravel() / s2d.H_CELL - 0.5, wy.ravel() / s2d.H_CELL - 0.5], order=1,
                        mode='nearest').reshape(X.shape)
    truth = H0 * g_age(t) * c
    obs = ~np.isnan(st.th_val) & (c >= s2d.C_THR)
    est = st.th_val * np.sqrt((st.th_t + T0_SPREAD) / (t + T0_SPREAD))
    tr, es = 1e3 * truth[obs], 1e3 * est[obs]
    r2 = 1 - np.sum((es - tr) ** 2) / np.sum((tr - tr.mean()) ** 2)
    rmse = float(np.sqrt(np.mean((es - tr) ** 2)))
    print(f'  inversion scatter (dual_corr, observed interior cells at 1800 s): n = {obs.sum()}, R^2 = {r2:.3f}, '
          f'RMSE = {rmse:.4f} mm')
    fig, ax = plt.subplots(figsize=(6, 5.5))
    ax.scatter(tr, es, s=12, color=COLORS['dual_corr'])
    lim = [0, max(tr.max(), es.max()) * 1.05]
    ax.plot(lim, lim, 'k--', lw=1)
    ax.set_xlabel('true thickness at cell centre (mm)'); ax.set_ylabel('mapped thickness (mm)')
    ax.set_title(f'Thickness map vs truth at 1800 s: n = {obs.sum()}, R^2 = {r2:.3f}, RMSE = {rmse:.4f} mm', fontsize=9)
    ax.grid(alpha=0.3)
    _save(fig, out, 'inversion_scatter.png')


def plot_sweeps(res, out):
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(12, 4.5))
    zbs = sorted(ZB_SWEEP + (card_zb(),))
    keys = ['dual' if zb == card_zb() else ('zb', zb) for zb in zbs]
    for m in res['mc']:
        a1.plot(zbs, [1e3 * m[k]['eps_mean'] for k in keys], color=COLORS['dual'], alpha=0.25)
    a1.plot(zbs, [1e3 * _mc(res, k, 'eps_mean').mean() for k in keys], 'o-', color=COLORS['dual'], label='mean')
    a1.axhline(1e3 * _mc(res, 'single', 'eps_mean').mean(), color=COLORS['single'], ls='--', label='single altitude 15 m')
    a1.axhline(1e3 * EPS_TARGET, color='k', ls=':', label='target')
    a1.axvspan(*ZB_RANGE, color='cyan', alpha=0.1, label='card z_B range')
    a1.set_xlabel('drone B altitude (m)'); a1.set_ylabel('mean thickness RMSE after 300 s (mm)')
    a1.legend(fontsize=8); a1.grid(alpha=0.3); a1.set_title('Thickness RMSE vs z_B (thin = trials)')
    a2.plot(zbs, [100 * _mc(res, k, 'obs_final').mean() for k in keys], 'o-', color=COLORS['dual'])
    a2.set_xlabel('drone B altitude (m)'); a2.set_ylabel('interior coarse cells observed at 1800 s (%)')
    a2.grid(alpha=0.3); a2.set_title('Mapped interior fraction vs z_B')
    _save(fig, out, 'zb_sweep.png')


def save_animation(res, out):
    frames = res['frames']
    n = len(frames)
    step = max(1, int(np.ceil((n - 1) / (ANIM_FRAMES - 1))))
    n_frames = int(np.ceil((n - 1) / step)) + 1
    idx = [min(f * step, n - 1) for f in range(n_frames)]
    fig, ax = plt.subplots(figsize=(7, 6.5))
    ext = [0, s2d.AREA_SIZE, 0, s2d.AREA_SIZE]
    im = ax.imshow(frames[0][1].T, origin='lower', extent=ext, cmap='inferno', vmin=0, vmax=1)
    wv = ax.imshow(frames[0][7].T, origin='lower', extent=[-50, 4050, -50, 4050], cmap='Blues', alpha=0.25)
    (tl,) = ax.plot([], [], 'r-', lw=1.2)
    (el,) = ax.plot([], [], '--', color='white', lw=1)
    (pa,) = ax.plot([], [], 'o', color='orange', ms=7, mec='k')
    (pb,) = ax.plot([], [], 's', color='cyan', ms=7, mec='k')
    txt = ax.text(0.02, 0.97, '', transform=ax.transAxes, color='w', va='top', fontsize=9)
    ax.set_xlim(1200, 3600); ax.set_ylim(1200, 3200)

    def update(f):
        t, C, tc, mu, dr, eta, eps, hs = frames[idx[f]]
        im.set_data(C.T); wv.set_data(hs.T)
        c = np.vstack([tc, tc[:1]]); m = np.vstack([mu, mu[:1]])
        tl.set_data(c[:, 0], c[:, 1]); el.set_data(m[:, 0], m[:, 1])
        pa.set_data([dr[0][0][0]], [dr[0][0][1]]); pb.set_data([dr[1][0][0]], [dr[1][0][1]])
        txt.set_text(f't = {t:.0f} s  eta = {eta:.2f}  thickness RMSE = {1e3 * eps:.4f} mm\n'
                     f'A z = {dr[0][1]:.1f} m, B z = {dr[1][1]:.1f} m')
        return im, wv, tl, el, pa, pb, txt

    anim = animation.FuncAnimation(fig, update, frames=n_frames, blit=False)
    p = os.path.join(out, 'animation.gif')
    anim.save(p, writer='pillow', fps=10, dpi=70)
    plt.close(fig)
    print(f'Saved: {p} ({n_frames} frames, every {step * s2d.EVAL_EVERY:.0f} s)')


if __name__ == '__main__':
    res = run_simulation()
    report(res)
    out = os.path.normpath(OUTPUT_DIR)
    os.makedirs(out, exist_ok=True)
    plot_trajectories_3d(res, out)
    plot_altitudes(res, out)
    plot_wave_field(res, out)
    plot_sensor_threshold(res, out)
    plot_timeseries(res, out)
    plot_inversion_scatter(res, out)
    plot_sweeps(res, out)
    save_animation(res, out)
