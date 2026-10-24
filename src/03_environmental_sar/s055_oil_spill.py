"""
S055 Coastline Oil Spill Tracking
=================================
An oil slick in a 4 x 4 km coastal area drifts with wind + tide (0.43 m/s) and spreads by
diffusion (advection-diffusion PDE on a 200 x 200 grid). Two drones with a binary
above/below-threshold oil sensor patrol the slick boundary (C = 0.08). Each drone flies a
zig-zag along its share of the planned boundary, samples the sensor every 20 m of flight,
and every change of the reading (a boundary crossing) gives a position measurement that
updates a per-landmark Kalman filter (36 landmarks). The ground station replans every 30 s.

Strategies (card):
  1. Static partition, no prediction: both drones patrol the t = 0 boundary forever
  2. Adaptive partition, no prediction: replan on the current estimate; the filter has no
     drift model (random walk whose process noise covers the drift)
  3. Adaptive partition + prediction: filter with the known drift, plan one epoch ahead
Boundary coverage eta(t) is the arc-length fraction of landmarks with a measurement in the
last 60 s. A 10-trial Monte Carlo randomises the wind direction (same truth for every
strategy), and a fleet-size sweep shows how many drones the 0.85 coverage target needs.

Usage:
    conda activate drones
    python src/03_environmental_sar/s055_oil_spill.py
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401
from contourpy import contour_generator
from scipy.ndimage import map_coordinates
from scipy.spatial import cKDTree

# ── Parameters (card) ─────────────────────────────────────────────────────────
AREA_SIZE   = 4000.0
GRID_N      = 200
H_CELL      = AREA_SIZE / GRID_N        # 20 m
C0          = 1.0
R0          = 200.0
X0          = np.array([2000.0, 2000.0])
U_CARD      = np.array([0.4, 0.15])     # m/s
D_DIFF      = 5.0                       # m^2/s
DT_FIELD    = 2.0                       # s
C_THR       = 0.08
N_LANDMARKS = 36
SIGMA_Q     = 0.5                       # m/s^0.5 (filter with drift model)
SIGMA_R     = 15.0                      # m
R_ASSOC     = 30.0                      # m
V_ARC       = 8.0                       # m/s
DELTA_S     = 20.0                      # m
T_REPLAN    = 30.0                      # s
T_STALE     = 60.0                      # s
T_MISSION   = 1800.0                    # s
DT_SIM      = 0.5                       # s
ALTITUDE    = 15.0                      # m (fixed)
ETA_TARGET  = 0.85
N_DRONES    = 2
N_TRIALS    = 10

# ── Parameters (added) ────────────────────────────────────────────────────────
ZZ_AMP      = 25.0                      # m — zig-zag half-width across the planned boundary
ZZ_WAVE     = 100.0                     # m — zig-zag wavelength along the boundary
GPS_SIGMA   = 3.0                       # m — position noise added to each crossing
SIGMA_Q_RW  = float(np.linalg.norm(U_CARD) * np.sqrt(T_REPLAN))   # m/s^0.5 — random-walk filter
EVAL_EVERY  = 10.0                      # s — metric sampling
SWEEP_K     = [2, 4, 8, 12]             # fleet sizes for the prediction strategy (trial 0)
ANIM_FRAMES = 90

STRATS = ['static', 'adaptive', 'predict']
LABELS = {'static': 'Static, no prediction', 'adaptive': 'Adaptive, no prediction',
          'predict': 'Adaptive + prediction'}
COLORS = {'static': '#7F7F7F', 'adaptive': '#1F77B4', 'predict': '#2CA02C'}
DRONE_COLORS = ['orange', 'cyan'] + list(plt.cm.tab10.colors)

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), '..', '..',
                          'outputs', '03_environmental_sar', 's055_oil_spill')

XS = (np.arange(GRID_N) + 0.5) * H_CELL


# ── Spill field ───────────────────────────────────────────────────────────────

def init_field(shape=(1.0, 0.0)):
    """Gaussian patch; shape = (axis factor f, orientation) gives semi-axes R0*f and R0/f."""
    f, ang = shape
    X, Y = np.meshgrid(XS, XS, indexing='ij')                 # C[i, j]: i -> x, j -> y
    u = (X - X0[0]) * np.cos(ang) + (Y - X0[1]) * np.sin(ang)
    w = -(X - X0[0]) * np.sin(ang) + (Y - X0[1]) * np.cos(ang)
    return C0 * np.exp(-(u ** 2 / (2 * (R0 * f) ** 2) + w ** 2 / (2 * (R0 / f) ** 2)))


def advance_field(C, u):
    adv_x = u[0] * (np.roll(C, -1, 0) - np.roll(C, 1, 0)) / (2 * H_CELL)
    adv_y = u[1] * (np.roll(C, -1, 1) - np.roll(C, 1, 1)) / (2 * H_CELL)
    lap = (np.roll(C, -1, 0) + np.roll(C, 1, 0) + np.roll(C, -1, 1) + np.roll(C, 1, 1)
           - 4 * C) / H_CELL ** 2
    return C - DT_FIELD * (adv_x + adv_y) + DT_FIELD * D_DIFF * lap


def sample_field(C, p):
    return float(map_coordinates(C, [[p[0] / H_CELL - 0.5], [p[1] / H_CELL - 0.5]],
                                 order=1, mode='nearest')[0])


def truth_contour(C):
    lines = contour_generator(XS, XS, C.T).lines(C_THR)
    v = max(lines, key=len)
    if np.allclose(v[0], v[-1]):
        v = v[:-1]
    if polygon_area(v) < 0:
        v = v[::-1]
    return v


def polygon_area(v):
    x, y = v[:, 0], v[:, 1]
    return 0.5 * np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y)


def resample_closed(v, n):
    w = np.vstack([v, v[:1]])
    seg = np.linalg.norm(np.diff(w, axis=0), axis=1)
    cum = np.concatenate([[0.0], np.cumsum(seg)])
    s = np.linspace(0, cum[-1], n, endpoint=False)
    return np.column_stack([np.interp(s, cum, w[:, 0]), np.interp(s, cum, w[:, 1])])


def perimeter(v):
    return float(np.sum(np.linalg.norm(np.roll(v, -1, axis=0) - v, axis=1)))


# ── Estimator ─────────────────────────────────────────────────────────────────

class BoundaryKF:
    def __init__(self, landmarks, drift, sigma_q):
        self.mu = landmarks.copy()
        self.P = np.tile(SIGMA_R ** 2 * np.eye(2), (len(landmarks), 1, 1))
        self.drift = drift
        self.sq = sigma_q
        self.t = 0.0

    def predict_to(self, t):
        dt = t - self.t
        if dt > 0:
            self.mu += self.drift * dt
            self.P += self.sq ** 2 * dt * np.eye(2)[None]
            self.t = t

    def normals(self):
        n = len(self.mu)
        tang = self.mu[(np.arange(n) + 1) % n] - self.mu[(np.arange(n) - 1) % n]
        tang /= np.linalg.norm(tang, axis=1, keepdims=True) + 1e-9
        return np.column_stack([tang[:, 1], -tang[:, 0]])          # outward for CCW order

    def update(self, k, m):
        nh = self.normals()[k]
        z = nh @ (m - self.mu[k])
        S = nh @ self.P[k] @ nh + SIGMA_R ** 2
        K = self.P[k] @ nh / S
        self.mu[k] += K * z
        self.P[k] = (np.eye(2) - np.outer(K, nh)) @ self.P[k]


def split_indices(mu, n_drones, balance):
    """Contiguous landmark arcs; balance=True equalises arc length (card rebalancing)."""
    n = len(mu)
    if not balance:
        b = [int(round(i * n / n_drones)) for i in range(n_drones + 1)]
    else:
        seg = np.linalg.norm(np.roll(mu, -1, axis=0) - mu, axis=1)
        cum = np.concatenate([[0.0], np.cumsum(seg)])
        b = [0] + [int(np.argmin(np.abs(cum - cum[-1] * i / n_drones))) for i in range(1, n_drones)] + [n]
    return [list(range(b[i], b[i + 1] + 1)) for i in range(n_drones)]   # share end landmarks


def zigzag_path(pts, normals_pts):
    """Dense reference path (2 m) along an open polyline with a sinusoidal lateral offset."""
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    cum = np.concatenate([[0.0], np.cumsum(seg)])
    s = np.arange(0.0, cum[-1] + 1e-9, 2.0)
    base = np.column_stack([np.interp(s, cum, pts[:, 0]), np.interp(s, cum, pts[:, 1])])
    nrm = np.column_stack([np.interp(s, cum, normals_pts[:, 0]), np.interp(s, cum, normals_pts[:, 1])])
    nrm /= np.linalg.norm(nrm, axis=1, keepdims=True) + 1e-9
    return base + (ZZ_AMP * np.sin(2 * np.pi * s / ZZ_WAVE))[:, None] * nrm


# ── Mission ───────────────────────────────────────────────────────────────────

class Drone:
    def __init__(self, pos, color):
        self.pos = pos.copy()
        self.path = None
        self.k = 0
        self.dir = 1
        self.dist_since = 0.0
        self.last_read = None
        self.last_pos = None
        self.traj = [pos.copy()]
        self.color = color

    def set_path(self, path):
        d = np.linalg.norm(path - self.pos, axis=1)
        self.path = path
        self.k = int(np.argmin(d))

    def step(self, dt):
        """Fly toward the next path point at V_ARC; ping-pong at the arc ends."""
        budget = V_ARC * dt
        moved = 0.0
        while budget > 1e-9:
            tgt = self.path[self.k]
            d = tgt - self.pos
            L = np.linalg.norm(d)
            if L <= budget:
                self.pos = tgt.copy()
                budget -= L
                moved += L
                nk = self.k + self.dir
                if nk < 0 or nk >= len(self.path):
                    self.dir *= -1
                    nk = self.k + self.dir
                    if nk < 0 or nk >= len(self.path):
                        break
                self.k = nk
            else:
                self.pos = self.pos + d / L * budget
                moved += budget
                budget = 0.0
        assert moved <= V_ARC * dt + 1e-6, 'speed limit exceeded'
        return moved


class Strategy:
    def __init__(self, name, lm0, u, n_drones, rng):
        self.name = name
        predict = name == 'predict'
        self.kf = BoundaryKF(lm0, u if predict else np.zeros(2), SIGMA_Q if predict else SIGMA_Q_RW)
        self.u = u
        self.predict = predict
        self.lm0 = lm0.copy()
        self.n_drones = n_drones
        self.rng = rng
        self.t_last = np.full(N_LANDMARKS, -1e9)       # verified (crossing) freshness; none yet
        self.t_nom = np.full(N_LANDMARKS, -1e9)        # card nominal freshness (any sample)
        self.arcs = split_indices(lm0, n_drones, balance=False)
        self.drones = [Drone(lm0[a[len(a) // 2] % N_LANDMARKS], DRONE_COLORS[i]) for i, a in enumerate(self.arcs)]
        self.n_meas = 0
        self.log = {'t': [], 'eta': [], 'eta_nom': [], 'err': [], 'err_sd': [], 'trP': [],
                    'per_est': [], 'per_true': []}
        self.snap = {}
        self.split_log = []

    def replan(self, t):
        self.kf.predict_to(t)
        if self.name == 'static':
            plan = self.lm0
            arcs = self.arcs
        else:
            plan = self.kf.mu + (self.u * T_REPLAN if self.predict else 0.0)
            arcs = split_indices(self.kf.mu, self.n_drones, balance=True)
            self.arcs = arcs
        self.split_log.append([a[0] for a in arcs])
        nrm = BoundaryKF(plan, np.zeros(2), 0).normals()
        for d, a in zip(self.drones, arcs):
            idx = [i % N_LANDMARKS for i in a]
            d.set_path(zigzag_path(plan[idx], nrm[idx]))

    def sense(self, d, C, t):
        reading = sample_field(C, d.pos) >= C_THR
        mu = self.kf.mu + self.kf.drift * (t - self.kf.t)       # current estimate (no update)
        dk = np.linalg.norm(mu - d.pos, axis=1)
        near = dk <= R_ASSOC
        self.t_nom[near] = t
        if d.last_read is not None and reading != d.last_read:
            m = 0.5 * (d.pos + d.last_pos) + self.rng.normal(0, GPS_SIGMA, 2)
            self.kf.predict_to(t)
            dk = np.linalg.norm(self.kf.mu - m, axis=1)
            k = int(np.argmin(dk))
            if dk[k] <= R_ASSOC:
                self.kf.update(k, m)
                self.t_last[k] = t
                self.n_meas += 1
        d.last_read = reading
        d.last_pos = d.pos.copy()

    def eta(self, t, verified=True):
        mu = self.kf.mu + self.kf.drift * (t - self.kf.t)
        seg = np.linalg.norm(np.roll(mu, -1, axis=0) - mu, axis=1)
        last = self.t_last if verified else self.t_nom
        return float(np.sum(seg * ((t - last) <= T_STALE)) / np.sum(seg))


def run_trial(u, rng_seed, sweep=False, record=False, shape=(1.0, 0.0)):
    C = init_field(shape)
    lm0 = resample_closed(truth_contour(C), N_LANDMARKS)
    configs = [(s, N_DRONES) for s in STRATS]
    if sweep:
        configs += [('predict', k) for k in SWEEP_K if k != N_DRONES]
    strats = {cfg: Strategy(cfg[0], lm0, u, cfg[1], np.random.default_rng(rng_seed)) for cfg in configs}
    n_steps = int(round(T_MISSION / DT_SIM))
    field_every = int(round(DT_FIELD / DT_SIM))
    replan_every = int(round(T_REPLAN / DT_SIM))
    eval_every = int(round(EVAL_EVERY / DT_SIM))
    snaps_t = [0.0, 600.0, 1200.0, 1800.0]
    frames = []
    true_log = []
    for step in range(n_steps + 1):
        t = step * DT_SIM
        if step % replan_every == 0 and step < n_steps:
            for st in strats.values():
                st.replan(t)
        if step % eval_every == 0:
            tc = truth_contour(C)
            tree = cKDTree(resample_closed(tc, 2000))
            per_t = perimeter(tc)
            true_log.append((t, per_t))
            for st in strats.values():
                mu = st.kf.mu + st.kf.drift * (t - st.kf.t)
                e, _ = tree.query(mu)
                st.log['t'].append(t)
                st.log['eta'].append(st.eta(t))
                st.log['eta_nom'].append(st.eta(t, verified=False))
                st.log['err'].append(float(e.mean()))
                st.log['err_sd'].append(float(e.std()))
                st.log['trP'].append(float(np.trace(st.kf.P, axis1=1, axis2=2).mean()))
                st.log['per_est'].append(perimeter(mu))
                st.log['per_true'].append(per_t)
                if record and any(abs(t - ts) < 1e-9 for ts in snaps_t):
                    st.snap[t] = (C.copy(), tc, mu.copy(), np.trace(st.kf.P, axis1=1, axis2=2).copy())
            if record:
                st = strats[('predict', N_DRONES)]
                mu = st.kf.mu + st.kf.drift * (t - st.kf.t)
                frames.append((t, C.copy(), tc, mu.copy(), [d.pos.copy() for d in st.drones],
                               st.log['eta'][-1]))
        if step == n_steps:
            break
        for st in strats.values():
            for d in st.drones:
                d.dist_since += d.step(DT_SIM)
                if record:
                    d.traj.append(d.pos.copy())
                if d.dist_since >= DELTA_S - 1e-9:
                    d.dist_since -= DELTA_S
                    st.sense(d, C, t + DT_SIM)
        if (step + 1) % field_every == 0:
            C = advance_field(C, u)
            assert C.min() > -1e-3, 'field went negative (unstable scheme)'
    return strats, frames, true_log


def summarise(st):
    lg = {k: np.array(v) for k, v in st.log.items()}
    dt = EVAL_EVERY
    lost = np.where(lg['err'] > ZZ_AMP)[0]
    return {
        't_lost': float(lg['t'][lost[0]]) if len(lost) else np.nan,
        'eta_mean': float(lg['eta'].mean()), 'eta_nom_mean': float(lg['eta_nom'].mean()),
        'eta_frac_ok': float(np.mean(lg['eta'] >= ETA_TARGET)),
        'err_mean': float(lg['err'].mean()), 'err_final': float(lg['err'][-1]),
        'stale_kms': float(np.sum((1 - lg['eta']) * lg['per_true']) * dt / 1000.0),
        'per_err_final': float(lg['per_est'][-1] - lg['per_true'][-1]),
        'n_meas': st.n_meas,
    }


def run_simulation():
    strats, frames, true_log = run_trial(U_CARD, 0, sweep=True, record=True)
    main = {s: strats[(s, N_DRONES)] for s in STRATS}
    sweep = {k: summarise(strats[('predict', k)]) for k in SWEEP_K}
    rng = np.random.default_rng(55)
    angles = rng.uniform(0, 2 * np.pi, N_TRIALS)
    speed = float(np.linalg.norm(U_CARD))
    mc = {s: [] for s in STRATS}
    for i, a in enumerate(angles):
        u = speed * np.array([np.cos(a), np.sin(a)])
        st, _, _ = run_trial(u, 100 + i)
        for s in STRATS:
            mc[s].append(summarise(st[(s, N_DRONES)]))
    # second Monte Carlo: elongated slicks (breaks the rotational symmetry of the first one)
    rng2 = np.random.default_rng(56)
    shapes = [(float(rng2.uniform(1.0, 1.5)), float(rng2.uniform(0, np.pi)),
               float(rng2.uniform(0, 2 * np.pi))) for _ in range(N_TRIALS)]
    mc2 = {s: [] for s in STRATS}
    for i, (f, ang, a) in enumerate(shapes):
        u = speed * np.array([np.cos(a), np.sin(a)])
        st, _, _ = run_trial(u, 200 + i, shape=(f, ang))
        for s in STRATS:
            mc2[s].append(summarise(st[(s, N_DRONES)]))
    return main, sweep, (mc, mc2, shapes), frames, true_log, angles


# ── Plotting ──────────────────────────────────────────────────────────────────

def _save(fig, out_dir, name):
    path = os.path.join(out_dir, name)
    fig.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def _closed(v):
    return np.vstack([v, v[:1]])


def plot_snapshots(main, out_dir):
    st = main['predict']
    fig, axes = plt.subplots(1, 4, figsize=(20, 5.4))
    for ax, t in zip(axes, sorted(st.snap)):
        C, tc, mu, trP = st.snap[t]
        ax.imshow(C.T, origin='lower', extent=[0, AREA_SIZE, 0, AREA_SIZE], cmap='inferno', vmin=0, vmax=1)
        ax.plot(*_closed(tc).T, '-', color='red', lw=1.5, label='truth boundary')
        ax.plot(*_closed(mu).T, '--', color='white', lw=1.2, label='estimate')
        ax.scatter(mu[:, 0], mu[:, 1], s=0.5 * np.sqrt(trP), c='lime', edgecolor='k', lw=0.4, zorder=4,
                   label='landmarks (area ~ sqrt(tr P))')
        ax.set_xlim(1200, 3600)
        ax.set_ylim(1200, 3200)
        ax.set_title(f't = {t:.0f} s', fontsize=10)
        ax.set_xlabel('x (m)')
    axes[0].set_ylabel('y (m)')
    axes[0].legend(fontsize=7, loc='upper left')
    fig.suptitle('S055 Spill evolution and boundary estimate (adaptive + prediction, card wind)')
    _save(fig, out_dir, 'spill_snapshots.png')


def plot_trajectories(main, frames, out_dir):
    fig, axes = plt.subplots(1, 3, figsize=(19, 6.5))
    t, C, tc, mu, _, _ = frames[-1]
    for ax, s in zip(axes, STRATS):
        st = main[s]
        ax.imshow(C.T, origin='lower', extent=[0, AREA_SIZE, 0, AREA_SIZE], cmap='Greys', vmin=0, vmax=1)
        ax.plot(*_closed(tc).T, '-', color='red', lw=1.5, label='truth at 1800 s')
        mu_s = st.kf.mu
        ax.plot(*_closed(mu_s).T, '--', color='k', lw=1.0, label='estimate at 1800 s')
        for d in st.drones:
            tr = np.array(d.traj)
            ax.plot(tr[:, 0], tr[:, 1], '-', color=d.color, lw=0.6)
        for b in st.split_log[-1]:
            ax.plot(*mu_s[b % N_LANDMARKS], 'D', color='magenta', ms=8, mec='k')
        ax.set_xlim(1200, 3600)
        ax.set_ylim(1200, 3200)
        ax.set_aspect('equal')
        ax.set_title(f'{LABELS[s]} ({st.n_meas} crossings)', fontsize=10)
        ax.set_xlabel('x (m)')
    axes[0].set_ylabel('y (m)')
    axes[0].legend(fontsize=7, loc='upper left')
    fig.suptitle('S055 Drone trails over the whole 30 min mission (orange = drone A, cyan = drone B; the slick '
                 'moved 769 m), magenta = last partition split points', fontsize=11)
    _save(fig, out_dir, 'trajectories.png')


def plot_coverage(main, sweep_traces, out_dir):
    fig, ax = plt.subplots(figsize=(10, 4.8))
    for s in STRATS:
        lg = main[s].log
        ax.plot(lg['t'], lg['eta'], color=COLORS[s], label=f'{LABELS[s]} (verified)')
        ax.plot(lg['t'], lg['eta_nom'], color=COLORS[s], ls=':', lw=1,
                label=f'{LABELS[s]} (card: any sample near landmark)')
    ax.axhline(ETA_TARGET, color='red', ls='--', label='target 0.85')
    for tr in np.arange(0, T_MISSION + 1, T_REPLAN):
        ax.plot([tr, tr], [-0.03, -0.01], color='k', lw=0.6)
    ax.set_ylim(-0.04, 1.05)
    ax.set_xlabel('time (s)  (ticks = replanning epochs)')
    ax.set_ylabel('boundary coverage η')
    ax.legend(fontsize=7, ncol=2)
    ax.grid(alpha=0.3)
    ax.set_title('S055 Boundary coverage η(t), two drones, card wind')
    _save(fig, out_dir, 'coverage_timeseries.png')


def plot_error(main, out_dir):
    fig, axes = plt.subplots(1, 3, figsize=(18, 4.6))
    for s in STRATS:
        lg = {k: np.array(v) for k, v in main[s].log.items()}
        axes[0].plot(lg['t'], lg['err'], color=COLORS[s], label=LABELS[s])
        axes[0].fill_between(lg['t'], np.maximum(lg['err'] - lg['err_sd'], 0), lg['err'] + lg['err_sd'],
                             color=COLORS[s], alpha=0.15)
        axes[1].plot(lg['t'], lg['trP'], color=COLORS[s], label=LABELS[s])
        axes[2].plot(lg['t'], lg['per_est'], color=COLORS[s], label=f'{LABELS[s]} estimate')
    lg = main['predict'].log
    axes[2].plot(lg['t'], lg['per_true'], color='red', lw=2, ls='--', label='truth')
    axes[0].set_ylabel('mean landmark error ē (m)')
    axes[0].set_yscale('log')
    axes[1].set_ylabel('mean tr(P) (m²)')
    axes[1].set_yscale('log')
    axes[2].set_ylabel('perimeter (m)')
    for ax, t in zip(axes, ['Boundary estimation error (±1 std band)', 'Kalman covariance trace',
                            'Perimeter growth']):
        ax.set_xlabel('time (s)')
        ax.set_title(t, fontsize=10)
        ax.grid(alpha=0.3)
        ax.legend(fontsize=7)
    fig.suptitle('S055 Estimation quality, card wind')
    plt.tight_layout()
    _save(fig, out_dir, 'estimation_error.png')


def plot_mc(mc, mc2, sweep, out_dir):
    fig, axes = plt.subplots(1, 4, figsize=(19, 4.6))
    keys = [('eta_mean', 'mean η (verified)'), ('err_mean', 'mean landmark error ē (m)'),
            ('stale_kms', 'stale arc-time (km·s)')]
    for ax, (k, lab) in zip(axes[:3], keys):
        vals = [np.array([r[k] for r in mc[s]]) for s in STRATS]
        vals2 = [np.array([r[k] for r in mc2[s]]) for s in STRATS]
        ax.bar(np.arange(3) - 0.2, [v.mean() for v in vals], 0.4, yerr=[v.std() for v in vals],
               color=[COLORS[s] for s in STRATS], edgecolor='k', capsize=4,
               label='MC1: circular slick, random wind direction')
        ax.bar(np.arange(3) + 0.2, [v.mean() for v in vals2], 0.4, yerr=[v.std() for v in vals2],
               color=[COLORS[s] for s in STRATS], edgecolor='k', hatch='//', capsize=4,
               label='MC2: elongated slick, random orientation and wind')
        ax.set_xticks(range(3))
        ax.set_xticklabels([LABELS[s].replace(', ', ',\n') for s in STRATS], fontsize=8)
        ax.set_title(lab, fontsize=10)
        if k == 'eta_mean':
            ax.axhline(ETA_TARGET, color='red', ls='--', lw=1)
            ax.set_ylim(0, 1)
            ax.legend(fontsize=7)
        if k == 'err_mean':
            ax.set_yscale('log')
    ks = sorted(sweep)
    axes[3].plot(ks, [sweep[k]['eta_mean'] for k in ks], 'o-', color=COLORS['predict'], label='mean η')
    axes[3].plot(ks, [sweep[k]['eta_frac_ok'] for k in ks], 's--', color='k', label='time fraction η ≥ 0.85')
    axes[3].axhline(ETA_TARGET, color='red', ls='--', lw=1)
    axes[3].set_xlabel('number of drones')
    axes[3].set_ylim(0, 1.05)
    axes[3].legend(fontsize=8)
    axes[3].set_title('Fleet size (adaptive + prediction, card wind)', fontsize=10)
    fig.suptitle(f'S055 Monte Carlo, {N_TRIALS} trials each (plain = MC1, hatched = MC2; mean ± std) and fleet-size sweep')
    plt.tight_layout()
    _save(fig, out_dir, 'mc_comparison.png')


def plot_spacetime_3d(main, frames, out_dir):
    fig = plt.figure(figsize=(10, 7.5))
    ax = fig.add_subplot(111, projection='3d')
    for f in frames[::18]:
        t, C, tc, mu, _, _ = f
        ax.plot(*_closed(tc).T, np.full(len(tc) + 1, t), color='red', lw=1)
        ax.plot(*_closed(mu).T, np.full(N_LANDMARKS + 1, t), '--', color='tab:green', lw=1)
    tr = np.array(main['predict'].drones[0].traj)
    tt = np.arange(len(tr)) * DT_SIM
    ax.plot(tr[::4, 0], tr[::4, 1], tt[::4], color='orange', lw=0.5, label='drone A (x, y, t)')
    ax.plot([], [], color='red', label='truth boundary')
    ax.plot([], [], '--', color='tab:green', label='estimate (prediction)')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_zlabel('time (s)')
    ax.view_init(elev=22, azim=-55)
    ax.legend(fontsize=8)
    ax.set_title('S055 Space-time view: the boundary drifts and grows; drone A patrols its half')
    _save(fig, out_dir, 'spacetime_3d.png')


def save_animation(frames, main, out_dir):
    n = len(frames)
    step = max(1, int(np.ceil((n - 1) / ANIM_FRAMES)))
    idx = [min(i * step, n - 1) for i in range(int(np.ceil((n - 1) / step)) + 1)]
    st = main['predict']
    trs = [np.array(d.traj) for d in st.drones]
    fig, ax = plt.subplots(figsize=(6.5, 6))
    im = ax.imshow(frames[0][1].T, origin='lower', extent=[0, AREA_SIZE, 0, AREA_SIZE], cmap='inferno',
                   vmin=0, vmax=1)
    tl, = ax.plot([], [], '-', color='red', lw=1.5)
    el, = ax.plot([], [], '--', color='white', lw=1.2)
    dl = [ax.plot([], [], 'o', color=c, ms=7, mec='k')[0] for c in DRONE_COLORS[:N_DRONES]]
    tails = [ax.plot([], [], '-', color=c, lw=0.6)[0] for c in DRONE_COLORS[:N_DRONES]]
    ax.set_xlim(1200, 3600)
    ax.set_ylim(1200, 3200)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')

    def update(fi):
        t, C, tc, mu, dpos, eta = frames[idx[fi]]
        im.set_data(C.T)
        tl.set_data(*_closed(tc).T)
        el.set_data(*_closed(mu).T)
        k = int(round(t / DT_SIM))
        for d, p, tail, tr in zip(dl, dpos, tails, trs):
            d.set_data([p[0]], [p[1]])
            tail.set_data(tr[max(0, k - 120):k + 1, 0], tr[max(0, k - 120):k + 1, 1])
        flash = ' [replan]' if t % T_REPLAN < EVAL_EVERY - 1e-9 else ''
        ax.set_title(f'S055 Adaptive + prediction, t = {t:.0f} s{flash}\n'
                     f'η = {eta:.2f} (red = truth, white = estimate)', fontsize=10)
        return [im, tl, el] + dl + tails

    anim = animation.FuncAnimation(fig, update, frames=len(idx), blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    anim.save(path, writer='pillow', fps=10, dpi=80)
    plt.close(fig)
    from PIL import Image
    with Image.open(path) as gif:
        n_file = gif.n_frames
    print(f'Saved: {path} ({len(idx)} frames rendered, {n_file} in the file, one every '
          f'{step * EVAL_EVERY:.0f} s)')


if __name__ == '__main__':
    main, sweep, (mc, mc2, shapes), frames, true_log, angles = run_simulation()
    C = init_field()
    r_b0 = R0 * np.sqrt(2 * np.log(C0 / C_THR))
    sig_f = np.sqrt(R0 ** 2 + 2 * D_DIFF * T_MISSION)
    r_b1 = sig_f * np.sqrt(2 * np.log(C0 * R0 ** 2 / sig_f ** 2 / C_THR))
    print('\n=== S055 Coastline Oil Spill Tracking — Results ===')
    print(f'  Stability: u dt/h = {np.linalg.norm(U_CARD) * DT_FIELD / H_CELL:.3f}, '
          f'D dt/h^2 = {D_DIFF * DT_FIELD / H_CELL ** 2:.3f}, cell Peclet = '
          f'{np.max(np.abs(U_CARD)) * H_CELL / D_DIFF:.2f}')
    print(f'  Analytic boundary radius: {r_b0:.0f} m at t = 0, {r_b1:.0f} m at t = 1800 s; '
          f'drift {np.linalg.norm(U_CARD) * T_MISSION:.0f} m')
    print(f'  Truth perimeter: {true_log[0][1]:.0f} m at t = 0, {true_log[-1][1]:.0f} m at t = 1800 s')
    zz = zigzag_path(np.array([[0.0, 0.0], [1000.0, 0.0]]), np.array([[0.0, 1.0], [0.0, 1.0]]))
    zz_factor = float(np.sum(np.linalg.norm(np.diff(zz, axis=0), axis=1)) / 1000.0)
    per = true_log[0][1]
    print(f'  Zig-zag path factor {zz_factor:.3f} -> along-boundary speed {V_ARC / zz_factor:.2f} m/s')
    per1 = true_log[-1][1]
    need = lambda P, v: ETA_TARGET * P / (v * T_STALE)
    print(f'  Fresh arc per drone in T_stale: {V_ARC / zz_factor * T_STALE:.0f} m; '
          f'drones needed for eta = 0.85 (one pass in T_stale): with the zig-zag '
          f'{need(per, V_ARC / zz_factor):.1f} (t = 0 perimeter {per:.0f} m) to '
          f'{need(per1, V_ARC / zz_factor):.1f} (final {per1:.0f} m); at the full {V_ARC:.0f} m/s without '
          f'zig-zag {need(per, V_ARC):.1f} to {need(per1, V_ARC):.1f}')
    print(f'  Drift per replanning epoch {np.linalg.norm(U_CARD) * T_REPLAN:.1f} m; '
          f'half the t = 0 perimeter (one drone arc) {per / 2:.0f} m')
    print(f'  Random-walk filter process noise sigma_q = |u| sqrt(T_replan) = {SIGMA_Q_RW:.2f} m/s^0.5')
    print('  [Card wind, 2 drones]')
    for s in STRATS:
        r = summarise(main[s])
        print(f"    {LABELS[s]:25s} eta mean {r['eta_mean']:.3f} (card nominal {r['eta_nom_mean']:.3f}), "
              f"time with eta >= 0.85 {100 * r['eta_frac_ok']:.1f}%, error mean {r['err_mean']:.1f} m, "
              f"final {r['err_final']:.1f} m, stale arc-time {r['stale_kms']:.0f} km*s, "
              f"final perimeter error {r['per_err_final']:+.0f} m, crossings {r['n_meas']}, "
              f"mean error first > {ZZ_AMP:.0f} m at {r['t_lost']:.0f} s")
    print(f'  Monte Carlo ({N_TRIALS} wind directions, |u| = {np.linalg.norm(U_CARD):.3f} m/s): '
          f'angles ' + ', '.join(f'{np.degrees(a):.0f}' for a in angles) + ' deg')
    for s in STRATS:
        a = {k: np.array([r[k] for r in mc[s]]) for k in mc[s][0]}
        print(f"    {LABELS[s]:25s} eta {a['eta_mean'].mean():.3f} +- {a['eta_mean'].std():.3f} "
              f"(nominal {a['eta_nom_mean'].mean():.3f}), error {a['err_mean'].mean():.1f} +- "
              f"{a['err_mean'].std():.1f} m (final {a['err_final'].mean():.1f} +- {a['err_final'].std():.1f}), "
              f"stale {a['stale_kms'].mean():.0f} +- {a['stale_kms'].std():.0f} km*s, "
              f"crossings {a['n_meas'].mean():.0f}, mean error > {ZZ_AMP:.0f} m in "
              f"{int(np.sum(np.isfinite(a['t_lost'])))}/{N_TRIALS} trials "
              f"(first at {np.nanmean(a['t_lost']) if np.isfinite(a['t_lost']).any() else np.nan:.0f} s on average)")
    print(f'  Monte Carlo 2 ({N_TRIALS} elongated slicks, axis factor U(1, 1.5), random orientation '
          f'and wind direction):')
    for s in STRATS:
        a = {k: np.array([r[k] for r in mc2[s]]) for k in mc2[s][0]}
        print(f"    {LABELS[s]:25s} eta {a['eta_mean'].mean():.3f} +- {a['eta_mean'].std():.3f}, "
              f"error {a['err_mean'].mean():.1f} +- {a['err_mean'].std():.1f} m "
              f"(final {a['err_final'].mean():.1f} +- {a['err_final'].std():.1f}), "
              f"stale {a['stale_kms'].mean():.0f} +- {a['stale_kms'].std():.0f} km*s, "
              f"crossings {a['n_meas'].mean():.0f}")
    e2 = np.array([[r['err_mean'] for r in mc2[s]] for s in STRATS])
    print('  MC2 lowest mean error: ' + ', '.join(f'{LABELS[s]} {int(np.sum(np.argmin(e2, 0) == i))}'
                                                  for i, s in enumerate(STRATS)) + f' of {N_TRIALS}')
    r_err = np.array([r['err_mean'] for r in mc['adaptive']]).mean() / \
        np.array([r['err_mean'] for r in mc['predict']]).mean()
    print(f'  MC1 mean error ratio adaptive (no prediction) / prediction: {r_err:.1f}x')
    e = np.array([[r['err_mean'] for r in mc[s]] for s in STRATS])
    h = np.array([[r['eta_mean'] for r in mc[s]] for s in STRATS])
    print('  Lowest mean error: ' + ', '.join(f'{LABELS[s]} {int(np.sum(np.argmin(e, 0) == i))}'
                                              for i, s in enumerate(STRATS)) + f' of {N_TRIALS}')
    print('  Highest mean eta:  ' + ', '.join(f'{LABELS[s]} {int(np.sum(np.argmax(h, 0) == i))}'
                                              for i, s in enumerate(STRATS)) + f' of {N_TRIALS}')
    print('  Trials with mean eta >= 0.85: ' + ', '.join(
        f'{LABELS[s]} {int(np.sum(h[i] >= ETA_TARGET))}' for i, s in enumerate(STRATS)))
    print('  Fleet-size sweep (adaptive + prediction, card wind):')
    for k in SWEEP_K:
        r = sweep[k]
        print(f"    K = {k:2d}: eta mean {r['eta_mean']:.3f}, time with eta >= 0.85 "
              f"{100 * r['eta_frac_ok']:.1f}%, error mean {r['err_mean']:.1f} m")

    out = os.path.normpath(OUTPUT_DIR)
    os.makedirs(out, exist_ok=True)
    plot_snapshots(main, out)
    plot_trajectories(main, frames, out)
    plot_coverage(main, None, out)
    plot_error(main, out)
    plot_mc(mc, mc2, sweep, out)
    plot_spacetime_3d(main, frames, out)
    save_animation(frames, main, out)
