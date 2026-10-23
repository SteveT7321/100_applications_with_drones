"""
S045 Chemical Plume Tracing
============================
A single drone traces a chemical plume back to its source. Wind blows along +x,
so the plume extends DOWNWIND (x > x_source); the drone starts downwind at
(170, 100) m and must move UPWIND (-x) to reach the source.

Plume: Gaussian plume (Pasquill-Gifford class D) from a leak released at the
drone's flight height (2 m), with ground reflection, expressed in mg/m^3
(normalised units). Measurements are intermittent (Bernoulli dropout) with
additive Gaussian noise and clipped at zero.

Strategies (same sensor, speed limits and source positions per trial):
  1. Gradient ascent     — always move along the finite-difference gradient.
  2. Cast-and-surge      — surge upwind (biased by the gradient) while in contact,
                           cast crosswind with growing legs when contact is lost,
                           return to the last contact point after 30 s.
  3. PF guided           — cast-and-surge blended 50/50 with the direction to the
                           particle-filter posterior mean once the posterior is
                           concentrated (std < 15 m).
The particle filter uses the exact measurement model (intermittency mixture),
systematic resampling with a seeded generator and 1 m roughening jitter.

Usage:
    conda activate drones
    python src/03_environmental_sar/s045_plume_tracing.py
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.colors as mcolors
from scipy.special import ndtr

# ── Parameters ─────────────────────────────────────────────────────────────────
WIND_SPEED   = 3.0      # m/s, along +x axis
Q_SOURCE     = 1.0      # g/s, emission rate
Z_DRONE      = 2.0      # m, flight altitude
H_SOURCE     = 2.0      # m, release height of the leak (= flight altitude)
C_REF        = 1e-3     # g/m^3 per normalised unit (normalised units = mg/m^3)
SIGMA_NOISE  = 0.05     # normalised concentration noise std
C_THRESH     = 0.15     # contact threshold (3 sigma_noise)
C_INTERMIT   = 2.0      # intermittency scale: p_detect = 1 - exp(-C / C_INTERMIT)
V_SURGE      = 1.5      # m/s, surge speed
V_CAST       = 0.8      # m/s, cast speed
V_DRIFT      = 0.2      # m/s, upwind drift while casting / gradient ascent out of contact (card)
L_CAST       = 8.0      # m, first cast leg; leg k has length k * L_CAST
T_CAST_MAX   = 30.0     # s, max cast duration before declaring lost
ALPHA_UPWIND = 0.4      # upwind bias weight in surge command
GRAD_DELTA   = 1.0      # m, finite-difference probe offset
N_PARTICLES  = 500      # particle filter population
PF_JITTER    = 1.0      # m, roughening noise added after resampling
PF_CONF_STD  = 15.0     # m, PF guidance used once posterior std is below this
R_CAPTURE    = 3.0      # m, source-found radius
V_MAX        = 2.0      # m/s, max drone speed
TAU_V        = 0.5      # s, first-order speed response time constant
DT           = 0.1      # s, simulation timestep
T_MAX        = 600.0    # s, mission timeout
AREA         = 200.0    # m, arena side length
N_TRIALS     = 20       # Monte Carlo runs per strategy

UPWIND     = np.array([-1.0, 0.0])          # wind blows toward +x
POS_START  = np.array([170.0, 100.0])       # drone start (downwind)
DEMO_SOURCE = np.array([55.0, 107.0])       # source of the demonstration run
SRC_X_RANGE = (40.0, 70.0)                  # MC source region (upwind part of the area)
SRC_Y_RANGE = (60.0, 140.0)
DEMO_SEED  = 999
N_ANIM_FRAMES = 150

STRATS = ['gradient', 'cast_surge', 'pf_guided']
LABELS = {'gradient': 'Gradient Ascent', 'cast_surge': 'Cast-and-Surge', 'pf_guided': 'PF Guided'}
COLORS = {'gradient': 'orange', 'cast_surge': 'deepskyblue', 'pf_guided': 'lime'}

OUTPUT_DIR = os.path.normpath(os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '03_environmental_sar', 's045_plume_tracing',
))

# ── Physics ────────────────────────────────────────────────────────────────────

def sigma_y(x):
    """Lateral dispersion coefficient — Pasquill-Gifford class D."""
    return 0.08 * x / np.sqrt(1.0 + 0.0002 * x)


def sigma_z(x):
    """Vertical dispersion coefficient — Pasquill-Gifford class D."""
    return 0.06 * x / np.sqrt(1.0 + 0.0015 * x)


def concentration(pos, src):
    """
    Time-averaged concentration (normalised, mg/m^3) at pos from source(s) src.
    Gaussian plume with release height H and ground reflection, sampled at z = Z_DRONE:
      C = Q / (2 pi sy sz u) exp(-dy^2 / 2 sy^2) [exp(-(z-H)^2/2sz^2) + exp(-(z+H)^2/2sz^2)]
    Zero upwind of the source (dx <= 0). Vectorised over src (N, 2) or pos (N, 2).
    """
    pos = np.asarray(pos, dtype=float)
    src = np.asarray(src, dtype=float)
    dx = pos[..., 0] - src[..., 0]
    dy = pos[..., 1] - src[..., 1]
    x = np.maximum(dx, 1e-3)
    sy, sz = sigma_y(x), sigma_z(x)
    vert = (np.exp(-(Z_DRONE - H_SOURCE) ** 2 / (2 * sz ** 2))
            + np.exp(-(Z_DRONE + H_SOURCE) ** 2 / (2 * sz ** 2)))
    c = Q_SOURCE / (2 * np.pi * sy * sz * WIND_SPEED) * np.exp(-dy ** 2 / (2 * sy ** 2)) * vert / C_REF
    return np.where(dx > 0, c, 0.0)


def p_detect(c):
    return 1.0 - np.exp(-c / C_INTERMIT)


def measure(pos, src, rng):
    """Noisy, intermittent reading: max(0, C * xi + eta), xi ~ Bernoulli(p_detect(C))."""
    c = float(concentration(pos, src))
    xi = rng.random() < p_detect(c)
    return max(0.0, c * xi + rng.normal(0.0, SIGMA_NOISE)), (c > 0 and not xi)


def gradient_hat(pos, src, rng):
    """Unit finite-difference gradient from four noisy probes at +/- delta."""
    m = [measure(pos + d, src, rng)[0] for d in
         (np.array([GRAD_DELTA, 0]), np.array([-GRAD_DELTA, 0]),
          np.array([0, GRAD_DELTA]), np.array([0, -GRAD_DELTA]))]
    g = np.array([(m[0] - m[1]), (m[2] - m[3])]) / (2 * GRAD_DELTA)
    return g / (np.linalg.norm(g) + 1e-9)


def unit(v):
    return v / (np.linalg.norm(v) + 1e-9)


# ── Navigation ─────────────────────────────────────────────────────────────────

class CastSurgeNavigator:
    """Surge (in contact) / cast (searching, growing crosswind legs) / lost (return)."""

    def __init__(self):
        self.state = 'cast'
        self.cast_dir = 1.0
        self.leg = 1
        self.leg_progress = 0.0
        self.t_no_contact = 0.0
        self.last_contact_pos = None
        self.last_grad_y = 1.0

    def command(self, pos, c, g):
        if c >= C_THRESH:
            self.state = 'surge'
            self.t_no_contact = 0.0
            self.leg, self.leg_progress = 1, 0.0
            self.last_contact_pos = pos.copy()
            self.last_grad_y = g[1] if abs(g[1]) > 1e-6 else self.last_grad_y
            self.cast_dir = np.sign(self.last_grad_y)
            return V_SURGE * unit(ALPHA_UPWIND * UPWIND + (1 - ALPHA_UPWIND) * g)
        self.t_no_contact += DT
        if self.t_no_contact > T_CAST_MAX and self.last_contact_pos is not None:
            self.state = 'lost'
            to_last = self.last_contact_pos - pos
            if np.linalg.norm(to_last) < 1.0:
                self.t_no_contact = 0.0
                self.leg, self.leg_progress = 1, 0.0
                self.state = 'cast'
            else:
                return V_CAST * unit(to_last)
        self.state = 'cast'
        self.leg_progress += V_CAST * DT
        if self.leg_progress >= self.leg * L_CAST:
            self.cast_dir *= -1.0
            self.leg += 1
            self.leg_progress = 0.0
        return V_CAST * np.array([0.0, self.cast_dir]) + V_DRIFT * UPWIND


class ParticleFilter:
    """Source-location particle filter with the exact intermittent measurement model."""

    def __init__(self, rng):
        self.rng = rng
        self.p = rng.uniform([0, 0], [AREA, AREA], size=(N_PARTICLES, 2))
        self.logw = np.zeros(N_PARTICLES)
        self.n_resample = 0

    def weights(self):
        w = np.exp(self.logw - self.logw.max())
        return w / w.sum()

    def update(self, pos, z):
        c = concentration(pos, self.p)
        pd = p_detect(c)
        if z <= 0.0:
            lik = pd * ndtr(-c / SIGMA_NOISE) + (1 - pd) * 0.5
        else:
            phi = lambda u: np.exp(-0.5 * u ** 2) / (np.sqrt(2 * np.pi) * SIGMA_NOISE)
            lik = pd * phi((z - c) / SIGMA_NOISE) + (1 - pd) * phi(z / SIGMA_NOISE)
        self.logw += np.log(np.maximum(lik, 1e-300))
        w = self.weights()
        if 1.0 / np.sum(w ** 2) < N_PARTICLES / 2:
            # systematic resampling (seeded) + roughening
            u = (self.rng.random() + np.arange(N_PARTICLES)) / N_PARTICLES
            idx = np.minimum(np.searchsorted(np.cumsum(w), u), N_PARTICLES - 1)
            self.p = self.p[idx] + self.rng.normal(0, PF_JITTER, size=(N_PARTICLES, 2))
            self.p = np.clip(self.p, 0, AREA)
            self.logw[:] = 0.0
            self.n_resample += 1

    def estimate(self):
        w = self.weights()
        m = w @ self.p
        sd = np.sqrt(np.sum(w * np.sum((self.p - m) ** 2, axis=1)))
        return m, sd


# ── Simulation ─────────────────────────────────────────────────────────────────

def run_one(strategy, src, seed, record=False):
    rng = np.random.default_rng([seed, 1])     # sensor (same seed for every strategy)
    rng_pf = np.random.default_rng([seed, 2])
    pos = POS_START.copy()
    vel = np.zeros(2)
    nav = CastSurgeNavigator()
    pf = ParticleFilter(rng_pf) if (strategy == 'pf_guided') else None
    traj, concs, dropouts, states, pf_est, pf_sd = [pos.copy()], [], [], [], [], []
    pf_snaps, frames = {}, []
    n_steps = int(round(T_MAX / DT))
    found, t = False, 0.0
    for step in range(n_steps):
        t = step * DT
        z, dropped = measure(pos, src, rng)
        concs.append(z)
        dropouts.append(dropped)
        g = gradient_hat(pos, src, rng)
        if pf is not None:
            if record and step in (0, 300, 900, 1800):
                pf_snaps[step] = (pf.p.copy(), pf.weights())
            pf.update(pos, z)
            m, sd = pf.estimate()
            pf_est.append(m)
            pf_sd.append(sd)
        if strategy == 'gradient':
            # card: follow the gradient while in contact, otherwise drift slowly upwind
            v_cmd = V_SURGE * g if z >= C_THRESH else V_DRIFT * UPWIND
            states.append('gradient')
        else:
            v_cmd = nav.command(pos, z, g)
            states.append(nav.state)
            if strategy == 'pf_guided' and pf_sd[-1] < PF_CONF_STD:
                v_cmd = V_SURGE * unit(0.5 * unit(v_cmd) + 0.5 * unit(pf_est[-1] - pos))
        if record and step % max(1, n_steps // 1000) == 0:
            frames.append((t, pos.copy(), None if pf is None else pf.p.copy(),
                           None if pf is None else pf_est[-1].copy(), states[-1]))
        vel = vel + (v_cmd - vel) / TAU_V * DT
        sp = np.linalg.norm(vel)
        if sp > V_MAX:
            vel *= V_MAX / sp
        pos = np.clip(pos + vel * DT, 0.0, AREA)
        traj.append(pos.copy())
        if np.linalg.norm(pos - src) <= R_CAPTURE:
            found, t = True, (step + 1) * DT
            break
    else:
        t = T_MAX
    if pf is not None and record:
        for s in (0, 300, 900, 1800):
            if s not in pf_snaps:
                pf_snaps[s] = (pf.p.copy(), pf.weights())
    traj = np.array(traj)
    concs = np.array(concs)
    return dict(traj=traj, conc=concs, dropouts=np.array(dropouts), states=states,
                pf_est=np.array(pf_est), pf_sd=np.array(pf_sd), pf_snaps=pf_snaps,
                frames=frames, found=found, time=t, src=src,
                path=float(np.sum(np.linalg.norm(np.diff(traj, axis=0), axis=1))),
                n_resample=0 if pf is None else pf.n_resample,
                min_dist=float(np.min(np.linalg.norm(traj - src, axis=1))))


def run_simulation():
    # Plume check along the centreline from the demo source
    print('=== Plume check (demo source, centreline, normalised mg/m^3) ===')
    for d in (10, 30, 60, 115):
        c = float(concentration(DEMO_SOURCE + np.array([d, 0.0]), DEMO_SOURCE))
        print(f'  {d:4d} m downwind: C = {c:7.3f}, sigma_y = {sigma_y(d):5.2f} m, '
              f'p_detect = {p_detect(c):.2f}, SNR = {c / SIGMA_NOISE:6.1f}')
    c_start = float(concentration(POS_START, DEMO_SOURCE))
    print(f'  At the drone start (demo): C = {c_start:.3f} '
          f'({"above" if c_start > C_THRESH else "below"} C_thresh = {C_THRESH})')
    print(f'  Noise-only false contact probability P(eta > C_thresh) = {1 - ndtr(C_THRESH / SIGMA_NOISE):.4f}')

    data = {}
    print('=== Demonstration runs (source at (%.0f, %.0f)) ===' % tuple(DEMO_SOURCE))
    for s in STRATS:
        r = run_one(s, DEMO_SOURCE, DEMO_SEED, record=True)
        data[s] = r
        frac_contact = np.mean(r['conc'] >= C_THRESH) * 100
        print(f"  {LABELS[s]:15s}: {'FOUND' if r['found'] else 'TIMEOUT'} t = {r['time']:.1f} s, "
              f"path = {r['path']:.1f} m, closest approach = {r['min_dist']:.1f} m, "
              f"contact {frac_contact:.1f}% of steps, dropouts {int(r['dropouts'].sum())}")
        if s == 'cast_surge':
            st = r['states']
            print('                   states: ' + ', '.join(
                f'{k} {100 * st.count(k) / len(st):.1f}%' for k in ('surge', 'cast', 'lost')))
        if s == 'pf_guided':
            print(f"                   PF final estimate ({r['pf_est'][-1][0]:.1f}, {r['pf_est'][-1][1]:.1f}), "
                  f"error {np.linalg.norm(r['pf_est'][-1] - DEMO_SOURCE):.1f} m, posterior std "
                  f"{r['pf_sd'][-1]:.1f} m, resamplings {r['n_resample']}")

    print(f'=== Monte Carlo: {N_TRIALS} trials, source ~ U{SRC_X_RANGE} x U{SRC_Y_RANGE} ===')
    mc = {s: {'times': [], 'paths': [], 'found': [], 'min_dist': []} for s in STRATS}
    starts_in_plume = 0
    in_plume = []
    for trial in range(N_TRIALS):
        rs = np.random.default_rng([trial, 0])
        src = np.array([rs.uniform(*SRC_X_RANGE), rs.uniform(*SRC_Y_RANGE)])
        starts_in_plume += concentration(POS_START, src) >= C_THRESH
        in_plume.append(bool(concentration(POS_START, src) >= C_THRESH))
        for s in STRATS:
            r = run_one(s, src, trial)
            mc[s]['times'].append(r['time'])
            mc[s]['paths'].append(r['path'])
            mc[s]['found'].append(r['found'])
            mc[s]['min_dist'].append(r['min_dist'])
    print(f'  Trials whose start point has mean C >= C_thresh: {starts_in_plume}/{N_TRIALS}')
    for s in STRATS:
        f = np.array(mc[s]['found'])
        tt = np.array(mc[s]['times'])
        pp = np.array(mc[s]['paths'])
        md = np.array(mc[s]['min_dist'])
        med_t = np.median(tt[f]) if f.any() else float('nan')
        med_p = np.median(pp[f]) if f.any() else float('nan')
        print(f'  {LABELS[s]:15s}: found {f.sum()}/{N_TRIALS}, median time (found) {med_t:.1f} s, '
              f'median path (found) {med_p:.1f} m, median closest approach (all) {np.median(md):.1f} m')
        ip = np.array(in_plume)
        print(f'  {"":15s}  start in plume: {f[ip].sum()}/{ip.sum()} found; '
              f'start outside plume: {f[~ip].sum()}/{(~ip).sum()} found; '
              f'closest approach of failures: ' + ', '.join(f'{v:.1f}' for v in md[~f]) + ' m')
    data['mc'] = mc
    data['in_plume'] = in_plume

    xs = np.linspace(0.0, AREA, 401)
    XX, YY = np.meshgrid(xs, xs)
    CC = concentration(np.stack([XX, YY], axis=-1), DEMO_SOURCE)
    data['grid'] = (XX, YY, CC)
    return data


# ── Plotting ───────────────────────────────────────────────────────────────────

def _bg(ax, data, alpha=1.0):
    XX, YY, CC = data['grid']
    im = ax.pcolormesh(XX, YY, np.minimum(CC, 20.0), cmap='YlOrRd', shading='auto',
                       norm=mcolors.PowerNorm(gamma=0.4, vmin=0, vmax=20.0), alpha=alpha)
    ax.annotate('', xy=(35, 10), xytext=(10, 10), arrowprops=dict(arrowstyle='->', color='navy', lw=2))
    ax.text(37, 10, 'Wind (+x)', color='navy', va='center', fontsize=9)
    ax.set_xlim(0, AREA)
    ax.set_ylim(0, AREA)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    return im


def plot_concentration_map(data, out_dir):
    fig, ax = plt.subplots(figsize=(8, 7))
    im = _bg(ax, data)
    plt.colorbar(im, ax=ax, label='Mean concentration (mg/m³, clipped at 20)')
    x_dw = np.linspace(0.5, AREA - DEMO_SOURCE[0], 300)
    for n in (1, 2):
        ax.plot(DEMO_SOURCE[0] + x_dw, DEMO_SOURCE[1] + n * sigma_y(x_dw), 'k--', lw=0.8, alpha=0.6)
        ax.plot(DEMO_SOURCE[0] + x_dw, DEMO_SOURCE[1] - n * sigma_y(x_dw), 'k--', lw=0.8, alpha=0.6,
                label=f'±{n}σ_y' if n == 1 else '±2σ_y')
    XX, YY, CC = data['grid']
    ax.contour(XX, YY, CC, levels=[C_THRESH], colors='blue', linewidths=1.0)
    ax.plot([], [], 'b-', label=f'C = C_thresh ({C_THRESH})')
    ax.add_patch(plt.Rectangle((SRC_X_RANGE[0], SRC_Y_RANGE[0]), SRC_X_RANGE[1] - SRC_X_RANGE[0],
                               SRC_Y_RANGE[1] - SRC_Y_RANGE[0], fill=False, ec='purple', ls=':',
                               label='MC source region'))
    ax.plot(*DEMO_SOURCE, 'r*', ms=18, zorder=10, label='True source (demo)')
    ax.plot(*POS_START, 'bs', ms=10, zorder=10, label='Drone start')
    ax.set_title('Gaussian plume (Pasquill-Gifford D, release height 2 m)')
    ax.legend(loc='upper left', fontsize=8)
    path = os.path.join(out_dir, 'concentration_map.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_trajectories(data, out_dir):
    from matplotlib.collections import LineCollection
    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(17, 7.5))
    im = _bg(ax, data, alpha=0.6)
    for s in STRATS:
        r = data[s]
        tr = r['traj']
        ax.plot(tr[:, 0], tr[:, 1], color=COLORS[s], lw=1.4,
                label=f"{LABELS[s]} ({'found' if r['found'] else 'timeout'}, {r['time']:.0f} s, {r['path']:.0f} m)")
        if r['found']:
            ax.plot(*tr[-1], '*', color=COLORS[s], ms=14, mec='k', mew=0.8)
    pe = data['pf_guided']['pf_est']
    ax.plot(pe[:, 0], pe[:, 1], ':', color='darkgreen', lw=1, label='PF posterior mean (PF guided)')
    for a in (ax, ax2):
        a.add_patch(plt.Circle(DEMO_SOURCE, R_CAPTURE, color='red', fill=False, lw=1.5, ls='--'))
        a.plot(*DEMO_SOURCE, 'r*', ms=16, zorder=10)
        a.plot(*POS_START, 'bs', ms=9, zorder=10)
    ax.set_title('Trajectory comparison (demonstration run)')
    ax.legend(loc='upper left', fontsize=8)
    # cast-and-surge path coloured by state
    _bg(ax2, data, alpha=0.4)
    r = data['cast_surge']
    tr = r['traj']
    st_cols = {'surge': 'dodgerblue', 'cast': 'gold', 'lost': 'tomato'}
    segs = np.stack([tr[:-1], tr[1:]], axis=1)[:len(r['states'])]
    lc = LineCollection(segs, colors=[st_cols[x] for x in r['states']], linewidths=2.2)
    ax2.add_collection(lc)
    for k, c in st_cols.items():
        frac = 100 * r['states'].count(k) / len(r['states'])
        ax2.plot([], [], color=c, lw=3, label=f'{k} ({frac:.1f}% of steps)')
    for txt in list(ax2.texts):      # wind label lies outside the zoomed view
        txt.remove()
    ax2.set_xlim(45, 180)
    ax2.set_ylim(85, 125)
    ax2.set_aspect('auto')
    ax2.set_title('Cast-and-surge path coloured by state (zoom on y = 85-125 m)')
    ax2.legend(loc='upper left', fontsize=8)
    plt.colorbar(im, ax=[ax, ax2], label='Mean concentration (mg/m³, clipped at 20)', shrink=0.8)
    path = os.path.join(out_dir, 'trajectories.png')
    plt.savefig(path, dpi=130, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_concentration_timeseries(data, out_dir):
    fig, axes = plt.subplots(3, 1, figsize=(11, 8))
    for ax, s in zip(axes, STRATS):
        r = data[s]
        c = r['conc']
        t = np.arange(len(c)) * DT
        ax.plot(t, c, color=COLORS[s] if s != 'pf_guided' else 'green', lw=0.6, label=LABELS[s])
        d = r['dropouts']
        ax.plot(t[d], np.zeros(d.sum()), '|', color='k', ms=6, label=f'Intermittency dropouts ({d.sum()})')
        ax.axhline(C_THRESH, color='red', lw=1.0, ls='--', label=f'C_thresh = {C_THRESH}')
        if r['found']:
            ax.axvline(r['time'], color='green', lw=1.5, ls=':', label='Source found')
        ax.set_yscale('symlog', linthresh=0.1)
        ax.set_ylim(0, 2000)
        ax.set_ylabel('C_meas (mg/m³)')
        ax.set_title(LABELS[s], fontsize=10)
        ax.legend(loc='upper left', fontsize=7)
    axes[-1].set_xlabel('Time (s)')
    fig.suptitle('Measured concentration (symlog scale) with intermittency dropouts', fontsize=11)
    plt.tight_layout()
    path = os.path.join(out_dir, 'concentration_timeseries.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_pf_snapshots(data, out_dir):
    r = data['pf_guided']
    fig, axes = plt.subplots(2, 2, figsize=(11, 10))
    for ax, step in zip(axes.flatten(), sorted(r['pf_snaps'])):
        parts, w = r['pf_snaps'][step]
        _bg(ax, data, alpha=0.4)
        ax.scatter(parts[:, 0], parts[:, 1], s=np.clip(5 + 2000 * w, 3, 60), c='teal', alpha=0.5,
                   linewidths=0, label='Particles')
        k = min(step, len(r['traj']) - 1)
        ax.plot(*r['traj'][k], 'bo', ms=7, label='Drone')
        ax.plot(*DEMO_SOURCE, 'r*', ms=15, label='True source')
        t_s = min(step * DT, r['time'])
        ax.set_title(f't = {step * DT:.0f} s' + ('' if step * DT <= r['time'] else f' (run ended at {r["time"]:.0f} s)'))
        ax.legend(loc='upper left', fontsize=7)
    fig.suptitle('Particle filter posterior (PF-guided demonstration run, weights before the update)')
    plt.tight_layout()
    path = os.path.join(out_dir, 'pf_snapshots.png')
    plt.savefig(path, dpi=130, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_state_timeline(data, out_dir):
    r = data['cast_surge']
    smap = {'lost': 0, 'cast': 1, 'surge': 2}
    arr = np.array([smap[s] for s in r['states']])
    t = np.arange(len(arr)) * DT
    fig, ax = plt.subplots(figsize=(12, 2))
    cmap = mcolors.ListedColormap(['tomato', 'gold', 'dodgerblue'])
    ax.imshow(arr[np.newaxis, :], aspect='auto', cmap=cmap,
              norm=mcolors.BoundaryNorm([-0.5, 0.5, 1.5, 2.5], 3), extent=[0, t[-1] + DT, 0, 1])
    ax.set_yticks([])
    ax.set_xlabel('Time (s)')
    ax.set_title('Cast-and-surge state timeline (demonstration run)')
    ax.legend(handles=[mpatches.Patch(color='tomato', label='Lost'),
                       mpatches.Patch(color='gold', label='Cast'),
                       mpatches.Patch(color='dodgerblue', label='Surge')], loc='upper right', fontsize=8)
    plt.tight_layout()
    path = os.path.join(out_dir, 'state_timeline.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_performance_summary(data, out_dir):
    mc = data['mc']
    fig, (a1, a2, a3) = plt.subplots(1, 3, figsize=(15, 5))
    labels = [LABELS[s].replace(' ', '\n', 1) for s in STRATS]
    for ax, key, ttl in [(a1, 'times', 'Time to find (s)'), (a2, 'paths', 'Path length (m)')]:
        vals = []
        for s in STRATS:
            f = np.array(mc[s]['found'])
            v = np.array(mc[s][key])[f]
            vals.append(v if len(v) else [np.nan])
        bp = ax.boxplot(vals, patch_artist=True)
        for patch, s in zip(bp['boxes'], STRATS):
            patch.set_facecolor(COLORS[s])
            patch.set_alpha(0.7)
        ax.set_xticks([1, 2, 3])
        ax.set_xticklabels(labels)
        ax.set_ylabel(ttl)
        ax.set_title(f'{ttl} — successful runs only')
    rates = [np.mean(mc[s]['found']) * 100 for s in STRATS]
    a3.bar(range(3), rates, color=[COLORS[s] for s in STRATS], alpha=0.8)
    for i, rr in enumerate(rates):
        a3.text(i, rr + 1, f'{int(round(rr * N_TRIALS / 100))}/{N_TRIALS}', ha='center')
    a3.set_xticks(range(3))
    a3.set_xticklabels(labels)
    a3.set_ylim(0, 110)
    a3.set_ylabel('Success rate (%)')
    a3.set_title(f'Source found within {T_MAX:.0f} s')
    fig.suptitle(f'Monte Carlo comparison ({N_TRIALS} random source positions, same for every strategy)')
    plt.tight_layout()
    path = os.path.join(out_dir, 'performance_summary.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def save_animation(data, out_dir):
    """All three demo runs on one map; PF particle cloud from the PF-guided run."""
    import matplotlib.animation as animation
    fig, ax = plt.subplots(figsize=(8, 7))
    _bg(ax, data, alpha=0.55)
    ax.plot(*DEMO_SOURCE, 'r*', ms=16, zorder=10)
    ax.add_patch(plt.Circle(DEMO_SOURCE, R_CAPTURE, color='red', fill=False, lw=1.2, ls='--'))
    lines = {s: ax.plot([], [], color=COLORS[s], lw=1.2, label=LABELS[s])[0] for s in STRATS}
    dots = {s: ax.plot([], [], 'o', color=COLORS[s], mec='k', ms=7, zorder=12)[0] for s in STRATS}
    scat = ax.scatter([], [], s=4, c='teal', alpha=0.4, linewidths=0)
    est, = ax.plot([], [], 'X', color='darkgreen', ms=9, zorder=11, label='PF estimate')
    txt = ax.text(5, AREA - 12, '', fontsize=9, bbox=dict(boxstyle='round', fc='white', alpha=0.7))
    ax.legend(loc='lower left', fontsize=8)
    t_end = max(data[s]['time'] for s in STRATS)
    times = np.linspace(0, t_end, N_ANIM_FRAMES)

    def update(i):
        tf = times[i]
        k_all = int(round(tf / DT))
        info = [f't = {tf:.0f} s']
        for s in STRATS:
            tr = data[s]['traj']
            k = min(k_all, len(tr) - 1)
            lines[s].set_data(tr[:k + 1, 0], tr[:k + 1, 1])
            dots[s].set_data([tr[k, 0]], [tr[k, 1]])
        fr = data['pf_guided']['frames']
        j = min(range(len(fr)), key=lambda q: abs(fr[q][0] - tf))
        scat.set_offsets(fr[j][2])
        est.set_data([fr[j][3][0]], [fr[j][3][1]])
        cs = data['cast_surge']['states']
        info.append(f"cast-surge: {cs[min(k_all, len(cs) - 1)]}")
        txt.set_text('   '.join(info))
        return []

    ani = animation.FuncAnimation(fig, update, frames=N_ANIM_FRAMES, interval=80, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer='pillow', fps=12, dpi=80)
    plt.close()
    print(f'Saved: {path}  ({N_ANIM_FRAMES} frames)')


# ── Entry point ────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    data = run_simulation()
    plot_concentration_map(data, OUTPUT_DIR)
    plot_trajectories(data, OUTPUT_DIR)
    plot_concentration_timeseries(data, OUTPUT_DIR)
    plot_pf_snapshots(data, OUTPUT_DIR)
    plot_state_timeline(data, OUTPUT_DIR)
    plot_performance_summary(data, OUTPUT_DIR)
    save_animation(data, OUTPUT_DIR)
    print('Done.')
