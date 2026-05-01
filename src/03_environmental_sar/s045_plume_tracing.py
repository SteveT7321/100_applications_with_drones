"""
S045 Chemical Plume Tracing
============================
Simulates a single drone tracing a chemical plume back to its source using
three strategies: pure gradient ascent, cast-and-surge chemotaxis, and
particle-filter-guided navigation. The plume disperses from a fixed point
source under steady wind according to Gaussian (Pasquill-Gifford class D)
dispersion theory with turbulent intermittency. A particle filter maintains
a posterior distribution over the unknown source location and is updated
with each noisy concentration measurement.

Usage:
    conda activate drones
    python src/03_environmental_sar/s045_plume_tracing.py
"""

import sys
import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.colors as mcolors
from matplotlib.animation import FuncAnimation

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

# ── Parameters ─────────────────────────────────────────────────────────────────
WIND_SPEED   = 3.0      # m/s, along +x axis
Q_SOURCE     = 1.0      # g/s, emission rate
Z_DRONE      = 2.0      # m, flight altitude
SIGMA_NOISE  = 0.05     # normalised concentration noise std
C_THRESH     = 0.02     # detection threshold (normalised units)
V_SURGE      = 1.5      # m/s, surge speed
V_CAST       = 0.8      # m/s, cast speed
L_CAST       = 8.0      # m, cast leg length before reversing direction
T_CAST_MAX   = 30.0     # s, max cast duration before declaring lost
ALPHA_UPWIND = 0.4      # upwind bias weight in surge command
GRAD_DELTA   = 1.0      # m, finite-difference probe offset
N_PARTICLES  = 500      # particle filter population
R_CAPTURE    = 3.0      # m, source-found radius
V_MAX        = 2.0      # m/s, max drone speed
TAU_V        = 0.5      # s, first-order speed response time constant
DT           = 0.1      # s, simulation timestep
T_MAX        = 600.0    # s, mission timeout
AREA         = 200.0    # m, arena side length
N_TRIALS     = 20       # Monte Carlo runs for performance comparison

POS_SOURCE = np.array([140.0, 95.0])   # true source (unknown to drone)
POS_START  = np.array([20.0, 80.0])    # drone start (downwind, low x)

OUTPUT_DIR = os.path.normpath(os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '03_environmental_sar', 's045_plume_tracing',
))

RNG = np.random.default_rng(0)

# ── Physics helpers ─────────────────────────────────────────────────────────────

def sigma_y(x):
    """Lateral dispersion coefficient — Pasquill-Gifford class D."""
    x = max(float(x), 1e-3)
    return 0.08 * x / np.sqrt(1.0 + 0.0002 * x)


def sigma_z(x):
    """Vertical dispersion coefficient — Pasquill-Gifford class D."""
    x = max(float(x), 1e-3)
    return 0.06 * x / np.sqrt(1.0 + 0.0015 * x)


def gaussian_concentration(pos_drone, pos_source):
    """Time-averaged 2-D Gaussian plume concentration (top-down view).

    Returns zero upwind of the source (dx <= 0).
    """
    dx = float(pos_drone[0] - pos_source[0])   # downwind offset
    dy = float(pos_drone[1] - pos_source[1])   # crosswind offset
    if dx <= 0.0:
        return 0.0
    sy = sigma_y(dx)
    sz = sigma_z(dx)
    q_eff = Q_SOURCE * np.exp(-Z_DRONE**2 / (2.0 * sz**2)) / sz
    return (q_eff / (2.0 * np.pi * sy * WIND_SPEED)) * np.exp(-dy**2 / (2.0 * sy**2))


def measure_concentration(pos_drone, pos_source, rng):
    """Noisy, turbulence-intermittent concentration reading at pos_drone."""
    c_mean = gaussian_concentration(pos_drone, pos_source)
    p_detect = 1.0 - np.exp(-c_mean / max(C_THRESH, 1e-9))
    detected = rng.random() < p_detect
    noise = rng.normal(0.0, SIGMA_NOISE)
    return max(0.0, c_mean * float(detected) + noise)


def estimate_gradient(pos_drone, pos_source, rng):
    """Finite-difference gradient estimate from four noisy measurements."""
    dx_vec = np.array([GRAD_DELTA, 0.0])
    dy_vec = np.array([0.0, GRAD_DELTA])
    c_xp = measure_concentration(pos_drone + dx_vec, pos_source, rng)
    c_xm = measure_concentration(pos_drone - dx_vec, pos_source, rng)
    c_yp = measure_concentration(pos_drone + dy_vec, pos_source, rng)
    c_ym = measure_concentration(pos_drone - dy_vec, pos_source, rng)
    grad = np.array([(c_xp - c_xm) / (2.0 * GRAD_DELTA),
                     (c_yp - c_ym) / (2.0 * GRAD_DELTA)])
    norm = np.linalg.norm(grad) + 1e-9
    return grad / norm


# ── Navigation strategies ───────────────────────────────────────────────────────

class CastSurgeNavigator:
    """Three-state machine: surge (in-plume) → cast (searching) → lost (return)."""

    def __init__(self):
        self.state = 'surge'
        self.cast_dir = 1.0           # +1 = +y side, -1 = -y side
        self.cast_dist_accum = 0.0
        self.t_no_contact = 0.0
        self.last_contact_pos = None

    def command(self, pos, concentration, grad_hat, dt):
        """Return velocity command vector (not yet speed-limited)."""
        upwind = np.array([1.0, 0.0])

        if concentration >= C_THRESH:
            self.state = 'surge'
            self.t_no_contact = 0.0
            self.cast_dist_accum = 0.0
            self.last_contact_pos = pos.copy()
            v_cmd = V_SURGE * (ALPHA_UPWIND * upwind + (1.0 - ALPHA_UPWIND) * grad_hat)
            return v_cmd / (np.linalg.norm(v_cmd) + 1e-9) * V_SURGE

        self.t_no_contact += dt

        if self.t_no_contact > T_CAST_MAX and self.last_contact_pos is not None:
            self.state = 'lost'
            toward_last = self.last_contact_pos - pos
            d = np.linalg.norm(toward_last)
            if d < 1.0:
                self.t_no_contact = 0.0
                self.state = 'cast'
            return V_CAST * toward_last / (d + 1e-9)

        self.state = 'cast'
        cast_vec = np.array([0.0, self.cast_dir])
        self.cast_dist_accum += V_CAST * dt
        if self.cast_dist_accum >= L_CAST:
            self.cast_dir *= -1.0
            self.cast_dist_accum = 0.0
        return V_CAST * cast_vec + 0.2 * upwind


class ParticleFilter:
    """Source-location particle filter over the 2-D search area."""

    def __init__(self, bounds, n_particles, rng):
        self.n = n_particles
        self.particles = rng.uniform(
            [bounds[0], bounds[2]], [bounds[1], bounds[3]],
            size=(n_particles, 2),
        )
        self.weights = np.ones(n_particles) / n_particles

    def update(self, pos_drone, c_meas):
        for i, ps in enumerate(self.particles):
            c_pred = gaussian_concentration(pos_drone, ps)
            self.weights[i] *= np.exp(-(c_meas - c_pred)**2 / (2.0 * SIGMA_NOISE**2))
        w_sum = self.weights.sum()
        if w_sum < 1e-300:
            self.weights[:] = 1.0 / self.n
        else:
            self.weights /= w_sum
        ess = 1.0 / (np.sum(self.weights**2) + 1e-300)
        if ess < self.n / 2.0:
            idx = np.searchsorted(np.cumsum(self.weights),
                                  np.random.default_rng().random(self.n))
            idx = np.clip(idx, 0, self.n - 1)
            self.particles = self.particles[idx]
            self.weights[:] = 1.0 / self.n

    def estimate(self):
        return np.average(self.particles, weights=self.weights, axis=0)

    def snapshot(self):
        return self.particles.copy(), self.weights.copy()


# ── Simulation core ─────────────────────────────────────────────────────────────

def _run_one(strategy, rng, pos_source=None, pos_start=None):
    """Run a single simulation episode.  Returns result dict."""
    if pos_source is None:
        pos_source = POS_SOURCE.copy()
    if pos_start is None:
        pos_start = POS_START.copy()

    bounds = [0.0, AREA, 0.0, AREA]
    pos = pos_start.copy().astype(float)
    vel = np.zeros(2)

    navigator = CastSurgeNavigator()
    pf = ParticleFilter(bounds, N_PARTICLES, rng)

    trajectory  = [pos.copy()]
    concentrations = []
    pf_estimates   = []
    states         = []
    pf_snapshots   = {}   # {step: (particles, weights)}
    snapshot_steps = {0, 300, 900, 1800}   # roughly t=0,30,90,180 s at DT=0.1

    t    = 0.0
    step = 0
    found = False

    while t < T_MAX:
        if step in snapshot_steps:
            pf_snapshots[step] = pf.snapshot()

        c_meas = measure_concentration(pos, pos_source, rng)
        concentrations.append(float(c_meas))
        pf.update(pos, c_meas)
        pf_est = pf.estimate()
        pf_estimates.append(pf_est.copy())

        if strategy == 'gradient':
            grad_hat = estimate_gradient(pos, pos_source, rng)
            if c_meas >= C_THRESH:
                v_cmd = V_SURGE * grad_hat
            else:
                v_cmd = np.array([0.2, 0.0])   # slow upwind drift when lost

        elif strategy == 'cast_surge':
            grad_hat = estimate_gradient(pos, pos_source, rng)
            v_cmd = navigator.command(pos, c_meas, grad_hat, DT)
            states.append(navigator.state)

        elif strategy == 'pf_guided':
            grad_hat = estimate_gradient(pos, pos_source, rng)
            toward_est = pf_est - pos
            d_est = np.linalg.norm(toward_est) + 1e-9
            nav_cmd = navigator.command(pos, c_meas, grad_hat, DT)
            v_cmd = 0.5 * nav_cmd + 0.5 * V_SURGE * toward_est / d_est

        else:
            raise ValueError(f'Unknown strategy: {strategy}')

        # First-order speed response
        vel = vel + (v_cmd - vel) / TAU_V * DT
        speed = np.linalg.norm(vel)
        if speed > V_MAX:
            vel = vel / speed * V_MAX

        pos = pos + vel * DT
        pos = np.clip(pos, [bounds[0], bounds[2]], [bounds[1], bounds[3]])
        trajectory.append(pos.copy())
        t    += DT
        step += 1

        if np.linalg.norm(pos - pos_source) <= R_CAPTURE:
            found = True
            break

    # capture final snapshot
    final_step = step
    for s in snapshot_steps:
        if s > final_step and s not in pf_snapshots:
            pf_snapshots[s] = pf.snapshot()

    path_len = float(np.sum(np.linalg.norm(np.diff(np.array(trajectory), axis=0), axis=1)))

    return {
        'trajectory':     np.array(trajectory),
        'concentrations': np.array(concentrations),
        'pf_estimates':   np.array(pf_estimates),
        'pos_source':     pos_source,
        'found':          found,
        'time':           t,
        'path_length':    path_len,
        'states':         states,
        'pf_snapshots':   pf_snapshots,
    }


def run_simulation():
    """Run all three strategies + Monte Carlo comparison. Return data bundle."""
    rng = np.random.default_rng(0)

    print('Running cast-and-surge ...')
    cs_data = _run_one('cast_surge', rng)

    rng2 = np.random.default_rng(1)
    print('Running gradient ascent ...')
    gr_data = _run_one('gradient', rng2)

    rng3 = np.random.default_rng(2)
    print('Running particle-filter guided ...')
    pf_data = _run_one('pf_guided', rng3)

    # ── Monte Carlo comparison ──────────────────────────────────────────────
    print(f'Running Monte Carlo ({N_TRIALS} trials per strategy) ...')
    mc_results = {s: {'times': [], 'paths': [], 'found': []}
                  for s in ('gradient', 'cast_surge', 'pf_guided')}

    for trial in range(N_TRIALS):
        for strat in ('gradient', 'cast_surge', 'pf_guided'):
            seed = 1000 + trial * 3 + ['gradient', 'cast_surge', 'pf_guided'].index(strat)
            r = _run_one(strat, np.random.default_rng(seed))
            mc_results[strat]['times'].append(r['time'])
            mc_results[strat]['paths'].append(r['path_length'])
            mc_results[strat]['found'].append(r['found'])

    # ── Build concentration map grid ────────────────────────────────────────
    xs = np.linspace(0.0, AREA, 200)
    ys = np.linspace(0.0, AREA, 200)
    XX, YY = np.meshgrid(xs, ys)
    CC = np.zeros_like(XX)
    for i in range(XX.shape[0]):
        for j in range(XX.shape[1]):
            CC[i, j] = gaussian_concentration(
                np.array([XX[i, j], YY[i, j]]), POS_SOURCE
            )

    return {
        'cast_surge': cs_data,
        'gradient':   gr_data,
        'pf_guided':  pf_data,
        'mc':         mc_results,
        'grid':       (XX, YY, CC),
    }


# ── Plotting ────────────────────────────────────────────────────────────────────

def plot_concentration_map(data, out_dir):
    """Heatmap of Gaussian plume with plume boundary contours."""
    XX, YY, CC = data['grid']
    pos_source = POS_SOURCE

    fig, ax = plt.subplots(figsize=(8, 7))
    im = ax.pcolormesh(XX, YY, CC, cmap='YlOrRd', shading='auto',
                       norm=mcolors.PowerNorm(gamma=0.4, vmin=0, vmax=CC.max()))
    plt.colorbar(im, ax=ax, label='Concentration (normalised g/m²)')

    # Plume boundary contours (sigma_y lines)
    x_line = np.linspace(1.0, AREA, 300)
    for n_sig in (1, 2):
        sy_vals = np.array([sigma_y(pos_source[0] - x) if x < pos_source[0] else
                            sigma_y(x - pos_source[0]) for x in x_line])
        # actual plume spreads downwind (x > source x from drone perspective)
        x_dw = np.linspace(pos_source[0], AREA, 200)
        sy_dw = np.array([sigma_y(xi - pos_source[0]) for xi in x_dw])
        ax.plot(x_dw, pos_source[1] + n_sig * sy_dw, 'w--', lw=0.8, alpha=0.7)
        ax.plot(x_dw, pos_source[1] - n_sig * sy_dw, 'w--', lw=0.8, alpha=0.7)

    ax.plot(*pos_source, 'r*', ms=18, zorder=10, label='True source')
    ax.plot(*POS_START, 'bs', ms=10, zorder=10, label='Drone start')
    ax.annotate('', xy=(30, 10), xytext=(5, 10),
                arrowprops=dict(arrowstyle='->', color='cyan', lw=2))
    ax.text(32, 10, 'Wind', color='cyan', va='center', fontsize=9)
    ax.set_xlim(0, AREA)
    ax.set_ylim(0, AREA)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title('Gaussian Plume Concentration Map (Pasquill-Gifford Class D)')
    ax.legend(loc='upper left')

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'concentration_map.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_trajectories(data, out_dir):
    """Drone paths for all three strategies overlaid on the concentration map."""
    XX, YY, CC = data['grid']

    fig, ax = plt.subplots(figsize=(9, 8))
    im = ax.pcolormesh(XX, YY, CC, cmap='YlOrRd', shading='auto',
                       norm=mcolors.PowerNorm(gamma=0.4, vmin=0, vmax=CC.max()),
                       alpha=0.6)
    plt.colorbar(im, ax=ax, label='Concentration (normalised)')

    styles = {
        'gradient':   ('orange',  'Gradient Ascent'),
        'cast_surge': ('deepskyblue', 'Cast-and-Surge'),
        'pf_guided':  ('lime',    'PF Guided'),
    }

    for strat, (color, label) in styles.items():
        traj = data[strat]['trajectory']
        found = data[strat]['found']
        t_val = data[strat]['time']
        ax.plot(traj[:, 0], traj[:, 1], color=color, lw=1.4, alpha=0.85,
                label=f'{label} ({"found" if found else "timeout"}, {t_val:.0f}s)')
        ax.plot(*traj[0], 'o', color=color, ms=6)
        if found:
            ax.plot(*traj[-1], '*', color=color, ms=14, markeredgecolor='white', mew=0.8)

    # Capture circle
    circle = plt.Circle(POS_SOURCE, R_CAPTURE, color='red', fill=False,
                         lw=1.5, linestyle='--', label=f'Capture radius ({R_CAPTURE}m)')
    ax.add_patch(circle)
    ax.plot(*POS_SOURCE, 'r*', ms=18, zorder=10)
    ax.plot(*POS_START, 'bs', ms=9, zorder=10)

    ax.annotate('', xy=(30, 8), xytext=(5, 8),
                arrowprops=dict(arrowstyle='->', color='white', lw=1.5))
    ax.text(32, 8, 'Wind', color='white', fontsize=8, va='center')
    ax.set_xlim(0, AREA)
    ax.set_ylim(0, AREA)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title('Trajectory Comparison — Three Navigation Strategies')
    ax.legend(loc='upper left', fontsize=8)

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'trajectories.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_concentration_timeseries(data, out_dir):
    """Measured C_meas(t) over time for each strategy."""
    fig, axes = plt.subplots(3, 1, figsize=(11, 7), sharex=False)

    styles = [
        ('gradient',   'Gradient Ascent',    'orange'),
        ('cast_surge', 'Cast-and-Surge',      'deepskyblue'),
        ('pf_guided',  'PF Guided',           'lime'),
    ]

    for ax, (strat, label, color) in zip(axes, styles):
        conc = data[strat]['concentrations']
        t_arr = np.arange(len(conc)) * DT
        ax.plot(t_arr, conc, color=color, lw=0.8, alpha=0.9, label=label)
        ax.axhline(C_THRESH, color='red', lw=1.2, linestyle='--', label='C_thresh')

        # Shade dropout regions (below threshold)
        below = conc < C_THRESH
        # Group consecutive below-threshold indices
        in_drop = False
        drop_start = 0
        for i, b in enumerate(below):
            if b and not in_drop:
                drop_start = i
                in_drop = True
            elif not b and in_drop:
                ax.axvspan(t_arr[drop_start], t_arr[i - 1],
                           alpha=0.15, color='gray')
                in_drop = False
        if in_drop:
            ax.axvspan(t_arr[drop_start], t_arr[-1], alpha=0.15, color='gray')

        if data[strat]['found']:
            ax.axvline(data[strat]['time'], color='green', lw=1.5,
                       linestyle=':', label='Source found')
        ax.set_ylabel('C_meas')
        ax.set_title(label)
        ax.legend(loc='upper right', fontsize=7)
        ax.set_ylim(bottom=0)

    axes[-1].set_xlabel('Time (s)')
    fig.suptitle('Concentration Time Series — Turbulent Intermittency Visible',
                 fontsize=11)
    plt.tight_layout()

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'concentration_timeseries.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_pf_snapshots(data, out_dir):
    """Four snapshots of the PF particle cloud showing convergence."""
    pf_snaps = data['cast_surge']['pf_snapshots']
    XX, YY, CC = data['grid']

    snap_steps = sorted(pf_snaps.keys())[:4]
    times_s = [s * DT for s in snap_steps]

    fig, axes = plt.subplots(2, 2, figsize=(11, 9))
    axes = axes.flatten()

    for ax, step_idx, t_s in zip(axes, snap_steps, times_s):
        parts, wts = pf_snaps[step_idx]
        ax.pcolormesh(XX, YY, CC, cmap='YlOrRd', shading='auto',
                      norm=mcolors.PowerNorm(gamma=0.4, vmin=0, vmax=CC.max()),
                      alpha=0.5)
        # Particle size scaled by weight
        sc_size = 20 + 4000 * wts
        sc_size = np.clip(sc_size, 2, 80)
        ax.scatter(parts[:, 0], parts[:, 1], s=sc_size, c='cyan',
                   alpha=0.5, linewidths=0, label='Particles')
        ax.plot(*POS_SOURCE, 'r*', ms=16, label='True source', zorder=10)
        # PF estimate at this snapshot
        pf_est = data['cast_surge']['pf_estimates']
        est_idx = min(step_idx, len(pf_est) - 1)
        if est_idx >= 0:
            ax.plot(*pf_est[est_idx], 'yo', ms=9, zorder=9,
                    label=f'PF estimate')
        ax.set_xlim(0, AREA)
        ax.set_ylim(0, AREA)
        ax.set_title(f't = {t_s:.0f} s')
        ax.set_xlabel('x (m)')
        ax.set_ylabel('y (m)')
        ax.legend(loc='upper left', fontsize=7)

    fig.suptitle('Particle Filter Posterior Convergence (Cast-and-Surge Strategy)',
                 fontsize=11)
    plt.tight_layout()

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'pf_snapshots.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_state_timeline(data, out_dir):
    """Cast-and-surge state machine timeline (colour bar per timestep)."""
    states = data['cast_surge']['states']
    if not states:
        return

    state_map = {'surge': 2, 'cast': 1, 'lost': 0}
    state_arr = np.array([state_map.get(s, 1) for s in states])
    t_arr = np.arange(len(state_arr)) * DT

    fig, ax = plt.subplots(figsize=(12, 2))
    cmap_sm = mcolors.ListedColormap(['tomato', 'gold', 'dodgerblue'])
    bounds_sm = [-0.5, 0.5, 1.5, 2.5]
    norm_sm = mcolors.BoundaryNorm(bounds_sm, cmap_sm.N)

    ax.imshow(state_arr[np.newaxis, :], aspect='auto', cmap=cmap_sm, norm=norm_sm,
              extent=[t_arr[0], t_arr[-1], 0, 1])
    ax.set_yticks([])
    ax.set_xlabel('Time (s)')
    ax.set_title('Cast-and-Surge State Machine Timeline')

    patches = [mpatches.Patch(color='tomato',     label='Lost'),
               mpatches.Patch(color='gold',        label='Cast'),
               mpatches.Patch(color='dodgerblue',  label='Surge')]
    ax.legend(handles=patches, loc='upper right', fontsize=8)

    if data['cast_surge']['found']:
        ax.axvline(data['cast_surge']['time'], color='green', lw=2, label='Found')

    plt.tight_layout()

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'state_timeline.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_performance_summary(data, out_dir):
    """Box-and-whisker Monte Carlo performance comparison."""
    mc = data['mc']
    strategies = ['gradient', 'cast_surge', 'pf_guided']
    labels = ['Gradient\nAscent', 'Cast-and-\nSurge', 'PF\nGuided']
    colors = ['orange', 'deepskyblue', 'lime']

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 5))

    # Time-to-find (only successful runs)
    times_data, paths_data, fail_rates = [], [], []
    for strat in strategies:
        found_mask = np.array(mc[strat]['found'])
        times_s = np.array(mc[strat]['times'])
        paths_s = np.array(mc[strat]['paths'])
        successful_t = times_s[found_mask]
        successful_p = paths_s[found_mask]
        times_data.append(successful_t if len(successful_t) > 0 else [T_MAX])
        paths_data.append(successful_p if len(successful_p) > 0 else [0.0])
        fail_rate = 1.0 - found_mask.mean()
        fail_rates.append(fail_rate)

    bp1 = ax1.boxplot(times_data, patch_artist=True, notch=False)
    for patch, color in zip(bp1['boxes'], colors):
        patch.set_facecolor(color)
        patch.set_alpha(0.7)
    ax1.set_xticks([1, 2, 3])
    ax1.set_xticklabels(labels)
    ax1.set_ylabel('Time to Find Source (s)')
    ax1.set_title(f'Time to Find (N={N_TRIALS} trials, successful runs only)')
    for i, fr in enumerate(fail_rates):
        ax1.text(i + 1, ax1.get_ylim()[1] * 0.95, f'Fail: {fr*100:.0f}%',
                 ha='center', fontsize=8, color='red')

    bp2 = ax2.boxplot(paths_data, patch_artist=True, notch=False)
    for patch, color in zip(bp2['boxes'], colors):
        patch.set_facecolor(color)
        patch.set_alpha(0.7)
    ax2.set_xticks([1, 2, 3])
    ax2.set_xticklabels(labels)
    ax2.set_ylabel('Path Length (m)')
    ax2.set_title(f'Path Length (N={N_TRIALS} trials, successful runs only)')

    plt.suptitle('Monte Carlo Performance Comparison', fontsize=12)
    plt.tight_layout()

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'performance_summary.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def save_animation(data, out_dir):
    """Animated drone traversing the plume field with PF particle cloud."""
    import matplotlib.animation as animation

    XX, YY, CC = data['grid']
    cs = data['cast_surge']
    traj     = cs['trajectory']
    concs    = cs['concentrations']
    pf_ests  = cs['pf_estimates']
    states   = cs['states']

    # Decimate to keep GIF manageable
    step_skip = 5
    n_frames = len(traj) // step_skip

    fig, ax = plt.subplots(figsize=(8, 7))
    ax.pcolormesh(XX, YY, CC, cmap='YlOrRd', shading='auto',
                  norm=mcolors.PowerNorm(gamma=0.4, vmin=0, vmax=CC.max()),
                  alpha=0.55)
    ax.plot(*POS_SOURCE, 'r*', ms=16, zorder=10)
    ax.plot(*POS_START,  'bs', ms=9,  zorder=10)
    capture_circ = plt.Circle(POS_SOURCE, R_CAPTURE, color='red',
                               fill=False, lw=1.5, linestyle='--')
    ax.add_patch(capture_circ)
    ax.annotate('', xy=(30, 8), xytext=(5, 8),
                arrowprops=dict(arrowstyle='->', color='white', lw=1.5))
    ax.text(32, 8, 'Wind', color='white', fontsize=8, va='center')
    ax.set_xlim(0, AREA)
    ax.set_ylim(0, AREA)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')

    path_line, = ax.plot([], [], 'deepskyblue', lw=1.2, alpha=0.7)
    drone_dot, = ax.plot([], [], 'wo', ms=8, zorder=12)
    pf_scat = ax.scatter([], [], s=4, c='cyan', alpha=0.4, linewidths=0)
    pf_est_dot, = ax.plot([], [], 'yo', ms=8, zorder=11)
    state_text = ax.text(5, AREA - 10, '', color='white', fontsize=10,
                         bbox=dict(boxstyle='round', facecolor='black', alpha=0.5))
    time_text = ax.text(5, AREA - 20, '', color='white', fontsize=9)
    ax.set_title('Cast-and-Surge Plume Tracing with Particle Filter')

    # Snapshot particles for animation (use nearest stored snapshot)
    snap_steps = sorted(cs['pf_snapshots'].keys())

    def _get_snapshot(frame_step):
        best = snap_steps[0]
        for s in snap_steps:
            if s <= frame_step:
                best = s
        return cs['pf_snapshots'][best]

    def init():
        path_line.set_data([], [])
        drone_dot.set_data([], [])
        pf_scat.set_offsets(np.empty((0, 2)))
        pf_est_dot.set_data([], [])
        state_text.set_text('')
        time_text.set_text('')
        return path_line, drone_dot, pf_scat, pf_est_dot, state_text, time_text

    def update(frame):
        idx = frame * step_skip
        idx = min(idx, len(traj) - 1)

        path_line.set_data(traj[:idx + 1, 0], traj[:idx + 1, 1])
        drone_dot.set_data([traj[idx, 0]], [traj[idx, 1]])

        parts, wts = _get_snapshot(idx)
        pf_scat.set_offsets(parts)

        if idx < len(pf_ests):
            pf_est_dot.set_data([pf_ests[idx, 0]], [pf_ests[idx, 1]])

        st = states[min(idx, len(states) - 1)] if states else ''
        state_text.set_text(f'State: {st.upper()}')
        time_text.set_text(f't = {idx * DT:.1f} s')
        return path_line, drone_dot, pf_scat, pf_est_dot, state_text, time_text

    ani = animation.FuncAnimation(fig, update, frames=n_frames,
                                   init_func=init, blit=True, interval=80)

    os.makedirs(out_dir, exist_ok=True)
    gif_path = os.path.join(out_dir, 'animation.gif')
    ani.save(gif_path, writer='pillow', fps=12, dpi=100)
    plt.close()
    print(f'Saved: {gif_path}')


# ── Entry point ─────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    data = run_simulation()

    # ── Print key metrics ───────────────────────────────────────────────────
    for strat, label in [('gradient', 'Gradient Ascent'),
                         ('cast_surge', 'Cast-and-Surge'),
                         ('pf_guided', 'PF Guided')]:
        d = data[strat]
        status = 'FOUND' if d['found'] else 'TIMEOUT'
        print(f'{label}: {status}  time={d["time"]:.1f}s  '
              f'path={d["path_length"]:.1f}m')

    mc = data['mc']
    for strat, label in [('gradient', 'Gradient Ascent'),
                         ('cast_surge', 'Cast-and-Surge'),
                         ('pf_guided', 'PF Guided')]:
        found_arr = np.array(mc[strat]['found'])
        times_arr = np.array(mc[strat]['times'])
        succ_times = times_arr[found_arr]
        fail_rate = 1.0 - found_arr.mean()
        med_t = float(np.median(succ_times)) if len(succ_times) > 0 else float('nan')
        print(f'MC {label}: fail={fail_rate*100:.0f}%  '
              f'median_time={med_t:.1f}s  '
              f'n_found={found_arr.sum()}/{N_TRIALS}')

    out_dir = OUTPUT_DIR
    plot_concentration_map(data, out_dir)
    plot_trajectories(data, out_dir)
    plot_concentration_timeseries(data, out_dir)
    plot_pf_snapshots(data, out_dir)
    plot_state_timeline(data, out_dir)
    plot_performance_summary(data, out_dir)
    save_animation(data, out_dir)
