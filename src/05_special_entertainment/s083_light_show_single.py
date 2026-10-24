"""
S083 Light Show Single-Drone Test
=================================
One show drone flies a scripted figure-8 (Lissajous, A_x = A_y = 20 m) at z = 50 m in 60 s.
The script is 200 waypoints, interpolated with a cubic spline. A PID position controller
tracks the spline under Gaussian wind (sigma_w = 0.5 per axis, re-sampled every 0.05 s).
Two controllers are compared with the same wind samples:
  (1) PID-only
  (2) PID + feedforward of the spline's reference acceleration
  (3) PID + feedforward with a flying start (added): the drone already moves at the
      reference velocity at t = 0, which removes the launch transient of (1) and (2)
Point-mass dynamics are integrated with scipy.integrate.odeint, one call per 0.05 s step
(wind held constant within the step), vectorised over all Monte Carlo seeds.

Usage:
    conda activate drones
    python src/05_special_entertainment/s083_light_show_single.py
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401
from scipy.interpolate import CubicSpline
from scipy.integrate import odeint

# ── Parameters (card) ─────────────────────────────────────────────────────────
LED_SHOW_HEIGHT = 50.0    # m
AMP_X = 20.0              # m
AMP_Y = 20.0              # m
T_TOTAL = 60.0            # s
N_WAYPOINTS = 200
DT = 0.05                 # s
SIGMA_W = 0.5             # m/s^2 per axis (card writes m/s; it is added to the acceleration)
PASS_THRESHOLD = 0.5      # m
RMS_SPEC = 0.1            # m
KP, KI, KD = 2.0, 0.1, 0.8

# ── Parameters (added) ────────────────────────────────────────────────────────
N_MC = 100                # wind seeds (common random numbers for both controllers)
SEED = 83
T_SETTLE = 5.0            # s — metrics are also reported after this launch transient
SIGMA_SWEEP = [0.0, 0.1, 0.2, 0.3, 0.5, 1.0]
N_SWEEP = 30              # seeds per sweep point
A_LIMIT = 15.0            # m/s^2 — sanity bound on the commanded acceleration (card ext. 3)
ANIM_STEP = 5             # sim steps per GIF frame (0.25 s)
TRAIL_S = 5.0             # s — trailing path in the animation

N_STEPS = int(round(T_TOTAL / DT))
T_GRID = np.arange(N_STEPS) * DT
VARIANTS = ['pid', 'ff', 'fly']
LABELS = {'pid': 'PID-only', 'ff': 'PID + FF', 'fly': 'PID + FF, flying start'}
COLORS = {'pid': 'red', 'ff': 'blue', 'fly': 'purple'}

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), '..', '..',
                          'outputs', '05_special_entertainment', 's083_light_show_single')


# ── Reference ─────────────────────────────────────────────────────────────────

def make_reference():
    """200 waypoints uniformly in time, closed with the first waypoint at T, not-a-knot spline."""
    tau = np.linspace(0.0, 1.0, N_WAYPOINTS, endpoint=False)
    t_wp = tau * T_TOTAL
    wp = np.column_stack([AMP_X * np.sin(2 * np.pi * tau), AMP_Y * np.sin(4 * np.pi * tau),
                          np.full(N_WAYPOINTS, LED_SHOW_HEIGHT)])
    cs = CubicSpline(np.append(t_wp, T_TOTAL), np.vstack([wp, wp[:1]]))   # not-a-knot
    return cs, cs.derivative(1), cs.derivative(2), wp


def analytic_reference(t):
    w = 2 * np.pi / T_TOTAL
    return np.column_stack([AMP_X * np.sin(w * t), AMP_Y * np.sin(2 * w * t),
                            np.full_like(t, LED_SHOW_HEIGHT)])


# ── Simulation ────────────────────────────────────────────────────────────────

def simulate(feedforward, wind, ref, flying_start=False):
    """Integrate all seeds at once. wind: (S, N_STEPS, 3). State per seed: p, v, integral(e).

    flying_start=False: the drone starts at rest on the first waypoint (card).
    flying_start=True : the drone already moves with the reference velocity at t = 0.
    """
    cs, d1, d2, wp = ref
    S = wind.shape[0]

    def rhs(y, t, w_k):
        s = y.reshape(S, 9)
        p, v, ie = s[:, :3], s[:, 3:6], s[:, 6:]
        e = cs(t) - p
        a = KP * e + KI * ie + KD * (d1(t) - v) + w_k
        if feedforward:
            a = a + d2(t)
        return np.concatenate([v, a, e], axis=1).ravel()

    state = np.zeros((S, 9))
    state[:, :3] = wp[0]
    if flying_start:
        state[:, 3:6] = d1(0.0)
    traj = np.zeros((N_STEPS, S, 3))
    a_cmd = np.zeros((N_STEPS, S, 3))
    traj[0] = state[:, :3]
    for k in range(N_STEPS):
        t = T_GRID[k]
        p, v, ie = state[:, :3], state[:, 3:6], state[:, 6:]
        e = cs(t) - p
        a_cmd[k] = KP * e + KI * ie + KD * (d1(t) - v) + (d2(t) if feedforward else 0.0)
        if k == N_STEPS - 1:
            break
        state = odeint(rhs, state.ravel(), [t, T_GRID[k + 1]], args=(wind[:, k],),
                       rtol=1e-7, atol=1e-8)[-1].reshape(S, 9)
        traj[k + 1] = state[:, :3]
        assert np.all(np.linalg.norm(traj[k + 1] - traj[k], axis=1) < 10.0 * DT)  # no jumps

    ref_pos = cs(T_GRID)
    err = np.linalg.norm(traj - ref_pos[:, None, :], axis=2)          # (N, S)
    a_norm = np.linalg.norm(a_cmd, axis=2)
    assert np.all(traj[:, :, 2] > 0.0)                               # never below ground
    assert np.all(a_norm <= A_LIMIT)                                  # thrust sanity
    settled = T_GRID >= T_SETTLE
    return {
        'traj': traj, 'err': err, 'ref_pos': ref_pos,
        'rms': np.sqrt(np.mean(err ** 2, axis=0)),
        'max': err.max(axis=0),
        'compl': 100.0 * np.mean(err <= PASS_THRESHOLD, axis=0),
        'rms_s': np.sqrt(np.mean(err[settled] ** 2, axis=0)),
        'max_s': err[settled].max(axis=0),
        'compl_s': 100.0 * np.mean(err[settled] <= PASS_THRESHOLD, axis=0),
        'a_peak': float(a_norm.max()),
    }


def run_simulation():
    ref = make_reference()
    rng = np.random.default_rng(SEED)
    unit_wind = rng.standard_normal((N_MC, N_STEPS, 3))             # shared by both controllers
    results = {v: simulate(v != 'pid', SIGMA_W * unit_wind, ref, v == 'fly') for v in VARIANTS}

    sweep = {v: [] for v in VARIANTS}
    for sig in SIGMA_SWEEP:
        for v in VARIANTS:
            r = simulate(v != 'pid', sig * unit_wind[:N_SWEEP], ref, v == 'fly')
            sweep[v].append((r['rms'].mean(), r['rms_s'].mean(), r['rms_s'].std(),
                             int(np.sum(r['rms'] <= RMS_SPEC)),
                             int(np.sum(r['rms_s'] <= RMS_SPEC))))

    # Wind-free runs: how the launch transient decays (integral mode)
    calm = np.zeros((1, N_STEPS, 3))
    decay = {v: simulate(v != 'pid', calm, ref, v == 'fly')['err'][:, 0] for v in VARIANTS}
    return ref, results, sweep, decay


# ── Plotting ──────────────────────────────────────────────────────────────────

def plot_trajectory_3d(results, out_dir, seed=0):
    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(111, projection='3d')
    ref = results['pid']['ref_pos']
    ax.plot(ref[:, 0], ref[:, 1], ref[:, 2], color='green', lw=2.5, label='Reference (spline)')
    for v in VARIANTS:
        tr = results[v]['traj'][:, seed]
        ax.plot(tr[:, 0], tr[:, 1], tr[:, 2], color=COLORS[v], lw=1.0, alpha=0.85,
                label=f"{LABELS[v]} (RMS {100 * results[v]['rms'][seed]:.1f} cm)")
    ax.scatter(*ref[0], color='k', s=40, label='Start')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_zlabel('z (m)')
    ax.set_zlim(48, 52)
    ax.set_box_aspect((1, 1, 0.35), zoom=0.9)
    ax.set_title(f'S083 Figure-8 light-show path at z = {LED_SHOW_HEIGHT:.0f} m (seed {seed})')
    ax.legend(fontsize=8, loc='upper left')
    path = os.path.join(out_dir, 'trajectory_3d.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_error_timeseries(results, decay, out_dir, seed=0):
    fig, axes = plt.subplots(2, 1, figsize=(11, 8), sharex=True)
    series = [(axes[0], {v: results[v]['err'][:, seed] for v in VARIANTS},
               f'With wind σ_w = {SIGMA_W} (seed {seed})'),
              (axes[1], decay, 'Without wind: launch transient only')]
    for ax, errs, title in series:
        for v in VARIANTS:
            ax.plot(T_GRID, 100 * np.maximum(errs[v], 1e-4), color=COLORS[v], lw=1.1,
                    label=LABELS[v])
        ax.axhline(100 * RMS_SPEC, color='k', ls='--', lw=1.3,
                   label=f'RMS spec {100 * RMS_SPEC:.0f} cm')
        ax.axhline(100 * PASS_THRESHOLD, color='grey', ls=':', lw=1.3,
                   label=f'Path threshold {100 * PASS_THRESHOLD:.0f} cm')
        ax.axvline(T_SETTLE, color='k', ls='-.', lw=0.8, label=f'settling cut {T_SETTLE:.0f} s')
        ax.set_yscale('log')
        ax.set_ylim(1e-2, 400)
        ax.set_ylabel('Position error (cm, log)')
        ax.set_title(title)
        ax.grid(True, which='both', alpha=0.3)
    axes[0].legend(fontsize=8, ncol=3, loc='lower left')
    axes[1].set_xlabel('Time (s)')
    fig.suptitle('S083 Tracking error vs time')
    plt.tight_layout()
    path = os.path.join(out_dir, 'error_timeseries.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_monte_carlo(results, out_dir):
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.8))
    items = [('rms', 'rms_s', 'RMS error (cm)', 100), ('max', 'max_s', 'Max error (cm)', 100),
             ('compl', 'compl_s', 'Path completion (%)', 1)]
    for ax, (k_full, k_set, lab, sc) in zip(axes, items):
        data = [sc * results[v][k] for v in VARIANTS for k in (k_full, k_set)]
        bp = ax.boxplot(data, showmeans=True, patch_artist=True)
        for patch, v in zip(bp['boxes'], [v for v in VARIANTS for _ in range(2)]):
            patch.set_facecolor(COLORS[v])
            patch.set_alpha(0.35)
        short = {'pid': 'PID', 'ff': 'PID+FF', 'fly': 'FF fly'}
        ax.set_xticks(range(1, 2 * len(VARIANTS) + 1),
                      [f'{short[v]}\n{w}' for v in VARIANTS for w in ('full', '>5 s')])
        ax.set_ylabel(lab)
        ax.grid(alpha=0.3)
        if k_full == 'rms':
            ax.axhline(100 * RMS_SPEC, color='k', ls='--', lw=1, label='spec 10 cm')
            ax.set_yscale('log')
            ax.legend(fontsize=8)
    fig.suptitle(f'S083 Monte Carlo over {N_MC} wind seeds (same wind for every controller; '
                 'triangle = mean)')
    plt.tight_layout()
    path = os.path.join(out_dir, 'monte_carlo.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_wind_sweep(sweep, out_dir):
    fig, ax = plt.subplots(figsize=(8, 5))
    for v in VARIANTS:
        s = np.array(sweep[v], dtype=float)
        ax.plot(SIGMA_SWEEP, 100 * s[:, 0], 'o:', color=COLORS[v], alpha=0.6,
                label=f'{LABELS[v]} (full mission)')
        ax.errorbar(SIGMA_SWEEP, 100 * s[:, 1], yerr=100 * s[:, 2], fmt='s-', capsize=3,
                    color=COLORS[v], label=f'{LABELS[v]} (after {T_SETTLE:.0f} s)')
    ax.axhline(100 * RMS_SPEC, color='k', ls='--', lw=1.2, label='RMS spec 10 cm')
    ax.set_ylim(0, 55)
    ax.set_xlabel(r'Wind std $\sigma_w$ (m/s$^2$ per axis)')
    ax.set_ylabel('RMS error (cm)')
    ax.set_title(f'S083 RMS error vs wind strength ({N_SWEEP} seeds per point)')
    ax.grid(alpha=0.3, which='both')
    ax.legend(fontsize=8, loc='upper left', bbox_to_anchor=(1.01, 1.0))
    path = os.path.join(out_dir, 'wind_sweep.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def anim_indices(n, step):
    n_frames = int(np.ceil((n - 1) / step)) + 1
    return [min(i * step, n - 1) for i in range(n_frames)]


def save_animation(results, out_dir, seed=0):
    idx = anim_indices(N_STEPS, ANIM_STEP)
    ref = results['pid']['ref_pos']
    trail = int(round(TRAIL_S / DT))
    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(12, 5.6),
                                  gridspec_kw={'width_ratios': [1.0, 1.15]})
    ax.plot(ref[:, 0], ref[:, 1], color='green', lw=1.5, alpha=0.5, label='Reference')
    lines, dots = {}, {}
    for v in VARIANTS:
        lines[v], = ax.plot([], [], color=COLORS[v], lw=1.2, alpha=0.7, label=LABELS[v])
        dots[v], = ax.plot([], [], 'o', color=COLORS[v], ms=7)
    rdot, = ax.plot([], [], 'o', color='green', ms=6)
    txt = ax.text(0.02, 0.95, '', transform=ax.transAxes)
    ax.set_xlim(-AMP_X * 1.3, AMP_X * 1.3)
    ax.set_ylim(-AMP_Y * 1.3, AMP_Y * 1.3)
    ax.set_aspect('equal')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title('Top-down view (last 5 s trail)')
    ax.legend(loc='upper right', fontsize=8)
    ax.grid(alpha=0.3)

    elines = {}
    for v in VARIANTS:
        elines[v], = ax2.plot([], [], color=COLORS[v], lw=1.1, label=LABELS[v])
    ax2.axhline(100 * RMS_SPEC, color='k', ls='--', lw=1.0, label='spec 10 cm')
    ax2.set_xlim(0, T_TOTAL)
    ax2.set_yscale('log')
    ymax = 100 * max(results[v]['err'][:, seed].max() for v in VARIANTS)
    ax2.set_ylim(0.1, ymax * 1.5)
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Error to reference (cm)')
    ax2.set_title('Tracking error (zoomed: invisible at map scale)')
    ax2.legend(fontsize=8, loc='upper right')
    ax2.grid(alpha=0.3, which='both')
    fig.suptitle(f'S083 Light Show Single-Drone Test (seed {seed})')

    def update(i):
        k = idx[i]
        k0 = max(0, k - trail)
        for v in VARIANTS:
            tr = results[v]['traj'][:, seed]
            lines[v].set_data(tr[k0:k + 1, 0], tr[k0:k + 1, 1])
            dots[v].set_data([tr[k, 0]], [tr[k, 1]])
            elines[v].set_data(T_GRID[:k + 1], 100 * results[v]['err'][:k + 1, seed])
        rdot.set_data([ref[k, 0]], [ref[k, 1]])
        txt.set_text(f't = {T_GRID[k]:.2f} s')
        return list(lines.values()) + list(dots.values()) + list(elines.values()) + [rdot, txt]

    ani = animation.FuncAnimation(fig, update, frames=len(idx), blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer='pillow', fps=20, dpi=72)
    plt.close(fig)
    print(f'Saved: {path} ({len(idx)} frames)')


# ── Main ──────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    ref, results, sweep, decay = run_simulation()
    cs, d1, d2, wp = ref

    t_fine = np.linspace(0, T_TOTAL, 6001)
    spl_err = np.linalg.norm(cs(t_fine) - analytic_reference(t_fine), axis=1).max()
    speed = np.linalg.norm(d1(t_fine), axis=1)
    acc = np.linalg.norm(d2(t_fine), axis=1)
    print('\n=== S083 Light Show Single-Drone Test — Results ===')
    print(f'  Spline vs analytic Lissajous: max deviation {1000 * spl_err:.3f} mm')
    print(f'  Reference speed: start {speed[0]:.2f} m/s, max {speed.max():.2f} m/s; '
          f'max reference acceleration {acc.max():.3f} m/s^2')
    print(f'  Steady lag of PID-only at peak reference accel: a/Kp = {acc.max() / KP:.3f} m')
    poles = np.roots([1.0, KD, KP, KI])
    print('  Closed-loop poles (s^3 + Kd s^2 + Kp s + Ki): ' +
          ', '.join(f'{p.real:.4f}{p.imag:+.4f}j' for p in poles) +
          f'; slowest time constant {1.0 / np.min(np.abs(poles.real)):.1f} s')
    print(f'  Monte Carlo: {N_MC} wind seeds, same wind for every controller')
    hdr = f"  {'Controller':<24}{'RMS (cm)':>18}{'Max (cm)':>18}{'Completion (%)':>18}  Spec pass"
    for tag, sfx in [('full 60 s mission (card metric)', ''), (f'after {T_SETTLE:.0f} s', '_s')]:
        print(f'  -- {tag} --')
        print(hdr)
        for v in VARIANTS:
            r = results[v]
            npass = int(np.sum(r['rms' + sfx] <= RMS_SPEC))
            print(f"  {LABELS[v]:<24}{100 * r['rms' + sfx].mean():>10.2f} ± {100 * r['rms' + sfx].std():5.2f}"
                  f"{100 * r['max' + sfx].mean():>11.2f} ± {100 * r['max' + sfx].std():5.2f}"
                  f"{r['compl' + sfx].mean():>11.2f} ± {r['compl' + sfx].std():5.2f}"
                  f"  {npass}/{N_MC}")
    for a, b in [('ff', 'pid'), ('fly', 'ff')]:
        for sfx, tag in [('', 'full'), ('_s', f'after {T_SETTLE:.0f} s')]:
            w_rms = int(np.sum(results[a]['rms' + sfx] < results[b]['rms' + sfx]))
            w_max = int(np.sum(results[a]['max' + sfx] < results[b]['max' + sfx]))
            print(f'  {LABELS[a]} better than {LABELS[b]} ({tag}): RMS in {w_rms}/{N_MC} seeds, '
                  f'max in {w_max}/{N_MC} seeds')
    print('  Seed 0 (figures):')
    for v in VARIANTS:
        r = results[v]
        ok = 'PASS' if r['rms'][0] <= RMS_SPEC else 'FAIL'
        print(f"    [{LABELS[v]}] RMS {100 * r['rms'][0]:.2f} cm ({ok} spec {100 * RMS_SPEC:.0f} cm), "
              f"max {100 * r['max'][0]:.2f} cm, completion {r['compl'][0]:.1f}%; "
              f"after {T_SETTLE:.0f} s: RMS {100 * r['rms_s'][0]:.2f} cm")
    for v in VARIANTS:
        e0 = results[v]['err'][:, 0]
        print(f"    [{LABELS[v]}] error at t = 0.5 / 1 / 2 s: "
              f"{100 * e0[10]:.1f} / {100 * e0[20]:.1f} / {100 * e0[40]:.1f} cm")
    print('  Peak commanded acceleration: ' + ', '.join(
        f"{LABELS[v]} {results[v]['a_peak']:.2f}" for v in VARIANTS) +
        f' m/s^2 (sanity bound {A_LIMIT:.0f})')
    print('  Wind-free RMS error per time window (cm): 0-5 / 5-10 / 10-20 / 20-30 / 30-45 / 45-60 s')
    for v in VARIANTS:
        vals = []
        for t0, t1 in [(0, 5), (5, 10), (10, 20), (20, 30), (30, 45), (45, 60)]:
            sel = (T_GRID >= t0) & (T_GRID < t1)
            vals.append(f'{100 * np.sqrt(np.mean(decay[v][sel] ** 2)):.2f}')
        print(f'    {LABELS[v]:<24}' + ' / '.join(vals))
    print(f'  Wind sweep ({N_SWEEP} seeds): sigma_w -> RMS full / after {T_SETTLE:.0f} s (cm), '
          f'seeds passing 10 cm (full / after {T_SETTLE:.0f} s)')
    for j, sig in enumerate(SIGMA_SWEEP):
        print(f'    {sig:.1f}: ' + '; '.join(
            f'{LABELS[v]} {100 * sweep[v][j][0]:.2f} / {100 * sweep[v][j][1]:.2f} '
            f'({sweep[v][j][3]} / {sweep[v][j][4]})' for v in VARIANTS))

    out = os.path.normpath(OUTPUT_DIR)
    os.makedirs(out, exist_ok=True)
    plot_trajectory_3d(results, out)
    plot_error_timeseries(results, decay, out)
    plot_monte_carlo(results, out)
    plot_wind_sweep(sweep, out)
    save_animation(results, out)
