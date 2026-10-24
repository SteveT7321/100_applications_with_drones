"""
S083 3D Upgrade — Light Show Single Drone Test
==============================================
The S083 show drone (200-waypoint not-a-knot spline, PID Kp/Ki/Kd = 2.0/0.1/0.8, point mass,
Gaussian wind sigma_w = 0.5 m/s^2 per axis re-sampled every 0.05 s, odeint per step) now flies
a figure-8 whose altitude is part of the choreography:

    p_ref(tau) = (20 sin 2 pi tau,  20 sin 4 pi tau,  50 + A_z sin(4 pi tau + phi_z)),  tau = t/60

Paths compared (same wind per seed for every path and controller — common random numbers):
  flat   A_z = 0          : the S083 2D path at 50 m (2D baseline, X5)
  card   A_z = 10, phi = 0 : the card's formula; z = 50 + y/2, i.e. a figure-8 in a TILTED PLANE
  arc    A_z = 10, phi = -90 deg : the shape the card describes in words (high on the outer
                           lobes, low at the crossing); z = 40 + x^2/20
  opt    (A_z, phi_z) maximising the audience legibility score (grid + gradient ascent)
Controllers (from S083): PID-only (card), PID + FF (card), PID + FF with a flying start (added in S083).
The LED brightness follows altitude: l = clip(0.3 + 0.7 (z - 40)/20, 0.3, 1.0).

Usage:
    conda activate drones
    python src/05_special_entertainment/3d/s083_3d_light_show.py
"""

import os
import importlib.util
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.collections import LineCollection
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401
from mpl_toolkits.mplot3d.art3d import Line3DCollection
from scipy.interpolate import CubicSpline
import matplotlib.colors as mcolors

_HERE = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location(
    's083_2d', os.path.join(_HERE, '..', 's083_light_show_single.py'))
m2d = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(m2d)

OUTPUT_DIR = os.path.normpath(os.path.join(
    _HERE, '..', '..', '..', 'outputs', '05_special_entertainment', '3d', 's083_3d_light_show'))

# ── Parameters (card) ─────────────────────────────────────────────────────────
Z_C        = 50.0                 # m
Z_LOW, Z_HIGH = 40.0, 60.0        # m
A_Z        = 10.0                 # m
AMP_X = AMP_Y = m2d.AMP_X         # 20 m
T_TOTAL    = m2d.T_TOTAL          # 60 s
N_WP       = m2d.N_WAYPOINTS      # 200
DT         = m2d.DT               # 0.05 s
SIGMA_W    = m2d.SIGMA_W          # 0.5 m/s^2 per axis (card writes m/s; corrected)
KP, KI, KD = m2d.KP, m2d.KI, m2d.KD
RMS_SPEC_3D = 0.15                # m (card, 3D)
RMS_SPEC_2D = m2d.RMS_SPEC        # 0.10 m (S083)
PASS_THR   = m2d.PASS_THRESHOLD   # 0.5 m
L_MIN, L_MAX = 0.3, 1.0
H_WARM, H_COLD = 30.0, 200.0      # deg (HSV hue)
R_AUD      = 100.0                # m
N_AUD      = 36
Z_SOFT     = (Z_LOW - 5.0, Z_HIGH + 5.0)   # 35 - 65 m

# ── Parameters (added) ────────────────────────────────────────────────────────
N_MC       = 100
SEED       = 8300                 # wind of seed s from default_rng(SEED + s)
T_SETTLE   = m2d.T_SETTLE         # 5 s (S083)
AZ_GRID    = np.linspace(0.0, 10.0, 41)        # legibility search, A_z <= 10 keeps 40-60 m
PHI_GRID   = np.radians(np.arange(0, 360, 5))
GA_STEP    = (0.5, np.radians(5.0))            # gradient-ascent step scale (A_z m, phi rad)
GA_ITERS   = 200
R_AUD_SWEEP = [50.0, 100.0, 200.0]
AZ_SWEEP   = [0.0, 2.5, 5.0, 7.5, 10.0]       # PID-only wind-free lag probe
ANIM_STEP  = 6                                 # 0.3 s per frame

PATHS = {'flat': (0.0, 0.0), 'card': (A_Z, 0.0), 'arc': (A_Z, -np.pi / 2)}
PLAB = {'flat': '2D baseline: flat at 50 m', 'card': 'Card formula (tilted plane)',
        'arc': 'Arc (high on lobes)', 'opt': 'Legibility-optimised'}
PCOL = {'flat': 'grey', 'card': 'darkorange', 'arc': 'teal', 'opt': 'magenta'}
CTRL = ['pid', 'ff', 'fly']
CLAB = {'pid': 'PID-only', 'ff': 'PID + FF', 'fly': 'PID + FF, flying start'}


# ── Reference, LED, audience ──────────────────────────────────────────────────

def waypoints(az, phi, n=N_WP):
    tau = np.linspace(0.0, 1.0, n, endpoint=False)
    return np.column_stack([AMP_X * np.sin(2 * np.pi * tau), AMP_Y * np.sin(4 * np.pi * tau),
                            Z_C + az * np.sin(4 * np.pi * tau + phi)])


def make_reference(az, phi):
    wp = waypoints(az, phi)
    t_wp = np.linspace(0.0, T_TOTAL, N_WP, endpoint=False)
    cs = CubicSpline(np.append(t_wp, T_TOTAL), np.vstack([wp, wp[:1]]))   # not-a-knot, as S083
    return cs, cs.derivative(1), cs.derivative(2), wp


def led(z):
    return np.clip(L_MIN + (L_MAX - L_MIN) * (z - Z_LOW) / (Z_HIGH - Z_LOW), L_MIN, L_MAX)


def led_rgb(lv):
    hue = (H_WARM + (H_COLD - H_WARM) * lv) / 360.0
    return mcolors.hsv_to_rgb(np.column_stack([hue, np.ones_like(hue), np.ones_like(hue)]))


def observers(r_aud=R_AUD):
    ph = np.pi * np.arange(N_AUD) / (N_AUD - 1)
    return np.column_stack([r_aud * np.cos(ph), r_aud * np.sin(ph), np.zeros(N_AUD)])


def legibility(wp, r_aud=R_AUD, per_segment=False):
    """Card score: mean over segments of min over observers of asin(|d x r|) (deg)."""
    q = observers(r_aud)
    d = np.diff(wp, axis=0)
    d /= np.linalg.norm(d, axis=1, keepdims=True)
    mid = 0.5 * (wp[1:] + wp[:-1])
    r = mid[:, None, :] - q[None, :, :]
    r /= np.linalg.norm(r, axis=2, keepdims=True)
    phi = np.degrees(np.arcsin(np.clip(np.linalg.norm(np.cross(d[:, None, :], r), axis=2), 0, 1)))
    m = phi.min(axis=1)
    return (m, int(np.sum(m < 5.0))) if per_segment else float(m.mean())


def optimise_legibility(r_aud=R_AUD):
    grid = np.array([[legibility(waypoints(a, p), r_aud) for p in PHI_GRID] for a in AZ_GRID])
    ia, ip = np.unravel_index(np.argmax(grid), grid.shape)     # first max: deterministic ties
    return grid, (AZ_GRID[ia], PHI_GRID[ip], grid[ia, ip])


def gradient_ascent(x0, r_aud=R_AUD):
    """Finite-difference gradient ascent on (A_z, phi) with A_z clipped to [0, 10]."""
    x = np.array(x0, float)
    f = lambda v: legibility(waypoints(np.clip(v[0], 0, 10), v[1]), r_aud)
    h = np.array([1e-3, 1e-3])
    for _ in range(GA_ITERS):
        g = np.array([(f(x + h[i] * np.eye(2)[i]) - f(x - h[i] * np.eye(2)[i])) / (2 * h[i])
                      for i in range(2)])
        if np.linalg.norm(g) < 1e-9:
            break
        step = np.array(GA_STEP) * g / np.linalg.norm(g)
        for _ in range(10):                      # backtracking: halve the step until f improves
            xn = x + step
            xn[0] = np.clip(xn[0], 0, 10)
            if f(xn) > f(x):
                break
            step = 0.5 * step
        else:
            break
        x = xn
    return np.clip(x[0], 0, 10), np.mod(x[1], 2 * np.pi), f(x)


# ── Simulation ────────────────────────────────────────────────────────────────

def unit_wind(seeds):
    return np.stack([np.random.default_rng(SEED + s).standard_normal((m2d.N_STEPS, 3))
                     for s in seeds])


def simulate(ctrl, wind, ref):
    """The S083 simulator (same PID, odeint per step, asserts) on a 3D reference."""
    r = m2d.simulate(ctrl != 'pid', wind, ref, flying_start=(ctrl == 'fly'))
    z = r['traj'][:, :, 2]
    assert np.all(z > Z_SOFT[0]) and np.all(z < Z_SOFT[1])      # soft altitude bounds never reached
    ez = r['traj'] - r['ref_pos'][:, None, :]
    settled = m2d.T_GRID >= T_SETTLE
    r['rms_z_s'] = np.sqrt(np.mean(ez[settled, :, 2] ** 2, axis=0))
    r['rms_xy_s'] = np.sqrt(np.mean(np.sum(ez[settled, :, :2] ** 2, axis=2), axis=0))
    r['led_err'] = np.sqrt(np.mean((led(z) - led(r['ref_pos'][:, None, 2])) ** 2, axis=0))
    r['zmin'], r['zmax'] = float(z.min()), float(z.max())
    return r


def run_simulation():
    grid, (a_g, p_g, s_g) = optimise_legibility()
    a_o, p_o, s_o = gradient_ascent((a_g, p_g))
    a_c, p_c, s_c = gradient_ascent((A_Z, 0.0))           # card's start point
    paths = dict(PATHS, opt=(a_o, p_o))
    refs = {k: make_reference(*v) for k, v in paths.items()}
    wind = SIGMA_W * unit_wind(range(N_MC))
    res = {(p, c): simulate(c, wind, refs[p]) for p in paths for c in CTRL}
    calm = np.zeros((1, m2d.N_STEPS, 3))
    lag = {(a, ph): simulate('pid', calm, make_reference(a, ph))
           for a in AZ_SWEEP for ph in (0.0, -np.pi / 2)}
    raud = {r: {k: legibility(waypoints(*v), r) for k, v in paths.items()} for r in R_AUD_SWEEP}
    raud_best = {r: optimise_legibility(r)[1] for r in R_AUD_SWEEP}
    # CRN order check: the wind of seed s does not depend on the batch; odeint's adaptive step
    # control acts on the whole batch, so trajectories agree only to solver tolerance
    sub = simulate('fly', SIGMA_W * unit_wind([5, 2]), refs['card'])
    batch_diff = float(np.abs(sub['traj'] - res[('card', 'fly')]['traj'][:, [5, 2]]).max())
    assert batch_diff < 1e-5
    opt_info = dict(grid=grid, grid_best=(a_g, p_g, s_g), ga=(a_o, p_o, s_o), ga_card=(a_c, p_c, s_c),
                    batch_diff=batch_diff)
    return paths, refs, res, lag, raud, raud_best, opt_info


# ── Plotting ──────────────────────────────────────────────────────────────────

def plot_trajectory_3d(paths, refs, res, out_dir, seed=0):
    fig = plt.figure(figsize=(15, 6.5))
    for i, p in enumerate(['card', 'arc', 'opt']):
        ax = fig.add_subplot(1, 3, i + 1, projection='3d')
        ref = res[(p, 'fly')]['ref_pos']
        lv = led(ref[:, 2])
        segs = np.stack([ref[:-1], ref[1:]], axis=1)
        lc = Line3DCollection(segs, colors=led_rgb(lv[:-1]), lw=3.5, label='Reference (LED colour)')
        ax.add_collection3d(lc)
        fl = res[('flat', 'fly')]['ref_pos']
        ax.plot(fl[:, 0], fl[:, 1], fl[:, 2], color='grey', lw=1, ls='--', label='2D path at 50 m')
        for c, col in (('pid', 'red'), ('fly', 'blue')):
            tr = res[(p, c)]['traj'][:, seed]
            ax.plot(tr[:, 0], tr[:, 1], tr[:, 2], color=col, lw=0.8,
                    label=f"{CLAB[c]} (RMS {100 * res[(p, c)]['rms'][seed]:.1f} cm)")
        ax.set_xlim(-22, 22)
        ax.set_ylim(-22, 22)
        ax.set_zlim(35, 65)
        ax.set_xlabel('x (m)')
        ax.set_ylabel('y (m)')
        ax.set_zlabel('z (m)')
        a, ph = paths[p]
        ax.set_title(f'{PLAB[p]}\nA_z = {a:.2f} m, phi_z = {np.degrees(ph):.0f} deg', fontsize=10)
        ax.view_init(elev=18, azim=-55)
        ax.legend(fontsize=7, loc='upper left')
    fig.suptitle(f'S083 3D light-show paths (seed {seed}); reference coloured by LED hue '
                 '(yellow-green = low / dim, sky-blue = high / bright)')
    plt.tight_layout()
    path = os.path.join(out_dir, 'trajectory_3d.png')
    plt.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_altitude_led(res, out_dir, seed=0):
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(12, 7.5), sharex=True)
    t = m2d.T_GRID
    p = 'arc'
    a1.plot(t, res[(p, 'fly')]['ref_pos'][:, 2], 'g-', lw=2.5, label='Reference (arc)')
    for c, col in (('pid', 'red'), ('ff', 'orange'), ('fly', 'blue')):
        a1.plot(t, res[(p, c)]['traj'][:, seed, 2], color=col, lw=1, label=CLAB[c])
    a1.plot(t, res[('card', 'fly')]['ref_pos'][:, 2], color=PCOL['card'], lw=1, ls=':',
            label='Reference (card formula)')
    for zz in (Z_LOW, Z_HIGH):
        a1.axhline(zz, color='k', ls='--', lw=0.7)
    a1.set_ylabel('Altitude (m)')
    a1.legend(fontsize=8, ncol=3)
    a1.grid(alpha=0.3)
    a1.set_title(f'Altitude, arc path (seed {seed})')
    a2.plot(t, led(res[(p, 'fly')]['ref_pos'][:, 2]), 'g-', lw=2.5, label='Reference brightness')
    for c, col in (('pid', 'red'), ('fly', 'blue')):
        a2.plot(t, led(res[(p, c)]['traj'][:, seed, 2]), color=col, lw=1, label=CLAB[c])
    a2.axhline(led(np.array([Z_C]))[0], color='grey', ls='--', lw=1, label='2D path (constant)')
    a2.set_ylabel('LED intensity')
    a2.set_xlabel('Time (s)')
    a2.legend(fontsize=8, ncol=2)
    a2.grid(alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'altitude_led.png')
    plt.savefig(path, dpi=140, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_error(res, out_dir, seed=0):
    fig, axes = plt.subplots(1, 2, figsize=(14, 4.8), sharey=True)
    t = m2d.T_GRID
    for ax, c in zip(axes, ['pid', 'fly']):
        for p in ['flat', 'card', 'arc', 'opt']:
            ax.plot(t, 100 * res[(p, c)]['err'][:, seed], color=PCOL[p], lw=0.9,
                    label=f"{PLAB[p]} (RMS {100 * res[(p, c)]['rms'][seed]:.1f} cm)")
        ax.axhline(100 * RMS_SPEC_3D, color='k', ls='--', lw=1, label='3D spec 15 cm')
        ax.set_yscale('log')
        ax.set_ylim(0.5, 400)
        ax.set_xlabel('Time (s)')
        ax.set_title(CLAB[c])
        ax.grid(alpha=0.3, which='both')
        ax.legend(fontsize=7)
    axes[0].set_ylabel('Tracking error (cm, log)')
    fig.suptitle(f'Tracking error (seed {seed}); with feedforward + flying start the four curves coincide')
    plt.tight_layout()
    path = os.path.join(out_dir, 'error_timeseries.png')
    plt.savefig(path, dpi=140, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_legibility(paths, opt_info, raud, out_dir):
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(14, 5))
    g = opt_info['grid']
    im = a1.imshow(g, origin='lower', aspect='auto', cmap='viridis',
                   extent=[0, 360, AZ_GRID[0], AZ_GRID[-1]])
    plt.colorbar(im, ax=a1, label=r'$\bar\Phi_{min}$ (deg)')
    for p, mk in (('flat', 'o'), ('card', 's'), ('arc', '^'), ('opt', '*')):
        a, ph = paths[p]
        a1.plot(np.degrees(np.mod(ph, 2 * np.pi)), a, mk, color='white', mec='k', ms=11, label=PLAB[p])
    a1.set_xlim(-8, 368)
    a1.set_ylim(-0.4, 10.4)
    a1.set_xlabel(r'Vertical phase $\phi_z$ (deg)')
    a1.set_ylabel('$A_z$ (m)')
    a1.set_title(f'Audience legibility, R_aud = {R_AUD:.0f} m')
    a1.legend(fontsize=7, loc='center right')
    x = np.arange(len(R_AUD_SWEEP))
    w = 0.2
    for i, p in enumerate(['flat', 'card', 'arc', 'opt']):
        a2.bar(x + (i - 1.5) * w, [raud[r][p] for r in R_AUD_SWEEP], w, color=PCOL[p], label=PLAB[p])
    a2.set_xticks(x, [f'R_aud = {r:.0f} m' for r in R_AUD_SWEEP])
    a2.set_ylabel(r'$\bar\Phi_{min}$ (deg)')
    a2.set_title('Legibility score of each path vs audience radius')
    a2.legend(fontsize=8)
    a2.grid(alpha=0.3, axis='y')
    plt.tight_layout()
    path = os.path.join(out_dir, 'legibility.png')
    plt.savefig(path, dpi=140, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_monte_carlo(res, out_dir):
    fig, axes = plt.subplots(1, 3, figsize=(16, 4.8))
    for ax, c in zip(axes, CTRL):
        data = [100 * res[(p, c)]['rms_s'] for p in ['flat', 'card', 'arc', 'opt']]
        bp = ax.boxplot(data, showmeans=True, patch_artist=True)
        for patch, p in zip(bp['boxes'], ['flat', 'card', 'arc', 'opt']):
            patch.set_facecolor(PCOL[p])
            patch.set_alpha(0.4)
        ax.set_xticks(range(1, 5), ['flat (2D)', 'card', 'arc', 'opt'])
        ax.axhline(100 * RMS_SPEC_3D, color='k', ls='--', lw=1, label='3D spec 15 cm')
        ax.axhline(100 * RMS_SPEC_2D, color='k', ls=':', lw=1, label='2D spec 10 cm')
        ax.set_title(f'{CLAB[c]}: RMS after {T_SETTLE:.0f} s')
        ax.set_ylabel('RMS error (cm)')
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
    fig.suptitle(f'S083 3D Monte Carlo, {N_MC} wind seeds (same wind for every path and controller)')
    plt.tight_layout()
    path = os.path.join(out_dir, 'monte_carlo.png')
    plt.savefig(path, dpi=140, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def anim_indices(n, step):
    n_frames = int(np.ceil((n - 1) / step)) + 1
    return [min(i * step, n - 1) for i in range(n_frames)]


def save_animation(res, out_dir, seed=0):
    r = res[('arc', 'fly')]
    tr = r['traj'][:, seed]
    ref = r['ref_pos']
    idx = anim_indices(m2d.N_STEPS, ANIM_STEP)
    fig = plt.figure(figsize=(7, 6))
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(ref[:, 0], ref[:, 1], ref[:, 2], color='green', lw=0.8, alpha=0.4)
    ax.set_xlim(-22, 22)
    ax.set_ylim(-22, 22)
    ax.set_zlim(35, 65)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_zlabel('z (m)')
    ax.view_init(elev=18, azim=-55)
    trail, = ax.plot([], [], [], color='k', lw=0.8, alpha=0.5)
    dot = ax.scatter([tr[0, 0]], [tr[0, 1]], [tr[0, 2]], s=120)

    def update(i):
        nonlocal dot
        k = idx[i]
        k0 = max(0, k - 100)
        trail.set_data(tr[k0:k + 1, 0], tr[k0:k + 1, 1])
        trail.set_3d_properties(tr[k0:k + 1, 2])
        dot.remove()
        lv = led(np.array([tr[k, 2]]))
        dot = ax.scatter([tr[k, 0]], [tr[k, 1]], [tr[k, 2]], s=60 + 140 * lv[0],
                         color=led_rgb(lv), edgecolors='k')
        ax.set_title(f'Arc path, PID + FF flying start (seed {seed})\n'
                     f't = {m2d.T_GRID[k]:.1f} s   z = {tr[k, 2]:.1f} m   LED = {lv[0]:.2f}', fontsize=9)
        return trail, dot

    ani = animation.FuncAnimation(fig, update, frames=len(idx), blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    ani.save(path, writer='pillow', fps=20, dpi=72)
    plt.close(fig)
    print(f'Saved: {path} ({len(idx)} frames)')


# ── Main ──────────────────────────────────────────────────────────────────────

def ms(a, sc=100.0):
    return f'{sc * a.mean():.2f} ± {sc * a.std():.2f}'


if __name__ == '__main__':
    paths, refs, res, lag, raud, raud_best, oi = run_simulation()
    print('=== S083 3D Light Show ===')
    for p, (a, ph) in paths.items():
        wp = waypoints(a, ph)
        tf = np.linspace(0, T_TOTAL, 6001)
        cs, d1, d2, _ = refs[p]
        acc = np.linalg.norm(d2(tf), axis=1)
        accz = np.abs(d2(tf)[:, 2])
        spd = np.linalg.norm(d1(tf), axis=1)
        print(f'  {PLAB[p]:<30} A_z = {a:5.2f} m, phi_z = {np.degrees(np.mod(ph, 2 * np.pi)):6.1f} deg: '
              f'z {wp[:, 2].min():.2f}-{wp[:, 2].max():.2f} m, z at crossing (tau = 0) {wp[0, 2]:.2f} m, '
              f'speed max {spd.max():.2f} m/s, start v_z {float(d1(0.0)[2]):.2f} m/s, |a_ref| max {acc.max():.3f} m/s^2, |a_z,ref| max {accz.max():.3f} m/s^2')
    w = 4 * np.pi / T_TOTAL
    wpc = waypoints(A_Z, 0.0)
    print(f'  Card formula: max |z - (50 + y/2)| = {np.abs(wpc[:, 2] - (Z_C + wpc[:, 1] / 2)).max():.2e} m '
          f'(planar, tilt {np.degrees(np.arctan(A_Z / AMP_Y)):.1f} deg); arc: max |z - (40 + x^2/20)| = '
          f'{np.abs(waypoints(A_Z, -np.pi / 2)[:, 2] - (Z_LOW + waypoints(A_Z, -np.pi / 2)[:, 0] ** 2 / 20)).max():.2e} m')
    print('  Audience legibility (R_aud = 100 m, 36 observers, 199 segments): mean min angle (deg), '
          'segments with min angle < 5 deg')
    for p, (a, ph) in paths.items():
        m, nlow = legibility(waypoints(a, ph), per_segment=True)
        print(f'    {PLAB[p]:<30} {m.mean():6.2f} deg  (min segment {m.min():.2f} deg, {nlow} segments < 5 deg)')
    a_g, p_g, s_g = oi['grid_best']
    a_o, p_o, s_o = oi['ga']
    a_c, p_c, s_c = oi['ga_card']
    print(f'    grid best ({len(AZ_GRID)} x {len(PHI_GRID)}): A_z = {a_g:.2f} m, phi = {np.degrees(p_g):.0f} deg, '
          f'{s_g:.2f} deg; gradient ascent from it: A_z = {a_o:.2f} m, phi = {np.degrees(p_o):.1f} deg, {s_o:.2f} deg')
    print(f'    gradient ascent from the card start (10 m, 0 deg): A_z = {a_c:.2f} m, phi = '
          f'{np.degrees(p_c):.1f} deg, {s_c:.2f} deg')
    for ph in (0.0, 90.0, 180.0, 270.0):
        print(f'    probe A_z = 10 m, phi = {ph:.0f} deg: {legibility(waypoints(A_Z, np.radians(ph))):.2f} deg'
              + ('  (plane z = 50 + y/2, high side toward the audience)' if ph == 0 else '')
              + ('  (plane z = 50 - y/2, high side away from the audience)' if ph == 180 else ''))
    print(f"    gain over the flat 2D path: card {raud[R_AUD]['card'] - raud[R_AUD]['flat']:+.2f} deg, "
          f"arc {raud[R_AUD]['arc'] - raud[R_AUD]['flat']:+.2f} deg, opt {raud[R_AUD]['opt'] - raud[R_AUD]['flat']:+.2f} deg")
    print('  Audience-radius sweep (score of each path; best grid point for that radius):')
    for r in R_AUD_SWEEP:
        b = raud_best[r]
        print(f'    R_aud = {r:5.0f} m: ' + ', '.join(f'{p} {raud[r][p]:.2f}' for p in paths) +
              f' deg; grid best A_z = {b[0]:.2f} m, phi = {np.degrees(b[1]):.0f} deg, {b[2]:.2f} deg; '
              f"card - flat {raud[r]['card'] - raud[r]['flat']:+.2f} deg, arc - flat {raud[r]['arc'] - raud[r]['flat']:+.2f} deg")
    print(f'  Tracking, {N_MC} wind seeds (sigma_w = {SIGMA_W} m/s^2), same wind for every path/controller')
    print(f"  {'Path':<30}{'Controller':<24}{'RMS full (cm)':>16}{'Max full (cm)':>17}{'RMS >5 s (cm)':>16}"
          f"{'z / xy RMS >5 s (cm)':>22}{'pass 15 / 10 cm':>17}{'LED RMS err':>13}")
    for p in paths:
        for c in CTRL:
            r = res[(p, c)]
            print(f"  {PLAB[p]:<30}{CLAB[c]:<24}{ms(r['rms']):>16}{ms(r['max']):>17}{ms(r['rms_s']):>16}"
                  f"{100 * r['rms_z_s'].mean():>11.2f} / {100 * r['rms_xy_s'].mean():5.2f}"
                  f"{int(np.sum(r['rms'] <= RMS_SPEC_3D)):>9}/{int(np.sum(r['rms'] <= RMS_SPEC_2D))}"
                  f"{r['led_err'].mean():>13.4f}")
    print('  Per-seed comparison with the flat 2D path (same controller, same wind):')
    for p in ['card', 'arc', 'opt']:
        for c in CTRL:
            a, b = res[(p, c)], res[('flat', c)]
            d = a['rms'] - b['rms']
            print(f"    {p:<5} {CLAB[c]:<24} RMS higher than flat in {int(np.sum(d > 0))}/{N_MC} seeds, "
                  f"mean diff {100 * d.mean():+.2e} cm; after 5 s {100 * (a['rms_s'] - b['rms_s']).mean():+.2e} cm; "
                  f"max |error diff| over all steps {100 * np.abs(a['err'] - b['err']).max():.2e} cm")
    print('  Wind-free PID-only lag probe (RMS after 5 s, cm) vs A_z; analytic vertical lag amplitude '
          '|1 - H(jw)| A_z at w = 4 pi / 60:')
    s_ = 1j * w
    Hc = (KD * s_ ** 2 + KP * s_ + KI) / (s_ ** 3 + KD * s_ ** 2 + KP * s_ + KI)
    for a in AZ_SWEEP:
        print(f"    A_z = {a:4.1f} m: card phase {100 * lag[(a, 0.0)]['rms_s'][0]:.2f} cm "
              f"(z part {100 * lag[(a, 0.0)]['rms_z_s'][0]:.2f}), arc phase {100 * lag[(a, -np.pi / 2)]['rms_s'][0]:.2f} cm "
              f"(z part {100 * lag[(a, -np.pi / 2)]['rms_z_s'][0]:.2f}); analytic z lag amplitude "
              f"{100 * abs(1 - Hc) * a:.2f} cm (RMS {100 * abs(1 - Hc) * a / np.sqrt(2):.2f} cm)")
    zmin = min(r['zmin'] for r in res.values())
    zmax = max(r['zmax'] for r in res.values())
    print(f'  Altitude over all runs: {zmin:.2f} - {zmax:.2f} m (soft bounds {Z_SOFT[0]:.0f}-{Z_SOFT[1]:.0f} m, '
          f'never reached; asserted)')
    print(f"  Batch check: seeds 5 and 2 re-run alone differ from the 100-seed batch by at most "
          f"{oi['batch_diff']:.1e} m (odeint step control is shared across the batch)")
    print('  Peak commanded acceleration (m/s^2): ' + ', '.join(
        f"{p}/{c} {res[(p, c)]['a_peak']:.2f}" for p in paths for c in CTRL))

    os.makedirs(OUTPUT_DIR, exist_ok=True)
    plot_trajectory_3d(paths, refs, res, OUTPUT_DIR)
    plot_altitude_led(res, OUTPUT_DIR)
    plot_error(res, OUTPUT_DIR)
    plot_legibility(paths, oi, raud, OUTPUT_DIR)
    plot_monte_carlo(res, OUTPUT_DIR)
    save_animation(res, OUTPUT_DIR)
