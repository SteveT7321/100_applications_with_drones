"""
S072 Oil Pipeline Leak Detection
================================
A drone patrols a 200 m piecewise-linear pipeline at 3 m height and 2 m/s, tracking it
with three Frenet-frame PID loops on noisy GPS. A leak on the pipeline emits a chemical
tracer as discrete Gaussian puffs (1 g/s) advected by a steady wind. When the sensor
reading exceeds the threshold the drone switches to localisation. Three strategies are
compared on the same random numbers (common random numbers):

  1. Patrol-only        - finish the pass; estimate = position of the highest reading.
  2. Gradient ascent    - descend to 1 m and climb the finite-difference xy gradient.
  3. 3D inward spiral   - descend to 1 m while flying shrinking circles that re-centre
                          on the best reading of each turn.

Monte Carlo: 30 trials with random leak positions, in the card's steady plume and in a
turbulent plume (random per-puff wind perturbation).

Usage:
    conda activate drones
    python src/04_industrial_agriculture/s072_pipeline_leak.py
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401
from scipy.stats import norm

# ── Pipeline geometry (card) ──────────────────────────────────────────────────
PIPELINE_VERTICES = np.array([
    [0.0, 0.0, 0.0],
    [50.0, 5.0, 0.0],
    [100.0, -3.0, 0.0],
    [150.0, 4.0, 0.0],
    [200.0, 0.0, 0.0],
])
LEAK_ARC_LENGTH = 117.0   # m, nominal leak position along the pipeline (card)

# ── Parameters (card) ─────────────────────────────────────────────────────────
GPS_ERROR     = 0.1       # m, GPS noise std (each axis)
PATROL_SPEED  = 2.0       # m/s
PATROL_HEIGHT = 3.0       # m
SAMPLE_DT     = 0.5       # s, sensor sampling interval
DT            = 0.05      # s, simulation timestep
Q_PUFF        = 0.5       # g per puff
EMIT_RATE     = 1.0       # g/s  (= Q_PUFF / PUFF_DT)
D_DIFFUSE     = 0.5       # m^2/s
WIND_VEL      = np.array([0.5, 0.2, 0.0])   # m/s
PUFF_DT       = 0.5       # s
SIGMA_SENSOR  = 5e-4      # g/m^3 sensor noise std
C_THRESH      = 0.01      # g/m^3 detection threshold
V_ASCENT      = 1.0       # m/s
V_DESCENT     = 0.1       # m/s
Z_MIN         = 0.5       # m, absolute floor (asserted)
GRAD_DELTA    = 0.5       # m, finite-difference probe offset
T_MAX_LOC     = 120.0     # s, localisation time budget
EPS_TARGET    = 2.0       # m, localisation accuracy target
KP_T          = 1.2       # along-track speed gain (1/s)
KP_N, KD_N, KI_N = 2.0, 0.8, 0.1
KP_B, KD_B    = 2.5, 1.0
N_MC          = 30        # Monte Carlo trials per plume condition
NOMINAL_SEED  = 42

# ── Added parameters (see README Deviations) ──────────────────────────────────
Z_LOC         = 1.0       # m, localisation height (card strategies 2 and 3)
T_PRE         = 60.0      # s, leak already running when the patrol starts
SIGMA_TURB    = 0.3       # m/s, per-puff wind perturbation in the turbulent plume
R0_SPIRAL     = 5.0       # m, first spiral radius
SHRINK        = 0.6       # radius ratio per spiral turn
R_MIN_SPIRAL  = 0.5       # m, spiral ends below this radius
LEAK_S_RANGE  = (20.0, 180.0)   # m, Monte Carlo leak arc-length range
MC_SEED0      = 2000
V_LIMIT       = 3.0       # m/s, asserted speed bound
N_ANIM_FRAMES = 150

STRATS = ['patrol', 'gradient', 'spiral']
LABELS = {'patrol': 'Patrol-only', 'gradient': 'Patrol + gradient ascent (xy)',
          'spiral': 'Patrol + 3D spiral'}
COLORS = {'patrol': 'tab:gray', 'gradient': 'tab:orange', 'spiral': 'tab:purple'}
CONDS = ['steady', 'turbulent']

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '04_industrial_agriculture', 's072_pipeline_leak',
)

# ── Pipeline helpers ──────────────────────────────────────────────────────────
SEG_P0 = PIPELINE_VERTICES[:-1]
SEG_VEC = np.diff(PIPELINE_VERTICES, axis=0)
SEG_LEN = np.linalg.norm(SEG_VEC, axis=1)
SEG_T = SEG_VEC / SEG_LEN[:, None]
SEG_N = np.cross(SEG_T, np.array([0.0, 0.0, 1.0]))
SEG_N /= np.linalg.norm(SEG_N, axis=1)[:, None]
SEG_CUM = np.concatenate([[0.0], np.cumsum(SEG_LEN)])
TOTAL_ARC = SEG_CUM[-1]


def arc_to_world(s):
    i = int(np.clip(np.searchsorted(SEG_CUM, s, side='right') - 1, 0, len(SEG_LEN) - 1))
    return SEG_P0[i] + (s - SEG_CUM[i]) * SEG_T[i]


def project_to_pipeline(p):
    """Nearest point on the polyline (xy only, pipeline is at z = 0)."""
    v = p[None, :2] - SEG_P0[:, :2]
    tp = np.clip(np.einsum('ij,ij->i', v, SEG_T[:, :2]), 0.0, SEG_LEN)
    ref = SEG_P0[:, :2] + tp[:, None] * SEG_T[:, :2]
    d = np.linalg.norm(p[None, :2] - ref, axis=1)
    i = int(np.argmin(d))
    return SEG_CUM[i] + tp[i], np.array([*ref[i], 0.0]), i


# ── Plume ─────────────────────────────────────────────────────────────────────
class Plume:
    """Discrete puffs released every PUFF_DT from t = -T_PRE, each advected by the wind
    plus an optional constant per-puff perturbation (turbulent condition)."""

    def __init__(self, leak, du):
        self.leak = leak
        n = len(du)
        self.t_rel = -T_PRE + np.arange(n) * PUFF_DT
        self.vel = WIND_VEL[None, :] + du

    def conc(self, pts, t):
        pts = np.atleast_2d(pts)
        m = self.t_rel < t - 1e-9
        age = t - self.t_rel[m]
        cen = self.leak[None, :] + self.vel[m] * age[:, None]
        r2 = ((pts[:, None, :] - cen[None, :, :]) ** 2).sum(axis=2)
        c = (Q_PUFF / (4 * np.pi * D_DIFFUSE * age) ** 1.5)[None, :] * \
            np.exp(-r2 / (4 * D_DIFFUSE * age)[None, :])
        return c.sum(axis=1)


def draw_randoms(seed, turbulent):
    rng = np.random.default_rng(seed)
    n_puff = int((T_PRE + 400.0) / PUFF_DT) + 1
    du = rng.normal(0.0, SIGMA_TURB, (n_puff, 2))
    du = np.hstack([du, np.zeros((n_puff, 1))]) if turbulent else np.zeros((n_puff, 3))
    sens = rng.normal(0.0, SIGMA_SENSOR, (2000, 5))       # [sample, probe]
    gps = rng.normal(0.0, GPS_ERROR, (8000, 3))           # per timestep
    s_leak = rng.uniform(*LEAK_S_RANGE)
    return dict(du=du, sens=sens, gps=gps, s_leak=s_leak)


# ── Mission ───────────────────────────────────────────────────────────────────
def simulate(strategy, leak_s, rnd):
    leak = arc_to_world(leak_s)
    plume = Plume(leak, rnd['du'])
    pos = PIPELINE_VERTICES[0] + np.array([0.0, 0.0, PATROL_HEIGHT])
    vel = np.zeros(3)
    i_n = 0.0
    t, k_step, k_samp = 0.0, 0, 0
    mode = 'patrol'
    t_det, p_det = None, None
    traj, modes, samples = [pos.copy()], ['patrol'], []   # samples: (t, c_meas, c_true, p_meas)
    best_c, best_p = -np.inf, None
    v_hold = np.zeros(3)
    sp = None
    n_steps_per_sample = int(round(SAMPLE_DT / DT))
    while True:
        p_meas = pos + rnd['gps'][k_step]
        # ── sensor sample ────────────────────────────────────────────────────
        if k_step % n_steps_per_sample == 0:
            if mode == 'gradient':
                probes = pos[None, :] + np.array([[0, 0, 0], [GRAD_DELTA, 0, 0],
                                                  [-GRAD_DELTA, 0, 0], [0, GRAD_DELTA, 0],
                                                  [0, -GRAD_DELTA, 0]])
            else:
                probes = pos[None, :]
            c_true = plume.conc(probes, t)
            c_meas = np.maximum(c_true + rnd['sens'][k_samp, :len(probes)], 0.0)
            samples.append((t, c_meas[0], c_true[0], p_meas.copy(), mode))
            k_samp += 1
            if mode == 'patrol' and t_det is None and c_meas[0] > C_THRESH:
                t_det, p_det = t, p_meas.copy()
                if strategy != 'patrol':
                    mode = strategy
                    if strategy == 'spiral':
                        sp = dict(c=p_meas[:2].copy(), r0=R0_SPIRAL, th=0.0, phase='transit',
                                  best_c=c_meas[0], best_p=p_meas[:2].copy(), n_turn=0,
                                  done=False)
            if mode == 'gradient' and len(c_meas) == 5:
                g = np.array([(c_meas[1] - c_meas[2]) / (2 * GRAD_DELTA),
                              (c_meas[3] - c_meas[4]) / (2 * GRAD_DELTA), 0.0])
                gn = np.linalg.norm(g)
                v_hold = V_ASCENT * g / gn if gn > 0 else np.zeros(3)
            if mode == 'spiral' and c_meas[0] > sp['best_c']:
                sp['best_c'], sp['best_p'] = c_meas[0], p_meas[:2].copy()

        # ── motion ───────────────────────────────────────────────────────────
        if mode == 'patrol':
            s, p_ref, i = project_to_pipeline(p_meas)
            T, N = SEG_T[i], SEG_N[i]
            diff = p_meas - (p_ref + np.array([0.0, 0.0, PATROL_HEIGHT]))
            e_n, e_b = diff @ N, diff[2]
            i_n += e_n * DT
            a_t = KP_T * (PATROL_SPEED - vel @ T)
            a_n = -(KP_N * e_n + KD_N * (vel @ N) + KI_N * i_n)
            a_b = -(KP_B * e_b + KD_B * vel[2])
            vel = vel + (a_t * T + a_n * N + a_b * np.array([0.0, 0.0, 1.0])) * DT
            pos = pos + vel * DT
            if s >= TOTAL_ARC - 0.5:
                break
        else:
            vz = -V_DESCENT if pos[2] > Z_LOC + 1e-9 else 0.0
            if mode == 'gradient':
                v_xy = v_hold.copy()
            else:
                v_xy = spiral_velocity(sp, pos)
            vel = np.array([v_xy[0], v_xy[1], vz])
            pos = pos + vel * DT
            pos[2] = max(pos[2], Z_LOC)
            if t - t_det >= T_MAX_LOC - 1e-9:
                break
        assert np.linalg.norm(vel) <= V_LIMIT, 'speed bound exceeded'
        assert pos[2] >= Z_MIN - 1e-9, 'below minimum altitude'
        assert np.linalg.norm(pos - traj[-1]) <= V_LIMIT * DT + 1e-9, 'teleport'
        t += DT
        k_step += 1
        traj.append(pos.copy())
        modes.append(mode)
        if t > 400.0:
            break

    # Leak estimate: measured position at the highest reading (patrol-only: whole patrol;
    # localising strategies: readings from detection on). Running error for convergence.
    t_from = 0.0 if strategy == 'patrol' else (t_det if t_det is not None else np.inf)
    best_c, best_p, run_err = -np.inf, None, []
    for (ts, cm, ct, pm, md) in samples:
        if ts < t_from - 1e-9:
            continue
        if cm > best_c:
            best_c, best_p = cm, pm
        if t_det is not None and ts >= t_det - 1e-9:
            run_err.append((ts - t_det, np.linalg.norm(best_p[:2] - leak[:2])))
    est_err_h = np.linalg.norm(best_p[:2] - leak[:2]) if best_p is not None else np.nan
    est_err_3d = np.linalg.norm(best_p - leak) if best_p is not None else np.nan
    # false alarm: reading above threshold in clean air (true C below C_thresh / 10);
    # near-threshold: true C below threshold but within 4 sigma of it, reading above it.
    # Counted over this strategy's own readings: all of them for patrol-only, only the
    # localisation readings for the other two (their patrol readings up to and including
    # the detection are identical to patrol-only's and would be counted three times).
    own = [x for x in samples if strategy == 'patrol' or x[4] != 'patrol']
    fa = sum(1 for (ts, cm, ct, pm, md) in own
             if cm > C_THRESH and ct < C_THRESH / 10)
    n_clean = sum(1 for (ts, cm, ct, pm, md) in own if ct < C_THRESH / 10)
    n_near = sum(1 for (ts, cm, ct, pm, md) in own
                 if cm > C_THRESH and C_THRESH - 4 * SIGMA_SENSOR <= ct <= C_THRESH)
    t2 = np.nan
    if len(run_err):
        ok = np.asarray(run_err)[:, 1] <= EPS_TARGET
        if ok[-1]:
            bad = np.where(~ok)[0]
            t2 = run_err[bad[-1] + 1][0] if len(bad) else run_err[0][0]
    return dict(strategy=strategy, leak=leak, leak_s=leak_s, traj=np.array(traj),
                modes=np.array(modes), samples=samples, t_det=t_det, p_det=p_det,
                best_p=best_p, best_c=best_c, err_h=est_err_h, err_3d=est_err_3d,
                t_end=t, run_err=np.array(run_err), false_alarms=fa, plume=plume,
                n_clean=n_clean, n_near=n_near, t_2m=t2,
                n_samples=len(own))


def spiral_velocity(sp, pos):
    """Shrinking-circle search: transit to the turn start, fly one turn whose radius
    shrinks from r0 to SHRINK*r0, re-centre on the best reading, repeat."""
    if sp['done']:
        d = sp['best_p'] - pos[:2]
        dist = np.linalg.norm(d)
        return V_ASCENT * d / dist if dist > V_ASCENT * DT else d / DT
    if sp['phase'] == 'transit':
        start = sp['c'] + np.array([sp['r0'], 0.0])
        d = start - pos[:2]
        dist = np.linalg.norm(d)
        if dist <= V_ASCENT * DT:
            sp['phase'], sp['th'] = 'turn', 0.0
            return d / DT
        return V_ASCENT * d / dist
    r0, r1 = sp['r0'], SHRINK * sp['r0']
    r = r0 - (r0 - r1) * sp['th'] / (2 * np.pi)
    sp['th'] += V_ASCENT * DT / r
    if sp['th'] >= 2 * np.pi:
        sp['n_turn'] += 1
        sp['c'] = sp['best_p'].copy()
        sp['r0'] = r1
        sp['phase'] = 'transit'
        if r1 < R_MIN_SPIRAL:
            sp['done'] = True
        return np.zeros(2)
    r_new = r0 - (r0 - r1) * sp['th'] / (2 * np.pi)
    target = sp['c'] + r_new * np.array([np.cos(sp['th']), np.sin(sp['th'])])
    v = (target - pos[:2]) / DT
    sp_v = np.linalg.norm(v)
    return v if sp_v <= V_LIMIT * 0.99 else v / sp_v * V_LIMIT * 0.99


def run_simulation():
    # Nominal case (card): leak at s = 117 m, seed 42, steady plume
    rnd0 = draw_randoms(NOMINAL_SEED, turbulent=False)
    nominal = {s: simulate(s, LEAK_ARC_LENGTH, rnd0) for s in STRATS}
    rnd0t = draw_randoms(NOMINAL_SEED, turbulent=True)
    nominal_turb = {s: simulate(s, LEAK_ARC_LENGTH, rnd0t) for s in STRATS}
    mc = {}
    for cond in CONDS:
        mc[cond] = {s: [] for s in STRATS}
        for k in range(N_MC):
            rnd = draw_randoms(MC_SEED0 + k, turbulent=(cond == 'turbulent'))
            for s in STRATS:
                r = simulate(s, rnd['s_leak'], rnd)
                mc[cond][s].append(dict(err_h=r['err_h'], err_3d=r['err_3d'], t_det=r['t_det'],
                                        leak_s=r['leak_s'], fa=r['false_alarms'],
                                        n_clean=r['n_clean'], n_near=r['n_near'],
                                        t_2m=r['t_2m'],
                                        run_err=r['run_err'], n_samples=r['n_samples'],
                                        det_dist=(np.linalg.norm(r['p_det'][:2] - r['leak'][:2])
                                                  if r['p_det'] is not None else np.nan),
                                        det_ds=(project_to_pipeline(r['p_det'])[0] - r['leak_s']
                                                if r['p_det'] is not None else np.nan)))
    return nominal, nominal_turb, mc


# ── Plots ─────────────────────────────────────────────────────────────────────
def plot_scene_3d(nominal, out_dir):
    fig = plt.figure(figsize=(16, 7))
    leak = nominal['gradient']['leak']
    P = PIPELINE_VERTICES
    for j, zoom in enumerate([False, True]):
        ax = fig.add_subplot(1, 2, j + 1, projection='3d')
        if zoom:
            lo, hi = leak[0] - 8, leak[0] + 10
            ax.set_xlim(lo, hi)
            ax.set_ylim(leak[1] - 8, leak[1] + 8)
            xs = np.linspace(lo, hi, 50)
            ys = np.interp(xs, P[:, 0], P[:, 1])
            ax.plot(xs, ys, 0 * xs, color='grey', lw=5, label='Pipeline')
        else:
            ax.plot(P[:, 0], P[:, 1], P[:, 2], color='grey', lw=4, label='Pipeline')
            ax.plot(P[:, 0], P[:, 1], P[:, 2] + PATROL_HEIGHT, 'k--', lw=1,
                    label='Nominal patrol path (3 m)')
            ax.set_ylim(-20, 20)
        for s in ['gradient', 'spiral']:
            r = nominal[s]
            tr, md = r['traj'], r['modes'].copy()
            if zoom:
                keep = (tr[:, 0] > leak[0] - 8) & (tr[:, 0] < leak[0] + 10)
            else:
                keep = np.ones(len(tr), dtype=bool)
            pm = (md == 'patrol') & keep
            lm = (md != 'patrol') & keep
            if s == 'gradient':
                ax.plot(tr[pm, 0], tr[pm, 1], tr[pm, 2], color='red', lw=1,
                        label='Patrol (same for both)')
            ax.plot(tr[lm, 0], tr[lm, 1], tr[lm, 2], color=COLORS[s], lw=1.3,
                    label=f'{LABELS[s]}: localisation')
            ax.scatter(*r['best_p'], marker='D', s=50, c=COLORS[s], edgecolors='k',
                       label=f"estimate, horizontal error {r['err_h']:.2f} m")
        ax.scatter(*leak, marker='*', s=250, c='green', edgecolors='k', label='True leak')
        if not zoom:
            w = WIND_VEL / np.linalg.norm(WIND_VEL) * 15
            ax.quiver(20, -15, 6, w[0], w[1], 0, color='tab:blue', arrow_length_ratio=0.2)
            ax.text(20, -15, 7.5, 'wind', color='tab:blue')
            ax.set_box_aspect((4, 1, 0.5))
            ax.set_title(f'Full patrol: leak at s = {LEAK_ARC_LENGTH:.0f} m, steady plume')
            ax.legend(loc='upper left', fontsize=7)
        else:
            ax.set_box_aspect((1.1, 1, 0.6))
            ax.set_title('Zoom on the leak: descent from 3 m to 1 m while localising')
        ax.set_zlim(0, 4)
        ax.set_zticks([0, 1, 2, 3, 4])
        ax.set_xlabel('x (m)')
        ax.set_ylabel('y (m)')
        ax.set_zlabel('z (m)')
        ax.view_init(elev=25, azim=-65)
    plt.tight_layout()
    path = os.path.join(out_dir, 'scene_3d.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_concentration(nominal, out_dir):
    fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    for ax, s in zip(axes, STRATS):
        r = nominal[s]
        ts = np.array([x[0] for x in r['samples']])
        cm = np.array([x[1] for x in r['samples']])
        ax.axhspan(0, 3 * SIGMA_SENSOR, color='lightgrey', alpha=0.6, label='noise floor (3 sigma)')
        ax.plot(ts, cm, color=COLORS[s], lw=1)
        ax.axhline(C_THRESH, color='red', ls='--', lw=1, label='C_thresh')
        if r['t_det'] is not None:
            ax.axvline(r['t_det'], color='k', ls=':', lw=1)
            ax.text(r['t_det'] + 1, C_THRESH * 1.3, f"detect t = {r['t_det']:.1f} s", fontsize=8)
        i = int(np.argmax(np.where(ts >= (r['t_det'] if s != 'patrol' else 0), cm, -1)))
        ax.plot(ts[i], cm[i], '*', color='gold', mec='k', ms=12, label='estimate sample')
        ax.set_yscale('symlog', linthresh=1e-3)
        ax.set_ylabel('C_meas (g/m^3)')
        ax.set_title(LABELS[s], fontsize=10)
        ax.legend(loc='upper left', fontsize=7)
    axes[-1].set_xlabel('time (s)')
    plt.tight_layout()
    path = os.path.join(out_dir, 'concentration_timeseries.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_plan_view(nominal, out_dir, tag='steady'):
    r0 = nominal['gradient']
    leak = r0['leak']
    xs = np.linspace(leak[0] - 6, leak[0] + 10, 161)
    ys = np.linspace(leak[1] - 6, leak[1] + 6, 121)
    X, Y = np.meshgrid(xs, ys)
    fig, axes = plt.subplots(1, 2, figsize=(14, 5.5), sharey=True)
    for ax, s in zip(axes, ['gradient', 'spiral']):
        r = nominal[s]
        pts = np.stack([X.ravel(), Y.ravel(), np.full(X.size, Z_LOC)], axis=1)
        C = r['plume'].conc(pts, r['t_det']).reshape(X.shape)
        cf = ax.contourf(X, Y, C, levels=np.linspace(0, max(C.max(), 1e-6), 15), cmap='YlOrBr')
        plt.colorbar(cf, ax=ax, format='%.3f', shrink=0.8,
                     label=f'C at z = {Z_LOC:.0f} m at t_detect (g/m^3)')
        P = PIPELINE_VERTICES
        ax.plot(P[:, 0], P[:, 1], 'k-', lw=2, label='pipeline')
        tr, md = r['traj'], r['modes']
        ax.plot(tr[md == 'patrol', 0], tr[md == 'patrol', 1], color='red', lw=1, label='patrol')
        ax.plot(tr[md != 'patrol', 0], tr[md != 'patrol', 1], '--', color=COLORS[s], lw=1.2,
                label='localisation')
        ax.plot(*r['p_det'][:2], 'o', color='orange', mec='k', ms=8, label='detection point')
        ax.plot(*leak[:2], 'X', color='green', mec='k', ms=12, label='true leak')
        ax.plot(*r['best_p'][:2], 'D', color='white', mec='k', ms=8,
                label=f"estimate (err {r['err_h']:.2f} m)")
        ax.set_xlim(xs[0], xs[-1])
        ax.set_ylim(ys[0], ys[-1])
        ax.set_aspect('equal')
        ax.set_xlabel('x (m)')
        ax.set_title(f'{LABELS[s]} ({tag} plume)')
        ax.legend(loc='upper left', fontsize=7)
    axes[0].set_ylabel('y (m)')
    plt.tight_layout()
    path = os.path.join(out_dir, f'plan_view_{tag}.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_mc(mc, out_dir):
    fig, axes = plt.subplots(2, 2, figsize=(13, 9))
    bins = np.linspace(0, 5, 21)
    for j, cond in enumerate(CONDS):
        ax = axes[0, j]
        for s in STRATS:
            e = np.array([x['err_h'] for x in mc[cond][s]])
            ax.hist(np.clip(e, 0, 4.99), bins=bins, alpha=0.5, color=COLORS[s],
                    label=f'{LABELS[s]} (mean {e.mean():.2f} m)')
            ax.axvline(e.mean(), color=COLORS[s], ls='--', lw=1.2)
        ax.axvline(EPS_TARGET, color='red', lw=1.5, label='target 2 m')
        ax.set_xlabel('horizontal localisation error (m; last bin = 5 m or more)')
        ax.set_ylabel('trials')
        ax.set_title(f'{cond.capitalize()} plume, {N_MC} trials (CRN)')
        ax.legend(fontsize=7)
        ax = axes[1, j]
        for s in ['gradient', 'spiral']:
            curves = [x['run_err'] for x in mc[cond][s]]
            tg = np.arange(0, T_MAX_LOC + 1e-9, SAMPLE_DT)
            M = np.array([np.interp(tg, c[:, 0], c[:, 1]) for c in curves])
            ax.plot(tg, np.median(M, axis=0), color=COLORS[s], lw=2, label=f'{LABELS[s]} median')
            ax.fill_between(tg, np.percentile(M, 25, axis=0), np.percentile(M, 75, axis=0),
                            color=COLORS[s], alpha=0.2)
        ax.axhline(EPS_TARGET, color='red', lw=1, ls='--')
        ax.set_xlabel('time since detection (s)')
        ax.set_ylabel('error of running estimate (m)')
        ax.set_ylim(0, 12)
        ax.set_title(f'{cond.capitalize()} plume: estimate error vs time (median, IQR)')
        ax.legend(fontsize=8)
    plt.tight_layout()
    path = os.path.join(out_dir, 'monte_carlo.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_detection(mc, out_dir):
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.5))
    data = [[x['t_det'] for x in mc[c]['gradient']] for c in CONDS]
    axes[0].boxplot(data, tick_labels=CONDS)
    axes[0].set_ylabel('detection time t_detect (s)')
    axes[0].set_title('Detection time (identical for all strategies, CRN)')
    for c, mk in zip(CONDS, ['o', 's']):
        ls = [x['leak_s'] for x in mc[c]['gradient']]
        dd = [x['det_ds'] for x in mc[c]['gradient']]
        axes[1].plot(ls, dd, mk, label=c, alpha=0.7)
    axes[1].axhline(0, color='k', lw=0.8)
    axes[1].set_xlabel('leak arc-length s_leak (m)')
    axes[1].set_ylabel('detection point - leak, along pipeline (m)')
    axes[1].set_title('Where the threshold is first crossed')
    axes[1].legend(fontsize=8)
    for ax in axes:
        ax.grid(alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, 'detection_stats.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def save_animation(nominal, out_dir):
    """Top-down: full pipeline during patrol, then zoom on the leak while localising
    (gradient-ascent run, steady plume)."""
    r = nominal['gradient']
    rs = nominal['spiral']
    tr, md = r['traj'], r['modes']
    n = len(tr)
    step = int(np.ceil((n - 1) / (N_ANIM_FRAMES - 1)))
    n_frames = int(np.ceil((n - 1) / step)) + 1
    leak = r['leak']
    fig, ax = plt.subplots(figsize=(9, 4.5))
    gx = np.linspace(-5, 205, 211)
    gy = np.linspace(-20, 30, 51)
    zx = np.linspace(leak[0] - 12, leak[0] + 18, 91)
    zy = np.linspace(leak[1] - 8, leak[1] + 10, 55)
    img = ax.imshow(np.zeros((51, 211)), origin='lower', cmap='YlOrBr', vmin=0, vmax=0.05,
                    extent=[gx[0], gx[-1], gy[0], gy[-1]], aspect='auto', alpha=0.8)
    P = PIPELINE_VERTICES
    ax.plot(P[:, 0], P[:, 1], 'k-', lw=2)
    ax.plot(*leak[:2], 'X', color='green', mec='k', ms=12, zorder=5)
    trail, = ax.plot([], [], color='red', lw=1)
    trail_s, = ax.plot([], [], color=COLORS['spiral'], lw=1, alpha=0.8)
    dot, = ax.plot([], [], 'o', color='red', mec='k', ms=7, zorder=6)
    dot_s, = ax.plot([], [], 'o', color=COLORS['spiral'], mec='k', ms=6, zorder=6)
    est, = ax.plot([], [], 'D', color='white', mec='k', ms=8, zorder=6)
    txt = ax.text(0.01, 0.97, '', transform=ax.transAxes, va='top', fontsize=9,
                  bbox=dict(fc='white', alpha=0.8))
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ts = np.array([x[0] for x in r['samples']])
    cm = np.array([x[1] for x in r['samples']])
    pm = np.array([x[3] for x in r['samples']])

    def update(f):
        k = min(f * step, n - 1)
        t = k * DT
        loc = md[k] != 'patrol'
        if loc:
            X, Y = np.meshgrid(zx, zy)
            ext = [zx[0], zx[-1], zy[0], zy[-1]]
        else:
            X, Y = np.meshgrid(gx, gy)
            ext = [gx[0], gx[-1], gy[0], gy[-1]]
        z = tr[k, 2]
        C = r['plume'].conc(np.stack([X.ravel(), Y.ravel(), np.full(X.size, z)], 1), t)
        img.set_data(C.reshape(X.shape))
        img.set_extent(ext)
        ax.set_xlim(ext[0], ext[1])
        ax.set_ylim(ext[2], ext[3])
        trail.set_data(tr[:k + 1, 0], tr[:k + 1, 1])
        dot.set_data([tr[k, 0]], [tr[k, 1]])
        ks = min(k, len(rs['traj']) - 1)
        if loc:
            trail_s.set_data(rs['traj'][:ks + 1, 0], rs['traj'][:ks + 1, 1])
            dot_s.set_data([rs['traj'][ks, 0]], [rs['traj'][ks, 1]])
        else:
            trail_s.set_data([], [])
            dot_s.set_data([], [])
        m = (ts <= t + 1e-9) & (ts >= r['t_det'] - 1e-9)
        if loc and m.any():
            j = np.where(m)[0][np.argmax(cm[m])]
            est.set_data([pm[j, 0]], [pm[j, 1]])
        else:
            est.set_data([], [])
        mode = 'LOCALISE (red: gradient ascent, purple: spiral)' if loc else 'PATROL'
        txt.set_text(f't = {t:5.1f} s | {mode} | z = {z:.2f} m')
        return img, trail, dot, est, txt

    anim = animation.FuncAnimation(fig, update, frames=n_frames, interval=100, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    anim.save(path, writer='pillow', fps=10, dpi=80)
    plt.close(fig)
    print(f'Saved: {path} ({n_frames} frames)')


if __name__ == '__main__':
    nominal, nominal_turb, mc = run_simulation()
    print('\n=== S072 Oil Pipeline Leak Detection — Results ===')
    print(f'  Pipeline arc length: {TOTAL_ARC:.2f} m; patrol {PATROL_SPEED} m/s at {PATROL_HEIGHT} m')
    print(f'  P_FA per sample = 1 - Phi(C_thresh / sigma) = 1 - Phi({C_THRESH / SIGMA_SENSOR:.0f}) '
          f'= {norm.sf(C_THRESH / SIGMA_SENSOR):.2e}')
    for name, nom in [('steady', nominal), ('turbulent', nominal_turb)]:
        g = nom['gradient']
        print(f'  [Nominal leak s = {LEAK_ARC_LENGTH:.0f} m at ({g["leak"][0]:.2f}, '
              f'{g["leak"][1]:.2f}, 0), seed {NOMINAL_SEED}, {name} plume]')
        print(f"    Detection time {g['t_det']:.1f} s at ({g['p_det'][0]:.2f}, {g['p_det'][1]:.2f}, "
              f"{g['p_det'][2]:.2f}) m, {project_to_pipeline(g['p_det'])[0] - LEAK_ARC_LENGTH:+.2f} m "
              f"along the pipeline from the leak")
        for s in STRATS:
            r = nom[s]
            print(f"    {LABELS[s]:<32} err_h {r['err_h']:6.2f} m, err_3d {r['err_3d']:6.2f} m, "
                  f"est ({r['best_p'][0]:.2f}, {r['best_p'][1]:.2f}, {r['best_p'][2]:.2f}), "
                  f"peak C {r['best_c']:.4f}, mission {r['t_end']:.1f} s, "
                  f"false alarms {r['false_alarms']}")
    for cond in CONDS:
        print(f'  [Monte Carlo, {cond} plume, {N_MC} trials, leak s ~ U{LEAK_S_RANGE}, CRN]')
        td = np.array([x['t_det'] if x['t_det'] is not None else np.nan for x in mc[cond]['gradient']])
        dd = np.array([x['det_ds'] for x in mc[cond]['gradient']])
        print(f'    Detected during patrol: {int(np.isfinite(td).sum())}/{N_MC}; '
              f't_detect mean {np.nanmean(td):.1f} s, min {np.nanmin(td):.1f} s, '
              f'max {np.nanmax(td):.1f} s')
        print(f'    Detection point along pipeline relative to leak: mean {np.nanmean(dd):+.2f} m, '
              f'min {np.nanmin(dd):+.2f} m, max {np.nanmax(dd):+.2f} m')
        de = np.array([x['det_dist'] for x in mc[cond]['gradient']])
        print(f'    Detection point as estimate: horizontal error mean {np.nanmean(de):.2f} m, '
              f'max {np.nanmax(de):.2f} m; within 2 m: {int((de <= EPS_TARGET).sum())}/{N_MC}')
        fa = sum(x['fa'] for s in STRATS for x in mc[cond][s])
        ncl = sum(x['n_clean'] for s in STRATS for x in mc[cond][s])
        nn = sum(x['n_near'] for s in STRATS for x in mc[cond][s])
        ns = sum(x['n_samples'] for s in STRATS for x in mc[cond][s])
        print(f'    False alarms (reading > thresh in clean air, C_true < C_thresh/10): '
              f'{fa} in {ncl} distinct clean-air readings ({ns} distinct readings in total; '
              f'shared patrol readings counted once)')
        print(f'    Noise-pushed crossings (C_true within 4 sigma below thresh): {nn}')
        for s in STRATS:
            e = np.array([x['err_h'] for x in mc[cond][s]])
            e3 = np.array([x['err_3d'] for x in mc[cond][s]])
            t2 = np.array([x['t_2m'] for x in mc[cond][s]])
            t2s = (f'; time to stay within 2 m: median {np.nanmedian(t2):5.1f} s, '
                   f'max {np.nanmax(t2):5.1f} s' if s != 'patrol' else '')
            print(f'    {LABELS[s]:<32} err_h mean {e.mean():5.2f} ± {e.std():4.2f} m, '
                  f'median {np.median(e):5.2f}, max {e.max():6.2f}; within 2 m: '
                  f'{int((e <= EPS_TARGET).sum()):2d}/{N_MC}; err_3d mean {e3.mean():5.2f} m'
                  + t2s)
        tg = np.array([x['t_2m'] for x in mc[cond]['gradient']])
        tsp = np.array([x['t_2m'] for x in mc[cond]['spiral']])
        print(f'    Paired time to 2 m: gradient faster {int((tg < tsp).sum())}, '
              f'spiral faster {int((tsp < tg).sum())}, ties {int((tsp == tg).sum())}')
        eg = np.array([x['err_h'] for x in mc[cond]['gradient']])
        es = np.array([x['err_h'] for x in mc[cond]['spiral']])
        ep = np.array([x['err_h'] for x in mc[cond]['patrol']])
        print(f'    Paired (gradient vs spiral): gradient better {int((eg < es).sum())}, '
              f'spiral better {int((es < eg).sum())}, ties {int((es == eg).sum())}')
        print(f'    Paired (gradient vs patrol-only): gradient better {int((eg < ep).sum())}, '
              f'patrol better {int((ep < eg).sum())}')
        print(f'    Paired (spiral vs patrol-only): spiral better {int((es < ep).sum())}, '
              f'patrol better {int((ep < es).sum())}')
    out = os.path.normpath(OUTPUT_DIR)
    os.makedirs(out, exist_ok=True)
    plot_scene_3d(nominal, out)
    plot_concentration(nominal, out)
    plot_plan_view(nominal, out, 'steady')
    plot_plan_view(nominal_turb, out, 'turbulent')
    plot_mc(mc, out)
    plot_detection(mc, out)
    save_animation(nominal, out)
