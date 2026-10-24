"""
S079 Offshore Wind Farm Installation Assistance
===============================================
A delivery drone flies from a supply station at (0, 0, 30) m to the top of an installation
mast at (10, 0) m whose deck heaves with the swell, z_p(t) = 20 + 0.8 sin(2 pi t / 6) m.
It must touch down with position error <= 0.15 m and relative vertical speed <= 0.1 m/s
inside a 3 s landing window, under white horizontal gusts (sigma_w = 0.05 m/s per step).
The drone is the card's 10-state linearised quadrotor (position, velocity, roll, pitch and
their rates; thrust and two torques), discretised exactly (zero-order hold) at 50 Hz.
Controllers compared (200 Monte Carlo gust realisations, common random numbers):
  1. Cascaded PID on position error only (unaware of the platform motion)
  2. LQR tracking the current platform state (position + velocity reference)
  3. LQR + swell look-ahead: reference predicted tau = 0.5 s ahead (card's feedforward)
  4. LQR + acceleration feedforward m * zdd_p(t) on thrust (added, model-based)

Usage:
    conda activate drones
    python src/04_industrial_agriculture/s079_offshore_wind.py
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401
from scipy.linalg import solve_continuous_are, solve_discrete_are, expm

# ── Parameters (card) ─────────────────────────────────────────────────────────
G        = 9.81
MASS     = 1.0       # kg
I_XX     = 0.01      # kg m^2
I_YY     = 0.01
A_SWELL  = 0.8       # m
T_SWELL  = 6.0       # s
Z_MAST   = 20.0      # m
X_TARGET, Y_TARGET = 10.0, 0.0
START    = np.array([0.0, 0.0, 30.0])
R_LAND   = 0.15      # m
V_LAND   = 0.10      # m/s
T_WINDOW = 3.0       # s
TAU_PRED = 0.5       # s   look-ahead of the card's feedforward
SIGMA_W  = 0.05      # m/s horizontal gust added to velocity each step
Q_DIAG   = np.array([20., 20., 50., 3., 3., 8., 5., 5., 1., 1.])
R_DIAG   = np.array([0.1, 1.0, 1.0])
DT       = 0.02      # s
T_MAX    = 40.0      # s
N_TRIALS = 200
ANGLE_MAX = 0.4      # rad (card's attitude clip)

# ── Added parameters ──────────────────────────────────────────────────────────
ARM_RADIUS  = 1.0    # m  landing logic is armed once the drone is this close to the target
GLIDE       = 1.0    # reference height above the deck per metre of horizontal distance
INTEG_MAX   = 0.5    # m s  PID integrator clamp (anti-windup)
DECK_RADIUS = 1.0    # m  deck footprint: below z_p - r_land inside it = deck strike
THRUST_MAX  = 2.0    # x m g, total thrust range [0, 2 m g]
PID_KP, PID_KI, PID_KD = 22.36, 2.0, 11.17   # outer loop (kp, kd = LQR z-loop gains)
ATT_KP, ATT_KD = 400.0, 40.0                  # inner attitude PD (1/s^2, 1/s): 20 rad/s, zeta 1
T_AVAIL0    = 15.0   # s  start of the landing-availability measurement (track-only runs)
TAU_SWEEP   = [0.0, 0.05, 0.1, 0.2, 0.3, 0.5]
N_SWEEP     = 100
SEED        = 0
ANIM_STEP   = 10     # simulation steps per GIF frame (0.2 s)

OMEGA = 2 * np.pi / T_SWELL
N_STEPS = int(round(T_MAX / DT))
CTRL = ['pid', 'lqr', 'lqr_look', 'lqr_acc']
LABELS = {'pid': 'PID (no platform knowledge)', 'lqr': 'LQR (current platform state)',
          'lqr_look': f'LQR + {TAU_PRED} s look-ahead (card FF)',
          'lqr_acc': 'LQR + acceleration FF'}
SHORT = {'pid': 'PID', 'lqr': 'LQR', 'lqr_look': f'LQR + {TAU_PRED} s look-ahead',
         'lqr_acc': 'LQR + acc. FF'}
COLORS = {'pid': '#7F7F7F', 'lqr': '#1F77B4', 'lqr_look': '#D62728', 'lqr_acc': '#9467BD'}
PLATFORM_COLOR = '#2CA02C'

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), '..', '..',
                          'outputs', '04_industrial_agriculture', 's079_offshore_wind')


# ── Platform ──────────────────────────────────────────────────────────────────

def platform_z(t):
    return Z_MAST + A_SWELL * np.sin(OMEGA * t)


def platform_zd(t):
    return A_SWELL * OMEGA * np.cos(OMEGA * t)


def platform_zdd(t):
    return -A_SWELL * OMEGA ** 2 * np.sin(OMEGA * t)


# ── Model and gains ───────────────────────────────────────────────────────────

def linear_model():
    A = np.zeros((10, 10)); B = np.zeros((10, 3))
    A[0, 3] = A[1, 4] = A[2, 5] = 1.0
    A[3, 7] = G; A[4, 6] = -G
    A[6, 8] = A[7, 9] = 1.0
    B[5, 0] = 1.0 / MASS; B[8, 1] = 1.0 / I_XX; B[9, 2] = 1.0 / I_YY
    return A, B


def discretise(A, B):
    M = np.zeros((13, 13)); M[:10, :10] = A; M[:10, 10:] = B
    E = expm(M * DT)
    return E[:10, :10], E[:10, 10:]


def gains():
    A, B = linear_model()
    Q, R = np.diag(Q_DIAG), np.diag(R_DIAG)
    Kc = np.linalg.solve(R, B.T @ solve_continuous_are(A, B, Q, R))
    Ad, Bd = discretise(A, B)
    P = solve_discrete_are(Ad, Bd, Q * DT, R * DT)
    Kd = np.linalg.solve(R * DT + Bd.T @ P @ Bd, Bd.T @ P @ Ad)
    rho_c = np.max(np.abs(np.linalg.eigvals(Ad - Bd @ Kc)))
    rho_euler = np.max(np.abs(np.linalg.eigvals(np.eye(10) + DT * (A - B @ Kc))))
    rho_d = np.max(np.abs(np.linalg.eigvals(Ad - Bd @ Kd)))
    return Ad, Bd, Kc, Kd, dict(rho_c=rho_c, rho_euler=rho_euler, rho_d=rho_d)


# ── Simulation (vectorised over trials) ───────────────────────────────────────

def simulate(ctrl, seeds, tau=TAU_PRED, keep=False, land=True):
    """land=False: track the deck for the whole horizon (no touchdown, no strike check) to
    measure how often all three landing conditions hold after T_AVAIL0."""
    Ad, Bd, _, K, _ = gains()
    n = len(seeds)
    gusts = np.stack([np.random.default_rng(s).normal(0.0, SIGMA_W, (N_STEPS, 2)) for s in seeds],
                     axis=1)                                   # (steps, n, 2), shared by all ctrl
    x = np.zeros((n, 10)); x[:, :3] = START
    integ = np.zeros((n, 3))
    landed = np.zeros(n, bool); crashed = np.zeros(n, bool); armed = np.zeros(n, bool)
    w_open = np.zeros(n, bool); w_start = np.zeros(n); w_count = np.zeros(n, int)
    misses = np.zeros(n, int)
    t_land = np.full(n, np.nan); first_ok = np.zeros(n, bool)
    land_err = np.full((n, 3), np.nan)                         # xy err, z err, |dzdot| at touchdown
    active = np.ones(n, bool)
    avail = np.zeros(n); n_avail = 0
    sq_ez = np.zeros(n); sq_dzd = np.zeros(n)
    log = dict(t=[], x=[], zp=[]) if keep else None
    for k in range(N_STEPS):
        t = k * DT
        zp = platform_z(t)
        # glide slope: approach from above, descend onto the deck as the drone centres over it
        h_off = GLIDE * np.hypot(x[:, 0] - X_TARGET, x[:, 1] - Y_TARGET)
        # --- controller ---
        if ctrl == 'pid':
            ref = np.column_stack([np.full(n, X_TARGET), np.full(n, Y_TARGET), zp + h_off])
            e = ref - x[:, :3]
            integ = np.clip(integ + e * DT * active[:, None], -INTEG_MAX, INTEG_MAX)
            a_cmd = PID_KP * e + PID_KI * integ - PID_KD * x[:, 3:6]
            theta_d = np.clip(a_cmd[:, 0] / G, -ANGLE_MAX, ANGLE_MAX)
            phi_d = np.clip(-a_cmd[:, 1] / G, -ANGLE_MAX, ANGLE_MAX)
            u = np.column_stack([
                MASS * a_cmd[:, 2],
                I_XX * (ATT_KP * (phi_d - x[:, 6]) - ATT_KD * x[:, 8]),
                I_YY * (ATT_KP * (theta_d - x[:, 7]) - ATT_KD * x[:, 9])])
        else:
            tr = t + (tau if ctrl == 'lqr_look' else 0.0)
            ref = np.zeros((n, 10))
            ref[:, 0], ref[:, 1] = X_TARGET, Y_TARGET
            ref[:, 2] = platform_z(tr) + h_off
            ref[:, 5] = platform_zd(tr)
            u = -(x - ref) @ K.T
            if ctrl == 'lqr_acc':
                u[:, 0] += MASS * platform_zdd(t)
        u[:, 0] = np.clip(u[:, 0], -MASS * G, (THRUST_MAX - 1.0) * MASS * G)
        # --- plant (ZOH-exact linear model, deviation thrust) + gusts ---
        x_new = x @ Ad.T + u @ Bd.T
        x_new[:, 3:5] += gusts[k]
        x_new[:, 6:8] = np.clip(x_new[:, 6:8], -ANGLE_MAX, ANGLE_MAX)
        x = np.where(active[:, None], x_new, x)
        t1 = t + DT
        zp1, zdp1 = platform_z(t1), platform_zd(t1)
        exy = np.hypot(x[:, 0] - X_TARGET, x[:, 1] - Y_TARGET)
        ez = np.abs(x[:, 2] - zp1)
        dzd = np.abs(x[:, 5] - zdp1)
        assert np.all(x[:, 2] > 0.0), 'drone in the sea'
        if not land:
            if t1 >= T_AVAIL0 - 1e-9:
                avail += (exy <= R_LAND) & (ez <= R_LAND) & (dzd <= V_LAND)
                sq_ez += ez ** 2; sq_dzd += dzd ** 2
                n_avail += 1
            if keep:
                log['t'].append(t1); log['x'].append(x.copy()); log['zp'].append(zp1)
            continue
        # --- deck strike ---
        strike = active & (exy <= DECK_RADIUS) & (x[:, 2] < zp1 - R_LAND)
        crashed |= strike
        active &= ~strike
        # --- landing window logic ---
        armed |= active & (np.sqrt(exy ** 2 + ez ** 2) <= ARM_RADIUS)
        opening = active & armed & ~w_open & (dzd <= V_LAND)
        w_open |= opening
        w_start = np.where(opening, t1, w_start)
        w_count += opening
        ok = active & w_open & (exy <= R_LAND) & (ez <= R_LAND) & (dzd <= V_LAND)
        land_err[ok] = np.column_stack([exy, ez, dzd])[ok]
        t_land[ok] = t1
        first_ok |= ok & (w_count == 1)
        landed |= ok
        active &= ~ok
        expire = active & w_open & (t1 - w_start > T_WINDOW)
        misses += expire
        w_open &= ~expire
        if keep:
            log['t'].append(t1); log['x'].append(x.copy()); log['zp'].append(zp1)
        if not active.any():
            break
    if not land:
        res = dict(avail=avail / n_avail, rms_ez=np.sqrt(sq_ez / n_avail),
                   rms_dzd=np.sqrt(sq_dzd / n_avail))
        if keep:
            res['log'] = {k: np.array(v) for k, v in log.items()}
        return res
    res = dict(landed=landed, first=first_ok, crashed=crashed, misses=misses, t_land=t_land,
               land_err=land_err, armed=armed)
    if keep:
        res['log'] = {k: np.array(v) for k, v in log.items()}
    return res


def run_simulation():
    seeds = list(range(N_TRIALS))
    single = {c: simulate(c, [SEED], keep=True) for c in CTRL}
    mc = {c: simulate(c, seeds) for c in CTRL}
    track = {c: simulate(c, seeds, land=False) for c in CTRL}
    track_single = {c: simulate(c, [SEED], land=False, keep=True) for c in CTRL}
    sweep = [simulate('lqr_look', list(range(N_SWEEP)), tau=tau) for tau in TAU_SWEEP]
    return single, mc, sweep, track, track_single


def steady_state_errors(K):
    """Analytic z and vz tracking-error amplitudes of the LQR z loop under the swell."""
    kz, kv = K[0, 2] / MASS, K[0, 5] / MASS
    s = 1j * OMEGA
    out = {}
    for tau in TAU_SWEEP:
        H = (kz + kv * s) / (s ** 2 + kv * s + kz) * np.exp(s * tau)
        out[tau] = A_SWELL * abs(H - 1)
    Hp = PID_KP / (s ** 2 + PID_KD * s + PID_KP + PID_KI / s) * (1 + PID_KI / (PID_KP * s))
    out['pid'] = A_SWELL * abs(Hp - 1)
    return out


# ── Plotting ──────────────────────────────────────────────────────────────────

def _save(fig, out_dir, name, dpi=140):
    path = os.path.join(out_dir, name)
    fig.savefig(path, dpi=dpi, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_trajectory_3d(single, out_dir):
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    for c in CTRL:
        X = single[c]['log']['x'][:, 0]
        ax.plot(X[:, 0], X[:, 1], X[:, 2], color=COLORS[c], lw=1.4, label=LABELS[c])
        ax.scatter(*X[-1, :3], color=COLORS[c], s=60, marker='x' if single[c]['crashed'][0]
                   else 'o', depthshade=False)
    ax.plot([X_TARGET] * 2, [Y_TARGET] * 2, [0, Z_MAST - A_SWELL], color='grey', lw=5,
            label='Mast')
    tt = np.linspace(0, T_MAX, 400)
    ax.scatter(np.full_like(tt, X_TARGET), np.full_like(tt, Y_TARGET), platform_z(tt),
               color=PLATFORM_COLOR, s=3, alpha=0.5, label='Deck heave range')
    ax.scatter(*START, color='k', marker='*', s=150, label='Supply station')
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)'); ax.set_zlabel('z (m)')
    ax.set_ylim(-5, 5); ax.set_zlim(0, 32)
    ax.view_init(elev=15, azim=-70)
    ax.set_title(f'S079 Approach Trajectories (gust seed {SEED}); × = deck strike, o = landed')
    ax.legend(fontsize=7, loc='upper left')
    _save(fig, out_dir, 'trajectory_3d.png')


def plot_vertical_tracking(single, landing, out_dir):
    fig, (a1, a2, a3) = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    tt = np.arange(0, T_MAX, DT)
    a1.fill_between(tt, platform_z(tt) - R_LAND, platform_z(tt) + R_LAND, color=PLATFORM_COLOR,
                    alpha=0.2, label='deck ± $r_{land}$')
    a1.plot(tt, platform_z(tt), color=PLATFORM_COLOR, lw=1)
    a2.axhspan(-V_LAND, V_LAND, color=PLATFORM_COLOR, alpha=0.2, label='± $v_{land,max}$')
    a3.axhline(R_LAND, color='k', ls='--', lw=1, label='$r_{land}$')
    for c in CTRL:
        L = single[c]['log']
        X = L['x'][:, 0]
        a1.plot(L['t'], X[:, 2], color=COLORS[c], lw=1.2, label=LABELS[c])
        a2.plot(L['t'], X[:, 5] - platform_zd(L['t']), color=COLORS[c], lw=1)
        a3.plot(L['t'], np.hypot(X[:, 0] - X_TARGET, X[:, 1] - Y_TARGET), color=COLORS[c], lw=1)
        if not np.isnan(landing[c]['t_land'][0]):
            for a in (a1, a2, a3):
                a.axvline(landing[c]['t_land'][0], color=COLORS[c], ls=':', lw=1)
        if landing[c]['crashed'][0]:
            ts = landing[c]['log']['t'][-1]
            a1.plot(ts, landing[c]['log']['x'][-1, 0, 2], 'x', color=COLORS[c], ms=12, mew=3)
    a1.set_ylim(18, 23); a1.set_ylabel('z (m)')
    a2.set_ylim(-1.2, 1.2); a2.set_ylabel('$\\dot z - \\dot z_p$ (m/s)')
    a3.set_ylim(0, 1.0); a3.set_ylabel('xy error (m)'); a3.set_xlabel('Time (s); dotted = landing')
    a1.legend(fontsize=7, ncol=3, loc='upper right'); a2.legend(fontsize=7); a3.legend(fontsize=7)
    for a in (a1, a2, a3):
        a.grid(alpha=0.3)
    fig.suptitle(f'S079 Tracking of the Heaving Deck, touchdown disabled (gust seed {SEED}); '
                 f'dotted = touchdown time and × = deck strike in the landing run')
    plt.tight_layout()
    _save(fig, out_dir, 'vertical_tracking.png')


def plot_monte_carlo(mc, track, out_dir):
    fig, (a1, a2, a3) = plt.subplots(1, 3, figsize=(19, 5))
    x = np.arange(len(CTRL)); w = 0.27
    for j, (key, lbl, col) in enumerate((('first', 'landed in first window', '#2CA02C'),
                                         ('landed', f'landed within {T_MAX:.0f} s', '#98DF8A'),
                                         ('crashed', 'deck strike', '#D62728'))):
        v = [100 * mc[c][key].mean() for c in CTRL]
        bars = a1.bar(x + (j - 1) * w, v, w, color=col, edgecolor='k', label=lbl)
        for b, vv in zip(bars, v):
            a1.text(b.get_x() + b.get_width() / 2, vv + 1, f'{vv:.1f}', ha='center', fontsize=7)
    a1.axhline(90, color='k', ls='--', lw=1, label='card target 90 %')
    a1.set_xticks(x); a1.set_xticklabels([SHORT[c] for c in CTRL], fontsize=8)
    a1.set_ylabel(f'% of {N_TRIALS} gust realisations'); a1.set_ylim(0, 110)
    a1.legend(fontsize=7, loc='center right')
    a1.set_title('Outcome')
    bins = np.arange(0, 8) - 0.5
    for c in CTRL:
        a2.hist(mc[c]['misses'], bins=bins, histtype='step', lw=2, color=COLORS[c],
                label=LABELS[c], ls='--' if c == 'lqr' else '-')
    a2.set_xlabel('Missed landing windows per trial'); a2.set_ylabel('Trials')
    a2.legend(fontsize=7); a2.set_title('Window misses')
    v = [100 * track[c]['avail'].mean() for c in CTRL]
    sd = [100 * track[c]['avail'].std() for c in CTRL]
    bars = a3.bar(x, v, yerr=sd, capsize=5, color=[COLORS[c] for c in CTRL], edgecolor='k')
    for b, vv in zip(bars, v):
        a3.text(b.get_x() + b.get_width() / 2, vv + 2, f'{vv:.1f}', ha='center', fontsize=8)
    a3.set_xticks(x); a3.set_xticklabels([SHORT[c] for c in CTRL], fontsize=8)
    a3.set_ylabel('% of time all landing conditions hold'); a3.set_ylim(0, 110)
    a3.set_title(f'Landing availability, touchdown disabled, t ≥ {T_AVAIL0:.0f} s')
    fig.suptitle(f'S079 Monte Carlo ({N_TRIALS} gust realisations, same gusts for every controller)')
    plt.tight_layout()
    _save(fig, out_dir, 'monte_carlo.png')


def plot_lookahead_sweep(sweep, ss, out_dir):
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(12, 4.5))
    a1.plot(TAU_SWEEP, [100 * r['first'].mean() for r in sweep], 'o-', color='#2CA02C',
            label='landed in first window')
    a1.plot(TAU_SWEEP, [100 * r['crashed'].mean() for r in sweep], 's-', color='#D62728',
            label='deck strike')
    a1.set_xlabel('Look-ahead τ (s)'); a1.set_ylabel(f'% of {N_SWEEP} trials')
    a1.legend(fontsize=8); a1.grid(alpha=0.3)
    a1.set_title('Simulated outcome vs look-ahead')
    a2.plot(TAU_SWEEP, [ss[t] for t in TAU_SWEEP], 'o-', color='k', label='z error amplitude')
    a2.plot(TAU_SWEEP, [ss[t] * OMEGA for t in TAU_SWEEP], 's--', color='grey',
            label='$\\dot z$ error amplitude (m/s)')
    a2.axhline(R_LAND, color='k', ls=':', lw=1, label='$r_{land}$ = 0.15 m')
    a2.axhline(V_LAND, color='grey', ls=':', lw=1, label='$v_{land}$ = 0.1 m/s')
    a2.set_xlabel('Look-ahead τ (s)'); a2.set_ylabel('Steady-state amplitude')
    a2.legend(fontsize=8); a2.grid(alpha=0.3)
    a2.set_title('Analytic LQR z-loop tracking error')
    fig.suptitle('S079 Swell Look-ahead Sweep')
    plt.tight_layout()
    _save(fig, out_dir, 'lookahead_sweep.png')


def save_animation(single, out_dir):
    fig = plt.figure(figsize=(9, 6.5))
    ax = fig.add_subplot(111, projection='3d')
    n = max(len(single[c]['log']['t']) for c in CTRL)
    n_frames = int(np.ceil((n - 1) / ANIM_STEP)) + 1
    ax.plot([X_TARGET] * 2, [Y_TARGET] * 2, [0, Z_MAST - A_SWELL], color='grey', lw=5)
    deck, = ax.plot([], [], [], color=PLATFORM_COLOR, lw=6)
    tgt, = ax.plot([], [], [], '^', color=PLATFORM_COLOR, ms=10)
    trails = {c: ax.plot([], [], [], color=COLORS[c], lw=1.2, label=LABELS[c])[0] for c in CTRL}
    dots = {c: ax.plot([], [], [], 'o', color=COLORS[c], mec='k')[0] for c in CTRL}
    ax.set_xlim(-1, 12); ax.set_ylim(-3, 3); ax.set_zlim(15, 31)
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)'); ax.set_zlabel('z (m)')
    ax.view_init(elev=5, azim=-90)
    ax.legend(fontsize=7, loc='upper right')
    title = ax.set_title('')

    def update(f):
        i = min(f * ANIM_STEP, n - 1)
        t = (i + 1) * DT
        zp = platform_z(t)
        deck.set_data([X_TARGET - DECK_RADIUS, X_TARGET + DECK_RADIUS], [Y_TARGET] * 2)
        deck.set_3d_properties([zp, zp])
        tgt.set_data([X_TARGET], [Y_TARGET]); tgt.set_3d_properties([zp])
        txt = []
        for c in CTRL:
            L = single[c]['log']
            j = min(i, len(L['t']) - 1)
            X = L['x'][:j + 1, 0]
            trails[c].set_data(X[:, 0], X[:, 1]); trails[c].set_3d_properties(X[:, 2])
            dots[c].set_data([X[-1, 0]], [X[-1, 1]]); dots[c].set_3d_properties([X[-1, 2]])
            if j < i or j == len(L['t']) - 1 and len(L['t']) < N_STEPS:
                state = 'landed' if single[c]['landed'][0] else 'STRIKE'
            else:
                state = f"dz {abs(X[-1, 2] - platform_z(L['t'][j])):.2f} m"
            txt.append(f"{c}: {state}")
        title.set_text(f'S079 t = {t:.1f} s (gust seed {SEED})\n' + ' | '.join(txt))
        return []

    anim = animation.FuncAnimation(fig, update, frames=n_frames, interval=50, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    anim.save(path, writer='pillow', fps=20, dpi=70)
    plt.close(fig)
    print(f'Saved: {path} ({n_frames} frames)')


# ── Main ──────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    single, mc, sweep, track, track_single = run_simulation()
    Ad, Bd, Kc, Kd, rho = gains()
    ss = steady_state_errors(Kd)
    print('=== S079 Offshore Wind Farm Installation Assistance — Results ===')
    print(f'  Spectral radius at 50 Hz: continuous LQR gain with Euler plant '
          f'{rho["rho_euler"]:.4f}, with exact ZOH plant {rho["rho_c"]:.4f}; '
          f'discrete LQR gain {rho["rho_d"]:.4f}')
    print(f'  Continuous LQR z-loop gains kz = {Kc[0, 2]:.2f}, kvz = {Kc[0, 5]:.2f}; '
          f'discrete kz = {Kd[0, 2]:.2f}, kvz = {Kd[0, 5]:.2f}')
    print(f'  Card code thrust without gravity compensation: steady z sag mg/kz = '
          f'{MASS * G / Kc[0, 2]:.3f} m')
    print(f'  Swell: peak deck speed {A_SWELL * OMEGA:.3f} m/s, peak acceleration '
          f'{A_SWELL * OMEGA ** 2:.3f} m/s^2')
    print('  Analytic steady-state z error amplitude of the LQR z loop vs look-ahead tau: ' +
          ', '.join(f'{t:.2f} s -> {ss[t]:.3f} m ({ss[t] * OMEGA:.3f} m/s)' for t in TAU_SWEEP))
    print(f'  Analytic PID z error amplitude {ss["pid"]:.3f} m ({ss["pid"] * OMEGA:.3f} m/s)')
    for c in CTRL:
        r = single[c]
        st = 'LANDED' if r['landed'][0] else ('DECK STRIKE' if r['crashed'][0] else 'no landing')
        extra = ''
        if r['landed'][0]:
            e = r['land_err'][0]
            extra = (f' at t = {r["t_land"][0]:.2f} s, xy err {e[0]:.3f} m, z err {e[1]:.3f} m, '
                     f'|dzdot| {e[2]:.3f} m/s')
        print(f'  Seed {SEED} [{LABELS[c]}]: {st}{extra}; window misses {r["misses"][0]}')
    print(f'  --- Monte Carlo, {N_TRIALS} gust realisations (common random numbers) ---')
    for c in CTRL:
        r = mc[c]
        tl = r['t_land'][r['landed']]
        le = r['land_err'][r['landed']]
        print(f'  [{LABELS[c]}] first window {100 * r["first"].mean():.1f}%, landed by '
              f'{T_MAX:.0f} s {100 * r["landed"].mean():.1f}%, deck strike '
              f'{100 * r["crashed"].mean():.1f}%, armed {100 * r["armed"].mean():.1f}%, misses '
              f'{r["misses"].mean():.2f} ± {r["misses"].std():.2f}' +
              (f', landing time {tl.mean():.2f} ± {tl.std():.2f} s, touchdown xy '
               f'{le[:, 0].mean():.3f} m, z {le[:, 1].mean():.3f} m, |dzdot| '
               f'{le[:, 2].mean():.3f} m/s' if len(tl) else ''))
    print(f'  --- Track-only runs (no touchdown), t >= {T_AVAIL0:.0f} s, {N_TRIALS} trials ---')
    for c in CTRL:
        r = track[c]
        print(f'  [{LABELS[c]}] landing conditions hold {100 * r["avail"].mean():.1f} ± '
              f'{100 * r["avail"].std():.1f}% of the time; RMS z error {r["rms_ez"].mean():.3f} m; '
              f'RMS |dzdot| {r["rms_dzd"].mean():.3f} m/s')
    av_a, av_l = track['lqr_acc']['avail'], track['lqr']['avail']
    print(f'  Paired availability, acceleration FF vs LQR: higher in {int(np.sum(av_a > av_l))}, '
          f'equal in {int(np.sum(av_a == av_l))}, lower in {int(np.sum(av_a < av_l))} of {N_TRIALS}')
    a, b = mc['lqr_acc']['first'], mc['lqr']['first']
    print(f'  Paired first-window outcome, acceleration FF vs LQR: both {int(np.sum(a & b))}, '
          f'only acc-FF {int(np.sum(a & ~b))}, only LQR {int(np.sum(~a & b))}, neither '
          f'{int(np.sum(~a & ~b))}')
    print(f'  Look-ahead sweep ({N_SWEEP} trials): ' + ', '.join(
        f'{t:.2f} s -> first {100 * r["first"].mean():.0f}% / strike {100 * r["crashed"].mean():.0f}%'
        for t, r in zip(TAU_SWEEP, sweep)))

    out = os.path.normpath(OUTPUT_DIR)
    os.makedirs(out, exist_ok=True)
    plot_trajectory_3d(single, out)
    plot_vertical_tracking(track_single, single, out)
    plot_monte_carlo(mc, track, out)
    plot_lookahead_sweep(sweep, ss, out)
    save_animation(single, out)
