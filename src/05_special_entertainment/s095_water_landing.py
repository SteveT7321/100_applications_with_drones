"""
S095 Water Surface Landing Simulation
=====================================
A drone flies from a station at (0, 0, 15) m to a 1 x 1 m boat deck at (8, 0) m that heaves
with a two-frequency swell (0.40 m / 6 s plus 0.15 m / 2.5 s). It hovers at 3.5 m, records
64 deck-height samples at 10 Hz (sigma = 1 cm), and then descends. A touchdown is soft when
the drone is within 0.3 m of the deck centre and the relative vertical speed is <= 0.1 m/s.

Six descent strategies plus one ablation are compared on the same 200 random wave phases and
the same sensor noise (common random numbers):
  1. Naive (card)          - constant 0.4 m/s descent right after the observation window.
  2. Phase-blind flare     - 0.4 m/s cruise, flare to a 0.05 m/s creep just above the highest
                             deck sample seen, started right away (no timing).
  3. FFT bins, absolute    - the card's FFT estimator (64-point rFFT, two largest bins) predicts
                             the next crest; the flare profile is timed to reach it (card ref).
  4. FFT+LS, absolute      - zero-padded FFT peaks initialise a nonlinear least-squares fit of
                             the two sinusoids; same timed absolute profile.
  5. FFT bins, deck-rel.   - card estimator; the flare also follows the predicted deck motion.
  6. FFT+LS, deck-rel.     - least-squares estimator with the deck-relative flare (best).
  Ablation                 - strategy 6 without ground-effect compensation.
Sensitivity runs: sensor noise, and the added parameters TAU_THRUST, V_TD, T_FLARE, Z_ROTOR
(with a strategy ranking at the two stress points TAU_THRUST = 0.1 s and V_TD = 0.07 m/s).

Physics: NumPy point-mass model (no attitude states) with a first-order thrust lag, thrust and
tilt limits, and a Cheeseman-Bennett ground effect from the deck surface. The controller
cancels the ground effect using a downward rangefinder; an ablation turns that off.

Usage:
    conda activate drones
    python src/05_special_entertainment/s095_water_landing.py
"""

import os
import math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from scipy.optimize import least_squares

# ── Parameters (card) ─────────────────────────────────────────────────────────
G          = 9.81      # m/s^2
MASS       = 1.0       # kg
A1, T1     = 0.40, 6.0     # m, s  primary swell
A2, T2     = 0.15, 2.5     # m, s  secondary chop
X_DECK, Y_DECK = 8.0, 0.0  # m
Z_DECK_NOMINAL = 2.0       # m  mean deck height above sea level
DECK_SIZE  = 1.0           # m  deck edge
START      = np.array([0.0, 0.0, 15.0])
Z_HOLD     = Z_DECK_NOMINAL + 1.5   # m  observation hold altitude (3.5 m)
R_LAND     = 0.30      # m    horizontal tolerance
V_LAND_MAX = 0.10      # m/s  relative vertical speed tolerance
V_DESC     = 0.40      # m/s  nominal descent speed
T_SCAN     = 8.0       # s    crest search window (after the earliest feasible arrival)
F_SAMPLE   = 10.0      # Hz   deck sensor rate
N_OBS      = 64        # samples per FFT window (6.4 s)
SIGMA_Z    = 0.01      # m    deck sensor noise
KP_XY, KD_XY = 2.0, 1.2
KP_Z, KD_Z   = 6.0, 3.0
DT         = 0.02      # s
T_MAX      = 60.0      # s
N_TRIALS   = 200

# ── Parameters (added, not in the card) ───────────────────────────────────────
V_TRANSIT  = 2.0       # m/s  speed of the straight-line transit reference
V_TD       = 0.05      # m/s  creep (touchdown) sink rate at the end of the flare
T_FLARE    = 2.0       # s    flare: linear speed ramp from V_DESC to V_TD
TAU_THRUST = 0.05      # s    first-order lag on thrust / horizontal acceleration
TILT_MAX   = 0.4       # rad  -> horizontal acceleration limit g*tan(0.4)
A_XY_MAX   = G * math.tan(TILT_MAX)
F_MAX      = 2.0 * G   # max specific thrust (thrust / mass), card clip 2 m g
R_ROTOR    = 0.12      # m    rotor radius for the ground-effect model
Z_ROTOR    = 0.10      # m    rotor-plane height above the landing-gear contact point
V_LIMIT    = 5.0       # m/s  sanity bound asserted on the speed
N_PAD      = 4096      # zero-padded FFT length used to initialise the LS fit
SEED       = 95
NOISE_SWEEP = [0.005, 0.01, 0.02, 0.05]
STRESS_POINTS = [('TAU_THRUST', 0.1), ('V_TD', 0.07)]
STRESS_STRATEGIES = ['fft_bin', 'fft_ls', 'fft_ls_rel_nogc']   # best strategy is in the sweep
PARAM_SWEEPS = {                         # added-parameter sensitivity of the best strategy
    'TAU_THRUST': [0.02, 0.05, 0.1],
    'V_TD':       [0.02, 0.03, 0.05, 0.07],
    'T_FLARE':    [1.0, 2.0, 3.0],
    'Z_ROTOR':    [0.05, 0.1, 0.2],      # run with and without ground-effect compensation
}
DEMO_PHI   = (0.3, 1.1)    # card's single-run phases

D_FLARE    = T_FLARE * (V_DESC + V_TD) / 2.0      # 0.45 m covered during the flare
OBS_EVERY  = int(round(1.0 / (F_SAMPLE * DT)))    # 5 steps between samples
N_STEPS    = int(round(T_MAX / DT))

STRATEGIES = ['naive', 'blind', 'fft_bin', 'fft_ls', 'fft_bin_rel', 'fft_ls_rel']
ABLATION = 'fft_ls_rel_nogc'      # proposed strategy without ground-effect compensation
LABELS = {
    'naive':       'Naive 0.4 m/s (card)',
    'blind':       'Phase-blind flare',
    'fft_bin':     'FFT bins, absolute ref (card)',
    'fft_ls':      'FFT+LS, absolute ref',
    'fft_bin_rel': 'FFT bins, deck-relative flare',
    'fft_ls_rel':  'FFT+LS, deck-relative flare',
    ABLATION:      'FFT+LS rel., no GE comp.',
}
COLORS = {'naive': '#1f77b4', 'blind': '#9467bd', 'fft_bin': '#ff7f0e', 'fft_ls': '#e377c2',
          'fft_bin_rel': '#8c564b', 'fft_ls_rel': '#d62728', ABLATION: '#7f7f7f'}
STYLES = {'naive': '--', 'blind': '--', 'fft_bin': ':', 'fft_ls': ':', 'fft_bin_rel': '-.',
          'fft_ls_rel': '-'}

OUTPUT_DIR = os.path.normpath(os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '05_special_entertainment', 's095_water_landing'))


# ── Wave kinematics ───────────────────────────────────────────────────────────
W1, W2 = 2 * math.pi / T1, 2 * math.pi / T2


def deck_z(t, phi1, phi2):
    return Z_DECK_NOMINAL + A1 * np.sin(W1 * t + phi1) + A2 * np.sin(W2 * t + phi2)


def deck_zdot(t, phi1, phi2):
    return A1 * W1 * np.cos(W1 * t + phi1) + A2 * W2 * np.cos(W2 * t + phi2)


def true_crests(phi1, phi2):
    """All local maxima of the true deck height in [0, T_MAX] (1 ms resolution)."""
    tt = np.arange(0.0, T_MAX + 1.0, 0.001)
    zd = deck_zdot(tt, phi1, phi2)
    idx = np.where((zd[:-1] > 0) & (zd[1:] <= 0))[0]
    return tt[idx] + 0.001 * zd[idx] / (zd[idx] - zd[idx + 1])


def ground_effect(h_gear):
    """Cheeseman-Bennett thrust ratio T_IGE / T_OGE at rotor height h = h_gear + Z_ROTOR.

    Capped at h = R/2 (ratio 4/3); with Z_ROTOR > R/2 the cap is never reached in flight."""
    h = max(h_gear + Z_ROTOR, 0.5 * R_ROTOR)
    return 1.0 / (1.0 - (R_ROTOR / (4.0 * h)) ** 2)


# ── Wave estimators ───────────────────────────────────────────────────────────

def fft_bin_estimator(t_obs, z_obs):
    """Card estimator: 64-point rFFT, two largest non-DC bins, amplitude 2|Z|/N.

    The phase uses the corrected conversion (see card_phase_check): for
    z = A sin(2 pi f t + phi), angle(Z[k]) = phi_0 - pi/2 at the first sample t_0, so
    phi = angle(Z[k]) + pi/2 - 2 pi f t_0. The mean deck height is the known nominal (card).
    """
    n = len(z_obs)
    Z = np.fft.rfft(z_obs - z_obs.mean())
    f = np.fft.rfftfreq(n, d=1.0 / F_SAMPLE)
    mag = np.abs(Z)
    mag[0] = 0.0
    comps = []
    for k in np.argsort(mag)[::-1][:2]:
        amp = 2.0 * mag[k] / n
        phase = np.angle(Z[k]) + np.pi / 2 - 2 * np.pi * f[k] * t_obs[0]
        comps.append((amp, f[k], phase))

    def z_hat(t):
        return Z_DECK_NOMINAL + sum(a * np.sin(2 * np.pi * fk * t + p) for a, fk, p in comps)
    return z_hat, comps


def _two_sine(x, tr):
    return (x[0] + x[2] * np.sin(2 * np.pi * x[1] * tr) + x[3] * np.cos(2 * np.pi * x[1] * tr)
            + x[5] * np.sin(2 * np.pi * x[4] * tr) + x[6] * np.cos(2 * np.pi * x[4] * tr))


def fft_ls_estimator(t_obs, z_obs):
    """Zero-padded FFT picks two spectral peaks; Levenberg-Marquardt then fits
    offset + two sinusoids (frequencies free) to the 64 samples."""
    tr = t_obs - t_obs[0]
    Z = np.fft.rfft(z_obs - z_obs.mean(), N_PAD)
    f = np.fft.rfftfreq(N_PAD, d=1.0 / F_SAMPLE)
    mag = np.abs(Z)
    pk = np.where((mag[1:-1] > mag[:-2]) & (mag[1:-1] >= mag[2:]))[0] + 1
    pk = pk[np.argsort(mag[pk])[::-1][:2]]
    f0 = np.sort(f[pk]) if len(pk) == 2 else np.array([1.0 / T1, 1.0 / T2])
    M = np.column_stack([np.ones_like(tr)] + [g(2 * np.pi * fi * tr) for fi in f0
                                              for g in (np.sin, np.cos)])
    c = np.linalg.lstsq(M, z_obs, rcond=None)[0]
    x0 = np.array([c[0], f0[0], c[1], c[2], f0[1], c[3], c[4]])
    res = least_squares(lambda x: _two_sine(x, tr) - z_obs, x0, method='lm')
    x = res.x
    t0 = t_obs[0]

    def z_hat(t):
        return _two_sine(x, np.asarray(t) - t0)
    comps = [(math.hypot(x[2], x[3]), x[1]), (math.hypot(x[5], x[6]), x[4])]
    return z_hat, comps


def card_phase_check(t0=10.0):
    """Card formula phase = angle(Z[k]) + 2 pi f t_start vs the corrected conversion, on a
    bin-aligned noise-free sinusoid (f = 1 bin = 0.15625 Hz) observed from t_start = t0."""
    fk = F_SAMPLE / N_OBS
    t_obs = t0 + np.arange(N_OBS) / F_SAMPLE
    z = 0.4 * np.sin(2 * np.pi * fk * t_obs + 0.7)
    Z = np.fft.rfft(z)
    amp = 2 * abs(Z[1]) / N_OBS
    p_card = np.angle(Z[1]) + 2 * np.pi * fk * t_obs[0]
    p_corr = np.angle(Z[1]) + np.pi / 2 - 2 * np.pi * fk * t_obs[0]
    tf = t_obs[-1] + np.linspace(0, 8, 401)
    truth = 0.4 * np.sin(2 * np.pi * fk * tf + 0.7)
    rms = lambda p: float(np.sqrt(np.mean((amp * np.sin(2 * np.pi * fk * tf + p) - truth) ** 2)))
    return amp, rms(p_card), rms(p_corr)


def find_timed_crest(z_hat, t_now):
    """First predicted crest that the timed profile can still reach.

    Returns (t_crest, z_crest, t_start) or None."""
    tt = np.arange(t_now, t_now + 6.0 + T_SCAN, DT / 4)
    zz = z_hat(tt)
    dz = np.diff(zz)
    idx = np.where((dz[:-1] > 0) & (dz[1:] <= 0))[0] + 1
    for i in idx:
        t_c, z_c = tt[i], zz[i]
        t_s = t_c - descent_duration(z_c)
        if t_s >= t_now and z_c < Z_HOLD - D_FLARE:
            return t_c, z_c, t_s
    return None


# ── Descent references ────────────────────────────────────────────────────────

def descent_duration(z_target):
    """Time of the cruise + flare profile from Z_HOLD down to z_target."""
    d = Z_HOLD - z_target
    return (d - D_FLARE) / V_DESC + T_FLARE


def flare_reference(tau, z_target):
    """z, vz, az of the cruise (V_DESC) -> flare (linear to V_TD) -> creep (V_TD) profile."""
    t_cr = (Z_HOLD - z_target - D_FLARE) / V_DESC
    if tau < t_cr:
        return Z_HOLD - V_DESC * tau, -V_DESC, 0.0
    s = tau - t_cr
    if s < T_FLARE:
        dec = (V_DESC - V_TD) / T_FLARE
        return (Z_HOLD - V_DESC * t_cr - (V_DESC * s - 0.5 * dec * s * s),
                -(V_DESC - dec * s), dec)
    return z_target - V_TD * (s - T_FLARE), -V_TD, 0.0


def deck_relative(zr, vzr, azr, t, plan):
    """Blend the predicted deck motion into the flare: z_ref += w(t) (z_hat(t) - z_c).

    w = smoothstep over [t_c - T_FLARE, t_c] and 1 afterwards, so the reference equals the
    absolute profile before the flare and follows the predicted deck (minus the V_TD creep)
    after the predicted crest. Derivatives of z_hat by central differences."""
    t_c, z_c, f = plan['t_crest'], plan['z_target'], plan['z_hat']
    u = (t - (t_c - T_FLARE)) / T_FLARE
    if u <= 0.0:
        return zr, vzr, azr
    if u >= 1.0:
        w, dw, ddw = 1.0, 0.0, 0.0
    else:
        w = 3 * u * u - 2 * u ** 3
        dw = (6 * u - 6 * u * u) / T_FLARE
        ddw = (6 - 12 * u) / T_FLARE ** 2
    e = 1e-3
    zm, z0, zp = (float(x) for x in f(np.array([t - e, t, t + e])))
    dz = (zp - zm) / (2 * e)
    ddz = (zp - 2 * z0 + zm) / e ** 2
    g = z0 - z_c
    return zr + w * g, vzr + dw * g + w * dz, azr + ddw * g + 2 * dw * dz + w * ddz


# ── One landing attempt ───────────────────────────────────────────────────────

def run_simulation(strategy, phi1, phi2, noise, ge_comp=True, record=False):
    """Simulate one landing attempt. noise: N_OBS standard-normal draws scaled to metres."""
    p = START.copy()
    v = np.zeros(3)
    f_act = G                       # specific thrust (hover)
    axy_act = np.zeros(2)
    hold = np.array([X_DECK, Y_DECK, Z_HOLD])
    leg = hold - START
    t_transit = np.linalg.norm(leg) / V_TRANSIT
    phase = 0                        # 0 transit, 1 observe, 2 wait, 3 descent
    obs_t, obs_z = [], []
    k_obs0 = None
    plan = dict(t_start=None, t_crest=None, z_target=None, z_hat=None, comps=None,
                fallback=False)
    log = dict(t=[], pos=[], vel=[], zd=[], zdd=[], zref=[], phase=[]) if record else None
    out = dict(contact=False, success=False, dvz=np.nan, xy_err=np.nan, t_contact=np.nan)

    for k in range(N_STEPS):
        t = k * DT
        zd = float(deck_z(t, phi1, phi2))
        az_ff = 0.0
        if phase == 0:
            s = min(t / t_transit, 1.0)
            ref = START + s * leg
            vref = leg / t_transit if s < 1.0 else np.zeros(3)
            if (s >= 1.0 and math.hypot(p[0] - X_DECK, p[1] - Y_DECK) < 0.5
                    and abs(p[2] - Z_HOLD) < 0.3 and abs(v[2]) < 0.05):
                phase, k_obs0 = 1, k
        else:
            ref, vref = hold.copy(), np.zeros(3)
        if phase == 1 and (k - k_obs0) % OBS_EVERY == 0:
            obs_t.append(t)
            obs_z.append(zd + noise[len(obs_z)])
            if len(obs_z) == N_OBS:
                plan.update(make_plan(strategy, np.array(obs_t), np.array(obs_z), t))
                phase = 2
        if phase == 2 and t >= plan['t_start'] - 1e-9:
            phase = 3
        if phase == 3:
            tau = t - plan['t_start']
            if strategy == 'naive':
                zr, vzr = Z_HOLD - V_DESC * tau, -V_DESC
            else:
                zr, vzr, az_ff = flare_reference(tau, plan['z_target'])
                if strategy.endswith('_rel') and plan['t_crest'] is not None:
                    zr, vzr, az_ff = deck_relative(zr, vzr, az_ff, t, plan)
            ref[2], vref[2] = zr, vzr

        # PD -> commanded accelerations (point mass), limits, ground-effect compensation
        a_xy = KP_XY * (ref[:2] - p[:2]) + KD_XY * (vref[:2] - v[:2])
        nrm = math.hypot(a_xy[0], a_xy[1])
        if nrm > A_XY_MAX:
            a_xy *= A_XY_MAX / nrm
        on_deck = abs(p[0] - X_DECK) <= DECK_SIZE / 2 and abs(p[1] - Y_DECK) <= DECK_SIZE / 2
        h = p[2] - zd if on_deck else p[2]          # rangefinder: deck or sea (z = 0)
        k_ge = ground_effect(h)
        f_cmd = KP_Z * (ref[2] - p[2]) + KD_Z * (vref[2] - v[2]) + az_ff + G
        if ge_comp:
            f_cmd /= k_ge
        f_cmd = min(max(f_cmd, 0.0), F_MAX)
        f_act += (f_cmd - f_act) * DT / TAU_THRUST
        axy_act += (a_xy - axy_act) * DT / TAU_THRUST
        acc = np.array([axy_act[0], axy_act[1], f_act * k_ge - G])
        v = v + acc * DT
        p = p + v * DT
        assert 0.0 <= f_act <= F_MAX + 1e-9
        assert np.linalg.norm(v) < V_LIMIT, 'speed bound exceeded'
        assert p[2] > 0.0, 'below sea level'

        t1 = t + DT
        zd1 = float(deck_z(t1, phi1, phi2))
        if record:
            log['t'].append(t1); log['pos'].append(p.copy()); log['vel'].append(v.copy())
            log['zd'].append(zd1); log['zdd'].append(float(deck_zdot(t1, phi1, phi2)))
            log['zref'].append(ref[2]); log['phase'].append(phase)
        over = abs(p[0] - X_DECK) <= DECK_SIZE / 2 and abs(p[1] - Y_DECK) <= DECK_SIZE / 2
        if over and p[2] <= zd1:
            zdd1 = float(deck_zdot(t1, phi1, phi2))
            # penetration within one step of closing speed (no tunnelling)
            a_deck = A1 * W1 ** 2 + A2 * W2 ** 2
            assert zd1 - p[2] <= (abs(v[2]) + abs(zdd1)) * DT + 0.5 * a_deck * DT ** 2 + 1e-9
            assert phase == 3, 'deck contact before the descent started'
            dvz = v[2] - zdd1
            xy = math.hypot(p[0] - X_DECK, p[1] - Y_DECK)
            out.update(contact=True, dvz=dvz, xy_err=xy, t_contact=t1,
                       success=(xy <= R_LAND and abs(dvz) <= V_LAND_MAX))
            break
        assert not (p[2] <= zd1 and abs(p[0] - X_DECK) < DECK_SIZE), 'through deck edge'

    crest = true_crests(phi1, phi2)
    if out['contact']:
        j = np.argmin(np.abs(crest - out['t_contact']))
        out['timing_err'] = float(out['t_contact'] - crest[j])
    else:
        out['timing_err'] = np.nan
    if plan['z_hat'] is not None:
        tf = plan['t_obs_end'] + np.linspace(0.0, 8.0, 401)
        out['pred_rms'] = float(np.sqrt(np.mean((plan['z_hat'](tf) - deck_z(tf, phi1, phi2)) ** 2)))
    else:
        out['pred_rms'] = np.nan
    out.update(t_start=plan['t_start'], t_crest=plan['t_crest'], z_target=plan['z_target'],
               fallback=plan['fallback'], comps=plan['comps'], z_hat=plan['z_hat'],
               obs_t=np.array(obs_t), obs_z=np.array(obs_z))
    if record:
        for key in log:
            log[key] = np.array(log[key])
        return log, out
    return out


def make_plan(strategy, t_obs, z_obs, t_now):
    plan = dict(t_obs_end=t_now)
    if strategy == 'naive':
        plan.update(t_start=t_now)
        return plan
    if strategy == 'blind':
        plan.update(t_start=t_now, z_target=float(z_obs.max()))
        return plan
    est = fft_bin_estimator if strategy.startswith('fft_bin') else fft_ls_estimator
    z_hat, comps = est(t_obs, z_obs)
    plan.update(z_hat=z_hat, comps=comps)
    sol = find_timed_crest(z_hat, t_now)
    if sol is None:              # no reachable crest predicted: behave like phase-blind
        plan.update(t_start=t_now, z_target=float(z_obs.max()), fallback=True)
    else:
        plan.update(t_crest=sol[0], z_target=float(sol[1]), t_start=sol[2])
    return plan


# ── Monte Carlo ───────────────────────────────────────────────────────────────

def draw_trials(n=N_TRIALS, seed=SEED):
    rng = np.random.default_rng(seed)
    phis = rng.uniform(0.0, 2 * np.pi, size=(n, 2))
    noise = rng.standard_normal(size=(n, N_OBS))
    return phis, noise


def set_params(**kw):
    """Override added parameters (TAU_THRUST, V_TD, T_FLARE, Z_ROTOR) for a sensitivity run.

    Returns the previous values so they can be restored."""
    global D_FLARE
    old = {k: globals()[k] for k in kw}
    globals().update(kw)
    D_FLARE = T_FLARE * (V_DESC + V_TD) / 2.0
    return old


def run_monte_carlo(phis, noise, sigma=SIGMA_Z, strategies=STRATEGIES):
    res = {}
    for s in strategies:
        base, gc = (s, True) if s != ABLATION else ('fft_ls_rel', False)
        rows = [run_simulation(base, phis[i, 0], phis[i, 1], sigma * noise[i], ge_comp=gc)
                for i in range(len(phis))]
        res[s] = dict(
            success=np.array([r['success'] for r in rows]),
            contact=np.array([r['contact'] for r in rows]),
            dvz=np.array([r['dvz'] for r in rows]),
            xy=np.array([r['xy_err'] for r in rows]),
            timing=np.array([r['timing_err'] for r in rows]),
            t_contact=np.array([r['t_contact'] for r in rows]),
            pred_rms=np.array([r['pred_rms'] for r in rows]),
            fallback=np.array([r['fallback'] for r in rows]),
        )
    return res


# ── Plots ─────────────────────────────────────────────────────────────────────

def _deck_square(z):
    h = DECK_SIZE / 2
    return [[(X_DECK - h, Y_DECK - h, z), (X_DECK + h, Y_DECK - h, z),
             (X_DECK + h, Y_DECK + h, z), (X_DECK - h, Y_DECK + h, z)]]


def plot_approach(demo, out_dir):
    fig = plt.figure(figsize=(17, 12))
    ax1 = fig.add_subplot(2, 2, 1, projection='3d')
    for s in STRATEGIES:
        log = demo[s][0]
        ax1.plot(log['pos'][:, 0], log['pos'][:, 1], log['pos'][:, 2], STYLES[s],
                 color=COLORS[s], lw=1.4, label=LABELS[s])
    tt = np.linspace(0, T_MAX, 400)
    ax1.scatter(np.full(400, X_DECK), np.full(400, Y_DECK), deck_z(tt, *DEMO_PHI),
                c='green', s=2, alpha=0.3, label='Deck heave envelope')
    ax1.scatter(*START, c='k', s=60, label='Start')
    ax1.add_collection3d(Poly3DCollection(_deck_square(Z_DECK_NOMINAL), color='green', alpha=0.4))
    ax1.set_xlabel('x (m)'); ax1.set_ylabel('y (m)'); ax1.set_zlabel('z (m)')
    ax1.set_ylim(-3, 3)
    ax1.set_title(f'3D approach (demo phases φ1 = {DEMO_PHI[0]}, φ2 = {DEMO_PHI[1]})')
    ax1.legend(fontsize=7, loc='upper left')

    ax2 = fig.add_subplot(2, 2, 2)
    o_ref = demo['fft_ls_rel'][1]
    t_obs0 = o_ref['obs_t'][0]
    t_end = max(demo[s][0]['t'][-1] for s in STRATEGIES) + 1.0
    tt = np.arange(t_obs0 - 1.0, t_end, 0.01)
    ax2.plot(tt, deck_z(tt, *DEMO_PHI), 'g-', lw=2.2, label='Deck $z_{deck}(t)$')
    for s in STRATEGIES:
        log, o = demo[s]
        ax2.plot(log['t'], log['pos'][:, 2], STYLES[s], color=COLORS[s], lw=1.4, label=LABELS[s])
        if o['contact']:
            ax2.plot(o['t_contact'], log['pos'][-1, 2], 'o' if o['success'] else 'x',
                     color=COLORS[s], ms=9, mew=2, mfc='none' if o['success'] else COLORS[s])
        if o['t_crest'] is not None and s.endswith('_rel'):
            ax2.axvline(o['t_crest'], color=COLORS[s], ls=':', lw=1.2)
    ax2.axvspan(o_ref['obs_t'][0], o_ref['obs_t'][-1], color='grey', alpha=0.12,
                label='Observation window')
    ax2.set_xlim(t_obs0 - 1.0, t_end); ax2.set_ylim(1.4, 3.8)
    ax2.set_xlabel('Time (s)'); ax2.set_ylabel('Altitude (m)')
    ax2.set_title('Vertical tracking (o = soft touchdown, x = hard; dotted = predicted crest)')
    ax2.legend(fontsize=7, loc='lower left', ncol=2)

    ax3 = fig.add_subplot(2, 2, 3)
    for s in STRATEGIES:
        log, _ = demo[s]
        m = log['phase'] == 3
        ax3.plot(log['t'][m], log['vel'][m, 2] - log['zdd'][m], STYLES[s], color=COLORS[s],
                 lw=1.4, label=LABELS[s])
    ax3.axhspan(-V_LAND_MAX, V_LAND_MAX, color='green', alpha=0.12,
                label=f'±{V_LAND_MAX} m/s band')
    ax3.set_xlabel('Time (s)'); ax3.set_ylabel(r'$\dot z_{drone} - \dot z_{deck}$ (m/s)')
    ax3.set_title('Relative vertical velocity during the descent (each curve ends at contact)')
    ax3.legend(fontsize=7, ncol=2)

    ax4 = fig.add_subplot(2, 2, 4)
    for s in STRATEGIES:
        log, _ = demo[s]
        e = np.hypot(log['pos'][:, 0] - X_DECK, log['pos'][:, 1] - Y_DECK)
        ax4.semilogy(log['t'], np.maximum(e, 1e-5), STYLES[s], color=COLORS[s], lw=1.4,
                     label=LABELS[s])
    ax4.axhline(R_LAND, color='k', ls=':', label=f'$r_{{land}}$ = {R_LAND} m')
    ax4.set_xlabel('Time (s)'); ax4.set_ylabel('Horizontal error to deck centre (m)')
    ax4.set_title('XY positioning error (log scale)')
    ax4.legend(fontsize=7)
    fig.suptitle('S095 Water Surface Landing — demo approach, all six strategies', fontsize=14)
    fig.tight_layout()
    path = os.path.join(out_dir, 'approach_analysis.png')
    fig.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_wave_prediction(demo, mc, out_dir):
    fig, axes = plt.subplots(1, 2, figsize=(15, 5))
    ax = axes[0]
    o = demo['fft_ls'][1]
    ob = demo['fft_bin'][1]
    tt = np.linspace(o['obs_t'][0], o['obs_t'][-1] + 8.0, 800)
    ax.plot(tt, deck_z(tt, *DEMO_PHI), 'g-', lw=2.2, label='True deck height')
    ax.plot(o['obs_t'], o['obs_z'], 'k.', ms=4, label=f'{N_OBS} noisy samples (10 Hz)')
    ax.plot(tt, ob['z_hat'](tt), '-', color=COLORS['fft_bin'], lw=1.5,
            label='FFT bins (card estimator)')
    ax.plot(tt, o['z_hat'](tt), '--', color=COLORS['fft_ls_rel'], lw=1.5,
            label='FFT + least squares')
    ax.axvline(o['obs_t'][-1], color='grey', ls=':')
    ax.set_xlabel('Time (s)'); ax.set_ylabel('Deck height (m)')
    ax.set_title('Demo trial: observation window, then 8 s of prediction')
    ax.legend(fontsize=8, loc='lower left')
    ax = axes[1]
    bins = np.logspace(-3, 0, 40)
    ax.hist(mc['fft_bin']['pred_rms'], bins=bins, color=COLORS['fft_bin'], alpha=0.6,
            label='FFT bins (card estimator)')
    ax.hist(mc['fft_ls']['pred_rms'], bins=bins, color=COLORS['fft_ls_rel'], alpha=0.6,
            label='FFT + least squares')
    ax.set_xscale('log')
    ax.axvline(SIGMA_Z, color='k', ls=':', label=f'sensor σ = {SIGMA_Z} m')
    ax.set_xlabel('RMS prediction error over the next 8 s (m)')
    ax.set_ylabel('Trials')
    ax.set_title(f'Prediction error over {N_TRIALS} Monte Carlo trials')
    ax.legend(fontsize=8)
    fig.tight_layout()
    path = os.path.join(out_dir, 'wave_prediction.png')
    fig.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_monte_carlo(mc, out_dir):
    keys = STRATEGIES + [ABLATION]
    fig, axes = plt.subplots(1, 3, figsize=(20, 6))
    ax = axes[0]
    vals = [100 * mc[s]['success'].mean() for s in keys]
    bars = ax.bar(range(len(keys)), vals, color=[COLORS[s] for s in keys], alpha=0.85)
    for b, v in zip(bars, vals):
        ax.text(b.get_x() + b.get_width() / 2, v + 1, f'{v:.1f}%', ha='center', fontsize=9,
                fontweight='bold')
    ax.axhline(85, color='k', ls='--', lw=1, label='card target 85 % (FFT-timed)')
    ax.axhline(45, color='grey', ls=':', lw=1, label='card ceiling 45 % (naive)')
    ax.set_xticks(range(len(keys)))
    ax.set_xticklabels([LABELS[s] for s in keys], rotation=25, ha='right', fontsize=8)
    ax.set_ylim(0, 110); ax.set_ylabel('Soft-landing success (%)')
    ax.set_title(f'Success rate, N = {N_TRIALS} common-random-number trials')
    ax.legend(fontsize=8, loc='upper left')

    ax = axes[1]
    bins = np.linspace(-1.2, 0.3, 61)
    for s in STRATEGIES:
        d = mc[s]['dvz'][mc[s]['contact']]
        ax.hist(np.clip(d, bins[0], bins[-1]), bins=bins, histtype='step', lw=1.6,
                color=COLORS[s], ls=STYLES[s], label=LABELS[s])
    ax.axvspan(-V_LAND_MAX, V_LAND_MAX, color='green', alpha=0.15, label='success band')
    ax.set_xlabel(r'Relative vertical velocity at contact $\Delta\dot z$ (m/s)')
    ax.set_ylabel('Trials'); ax.set_title('Contact velocity distribution')
    ax.legend(fontsize=7)

    ax = axes[2]
    bins = np.linspace(-1.3, 1.3, 53)
    for s in STRATEGIES:
        d = mc[s]['timing'][mc[s]['contact']]
        ax.hist(d, bins=bins, histtype='step', lw=1.6, color=COLORS[s], ls=STYLES[s],
                label=LABELS[s])
    ax.set_xlabel(r'$t_{contact} - t_{crest}$ (nearest true crest, s)')
    ax.set_ylabel('Trials'); ax.set_title('Touchdown timing relative to the nearest crest')
    ax.legend(fontsize=7)
    fig.tight_layout()
    path = os.path.join(out_dir, 'monte_carlo.png')
    fig.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


SWEEP_STRATEGIES = ['fft_ls', 'fft_ls_rel']


def plot_noise_sensitivity(sweep, out_dir):
    fig, ax = plt.subplots(figsize=(7.5, 4.8))
    xs = [100 * x for x in NOISE_SWEEP]
    for s in SWEEP_STRATEGIES:
        ax.plot(xs, [100 * sweep[x][s] for x in NOISE_SWEEP], 'o' + STYLES[s],
                color=COLORS[s], lw=1.8, label=LABELS[s])
    ax.axhline(85, color='k', ls='--', lw=1, label='card target 85 %')
    ax.set_xscale('log')
    ax.set_xticks(xs)
    ax.set_xticklabels([f'{x:g}' for x in xs])
    ax.set_xlabel('Deck sensor noise σ (cm)'); ax.set_ylabel('Soft-landing success (%)')
    ax.set_ylim(0, 105)
    ax.set_title(f'Sensitivity to sensor noise (same {N_TRIALS} trials)')
    ax.legend(fontsize=8)
    fig.tight_layout()
    path = os.path.join(out_dir, 'noise_sensitivity.png')
    fig.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_param_sensitivity(psens, out_dir):
    units = {'TAU_THRUST': 's', 'V_TD': 'm/s', 'T_FLARE': 's', 'Z_ROTOR': 'm'}
    fig, axes = plt.subplots(1, 4, figsize=(18, 4.2), sharey=True)
    for ax, (name, d) in zip(axes, psens.items()):
        for gc, ls, lab in ((True, 'o-', 'GE compensated'), (False, 's--', 'no GE comp.')):
            pts = sorted((v, s) for (v, g), s in d.items() if g == gc)
            if pts:
                ax.plot([p[0] for p in pts], [100 * p[1] for p in pts], ls,
                        color=COLORS['fft_ls_rel'] if gc else COLORS[ABLATION], label=lab)
        ax.axvline(globals()[name], color='grey', ls=':', lw=1)
        ax.axhline(85, color='k', ls='--', lw=1)
        ax.set_xlabel(f'{name} ({units[name]})'); ax.set_title(f'{name} (dotted = baseline)')
        ax.legend(fontsize=8)
    axes[0].set_ylabel('Soft-landing success (%)'); axes[0].set_ylim(0, 105)
    fig.suptitle('FFT+LS deck-relative flare: sensitivity to added parameters '
                 f'({N_TRIALS} trials each, dashed = card 85 % target)')
    fig.tight_layout()
    path = os.path.join(out_dir, 'parameter_sensitivity.png')
    fig.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def gif_indices(n, step):
    n_frames = math.ceil((n - 1) / step) + 1
    return [min(i * step, n - 1) for i in range(n_frames)]


def save_animation(demo, out_dir, step=10):
    log, _ = demo['fft_ls_rel']
    lb, _ = demo['naive']
    n = len(log['t'])
    idx = gif_indices(n, step)
    fig = plt.figure(figsize=(12, 5.2))
    ax = fig.add_subplot(1, 2, 1, projection='3d')
    ax2 = fig.add_subplot(1, 2, 2)
    t_all = log['t']
    ax2.plot(t_all, log['zd'], 'g-', lw=2, alpha=0.35)
    deck_line, = ax2.plot([], [], 'g-', lw=2, label='Deck')
    zl, = ax2.plot([], [], '-', color=COLORS['fft_ls_rel'], lw=1.6, label=LABELS['fft_ls_rel'])
    zlb, = ax2.plot([], [], '--', color=COLORS['naive'], lw=1.3, label=LABELS['naive'])
    ax2.set_xlim(t_all[0], t_all[-1] + 0.5); ax2.set_ylim(1.3, 4.0)
    ax2.set_xlabel('Time (s)'); ax2.set_ylabel('Altitude (m)')
    ax2.set_title('Altitude vs deck (descent close-up)')
    ax2.legend(fontsize=8, loc='upper right')
    deck_poly = [None]
    dot, = ax.plot([], [], [], 'o', color=COLORS['fft_ls_rel'], ms=7, label=LABELS['fft_ls_rel'])
    trail, = ax.plot([], [], [], '-', color=COLORS['fft_ls_rel'], lw=1.2)
    dotb, = ax.plot([], [], [], 'o', color=COLORS['naive'], ms=6, label=LABELS['naive'])
    trailb, = ax.plot([], [], [], '--', color=COLORS['naive'], lw=1.0)
    xs, ys = np.meshgrid([-1, 11], [-3, 3])
    ax.plot_surface(xs, ys, np.zeros_like(xs, dtype=float), color='cyan', alpha=0.15)
    ax.set_xlim(-1, 11); ax.set_ylim(-3, 3); ax.set_zlim(0, 16)
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)'); ax.set_zlabel('z (m)')
    ax.view_init(elev=14, azim=-68)
    ax.legend(loc='upper left', fontsize=8)
    ax.set_title('S095 FFT-timed landing on a heaving deck')
    txt = ax2.text(0.02, 0.97, '', transform=ax2.transAxes, fontsize=8, family='monospace',
                   va='top')
    names = {0: 'transit', 1: 'observe', 2: 'wait', 3: 'descent'}

    def update(fi):
        k = idx[fi]
        if deck_poly[0] is not None:
            deck_poly[0].remove()
        deck_poly[0] = ax.add_collection3d(Poly3DCollection(_deck_square(log['zd'][k]),
                                                            color='green', alpha=0.7))
        dot.set_data([log['pos'][k, 0]], [log['pos'][k, 1]])
        dot.set_3d_properties([log['pos'][k, 2]])
        trail.set_data(log['pos'][:k + 1, 0], log['pos'][:k + 1, 1])
        trail.set_3d_properties(log['pos'][:k + 1, 2])
        kb = min(k, len(lb['t']) - 1)
        dotb.set_data([lb['pos'][kb, 0]], [lb['pos'][kb, 1]])
        dotb.set_3d_properties([lb['pos'][kb, 2]])
        trailb.set_data(lb['pos'][:kb + 1, 0], lb['pos'][:kb + 1, 1])
        trailb.set_3d_properties(lb['pos'][:kb + 1, 2])
        deck_line.set_data(t_all[:k + 1], log['zd'][:k + 1])
        zl.set_data(t_all[:k + 1], log['pos'][:k + 1, 2])
        zlb.set_data(lb['t'][:kb + 1], lb['pos'][:kb + 1, 2])
        dvz = log['vel'][k, 2] - log['zdd'][k]
        txt.set_text(f't = {log["t"][k]:5.2f} s  [{names[int(log["phase"][k])]}]\n'
                     f'h above deck = {log["pos"][k, 2] - log["zd"][k]:6.3f} m\n'
                     f'dvz = {dvz:+.3f} m/s')
        return dot, trail, dotb, trailb, deck_line, zl, zlb, txt

    anim = animation.FuncAnimation(fig, update, frames=len(idx), interval=50, blit=False)
    path = os.path.join(out_dir, 'animation.gif')
    anim.save(path, writer='pillow', fps=20, dpi=72)
    plt.close(fig)
    print(f'Saved: {path}  ({len(idx)} frames, step {step}, {n} samples)')


# ── Main ──────────────────────────────────────────────────────────────────────

def summary_line(r):
    c = r['contact']
    ad = np.abs(r['dvz'][c])
    tm = np.abs(r['timing'][c])
    return (f'success {100 * r["success"].mean():5.1f}% ({r["success"].sum()}/{len(c)}), '
            f'contact {c.sum()}/{len(c)}, |dvz| {ad.mean():.3f} ± {ad.std():.3f} m/s '
            f'(median {np.median(ad):.3f}), |timing| {tm.mean():.3f} ± {tm.std():.3f} s, '
            f'max xy {np.nanmax(r["xy"]):.4f} m, mean contact time {np.nanmean(r["t_contact"]):.2f} s, '
            f'fallbacks {r["fallback"].sum()}')


if __name__ == '__main__':
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    print('=== S095 Water Surface Landing ===')
    amp, rms_card, rms_corr = card_phase_check()
    print(f'Card phase-formula check (bin-aligned 0.4 m sinusoid, window from t0 = 10 s, 8 s '
          f'ahead): amplitude {amp:.4f} m, RMS error card formula {rms_card:.4f} m, '
          f'corrected {rms_corr:.2e} m')
    t0s = np.arange(0.0, 1.0 / (F_SAMPLE / N_OBS), 0.05)          # one full period, 6.4 s
    errs = np.array([card_phase_check(t0)[1:] for t0 in t0s])
    print(f'  over window start t0 in [0, {t0s[-1]:.2f}] s: card formula RMS error '
          f'{errs[:, 0].min():.4f} to {errs[:, 0].max():.4f} m (corrected max '
          f'{errs[:, 1].max():.1e} m)')
    print(f'FFT bin spacing {F_SAMPLE / N_OBS:.5f} Hz vs swell {1 / T1:.5f} Hz '
          f'({N_OBS / F_SAMPLE / T1:.3f} bins) and chop {1 / T2:.5f} Hz '
          f'({N_OBS / F_SAMPLE / T2:.3f} bins)')
    print(f'Ground effect T_IGE/T_OGE (gear height above deck): 0.30 m {ground_effect(0.30):.4f}, '
          f'0.10 m {ground_effect(0.10):.4f}, 0 m (contact) {ground_effect(0.0):.4f}')
    print(f'Max deck heave speed {A1 * W1 + A2 * W2:.3f} m/s, max deck acceleration '
          f'{A1 * W1 ** 2 + A2 * W2 ** 2:.3f} m/s^2')
    print(f'Flare distance {D_FLARE:.3f} m; descent duration to a 2.55 m crest '
          f'{descent_duration(2.55):.2f} s, to a 1.75 m crest {descent_duration(1.75):.2f} s')
    print(f'Card criterion check: a constant {V_DESC} m/s descent onto a crest (deck speed 0) '
          f'gives |dvz| = {V_DESC:.2f} m/s > {V_LAND_MAX} m/s')

    phis, noise = draw_trials()
    demo = {s: run_simulation(s, *DEMO_PHI, SIGMA_Z * noise[0], record=True) for s in STRATEGIES}
    print(f'\nDemo run (phi1={DEMO_PHI[0]}, phi2={DEMO_PHI[1]}, noise draws of trial 0):')
    for s in STRATEGIES:
        o = demo[s][1]
        extra = ''
        if o['t_crest'] is not None:
            extra = (f', predicted crest {o["t_crest"]:.2f} s at {o["z_target"]:.3f} m, '
                     f'descent start {o["t_start"]:.2f} s')
        print(f'  {LABELS[s]:30s} contact {o["t_contact"]:.2f} s, dvz {o["dvz"]:+.3f} m/s, '
              f'xy {o["xy_err"]:.4f} m, timing {o["timing_err"]:+.3f} s, '
              f'success {o["success"]}{extra}')
    o = demo['fft_ls'][1]
    print(f'  Observation window {o["obs_t"][0]:.2f}-{o["obs_t"][-1]:.2f} s')
    print('  FFT-bin components (A, f): ' + ', '.join(
        f'({a:.3f} m, {f:.4f} Hz)' for a, f, _ in demo['fft_bin'][1]['comps']))
    print('  FFT+LS components (A, f): ' + ', '.join(
        f'({a:.3f} m, {f:.4f} Hz)' for a, f in demo['fft_ls'][1]['comps']))
    print(f'  8 s prediction RMS: bins {demo["fft_bin"][1]["pred_rms"]:.4f} m, '
          f'LS {demo["fft_ls"][1]["pred_rms"]:.4f} m')

    print(f'\nMonte Carlo ({N_TRIALS} trials, common phases and noise, sigma = {SIGMA_Z} m):')
    keys = STRATEGIES + [ABLATION]
    mc = run_monte_carlo(phis, noise, strategies=keys)
    for s in keys:
        print(f'  {LABELS[s]:30s} ' + summary_line(mc[s]))
    for s in ('fft_bin', 'fft_ls'):
        r = mc[s]
        print(f'  8 s prediction RMS, {s:8s}: {np.nanmean(r["pred_rms"]):.4f} ± '
              f'{np.nanstd(r["pred_rms"]):.4f} m (median {np.nanmedian(r["pred_rms"]):.4f}, '
              f'max {np.nanmax(r["pred_rms"]):.4f})')
    best = mc['fft_ls_rel']['success']
    print('  Per-trial wins / losses of FFT+LS deck-relative against:')
    for s in [x for x in keys if x != 'fft_ls_rel']:
        o2 = mc[s]['success']
        print(f'    {LABELS[s]:30s} wins {int((best & ~o2).sum())}, '
              f'losses {int((~best & o2).sum())}, both succeed {int((best & o2).sum())}, '
              f'both fail {int((~best & ~o2).sum())}')
    fails = np.where(~best)[0]
    if len(fails):
        r = mc['fft_ls_rel']
        print(f'  FFT+LS deck-relative failures: {len(fails)}; (trial, dvz, xy, pred RMS): '
              + ', '.join(f'({i}, {r["dvz"][i]:+.3f}, {r["xy"][i]:.3f}, {r["pred_rms"][i]:.4f})'
                          for i in fails))

    print('\nNoise sensitivity (success %, same phases, noise draws scaled):')
    sweep = {}
    for sg in NOISE_SWEEP:
        r = mc if sg == SIGMA_Z else run_monte_carlo(phis, noise, sigma=sg,
                                                     strategies=SWEEP_STRATEGIES)
        sweep[sg] = {s: float(r[s]['success'].mean()) for s in SWEEP_STRATEGIES}
        print(f'  sigma {100 * sg:4.1f} cm: ' + ', '.join(
            f'{LABELS[s]} {100 * sweep[sg][s]:.1f}%' for s in SWEEP_STRATEGIES))

    print('\nAdded-parameter sensitivity of FFT+LS deck-relative (success %, same 200 trials):')
    psens = {}
    for name, values in PARAM_SWEEPS.items():
        psens[name] = {}
        base_val = globals()[name]
        for val in values:
            for gc in ((True, False) if name == 'Z_ROTOR' else (True,)):
                key = 'fft_ls_rel' if gc else ABLATION
                if val == base_val:
                    succ = float(mc[key]['success'].mean())
                else:
                    old = set_params(**{name: val})
                    succ = float(run_monte_carlo(phis, noise, strategies=[key])[key]
                                 ['success'].mean())
                    set_params(**old)
                psens[name][(val, gc)] = succ
        print(f'  {name:10s}: ' + ', '.join(
            f'{v:g}{"" if gc else " (no GE comp.)"} -> {100 * s:.1f}%'
            for (v, gc), s in psens[name].items()))
    print('  Strategy ranking at the two stress points (success %, same 200 trials):')
    for name, val in STRESS_POINTS:
        old = set_params(**{name: val})
        r = run_monte_carlo(phis, noise, strategies=STRESS_STRATEGIES)
        set_params(**old)
        print(f'    {name} = {val:g}: ' + ', '.join(
            f'{LABELS[s]} {100 * r[s]["success"].mean():.1f}%' for s in STRESS_STRATEGIES)
              + (f' (best strategy: {100 * psens[name][(val, True)]:.1f}%, see above)'))
    print(f'  (baseline TAU_THRUST={TAU_THRUST}, V_TD={V_TD}, T_FLARE={T_FLARE}, '
          f'Z_ROTOR={Z_ROTOR}; D_FLARE back to {D_FLARE:.3f} m)')

    print()
    plot_approach(demo, OUTPUT_DIR)
    plot_wave_prediction(demo, mc, OUTPUT_DIR)
    plot_monte_carlo(mc, OUTPUT_DIR)
    plot_noise_sensitivity(sweep, OUTPUT_DIR)
    plot_param_sensitivity(psens, OUTPUT_DIR)
    save_animation(demo, OUTPUT_DIR)
