"""
S097 Aerial Puzzle Assembly
===========================
Six drones (0.5 kg) each carry one equilateral-triangle piece (side sqrt(3) m). Together the
pieces tile a regular hexagon whose slot centroids lie on a 1 m circle at z = 10 m. The drones
start on a 5 m staging circle at z = 8 m, climb, converge with a consensus law over a ring
graph, switch to impedance fitting when every drone is within 50 mm of its slot, and then hold
for 10 s while wind and sensor errors act on them.

Strategies (same wind, sensor errors and start jitter for all; CRN):
  1. Card law      - consensus on absolute positions, p += eps*sum(p_j - p_k) + alpha*(p* - p).
                     Its fixed point is a shrunken hexagon, so it never reaches the 50 mm switch.
  2. Independent   - eps = 0: each drone tracks its own slot; impedance fitting.
  3. Consensus     - consensus on slot errors e_k = p_k - p_k* (formation consensus) with the
                     card gains; in the fitting phase the same coupling acts as virtual springs
                     between ring neighbours using relative (vision) measurements.

Physics: point-mass drones with |a| <= 5 m/s^2, common + individual Ornstein-Uhlenbeck wind,
an absolute-position bias (RTK-like, slowly varying) and 1 mm relative sensing noise, soft
inter-drone repulsion, and a spring-damper edge contact between neighbouring pieces.

Usage:
    conda activate drones
    python src/05_special_entertainment/s097_aerial_puzzle.py
"""

import os
import math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Patch
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# ── Parameters (card) ─────────────────────────────────────────────────────────
N_DRONES     = 6
M_DRONE      = 0.5          # kg
DT           = 0.02         # s
P_CENTRE     = np.array([0.0, 0.0, 10.0])
R_HEX        = 1.0          # m  slot-centroid radius
Z_ASM        = 10.0
R_STAGE      = 5.0
Z_STAGE      = 8.0
EPSILON      = 0.05         # consensus coupling
ALPHA        = 0.10         # target attraction
K_IMP        = 8.0          # N/m
B_IMP        = 4.0          # N s/m
DELTA_SWITCH = 0.05         # m
DELTA_FIT    = 0.005        # m
D_MIN        = 0.30         # m
K_REP        = 1.5
K_PSI        = 1.0
JITTER       = 0.3          # rad  start-angle jitter (card code)
KP_TRACK, KD_TRACK = 10.0, 5.0   # card: acc = 10 (p_cmd - p) - 5 v in the consensus phase
KP_ASC, KD_ASC = 8.0, 4.0        # card ascent PD

# ── Parameters (added) ────────────────────────────────────────────────────────
A_MAX        = 5.0          # m/s^2 acceleration limit
SIDE         = math.sqrt(3) * R_HEX   # piece side so the six triangles tile the hexagon
K_REL        = K_IMP * EPSILON / ALPHA   # 4 N/m neighbour coupling in the fitting phase
K_C, B_C     = 50.0, 5.0    # edge contact spring / damper (card extension 6)
T_HOLD       = 10.0         # s  hold window over which the fit is scored
T_LIMIT      = 90.0         # s  stop if the switch threshold is never reached
WIND_COMMON  = 0.03         # m/s^2 std of the common gust (OU), light air
WIND_INDIV   = 0.01         # m/s^2 std of each drone's turbulence (OU)
WIND_STRONG  = 0.20         # m/s^2 common-gust std for the strong-wind check
TAU_WIND     = 3.0          # s
SIGMA_BIAS   = 0.005        # m  absolute-position bias std (OU, RTK-like)
TAU_BIAS     = 10.0         # s
SIGMA_REL    = 0.001        # m  relative (neighbour) measurement noise
N_MC         = 20
BIAS_SWEEP   = [0.0, 0.005, 0.01]
WIND_SWEEP   = [(0.0, 0.0), (0.01, 0.003)]   # (common, individual) m/s^2, run with no bias
STRATS       = ['card', 'indep', 'consensus']
LABELS       = {'card': 'Card law (absolute consensus)', 'indep': 'Independent (eps = 0)',
                'consensus': 'Error consensus + coupled fit'}
SCOL         = {'card': '#7f7f7f', 'indep': '#1f77b4', 'consensus': '#d62728'}
DRONE_COLORS = ['tab:red', 'tab:orange', 'tab:green', 'tab:cyan', 'tab:blue', 'tab:purple']

OUTPUT_DIR = os.path.normpath(os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '05_special_entertainment', 's097_aerial_puzzle'))

ANG = 2 * np.pi * np.arange(N_DRONES) / N_DRONES
TARGETS = np.column_stack([P_CENTRE[0] + R_HEX * np.cos(ANG), P_CENTRE[1] + R_HEX * np.sin(ANG),
                           np.full(N_DRONES, Z_ASM)])
PSI_T = ANG.copy()
# shared edge between piece k and k+1 lies along angle ANG_k + 30 deg; its in-plane normal
EDGE_N = np.column_stack([-np.sin(ANG + np.pi / 6), np.cos(ANG + np.pi / 6),
                          np.zeros(N_DRONES)])


def ring_laplacian(n):
    L = 2.0 * np.eye(n)
    for k in range(n):
        L[k, (k - 1) % n] = L[k, (k + 1) % n] = -1.0
    return L


LAP = ring_laplacian(N_DRONES)


def card_fixed_point():
    """Fixed point of the card's absolute-position consensus law."""
    A = EPSILON * LAP + ALPHA * np.eye(N_DRONES)
    P = np.linalg.solve(A, ALPHA * TARGETS)
    return P


def piece_triangle(p, psi):
    """Corners of the equilateral piece centred at p, one vertex pointing to the centre."""
    rc = SIDE / math.sqrt(3)
    angs = psi + np.pi + np.array([0.0, 2 * np.pi / 3, 4 * np.pi / 3])
    return np.column_stack([p[0] + rc * np.cos(angs), p[1] + rc * np.sin(angs),
                            np.full(3, p[2])])


# ── Simulation ────────────────────────────────────────────────────────────────

def ou_step(x, sigma, tau, rng_draw):
    return x + (-x / tau) * DT + sigma * math.sqrt(2 * DT / tau) * rng_draw


def run_simulation(strategy, seed=0, sigma_bias=SIGMA_BIAS, record=False,
                   wind_common=WIND_COMMON, wind_indiv=WIND_INDIV):
    rng = np.random.default_rng(seed)
    ang0 = ANG + rng.uniform(-JITTER, JITTER, N_DRONES)
    n_steps = int(round((T_LIMIT + T_HOLD) / DT))
    # pre-draw every random number so all strategies see the same sequence (CRN)
    w_c_draw = rng.standard_normal((n_steps, 3))
    w_i_draw = rng.standard_normal((n_steps, N_DRONES, 3))
    b_draw = rng.standard_normal((n_steps, N_DRONES, 3))
    r_draw = rng.standard_normal((n_steps, N_DRONES, 3))
    bias = sigma_bias * rng.standard_normal((N_DRONES, 3))
    pos = np.column_stack([R_STAGE * np.cos(ang0), R_STAGE * np.sin(ang0),
                           np.full(N_DRONES, Z_STAGE)])
    vel = np.zeros((N_DRONES, 3))
    psi = ang0.copy()
    w_c = np.zeros(3)
    w_i = np.zeros((N_DRONES, 3))
    phase, t_asc, t_conv, t_hold_end = 0, None, None, None
    log = dict(t=[], pos=[], psi=[], err=[], gap=[], phase=[], min_sep=[], fc=[]) \
        if record else None
    hold_err, hold_gap, hold_fc, hold_rel = [], [], [], []
    t_fit = None
    min_sep_all = np.inf
    k_hold0 = None
    for k in range(n_steps):
        t = k * DT
        e_true = pos - TARGETS
        meas = pos + bias                                   # absolute (biased) estimate
        e_meas = meas - TARGETS
        # relative measurements of ring neighbours (true relative + 1 mm noise)
        rel_noise = SIGMA_REL * r_draw[k]
        if phase == 0 and np.all(pos[:, 2] >= Z_ASM - 0.05):
            phase, t_asc = 1, t
        if phase == 1 and np.abs(e_meas).max() < np.inf and \
                np.linalg.norm(e_meas, axis=1).max() <= DELTA_SWITCH:
            phase, t_conv = 2, t
            k_hold0 = k
        if phase == 0:
            p_des = np.column_stack([R_STAGE * np.cos(ang0), R_STAGE * np.sin(ang0),
                                     np.full(N_DRONES, Z_ASM)])
            acc = KP_ASC * (p_des - meas) - KD_ASC * vel
        elif phase == 1:
            if strategy == 'card':
                upd = -EPSILON * (LAP @ meas) + ALPHA * (TARGETS - meas)
            else:
                eps = EPSILON if strategy == 'consensus' else 0.0
                # L e computed from neighbour-relative measurements:
                # (L e)_k = sum_j (e_k - e_j) = sum_j ((p_k - p_j) - (p_k* - p_j*))
                rel_err = relative_errors(pos, rel_noise)
                upd = -eps * rel_err - ALPHA * e_meas
            acc = KP_TRACK * upd - KD_TRACK * vel
        else:
            F = K_IMP * (-e_meas) - B_IMP * vel
            if strategy == 'consensus':
                F = F - K_REL * relative_errors(pos, rel_noise)
            acc = F / M_DRONE
        # soft repulsion (card), all phases
        diff = pos[:, None, :] - pos[None, :, :]
        dist = np.linalg.norm(diff, axis=2) + np.eye(N_DRONES) * 1e9
        close = dist < 2 * D_MIN
        rep = np.where(close[:, :, None],
                       K_REP * (2 * D_MIN - dist)[:, :, None] / dist[:, :, None] ** 2 * diff,
                       0.0).sum(axis=1)
        acc = acc + rep
        nrm = np.linalg.norm(acc, axis=1, keepdims=True)
        acc = np.where(nrm > A_MAX, acc * A_MAX / np.maximum(nrm, 1e-12), acc)
        # edge contact between neighbouring pieces (overlap of the shared edge)
        e_next = np.roll(e_true, -1, axis=0)
        v_next = np.roll(vel, -1, axis=0)
        gap = np.einsum('ij,ij->i', EDGE_N, e_next - e_true)     # +: edge k|k+1 opens
        gap_rate = np.einsum('ij,ij->i', EDGE_N, v_next - vel)
        # pieces can only touch edge-to-edge when both are near their seats (the linearised
        # gap is meaningless far from the assembled pose)
        err_n = np.linalg.norm(e_true, axis=1)
        seated = (err_n <= DELTA_SWITCH) & (np.roll(err_n, -1) <= DELTA_SWITCH)
        pen = np.where(seated, np.maximum(-gap, 0.0), 0.0)
        fmag = np.where(pen > 0, np.maximum(K_C * pen - B_C * gap_rate, 0.0), 0.0)
        fc = np.zeros((N_DRONES, 3))
        fc -= fmag[:, None] * EDGE_N                          # pushes piece k back
        fc += np.roll(fmag[:, None] * EDGE_N, 1, axis=0)       # pushes piece k+1 forward
        # wind (common + individual OU)
        w_c = ou_step(w_c, wind_common, TAU_WIND, w_c_draw[k])
        w_i = ou_step(w_i, wind_indiv, TAU_WIND, w_i_draw[k])
        bias = ou_step(bias, sigma_bias, TAU_BIAS, b_draw[k]) if sigma_bias > 0 else bias
        a_tot = acc + fc / M_DRONE + w_c[None, :] + w_i
        vel = vel + a_tot * DT
        pos = pos + vel * DT
        psi = psi + K_PSI * (PSI_T - psi) * DT
        assert np.all(pos[:, 2] > 0.0)
        assert np.all(np.linalg.norm(vel, axis=1) < 10.0)
        sep = dist[np.triu_indices(N_DRONES, 1)].min()
        min_sep_all = min(min_sep_all, sep)
        if record:
            log['t'].append(t); log['pos'].append(pos.copy()); log['psi'].append(psi.copy())
            log['err'].append(np.linalg.norm(pos - TARGETS, axis=1))
            log['gap'].append(gap.copy()); log['phase'].append(phase)
            log['min_sep'].append(sep); log['fc'].append(fmag.copy())
        if phase >= 2:
            hold_err.append(np.linalg.norm(pos - TARGETS, axis=1).max())
            if t_fit is None and hold_err[-1] <= DELTA_FIT:
                t_fit = t + DT            # card: impedance phase ends at eps <= 5 mm
            hold_gap.append(np.abs(gap).max())
            hold_rel.append(np.linalg.norm(np.roll(e_true, -1, axis=0) - e_true, axis=1).max())
            hold_fc.append(fmag.max())
            if t - t_conv >= T_HOLD - 1e-9:
                t_hold_end = t
                break
        if phase < 2 and t >= T_LIMIT:
            break
    res = dict(t_asc=t_asc, t_conv=t_conv, converged=t_conv is not None, min_sep=min_sep_all,
               fit_time=(t_fit - t_conv) if t_fit is not None else np.nan,
               final_err=float(np.linalg.norm(pos - TARGETS, axis=1).max()),
               final_radius=float(np.linalg.norm(pos[:, :2] - P_CENTRE[:2], axis=1).mean()))
    if hold_err:
        he, hg, hr = np.array(hold_err), np.array(hold_gap), np.array(hold_rel)
        # score the last 8 s of the hold (first 2 s = fitting transient)
        m = np.arange(len(he)) * DT >= 2.0
        res.update(hold_err_max=float(he[m].max()), hold_err_mean=float(he[m].mean()),
                   hold_gap_max=float(hg[m].max()), hold_gap_mean=float(hg[m].mean()),
                   hold_rel_max=float(hr[m].max()),
                   frac_err_ok=float((he[m] <= DELTA_FIT).mean()),
                   frac_gap_ok=float((hg[m] <= DELTA_FIT).mean()),
                   fc_max=float(np.array(hold_fc).max()))
    if record:
        for key in log:
            log[key] = np.array(log[key])
        res['log'] = log
    return res


def relative_errors(pos, rel_noise):
    """(L e)_k from neighbour-relative measurements with additive noise."""
    out = np.zeros((N_DRONES, 3))
    for d in (-1, 1):
        rel = pos - np.roll(pos, d, axis=0) + rel_noise * (1 if d == 1 else -1)
        out += rel - (TARGETS - np.roll(TARGETS, d, axis=0))
    return out


# ── Experiments ───────────────────────────────────────────────────────────────

def monte_carlo(strategy, sigma_bias=SIGMA_BIAS, n=N_MC, wind=(WIND_COMMON, WIND_INDIV)):
    rows = [run_simulation(strategy, seed=s, sigma_bias=sigma_bias, wind_common=wind[0],
                           wind_indiv=wind[1]) for s in range(n)]
    keys = ['t_conv', 'fit_time', 'hold_err_max', 'hold_err_mean', 'hold_gap_max', 'hold_gap_mean',
            'hold_rel_max', 'frac_err_ok', 'frac_gap_ok', 'fc_max', 'min_sep', 'final_radius']
    out = {k: np.array([r.get(k, np.nan) if r.get(k) is not None else np.nan for r in rows],
                       dtype=float) for k in keys}
    out['converged'] = np.array([r['converged'] for r in rows])
    return out


# ── Plots ─────────────────────────────────────────────────────────────────────

def plot_3d(runs, out_dir):
    fig = plt.figure(figsize=(16, 7))
    for i, s in enumerate(['card', 'consensus']):
        ax = fig.add_subplot(1, 2, i + 1, projection='3d')
        log = runs[s]['log']
        P = log['pos']
        for k in range(N_DRONES):
            ax.plot(P[:, k, 0], P[:, k, 1], P[:, k, 2], color=DRONE_COLORS[k], lw=1.3,
                    label=f'Drone {k}')
            ax.scatter(*P[0, k], color=DRONE_COLORS[k], s=40, facecolors='none')
            ax.scatter(*TARGETS[k], color=DRONE_COLORS[k], marker='*', s=110)
            tri = piece_triangle(P[-1, k], log['psi'][-1, k])
            ax.add_collection3d(Poly3DCollection([tri], color=DRONE_COLORS[k], alpha=0.35))
        hx = np.vstack([TARGETS, TARGETS[0]])
        ax.plot(hx[:, 0], hx[:, 1], hx[:, 2], 'k--', lw=1, alpha=0.6, label='Slot hexagon')
        ax.scatter(*P_CENTRE, color='green', marker='^', s=80, label='Assembly centre')
        ax.set_xlim(-5.5, 5.5); ax.set_ylim(-5.5, 5.5); ax.set_zlim(7.5, 10.5)
        ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
        r = runs[s]
        extra = (f'converged at {r["t_conv"]:.1f} s' if r['converged'] else
                 f'never reaches 50 mm; final radius {r["final_radius"]:.3f} m')
        ax.set_title(f'{LABELS[s]}\n(seed 0, {extra}; triangles = pieces at the end)', fontsize=10)
        if i == 0:
            ax.legend(fontsize=7, loc='upper left')
    fig.tight_layout()
    path = os.path.join(out_dir, 's097_3d_trajectory.png')
    fig.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_metrics(run, out_dir, name, title):
    log = run['log']
    t, ph = log['t'], log['phase']
    fig, axes = plt.subplots(2, 2, figsize=(14, 8.5))
    pc = {0: '#fff3e0', 1: '#e8f5e9', 2: '#e3f2fd'}
    pl = {0: 'Ascent', 1: 'Consensus', 2: 'Impedance fit + hold'}
    for ax in axes.flat:
        for p in range(3):
            m = ph == p
            if m.any():
                ax.axvspan(t[m][0], t[m][-1], color=pc[p], alpha=0.6)
    ax = axes[0, 0]
    ax.semilogy(t, log['err'].max(axis=1), 'k-', lw=1.8, label=r'$\varepsilon_{assembly}$ (true)')
    ax.semilogy(t, np.abs(log['gap']).max(axis=1), '-', color='tab:purple', lw=1.0,
                label='max |edge gap|')
    ax.axhline(DELTA_SWITCH, color='orange', ls='--', label=r'$\delta_{switch}$ = 50 mm')
    ax.axhline(DELTA_FIT, color='red', ls=':', label=r'$\delta_{fit}$ = 5 mm')
    ax.set_title('Assembly error (log)'); ax.set_xlabel('Time (s)'); ax.legend(fontsize=8)
    ax = axes[0, 1]
    for k in range(N_DRONES):
        ax.plot(t, log['err'][:, k], color=DRONE_COLORS[k], lw=1.1, label=f'Drone {k}')
    ax.axhline(DELTA_FIT, color='red', ls=':')
    ax.set_title(r'Per-drone slot error $\|p_k - p_k^*\|$ (m)'); ax.set_xlabel('Time (s)')
    ax.legend(fontsize=7, ncol=2)
    ax = axes[1, 0]
    ax.plot(t, log['min_sep'], color='tab:brown', lw=1.4, label='min separation')
    ax.axhline(D_MIN, color='red', ls=':', label=f'$d_{{min}}$ = {D_MIN} m')
    ax.set_ylim(0, None)
    ax.set_title('Minimum inter-drone separation (m)'); ax.set_xlabel('Time (s)')
    ax.legend(fontsize=8)
    ax = axes[1, 1]
    for k in range(N_DRONES):
        ax.plot(t, log['pos'][:, k, 2], color=DRONE_COLORS[k], lw=1.1)
    ax.axhline(Z_ASM, color='grey', ls='--', label=f'$z_{{asm}}$ = {Z_ASM} m')
    ax.set_title('Altitude (m)'); ax.set_xlabel('Time (s)'); ax.legend(fontsize=8)
    fig.legend(handles=[Patch(facecolor=pc[p], label=pl[p]) for p in range(3)],
               loc='lower center', ncol=3, fontsize=9)
    fig.suptitle(title, fontsize=13)
    fig.tight_layout(rect=[0, 0.05, 1, 0.97])
    path = os.path.join(out_dir, name)
    fig.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_monte_carlo(mc, sweep, out_dir):
    fig, axes = plt.subplots(1, 3, figsize=(18, 5))
    ax = axes[0]
    for s in ('indep', 'consensus'):
        ax.hist(1000 * mc[s]['hold_gap_max'], bins=np.linspace(0, 40, 41), color=SCOL[s],
                alpha=0.55, label=f'{LABELS[s]}: max |edge gap|')
        ax.hist(1000 * mc[s]['hold_err_max'], bins=np.linspace(0, 40, 41), color=SCOL[s],
                histtype='step', lw=2, ls='--', label=f'{LABELS[s]}: max slot error')
    ax.axvline(1000 * DELTA_FIT, color='red', ls=':', label='5 mm')
    ax.set_xlabel('Worst value over the scored hold (mm)'); ax.set_ylabel('Seeds')
    ax.set_title(f'Hold-phase fit, {N_MC} seeds (CRN)'); ax.legend(fontsize=7)
    ax = axes[1]
    for s in ('indep', 'consensus'):
        ax.hist(100 * mc[s]['frac_gap_ok'], bins=np.linspace(0, 100, 21), color=SCOL[s],
                alpha=0.55, label=LABELS[s])
    ax.set_xlabel('% of hold time with every edge gap <= 5 mm'); ax.set_ylabel('Seeds')
    ax.set_title('Time inside the 5 mm gap tolerance'); ax.legend(fontsize=8)
    ax = axes[2]
    xs = [1000 * b for b in BIAS_SWEEP]
    for s in ('indep', 'consensus'):
        ax.plot(xs, [1000 * np.mean(sweep[b][s]['hold_gap_max']) for b in BIAS_SWEEP], 'o-',
                color=SCOL[s], label=f'{LABELS[s]}: edge gap')
        ax.plot(xs, [1000 * np.mean(sweep[b][s]['hold_err_max']) for b in BIAS_SWEEP], 's--',
                color=SCOL[s], label=f'{LABELS[s]}: slot error')
    ax.axhline(1000 * DELTA_FIT, color='red', ls=':', label='5 mm')
    ax.set_xlabel('Absolute-position bias std (mm)'); ax.set_ylabel('Mean of worst value (mm)')
    ax.set_title('Sensitivity to the absolute-position bias'); ax.legend(fontsize=7)
    fig.tight_layout()
    path = os.path.join(out_dir, 's097_monte_carlo.png')
    fig.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def gif_indices(n, step):
    return [min(i * step, n - 1) for i in range(math.ceil((n - 1) / step) + 1)]


def save_animation(run, out_dir, step=15):
    log = run['log']
    P, psi, ph, t = log['pos'], log['psi'], log['phase'], log['t']
    n = len(t)
    idx = gif_indices(n, step)
    fig = plt.figure(figsize=(8, 6.5))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim(-5.5, 5.5); ax.set_ylim(-5.5, 5.5); ax.set_zlim(7.5, 10.5)
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
    hx = np.vstack([TARGETS, TARGETS[0]])
    ax.plot(hx[:, 0], hx[:, 1], hx[:, 2], 'k--', lw=1, alpha=0.5)
    ax.scatter(*P_CENTRE, color='green', marker='^', s=60)
    dots = [ax.plot([], [], [], 'o', color=c, ms=7)[0] for c in DRONE_COLORS]
    trails = [ax.plot([], [], [], '-', color=c, lw=0.8, alpha=0.6)[0] for c in DRONE_COLORS]
    polys = [None] * N_DRONES
    names = {0: 'Ascent', 1: 'Consensus', 2: 'Impedance fit + hold'}

    def update(fi):
        k = idx[fi]
        for i in range(N_DRONES):
            dots[i].set_data([P[k, i, 0]], [P[k, i, 1]])
            dots[i].set_3d_properties([P[k, i, 2]])
            lo = max(0, k - 60)
            trails[i].set_data(P[lo:k + 1, i, 0], P[lo:k + 1, i, 1])
            trails[i].set_3d_properties(P[lo:k + 1, i, 2])
            if polys[i] is not None:
                polys[i].remove()
            polys[i] = ax.add_collection3d(Poly3DCollection(
                [piece_triangle(P[k, i], psi[k, i])], color=DRONE_COLORS[i], alpha=0.3))
        ax.set_title(f'S097 aerial puzzle (error consensus, seed 0)\nt = {t[k]:5.2f} s  '
                     f'[{names[int(ph[k])]}]  max slot error = {1000 * log["err"][k].max():6.1f} mm')
        return dots + trails

    anim = animation.FuncAnimation(fig, update, frames=len(idx), interval=50, blit=False)
    path = os.path.join(out_dir, 's097_animation.gif')
    anim.save(path, writer='pillow', fps=20, dpi=70)
    plt.close(fig)
    print(f'Saved: {path}  ({len(idx)} frames, step {step}, {n} samples)')


# ── Main ──────────────────────────────────────────────────────────────────────

def mc_line(tag, m):
    c = m['converged']
    s = f'  {tag:32s} converged {c.sum()}/{len(c)}'
    if c.any():
        s += (f', t_conv {np.nanmean(m["t_conv"]):.2f} ± {np.nanstd(m["t_conv"]):.2f} s'
              f', worst slot error {1000 * np.nanmean(m["hold_err_max"]):.2f} ± '
              f'{1000 * np.nanstd(m["hold_err_max"]):.2f} mm'
              f', worst edge gap {1000 * np.nanmean(m["hold_gap_max"]):.2f} ± '
              f'{1000 * np.nanstd(m["hold_gap_max"]):.2f} mm'
              f', time all gaps <= 5 mm {100 * np.nanmean(m["frac_gap_ok"]):.1f}%'
              f', time slot error <= 5 mm {100 * np.nanmean(m["frac_err_ok"]):.1f}%'
              f', max contact force {np.nanmax(m["fc_max"]):.3f} N'
              f', impedance fit to 5 mm reached in {np.isfinite(m["fit_time"]).sum()}/{len(c)}'
              + (f' (mean {np.nanmean(m["fit_time"]):.2f} s after the switch)'
                 if np.isfinite(m['fit_time']).any() else ''))
    else:
        s += f', final mean radius {np.nanmean(m["final_radius"]):.3f} m'
    s += f', min separation {np.nanmin(m["min_sep"]):.3f} m'
    print(s)


if __name__ == '__main__':
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    print('=== S097 Aerial Puzzle Assembly ===')
    lam = np.sort(np.linalg.eigvalsh(LAP))
    rho = np.abs(1 - EPSILON * lam - ALPHA)
    print('Ring Laplacian eigenvalues: ' + ', '.join(f'{x:.3f}' for x in lam))
    print(f'|1 - eps*lambda - alpha| per eigenvalue: ' + ', '.join(f'{x:.3f}' for x in rho)
          + f' -> spectral radius {rho.max():.3f} (card states 0.70)')
    fp = card_fixed_point()
    print(f'Card absolute-consensus fixed point: slot radius '
          f'{np.linalg.norm(fp[0, :2] - P_CENTRE[:2]):.4f} m (target {R_HEX} m), '
          f'error {1000 * np.linalg.norm(fp - TARGETS, axis=1).max():.1f} mm '
          f'= alpha/(eps*lambda_1 + alpha) scaling {ALPHA / (EPSILON * 1.0 + ALPHA):.4f}')
    wn, zeta = math.sqrt(K_IMP / M_DRONE), B_IMP / (2 * math.sqrt(K_IMP * M_DRONE))
    print(f'Impedance: omega_n = {wn:.3f} rad/s, zeta = {zeta:.3f}; static error under a '
          f'{WIND_COMMON} m/s^2 gust = {1000 * WIND_COMMON * M_DRONE / K_IMP:.1f} mm')
    print(f'Piece side {SIDE:.4f} m; adjacent slot spacing {R_HEX:.3f} m; K_REL = {K_REL:.1f} N/m')

    runs = {s: run_simulation(s, seed=0, record=True) for s in STRATS}
    print('\nSeed 0:')
    for s in STRATS:
        r = runs[s]
        line = (f'  {LABELS[s]:32s} ascent done {r["t_asc"]:.2f} s, '
                f'converged {r["converged"]}')
        if r['converged']:
            line += (f' at {r["t_conv"]:.2f} s, worst slot error over hold '
                     f'{1000 * r["hold_err_max"]:.2f} mm, worst edge gap '
                     f'{1000 * r["hold_gap_max"]:.2f} mm, worst neighbour misalignment '
                     f'{1000 * r["hold_rel_max"]:.2f} mm, impedance fit to 5 mm '
                     + (f'{r["fit_time"]:.2f} s after the switch' if np.isfinite(r['fit_time'])
                        else 'not reached during the hold'))
        else:
            line += (f' (stopped at {T_LIMIT:.0f} s), final mean radius '
                     f'{r["final_radius"]:.4f} m, final error {1000 * r["final_err"]:.1f} mm')
        line += f', min separation {r["min_sep"]:.3f} m'
        print(line)

    print(f'\nStrong-wind check (common gust std {WIND_STRONG} m/s^2, individual 0.05 m/s^2, seed 0): '
          f'consensus-phase stiffness is KP_TRACK*ALPHA = {KP_TRACK * ALPHA:.1f} s^-2, so a '
          f'{WIND_STRONG} m/s^2 gust alone holds a {WIND_STRONG / (KP_TRACK * ALPHA):.2f} m error')
    for s in ('indep', 'consensus'):
        r = run_simulation(s, seed=0, wind_common=WIND_STRONG, wind_indiv=0.05)
        print(f'  {LABELS[s]:32s} converged {r["converged"]}, error at {T_LIMIT:.0f} s '
              f'{1000 * r["final_err"]:.1f} mm')

    print(f'\nMonte Carlo ({N_MC} seeds, same wind / bias / noise / jitter for each strategy):')
    mc = {s: monte_carlo(s) for s in ('indep', 'consensus')}
    for s in mc:
        mc_line(LABELS[s], mc[s])
    gi, gc = mc['indep']['hold_gap_max'], mc['consensus']['hold_gap_max']
    ei, ec = mc['indep']['hold_err_max'], mc['consensus']['hold_err_max']
    print(f'  Worst edge gap: consensus smaller in {(gc < gi).sum()}/{N_MC} seeds '
          f'(mean ratio {np.mean(gc / gi):.3f})')
    print(f'  Worst slot error: consensus smaller in {(ec < ei).sum()}/{N_MC} seeds '
          f'(mean ratio {np.mean(ec / ei):.3f})')
    print(f'  Seeds with worst edge gap <= 5 mm: independent {(gi <= DELTA_FIT).sum()}, '
          f'consensus {(gc <= DELTA_FIT).sum()}; worst slot error <= 5 mm: independent '
          f'{(ei <= DELTA_FIT).sum()}, consensus {(ec <= DELTA_FIT).sum()}')
    tci, tcc = mc['indep']['t_conv'], mc['consensus']['t_conv']
    print(f'  Convergence time: consensus faster in {(tcc < tci).sum()}/{N_MC} seeds')

    print('\nBias sweep (mean over seeds of the worst value, mm):')
    sweep = {}
    for b in BIAS_SWEEP:
        sweep[b] = mc if b == SIGMA_BIAS else {s: monte_carlo(s, sigma_bias=b)
                                              for s in ('indep', 'consensus')}
        print(f'  bias {1000 * b:4.1f} mm: ' + '; '.join(
            f'{LABELS[s]} gap {1000 * np.mean(sweep[b][s]["hold_gap_max"]):.2f}, '
            f'slot {1000 * np.mean(sweep[b][s]["hold_err_max"]):.2f} (max '
            f'{1000 * np.max(sweep[b][s]["hold_err_max"]):.2f}, min '
            f'{1000 * np.min(sweep[b][s]["hold_err_max"]):.2f}), first fit to 5 mm in '
            f'{np.isfinite(sweep[b][s]["fit_time"]).sum()}/{N_MC} '
            f'({np.nanmin(sweep[b][s]["fit_time"]) if np.isfinite(sweep[b][s]["fit_time"]).any() else np.nan:.1f}'
            f'-{np.nanmax(sweep[b][s]["fit_time"]) if np.isfinite(sweep[b][s]["fit_time"]).any() else np.nan:.1f} s '
            f'after switch), worst slot <= 5 mm over the scored hold in '
            f'{int((sweep[b][s]["hold_err_max"] <= DELTA_FIT).sum())}/{N_MC}, scored time with '
            f'slot <= 5 mm {100 * np.mean(sweep[b][s]["frac_err_ok"]):.1f}%'
            for s in ('indep', 'consensus')))

    print('\nWind sweep with no position bias (mean over seeds of the worst value, mm):')
    for wc, wi in WIND_SWEEP:
        r = {s: monte_carlo(s, sigma_bias=0.0, wind=(wc, wi)) for s in ('indep', 'consensus')}
        print(f'  wind common {wc} / individual {wi} m/s^2: ' + '; '.join(
            f'{LABELS[s]} gap {1000 * np.mean(r[s]["hold_gap_max"]):.2f}, '
            f'slot {1000 * np.mean(r[s]["hold_err_max"]):.2f} (max '
            f'{1000 * np.max(r[s]["hold_err_max"]):.2f}), first fit to 5 mm in '
            f'{np.isfinite(r[s]["fit_time"]).sum()}/{N_MC}, worst slot <= 5 mm over the scored '
            f'hold in {int((r[s]["hold_err_max"] <= DELTA_FIT).sum())}/{N_MC}, scored time with '
            f'slot <= 5 mm {100 * np.mean(r[s]["frac_err_ok"]):.1f}%' for s in ('indep', 'consensus')))

    print()
    plot_3d(runs, OUTPUT_DIR)
    plot_metrics(runs['consensus'], OUTPUT_DIR, 's097_metrics_panel.png',
                 'S097 error consensus + coupled impedance fit (seed 0)')
    plot_metrics(runs['card'], OUTPUT_DIR, 's097_card_law_panel.png',
                 'S097 card law: absolute-position consensus stalls at a shrunken hexagon (seed 0)')
    plot_monte_carlo(mc, sweep, OUTPUT_DIR)
    save_animation(runs['consensus'], OUTPUT_DIR)
