"""
S098 3D Upgrade — Swarm Synchronized Dance
==========================================
The S098 clock study (ten drones play a scripted dance at their own drifting local clocks:
random-walk drift 5 ms per 0.5 s, U(-5, 5) ms initial offsets, 1 ms timestamp jitter; no sync,
peer PTP beta 0.5 / 0.9 and the S098 master-referenced PTP) is run on the 3D card's
choreography:

  Act I   (0-15 s)   inner ring r = 8 m at 35 m, outer ring r = 16 m at 55 m, counter-rotating
  Act II  (15-30 s)  double helix, r = 12 m, ten levels from 35 to 65 m
  Act III (30-45 s)  flat star burst at 50 m (radii 8 / 16 m, then x 1.2)
  Act IV  (45-60 s)  10-point sphere lattice, r = 12 m around (0, 0, 50)

Nine keyframes per drone (act boundaries and midpoints), degree-7 minimum-snap segments with zero
velocity, acceleration and jerk at every keyframe (card), slot assignment between formations by
the Hungarian method (added). The card's 3D collision-avoidance layer (repulsion v_rep = 0.5 m/s
inside d_safe = 2 m, recovered with t_rec = 0.2 s) acts on the played-back positions.

The clock offsets delta_k(t) are produced by the S098 functions with the S098 seeds, so they are
the same arrays as in S098: altitude cannot change the clock-sync result itself. What altitude can
change is how a clock offset turns into a position error, an altitude error and a separation
loss; those are the card's 3D claims tested here.

2D baseline (same seeds, same clock offsets): the same choreography flattened to a fixed altitude
z_f (xy paths unchanged), swept over 30 / 50 / 70 m, plus the S098 base choreography itself.

Usage:
    conda activate drones
    python src/05_special_entertainment/3d/s098_3d_synchronized_dance.py
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
from scipy.optimize import linear_sum_assignment

_HERE = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location(
    's098_2d', os.path.join(_HERE, '..', 's098_synchronized_dance.py'))
m2d = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(m2d)

OUTPUT_DIR = os.path.normpath(os.path.join(
    _HERE, '..', '..', '..', 'outputs', '05_special_entertainment', '3d',
    's098_3d_synchronized_dance'))

# ── Parameters (card / S098) ──────────────────────────────────────────────────
N = m2d.N_DRONES
T_SHOW, DT_SIM, DT_PTP = m2d.T_SHOW, m2d.DT_SIM, m2d.DT_PTP
SYNC_TARGET_MS = m2d.SYNC_TARGET_MS
RNG_SEED, N_MC = m2d.RNG_SEED, m2d.N_MC
STRATS, LABELS, COLS = m2d.STRATS, m2d.LABELS, m2d.COLS
T_KF = np.linspace(0.0, T_SHOW, 9)          # card: 9 keyframes (act boundaries + midpoints)
R1, R2, R_SPHERE, R_HELIX = 8.0, 16.0, 12.0, 12.0
Z_LOW, Z_HIGH, Z_MID = 35.0, 55.0, 50.0
Z_HELIX = (35.0, 65.0)
D_SAFE, V_REP, T_REC = 2.0, 0.5, 0.2
ALT_TARGET = 0.5                            # m, card epsilon_alt target
V_TARGET = 0.90
# ── Parameters (added) ────────────────────────────────────────────────────────
BURST = 1.2                                 # Act III radius factor at the midpoint keyframe
ZF_SWEEP = [30.0, 50.0, 70.0]
ACT_COL = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728']
DRONE_COLORS = plt.cm.tab10(np.arange(N))


# ── Choreography ──────────────────────────────────────────────────────────────

def formations():
    """Slot sets (9 keyframes x N x 3) before assignment."""
    a5 = 2 * np.pi * np.arange(5) / 5
    F = []
    for rot in (0.0, np.pi / 5):                                   # Act I, counter-rotating
        inner = np.column_stack([R1 * np.cos(a5 + rot), R1 * np.sin(a5 + rot), np.full(5, Z_LOW)])
        outer = np.column_stack([R2 * np.cos(a5 + np.pi / 5 - rot),
                                 R2 * np.sin(a5 + np.pi / 5 - rot), np.full(5, Z_HIGH)])
        F.append(np.vstack([inner, outer]))
    lev = np.arange(N)
    th = np.pi * (lev % 2) + (lev // 2) * 2 * np.pi / 5
    zl = Z_HELIX[0] + (Z_HELIX[1] - Z_HELIX[0]) * lev / (N - 1)
    for rot in (0.0, 2 * np.pi / 5):                               # Act II, double helix
        F.append(np.column_stack([R_HELIX * np.cos(th + rot), R_HELIX * np.sin(th + rot), zl]))
    a10 = 2 * np.pi * np.arange(N) / N
    rr = np.where(np.arange(N) % 2 == 0, R2, R1)
    for fac, rot in ((1.0, 0.0), (BURST, np.pi / 5)):              # Act III, star burst
        F.append(np.column_stack([fac * rr * np.cos(a10 + rot), fac * rr * np.sin(a10 + rot),
                                  np.full(N, Z_MID)]))
    i = np.arange(N) + 0.5                                         # Act IV, Fibonacci sphere
    zc = 1 - 2 * i / N
    rc = np.sqrt(1 - zc ** 2)
    ph = np.pi * (3 - math.sqrt(5)) * np.arange(N)
    for rot in (0.0, 2 * np.pi / 5, 4 * np.pi / 5):
        F.append(np.column_stack([R_SPHERE * rc * np.cos(ph + rot), R_SPHERE * rc * np.sin(ph + rot),
                                  Z_MID + R_SPHERE * zc]))
    return np.array(F)


def build_keyframes():
    """Assign slots so each drone moves the least (sum of squared distances) at formation
    changes; inside an act the slots just rotate, so the assignment is kept."""
    F = formations()
    kf = np.zeros((N, len(T_KF), 3))
    kf[:, 0] = F[0]
    perm = np.arange(N)
    for n in range(1, len(T_KF)):
        if n in (2, 4, 6):                                         # new formation
            cost = ((kf[:, n - 1][:, None, :] - F[n][None, :, :]) ** 2).sum(-1)
            _, perm = linear_sum_assignment(cost)
        kf[:, n] = F[n][perm]
    return kf


def snap_s(tau, nu=0):
    """Degree-7 rest-to-rest smoothstep and its derivatives in normalised time."""
    if nu == 0:
        return tau ** 4 * (35 - 84 * tau + 70 * tau ** 2 - 20 * tau ** 3)
    if nu == 1:
        return 140 * tau ** 3 * (1 - tau) ** 3
    raise ValueError(nu)


class MinSnapTrack:
    """Piecewise degree-7 minimum-snap track through keyframes (zero v, a, j at keyframes).
    Callable like a scipy CubicSpline: track(t) -> (n, 3), track(t, 1) -> velocity."""

    def __init__(self, t_kf, q_kf):
        self.t, self.q = np.asarray(t_kf), np.asarray(q_kf)

    def __call__(self, t, nu=0):
        t = np.clip(np.atleast_1d(np.asarray(t, float)), self.t[0], self.t[-1])
        i = np.clip(np.searchsorted(self.t, t, 'right') - 1, 0, len(self.t) - 2)
        T = self.t[i + 1] - self.t[i]
        tau = (t - self.t[i]) / T
        dq = self.q[i + 1] - self.q[i]
        if nu == 0:
            return self.q[i] + dq * snap_s(tau)[:, None]
        return dq * (snap_s(tau, 1) / T)[:, None]


def solve_8x8_check(q0, q1, T):
    """Card: eight boundary conditions -> 8 x 8 linear system; compare with the closed form."""
    rows = []
    for s in (0.0, 1.0):
        for d in range(4):
            r = np.zeros(8)
            for kk in range(d, 8):
                r[kk] = math.prod(range(kk - d + 1, kk + 1)) * s ** (kk - d)
            rows.append(r)
    A = np.array(rows)
    b = np.array([q0, 0, 0, 0, q1, 0, 0, 0])
    c = np.linalg.solve(A, b)
    tau = np.linspace(0, 1, 101)
    return np.abs(np.polyval(c[::-1], tau) - (q0 + (q1 - q0) * snap_s(tau))).max()


# ── Playback with the collision-avoidance layer ───────────────────────────────

def playback(tracks, t, delta, card_blend=False):
    """Commanded positions at the local clocks plus the card's repulsion layer.

    Fixed form (used): repulsion offset o_k, o <- a o + dv dt with a = exp(-dt / t_rec), p = p_cmd + o.
    card_blend=True: the card's literal p <- (1 - a) p_cmd + a (p + dv dt), which also low-passes
    the choreography itself."""
    n = len(t)
    cmd = np.stack([tracks[k](np.clip(t + delta[:, k], 0.0, T_SHOW)) for k in range(N)], 1)
    a = math.exp(-DT_SIM / T_REC)
    iu = np.triu_indices(N, 1)
    p = cmd[0].copy()
    o = np.zeros((N, 3))
    act = np.zeros_like(cmd)
    act[0] = p
    n_rep = 0
    for i in range(1, n):
        r = p[:, None, :] - p[None, :, :]
        d = np.linalg.norm(r, axis=2) + np.eye(N) * 1e9
        m = d < D_SAFE
        dv = np.where(m[:, :, None], V_REP * ((D_SAFE - d) / d)[:, :, None] * r, 0.0).sum(1)
        n_rep += int(m[iu].sum())
        if card_blend:
            p = (1 - a) * cmd[i] + a * (p + dv * DT_SIM)
        else:
            o = a * o + dv * DT_SIM
            p = cmd[i] + o
        act[i] = p
    return act, cmd, n_rep


def vmax_tracks(tracks):
    """Largest script speed on a 1 ms grid (cached on the first track object)."""
    if not hasattr(tracks[0], 'vmax_all'):
        td = np.arange(0.0, T_SHOW + 1e-9, 0.001)
        tracks[0].vmax_all = max(np.linalg.norm(tr(td, 1), axis=1).max() for tr in tracks)
    return tracks[0].vmax_all


def run_simulation(tracks, strategy, seed=RNG_SEED, card_blend=False, keep=False):
    noise = m2d.draw_noise(seed)
    t, delta, _ = m2d.simulate_clocks(strategy, noise)
    act, cmd, n_rep = playback(tracks, t, delta, card_blend)
    ide = np.stack([tracks[k](t) for k in range(N)], 1)
    spd = np.stack([np.linalg.norm(tracks[k](t, 1), axis=1) for k in range(N)], 1)
    met = m2d.metrics(t, delta, act, ide, spd)
    iu = np.triu_indices(N, 1)
    d_act = np.linalg.norm(act[:, :, None] - act[:, None, :], axis=3)[:, iu[0], iu[1]]
    d_ide = np.linalg.norm(ide[:, :, None] - ide[:, None, :], axis=3)[:, iu[0], iu[1]]
    d_cmd = np.linalg.norm(cmd[:, :, None] - cmd[:, None, :], axis=3)[:, iu[0], iu[1]]
    alt = np.abs(act[:, :, 2] - ide[:, :, 2])
    # invariants (X4): finite, nobody below 25 m (volume 30-70 m), no teleport
    assert np.all(np.isfinite(act)), 'non-finite position'
    assert act[:, :, 2].min() > 25.0, 'left the show volume downwards'
    vmax = vmax_tracks(tracks)
    jump = np.linalg.norm(np.diff(act, axis=0), axis=2)
    dstep = np.abs(np.diff(delta, axis=0))
    bound = vmax * (DT_SIM + dstep) + V_REP * D_SAFE * DT_SIM * N + 1e-9
    assert np.all(jump <= bound), 'position jump exceeds the speed bound'
    out = dict(max_abs=met['err_abs'].max(), max_rel=met['err_rel'].max(),
               max_pos=met['pos_err'].max(), meanV=met['V'].mean(), minV=met['V'].min(),
               alt_mean_max=alt.mean(1).max(), alt_max=alt.max(),
               frac_alt_ok=(alt.mean(1) < ALT_TARGET).mean(),
               min_sep=d_act.min(), min_sep_cmd=d_cmd.min(), min_sep_script=d_ide.min(),
               sep_loss=(d_ide - d_cmd).max(), viol_time=(d_act.min(1) < D_SAFE).sum() * DT_SIM,
               viol_time_cmd=(d_cmd.min(1) < D_SAFE).sum() * DT_SIM, n_rep=n_rep,
               rep_shift=np.linalg.norm(act - cmd, axis=2).max())
    if keep:
        out.update(t=t, delta=delta, act=act, ide=ide, cmd=cmd, alt=alt, d_act=d_act,
                   err_abs=met['err_abs'], V=met['V'], pos_err=met['pos_err'])
    return out


def monte_carlo(tracks, strategy, n=N_MC):
    rows = [run_simulation(tracks, strategy, seed=1000 + s) for s in range(n)]
    return {k: np.array([r[k] for r in rows]) for k in rows[0]}


# ── Plots ─────────────────────────────────────────────────────────────────────

def plot_choreography(tracks, kf, out_dir):
    fig = plt.figure(figsize=(9, 8))
    ax = fig.add_subplot(111, projection='3d')
    for a in range(4):
        tt = np.linspace(15 * a, 15 * (a + 1), 150)
        for k in range(N):
            P = tracks[k](tt)
            ax.plot(P[:, 0], P[:, 1], P[:, 2], color=ACT_COL[a], lw=1.0,
                    label=f'Act {"I II III IV".split()[a]}' if k == 0 else None)
    ax.scatter(kf[:, :, 0].ravel(), kf[:, :, 1].ravel(), kf[:, :, 2].ravel(), c='k', s=8,
               label='keyframes')
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)'); ax.set_zlabel('z (m)')
    ax.set_xlim(-20, 20); ax.set_ylim(-20, 20); ax.set_zlim(30, 70)
    ax.set_title('S098 3D choreography: rings / double helix / star burst / sphere lattice')
    ax.legend(fontsize=8)
    path = os.path.join(out_dir, 'choreography_3d.png')
    fig.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_kpis(res, res_flat, out_dir):
    fig, axes = plt.subplots(2, 2, figsize=(15, 9), sharex=True)
    for s in STRATS:
        r, rf = res[s], res_flat[s]
        axes[0, 0].plot(r['t'], r['err_abs'], color=COLS[s], label=LABELS[s])
        axes[0, 1].plot(r['t'], 100 * r['alt'].mean(1), color=COLS[s], label=LABELS[s])
        axes[1, 0].plot(r['t'], 100 * r['pos_err'].max(1), color=COLS[s], label=f'{LABELS[s]} 3D')
        axes[1, 0].plot(rf['t'], 100 * rf['pos_err'].max(1), ':', color=COLS[s], lw=1)
        axes[1, 1].plot(r['t'], r['d_act'].min(1), color=COLS[s], label=f'{LABELS[s]} 3D')
        axes[1, 1].plot(rf['t'], rf['d_act'].min(1), ':', color=COLS[s], lw=1)
    iu = np.triu_indices(N, 1)
    ide = res['no_sync']['ide']
    axes[1, 1].plot(res['no_sync']['t'], np.linalg.norm(
        ide[:, :, None] - ide[:, None, :], axis=3)[:, iu[0], iu[1]].min(1), 'k--', lw=1,
        label='script (perfect clocks)')
    axes[0, 0].axhline(SYNC_TARGET_MS, color='green', ls='--', label='10 ms')
    axes[0, 0].set_ylabel('max |delta_k| (ms)')
    axes[0, 0].set_title('Card sync KPI (identical to S098: same clock model and seeds)')
    axes[0, 1].axhline(100 * ALT_TARGET, color='green', ls='--', label='0.5 m target')
    axes[0, 1].set_ylabel('epsilon_alt (cm)')
    axes[0, 1].set_title('Altitude-layer error (mean over drones)')
    axes[1, 0].set_ylabel('max 3D position error (cm)')
    axes[1, 0].set_title('Position error: solid = 3D show, dotted = flattened show (z = 50 m)')
    axes[1, 1].axhline(D_SAFE, color='red', ls='--', label='d_safe 2 m')
    axes[1, 1].set_ylabel('min pairwise separation (m)')
    axes[1, 1].set_title('Minimum separation: solid = 3D show, dotted = flattened show')
    axes[1, 1].set_ylim(0, None)
    for ax in axes.flat:
        ax.legend(fontsize=7)
        for x in (15, 30, 45):
            ax.axvline(x, color='grey', lw=0.5)
    for ax in axes[1]:
        ax.set_xlabel('Global time (s)')
    fig.suptitle(f'S098 3D KPIs, seed {RNG_SEED} (vertical lines = act boundaries)')
    fig.tight_layout()
    path = os.path.join(out_dir, 'kpi_timeseries.png')
    fig.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_altitude(res, out_dir):
    fig, ax = plt.subplots(figsize=(13, 5))
    r = res['peer_05']
    for k in range(N):
        ax.plot(r['t'], r['ide'][:, k, 2], color=DRONE_COLORS[k], lw=1.6, label=f'drone {k}')
        ax.plot(r['t'], r['act'][:, k, 2], ':', color='k', lw=0.6)
    for x in (15, 30, 45):
        ax.axvline(x, color='grey', lw=0.8)
    ax.set_xlabel('Global time (s)'); ax.set_ylabel('z (m)')
    ax.set_title('Altitude of every drone (coloured = script, black dotted = played back under '
                 'peer PTP beta = 0.5; the difference is below the line width)')
    ax.legend(fontsize=7, ncol=5)
    path = os.path.join(out_dir, 'altitude_timeseries.png')
    fig.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_monte_carlo(mc, mcf, mcb, out_dir):
    fig, axes = plt.subplots(1, 3, figsize=(18, 5))
    x = np.arange(len(STRATS))
    w = 0.27
    for j, (m, lab) in enumerate([(mc, '3D show'), (mcf, 'flattened (z = 50 m)'),
                                  (mcb, 'S098 base show')]):
        axes[0].bar(x + (j - 1) * w, [100 * np.mean(m[s]['max_pos']) for s in STRATS], w,
                    label=lab)
    axes[0].set_xticks(x); axes[0].set_xticklabels([LABELS[s] for s in STRATS], fontsize=8)
    axes[0].set_ylabel('per-show max position error (cm), mean')
    axes[0].set_title(f'Position error, {N_MC} shows')
    axes[0].legend(fontsize=8)
    axes[1].boxplot([100 * mc[s]['alt_mean_max'] for s in STRATS],
                    tick_labels=[LABELS[s] for s in STRATS])
    axes[1].axhline(100 * ALT_TARGET, color='green', ls='--', label='card target 0.5 m')
    axes[1].set_ylabel('per-show max epsilon_alt (cm)')
    axes[1].set_title('Altitude-layer error (3D show)')
    axes[1].tick_params(axis='x', labelsize=8)
    axes[1].legend(fontsize=8)
    axes[2].boxplot([100 * mc[s]['sep_loss'] for s in STRATS],
                    tick_labels=[LABELS[s] for s in STRATS])
    axes[2].set_ylabel('largest separation loss vs script (cm)')
    axes[2].set_title('Separation lost to clock errors (3D show)')
    axes[2].tick_params(axis='x', labelsize=8)
    fig.tight_layout()
    path = os.path.join(out_dir, 'monte_carlo.png')
    fig.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def save_animation(res, out_dir, step=8, mag=20.0):
    r0, r1 = res['no_sync'], res['master_05']
    n = len(r0['t'])
    idx = [min(i * step, n - 1) for i in range(math.ceil((n - 1) / step) + 1)]
    fig = plt.figure(figsize=(14, 5.5))
    axs = [fig.add_subplot(1, 3, 1, projection='3d'), fig.add_subplot(1, 3, 2, projection='3d'),
           fig.add_subplot(1, 3, 3)]

    def update(fi):
        k = idx[fi]
        for ax, r, name in ((axs[0], r0, 'no sync'), (axs[1], r1, 'master PTP')):
            ax.cla()
            ide, act = r['ide'][k], r['act'][k]
            ax.scatter(ide[:, 0], ide[:, 1], ide[:, 2], c='lightgrey', s=30)
            ax.scatter(act[:, 0], act[:, 1], act[:, 2], c=DRONE_COLORS, s=22)
            for j in range(N):
                e = ide[j] + mag * (act[j] - ide[j])
                ax.plot([ide[j, 0], e[0]], [ide[j, 1], e[1]], [ide[j, 2], e[2]], 'k-', lw=0.8)
            ax.set_xlim(-20, 20); ax.set_ylim(-20, 20); ax.set_zlim(30, 70)
            ax.set_title(f'{name}: error x{mag:.0f}, max {100 * r["pos_err"][k].max():.1f} cm',
                         fontsize=9)
        axs[2].cla()
        xx = np.arange(N)
        axs[2].bar(xx - 0.2, 100 * r0['alt'][k], 0.4, color='crimson', label='no sync')
        axs[2].bar(xx + 0.2, 100 * r1['alt'][k], 0.4, color='seagreen', label='master PTP')
        axs[2].set_ylim(0, 60)
        axs[2].set_xlabel('drone'); axs[2].set_ylabel('|z error| (cm)')
        axs[2].axhline(100 * ALT_TARGET, color='green', ls='--', lw=0.8)
        axs[2].legend(fontsize=7)
        axs[2].set_title(f't = {r0["t"][k]:5.2f} s', fontsize=9)
        return []

    anim = animation.FuncAnimation(fig, update, frames=len(idx), interval=100, blit=False)
    path = os.path.join(out_dir, 'sync_dance_animation.gif')
    anim.save(path, writer='pillow', fps=10, dpi=55)
    plt.close(fig)
    print(f'Saved: {path}  ({len(idx)} frames, step {step}, {n} samples)')


# ── Main ──────────────────────────────────────────────────────────────────────

def fmt(m):
    return (f'max KPI {np.mean(m["max_abs"]):.1f} ms (worst {np.max(m["max_abs"]):.1f}); max pos err '
            f'{100 * np.mean(m["max_pos"]):.1f} cm (worst {100 * np.max(m["max_pos"]):.1f}); max eps_alt '
            f'{100 * np.mean(m["alt_mean_max"]):.1f} cm (worst {100 * np.max(m["alt_mean_max"]):.1f}, '
            f'shows with eps_alt < 0.5 m throughout {int(np.sum(m["alt_mean_max"] < ALT_TARGET))}); '
            f'worst single-drone |z err| {100 * np.max(m["alt_max"]):.1f} cm; mean V '
            f'{100 * np.mean(m["meanV"]):.2f}%, min V {100 * np.min(m["minV"]):.1f}%; min separation '
            f'{np.min(m["min_sep"]):.3f} m (script {np.min(m["min_sep_script"]):.3f} m), separation '
            f'loss max {100 * np.max(m["sep_loss"]):.1f} cm (mean {100 * np.mean(m["sep_loss"]):.1f}), '
            f'shows with a violation {int(np.sum(m["viol_time"] > 0))}, repulsion pair-steps '
            f'{int(np.sum(m["n_rep"]))}')


def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    print('=== S098 3D Swarm Synchronized Dance ===')
    print(f'Card min-snap: 8 x 8 boundary system vs closed form, max difference '
          f'{solve_8x8_check(0.0, 10.0, 1.0):.2e} m (the eight conditions fix the polynomial; '
          f'nothing is left to minimise); peak speed = {snap_s(0.5, 1):.4f} |dq| / T')
    kf = build_keyframes()
    tracks = [MinSnapTrack(T_KF, kf[k]) for k in range(N)]
    kf_flat = kf.copy()
    kf_flat[:, :, 2] = Z_MID
    tracks_flat = [MinSnapTrack(T_KF, kf_flat[k]) for k in range(N)]
    t_b, wp_b = m2d.build_choreography()
    tracks_base = m2d.build_splines(t_b, wp_b)
    td = np.arange(0.0, T_SHOW + 1e-9, 0.01)
    iu = np.triu_indices(N, 1)
    for name, tr in (('3D show', tracks), ('flattened (z = 50 m)', tracks_flat),
                     ('S098 base show', tracks_base)):
        P = np.stack([tr[k](td) for k in range(N)], 1)
        V = np.stack([tr[k](td, 1) for k in range(N)], 1)
        dmin = np.linalg.norm(P[:, :, None] - P[:, None, :], axis=3)[:, iu[0], iu[1]].min(1)
        print(f'  {name:22s}: max speed {np.linalg.norm(V, axis=2).max():.2f} m/s, max |v_z| '
              f'{np.abs(V[:, :, 2]).max():.2f} m/s, max horizontal speed '
              f'{np.linalg.norm(V[:, :, :2], axis=2).max():.2f} m/s; scripted min separation '
              f'{dmin.min():.2f} m at t = {td[np.argmin(dmin)]:.2f} s; time below d_safe '
              f'{(dmin < D_SAFE).sum() * 0.01:.2f} s; z range {P[:, :, 2].min():.1f}-'
              f'{P[:, :, 2].max():.1f} m; |x|, |y| <= {np.abs(P[:, :, :2]).max():.1f} m')
    for a in range(4):
        sl = (td >= 15 * a) & (td <= 15 * (a + 1))
        P = np.stack([tracks[k](td[sl]) for k in range(N)], 1)
        V = np.stack([tracks[k](td[sl], 1) for k in range(N)], 1)
        dmin = np.linalg.norm(P[:, :, None] - P[:, None, :], axis=3)[:, iu[0], iu[1]].min()
        print(f'    Act {"I II III IV".split()[a]:3s}: max speed {np.linalg.norm(V, axis=2).max():.2f} '
              f'm/s, max |v_z| {np.abs(V[:, :, 2]).max():.2f} m/s, scripted min separation {dmin:.2f} m')

    # the card's literal blend low-passes the choreography (perfect clocks: master strategy is
    # irrelevant, use the clock offsets set to zero through a fake strategy-free playback)
    t0 = np.arange(int(round(T_SHOW / DT_SIM)) + 1) * DT_SIM
    z0 = np.zeros((len(t0), N))
    act_l, cmd_l, _ = playback(tracks, t0, z0, card_blend=True)
    act_f, cmd_f, _ = playback(tracks, t0, z0, card_blend=False)
    print(f'Card literal recovery blend with perfect clocks: max position error '
          f'{100 * np.linalg.norm(act_l - cmd_l, axis=2).max():.1f} cm (a {T_REC} s first-order lag '
          f'of the whole choreography); fixed offset filter: '
          f'{100 * np.linalg.norm(act_f - cmd_f, axis=2).max():.1f} cm')

    res = {s: run_simulation(tracks, s, keep=True) for s in STRATS}
    res_flat = {s: run_simulation(tracks_flat, s, keep=True) for s in STRATS}
    same = all(np.array_equal(res[s]['delta'], m2d.run_simulation(tracks_base, s)['delta'])
               for s in STRATS)
    print(f'\nSeed {RNG_SEED} (clock offsets identical to S098 for every strategy: {same}):')
    assert same
    for s in STRATS:
        for tag, r in (('3D  ', res[s]), ('flat', res_flat[s])):
            print(f'  {LABELS[s]:18s} {tag}: max KPI {r["max_abs"]:.1f} ms, max pos err '
                  f'{100 * r["max_pos"]:.1f} cm, max eps_alt {100 * r["alt_mean_max"]:.1f} cm, '
                  f'worst |z err| {100 * r["alt_max"]:.1f} cm, mean V {100 * r["meanV"]:.2f}%, '
                  f'min V {100 * r["minV"]:.1f}%, min separation {r["min_sep"]:.3f} m (script '
                  f'{r["min_sep_script"]:.3f} m), separation loss {100 * r["sep_loss"]:.1f} cm, '
                  f'violation {r["viol_time"]:.2f} s, repulsion shift {100 * r["rep_shift"]:.1f} cm')
    lit = run_simulation(tracks, 'no_sync', card_blend=True)
    print(f'  card literal blend, no sync: max pos err {100 * lit["max_pos"]:.1f} cm, max eps_alt '
          f'{100 * lit["alt_mean_max"]:.1f} cm, mean V {100 * lit["meanV"]:.2f}%')

    print(f'\nMonte Carlo ({N_MC} shows, seeds 1000-{999 + N_MC}, same clock draws as S098):')
    mc = {s: monte_carlo(tracks, s) for s in STRATS}
    mcf = {s: monte_carlo(tracks_flat, s) for s in STRATS}
    mcb = {}
    for s in STRATS:
        rows = [m2d.summary(m2d.run_simulation(tracks_base, s, seed=1000 + i)) for i in range(N_MC)]
        mcb[s] = {k: np.array([r[k] for r in rows]) for k in rows[0]}
    for s in STRATS:
        print(f'  {LABELS[s]:18s} 3D show:  ' + fmt(mc[s]))
        print(f'  {LABELS[s]:18s} flat z50: ' + fmt(mcf[s]))
        print(f'  {LABELS[s]:18s} S098 base show: max pos err {100 * np.mean(mcb[s]["max_pos"]):.1f}'
              f' cm (worst {100 * np.max(mcb[s]["max_pos"]):.1f}), mean V '
              f'{100 * np.mean(mcb[s]["meanV"]):.2f}%, min V {100 * np.min(mcb[s]["minV"]):.1f}%')
        ratio = mc[s]['max_pos'] / mcf[s]['max_pos']
        print(f'    3D / flattened per-show max position error: mean ratio {np.mean(ratio):.3f} '
              f'(min {np.min(ratio):.3f}, max {np.max(ratio):.3f}); 3D larger in '
              f'{int(np.sum(ratio > 1))}/{N_MC}; 3D vs S098 base show: larger in '
              f'{int(np.sum(mc[s]["max_pos"] > mcb[s]["max_pos"]))}/{N_MC}')
    a, b = mc['no_sync']['alt_mean_max'], mc['master_05']['alt_mean_max']
    print(f'  eps_alt: master smaller than no sync in {int(np.sum(b < a))}/{N_MC} shows')
    for s in STRATS:
        print(f'  sync ranking check {LABELS[s]:18s}: per-show max KPI 3D == S098 base in '
              f'{int(np.sum(mc[s]["max_abs"] == mcb[s]["max_abs"]))}/{N_MC} shows')

    print('\n2D baseline altitude sweep (flattened show at z_f; same seeds, no sync and master):')
    for zf in ZF_SWEEP:
        kz = kf.copy()
        kz[:, :, 2] = zf
        tz = [MinSnapTrack(T_KF, kz[k]) for k in range(N)]
        for s in ('no_sync', 'master_05'):
            m = monte_carlo(tz, s, n=50)
            print(f'  z_f {zf:4.1f} m, {LABELS[s]:16s}: max pos err {100 * np.mean(m["max_pos"]):.2f} cm, '
                  f'min separation {np.min(m["min_sep"]):.3f} m (script '
                  f'{np.min(m["min_sep_script"]):.3f} m), shows with a violation '
                  f'{int(np.sum(m["viol_time"] > 0))}/50')

    print()
    plot_choreography(tracks, kf, OUTPUT_DIR)
    plot_kpis(res, res_flat, OUTPUT_DIR)
    plot_altitude(res, OUTPUT_DIR)
    plot_monte_carlo(mc, mcf, mcb, OUTPUT_DIR)
    save_animation(res, OUTPUT_DIR)


if __name__ == '__main__':
    main()
