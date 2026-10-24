"""
S098 Swarm Synchronized Dance
=============================
Ten drones fly a 60 s scripted dance at 50 m. Each drone plays its cubic-spline track at its own
local clock, and every clock drifts as a random walk (5 ms std per 0.5 s). Every 0.5 s the drones
exchange timestamps (1 ms jitter) and correct their clocks.

Strategies (same drift and jitter draws for all; CRN):
  1. No sync           - clocks drift freely.
  2. Peer PTP b=0.5    - card consensus: EMA (beta = 0.5) of the mean peer offset, sign corrected.
  3. Peer PTP b=0.9    - same with beta = 0.9.
  4. Master PTP b=0.5  - (added) every drone corrects toward a GNSS-disciplined ground master
                         clock instead of the peer mean.
The card's literal update (clock moved by -EMA of the peer offset) is run once to show it
diverges.

The clocks only change which point of the script a drone flies to; the drones are assumed to
track their commanded spline position exactly (kinematic playback, as in the card).

Usage:
    conda activate drones
    python src/05_special_entertainment/s098_synchronized_dance.py
"""

import os
import math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401
from scipy.interpolate import CubicSpline

# ── Parameters (card) ─────────────────────────────────────────────────────────
N_DRONES        = 10
T_SHOW          = 60.0
LED_HEIGHT      = 50.0
DT_SIM          = 0.05
DT_PTP          = 0.5
SIGMA_DRIFT     = 5e-3          # s per PTP interval
SYNC_TARGET_MS  = 10.0
BETA_STANDARD   = 0.5
BETA_AGGRESSIVE = 0.9
N_WP            = 13
RNG_SEED        = 42

# ── Parameters (added) ────────────────────────────────────────────────────────
SIGMA_TS        = 1e-3          # s  timestamp jitter per exchanged message
N_MC            = 200           # Monte Carlo shows
DT_SWEEP        = [0.1, 0.25, 0.5, 1.0]   # PTP interval sweep (drift rate kept the same)
STRATS = ['no_sync', 'peer_05', 'peer_09', 'master_05']
LABELS = {'no_sync': 'No sync', 'peer_05': 'Peer PTP β=0.5', 'peer_09': 'Peer PTP β=0.9',
          'master_05': 'Master PTP β=0.5', 'card_literal': 'Card update (literal)'}
COLS = {'no_sync': 'crimson', 'peer_05': 'steelblue', 'peer_09': 'darkorange',
        'master_05': 'seagreen', 'card_literal': 'grey'}

OUTPUT_DIR = os.path.normpath(os.path.join(
    os.path.dirname(__file__), '..', '..',
    'outputs', '05_special_entertainment', 's098_synchronized_dance'))


# ── Choreography ──────────────────────────────────────────────────────────────

def build_choreography():
    """Card choreography: two rings (8 m and 16 m) with phase, radius and altitude modulation."""
    t_wp = np.linspace(0.0, T_SHOW, N_WP)
    wp = np.zeros((N_DRONES, N_WP, 3))
    for k in range(N_DRONES):
        ring = 0 if k < 5 else 1
        r_base = 8.0 if ring == 0 else 16.0
        phi0 = 2 * np.pi * k / 5 + ring * np.pi / 5
        for n, t in enumerate(t_wp):
            phase = phi0 + 0.5 * np.pi * np.sin(2 * np.pi * t / T_SHOW)
            r = r_base + 3.0 * np.sin(4 * np.pi * t / T_SHOW + phi0)
            z = LED_HEIGHT + 5.0 * np.sin(2 * np.pi * t / T_SHOW + k * np.pi / N_DRONES)
            wp[k, n] = [r * np.cos(phase), r * np.sin(phase), z]
    return t_wp, wp


def build_splines(t_wp, wp):
    # The card text says "natural" spline but its code uses not-a-knot; the code is followed.
    return [CubicSpline(t_wp, wp[k], axis=0, bc_type='not-a-knot') for k in range(N_DRONES)]


# ── Clock simulation ──────────────────────────────────────────────────────────

def draw_noise(seed, dt_ptp=DT_PTP):
    """Drift increments per sim step and timestamp jitter per PTP event (CRN)."""
    rng = np.random.default_rng(seed)
    n_steps = int(round(T_SHOW / DT_SIM)) + 1
    n_ptp = int(round(T_SHOW / dt_ptp)) + 1
    return dict(
        delta0=rng.uniform(-5e-3, 5e-3, N_DRONES),
        drift=rng.standard_normal((n_steps, N_DRONES)),
        jitter=rng.standard_normal((n_ptp, N_DRONES, N_DRONES)),
        jitter_m=rng.standard_normal((n_ptp, N_DRONES)),
    )


def simulate_clocks(strategy, noise, dt_ptp=DT_PTP):
    """Return times (n,), offsets delta (n, N) in s, and PTP event times.

    Drift: continuous random walk, variance SIGMA_DRIFT^2 per DT_PTP (= per 0.5 s), applied in
    every DT_SIM step, so the drift rate does not depend on the PTP interval.
    """
    n_steps = int(round(T_SHOW / DT_SIM)) + 1
    t = np.arange(n_steps) * DT_SIM
    delta = noise['delta0'].copy()
    dhat = np.zeros(N_DRONES)
    beta = {'no_sync': 0.0, 'peer_05': BETA_STANDARD, 'peer_09': BETA_AGGRESSIVE,
            'master_05': BETA_STANDARD, 'card_literal': BETA_STANDARD}[strategy]
    step_sigma = SIGMA_DRIFT * math.sqrt(DT_SIM / DT_PTP)
    every = int(round(dt_ptp / DT_SIM))
    log = np.zeros((n_steps, N_DRONES))
    events = []
    j = 0
    for i in range(n_steps):
        if i > 0:
            delta = delta + step_sigma * noise['drift'][i]
        if strategy != 'no_sync' and i > 0 and i % every == 0:
            j += 1
            if strategy == 'master_05':
                # offset of the master (global time) seen by k: 0 - delta_k + jitter
                Delta = -delta + SIGMA_TS * noise['jitter_m'][j]
            else:
                # timestamp of peer m as seen by k: delta_m - delta_k + jitter (k != m)
                M = delta[None, :] - delta[:, None] + SIGMA_TS * noise['jitter'][j]
                np.fill_diagonal(M, 0.0)
                Delta = M.sum(axis=1) / (N_DRONES - 1)
            dhat = (1 - beta) * dhat + beta * Delta
            if strategy == 'card_literal':
                delta = delta - dhat               # card: delta_k <- delta_k - dhat_k
            else:
                delta = delta + dhat               # move toward the peers / master
            events.append(t[i])
        log[i] = delta
    return t, log, events


def playback(splines, t, delta):
    """Actual and ideal positions (n, N, 3) and ideal speeds (n, N)."""
    n = len(t)
    act = np.zeros((n, N_DRONES, 3))
    ide = np.zeros((n, N_DRONES, 3))
    spd = np.zeros((n, N_DRONES))
    for k in range(N_DRONES):
        act[:, k] = splines[k](np.clip(t + delta[:, k], 0.0, T_SHOW))
        ide[:, k] = splines[k](t)
        spd[:, k] = np.linalg.norm(splines[k](t, 1), axis=1)
    return act, ide, spd


def metrics(t, delta, act, ide, spd):
    err_abs = np.abs(delta).max(axis=1) * 1e3                           # ms, card KPI
    err_rel = np.abs(delta - delta.mean(axis=1, keepdims=True)).max(axis=1) * 1e3
    pos_err = np.linalg.norm(act - ide, axis=2)                          # (n, N) m
    d_move = np.maximum(spd.max(axis=1) * DT_PTP, 0.01)                  # card definition
    vk = np.clip(1.0 - pos_err / d_move[:, None], 0.0, 1.0)
    V = vk.mean(axis=1)
    # card code variant: each drone normalised by its own speed
    vk_pd = np.clip(1.0 - pos_err / np.maximum(spd * DT_PTP, 0.01), 0.0, 1.0)
    return dict(err_abs=err_abs, err_rel=err_rel, pos_err=pos_err, V=V, V_pd=vk_pd.mean(axis=1))


_VMAX = {}


def vmax_dense(splines):
    """Largest script speed on a 1 ms grid (bound for the invariant checks)."""
    key = id(splines)
    if key not in _VMAX:
        td = np.arange(0.0, T_SHOW + 1e-9, 0.001)
        _VMAX[key] = max(np.linalg.norm(sp(td, 1), axis=1).max() for sp in splines)
    return _VMAX[key]


def run_simulation(splines, strategy, seed=RNG_SEED, dt_ptp=DT_PTP):
    noise = draw_noise(seed, dt_ptp)
    t, delta, events = simulate_clocks(strategy, noise, dt_ptp)
    act, ide, spd = playback(splines, t, delta)
    out = dict(t=t, delta=delta, events=events, act=act, ide=ide, spd=spd)
    out.update(metrics(t, delta, act, ide, spd))
    # invariants (X4), checked where they can actually fail:
    # 1. finite clock offsets;
    # 2. every local clock moves forward each step (dt + clock step > 0): a PTP correction must
    #    never make a drone play its script backwards (fails if a step exceeds -dt = -50 ms);
    # 3. the commanded position moves at most v_max * (dt + |clock step|) per step, i.e. a clock
    #    step can only move the drone along its own script by v_max * |clock step|
    jump = np.linalg.norm(np.diff(act, axis=0), axis=2)
    dlocal = np.diff(t[:, None] + delta, axis=0)
    dstep = np.abs(np.diff(delta, axis=0))
    out['max_jump'] = float(jump.max())
    out['max_clock_step'] = float(dstep.max())
    out['min_local_step'] = float(dlocal.min())
    if strategy != 'card_literal':                       # the card's update diverges by design
        assert np.all(np.isfinite(delta)), 'non-finite clock offset'
        assert np.all(dlocal > 0), 'a local clock ran backwards (script played in reverse)'
        bound = vmax_dense(splines) * (DT_SIM + dstep) * (1 + 1e-6) + 1e-9
        assert np.all(jump <= bound), 'commanded position jump exceeds v_max * time step'
    return out


def summary(r):
    return dict(max_abs=r['err_abs'].max(), max_rel=r['err_rel'].max(),
                frac_abs_ok=(r['err_abs'] < SYNC_TARGET_MS).mean(),
                frac_rel_ok=(r['err_rel'] < SYNC_TARGET_MS).mean(),
                meanV=r['V'].mean(), minV=r['V'].min(), max_pos=r['pos_err'].max(),
                meanV_pd=r['V_pd'].mean(), minV_pd=r['V_pd'].min(),
                rms_abs=np.sqrt(np.mean(r['delta'] ** 2)) * 1e3)


def monte_carlo(splines, strategy, dt_ptp=DT_PTP, n=N_MC):
    rows = [summary(run_simulation(splines, strategy, seed=1000 + s, dt_ptp=dt_ptp))
            for s in range(n)]
    return {k: np.array([r[k] for r in rows]) for k in rows[0]}


# ── Plots ─────────────────────────────────────────────────────────────────────

def plot_sync_error(res, out_dir):
    fig, axes = plt.subplots(2, 1, figsize=(11, 7.5), sharex=True)
    for s in STRATS:
        axes[0].plot(res[s]['t'], res[s]['err_abs'], color=COLS[s], lw=1.6, label=LABELS[s])
        axes[1].plot(res[s]['t'], res[s]['err_rel'], color=COLS[s], lw=1.6, label=LABELS[s])
    for ax in axes:
        ax.axhline(SYNC_TARGET_MS, color='green', ls='--', lw=1.4, label='Target 10 ms')
        ax.set_ylim(0, None)
        ax.set_ylabel('ms')
        ax.legend(fontsize=8, ncol=3)
    axes[0].set_title(r'Card KPI $\varepsilon_{sync}(t) = \max_k |\delta_k|$ (vs global time), '
                      f'seed {RNG_SEED}')
    axes[1].set_title(r'Relative error $\max_k |\delta_k - \bar\delta|$ (what the audience sees '
                      'between drones)')
    axes[1].set_xlabel('Global time t (s)')
    axes[1].set_xlim(0, T_SHOW)
    fig.tight_layout()
    path = os.path.join(out_dir, 'sync_error.png')
    fig.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_visual_fidelity(res, out_dir):
    fig, axes = plt.subplots(1, 2, figsize=(15, 4.5))
    for s in STRATS:
        axes[0].plot(res[s]['t'], 100 * res[s]['V'], color=COLS[s], lw=1.5, label=LABELS[s])
        axes[1].plot(res[s]['t'], 100 * res[s]['pos_err'].max(axis=1), color=COLS[s], lw=1.5,
                     label=LABELS[s])
    axes[0].axhline(90, color='green', ls='--', label='Acceptable 90%')
    axes[0].set_ylim(80, 100.5)
    axes[0].set_ylabel('V(t) (%)'); axes[0].set_title('Card visual fidelity V(t)')
    axes[1].set_ylabel('cm'); axes[1].set_title('Largest position error vs ideal script')
    for ax in axes:
        ax.set_xlabel('Global time t (s)'); ax.set_xlim(0, T_SHOW); ax.legend(fontsize=8)
    fig.tight_layout()
    path = os.path.join(out_dir, 'visual_fidelity.png')
    fig.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_clock_offsets(res, out_dir):
    fig, axes = plt.subplots(1, 2, figsize=(15, 4.5), sharey=True)
    cmap = plt.get_cmap('tab10')
    for ax, s in zip(axes, ['peer_05', 'master_05']):
        r = res[s]
        for k in range(N_DRONES):
            ax.plot(r['t'], 1e3 * r['delta'][:, k], color=cmap(k), lw=1.0, label=f'Drone {k + 1}')
        ax.plot(r['t'], 1e3 * r['delta'].mean(axis=1), 'k-', lw=2.2, label='Swarm mean')
        for ev in r['events'][::4]:
            ax.axvline(ev, color='grey', lw=0.3, ls=':')
        ax.axhline(0, color='k', lw=0.6)
        ax.set_title(f'Clock offsets — {LABELS[s]} (dotted: every 4th PTP event)')
        ax.set_xlabel('Global time t (s)'); ax.set_xlim(0, T_SHOW)
    axes[0].set_ylabel(r'$\delta_k$ (ms)')
    axes[0].legend(fontsize=7, ncol=4, loc='lower left')
    fig.tight_layout()
    path = os.path.join(out_dir, 'clock_offsets.png')
    fig.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_choreography(splines, out_dir):
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    cmap = plt.get_cmap('tab10')
    td = np.linspace(0, T_SHOW, 300)
    for k in range(N_DRONES):
        tr = splines[k](td)
        ax.plot(tr[:, 0], tr[:, 1], tr[:, 2], color=cmap(k), lw=1.4, label=f'Drone {k + 1}')
        ax.scatter(*tr[0], color=cmap(k), s=40)
        ax.scatter(*tr[-1], color=cmap(k), s=40, marker='^')
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
    ax.set_title('Ideal choreography, all 10 tracks (o = start, ^ = end)')
    ax.legend(fontsize=7, ncol=2, loc='upper left')
    path = os.path.join(out_dir, 'choreography_3d.png')
    fig.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_monte_carlo(mc, sweep, out_dir):
    fig, axes = plt.subplots(1, 3, figsize=(18, 5))
    ax = axes[0]
    data = [mc[s]['max_abs'] for s in STRATS]
    bp = ax.boxplot(data, tick_labels=[LABELS[s].replace(' β', '\nβ') for s in STRATS],
                    patch_artist=True)
    for p, s in zip(bp['boxes'], STRATS):
        p.set_facecolor(COLS[s])
    ax.axhline(SYNC_TARGET_MS, color='green', ls='--')
    ax.set_yscale('log'); ax.set_ylabel('ms')
    ax.set_title(f'Max card KPI over the show ({N_MC} shows)')
    ax = axes[1]
    data = [mc[s]['max_rel'] for s in STRATS]
    bp = ax.boxplot(data, tick_labels=[LABELS[s].replace(' β', '\nβ') for s in STRATS],
                    patch_artist=True)
    for p, s in zip(bp['boxes'], STRATS):
        p.set_facecolor(COLS[s])
    ax.axhline(SYNC_TARGET_MS, color='green', ls='--')
    ax.set_yscale('log'); ax.set_ylabel('ms')
    ax.set_title('Max relative error over the show')
    ax = axes[2]
    for s in ('peer_05', 'master_05'):
        ax.plot(DT_SWEEP, [100 * sweep[d][s]['frac_abs_ok'].mean() for d in DT_SWEEP], 'o-',
                color=COLS[s], label=f'{LABELS[s]}: time KPI < 10 ms')
        ax.plot(DT_SWEEP, [100 * sweep[d][s]['frac_rel_ok'].mean() for d in DT_SWEEP], 's--',
                color=COLS[s], label=f'{LABELS[s]}: time relative < 10 ms')
    ax.set_xlabel('PTP interval (s)'); ax.set_ylabel('% of show time (mean over shows)')
    ax.set_ylim(0, 105)
    ax.set_title('PTP interval sweep (same drift rate)')
    ax.legend(fontsize=7)
    fig.tight_layout()
    path = os.path.join(out_dir, 'monte_carlo.png')
    fig.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def gif_indices(n, step):
    return [min(i * step, n - 1) for i in range(math.ceil((n - 1) / step) + 1)]


MAG = 50.0   # error-vector magnification in the animation


def save_animation(res, out_dir, step=4):
    n = len(res['no_sync']['t'])
    idx = gif_indices(n, step)
    fig, axes = plt.subplots(1, 2, figsize=(13, 6.2))
    cmap = plt.get_cmap('tab10')
    arts = []
    for ax, s in zip(axes, ['no_sync', 'master_05']):
        ax.set_xlim(-22, 22); ax.set_ylim(-22, 22); ax.set_aspect('equal')
        ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
        ax.set_title(f'{LABELS[s]} (arrows: error x{MAG:.0f})')
        r0 = res[s]
        si = ax.scatter(r0['ide'][0, :, 0], r0['ide'][0, :, 1], c='grey', s=30, alpha=0.5,
                        label='Ideal')
        sa = ax.scatter(r0['act'][0, :, 0], r0['act'][0, :, 1],
                        c=[cmap(k) for k in range(N_DRONES)], s=55, zorder=5, label='Actual')
        q = ax.quiver(np.zeros(N_DRONES), np.zeros(N_DRONES), np.zeros(N_DRONES),
                      np.zeros(N_DRONES), angles='xy', scale_units='xy', scale=1, color='k',
                      width=0.004)
        tx = ax.text(0.02, 0.95, '', transform=ax.transAxes, fontsize=9, family='monospace')
        ax.legend(loc='upper right', fontsize=8)
        arts.append((s, si, sa, q, tx))

    def update(fi):
        i = idx[fi]
        out = []
        for s, si, sa, q, tx in arts:
            r = res[s]
            si.set_offsets(r['ide'][i, :, :2])
            sa.set_offsets(r['act'][i, :, :2])
            q.set_offsets(r['ide'][i, :, :2])
            d = (r['act'][i, :, :2] - r['ide'][i, :, :2]) * MAG
            q.set_UVC(d[:, 0], d[:, 1])
            tx.set_text(f't = {r["t"][i]:5.2f} s  KPI = {r["err_abs"][i]:5.1f} ms\n'
                        f'max pos err = {100 * r["pos_err"][i].max():4.1f} cm')
            out += [si, sa, q, tx]
        return out

    anim = animation.FuncAnimation(fig, update, frames=len(idx), interval=100, blit=False)
    path = os.path.join(out_dir, 'sync_dance_animation.gif')
    anim.save(path, writer='pillow', fps=10, dpi=65)
    plt.close(fig)
    print(f'Saved: {path}  ({len(idx)} frames, step {step}, {n} samples)')


# ── Main ──────────────────────────────────────────────────────────────────────

def fmt(m):
    return (f'per-show max KPI: mean {np.mean(m["max_abs"]):.1f} ± {np.std(m["max_abs"]):.1f} ms, '
            f'worst {np.max(m["max_abs"]):.2f} ms; '
            f'per-show max relative: mean {np.mean(m["max_rel"]):.1f} ± '
            f'{np.std(m["max_rel"]):.1f} ms, worst {np.max(m["max_rel"]):.2f} ms; '
            f'RMS offset {np.mean(m["rms_abs"]):.2f} ms, '
            f'time KPI<10ms {100 * np.mean(m["frac_abs_ok"]):.1f}%, '
            f'time relative<10ms {100 * np.mean(m["frac_rel_ok"]):.1f}%, '
            f'shows with KPI<10ms throughout {int((m["max_abs"] < SYNC_TARGET_MS).sum())}, '
            f'mean V {100 * np.mean(m["meanV"]):.2f}%, min V {100 * np.min(m["minV"]):.1f}%, '
            f'max pos err {100 * np.max(m["max_pos"]):.1f} cm; per-drone-normalised V '
            f'(card code): mean {100 * np.mean(m["meanV_pd"]):.2f}%, '
            f'min {100 * np.min(m["minV_pd"]):.1f}%')


if __name__ == '__main__':
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    print('=== S098 Swarm Synchronized Dance ===')
    t_wp, wp = build_choreography()
    splines = build_splines(t_wp, wp)
    td = np.arange(0, T_SHOW + 1e-9, DT_SIM)
    vmax = max(np.linalg.norm(splines[k](td, 1), axis=1).max() for k in range(N_DRONES))
    print(f'Choreography: max speed {vmax:.2f} m/s -> 10 ms of clock error = '
          f'{100 * vmax * 0.01:.1f} cm of position error at most')
    n_int = T_SHOW / DT_PTP
    print(f'Free drift after {T_SHOW:.0f} s: std {1e3 * SIGMA_DRIFT * math.sqrt(n_int):.1f} ms per '
          f'drone; swarm-mean std {1e3 * SIGMA_DRIFT * math.sqrt(n_int / N_DRONES):.1f} ms '
          f'(a peer consensus cannot remove the mean)')
    print(f'Fresh drift within one PTP interval: std {1e3 * SIGMA_DRIFT:.1f} ms per drone')
    # peer-PTP relative-mode dynamics [e, dhat]: e' = (1-b)(e + dhat) approx, |eig| = sqrt(1-b)
    for b in (BETA_STANDARD, BETA_AGGRESSIVE):
        g = N_DRONES / (N_DRONES - 1)
        A = np.array([[1 - b * g, 1 - b], [-b * g, 1 - b]])
        print(f'Corrected update, beta={b}: relative-mode eigenvalue modulus '
              f'{np.abs(np.linalg.eigvals(A)).max():.3f}')
        Ac = np.array([[1 + b * g, -(1 - b)], [-b * g, 1 - b]])
        print(f'Card literal update, beta={b}: eigenvalue modulus '
              f'{np.abs(np.linalg.eigvals(Ac)).max():.3f} (> 1 means divergence)')

    res = {s: run_simulation(splines, s) for s in STRATS}
    lit = run_simulation(splines, 'card_literal')
    print(f'\nSeed {RNG_SEED}:')
    for s in STRATS:
        sm = summary(res[s])
        print(f'  {LABELS[s]:18s} max KPI {sm["max_abs"]:.1f} ms, max relative '
              f'{sm["max_rel"]:.1f} ms, time KPI<10ms {100 * sm["frac_abs_ok"]:.1f}%, '
              f'mean V {100 * sm["meanV"]:.2f}%, min V {100 * sm["minV"]:.1f}%, '
              f'max pos err {100 * sm["max_pos"]:.1f} cm, final mean offset '
              f'{1e3 * res[s]["delta"][-1].mean():+.1f} ms')
    vd = vmax_dense(splines)
    print(f'  Invariant check: script speed bound (1 ms grid) {vd:.3f} m/s; per 0.05 s step the '
          f'commanded position may move at most v_max * (0.05 s + |clock step|)')
    for s in STRATS:
        r = res[s]
        print(f'    {LABELS[s]:18s} largest clock step {1e3 * r["max_clock_step"]:.2f} ms, largest '
              f'commanded-position step {100 * r["max_jump"]:.2f} cm (bound '
              f'{100 * vd * (DT_SIM + r["max_clock_step"]):.2f} cm), smallest local-clock '
              f'advance per step {1e3 * r["min_local_step"]:.2f} ms (of {1e3 * DT_SIM:.0f} ms)')
    print(f'  Card literal update: max KPI {lit["err_abs"].max():.3g} ms '
          f'(at 5 s {lit["err_abs"][int(5 / DT_SIM)]:.3g} ms)')

    print(f'\nMonte Carlo ({N_MC} shows, same drift and jitter draws for every strategy):')
    mc = {s: monte_carlo(splines, s) for s in STRATS}
    for s in STRATS:
        print(f'  {LABELS[s]:18s} ' + fmt(mc[s]))
    a, b = mc['peer_09']['max_rel'], mc['peer_05']['max_rel']
    print(f'  beta 0.9 vs 0.5 (peer): smaller max relative error in {(a < b).sum()}/{N_MC} shows')
    a, b = mc['master_05']['max_abs'], mc['peer_05']['max_abs']
    print(f'  master vs peer (beta 0.5): smaller max KPI in {(a < b).sum()}/{N_MC} shows')

    print('\nPTP interval sweep (100 shows each; drift rate unchanged):')
    sweep = {}
    for d in DT_SWEEP:
        sweep[d] = {s: monte_carlo(splines, s, dt_ptp=d, n=100) for s in ('peer_05', 'master_05')}
        print(f'  interval {d:4.2f} s: ' + '; '.join(
            f'{LABELS[s]} time KPI<10ms {100 * sweep[d][s]["frac_abs_ok"].mean():.1f}%, '
            f'relative<10ms {100 * sweep[d][s]["frac_rel_ok"].mean():.1f}%, '
            f'per-show max KPI mean {sweep[d][s]["max_abs"].mean():.1f} ms / worst '
            f'{sweep[d][s]["max_abs"].max():.2f} ms, shows always <10 ms '
            f'{int((sweep[d][s]["max_abs"] < SYNC_TARGET_MS).sum())}/100'
            for s in ('peer_05', 'master_05')))

    print()
    plot_sync_error(res, OUTPUT_DIR)
    plot_visual_fidelity(res, OUTPUT_DIR)
    plot_clock_offsets(res, OUTPUT_DIR)
    plot_choreography(splines, OUTPUT_DIR)
    plot_monte_carlo(mc, sweep, OUTPUT_DIR)
    save_animation(res, OUTPUT_DIR)
